//! 承载判定与碎裂的拍内编排(REQ-010 §4.6、§7.3;终审审计 P3 拆分)。
//!
//! 纯函数(承载最大流、切块、断键、拆分、爆炸逐格)都在 [`crate::cohesion`],
//! 本文件只留"采样—预算—名额—outbox 产出"的粘合:碎裂采样
//! ([`ShardSim::evaluate_fractures`])、承载接入([`ShardSim::evaluate_load_tick`])
//! 与其单格写入([`ShardSim::convert_load_cell`])。调度入口仍在 `sim.rs`
//! (`step` 与 `evaluate_collapse` 的开关分派);REQ-009 的逐格坍塌路径
//! (`evaluate_collapse` 本体)不在此,留在 `sim.rs`。

use super::*;

/// 复合体冲量流的定量追踪(G02,验收观测口;`#[cfg(test)]`,生产零开销):
/// 每个入选并求值的事件一条。字段齐全、不含句柄。
#[cfg(test)]
#[derive(Debug, Clone)]
pub(crate) struct CompoundFlowTrace {
    /// 事件(B 的刚体局部系;格心 = BlockPos + 0.5)。
    pub event: crate::compound_flow::CompoundEvent,
    /// 网络判定。
    pub outcome: crate::compound_flow::CompoundFlowOutcome,
    /// 动量分配(断开事件;扛住为默认值)。
    pub split: crate::compound_flow::MomentumSplit,
    /// 本体各块的初速度(世界系;与 `piece_requests` / `compound_requests`
    /// 里落下去的顺序一致:S_k 在前、R_j 在后)。
    pub piece_velocities: Vec<(Vec3, Vec3)>,
    /// 对方修正(世界系)及去向:`None` = 不存在(对方是地形或无动态对方);
    /// `Some((冲量, 作用点, EntityId, 去刚体))` 或
    /// `Some((冲量, 作用点, EntityId, 去分块请求))`。
    pub other_corrections: Vec<(Vec3, Vec3, types::EntityId, bool)>,
}

/// `compound_flow` 新路径的按实体事件累计(G02,§5.8.1):`contacts` = 计入
/// 接触点(局部系);`other_impulses` = 动态对方 → 计入点冲量和(o 一侧;
/// 同一对方跨多个流形累加);`other_points` = 该对方 o 一侧接触点世界位置
/// 按冲量加权的累计量(修正施加点);`other_cell_impulses` = o 一侧接触冲量
/// 按 **o 的原形状格 offset** 累计(采样时 o 在场、collider_to_cell 有效,
/// 键 = o 的 offset;供"对方已拆分"时定位含接触格的分块请求,§5.8.2——
/// 裁定 G02-2①:处理期 o 的 collider 已随 remove_body 删除,不能事后反查);
/// `other_props` = o 的质量属性快照(采样时的 rapier 质量/世界逆惯量/世界
/// 质心;o 本拍被拆、刚体已删后 m_eff 仍可按采样时口径拆账)。`linvel` /
/// `angvel` = B 本拍步进后的刚体速度(局部系)。
struct CompoundFlowSample {
    contacts: Vec<crate::compound_flow::ContactPoint>,
    other_impulses: BTreeMap<types::EntityId, f32>,
    other_points: BTreeMap<types::EntityId, (Vec3, f32)>,
    other_cell_impulses: BTreeMap<types::EntityId, BTreeMap<BlockPos, f32>>,
    other_props: BTreeMap<types::EntityId, crate::impact_flow::ImpactBodyProps>,
    linvel: Vec3,
    angvel: Vec3,
}

impl ShardSim {
    /// 处理 `max_fractures_per_tick` 个复合体,按 `J_c = J/|接触格|` 降序、同值
    /// `EntityId` 升序,其余本拍不断。处理(§7.3 第 3–5 条):`fracture_bonds`
    /// 断键 → `fracture_components` 拆分 → 多于 1 个分量时原实体本拍删除,
    /// `fracture_pieces` 产出分块请求(`index` = 分量序号)经调度器 N+1 生成
    /// (复用单元 05 路径);碎了即删除,凝固段自然不再判整块凝固。遥测
    /// (§7.3 第 8 条):`bonds_broken`、`fracture_components`(仅拆分计)。
    pub(super) fn evaluate_fractures(&mut self) {
        if !self.load_enabled() {
            return;
        }
        // §5.8 分派(G02):`compound_flow` 生效时新路径接管"断哪里与动量
        // 分配",§7.1 的局部断键与压向豁免不再执行;关闭时原 §7 路径逐位
        // 不变(AC-G4)。
        if self.cfg.cohesion.compound_flow {
            self.evaluate_fractures_compound_flow();
            return;
        }
        self.evaluate_fractures_legacy();
    }

    /// 原 §7 路径(单元 06;`compound_flow = false` 时唯一路径,AC-G4 逐位
    /// 不变;`compound_flow = true` 时不执行,两套规则不叠加)。
    fn evaluate_fractures_legacy(&mut self) {
        // 本体资格表(§7.3 第 2 条):醒着的**复合体**;先于接触映射建表(终审
        // 审计 P3):绝大多数拍没有合格复合体,这里直接早退,免付全量建表。
        // 资格表携带本步前线速度 + 角速度 + 世界质心(勘误 2,§5.7.9):
        // 接近速度按接触点相对速度算。
        let eligible: BTreeMap<EntityId, (Vec3, PreImpactSpin)> = self
            .pre_impulse_vels
            .iter()
            .filter(|(id, _)| {
                self.entities
                    .get(id)
                    .is_some_and(|rec| rec.desc.shape().is_some())
                    && self
                        .phys
                        .bodies
                        .get(self.entities[id].handle)
                        .is_some_and(|b| b.is_dynamic() && !b.is_sleeping())
            })
            .filter_map(|(id, v)| {
                let spin = self.pre_impulse_spin.get(id).copied()?;
                Some((*id, (*v, spin)))
            })
            .collect();
        if eligible.is_empty() {
            return;
        }
        // 对方速度表(勘误 2"对方"条,F02a-2 拆出):**任何醒着的动态实体**——
        // 单格实体(玩家放出的方块等,没有 `shape`)也算(只认复合体会让单格
        // 投射物击中醒着的复合体时对方速度按 0 计,12 m/s 单格方块实测不断)。
        // 不参与上方早退:场上只有醒着的单格实体、没有合格复合体时,本拍不判
        // 碎裂,也不必为对方速度付全量建表。
        let other_vels: BTreeMap<EntityId, (Vec3, PreImpactSpin)> = self
            .pre_impulse_vels
            .iter()
            .filter(|(id, _)| {
                self.entities.contains_key(id)
                    && self
                        .phys
                        .bodies
                        .get(self.entities[id].handle)
                        .is_some_and(|b| b.is_dynamic() && !b.is_sleeping())
            })
            .filter_map(|(id, v)| {
                let spin = self.pre_impulse_spin.get(id).copied()?;
                Some((*id, (*v, spin)))
            })
            .collect();
        // collider → 实体(复合体 = 全部格 collider)与复合体 collider → 形状格
        // 序号(局部平移 = 偏移,按偏移匹配,不依赖 colliders() 的插入顺序)。
        // **全部实体都登记**(单格 = 代表 collider;勘误 2"对方"条:对方的
        // collider → 实体映射须覆盖单格实体,否则对方速度查不到)。
        let mut collider_to_entity: BTreeMap<(u32, u32), EntityId> = BTreeMap::new();
        let mut collider_to_cell: BTreeMap<(u32, u32), usize> = BTreeMap::new();
        for (id, rec) in &self.entities {
            let idx = rec.collider.into_raw_parts();
            collider_to_entity.insert((idx.0, idx.1), *id);
            let Some(shape) = rec.desc.shape() else {
                continue;
            };
            let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
            let by_offset: BTreeMap<[i32; 3], usize> = shape
                .cells()
                .iter()
                .enumerate()
                .map(|(i, c)| (c.offset, i))
                .collect();
            for ch in body.colliders() {
                let idx = ch.into_raw_parts();
                collider_to_entity.insert((idx.0, idx.1), *id);
                if let Some(col) = self.phys.colliders.get(*ch) {
                    let t = col
                        .position_wrt_parent()
                        .map_or(Vec3::ZERO, |p| p.translation);
                    let off = [t.x.round() as i32, t.y.round() as i32, t.z.round() as i32];
                    if let Some(&i) = by_offset.get(&off) {
                        collider_to_cell.insert((idx.0, idx.1), i);
                    }
                }
            }
        }
        struct FractureSample {
            total: f32,
            cells: BTreeMap<usize, Vec3>,
        }
        let mut samples: BTreeMap<EntityId, FractureSample> = BTreeMap::new();
        for pair in self.phys.narrow_phase.contact_pairs() {
            let key1 = pair.collider1.into_raw_parts();
            let key2 = pair.collider2.into_raw_parts();
            // 碰撞体世界位姿(勘误 2:接触点世界坐标 = 位姿 × local_p;
            // 簇的 `local_p` 已折算到碰撞体坐标系,无需子形状位姿)。
            // **D4 采样只需要本体一侧的位姿**(地形一侧速度为 0,用不到;
            // 对方是动态实体时用本体侧接触点加对方的本步前快照,已足够)。
            // 同拍 `evaluate_impacts` 撞脱地形格会**重建地形 collider**,
            // 之后本采样遍历的接触对里地形侧句柄已失效——**只在本体侧
            // collider 缺失时才跳过该侧,不整对跳过**(整对跳过会把该对
            // 的冲量整个丢掉:角先着地第 76 拍 32.9 的断键冲量就是这样
            // 丢的,AC-C6 翻红)。
            let pos1 = self
                .phys
                .colliders
                .get(pair.collider1)
                .map(|c| *c.position());
            let pos2 = self
                .phys
                .colliders
                .get(pair.collider2)
                .map(|c| *c.position());
            // 对方实体(醒着的动态实体都算,单格也算——勘误 2"对方"条):
            // 本步前快照(线速度 + 角速度/质心,接触点速度按接触点算);对方
            // 是地形、睡眠体或本步前没有快照(本拍新出生、或本步开始后入表)
            // 时返回 None,速度按 **0** 处理(§5.7.9 勘误 2"对方"条口径:
            // 地形、睡眠体或没有快照时按 0)。
            let other_vel = |side1: bool| -> Option<(Vec3, PreImpactSpin)> {
                let oid = collider_to_entity.get(if side1 { &key2 } else { &key1 })?;
                other_vels.get(oid).copied()
            };
            // 读解算器实际使用的流形(REQ-010 §5.7.10 末条、AC-F17;同
            // `evaluate_impacts`):多流形对被 contact clustering 合并解算,
            // 冲量只回写在簇上,plain 流形恒 0。簇的 `subshape_pos = None`、
            // 接触点已折算到碰撞体坐标系——本采样只按 collider(格)聚合,
            // 不用子形状位姿,不受影响。
            for m in pair.solver_manifolds() {
                // 法向为 world 系、以碰撞体 1 → 2 为正:对 1 侧"指向本体内部"
                // = −normal,对 2 侧 = +normal(与撞击采样"撞击方向"互为反向量)。
                // 逐侧取本体侧合格碰撞体(单格 = 代表 collider;复合体 = 全部
                // 格 collider),同一流形对两侧各判一次。
                for (key, dir_into, side1) in
                    [(key1, -m.data.normal, true), (key2, m.data.normal, false)]
                {
                    let Some(&id) = collider_to_entity.get(&key) else {
                        continue;
                    };
                    let Some(&cell) = collider_to_cell.get(&key) else {
                        continue;
                    };
                    let Some((pre_vel, spin)) = eligible.get(&id) else {
                        continue;
                    };
                    // 本体侧碰撞体位姿(勘误 2:接触点世界坐标 = 位姿 × local_p;
                    // 簇的 `local_p` 已折算到碰撞体坐标系,无需子形状位姿)。
                    // **按本体在哪一侧取位姿**——一律用 pos1 是错的(支点错位
                    // 会让 r 与接触点速度全错)。本体侧 collider 缺失(同拍撞击
                    // 重建后句柄失效的是地形侧;碎裂删实体的是本体侧)→ 只跳过
                    // 本侧,另一侧照常判。
                    let Some(pos_self) = (if side1 { pos1 } else { pos2 }) else {
                        continue;
                    };
                    // 对方 = 动态实体时按其本步前快照算接触点速度;对方没有
                    // 快照(本拍新出生、或本步开始后入表)时按 **0** 处理——
                    // 对方的速度未知,不能默认它是静止的把相对速度当成本体
                    // 速度(勘误 2 的保守侧)。
                    let other_snap = other_vel(side1);
                    // 接近速度(§5.7.9 勘误 2,F03b 口径):**质心口径与接触
                    // 点口径取大**——
                    // - 质心口径(旧口径):本体本步前质心速度减对方质心速度,
                    //   沿接触法向**朝对方**的分量;对方是地形或无快照时按 0。
                    // - 接触点口径:流形内各点的接触点相对速度(本体侧
                    //   v + ω×(p − com) 减对方侧同点速度)的同向分量。
                    // 两者最大值 ≥ 门槛则整流形计入。质心口径保住"撞击分几拍
                    // 到账"的后几拍;接触点口径覆盖旋转体;静止对挤两口径都
                    // < 1 仍排除;非旋转体两项相等,与旧口径逐位一致。
                    let com_speed = match other_snap {
                        Some((ovel, _)) => (ovel - *pre_vel).dot(dir_into),
                        None => (-*pre_vel).dot(dir_into),
                    };
                    let max_approach = m
                        .points
                        .iter()
                        .map(|pt| {
                            // 本体在 1 侧用 local_p1、2 侧用 local_p2(接触点
                            // 在各自碰撞体的局部系,位姿配对取点)。
                            let p_self = if side1 {
                                pos_self * pt.local_p1
                            } else {
                                pos_self * pt.local_p2
                            };
                            let r_self = p_self - spin.world_com;
                            let v_self = *pre_vel + spin.angvel.cross(r_self);
                            let v_other = match other_snap {
                                Some((ovel, ospin)) => {
                                    // 对方实体侧接触点:**用本体侧接触点近似**
                                    // (同一物理点,两侧坐标差 = 穿透深度量级,
                                    // 远小于 r;对方侧位姿本采样不需要,地形侧
                                    // 句柄可能已失效)。
                                    let r_other = p_self - ospin.world_com;
                                    ovel + ospin.angvel.cross(r_other)
                                }
                                // 对方是地形或无快照:速度按 0。
                                None => Vec3::ZERO,
                            };
                            (v_other - v_self).dot(dir_into)
                        })
                        .fold(com_speed, f32::max);
                    if max_approach < crate::destruction::IMPACT_MIN_APPROACH_SPEED {
                        continue;
                    }
                    let s = samples.entry(id).or_insert(FractureSample {
                        total: 0.0,
                        cells: BTreeMap::new(),
                    });
                    for pt in &m.points {
                        if pt.data.impulse <= 0.0 {
                            continue;
                        }
                        s.total += pt.data.impulse;
                        *s.cells.entry(cell).or_insert(Vec3::ZERO) += dir_into;
                    }
                }
            }
        }
        // 上限与顺序(§7.3 第 6 条):J_c 降序、同值 EntityId 升序。
        // 候选 = (J_c, 实体, 冲量合计, 各接触格及受力方向)。
        type FractureCandidate = (f32, EntityId, f32, Vec<(usize, Vec3)>);
        let mut candidates: Vec<FractureCandidate> = samples
            .into_iter()
            .filter(|(_, s)| !s.cells.is_empty())
            .map(|(id, s)| {
                let j_c = s.total / s.cells.len() as f32;
                let contacts: Vec<(usize, Vec3)> = s
                    .cells
                    .into_iter()
                    .map(|(i, d)| (i, d.normalize_or_zero()))
                    .collect();
                (j_c, id, s.total, contacts)
            })
            .collect();
        candidates.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        for (_, id, total, contacts) in candidates
            .into_iter()
            .take(self.cfg.cohesion.max_fractures_per_tick)
        {
            let prepared = self.entities.get(&id).and_then(|rec| {
                let shape = rec.desc.shape()?;
                Some((
                    rec.handle,
                    shape.clone(),
                    rec.desc.clone(),
                    self.read_state(rec),
                ))
            });
            let Some((handle, shape, desc, state)) = prepared else {
                continue;
            };
            let broken = crate::cohesion::fracture_bonds(
                &shape,
                state.rotation,
                total,
                &contacts,
                self.registry.as_ref(),
            );
            if broken.is_empty() {
                continue;
            }
            let components = crate::cohesion::fracture_components(&shape, &broken);
            self.outbox.cohesion_stats.bonds_broken += broken.len() as u64;
            if components.len() <= 1 {
                continue;
            }
            // 拆分(§7.3 第 5 条):原实体本拍删除,分量 N+1 出现(index = 分量序号)。
            let pieces = crate::cohesion::fracture_pieces(&state, &desc, &components);
            for (index, (piece_state, piece_desc)) in pieces.into_iter().enumerate() {
                self.outbox
                    .piece_requests
                    .push(crate::cohesion::PieceRequest {
                        parent: id,
                        index: index as u32,
                        state: piece_state,
                        desc: piece_desc,
                    });
            }
            self.entities.remove(&id).expect("存在");
            self.phys.remove_body(handle);
            self.outbox.despawned.push(id);
            self.outbox.cohesion_stats.fracture_components += components.len() as u64;
        }
    }

    /// 复合体冲量流新路径(G02,§5.8;`compound_flow = true` 时由
    /// [`Self::evaluate_fractures`] 分派):
    ///
    /// - **采样**(沿用原路径的流形筛选:勘误 2 接近速度、门槛 1 m/s、本体
    ///   侧位姿、地形侧句柄失效只跳本侧):对每个计入点 `j_k > 0` 记录所在格
    ///   (`collider_to_cell` → 形状格序号 → offset 即局部 `BlockPos`)、局部
    ///   作用点(`R⁻¹(p_w − t) + 0.5`,c65a8f6 坐标对应;格心 = BlockPos +
    ///   0.5)、局部法向(`R⁻¹·dir_into`,指向 B 内部)、动态对方;`linvel` /
    ///   `angvel` = B 本拍步进后的刚体速度转局部系。debug 断言守卫:每个点
    ///   落在所在格盒 `[offset, offset+1]³` 外扩 0.1 内(坐标错位 0.5 或漏乘
    ///   旋转立刻 panic;AC-G3 的坐标守卫测试在测试层独立断言)。
    /// - **事件向量与 §5.7 修正扣除**(§5.8.1):`J = Σ j_k·d_k`;B 是 §5.7
    ///   撞击者且本拍已被修正时 `correction = −R⁻¹·c_w`(c_w 沿前进方向、与
    ///   J 反向,净冲量 = J + c_w;表在 `apply_impact_flow_events` 写入、
    ///   拍内有效)。
    /// - **名额与顺序**(§5.8.3):按 s = |J| 降序、同值 EntityId 升序取前
    ///   `max_fractures_per_tick` 个;再按 EntityId 升序依次处理,后一个事件
    ///   读到前一个事件处理后的状态(被拆实体不在场)。
    /// - **断开与成形**(§5.8.2):`evaluate_compound_flow` →
    ///   `distribute_momentum`;分块 = S_k ∪ R_j(offset → 形状格序号,
    ///   `fracture_pieces` 生成平移/旋转/描述),速度用 `piece_velocity` 在
    ///   局部系算出乘 R 覆盖(R_j 取 r_refunds/q_j,S_k 取 s_impulses/p_k,
    ///   单格走单格分支);原实体本拍删除,分块请求按 (parent, index) 走
    ///   现有 N+1 路径。扛住或预筛:什么都不做。
    /// - **对方修正**(§5.8.2):动态对方按接触冲量份额拆账,修正
    ///   `R·other_corrections[o]` 在 o 一侧接触点质心 `apply_impulse_at_point`
    ///   并唤醒;o 本拍已被拆分(EntityId 更小、先处理并已删除)时加到其
    ///   含接触格的分块请求(线速度 + P/M;多格块角速度 + I⁻¹(r × P),
    ///   世界系块的旋转;单格不加)。地形份额无修正。
    /// - **遥测**(§5.8.4):`compound_flow_*` 5 个计数;`bonds_broken` =
    ///   cut_bonds、`fracture_components` = 分块数照常计。
    fn evaluate_fractures_compound_flow(&mut self) {
        // 采样前置表(与原路径同构):资格表(醒着复合体)、对方速度表、
        // collider → 实体 / 形状格序号。
        let eligible: BTreeMap<EntityId, (Vec3, PreImpactSpin)> = self
            .pre_impulse_vels
            .iter()
            .filter(|(id, _)| {
                self.entities
                    .get(id)
                    .is_some_and(|rec| rec.desc.shape().is_some())
                    && self
                        .phys
                        .bodies
                        .get(self.entities[id].handle)
                        .is_some_and(|b| b.is_dynamic() && !b.is_sleeping())
            })
            .filter_map(|(id, v)| {
                let spin = self.pre_impulse_spin.get(id).copied()?;
                Some((*id, (*v, spin)))
            })
            .collect();
        if eligible.is_empty() {
            return;
        }
        let other_vels: BTreeMap<EntityId, (Vec3, PreImpactSpin)> = self
            .pre_impulse_vels
            .iter()
            .filter(|(id, _)| {
                self.entities.contains_key(id)
                    && self
                        .phys
                        .bodies
                        .get(self.entities[id].handle)
                        .is_some_and(|b| b.is_dynamic() && !b.is_sleeping())
            })
            .filter_map(|(id, v)| {
                let spin = self.pre_impulse_spin.get(id).copied()?;
                Some((*id, (*v, spin)))
            })
            .collect();
        let mut collider_to_entity: BTreeMap<(u32, u32), EntityId> = BTreeMap::new();
        let mut collider_to_cell: BTreeMap<(u32, u32), usize> = BTreeMap::new();
        for (id, rec) in &self.entities {
            let idx = rec.collider.into_raw_parts();
            collider_to_entity.insert((idx.0, idx.1), *id);
            let Some(shape) = rec.desc.shape() else {
                continue;
            };
            let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
            let by_offset: BTreeMap<[i32; 3], usize> = shape
                .cells()
                .iter()
                .enumerate()
                .map(|(i, c)| (c.offset, i))
                .collect();
            for ch in body.colliders() {
                let idx = ch.into_raw_parts();
                collider_to_entity.insert((idx.0, idx.1), *id);
                if let Some(col) = self.phys.colliders.get(*ch) {
                    let t = col
                        .position_wrt_parent()
                        .map_or(Vec3::ZERO, |p| p.translation);
                    let off = [t.x.round() as i32, t.y.round() as i32, t.z.round() as i32];
                    if let Some(&i) = by_offset.get(&off) {
                        collider_to_cell.insert((idx.0, idx.1), i);
                    }
                }
            }
        }
        // 采样(局部系事件):按实体累计接触点与对方冲量。
        let mut samples: BTreeMap<EntityId, CompoundFlowSample> = BTreeMap::new();
        for pair in self.phys.narrow_phase.contact_pairs() {
            let key1 = pair.collider1.into_raw_parts();
            let key2 = pair.collider2.into_raw_parts();
            let pos1 = self
                .phys
                .colliders
                .get(pair.collider1)
                .map(|c| *c.position());
            let pos2 = self
                .phys
                .colliders
                .get(pair.collider2)
                .map(|c| *c.position());
            let other_vel = |side1: bool| -> Option<(Vec3, PreImpactSpin)> {
                let oid = collider_to_entity.get(if side1 { &key2 } else { &key1 })?;
                other_vels.get(oid).copied()
            };
            for m in pair.solver_manifolds() {
                for (key, dir_into, side1) in
                    [(key1, -m.data.normal, true), (key2, m.data.normal, false)]
                {
                    let Some(&id) = collider_to_entity.get(&key) else {
                        continue;
                    };
                    let Some(&cell_i) = collider_to_cell.get(&key) else {
                        continue;
                    };
                    let Some((_pre_vel, _spin)) = eligible.get(&id) else {
                        continue;
                    };
                    let Some(pos_self) = (if side1 { pos1 } else { pos2 }) else {
                        continue;
                    };
                    let other_snap = other_vel(side1);
                    // 接近速度(勘误 2,与原路径同式)。
                    let com_speed = match other_snap {
                        Some((ovel, _)) => (ovel - self.pre_impulse_vels[&id]).dot(dir_into),
                        None => (-self.pre_impulse_vels[&id]).dot(dir_into),
                    };
                    let max_approach = m
                        .points
                        .iter()
                        .map(|pt| {
                            let p_self = if side1 {
                                pos_self * pt.local_p1
                            } else {
                                pos_self * pt.local_p2
                            };
                            let r_self = p_self - self.pre_impulse_spin[&id].world_com;
                            let v_self = self.pre_impulse_vels[&id]
                                + self.pre_impulse_spin[&id].angvel.cross(r_self);
                            let v_other = match other_snap {
                                Some((ovel, ospin)) => {
                                    let r_other = p_self - ospin.world_com;
                                    ovel + ospin.angvel.cross(r_other)
                                }
                                None => Vec3::ZERO,
                            };
                            (v_other - v_self).dot(dir_into)
                        })
                        .fold(com_speed, f32::max);
                    if max_approach < crate::destruction::IMPACT_MIN_APPROACH_SPEED {
                        continue;
                    }
                    // 本体刚体位姿与步进后速度(局部系事件的换算基准)。B 步进
                    // 后速度已含本拍 §5.7 修正(§5.8.1)。
                    let rec = self.entities.get(&id).expect("资格表实体在场");
                    let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
                    let t_w = conv::rapier_to_vec3(body.translation());
                    let r_w = conv::rapier_to_quat(body.rotation());
                    let r_inv = r_w.inverse();
                    let linvel_w = conv::rapier_to_vec3(body.linvel());
                    let angvel_w = conv::rapier_to_vec3(body.angvel());
                    let linvel = r_inv * linvel_w;
                    let angvel = r_inv * angvel_w;
                    // 形状格 offset(collider_to_cell 的格序号 → offset;局部
                    // BlockPos = offset,c65a8f6 坐标对应)。
                    let shape = rec.desc.shape().expect("资格表只有复合体");
                    let offset = shape.cells()[cell_i].offset;
                    let cell_pos = BlockPos::new(offset[0], offset[1], offset[2]);
                    let s = samples.entry(id).or_insert(CompoundFlowSample {
                        contacts: Vec::new(),
                        other_impulses: BTreeMap::new(),
                        other_points: BTreeMap::new(),
                        other_cell_impulses: BTreeMap::new(),
                        other_props: BTreeMap::new(),
                        linvel,
                        angvel,
                    });
                    let oid = collider_to_entity
                        .get(if side1 { &key2 } else { &key1 })
                        .copied();
                    let oid = oid.filter(|o| {
                        other_vels.contains_key(o)
                            && o != &id
                            && self.entities.get(o).is_some_and(|r2| {
                                self.phys
                                    .bodies
                                    .get(r2.handle)
                                    .is_some_and(|b2| b2.is_dynamic())
                            })
                    });
                    if let Some(o) = oid {
                        // o 的质量属性快照(采样时;处理循环里 o 可能已被先
                        // 处理的事件拆掉,届时 m_eff 仍按此拆账,§5.8.2)。
                        let rec_o = self.entities.get(&o).expect("oid 过滤已验证在场");
                        let body_o = self
                            .phys
                            .bodies
                            .get(rec_o.handle)
                            .expect("对方 handle 有效");
                        let inv = body_o.mass_properties().effective_world_inv_inertia;
                        s.other_props
                            .entry(o)
                            .or_insert(crate::impact_flow::ImpactBodyProps {
                                mass: body_o.mass(),
                                inv_inertia: [
                                    [inv.m11, inv.m12, inv.m13],
                                    [inv.m12, inv.m22, inv.m23],
                                    [inv.m13, inv.m23, inv.m33],
                                ],
                                center_of_mass: conv::rapier_to_vec3(
                                    body_o.mass_properties().world_com,
                                ),
                            });
                    }
                    // o 一侧接触冲量按 **o 的原形状格 offset** 累计(G02-2①:
                    // 采样时 o 在场,collider_to_cell 的键有效;处理期 o 被拆
                    // 后 collider 已删,不能事后反查)。
                    let other_key = if side1 { key2 } else { key1 };
                    let other_cell = if oid.is_some() {
                        collider_to_cell.get(&other_key).copied()
                    } else {
                        None
                    };
                    for pt in &m.points {
                        if pt.data.impulse <= 0.0 {
                            continue;
                        }
                        if let (Some(o), Some(cell_i_o)) = (oid, other_cell) {
                            let off = {
                                let rec_o = self.entities.get(&o).expect("oid 过滤已验证在场");
                                let shape_o = rec_o.desc.shape().expect("对方也可能是复合体或单格");
                                shape_o.cells()[cell_i_o].offset
                            };
                            *s.other_cell_impulses
                                .entry(o)
                                .or_default()
                                .entry(BlockPos::new(off[0], off[1], off[2]))
                                .or_insert(0.0) += pt.data.impulse;
                        }
                        // 世界作用点与局部换算(坐标对应:c65a8f6)。
                        let p_w = if side1 {
                            pos_self * pt.local_p1
                        } else {
                            pos_self * pt.local_p2
                        };
                        let pw = conv::rapier_to_vec3(p_w);
                        let local_p = r_inv * (pw - t_w) + Vec3::new(0.5, 0.5, 0.5);
                        let local_n = r_inv * dir_into;
                        // 结构性守卫(debug):点必须落在所在格盒 [offset,
                        // offset+1]³ 外扩 0.1 内——漏加 0.5(错位半格)或漏乘
                        // R⁻¹(任意旋转体)立刻 panic(§5.8.1 坐标对应;AC-G3
                        // 的坐标守卫测试在测试层独立断言,不只靠这里)。
                        #[cfg(debug_assertions)]
                        {
                            let lo = Vec3::new(
                                offset[0] as f32 - 0.1,
                                offset[1] as f32 - 0.1,
                                offset[2] as f32 - 0.1,
                            );
                            let hi = Vec3::new(
                                offset[0] as f32 + 1.1,
                                offset[1] as f32 + 1.1,
                                offset[2] as f32 + 1.1,
                            );
                            assert!(
                                local_p.x >= lo.x
                                    && local_p.y >= lo.y
                                    && local_p.z >= lo.z
                                    && local_p.x <= hi.x
                                    && local_p.y <= hi.y
                                    && local_p.z <= hi.z,
                                "复合体冲量流采样坐标越界:格 {cell_pos:?} 盒 \
                                 [{lo:?}, {hi:?}],局部点 {local_p:?}(实体 {id:?})——\
                                 坐标对应破坏(offset 未加 0.5 或漏乘 R⁻¹)"
                            );
                        }
                        s.contacts.push(crate::compound_flow::ContactPoint {
                            cell: cell_pos,
                            impulse: pt.data.impulse,
                            point: local_p,
                            normal: local_n.normalize_or_zero(),
                        });
                        if let Some(o) = oid {
                            *s.other_impulses.entry(o).or_insert(0.0) += pt.data.impulse;
                            let e = s.other_points.entry(o).or_insert((Vec3::ZERO, 0.0));
                            // 对方一侧接触点质心:同一物理点,用本体侧世界点
                            // 近似(与原路径 F03b 口径一致)。
                            e.0 += pw * pt.data.impulse;
                            e.1 += pt.data.impulse;
                        }
                    }
                }
            }
        }
        if samples.is_empty() {
            return;
        }
        // 事件成形(§5.8.1):J、correction(局部系)、接触格由 contacts 携带。
        type CompoundCandidate = (
            f32,
            EntityId,
            crate::compound_flow::CompoundEvent,
            BTreeMap<types::EntityId, (Vec3, f32)>,
            BTreeMap<types::EntityId, f32>,
            BTreeMap<types::EntityId, BTreeMap<BlockPos, f32>>,
            BTreeMap<types::EntityId, crate::impact_flow::ImpactBodyProps>,
        );
        let mut candidates: Vec<CompoundCandidate> = samples
            .into_iter()
            .filter(|(_, s)| !s.contacts.is_empty())
            .map(|(id, s)| {
                let mut ev = crate::compound_flow::CompoundEvent {
                    cells: {
                        let rec = self.entities.get(&id).expect("采样实体在场");
                        rec.desc
                            .shape()
                            .expect("资格表只有复合体")
                            .cells()
                            .iter()
                            .map(|c| {
                                (
                                    BlockPos::new(c.offset[0], c.offset[1], c.offset[2]),
                                    c.block,
                                )
                            })
                            .collect()
                    },
                    contacts: s.contacts,
                    linvel: s.linvel,
                    angvel: s.angvel,
                    correction: Vec3::ZERO,
                };
                // §5.7 修正扣除(c65a8f6 符号条):correction 按"沿 J 退回的
                // 部分"定义 = −R⁻¹·c_w(施加给撞击者的 c_w 沿前进方向,与
                // B 的 J 反向,净冲量 = J + c_w)。
                if let Some(&c_w) = self.impact_corrections_this_tick.get(&id) {
                    let rec = self.entities.get(&id).expect("采样实体在场");
                    let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
                    let r_inv = conv::rapier_to_quat(body.rotation()).inverse();
                    ev.correction = -(r_inv * c_w);
                }
                let s_total = ev.total_impulse();
                (
                    s_total,
                    id,
                    ev,
                    s.other_points.clone(),
                    s.other_impulses.clone(),
                    s.other_cell_impulses.clone(),
                    s.other_props.clone(),
                )
            })
            .collect();
        // 名额与顺序(§5.8.3):s 降序、同值 EntityId 升序选取;再按 EntityId
        // 升序依次处理——后一个事件读到前一个事件处理后的状态(被拆实体
        // 不在场;对方已拆分的修正加到其分块请求,见处理循环)。
        candidates.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        candidates.truncate(self.cfg.cohesion.max_fractures_per_tick);
        candidates.sort_by_key(|(_, id, ..)| *id);
        // 对方已拆分映射(G02-2①):被拆实体 → "原 offset → 该格所在分块
        // 请求在 outbox.piece_requests 中的下标"。S_k、R_j 的格都是原形状
        // offset,拆分时直接可得;后续事件对它的修正按 o 一侧累计冲量最大
        // 的接触格(同值 BlockPos 升序)经此表定位。
        let mut split_piece_index: BTreeMap<EntityId, BTreeMap<BlockPos, usize>> = BTreeMap::new();
        #[cfg(test)]
        let mut traces: Vec<CompoundFlowTrace> = Vec::new();
        for (_, id, ev, other_points, other_impulses, other_cells, other_props) in candidates {
            // 遥测(§5.8.4):events **含预筛命中**(只要成形并进了处理循环)。
            self.outbox.cohesion_stats.compound_flow_events += 1;
            let outcome = crate::compound_flow::evaluate_compound_flow(self.registry.as_ref(), &ev);
            if outcome.prefiltered {
                // 预筛跳过:不建网、无 `C`,不记 FlowEventInfo(§3.5.1)。
                self.outbox.cohesion_stats.compound_flow_prefiltered += 1;
                #[cfg(test)]
                traces.push(CompoundFlowTrace {
                    event: ev,
                    outcome,
                    split: Default::default(),
                    piece_velocities: Vec::new(),
                    other_corrections: Vec::new(),
                });
                continue;
            }
            // 事件摘要(REQ-011 §3.5.1;W03):`s`/`C` 由判定的同一份量化值
            // 换算;`broke` 与判定同比较;position = 该刚体本拍步进后的平移
            // (复合体原点;此时本体必在场——同实体至多一候选,先前候选只
            // 拆它们自己的本体)。
            let info = {
                let rec = self.entities.get(&id).expect("本体在场");
                let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
                crate::sim::FlowEventInfo {
                    kind: crate::sim::FlowKind::Compound,
                    // 量化单位与 compound_flow 内部常量同值 1000(impact_flow
                    // 的 pub(crate) 镜像;§3.5.1"同一份数据")。
                    supply: outcome.supply_q as f32 / crate::impact_flow::QUANT as f32,
                    cut: outcome.cut_q as f32 / crate::impact_flow::QUANT as f32,
                    broke: !outcome.detached.is_empty(),
                    detached_cells: outcome.detached.len() as u64,
                    position: conv::rapier_to_vec3(body.translation()),
                }
            };
            self.outbox.flow_event =
                crate::sim::FlowEventInfo::pick(self.outbox.flow_event.take(), Some(info));
            if outcome.detached.is_empty() {
                // 扛住:无断开、无修正(§5.8.2:修正只随断开)。
                #[cfg(test)]
                traces.push(CompoundFlowTrace {
                    event: ev,
                    outcome,
                    split: Default::default(),
                    piece_velocities: Vec::new(),
                    other_corrections: Vec::new(),
                });
                continue;
            }
            // 动量分配(§5.8.2):动态对方的 m_eff(§5.7.2 公式,o 的 rapier
            // 质量/世界系有效逆惯量/世界质心,o 一侧接触点质心沿 R·d̂)与
            // 冲量份额(o 的计入点冲量和 / 全部计入点冲量和)。
            let s_total = ev.total_impulse();
            let body_b = self
                .phys
                .bodies
                .get(self.entities[&id].handle)
                .expect("本体在场");
            let d_hat_w = conv::rapier_to_quat(body_b.rotation()) * ev.dir_hat();
            let mut others: Vec<crate::compound_flow::OtherBody> = Vec::new();
            let mut other_list: Vec<(EntityId, Vec3)> = Vec::new(); // (对方, o 一侧作用点)
            for (o, (pw_sum, w)) in &other_points {
                let imp = other_impulses.get(o).copied().unwrap_or(0.0);
                if imp <= 0.0 || s_total <= 0.0 || *w <= 0.0 {
                    continue;
                }
                // m_eff 用**采样时**的对方质量属性快照:o 本拍可能已被先处理的
                // 事件拆掉(刚体已删),拆账口径不能随处理顺序变(§5.8.2)。
                let Some(props) = other_props.get(o) else {
                    continue;
                };
                let point_o = pw_sum / w;
                let m_eff = crate::impact_flow::effective_mass(props, point_o, d_hat_w);
                if !m_eff.is_finite() || m_eff <= 0.0 {
                    continue;
                }
                others.push(crate::compound_flow::OtherBody {
                    effective_mass: m_eff,
                    impulse_share: imp / s_total,
                });
                other_list.push((*o, point_o));
            }
            let split = crate::compound_flow::distribute_momentum(
                self.registry.as_ref(),
                &ev,
                &outcome,
                &others,
            );
            // 本体状态与形状(成形;借用期在 pieces 产出前结束)。
            let (handle, shape, desc, state) = {
                let rec = self.entities.get(&id).expect("本体在本拍采样时在场");
                (
                    rec.handle,
                    rec.desc.shape().expect("资格表只有复合体").clone(),
                    rec.desc.clone(),
                    self.read_state(rec),
                )
            };
            // 分块 = S_k ∪ R_j:局部分量格 → 形状格序号,`fracture_pieces`
            // 生成平移/旋转/描述;速度用 `piece_velocity`(局部系)算出乘 R
            // 覆盖(R_j 取 r_refunds/q_j,S_k 取 s_impulses/p_k)。
            let piece_cells: Vec<Vec<usize>> = outcome
                .s_components
                .iter()
                .chain(split.r_components.iter().map(|r| &r.cells))
                .map(|cells| {
                    cells
                        .iter()
                        .map(|c| {
                            shape
                                .cells()
                                .iter()
                                .position(|sc| sc.offset == [c.x, c.y, c.z])
                                .expect("S/R 分量格必在形状内")
                        })
                        .collect()
                })
                .collect();
            let mut pieces = crate::cohesion::fracture_pieces(&state, &desc, &piece_cells);
            // 覆盖速度(§5.8.2"各块速度"):局部系 piece_velocity → 乘 R。
            // S_k 的冲量/作用点来自 split;R_j 来自 r_refunds;单格走单格分支。
            let body = {
                let rec = self.entities.get(&id).expect("本体在场");
                self.read_state(rec)
            };
            let props_all = crate::compound_flow::body_props(
                &ev.cells.iter().map(|(c, _)| *c).collect::<Vec<_>>(),
            );
            let mut piece_velocities: Vec<(Vec3, Vec3)> = Vec::new();
            {
                let n_s = outcome.s_components.len();
                for (k, comp) in outcome.s_components.iter().enumerate() {
                    let impulse = split.s_impulses[k];
                    let point = split.s_points[k];
                    let world_cells: Vec<BlockPos> = comp.clone();
                    let (v, w) = crate::compound_flow::piece_velocity(
                        &world_cells,
                        &props_all,
                        ev.linvel,
                        ev.angvel,
                        point,
                        impulse,
                        comp.len() == 1,
                    );
                    piece_velocities.push((body.rotation * v, body.rotation * w));
                }
                for (j, (q, refund)) in split.r_refunds.iter().enumerate() {
                    let comp = &split.r_components[j].cells;
                    let (v, w) = crate::compound_flow::piece_velocity(
                        comp,
                        &props_all,
                        ev.linvel,
                        ev.angvel,
                        *q,
                        *refund,
                        comp.len() == 1,
                    );
                    piece_velocities.push((body.rotation * v, body.rotation * w));
                }
                let _ = n_s;
            }
            for ((piece_state, _piece_desc), (v, w)) in pieces.iter_mut().zip(&piece_velocities) {
                piece_state.linvel = *v;
                piece_state.angvel = *w;
            }
            // 名额(§5.8.3):事件名额已在 truncate 处按 s 降序执行;入选事件
            // 的分块不再受 detach 名额约束(legacy 同款——§7.3 的名额就是事件
            // 名额,分块请求不经 detach 路径)。
            // 断开遥测(§5.8.4)。
            self.outbox.cohesion_stats.compound_flow_breaks += 1;
            self.outbox.cohesion_stats.compound_flow_detached_cells +=
                outcome.detached.len() as u64;
            self.outbox.cohesion_stats.bonds_broken += outcome.cut_bonds as u64;
            // 写入分块请求并删除原实体(§7.2 成形,(parent, index) N+1 路径)。
            let piece_base = self.outbox.piece_requests.len();
            for (index, (piece_state, piece_desc)) in pieces.into_iter().enumerate() {
                self.outbox
                    .piece_requests
                    .push(crate::cohesion::PieceRequest {
                        parent: id,
                        index: index as u32,
                        state: piece_state,
                        desc: piece_desc,
                    });
            }
            self.entities.remove(&id).expect("存在");
            self.phys.remove_body(handle);
            self.outbox.despawned.push(id);
            self.outbox.cohesion_stats.fracture_components += piece_cells.len() as u64;
            // 对方已拆分映射(G02-2①):本实体被拆后,"原 offset → 该格所在
            // 分块请求在 piece_requests 里的下标"(S_k∪R_j 的格都是原形状
            // offset,第 i 块的格 = piece_cells[i],全局下标 = base+i)。
            {
                let mut cell_to_req: BTreeMap<BlockPos, usize> = BTreeMap::new();
                for (i, cells) in piece_cells.iter().enumerate() {
                    for &ci in cells {
                        let off = shape.cells()[ci].offset;
                        cell_to_req.insert(BlockPos::new(off[0], off[1], off[2]), piece_base + i);
                    }
                }
                split_piece_index.insert(id, cell_to_req);
            }
            // 对方修正(§5.8.2):`R·correction` 在 o 一侧接触点质心施加并
            // 唤醒;o 已被拆时加到其**含接触格**的那一块分块请求(线速度
            // + P/M;多格块角速度 + R_k·I_k⁻¹·R_kᵀ·(r × P);单格不加)。
            // 定位(G02-2①):o 一侧累计冲量最大的接触格(同值 BlockPos
            // 升序;采样时按 o 的原形状 offset 记账)→ 拆分时记的
            // "原 offset → piece 下标"表;不再做任何兜底猜测。
            let mut trace_corrections: Vec<(Vec3, Vec3, types::EntityId, bool)> = Vec::new();
            for ((o, _share), corr) in other_list.iter().zip(&split.other_corrections) {
                if corr == &Vec3::ZERO {
                    continue;
                }
                let corr_w = body.rotation * *corr;
                let point_w = {
                    let (pw, w) = other_points.get(o).expect("other_list ⊆ other_points");
                    pw / *w
                };
                if let Some(rec_o) = self.entities.get_mut(o) {
                    // 去刚体:施加 + 唤醒(真正施加才计数,§5.8.4)。
                    let h = rec_o.handle;
                    let b = self.phys.bodies.get_mut(h).expect("对方 handle 有效");
                    b.apply_impulse_at_point(
                        conv::vec3_to_rapier(corr_w),
                        conv::vec3_to_rapier(point_w),
                        true,
                    );
                    self.outbox.cohesion_stats.compound_flow_corrections += 1;
                    trace_corrections.push((corr_w, point_w, *o, true));
                } else if let Some(cell_to_req) = split_piece_index.get(o) {
                    // 去分块请求:o 已被拆。目标 = o 一侧累计冲量最大的接触格
                    // 所在的块(同值按 BlockPos 升序取首个;BTreeMap 升序保证)。
                    let cell_impulses = other_cells
                        .get(o)
                        .expect("已拆分对方必有采样时格冲量表(o 侧点进过 other_list)");
                    let target_cell = cell_impulses
                        .iter()
                        .max_by(|a, b| a.1.total_cmp(b.1).then(b.0.cmp(a.0)))
                        .map(|(pos, _)| *pos)
                        .expect("格冲量表非空(other_list ⊆ other_cell_impulses)");
                    let k = cell_to_req.get(&target_cell).copied().unwrap_or_else(|| {
                        // 接触格不在 S_k∪R_j 的任何块 = 不可达(拆分覆盖
                        // 全部格;合成校验外不存在)。debug panic,release
                        // 丢弃修正、不计数(裁定 G02-2①)。
                        debug_assert!(false, "接触格 {target_cell:?} 不在被拆实体的任何分块请求里");
                        usize::MAX
                    });
                    if k != usize::MAX {
                        let req = &mut self.outbox.piece_requests[k];
                        crate::compound_flow::apply_impulse_to_piece(
                            &mut req.state,
                            req.desc.shape.as_ref(),
                            point_w,
                            corr_w,
                        );
                        self.outbox.cohesion_stats.compound_flow_corrections += 1;
                        trace_corrections.push((corr_w, point_w, *o, false));
                    } else {
                        trace_corrections.push((corr_w, point_w, *o, false));
                    }
                } else {
                    // 对方既不在场也没有分块请求(地形份额不会进 other_list;
                    // 防御):修正丢弃、不计数(§5.8.4:只在真正施加时计)。
                    trace_corrections.push((corr_w, point_w, *o, false));
                }
            }
            #[cfg(test)]
            traces.push(CompoundFlowTrace {
                event: ev,
                outcome,
                split,
                piece_velocities,
                other_corrections: trace_corrections,
            });
        }
        #[cfg(test)]
        self.compound_flow_trace.extend(traces);
    }

    /// 承载判定接入(REQ-010 §4.6、§9;step 末调用,**取代** REQ-009 的
    /// `find_collapse`,不计 `collapse_*` 遥测)。流程:
    /// 1. `changed` = 本拍登记的触发格(§4.4:写入前后方块不同、至少一侧实心,
    ///    六路来源);非空时对当前视图求一次 F(接地按柱记忆化,§4.2);
    /// 2. 本拍应用过邻片的承载转换写入(记录非空)时,把记录格换回写入前的
    ///    方块再求一次"转换前"的 F(接地判定同世界——恢复的格可能让上方的格
    ///    重新接地),两次取并集。根因:承载判定不单调,只按写入后的视图会把
    ///    残余部分"减负"后误判挂得住(§9);
    /// 3. F ∩ 本体区 ∩ 当前仍实心 并入待转换集合;按 `BlockPos` 升序**直接
    ///    转换、不重新求值**,数量受本拍剩余名额约束,轮到时已不是实心的格丢弃
    ///    (§4.6 第 4 条);
    /// 4. 转换 = 写 air、带承载转换标记的 `Emergent` 写入、§4.11 唤醒、重建
    ///    collider、冲量 0 的脱落请求(逐格单格碎块,同 REQ-009);转换的格登记
    ///    为下一拍的触发(§4.4 连锁断裂)。遥测:判定计 `CohesionStats::load_*`,
    ///    转换照常计 REQ-009 的 `cells_collapsed`/`detached`。
    pub(super) fn evaluate_load_tick(&mut self) {
        let changed: Vec<BlockPos> = std::mem::take(&mut self.collapse_triggers)
            .into_iter()
            .collect();
        let records = std::mem::take(&mut self.load_conversion_records);
        // 冲量表(K02 §5.6 第 1 条):本拍撞击路径的带冲量触发格,取走即清——
        // 每拍重建/清空,不跨拍残留;只有撞击路径写入(evaluate_impacts),
        // 爆炸/放置等其余来源不带。本拍采样、本拍转换,同拍生效。
        let impact_impulses = std::mem::take(&mut self.impact_impulses);
        let min = self.cfg.shard.pos.min_chunk().block_min();
        let max_x = min.x + SHARD_CHUNKS * CHUNK_SIZE;
        let max_z = min.z + SHARD_CHUNKS * CHUNK_SIZE;
        let in_core = |p: BlockPos| p.x >= min.x && p.x < max_x && p.z >= min.z && p.z < max_z;
        let block_at = |p: BlockPos| -> Option<BlockId> {
            if !(types::WORLD_Y_MIN..=types::WORLD_Y_MAX).contains(&p.y) {
                // 世界高度范围之外按空气,不是未装载(同 REQ-009 口径)。
                return Some(BlockId::AIR);
            }
            // 所在 chunk 未装载 → None(保守:看不见的地方当作有地基)。
            self.voxels.get_chunk(p.chunk_of())?;
            Some(self.voxels.get_block(p))
        };

        let mut falling: BTreeSet<BlockPos> = BTreeSet::new();
        if !changed.is_empty() {
            // ① 写入后的世界(§4.3;接地按柱记忆化,§4.2 修订 2)。
            let grounded =
                crate::cohesion::grounded_by_column_memo(&block_at, self.registry.as_ref());
            let after = crate::cohesion::evaluate_load(
                &block_at,
                &grounded,
                self.registry.as_ref(),
                &changed,
                self.cfg.cohesion.load_max_cells,
            );
            {
                let stats = &mut self.outbox.cohesion_stats;
                stats.load_evaluations += 1;
                stats.load_visited += after.visited as u64;
                stats.load_budget_stable += after.budget_stable as u64;
            }
            falling.extend(after.falling);
            if !records.is_empty() {
                // ② "转换前"的世界(§9):记录格换回写入前的方块,接地同世界。
                let block_before = |p: BlockPos| -> Option<BlockId> {
                    records.get(&p).copied().or_else(|| block_at(p))
                };
                let grounded =
                    crate::cohesion::grounded_by_column_memo(&block_before, self.registry.as_ref());
                let before = crate::cohesion::evaluate_load(
                    &block_before,
                    &grounded,
                    self.registry.as_ref(),
                    &changed,
                    self.cfg.cohesion.load_max_cells,
                );
                let stats = &mut self.outbox.cohesion_stats;
                stats.load_evaluations += 1;
                stats.load_double_evaluations += 1;
                stats.load_visited += before.visited as u64;
                stats.load_budget_stable += before.budget_stable as u64;
                falling.extend(before.falling);
            }
            self.outbox.cohesion_stats.load_falling += falling.len() as u64;
        }

        // 待转换集合:F ∩ 本体区 ∩ 当前仍实心(§4.6 第 4 条;"转换前"F 里的
        // 格可能刚被邻片转换成 air,本体区外的格由其 owner 转换)。
        for &cell in &falling {
            if !in_core(cell) {
                continue;
            }
            let solid = block_at(cell).is_some_and(|b| self.registry.material(b).is_some());
            if solid {
                self.load_pending.insert(cell);
            }
        }

        // 转换(§5.5,修订 5):待转换队列里仍为实心的格先按 `split_segments`
        // 划段(段内升序、各段按首格升序),依次处理——1 格的段 = 单格脱落请求
        // (同 REQ-009),≥ 2 格的段 = 一个复合体生成请求;每段占 1 个实体名额
        // (与同拍已有的脱落/复合体请求共用 `max_detach_per_tick`),格数另受
        // `max_converted_cells_per_tick` 约束;**某段放不下时本拍停止转换**,该段
        // 与其后各段整段留在待转换集合,下一拍继续——不跳段、不拆段(§5.5 第
        // 2 条;跳段会让转换顺序随名额变化,回放与跨分片不一致)。
        let mut quota = self.cfg.destruction.max_detach_per_tick.saturating_sub(
            self.outbox.detach_requests.len() + self.outbox.compound_requests.len(),
        );
        let mut cells_budget = self.cfg.cohesion.max_converted_cells_per_tick;
        let queue: Vec<BlockPos> = self.load_pending.iter().copied().collect();
        // 各格转换前的方块(划段求 bond 与构造形状都要它;逐段写 air 不改未
        // 转换格,先读齐再处理,不做跨段重读)。
        let mut solid_blocks: BTreeMap<BlockPos, BlockId> = BTreeMap::new();
        for cell in &queue {
            let block = self.voxels.get_block(*cell);
            if self.registry.material(block).is_some() {
                solid_blocks.insert(*cell, block);
            } else {
                // 轮到时已不是实心(同拍爆炸/撞击已写掉):丢弃,不占名额。
                self.load_pending.remove(cell);
            }
        }
        let still_solid: Vec<BlockPos> = solid_blocks.keys().copied().collect();
        let reg = self.registry.as_ref();
        let lookup = |p: BlockPos| solid_blocks.get(&p).copied();
        let segments = crate::cohesion::split_segments(
            &lookup,
            reg,
            &still_solid,
            self.cfg.cohesion.compound_split_size,
        );
        let mut dirty: Vec<ChunkPos> = Vec::new();
        // 遥测(§5.5 第 6 条):切块产生的段数(含 1 格段)。
        self.outbox.cohesion_stats.segments += segments.len() as u64;
        for segment in segments {
            if quota == 0 || cells_budget < segment.len() {
                // 名额或格数不足:本拍停止转换,该段与其后各段整段留下(§5.5 第 2 条)。
                break;
            }
            quota -= 1;
            cells_budget -= segment.len();
            if segment.len() == 1 {
                let cell = segment[0];
                let block = solid_blocks[&cell];
                self.convert_load_cell(cell, &mut dirty);
                self.outbox
                    .detach_requests
                    .push(crate::destruction::DetachRequest {
                        cell,
                        block,
                        impulse: Vec3::ZERO,
                    });
            } else {
                let origin = crate::cohesion::compound_origin(&segment);
                let world: Vec<(BlockPos, BlockId)> =
                    segment.iter().map(|&c| (c, solid_blocks[&c])).collect();
                let shape = types::CompoundShape::from_world(origin, &world)
                    .expect("段内 ≥ 2 格、偏移不重复(划分保证)");
                // 邻接继承(K02 §5.6 第 2 条):段内存在格与本拍带冲量触发格
                // 6 邻邻接 ⟺ 继承;对每段,取其邻接的去重后每个带冲量触发格,
                // 生成一条 (β·J, 触发格中心)(β 构造时乘入);不做经空气的
                // 远距传播;多个失载段各继承全额(逐段独立调用)。无撞击来源
                // 的拍冲量表为空 → impulses 恒空,零速不变(AC-K2)。
                let impulses = crate::cohesion::inherit_impulses(
                    &segment,
                    &impact_impulses,
                    self.cfg.cohesion.load_knockback_factor,
                );
                for &cell in &segment {
                    self.convert_load_cell(cell, &mut dirty);
                }
                // 遥测(§5.5 第 6 条):复合体在转换拍计数。
                self.outbox.cohesion_stats.compounds_spawned += 1;
                self.outbox.cohesion_stats.compound_cells += segment.len() as u64;
                self.outbox
                    .compound_requests
                    .push(crate::cohesion::CompoundRequest {
                        origin,
                        shape,
                        impulses,
                        // §5.6 邻接继承路径无初速度字段(REQ-010 §5.7.3:F02b
                        // 的 `velocity` 只属于冲量流路径;默认零,既有行为不变)。
                        velocity: (types::glam::Vec3::ZERO, types::glam::Vec3::ZERO),
                    });
            }
            for &cell in &segment {
                self.load_pending.remove(&cell);
            }
        }
        // 细则 4/5 同款守卫:只重建已建 ②(同 REQ-009 转换);S03 起记入
        // 待重建集合,step 末统一重建。
        self.defer_chunk_collider_rebuild(dirty);
        // 与同拍爆炸/撞击的脱落请求拼接后,保持 BlockPos 升序不变量。
        self.outbox.detach_requests.sort_by_key(|req| req.cell);
        self.outbox.cohesion_stats.load_pending = self.load_pending.len() as u64;
    }

    /// 承载转换的单格写入(§5.5 第 3 条):写 air、带承载转换标记的
    /// `Emergent` 写入、§4.11 唤醒、登记为下一拍的触发(连锁断裂);
    /// REQ-009 口径遥测(cells_collapsed/detached,单格与复合体的格都计)。
    /// caller 负责名额扣减、请求产出与 collider 重建。
    fn convert_load_cell(&mut self, cell: BlockPos, dirty: &mut Vec<ChunkPos>) {
        let _ = self.voxels.set_block(cell, BlockId::AIR);
        self.outbox.destruction_stats.cells_collapsed += 1;
        self.outbox.destruction_stats.detached += 1;
        self.outbox.block_updates.push(BlockWrite {
            pos: cell,
            block: BlockId::AIR,
            kind: BlockWriteKind::Emergent,
            load_conversion: true,
        });
        // §4.11 统一出口:脚下被掏空的冻结体下一拍被唤醒。
        self.register_terrain_write(cell);
        // 转换的写入登记为下一拍的触发(§4.4 连锁断裂自然发生)。
        self.collapse_triggers.insert(cell);
        dirty.push(cell.chunk_of());
    }
}
