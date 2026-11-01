//! 承载判定与碎裂的拍内编排(REQ-010 §4.6、§7.3;终审审计 P3 拆分)。
//!
//! 纯函数(承载最大流、切块、断键、拆分、爆炸逐格)都在 [`crate::cohesion`],
//! 本文件只留"采样—预算—名额—outbox 产出"的粘合:碎裂采样
//! ([`ShardSim::evaluate_fractures`])、承载接入([`ShardSim::evaluate_load_tick`])
//! 与其单格写入([`ShardSim::convert_load_cell`])。调度入口仍在 `sim.rs`
//! (`step` 与 `evaluate_collapse` 的开关分派);REQ-009 的逐格坍塌路径
//! (`evaluate_collapse` 本体)不在此,留在 `sim.rs`。

use super::*;

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
        // 细则 4/5 同款守卫:只重建已建 ②(同 REQ-009 转换)。
        for cp in expand_boundary(dirty, &self.voxels) {
            if self.built_colliders.contains(&cp) {
                self.rebuild_chunk_collider(cp);
            }
        }
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
