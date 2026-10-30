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
        // 采样资格(§7.3 第 2 条):醒着的复合体;速度基准 = 本步之前的线速度。
        // 先于接触映射建表(终审审计 P3):资格构造不依赖下方映射,绝大多数拍
        // 没有合格复合体,这里直接早退,免付全量建表。
        let eligible: BTreeMap<EntityId, Vec3> = self
            .pre_impulse_vels
            .iter()
            .filter(|(id, _)| {
                let Some(rec) = self.entities.get(id) else {
                    return false;
                };
                rec.desc.shape().is_some()
                    && self
                        .phys
                        .bodies
                        .get(rec.handle)
                        .is_some_and(|b| b.is_dynamic() && !b.is_sleeping())
            })
            .map(|(id, v)| (*id, *v))
            .collect();
        if eligible.is_empty() {
            return;
        }
        // collider → 实体(复合体 = 全部格 collider)与复合体 collider → 形状格
        // 序号(局部平移 = 偏移,按偏移匹配,不依赖 colliders() 的插入顺序)。
        // 只登记有形状的实体(终审审计 P3):单格实体的 collider 在下方采样里
        // 本就会被 `collider_to_cell` 查空挡下,表里不占位。
        let mut collider_to_entity: BTreeMap<(u32, u32), EntityId> = BTreeMap::new();
        let mut collider_to_cell: BTreeMap<(u32, u32), usize> = BTreeMap::new();
        for (id, rec) in &self.entities {
            let Some(shape) = rec.desc.shape() else {
                continue;
            };
            let idx = rec.collider.into_raw_parts();
            collider_to_entity.insert((idx.0, idx.1), *id);
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
            for m in pair.manifolds() {
                // 法向为 world 系、以碰撞体 1 → 2 为正:对 1 侧"指向本体内部"
                // = −normal,对 2 侧 = +normal(与撞击采样"撞击方向"互为反向量)。
                for (key, dir_into) in [(key1, -m.data.normal), (key2, m.data.normal)] {
                    let Some(&id) = collider_to_entity.get(&key) else {
                        continue;
                    };
                    let Some(&cell) = collider_to_cell.get(&key) else {
                        continue;
                    };
                    let Some(pre_vel) = eligible.get(&id) else {
                        continue;
                    };
                    // 接近速度(REQ-009 §5 修订 9 同口径):本步前线速度沿接触
                    // 法向**朝对方**的分量 ≥ 1 m/s 才计入;受力方向指向本体
                    // 内部,与接近方向互为反向量。
                    if (-pre_vel).dot(dir_into) < crate::destruction::IMPACT_MIN_APPROACH_SPEED {
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
