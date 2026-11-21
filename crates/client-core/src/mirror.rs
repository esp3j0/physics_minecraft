//! 世界镜像与 subchunk 脏队列(REQ-013 C01b-1 §2.1)。
//!
//! 纯逻辑、零渲染依赖、零墙钟、BTree 系容器保证确定性顺序。
//! 每个 chunk 高度 384(世界 y ∈ [-64, 319])按 16 格切为 24 段(section 0..=23)。

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex};

use types::{BlockId, BlockPos, CHUNK_SIZE, ChunkPos, WORLD_Y_MAX, WORLD_Y_MIN};
use voxel::Chunk;

/// 每个 chunk 的纵向 subchunk 段数(384 / 16 = 24)。
pub const SECTIONS_PER_CHUNK: u8 = 24;

/// 单个 subchunk 的纵向高度(格)。
pub const SECTION_HEIGHT: usize = 16;

/// 四水平邻 chunk 偏移(x, z)。
const HORIZONTAL_NEIGHBORS: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

/// Subchunk 坐标:chunk 网格坐标 + 纵向段号(0..=23)。
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct SubchunkPos {
    pub chunk: ChunkPos,
    pub section: u8,
}

/// 脏标记优先级:`Edit` 优先于 `Load`。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DirtyKind {
    Edit,
    Load,
}

/// LRU 逻辑时钟状态(零墙钟,确定性单调计数)。
#[derive(Default)]
struct LruState {
    last_use: BTreeMap<ChunkPos, u64>,
    use_clock: u64,
}

/// 客户端侧世界体素镜像与确定性脏队列。
///
/// 内部使用 `Arc<Chunk>` 支持写时复制(`Arc::make_mut`),便于后续异步网格任务共享快照;
/// 脏表拆分为互斥的 `dirty_edit` / `dirty_load` 两个 `BTreeSet`,确保 `Edit` 不被 `Load`
/// 覆盖且弹出顺序严格确定。
pub struct ChunkMirror {
    chunks: BTreeMap<ChunkPos, Arc<Chunk>>,
    dirty_edit: BTreeSet<SubchunkPos>,
    dirty_load: BTreeSet<SubchunkPos>,
    epochs: BTreeMap<SubchunkPos, u64>,
    lru: Mutex<LruState>,
}

impl Default for ChunkMirror {
    fn default() -> Self {
        Self::new()
    }
}

impl ChunkMirror {
    /// 构造空镜像。
    pub fn new() -> Self {
        Self {
            chunks: BTreeMap::new(),
            dirty_edit: BTreeSet::new(),
            dirty_load: BTreeSet::new(),
            epochs: BTreeMap::new(),
            lru: Mutex::new(LruState::default()),
        }
    }

    /// 命中 chunk 时推进逻辑时钟并记录 `last_use[pos]`(不跨 await、不嵌套锁)。
    fn touch(&self, pos: ChunkPos) {
        let mut lru = self.lru.lock().expect("ChunkMirror LRU lock poisoned");
        lru.use_clock = lru.use_clock.saturating_add(1);
        let clock = lru.use_clock;
        lru.last_use.insert(pos, clock);
    }

    /// 装载(或替换)一个 chunk:本 chunk 全 24 段标 `Load`,已装载的四水平邻 chunk
    /// 全 24 段亦标 `Load`(已有 `Edit` 标记不被降级)。
    pub fn insert_chunk(&mut self, pos: ChunkPos, chunk: Chunk) {
        self.chunks.insert(pos, Arc::new(chunk));
        self.touch(pos);
        self.mark_chunk_load(pos);
        for (dx, dz) in HORIZONTAL_NEIGHBORS {
            let neighbor = ChunkPos::new(pos.x + dx, pos.z + dz);
            if self.chunks.contains_key(&neighbor) {
                self.mark_chunk_load(neighbor);
            }
        }
    }

    /// 卸载一个 chunk:移除本体、清除其全部脏条目,并将已装载的四水平邻 chunk 全 24 段标 `Load`。
    pub fn remove_chunk(&mut self, pos: ChunkPos) {
        self.chunks.remove(&pos);
        self.lru
            .lock()
            .expect("ChunkMirror LRU lock poisoned")
            .last_use
            .remove(&pos);
        for section in 0..SECTIONS_PER_CHUNK {
            let sc = SubchunkPos {
                chunk: pos,
                section,
            };
            self.dirty_edit.remove(&sc);
            self.dirty_load.remove(&sc);
        }
        for (dx, dz) in HORIZONTAL_NEIGHBORS {
            let neighbor = ChunkPos::new(pos.x + dx, pos.z + dz);
            if self.chunks.contains_key(&neighbor) {
                self.mark_chunk_load(neighbor);
            }
        }
    }

    /// 按 LRU 上限驱逐非保护 chunk(REQ-013 C02-2 D1):
    /// 当 `chunks.len() > cap` 时循环在非 `protect` 候选中选 `(last_use, ChunkPos)` 最小者
    /// 经 [`Self::remove_chunk`] 移除并按移除顺序返回;无非保护候选时立即退出。
    pub fn enforce_cap(&mut self, cap: usize, protect: &BTreeSet<ChunkPos>) -> Vec<ChunkPos> {
        let mut evicted = Vec::new();
        while self.chunks.len() > cap {
            let victim = {
                let lru = self.lru.lock().expect("ChunkMirror LRU lock poisoned");
                self.chunks
                    .keys()
                    .copied()
                    .filter(|pos| !protect.contains(pos))
                    .min_by_key(|pos| (lru.last_use.get(pos).copied().unwrap_or(0), *pos))
            };
            let Some(pos) = victim else {
                break;
            };
            self.remove_chunk(pos);
            evicted.push(pos);
        }
        evicted
    }

    /// 批量应用世界坐标方块更新(写时复制 `Arc::make_mut`),并标脏本格所在段及存在的边界邻段。
    /// 世界高度外或未装载 chunk 的更新静默忽略。
    pub fn apply_updates(&mut self, updates: &[(BlockPos, BlockId)]) {
        for &(world, block) in updates {
            if !(WORLD_Y_MIN..=WORLD_Y_MAX).contains(&world.y) {
                continue;
            }
            let chunk_pos = world.chunk_of();
            let local_x = world.x.rem_euclid(CHUNK_SIZE) as usize;
            let local_y = (world.y - WORLD_Y_MIN) as usize;
            let local_z = world.z.rem_euclid(CHUNK_SIZE) as usize;

            let Some(arc) = self.chunks.get_mut(&chunk_pos) else {
                continue;
            };
            Arc::make_mut(arc).set(local_x, local_y, local_z, block);
            self.touch(chunk_pos);

            let section = (local_y / SECTION_HEIGHT) as u8;
            self.mark_dirty(
                SubchunkPos {
                    chunk: chunk_pos,
                    section,
                },
                DirtyKind::Edit,
            );

            if local_x == 0 {
                let n = ChunkPos::new(chunk_pos.x - 1, chunk_pos.z);
                if self.chunks.contains_key(&n) {
                    self.mark_dirty(SubchunkPos { chunk: n, section }, DirtyKind::Edit);
                }
            } else if local_x == 15 {
                let n = ChunkPos::new(chunk_pos.x + 1, chunk_pos.z);
                if self.chunks.contains_key(&n) {
                    self.mark_dirty(SubchunkPos { chunk: n, section }, DirtyKind::Edit);
                }
            }

            if local_z == 0 {
                let n = ChunkPos::new(chunk_pos.x, chunk_pos.z - 1);
                if self.chunks.contains_key(&n) {
                    self.mark_dirty(SubchunkPos { chunk: n, section }, DirtyKind::Edit);
                }
            } else if local_z == 15 {
                let n = ChunkPos::new(chunk_pos.x, chunk_pos.z + 1);
                if self.chunks.contains_key(&n) {
                    self.mark_dirty(SubchunkPos { chunk: n, section }, DirtyKind::Edit);
                }
            }

            let y_in_sec = local_y % SECTION_HEIGHT;
            if y_in_sec == 0 && section > 0 {
                self.mark_dirty(
                    SubchunkPos {
                        chunk: chunk_pos,
                        section: section - 1,
                    },
                    DirtyKind::Edit,
                );
            } else if y_in_sec == 15 && section + 1 < SECTIONS_PER_CHUNK {
                self.mark_dirty(
                    SubchunkPos {
                        chunk: chunk_pos,
                        section: section + 1,
                    },
                    DirtyKind::Edit,
                );
            }
        }
    }

    /// 世界坐标取格;未装载 chunk 或世界高度外返回 `None`。
    pub fn get_block(&self, world: BlockPos) -> Option<BlockId> {
        if !(WORLD_Y_MIN..=WORLD_Y_MAX).contains(&world.y) {
            return None;
        }
        let chunk_pos = world.chunk_of();
        let chunk = self.chunks.get(&chunk_pos)?;
        self.touch(chunk_pos);
        let local_x = world.x.rem_euclid(CHUNK_SIZE) as usize;
        let local_y = (world.y - WORLD_Y_MIN) as usize;
        let local_z = world.z.rem_euclid(CHUNK_SIZE) as usize;
        Some(chunk.get(local_x, local_y, local_z))
    }

    /// 当前已装载 chunk 数量。
    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }

    /// 弹出优先级最高的脏段:`Edit` 先于 `Load`;同级按 [`SubchunkPos`] `Ord` 升序(确定性)。
    pub fn pop_dirty(&mut self) -> Option<(SubchunkPos, DirtyKind)> {
        if let Some(sc) = self.dirty_edit.pop_first() {
            return Some((sc, DirtyKind::Edit));
        }
        self.dirty_load.pop_first().map(|sc| (sc, DirtyKind::Load))
    }

    /// 当前待处理脏段总数。
    pub fn dirty_len(&self) -> usize {
        self.dirty_edit.len() + self.dirty_load.len()
    }

    /// 零拷贝克隆已装载 chunk 的 `Arc` 句柄(供异步网格快照使用)。
    pub fn chunk_arc(&self, pos: ChunkPos) -> Option<Arc<Chunk>> {
        let arc = self.chunks.get(&pos).map(Arc::clone)?;
        self.touch(pos);
        Some(arc)
    }

    /// 返回指定 subchunk 的脏世代计数(初值 0,每次 `mark_dirty` 命中 +1)。
    ///
    /// `remove_chunk` 清自身脏条目时**保留** `epoch` 计数不清零——卸载再装载同段
    /// 视为新一代,计数继续累加。
    pub fn epoch(&self, sc: SubchunkPos) -> u64 {
        self.epochs.get(&sc).copied().unwrap_or(0)
    }

    /// 将 subchunk 重新入脏队(等价内部 `mark_dirty`,供异步过期结果回队或 pending 跳过回队)。
    pub fn requeue(&mut self, sc: SubchunkPos, kind: DirtyKind) {
        self.mark_dirty(sc, kind);
    }

    /// 按世界坐标读取方块;越界或所在 chunk 未装载返回 `None`。
    pub fn block(&self, pos: BlockPos) -> Option<BlockId> {
        if !(WORLD_Y_MIN..=WORLD_Y_MAX).contains(&pos.y) {
            return None;
        }
        let chunk_pos = pos.chunk_of();
        let chunk = self.chunks.get(&chunk_pos)?;
        self.touch(chunk_pos);
        Some(chunk.get(
            pos.x.rem_euclid(CHUNK_SIZE) as usize,
            (pos.y - WORLD_Y_MIN) as usize,
            pos.z.rem_euclid(CHUNK_SIZE) as usize,
        ))
    }

    fn mark_chunk_load(&mut self, pos: ChunkPos) {
        for section in 0..SECTIONS_PER_CHUNK {
            self.mark_dirty(
                SubchunkPos {
                    chunk: pos,
                    section,
                },
                DirtyKind::Load,
            );
        }
    }

    fn mark_dirty(&mut self, sc: SubchunkPos, kind: DirtyKind) {
        let entry = self.epochs.entry(sc).or_insert(0);
        *entry = entry.saturating_add(1);
        match kind {
            DirtyKind::Edit => {
                self.dirty_load.remove(&sc);
                self.dirty_edit.insert(sc);
            }
            DirtyKind::Load => {
                if !self.dirty_edit.contains(&sc) {
                    self.dirty_load.insert(sc);
                }
            }
        }
    }
}

/// 异步网格任务过期判定:任务记录的世代与镜像当前世代不等即过期(纯函数,C01b-2 MS4)。
#[inline]
pub fn is_stale(pending_epoch: u64, current_epoch: u64) -> bool {
    pending_epoch != current_epoch
}

impl crate::script::WorldQuery for ChunkMirror {
    fn live_entities(&self) -> Vec<(types::EntityId, types::glam::Vec3)> {
        Vec::new()
    }

    fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
        self.block(pos)
    }

    fn chunks_loaded(&self) -> usize {
        self.chunks.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STONE: BlockId = BlockId(1);
    const DIRT: BlockId = BlockId(2);

    /// MB1:insert chunk(0,0) 含 STONE@(0,-60,0) → get_block/chunk_count/越界/remove 行为。
    #[test]
    fn mb1_insert_get_oob_remove() {
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        // 世界 y = -60 → 局部 y = -60 + 64 = 4
        chunk.set(0, 4, 0, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        assert_eq!(mirror.get_block(BlockPos::new(0, -60, 0)), Some(STONE));
        assert_eq!(mirror.chunk_count(), 1);
        assert_eq!(mirror.get_block(BlockPos::new(0, -65, 0)), None);

        mirror.remove_chunk(ChunkPos::new(0, 0));
        assert_eq!(mirror.get_block(BlockPos::new(0, -60, 0)), None);
        assert_eq!(mirror.chunk_count(), 0);
    }

    /// MB2(裁定 §2):insert air chunk(0,0),(1,0) → 清空 48 条初始 Load →
    /// apply_updates([(15,-60,0,STONE),(0,-49,0,DIRT)]) → 脏表恰 3 条 Edit 且按 Ord 升序弹出。
    #[test]
    fn mb2_apply_updates_boundary_edit_dirty() {
        let mut mirror = ChunkMirror::new();
        mirror.insert_chunk(ChunkPos::new(0, 0), Chunk::air());
        mirror.insert_chunk(ChunkPos::new(1, 0), Chunk::air());

        for _ in 0..48 {
            let popped = mirror.pop_dirty();
            assert!(matches!(popped, Some((_, DirtyKind::Load))));
        }
        assert_eq!(mirror.dirty_len(), 0);

        mirror.apply_updates(&[
            (BlockPos::new(15, -60, 0), STONE),
            (BlockPos::new(0, -49, 0), DIRT),
        ]);

        assert_eq!(mirror.dirty_len(), 3);
        assert_eq!(
            mirror.pop_dirty(),
            Some((
                SubchunkPos {
                    chunk: ChunkPos::new(0, 0),
                    section: 0,
                },
                DirtyKind::Edit
            ))
        );
        assert_eq!(
            mirror.pop_dirty(),
            Some((
                SubchunkPos {
                    chunk: ChunkPos::new(0, 0),
                    section: 1,
                },
                DirtyKind::Edit
            ))
        );
        assert_eq!(
            mirror.pop_dirty(),
            Some((
                SubchunkPos {
                    chunk: ChunkPos::new(1, 0),
                    section: 0,
                },
                DirtyKind::Edit
            ))
        );
        assert_eq!(mirror.pop_dirty(), None);
        assert_eq!(mirror.get_block(BlockPos::new(15, -60, 0)), Some(STONE));
    }

    /// MB3:insert air chunk(2,0)(24 段 Load) + apply_updates([(32,-60,0,STONE)]) →
    /// 首 pop 为 ((2,0), 0, Edit),其后 23 条全为 Load。
    #[test]
    fn mb3_edit_priority_before_load() {
        let mut mirror = ChunkMirror::new();
        mirror.insert_chunk(ChunkPos::new(2, 0), Chunk::air());
        assert_eq!(mirror.dirty_len(), 24);

        mirror.apply_updates(&[(BlockPos::new(32, -60, 0), STONE)]);
        assert_eq!(mirror.dirty_len(), 24);

        assert_eq!(
            mirror.pop_dirty(),
            Some((
                SubchunkPos {
                    chunk: ChunkPos::new(2, 0),
                    section: 0,
                },
                DirtyKind::Edit
            ))
        );

        let mut load_count = 0;
        while let Some((sc, kind)) = mirror.pop_dirty() {
            assert_eq!(sc.chunk, ChunkPos::new(2, 0));
            assert_eq!(kind, DirtyKind::Load);
            load_count += 1;
        }
        assert_eq!(load_count, 23);
    }

    /// MB4(C01b-1-R1):insert_chunk / remove_chunk 对已装载水平邻 chunk 的连带 Load 标脏与确定性顺序。
    #[test]
    fn mb4_neighbor_load_marks_on_insert_and_remove() {
        let mut mirror = ChunkMirror::new();

        // 1. insert (0,0) → 24 段 Load,section 0..=23 升序
        mirror.insert_chunk(ChunkPos::new(0, 0), Chunk::air());
        assert_eq!(mirror.dirty_len(), 24);
        for section in 0..24u8 {
            assert_eq!(
                mirror.pop_dirty(),
                Some((
                    SubchunkPos {
                        chunk: ChunkPos::new(0, 0),
                        section,
                    },
                    DirtyKind::Load
                ))
            );
        }
        assert_eq!(mirror.dirty_len(), 0);

        // 2. insert (1,0) → 自身 24 + 已装载邻 (0,0) 24 = 48 段 Load;
        //    Ord 升序:前 24 为 (0,0) section 0..=23,后 24 为 (1,0) section 0..=23
        mirror.insert_chunk(ChunkPos::new(1, 0), Chunk::air());
        assert_eq!(mirror.dirty_len(), 48);
        for section in 0..24u8 {
            assert_eq!(
                mirror.pop_dirty(),
                Some((
                    SubchunkPos {
                        chunk: ChunkPos::new(0, 0),
                        section,
                    },
                    DirtyKind::Load
                ))
            );
        }
        for section in 0..24u8 {
            assert_eq!(
                mirror.pop_dirty(),
                Some((
                    SubchunkPos {
                        chunk: ChunkPos::new(1, 0),
                        section,
                    },
                    DirtyKind::Load
                ))
            );
        }
        assert_eq!(mirror.dirty_len(), 0);

        // 3. remove (1,0) → 清自身脏并标已装载邻 (0,0) 24 段 Load
        mirror.remove_chunk(ChunkPos::new(1, 0));
        assert_eq!(mirror.dirty_len(), 24);
        for section in 0..24u8 {
            assert_eq!(
                mirror.pop_dirty(),
                Some((
                    SubchunkPos {
                        chunk: ChunkPos::new(0, 0),
                        section,
                    },
                    DirtyKind::Load
                ))
            );
        }
        assert_eq!(mirror.dirty_len(), 0);
        assert_eq!(mirror.pop_dirty(), None);
    }

    /// MS2(C01b-2):epoch 世代计数 0→1→2→3→4→5,且 remove_chunk 保留已卸载段 epoch 不清零。
    #[test]
    fn ms2_epoch_lifecycle_and_retention_on_remove() {
        let mut mirror = ChunkMirror::new();
        let s0 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let s1 = SubchunkPos {
            chunk: ChunkPos::new(1, 0),
            section: 0,
        };

        assert_eq!(mirror.epoch(s0), 0);

        mirror.insert_chunk(ChunkPos::new(0, 0), Chunk::air());
        assert_eq!(mirror.epoch(s0), 1);

        for _ in 0..24 {
            assert!(matches!(mirror.pop_dirty(), Some((_, DirtyKind::Load))));
        }
        assert_eq!(mirror.dirty_len(), 0);

        mirror.apply_updates(&[(BlockPos::new(0, -60, 0), STONE)]);
        assert_eq!(mirror.epoch(s0), 2);

        mirror.apply_updates(&[(BlockPos::new(15, -60, 0), DIRT)]);
        assert_eq!(mirror.epoch(s0), 3);

        mirror.insert_chunk(ChunkPos::new(1, 0), Chunk::air());
        assert_eq!(mirror.epoch(s0), 4);
        assert_eq!(mirror.epoch(s1), 1);

        mirror.remove_chunk(ChunkPos::new(1, 0));
        assert_eq!(mirror.epoch(s0), 5);
        assert_eq!(mirror.epoch(s1), 1);
    }

    /// MS3(C01b-2):requeue(Edit) 弹出 Edit;requeue(Edit) 后再 requeue(Load) 不降级且不重复入队。
    #[test]
    fn ms3_requeue_priority_no_downgrade() {
        let mut mirror = ChunkMirror::new();
        let s0 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };

        mirror.requeue(s0, DirtyKind::Edit);
        assert_eq!(mirror.pop_dirty(), Some((s0, DirtyKind::Edit)));
        assert_eq!(mirror.pop_dirty(), None);

        mirror.requeue(s0, DirtyKind::Edit);
        mirror.requeue(s0, DirtyKind::Load);
        assert_eq!(mirror.pop_dirty(), Some((s0, DirtyKind::Edit)));
        assert_eq!(mirror.pop_dirty(), None);
    }

    /// MS4(C01b-2):异步网格任务过期判定纯函数。
    #[test]
    fn ms4_is_stale_epoch_check() {
        assert!(is_stale(1, 2));
        assert!(!is_stale(2, 2));
    }

    fn ms25_setup_abc_touch_a() -> ChunkMirror {
        let mut mirror = ChunkMirror::new();
        mirror.insert_chunk(ChunkPos::new(0, 0), Chunk::air());
        mirror.insert_chunk(ChunkPos::new(1, 0), Chunk::air());
        mirror.insert_chunk(ChunkPos::new(2, 0), Chunk::air());
        for _ in 0..72 {
            assert!(matches!(mirror.pop_dirty(), Some((_, DirtyKind::Load))));
        }
        assert_eq!(mirror.dirty_len(), 0);
        // 读 A → A 时钟推进至 4(B=2, C=3, A=4)。
        assert_eq!(mirror.block(BlockPos::new(0, -60, 0)), Some(BlockId::AIR));
        mirror
    }

    /// MS25 t1(C02-2 D1 + R1 B1 / PM28 靶点):读 A 后驱逐最久未用的 B(1,0),
    /// 连带标脏已装载邻 A/C 共 48 段,且 B 的 section0 epoch 保留 == 2(insert B 为 1, insert C 邻标为 2)。
    #[test]
    fn ms25_t1_evict_lru_retains_epoch() {
        let mut mirror = ms25_setup_abc_touch_a();
        let evicted = mirror.enforce_cap(2, &BTreeSet::new());
        assert_eq!(evicted, vec![ChunkPos::new(1, 0)]);
        assert_eq!(mirror.chunk_count(), 2);
        assert_eq!(mirror.get_block(BlockPos::new(16, -60, 0)), None);
        assert_eq!(mirror.dirty_len(), 48);
        assert_eq!(
            mirror.epoch(SubchunkPos {
                chunk: ChunkPos::new(1, 0),
                section: 0,
            }),
            2
        );
    }

    /// MS25 t2(C02-2 D1 / PM27 靶点):protect={B} 时跳过更旧的 B,驱逐次旧的 C(2,0),
    /// 仅连带已装载邻 B 共 24 段。
    #[test]
    fn ms25_t2_protect_skips_oldest() {
        let mut mirror = ms25_setup_abc_touch_a();
        let protect = BTreeSet::from([ChunkPos::new(1, 0)]);
        let evicted = mirror.enforce_cap(2, &protect);
        assert_eq!(evicted, vec![ChunkPos::new(2, 0)]);
        assert_eq!(mirror.dirty_len(), 24);
    }

    /// MS25 t3(C02-2 D1):已装载数 <= cap 时不驱逐、脏队列保持 0。
    #[test]
    fn ms25_t3_under_cap_noop() {
        let mut mirror = ms25_setup_abc_touch_a();
        let evicted = mirror.enforce_cap(3, &BTreeSet::new());
        assert!(evicted.is_empty());
        assert_eq!(mirror.dirty_len(), 0);
    }

    /// MS25 t4(C02-2 D1):全部 chunk 均在 protect 内时不驱逐且不死循环。
    #[test]
    fn ms25_t4_all_protected_no_evict() {
        let mut mirror = ms25_setup_abc_touch_a();
        let protect = BTreeSet::from([
            ChunkPos::new(0, 0),
            ChunkPos::new(1, 0),
            ChunkPos::new(2, 0),
        ]);
        let evicted = mirror.enforce_cap(1, &protect);
        assert!(evicted.is_empty());
        assert_eq!(mirror.chunk_count(), 3);
    }

    /// MS25 t5(C02-2 D1 / R2 期望更正):A/B/C/D 依次装载后不读,enforce_cap(2) 依序驱逐
    /// A(0,0)、B(1,0);移 A 连带 B 24 段,但移 B 时 remove_chunk 先清 B 自身这 24 条待处理
    /// 标记再连带 C 24 段 → 总 dirty_len == 24(架构师 R2 手算更正,原 48 漏清自身步骤)。
    #[test]
    fn ms25_t5_multi_evict_order() {
        let mut mirror = ChunkMirror::new();
        for cx in 0..4 {
            mirror.insert_chunk(ChunkPos::new(cx, 0), Chunk::air());
        }
        for _ in 0..96 {
            assert!(matches!(mirror.pop_dirty(), Some((_, DirtyKind::Load))));
        }
        assert_eq!(mirror.dirty_len(), 0);

        let evicted = mirror.enforce_cap(2, &BTreeSet::new());
        assert_eq!(evicted, vec![ChunkPos::new(0, 0), ChunkPos::new(1, 0)]);
        assert_eq!(mirror.dirty_len(), 24);
    }
}
