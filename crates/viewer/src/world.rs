//! world —— viewer 的世界状态(REQ-008 单元 02):chunk 方块镜像、实体的服务器位置、
//! 服务器拍号、待取的 Spawned 回执。纯逻辑,零 macroquad;渲染层据此维护 Mesh 与实体插值,
//! 脚本引擎经 [`WorldQuery`] 读取。
//!
//! 语义(验收用例据此编写):
//! - `Welcome` / `Entities` 更新拍号;
//! - `Chunk` 整块替换,`ChunkGone` 移除;两者都把该 chunk 记为"脏"(渲染层据此重建或删除 Mesh);
//! - `Blocks` 只改已装载 chunk 的镜像并记脏;**未装载 chunk 的方块增量忽略**(随后到达的
//!   ChunkData 已含最新内容,不为它造一个只有空气的假 chunk);
//! - `Entities` 增量按 latest-wins 更新、`removed` 删除;`full` 时以 `states` 重建实体表;
//!   `live_entities` 返回服务器位置(不是渲染插值后的位置);
//! - `Spawned` 回执按到达顺序累积,由 [`WorldState::take_spawned`] 取走;
//! - `Disconnected` 不改状态(由渲染层处理)。

use std::collections::BTreeMap;

use types::glam::Vec3;
use types::{BlockId, BlockPos, ChunkPos, EntityId};

use crate::client::ViewEvent;
use crate::script::WorldQuery;

/// viewer 的世界状态。
#[derive(Debug, Default)]
pub struct WorldState {
    chunks: BTreeMap<ChunkPos, voxel::Chunk>,
    entities: BTreeMap<EntityId, Vec3>,
    tick: u64,
    /// 自上次 `take_dirty_chunks` 以来变脏的 chunk(BTreeSet:升序、去重)。
    dirty: std::collections::BTreeSet<ChunkPos>,
    /// 自上次 `take_spawned` 以来收到的 Spawned 回执(到达顺序)。
    spawned: Vec<EntityId>,
}

impl WorldState {
    pub fn new() -> Self {
        Self::default()
    }

    /// chunk 局部坐标换算(负方向 floor;y 全高,局部 0 = WORLD_Y_MIN)。
    fn local(pos: BlockPos) -> (usize, usize, usize) {
        (
            pos.x.rem_euclid(types::CHUNK_SIZE) as usize,
            (pos.y - types::WORLD_Y_MIN) as usize,
            pos.z.rem_euclid(types::CHUNK_SIZE) as usize,
        )
    }

    /// 应用一个事件(语义见模块文档)。
    pub fn apply(&mut self, event: ViewEvent) {
        match event {
            ViewEvent::Welcome { tick, .. } => self.tick = tick.0,
            ViewEvent::Chunk { pos, chunk } => {
                self.chunks.insert(pos, chunk);
                self.dirty.insert(pos);
            }
            ViewEvent::ChunkGone { pos } => {
                self.chunks.remove(&pos);
                self.dirty.insert(pos);
            }
            ViewEvent::Entities {
                tick,
                states,
                removed,
                full,
            } => {
                self.tick = tick.0;
                if full {
                    // 全量对账:以 states 为准重建实体表(协议 full 同语义)。
                    self.entities.clear();
                }
                for (id, state) in states {
                    self.entities.insert(id, state.translation);
                }
                for id in removed {
                    self.entities.remove(&id);
                }
            }
            ViewEvent::Blocks { updates } => {
                // 只改已装载 chunk 的镜像;未装载忽略(不造假 chunk)。
                for (pos, block) in updates {
                    let cpos = pos.chunk_of();
                    let Some(chunk) = self.chunks.get_mut(&cpos) else {
                        continue;
                    };
                    let (lx, ly, lz) = Self::local(pos);
                    chunk.set(lx, ly, lz, block);
                    self.dirty.insert(cpos);
                }
            }
            ViewEvent::Spawned { entity } => self.spawned.push(entity),
            ViewEvent::Disconnected { .. } => {}
        }
    }

    /// 最新服务器拍号。
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// 某个已装载 chunk 的方块镜像。
    pub fn chunk(&self, pos: ChunkPos) -> Option<&voxel::Chunk> {
        self.chunks.get(&pos)
    }

    /// 取走自上次调用以来变"脏"的 chunk(升序、去重)。
    pub fn take_dirty_chunks(&mut self) -> Vec<ChunkPos> {
        let taken = std::mem::take(&mut self.dirty);
        taken.into_iter().collect()
    }

    /// 取走自上次调用以来收到的 Spawned 回执(按到达顺序)。
    pub fn take_spawned(&mut self) -> Vec<EntityId> {
        std::mem::take(&mut self.spawned)
    }
}

impl WorldQuery for WorldState {
    fn live_entities(&self) -> Vec<(EntityId, Vec3)> {
        self.entities.iter().map(|(id, p)| (*id, *p)).collect()
    }

    fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
        let (lx, ly, lz) = Self::local(pos);
        if ly >= voxel::CHUNK_Y {
            return None;
        }
        self.chunks
            .get(&pos.chunk_of())
            .map(|chunk| chunk.get(lx, ly, lz))
    }

    fn chunks_loaded(&self) -> usize {
        self.chunks.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use messaging::InputSource;
    use types::{EntityState, Tick};

    fn block_at_pos(w: &WorldState, pos: BlockPos) -> Option<BlockId> {
        w.block_at(pos)
    }

    /// 未装载 chunk 的增量被忽略,不虚增 chunks_loaded、不造假可查格(锚定模块
    /// 文档"未装载忽略";若实现造出假 chunk,`chunks_loaded` 与 `block_at` 断言红)。
    #[test]
    fn world_blocks_update_ignores_unloaded_chunks() {
        let mut w = WorldState::new();
        assert_eq!(w.chunks_loaded(), 0);
        assert_eq!(block_at_pos(&w, BlockPos::new(0, -61, 0)), None);
        w.apply(ViewEvent::Blocks {
            updates: vec![(BlockPos::new(0, -61, 0), BlockId(1))],
        });
        assert_eq!(w.chunks_loaded(), 0, "未装载 chunk 的增量不造假 chunk");
        assert_eq!(block_at_pos(&w, BlockPos::new(0, -61, 0)), None);
    }

    /// 负坐标换算:世界 (−5, −61, 40) → chunk (−1, 2) 局部 (11, 3, 8)(rem_euclid
    /// 语义;若实现用向零截断,读 −6 列时错位,本测试红)。
    #[test]
    fn world_block_at_maps_negative_coords() {
        let mut w = WorldState::new();
        let west = ChunkPos::new(-1, 2);
        let mut chunk = voxel::Chunk::air();
        chunk.set(11, 3, 8, BlockId(2));
        w.apply(ViewEvent::Chunk { pos: west, chunk });
        assert_eq!(w.chunks_loaded(), 1);
        assert_eq!(
            block_at_pos(&w, BlockPos::new(-5, -61, 40)),
            Some(BlockId(2))
        );
        assert_eq!(
            block_at_pos(&w, BlockPos::new(-6, -61, 40)),
            Some(BlockId::AIR)
        );
        // 超出世界 y 高度:不可查(防 y 越界 panic / 误读)。
        assert_eq!(block_at_pos(&w, BlockPos::new(-5, 400, 40)), None);
    }

    /// 脏集合:Chunk / ChunkGone / Blocks 各自记脏;升序去重;取走即清。
    #[test]
    fn world_dirty_chunks_are_ordered_deduped_and_cleared() {
        let mut w = WorldState::new();
        let a = ChunkPos::new(1, 0);
        let b = ChunkPos::new(-2, 3);
        w.apply(ViewEvent::Chunk {
            pos: a,
            chunk: voxel::Chunk::air(),
        });
        w.apply(ViewEvent::Chunk {
            pos: b,
            chunk: voxel::Chunk::air(),
        });
        w.apply(ViewEvent::Blocks {
            updates: vec![(BlockPos::new(20, -61, 0), BlockId(1))], // chunk a 内
        });
        w.apply(ViewEvent::ChunkGone { pos: a });
        assert_eq!(w.take_dirty_chunks(), vec![b, a], "升序、去重(a 两脏合一)");
        assert!(w.take_dirty_chunks().is_empty(), "取走即清");
    }

    /// 实体表与拍号:增量 latest-wins、removed 删除、full 重建;Welcome/Entities
    /// 更新拍号;Spawned 回执按到达顺序取走;Disconnected 不改任何状态。
    #[test]
    fn world_entities_ticks_and_receipts_follow_events() {
        let mut w = WorldState::new();
        w.apply(ViewEvent::Welcome {
            player_id: InputSource(9),
            tick: Tick(3),
        });
        assert_eq!(w.tick(), 3);
        let p1 = Vec3::new(1.0, 2.0, 3.0);
        let p2 = Vec3::new(4.0, 5.0, 6.0);
        w.apply(ViewEvent::Entities {
            tick: Tick(8),
            states: vec![(
                EntityId::from_parts(0, 1),
                EntityState {
                    translation: p1,
                    ..EntityState::default()
                },
            )],
            removed: vec![],
            full: false,
        });
        w.apply(ViewEvent::Entities {
            tick: Tick(12),
            states: vec![(
                EntityId::from_parts(0, 1),
                EntityState {
                    translation: p2,
                    ..EntityState::default()
                },
            )],
            removed: vec![],
            full: false,
        });
        assert_eq!(w.tick(), 12, "Entities 包推进拍号");
        assert_eq!(
            w.live_entities(),
            vec![(EntityId::from_parts(0, 1), p2)],
            "latest-wins"
        );

        w.apply(ViewEvent::Entities {
            tick: Tick(20),
            states: vec![(
                EntityId::from_parts(0, 7),
                EntityState {
                    translation: p1,
                    ..EntityState::default()
                },
            )],
            removed: vec![EntityId::from_parts(0, 1)],
            full: false,
        });
        assert_eq!(
            w.live_entities(),
            vec![(EntityId::from_parts(0, 7), p1)],
            "removed 删除"
        );

        w.apply(ViewEvent::Entities {
            tick: Tick(30),
            states: vec![],
            removed: vec![],
            full: true,
        });
        assert!(w.live_entities().is_empty(), "full 清空实体表(全量对账)");
        assert_eq!(w.tick(), 30);

        w.apply(ViewEvent::Spawned {
            entity: EntityId::from_parts(0, 2),
        });
        w.apply(ViewEvent::Spawned {
            entity: EntityId::from_parts(0, 1),
        });
        assert_eq!(
            w.take_spawned(),
            vec![EntityId::from_parts(0, 2), EntityId::from_parts(0, 1)],
            "回执按到达顺序"
        );
        assert!(w.take_spawned().is_empty(), "取走即清");

        let entities_before = w.live_entities();
        let tick_before = w.tick();
        w.apply(ViewEvent::Disconnected {
            reason: "bye".to_owned(),
        });
        assert_eq!(w.tick(), tick_before, "断连不改拍号");
        assert_eq!(w.live_entities(), entities_before, "断连不改实体表");
    }
}
