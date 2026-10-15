//! 批量编辑与脏 chunk 标记。

use std::collections::{BTreeMap, BTreeSet};

use types::{BlockId, BlockPos, ChunkPos, WORLD_Y_MAX, WORLD_Y_MIN};

use crate::world::{VoxelError, VoxelWorld};

/// 一批方块写入。两阶段应用:先全量校验(y 界内),再写入——要么全成、要么世界不动。
#[derive(Debug, Default)]
pub struct VoxelEdit {
    sets: BTreeMap<BlockPos, BlockId>,
}

impl VoxelEdit {
    pub fn new() -> Self {
        Self::default()
    }

    /// 追加一次写入(链式)。
    pub fn set(mut self, pos: BlockPos, block: BlockId) -> Self {
        self.sets.insert(pos, block);
        self
    }

    pub fn len(&self) -> usize {
        self.sets.len()
    }

    pub fn is_empty(&self) -> bool {
        self.sets.is_empty()
    }

    /// 应用到世界,返回受影响(脏)chunk 集——collider 重建的输入(蓝图 §3)。
    /// 两阶段:先全量校验(只读检查,不碰世界),再写入(原子性)。
    pub fn apply(self, world: &mut VoxelWorld) -> Result<BTreeSet<ChunkPos>, VoxelError> {
        for pos in self.sets.keys() {
            if !(WORLD_Y_MIN..=WORLD_Y_MAX).contains(&pos.y) {
                return Err(VoxelError::YOutOfBounds { y: pos.y });
            }
        }
        let mut dirty = BTreeSet::new();
        for (pos, block) in &self.sets {
            world
                .set_block(*pos, *block)
                .expect("校验过的写入不可能越界");
            dirty.insert(pos.chunk_of());
        }
        Ok(dirty)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STONE: BlockId = BlockId(1);

    #[test]
    fn apply_reports_dirty_chunks_across_boundaries() {
        let mut world = VoxelWorld::new();
        let dirty = VoxelEdit::new()
            .set(BlockPos::new(0, 0, 0), STONE)
            .set(BlockPos::new(15, 0, 0), STONE)
            .set(BlockPos::new(16, 0, 0), STONE)
            .set(BlockPos::new(100, 50, -100), STONE)
            .apply(&mut world)
            .unwrap();
        // (0,0,0) 与 (15,0,0) 同 chunk;(16,0,0) 与 (100,-100) 各成一片
        assert_eq!(
            dirty,
            BTreeSet::from([
                ChunkPos::new(0, 0),
                ChunkPos::new(1, 0),
                ChunkPos::new(6, -7)
            ])
        );
        assert_eq!(world.get_block(BlockPos::new(16, 0, 0)), STONE);
        assert_eq!(world.get_block(BlockPos::new(100, 50, -100)), STONE);
    }

    #[test]
    fn apply_is_atomic_on_out_of_range() {
        let mut world = VoxelWorld::new();
        let result = VoxelEdit::new()
            .set(BlockPos::new(0, 0, 0), STONE)
            .set(BlockPos::new(0, 999, 0), STONE)
            .apply(&mut world);
        assert!(matches!(result, Err(VoxelError::YOutOfBounds { y: 999 })));
        // 原子性:合法的那条也不能落盘
        assert_eq!(world.get_block(BlockPos::new(0, 0, 0)), BlockId::AIR);
        assert_eq!(world.chunk_count(), 0);
    }

    #[test]
    fn empty_edit_is_noop() {
        let mut world = VoxelWorld::new();
        let dirty = VoxelEdit::new().apply(&mut world).unwrap();
        assert!(dirty.is_empty());
        assert_eq!(world.chunk_count(), 0);
    }
}
