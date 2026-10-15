//! 体素世界:chunk 内存表(BTreeMap,确定性迭代)。

use std::collections::BTreeMap;

use types::{BlockId, BlockPos, ChunkPos, WORLD_Y_MAX, WORLD_Y_MIN};

use crate::chunk::Chunk;

/// 可预期的世界层错误(一致性违规直接 panic,不走这里,蓝图 §4)。
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum VoxelError {
    #[error("y {y} 越界,有效范围 [{}, {}]", WORLD_Y_MIN, WORLD_Y_MAX)]
    YOutOfBounds { y: i32 },
}

/// 世界锚定的体素世界。水平方向无限(按需建 chunk),y 限 [-64, 319]。
#[derive(Debug, Default)]
pub struct VoxelWorld {
    chunks: BTreeMap<ChunkPos, Chunk>,
}

impl VoxelWorld {
    pub fn new() -> Self {
        Self::default()
    }

    /// 读方块:未加载 chunk 或 y 界外 → AIR(界外即空气,查询永不失败)。
    pub fn get_block(&self, pos: BlockPos) -> BlockId {
        if !(WORLD_Y_MIN..=WORLD_Y_MAX).contains(&pos.y) {
            return BlockId::AIR;
        }
        let cp = pos.chunk_of();
        let Some(chunk) = self.chunks.get(&cp) else {
            return BlockId::AIR;
        };
        let min = cp.block_min();
        chunk.get(
            (pos.x - min.x) as usize,
            (pos.y - WORLD_Y_MIN) as usize,
            (pos.z - min.z) as usize,
        )
    }

    /// 写方块:y 界外返回 Err;所在 chunk 未加载时自动创建。
    pub fn set_block(&mut self, pos: BlockPos, block: BlockId) -> Result<(), VoxelError> {
        if !(WORLD_Y_MIN..=WORLD_Y_MAX).contains(&pos.y) {
            return Err(VoxelError::YOutOfBounds { y: pos.y });
        }
        let cp = pos.chunk_of();
        let min = cp.block_min();
        let chunk = self.ensure_chunk(cp);
        chunk.set(
            (pos.x - min.x) as usize,
            (pos.y - WORLD_Y_MIN) as usize,
            (pos.z - min.z) as usize,
            block,
        );
        Ok(())
    }

    pub fn get_chunk(&self, cp: ChunkPos) -> Option<&Chunk> {
        self.chunks.get(&cp)
    }

    /// 取 chunk,不存在则创建全空气 chunk。
    pub fn ensure_chunk(&mut self, cp: ChunkPos) -> &mut Chunk {
        self.chunks.entry(cp).or_insert_with(Chunk::air)
    }

    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }

    /// 已加载 chunk 坐标,按 ChunkPos 升序(BTreeMap 确定性迭代,蓝图 §4)。
    pub fn chunk_positions(&self) -> impl Iterator<Item = ChunkPos> + '_ {
        self.chunks.keys().copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::WORLD_Y_MIN;

    const STONE: BlockId = BlockId(1);

    #[test]
    fn set_get_roundtrip_and_air_default() {
        let mut world = VoxelWorld::new();
        let pos = BlockPos::new(3, 0, 5);
        world.set_block(pos, STONE).unwrap();
        assert_eq!(world.get_block(pos), STONE);
        assert_eq!(world.get_block(BlockPos::new(4, 0, 5)), BlockId::AIR);
    }

    #[test]
    fn world_y_bounds() {
        let mut world = VoxelWorld::new();
        world
            .set_block(BlockPos::new(0, WORLD_Y_MIN, 0), STONE)
            .unwrap();
        world
            .set_block(BlockPos::new(0, WORLD_Y_MAX, 0), STONE)
            .unwrap();
        assert_eq!(
            world.set_block(BlockPos::new(0, WORLD_Y_MAX + 1, 0), STONE),
            Err(VoxelError::YOutOfBounds { y: WORLD_Y_MAX + 1 })
        );
        assert_eq!(
            world.set_block(BlockPos::new(0, WORLD_Y_MIN - 1, 0), STONE),
            Err(VoxelError::YOutOfBounds { y: WORLD_Y_MIN - 1 })
        );
    }

    #[test]
    fn get_outside_or_missing_returns_air() {
        let world = VoxelWorld::new();
        // y 界外
        assert_eq!(world.get_block(BlockPos::new(0, 400, 0)), BlockId::AIR);
        // 水平远处的未加载 chunk
        assert_eq!(
            world.get_block(BlockPos::new(99999, 0, -99999)),
            BlockId::AIR
        );
    }

    #[test]
    fn set_across_chunk_boundary_auto_creates() {
        let mut world = VoxelWorld::new();
        world.set_block(BlockPos::new(15, 0, 0), STONE).unwrap();
        world.set_block(BlockPos::new(16, 0, 0), STONE).unwrap();
        world.set_block(BlockPos::new(-1, 0, 0), STONE).unwrap();
        assert_eq!(world.chunk_count(), 3);
        assert_eq!(world.get_block(BlockPos::new(16, 0, 0)), STONE);
        assert_eq!(world.get_block(BlockPos::new(-1, 0, 0)), STONE);
    }

    #[test]
    fn chunk_positions_are_deterministically_ordered() {
        let mut world = VoxelWorld::new();
        // 乱序写入
        for cp in [(2, 2), (-1, 0), (0, 0), (0, 2), (-3, -3)] {
            world.ensure_chunk(ChunkPos::new(cp.0, cp.1));
        }
        let positions: Vec<_> = world.chunk_positions().collect();
        let mut sorted = positions.clone();
        sorted.sort();
        assert_eq!(positions, sorted, "BTreeMap 迭代必须升序");
        assert_eq!(positions.len(), 5);
    }
}
