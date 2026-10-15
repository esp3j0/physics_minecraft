//! chunk 存储(扁平数组起步;palette 压缩后置,蓝图 §8)。
//! 局部坐标:x/z ∈ [0,16),y ∈ [0,384)(世界 y + 64)。

use serde::{Deserialize, Serialize};
use types::BlockId;

pub const CHUNK_X: usize = 16;
pub const CHUNK_Z: usize = 16;
/// 全高 384:世界 y ∈ [-64, 319](主世界尺寸,types::WORLD_Y_MIN/MAX)。
pub const CHUNK_Y: usize = 384;

/// 单个 chunk 的方块存储。y-major 布局:`(y * CHUNK_Z + z) * CHUNK_X + x`。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Chunk {
    blocks: Vec<BlockId>,
}

impl Chunk {
    /// 全部填充为同一方块。
    pub fn filled(block: BlockId) -> Self {
        Self {
            blocks: vec![block; CHUNK_X * CHUNK_Y * CHUNK_Z],
        }
    }

    /// 全空气 chunk。
    pub fn air() -> Self {
        Self::filled(BlockId::AIR)
    }

    #[inline]
    pub fn get(&self, x: usize, y: usize, z: usize) -> BlockId {
        self.blocks[Self::index(x, y, z)]
    }

    #[inline]
    pub fn set(&mut self, x: usize, y: usize, z: usize, block: BlockId) {
        let i = Self::index(x, y, z);
        self.blocks[i] = block;
    }

    #[inline]
    fn index(x: usize, y: usize, z: usize) -> usize {
        debug_assert!(
            x < CHUNK_X && y < CHUNK_Y && z < CHUNK_Z,
            "chunk 局部坐标越界"
        );
        (y * CHUNK_Z + z) * CHUNK_X + x
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STONE: BlockId = BlockId(1);

    #[test]
    fn chunk_height_matches_world_constants() {
        assert_eq!(CHUNK_Y, 384);
        assert_eq!(CHUNK_X, 16);
        assert_eq!(CHUNK_Z, 16);
    }

    #[test]
    fn filled_get_set_roundtrip() {
        let mut chunk = Chunk::air();
        assert_eq!(chunk.get(0, 0, 0), BlockId::AIR);
        chunk.set(3, 100, 7, STONE);
        assert_eq!(chunk.get(3, 100, 7), STONE);
        // 最高角
        chunk.set(CHUNK_X - 1, CHUNK_Y - 1, CHUNK_Z - 1, STONE);
        assert_eq!(chunk.get(CHUNK_X - 1, CHUNK_Y - 1, CHUNK_Z - 1), STONE);
        // 相邻格不受影响
        assert_eq!(chunk.get(4, 100, 7), BlockId::AIR);
    }

    #[test]
    fn air_equals_filled_air() {
        assert_eq!(Chunk::air(), Chunk::filled(BlockId::AIR));
    }

    #[test]
    fn serde_roundtrip() {
        let mut chunk = Chunk::air();
        chunk.set(1, 2, 3, STONE);
        let s = serde_json::to_string(&chunk).unwrap();
        let back: Chunk = serde_json::from_str(&s).unwrap();
        assert_eq!(chunk, back);
    }
}
