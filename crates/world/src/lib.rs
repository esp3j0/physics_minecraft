//! 方块世界模块:方块、区块与(将来的)存档。
//!
//! 区块/协议/存档的实现思路重点参考:
//! - [pumpkin](https://github.com/Pumpkin-MC/pumpkin)(本地在 `references/pumpkin`)
//! - [ferrumc](https://github.com/ferrumc-rs/ferrumc)(本地在 `references/ferrumc`,主要看 v2)

/// 方块 ID(占位,后续接入注册表)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockId(pub u16);

/// 空气。
pub const AIR: BlockId = BlockId(0);

pub const CHUNK_X: usize = 16;
pub const CHUNK_Z: usize = 16;
/// 与原版主世界一致:384 格高(y ∈ -64..=320)。
pub const CHUNK_Y: usize = 384;

/// 用扁平数组存储的区块(占位实现)。
#[derive(Debug, Clone)]
pub struct Chunk {
    blocks: Vec<BlockId>,
}

impl Chunk {
    pub fn filled(block: BlockId) -> Self {
        Self {
            blocks: vec![block; CHUNK_X * CHUNK_Y * CHUNK_Z],
        }
    }

    pub fn get(&self, x: usize, y: usize, z: usize) -> BlockId {
        self.blocks[self.index(x, y, z)]
    }

    pub fn set(&mut self, x: usize, y: usize, z: usize, block: BlockId) {
        let i = self.index(x, y, z);
        self.blocks[i] = block;
    }

    /// y-major 布局;接 palette/压缩存储时再优化。
    fn index(&self, x: usize, y: usize, z: usize) -> usize {
        debug_assert!(x < CHUNK_X && y < CHUNK_Y && z < CHUNK_Z);
        (y * CHUNK_Z + z) * CHUNK_X + x
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_then_get_roundtrip() {
        let mut chunk = Chunk::filled(AIR);
        let stone = BlockId(1);
        chunk.set(3, 100, 7, stone);
        assert_eq!(chunk.get(3, 100, 7), stone);
        assert_eq!(chunk.get(0, 0, 0), AIR);
    }
}
