//! tests_support —— tools 测试与 e2e 自查共用的最小体素世界(纯逻辑,
//! 零 macroquad;不进任何运行路径)。

use std::collections::BTreeMap;

use types::glam::Vec3;
use types::{BlockId, BlockPos, CHUNK_SIZE, ChunkPos, WORLD_Y_MAX, WORLD_Y_MIN, WorldSeed};
use voxel::{Chunk, Superflat, WorldGenerator};

use crate::script::WorldQuery;

/// 测试世界:列出的 chunk 生成超平坦(可再用 blocks 覆写单格;覆写所在
/// chunk 不在列表里时造一个空气 chunk 承接)。未列出的 chunk = 未装载。
pub struct FakeVoxelWorld {
    chunks: BTreeMap<ChunkPos, Chunk>,
}

impl FakeVoxelWorld {
    /// 纯空气世界(无任何装载 chunk;block_at 恒 None)。
    #[must_use]
    pub fn empty() -> Self {
        Self {
            chunks: BTreeMap::new(),
        }
    }

    /// 九宫格超平坦(chunk (−1,−1)..(1,1) 全装载)+ 补写格。
    #[must_use]
    pub fn superflat(blocks: &[(i32, i32, i32, BlockId)]) -> Self {
        let mut loaded = Vec::new();
        for cx in -1..=1 {
            for cz in -1..=1 {
                loaded.push(ChunkPos::new(cx, cz));
            }
        }
        Self::new(&loaded, blocks)
    }

    /// 指定装载 chunk + 补写格。
    #[must_use]
    pub fn new(loaded: &[ChunkPos], blocks: &[(i32, i32, i32, BlockId)]) -> Self {
        let mut w = Self {
            chunks: BTreeMap::new(),
        };
        for &pos in loaded {
            w.chunks
                .insert(pos, Superflat::classic().generate(WorldSeed(0), pos));
        }
        for &(x, y, z, b) in blocks {
            w.set_block(x, y, z, b);
        }
        w
    }

    /// 补写一格(所在 chunk 未装载则造空气 chunk 承接)。
    pub fn set_block(&mut self, x: i32, y: i32, z: i32, b: BlockId) {
        let pos = BlockPos::new(x, y, z).chunk_of();
        let chunk = self.chunks.entry(pos).or_insert_with(Chunk::air);
        chunk.set(
            x.rem_euclid(CHUNK_SIZE) as usize,
            (y - WORLD_Y_MIN) as usize,
            z.rem_euclid(CHUNK_SIZE) as usize,
            b,
        );
    }
}

impl WorldQuery for FakeVoxelWorld {
    fn live_entities(&self) -> Vec<(types::EntityId, Vec3)> {
        vec![]
    }

    fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
        if pos.y < WORLD_Y_MIN || pos.y > WORLD_Y_MAX {
            return None;
        }
        self.chunks.get(&pos.chunk_of()).map(|c| {
            c.get(
                pos.x.rem_euclid(CHUNK_SIZE) as usize,
                (pos.y - WORLD_Y_MIN) as usize,
                pos.z.rem_euclid(CHUNK_SIZE) as usize,
            )
        })
    }

    fn chunks_loaded(&self) -> usize {
        self.chunks.len()
    }
}
