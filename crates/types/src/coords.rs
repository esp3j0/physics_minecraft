//! 世界锚定坐标与换算。体素约定(与 demo C 一致):格 (x,y,z) 覆盖 [x,x+1)³,
//! 即格点在整数坐标、体素中心在 (x+0.5, y+0.5, z+0.5)。

use serde::{Deserialize, Serialize};

use crate::ids::{SHARD_CHUNKS, ShardId, ShardPos};

/// chunk 一条边的方块数。
pub const CHUNK_SIZE: i32 = 16;
/// 世界最低 y(含)。chunk 全高覆盖 [-64, 319] 共 384 格(主世界尺寸)。
pub const WORLD_Y_MIN: i32 = -64;
/// 世界最高 y(含)。
pub const WORLD_Y_MAX: i32 = 319;

/// 方块坐标(世界锚定)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct BlockPos {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl BlockPos {
    pub const ZERO: Self = Self { x: 0, y: 0, z: 0 };

    pub const fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    /// 所在 chunk。负方向向下取整:-1 号方块属于 chunk -1(Rust `/` 向零截断,不可直接用)。
    pub fn chunk_of(self) -> ChunkPos {
        ChunkPos::new(floor_div(self.x, CHUNK_SIZE), floor_div(self.z, CHUNK_SIZE))
    }
}

/// chunk 网格坐标(x/z 方向;y 由 chunk 全高覆盖)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ChunkPos {
    pub x: i32,
    pub z: i32,
}

impl ChunkPos {
    pub const fn new(x: i32, z: i32) -> Self {
        Self { x, z }
    }

    /// 本 chunk 最小角的方块坐标(y 取世界最低)。
    pub fn block_min(self) -> BlockPos {
        BlockPos::new(self.x * CHUNK_SIZE, WORLD_Y_MIN, self.z * CHUNK_SIZE)
    }

    /// 所属分片(8×8 chunk 一片)。
    pub fn shard_of(self) -> ShardPos {
        ShardPos::new(
            floor_div(self.x, SHARD_CHUNKS),
            floor_div(self.z, SHARD_CHUNKS),
        )
    }
}

impl ShardPos {
    /// 分片最小角的 chunk 坐标(shard << 3)。
    pub fn min_chunk(self) -> ChunkPos {
        ChunkPos::new(self.x * SHARD_CHUNKS, self.z * SHARD_CHUNKS)
    }
}

/// 向下取整除法:坐标→网格换算统一入口。
pub(crate) fn floor_div(a: i32, b: i32) -> i32 {
    let q = a / b;
    if a % b != 0 && ((a < 0) != (b < 0)) {
        q - 1
    } else {
        q
    }
}

/// chunk 网格的闭区间矩形(单位:chunk;REQ-005 §4.1,D2/D4)。
/// 分片的静态范围 = `core(shard).expanded(region_margin_chunks)`。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ChunkRegion {
    pub min: ChunkPos,
    pub max: ChunkPos,
}

impl ChunkRegion {
    /// 分片本体区:8×8 chunk,闭区间(负方向 floor 语义由 `min_chunk` 保证)。
    pub fn core(shard: ShardId) -> Self {
        let min = shard.pos.min_chunk();
        Self {
            min,
            max: ChunkPos::new(min.x + SHARD_CHUNKS - 1, min.z + SHARD_CHUNKS - 1),
        }
    }

    /// 向外扩 margin 个 chunk(四边都扩;0 恒等,负值允许收缩,测试用)。
    pub fn expanded(self, margin: i32) -> Self {
        Self {
            min: ChunkPos::new(self.min.x - margin, self.min.z - margin),
            max: ChunkPos::new(self.max.x + margin, self.max.z + margin),
        }
    }

    /// 闭区间包含判定。
    pub fn contains(&self, pos: ChunkPos) -> bool {
        pos.x >= self.min.x && pos.x <= self.max.x && pos.z >= self.min.z && pos.z <= self.max.z
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floor_div_negative() {
        assert_eq!(floor_div(-1, 16), -1);
        assert_eq!(floor_div(-16, 16), -1);
        assert_eq!(floor_div(-17, 16), -2);
        assert_eq!(floor_div(15, 16), 0);
        assert_eq!(floor_div(16, 16), 1);
        assert_eq!(floor_div(-31, 8), -4);
    }

    #[test]
    fn block_pos_chunk_of_boundary() {
        assert_eq!(BlockPos::new(0, 0, 0).chunk_of(), ChunkPos::new(0, 0));
        assert_eq!(BlockPos::new(15, 319, 15).chunk_of(), ChunkPos::new(0, 0));
        assert_eq!(BlockPos::new(16, 0, 0).chunk_of(), ChunkPos::new(1, 0));
        assert_eq!(BlockPos::new(-1, 0, 0).chunk_of(), ChunkPos::new(-1, 0));
        assert_eq!(BlockPos::new(-16, 0, -1).chunk_of(), ChunkPos::new(-1, -1));
        assert_eq!(BlockPos::new(-17, 0, 0).chunk_of(), ChunkPos::new(-2, 0));
    }

    #[test]
    fn chunk_block_min_roundtrip() {
        let cp = ChunkPos::new(-2, 3);
        let min = cp.block_min();
        assert_eq!(min, BlockPos::new(-32, WORLD_Y_MIN, 48));
        assert_eq!(min.chunk_of(), cp);
    }

    #[test]
    fn chunk_shard_roundtrip() {
        let sp = ShardPos::new(-1, 1);
        assert_eq!(sp.min_chunk(), ChunkPos::new(-8, 8));
        // 分片 (-1,1) 恰含 chunk x ∈ [-8,-1](负坐标边界);
        // x ∈ [0,7] 属于下一个分片 (0,1)
        for x in -8..=-1 {
            assert_eq!(ChunkPos::new(x, 8).shard_of(), sp);
        }
        for x in 0..=7 {
            assert_eq!(ChunkPos::new(x, 8).shard_of(), ShardPos::new(0, 1));
        }
        assert_eq!(ChunkPos::new(-1, 8).shard_of(), sp);
    }

    /// 任务书 req005/00 测试 1:core 8×8;expanded(1) 四边各扩 1;contains
    /// 边界正确(闭区间)。
    #[test]
    fn chunk_region_core_and_margin() {
        let shard = ShardId::new(
            crate::WorldId(0),
            crate::DimensionId(0),
            ShardPos::new(0, 0),
        );
        let core = ChunkRegion::core(shard);
        // 8×8 闭区间:min = (0,0),max = (7,7)。
        assert_eq!(core.min, ChunkPos::new(0, 0));
        assert_eq!(core.max, ChunkPos::new(7, 7));
        assert!(core.contains(ChunkPos::new(0, 0)));
        assert!(core.contains(ChunkPos::new(7, 7)));
        assert!(!core.contains(ChunkPos::new(8, 0)), "x=8 属邻片本体区");
        assert!(!core.contains(ChunkPos::new(-1, 0)));

        // 负方向分片:floor 语义。
        let west = ChunkRegion::core(ShardId::new(
            crate::WorldId(0),
            crate::DimensionId(0),
            ShardPos::new(-1, 0),
        ));
        assert_eq!(west.min, ChunkPos::new(-8, 0));
        assert_eq!(west.max, ChunkPos::new(-1, 7), "8×8 方形:x 与 z 都扩 7");

        // expanded(1):四边各扩 1 → 10×10。
        let ring = core.expanded(1);
        assert_eq!(ring.min, ChunkPos::new(-1, -1));
        assert_eq!(ring.max, ChunkPos::new(8, 8));
        assert!(ring.contains(ChunkPos::new(8, 0)), "扩张环含邻片第 1 列");
        assert!(!ring.contains(ChunkPos::new(9, 0)), "扩张环不含第 2 列");
        // expanded(0) 恒等。
        assert_eq!(core.expanded(0), core);
    }
}
