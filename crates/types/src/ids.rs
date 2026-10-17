//! 标识类型:方块、世界、维度、分片、实体。
//! 单一事实源:其他 crate 一律 re-export,禁止重复定义(蓝图 §0)。

use serde::{Deserialize, Serialize};

/// 方块 ID。`BlockId::AIR`(0)= 空气;注册表保证 0 号恒为空气(voxel::block)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct BlockId(pub u16);

impl BlockId {
    pub const AIR: Self = Self(0);

    pub fn is_air(self) -> bool {
        self == Self::AIR
    }
}

/// 世界 ID(多世界支持,当前恒为默认世界)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct WorldId(pub u32);

/// 世界生成种子(REQ-002 §4.1):同 `(seed, pos)` 必须位级确定生成(AC-G1)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct WorldSeed(pub u64);

/// 维度 ID(主世界/下界/末地,含义由游戏层定义)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct DimensionId(pub u32);

/// 分片坐标:x/z 为 chunk 网格坐标 / 8(8×8 chunk 一片,REQ-001 §4.1)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ShardPos {
    pub x: i32,
    pub z: i32,
}

impl ShardPos {
    pub const fn new(x: i32, z: i32) -> Self {
        Self { x, z }
    }
}

/// 分片在(世界, 维度)网格上的唯一身份(借 v2 ShardId,REQ-001 §4.1)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ShardId {
    pub world: WorldId,
    pub dimension: DimensionId,
    pub pos: ShardPos,
}

impl ShardId {
    pub const fn new(world: WorldId, dimension: DimensionId, pos: ShardPos) -> Self {
        Self {
            world,
            dimension,
            pos,
        }
    }
}

/// 一个分片一条边的 chunk 数(8×8 分片,REQ-001 §4.1)。
pub const SHARD_CHUNKS: i32 = 8;

/// 实体 ID(REQ-001 §9.7):`(origin_shard 序号, local_seq)` 打包 u64,单调不回收,
/// 迁移后不变。origin 是分片的 **u32 序号**(由 shard 注册表分配),不是完整 ShardId。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct EntityId {
    bits: u64,
}

impl EntityId {
    pub fn from_parts(origin_shard: u32, local_seq: u32) -> Self {
        Self {
            bits: (u64::from(origin_shard) << 32) | u64::from(local_seq),
        }
    }

    pub fn origin_shard(self) -> u32 {
        (self.bits >> 32) as u32
    }

    pub fn local_seq(self) -> u32 {
        self.bits as u32
    }

    pub fn to_bits(self) -> u64 {
        self.bits
    }

    pub fn from_bits(bits: u64) -> Self {
        Self { bits }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn block_air_is_zero_and_predicate() {
        assert_eq!(BlockId::AIR, BlockId(0));
        assert!(BlockId::AIR.is_air());
        assert!(!BlockId(7).is_air());
    }

    #[test]
    fn shard_id_composite_identity() {
        let a = ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(1, 2));
        let b = ShardId::new(WorldId(1), DimensionId(0), ShardPos::new(1, 2));
        assert_ne!(a, b); // 世界不同 → 身份不同
        assert_eq!(
            a,
            ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(1, 2))
        );
    }

    #[test]
    fn entity_id_packing_roundtrip() {
        let id = EntityId::from_parts(0x0000_00ff, 0x0000_0042);
        assert_eq!(id.origin_shard(), 0xff);
        assert_eq!(id.local_seq(), 0x42);
        assert_eq!(EntityId::from_bits(id.to_bits()), id);
    }

    #[test]
    fn entity_id_ordering_follows_origin_then_seq() {
        let a = EntityId::from_parts(1, 99);
        let b = EntityId::from_parts(2, 0);
        let c = EntityId::from_parts(1, 100);
        assert!(a < c && c < b); // origin 在高位,先比 origin 再比 seq
    }
}
