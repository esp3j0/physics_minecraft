//! # types —— 跨 crate 类型地基(L0)
//!
//! 职责:全项目共享的标识(EntityId/ShardId/BlockId)、坐标(BlockPos/ChunkPos/
//! ShardPos)、逻辑时钟(Tick)、实体迁移载荷(EntityState);`glam` 数学库的唯一
//! 转发入口(其他 crate 一律 `use types::glam`)。
//!
//! 依赖边界:仅依赖 `glam`;不含任何业务逻辑、IO、时钟、随机数。
//! 禁止:其他 crate 重复定义这些类型(单一事实源,蓝图 §0)。
//!
//! 图纸:[docs/design/codebase-layout.md §3 types](../../docs/design/codebase-layout.md)

pub mod coords;
pub mod ids;
pub mod state;
pub mod tick;

pub use coords::{BlockPos, CHUNK_SIZE, ChunkPos, ChunkRegion, WORLD_Y_MAX, WORLD_Y_MIN};
pub use ids::{
    BlockId, DimensionId, EMERGENT_ORIGIN, EntityId, SHARD_CHUNKS, ShardId, ShardPos, WorldId,
    WorldSeed,
};
pub use state::{EntityDesc, EntityState};
pub use tick::Tick;

/// 数学库唯一转发入口(蓝图 §2):其他 crate `use types::glam`。
pub use glam;
