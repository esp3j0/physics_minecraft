//! # voxel —— 体素世界唯一真相(L0)
//!
//! 职责:chunk 存储、方块注册表、批量编辑与脏 chunk 标记。
//! 体素数组是全项目唯一真相(REQ-001 共识 2):物理 collider、渲染、网络同步
//! 全部是它的派生缓存。
//!
//! 依赖边界:仅依赖 `types`;不含物理概念、渲染概念。
//! 禁止:IO、时钟、随机数、线程(确定性核纪律,蓝图 §4)。
//!
//! 图纸:[docs/design/codebase-layout.md §3 voxel](../../docs/design/codebase-layout.md)

mod block;
mod chunk;
mod edit;
mod generator;
mod world;

pub use block::{
    BOND_INFINITE, BlockRegistry, Material, SimpleRegistry, SimpleRegistryError, Support, builtin,
};
pub use chunk::{CHUNK_X, CHUNK_Y, CHUNK_Z, Chunk};
pub use edit::VoxelEdit;
pub use generator::{SeededScatter, Superflat, WorldGenerator};
pub use world::{VoxelError, VoxelWorld};

/// 便捷 re-export(单一事实源在 types,蓝图 §0)。
pub use types::BlockId;
