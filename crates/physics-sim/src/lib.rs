//! # physics-sim —— 单分片确定性物理(L1)
//!
//! 职责:rapier 封装(`ShardSim`)、体素→collider 纯函数(demo B)、凝固桥(demo C)、
//! 迁移提取/重建(demo A)、三档激活、tick 末迁移检测、跨片事件本地求值(demo D)。
//!
//! 依赖边界:仅依赖 `types`/`voxel`/`messaging`/`rapier`;
//! **防腐层:`rapier` 类型不得泄漏出本 crate**(蓝图 §0)。
//! 禁止:线程、IO、时钟、随机数;一切位姿变更 = 删除+重建(REQ-001 §5 行 10)。
//!
//! 图纸:[docs/design/codebase-layout.md §3 physics-sim](../../docs/design/codebase-layout.md)

pub mod activation;
pub mod collider_build;
pub mod explosion;
pub mod sim;
pub mod snapshot;
pub mod tickets;

mod conv;
mod solidify;

pub use activation::{ActivationConfig, ActivationController, ActivationDecisions};
pub use sim::{BlockWrite, BlockWriteKind, Outbox, ShardSim, ShardSimConfig, SolidifyConfig};
pub use snapshot::SimSnapshotError;
pub use tickets::{
    AreaId, RegionSpec, TicketClock, TicketHolder, TicketKind, TicketState, TicketView, WakeSpec,
};
