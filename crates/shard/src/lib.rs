//! # shard —— 分片编排(L2)
//!
//! 职责:Shard 所有权表(claim/release)、固定 worker 池(OS 线程 + capacity-one
//! 通道 + 每 tick 全员汇合屏障,分片所有权转移)、TickCoordinator(逻辑 tick 单调 +1、
//! 不看墙钟、无追赶)、生命周期(Created→Active→Draining→Stopped,懒创建、
//! 空片无票 30s despawn)、命令路由与有界收件箱(满则断开权威输入)。
//!
//! 依赖边界:仅依赖 `types`/`messaging`/`physics-sim`;
//! **全项目唯一允许创建线程的地方**(蓝图 §2)。
//! 禁止:IO、墙钟(逻辑 tick 不看墙钟,REQ-001 §4.4)。
//!
//! 图纸:[docs/design/codebase-layout.md §3 shard](../../docs/design/codebase-layout.md)

pub mod chunks;
pub mod coordinator;
pub mod entities;
pub mod lifecycle;
pub mod ownership;
pub mod router;
pub mod scheduler;
pub mod tickets;

pub use entities::{EntityOwner, EntityOwnership};
pub use lifecycle::LifecycleSnapshot;
pub use router::RouterSnapshot;
pub use scheduler::{
    ActivationTelemetry, Scheduler, SchedulerConfig, SchedulerSnapshot, TickSummary,
};
