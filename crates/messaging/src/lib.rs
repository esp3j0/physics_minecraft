//! # messaging —— 命令、事件与信封(L1)
//!
//! 职责:`SimCommand`(外部命令)、`CrossEvent`(跨片事件及共享参数)、
//! `Envelope`(盖章/五元组 canonical 排序/有界队列,照 ferrumc v2 cross_shard.rs 移植)、
//! 统一输入模型 `StampedInput`(一切输入由调度器盖章 apply_at)。
//!
//! 依赖边界:仅依赖 `types`;不含 IO、rapier 类型、voxel 内部结构。
//!
//! 图纸:[docs/design/codebase-layout.md §3 messaging](../../docs/design/codebase-layout.md)

mod command;
mod envelope;
mod event;
mod ordering;

pub use command::{InputPayload, InputSource, SimCommand, StampedInput};
pub use envelope::{
    AdmitError, CrossShardQueue, DEFAULT_QUEUE_CAPACITY, Envelope, EnvelopePayload, Intent,
};
pub use event::CrossEvent;
pub use ordering::CanonicalKey;
