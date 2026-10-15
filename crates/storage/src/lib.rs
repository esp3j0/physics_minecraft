//! # storage —— 持久化唯一入口(L3)
//!
//! 职责:`ChunkStore`/`CheckpointStore` trait、单写 worker(批量缓冲 + 显式
//! flush→oneshot ack,复刻 v2 "卸载前持久化屏障")、redb 实现与内存实现(测试/回放)。
//!
//! 依赖边界:仅依赖 `types`/`voxel`;**禁止依赖任何模拟 crate**(shard/sim-api
//! 只面向本 crate 的 trait,由 app 装配)。存储格式(bincode/redb 表结构)不得
//! 泄漏出本 crate。
//!
//! 图纸:[docs/design/codebase-layout.md §3 storage](../../docs/design/codebase-layout.md)

pub mod mem;
pub mod redb;
pub mod traits;
pub mod worker;
