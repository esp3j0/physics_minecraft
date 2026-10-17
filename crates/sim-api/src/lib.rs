//! # sim-api —— headless 缝(L3)
//!
//! 职责:会话(`Session`:命令进、快照/增量出,**接口形状 = 网络形状**,REQ-001
//! 共识 3)、全局快照(分片快照集合 + 所有权表 + tick)、回放器(checkpoint +
//! 外部命令日志;信封/事件/迁移由重放确定性再生,AC8)。
//!
//! 依赖边界:依赖 `types`/`messaging`/`shard`/`storage`,外加
//! `physics-sim`(仅 `ShardSim` 恢复与注册,分片字节仍不透明)与
//! `voxel`(仅 `BlockRegistry` 注入)——README 决策 1 对蓝图 §2 的修订;
//! 禁止:rapier 类型泄漏到本 crate 之外、具体网络实现(未来协议只映射本 crate 的接口)。
//!
//! 图纸:[docs/design/codebase-layout.md §3 sim-api](../../docs/design/codebase-layout.md)

pub mod log;
pub mod proto;
pub mod replay;
pub mod session;
pub mod snapshot;

pub use proto::{
    C2S, FRAME_MAX, S2C, decode_c2s_frame, decode_frame, encode_c2s_frame, encode_frame,
};
