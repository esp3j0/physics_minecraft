//! # app —— 装配与驱动(L4)
//!
//! 职责:配置装配(全部参数单一来源)、驱动 loop(biased select:关机>tick>命令,
//! MissedTickBehavior::Skip)、tick 末端收 outbox→盖章分发→持久化屏障→下一拍、
//! 度量告警(step P95>5ms×60tick,只度量不决策)、网络连接层(REQ-003 §4.2,
//! app::net)与 CLI。
//!
//! 依赖边界:唯一允许"认识所有人"的 crate;但业务调用只能经 sim-api 或装配期构造,
//! 禁止直操 physics-sim/shard 内部(蓝图 §2 依赖规则)。
//!
//! 图纸:[docs/design/codebase-layout.md §3 app](../../docs/design/codebase-layout.md)

pub mod config;
pub mod driver;
pub mod net;
pub mod telemetry;
