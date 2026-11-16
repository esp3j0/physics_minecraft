//! REQ-012 D01a:策略包库(观测函数、`policy.toml`、加载校验、tract 推理、
//! 动作 → 关节目标)。纯库,不碰内核 / 驱动 / 协议(接线在 D01b)。
//!
//! 模块:[`manifest`] 策略包清单、[`observe`] 观测函数、
//! [`runtime`] 加载与推理、[`error`] 错误类型。
//!
//! 非确定性核,但禁墙钟(工作区 clippy.toml);只有 AC-D4 计时测试局部豁免。

pub mod error;
pub mod manifest;
pub mod observe;
pub mod runtime;

pub use error::PolicyError;
pub use manifest::{Normalization, ObsEntry, PolicyManifest};
pub use observe::{AgentSnapshot, ObsTerm, observe};
pub use runtime::{Action, Policy};
