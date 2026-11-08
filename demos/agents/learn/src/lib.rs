//! REQ-012 A00-5 demo `learn`(R2,2026-11-08):Rust 环境(双关节摆臂
//! `arm`)+ Python PyTorch PPO(tools/train,CleanRL 底本)。
//!
//! 本 crate 两个形态:
//! - **rlib**:`learn` 二进制(l0/eval/order/det/throughput 子命令)直接
//!   用 `arm::Env`,评估加载 `tract` ONNX 策略(部署路径);
//! - **cdylib**(`--features pyo3`):`learn_env.VecEnv` Python 模块,
//!   `maturin develop --release` 装进 `tools/train/.venv`,训练侧只通过
//!   `tools/train/train/backends/rapier.py` 的适配器访问。
//!
//! 统一口径(R2 再补充):60 Hz 物理生产参数(只覆写 dt)、无地面、世界
//! 重力不改;30 Hz 控制(每 2 拍一个控制周期)。四足环境的 R1 代码已随
//! 手写 PPO 一起退役;开环步态探针(同一环境 0.406 m/s)与 R1 吞吐表
//! 作为记录保留在 demo README。

pub mod arm;
#[cfg(feature = "pyo3")]
pub mod pyenv;

/// 物理步长(60 Hz)。
pub const DT: f32 = 1.0 / 60.0;
/// 每个控制周期的物理拍数(30 Hz;60 Hz 对照传 1)。
pub const TICKS_PER_CONTROL: usize = 2;
