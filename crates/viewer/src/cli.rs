//! cli —— viewer 命令行参数(REQ-008 §4):交互模式或脚本模式。
//!
//! **总架构师接口桩(REQ-008 单元 02)**:公开类型与签名固定,函数体由实施方实现。
//!
//! 写法:`viewer [--addr <地址>] [--script <场景文件> --out <输出目录>]`。
//! - 不带参数 → 交互模式(行为同 REQ-004A);
//! - `--script` 与 `--out` 必须同时给出;
//! - `--addr` 缺省时由调用方回落到环境变量 `VIEWER_ADDR` 或 `127.0.0.1:25565`;
//! - 未知参数、缺少参数值、重复参数 → 错误。

use std::path::PathBuf;

/// 运行模式。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ViewerMode {
    Interactive,
    Script { script: PathBuf, out: PathBuf },
}

/// 解析后的命令行参数。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViewerArgs {
    pub mode: ViewerMode,
    pub addr: Option<String>,
}

/// 解析命令行参数(不含程序名)。
pub fn parse_args<I: IntoIterator<Item = String>>(args: I) -> Result<ViewerArgs, String> {
    let _ = args.into_iter();
    unimplemented!("REQ-008 单元 02:由实施方实现")
}
