//! smoke —— 图形冒烟编排的纯逻辑部分(REQ-008 §5;AC-S5)。
//!
//! **总架构师接口桩(REQ-008 单元 03)**:公开类型与签名固定,函数体由实施方实现。

use std::path::{Path, PathBuf};

/// 生成场景用的服务器配置:以 `app::config::Config::default()` 为底,叠加场景 `[server]` 覆盖项
/// (键与 `Config` 的 toml 同名),再写入 `listen_addr` 与 `data_dir`。经 `Config` 的解析与校验:
/// 未知键、类型错误、违反校验规则(如 ② 半径大于 ① 半径)一律报错。
pub fn server_config(
    server: &toml::Table,
    listen_addr: &str,
    data_dir: &Path,
) -> Result<app::config::Config, String> {
    let _ = (server, listen_addr, data_dir);
    unimplemented!("REQ-008 单元 03:由实施方实现")
}

/// 服务器日志判定。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ServerLog {
    Clean,
    /// 含 Rust panic 消息(`panicked at`)的那一行。
    Panicked {
        line: String,
    },
}

/// 扫描服务器的标准输出与标准错误:任一处出现 panic 即 `Panicked`(取第一处)。
pub fn scan_server_log(stdout: &str, stderr: &str) -> ServerLog {
    let _ = (stdout, stderr);
    unimplemented!("REQ-008 单元 03:由实施方实现")
}

/// 选出要跑的场景文件:
/// - `selectors` 为空 → `dir` 下全部 `*.toml`,按文件名升序;
/// - 每个选择项先当作已存在的文件路径,否则当作场景名(`dir/<名>.toml`);
/// - 找不到 → 报错并点名;`dir` 下没有任何场景 → 报错;重复项只保留第一次。
pub fn resolve_scenarios(dir: &Path, selectors: &[String]) -> Result<Vec<PathBuf>, String> {
    let _ = (dir, selectors);
    unimplemented!("REQ-008 单元 03:由实施方实现")
}

/// 单个场景的运行结果。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScenarioRun {
    pub name: String,
    /// viewer 进程退出码;被编排器超时杀掉时为 None。
    pub viewer_exit: Option<i32>,
    pub timed_out: bool,
    pub server_panicked: bool,
}

impl ScenarioRun {
    /// 通过 = viewer 退出码 0、没有超时、服务器没有 panic。
    pub fn passed(&self) -> bool {
        unimplemented!("REQ-008 单元 03:由实施方实现")
    }
}

/// 汇总退出码:全部通过为 0;任一未通过或一个场景都没跑为 1。
pub fn overall_exit_code(runs: &[ScenarioRun]) -> i32 {
    let _ = runs;
    unimplemented!("REQ-008 单元 03:由实施方实现")
}
