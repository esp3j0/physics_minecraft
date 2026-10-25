//! report —— 冒烟报告(REQ-008 §7;AC-S7)。
//!
//! **总架构师接口桩(REQ-008 单元 04)**:公开类型与签名固定,函数体由实施方实现。
//!
//! `report_markdown` 必须包含:
//! - 汇总行:场景通过数 / 总数、待复核截图数;校准状态一行——"校准:通过" / "校准:未通过"
//!   (并列出判错的用例)/ "AI 未运行";
//! - 每个场景一节:场景名、"通过"或"未通过"、viewer 退出码、是否超时、服务器是否 panic;
//!   manifest 存在时列出每步的序号、动作、结果,失败步骤带原因;
//! - 每张截图:`![<文件名>](<场景名>/<文件名>)` 形式的相对路径图片、期望文本、AI 的 `observed`、
//!   结论(通过 / 不通过 / 无法判定)、置信度、问题列表;待复核的截图带"待复核"字样。
//!
//! `report_json` 输出一个 JSON 对象:
//! - `summary`:`{"scenarios_total", "scenarios_passed", "shots_total", "shots_flagged"}`;
//! - `calibration`:`null`(AI 未运行)或 `{"ok", "mismatches"}`;
//! - `scenarios`:数组,每项 `{"name", "passed", "viewer_exit", "timed_out", "server_panicked",
//!   "manifest"}`(`manifest` 为 manifest 的 JSON 对象或 `null`);
//! - `shots`:数组,每项 `{"scenario", "file", "expect", "flagged", "review"}`;`review` 为 `null`
//!   (未审)、`{"kind": "judged", "observed", "pass", "issues", "confidence"}` 或
//!   `{"kind": "unknown", "reason"}`。

use crate::review::{CalibrationOutcome, ReviewResult};
use crate::smoke::ScenarioRun;

/// 一个场景的运行结果与运行记录。
#[derive(Debug, Clone)]
pub struct ScenarioEntry {
    pub run: ScenarioRun,
    pub manifest: Option<viewer::script::manifest::Manifest>,
}

/// 一张截图的审图条目。
#[derive(Debug, Clone)]
pub struct ShotEntry {
    pub scenario: String,
    /// 截图文件名(相对于场景输出目录)。
    pub file: String,
    pub expect: String,
    /// AI 未运行为 None。
    pub review: Option<ReviewResult>,
    pub flagged: bool,
}

/// 一次运行的报告数据。
#[derive(Debug, Clone)]
pub struct RunReport {
    pub scenarios: Vec<ScenarioEntry>,
    /// AI 未运行为 None。
    pub calibration: Option<CalibrationOutcome>,
    pub shots: Vec<ShotEntry>,
}

pub fn report_markdown(report: &RunReport) -> String {
    let _ = report;
    unimplemented!("REQ-008 单元 04:由实施方实现")
}

pub fn report_json(report: &RunReport) -> String {
    let _ = report;
    unimplemented!("REQ-008 单元 04:由实施方实现")
}
