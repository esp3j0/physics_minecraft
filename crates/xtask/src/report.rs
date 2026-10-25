//! report —— 冒烟报告(REQ-008 §7;AC-S7)。
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
    let mut md = String::new();
    md.push_str("# 冒烟报告\n\n");
    let passed = report.scenarios.iter().filter(|e| e.run.passed()).count();
    let flagged = report.shots.iter().filter(|s| s.flagged).count();
    md.push_str(&format!(
        "共 {total} 个场景,通过 {passed} 个;截图 {shots} 张,其中待复核 {flagged} 张。\n\n",
        total = report.scenarios.len(),
        shots = report.shots.len(),
    ));
    match &report.calibration {
        None => md.push_str("校准:AI 未运行\n\n"),
        Some(cal) if cal.ok => md.push_str("校准:通过\n\n"),
        Some(cal) => {
            md.push_str("校准:未通过\n");
            for m in &cal.mismatches {
                md.push_str("- ");
                md.push_str(m);
                md.push('\n');
            }
            md.push('\n');
        }
    }
    for entry in &report.scenarios {
        md.push_str("## ");
        md.push_str(&entry.run.name);
        md.push_str("——");
        md.push_str(if entry.run.passed() {
            "通过"
        } else {
            "未通过"
        });
        md.push_str("\n\n- viewer 退出码:");
        match entry.run.viewer_exit {
            Some(code) => md.push_str(&code.to_string()),
            None => md.push_str("无(被编排器超时杀掉)"),
        }
        md.push_str("\n- 编排器超时:");
        md.push_str(if entry.run.timed_out { "是" } else { "否" });
        md.push_str("\n- 服务器 panic:");
        md.push_str(if entry.run.server_panicked {
            "是"
        } else {
            "否"
        });
        md.push_str("\n\n");
        match &entry.manifest {
            Some(manifest) => {
                for step in &manifest.steps {
                    md.push_str(&format!(
                        "- 第 {n} 步 {action}:{outcome}",
                        n = step.step,
                        action = step.action,
                        outcome = step.outcome,
                    ));
                    if !step.messages.is_empty() {
                        md.push('(');
                        md.push_str(&step.messages.join(";"));
                        md.push(')');
                    }
                    md.push('\n');
                }
                md.push('\n');
            }
            None => md.push_str("-(manifest 缺失:viewer 未写出或解析失败)\n\n"),
        }
    }
    md.push_str("## 截图与 AI 审图\n\n");
    for shot in &report.shots {
        md.push_str(&format!(
            "### {scenario}/{file}{marker}\n\n",
            scenario = shot.scenario,
            file = shot.file,
            marker = if shot.flagged { "(待复核)" } else { "" },
        ));
        // 相对输出根目录的链接(风险 3):报告与截图一起拷走仍可打开。
        md.push_str(&format!(
            "![{file}]({scenario}/{file})\n\n",
            file = shot.file,
            scenario = shot.scenario,
        ));
        md.push_str(&format!("- 期望:{expect}\n", expect = shot.expect));
        match &shot.review {
            Some(ReviewResult::Judged(v)) => {
                md.push_str(&format!("- AI 描述:{observed}\n", observed = v.observed));
                md.push_str(&format!(
                    "- 结论:{verdict}\n",
                    verdict = if v.pass { "通过" } else { "不通过" },
                ));
                md.push_str(&format!("- 置信度:{conf:.2}\n", conf = v.confidence));
                if v.issues.is_empty() {
                    md.push_str("- 问题:无\n");
                } else {
                    md.push_str(&format!("- 问题:{issues}\n", issues = v.issues.join(";")));
                }
            }
            Some(ReviewResult::Unknown { reason }) => {
                md.push_str(&format!("- 结论:无法判定({reason})\n"));
            }
            None => md.push_str("- 结论:AI 未运行\n"),
        }
        if shot.flagged {
            md.push_str("- **待复核**\n");
        }
        md.push('\n');
    }
    md
}

pub fn report_json(report: &RunReport) -> String {
    let value = serde_json::json!({
        "summary": {
            "scenarios_total": report.scenarios.len(),
            "scenarios_passed": report.scenarios.iter().filter(|e| e.run.passed()).count(),
            "shots_total": report.shots.len(),
            "shots_flagged": report.shots.iter().filter(|s| s.flagged).count(),
        },
        "calibration": report.calibration.as_ref().map(|cal| {
            serde_json::json!({ "ok": cal.ok, "mismatches": cal.mismatches })
        }),
        "scenarios": report
            .scenarios
            .iter()
            .map(|entry| {
                serde_json::json!({
                    "name": entry.run.name,
                    "passed": entry.run.passed(),
                    "viewer_exit": entry.run.viewer_exit,
                    "timed_out": entry.run.timed_out,
                    "server_panicked": entry.run.server_panicked,
                    "manifest": entry.manifest.as_ref(),
                })
            })
            .collect::<Vec<_>>(),
        "shots": report
            .shots
            .iter()
            .map(|shot| {
                let review = match &shot.review {
                    Some(ReviewResult::Judged(v)) => serde_json::json!({
                        "kind": "judged",
                        "observed": v.observed,
                        "pass": v.pass,
                        "issues": v.issues,
                        "confidence": v.confidence,
                    }),
                    Some(ReviewResult::Unknown { reason }) => serde_json::json!({
                        "kind": "unknown",
                        "reason": reason,
                    }),
                    None => serde_json::Value::Null,
                };
                serde_json::json!({
                    "scenario": shot.scenario,
                    "file": shot.file,
                    "expect": shot.expect,
                    "flagged": shot.flagged,
                    "review": review,
                })
            })
            .collect::<Vec<_>>(),
    });
    serde_json::to_string_pretty(&value).expect("报告 JSON 序列化不可能失败(纯数据)")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// AI 未运行(review = None、calibration = None)的形态:JSON 里 review 与 calibration
    /// 均为 null;Markdown 逐张给出"AI 未运行"结论并标"待复核";截图链接仍带场景名。
    ///
    /// 会变红的实现:--no-ai 路径把未审渲染成 judged;null 变字符串;漏待复核标记;
    /// 链接用绝对路径或不含场景名。
    #[test]
    fn report_without_ai_marks_shots_unreviewed() {
        let report = RunReport {
            scenarios: vec![],
            calibration: None,
            shots: vec![ShotEntry {
                scenario: "alpha".to_owned(),
                file: "01-a.png".to_owned(),
                expect: "e".to_owned(),
                review: None,
                flagged: true,
            }],
        };
        let v: serde_json::Value = serde_json::from_str(&report_json(&report)).expect("合法 JSON");
        assert!(
            v["calibration"].is_null(),
            "AI 未运行时 calibration 为 null"
        );
        assert_eq!(v["summary"]["shots_flagged"], 1);
        assert_eq!(v["shots"][0]["review"], serde_json::Value::Null);
        let md = report_markdown(&report);
        assert!(md.contains("AI 未运行"), "校准行与逐张结论都要点名:{md}");
        assert!(md.contains("待复核"));
        assert!(md.contains("](alpha/01-a.png)"), "相对路径含场景名:{md}");
    }
}
