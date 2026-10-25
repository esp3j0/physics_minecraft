//! REQ-008 单元 04 验收用例(总架构师编写;issue #16):AI 审图、校准、报告的纯逻辑部分。
//!
//! 约定同其他验收文件:实施方不得修改断言,完成后只去掉 `#[ignore]`。真实 zcode 调用、
//! `--selftest` 与带 AI 的完整运行由总架构师实跑核对。

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use viewer::script::manifest::{Manifest, ManifestCamera, ManifestResult, ManifestStep};
use xtask::report::{RunReport, ScenarioEntry, ShotEntry, report_json, report_markdown};
use xtask::review::{
    CalibrationCase, CalibrationOutcome, ReviewResult, Reviewer, Verdict, build_prompt,
    load_calibration, parse_verdict, review_image, run_calibration, shot_flagged,
};
use xtask::smoke::ScenarioRun;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn judged(pass: bool, confidence: f32) -> ReviewResult {
    ReviewResult::Judged(Verdict {
        observed: "地面上有灰色方块".to_owned(),
        pass,
        issues: if pass {
            vec![]
        } else {
            vec!["有蓝色立方体".to_owned()]
        },
        confidence,
    })
}

/// 假审图器:按提示词中出现的关键字返回预设回复;记录每次调用。
struct FakeReviewer {
    answers: Vec<(String, Result<String, String>)>,
    calls: Vec<(PathBuf, String)>,
}

impl FakeReviewer {
    fn new(answers: &[(&str, Result<&str, &str>)]) -> Self {
        Self {
            answers: answers
                .iter()
                .map(|(k, v)| ((*k).to_owned(), v.map(str::to_owned).map_err(str::to_owned)))
                .collect(),
            calls: Vec::new(),
        }
    }
}

impl Reviewer for FakeReviewer {
    fn review(&mut self, image: &Path, prompt: &str) -> Result<String, String> {
        self.calls.push((image.to_path_buf(), prompt.to_owned()));
        for (key, answer) in &self.answers {
            if prompt.contains(key.as_str()) {
                return answer.clone();
            }
        }
        Err("假审图器:没有匹配的预设回复".to_owned())
    }
}

const PASS_JSON: &str =
    r#"{"observed": "地面上有灰色方块", "pass": true, "issues": [], "confidence": 0.9}"#;
const FAIL_JSON: &str =
    r#"{"observed": "空中有蓝色立方体", "pass": false, "issues": ["悬空"], "confidence": 0.85}"#;

// ===========================================================================
// AC-S6 提示词与回复解析
// ===========================================================================

/// 提示词:模板中所有 `{expect}` 被替换;模板缺占位符报错;仓库内模板写明了画面约定。
///
/// 会变红的实现:只替换第一处;缺占位符时静默把期望丢掉;模板没说明蓝色立方体的含义。
#[test]
fn acc_s6_build_prompt() {
    assert_eq!(
        build_prompt("A {expect} B {expect}", "期望X").expect("有占位符"),
        "A 期望X B 期望X"
    );
    assert!(build_prompt("没有占位符", "期望X").is_err());
    let template = std::fs::read_to_string(repo_root().join("smoke/review-prompt.md"))
        .expect("仓库内审图模板存在");
    let prompt = build_prompt(&template, "地面上有方块").expect("仓库模板有占位符");
    assert!(prompt.contains("地面上有方块"));
    assert!(prompt.contains("蓝色立方体是尚未凝固"), "模板说明画面约定");
    assert!(!prompt.contains("{expect}"));
}

/// 解析:纯 JSON、代码块包裹、前后有说明文字、字符串里含花括号都能取到第一个对象;
/// 字段缺失、类型错误、置信度越界、没有 JSON 都是 `Unknown`。
///
/// 会变红的实现:只认纯 JSON;用"第一个 `{` 到最后一个 `}`"截取(多个对象或字符串里有花括号时
/// 出错);不校验字段类型与取值范围。
#[test]
fn acc_s6_parse_verdict() {
    let want = Verdict {
        observed: "地面上有灰色方块".to_owned(),
        pass: true,
        issues: vec![],
        confidence: 0.9,
    };
    assert_eq!(parse_verdict(PASS_JSON), ReviewResult::Judged(want.clone()));
    assert_eq!(
        parse_verdict(&format!(
            "好的,结论如下:\n```json\n{PASS_JSON}\n```\n以上。"
        )),
        ReviewResult::Judged(want.clone()),
        "代码块包裹与前后文字"
    );
    assert_eq!(
        parse_verdict(&format!("{PASS_JSON}\n补充:{FAIL_JSON}")),
        ReviewResult::Judged(want),
        "多个对象取第一个"
    );
    assert_eq!(
        parse_verdict(
            r#"{"observed": "画面右上角有 {HUD} 字样", "pass": false, "issues": ["多了 } 号"], "confidence": 0.8}"#
        ),
        ReviewResult::Judged(Verdict {
            observed: "画面右上角有 {HUD} 字样".to_owned(),
            pass: false,
            issues: vec!["多了 } 号".to_owned()],
            confidence: 0.8,
        }),
        "字符串内的花括号不影响对象边界"
    );
    for (bad, what) in [
        (
            r#"{"observed": "x", "pass": true, "issues": []}"#,
            "缺 confidence",
        ),
        (
            r#"{"observed": "x", "pass": "true", "issues": [], "confidence": 0.9}"#,
            "pass 不是布尔",
        ),
        (
            r#"{"observed": "x", "pass": true, "issues": [], "confidence": 1.5}"#,
            "置信度大于 1",
        ),
        (
            r#"{"observed": "x", "pass": true, "issues": [], "confidence": -0.1}"#,
            "置信度小于 0",
        ),
        (
            r#"{"observed": "x", "pass": true, "issues": "无", "confidence": 0.9}"#,
            "issues 不是数组",
        ),
        ("我觉得画面没问题。", "没有 JSON"),
        ("{\"observed\": \"x\", \"pass\": tru", "JSON 不完整"),
    ] {
        assert!(
            matches!(parse_verdict(bad), ReviewResult::Unknown { .. }),
            "{what}:应为 Unknown"
        );
    }
}

/// 待复核规则:不通过、置信度 < 0.7、无法判定都待复核;AI 未运行待复核;校准未通过时一律待复核。
///
/// 会变红的实现:门槛写成 `<=`;只看 pass 不看置信度;校准失败时仍采信 AI 结论。
#[test]
fn acc_s6_flag_rules() {
    assert!(!judged(true, 0.7).flagged(), "恰好 0.7 不待复核");
    assert!(judged(true, 0.69).flagged(), "低于 0.7 待复核");
    assert!(judged(false, 0.95).flagged(), "不通过待复核");
    assert!(
        ReviewResult::Unknown {
            reason: "x".to_owned()
        }
        .flagged()
    );
    let good = judged(true, 0.9);
    assert!(!shot_flagged(Some(&good), true));
    assert!(shot_flagged(Some(&good), false), "校准未通过 → 一律待复核");
    assert!(shot_flagged(None, true), "AI 未运行 → 待复核");
}

/// `review_image`:提示词带上期望、把图片交给审图器;审图器出错或模板缺占位符 → Unknown
/// (模板缺占位符时不调用审图器)。
///
/// 会变红的实现:没把期望放进提示词;审图器出错时 panic;模板错误时仍去调用审图器。
#[test]
fn acc_s6_review_image_with_fake_reviewer() {
    let mut fake = FakeReviewer::new(&[("期望甲", Ok(PASS_JSON)), ("期望乙", Err("超时"))]);
    let img = Path::new("shots/01-a.png");
    assert_eq!(
        review_image(&mut fake, "看图:{expect}", img, "期望甲"),
        judged(true, 0.9)
    );
    assert_eq!(fake.calls.len(), 1);
    assert_eq!(fake.calls[0].0, img.to_path_buf());
    assert_eq!(fake.calls[0].1, "看图:期望甲");
    assert!(matches!(
        review_image(&mut fake, "看图:{expect}", img, "期望乙"),
        ReviewResult::Unknown { .. }
    ));
    let before = fake.calls.len();
    assert!(matches!(
        review_image(&mut fake, "没有占位符", img, "期望甲"),
        ReviewResult::Unknown { .. }
    ));
    assert_eq!(fake.calls.len(), before, "模板错误时不调用审图器");
}

// ===========================================================================
// AC-S8 校准
// ===========================================================================

/// 校准:全部判对才通过;判错或无法判定的用例记入 mismatches(含文件名);空集合不通过;
/// 置信度不参与校准判定。
///
/// 会变红的实现:只要多数判对就算通过;Unknown 被当作判对;mismatches 不点名文件;空集合算通过。
#[test]
fn acc_s8_run_calibration() {
    let case = |file: &str, expect: &str, pass: bool| CalibrationCase {
        file: PathBuf::from(file),
        expect: expect.to_owned(),
        pass,
    };
    let cases = vec![
        case("good.png", "没有蓝色立方体", true),
        case("bad.png", "有蓝色立方体在空中", false),
    ];
    let low_conf_fail =
        r#"{"observed": "地面", "pass": false, "issues": ["没有蓝色立方体"], "confidence": 0.5}"#;
    let mut right = FakeReviewer::new(&[
        ("没有蓝色立方体", Ok(PASS_JSON)),
        ("有蓝色立方体在空中", Ok(low_conf_fail)),
    ]);
    let ok = run_calibration(&mut right, "{expect}", &cases);
    assert_eq!(
        ok,
        CalibrationOutcome {
            ok: true,
            mismatches: vec![]
        },
        "全部判对(低置信度的正确判定也算对)"
    );

    let mut wrong = FakeReviewer::new(&[
        ("没有蓝色立方体", Ok(PASS_JSON)),
        ("有蓝色立方体在空中", Ok(PASS_JSON)),
    ]);
    let bad = run_calibration(&mut wrong, "{expect}", &cases);
    assert!(!bad.ok);
    assert_eq!(bad.mismatches.len(), 1);
    assert!(bad.mismatches[0].contains("bad.png"), "点名判错的图片");

    let mut broken = FakeReviewer::new(&[("没有蓝色立方体", Ok("看不懂"))]);
    let unknown = run_calibration(&mut broken, "{expect}", &cases);
    assert!(!unknown.ok);
    assert_eq!(unknown.mismatches.len(), 2, "无法判定与调用失败都算判错");

    assert!(
        !run_calibration(&mut right, "{expect}", &[]).ok,
        "空集合不通过"
    );
}

/// 仓库内校准集:可读、图片都在、至少 2 条应判通过与 2 条应判不通过、至少 3 张不同的图。
///
/// 会变红的实现:`file` 没拼上校准目录;字段读错。
#[test]
fn acc_s8_committed_calibration_set() {
    let dir = repo_root().join("smoke/calibration");
    let cases = load_calibration(&dir).expect("校准集可读");
    assert!(cases.iter().filter(|c| c.pass).count() >= 2);
    assert!(cases.iter().filter(|c| !c.pass).count() >= 2);
    let mut images = std::collections::BTreeSet::new();
    for c in &cases {
        assert!(c.file.starts_with(&dir), "file 拼上校准目录:{:?}", c.file);
        assert!(c.file.is_file(), "图片存在:{:?}", c.file);
        assert!(!c.expect.is_empty());
        images.insert(c.file.clone());
    }
    assert!(images.len() >= 3);

    let empty = tempfile::tempdir().expect("临时目录");
    assert!(
        load_calibration(empty.path()).is_err(),
        "没有 calibration.toml 报错"
    );
    std::fs::write(
        empty.path().join("calibration.toml"),
        "[[case]]\nfile = \"missing.png\"\nexpect = \"x\"\npass = true\n",
    )
    .expect("写文件");
    assert!(load_calibration(empty.path()).is_err(), "图片不存在报错");
}

// ===========================================================================
// AC-S7 报告
// ===========================================================================

fn step(n: usize, action: &str, outcome: &str, messages: &[&str]) -> ManifestStep {
    ManifestStep {
        step: n,
        label: None,
        action: action.to_owned(),
        start_tick: 10,
        end_tick: 20,
        camera: ManifestCamera {
            pos: [0.0, -50.0, 0.0],
            yaw: 0.0,
            pitch: 0.0,
        },
        outcome: outcome.to_owned(),
        messages: messages.iter().map(|m| (*m).to_owned()).collect(),
        screenshot: None,
    }
}

fn fixture(calibration: Option<CalibrationOutcome>) -> RunReport {
    let manifest = |name: &str, steps: Vec<ManifestStep>, exit: i32| Manifest {
        scenario: name.to_owned(),
        req: None,
        description: "d".to_owned(),
        steps,
        result: Some(ManifestResult {
            status: "done".to_owned(),
            exit_code: exit,
            failed_checks: usize::from(exit == 1),
            reason: None,
        }),
    };
    RunReport {
        scenarios: vec![
            ScenarioEntry {
                run: ScenarioRun {
                    name: "alpha".to_owned(),
                    viewer_exit: Some(0),
                    timed_out: false,
                    server_panicked: false,
                },
                manifest: Some(manifest("alpha", vec![step(1, "camera", "ok", &[])], 0)),
            },
            ScenarioEntry {
                run: ScenarioRun {
                    name: "beta".to_owned(),
                    viewer_exit: Some(1),
                    timed_out: false,
                    server_panicked: false,
                },
                manifest: Some(manifest(
                    "beta",
                    vec![step(
                        1,
                        "check",
                        "check_failed",
                        &["活实体数 3 不满足 Eq(0)"],
                    )],
                    1,
                )),
            },
        ],
        calibration,
        shots: vec![
            ShotEntry {
                scenario: "alpha".to_owned(),
                file: "02-good.png".to_owned(),
                expect: "期望良好".to_owned(),
                review: Some(judged(true, 0.9)),
                flagged: false,
            },
            ShotEntry {
                scenario: "alpha".to_owned(),
                file: "03-bad.png".to_owned(),
                expect: "期望不符".to_owned(),
                review: Some(judged(false, 0.8)),
                flagged: true,
            },
            ShotEntry {
                scenario: "beta".to_owned(),
                file: "04-unk.png".to_owned(),
                expect: "期望未知".to_owned(),
                review: Some(ReviewResult::Unknown {
                    reason: "回复里没有 JSON".to_owned(),
                }),
                flagged: true,
            },
        ],
    }
}

/// Markdown 报告:汇总、校准状态、每个场景的通过与失败步骤原因、每张截图(相对路径、期望、
/// AI 描述与结论、待复核标记)。
///
/// 会变红的实现:漏写失败原因;截图用绝对路径或路径不含场景名;待复核没有标出;校准状态缺失。
#[test]
fn acc_s7_report_markdown() {
    let md = report_markdown(&fixture(Some(CalibrationOutcome {
        ok: true,
        mismatches: vec![],
    })));
    for want in [
        "alpha",
        "beta",
        "通过",
        "未通过",
        "活实体数 3 不满足 Eq(0)",
        "](alpha/02-good.png)",
        "](alpha/03-bad.png)",
        "](beta/04-unk.png)",
        "期望良好",
        "期望不符",
        "地面上有灰色方块",
        "有蓝色立方体",
        "回复里没有 JSON",
        "待复核",
        "校准:通过",
    ] {
        assert!(md.contains(want), "Markdown 应包含 {want:?}");
    }
    let failed = report_markdown(&fixture(Some(CalibrationOutcome {
        ok: false,
        mismatches: vec!["blue-cubes.png:应判不通过,实判通过".to_owned()],
    })));
    assert!(failed.contains("校准:未通过"));
    assert!(failed.contains("blue-cubes.png"), "列出判错的用例");
    assert!(report_markdown(&fixture(None)).contains("AI 未运行"));
}

/// JSON 报告:字段齐全、取值正确。
///
/// 会变红的实现:字段名或结构与约定不符;汇总计数错;review 三种形态没区分。
#[test]
fn acc_s7_report_json() {
    let v: serde_json::Value =
        serde_json::from_str(&report_json(&fixture(Some(CalibrationOutcome {
            ok: true,
            mismatches: vec![],
        }))))
        .expect("合法 JSON");
    assert_eq!(v["summary"]["scenarios_total"], 2);
    assert_eq!(v["summary"]["scenarios_passed"], 1);
    assert_eq!(v["summary"]["shots_total"], 3);
    assert_eq!(v["summary"]["shots_flagged"], 2);
    assert_eq!(v["calibration"]["ok"], true);
    assert_eq!(v["calibration"]["mismatches"], serde_json::json!([]));
    let names: Vec<_> = v["scenarios"]
        .as_array()
        .expect("scenarios 为数组")
        .iter()
        .map(|s| {
            (
                s["name"].clone(),
                s["passed"].clone(),
                s["viewer_exit"].clone(),
            )
        })
        .collect();
    assert_eq!(
        names,
        vec![
            (
                serde_json::json!("alpha"),
                serde_json::json!(true),
                serde_json::json!(0)
            ),
            (
                serde_json::json!("beta"),
                serde_json::json!(false),
                serde_json::json!(1)
            ),
        ]
    );
    assert_eq!(v["scenarios"][1]["timed_out"], false);
    assert_eq!(v["scenarios"][1]["server_panicked"], false);
    assert_eq!(v["scenarios"][1]["manifest"]["scenario"], "beta");
    let shots = v["shots"].as_array().expect("shots 为数组");
    let by_file: BTreeMap<String, &serde_json::Value> = shots
        .iter()
        .map(|s| (s["file"].as_str().unwrap_or_default().to_owned(), s))
        .collect();
    let good = by_file["02-good.png"];
    assert_eq!(good["scenario"], "alpha");
    assert_eq!(good["expect"], "期望良好");
    assert_eq!(good["flagged"], false);
    assert_eq!(good["review"]["kind"], "judged");
    assert_eq!(good["review"]["pass"], true);
    assert_eq!(good["review"]["observed"], "地面上有灰色方块");
    assert!((good["review"]["confidence"].as_f64().unwrap_or(0.0) - 0.9).abs() < 1e-6);
    assert_eq!(
        by_file["03-bad.png"]["review"]["issues"],
        serde_json::json!(["有蓝色立方体"])
    );
    assert_eq!(by_file["04-unk.png"]["review"]["kind"], "unknown");
    assert_eq!(by_file["04-unk.png"]["review"]["reason"], "回复里没有 JSON");

    let no_ai: serde_json::Value =
        serde_json::from_str(&report_json(&fixture(None))).expect("合法 JSON");
    assert!(no_ai["calibration"].is_null());
}
