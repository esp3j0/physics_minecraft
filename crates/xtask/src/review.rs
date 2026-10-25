//! review —— AI 审图(REQ-008 §6;AC-S6、AC-S8)。
//!
//! 总架构师接口(单元 04 桩):公开类型与签名以桩为准,实现于本文件;真实 zcode
//! CLI 调用见 [`crate::zcode::ZcodeReviewer`]。

use std::path::{Path, PathBuf};

/// 置信度门槛:低于此值的结论标记待复核(REQ-008 §6)。
pub const CONFIDENCE_THRESHOLD: f32 = 0.7;

/// AI 对一张截图的结论。
#[derive(Debug, Clone, PartialEq)]
pub struct Verdict {
    pub observed: String,
    pub pass: bool,
    pub issues: Vec<String>,
    /// 0.0–1.0。
    pub confidence: f32,
}

/// 审图结果。
#[derive(Debug, Clone, PartialEq)]
pub enum ReviewResult {
    Judged(Verdict),
    /// 调用失败、回复里找不到合法 JSON、字段缺失或取值越界。
    Unknown {
        reason: String,
    },
}

impl ReviewResult {
    /// 待复核:不通过、置信度低于 [`CONFIDENCE_THRESHOLD`]、或无法判定。
    pub fn flagged(&self) -> bool {
        match self {
            Self::Unknown { .. } => true,
            Self::Judged(v) => !v.pass || v.confidence < CONFIDENCE_THRESHOLD,
        }
    }
}

/// 把模板里的 `{expect}` 替换为期望文本(全部出现处);模板里没有该占位符 → 错误。
pub fn build_prompt(template: &str, expect: &str) -> Result<String, String> {
    if template.contains("{expect}") {
        Ok(template.replace("{expect}", expect))
    } else {
        Err("模板缺少 {expect} 占位符,期望文本没有落点".to_owned())
    }
}

/// 回复文本里第一个完整的 JSON 对象(起点为第一个 `{`,字符串感知地配对花括号:
/// 字符串内的 `{`/`}` 与 `\"` 等转义不参与计数)。没有完整对象 → `None`。
/// 不得用"第一个 `{` 到最后一个 `}`"截取(放行评论风险 1)。
pub(crate) fn first_json_object(text: &str) -> Option<&str> {
    let bytes = text.as_bytes();
    for (i, &b) in bytes.iter().enumerate() {
        if b != b'{' {
            continue;
        }
        let mut depth = 0usize;
        let mut in_string = false;
        let mut escaped = false;
        for (j, &b) in bytes[i..].iter().enumerate() {
            if in_string {
                if escaped {
                    escaped = false;
                } else if b == b'\\' {
                    escaped = true;
                } else if b == b'"' {
                    in_string = false;
                }
                continue;
            }
            match b {
                b'"' => in_string = true,
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        return Some(&text[i..=i + j]);
                    }
                }
                _ => {}
            }
        }
        // 从这个 `{` 起到文本结束都没有闭合:更靠后的 `{` 都嵌在它里面,也不可能闭合。
        return None;
    }
    None
}

/// 从模型回复中取第一个 JSON 对象(容忍 ```json 代码块包裹与前后的说明文字):
/// `observed`(字符串)、`pass`(布尔)、`issues`(字符串数组)、`confidence`(0.0–1.0 的数)
/// 齐全且类型正确 → `Judged`;否则 `Unknown`(附原因)。第一个对象不合法时不回退到
/// 后面的对象(规格:取第一个,失败即 unknown)。
pub fn parse_verdict(response: &str) -> ReviewResult {
    let unknown = |reason: String| ReviewResult::Unknown { reason };
    let Some(obj) = first_json_object(response) else {
        return unknown("回复里没有 JSON 对象".to_owned());
    };
    let value: serde_json::Value = match serde_json::from_str(obj) {
        Ok(value) => value,
        Err(e) => return unknown(format!("第一个 JSON 对象解析失败:{e}")),
    };
    let Some(observed) = value.get("observed").and_then(serde_json::Value::as_str) else {
        return unknown("缺少字符串字段 observed".to_owned());
    };
    let Some(pass) = value.get("pass").and_then(serde_json::Value::as_bool) else {
        return unknown("缺少布尔字段 pass".to_owned());
    };
    let Some(issues) = value
        .get("issues")
        .and_then(serde_json::Value::as_array)
        .map(|items| {
            items
                .iter()
                .map(|item| item.as_str().map(str::to_owned))
                .collect::<Option<Vec<_>>>()
        })
    else {
        return unknown("缺少数组字段 issues".to_owned());
    };
    let Some(issues) = issues else {
        return unknown("issues 不是字符串数组".to_owned());
    };
    let Some(confidence) = value.get("confidence").and_then(serde_json::Value::as_f64) else {
        return unknown("缺少数字字段 confidence".to_owned());
    };
    if !(0.0..=1.0).contains(&confidence) {
        return unknown(format!("confidence 越界:{confidence}"));
    }
    ReviewResult::Judged(Verdict {
        observed: observed.to_owned(),
        pass,
        issues,
        confidence: confidence as f32,
    })
}

/// 审图器:真实实现调 zcode CLI,测试用假审图器。返回模型回复的原始文本。
pub trait Reviewer {
    fn review(&mut self, image: &Path, prompt: &str) -> Result<String, String>;
}

/// 用模板与期望构造提示词,调一次审图器并解析;构造或调用失败 → `Unknown`。
pub fn review_image(
    reviewer: &mut dyn Reviewer,
    template: &str,
    image: &Path,
    expect: &str,
) -> ReviewResult {
    let prompt = match build_prompt(template, expect) {
        Ok(prompt) => prompt,
        Err(e) => return ReviewResult::Unknown { reason: e },
    };
    match reviewer.review(image, &prompt) {
        Ok(response) => parse_verdict(&response),
        Err(e) => ReviewResult::Unknown {
            reason: format!("审图器调用失败:{e}"),
        },
    }
}

/// 一条校准用例。
#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationCase {
    /// 图片路径(= 校准目录 join 文件名)。
    pub file: PathBuf,
    pub expect: String,
    /// 应得的判定。
    pub pass: bool,
}

/// 读 `<dir>/calibration.toml`(`[[case]]` 数组,字段 `file`、`expect`、`pass`);`file` 解析为
/// `dir.join(file)`。文件不存在、字段缺失、图片不存在 → 错误。
pub fn load_calibration(dir: &Path) -> Result<Vec<CalibrationCase>, String> {
    let path = dir.join("calibration.toml");
    let text =
        std::fs::read_to_string(&path).map_err(|e| format!("读取 {} 失败:{e}", path.display()))?;
    let table: toml::Table = text
        .parse()
        .map_err(|e| format!("calibration.toml 解析失败:{e}"))?;
    let items = table
        .get("case")
        .and_then(toml::Value::as_array)
        .ok_or_else(|| "calibration.toml 缺少 [[case]] 数组".to_owned())?;
    let mut cases = Vec::new();
    for (i, item) in items.iter().enumerate() {
        let fields = item
            .as_table()
            .ok_or_else(|| format!("calibration.toml 第 {} 条 case 不是表", i + 1))?;
        let text_field = |key: &str| {
            fields
                .get(key)
                .and_then(toml::Value::as_str)
                .ok_or_else(|| {
                    format!("calibration.toml 第 {} 条 case 缺少字符串字段 {key}", i + 1)
                })
        };
        let file = text_field("file")?;
        let expect = text_field("expect")?;
        let pass = fields
            .get("pass")
            .and_then(toml::Value::as_bool)
            .ok_or_else(|| format!("calibration.toml 第 {} 条 case 缺少布尔字段 pass", i + 1))?;
        let file = dir.join(file);
        if !file.is_file() {
            return Err(format!("校准图 {} 不存在", file.display()));
        }
        cases.push(CalibrationCase {
            file,
            expect: expect.to_owned(),
            pass,
        });
    }
    Ok(cases)
}

/// 校准结果。
#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationOutcome {
    pub ok: bool,
    /// 判错的用例(每条含图片文件名与原因)。
    pub mismatches: Vec<String>,
}

/// 逐条审校准图:结果为 `Judged` 且 `pass` 与应得一致才算判对(置信度不参与);`Unknown` 或
/// 不一致记入 `mismatches`。全部判对才 `ok`;空集合 → `ok = false`。
pub fn run_calibration(
    reviewer: &mut dyn Reviewer,
    template: &str,
    cases: &[CalibrationCase],
) -> CalibrationOutcome {
    let mut mismatches = Vec::new();
    for case in cases {
        let name = match case.file.file_name().and_then(|f| f.to_str()) {
            Some(name) => name.to_owned(),
            None => case.file.display().to_string(),
        };
        match review_image(reviewer, template, &case.file, &case.expect) {
            ReviewResult::Judged(v) if v.pass == case.pass => {}
            ReviewResult::Judged(v) => mismatches.push(format!(
                "{name}:应判{},实判{}",
                if case.pass { "通过" } else { "不通过" },
                if v.pass { "通过" } else { "不通过" },
            )),
            ReviewResult::Unknown { reason } => {
                mismatches.push(format!("{name}:无法判定({reason})"));
            }
        }
    }
    CalibrationOutcome {
        ok: !cases.is_empty() && mismatches.is_empty(),
        mismatches,
    }
}

/// 截图最终是否待复核:AI 未运行(None)→ 是;校准未通过 → 一律是;否则看 `flagged()`。
pub fn shot_flagged(result: Option<&ReviewResult>, calibration_ok: bool) -> bool {
    if !calibration_ok {
        return true;
    }
    match result {
        None => true,
        Some(r) => r.flagged(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 只回一种预设答案的假审图器(其余调用沿用同一答案)。
    struct OneAnswerReviewer {
        answer: Result<String, String>,
    }

    impl Reviewer for OneAnswerReviewer {
        fn review(&mut self, _image: &Path, _prompt: &str) -> Result<String, String> {
            self.answer.clone()
        }
    }

    /// 扫描器:字符串内的引号转义与花括号不破坏对象边界;嵌套对象能配平;
    /// 第一个对象不合法时不回退到第二个(规格:取第一个,失败即 Unknown)。
    ///
    /// 会变红的实现:转义处理缺失(引号被提前闭合);花括号计数把字符串内部的
    /// `{`/`}` 算进去;解析失败后继续尝试后续对象。
    #[test]
    fn first_object_scanner_handles_strings_and_keeps_first() {
        let escaped =
            r#"{"observed": "说 \"你好\" {x}", "pass": true, "issues": [], "confidence": 0.8}"#;
        assert_eq!(
            parse_verdict(escaped),
            ReviewResult::Judged(Verdict {
                observed: "说 \"你好\" {x}".to_owned(),
                pass: true,
                issues: vec![],
                confidence: 0.8,
            }),
            "转义引号与字符串内花括号"
        );
        let nested = r#"{"observed": "a", "pass": false, "issues": [], "confidence": 1.0, "extra": {"k": "}"}}"#;
        assert_eq!(
            parse_verdict(nested),
            ReviewResult::Judged(Verdict {
                observed: "a".to_owned(),
                pass: false,
                issues: vec![],
                confidence: 1.0,
            }),
            "嵌套对象配平"
        );
        let first_bad =
            r#"{不是 JSON}{"observed": "a", "pass": true, "issues": [], "confidence": 1.0}"#;
        assert!(
            matches!(parse_verdict(first_bad), ReviewResult::Unknown { .. }),
            "第一个对象解析失败不回退到第二个"
        );
    }

    /// 校准 mismatches 点名文件并写明应得 / 实得两个方向。
    ///
    /// 会变红的实现:mismatches 不含文件名;方向词丢失(分不清应得与实得)。
    #[test]
    fn run_calibration_mismatch_names_file_and_directions() {
        let case = CalibrationCase {
            file: PathBuf::from("good.png"),
            expect: "没有蓝色立方体".to_owned(),
            pass: true,
        };
        let fail_json = r#"{"observed": "x", "pass": false, "issues": [], "confidence": 0.9}"#;
        let mut reviewer = OneAnswerReviewer {
            answer: Ok(fail_json.to_owned()),
        };
        let outcome = run_calibration(&mut reviewer, "{expect}", &[case]);
        assert!(!outcome.ok);
        assert_eq!(outcome.mismatches.len(), 1);
        let message = &outcome.mismatches[0];
        assert!(message.contains("good.png"), "点名文件:{message}");
        assert!(
            message.contains("应判通过") && message.contains("实判不通过"),
            "两个方向都在:{message}"
        );
    }
}
