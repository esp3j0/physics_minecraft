//! review —— AI 审图(REQ-008 §6;AC-S6、AC-S8)。
//!
//! **总架构师接口桩(REQ-008 单元 04)**:公开类型与签名固定,函数体由实施方实现。

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
        unimplemented!("REQ-008 单元 04:由实施方实现")
    }
}

/// 把模板里的 `{expect}` 替换为期望文本(全部出现处);模板里没有该占位符 → 错误。
pub fn build_prompt(template: &str, expect: &str) -> Result<String, String> {
    let _ = (template, expect);
    unimplemented!("REQ-008 单元 04:由实施方实现")
}

/// 从模型回复中取第一个 JSON 对象(容忍 ```json 代码块包裹与前后的说明文字):
/// `observed`(字符串)、`pass`(布尔)、`issues`(字符串数组)、`confidence`(0.0–1.0 的数)
/// 齐全且类型正确 → `Judged`;否则 `Unknown`(附原因)。
pub fn parse_verdict(response: &str) -> ReviewResult {
    let _ = response;
    unimplemented!("REQ-008 单元 04:由实施方实现")
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
    let _ = (reviewer, template, image, expect);
    unimplemented!("REQ-008 单元 04:由实施方实现")
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
    let _ = dir;
    unimplemented!("REQ-008 单元 04:由实施方实现")
}

/// 校准结果。
#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationOutcome {
    pub ok: bool,
    /// 判错的用例(每条含图片文件名与原因)。
    pub mismatches: Vec<String>,
}

/// 逐条审校准图:结果为 `Judged` 且 `pass` 与应得一致才算判对(置信度不参与);`Unknown` 或不一致
/// 记入 `mismatches`。全部判对才 `ok`;空集合 → `ok = false`。
pub fn run_calibration(
    reviewer: &mut dyn Reviewer,
    template: &str,
    cases: &[CalibrationCase],
) -> CalibrationOutcome {
    let _ = (reviewer, template, cases);
    unimplemented!("REQ-008 单元 04:由实施方实现")
}

/// 截图最终是否待复核:AI 未运行(None)→ 是;校准未通过 → 一律是;否则看 `flagged()`。
pub fn shot_flagged(result: Option<&ReviewResult>, calibration_ok: bool) -> bool {
    let _ = (result, calibration_ok);
    unimplemented!("REQ-008 单元 04:由实施方实现")
}
