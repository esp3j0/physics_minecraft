//! manifest —— 场景运行记录(REQ-008 §4):viewer 脚本模式写出,编排器与审图器读取。
//!
//! **总架构师接口桩(REQ-008 单元 02)**:公开类型、字段与签名固定(编排器与审图器依赖这份
//! 格式),函数体由实施方实现。
//!
//! 语义(验收用例据此编写):
//! - `new` 取场景的 `name`/`req`/`description`,步骤为空、`result` 为 None;
//! - `push` 追加一条步骤记录:相机位姿、起止拍号、结果原样转写;`outcome` 取 `"ok"` /
//!   `"check_failed"` / `"error"`,`messages` 为失败原因(ok 时为空);只有截图步骤带 `screenshot`,
//!   其中 `live_entities`、`chunks_loaded` 取 push 当时 `world` 的值,`file` 为截图文件名;
//! - `finish` 写入结果:`status` 为 `"done"` 或 `"failed"`,`exit_code` 与
//!   `Engine::exit_code` 同义(0/1/2),`failed_checks` 取 Done 的值(Failed 时为已记录步骤中
//!   check_failed 的个数),`reason` 只在 failed 时有值;
//! - `to_json` 输出 JSON(字段名即结构体字段名),`from_json` 能读回相等的值。

use serde::{Deserialize, Serialize};

use super::{EngineStatus, Scenario, StepRecord, WorldQuery};

/// 场景运行记录。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Manifest {
    pub scenario: String,
    pub req: Option<String>,
    pub description: String,
    pub steps: Vec<ManifestStep>,
    pub result: Option<ManifestResult>,
}

/// 一步的记录。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManifestStep {
    /// 步骤序号(从 1 开始)。
    pub step: usize,
    pub label: Option<String>,
    pub action: String,
    pub start_tick: u64,
    pub end_tick: u64,
    pub camera: ManifestCamera,
    /// `"ok"` / `"check_failed"` / `"error"`。
    pub outcome: String,
    pub messages: Vec<String>,
    pub screenshot: Option<ManifestShot>,
}

/// 相机位姿(角度为度)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManifestCamera {
    pub pos: [f32; 3],
    pub yaw: f32,
    pub pitch: f32,
}

/// 截图信息。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManifestShot {
    pub file: String,
    pub name: String,
    pub expect: String,
    pub hud: bool,
    pub live_entities: usize,
    pub chunks_loaded: usize,
}

/// 场景结果。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManifestResult {
    /// `"done"` / `"failed"`。
    pub status: String,
    pub exit_code: i32,
    pub failed_checks: usize,
    pub reason: Option<String>,
}

impl Manifest {
    pub fn new(scenario: &Scenario) -> Self {
        let _ = scenario;
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }

    pub fn push(&mut self, record: &StepRecord, world: &dyn WorldQuery) {
        let _ = (record, world);
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }

    pub fn finish(&mut self, status: &EngineStatus) {
        let _ = status;
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }

    pub fn to_json(&self) -> String {
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }

    pub fn from_json(text: &str) -> Result<Self, String> {
        let _ = text;
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }
}
