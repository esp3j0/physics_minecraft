//! manifest —— 场景运行记录(REQ-008 §4):viewer 脚本模式写出,编排器与审图器读取。
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

use super::{EngineStatus, Scenario, StepOutcome, StepRecord, WorldQuery};

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
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
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

/// `StepOutcome` → (outcome 字符串, messages)。
fn outcome_fields(outcome: &StepOutcome) -> (String, Vec<String>) {
    match outcome {
        StepOutcome::Ok => ("ok".to_owned(), Vec::new()),
        StepOutcome::CheckFailed(msgs) => ("check_failed".to_owned(), msgs.clone()),
        StepOutcome::Error(reason) => ("error".to_owned(), vec![reason.clone()]),
    }
}

impl Manifest {
    pub fn new(scenario: &Scenario) -> Self {
        Self {
            scenario: scenario.name.clone(),
            req: scenario.req.clone(),
            description: scenario.description.clone(),
            steps: Vec::new(),
            result: None,
        }
    }

    pub fn push(&mut self, record: &StepRecord, world: &dyn WorldQuery) {
        let (outcome, messages) = outcome_fields(&record.outcome);
        let screenshot = record.screenshot.as_ref().map(|shot| ManifestShot {
            file: shot.file_name(),
            name: shot.name.clone(),
            expect: shot.expect.clone(),
            hud: shot.hud,
            live_entities: world.live_entities().len(),
            chunks_loaded: world.chunks_loaded(),
        });
        self.steps.push(ManifestStep {
            step: record.step,
            label: record.label.clone(),
            action: record.action.clone(),
            start_tick: record.start_tick,
            end_tick: record.end_tick,
            camera: ManifestCamera {
                pos: [
                    record.camera.pos.x,
                    record.camera.pos.y,
                    record.camera.pos.z,
                ],
                yaw: record.camera.yaw,
                pitch: record.camera.pitch,
            },
            outcome,
            messages,
            screenshot,
        });
    }

    pub fn finish(&mut self, status: &EngineStatus) {
        let result = match status {
            EngineStatus::Done { failed_checks } => ManifestResult {
                status: "done".to_owned(),
                exit_code: if *failed_checks == 0 { 0 } else { 1 },
                failed_checks: *failed_checks,
                reason: None,
            },
            EngineStatus::Failed { reason, .. } => {
                let failed_checks = self
                    .steps
                    .iter()
                    .filter(|s| s.outcome == "check_failed")
                    .count();
                ManifestResult {
                    status: "failed".to_owned(),
                    exit_code: 2,
                    failed_checks,
                    reason: Some(reason.clone()),
                }
            }
            EngineStatus::Running { .. } => return, // 未结束:不写结果
        };
        self.result = Some(result);
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("manifest 序列化不可能失败(纯数据结构)")
    }

    pub fn from_json(text: &str) -> Result<Self, String> {
        serde_json::from_str(text).map_err(|e| format!("manifest JSON 解析失败:{e}"))
    }
}

#[cfg(test)]
mod tests {
    use types::EntityId;
    use types::glam::Vec3;

    use super::*;
    use crate::script::{CameraPose, parse_scenario};

    /// 假世界:固定实体表与 chunk 数(只为 push 统计与引擎驱动)。
    struct FakeWorld {
        entities: Vec<(EntityId, Vec3)>,
        chunks: usize,
    }

    impl FakeWorld {
        fn new(entities: Vec<(EntityId, Vec3)>, chunks: usize) -> Self {
            Self { entities, chunks }
        }
    }

    impl WorldQuery for FakeWorld {
        fn live_entities(&self) -> Vec<(EntityId, Vec3)> {
            self.entities.clone()
        }
        fn block_at(&self, _pos: types::BlockPos) -> Option<types::BlockId> {
            None
        }
        fn chunks_loaded(&self) -> usize {
            self.chunks
        }
    }

    fn start() -> CameraPose {
        CameraPose {
            pos: Vec3::new(0.0, -50.0, 0.0),
            yaw: 0.0,
            pitch: 0.0,
        }
    }

    /// 最小场景(camera + check 失败)跑一帧,push 进 manifest 并收尾。
    fn sample() -> Manifest {
        let text = "name = \"t\"\nreq = \"REQ-008\"\ndescription = \"d\"\n\n[[step]]\ncamera = \
                    { pos = [1.0, -50.0, 2.0], yaw = 10.0, pitch = 0.0 }\n\n[[step]]\ncheck = \
                    { live_entities = { eq = 0 } }\n";
        let scenario = parse_scenario(text).expect("可解析");
        let mut m = Manifest::new(&scenario);
        assert!(m.result.is_none(), "new 后 result 为 None");
        assert!(m.steps.is_empty(), "new 后步骤为空");
        assert_eq!(
            (
                m.scenario.as_str(),
                m.req.as_deref(),
                m.description.as_str()
            ),
            ("t", Some("REQ-008"), "d"),
            "new 转写场景元数据"
        );

        let world = FakeWorld::new(vec![(EntityId::from_parts(0, 1), Vec3::ONE)], 7);
        let mut e = crate::script::Engine::new(scenario, start());
        let out = e.frame(&crate::script::FrameInput {
            dt: 0.016,
            server_tick: 41,
            world: &world,
            spawned: &[],
        });
        assert_eq!(out.finished.len(), 2, "两个即时步骤同帧结束");
        for r in &out.finished {
            m.push(r, &world);
        }
        m.finish(e.status());
        m
    }

    /// 字段转写:outcome 三态映射、失败原因进 messages、相机与拍号逐字段一致、
    /// Done 结果(转写错位任一处即红)。
    #[test]
    fn manifest_push_transcribes_record_fields() {
        let m = sample();
        assert_eq!(m.steps.len(), 2);
        let first = &m.steps[0];
        assert_eq!((first.step, first.action.as_str()), (1, "camera"));
        assert_eq!(first.label, None);
        assert_eq!((first.start_tick, first.end_tick), (41, 41));
        assert_eq!(first.camera.pos, [1.0, -50.0, 2.0]);
        assert_eq!((first.camera.yaw, first.camera.pitch), (10.0, 0.0));
        assert_eq!(first.outcome, "ok");
        assert!(first.messages.is_empty());
        assert!(first.screenshot.is_none());
        let second = &m.steps[1];
        assert_eq!(second.action, "check");
        assert_eq!(second.outcome, "check_failed");
        assert!(!second.messages.is_empty(), "check 失败原因进 messages");

        let result = m.result.as_ref().expect("finish 已写结果");
        assert_eq!((result.status.as_str(), result.exit_code), ("done", 1));
        assert_eq!(result.failed_checks, 1);
        assert_eq!(result.reason, None);
    }

    /// 截图步骤:file 为 `NN-<name>.png`,统计取 push 当时 world 的值,hud 缺省 false。
    #[test]
    fn manifest_push_records_screenshot_stats() {
        let live = vec![(EntityId::from_parts(0, 1), Vec3::ONE)];
        let world = FakeWorld::new(live, 9);
        let text = "name = \"s\"\ndescription = \"d\"\n\n[[step]]\nscreenshot = { name = \"a\", \
                    expect = \"e\" }\n";
        let scenario = parse_scenario(text).expect("可解析");
        let mut m = Manifest::new(&scenario);
        let mut e = crate::script::Engine::new(scenario, start());
        let out = e.frame(&crate::script::FrameInput {
            dt: 0.016,
            server_tick: 5,
            world: &world,
            spawned: &[],
        });
        let record = out.finished.first().expect("截图步当帧结束");
        let request = record.screenshot.as_ref().expect("记录带截图请求");
        assert_eq!(request.file_name(), "01-a.png");
        m.push(record, &world);
        let shot = m.steps[0].screenshot.as_ref().expect("截图步带 screenshot");
        assert_eq!(shot.file, "01-a.png");
        assert_eq!(
            (shot.name.as_str(), shot.expect.as_str(), shot.hud),
            ("a", "e", false)
        );
        assert_eq!(shot.live_entities, 1, "统计取 push 时 world");
        assert_eq!(shot.chunks_loaded, 9);
    }

    /// Failed 收尾:status failed、exit_code 2、reason 非空、failed_checks 为已记录
    /// 步骤中 check_failed 的个数;Running 不写结果。
    #[test]
    fn manifest_finish_failed_and_running() {
        // 一步 check 失败(不终止场景)+ 一步 detonate nearest 无实体(步骤错误 → Failed)。
        let text = "name = \"f\"\ndescription = \"d\"\n\n[[step]]\ncheck = { live_entities = \
                    { eq = 9 } }\n\n[[step]]\ndetonate = { nearest = true, power = 12.0, radius \
                    = 8.0 }\n";
        let scenario = parse_scenario(text).expect("可解析");
        let mut m = Manifest::new(&scenario);
        let world = FakeWorld::new(vec![], 0);
        let mut e = crate::script::Engine::new(scenario, start());
        let out = e.frame(&crate::script::FrameInput {
            dt: 0.016,
            server_tick: 1,
            world: &world,
            spawned: &[],
        });
        for r in &out.finished {
            m.push(r, &world);
        }
        assert!(
            matches!(e.status(), EngineStatus::Failed { step: 2, .. }),
            "detonate 无实体 → 场景失败,实为 {:?}",
            e.status()
        );
        m.finish(&EngineStatus::Running { step: 2 });
        assert!(m.result.is_none(), "Running 不写结果");
        m.finish(e.status());
        let result = m.result.as_ref().expect("Failed 写结果");
        assert_eq!(result.status, "failed");
        assert_eq!(result.exit_code, 2);
        assert_eq!(
            result.failed_checks, 1,
            "Failed 的 failed_checks 数已记录的 check_failed 步骤"
        );
        assert!(
            result.reason.as_deref().is_some_and(|r| !r.is_empty()),
            "失败带原因"
        );
    }

    /// JSON 字段名即结构体字段名;往返相等;残缺报错。
    #[test]
    fn manifest_json_roundtrip() {
        let m = sample();
        let json = m.to_json();
        let v: serde_json::Value = serde_json::from_str(&json).expect("合法 JSON");
        assert_eq!(v["scenario"], "t");
        assert_eq!(v["req"], "REQ-008");
        assert_eq!(v["description"], "d");
        assert_eq!(v["steps"].as_array().map(|s| s.len()), Some(2));
        assert_eq!(v["result"]["status"], "done");
        assert_eq!(v["result"]["exit_code"], 1);
        assert_eq!(Manifest::from_json(&json), Ok(m), "from_json 读回相等的值");
        assert!(Manifest::from_json("{}").is_err(), "残缺 JSON 报错");
    }

    /// 场景失败(det error)时已记录步骤 outcome = error、messages 带原因
    /// (acc_s4_manifest_failed_scenario 之外的冗余锚)。
    #[test]
    fn manifest_records_error_outcome_for_failed_step() {
        let text = "name = \"f\"\ndescription = \"d\"\n\n[[step]]\ndetonate = { nearest = true, \
                    power = 12.0, radius = 8.0 }\n";
        let scenario = parse_scenario(text).expect("可解析");
        let mut m = Manifest::new(&scenario);
        let world = FakeWorld::new(vec![], 0);
        let mut e = crate::script::Engine::new(scenario, start());
        let out = e.frame(&crate::script::FrameInput {
            dt: 0.016,
            server_tick: 2,
            world: &world,
            spawned: &[],
        });
        for r in &out.finished {
            m.push(r, &world);
        }
        m.finish(e.status());
        assert_eq!(m.steps[0].outcome, "error");
        assert_eq!(m.steps[0].messages.len(), 1);
        assert!(!m.steps[0].messages[0].is_empty());
        let result = m.result.as_ref().expect("结果已写");
        assert_eq!(result.status, "failed");
        assert_eq!(result.failed_checks, 0, "error 步骤不计 check_failed");
    }
}
