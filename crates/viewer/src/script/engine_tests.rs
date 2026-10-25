//! script 模块单元测试(纪律 11、12):引擎状态机与检查项的关键分支,
//! 每条锚定一个"功能失效会变红"的实现点;另含解析/引擎测试共用的文本工具。

use crate::script::{
    CameraPose, Engine, EngineStatus, FrameInput, ScreenshotRequest, StepOutcome, WorldQuery,
    parse_scenario,
};
use types::EntityId;
use types::glam::Vec3;

pub(crate) const HEADER: &str = "name = \"t\"\ndescription = \"d\"\n";

/// steps 可含多个 `[[step]]` 块。
pub(crate) fn scenario_text(steps: &str) -> String {
    format!("{HEADER}{steps}")
}

pub(crate) fn eid(n: u32) -> EntityId {
    EntityId::from_parts(0, n)
}

/// 假世界:全范围装载,y ≤ −61 为石头,其余空气;可逐格覆盖。
#[derive(Default)]
struct FlatWorld {
    entities: Vec<(EntityId, Vec3)>,
    chunks: usize,
    overrides: std::collections::BTreeMap<(i32, i32, i32), types::BlockId>,
}

impl FlatWorld {
    fn with_entities(entities: &[(EntityId, Vec3)]) -> Self {
        Self {
            entities: entities.to_vec(),
            chunks: 4,
            ..Self::default()
        }
    }
}

impl WorldQuery for FlatWorld {
    fn live_entities(&self) -> Vec<(EntityId, Vec3)> {
        self.entities.clone()
    }
    fn block_at(&self, pos: types::BlockPos) -> Option<types::BlockId> {
        if let Some(b) = self.overrides.get(&(pos.x, pos.y, pos.z)) {
            return Some(*b);
        }
        Some(if pos.y <= -61 {
            types::BlockId(1)
        } else {
            types::BlockId(0)
        })
    }
    fn chunks_loaded(&self) -> usize {
        self.chunks
    }
}

const STONE: types::BlockId = types::BlockId(1);
const AIR: types::BlockId = types::BlockId(0);

fn start_pose() -> CameraPose {
    CameraPose {
        pos: Vec3::new(0.0, -50.0, 0.0),
        yaw: 0.0,
        pitch: 0.0,
    }
}

fn engine(steps: &str) -> Engine {
    Engine::new(
        parse_scenario(&scenario_text(steps)).expect("场景应可解析"),
        start_pose(),
    )
}

fn frame(
    e: &mut Engine,
    dt: f32,
    tick: u64,
    world: &FlatWorld,
    spawned: &[EntityId],
) -> crate::script::FrameOutput {
    e.frame(&FrameInput {
        dt,
        server_tick: tick,
        world,
        spawned,
    })
}

/// look_at 重合目标保持原朝向,不得产生 NaN(细则 5)。
#[test]
fn look_at_same_point_keeps_orientation() {
    let mut e = engine(
        "[[step]]
look_at = { target = [0.0, -50.0, 0.0] }",
    );
    let w = FlatWorld::with_entities(&[]);
    let out = frame(&mut e, 0.016, 0, &w, &[]);
    assert_eq!(out.finished.len(), 1, "重合目标也当帧结束");
    assert!(!out.camera.yaw.is_nan() && !out.camera.pitch.is_nan());
    assert_eq!(out.camera.yaw, 0.0, "重合时保持原 yaw");
    assert_eq!(out.camera.pitch, 0.0, "重合时保持原 pitch");
}

/// look_at 正上方:水平距离为 0 时 yaw 保持,pitch = ±90。
#[test]
fn look_at_straight_up_keeps_yaw() {
    let mut e = engine(
        "[[step]]
look_at = { target = [0.0, -40.0, 0.0] }",
    );
    let w = FlatWorld::with_entities(&[]);
    let out = frame(&mut e, 0.016, 0, &w, &[]);
    assert!((out.camera.pitch - 90.0).abs() < 1e-3, "正上方 pitch 90°");
    assert_eq!(out.camera.yaw, 0.0, "水平分量为零时 yaw 不动");
    assert!(!out.camera.forward().is_nan());
}

/// fly 的 dt 推进:首帧就按该帧 dt 移动 speed·dt(细则 4)。
#[test]
fn fly_first_frame_advances_by_dt() {
    let mut e = engine(
        "[[step]]
fly = { to = [100.0, -50.0, 0.0] }",
    );
    let w = FlatWorld::with_entities(&[]);
    let out = frame(&mut e, 0.5, 0, &w, &[]);
    assert!(!out.finished.is_empty() || out.camera.pos.x > 0.0);
    assert!(
        (out.camera.pos.x - 8.0).abs() < 1e-3,
        "0.5 s × 16 m/s = 8 m,实为 {}",
        out.camera.pos.x
    );
}

/// wait_until 超时用 `>=`(细则 3):恰好在 start + timeout 那拍失败。
#[test]
fn wait_until_timeout_at_exact_boundary() {
    let mut e = engine(
        "[[step]]
wait_until = { live_entities = { eq = 0 }, timeout_ticks = 10 }",
    );
    let w = FlatWorld::with_entities(&[(eid(1), Vec3::new(0.5, -59.5, 0.5))]);
    frame(&mut e, 0.016, 100, &w, &[]);
    frame(&mut e, 0.016, 105, &w, &[]);
    let out = frame(&mut e, 0.016, 110, &w, &[]);
    assert!(matches!(e.status(), EngineStatus::Failed { step: 1, .. }));
    assert!(
        out.finished
            .last()
            .map(|r| matches!(r.outcome, crate::script::StepOutcome::Error(_)))
            .unwrap_or(false),
        "拍 110 = 起点 100 + 10,应超时"
    );
}

/// spawn 回执超时边界:start + 120 拍整点失败(细则 6)。
#[test]
fn spawn_receipt_timeout_boundary() {
    let mut e = engine(
        "[[step]]
spawn = { at = [0.0, -50.0, 0.0] }",
    );
    let w = FlatWorld::with_entities(&[]);
    frame(&mut e, 0.016, 100, &w, &[]);
    let out = frame(&mut e, 0.016, 219, &w, &[]);
    assert!(out.finished.is_empty(), "拍 219 = 100 + 119,还差一拍");
    frame(&mut e, 0.016, 220, &w, &[]);
    assert!(
        matches!(e.status(), EngineStatus::Failed { step: 1, .. }),
        "拍 220 = 100 + 120,回执超时 → 失败"
    );
}

/// spawn 网格第 5 个(下标 i = 5)折行到 z + spacing;基点用绝对坐标(细则 6)。
#[test]
fn spawn_grid_wraps_every_five() {
    let mut e = engine(
        "[[step]]\nspawn = { at = [10.0, -50.0, 20.0], count = 7, spacing = 2.0, velocity = [0.0, 1.0, 0.0] }",
    );
    let w = FlatWorld::with_entities(&[]);
    let out = frame(&mut e, 0.016, 0, &w, &[]);
    let at: Vec<Vec3> = out
        .commands
        .iter()
        .map(|c| match c {
            crate::script::EngineCommand::Spawn { state } => state.translation,
            other => panic!("只应有生成命令:{other:?}"),
        })
        .collect();
    assert_eq!(at.len(), 7);
    // i=0..7:x = 10 + (i%5)*2,z = 20 + (i/5)*2。
    let want = |i: f32| {
        Vec3::new(
            10.0 + (i as i32 % 5) as f32 * 2.0,
            -50.0,
            20.0 + (i as i32 / 5) as f32 * 2.0,
        )
    };
    for (i, p) in at.iter().enumerate() {
        assert!(
            (*p - want(i as u32 as f32)).length() < 1e-4,
            "第 {i} 个位置 {p:?} 应为 {:?}",
            want(i as f32)
        );
    }
}

/// 截图帧之后,下一帧才继续推进(camera 已在截图前生效,细则 2)。
#[test]
fn screenshot_delays_following_wait_step() {
    let mut e = engine(
        "[[step]]\nscreenshot = { name = \"a\", expect = \"x\" }\n\n[[step]]\nwait_ticks = { n = 5 }",
    );
    let w = FlatWorld::with_entities(&[]);
    let out = frame(&mut e, 0.016, 100, &w, &[]);
    assert_eq!(out.finished.len(), 1, "截图帧不推进 wait_ticks");
    assert_eq!(out.finished[0].action, "screenshot");
    let out = frame(&mut e, 0.016, 101, &w, &[]);
    assert!(out.finished.is_empty(), "wait_ticks 从拍 101 起,还需等");
    let out = frame(&mut e, 0.016, 106, &w, &[]);
    assert_eq!(out.finished.len(), 1, "拍 106 = 101 + 5,wait_ticks 结束");
    assert_eq!(out.finished[0].action, "wait_ticks");
}

/// no_floating:柱内最高实心方块取"从底面向下"的第一块(下方被挖空不影响);
/// 实心判断用内置注册表(air 非实心)。
#[test]
fn no_floating_finds_highest_solid_below_bottom() {
    // 先等 2 拍攒足历史,再 check:实体中心 (0.5, −58.0, 0.5),底面 −58.5,
    // 柱 (0,0) 顶面 −60 → 高出 1.5,不 > 1.5 → Ok(严格 >)。
    let mut e = engine(
        "[[step]]\nwait_ticks = { n = 2 }\n\n[[step]]\ncheck = { no_floating = { height = 1.5, still_ticks = 2 } }",
    );
    let w = FlatWorld::with_entities(&[(eid(1), Vec3::new(0.5, -58.0, 0.5))]);
    frame(&mut e, 0.016, 0, &w, &[]);
    let out = frame(&mut e, 0.016, 1, &w, &[]);
    assert!(out.finished.is_empty(), "wait_ticks 未结束");
    let out = frame(&mut e, 0.016, 2, &w, &[]);
    assert_eq!(
        out.finished.first().map(|r| r.outcome.clone()),
        Some(StepOutcome::Ok),
        "高出恰 1.5 不算悬空(严格 >)"
    );

    // 新场景:把 (0,−61,0) 挖掉,柱顶面变 −62 的顶 −61 → 底面 −58.5 高出 2.5 > 1.5 → 判悬空。
    let mut w2 = w.clone_shallow();
    w2.overrides.insert((0, -61, 0), AIR);
    let mut e2 = engine(
        "[[step]]\nwait_ticks = { n = 2 }\n\n[[step]]\ncheck = { no_floating = { height = 1.5, still_ticks = 2 } }",
    );
    frame(&mut e2, 0.016, 0, &w2, &[]);
    frame(&mut e2, 0.016, 1, &w2, &[]);
    let out = frame(&mut e2, 0.016, 2, &w2, &[]);
    assert!(
        matches!(
            out.finished
                .iter()
                .find(|r| r.action == "check")
                .map(|r| r.outcome.clone()),
            Some(StepOutcome::CheckFailed(_))
        ),
        "挖掉地表后应判悬空"
    );
    let _ = STONE;
}

/// 场景失败后不再推进:后续帧无命令、无截图、无记录(细则 11)。
#[test]
fn failed_engine_stays_frozen() {
    let mut e = engine(
        "[[step]]
detonate = { nearest = true, power = 1.0, radius = 1.0 }",
    );
    let w = FlatWorld::with_entities(&[]);
    frame(&mut e, 0.016, 0, &w, &[]);
    assert!(matches!(e.status(), EngineStatus::Failed { .. }));
    let out = frame(&mut e, 0.016, 5, &w, &[eid(3)]);
    assert!(out.commands.is_empty() && out.screenshot.is_none() && out.finished.is_empty());
    assert_eq!(out.camera, start_pose(), "结束后返回当前相机");
    assert_eq!(e.exit_code(), Some(2));
}

/// 截图文件名两位补零:步骤 3 → "03-x.png"(细则 1)。
#[test]
fn screenshot_file_name_zero_pads_step() {
    let req = ScreenshotRequest {
        step: 3,
        name: "x".to_owned(),
        expect: String::new(),
        hud: false,
    };
    assert_eq!(req.file_name(), "03-x.png");
    let req12 = ScreenshotRequest {
        step: 12,
        name: "y".to_owned(),
        expect: String::new(),
        hud: false,
    };
    assert_eq!(req12.file_name(), "12-y.png");
}

/// 未知标签按 0 个(细则 8):labeled_alive 引用没记过的标签 → 检查失败(非 Ok)。
#[test]
fn labeled_alive_unknown_label_counts_zero() {
    let mut e = engine(
        "[[step]]
check = { labeled_alive = { label = \"ghost\", ge = 1 } }",
    );
    let w = FlatWorld::with_entities(&[(eid(1), Vec3::new(0.5, -59.5, 0.5))]);
    let out = frame(&mut e, 0.016, 0, &w, &[]);
    assert!(matches!(
        out.finished.first().map(|r| r.outcome.clone()),
        Some(crate::script::StepOutcome::CheckFailed(_))
    ));
}

/// FlatWorld 浅拷贝工具(overrides 可分叉)。
impl FlatWorld {
    fn clone_shallow(&self) -> Self {
        Self {
            entities: self.entities.clone(),
            chunks: self.chunks,
            overrides: self.overrides.clone(),
        }
    }
}
