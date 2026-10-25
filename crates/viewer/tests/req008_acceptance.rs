//! REQ-008 验收用例(总架构师编写;issue #16)。
//!
//! 约定同 REQ-007:实施方**不得修改断言**,实现完成后只去掉对应单元用例上的 `#[ignore]`;
//! 认为断言与需求不符时在 #16 提出,由总架构师裁定。每个用例注释写明"在什么错误实现下
//! 会变红"(纪律 12)。
//!
//! 单元 01 的用例只驱动 `viewer::script` 的公开接口(解析 + 引擎 + 检查),用假世界代替
//! 渲染层,不开窗口。坐标约定:生产超平坦地表最高实心方块 y = −61,顶面 y = −60;
//! 半尺寸 0.5 的实体贴地时中心 y = −59.5。

use std::collections::BTreeMap;

use types::glam::Vec3;
use types::{BlockId, BlockPos, EntityId};
use viewer::script::{
    Action, CameraPose, Check, Cmp, DetonateAt, Engine, EngineCommand, EngineStatus, FrameInput,
    FrameOutput, Scenario, SpawnAt, StepOutcome, WorldQuery, parse_scenario,
};

const AIR: BlockId = BlockId(0);
const STONE: BlockId = BlockId(1);
const DIRT: BlockId = BlockId(2);

fn eid(n: u32) -> EntityId {
    EntityId::from_parts(0, n)
}

/// 假世界:|x|、|z| < 256 的范围已装载,y ≤ −61 为石头,其余为空气;可逐格覆盖。
#[derive(Clone, Default)]
struct World {
    entities: Vec<(EntityId, Vec3)>,
    chunks: usize,
    overrides: BTreeMap<(i32, i32, i32), BlockId>,
}

impl World {
    fn with_entities(entities: &[(EntityId, Vec3)]) -> Self {
        Self {
            entities: entities.to_vec(),
            chunks: 9,
            ..Self::default()
        }
    }
}

impl WorldQuery for World {
    fn live_entities(&self) -> Vec<(EntityId, Vec3)> {
        self.entities.clone()
    }
    fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
        if pos.x.abs() >= 256 || pos.z.abs() >= 256 {
            return None;
        }
        if let Some(b) = self.overrides.get(&(pos.x, pos.y, pos.z)) {
            return Some(*b);
        }
        Some(if pos.y <= -61 { STONE } else { AIR })
    }
    fn chunks_loaded(&self) -> usize {
        self.chunks
    }
}

fn start_pose() -> CameraPose {
    CameraPose {
        pos: Vec3::new(0.0, -50.0, 0.0),
        yaw: 0.0,
        pitch: 0.0,
    }
}

fn engine(text: &str) -> Engine {
    Engine::new(parse_scenario(text).expect("场景应可解析"), start_pose())
}

/// 推进一帧。
fn frame(e: &mut Engine, dt: f32, tick: u64, world: &World, spawned: &[EntityId]) -> FrameOutput {
    e.frame(&FrameInput {
        dt,
        server_tick: tick,
        world,
        spawned,
    })
}

fn assert_vec3(got: Vec3, want: Vec3, what: &str) {
    assert!(
        (got - want).length() < 1e-3,
        "{what}:得到 {got:?},应为 {want:?}"
    );
}

const HEADER: &str = "name = \"t\"\ndescription = \"d\"\n";

fn scenario(steps: &str) -> String {
    format!("{HEADER}{steps}")
}

// ===========================================================================
// 单元 01 · AC-S1 脚本解析
// ===========================================================================

/// 全部动作与检查项都能解析,字段逐个落到正确的值(方块名经内置注册表解析为 ID)。
///
/// 会变红的实现:漏掉某个动作或检查项;角度/坐标字段错位;方块名没有解析成 ID;
/// `[server]` 覆盖项丢失。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s1_parses_all_actions_and_checks() {
    let text = r#"
name = "all"
req = "REQ-007"
description = "全部动作"
timeout_secs = 240

[server]
world_seed = 7

[viewer]
view_distance = 6

[[step]]
label = "就位"
camera = { pos = [8.0, -48.0, 24.0], yaw = -90.0, pitch = -30.0 }

[[step]]
look_at = { target = [8.0, -60.0, 10.0] }

[[step]]
fly = { to = [80.0, -48.0, 24.0], speed = 20.0 }

[[step]]
couple = { on = false }

[[step]]
spawn = { ahead = 8.0, lift = 6.0, count = 8, spacing = 2.0, velocity = [0.0, 3.0, 0.0], label = "blast" }

[[step]]
spawn = { at = [1.0, -50.0, 2.0] }

[[step]]
detonate = { nearest = true, power = 12.0, radius = 8.0 }

[[step]]
detonate = { at = [3.0, -59.0, 4.0], power = 6.0, radius = 4.0 }

[[step]]
set_block = { pos = [1, -61, 2], block = "air" }

[[step]]
wait_ticks = { n = 60 }

[[step]]
wait_until = { live_entities = { eq = 0 }, timeout_ticks = 900 }

[[step]]
check = { labeled_alive = { label = "blast", le = 2 }, block = { pos = [1, -61, 2], is = "dirt" }, chunks_loaded = { ge = 9 }, no_floating = { height = 1.5, still_ticks = 30 } }

[[step]]
screenshot = { name = "return", expect = "地面上有方块", hud = true }
"#;
    let s: Scenario = parse_scenario(text).expect("全动作场景应可解析");
    assert_eq!(s.name, "all");
    assert_eq!(s.req.as_deref(), Some("REQ-007"));
    assert_eq!(s.description, "全部动作");
    assert_eq!(s.timeout_secs, 240);
    assert_eq!(
        s.server.get("world_seed").and_then(|v| v.as_integer()),
        Some(7),
        "[server] 覆盖项原样保留"
    );
    assert_eq!(s.view_distance, Some(6));
    assert_eq!(s.steps.len(), 13);
    assert_eq!(s.steps[0].label.as_deref(), Some("就位"));
    assert_eq!(
        s.steps[0].action,
        Action::Camera {
            pos: Vec3::new(8.0, -48.0, 24.0),
            yaw: -90.0,
            pitch: -30.0
        }
    );
    assert_eq!(
        s.steps[1].action,
        Action::LookAt {
            target: Vec3::new(8.0, -60.0, 10.0)
        }
    );
    assert_eq!(
        s.steps[2].action,
        Action::Fly {
            to: Vec3::new(80.0, -48.0, 24.0),
            speed: 20.0
        }
    );
    assert_eq!(s.steps[3].action, Action::Couple { on: false });
    assert_eq!(
        s.steps[4].action,
        Action::Spawn {
            at: SpawnAt::Ahead {
                ahead: 8.0,
                lift: 6.0
            },
            count: 8,
            spacing: 2.0,
            velocity: Vec3::new(0.0, 3.0, 0.0),
            label: Some("blast".to_owned()),
        }
    );
    assert_eq!(
        s.steps[5].action,
        Action::Spawn {
            at: SpawnAt::At(Vec3::new(1.0, -50.0, 2.0)),
            count: 1,
            spacing: 1.6,
            velocity: Vec3::ZERO,
            label: None,
        },
        "spawn 缺省:count 1、spacing 1.6、velocity 0"
    );
    assert_eq!(
        s.steps[6].action,
        Action::Detonate {
            at: DetonateAt::Nearest,
            power: 12.0,
            radius: 8.0
        }
    );
    assert_eq!(
        s.steps[7].action,
        Action::Detonate {
            at: DetonateAt::At(Vec3::new(3.0, -59.0, 4.0)),
            power: 6.0,
            radius: 4.0
        }
    );
    assert_eq!(
        s.steps[8].action,
        Action::SetBlock {
            pos: BlockPos::new(1, -61, 2),
            block: AIR
        }
    );
    assert_eq!(s.steps[9].action, Action::WaitTicks { n: 60 });
    assert_eq!(
        s.steps[10].action,
        Action::WaitUntil {
            checks: vec![Check::LiveEntities(Cmp::Eq(0))],
            timeout_ticks: 900
        }
    );
    match &s.steps[11].action {
        Action::Check { checks } => {
            assert_eq!(checks.len(), 4, "四个检查项都在");
            assert!(checks.contains(&Check::LabeledAlive {
                label: "blast".to_owned(),
                cmp: Cmp::Le(2)
            }));
            assert!(checks.contains(&Check::Block {
                pos: BlockPos::new(1, -61, 2),
                is: DIRT
            }));
            assert!(checks.contains(&Check::ChunksLoaded(Cmp::Ge(9))));
            assert!(checks.contains(&Check::NoFloating {
                height: 1.5,
                still_ticks: 30
            }));
        }
        other => panic!("第 12 步应为 check,实为 {other:?}"),
    }
    assert_eq!(
        s.steps[12].action,
        Action::Screenshot {
            name: "return".to_owned(),
            expect: "地面上有方块".to_owned(),
            hud: true
        }
    );
}

/// 缺省值:`timeout_secs` 180、`[server]` 空表、`view_distance` None、`fly.speed` 16、
/// `screenshot.hud` false、`label` None。
///
/// 会变红的实现:缺省值写错;可选字段被当成必填。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s1_defaults() {
    let s = parse_scenario(&scenario(
        r#"
[[step]]
fly = { to = [1.0, 2.0, 3.0] }

[[step]]
screenshot = { name = "a", expect = "b" }
"#,
    ))
    .expect("最小场景应可解析");
    assert_eq!(s.req, None);
    assert_eq!(s.timeout_secs, 180);
    assert!(s.server.is_empty());
    assert_eq!(s.view_distance, None);
    assert_eq!(s.steps[0].label, None);
    assert_eq!(
        s.steps[0].action,
        Action::Fly {
            to: Vec3::new(1.0, 2.0, 3.0),
            speed: 16.0
        }
    );
    assert_eq!(
        s.steps[1].action,
        Action::Screenshot {
            name: "a".to_owned(),
            expect: "b".to_owned(),
            hud: false
        }
    );
}

/// 各类错误都报错,并报出出错步骤的序号(从 1 开始;顶层错误为 None)。
///
/// 会变红的实现:未知键被静默忽略;一步多个动作只取其一;错误不带步骤序号或序号从 0 开始;
/// 未知方块名被当成空气。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s1_errors_report_step_index() {
    let ok_step = "[[step]]\nwait_ticks = { n = 1 }\n";
    let cases: Vec<(String, Option<usize>, &str)> = vec![
        (format!("{HEADER}foo = 1\n{ok_step}"), None, "未知顶层键"),
        (format!("description = \"d\"\n{ok_step}"), None, "缺 name"),
        (
            scenario(&format!(
                "{ok_step}[[step]]\nteleport = {{ pos = [0.0, 0.0, 0.0] }}\n"
            )),
            Some(2),
            "未知动作",
        ),
        (
            scenario("[[step]]\nwait_ticks = { n = 1 }\ncouple = { on = true }\n"),
            Some(1),
            "一步两个动作",
        ),
        (
            scenario("[[step]]\nlabel = \"空\"\n"),
            Some(1),
            "一步没有动作",
        ),
        (
            scenario("[[step]]\ncamera = { yaw = 0.0, pitch = 0.0 }\n"),
            Some(1),
            "camera 缺 pos",
        ),
        (
            scenario(
                "[[step]]\ncamera = { pos = [0.0, 0.0, 0.0], yaw = 0.0, pitch = 0.0, zoom = 2.0 }\n",
            ),
            Some(1),
            "动作内未知键",
        ),
        (
            scenario(&format!(
                "{ok_step}{ok_step}[[step]]\nwait_ticks = {{ n = \"x\" }}\n"
            )),
            Some(3),
            "类型错误",
        ),
        (
            scenario("[[step]]\nset_block = { pos = [0, 0, 0], block = \"lava\" }\n"),
            Some(1),
            "未知方块名",
        ),
        (
            scenario("[[step]]\ncheck = { live_entities = { eq = 1, le = 2 } }\n"),
            Some(1),
            "比较写了两个",
        ),
        (
            scenario("[[step]]\nspawn = { at = [0.0, 0.0, 0.0], ahead = 8.0, lift = 6.0 }\n"),
            Some(1),
            "spawn 同时给 at 与 ahead",
        ),
    ];
    for (text, step, what) in cases {
        match parse_scenario(&text) {
            Ok(_) => panic!("{what}:应报错"),
            Err(e) => {
                assert_eq!(e.step, step, "{what}:步骤序号");
                assert!(!e.message.is_empty(), "{what}:错误信息不应为空");
                if let Some(n) = step {
                    assert!(
                        e.to_string().contains(&format!("第 {n} 步")),
                        "{what}:Display 应带步骤序号:{e}"
                    );
                }
            }
        }
    }
}

// ===========================================================================
// 单元 01 · AC-S2 引擎状态机
// ===========================================================================

/// `camera` 与 `look_at` 当帧生效,同一帧内连续执行;`look_at` 按约定换算偏航与俯仰。
///
/// 会变红的实现:即时步骤要等下一帧;`look_at` 的角度换算与相机朝向约定不一致。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_camera_and_look_at_apply_immediately() {
    let mut e = engine(&scenario(
        r#"
[[step]]
camera = { pos = [8.0, -48.0, 24.0], yaw = 10.0, pitch = 0.0 }
[[step]]
look_at = { target = [8.0, -53.0, 29.0] }
"#,
    ));
    let w = World::with_entities(&[]);
    let out = frame(&mut e, 0.016, 100, &w, &[]);
    assert_vec3(out.camera.pos, Vec3::new(8.0, -48.0, 24.0), "相机位置");
    assert!(
        (out.camera.yaw - 90.0).abs() < 1e-3,
        "偏航应为 90°:{}",
        out.camera.yaw
    );
    assert!(
        (out.camera.pitch + 45.0).abs() < 1e-3,
        "俯仰应为 −45°:{}",
        out.camera.pitch
    );
    assert_vec3(
        out.camera.forward(),
        Vec3::new(0.0, -0.5_f32.sqrt(), 0.5_f32.sqrt()),
        "前向指向目标",
    );
    let actions: Vec<&str> = out.finished.iter().map(|r| r.action.as_str()).collect();
    assert_eq!(actions, ["camera", "look_at"], "两个即时步骤同一帧结束");
    assert_eq!(out.finished[0].step, 1);
    assert_eq!(out.finished[1].step, 2);
    assert!(out.coupled, "耦合缺省为开");
    assert_eq!(e.status(), &EngineStatus::Done { failed_checks: 0 });
    assert_eq!(e.exit_code(), Some(0));
}

/// `fly` 按速度逐帧推进,到达目标的那一帧结束,不提前也不越过。
///
/// 会变红的实现:瞬移到终点;速度没乘帧时长;越过目标;到达前就报结束。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_fly_moves_at_speed_and_arrives() {
    let mut e = engine(&scenario(
        "[[step]]\nfly = { to = [16.0, -50.0, 0.0], speed = 16.0 }\n",
    ));
    let w = World::with_entities(&[]);
    for (k, x) in [4.0_f32, 8.0, 12.0].into_iter().enumerate() {
        let out = frame(&mut e, 0.25, 100, &w, &[]);
        assert_vec3(
            out.camera.pos,
            Vec3::new(x, -50.0, 0.0),
            &format!("第 {} 帧位置", k + 1),
        );
        assert!(out.finished.is_empty(), "第 {} 帧尚未到达", k + 1);
    }
    let out = frame(&mut e, 0.25, 100, &w, &[]);
    assert_vec3(out.camera.pos, Vec3::new(16.0, -50.0, 0.0), "第 4 帧到达");
    assert_eq!(out.finished.len(), 1, "到达那一帧结束");
    assert_eq!(out.finished[0].action, "fly");
    let out = frame(&mut e, 0.25, 100, &w, &[]);
    assert_vec3(out.camera.pos, Vec3::new(16.0, -50.0, 0.0), "到达后不越过");
}

/// `wait_ticks` 以服务器拍号计:起点 100、n = 30,拍号到 130 才结束;记录起止拍号。
///
/// 会变红的实现:按帧数或墙钟计时;起点取错;用 `>` 而不是 `>=`。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_wait_ticks_uses_server_ticks() {
    let mut e = engine(&scenario("[[step]]\nwait_ticks = { n = 30 }\n"));
    let w = World::with_entities(&[]);
    for tick in [100, 100, 110, 120, 129] {
        let out = frame(&mut e, 1.0, tick, &w, &[]);
        assert!(out.finished.is_empty(), "拍 {tick}:尚未结束");
    }
    let out = frame(&mut e, 0.016, 130, &w, &[]);
    assert_eq!(out.finished.len(), 1, "拍 130 结束");
    assert_eq!(
        (out.finished[0].start_tick, out.finished[0].end_tick),
        (100, 130)
    );
    assert_eq!(out.finished[0].outcome, StepOutcome::Ok);
}

/// `wait_until`:条件成立即结束;超时则场景失败,退出码 2。
///
/// 会变红的实现:条件成立不结束;超时不判失败;超时只记检查失败而不终止场景。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_wait_until_succeeds_or_times_out() {
    let text =
        scenario("[[step]]\nwait_until = { live_entities = { eq = 0 }, timeout_ticks = 100 }\n");
    let busy = World::with_entities(&[(eid(1), Vec3::new(0.5, -59.5, 0.5))]);
    let empty = World::with_entities(&[]);

    let mut ok = engine(&text);
    assert!(frame(&mut ok, 0.016, 0, &busy, &[]).finished.is_empty());
    assert!(frame(&mut ok, 0.016, 50, &busy, &[]).finished.is_empty());
    let out = frame(&mut ok, 0.016, 60, &empty, &[]);
    assert_eq!(out.finished.len(), 1, "实体清空即结束");
    assert_eq!(out.finished[0].outcome, StepOutcome::Ok);
    assert_eq!(ok.exit_code(), Some(0));

    let mut late = engine(&text);
    assert!(frame(&mut late, 0.016, 0, &busy, &[]).finished.is_empty());
    assert!(frame(&mut late, 0.016, 99, &busy, &[]).finished.is_empty());
    let out = frame(&mut late, 0.016, 100, &busy, &[]);
    assert!(
        matches!(
            out.finished.last().map(|r| &r.outcome),
            Some(StepOutcome::Error(_))
        ),
        "超时记为步骤错误"
    );
    assert!(
        matches!(late.status(), EngineStatus::Failed { step: 1, .. }),
        "超时 → 场景失败:{:?}",
        late.status()
    );
    assert_eq!(late.exit_code(), Some(2));
}

/// `spawn`:第一帧按网格发出全部生成命令(第 i 个偏移 = (i mod 5, 0, ⌊i/5⌋) × spacing,
/// 基点为相机前方 `ahead` 米再抬高 `lift` 米),带初速度;收齐回执才结束,并把回执 ID 记在
/// 标签下;收不齐回执 120 拍即场景失败。
///
/// 会变红的实现:基点没按相机朝向算;网格公式不同;忽略初速度;不等回执就结束;标签没记
/// 回执 ID;等回执没有超时。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_spawn_emits_grid_and_waits_for_receipts() {
    let text = scenario(
        r#"
[[step]]
spawn = { ahead = 8.0, lift = 6.0, count = 3, spacing = 2.0, velocity = [0.0, 5.0, 0.0], label = "b" }
[[step]]
check = { labeled_alive = { label = "b", eq = 3 } }
"#,
    );
    let w0 = World::with_entities(&[]);
    let mut e = engine(&text);
    let out = frame(&mut e, 0.016, 10, &w0, &[]);
    let spawns: Vec<_> = out
        .commands
        .iter()
        .map(|c| match c {
            EngineCommand::Spawn { state } => (state.translation, state.linvel),
            other => panic!("只应有生成命令:{other:?}"),
        })
        .collect();
    assert_eq!(spawns.len(), 3, "第一帧发出全部 3 个生成命令");
    // 相机在 (0, −50, 0) 朝 +x:基点 (8, −44, 0)。
    for (i, x) in [8.0_f32, 10.0, 12.0].into_iter().enumerate() {
        assert_vec3(
            spawns[i].0,
            Vec3::new(x, -44.0, 0.0),
            &format!("第 {i} 个生成位置"),
        );
        assert_vec3(
            spawns[i].1,
            Vec3::new(0.0, 5.0, 0.0),
            &format!("第 {i} 个初速度"),
        );
    }
    assert!(out.finished.is_empty(), "回执未到不结束");
    let out = frame(&mut e, 0.016, 20, &w0, &[eid(7), eid(8)]);
    assert!(out.commands.is_empty(), "不重复发送");
    assert!(out.finished.is_empty(), "回执未收齐");
    let all = World::with_entities(&[
        (eid(7), Vec3::new(8.0, -44.0, 0.0)),
        (eid(8), Vec3::new(10.0, -44.0, 0.0)),
        (eid(9), Vec3::new(12.0, -44.0, 0.0)),
    ]);
    let out = frame(&mut e, 0.016, 30, &all, &[eid(9)]);
    let outcomes: Vec<_> = out.finished.iter().map(|r| r.outcome.clone()).collect();
    assert_eq!(
        outcomes,
        [StepOutcome::Ok, StepOutcome::Ok],
        "收齐回执后结束,随后按标签检查存活 3 个"
    );

    let mut lost = engine(&text);
    frame(&mut lost, 0.016, 10, &w0, &[]);
    assert!(frame(&mut lost, 0.016, 129, &w0, &[]).finished.is_empty());
    frame(&mut lost, 0.016, 130, &w0, &[]);
    assert_eq!(lost.exit_code(), Some(2), "120 拍收不齐回执 → 场景失败");
}

/// `detonate`:`nearest` 取相机 25 m 内最近的活实体为爆心;25 m 内没有活实体 → 场景失败;
/// 绝对坐标原样使用。
///
/// 会变红的实现:吸附不限距离;取了最远的;找不到实体时静默跳过或改用别的点。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_detonate_nearest_and_absolute() {
    let w = World::with_entities(&[
        (eid(1), Vec3::new(30.0, -59.5, 0.0)),
        (eid(2), Vec3::new(5.0, -59.5, 0.0)),
        (eid(3), Vec3::new(9.0, -59.5, 0.0)),
    ]);
    let mut e = engine(&scenario(
        r#"
[[step]]
detonate = { nearest = true, power = 12.0, radius = 8.0 }
[[step]]
detonate = { at = [1.0, 2.0, 3.0], power = 6.0, radius = 4.0 }
"#,
    ));
    let out = frame(&mut e, 0.016, 10, &w, &[]);
    assert_eq!(
        out.commands,
        [
            EngineCommand::Detonate {
                center: Vec3::new(5.0, -59.5, 0.0),
                power: 12.0,
                radius: 8.0
            },
            EngineCommand::Detonate {
                center: Vec3::new(1.0, 2.0, 3.0),
                power: 6.0,
                radius: 4.0
            },
        ]
    );

    let far = World::with_entities(&[(eid(1), Vec3::new(30.0, -59.5, 0.0))]);
    let mut none = engine(&scenario(
        "[[step]]\ndetonate = { nearest = true, power = 12.0, radius = 8.0 }\n",
    ));
    let out = frame(&mut none, 0.016, 10, &far, &[]);
    assert!(out.commands.is_empty(), "25 m 内没有实体:不发命令");
    assert_eq!(none.exit_code(), Some(2), "25 m 内没有实体 → 场景失败");
}

/// `set_block` 发出放置命令;`couple` 切换耦合标志。
///
/// 会变红的实现:方块 ID 错;耦合开关不生效。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_set_block_and_couple() {
    let mut e = engine(&scenario(
        r#"
[[step]]
set_block = { pos = [1, -61, 2], block = "air" }
[[step]]
couple = { on = false }
"#,
    ));
    let out = frame(&mut e, 0.016, 10, &World::with_entities(&[]), &[]);
    assert_eq!(
        out.commands,
        [EngineCommand::SetBlock {
            pos: BlockPos::new(1, -61, 2),
            block: AIR
        }]
    );
    assert!(!out.coupled, "couple off 当帧生效");
}

/// 截图:每帧至多一个请求;发出截图请求的那一帧不再推进后续步骤;文件名两位补零。
///
/// 会变红的实现:同一帧发出多个截图;截图后继续执行后续步骤(截到的不是期望的画面);
/// 文件名格式不同。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_one_screenshot_per_frame() {
    let mut e = engine(&scenario(
        r#"
[[step]]
screenshot = { name = "a", expect = "第一张" }
[[step]]
camera = { pos = [1.0, -50.0, 1.0], yaw = 0.0, pitch = 0.0 }
[[step]]
screenshot = { name = "b", expect = "第二张", hud = true }
"#,
    ));
    let w = World::with_entities(&[]);
    let out = frame(&mut e, 0.016, 10, &w, &[]);
    let shot = out.screenshot.clone().expect("第一帧截第一张");
    assert_eq!((shot.step, shot.name.as_str(), shot.hud), (1, "a", false));
    assert_eq!(shot.file_name(), "01-a.png");
    assert_eq!(out.finished.len(), 1, "截图那一帧只结束截图步骤");
    assert_vec3(out.camera.pos, start_pose().pos, "截图帧不执行后续 camera");
    assert_eq!(
        out.finished[0].screenshot.as_ref(),
        Some(&shot),
        "记录带截图请求"
    );

    let out = frame(&mut e, 0.016, 10, &w, &[]);
    let shot = out.screenshot.expect("第二帧截第二张");
    assert_eq!(shot.file_name(), "03-b.png");
    assert!(shot.hud);
    assert_vec3(
        out.camera.pos,
        Vec3::new(1.0, -50.0, 1.0),
        "camera 在截图前生效",
    );
    let actions: Vec<&str> = out.finished.iter().map(|r| r.action.as_str()).collect();
    assert_eq!(actions, ["camera", "screenshot"]);
}

/// `check` 失败记入结果、场景继续执行;结束后退出码 1。全部成立则为 0。
///
/// 会变红的实现:检查失败就终止场景;失败原因为空;退出码不区分检查失败与场景失败。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_check_failure_continues_with_exit_code_1() {
    let w = World::with_entities(&[(eid(1), Vec3::new(0.5, -59.5, 0.5))]);
    let mut e = engine(&scenario(
        r#"
[[step]]
check = { live_entities = { eq = 0 } }
[[step]]
camera = { pos = [1.0, -50.0, 1.0], yaw = 0.0, pitch = 0.0 }
"#,
    ));
    let out = frame(&mut e, 0.016, 10, &w, &[]);
    assert_eq!(out.finished.len(), 2, "检查失败后继续执行下一步");
    match &out.finished[0].outcome {
        StepOutcome::CheckFailed(msgs) => {
            assert!(
                !msgs.is_empty() && msgs.iter().all(|m| !m.is_empty()),
                "失败原因非空"
            )
        }
        other => panic!("应为检查失败:{other:?}"),
    }
    assert_eq!(e.status(), &EngineStatus::Done { failed_checks: 1 });
    assert_eq!(e.exit_code(), Some(1));
}

/// 整场景墙钟超时:帧时长累计超过 `timeout_secs` 即场景失败。
///
/// 会变红的实现:不计墙钟;按服务器拍号算超时。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s2_scenario_timeout() {
    let mut e = engine(&format!(
        "{HEADER}timeout_secs = 1\n[[step]]\nwait_ticks = {{ n = 1000 }}\n"
    ));
    let w = World::with_entities(&[]);
    for _ in 0..3 {
        frame(&mut e, 0.3, 10, &w, &[]);
        assert_eq!(e.exit_code(), None, "0.9 s 内仍在运行");
    }
    frame(&mut e, 0.3, 10, &w, &[]);
    assert_eq!(e.exit_code(), Some(2), "累计 1.2 s > 1 s → 场景失败");
}

// ===========================================================================
// 单元 01 · AC-S3 检查项
// ===========================================================================

/// 跑一个只含一步 `check` 的场景,返回该步结果。
fn check_outcome(check: &str, world: &World) -> StepOutcome {
    let mut e = engine(&scenario(&format!("[[step]]\ncheck = {{ {check} }}\n")));
    let out = frame(&mut e, 0.016, 10, world, &[]);
    out.finished
        .first()
        .map(|r| r.outcome.clone())
        .expect("check 当帧结束")
}

/// 计数类与方块检查:每项都有成立与不成立两种构造。
///
/// 会变红的实现:比较方向写反;方块检查读错格子;chunk 计数取错来源。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s3_count_and_block_checks() {
    let two = World::with_entities(&[
        (eid(1), Vec3::new(0.5, -59.5, 0.5)),
        (eid(2), Vec3::new(2.5, -59.5, 0.5)),
    ]);
    let mut dug = two.clone();
    dug.overrides.insert((3, -61, 4), AIR);
    for (check, world, pass) in [
        ("live_entities = { eq = 2 }", &two, true),
        ("live_entities = { eq = 1 }", &two, false),
        ("live_entities = { le = 2 }", &two, true),
        ("live_entities = { le = 1 }", &two, false),
        ("live_entities = { ge = 2 }", &two, true),
        ("live_entities = { ge = 3 }", &two, false),
        ("chunks_loaded = { ge = 9 }", &two, true),
        ("chunks_loaded = { ge = 10 }", &two, false),
        ("block = { pos = [3, -61, 4], is = \"stone\" }", &two, true),
        ("block = { pos = [3, -61, 4], is = \"stone\" }", &dug, false),
        ("block = { pos = [3, -61, 4], is = \"air\" }", &dug, true),
        ("block = { pos = [300, -61, 4], is = \"air\" }", &two, false),
    ] {
        let got = check_outcome(check, world);
        if pass {
            assert_eq!(got, StepOutcome::Ok, "{check} 应成立");
        } else {
            assert!(
                matches!(got, StepOutcome::CheckFailed(_)),
                "{check} 应不成立,实为 {got:?}"
            );
        }
    }
}

/// `no_floating`:悬空且静止(最近 30 拍位移 ≤ 0.05 m、底面高出地表顶面 1.5 m 以上)的
/// 活实体判失败;正在下落的、贴地的、观测不足 30 拍的都不判失败。
///
/// 会变红的实现:只看高度不看静止(把下落中的碎片判为悬空);只看静止不看高度(把贴地的
/// 判为悬空);历史不足也下结论;高度没减去半尺寸或没加上方块顶面。
#[test]
#[ignore = "REQ-008 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s3_no_floating() {
    let text = scenario(
        r#"
[[step]]
wait_ticks = { n = 40 }
[[step]]
check = { no_floating = { height = 1.5, still_ticks = 30 } }
"#,
    );
    // 每 10 拍一个状态包;y(k) 给出第 k 包时实体的中心高度。
    let run = |y_at: &dyn Fn(u64) -> Option<f32>| -> StepOutcome {
        let mut e = engine(&text);
        let mut last = None;
        for tick in [0_u64, 10, 20, 30, 40] {
            let world = match y_at(tick) {
                Some(y) => World::with_entities(&[(eid(1), Vec3::new(0.5, y, 0.5))]),
                None => World::with_entities(&[]),
            };
            let out = frame(&mut e, 0.016, tick, &world, &[]);
            if let Some(r) = out.finished.iter().find(|r| r.action == "check") {
                last = Some(r.outcome.clone());
            }
        }
        last.expect("check 在拍 40 结束")
    };
    // 悬空静止:中心 −55.5 → 底面 −56,高出顶面 −60 共 4 m。
    assert!(
        matches!(run(&|_| Some(-55.5)), StepOutcome::CheckFailed(_)),
        "悬空静止应判失败"
    );
    // 正在下落:每包下降 1 m。
    assert_eq!(
        run(&|t| Some(-50.0 - t as f32 / 10.0)),
        StepOutcome::Ok,
        "下落中不算悬空"
    );
    // 贴地静止。
    assert_eq!(run(&|_| Some(-59.5)), StepOutcome::Ok, "贴地不算悬空");
    // 刚好 1.5 m 不到:中心 −58.1 → 底面 −58.6,高出 1.4 m。
    assert_eq!(
        run(&|_| Some(-58.1)),
        StepOutcome::Ok,
        "高出不足 1.5 m 不算悬空"
    );
    // 拍 30 才出现:到拍 40 只观测了 10 拍,不足 30 拍,不下结论。
    assert_eq!(
        run(&|t| (t >= 30).then_some(-55.5)),
        StepOutcome::Ok,
        "观测不足 still_ticks 不判悬空"
    );
}
