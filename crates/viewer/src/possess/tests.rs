//! 附身状态机与意图映射单测(判据 A06-1、A06-2)。

use super::*;
use types::EntityId;

fn keys(w: bool, a: bool, s: bool, d: bool, shift: bool) -> Keys {
    Keys {
        w,
        a,
        s,
        d,
        shift,
        jump_down: false,
    }
}

// ---------------------------------------------------------------------------
// A06-2 意图映射:intent_from_keys
// ---------------------------------------------------------------------------

/// 相机 yaw 取 0°/90°/180°/−45°,W 产生的前向与世界系相机水平前向一致
/// (误差 ≤ 1e-5;速度 4.3)。
#[test]
fn intent_w_matches_camera_forward_all_yaws() {
    // viewer yaw 约定:flat_forward = (cos yaw, sin yaw)(x, z)。
    for yaw_deg in [0.0_f32, 90.0, 180.0, -45.0] {
        let yaw = yaw_deg.to_radians();
        let expect = (yaw.cos() * WALK_SPEED, yaw.sin() * WALK_SPEED);
        let got = intent_from_keys(&keys(true, false, false, false, false), yaw_deg);
        assert!(
            (got.0 - expect.0).abs() <= 1e-5 && (got.1 - expect.1).abs() <= 1e-5,
            "yaw {yaw_deg}°:W 前向实得 {got:?},期望 {expect:?}"
        );
    }
}

/// A = 左、S = 后、D = 右(与飞行相机同一套向量代数:right = 前 × Y =
/// (−sin yaw, cos yaw)。yaw=0:前向 +x,右 +z,左 −z,后 −x)。
#[test]
fn intent_a_s_d_directions() {
    let left = intent_from_keys(&keys(false, true, false, false, false), 0.0);
    assert!(
        left.0.abs() <= 1e-5 && (left.1 + WALK_SPEED).abs() <= 1e-5,
        "A = −右(−z),实得 {left:?}"
    );
    let back = intent_from_keys(&keys(false, false, true, false, false), 0.0);
    assert!(
        (back.0 + WALK_SPEED).abs() <= 1e-5 && back.1.abs() <= 1e-5,
        "S = −前向(−x),实得 {back:?}"
    );
    let right = intent_from_keys(&keys(false, false, false, true, false), 0.0);
    assert!(
        right.0.abs() <= 1e-5 && (right.1 - WALK_SPEED).abs() <= 1e-5,
        "D = 右(+z),实得 {right:?}"
    );
    // yaw = 90°:前向 = (0, 1)(+z);右 = (−1, 0)(−x)。W 沿 +z。
    let w90 = intent_from_keys(&keys(true, false, false, false, false), 90.0);
    assert!(
        w90.0.abs() <= 1e-5 && (w90.1 - WALK_SPEED).abs() <= 1e-5,
        "yaw 90° 的 W = +z,实得 {w90:?}"
    );
    let d90 = intent_from_keys(&keys(false, false, false, true, false), 90.0);
    assert!(
        (d90.0 + WALK_SPEED).abs() <= 1e-5 && d90.1.abs() <= 1e-5,
        "yaw 90° 的 D = −x,实得 {d90:?}"
    );
}

/// W+D 组合 = 前向与右向的单位化和(45° 方向,速度仍 4.3)。
#[test]
fn intent_diagonal_is_normalized() {
    let got = intent_from_keys(&keys(true, false, false, true, false), 0.0);
    let expect = (
        WALK_SPEED * std::f32::consts::FRAC_1_SQRT_2,
        WALK_SPEED * std::f32::consts::FRAC_1_SQRT_2,
    );
    assert!(
        (got.0 - expect.0).abs() <= 1e-5 && (got.1 - expect.1).abs() <= 1e-5,
        "W+D = 单位化和 × 速度,实得 {got:?},期望 {expect:?}"
    );
    let len = (got.0 * got.0 + got.1 * got.1).sqrt();
    assert!(
        (len - WALK_SPEED).abs() <= 1e-5,
        "组合速度模长仍 4.3,实得 {len}"
    );
}

/// Shift → 5.6 m/s(方向不变);无方向键 → 零速度(松键发零)。
#[test]
fn intent_shift_and_zero() {
    let fast = intent_from_keys(&keys(true, false, false, false, true), 0.0);
    assert!(
        (fast.0 - SPRINT_SPEED).abs() <= 1e-5,
        "Shift 速度 5.6,实得 {fast:?}"
    );
    let fast90 = intent_from_keys(&keys(true, false, false, false, true), 90.0);
    assert!(
        fast90.0.abs() <= 1e-5 && (fast90.1 - SPRINT_SPEED).abs() <= 1e-5,
        "yaw 90° 的 Shift W = +z × 5.6,实得 {fast90:?}"
    );
    let none = intent_from_keys(&keys(false, false, false, false, false), 30.0);
    assert_eq!(none, (0.0, 0.0), "无方向键 = 零速度(松键发零)");
    // 只按 Shift 无方向:也是零(速度乘方向,方向为零)。
    let shift_only = intent_from_keys(&keys(false, false, false, false, true), 0.0);
    assert_eq!(shift_only, (0.0, 0.0));
}

// ---------------------------------------------------------------------------
// A06-1 回执与附身状态机
// ---------------------------------------------------------------------------

fn agent_id(n: u32) -> EntityId {
    EntityId::from_parts(0, n)
}

/// 假服务器时序:P(无身体)→ SpawnAgent 命令 → Spawned 回执 + 主体流可见
/// → 自动 Possess → Possessed(Some) 登记。中间插一条**方块 Spawn** 的
/// Spawned 回执:序号错开,不会被附身(方块请求占序号,且自动附身只对
/// 主体流里可见的 ID 发 Possess)。
#[test]
fn possess_flow_spawn_then_auto_possess() {
    let mut st = PossessState::new();
    // P 按下(无身体;拾取目标 = 地面格顶面上方)。
    let cmds = st.frame(
        true,
        &keys(false, false, false, false, false),
        0.0,
        Some((Vec3::new(8.0, -57.0, 24.0), 45.0)),
        None,
    );
    assert_eq!(
        cmds,
        vec![PossessCommand::SpawnAgent {
            position: Vec3::new(8.0, -57.0, 24.0),
            yaw: 45.0,
        }],
        "P → SpawnAgent 命令"
    );
    // 一条方块 Spawn 的回执先到(viewer 之前放过一个方块):序号被方块占。
    st.on_spawn_receipt(); // 方块的
    // 主体的 Spawned 回执到达。
    st.on_spawn_receipt(); // 主体的
    // 主体流可见(服务器 AgentStates 全量)。
    st.observe_agents(&[agent_id(1)], &[]);
    // 渲染层调用 try_auto_possess(候选 = 刚 Spawned 且主体流可见的 ID)。
    let got = st.try_auto_possess(Some(agent_id(1)));
    assert_eq!(
        got,
        Some(agent_id(1)),
        "Spawned + 主体流可见 → 自动附身候选"
    );
    // Possessed(Some) 到达:登记附身。
    st.on_possessed(Some(agent_id(1)));
    assert_eq!(st.possessed(), Some(agent_id(1)));
    // 附身后 pending 已消费:再次调用不重复发。
    assert_eq!(
        st.try_auto_possess(Some(agent_id(1))),
        None,
        "自动附身只发一次"
    );
}

/// 中间插方块 Spawned 不会把方块 ID 附身:自动附身候选必须是主体流可见的
/// ID;渲染层传方块 ID(不在 agent_seen)→ try_auto_possess 不发。
#[test]
fn block_spawn_id_never_possessed() {
    let mut st = PossessState::new();
    let _ = st.frame(true, &Keys::default(), 0.0, Some((Vec3::ZERO, 0.0)), None);
    st.on_spawn_receipt(); // 主体的 Spawned
    // 主体流(AgentStates)只含主体——方块实体走 EntityStates,永不进
    // agent_seen。候选传一个从没见过的 ID(比如把方块 ID 当主体):不附身。
    assert_eq!(
        st.try_auto_possess(Some(EntityId::from_parts(0, 999))),
        None
    );
    // 传主体流里的正确 ID → 附身。
    st.observe_agents(&[agent_id(1)], &[]);
    assert_eq!(st.try_auto_possess(Some(agent_id(1))), Some(agent_id(1)));
}

/// Possessed(None) → 回飞行相机(possessed 清空;意图状态清掉)。
#[test]
fn possessed_none_returns_to_fly_camera() {
    let mut st = PossessState::new();
    st.on_possessed(Some(agent_id(2)));
    assert_eq!(st.possessed(), Some(agent_id(2)));
    st.on_possessed(None);
    assert_eq!(st.possessed(), None, "Possessed(None) → 飞行相机");
    // 解除后无方向键 + P:重新走生成路径(不再发意图)。
    let cmds = st.frame(
        false,
        &keys(true, false, false, false, false),
        0.0,
        None,
        Some(agent_id(2)),
    );
    assert!(cmds.is_empty(), "未附身且无 P:不发任何命令(意图停发)");
}

/// 已附身时按 P → Unpossess(附带一条零意图让主体停下,放行 §2 表:
/// Unpossess 语义 = 解除 + 主体停;两条命令同帧发出,顺序 Unpossess 在前)。
/// 解除附身后再附身:意图节流状态必须已清——重附身第一帧同方向意图要
/// 立即发出(否则残留的 last_intent 判"无变化"吞掉首帧,主体失控一拍)。
#[test]
fn repossess_after_unpossess_resends_intent_immediately() {
    let mut st = PossessState::new();
    st.on_possessed(Some(agent_id(5)));
    // 附身中向东走(登记 last_intent = 东)。
    let cmds = st.frame(
        false,
        &keys(true, false, false, false, false),
        0.0,
        None,
        None,
    );
    assert!(
        cmds.iter()
            .any(|c| matches!(c, PossessCommand::Intent { .. })),
        "布景:行走中发意图"
    );
    // 跨过 50 ms 节流窗(只验"变化才发"语义,不与节流耦合)。
    std::thread::sleep(super::INTENT_INTERVAL);
    // 服务器强制解除(被撞出世界等)→ Possessed(None)。
    st.on_possessed(None);
    // 重新附身同一主体。
    st.on_possessed(Some(agent_id(5)));
    // 第一帧同方向意图:必须立即发出(last_intent 已随解除清空)。
    let cmds = st.frame(
        false,
        &keys(true, false, false, false, false),
        0.0,
        None,
        None,
    );
    assert!(
        cmds.iter()
            .any(|c| matches!(c, PossessCommand::Intent { .. })),
        "重附身首帧同方向意图必须发出(残留 last_intent 会吞掉它)"
    );
}

#[test]
fn p_toggles_unpossess_and_possess_existing_body() {
    let mut st = PossessState::new();
    st.on_possessed(Some(agent_id(3)));
    let cmds = st.frame(true, &Keys::default(), 0.0, None, None);
    assert_eq!(
        cmds.first(),
        Some(&PossessCommand::Unpossess),
        "已附身按 P → Unpossess(首条)"
    );
    assert!(
        !cmds.is_empty(),
        "解除帧可附带停机意图(渲染层只透传 Unpossess 本体,零意图由服务器发)"
    );
    st.on_possessed(None);
    // 未附身 + 有身体(agent_seen 可见):P 附身它(不是再生成)。
    st.observe_agents(&[agent_id(3)], &[]);
    let cmds = st.frame(true, &Keys::default(), 0.0, None, Some(agent_id(3)));
    assert_eq!(
        cmds,
        vec![PossessCommand::Possess(agent_id(3))],
        "有身体未附身按 P → Possess"
    );
}

// ---------------------------------------------------------------------------
// 意图节流与跳跃(A06-2 后半)
// ---------------------------------------------------------------------------

/// 意图有变化才发;50 ms 节流(墙钟);松开所有键发一次零速度;跳跃按下沿
/// 只发一次(按住不连发)。
#[test]
fn intent_throttle_change_and_zero_on_release() {
    let mut st = PossessState::new();
    st.on_possessed(Some(agent_id(4)));
    // 第一帧:W 按下(未发过 → 立即发)。
    let cmds = st.frame(
        false,
        &keys(true, false, false, false, false),
        0.0,
        None,
        None,
    );
    assert_eq!(cmds.len(), 1, "首条意图立即发");
    assert!(matches!(cmds[0], PossessCommand::Intent { .. }));
    // 第二帧(50 ms 内):状态未变不发;状态变化(换键 W→D)也被节流窗拦下。
    let cmds = st.frame(
        false,
        &keys(true, false, false, false, false),
        0.0,
        None,
        None,
    );
    assert!(cmds.is_empty(), "无变化不重发(节流窗内)");
    let cmds = st.frame(
        false,
        &keys(false, true, false, false, false),
        0.0,
        None,
        None,
    );
    assert!(cmds.is_empty(), "窗内有变化也节流(≤50 ms 一条)");
    // 状态变化(松开 W → 零速度):即使节流窗内也……按放行口径"松开所有
    // 方向键要发一次零速度"+"每 50 ms 最多一条"——零速度是变化,须等节流。
    // 本测试等 50 ms 后发。
    std::thread::sleep(INTENT_INTERVAL);
    let cmds = st.frame(
        false,
        &keys(false, false, false, false, false),
        0.0,
        None,
        None,
    );
    assert_eq!(cmds.len(), 1, "松键 → 零速度意图");
    match &cmds[0] {
        PossessCommand::Intent { move_xz, jump, .. } => {
            assert_eq!(*move_xz, (0.0, 0.0), "零速度");
            assert!(!jump, "松键不发跳跃");
        }
        other => panic!("应为 Intent,实为 {other:?}"),
    }
}

/// 跳跃按下沿:即使方向/朝向没变、节流窗未到,也立即发一条(带 jump);
/// 按住不放(第二帧 jump_down 仍 = true)不连发。
#[test]
fn jump_edge_sends_once() {
    let mut st = PossessState::new();
    st.on_possessed(Some(agent_id(5)));
    // 静止 + 跳按下沿。
    let mut k = keys(false, false, false, false, false);
    k.jump_down = true;
    let cmds = st.frame(false, &k, 0.0, None, None);
    assert_eq!(cmds.len(), 1, "跳跃按下沿立即发");
    match &cmds[0] {
        PossessCommand::Intent { move_xz, jump, .. } => {
            assert_eq!(*move_xz, (0.0, 0.0));
            assert!(*jump, "jump = true");
        }
        other => panic!("应为 Intent,实为 {other:?}"),
    }
    // 按住 Space 但松开沿已过(渲染层的 is_key_pressed 只在按下沿为真,
    // 这里模拟下一帧按住未松(jump_down = true)——沿检测在状态机:零变化
    // + 非沿 → 不发。
    let mut held = keys(false, false, false, false, false);
    held.jump_down = true;
    let cmds = st.frame(false, &held, 0.0, None, None);
    assert!(cmds.is_empty(), "按住不连发(状态机 diff 出按下沿)");
}

/// 跳跃时方向变化:跳与意图合并成一条(jump = true 的新意图)。
#[test]
fn jump_with_direction_change_single_intent() {
    let mut st = PossessState::new();
    st.on_possessed(Some(agent_id(6)));
    let mut k = keys(true, false, false, false, false);
    k.jump_down = true;
    let cmds = st.frame(false, &k, 0.0, None, None);
    assert_eq!(cmds.len(), 1, "跳与意图合一帧一条");
    match &cmds[0] {
        PossessCommand::Intent { move_xz, jump, .. } => {
            assert!(*jump);
            assert!(
                (move_xz.0 - WALK_SPEED).abs() <= 1e-5,
                "W 前向 x 分量 = +4.3, 实得 {}",
                move_xz.0
            );
            assert!(
                move_xz.1.abs() <= 1e-5,
                "W 前向 z 分量 = 0, 实得 {}",
                move_xz.1
            );
        }
        other => panic!("应为 Intent,实为 {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// 第三人称相机与滚轮
// ---------------------------------------------------------------------------

/// 第三人称相机:看向根位置上方 0.6 m,位于其后方 dist 处(yaw/pitch 同
/// 飞行相机约定)。pitch 用 +90 表示向上看时相机在后下?——本约定:pitch
/// 是**相机的俯仰**(抬头为正,同飞行相机);相机在前向的反方向,故
/// pitch = +90(抬头)→ 相机在正下方。俯视身体(pitch = −90)相机在头顶。
#[test]
fn third_person_camera_positions() {
    let root = Vec3::new(10.0, -58.0, 20.0);
    // yaw = 0,pitch = 0,dist = 4:前向 = (1,0,0),相机在 (6, -57.4, 20)。
    let (pos, look) = third_person_camera(root, 0.0, 0.0, 4.0);
    assert!((pos.x - 6.0).abs() <= 1e-5 && (pos.y - (root.y + LOOK_HEIGHT)).abs() <= 1e-5);
    assert_eq!(look, Vec3::new(10.0, root.y + LOOK_HEIGHT, 20.0));
    // pitch = −90(俯视):fwd = (0,−1,0),相机在注视点正上方 4 m。
    let (pos, _) = third_person_camera(root, 0.0, -90.0, 4.0);
    assert!(
        (pos.y - (root.y + LOOK_HEIGHT + 4.0)).abs() <= 1e-4,
        "pitch −90° 相机在头顶,实得 y {}",
        pos.y
    );
    // 距离缩放。
    let (pos, _) = third_person_camera(root, 0.0, 0.0, 10.0);
    assert!((pos.x - 0.0).abs() <= 1e-4, "dist 10 → 相机在 −10 前向处");
}

/// 滚轮距离 2~10,缺省 4;滚轮方向:上滚(wheel > 0)= 拉近。
#[test]
fn wheel_distance_clamped() {
    let mut st = PossessState::new();
    assert!((st.camera_dist() - CAMERA_DIST_DEFAULT).abs() <= 1e-6);
    st.adjust_camera_dist(-1.0); // 下滚 = 拉远
    assert!((st.camera_dist() - 5.0).abs() <= 1e-6);
    for _ in 0..20 {
        st.adjust_camera_dist(-1.0);
    }
    assert!(
        (st.camera_dist() - CAMERA_DIST_MAX).abs() <= 1e-6,
        "夹上限 10"
    );
    for _ in 0..20 {
        st.adjust_camera_dist(1.0);
    }
    assert!(
        (st.camera_dist() - CAMERA_DIST_MIN).abs() <= 1e-6,
        "夹下限 2"
    );
}

// ---------------------------------------------------------------------------
// 主体流账本(observe_agents):removed 立即清(agent_seen 收缩)。
// ---------------------------------------------------------------------------

#[test]
fn removed_agents_retract_from_seen() {
    let mut st = PossessState::new();
    st.observe_agents(&[agent_id(1), agent_id(2)], &[]);
    st.observe_agents(&[], &[agent_id(1)]);
    // agent_id(1) 已被移除:自动附身不再接受它。
    assert_eq!(st.try_auto_possess(Some(agent_id(1))), None);
    st.observe_agents(&[], &[agent_id(2)]);
    assert_eq!(st.try_auto_possess(Some(agent_id(2))), None);
}

/// 未知哈希/占位路径在这里测不了(渲染层职责);本文件的占位计数走
/// render 侧 body table 测试(render.rs 单测)。
#[test]
fn spawn_request_receipts_fifo_order() {
    let mut st = PossessState::new();
    let _ = st.frame(true, &Keys::default(), 0.0, Some((Vec3::ZERO, 0.0)), None);
    // 三条回执(方块、主体、多余):队列只有一条(主体的),方块先到。
    st.on_spawn_receipt(); // 方块
    assert_eq!(st.try_auto_possess(None), None, "主体回执未到,不自动附身");
    st.on_spawn_receipt(); // 主体
    // 多余回执:队列空,忽略(不 panic)。
    st.on_spawn_receipt();
}
