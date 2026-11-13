//! 附身状态机与意图映射单测(判据 A06-1 → T6 状态机逐行覆盖、A06-2 意图映射)。
//!
//! 时间全部显式推(`t(ms)` 辅助),状态机不读墙钟。

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

/// 显式时间轴:第 n 毫秒。
fn t(ms: u64) -> std::time::Instant {
    // Instant 只能取 now ± duration;用减法构造过去时刻,单调即可。
    // (墙钟仅在测试时间轴构造点用一次;clippy.toml 对 viewer 测试的
    // disallowed_methods 在此放行——同 render.rs 渲染节流的局部授权口径。)
    #[allow(clippy::disallowed_methods)]
    {
        std::time::Instant::now() - std::time::Duration::from_millis(100_000 - ms)
    }
}

fn agent_id(n: u32) -> EntityId {
    EntityId::from_parts(0, n)
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
// T6:状态机逐行覆盖(R1 §3.2 转移表)
// ---------------------------------------------------------------------------

/// (a) 视野里先有一个 ID 更大的**别人的**主体,按 P 仍然发 SpawnAgent,
/// 之后只附身自己的(不是 ID 最大的那个)。变红实现:归属改回"取视野里
/// ID 最大的主体"(放行变异 1)。
#[test]
fn t6a_p_with_bigger_foreign_agent_spawns_and_possesses_own() {
    let mut st = PossessState::new();
    // 视野里已有别人的主体 ID 42(AI 主体)。
    st.on_agents(&[agent_id(42)], &[]);
    assert_eq!(st.phase(), Phase::NoBody);
    // 按 P:发 SpawnAgent(不是 Possess(42))。
    let cmds = st.on_p_pressed(Some((Vec3::new(8.0, -57.0, 24.0), 45.0)), t(0));
    assert_eq!(
        cmds,
        vec![PossessCommand::SpawnAgent {
            position: Vec3::new(8.0, -57.0, 24.0),
            yaw: 45.0,
        }],
        "视野里有别人的主体,按 P 仍发 SpawnAgent"
    );
    assert_eq!(st.phase(), Phase::SpawnRequested);
    // 自己的身体回执到达,ID 更小(比如 7)。
    st.on_spawned(agent_id(7));
    // 主体流同时含 42 与 7:只附身 7。
    let cmds = st.on_agents(&[agent_id(42), agent_id(7)], &[]);
    assert_eq!(
        cmds,
        vec![PossessCommand::Possess(agent_id(7))],
        "自动附身只发自己的身体(回执+主体流),不是 ID 最大的"
    );
    // Possessed(Some(7)) 确认。
    st.on_possessed(Some(agent_id(7)));
    assert_eq!(st.possessed(), Some(agent_id(7)));
    assert_eq!(st.my_body(), Some(agent_id(7)));
}

/// (b) 先收到一条**方块**回执、再收到身体回执:只附身身体,方块 ID 不进
/// my_body(方块永不进主体流,自然不命中)。变红实现:回执不按队列、
/// 主体优先(T7 同款)。
#[test]
fn t6b_block_receipt_first_then_body_receipt() {
    let mut st = PossessState::new();
    // P 生成(请求在途)。
    let _ = st.on_p_pressed(Some((Vec3::ZERO, 0.0)), t(0));
    // 先到一条方块 Spawned 回执(此前 viewer 放过方块)。
    st.on_spawned(EntityId::from_parts(0, 999));
    // 再到身体的 Spawned 回执。
    st.on_spawned(agent_id(3));
    // 主体流只有 3(方块 999 永不进 AgentStates)。
    let cmds = st.on_agents(&[agent_id(3)], &[]);
    assert_eq!(
        cmds,
        vec![PossessCommand::Possess(agent_id(3))],
        "只附身身体;方块 ID 不在主体流,永不归属"
    );
    assert_eq!(st.my_body(), Some(agent_id(3)));
    assert_ne!(st.my_body(), Some(EntityId::from_parts(0, 999)));
}

/// (c) `SpawnRequested` 2 s 无回应 → 回 `NoBody`(请求被拒),清
/// auto_possess;再按 P 可以重新生成。
#[test]
fn t6c_spawn_requested_times_out_to_no_body() {
    let mut st = PossessState::new();
    let _ = st.on_p_pressed(Some((Vec3::ZERO, 0.0)), t(0));
    assert_eq!(st.phase(), Phase::SpawnRequested);
    // 1.9 s:还没超时。
    st.tick(t(1_900));
    assert_eq!(st.phase(), Phase::SpawnRequested);
    // 2.0 s:超时 → NoBody。
    st.tick(t(2_000));
    assert_eq!(
        st.phase(),
        Phase::NoBody,
        "SpawnRequested 2 s 无回应 → NoBody"
    );
    // 超时后身体回执迟到:不再自动附身(auto_possess 已清)。
    st.on_spawned(agent_id(1));
    let cmds = st.on_agents(&[agent_id(1)], &[]);
    assert!(cmds.is_empty(), "被拒后的迟到回执不自动附身");
    assert_eq!(st.phase(), Phase::HasBody);
    // 再按 P:有身体未附身 → Possess(走 HasBody 分支,不是再生成)。
    let cmds = st.on_p_pressed(None, t(2_100));
    assert_eq!(cmds, vec![PossessCommand::Possess(agent_id(1))]);
}

/// (c') `PossessRequested` 2 s 无回应 → 回 `HasBody`。
#[test]
fn t6c2_possess_requested_times_out_to_has_body() {
    let mut st = PossessState::new();
    // 布景:已有身体(回执 + 主体流)。
    st.on_spawned(agent_id(2));
    let _ = st.on_agents(&[agent_id(2)], &[]);
    assert_eq!(st.phase(), Phase::HasBody);
    // P → PossessRequested。
    let _ = st.on_p_pressed(None, t(0));
    assert_eq!(st.phase(), Phase::PossessRequested);
    // 2 s 无 Possessed 回执 → HasBody。
    st.tick(t(2_000));
    assert_eq!(
        st.phase(),
        Phase::HasBody,
        "PossessRequested 超时 → HasBody"
    );
    assert_eq!(st.my_body(), Some(agent_id(2)), "身体归属不丢");
}

/// (d) 服务器主动重生:未请求的 Spawned(id) + AgentStates 含 id +
/// Possessed(Some(id)) → my_body = id、Possessed。
#[test]
fn t6d_server_initiated_respawn_becomes_my_body() {
    let mut st = PossessState::new();
    // 未发任何请求:回执直接到(服务器主动生成)。
    st.on_spawned(agent_id(9));
    // Possessed(Some(9)) 先到(驱动重生路径:Spawned → Possessed 连发)。
    st.on_possessed(Some(agent_id(9)));
    assert_eq!(st.possessed(), Some(agent_id(9)));
    assert_eq!(st.my_body(), Some(agent_id(9)));
    // 主体流随后含 9:归属已确认,保持 Possessed(不被回退)。
    let cmds = st.on_agents(&[agent_id(9)], &[]);
    assert!(cmds.is_empty(), "已确认的归属不重复发 Possess");
    assert_eq!(st.phase(), Phase::Possessed);
    assert_eq!(st.my_body(), Some(agent_id(9)));
}

/// (e) removed 含 my_body → NoBody;Possessed(None) 收尾后按 P 走生成。
#[test]
fn t6e_removed_my_body_goes_no_body() {
    let mut st = PossessState::new();
    // 布景:已附身 3。
    st.on_spawned(agent_id(3));
    let _ = st.on_agents(&[agent_id(3)], &[]);
    let _ = st.on_p_pressed(None, t(0));
    st.on_possessed(Some(agent_id(3)));
    assert_eq!(st.phase(), Phase::Possessed);
    // 掉出世界:removed 含 3 → NoBody(等 Possessed(None) 收尾)。
    let cmds = st.on_agents(&[], &[agent_id(3)]);
    assert!(cmds.is_empty());
    assert_eq!(st.phase(), Phase::NoBody);
    assert_eq!(st.my_body(), None);
    // Possessed(None) 到达:仍 NoBody(my_body 已空)。
    st.on_possessed(None);
    assert_eq!(st.phase(), Phase::NoBody);
    // 再按 P(有拾取目标):重新生成。
    let cmds = st.on_p_pressed(Some((Vec3::ONE, 0.0)), t(10));
    assert!(matches!(
        cmds.as_slice(),
        [PossessCommand::SpawnAgent { .. }]
    ));
}

/// (f) `Possessed(None)`:
/// - my_body 还在(主动 Unpossess)→ HasBody;
/// - my_body 没了 → NoBody。意图状态都清零(重附身首帧意图立即发)。
#[test]
fn t6f_possessed_none_branches_on_body_presence() {
    // 分支 1:主动解除(身体还在)。
    let mut st = PossessState::new();
    st.on_spawned(agent_id(4));
    let _ = st.on_agents(&[agent_id(4)], &[]);
    let _ = st.on_p_pressed(None, t(0));
    st.on_possessed(Some(agent_id(4)));
    st.on_possessed(None);
    assert_eq!(st.phase(), Phase::HasBody, "身体还在 → HasBody");
    assert_eq!(st.my_body(), Some(agent_id(4)));
    // 分支 2:服务器强制解除 + 身体已移除(removed 先行 → NoBody,再收
    // Possessed(None) 收尾仍 NoBody)。
    let mut st2 = PossessState::new();
    st2.on_possessed(Some(agent_id(5)));
    let _ = st2.on_agents(&[], &[agent_id(5)]);
    assert_eq!(st2.phase(), Phase::NoBody, "身体被移除 → NoBody");
    st2.on_possessed(None);
    assert_eq!(st2.phase(), Phase::NoBody);
    // 意图状态清零:重附身首帧同方向意图立即发。
    st.on_possessed(Some(agent_id(4)));
    let cmds = st.on_keys(&keys(true, false, false, false, false), 0.0, t(10));
    assert!(
        matches!(cmds.as_slice(), [PossessCommand::Intent { .. }]),
        "重附身首帧意图立即发"
    );
}

/// (g) 已附身按 P → Unpossess;状态保持 Possessed 等 Possessed(None)。
/// 中途插 Spawned 回执不影响。
#[test]
fn t6g_p_while_possessed_sends_unpossess() {
    let mut st = PossessState::new();
    st.on_possessed(Some(agent_id(6)));
    let cmds = st.on_p_pressed(None, t(0));
    assert_eq!(cmds, vec![PossessCommand::Unpossess]);
    assert_eq!(st.phase(), Phase::Possessed, "Unpossess 发出后等服务器确认");
    // 服务器确认解除。
    st.on_possessed(None);
    assert_eq!(st.phase(), Phase::HasBody);
}

/// (h) 请求等待期(SpawnRequested/PossessRequested)按 P 被忽略(转移表
/// "其他 → 忽略")。
#[test]
fn t6h_p_ignored_while_request_pending() {
    let mut st = PossessState::new();
    let _ = st.on_p_pressed(Some((Vec3::ZERO, 0.0)), t(0));
    let cmds = st.on_p_pressed(Some((Vec3::ONE, 0.0)), t(100));
    assert!(cmds.is_empty(), "SpawnRequested 期间按 P 忽略(不重复生成)");
    assert_eq!(st.phase(), Phase::SpawnRequested);
}

/// (i) 节流语义(传入 now):首条立即发;50 ms 窗内变化被节流;窗外变化
/// 发出;松键发零速度;跳跃按下沿即使窗内也立即发,按住不连发。
#[test]
fn t6i_intent_throttle_with_injected_now() {
    let mut st = PossessState::new();
    st.on_possessed(Some(agent_id(8)));
    // 首条(W):立即发。
    let cmds = st.on_keys(&keys(true, false, false, false, false), 0.0, t(0));
    assert_eq!(cmds.len(), 1, "首条意图立即发");
    // 窗内(30 ms)无变化:不发。
    let cmds = st.on_keys(&keys(true, false, false, false, false), 0.0, t(30));
    assert!(cmds.is_empty(), "50 ms 窗内无变化不发");
    // 窗内(45 ms)变化(W→D):被节流。
    let cmds = st.on_keys(&keys(false, false, false, true, false), 0.0, t(45));
    assert!(cmds.is_empty(), "窗内有变化也节流");
    // 窗外(60 ms,距上次发送 t(0) 已 60 ms):变化发出(D)。
    let cmds = st.on_keys(&keys(false, false, false, true, false), 0.0, t(60));
    assert_eq!(cmds.len(), 1, "窗外变化发出");
    // 窗内(70 ms)松键:零速度被节流。
    let cmds = st.on_keys(&keys(false, false, false, false, false), 0.0, t(70));
    assert!(cmds.is_empty(), "窗内零速度也节流");
    // 窗外(110 ms,距 t(60) 已 50 ms):零速度(松键)发出。
    let cmds = st.on_keys(&keys(false, false, false, false, false), 0.0, t(110));
    assert_eq!(cmds.len(), 1, "窗外零速度发出");
    match &cmds[0] {
        PossessCommand::Intent { move_xz, jump, .. } => {
            assert_eq!(*move_xz, (0.0, 0.0));
            assert!(!jump);
        }
        other => panic!("应为 Intent,实为 {other:?}"),
    }
    // 跳跃按下沿(窗内):立即发一条带 jump 的意图。
    let mut k = keys(false, false, false, false, false);
    k.jump_down = true;
    let cmds = st.on_keys(&k, 0.0, t(120));
    assert_eq!(cmds.len(), 1, "跳跃按下沿立即发(无视节流窗)");
    assert!(matches!(
        &cmds[0],
        PossessCommand::Intent { jump: true, .. }
    ));
    // 按住(下一帧仍 true):沿已过,不连发。
    let cmds = st.on_keys(&k, 0.0, t(200));
    assert!(cmds.is_empty(), "按住不连发");
    // 未附身:on_keys 空。
    let mut st2 = PossessState::new();
    let cmds = st2.on_keys(&k, 0.0, t(0));
    assert!(cmds.is_empty(), "未附身不发意图");
}

/// 回执去重:同一 ID 出现在 present 多帧,只有第一帧归属(receipts 删除)。
#[test]
fn t6j_receipt_consumed_once() {
    let mut st = PossessState::new();
    // 手动路径(P 有身体未附身)布景:回执 + 主体流 → HasBody;再按 P 发 Possess。
    st.on_spawned(agent_id(11));
    let _ = st.on_agents(&[agent_id(11)], &[]);
    assert_eq!(st.phase(), Phase::HasBody);
    assert_eq!(st.my_body(), Some(agent_id(11)));
    // 下一帧 11 还在 present:回执已消费,不再归属(不重复触发转移)。
    let cmds = st.on_agents(&[agent_id(11)], &[]);
    assert!(cmds.is_empty(), "回执已消费,不重复");
    assert_eq!(st.phase(), Phase::HasBody);
    // 自动路径:回执窗口内同 ID 第二次出现在 present,不再发 Possess。
    let mut st2 = PossessState::new();
    let _ = st2.on_p_pressed(Some((Vec3::ZERO, 0.0)), t(0));
    st2.on_spawned(agent_id(12));
    let cmds = st2.on_agents(&[agent_id(12)], &[]);
    assert_eq!(
        cmds,
        vec![PossessCommand::Possess(agent_id(12))],
        "首帧归属发 Possess"
    );
    let cmds = st2.on_agents(&[agent_id(12)], &[]);
    assert!(cmds.is_empty(), "第二帧不再发");
}

/// 回执窗口容量 64:挤掉最旧(不无限增长)。
#[test]
fn t6k_receipts_capped() {
    let mut st = PossessState::new();
    for n in 0..70_u32 {
        st.on_spawned(agent_id(n));
    }
    // 最早 6 条(0..6)已被挤掉:主体流出现 0 不再归属(无 auto_possess,
    // 归属也不发命令——用 my_body 判)。
    let _ = st.on_agents(&[agent_id(0)], &[]);
    assert_eq!(st.my_body(), None, "最旧回执已被挤出窗口,不归属");
    // 窗口内的 69 仍可归属。
    let _ = st.on_agents(&[agent_id(69)], &[]);
    assert_eq!(st.my_body(), Some(agent_id(69)), "窗口内回执仍有效");
}

/// (R2-5)自动附身的 Possess 无计时起点 → tick 补起算:按 P → 回执 →
/// on_agents 自动发 Possess(requested_at 为 None,该路径拿不到 now)→
/// 不给 Possessed 回执,推 2 s 多 → 状态回 HasBody,再按 P 能重发 Possess。
/// 变红实现:tick 不给 requested_at 补计时(服务器不回时永远停在
/// PossessRequested,P 键失效)。
#[test]
fn r2_5_tick_arms_timeout_for_auto_possess_request() {
    let mut st = PossessState::new();
    // 按 P 生成 → SpawnRequested(此刻有计时起点,走 on_agents 时被清)。
    let _ = st.on_p_pressed(Some((Vec3::ZERO, 0.0)), t(0));
    assert_eq!(st.phase(), Phase::SpawnRequested);
    st.on_spawned(agent_id(20));
    let cmds = st.on_agents(&[agent_id(20)], &[]);
    assert_eq!(
        cmds,
        vec![PossessCommand::Possess(agent_id(20))],
        "自动附身发 Possess"
    );
    assert_eq!(st.phase(), Phase::PossessRequested);
    // 服务器不回 Possessed:推 2.1 s(超过 REQUEST_TIMEOUT)→ HasBody。
    // 计时起点由第一帧 tick(t(0))补上——从这一帧起算 2 s。
    st.tick(t(0));
    st.tick(t(1_900));
    assert_eq!(st.phase(), Phase::PossessRequested, "1.9 s 还没超时");
    st.tick(t(2_100));
    assert_eq!(
        st.phase(),
        Phase::HasBody,
        "自动附身请求 2 s 无回应 → HasBody(超时兜底生效)"
    );
    // P 键恢复有效:再按 P 重发 Possess。
    let cmds = st.on_p_pressed(None, t(2_200));
    assert_eq!(
        cmds,
        vec![PossessCommand::Possess(agent_id(20))],
        "超时回退后 P 键能重新发 Possess"
    );
    assert_eq!(st.phase(), Phase::PossessRequested);
}

// ---------------------------------------------------------------------------
// 第三人称相机与滚轮
// ---------------------------------------------------------------------------

/// 第三人称相机:看向根位置上方 0.6 m,位于其后方 dist 处(yaw/pitch 同
/// 飞行相机约定)。pitch 是**相机的俯仰**(抬头为正,同飞行相机);相机
/// 在前向的反方向,故 pitch = +90(抬头)→ 相机在正下方。俯视身体
/// (pitch = −90)相机在头顶。
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
// T8(R1 §3.4):相机写回——third_person_camera 的 pos→look 方向与
// FlyCamera::forward() 同式(误差 ≤ 1e-5),写回后 to_camera() 画面不变。
// viewer 侧 forward = (cos yaw·cos pitch, sin pitch, sin yaw·cos pitch)。
// ---------------------------------------------------------------------------

fn fly_forward_deg(yaw_deg: f32, pitch_deg: f32) -> Vec3 {
    let yaw = yaw_deg.to_radians();
    let pitch = pitch_deg.to_radians();
    let (sy, cy) = yaw.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    Vec3::new(cy * cp, sp, sy * cp)
}

#[test]
fn t8_third_person_look_dir_matches_fly_forward() {
    let root = Vec3::new(37.5, -57.0, -12.5);
    for (yaw_deg, pitch_deg) in [
        (0.0_f32, 0.0_f32),
        (90.0, -30.0),
        (180.0, 45.0),
        (-45.0, -89.0),
    ] {
        let (pos, look) = third_person_camera(root, yaw_deg, pitch_deg, 4.0);
        let dir = (look - pos).normalize();
        let expect = fly_forward_deg(yaw_deg, pitch_deg);
        assert!(
            (dir - expect).length() <= 1e-5,
            "yaw {yaw_deg}° pitch {pitch_deg}°:pos→look 实得 {dir:?},期望 forward {expect:?}"
        );
    }
}

/// MUT15 锚(R1 §3.4):附身帧写回 FlyCamera 的位置 = third_person_camera
/// 的 pos(每帧写回;变异"不写回"= 渲染循环丢了这一步,该纯函数与写回点
/// 同源,本测试钉住函数本体;写回点经 render 循环接线)。
#[test]
fn mut15_possessed_camera_position_matches_third_person() {
    let root = Vec3::new(24.5, -58.1, 14.5);
    for (yaw_deg, pitch_deg, dist) in [
        (0.0_f32, 0.0_f32, 4.0_f32),
        (90.0, -25.0, 2.0),
        (-45.0, 60.0, 10.0),
        (180.0, 0.0, 7.5),
    ] {
        let got = possessed_camera_position(root, yaw_deg, pitch_deg, dist);
        let expect = third_person_camera(root, yaw_deg, pitch_deg, dist).0;
        assert!(
            (got - expect).length() <= 1e-5,
            "yaw {yaw_deg} pitch {pitch_deg} dist {dist}:实得 {got:?},期望 {expect:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// R2-3(#25):输入路由——附身时空格/滚轮/WASD 不再驱动通用交互
// ---------------------------------------------------------------------------

/// 非附身:三项全 true(既有行为不变);附身:三项全 false。
/// 变红实现:`input_routing(true)` 仍返回全 true(附身时空格既跳又切
/// 耦合——R1 验收的串键根因)。
#[test]
fn r2_3_input_routing_by_possession() {
    let free = input_routing(false);
    assert!(
        free.space_toggles_coupling && free.wheel_sets_launch_speed && free.wasd_moves_camera,
        "非附身:三项全 true(既有行为),实得 {free:?}"
    );
    let possessed = input_routing(true);
    assert!(
        !possessed.space_toggles_coupling
            && !possessed.wheel_sets_launch_speed
            && !possessed.wasd_moves_camera,
        "附身:三项全 false(空格=跳跃/滚轮=相机距离/WASD=行走意图),实得 {possessed:?}"
    );
}

// ---------------------------------------------------------------------------
// R3(#25 §6 修订):camera_after_body_lost / p_press_hint
// ---------------------------------------------------------------------------

/// 相机在世界高度以内 → 原样返回(主动按 P 退出附身的常规路径不受影响)。
/// 变红实现:恒返回 `current` 且情形不分(变异 7 同型)。
#[test]
fn r3_camera_within_world_height_returned_as_is() {
    let cur = Vec3::new(30.0, -40.0, 20.0);
    let got = camera_after_body_lost(cur, Some(Vec3::new(1.0, 2.0, 3.0)), -64.0);
    assert_eq!(got, cur, "y ≥ y_min → 原样返回(last_safe 不用)");
}

/// 相机落到世界底下(y < y_min)→ 用附身期间记录的最后一个安全位。
#[test]
fn r3_camera_below_world_uses_last_safe() {
    let cur = Vec3::new(30.0, -75.0, 20.0);
    let safe = Vec3::new(24.5, -50.0, 12.5);
    let got = camera_after_body_lost(cur, Some(safe), -64.0);
    assert_eq!(got, safe, "世界底下 → 放回最后安全相机位");
}

/// 世界底下且没有记录(last_safe = None)→ 只把 y 抬到 y_min + 16,
/// x/z 保持(至少回到能拾取地面的高度)。
#[test]
fn r3_camera_below_world_without_last_safe_lifts_y() {
    let cur = Vec3::new(30.0, -75.0, 20.0);
    let got = camera_after_body_lost(cur, None, -64.0);
    assert_eq!(
        got,
        Vec3::new(30.0, -48.0, 20.0),
        "无 last_safe → y = y_min + 16,x/z 保持"
    );
}

/// p_press_hint 各相位:只有 `NoBody` 且无拾取目标才提示;其余相位
/// (有身体、请求在途、已附身)与 `NoBody` 有目标都不提示。
#[test]
fn r3_p_press_hint_by_phase_and_target() {
    assert_eq!(
        p_press_hint(Phase::NoBody, false),
        Some("aim at the ground, then press P"),
        "NoBody 且无拾取目标 → 提示"
    );
    for (phase, name) in [
        (Phase::NoBody, "NoBody"),
        (Phase::SpawnRequested, "SpawnRequested"),
        (Phase::HasBody, "HasBody"),
        (Phase::PossessRequested, "PossessRequested"),
        (Phase::Possessed, "Possessed"),
    ] {
        assert_eq!(
            p_press_hint(phase, true),
            None,
            "{name} 有拾取目标 → 不提示"
        );
    }
    for (phase, name) in [
        (Phase::SpawnRequested, "SpawnRequested"),
        (Phase::HasBody, "HasBody"),
        (Phase::PossessRequested, "PossessRequested"),
        (Phase::Possessed, "Possessed"),
    ] {
        assert_eq!(
            p_press_hint(phase, false),
            None,
            "{name} 无拾取目标也不提示(有身体/请求在途/已附身的按 P 有别的语义)"
        );
    }
}
