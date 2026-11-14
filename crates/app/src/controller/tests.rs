//! B02a 控制者纯逻辑测试(任务书 §3 T1–T5、H1–H4;进门禁)。

use types::{EntityId, Tick};

use super::gait::{Gait, QUAD_DOF, QUAD_L, ik, stand_targets, trot_targets};
use super::{AttachError, ControllerHost, ControllerKind, ScriptKind, TROT_SETTLE_TICKS};

/// T1 的期望站立角(资产 `default_angle`,任务书 §3)。
const STAND_HIP: f32 = -0.853_596_4;
const STAND_KNEE: f32 = 1.707_192_8;

fn quad_asset() -> &'static str {
    include_str!("../../../../assets/bodies/quad.body.toml")
}

fn quad_def() -> types::BodyDef {
    let mut bodies = physics_sim::BodyRegistry::new();
    let hash = bodies
        .insert_toml(quad_asset())
        .expect("quad.body.toml 合法");
    bodies.get(&hash).expect("哈希必在").clone()
}

fn e(id: u32) -> EntityId {
    EntityId::from_parts(id, 1)
}

/// T1:`stand_targets` 等于资产里的 `default_angle`(lat 0,hip −0.853596389,
/// knee 1.70719278),误差 < 1e-6。
#[test]
fn t1_stand_targets_match_asset_default_angle() {
    let def = quad_def();
    let stand = stand_targets(&Gait::DEFAULT);
    for (dof_i, joint) in def.dof_joints().enumerate() {
        let want = joint
            .kind
            .params()
            .default_angle
            .expect("quad 全部有 default_angle");
        let got = stand[dof_i];
        let lim = joint.kind.params().limits.expect("quad 全部有 limits");
        assert!(
            lim.lower <= want && want <= lim.upper,
            "资产 default_angle 本身超限(布景自检)dof {dof_i}"
        );
        assert!(
            (got - want).abs() < 1e-6,
            "dof {dof_i}:stand {got} vs default_angle {want}"
        );
    }
    // 任务书给的字面值(防资产改错方向)。
    for leg in 0..4 {
        assert!(stand[leg * 3].abs() < 1e-6, "lat 恒 0");
        assert!((stand[leg * 3 + 1] - STAND_HIP).abs() < 1e-6);
        assert!((stand[leg * 3 + 2] - STAND_KNEE).abs() < 1e-6);
    }
}

/// T2:逆解回代——一个周期内 200 个采样时刻,由 (hip, knee) 正解回足端,
/// 与 `foot_target` 的误差 < 1e-5 m。
#[test]
fn t2_ik_roundtrip_within_1e5() {
    let gait = Gait::DEFAULT;
    let period = 1.0 / gait.freq;
    for leg in 0..4 {
        for i in 0..200 {
            let t = period * (i as f32 + 0.5) / 200.0 + 1.5; // 1.5 s:过渐入段
            let (dx, dy) = gait.foot_target(leg, t);
            let (hip, knee) = ik(dx, dy);
            // 正解(膝朝后,thigh/shank 各长 L):大腿方向 (sin·…, 由 hip 定义)。
            // hip = phi − k/2,phi = atan2(dx, −dy):大腿从髋指向 (sin(hip+…))。
            // 直接由几何回代:足端 = 髋 + R(hip)·(0,−L) + R(hip+k)·(0,−L),
            // R(θ) 为绕 z 轴旋转(膝朝后 = 取 +k 累加方向;demo ik 的定义)。
            let a1 = hip;
            let a2 = hip + knee;
            let fx = QUAD_L * a1.sin() + QUAD_L * a2.sin();
            let fy = -QUAD_L * a1.cos() - QUAD_L * a2.cos();
            assert!(
                ((fx - dx).abs() < 1e-5) && ((fy - dy).abs() < 1e-5),
                "leg {leg} t {t}:回代 ({fx},{fy}) vs 目标 ({dx},{dy})"
            );
        }
    }
}

/// T3:相位——`t` 取 1 秒之后(过渐入),腿 0 与腿 3 相同、腿 1 与腿 2
/// 相同;腿 0 与腿 1 相差半个周期。
#[test]
fn t3_trot_phases_diagonal_pairs() {
    let gait = Gait::DEFAULT;
    let period = 1.0 / gait.freq;
    let t = 1.5; // > 1 s 渐入
    let full = trot_targets(&gait, t);
    let half = trot_targets(&gait, t + period / 2.0);
    // 腿基数:FL=0、FR=1、RL=2、RR=3(dof 下标 = leg*3 + j)。
    const FL: usize = 0;
    const FR: usize = 1;
    const RL: usize = 2;
    const RR: usize = 3;
    for j in 0..3 {
        let fl = FL * 3 + j;
        let fr = FR * 3 + j;
        let rl = RL * 3 + j;
        let rr = RR * 3 + j;
        assert!((full[fl] - full[rr]).abs() < 1e-6, "FL vs RR joint {j}");
        assert!((full[fr] - full[rl]).abs() < 1e-6, "FR vs RL joint {j}");
        assert!(
            (full[fl] - half[fr]).abs() < 1e-6,
            "FL 与 FR 差半周期 joint {j}:{} vs {}",
            full[fl],
            half[fr]
        );
    }
}

/// T4:一个周期内所有目标在资产的关节限位之内;渐入——`t = 0` 时
/// `trot_targets` 等于站立目标。
#[test]
fn t4_trot_targets_within_limits_and_ramp_at_zero() {
    let def = quad_def();
    let limits: Vec<(f32, f32)> = def
        .dof_joints()
        .map(|j| {
            let lim = j.kind.params().limits.expect("quad 全部有 limits");
            (lim.lower, lim.upper)
        })
        .collect();
    assert_eq!(limits.len(), QUAD_DOF);
    let gait = Gait::DEFAULT;
    let period = 1.0 / gait.freq;
    // 渐入段之后的一个完整周期(支撑相 + 摆动相全覆盖)。
    for i in 0..200 {
        let t = 1.5 + period * i as f32 / 200.0;
        let targets = trot_targets(&gait, t);
        for (dof_i, &q) in targets.iter().enumerate() {
            let (lo, hi) = limits[dof_i];
            assert!(lo <= q && q <= hi, "dof {dof_i} t {t}:{q} 超限 [{lo},{hi}]");
        }
    }
    // 渐入:t = 0(步幅/抬脚 ramp = 0)→ 足端恒 (x0, −h) → 站立目标。
    let t0 = trot_targets(&gait, 0.0);
    let stand = stand_targets(&gait);
    assert_eq!(t0, stand, "t = 0 时 trot_targets 应等于站立目标");
}

/// T5:关节名顺序钉住——资产 joints 顺序 = `leg * 3 + {lat, hip, knee}`,
/// 腿序 FL、FR、RL、RR。
#[test]
fn t5_joint_order_matches_asset() {
    let def = quad_def();
    let names: Vec<&str> = def.dof_joints().map(|j| j.child.as_str()).collect();
    let legs = [
        "leg0_front_left",
        "leg1_front_right",
        "leg2_rear_left",
        "leg3_rear_right",
    ];
    let joints = ["_hip", "_thigh", "_shank"];
    let mut want = Vec::new();
    for leg in legs {
        for j in joints {
            want.push(format!("{leg}{j}"));
        }
    }
    assert_eq!(names, want, "dof 顺序 = 资产 joints 里非 fixed 顺序");
}

fn host_with(
    entity: EntityId,
    kind: ControllerKind,
    dof: usize,
    period: u32,
    now: u64,
) -> ControllerHost {
    let mut host = ControllerHost::default();
    host.attach(entity, kind, dof, period, Tick(now))
        .expect("挂载合法");
    host
}

/// H1:周期——period 2 挂在拍 10,拍 10–19 里只有 10、12、14、16、18 有
/// 动作;period 1 每拍都有。
#[test]
fn h1_period_gating() {
    let host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        2,
        10,
    );
    for now in 10..20u64 {
        let acts = host.actions_for_tick(Tick(now));
        if (now - 10) % 2 == 0 {
            assert_eq!(acts.len(), 1, "拍 {now} 应有动作");
            assert_eq!(acts[0].0, e(1));
            assert_eq!(acts[0].1.len(), QUAD_DOF);
        } else {
            assert!(acts.is_empty(), "拍 {now} 不应有动作");
        }
    }
    let host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        1,
        10,
    );
    for now in 10..20u64 {
        assert_eq!(
            host.actions_for_tick(Tick(now)).len(),
            1,
            "period 1 拍 {now} 应有动作"
        );
    }
}

/// H2:多只主体按 EntityId 升序输出;`None` 不输出。
#[test]
fn h2_multi_entity_sorted_and_none_silent() {
    let mut host = ControllerHost::default();
    // 故意降序挂载,验证输出仍升序。
    for id in [5u32, 3, 4] {
        host.attach(
            e(id),
            ControllerKind::Script(ScriptKind::QuadStand),
            QUAD_DOF,
            1,
            Tick(0),
        )
        .expect("挂载合法");
    }
    host.attach(e(2), ControllerKind::None, 1, 1, Tick(0))
        .expect("None 不限 dof");
    let acts = host.actions_for_tick(Tick(0));
    let ids: Vec<EntityId> = acts.iter().map(|(id, _)| *id).collect();
    assert_eq!(ids, vec![e(3), e(4), e(5)], "升序;None(e(2)) 不出现");
    for (_, targets) in &acts {
        assert_eq!(targets.len(), QUAD_DOF);
    }
}

/// H3:detach / on_removed 之后不再输出;重复挂同一只 → AlreadyAttached;
/// dof 11 → DofMismatch;period 0 → ZeroPeriod。
#[test]
fn h3_detach_remove_and_attach_errors() {
    let mut host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        1,
        0,
    );
    assert_eq!(host.actions_for_tick(Tick(1)).len(), 1);
    assert!(host.detach(e(1)), "detach 返回 true");
    assert!(!host.detach(e(1)), "重复 detach 返回 false");
    assert!(host.actions_for_tick(Tick(1)).is_empty(), "detach 后不输出");

    let mut host = host_with(
        e(2),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        1,
        0,
    );
    host.on_removed(e(2));
    assert!(
        host.actions_for_tick(Tick(1)).is_empty(),
        "on_removed 后不输出"
    );
    host.on_removed(e(999)); // 不存在也不报错

    let mut host = ControllerHost::default();
    host.attach(
        e(3),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        1,
        Tick(0),
    )
    .expect("首次挂载合法");
    assert_eq!(
        host.attach(
            e(3),
            ControllerKind::Script(ScriptKind::QuadStand),
            QUAD_DOF,
            1,
            Tick(0)
        ),
        Err(AttachError::AlreadyAttached)
    );
    assert_eq!(
        host.attach(
            e(4),
            ControllerKind::Script(ScriptKind::QuadStand),
            11,
            1,
            Tick(0)
        ),
        Err(AttachError::DofMismatch {
            expected: QUAD_DOF,
            got: 11
        })
    );
    // None 不限 dof:11 也收。
    host.attach(e(4), ControllerKind::None, 11, 1, Tick(0))
        .expect("None 不限 dof");
    assert_eq!(
        host.attach(
            e(5),
            ControllerKind::Script(ScriptKind::QuadStand),
            QUAD_DOF,
            0,
            Tick(0)
        ),
        Err(AttachError::ZeroPeriod)
    );
}

/// H5:drive 的被拒计数——send_agent_joint_targets_to 被拒(实体不存在,
/// 迁移窗口等)时 send_rejected += 1、actions_emitted 不变、log 长度不变,
/// 且继续不 panic。变异 9(drive 被拒不计数)/变异 11(drive 成功不
/// append)的捕手。
#[test]
fn h5_drive_rejected_counted() {
    let mut host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        1,
        0,
    );
    // 空会话:实体不存在 → owner 查表必拒(迁移窗口同路径)。
    let mut session = sim_api::session::Session::new(sim_api::session::SessionConfig::default());
    let mut log = sim_api::log::CommandLog::new();
    host.drive(&mut session, &mut log, Tick(0));
    session.shutdown();
    assert_eq!(host.counters.send_rejected, 1, "被拒恰计 1 次");
    assert_eq!(host.counters.actions_emitted, 0, "被拒不算发出");
    assert!(log.entries().is_empty(), "被拒时 log 不追加");
}

/// H4:QuadTrot 第 119 拍还是站立目标,第 120 拍起是步态;两次调用
/// `actions_for_tick` 结果相同(纯函数)。
#[test]
fn h4_trot_settle_boundary_and_pure() {
    use super::gait::{SIM_DT, trot_targets};
    // 常量钉住:任务书定 TROT_SETTLE_TICKS = 120(编译期防呆,变异 =1/=0
    // 时这里先红,不进下面的运行期断言)。
    assert_eq!(TROT_SETTLE_TICKS, 120, "TROT_SETTLE_TICKS 定值 120(§2.2)");
    let host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        1,
        0,
    );
    let stand = stand_targets(&Gait::DEFAULT).to_vec();
    let before = host.actions_for_tick(Tick(119));
    assert_eq!(before[0].1, stand, "第 119 拍仍是站立目标");
    let after = host.actions_for_tick(Tick(120));
    assert_eq!(
        after[0].1,
        trot_targets(&Gait::DEFAULT, 0.0).to_vec(),
        "第 120 拍起走 trot 分支(t = 0)"
    );
    let next = host.actions_for_tick(Tick(121));
    assert_eq!(
        next[0].1,
        trot_targets(&Gait::DEFAULT, SIM_DT).to_vec(),
        "第 121 拍 = trot_targets(SIM_DT)"
    );
    // 渐入段 t = 1/60 s 时步幅已非 0:支撑相足端 x ≠ x0 = 0,与站立分化。
    assert_ne!(
        next[0].1[1], stand[1],
        "渐入段的步态目标应与站立不同(支撑相 stride/ramp > 0)"
    );
    let again = host.actions_for_tick(Tick(TROT_SETTLE_TICKS));
    assert_eq!(after, again, "纯函数:同一拍两次调用结果相同");
}
