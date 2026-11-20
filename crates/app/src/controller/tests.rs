//! B02a 控制者纯逻辑测试(任务书 §3 T1–T5、H1–H4;进门禁)。

use types::{EntityId, Tick};

use super::gait::{Gait, QUAD_DOF, QUAD_L, ik, stand_targets, trot_targets};
use super::{
    AttachError, ControllerCounters, ControllerHost, ControllerKind, ScriptKind, TROT_SETTLE_TICKS,
};

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
    host.attach(entity, kind, dof, period, Tick(now), None)
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
    // B02b 第 0 节(验收缺口 1):奇数拍挂载——period 2 挂在拍 11,拍
    // 11–20 里只有 11、13、15、17、19 有动作(n = now − attached_at 的
    // 偶数门;`n` 不减 `attached_at` 的变异在偶数拍挂载的布景下活,这里
    // 钉住奇数拍挂载)。
    let host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        2,
        11,
    );
    for now in 11..21u64 {
        let acts = host.actions_for_tick(Tick(now));
        if now % 2 == 1 {
            assert_eq!(acts.len(), 1, "拍 {now} 应有动作(奇数拍)");
            assert_eq!(acts[0].0, e(1));
            assert_eq!(acts[0].1.len(), QUAD_DOF);
        } else {
            assert!(acts.is_empty(), "拍 {now} 不应有动作(偶数拍)");
        }
    }
    // 同缺口第二组:QuadTrot 挂在拍 7——settle 边界随挂载拍平移:第 126 拍
    // (n = 119)仍是站立目标,第 127 拍(n = 120)起是步态(t = 0)。
    let host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        1,
        7,
    );
    let stand = stand_targets(&Gait::DEFAULT).to_vec();
    let at126 = host.actions_for_tick(Tick(126));
    assert_eq!(at126.len(), 1, "第 126 拍应有动作(period 1)");
    assert_eq!(at126[0].1, stand, "第 126 拍(n = 119)仍是站立目标");
    let at127 = host.actions_for_tick(Tick(127));
    assert_eq!(
        at127[0].1,
        trot_targets(&Gait::DEFAULT, 0.0).to_vec(),
        "第 127 拍(n = 120)起走 trot 分支(t = 0)"
    );
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
            None,
        )
        .expect("挂载合法");
    }
    host.attach(e(2), ControllerKind::None, 1, 1, Tick(0), None)
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
        None,
    )
    .expect("首次挂载合法");
    assert_eq!(
        host.attach(
            e(3),
            ControllerKind::Script(ScriptKind::QuadStand),
            QUAD_DOF,
            1,
            Tick(0),
            None,
        ),
        Err(AttachError::AlreadyAttached)
    );
    assert_eq!(
        host.attach(
            e(4),
            ControllerKind::Script(ScriptKind::QuadStand),
            11,
            1,
            Tick(0),
            None,
        ),
        Err(AttachError::DofMismatch {
            expected: QUAD_DOF,
            got: 11
        })
    );
    // None 不限 dof:11 也收。
    host.attach(e(4), ControllerKind::None, 11, 1, Tick(0), None)
        .expect("None 不限 dof");
    assert_eq!(
        host.attach(
            e(5),
            ControllerKind::Script(ScriptKind::QuadStand),
            QUAD_DOF,
            0,
            Tick(0),
            None,
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
    host.drive(
        &mut session,
        &mut log,
        Tick(0),
        &std::collections::BTreeSet::new(),
        &std::collections::BTreeSet::new(),
    );
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

// ---- B02b(§2.3/§2.4/§2.5):L1 / L2 / L3 ----

use super::{ControllerRecord, ControllerSlot, Leash};

fn leash(center: [f32; 2], radius: f32) -> Option<Leash> {
    Some(Leash {
        center_xz: center,
        radius_m: radius,
    })
}

/// L1(§3):`apply_leash`——半径 2、圆心 (0,0):根在 1.9 m → 仍是
/// `QuadTrot`;2.1 m → 变 `QuadStand`、`leash_stops == 1`;再调一次不重复
/// 计数;走回圈内也不恢复。
#[test]
fn l1_apply_leash_stops_trot() {
    let mut host = ControllerHost::default();
    host.attach(
        e(1),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        1,
        Tick(0),
        leash([0.0, 0.0], 2.0),
    )
    .expect("挂载合法");
    // 1.9 m:圈内,不触发。
    host.apply_leash(|id| {
        assert_eq!(id, e(1));
        Some([1.9, 0.0])
    });
    assert_eq!(
        host.kind_of(e(1)),
        Some(ControllerKind::Script(ScriptKind::QuadTrot)),
        "1.9 m < 2.0 m:仍 QuadTrot"
    );
    assert_eq!(host.counters.leash_stops, 0, "未触发不计数");
    // 2.1 m:圈外,触发。
    host.apply_leash(|_| Some([2.1, 0.0]));
    assert_eq!(
        host.kind_of(e(1)),
        Some(ControllerKind::Script(ScriptKind::QuadStand)),
        "2.1 m > 2.0 m:永久改挂 QuadStand"
    );
    assert_eq!(host.counters.leash_stops, 1, "触发恰计 1 次");
    // attached_at 不动(改挂不重置相位)。
    let records = host.to_records();
    assert_eq!(records[0].slot.attached_at, Tick(0), "attached_at 不动");
    // 再调一次:已是 QuadStand,不再检查、不重复计数。
    host.apply_leash(|_| Some([100.0, 100.0]));
    assert_eq!(host.counters.leash_stops, 1, "不重复计数");
    // 走回圈内也不恢复(kind 已是 QuadStand,永不回 QuadTrot)。
    host.apply_leash(|_| Some([0.0, 0.0]));
    assert_eq!(
        host.kind_of(e(1)),
        Some(ControllerKind::Script(ScriptKind::QuadStand)),
        "改挂后不恢复"
    );
    assert_eq!(host.counters.leash_stops, 1);
    // 无 leash 的 QuadTrot 不受约束;根读不到(root_xz = None)也不触发。
    let mut host = ControllerHost::default();
    host.attach(
        e(2),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        1,
        Tick(0),
        None,
    )
    .expect("挂载合法(无 leash)");
    host.apply_leash(|_| Some([1000.0, 1000.0]));
    assert_eq!(
        host.kind_of(e(2)),
        Some(ControllerKind::Script(ScriptKind::QuadTrot)),
        "无 leash 不约束"
    );
    host.apply_leash(|_| None);
    assert_eq!(
        host.kind_of(e(2)),
        Some(ControllerKind::Script(ScriptKind::QuadTrot)),
        "根读不到不触发"
    );
    // R1-3:距离必须含 z 分量(dx²+dz²)——z 向/斜向组。圆心 (0,0)、半径 2:
    // (0, 2.1) 距离 2.1 → 触发;(1.5, 1.5) 距离 ≈2.12 → 触发;
    // (1.4, 1.4) 距离 ≈1.98 → 不触发。变异 `dx*dx + 0*dz` 下前两条存活。
    let mut host = ControllerHost::default();
    host.attach(
        e(3),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        1,
        Tick(0),
        leash([0.0, 0.0], 2.0),
    )
    .expect("挂载合法");
    host.apply_leash(|_| Some([0.0, 2.1]));
    assert_eq!(
        host.kind_of(e(3)),
        Some(ControllerKind::Script(ScriptKind::QuadStand)),
        "(0, 2.1):纯 z 向出圈,触发(实得仍 QuadTrot)"
    );
    let mut host = ControllerHost::default();
    host.attach(
        e(4),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        1,
        Tick(0),
        leash([0.0, 0.0], 2.0),
    )
    .expect("挂载合法");
    host.apply_leash(|_| Some([1.5, 1.5]));
    assert_eq!(
        host.kind_of(e(4)),
        Some(ControllerKind::Script(ScriptKind::QuadStand)),
        "(1.5, 1.5):斜向距离 2.12 > 2,触发(实得仍 QuadTrot)"
    );
    let mut host = ControllerHost::default();
    host.attach(
        e(5),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        1,
        Tick(0),
        leash([0.0, 0.0], 2.0),
    )
    .expect("挂载合法");
    host.apply_leash(|_| Some([1.4, 1.4]));
    assert_eq!(
        host.kind_of(e(5)),
        Some(ControllerKind::Script(ScriptKind::QuadTrot)),
        "(1.4, 1.4):斜向距离 1.98 < 2,不触发(实得 QuadStand)"
    );
}

/// L2(§3):`retain_alive` 摘掉不在的条目并返回条数。
#[test]
fn l2_retain_alive_removes_and_counts() {
    let mut host = ControllerHost::default();
    for id in [1u32, 2, 3] {
        host.attach(
            e(id),
            ControllerKind::Script(ScriptKind::QuadStand),
            QUAD_DOF,
            1,
            Tick(0),
            None,
        )
        .expect("挂载合法");
    }
    // e(2) 已不在:e(1)、e(3) 留。返回 1,输出只剩 2 条。
    let removed = host.retain_alive(|id| id != e(2));
    assert_eq!(removed, 1, "摘掉 1 条");
    assert_eq!(host.kind_of(e(2)), None, "e(2) 已摘");
    assert!(host.kind_of(e(1)).is_some(), "e(1) 应仍在");
    assert!(host.kind_of(e(3)).is_some(), "e(3) 应仍在");
    let removed = host.retain_alive(|_| true);
    assert_eq!(removed, 0, "全活:摘 0 条");
    let removed = host.retain_alive(|_| false);
    assert_eq!(removed, 2, "全死:摘 2 条");
    assert_eq!(host.to_records().len(), 0);
}

/// L3(§3):`to_records` / `from_records` 往返相等;序列化再反序列化相等。
#[test]
fn l3_records_roundtrip() {
    let mut host = ControllerHost::default();
    host.attach(
        e(1),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        2,
        Tick(7),
        leash([3.0, 4.0], 16.0),
    )
    .expect("挂载合法");
    host.attach(
        e(2),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        1,
        Tick(9),
        None,
    )
    .expect("挂载合法");
    host.attach(e(3), ControllerKind::None, 11, 1, Tick(0), None)
        .expect("None 不限 dof");
    // 往返:to_records → from_records → to_records 逐条相等。
    let records = host.to_records();
    let rebuilt = ControllerHost::from_records(records.clone());
    assert_eq!(rebuilt.to_records(), records, "往返逐条相等");
    // 计数清零(遥测不进检查点)。
    assert_eq!(rebuilt.counters, ControllerCounters::default());
    // 序列化再反序列化相等(bincode;检查点就是这个编码面)。
    let bytes = bincode::serde::encode_to_vec(&records, bincode::config::standard())
        .expect("records 可编码");
    let (back, _): (Vec<ControllerRecord>, usize) =
        bincode::serde::decode_from_slice(&bytes, bincode::config::standard())
            .expect("records 可解码");
    assert_eq!(back, records, "序列化往返相等");
    let rebuilt = ControllerHost::from_records(back);
    assert_eq!(
        rebuilt.kind_of(e(1)),
        Some(ControllerKind::Script(ScriptKind::QuadTrot))
    );
    assert_eq!(rebuilt.to_records()[0].slot.leash, leash([3.0, 4.0], 16.0));
    let _ = ControllerSlot {
        kind: ControllerKind::None,
        dof: 0,
        period_ticks: 1,
        attached_at: Tick(0),
        leash: None,
        intent: [0.0; 3],
    };
}

/// H6(B03a 验收 R1):在途跳过——挂一只脚本控制者(QuadStand、period 1,
/// attached_at = 0,Tick(10) 是控制拍),`drive` 的 `skip` 含该实体。
/// 断言 `skipped_in_transit == 1`、`send_rejected == 0`、
/// `actions_emitted == 0`、日志为空(在途投递会被内核静默拒绝,提前跳过
/// 不走被拒路径)。对照:同一布景 `skip` 为空时 `send_rejected == 1`
/// (H5 已覆盖空会话拒绝路径,此处不再重复建对照布景)。
/// 变异 9(drive 不跳过在途)注入后本测试红。
#[test]
fn h6_drive_skips_in_transit() {
    let mut host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        1,
        0,
    );
    // 空会话:实体不存在 → 若不跳过,send 必被拒(H5 同路径)。skip 含
    // 该实体 → 在途分支接管。
    let mut session = sim_api::session::Session::new(sim_api::session::SessionConfig::default());
    let mut log = sim_api::log::CommandLog::new();
    let mut skip = std::collections::BTreeSet::new();
    skip.insert(e(1));
    let frozen = std::collections::BTreeSet::new();
    host.drive(&mut session, &mut log, Tick(10), &skip, &frozen);
    session.shutdown();
    assert_eq!(host.counters.skipped_in_transit, 1, "在途跳过恰计 1 次");
    assert_eq!(host.counters.send_rejected, 0, "跳过不走被拒路径");
    assert_eq!(host.counters.actions_emitted, 0, "跳过不算发出");
    assert!(log.entries().is_empty(), "跳过时 log 不追加");
}

/// H8(B03c 3.3):冻结跳过——`frozen` 含该实体的控制拍 →
/// `skipped_frozen == 1`、`send_rejected == 0`、`actions_emitted == 0`、
/// 日志为空(冻结期间控制者不发目标,§0 规则 5)。
/// 变异 6(drive 不跳过冻结)注入后本测试红。
#[test]
fn h8_drive_skips_frozen() {
    let mut host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        1,
        0,
    );
    // 空会话:实体不存在 → 若不跳过,send 必被拒(H6 同路径)。frozen 含
    // 该实体 → 冻结分支接管。
    let mut session = sim_api::session::Session::new(sim_api::session::SessionConfig::default());
    let mut log = sim_api::log::CommandLog::new();
    let skip = std::collections::BTreeSet::new();
    let mut frozen = std::collections::BTreeSet::new();
    frozen.insert(e(1));
    host.drive(&mut session, &mut log, Tick(10), &skip, &frozen);
    session.shutdown();
    assert_eq!(host.counters.skipped_frozen, 1, "冻结跳过恰计 1 次");
    assert_eq!(host.counters.send_rejected, 0, "跳过不走被拒路径");
    assert_eq!(host.counters.actions_emitted, 0, "跳过不算发出");
    assert!(log.entries().is_empty(), "跳过时 log 不追加");
}

/// H7(B03a 验收 R1):挂载拍在未来——`attached_at = Tick(5)`、周期 2。
/// `actions_for_tick(Tick(3))`、`Tick(4)` 为空(挂载拍在未来不发目标);
/// `Tick(5)` 有 1 条;`Tick(6)` 为空(n=1 非控制拍);`Tick(7)` 有 1 条。
/// startup attach 的 `attached_at = spawn 拍 +1` 落在这里:去掉
/// `now.0 < slot.attached_at.0` 判断后 Tick(3/4) 会因 saturating_sub 下溢
/// 把 n 折 0 误触发,本测试红。
#[test]
fn h7_future_attached_at_not_emitted() {
    let host = host_with(
        e(1),
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        2,
        5,
    );
    assert!(
        host.actions_for_tick(Tick(3)).is_empty(),
        "挂载拍在未来(Tick(3) < Tick(5)):不发目标"
    );
    assert!(
        host.actions_for_tick(Tick(4)).is_empty(),
        "挂载拍在未来(Tick(4) < Tick(5)):不发目标"
    );
    let at5 = host.actions_for_tick(Tick(5));
    assert_eq!(at5.len(), 1, "挂载拍 Tick(5):n = 0,恰发 1 条");
    assert_eq!(at5[0].0, e(1));
    assert!(
        host.actions_for_tick(Tick(6)).is_empty(),
        "Tick(6):n = 1 非控制拍,不发"
    );
    let at7 = host.actions_for_tick(Tick(7));
    assert_eq!(at7.len(), 1, "Tick(7):n = 2 控制拍,发 1 条");
}
// ---- D01b-1(§1.2/§1.3):U1–U4、build_snapshot、策略驱动 ----

use super::policy_table::PolicyTable;
use super::policy_worker::{PolicyWorker, WorkerOptions};
use super::{PolicyKey, build_snapshot};

/// 测试用工作线程:等待预算 5 s(余量充足)、无调试延迟。
fn worker5s(table: PolicyTable) -> PolicyWorker {
    PolicyWorker::spawn(
        table,
        WorkerOptions {
            wait_budget: std::time::Duration::from_secs(5),
            debug_delay: std::time::Duration::ZERO,
        },
    )
}

/// U1:`PolicyKey::of` = 名字 UTF-8 字节的 FNV-1a(与 BodyHash 同算法)。
#[test]
fn u1_policy_key_fnv1a_of_name() {
    assert_eq!(
        PolicyKey::of("stand"),
        PolicyKey::of("stand"),
        "同名两次相同"
    );
    assert_ne!(PolicyKey::of("stand"), PolicyKey::of("wobble"));
    assert_eq!(
        PolicyKey::of("stand").0,
        types::BodyHash::fnv1a(b"stand").0,
        "与 BodyHash::fnv1a 同值"
    );
}

/// U2:策略槽位在 skip 里 → skipped_in_transit += 1、不推理;在 frozen
/// 里 → skipped_frozen += 1、不推理(空会话:若不跳过会走 policy_no_state,
/// 计数对不上即红;变异 9 注入后本测试红)。
#[test]
fn u2_policy_drive_skips_in_transit_and_frozen() {
    let mut host = host_with(
        e(1),
        ControllerKind::Policy(PolicyKey::of("stand")),
        QUAD_DOF,
        2,
        0,
    );
    let mut session = sim_api::session::Session::new(sim_api::session::SessionConfig::default());
    let mut log = sim_api::log::CommandLog::new();
    let mut worker = worker5s(PolicyTable::default());
    let mut skip = std::collections::BTreeSet::new();
    skip.insert(e(1));
    let frozen = std::collections::BTreeSet::new();
    host.drive_policies(
        &mut session,
        &mut log,
        &mut worker,
        Tick(10),
        &skip,
        &frozen,
    );
    assert_eq!(host.counters.skipped_in_transit, 1, "在途跳过恰计 1 次");
    assert_eq!(host.counters.policy_inferences, 0, "跳过不推理");
    assert_eq!(host.counters.policy_no_state, 0, "跳过不走取状态分支");
    assert!(log.entries().is_empty(), "跳过时 log 不追加");
    let skip = std::collections::BTreeSet::new();
    let mut frozen = std::collections::BTreeSet::new();
    frozen.insert(e(1));
    host.drive_policies(
        &mut session,
        &mut log,
        &mut worker,
        Tick(12),
        &skip,
        &frozen,
    );
    assert_eq!(host.counters.skipped_frozen, 1, "冻结跳过恰计 1 次");
    assert_eq!(host.counters.policy_inferences, 0, "跳过不推理");
    assert_eq!(host.counters.policy_no_state, 0, "跳过不走取状态分支");
    assert!(log.entries().is_empty());
    session.shutdown();
}

/// U3:`ControllerRecord` 带 Policy 槽位 bincode 往返逐字段相等(intent
/// 在内);from_records 重建后 last_action 从全 0 开始。
#[test]
fn u3_policy_record_roundtrip() {
    let mut host = ControllerHost::default();
    host.attach(
        e(1),
        ControllerKind::Policy(PolicyKey::of("stand")),
        QUAD_DOF,
        2,
        Tick(7),
        leash([3.0, 4.0], 16.0),
    )
    .expect("挂载合法");
    host.set_intent(e(1), [1.0, 0.0, -1.0]);
    let records = host.to_records();
    let bytes = bincode::serde::encode_to_vec(&records, bincode::config::standard())
        .expect("records 可编码");
    let (back, _): (Vec<ControllerRecord>, usize) =
        bincode::serde::decode_from_slice(&bytes, bincode::config::standard())
            .expect("records 可解码");
    assert_eq!(back, records, "序列化往返相等");
    assert_eq!(
        back[0].slot.kind,
        ControllerKind::Policy(PolicyKey::of("stand"))
    );
    assert_eq!(back[0].slot.intent, [1.0, 0.0, -1.0], "intent 随档走");
    let rebuilt = ControllerHost::from_records(back);
    assert_eq!(rebuilt.to_records(), records, "重建往返相等");
    assert_eq!(
        rebuilt.policy_state[&e(1)].last_action,
        vec![0.0; QUAD_DOF],
        "恢复后 last_action 从全 0 开始"
    );
}

/// U4:retain_alive / on_removed / detach 摘掉策略槽位后 policy_state
/// 无残留;同一 EntityId 重新挂上,last_action 从全 0 开始。
#[test]
fn u4_policy_state_cleared_on_removal() {
    let kind = ControllerKind::Policy(PolicyKey::of("wobble"));
    let mut host = host_with(e(1), kind, QUAD_DOF, 2, 0);
    assert!(host.policy_state.contains_key(&e(1)), "挂载即建运行态");
    assert_eq!(host.retain_alive(|_| false), 1);
    assert!(host.policy_state.is_empty(), "retain_alive 后无残留");
    host.attach(e(1), kind, QUAD_DOF, 2, Tick(0), None)
        .expect("重新挂载");
    assert_eq!(
        host.policy_state[&e(1)].last_action,
        vec![0.0; QUAD_DOF],
        "重挂后从全 0 开始"
    );
    host.on_removed(e(1));
    assert!(host.policy_state.is_empty(), "on_removed 后无残留");
    host.attach(e(1), kind, QUAD_DOF, 2, Tick(0), None)
        .expect("重新挂载");
    assert!(host.detach(e(1)));
    assert!(host.policy_state.is_empty(), "detach 后无残留");
}

/// Q8(单测版):空宿主调 drive_policies → 计数全 0、日志为空
/// (没有策略槽位时零动作,不调 Session 的状态方法)。
#[test]
fn q8_empty_host_drive_policies_no_change() {
    let mut host = ControllerHost::default();
    let mut session = sim_api::session::Session::new(sim_api::session::SessionConfig::default());
    let mut log = sim_api::log::CommandLog::new();
    let mut worker = worker5s(PolicyTable::default());
    let empty = std::collections::BTreeSet::new();
    host.drive_policies(
        &mut session,
        &mut log,
        &mut worker,
        Tick(10),
        &empty,
        &empty,
    );
    session.shutdown();
    assert_eq!(host.counters, ControllerCounters::default(), "计数全 0");
    assert!(log.entries().is_empty(), "日志为空");
}

/// build_snapshot 逐字段透传(钉住变异 3:joint_pos 传成全 0)。
#[test]
fn build_snapshot_passes_fields_through() {
    let root = types::EntityState {
        translation: types::glam::Vec3::new(1.0, 2.0, 3.0),
        rotation: types::glam::Quat::from_rotation_y(0.3),
        linvel: types::glam::Vec3::new(0.4, 0.5, 0.6),
        angvel: types::glam::Vec3::new(0.7, 0.8, 0.9),
    };
    let joints = sim_api::session::AgentJointState {
        positions: vec![0.11, 0.22, 0.33],
        velocities: vec![0.44, 0.55, 0.66],
    };
    let snap = build_snapshot(&root, &joints, [1.0, 0.0, -1.0], &[0.77, 0.88, 0.99]);
    assert_eq!(snap.root_rotation, root.rotation);
    assert_eq!(snap.root_linvel, root.linvel);
    assert_eq!(snap.root_angvel, root.angvel);
    assert_eq!(snap.joint_pos, joints.positions, "关节角原样透传");
    assert_eq!(snap.joint_vel, joints.velocities, "关节角速度原样透传");
    assert_eq!(snap.intent, [1.0, 0.0, -1.0], "intent 原样带入");
    assert_eq!(snap.last_action, vec![0.77, 0.88, 0.99]);
}

/// 推理成功后运行态 last_action = action.raw(钉住变异 4:不更新)。
/// 真实会话(单分片 + 平地 + 生成四足),wobble 布局(lin6x12:
/// base_ang_vel + intent),intent [1,0,-1] → raw[3] = 0.4、raw[5] = −0.6
/// (任务书 Q3 手算同源:输出只接 intent[0]、intent[2])。
/// 变异 2(intent 恒 0)注入后本测试也红。
#[test]
fn policy_drive_updates_last_action() {
    let mut bodies = physics_sim::BodyRegistry::new();
    let hash = bodies.insert_toml(quad_asset()).expect("quad 合法");
    let def = bodies.get(&hash).expect("哈希必在").clone();
    let shard = types::ShardId::new(
        types::WorldId(0),
        types::DimensionId(0),
        types::ShardPos::new(0, 0),
    );
    let mut cfg = physics_sim::ShardSimConfig::new(shard);
    cfg.solidify.enabled = false;
    cfg.cohesion.enabled = false;
    let sim = physics_sim::ShardSim::new_with_bodies(
        cfg,
        Box::new(voxel::SimpleRegistry::builtin()),
        bodies.clone(),
    );
    let mut session = sim_api::session::Session::new(sim_api::session::SessionConfig::default());
    session.register(sim);
    for cx in 0..8i32 {
        for cz in 0..8i32 {
            let mut chunk = voxel::Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    chunk.set(x, 64, z, types::BlockId(1));
                }
            }
            session.apply_chunk_load(shard, types::ChunkPos::new(cx, cz), chunk);
        }
    }
    let a = e(1);
    session
        .spawn_agent(
            shard,
            a,
            hash,
            types::EntityState {
                translation: types::glam::Vec3::new(8.0, 66.0, 8.0),
                rotation: types::glam::Quat::IDENTITY,
                linvel: types::glam::Vec3::ZERO,
                angvel: types::glam::Vec3::ZERO,
            },
        )
        .expect("生成四足");
    session.tick();
    // 策略包:临时目录现拼(toml 字符串 + 夹具 ONNX)。
    let dir = tempfile::TempDir::new().expect("临时目录");
    let pkg = dir.path().join("wobble");
    std::fs::create_dir(&pkg).expect("建策略包目录");
    std::fs::write(
        pkg.join("policy.toml"),
        format!(
            "name = \"wobble\"\nbody_hash = \"{}\"\ncontrol_period_ticks = 2\naction_scale = 0.1\naction_clip = 1.0\nobs = [{{ term = \"base_ang_vel\" }}, {{ term = \"intent\" }}]\n",
            hash.hex()
        ),
    )
    .expect("写 policy.toml");
    std::fs::write(
        pkg.join("policy.onnx"),
        include_bytes!("../../../../assets/policies/test_fixtures/lin6x12.onnx"),
    )
    .expect("写 policy.onnx");
    let mut table = PolicyTable::default();
    let key = table.load(dir.path(), "wobble", &def).expect("策略包合法");
    assert_eq!(key, PolicyKey::of("wobble"));
    let mut worker = worker5s(table);
    let mut host = ControllerHost::default();
    host.attach(a, ControllerKind::Policy(key), QUAD_DOF, 1, Tick(0), None)
        .expect("挂载合法");
    host.set_intent(a, [1.0, 0.0, -1.0]);
    let mut log = sim_api::log::CommandLog::new();
    let empty = std::collections::BTreeSet::new();
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(1), &empty, &empty);
    assert_eq!(host.counters.policy_inferences, 1, "成功推理恰 1 次");
    assert_eq!(host.counters.policy_errors, 0);
    assert_eq!(host.counters.actions_emitted, 1);
    assert_eq!(log.entries().len(), 1, "动作进日志");
    let last = &host.policy_state[&a].last_action;
    assert_eq!(last.len(), QUAD_DOF);
    assert!(
        (last[3] - 0.4).abs() < 1e-5,
        "last_action[3] = raw 0.4,实得 {}",
        last[3]
    );
    assert!(
        (last[5] + 0.6).abs() < 1e-5,
        "last_action[5] = raw −0.6,实得 {}",
        last[5]
    );
    session.shutdown();
}

// ---- D01b-2(§1.3/§2.1):工作线程 W1–W5、meta -------------------------------

const STAND_ONNX: &[u8] =
    include_bytes!("../../../../assets/policies/test_fixtures/zero45x12.onnx");
const BROKEN_ONNX: &[u8] = include_bytes!("../../../../assets/policies/test_fixtures/nan6x12.onnx");
const STAND_OBS: &str = "obs = [{ term = \"base_lin_vel\" }, { term = \"base_ang_vel\" }, { term = \"projected_gravity\" }, { term = \"joint_pos\" }, { term = \"joint_vel\" }, { term = \"last_action\" }]\n";
const BROKEN_OBS: &str = "obs = [{ term = \"base_ang_vel\" }, { term = \"intent\" }]\n";

/// W 组布景(复用 `policy_drive_updates_last_action` 的真实会话):单分片 +
/// 平地 + 生成四足,再加载一个策略包。返回 (Session, 主体, 已加载策略表)。
/// 控制周期由调用方 attach(任务书:周期 2,t0 = 第一个控制拍 Tick(0))。
fn policy_session(
    name: &str,
    onnx: &[u8],
    obs: &str,
    scale: f32,
    clip: Option<f32>,
) -> (sim_api::session::Session, EntityId, PolicyTable) {
    let mut bodies = physics_sim::BodyRegistry::new();
    let hash = bodies.insert_toml(quad_asset()).expect("quad 合法");
    let def = bodies.get(&hash).expect("哈希必在").clone();
    let shard = types::ShardId::new(
        types::WorldId(0),
        types::DimensionId(0),
        types::ShardPos::new(0, 0),
    );
    let mut cfg = physics_sim::ShardSimConfig::new(shard);
    cfg.solidify.enabled = false;
    cfg.cohesion.enabled = false;
    let sim = physics_sim::ShardSim::new_with_bodies(
        cfg,
        Box::new(voxel::SimpleRegistry::builtin()),
        bodies.clone(),
    );
    let mut session = sim_api::session::Session::new(sim_api::session::SessionConfig::default());
    session.register(sim);
    for cx in 0..8i32 {
        for cz in 0..8i32 {
            let mut chunk = voxel::Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    chunk.set(x, 64, z, types::BlockId(1));
                }
            }
            session.apply_chunk_load(shard, types::ChunkPos::new(cx, cz), chunk);
        }
    }
    let a = e(1);
    session
        .spawn_agent(
            shard,
            a,
            hash,
            types::EntityState {
                translation: types::glam::Vec3::new(8.0, 66.0, 8.0),
                rotation: types::glam::Quat::IDENTITY,
                linvel: types::glam::Vec3::ZERO,
                angvel: types::glam::Vec3::ZERO,
            },
        )
        .expect("生成四足");
    session.tick();
    let dir = tempfile::TempDir::new().expect("临时目录");
    let pkg = dir.path().join(name);
    std::fs::create_dir(&pkg).expect("建策略包目录");
    let clip_line = clip
        .map(|c| format!("action_clip = {c}\n"))
        .unwrap_or_default();
    std::fs::write(
        pkg.join("policy.toml"),
        format!(
            "name = \"{name}\"\nbody_hash = \"{}\"\ncontrol_period_ticks = 2\naction_scale = {scale}\n{clip_line}{obs}",
            hash.hex()
        ),
    )
    .expect("写 policy.toml");
    std::fs::write(pkg.join("policy.onnx"), onnx).expect("写 policy.onnx");
    let mut table = PolicyTable::default();
    table.load(dir.path(), name, &def).expect("策略包合法");
    (session, a, table)
}

/// 挂一个周期 2、attached_at = Tick(0) 的策略槽位(t0 = Tick(0))。
fn attach_policy(host: &mut ControllerHost, a: EntityId, name: &str) {
    host.attach(
        a,
        ControllerKind::Policy(PolicyKey::of(name)),
        QUAD_DOF,
        2,
        Tick(0),
        None,
    )
    .expect("挂载合法");
}

/// W1 当拍收到:wait_budget 5 s、debug_delay 0。drive(t0) 后推理 1、晚到 0、
/// 超时 0、requests_sent 1、日志 1 条。
#[test]
fn w1_reply_within_tick() {
    let (mut session, a, table) = policy_session("stand", STAND_ONNX, STAND_OBS, 0.25, None);
    let mut worker = worker5s(table);
    let mut host = ControllerHost::default();
    attach_policy(&mut host, a, "stand");
    let mut log = sim_api::log::CommandLog::new();
    let empty = std::collections::BTreeSet::new();
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(0), &empty, &empty);
    assert_eq!(host.counters.policy_inferences, 1, "当拍推理 1 次");
    assert_eq!(host.counters.policy_late, 0, "当拍收到不算晚到");
    assert_eq!(host.counters.policy_timeouts, 0);
    assert_eq!(worker.requests_sent(), 1);
    assert_eq!(log.entries().len(), 1, "动作进日志");
    session.shutdown();
}

/// W2 超时与作废:wait_budget 1 ms、debug_delay 300 ms。
#[test]
fn w2_timeout_and_abandon() {
    let (mut session, a, table) = policy_session("stand", STAND_ONNX, STAND_OBS, 0.25, None);
    let mut worker = PolicyWorker::spawn(
        table,
        WorkerOptions {
            wait_budget: std::time::Duration::from_millis(1),
            debug_delay: std::time::Duration::from_millis(300),
        },
    );
    let mut host = ControllerHost::default();
    attach_policy(&mut host, a, "stand");
    let mut log = sim_api::log::CommandLog::new();
    let empty = std::collections::BTreeSet::new();
    // 1. drive(t0):发出请求,1 ms 内没回。
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(0), &empty, &empty);
    assert_eq!(host.counters.policy_inferences, 0);
    assert_eq!(worker.requests_sent(), 1);
    // 2. drive(t0+1)(非控制拍):计数不变。
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(1), &empty, &empty);
    assert_eq!(worker.requests_sent(), 1);
    assert_eq!(host.counters.policy_timeouts, 0);
    // 3. drive(t0+2)(控制拍):上一个还没回 → 超时 1、不发新请求。
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(2), &empty, &empty);
    assert_eq!(host.counters.policy_timeouts, 1);
    assert_eq!(worker.requests_sent(), 1, "在途时不发新请求");
    // 4. sleep(600 ms) 让作废的回复回来;drive(t0+3):丢弃 1、推理 0、日志 0。
    std::thread::sleep(std::time::Duration::from_millis(600));
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(3), &empty, &empty);
    assert_eq!(host.counters.policy_discarded, 1, "作废回复被丢弃");
    assert_eq!(host.counters.policy_inferences, 0);
    assert_eq!(log.entries().len(), 0);
    // 5. drive(t0+4)(控制拍):再下一个控制拍才发新请求 → requests_sent 2、超时仍 1。
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(4), &empty, &empty);
    assert_eq!(worker.requests_sent(), 2);
    assert_eq!(host.counters.policy_timeouts, 1);
    session.shutdown();
}

/// W3 晚到但在周期内:wait_budget 1 ms、debug_delay 50 ms。drive(t0) 推理 0;
/// sleep(300 ms);drive(t0+1) 推理 1、晚到 1、发出 1、日志 1、超时 0。
#[test]
fn w3_late_within_period() {
    let (mut session, a, table) = policy_session("stand", STAND_ONNX, STAND_OBS, 0.25, None);
    let mut worker = PolicyWorker::spawn(
        table,
        WorkerOptions {
            wait_budget: std::time::Duration::from_millis(1),
            debug_delay: std::time::Duration::from_millis(50),
        },
    );
    let mut host = ControllerHost::default();
    attach_policy(&mut host, a, "stand");
    let mut log = sim_api::log::CommandLog::new();
    let empty = std::collections::BTreeSet::new();
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(0), &empty, &empty);
    assert_eq!(host.counters.policy_inferences, 0);
    std::thread::sleep(std::time::Duration::from_millis(300));
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(1), &empty, &empty);
    assert_eq!(host.counters.policy_inferences, 1, "晚到回复被采用");
    assert_eq!(host.counters.policy_late, 1);
    assert_eq!(host.counters.actions_emitted, 1);
    assert_eq!(log.entries().len(), 1);
    assert_eq!(host.counters.policy_timeouts, 0);
    session.shutdown();
}

/// W4 在途时主体被移除:wait_budget 1 ms、debug_delay 100 ms。drive(t0);
/// on_removed;sleep(400 ms);drive(t0+1) → 丢弃 1、推理 0、不 panic。
#[test]
fn w4_removed_while_pending() {
    let (mut session, a, table) = policy_session("stand", STAND_ONNX, STAND_OBS, 0.25, None);
    let mut worker = PolicyWorker::spawn(
        table,
        WorkerOptions {
            wait_budget: std::time::Duration::from_millis(1),
            debug_delay: std::time::Duration::from_millis(100),
        },
    );
    let mut host = ControllerHost::default();
    attach_policy(&mut host, a, "stand");
    let mut log = sim_api::log::CommandLog::new();
    let empty = std::collections::BTreeSet::new();
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(0), &empty, &empty);
    host.on_removed(a);
    std::thread::sleep(std::time::Duration::from_millis(400));
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(1), &empty, &empty);
    assert_eq!(host.counters.policy_discarded, 1, "主体已无槽位 → 丢弃");
    assert_eq!(host.counters.policy_inferences, 0);
    session.shutdown();
}

/// W5 坏策略:broken、wait_budget 5 s。drive(t0) 错误 1、推理 0、pending 已清;
/// 再 drive(t0+2) 错误 2、超时 0。
#[test]
fn w5_broken_policy_clears_pending() {
    let (mut session, a, table) = policy_session("broken", BROKEN_ONNX, BROKEN_OBS, 0.25, None);
    let mut worker = worker5s(table);
    let mut host = ControllerHost::default();
    attach_policy(&mut host, a, "broken");
    let mut log = sim_api::log::CommandLog::new();
    let empty = std::collections::BTreeSet::new();
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(0), &empty, &empty);
    assert_eq!(host.counters.policy_errors, 1);
    assert_eq!(host.counters.policy_inferences, 0);
    host.drive_policies(&mut session, &mut log, &mut worker, Tick(2), &empty, &empty);
    assert_eq!(
        host.counters.policy_errors, 2,
        "Err 清 pending,下个控制拍再发"
    );
    assert_eq!(host.counters.policy_timeouts, 0);
    session.shutdown();
}

/// meta()(D01b-2 裁定):stand 的周期 = manifest 写的值、dof = 身体自由度、
/// 条目数 = 已加载策略数。
#[test]
fn policy_meta_matches_manifest() {
    let (session, _a, table) = policy_session("stand", STAND_ONNX, STAND_OBS, 0.25, None);
    let meta = table.meta();
    assert_eq!(meta.len(), 1, "条目数 = 已加载策略数");
    let m = meta[&PolicyKey::of("stand")];
    assert_eq!(m.control_period_ticks, 2, "周期 = manifest 里写的值");
    assert_eq!(m.dof, QUAD_DOF, "dof = 身体自由度数");
    session.shutdown();
}

// ---- E03 §4.1:policy_intent 换算(M-a/M-b/M-c,手算期望,容差 1e-6)----

fn assert_intent(got: [f32; 3], want: [f32; 3], tag: &str) {
    for i in 0..3 {
        assert!(
            (got[i] - want[i]).abs() < 1e-6,
            "{tag} 分量 {i}:得 {} 期望 {}",
            got[i],
            want[i]
        );
    }
}

/// M-a:h=0(单位四元数)。
#[test]
fn policy_intent_heading_zero() {
    use super::policy_intent;
    use types::glam::Quat;
    let id = Quat::IDENTITY;
    assert_intent(
        policy_intent(id, Some((0.6, 0.0)), None),
        [1.0, 0.0, 0.0],
        "M-a (0.6,0)",
    );
    assert_intent(
        policy_intent(id, Some((0.0, 0.3)), None),
        [0.0, 1.0, 0.0],
        "M-a (0,0.3)",
    );
    assert_intent(
        policy_intent(id, Some((1.2, 0.0)), None),
        [1.0, 0.0, 0.0],
        "M-a a 钳 1",
    );
    assert_intent(
        policy_intent(id, None, None),
        [0.0, 0.0, 0.0],
        "M-a move None",
    );
    // R1-b:区分 V_FWD_MAX(0.3/0.6=0.5)与 V_LAT_MAX(0.15/0.3=0.5)。
    assert_intent(
        policy_intent(id, Some((0.3, 0.0)), None),
        [0.5, 0.0, 0.0],
        "M-a (0.3,0)→0.5",
    );
    assert_intent(
        policy_intent(id, Some((0.0, 0.15)), None),
        [0.0, 0.5, 0.0],
        "M-a (0,0.15)→0.5",
    );
}

/// M-b:h=−π/2(绕 y +90°)。
#[test]
fn policy_intent_heading_neg_pi_2() {
    use super::policy_intent;
    use types::glam::Quat;
    let rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    assert_intent(
        policy_intent(rot, Some((0.6, 0.0)), None),
        [0.0, 1.0, 0.0],
        "M-b (0.6,0)",
    );
    assert_intent(
        policy_intent(rot, Some((0.0, 0.6)), None),
        [-1.0, 0.0, 0.0],
        "M-b (0,0.6)",
    );
}

/// M-c:偏航速率占比 c(wrap 差,不是绝对航向)。
#[test]
fn policy_intent_yaw_rate() {
    use super::policy_intent;
    use types::glam::Quat;
    let id = Quat::IDENTITY;
    assert_intent(
        policy_intent(id, None, Some(0.4)),
        [0.0, 0.0, -0.4],
        "M-c yaw 0.4",
    );
    assert_intent(
        policy_intent(id, None, Some(3.0)),
        [0.0, 0.0, -1.0],
        "M-c yaw 3.0 钳 -1",
    );
    assert_intent(
        policy_intent(id, None, None),
        [0.0, 0.0, 0.0],
        "M-c yaw None",
    );
    // h=3.0(绕 y −3.0),yaw=−3.0 → c = wrap_pi(3−(−3)) = wrap_pi(6) = 6−2π ≈ −0.283185。
    let rot = Quat::from_rotation_y(-3.0);
    let want_c = 6.0 - std::f32::consts::TAU;
    assert_intent(
        policy_intent(rot, None, Some(-3.0)),
        [0.0, 0.0, want_c],
        "M-c wrap 差",
    );
}

/// REQ-013 C01b-3 MS7:`acc_policy_perf` 依次喂入 1500 / 2500 / 0 →
/// `(1,1500,1500)` → `(2,4000,2500)` → `(3,4000,2500)`。
#[test]
fn ms7_acc_policy_perf() {
    use super::{PolicyPerf, acc_policy_perf};

    let mut p = PolicyPerf::default();
    acc_policy_perf(&mut p, 1500);
    assert_eq!(
        p,
        PolicyPerf {
            count: 1,
            total_us: 1500,
            max_us: 1500,
        }
    );
    acc_policy_perf(&mut p, 2500);
    assert_eq!(
        p,
        PolicyPerf {
            count: 2,
            total_us: 4000,
            max_us: 2500,
        }
    );
    acc_policy_perf(&mut p, 0);
    assert_eq!(
        p,
        PolicyPerf {
            count: 3,
            total_us: 4000,
            max_us: 2500,
        }
    );
}
