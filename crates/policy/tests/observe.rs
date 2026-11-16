//! REQ-012 D01a O 组:观测函数(任务书第 2 节)。
//! 期望值由架构师手算给出(issue #25),照抄,不改数字。浮点容差 1e-5。

use policy::{AgentSnapshot, ObsEntry, ObsTerm, PolicyError, PolicyManifest, observe};
use types::glam::{Quat, Vec3};

const TOL: f32 = 1e-5;

fn entry(term: ObsTerm, scale: f32) -> ObsEntry {
    ObsEntry { term, scale }
}

fn snap(rot: Quat, linvel: Vec3, angvel: Vec3) -> AgentSnapshot {
    AgentSnapshot {
        root_rotation: rot,
        root_linvel: linvel,
        root_angvel: angvel,
        joint_pos: vec![],
        joint_vel: vec![],
        last_action: vec![],
        intent: [0.0; 3],
    }
}

fn assert_vec(got: &[f32], expected: &[f32], tag: &str) {
    assert_eq!(got.len(), expected.len(), "{tag} 长度");
    for (i, (g, e)) in got.iter().zip(expected).enumerate() {
        assert!(
            (g - e).abs() <= TOL,
            "{tag}[{i}] = {g} 期望 {e}(差 {})",
            g - e
        );
    }
}

/// O1 偏航:绕 y +90°。
#[test]
fn o1_yaw() {
    let rot = Quat::from_xyzw(0.0, 0.70710678, 0.0, 0.70710678);
    let s = snap(rot, Vec3::new(1.0, 2.0, 3.0), Vec3::new(0.1, 0.2, 0.3));
    let entries = [
        entry(ObsTerm::BaseLinVel, 1.0),
        entry(ObsTerm::BaseAngVel, 1.0),
        entry(ObsTerm::ProjectedGravity, 1.0),
    ];
    let mut out = vec![9.0; 3]; // 预填脏数据,验证先清空
    observe(&entries, &[], &s, &mut out).expect("观测成功");
    assert_vec(&out[0..3], &[-3.0, 2.0, 1.0], "O1 base_lin_vel");
    assert_vec(&out[3..6], &[-0.3, 0.2, 0.1], "O1 base_ang_vel");
    assert_vec(&out[6..9], &[0.0, -1.0, 0.0], "O1 projected_gravity");
}

/// O2 俯仰:绕 x +90°。
#[test]
fn o2_pitch() {
    let rot = Quat::from_xyzw(0.70710678, 0.0, 0.0, 0.70710678);
    let s = snap(rot, Vec3::new(1.0, 2.0, 3.0), Vec3::ZERO);
    let entries = [
        entry(ObsTerm::BaseLinVel, 1.0),
        entry(ObsTerm::ProjectedGravity, 1.0),
    ];
    let mut out = Vec::new();
    observe(&entries, &[], &s, &mut out).expect("观测成功");
    assert_vec(&out[0..3], &[1.0, 3.0, -2.0], "O2 base_lin_vel");
    assert_vec(&out[3..6], &[0.0, 0.0, 1.0], "O2 projected_gravity");
}

fn o3_snapshot() -> AgentSnapshot {
    AgentSnapshot {
        root_rotation: Quat::IDENTITY,
        root_linvel: Vec3::ZERO,
        root_angvel: Vec3::ZERO,
        joint_pos: vec![0.5, 0.2, -0.3],
        joint_vel: vec![1.0, 2.0, 3.0],
        last_action: vec![0.5, -0.5, 0.0],
        intent: [1.0, 0.0, -1.0],
    }
}

/// O3 关节项:joint_pos 减 default_pose,joint_vel 乘 scale 0.05。
#[test]
fn o3_joint_terms() {
    let entries = [
        entry(ObsTerm::JointPos, 1.0),
        entry(ObsTerm::JointVel, 0.05),
        entry(ObsTerm::LastAction, 1.0),
        entry(ObsTerm::Intent, 1.0),
    ];
    let mut out = Vec::new();
    observe(&entries, &[0.1, 0.2, 0.3], &o3_snapshot(), &mut out).expect("观测成功");
    assert_vec(
        &out,
        &[0.4, 0.0, -0.6, 0.05, 0.1, 0.15, 0.5, -0.5, 0.0, 1.0, 0.0, -1.0],
        "O3",
    );
}

/// O4 顺序:O3 的项倒过来,输出按新顺序排列。
#[test]
fn o4_order() {
    let entries = [
        entry(ObsTerm::Intent, 1.0),
        entry(ObsTerm::LastAction, 1.0),
        entry(ObsTerm::JointVel, 0.05),
        entry(ObsTerm::JointPos, 1.0),
    ];
    let mut out = Vec::new();
    observe(&entries, &[0.1, 0.2, 0.3], &o3_snapshot(), &mut out).expect("观测成功");
    assert_vec(
        &out,
        &[1.0, 0.0, -1.0, 0.5, -0.5, 0.0, 0.05, 0.1, 0.15, 0.4, 0.0, -0.6],
        "O4",
    );
}

/// O5 长度:joint_vel 长度 2、dof 3 → SnapshotDof { expected: 3, got: 2 }。
#[test]
fn o5_snapshot_len() {
    let mut s = o3_snapshot();
    s.joint_vel = vec![1.0, 2.0];
    let entries = [entry(ObsTerm::JointPos, 1.0)];
    let mut out = Vec::new();
    let err = observe(&entries, &[0.1, 0.2, 0.3], &s, &mut out).expect_err("长度不符");
    assert_eq!(
        err,
        PolicyError::SnapshotDof {
            expected: 3,
            got: 2
        }
    );
}

/// O6 维度:quad(dof 12)七个项全列 → 48。
#[test]
fn o6_obs_dim() {
    let text = r#"
name = "t"
body_hash = "0000000000000000"
control_period_ticks = 2
action_scale = 0.25
[[obs]]
term = "base_lin_vel"
[[obs]]
term = "base_ang_vel"
[[obs]]
term = "projected_gravity"
[[obs]]
term = "joint_pos"
[[obs]]
term = "joint_vel"
[[obs]]
term = "last_action"
[[obs]]
term = "intent"
"#;
    let m = PolicyManifest::from_toml_str(text).expect("manifest 合法");
    assert_eq!(m.obs_dim(12), 3 + 3 + 3 + 12 + 12 + 12 + 3);
    assert_eq!(m.obs_dim(12), 48);
}