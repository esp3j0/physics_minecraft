//! REQ-012 E01a §1:PolicyCodec 单测(C1–C4)。quad 身体,关节上下限和
//! 默认角从身体定义读(不硬编码)。期望值照任务书。

// 姿态字面量照抄任务书(0.70710678),不改写成常量、不降低精度。
#![allow(clippy::approx_constant, clippy::excessive_precision)]

use std::path::{Path, PathBuf};

use types::BodyDef;
use types::glam::{Quat, Vec3};

use super::PolicyCodec;
use crate::error::PolicyError;
use crate::manifest::PolicyManifest;
use crate::observe::{AgentSnapshot, observe};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("仓库根")
}

fn quad() -> BodyDef {
    let text =
        std::fs::read_to_string(repo_root().join("assets/bodies/quad.body.toml")).expect("读 quad");
    BodyDef::from_toml_str(&text).expect("quad 合法")
}

/// 从身体读各自由度默认角与限位。
fn defaults_and_limits(body: &BodyDef) -> (Vec<f32>, Vec<Option<types::JointLimits>>) {
    let joints: Vec<_> = body.dof_joints().collect();
    let defaults = joints
        .iter()
        .map(|j| j.kind.params().default_angle.unwrap_or(0.0))
        .collect();
    let limits = joints.iter().map(|j| j.kind.params().limits).collect();
    (defaults, limits)
}

fn manifest(body_hash: &str, scale: f32, terms: &[&str], extra: &str) -> PolicyManifest {
    let mut s = format!(
        "name = \"codec\"\nbody_hash = \"{body_hash}\"\ncontrol_period_ticks = 2\naction_scale = {scale}\n{extra}"
    );
    for t in terms {
        s.push_str(&format!("[[obs]]\nterm = \"{t}\"\n"));
    }
    PolicyManifest::from_toml_str(&s).expect("manifest 合法")
}

fn snap() -> AgentSnapshot {
    AgentSnapshot {
        root_rotation: Quat::from_xyzw(0.0, 0.70710678, 0.0, 0.70710678),
        root_linvel: Vec3::new(1.0, 2.0, 3.0),
        root_angvel: Vec3::new(0.1, 0.2, 0.3),
        joint_pos: vec![0.1; 12],
        joint_vel: vec![0.2; 12],
        last_action: vec![0.3; 12],
        intent: [1.0, 0.0, -1.0],
    }
}

/// C1 decode:scale=1、clip=1;model_out 全 0,只 [3]=3、[5]=−1。
/// raw[3]=1、raw[5]=−1;targets[3] 逐位 = 关节 3 上限;targets[5] = 默认角 −1
/// (误差 <1e-6、在限位内);其余 targets[i] 逐位 = 默认角;clamped=1。
#[test]
fn c1_decode() {
    let body = quad();
    let (defaults, limits) = defaults_and_limits(&body);
    let m = manifest(&body.hash().hex(), 1.0, &["intent"], "action_clip = 1.0\n");
    let codec = PolicyCodec::new(m, &body).expect("codec 合法");
    let mut model_out = vec![0.0f32; 12];
    model_out[3] = 3.0;
    model_out[5] = -1.0;
    let a = codec.decode(&model_out).expect("decode 成功");
    assert_eq!(a.raw[3], 1.0, "raw[3] 裁剪到 1");
    assert_eq!(a.raw[5], -1.0, "raw[5] = −1");
    let upper3 = limits[3].expect("关节 3 有限位").upper;
    assert_eq!(
        a.targets[3].to_bits(),
        upper3.to_bits(),
        "targets[3] 逐位 = 上限"
    );
    let expect5 = defaults[5] - 1.0;
    assert!(
        (a.targets[5] - expect5).abs() < 1e-6,
        "targets[5] = {} 期望 {expect5}",
        a.targets[5]
    );
    let l5 = limits[5].expect("关节 5 有限位");
    assert!(
        a.targets[5] >= l5.lower && a.targets[5] <= l5.upper,
        "targets[5] 在限位内"
    );
    for (i, (&t, &d)) in a.targets.iter().zip(&defaults).enumerate() {
        if i == 3 || i == 5 {
            continue;
        }
        assert_eq!(t.to_bits(), d.to_bits(), "targets[{i}] 逐位 = 默认角");
    }
    assert_eq!(a.clamped, 1, "只关节 3 越限");
}

/// C2 decode 错误:长度 11 → Err;含 NaN → NonFiniteOutput。
#[test]
fn c2_decode_errors() {
    let body = quad();
    let m = manifest(&body.hash().hex(), 1.0, &["intent"], "action_clip = 1.0\n");
    let codec = PolicyCodec::new(m, &body).expect("codec 合法");
    let err = codec.decode(&[0.0f32; 11]).expect_err("长度 11");
    assert_eq!(
        err,
        PolicyError::ActionDimMismatch {
            body: 12,
            model: 11
        }
    );
    let mut nan = vec![0.0f32; 12];
    nan[4] = f32::NAN;
    let err = codec.decode(&nan).expect_err("NaN");
    assert_eq!(err, PolicyError::NonFiniteOutput);
}

/// C3 encode 与 observe 一致(无归一化、无裁剪):逐位相同。
#[test]
fn c3_encode_matches_observe() {
    let body = quad();
    let (defaults, _limits) = defaults_and_limits(&body);
    let m = manifest(&body.hash().hex(), 0.25, &["base_ang_vel", "intent"], "");
    let codec = PolicyCodec::new(m.clone(), &body).expect("codec 合法");
    let s = snap();
    let mut got = Vec::new();
    codec.encode(&s, &mut got).expect("encode 成功");
    let mut want = Vec::new();
    observe(&m.obs, &defaults, &s, &mut want).expect("observe 成功");
    assert_eq!(got.len(), want.len(), "长度");
    for (i, (g, w)) in got.iter().zip(&want).enumerate() {
        assert_eq!(g.to_bits(), w.to_bits(), "encode[{i}] 与 observe 逐位不同");
    }
}

/// C4 encode 非有限:joint_vel[0]=NaN → NonFiniteObservation。
#[test]
fn c4_encode_non_finite() {
    let body = quad();
    let m = manifest(&body.hash().hex(), 0.25, &["joint_vel", "intent"], "");
    let codec = PolicyCodec::new(m, &body).expect("codec 合法");
    let mut s = snap();
    s.joint_vel[0] = f32::NAN;
    let mut out = Vec::new();
    let err = codec.encode(&s, &mut out).expect_err("NaN 观测");
    assert_eq!(err, PolicyError::NonFiniteObservation);
}
