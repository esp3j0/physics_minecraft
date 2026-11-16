//! REQ-012 D01a A 组:推理(任务书第 2 节)。
//! 期望值由架构师手算给出(issue #25),照抄,不改数字。浮点容差 1e-5。

// 期望值/姿态字面量照抄任务书(0.70710678、−0.853596389、1.70719278 等),
// 不改写成常量、不降低精度。
#![allow(clippy::approx_constant, clippy::excessive_precision)]

use std::path::{Path, PathBuf};

use policy::{AgentSnapshot, Policy, PolicyError, PolicyManifest};
use types::BodyDef;
use types::glam::{Quat, Vec3};

const TOL: f32 = 1e-5;

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

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(repo_root().join("assets/policies/test_fixtures").join(name))
        .expect("读夹具 ONNX")
}

fn manifest_text(body_hash: &str, scale: f32, terms: &[&str], extra: &str) -> String {
    let mut s = format!(
        "name = \"test\"\nbody_hash = \"{body_hash}\"\ncontrol_period_ticks = 2\naction_scale = {scale}\n{extra}"
    );
    for t in terms {
        s.push_str(&format!("[[obs]]\nterm = \"{t}\"\n"));
    }
    s
}

fn policy(model: &str, scale: f32, terms: &[&str], extra: &str) -> Policy {
    let q = quad();
    let text = manifest_text(&q.hash().hex(), scale, terms, extra);
    let m = PolicyManifest::from_toml_str(&text).expect("manifest 合法");
    Policy::from_parts(m, &fixture(model), &q).expect("加载成功")
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

/// A2 的 snapshot:O1 的姿态和角速度,intent [1, 0, −1]。
fn snap6() -> AgentSnapshot {
    AgentSnapshot {
        root_rotation: Quat::from_xyzw(0.0, 0.70710678, 0.0, 0.70710678),
        root_linvel: Vec3::new(1.0, 2.0, 3.0),
        root_angvel: Vec3::new(0.1, 0.2, 0.3),
        joint_pos: vec![0.0; 12],
        joint_vel: vec![0.0; 12],
        last_action: vec![0.0; 12],
        intent: [1.0, 0.0, -1.0],
    }
}

const TERMS6: [&str; 2] = ["base_ang_vel", "intent"];
const TERMS45: [&str; 6] = [
    "base_lin_vel",
    "base_ang_vel",
    "projected_gravity",
    "joint_pos",
    "joint_vel",
    "last_action",
];
const TERMS48: [&str; 7] = [
    "base_lin_vel",
    "base_ang_vel",
    "projected_gravity",
    "joint_pos",
    "joint_vel",
    "last_action",
    "intent",
];

/// quad 的 12 个默认角(任务书第 2 节:各腿 0、−0.853596389、1.70719278)。
const QUAD_DEFAULTS: [f32; 12] = [
    0.0,
    -0.853596389,
    1.70719278,
    0.0,
    -0.853596389,
    1.70719278,
    0.0,
    -0.853596389,
    1.70719278,
    0.0,
    -0.853596389,
    1.70719278,
];

/// A1 零模型:raw 全 0,targets = 默认角,clamped = 0。
#[test]
fn a1_zero_model() {
    let mut p = policy("zero45x12.onnx", 0.25, &TERMS45, "");
    let s = AgentSnapshot {
        root_rotation: Quat::IDENTITY,
        root_linvel: Vec3::new(1.0, 2.0, 3.0),
        root_angvel: Vec3::new(0.1, 0.2, 0.3),
        joint_pos: QUAD_DEFAULTS.to_vec(),
        joint_vel: vec![0.0; 12],
        last_action: vec![0.0; 12],
        intent: [0.0; 3],
    };
    let a = p.act(&s).expect("推理成功");
    assert_vec(&a.raw, &[0.0; 12], "A1 raw");
    assert_vec(&a.targets, &QUAD_DEFAULTS, "A1 targets");
    assert_eq!(a.clamped, 0);
}

/// A2 线性模型:action_clip 夹最后一项。
#[test]
fn a2_linear_model() {
    let mut p = policy("lin6x12.onnx", 0.25, &TERMS6, "action_clip = 1.0\n");
    let a = p.act(&snap6()).expect("推理成功");
    assert_vec(
        &a.raw,
        &[
            -0.03, 0.04, 0.03, 0.4, 0.0, -0.6, -0.21, 0.16, 0.09, 1.0, 0.0, -1.0,
        ],
        "A2 raw",
    );
    assert_vec(
        &a.targets,
        &[
            -0.0075,
            -0.843596389,
            1.71469278,
            0.1,
            -0.853596389,
            1.55719278,
            -0.0525,
            -0.813596389,
            1.72969278,
            0.25,
            -0.853596389,
            1.45719278,
        ],
        "A2 targets",
    );
    assert_eq!(a.clamped, 0);
}

/// A3 限位:action_scale 2.0 → targets[9] = 0.8、targets[11] = 0.0、
/// targets[3] = 0.8(边界不算被夹),clamped = 2。
#[test]
fn a3_joint_limits() {
    let mut p = policy("lin6x12.onnx", 2.0, &TERMS6, "action_clip = 1.0\n");
    let a = p.act(&snap6()).expect("推理成功");
    assert!(
        (a.targets[9] - 0.8).abs() <= TOL,
        "targets[9] = {}",
        a.targets[9]
    );
    assert!(
        (a.targets[11] - 0.0).abs() <= TOL,
        "targets[11] = {}",
        a.targets[11]
    );
    assert!(
        (a.targets[3] - 0.8).abs() <= TOL,
        "targets[3] = {}",
        a.targets[3]
    );
    assert_eq!(a.clamped, 2);
}

/// A4 归一化:mean/std 后观测 (0,0,0,0.5,0,−0.5)。
#[test]
fn a4_normalization() {
    let extra = "[normalization]\nmean = [-0.3, 0.2, 0.1, 0.0, 0.0, 0.0]\nstd = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0]\n";
    let mut p = policy("lin6x12.onnx", 0.25, &TERMS6, extra);
    let a = p.act(&snap6()).expect("推理成功");
    assert_vec(
        &a.raw,
        &[0.0, 0.0, 0.0, 0.2, 0.0, -0.3, 0.0, 0.0, 0.0, 0.5, 0.0, -0.6],
        "A4 raw",
    );
}

/// A5 观测裁剪:obs_clip 0.5 → 观测 (−0.3,0.2,0.1,0.5,0,−0.5)。
#[test]
fn a5_obs_clip() {
    let mut p = policy("lin6x12.onnx", 0.25, &TERMS6, "obs_clip = 0.5\n");
    let a = p.act(&snap6()).expect("推理成功");
    assert!((a.raw[3] - 0.2).abs() <= TOL, "raw[3] = {}", a.raw[3]);
    assert!((a.raw[9] - 0.5).abs() <= TOL, "raw[9] = {}", a.raw[9]);
    assert!((a.raw[11] - (-0.6)).abs() <= TOL, "raw[11] = {}", a.raw[11]);
}

/// A10 裁剪顺序(变异 11 的补强,任务书第 3 节授权):同时带 normalization
/// 和 obs_clip,裁剪必须在归一化之后。推导(手算):
/// A2 观测 (−0.3, 0.2, 0.1, 1, 0, −1);mean [−0.3, 0.2, 0.1, 0, 0, 0]、
/// std [1, 1, 1, 2, 2, 2] → 归一化后 (0, 0, 0, 0.5, 0, −0.5);
/// obs_clip 0.25 → (0, 0, 0, 0.25, 0, −0.25);
/// action[i] = 0.1×(i+1)×obs[i mod 6] → raw[3] = 0.4×0.25 = 0.1、
/// raw[5] = 0.6×(−0.25) = −0.15、raw[9] = 1.0×0.25 = 0.25、
/// raw[11] = 1.2×(−0.25) = −0.3,其余 0。
/// 若裁剪在归一化之前:obs[0] 先夹成 −0.25,归一化后 (−0.25+0.3)/1 = 0.05,
/// raw[0] = 0.1×0.05 = 0.005 ≠ 0,可区分。
#[test]
fn a10_clip_after_normalization() {
    let extra = "obs_clip = 0.25\n[normalization]\nmean = [-0.3, 0.2, 0.1, 0.0, 0.0, 0.0]\nstd = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0]\n";
    let mut p = policy("lin6x12.onnx", 0.25, &TERMS6, extra);
    let a = p.act(&snap6()).expect("推理成功");
    assert_vec(
        &a.raw,
        &[
            0.0, 0.0, 0.0, 0.1, 0.0, -0.15, 0.0, 0.0, 0.0, 0.25, 0.0, -0.3,
        ],
        "A10 raw",
    );
}

/// A6 NaN 输出:nan6x12 → NonFiniteOutput。
#[test]
fn a6_nan_output() {
    let mut p = policy("nan6x12.onnx", 0.25, &TERMS6, "");
    let err = p.act(&snap6()).expect_err("NaN 输出");
    assert_eq!(err, PolicyError::NonFiniteOutput);
}

/// A7 NaN 输入:root_angvel 含 NaN → NonFiniteObservation。
#[test]
fn a7_nan_input() {
    let mut p = policy("lin6x12.onnx", 0.25, &TERMS6, "action_clip = 1.0\n");
    let mut s = snap6();
    s.root_angvel = Vec3::new(f32::NAN, 0.2, 0.3);
    let err = p.act(&s).expect_err("NaN 观测");
    assert_eq!(err, PolicyError::NonFiniteObservation);
}

fn snap48() -> AgentSnapshot {
    AgentSnapshot {
        root_rotation: Quat::IDENTITY,
        root_linvel: Vec3::new(0.1, 0.2, 0.3),
        root_angvel: Vec3::new(0.01, 0.02, 0.03),
        joint_pos: QUAD_DEFAULTS.to_vec(),
        joint_vel: vec![0.1; 12],
        last_action: vec![0.0; 12],
        intent: [0.5, 0.0, 0.1],
    }
}

fn bits(v: &[f32]) -> Vec<u32> {
    v.iter().map(|x| x.to_bits()).collect()
}

/// A8 确定性:mlp48x12,同 snapshot 100 次逐位相同;第二个 Policy 也相同。
#[test]
fn a8_deterministic() {
    let mut p = policy("mlp48x12.onnx", 0.25, &TERMS48, "");
    let s = snap48();
    let first = p.act(&s).expect("推理成功");
    let reference = bits(&first.raw);
    for i in 0..100 {
        let a = p.act(&s).expect("推理成功");
        assert_eq!(bits(&a.raw), reference, "第 {i} 次 raw 逐位不同");
    }
    let mut p2 = policy("mlp48x12.onnx", 0.25, &TERMS48, "");
    let a2 = p2.act(&s).expect("第二个 Policy 推理成功");
    assert_eq!(bits(&a2.raw), reference, "第二个 Policy raw 逐位不同");
}

/// A9 推理开销(AC-D4):热身 1000 次,计时 10000 次,P95 ≤ 1 ms。
/// 局部豁免墙钟:只用于这条性能测试的计时,不进生产路径。
#[test]
#[allow(clippy::disallowed_methods)]
fn a9_inference_latency() {
    use std::time::Instant;

    let mut p = policy("mlp48x12.onnx", 0.25, &TERMS48, "");
    let s = snap48();
    for _ in 0..1000 {
        p.act(&s).expect("热身推理成功");
    }
    let mut us: Vec<f64> = Vec::with_capacity(10000);
    for _ in 0..10000 {
        let t0 = Instant::now();
        p.act(&s).expect("推理成功");
        us.push(t0.elapsed().as_secs_f64() * 1e6);
    }
    us.sort_by(f64::total_cmp);
    let p50 = us[5000];
    let p95 = us[9500];
    let p99 = us[9900];
    let max = us[9999];
    println!("a9: P50 {p50:.1} µs, P95 {p95:.1} µs, P99 {p99:.1} µs, max {max:.1} µs");
    assert!(p95 <= 1000.0, "P95 = {p95:.1} µs > 1 ms");
}
