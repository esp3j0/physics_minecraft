//! REQ-012 E02c-1 §1:机身系纯函数与批量编解码的 Rust 测试。
//! R1 body_frame 与 AgentEnv::step 的 RewardInputs 三段逐位相同;
//! R2 绕轴 +90° 的手算期望;批量辅助与逐条调 PolicyCodec 逐位相同。

use std::f32::consts::FRAC_PI_2;
use std::path::{Path, PathBuf};

use agent_env::{
    AgentEnv, EnvConfig, body_frame, body_frame_batch, decode_actions, encode_snapshots,
};
use policy::{AgentSnapshot, PolicyCodec, PolicyManifest};
use types::BodyDef;
use types::glam::{Quat, Vec3};

const DOF: usize = 12;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("仓库根")
}

fn quad_text() -> String {
    std::fs::read_to_string(repo_root().join("assets/bodies/quad.body.toml")).expect("读 quad")
}

fn quad_def() -> BodyDef {
    BodyDef::from_toml_str(&quad_text()).expect("quad 合法")
}

fn manifest_terms(scale: f32, terms: &[&str]) -> PolicyManifest {
    let hash = quad_def().hash().hex();
    let mut s = format!(
        "name = \"e02c\"\nbody_hash = \"{hash}\"\ncontrol_period_ticks = 2\naction_scale = {scale}\naction_clip = 1.0\n"
    );
    for t in terms {
        s.push_str(&format!("[[obs]]\nterm = \"{t}\"\n"));
    }
    PolicyManifest::from_toml_str(&s).expect("manifest 合法")
}

fn action_seq(k: usize) -> Vec<f32> {
    (0..DOF)
        .map(|i| 0.5 * ((0.1 * k as f32) + i as f32).sin())
        .collect()
}

fn bits(v: &[f32]) -> Vec<u32> {
    v.iter().map(|x| x.to_bits()).collect()
}

/// R1:走 E1 固定动作 20 步,每步 body_frame(根状态) 与 step 的 reward 三段逐位相同。
#[test]
fn r1_body_frame_matches_step() {
    let m = manifest_terms(0.25, &["base_lin_vel", "base_ang_vel", "projected_gravity"]);
    let mut env = AgentEnv::new(EnvConfig {
        body_toml: quad_text(),
        manifest: m,
        episode_controls: 200,
        settle_ticks: 120,
        fall_up_y_min: 0.5,
        intent_range: [[0.0, 0.0]; 3],
    })
    .expect("环境");
    env.reset(7).expect("reset");
    for k in 0..20 {
        let out = env.step(&action_seq(k)).expect("step");
        let root = env.link_states()[0];
        let bf = body_frame(root.rotation, root.linvel, root.angvel);
        assert_eq!(
            bits(&bf.linvel),
            bits(&out.reward.base_linvel_body),
            "第 {k} 步 linvel"
        );
        assert_eq!(
            bits(&bf.angvel),
            bits(&out.reward.base_angvel_body),
            "第 {k} 步 angvel"
        );
        assert_eq!(
            bits(&bf.projected_gravity),
            bits(&out.reward.projected_gravity),
            "第 {k} 步 projected_gravity"
        );
    }
}

/// R2:绕 y +90° 世界线速度 (1,0,0) → linvel (0,0,1);绕 x +90° →
/// projected_gravity (0,0,1)、up_y 0(误差 < 1e-6)。
#[test]
fn r2_axis_rotations() {
    let bf = body_frame(
        Quat::from_rotation_y(FRAC_PI_2),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::ZERO,
    );
    for (got, want) in bf.linvel.iter().zip([0.0f32, 0.0, 1.0]) {
        assert!((got - want).abs() < 1e-6, "绕 y linvel {got} ≠ {want}");
    }
    let bf = body_frame(Quat::from_rotation_x(FRAC_PI_2), Vec3::ZERO, Vec3::ZERO);
    for (got, want) in bf.projected_gravity.iter().zip([0.0f32, 0.0, 1.0]) {
        assert!((got - want).abs() < 1e-6, "绕 x gravity {got} ≠ {want}");
    }
    assert!(bf.up_y.abs() < 1e-6, "绕 x up_y {} ≠ 0", bf.up_y);
}

fn sample_snapshots() -> Vec<AgentSnapshot> {
    (0..5)
        .map(|i| {
            let f = i as f32;
            AgentSnapshot {
                root_rotation: Quat::from_rotation_y(0.3 * f) * Quat::from_rotation_x(0.2 * f),
                root_linvel: Vec3::new(0.1 * f, 0.2 - 0.05 * f, -0.3 * f),
                root_angvel: Vec3::new(0.4 * f, -0.1 * f, 0.2 * f),
                joint_pos: (0..DOF).map(|j| 0.01 * (j as f32 + f)).collect(),
                joint_vel: (0..DOF).map(|j| 0.1 * (j as f32 - f)).collect(),
                last_action: (0..DOF).map(|j| 0.05 * (j as f32 + 2.0 * f)).collect(),
                intent: [0.1 * f, -0.2 * f, 0.3 * f],
            }
        })
        .collect()
}

/// 批量编码/解码/机身系与逐条调 PolicyCodec / body_frame 逐位相同。
#[test]
fn batch_matches_per_row() {
    let m = manifest_terms(
        0.25,
        &[
            "base_lin_vel",
            "base_ang_vel",
            "projected_gravity",
            "joint_pos",
            "joint_vel",
            "last_action",
            "intent",
        ],
    );
    let codec = PolicyCodec::new(m, &quad_def()).expect("codec");
    let snaps = sample_snapshots();
    let n = snaps.len();

    let batch = encode_snapshots(&codec, &snaps).expect("批量编码");
    let mut per = Vec::new();
    for s in &snaps {
        let mut row = Vec::new();
        codec.encode(s, &mut row).expect("逐条编码");
        per.extend_from_slice(&row);
    }
    assert_eq!(bits(&batch), bits(&per), "encode_batch 逐位");

    let actions: Vec<f32> = snaps.iter().flat_map(|s| s.last_action.clone()).collect();
    let db = decode_actions(&codec, &actions, n).expect("批量解码");
    let mut raw = Vec::new();
    let mut targets = Vec::new();
    let mut clamped = Vec::new();
    for row in actions.chunks(DOF) {
        let a = codec.decode(row).expect("逐条解码");
        raw.extend_from_slice(&a.raw);
        targets.extend_from_slice(&a.targets);
        clamped.push(a.clamped as u32);
    }
    assert_eq!(bits(&db.raw), bits(&raw), "decode raw 逐位");
    assert_eq!(bits(&db.targets), bits(&targets), "decode targets 逐位");
    assert_eq!(db.clamped, clamped, "decode clamped");

    let mut q = Vec::new();
    let mut lv = Vec::new();
    let mut av = Vec::new();
    for s in &snaps {
        let r = s.root_rotation;
        q.extend_from_slice(&[r.x, r.y, r.z, r.w]);
        lv.extend_from_slice(&[s.root_linvel.x, s.root_linvel.y, s.root_linvel.z]);
        av.extend_from_slice(&[s.root_angvel.x, s.root_angvel.y, s.root_angvel.z]);
    }
    let bf = body_frame_batch(&q, &lv, &av, n);
    let mut bl = Vec::new();
    let mut ba = Vec::new();
    let mut pg = Vec::new();
    let mut up = Vec::new();
    for s in &snaps {
        let b = body_frame(s.root_rotation, s.root_linvel, s.root_angvel);
        bl.extend_from_slice(&b.linvel);
        ba.extend_from_slice(&b.angvel);
        pg.extend_from_slice(&b.projected_gravity);
        up.push(b.up_y);
    }
    assert_eq!(bits(&bf.linvel), bits(&bl), "body_frame linvel 逐位");
    assert_eq!(bits(&bf.angvel), bits(&ba), "body_frame angvel 逐位");
    assert_eq!(
        bits(&bf.projected_gravity),
        bits(&pg),
        "body_frame gravity 逐位"
    );
    assert_eq!(bits(&bf.up_y), bits(&up), "body_frame up_y 逐位");
}
