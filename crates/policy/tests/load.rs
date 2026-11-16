//! REQ-012 D01a L 组:加载校验(任务书第 2 节,AC-D1)。
//! 每条断言具体的错误变体和里面的数。期望值照抄任务书。

use std::path::{Path, PathBuf};

use policy::{Policy, PolicyError, PolicyManifest};
use types::BodyDef;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("仓库根")
}

fn body(name: &str) -> BodyDef {
    let text =
        std::fs::read_to_string(repo_root().join("assets/bodies").join(name)).expect("读身体文件");
    BodyDef::from_toml_str(&text).expect("身体合法")
}

fn quad() -> BodyDef {
    body("quad.body.toml")
}

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(repo_root().join("assets/policies/test_fixtures").join(name))
        .expect("读夹具 ONNX")
}

/// manifest 文本:extra 是 [[obs]] 之前的附加字段/表。
fn manifest_text(body_hash: &str, terms: &[&str], extra: &str) -> String {
    let mut s = format!(
        "name = \"test\"\nbody_hash = \"{body_hash}\"\ncontrol_period_ticks = 2\naction_scale = 0.25\n{extra}"
    );
    for t in terms {
        s.push_str(&format!("[[obs]]\nterm = \"{t}\"\n"));
    }
    s
}

const TERMS45: [&str; 6] = [
    "base_lin_vel",
    "base_ang_vel",
    "projected_gravity",
    "joint_pos",
    "joint_vel",
    "last_action",
];

/// 把夹具复制成 TempDir 里的 policy.onnx + policy.toml。
fn policy_dir(toml_text: &str, onnx: &[u8]) -> tempfile::TempDir {
    let dir = tempfile::TempDir::new().expect("临时目录");
    std::fs::write(dir.path().join("policy.toml"), toml_text).expect("写 toml");
    std::fs::write(dir.path().join("policy.onnx"), onnx).expect("写 onnx");
    dir
}

/// L1 正常:zero45x12 + quad + 45 维布局。
#[test]
fn l1_ok() {
    let q = quad();
    let text = manifest_text(&q.hash().hex(), &TERMS45, "");
    let dir = policy_dir(&text, &fixture("zero45x12.onnx"));
    let p = Policy::load(dir.path(), &q).expect("加载成功");
    assert_eq!(p.dof(), 12);
    assert_eq!(p.obs_dim(), 45);
}

/// L2 身体哈希:错误文本含两个哈希。
#[test]
fn l2_body_hash() {
    let q = quad();
    let text = manifest_text("0000000000000000", &TERMS45, "");
    let m = PolicyManifest::from_toml_str(&text).expect("manifest 自身合法");
    let err = Policy::from_parts(m, &fixture("zero45x12.onnx"), &q).expect_err("哈希不符");
    let got = q.hash().hex();
    assert_eq!(
        err,
        PolicyError::BodyHashMismatch {
            expected: "0000000000000000".to_string(),
            got: got.clone(),
        }
    );
    let msg = err.to_string();
    assert!(
        msg.contains("0000000000000000"),
        "错误文本含策略包哈希:{msg}"
    );
    assert!(msg.contains(&got), "错误文本含身体哈希:{msg}");
}

/// L3 观测维度:lin6x12 配 45 维布局。
#[test]
fn l3_obs_dim() {
    let q = quad();
    let text = manifest_text(&q.hash().hex(), &TERMS45, "");
    let m = PolicyManifest::from_toml_str(&text).expect("manifest 合法");
    let err = Policy::from_parts(m, &fixture("lin6x12.onnx"), &q).expect_err("维度不符");
    assert_eq!(
        err,
        PolicyError::ObsDimMismatch {
            manifest: 45,
            model: 6
        }
    );
}

/// L4 动作维度:lin6x12 + capsule → NoDof;内联小身体(dof 1)→
/// ActionDimMismatch { body: 1, model: 12 }。
#[test]
fn l4_action_dim() {
    let cap = body("capsule.body.toml");
    let text = manifest_text(&cap.hash().hex(), &TERMS45, "");
    let m = PolicyManifest::from_toml_str(&text).expect("manifest 合法");
    let err = Policy::from_parts(m, &fixture("lin6x12.onnx"), &cap).expect_err("0 自由度");
    assert_eq!(err, PolicyError::NoDof);

    // 内联小身体:两个连杆、一个转动副(dof 1),格式照 capsule/quad。
    let tiny = BodyDef::from_toml_str(
        r#"
name = "tiny"
[[links]]
name = "root"
mass = 1.0
friction = 0.6
restitution = 0.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[links]]
name = "child"
mass = 0.5
friction = 0.6
restitution = 0.0
[[links.shapes]]
kind = "sphere"
radius = 0.05
[[joints]]
parent = "root"
child = "child"
type = "revolute"
axis = [0.0, 1.0, 0.0]
anchor = [0.0, 0.0, 0.0]
limits = { lower = -1.0, upper = 1.0 }
max_torque = 10.0
kp = 100.0
kd = 3.0
default_angle = 0.0
"#,
    )
    .expect("小身体合法");
    assert_eq!(tiny.dof_count(), 1);
    let text = manifest_text(&tiny.hash().hex(), &["base_ang_vel", "intent"], "");
    let m = PolicyManifest::from_toml_str(&text).expect("manifest 合法");
    let err = Policy::from_parts(m, &fixture("lin6x12.onnx"), &tiny).expect_err("动作维度不符");
    assert_eq!(err, PolicyError::ActionDimMismatch { body: 1, model: 12 });
}

/// L5 manifest 自身校验。
#[test]
fn l5_manifest() {
    let q = quad();
    let hash = q.hash().hex();
    let onnx = fixture("zero45x12.onnx");
    let cases: Vec<(String, &str)> = vec![
        (manifest_text("XYZ", &TERMS45, ""), "body_hash 非十六进制"),
        (
            manifest_text(&hash, &TERMS45, "")
                .replace("control_period_ticks = 2", "control_period_ticks = 0"),
            "control_period_ticks = 0",
        ),
        (
            manifest_text(&hash, &TERMS45, "").replace("action_scale = 0.25", "action_scale = 0"),
            "action_scale = 0",
        ),
        (manifest_text(&hash, &[], ""), "obs 为空"),
        (manifest_text(&hash, &["height_scan"], ""), "不认识的 term"),
        (
            manifest_text(&hash, &TERMS45, "bogus_field = 1\n"),
            "未知字段",
        ),
    ];
    for (text, tag) in cases {
        let err = match PolicyManifest::from_toml_str(&text) {
            Err(e) => e,
            Ok(m) => Policy::from_parts(m, &onnx, &q).expect_err(tag),
        };
        assert!(
            matches!(err, PolicyError::Manifest(_)),
            "{tag}:实际 {err:?}"
        );
        if tag == "不认识的 term" {
            assert!(err.to_string().contains("height_scan"), "文本含名字:{err}");
        }
    }
}

/// L6 长度:default_pose 11、normalization.mean 44、std 含 0。
#[test]
fn l6_lengths() {
    let q = quad();
    let hash = q.hash().hex();
    let onnx = fixture("zero45x12.onnx");

    let pose11 = format!("[{}]", ["0.0"; 11].join(", "));
    let text = manifest_text(&hash, &TERMS45, &format!("default_pose = {pose11}\n"));
    let m = PolicyManifest::from_toml_str(&text).expect("manifest 合法");
    let err = Policy::from_parts(m, &onnx, &q).expect_err("default_pose 长度");
    assert_eq!(
        err,
        PolicyError::DefaultPoseLen {
            expected: 12,
            got: 11
        }
    );

    let mean44 = format!("[{}]", ["0.0"; 44].join(", "));
    let std45 = format!("[{}]", ["1.0"; 45].join(", "));
    let text = manifest_text(
        &hash,
        &TERMS45,
        &format!("[normalization]\nmean = {mean44}\nstd = {std45}\n"),
    );
    let m = PolicyManifest::from_toml_str(&text).expect("manifest 合法");
    let err = Policy::from_parts(m, &onnx, &q).expect_err("mean 长度");
    assert_eq!(
        err,
        PolicyError::NormalizationLen {
            expected: 45,
            got: 44
        }
    );

    let mean45 = format!("[{}]", ["0.0"; 45].join(", "));
    let mut stds = vec!["1.0".to_string(); 45];
    stds[7] = "0.0".to_string();
    let text = manifest_text(
        &hash,
        &TERMS45,
        &format!(
            "[normalization]\nmean = {mean45}\nstd = [{}]\n",
            stds.join(", ")
        ),
    );
    let err = match PolicyManifest::from_toml_str(&text) {
        Err(e) => e,
        Ok(m) => Policy::from_parts(m, &onnx, &q).expect_err("std 含 0"),
    };
    assert!(matches!(err, PolicyError::Manifest(_)), "std 含 0:{err:?}");
}

/// L7 文件:缺 policy.onnx → Io;乱字节 → Onnx。
#[test]
fn l7_files() {
    let q = quad();
    let text = manifest_text(&q.hash().hex(), &TERMS45, "");

    let dir = tempfile::TempDir::new().expect("临时目录");
    std::fs::write(dir.path().join("policy.toml"), &text).expect("写 toml");
    let err = Policy::load(dir.path(), &q).expect_err("缺 onnx");
    assert!(matches!(err, PolicyError::Io { .. }), "缺文件:{err:?}");

    let dir = policy_dir(&text, b"not an onnx file at all");
    let err = Policy::load(dir.path(), &q).expect_err("乱字节");
    assert!(matches!(err, PolicyError::Onnx(_)), "乱字节:{err:?}");
}
