//! REQ-012 C01a R 组:往返(emit → parse)与输出文本确定性。

use body_import::{ImportOptions, UpAxis, body_to_toml, import_urdf, report_to_markdown};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

fn data_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
}

fn testbot() -> (types::BodyDef, body_import::ImportReport) {
    let xml = std::fs::read_to_string(data_dir().join("testbot.urdf")).expect("read testbot.urdf");
    let opts = ImportOptions {
        name: "testbot".to_string(),
        up_axis: UpAxis::Z,
        kp: 100.0,
        kd: 3.0,
        friction: 0.6,
        default_angles: BTreeMap::new(),
        cylinder_segments: 16,
    };
    import_urdf(&xml, &data_dir(), &opts).expect("import testbot")
}

/// quad/capsule 资产往返:from_toml → body_to_toml → from_toml,
/// 两个 BodyDef 相等且 hash 相等。
fn assert_asset_roundtrip(file: &str) {
    let path = repo_root().join("assets").join("bodies").join(file);
    let text = std::fs::read_to_string(&path).expect("read asset toml");
    let def = types::BodyDef::from_toml_str(&text).expect("parse asset toml");
    let text2 = body_to_toml(&def);
    let def2 = types::BodyDef::from_toml_str(&text2).expect("re-parse emitted toml");
    assert_eq!(def, def2, "{file} 往返 BodyDef 相等");
    assert_eq!(def.hash(), def2.hash(), "{file} 往返 hash 相等");
}

#[test]
fn r1_quad() {
    assert_asset_roundtrip("quad.body.toml");
}

#[test]
fn r1_capsule() {
    assert_asset_roundtrip("capsule.body.toml");
}

#[test]
fn r2_testbot() {
    let (def, _) = testbot();
    let text = body_to_toml(&def);
    let def2 = types::BodyDef::from_toml_str(&text).expect("re-parse testbot toml");
    assert_eq!(def, def2);
    assert_eq!(def.hash(), def2.hash());
}

#[test]
fn r3_deterministic_text() {
    let (a, _) = testbot();
    let (b, _) = testbot();
    let ta = body_to_toml(&a);
    let tb = body_to_toml(&b);
    assert_eq!(ta, tb, "两次导入的 toml 文本逐字节相同");
}

#[test]
fn r4_report_markdown() {
    let (_, rep) = testbot();
    let md = report_to_markdown(&rep);
    assert!(md.contains("15"), "总质量 15 应出现");
    for j in ["hip_joint", "thigh_joint", "slide_joint", "spin_joint"] {
        assert!(md.contains(j), "关节名 {j} 应出现在 markdown");
    }
    assert!(md.contains("imu"), "imu 应出现在丢弃清单");
    assert!(md.contains("0.974"), "圆柱体积比 0.974... 应出现");
    assert!(md.contains("不使用 com 和 inertia"), "内核提示应出现");
}
