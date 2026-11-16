//! REQ-012 C02 P 组集成:visual 预处理经 import_urdf 的行为。
//! P1–P3 的文本扫描断言在 src/prefilter.rs 单元测试里。

use body_import::{ImportError, ImportOptions, UpAxis};
use std::collections::BTreeMap;
use std::path::Path;

fn opts() -> ImportOptions {
    ImportOptions {
        name: "p4bot".to_string(),
        up_axis: UpAxis::Z,
        kp: 100.0,
        kd: 3.0,
        friction: 0.6,
        default_angles: BTreeMap::new(),
        cylinder_segments: 16,
        max_hull_points: 32,
        package_dirs: BTreeMap::new(),
    }
}

/// P2(集成):未闭合的 `<visual>` → ImportError::Xml,文本含“visual 未闭合”。
#[test]
fn p2_unclosed_visual_is_xml_error() {
    let xml = "<robot name=\"x\"><link name=\"body\"><visual><geometry/></link></robot>";
    let e = body_import::import_urdf(xml, Path::new("."), &opts()).expect_err("应失败");
    match e {
        ImportError::Xml(msg) => assert!(msg.contains("visual 未闭合"), "msg = {msg}"),
        other => panic!("want Xml, got {other:?}"),
    }
}

/// P4:单连杆球,`<visual>` 里两个 `<material>` → 导入成功;
/// ignored visual=1、material=2;notes 含“视觉元素”。
#[test]
fn p4_duplicate_material_in_visual() {
    let xml = r#"<robot name="p4bot">
  <link name="body">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
    <visual>
      <material name="a"><color rgba="1 0 0 1"/></material>
      <material name="b"><color rgba="0 1 0 1"/></material>
      <geometry><sphere radius="0.1"/></geometry>
    </visual>
  </link>
</robot>"#;
    let (def, report) = body_import::import_urdf(xml, Path::new("."), &opts()).expect("导入应成功");
    assert_eq!(def.links.len(), 1);
    let ignored: BTreeMap<&str, usize> = report
        .ignored
        .iter()
        .map(|(k, n)| (k.as_str(), *n))
        .collect();
    assert_eq!(ignored.get("visual"), Some(&1), "ignored = {:?}", report.ignored);
    assert_eq!(ignored.get("material"), Some(&2), "ignored = {:?}", report.ignored);
    assert!(
        report.notes.iter().any(|n| n.contains("视觉元素")),
        "notes 应含视觉元素提示:{:?}",
        report.notes
    );
}