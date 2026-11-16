//! REQ-012 C01a J 组:非法输入拒绝测试。
//! 每条一个最小 URDF(字符串内联),断言具体错误变体与其中的名字。

use body_import::{ImportError, ImportOptions, UpAxis};
use std::collections::BTreeMap;
use std::path::Path;

fn opts() -> ImportOptions {
    ImportOptions {
        name: "t".to_string(),
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

/// 合法连杆:质量 1、对角 0.01 惯量、半径 0.1 球。
fn link(name: &str) -> String {
    format!(
        r#"<link name="{n}">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>"#,
        n = name
    )
}

/// 最小合法机器人,连杆与关节由调用方拼。
fn robot(links: &[String], joints: &str) -> String {
    format!(
        r#"<robot name="t">
  {}
  {joints}
</robot>"#,
        links.join("\n  "),
        joints = joints
    )
}

fn joint(name: &str, ty: &str, parent: &str, child: &str, extra: &str) -> String {
    format!(
        r#"<joint name="{n}" type="{t}">
    <parent link="{p}"/><child link="{c}"/>
    {extra}
  </joint>"#,
        n = name,
        t = ty,
        p = parent,
        c = child,
        extra = extra
    )
}

fn revolute_extra(lower: &str, upper: &str) -> String {
    format!(
        r#"<axis xyz="1 0 0"/>
    <limit lower="{l}" upper="{u}" effort="10" velocity="5"/>"#,
        l = lower,
        u = upper
    )
}

fn err(xml: &str) -> ImportError {
    let o = opts();
    body_import::import_urdf(xml, Path::new("."), &o).expect_err("import should fail")
}

#[test]
fn j1_cycle() {
    let xml = robot(
        &[link("a"), link("b"), link("c")],
        &(joint("j1", "revolute", "a", "c", &revolute_extra("-1", "1")).to_string()
            + &joint("j2", "revolute", "b", "c", &revolute_extra("-1", "1"))),
    );
    match err(&xml) {
        ImportError::Cycle(list) => assert!(list.contains(&"c".to_string()), "list {list:?}"),
        other => panic!("want Cycle, got {other:?}"),
    }
}

#[test]
fn j2_multiple_roots() {
    let xml = robot(
        &[link("a"), link("b"), link("c")],
        &joint("j1", "revolute", "a", "b", &revolute_extra("-1", "1")),
    );
    match err(&xml) {
        ImportError::MultipleRoots(list) => {
            assert!(list.contains(&"a".to_string()), "list {list:?}");
            assert!(list.contains(&"c".to_string()), "list {list:?}");
        }
        other => panic!("want MultipleRoots, got {other:?}"),
    }
}

#[test]
fn j3_unknown_link() {
    let xml = robot(
        &[link("a")],
        &joint("j1", "revolute", "a", "ghost", &revolute_extra("-1", "1")),
    );
    match err(&xml) {
        ImportError::UnknownLink { joint, link } => {
            assert_eq!(joint, "j1");
            assert_eq!(link, "ghost");
        }
        other => panic!("want UnknownLink, got {other:?}"),
    }
}

#[test]
fn j4_floating() {
    let xml = robot(
        &[link("a"), link("b")],
        &joint("j1", "floating", "a", "b", ""),
    );
    match err(&xml) {
        ImportError::UnsupportedJoint { joint, kind } => {
            assert_eq!(joint, "j1");
            assert!(kind.to_lowercase().contains("floating"), "kind {kind:?}");
        }
        other => panic!("want UnsupportedJoint, got {other:?}"),
    }
}

#[test]
fn j4_unknown_type() {
    let xml = robot(
        &[link("a"), link("b")],
        &joint("j1", "wobble", "a", "b", ""),
    );
    let e = err(&xml);
    let s = e.to_string();
    assert!(
        matches!(
            e,
            ImportError::UnsupportedJoint { .. } | ImportError::Xml(_)
        ),
        "want UnsupportedJoint or Xml, got {e:?}"
    );
    assert!(s.contains("wobble"), "message {s:?}");
}

#[test]
fn j5_missing_limit() {
    // revolute 无 <limit>:urdf-rs 把缺省 lower/upper 填 0.0,库按 lower==0
    // && upper==0 判"没写"。
    let xml = robot(
        &[link("a"), link("b")],
        &joint("j1", "revolute", "a", "b", r#"<axis xyz="1 0 0"/>"#),
    );
    match err(&xml) {
        ImportError::MissingLimit { joint } => assert_eq!(joint, "j1"),
        other => panic!("want MissingLimit, got {other:?}"),
    }
}

#[test]
fn j6_bad_axis() {
    let xml = robot(
        &[link("a"), link("b")],
        &joint(
            "j1",
            "revolute",
            "a",
            "b",
            r#"<axis xyz="0 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="5"/>"#,
        ),
    );
    match err(&xml) {
        ImportError::BadAxis { joint } => assert_eq!(joint, "j1"),
        other => panic!("want BadAxis, got {other:?}"),
    }
}

#[test]
fn j7_missing_inertial() {
    let xml = robot(
        &[
            link("a"),
            r#"<link name="b">
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>"#
                .to_string(),
        ],
        &joint("j1", "revolute", "a", "b", &revolute_extra("-1", "1")),
    );
    match err(&xml) {
        ImportError::MissingInertial { link } => assert_eq!(link, "b"),
        other => panic!("want MissingInertial, got {other:?}"),
    }
}

#[test]
fn j7_zero_mass() {
    let xml = robot(
        &[link("a"), r#"<link name="b">
    <inertial><mass value="0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>"#.to_string()],
        &joint("j1", "revolute", "a", "b", &revolute_extra("-1", "1")),
    );
    match err(&xml) {
        ImportError::BadMass { link, mass } => {
            assert_eq!(link, "b");
            assert_eq!(mass, 0.0);
        }
        other => panic!("want BadMass, got {other:?}"),
    }
}

#[test]
fn j8_nan_inertia() {
    let xml = robot(
        &[link("a"), r#"<link name="b">
    <inertial><mass value="1"/><inertia ixx="NaN" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>"#.to_string()],
        &joint("j1", "revolute", "a", "b", &revolute_extra("-1", "1")),
    );
    match err(&xml) {
        ImportError::BadInertia { link } => assert_eq!(link, "b"),
        ImportError::Xml(_) => {} // urdf-rs 解析层拒绝也接受(进度说明里报告)
        other => panic!("want BadInertia or Xml, got {other:?}"),
    }
}

#[test]
fn j8_negative_diag() {
    let xml = robot(
        &[link("a"), r#"<link name="b">
    <inertial><mass value="1"/><inertia ixx="-0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>"#.to_string()],
        &joint("j1", "revolute", "a", "b", &revolute_extra("-1", "1")),
    );
    match err(&xml) {
        ImportError::BadInertia { link } => assert_eq!(link, "b"),
        other => panic!("want BadInertia, got {other:?}"),
    }
}

#[test]
fn j9_no_collision() {
    let xml = robot(
        &[link("a"), r#"<link name="b">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>"#.to_string()],
        &joint("j1", "revolute", "a", "b", &revolute_extra("-1", "1")),
    );
    match err(&xml) {
        ImportError::NoCollision { link } => assert_eq!(link, "b"),
        other => panic!("want NoCollision, got {other:?}"),
    }
}

#[test]
fn j10_mesh() {
    let xml = robot(
        &[link("a"), r#"<link name="b">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><geometry><mesh filename="package://x/leg.stl"/></geometry></collision>
  </link>"#.to_string()],
        &joint("j1", "revolute", "a", "b", &revolute_extra("-1", "1")),
    );
    match err(&xml) {
        ImportError::MeshFileMissing { link, file, .. } => {
            assert_eq!(link, "b");
            assert!(file.contains("leg.stl"), "file {file:?}");
        }
        other => panic!("want MeshFileMissing, got {other:?}"),
    }
}

#[test]
fn j11_too_many_shapes() {
    // 根 1 球 + 8 个 fixed 子连杆各 1 球 → 合并后 9 形状。
    let mut joints = String::new();
    let mut links = vec![link("root")];
    for i in 0..8 {
        links.push(link(&format!("w{i}")));
        joints.push_str(&format!(
            r#"<joint name="f{i}" type="fixed">
    <parent link="root"/><child link="w{i}"/>
  </joint>
  "#
        ));
    }
    let xml = robot(&links, joints.trim_end());
    match err(&xml) {
        ImportError::TooManyShapes { link, count } => {
            assert_eq!(link, "root");
            assert_eq!(count, 9);
        }
        other => panic!("want TooManyShapes, got {other:?}"),
    }
}

#[test]
fn j12_bad_pose() {
    let base = robot(
        &[link("a"), link("b")],
        &joint("j1", "revolute", "a", "b", &revolute_extra("-1", "1")),
    );
    // 场景 1:URDF 没有的关节名。
    let mut o = opts();
    o.default_angles.insert("nope".to_string(), 0.1);
    let e = body_import::import_urdf(&base, Path::new("."), &o).expect_err("want BadOption");
    assert!(matches!(e, ImportError::BadOption(_)), "got {e:?}");

    // 场景 2:指向 fixed 关节。
    let xml2 = robot(&[link("a"), link("b")], &joint("fj", "fixed", "a", "b", ""));
    let mut o2 = opts();
    o2.default_angles.insert("fj".to_string(), 0.1);
    let e2 = body_import::import_urdf(&xml2, Path::new("."), &o2).expect_err("want BadOption");
    assert!(matches!(e2, ImportError::BadOption(_)), "got {e2:?}");
}

#[test]
fn j13_bad_xml() {
    let e = err("<robot");
    assert!(matches!(e, ImportError::Xml(_)), "got {e:?}");
}

#[test]
fn j14_no_links() {
    let e = err(r#"<robot name="x"/>"#);
    assert!(matches!(e, ImportError::NoLinks), "got {e:?}");
}
