//! REQ-012 C01a I 组:导入字段测试。
//! 期望值由架构师手算给出(issue #25 评论 6904557857),照抄,不改数字。
//! 浮点容差一律 1e-5。

use body_import::{ImportOptions, ImportReport, UpAxis};
use std::collections::BTreeMap;
use std::path::Path;

const TOL: f32 = 1e-5;

fn data_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
}

fn testbot_xml() -> String {
    std::fs::read_to_string(data_dir().join("testbot.urdf")).expect("read testbot.urdf")
}

fn default_opts() -> ImportOptions {
    ImportOptions {
        name: "testbot".to_string(),
        up_axis: UpAxis::Z,
        kp: 100.0,
        kd: 3.0,
        friction: 0.6,
        default_angles: BTreeMap::new(),
        cylinder_segments: 16,
    }
}

fn import(xml: &str, opts: &ImportOptions) -> (types::BodyDef, ImportReport) {
    body_import::import_urdf(xml, &data_dir(), opts).expect("import should succeed")
}

fn assert_vec3(actual: [f32; 3], expected: [f32; 3]) {
    for i in 0..3 {
        assert!(
            (actual[i] - expected[i]).abs() <= TOL,
            "vec3[{i}] actual {actual:?} vs expected {expected:?}"
        );
    }
}

#[test]
fn i1_links() {
    let (def, _) = import(&testbot_xml(), &default_opts());
    assert_eq!(def.name, "testbot");
    assert_eq!(def.links.len(), 5);
    let names: Vec<&str> = def.links.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, ["base", "hip", "thigh", "slider", "wheel"]);

    let masses = [10.0f32, 1.0, 3.0, 0.5, 0.5];
    let coms: [[f32; 3]; 5] = [
        [0.1, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, -0.2666667, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ];
    let inertias: [[f32; 3]; 5] = [
        [2.0, 3.0, 1.0],
        [0.01, 0.01, 0.01],
        [0.0476667, 0.011, 0.0476667],
        [0.005, 0.005, 0.005],
        [0.005, 0.005, 0.005],
    ];
    for i in 0..5 {
        let l = &def.links[i];
        assert!(
            (l.mass - masses[i]).abs() <= TOL,
            "link {i} mass {}",
            l.mass
        );
        assert_vec3(l.com, coms[i]);
        let inertia = l.inertia.expect("inertia should be written");
        assert_vec3(inertia, inertias[i]);
        assert!(
            (l.friction - 0.6).abs() <= TOL,
            "link {i} friction {}",
            l.friction
        );
    }
}

#[test]
fn i1_joints() {
    let (def, _) = import(&testbot_xml(), &default_opts());
    assert_eq!(def.joints.len(), 4);

    let types::JointKind::Revolute(p) = &def.joints[0].kind else {
        panic!("joint 0 should be revolute")
    };
    assert_eq!(def.joints[0].parent, "base");
    assert_eq!(def.joints[0].child, "hip");
    assert_vec3(p.anchor, [0.4, 0.5, -0.2]);
    assert_vec3(p.axis, [0.0, 0.0, -1.0]);
    assert_vec3(p.child_anchor, [0.0, 0.0, 0.0]);
    let lim = p.limits.expect("hip_joint limits");
    assert!((lim.lower - (-1.0)).abs() <= TOL && (lim.upper - 1.0).abs() <= TOL);
    assert_eq!(p.max_torque, Some(20.0));
    assert_eq!(p.max_velocity, Some(10.0));
    assert_eq!(p.kp, Some(100.0));
    assert_eq!(p.kd, Some(3.0));
    assert_eq!(p.default_angle, Some(0.0));

    let types::JointKind::Revolute(p) = &def.joints[1].kind else {
        panic!("joint 1 should be revolute")
    };
    assert_eq!(def.joints[1].parent, "hip");
    assert_eq!(def.joints[1].child, "thigh");
    assert_vec3(p.anchor, [0.0, 0.0, -1.0]);
    assert_vec3(p.axis, [-1.0, 0.0, 0.0]);
    assert_vec3(p.child_anchor, [0.0, 0.0, 0.0]);
    let lim = p.limits.expect("thigh_joint limits");
    assert!((lim.lower - (-2.0)).abs() <= TOL && (lim.upper - 2.0).abs() <= TOL);
    assert_eq!(p.max_torque, Some(30.0));
    assert_eq!(p.max_velocity, Some(15.0));
    assert_eq!(p.kp, Some(100.0));
    assert_eq!(p.kd, Some(3.0));

    let types::JointKind::Prismatic(p) = &def.joints[2].kind else {
        panic!("joint 2 should be prismatic")
    };
    assert_eq!(def.joints[2].parent, "base");
    assert_eq!(def.joints[2].child, "slider");
    assert_vec3(p.anchor, [-0.4, 0.8, 0.0]);
    assert_vec3(p.axis, [0.0, 1.0, 0.0]);
    assert_vec3(p.child_anchor, [0.0, 0.0, 0.0]);
    let lim = p.limits.expect("slide_joint limits");
    assert!((lim.lower - 0.0).abs() <= TOL && (lim.upper - 0.5).abs() <= TOL);
    assert_eq!(p.max_torque, Some(50.0));
    assert_eq!(p.max_velocity, Some(2.0));

    let types::JointKind::Revolute(p) = &def.joints[3].kind else {
        panic!("joint 3 should be revolute")
    };
    assert_eq!(def.joints[3].parent, "base");
    assert_eq!(def.joints[3].child, "wheel");
    assert_vec3(p.anchor, [0.0, 0.5, 0.4]);
    assert_vec3(p.axis, [0.0, 0.0, -1.0]);
    assert_eq!(p.limits, None);
    assert_eq!(p.max_torque, Some(5.0));
    assert_eq!(p.max_velocity, None);
}

#[test]
fn i2_up_axis_y() {
    let mut opts = default_opts();
    opts.up_axis = UpAxis::Y;
    let (def, _) = import(&testbot_xml(), &opts);

    let types::JointKind::Revolute(p) = &def.joints[0].kind else {
        panic!("joint 0 should be revolute")
    };
    assert_vec3(p.axis, [0.0, 1.0, 0.0]);
    assert_vec3(p.anchor, [0.4, 0.2, 0.5]);

    let types::JointKind::Prismatic(p) = &def.joints[2].kind else {
        panic!("joint 2 should be prismatic")
    };
    assert_vec3(p.axis, [0.0, 0.0, 1.0]);
    assert_vec3(p.anchor, [-0.4, 0.0, 0.8]);

    let types::ShapeDef::Box { half, offset } = &def.links[0].shapes[0] else {
        panic!("link 0 shape should be box")
    };
    assert_vec3(*half, [0.5, 0.3, 0.2]);
    assert_vec3(*offset, [0.0, 0.0, 0.5]);
}

#[test]
fn i3b_rpy_order() {
    let link = |name: &str| {
        format!(
            r#"<link name="{n}">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>"#,
            n = name
        )
    };
    let xml = format!(
        r#"<robot name="rpy_chain">
  {a}
  {b}
  {c}
  <joint name="j1" type="revolute">
    <parent link="a"/><child link="b"/>
    <origin xyz="0 0 0" rpy="1.5707963267948966 0 1.5707963267948966"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="5"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="b"/><child link="c"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="5"/>
  </joint>
</robot>"#,
        a = link("a"),
        b = link("b"),
        c = link("c")
    );
    let (def, _) = import(&xml, &default_opts());
    assert_eq!(def.links.len(), 3);
    assert_eq!(def.joints.len(), 2);
    let types::JointKind::Revolute(p2) = &def.joints[1].kind else {
        panic!("j2 should be revolute")
    };
    assert_vec3(p2.anchor, [1.0, 0.0, 0.0]);
}

/// I3c(返修 R1 存活 1):累计旋转的乘法顺序。I3b 里只有一个带旋转的关节,
/// `A_P · R` 与 `R · A_P` 结果相同;这条让 j1、j2 都带 rpy——j2 的累计
/// 位姿 `A_C = A_P · R(rpy_J)` 才有区分度。期望值架构师手算,照抄。
#[test]
fn i3c_two_rotated_joints() {
    let link = |name: &str| {
        format!(
            r#"<link name="{n}">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>"#,
            n = name
        )
    };
    let xml = format!(
        r#"<robot name="rot_chain">
  {a}
  {b}
  {c}
  {d}
  <joint name="j1" type="revolute">
    <parent link="a"/><child link="b"/>
    <origin xyz="0 0 0" rpy="0 0 1.5707963267948966"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="5"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="b"/><child link="c"/>
    <origin xyz="0 0 0" rpy="1.5707963267948966 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="5"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="c"/><child link="d"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="5"/>
  </joint>
</robot>"#,
        a = link("a"),
        b = link("b"),
        c = link("c"),
        d = link("d")
    );
    let (def, _) = import(&xml, &default_opts());
    assert_eq!(def.links.len(), 4);
    assert_eq!(def.joints.len(), 3);
    let types::JointKind::Revolute(p2) = &def.joints[2].kind else {
        panic!("j3 should be revolute")
    };
    assert_vec3(p2.anchor, [1.0, 0.0, 0.0]);
    assert_vec3(p2.axis, [0.0, 0.0, -1.0]);
}

#[test]
fn i4_fixed_merge() {
    let (def, report) = import(&testbot_xml(), &default_opts());

    // 连杆 0 = base(trunk 并入)
    let l0 = &def.links[0];
    assert!((l0.mass - 10.0).abs() <= TOL, "link0 mass {}", l0.mass);
    assert_vec3(l0.com, [0.1, 0.5, 0.0]);
    let types::ShapeDef::Box { offset, .. } = &l0.shapes[0] else {
        panic!("link0 shape should be box")
    };
    assert_vec3(*offset, [0.0, 0.5, 0.0]);

    // 连杆 2 = thigh(foot 并入)
    let l2 = &def.links[2];
    assert!((l2.mass - 3.0).abs() <= TOL, "link2 mass {}", l2.mass);
    assert_vec3(l2.com, [0.0, -0.2666667, 0.0]);
    let types::ShapeDef::ConvexHull { offset, .. } = &l2.shapes[0] else {
        panic!("link2 shape[0] should be convex hull")
    };
    assert_vec3(*offset, [0.0, -0.2, 0.0]);
    let types::ShapeDef::Sphere { offset, .. } = &l2.shapes[1] else {
        panic!("link2 shape[1] should be sphere")
    };
    assert_vec3(*offset, [0.0, -0.4, 0.0]);

    // 簿记
    assert_eq!(report.dropped_frames, ["imu"]);
    let mut merged: Vec<(&str, &str)> = report
        .merged_fixed
        .iter()
        .map(|(a, b)| (a.as_str(), b.as_str()))
        .collect();
    merged.sort_unstable();
    assert_eq!(merged, [("foot", "thigh"), ("trunk", "base")]);
}

#[test]
fn i5_inertia() {
    let (def, report) = import(&testbot_xml(), &default_opts());
    assert_vec3(
        def.links[0].inertia.expect("link0 inertia"),
        [2.0, 3.0, 1.0],
    );
    assert_vec3(
        def.links[2].inertia.expect("link2 inertia"),
        [0.0476667, 0.011, 0.0476667],
    );
    // 报告:base 的 offdiag_ratio = 0.5,其余四个 = 0
    assert_eq!(report.inertia.len(), 5);
    assert!(
        (report.inertia[0].offdiag_ratio - 0.5).abs() <= TOL,
        "base offdiag_ratio {}",
        report.inertia[0].offdiag_ratio
    );
    for (i, note) in report.inertia.iter().enumerate().skip(1) {
        assert!(
            note.offdiag_ratio.abs() <= TOL,
            "link {i} offdiag_ratio {}",
            note.offdiag_ratio
        );
    }
}

#[test]
fn i6_shapes() {
    let (def, report) = import(&testbot_xml(), &default_opts());

    // base:盒
    let types::ShapeDef::Box { half, offset } = &def.links[0].shapes[0] else {
        panic!("base shape should be box")
    };
    assert_vec3(*half, [0.5, 0.2, 0.3]);
    assert_vec3(*offset, [0.0, 0.5, 0.0]);

    // hip:球
    let types::ShapeDef::Sphere { radius, offset } = &def.links[1].shapes[0] else {
        panic!("hip shape should be sphere")
    };
    assert!((radius - 0.1).abs() <= TOL, "hip radius {radius}");
    assert_vec3(*offset, [0.0; 3]);

    // thigh:圆柱 → 凸包 32 点
    let types::ShapeDef::ConvexHull { points, offset } = &def.links[2].shapes[0] else {
        panic!("thigh shape[0] should be convex hull")
    };
    assert_vec3(*offset, [0.0, -0.2, 0.0]);
    assert_eq!(points.len(), 32);
    let mut pos = 0;
    let mut neg = 0;
    for p in points {
        assert!((p[0].abs() - 0.2).abs() <= TOL, "hull |x| {p:?}");
        let rr = (p[1] * p[1] + p[2] * p[2]).sqrt();
        assert!((rr - 0.05).abs() <= TOL, "hull yz radius {p:?}");
        if p[0] > 0.0 {
            pos += 1;
        } else {
            neg += 1;
        }
    }
    assert_eq!(pos, 16, "x=+0.2 应有 16 点");
    assert_eq!(neg, 16, "x=-0.2 应有 16 点");

    // foot 的球并入 thigh:shapes[1]
    let types::ShapeDef::Sphere { radius, .. } = &def.links[2].shapes[1] else {
        panic!("thigh shape[1] should be sphere")
    };
    assert!((radius - 0.05).abs() <= TOL, "foot radius {radius}");

    // slider:盒
    let types::ShapeDef::Box { half, offset } = &def.links[3].shapes[0] else {
        panic!("slider shape should be box")
    };
    assert_vec3(*half, [0.2, 0.05, 0.1]);
    assert_vec3(*offset, [0.0; 3]);

    // wheel:8 点凸包,当集合比较
    let types::ShapeDef::ConvexHull { points, offset } = &def.links[4].shapes[0] else {
        panic!("wheel shape should be convex hull")
    };
    assert_vec3(*offset, [0.0; 3]);
    assert_eq!(points.len(), 8);
    let mut expected = Vec::new();
    for sy in [-1.0f32, 1.0] {
        expected.push([0.0f32, sy * 0.1, -0.1414214]);
        expected.push([0.0, sy * 0.1, 0.1414214]);
        expected.push([0.1414214, sy * 0.1, 0.0]);
        expected.push([-0.1414214, sy * 0.1, 0.0]);
    }
    let mut used = [false; 8];
    for e in &expected {
        let mut found = None;
        for (idx, (p, u)) in points.iter().zip(used.iter()).enumerate() {
            if !*u
                && (p[0] - e[0]).abs() <= TOL
                && (p[1] - e[1]).abs() <= TOL
                && (p[2] - e[2]).abs() <= TOL
            {
                found = Some(idx);
                break;
            }
        }
        let idx = found.unwrap_or_else(|| panic!("wheel hull missing {e:?}; got {points:?}"));
        used[idx] = true;
    }

    // 报告里的形状记录
    let note = report
        .shapes
        .iter()
        .find(|n| n.link == "thigh" && n.from == "cylinder")
        .expect("thigh cylinder note");
    assert_eq!(note.to, "convex_hull");
    assert_eq!(note.hull_points, 32);
    let ratio = note.volume_out / note.volume_src;
    assert!(
        (ratio - 0.974495).abs() <= TOL,
        "thigh volume ratio {ratio}"
    );
    let wnote = report
        .shapes
        .iter()
        .find(|n| n.link == "wheel")
        .expect("wheel note");
    assert_eq!(wnote.hull_points, 8);

    // 胶囊(内联 URDF,单连杆)
    let caps_xml = r#"<robot name="caps">
  <link name="cap">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision><geometry><capsule radius="0.1" length="0.6"/></geometry></collision>
  </link>
</robot>"#;
    let (cdef, _) = import(caps_xml, &default_opts());
    assert_eq!(cdef.links.len(), 1);
    assert_eq!(cdef.joints.len(), 0);
    let types::ShapeDef::CapsuleY {
        half_height,
        radius,
        offset,
    } = &cdef.links[0].shapes[0]
    else {
        panic!("cap shape should be CapsuleY")
    };
    assert!(
        (half_height - 0.3).abs() <= TOL,
        "capsule half {half_height}"
    );
    assert!((radius - 0.1).abs() <= TOL, "capsule radius {radius}");
    assert_vec3(*offset, [0.0; 3]);
}

#[test]
fn i7_deterministic() {
    let xml = testbot_xml();
    let opts = default_opts();
    let (a, ra) = import(&xml, &opts);
    let (b, rb) = import(&xml, &opts);
    assert_eq!(a, b, "BodyDef 应相等");
    assert_eq!(ra, rb, "ImportReport 应相等");
    assert_eq!(a.hash(), b.hash(), "BodyDef::hash 应相等");
}

#[test]
fn i8_report() {
    let (_, r) = import(&testbot_xml(), &default_opts());
    assert!(
        (r.total_mass - 15.0).abs() <= TOL,
        "total_mass {}",
        r.total_mass
    );
    assert_eq!(r.links, 5);
    assert_eq!(r.dof, 4);
    assert_eq!(
        r.dof_order,
        ["hip_joint", "thigh_joint", "slide_joint", "spin_joint"]
    );
    let mut merged: Vec<(&str, &str)> = r
        .merged_fixed
        .iter()
        .map(|(a, b)| (a.as_str(), b.as_str()))
        .collect();
    merged.sort_unstable();
    assert_eq!(merged, [("foot", "thigh"), ("trunk", "base")]);
    assert_eq!(r.dropped_frames, ["imu"]);

    let ig = |name: &str| r.ignored.iter().find(|(n, _)| n == name).map(|(_, c)| *c);
    assert_eq!(ig("visual"), Some(1));
    assert_eq!(ig("dynamics"), Some(1));
    assert_eq!(ig("transmission"), Some(1));
    assert_eq!(ig("gazebo"), Some(1));
    let mat = ig("material").expect("material ignored entry");
    assert!(mat >= 1, "material count {mat}");
    for absent in ["mimic", "safety_controller", "calibration"] {
        let c = ig(absent);
        assert!(c.is_none() || c == Some(0), "{absent} = {c:?}");
    }
    assert!(
        r.notes.iter().any(|n| n.contains("不使用 com 和 inertia")),
        "notes 缺提示:{:?}",
        r.notes
    );
}

#[test]
fn i9_default_angles() {
    let mut opts = default_opts();
    opts.default_angles.insert("hip_joint".to_string(), 0.5);
    opts.default_angles.insert("thigh_joint".to_string(), 3.0);
    let (def, _) = import(&testbot_xml(), &opts);
    let da = |i: usize| def.joints[i].kind.params().default_angle;
    assert_eq!(da(0), Some(0.5));
    assert_eq!(da(1), Some(2.0), "thigh_joint 3.0 应被夹进 (-2, 2)");
    assert_eq!(da(2), Some(0.0));
    assert_eq!(da(3), Some(0.0));
}
