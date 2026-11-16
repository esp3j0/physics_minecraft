//! REQ-012 C02 G 组:Unitree Go2 导入(读 assets/third_party 的 URDF)。
//! 期望值由架构师手算给出(issue #25 C02 任务书),照抄,不改数字。

use body_import::{ImportOptions, UpAxis};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

const TOL: f32 = 1e-5;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .canonicalize()
        .expect("仓库根")
}

fn go2_urdf() -> String {
    std::fs::read_to_string(
        repo_root()
            .join("assets")
            .join("third_party")
            .join("unitree_go2")
            .join("go2_description.urdf"),
    )
    .expect("读 Go2 URDF")
}

/// 与 1.2 命令相同的选项:髋 0、大腿 0.8、小腿 −1.5。
fn go2_opts() -> ImportOptions {
    let mut default_angles = BTreeMap::new();
    for leg in ["FL", "FR", "RL", "RR"] {
        default_angles.insert(format!("{leg}_thigh_joint"), 0.8f32);
        default_angles.insert(format!("{leg}_calf_joint"), -1.5f32);
    }
    ImportOptions {
        name: "go2".to_string(),
        up_axis: UpAxis::Z,
        kp: 100.0,
        kd: 3.0,
        friction: 0.6,
        default_angles,
        cylinder_segments: 16,
        max_hull_points: 32,
        package_dirs: BTreeMap::new(),
    }
}

fn import_go2() -> (types::BodyDef, body_import::ImportReport) {
    body_import::import_urdf(&go2_urdf(), &repo_root(), &go2_opts()).expect("Go2 导入应成功")
}

/// G1:13 连杆、12 自由度、dof 序、总质量、丢弃帧、合并固定关节、无网格。
#[test]
fn g1_structure() {
    let (def, report) = import_go2();
    assert_eq!(report.links, 13);
    assert_eq!(def.links.len(), 13);
    assert_eq!(report.dof, 12);
    assert_eq!(def.joints.len(), 12);
    let expected_order: Vec<String> = ["FL", "FR", "RL", "RR"]
        .iter()
        .flat_map(|leg| {
            ["hip", "thigh", "calf"]
                .iter()
                .map(move |part| format!("{leg}_{part}_joint"))
        })
        .collect();
    assert_eq!(report.dof_order, expected_order);
    assert!(
        (report.total_mass - 16.087).abs() < 1e-3,
        "total_mass = {}",
        report.total_mass
    );
    let mut dropped = report.dropped_frames.clone();
    dropped.sort();
    assert_eq!(dropped, ["front_camera", "imu", "radar"]);
    assert_eq!(
        report.merged_fixed.len(),
        26,
        "merged_fixed = {:?}",
        report.merged_fixed
    );
    assert!(report.meshes.is_empty(), "Go2 碰撞没有网格");
}

/// G2:字段抽查(架构师对照 URDF 手算)。对不上先打印实际值。
// 期望值是任务书照抄的字面量(如 1.0472),不改写成 π/3 常量。
#[allow(clippy::approx_constant)]
#[test]
fn g2_field_spot_checks() {
    let (def, _) = import_go2();
    let base = def
        .links
        .iter()
        .find(|l| l.name == "base")
        .expect("base 连杆");
    match &base.shapes[0] {
        types::ShapeDef::Box { half, offset } => {
            println!("g2: base.shapes[0] = Box half {half:?} offset {offset:?}");
            for (i, e) in [0.1881f32, 0.057, 0.04675].iter().enumerate() {
                assert!(
                    (half[i] - e).abs() <= TOL,
                    "base half[{i}] = {} 期望 {e}",
                    half[i]
                );
            }
            assert_eq!(*offset, [0.0, 0.0, 0.0], "base offset 实测 {offset:?}");
        }
        other => panic!("base 第一个形状应是 Box,实测 {other:?}"),
    }
    let p0 = def.joints[0].kind.params();
    println!("g2: joint0({}) = {p0:?}", def.joints[0].child);
    assert_eq!(def.joints[0].child, "FL_hip");
    assert_eq!(p0.axis, [1.0, 0.0, 0.0], "FL_hip axis 实测 {:?}", p0.axis);
    let l0 = p0.limits.expect("FL_hip 限位");
    assert!(
        (l0.lower - (-1.0472)).abs() <= TOL,
        "FL_hip lower = {}",
        l0.lower
    );
    assert!(
        (l0.upper - 1.0472).abs() <= TOL,
        "FL_hip upper = {}",
        l0.upper
    );
    assert_eq!(
        p0.max_torque,
        Some(23.7),
        "FL_hip max_torque 实测 {:?}",
        p0.max_torque
    );
    assert_eq!(
        p0.max_velocity,
        Some(30.1),
        "FL_hip max_velocity 实测 {:?}",
        p0.max_velocity
    );
    assert_eq!(
        p0.default_angle,
        Some(0.0),
        "FL_hip default_angle 实测 {:?}",
        p0.default_angle
    );

    let p2 = def.joints[2].kind.params();
    println!("g2: joint2({}) = {p2:?}", def.joints[2].child);
    let l2 = p2.limits.expect("FL_calf 限位");
    assert!(
        (l2.lower - (-2.7227)).abs() <= TOL,
        "FL_calf lower = {}",
        l2.lower
    );
    assert!(
        (l2.upper - (-0.83776)).abs() <= TOL,
        "FL_calf upper = {}",
        l2.upper
    );
    assert_eq!(
        p2.max_torque,
        Some(45.43),
        "FL_calf max_torque 实测 {:?}",
        p2.max_torque
    );
    assert_eq!(
        p2.max_velocity,
        Some(15.7),
        "FL_calf max_velocity 实测 {:?}",
        p2.max_velocity
    );
    assert_eq!(
        p2.default_angle,
        Some(-1.5),
        "FL_calf default_angle 实测 {:?}",
        p2.default_angle
    );

    let p1 = def.joints[1].kind.params();
    println!("g2: joint1({}) = {p1:?}", def.joints[1].child);
    assert_eq!(
        p1.default_angle,
        Some(0.8),
        "FL_thigh default_angle 实测 {:?}",
        p1.default_angle
    );
    assert_eq!(
        p1.axis,
        [0.0, 0.0, -1.0],
        "FL_thigh axis 实测 {:?}",
        p1.axis
    );
}

/// G3:入库文件是最新的——重新导入的 hash 等于文件读入的 hash。
#[test]
fn g3_asset_up_to_date() {
    let (def, _) = import_go2();
    let text = std::fs::read_to_string(
        repo_root()
            .join("assets")
            .join("bodies")
            .join("go2.body.toml"),
    )
    .expect("读 go2.body.toml");
    let from_file = types::BodyDef::from_toml_str(&text).expect("文件应能解析");
    assert_eq!(
        def.hash(),
        from_file.hash(),
        "入库文件与当前导入逻辑输出不一致"
    );
}

/// G4:确定性——导入两次,body_to_toml 逐字节相同。
#[test]
fn g4_deterministic() {
    let (a, _) = import_go2();
    let (b, _) = import_go2();
    assert_eq!(body_import::body_to_toml(&a), body_import::body_to_toml(&b));
}

/// N1(只报数):由碰撞形状推出的惯量对角元 vs LinkDef.inertia(URDF)。
/// 照内核的建形状口(cuboid/ball/capsule_y/convex_hull),parry3d 密度 1
/// 算质量属性,按体积比例分连杆质量,平行轴定理合成到连杆质心。
#[test]
fn n1_inertia_table() {
    use parry3d::math::Vector;
    use parry3d::shape::SharedShape;
    let (def, _) = import_go2();
    println!(
        "n1: | 连杆 | 质量 | 形状数 | Ixx_shape | Iyy_shape | Izz_shape | Ixx_urdf | Iyy_urdf | Izz_urdf | rx | ry | rz |"
    );
    println!("n1: |---|---|---|---|---|---|---|---|---|---|---|---|");
    for link in &def.links {
        let mut parts: Vec<(f64, [f64; 3], [[f64; 3]; 3])> = Vec::new();
        for s in &link.shapes {
            let (shape, offset) = match s {
                types::ShapeDef::Box { half, offset } => {
                    (SharedShape::cuboid(half[0], half[1], half[2]), *offset)
                }
                types::ShapeDef::Sphere { radius, offset } => (SharedShape::ball(*radius), *offset),
                types::ShapeDef::CapsuleY {
                    half_height,
                    radius,
                    offset,
                } => (SharedShape::capsule_y(*half_height, *radius), *offset),
                types::ShapeDef::ConvexHull { points, offset } => {
                    let pts: Vec<Vector> = points
                        .iter()
                        .map(|p| Vector::new(p[0], p[1], p[2]))
                        .collect();
                    (SharedShape::convex_hull(&pts).expect("凸包可建"), *offset)
                }
            };
            let mp = shape.mass_properties(1.0);
            let v = f64::from(mp.mass());
            let com = [
                f64::from(offset[0]) + f64::from(mp.local_com.x),
                f64::from(offset[1]) + f64::from(mp.local_com.y),
                f64::from(offset[2]) + f64::from(mp.local_com.z),
            ];
            let m = mp.reconstruct_inertia_matrix();
            let cols = m.to_cols_array();
            let mut tensor = [[0.0f64; 3]; 3];
            for (i, row) in tensor.iter_mut().enumerate() {
                for (j, slot) in row.iter_mut().enumerate() {
                    *slot = f64::from(cols[j * 3 + i]);
                }
            }
            parts.push((v, com, tensor));
        }
        let vol: f64 = parts.iter().map(|p| p.0).sum();
        let mass = f64::from(link.mass);
        let mut com = [0.0f64; 3];
        for (v, c, _) in &parts {
            let mi = mass * v / vol;
            for (k, slot) in com.iter_mut().enumerate() {
                *slot += mi * c[k];
            }
        }
        for slot in com.iter_mut() {
            *slot /= mass;
        }
        let mut diag = [0.0f64; 3];
        for (v, c, t) in &parts {
            let mi = mass * v / vol;
            let scale = mi / v;
            let d = [c[0] - com[0], c[1] - com[1], c[2] - com[2]];
            let d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            for (k, slot) in diag.iter_mut().enumerate() {
                let parallel = mi * (d2 - d[k] * d[k]);
                *slot += scale * t[k][k] + parallel;
            }
        }
        let urdf = link.inertia.unwrap_or([0.0, 0.0, 0.0]);
        let ratio: Vec<String> = (0..3)
            .map(|k| {
                if urdf[k] > 0.0 {
                    format!("{:.3}", diag[k] / f64::from(urdf[k]))
                } else {
                    "-".to_owned()
                }
            })
            .collect();
        println!(
            "n1: | {} | {:.4} | {} | {:.6e} | {:.6e} | {:.6e} | {:.6e} | {:.6e} | {:.6e} | {} | {} | {} |",
            link.name,
            link.mass,
            parts.len(),
            diag[0],
            diag[1],
            diag[2],
            urdf[0],
            urdf[1],
            urdf[2],
            ratio[0],
            ratio[1],
            ratio[2]
        );
        println!(
            "n1-com: {} com_shape = [{:.5}, {:.5}, {:.5}] com_urdf = {:?} vol = {:.6e}",
            link.name, com[0], com[1], com[2], link.com, vol
        );
    }
}
