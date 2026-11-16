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
    assert_eq!(report.merged_fixed.len(), 26, "merged_fixed = {:?}", report.merged_fixed);
    assert!(report.meshes.is_empty(), "Go2 碰撞没有网格");
}

/// G2:字段抽查(架构师对照 URDF 手算)。对不上先打印实际值。
#[test]
fn g2_field_spot_checks() {
    let (def, _) = import_go2();
    let base = def.links.iter().find(|l| l.name == "base").expect("base 连杆");
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
    assert!((l0.lower - (-1.0472)).abs() <= TOL, "FL_hip lower = {}", l0.lower);
    assert!((l0.upper - 1.0472).abs() <= TOL, "FL_hip upper = {}", l0.upper);
    assert_eq!(p0.max_torque, Some(23.7), "FL_hip max_torque 实测 {:?}", p0.max_torque);
    assert_eq!(p0.max_velocity, Some(30.1), "FL_hip max_velocity 实测 {:?}", p0.max_velocity);
    assert_eq!(p0.default_angle, Some(0.0), "FL_hip default_angle 实测 {:?}", p0.default_angle);

    let p2 = def.joints[2].kind.params();
    println!("g2: joint2({}) = {p2:?}", def.joints[2].child);
    let l2 = p2.limits.expect("FL_calf 限位");
    assert!((l2.lower - (-2.7227)).abs() <= TOL, "FL_calf lower = {}", l2.lower);
    assert!((l2.upper - (-0.83776)).abs() <= TOL, "FL_calf upper = {}", l2.upper);
    assert_eq!(p2.max_torque, Some(45.43), "FL_calf max_torque 实测 {:?}", p2.max_torque);
    assert_eq!(p2.max_velocity, Some(15.7), "FL_calf max_velocity 实测 {:?}", p2.max_velocity);
    assert_eq!(p2.default_angle, Some(-1.5), "FL_calf default_angle 实测 {:?}", p2.default_angle);

    let p1 = def.joints[1].kind.params();
    println!("g2: joint1({}) = {p1:?}", def.joints[1].child);
    assert_eq!(p1.default_angle, Some(0.8), "FL_thigh default_angle 实测 {:?}", p1.default_angle);
    assert_eq!(p1.axis, [0.0, 0.0, -1.0], "FL_thigh axis 实测 {:?}", p1.axis);
}

/// G3:入库文件是最新的——重新导入的 hash 等于文件读入的 hash。
#[test]
fn g3_asset_up_to_date() {
    let (def, _) = import_go2();
    let text = std::fs::read_to_string(repo_root().join("assets").join("bodies").join("go2.body.toml"))
        .expect("读 go2.body.toml");
    let from_file = types::BodyDef::from_toml_str(&text).expect("文件应能解析");
    assert_eq!(def.hash(), from_file.hash(), "入库文件与当前导入逻辑输出不一致");
}

/// G4:确定性——导入两次,body_to_toml 逐字节相同。
#[test]
fn g4_deterministic() {
    let (a, _) = import_go2();
    let (b, _) = import_go2();
    assert_eq!(body_import::body_to_toml(&a), body_import::body_to_toml(&b));
}