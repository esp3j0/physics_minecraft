//! REQ-012 C01b M 组:网格导入测试。
//! 期望值由架构师手算给出(issue #25 C01b 任务书),照抄,不改数字。
//! 浮点容差 1e-5,体积容差 1e-6;凸包点一律当集合比较。

use body_import::{ImportError, ImportOptions, ImportReport, UpAxis};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

const TOL: f32 = 1e-5;
const VTOL: f32 = 1e-6;

fn data_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
}

fn opts() -> ImportOptions {
    ImportOptions {
        name: "body".to_string(),
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

/// 单连杆 `body`:质量 1、对角 0.01 惯量,碰撞是一个网格。
fn mesh_urdf(filename: &str, scale: Option<&str>, rpy: Option<&str>) -> String {
    let scale_attr = scale.map(|s| format!(" scale=\"{s}\"")).unwrap_or_default();
    let origin = rpy
        .map(|r| format!("<origin xyz=\"0 0 0\" rpy=\"{r}\"/>"))
        .unwrap_or_default();
    format!(
        r#"<robot name="body">
  <link name="body">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <collision>{origin}<geometry><mesh filename="{filename}"{scale_attr}/></geometry></collision>
  </link>
</robot>"#
    )
}

fn import(xml: &str, base: &Path, o: &ImportOptions) -> (types::BodyDef, ImportReport) {
    body_import::import_urdf(xml, base, o).expect("导入应成功")
}

fn import_err(xml: &str, base: &Path, o: &ImportOptions) -> ImportError {
    body_import::import_urdf(xml, base, o).expect_err("导入应失败")
}

fn hull_points(def: &types::BodyDef) -> Vec<[f32; 3]> {
    assert_eq!(def.links.len(), 1);
    assert_eq!(def.links[0].shapes.len(), 1);
    match &def.links[0].shapes[0] {
        types::ShapeDef::ConvexHull { points, offset } => {
            assert_eq!(*offset, [0.0, 0.0, 0.0], "碰撞原点无平移");
            points.clone()
        }
        other => panic!("期望 ConvexHull,得到 {other:?}"),
    }
}

/// 凸包点当集合比较:数量相等,每个期望点都能在实际点里找到(容差 TOL)。
fn assert_point_set(actual: &[[f32; 3]], expected: &[[f32; 3]]) {
    assert_eq!(actual.len(), expected.len(), "点数:实际 {actual:?}");
    for e in expected {
        assert!(
            actual
                .iter()
                .any(|p| (0..3).all(|i| (p[i] - e[i]).abs() <= TOL)),
            "缺期望点 {e:?},实际 {actual:?}"
        );
    }
}

fn assert_vol(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() <= VTOL,
        "体积 actual {actual} vs expected {expected}"
    );
}

/// 盒的 8 个角点(期望集合,±hx × ±hy × ±hz)。
fn corners(hx: f32, hy: f32, hz: f32) -> Vec<[f32; 3]> {
    let mut v = Vec::new();
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                v.push([sx * hx, sy * hy, sz * hz]);
            }
        }
    }
    v
}

/// M1:box.obj → 8 点凸包 (±0.1, ±0.3, ±0.2);统计与体积照任务书。
#[test]
fn m1_box_obj() {
    let (def, report) = import(&mesh_urdf("box.obj", None, None), &data_dir(), &opts());
    assert_point_set(&hull_points(&def), &corners(0.1, 0.3, 0.2));
    assert_eq!(report.meshes.len(), 1);
    let m = &report.meshes[0];
    assert_eq!(m.link, "body");
    assert_eq!(m.file, "box.obj");
    assert_eq!(m.vertices, 10);
    assert_eq!(m.triangles, 12);
    assert_eq!(m.hull_full, 8);
    assert_eq!(m.hull_out, 8);
    assert_vol(m.mesh_volume, 0.048);
    assert_vol(m.hull_volume, 0.048);
    assert_vol(m.out_volume, 0.048);
    let sn = &report.shapes[0];
    assert_eq!(sn.from, "mesh");
    assert_eq!(sn.to, "convex_hull");
    assert_eq!(sn.hull_points, 8);
    assert_vol(sn.volume_src, 0.048);
    assert_vol(sn.volume_out, 0.048);
}

/// M1b(变异 11 补):box.obj 所有面反向,体积仍应取绝对值 0.048。
#[test]
fn m1b_reversed_faces() {
    let tmp = tempfile::TempDir::new().expect("临时目录");
    let obj = "# 面全部反向的盒\nv -0.1 -0.2 -0.3\nv 0.1 -0.2 -0.3\nv 0.1 0.2 -0.3\nv -0.1 0.2 -0.3\nv -0.1 -0.2 0.3\nv 0.1 -0.2 0.3\nv 0.1 0.2 0.3\nv -0.1 0.2 0.3\nv 0 0 0\nv 0.1 0.2 0.3\nf 1 2 3 4\nf 8 7 6 5\nf 5 6 2 1\nf 6 7 3 2\nf 7 8 4 3\nf 8 5 1 4\n";
    std::fs::write(tmp.path().join("rbox.obj"), obj).expect("写反向 OBJ");
    let (def, report) = import(&mesh_urdf("rbox.obj", None, None), tmp.path(), &opts());
    assert_point_set(&hull_points(&def), &corners(0.1, 0.3, 0.2));
    let m = &report.meshes[0];
    assert_eq!(m.vertices, 10);
    assert_eq!(m.triangles, 12);
    assert_eq!(m.hull_full, 8);
    assert_vol(m.mesh_volume, 0.048);
    assert_vol(m.hull_volume, 0.048);
    assert_vol(m.out_volume, 0.048);
}

/// M2:scale="2 1 0.5" → (±0.2, ±0.15, ±0.2),三个体积都是 0.048。
#[test]
fn m2_scale() {
    let (def, report) = import(
        &mesh_urdf("box.obj", Some("2 1 0.5"), None),
        &data_dir(),
        &opts(),
    );
    assert_point_set(&hull_points(&def), &corners(0.2, 0.15, 0.2));
    let m = &report.meshes[0];
    assert_vol(m.mesh_volume, 0.048);
    assert_vol(m.hull_volume, 0.048);
    assert_vol(m.out_volume, 0.048);
}

/// M3:碰撞原点 rpy 绕 z 90° → (±0.2, ±0.3, ±0.1),体积 0.048。
#[test]
fn m3_rotated() {
    let (def, report) = import(
        &mesh_urdf("box.obj", None, Some("0 0 1.5707963267948966")),
        &data_dir(),
        &opts(),
    );
    assert_point_set(&hull_points(&def), &corners(0.2, 0.3, 0.1));
    let m = &report.meshes[0];
    assert_vol(m.mesh_volume, 0.048);
    assert_vol(m.hull_volume, 0.048);
    assert_vol(m.out_volume, 0.048);
}

/// M4 的导入(布景 M8 / M13 复用)。
fn import_lshape() -> (types::BodyDef, ImportReport) {
    import(
        &mesh_urdf("lshape.obj", Some("0.1 0.1 0.1"), None),
        &data_dir(),
        &opts(),
    )
}

/// M4:L 形柱非凸;统计、体积、点集、非凸提示照任务书。
#[test]
fn m4_lshape_nonconvex() {
    let (def, report) = import_lshape();
    let m = &report.meshes[0];
    assert_eq!(m.vertices, 12);
    assert_eq!(m.triangles, 20);
    assert_eq!(m.hull_full, 10);
    assert_eq!(m.hull_out, 10);
    assert_vol(m.mesh_volume, 0.003);
    assert_vol(m.hull_volume, 0.0035);
    assert_vol(m.out_volume, 0.0035);
    let mut expected = Vec::new();
    for y in [0.0f32, 0.1] {
        for (x, z) in [
            (0.0f32, 0.0f32),
            (0.2, 0.0),
            (0.2, -0.1),
            (0.1, -0.2),
            (0.0, -0.2),
        ] {
            expected.push([x, y, z]);
        }
    }
    assert_point_set(&hull_points(&def), &expected);
    assert!(
        report
            .notes
            .iter()
            .any(|n| n.contains("非凸") && n.contains("1.17")),
        "notes 应有非凸提示(1.17 倍):{:?}",
        report.notes
    );
}

/// M5:ASCII STL 正八面体,scale 0.1 → 6 点,三个体积 0.0013333。
#[test]
fn m5_octa_stl_ascii() {
    let (def, report) = import(
        &mesh_urdf("octa.stl", Some("0.1 0.1 0.1"), None),
        &data_dir(),
        &opts(),
    );
    let m = &report.meshes[0];
    assert_eq!(m.vertices, 24);
    assert_eq!(m.triangles, 8);
    assert_eq!(m.hull_full, 6);
    let expected = [
        [0.1, 0.0, 0.0],
        [-0.1, 0.0, 0.0],
        [0.0, 0.1, 0.0],
        [0.0, -0.1, 0.0],
        [0.0, 0.0, 0.1],
        [0.0, 0.0, -0.1],
    ];
    assert_point_set(&hull_points(&def), &expected);
    assert_vol(m.mesh_volume, 0.0013333);
    assert_vol(m.hull_volume, 0.0013333);
    assert_vol(m.out_volume, 0.0013333);
}

/// 手写二进制 STL(测试布景用)。
fn write_binary_stl(path: &Path, tris: &[[[f32; 3]; 3]]) {
    let mut b = vec![0u8; 80];
    b.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for t in tris {
        b.extend_from_slice(&[0.0f32; 3].map(f32::to_le_bytes).concat());
        for v in t {
            b.extend_from_slice(&v.map(f32::to_le_bytes).concat());
        }
        b.extend_from_slice(&[0u8; 2]);
    }
    std::fs::write(path, b).expect("写二进制 STL");
}

/// M6:二进制 STL 四面体(测试里生成),URDF 同目录。
#[test]
fn m6_stl_binary() {
    let tmp = tempfile::TempDir::new().expect("临时目录");
    let o = [0.0f32, 0.0, 0.0];
    let x = [1.0f32, 0.0, 0.0];
    let y = [0.0f32, 1.0, 0.0];
    let z = [0.0f32, 0.0, 1.0];
    write_binary_stl(
        &tmp.path().join("tetra.stl"),
        &[[o, y, x], [o, x, z], [o, z, y], [x, y, z]],
    );
    let (def, report) = import(&mesh_urdf("tetra.stl", None, None), tmp.path(), &opts());
    let m = &report.meshes[0];
    assert_eq!(m.vertices, 12);
    assert_eq!(m.triangles, 4);
    assert_eq!(m.hull_full, 4);
    assert_vol(m.mesh_volume, 0.1666667);
    assert_vol(m.hull_volume, 0.1666667);
    assert_vol(m.out_volume, 0.1666667);
    let expected = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ];
    assert_point_set(&hull_points(&def), &expected);
}

/// M7:两圈各 64 点的圆柱面(只有 v 行),max_hull_points = 16 → 精简;
/// 确定性:两次导入 hash 与点序完全相同。
#[test]
fn m7_reduce() {
    let tmp = tempfile::TempDir::new().expect("临时目录");
    let mut obj = String::new();
    let mut input_link: Vec<[f32; 3]> = Vec::new();
    for z in [0.5f64, -0.5] {
        for k in 0..64 {
            let t = 2.0 * std::f64::consts::PI * (k as f64) / 64.0;
            let (s, c) = t.sin_cos();
            obj.push_str(&format!("v {c} {s} {z}\n"));
            // URDF (x,y,z) → 连杆系 (x,z,−y)。
            input_link.push([c as f32, z as f32, -(s as f32)]);
        }
    }
    std::fs::write(tmp.path().join("cyl.obj"), obj).expect("写 OBJ");
    let mut o = opts();
    o.max_hull_points = 16;
    let xml = mesh_urdf("cyl.obj", None, None);
    let (def, report) = import(&xml, tmp.path(), &o);
    let m = &report.meshes[0];
    assert_eq!(m.hull_full, 128);
    assert_eq!(m.hull_out, 16);
    let pts = hull_points(&def);
    for p in &pts {
        assert!(
            input_link
                .iter()
                .any(|q| (0..3).all(|i| (p[i] - q[i]).abs() <= TOL)),
            "输出点 {p:?} 不在输入点集里"
        );
    }
    assert!(m.out_volume < m.hull_volume);
    assert!(m.out_volume > 0.5 * m.hull_volume);
    assert!(
        (m.hull_volume - 3.136548).abs() < 1e-3,
        "hull_volume = {}",
        m.hull_volume
    );
    assert_vol(m.mesh_volume, 0.0);
    assert!(
        report.notes.iter().any(|n| n.contains("体积偏差无法给出")),
        "notes 应有零体积提示:{:?}",
        report.notes
    );
    let (def2, _) = import(&xml, tmp.path(), &o);
    assert_eq!(def.hash(), def2.hash(), "两次导入同哈希");
    assert_eq!(pts, hull_points(&def2), "两次导入点序完全相同");
}

/// M8:M4 的身体进内核(照 kernel_reconcile 布景),步进 60 拍不 panic、
/// 位姿有限。
#[test]
fn m8_kernel_accepts() {
    use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
    use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
    use types::glam::{Quat, Vec3};
    use types::{ChunkPos, DimensionId, EntityId, ShardPos, Tick, WorldId};
    use voxel::SimpleRegistry;

    let (def, _) = import_lshape();
    let toml_text = body_import::body_to_toml(&def);
    let mut bodies = BodyRegistry::new();
    let hash = bodies.insert_toml(&toml_text).expect("注册表收下");
    let mut cfg = ShardSimConfig::new(types::ShardId::new(
        WorldId(0),
        DimensionId(0),
        ShardPos::new(0, 0),
    ));
    cfg.agent_mass_scale = 1.0;
    let mut sim = ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), bodies);
    let stone = types::BlockId(1);
    for cx in 0..3i32 {
        for cz in 0..3i32 {
            let mut chunk = voxel::Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    chunk.set(x, 64, z, stone);
                }
            }
            sim.load_chunk(ChunkPos::new(cx, cz), chunk);
        }
    }
    let entity = EntityId::from_parts(1, 1);
    sim.apply_prefix(&[StampedInput {
        apply_at: Tick::ZERO,
        source: InputSource::SCHEDULER,
        sequence: 0,
        payload: InputPayload::Command(SimCommand::SpawnAgent {
            entity,
            body: hash,
            state: types::EntityState {
                translation: Vec3::new(24.0, 20.0, 24.0),
                rotation: Quat::IDENTITY,
                ..types::EntityState::default()
            },
        }),
    }]);
    for _ in 0..60 {
        sim.step();
        sim.run_transient_lifecycle();
        let _ = sim.take_outbox();
    }
    let (.., poses) = sim
        .agent_states()
        .into_iter()
        .find(|(id, ..)| *id == entity)
        .expect("主体已生成");
    assert_eq!(poses.len(), 1);
    assert!(
        poses[0].translation.is_finite(),
        "{:?}",
        poses[0].translation
    );
    assert!(poses[0].rotation.is_finite(), "{:?}", poses[0].rotation);
}

/// M9:缺网格文件 → MeshFileMissing,tried 3 条(AC-C2)。
#[test]
fn m9_missing_file() {
    match import_err(
        &mesh_urdf("package://robot/meshes/none.stl", None, None),
        &data_dir(),
        &opts(),
    ) {
        ImportError::MeshFileMissing { link, file, tried } => {
            assert_eq!(link, "body");
            assert!(file.contains("none.stl"), "file {file:?}");
            assert_eq!(tried.len(), 3, "tried {tried:?}");
        }
        other => panic!("want MeshFileMissing, got {other:?}"),
    }
}

/// M10:package:// 回退 / 显式包目录 / `..` 拦截。
#[test]
fn m10_resolve() {
    let tmp = tempfile::TempDir::new().expect("临时目录");
    let urdf_dir = tmp.path().join("pkg").join("urdf");
    let meshes = tmp.path().join("pkg").join("meshes");
    std::fs::create_dir_all(&urdf_dir).expect("建 urdf 目录");
    std::fs::create_dir_all(&meshes).expect("建 meshes 目录");
    std::fs::copy(data_dir().join("box.obj"), meshes.join("box.obj")).expect("拷网格");
    let xml = mesh_urdf("package://pkg/meshes/box.obj", None, None);
    // 回退:base_dir/../meshes/box.obj。
    let (def, report) = import(&xml, &urdf_dir, &opts());
    assert_eq!(hull_points(&def).len(), 8);
    assert_eq!(report.meshes.len(), 1);
    // 显式包目录指向空目录 → 只试 1 条。
    let empty = tmp.path().join("empty");
    std::fs::create_dir_all(&empty).expect("建空目录");
    let mut o = opts();
    o.package_dirs.insert("pkg".to_owned(), empty);
    match import_err(&xml, &urdf_dir, &o) {
        ImportError::MeshFileMissing { tried, .. } => {
            assert_eq!(tried.len(), 1, "tried {tried:?}");
        }
        other => panic!("want MeshFileMissing, got {other:?}"),
    }
    // `..`:目标真实存在也不解析。
    std::fs::copy(
        data_dir().join("box.obj"),
        tmp.path().join("pkg").join("secret.obj"),
    )
    .expect("拷 secret");
    match import_err(&mesh_urdf("../secret.obj", None, None), &urdf_dir, &opts()) {
        ImportError::MeshFileMissing { .. } => {}
        other => panic!("want MeshFileMissing, got {other:?}"),
    }
}

/// M11:坏文件各变体。scale="0 1 1" 的实际变体打印出来(回报用)。
#[test]
fn m11_bad_files() {
    let tmp = tempfile::TempDir::new().expect("临时目录");
    let base = tmp.path();
    // 不支持的格式。
    match import_err(&mesh_urdf("model.dae", None, None), base, &opts()) {
        ImportError::UnsupportedShape { link, what } => {
            assert_eq!(link, "body");
            assert!(what.contains("model.dae"), "what {what:?}");
        }
        other => panic!("want UnsupportedShape, got {other:?}"),
    }
    // OBJ 面下标越界。
    std::fs::write(base.join("bad.obj"), "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n").expect("写坏 OBJ");
    assert!(matches!(
        import_err(&mesh_urdf("bad.obj", None, None), base, &opts()),
        ImportError::MeshParse { .. }
    ));
    // 二进制 STL 长度不对。
    write_binary_stl(
        &base.join("short.stl"),
        &[[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]],
    );
    let full = std::fs::read(base.join("short.stl")).expect("读回");
    std::fs::write(base.join("short.stl"), &full[..full.len() - 1]).expect("截短");
    assert!(matches!(
        import_err(&mesh_urdf("short.stl", None, None), base, &opts()),
        ImportError::MeshParse { .. }
    ));
    // 4 个共面点。
    std::fs::write(
        base.join("flat.obj"),
        "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n",
    )
    .expect("写共面 OBJ");
    assert!(matches!(
        import_err(&mesh_urdf("flat.obj", None, None), base, &opts()),
        ImportError::MeshDegenerate { .. }
    ));
    // 4 个共面点 + 旋转(裁定):体积未必是精确 0,仍应 MeshDegenerate。
    assert!(matches!(
        import_err(
            &mesh_urdf("flat.obj", None, Some("0.3 0.5 0.7")),
            base,
            &opts()
        ),
        ImportError::MeshDegenerate { .. }
    ));
    // 只有 3 个点。
    std::fs::write(base.join("tri.obj"), "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
        .expect("写 3 点 OBJ");
    assert!(matches!(
        import_err(&mesh_urdf("tri.obj", None, None), base, &opts()),
        ImportError::MeshDegenerate { .. }
    ));
    // scale 含 0:MeshParse 或 MeshDegenerate(实际变体写进回报)。
    let e = import_err(
        &mesh_urdf("box.obj", Some("0 1 1"), None),
        &data_dir(),
        &opts(),
    );
    println!("m11:scale=\"0 1 1\" 实际变体:{e:?}");
    assert!(matches!(
        e,
        ImportError::MeshParse { .. } | ImportError::MeshDegenerate { .. }
    ));
}

/// M12:max_hull_points 越界 → BadOption。
#[test]
fn m12_options() {
    for bad in [3u32, 256] {
        let mut o = opts();
        o.max_hull_points = bad;
        assert!(
            matches!(
                import_err(&mesh_urdf("box.obj", None, None), &data_dir(), &o),
                ImportError::BadOption(_)
            ),
            "max_hull_points = {bad} 应 BadOption"
        );
    }
}

/// M13:M4 的报告 markdown 含网格一节;无网格的报告不含"## 网格"。
#[test]
fn m13_report_markdown() {
    let (_, report) = import_lshape();
    let md = body_import::report_to_markdown(&report);
    assert!(md.contains("## 网格"), "报告应含网格一节:{md}");
    assert!(md.contains("lshape.obj"));
    assert!(md.contains("0.0035"));
    let xml = std::fs::read_to_string(data_dir().join("testbot.urdf")).expect("读 testbot");
    let mut o = opts();
    o.name = "testbot".to_owned();
    let (_, rep2) = import(&xml, &data_dir(), &o);
    let md2 = body_import::report_to_markdown(&rep2);
    assert!(!md2.contains("## 网格"), "无网格不应出现网格节:{md2}");
}
