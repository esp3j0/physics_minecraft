//! REQ-012 E02a §2:body_to_mjcf 测试(M1–M5)。期望值照任务书。

use std::path::{Path, PathBuf};

use types::BodyDef;
use xtask::export_mjcf::{MjcfOptions, body_to_mjcf};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("仓库根")
}

fn body(name: &str) -> BodyDef {
    let text = std::fs::read_to_string(repo_root().join(format!("assets/bodies/{name}.body.toml")))
        .expect("读身体");
    BodyDef::from_toml_str(&text).expect("身体合法")
}

fn count(hay: &str, needle: &str) -> usize {
    hay.match_indices(needle).count()
}

/// M1:quad/go2 都能导出;<body 数=连杆数,<position 数=自由度数(12);含关键属性。
#[test]
fn m1_export_counts() {
    for name in ["quad", "go2"] {
        let def = body(name);
        let (xml, _rep) = body_to_mjcf(&def, &MjcfOptions::default());
        assert_eq!(
            count(&xml, "<body "),
            def.links.len(),
            "{name} <body 数 = 连杆数"
        );
        assert_eq!(
            count(&xml, "<position "),
            def.dof_count(),
            "{name} <position 数 = 自由度数"
        );
        assert_eq!(def.dof_count(), 12, "{name} 12 自由度");
        assert!(xml.contains("gravity=\"0 -9.81 0\""), "{name} 重力");
        assert!(xml.contains("zaxis=\"0 1 0\""), "{name} 地面 zaxis");
        assert!(
            xml.contains("inertiafromgeom=\"false\""),
            "{name} inertiafromgeom"
        );
    }
}

/// M2 确定性:同样输入两次导出逐字节相同。
#[test]
fn m2_deterministic() {
    let def = body("go2");
    let opts = MjcfOptions::default();
    let (a, ra) = body_to_mjcf(&def, &opts);
    let (b, rb) = body_to_mjcf(&def, &opts);
    assert_eq!(a, b, "MJCF 文本逐字节相同");
    assert_eq!(
        ra.to_json().to_string(),
        rb.to_json().to_string(),
        "报告逐字节相同"
    );
}

/// 提取所有 <mesh ... vertex="..."/> 的 vertex 字符串。
fn mesh_vertices(xml: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = xml;
    while let Some(pos) = rest.find("<mesh ") {
        rest = &rest[pos..];
        let end = rest.find("/>").expect("mesh 自闭合");
        let tag = &rest[..end];
        let vstart = tag.find("vertex=\"").expect("mesh 有 vertex") + "vertex=\"".len();
        let vrest = &tag[vstart..];
        let vend = vrest.find('"').expect("vertex 闭合引号");
        out.push(vrest[..vend].to_owned());
        rest = &rest[end..];
    }
    out
}

/// M3 凸包:go2 <mesh 数=定义里凸包个数;每条 vertex 数字个数=点数×3(3 的倍数)。
#[test]
fn m3_convex_hulls() {
    let def = body("go2");
    let (xml, _rep) = body_to_mjcf(&def, &MjcfOptions::default());
    let hulls: Vec<usize> = def
        .links
        .iter()
        .flat_map(|l| l.shapes.iter())
        .filter_map(|s| match s {
            types::ShapeDef::ConvexHull { points, .. } => Some(points.len()),
            _ => None,
        })
        .collect();
    let verts = mesh_vertices(&xml);
    assert_eq!(verts.len(), hulls.len(), "<mesh 数 = 凸包个数");
    for (v, &npts) in verts.iter().zip(&hulls) {
        let nums = v.split_whitespace().count();
        assert_eq!(nums % 3, 0, "vertex 数字个数是 3 的倍数");
        assert_eq!(nums, npts * 3, "vertex 数字个数 = 点数×3");
    }
}

/// M4 Fixed/Prismatic:三连杆(根+Fixed 子+Prismatic 子,盒);<joint 只 1 条且 slide,
/// <position 只 1 条。
#[test]
fn m4_fixed_prismatic() {
    let text = r#"
name = "tri"
[[links]]
name = "root"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[links]]
name = "fixed_child"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[links]]
name = "slide_child"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[joints]]
parent = "root"
child = "fixed_child"
type = "fixed"
axis = [1.0, 0.0, 0.0]
anchor = [0.2, 0.0, 0.0]
[[joints]]
parent = "root"
child = "slide_child"
type = "prismatic"
axis = [0.0, 1.0, 0.0]
anchor = [-0.2, 0.0, 0.0]
limits = { lower = -0.5, upper = 0.5 }
max_torque = 10.0
kp = 50.0
kd = 2.0
"#;
    let def = BodyDef::from_toml_str(text).expect("三连杆合法");
    let (xml, _rep) = body_to_mjcf(&def, &MjcfOptions::default());
    assert_eq!(count(&xml, "<joint "), 1, "只有 1 条 <joint(Fixed 不写)");
    assert!(xml.contains("type=\"slide\""), "Prismatic → slide");
    assert!(!xml.contains("type=\"hinge\""), "没有 hinge");
    assert_eq!(count(&xml, "<position "), 1, "只有 1 条执行器");
}

/// M5 报告:quad 连杆 0 惯量对角与 K1 相对误差<1e-4;自由度 3 upper 逐位等于定义。
#[test]
fn m5_report() {
    let def = body("quad");
    let (_xml, rep) = body_to_mjcf(&def, &MjcfOptions::default());
    let want = [0.012591606f32, 0.088378863, 0.090778143];
    let inertia = rep.links[0].inertia;
    let diag = [inertia[0][0], inertia[1][1], inertia[2][2]];
    for (i, (&g, &w)) in diag.iter().zip(&want).enumerate() {
        let rel = (g - w).abs() / w.abs();
        assert!(
            rel < 1e-4,
            "报告连杆 0 惯量对角[{i}] = {g} 期望 {w} 相对误差 {rel}"
        );
    }
    let dof3_upper = def
        .dof_joints()
        .nth(3)
        .expect("自由度 3")
        .kind
        .params()
        .limits
        .expect("自由度 3 有限位")
        .upper;
    assert_eq!(
        rep.dofs[3].upper.expect("报告自由度 3 有 upper").to_bits(),
        dof3_upper.to_bits(),
        "报告自由度 3 upper 逐位等于定义"
    );
}
