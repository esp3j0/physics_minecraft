//! REQ-012 E02a §1:agent_link_mass_properties 单测(K1–K3,quad 身体)。
//! 期望值照任务书手算,不改。

use std::path::{Path, PathBuf};

use physics_sim::agent_link_mass_properties;
use types::BodyDef;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("仓库根")
}

fn quad() -> BodyDef {
    let text =
        std::fs::read_to_string(repo_root().join("assets/bodies/quad.body.toml")).expect("读 quad");
    BodyDef::from_toml_str(&text).expect("quad 合法")
}

fn rel_err(got: f32, want: f32) -> f32 {
    (got - want).abs() / want.abs()
}

/// K1 机身盒:mass==6.921;com≈0;惯量对角 m/3·(b²+c²) 等,相对误差<1e-4;非对角<1e-8。
#[test]
fn k1_body_box() {
    let def = quad();
    let props = agent_link_mass_properties(&def);
    let p = &props[0];
    assert_eq!(p.mass, 6.921, "机身 mass");
    for c in p.com {
        assert!(c.abs() < 1e-7, "机身 com 分量 {c} 应 ≈ 0");
    }
    let diag = [p.inertia[0][0], p.inertia[1][1], p.inertia[2][2]];
    let want = [0.012591606f32, 0.088378863, 0.090778143];
    for (i, (&g, &w)) in diag.iter().zip(&want).enumerate() {
        assert!(
            rel_err(g, w) < 1e-4,
            "机身惯量对角[{i}] = {g} 期望 {w} 相对误差 {}",
            rel_err(g, w)
        );
    }
    for (i, row) in p.inertia.iter().enumerate() {
        for (j, &v) in row.iter().enumerate() {
            if i != j {
                assert!(v.abs() < 1e-8, "机身惯量非对角[{i}][{j}] = {v} 应 ≈ 0");
            }
        }
    }
}

/// K2 leg0 髋球:三个对角元都是 2/5·m·r² = 0.00033222,相对误差<1e-4。
#[test]
fn k2_hip_sphere() {
    let def = quad();
    let props = agent_link_mass_properties(&def);
    let p = &props[1];
    assert_eq!(p.mass, 0.678, "髋 mass");
    let want = 0.00033222f32;
    for (i, &g) in [p.inertia[0][0], p.inertia[1][1], p.inertia[2][2]]
        .iter()
        .enumerate()
    {
        assert!(
            rel_err(g, want) < 1e-4,
            "髋惯量对角[{i}] = {g} 期望 {want} 相对误差 {}",
            rel_err(g, want)
        );
    }
}

/// K3:各连杆 mass 逐位等于定义;13 连杆都有限、对角>0;小腿 com.y<0。
#[test]
fn k3_all_links() {
    let def = quad();
    let props = agent_link_mass_properties(&def);
    assert_eq!(props.len(), 13, "quad 13 连杆");
    assert_eq!(props.len(), def.links.len());
    for (i, (p, link)) in props.iter().zip(&def.links).enumerate() {
        assert_eq!(
            p.mass.to_bits(),
            link.mass.to_bits(),
            "连杆 {i} mass 逐位等于定义"
        );
        assert!(p.mass.is_finite(), "连杆 {i} mass 有限");
        for c in p.com {
            assert!(c.is_finite(), "连杆 {i} com 有限");
        }
        for (r, row) in p.inertia.iter().enumerate() {
            for &v in row {
                assert!(v.is_finite(), "连杆 {i} 惯量有限");
            }
            assert!(row[r] > 0.0, "连杆 {i} 惯量对角[{r}] > 0");
        }
        if link.name.contains("shank") {
            assert!(
                p.com[1] < 0.0,
                "小腿 {} com.y = {} 应 < 0(脚球往下拉)",
                link.name,
                p.com[1]
            );
        }
    }
}
