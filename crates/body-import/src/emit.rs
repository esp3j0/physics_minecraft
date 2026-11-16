//! REQ-012 C01a:输出文本(任务书 1.7)。
//!
//! `body_to_toml`:`toml::to_string` 直接可用(BodyDef 的 serde 属性
//! 完备:`flatten` + 内部 tag 正确展开、`Option::None` 跳过、f32 走最短
//! 往返文本);唯一与手写资产的差异是 `limits` 写成子表
//! `[joints.limits]` 而非 inline 表——合法 TOML,解析等价。
//! `report_to_markdown`:纯文本拼装。

/// BodyDef → body.toml 文本。
/// `BodyDef::from_toml_str(body_to_toml(d))` 的 `hash()` 必须等于 `d.hash()`。
/// 本函数不加文件头注释(注释由 xtask 那一层加)。
pub fn body_to_toml(def: &types::BodyDef) -> String {
    match toml::to_string(def) {
        Ok(text) => text,
        // 序列化失败:手写输出兜底(参照 assets/bodies/quad.body.toml 的
        // 字段序)。目前 BodyDef 全字段可序列化,此分支不可达。
        Err(_) => body_to_toml_manual(def),
    }
}

/// 手写序列化兜底(C01b 之前不会被走到)。
fn body_to_toml_manual(def: &types::BodyDef) -> String {
    let mut s = String::new();
    let f = |v: f32| format!("{v}");
    s.push_str(&format!("name = {:?}\n", def.name));
    s.push_str(&format!("lock_rotations = {}\n", def.lock_rotations));
    for l in &def.links {
        s.push_str("\n[[links]]\n");
        s.push_str(&format!("name = {:?}\n", l.name));
        s.push_str(&format!("mass = {}\n", f(l.mass)));
        s.push_str(&format!(
            "com = [{}, {}, {}]\n",
            f(l.com[0]),
            f(l.com[1]),
            f(l.com[2])
        ));
        if let Some(i) = &l.inertia {
            s.push_str(&format!(
                "inertia = [{}, {}, {}]\n",
                f(i[0]),
                f(i[1]),
                f(i[2])
            ));
        }
        s.push_str(&format!("friction = {}\n", f(l.friction)));
        s.push_str(&format!("restitution = {}\n", f(l.restitution)));
        for sh in &l.shapes {
            s.push_str("\n[[links.shapes]]\n");
            match sh {
                types::ShapeDef::Box { half, offset } => {
                    s.push_str("kind = \"box\"\n");
                    s.push_str(&format!(
                        "half = [{}, {}, {}]\n",
                        f(half[0]),
                        f(half[1]),
                        f(half[2])
                    ));
                    s.push_str(&format!(
                        "offset = [{}, {}, {}]\n",
                        f(offset[0]),
                        f(offset[1]),
                        f(offset[2])
                    ));
                }
                types::ShapeDef::Sphere { radius, offset } => {
                    s.push_str("kind = \"sphere\"\n");
                    s.push_str(&format!("radius = {}\n", f(*radius)));
                    s.push_str(&format!(
                        "offset = [{}, {}, {}]\n",
                        f(offset[0]),
                        f(offset[1]),
                        f(offset[2])
                    ));
                }
                types::ShapeDef::CapsuleY {
                    half_height,
                    radius,
                    offset,
                } => {
                    s.push_str("kind = \"capsule_y\"\n");
                    s.push_str(&format!("half_height = {}\n", f(*half_height)));
                    s.push_str(&format!("radius = {}\n", f(*radius)));
                    s.push_str(&format!(
                        "offset = [{}, {}, {}]\n",
                        f(offset[0]),
                        f(offset[1]),
                        f(offset[2])
                    ));
                }
                types::ShapeDef::ConvexHull { points, offset } => {
                    s.push_str("kind = \"convex_hull\"\n");
                    let pts: Vec<String> = points
                        .iter()
                        .map(|p| format!("[{}, {}, {}]", f(p[0]), f(p[1]), f(p[2])))
                        .collect();
                    s.push_str(&format!("points = [{}]\n", pts.join(", ")));
                    s.push_str(&format!(
                        "offset = [{}, {}, {}]\n",
                        f(offset[0]),
                        f(offset[1]),
                        f(offset[2])
                    ));
                }
            }
        }
    }
    for j in &def.joints {
        s.push_str("\n[[joints]]\n");
        s.push_str(&format!("parent = {:?}\n", j.parent));
        s.push_str(&format!("child = {:?}\n", j.child));
        let p = j.kind.params();
        let ty = match j.kind {
            types::JointKind::Revolute(_) => "revolute",
            types::JointKind::Prismatic(_) => "prismatic",
            types::JointKind::Fixed(_) => "fixed",
        };
        s.push_str(&format!("type = \"{ty}\"\n"));
        s.push_str(&format!(
            "axis = [{}, {}, {}]\n",
            f(p.axis[0]),
            f(p.axis[1]),
            f(p.axis[2])
        ));
        s.push_str(&format!(
            "anchor = [{}, {}, {}]\n",
            f(p.anchor[0]),
            f(p.anchor[1]),
            f(p.anchor[2])
        ));
        s.push_str(&format!(
            "child_anchor = [{}, {}, {}]\n",
            f(p.child_anchor[0]),
            f(p.child_anchor[1]),
            f(p.child_anchor[2])
        ));
        if let Some(lim) = &p.limits {
            s.push_str(&format!(
                "limits = {{ lower = {}, upper = {} }}\n",
                f(lim.lower),
                f(lim.upper)
            ));
        }
        if let Some(t) = p.max_torque {
            s.push_str(&format!("max_torque = {}\n", f(t)));
        }
        if let Some(v) = p.max_velocity {
            s.push_str(&format!("max_velocity = {}\n", f(v)));
        }
        if let Some(k) = p.kp {
            s.push_str(&format!("kp = {}\n", f(k)));
        }
        if let Some(k) = p.kd {
            s.push_str(&format!("kd = {}\n", f(k)));
        }
        if let Some(a) = p.default_angle {
            s.push_str(&format!("default_angle = {}\n", f(a)));
        }
    }
    s
}

/// 报告 → markdown(任务书 1.7 顺序:总质量/连杆数/自由度 → 自由度顺序
/// 表 → 合并清单 → 丢弃清单 → 忽略字段 → 形状换算表 → 惯量表 → 提示)。
pub fn report_to_markdown(report: &crate::ImportReport) -> String {
    let mut s = String::new();
    s.push_str("# 导入报告\n\n");
    s.push_str(&format!(
        "- 总质量:{:.6} kg\n- 连杆:{}\n- 自由度:{}\n",
        report.total_mass, report.links, report.dof
    ));
    s.push_str("\n## 自由度顺序\n\n| 下标 | 关节 |\n|---|---|\n");
    for (i, name) in report.dof_order.iter().enumerate() {
        s.push_str(&format!("| {i} | {name} |\n"));
    }
    s.push_str("\n## 合并的固定关节\n\n");
    if report.merged_fixed.is_empty() {
        s.push_str("(无)\n");
    } else {
        s.push_str("| 子连杆 | 并入 |\n|---|---|\n");
        for (child, into) in &report.merged_fixed {
            s.push_str(&format!("| {child} | {into} |\n"));
        }
    }
    s.push_str("\n## 丢弃的纯坐标系连杆\n\n");
    if report.dropped_frames.is_empty() {
        s.push_str("(无)\n");
    } else {
        for n in &report.dropped_frames {
            s.push_str(&format!("- {n}\n"));
        }
    }
    s.push_str("\n## 忽略的字段\n\n| 字段 | 次数 |\n|---|---|\n");
    for (name, count) in &report.ignored {
        s.push_str(&format!("| {name} | {count} |\n"));
    }
    s.push_str("\n## 形状换算\n\n| 连杆 | 来源 | 输出 | 源体积 | 输出体积 | 体积比 | 凸包点数 |\n|---|---|---|---|---|---|---|\n");
    for n in &report.shapes {
        let ratio = if n.volume_src > 0.0 {
            n.volume_out / n.volume_src
        } else {
            0.0
        };
        s.push_str(&format!(
            "| {} | {} | {} | {:.6} | {:.6} | {:.6} | {} |\n",
            n.link, n.from, n.to, n.volume_src, n.volume_out, ratio, n.hull_points
        ));
    }
    if !report.meshes.is_empty() {
        s.push_str(
            "\n## 网格\n\n| 连杆 | 文件 | 顶点 | 三角形 | 完整凸包顶点 | 输出顶点 | 网格体积 | 凸包体积 | 输出体积 | 凸包/网格 | 输出/凸包 |\n|---|---|---|---|---|---|---|---|---|---|---|\n",
        );
        for m in &report.meshes {
            let hull_over_mesh = if m.mesh_volume > 0.0 {
                m.hull_volume / m.mesh_volume
            } else {
                0.0
            };
            let out_over_hull = if m.hull_volume > 0.0 {
                m.out_volume / m.hull_volume
            } else {
                0.0
            };
            s.push_str(&format!(
                "| {} | {} | {} | {} | {} | {} | {:.6} | {:.6} | {:.6} | {:.6} | {:.6} |\n",
                m.link,
                m.file,
                m.vertices,
                m.triangles,
                m.hull_full,
                m.hull_out,
                m.mesh_volume,
                m.hull_volume,
                m.out_volume,
                hull_over_mesh,
                out_over_hull
            ));
        }
    }
    s.push_str("\n## 惯量\n\n| 连杆 | offdiag_ratio |\n|---|---|\n");
    for n in &report.inertia {
        s.push_str(&format!("| {} | {:.6} |\n", n.link, n.offdiag_ratio));
    }
    if !report.notes.is_empty() {
        s.push_str("\n## 提示\n\n");
        for n in &report.notes {
            s.push_str(&format!("- {n}\n"));
        }
    }
    s
}
