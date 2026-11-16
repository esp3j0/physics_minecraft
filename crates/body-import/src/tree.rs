//! 连杆树校验、遍历顺序、固定关节合并、`BodyDef` 组装
//! (REQ-012 C01a 任务书 1.4、1.6)。
//!
//! 换算统一口径(任务书 1.3):每个 URDF 连杆先算出它在 URDF 根系里的
//! 原点 `p_L`(`p_C = p_P + A_P · xyz_J`)与朝向 `A_L`(`A_C = A_P · R(rpy_J)`);
//! `W_L = G · A_L` 把连杆局部系向量变到 BodyDef 系。所有 BodyDef 量都能
//! 写成 `G · (根系量 − p_组头)`:固定关节合并、anchor、com 因此都只是
//! 根系平移差(任务书 1.4"合并只是平移"的等价形式)。

use std::collections::BTreeMap;

use crate::frames::{clean, clean_vec, mat_mul, mat_vec, normalize, rpy_to_mat, up_axis_matrix};
use crate::inertia::{Tensor, rotate, shift};
use crate::report::{ImportOptions, ImportReport, InertiaNote, MeshNote, ShapeNote};
use crate::shapes;

/// 树中每个 URDF 连杆的换算结果。
struct Node {
    name: String,
    /// 原点在 URDF 根系里的位置。
    p_root: [f64; 3],
    /// 累计旋转(URDF 连杆系 → URDF 根系)。
    a_l: crate::frames::Mat3,
    /// `W_L = G · A_L`(连杆系 → BodyDef 系)。
    w_l: crate::frames::Mat3,
    /// URDF inertial。探针实证:urdf-rs 0.10 把缺 `<inertial>` 静默默认成
    /// mass=0 + 张量全零;真实模型质量不为 0,据此判"缺失"。
    inertial: Option<urdf_rs::Inertial>,
    /// 碰撞形状(已换算到 BodyDef 系;offset 尚未加合并平移)。
    shapes: Vec<(types::ShapeDef, ShapeNote, Option<MeshNote>)>,
}

/// 合并组(一个输出连杆)。
struct Group {
    /// 输出连杆名 = 组内最靠近根的 URDF 连杆名。
    head: String,
    /// (成员连杆下标, 质心在 BodyDef 系里的位置)。
    members: Vec<(usize, [f64; 3])>,
    /// 形状(offset 已平移)。
    shapes: Vec<(types::ShapeDef, ShapeNote, Option<MeshNote>)>,
    any_inertial: bool,
}

/// 关节类型分类。`urdf-rs` 0.10 把缺 `<limit>` 静默默认 lower=upper=0
/// (探针实证);合法模型不会给活动关节写 [0,0] 限位,以此判"缺 limit"。
enum Kind {
    RevoluteLimited,
    Continuous,
    PrismaticLimited,
    Fixed,
}

fn kind_of(joint: &urdf_rs::Joint) -> Result<Kind, crate::ImportError> {
    let l = &joint.limit;
    let limit_absent = l.lower == 0.0 && l.upper == 0.0;
    Ok(match &joint.joint_type {
        urdf_rs::JointType::Revolute if limit_absent => {
            return Err(crate::ImportError::MissingLimit {
                joint: joint.name.clone(),
            });
        }
        urdf_rs::JointType::Prismatic if limit_absent => {
            return Err(crate::ImportError::MissingLimit {
                joint: joint.name.clone(),
            });
        }
        urdf_rs::JointType::Revolute => Kind::RevoluteLimited,
        urdf_rs::JointType::Continuous => Kind::Continuous,
        urdf_rs::JointType::Prismatic => Kind::PrismaticLimited,
        urdf_rs::JointType::Fixed => Kind::Fixed,
        other => {
            return Err(crate::ImportError::UnsupportedJoint {
                joint: joint.name.clone(),
                kind: format!("{other:?}"),
            });
        }
    })
}

/// inertial 是否实际缺失(urdf-rs 静默默认 = mass 0 + 张量全零)。
fn inertial_missing(inertial: &urdf_rs::Inertial) -> bool {
    let i = &inertial.inertia;
    inertial.mass.value == 0.0
        && i.ixx == 0.0
        && i.ixy == 0.0
        && i.ixz == 0.0
        && i.iyy == 0.0
        && i.iyz == 0.0
        && i.izz == 0.0
}

pub(crate) fn import_impl(
    urdf_xml: &str,
    base_dir: &std::path::Path,
    opts: &ImportOptions,
) -> Result<(types::BodyDef, ImportReport), crate::ImportError> {
    // ---- 解析前去掉 <visual>(C02 任务书 1.1;未知关节类型字符串在 urdf-rs 层被拒绝 → Xml)----
    let (stripped, visual_count) =
        crate::prefilter::strip_visuals(urdf_xml).map_err(crate::ImportError::Xml)?;
    let robot =
        urdf_rs::read_from_string(&stripped).map_err(|e| crate::ImportError::Xml(e.to_string()))?;
    if robot.links.is_empty() {
        return Err(crate::ImportError::NoLinks);
    }

    // ---- 选项校验 ----
    if !(8..=64).contains(&opts.cylinder_segments) {
        return Err(crate::ImportError::BadOption(format!(
            "cylinder_segments = {} 超出范围 8..=64",
            opts.cylinder_segments
        )));
    }
    if !(4..=255).contains(&opts.max_hull_points) {
        return Err(crate::ImportError::BadOption(format!(
            "max_hull_points = {} 超出范围 4..=255",
            opts.max_hull_points
        )));
    }
    for (k, v) in [
        ("kp", opts.kp),
        ("kd", opts.kd),
        ("friction", opts.friction),
    ] {
        if !v.is_finite() || v < 0.0 {
            return Err(crate::ImportError::BadOption(format!("{k} = {v} 非法")));
        }
    }

    // ---- 树校验(任务书 1.4:先报先停)----
    let mut by_name: BTreeMap<&str, usize> = BTreeMap::new();
    for (i, link) in robot.links.iter().enumerate() {
        if by_name.insert(link.name.as_str(), i).is_some() {
            return Err(crate::ImportError::Cycle(vec![link.name.clone()]));
        }
    }
    let mut parent_of: BTreeMap<&str, usize> = BTreeMap::new(); // child 连杆名 → 关节下标
    for (ji, joint) in robot.joints.iter().enumerate() {
        if !by_name.contains_key(joint.parent.link.as_str()) {
            return Err(crate::ImportError::UnknownLink {
                joint: joint.name.clone(),
                link: joint.parent.link.clone(),
            });
        }
        if !by_name.contains_key(joint.child.link.as_str()) {
            return Err(crate::ImportError::UnknownLink {
                joint: joint.name.clone(),
                link: joint.child.link.clone(),
            });
        }
        if parent_of.insert(joint.child.link.as_str(), ji).is_some() {
            return Err(crate::ImportError::Cycle(vec![joint.child.link.clone()]));
        }
    }
    let roots: Vec<usize> = (0..robot.links.len())
        .filter(|&i| !parent_of.contains_key(robot.links[i].name.as_str()))
        .collect();
    if roots.is_empty() {
        // 没有根说明成环(任务书 1.4)。
        return Err(crate::ImportError::Cycle(
            robot.links.iter().map(|l| l.name.clone()).collect(),
        ));
    }
    if roots.len() > 1 {
        return Err(crate::ImportError::MultipleRoots(
            roots.iter().map(|&i| robot.links[i].name.clone()).collect(),
        ));
    }
    // 先序 DFS;同一父下按关节文件序;走不到 → Cycle(独立环)。
    let mut children_joints: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
    for (ji, joint) in robot.joints.iter().enumerate() {
        children_joints
            .entry(joint.parent.link.as_str())
            .or_default()
            .push(ji);
    }
    let mut order: Vec<usize> = Vec::new();
    let mut stack = vec![roots[0]];
    while let Some(&i) = stack.last() {
        stack.pop();
        order.push(i);
        let name = robot.links[i].name.as_str();
        if let Some(js) = children_joints.get(name) {
            // 栈后进先出:倒序入栈,弹出仍是文件序。
            for &ji in js.iter().rev() {
                stack.push(by_name[robot.joints[ji].child.link.as_str()]);
            }
        }
    }
    if order.len() != robot.links.len() {
        let reached: std::collections::BTreeSet<&str> = order
            .iter()
            .map(|&i| robot.links[i].name.as_str())
            .collect();
        return Err(crate::ImportError::Cycle(
            robot
                .links
                .iter()
                .map(|l| l.name.clone())
                .filter(|n| !reached.contains(n.as_str()))
                .collect(),
        ));
    }

    // ---- default_angles 校验(URDF 没有的关节名、fixed 关节 → BadOption)----
    for name in opts.default_angles.keys() {
        match robot.joints.iter().find(|j| &j.name == name) {
            None => {
                return Err(crate::ImportError::BadOption(format!(
                    "default_angles 里的关节 `{name}` 在 URDF 中不存在"
                )));
            }
            Some(j) if j.joint_type == urdf_rs::JointType::Fixed => {
                return Err(crate::ImportError::BadOption(format!(
                    "default_angles 指向 fixed 关节 `{name}`"
                )));
            }
            Some(_) => {}
        }
    }

    // ---- 逐连杆换算(先序:父先于子)----
    let g = up_axis_matrix(opts.up_axis);
    let mut nodes: Vec<Node> = Vec::with_capacity(robot.links.len());
    for _ in 0..robot.links.len() {
        nodes.push(Node {
            name: String::new(),
            p_root: [0.0; 3],
            a_l: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            w_l: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            inertial: None,
            shapes: Vec::new(),
        });
    }
    for &i in &order {
        let link = &robot.links[i];
        // 父的数据先取(避免与 node 的可变借用冲突;先序保证父已换算)。
        let inherited = parent_of.get(link.name.as_str()).map(|&ji| {
            let joint = &robot.joints[ji];
            let p = by_name[joint.parent.link.as_str()];
            (joint, nodes[p].a_l, nodes[p].p_root)
        });
        let node = &mut nodes[i];
        node.name = link.name.clone();
        if let Some((joint, p_a, p_origin)) = inherited {
            let r_j = rpy_to_mat(joint.origin.rpy.0);
            node.a_l = mat_mul(&p_a, &r_j);
            node.p_root = crate::frames::vec_add(p_origin, mat_vec(&p_a, joint.origin.xyz.0));
        }
        node.w_l = mat_mul(&g, &node.a_l);
        let w = node.w_l;
        let raw = &link.inertial;
        if !inertial_missing(raw) {
            // 质量与张量的有限性在组合成段校验(BadMass / BadInertia)。
            node.inertial = Some(raw.clone());
        }
        let mesh_ctx = shapes::MeshCtx {
            base_dir,
            packages: &opts.package_dirs,
            max_points: opts.max_hull_points,
        };
        for col in &link.collision {
            let r_s = mat_mul(&w, &rpy_to_mat(col.origin.rpy.0));
            let offset = clean_vec(mat_vec(&w, col.origin.xyz.0));
            let converted = shapes::convert(
                &link.name,
                &col.geometry,
                offset,
                &r_s,
                opts.cylinder_segments,
                &mesh_ctx,
            )?;
            node.shapes
                .push((converted.shape, converted.note, converted.mesh));
        }
    }

    // ---- 合并组:组头 = 沿父链第一个不经 fixed 关节相连的祖先 ----
    let mut head_of = vec![0usize; robot.links.len()];
    let mut d_of: Vec<[f64; 3]> = vec![[0.0; 3]; robot.links.len()];
    for &i in &order {
        match parent_of.get(robot.links[i].name.as_str()) {
            None => head_of[i] = i,
            Some(&ji) => {
                if robot.joints[ji].joint_type == urdf_rs::JointType::Fixed {
                    let p = by_name[robot.joints[ji].parent.link.as_str()];
                    head_of[i] = head_of[p];
                    // d = G · (p_i − p_M)(见模块注释)。
                    d_of[i] = mat_vec(
                        &g,
                        crate::frames::vec_add(nodes[i].p_root, neg(nodes[head_of[i]].p_root)),
                    );
                } else {
                    head_of[i] = i;
                }
            }
        }
    }

    // ---- 组装组(先序;组按组头首次出现序)----
    let mut groups: Vec<Group> = Vec::new();
    let mut group_of_head: BTreeMap<usize, usize> = BTreeMap::new();
    let mut merged_fixed: Vec<(String, String)> = Vec::new();
    let mut dropped_frames: Vec<String> = Vec::new();
    for &i in &order {
        let node = &nodes[i];
        let head = head_of[i];
        let is_head = head == i;
        let gi;
        if is_head {
            gi = groups.len();
            group_of_head.insert(i, gi);
            groups.push(Group {
                head: node.name.clone(),
                members: Vec::new(),
                shapes: Vec::new(),
                any_inertial: false,
            });
        } else {
            gi = group_of_head[&head];
            // 非组头的固定子连杆三路分类(任务书 1.4,互斥):纯坐标系
            // (无 inertial 无碰撞)→ dropped_frames;其余 → merged_fixed。
            // 组头本身是输出连杆(名字保留),不进任何清单;空组头稍后由
            // MissingInertial 兜住。
            if node.inertial.is_none() && node.shapes.is_empty() {
                dropped_frames.push(node.name.clone());
                continue;
            }
            merged_fixed.push((node.name.clone(), groups[gi].head.clone()));
        }
        let d = d_of[i];
        let group = &mut groups[gi];
        if let Some(inertial) = &node.inertial {
            // 质心(任务书 1.3):com = W_L · xyz_inertial + d。
            let com = crate::frames::vec_add(mat_vec(&node.w_l, inertial.origin.xyz.0), d);
            group.members.push((i, com));
            group.any_inertial = true;
            // 质量合法性:≤ 0 → BadMass(0 + 全零张量已在上面按缺失分流)。
            if inertial.mass.value <= 0.0 || !inertial.mass.value.is_finite() {
                return Err(crate::ImportError::BadMass {
                    link: node.name.clone(),
                    mass: inertial.mass.value as f32,
                });
            }
        }
        for (s, n, m) in &node.shapes {
            let mut s = s.clone();
            translate_shape(&mut s, d);
            group.shapes.push((s, n.clone(), m.clone()));
        }
    }

    // ---- 每组:质量/质心/惯量合并、校验、LinkDef ----
    let mut out_links: Vec<types::LinkDef> = Vec::new();
    let mut shape_notes: Vec<ShapeNote> = Vec::new();
    let mut mesh_notes: Vec<MeshNote> = Vec::new();
    let mut inertia_notes: Vec<InertiaNote> = Vec::new();
    let mut total_mass = 0.0f32;
    for group in &groups {
        if !group.any_inertial {
            return Err(crate::ImportError::MissingInertial {
                link: group.head.clone(),
            });
        }
        // m = Σ mᵢ;com = Σ mᵢcᵢ / m(任务书 1.6)。
        let mut m = 0.0f64;
        let mut weighted = [0.0f64; 3];
        for (mi, com) in &group.members {
            let mass = nodes[*mi]
                .inertial
                .as_ref()
                .expect("成员有 inertial")
                .mass
                .value;
            m += mass;
            weighted =
                crate::frames::vec_add(weighted, [mass * com[0], mass * com[1], mass * com[2]]);
        }
        let com = [weighted[0] / m, weighted[1] / m, weighted[2] / m];
        // 逐成员:张量旋转到 BodyDef 系(W_L · R(rpy_inertial)),对角校验,
        // 平行轴搬到组质心后求和(任务书 1.6)。
        let mut total = Tensor([[0.0; 3]; 3]);
        for (mi, com_i) in &group.members {
            let inertial = nodes[*mi].inertial.as_ref().expect("成员有 inertial");
            let raw = Tensor([
                [
                    inertial.inertia.ixx,
                    inertial.inertia.ixy,
                    inertial.inertia.ixz,
                ],
                [
                    inertial.inertia.ixy,
                    inertial.inertia.iyy,
                    inertial.inertia.iyz,
                ],
                [
                    inertial.inertia.ixz,
                    inertial.inertia.iyz,
                    inertial.inertia.izz,
                ],
            ]);
            for row in &raw.0 {
                for v in row {
                    if !v.is_finite() {
                        return Err(crate::ImportError::BadInertia {
                            link: group.head.clone(),
                        });
                    }
                }
            }
            let r_full = mat_mul(&nodes[*mi].w_l, &rpy_to_mat(inertial.origin.rpy.0));
            let rotated = rotate(&raw, &r_full);
            for (dg, row) in rotated.0.iter().enumerate() {
                if !row[dg].is_finite() || row[dg] <= 0.0 {
                    return Err(crate::ImportError::BadInertia {
                        link: group.head.clone(),
                    });
                }
            }
            let rel = [com_i[0] - com[0], com_i[1] - com[1], com_i[2] - com[2]];
            total.0 = add_t(&total.0, &shift(&rotated, inertial.mass.value, rel).0);
        }
        // offdiag_ratio(任务书 1.2):惯性积绝对值最大者 / 对角元最小者。
        let diag_min = total.0[0][0].min(total.0[1][1]).min(total.0[2][2]);
        let offdiag_max = total.0[0][1]
            .abs()
            .max(total.0[0][2].abs())
            .max(total.0[1][0].abs())
            .max(total.0[1][2].abs())
            .max(total.0[2][0].abs())
            .max(total.0[2][1].abs());
        let ratio = if diag_min > 0.0 {
            (offdiag_max / diag_min) as f32
        } else {
            0.0
        };
        inertia_notes.push(InertiaNote {
            link: group.head.clone(),
            offdiag_ratio: ratio,
        });
        if group.shapes.is_empty() {
            return Err(crate::ImportError::NoCollision {
                link: group.head.clone(),
            });
        }
        if group.shapes.len() > 8 {
            return Err(crate::ImportError::TooManyShapes {
                link: group.head.clone(),
                count: group.shapes.len(),
            });
        }
        for (_, n, m) in &group.shapes {
            shape_notes.push(n.clone());
            if let Some(mn) = m {
                mesh_notes.push(mn.clone());
            }
        }
        out_links.push(types::LinkDef {
            name: group.head.clone(),
            mass: m as f32,
            com: clean_vec(com),
            inertia: Some([
                clean(total.0[0][0]),
                clean(total.0[1][1]),
                clean(total.0[2][2]),
            ]),
            friction: opts.friction,
            restitution: 0.0,
            shapes: group.shapes.iter().map(|(s, ..)| s.clone()).collect(),
        });
        total_mass += m as f32;
    }

    // ---- 输出关节(组树;joints 与 links[1..] 同序)----
    // 子组 → (父组, 关节下标)。anchor = G · (p_关节原点 − p_父组头)
    // (等价于任务书 1.4 的"parent 改 M、anchor += d",见模块注释);
    // 轴 = W_C · axis_J、归一化(fixed 写 [1,0,0])。
    let mut parent_group_of_group: BTreeMap<usize, usize> = BTreeMap::new();
    let mut joint_of_group: BTreeMap<usize, usize> = BTreeMap::new();
    for (gi, group) in groups.iter().enumerate() {
        let head_i = by_name[group.head.as_str()];
        if let Some(&ji) = parent_of.get(robot.links[head_i].name.as_str()) {
            let p = by_name[robot.joints[ji].parent.link.as_str()];
            let pgi = group_of_head[&head_of[p]];
            if pgi != gi {
                parent_group_of_group.insert(gi, pgi);
                joint_of_group.insert(gi, ji);
            }
            // pgi == gi 不可能:关节 parent 与 child 不在同一组(组内全是
            // fixed 链,而本关节把 head 当 child,是非 fixed 关节)。
        }
    }
    let root_gi = group_of_head[&head_of[roots[0]]];
    // 先序 DFS 收集组序列(组下标 = 先序,直接按下标升序走;父组先于子组)。
    // 输出关节 = 每个非根组各一条,按子组先序 append → joints 与 links[1..]
    // 同序、同一父下按文件序(组先序由连杆先序决定,连杆先序按关节文件序)。
    debug_assert!(root_gi == 0);
    let mut out_joints: Vec<types::JointDef> = Vec::new();
    let mut dof_order: Vec<String> = Vec::new();
    for cgi in 0..groups.len() {
        if cgi == root_gi {
            continue;
        }
        let ji = joint_of_group[&cgi];
        let joint = &robot.joints[ji];
        let child_i = by_name[joint.child.link.as_str()];
        let p_head_i = by_name[groups[parent_group_of_group[&cgi]].head.as_str()];
        // anchor = G · (p_child − p_父组头)。
        let anchor = mat_vec(
            &g,
            crate::frames::vec_add(nodes[child_i].p_root, neg(nodes[p_head_i].p_root)),
        );
        let def = make_joint_def(
            joint,
            &nodes[child_i].w_l,
            anchor,
            opts,
            groups[parent_group_of_group[&cgi]].head.clone(),
            groups[cgi].head.clone(),
        )?;
        if def.kind.joint_type() != types::JointType::Fixed {
            dof_order.push(joint.name.clone());
        }
        out_joints.push(def);
    }

    let def = types::BodyDef {
        name: opts.name.clone(),
        lock_rotations: false,
        links: out_links,
        joints: out_joints,
        self_collision: types::SelfCollisionDef::default(),
        walker: None,
    };
    def.validate().map_err(crate::ImportError::Validate)?;

    // 网格提示(C01b 任务书 1.6):明显非凸 / 体积为 0。
    let mut notes = vec!["当前内核生成主体时不使用 com 和 inertia,惯量由碰撞形状推出。".to_owned()];
    if visual_count > 0 {
        notes.push("视觉元素(visual)在解析前已去掉,不参与导入。".to_owned());
    }
    for m in &mesh_notes {
        if m.mesh_volume > 0.0 && m.hull_volume / m.mesh_volume > 1.1 {
            notes.push(format!(
                "连杆 {} 的网格 {} 明显非凸,凸包体积是网格的 {:.2} 倍;不做凸分解。",
                m.link,
                m.file,
                m.hull_volume / m.mesh_volume
            ));
        }
        if m.mesh_volume == 0.0 {
            notes.push(format!(
                "连杆 {} 的网格 {} 体积为 0(没有面,或面不闭合恰好抵消),体积偏差无法给出。",
                m.link, m.file
            ));
        }
    }
    let report = ImportReport {
        total_mass,
        links: def.links.len(),
        dof: dof_order.len(),
        dof_order,
        merged_fixed,
        dropped_frames,
        ignored: collect_ignored(urdf_xml, &robot, visual_count),
        shapes: shape_notes,
        meshes: mesh_notes,
        inertia: inertia_notes,
        notes,
    };
    Ok((def, report))
}

fn neg(v: [f64; 3]) -> [f64; 3] {
    [-v[0], -v[1], -v[2]]
}

fn add_t(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][j] + b[i][j];
        }
    }
    out
}

/// 平移一个 ShapeDef 的 offset(固定关节合并:offset += d,任务书 1.4)。
fn translate_shape(s: &mut types::ShapeDef, d: [f64; 3]) {
    let off = match s {
        types::ShapeDef::Box { offset, .. }
        | types::ShapeDef::Sphere { offset, .. }
        | types::ShapeDef::CapsuleY { offset, .. }
        | types::ShapeDef::ConvexHull { offset, .. } => offset,
    };
    *off = clean_vec([
        f64::from(off[0]) + d[0],
        f64::from(off[1]) + d[1],
        f64::from(off[2]) + d[2],
    ]);
}

/// URDF 关节 → BodyDef JointDef(任务书 1.4)。
fn make_joint_def(
    joint: &urdf_rs::Joint,
    w_child: &crate::frames::Mat3,
    anchor: [f64; 3],
    opts: &ImportOptions,
    parent: String,
    child: String,
) -> Result<types::JointDef, crate::ImportError> {
    let kind = kind_of(joint)?;
    let (joint_type, limits) = match kind {
        Kind::RevoluteLimited | Kind::PrismaticLimited => {
            let l = &joint.limit;
            let t = if joint.joint_type == urdf_rs::JointType::Revolute {
                types::JointType::Revolute
            } else {
                types::JointType::Prismatic
            };
            (
                t,
                Some(types::JointLimits {
                    lower: clean(l.lower),
                    upper: clean(l.upper),
                }),
            )
        }
        // continuous → Revolute、limits = None(任务书 1.4)。
        Kind::Continuous => (types::JointType::Revolute, None),
        Kind::Fixed => (types::JointType::Fixed, None),
    };
    // 轴(任务书 1.3):W_C · axis_J,归一化;fixed 写 [1, 0, 0]。
    // 变异自证 4:用 W_P 会被 hip 轴(父系 (0,0,−1) ≠ 子系 (−1,0,0))抓红。
    let axis = if joint_type == types::JointType::Fixed {
        [1.0f64, 0.0, 0.0]
    } else {
        let a = &joint.axis.xyz.0;
        if a.iter().any(|v| !v.is_finite()) || a.iter().all(|v| *v == 0.0) {
            return Err(crate::ImportError::BadAxis {
                joint: joint.name.clone(),
            });
        }
        normalize(mat_vec(w_child, *a))
    };
    // default_angle:opts.default_angles[URDF 关节名],无则 0,再夹进限位。
    let raw = opts.default_angles.get(&joint.name).copied().unwrap_or(0.0);
    let default_angle = match limits {
        Some(l) => raw.clamp(l.lower, l.upper),
        None => raw,
    };
    // max_torque = effort、max_velocity = velocity(> 0 才写);
    // kp/kd 写进每个非 fixed 关节(fixed 全 None)。
    let (max_torque, max_velocity, kp, kd, default_angle) = if joint_type == types::JointType::Fixed
    {
        (None, None, None, None, None)
    } else {
        (
            (joint.limit.effort > 0.0 && joint.limit.effort.is_finite())
                .then(|| clean(joint.limit.effort)),
            (joint.limit.velocity > 0.0 && joint.limit.velocity.is_finite())
                .then(|| clean(joint.limit.velocity)),
            Some(opts.kp),
            Some(opts.kd),
            Some(default_angle),
        )
    };
    let params = types::JointParams {
        axis: clean_vec(axis),
        anchor: clean_vec(anchor),
        child_anchor: [0.0; 3],
        limits,
        max_torque,
        max_velocity,
        kp,
        kd,
        default_angle,
    };
    Ok(types::JointDef {
        parent,
        child,
        kind: match joint_type {
            types::JointType::Revolute => types::JointKind::Revolute(params),
            types::JointType::Prismatic => types::JointKind::Prismatic(params),
            types::JointType::Fixed => types::JointKind::Fixed(params),
        },
    })
}

/// ignored 字段统计(任务书 1.7,C02 1.1 改口径):visual 按预处理去掉的
/// 块数;material 按原文里 `<material` 子串出现次数(顶层和 visual 里的都算);
/// 关节 dynamics、mimic、safety_controller、calibration 各按出现次数;
/// transmission、gazebo 这两类 urdf-rs 不解析的元素在原文里按子串计数。
fn collect_ignored(urdf_xml: &str, robot: &urdf_rs::Robot, visual_count: usize) -> Vec<(String, usize)> {
    let mut out: Vec<(String, usize)> = Vec::new();
    let mut push = |k: &str, n: usize| {
        if n > 0 {
            out.push((k.to_owned(), n));
        }
    };
    push("visual", visual_count);
    push("material", urdf_xml.matches("<material").count());
    let mut dynamics = 0;
    let mut mimic = 0;
    let mut safety = 0;
    let mut calibration = 0;
    for j in &robot.joints {
        if j.dynamics.is_some() {
            dynamics += 1;
        }
        if j.mimic.is_some() {
            mimic += 1;
        }
        if j.safety_controller.is_some() {
            safety += 1;
        }
        if j.calibration.is_some() {
            calibration += 1;
        }
    }
    push("dynamics", dynamics);
    push("mimic", mimic);
    push("safety_controller", safety);
    push("calibration", calibration);
    push("transmission", urdf_xml.matches("<transmission").count());
    push("gazebo", urdf_xml.matches("<gazebo").count());
    out
}
