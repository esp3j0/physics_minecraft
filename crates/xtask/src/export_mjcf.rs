//! REQ-012 E02a §2:`BodyDef` → MJCF 导出(AC-E1b 同构部分)。
//!
//! 口径(任务书):真实单位 k = 1;坐标不转(y 向上,重力 `0 -9.81 0`,
//! 地面 `zaxis="0 1 0"`);质量/质心/惯量由内核 [`agent_link_mass_properties`]
//! 给出,显式写进 `<inertial>`。纯函数,同样输入逐字节相同。

use std::path::PathBuf;

use physics_sim::{ShardSimConfig, agent_link_mass_properties};
use types::{BodyDef, JointType, ShapeDef};

/// 导出选项(任务书 §2)。
#[derive(Debug, Clone)]
pub struct MjcfOptions {
    pub timestep: f64,
    pub floor_friction: f32,
    pub spawn_height: f32,
    pub joint_damping: f32,
}

impl Default for MjcfOptions {
    fn default() -> Self {
        Self {
            timestep: 0.002,
            floor_friction: 0.6,
            spawn_height: 0.35,
            joint_damping: 0.1,
        }
    }
}

/// 报告里一个连杆的质量属性。
#[derive(Debug, Clone)]
pub struct ReportLink {
    pub name: String,
    pub mass: f32,
    pub com: [f32; 3],
    pub inertia: [[f32; 3]; 3],
}

/// 报告里一个自由度。
#[derive(Debug, Clone)]
pub struct ReportDof {
    pub joint: String,
    pub child: String,
    /// MJCF 关节类型:`hinge` / `slide`。
    pub kind: String,
    pub axis: [f32; 3],
    pub lower: Option<f32>,
    pub upper: Option<f32>,
    pub max_torque: Option<f32>,
    pub kp: f32,
    pub kd: f32,
    pub default_angle: f32,
}

/// 导出报告(`--report` 写成 JSON,给 Python 核对脚本用)。
#[derive(Debug, Clone)]
pub struct MjcfReport {
    pub links: Vec<ReportLink>,
    pub dofs: Vec<ReportDof>,
    pub options: MjcfOptions,
    pub gravity: [f32; 3],
}

impl MjcfReport {
    /// 序列化成 JSON 值(xtask 无 serde derive 依赖,用 serde_json::json! 拼)。
    pub fn to_json(&self) -> serde_json::Value {
        let links: Vec<serde_json::Value> = self
            .links
            .iter()
            .map(|l| {
                serde_json::json!({
                    "name": l.name,
                    "mass": l.mass,
                    "com": l.com,
                    "inertia": l.inertia,
                })
            })
            .collect();
        let dofs: Vec<serde_json::Value> = self
            .dofs
            .iter()
            .map(|d| {
                serde_json::json!({
                    "joint": d.joint,
                    "child": d.child,
                    "type": d.kind,
                    "axis": d.axis,
                    "lower": d.lower,
                    "upper": d.upper,
                    "max_torque": d.max_torque,
                    "kp": d.kp,
                    "kd": d.kd,
                    "default_angle": d.default_angle,
                })
            })
            .collect();
        let o = &self.options;
        serde_json::json!({
            "links": links,
            "dofs": dofs,
            "options": {
                "timestep": o.timestep,
                "floor_friction": o.floor_friction,
                "spawn_height": o.spawn_height,
                "joint_damping": o.joint_damping,
            },
            "gravity": self.gravity,
        })
    }
}

fn gravity() -> [f32; 3] {
    let shard = types::ShardId::new(
        types::WorldId(0),
        types::DimensionId(0),
        types::ShardPos::new(0, 0),
    );
    let g = ShardSimConfig::new(shard).gravity;
    [g.x, g.y, g.z]
}

fn v3(v: [f32; 3]) -> String {
    format!("{} {} {}", v[0], v[1], v[2])
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn mjcf_joint_type(t: JointType) -> &'static str {
    match t {
        JointType::Revolute => "hinge",
        JointType::Prismatic => "slide",
        JointType::Fixed => "fixed",
    }
}

/// 先序遍历连杆树(根 = 从不作为 child 的连杆;同一父亲下的子按 joints 顺序)。
fn preorder(def: &BodyDef) -> Vec<String> {
    let mut child_names: Vec<&str> = def.joints.iter().map(|j| j.child.as_str()).collect();
    child_names.sort_unstable();
    let root = def
        .links
        .iter()
        .find(|l| child_names.binary_search(&l.name.as_str()).is_err())
        .expect("身体定义有根连杆");
    let mut order = Vec::new();
    let mut stack = vec![root.name.clone()];
    while let Some(name) = stack.pop() {
        order.push(name.clone());
        let children: Vec<&str> = def
            .joints
            .iter()
            .filter(|j| j.parent == name)
            .map(|j| j.child.as_str())
            .collect();
        for c in children.into_iter().rev() {
            stack.push(c.to_owned());
        }
    }
    order
}

fn incoming_joint<'a>(def: &'a BodyDef, child: &str) -> Option<&'a types::JointDef> {
    def.joints.iter().find(|j| j.child == child)
}

/// 一个形状 → geom 标签(连杆局部系;contype=1 conaffinity=0 只与地面碰)。
fn geom_xml(link: &types::LinkDef, si: usize, shape: &ShapeDef, indent: &str) -> String {
    let friction = format!("{} 0.005 0.0001", link.friction);
    let common = format!("contype=\"1\" conaffinity=\"0\" friction=\"{friction}\"");
    match shape {
        ShapeDef::Box { half, offset } => format!(
            "{indent}<geom type=\"box\" size=\"{}\" pos=\"{}\" {common}/>\n",
            v3(*half),
            v3(*offset)
        ),
        ShapeDef::Sphere { radius, offset } => format!(
            "{indent}<geom type=\"sphere\" size=\"{radius}\" pos=\"{}\" {common}/>\n",
            v3(*offset)
        ),
        ShapeDef::CapsuleY {
            half_height,
            radius,
            offset,
        } => {
            let from = [offset[0], offset[1] - half_height, offset[2]];
            let to = [offset[0], offset[1] + half_height, offset[2]];
            format!(
                "{indent}<geom type=\"capsule\" size=\"{radius}\" fromto=\"{} {}\" {common}/>\n",
                v3(from),
                v3(to)
            )
        }
        ShapeDef::ConvexHull { .. } => format!(
            "{indent}<geom type=\"mesh\" mesh=\"{}_s{si}\" {common}/>\n",
            link.name
        ),
    }
}

/// 导出 MJCF 文本与报告。纯函数,同样输入逐字节相同。
pub fn body_to_mjcf(def: &BodyDef, opts: &MjcfOptions) -> (String, MjcfReport) {
    let grav = gravity();
    let order = preorder(def);
    let props = agent_link_mass_properties(def);
    let prop_by_name: Vec<(&str, &physics_sim::LinkMassProps)> = def
        .links
        .iter()
        .zip(&props)
        .map(|(l, p)| (l.name.as_str(), p))
        .collect();
    let link_by_name = |name: &str| -> &types::LinkDef {
        def.links
            .iter()
            .find(|l| l.name == name)
            .expect("先序遍历的连杆都在定义里")
    };
    let props_of = |name: &str| -> &physics_sim::LinkMassProps {
        prop_by_name
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, p)| *p)
            .expect("连杆质量属性")
    };

    let mut s = String::new();
    s.push_str(&format!("<mujoco model=\"{}\">\n", def.name));
    s.push_str("  <compiler angle=\"radian\" autolimits=\"true\" inertiafromgeom=\"false\"/>\n");
    s.push_str(&format!(
        "  <option timestep=\"{}\" gravity=\"{}\"/>\n",
        opts.timestep,
        v3(grav)
    ));

    // asset:每个凸包形状一条 mesh(vertex = 顶点 + offset 平铺)。
    s.push_str("  <asset>\n");
    for name in &order {
        let link = link_by_name(name);
        for (si, shape) in link.shapes.iter().enumerate() {
            if let ShapeDef::ConvexHull { points, offset } = shape {
                let mut verts = Vec::with_capacity(points.len() * 3);
                for p in points {
                    verts.push(format!("{}", p[0] + offset[0]));
                    verts.push(format!("{}", p[1] + offset[1]));
                    verts.push(format!("{}", p[2] + offset[2]));
                }
                s.push_str(&format!(
                    "    <mesh name=\"{}_s{si}\" vertex=\"{}\"/>\n",
                    link.name,
                    verts.join(" ")
                ));
            }
        }
    }
    s.push_str("  </asset>\n");

    // worldbody:先序遍历递归写 body。
    s.push_str("  <worldbody>\n");
    s.push_str(&format!(
        "    <geom name=\"floor\" type=\"plane\" size=\"0 0 1\" zaxis=\"0 1 0\" friction=\"{} 0.005 0.0001\" contype=\"0\" conaffinity=\"1\"/>\n",
        opts.floor_friction
    ));
    write_body(def, opts, &order, &props, &mut s, 2);
    s.push_str("  </worldbody>\n");

    // actuator:每个自由度一条(自由度顺序)。
    s.push_str("  <actuator>\n");
    let mut dofs = Vec::new();
    for j in def.dof_joints() {
        let p = j.kind.params();
        let jt = mjcf_joint_type(j.kind.joint_type());
        let kp = p.kp.unwrap_or(0.0);
        let kd = p.kd.unwrap_or(0.0);
        let joint_name = format!("{}_joint", j.child);
        let mut act = format!(
            "    <position name=\"{}_act\" joint=\"{joint_name}\" kp=\"{kp}\" kv=\"{kd}\"",
            j.child
        );
        if let Some(tau) = p.max_torque {
            act.push_str(&format!(" forcerange=\"-{tau} {tau}\""));
        }
        if let Some(lim) = p.limits {
            act.push_str(&format!(" ctrlrange=\"{} {}\"", lim.lower, lim.upper));
        }
        act.push_str("/>\n");
        s.push_str(&act);
        dofs.push(ReportDof {
            joint: joint_name,
            child: j.child.clone(),
            kind: jt.to_owned(),
            axis: p.axis,
            lower: p.limits.map(|l| l.lower),
            upper: p.limits.map(|l| l.upper),
            max_torque: p.max_torque,
            kp,
            kd,
            default_angle: p.default_angle.unwrap_or(0.0),
        });
    }
    s.push_str("  </actuator>\n");

    // keyframe:stand = 根 pos/quat + 各自由度默认角。
    let defaults: Vec<f32> = def
        .dof_joints()
        .map(|j| j.kind.params().default_angle.unwrap_or(0.0))
        .collect();
    let dq: Vec<String> = defaults.iter().map(|d| format!("{d}")).collect();
    let qpos = format!("0 {} 0 1 0 0 0 {}", opts.spawn_height, dq.join(" "));
    s.push_str("  <keyframe>\n");
    s.push_str(&format!(
        "    <key name=\"stand\" qpos=\"{qpos}\" ctrl=\"{}\"/>\n",
        dq.join(" ")
    ));
    s.push_str("  </keyframe>\n");
    s.push_str("</mujoco>\n");

    let links = order
        .iter()
        .map(|name| {
            let p = props_of(name);
            ReportLink {
                name: name.clone(),
                mass: p.mass,
                com: p.com,
                inertia: p.inertia,
            }
        })
        .collect();
    let report = MjcfReport {
        links,
        dofs,
        options: opts.clone(),
        gravity: grav,
    };
    (s, report)
}

/// 先序遍历写 `<body>` 的上下文(游标按先序序列消费,子树紧跟父亲)。
struct BodyWriter<'a> {
    def: &'a BodyDef,
    opts: &'a MjcfOptions,
    order: &'a [String],
    /// 与 `def.links` 同序的质量属性。
    props: &'a [physics_sim::LinkMassProps],
    cur: usize,
}

impl BodyWriter<'_> {
    fn link(&self, name: &str) -> &types::LinkDef {
        self.def
            .links
            .iter()
            .find(|l| l.name == name)
            .expect("先序遍历的连杆都在定义里")
    }

    fn props_of(&self, name: &str) -> &physics_sim::LinkMassProps {
        let i = self
            .def
            .links
            .iter()
            .position(|l| l.name == name)
            .expect("连杆下标");
        &self.props[i]
    }

    fn rec(&mut self, s: &mut String, depth: usize) {
        let name = self.order[self.cur].clone();
        self.cur += 1;
        let link = self.link(&name);
        let p = self.props_of(&name);
        let ind = "  ".repeat(depth);
        let ind2 = "  ".repeat(depth + 1);
        let incoming = incoming_joint(self.def, &name);
        let pos = match incoming {
            None => format!("0 {} 0", self.opts.spawn_height),
            Some(j) => {
                let prm = j.kind.params();
                v3(sub(prm.anchor, prm.child_anchor))
            }
        };
        s.push_str(&format!("{ind}<body name=\"{name}\" pos=\"{pos}\">\n"));
        match incoming {
            None => s.push_str(&format!("{ind2}<freejoint name=\"root\"/>\n")),
            Some(j) => {
                let t = j.kind.joint_type();
                if t != JointType::Fixed {
                    let prm = j.kind.params();
                    let jt = mjcf_joint_type(t);
                    let mut joint = format!(
                        "{ind2}<joint name=\"{}_joint\" type=\"{jt}\" pos=\"{}\" axis=\"{}\"",
                        j.child,
                        v3(prm.child_anchor),
                        v3(prm.axis)
                    );
                    if let Some(lim) = prm.limits {
                        joint.push_str(&format!(" range=\"{} {}\"", lim.lower, lim.upper));
                    }
                    joint.push_str(&format!(" damping=\"{}\"/>\n", self.opts.joint_damping));
                    s.push_str(&joint);
                }
            }
        }
        let inertia = &p.inertia;
        s.push_str(&format!(
            "{ind2}<inertial pos=\"{}\" mass=\"{}\" fullinertia=\"{} {} {} {} {} {}\"/>\n",
            v3(p.com),
            p.mass,
            inertia[0][0],
            inertia[1][1],
            inertia[2][2],
            inertia[0][1],
            inertia[0][2],
            inertia[1][2]
        ));
        for (si, shape) in link.shapes.iter().enumerate() {
            s.push_str(&geom_xml(link, si, shape, &ind2));
        }
        let child_count = self.def.joints.iter().filter(|j| j.parent == name).count();
        for _ in 0..child_count {
            self.rec(s, depth + 1);
        }
        s.push_str(&format!("{ind}</body>\n"));
    }
}

/// 递归写一个连杆的 `<body>`(先序:自身 → 子连杆)。`depth` 是缩进层级。
fn write_body(
    def: &BodyDef,
    opts: &MjcfOptions,
    order: &[String],
    props: &[physics_sim::LinkMassProps],
    s: &mut String,
    depth: usize,
) {
    let mut w = BodyWriter {
        def,
        opts,
        order,
        props,
        cur: 0,
    };
    w.rec(s, depth);
}

/// `cargo xtask export-mjcf <body.toml> --out <xml> [--report <json>] [选项]`。
pub fn cmd_export_mjcf(args: &[String]) -> i32 {
    let mut body_path: Option<PathBuf> = None;
    let mut out: Option<PathBuf> = None;
    let mut report: Option<PathBuf> = None;
    let mut opts = MjcfOptions::default();
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        let mut value = || -> Option<String> { iter.next().cloned() };
        match arg.as_str() {
            "--out" => out = value().map(PathBuf::from),
            "--report" => report = value().map(PathBuf::from),
            "--timestep" => match value().and_then(|v| v.parse().ok()) {
                Some(v) => opts.timestep = v,
                None => {
                    eprintln!("--timestep 需要数值");
                    return 2;
                }
            },
            "--floor-friction" => match value().and_then(|v| v.parse().ok()) {
                Some(v) => opts.floor_friction = v,
                None => {
                    eprintln!("--floor-friction 需要数值");
                    return 2;
                }
            },
            "--spawn-height" => match value().and_then(|v| v.parse().ok()) {
                Some(v) => opts.spawn_height = v,
                None => {
                    eprintln!("--spawn-height 需要数值");
                    return 2;
                }
            },
            "--joint-damping" => match value().and_then(|v| v.parse().ok()) {
                Some(v) => opts.joint_damping = v,
                None => {
                    eprintln!("--joint-damping 需要数值");
                    return 2;
                }
            },
            other => {
                if other.starts_with("--") {
                    eprintln!("未知参数 `{other}`");
                    return 2;
                }
                if body_path.is_some() {
                    eprintln!("只接受一个 body.toml 位置参数");
                    return 2;
                }
                body_path = Some(PathBuf::from(other));
            }
        }
    }
    let Some(body_path) = body_path else {
        eprintln!("用法:cargo xtask export-mjcf <body.toml> --out <xml> [--report <json>]");
        return 2;
    };
    let Some(out) = out else {
        eprintln!("缺少 --out <xml>");
        return 2;
    };
    let text = match std::fs::read_to_string(&body_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("读 {} 失败:{e}", body_path.display());
            return 2;
        }
    };
    let def = match BodyDef::from_toml_str(&text) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("身体定义错误:{e}");
            return 2;
        }
    };
    let (xml, rep) = body_to_mjcf(&def, &opts);
    if let Err(e) = std::fs::write(&out, xml) {
        eprintln!("写 {} 失败:{e}", out.display());
        return 2;
    }
    println!("MJCF:{}", out.display());
    if let Some(rp) = report {
        let json = match serde_json::to_string_pretty(&rep.to_json()) {
            Ok(j) => j,
            Err(e) => {
                eprintln!("报告序列化失败:{e}");
                return 2;
            }
        };
        if let Err(e) = std::fs::write(&rp, json) {
            eprintln!("写 {} 失败:{e}", rp.display());
            return 2;
        }
        println!("报告:{}", rp.display());
    }
    0
}
