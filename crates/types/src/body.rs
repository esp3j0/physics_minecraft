//! 身体定义 `BodyDef`(REQ-012 §2.1,A01):只读数据资产,服务端启动时从
//! `assets/bodies/*.body.toml` 加载,按**内容哈希**标识(REQ-012 §3:客户端
//! 只能按名字引用,不能上传)。
//!
//! - 解析:toml → 结构体(真实单位:千克、米、牛·米);未知关节类型/形状
//!   类别在 serde 层拒绝(toml → `BodyDefError::Parse`);
//! - 校验:`validate`(类型化错误、不 panic):树形(拒绝闭环/多根/父不存在)、
//!   每连杆 ≤ 8 个碰撞形状、质量 > 0、全部数值有限、关节上限 < 下限拒绝;
//! - 哈希:**对解析后的规范形式**求 FNV-1a 64 位(`canonical_bytes` 显式
//!   字段定序、`f32::to_bits` 逐字节哈希)——注释、空白、键序变化不影响
//!   哈希,任一数值变化必改哈希。FNV-1a 纯字节流运算,与平台字节序无关,
//!   跨平台稳定;自实现零新增依赖。
//!
//! 本模块不含 IO(目录加载在 app 装配层)。

use serde::{Deserialize, Serialize};

/// 身体内容哈希(解析后规范形式的 FNV-1a 64 位,§2.1)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct BodyHash(pub [u8; 8]);

impl BodyHash {
    /// FNV-1a 64 位(offset `0xcbf29ce484222325`,prime `0x100000001b3`,
    /// u64 wrapping 乘加;逐字节,与平台字节序无关)。
    pub fn fnv1a(bytes: &[u8]) -> Self {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in bytes {
            h ^= u64::from(*b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        Self(h.to_be_bytes())
    }

    /// 16 位十六进制小写字符串(日志/遥测展示)。
    pub fn hex(&self) -> String {
        self.0.iter().map(|b| format!("{b:02x}")).collect()
    }
}

impl std::fmt::Display for BodyHash {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.hex())
    }
}

/// 碰撞形状定义(REQ-012 §2.1:盒、球、胶囊、凸包)。
/// serde 内部 tag(`kind`):未知类别在解析层拒绝;每连杆形状数上限 8(校验层)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ShapeDef {
    /// 轴对齐盒(半边长)。
    Box { half: [f32; 3] },
    /// 球。
    Sphere { radius: f32 },
    /// y 向胶囊(总高 = 2·(half_height + radius))。
    CapsuleY { half_height: f32, radius: f32 },
    /// 凸包(顶点集;导入工具生成)。
    ConvexHull { points: Vec<[f32; 3]> },
}

/// 关节类型(REQ-012 §2.1;未知类型在 serde 层拒绝)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointType {
    Revolute,
    Prismatic,
    Fixed,
}

/// 关节上下限(rad 或 m,按类型)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct JointLimits {
    pub lower: f32,
    pub upper: f32,
}

/// 关节参数(三种类型共用;真实单位,生成时按 §4 乘全局系数 k)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointParams {
    pub axis: [f32; 3],
    pub anchor: [f32; 3],
    /// 上限 < 下限 → `BodyDefError::BadLimits`(校验层)。
    pub limits: Option<JointLimits>,
    /// 力矩上限(N·m;真实单位)。
    pub max_torque: Option<f32>,
    /// 速度上限(rad/s 或 m/s)。
    pub max_velocity: Option<f32>,
    /// PD 增益(§4:生成时与质量乘同一系数 k)。
    pub kp: Option<f32>,
    pub kd: Option<f32>,
    /// 默认角度(§3 迁移重建写回目标用)。
    pub default_angle: Option<f32>,
}

/// 关节类型与参数(内部 tag `type`;未知类型字符串 → 解析错误)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum JointKind {
    Revolute(JointParams),
    Prismatic(JointParams),
    Fixed(JointParams),
}

impl JointKind {
    pub fn joint_type(&self) -> JointType {
        match self {
            JointKind::Revolute(_) => JointType::Revolute,
            JointKind::Prismatic(_) => JointType::Prismatic,
            JointKind::Fixed(_) => JointType::Fixed,
        }
    }

    pub fn params(&self) -> &JointParams {
        match self {
            JointKind::Revolute(p) | JointKind::Prismatic(p) | JointKind::Fixed(p) => p,
        }
    }
}

/// 关节定义(REQ-012 §2.1:父子连杆、类型、轴、锚点、上下限、力矩上限、
/// 速度上限、PD 增益、默认角度)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointDef {
    pub parent: String,
    pub child: String,
    #[serde(flatten)]
    pub kind: JointKind,
}

/// 自碰撞定义(REQ-012 §2.1:默认关闭,可按连杆对开启;相邻连杆一律关闭)。
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SelfCollisionDef {
    /// 额外开启接触的连杆对(相邻对不生效——生成层强制 `contacts_enabled(false)`)。
    #[serde(default)]
    pub extra_pairs: Vec<[String; 2]>,
}

/// 连杆定义(REQ-012 §2.1:名字、质量、质心、惯量、碰撞形状、摩擦)。
/// 全部真实单位(千克、米);生成时质量与惯量乘全局系数 k(§4)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LinkDef {
    pub name: String,
    /// 质量(kg,真实单位;≤ 0 → `BadMass`)。
    pub mass: f32,
    /// 质心(连杆局部系;缺省原点)。
    #[serde(default)]
    pub com: [f32; 3],
    /// 惯量(对角线;缺省 = 无,由 rapier 按碰撞形状算)。
    #[serde(default)]
    pub inertia: Option<[f32; 3]>,
    /// 摩擦系数(缺省 0.6,与普通实体的摩擦一致;capsule 资产显式写 0.3:
    /// μ 只决定主体对方块的摩擦)。
    #[serde(default = "default_friction")]
    pub friction: f32,
    /// 恢复系数(capsule demo:0)。
    #[serde(default)]
    pub restitution: f32,
    /// 碰撞形状(1..=8,校验层)。
    pub shapes: Vec<ShapeDef>,
}

fn default_friction() -> f32 {
    0.6
}

/// 身体定义(REQ-012 §2.1)。单连杆是退化情况(1 连杆、0 关节)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyDef {
    /// 身体名(注册表 by_name 索引;客户端按名字引用)。
    pub name: String,
    /// 锁定根连杆转动(capsule 玩家身体:true)。
    #[serde(default)]
    pub lock_rotations: bool,
    /// 连杆(≥ 1)。
    pub links: Vec<LinkDef>,
    /// 关节(单连杆为空)。
    #[serde(default)]
    pub joints: Vec<JointDef>,
    /// 自碰撞(默认全关)。
    #[serde(default)]
    pub self_collision: SelfCollisionDef,
}

/// 解析与校验失败原因(类型化,不 panic)。
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum BodyDefError {
    #[error("body.toml 解析失败:{0}")]
    Parse(String),
    #[error("连杆名重复:{0}")]
    DuplicateLink(String),
    #[error("连杆树不是一棵树:多个根 {0:?}")]
    MultipleRoots(Vec<String>),
    #[error("关节的连杆不存在:child={child} parent={parent}")]
    UnknownParent { child: String, parent: String },
    #[error("连杆树有闭环:{0:?}")]
    Cycle(Vec<String>),
    #[error("连杆不连通(不在根的子树内):{0:?}")]
    Detached(Vec<String>),
    #[error("连杆 {link} 有 {count} 个碰撞形状,超过上限 8")]
    TooManyShapes { link: String, count: usize },
    #[error("连杆 {0} 质量须 > 0")]
    BadMass(String),
    #[error("字段 {0} 数值非有限")]
    NonFinite(String),
    #[error("关节 {joint} 上限 {upper} 小于下限 {lower}")]
    BadLimits {
        joint: String,
        lower: f32,
        upper: f32,
    },
    #[error("至少要有一个连杆")]
    EmptyLinks,
}

impl BodyDef {
    /// 从 toml 文本解析 + 校验(解析即规范:哈希基于解析后的结构)。
    pub fn from_toml_str(text: &str) -> Result<Self, BodyDefError> {
        let def: BodyDef = toml::from_str(text).map_err(|e| BodyDefError::Parse(e.to_string()))?;
        def.validate()?;
        Ok(def)
    }

    /// 校验(REQ-012 A01 第 1 节,每项一条单测):树形、形状上限、质量、
    /// 有限性、关节上下限。返回类型化错误、不 panic。
    pub fn validate(&self) -> Result<(), BodyDefError> {
        if self.links.is_empty() {
            return Err(BodyDefError::EmptyLinks);
        }
        // 连杆名唯一。
        let mut seen = std::collections::BTreeSet::new();
        for l in &self.links {
            if !seen.insert(l.name.as_str()) {
                return Err(BodyDefError::DuplicateLink(l.name.clone()));
            }
        }
        // ---- 数值有限性与质量 ----
        for l in &self.links {
            if !l.mass.is_finite() {
                return Err(BodyDefError::NonFinite(format!("{}.mass", l.name)));
            }
            if l.mass <= 0.0 {
                return Err(BodyDefError::BadMass(l.name.clone()));
            }
            for (i, v) in l.com.iter().enumerate() {
                if !v.is_finite() {
                    return Err(BodyDefError::NonFinite(format!("{}.com[{i}]", l.name)));
                }
            }
            if let Some(inertia) = l.inertia {
                for (i, v) in inertia.iter().enumerate() {
                    if !v.is_finite() {
                        return Err(BodyDefError::NonFinite(format!("{}.inertia[{i}]", l.name)));
                    }
                }
            }
            if !l.friction.is_finite() || !l.restitution.is_finite() {
                return Err(BodyDefError::NonFinite(format!(
                    "{}.friction/restitution",
                    l.name
                )));
            }
            for (i, s) in l.shapes.iter().enumerate() {
                let field = |name: &str| format!("{}.shapes[{i}].{name}", l.name);
                match s {
                    ShapeDef::Box { half } => {
                        if half.iter().any(|v| !v.is_finite()) {
                            return Err(BodyDefError::NonFinite(field("half")));
                        }
                    }
                    ShapeDef::Sphere { radius } => {
                        if !radius.is_finite() {
                            return Err(BodyDefError::NonFinite(field("radius")));
                        }
                    }
                    ShapeDef::CapsuleY {
                        half_height,
                        radius,
                    } => {
                        if !half_height.is_finite() || !radius.is_finite() {
                            return Err(BodyDefError::NonFinite(field("capsule_y")));
                        }
                    }
                    ShapeDef::ConvexHull { points } => {
                        if points.iter().any(|p| p.iter().any(|v| !v.is_finite())) {
                            return Err(BodyDefError::NonFinite(field("points")));
                        }
                    }
                }
            }
            if l.shapes.len() > 8 {
                return Err(BodyDefError::TooManyShapes {
                    link: l.name.clone(),
                    count: l.shapes.len(),
                });
            }
        }
        // ---- 关节:父子连杆存在、数值有限、上下限 ----
        for j in &self.joints {
            if !seen.contains(j.parent.as_str()) {
                return Err(BodyDefError::UnknownParent {
                    child: j.child.clone(),
                    parent: j.parent.clone(),
                });
            }
            if !seen.contains(j.child.as_str()) {
                return Err(BodyDefError::UnknownParent {
                    child: j.child.clone(),
                    parent: j.parent.clone(),
                });
            }
            let p = j.kind.params();
            if p.axis.iter().any(|v| !v.is_finite()) || p.anchor.iter().any(|v| !v.is_finite()) {
                return Err(BodyDefError::NonFinite(format!(
                    "{}->{}.axis/anchor",
                    j.parent, j.child
                )));
            }
            for (field, v) in [
                ("max_torque", p.max_torque),
                ("max_velocity", p.max_velocity),
                ("kp", p.kp),
                ("kd", p.kd),
                ("default_angle", p.default_angle),
            ] {
                if v.is_some_and(|x| !x.is_finite()) {
                    return Err(BodyDefError::NonFinite(format!(
                        "{}->{}.{field}",
                        j.parent, j.child
                    )));
                }
            }
            if let Some(lim) = p.limits {
                if !lim.lower.is_finite() || !lim.upper.is_finite() {
                    return Err(BodyDefError::NonFinite(format!(
                        "{}->{}.limits",
                        j.parent, j.child
                    )));
                }
                if lim.upper < lim.lower {
                    return Err(BodyDefError::BadLimits {
                        joint: format!("{}->{}", j.parent, j.child),
                        lower: lim.lower,
                        upper: lim.upper,
                    });
                }
            }
        }
        // ---- 连杆树:每连杆至多一个父关节(多父 = 非树)----
        let mut child_of: std::collections::BTreeMap<&str, &str> =
            std::collections::BTreeMap::new();
        for j in &self.joints {
            if child_of.contains_key(j.child.as_str()) {
                return Err(BodyDefError::Cycle(vec![j.child.clone()]));
            }
            child_of.insert(j.child.as_str(), j.parent.as_str());
        }
        let roots: Vec<&str> = self
            .links
            .iter()
            .map(|l| l.name.as_str())
            .filter(|n| !child_of.contains_key(n))
            .collect();
        match roots.as_slice() {
            [] => {
                // 全部连杆都有父 → 必有环。
                let cycle = Self::find_cycle(&child_of);
                return Err(BodyDefError::Cycle(cycle));
            }
            [root] => {
                // BFS 连通性:从唯一根可达全部连杆 = 无环 + 无游离 subtree。
                let mut reachable = std::collections::BTreeSet::new();
                let mut stack = vec![*root];
                reachable.insert(*root);
                while let Some(cur) = stack.pop() {
                    for j in &self.joints {
                        if j.parent.as_str() == cur && reachable.insert(j.child.as_str()) {
                            stack.push(j.child.as_str());
                        }
                    }
                }
                let detached: Vec<String> = self
                    .links
                    .iter()
                    .filter(|l| !reachable.contains(l.name.as_str()))
                    .map(|l| l.name.clone())
                    .collect();
                if !detached.is_empty() {
                    return Err(BodyDefError::Detached(detached));
                }
            }
            [..] => {
                return Err(BodyDefError::MultipleRoots(
                    roots.iter().map(|s| (*s).to_owned()).collect(),
                ));
            }
        }
        Ok(())
    }

    /// 环查找(child_of:每连杆至多一父;沿父链走到已见节点即环。
    /// 起点 = 最小连杆名,确定性)。
    fn find_cycle(child_of: &std::collections::BTreeMap<&str, &str>) -> Vec<String> {
        let Some(&start) = child_of.keys().next() else {
            return Vec::new();
        };
        let mut seen = std::collections::BTreeMap::new();
        let mut order: Vec<&str> = Vec::new();
        let mut cur = start;
        loop {
            if let Some(&pos) = seen.get(cur) {
                return order[pos..].iter().map(|s| (*s).to_owned()).collect();
            }
            match child_of.get(cur) {
                None => return Vec::new(),
                Some(&parent) => {
                    seen.insert(cur, order.len());
                    order.push(cur);
                    cur = parent;
                }
            }
        }
    }

    /// 内容哈希(REQ-012 A01 第 1 节):对解析后的规范形式求 FNV-1a 64 位。
    /// 注释、空白、键序变化不影响;任一数值变化必变(to_bits 逐字节)。
    pub fn hash(&self) -> BodyHash {
        BodyHash::fnv1a(&self.canonical_bytes())
    }

    /// 规范字节流:字段显式定序,f32 用 `to_bits`(u32 LE),变长段 u32 长度
    /// 前缀;字符串 = 长度前缀 + 字节。手写编码零依赖,顺序即契约;格式
    /// 哨兵字节防字段集变化后跨版本撞哈希。
    fn canonical_bytes(&self) -> Vec<u8> {
        let mut b = Vec::new();
        b.push(1);
        put_str(&mut b, &self.name);
        b.push(u8::from(self.lock_rotations));
        put_len(&mut b, self.links.len());
        for l in &self.links {
            put_str(&mut b, &l.name);
            put_f32(&mut b, l.mass);
            put_f32s(&mut b, &l.com);
            match l.inertia {
                None => b.push(0),
                Some(inertia) => {
                    b.push(1);
                    put_f32s(&mut b, &inertia);
                }
            }
            put_f32(&mut b, l.friction);
            put_f32(&mut b, l.restitution);
            put_len(&mut b, l.shapes.len());
            for s in &l.shapes {
                match s {
                    ShapeDef::Box { half } => {
                        b.push(1);
                        put_f32s(&mut b, half);
                    }
                    ShapeDef::Sphere { radius } => {
                        b.push(2);
                        put_f32(&mut b, *radius);
                    }
                    ShapeDef::CapsuleY {
                        half_height,
                        radius,
                    } => {
                        b.push(3);
                        put_f32(&mut b, *half_height);
                        put_f32(&mut b, *radius);
                    }
                    ShapeDef::ConvexHull { points } => {
                        b.push(4);
                        put_len(&mut b, points.len());
                        for p in points {
                            put_f32s(&mut b, p);
                        }
                    }
                }
            }
        }
        put_len(&mut b, self.joints.len());
        for j in &self.joints {
            put_str(&mut b, &j.parent);
            put_str(&mut b, &j.child);
            b.push(match j.kind.joint_type() {
                JointType::Revolute => 1,
                JointType::Prismatic => 2,
                JointType::Fixed => 3,
            });
            let p = j.kind.params();
            put_f32s(&mut b, &p.axis);
            put_f32s(&mut b, &p.anchor);
            match p.limits {
                None => b.push(0),
                Some(l) => {
                    b.push(1);
                    put_f32(&mut b, l.lower);
                    put_f32(&mut b, l.upper);
                }
            }
            for v in [p.max_torque, p.max_velocity, p.kp, p.kd, p.default_angle] {
                match v {
                    None => b.push(0),
                    Some(x) => {
                        b.push(1);
                        put_f32(&mut b, x);
                    }
                }
            }
        }
        put_len(&mut b, self.self_collision.extra_pairs.len());
        for pair in &self.self_collision.extra_pairs {
            put_str(&mut b, &pair[0]);
            put_str(&mut b, &pair[1]);
        }
        b
    }
}

fn put_str(b: &mut Vec<u8>, s: &str) {
    put_len(b, s.len());
    b.extend_from_slice(s.as_bytes());
}

fn put_len(b: &mut Vec<u8>, n: usize) {
    b.extend_from_slice(&(n as u64).to_le_bytes());
}

fn put_f32(b: &mut Vec<u8>, v: f32) {
    b.extend_from_slice(&v.to_bits().to_le_bytes());
}

fn put_f32s(b: &mut Vec<u8>, vs: &[f32]) {
    for v in vs {
        put_f32(b, *v);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 单连杆 capsule(与 assets/bodies/capsule.body.toml 同构的最小样例)。
    fn capsule_toml() -> String {
        r#"
name = "capsule"
lock_rotations = true

[[links]]
name = "torso"
mass = 80.0
friction = 0.3
restitution = 0.0

[[links.shapes]]
kind = "capsule_y"
half_height = 0.6
radius = 0.3
"#
        .to_owned()
    }

    /// 解析:字段齐全的单连杆;默认值(friction 缺省 0.6)生效。
    #[test]
    fn parse_single_link_capsule() {
        let def = BodyDef::from_toml_str(&capsule_toml()).expect("合法 body");
        assert_eq!(def.name, "capsule");
        assert!(def.lock_rotations);
        assert_eq!(def.links.len(), 1);
        assert!(def.joints.is_empty());
        let l = &def.links[0];
        assert_eq!(l.name, "torso");
        assert_eq!(l.mass, 80.0);
        assert_eq!(l.friction, 0.3);
        assert_eq!(l.restitution, 0.0);
        assert_eq!(
            l.shapes,
            vec![ShapeDef::CapsuleY {
                half_height: 0.6,
                radius: 0.3,
            }]
        );
        // hash 可复现(同文本两次解析)。
        assert_eq!(
            def.hash(),
            BodyDef::from_toml_str(&capsule_toml()).unwrap().hash()
        );
    }

    /// 哈希:注释、空白、键序变化不影响;任一数值变化必变。
    #[test]
    fn hash_ignores_comments_whitespace_key_order_but_changes_with_values() {
        let a = BodyDef::from_toml_str(&capsule_toml()).expect("合法");
        // 键序打乱 + 注释 + 空白:哈希不变。
        let reordered = r#"
# 注释与空白随意
lock_rotations = true
name = "capsule"
[[links]]
restitution = 0.0
mass = 80.0
friction = 0.3
name = "torso"
[[links.shapes]]
radius = 0.3
kind = "capsule_y"
half_height = 0.6
"#;
        let b = BodyDef::from_toml_str(reordered).expect("合法(键序无关)");
        assert_eq!(a.hash(), b.hash(), "注释/空白/键序不影响哈希");

        // 任一数值变化 → 哈希变(质量)。
        let mass_changed = capsule_toml().replace("mass = 80.0", "mass = 79.0");
        let c = BodyDef::from_toml_str(&mass_changed).expect("合法");
        assert_ne!(a.hash(), c.hash(), "质量变化必改哈希");
        // 数值变化(形状半径)→ 哈希变。
        let shape_changed = capsule_toml().replace("radius = 0.3", "radius = 0.31");
        let d = BodyDef::from_toml_str(&shape_changed).expect("合法");
        assert_ne!(a.hash(), d.hash(), "形状数值变化必改哈希");
        // 名字变化 → 哈希变(名字属于内容)。
        let name_changed = capsule_toml().replace("name = \"capsule\"", "name = \"other\"");
        let e = BodyDef::from_toml_str(&name_changed).expect("合法");
        assert_ne!(a.hash(), e.hash(), "名字变化改哈希");
    }

    /// 哈希算法跨平台稳定:FNV-1a 64 位对固定输入的已知答案(逐字节运算,
    /// 与平台字节序无关;钉死实现防意外变更)。
    #[test]
    fn fnv1a_known_answers() {
        // 空输入 = offset basis。
        assert_eq!(
            BodyHash::fnv1a(&[]),
            BodyHash([0xcb, 0xf2, 0x9c, 0xe4, 0x84, 0x22, 0x23, 0x25])
        );
        // "a"、"b"、"foobar" 的 FNV-1a 64 位已知答案。
        assert_eq!(
            BodyHash::fnv1a(b"a").0,
            0xaf63_dc4c_8601_ec8cu64.to_be_bytes()
        );
        assert_eq!(
            BodyHash::fnv1a(b"b").0,
            0xaf63_df4c_8601_f1a5u64.to_be_bytes()
        );
        assert_eq!(
            BodyHash::fnv1a(b"foobar").0,
            0x8594_4171_f739_67e8u64.to_be_bytes()
        );
    }

    /// 校验:闭环(根→子→父成环)拒绝;多根拒绝;父连杆不存在拒绝。
    #[test]
    fn validate_rejects_cycle_multiple_roots_unknown_parent() {
        let links = |names: &[&str]| {
            names
                .iter()
                .map(|n| LinkDef {
                    name: (*n).to_owned(),
                    mass: 1.0,
                    com: [0.0; 3],
                    inertia: None,
                    friction: 0.6,
                    restitution: 0.0,
                    shapes: vec![ShapeDef::Sphere { radius: 0.1 }],
                })
                .collect::<Vec<_>>()
        };
        let joint = |parent: &str, child: &str| JointDef {
            parent: parent.to_owned(),
            child: child.to_owned(),
            kind: JointKind::Revolute(JointParams {
                axis: [1.0, 0.0, 0.0],
                anchor: [0.0; 3],
                limits: None,
                max_torque: None,
                max_velocity: None,
                kp: None,
                kd: None,
                default_angle: None,
            }),
        };
        let mk = |l: Vec<LinkDef>, j: Vec<JointDef>| BodyDef {
            name: "t".to_owned(),
            lock_rotations: false,
            links: l,
            joints: j,
            self_collision: SelfCollisionDef::default(),
        };

        // 闭环:a→b→a(无根)。
        let cyc = mk(links(&["a", "b"]), vec![joint("a", "b"), joint("b", "a")]);
        assert!(
            matches!(cyc.validate(), Err(BodyDefError::Cycle(_))),
            "闭环拒绝"
        );

        // 多根:a、b 互不相连(无关节)。
        let multi = mk(links(&["a", "b"]), vec![]);
        assert_eq!(
            multi.validate(),
            Err(BodyDefError::MultipleRoots(vec![
                "a".to_owned(),
                "b".to_owned()
            ])),
            "多根拒绝"
        );

        // 父连杆不存在。
        let orphan = mk(links(&["a"]), vec![joint("ghost", "a")]);
        assert_eq!(
            orphan.validate(),
            Err(BodyDefError::UnknownParent {
                child: "a".to_owned(),
                parent: "ghost".to_owned(),
            }),
            "父连杆不存在拒绝"
        );

        // 游离子树(两棵树各有根?不——只有一个根的判定先走)。
        // a→b、c 孤立:c 是第二个根 → MultipleRoots;这里验证 Detached 用
        // 伪关节成环 + 单根可达其余:child 重复占用 → Cycle 分支先到。
        // 游离:根 a,b 挂在 a 下,c 挂在 b 下,d 无关节 → 两根 → MultipleRoots。
        let two = mk(
            links(&["a", "b", "c", "d"]),
            vec![joint("a", "b"), joint("b", "c")],
        );
        assert!(
            matches!(two.validate(), Err(BodyDefError::MultipleRoots(_))),
            "a-b-c 树 + 孤立 d = 两根,拒绝"
        );
    }

    /// 校验:某连杆超过 8 个碰撞形状拒绝;恰 8 个合法。
    #[test]
    fn validate_rejects_more_than_eight_shapes() {
        let link = |count: usize| LinkDef {
            name: "torso".to_owned(),
            mass: 1.0,
            com: [0.0; 3],
            inertia: None,
            friction: 0.6,
            restitution: 0.0,
            shapes: vec![ShapeDef::Sphere { radius: 0.1 }; count],
        };
        let mk = |count: usize| BodyDef {
            name: "t".to_owned(),
            lock_rotations: false,
            links: vec![link(count)],
            joints: Vec::new(),
            self_collision: SelfCollisionDef::default(),
        };
        assert!(mk(8).validate().is_ok(), "恰 8 个合法");
        assert_eq!(
            mk(9).validate(),
            Err(BodyDefError::TooManyShapes {
                link: "torso".to_owned(),
                count: 9,
            }),
            "9 个拒绝"
        );
    }

    /// 校验:质量 ≤ 0 拒绝;任何数值非有限(NaN/inf)拒绝,错误点名字段。
    #[test]
    fn validate_rejects_nonpositive_mass_and_nonfinite() {
        let mut def = BodyDef::from_toml_str(&capsule_toml()).expect("合法");
        def.links[0].mass = 0.0;
        assert_eq!(
            def.validate(),
            Err(BodyDefError::BadMass("torso".to_owned())),
            "质量 0 拒绝"
        );
        def.links[0].mass = -1.0;
        assert!(matches!(def.validate(), Err(BodyDefError::BadMass(_))));
        def.links[0].mass = 80.0;
        def.links[0].friction = f32::NAN;
        assert_eq!(
            def.validate(),
            Err(BodyDefError::NonFinite(
                "torso.friction/restitution".to_owned()
            )),
            "NaN 摩擦拒绝"
        );
        def.links[0].friction = 0.3;
        def.links[0].com = [0.0, f32::INFINITY, 0.0];
        assert!(matches!(def.validate(), Err(BodyDefError::NonFinite(_))));
    }

    /// 校验:关节上限小于下限拒绝;相等合法(单值关节)。
    #[test]
    fn validate_rejects_limits_upper_below_lower() {
        let toml = |lower: f64, upper: f64| {
            format!(
                r#"
name = "leg"
[[links]]
name = "hip"
mass = 5.0
[[links.shapes]]
kind = "sphere"
radius = 0.1
[[links]]
name = "knee"
mass = 2.0
[[links.shapes]]
kind = "sphere"
radius = 0.1
[[joints]]
parent = "hip"
child = "knee"
type = "revolute"
axis = [1.0, 0.0, 0.0]
anchor = [0.0, -0.1, 0.0]
limits = {{ lower = {lower}, upper = {upper} }}
"#
            )
        };
        let bad = BodyDef::from_toml_str(&toml(-1.0, -2.0));
        assert_eq!(
            bad,
            Err(BodyDefError::BadLimits {
                joint: "hip->knee".to_owned(),
                lower: -1.0,
                upper: -2.0,
            }),
            "上限 < 下限拒绝"
        );
        assert!(
            BodyDef::from_toml_str(&toml(-1.0, -1.0)).is_ok(),
            "相等合法"
        );
        assert!(BodyDef::from_toml_str(&toml(-1.0, 1.0)).is_ok(), "常规合法");
    }

    /// 多连杆合法定义:树形、类型已知、解析往返;未知关节类型在解析层拒绝。
    #[test]
    fn multi_link_tree_and_unknown_joint_type() {
        let ok = r#"
name = "two-link"
[[links]]
name = "base"
mass = 10.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[links]]
name = "arm"
mass = 1.0
[[links.shapes]]
kind = "capsule_y"
half_height = 0.25
radius = 0.05
[[joints]]
parent = "base"
child = "arm"
type = "revolute"
axis = [0.0, 1.0, 0.0]
anchor = [0.0, 0.1, 0.0]
limits = { lower = -3.14159, upper = 3.14159 }
max_torque = 23.7
max_velocity = 10.0
kp = 100.0
kd = 3.0
default_angle = 0.0
[self_collision]
extra_pairs = [["base", "arm"]]
"#;
        let def = BodyDef::from_toml_str(ok).expect("两连杆合法");
        assert_eq!(def.joints.len(), 1);
        assert_eq!(def.joints[0].kind.joint_type(), JointType::Revolute);
        assert_eq!(def.self_collision.extra_pairs.len(), 1);
        // 多连杆哈希可复现。
        assert_eq!(def.hash(), BodyDef::from_toml_str(ok).unwrap().hash());

        // 未知关节类型:解析层类型化拒绝(不 panic)。
        let unknown = ok.replace("type = \"revolute\"", "type = \"screw\"");
        match BodyDef::from_toml_str(&unknown) {
            Err(BodyDefError::Parse(msg)) => {
                assert!(msg.contains("screw"), "错误信息应含未知类型:{msg}");
            }
            other => panic!("未知关节类型必须 Err(Parse),实为 {other:?}"),
        }
        // 未知形状类别:同样解析层拒绝。
        let bad_shape = capsule_toml().replace("capsule_y", "torus_y");
        assert!(matches!(
            BodyDef::from_toml_str(&bad_shape),
            Err(BodyDefError::Parse(_))
        ));
    }
}
