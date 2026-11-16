//! REQ-012 C01a:URDF → `BodyDef` 导入工具(转换核心)。
//!
//! 公开接口(任务书 1.2):
//! - [`import_urdf`]:URDF 文本 → `BodyDef` + [`ImportReport`](report::ImportReport)
//!   (纯函数:同样输入,输出逐字节相同);
//! - [`body_to_toml`] / [`report_to_markdown`]:输出文本。
//!
//! 模块:[`frames`] 坐标换算、[`tree`] 连杆树与固定关节合并、
//! [`shapes`] 形状转换、[`inertia`] 惯量、[`emit`] 输出文本。
//!
//! 事实基础(任务书第 0 节):
//! 1. `BodyDef` 连杆坐标系无旋转——所有旋转烘进数据,BodyDef 各连杆
//!    坐标系互相平行;
//! 2. 内核不使用 `LinkDef.com` / `LinkDef.inertia`(生成时由碰撞形状
//!    推出),导入照算、写文件,报告注明;
//! 3. 质量比例 `agent_mass_scale` 在生成时乘,导入输出真实单位。

pub mod emit;
pub mod frames;
pub mod inertia;
mod mesh;
pub mod shapes;
pub mod tree;

/// 报告与选项类型。
pub mod report {
    /// 竖直轴(任务书 1.2)。URDF 惯例 Z(默认)。
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum UpAxis {
        Z,
        Y,
    }

    /// 导入选项(任务书 1.2)。
    #[derive(Debug, Clone)]
    pub struct ImportOptions {
        /// 写进 `BodyDef.name`。
        pub name: String,
        /// URDF 的竖直轴。
        pub up_axis: UpAxis,
        /// 写进每个非 fixed 关节(URDF 没有 PD 增益)。默认 100 / 3。
        pub kp: f32,
        pub kd: f32,
        /// 写进每个连杆。默认 0.6。
        pub friction: f32,
        /// URDF 关节名 → 默认角。没列出的关节取 0,再夹进限位。
        pub default_angles: std::collections::BTreeMap<String, f32>,
        /// 圆柱转凸包时每圈的点数。默认 16,范围 8..=64。
        pub cylinder_segments: u32,
        /// 网格凸包最多保留的顶点数。默认 32,范围 4..=255。
        pub max_hull_points: u32,
        /// `package://<包名>/…` 的包名 → 目录。没列出的包名按回退规则找。
        pub package_dirs: std::collections::BTreeMap<String, std::path::PathBuf>,
    }

    /// 形状换算记录(任务书 1.2)。
    #[derive(Debug, Clone, PartialEq)]
    pub struct ShapeNote {
        pub link: String,
        /// "box" | "sphere" | "cylinder" | "capsule"
        pub from: String,
        /// "box" | "sphere" | "capsule_y" | "convex_hull"
        pub to: String,
        /// 源形状体积、输出形状体积(m³)。
        pub volume_src: f32,
        pub volume_out: f32,
        /// 凸包的顶点数(非凸包为 0)。
        pub hull_points: usize,
    }

    /// 惯量记录(任务书 1.2)。
    #[derive(Debug, Clone, PartialEq)]
    pub struct InertiaNote {
        pub link: String,
        /// 转到连杆坐标系后,惯性积绝对值的最大者 / 对角元的最小者。
        pub offdiag_ratio: f32,
    }

    /// 网格碰撞形状记录(C01b 任务书 1.2)。
    #[derive(Debug, Clone, PartialEq)]
    pub struct MeshNote {
        pub link: String,
        /// URDF 里写的 filename 原文。
        pub file: String,
        /// 文件里的顶点数(去重前;STL 是三角形数 × 3)。
        pub vertices: usize,
        /// 文件里的三角形数(OBJ 的多边形面按扇形拆成三角形后计)。
        pub triangles: usize,
        /// 完整凸包的顶点数。
        pub hull_full: usize,
        /// 写进 BodyDef 的顶点数(≤ max_hull_points)。
        pub hull_out: usize,
        /// 网格自身的体积:带符号四面体体积之和取绝对值(m³,缩放后)。
        /// 没有面时为 0。
        pub mesh_volume: f32,
        /// 完整凸包的体积。
        pub hull_volume: f32,
        /// 输出凸包(精简后)的体积。
        pub out_volume: f32,
    }

    /// 导入报告(任务书 1.2)。
    #[derive(Debug, Clone, PartialEq)]
    pub struct ImportReport {
        pub total_mass: f32,
        pub links: usize,
        pub dof: usize,
        /// 自由度顺序:(BodyDef 关节下标,URDF 关节名)——存 URDF 关节名。
        pub dof_order: Vec<String>,
        /// 被并入父连杆的固定关节子连杆:(子,并入到谁)。
        pub merged_fixed: Vec<(String, String)>,
        /// 没有质量也没有碰撞、被当作纯坐标系丢弃的连杆。
        pub dropped_frames: Vec<String>,
        /// 读到了但不使用的字段,每类一条,带出现次数。
        pub ignored: Vec<(String, usize)>,
        /// 形状换算记录。
        pub shapes: Vec<ShapeNote>,
        /// 每个网格碰撞形状一条。
        pub meshes: Vec<MeshNote>,
        /// 每个输出连杆的惯量记录。
        pub inertia: Vec<InertiaNote>,
        /// 其它提示。
        pub notes: Vec<String>,
    }
}

pub use report::{ImportOptions, ImportReport, InertiaNote, MeshNote, ShapeNote, UpAxis};

/// 导入失败原因(任务书 1.2)。
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ImportError {
    #[error("URDF 解析失败:{0}")]
    Xml(String),
    #[error("没有连杆")]
    NoLinks,
    #[error("连杆树有多个根:{0:?}")]
    MultipleRoots(Vec<String>),
    #[error("连杆树有闭环或一个连杆有多个父关节:{0:?}")]
    Cycle(Vec<String>),
    #[error("关节 {joint} 引用了不存在的连杆 {link}")]
    UnknownLink { joint: String, link: String },
    #[error("关节 {joint} 的类型 {kind} 不支持(只支持 revolute、continuous、prismatic、fixed)")]
    UnsupportedJoint { joint: String, kind: String },
    #[error("关节 {joint} 缺少 limit")]
    MissingLimit { joint: String },
    #[error("关节 {joint} 的轴为零向量或非有限")]
    BadAxis { joint: String },
    #[error("连杆 {link} 缺少惯量(inertial)")]
    MissingInertial { link: String },
    #[error("连杆 {link} 的质量 {mass} 须 > 0")]
    BadMass { link: String, mass: f32 },
    #[error("连杆 {link} 的惯量张量非法(非有限,或对角元 ≤ 0)")]
    BadInertia { link: String },
    #[error("连杆 {link} 没有碰撞形状")]
    NoCollision { link: String },
    #[error("连杆 {link} 的形状不支持:{what}")]
    UnsupportedShape { link: String, what: String },
    #[error("连杆 {link} 的网格文件 {file} 找不到(试过:{tried:?})")]
    MeshFileMissing {
        link: String,
        file: String,
        tried: Vec<String>,
    },
    #[error("连杆 {link} 的网格文件 {file} 读取或解析失败:{reason}")]
    MeshParse {
        link: String,
        file: String,
        reason: String,
    },
    #[error("连杆 {link} 的网格 {file} 建不出凸包(顶点太少、共线或共面):{reason}")]
    MeshDegenerate {
        link: String,
        file: String,
        reason: String,
    },
    #[error("连杆 {link} 合并后有 {count} 个碰撞形状,超过上限 8")]
    TooManyShapes { link: String, count: usize },
    #[error("选项非法:{0}")]
    BadOption(String),
    #[error("生成的身体没有通过校验:{0}")]
    Validate(types::BodyDefError),
}

/// URDF 文本 → BodyDef + 导入报告。纯函数:同样的输入文件和选项,输出
/// 逐字节相同。`base_dir` 是 URDF 所在目录(网格文件查找的基点)。
pub fn import_urdf(
    urdf_xml: &str,
    base_dir: &std::path::Path,
    opts: &ImportOptions,
) -> Result<(types::BodyDef, ImportReport), ImportError> {
    tree::import_impl(urdf_xml, base_dir, opts)
}

/// URDF 的 robot name(xtask `--name` 缺省值)。
pub fn robot_name(urdf_xml: &str) -> Result<String, ImportError> {
    let robot = urdf_rs::read_from_string(urdf_xml).map_err(|e| ImportError::Xml(e.to_string()))?;
    Ok(robot.name)
}

/// BodyDef → body.toml 文本(任务书 1.7)。
pub fn body_to_toml(def: &types::BodyDef) -> String {
    emit::body_to_toml(def)
}

/// 报告 → markdown(任务书 1.7)。
pub fn report_to_markdown(report: &ImportReport) -> String {
    emit::report_to_markdown(report)
}
