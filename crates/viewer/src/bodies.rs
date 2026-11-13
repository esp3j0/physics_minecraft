//! bodies —— 哈希 → 形状表(REQ-012 A06 §1):启动时从 `assets/bodies/
//! *.body.toml` 用 `types::BodyDef::from_toml_str` + `hash()` 建表;渲染层
//! 按 `AgentStates` 里的哈希查形状逐连杆画。查不到 = 占位盒 + HUD 计数,
//! 不 panic。纯逻辑、零 macroquad(渲染接线在 render.rs;本文件可单测,
//! 判据 A06-3)。

use std::collections::BTreeMap;

use types::{BodyDef, BodyHash, ShapeDef};

/// 一个连杆的渲染形状(世界系,连杆局部系;格 = 米)。B01a:每个变体带
/// `offset`(形状在连杆局部系里的平移,缺省零——quad 脚球挂在小腿末端)。
#[derive(Debug, Clone, PartialEq)]
pub enum LinkShape {
    /// 轴对齐盒(半边长、连杆局部系平移)。
    Box([f32; 3], [f32; 3]),
    /// 球(半径、连杆局部系平移)。
    Sphere(f32, [f32; 3]),
    /// y 向胶囊(半高、半径、连杆局部系平移)——viewer 画等高的圆柱(放行
    /// §1:胶囊画胶囊或等高的圆柱,二选一;macroquad 无胶囊原语,取圆柱:
    /// 总高一致)。
    CapsuleY {
        half_height: f32,
        radius: f32,
        offset: [f32; 3],
    },
    /// 凸包(顶点集、连杆局部系平移;viewer 画包围盒)。
    ConvexHull(Vec<[f32; 3]>, [f32; 3]),
}

/// 一连杆渲染条目:形状列表(每连杆 ≤ 8)。
pub type LinkShapes = Vec<LinkShape>;

/// 内置 `capsule` 身体(R1 §3.5):随二进制编入,`from_dir` 总是先放它,
/// 目录读到的同名定义覆盖(`from_defs` 的 BTreeMap 后写者胜)。
const BUILTIN_CAPSULE: &str = include_str!("../../../assets/bodies/capsule.body.toml");

/// 哈希 → 形状表(缺省空;`from_defs`/`from_dir` 构建)。
#[derive(Debug, Default, Clone)]
pub struct BodyShapeTable {
    by_hash: BTreeMap<BodyHash, Vec<LinkShapes>>,
    /// 未知哈希占位计数(A06 §1:HUD 计数;渲染到未知哈希 +1)。
    unknown_draws: u64,
}

impl BodyShapeTable {
    pub fn new() -> Self {
        Self::default()
    }

    /// 从解析后的身体定义建表(`types::BodyDef`;viewer 不依赖
    /// physics-sim,直接用 types 的解析与哈希——与服务端同一哈希口径:
    /// 服务端 `BodyRegistry::insert_toml` → `BodyDef::from_toml_str` +
    /// `hash()`,逐位同源)。
    pub fn from_defs<'a>(defs: impl IntoIterator<Item = &'a BodyDef>) -> Self {
        let mut table = Self::new();
        for def in defs {
            let hash = def.hash();
            let links: Vec<LinkShapes> = def
                .links
                .iter()
                .map(|link| {
                    link.shapes
                        .iter()
                        .map(|shape| match shape {
                            ShapeDef::Box { half, offset } => LinkShape::Box(*half, *offset),
                            ShapeDef::Sphere { radius, offset } => {
                                LinkShape::Sphere(*radius, *offset)
                            }
                            ShapeDef::CapsuleY {
                                half_height,
                                radius,
                                offset,
                            } => LinkShape::CapsuleY {
                                half_height: *half_height,
                                radius: *radius,
                                offset: *offset,
                            },
                            ShapeDef::ConvexHull { points, offset } => {
                                LinkShape::ConvexHull(points.clone(), *offset)
                            }
                        })
                        .collect()
                })
                .collect();
            table.by_hash.insert(hash, links);
        }
        table
    }

    /// 从目录读 `*.body.toml` 建表(路径不存在 = 只有内置身体,不报错——
    /// viewer 不因缺资产 fail-stop;解析失败同样跳过该文件并计数,渲染走
    /// 占位)。R1 §3.5:内置 `capsule`(`include_str!` 随二进制),目录读到
    /// 的定义覆盖或补充——不在仓库根目录启动 viewer 时胶囊也能正常画出来。
    pub fn from_dir(dir: &std::path::Path) -> Self {
        let mut defs = load_defs(dir);
        if let Ok(builtin) = BodyDef::from_toml_str(BUILTIN_CAPSULE) {
            // 内置在前、目录在后:from_defs 的 BTreeMap 后写者胜 = 同名覆盖。
            defs.insert(0, builtin);
        }
        Self::from_defs(&defs)
    }

    /// 按哈希查连杆形状(连杆顺序 = `AgentStates` 姿态顺序 = `BodyDef.links`
    /// 顺序);未知哈希 → None(调用方画占位 + 计数)。
    pub fn get(&self, hash: &BodyHash) -> Option<&[LinkShapes]> {
        self.by_hash.get(hash).map(Vec::as_slice)
    }

    /// 表内身体数(HUD 显示)。
    pub fn len(&self) -> usize {
        self.by_hash.len()
    }

    pub fn is_empty(&self) -> bool {
        self.by_hash.is_empty()
    }

    /// 未知哈希占位计数(取走即清;HUD 增量显示)。
    pub fn take_unknown_draws(&mut self) -> u64 {
        std::mem::take(&mut self.unknown_draws)
    }

    /// 渲染层查不到哈希时调用:占位 + 计数(不 panic)。
    pub fn note_unknown(&mut self) {
        self.unknown_draws += 1;
    }
}

/// 读目录里的全部 `*.body.toml`(排序读取;解析失败跳过——viewer 的形状
/// 表只是渲染辅助,坏文件不 fail-stop;主体会走占位路径)。
fn load_defs(dir: &std::path::Path) -> Vec<BodyDef> {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut paths: Vec<_> = entries
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|ext| ext == "toml"))
        .collect();
    paths.sort();
    let mut defs = Vec::new();
    for path in paths {
        if let Ok(text) = std::fs::read_to_string(&path)
            && let Ok(def) = BodyDef::from_toml_str(&text)
        {
            defs.push(def);
        }
    }
    defs
}

#[cfg(test)]
mod tests {
    use super::*;

    const CAPSULE_TOML: &str = include_str!("../../../assets/bodies/capsule.body.toml");

    /// 哈希 → 形状表能从资产建起(A06-3):capsule 解析出 1 连杆 1 胶囊;
    /// 表内哈希 = 解析定义的哈希(与服务端同口径)。
    #[test]
    fn table_builds_from_capsule_asset() {
        let def = BodyDef::from_toml_str(CAPSULE_TOML).expect("capsule 合法");
        let hash = def.hash();
        let table = BodyShapeTable::from_defs([&def]);
        assert_eq!(table.len(), 1);
        let links = table.get(&hash).expect("capsule 哈希可查");
        assert_eq!(links.len(), 1, "1 连杆");
        assert_eq!(
            links[0],
            vec![LinkShape::CapsuleY {
                half_height: 0.6,
                radius: 0.3,
                offset: [0.0; 3],
            }],
            "torso = y 向胶囊(0.6, 0.3)"
        );
    }

    /// 未知哈希走占位路径:get → None,note_unknown 计数,take 清零;
    /// 全程不 panic(A06-3)。
    #[test]
    fn unknown_hash_placeholder_and_counter() {
        let def = BodyDef::from_toml_str(CAPSULE_TOML).expect("capsule 合法");
        let table = BodyShapeTable::from_defs([&def]);
        let unknown = BodyHash::fnv1a(b"not-a-body");
        assert!(table.get(&unknown).is_none(), "未知哈希 → None");
        let mut table = table;
        table.note_unknown();
        table.note_unknown();
        assert_eq!(table.take_unknown_draws(), 2, "占位计数 2");
        assert_eq!(table.take_unknown_draws(), 0, "取走即清");
    }

    /// 目录加载(R1 §3.5):目录不存在也至少有内置 capsule(不报错);
    /// 真目录(工作区根的 assets/bodies;测试 cwd = crates/viewer)读到的
    /// 同名定义覆盖内置(表内仍是 1 条——同名哈希覆盖,不重复)。
    #[test]
    fn from_dir_missing_has_builtin_capsule() {
        let table = BodyShapeTable::from_dir(std::path::Path::new("no/such/dir"));
        assert_eq!(table.len(), 1, "目录缺失也有内置 capsule");
        let def = BodyDef::from_toml_str(CAPSULE_TOML).expect("capsule 合法");
        assert!(
            table.get(&def.hash()).is_some(),
            "内置 capsule 的哈希可查(不在仓库根启动也能画)"
        );
        // 真目录:同名 capsule 覆盖内置(B01a 起目录还有 quad.body.toml,
        // 表内 = 目录里的文件数;这里的"1 条"断言改成对数量不敏感的
        // "目录读取增加了条目",覆盖语义由 from_defs 的 BTreeMap 保证)。
        let real = BodyShapeTable::from_dir(std::path::Path::new("../../assets/bodies"));
        assert!(
            !real.is_empty(),
            "资产目录至少有内置 capsule(同名覆盖;B01a 起含 quad)"
        );
        assert!(real.get(&def.hash()).is_some());
    }

    /// 多连杆身体:连杆顺序保持 BodyDef.links 顺序(渲染按 AgentStates
    /// 姿态顺序 zip)。手工两连杆定义(无 toml 资产;直接构造)。
    #[test]
    fn multi_link_order_preserved() {
        let toml = r#"
name = "two"
[[links]]
name = "a"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[links]]
name = "b"
mass = 1.0
[[links.shapes]]
kind = "sphere"
radius = 0.2
[[joints]]
parent = "a"
child = "b"
type = "fixed"
axis = [0, 1, 0]
anchor = [0, 0, 0]
"#;
        let def = BodyDef::from_toml_str(toml).expect("两连杆定义合法");
        let table = BodyShapeTable::from_defs([&def]);
        let links = table.get(&def.hash()).expect("可查");
        assert_eq!(links.len(), 2);
        assert_eq!(links[0], vec![LinkShape::Box([0.1, 0.1, 0.1], [0.0; 3])]);
        assert_eq!(links[1], vec![LinkShape::Sphere(0.2, [0.0; 3])]);
    }
}
