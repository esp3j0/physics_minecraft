//! 方块注册表。

use std::collections::BTreeMap;

use types::BlockId;

/// 方块注册表:名称 ↔ BlockId、方块属性。ID 分配后必须保持稳定(存档兼容)。
///
/// `Send` supertrait:shard worker 池要跨线程转移持有 `Box<dyn BlockRegistry>` 的
/// `ShardSim`(任务书 05 / README 决策 5);数据注册表(`SimpleRegistry`)自动满足。
pub trait BlockRegistry: Send {
    fn id_of(&self, name: &str) -> Option<BlockId>;
    fn name_of(&self, id: BlockId) -> Option<&str>;
    /// 是否参与碰撞与暴露面判定(空气与非实心方块为 false)。
    fn is_solid(&self, id: BlockId) -> bool;
    /// 已注册方块数(有效 ID 范围 0..block_count)。
    fn block_count(&self) -> u16;
    /// 方块材料(REQ-009 §3.1):实心方块恒为 `Some`;空气、非实心方块与未知 ID 为 `None`。
    fn material(&self, id: BlockId) -> Option<Material>;
}

/// 方块的支撑规则(REQ-009 §3.1、§6)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Support {
    /// 与地基连通即不落(石、土)。
    Connected,
    /// 正下方不是实心就落,与连通性无关(沙)。
    NeedsBelow,
    /// 本身就是地基,坍塌搜索遇到即判定锚定(基岩)。
    Anchor,
}

/// 承载粘结的"∞"(REQ-010 §3.1):不会被拉断。
pub const BOND_INFINITE: u32 = u32::MAX;

/// 方块材料(REQ-009 §3.1;REQ-010 §3.1 追加 `bond`)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Material {
    /// 抗爆强度 R,与爆炸压力同单位;`f32::INFINITY` = 不可破坏。
    pub resistance: f32,
    /// 撞击破坏所需的单格冲量 I,单位 kg·m/s;`f32::INFINITY` = 撞不坏。
    pub impact_threshold: f32,
    pub support: Support,
    /// 粉碎倍数 k:爆炸压力 > k·R 粉碎,R < 压力 ≤ k·R 脱落。
    pub pulverize_factor: f32,
    /// 承载粘结 B(REQ-010 §3.1):一个共面连接能传递的静载,单位为格重 `W0`;
    /// [`BOND_INFINITE`] = 不会被拉断。两格之间取较小值。
    pub bond: u32,
}

impl Material {
    /// 不可破坏:`SimpleRegistry::register` 注册的实心方块的默认材料(REQ-009 §3.2),
    /// 让测试用的自定义方块表行为不变。
    pub const INDESTRUCTIBLE: Self = Self {
        resistance: f32::INFINITY,
        impact_threshold: f32::INFINITY,
        support: Support::Connected,
        pulverize_factor: 2.0,
        bond: BOND_INFINITE,
    };
}

/// 内置方块 ID(`SimpleRegistry::builtin` 的注册顺序;只追加,存档兼容)。
pub mod builtin {
    use types::BlockId;

    pub const AIR: BlockId = BlockId::AIR;
    pub const STONE: BlockId = BlockId(1);
    pub const DIRT: BlockId = BlockId(2);
    /// REQ-009 新增:不可破坏,超平坦最底层。
    pub const BEDROCK: BlockId = BlockId(3);
    /// REQ-009 新增:`Support::NeedsBelow`。
    pub const SAND: BlockId = BlockId(4);
}

/// 注册失败原因。
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum SimpleRegistryError {
    #[error("方块名重复注册: {0}")]
    DuplicateName(String),
}

/// 顺序注册的简单注册表(测试与内置方块用)。
/// 0 号恒为 "air"(非实心)——`BlockId::AIR` 的语义由注册表保证。
#[derive(Debug)]
pub struct SimpleRegistry {
    /// id -> (name, solid, material);非实心方块材料为 `None`(REQ-009 §3.1)。
    entries: Vec<(String, bool, Option<Material>)>,
    /// name -> id
    index: BTreeMap<String, u16>,
}

impl SimpleRegistry {
    /// 空注册表 + 预注册的 air(0 号,非实心)。
    pub fn new() -> Self {
        let mut reg = Self {
            entries: Vec::new(),
            index: BTreeMap::new(),
        };
        reg.register("air", false).expect("内置 air 注册不可能重名");
        reg
    }

    /// 追加注册一个方块,返回其 ID。实心方块的默认材料为
    /// [`Material::INDESTRUCTIBLE`](REQ-009 §3.2:测试用的自定义方块表行为不变)。
    pub fn register(&mut self, name: &str, solid: bool) -> Result<BlockId, SimpleRegistryError> {
        let material = solid.then_some(Material::INDESTRUCTIBLE);
        self.push(name, solid, material)
    }

    /// 追加注册一个带材料的实心方块(REQ-009 §3.1)。
    pub fn register_with_material(
        &mut self,
        name: &str,
        material: Material,
    ) -> Result<BlockId, SimpleRegistryError> {
        self.push(name, true, Some(material))
    }

    /// 追加入口:重名拒绝且不占 ID。
    fn push(
        &mut self,
        name: &str,
        solid: bool,
        material: Option<Material>,
    ) -> Result<BlockId, SimpleRegistryError> {
        if self.index.contains_key(name) {
            return Err(SimpleRegistryError::DuplicateName(name.to_owned()));
        }
        let id = self.entries.len() as u16;
        self.entries.push((name.to_owned(), solid, material));
        self.index.insert(name.to_owned(), id);
        Ok(BlockId(id))
    }

    /// 内置方块表(REQ-009 §3.2,ID 只追加):air(0)、stone(1, R6/I18)、
    /// dirt(2, R2/I7)、bedrock(3, ∞/∞, Anchor)、sand(4, R2/I4, NeedsBelow);
    /// 全部材料 k = 2。承载粘结 B(REQ-010 §3.2):stone 8、dirt 3、bedrock ∞、sand 0。
    pub fn builtin() -> Self {
        let mut reg = Self::new();
        let mut add = |name: &str, material: Material| {
            reg.register_with_material(name, material)
                .expect("内置方块注册不可能重名")
        };
        add(
            "stone",
            Material {
                resistance: 6.0,
                impact_threshold: 18.0,
                support: Support::Connected,
                pulverize_factor: 2.0,
                bond: 8,
            },
        );
        add(
            "dirt",
            Material {
                resistance: 2.0,
                impact_threshold: 7.0,
                support: Support::Connected,
                pulverize_factor: 2.0,
                bond: 3,
            },
        );
        add(
            "bedrock",
            Material {
                resistance: f32::INFINITY,
                impact_threshold: f32::INFINITY,
                support: Support::Anchor,
                pulverize_factor: 2.0,
                bond: BOND_INFINITE,
            },
        );
        add(
            "sand",
            Material {
                resistance: 2.0,
                impact_threshold: 4.0,
                support: Support::NeedsBelow,
                pulverize_factor: 2.0,
                bond: 0,
            },
        );
        reg
    }
}

impl Default for SimpleRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl BlockRegistry for SimpleRegistry {
    fn id_of(&self, name: &str) -> Option<BlockId> {
        self.index.get(name).map(|id| BlockId(*id))
    }

    fn name_of(&self, id: BlockId) -> Option<&str> {
        self.entries
            .get(usize::from(id.0))
            .map(|(name, _, _)| name.as_str())
    }

    fn is_solid(&self, id: BlockId) -> bool {
        self.entries
            .get(usize::from(id.0))
            .is_some_and(|(_, solid, _)| *solid)
    }

    fn block_count(&self) -> u16 {
        self.entries.len() as u16
    }

    /// 实心方块恒为 `Some`;空气、非实心方块与未知 ID 为 `None`(REQ-009 §3.1)。
    fn material(&self, id: BlockId) -> Option<Material> {
        self.entries
            .get(usize::from(id.0))
            .and_then(|(_, _, material)| *material)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_has_air_at_zero() {
        let reg = SimpleRegistry::new();
        assert_eq!(reg.block_count(), 1);
        assert_eq!(reg.id_of("air"), Some(BlockId::AIR));
        assert_eq!(reg.name_of(BlockId::AIR), Some("air"));
        assert!(!reg.is_solid(BlockId::AIR));
    }

    #[test]
    fn register_and_lookup_roundtrip() {
        let mut reg = SimpleRegistry::new();
        let stone = reg.register("stone", true).unwrap();
        assert_eq!(stone, BlockId(1));
        assert_eq!(reg.id_of("stone"), Some(stone));
        assert_eq!(reg.name_of(stone), Some("stone"));
        assert!(reg.is_solid(stone));
        assert_eq!(reg.block_count(), 2);
    }

    #[test]
    fn duplicate_name_is_rejected() {
        let mut reg = SimpleRegistry::new();
        reg.register("stone", true).unwrap();
        assert_eq!(
            reg.register("stone", false),
            Err(SimpleRegistryError::DuplicateName("stone".into()))
        );
        // 失败注册不占用 ID
        assert_eq!(reg.register("dirt", true), Ok(BlockId(2)));
    }

    #[test]
    fn unknown_lookups_are_none_and_not_solid() {
        let reg = SimpleRegistry::builtin();
        assert_eq!(reg.id_of("不存在"), None);
        assert_eq!(reg.name_of(BlockId(99)), None);
        assert!(!reg.is_solid(BlockId(99)));
    }

    #[test]
    fn builtin_ids_are_stable() {
        let reg = SimpleRegistry::builtin();
        assert_eq!(reg.id_of("air"), Some(BlockId(0)));
        assert_eq!(reg.id_of("stone"), Some(BlockId(1)));
        assert_eq!(reg.id_of("dirt"), Some(BlockId(2)));
        // REQ-009 §3.2 追加:bedrock(3)、sand(4)。
        assert_eq!(reg.id_of("bedrock"), Some(BlockId(3)));
        assert_eq!(reg.id_of("sand"), Some(BlockId(4)));
        assert!(!reg.is_solid(BlockId(0)));
        assert!(reg.is_solid(BlockId(1)) && reg.is_solid(BlockId(2)));
        assert!(reg.is_solid(BlockId(3)) && reg.is_solid(BlockId(4)));
    }

    /// REQ-009 §3.2:`register` 注册的实心方块默认不可破坏(既有自定义方块表行为不变),
    /// 非实心与未知 ID 没有材料。
    #[test]
    fn register_solid_defaults_indestructible() {
        let mut reg = SimpleRegistry::new();
        let solid = reg.register("wall", true).unwrap();
        let nonsolid = reg.register("water", false).unwrap();
        assert_eq!(reg.material(solid), Some(Material::INDESTRUCTIBLE));
        assert_eq!(reg.material(nonsolid), None, "非实心没有材料");
        assert_eq!(reg.material(BlockId(99)), None, "未知 ID 没有材料");
    }

    /// REQ-009 §3.1:`register_with_material` 注册的方块是实心、材料原样返回;
    /// 重名同样报错且不占 ID。
    #[test]
    fn register_with_material_roundtrip_and_duplicate() {
        let mut reg = SimpleRegistry::new();
        let glass = Material {
            resistance: 1.0,
            impact_threshold: 3.0,
            support: Support::Connected,
            pulverize_factor: 2.0,
            bond: 5,
        };
        let id = reg.register_with_material("glass", glass).unwrap();
        assert_eq!(id, BlockId(1));
        assert!(reg.is_solid(id), "带材料注册的方块是实心");
        assert_eq!(reg.material(id), Some(glass));
        assert_eq!(
            reg.register_with_material("glass", glass),
            Err(SimpleRegistryError::DuplicateName("glass".into()))
        );
        assert_eq!(
            reg.register("stone", true),
            Ok(BlockId(2)),
            "重名失败不占 ID"
        );
    }

    /// REQ-009 §3.2 内置材料数值:stone 6/18、dirt 2/7、bedrock ∞/∞ + Anchor、
    /// sand 2/4 + NeedsBelow;全部 k = 2。
    #[test]
    fn builtin_materials_match_req009() {
        let reg = SimpleRegistry::builtin();
        let mat = |id: BlockId| reg.material(id).expect("内置实心方块必有材料");
        let stone = mat(BlockId(1));
        assert_eq!(stone.resistance, 6.0);
        assert_eq!(stone.impact_threshold, 18.0);
        assert_eq!(stone.support, Support::Connected);
        let dirt = mat(BlockId(2));
        assert_eq!(dirt.resistance, 2.0);
        assert_eq!(dirt.impact_threshold, 7.0);
        let bedrock = mat(BlockId(3));
        assert_eq!(bedrock.resistance, f32::INFINITY);
        assert_eq!(bedrock.impact_threshold, f32::INFINITY);
        assert_eq!(bedrock.support, Support::Anchor);
        let sand = mat(BlockId(4));
        assert_eq!(sand.resistance, 2.0);
        assert_eq!(sand.impact_threshold, 4.0);
        assert_eq!(sand.support, Support::NeedsBelow);
        for id in [BlockId(1), BlockId(2), BlockId(3), BlockId(4)] {
            assert_eq!(mat(id).pulverize_factor, 2.0, "全部材料 k = 2");
        }
    }
}
