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

/// 方块材料(REQ-009 §3.1)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Material {
    /// 抗爆强度 R,与爆炸压力同单位;`f32::INFINITY` = 不可破坏。
    pub resistance: f32,
    /// 撞击破坏所需的单格冲量 I,单位 kg·m/s;`f32::INFINITY` = 撞不坏。
    pub impact_threshold: f32,
    pub support: Support,
    /// 粉碎倍数 k:爆炸压力 > k·R 粉碎,R < 压力 ≤ k·R 脱落。
    pub pulverize_factor: f32,
}

impl Material {
    /// 不可破坏:`SimpleRegistry::register` 注册的实心方块的默认材料(REQ-009 §3.2),
    /// 让测试用的自定义方块表行为不变。
    pub const INDESTRUCTIBLE: Self = Self {
        resistance: f32::INFINITY,
        impact_threshold: f32::INFINITY,
        support: Support::Connected,
        pulverize_factor: 2.0,
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
    /// id -> (name, solid)
    entries: Vec<(String, bool)>,
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

    /// 追加注册一个方块,返回其 ID。
    pub fn register(&mut self, name: &str, solid: bool) -> Result<BlockId, SimpleRegistryError> {
        if self.index.contains_key(name) {
            return Err(SimpleRegistryError::DuplicateName(name.to_owned()));
        }
        let id = self.entries.len() as u16;
        self.entries.push((name.to_owned(), solid));
        self.index.insert(name.to_owned(), id);
        Ok(BlockId(id))
    }

    /// 追加注册一个带材料的实心方块(REQ-009 §3.1)。
    ///
    /// **总架构师接口桩(REQ-009 单元 01)**:签名固定,由实施方实现。
    pub fn register_with_material(
        &mut self,
        name: &str,
        material: Material,
    ) -> Result<BlockId, SimpleRegistryError> {
        let _ = (name, material);
        unimplemented!("REQ-009 单元 01:由实施方实现")
    }

    /// 内置方块表:air(0)、stone(1)、dirt(2);REQ-009 起追加 bedrock(3)、sand(4),
    /// 材料数值见 REQ-009 §3.2(**由实施方补齐**)。
    pub fn builtin() -> Self {
        let mut reg = Self::new();
        reg.register("stone", true).expect("内置方块注册不可能重名");
        reg.register("dirt", true).expect("内置方块注册不可能重名");
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
            .map(|(name, _)| name.as_str())
    }

    fn is_solid(&self, id: BlockId) -> bool {
        self.entries
            .get(usize::from(id.0))
            .is_some_and(|(_, solid)| *solid)
    }

    fn block_count(&self) -> u16 {
        self.entries.len() as u16
    }

    /// **总架构师接口桩(REQ-009 单元 01)**:由实施方实现。
    fn material(&self, id: BlockId) -> Option<Material> {
        let _ = id;
        unimplemented!("REQ-009 单元 01:由实施方实现")
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
        assert!(!reg.is_solid(BlockId(0)));
        assert!(reg.is_solid(BlockId(1)) && reg.is_solid(BlockId(2)));
    }
}
