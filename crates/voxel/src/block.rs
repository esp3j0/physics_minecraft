//! 方块注册表。

use std::collections::BTreeMap;

use types::BlockId;

/// 方块注册表:名称 ↔ BlockId、方块属性。ID 分配后必须保持稳定(存档兼容)。
pub trait BlockRegistry {
    fn id_of(&self, name: &str) -> Option<BlockId>;
    fn name_of(&self, id: BlockId) -> Option<&str>;
    /// 是否参与碰撞与暴露面判定(空气与非实心方块为 false)。
    fn is_solid(&self, id: BlockId) -> bool;
    /// 已注册方块数(有效 ID 范围 0..block_count)。
    fn block_count(&self) -> u16;
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

    /// 内置方块表:air(0)、stone(1)、dirt(2)。
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
