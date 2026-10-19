//! 全局实体所有权索引(REQ-006 任务 00)。
//!
//! 真相仍是各 `ShardSim` 的实体表;本表只是 `EntityId → 分片/来源/actor` 的
//! 索引,供外部实体命令路由与恢复校验。表不入快照,恢复时 `rebuild`。

use std::collections::{BTreeMap, BTreeSet};

use messaging::InputSource;
use types::{EntityId, ShardId};

/// 实体的权威归属与政策元数据。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntityOwner {
    /// 当前持有该实体的分片(唯一)。
    pub shard: ShardId,
    /// 外部 spawn 的玩家;服务端/内部实体为 None。
    pub source: Option<InputSource>,
    /// 演员政策标记(REQ-002 决议 4b);迁移/传送保留。
    pub actor: bool,
}

/// `EntityId → EntityOwner` 索引。重复 claim / 缺失 release = fail-stop。
#[derive(Debug, Default, Clone)]
pub struct EntityOwnership {
    owners: BTreeMap<EntityId, EntityOwner>,
}

impl EntityOwnership {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn owner_of(&self, entity: EntityId) -> Option<EntityOwner> {
        self.owners.get(&entity).copied()
    }

    pub fn len(&self) -> usize {
        self.owners.len()
    }

    pub fn is_empty(&self) -> bool {
        self.owners.is_empty()
    }

    /// 登记唯一归属;已存在 → panic(全局唯一性 fail-stop)。
    pub fn claim(&mut self, entity: EntityId, owner: EntityOwner) {
        if let Some(prev) = self.owners.insert(entity, owner) {
            panic!(
                "实体 {entity:?} 重复 claim(已归 {:?},又归 {:?};全局唯一性 fail-stop)",
                prev.shard, owner.shard
            );
        }
    }

    /// 释放归属;不存在 → panic(内部一致性违规;hostile 路径必须先查 owner)。
    pub fn release(&mut self, entity: EntityId) {
        if self.owners.remove(&entity).is_none() {
            panic!("实体 {entity:?} 不存在却 release(内部一致性 fail-stop)");
        }
    }

    /// 更新已存在实体的元数据(owner 分片不得变);不存在 → panic。
    /// 用于 MarkActor 后同步 actor。
    pub fn update(&mut self, entity: EntityId, owner: EntityOwner) {
        let prev = self
            .owners
            .get_mut(&entity)
            .unwrap_or_else(|| panic!("实体 {entity:?} 不存在却 update(内部一致性)"));
        if prev.shard != owner.shard {
            panic!(
                "实体 {entity:?} update 改变 owner 分片({:?} → {:?};迁移必须 release+claim)",
                prev.shard, owner.shard
            );
        }
        *prev = owner;
    }

    /// 从各分片实体表重建;重复 EntityId → panic。
    pub fn rebuild(&mut self, entries: impl IntoIterator<Item = (EntityId, EntityOwner)>) {
        self.owners.clear();
        for (entity, owner) in entries {
            self.claim(entity, owner);
        }
    }

    pub fn entities_of(&self, shard: ShardId) -> BTreeSet<EntityId> {
        self.owners
            .iter()
            .filter_map(|(id, owner)| (owner.shard == shard).then_some(*id))
            .collect()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&EntityId, &EntityOwner)> {
        self.owners.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::{DimensionId, ShardPos, WorldId};

    fn sid(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    fn owner(shard: ShardId, source: Option<InputSource>, actor: bool) -> EntityOwner {
        EntityOwner {
            shard,
            source,
            actor,
        }
    }

    #[test]
    fn claim_release_roundtrip() {
        let mut own = EntityOwnership::new();
        let e = EntityId::from_parts(1, 1);
        own.claim(e, owner(sid(0, 0), None, false));
        assert_eq!(own.owner_of(e).map(|o| o.shard), Some(sid(0, 0)));
        own.release(e);
        assert!(own.owner_of(e).is_none());
        assert!(own.is_empty());
    }

    #[test]
    #[should_panic(expected = "重复 claim")]
    fn duplicate_claim_panics() {
        let mut own = EntityOwnership::new();
        let e = EntityId::from_parts(1, 1);
        own.claim(e, owner(sid(0, 0), None, false));
        own.claim(e, owner(sid(1, 0), None, false));
    }

    #[test]
    #[should_panic(expected = "不存在却 release")]
    fn missing_release_panics() {
        EntityOwnership::new().release(EntityId::from_parts(1, 1));
    }

    #[test]
    fn rebuild_entities_of_and_metadata() {
        let mut own = EntityOwnership::new();
        own.rebuild([
            (
                EntityId::from_parts(1, 1),
                owner(sid(0, 0), Some(InputSource(7)), true),
            ),
            (EntityId::from_parts(1, 2), owner(sid(1, 0), None, false)),
        ]);
        assert_eq!(own.len(), 2);
        assert_eq!(
            own.entities_of(sid(0, 0)),
            BTreeSet::from([EntityId::from_parts(1, 1)])
        );
        assert_eq!(
            own.owner_of(EntityId::from_parts(1, 1)),
            Some(owner(sid(0, 0), Some(InputSource(7)), true))
        );
    }
}
