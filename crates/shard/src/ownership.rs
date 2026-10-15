//! OwnershipMap:claim/release/owner(BTreeMap),重复 claim fail-stop(蓝图 §3)。

use std::collections::BTreeMap;

use types::{ShardId, Tick};

/// 一次 claim 的记录。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClaimRecord {
    /// 声明发生的拍。
    pub since_tick: Tick,
}

/// 分片所有权表:哪些分片当前被系统持有(BTreeMap,确定性迭代)。
/// 重复 claim / 释放未持有 = 一致性违规 → panic(fail-stop,蓝图 §4)。
#[derive(Debug, Default)]
pub struct OwnershipMap {
    claims: BTreeMap<ShardId, ClaimRecord>,
}

impl OwnershipMap {
    /// 空表。
    pub fn new() -> Self {
        Self::default()
    }

    /// 声明所有权。已持有 → panic(fail-stop:同分片同拍至多一持有者,AC5)。
    pub fn claim(&mut self, shard: ShardId, at: Tick) {
        // 先查后插:panic 时表保持不变(fail-stop 不污染状态)。
        if let Some(old) = self.claims.get(&shard) {
            panic!(
                "重复 claim:分片 {:?} 已自 tick {} 被持有,不能在 tick {} 再次声明",
                shard, old.since_tick.0, at.0
            );
        }
        self.claims.insert(shard, ClaimRecord { since_tick: at });
    }

    /// 释放所有权。未持有 → panic(fail-stop:释放从未持有的分片是调用方 bug)。
    pub fn release(&mut self, shard: ShardId) {
        if self.claims.remove(&shard).is_none() {
            panic!("释放未持有的分片: {:?}", shard);
        }
    }

    /// 查询(未持有返回 None;查询永不 panic)。
    pub fn owner(&self, shard: ShardId) -> Option<ClaimRecord> {
        self.claims.get(&shard).copied()
    }

    /// 是否被持有(永不 panic)。
    pub fn contains(&self, shard: ShardId) -> bool {
        self.claims.contains_key(&shard)
    }

    /// 当前持有数。
    pub fn len(&self) -> usize {
        self.claims.len()
    }

    /// 是否一无所有。
    pub fn is_empty(&self) -> bool {
        self.claims.is_empty()
    }

    /// 确定性迭代(ShardId 升序,BTreeMap 保证)。
    pub fn iter(&self) -> impl Iterator<Item = (ShardId, ClaimRecord)> + '_ {
        self.claims.iter().map(|(&id, &record)| (id, record))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::{DimensionId, ShardPos, WorldId};

    /// 以 (x, z) 构造默认世界/维度 0 的分片 ID,便于测试书写。
    fn shard(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    #[test]
    fn claim_then_query_roundtrip() {
        let mut map = OwnershipMap::new();
        map.claim(shard(1, 2), Tick(7));
        assert_eq!(
            map.owner(shard(1, 2)),
            Some(ClaimRecord {
                since_tick: Tick(7)
            })
        );
    }

    #[test]
    #[should_panic(expected = "重复 claim")]
    fn duplicate_claim_panics() {
        let mut map = OwnershipMap::new();
        map.claim(shard(0, 0), Tick(1));
        map.claim(shard(0, 0), Tick(2)); // 已持有 → fail-stop
    }

    #[test]
    #[should_panic(expected = "未持有")]
    fn release_unclaimed_panics() {
        let mut map = OwnershipMap::new();
        map.release(shard(5, 5)); // 从未 claim → fail-stop
    }

    #[test]
    fn release_allows_reclaim() {
        let mut map = OwnershipMap::new();
        map.claim(shard(3, 4), Tick(10));
        map.release(shard(3, 4));
        assert_eq!(map.owner(shard(3, 4)), None);
        map.claim(shard(3, 4), Tick(20));
        assert_eq!(
            map.owner(shard(3, 4)),
            Some(ClaimRecord {
                since_tick: Tick(20)
            })
        );
    }

    #[test]
    fn iteration_is_deterministic() {
        let mut map = OwnershipMap::new();
        map.claim(shard(2, 0), Tick(1));
        map.claim(shard(0, 5), Tick(1));
        map.claim(shard(1, 9), Tick(1));
        let order: Vec<ShardId> = map.iter().map(|(id, _)| id).collect();
        assert_eq!(order, vec![shard(0, 5), shard(1, 9), shard(2, 0)]);
    }

    #[test]
    fn len_tracks_claims() {
        let mut map = OwnershipMap::new();
        assert!(map.is_empty());
        assert_eq!(map.len(), 0);
        map.claim(shard(0, 0), Tick(1));
        map.claim(shard(0, 1), Tick(1));
        assert_eq!(map.len(), 2);
        assert!(!map.is_empty());
        map.release(shard(0, 0));
        assert_eq!(map.len(), 1);
        map.release(shard(0, 1));
        assert!(map.is_empty());
    }
}
