//! chunk 持有表(REQ-005 D2/D4):登记"哪些分片持有一份 chunk 副本"。
//!
//! **owner 是纯函数**(`ChunkPos::shard_of()` + world/dim,见
//! `Scheduler::owner_of`),不入表;本表只登记可变的 **holder 集合**
//! (owner 永远在 holder 里,只要 owner 加载着它)。表不入快照——恢复时
//! 从各分片已加载 chunk 重建([`ChunkHolders::rebuild`],纯函数)。
//!
//! 卸载/持久化归属(D6)由调度器按 `owner_of` 判定,本表只回答
//! "谁持有副本"。

use std::collections::{BTreeMap, BTreeSet};

use types::{ChunkPos, ShardId};

/// holder 登记:`(owner, chunk) → 持有分片集合`(owner = chunk 的本体分片)。
#[derive(Debug, Default, Clone)]
pub struct ChunkHolders {
    holders: BTreeMap<(ShardId, ChunkPos), BTreeSet<ShardId>>,
}

impl ChunkHolders {
    pub fn new() -> Self {
        Self::default()
    }

    /// 登记 `holder` 持有 `(owner, pos)` 的一份副本(BTreeSet 天然去重,
    /// 重复登记幂等)。
    pub fn register(&mut self, owner: ShardId, pos: ChunkPos, holder: ShardId) {
        self.holders.entry((owner, pos)).or_default().insert(holder);
    }

    /// 移除 `holder` 对 `(owner, pos)` 的持有;集合空 → 删除条目。
    pub fn unregister(&mut self, owner: ShardId, pos: ChunkPos, holder: ShardId) {
        if let Some(set) = self.holders.get_mut(&(owner, pos)) {
            set.remove(&holder);
            if set.is_empty() {
                self.holders.remove(&(owner, pos));
            }
        }
    }

    /// 持有 `(owner, pos)` 副本的分片集合(含 owner,若它加载着);无 → 空集。
    pub fn holders(&self, owner: ShardId, pos: ChunkPos) -> BTreeSet<ShardId> {
        self.holders.get(&(owner, pos)).cloned().unwrap_or_default()
    }

    /// 出现过的 owner 去重集合(校验"owner 必须已注册"用)。
    pub fn owners(&self) -> BTreeSet<ShardId> {
        self.holders.keys().map(|(owner, _)| *owner).collect()
    }

    /// 某分片当前持有副本的全部 chunk(REQ-005 任务 03:覆盖判定按持有
    /// chunk 枚举;表内条目即"该分片已装载此 chunk",ChunkPos 升序集合)。
    pub fn chunks_of(&self, holder: ShardId) -> BTreeSet<ChunkPos> {
        self.holders
            .iter()
            .filter(|(_, holders)| holders.contains(&holder))
            .map(|((_, pos), _)| *pos)
            .collect()
    }

    /// 从"各分片 → 已加载 chunk"重建(恢复/回放用;先清空)。
    /// owner 由调用方按纯函数换算传入,本方法不做换算(保持纯登记)。
    pub fn rebuild<'a>(
        &mut self,
        loaded: impl Iterator<
            Item = (
                ShardId,
                &'a [ChunkPos],
                Box<dyn Fn(ChunkPos) -> ShardId + 'a>,
            ),
        >,
    ) {
        self.holders.clear();
        for (shard, chunks, owner_of) in loaded {
            for &cp in chunks {
                let owner = owner_of(cp);
                self.register(owner, cp, shard);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::{DimensionId, WorldId};

    fn shard(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), types::ShardPos::new(x, z))
    }

    fn pos(x: i32, z: i32) -> ChunkPos {
        ChunkPos::new(x, z)
    }

    /// 任务书 req005/00 测试 4(holder 层):登记后 holder 含 shard;
    /// 重复登记不重复;移除后条目删除。
    #[test]
    fn holders_register_unregister_idempotent() {
        let mut h = ChunkHolders::new();
        h.register(shard(0, 0), pos(8, 0), shard(1, 0));
        assert_eq!(
            h.holders(shard(0, 0), pos(8, 0)),
            BTreeSet::from([shard(1, 0)])
        );
        // 重复登记幂等(BTreeSet)。
        h.register(shard(0, 0), pos(8, 0), shard(1, 0));
        assert_eq!(h.holders(shard(0, 0), pos(8, 0)).len(), 1);
        // owner 也加载 → 两个 holder。
        h.register(shard(0, 0), pos(8, 0), shard(0, 0));
        assert_eq!(
            h.holders(shard(0, 0), pos(8, 0)),
            BTreeSet::from([shard(0, 0), shard(1, 0)])
        );
        assert_eq!(h.owners(), BTreeSet::from([shard(0, 0)]));
        // 逐个移除;全空后条目消失(holders 返回空集)。
        h.unregister(shard(0, 0), pos(8, 0), shard(1, 0));
        assert_eq!(
            h.holders(shard(0, 0), pos(8, 0)),
            BTreeSet::from([shard(0, 0)])
        );
        h.unregister(shard(0, 0), pos(8, 0), shard(0, 0));
        assert!(h.holders(shard(0, 0), pos(8, 0)).is_empty());
        assert!(h.owners().is_empty());
    }

    /// rebuild 先清空再按输入重建(owner 由调用方换算)。
    #[test]
    fn holders_rebuild_from_loaded() {
        let mut h = ChunkHolders::new();
        h.register(shard(5, 5), pos(0, 0), shard(5, 5)); // 旧残留应被清掉
        let shard0 = shard(0, 0);
        let chunks = vec![pos(0, 0), pos(8, 0)];
        h.rebuild(std::iter::once((
            shard0,
            chunks.as_slice(),
            Box::new(|cp: ChunkPos| ShardId::new(WorldId(0), DimensionId(0), cp.shard_of()))
                as Box<dyn Fn(ChunkPos) -> ShardId>,
        )));
        assert!(h.holders(shard(0, 0), pos(0, 0)).contains(&shard0));
        // chunk(8,0) 的 owner 是分片 (1,0),holder 是分片 (0,0)。
        assert_eq!(h.holders(shard(1, 0), pos(8, 0)), BTreeSet::from([shard0]));
    }
}
