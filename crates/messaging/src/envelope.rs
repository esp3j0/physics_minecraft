//! 跨片信封:分片产出 Intent → 仅调度器可盖章为 Envelope → 有界确定性队列 →
//! 恰好在 apply_at 拍作为目的分片的输入前缀应用(照 v2 cross_shard.rs,REQ-001 §4.2 (a))。

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use types::{EntityDesc, EntityId, EntityState, ShardId, Tick};

/// 队列默认容量(REQ-001 §4.2 决议:1024)。
pub const DEFAULT_QUEUE_CAPACITY: usize = 1024;

/// 信封载荷。变体只允许**追加**。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum EnvelopePayload {
    /// 实体迁移(REQ-001 §4.2 (b)):tick N 末提取的四向量状态 + 实体描述。
    MigrateEntity {
        entity: EntityId,
        state: EntityState,
        desc: EntityDesc,
    },
}

impl EnvelopePayload {
    /// 枚举声明序(canonical 类型序号的低位)。
    pub const fn ordinal(self) -> u16 {
        match self {
            Self::MigrateEntity { .. } => 0,
        }
    }
}

/// 分片产出的跨片意图(未经调度器盖章,分片内部不可投递)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Intent {
    pub destination: ShardId,
    pub payload: EnvelopePayload,
}

/// 盖章后的信封。`apply_at ≡ produced_at + 1`(恒下一拍,v2 语义)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Envelope {
    pub produced_at: Tick,
    pub apply_at: Tick,
    pub source: ShardId,
    pub destination: ShardId,
    pub source_sequence: u64,
    pub payload: EnvelopePayload,
}

impl Envelope {
    /// 盖章:仅调度器调用(分片只产出 Intent)。
    pub fn stamp(source: ShardId, intent: Intent, produced_at: Tick, source_sequence: u64) -> Self {
        Self {
            apply_at: produced_at.next(),
            produced_at,
            source,
            destination: intent.destination,
            source_sequence,
            payload: intent.payload,
        }
    }
}

/// 准入拒绝原因(队列级;UnknownDestination/DestinationNotActive 属调度器层)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum AdmitError {
    #[error("跨片队列已满(容量 {capacity})")]
    QueueFull { capacity: usize },
    #[error("源与目的为同一分片")]
    SameShard,
    #[error("apply_at 必须等于 produced_at + 1")]
    NoNextTick,
}

/// 跨片队列:有界、准入不驱逐、按 (destination, apply_at) 分桶、桶内 canonical 序。
#[derive(Debug)]
pub struct CrossShardQueue {
    capacity: usize,
    buckets: BTreeMap<(ShardId, Tick), Vec<Envelope>>,
}

impl CrossShardQueue {
    pub fn new(capacity: usize) -> Self {
        Self {
            capacity,
            buckets: BTreeMap::new(),
        }
    }

    /// 默认容量(1024)。
    pub fn with_default_capacity() -> Self {
        Self::new(DEFAULT_QUEUE_CAPACITY)
    }

    /// 在队信封总数。
    pub fn len(&self) -> usize {
        self.buckets.values().map(Vec::len).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.buckets.values().all(|v| v.is_empty())
    }

    /// 准入一枚信封。不驱逐已有条目;满则拒绝(反压由调度器处理)。
    pub fn admit(&mut self, envelope: Envelope) -> Result<(), AdmitError> {
        if envelope.source == envelope.destination {
            return Err(AdmitError::SameShard);
        }
        if envelope.apply_at != envelope.produced_at.next() {
            return Err(AdmitError::NoNextTick);
        }
        if self.len() >= self.capacity {
            return Err(AdmitError::QueueFull {
                capacity: self.capacity,
            });
        }
        self.buckets
            .entry((envelope.destination, envelope.apply_at))
            .or_default()
            .push(envelope);
        Ok(())
    }

    /// 取走 destination 在 apply_at 拍的全部信封(桶内按 payload 序号 → source →
    /// source_sequence 排序)。**恰好一次**:取走即移除,再次取为空。
    pub fn take_batch(&mut self, destination: ShardId, apply_at: Tick) -> Vec<Envelope> {
        match self.buckets.remove(&(destination, apply_at)) {
            Some(mut batch) => {
                batch.sort_by(|a, b| {
                    (a.payload.ordinal(), a.source, a.source_sequence).cmp(&(
                        b.payload.ordinal(),
                        b.source,
                        b.source_sequence,
                    ))
                });
                batch
            }
            None => Vec::new(),
        }
    }
}

impl Default for CrossShardQueue {
    fn default() -> Self {
        Self::with_default_capacity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::{DimensionId, WorldId};

    fn shard(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), types::ShardPos::new(x, z))
    }

    fn migrate(seq: u64, source: ShardId, destination: ShardId) -> Envelope {
        Envelope::stamp(
            source,
            Intent {
                destination,
                payload: EnvelopePayload::MigrateEntity {
                    entity: EntityId::from_parts(1, seq as u32),
                    state: EntityState::default(),
                    desc: EntityDesc::default(),
                },
            },
            Tick(10),
            seq,
        )
    }

    #[test]
    fn payload_ordinals_follow_declared_order() {
        let p = EnvelopePayload::MigrateEntity {
            entity: EntityId::from_parts(1, 1),
            state: EntityState::default(),
            desc: EntityDesc::default(),
        };
        assert_eq!(p.ordinal(), 0);
    }

    #[test]
    fn stamp_applies_exactly_next_tick() {
        let e = migrate(1, shard(0, 0), shard(1, 0));
        assert_eq!(e.produced_at, Tick(10));
        assert_eq!(e.apply_at, Tick(11), "apply_at ≡ produced_at + 1");
        assert_eq!(e.source, shard(0, 0));
        assert_eq!(e.destination, shard(1, 0));
    }

    #[test]
    fn admit_rejects_same_shard() {
        let mut q = CrossShardQueue::with_default_capacity();
        let e = migrate(1, shard(0, 0), shard(0, 0));
        assert_eq!(q.admit(e), Err(AdmitError::SameShard));
    }

    #[test]
    fn admit_rejects_wrong_apply_at() {
        let mut q = CrossShardQueue::with_default_capacity();
        let mut e = migrate(1, shard(0, 0), shard(1, 0));
        e.apply_at = Tick(10); // 破坏 produced_at+1 不变量
        assert_eq!(q.admit(e), Err(AdmitError::NoNextTick));
    }

    #[test]
    fn queue_full_rejects_without_evicting() {
        let mut q = CrossShardQueue::new(2);
        assert!(q.admit(migrate(1, shard(0, 0), shard(1, 0))).is_ok());
        assert!(q.admit(migrate(2, shard(0, 0), shard(1, 0))).is_ok());
        assert_eq!(
            q.admit(migrate(3, shard(0, 0), shard(1, 0))),
            Err(AdmitError::QueueFull { capacity: 2 })
        );
        assert_eq!(q.len(), 2, "满拒绝不驱逐已有条目");
        assert_eq!(q.take_batch(shard(1, 0), Tick(11)).len(), 2);
    }

    #[test]
    fn take_batch_is_canonical_and_exactly_once() {
        let mut q = CrossShardQueue::with_default_capacity();
        let dest = shard(1, 0);
        // 乱序准入:seq 2 先进,来源也不同
        q.admit(migrate(2, shard(5, 5), dest)).unwrap();
        q.admit(migrate(1, shard(5, 5), dest)).unwrap();
        q.admit(migrate(9, shard(1, 1), dest)).unwrap();
        // 其他目的/其他 tick 不受影响
        q.admit(migrate(7, shard(0, 0), shard(9, 9))).unwrap();

        let batch = q.take_batch(dest, Tick(11));
        assert_eq!(batch.len(), 3);
        // 桶内序:source(ShardId 序)先于 source_sequence
        assert_eq!(batch[0].source, shard(1, 1));
        assert_eq!(batch[1].source_sequence, 1);
        assert_eq!(batch[2].source_sequence, 2);
        // 恰好一次
        assert!(q.take_batch(dest, Tick(11)).is_empty());
        // 其他桶仍在
        assert_eq!(q.len(), 1);
    }
}
