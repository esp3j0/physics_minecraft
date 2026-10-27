//! 跨片信封:分片产出 Intent → 仅调度器可盖章为 Envelope → 有界确定性队列 →
//! 恰好在 apply_at 拍作为目的分片的输入前缀应用(照 v2 cross_shard.rs,REQ-001 §4.2 (a))。

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use types::{BlockId, BlockPos, EntityDesc, EntityId, EntityState, ShardId, Tick};

use crate::event::CrossEvent;

/// 队列默认容量(REQ-001 §4.2 决议 1024;**REQ-009 修订 10 改为 8192**:一次
/// 爆炸至多 `max_cells_per_explosion` = 2048 格 × 至多 3 个持副本的邻片,另留余量)。
pub const DEFAULT_QUEUE_CAPACITY: usize = 8192;

/// 信封载荷。变体只允许**追加**。
/// REQ-010 §5.4:MigrateEntity 的 `EntityDesc` 不再 `Copy`,本枚举随之只有 `Clone`。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum EnvelopePayload {
    /// 实体迁移(REQ-001 §4.2 (b)):tick N 末提取的四向量状态 + 实体描述 +
    /// 政策状态(REQ-006:actor/source 必须随迁移保留)。
    MigrateEntity {
        entity: EntityId,
        state: EntityState,
        desc: EntityDesc,
        /// 演员政策标记(REQ-002 决议 4b);迁移后不得丢。
        actor: bool,
        /// 实体来源:外部 spawn = 玩家;服务端/内部 = None。
        source: Option<crate::InputSource>,
        /// 常驻运动票持票记账/触顶标记(REQ-007 任务 04a 遗留返修):
        /// 迁移带走(与 actor/source 同款),否则触顶实体借迁移绕过上限
        /// ——目的分片 claim 后计数从零、重新签发。
        actor_motion_ticks: Option<u64>,
    },
    /// 涌现事件(REQ-005 §4.3.2/D10):共享参数,各片本地求值;apply_at = N+1。
    Event(CrossEvent),
    /// 涌现 block write(REQ-005 §4.3.3):owner + holders 应用;apply_at = N+1;
    /// 应用后不再产 `block_updates`(防广播回环)。
    BlockWrite {
        pos: BlockPos,
        block: BlockId,
        /// 承载转换标记(REQ-010 §9):接收方据此在本拍多求一次"转换前"的 F。
        load_conversion: bool,
    },
}

impl EnvelopePayload {
    /// 枚举声明序(canonical 类型序号的低位)。
    pub const fn ordinal(&self) -> u16 {
        match self {
            Self::MigrateEntity { .. } => 0,
            Self::Event(_) => 1,
            Self::BlockWrite { .. } => 2,
        }
    }
}

/// 分片产出的跨片意图(未经调度器盖章,分片内部不可投递)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Intent {
    pub destination: ShardId,
    pub payload: EnvelopePayload,
}

/// 盖章后的信封。`apply_at ≡ produced_at + 1`(恒下一拍,v2 语义)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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

    /// 默认容量(`DEFAULT_QUEUE_CAPACITY`,修订 10 起 8192)。
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

    /// 仍在队的全部目的分片(升序;桶键即真相,REQ-005 D5 驱动 ensure 用)。
    /// 桶数 ≤ 队列容量,每次全扫可接受。
    pub fn destinations(&self) -> std::collections::BTreeSet<ShardId> {
        self.buckets.keys().map(|(dest, _)| *dest).collect()
    }

    /// destination 仍在队信封的最早 apply_at(无 → None)。
    /// 与 `destinations()` 同源(桶键),不存在陈旧条目问题。
    pub fn earliest_apply_at(&self, destination: ShardId) -> Option<Tick> {
        self.buckets
            .keys()
            .filter(|(dest, _)| *dest == destination)
            .map(|(_, apply_at)| *apply_at)
            .min()
    }

    /// 全部在队信封汇总(REQ-005 D7:Router 快照载荷),按
    /// `(destination, apply_at, payload ordinal, source, source_sequence)`
    /// 升序输出——桶内已 canonical(take_batch 同键),跨桶再排一次即全序;
    /// 恢复后的 take_batch 行为与原队逐位一致。
    pub fn snapshot(&self) -> Vec<Envelope> {
        let mut all: Vec<Envelope> = self.buckets.values().flatten().cloned().collect();
        all.sort_by_key(|e| {
            (
                e.destination,
                e.apply_at,
                e.payload.ordinal(),
                e.source,
                e.source_sequence,
            )
        });
        all
    }

    /// 从快照恢复(REQ-005 D7):逐封校验 `apply_at == produced_at + 1`(违反 =
    /// 内部状态损坏 → panic fail-stop)与队列容量(超出 = 损坏 → panic)后
    /// 按 `(destination, apply_at)` 放回对应桶。恢复即重建桶,不清旧(调用方
    /// 约定只在空队列上恢复);桶容量耗尽的判定与 `admit` 同口径(计数)。
    pub fn restore(&mut self, envelopes: Vec<Envelope>) {
        if envelopes.len() > self.capacity {
            panic!(
                "跨片队列恢复超容量:{} > {}(内部状态损坏,fail-stop)",
                envelopes.len(),
                self.capacity
            );
        }
        for envelope in envelopes {
            if envelope.apply_at != envelope.produced_at.next() {
                panic!(
                    "跨片队列恢复遇非法信封:apply_at {} ≠ produced_at {} + 1\
                     (内部状态损坏,fail-stop)",
                    envelope.apply_at.0, envelope.produced_at.0
                );
            }
            self.buckets
                .entry((envelope.destination, envelope.apply_at))
                .or_default()
                .push(envelope);
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
                    actor: false,
                    source: None,
                    actor_motion_ticks: None,
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
            actor: false,
            source: None,
            actor_motion_ticks: None,
        };
        assert_eq!(p.ordinal(), 0);
    }

    /// 任务书 03(messaging 连带):追加变体 Event/BlockWrite 的声明序 ——
    /// MigrateEntity=0、Event=1、BlockWrite=2(枚举只许追加,REQ-001 §9.6)。
    #[test]
    fn event_blockwrite_ordinals_appended_in_declared_order() {
        let event = EnvelopePayload::Event(CrossEvent::Detonate {
            center: types::glam::Vec3::ZERO,
            power: 1.0,
            radius: 1.0,
        });
        let write = EnvelopePayload::BlockWrite {
            pos: types::BlockPos::ZERO,
            block: types::BlockId(1),
            load_conversion: false,
        };
        assert_eq!(event.ordinal(), 1, "Event = 追加序 1(任务 03)");
        assert_eq!(write.ordinal(), 2, "BlockWrite = 追加序 2(任务 03)");
        let migrate = EnvelopePayload::MigrateEntity {
            entity: EntityId::from_parts(1, 1),
            state: EntityState::default(),
            desc: EntityDesc::default(),
            actor: false,
            source: None,
            actor_motion_ticks: None,
        };
        assert!(migrate.ordinal() < event.ordinal());
        assert!(event.ordinal() < write.ordinal(), "声明序 = 序号升序");
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

    // ---- REQ-005 任务书 02:跨拍队列快照/恢复(D7:Router 待投递信封入快照)----

    /// 测试 `queue_snapshot_restore_roundtrip`(任务书 02):多桶多来源在队 →
    /// snapshot() 全量汇总且按 (destination, apply_at, payload ordinal, source,
    /// source_sequence) canonical 全序;restore 到空队列 → 逐字段相等、
    /// take_batch 行为不变。
    #[test]
    fn queue_snapshot_restore_roundtrip() {
        let mut q = CrossShardQueue::with_default_capacity();
        // 三个桶:dest A(拍 11)、dest B(拍 11)、dest B(拍 12);来源交错。
        q.admit(migrate(3, shard(5, 5), shard(1, 0))).unwrap();
        q.admit(migrate(1, shard(1, 1), shard(1, 0))).unwrap();
        q.admit(migrate(2, shard(5, 5), shard(2, 0))).unwrap();
        q.admit(migrate(9, shard(0, 0), shard(2, 0))).unwrap();
        // dest B 拍 12:produced_at = 11。
        let mut later = migrate(4, shard(5, 5), shard(2, 0));
        later.produced_at = Tick(11);
        later.apply_at = Tick(12);
        q.admit(later).unwrap();

        let snap = q.snapshot();
        assert_eq!(snap.len(), 5, "全部在队信封汇总");
        // canonical 全序:跨桶按 (destination, apply_at, …)。
        let keys: Vec<_> = snap
            .iter()
            .map(|e| {
                (
                    e.destination,
                    e.apply_at,
                    e.payload.ordinal(),
                    e.source,
                    e.source_sequence,
                )
            })
            .collect();
        let mut sorted = keys.clone();
        sorted.sort();
        assert_eq!(keys, sorted, "snapshot 输出已按 canonical 全序排序");

        // 恢复到等容量空队列 → 逐封相等;分发行为不变。
        let mut restored = CrossShardQueue::with_default_capacity();
        restored.restore(snap);
        assert_eq!(restored.len(), 5, "恢复后条数一致");
        assert_eq!(
            restored.snapshot(),
            q.snapshot(),
            "恢复后快照与原队逐封相等"
        );
        // 分发语义不变:A 拍 11 的桶恰 2 封,canonical 序保持。
        let batch = restored.take_batch(shard(1, 0), Tick(11));
        assert_eq!(batch.len(), 2);
        assert_eq!(batch[0].source, shard(1, 1));
        assert_eq!(batch[1].source_sequence, 3);
        assert_eq!(restored.take_batch(shard(2, 0), Tick(12)).len(), 1);
    }

    /// 测试 `queue_restore_rejects_broken_invariant`:apply_at ≠ produced_at+1
    /// 的信封 = 内部状态损坏 → panic(fail-stop),不走类型化错误。
    #[test]
    #[should_panic(expected = "内部状态损坏")]
    fn queue_restore_rejects_broken_invariant() {
        let mut e = migrate(1, shard(0, 0), shard(1, 0));
        e.apply_at = Tick(10); // 破坏 produced_at + 1
        let mut q = CrossShardQueue::with_default_capacity();
        q.restore(vec![e]);
    }

    /// 测试 `queue_restore_respects_capacity`:恢复超过容量的集合 → panic
    /// (恢复路径的桶容量校验;快照本不可能超容,超容 = 损坏)。
    #[test]
    #[should_panic(expected = "容量")]
    fn queue_restore_respects_capacity() {
        let mut q = CrossShardQueue::new(1);
        q.restore(vec![
            migrate(1, shard(0, 0), shard(1, 0)),
            migrate(2, shard(0, 0), shard(1, 0)),
        ]);
    }

    /// REQ-009 修订 10(单元 07):默认队列容量 8192(一次爆炸至多 2048 格 ×
    /// 至多 3 个持副本的邻片,另留余量);`with_default_capacity` 跟随常量。
    ///
    /// 变红:常量回退 1024(默认配置下四分片交汇爆炸 fail-stop)。
    #[test]
    fn default_queue_capacity_is_8192() {
        assert_eq!(DEFAULT_QUEUE_CAPACITY, 8192);
        // 行为验证:默认队列恰能容纳 8192 枚,第 8193 枚被拒。
        let mut q = CrossShardQueue::with_default_capacity();
        for k in 0..8192u64 {
            q.admit(migrate(k + 1, shard(0, 0), shard(1, 0)))
                .expect("默认容量 ≥ 8192");
        }
        assert!(
            q.admit(migrate(9999, shard(0, 0), shard(1, 0))).is_err(),
            "第 8193 枚被拒(不多不少)"
        );
    }
}
