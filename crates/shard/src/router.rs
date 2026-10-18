//! 命令路由:有界收件箱(满 = 类型化错误,反压断开权威输入)、信封盖章与分发
//! (tick N+1 输入前缀,蓝图 §3;照 v2 enforce_input_delivery)。
//!
//! Router 是调度器的输入面:本地命令走每分片有界收件箱;跨片 Intent 经
//! `Envelope::stamp` 入 [`CrossShardQueue`],下一拍作为目的分片的输入前缀,
//! 与收件箱中同拍命令按 canonical 序合并分发。

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use serde::{Deserialize, Serialize};

use messaging::{
    AdmitError, CrossShardQueue, DEFAULT_QUEUE_CAPACITY, Envelope, InputPayload, InputSource,
    Intent, StampedInput,
};
use types::{ShardId, Tick};

/// 路由配置(README 决策 1)。
#[derive(Debug, Clone, Copy)]
pub struct RouterConfig {
    /// 每分片本地收件箱容量(默认 256,README 决策 1)。
    pub inbox_capacity: usize,
    /// 跨片信封队列容量(默认 `DEFAULT_QUEUE_CAPACITY` = 1024)。
    pub queue_capacity: usize,
}

impl Default for RouterConfig {
    fn default() -> Self {
        Self {
            inbox_capacity: 256,
            queue_capacity: DEFAULT_QUEUE_CAPACITY,
        }
    }
}

/// 路由失败(可预期,类型化;反压语义:调用方断开该权威输入)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum RouteError {
    #[error("分片收件箱已满(容量 {capacity})")]
    InboxFull { capacity: usize },
}

/// 路由快照(REQ-005 任务书 02,签名固定):待投递信封 + per-source sequence。
/// **inboxes 不入快照**——本地命令都在命令日志里,回放按 dest 重投;
/// `sequences` 原样恢复(不恢复会改变后续迁移的 source_sequence,破坏
/// canonical 序,AC-C5 位级一致性的第一坑)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouterSnapshot {
    pub envelopes: Vec<Envelope>,
    pub sequences: BTreeMap<ShardId, u64>,
}

/// 命令路由器:有界收件箱 + 信封盖章队列(任务书 04)。
pub struct Router {
    queue: CrossShardQueue,
    inboxes: BTreeMap<ShardId, VecDeque<StampedInput>>,
    /// 每个 source 分片的信封序号计数器(单调 +1,从 0 起)。
    sequences: BTreeMap<ShardId, u64>,
    cfg: RouterConfig,
}

impl Router {
    pub fn new(cfg: RouterConfig) -> Self {
        Self {
            queue: CrossShardQueue::new(cfg.queue_capacity),
            inboxes: BTreeMap::new(),
            sequences: BTreeMap::new(),
            cfg,
        }
    }

    /// 本地权威命令入箱(有界;满即 Err,不驱逐已有条目)。
    pub fn route_local(&mut self, shard: ShardId, input: StampedInput) -> Result<(), RouteError> {
        let inbox = self.inboxes.entry(shard).or_default();
        if inbox.len() >= self.cfg.inbox_capacity {
            return Err(RouteError::InboxFull {
                capacity: self.cfg.inbox_capacity,
            });
        }
        inbox.push_back(input);
        Ok(())
    }

    /// tick 末:把源分片 Outbox 里的 Intent 逐个盖章入队。
    /// 序号按 source 严格单调;`AdmitError` 透传(已入队的不回滚,调用方处理)。
    pub fn collect_intents(
        &mut self,
        source: ShardId,
        intents: &[Intent],
        produced_at: Tick,
    ) -> Result<(), AdmitError> {
        for &intent in intents {
            let seq = self.sequences.entry(source).or_insert(0);
            let envelope = Envelope::stamp(source, intent, produced_at, *seq);
            *seq += 1;
            self.queue.admit(envelope)?;
        }
        Ok(())
    }

    /// tick 开:取 destination 在 apply_at 拍的信封批 + 收件箱中 apply_at 相符的命令,
    /// 合并为该拍输入前缀(canonical 序)。恰好一次:取走即移除。
    /// 未来拍的命令保留在收件箱,直到其 apply_at 到来。
    pub fn dispatch_prefix(&mut self, destination: ShardId, apply_at: Tick) -> Vec<StampedInput> {
        // 收件箱:只取 apply_at == 目标拍者,其余原序保留(VecDeque 保序 split)。
        let mut due: Vec<StampedInput> = Vec::new();
        if let Some(inbox) = self.inboxes.get_mut(&destination) {
            let mut kept = VecDeque::with_capacity(inbox.len());
            for input in inbox.drain(..) {
                if input.apply_at == apply_at {
                    due.push(input);
                } else {
                    kept.push_back(input);
                }
            }
            *inbox = kept;
        }
        // 信封批:take_batch 已给桶内 canonical 序;转 StampedInput(README 决策 3:
        // source 统一 SCHEDULER,sequence 用 envelope.source_sequence)。
        let envelopes = self
            .queue
            .take_batch(destination, apply_at)
            .into_iter()
            .map(|e| StampedInput {
                apply_at: e.apply_at,
                source: InputSource::SCHEDULER,
                sequence: e.source_sequence,
                payload: InputPayload::Envelope(e),
            });
        // 该拍桶已取走(take_batch 恰好一次);待投递查询直查 queue 桶键
        // (评审 Blocker 2:不再维护冗余表,无陈旧条目)。
        let mut prefix = due;
        prefix.extend(envelopes);
        // 稳定排序:命令间/信封间相对序不变,信封排在命令后(类型序号决定)。
        prefix.sort_by_key(|input| input.canonical_key());
        prefix
    }

    /// 待投递信封的目的分片集合(REQ-005 D5:驱动 ensure 用;两拍之间)。
    /// 直查队列桶键——dispatch 取走的桶即从集合消失,不陈旧。
    pub fn pending_destinations(&self) -> BTreeSet<ShardId> {
        self.queue.destinations()
    }

    /// `destination` 待投递信封的最早 apply_at(无 → None;迟到校验用)。
    pub fn pending_earliest(&self, destination: ShardId) -> Option<Tick> {
        self.queue.earliest_apply_at(destination)
    }

    pub fn pending_inbox(&self, shard: ShardId) -> usize {
        self.inboxes.get(&shard).map_or(0, VecDeque::len)
    }

    /// 收件箱容量(REQ-005 任务 01:批量投递预检用)。
    pub fn capacity(&self) -> usize {
        self.cfg.inbox_capacity
    }

    pub fn pending_envelopes(&self) -> usize {
        self.queue.len()
    }

    /// 快照(REQ-005 D7):待投递信封全量(canonical 全序,见
    /// `CrossShardQueue::snapshot`)+ per-source sequence 原样。
    /// 两拍之间调用(dispatch 取走的桶已清,快照即此刻真相)。
    pub fn snapshot(&self) -> RouterSnapshot {
        RouterSnapshot {
            envelopes: self.queue.snapshot(),
            sequences: self.sequences.clone(),
        }
    }

    /// 从快照恢复(REQ-005 D7):信封经 `CrossShardQueue::restore` 的不变量
    /// 校验(`apply_at == produced_at + 1`、容量;违反 = 内部状态损坏 →
    /// panic fail-stop)后放回桶;sequences 原样覆盖(不 merge——恢复线
    /// sequence 表只来自快照)。queue 容量由本 Router 的当前 cfg 承载。
    pub fn restore(&mut self, snap: RouterSnapshot) {
        self.queue.restore(snap.envelopes);
        self.sequences = snap.sequences;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use messaging::SimCommand;
    use types::{
        BlockId, BlockPos, DimensionId, EntityDesc, EntityId, EntityState, ShardPos, WorldId,
    };

    fn shard(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    /// 盖章命令:source 固定 1,sequence 可控。
    fn cmd(sequence: u64, apply_at: Tick) -> StampedInput {
        StampedInput {
            apply_at,
            source: InputSource(1),
            sequence,
            payload: InputPayload::Command(SimCommand::SetBlock {
                pos: BlockPos::ZERO,
                block: BlockId(1),
            }),
        }
    }

    fn migrate(entity: u32, destination: ShardId) -> Intent {
        Intent {
            destination,
            payload: messaging::EnvelopePayload::MigrateEntity {
                entity: EntityId::from_parts(1, entity),
                state: EntityState::default(),
                desc: EntityDesc::default(),
            },
        }
    }

    fn default_router() -> Router {
        Router::new(RouterConfig::default())
    }

    #[test]
    fn inbox_full_rejects_without_evicting() {
        let a = shard(0, 0);
        let mut router = Router::new(RouterConfig {
            inbox_capacity: 2,
            ..RouterConfig::default()
        });
        assert!(router.route_local(a, cmd(7, Tick(10))).is_ok());
        assert!(router.route_local(a, cmd(3, Tick(10))).is_ok());
        assert_eq!(
            router.route_local(a, cmd(9, Tick(10))),
            Err(RouteError::InboxFull { capacity: 2 }),
            "第三条必须被拒"
        );
        // 拒绝不驱逐:前两条随后照常分发(canonical 序按 sequence 升序)。
        let prefix = router.dispatch_prefix(a, Tick(10));
        assert_eq!(
            prefix.iter().map(|i| i.sequence).collect::<Vec<_>>(),
            vec![3, 7]
        );
        assert_eq!(router.pending_inbox(a), 0);
    }

    #[test]
    fn future_command_held_until_due() {
        let a = shard(0, 0);
        let mut router = default_router();
        assert!(router.route_local(a, cmd(1, Tick(105))).is_ok());
        assert!(
            router.dispatch_prefix(a, Tick(104)).is_empty(),
            "未到拍不分发"
        );
        assert_eq!(router.pending_inbox(a), 1, "未来命令保留在收件箱");
        let prefix = router.dispatch_prefix(a, Tick(105));
        assert_eq!(prefix.len(), 1);
        assert_eq!(prefix[0].sequence, 1);
        assert_eq!(router.pending_inbox(a), 0);
    }

    #[test]
    fn intents_stamped_monotonic_per_source() {
        let (a, b, c) = (shard(0, 0), shard(1, 0), shard(2, 0));
        let mut router = default_router();
        // source A 第一批(拍 10):序号 0、1,apply_at = 11。
        router
            .collect_intents(a, &[migrate(1, b), migrate(2, b)], Tick(10))
            .unwrap();
        // source A 第二批(拍 11):序号继续 2..4,apply_at = 12。
        router
            .collect_intents(a, &[migrate(3, b), migrate(4, b), migrate(5, b)], Tick(11))
            .unwrap();

        let first = router.dispatch_prefix(b, Tick(11));
        assert_eq!(
            first.iter().map(|i| i.sequence).collect::<Vec<_>>(),
            vec![0, 1],
            "第一批序号从 0 连续"
        );
        assert!(
            first.iter().all(|i| i.apply_at == Tick(11)),
            "apply_at = produced + 1"
        );
        let second = router.dispatch_prefix(b, Tick(12));
        assert_eq!(
            second.iter().map(|i| i.sequence).collect::<Vec<_>>(),
            vec![2, 3, 4],
            "第二批延续计数,仍连续"
        );

        // 不同 source 各自独立计数。
        router
            .collect_intents(c, &[migrate(6, b)], Tick(20))
            .unwrap();
        let from_c = router.dispatch_prefix(b, Tick(21));
        assert_eq!(from_c.len(), 1);
        assert_eq!(from_c[0].sequence, 0, "source C 从 0 起独立计数");
    }

    #[test]
    fn same_shard_intent_rejected() {
        let a = shard(0, 0);
        let mut router = default_router();
        assert_eq!(
            router.collect_intents(a, &[migrate(1, a)], Tick(10)),
            Err(AdmitError::SameShard),
            "同片 Intent 由队列拒绝,错误透传"
        );
        assert_eq!(router.pending_envelopes(), 0);
    }

    #[test]
    fn queue_full_propagates() {
        let (a, b) = (shard(0, 0), shard(1, 0));
        let mut router = Router::new(RouterConfig {
            queue_capacity: 1,
            ..RouterConfig::default()
        });
        assert_eq!(
            router.collect_intents(a, &[migrate(1, b), migrate(2, b)], Tick(10)),
            Err(AdmitError::QueueFull { capacity: 1 }),
            "第二枚撞满,错误透传"
        );
        assert_eq!(router.pending_envelopes(), 1, "已入队的不回滚");
    }

    #[test]
    fn dispatch_merges_commands_and_envelopes_canonically() {
        let (a, b) = (shard(0, 0), shard(1, 0));
        let mut router = default_router();
        // 收件箱:seq 5 与 seq 1 两条命令,apply_at 与信封同拍。
        router.route_local(b, cmd(5, Tick(11))).unwrap();
        router.route_local(b, cmd(1, Tick(11))).unwrap();
        // 信封:拍 10 产出,apply_at 11。
        router
            .collect_intents(a, &[migrate(9, b)], Tick(10))
            .unwrap();

        let prefix = router.dispatch_prefix(b, Tick(11));
        assert_eq!(prefix.len(), 3);
        // canonical 序:Command 类型序号小于 Envelope,命令间按 sequence。
        assert!(matches!(prefix[0].payload, InputPayload::Command(_)));
        assert_eq!(prefix[0].sequence, 1);
        assert!(matches!(prefix[1].payload, InputPayload::Command(_)));
        assert_eq!(prefix[1].sequence, 5);
        assert!(matches!(prefix[2].payload, InputPayload::Envelope(_)));
        for pair in prefix.windows(2) {
            assert!(pair[0].canonical_key() <= pair[1].canonical_key());
        }
    }

    #[test]
    fn dispatch_is_exactly_once() {
        let (a, b) = (shard(0, 0), shard(1, 0));
        let mut router = default_router();
        router.route_local(b, cmd(1, Tick(11))).unwrap();
        router
            .collect_intents(a, &[migrate(1, b)], Tick(10))
            .unwrap();
        assert_eq!(router.dispatch_prefix(b, Tick(11)).len(), 2);
        assert!(
            router.dispatch_prefix(b, Tick(11)).is_empty(),
            "同拍二次分发必为空"
        );
    }

    /// 评审 Blocker 2(单元级):A/B 两目的各一封,dispatch A 后集合不含 A、
    /// 仍含 B——待投递查询直查桶键,无冗余表陈旧条目。
    #[test]
    fn pending_destinations_drop_after_dispatch() {
        let mut r = Router::new(RouterConfig::default());
        let migrate = |dest| messaging::Intent {
            destination: dest,
            payload: messaging::EnvelopePayload::MigrateEntity {
                entity: types::EntityId::from_parts(1, 1),
                state: types::EntityState::default(),
                desc: types::EntityDesc::default(),
            },
        };
        r.collect_intents(
            shard(0, 0),
            &[migrate(shard(1, 0)), migrate(shard(2, 0))],
            Tick(0),
        )
        .expect("队列未满");
        assert_eq!(
            r.pending_destinations(),
            BTreeSet::from([shard(1, 0), shard(2, 0)])
        );
        assert_eq!(r.pending_earliest(shard(1, 0)), Some(Tick(1)));
        // dispatch A 的拍 1 桶 → A 从集合消失,B 仍在。
        let _ = r.dispatch_prefix(shard(1, 0), Tick(1));
        assert_eq!(r.pending_destinations(), BTreeSet::from([shard(2, 0)]));
        assert_eq!(r.pending_earliest(shard(1, 0)), None, "取尽即无");
        assert_eq!(r.pending_earliest(shard(2, 0)), Some(Tick(1)));
    }

    #[test]
    fn other_destinations_untouched() {
        let (a, b, other) = (shard(0, 0), shard(1, 0), shard(7, 7));
        let mut router = default_router();
        router
            .collect_intents(a, &[migrate(1, b), migrate(2, b)], Tick(10))
            .unwrap();
        assert_eq!(router.pending_envelopes(), 2);
        // 发往 B 的信封不影响其他分片(含其他拍)。
        assert!(router.dispatch_prefix(other, Tick(11)).is_empty());
        assert!(router.dispatch_prefix(b, Tick(12)).is_empty());
        assert_eq!(router.pending_envelopes(), 2, "未取的桶不动");
        let batch = router.dispatch_prefix(b, Tick(11));
        assert_eq!(batch.len(), 2);
        assert_eq!(router.pending_envelopes(), 0);
    }
}
