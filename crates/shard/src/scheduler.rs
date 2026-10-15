//! 固定 worker 池:capacity-one 通道、每拍全员屏障、分片所有权转移(蓝图 §3)。
//!
//! 全项目**唯一允许创建线程的地方**(蓝图 §2)。核心机制(AC5 的结构保证):
//! `ShardSim` 实例本身在 worker 间**按拍转移所有权**——调度器把 sim 连同本拍输入
//! 前缀一起发进 worker 的 capacity-one 通道,worker 处理完把 sim 连同 outbox 发回;
//! 调度器收齐全部结果才推进下一拍(全屏障)。任一时刻一个分片只在一个线程手里。
//!
//! 确定性要点(蓝图 §4):worker 完成顺序不影响结果(屏障后回传进 `BTreeMap`
//! 按 ShardId 升序归一);worker 内不读墙钟/不随机;分片分配 = ShardId 升序上的
//! 静态轮转(照 v2 scheduler.rs `WorkerSlot(ordinal % worker_slots)` 语义);
//! 内部通道 send/recv 失败 = 一致性违规 → panic(fail-stop)。

use std::collections::{BTreeMap, BTreeSet};
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::thread::JoinHandle;

use messaging::{AdmitError, StampedInput};
use physics_sim::{Outbox, ShardSim};
use types::{EntityId, EntityState, ShardId, Tick};

use crate::coordinator::TickCoordinator;
use crate::lifecycle::{LifecycleConfig, ShardLifecycle};
use crate::ownership::OwnershipMap;
use crate::router::{RouteError, Router, RouterConfig};

/// 一个 worker 本拍的批次任务:同 worker 分到的多个分片打包成一个消息
/// (元素 = (分片, sim 所有权, 本拍输入前缀),ShardId 升序)。
type WorkerJob = Vec<(ShardId, ShardSim, Vec<StampedInput>)>;

/// worker 回传:sim 连同本拍产出(回传完成顺序不定,屏障后进 BTreeMap 归一)。
type WorkerResult = Vec<(ShardId, ShardSim, Outbox)>;

/// worker 线程主体:循环收批;逐 job `apply_prefix → step → take_outbox` 后回传。
///
/// `recv` 返回 Err(发送端全部释放)= 正常退出;回传 send 失败 = 调度器已死,
/// 一致性违规 → panic(fail-stop)。
fn worker_main(job_rx: Receiver<WorkerJob>, result_tx: SyncSender<WorkerResult>) {
    while let Ok(job) = job_rx.recv() {
        let result: WorkerResult = job
            .into_iter()
            .map(|(shard, mut sim, prefix)| {
                sim.apply_prefix(&prefix);
                sim.step();
                let outbox = sim.take_outbox();
                (shard, sim, outbox)
            })
            .collect();
        if result_tx.send(result).is_err() {
            panic!(
                "worker 回传通道已关闭:一致性违规(fail-stop),\
                 调度器不应在屏障前释放结果接收端"
            );
        }
    }
}

/// 调度器配置(任务书 05,签名固定)。
#[derive(Debug, Clone)]
pub struct SchedulerConfig {
    /// worker 数,≥ 1。测试建议 2。
    pub worker_count: usize,
    pub router: RouterConfig,
    /// 传给各分片 lifecycle(本轮只接线,不自动 despawn;README 决策 4)。
    pub despawn_after_ticks: u64,
    /// 起始拍(README 决策 6):回放从快照拍对齐续跑;默认 `Tick::ZERO`,
    /// tick 语义不变(第 k 次 `tick()` 执行第 k-1 拍)。
    pub start_tick: Tick,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            worker_count: 4,
            router: RouterConfig::default(),
            despawn_after_ticks: 1800,
            start_tick: Tick::ZERO,
        }
    }
}

/// 一拍的执行摘要。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TickSummary {
    /// 本拍执行的拍号(tick() 第 k 次调用返回 k-1,README 决策 7)。
    pub tick: Tick,
    /// 本拍 step 的分片数。
    pub stepped_shards: usize,
    /// 本拍成功入队的迁移信封数。
    pub migrations_admitted: usize,
}

/// 固定 worker 池调度器(OS 线程 + capacity-one 通道 + 每拍全员汇合屏障)。
pub struct Scheduler {
    cfg: SchedulerConfig,
    coordinator: TickCoordinator,
    router: Router,
    ownership: OwnershipMap,
    /// 分片 → 生命周期记录(与 sims 同键)。
    lifecycles: BTreeMap<ShardId, ShardLifecycle>,
    /// 分片 → sim。稳态归调度器持有;tick 期间整体移出、经通道转移给 worker,
    /// 屏障后按 ShardId 升序收回——任一时刻一个分片只在一个线程手里(AC5)。
    sims: BTreeMap<ShardId, ShardSim>,
    job_txs: Vec<SyncSender<WorkerJob>>,
    result_rxs: Vec<Receiver<WorkerResult>>,
    workers: Vec<Option<JoinHandle<()>>>,
    /// 静态轮转游标:每分配一个分片后 +1 取模 worker 数。
    next_slot: usize,
}

impl Scheduler {
    /// 起 worker 线程池。worker_count == 0 → panic(配置错误)。
    pub fn new(cfg: SchedulerConfig) -> Self {
        if cfg.worker_count == 0 {
            panic!("SchedulerConfig.worker_count 必须 ≥ 1(配置错误)");
        }
        let mut job_txs = Vec::with_capacity(cfg.worker_count);
        let mut result_rxs = Vec::with_capacity(cfg.worker_count);
        let mut workers = Vec::with_capacity(cfg.worker_count);
        for i in 0..cfg.worker_count {
            // capacity-one rendezvous:更大队列只会纵容意外的 tick 重叠(照 v2)。
            let (job_tx, job_rx) = sync_channel::<WorkerJob>(1);
            let (result_tx, result_rx) = sync_channel::<WorkerResult>(1);
            let handle = std::thread::Builder::new()
                .name(format!("shard-worker-{i}"))
                .spawn(move || worker_main(job_rx, result_tx))
                .expect("worker 线程创建失败(系统资源耗尽)");
            job_txs.push(job_tx);
            result_rxs.push(result_rx);
            workers.push(Some(handle));
        }
        Self {
            router: Router::new(cfg.router),
            coordinator: TickCoordinator::new(cfg.start_tick),
            ownership: OwnershipMap::new(),
            lifecycles: BTreeMap::new(),
            sims: BTreeMap::new(),
            job_txs,
            result_rxs,
            workers,
            next_slot: 0,
            cfg,
        }
    }

    /// 注册一个分片(以构造好的 ShardSim 形式进入)。已注册 → panic;
    /// 同时 ownership.claim + lifecycle.activate(懒创建语义由注册方掌握)。
    pub fn register(&mut self, sim: ShardSim) {
        let shard = sim.shard();
        if self.sims.contains_key(&shard) {
            panic!("重复注册分片 {shard:?}(所有权唯一,AC5 fail-stop)");
        }
        let at = self.coordinator.tick();
        self.ownership.claim(shard, at);
        self.lifecycles.insert(
            shard,
            ShardLifecycle::activate(
                at,
                LifecycleConfig {
                    despawn_after_ticks: self.cfg.despawn_after_ticks,
                },
            ),
        );
        self.sims.insert(shard, sim);
    }

    /// 本地权威命令入箱(经 Router)。未注册分片 → panic(README 决策 2);
    /// apply_at < 当前 tick → panic(迟到输入,README 决策 6);
    /// 收件箱满 → Err(InboxFull)(反压,调用方断开权威输入)。
    pub fn submit(&mut self, shard: ShardId, input: StampedInput) -> Result<(), RouteError> {
        if !self.sims.contains_key(&shard) {
            panic!("提交给未注册分片 {shard:?}(调用方 bug,fail-stop)");
        }
        let now = self.coordinator.tick();
        if input.apply_at < now {
            panic!(
                "迟到输入:apply_at = {} < 当前拍 {}(该拍已分发,调用方 bug,fail-stop)",
                input.apply_at.0, now.0
            );
        }
        self.router.route_local(shard, input)
    }

    /// 执行一拍(全屏障,流程严格按任务书 05 的 1→8):
    /// 分发前缀 → 静态轮转派批 → capacity-one 发送 → 屏障回收(BTreeMap 归一)
    /// → 迁移盖章 → lifecycle 接线 → advance。
    pub fn tick(&mut self) -> TickSummary {
        // 1. 本拍号。
        let at = self.coordinator.tick();

        // 2+3. 按 ShardId 升序取本拍前缀(BTreeMap into_iter 即升序),
        //      静态轮转分配 WorkerSlot:每分配一个分片后游标 +1 取模。
        //      把整表移出:sim 所有权离开调度器,屏障后才回来(AC5)。
        let sims = std::mem::take(&mut self.sims);
        let stepped_shards = sims.len();
        let mut slots: Vec<WorkerJob> = (0..self.cfg.worker_count).map(|_| Vec::new()).collect();
        let mut dispatched: BTreeSet<usize> = BTreeSet::new();
        // 空片判定要留证:前缀非空的分片集合(步骤 7 用,屏障后 prefix 已消耗)。
        let mut had_prefix: BTreeSet<ShardId> = BTreeSet::new();
        for (shard, sim) in sims {
            let prefix = self.router.dispatch_prefix(shard, at);
            if !prefix.is_empty() {
                had_prefix.insert(shard);
            }
            let slot = self.next_slot;
            self.next_slot = (self.next_slot + 1) % self.cfg.worker_count;
            if slots[slot].is_empty() {
                dispatched.insert(slot);
            }
            slots[slot].push((shard, sim, prefix));
        }

        // 4. 经每 worker 的 capacity-one 通道发送(每 worker 本拍至多一条任务,
        //    结果通道上一拍已排空 → 发送不会与回收互锁)。失败 = worker 已死。
        for &slot in &dispatched {
            let job = std::mem::take(&mut slots[slot]);
            if self.job_txs[slot].send(job).is_err() {
                panic!("worker {slot} 任务通道已关闭:一致性违规(fail-stop)");
            }
        }

        // 5. 屏障:收齐全部已派发 worker 的结果;回传完成顺序不定,
        //    BTreeMap<ShardId, _> 归一(确定性)。
        let mut results: BTreeMap<ShardId, (ShardSim, Outbox)> = BTreeMap::new();
        for &slot in &dispatched {
            let batch = self.result_rxs[slot]
                .recv()
                .expect("worker 结果通道断开:一致性违规(fail-stop)");
            for (shard, sim, outbox) in batch {
                results.insert(shard, (sim, outbox));
            }
        }

        // 6. sim 放回自有表;按 ShardId 升序处理 outbox(results 即 BTreeMap,
        //    迭代天然升序):迁移意图盖章入队。
        let mut migrations_admitted = 0usize;
        for (shard, (sim, outbox)) in results {
            match self.router.collect_intents(shard, &outbox.migrations, at) {
                Ok(()) => migrations_admitted += outbox.migrations.len(),
                // TODO(反压):队列满当前的策略是 fail-stop;后续轮次接入
                // "暂停源分片/降载"类反压,不外溢为可恢复错误。
                Err(AdmitError::QueueFull { capacity }) => panic!(
                    "跨片信封队列满(capacity {capacity})于拍 {}:fail-stop,\
                     反压策略待接入(任务书 05)",
                    at.0
                ),
                Err(e) => panic!("迁移盖章被拒(编程错误,fail-stop):{e}"),
            }
            self.sims.insert(shard, sim);
        }

        // 7. lifecycle 接线(README 决策 4:只记活动,不自动 despawn):
        //    本拍有输入前缀或非空实体 → 活动;空片不记,空闲计时才走得动。
        for (&shard, sim) in &self.sims {
            if had_prefix.contains(&shard) || sim.entity_count() > 0 {
                self.lifecycles
                    .get_mut(&shard)
                    .expect("已注册分片必有生命周期记录")
                    .note_activity(at);
            }
        }

        // 8. 推进恰好一拍并汇报(决策 7:第 k 次调用执行第 k-1 拍)。
        self.coordinator.advance();
        TickSummary {
            tick: at,
            stepped_shards,
            migrations_admitted,
        }
    }

    /// 查询:未注册或实体不在该分片 → None。
    pub fn entity_state(&self, shard: ShardId, entity: EntityId) -> Option<EntityState> {
        self.sims.get(&shard)?.entity_state(entity)
    }

    /// 分片快照(任务书 01):委托 `ShardSim::snapshot` 产出确定性字节。
    ///
    /// **只在两拍之间调用**(sims 全部归位、屏障已收齐):tick 进行中不存在
    /// 该窗口——本调度器是单线程驱动,`tick()` 返回后调用方才能再进,
    /// 借此满足"调用时 sim 必在手"的约定,无运行期检查。
    /// 未注册分片 → None。
    pub fn snapshot_shard(&self, shard: ShardId) -> Option<Vec<u8>> {
        self.sims.get(&shard).map(ShardSim::snapshot)
    }

    /// 已注册分片,ShardId 升序(BTreeMap 键序)。
    pub fn shard_ids(&self) -> Vec<ShardId> {
        self.sims.keys().copied().collect()
    }

    pub fn shard_count(&self) -> usize {
        self.sims.len()
    }

    /// 下一将被执行的拍号。
    pub fn tick_now(&self) -> Tick {
        self.coordinator.tick()
    }

    /// 关停:生命周期 Active→Draining→Stopped(排空语义:一拍屏障后即空),
    /// 释放全部 sim 与 claim,释放任务发送端让 worker 正常退出,join 全部 worker。
    /// 幂等(句柄与发送端均 take/clear)。
    pub fn shutdown(mut self) {
        for lifecycle in self.lifecycles.values_mut() {
            lifecycle.begin_draining();
            lifecycle.note_drained();
            lifecycle.stop();
        }
        self.sims.clear();
        let owned: Vec<ShardId> = self.ownership.iter().map(|(id, _)| id).collect();
        for shard in owned {
            self.ownership.release(shard);
        }
        // 释放任务发送端 → worker 的 recv 返回 Err → 正常退出。
        self.job_txs.clear();
        self.join_workers();
    }

    /// join 全部 worker;`Option::take` 保证幂等(shutdown 与 Drop 共用)。
    /// worker 在 panic 中退出 = 一致性违规 → panic(fail-stop)。
    fn join_workers(&mut self) {
        for slot in &mut self.workers {
            if let Some(handle) = slot.take()
                && handle.join().is_err()
            {
                panic!("worker 线程 panic 中退出:一致性违规(fail-stop)");
            }
        }
    }
}

impl Drop for Scheduler {
    /// Drop 同样 join(`Option::take` 保证幂等),防线程泄漏。此处 join 失败
    /// 不 panic(Drop 路径 panic 会 abort;fail-stop 的显式路径由 shutdown 承担)。
    fn drop(&mut self) {
        self.job_txs.clear();
        for slot in &mut self.workers {
            if let Some(handle) = slot.take() {
                let _ = handle.join();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use physics_sim::{ShardSimConfig, SolidifyConfig};
    use types::{BlockPos, DimensionId, ShardPos, WorldId};
    use voxel::SimpleRegistry;
    use voxel::VoxelWorld;

    fn shard_id(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    /// 虚空分片(凝固桥关闭:调度器测试不关心桥行为)。
    fn sim(x: i32, z: i32) -> ShardSim {
        let mut cfg = ShardSimConfig::new(shard_id(x, z));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
    }

    fn sched_cfg(worker_count: usize) -> SchedulerConfig {
        SchedulerConfig {
            worker_count,
            ..SchedulerConfig::default()
        }
    }

    #[test]
    #[should_panic(expected = "worker_count")]
    fn config_zero_workers_panics() {
        let _ = Scheduler::new(sched_cfg(0));
    }

    #[test]
    fn tick_summary_reports_stepped_shards() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let summary = sched.tick();
        assert_eq!(
            summary,
            TickSummary {
                tick: Tick::ZERO,
                stepped_shards: 2,
                migrations_admitted: 0,
            }
        );
        sched.shutdown();
    }

    /// 测试清单(shard):start_tick = Tick(100) → 首次 tick() 的 summary.tick
    /// == Tick(100),tick_now 从 100 走到 101(README 决策 6)。
    #[test]
    fn start_tick_advances_from_given_tick() {
        let cfg = SchedulerConfig {
            start_tick: Tick(100),
            ..sched_cfg(2)
        };
        let mut sched = Scheduler::new(cfg);
        assert_eq!(sched.tick_now(), Tick(100), "起动即对齐到起始拍");
        sched.register(sim(0, 0));
        let summary = sched.tick();
        assert_eq!(
            summary.tick,
            Tick(100),
            "第 1 次 tick() 执行第 start_tick 拍"
        );
        assert_eq!(sched.tick_now(), Tick(101), "拍号恰 +1");
        sched.shutdown();
    }

    /// 测试清单(shard):注册 sim → snapshot_shard → restore → 再注册可跑;
    /// 未注册 → None。
    #[test]
    fn snapshot_shard_roundtrip_via_restore() {
        let mut sched = Scheduler::new(sched_cfg(2));
        assert_eq!(
            sched.snapshot_shard(shard_id(9, 9)),
            None,
            "未注册分片 → None"
        );

        let shard = shard_id(0, 0);
        let mut cfg = ShardSimConfig::new(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        let mut sim0 = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
        // 放一块地形,让快照字节非平凡(体素路径)。
        let ground = BlockPos::new(1, 0, 1);
        let mut voxels = VoxelWorld::new();
        voxels.set_block(ground, types::BlockId(1)).expect("界内");
        for cp in voxels.chunk_positions().collect::<Vec<_>>() {
            let chunk = voxels.remove_chunk(cp).expect("刚列出");
            sim0.load_chunk(cp, chunk);
        }
        // 再注入一个实体(实体路径)。
        sim0.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 0,
            payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                entity: types::EntityId::from_parts(1, 1),
                state: types::EntityState {
                    translation: types::glam::Vec3::new(1.5, 3.0, 1.5),
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);
        sched.register(sim0);

        let bytes = sched.snapshot_shard(shard).expect("已注册分片必有快照");
        assert!(!bytes.is_empty());

        let registry = Box::new(SimpleRegistry::builtin());
        let restored = ShardSim::restore(&bytes, registry).expect("自有快照必可解码");
        assert_eq!(restored.shard(), shard, "恢复后分片身份不变");

        // 恢复实例可注册、可跑(冒烟)。
        let mut sched2 = Scheduler::new(sched_cfg(2));
        sched2.register(restored);
        sched2.tick();
        sched2.shutdown();
        sched.shutdown();
    }
}
