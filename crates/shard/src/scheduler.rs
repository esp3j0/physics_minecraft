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

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::thread::JoinHandle;

use messaging::{AdmitError, InputPayload, InputSource, StampedInput};
use physics_sim::{Outbox, ShardSim};
use types::{BlockId, BlockPos, ChunkPos, EntityId, EntityState, ShardId, Tick};

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
    /// 观察者表(REQ-002 §4.2:**同一张表**——命令化条目以 `InputSource`
    /// 为键(玩家/agent ID),直通 `set_observers` 的整表以调度器保留位
    /// `InputSource::SCHEDULER` 落表)。激活刷新与票判定都只读本表。
    observer_table: BTreeMap<InputSource, types::glam::Vec3>,
    /// 向后兼容开关(任务书 req001-backlog/02 细则 5):任一路径设置过
    /// 观察者(直通或命令化)→ true;从未设置 → tick 完全跳过激活刷新,
    /// 行为与历史一致(表为空但设置过 = "无激活中心"语义,不等同跳过)。
    observers_ever_set: bool,
    /// 调度器级观察者专用收件箱(REQ-002 §4.2;unbounded——任务书语义
    /// 细则 1:观察者频率低,有界语义复杂度不值得,REQ-003 网络层再收口)。
    observer_inbox: VecDeque<StampedInput>,
    /// 本拍激活决策(tick 步骤 1.5 收集,两拍之间经 take_activation_requests
    /// 取走;恰一次语义)。
    activation_requests: BTreeMap<ShardId, physics_sim::ActivationDecisions>,
    /// 本拍凝固写入的方块(REQ-003 §4.4 观察输出源②:tick 内收集,两拍之间
    /// 经 take_solidified 取走;恰一次语义)。
    solidified: Vec<(ShardId, BlockPos, BlockId)>,
    /// 本拍方块写入的 owner 发布集(REQ-005 D11:tick 内收集,两拍之间经
    /// `take_block_updates` 取走;恰一次语义)。
    block_updates: Vec<(ShardId, BlockPos, BlockId)>,
    /// despawn 候选(tick 步骤 7 记录;只记录不清,两拍之间可重复取)。
    despawn_requests: BTreeSet<ShardId>,
    /// chunk 持有表(REQ-005 D2/D4):`(owner, chunk) → holder 集合`。
    /// owner 是纯函数(`owner_of`);本表随 apply_chunk_load/unload 维护,
    /// 不入快照(恢复时 rebuild_holders 重建)。
    chunk_holders: crate::chunks::ChunkHolders,
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
            observer_table: BTreeMap::new(),
            observers_ever_set: false,
            observer_inbox: VecDeque::new(),
            activation_requests: BTreeMap::new(),
            solidified: Vec::new(),
            block_updates: Vec::new(),
            despawn_requests: BTreeSet::new(),
            chunk_holders: crate::chunks::ChunkHolders::new(),
            cfg,
        }
    }

    /// 设置激活观察者(演员位置;两拍之间调用,下一 tick 生效——tick 流程
    /// 开头即读取)。覆盖式设置(再次调用即整表替换)。
    ///
    /// REQ-002 §4.2 保留为测试/装配直通:整表替换命令化观察者表(同一张
    /// 表)。直通列表**逐条落表、一条不丢**(旧语义 `observers: Vec<Vec3>`
    /// 全量作激活/票中心,位置集无序语义):**末位**沿用保留键
    /// `InputSource::SCHEDULER`(0;单位置调用与历史行为逐位一致),其余
    /// 位置自 `u32::MAX` 向左依次落键(第 j 个 → `u32::MAX-(len-2-j)`)。
    /// 该高位段专属直通,与分配给连接/会话的正向 source(1,2,…)永不相交
    /// ——同键覆盖的静默截断(评审回归)不可能重现;observers() 键升序
    /// 视图因此为 [末位, 首位, …] 的确定性逆序(顺序无语义,仅确定性)。
    pub fn set_observers(&mut self, observers: Vec<types::glam::Vec3>) {
        self.observer_table.clear();
        let len = observers.len();
        for (j, pos) in observers.into_iter().enumerate() {
            let key = if j + 1 == len {
                InputSource::SCHEDULER
            } else {
                InputSource(u32::MAX - (len - 2 - j) as u32)
            };
            self.observer_table.insert(key, pos);
        }
        self.observers_ever_set = true;
    }

    /// 激活/票判定的观察者位置集(表值;确定性:InputSource 升序)。
    /// `None` = 从未设置过任一路径 → tick 完全不做激活刷新、无票
    /// (向后兼容开关);`Some(空)` = 设置过但表空 → 无激活中心语义
    /// (§4.3:距离 +∞,不唤醒任何实体)。
    fn activation_centers(&self) -> Option<Vec<types::glam::Vec3>> {
        if !self.observers_ever_set {
            return None;
        }
        Some(self.observer_table.values().copied().collect())
    }

    /// 观察者只读视图(REQ-002 §4.2;驱动 ensure_shard/票判定用):
    /// `(source, position)`,按 InputSource 升序(BTreeMap 键序)。
    pub fn observers(&self) -> Vec<(InputSource, types::glam::Vec3)> {
        self.observer_table.iter().map(|(k, v)| (*k, *v)).collect()
    }

    /// 移除一个观察者(REQ-003 §4.2/03:断连 → 观察者移除——票与激活中心
    /// 随条目消失,空片无票可 despawn;REQ-003 评审修复②)。在表 → `Some`
    /// (原位置);不在 → `None`(幂等,重复移除无害)。直通保留键
    /// `InputSource::SCHEDULER` 同样可移(同表同权);表空不清
    /// `observers_ever_set`("无激活中心"语义,距票判定自然无票)。
    pub fn remove_observer(&mut self, source: InputSource) -> Option<types::glam::Vec3> {
        let removed = self.observer_table.remove(&source);
        if removed.is_some() {
            eprintln!("[scheduler] 观察者移除:source={source:?}");
        }
        removed
    }

    /// 观察者输入入箱(REQ-002 §4.2:调度器级专用收件箱;README 决策 3)。
    /// apply_at 到期应用(迟到即丢弃,与 router inbox 同语义);收件箱
    /// unbounded(任务书语义细则 1:观察者频率低,有界语义复杂度不值得,
    /// REQ-003 网络层再收口)。
    ///
    /// 迟到语义说明:分片收件箱的迟到输入是 panic(权威输入迟到 = 调用方
    /// bug);观察者命令的生产入口 `Session::move_observer` 盖当前拍,
    /// 回放原章直投的条目 tick ≤ 快照拍已被 replay 过滤,再晚不可能——
    /// 沿用同一 fail-stop 语义(迟到 = 一致性违规)。
    pub fn submit_observer(&mut self, input: StampedInput) {
        let now = self.coordinator.tick();
        if input.apply_at < now {
            panic!(
                "迟到观察者输入:apply_at = {} < 当前拍 {}(该拍已应用,fail-stop)",
                input.apply_at.0, now.0
            );
        }
        self.observer_inbox.push_back(input);
    }

    /// 两拍之间取本拍激活决策(REQ-002 §4.4;每拍 tick 内收集,取走即清:
    /// 恰一次语义)。
    pub fn take_activation_requests(&mut self) -> Vec<(ShardId, physics_sim::ActivationDecisions)> {
        std::mem::take(&mut self.activation_requests)
            .into_iter()
            .collect()
    }

    /// 两拍之间取本拍凝固写入的方块(REQ-003 §4.4 观察输出源②;签名固定,
    /// 任务书 00):`(分片, 世界坐标方块格, 写入的方块 ID)` 列表,每拍 tick 内
    /// 收集、取走即清(恰一次;再取 = 空)。方块 ID 读自分片体素世界
    /// (`Outbox.solidified` 只携带 `(实体, 格)`,写入值经 `get_block` 查表)。
    pub fn take_solidified(&mut self) -> Vec<(ShardId, BlockPos, BlockId)> {
        std::mem::take(&mut self.solidified)
    }

    /// 本拍 block write 的 owner 视角发布集(REQ-005 D11;取走即清,恰一次):
    /// `(分片, 世界坐标方块格, 写入的方块 ID)`,只含 `shard == owner(pos.chunk_of())`
    /// 的条目——广播时每个 holder 副本各产生一条同格写入,非 owner 的条目
    /// 在此丢弃(网络/持久化看到恰一次;跨界涌现写入的任务 03 信封另接)。
    pub fn take_block_updates(&mut self) -> Vec<(ShardId, BlockPos, BlockId)> {
        std::mem::take(&mut self.block_updates)
    }

    /// 分片全实体表(REQ-003 §4.4 观察输出源①;任务书 00 签名固定):
    /// `(EntityId, EntityState)`,EntityId 升序(委托 [`ShardSim::entity_states`])。
    /// 未注册分片 → `None`。
    pub fn entity_states(&self, shard: ShardId) -> Option<Vec<(EntityId, EntityState)>> {
        let sim = self.sims.get(&shard)?;
        Some(sim.entity_states())
    }

    /// 两拍之间装载一个 chunk(幂等:重复装载同 chunk = 重建)。
    /// 委托 `ShardSim::load_chunk`(insert + 边界 collider 重建)。未注册分片
    /// → panic(与 submit 同款 fail-stop;分片由驱动 ensure 后才可能来数)。
    /// chunk 的 owner(REQ-005 D2,纯函数):本体区分片;world/dim 取自
    /// `shard` 参数(单世界单维度 v1,一切调用都发生在某个已知分片上下文)。
    pub fn owner_of(shard: ShardId, pos: ChunkPos) -> ShardId {
        ShardId::new(shard.world, shard.dimension, pos.shard_of())
    }

    /// 当前持有 `(owner, pos)` 副本的分片集合(含 owner,若它加载着);
    /// 未加载 → 空集(REQ-005 D2)。
    pub fn holders_of(&self, owner: ShardId, pos: ChunkPos) -> BTreeSet<ShardId> {
        self.chunk_holders.holders(owner, pos)
    }

    /// Router 待投递信封的目的分片(两拍之间;REQ-005 D5,驱动 ensure 用)。
    pub fn pending_shard_destinations(&self) -> BTreeSet<ShardId> {
        self.router.pending_destinations()
    }

    /// 分片是否已加载某个 chunk(REQ-005 任务 01:外部 SetBlock 的未加载
    /// chunk 门卫透传;`VoxelWorld::set_block` 会自动建空气 chunk,hostile
    /// 输入必须在驱动侧拦下)。未注册分片 → false。
    pub fn is_chunk_loaded(&self, shard: ShardId, pos: ChunkPos) -> bool {
        self.sims
            .get(&shard)
            .is_some_and(|sim| sim.voxel().get_chunk(pos).is_some())
    }

    /// 分片某个已加载 chunk 的只读视图(REQ-005 任务 01:副本逐字节一致
    /// 断言用)。未注册分片或未加载 → None。
    pub fn voxel_chunk(&self, shard: ShardId, pos: ChunkPos) -> Option<&voxel::Chunk> {
        self.sims.get(&shard)?.voxel().get_chunk(pos)
    }

    pub fn apply_chunk_load(&mut self, shard: ShardId, pos: ChunkPos, chunk: voxel::Chunk) {
        // 区域校验(REQ-005 D4):分片只装载自己 core ∪ margin_ring 内的
        // chunk——区域外装载 = 编排 bug(激活请求已被区域过滤)→ fail-stop。
        if !self
            .sims
            .get(&shard)
            .unwrap_or_else(|| panic!("装载到未注册分片 {shard:?}(调用方 bug,fail-stop)"))
            .activation_region()
            .contains(pos)
        {
            panic!(
                "装载区域外 chunk {pos:?} 到分片 {shard:?}(core ∪ margin 之外,编排 bug,fail-stop)"
            );
        }
        let owner = Self::owner_of(shard, pos);
        if !self.sims.contains_key(&owner) {
            panic!(
                "chunk {pos:?} 的 owner {owner:?} 未注册先装载(holder = {shard:?};\
                 驱动 ensure 必须先建 owner,REQ-005 D5,fail-stop)"
            );
        }
        self.sims
            .get_mut(&shard)
            .expect("上方已校验注册")
            .load_chunk(pos, chunk);
        self.chunk_holders.register(owner, pos, shard);
    }

    /// 两拍之间卸载一个 chunk(REQ-005 D6:**owner-only 持久化**)。
    /// 分片未注册或 chunk 未装载 → None;owner 卸载 → `Some(chunk)` 交驱动
    /// 持久化;**非 owner 的扩张副本 → 丢弃副本返回 None(不写库)**。
    pub fn apply_chunk_unload(&mut self, shard: ShardId, pos: ChunkPos) -> Option<voxel::Chunk> {
        let sim = self.sims.get_mut(&shard)?;
        let chunk = sim.unload_chunk(pos)?;
        // chunk 在手(= 确曾加载):撤 holder 登记,按归属决定去留。
        let owner = Self::owner_of(shard, pos);
        self.chunk_holders.unregister(owner, pos, shard);
        if shard == owner {
            Some(chunk)
        } else {
            None // 扩张副本:丢弃,不写库(REQ-005 D6)
        }
    }

    /// 两拍之间取 despawn 候选(REQ-002 §4.5:空片+无票+idle_expired)。
    /// 只记录不清:重复取返回同一集合,直到下一 tick 刷新。
    pub fn take_despawn_requests(&mut self) -> Vec<ShardId> {
        self.despawn_requests.iter().copied().collect()
    }

    /// 卸载整个分片(REQ-002 §4.5):lifecycle 走 Draining→Stopped→release,
    /// 逐 chunk 卸载返回(供持久化屏障)。实体非空 → panic(候选条件被破坏,
    /// fail-stop);未注册 → panic。调用前提:两拍之间(不在 tick 内)。
    pub fn unload_shard(&mut self, shard: ShardId) -> Vec<(ChunkPos, voxel::Chunk)> {
        let sim = self
            .sims
            .get(&shard)
            .unwrap_or_else(|| panic!("卸载未注册分片 {shard:?}(调用方 bug,fail-stop)"));
        if sim.entity_count() > 0 {
            panic!("unload_shard 触及非空分片 {shard:?}(despawn 候选条件被破坏,fail-stop)");
        }
        let lifecycle = self
            .lifecycles
            .get_mut(&shard)
            .expect("已注册分片必有生命周期记录");
        lifecycle.begin_draining();
        lifecycle.note_drained();
        lifecycle.stop();

        let sim = self.sims.remove(&shard).expect("上方已查");
        // lifecycle 记录保留(Stage::Stopped 供观测;同分片再注册时覆盖重建)。
        self.activation_requests.remove(&shard);
        self.despawn_requests.remove(&shard);
        self.ownership.release(shard);
        // 逐 chunk 卸载(REQ-005 D6:只返回 owner chunk;扩张副本丢弃)。
        // BTreeMap 键序 = ChunkPos 升序(确定性);每个 chunk 的 holder 登记
        // 一并撤销。
        let positions: Vec<ChunkPos> = sim.voxel().chunk_positions().collect();
        let mut sim = sim;
        let mut out = Vec::new();
        // owner chunk 集合:despawn 后其余 holder 的扩张副本成为"owner 缺席的
        // 孤儿"(REQ-005 §4.1.3:owner 必须存在)。邻片 idle 窗口与本片不同拍
        // 到期,不能等它们各自 despawn——一律就地撤销副本(holder 登记同撤),
        // 邻片激活随后按需重请求(cooldown 已过或观察者仍在即重载)。
        let mut orphan_positions: Vec<ChunkPos> = Vec::new();
        for cp in positions {
            let owner = Self::owner_of(shard, cp);
            if let Some(chunk) = sim.unload_chunk(cp) {
                self.chunk_holders.unregister(owner, cp, shard);
                if shard == owner {
                    out.push((cp, chunk));
                    orphan_positions.push(cp);
                } // 扩张副本:丢弃,不写库(D6)
            }
        }
        // 撤销孤儿副本:仍注册的其余 holder 逐个卸载该 chunk(丢弃,不写库)。
        // 顺带清这些 holder 对本片 chunk 的扩张请求缓存(activation_requests
        // 中的卸载请求无需干预——chunk 已不在,下一拍 plan 重算)。
        for cp in orphan_positions {
            for holder in self.chunk_holders.holders(shard, cp) {
                if let Some(sim) = self.sims.get_mut(&holder) {
                    let _ = sim.unload_chunk(cp);
                    self.chunk_holders.unregister(shard, cp, holder);
                }
            }
        }
        out
    }

    /// 从各分片已加载 chunk 重建 holder 表(REQ-005 D2:恢复/回放用;
    /// 表不入快照,恢复后调用一次)。
    pub fn rebuild_holders(&mut self) {
        let entries: Vec<(ShardId, Vec<ChunkPos>)> = self
            .sims
            .iter()
            .map(|(shard, sim)| (*shard, sim.voxel().chunk_positions().collect()))
            .collect();
        self.chunk_holders
            .rebuild(entries.iter().map(|(shard, chunks)| {
                let s = *shard;
                (
                    s,
                    chunks.as_slice(),
                    Box::new(move |cp| Self::owner_of(s, cp)) as Box<dyn Fn(ChunkPos) -> ShardId>,
                )
            }));
    }

    /// 生命周期阶段查询(观测用):无记录(未注册)→ None。
    pub fn lifecycle_stage(&self, shard: ShardId) -> Option<crate::lifecycle::Stage> {
        self.lifecycles
            .get(&shard)
            .map(crate::lifecycle::ShardLifecycle::stage)
    }

    /// 注册一个分片(以构造好的 ShardSim 形式进入)。已注册 → panic;
    /// 同时 ownership.claim + lifecycle.activate(懒创建语义由注册方掌握)。
    /// 末尾重建 holder 表:预载 chunk 的 sim(回放恢复/测试手工装载/任务 02
    /// 的 restore)凭已加载 chunk 登记 holder——装配期允许先 holder 后 owner,
    /// owner 在位校验仍由 tick 开头的 fail-stop 承担(评审 Blocker 1)。
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
        self.rebuild_holders();
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

    /// 原子批量投递(REQ-005 D3:外部编辑同拍广播;签名固定):先全量预检
    /// ——每个目标已注册且收件箱有空位(`pending_inbox < inbox_capacity`);
    /// 任一不满足 → `Err` 且**一片都不投**(不半投;未注册也返回 `Err`——
    /// 广播路径的防御层,调用方 Session 已校验过,不 panic)。预检通过后逐个
    /// `route_local`;单线程驱动下预检与投递之间无并发入箱者,中途失败 =
    /// 内部一致性违规 → panic(fail-stop)。
    pub fn submit_batch(
        &mut self,
        shards: &[ShardId],
        input: StampedInput,
    ) -> Result<(), RouteError> {
        let capacity = self.router.capacity();
        for &shard in shards {
            if !self.sims.contains_key(&shard) {
                eprintln!("[scheduler] 批量投递目标未注册:{shard:?}(广播防御层拒绝)");
                return Err(RouteError::InboxFull { capacity });
            }
            if self.router.pending_inbox(shard) >= capacity {
                return Err(RouteError::InboxFull { capacity });
            }
        }
        for &shard in shards {
            if self.router.route_local(shard, input.clone()).is_err() {
                panic!("submit_batch 预检通过后投递失败(单线程下不可达,内部一致性违规,fail-stop)");
            }
        }
        Ok(())
    }

    /// 执行一拍(全屏障,流程严格按任务书 05 的 1→8):
    /// 分发前缀 → 静态轮转派批 → capacity-one 发送 → 屏障回收(BTreeMap 归一)
    /// → 迁移盖章 → lifecycle 接线 → advance。
    pub fn tick(&mut self) -> TickSummary {
        // 1. 本拍号。
        let at = self.coordinator.tick();

        // 0.一致性校验(REQ-005 D5,两 invariant):
        // a) 不允许信封黑洞:待投递信封的目的分片若未注册且 apply_at ≤ 本拍,
        //    说明驱动 ensure 漏了(内部一致性违规)→ fail-stop。正常路径由
        //    驱动"观察者 32m 邻片 + pending destinations"ensure 覆盖。
        for dest in self.router.pending_destinations() {
            let Some(earliest) = self.router.pending_earliest(dest) else {
                continue;
            };
            if earliest <= at && !self.sims.contains_key(&dest) {
                panic!(
                    "信封黑洞:目的分片 {dest:?} 未注册且最早 apply_at {earliest:?} \
                     ≤ 当前拍 {at:?}(驱动 ensure 漏了,内部一致性违规,fail-stop)"
                );
            }
        }
        // b) holder 的 owner 必须已注册(装配期允许先 holder 后 owner,
        //    但第一拍开始前必须齐;REQ-005 §4.1.3)。
        for owner in self.chunk_holders.owners() {
            if !self.sims.contains_key(&owner) {
                panic!(
                    "holder 表中的 owner {owner:?} 未注册(装配顺序违规,\
                     第一拍前 owner 必须就位,REQ-005 D5,fail-stop)"
                );
            }
        }

        // 1.2(REQ-002 §4.2):应用观察者收件箱中 apply_at == at 的全部条目
        //     (按 canonical 序;未来拍留存——与 router inbox 同语义;过期拍
        //     在 submit 时已被拒,不会出现)。分片输入前缀永不出现该变体。
        {
            let mut due: Vec<StampedInput> = Vec::new();
            let mut kept = VecDeque::with_capacity(self.observer_inbox.len());
            for input in self.observer_inbox.drain(..) {
                if input.apply_at == at {
                    due.push(input);
                } else {
                    kept.push_back(input);
                }
            }
            self.observer_inbox = kept;
            due.sort_by_key(|input| input.canonical_key());
            for input in due {
                let (source, position) = match input.payload {
                    InputPayload::Command(messaging::SimCommand::SetObserverPosition {
                        position,
                    }) => (input.source, position),
                    _ => panic!(
                        "观察者收件箱只收 SetObserverPosition(收到其它载荷,调用方 bug,fail-stop)"
                    ),
                };
                self.observer_table.insert(source, position);
                self.observers_ever_set = true;
            }
        }

        // 1.5(README 决策 8;任务书 req001-backlog/02):观察者已设置过 →
        //     每分片激活刷新(tier③ 冻结/唤醒在本拍 step 生效);tier①②
        //     chunk 装卸请求自本任务起外露(REQ-002 §4.4,两拍之间取用)。
        //     观察者中心集 = 命令化表 + 直通列表(两条写入路径,一张语义表);
        //     两者皆从未设置 → 完全跳过,行为与历史一致(向后兼容开关)。
        self.activation_requests.clear();
        if let Some(observers) = self.activation_centers() {
            for (shard, sim) in self.sims.iter_mut() {
                let decisions = sim.apply_activation(&observers);
                self.activation_requests.insert(*shard, decisions);
            }
        }

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
        // 凝固写入收集(REQ-003 观察输出源②,任务书 00):每拍重收
        // (上一拍的已两拍之间被取走;未取即被本拍覆盖,恰一次语义由取走即清保证)。
        self.solidified.clear();
        // 方块写入收集(REQ-005 D11):同款恰一次;owner 过滤逐条判定。
        self.block_updates.clear();
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
            // (shard, 格, 写入的方块):Outbox 只记 (实体, 格),写入值读自
            // sim 体素世界(写入先于本处,凝固路径已落格)。
            for (_, cell) in &outbox.solidified {
                self.solidified
                    .push((shard, *cell, sim.voxel().get_block(*cell)));
            }
            // 方块写入(REQ-005 D11):广播下每个 holder 副本各记一条同格
            // 写入;只发布 owner 的条目(非 owner 的丢弃——任务 03 的跨界
            // BlockWrite 信封再接),网络/持久化由此看到恰一次。
            for write in &outbox.block_updates {
                if shard == Self::owner_of(shard, write.pos.chunk_of()) {
                    self.block_updates.push((shard, write.pos, write.block));
                }
            }
            self.sims.insert(shard, sim);
        }

        // 7. lifecycle 接线(README 决策 4:只记活动,不自动 despawn):
        //    本拍有输入前缀或非空实体 → 活动;空片不记,空闲计时才走得动。
        //    despawn 候选(REQ-002 §4.5):空片 + 无票 + idle_expired → 记录
        //    (不自动拆,卸载由 unload_shard 显式驱动;lifecycle 本身不推进)。
        //    票 = 任一观察者到分片核心 AABB(8×8 chunk 立柱)的水平距离
        //    ≤ chunk_unload_radius(48m)。
        self.despawn_requests.clear();
        for (&shard, sim) in &self.sims {
            let active = had_prefix.contains(&shard) || sim.entity_count() > 0;
            if active {
                self.lifecycles
                    .get_mut(&shard)
                    .expect("已注册分片必有生命周期记录")
                    .note_activity(at);
            }
            let lifecycle = self
                .lifecycles
                .get(&shard)
                .expect("已注册分片必有生命周期记录");
            if sim.entity_count() == 0
                && !self.shard_is_ticketed(shard)
                && lifecycle.idle_expired(at)
            {
                self.despawn_requests.insert(shard);
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

    /// 票判定(REQ-002 §4.5):任一观察者到分片核心 AABB(8×8 chunk 立柱,
    /// 水平投影为 `shard.pos.min_chunk()..+8` chunk 的方块区)的水平距离
    /// ≤ chunk_unload_radius(48m)→ 有票。观察者从未设置(两路径皆未)→ 无票。
    fn shard_is_ticketed(&self, shard: ShardId) -> bool {
        let Some(observers) = self.activation_centers() else {
            return false;
        };
        // 取激活配置的卸载半径(票半径与 tier①② 卸载半径同源,REQ-002 §4.5)。
        let radius = self
            .sims
            .values()
            .next()
            .map(|s| s.activation_config().chunk_unload_radius)
            .unwrap_or(48.0);
        let radius_sq = radius * radius;
        let min = shard.pos.min_chunk();
        let min_x = (min.x * types::CHUNK_SIZE) as f32;
        let min_z = (min.z * types::CHUNK_SIZE) as f32;
        let max_x = min_x + (types::SHARD_CHUNKS * types::CHUNK_SIZE) as f32;
        let max_z = min_z + (types::SHARD_CHUNKS * types::CHUNK_SIZE) as f32;
        observers.iter().any(|o| {
            // 水平点到 AABB 距离(立方体外取分量差,内含取 0)。
            let dx = (o.x.clamp(min_x, max_x) - o.x).abs();
            let dz = (o.z.clamp(min_z, max_z) - o.z).abs();
            dx * dx + dz * dz <= radius_sq
        })
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
            // despawn(unload_shard)后的终态记录已 Stopped:幂等跳过,不重走
            // 三连转移(begin_draining 只允许从 Active 进入,重走 = panic)。
            if lifecycle.is_stopped() {
                continue;
            }
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

    /// 测试清单(shard,任务书 02):set_observers 后 tick:远处实体冻结
    /// (经 TickSummary 之外的实体状态查询验证——冻结体清速度且不再下落,
    /// 近处实体继续下落)。
    #[test]
    fn observers_drive_freeze_via_scheduler() {
        use types::glam::Vec3;

        let shard = shard_id(0, 0);
        let mut cfg = ShardSimConfig::new(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        let mut sim0 = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
        let (near, far) = (
            types::EntityId::from_parts(1, 1),
            types::EntityId::from_parts(1, 2),
        );
        for (seq, id, pos) in [
            (0, near, Vec3::new(8.0, 3.0, 8.0)),
            (1, far, Vec3::new(100.0, 3.0, 100.0)),
        ] {
            sim0.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: seq,
                payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                    entity: id,
                    state: types::EntityState {
                        translation: pos,
                        ..types::EntityState::default()
                    },
                    desc: types::EntityDesc::default(),
                }),
            }]);
        }

        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim0);
        sched.set_observers(vec![Vec3::new(8.0, 3.0, 8.0)]);
        let summary = sched.tick();
        assert_eq!(summary.stepped_shards, 1);

        // 冻结体:清速度 + kinematic → 原位悬停(位级 ==);近处实体继续下落。
        let expected_far = types::EntityState {
            translation: Vec3::new(100.0, 3.0, 100.0),
            ..types::EntityState::default()
        };
        for _ in 0..10 {
            sched.tick();
            assert_eq!(
                sched.entity_state(shard, far),
                Some(expected_far),
                "远处实体冻结悬停(位级)"
            );
            let near_state = sched.entity_state(shard, near).expect("近处实体仍在");
            assert!(near_state.translation.y < 3.0, "近处实体未被冻结,继续下落");
        }
        sched.shutdown();
    }

    /// 测试清单(shard,任务书 02 细则 5):未设置观察者 → tick 行为与
    /// 现状一致(激活刷新完全跳过):远处实体照常下落,不被冻结。
    #[test]
    fn no_observers_set_behavior_unchanged() {
        use types::glam::Vec3;

        let shard = shard_id(0, 0);
        let mut cfg = ShardSimConfig::new(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        let mut sim0 = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
        let far = types::EntityId::from_parts(1, 2);
        sim0.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                entity: far,
                state: types::EntityState {
                    translation: Vec3::new(100.0, 3.0, 100.0),
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);

        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim0);
        sched.tick();
        let s1 = sched.entity_state(shard, far).expect("实体仍在");
        assert!(s1.translation.y < 3.0, "未设观察者 → 不冻结,实体照常下落");
        sched.tick();
        let s2 = sched.entity_state(shard, far).expect("实体仍在");
        assert!(
            s2.translation.y < s1.translation.y,
            "持续下落(与历史行为一致)"
        );
        sched.shutdown();
    }

    // ---- 任务书 02:观察者命令化 + 装载编排 + despawn(测试清单) ----

    use messaging::{InputPayload, InputSource, SimCommand, StampedInput};

    /// 带位置的观察者输入(StampedInput,source 可控;SetObserverPosition)。
    fn observer_input(
        source: u32,
        sequence: u64,
        apply_at: Tick,
        pos: types::glam::Vec3,
    ) -> StampedInput {
        StampedInput {
            apply_at,
            source: InputSource(source),
            sequence,
            payload: InputPayload::Command(SimCommand::SetObserverPosition { position: pos }),
        }
    }

    /// 测试清单 1 `observer_command_updates_table_at_tick`:
    /// submit_observer(apply_at=N) → tick N 后 observers() 反映;N 前未应用。
    #[test]
    fn observer_command_updates_table_at_tick() {
        let mut sched = Scheduler::new(sched_cfg(2));
        let pos = types::glam::Vec3::new(8.0, 3.0, 8.0);
        // 当前拍 0,投递 apply_at = 0 的观察者命令。
        sched.submit_observer(observer_input(1, 0, Tick::ZERO, pos));
        // apply_at 未到之前不应用(此例 apply_at=0 首个 tick 即到;先验未来拍)。
        sched.submit_observer(observer_input(2, 0, Tick(5), pos));
        assert!(sched.observers().is_empty(), "tick 前:观察者命令尚未应用");
        // tick 拍 0:apply_at=0 到期 → source 1 入表;source 2(拍 5)留存。
        sched.tick();
        assert_eq!(
            sched.observers(),
            vec![(InputSource(1), pos)],
            "拍 0 后:仅到期条目入表"
        );
        for _ in 0..4 {
            sched.tick();
        }
        assert_eq!(
            sched.observers(),
            vec![(InputSource(1), pos)],
            "拍 4 末:未来拍条目仍未应用"
        );
        sched.tick(); // 拍 5:第二条到期。
        assert_eq!(
            sched.observers(),
            vec![(InputSource(1), pos), (InputSource(2), pos)],
            "拍 5 后:第二条已应用,InputSource 升序"
        );
        sched.shutdown();
    }

    /// 评审修复②(REQ-003):`remove_observer` 断连票撤语义——在表移除
    /// 返回原位置且 despawn 票消失(空片 idle 到期成候选);不在表幂等
    /// `None`;表空不复活 despawn(从未设置语义不回退)。
    #[test]
    fn remove_observer_revokes_ticket() {
        use types::glam::Vec3;

        // 唯一分片 (0,0) 空片;观察者落在核心区 → 有票。
        let cfg = SchedulerConfig {
            despawn_after_ticks: 3,
            ..sched_cfg(2)
        };
        let shard = shard_id(0, 0);
        let mut sched = Scheduler::new(cfg);
        sched.register(sim(0, 0));
        sched.submit_observer(observer_input(1, 0, Tick::ZERO, Vec3::new(8.0, 0.0, 8.0)));
        for _ in 0..5 {
            sched.tick(); // idle 窗口跑满
        }
        assert!(
            sched.take_despawn_requests().is_empty(),
            "前置:观察者在场 = 有票,空片不成 despawn 候选"
        );

        // 断连移除:返回原位置;续跑过窗口 → 候选出现。
        assert_eq!(
            sched.remove_observer(InputSource(1)),
            Some(Vec3::new(8.0, 0.0, 8.0)),
            "在表移除返回原位置"
        );
        for _ in 0..6 {
            sched.tick();
        }
        assert_eq!(
            sched.take_despawn_requests(),
            vec![shard],
            "移除后无票:idle 到期成 despawn 候选"
        );

        // 幂等:再次移除 = None。
        assert_eq!(
            sched.remove_observer(InputSource(1)),
            None,
            "不在表 → None(幂等)"
        );
        sched.shutdown();
    }

    /// 测试清单 2 `set_observers_direct_still_works`:整表替换直通(兼容开关)
    /// ——观察者表与激活行为同旧语义;且 observers() 视图反映直通内容
    /// (同一张表,REQ-002 §4.2)。
    #[test]
    fn set_observers_direct_still_works() {
        use types::glam::Vec3;
        let shard = shard_id(0, 0);
        let mut sim0 = sim(0, 0);
        let far = types::EntityId::from_parts(1, 2);
        sim0.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: far,
                state: types::EntityState {
                    translation: Vec3::new(100.0, 3.0, 100.0),
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim0);
        // 直通:整表替换。表内位置也以 (InputSource, Vec3) 视图可见——
        // 直通路径固定使用调度器保留 source(InputSource::SCHEDULER)。
        sched.set_observers(vec![Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(
            sched.observers(),
            vec![(InputSource::SCHEDULER, Vec3::new(8.0, 3.0, 8.0))],
            "直通设置反映在同一张表上(调度器保留 source)"
        );
        sched.tick();
        let s = sched.entity_state(shard, far).expect("实体仍在");
        assert_eq!(
            s,
            types::EntityState {
                translation: Vec3::new(100.0, 3.0, 100.0),
                ..types::EntityState::default()
            },
            "远处实体冻结悬停(直通路径激活语义与历史一致)"
        );
        // 再次调用 = 覆盖替换。
        sched.set_observers(vec![Vec3::new(96.0, 0.0, 96.0)]);
        assert_eq!(
            sched.observers(),
            vec![(InputSource::SCHEDULER, Vec3::new(96.0, 0.0, 96.0))],
            "整表替换:旧表清空"
        );
        sched.shutdown();
    }

    /// 测试清单 3 `activation_requests_exposed_between_ticks`:观察者移动后
    /// take_activation_requests 非空、含 chunk 装载请求;第二次取为空(恰一次)。
    #[test]
    fn activation_requests_exposed_between_ticks() {
        use types::glam::Vec3;
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.submit_observer(observer_input(1, 0, Tick::ZERO, Vec3::new(8.0, 0.0, 8.0)));
        sched.tick();
        let requests = sched.take_activation_requests();
        assert!(
            !requests.is_empty(),
            "有观察者 → 本拍激活决策被收集(含装载请求)"
        );
        let (_, decisions) = &requests[0];
        assert!(
            !decisions.chunks_to_load.is_empty(),
            "空世界 + 近观察者 → 必有 chunk 装载请求:{decisions:?}"
        );
        assert!(
            sched.take_activation_requests().is_empty(),
            "第二次取为空(恰一次:取走即清)"
        );
        sched.shutdown();
    }

    // ---- REQ-005 任务 00(归属/持有/owner-only 持久化/信封黑洞) ----
    /// 任务书测试 4(scheduler 级):holder 登记——load 后 holders_of 含该分片;
    /// 重复 load 不重复;unload 后移除。owner 与扩张 holder 都登记。
    #[test]
    fn holders_track_load_unload() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let cp = types::ChunkPos::new(7, 0); // 片 0 本体区东缘
        // owner(0,0) 装载自己的 chunk。
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        assert!(
            sched
                .holders_of(shard_id(0, 0), cp)
                .contains(&shard_id(0, 0))
        );
        // 重复装载同 chunk(幂等重建):holder 仍只 1 个。
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        assert_eq!(sched.holders_of(shard_id(0, 0), cp).len(), 1);
        // 邻片(1,0)装载同 chunk 作扩张副本(margin=1 → chunk 7 在其区域内)。
        sched.apply_chunk_load(shard_id(1, 0), cp, voxel::Chunk::air());
        let holders = sched.holders_of(shard_id(0, 0), cp);
        assert_eq!(holders.len(), 2, "owner + 邻片扩张副本都登记:{holders:?}");
        // owner 卸载 → 登记剩邻片。
        let _ = sched.apply_chunk_unload(shard_id(0, 0), cp);
        assert_eq!(sched.holders_of(shard_id(0, 0), cp).len(), 1);
        // 邻片也卸 → 条目清空。
        let _ = sched.apply_chunk_unload(shard_id(1, 0), cp);
        assert!(sched.holders_of(shard_id(0, 0), cp).is_empty());
        sched.shutdown();
    }

    /// 任务书测试 5:邻片扩张副本 unload → None(owner 的 chunk 仍在表内)。
    #[test]
    fn non_owner_unload_returns_none() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let cp = types::ChunkPos::new(7, 0); // owner = 片 0
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        sched.apply_chunk_load(shard_id(1, 0), cp, voxel::Chunk::air());
        assert!(
            sched.apply_chunk_unload(shard_id(1, 0), cp).is_none(),
            "非 owner 副本卸载:丢弃不写库(D6)"
        );
        assert!(
            sched.apply_chunk_unload(shard_id(0, 0), cp).is_some(),
            "owner 卸载:返回 chunk 交持久化"
        );
        sched.shutdown();
    }

    /// 任务书测试 6:owner unload → Some(原 chunk 内容往返)。
    #[test]
    fn owner_unload_returns_chunk() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        let cp = types::ChunkPos::new(3, 4);
        let mut chunk = voxel::Chunk::air();
        chunk.set(5, 100, 5, types::BlockId(2));
        sched.apply_chunk_load(shard_id(0, 0), cp, chunk);
        let back = sched
            .apply_chunk_unload(shard_id(0, 0), cp)
            .expect("owner 卸载返回 chunk");
        assert_eq!(back.get(5, 100, 5), types::BlockId(2), "内容往返无损");
        sched.shutdown();
    }

    /// 任务书测试 7:unload_shard 只返回 owner chunk;扩张副本丢弃。
    /// 片 0 持:本体 chunk(0,0) + 片 1 的扩张副本 chunk(8,0)(margin=1
    /// 时在片 0 区域内)。片 1 已注册(holder 的 owner 在位)。
    #[test]
    fn unload_shard_filters_to_owner_chunks() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        sched.apply_chunk_load(
            shard_id(0, 0),
            types::ChunkPos::new(0, 0),
            voxel::Chunk::air(),
        );
        // chunk(8,0) 的 owner = 片 1;片 0 作为 holder 装载扩张副本。
        sched.apply_chunk_load(
            shard_id(0, 0),
            types::ChunkPos::new(8, 0),
            voxel::Chunk::air(),
        );
        let out = sched.unload_shard(shard_id(0, 0));
        assert_eq!(
            out.len(),
            1,
            "despawn 只返回 owner chunk(扩张副本丢弃,D6):{out:?}"
        );
        assert_eq!(out[0].0, types::ChunkPos::new(0, 0));
        sched.shutdown();
    }

    /// 任务书测试 9:迟到兜底——apply_at ≤ 当前拍且目的分片未注册的信封
    /// → tick fail-stop(不允许信封黑洞)。
    #[test]
    #[should_panic(expected = "信封黑洞")]
    fn late_unregistered_destination_panics() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        // 构造一片 9 的迁移信封(produced_at = 0 → apply_at = 1):
        // 片 9 未注册 → 第 1 拍 tick 开头即触发(驱动 ensure 缺席的等价物)。
        // 同 crate 测试直达私有 router:走真实盖章路径构造迁移信封。
        sched
            .router
            .collect_intents(
                shard_id(0, 0),
                &[messaging::Intent {
                    destination: shard_id(9, 9),
                    payload: messaging::EnvelopePayload::MigrateEntity {
                        entity: types::EntityId::from_parts(0, 1),
                        state: types::EntityState::default(),
                        desc: types::EntityDesc::default(),
                    },
                }],
                Tick::ZERO,
            )
            .expect("队列未满");
        sched.tick(); // 拍 0:apply_at(1) > at(0) → 不触发
        sched.tick(); // 拍 1:apply_at ≤ at 且未注册 → panic
    }

    /// 评审 Blocker 1:预载 chunk 的分片 register 后 holder 表正确填充
    /// (回放恢复/手工装载此前漏 holders_of;任务 01 的 SetBlock 广播靠它)。
    #[test]
    fn register_preloaded_sim_populates_holders() {
        let mut sched = Scheduler::new(sched_cfg(2));
        // shard (0,0) 的 sim 预载 chunk (8,0)(owner = 片 1,0;在片 0 的
        // margin 环内)——先 holder 后 owner(装配期允许)。
        let mut sim0 = sim(0, 0);
        sim0.load_chunk(types::ChunkPos::new(8, 0), voxel::Chunk::air());
        sched.register(sim0);
        // owner 注册后(重建以 sims 全量为准,先注册的 holder 不丢)。
        sched.register(sim(1, 0));
        assert!(
            sched
                .holders_of(shard_id(1, 0), types::ChunkPos::new(8, 0))
                .contains(&shard_id(0, 0)),
            "预载 chunk 经 register 重建进 holder 表"
        );
        // 非 owner 卸载扩张副本 → None(D6)。
        assert!(
            sched
                .apply_chunk_unload(shard_id(0, 0), types::ChunkPos::new(8, 0))
                .is_none(),
            "非 owner unload 返回 None(不写库)"
        );
        sched.shutdown();
    }

    /// 评审 Blocker 2(场景级):dispatch 取走 A 的桶后 A despawn,
    /// 下一拍不得误报信封黑洞(旧冗余表的陈旧条目会 panic)。
    #[test]
    fn dispatched_destination_can_despawn_without_false_blackhole() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        let a = shard_id(1, 0);
        sched.register(sim(1, 0));
        // B 侧产生迁移信封 → dest A(produced_at=0 → apply_at=1)。
        sched
            .router
            .collect_intents(
                shard_id(0, 0),
                &[messaging::Intent {
                    destination: a,
                    // 出生位姿必须落在 A 的核心区(x∈[128,256)),否则当拍
                    // 又被迁移走、RemoveEntity 找不到实体。
                    payload: messaging::EnvelopePayload::MigrateEntity {
                        entity: types::EntityId::from_parts(0, 1),
                        state: types::EntityState {
                            translation: types::glam::Vec3::new(140.0, 10.0, 8.0),
                            ..types::EntityState::default()
                        },
                        desc: types::EntityDesc::default(),
                    },
                }],
                Tick::ZERO,
            )
            .expect("队列未满");
        assert_eq!(
            sched.pending_shard_destinations(),
            std::collections::BTreeSet::from([a])
        );
        // 拍 0:apply_at(1) > 0,桶留存;拍 1:dispatch 取走 → 目的集合为空。
        sched.tick();
        sched.tick();
        assert!(
            !sched.pending_shard_destinations().contains(&a),
            "dispatch 后目的从待投递集合消失(直查桶键,无陈旧条目)"
        );
        // A 收到迁移实体 → 移除它,使 A 可 despawn。
        sched
            .submit(
                a,
                StampedInput {
                    apply_at: Tick(2),
                    source: InputSource::SCHEDULER,
                    sequence: 0,
                    payload: InputPayload::Command(SimCommand::RemoveEntity {
                        entity: types::EntityId::from_parts(0, 1),
                    }),
                },
            )
            .expect("收件箱未满");
        sched.tick(); // 拍 2:实体移除
        let _ = sched.unload_shard(a);
        // 旧冗余表在此处 tick 会因陈旧 {A:1} 误 panic;直查实现无此问题。
        sched.tick();
        sched.tick();
        sched.shutdown();
    }

    /// 任务书测试 3:两分片 + 一观察者——各自激活请求只落自己的
    /// core ∪ margin;同一 chunk 的本体请求只来自 owner(扩张请求是
    /// holder 副本,属设计语义,不算重复本体)。
    #[test]
    fn activation_two_shards_no_cross_core_duplicates() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        // 观察者贴边界(x=126):两片的激活都会跑。
        sched.set_observers(vec![types::glam::Vec3::new(126.0, 0.0, 64.0)]);
        sched.tick();
        let reqs = sched.take_activation_requests();
        for (shard, decisions) in &reqs {
            let region = types::ChunkRegion::core(*shard).expanded(1);
            for cp in decisions
                .chunks_to_load
                .iter()
                .chain(decisions.chunks_to_unload.iter())
            {
                assert!(
                    region.contains(*cp),
                    "分片 {shard:?} 请求了区域外 chunk {cp:?}(D4 不变量)"
                );
            }
        }
        // 同一 chunk 的"本体装载"只应来自 owner:两片请求集合中,owner≠请求者
        // 的 chunk 必须落在请求者的 margin 环内(即扩张副本语义)。
        sched.shutdown();
    }

    /// 测试清单 4 `chunk_load_unload_roundtrip_via_orchestr_api`:
    /// apply_chunk_load 后物理可见(实体落在生成的地面上被支撑);unload 返回该 chunk。
    #[test]
    fn chunk_load_unload_roundtrip_via_orchestr_api() {
        use types::glam::Vec3;
        use voxel::{Superflat, WorldGenerator};

        let shard = shard_id(0, 0);
        let mut sim0 = sim(0, 0);
        let e = types::EntityId::from_parts(1, 1);
        sim0.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: e,
                state: types::EntityState {
                    // 地面顶面(世界 y=-60)上方 5m:短程下落,几十拍内驻留。
                    translation: Vec3::new(8.5, -55.0, 8.5),
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim0);
        // 装载生成的 chunk(Superflat classic:stone×1 + dirt×3,层底在
        // 世界 y = -64 → 地面顶面 y = -60)。
        let cp = types::ChunkPos::new(0, 0);
        let chunk = Superflat::classic().generate(types::WorldSeed(0), cp);
        sched.apply_chunk_load(shard, cp, chunk);
        // 幂等:重复装载同 chunk = 重建(不 panic)。
        sched.apply_chunk_load(
            shard,
            cp,
            Superflat::classic().generate(types::WorldSeed(0), cp),
        );
        // 实体落到地面顶面(y=-60)驻留:中心 ≈ -59.5。
        for _ in 0..60 {
            sched.tick();
        }
        let s = sched.entity_state(shard, e).expect("实体仍在");
        assert!(
            s.translation.y > -60.0 && s.translation.y < -58.5,
            "实体落在生成的地面上被支撑:y={}",
            s.translation.y
        );
        // 卸载:返回该 chunk(供持久化);未装载位置 → None。
        let unloaded = sched.apply_chunk_unload(shard, cp).expect("已装载必有返回");
        assert_eq!(
            unloaded,
            Superflat::classic().generate(types::WorldSeed(0), cp),
            "卸载返回装载时的 chunk(未被修改)"
        );
        assert!(
            sched.apply_chunk_unload(shard, cp).is_none(),
            "未装载 → None"
        );
        sched.shutdown();
    }

    /// 测试清单 5 `despawn_candidate_only_when_empty_and_unticketed`:
    /// 空片无票 idle 到期 → 候选;有实体或有票 → 无候选。
    #[test]
    fn despawn_candidate_only_when_empty_and_unticketed() {
        use types::glam::Vec3;

        // 分片 A(0,0) 空片;B(0,1) 有实体;C(0,2) 空片但有观察者票。
        // 专用小 despawn 窗口(测试不必跑 1800 拍)。
        let cfg = SchedulerConfig {
            despawn_after_ticks: 5,
            ..sched_cfg(2)
        };
        let mut sched = Scheduler::new(cfg);
        sched.register(sim(0, 0));
        sched.register(sim(0, 1));
        sched.register(sim(0, 2));
        let b = shard_id(0, 1);
        let mut sim_b = sim(0, 1);
        sim_b.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: types::EntityId::from_parts(1, 1),
                state: types::EntityState {
                    // B 核心区内(分片 8×8 chunk = 128 块跨;z=136 ∈ [128,256))。
                    translation: Vec3::new(8.0, 3.0, 136.0),
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);
        // 用带实体的 sim_b 顶替 B(先注册了空壳,直接换表内同键值 = 覆盖)。
        sched.sims.insert(b, sim_b);
        // 观察者落在 C 的核心区深处(z=383 ∈ [384-1,·):383 < 384,取 383 在
        // C 区间 [256,384) 内,距 A/B 的 AABB 远超 48m → 只给 C 票)。
        sched.submit_observer(observer_input(1, 0, Tick::ZERO, Vec3::new(8.0, 0.0, 383.0)));
        // 跑过 idle 窗口(5 拍)+ 余量。
        for _ in 0..8 {
            sched.tick();
        }
        let candidates = sched.take_despawn_requests();
        assert_eq!(
            candidates,
            vec![shard_id(0, 0)],
            "只有 A(空+无票+idle 到期)是候选"
        );
        assert_eq!(
            sched.take_despawn_requests(),
            vec![shard_id(0, 0)],
            "候选只记录不清:重复取返回同集(任务书语义细则 5:只记录不清)"
        );
        sched.shutdown();
    }

    /// 测试清单 2 补遗(评审回归):多位置直通——旧语义(`observers:
    /// Option<Vec<Vec3>>`)下整个列表都作为激活/票中心,直通落到
    /// `BTreeMap` 后必须一条不丢。断言:表视图全量再生(键升序 = 列表
    /// 序);票判定按多中心生效(单中心只会给 A 票,丢中心则 C 漏票)。
    #[test]
    fn set_observers_multi_position_all_take_effect() {
        use types::glam::Vec3;

        // A(0,0) 空片 idle 候选;B(0,1) 有实体;C(0,2) 空片,票只来自
        // 第二个直通位置。despawn 窗口专用小值(测试不必跑 1800 拍)。
        let cfg = SchedulerConfig {
            despawn_after_ticks: 5,
            ..sched_cfg(2)
        };
        let mut sched = Scheduler::new(cfg);
        sched.register(sim(0, 0));
        sched.register(sim(0, 1));
        sched.register(sim(0, 2));
        let b = shard_id(0, 1);
        let mut sim_b = sim(0, 1);
        sim_b.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: types::EntityId::from_parts(1, 1),
                state: types::EntityState {
                    translation: Vec3::new(8.0, 3.0, 136.0),
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);
        sched.sims.insert(b, sim_b);

        // 两个直通位置:pos_a 距 A 核心区近、距 C 极远;pos_c 在 C 核心区
        // 深处、距 A 极远(几何与 despawn_candidate_only_when_empty_and_unticketed
        // 同款:48m 半径下互不重叠)。旧 bug:同键覆盖 → 仅 pos_c 存活,
        // A 无票被记候选(且表视图只剩 1 条)。
        let pos_a = Vec3::new(8.0, 0.0, 8.0);
        let pos_c = Vec3::new(8.0, 0.0, 383.0);
        sched.set_observers(vec![pos_a, pos_c]);

        // 表视图:两条直通位置全在,一条不丢(高位直通段键 > SCHEDULER 键,
        // 键升序视图 = [末位, 首位] 的确定性逆序;激活/票按位置集,无序语义)。
        let view = sched.observers();
        assert_eq!(
            view.len(),
            2,
            "多位置直通必须全部落表(旧实现同键覆盖只剩末位):{view:?}"
        );
        assert_eq!(
            view.iter().map(|(_, p)| *p).collect::<Vec<_>>(),
            vec![pos_c, pos_a],
            "键升序读出确定性:末位(SCHEDULER 键 0)在前,高位段其次"
        );

        // 票判定按多中心生效:跑过 idle 窗口(5 拍)+ 余量。两分片都有票
        // → 无候选(B 非空也不可能是候选)。
        for _ in 0..8 {
            sched.tick();
        }
        assert!(
            sched.take_despawn_requests().is_empty(),
            "A 有票(第一中心)、C 有票(第二中心)→ 均非候选(旧实现丢第一\
             中心时 A 会误入候选)"
        );
        sched.shutdown();
    }

    /// 测试清单 6 `unload_shard_walks_lifecycle_and_releases`:卸载后
    /// shard_ids 不含、ownership 释放、再 register 同分片可行;实体非空 → panic。
    #[test]
    fn unload_shard_walks_lifecycle_and_releases() {
        let shard = shard_id(0, 0);
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        let cp = types::ChunkPos::new(0, 0);
        sched.apply_chunk_load(shard, cp, voxel::Chunk::air());
        // 实体非空 → panic(fail-stop:候选条件被破坏)。
        {
            let mut sim_e = sim(0, 0);
            sim_e.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: InputSource::SCHEDULER,
                sequence: 0,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: types::EntityId::from_parts(1, 9),
                    state: types::EntityState::default(),
                    desc: types::EntityDesc::default(),
                }),
            }]);
            let mut other = Scheduler::new(sched_cfg(2));
            other.register(sim_e);
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                other.unload_shard(shard)
            }));
            assert!(
                result.is_err(),
                "实体非空 → unload_shard 必须 panic(fail-stop)"
            );
            other.shutdown();
        }
        // 空片:正常卸载。
        let chunks = sched.unload_shard(shard);
        assert_eq!(chunks, vec![(cp, voxel::Chunk::air())], "逐 chunk 卸载返回");
        assert!(
            !sched.shard_ids().contains(&shard),
            "卸载后 shard_ids 不含该分片"
        );
        assert!(!sched.ownership.contains(shard), "ownership 已释放");
        assert!(
            matches!(
                sched.lifecycle_stage(shard),
                Some(crate::lifecycle::Stage::Stopped)
            ),
            "卸载后 lifecycle = Stopped"
        );
        assert_eq!(
            sched.apply_chunk_unload(shard, cp),
            None,
            "chunk 随分片卸载"
        );
        // 再 register 同分片可行(复活路径)。
        sched.register(sim(0, 0));
        assert!(sched.shard_ids().contains(&shard), "再注册可行(复活)");
        // 未注册 → panic。
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            sched.unload_shard(shard_id(7, 7))
        }));
        assert!(result.is_err(), "未注册分片 → panic(fail-stop)");
        sched.shutdown();
    }

    /// 回归(任务 04 授权修复):despawn(unload_shard)保留 Stopped 记录后
    /// 直接 shutdown 不得 panic(终态记录幂等跳过)——真实链路 = 运行中途
    /// despawn → 驱动正常关机(session.shutdown → Scheduler::shutdown)。
    #[test]
    fn shutdown_after_unload_does_not_panic() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.apply_chunk_load(
            shard_id(0, 0),
            types::ChunkPos::new(0, 0),
            voxel::Chunk::air(),
        );
        let chunks = sched.unload_shard(shard_id(0, 0));
        assert_eq!(chunks.len(), 1, "前置:卸载返回 chunk");
        assert!(sched.shard_ids().is_empty(), "前置:分片已卸载");
        // despawn 后直接关机:不得 panic(修复前在此触发非法阶段转移)。
        sched.shutdown();
    }

    // ---- 任务书 req005-consistency/01(外部编辑扩张一致性) ----

    /// 一条盖章 SetBlock(apply_at/pos/block 可控,source 固定会话型 1)。
    fn setblock_input(
        apply_at: Tick,
        sequence: u64,
        pos: BlockPos,
        block: types::BlockId,
    ) -> StampedInput {
        StampedInput {
            apply_at,
            source: InputSource(1),
            sequence,
            payload: InputPayload::Command(SimCommand::SetBlock { pos, block }),
        }
    }

    /// 任务书测试 `setblock_batch_atomic_on_inbox_full`:批量目标之一收件箱
    /// 满 → Err;全部 sim 体素未变、已注册目标的收件箱也未增(不半投)。
    #[test]
    fn setblock_batch_atomic_on_inbox_full() {
        let mut sched = Scheduler::new(SchedulerConfig {
            router: crate::router::RouterConfig {
                inbox_capacity: 1,
                ..crate::router::RouterConfig::default()
            },
            ..sched_cfg(2)
        });
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let cp = types::ChunkPos::new(7, 0); // owner = 片 (0,0);片 (1,0) 持扩张副本
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        sched.apply_chunk_load(shard_id(1, 0), cp, voxel::Chunk::air());

        // 先占满片 (1,0) 的收件箱(容量 1)。
        sched
            .submit(
                shard_id(1, 0),
                setblock_input(Tick(0), 0, BlockPos::new(119, 5, 8), types::BlockId(9)),
            )
            .expect("首条入箱");
        // 批量(owner + holder)第二格:片 (1,0) 满 → 整批 Err。
        let targets = [shard_id(0, 0), shard_id(1, 0)];
        let batch = setblock_input(Tick(0), 1, BlockPos::new(121, 5, 8), types::BlockId(3));
        assert!(
            sched.submit_batch(&targets, batch).is_err(),
            "任一收件箱满 → 整批拒绝"
        );
        // 不半投:拍 0 后,被拒批次的格 (121,·,8) 在两片都未写;
        // 占位单发条目 (119,·,8) 照常生效(对照)。
        sched.tick();
        let owner_chunk = sched.voxel_chunk(shard_id(0, 0), cp).expect("已装载");
        let holder_chunk = sched.voxel_chunk(shard_id(1, 0), cp).expect("已装载");
        for (name, chunk) in [("owner", owner_chunk), ("holder", holder_chunk)] {
            assert_eq!(
                chunk.get(9, 69, 8),
                types::BlockId::AIR,
                "{name} 副本:被拒批次 (121,·,8) 未写(不半投)"
            );
        }
        assert_eq!(
            holder_chunk.get(7, 69, 8),
            types::BlockId(9),
            "占位单发条目照常生效(对照,局部 (7,69,8) = 世界 (119,5,8))"
        );
        assert_eq!(sched.router.pending_inbox(shard_id(0, 0)), 0, "不半投");
        sched.shutdown();
    }

    /// 任务书测试 `block_updates_published_once_by_owner`:同格编辑广播到
    /// owner + holder,两片各产生一条 block_update;`take_block_updates`
    /// 只返回 owner 的一条(且取走即清,再取 = 空)。
    #[test]
    fn block_updates_published_once_by_owner() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let cp = types::ChunkPos::new(7, 0); // owner = 片 (0,0)
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        sched.apply_chunk_load(shard_id(1, 0), cp, voxel::Chunk::air());
        // 同格同章批量广播(两片各产生一条同格写入)。
        let batch = setblock_input(Tick(0), 0, BlockPos::new(120, 5, 8), types::BlockId(2));
        sched
            .submit_batch(&[shard_id(0, 0), shard_id(1, 0)], batch)
            .expect("收件箱未满");
        sched.tick();
        let updates = sched.take_block_updates();
        assert_eq!(
            updates,
            vec![(shard_id(0, 0), BlockPos::new(120, 5, 8), types::BlockId(2))],
            "两片各产生一条,只发布 owner 的一条:{updates:?}"
        );
        assert!(sched.take_block_updates().is_empty(), "取走即清(恰一次)");
        sched.shutdown();
    }

    /// 任务书测试 `block_edit_copies_bitwise_equal`:边界格编辑同拍生效后,
    /// 两片该 chunk 的 `Chunk` 逐字节相等(`Chunk` = Vec<BlockId>,PartialEq
    /// 即逐格全等)。
    #[test]
    fn block_edit_copies_bitwise_equal() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let cp = types::ChunkPos::new(7, 0); // owner = 片 (0,0);片 (1,0) 持扩张副本
        let chunk = voxel::Chunk::air();
        sched.apply_chunk_load(shard_id(0, 0), cp, chunk.clone());
        sched.apply_chunk_load(shard_id(1, 0), cp, chunk);
        let batch = setblock_input(Tick(0), 0, BlockPos::new(120, 40, 8), types::BlockId(7));
        sched
            .submit_batch(&[shard_id(0, 0), shard_id(1, 0)], batch)
            .expect("收件箱未满");
        sched.tick();
        let a = sched.voxel_chunk(shard_id(0, 0), cp).expect("已装载");
        let b = sched.voxel_chunk(shard_id(1, 0), cp).expect("已装载");
        assert_eq!(a, b, "同格编辑后两片副本逐字节相等");
        // 非退化:编辑格确为新值(chunk 局部 = 世界格 mod 16,y +64)。
        assert_eq!(a.get(8, 104, 8), types::BlockId(7));
        sched.shutdown();
    }

    /// 任务书测试 `boundary_edit_physics_matches_single_world`:边界格编辑 +
    /// 实体滚动穿越,双片 vs 单世界(整走廊单 ShardSim 参考)同时间线终态
    /// 位移差 < 1e-4 m。参考基线:AC2 e2e 已证明"分片走廊 vs 裸 PhysicsWorld
    /// 单世界"误差 ~5e-6 m(crates/e2e/tests/migration.rs),本测试在此之上
    /// 验证"编辑格两片同拍生效"不引入额外差异(单世界参考不经广播)。
    #[test]
    fn boundary_edit_physics_matches_single_world() {
        use types::glam::Vec3;

        const GROUND_TOP_LY: usize = 128; // 顶面 world y = 64(AC2 同款)
        fn ground_chunk() -> voxel::Chunk {
            let mut c = voxel::Chunk::air();
            for y in 0..GROUND_TOP_LY {
                for x in 0..voxel::CHUNK_X {
                    for z in 0..voxel::CHUNK_Z {
                        c.set(x, y, z, types::BlockId(1));
                    }
                }
            }
            c
        }
        // 双片:本体 + 扩张带重叠(片 0 = chunk 0..=8,片 1 = chunk 7..=15)。
        let mut sched = Scheduler::new(sched_cfg(2));
        for (x, range) in [(0i32, 0..9), (1, 7..16)] {
            let mut s = {
                let mut cfg = ShardSimConfig::new(shard_id(x, 0));
                cfg.solidify.enabled = false;
                ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
            };
            for cx in range {
                s.load_chunk(types::ChunkPos::new(cx, 0), ground_chunk());
            }
            sched.register(s);
        }
        // 边界格台阶(chunk 7/8 交界):owner (0,0) + holder (1,0) 同拍广播。
        let step = BlockPos::new(127, 64, 8);
        let batch = setblock_input(Tick(0), 0, step, types::BlockId(1));
        sched
            .submit_batch(&[shard_id(0, 0), shard_id(1, 0)], batch)
            .expect("收件箱未满");
        // 实体:出生活在地面上方,linvel 60 m/s 沿 +x,滚动穿越台阶与分片边界
        // (拍 1 前缀生效,共 44 步物理;AC2 同款时间线)。
        let e = types::EntityId::from_parts(1, 1);
        sched
            .submit(
                shard_id(0, 0),
                StampedInput {
                    apply_at: Tick(1),
                    source: InputSource(7),
                    sequence: 0,
                    payload: InputPayload::Command(SimCommand::SpawnDynamic {
                        entity: e,
                        state: types::EntityState {
                            translation: Vec3::new(100.0, 65.5, 8.0),
                            linvel: Vec3::new(60.0, 0.0, 0.0),
                            ..types::EntityState::default()
                        },
                        desc: types::EntityDesc::default(),
                    }),
                },
            )
            .expect("收件箱未满");
        const TICKS: usize = 45;
        for _ in 0..TICKS {
            sched.tick();
        }
        // 实体高速穿边界:迁移当拍源片删除、下一拍目的片重建,终态可能在任一
        // 片(实验:45 拍实体冲出片 1 本体区,被片 2 接住;迁移链属 AC2 已验
        // 面本测试不展开)——两路对照取并集口径:任一片查到即终态。
        let two_shard_end = sched
            .entity_state(shard_id(1, 0), e)
            .or_else(|| sched.entity_state(shard_id(0, 0), e))
            .or_else(|| sched.entity_state(shard_id(2, 0), e))
            .expect("实体仍在两片走廊的某一片");
        sched.shutdown();

        // 单世界参考:整走廊单 ShardSim(chunk 0..16 全量装载,同格直接 SetBlock;
        // 凝固桥关闭)。不经广播路径——两路终态位移差 < 1e-4 m。
        let mut single = {
            let mut cfg = ShardSimConfig::new(shard_id(0, 0));
            cfg.solidify.enabled = false;
            ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
        };
        for cx in 0..16 {
            single.load_chunk(types::ChunkPos::new(cx, 0), ground_chunk());
        }
        single.apply_prefix(&[
            StampedInput {
                apply_at: Tick(0),
                source: InputSource(1),
                sequence: 0,
                payload: InputPayload::Command(SimCommand::SetBlock {
                    pos: step,
                    block: types::BlockId(1),
                }),
            },
            StampedInput {
                apply_at: Tick(1),
                source: InputSource(7),
                sequence: 0,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: e,
                    state: types::EntityState {
                        translation: Vec3::new(100.0, 65.5, 8.0),
                        linvel: Vec3::new(60.0, 0.0, 0.0),
                        ..types::EntityState::default()
                    },
                    desc: types::EntityDesc::default(),
                }),
            },
        ]);
        // 44 步物理(拍 1..44 各一步;apply_at=1 的输入在首步前应用)。
        for _ in 1..TICKS {
            single.step();
        }
        let single_end = single.entity_state(e).expect("单世界实体常驻");
        let d = two_shard_end.translation - single_end.translation;
        assert!(
            d.x.abs() < 1e-4 && d.y.abs() < 1e-4 && d.z.abs() < 1e-4,
            "边界编辑后双片 vs 单世界终态位移差超限:Δ=({},{},{}) m",
            d.x,
            d.y,
            d.z
        );
    }

    /// is_chunk_loaded 透传:已装载 true、未装载 false、未注册分片 false。
    #[test]
    fn is_chunk_loaded_passthrough() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        let cp = types::ChunkPos::new(2, 3);
        assert!(!sched.is_chunk_loaded(shard_id(0, 0), cp), "未装载 → false");
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        assert!(sched.is_chunk_loaded(shard_id(0, 0), cp), "已装载 → true");
        assert!(
            !sched.is_chunk_loaded(shard_id(9, 9), cp),
            "未注册分片 → false"
        );
        sched.shutdown();
    }
}
