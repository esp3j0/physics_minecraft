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

use messaging::{AdmitError, CrossEvent, InputPayload, InputSource, StampedInput};
use physics_sim::{BlockWriteKind, Outbox, ShardSim};
use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{BlockId, BlockPos, ChunkPos, EntityId, EntityState, ShardId, Tick};

use crate::coordinator::TickCoordinator;
use crate::entities::{EntityOwner, EntityOwnership};
use crate::lifecycle::{LifecycleConfig, LifecycleSnapshot, ShardLifecycle};
use crate::ownership::OwnershipMap;
use crate::router::{RouteError, Router, RouterConfig, RouterSnapshot};

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

/// 调度器快照(REQ-005 任务书 02,签名固定):观察者表 + 设置过标志 +
/// Router 待投递信封与 sequences + 各分片生命周期。
///
/// **不入快照**(细则 1):`observer_inbox`/`router.inboxes`(观察者与分片
/// 的未来命令由日志回放)、`activation_requests`/`despawn_requests`/
/// `solidified`/`block_updates`(每拍派生物,下一拍重算)、`chunk_holders`
/// (从各分片已加载 chunk 重建)、`lifecycles` 的 `Stopped` 观测残留
/// (unload_shard 留下;快照只存当前 `sims` 键集)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SchedulerSnapshot {
    pub tick: Tick,
    pub observers: BTreeMap<InputSource, Vec3>,
    pub observers_ever_set: bool,
    /// 票据表(REQ-007 任务 02 交付 7;票据收件箱未来条目不入——由日志回放)。
    pub tickets: crate::tickets::TicketTable,
    /// 曾存在区域票(开关推广的第二来源;单向)。
    pub tickets_ever_set: bool,
    pub router: RouterSnapshot,
    pub lifecycles: BTreeMap<ShardId, LifecycleSnapshot>,
}

/// 票视图锚点列表(命令唤醒预算强制的距离中心;order 序,确定性)。
fn views_anchors(views: &[physics_sim::tickets::TicketView]) -> Vec<Vec3> {
    views.iter().map(|t| t.anchor).collect()
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
    /// 曾存在区域票(REQ-007 任务 02:开关推广的第二来源;单向,随
    /// `observers_ever_set` 同语义;区域票移除不回退)。
    tickets_ever_set: bool,
    /// 调度器级观察者专用收件箱(REQ-002 §4.2;unbounded——任务书语义
    /// 细则 1:观察者频率低,有界语义复杂度不值得,REQ-003 网络层再收口)。
    /// REQ-007 任务 02:推广为**票据收件箱**(§9-5,观察者 Set/Remove +
    /// 区域票 Set/Remove 同箱同语义;观察者命令语义不变)。
    observer_inbox: VecDeque<StampedInput>,
    /// 票据表(REQ-007 §4.2:调度器级全局唯一一张;键序遍历)。观察者票
    /// 与 observer_table 同步(命令化条目双写:表 = 锚点来源,票 = 决策输入);
    /// 区域票/实体票(03)只在此表。
    tickets: crate::tickets::TicketTable,
    /// 区域票上限(app::Config 注入;默认 128 m / 64 张,§4.2.3)。
    area_ticket_max_radius_m: f32,
    area_ticket_max_count: usize,
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
    /// 实体归属索引(REQ-006 任务 00):各分片实体表是真相,本表每拍同步;
    /// 不入快照,恢复时 `rebuild_entity_ownership` 从实体表重建。
    entity_ownership: EntityOwnership,
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
            tickets_ever_set: false,
            observer_inbox: VecDeque::new(),
            tickets: crate::tickets::TicketTable::new(),
            area_ticket_max_radius_m: 128.0,
            area_ticket_max_count: 64,
            activation_requests: BTreeMap::new(),
            solidified: Vec::new(),
            block_updates: Vec::new(),
            despawn_requests: BTreeSet::new(),
            chunk_holders: crate::chunks::ChunkHolders::new(),
            entity_ownership: EntityOwnership::new(),
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
        // 旧表全部撤票(直通 = 整表替换)。
        let old_keys: Vec<_> = self
            .observer_table
            .keys()
            .map(|src| {
                (
                    physics_sim::tickets::TicketHolder::Observer(*src),
                    physics_sim::tickets::TicketKind::Observer,
                )
            })
            .collect();
        for key in old_keys {
            self.tickets.end(key);
        }
        self.observer_table.clear();
        let len = observers.len();
        let cfg_r = self
            .sims
            .values()
            .next()
            .map(|s| *s.activation_config())
            .unwrap_or_default();
        for (j, pos) in observers.into_iter().enumerate() {
            let key = if j + 1 == len {
                InputSource::SCHEDULER
            } else {
                InputSource(u32::MAX - (len - 2 - j) as u32)
            };
            self.observer_table.insert(key, pos);
            // 观察者票双写(REQ-007 任务 02;直通路径同语义,续期)。
            self.tickets.issue(
                (
                    physics_sim::tickets::TicketHolder::Observer(key),
                    physics_sim::tickets::TicketKind::Observer,
                ),
                crate::tickets::StoredTicket {
                    priority: 0,
                    data: physics_sim::tickets::RegionSpec::Radius {
                        load_m: cfg_r.data_load_radius,
                        unload_m: cfg_r.data_unload_radius,
                    },
                    collider: physics_sim::tickets::RegionSpec::Radius {
                        load_m: cfg_r.collider_load_radius,
                        unload_m: cfg_r.collider_unload_radius,
                    },
                    wake: physics_sim::tickets::WakeSpec::Radius {
                        load_m: cfg_r.awake_load_radius,
                        unload_m: cfg_r.awake_unload_radius,
                    },
                    clock: physics_sim::tickets::TicketClock {
                        issued_at: self.coordinator.tick(),
                        active_ticks: 0,
                        limit: None,
                        starts_at: None,
                    },
                    state: physics_sim::tickets::TicketState::Active,
                    center: None,
                },
                self.coordinator.tick(),
            );
        }
        self.observers_ever_set = true;
    }

    /// D17 激活启用开关(REQ-007 §4.0,任务 02 推广):曾设置过观察者**或**
    /// 曾存在区域票即启用。单向,随快照(`observers_ever_set` 位置;区域票
    /// 曾在亦置位——移除后不回退)。未启用时:装载合一(① + ② 同建,
    /// `apply_chunk_load` 走旧路径)、激活完全不生效——行为与 `8702670` 一致。
    pub fn activation_enabled(&self) -> bool {
        self.observers_ever_set || self.tickets_ever_set
    }

    /// 票据表只读访问(测试/断言用)。
    pub fn ticket_table(&self) -> &crate::tickets::TicketTable {
        &self.tickets
    }

    /// 区域票半径上限(校验用;config 注入的默认 128)。
    pub fn area_ticket_max_radius(&self) -> f32 {
        self.area_ticket_max_radius_m
    }

    /// 区域票数量上限。
    pub fn area_ticket_max_count(&self) -> usize {
        self.area_ticket_max_count
    }

    /// 区域票 ① "进"区域覆盖的分片集(REQ-007 任务 02 细则 7:驱动 ensure
    /// 用)。Radius 票:锚点圆覆盖(水平)的分片,键序。Suspended 不 ensure。
    pub fn area_ticket_shards(&self) -> BTreeSet<ShardId> {
        use physics_sim::tickets::{TicketHolder, TicketState};
        const SHARD_M: f32 = (types::SHARD_CHUNKS * types::CHUNK_SIZE) as f32;
        let mut out = BTreeSet::new();
        for (holder, t) in self.tickets.iter() {
            if t.state == TicketState::Suspended {
                continue;
            }
            let TicketHolder::Area(id) = holder.0 else {
                continue; // 观察者票走 ① 观察者邻片 ensure;实体票 03
            };
            let Some(center) = t.center else { continue };
            let physics_sim::tickets::RegionSpec::Radius { load_m, .. } = t.data else {
                continue;
            };
            let min_x = ((center.x - load_m) / SHARD_M).floor() as i32;
            let max_x = ((center.x + load_m) / SHARD_M).floor() as i32;
            let min_z = ((center.z - load_m) / SHARD_M).floor() as i32;
            let max_z = ((center.z + load_m) / SHARD_M).floor() as i32;
            for sx in min_x..=max_x {
                for sz in min_z..=max_z {
                    // 分片核心 AABB 水平距离 ≤ load_m(与 shards_within 同口径)。
                    let (cx, cz) = (sx as f32 * SHARD_M, sz as f32 * SHARD_M);
                    let dx = (cx - center.x).max(0.0).max(center.x - (cx + SHARD_M));
                    let dz = (cz - center.z).max(0.0).max(center.z - (cz + SHARD_M));
                    if dx * dx + dz * dz <= load_m * load_m {
                        let _ = id;
                        out.insert(ShardId::new(
                            types::WorldId(0),
                            types::DimensionId(0),
                            types::ShardPos::new(sx, sz),
                        ));
                    }
                }
            }
        }
        out
    }

    /// 区域票上限注入(bootstrap/测试;装配期一次性)。
    pub fn set_area_ticket_caps(&mut self, max_radius_m: f32, max_count: usize) {
        self.area_ticket_max_radius_m = max_radius_m;
        self.area_ticket_max_count = max_count;
    }

    /// 区域票签发(生产入口;Session 经此入表)。幂等语义在
    /// `TicketTable::issue`(同 id 覆盖 = 续期 S3)。上限校验在 Session 层
    /// (类型化拒绝);本入口假设已校验。
    pub fn apply_area_ticket(
        &mut self,
        id: physics_sim::tickets::AreaId,
        center: types::glam::Vec3,
        radius_m: f32,
        tiers: u8,
        ttl_ticks: Option<u64>,
    ) {
        use physics_sim::tickets::{
            RegionSpec, TicketClock, TicketHolder, TicketKind, TicketState, WakeSpec,
        };
        // 同 id 已在 → 计数不变(覆盖);否则占新名额(Session 层已校验上限)。
        let tiers_mask = tiers & 0b111;
        let data = if tiers_mask & 0b001 != 0 {
            RegionSpec::Radius {
                load_m: radius_m,
                unload_m: radius_m * 1.5,
            }
        } else {
            RegionSpec::None
        };
        let collider = if tiers_mask & 0b010 != 0 {
            RegionSpec::Radius {
                load_m: radius_m,
                unload_m: radius_m * 1.5,
            }
        } else {
            RegionSpec::None
        };
        let wake = if tiers_mask & 0b100 != 0 {
            WakeSpec::Radius {
                load_m: radius_m,
                unload_m: radius_m * 1.5,
            }
        } else {
            WakeSpec::None
        };
        let key = (TicketHolder::Area(id), TicketKind::Area);
        let is_new = self.ticket_table().get(key).is_none();
        self.tickets.issue(
            key,
            crate::tickets::StoredTicket {
                priority: 1, // §4.2.3:区域票 priority 1(观察者 0)
                data,
                collider,
                wake,
                clock: TicketClock {
                    issued_at: self.coordinator.tick(),
                    active_ticks: 0,
                    limit: ttl_ticks,
                    starts_at: None,
                },
                state: TicketState::Active,
                center: Some(center),
            },
            self.coordinator.tick(),
        );
        if is_new {
            // 曾有区域票 → 启用激活(单向;§4.0/D17 推广)。
            self.tickets_ever_set = true;
        }
    }

    /// 区域票移除(幂等;§4.3.1 `End`)。
    pub fn apply_remove_area_ticket(&mut self, id: physics_sim::tickets::AreaId) {
        use physics_sim::tickets::{TicketHolder, TicketKind};
        self.tickets.end((TicketHolder::Area(id), TicketKind::Area));
    }

    /// 锚点派生(任务书细则 3):观察者票 = 观察者坐标;区域票 = center;
    /// 实体票 = 实体位置(03 接;当前无实体票,`None` → views 跳过/panic
    /// 语义由 TicketTable::views 决定)。观察者不在表(已删)而票在 = 一致
    /// 违规(views 内 panic;此处观察者票锚点直接查表)。
    fn ticket_anchor_of(&self, holder: &physics_sim::tickets::TicketHolder) -> Option<Vec3> {
        use physics_sim::tickets::TicketHolder;
        match holder {
            TicketHolder::Observer(src) => self.observer_table.get(src).copied(),
            TicketHolder::Area(_) => None, // 由调用方走 center(见 ticket_views)
            TicketHolder::Entity(_) => None, // 03 接
        }
    }

    /// 决策用票视图(键序;区域票锚点取 center)。观察者票无锚(已删而票在)
    /// → 一致性违规 panic(TicketTable::views)。注意:观察者票与 observer_table
    /// 双写同步(Set 应用时同时落表),此处派生顺序:观察者票锚点查表,区域票
    /// 锚点查 center。
    fn ticket_views(&self) -> Vec<physics_sim::tickets::TicketView> {
        use physics_sim::tickets::TicketHolder;
        let area_centers: std::collections::BTreeMap<physics_sim::tickets::AreaId, Vec3> = self
            .tickets
            .iter()
            .filter_map(|(k, t)| match k.0 {
                TicketHolder::Area(id) => t.center.map(|c| (id, c)),
                _ => None,
            })
            .collect();
        self.tickets.views(|holder| match holder {
            TicketHolder::Observer(src) => self.observer_table.get(src).copied(),
            TicketHolder::Area(id) => area_centers.get(id).copied(),
            TicketHolder::Entity(_) => None,
        })
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
            // 观察者票随锚点消失(REQ-007 任务 02;与收件箱 Remove 分支同语义)。
            self.tickets.end((
                physics_sim::tickets::TicketHolder::Observer(source),
                physics_sim::tickets::TicketKind::Observer,
            ));
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
    /// `(分片, 世界坐标方块格, 写入的方块 ID)`。任务 03 起两类条目都进本集:
    /// `External` 只发布 owner 的条目(广播下每个 holder 副本各产生一条同格
    /// 写入,非 owner 的丢弃);`Emergent` 发布产生者的条目(网络在产生拍
    /// 可见,owner 持久化前差一拍——任务书语义已接受)。
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

    /// 待投递 `BlockWrite` 信封的目标格(REQ-005 任务 03 终审修复;两拍
    /// 之间):`目的分片 → 仍有未应用涌现写入的 chunk 集`。驱动激活卸载
    /// 决策处延迟命中 chunk 的卸载(信封 N+1 应用前接收方卸载目标 chunk
    /// 是时序竞态,不该 fail-stop)。信封应用后查询自然清空。
    pub fn pending_blockwrite_cells(&self) -> BTreeMap<ShardId, BTreeSet<ChunkPos>> {
        self.router.pending_blockwrite_cells()
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

    /// ② 是否已建(REQ-007 任务 01;基线比对/断言观测用)。
    /// 分片未注册 → false。
    pub fn has_collider(&self, shard: ShardId, pos: ChunkPos) -> bool {
        self.sims
            .get(&shard)
            .is_some_and(|sim| sim.has_collider(pos))
    }

    /// 覆盖判定(REQ-005 §4.3.1/任务书 03,签名固定):核心区(core AABB,
    /// 8×8 chunk × 128m 的水平立柱;y 不参与——分片是整高柱)或该分片持有的
    /// 任一 chunk(owner 本体副本或扩张缓存副本,查 `chunk_holders`——表内
    /// 条目即"该分片已装载此 chunk")与事件 AABB(center ± radius,水平面)
    /// 相交的**已注册**分片,ShardId 升序(`BTreeSet` 键序)。中心分片未注册
    /// 不影响邻片命中(爆心可落在未注册区域)。
    pub fn shards_covering(&self, center: Vec3, radius: f32) -> BTreeSet<ShardId> {
        const SHARD_M: f32 = (types::SHARD_CHUNKS * types::CHUNK_SIZE) as f32;
        const CHUNK_M: f32 = types::CHUNK_SIZE as f32;
        // 事件 AABB(水平面;y 无界——柱状交集只看 x/z)。
        let (ev_min_x, ev_max_x) = (center.x - radius, center.x + radius);
        let (ev_min_z, ev_max_z) = (center.z - radius, center.z + radius);
        // 轴向相交判定(闭区间:相切 = 相交,半径语义取含边界)。
        let range_hits =
            |a_min: f32, a_max: f32, b_min: f32, b_max: f32| a_min <= b_max && b_min <= a_max;
        let mut covered: BTreeSet<ShardId> = BTreeSet::new();
        for &shard in self.sims.keys() {
            // ① core AABB(128m 方柱)与事件 AABB 相交。
            let min = shard.pos.min_chunk();
            let core_min_x = min.x as f32 * CHUNK_M;
            let core_min_z = min.z as f32 * CHUNK_M;
            let core_max_x = core_min_x + SHARD_M;
            let core_max_z = core_min_z + SHARD_M;
            if range_hits(core_min_x, core_max_x, ev_min_x, ev_max_x)
                && range_hits(core_min_z, core_max_z, ev_min_z, ev_max_z)
            {
                covered.insert(shard);
                continue;
            }
            // ② 该分片持有的任一 chunk 与事件 AABB 相交(holder 表登记即
            //    已装载;chunk AABB = 16m 方柱)。
            let hits = self.chunk_holders.chunks_of(shard).iter().any(|&cp| {
                let c_min_x = cp.x as f32 * CHUNK_M;
                let c_min_z = cp.z as f32 * CHUNK_M;
                range_hits(c_min_x, c_min_x + CHUNK_M, ev_min_x, ev_max_x)
                    && range_hits(c_min_z, c_min_z + CHUNK_M, ev_min_z, ev_max_z)
            });
            if hits {
                covered.insert(shard);
            }
        }
        covered
    }

    /// 涌现事件广播(REQ-005 §4.3.2/D10,任务书 03 签名固定):目标 =
    /// `shards_covering(event 中心/半径) − {origin}`(origin 已在 step 内
    /// 本地求值,不给自己发;空目标集 → `Ok`)。逐目标
    /// `Intent { payload: Event }` 经 `Router::collect_intents` 盖章
    /// (`apply_at = produced_at + 1`,per-source sequence 保证 canonical 序,
    /// 每目标恰好一个信封)。队列满 → `Err(AdmitError::QueueFull)` 类型化
    /// 反压(不静默丢;`tick()` 内调用方 fail-stop)。
    pub fn broadcast_event(
        &mut self,
        origin: ShardId,
        event: CrossEvent,
        produced_at: Tick,
    ) -> Result<(), AdmitError> {
        let (center, radius) = match event {
            CrossEvent::Detonate {
                center,
                power: _,
                radius,
            } => (center, radius),
        };
        let intents: Vec<messaging::Intent> = self
            .shards_covering(center, radius)
            .into_iter()
            .filter(|&dest| dest != origin)
            .map(|dest| messaging::Intent {
                destination: dest,
                payload: messaging::EnvelopePayload::Event(event),
            })
            .collect();
        if intents.is_empty() {
            return Ok(());
        }
        self.router.collect_intents(origin, &intents, produced_at)
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
        // D17 装载分支(REQ-007 §4.0/任务 01):启用 → 只装 ①(② 由激活
        // 决策构建);未启用 → 合一装载(① + ② 同建,行为与 8702670 一致)。
        let enabled = self.activation_enabled();
        self.sims
            .get_mut(&shard)
            .expect("上方已校验注册")
            .apply_chunk_load_tiered(pos, chunk, enabled);
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

    /// 从各分片实体表重建全局 ownership(REQ-006 任务 00;表不入快照)。
    /// 重复 EntityId → panic(fail-stop)。
    pub fn rebuild_entity_ownership(&mut self) {
        let entries: Vec<(EntityId, EntityOwner)> = self
            .sims
            .iter()
            .flat_map(|(shard, sim)| {
                sim.entity_owners()
                    .into_iter()
                    .map(move |(id, source, actor)| {
                        (
                            id,
                            EntityOwner {
                                shard: *shard,
                                source,
                                actor,
                            },
                        )
                    })
            })
            .collect();
        self.entity_ownership.rebuild(entries);
    }

    /// 用分片实体表(真相)同步本拍 ownership:释放消失的实体,claim 新实体,
    /// 更新 actor 元数据;同一 ID 出现在两个分片 → claim/duplicate panic。
    fn sync_entity_ownership(&mut self, shard: ShardId, sim: &ShardSim) {
        let current = sim.entity_owners();
        let current_ids: BTreeSet<EntityId> = current.iter().map(|(id, _, _)| *id).collect();
        // 先释放本片已消失的条目(迁移/Remove/凝固)。
        let stale: Vec<EntityId> = self
            .entity_ownership
            .entities_of(shard)
            .into_iter()
            .filter(|id| !current_ids.contains(id))
            .collect();
        for id in stale {
            self.entity_ownership.release(id);
        }
        // 再 claim/update 当前实体;若 ID 属于别的分片 = 全局重复,fail-stop。
        for (id, source, actor) in current {
            let owner = EntityOwner {
                shard,
                source,
                actor,
            };
            match self.entity_ownership.owner_of(id) {
                None => self.entity_ownership.claim(id, owner),
                Some(prev) if prev.shard == shard => self.entity_ownership.update(id, owner),
                Some(prev) => panic!(
                    "实体 {id:?} 同时出现在分片 {shard:?} 与 {:?}(全局唯一性 fail-stop)",
                    prev.shard
                ),
            }
        }
    }

    /// 实体当前归属(REQ-006);不存在/迁移窗口 → None。
    pub fn owner_of_entity(&self, entity: EntityId) -> Option<EntityOwner> {
        self.entity_ownership.owner_of(entity)
    }

    /// 某分片当前持有的实体集合(升序)。
    pub fn entities_of(&self, shard: ShardId) -> BTreeSet<EntityId> {
        self.entity_ownership.entities_of(shard)
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
        self.rebuild_entity_ownership();
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
        //     REQ-005 任务书 02:SetObserverPosition → 插入/覆盖 (source,
        //     position);RemoveObserver → 删除 source(不存在 = 幂等不
        //     panic,票与激活中心随条目消失);其它载荷 = 一致性违规 fail-stop。
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
                match input.payload {
                    InputPayload::Command(messaging::SimCommand::SetObserverPosition {
                        position,
                    }) => {
                        self.observer_table.insert(input.source, position);
                        self.observers_ever_set = true;
                        // 观察者票双写(REQ-007 任务 02:observer_table = 锚点
                        // 来源,票据表 = 决策输入;覆盖 = 续期,规格与任务 01
                        // 等价路径同源)。半径取激活配置(任一 sim 同源;无 sim
                        // 时默认——与 shard_is_ticketed 同口径)。
                        let cfg_r = self
                            .sims
                            .values()
                            .next()
                            .map(|s| *s.activation_config())
                            .unwrap_or_default();
                        self.tickets.issue(
                            (
                                physics_sim::tickets::TicketHolder::Observer(input.source),
                                physics_sim::tickets::TicketKind::Observer,
                            ),
                            crate::tickets::StoredTicket {
                                priority: 0,
                                data: physics_sim::tickets::RegionSpec::Radius {
                                    load_m: cfg_r.data_load_radius,
                                    unload_m: cfg_r.data_unload_radius,
                                },
                                collider: physics_sim::tickets::RegionSpec::Radius {
                                    load_m: cfg_r.collider_load_radius,
                                    unload_m: cfg_r.collider_unload_radius,
                                },
                                wake: physics_sim::tickets::WakeSpec::Radius {
                                    load_m: cfg_r.awake_load_radius,
                                    unload_m: cfg_r.awake_unload_radius,
                                },
                                clock: physics_sim::tickets::TicketClock {
                                    issued_at: at,
                                    active_ticks: 0,
                                    limit: None,
                                    starts_at: None,
                                },
                                state: physics_sim::tickets::TicketState::Active,
                                center: None,
                            },
                            at,
                        );
                    }
                    InputPayload::Command(messaging::SimCommand::RemoveObserver) => {
                        // 幂等:不在表 = 无动作(断线竞先/重复移除无害,票撤
                        // 语义与 `remove_observer` 一致);表空不清
                        // `observers_ever_set`(同 `remove_observer`)。
                        let okey = (
                            physics_sim::tickets::TicketHolder::Observer(input.source),
                            physics_sim::tickets::TicketKind::Observer,
                        );
                        self.observer_table.remove(&input.source);
                        self.tickets.end(okey); // 观察者票随锚点消失(§4.3.4)
                    }
                    // 区域票命令(REQ-007 任务 02):票据收件箱同语义——当拍
                    // 生效;签发/移除幂等语义在 TicketTable(S3 续期/End 幂等)。
                    InputPayload::Command(messaging::SimCommand::SetAreaTicket {
                        id,
                        center,
                        radius_m,
                        tiers,
                        ttl_ticks,
                    }) => {
                        self.apply_area_ticket(id, center, radius_m, tiers, ttl_ticks);
                    }
                    InputPayload::Command(messaging::SimCommand::RemoveAreaTicket { id }) => {
                        self.apply_remove_area_ticket(id);
                    }
                    _ => panic!(
                        "票据收件箱只收 Set/RemoveObserver/Set/RemoveAreaTicket\
                         (收到其它载荷,调用方 bug,fail-stop)"
                    ),
                }
            }
        }

        // 1.5(README 决策 8;任务书 req001-backlog/02):观察者已设置过 →
        //     每分片激活刷新(tier③ 冻结/唤醒在本拍 step 生效);tier①②
        //     chunk 装卸请求自本任务起外露(REQ-002 §4.4,两拍之间取用)。
        //     观察者中心集 = 命令化表 + 直通列表(两条写入路径,一张语义表);
        //     两者皆从未设置 → 完全跳过,行为与历史一致(向后兼容开关)。
        self.activation_requests.clear();
        if self.activation_enabled() {
            // 票据视图派生(REQ-007 任务 02:锚点 = 票持有者;plan 消费视图,
            // 无票种分支)。仅观察者票时与任务 01 等价(票规格同源半径)。
            let views = self.ticket_views();
            for (shard, sim) in self.sims.iter_mut() {
                let decisions = sim.apply_activation_tickets(&views, &views_anchors(&views));
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
        //    迭代天然升序):迁移意图盖章入队;涌现事件广播;凝固/涌现 block
        //    write 分流(External 发布 / Emergent 发布+solidified 兼容+信封)。
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
            // 涌现事件(REQ-005 任务 03):`Outbox.events` 逐个广播(目标 =
            // shards_covering − 自片;apply_at = N+1)。队列满 = fail-stop
            // (反压策略待接入,同迁移盖章)。本任务 sim 不产事件,通道就绪。
            for event in &outbox.events {
                match self.broadcast_event(shard, *event, at) {
                    Ok(()) => {}
                    Err(AdmitError::QueueFull { capacity }) => panic!(
                        "涌现事件队列满(capacity {capacity})于拍 {}:fail-stop,\
                         反压策略待接入(REQ-005 任务书 03)",
                        at.0
                    ),
                    Err(e) => panic!("涌现事件盖章被拒(编程错误,fail-stop):{e}"),
                }
            }
            // (shard, 格, 写入的方块):凝固写入值读自 sim 体素世界(写入先于
            // 本处,凝固路径已落格)。任务 03 起凝固桥产 `Emergent`——同一格
            // 的条目在下方分流时一并写入本兼容字段(同格同值,take_solidified
            // 语义不变,驱动/既有测试零改动)。
            for (_, cell) in &outbox.solidified {
                self.solidified
                    .push((shard, *cell, sim.voxel().get_block(*cell)));
            }
            // 方块写入分流(REQ-005 D11 + 任务 03):
            // - `External`(外部 SetBlock 广播):只发布 owner 的条目(非 owner
            //   副本的条目丢弃),不做信封——同拍广播已覆盖全部 holder。
            // - `Emergent`(凝固桥等涌现写入):①发布产生者的条目(网络在产生
            //   拍可见;owner 持久化前的可见性差一拍,任务书语义已接受);
            //   ②同步进 `solidified` 兼容字段(take_solidified 语义不变);
            //   ③对 owner + holders − {producer} 逐个盖 `EnvelopePayload::
            //   BlockWrite`(apply_at = N+1,副本下一拍一致);producer 若自持
            //   该 chunk(owner 或 holder)不发自己——已本地落格。
            //   owner 未持有该 chunk → panic(任务 00 ensure 保证的不变量)。
            for write in &outbox.block_updates {
                let chunk = write.pos.chunk_of();
                let owner = Self::owner_of(shard, chunk);
                match write.kind {
                    BlockWriteKind::External => {
                        if shard == owner {
                            self.block_updates.push((shard, write.pos, write.block));
                        }
                    }
                    BlockWriteKind::Emergent => {
                        // ① 产生者发布(网络可见性在产生拍)。
                        self.block_updates.push((shard, write.pos, write.block));
                        // ② solidified 兼容字段(观察输出源②语义不变)。
                        self.solidified.push((shard, write.pos, write.block));
                        // ③ owner + holders − producer 的信封(下一拍副本一致)。
                        let mut targets = self.chunk_holders.holders(owner, chunk);
                        if !targets.contains(&owner) {
                            panic!(
                                "涌现 block write 的 owner {owner:?} 未持有 chunk {chunk:?}\
                                 (任务 00 ensure 保证的不变量被破坏,fail-stop)"
                            );
                        }
                        targets.remove(&shard); // producer 已本地落格,不发自己
                        let intents: Vec<messaging::Intent> = targets
                            .into_iter()
                            .map(|dest| messaging::Intent {
                                destination: dest,
                                payload: messaging::EnvelopePayload::BlockWrite {
                                    pos: write.pos,
                                    block: write.block,
                                },
                            })
                            .collect();
                        if !intents.is_empty() {
                            match self.router.collect_intents(shard, &intents, at) {
                                Ok(()) => {}
                                Err(AdmitError::QueueFull { capacity }) => panic!(
                                    "涌现 block write 队列满(capacity {capacity})于拍 {}:\
                                     fail-stop,反压策略待接入(REQ-005 任务书 03)",
                                    at.0
                                ),
                                Err(e) => {
                                    panic!("BlockWrite 盖章被拒(编程错误,fail-stop):{e}")
                                }
                            }
                        }
                    }
                }
            }
            // 本拍实体生命周期:以分片实体表(真相)同步全局 ownership
            // (REQ-006 任务 00:spawn/migrate/remove/solidify/teleport 都覆盖)。
            self.sync_entity_ownership(shard, &sim);
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

        // 7.5 票据时钟(REQ-007 §4.3.2,任务 02):Active 票 +1;TTL 到期的
        //     区域票直接 End(键序;实体票内部指令 03 接)。激活未启用时无票,
        //     零动作。
        self.tickets.advance_clocks();
        for key in self.tickets.expired() {
            if matches!(key.0, physics_sim::tickets::TicketHolder::Area(_)) {
                self.tickets.end(key);
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

    /// 票判定(REQ-002 §4.5;REQ-007 任务 02 改读票据表):任一 Active/Paused
    /// 票 ① "出"区域与分片核心 AABB(8×8 chunk 立柱)相交 → 有票。
    /// Radius 票 = 锚点到 AABB 水平距离 ≤ unload_m(观察者票 unload = 48m,
    /// 与现状逐位一致);Suspended 不保活(§4.2.1)。
    fn shard_is_ticketed(&self, shard: ShardId) -> bool {
        if !self.activation_enabled() {
            return false;
        }
        // 半径回退:无 sim(全卸后)时取默认 48(与旧实现同口径)。
        let fallback = self
            .sims
            .values()
            .next()
            .map(|s| s.activation_config().data_unload_radius)
            .unwrap_or(48.0);
        let min = shard.pos.min_chunk();
        let min_x = (min.x * types::CHUNK_SIZE) as f32;
        let min_z = (min.z * types::CHUNK_SIZE) as f32;
        let max_x = min_x + (types::SHARD_CHUNKS * types::CHUNK_SIZE) as f32;
        let max_z = min_z + (types::SHARD_CHUNKS * types::CHUNK_SIZE) as f32;
        self.tickets.iter().any(|(holder, t)| {
            if t.state == physics_sim::tickets::TicketState::Suspended {
                return false;
            }
            let physics_sim::tickets::RegionSpec::Radius { unload_m, .. } = t.data else {
                return false; // None/Chunks(03)不保活;Footprint 03 接
            };
            let Some(anchor) = self.ticket_anchor_of(&holder.0).or(t.center) else {
                return false; // 无锚票(03 语义);保活忽略
            };
            let radius = if matches!(holder.0, physics_sim::tickets::TicketHolder::Observer(_)) {
                fallback // 观察者票半径与 ① 配置同源(现状口径)
            } else {
                unload_m
            };
            let radius_sq = radius * radius;
            let dx = (anchor.x.clamp(min_x, max_x) - anchor.x).abs();
            let dz = (anchor.z.clamp(min_z, max_z) - anchor.z).abs();
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

    /// 调度器快照(REQ-005 任务书 02;**只在两拍之间调用**,同
    /// [`Scheduler::snapshot_shard`]):tick 取 coordinator(= 下一待执行拍,
    /// 与 GlobalSnapshot.tick 同源)、观察者表直出、Router 全量、lifecycles
    /// 只存当前 `sims` 键集(unload_shard 留下的 `Stopped` 观测残留不入——
    /// 恢复线按已注册分片重建 Active/恢复记录)。
    pub fn snapshot_state(&self) -> SchedulerSnapshot {
        SchedulerSnapshot {
            tick: self.coordinator.tick(),
            observers: self.observer_table.clone(),
            observers_ever_set: self.observers_ever_set,
            tickets: self.tickets.clone(),
            tickets_ever_set: self.tickets_ever_set,
            router: self.router.snapshot(),
            lifecycles: self
                .lifecycles
                .iter()
                .filter(|(shard, _)| self.sims.contains_key(shard))
                .map(|(shard, lc)| (*shard, lc.snapshot()))
                .collect(),
        }
    }

    /// 从快照恢复调度器状态(REQ-005 任务书 02;调用前提:分片已全部
    /// 注册、coordinator 已对齐 `start_tick = snap.tick`)。
    ///
    /// 校验(违反 = **内部状态**不一致 → panic fail-stop,细则 6):
    /// - `snap.tick == coordinator.tick()`(快照拍对齐);
    /// - lifecycles 键集 == 当前 `sims` 键集(少/多都 = 恢复装配错误);
    /// - Router 信封目的地必须在当前已注册分片内(允许先沉队再 ensure 的
    ///   分片**不在**此列——恢复线上驱动 ensure 只发生在日志回放后的编排;
    ///   恢复时刻目的地未注册 = 拍 N 末已产生信封而分片不在快照,即快照
    ///   不完整,内部状态损坏)。
    ///
    /// 恢复顺序:观察者表整表替换(含 `observers_ever_set`)→ Router
    /// (envelopes 经队列不变量校验放回;sequences 原样——不恢复会改变
    /// 后续迁移的 canonical 序)→ lifecycles 逐片 restore(cfg 用当前配置)
    /// → 从各 sim 已加载 chunk 重建 holder 表(表不入快照,REQ-005 D2)。
    pub fn restore_state(&mut self, snap: SchedulerSnapshot) {
        let now = self.coordinator.tick();
        if snap.tick != now {
            panic!(
                "调度器快照拍不一致:快照 tick {} ≠ 当前 coordinator tick {}(fail-stop)",
                snap.tick.0, now.0
            );
        }
        let snap_keys: BTreeSet<ShardId> = snap.lifecycles.keys().copied().collect();
        let sim_keys: BTreeSet<ShardId> = self.sims.keys().copied().collect();
        if snap_keys != sim_keys {
            panic!("调度器快照分片键集不一致:快照 {snap_keys:?} ≠ 已注册 {sim_keys:?}(fail-stop)");
        }
        for envelope in &snap.router.envelopes {
            if !self.sims.contains_key(&envelope.destination) {
                panic!(
                    "快照信封目的地 {dest:?} 未注册(快照时刻待投递信封的目的分片\
                     必须已在快照分片集内,内部状态损坏,fail-stop)",
                    dest = envelope.destination
                );
            }
        }
        // 观察者表整表替换(快照是 checkpoint 时刻兜底;日志是其后权威)。
        self.observer_table = snap.observers;
        self.observers_ever_set = snap.observers_ever_set;
        // 票据表整表替换(REQ-007 任务 02;区域票 Set/Remove 照常入日志回放,
        // 快照是兜底)。
        self.tickets = snap.tickets;
        self.tickets_ever_set = snap.tickets_ever_set;
        // Router:envelopes 校验 + 放桶,sequences 原样。
        self.router.restore(snap.router);
        // lifecycles:逐片恢复(cfg 用当前配置);键集已校验一致。
        for (shard, lc_snap) in snap.lifecycles {
            let cfg = LifecycleConfig {
                despawn_after_ticks: self.cfg.despawn_after_ticks,
            };
            self.lifecycles
                .insert(shard, ShardLifecycle::restore(lc_snap, cfg));
        }
        // holder 表从各分片已加载 chunk 重建(表不入快照)。
        self.rebuild_holders();
        // ownership 从各分片实体表重建并校验全局唯一(REQ-006 任务 00)。
        self.rebuild_entity_ownership();
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

    // ---- REQ-007 任务 01(D17 激活启用开关;任务书测试
    // `activation_enabled_flag_semantics`,语义载体 = 调度器) ----

    /// 未设观察者 → false;设置一次 → 永久 true(单向,remove_observer 不回退);
    /// 快照往返保持(`observers_ever_set` 沿用原快照位置)。另锚定装载分档:
    /// 未启用时合一(①② 同建),启用后装载只 ①、② 待激活决策。
    #[test]
    fn activation_enabled_flag_semantics() {
        use types::glam::Vec3;
        // 实心 chunk(有碰撞盒:has_collider 与"生成了碰撞盒"在 D18 独立集
        // 语义下解耦,实心者两侧都真,判别更直观;空气 chunk 现在也算已建,
        // 见 physics-sim air_chunk_counts_as_built_collider)。
        let solid = {
            let mut ch = voxel::Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    ch.set(x, 64, z, types::BlockId(1));
                }
            }
            ch
        };
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        assert!(
            !sched.activation_enabled(),
            "未设观察者 → 未启用(D17:激活完全不生效)"
        );

        // 未启用装载 = 合一(①② 同建;§14-9 现状行为)。
        let cp = types::ChunkPos::new(0, 0);
        sched.apply_chunk_load(shard_id(0, 0), cp, solid.clone());
        assert!(
            sched.sims[&shard_id(0, 0)].has_collider(cp),
            "未启用:合一装载建 ②(行为与 8702670 一致)"
        );

        // 设置一次 → 永久启用;remove_observer 不回退(单向)。
        sched.submit_observer(observer_input(1, 0, Tick::ZERO, Vec3::new(8.0, 0.0, 8.0)));
        sched.tick();
        assert!(sched.activation_enabled(), "曾设观察者 → 启用");
        sched.remove_observer(InputSource(1));
        assert!(
            sched.activation_enabled(),
            "单向开关:移除观察者不回退(表空不清 observers_ever_set)"
        );

        // 启用后装载 = 只 ①(② 由激活决策构建,细则 11)。
        let cp2 = types::ChunkPos::new(1, 0);
        sched.apply_chunk_load(shard_id(0, 0), cp2, solid);
        let s = &sched.sims[&shard_id(0, 0)];
        assert!(!s.has_collider(cp2), "启用:装载只 ①,② 待激活决策");
        assert!(s.has_collider(cp), "旧 chunk 的 ② 不受影响");

        // 快照往返保持:开关 + 观察者表沿原快照位置走。
        let snap = sched.snapshot_state();
        let mut sched2 = Scheduler::new(SchedulerConfig {
            start_tick: snap.tick,
            ..sched_cfg(2)
        });
        sched2.register(sim(0, 0));
        sched2.restore_state(snap);
        assert!(
            sched2.activation_enabled(),
            "observers_ever_set 随 SchedulerSnapshot 往返"
        );
        sched.shutdown();
        sched2.shutdown();
    }

    /// 测试清单(shard,任务书 02):despawn 读票据表——有 Active/Paused 票
    /// (① 出区域与分片核心相交)的分片不进候选;票清 + 空 + idle → 候选出现。
    #[test]
    fn despawn_reads_ticket_table() {
        use types::glam::Vec3;
        let shard = shard_id(0, 0);
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        // 区域票(中心在分片核心内,unload = 1.5×32 = 48 > 0 → 保活)。
        sched.apply_area_ticket(1, Vec3::new(64.0, 0.0, 64.0), 32.0, 0b001, None);
        sched.tick();
        // 空片 + 有票 + idle 短:不进候选。
        assert!(
            !sched.take_despawn_requests().contains(&shard),
            "有 Active 区域票 → 不 despawn"
        );
        // 移除票 → 下一拍候选出现(idle_after_ticks 默认 ?——直接推进足够拍)。
        sched.apply_remove_area_ticket(1);
        let idle = SchedulerConfig::default().despawn_after_ticks;
        for _ in 0..=idle {
            sched.tick();
        }
        assert!(
            sched.take_despawn_requests().contains(&shard),
            "票清 + 空 + idle 满 → 候选出现"
        );
        sched.shutdown();
    }

    /// 测试清单(shard,任务书 02/T24):未设观察者、SetAreaTicket → 启用
    /// (激活开始生效);Remove 后仍启用(单向)。
    #[test]
    fn activation_enabled_by_area_ticket() {
        use types::glam::Vec3;
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        assert!(!sched.activation_enabled(), "未启用(无观察者无票)");
        sched.apply_area_ticket(1, Vec3::new(64.0, 0.0, 64.0), 32.0, 0b001, None);
        assert!(sched.activation_enabled(), "区域票 → 启用(D17 推广)");
        sched.apply_remove_area_ticket(1);
        assert!(sched.activation_enabled(), "单向:移除不回退");
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
                        actor: false,
                        source: None,
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
                        actor: false,
                        source: None,
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

    /// 任务书测试 3(REQ-005 任务 04 补强):两分片 + 一观察者贴边界——
    /// ① 各自激活请求只落自己的 core ∪ margin(D4);② **core 装载请求只
    /// 来自 owner(精确集合)**:被请求 chunk 的「把它当 core 的请求者」
    /// 集合恰 = {owner};非 owner 的请求必须落在请求者的 margin 环内
    /// (扩张副本语义,D4/D5 设计,不算重复本体);③ 请求集与手算期望
    /// 精确集合全等(观察者在两片边界 → x=7/x=8 相邻列双请求确实发生,
    /// 深处 core chunk 恰单请求——「重复」只允许出现在 margin 列)。
    ///
    /// 拍数口径:预算 2/拍 + 冷却 180 拍(决策即簿记)。每片 ≤32m 装载候选
    /// 各 6 个,2/拍 → 3 拍排空;400 拍覆盖 2 个冷却重火窗 → 每个区域够得着
    /// 且距离够近的 chunk 必被记录到(重复请求经集合去重)。
    #[test]
    fn activation_two_shards_no_cross_core_duplicates() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        // 观察者贴边界(x=126 → chunk (7,4)):两片的激活都跑,且两片装载
        // 候选区在 x=7/x=8 相邻列上重叠(测试的被测面)。
        sched.set_observers(vec![types::glam::Vec3::new(126.0, 0.0, 64.0)]);

        // 逐拍累计「chunk → 请求者集合」+ 区域不变量。
        let mut requesters: BTreeMap<ChunkPos, BTreeSet<ShardId>> = BTreeMap::new();
        let mut load_decisions = 0usize;
        for _ in 0..400 {
            sched.tick();
            for (shard, decisions) in sched.take_activation_requests() {
                let region = types::ChunkRegion::core(shard).expanded(1);
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
                load_decisions += decisions.chunks_to_load.len();
                for cp in decisions.chunks_to_load {
                    requesters.entry(cp).or_default().insert(shard);
                }
            }
        }

        let (s0, s1) = (shard_id(0, 0), shard_id(1, 0));
        let owner_of_chunk =
            |cp: ChunkPos| ShardId::new(types::WorldId(0), types::DimensionId(0), cp.shard_of());
        let both: BTreeSet<ShardId> = [s0, s1].into_iter().collect();

        // —— ② core 装载请求只来自 owner(逐 chunk 精确集合)——
        assert!(load_decisions > 0, "前置:激活装载请求确实发生");
        for (cp, reqs) in &requesters {
            let owner = owner_of_chunk(*cp);
            // 把 cp 当自家 core chunk 装片的请求者,恰 = {owner}:
            // 没有任何分片把别人的 core chunk 当本体装。
            let core_requesters: BTreeSet<ShardId> = reqs
                .iter()
                .filter(|y| types::ChunkRegion::core(**y).contains(*cp))
                .copied()
                .collect();
            assert_eq!(
                core_requesters,
                BTreeSet::from([owner]),
                "chunk {cp:?} 的 core 装载请求必须恰来自 owner {owner:?},实得 {core_requesters:?}\
                 (请求者全集 {reqs:?})"
            );
            // 非 owner 的请求 = margin 环扩张副本(设计语义),逐请求者验证
            // 其确在「请求者区域 ∖ 请求者 core」内。
            for y in reqs {
                if *y != owner {
                    assert!(
                        types::ChunkRegion::core(*y).expanded(1).contains(*cp),
                        "扩张副本请求 {y:?}←{cp:?} 越出请求者 margin 区"
                    );
                }
            }
        }

        // —— ③ 精确集合全等:观察者 (126,0,64) 手算的 12 个够近 chunk
        //   (片 0:x=6/7 列 × z∈{2..5} 中距 ≤32 者;片 1:x=8/9 列同理)。
        //   距观察者水平距:chunk 中心 (16x+8, 16z+8)。
        let mut expected: BTreeMap<ChunkPos, BTreeSet<ShardId>> = BTreeMap::new();
        // x=7 列(片 0 core ∩ 片 1 margin ring,z∈2..=5 距 ≤32):双请求。
        for z in 2..=5 {
            expected.insert(ChunkPos::new(7, z), both.clone());
        }
        // x=8 列(片 1 core ∩ 片 0 margin ring,z∈2..=5 距 ≤32):双请求。
        for z in 2..=5 {
            expected.insert(ChunkPos::new(8, z), both.clone());
        }
        // x=6 列(片 0 core 深处,片 1 区域外):恰 owner 单请求。
        for z in [3, 4] {
            expected.insert(ChunkPos::new(6, z), [s0].into_iter().collect());
        }
        // x=9 列(片 1 core 深处,片 0 区域外):恰 owner 单请求。
        for z in [3, 4] {
            expected.insert(ChunkPos::new(9, z), [s1].into_iter().collect());
        }
        assert_eq!(
            requesters, expected,
            "请求集必须与手算精确集合全等:双请求只出现在 x=7/8 margin 重叠列,\
             x=6/9 深处 core chunk 恰单请求(实测 {requesters:?})"
        );
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

    // ---- REQ-005 任务书 02:全局快照(SchedulerSnapshot)+ 观察者移除命令 ----

    /// 任务书测试 `scheduler_snapshot_roundtrip`:观察者/信封/sequences/
    /// lifecycle 快照→恢复逐字段相等;恢复线续跑时 pending 信封在 apply_at 拍
    /// 照常投递(数据面 + 功能面)。
    #[test]
    fn scheduler_snapshot_roundtrip() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        // 观察者直通落表(设置过语义)。
        sched.set_observers(vec![types::glam::Vec3::new(8.0, 0.0, 8.0)]);
        sched.tick(); // 拍 0
        // 拍 0 末:两个来源各产一封迁移信封(apply_at = 1,下一拍)。
        // 出生位姿必须落在目的片核心区(同 dispatched_destination_can_despawn_
        // without_false_blackhole 的坑:否则目的片当拍迁移检测又把它送走)。
        let migrate_to = |dest| messaging::Intent {
            destination: dest,
            payload: messaging::EnvelopePayload::MigrateEntity {
                // REQ-006 任务 00:同拍两个目的片不能重建同一个 EntityId
                // (全局唯一);用两个不同 ID 各测一条信封。
                entity: if dest == shard_id(1, 0) {
                    types::EntityId::from_parts(0, 1)
                } else {
                    types::EntityId::from_parts(0, 2)
                },
                state: types::EntityState {
                    translation: if dest == shard_id(1, 0) {
                        types::glam::Vec3::new(140.0, 10.0, 8.0)
                    } else {
                        types::glam::Vec3::new(8.0, 10.0, 8.0)
                    },
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
                actor: false,
                source: None,
            },
        };
        sched
            .router
            .collect_intents(shard_id(0, 0), &[migrate_to(shard_id(1, 0))], Tick::ZERO)
            .expect("队列未满");
        sched
            .router
            .collect_intents(shard_id(1, 0), &[migrate_to(shard_id(0, 0))], Tick::ZERO)
            .expect("队列未满");

        let snap = sched.snapshot_state();
        assert_eq!(snap.tick, Tick(1), "tick = coordinator(tick_now)");
        assert_eq!(
            snap.observers,
            std::iter::once((
                InputSource::SCHEDULER,
                types::glam::Vec3::new(8.0, 0.0, 8.0)
            ))
            .collect(),
            "观察者表直通条目入快照"
        );
        assert!(snap.observers_ever_set);
        assert_eq!(snap.router.envelopes.len(), 2, "两封在队信封入快照");
        assert_eq!(
            snap.router.sequences,
            [(shard_id(0, 0), 1u64), (shard_id(1, 0), 1u64)]
                .into_iter()
                .collect(),
            "per-source sequence 原样入快照"
        );
        assert_eq!(snap.lifecycles.len(), 2, "lifecycles 只存当前 sims 键集");
        assert!(
            snap.lifecycles
                .values()
                .all(|lc| lc.stage == crate::lifecycle::Stage::Active && !lc.drained),
            "已注册分片 = Active 未排空"
        );
        sched.shutdown();

        // 恢复:同 start_tick、同键集空片;先设干扰观察者验证整表替换。
        let mut restored = Scheduler::new(SchedulerConfig {
            start_tick: snap.tick,
            ..sched_cfg(2)
        });
        restored.register(sim(0, 0));
        restored.register(sim(1, 0));
        restored.set_observers(vec![types::glam::Vec3::new(999.0, 999.0, 999.0)]);
        restored.restore_state(snap.clone());
        assert_eq!(
            restored.snapshot_state(),
            snap,
            "恢复后再快照 = 原快照(逐字段相等)"
        );

        // 功能面:pending 信封在 apply_at(拍 1)照常投递——片 (1,0) 重建实体。
        restored.tick();
        assert!(
            restored
                .entity_state(shard_id(1, 0), types::EntityId::from_parts(0, 1))
                .is_some(),
            "恢复线拍 1:迁移信封照常投递(队列 + sequences 恢复有效)"
        );
        // sequences 原样恢复:同一来源再产信封序号从 1 续(不归零)。
        restored
            .router
            .collect_intents(shard_id(0, 0), &[migrate_to(shard_id(1, 0))], Tick(1))
            .expect("队列未满");
        let again = restored.snapshot_state();
        assert_eq!(
            again.router.sequences.get(&shard_id(0, 0)),
            Some(&2),
            "恢复后 sequence 从恢复值续计(不归零,canonical 序不变)"
        );
        restored.shutdown();
    }

    /// 任务书测试 `checkpoint_restores_lifecycle_timers`:空闲计时跨 checkpoint
    /// 不重置——原线与恢复线的 despawn 候选拍一致(lifecycle.last_activity
    /// 恢复;若被 register 的 activate 重置,恢复线候选会晚 3 拍)。
    #[test]
    fn checkpoint_restores_lifecycle_timers() {
        let mk = |start_tick| SchedulerConfig {
            despawn_after_ticks: 5,
            start_tick,
            ..sched_cfg(2)
        };
        // 原线:拍 0 注册(activate at=0),空片无输入 → last_activity 停在 0;
        // 候选在拍 5(at - 0 >= 5)。
        let mut orig = Scheduler::new(mk(Tick::ZERO));
        orig.register(sim(0, 0));
        for _ in 0..3 {
            orig.tick();
        }
        let snap = orig.snapshot_state();
        assert_eq!(snap.tick, Tick(3));
        orig.tick(); // 拍 3
        orig.tick(); // 拍 4
        orig.tick(); // 拍 5:候选出现
        assert_eq!(
            orig.take_despawn_requests(),
            vec![shard_id(0, 0)],
            "原线拍 5 出候选(空闲计时基准 = 注册拍 0)"
        );
        orig.shutdown();

        // 恢复线:快照(拍 3 之前)→ restore → 跑到拍 5,候选拍不变。
        let mut restored = Scheduler::new(mk(snap.tick));
        restored.register(sim(0, 0));
        restored.restore_state(snap);
        for _ in 0..3 {
            restored.tick();
        }
        assert_eq!(
            restored.take_despawn_requests(),
            vec![shard_id(0, 0)],
            "恢复线候选仍在拍 5(last_activity 跨 checkpoint 保留;重置则拍 8)"
        );
        restored.shutdown();
    }

    /// REQ-005 D9:观察者收件箱的 RemoveObserver 分支——apply_at 到期后删除
    /// observer_table 的 source(票撤);不存在 = 幂等不 panic;SetObserverPosition
    /// 保持原语义。
    #[test]
    fn observer_remove_command_deletes_entry() {
        use types::glam::Vec3;

        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.submit_observer(observer_input(1, 0, Tick::ZERO, Vec3::new(8.0, 0.0, 8.0)));
        // 同拍再投 RemoveObserver(source 1):同拍两输入按 canonical 序
        // (source 同 → sequence 0 < 1)先设置后移除 → 表空。
        sched.submit_observer(StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource(1),
            sequence: 1,
            payload: InputPayload::Command(SimCommand::RemoveObserver),
        });
        sched.tick();
        assert!(
            sched.observers().is_empty(),
            "同拍 Set 后 Remove:条目被删除(票撤)"
        );
        // 幂等:再投一条 RemoveObserver(不在表)→ 不 panic,表仍空。
        sched.submit_observer(StampedInput {
            apply_at: Tick(1),
            source: InputSource(1),
            sequence: 2,
            payload: InputPayload::Command(SimCommand::RemoveObserver),
        });
        sched.tick();
        assert!(sched.observers().is_empty(), "幂等:移除不在表条目无害");

        // SetObserverPosition 语义保持:再设置 → 入表。
        sched.submit_observer(observer_input(2, 0, Tick(2), Vec3::new(40.0, 0.0, 40.0)));
        sched.tick();
        assert_eq!(
            sched.observers(),
            vec![(InputSource(2), Vec3::new(40.0, 0.0, 40.0))],
            "SetObserverPosition 原语义保持"
        );
        sched.shutdown();
    }

    // ---- 任务书 req005-consistency/03:跨片事件广播与涌现 block write ----

    /// Detonate 事件构造(center/radius 可控)。
    fn blast(center: Vec3, power: f32, radius: f32) -> CrossEvent {
        CrossEvent::Detonate {
            center,
            power,
            radius,
        }
    }

    /// 边界爆心(两片 core 交界 x=128 上;两片 core AABB 距爆心 0)。
    const BOUNDARY: Vec3 = Vec3::new(128.0, 3.0, 64.0);

    /// Emergent 写入的涌现路径驱动(测试用):给分片塞一个真静止实体
    /// (零重力 + 位移门 0 = 直通凝固),走真实 `apply_prefix → step →
    /// take_outbox` 产出 Emergent——凝固在 20 拍内必然发生(P2 计数达标)。
    /// `entity` = 实体身份;返回前把实体塞进指定分片的 sim(仅测试模块,
    /// 借 `Scheduler` 的 sims 直通——shard crate 测试与 sim 同依赖可构造,
    /// 无新增公开面)。
    fn spawn_resting_entity(sched: &mut Scheduler, shard: ShardId, entity: EntityId, pos: Vec3) {
        let mut sim = {
            let mut cfg = physics_sim::ShardSimConfig::new(shard);
            cfg.gravity = Vec3::ZERO;
            cfg.solidify.displacement_gate_m = 0.0;
            ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
        };
        sim.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity,
                state: types::EntityState {
                    translation: pos,
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);
        // 顶替注册表中的空壳 sim(保留其 lifecycle/ownership 登记)。
        sched.sims.insert(shard, sim);
    }

    /// 跑到 producer 分片产出 Emergent 写入(≤40 拍;P2 计数 20 拍达标)。
    fn run_until_emergent(sched: &mut Scheduler, shard: ShardId, entity: EntityId) {
        for _ in 0..40 {
            sched.tick();
            if sched.entity_state(shard, entity).is_none() {
                return; // 已凝固(实体移除 = Emergent 已产出并被分流)
            }
        }
        panic!("40 拍内未凝固(场景退化)");
    }

    /// 任务书测试 `detonate_targets_all_covering_shards`:边界爆心两片都在
    /// `shards_covering`(core AABB 相交);`broadcast_event` 后两片各自在
    /// N+1 拍收到恰一封 Event 信封(origin 排除语义由 origin 自身不在目标集
    /// 承担——本例 origin = 片 (0,0),它也在覆盖集但只作为 source 不收信)。
    #[test]
    fn detonate_targets_all_covering_shards() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let covering = sched.shards_covering(BOUNDARY, 1.0);
        assert_eq!(
            covering,
            std::collections::BTreeSet::from([shard_id(0, 0), shard_id(1, 0)]),
            "边界爆心:两片 core AABB 都与事件 AABB 相交"
        );
        sched
            .broadcast_event(shard_id(0, 0), blast(BOUNDARY, 6.0, 1.0), Tick(0))
            .expect("队列未满");
        // 拍 0:apply_at(1) 未到;拍 1:目标 (1,0) 恰收一封(origin (0,0)
        // 已在 step 内本地求值,不给自分片发——信封面;命令复制面每片一条
        // LogEntry 由 sim-api detonate_from 覆盖)。
        sched.tick();
        let batch = sched.router.dispatch_prefix(shard_id(1, 0), Tick(1));
        assert_eq!(batch.len(), 1, "目标 (1,0) 恰一封信封");
        let InputPayload::Envelope(envelope) = &batch[0].payload else {
            panic!("应为信封载荷,实为 {:?}", batch[0].payload)
        };
        assert!(
            matches!(
                envelope.payload,
                messaging::EnvelopePayload::Event(CrossEvent::Detonate { .. })
            ),
            "收到的是 Event(Detonate) 信封"
        );
        assert_eq!(envelope.apply_at, Tick(1), "apply_at = N+1");
        assert_eq!(envelope.source, shard_id(0, 0), "source = origin");
        assert_eq!(batch[0].sequence, 0, "dispatch 侧 sequence = 信封序号");
        // origin 自身同拍无信封;恰好一次:再取为空。
        assert!(
            sched
                .router
                .dispatch_prefix(shard_id(0, 0), Tick(1))
                .is_empty(),
            "origin 不收自己广播的事件"
        );
        assert!(
            sched
                .router
                .dispatch_prefix(shard_id(1, 0), Tick(1))
                .is_empty(),
            "take 后桶清(恰好一次)"
        );
        sched.shutdown();
    }

    /// 任务书测试 `detonate_excludes_far_shards`:远离爆心的已注册分片不在
    /// 目标集(core AABB 与事件 AABB 不相交、也不持有爆心附近 chunk)。
    #[test]
    fn detonate_excludes_far_shards() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        sched.register(sim(9, 9));
        let covering = sched.shards_covering(BOUNDARY, 30.0);
        assert!(
            covering.contains(&shard_id(0, 0)) && covering.contains(&shard_id(1, 0)),
            "两边界片在目标集:{covering:?}"
        );
        assert!(
            !covering.contains(&shard_id(9, 9)),
            "远片 (9,9)(core x≥1152)不在目标集:{covering:?}"
        );
        sched.shutdown();
    }

    /// 任务书测试 `detonate_center_in_empty_shard_hits_registered_neighbor`:
    /// 爆心所在分片未注册,邻片已注册且持有爆心附近的扩张 chunk(holder 表
    /// 命中)→ 目标集含邻片;不要求中心分片存在。几何:爆心 (150,·,8) 在
    /// 未注册片 (1,0) core 深处;事件 AABB x∈[130,170] 与邻片 (0,0) 持有的
    /// chunk (8,0)(AABB x∈[128,144])相交 → 邻片经 holder 命中(core AABB
    /// x∈[0,128) 与事件 AABB 不相交,命中只能来自持有)。
    #[test]
    fn detonate_center_in_empty_shard_hits_registered_neighbor() {
        let mut sched = Scheduler::new(sched_cfg(2));
        // 只注册片 (0,0);爆心落在未注册片 (1,0) 的 core 深处。
        // 片 (0,0) 持有 chunk (8,0)(margin=2 才在其激活区域内;装载校验
        // 按 activation_region 判,owner 片 (1,0) 未注册会被拦——装载前置
        // 要求 owner 在位。绕开:注册 owner 片 (1,0) 后装载,再不参与判定
        // ——不行,注册即在 sims 里。改用负向:爆心 (-150,·,8) 在未注册片
        // (-1,0) core;邻片 (0,0) 注册且持 chunk (-1,0)(margin=1 即覆盖,
        // owner = 片 (-1,0) 未注册仍会被 apply_chunk_load 拦)。
        // 结论:装载前置要求 owner 在位,故"邻片持未注册 owner 的 chunk"
        // 只能构造在 sims 直塞层(与装配期先 holder 后 owner 同型)。
        let mut sim0 = sim(0, 0);
        sim0.load_chunk(types::ChunkPos::new(8, 0), voxel::Chunk::air());
        sched.register(sim0); // register 末尾 rebuild_holders 登记持有
        let covering = sched.shards_covering(Vec3::new(150.0, 3.0, 8.0), 20.0);
        assert!(
            !covering.contains(&shard_id(1, 0)),
            "未注册分片永不在目标集(遍历 sims):{covering:?}"
        );
        assert_eq!(
            covering,
            std::collections::BTreeSet::from([shard_id(0, 0)]),
            "注册邻片经持有 chunk (8,0) 命中(core 不相交,holder 命中):{covering:?}"
        );
        sched.shutdown();
    }

    /// 任务书测试 `emergent_event_envelope_next_tick`(pure broadcast 面):
    /// `broadcast_event` 后目标在 N+1 收到 Event;origin 不重复(同片场景见
    /// `detonate_targets_all_covering_shards`,此处 origin 也在覆盖集)。
    #[test]
    fn emergent_event_envelope_next_tick() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        // origin = (0,0);事件中心也落在 (0,0) 的 core 深处(两片都覆盖:
        // 半径 80 跨过 x=128 边界)。
        let center = Vec3::new(64.0, 3.0, 64.0);
        sched
            .broadcast_event(shard_id(0, 0), blast(center, 6.0, 80.0), Tick(0))
            .expect("队列未满");
        assert_eq!(
            sched.router.pending_envelopes(),
            1,
            "origin 自己不发:只有远片 (1,0) 一封"
        );
        let batch = sched.router.dispatch_prefix(shard_id(1, 0), Tick(1));
        assert_eq!(batch.len(), 1, "目标 N+1 收到恰一封");
        assert_eq!(batch[0].apply_at, Tick(1));
        // origin 自身同拍无信封。
        assert!(
            sched
                .router
                .dispatch_prefix(shard_id(0, 0), Tick(1))
                .is_empty(),
            "origin 不收自己广播的事件"
        );
        sched.shutdown();
    }

    /// 任务书测试 `emergent_event_canonical_and_once`:同拍多事件 → 同目标
    /// 桶内按 (payload ordinal, source, source_sequence) canonical 序;
    /// 每目标恰一次(不因多事件重复投同封)。
    #[test]
    fn emergent_event_canonical_and_once() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let dest = shard_id(1, 0);
        // 同一 origin 连发两事件(同 apply_at):per-source sequence 0、1。
        sched
            .broadcast_event(
                shard_id(0, 0),
                blast(Vec3::new(64.0, 3.0, 64.0), 6.0, 80.0),
                Tick(0),
            )
            .expect("第一发入队");
        sched
            .broadcast_event(
                shard_id(0, 0),
                blast(Vec3::new(60.0, 3.0, 64.0), 6.0, 80.0),
                Tick(0),
            )
            .expect("第二发入队");
        // 另一 origin 也发一封到同目标:source 序在 (0,0) 之后。
        sched
            .broadcast_event(
                shard_id(0, 1),
                blast(Vec3::new(64.0, 3.0, 64.0), 6.0, 80.0),
                Tick(0),
            )
            .expect("第三发入队");
        let batch = sched.router.dispatch_prefix(dest, Tick(1));
        assert_eq!(batch.len(), 3, "同拍三封(每目标不去重事件,只去重信封)");
        // canonical 序:source (0,0) 的 seq 0 → (0,0) 的 seq 1 → (0,1) 的 seq 0
        // (dispatch 侧 sequence 字段 = 信封 source_sequence)。
        // 序列语义:collect_intents 按 targets 升序盖章——第三发(origin
        // (0,1))的目标 [(0,0), (1,0)] 中 (0,1) 自己排第一,故发往 (1,0)
        // 的那封是该源的 seq 1(它给 (0,0) 的 seq 0 在另一个桶里)。
        assert_eq!(batch[0].sequence, 0, "同源首事件 seq 0");
        assert_eq!(batch[1].sequence, 1, "同源次事件 seq 1");
        assert_eq!(batch[2].sequence, 1, "origin (0,1) 的第二目标 seq 1");
        let src = |i: usize| match &batch[i].payload {
            InputPayload::Envelope(e) => e.source,
            other => panic!("应为信封载荷,实为 {other:?}"),
        };
        assert_eq!(src(0), shard_id(0, 0));
        assert_eq!(src(1), shard_id(0, 0));
        assert_eq!(src(2), shard_id(0, 1));
        // 恰好一次。
        assert!(sched.router.dispatch_prefix(dest, Tick(1)).is_empty());
        sched.shutdown();
    }

    /// 任务书测试 `event_queue_full_is_typed`:队列满 → `Err(AdmitError::
    /// QueueFull)` 类型化返回,不 panic 不静默丢。
    #[test]
    fn event_queue_full_is_typed() {
        let mut sched = Scheduler::new(SchedulerConfig {
            router: crate::router::RouterConfig {
                queue_capacity: 1,
                ..crate::router::RouterConfig::default()
            },
            ..sched_cfg(2)
        });
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let event = blast(Vec3::new(64.0, 3.0, 64.0), 6.0, 80.0);
        sched
            .broadcast_event(shard_id(0, 0), event, Tick(0))
            .expect("第一发入队(容量 1)");
        // 目标只余 (1,0)(origin 排除后恰一封 → 已满)。
        assert_eq!(
            sched.broadcast_event(shard_id(0, 1), event, Tick(0)),
            Err(AdmitError::QueueFull { capacity: 1 }),
            "队列满 → 类型化 Err(不静默丢不 panic)"
        );
        sched.shutdown();
    }

    /// 任务书测试 `detonate_logs_one_entry_per_shard` 的 shard 级前置(信封
    /// 侧):broadcast_event 的多目标信封 per-source sequence 连续、apply_at
    /// 同拍;LogEntry 面由 sim-api `detonate_logs_one_entry_per_shard` 覆盖。
    #[test]
    fn broadcast_event_stamps_monotonic_sequences() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        sched.register(sim(2, 0));
        // 事件覆盖三片(x∈[0,384) 全在半径内):origin (0,0) 发两事件。
        let big = blast(Vec3::new(192.0, 3.0, 64.0), 6.0, 400.0);
        sched
            .broadcast_event(shard_id(0, 0), big, Tick(0))
            .expect("第一发");
        sched
            .broadcast_event(shard_id(0, 0), big, Tick(1))
            .expect("第二发");
        // 拍 1 与拍 2 的目标桶:两目标各两封,sequence 依 (事件, 目标) 顺序
        // 单调 collect_intents 盖章(事件序主导,同一事件内按目标序)。
        let first = sched.router.dispatch_prefix(shard_id(1, 0), Tick(1));
        assert_eq!(first.len(), 1);
        assert_eq!(first[0].sequence, 0, "首事件→首目标 seq 0");
        let at2 = sched.router.dispatch_prefix(shard_id(1, 0), Tick(2));
        assert_eq!(at2.len(), 1);
        assert_eq!(at2[0].sequence, 2, "次事件→同目标 seq 续 2");
        let other = sched.router.dispatch_prefix(shard_id(2, 0), Tick(1));
        assert_eq!(other[0].sequence, 1, "首事件→次目标 seq 1");
        sched.shutdown();
    }

    /// 任务书测试 `blockwrite_envelope_reaches_owner_and_holders`(shard 级,
    /// 调度器分流面):producer(片 (1,0))上实体凝固(真路径:零重力 +
    /// 位移门 0 → 首拍睡眠兜底凝固),凝固格落在其本体 chunk (8,0);
    /// 邻片 (0,0) 持扩张副本 → ①发布集含 producer 条目、②solidified 兼容
    /// 字段同步、③producer 不发自己、holder (0,0) 下一拍应用 BlockWrite
    /// 信封(副本一致)。
    #[test]
    fn emergent_blockwrite_envelope_reaches_owner_and_holders() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        // chunk (8,0):owner = 片 (1,0)= producer;片 (0,0) 持扩张副本。
        let cp = types::ChunkPos::new(8, 0);
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        sched.apply_chunk_load(shard_id(1, 0), cp, voxel::Chunk::air());
        // producer 侧:真静止实体(130.5 ∈ 片 (1,0) core [128,256),chunk
        // (8,0) 已装载 = holder 表有登记,owner 不变量成立)。
        let e = types::EntityId::from_parts(1, 1);
        spawn_resting_entity(&mut sched, shard_id(1, 0), e, Vec3::new(130.5, 5.5, 8.5));
        run_until_emergent(&mut sched, shard_id(1, 0), e);
        // ① 发布集:producer(1,0) 的条目(网络在产生拍可见)。
        let updates = sched.take_block_updates();
        assert_eq!(
            updates,
            vec![(shard_id(1, 0), BlockPos::new(130, 5, 8), types::BlockId(1))],
            "Emergent → 发布产生者条目:{updates:?}"
        );
        // ② 兼容字段同步(take_solidified 语义不变)。
        let solid = sched.take_solidified();
        assert_eq!(
            solid,
            vec![(shard_id(1, 0), BlockPos::new(130, 5, 8), types::BlockId(1))],
            "Emergent → solidified 兼容字段同步"
        );
        // ③ holder (0,0) 在下一拍应用 BlockWrite 信封(副本一致);
        //    producer (1,0) 不收(已本地落格)。
        sched.tick();
        let owner_chunk = sched.voxel_chunk(shard_id(1, 0), cp).expect("已装载");
        assert_eq!(
            owner_chunk.get(2, 69, 8),
            types::BlockId(1),
            "producer 副本本地已写(凝固路径落格,局部 (2,69,8) = 世界 (130,5,8))"
        );
        let holder_chunk = sched.voxel_chunk(shard_id(0, 0), cp).expect("已装载");
        assert_eq!(
            holder_chunk.get(2, 69, 8),
            types::BlockId(1),
            "holder 副本经信封下一拍一致"
        );
        sched.shutdown();
    }

    /// 任务书测试 `blockwrite_does_not_rebroadcast`(调度器面):owner 应用
    /// BlockWrite 信封后**不产新信封**(应用路径不推 outbox——无回环)。
    #[test]
    fn emergent_blockwrite_no_rebroadcast_loop() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        // chunk (8,0):owner = 片 (1,0)= producer;片 (0,0) 持扩张副本。
        let cp = types::ChunkPos::new(8, 0);
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        sched.apply_chunk_load(shard_id(1, 0), cp, voxel::Chunk::air());
        let e = types::EntityId::from_parts(1, 1);
        spawn_resting_entity(&mut sched, shard_id(1, 0), e, Vec3::new(130.5, 5.5, 8.5));
        run_until_emergent(&mut sched, shard_id(1, 0), e); // 拍 k:分流,信封待投
        assert_eq!(sched.pending_shard_destinations().len(), 1, "一封待投");
        let _ = sched.take_block_updates();
        let _ = sched.take_solidified();
        sched.tick(); // 拍 k+1:owner 应用信封
        assert!(
            sched.pending_shard_destinations().is_empty(),
            "应用后零新增信封(无回环)"
        );
        // 再走一拍仍干净(环已结构性关闭)。
        sched.tick();
        assert!(sched.pending_shard_destinations().is_empty());
        sched.shutdown();
    }

    /// 终审修复(REQ-005 任务 03):`pending_blockwrite_cells` 是驱动卸载
    /// 保序的依据——信封 stamp 后(凝固拍末)命中接收方的目标 chunk,
    /// 信封应用后查询自然清空。时序竞态(接收方在 N 与 N+1 之间卸载目标
    /// chunk)由驱动延迟卸载消解,不再触发 dispatch 侧装载断言 fail-stop。
    #[test]
    fn pending_blockwrite_cells_defer_window() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        // chunk (8,0):owner = 片 (1,0) = producer;片 (0,0) 持扩张副本
        // → 信封只发 holder (0,0)(producer==owner 不发自己)。
        let cp = types::ChunkPos::new(8, 0);
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        sched.apply_chunk_load(shard_id(1, 0), cp, voxel::Chunk::air());
        let e = types::EntityId::from_parts(1, 1);
        spawn_resting_entity(&mut sched, shard_id(1, 0), e, Vec3::new(130.5, 5.5, 8.5));
        run_until_emergent(&mut sched, shard_id(1, 0), e); // 拍 k:信封待投
        let mut expected = std::collections::BTreeMap::new();
        expected.insert(shard_id(0, 0), std::collections::BTreeSet::from([cp]));
        assert_eq!(
            sched.pending_blockwrite_cells(),
            expected,
            "stamp 后查询命中 holder 的目标 chunk(卸载保序窗口内)"
        );
        // 发布集/兼容字段取走不影响查询(查询面只看信封队列)。
        let _ = sched.take_block_updates();
        let _ = sched.take_solidified();
        assert_eq!(sched.pending_blockwrite_cells(), expected);
        sched.tick(); // 拍 k+1:信封应用
        assert!(
            sched.pending_blockwrite_cells().is_empty(),
            "应用后查询清空(下一轮编排可正常卸载)"
        );
        sched.shutdown();
    }

    /// producer == owner 的 Emergent 写入:信封只发 holders − {owner}
    /// (owner 本地已落格,不发自己);发布集/兼容字段照常。
    #[test]
    fn emergent_blockwrite_producer_equals_owner_skips_owner_envelope() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        // chunk (7,0):owner = 片 (0,0) = producer;片 (1,0) 持扩张副本。
        let cp = types::ChunkPos::new(7, 0);
        sched.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        sched.apply_chunk_load(shard_id(1, 0), cp, voxel::Chunk::air());
        let e = types::EntityId::from_parts(1, 1);
        spawn_resting_entity(&mut sched, shard_id(0, 0), e, Vec3::new(120.5, 5.5, 8.5));
        run_until_emergent(&mut sched, shard_id(0, 0), e); // 拍 k:信封只发 (1,0)
        let _ = sched.take_block_updates();
        let _ = sched.take_solidified();
        assert_eq!(
            sched.pending_shard_destinations(),
            std::collections::BTreeSet::from([shard_id(1, 0)]),
            "producer == owner:信封只发其余 holder"
        );
        sched.tick();
        let holder_chunk = sched.voxel_chunk(shard_id(1, 0), cp).expect("已装载");
        assert_eq!(
            holder_chunk.get(8, 69, 8),
            types::BlockId(1),
            "holder 副本经信封一致"
        );
        sched.shutdown();
    }

    /// owner 未持有凝固格所在 chunk → panic(任务 00 ensure 保证的不变量;
    /// fail-stop 面)。实体的凝固格落在 chunk (7,0),但两片都未装载它
    /// (holder 表无条目)→ 分流时 fail-stop。
    #[test]
    #[should_panic(expected = "未持有 chunk")]
    fn emergent_blockwrite_owner_not_holder_panics() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        // 任何 chunk 都不装载:实体在虚空凝固(凝固写入不需要 chunk 预装,
        // set_block 自动建),chunk (7,0) 的 owner = 片 (0,0) 但 holder 表
        // 无登记 → 分流 fail-stop。
        let e = types::EntityId::from_parts(1, 1);
        spawn_resting_entity(&mut sched, shard_id(0, 0), e, Vec3::new(120.5, 5.5, 8.5));
        run_until_emergent(&mut sched, shard_id(0, 0), e);
    }

    // ---- REQ-006 任务 00:全局实体 ownership ----

    fn cmd_input(
        source: InputSource,
        sequence: u64,
        apply_at: Tick,
        cmd: SimCommand,
    ) -> StampedInput {
        StampedInput {
            apply_at,
            source,
            sequence,
            payload: InputPayload::Command(cmd),
        }
    }

    fn spawn_cmd(entity: EntityId, x: f32, z: f32) -> SimCommand {
        SimCommand::SpawnDynamic {
            entity,
            state: types::EntityState {
                translation: Vec3::new(x, 10.0, z),
                ..types::EntityState::default()
            },
            desc: types::EntityDesc::default(),
        }
    }

    #[test]
    fn ownership_claims_on_spawn_releases_on_remove() {
        let mut sched = Scheduler::new(sched_cfg(1));
        sched.register(sim(0, 0));
        let e = types::EntityId::from_parts(1, 1);
        sched
            .submit(
                shard_id(0, 0),
                cmd_input(InputSource(7), 1, Tick::ZERO, spawn_cmd(e, 8.0, 8.0)),
            )
            .expect("入箱");
        sched.tick();
        assert_eq!(
            sched
                .owner_of_entity(e)
                .map(|o| (o.shard, o.source, o.actor)),
            Some((shard_id(0, 0), Some(InputSource(7)), false)),
            "spawn 后 ownership 记录 owner/source/actor"
        );
        sched
            .submit(
                shard_id(0, 0),
                cmd_input(
                    InputSource(7),
                    2,
                    Tick(1),
                    SimCommand::RemoveEntity { entity: e },
                ),
            )
            .expect("入箱");
        sched.tick();
        assert!(
            sched.owner_of_entity(e).is_none(),
            "Remove 后 ownership 释放"
        );
        sched.shutdown();
    }

    #[test]
    fn teleport_keeps_owner_source_and_actor() {
        let mut sched = Scheduler::new(sched_cfg(1));
        sched.register(sim(0, 0));
        let e = types::EntityId::from_parts(1, 1);
        sched
            .submit(
                shard_id(0, 0),
                cmd_input(InputSource(7), 1, Tick::ZERO, spawn_cmd(e, 8.0, 8.0)),
            )
            .expect("入箱");
        sched.tick();
        sched
            .submit(
                shard_id(0, 0),
                cmd_input(
                    InputSource::SCHEDULER,
                    2,
                    Tick(1),
                    SimCommand::MarkActor {
                        entity: e,
                        actor: true,
                    },
                ),
            )
            .expect("入箱");
        sched.tick();
        assert_eq!(
            sched.owner_of_entity(e).map(|o| (o.source, o.actor)),
            Some((Some(InputSource(7)), true))
        );
        sched
            .submit(
                shard_id(0, 0),
                cmd_input(
                    InputSource(7),
                    3,
                    Tick(2),
                    SimCommand::TeleportEntity {
                        entity: e,
                        state: types::EntityState {
                            translation: Vec3::new(20.0, 10.0, 8.0),
                            ..types::EntityState::default()
                        },
                    },
                ),
            )
            .expect("入箱");
        sched.tick();
        assert_eq!(
            sched
                .owner_of_entity(e)
                .map(|o| (o.shard, o.source, o.actor)),
            Some((shard_id(0, 0), Some(InputSource(7)), true)),
            "Teleport 后 owner 不变,source/actor 保留"
        );
        sched.shutdown();
    }

    #[test]
    fn migration_moves_owner_at_envelope_apply_tick() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let e = types::EntityId::from_parts(1, 1);
        sched
            .submit(
                shard_id(0, 0),
                cmd_input(InputSource(7), 1, Tick::ZERO, spawn_cmd(e, 120.0, 8.0)),
            )
            .expect("入箱");
        sched.tick();
        assert_eq!(
            sched.owner_of_entity(e).map(|o| o.shard),
            Some(shard_id(0, 0))
        );
        // 传送到本片 core 外 → 本拍 despawn、信封 apply_at=N+1;窗口内 owner=None。
        sched
            .submit(
                shard_id(0, 0),
                cmd_input(
                    InputSource(7),
                    2,
                    Tick(1),
                    SimCommand::TeleportEntity {
                        entity: e,
                        state: types::EntityState {
                            translation: Vec3::new(136.0, 10.0, 8.0),
                            ..types::EntityState::default()
                        },
                    },
                ),
            )
            .expect("入箱");
        sched.tick();
        assert!(sched.owner_of_entity(e).is_none(), "迁移窗口 owner 缺失");
        sched.tick();
        assert_eq!(
            sched.owner_of_entity(e).map(|o| (o.shard, o.source)),
            Some((shard_id(1, 0), Some(InputSource(7)))),
            "目的片 spawn 后 owner=dest,source 保留"
        );
        sched.shutdown();
    }

    #[test]
    #[should_panic(expected = "全局唯一性")]
    fn duplicate_entity_spawn_across_shards_panics() {
        let mut sched = Scheduler::new(sched_cfg(2));
        sched.register(sim(0, 0));
        sched.register(sim(1, 0));
        let e = types::EntityId::from_parts(1, 1);
        sched
            .submit(
                shard_id(0, 0),
                cmd_input(InputSource(7), 1, Tick::ZERO, spawn_cmd(e, 8.0, 8.0)),
            )
            .expect("入箱");
        sched
            .submit(
                shard_id(1, 0),
                cmd_input(InputSource(7), 2, Tick::ZERO, spawn_cmd(e, 136.0, 8.0)),
            )
            .expect("入箱");
        sched.tick();
    }

    #[test]
    #[should_panic(expected = "全局唯一性")]
    fn rebuild_detects_duplicate_entity_across_shards() {
        let mut s0 = sim(0, 0);
        let mut s1 = sim(1, 0);
        let e = types::EntityId::from_parts(1, 1);
        for (s, x) in [(&mut s0, 8.0), (&mut s1, 136.0)] {
            s.apply_prefix(&[cmd_input(
                InputSource(7),
                1,
                Tick::ZERO,
                spawn_cmd(e, x, 8.0),
            )]);
        }
        let mut sched = Scheduler::new(sched_cfg(1));
        sched.register(s0);
        sched.register(s1); // rebuild 全量时发现重复
    }
}
