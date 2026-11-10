//! S02 压力基线(#24 放行评论):按默认配置测量多大规模的破坏事件还能稳住
//! 60 Hz(每拍 16.67 ms)。**只测量和报告,不做任何优化、不改物理行为。**
//!
//! 运行:`cargo run --release -p e2e --example stress_baseline [-- --quick]`
//!
//! ## 生产等价路径(回评口径)
//! - **生产配置**:`app::config::Config::default()`(激活/破坏/粘结/dt/
//!   worker_count=4 全默认,不单独关开关或调阈值)经 `session_config()` /
//!   `shard_sim_config()` 注入。
//! - **生产 tick 路径**:`Session::tick()`(Scheduler 全屏障,生产 worker 池)
//!   加每拍末复刻 `app::driver::orchestrate` 的激活编排:观察者邻片/票据/
//!   迁移目的地 ensure、`take_activation_requests`、`ChunkProvider::get`
//!   (worker 线程 + 生成器兜底,生产取数路径)、`apply_chunk_load/unload`、
//!   despawn。无网络、无渲染、不 sleep,tick 跑完马上下一拍。
//! - **chunk 装载**:走生产激活路径——观察者票驱动 plan,① 体素预算
//!   2 chunk/拍逐步装载,② collider 随 ① 同点构建。存储后端用
//!   `MemChunkStore`(免磁盘 IO 噪声污染计时;取数路径与生产 provider 相同)。
//! - **观察者**:一个,`move_observer` 放事件中心(`InputSource(1)` 会话
//!   source),激活全开。
//! - **场景布置约束(生产语义使然)**:醒体要求观察者 24 m 内、地形装载
//!   32 m 内、`max_awake_dynamic = 512` 硬预算——所有场景实体都摆在观察者
//!   装载半径内;超预算部分被冻结属生产预期行为,如实记录(峰值冻结列)。

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use app::config::Config;
use messaging::InputSource;
use physics_sim::ShardSim;
use physics_sim::cohesion::compound_origin;
use physics_sim::destruction::DEBRIS_HALF_SIZE;
use serde::Serialize;
use sim_api::proto::S2C;
use sim_api::session::Session;
use storage::mem::{MemCheckpointStore, MemChunkStore};
use storage::provider::ChunkProvider;
use storage::traits::ChunkKey;
use storage::worker::StoreWorker;
use types::glam::{Quat, Vec3};
use types::{
    BlockId, BlockPos, CompoundShape, DimensionId, EntityDesc, EntityId, EntityState, ShardId,
    ShardPos, WorldId,
};
use voxel::{SimpleRegistry, builtin};

/// 60 Hz 预算(ms/拍)。
const BUDGET_MS: f64 = 1000.0 / 60.0;
/// 观测窗口上界(拍),从事件触发起。
const MAX_TICKS: u64 = 1800;
/// P95 停止扩张阈值(ms)。
const STOP_P95_MS: f64 = 33.0;
/// 每档重复次数(full;quick = 1)。
const REPEATS: usize = 3;
/// 落定确认拍数:实体数归零须连续保持(碎裂原体的涌现分块是 N+1 生成,
/// despawn 拍的计数恰为 0——单拍归零会在断块拍误判落定,漏掉整批分块)。
const SETTLE_CALM_TICKS: u32 = 2;
/// 空闲基线观测拍数。
const IDLE_TICKS: u64 = 300;
/// 经典超平坦地表(最高实心格 = dirt,世界 y = â61;可站表面 = y â60)。
/// 全部场景坐标以地表为基准(REQ-009 冒烟爆心 â59.5 同款)。
const SURFACE: i32 = -60;

// ================================================================= 基础设施

fn shard(sx: i32, sz: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(sx, sz))
}

fn shard_of_pos(p: BlockPos) -> ShardId {
    shard(p.x.div_euclid(128), p.z.div_euclid(128))
}

fn shard_of_world(p: Vec3) -> ShardId {
    // floor 语义(spawn_owned 校验 = cell.floor → chunk → shard,负坐标向下)。
    shard(
        (p.x.floor() as i32).div_euclid(128),
        (p.z.floor() as i32).div_euclid(128),
    )
}

fn center_of(p: BlockPos) -> Vec3 {
    Vec3::new(p.x as f32 + 0.5, p.y as f32 + 0.5, p.z as f32 + 0.5)
}

/// 生产装配的压测版:`bootstrap`(内存存储)+ `wire_provider_and_new_sim`。
/// 分片不预注册——与生产一致,由编排按观察者/票据/迁移目的地 ensure;
/// 场景 build 只对 spawn 目标分片做一次性显式注册(等价生产"玩家到场、
/// 分片 ensure"的过程,回评写明)。
struct Rig {
    session: Session,
    store: StoreWorker,
    #[allow(dead_code)]
    provider: ChunkProvider,
    new_sim: Box<dyn FnMut(ShardId) -> ShardSim + Send>,
}

impl Rig {
    fn new() -> Self {
        let config = Config::default();
        let store = StoreWorker::new(
            Box::new(MemChunkStore::new()),
            Box::new(MemCheckpointStore::new()),
        );
        let mut session = Session::new(config.session_config());
        // 与 bootstrap 同:区域票上限注入。
        let (max_r, max_c) = config.area_ticket_caps();
        session.set_area_ticket_caps(max_r, max_c);
        let (provider, new_sim) = app::driver::wire_provider_and_new_sim(config, &store, || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("装配失败(fail-stop;MARGIN 不足等类型化错误)");
        Self {
            session,
            store,
            provider,
            new_sim,
        }
    }

    /// 一次性注册分片并按生产换算建 sim(`wire_provider_and_new_sim` 的
    /// `new_sim` 即 `ShardSim::new(config.shard_sim_config(shard))`)。
    fn register(&mut self, s: ShardId) {
        if !self.session.shard_ids().contains(&s) {
            let sim = (self.new_sim)(s);
            self.session.register(sim);
        }
    }

    fn ensure_registered(&mut self, shards: impl IntoIterator<Item = ShardId>) {
        for s in shards {
            self.register(s);
        }
    }

    /// 复刻 `app::driver::orchestrate` 的激活编排(去掉网络缝:观察流请求
    /// 与持久化批在压测无对应物;ensure → 激活装卸 → despawn 三步与生产
    /// 逐语句相同)。
    fn orchestrate(&mut self) {
        // ① 观察者邻片 ensure(ensure_radius = awake_unload_radius,生产值)。
        let ensure_radius = Config::default().activation.awake_unload_radius;
        let mut ensured: BTreeSet<ShardId> = BTreeSet::new();
        for (_source, position) in self.session.observers() {
            for s in shards_within(position, ensure_radius) {
                if ensured.insert(s) {
                    self.register(s);
                }
            }
        }
        // ①″ 票据 ensure + ①′ 迁移目的地 ensure。
        let mut ensure = self.session.ticket_ensure_shards();
        ensure.extend(self.session.pending_shard_destinations());
        for s in ensure {
            if ensured.insert(s) {
                self.register(s);
            }
        }
        // ② 激活请求 → provider 取数回填 / 卸载(BlockWrite 待写格延迟卸载)。
        let deferred = self.session.pending_blockwrite_cells();
        for (s, decisions) in self.session.take_activation_requests() {
            for pos in decisions.chunks_to_load {
                let key = ChunkKey {
                    world: s.world,
                    dim: s.dimension,
                    pos,
                };
                let chunk = self
                    .provider
                    .get(key)
                    .unwrap_or_else(|| panic!("provider.get 返回 None:{key:?}(fail-stop)"));
                self.session.apply_chunk_load(s, pos, chunk);
            }
            for pos in decisions.chunks_to_unload {
                if deferred.get(&s).is_some_and(|c| c.contains(&pos)) {
                    continue;
                }
                let _ = self.session.apply_chunk_unload(s, pos);
            }
        }
        // ③ despawn 候选(压测单次运行 ≤ 1800 拍 < 30 s 阈值,不会触发;
        //    保留与生产同序)。
        let pending = self.session.pending_shard_destinations();
        for s in self.session.take_despawn_requests() {
            if !self.session.shard_ids().contains(&s) || pending.contains(&s) {
                continue;
            }
            let _ = self.session.unload_shard(s);
        }
    }

    fn shutdown(self) {
        self.session.shutdown();
        let _ = self.store.shutdown();
    }
}

/// 生产 `driver::shards_within` 同款(该 fn 不 pub,逐语句复刻,半径语义
/// 一致:点到分片 core AABB 的水平距离)。
fn shards_within(p: Vec3, radius: f32) -> Vec<ShardId> {
    const SHARD_M: f32 = (types::SHARD_CHUNKS * types::CHUNK_SIZE) as f32;
    let min_x = ((p.x - radius) / SHARD_M).floor() as i32;
    let max_x = ((p.x + radius) / SHARD_M).floor() as i32;
    let min_z = ((p.z - radius) / SHARD_M).floor() as i32;
    let max_z = ((p.z + radius) / SHARD_M).floor() as i32;
    let mut out = Vec::new();
    for sx in min_x..=max_x {
        for sz in min_z..=max_z {
            let (cx, cz) = (sx as f32 * SHARD_M, sz as f32 * SHARD_M);
            let dx = (cx - p.x).max(0.0).max(p.x - (cx + SHARD_M));
            let dz = (cz - p.z).max(0.0).max(p.z - (cz + SHARD_M));
            if (dx * dx + dz * dz).sqrt() <= radius {
                out.push(shard(sx, sz));
            }
        }
    }
    out
}

/// 经 Session 正路写一批方块(盖章入日志;压测不回放,日志即弃)。
fn fill(rig: &mut Rig, cells: impl IntoIterator<Item = (BlockPos, BlockId)>) {
    for (pos, block) in cells {
        rig.session
            .set_block(shard_of_pos(pos), pos, block)
            .expect("set_block(owner 分片已注册、chunk 已装载)");
    }
}

/// spawn 单格方块实体(生产 SpawnDynamic 外部缝;server namespace ID)。
fn spawn_box(rig: &mut Rig, id: u32, pos: Vec3, linvel: Vec3, block: BlockId) {
    rig.session
        .spawn_owned(
            shard_of_world(pos),
            EntityId::from_parts(0, id),
            EntityState {
                translation: pos,
                rotation: Quat::IDENTITY,
                linvel,
                angvel: Vec3::ZERO,
            },
            EntityDesc {
                half_size: DEBRIS_HALF_SIZE,
                block,
                shape: None,
            },
            InputSource(7),
        )
        .expect("spawn_owned");
}

/// spawn 竖直 6 格土条复合体(生产形状协议;origin 按承载同款包络中心)。
fn spawn_column_compound(rig: &mut Rig, id: u32, base: BlockPos, vel: Vec3) {
    let cells: Vec<BlockPos> = (0..6)
        .map(|dy| BlockPos::new(base.x, base.y + dy, base.z))
        .collect();
    let origin = compound_origin(&cells);
    let world: Vec<(BlockPos, BlockId)> = cells.iter().map(|c| (*c, builtin::DIRT)).collect();
    let shape = CompoundShape::from_world(origin, &world).expect("6 格形状");
    rig.session
        .spawn_owned(
            shard_of_pos(origin),
            EntityId::from_parts(0, id),
            EntityState {
                translation: center_of(origin),
                rotation: Quat::IDENTITY,
                linvel: vel,
                angvel: Vec3::ZERO,
            },
            EntityDesc::compound(DEBRIS_HALF_SIZE, shape),
            InputSource(7),
        )
        .expect("spawn_owned compound");
}

// ================================================================= 测量

/// 一档一次运行的原始记录。
#[derive(Debug, Clone, Default, Serialize)]
pub(crate) struct Run {
    ticks_observed: u64,
    tick_ms: Vec<f64>,
    /// 峰值醒体数(全窗口 telemetry.entities_awake 最大)。
    peak_awake: usize,
    /// 峰值实体数(全分片 entity_states 合计最大)。
    peak_entities: usize,
    /// 峰值冻结实体数(telemetry.entities_frozen 最大;含滞回与超预算冻结)。
    peak_frozen: usize,
    /// 峰值待地形实体数(telemetry.entities_awaiting_terrain 最大)。
    peak_awaiting: usize,
    /// 强制落地累计(deadline + budget)。
    forced_landings: u64,
    /// 破坏方块累计(blasted + impacted + collapsed)。
    blocks_destroyed: u64,
    /// 非空气凝固写入累计(R3:实体落定的直接证据;空气写入不计)。
    solidified_solid: u64,
    /// 空气写入累计(凝固/强制落地/破坏等 Emergent 写 air;R3 拆分)。
    solidified_air: u64,
    /// 承载塌落生成的复合体累计。
    compounds_spawned: u64,
    /// 落定所需拍数(全世界实体数 == 0,R4;窗口耗尽 = None)。
    settle_ticks: Option<u64>,
    /// 窗口耗尽仍残留的实体数(R4;落定 = 0)。
    leftover_entities: usize,
    /// 实体数峰值拍的一次全量 EntityStates bincode 字节数(全分片合并后
    /// 编码,R5:与生产全量包同口径,removed 空、full = true)。
    peak_snapshot_bytes: usize,
    /// 该拍快照编码耗时(ms)。
    peak_snapshot_ms: f64,
    /// R2:最长连续超预算拍数(持久化字段;计算见 `max_over_budget_streak_fn`)。
    max_over_budget_streak: u64,
    /// R2:累计超预算量 Σ max(0, t − budget)(ms)(持久化字段)。
    total_over_budget_ms: f64,
    /// R2:最差连续 60 拍均值(ms)(持久化字段)。
    worst_60_mean_ms: f64,
    /// S03 遥测:峰值拍(整拍耗时最大那一拍)的 collider 重建次数
    /// (TickSummary.telemetry.collider_rebuilds;只计数不进模拟)。
    peak_tick_rebuilds: u64,
    /// 每拍的 collider 重建次数(与 tick_ms 同序;峰值拍提取用,不入 json)。
    #[serde(skip)]
    tick_rebuilds: Vec<u64>,
    /// 最近一拍全世界实体数(R4 落定判据;measure_tick 内一次遍历顺带
    /// 计出——count 另走一遍 entity_states 会在 debug 下成倍拖慢冒烟)。
    #[serde(skip)]
    pub(crate) last_entity_count: usize,
}

impl Run {
    fn percentile(&self, q: f64) -> f64 {
        percentile(&self.tick_ms, q)
    }
    fn p95(&self) -> f64 {
        self.percentile(0.95)
    }
    fn over_budget(&self) -> usize {
        self.tick_ms.iter().filter(|t| **t > BUDGET_MS).count()
    }
    fn max_ms(&self) -> f64 {
        self.tick_ms
            .iter()
            .cloned()
            .max_by(|a, b| a.partial_cmp(b).expect("tick_ms 非 NaN"))
            .unwrap_or(0.0)
    }
    /// R2:最长连续超预算拍数。
    fn max_over_budget_streak_fn(&self) -> u64 {
        let mut best = 0u64;
        let mut cur = 0u64;
        for t in &self.tick_ms {
            if *t > BUDGET_MS {
                cur += 1;
                best = best.max(cur);
            } else {
                cur = 0;
            }
        }
        best
    }
    /// R2:累计超预算量 Σ max(0, t − budget)(ms)。
    fn total_over_budget_ms_fn(&self) -> f64 {
        self.tick_ms.iter().map(|t| (t - BUDGET_MS).max(0.0)).sum()
    }
    /// R2:最差连续 60 拍均值(不足 60 拍时取全长窗口)。
    fn worst_60_mean_ms_fn(&self) -> f64 {
        const W: usize = 60;
        if self.tick_ms.is_empty() {
            return 0.0;
        }
        let n = self.tick_ms.len();
        if n <= W {
            return self.tick_ms.iter().sum::<f64>() / n as f64;
        }
        // 滑动窗口:先算首窗和,再逐拍增减(O(n))。
        let mut sum: f64 = self.tick_ms[..W].iter().sum();
        let mut worst = sum;
        for i in W..n {
            sum += self.tick_ms[i] - self.tick_ms[i - W];
            worst = worst.max(sum);
        }
        worst / W as f64
    }
}

/// 一档:场景 + 规模 + 全部重复运行。
#[derive(Debug, Clone, Serialize)]
pub(crate) struct Tier {
    pub(crate) scenario: String,
    pub(crate) scale: String,
    pub(crate) runs: Vec<Run>,
    /// 该档因 P95 超阈触发停止时的说明(该档之后只再多跑一档)。
    pub(crate) stop_reason: Option<String>,
}

impl Tier {
    /// 报告口径:P95 的中位数。
    fn p95_median(&self) -> f64 {
        let mut v: Vec<f64> = self.runs.iter().map(|r| r.p95()).collect();
        v.sort_by(|a, b| a.partial_cmp(b).expect("P95 非 NaN"));
        percentile(&v, 0.5)
    }
    /// 3 次 P95 的极差(max − min)。
    fn p95_spread(&self) -> f64 {
        let v: Vec<f64> = self.runs.iter().map(|r| r.p95()).collect();
        let mut mn = f64::INFINITY;
        let mut mx = f64::NEG_INFINITY;
        for x in &v {
            mn = mn.min(*x);
            mx = mx.max(*x);
        }
        mx - mn
    }
    /// 中位 P95 的那次 run(报告行;同值取首个)。
    fn median_run(&self) -> &Run {
        let mut order: Vec<usize> = (0..self.runs.len()).collect();
        order.sort_by(|&a, &b| {
            self.runs[a]
                .p95()
                .partial_cmp(&self.runs[b].p95())
                .expect("P95 非 NaN")
        });
        let mid = order[self.runs.len() / 2];
        &self.runs[mid]
    }
}

/// 线性插值百分位(内部升序复制后计算;q ∈ [0,1];空样本 panic)。
pub(crate) fn percentile(samples: &[f64], q: f64) -> f64 {
    assert!(!samples.is_empty(), "percentile: 空样本");
    assert!((0.0..=1.0).contains(&q), "percentile: q 越界 {q}");
    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).expect("样本非 NaN"));
    if sorted.len() == 1 {
        return sorted[0];
    }
    let pos = q * (sorted.len() - 1) as f64;
    let lo = pos.floor() as usize;
    let hi = pos.ceil() as usize;
    let frac = pos - lo as f64;
    sorted[lo] * (1.0 - frac) + sorted[hi] * frac
}

// ================================================================= 场景

/// 装配产物:事件中心(观察者位置)、触发方式、遥测断言下限。
struct Setup {
    center: Vec3,
    trigger: Trigger,
    /// 预热拍数(生产路径装载 ① 2 chunk/拍,装载圆 ~25 chunk/片,90 拍够)。
    warmup_ticks: u64,
    /// 触发前要求无承载塌落(bridge 静置检查;自塌 = 布景错误 fail-stop)。
    quiet_until_trigger: bool,
    /// 要求 spawn/生成出的实体峰值 ≥ 此值(0 = 不查)。
    expect_min_entities: usize,
    /// 要求破坏方块累计 ≥ 此值(0 = 不查)。
    expect_min_destroyed: u64,
    /// 要求撞击破坏累计 > 0(wall-impact)。
    expect_impacted: bool,
    /// 要求承载塌落复合体累计 > 0(bridge)。
    expect_compounds: bool,
    /// 要求凝固写入累计 > 0(一切有下落实体的场景)。
    expect_solidified: bool,
    /// R4 守恒断言:落定且非空气凝固 == 该值(loose=N、bridge=cells)。
    expect_conservation: bool,
    expect_conservation_cells: u64,
}

impl Default for Setup {
    fn default() -> Self {
        Self {
            center: Vec3::ZERO,
            trigger: Trigger::None,
            warmup_ticks: 90,
            quiet_until_trigger: false,
            expect_min_entities: 0,
            expect_min_destroyed: 0,
            expect_impacted: false,
            expect_compounds: false,
            expect_solidified: false,
            expect_conservation: false,
            expect_conservation_cells: 0,
        }
    }
}

enum Trigger {
    /// 无独立触发命令:实体 spawn 即事件(loose / rain / wall)。
    None,
    /// 引爆(center, power, radius)。
    Blast { power: f32, radius: f32 },
    /// 拆支撑格(写 air)→ 承载塌落(bridge)。
    RemoveSupports(Vec<BlockPos>),
}

struct ScenarioDef {
    scenario: &'static str,
    scale: String,
    build: Box<dyn Fn(&mut Setup) -> Build>,
}

/// 装配产物(build 不 tick,只摆数据):spawn 列表 + 装载后写入的方块。
struct Build {
    /// 预热后 spawn 的实体(生产外部缝;装载完成后再 spawn,避免待地形)。
    spawns: Vec<Spawn>,
    /// 预热后经 set_block 写入的方块(桥/墙;chunk 必须已被激活路径装载)。
    blocks: Vec<(BlockPos, BlockId)>,
    /// blocks 写入后到触发前的静置拍数(0 = 无静置要求)。
    settle_quiet_ticks: u64,
}

/// 一次 spawn 动作。`base` = 实体锚点格(column 用其作竖条底格;box 用
/// `pos` 世界坐标)。
struct Spawn {
    id: u32,
    pos: Vec3,
    base: BlockPos,
    linvel: Vec3,
    block: BlockId,
    /// true = 竖直 6 格复合体(土条,`base` 为底格);false = 单格。
    column: bool,
}

/// 场景阶梯(升序;quick 只取每场景最小档 + cross-shard 两档)。
fn ladder(quick: bool) -> Vec<ScenarioDef> {
    let loose: [usize; 5] = [64, 256, 512, 1024, 2048];
    let bridge: [usize; 3] = [64, 256, 1024];
    let blast: [u32; 4] = [4, 8, 12, 16];
    let wall: [(u32, i32, i32); 3] = [(1, 8, 8), (4, 16, 16), (16, 32, 16)];
    let rain: [usize; 3] = [8, 32, 128];
    let mut all = Vec::new();
    for n in loose {
        all.push(ScenarioDef {
            scenario: "loose",
            scale: format!("N={n}"),
            build: Box::new(move |s| build_loose((), s, n)),
        });
    }
    for cells in bridge {
        all.push(ScenarioDef {
            scenario: "bridge",
            scale: format!("cells={cells}"),
            build: Box::new(move |s| build_bridge((), s, cells)),
        });
    }
    for r in blast {
        all.push(ScenarioDef {
            scenario: "blast",
            scale: format!("radius={r}"),
            build: Box::new(move |s| build_blast((), s, r)),
        });
    }
    for (k, w, h) in wall {
        all.push(ScenarioDef {
            scenario: "wall-impact",
            scale: format!("k={k} wall={w}x{h}"),
            build: Box::new(move |s| build_wall((), s, k, w, h)),
        });
    }
    for m in rain {
        all.push(ScenarioDef {
            scenario: "compound-rain",
            scale: format!("M={m}"),
            build: Box::new(move |s| build_rain((), s, m)),
        });
    }
    all.push(ScenarioDef {
        scenario: "cross-shard",
        scale: "blast radius=8 @4-shard corner".to_string(),
        build: Box::new(|s| build_cross_blast((), s)),
    });
    all.push(ScenarioDef {
        scenario: "cross-shard",
        scale: "loose N=512 @4-shard corner".to_string(),
        build: Box::new(|s| build_cross_loose((), s)),
    });
    if quick {
        // 每场景保留首档(最小档);cross-shard 只有两档且都是该场景的
        // 最小覆盖,两档全留。
        let mut seen = BTreeSet::new();
        let mut out = Vec::new();
        for d in all {
            if d.scenario == "cross-shard" || seen.insert(d.scenario.to_string()) {
                out.push(d);
            }
        }
        out
    } else {
        all
    }
}

/// 圆内网格布点(格间距 `spacing`,中心 `(cx, cz)`,距中心 ≤ r_max 的前
/// `n` 个格;按距离升序保证大 N 也全在装载半径内)。
fn disc_cells(n: usize, spacing: i32, cx: i32, cz: i32, r_max: f32) -> Vec<(i32, i32)> {
    let span = (r_max / spacing as f32).ceil() as i32 + 1;
    let mut pts: Vec<(f32, i32, i32)> = Vec::new();
    for dx in -span..=span {
        for dz in -span..=span {
            let d = ((dx * spacing) as f32).hypot((dz * spacing) as f32);
            if d <= r_max {
                pts.push((d, cx + dx * spacing, cz + dz * spacing));
            }
        }
    }
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).expect("距离非 NaN"));
    pts.into_iter().take(n).map(|(_, x, z)| (x, z)).collect()
}

/// 单格方块 spawn 项。
fn box_spawn(id: u32, pos: Vec3, linvel: Vec3, block: BlockId) -> Spawn {
    Spawn {
        id,
        pos,
        base: BlockPos::ZERO,
        linvel,
        block,
        column: false,
    }
}

/// loose:N 个单格土块,10 m 高落下,1 格间距圆内网格(分片 (0,0) 中央)。
///
/// 断言:实体峰值 ≥ N;凝固 ≥ 1(醒着的落定)。超 512 预算的实体被冻结
/// 悬空属生产预期(峰值冻结列如实记录)。
fn build_loose(_rig_witness: (), s: &mut Setup, n: usize) -> Build {
    let (cx, cz) = (64, 64);
    // 装载半径 32 m 内布点(观察者在网格中心上空 10 m,3D 唤醒覆盖)。
    let cells = disc_cells(n, 1, cx, cz, 31.0);
    assert_eq!(cells.len(), n, "loose {n}: 装载半径内布点不足");
    s.center = Vec3::new(
        cx as f32 + 0.5,
        (SURFACE + 10) as f32 + 0.5,
        cz as f32 + 0.5,
    );
    // 生产醒体预算 512(§9.4):N > 512 时,超额瞬态体在 spawn 当拍即被
    // 预算强制落地(就地转方块,§4.8/§4.16 预算归因)——真实规模 = 512
    // 醒体下落 + 其余就地凝固。期望按此下限;强制落地数与醒体峰值在
    // 报告列如实呈现。R4 守恒:全部 N 个最终都进世界(落体凝固 + 强制
    // 落地写块都是非空气 Emergent 写入)→ 非空气凝固 == N。
    s.expect_min_entities = n.min(512);
    s.expect_solidified = true;
    s.expect_conservation = true;
    s.expect_conservation_cells = n as u64;
    Build {
        spawns: cells
            .iter()
            .enumerate()
            .map(|(i, (x, z))| {
                box_spawn(
                    i as u32 + 1,
                    Vec3::new(*x as f32 + 0.5, (SURFACE + 10) as f32, *z as f32 + 0.5),
                    Vec3::ZERO,
                    builtin::DIRT,
                )
            })
            .collect(),
        blocks: Vec::new(),
        settle_quiet_ticks: 0,
    }
}

/// bridge:stone 平板(w×w,悬空 5 m)+ 四角柱(柱脚伸进地面顶格 y=−1,
/// 与超平坦 dirt 层连续 → 接地直达 bedrock;REQ-010 §4.2 接地判定 = 沿 −y
/// 连续实心扫描,悬空 1 格即断)。拆柱后整板承载塌落。cells = 板格数
/// (w = √cells);塌落格数 ≈ 板格数(地面保留)。
fn build_bridge(_witness: (), s: &mut Setup, cells: usize) -> Build {
    let w = (cells as f64).sqrt().round() as i32;
    assert_eq!((w * w) as usize, cells, "bridge cells 须为完全平方数");
    let x0 = 64 - w / 2;
    let z0 = 64 - w / 2;
    let top = SURFACE + 5; // 板层;柱从地表顶格(SURFACE−1)到 top−1
    // 支撑柱阵:每 4 格一根(贯穿 6 层,柱脚埋进 dirt 顶格,接地链直达
    // bedrock)。4 格柱距是承载模型的最小可用间距:bond-8 水平粘结下,
    // 每柱列经 2–4 条邻边吸载 ≈ 24 格重,板非柱列格数(柱距 4 时 ≈ 15/16
    // 的板)必须小于 Σ吸载(4 距时恒成立;6 距在 w ≥ 16 时不够,装配期
    // 分批写入就会塌)。方块提交按"柱 → 柱列板格 → 其余板格按到最近柱
    // 列的切比雪夫距离分层"排序:每批写入后已写部分的悬空源恒小于汇边
    // 容量,装配期不塌;拆柱(Trigger::RemoveSupports)后板与地完全断开,
    // F = 全板,按世界对齐 6 格切块塌落(复合体段)。
    let pillar_cols: Vec<(i32, i32)> = (0..w)
        .filter(|i| i % 4 == 0)
        .flat_map(|ix| {
            (0..w)
                .filter(|k| k % 4 == 0)
                .map(move |iz| (x0 + ix, z0 + iz))
        })
        .collect();
    let pillars: Vec<BlockPos> = pillar_cols
        .iter()
        .flat_map(|(x, z)| ((SURFACE - 1)..top).map(move |y| BlockPos::new(*x, y, *z)))
        .collect();
    let deck_top: Vec<BlockPos> = pillar_cols
        .iter()
        .map(|(x, z)| BlockPos::new(*x, top, *z))
        .collect();
    let chebyshev = |x: i32, z: i32| -> i32 {
        pillar_cols
            .iter()
            .map(|(px, pz)| (x - px).abs().max((z - pz).abs()))
            .min()
            .expect("柱列非空")
    };
    let mut rest: Vec<(i32, i32, i32)> = Vec::new();
    for x in x0..x0 + w {
        for z in z0..z0 + w {
            if pillar_cols.contains(&(x, z)) {
                continue;
            }
            rest.push((chebyshev(x, z), x, z));
        }
    }
    rest.sort_by_key(|(d, x, z)| (*d, *x, *z));
    let deck_rest: Vec<BlockPos> = rest
        .into_iter()
        .map(|(_, x, z)| BlockPos::new(x, top, z))
        .collect();
    s.center = Vec3::new(64.0, top as f32 + 0.5, 64.0);
    s.warmup_ticks = 90;
    s.quiet_until_trigger = true;
    s.trigger = Trigger::RemoveSupports(pillars.clone());
    s.expect_min_entities = 1;
    s.expect_compounds = true;
    s.expect_solidified = true;
    // R4 守恒:落定后全部板格凝固回世界(非空气凝固 == cells;柱格被拆,
    // 不回填)。塌落体可能经强制落地写回(已知现象:复合体待地形挂起 →
    // 1200 拍截止强制落地),两种路径都计非空气 Emergent 写入。
    s.expect_conservation = true;
    s.expect_conservation_cells = cells as u64;
    let mut blocks: Vec<(BlockPos, BlockId)> = pillars
        .iter()
        .chain(deck_top.iter())
        .map(|c| (*c, builtin::STONE))
        .collect();
    blocks.extend(deck_rest.iter().map(|c| (*c, builtin::STONE)));
    Build {
        spawns: Vec::new(),
        blocks,
        settle_quiet_ticks: 30,
    }
}

/// blast:圆柱土堆真实爆炸(R1)。圆柱半径 r+1、高 2r+2,从地表(SURFACE)
/// 向上堆到 y = SURFACE + 2r+1;每一列从地表直起、接地链经脚下 dirt 层直达
/// bedrock。爆心 = 圆柱中心(地表上 r+1 格);power = 5r+8(dirt R=2,射线
/// 衰减后近爆心仍 P > k·R = 4,半径内处处可炸)。方块从下往上分批写入
/// (submit_batched 的批间 tick 让承载判定全程看到"柱列接地"的稳定结构)。
fn build_blast(_witness: (), s: &mut Setup, radius: u32) -> Build {
    let r = radius as i32;
    let (cx, cz) = (64, 64);
    let height = 2 * r + 2;
    let mut blocks = Vec::new();
    for dy in 0..height {
        for dx in -r - 1..=r + 1 {
            for dz in -r - 1..=r + 1 {
                if dx * dx + dz * dz <= (r + 1) * (r + 1) {
                    blocks.push((BlockPos::new(cx + dx, SURFACE + dy, cz + dz), builtin::DIRT));
                }
            }
        }
    }
    s.center = Vec3::new(
        cx as f32 + 0.5,
        (SURFACE + height / 2) as f32 + 0.5,
        cz as f32 + 0.5,
    );
    s.trigger = Trigger::Blast {
        power: 5.0 * radius as f32 + 8.0,
        radius: radius as f32,
    };
    // R1 断言口径(评论字面):destroyed ≥ min(2048, 0.15 × (4/3)πr³)。
    // 纯粉碎爆炸不承诺碎屑实体:expect_solidified 不设(旧口径的"凝固"
    // 计数就是爆炸写 air,R3 拆列后不再把它当凝固证据)。落定 = 实体清零
    // (全粉碎时触发后 1 拍即达,如实报告)。
    let volume = (4.0 / 3.0) * std::f64::consts::PI * (r as f64).powi(3);
    s.expect_min_destroyed = ((0.15 * volume) as u64).min(2048);
    Build {
        spawns: Vec::new(),
        blocks,
        settle_quiet_ticks: 0,
    }
}

/// wall-impact:w×h 土墙(厚 1 格,接地),k 个 stone 方块 40 m/s 从 +z
/// 12 m 外撞向墙面(观察者放墙与投射物之间,两侧都在唤醒半径内)。
fn build_wall(_witness: (), s: &mut Setup, k: u32, w: i32, h: i32) -> Build {
    let z = 64;
    let x0 = 64 - w / 2;
    // 墙从地表第一格(SURFACE)起,向上 h 格,接地链经脚下 dirt 直达 bedrock。
    let wall: Vec<BlockPos> = (x0..x0 + w)
        .flat_map(|x| (SURFACE..SURFACE + h).map(move |y| BlockPos::new(x, y, z)))
        .collect();
    s.center = Vec3::new(64.0, SURFACE as f32 + h as f32 * 0.5, z as f32 + 4.5);
    s.trigger = Trigger::None;
    s.warmup_ticks = 90;
    s.expect_min_entities = k as usize;
    s.expect_impacted = true;
    s.expect_solidified = true;
    let cols = (k as i32).min(4);
    let mut spawns = Vec::new();
    // 首触相位对齐(REQ-010 F02a 已知坑:0 穿透 → 微脉冲弹出、0.1 穿透 →
    // 全脉冲断格):40 m/s 步长 2/3 m/拍,起点取墙面接触中心(64.45)后
    // 12 步 − 0.1 穿透余量。飞行 12 拍重力下坠 ½g·(12/60)² ≈ 0.196 m,
    // 起点 y 按此补偿 → 撞击时恰好对准目标格心(half 0.45 < 0.5,单格受
    // 全冲量,不因跨格均摊减半)。
    let start_z = 64.45 + 12.0 * (2.0 / 3.0) - 0.1;
    let drop = 0.5 * 9.81 * (12.0f32 / 60.0).powi(2);
    for i in 0..k {
        let (col, row) = (i as i32 % 4, i as i32 / 4);
        let x = (x0 + w / 2 - (cols - 1) / 2 + col) as f32 + 0.5;
        let row_y = (SURFACE + h / 2 + row * 2).min(SURFACE + h - 1);
        let y = row_y as f32 + 0.5 + drop;
        spawns.push(box_spawn(
            9000 + i,
            Vec3::new(x, y, start_z),
            Vec3::new(0.0, 0.0, -40.0),
            builtin::STONE,
        ));
    }
    Build {
        spawns,
        blocks: wall.iter().map(|c| (*c, builtin::DIRT)).collect(),
        settle_quiet_ticks: 0,
    }
}

/// compound-rain:M 根 6 格土条竖直悬于 12 m 高,1 格间距圆内网格,自由落地。
fn build_rain(_witness: (), s: &mut Setup, m: usize) -> Build {
    let (cx, cz) = (64, 64);
    let cells = disc_cells(m, 1, cx, cz, 31.0);
    assert_eq!(cells.len(), m, "rain {m}: 装载半径内布点不足");
    s.center = Vec3::new(cx as f32 + 0.5, (SURFACE + 12) as f32, cz as f32 + 0.5);
    // 醒体预算按格数计(复合体 = 格数,§9.4):6 格土条 × M > 512 时超额
    // 部分预算强制落地(整块转方块),真实同时下落的根数 ≥ 512/6。
    s.expect_min_entities = (m * 6).min(512) / 6;
    s.expect_solidified = true;
    Build {
        spawns: cells
            .iter()
            .enumerate()
            .map(|(i, (x, z))| Spawn {
                id: i as u32 + 1,
                pos: Vec3::ZERO,
                base: BlockPos::new(*x, SURFACE + 12, *z),
                linvel: Vec3::ZERO,
                block: builtin::DIRT,
                column: true,
            })
            .collect(),
        blocks: Vec::new(),
        settle_quiet_ticks: 0,
    }
}

/// cross-shard:blast radius=8 放在四片交汇角 (0,0)(分片界上的事件);
/// R1 同款圆柱土堆(半径 r+1、高 2r+2,从地表向上),爆心 = 圆柱中心,
/// power = 5r+8。圆柱骑在分片界上,四片都要注册(布景写入的 owner 校验)。
fn build_cross_blast(_witness: (), s: &mut Setup) -> Build {
    let r = 8;
    let height = 2 * r + 2;
    let mut blocks = Vec::new();
    for dy in 0..height {
        for dx in -r - 1..=r + 1 {
            for dz in -r - 1..=r + 1 {
                if dx * dx + dz * dz <= (r + 1) * (r + 1) {
                    blocks.push((BlockPos::new(dx, SURFACE + dy, dz), builtin::DIRT));
                }
            }
        }
    }
    s.center = Vec3::new(0.5, (SURFACE + height / 2) as f32 + 0.5, 0.5);
    s.trigger = Trigger::Blast {
        power: 5.0 * r as f32 + 8.0,
        radius: r as f32,
    };
    // R1 断言口径(评论字面):destroyed ≥ min(2048, 0.15 × (4/3)πr³)。
    // 同 build_blast:纯粉碎不设 expect_solidified。
    let volume = (4.0 / 3.0) * std::f64::consts::PI * (r as f64).powi(3);
    s.expect_min_destroyed = ((0.15 * volume) as u64).min(2048);
    Build {
        spawns: Vec::new(),
        blocks,
        settle_quiet_ticks: 0,
    }
}

/// cross-shard:loose N=512 网格骑在四片交汇角上(跨片实体布点)。
fn build_cross_loose(_witness: (), s: &mut Setup) -> Build {
    let cells = disc_cells(512, 1, 0, 0, 31.0);
    assert_eq!(cells.len(), 512, "cross-loose: 布点不足");
    s.center = Vec3::new(0.5, (SURFACE + 10) as f32 + 0.5, 0.5);
    s.expect_min_entities = 512;
    s.expect_solidified = true;
    Build {
        spawns: cells
            .iter()
            .enumerate()
            .map(|(i, (x, z))| {
                box_spawn(
                    i as u32 + 1,
                    Vec3::new(*x as f32 + 0.5, (SURFACE + 10) as f32, *z as f32 + 0.5),
                    Vec3::ZERO,
                    builtin::DIRT,
                )
            })
            .collect(),
        blocks: Vec::new(),
        settle_quiet_ticks: 0,
    }
}

// ================================================================= 运行

#[derive(Default)]
struct WindowTotals {
    forced_landings: u64,
    blocks_destroyed: u64,
    solidified_solid: u64,
    solidified_air: u64,
    compounds_spawned: u64,
    cells_impacted: u64,
    flow_events: u64,
    flow_prefiltered: u64,
    flow_breaks: u64,
}

/// 跑一档一次:装配 → 观察者 → 预热(生产路径装载)→ 写方块/ spawn →
/// (静置检查)→ 触发 → 观测窗口 → 遥测断言。
fn run_once(def: &ScenarioDef) -> Run {
    let mut rig = Rig::new();
    let mut setup = Setup::default();
    let build = (def.build)(&mut setup);

    // 观察者(事件中心;下一拍生效,激活开启)。
    rig.session.move_observer(setup.center).expect("观察者设置");

    // 预热:激活装载就绪(① 预算 2 chunk/拍;装载圆 ~25 chunk/片,90 拍够)。
    warmup(&mut rig, setup.warmup_ticks);

    // 观测窗口起点:Trigger::None(loose/rain/wall)的实体 spawn 即事件
    // ——窗口从第一批实体提交起计(分批 spawn 的批拍计入窗口,峰值实体
    // 因此可见);Blast / RemoveSupports 的布景提交是装配,窗口从触发起计。
    let measure_spawns = matches!(setup.trigger, Trigger::None);
    let mut run = Run::default();
    let mut totals = WindowTotals::default();

    // 装载就绪后写方块(set_block 门卫要求 chunk 已装载)与 spawn 实体
    // (生产外部缝;全部落在已装载地形上)。收件箱容量 = 生产默认 256,
    // 大布景分批提交(每批 ≤ 192 条,批间 tick + 编排)。
    submit_batched(
        &mut rig,
        &build.blocks,
        &build.spawns,
        measure_spawns,
        &mut run,
        &mut totals,
    );
    // spawn/写方块后的静置期(bridge:确认结构不自塌,承载 F 为空)。
    if build.settle_quiet_ticks > 0 {
        for _ in 0..build.settle_quiet_ticks {
            let summary = rig.session.tick();
            if summary.cohesion.compounds_spawned != 0 {
                panic!(
                    "{} [{}]: 静置期出现塌落 = 布景错误(fail-stop):falling={} pending={} compounds={} cells={} segments={}",
                    def.scenario,
                    def.scale,
                    summary.cohesion.load_falling,
                    summary.cohesion.load_pending,
                    summary.cohesion.compounds_spawned,
                    summary.cohesion.compound_cells,
                    summary.cohesion.segments,
                );
            }
            let _ = rig.session.take_solidified();
            let _ = rig.session.take_block_updates();
            rig.orchestrate();
        }
    }

    // 触发(Blast / RemoveSupports = 观测窗口拍 0;Trigger::None 已起计)。
    // 拆支撑超收件箱容量时同样分批,批间拍计入窗口(塌落正是窗口内容)。
    match &setup.trigger {
        Trigger::None => {}
        Trigger::Blast { power, radius } => {
            rig.session
                .detonate(setup.center, *power, *radius)
                .expect("detonate");
        }
        Trigger::RemoveSupports(cells) => {
            const BATCH: usize = 192;
            for chunk in cells.chunks(BATCH) {
                fill(&mut rig, chunk.iter().map(|c| (*c, builtin::AIR)));
                measure_tick(&mut rig, &mut run, &mut totals);
                rig.orchestrate();
            }
        }
    }

    // ---- 观测窗口(续;Blast/RemoveSupports 从触发起,Trigger::None 已在
    //      提交段起计)。
    // R4:落定判据 = 全世界实体数为 0(含冻结/待地形/在途——凝固或移除
    // 才会消失),且连续 SETTLE_CALM_TICKS 拍保持(碎裂原体的分块是调度器
    // N+1 生成:despawn 拍实体数恰为 0,下一拍才见分块);窗口耗尽报
    // "未落定"并给出残留实体数。先测后判。
    let mut calm: u32 = 0;
    for _ in 0..MAX_TICKS {
        measure_tick(&mut rig, &mut run, &mut totals);
        if run.last_entity_count == 0 {
            calm += 1;
            if calm >= SETTLE_CALM_TICKS {
                run.settle_ticks = Some(run.ticks_observed);
                break;
            }
        } else {
            calm = 0;
        }
        rig.orchestrate();
    }
    run.leftover_entities = run.last_entity_count;
    run.forced_landings = totals.forced_landings;
    run.blocks_destroyed = totals.blocks_destroyed;
    run.solidified_solid = totals.solidified_solid;
    run.solidified_air = totals.solidified_air;
    run.compounds_spawned = totals.compounds_spawned;
    // R2 指标进 report.json(渲染时方法口径同源;这里落盘持久化值)。
    run.max_over_budget_streak = run.max_over_budget_streak_fn();
    run.total_over_budget_ms = run.total_over_budget_ms_fn();
    run.worst_60_mean_ms = run.worst_60_mean_ms_fn();
    // S03:峰值拍(整拍耗时最大)的重建次数——与 tick_ms 同下标提取;
    // 窗口为空(不会发生,防御)时 0。
    {
        let max_ms = run.max_ms();
        let idx = run
            .tick_ms
            .iter()
            .position(|t| *t == max_ms)
            .unwrap_or(usize::MAX);
        run.peak_tick_rebuilds = run.tick_rebuilds.get(idx).copied().unwrap_or(0);
    }
    // ---- 遥测断言:场景必须真的发生了,否则直接报错退出。
    let tag = format!("{} [{}]", def.scenario, def.scale);
    if setup.expect_min_entities > 0 {
        assert!(
            run.peak_entities >= setup.expect_min_entities,
            "{tag}: 实体峰值 {} < 期望 {},场景未发生(fail-stop)",
            run.peak_entities,
            setup.expect_min_entities
        );
    }
    if setup.expect_min_destroyed > 0 {
        assert!(
            run.blocks_destroyed >= setup.expect_min_destroyed,
            "{tag}: 破坏方块 {} < 期望 {},场景未发生(fail-stop)",
            run.blocks_destroyed,
            setup.expect_min_destroyed
        );
    }
    if setup.expect_impacted {
        assert!(
            totals.cells_impacted > 0,
            "{tag}: 无撞击破坏(cells_impacted == 0),场景未发生(fail-stop):\
             peak_entities={} peak_awake={} flow_breaks={} flow_prefiltered={} flow_events={} \
             cells_impacted={} settle={:?} ticks={}",
            run.peak_entities,
            run.peak_awake,
            totals.flow_breaks,
            totals.flow_prefiltered,
            totals.flow_events,
            totals.cells_impacted,
            run.settle_ticks,
            run.ticks_observed,
        );
    }
    if setup.expect_compounds {
        assert!(
            run.compounds_spawned > 0,
            "{tag}: 无承载塌落复合体,场景未发生(fail-stop)"
        );
    }
    if setup.expect_solidified {
        assert!(
            run.solidified_solid > 0,
            "{tag}: 无非空气凝固写入,实体未落定(fail-stop):\
             air={} destroyed={} compounds={} forced={} peak_entities={} settle={:?} \
             leftover={} ticks={}",
            run.solidified_air,
            run.blocks_destroyed,
            run.compounds_spawned,
            run.forced_landings,
            run.peak_entities,
            run.settle_ticks,
            run.leftover_entities,
            run.ticks_observed,
        );
    }
    // R4 守恒断言:loose / bridge 落定时非空气凝固 == 规模(N / cells)。
    if setup.expect_conservation {
        assert!(
            run.settle_ticks.is_some(),
            "{tag}: 守恒断言要求落定,窗口耗尽(残留 {} 实体)",
            run.leftover_entities
        );
        assert_eq!(
            run.solidified_solid, setup.expect_conservation_cells,
            "{tag}: 非空气凝固 {} != 期望 {},守恒破坏(fail-stop)",
            run.solidified_solid, setup.expect_conservation_cells
        );
    }

    rig.shutdown();
    run
}

/// 预拍(激活装载就绪;每拍编排同窗口内)。
fn warmup(rig: &mut Rig, ticks: u64) {
    for _ in 0..ticks {
        let _ = rig.session.tick();
        let _ = rig.session.take_solidified();
        let _ = rig.session.take_block_updates();
        rig.orchestrate();
    }
}

/// 分批提交布景命令(生产收件箱容量 256 的约束;每批 ≤ 192 条,批间
/// tick + 派生物取走 + 编排)。先全部方块(装配,不计窗口)后全部实体;
/// `measure_spawns` = 实体提交即事件(loose/rain/wall):实体批的 tick
/// 计入观测窗口(计时 + 遥测累计),窗口从第一批实体提交起。
fn submit_batched(
    rig: &mut Rig,
    blocks: &[(BlockPos, BlockId)],
    spawns: &[Spawn],
    measure_spawns: bool,
    run: &mut Run,
    totals: &mut WindowTotals,
) {
    const BATCH: usize = 192;
    for chunk in blocks.chunks(BATCH) {
        fill(rig, chunk.iter().copied());
        rig.session.tick();
        let _ = rig.session.take_solidified();
        let _ = rig.session.take_block_updates();
        rig.orchestrate();
    }
    for chunk in spawns.chunks(BATCH) {
        for sp in chunk {
            if sp.column {
                spawn_column_compound(rig, sp.id, sp.base, sp.linvel);
            } else {
                spawn_box(rig, sp.id, sp.pos, sp.linvel, sp.block);
            }
        }
        if measure_spawns {
            measure_tick(rig, run, totals);
        } else {
            rig.session.tick();
            let _ = rig.session.take_solidified();
            let _ = rig.session.take_block_updates();
        }
        rig.orchestrate();
    }
}

/// 计时一拍(tick 全程)+ 遥测累计与峰值采样(观测窗口的最小单位)。
fn measure_tick(rig: &mut Rig, run: &mut Run, totals: &mut WindowTotals) {
    // Instant 属压测授权面(计时只属于测量,不进模拟;driver 同款 allow)。
    #[allow(clippy::disallowed_methods)]
    let t0 = std::time::Instant::now();
    let summary = rig.session.tick();
    #[allow(clippy::disallowed_methods)]
    {
        run.tick_ms.push(t0.elapsed().as_secs_f64() * 1000.0);
    }
    run.ticks_observed += 1;
    // R3:凝固写入按空气/非空气拆分(take_solidified 是"涌现写块"兼容字段,
    // 含破坏写 air;非空气才是实体落定的证据)。
    for (_, _, block) in rig.session.take_solidified() {
        if block.is_air() {
            totals.solidified_air += 1;
        } else {
            totals.solidified_solid += 1;
        }
    }
    let _ = rig.session.take_block_updates();
    let t = &summary.telemetry;
    totals.forced_landings += t.forced_landings_deadline + t.forced_landings_budget;
    totals.blocks_destroyed += summary.destruction.cells_blasted
        + summary.destruction.cells_impacted
        + summary.destruction.cells_collapsed;
    totals.cells_impacted += summary.destruction.cells_impacted;
    totals.compounds_spawned += summary.cohesion.compounds_spawned;
    totals.flow_events += summary.cohesion.impact_flow_events;
    totals.flow_prefiltered += summary.cohesion.impact_flow_prefiltered;
    totals.flow_breaks += summary.cohesion.impact_flow_breaks;
    // S03:峰值拍(本拍整拍耗时为窗口内最大)的重建次数;最后按窗口重扫
    // 一遍取"max 拍"的计数(见 run_once 收尾)。
    run.tick_rebuilds.push(t.collider_rebuilds);
    run.peak_awake = run.peak_awake.max(t.entities_awake);
    run.peak_frozen = run.peak_frozen.max(t.entities_frozen);
    run.peak_awaiting = run.peak_awaiting.max(t.entities_awaiting_terrain);
    // 全分片实体表:每拍只遍历计实体数(R4 落定判据 + 峰值);新峰值拍
    // 才做一次合并(与生产 driver `visible` 全量口径同构:跨分片合并的
    // BTreeMap,压测不做视距过滤 = 全表)并编码 S2C::EntityStates。
    // 每拍无谓地克隆合并会在 debug 下成倍拖慢冒烟(31 s → 155 s 实测),
    // 故合并只发生在峰值更替拍。
    let mut entities = 0usize;
    for s in rig.session.shard_ids() {
        if let Some(states) = rig.session.entity_states(s) {
            entities += states.len();
        }
    }
    run.last_entity_count = entities;
    if entities > run.peak_entities {
        run.peak_entities = entities;
        let mut merged: std::collections::BTreeMap<EntityId, types::EntityState> =
            std::collections::BTreeMap::new();
        for s in rig.session.shard_ids() {
            if let Some(states) = rig.session.entity_states(s) {
                for (id, state) in states {
                    merged.insert(id, state);
                }
            }
        }
        #[allow(clippy::disallowed_methods)]
        let t0 = std::time::Instant::now();
        let bytes = bincode::serde::encode_to_vec(
            &S2C::EntityStates {
                tick: summary.tick,
                states: merged.into_iter().collect(),
                removed: Vec::new(),
                full: true,
            },
            bincode::config::standard(),
        )
        .expect("EntityStates 编码");
        #[allow(clippy::disallowed_methods)]
        {
            run.peak_snapshot_ms = t0.elapsed().as_secs_f64() * 1000.0;
        }
        run.peak_snapshot_bytes = bytes.len();
    }
}

/// 跑一档(升序调用;返回含停止说明的 Tier)。
///
/// 停止规则(放行评论):某档 P95 中位数 > 33 ms 时,**再往上多跑一档就停**。
/// 由调用方(`run_scenario`)按档序驱动,这里只跑单档。
fn run_tier(def: &ScenarioDef, repeats: usize) -> Tier {
    let mut runs = Vec::new();
    for _ in 0..repeats {
        runs.push(run_once(def));
    }
    Tier {
        scenario: def.scenario.to_string(),
        scale: def.scale.clone(),
        runs,
        stop_reason: None,
    }
}

/// 跑一个场景的整条阶梯(带停止规则与时间预算降级)。
///
/// 时间预算(放行评论:全部跑完控制在 30 分钟内,超了减少重复次数):
/// 从第 900 s 起,后续档 repeats 降为 1(场景不缩减);返回降级说明。
fn run_scenario(
    name: &str,
    defs: &[&ScenarioDef],
    repeats_budget: usize,
    #[allow(clippy::disallowed_methods)] started: std::time::Instant,
) -> (Vec<Tier>, Option<String>) {
    let mut tiers = Vec::new();
    let mut degraded = false;
    let mut overshoot_idx: Option<usize> = None;
    for (i, def) in defs.iter().enumerate() {
        // 降级判定:任何一档开跑前,总耗时超预算一半 → 后续档只跑 1 次
        // (quick 恒 1 次)。
        #[allow(clippy::disallowed_methods)]
        let repeats = if repeats_budget == 1 {
            1
        } else if started.elapsed().as_secs_f64() > 15.0 * 60.0 || degraded {
            degraded = true;
            1
        } else {
            repeats_budget
        };
        let tier = run_tier(def, repeats);
        let over = tier.p95_median() > STOP_P95_MS;
        println!(
            "  {} [{}] runs={} p95_med={:.2} ms spread={:.2} ms{}",
            tier.scenario,
            tier.scale,
            tier.runs.len(),
            tier.p95_median(),
            tier.p95_spread(),
            if over { "  > 33ms 超阈" } else { "" }
        );
        let must_stop_next = overshoot_idx.is_some();
        tiers.push(tier);
        if over && overshoot_idx.is_none() {
            overshoot_idx = Some(i);
        }
        // 超阈档之后再跑一档即停。
        if must_stop_next {
            let reason = format!(
                "场景 {name} 在档「{}」P95 中位数超 {STOP_P95_MS} ms,按规则再跑一档「{}」后停止扩张",
                defs[overshoot_idx.unwrap()].scale,
                def.scale
            );
            if let Some(last) = tiers.last_mut() {
                last.stop_reason = Some(reason.clone());
            }
            return (tiers, Some(reason));
        }
        // 顶档超阈:无更高档可跑,直接停。
        if over && i + 1 == defs.len() {
            let reason = format!(
                "场景 {name} 顶档「{}」P95 中位数超 {STOP_P95_MS} ms(无更高档),停止",
                def.scale
            );
            if let Some(last) = tiers.last_mut() {
                last.stop_reason = Some(reason.clone());
            }
            return (tiers, Some(reason));
        }
    }
    (tiers, None)
}

// ================================================================= 报告

#[derive(Serialize)]
struct Report {
    generated_at_unix_ms: u64,
    quick: bool,
    total_wall_s: f64,
    note: String,
    idle: Run,
    tiers: Vec<Tier>,
}

fn write_report(report: &Report, out_root: &Path) -> PathBuf {
    #[allow(clippy::disallowed_methods)]
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("系统时钟")
        .as_millis();
    let dir = out_root.join(format!("{ts}"));
    std::fs::create_dir_all(&dir).expect("创建输出目录");
    let json = serde_json::to_string_pretty(report).expect("report.json 序列化");
    let json_path = dir.join("report.json");
    std::fs::write(&json_path, &json).expect("写 report.json");
    let md = render_md(report);
    let md_path = dir.join("report.md");
    std::fs::write(&md_path, &md).expect("写 report.md");
    println!("报告:{}", md_path.display());
    println!("     {}", json_path.display());
    dir
}

fn render_md(report: &Report) -> String {
    let mut md = String::new();
    md.push_str("# S02 压力基线报告\n\n");
    md.push_str(&format!(
        "- 模式:{}\n- 总耗时:{:.1} s\n- 预算:{BUDGET_MS:.2} ms/拍(60 Hz);\
         停止阈值:P95 中位数 > {STOP_P95_MS} ms\n- 每档重复:{}(降级时见说明)\n",
        if report.quick { "--quick" } else { "完整" },
        report.total_wall_s,
        if report.quick { 1 } else { REPEATS },
    ));
    md.push_str(&format!("- 说明:{}\n\n", report.note));
    md.push_str(
        "| 场景 | 规模 | 观测拍数 | P50 (ms) | P95 (ms) | P99 (ms) | max (ms) | 超预算拍数 | 最长连续超预算 | 累计超预算 (ms) | 最差 60 拍均值 (ms) | 峰值实体 | 峰值醒体 | 峰值冻结/待地形 | 强制落地 | 落定拍 | 残留实体 | 破坏方块 | 凝固(非空气) | 空气写入 | 复合体 | 快照字节 | 快照耗时 (ms) | 峰值拍重建次数 |\n",
    );
    md.push_str(
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n",
    );
    for tier in &report.tiers {
        let r = tier.median_run();
        md.push_str(&format!(
            "| {} | {} | {} | {:.2} | {:.2} | {:.2} | {:.2} | {} | {} | {:.1} | {:.2} | {} | {} | {}/{} | {} | {} | {} | {} | {} | {} | {} | {} | {:.3} | {} |\n",
            tier.scenario,
            tier.scale,
            r.ticks_observed,
            r.percentile(0.5),
            tier.p95_median(),
            r.percentile(0.99),
            r.max_ms(),
            r.over_budget(),
            r.max_over_budget_streak,
            r.total_over_budget_ms,
            r.worst_60_mean_ms,
            r.peak_entities,
            r.peak_awake,
            r.peak_frozen,
            r.peak_awaiting,
            r.forced_landings,
            match r.settle_ticks {
                Some(t) => t.to_string(),
                None => "未落定".to_string(),
            },
            r.leftover_entities,
            r.blocks_destroyed,
            r.solidified_solid,
            r.solidified_air,
            r.compounds_spawned,
            r.peak_snapshot_bytes,
            r.peak_snapshot_ms,
            r.peak_tick_rebuilds,
        ));
        if let Some(reason) = &tier.stop_reason {
            md.push_str(&format!("\n> 停止:{reason}\n"));
        }
    }
    md.push_str(&format!(
        "\n空闲基线:{} 拍,P50 {:.2} ms / P95 {:.2} ms / max {:.2} ms,醒体峰值 {}\n",
        report.idle.ticks_observed,
        report.idle.percentile(0.5),
        report.idle.p95(),
        report.idle.max_ms(),
        report.idle.peak_awake,
    ));
    md
}

// ================================================================= 入口

/// 跑完整基线(quick 或 full),返回全部档数据(测试用;不写文件)。
pub(crate) fn run_all(quick: bool, out_root: Option<&Path>) -> Vec<Tier> {
    #[allow(clippy::disallowed_methods)]
    let started = std::time::Instant::now();
    let defs = ladder(quick);
    // 按场景分组(保持阶梯顺序)。
    let mut groups: Vec<(String, Vec<&ScenarioDef>)> = Vec::new();
    for def in &defs {
        match groups.last_mut() {
            Some((name, list)) if *name == def.scenario => list.push(def),
            _ => groups.push((def.scenario.to_string(), vec![def])),
        }
    }
    let mut all_tiers: Vec<Tier> = Vec::new();
    let mut notes: Vec<String> = Vec::new();
    for (name, group) in &groups {
        println!("场景 {name}:{} 档", group.len());
        let (tiers, stop) = run_scenario(name, group, if quick { 1 } else { REPEATS }, started);
        if let Some(reason) = stop {
            notes.push(reason);
        }
        all_tiers.extend(tiers);
    }

    // 空闲基线:同样的世界,没有事件。
    let idle = run_idle();
    println!(
        "空闲基线:{} 拍 P95 {:.2} ms",
        idle.ticks_observed,
        idle.p95()
    );

    let total_wall_s = {
        #[allow(clippy::disallowed_methods)]
        {
            started.elapsed().as_secs_f64()
        }
    };
    let report = Report {
        generated_at_unix_ms: {
            #[allow(clippy::disallowed_methods)]
            {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .expect("系统时钟")
                    .as_millis() as u64
            }
        },
        quick,
        total_wall_s,
        note: if notes.is_empty() {
            "全部档位按计划重复次数完成".to_string()
        } else {
            notes.join(";")
        },
        idle,
        tiers: all_tiers,
    };
    // stdout 表格(与 md 同源)。
    print!("{}", render_md(&report));
    if let Some(root) = out_root {
        write_report(&report, root);
    }
    report.tiers
}

/// 空闲基线:同样的世界(观察者 + 装载就绪),无事件。
fn run_idle() -> Run {
    let mut rig = Rig::new();
    let center = Vec3::new(64.5, (SURFACE + 5) as f32 + 0.5, 64.5);
    rig.ensure_registered([shard(0, 0)]);
    rig.session.move_observer(center).expect("观察者设置");
    warmup(&mut rig, 60);
    let mut run = Run::default();
    for _ in 0..IDLE_TICKS {
        #[allow(clippy::disallowed_methods)]
        let t0 = std::time::Instant::now();
        let summary = rig.session.tick();
        #[allow(clippy::disallowed_methods)]
        {
            run.tick_ms.push(t0.elapsed().as_secs_f64() * 1000.0);
        }
        run.ticks_observed += 1;
        let _ = rig.session.take_solidified();
        let _ = rig.session.take_block_updates();
        run.peak_awake = run.peak_awake.max(summary.telemetry.entities_awake);
        rig.orchestrate();
    }
    rig.shutdown();
    run
}

// include 进测试 target 时 main 不作为入口,dead_code 提示按 test-only 豁免。
#[allow(dead_code)]
fn main() {
    let args: Vec<String> = std::env::args().collect();
    let quick = args.iter().any(|a| a == "--quick");
    run_all(quick, Some(Path::new("target/stress")));
}
