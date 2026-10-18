//! REQ-002 验收套件(任务书 05):AC-G1~G6 端到端(qwen 执笔,与实现交叉)。
//!
//! 覆盖 REQ-002 §10 六条验收标准;工程细节(任务书):driver 路径用
//! `#[tokio::test(start_paused = true)]`;despawn 窗口用小配置加速;
//! AC-G4 的"复活"经驱动 ensure 路径(观察者移回,非手工 register);
//! 位级断言 `==`,物理量按容差。
//!
//! 场景几何口径(照 driver.rs 任务 04 测试 / app_e2e.rs):
//! - Superflat classic = stone×1 + dirt×3,层底世界 y = WORLD_Y_MIN(-64)
//!   → 地面顶面 world y = -60(实体半边 0.5,驻留中心 ≈ -59.4);
//! - 观察者/实体放置 y 取地面附近,保证 3D 距离 ≤ 唤醒半径 24m(不被 tier③ 冻结)。

use std::sync::{Arc, Mutex};

use app::config::Config;
use app::driver::{CheckpointBundle, DriverArgs, DriverCommand, run};
use messaging::SimCommand;
use physics_sim::{
    ActivationConfig as SimActivationConfig, ShardSim, ShardSimConfig, SolidifyConfig,
};
use shard::SchedulerConfig;
use sim_api::log::CommandLog;
use sim_api::replay::replay;
use sim_api::session::{Session, SessionConfig};
use sim_api::snapshot::GlobalSnapshot;
use storage::mem::MemCheckpointStore;
use storage::provider::ChunkProvider;
use storage::redb::RedbStore;
use storage::traits::{CheckpointId, CheckpointStore, ChunkKey, ChunkStore, StoreError};
use storage::worker::StoreWorker;
use types::glam::Vec3;
use types::{
    BlockId, BlockPos, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos,
    Tick, WorldId, WorldSeed,
};
use voxel::{BlockRegistry, Chunk, SimpleRegistry, Superflat, WorldGenerator};

/// Superflat classic 地面顶面 world y(-64 + 层厚 4)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;

/// 测试用分片身份(与 driver / shard 测试同风格)。
fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// 实体身份。
fn eid(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

/// chunk 存储键(单世界单维度)。
fn key(world: u32, dim: u32, x: i32, z: i32) -> ChunkKey {
    ChunkKey {
        world: WorldId(world),
        dim: DimensionId(dim),
        pos: ChunkPos::new(x, z),
    }
}

/// registry 工厂。
fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

/// 共享 mem 检查点存储(测试侧留克隆读回;照 driver.rs 测试同款)。
#[derive(Clone, Default)]
struct SharedCheckpoints(Arc<Mutex<MemCheckpointStore>>);

impl CheckpointStore for SharedCheckpoints {
    fn append(&mut self, at: types::Tick, blob: &[u8]) -> Result<CheckpointId, StoreError> {
        self.0.lock().expect("检查点锁毒化").append(at, blob)
    }
    fn read_latest(&self) -> Result<Option<(types::Tick, Vec<u8>)>, StoreError> {
        self.0.lock().expect("检查点锁毒化").read_latest()
    }
    fn read(&self, id: CheckpointId) -> Result<Option<(types::Tick, Vec<u8>)>, StoreError> {
        self.0.lock().expect("检查点锁毒化").read(id)
    }
}

/// 共享 mem chunk 存储(测试侧读回断言持久化内容;照 driver.rs
/// chunk_unload 测试同款)。
#[derive(Clone, Default)]
struct SharedChunks(Arc<Mutex<storage::mem::MemChunkStore>>);

impl ChunkStore for SharedChunks {
    fn put_batch(&mut self, batch: &[(ChunkKey, Chunk)]) -> Result<(), StoreError> {
        self.0.lock().expect("chunk 锁毒化").put_batch(batch)
    }
    fn get(&self, k: ChunkKey) -> Result<Option<Chunk>, StoreError> {
        self.0.lock().expect("chunk 锁毒化").get(k)
    }
    fn len(&self) -> Result<usize, StoreError> {
        self.0.lock().expect("chunk 锁毒化").len()
    }
    fn is_empty(&self) -> Result<bool, StoreError> {
        self.0.lock().expect("chunk 锁毒化").is_empty()
    }
}

/// 暂停时间下推进 N 拍(照 driver.rs / app_e2e.rs 同款:auto-advance,
/// 测试 sleep 与驱动 interval 同刻到期 → 每 sleep 恰一拍)。
async fn advance_ticks(ticks: u32) {
    let dt = std::time::Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

/// 解码检查点 blob → bundle(自有编码必可解码)。
fn decode_bundle(blob: &[u8]) -> CheckpointBundle {
    let (bundle, _len) =
        bincode::serde::decode_from_slice::<CheckpointBundle, _>(blob, bincode::config::standard())
            .expect("驱动自有编码必可解码");
    bundle
}

/// 凝固桥关闭的 new_sim 装配(激活参数取 Config 换算;AC-G3 用:实体保持
/// Dynamic,跨多检查点采样"≥200 拍稳定",不被 P2′ 合法凝固吃掉)。
fn new_sim_no_bridge(config: Config) -> Box<dyn FnMut(ShardId) -> ShardSim + Send> {
    Box::new(move |shard: ShardId| {
        let mut cfg = config.shard_sim_config(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
    })
}

/// 注入观察者移动(驱动命令通道)。
async fn inject_move(tx: &tokio::sync::mpsc::Sender<DriverCommand>, position: Vec3) {
    tx.send(DriverCommand::MoveObserver { position })
        .await
        .expect("驱动仍存活");
}

// ---------------------------------------------------------------------------
// AC-G1:生成确定性
// ---------------------------------------------------------------------------

/// AC-G1:同 (seed, pos) 跨实例两次生成位级相等(Superflat 与 SeededScatter);
/// 不同 seed 的 SeededScatter 必有差异。位级断言 = Chunk 的 Eq(全格比较)。
#[test]
fn acg1_generation_is_bitwise_deterministic() {
    // 跨实例:两个独立构造的生成器(模拟"进程两次装配")。
    let seed = WorldSeed(0x0123_4567_89ab_cdef);
    let scatter_a = voxel::SeededScatter::new(0.5);
    let scatter_b = voxel::SeededScatter::new(0.5);
    for pos in [
        ChunkPos::new(-3, 7),
        ChunkPos::new(0, 0),
        ChunkPos::new(12, -4),
    ] {
        let a = scatter_a.generate(seed, pos);
        let b = scatter_b.generate(seed, pos);
        assert_eq!(a, b, "SeededScatter 同 (seed, {pos:?}) 跨实例位级相等");
    }
    let flat_a = Superflat::classic();
    let flat_b = Superflat::classic();
    let pos = ChunkPos::new(5, -9);
    assert_eq!(
        flat_a.generate(WorldSeed(77), pos),
        flat_b.generate(WorldSeed(77), pos),
        "Superflat 同 (seed, pos) 跨实例位级相等"
    );

    // 不同 seed:SeededScatter 有差(散布层由 seed 哈希驱动),且差异非单格偶发。
    let pos = ChunkPos::new(2, 3);
    let s1 = scatter_a.generate(WorldSeed(1), pos);
    let s2 = scatter_b.generate(WorldSeed(2), pos);
    assert_ne!(s1, s2, "不同 seed 的 SeededScatter 必须位级有差");
    let mut diff = 0usize;
    for y in 0..voxel::CHUNK_Y {
        for z in 0..voxel::CHUNK_Z {
            for x in 0..voxel::CHUNK_X {
                if s1.get(x, y, z) != s2.get(x, y, z) {
                    diff += 1;
                }
            }
        }
    }
    assert!(diff > 0, "不同 seed 差异格数为 0(退化)");
}

// ---------------------------------------------------------------------------
// AC-G2:数据源序(持久层优先)
// ---------------------------------------------------------------------------

/// AC-G2:chunk 被修改并持久化后,重装载取回**修改值**而非初生值——全程经
/// ChunkProvider 真路径(redb 后端 + worker 串行读写):兜底 → 修改 put +
/// flush 屏障 → 重取 = 修改值;新 provider 实例(重装载等价)仍取修改值。
#[test]
fn acg2_store_takes_priority_over_generator() {
    let dir = tempfile::tempdir().expect("临时目录");
    let chunks = RedbStore::open(&dir.path().join("chunks.redb")).expect("开 redb 库");
    let worker = StoreWorker::new(Box::new(chunks), Box::new(MemCheckpointStore::new()));

    let seed = WorldSeed(99);
    let generator = voxel::SeededScatter::new(0.3);
    let provider = ChunkProvider::new(&worker, Box::new(generator.clone()), seed);

    let k = key(0, 0, 3, -4);
    let fresh = generator.generate(seed, k.pos);
    assert_eq!(
        provider.get(k),
        Some(fresh.clone()),
        "前置:空持久层 → 兜底生成器初生值"
    );

    // 修改:底层 stone(1) 改成 dirt(2)——与初生值可区分。
    let mut modified = fresh.clone();
    modified.set(1, 0, 2, BlockId(2));
    assert_ne!(modified, fresh, "前提:修改值 ≠ 初生值");
    worker.put_chunks(vec![(k, modified.clone())]);
    assert!(
        worker.flush().recv().expect("flush ack 必达").is_ok(),
        "put 已应用(屏障)"
    );

    // 重装载(provider 读路径 = worker Get):必须取回修改值。
    let reloaded = provider.get(k).expect("必有值");
    assert_eq!(reloaded, modified, "AC-G2:取回修改值");
    assert_ne!(reloaded, fresh, "AC-G2:不得被生成器初生值覆盖");

    // "重装载"的另一半:新 provider 实例(进程内重建)读同一持久层,仍是修改值。
    let provider2 = ChunkProvider::new(&worker, Box::new(generator.clone()), seed);
    assert_eq!(
        provider2.get(k),
        Some(modified),
        "新 provider 亦走持久层优先"
    );
    // 非退化对照:未修改的键仍取回初生值(数据源序只覆盖被改过的 chunk)。
    let k2 = key(0, 0, 3, -5);
    assert_eq!(
        provider2.get(k2),
        Some(generator.generate(seed, k2.pos)),
        "未修改键 = 生成器初生值"
    );

    worker.shutdown().expect("正常关停");
}

// ---------------------------------------------------------------------------
// AC-G3:装载闭环(驱动全链)
// ---------------------------------------------------------------------------

/// AC-G3:驱动全链——MoveObserver 进新区 → 分片由驱动 ensure 生成 → 激活请求
/// 经 provider 真路径装载生成地面 → SpawnDynamic 实体落在生成地面上被支撑,
/// 且 **≥200 拍稳定**(凝固桥关闭;checkpoint_interval=30 → 逐检查点采样,
/// 落稳后的 8 个采样[跨度 ≥200 拍]漂移 < 5cm)。
#[tokio::test(start_paused = true)]
async fn acg3_observer_walk_builds_playable_world() {
    let config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 30,
        ..Config::default()
    };
    // 空会话:分片必由驱动 ensure 造出(世界自己长出来)。
    let session = Session::new(config.session_config());
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(16);
    let checkpoints = SharedCheckpoints::default();
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(checkpoints.clone()),
    );
    let provider = ChunkProvider::new(&store, Box::new(Superflat::classic()), config.world_seed);
    let driver = tokio::spawn(run(DriverArgs {
        config: config.clone(),
        session,
        store,
        log: CommandLog::new(),
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim: new_sim_no_bridge(config.clone()),
        observe: None,
    }));

    // 观察者进分片 (1,0) 新区(x=200 → chunk 12 → 分片 (1,0));y 取地面顶面
    // 附近,避免 3D 冻结半径误冻(同 driver.rs 任务 04 测试口径)。
    inject_move(&tx, Vec3::new(200.0, GROUND_TOP_Y, 8.0)).await;
    // 先过 1 拍:tick 末端编排 ensure 出分片,之后才允许向该分片发命令
    // (未注册分片 send = 调度器 fail-stop)。
    advance_ticks(1).await;
    tx.send(DriverCommand::Send {
        shard: shard_id(1, 0),
        command: SimCommand::SpawnDynamic {
            entity: eid(1),
            state: EntityState {
                translation: Vec3::new(200.5, -55.0, 8.5),
                ..EntityState::default()
            },
            desc: EntityDesc::default(),
        },
    })
    .await
    .expect("驱动仍存活");
    // 360 拍:装载(晚一拍)+ 落稳(探针实测 ~80 拍,driver.rs 口径)+ 驻留;
    // 检查点拍 30/60/…/360 → 实体 y 采样 ≥12 个;最后 8 个(跨度 ≥210 拍,
    // 落稳 ~80 拍之后)为稳定窗断言对象。
    advance_ticks(360).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome = driver.await.expect("驱动不 panic");

    assert!(
        outcome.shards_spawned >= 1,
        "观察者进新区 → 分片经 ensure 创建"
    );
    assert!(outcome.chunks_loaded >= 1, "激活请求经 provider 真路径装载");

    // 逐检查点采样实体 y(每个 checkpoint 恢复分片快照查实体)。
    let mut samples: Vec<(u64, f32)> = Vec::new();
    for id in 1..=64u64 {
        let Some((_tick, blob)) = checkpoints.read(CheckpointId(id)).expect("读检查点") else {
            break;
        };
        let bundle = decode_bundle(&blob);
        let bytes = bundle
            .snapshot
            .per_shard
            .get(&shard_id(1, 0))
            .expect("观察者分片在快照中");
        let restored =
            ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        let state = restored
            .entity_state(eid(1))
            .expect("凝固桥关闭:实体恒存在");
        samples.push((bundle.snapshot.tick.0, state.translation.y));
    }
    assert!(
        samples.len() >= 10,
        "前置:30 拍间隔 × 360 拍 ≥ 11 个采样,实得 {samples:?}"
    );
    // 非退化:早期采样仍显著高于驻留位(出生 -55,驻留 ≈ -59.4)——确实发生
    // 下落并被生成地面接住。
    assert!(
        samples[0].1 > -58.0,
        "早期采样仍在下落段:y={}",
        samples[0].1
    );
    // 驻留带:最后 8 个采样(拍 ≈60-360,跨度 ≥200 拍;落稳 ~80 拍后的段)
    // 全部被生成地面支撑,且互相漂移 < 5cm(物理量按容差,任务书工程细节)。
    let settled: Vec<f32> = samples.iter().rev().take(8).map(|(_, y)| *y).collect();
    for y in &settled {
        assert!(
            *y > GROUND_TOP_Y && *y < GROUND_TOP_Y + 1.5,
            "实体驻留在生成地面顶面上方(顶面 {GROUND_TOP_Y}):y={y}"
        );
    }
    let ymin = settled.iter().copied().fold(f32::INFINITY, f32::min);
    let ymax = settled.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    assert!(
        ymax - ymin < 0.05,
        "≥200 拍稳定:驻留采样 y 漂移 < 5cm(实测 {:.6},采样 {samples:?})",
        ymax - ymin
    );
}

// ---------------------------------------------------------------------------
// AC-G4:生命周期闭环
// ---------------------------------------------------------------------------

/// AC-G4:驱动全链——观察者进区建片 + 装载 + SetBlock 改方块 → 观察者远离 →
/// chunk 卸载持久化 → 空片无票到期 despawn(缩短窗口 despawn_after_secs=2)
/// → 观察者回 → 分片经驱动 ensure 复活 → 修改过的 chunk 从持久层等价取回。
#[tokio::test(start_paused = true)]
async fn acg4_lifecycle_roundtrip_preserves_data() {
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 600,
        // despawn 窗口 2s = 120 拍(任务书工程细节:小配置加速)。
        despawn_after_secs: 2,
        ..Config::default()
    };
    // 冷却关(瞬移装卸立即生效)+ 预算给足(卸载候选与远邻域装载候选同队竞争,
    // 照 driver.rs chunk_unload 测试同款)。
    config.activation.cooldown_secs = 0;
    config.swap_budget_chunks_per_tick = 14;

    let session = Session::new(config.session_config());
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(16);
    let checkpoints = SharedCheckpoints::default();
    let chunks_store = SharedChunks::default();
    let store = StoreWorker::new(
        Box::new(chunks_store.clone()),
        Box::new(checkpoints.clone()),
    );
    // provider/new_sim 走生产装配函数(seed/world_kind 来自 Config;Superflat)。
    let (provider, new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let driver = tokio::spawn(run(DriverArgs {
        config: config.clone(),
        session,
        store,
        log: CommandLog::new(),
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: None,
    }));

    let shard = shard_id(1, 0);
    let edited = BlockPos::new(200, types::WORLD_Y_MIN, 8); // chunk (12,0) 底层 stone(1)
    // ① 进区:ensure 建片 + 激活装载。
    inject_move(&tx, Vec3::new(200.0, GROUND_TOP_Y, 8.0)).await;
    advance_ticks(8).await;
    // ② 改方块:chunk (12,0) 观察者中心距 0,首拍预算即装(预算 14 ≥ 邻域 12)。
    tx.send(DriverCommand::Send {
        shard,
        command: SimCommand::SetBlock {
            pos: edited,
            block: BlockId(2), // stone → dirt:与初生值可区分
        },
    })
    .await
    .expect("驱动仍存活");
    advance_ticks(2).await;
    // ③ 远离:无票 → chunk 逐卸载(值入持久化批)→ 空片 idle 120 拍 →
    //    despawn 候选 → unload_shard → 批 put + flush 屏障。
    inject_move(&tx, Vec3::new(50_000.0, 0.0, 8.0)).await;
    advance_ticks(150).await;
    // ④ 回来:ensure 复活分片(非手工 register,任务书工程细节)→ 激活请求 →
    //    provider 取数命中持久层(修改值)→ 装载。
    inject_move(&tx, Vec3::new(200.0, GROUND_TOP_Y, 8.0)).await;
    advance_ticks(8).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome = driver.await.expect("驱动不 panic");

    // 生命周期计数:至少两次 ensure(初生 + 复活;瞬移远端也 ensure 一个,
    // 计数只钉下界);离开期间发生卸载与 despawn。
    assert!(
        outcome.shards_spawned >= 2,
        "初生 + 复活各一次 ensure(≥2),实得 {}",
        outcome.shards_spawned
    );
    assert!(outcome.chunks_unloaded >= 1, "离开时发生 chunk 卸载");
    assert!(
        outcome.shards_despawned >= 1,
        "空片无票到期被 despawn(观察区片)"
    );

    // 期望 chunk 值:Superflat 初生 + 修改点(世界 (200,-64,8) → 局部 (8,0,8))。
    let generator = Superflat::classic();
    let mut modified = generator.generate(config.world_seed, ChunkPos::new(12, 0));
    modified.set(8, 0, 8, BlockId(2));
    let pristine_12 = generator.generate(config.world_seed, ChunkPos::new(12, 0));
    assert_ne!(modified, pristine_12, "前提:修改值 ≠ 初生值");

    // ⑤ 持久层(经 worker 真路径落库):修改过的 chunk = 修改值;
    //    相邻未修改 chunk = 初生值(数据等价,含修改过的 chunk)。
    let guard = chunks_store.0.lock().expect("chunk 锁毒化");
    let stored_edited = ChunkStore::get(&*guard, key(0, 0, 12, 0))
        .expect("读库不失败")
        .expect("修改过的 chunk 已落库");
    assert_eq!(stored_edited, modified, "AC-G4:持久化的修改 chunk 完整");
    let stored_11 = ChunkStore::get(&*guard, key(0, 0, 11, 0))
        .expect("读库不失败")
        .expect("未修改 chunk 亦落库");
    assert_eq!(
        stored_11,
        generator.generate(config.world_seed, ChunkPos::new(11, 0)),
        "AC-G4:未修改 chunk = 初生值"
    );
    drop(guard);

    // ⑥ 复活终态(终末检查点快照):观察者分片复活,修改从持久层等价取回。
    let (_tick, blob) = checkpoints
        .read_latest()
        .expect("读最新检查点")
        .expect("必有终末检查点");
    let bundle = decode_bundle(&blob);
    let bytes = bundle
        .snapshot
        .per_shard
        .get(&shard)
        .expect("分片复活并进入终末快照");
    let restored = ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
    assert_eq!(
        restored.voxel().get_block(edited),
        BlockId(2),
        "复活分片的修改点 = 持久层修改值(非初生值)"
    );
    assert_eq!(
        restored
            .voxel()
            .get_block(BlockPos::new(200, types::WORLD_Y_MIN, 9)),
        BlockId(1),
        "同 chunk 其余方块保持初生值(等价,非整体重写)"
    );
    assert!(
        restored.voxel().chunk_positions().count() >= 1,
        "复活分片确有装载内容"
    );
}

// ---------------------------------------------------------------------------
// AC-G5:回放兼容(观察者移动/装载/卸载/爆炸,600 拍,拍 300 快照)
// ---------------------------------------------------------------------------

/// AC-G5 激活配置:冷却 0 + 预算 14。口径说明:本 AC 考察"命令入日志 →
/// 重放再生装载与冻结"的确定性再生;激活簿记时钟是"apply_activation 调用
/// 次数",原线拍 20-300 持续刷新而重放线拍 300-320 跳过(观察者从未设置
/// → tick 跳过刷新),两线该时钟在拍 320 后相差 20——cooldown_ticks=0 时
/// 冷却判据恒真,时钟差不影响任何决策 → 两线判据退化为纯距离/状态函数。
fn acg5_activation_config() -> SimActivationConfig {
    SimActivationConfig {
        cooldown_ticks: 0,
        swap_budget_chunks_per_tick: 14,
        ..SimActivationConfig::default()
    }
}

fn acg5_session_config() -> SessionConfig {
    SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    }
}

fn acg5_registry() -> Box<dyn BlockRegistry> {
    Box::new(SimpleRegistry::builtin())
}

// AC-G5 时间线(全部命令化;对拍 long_replay.rs 场景 + 观察者/装载扩展):
// - 拍 20:move_observer → A=(8,-58,8)(激活开始:邻域装载,预算 14 ≥ 邻域 12
//   → 一拍装齐;A 距分片 (1,0) 核心 AABB 120m > 48 → (1,0) 无票但 idle 窗
//   1800 拍 > 600 → 本场景无 despawn,AC-G4 覆盖);
// - 拍 50:SpawnDynamic e1 @ (8.5,-55,8.5)(地面已载 → 落地驻留 ≈-59.4);
// - 拍 300:快照(两拍之间);
// - 拍 320:move_observer → B=(8,-58,72)(+z 64m:A 邻域 >48 → 卸载;e1 距
//   64 >32 → 冻结悬停;B 邻域 → 新装载);
// - 拍 380:move_observer → A(返回:B 邻域卸载、A 邻域重装载[e1 出生列取
//   持久层…本线无持久化,生成器等价]、e1 唤醒);
// - 拍 440:Detonate 常量参数广播两分片(awake 的 e1 被炸动;爆心距 e1 驻位
//   ≈3.5m → 冲量 ≈ power×(1−3.5/30) = 7.07,m=1 → Δv ≈ 7 m/s,摩擦 ~1.2s 刹停
//   滑行 ~4m,始终在 (0,0) 本体区的已载 chunk 上;参数是命令常量,两线位级
//   同源);
// - 拍 600:终态。
const A_POS: Vec3 = Vec3::new(8.0, -58.0, 8.0);
const B_POS: Vec3 = Vec3::new(8.0, -58.0, 72.0);
const MOVE1_TICK: u64 = 20;
const SPAWN_TICK: u64 = 50;
const SNAPSHOT_TICK: u64 = 300;
const MOVE2_TICK: u64 = 320;
const MOVE3_TICK: u64 = 380;
const BLAST_TICK: u64 = 440;
const BLAST_CENTER: Vec3 = Vec3::new(12.0, -59.2, 8.5);
const BLAST_POWER: f32 = 8.0;
const BLAST_RADIUS: f32 = 30.0;
const FINAL_TICK: Tick = Tick(600);

/// 每拍采样:两分片视角的 e1 状态。
#[derive(Debug, Clone, Copy, PartialEq)]
struct Acg5Sample {
    s0: Option<EntityState>,
    s1: Option<EntityState>,
}

/// AC-G5 单线运行结果。
struct Acg5Line {
    samples: Vec<Acg5Sample>,
    summaries: Vec<shard::TickSummary>,
    offset: u64,
    log: CommandLog,
    /// 装载/卸载落实次数(非退化取证;原线统计全程,重放线统计重放窗,
    /// 一致性与"发生在拍 >300"分别断言)。
    loaded: u64,
    unloaded: u64,
    shard_ids: Vec<ShardId>,
    tick_now: Tick,
}

fn acg5_sample(session: &Session) -> Acg5Sample {
    Acg5Sample {
        s0: session.entity_state(shard_id(0, 0), eid(1)),
        s1: session.entity_state(shard_id(1, 0), eid(1)),
    }
}

/// 生成器 provider(每线独立 mem worker;本场景无写入 → 取数恒走位级确定
/// 生成器兜底,两线内容等价)。
fn acg5_provider() -> (ChunkProvider, StoreWorker) {
    let worker = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    let provider = ChunkProvider::new(&worker, Box::new(Superflat::classic()), WorldSeed(7));
    (provider, worker)
}

/// 驱动编排复刻(AC-G5 场景无 tokio 驱动,测试充当执行方,与 app
/// orchestrate 同序:取请求 → provider.get → apply_chunk_load / 卸载;
/// 卸载值丢弃——两线独立 worker 无交叉,读侧全兜底,等价生成器)。
fn acg5_orchestrate(
    session: &mut Session,
    provider: &ChunkProvider,
    loaded: &mut u64,
    unloaded: &mut u64,
) {
    for (shard, decisions) in session.take_activation_requests() {
        for pos in decisions.chunks_to_load {
            // REQ-005 D5(任务 00 连带):扩张 chunk 的 owner 必须先注册——
            // 原线/重放线共用的编排此前无 ensure,依赖"装载无归属校验"的
            // 旧行为;新不变量下按 owner 惰性 ensure(与驱动 ① 同语义)。
            let owner = types::ShardId::new(shard.world, shard.dimension, pos.shard_of());
            if !session.shard_ids().contains(&owner) {
                let cfg = ShardSimConfig {
                    activation: acg5_activation_config(),
                    solidify: SolidifyConfig {
                        enabled: false,
                        ..SolidifyConfig::default()
                    },
                    ..ShardSimConfig::new(owner)
                };
                session.register(ShardSim::new(cfg, acg5_registry()));
            }
            let key = ChunkKey {
                world: shard.world,
                dim: shard.dimension,
                pos,
            };
            let chunk = provider
                .get(key)
                .unwrap_or_else(|| panic!("provider.get 返回 None(生成器缺陷):{key:?}"));
            session.apply_chunk_load(shard, pos, chunk);
            *loaded += 1;
        }
        for pos in decisions.chunks_to_unload {
            if session.apply_chunk_unload(shard, pos).is_some() {
                *unloaded += 1;
            }
        }
    }
}

/// 跑 AC-G5 原线:固定时间线到拍 600,拍 300 在两拍之间取全局快照。
fn acg5_run_original(snapshot_at: Option<Tick>) -> (Acg5Line, Option<GlobalSnapshot>) {
    let (provider, worker) = acg5_provider();
    let mut session = Session::new(acg5_session_config());
    for x in [0, 1] {
        let cfg = ShardSimConfig {
            activation: acg5_activation_config(),
            solidify: SolidifyConfig {
                enabled: false,
                ..SolidifyConfig::default()
            },
            ..ShardSimConfig::new(shard_id(x, 0))
        };
        session.register(ShardSim::new(cfg, Box::new(SimpleRegistry::builtin())));
    }

    let mut log = CommandLog::new();
    let mut samples = Vec::new();
    let mut summaries = Vec::new();
    let mut snapshot = None;
    let (mut loaded, mut unloaded) = (0u64, 0u64);
    while session.tick_now() < FINAL_TICK {
        let at = session.tick_now();
        if Some(at) == snapshot_at {
            snapshot = Some(session.global_snapshot());
        }
        match at.0 {
            MOVE1_TICK | MOVE2_TICK | MOVE3_TICK => {
                let position = if at.0 == MOVE2_TICK { B_POS } else { A_POS };
                let entry = session
                    .move_observer(position)
                    .expect("观察者收件箱 unbounded");
                log.append(entry);
            }
            SPAWN_TICK => {
                let entry = session
                    .send(
                        shard_id(0, 0),
                        SimCommand::SpawnDynamic {
                            entity: eid(1),
                            state: EntityState {
                                translation: Vec3::new(8.5, -55.0, 8.5),
                                ..EntityState::default()
                            },
                            desc: EntityDesc::default(),
                        },
                    )
                    .expect("收件箱未满");
                log.append(entry);
            }
            BLAST_TICK => {
                // 广播语义照 long_replay.rs:同参数两份分别投两分片(常量参数,
                // 两线位级同源)。
                for shard in [shard_id(0, 0), shard_id(1, 0)] {
                    let entry = session
                        .send(
                            shard,
                            SimCommand::Detonate {
                                center: BLAST_CENTER,
                                power: BLAST_POWER,
                                radius: BLAST_RADIUS,
                            },
                        )
                        .expect("收件箱未满");
                    log.append(entry);
                }
            }
            _ => {}
        }
        summaries.push(session.tick());
        samples.push(acg5_sample(&session));
        acg5_orchestrate(&mut session, &provider, &mut loaded, &mut unloaded);
    }
    let line = Acg5Line {
        samples,
        summaries,
        offset: 0,
        log,
        loaded,
        unloaded,
        shard_ids: session.shard_ids(),
        tick_now: session.tick_now(),
    };
    session.shutdown();
    worker.shutdown().expect("正常关停");
    (line, snapshot)
}

/// 跑 AC-G5 重放线:快照 + tick>300 日志(replay 按变体路由观察者命令 →
/// 调度器专用收件箱);测试逐拍继续复刻编排直到拍 600。
fn acg5_run_replay(snapshot: &GlobalSnapshot, log: &CommandLog) -> Acg5Line {
    let (provider, worker) = acg5_provider();
    let mut session =
        replay(snapshot, log, acg5_session_config(), acg5_registry).expect("自有快照必可恢复");
    assert_eq!(session.tick_now(), snapshot.tick, "重放线自快照拍对齐续跑");
    let mut samples = Vec::new();
    let mut summaries = Vec::new();
    let (mut loaded, mut unloaded) = (0u64, 0u64);
    while session.tick_now() < FINAL_TICK {
        summaries.push(session.tick());
        samples.push(acg5_sample(&session));
        acg5_orchestrate(&mut session, &provider, &mut loaded, &mut unloaded);
    }
    let line = Acg5Line {
        samples,
        summaries,
        offset: snapshot.tick.0,
        // 重放线命令全部原章直投,不产新日志条目。
        log: CommandLog::new(),
        loaded,
        unloaded,
        shard_ids: session.shard_ids(),
        tick_now: session.tick_now(),
    };
    session.shutdown();
    worker.shutdown().expect("正常关停");
    line
}

/// AC-G5:含观察者移动/装载/卸载/爆炸的场景跑 600 拍,拍 300 快照 +
/// tick>300 日志重放到拍 600——重放窗逐拍采样/TickSummary 位级一致 +
/// 终态位级一致(观察者命令入日志 → 装载与冻结确定性再生;对拍 AC8 长程
/// 的扩展场景)。
#[test]
fn acg5_replay_with_observers_and_loading() {
    let (orig, snapshot) = acg5_run_original(Some(Tick(SNAPSHOT_TICK)));
    let snapshot = snapshot.expect("拍 300 必已截取快照");
    assert_eq!(snapshot.tick, Tick(SNAPSHOT_TICK));
    assert_eq!(orig.tick_now, FINAL_TICK);

    // 时间线非退化:3 观察者移动 + 1 出生 + 2 广播爆炸 = 6 条,全入日志。
    let observer_moves = orig
        .log
        .entries()
        .iter()
        .filter(|e| matches!(e.command, SimCommand::SetObserverPosition { .. }))
        .count();
    let blasts = orig
        .log
        .entries()
        .iter()
        .filter(|e| matches!(e.command, SimCommand::Detonate { .. }))
        .count();
    assert_eq!(orig.log.len(), 6, "固定时间线共 6 条");
    assert_eq!(observer_moves, 3, "3 次观察者移动全部入日志");
    assert_eq!(blasts, 2, "Detonate 广播 2 份入日志");
    // 非退化:全程装载与卸载都发生;重放窗(>300)内两线各自独立发生。
    assert!(
        orig.loaded >= 12 && orig.unloaded >= 12,
        "非退化:原线装 {}/卸 {} 均需 ≥12(邻域 12 chunk)",
        orig.loaded,
        orig.unloaded
    );
    // 爆炸真把驻留的 e1 炸动(440 后 30 拍窗内 |v|max > 0.5)。
    let mut max_v = 0.0f32;
    for k in BLAST_TICK..BLAST_TICK + 30 {
        let s = orig.samples[(k - orig.offset) as usize];
        if let Some(st) = s.s0.or(s.s1) {
            max_v = max_v.max(st.linvel.length());
        }
    }
    assert!(max_v > 0.5, "非退化:爆炸必须炸动 e1(|v|max = {max_v})");

    let replayed = acg5_run_replay(&snapshot, &orig.log);
    // 再生证据:重放窗内装载与卸载确实发生(非"空转等同"),且两线重放窗
    // 计数一致(原线 0-300 的装载发生在快照内,不参与比较——原线全程计数
    // 只在重放窗语义下与 300-600 段断言)。
    assert!(
        replayed.loaded >= 12 && replayed.unloaded >= 12,
        "非退化:重放线再生装载/卸载:装 {}/卸 {}",
        replayed.loaded,
        replayed.unloaded
    );

    // 重放窗 [300,600) 逐拍位级一致。
    assert_eq!(replayed.samples.len(), (600 - SNAPSHOT_TICK) as usize);
    for (i, (o, r)) in orig.samples[SNAPSHOT_TICK as usize..]
        .iter()
        .zip(&replayed.samples)
        .enumerate()
    {
        assert_eq!(
            o,
            r,
            "重放窗内绝对拍 {} 采样位级一致",
            SNAPSHOT_TICK as usize + i
        );
    }
    for (i, (o, r)) in orig.summaries[SNAPSHOT_TICK as usize..]
        .iter()
        .zip(&replayed.summaries)
        .enumerate()
    {
        assert_eq!(
            o,
            r,
            "重放窗内绝对拍 {} TickSummary 一致",
            SNAPSHOT_TICK as usize + i
        );
    }
    // 终态位级:实体、分片键集、拍号。
    assert_eq!(
        replayed.samples.last(),
        orig.samples.last(),
        "AC-G5:到拍 600 终态位级一致"
    );
    assert_eq!(replayed.shard_ids, orig.shard_ids, "ownership ≡ 键集");
    assert_eq!(replayed.tick_now, orig.tick_now);
}

// ---------------------------------------------------------------------------
// AC-G6:进程重启后持久完整
// ---------------------------------------------------------------------------

/// AC-G6:驱动进区装载 + SetBlock 修改 → 远离卸载 + despawn(flush 屏障)→
/// Shutdown(run 终末 flush + store.shutdown 释放 redb 文件锁)→ **重开 redb
/// 库**(进程重启等价)逐 chunk 完整可读:修改过的 chunk 位级 = 修改版,
/// 未修改的已知 chunk = 初生值。
#[tokio::test(start_paused = true)]
async fn acg6_persisted_after_process_restart() {
    let dir = tempfile::tempdir().expect("临时目录");
    let mut config = Config {
        data_dir: dir.path().to_path_buf(),
        worker_count: 2,
        checkpoint_interval_ticks: 600,
        despawn_after_secs: 2,
        ..Config::default()
    };
    config.activation.cooldown_secs = 0;
    config.swap_budget_chunks_per_tick = 14;

    // —— 段 1:bootstrap(空目录)→ 驱动生命周期 → 完整退出(终末 flush +
    //    store.shutdown,redb 锁释放 = 进程重启的等价前置)——
    let (session, log, store, resume_from) =
        app::driver::bootstrap(&config, new_registry()).expect("装配空目录");
    assert_eq!(resume_from, None, "空目录全新会话");
    let (provider, new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(16);
    let driver = tokio::spawn(run(DriverArgs {
        config: config.clone(),
        session,
        store,
        log,
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: None,
    }));

    let shard = shard_id(1, 0);
    let edited = BlockPos::new(200, types::WORLD_Y_MIN, 8);
    inject_move(&tx, Vec3::new(200.0, GROUND_TOP_Y, 8.0)).await;
    advance_ticks(8).await;
    tx.send(DriverCommand::Send {
        shard,
        command: SimCommand::SetBlock {
            pos: edited,
            block: BlockId(2),
        },
    })
    .await
    .expect("驱动仍存活");
    advance_ticks(2).await;
    inject_move(&tx, Vec3::new(50_000.0, 0.0, 8.0)).await;
    advance_ticks(150).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome = driver.await.expect("驱动不 panic");
    assert!(
        outcome.shards_despawned >= 1 && outcome.chunks_unloaded >= 1,
        "前置:发生卸载与 despawn(屏障 flush)"
    );

    // —— 段 2:"进程重启" = 重新 open 同一个 chunks.redb,逐 chunk 完整可读 ——
    let reopened =
        RedbStore::open(&dir.path().join("chunks.redb")).expect("重开 redb 库(锁已释放)");
    assert!(
        !ChunkStore::is_empty(&reopened).expect("读库不失败"),
        "持久库非空"
    );
    let generator = Superflat::classic();
    let mut modified = generator.generate(config.world_seed, ChunkPos::new(12, 0));
    modified.set(8, 0, 8, BlockId(2));
    assert_eq!(
        generator
            .generate(config.world_seed, ChunkPos::new(12, 0))
            .get(8, 0, 8),
        BlockId(1),
        "期望构造自检:初生 (12,0) 底层是 stone"
    );
    for (cp, expect) in [
        (ChunkPos::new(12, 0), modified.clone()),
        (
            ChunkPos::new(11, 0),
            generator.generate(config.world_seed, ChunkPos::new(11, 0)),
        ),
        (
            ChunkPos::new(13, 0),
            generator.generate(config.world_seed, ChunkPos::new(13, 0)),
        ),
    ] {
        let k = key(0, 0, cp.x, cp.z);
        let stored = ChunkStore::get(&reopened, k)
            .expect("读库不失败")
            .unwrap_or_else(|| panic!("chunk {cp:?} 重开后完整可读"));
        assert_eq!(stored, expect, "chunk {cp:?} 位级完整(含修改点)");
    }
    // 检查点库同样落盘(重启可续跑的另一半;续跑链语义由 app_e2e 覆盖)。
    assert!(dir.path().join("checkpoints.redb").exists());
}
