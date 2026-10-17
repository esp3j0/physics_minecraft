//! biased select(关机>tick>命令)、MissedTickBehavior::Skip、checkpoint 节奏与断点续跑装配。
//!
//! 驱动形状照 ferrumc v2 driver(调研 §2):biased select 优先级 = 关机 > 到期
//! tick > 命令——命令洪水永远饿不死模拟;`MissedTickBehavior::Skip` 让超载降拍
//! 而不补帧。checkpoint = 快照拍 + 此后命令日志原子一体(README 决策 4),只存增量。

use std::time::Duration;

use serde::{Deserialize, Serialize};
use storage::traits::{CheckpointStore, ChunkKey};

use crate::config::{Config, ConfigError};

/// 驱动指令(外部缝;REQ-003 网络将复用同一形状)。变体只许追加(声明序续在最后)。
#[derive(Debug)]
pub enum DriverCommand {
    Send {
        shard: types::ShardId,
        command: messaging::SimCommand,
    },
    Shutdown,
    /// 观察者移动(REQ-002 §4.2/§4.4 命令化入口):经 `session.move_observer`
    /// 盖章入调度器专用收件箱 + 命令日志(回放确定性的硬要求)。
    MoveObserver {
        position: types::glam::Vec3,
    },
}

/// 驱动装配(全显式,无全局单例)。
pub struct DriverArgs {
    pub config: Config,
    pub session: sim_api::session::Session,
    pub store: storage::worker::StoreWorker,
    pub log: sim_api::log::CommandLog,
    pub commands: tokio::sync::mpsc::Receiver<DriverCommand>,
    /// 是否监听 ctrl_c 为关机源(测试置 false,README 决策 7)。
    pub listen_ctrl_c: bool,
    /// 世界 chunk 数据源(REQ-002 §4.3;bootstrap 装配:worker 通道 + 生成器
    /// (按 world_kind)+ seed)。tick 末端取数回填用。
    pub provider: storage::provider::ChunkProvider,
    /// 注册新分片的构造闭包(shard → ShardSim;REQ-002 §4.4 README 决策 6:
    /// ensure_shard 在 app 层,调度器不知生成器)。闭包捕获 shard_sim_config
    /// 与 registry 工厂。`Send`:run 作为 tokio 任务 spawn,跨 await 持有。
    pub new_sim: Box<dyn FnMut(types::ShardId) -> physics_sim::ShardSim + Send>,
}

/// 驱动结果。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DriverOutcome {
    /// 本次运行的 tick 数。
    pub ticks_run: u64,
    /// 写入的检查点(含终末),按写入序。
    pub checkpoints_written: Vec<types::Tick>,
    /// 因收件箱满被拒的命令数(反压计数,只记录)。
    pub commands_rejected: u64,
    /// 退出时告警是否处于激活(§9.2 只度量)。
    pub alert_active_at_exit: bool,
    /// 驱动编排新建的分片数(REQ-002 §4.4 观察 ensure)。
    pub shards_spawned: u64,
    /// 驱动编排卸载(despawn)的分片数。
    pub shards_despawned: u64,
    /// 经 provider 装载的 chunk 数。
    pub chunks_loaded: u64,
    /// 卸载并入持久化批的 chunk 数(含 despawn 卸载)。
    pub chunks_unloaded: u64,
}

/// 检查点捆绑:快照拍 + 此后命令日志,原子一体(README 决策 4)。
#[derive(Serialize, Deserialize)]
pub struct CheckpointBundle {
    pub snapshot: sim_api::snapshot::GlobalSnapshot,
    pub log_bytes: Vec<u8>,
}

/// 驱动层可预期失败(类型化;蓝图 §4)。
#[derive(Debug, thiserror::Error)]
pub enum AppError {
    #[error("配置错误:{0}")]
    Config(#[from] crate::config::ConfigError),
    #[error("存储错误:{0}")]
    Store(#[from] storage::traits::StoreError),
    #[error("会话错误:{0}")]
    Session(#[from] sim_api::session::SessionError),
    #[error("检查点解码失败:{0}")]
    Decode(String),
}

/// bundle → bincode 字节(standard 配置;全字段可序列化,不可能失败 → fail-stop)。
fn encode(bundle: &CheckpointBundle) -> Vec<u8> {
    bincode::serde::encode_to_vec(bundle, bincode::config::standard())
        .expect("检查点捆绑编码不可能失败(全字段可序列化,fail-stop)")
}

/// bincode 字节 → bundle;损坏 → `Err(AppError::Decode)`(可预期失败走类型化错误)。
fn decode(bytes: &[u8]) -> Result<CheckpointBundle, AppError> {
    let (bundle, _len): (CheckpointBundle, usize) =
        bincode::serde::decode_from_slice(bytes, bincode::config::standard())
            .map_err(|e| AppError::Decode(e.to_string()))?;
    Ok(bundle)
}

/// 检查点节奏:`ticks_run % interval == 0` → bundle = {global_snapshot(),
/// log.take() 的 to_bytes} → `append_checkpoint`(fire-and-forget,决策 5);
/// log.take() 重置后日志只存增量(崩溃丢失窗口 = checkpoint 间隔,§9.11)。
/// 期中与终末同构,差别只在 fire-and-forget 与否。
fn write_checkpoint(
    session: &sim_api::session::Session,
    log: &mut sim_api::log::CommandLog,
    store: &storage::worker::StoreWorker,
    checkpoints_written: &mut Vec<types::Tick>,
) {
    let taken = log.take();
    let bundle = CheckpointBundle {
        snapshot: session.global_snapshot(),
        log_bytes: encode_log(taken),
    };
    let tick = bundle.snapshot.tick;
    store.append_checkpoint(tick, encode(&bundle));
    checkpoints_written.push(tick);
}

/// 日志条目集 → bincode 字节(与 `CommandLog::to_bytes` 同构:重建临时日志整体
/// 编码,标准配置,不可能失败 → fail-stop)。
fn encode_log(entries: Vec<sim_api::log::LogEntry>) -> Vec<u8> {
    let mut log = sim_api::log::CommandLog::new();
    for entry in entries {
        log.append(entry);
    }
    log.to_bytes()
}

/// tick 末端编排累积的计数字段(每拍清零后由 [`orchestrate`] 填充)。
#[derive(Debug, Default, Clone, Copy)]
struct OrchestrationCounters {
    shards_spawned: u64,
    shards_despawned: u64,
    chunks_loaded: u64,
    chunks_unloaded: u64,
}

/// tick 末端编排(REQ-002 §4.4/§4.5;README 决策 8,顺序固定):
///
/// - **观察者分片 ensure**:`observers()` → 所在 shard 未注册 → `new_sim(shard)`
///   + `register`(观察者可走进未注册区域,惰性创建 = 世界自己长出来);
/// - **激活请求装载**:本拍 `take_activation_requests` → 装载 `provider.get(key)`
///   → `apply_chunk_load`;卸载 `apply_chunk_unload` → chunk 累积持久化批,
///   批后 `put_chunks`(fire-and-forget,屏障语义见 despawn 条);
/// - **despawn 候选**:`take_despawn_requests` → 逐个 `unload_shard` → chunks
///   并入持久化批 → **批后一次 flush + recv ack(屏障)**,粘滞错误 panic
///   沿用(与退出路径同款 fail-stop);
/// - **计数**:交还调用方入 outcome。
///
/// 屏障取舍(REQ-002 语义细则 2):常规卸载的 chunk 数据在本拍 fire-and-forget
/// 入队,不单独等——与 checkpoint 同窗口语义(§9.11,终末 flush 兜底);
/// 只有 despawn 批后强制 flush 屏障(分片即将消失,数据必须确定性落库)。
/// 无观察者且从未设置过观察者时,激活请求恒空、ensure/despawn 无候选 → 零动作
/// (向后兼容开关,既有测试行为不变)。
fn orchestrate(
    session: &mut sim_api::session::Session,
    store: &storage::worker::StoreWorker,
    provider: &storage::provider::ChunkProvider,
    new_sim: &mut dyn FnMut(types::ShardId) -> physics_sim::ShardSim,
) -> OrchestrationCounters {
    let mut counters = OrchestrationCounters::default();

    // ① 观察者分片 ensure(读 observers 视图;未注册 → new_sim + register)。
    for (_source, position) in session.observers() {
        let shard = shard_of_position(position);
        if !session.shard_ids().contains(&shard) {
            session.register(new_sim(shard));
            counters.shards_spawned += 1;
        }
    }

    // ② 激活请求 → provider 取数回填 / 卸载累积持久化批。
    let mut put_batch: Vec<(ChunkKey, voxel::Chunk)> = Vec::new();
    for (shard, decisions) in session.take_activation_requests() {
        for pos in decisions.chunks_to_load {
            let key = ChunkKey {
                world: shard.world,
                dim: shard.dimension,
                pos,
            };
            // 生成器兜底后不应 None——None = 生成器实现缺陷 → fail-stop(细则 1)。
            let chunk = provider.get(key).unwrap_or_else(|| {
                panic!("provider.get 返回 None(生成器实现缺陷,fail-stop):{key:?}")
            });
            session.apply_chunk_load(shard, pos, chunk);
            counters.chunks_loaded += 1;
        }
        for pos in decisions.chunks_to_unload {
            if let Some(chunk) = session.apply_chunk_unload(shard, pos) {
                put_batch.push((
                    ChunkKey {
                        world: shard.world,
                        dim: shard.dimension,
                        pos,
                    },
                    chunk,
                ));
                counters.chunks_unloaded += 1;
            }
        }
    }

    // ③ despawn 候选 → unload_shard → chunks 并批 → 一次 flush 屏障。
    let mut despawn_batch: Vec<(ChunkKey, voxel::Chunk)> = Vec::new();
    for shard in session.take_despawn_requests() {
        if !session.shard_ids().contains(&shard) {
            // 候选只记录不清,重复取会重复返回:已卸载的跳过(幂等)。
            continue;
        }
        for (pos, chunk) in session.unload_shard(shard) {
            despawn_batch.push((
                ChunkKey {
                    world: shard.world,
                    dim: shard.dimension,
                    pos,
                },
                chunk,
            ));
            counters.chunks_unloaded += 1;
        }
        counters.shards_despawned += 1;
    }
    if !despawn_batch.is_empty() {
        put_batch.extend(despawn_batch);
    }
    if !put_batch.is_empty() {
        store.put_chunks(put_batch);
    }
    if counters.shards_despawned > 0 {
        // 屏障:despawn 批(连同本拍常规卸载批)落库确认;粘滞错误 panic
        // 沿用退出路径同款 fail-stop。
        let ack = store.flush();
        let flush_result = ack
            .recv()
            .expect("storage-writer 在 despawn flush 前死亡:一致性违规(fail-stop)");
        if let Err(e) = flush_result {
            panic!("存储粘滞错误(fail-stop 停机):{e}");
        }
    }

    // ④ 计数交还。
    counters
}

/// 位置 → 所在分片(与 `Session::move_observer` 的 dest 换算同口径:
/// chunk_of → shard_of,floor 语义)。
fn shard_of_position(position: types::glam::Vec3) -> types::ShardId {
    types::ShardId::new(
        types::WorldId(0),
        types::DimensionId(0),
        types::ChunkPos::new(
            (position.x / 16.0).floor() as i32,
            (position.z / 16.0).floor() as i32,
        )
        .shard_of(),
    )
}

/// 驱动主循环(biased:关机 > tick > 命令;决策 2/3/5)。
/// 正常与关机路径都执行:终末 checkpoint → append → flush → recv ack →
/// session.shutdown → store.shutdown;返回 outcome。
///
/// 分支序(poll 优先级,README 决策 2「关机 > tick > 命令」照 v2 driver):
/// 1. 关机(ctrl_c 仅当 `listen_ctrl_c`);
/// 2. 到期 tick——排在命令前,命令洪水挤不掉模拟;
/// 3. 命令(经 commands 通道:Shutdown 与通道关闭同列)。
///    同刻就绪时 tick 先于命令执行,该命令盖章 = 已推进的拍号。
pub async fn run(args: DriverArgs) -> DriverOutcome {
    let DriverArgs {
        config,
        mut session,
        store,
        mut log,
        mut commands,
        listen_ctrl_c,
        provider,
        mut new_sim,
    } = args;

    let mut ticker = tokio::time::interval(Duration::from_secs_f32(config.dt));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    // tokio interval 首枚 tick 立即到期:构造后先吃掉再进 select,否则首拍连跳
    // (README 决策 2)。
    ticker.tick().await;

    // 度量告警(§9.2):阈值与滑窗来自 config(5ms × 60 tick);只记录不决策。
    let mut telemetry = crate::telemetry::TickTelemetry::new(config.shard_alert_ms, 60);

    let mut ticks_run: u64 = 0;
    let mut commands_rejected: u64 = 0;
    let mut checkpoints_written: Vec<types::Tick> = Vec::new();
    // 编排计数(REQ-002 §4.4:DriverOutcome 增计数字段)。
    let mut shards_spawned: u64 = 0;
    let mut shards_despawned: u64 = 0;
    let mut chunks_loaded: u64 = 0;
    let mut chunks_unloaded: u64 = 0;
    // 上次 checkpoint 时的 ticks_run(相对本次运行;bootstrap 的检查点在起点之前,
    // 故初值 0——起跑后任何 tick 都是"自上次 checkpoint 后的增量")。
    let mut ticks_at_last_checkpoint: u64 = 0;

    // ctrl_c 关机源(仅当 listen_ctrl_c;测试关掉,否则暂停时间测试被信号干扰)。
    let shutdown_signal = async {
        if listen_ctrl_c {
            let _ = tokio::signal::ctrl_c().await;
        } else {
            std::future::pending::<()>().await;
        }
    };
    tokio::pin!(shutdown_signal);

    loop {
        tokio::select! {
            biased;

            // 关机分支(优先级最高):ctrl_c 信号。
            _ = &mut shutdown_signal => break,

            // tick 分支(优先级次之):到期才执行,迟到即 Skip 不补帧。排在命令
            // 之前(biased 书写序 = poll 序):命令洪水(恒就绪的 recv)挤不掉到期
            // 的 tick——命令洪水饿不死模拟(v2 driver 语义,README 决策 2)。
            _ = ticker.tick() => {
                // 计时(§9.2):std Instant 属 app::telemetry 授权面(clippy.toml
                // 顶部注释);喂 telemetry 的只有现成 Duration。
                #[allow(clippy::disallowed_methods)]
                let t0 = std::time::Instant::now();
                // session.tick() 为阻塞调用,直接在 async loop 内执行(毫秒级、
                // 单装配单驱动;REQ-003 网络落地时再 spawn_blocking 化,README 决策 3)。
                // TODO(REQ-003):tick() 移入 spawn_blocking,消掉驱动的阻塞段。
                let _summary = session.tick();
                telemetry.record(t0.elapsed());
                ticks_run += 1;

                // tick 末端编排(REQ-002 §4.4/§4.5,顺序固定 ①②③④):
                // ensure 观察者分片 → 激活请求取数回填/卸载 → despawn 候选 +
                // 持久化屏障 → 计数。无观察者时零动作(向后兼容)。
                let counters = orchestrate(&mut session, &store, &provider, &mut new_sim);
                shards_spawned += counters.shards_spawned;
                shards_despawned += counters.shards_despawned;
                chunks_loaded += counters.chunks_loaded;
                chunks_unloaded += counters.chunks_unloaded;

                // checkpoint 节奏:整间隔倍数拍到期(fire-and-forget,决策 5)。
                if ticks_run.is_multiple_of(config.checkpoint_interval_ticks) {
                    write_checkpoint(&session, &mut log, &store, &mut checkpoints_written);
                    ticks_at_last_checkpoint = ticks_run;
                }
            }

            // 命令分支(优先级最低):Shutdown 指令 / 命令通道关闭。
            maybe_command = commands.recv() => {
                match maybe_command {
                    Some(DriverCommand::Shutdown) | None => break,
                    Some(DriverCommand::Send { shard, command }) => {
                        // send + append(决策 5:先缝后日志,失败即不发)。
                        match session.send(shard, command) {
                            Ok(entry) => log.append(entry),
                            Err(sim_api::session::SessionError::Route(
                                shard::router::RouteError::InboxFull { .. },
                            )) => {
                                // 反压:只计数,不断链(任务书语义细则 2)。
                                commands_rejected += 1;
                            }
                            Err(other) => panic!("会话发送失败(非反压):{other:?}"),
                        }
                    }
                    Some(DriverCommand::MoveObserver { position }) => {
                        // 观察者移动(REQ-002 §4.2 生产入口):盖章 + 专用收件箱
                        // + 命令日志(与 Send 同款:先缝后日志);收件箱 unbounded
                        // → 无 InboxFull 路径,非反压错误 panic(fail-stop)。
                        match session.move_observer(position) {
                            Ok(entry) => log.append(entry),
                            Err(other) => panic!("观察者移动失败(非预期):{other:?}"),
                        }
                    }
                }
            }
        }
    }

    // —— 退出路径(任何关机源,README 决策 5)——
    // 有未落检查点的增量(日志非空或自上次 checkpoint 后有 tick)→ 终末 checkpoint。
    if !log.is_empty() || ticks_run > ticks_at_last_checkpoint {
        write_checkpoint(&session, &mut log, &store, &mut checkpoints_written);
    }
    // flush 只在关停:屏障 = 此前全部操作(期中 + 终末)已落库。
    let ack = store.flush();
    // worker 的 ack 通道不允许被弃收——弃收 = worker panic(按其设计,决策 5)。
    let flush_result = ack
        .recv()
        .expect("storage-writer 在 flush 前死亡:一致性违规(fail-stop)");
    if let Err(e) = flush_result {
        panic!("存储粘滞错误(fail-stop 停机):{e}");
    }
    session.shutdown();
    store
        .shutdown()
        .expect("存储关停失败(粘滞错误已在上一步 panic,此处不应再失败)");

    DriverOutcome {
        ticks_run,
        checkpoints_written,
        commands_rejected,
        alert_active_at_exit: telemetry.alert(),
        shards_spawned,
        shards_despawned,
        chunks_loaded,
        chunks_unloaded,
    }
}

/// 装配:开库 → 查最新检查点(read_latest 在移交 worker 前,决策 6)→
/// 有则 replay 恢复(Session + log 续跑),无则全新;返回驱动所需全部件。
///
/// chunks 库 = `data_dir/chunks.redb`、checkpoints 库 = `data_dir/checkpoints.redb`
/// (目录不存在则创建);chunks 库为将来世界加载预留(本轮只开不用)。
pub fn bootstrap(
    config: &Config,
    new_registry: impl Fn() -> Box<dyn voxel::BlockRegistry>,
) -> Result<
    (
        sim_api::session::Session,
        sim_api::log::CommandLog,
        storage::worker::StoreWorker,
        Option<types::Tick>,
    ),
    AppError,
> {
    let data_dir = &config.data_dir;
    // 目录创建失败经 ConfigError::Io 上抛(AppError 无 IO 变体,任务书签名固定;
    // 配置指定的数据目录不可创建归配置错误)。
    std::fs::create_dir_all(data_dir).map_err(ConfigError::Io)?;
    let chunks_path = data_dir.join("chunks.redb");
    let checkpoints_path = data_dir.join("checkpoints.redb");
    let chunks = storage::redb::RedbStore::open(&chunks_path)?;
    let checkpoints = storage::redb::RedbStore::open(&checkpoints_path)?;

    // read_latest 必须在移交 worker 之前(决策 6:移交后文件锁在手)。
    let latest = checkpoints.read_latest()?;
    let (session, log, resume_from) = match latest {
        Some((at, blob)) => {
            // 有检查点:解码 bundle → replay 恢复(Session + log 续跑,拍号对齐)。
            let bundle = decode(&blob)?;
            let log = sim_api::log::CommandLog::from_bytes(&bundle.log_bytes)
                .map_err(|e| AppError::Decode(e.to_string()))?;
            if bundle.snapshot.tick != at {
                // 库内 (tick, blob) 元组与 bundle 快照拍不一致 = 存储损坏,fail-stop。
                panic!(
                    "检查点元数据不一致:记录 tick {} ≠ bundle 快照拍 {:?}(fail-stop)",
                    at.0, bundle.snapshot.tick
                );
            }
            let session = sim_api::replay::replay(
                &bundle.snapshot,
                &log,
                config.session_config(),
                new_registry,
            )?;
            (session, log, Some(at))
        }
        None => (
            sim_api::session::Session::new(config.session_config()),
            sim_api::log::CommandLog::new(),
            None,
        ),
    };

    let store = storage::worker::StoreWorker::new(Box::new(chunks), Box::new(checkpoints));
    Ok((session, log, store, resume_from))
}

/// bootstrap 之外的第二步装配(REQ-002 §4.3/§4.4):按 world_kind 造生成器,
/// 与 worker 句柄合成 ChunkProvider;new_sim 闭包捕获 **config 本体**(只读
/// 使用 shard_sim_config 换算;取所有权以免借用 DriverArgs.config)与 registry
/// 工厂(README 决策 6/7/9:ensure_shard 在 app 层,调度器不知生成器;
/// Config 是种子/世界种类单一来源)。
///
/// `world_kind`:
/// - `Superflat` → `Superflat::classic()`;
/// - `SeededScatter` → `SeededScatter::new(0.1)`(低密度散布,基底不变)。
pub fn wire_provider_and_new_sim(
    config: Config,
    store: &storage::worker::StoreWorker,
    new_registry: impl Fn() -> Box<dyn voxel::BlockRegistry> + Send + 'static,
) -> (
    storage::provider::ChunkProvider,
    Box<dyn FnMut(types::ShardId) -> physics_sim::ShardSim + Send>,
) {
    let generator: Box<dyn voxel::WorldGenerator> = match config.world_kind {
        crate::config::WorldKind::Superflat => Box::new(voxel::Superflat::classic()),
        crate::config::WorldKind::SeededScatter => Box::new(voxel::SeededScatter::new(0.1)),
    };
    let provider = storage::provider::ChunkProvider::new(store, generator, config.world_seed);
    let new_registry = new_registry;
    let new_sim = Box::new(move |shard: types::ShardId| {
        physics_sim::ShardSim::new(config.shard_sim_config(shard), new_registry())
    });
    (provider, new_sim)
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    use super::*;
    use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
    use storage::mem::{MemCheckpointStore, MemChunkStore};
    use storage::traits::{CheckpointId, CheckpointStore, StoreError};
    use types::glam::Vec3;
    use types::{DimensionId, EntityDesc, EntityId, EntityState, ShardPos, WorldId};
    use voxel::SimpleRegistry;

    /// 测试用分片身份(与 sim-api/shard 测试同风格)。
    fn shard_id(x: i32, z: i32) -> types::ShardId {
        types::ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    /// 凝固桥关闭的虚空分片(session 测试同款:不关心桥行为)。
    fn sim(x: i32, z: i32) -> ShardSim {
        let mut cfg = ShardSimConfig::new(shard_id(x, z));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
    }

    /// 测试配置:2 worker、checkpoint 间隔可覆写、dt 保持默认 1/60。
    fn test_config(checkpoint_interval_ticks: u64) -> Config {
        Config {
            worker_count: 2,
            checkpoint_interval_ticks,
            ..Config::default()
        }
    }

    /// 起一个注册了单个分片 (0,0) 的会话。
    fn session_with_one_shard(config: &Config) -> sim_api::session::Session {
        let mut session = sim_api::session::Session::new(config.session_config());
        session.register(sim(0, 0));
        session
    }

    /// 测试专用检查点存储:`Arc<Mutex<MemCheckpointStore>>` 共享句柄——驱动侧经
    /// `Box<dyn CheckpointStore>` 持有,测试侧留一份克隆在 run 结束后读回断言。
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

    /// 测试驱动参数:mem 后端、不监听 ctrl_c(README 决策 7);返回共享检查点句柄。
    /// 任务 04 起含 provider(superflat 生成器 + config.world_seed)与 new_sim
    /// 闭包(shard_sim_config 换算 + SimpleRegistry::builtin);可选覆写 world_kind。
    fn test_args(
        config: Config,
        session: sim_api::session::Session,
    ) -> (
        DriverArgs,
        tokio::sync::mpsc::Sender<DriverCommand>,
        SharedCheckpoints,
    ) {
        let checkpoints = SharedCheckpoints::default();
        let (tx, rx) = tokio::sync::mpsc::channel(16);
        let store = storage::worker::StoreWorker::new(
            Box::new(MemChunkStore::new()),
            Box::new(checkpoints.clone()),
        );
        let (provider, new_sim) = wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        });
        let args = DriverArgs {
            config,
            session,
            store,
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
        };
        (args, tx, checkpoints)
    }

    /// SpawnDynamic 命令(实体出生在虚空分片原点上方)。
    fn spawn_cmd(entity: EntityId) -> messaging::SimCommand {
        messaging::SimCommand::SpawnDynamic {
            entity,
            state: EntityState {
                translation: Vec3::new(1.5, 3.0, 1.5),
                ..EntityState::default()
            },
            desc: EntityDesc::default(),
        }
    }

    /// 拍间注入一条 Send(命令分支优先级最低:已到期的 tick 先跑,之后才处理
    /// 命令——盖章 apply_at = 注入后第一个 tick 分支执行后的 tick_now)。
    async fn inject_send(
        tx: &tokio::sync::mpsc::Sender<DriverCommand>,
        shard: types::ShardId,
        command: messaging::SimCommand,
    ) {
        tx.send(DriverCommand::Send { shard, command })
            .await
            .expect("驱动仍存活");
    }

    /// 暂停时间下推进 N 拍:测试任务 `sleep(dt)` 靠 **auto-advance** 推进
    /// (tokio clock.rs 文档:`advance` 不保证到期 timer 首次 poll 即交付,
    /// 推荐用 sleep + auto-advance 可靠等一拍)。测试 sleep 与驱动 interval
    /// 同刻到期,同 slot 一起被唤醒 → 每 sleep 恰一拍,无 burst(README 决策 10)。
    async fn advance_ticks(ticks: u32) {
        let dt = Duration::from_secs_f32(1.0 / 60.0);
        for _ in 0..ticks {
            tokio::time::sleep(dt).await;
            // sleep 醒后让驱动跑完本拍处理(tick 分支)再排下一拍。
            tokio::task::yield_now().await;
        }
    }

    /// 读回最新检查点并解码为 bundle(测试侧断言用)。
    fn latest_bundle(checkpoints: &SharedCheckpoints) -> CheckpointBundle {
        let (_tick, blob) = checkpoints
            .read_latest()
            .expect("读最新检查点")
            .expect("必有检查点");
        decode(&blob).expect("自有编码必可解码")
    }

    /// 测试清单 1:start_paused;3 拍 + 拍间注入 Send(SpawnDynamic)→
    /// outcome.ticks_run == 3,实体出现(经终末检查点快照恢复验证)。
    #[tokio::test(start_paused = true)]
    async fn run_executes_ticks_and_commands() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let shard = shard_id(0, 0);
        let entity = EntityId::from_parts(1, 1);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        inject_send(&tx, shard, spawn_cmd(entity)).await;
        advance_ticks(3).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.ticks_run, 3, "3 拍");
        assert_eq!(
            outcome.checkpoints_written,
            vec![types::Tick(3)],
            "终末检查点"
        );

        // 实体出现:终末检查点的分片快照恢复后可查到该实体。
        let bundle = latest_bundle(&checkpoints);
        assert_eq!(bundle.snapshot.tick, types::Tick(3));
        let bytes = bundle
            .snapshot
            .per_shard
            .get(&shard)
            .expect("快照含分片 (0,0)");
        let restored =
            ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        assert!(
            restored.entity_state(entity).is_some(),
            "3 拍后 SpawnDynamic 已生效,实体存在"
        );
    }

    /// 测试清单 2:发 Shutdown → run 返回,outcome 合法。
    #[tokio::test(start_paused = true)]
    async fn shutdown_command_stops_loop() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.ticks_run, 0, "未 tick 即关机");
        assert!(outcome.checkpoints_written.is_empty(), "零拍无检查点");
        assert_eq!(outcome.commands_rejected, 0);
        assert!(!outcome.alert_active_at_exit, "无耗时样本,告警不激活");
        assert!(
            checkpoints.read_latest().expect("读库").is_none(),
            "零拍零命令:不写任何检查点"
        );
    }

    /// 测试清单 3:interval=5,跑 12 拍 → checkpoints == [5, 10, 12]
    /// (12 = 终末);store 用 mem 后端。
    #[tokio::test(start_paused = true)]
    async fn checkpoints_at_interval_and_exit() {
        let config = test_config(5);
        let session = session_with_one_shard(&config);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        advance_ticks(12).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");

        assert_eq!(outcome.ticks_run, 12);
        assert_eq!(
            outcome.checkpoints_written,
            vec![types::Tick(5), types::Tick(10), types::Tick(12)],
            "间隔 5 写 [5,10],终末 12"
        );
        let bundle = latest_bundle(&checkpoints);
        assert_eq!(bundle.snapshot.tick, types::Tick(12), "最新 = 终末");
    }

    /// 测试清单 4:小 inbox(1)灌 2 条同拍命令 → rejected==1,循环仍活。
    #[tokio::test(start_paused = true)]
    async fn rejected_command_counted_not_fatal() {
        let mut config = test_config(600);
        config.inbox_capacity = 1;
        let session = session_with_one_shard(&config);
        let shard = shard_id(0, 0);
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 同拍连灌 2 条(容量 1):第 2 条必被拒(InboxFull),驱动不因此退出。
        inject_send(&tx, shard, spawn_cmd(EntityId::from_parts(1, 1))).await;
        inject_send(&tx, shard, spawn_cmd(EntityId::from_parts(1, 2))).await;
        advance_ticks(2).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.commands_rejected, 1, "恰拒 1 条");
        assert_eq!(outcome.ticks_run, 2, "循环仍活:拒绝不致命");
    }

    /// 测试清单 5:(暂停时间下)跑 20 拍 tick 计数恰 20,无连跳(首枚已吃)。
    #[tokio::test(start_paused = true)]
    async fn interval_skip_does_not_burst() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        advance_ticks(20).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(
            outcome.ticks_run, 20,
            "20 个周期恰 20 拍:Skip 不补帧、首枚已吃"
        );
    }

    /// 回归(评审发现 1):tick 到期与命令就绪同刻竞先时,tick 分支先执行——
    /// 命令洪水饿不死模拟(README 决策 2「关机 > tick > 命令」,v2 语义)。
    /// 判据 = LogEntry.tick(send 盖章 apply_at = tick_now):发送任务与 interval
    /// 第 3 枚 deadline 同刻到期(暂停时钟同 slot;发送任务先 spawn → 其 sleep
    /// 定时器先注册 → 同 slot 先被唤醒)。若命令分支排在 tick 前(biased 书写序),
    /// 命令在 tick#3 之前被处理 → 盖章 2(旧序实证);新序盖章 3。
    #[tokio::test(start_paused = true)]
    async fn due_tick_precedes_queued_command() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let shard = shard_id(0, 0);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 发送任务:与驱动第 3 枚 tick 同刻醒,把命令送入通道。
        let tx_sender = tx.clone();
        let sender = tokio::spawn(async move {
            tokio::time::sleep(Duration::from_secs_f32(1.0 / 60.0) * 3).await;
            tx_sender
                .send(DriverCommand::Send {
                    shard,
                    command: spawn_cmd(EntityId::from_parts(1, 1)),
                })
                .await
                .expect("驱动仍存活");
        });

        advance_ticks(3).await;
        sender.await.expect("发送任务不 panic");
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.ticks_run, 3, "命令排队不吞拍");

        // 终末检查点(bundle.tick = 3)里的日志:新序下命令在 tick#3 之后处理,
        // 盖章 = 已推进的 3 拍。
        let bundle = latest_bundle(&checkpoints);
        assert_eq!(bundle.snapshot.tick, types::Tick(3));
        let entries = sim_api::log::CommandLog::from_bytes(&bundle.log_bytes)
            .expect("自有编码必可解码")
            .take();
        assert_eq!(entries.len(), 1, "注入的命令已入日志");
        assert_eq!(
            entries[0].tick,
            types::Tick(3),
            "到期 tick 先于同刻就绪的命令(v2 语义);盖章 2 即旧序回归"
        );
    }

    /// 测试清单 6:bundle 编解码往返相等;空拍照常出 checkpoint(0 分片快照合法)。
    #[test]
    fn checkpoint_bundle_roundtrip() {
        // 非空:带分片字节 + 日志字节。
        let mut snap = sim_api::snapshot::GlobalSnapshot::new(types::Tick(7));
        snap.insert_shard(shard_id(0, 0), vec![1, 2, 3]);
        let bundle = CheckpointBundle {
            snapshot: snap,
            log_bytes: sim_api::log::CommandLog::new().to_bytes(),
        };
        let decoded = decode(&encode(&bundle)).expect("自有编码必可解码");
        assert_eq!(decoded.snapshot, bundle.snapshot, "快照往返逐字段相等");
        assert_eq!(decoded.log_bytes, bundle.log_bytes);

        // 空快照(0 分片)+ 空日志:同样合法。
        let empty = CheckpointBundle {
            snapshot: sim_api::snapshot::GlobalSnapshot::new(types::Tick(0)),
            log_bytes: Vec::new(),
        };
        let decoded = decode(&encode(&empty)).expect("空 bundle 必可解码");
        assert!(decoded.snapshot.per_shard.is_empty(), "0 分片快照合法");
        assert_eq!(decoded.snapshot.tick, types::Tick(0));
    }

    // ---- 任务 04:驱动接线(测试清单)----

    /// 地面顶面高度(world y):Superflat classic = stone×1 + dirt×3,层底在
    /// 世界 y = WORLD_Y_MIN(-64)→ 顶面 y = -60。
    const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;

    /// MoveObserver 命令(注入通道)。
    async fn inject_move(tx: &tokio::sync::mpsc::Sender<DriverCommand>, position: Vec3) {
        tx.send(DriverCommand::MoveObserver { position })
            .await
            .expect("驱动仍存活");
    }

    /// 测试清单 1 `observer_walk_spawns_shard_and_loads_ground`:MoveObserver 进
    /// 新区 → 分片创建(ensure)→ 激活请求 → provider 真路径装载 → 实体落在
    /// 生成的地面上被支撑。
    #[tokio::test(start_paused = true)]
    async fn observer_walk_spawns_shard_and_loads_ground() {
        let config = test_config(600);
        // 空会话(无预注册分片):分片由驱动 ensure 造出。
        let session = sim_api::session::Session::new(config.session_config());
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 观察者进分片 (1,0) 区域(x=200 → chunk 12 → 分片 (1,0));y 取地面
        // 顶面附近(-60):tier③ 冻结半径 32m 按 3D 欧氏计,观察者悬在 y=0 而
        // 实体在 y=-55 时距离 55m,实体会被立即冻结、无法验证"落在地面被支撑"。
        inject_move(&tx, Vec3::new(200.0, GROUND_TOP_Y, 8.0)).await;
        // 先过 1 拍:让编排 ① 在 tick 末端 ensure 出分片(命令分支比 tick 优先级
        // 低,同拍注入的 SpawnDynamic 会先于 ensure 被 session.send 拒绝——
        // 未注册分片 fail-stop 是调度器语义,测试须保证先建片再发命令)。
        advance_ticks(1).await;
        // 实体出生观察者上方 5m,随装载的地面驻留(到观察者 3D 距离 ≤5.5m,
        // 全程在唤醒半径 24m 内 → 不被冻结,可自由下落)。
        inject_send(
            &tx,
            shard_id(1, 0),
            messaging::SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 1),
                state: EntityState {
                    translation: Vec3::new(200.5, -55.0, 8.5),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            },
        )
        .await;
        // 装载发生在 tick 末端(晚一拍);实体从出生到落稳(探针实测 ~80 拍,
        // rapier 碰撞余量驻留在顶面上方 ~0.57m)需百余拍:120 拍留裕度。
        advance_ticks(120).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(
            outcome.shards_spawned >= 1,
            "观察者进入新区 → 分片被 ensure 创建"
        );
        assert!(outcome.chunks_loaded >= 1, "激活请求经 provider 真路径装载");

        // 实体落在生成的地面上(顶面 world y = -60):经终末检查点快照验证。
        // 驱动 new_sim 走 config.shard_sim_config → 凝固桥**默认开启**:实体
        // 落稳约 20 拍后经 P2′(速度+位移门)凝固成方块(合法落定,非误凝固)
        // ——entity_state 变 None。故断言二选一:仍是实体(顶面上方驻留带)或
        // 已凝固(实体出生列上出现非空气方块)。两者都只有"地面真装载并被
        // 物理支撑"才会发生。
        let bundle = latest_bundle(&_checkpoints);
        let bytes = bundle
            .snapshot
            .per_shard
            .get(&shard_id(1, 0))
            .expect("观察者分片已创建并进入快照");
        let restored =
            ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        let entity = EntityId::from_parts(1, 1);
        match restored.entity_state(entity) {
            Some(state) => {
                assert!(
                    state.translation.y > GROUND_TOP_Y && state.translation.y < GROUND_TOP_Y + 1.5,
                    "实体仍为实体:应驻留在地面顶面上方:y = {}(顶面 {GROUND_TOP_Y})",
                    state.translation.y
                );
            }
            None => {
                // 凝固落点 = 实体出生 x/z 的方块列(spawn x=200.5,z=8.5 →
                // 方块 (200,·,8));凝固桥只向 air 格写方块 → 该列必有非空气。
                let mut column_has_block = false;
                for y in types::WORLD_Y_MIN..=types::WORLD_Y_MAX {
                    if !restored
                        .voxel()
                        .get_block(types::BlockPos::new(200, y, 8))
                        .is_air()
                    {
                        column_has_block = true;
                        break;
                    }
                }
                assert!(
                    column_has_block,
                    "实体已凝固:出生列 (x=200, z=8) 应出现凝固方块(地面真装载且被支撑)"
                );
            }
        }
    }

    /// 测试清单 2 `chunk_unload_persisted_via_worker`:观察者远离 → 卸载请求 →
    /// put_chunks 落库(重开 store 验证;mem 后端经共享句柄读回)。
    #[tokio::test(start_paused = true)]
    async fn chunk_unload_persisted_via_worker() {
        // 读回句柄:驱动持 mem 后端的 Box,测试侧用共享句柄直接查 MemChunkStore。
        #[derive(Clone, Default)]
        struct SharedChunks(Arc<Mutex<storage::mem::MemChunkStore>>);

        impl storage::traits::ChunkStore for SharedChunks {
            fn put_batch(
                &mut self,
                batch: &[(storage::traits::ChunkKey, voxel::Chunk)],
            ) -> Result<(), storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").put_batch(batch)
            }
            fn get(
                &self,
                key: storage::traits::ChunkKey,
            ) -> Result<Option<voxel::Chunk>, storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").get(key)
            }
            fn len(&self) -> Result<usize, storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").len()
            }
            fn is_empty(&self) -> Result<bool, storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").is_empty()
            }
        }

        let mut config = test_config(600);
        // 激活滞回:观察者瞬移远超卸载半径 → 下拍必出卸载请求。冷却关(瞬移
        // 立即生效);预算调大——卸载候选按"最近观察者距离"升序与远邻域装载
        // 候选同队竞争(瞬移后卸载候选距 ~5 万 m 排在装载候选 ~12m 之后),
        // 默认预算 2 会被装载吃光、卸载恒饿死(激活策略机实测语义);预算给足
        // (卸载 2 + 远邻域 12)让卸载请求出队。
        config.activation.cooldown_secs = 0;
        config.swap_budget_chunks_per_tick = 14;
        let session = sim_api::session::Session::new(config.session_config());
        let chunks = SharedChunks::default();
        let (tx, rx) = tokio::sync::mpsc::channel(16);
        let checkpoints = SharedCheckpoints::default();
        let store = storage::worker::StoreWorker::new(
            Box::new(chunks.clone()),
            Box::new(checkpoints.clone()),
        );
        let (provider, new_sim) = wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        });
        let driver = tokio::spawn(run(DriverArgs {
            config,
            session,
            store,
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
        }));

        // 进区(分片 (1,0))→ 等装载 → 瞬移远走(分片 (50,0),超卸载半径)。
        inject_move(&tx, Vec3::new(200.0, 0.0, 8.0)).await;
        advance_ticks(5).await;
        inject_move(&tx, Vec3::new(50_000.0, 0.0, 8.0)).await;
        advance_ticks(5).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(outcome.chunks_loaded >= 1, "前置:先有装载");
        assert!(outcome.chunks_unloaded >= 1, "观察者远离 → chunk 被卸载");

        // 重开 store 验证:mem 后端无"重开",等价口径 = 卸载值确实经 worker
        // 落库(共享句柄可见),且值与生成器初生值一致(未被修改)。
        let key = storage::traits::ChunkKey {
            world: WorldId(0),
            dim: DimensionId(0),
            pos: types::ChunkPos::new(12, 0),
        };
        let stored =
            storage::traits::ChunkStore::get(&*chunks.0.lock().expect("chunk 锁毒化"), key)
                .expect("读回不失败")
                .expect("卸载的 chunk 已落库");
        let expected = voxel::WorldGenerator::generate(
            &voxel::Superflat::classic(),
            types::WorldSeed(0),
            key.pos,
        );
        assert_eq!(stored, expected, "落库值 = 卸载时的 chunk(初生值)");
    }

    /// 测试清单 3 `despawn_flow_with_barrier`:小 despawn 窗口下,空片无票到期
    /// → 候选 → unload_shard → put + flush ack 屏障 → outcome.shards_despawned
    /// == 1;有实体的分片不 despawn。
    #[tokio::test(start_paused = true)]
    async fn despawn_flow_with_barrier() {
        let mut config = test_config(600);
        config.despawn_after_secs = 0; // despawn_after_ticks = 0 → 空片当拍即候选
        // 预注册两个分片:A(0,0) 恒空;B(0,1) 带实体。无观察者注入(命令路径)。
        // 注意:无观察者 = 编排零动作(无票判定回退),但 despawn 候选判定在
        // 调度器 tick 内(与驱动编排无关)——A 满足空+无票+idle 到期。
        let mut session = sim_api::session::Session::new(config.session_config());
        session.register(sim(0, 0));
        let mut sim_b = sim(0, 1);
        sim_b.apply_prefix(&[messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 0,
            payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 1),
                state: EntityState {
                    translation: Vec3::new(8.0, 3.0, 136.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            }),
        }]);
        session.register(sim_b);

        let (tx, rx) = tokio::sync::mpsc::channel(16);
        let checkpoints = SharedCheckpoints::default();
        let store = storage::worker::StoreWorker::new(
            Box::new(MemChunkStore::new()),
            Box::new(checkpoints.clone()),
        );
        let (provider, new_sim) = wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        });
        let driver = tokio::spawn(run(DriverArgs {
            config,
            session,
            store,
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
        }));

        advance_ticks(3).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.shards_despawned, 1, "恰 A(空片无票)被 despawn");
        assert!(outcome.ticks_run >= 3, "前置:跑过 despawn 窗口");

        // B(带实体)仍在:终末检查点快照含 B、不含 A。
        let bundle = latest_bundle(&checkpoints);
        assert!(
            !bundle.snapshot.per_shard.contains_key(&shard_id(0, 0)),
            "despawn 的 A 不在终末快照"
        );
        assert!(
            bundle.snapshot.per_shard.contains_key(&shard_id(0, 1)),
            "有实体的 B 不 despawn"
        );
    }

    /// 测试清单 4 `move_observer_command_routed`:DriverCommand::MoveObserver →
    /// session 观察者表更新(两个口径:命令日志含 SetObserverPosition = 已盖章
    /// 路由;编排 ① 从 observers() 视图 ensure 出新分片 = 表已更新且驱动可见)。
    #[tokio::test(start_paused = true)]
    async fn move_observer_command_routed() {
        let config = test_config(600);
        let session = sim_api::session::Session::new(config.session_config());
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        let pos = Vec3::new(200.0, 0.0, 8.0);
        inject_move(&tx, pos).await;
        advance_ticks(2).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(outcome.ticks_run >= 2, "前置:拍已推进");

        // 命令日志含 SetObserverPosition(盖章入日志 = 已路由进 session)。
        let bundle = latest_bundle(&checkpoints);
        let log_again =
            sim_api::log::CommandLog::from_bytes(&bundle.log_bytes).expect("自有编码必可解码");
        assert!(
            log_again
                .entries()
                .iter()
                .any(|e| e.command == messaging::SimCommand::SetObserverPosition { position: pos }),
            "MoveObserver 已入命令日志"
        );

        // 观察者表更新(任务书口径):编排 ① 读 observers() 视图 ensure 出分片
        // ——表未更新则 shards_spawned 必为 0;恰 1 即表已更新且驱动可见。
        assert_eq!(
            outcome.shards_spawned, 1,
            "观察者表更新且驱动 ensure 出恰好 1 个新分片"
        );
    }

    /// 测试清单 5 `outcome_counters_reflect_activity`:计数字段与场景一致——
    /// 走一个 进区装载 → 远走卸载 → A 分片 despawn 的组合场景,四字段全非零
    /// 且 despawn 恰 1。
    #[tokio::test(start_paused = true)]
    async fn outcome_counters_reflect_activity() {
        let mut config = test_config(600);
        config.despawn_after_secs = 0;
        // 同 chunk_unload 测试:预算给足让远走后的卸载请求出队(否则被远邻域
        // 装载候选饿死);冷却关让瞬移立即生效。
        config.activation.cooldown_secs = 0;
        config.swap_budget_chunks_per_tick = 14;
        let session = sim_api::session::Session::new(config.session_config());
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 进区装载(分片 (1,0))→ 远走卸载 → despawn(空+无票,idle 窗口 0)。
        inject_move(&tx, Vec3::new(200.0, 0.0, 8.0)).await;
        advance_ticks(4).await;
        inject_move(&tx, Vec3::new(50_000.0, 0.0, 8.0)).await;
        advance_ticks(4).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(outcome.shards_spawned >= 1, "ensure 建片计数");
        assert!(outcome.chunks_loaded >= 1, "装载计数");
        assert!(outcome.chunks_unloaded >= 1, "卸载计数");
        assert_eq!(outcome.shards_despawned, 1, "离开后唯一分片 despawn");
    }

    /// 测试清单 6 `no_observers_no_orchestration`:无观察者时编排零动作
    /// (既有行为不变,回归):预注册分片跑 10 拍,四个计数全 0。
    #[tokio::test(start_paused = true)]
    async fn no_observers_no_orchestration() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        advance_ticks(10).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.ticks_run, 10, "前置:10 拍");
        assert_eq!(outcome.shards_spawned, 0, "无观察者:不建片");
        assert_eq!(outcome.shards_despawned, 0, "无观察者:不 despawn");
        assert_eq!(outcome.chunks_loaded, 0, "无观察者:不装载");
        assert_eq!(outcome.chunks_unloaded, 0, "无观察者:不卸载");
    }
}
