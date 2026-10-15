//! biased select(关机>tick>命令)、MissedTickBehavior::Skip、checkpoint 节奏与断点续跑装配。
//!
//! 驱动形状照 ferrumc v2 driver(调研 §2):biased select 优先级 = 关机 > 到期
//! tick > 命令——命令洪水永远饿不死模拟;`MissedTickBehavior::Skip` 让超载降拍
//! 而不补帧。checkpoint = 快照拍 + 此后命令日志原子一体(README 决策 4),只存增量。

use std::time::Duration;

use serde::{Deserialize, Serialize};
use storage::traits::CheckpointStore;

use crate::config::{Config, ConfigError};

/// 驱动指令(外部缝;REQ-003 网络将复用同一形状)。
#[derive(Debug)]
pub enum DriverCommand {
    Send {
        shard: types::ShardId,
        command: messaging::SimCommand,
    },
    Shutdown,
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
    let mut log = sim_api::log::CommandLog::default();
    for entry in entries {
        log.append(entry);
    }
    log.to_bytes()
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
        let args = DriverArgs {
            config,
            session,
            store: storage::worker::StoreWorker::new(
                Box::new(MemChunkStore::new()),
                Box::new(checkpoints.clone()),
            ),
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
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
}
