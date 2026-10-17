//! app 终极验收(任务 04):真 redb 库上的断点续跑端到端 + toml 文件覆写 +
//! 告警接线。与实现(glm,任务 01-03)交叉执笔(README 模型分工)。
//!
//! 工程细节(任务书):每测试独立 `tempfile::tempdir()`,`Config.data_dir`
//! 指向它;run 用 `#[tokio::test(start_paused = true)]`(README 决策 10);
//! 续跑链中前一段完整退出(`run` 的退出路径含 store.shutdown,redb 文件锁已
//! 释放)再开下一段——串行 await;实体/地形构造照 two_shards.rs 帮助函数风格
//! (凝固桥关闭、`SimpleRegistry::builtin`)。

use std::path::Path;

use app::config::Config;
use app::driver::{CheckpointBundle, DriverArgs, DriverCommand, bootstrap, run};
use messaging::SimCommand;
use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
use storage::mem::{MemCheckpointStore, MemChunkStore};
use storage::redb::RedbStore;
use storage::traits::CheckpointStore;
use storage::worker::StoreWorker;
use types::glam::Vec3;
use types::{
    BlockId, BlockPos, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos,
    Tick, WorldId,
};
use voxel::{Chunk, SimpleRegistry};

// ---------------------------------------------------------------------------
// 帮助函数(照 two_shards.rs / driver.rs 单测风格)
// ---------------------------------------------------------------------------

/// 测试用分片身份(与 sim-api/shard 测试同风格)。
fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// registry 工厂(bootstrap 逐分片注入,任务书 01 决策:registry 不序列化)。
fn new_registry() -> impl Fn() -> Box<dyn voxel::BlockRegistry> {
    || Box::new(SimpleRegistry::builtin())
}

/// 测试配置:独立临时目录为 data_dir、checkpoint 间隔 5、2 worker、dt 默认 1/60。
fn disk_config(dir: &Path, checkpoint_interval_ticks: u64) -> Config {
    Config {
        data_dir: dir.to_path_buf(),
        worker_count: 2,
        checkpoint_interval_ticks,
        ..Config::default()
    }
}

/// 顶层实心面在 world y = top_ly - 64 的地形 chunk(照 two_shards.rs 帮助函数;
/// top_ly=200 → 顶面 world y = 136)。
fn ground_chunk(top_ly: usize) -> Chunk {
    let mut c = Chunk::air();
    for y in 0..top_ly {
        for x in 0..16 {
            for z in 0..16 {
                c.set(x, y, z, BlockId(1));
            }
        }
    }
    c
}

/// 带地面 chunk 的分片 (0,0),凝固桥关闭(任务书工程细节)。
fn ground_sim() -> ShardSim {
    let mut cfg = ShardSimConfig::new(shard_id(0, 0));
    cfg.solidify = SolidifyConfig {
        enabled: false,
        ..SolidifyConfig::default()
    };
    let mut s = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
    s.load_chunk(ChunkPos::new(0, 0), ground_chunk(200));
    s
}

/// SpawnDynamic 命令:地面 (0,0) 顶面(136)上方,凝固桥关闭 → 实体恒为实体。
fn spawn_cmd(entity: EntityId) -> SimCommand {
    SimCommand::SpawnDynamic {
        entity,
        state: EntityState {
            translation: Vec3::new(8.5, 137.0, 8.5),
            ..EntityState::default()
        },
        desc: EntityDesc::default(),
    }
}

/// 暂停时间下推进 N 拍:照 driver.rs 单测同款——测试任务 `sleep(dt)` 靠
/// auto-advance 推进,sleep 与驱动 interval 同刻到期 → 每 sleep 恰一拍。
async fn advance_ticks(ticks: u32) {
    let dt = std::time::Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        // sleep 醒后让驱动跑完本拍处理(tick 分支)再排下一拍。
        tokio::task::yield_now().await;
    }
}

/// 无注入的一段驱动:spawn run → 推进 ticks 拍 → Shutdown → 收 outcome。
/// 命令发送端在推进期间保持存活(通道关闭 = 关机,不能提前 drop)。
/// 任务 04 起:用 wire_provider_and_new_sim 装配 provider/new_sim(无观察者
/// 场景编排零动作,行为与历史一致)。
async fn drive(
    config: Config,
    session: sim_api::session::Session,
    log: sim_api::log::CommandLog,
    store: StoreWorker,
    ticks: u32,
) -> app::driver::DriverOutcome {
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(16);
    let (provider, new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let driver = tokio::spawn(run(DriverArgs {
        config,
        session,
        store,
        log,
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
    }));
    advance_ticks(ticks).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    driver.await.expect("驱动不 panic")
}

/// 从磁盘 checkpoint 库读回最新检查点并解码为 bundle(段已完整退出、锁已释放)。
fn latest_bundle_from_disk(dir: &Path) -> CheckpointBundle {
    let store = RedbStore::open(&dir.join("checkpoints.redb")).expect("打开 checkpoint 库");
    let (tick, blob) = store
        .read_latest()
        .expect("读最新检查点")
        .expect("必有检查点");
    let (bundle, _len): (CheckpointBundle, usize) =
        bincode::serde::decode_from_slice(&blob, bincode::config::standard())
            .expect("驱动自有编码必可解码");
    assert_eq!(
        bundle.snapshot.tick, tick,
        "库内元组 tick 与 bundle 快照拍一致"
    );
    bundle
}

// ---------------------------------------------------------------------------
// 测试
// ---------------------------------------------------------------------------

/// 测试清单 1:真 redb 断点续跑链——bootstrap(空目录,续跑拍 None)→ 跑 7 拍
/// (interval=5 → 期中 5 + 终末 7)→ 段完整退出;同目录再 bootstrap → 续跑起始
/// 拍 == 7 → 跑 3 拍 → 再退出再续 → 起始拍 == 10(链式续跑)。
#[tokio::test(start_paused = true)]
async fn resume_from_redb_checkpoint_continues_ticks() {
    let dir = tempfile::tempdir().expect("创建临时目录");
    let config = disk_config(dir.path(), 5);

    // —— 段 1:全新目录,resume_from 为 None,跑 7 拍 ——
    let (session, log, store, resume_from) =
        bootstrap(&config, new_registry()).expect("装配空目录");
    assert_eq!(resume_from, None, "空目录:无检查点,全新会话");
    let outcome1 = drive(config.clone(), session, log, store, 7).await;
    assert_eq!(outcome1.ticks_run, 7, "段 1 跑 7 拍");
    assert_eq!(
        outcome1.checkpoints_written,
        vec![Tick(5), Tick(7)],
        "interval=5:期中 5 + 终末 7"
    );

    // —— 段 2:同目录续跑,起始拍对齐段 1 终末快照;再跑 3 拍 ——
    let (session2, log2, store2, resume_from2) =
        bootstrap(&config, new_registry()).expect("续跑装配");
    assert_eq!(resume_from2, Some(Tick(7)), "续跑起始拍 == 段 1 终末 7");
    let outcome2 = drive(config.clone(), session2, log2, store2, 3).await;
    assert_eq!(outcome2.ticks_run, 3, "段 2 跑 3 拍");
    assert_eq!(
        outcome2.checkpoints_written,
        vec![Tick(10)],
        "3 拍不足期中间隔,仅终末(全局拍 10)"
    );

    // —— 段 3:再续跑,起始拍 == 10(链式续跑)——
    let (session3, _log3, store3, resume_from3) =
        bootstrap(&config, new_registry()).expect("二次续跑装配");
    assert_eq!(resume_from3, Some(Tick(10)), "链式续跑:起始拍 == 10");
    assert_eq!(session3.tick_now(), Tick(10), "会话拍号与续跑起点一致");
    session3.shutdown();
    store3.shutdown().expect("正常关停");
}

/// 测试清单 2:首段注册带地形分片 + SpawnDynamic 跑 7 拍关停;续跑后向同分片
/// 发 SetBlock → 生效(实体经快照恢复可见,终态快照里 SetBlock 落块、地基原块
/// 未动、实体仍在)。
#[tokio::test(start_paused = true)]
async fn commands_after_resume_apply_to_restored_shards() {
    let dir = tempfile::tempdir().expect("创建临时目录");
    let config = disk_config(dir.path(), 5);
    let shard = shard_id(0, 0);
    let entity = EntityId::from_parts(1, 1);

    // —— 段 1:带地形分片 + SpawnDynamic,跑 7 拍关停 ——
    let (mut session, log, store, resume_from) = bootstrap(&config, new_registry()).expect("装配");
    assert_eq!(resume_from, None);
    session.register(ground_sim());
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(16);
    let (provider, new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let driver = tokio::spawn(run(DriverArgs {
        config: config.clone(),
        session,
        store,
        log,
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
    }));
    tx.send(DriverCommand::Send {
        shard,
        command: spawn_cmd(entity),
    })
    .await
    .expect("驱动仍存活");
    advance_ticks(7).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome1 = driver.await.expect("驱动不 panic");
    assert_eq!(outcome1.ticks_run, 7);
    assert_eq!(outcome1.checkpoints_written, vec![Tick(5), Tick(7)]);

    // —— 续跑:恢复的分片与实体在快照里可见 ——
    let (session2, log2, store2, resume_from2) =
        bootstrap(&config, new_registry()).expect("续跑装配");
    assert_eq!(resume_from2, Some(Tick(7)));
    assert_eq!(session2.shard_ids(), vec![shard], "分片经快照恢复注册");
    assert!(
        session2.entity_state(shard, entity).is_some(),
        "实体经快照恢复可见(凝固桥关闭,恒为实体)"
    );

    // —— 续跑后向同分片发 SetBlock,跑 1 拍(全局第 7 拍)生效 ——
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(16);
    let (provider, new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store2, new_registry());
    let driver = tokio::spawn(run(DriverArgs {
        config: config.clone(),
        session: session2,
        store: store2,
        log: log2,
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
    }));
    tx.send(DriverCommand::Send {
        shard,
        command: SimCommand::SetBlock {
            pos: BlockPos::new(8, 100, 8),
            block: BlockId(2),
        },
    })
    .await
    .expect("驱动仍存活");
    advance_ticks(1).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome2 = driver.await.expect("驱动不 panic");
    assert_eq!(outcome2.ticks_run, 1);
    assert_eq!(
        outcome2.checkpoints_written,
        vec![Tick(8)],
        "终末检查点在全局拍 8"
    );

    // —— 磁盘读回终末快照:命令确实在恢复后的分片上生效 ——
    let bundle = latest_bundle_from_disk(dir.path());
    assert_eq!(bundle.snapshot.tick, Tick(8));
    let bytes = bundle
        .snapshot
        .per_shard
        .get(&shard)
        .expect("快照含分片 (0,0)");
    let restored = ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
    assert_eq!(
        restored.voxel().get_block(BlockPos::new(8, 100, 8)),
        BlockId(2),
        "续跑后 SetBlock 生效:恢复的分片上 (8,100,8) = dirt"
    );
    assert_eq!(
        restored.voxel().get_block(BlockPos::new(8, 50, 8)),
        BlockId(1),
        "地基未动的方块仍是 stone(改动只有这一处)"
    );
    assert!(
        restored.entity_state(entity).is_some(),
        "续跑后 entity_state 仍 Some:实体一路活在快照链里"
    );
}

/// 测试清单 3:写临时 toml(只覆写 worker_count 与 checkpoint_interval_ticks)→
/// `Config::load(Some(path))` → 两字段生效、其余默认。
#[test]
fn toml_file_on_disk_overrides_config() {
    let dir = tempfile::tempdir().expect("创建临时目录");
    let path = dir.path().join("config.toml");
    std::fs::write(&path, "worker_count = 9\ncheckpoint_interval_ticks = 42\n")
        .expect("写临时配置文件");

    let loaded = Config::load(Some(&path)).expect("自有文件必可加载");
    let expected = Config {
        worker_count: 9,
        checkpoint_interval_ticks: 42,
        ..Config::default()
    };
    assert_eq!(loaded, expected, "两字段覆写生效,其余保持默认");
    // 覆写点单独再钉一次(等值语义直接可见);"其余默认"由上面的整体等式覆盖。
    assert_eq!(loaded.worker_count, 9);
    assert_eq!(loaded.checkpoint_interval_ticks, 42);
}

/// 测试清单 4:全新目录 bootstrap → 续跑起始拍 None,Session 空(0 分片、拍号 0)。
#[test]
fn fresh_bootstrap_without_store_starts_at_zero() {
    let dir = tempfile::tempdir().expect("创建临时目录");
    let config = disk_config(dir.path(), 5);

    let (session, log, store, resume_from) =
        bootstrap(&config, new_registry()).expect("装配空目录");
    assert_eq!(resume_from, None, "全新目录:无检查点可续");
    assert_eq!(session.tick_now(), Tick::ZERO, "拍号从 0 起");
    assert!(session.shard_ids().is_empty(), "Session 空:0 分片");
    assert!(log.is_empty(), "命令日志为空");
    session.shutdown();
    store.shutdown().expect("正常关停");
}

/// 测试清单 5:阈值调 0.001ms 跑 60+ 拍(满 60 滑窗)→
/// `outcome.alert_active_at_exit == true`——证明驱动真的在喂 telemetry。
#[tokio::test(start_paused = true)]
async fn telemetry_wired_into_driver_outcome() {
    // 阈值 0.001ms(1µs):带分片的整拍(线程屏障 + 物理步进)必然越过它。
    let config = Config {
        shard_alert_ms: 0.001,
        checkpoint_interval_ticks: 600,
        worker_count: 2,
        ..Config::default()
    };
    let mut session = sim_api::session::Session::new(config.session_config());
    session.register(ground_sim());
    let shard = shard_id(0, 0);
    let entity = EntityId::from_parts(1, 1);

    // mem 后端:本测试只验证告警接线,不涉磁盘。
    let store = StoreWorker::new(
        Box::new(MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(16);
    let (provider, new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
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
    tx.send(DriverCommand::Send {
        shard,
        command: spawn_cmd(entity),
    })
    .await
    .expect("驱动仍存活");
    // 睡 68 拍:满 60 滑窗有余量。sleep/interval 在暂停时钟下的相位锁定在
    // 50+ 拍后不保证严格 1:1(clock.rs 警告;实测长程 sleep 会偶发漏相位,
    // 驱动每拍至多一发是 Skip 语义保证)——故断言下界 ≥ 60 而非精确相等。
    advance_ticks(68).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome = driver.await.expect("驱动不 panic");

    assert!(
        outcome.ticks_run >= 60,
        "60+ 拍:滑窗必满,实际 ticks_run = {}",
        outcome.ticks_run
    );
    assert!(
        outcome.alert_active_at_exit,
        "阈值 0.001ms × 满窗:退出时告警必激活(驱动在喂 telemetry)"
    );
}
