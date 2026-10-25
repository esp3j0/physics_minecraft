//! REQ-008 验收用例(e2e 部分;总架构师编写,issue #16)。
//!
//! 约定同其他验收文件:实施方不得修改断言,完成后只去掉对应单元用例上的 `#[ignore]`。
//! 夹具照 `viewer_client.rs`:真驱动 + 真网络 + `ViewerClient`(独立线程,不开窗),暂停时钟。

use std::sync::Arc;
use std::sync::mpsc as std_mpsc;
use std::time::Duration;

use app::config::Config;
use app::driver::{DriverArgs, DriverCommand, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use physics_sim::ShardSim;
use sim_api::log::CommandLog;
use sim_api::session::Session;
use storage::mem::MemCheckpointStore;
use storage::worker::StoreWorker;
use tokio::sync::mpsc;
use types::ShardId;
use types::glam::Vec3;
use viewer::client::{ViewEvent, ViewerClient};
use voxel::SimpleRegistry;

/// 暂停时间下推进 N 拍。
async fn advance_ticks(ticks: u32) {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

/// 起驱动 + 网络(观察节奏:增量每 `interval` 拍、全量每 `full` 拍)。返回驱动命令端、
/// 驱动句柄与监听地址。
async fn start_server(
    interval: u64,
    full: u64,
) -> (
    mpsc::Sender<DriverCommand>,
    tokio::task::JoinHandle<app::driver::DriverOutcome>,
    std::net::SocketAddr,
) {
    let config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 600,
        ..Config::default()
    };
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 600,
        observe_interval_ticks: interval,
        full_snapshot_interval_ticks: full,
        pump: PumpConfig::default(),
    };
    let cadence = ObserveCadence::from(&net_cfg);
    let (up_tx, up_rx) = mpsc::channel::<NetUp>(256);
    let hub = Arc::new(OutboundHub::new());
    let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
    let addr = serve(net_cfg, up_tx, bcast_rx, hub.clone())
        .await
        .expect("监听成功");
    std::mem::forget(bcast_tx);
    let session = Session::new(config.session_config());
    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    let (provider, _) = app::driver::wire_provider_and_new_sim(config.clone(), &store, || {
        Box::new(SimpleRegistry::builtin())
    });
    let sim_source = config.clone();
    let new_sim = Box::new(move |shard: ShardId| {
        ShardSim::new(
            sim_source.shard_sim_config(shard),
            Box::new(SimpleRegistry::builtin()),
        )
    });
    let driver = tokio::spawn(run(DriverArgs {
        config,
        session,
        store,
        log: CommandLog::new(),
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: Some(ObserveLink {
            up: up_rx,
            hub,
            cadence,
        }),
        spawn_next_seq: 0,
    }));
    (tx, driver, addr)
}

/// AC-S5(修订 2,D4):世界里没有任何实体时,订阅并上报观察者位置的客户端仍按全量周期
/// 收到实体状态包(全量、空表),拍号严格递增。viewer 脚本模式靠它得知服务器拍号。
///
/// 会变红的实现:可见实体为空时连全量包也不发(当前行为)——空场景里客户端永远不知道服务器
/// 推进到了哪一拍,脚本的等待步骤卡死;也无法用空的全量包清掉残留的幽灵实体。
#[tokio::test(start_paused = true)]
#[ignore = "REQ-008 单元 03 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
async fn acc_s5_empty_world_still_streams_tick_heartbeat() {
    let (tx, driver, addr) = start_server(10, 60).await;
    let (events, control) = ViewerClient::spawn(addr, 2).expect("连接真服务器");
    control.move_observer(Vec3::new(8.0, -50.0, 8.0));

    let mut ticks: Vec<u64> = Vec::new();
    for _ in 0..400 {
        loop {
            match events.try_recv() {
                Ok(ViewEvent::Entities {
                    tick,
                    states,
                    removed,
                    full,
                }) => {
                    assert!(
                        states.is_empty() && removed.is_empty(),
                        "空世界:实体包应为空表"
                    );
                    assert!(full, "空世界只有全量包(增量无变化不发)");
                    ticks.push(tick.0);
                }
                Ok(_) => {}
                Err(std_mpsc::TryRecvError::Empty) => break,
                Err(std_mpsc::TryRecvError::Disconnected) => panic!("连接意外断开"),
            }
        }
        if ticks.len() >= 3 {
            break;
        }
        std::thread::sleep(Duration::from_millis(1));
        advance_ticks(1).await;
    }
    assert!(
        ticks.len() >= 3,
        "约 400 拍内应收到至少 3 个全量心跳包,实收 {ticks:?}"
    );
    assert!(
        ticks.windows(2).all(|w| w[0] < w[1]),
        "拍号严格递增:{ticks:?}"
    );
    assert!(
        ticks.windows(2).all(|w| w[1] - w[0] <= 60),
        "相邻心跳间隔不超过一个全量周期:{ticks:?}"
    );

    control.goodbye();
    tx.send(DriverCommand::Shutdown).await.expect("驱动存活");
    driver.await.expect("驱动不 panic");
}
