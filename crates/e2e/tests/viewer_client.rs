//! REQ-004A ACV2 端到端冒烟(任务书 02):经 viewer::client(不开窗)连真服务器。
//!
//! 装配 harness 照抄 tests/network.rs(真驱动 + 真网络,observe 三缝真接,
//! 127.0.0.1:0 实际端口,README 决策 10);客户端换为 viewer::client 的
//! [`ViewerClient`](viewer::client::ViewerClient)(独立线程连接循环 + std mpsc
//! 事件桥,README 决策 3)。时钟模型:暂停虚拟时钟只驱动服务端(泵/观察
//! cadence 走虚拟时间),viewer client 线程是真时间——空轮询每轮让出 1ms
//! 真实片 + 1 虚拟拍,两侧同时推进。渲染窗口/相机行为**不测试**(README
//! 决策 2;ACV4 手动清单,禁造假断言)。

use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::mpsc as std_mpsc;
use std::time::Duration;

use app::config::Config;
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::{MAX_LOADS_PER_PUMP, PumpConfig, chebyshev, view_square};
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::{InputSource, SimCommand};
use physics_sim::{ShardSim, SolidifyConfig};
use sim_api::log::CommandLog;
use sim_api::session::Session;
use storage::mem::MemCheckpointStore;
use storage::worker::StoreWorker;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{
    BlockId, BlockPos, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos,
    WorldId,
};
use viewer::client::{ViewEvent, ViewerClient};
use voxel::{BlockRegistry, Chunk, SimpleRegistry, Superflat, WorldGenerator};

/// 测试通用配置:2 worker、checkpoint 疏(600,终末必写)、其余默认。
fn base_config() -> Config {
    Config {
        worker_count: 2,
        checkpoint_interval_ticks: 600,
        ..Config::default()
    }
}

/// 原点区(分片 (0,0))实体的身份:origin=0,与 command_shard 的信息性
/// 路由口径一致(origin 0 → ShardPos(0,0);network.rs 同款)。
fn eid0(n: u32) -> EntityId {
    EntityId::from_parts(0, n)
}

/// 测试用分片身份(network.rs 同款)。
fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// registry 工厂(world_loading / network.rs 同款)。
fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

/// 暂停时间下推进 N 拍(auto-advance;network.rs 同款)。
async fn advance_ticks(ticks: u32) {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

// ---------------------------------------------------------------------------
// 装配 harness:真驱动 + 真网络(observe 缝真接)+ 127.0.0.1:0 实际端口
// (network.rs 同款;客户端 = ViewerClient,无需内嵌 TestClient)
// ---------------------------------------------------------------------------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: tokio::task::JoinHandle<DriverOutcome>,
    hub: Arc<OutboundHub>,
    addr: SocketAddr,
}

/// 起驱动 + 网络(照抄 network.rs harness):observe 三缝真接;provider 走
/// 生产装配;凝固桥关、可选零重力;read_timeout_secs=600(keepalive 在
/// start_paused 下是**虚拟** 10s 会误踢——取舍同 network.rs 模块头,
/// keepalive 语义本体由 REQ-003 任务 01 覆盖)。
async fn harness(config: Config, pump: PumpConfig, observe: (u64, u64), zero_g: bool) -> Harness {
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 600,
        observe_interval_ticks: observe.0,
        full_snapshot_interval_ticks: observe.1,
        pump,
    };
    let cadence = ObserveCadence::from(&net_cfg);
    let (up_tx, up_rx) = mpsc::channel::<NetUp>(256);
    let hub = Arc::new(OutboundHub::new());
    let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
    let addr = serve(net_cfg, up_tx, bcast_rx, hub.clone())
        .await
        .expect("监听成功(127.0.0.1:0)");
    // 关机广播发送端随测试存续(accept 循环以其存活为常驻条件;driver
    // observe_harness 同款 std::mem::forget 手法)。
    std::mem::forget(bcast_tx);

    let session = Session::new(config.session_config());
    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    let (provider, _) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let sim_source = config.clone();
    let new_sim = Box::new(move |shard: ShardId| {
        let mut cfg = sim_source.shard_sim_config(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        if zero_g {
            cfg.gravity = Vec3::ZERO;
        }
        ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
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
            hub: hub.clone(),
            cadence,
        }),
        spawn_next_seq: 0,
    }));
    Harness {
        tx,
        driver,
        hub,
        addr,
    }
}

/// 收口:Shutdown → join 驱动(panic = 红)。
async fn shutdown(h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    h.driver.await.expect("驱动不 panic")
}

// ---------------------------------------------------------------------------
// 事件桥读取 helpers(std mpsc try_recv 排空;viewer 主循环同款,README 决策 3)
// ---------------------------------------------------------------------------

/// 真时间让片:viewer client 在独立 OS 线程跑(真 socket IO + 解码),虚拟
/// 时钟快进几乎不耗真实时间——空轮询每轮让出 1ms,保证外部线程被调度、
/// 数据落桥。
fn real_slice() {
    std::thread::sleep(Duration::from_millis(1));
}

/// 排空式收一帧:空则让片 + 推进 1 虚拟拍(服务端泵/观察 cadence 走虚拟
/// 时钟);None = 预算尽或通道枯竭(发送端 drop = client 连接循环收口)。
async fn next_event(events: &std_mpsc::Receiver<ViewEvent>, budget: u32) -> Option<ViewEvent> {
    for _ in 0..budget {
        match events.try_recv() {
            Ok(event) => return Some(event),
            Err(std_mpsc::TryRecvError::Empty) => {
                real_slice();
                advance_ticks(1).await;
            }
            Err(std_mpsc::TryRecvError::Disconnected) => return None,
        }
    }
    None
}

/// 排空等一个命中谓词的事件;`max_empty` = 连续空轮询上限(不匹配的
/// 事件照常消耗,不计数)。None = 上限触顶或通道枯竭。
async fn wait_event(
    events: &std_mpsc::Receiver<ViewEvent>,
    max_empty: u32,
    mut pred: impl FnMut(&ViewEvent) -> bool,
) -> Option<ViewEvent> {
    let mut empty = 0;
    loop {
        match events.try_recv() {
            Ok(event) => {
                empty = 0;
                if pred(&event) {
                    return Some(event);
                }
            }
            Err(std_mpsc::TryRecvError::Empty) => {
                empty += 1;
                if empty >= max_empty {
                    return None;
                }
                real_slice();
                advance_ticks(1).await;
            }
            Err(std_mpsc::TryRecvError::Disconnected) => return None,
        }
    }
}

/// 双时钟共同推进 n 轮(每轮 1 虚拟拍 + 1ms 真片):控制端指令经
/// 桥线程/连接循环真时间管道送达服务端后,留给驱动兑现编排。
async fn settle(n: u32) {
    for _ in 0..n {
        advance_ticks(1).await;
        real_slice();
    }
}

/// 收集 Chunk 事件到 want 个(非 chunk 事件跳过;不足原样带回,数量判定
/// 归调用方;network.rs collect_chunk_loads 同式)。
async fn collect_chunks(
    events: &std_mpsc::Receiver<ViewEvent>,
    want: usize,
    per_event_budget: u32,
) -> Vec<(ChunkPos, Chunk)> {
    let mut out = Vec::new();
    while out.len() < want {
        match next_event(events, per_event_budget).await {
            Some(ViewEvent::Chunk { pos, chunk }) => out.push((pos, chunk)),
            Some(_) => continue,
            None => break,
        }
    }
    out
}

/// 视距方形的 center-out 升序表(纯函数口径,断言基准;network.rs 同款)。
fn sorted_square(center: ChunkPos, view: u8) -> Vec<ChunkPos> {
    let mut v: Vec<ChunkPos> = view_square(center, view).into_iter().collect();
    v.sort_by_key(|p| (chebyshev(center, *p), p.x, p.z));
    v
}

// ---------------------------------------------------------------------------
// 测试清单(任务书 02;ACV2)
// ---------------------------------------------------------------------------

/// 测试清单 1(ACV2 主链路):真 bootstrap + serve → ViewerClient::spawn
/// (独立线程,不开窗)→ Welcome(服务器分配身份)→ chunk 流 center-out
/// 有限步(首泵恰 16、次泵补 9,位级 == Superflat 生成值)→ 控制端
/// MoveObserver → 新区中心 chunk (12,0) 经流到达且位级正确 → 驱动侧世界
/// 装载推进(outcome 计数)。
#[tokio::test(start_paused = true)]
async fn viewer_client_connects_streams_and_moves() {
    let config = base_config();
    // pump 200ms/catchup=1:「单泵恰 16」可观测口径(AC-N2 同款)。
    let pump = PumpConfig {
        interval_ms: 200,
        max_catchup_rounds: 1,
    };
    let h = harness(config.clone(), pump, (2, 60), false).await;

    let (events, control) =
        ViewerClient::spawn(h.addr, 2).expect("spawn 连接 127.0.0.1:0 真服务器");

    // 首事件 = Welcome(单客户端身份从 1 起;0 保留)。
    let welcome = wait_event(&events, 400, |e| matches!(e, ViewEvent::Welcome { .. }))
        .await
        .expect("真服务器必回 Welcome(入会序列 Hello→Subscribe 经 viewer client)");
    match welcome {
        ViewEvent::Welcome { player_id, .. } => {
            assert_eq!(player_id, InputSource(1), "身份 = 服务器分配(单客户端 = 1)");
        }
        other => unreachable!("谓词已限定 Welcome,实为 {other:?}"),
    }

    // chunk 流:center-out 有限步(首泵限额 16,序 = 切比雪夫距再 x,z)。
    let expected = sorted_square(ChunkPos::new(0, 0), 2);
    let first_pump = collect_chunks(&events, MAX_LOADS_PER_PUMP, 400).await;
    assert_eq!(
        first_pump.len(),
        MAX_LOADS_PER_PUMP,
        "首泵恰 16(实 {})",
        first_pump.len()
    );
    assert_eq!(
        first_pump.iter().map(|(p, _)| *p).collect::<Vec<_>>(),
        expected[..MAX_LOADS_PER_PUMP],
        "首泵 center-out 升序"
    );
    // 真数据源位级:中心 chunk == Superflat 生成值。
    let chunk_00 = &first_pump
        .iter()
        .find(|(p, _)| *p == ChunkPos::new(0, 0))
        .expect("中心 chunk 必在首泵")
        .1;
    assert_eq!(
        *chunk_00,
        Superflat::classic().generate(config.world_seed, ChunkPos::new(0, 0)),
        "Chunk = 真数据源生成值(位级 ==)"
    );
    // 次泵补齐余 9(视距 2 方形 25 chunk;仍 center-out)。
    let second_pump = collect_chunks(&events, 25 - MAX_LOADS_PER_PUMP, 400).await;
    assert_eq!(
        second_pump.iter().map(|(p, _)| *p).collect::<Vec<_>>(),
        expected[MAX_LOADS_PER_PUMP..],
        "次泵续装余 9,续序不变(多泵补齐)"
    );

    // 控制端 MoveObserver(经桥线程 → tokio 通道 → 连接循环写帧)→
    // 新区中心 chunk (12,0) 经流到达,位级 == 生成值。
    control.move_observer(Vec3::new(200.0, -58.0, 8.0));
    let new_center = ChunkPos::new(12, 0);
    let mut new_chunk = None;
    while new_chunk.is_none() {
        match next_event(&events, 600).await {
            Some(ViewEvent::Chunk { pos, chunk }) if pos == new_center => new_chunk = Some(chunk),
            Some(_) => continue,
            None => panic!("MoveObserver 后新区 chunk (12,0) 未到(预算尽/通道枯竭)"),
        }
    }
    assert_eq!(
        new_chunk.expect("上一循环判定存在"),
        Superflat::classic().generate(config.world_seed, new_center),
        "新区 chunk = 真数据源生成值(位级;MoveObserver 生效)"
    );

    // 驱动侧世界装载推进(观察者应用 → ensure → 装载经 provider 真路径)。
    settle(120).await;
    let outcome = shutdown(h).await;
    assert!(
        outcome.shards_spawned >= 1,
        "MoveObserver → 分片经驱动编排 ensure 创建"
    );
    assert!(
        outcome.chunks_loaded >= 1,
        "激活装载 + 流取数经 provider 真路径"
    );
}

/// 测试清单 2(观察流事件,位级正确):控制端就位后外部注入 SpawnDynamic +
/// SetBlock(驱动命令通道,README 决策 9 路径无关镜像)→
/// ViewEvent::Entities 含实体四向量 == 出生态(零重力绝对静止,位级)、
/// ViewEvent::Blocks 含逐格更新 == 写入值。
#[tokio::test(start_paused = true)]
async fn viewer_client_receives_entity_and_block_events() {
    // 零重力(AC-N4 同款口径):绝对静止才允许位级断言。
    let h = harness(base_config(), PumpConfig::default(), (2, 60), true).await;
    let (events, control) = ViewerClient::spawn(h.addr, 2).expect("spawn 连真服务器");
    wait_event(&events, 400, |e| matches!(e, ViewEvent::Welcome { .. }))
        .await
        .expect("Welcome 必达");

    // 观察者就位(原点区分片 ensure 是命令先决;控制端真时间路径 + settle)。
    control.move_observer(Vec3::new(8.0, 3.0, 8.0));
    settle(60).await;

    // 外部注入实体(origin=0 → 路由分片 (0,0),acn4/acn6 同款身份)。
    let e = eid0(1);
    let born = EntityState {
        translation: Vec3::new(10.0, 3.0, 10.0),
        ..EntityState::default()
    };
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnDynamic {
            entity: e,
            state: born,
            desc: EntityDesc::default(),
        },
    })
    .await
    .expect("驱动存活");

    // Entities 事件:含该实体且四向量位级 == 出生态(零重力静止;
    // removed 不含在场实体)。
    let got = wait_event(&events, 1500, |ev| {
        matches!(ev, ViewEvent::Entities { states, removed, .. }
            if states.iter().any(|(id, st)| *id == e && *st == born) && !removed.contains(&e))
    })
    .await
    .expect("SpawnDynamic 经协议回流 ViewEvent::Entities(位级)");
    match got {
        ViewEvent::Entities { states, .. } => {
            let (_, st) = states.iter().find(|(id, _)| *id == e).expect("含实体");
            assert_eq!(*st, born, "实体四向量位级 == 出生态");
        }
        other => unreachable!("谓词已限定 Entities,实为 {other:?}"),
    }

    // 外部注入方块变更 → Blocks 事件逐格 == (位级;格位在观察视距内)。
    let cell = (BlockPos::new(4, 3, 4), BlockId(1));
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SetBlock {
            pos: cell.0,
            block: cell.1,
        },
    })
    .await
    .expect("驱动存活");
    let blocks = wait_event(
        &events,
        1500,
        |ev| matches!(ev, ViewEvent::Blocks { updates } if updates.contains(&cell)),
    )
    .await
    .expect("SetBlock 经协议回流 ViewEvent::Blocks");
    match blocks {
        ViewEvent::Blocks { updates } => {
            assert!(updates.contains(&cell), "更新位级含注入格");
        }
        other => unreachable!("谓词已限定 Blocks,实为 {other:?}"),
    }

    let outcome = shutdown(h).await;
    assert!(outcome.ticks_run > 0, "场景确有推进");
}

/// 测试清单 3(清洁离场):Goodbye → Disconnected 事件(客户端侧收口)
/// → 事件通道枯竭(client 线程退出 drop 发送端)→ 服务器侧 Left 清理
/// (hub 注销口径)+ 他人在场不受影响(此处双连接:A 走 Goodbye,B 在场)。
#[tokio::test(start_paused = true)]
async fn viewer_client_clean_disconnect() {
    let h = harness(base_config(), PumpConfig::default(), (2, 60), false).await;

    // 双连接:A(viewer client,走 Goodbye)+ B(hub 在场见证人;用同一
    // ViewerClient 形态,验证互不影响)。
    let (events_a, control_a) = ViewerClient::spawn(h.addr, 1).expect("A spawn");
    let (events_b, _control_b) = ViewerClient::spawn(h.addr, 1).expect("B spawn");
    let id_a = match wait_event(&events_a, 400, |e| matches!(e, ViewEvent::Welcome { .. }))
        .await
        .expect("A Welcome 必达")
    {
        ViewEvent::Welcome { player_id, .. } => player_id,
        other => unreachable!("谓词已限定 Welcome,实为 {other:?}"),
    };
    let id_b = match wait_event(&events_b, 400, |e| matches!(e, ViewEvent::Welcome { .. }))
        .await
        .expect("B Welcome 必达")
    {
        ViewEvent::Welcome { player_id, .. } => player_id,
        other => unreachable!("谓词已限定 Welcome,实为 {other:?}"),
    };
    assert_eq!((id_a, id_b), (InputSource(1), InputSource(2)), "身份递增");
    settle(3).await;
    assert!(
        h.hub.subscribers().contains(&id_a) && h.hub.subscribers().contains(&id_b),
        "Subscribe 完成(Welcome 到手)即注册进出站 hub"
    );

    // A 主动 Goodbye → Disconnected 事件(reason = goodbye,client 收口路径)。
    control_a.goodbye();
    let disc = wait_event(&events_a, 400, |e| {
        matches!(e, ViewEvent::Disconnected { .. })
    })
    .await
    .expect("Goodbye → Disconnected 事件必达");
    match disc {
        ViewEvent::Disconnected { reason } => {
            assert_eq!(reason, "goodbye", "Goodbye 收口原因(client 侧)");
        }
        other => unreachable!("谓词已限定 Disconnected,实为 {other:?}"),
    }

    // client 连接循环收口:Disconnected 事件后 drop 发送端 → 通道枯竭
    // (先吃光缓冲帧再判,竞先窗口容忍)。
    let mut drained = false;
    for _ in 0..200 {
        match events_a.try_recv() {
            Err(std_mpsc::TryRecvError::Disconnected) => {
                drained = true;
                break;
            }
            Err(std_mpsc::TryRecvError::Empty) => {
                real_slice();
                advance_ticks(1).await;
            }
            Ok(_) => {}
        }
    }
    assert!(drained, "Disconnected 后 client 线程退出,事件通道枯竭");

    // 服务器侧 Left 清理:hub 注销 A;B 不受影响。
    let mut cleaned = false;
    for _ in 0..200 {
        advance_ticks(1).await;
        real_slice();
        if !h.hub.subscribers().contains(&id_a) {
            cleaned = true;
            break;
        }
    }
    assert!(cleaned, "Goodbye → Left 上行 → hub 注销(服务器侧清理)");
    assert!(
        h.hub.subscribers().contains(&id_b),
        "B 不受 A 离场影响(仍注册)"
    );

    shutdown(h).await;
}
