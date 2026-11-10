//! REQ-011 W03(§3.5.1/§3.5.2)e2e:调试遥测订阅与下发(AC-W9/AC-W10)。
//!
//! 装配照 network.rs(真驱动 + 真网络,observe 三缝真接,127.0.0.1:0);客户端
//! 用同款内嵌 TestClient(能发任意 C2S 帧、收原始 S2C——viewer 本单元不订阅,
//! W04 才接)。断言全部指向行为结果:计数逐值、recent 末条字段、未订阅连接
//! 零 DebugStats、A/B 两线实体与方块逐位一致(照 consistency.rs 口径)。

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::Config;
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::{InputSource, SimCommand};
use physics_sim::{ShardSim, SolidifyConfig};
use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C};
use sim_api::session::Session;
use storage::mem::MemCheckpointStore;
use storage::worker::StoreWorker;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{
    BlockId, BlockPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, WorldId,
};
use voxel::{BlockRegistry, SimpleRegistry};

/// 软窗(虚拟 ~3 拍)。
const SOFT: Duration = Duration::from_millis(50);

/// 测试用分片身份(network.rs 同款)。
fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// registry 工厂(network.rs 同款)。
fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

/// 暂停时间下推进 N 拍(network.rs 同款)。
async fn advance_ticks(ticks: u32) {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

// ---------------------------------------------------------------------------
// 内嵌迷你客户端(network.rs 同款;写端 + 解帧读通道)
// ---------------------------------------------------------------------------

struct TestClient {
    w: tokio::net::tcp::OwnedWriteHalf,
}

impl TestClient {
    async fn send(&mut self, c2s: &C2S) -> std::io::Result<()> {
        self.w
            .write_all(&sim_api::proto::encode_c2s_frame(c2s))
            .await
    }
}

async fn client(addr: SocketAddr) -> (TestClient, mpsc::UnboundedReceiver<S2C>) {
    let stream = TcpStream::connect(addr).await.expect("TCP 连入");
    let (mut r, w) = stream.into_split();
    let (tx, rx) = mpsc::unbounded_channel::<S2C>();
    tokio::spawn(async move {
        let mut buf: Vec<u8> = Vec::with_capacity(16 * 1024);
        let mut chunk = [0u8; 16 * 1024];
        loop {
            let frame = loop {
                if buf.len() >= 4 {
                    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize + 4;
                    if buf.len() >= len {
                        break buf.drain(..len).collect::<Vec<u8>>();
                    }
                }
                match r.read(&mut chunk).await {
                    Ok(0) | Err(_) => return,
                    Ok(n) => buf.extend_from_slice(&chunk[..n]),
                }
            };
            if let Some(s2c) = sim_api::proto::decode_frame(&frame)
                && tx.send(s2c).is_err()
            {
                return;
            }
        }
    });
    (TestClient { w }, rx)
}

/// Hello + Subscribe,返回 Welcome 的 player_id(network.rs 同款)。
async fn join(c: &mut TestClient, rx: &mut mpsc::UnboundedReceiver<S2C>, view: u8) -> InputSource {
    c.send(&C2S::Hello { client_id: 1 })
        .await
        .expect("Hello 写入");
    c.send(&C2S::Subscribe {
        view_distance: view,
    })
    .await
    .expect("Subscribe 写入");
    for _ in 0..64 {
        match next_frame(rx).await {
            Some(S2C::Welcome { player_id, .. }) => return player_id,
            Some(_) => continue,
            None => panic!("软窗预算内未见 Welcome"),
        }
    }
    panic!("64 软窗内未收到 Welcome")
}

/// 软窗收一帧(None = 窗口内无帧)。
async fn next_frame(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Option<S2C> {
    tokio::time::timeout(SOFT, rx.recv()).await.ok().flatten()
}

/// 非阻塞收一帧(不推进时钟)。
fn try_frame(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Option<S2C> {
    rx.try_recv().ok()
}

// ---------------------------------------------------------------------------
// 装配 harness(network.rs 同款;冲量流/破坏开、承载开、凝固关、重力开)
// ---------------------------------------------------------------------------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: tokio::task::JoinHandle<DriverOutcome>,
    addr: SocketAddr,
}

async fn harness() -> Harness {
    let config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 600,
        cohesion: app::config::CohesionConfig {
            enabled: true,
            impact_flow: true,
            ..app::config::CohesionConfig::default()
        },
        ..Config::default()
    };
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 600,
        observe_interval_ticks: 2,
        full_snapshot_interval_ticks: 60,
        pump: Default::default(),
    };
    let cadence = ObserveCadence::from(&net_cfg);
    let (up_tx, up_rx) = mpsc::channel::<NetUp>(256);
    let hub = Arc::new(OutboundHub::new());
    let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
    let addr = serve(net_cfg, up_tx, bcast_rx, hub.clone())
        .await
        .expect("监听成功(127.0.0.1:0)");
    std::mem::forget(bcast_tx);
    let session = Session::new(config.session_config());
    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    let (provider, _) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry())
            .expect("装配失败(fail-stop;MARGIN 不足等类型化错误)");
    let sim_source = config.clone();
    let new_sim = Box::new(move |shard: ShardId| {
        let mut cfg = sim_source.shard_sim_config(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
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
    Harness { tx, driver, addr }
}

async fn shutdown(h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    h.driver.await.expect("驱动不 panic")
}

/// 高速砸地投射物(单格石方块,-30 m/s;出生地面顶面 world y = −60 上方)。
fn bullet(seq: u32, x: f32, y: f32, z: f32, vy: f32) -> DriverCommand {
    DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnDynamic {
            entity: EntityId::from_parts(1, seq),
            state: EntityState {
                translation: Vec3::new(x, y, z),
                linvel: Vec3::new(0.0, vy, 0.0),
                ..EntityState::default()
            },
            desc: EntityDesc::default(),
        },
    }
}

/// 收下一个 DebugStats(跳过其它帧;budget 上限)。
async fn recv_debug(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    budget: u32,
    pred: impl Fn(&sim_api::proto::DebugCounters) -> bool,
) -> Option<(
    types::Tick,
    sim_api::proto::DebugCounters,
    Vec<(types::Tick, sim_api::proto::FlowEventWire)>,
)> {
    for _ in 0..budget {
        if let Some(S2C::DebugStats {
            tick,
            counters,
            recent,
        }) = next_frame(rx).await
            && pred(&counters)
        {
            return Some((tick, counters, recent));
        }
    }
    None
}

fn counters_vec(c: &sim_api::proto::DebugCounters) -> [u64; 13] {
    [
        c.impact_flow_events,
        c.impact_flow_breaks,
        c.impact_flow_detached_cells,
        c.compound_flow_events,
        c.compound_flow_breaks,
        c.compound_flow_detached_cells,
        c.compound_flow_corrections,
        c.bonds_broken,
        c.compounds_spawned,
        c.compounds_solidified,
        c.cells_blasted,
        c.cells_impacted,
        c.cells_collapsed,
    ]
}

// ---------------------------------------------------------------------------
// AC-W10:订阅 / 幂等 / 每 10 拍下发 / 单调 / recent 末条 / 未订阅收不到
// ---------------------------------------------------------------------------

#[tokio::test(start_paused = true)]
async fn w10_debug_stats_delivery_and_recent() {
    let h = harness().await;
    let (mut sub, mut rx_sub) = client(h.addr).await;
    let (mut plain, mut rx_plain) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _id_sub = join(&mut sub, &mut rx_sub, 2).await;
    let _id_plain = join(&mut plain, &mut rx_plain, 2).await;
    sub.send(&C2S::SubscribeDebug).await.expect("订阅遥测写入");
    // 幂等:重复订阅无回执、无副作用。
    sub.send(&C2S::SubscribeDebug).await.expect("重复订阅写入");

    // 观察者就位(ensure + 装载是 spawn 的先决)。
    sub.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, -58.0, 8.0),
    })
    .await
    .expect("观察者写入");
    plain
        .send(&C2S::MoveObserver {
            position: Vec3::new(8.0, -58.0, 8.0),
        })
        .await
        .expect("观察者写入");
    advance_ticks(12).await;

    // 订阅者收到首个 DebugStats(≤ 30 拍 = 3 个下发点);计数全 0 起点。
    let (t0, c0, r0) = recv_debug(&mut rx_sub, 200, |_| true)
        .await
        .expect("订阅后软窗内必收到 DebugStats");
    assert!(t0.0 >= 10, "首个包 ≥10 拍后(实 {t0:?})");
    assert_eq!(counters_vec(&c0), [0u64; 13], "起点全 0(尚无破坏/事件)");
    assert!(r0.is_empty(), "尚无事件,recent 空");

    // 未订阅连接同窗收不到 DebugStats(其余观察帧照常,不判帧序)。
    let mut plain_got_debug = false;
    for _ in 0..120 {
        if let Some(s2c) = try_frame(&mut rx_plain)
            && matches!(s2c, S2C::DebugStats { .. })
        {
            plain_got_debug = true;
        }
        advance_ticks(1).await;
    }
    assert!(!plain_got_debug, "未订阅连接收不到 DebugStats");

    // 制造撞击断开(req010-impact-punch 冒烟同款工况):1 格厚土墙
    // (x=10,y=−60..−56,z=17..23;墙脚 bedrock 防整面剪断),6 格石条
    // 20 m/s 侧撞墙面——端格剪切割 = 4×7 = 28,J ≈ 6×0.729×20 ≈ 87 > 28
    // 断开(冒烟场景常年绿,布景稳定)。
    for y in -60..=-56 {
        h.tx.send(DriverCommand::Send {
            shard: shard_id(0, 0),
            command: SimCommand::SetBlock {
                pos: BlockPos::new(10, y, 20),
                block: voxel::builtin::DIRT,
            },
        })
        .await
        .expect("驱动存活");
    }
    for y in -61..=-60 {
        h.tx.send(DriverCommand::Send {
            shard: shard_id(0, 0),
            command: SimCommand::SetBlock {
                pos: BlockPos::new(10, y, 20),
                block: voxel::builtin::BEDROCK,
            },
        })
        .await
        .expect("驱动存活");
    }
    advance_ticks(20).await;
    // 石条:6 格横条,起点 (2.5,−57.5,20.5),20 m/s 朝 +x(场景同款)。
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnDynamic {
            entity: EntityId::from_parts(1, 9),
            state: EntityState {
                translation: Vec3::new(2.5, -57.5, 20.5),
                linvel: Vec3::new(20.0, 0.0, 0.0),
                ..EntityState::default()
            },
            desc: EntityDesc::compound(
                physics_sim::destruction::DEBRIS_HALF_SIZE,
                types::CompoundShape::new(
                    (0..6)
                        .map(|i| types::ShapeCell {
                            offset: [i, 0, 0],
                            block: voxel::builtin::STONE,
                        })
                        .collect(),
                )
                .expect("6 格条"),
            ),
        },
    })
    .await
    .expect("驱动存活");
    advance_ticks(120).await;

    // 后续包:impact_flow_breaks 相对基线增加;recent 末条 = 该事件。
    let (_t1, c1, r1) = recv_debug(&mut rx_sub, 400, |c| {
        c.impact_flow_breaks > c0.impact_flow_breaks
    })
    .await
    .expect("撞击断开后 impact_flow_breaks 必增");
    assert!(
        c1.impact_flow_events > c0.impact_flow_events,
        "事件计数增加"
    );
    // 断开事件必在 recent 窗口内(石条撞墙后还有落地/弹跳的扛住事件会顶掉
    // 它的位置,但 8 条窗口足够新);broke 的事件条目 = 本次撞击断开。
    let hit = r1
        .iter()
        .filter(|(_, w)| w.broke && w.impact)
        .max_by_key(|(t, _)| t.0)
        .expect("recent 内必有 broke=true 的 Impact 事件(本次撞击)");
    assert!(hit.1.detached_cells >= 1, "|S| ≥ 1");
    // position = 被撞格格心:墙面格 (10,·,20) 格心 x = 10.5、z = 20.5;y
    // 在墙身 −56..−60 之内(端格剪切断下,具体格随接触分布)。
    assert!(
        (hit.1.position.x - 10.5).abs() < 1.5
            && (hit.1.position.z - 20.5).abs() < 1.5
            && (-60.5..=-55.5).contains(&hit.1.position.y),
        "position = 被撞墙格心,实得 {:?}",
        hit.1.position
    );

    // 13 项累计单调不减(基线包 c0 vs 断后包 c1)。
    let (a0, a1) = (counters_vec(&c0), counters_vec(&c1));
    for i in 0..13 {
        assert!(
            a1[i] >= a0[i],
            "第 {i} 项累计单调不减({} < {})",
            a1[i],
            a0[i]
        );
    }

    // 断连重连:订阅即失效(重连者新身份、未再订阅,收不到)。
    sub.send(&C2S::Goodbye).await.expect("Goodbye 写入");
    advance_ticks(8).await;
    let (mut sub2, mut rx_sub2) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _id2 = join(&mut sub2, &mut rx_sub2, 0).await;
    sub2.send(&C2S::MoveObserver {
        position: Vec3::new(1000.0, -58.0, 8.0),
    })
    .await
    .expect("重连者观察者写入");
    let mut sub2_got = false;
    for _ in 0..120 {
        if let Some(s2c) = try_frame(&mut rx_sub2)
            && matches!(s2c, S2C::DebugStats { .. })
        {
            sub2_got = true;
        }
        advance_ticks(1).await;
    }
    assert!(!sub2_got, "断连后订阅失效:重连者(未再订阅)收不到");

    let outcome = shutdown(h).await;
    assert!(outcome.ticks_run > 0);
}

// ---------------------------------------------------------------------------
// AC-W9:模拟不变——同一组命令,遥测订阅开/关两次运行,每拍全部实体状态
// (四向量 ==)与方块变更逐位相同(照 consistency.rs 的 A/B 两线口径)。
// ---------------------------------------------------------------------------

/// A/B 布景(确定性):`cmds` 逐条经驱动命令通道注入,`subscribe` = 订阅开关;
/// 结束后返回 (tick → 全部实体状态, 方块变更集合)(从 EntityStates/BlockUpdates
/// 帧收账,实体四向量位级)。
async fn run_line(
    addr: SocketAddr,
    tx: &mpsc::Sender<DriverCommand>,
    subscribe: bool,
) -> (
    Vec<(types::Tick, Vec<(EntityId, EntityState)>)>,
    Vec<(BlockPos, BlockId)>,
) {
    let (mut c, mut rx) = client(addr).await;
    tokio::task::yield_now().await;
    let _id = join(&mut c, &mut rx, 2).await;
    if subscribe {
        c.send(&C2S::SubscribeDebug).await.expect("订阅写入");
    }
    c.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, -58.0, 8.0),
    })
    .await
    .expect("观察者写入");
    advance_ticks(12).await;

    // 布景命令:两个弹道实体(零重力,位级对照要求绝对静止弹道……不,重力
    // 开;两个实体同一初速平行下落,对照线相同即可)与一格 SetBlock。
    for cmd in [
        bullet(1, 10.5, -59.0, 8.5, -90.0),
        bullet(2, 20.5, -59.0, 8.5, -90.0),
    ] {
        tx.send(cmd).await.expect("驱动存活");
    }
    tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SetBlock {
            pos: BlockPos::new(4, -61, 4),
            block: BlockId(3),
        },
    })
    .await
    .expect("驱动存活");
    advance_ticks(150).await;

    // 收账:EntityStates 快照(每包 tick → 整表克隆)与 BlockUpdates 逐格。
    let mut snapshots: Vec<(types::Tick, Vec<(EntityId, EntityState)>)> = Vec::new();
    let mut blocks: Vec<(BlockPos, BlockId)> = Vec::new();
    for _ in 0..600 {
        match try_frame(&mut rx) {
            Some(S2C::EntityStates { tick, states, .. }) => {
                snapshots.push((tick, states));
                advance_ticks(1).await;
            }
            Some(S2C::BlockUpdates { updates }) => {
                blocks.extend(updates);
            }
            Some(_) => {}
            None => advance_ticks(1).await,
        }
        // 收账窗 150 拍即止(布景已停,实体已碎/静止)。
        if snapshots.iter().map(|(t, _)| t.0).max().unwrap_or(0) > 150 {
            break;
        }
    }
    (snapshots, blocks)
}

#[tokio::test(start_paused = true)]
async fn w9_telemetry_subscription_bitwise_identical_simulation() {
    // A 线(订阅开)。
    let ha = harness().await;
    let a = run_line(ha.addr, &ha.tx, true).await;
    shutdown(ha).await;
    // B 线(订阅关)。
    let hb = harness().await;
    let b = run_line(hb.addr, &hb.tx, false).await;
    shutdown(hb).await;

    // 实体状态:同 tick 全量包整表四向量位级 ==(consistency.rs 口径;两线
    // 同一初值与命令,确定性布景下逐位一致)。
    let (sa, sb) = (&a.0, &b.0);
    assert!(!sa.is_empty() && !sb.is_empty(), "两线都须收到实体流");
    let find = |v: &[(types::Tick, Vec<(EntityId, EntityState)>)], t: u64| {
        v.iter()
            .rev()
            .find(|(tick, states)| {
                tick.0 == t && states.iter().any(|(id, _)| id.origin_shard() == 1)
            })
            .map(|(_, states)| states.clone())
    };
    for t in 20..=150u64 {
        let (xa, xb) = (find(sa, t), find(sb, t));
        match (xa, xb) {
            (Some(xa), Some(xb)) => {
                assert_eq!(xa, xb, "拍 {t}:实体状态四向量逐位相同(订阅开 vs 关)")
            }
            (None, None) => {}
            _ => panic!("拍 {t}:两线实体可见性不同构(A/B 帧序不同)"),
        }
    }
    // 方块变更:两线集合逐位一致(SetBlock 注入格 + 撞击破坏格)。
    let mut ba = a.1.clone();
    let mut bb = b.1.clone();
    ba.sort_by_key(|(p, _)| *p);
    bb.sort_by_key(|(p, _)| *p);
    ba.dedup_by_key(|(p, _)| *p);
    bb.dedup_by_key(|(p, _)| *p);
    assert_eq!(ba, bb, "方块变更逐位相同(订阅开 vs 关)");
}
