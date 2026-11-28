//! REQ-014 M03:双客户端权威 e2e(ACM1 / ACM4 / ACM2 + F4 语义面)。
//!
//! 装配照 `network.rs`(真驱动 + serve 127.0.0.1:0 + observe 三缝真接;
//! `base_config` 关 cohesion;`read_timeout_secs = 600`;`start_paused = true`);
//! `TestClient` 复制内嵌版并补 `send_batch`(单次 write_all 拼帧)。

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::Config;
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::{InputSource, SimCommand};
use physics_sim::{BodyRegistry, ShardSim, SolidifyConfig};
use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use sim_api::session::Session;
use storage::mem::MemCheckpointStore;
use storage::worker::StoreWorker;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{
    BlockId, BlockPos, ChunkPos, DimensionId, EntityId, EntityState, ShardId, ShardPos, WorldId,
};
use voxel::{BlockRegistry, Chunk, SimpleRegistry};

/// Superflat classic 地面顶面 world y(-64 + 层厚 4 = -60.0)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;

/// 软窗:等一帧最多 50 ms(虚拟时钟)。
const SOFT: Duration = Duration::from_millis(50);

/// 测试用分片身份。
fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// registry 工厂。
fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

/// capsule 出生姿态。
fn capsule_spawn(pos: Vec3) -> EntityState {
    EntityState {
        translation: pos,
        ..EntityState::default()
    }
}

/// capsule 身体注册表。
fn capsule_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
}

/// 暂停时间下推进 N 拍。
async fn advance_ticks(ticks: u32) {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

/// 软窗收一帧(None = 窗口内无帧)。
async fn next_frame(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Option<S2C> {
    tokio::time::timeout(SOFT, rx.recv()).await.ok().flatten()
}

/// 软窗落空后的真实墙钟让步(1 ms),防止暂停时钟在空闲信道上饿死 loopback 往返。
fn real_yield() {
    std::thread::sleep(Duration::from_millis(1));
}

// ---------------------------------------------------------------------------
// 内嵌测试客户端(network.rs / a03_agents_net.rs 同款 + send_batch)
// ---------------------------------------------------------------------------

struct TestClient {
    w: tokio::net::tcp::OwnedWriteHalf,
}

impl TestClient {
    async fn send(&mut self, c2s: &C2S) -> std::io::Result<()> {
        self.w.write_all(&encode_c2s_frame(c2s)).await
    }

    #[allow(dead_code)]
    async fn send_batch(&mut self, msgs: &[C2S]) -> std::io::Result<()> {
        let mut batch = Vec::with_capacity(msgs.len() * 16);
        for msg in msgs {
            batch.extend_from_slice(&encode_c2s_frame(msg));
        }
        self.w.write_all(&batch).await
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
            if let Some(s2c) = decode_frame(&frame)
                && tx.send(s2c).is_err()
            {
                return;
            }
        }
    });
    (TestClient { w }, rx)
}

/// Hello + Subscribe,返回 Welcome 的 player_id。
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

/// C2S::Possess → 等 Possessed 回执。
async fn possess(
    c: &mut TestClient,
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    entity: EntityId,
    budget: u32,
) -> Option<Option<EntityId>> {
    while rx.try_recv().is_ok() {}
    c.send(&C2S::Possess { entity })
        .await
        .expect("Possess 写入");
    for _ in 0..budget {
        match next_frame(rx).await {
            Some(S2C::Possessed { entity }) => return Some(entity),
            Some(_) => continue,
            None => real_yield(),
        }
    }
    None
}

// ---------------------------------------------------------------------------
// 装配 harness(network.rs 同款 + 可选身体注册表)
// ---------------------------------------------------------------------------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: tokio::task::JoinHandle<DriverOutcome>,
    addr: SocketAddr,
}

fn base_config() -> Config {
    Config {
        worker_count: 2,
        checkpoint_interval_ticks: 600,
        cohesion: app::config::CohesionConfig {
            enabled: false,
            ..app::config::CohesionConfig::default()
        },
        ..Config::default()
    }
}

async fn harness(
    config: Config,
    pump: PumpConfig,
    observe: (u64, u64),
    zero_g: bool,
    bodies: BodyRegistry,
) -> Harness {
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
    std::mem::forget(bcast_tx);

    let session = Session::new(config.session_config());
    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    let (provider, _, _bodies) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry())
            .expect("装配失败(fail-stop)");
    let sim_source = config.clone();
    let sim_bodies = bodies.clone();
    let new_sim = Box::new(move |shard: ShardId| {
        let mut cfg = sim_source.shard_sim_config(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        if zero_g {
            cfg.gravity = Vec3::ZERO;
        }
        ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), sim_bodies.clone())
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
        bodies,
        resume: None,
    }));
    Harness { tx, driver, addr }
}

async fn shutdown(h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    h.driver.await.expect("驱动不 panic")
}

// ---------------------------------------------------------------------------
// T1 (ACM1):双客户端编辑互见 ≤2 tick
// ---------------------------------------------------------------------------

#[tokio::test(start_paused = true)]
async fn m03_t1_edit_visible_within_two_ticks() {
    let h = harness(
        base_config(),
        PumpConfig::default(),
        (1, 60),
        true,
        BodyRegistry::new(),
    )
    .await;

    let (mut a, mut rx_a) = client(h.addr).await;
    let (mut b, mut rx_b) = client(h.addr).await;
    tokio::task::yield_now().await;

    let _id_a = join(&mut a, &mut rx_a, 4).await;
    let _id_b = join(&mut b, &mut rx_b, 4).await;
    let pos = Vec3::new(8.0, -58.0, 8.0);
    a.send(&C2S::MoveObserver { position: pos })
        .await
        .expect("A MoveObserver 写入");
    b.send(&C2S::MoveObserver { position: pos })
        .await
        .expect("B MoveObserver 写入");
    advance_ticks(30).await;

    // 排干前置帧,确保随后检出的是本次编辑广播。
    while rx_a.try_recv().is_ok() {}
    while rx_b.try_recv().is_ok() {}

    let target = (BlockPos::new(10, -58, 10), BlockId(1));
    a.send(&C2S::Command(SimCommand::SetBlock {
        pos: target.0,
        block: target.1,
    }))
    .await
    .expect("A SetBlock 写入");
    real_yield();
    tokio::task::yield_now().await;

    // ≤2 tick 锚:至多 2 轮,每轮 advance_ticks(1) + SOFT 软窗排空 B 收帧通道。
    let mut b_hit = false;
    for _ in 0..2 {
        advance_ticks(1).await;
        for _ in 0..256 {
            match next_frame(&mut rx_b).await {
                Some(S2C::BlockUpdates { updates }) if updates.contains(&target) => {
                    b_hit = true;
                    break;
                }
                Some(_) => continue,
                None => break,
            }
        }
        if b_hit {
            break;
        }
    }
    assert!(
        b_hit,
        "ACM1: 2 轮(≤2 tick)内 B 必须收到含 {target:?} 的 BlockUpdates"
    );

    // 旁证:A 自己也收到含该格的 BlockUpdates(现网自见语义)。
    let mut a_hit = false;
    for _ in 0..256 {
        match next_frame(&mut rx_a).await {
            Some(S2C::BlockUpdates { updates }) if updates.contains(&target) => {
                a_hit = true;
                break;
            }
            Some(_) => continue,
            None => break,
        }
    }
    assert!(a_hit, "旁证:A 自己也必须收到含 {target:?} 的 BlockUpdates");

    let _outcome = shutdown(h).await;
}

// ---------------------------------------------------------------------------
// T2 (ACM4):迟到加入者看到活跃热块编辑
// ---------------------------------------------------------------------------

#[tokio::test(start_paused = true)]
async fn m03_t2_late_joiner_sees_hot_edit() {
    let pump = PumpConfig {
        interval_ms: 200,
        max_catchup_rounds: 1,
    };
    let h = harness(base_config(), pump, (2, 60), true, BodyRegistry::new()).await;

    let (mut a, mut rx_a) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _id_a = join(&mut a, &mut rx_a, 2).await;
    let pos = Vec3::new(8.0, -58.0, 8.0);
    a.send(&C2S::MoveObserver { position: pos })
        .await
        .expect("A MoveObserver 写入");
    advance_ticks(60).await;

    while rx_a.try_recv().is_ok() {}
    let target = (BlockPos::new(10, -58, 10), BlockId(1));
    a.send(&C2S::Command(SimCommand::SetBlock {
        pos: target.0,
        block: target.1,
    }))
    .await
    .expect("A SetBlock 写入");
    real_yield();
    tokio::task::yield_now().await;
    advance_ticks(10).await;

    let mut a_saw_edit = false;
    for _ in 0..256 {
        match next_frame(&mut rx_a).await {
            Some(S2C::BlockUpdates { updates }) if updates.contains(&target) => {
                a_saw_edit = true;
                break;
            }
            Some(_) => continue,
            None => break,
        }
    }
    assert!(a_saw_edit, "旁证:A 编辑后必须收到 BlockUpdates");

    // 迟到加入者 B:同位入场,收集 ChunkPos(0, 0)。
    let (mut b, mut rx_b) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _id_b = join(&mut b, &mut rx_b, 2).await;
    b.send(&C2S::MoveObserver { position: pos })
        .await
        .expect("B MoveObserver 写入");

    let mut received_chunk: Option<Chunk> = None;
    for _ in 0..100 {
        advance_ticks(1).await;
        for _ in 0..64 {
            match next_frame(&mut rx_b).await {
                Some(S2C::ChunkData {
                    pos: chunk_pos,
                    chunk,
                }) if chunk_pos == ChunkPos::new(0, 0) => {
                    received_chunk = Some(chunk);
                    break;
                }
                Some(_) => continue,
                None => break,
            }
        }
        if received_chunk.is_some() {
            break;
        }
    }
    let chunk = received_chunk.expect("预算 400 虚拟拍内 B 必须收到 ChunkData(0, 0)");

    // 局部 y = 世界 y - WORLD_Y_MIN = -58 - (-64) = 6;局部 x/z = 10。
    let local_y = (-58 - types::WORLD_Y_MIN) as usize;
    assert_eq!(local_y, 6);
    assert_eq!(
        chunk.get(10, local_y, 10),
        BlockId(1),
        "ACM4: 迟到者收到的活跃热块 (10,-58,10) 必须为编辑值 BlockId(1)"
    );

    let outcome = shutdown(h).await;
    assert!(
        outcome.chunks_loaded_hot >= 1,
        "chunks_loaded_hot 必须 >= 1,实为 {}",
        outcome.chunks_loaded_hot
    );
}

// ---------------------------------------------------------------------------
// T3 (ACM2 + F4 语义面):带负荷硬断线
// ---------------------------------------------------------------------------

#[tokio::test(start_paused = true)]
async fn m03_t3_hard_drop_under_load() {
    let bodies = capsule_registry();
    let capsule_hash = bodies.hash_of("capsule").expect("capsule 已装");
    let h = harness(base_config(), PumpConfig::default(), (2, 60), true, bodies).await;

    let (mut a, mut rx_a) = client(h.addr).await;
    let (mut b, mut rx_b) = client(h.addr).await;
    tokio::task::yield_now().await;

    let _id_a = join(&mut a, &mut rx_a, 4).await;
    let _id_b = join(&mut b, &mut rx_b, 4).await;
    let pos = Vec3::new(8.0, -58.0, 8.0);
    a.send(&C2S::MoveObserver { position: pos })
        .await
        .expect("A MoveObserver 写入");
    b.send(&C2S::MoveObserver { position: pos })
        .await
        .expect("B MoveObserver 写入");
    advance_ticks(30).await;

    // 经 trusted 通道生成服务端主体 900。
    let server_agent = EntityId::from_parts(0, 900);
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnAgent {
            entity: server_agent,
            body: capsule_hash,
            state: capsule_spawn(Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0)),
        },
    })
    .await
    .expect("commands 通道 Send SpawnAgent");
    advance_ticks(30).await;

    // A 附身服务端主体并走起来;B 同步施加方块编辑负荷。
    assert_eq!(
        possess(&mut a, &mut rx_a, server_agent, 200).await,
        Some(Some(server_agent)),
        "A 附身服务端主体 900 成功"
    );
    a.send(&C2S::AgentIntent {
        entity: server_agent,
        move_xz: (2.0, 0.0),
        yaw: 0.0,
        jump: false,
    })
    .await
    .expect("A AgentIntent 写入");
    b.send(&C2S::Command(SimCommand::SetBlock {
        pos: BlockPos::new(12, -58, 12),
        block: BlockId(1),
    }))
    .await
    .expect("B 负荷 SetBlock 写入");
    advance_ticks(10).await;

    // 硬断线:A 直接 drop 写端与读端(不发 Goodbye)。
    drop(a);
    drop(rx_a);
    real_yield();
    advance_ticks(30).await;

    // 锚 a:服务器不倒,B 再发 SetBlock(13,-58,12) → advance_ticks(2)+SOFT 内 B 收到 BlockUpdates。
    while rx_b.try_recv().is_ok() {}
    let check_cell = (BlockPos::new(13, -58, 12), BlockId(1));
    b.send(&C2S::Command(SimCommand::SetBlock {
        pos: check_cell.0,
        block: check_cell.1,
    }))
    .await
    .expect("B 断线后 SetBlock 写入");
    real_yield();
    tokio::task::yield_now().await;
    advance_ticks(2).await;

    let mut b_saw_block = false;
    let mut b_saw_agent = false;
    for _ in 0..256 {
        match next_frame(&mut rx_b).await {
            Some(S2C::BlockUpdates { updates }) if updates.contains(&check_cell) => {
                b_saw_block = true;
                break;
            }
            Some(S2C::AgentStates { states, .. })
                if states.iter().any(|(e, ..)| *e == server_agent) =>
            {
                b_saw_agent = true;
            }
            Some(_) => continue,
            None => break,
        }
    }
    assert!(
        b_saw_block,
        "锚 a: A 硬断线后服务器必须存活,B 应在 advance_ticks(2)+SOFT 内收到 {check_cell:?}"
    );

    // 锚 b:服务端主体保留,B 的 AgentStates 仍含 entity 900(兼顾 F7 静止零增量,推进至多覆盖一个全量周期 60 拍)。
    if !b_saw_agent {
        for _ in 0..35 {
            advance_ticks(2).await;
            for _ in 0..64 {
                match next_frame(&mut rx_b).await {
                    Some(S2C::AgentStates { states, .. })
                        if states.iter().any(|(e, ..)| *e == server_agent) =>
                    {
                        b_saw_agent = true;
                        break;
                    }
                    Some(_) => continue,
                    None => break,
                }
            }
            if b_saw_agent {
                break;
            }
        }
    }
    assert!(b_saw_agent, "锚 b: B 的 AgentStates 必须仍含服务端主体 900");

    // 锚 c:附身强制解除,B 可接手附身 entity 900。
    assert_eq!(
        possess(&mut b, &mut rx_b, server_agent, 200).await,
        Some(Some(server_agent)),
        "锚 c: A 硬断线后附身强制解除,B 必须可接手附身主体 900"
    );

    // 锚 d:shutdown outcome。
    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.agent.bodies_removed_on_disconnect, 0,
        "锚 d: A 无玩家身体,bodies_removed_on_disconnect 必须 == 0"
    );
    assert_eq!(
        outcome.agent.possess_rejected, 0,
        "锚 d: possess_rejected 必须 == 0"
    );
}
