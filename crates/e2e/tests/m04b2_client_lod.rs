//! REQ-014 M04b-2 T19/T20:真网络 LOD 客户端 → `client_core::entities::EntityMirror` 端到端测试。
//!
//! 复用 `m04b1_lod_wire.rs` harness 模式(真驱动 + 真 TCP `127.0.0.1:0`,暂停时钟,零重力):
//! - T19:quad(13 连杆)近距 Compact tier=1 解码 + Meta 先行 + B 未发 SetAgentStream ACM8 对照;
//! - T20:quad 远距 Lite 静姿展开 13 连杆 → MoveObserver 拉近升档 2→1,全程换档不发 removed 且清除 agent_speed。

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::Config;
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use client_core::entities::EntityMirror;
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
use types::{BodyDef, DimensionId, EntityId, EntityState, ShardId, ShardPos, WorldId};
use voxel::{BlockRegistry, SimpleRegistry};

const SOFT: Duration = Duration::from_millis(50);
const QUAD_TOML: &str = include_str!("../../../assets/bodies/quad.body.toml");

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

fn quad_spawn(pos: Vec3) -> EntityState {
    EntityState {
        translation: pos,
        ..EntityState::default()
    }
}

fn quad_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies.insert_toml(QUAD_TOML).expect("quad.body.toml 合法");
    bodies
}

async fn step_one_tick() {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    tokio::time::sleep(dt).await;
    for _ in 0..8 {
        tokio::task::yield_now().await;
    }
}

async fn next_frame(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Option<S2C> {
    tokio::time::timeout(SOFT, rx.recv()).await.ok().flatten()
}

fn real_yield() {
    std::thread::sleep(Duration::from_millis(1));
}

async fn drain_ready(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Vec<S2C> {
    let mut out = Vec::new();
    for _ in 0..16 {
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        while let Ok(msg) = rx.try_recv() {
            out.push(msg);
        }
        real_yield();
    }
    for _ in 0..4 {
        tokio::task::yield_now().await;
        while let Ok(msg) = rx.try_recv() {
            out.push(msg);
        }
    }
    out
}

struct TestClient {
    w: tokio::net::tcp::OwnedWriteHalf,
}

impl TestClient {
    async fn send(&mut self, c2s: &C2S) -> std::io::Result<()> {
        self.w.write_all(&encode_c2s_frame(c2s)).await
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

/// REQ-014 M04b-2 T19:quad(13 连杆)近距 Compact tier=1 → EntityMirror 解码 + ACM8 对照。
#[tokio::test(start_paused = true)]
async fn m04b2_t19_quad_compact_decode_and_acm8() {
    let bodies = quad_registry();
    let quad_def = BodyDef::from_toml_str(QUAD_TOML).expect("quad 合法");
    let quad_hash = quad_def.hash();
    assert_eq!(bodies.hash_of("quad"), Some(quad_hash));

    let h = harness(
        base_config(),
        PumpConfig {
            interval_ms: 60_000,
            max_catchup_rounds: 1,
        },
        (6, 60),
        true,
        bodies,
    )
    .await;

    let (mut a, mut rx_a) = client(h.addr).await;
    let (mut b, mut rx_b) = client(h.addr).await;
    tokio::task::yield_now().await;

    let _id_a = join(&mut a, &mut rx_a, 8).await;
    let _id_b = join(&mut b, &mut rx_b, 8).await;

    let obs_pos = Vec3::new(8.0, 3.0, 8.0);
    a.send(&C2S::MoveObserver { position: obs_pos })
        .await
        .expect("A MoveObserver");
    b.send(&C2S::MoveObserver { position: obs_pos })
        .await
        .expect("B MoveObserver");
    a.send(&C2S::SetAgentStream { lod: true })
        .await
        .expect("A SetAgentStream(true)");

    for _ in 0..4 {
        step_one_tick().await;
    }
    let _ = drain_ready(&mut rx_a).await;
    let _ = drain_ready(&mut rx_b).await;

    let e1 = EntityId::from_parts(0, 911);
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnAgent {
            entity: e1,
            body: quad_hash,
            state: quad_spawn(Vec3::new(8.0, 3.0, 28.0)),
        },
    })
    .await
    .expect("spawn e1");

    let mut mirror = EntityMirror::new();
    mirror.set_defs([(quad_hash, quad_def)]);
    let mut a_saw_compact = false;
    let mut b_saw_legacy = false;

    for _ in 0..60 {
        step_one_tick().await;
        for msg in drain_ready(&mut rx_a).await {
            if let S2C::AgentStatesCompact { states, .. } = &msg
                && states.iter().any(|s| s.entity == e1)
            {
                assert!(
                    mirror.meta.contains_key(&e1),
                    "Compact 到达时 mirror.meta 必须已含 e1"
                );
                a_saw_compact = true;
            }
            if matches!(msg, S2C::AgentStates { .. }) {
                panic!("lod 连接 A 不得收到旧 AgentStates");
            }
            mirror.apply(&msg);
        }
        for msg in drain_ready(&mut rx_b).await {
            match msg {
                S2C::AgentStates { states, .. } => {
                    if states.iter().any(|(id, ..)| *id == e1) {
                        b_saw_legacy = true;
                    }
                }
                S2C::AgentMeta { .. }
                | S2C::AgentStatesCompact { .. }
                | S2C::AgentStatesLite { .. } => {
                    panic!("ACM8 对照连接 B 不得收到 Meta/Compact/Lite");
                }
                _ => {}
            }
        }
        if a_saw_compact && b_saw_legacy {
            break;
        }
    }

    assert!(a_saw_compact, "60 拍内 A 必须收到含 e1 的 Compact");
    assert!(b_saw_legacy, "60 拍内 B 必须收到含 e1 的旧 AgentStates");

    let view = mirror.agents.get(&e1).expect("mirror.agents 应含 e1");
    assert_eq!(view.poses.len(), 13, "quad Compact 解码应为 13 连杆");
    assert_eq!(mirror.agent_tiers.get(&e1), Some(&1), "e1 应为 tier 1");
    assert_eq!(view.hash, quad_hash, "e1 hash 应为 quad_hash");
    assert_eq!(mirror.meta.get(&e1), Some(&quad_hash), "meta 应含 e1");

    let _outcome = shutdown(h).await;
}

/// REQ-014 M04b-2 T20:quad 远距 Lite 静姿展开 13 连杆 → MoveObserver 拉近升档 2→1 且全程无 removed。
#[tokio::test(start_paused = true)]
async fn m04b2_t20_lite_expand_upgrade_no_removed() {
    let bodies = quad_registry();
    let quad_def = BodyDef::from_toml_str(QUAD_TOML).expect("quad 合法");
    let quad_hash = quad_def.hash();

    let h = harness(
        base_config(),
        PumpConfig {
            interval_ms: 60_000,
            max_catchup_rounds: 1,
        },
        (6, 60),
        true,
        bodies,
    )
    .await;

    let (mut a, mut rx_a) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _id_a = join(&mut a, &mut rx_a, 8).await;

    a.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, 3.0, 8.0),
    })
    .await
    .expect("A MoveObserver 初始");
    a.send(&C2S::SetAgentStream { lod: true })
        .await
        .expect("A SetAgentStream(true)");

    for _ in 0..4 {
        step_one_tick().await;
    }
    let _ = drain_ready(&mut rx_a).await;

    let e2 = EntityId::from_parts(0, 912);
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnAgent {
            entity: e2,
            body: quad_hash,
            state: quad_spawn(Vec3::new(8.0, 3.0, 108.0)),
        },
    })
    .await
    .expect("spawn e2");

    let mut mirror = EntityMirror::new();
    mirror.set_defs([(quad_hash, quad_def)]);
    let mut removed_seen: Vec<EntityId> = Vec::new();

    // 阶段 1: <=90 拍内收到 Lite 并展开 13 连杆,tier == 2。
    let mut phase1_ok = false;
    for _ in 0..90 {
        step_one_tick().await;
        for msg in drain_ready(&mut rx_a).await {
            match &msg {
                S2C::AgentStatesCompact { removed, .. } | S2C::AgentStatesLite { removed, .. } => {
                    removed_seen.extend_from_slice(removed);
                }
                S2C::AgentStates { .. } => panic!("lod 连接 A 不得收到旧 AgentStates"),
                _ => {}
            }
            mirror.apply(&msg);
        }
        if mirror.agent_tiers.get(&e2) == Some(&2) && mirror.agents.contains_key(&e2) {
            phase1_ok = true;
            break;
        }
    }
    assert!(phase1_ok, "90 拍内 e2 必须进入 Lite tier 2 并解码进镜像");
    assert_eq!(
        mirror.agents[&e2].poses.len(),
        13,
        "Lite 静姿 FK 展开应为 13 连杆"
    );
    assert_eq!(mirror.agent_tiers.get(&e2), Some(&2));
    assert!(
        !removed_seen.contains(&e2),
        "阶段 1 e2 不得出现在任何 Compact/Lite removed 中:{removed_seen:?}"
    );

    // 阶段 2: MoveObserver 到 (8,3,108) -> d=0 -> 升档 2->1,poses.len 仍 13,无 removed,清 agent_speed。
    a.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, 3.0, 108.0),
    })
    .await
    .expect("A MoveObserver 拉近");

    let mut phase2_ok = false;
    for _ in 0..90 {
        step_one_tick().await;
        for msg in drain_ready(&mut rx_a).await {
            match &msg {
                S2C::AgentStatesCompact { removed, .. } | S2C::AgentStatesLite { removed, .. } => {
                    removed_seen.extend_from_slice(removed);
                }
                S2C::AgentStates { .. } => panic!("lod 连接 A 不得收到旧 AgentStates"),
                _ => {}
            }
            mirror.apply(&msg);
        }
        if mirror.agent_tiers.get(&e2) == Some(&1) {
            phase2_ok = true;
            break;
        }
    }
    assert!(phase2_ok, "90 拍内 e2 必须由 tier 2 升档至 tier 1");
    assert_eq!(
        mirror.agents[&e2].poses.len(),
        13,
        "升档后 poses.len 仍为 13"
    );
    assert!(
        !removed_seen.contains(&e2),
        "换档全程 e2 不得出现在任何 Compact/Lite removed 中:{removed_seen:?}"
    );
    assert!(
        !mirror.agent_speed.contains_key(&e2),
        "升档 Compact 后必须清除 agent_speed 中的 e2"
    );

    let _outcome = shutdown(h).await;
}
