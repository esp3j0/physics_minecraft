//! REQ-014 M04b-1 T12:服务器 LOD 状态机端到端真网络线协议测试。
//!
//! 复用 `m03_dual_client.rs` harness 模式(真驱动 + 真 TCP `127.0.0.1:0`,暂停时钟,零重力):
//! - A(lod=true)与 B(未发 SetAgentStream,ACM8 对照),vd=8;
//! - spawn 近距 capsule(d=20,L1)与远距 capsule(d=100,L2);
//! - 验证 A 先收 AgentMeta 后收 Compact tier=1 / Lite(full=true),B 收旧 AgentStates 含两实体;
//! - A possess 近者 → Compact tier=0 每拍;unpossess → P1 tier=0 removed + ≤1 拍 tier=1 补闪;
//! - 全程换档实体不出现在 Compact tier=1 / Lite 的 removed 中。

use std::collections::BTreeSet;
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
use types::{DimensionId, EntityId, EntityState, ShardId, ShardPos, WorldId};
use voxel::{BlockRegistry, SimpleRegistry};

const SOFT: Duration = Duration::from_millis(50);

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

fn capsule_spawn(pos: Vec3) -> EntityState {
    EntityState {
        translation: pos,
        ..EntityState::default()
    }
}

fn capsule_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
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

/// REQ-014 M04b-1 T12:LOD 端到端线协议(Meta 先行 / L1 Compact / L2 Lite / ACM8 对照 /
/// L0 60Hz 专流 / Unpossess P1 removed + L1 补闪 / 换档绝不发 removed)。
#[tokio::test(start_paused = true)]
async fn m04b1_t12_lod_wire_e2e() {
    let bodies = capsule_registry();
    let capsule_hash = bodies.hash_of("capsule").expect("capsule 已装");
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

    let id_a = join(&mut a, &mut rx_a, 8).await;
    let _id_b = join(&mut b, &mut rx_b, 8).await;
    let phase_a = u64::from(id_a.0);

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

    // 在 (0,0) 分片内生成近者(d=20 -> L1)与远者(d=100 -> L2)。
    let e_near = EntityId::from_parts(0, 901);
    let e_far = EntityId::from_parts(0, 902);
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnAgent {
            entity: e_near,
            body: capsule_hash,
            state: capsule_spawn(Vec3::new(8.0, 3.0, 28.0)),
        },
    })
    .await
    .expect("spawn e_near");
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnAgent {
            entity: e_far,
            body: capsule_hash,
            state: capsule_spawn(Vec3::new(8.0, 3.0, 108.0)),
        },
    })
    .await
    .expect("spawn e_far");

    // 阶段 1: 验证 A Meta 先行 + Compact tier=1(near) + Lite full=true(far); B 收旧 AgentStates 含两实体。
    let mut a_meta_seen: BTreeSet<EntityId> = BTreeSet::new();
    let mut a_saw_near_t1 = false;
    let mut a_saw_far_lite = false;
    let mut b_saw_near_legacy = false;
    let mut b_saw_far_legacy = false;

    for _ in 0..60 {
        step_one_tick().await;
        for msg in drain_ready(&mut rx_a).await {
            match msg {
                S2C::AgentMeta { entries } => {
                    for (id, hash) in entries {
                        assert_eq!(hash, capsule_hash, "Meta BodyHash 必须为 capsule");
                        a_meta_seen.insert(id);
                    }
                }
                S2C::AgentStatesCompact {
                    tier,
                    states,
                    removed,
                    ..
                } => {
                    assert!(
                        removed.is_empty(),
                        "阶段 1 未销毁/未解除附身,Compact tier={tier} 不得有 removed:{removed:?}"
                    );
                    for s in &states {
                        assert!(
                            a_meta_seen.contains(&s.entity),
                            "AgentMeta 必须先于对应 Compact 到达:entity={:?}",
                            s.entity
                        );
                        if tier == 1 && s.entity == e_near {
                            a_saw_near_t1 = true;
                        }
                    }
                }
                S2C::AgentStatesLite {
                    states,
                    removed,
                    full,
                    ..
                } => {
                    assert!(full, "Lite 包 full 必须恒为 true");
                    assert!(removed.is_empty(), "阶段 1 Lite 不得有 removed:{removed:?}");
                    for s in &states {
                        assert!(
                            a_meta_seen.contains(&s.entity),
                            "AgentMeta 必须先于对应 Lite 到达:entity={:?}",
                            s.entity
                        );
                        if s.entity == e_far {
                            a_saw_far_lite = true;
                        }
                    }
                }
                S2C::AgentStates { .. } => {
                    panic!("lod_enabled 连接 A 不得收到旧 AgentStates");
                }
                _ => {}
            }
        }
        for msg in drain_ready(&mut rx_b).await {
            match msg {
                S2C::AgentStates { states, .. } => {
                    for (id, _, _) in states {
                        if id == e_near {
                            b_saw_near_legacy = true;
                        }
                        if id == e_far {
                            b_saw_far_legacy = true;
                        }
                    }
                }
                S2C::AgentMeta { .. }
                | S2C::AgentStatesCompact { .. }
                | S2C::AgentStatesLite { .. } => {
                    panic!("ACM8 对照连接 B 不得收到新 LOD 包");
                }
                _ => {}
            }
        }
        if a_saw_near_t1 && a_saw_far_lite && b_saw_near_legacy && b_saw_far_legacy {
            break;
        }
    }
    assert!(a_saw_near_t1, "A 必须收到近者的 Compact tier=1");
    assert!(a_saw_far_lite, "A 必须收到远者的 Lite(full=true)");
    assert!(
        b_saw_near_legacy && b_saw_far_legacy,
        "ACM8: B 必须收到旧 AgentStates 含近远两实体"
    );

    // 阶段 2: A 附身近者 -> Compact tier=0 每拍。
    a.send(&C2S::Possess { entity: e_near })
        .await
        .expect("A Possess");
    let mut possessed_ok = false;
    for _ in 0..20 {
        step_one_tick().await;
        for msg in drain_ready(&mut rx_a).await {
            match msg {
                S2C::Possessed { entity } if entity == Some(e_near) => possessed_ok = true,
                S2C::AgentStatesCompact {
                    tier: 1, removed, ..
                }
                | S2C::AgentStatesLite { removed, .. } => {
                    assert!(removed.is_empty(), "附身换档不得发 tier1/Lite removed");
                }
                S2C::AgentStates { .. } => panic!("A 不得收到旧 AgentStates"),
                _ => {}
            }
        }
        if possessed_ok {
            break;
        }
    }
    assert!(possessed_ok, "A 必须成功附身 e_near");

    // 收集连续 6 拍 L0 Compact tier=0,并推进至非 l1_due 拍以真实考验解除附身补闪。
    let mut l0_ticks = Vec::new();
    let mut last_l0_tick = 0u64;
    for _ in 0..6 {
        step_one_tick().await;
        for msg in drain_ready(&mut rx_a).await {
            match msg {
                S2C::AgentStatesCompact {
                    tick,
                    tier: 0,
                    states,
                    removed,
                    full,
                } => {
                    assert!(!full && removed.is_empty());
                    assert_eq!(states.len(), 1);
                    assert_eq!(states[0].entity, e_near);
                    l0_ticks.push(tick.0);
                    last_l0_tick = tick.0;
                }
                S2C::AgentStatesCompact {
                    tier: 1, removed, ..
                }
                | S2C::AgentStatesLite { removed, .. } => {
                    assert!(removed.is_empty(), "附身期间不得发 tier1/Lite removed");
                }
                S2C::AgentStates { .. } => panic!("A 不得收到旧 AgentStates"),
                _ => {}
            }
        }
    }
    assert_eq!(l0_ticks.len(), 6, "L0 专流必须每拍 1 帧");
    for w in l0_ticks.windows(2) {
        assert_eq!(w[1], w[0] + 1, "L0 拍号必须连续");
    }

    // 对齐到一个非 l1_due 的拍(下一拍 (t+phase)%6 != 0 且 != 5),避免被常规周期掩盖 force_send。
    for _ in 0..6 {
        let next_t = last_l0_tick + 1;
        if !(next_t + phase_a).is_multiple_of(6) && !(next_t + 1 + phase_a).is_multiple_of(6) {
            break;
        }
        step_one_tick().await;
        for msg in drain_ready(&mut rx_a).await {
            if let S2C::AgentStatesCompact { tick, tier: 0, .. } = msg {
                last_l0_tick = tick.0;
            }
        }
    }

    // 阶段 3: A 解除附身 -> P1 Compact tier=0 removed + <=1 拍 Compact tier=1 补闪,全程无 tier1/Lite removed。
    a.send(&C2S::Unpossess).await.expect("A Unpossess");
    let mut saw_possessed_none = false;
    let mut removed_t0_tick: Option<u64> = None;
    let mut flash_t1_tick: Option<u64> = None;
    for _ in 0..10 {
        step_one_tick().await;
        for msg in drain_ready(&mut rx_a).await {
            match msg {
                S2C::Possessed { entity: None } => saw_possessed_none = true,
                S2C::AgentStatesCompact {
                    tick,
                    tier: 0,
                    states,
                    removed,
                    full,
                } => {
                    if !full && states.is_empty() && removed == vec![e_near] {
                        removed_t0_tick = Some(tick.0);
                    }
                }
                S2C::AgentStatesCompact {
                    tick,
                    tier: 1,
                    states,
                    removed,
                    ..
                } => {
                    assert!(
                        !removed.contains(&e_near) && !removed.contains(&e_far),
                        "换档实体不得出现在 tier1 removed:{removed:?}"
                    );
                    if states.iter().any(|p| p.entity == e_near) && flash_t1_tick.is_none() {
                        flash_t1_tick = Some(tick.0);
                    }
                }
                S2C::AgentStatesLite { removed, .. } => {
                    assert!(
                        !removed.contains(&e_near) && !removed.contains(&e_far),
                        "换档实体不得出现在 Lite removed:{removed:?}"
                    );
                }
                S2C::AgentStates { .. } => panic!("A 不得收到旧 AgentStates"),
                _ => {}
            }
        }
        if saw_possessed_none && removed_t0_tick.is_some() && flash_t1_tick.is_some() {
            break;
        }
    }
    assert!(saw_possessed_none, "Unpossess 必须收到 Possessed(None)");
    let t0_tick = removed_t0_tick.expect("Unpossess 必须收到 P1 Compact tier=0 removed=[e_near]");
    let t1_tick = flash_t1_tick.expect("Unpossess 后必须收到 Compact tier=1 含 e_near");
    assert!(
        t1_tick <= t0_tick + 1,
        "Unpossess 后 Compact tier=1 补闪必须在 <=1 拍内到达: t0_removed={t0_tick}, t1_flash={t1_tick}"
    );

    let _outcome = shutdown(h).await;
}
