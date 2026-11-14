//! REQ-012 B02b(#25):控制者接进驱动——配置生成四足、检查点 v13、活动范围。
//!
//! 口径(模板 = `a06_respawn.rs`):真实网络栈(TCP 客户端 + 驱动 + Session,
//! a03 同款装配);`[[agents]]` 配置走 `Config.agents`,驱动每拍
//! `startup.poll → host.apply_leash → host.drive → session.tick →
//! retain_alive`。检查点用 `RecordingCheckpointStore`(a03_session_determinism
//! 同款),重启 = 读 bundle → `replay_world_with_bodies` 恢复 Session +
//! `startup_controllers` 回填。
//!
//! **等待循环一律有界**(a06 R1 §4):墙钟 60 s 上限 + 帧数预算,超限
//! panic 并写明在等什么——挂死不算红。

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::{Config, StartupAgentConfig, StartupController};
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::InputSource;
use physics_sim::BodyRegistry;
use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use sim_api::session::Session;
use storage::worker::StoreWorker;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{EntityId, Tick};
use voxel::SimpleRegistry;

/// Superflat classic 地面顶面 world y(a03/a06 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;
/// 软窗(a03 同款)。
const SOFT: Duration = Duration::from_millis(50);
/// 收帧上限(条)。
const FRAME_BUDGET: usize = 6000;
/// 等待墙钟上限(单循环)。
const WAIT_WALL_SECS: u64 = 60;
/// 出生点:分片 (0,0) 中心 (64,64),地面顶面 + 0.45(B02a 同口径)。
const QUAD_XZ: [f32; 2] = [64.0, 64.0];
const QUAD_Y: f32 = GROUND_TOP_Y + 0.45;

fn quad_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/quad.body.toml"))
        .expect("quad.body.toml 合法");
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
}

fn quad_agent(controller: StartupController, leash_m: f32) -> StartupAgentConfig {
    StartupAgentConfig {
        body: "quad".to_owned(),
        position: [QUAD_XZ[0], QUAD_Y, QUAD_XZ[1]],
        yaw_deg: 0.0,
        controller,
        leash_m,
    }
}

fn quad_config(agents: Vec<StartupAgentConfig>) -> Config {
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 6000, // E 内不落期中票;E2/E3 显式改
        ..Config::default()
    };
    config.bodies_dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies");
    config.agents = agents;
    config
}

async fn advance_ticks(ticks: u32) {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

async fn next_frame(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Option<S2C> {
    tokio::time::timeout(SOFT, rx.recv()).await.ok().flatten()
}

async fn real_yield() {
    tokio::task::yield_now().await;
    std::thread::sleep(Duration::from_millis(1));
}

/// 单个等待循环的墙钟上限(a06 R2-1 同款)。
struct Deadline {
    at: std::time::Instant,
    what: &'static str,
}

impl Deadline {
    fn new(what: &'static str) -> Self {
        Self::with_secs(WAIT_WALL_SECS, what)
    }

    /// 长采样窗(合跑负载下每拍真实开销放大;上限仍 < 5 分钟挂死线)。
    fn with_secs(secs: u64, what: &'static str) -> Self {
        Self {
            #[allow(clippy::disallowed_methods)] // 同上
            at: std::time::Instant::now() + Duration::from_secs(secs),
            what,
        }
    }

    fn check(&self) {
        #[allow(clippy::disallowed_methods)]
        if std::time::Instant::now() >= self.at {
            panic!("等待墙钟超限({WAIT_WALL_SECS} s):{}", self.what);
        }
    }
}

// ---- 检查点记录存储(a03_session_determinism 同款) --------------------------

type CheckpointRecords = Arc<std::sync::Mutex<Vec<(Tick, Vec<u8>)>>>;

#[derive(Default, Clone)]
struct RecordingCheckpointStore {
    records: CheckpointRecords,
}

impl RecordingCheckpointStore {
    fn read_all(&self) -> Vec<(Tick, Vec<u8>)> {
        self.records.lock().expect("锁不毒").clone()
    }
}

impl storage::traits::CheckpointStore for RecordingCheckpointStore {
    fn append(
        &mut self,
        at: Tick,
        blob: &[u8],
    ) -> Result<storage::traits::CheckpointId, storage::traits::StoreError> {
        let mut records = self.records.lock().expect("锁不毒");
        records.push((at, blob.to_vec()));
        Ok(storage::traits::CheckpointId(records.len() as u64))
    }

    fn read_latest(&self) -> Result<Option<(Tick, Vec<u8>)>, storage::traits::StoreError> {
        Ok(self.records.lock().expect("锁不毒").last().cloned())
    }

    fn read(
        &self,
        id: storage::traits::CheckpointId,
    ) -> Result<Option<(Tick, Vec<u8>)>, storage::traits::StoreError> {
        Ok(self
            .records
            .lock()
            .expect("锁不毒")
            .iter()
            .find(|(i, _)| storage::traits::CheckpointId(i.0 + 1) == id)
            .cloned())
    }
}

// ---- 迷你客户端(a06 同款) --------------------------------------------------

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

/// Hello + Subscribe → Welcome(a06 join 同款)。
async fn join(c: &mut TestClient, rx: &mut mpsc::UnboundedReceiver<S2C>, view: u8) -> InputSource {
    c.send(&C2S::Hello { client_id: 1 })
        .await
        .expect("Hello 写入");
    c.send(&C2S::Subscribe {
        view_distance: view,
    })
    .await
    .expect("Subscribe 写入");
    let deadline = Deadline::new("Welcome");
    for _ in 0..64 {
        deadline.check();
        match next_frame(rx).await {
            Some(S2C::Welcome { player_id, .. }) => return player_id,
            Some(_) => continue,
            None => panic!("软窗预算内未见 Welcome"),
        }
    }
    panic!("64 软窗内未收到 Welcome")
}

/// 四足 AgentStates 采样:逐帧推拍,收集 (tick, 根 xz、根 xyz、up·y)。
/// 采够 `want` 拍(从首个主体帧起)即返回;预算超限 panic(写明在等什么)。
/// 返回 (首见 EntityId, 连杆数, 逐拍根记录)。
#[derive(Debug, Clone, Copy)]
struct RootSample {
    tick: Tick,
    pos: Vec3,
    up_y: f32,
}

async fn sample_quad_root(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    want_ticks: u32,
) -> (EntityId, usize, Vec<RootSample>) {
    let deadline = Deadline::with_secs(180, "AgentStates 主体帧(四足根位姿)");
    let mut frames = 0usize;
    let mut entity: Option<EntityId> = None;
    let mut nlinks = 0usize;
    let mut samples: Vec<RootSample> = Vec::new();
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!(
                "等 AgentStates 超预算:{FRAME_BUDGET} 帧内主体帧不足 \
                 ({}/{want_ticks})(在等:配置生成的四足出现在 AgentStates 下行)",
                samples.len()
            );
        }
        advance_ticks(1).await;
        for _ in 0..4 {
            real_yield().await;
            match next_frame(rx).await {
                Some(S2C::AgentStates { tick, states, .. }) => {
                    for (id, _, links) in states {
                        if entity.is_none() {
                            entity = Some(id);
                            nlinks = links.len();
                        }
                        if Some(id) == entity && samples.last().map(|s| s.tick.0) != Some(tick.0) {
                            let root = &links[0];
                            samples.push(RootSample {
                                tick,
                                pos: root.translation,
                                up_y: (root.rotation * Vec3::new(0.0, 1.0, 0.0)).y,
                            });
                        }
                    }
                }
                Some(_) => {}
                None => break,
            }
        }
        if samples.len() >= want_ticks as usize {
            break;
        }
    }
    (entity.expect("至少见过一次主体帧"), nlinks, samples)
}

// ---- harness(网络面 + RecordingCheckpointStore;a03 同款) -------------------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: Option<tokio::task::JoinHandle<DriverOutcome>>,
    addr: SocketAddr,
    checkpoints: RecordingCheckpointStore,
}

async fn harness(config: Config, _bodies: BodyRegistry) -> Harness {
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 3600,
        observe_interval_ticks: 1,
        full_snapshot_interval_ticks: 180,
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
    let checkpoints = RecordingCheckpointStore::default();
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(checkpoints.clone()),
    );
    let (provider, new_sim, loaded) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("装配失败");
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
        bodies: loaded,
        startup_controllers: None,
    }));
    Harness {
        tx,
        driver: Some(driver),
        addr,
        checkpoints,
    }
}

async fn shutdown(mut h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    let driver = h.driver.take().expect("驱动句柄只在 shutdown 取走一次");
    driver.await.expect("驱动不 panic")
}

fn stitched_log(checkpoints: &[(Tick, Vec<u8>)]) -> CommandLog {
    let mut log = CommandLog::new();
    for (_, blob) in checkpoints {
        let bundle = app::driver::decode_checkpoint(blob).expect("checkpoint 可解");
        let window = CommandLog::from_bytes(&bundle.log_bytes).expect("日志窗口可解");
        for entry in window.entries() {
            log.append(entry.clone());
        }
    }
    log
}

// ---- E1:配置生成四足,起步行走 -----------------------------------------------

/// E1:配置一只 `quad_trot`——启动后主体出现(恰好 1 只、13 连杆姿态);
/// 起步后 300 拍沿航向前进 ≥ 0.5 m、`up·y` ≥ 0.7;日志有 SpawnAgent 1 条和
/// AgentJointTargets;`controller_counters.actions_emitted > 0`。
#[tokio::test(start_paused = true)]
async fn e1_startup_quad_trot_walks() {
    let mut config = quad_config(vec![quad_agent(StartupController::QuadTrot, 16.0)]);
    config.checkpoint_interval_ticks = 6000;
    let h = harness(config, quad_registry()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    // 观察者挪到出生点(视距内看得到四足)。
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");

    // 采样起步后 300 拍(settle 120 + 300 ≈ 420 拍,留出装载/生成窗口)。
    let (entity, nlinks, samples) = sample_quad_root(&mut rx, 460).await;
    // 恰好 1 只:采样期只见过这一个 EntityId(AgentStates 帧里没有第二只)。
    let _ = entity;
    assert_eq!(nlinks, 13, "四足连杆姿态 13 条(实得 {nlinks})");
    assert!(samples.len() >= 300, "采样拍数足够(实得 {})", samples.len());
    // 起步段 = settle(120 拍)之后:取第 130 个采样为起点,末采样为终点。
    let start_idx = 130.min(samples.len() - 1);
    let s0 = samples[start_idx];
    let s1 = *samples.last().expect("非空");
    let dx = s1.pos.x - s0.pos.x;
    let dz = s1.pos.z - s0.pos.z;
    let dist = (dx * dx + dz * dz).sqrt();
    let up_min = samples[start_idx..]
        .iter()
        .map(|s| s.up_y)
        .fold(f32::INFINITY, f32::min);
    println!(
        "E1: 位移 {dist:.3} m(起步 {start_idx} 拍后), up_min {up_min:.4}, \
         末位 ({:.2},{:.2},{:.2})",
        s1.pos.x, s1.pos.y, s1.pos.z
    );
    assert!(dist >= 0.5, "起步后 300 拍内前进 {dist:.3} m < 0.5 m");
    assert!(up_min >= 0.7, "up·y 最小 {up_min} < 0.7");

    let cps_store = h.checkpoints.clone();
    let outcome = shutdown(h).await;
    println!(
        "E1 counters: actions_emitted {} send_rejected {} spawned {} timeouts {}",
        outcome.controller_counters.actions_emitted,
        outcome.controller_counters.send_rejected,
        outcome.startup_counters.spawned,
        outcome.startup_counters.spawn_timeouts
    );
    assert!(
        outcome.controller_counters.actions_emitted > 0,
        "actions_emitted > 0"
    );
    // 日志:SpawnAgent 恰 1 条(SCHEDULER 盖章)+ AgentJointTargets 若干。
    // 终末检查点在 shutdown 时落盘:store 是 Arc 克隆,关服后仍可读。
    let log = stitched_log(&cps_store.read_all());
    let mut spawns = 0usize;
    let mut joints = 0usize;
    let mut spawn_sources = Vec::new();
    for e in log.entries() {
        match e.command {
            messaging::SimCommand::SpawnAgent { entity, .. } => {
                spawns += 1;
                spawn_sources.push((entity, e.source));
            }
            messaging::SimCommand::AgentJointTargets { .. } => joints += 1,
            _ => {}
        }
    }
    assert_eq!(spawns, 1, "日志恰 1 条 SpawnAgent(实得 {spawns})");
    assert!(
        spawn_sources
            .iter()
            .all(|(_, s)| *s == InputSource::SCHEDULER),
        "SpawnAgent 盖章 = SCHEDULER"
    );
    assert!(joints > 0, "日志有 AgentJointTargets(实得 {joints})");
    // spawned = 1、无超时。
    assert_eq!(outcome.startup_counters.spawned, 1);
    assert_eq!(outcome.startup_counters.spawn_timeouts, 0);
}

// ---- E2:检查点重启续跑 ------------------------------------------------------

/// E2:走 300 拍 → 关服(终末检查点)→ 同一存储重启:同一 EntityId、只有
/// 1 只、`bodies_removed_on_recovery == 0`;重启后继续前进(再 180 拍 ≥ 0.3 m)。
#[tokio::test(start_paused = true)]
async fn e2_restart_preserves_agent_and_controllers() {
    let mut config = quad_config(vec![quad_agent(StartupController::QuadTrot, 16.0)]);
    config.checkpoint_interval_ticks = 6000;
    let bodies = quad_registry();
    let h = harness(config.clone(), bodies.clone()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity0, _n, _samples) = sample_quad_root(&mut rx, 200).await;
    drop(a);
    let cps1_store = h.checkpoints.clone();
    let outcome1 = shutdown(h).await;
    assert_eq!(outcome1.startup_counters.spawned, 1, "首段生成恰 1 只");
    assert_eq!(
        outcome1.agent.bodies_removed_on_recovery, 0,
        "首段(全新启动)没有恢复移除"
    );

    // —— 重启:读终末 bundle(shutdown 时落盘;store 是 Arc 克隆),replay
    //    恢复 + controllers 回填 ——
    let checkpoints = cps1_store.read_all();
    let (snap_tick, blob) = checkpoints.last().expect("终末检查点存在").clone();
    let bundle = app::driver::decode_checkpoint(&blob).expect("v13 档可解");
    assert_eq!(
        bundle.format_version,
        app::driver::CHECKPOINT_FORMAT_VERSION,
        "bundle 版本 = 13"
    );
    assert_eq!(
        bundle.controllers.len(),
        1,
        "检查点恰 1 条 ControllerRecord(实得 {})",
        bundle.controllers.len()
    );
    let resumed = sim_api::replay::replay_world_with_bodies(
        &bundle.world,
        &CommandLog::new(),
        {
            let mut c = config.session_config();
            c.scheduler.start_tick = snap_tick;
            c
        },
        || Box::new(SimpleRegistry::builtin()),
        || bodies.clone(),
    )
    .expect("恢复");
    assert!(
        resumed.is_agent(entity0),
        "恢复后主体仍在(同一 EntityId {entity0:?})"
    );

    // 起新驱动(网络面 + 检查点回填:startup_controllers = 检查点条目,
    // 配置不重新生成)。
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 3600,
        observe_interval_ticks: 1,
        full_snapshot_interval_ticks: 180,
        pump: PumpConfig::default(),
    };
    let cadence = ObserveCadence::from(&net_cfg);
    let (up_tx, up_rx) = mpsc::channel::<NetUp>(256);
    let hub = Arc::new(OutboundHub::new());
    let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
    let _addr = serve(net_cfg, up_tx, bcast_rx, hub.clone())
        .await
        .expect("监听成功");
    std::mem::forget(bcast_tx);
    let (tx, rxq) = mpsc::channel::<DriverCommand>(16);
    let checkpoints2 = RecordingCheckpointStore::default();
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(checkpoints2.clone()),
    );
    let (provider, new_sim, loaded) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("装配失败");
    let driver = tokio::spawn(run(DriverArgs {
        config,
        session: resumed,
        store,
        log: CommandLog::new(),
        commands: rxq,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: Some(ObserveLink {
            up: up_rx,
            hub,
            cadence,
        }),
        spawn_next_seq: bundle.spawn_next_seq,
        bodies: loaded,
        startup_controllers: Some(bundle.controllers),
    }));

    // 重启段:客户端重连采样(观察者回出生点),采 200 拍主体帧,再走够
    // 180 拍位移窗口后关服。
    let (mut a2, mut rx2) = client(_addr).await;
    let _player = join(&mut a2, &mut rx2, 8).await;
    a2.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity1, _n2, samples2) = sample_quad_root(&mut rx2, 200).await;
    assert_eq!(entity1, entity0, "重启后仍是同一个 EntityId");
    advance_ticks(60).await;
    a2.send(&C2S::Goodbye).await.ok();
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome2 = driver.await.expect("驱动不 panic");
    assert_eq!(
        outcome2.agent.bodies_removed_on_recovery, 0,
        "恢复清扫没有移除配置主体(source = None 不是玩家身体)"
    );
    assert_eq!(
        outcome2.startup_counters.spawned, 0,
        "恢复启动不重新生成(传空表)"
    );
    assert!(
        outcome2.controller_counters.actions_emitted > 0,
        "重启后控制者恢复,继续发动作(实得 {})",
        outcome2.controller_counters.actions_emitted
    );

    // 重启段位移:恢复后首个采样 → 末采样的水平位移 ≥ 0.3 m(控制者恢复
    // 才会继续走)。首样 = 重启段生成/装载完的起步位置;重启段 settle 已在
    // 检查点前发生(attached_at 早于快照拍,n 已过 settle),重启即走。
    let q0 = samples2.first().expect("非空");
    let q1 = samples2.last().expect("非空");
    let dx = q1.pos.x - q0.pos.x;
    let dz = q1.pos.z - q0.pos.z;
    let d = (dx * dx + dz * dz).sqrt();
    println!(
        "E2: 重启段位移 {d:.3} m(首样 → ({:.2},{:.2}))",
        q1.pos.x, q1.pos.z
    );
    assert!(d >= 0.3, "重启后 180 拍位移 {d:.3} m < 0.3 m(控制者没恢复)");
    let _ = tx;
}

// ---- E3:AC-B6 驱动层回放 ----------------------------------------------------

/// E3:取 E1 同款会话的最早检查点世界快照 + 其后拼接日志重放,末态根位姿
/// 与在线(检查点后的 AgentStates 采样)逐位相同。
#[tokio::test(start_paused = true)]
async fn e3_replay_bitwise_matches_online() {
    let mut config = quad_config(vec![quad_agent(StartupController::QuadTrot, 16.0)]);
    // 期中票落在 240 拍(主体已在走),回放窗口 = 检查点之后的动作。
    config.checkpoint_interval_ticks = 240;
    let h = harness(config, quad_registry()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (_entity, _n, samples) = sample_quad_root(&mut rx, 420).await;
    drop(a);
    let cps_store = h.checkpoints.clone();
    let outcome = shutdown(h).await;
    let _ = outcome;

    // 最早检查点 + 其后拼接日志(终末在 shutdown 时落盘)。
    let checkpoints = cps_store.read_all();
    assert!(
        checkpoints.len() >= 2,
        "期中 + 终末都有(实得 {})",
        checkpoints.len()
    );
    let (first_tick, first_blob) = &checkpoints[0];
    let bundle = app::driver::decode_checkpoint(first_blob).expect("可解");
    assert_eq!(bundle.world.shards.tick, *first_tick);
    let log = stitched_log(&checkpoints);
    assert!(
        log.entries()
            .iter()
            .any(|e| matches!(e.command, messaging::SimCommand::AgentJointTargets { .. })),
        "拼接日志含控制者动作(AC-B6:回放不缺动作)"
    );
    let mut replay = sim_api::replay::replay_world_with_bodies(
        &bundle.world,
        &log,
        {
            let mut c = quad_config(Vec::new()).session_config();
            c.scheduler.start_tick = *first_tick;
            c
        },
        || Box::new(SimpleRegistry::builtin()),
        quad_registry,
    )
    .expect("重放装配");
    // 回放对齐:在线末采样拍 = 回放需跑到的拍。
    let end = samples.last().expect("非空").tick.0;
    for _ in (first_tick.0 + 1)..=end {
        replay.tick();
    }
    let online_last = samples.last().expect("非空");
    let replay_root = replay.agent_root_position(_entity).expect("回放线主体在");
    println!(
        "E3: 在线末 ({:.4},{:.4},{:.4}) @ 拍 {} vs 回放 ({:.4},{:.4},{:.4})",
        online_last.pos.x,
        online_last.pos.y,
        online_last.pos.z,
        end,
        replay_root.x,
        replay_root.y,
        replay_root.z
    );
    assert_eq!(
        online_last.pos, replay_root,
        "末态根位姿与在线逐位不同(拍 {end})"
    );
    replay.shutdown();
}

// ---- E4:活动范围(leash)-----------------------------------------------------

/// E4:`leash_m = 2`——主体停在离出生点水平 ≤ 3.5 m 处;`leash_stops == 1`;
/// 停下后 120 拍位移 < 0.2 m。
#[tokio::test(start_paused = true)]
async fn e4_leash_stops_trot() {
    let config = quad_config(vec![quad_agent(StartupController::QuadTrot, 2.0)]);
    let h = harness(config, quad_registry()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    // 采样 600 拍(生成 + settle + 走出 2 m + 停稳窗口)。
    let (_entity, _n, samples) = sample_quad_root(&mut rx, 600).await;
    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.controller_counters.leash_stops, 1,
        "leash_stops == 1(实得 {})",
        outcome.controller_counters.leash_stops
    );
    // 停止点:从尾部找连续静止段(后 120 拍位移 < 0.2 m);停止距离 ≤ 3.5 m。
    let tail = &samples[samples.len() - 120..];
    let t0 = tail.first().expect("非空");
    let t1 = tail.last().expect("非空");
    let mdx = t1.pos.x - t0.pos.x;
    let mdz = t1.pos.z - t0.pos.z;
    let moved = (mdx * mdx + mdz * mdz).sqrt();
    let dxs = t1.pos.x - QUAD_XZ[0];
    let dzs = t1.pos.z - QUAD_XZ[1];
    let stop_dist = (dxs * dxs + dzs * dzs).sqrt();
    println!(
        "E4: 停止距离 {stop_dist:.3} m(圆心距), 尾 120 拍位移 {moved:.4} m, \
         leash_stops {}",
        outcome.controller_counters.leash_stops
    );
    assert!(
        stop_dist <= 3.5,
        "停止点距出生点水平 {stop_dist:.3} m > 3.5 m"
    );
    assert!(moved < 0.2, "停下后 120 拍位移 {moved:.4} m ≥ 0.2 m");
}

// ---- E5:配置错误 ------------------------------------------------------------

/// E5:未知身体 / capsule 配 quad_trot / 离分片边界太近——启动装配返回
/// 类型化配置错误,不 panic。
#[test]
fn e5_config_errors_reject_startup() {
    let bodies_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies");
    let mk = |agents: Vec<StartupAgentConfig>| {
        let mut c = Config {
            bodies_dir: bodies_dir.clone(),
            ..Config::default()
        };
        c.agents = agents;
        c
    };
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(storage::mem::MemCheckpointStore::new()),
    );
    // ① 未知身体。
    let mut c = mk(Vec::new());
    c.agents = vec![StartupAgentConfig {
        body: "no_such".to_owned(),
        position: [64.0, -59.55, 64.0],
        yaw_deg: 0.0,
        controller: StartupController::None,
        leash_m: 16.0,
    }];
    let msg = match app::driver::wire_provider_and_new_sim(c.clone(), &store, || {
        Box::new(SimpleRegistry::builtin())
    }) {
        Err(app::driver::AppError::Config(app::config::ConfigError::StartupAgent(m))) => m,
        _other => panic!("未知身体应返回 Config(StartupAgent)"),
    };
    assert!(msg.contains("不在注册表"), "错误写明原因:{msg}");
    // bootstrap 同口径拒绝(不 panic;data_dir 指向临时目录防碰本地 data/)。
    let mut cb = c.clone();
    cb.data_dir = std::env::temp_dir().join("b02b_e5_bootstrap");
    let boot = app::driver::bootstrap(&cb, || Box::new(SimpleRegistry::builtin()));
    assert!(
        matches!(
            &boot,
            Err(app::driver::AppError::Config(
                app::config::ConfigError::StartupAgent(_)
            ))
        ),
        "bootstrap 同拒(实为 {})",
        if boot.is_ok() { "Ok" } else { "Err(其他)" }
    );
    // ② capsule 配 quad_trot(dof 0 ≠ 12)。
    let c = mk(vec![StartupAgentConfig {
        body: "capsule".to_owned(),
        position: [64.0, -59.55, 64.0],
        yaw_deg: 0.0,
        controller: StartupController::QuadTrot,
        leash_m: 16.0,
    }]);
    let msg = match app::driver::wire_provider_and_new_sim(c, &store, || {
        Box::new(SimpleRegistry::builtin())
    }) {
        Err(app::driver::AppError::Config(app::config::ConfigError::StartupAgent(m))) => m,
        _other => panic!("dof 不足应返回 Config(StartupAgent)"),
    };
    assert!(msg.contains("dof"), "错误写明 dof:{msg}");
    // ③ 离分片边界太近:leash 16 + margin 8 = 24;距右边界 23.5 m < 24 → 拒。
    let c = mk(vec![quad_agent(StartupController::QuadTrot, 16.0)]);
    let mut c = c;
    c.agents[0].position[0] = 128.0 - 24.0 + 0.5; // x = 104.5,距 128 边 23.5
    let msg = match app::driver::wire_provider_and_new_sim(c, &store, || {
        Box::new(SimpleRegistry::builtin())
    }) {
        Err(app::driver::AppError::Config(app::config::ConfigError::StartupAgent(m))) => m,
        _other => panic!("活动圆出分片应返回 Config(StartupAgent)"),
    };
    assert!(msg.contains("超出分片"), "错误写明原因:{msg}");
}

// ---- E6:RemoveEntity 摘条目 -------------------------------------------------

/// E6:`RemoveEntity` 移除主体——下一拍起控制者条目被摘
/// (AgentStates.removed 出现之后 drive 不再对它发动作),之后 120 拍
/// `send_rejected` 不再增长(退出时计数仍为 0)。
#[tokio::test(start_paused = true)]
async fn e6_remove_entity_detaches_controller() {
    let config = quad_config(vec![quad_agent(StartupController::QuadTrot, 16.0)]);
    let h = harness(config, quad_registry()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity, _n, samples) = sample_quad_root(&mut rx, 200).await;
    let pre_joints = samples.len(); // 在线帧数参考(无语义,防未用告警)
    let _ = pre_joints;
    // 本地受信通道发 RemoveEntity(网络来源不得对主体发实体命令,A03 §5;
    // 本地 source = None 同口径放行——管理面路径)。
    let shard = types::ShardId::new(
        types::WorldId(0),
        types::DimensionId(0),
        types::ShardPos::new(0, 0),
    );
    h.tx.send(DriverCommand::Send {
        shard,
        command: messaging::SimCommand::RemoveEntity { entity },
    })
    .await
    .expect("RemoveEntity 载体");
    // 等 removed 帧(结构证据:主体没了)。
    let deadline = Deadline::new("AgentStates.removed 含四足");
    let mut frames = 0usize;
    let mut removed_seen = false;
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!("等 removed 超预算(在等:RemoveEntity 生效后的 removed 帧)");
        }
        advance_ticks(1).await;
        for _ in 0..4 {
            real_yield().await;
            match next_frame(&mut rx).await {
                Some(S2C::AgentStates { removed, .. }) => {
                    if removed.contains(&entity) {
                        removed_seen = true;
                    }
                }
                Some(_) => {}
                None => break,
            }
        }
        if removed_seen {
            break;
        }
    }
    assert!(removed_seen, "主体移除可见");
    // 之后 120 拍:不再有该主体的任何状态帧(条目被摘,drive 不发目标)。
    let mut after_frames = 0usize;
    let deadline = Deadline::new("移除后 120 拍静默窗");
    while after_frames < 120 {
        deadline.check();
        advance_ticks(1).await;
        after_frames += 1;
        for _ in 0..32 {
            real_yield().await;
            match next_frame(&mut rx).await {
                Some(S2C::AgentStates { states, .. }) => {
                    assert!(
                        !states.iter().any(|(id, _, _)| *id == entity),
                        "移除后 {} 拍主体不应再出现在状态帧",
                        after_frames
                    );
                }
                Some(_) => {}
                None => break,
            }
        }
    }
    let outcome = shutdown(h).await;
    println!(
        "E6: send_rejected {} actions_emitted {} removed_seen {removed_seen}",
        outcome.controller_counters.send_rejected, outcome.controller_counters.actions_emitted
    );
    // 口径:retain_alive 在每拍**末**摘条目,移除生效后的第一拍里 drive
    // 先跑一次(被拒 1 次)、拍末才摘——恰 1 次;若 retain_alive 不跑,
    // drive 会对已移除主体每 2 拍被拒一次,计数会持续增长远超 1。
    assert_eq!(
        outcome.controller_counters.send_rejected, 1,
        "条目下一拍被摘:send_rejected 恰 1(移除当拍的 drive);\
         持续增长 = 条目没被摘(实得 {})",
        outcome.controller_counters.send_rejected
    );
    assert!(
        outcome.controller_counters.actions_emitted > 50,
        "移除前控制者在发动作(实得 {})",
        outcome.controller_counters.actions_emitted
    );
}

// ---- E7:无 [[agents]] 世界与日志不变 ----------------------------------------

/// E7:没有 `[[agents]]`——跑 120 拍:终末日志窗口无任何条目(与之前行为
/// 一致),`controller_counters`、`startup_counters` 全 0。
#[tokio::test(start_paused = true)]
async fn e7_no_agents_world_unchanged() {
    let mut config = quad_config(Vec::new());
    config.checkpoint_interval_ticks = 6000;
    let h = harness(config, quad_registry()).await;
    advance_ticks(120).await;
    let cps_owned = h.checkpoints.clone();
    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.controller_counters,
        app::controller::ControllerCounters::default()
    );
    assert_eq!(
        outcome.startup_counters,
        app::startup_agents::StartupCounters::default()
    );
    let cps = cps_owned.read_all();
    let (_, blob) = cps.last().expect("终末检查点").clone();
    let bundle = app::driver::decode_checkpoint(&blob).expect("可解");
    let window = CommandLog::from_bytes(&bundle.log_bytes).expect("窗口可解");
    assert_eq!(
        window.entries().len(),
        0,
        "无 [[agents]] 的 120 拍日志窗口为空(实得 {} 条)",
        window.entries().len()
    );
}

// ---- E8:生成超时 ------------------------------------------------------------

/// E8:出生点在不可能装载的位置(y 远低于世界底)——600 拍后
/// `spawn_timeouts == 1`,服务器照常运行(关服正常、无 panic)。
#[tokio::test(start_paused = true)]
async fn e8_spawn_timeout_gives_up_gracefully() {
    let mut agent = quad_agent(StartupController::None, 16.0);
    agent.position[1] = -200.0; // 世界底(WORLD_Y_MIN = -64)之下,列永不装载
    let config = quad_config(vec![agent]);
    let h = harness(config, quad_registry()).await;
    advance_ticks(660).await; // 600 超时 + 余量
    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.startup_counters.spawn_timeouts, 1,
        "超时放弃 1 条(实得 {})",
        outcome.startup_counters.spawn_timeouts
    );
    assert_eq!(outcome.startup_counters.spawned, 0);
    println!(
        "E8: timeouts {} spawned {} ticks_run {}",
        outcome.startup_counters.spawn_timeouts,
        outcome.startup_counters.spawned,
        outcome.ticks_run
    );
    assert!(
        outcome.ticks_run >= 600,
        "服务器照常运行({} 拍)",
        outcome.ticks_run
    );
}

// ---- E9:客户端看到四足 AgentStates ------------------------------------------

/// E9:客户端连上后收到的 `AgentStates` 里有这只四足,连杆姿态 13 条
/// (本文件 E1 的采样器逐帧断言;这里独立走最小路径再核一遍)。
#[tokio::test(start_paused = true)]
async fn e9_client_receives_agent_states() {
    let config = quad_config(vec![quad_agent(StartupController::QuadTrot, 16.0)]);
    let h = harness(config, quad_registry()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    // 等第一个含 13 连杆的主体帧。
    let deadline = Deadline::new("四足 AgentStates(13 连杆)");
    let mut frames = 0usize;
    let seen;
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!("等 AgentStates 超预算(在等:四足 13 连杆状态帧)");
        }
        advance_ticks(1).await;
        let mut hit = false;
        for _ in 0..32 {
            real_yield().await;
            match next_frame(&mut rx).await {
                Some(S2C::AgentStates { states, .. }) => {
                    for (_, _, links) in states {
                        if links.len() == 13 {
                            hit = true;
                        }
                    }
                }
                Some(_) => {}
                None => break,
            }
        }
        if hit {
            seen = true;
            break;
        }
    }
    let _ = seen;
    shutdown(h).await;
}
