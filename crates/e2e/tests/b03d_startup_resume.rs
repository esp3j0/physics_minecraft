//! REQ-012 B03d(#25):启动主体两个遗留的端到端——
//!
//! Z1 生成前写检查点:重启走恢复分支,`[[agents]]` 里还没生成的那条要继续
//! 等、装载后生成(旧代码恢复分支传空配置表 → 永不再生成,Z1 在旧代码红)。
//!
//! 口径(夹具照 `b02b_startup_agents.rs`):真实网络栈(TCP 客户端 + 驱动 +
//! Session),重启 = 同 data_dir 再走一次**真实 `bootstrap`**(读终末 bundle →
//! replay 恢复 + 第 6 项原样透传)。**等待循环一律有界**:墙钟 60 s 上限 +
//! 帧数预算,超限 panic 并写明在等什么——挂死不算红。

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::{Config, StartupAgentConfig, StartupController};
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::InputSource;
use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use storage::traits::CheckpointStore as _;
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
        checkpoint_interval_ticks: 6000, // Z 内不落期中票;显式改才有
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
        Self {
            #[allow(clippy::disallowed_methods)] // 测试授权面
            at: std::time::Instant::now() + Duration::from_secs(WAIT_WALL_SECS),
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

// ---- 迷你客户端(a06/b02b 同款) ----------------------------------------------

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

/// 等首个含 13 连杆的 AgentStates 主体帧(有界:墙钟 + 帧数预算)。
/// 返回 (EntityId, 连杆数)。
async fn wait_quad_13_links(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    what: &'static str,
) -> (EntityId, usize) {
    let deadline = Deadline::new(what);
    let mut frames = 0usize;
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!("等 AgentStates 超预算:{FRAME_BUDGET} 帧内无 13 连杆主体帧(在等:{what})");
        }
        advance_ticks(1).await;
        for _ in 0..4 {
            real_yield().await;
            match next_frame(rx).await {
                Some(S2C::AgentStates { states, .. }) => {
                    for (id, _, links) in states {
                        if links.len() == 13 {
                            return (id, links.len());
                        }
                    }
                }
                Some(_) => {}
                None => break,
            }
        }
    }
}

// ---- harness(真实 bootstrap 起服 + 真实存储;b02b R1-1 同款) ----------------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: Option<tokio::task::JoinHandle<DriverOutcome>>,
    addr: SocketAddr,
    /// 数据目录(checkpoints.redb 所在;关服后读检查点用)。
    data_dir: std::path::PathBuf,
}

/// 关服后从 data_dir 读出全部检查点(ID 1..=latest)。
fn read_all_checkpoints(data_dir: &std::path::Path) -> Vec<(Tick, Vec<u8>)> {
    let store = storage::redb::RedbStore::open(&data_dir.join("checkpoints.redb"))
        .expect("checkpoint 库可开");
    let latest = store
        .read_latest()
        .expect("读最新不失败")
        .expect("至少一个检查点");
    let mut all = Vec::new();
    for id in 1..=latest.0.0 {
        if let Some(rec) = store
            .read(storage::traits::CheckpointId(id))
            .expect("读不失败")
        {
            all.push(rec);
        }
    }
    all
}

/// 起服辅助(b02b R1-1:**经真实 `bootstrap`**装配,第 6 项原样填进
/// `DriverArgs`;存储用 bootstrap 打开的 redb 文件,temp 下唯一目录)。
async fn bootstrap_harness(config: Config) -> Harness {
    // 目录唯一性:原子计数(并行测试同纳秒会撞 redb 文件锁)。
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let unique = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let data_dir = std::env::temp_dir().join(format!("b03d_{}_{}", std::process::id(), unique));
    // data_dir 必须指到 temp:bootstrap 从 config 读目录(默认 "data" = 仓库
    // 本地库,测试绝不碰)。
    let mut config = config;
    config.data_dir = data_dir.clone();
    let (session, log, store, _resume_from, spawn_next_seq, resume) =
        app::driver::bootstrap(&config, || Box::new(SimpleRegistry::builtin()))
            .expect("bootstrap 装配成功");

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

    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let (provider, new_sim, loaded) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("装配失败");
    let driver = tokio::spawn(run(DriverArgs {
        config,
        session,
        store,
        log,
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: Some(ObserveLink {
            up: up_rx,
            hub,
            cadence,
        }),
        spawn_next_seq,
        bodies: loaded,
        resume,
    }));
    Harness {
        tx,
        driver: Some(driver),
        addr,
        data_dir,
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

/// 重启段起驱动的网络面(E2 同款装配)。
async fn serve_restart_net() -> (SocketAddr, mpsc::Receiver<NetUp>, Arc<OutboundHub>) {
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
    let _ = cadence;
    (addr, up_rx, hub)
}

// ---- Z1:生成前写检查点,恢复分支续生成 ---------------------------------------

/// Z1:配置一条 `quad_trot`——启动后只推 1 拍就关服(主体还没生成,只有终末
/// 检查点带着"待生成项");重启走恢复分支 → 600 拍内生成(13 连杆主体帧)、
/// 挂控制者在发动作。**旧代码红**:恢复分支不带配置,这一条永不再生成。
#[tokio::test(start_paused = true)]
async fn z1_restart_with_pending_entry_still_spawns() {
    let mut config = quad_config(vec![quad_agent(StartupController::QuadTrot, 16.0)]);
    config.checkpoint_interval_ticks = 6000; // 首段只 1 拍,只有终末检查点
    let h = bootstrap_harness(config.clone()).await;
    // 只推 1 拍就关服:首拍只放票,出生列不可读 → 未生成。
    advance_ticks(1).await;
    let data_dir = h.data_dir.clone();
    let outcome1 = shutdown(h).await;
    assert_eq!(
        outcome1.startup_counters.spawned, 0,
        "布景自检:1 拍内不应已生成(若已生成,布景不成立,停下报告;实得 {})",
        outcome1.startup_counters.spawned
    );

    // 重启:同一 data_dir 再走一次**真实 bootstrap**(恢复分支;E2 同款,
    // 先读库再 bootstrap:redb 单写锁)。
    {
        let cps = read_all_checkpoints(&data_dir);
        let (_, blob) = cps.last().expect("终末检查点存在").clone();
        let bundle = app::driver::decode_checkpoint(&blob).expect("检查点可解");
        assert_eq!(
            bundle.format_version,
            app::driver::CHECKPOINT_FORMAT_VERSION,
            "bundle 版本 = 当前"
        );
    }
    let mut restart_config = config.clone();
    restart_config.data_dir = data_dir.clone();
    let (resumed, _log2, store2, resume_from, spawn_next_seq, resume) =
        app::driver::bootstrap(&restart_config, || Box::new(SimpleRegistry::builtin()))
            .expect("重启 bootstrap 装配成功");
    assert!(
        resume_from.is_some(),
        "重启段 bootstrap 走恢复分支(续跑自拍)"
    );

    let (addr, up_rx, hub) = serve_restart_net().await;
    let (tx, rxq) = mpsc::channel::<DriverCommand>(16);
    let (provider, new_sim, loaded) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store2, || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("装配失败");
    let driver = tokio::spawn(run(DriverArgs {
        config,
        session: resumed,
        store: store2,
        log: CommandLog::new(),
        commands: rxq,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: Some(ObserveLink {
            up: up_rx,
            hub,
            cadence: ObserveCadence::default(),
        }),
        spawn_next_seq,
        bodies: loaded,
        resume,
    }));

    // 恢复段:客户端连入 → 观察者到出生点 → 等 13 连杆主体帧(≤600 拍)。
    let (mut a, mut rx) = client(addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity, nlinks) = wait_quad_13_links(&mut rx, "Z1 恢复段主体帧(待生成项续生成)").await;
    assert_eq!(nlinks, 13, "13 连杆主体(实得 {nlinks})");
    let _ = entity;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome2 = driver.await.expect("驱动不 panic");
    assert_eq!(
        outcome2.startup_counters.spawned, 1,
        "恢复分支把待生成项补生成(实得 {})",
        outcome2.startup_counters.spawned
    );
    assert!(
        outcome2.controller_counters.actions_emitted > 0,
        "挂了控制者在发动作(实得 {})",
        outcome2.controller_counters.actions_emitted
    );
    assert!(
        outcome2.ticks_run < 600,
        "600 拍内生成(实得 {} 拍)",
        outcome2.ticks_run
    );
}

// ---- Z2:主体移除后撤票 -------------------------------------------------------

/// Z2:配置一条 `quad_stand`,生成后用 E6 的办法 RemoveEntity。之后 60 拍内
/// `tickets_released == 1`,票表里没有这张启动票(日志 RemoveAreaTicket 恰
/// 一条、同一保留区间票号)。
#[tokio::test(start_paused = true)]
async fn z2_remove_entity_releases_startup_ticket() {
    let config = quad_config(vec![quad_agent(StartupController::QuadStand, 16.0)]);
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity, _n) = wait_quad_13_links(&mut rx, "Z2 首段主体帧(生成)").await;
    // E6 同款:本地受信通道发 RemoveEntity(管理面路径)。
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
    // 之后 60 拍内撤票(等 release_dead 的 RemoveAreaTicket 进日志窗口;
    // 终末检查点在 shutdown 时写,直接关服后从检查点读)。
    advance_ticks(60).await;
    let cps_dir = h.data_dir.clone();
    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.startup_counters.tickets_released, 1,
        "主体没了撤启动票恰一次(实得 {})",
        outcome.startup_counters.tickets_released
    );
    // 日志:RemoveAreaTicket 恰一条、同一保留区间票号;SetAreaTicket 恰 1 条。
    let log = stitched_log(&read_all_checkpoints(&cps_dir));
    let mut set_ids: Vec<u64> = Vec::new();
    let mut removed_ids: Vec<u64> = Vec::new();
    for e in log.entries() {
        match e.command {
            messaging::SimCommand::SetAreaTicket { id, .. } => set_ids.push(id),
            messaging::SimCommand::RemoveAreaTicket { id } => removed_ids.push(id),
            _ => {}
        }
    }
    assert_eq!(
        set_ids,
        vec![app::startup_agents::STARTUP_TICKET_BASE],
        "全程放票恰 1 张(实得 {set_ids:?})"
    );
    assert_eq!(
        removed_ids,
        vec![app::startup_agents::STARTUP_TICKET_BASE],
        "撤的正是那张启动票(实得 {removed_ids:?})"
    );
}

// ---- Z3:生成 → 重启 → 移除 → 撤票(票对照关系进检查点) ----------------------

/// Z3:生成 → 关服(票对照进检查点 startup.tickets)→ 重启(恢复分支,
/// release_dead 用恢复回填的票表)→ RemoveEntity → 撤票。
#[tokio::test(start_paused = true)]
async fn z3_spawn_restart_then_remove_releases() {
    let mut config = quad_config(vec![quad_agent(StartupController::QuadStand, 16.0)]);
    config.checkpoint_interval_ticks = 6000;
    let h = bootstrap_harness(config.clone()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity0, _n) = wait_quad_13_links(&mut rx, "Z3 首段主体帧(生成)").await;
    drop(a);
    let data_dir = h.data_dir.clone();
    shutdown(h).await;

    // 重启段:真实 bootstrap(恢复分支)。
    {
        let cps = read_all_checkpoints(&data_dir);
        let (_, blob) = cps.last().expect("终末检查点存在").clone();
        let bundle = app::driver::decode_checkpoint(&blob).expect("检查点可解");
        assert_eq!(
            bundle.startup.tickets,
            vec![(entity0, app::startup_agents::STARTUP_TICKET_BASE)],
            "检查点带票对照关系(主体, BASE+0)"
        );
    }
    let mut restart_config = config.clone();
    restart_config.data_dir = data_dir.clone();
    let (resumed, _log2, store2, resume_from, spawn_next_seq, resume) =
        app::driver::bootstrap(&restart_config, || Box::new(SimpleRegistry::builtin()))
            .expect("重启 bootstrap 装配成功");
    assert!(resume_from.is_some(), "重启段走恢复分支");
    assert!(
        resumed.is_agent(entity0),
        "恢复后主体仍在(同一 EntityId {entity0:?})"
    );

    let (addr, up_rx, hub) = serve_restart_net().await;
    let (tx, rxq) = mpsc::channel::<DriverCommand>(16);
    let (provider, new_sim, loaded) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store2, || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("装配失败");
    let driver = tokio::spawn(run(DriverArgs {
        config,
        session: resumed,
        store: store2,
        log: CommandLog::new(),
        commands: rxq,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: Some(ObserveLink {
            up: up_rx,
            hub,
            cadence: ObserveCadence::default(),
        }),
        spawn_next_seq,
        bodies: loaded,
        resume,
    }));

    // 重连(观察者回出生点,主体保持),等主体帧再 RemoveEntity。
    let (mut a2, mut rx2) = client(addr).await;
    let _player = join(&mut a2, &mut rx2, 8).await;
    a2.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity1, _n2) = wait_quad_13_links(&mut rx2, "Z3 重启段主体帧").await;
    assert_eq!(entity1, entity0, "重启后同一主体");
    let shard = types::ShardId::new(
        types::WorldId(0),
        types::DimensionId(0),
        types::ShardPos::new(0, 0),
    );
    tx.send(DriverCommand::Send {
        shard,
        command: messaging::SimCommand::RemoveEntity { entity: entity1 },
    })
    .await
    .expect("RemoveEntity 载体");
    // 等 removed 帧。
    let deadline = Deadline::new("Z3 removed 帧");
    let mut frames = 0usize;
    let mut removed_seen = false;
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!("等 removed 超预算(在等:重启段 RemoveEntity 生效)");
        }
        advance_ticks(1).await;
        for _ in 0..4 {
            real_yield().await;
            match next_frame(&mut rx2).await {
                Some(S2C::AgentStates { removed, .. }) => {
                    if removed.contains(&entity1) {
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
    assert!(removed_seen, "重启段主体移除可见");
    advance_ticks(60).await;
    let cps_dir = data_dir;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome2 = driver.await.expect("驱动不 panic");
    assert_eq!(
        outcome2.startup_counters.tickets_released, 1,
        "重启段撤票(票对照来自检查点;实得 {})",
        outcome2.startup_counters.tickets_released
    );
    let _ = cps_dir;
}

// ---- Z4:两条配置各走一支(生成 / 永不装载超时) ------------------------------

/// Z4:第 1 条正常(生成);第 2 条出生点 y 在世界外(列恒不可读,永不装载,
/// E8 同款)。推到第 1 条生成后、第 2 条超时前关服;重启后:第 2 条继续等、
/// 到点放弃(`spawn_timeouts == 1`),第 1 条不重复生成(全程主体数恰 1、
/// 恢复段 `spawned == 0`)。
#[tokio::test(start_paused = true)]
async fn z4_two_entries_resume_waits_and_timeouts() {
    let mut second = quad_agent(StartupController::None, 16.0);
    second.position[1] = -200.0; // 世界底之下,列永不装载(E8 同款)
    let mut config = quad_config(vec![quad_agent(StartupController::QuadStand, 16.0), second]);
    config.checkpoint_interval_ticks = 6000;
    let h = bootstrap_harness(config.clone()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity0, _n) = wait_quad_13_links(&mut rx, "Z4 第 1 条生成").await;
    drop(a);
    let data_dir = h.data_dir.clone();
    let outcome1 = shutdown(h).await;
    assert_eq!(outcome1.startup_counters.spawned, 1, "首段只生成第 1 条");
    assert_eq!(outcome1.startup_counters.spawn_timeouts, 0, "第 2 条未超时");

    // 重启段:恢复分支——pending 只剩第 2 条(下标 1)。
    {
        let cps = read_all_checkpoints(&data_dir);
        let (_, blob) = cps.last().expect("终末检查点存在").clone();
        let bundle = app::driver::decode_checkpoint(&blob).expect("检查点可解");
        assert_eq!(
            bundle.startup.pending,
            vec![1],
            "待生成的只剩下标 1(第 2 条)"
        );
        assert_eq!(bundle.startup.tickets.len(), 1, "第 1 条的票在检查点");
    }
    let mut restart_config = config.clone();
    restart_config.data_dir = data_dir.clone();
    let (resumed, _log2, store2, resume_from, spawn_next_seq, resume) =
        app::driver::bootstrap(&restart_config, || Box::new(SimpleRegistry::builtin()))
            .expect("重启 bootstrap 装配成功");
    assert!(resume_from.is_some(), "重启段走恢复分支");

    let (addr, up_rx, hub) = serve_restart_net().await;
    let (tx, rxq) = mpsc::channel::<DriverCommand>(16);
    let (provider, new_sim, loaded) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store2, || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("装配失败");
    let driver = tokio::spawn(run(DriverArgs {
        config,
        session: resumed,
        store: store2,
        log: CommandLog::new(),
        commands: rxq,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: Some(ObserveLink {
            up: up_rx,
            hub,
            cadence: ObserveCadence::default(),
        }),
        spawn_next_seq,
        bodies: loaded,
        resume,
    }));

    // 重连,确认第 1 条不重复生成(仍是同一主体),再推过 600 拍等第 2 条超时。
    let (mut a2, mut rx2) = client(addr).await;
    let _player = join(&mut a2, &mut rx2, 8).await;
    a2.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity1, _n2) = wait_quad_13_links(&mut rx2, "Z4 重启段主体帧").await;
    assert_eq!(entity1, entity0, "第 1 条不重复生成(同一主体)");
    advance_ticks(660).await; // 600 超时 + 余量
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome2 = driver.await.expect("驱动不 panic");
    assert_eq!(
        outcome2.startup_counters.spawn_timeouts, 1,
        "重启段第 2 条到点放弃(实得 {})",
        outcome2.startup_counters.spawn_timeouts
    );
    assert_eq!(
        outcome2.startup_counters.spawned, 0,
        "恢复段不重复生成(实得 {})",
        outcome2.startup_counters.spawned
    );
    assert_eq!(
        outcome2.agent.bodies_removed_on_recovery, 0,
        "服务端主体不被恢复清扫移除"
    );
}

// ---- Z5:无 [[agents]] 世界与日志不变 -----------------------------------------

/// Z5:没有 `[[agents]]`——120 拍:`bundle.startup` 空、日志窗口 0 条、
/// startup_counters 全 0(无主体路径不变)。
#[tokio::test(start_paused = true)]
async fn z5_no_agents_startup_record_empty() {
    let mut config = quad_config(Vec::new());
    config.checkpoint_interval_ticks = 6000;
    let h = bootstrap_harness(config).await;
    advance_ticks(120).await;
    let cps_dir = h.data_dir.clone();
    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.startup_counters,
        app::startup_agents::StartupCounters::default()
    );
    let cps = read_all_checkpoints(&cps_dir);
    let (_, blob) = cps.last().expect("终末检查点").clone();
    let bundle = app::driver::decode_checkpoint(&blob).expect("可解");
    assert_eq!(
        bundle.startup,
        app::startup_agents::StartupRecord::default(),
        "无 [[agents]] 的 startup 记录为空"
    );
    let window = CommandLog::from_bytes(&bundle.log_bytes).expect("窗口可解");
    assert_eq!(
        window.entries().len(),
        0,
        "无 [[agents]] 的 120 拍日志窗口为空(实得 {} 条)",
        window.entries().len()
    );
}
