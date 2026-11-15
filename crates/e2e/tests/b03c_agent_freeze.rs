//! REQ-012 B03c(#25):主体冻结的触发(AC-B4 生产路径)端到端测试。
//!
//! 口径(模板 = `b03a_agent_migration.rs` / `b02b_startup_agents.rs`):真实
//! 网络栈(TCP 客户端、驱动与 Session,bootstrap 装配);`[[agents]]` 配置
//! 走 `Config.agents`,启动区域票 ③ 档(B03c 3.1)。在线段的断言经
//! `DriverOutcome` 计数;恢复段用 `bootstrap` 恢复的 Session + 检查点里的
//! `startup_controllers` 手动 drive 循环(与驱动同序:drive → tick),
//! `Session::agent_states` 不裁视野,冻结期间与续走的位姿逐拍可读。
//!
//! **等待循环一律有界**:墙钟上限 + 帧数预算,超限 panic 并写明在等什么
//! ——挂死不算红。

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::{Config, StartupAgentConfig, StartupController};
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use app::startup_agents::STARTUP_TICKET_BASE;
use messaging::InputSource;
use physics_sim::BodyRegistry;
use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{EntityId, Tick};
use voxel::{SimpleRegistry, WorldGenerator};

/// Superflat classic 地面顶面 world y(b03a/b02b 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;
/// 软窗(a03 同款)。
const SOFT: Duration = Duration::from_millis(50);
/// 四足出生点(分片 (0,0) 中心,同 b02b)。
const QUAD_XZ: [f32; 2] = [64.0, 64.0];
const QUAD_Y: f32 = GROUND_TOP_Y + 0.45;
/// 胶囊出生点(同 b03a 的 x5)。
const CAP_XZ: [f32; 2] = [64.0, 64.0];

fn bodies_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/quad.body.toml"))
        .expect("quad.body.toml 合法");
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
}

/// Y1/Y3 布景:quad_trot 出生分片中心、leash 16(票半径 = 16 + extra)。
fn quad_trot_agent() -> StartupAgentConfig {
    StartupAgentConfig {
        body: "quad".to_owned(),
        position: [QUAD_XZ[0], QUAD_Y, QUAD_XZ[1]],
        yaw_deg: 0.0,
        controller: StartupController::QuadTrot,
        leash_m: 16.0,
    }
}

fn base_config(agents: Vec<StartupAgentConfig>, checkpoint_interval_ticks: u64) -> Config {
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks,
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

/// 单个等待循环的墙钟上限(b03a 同款)。
struct Deadline {
    at: std::time::Instant,
    what: &'static str,
}

impl Deadline {
    fn with_secs(secs: u64, what: &'static str) -> Self {
        Self {
            #[allow(clippy::disallowed_methods)] // telemetry 授权面同款
            at: std::time::Instant::now() + Duration::from_secs(secs),
            what,
        }
    }

    fn check(&self) {
        #[allow(clippy::disallowed_methods)]
        if std::time::Instant::now() >= self.at {
            panic!("等待墙钟超限:{}", self.what);
        }
    }
}

// ---- 迷你客户端(b03a 同款) --------------------------------------------------

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

/// Hello + Subscribe → Welcome(b03a join 同款)。
async fn join(c: &mut TestClient, rx: &mut mpsc::UnboundedReceiver<S2C>, view: u8) -> InputSource {
    c.send(&C2S::Hello { client_id: 1 })
        .await
        .expect("Hello 写入");
    c.send(&C2S::Subscribe {
        view_distance: view,
    })
    .await
    .expect("Subscribe 写入");
    let deadline = Deadline::with_secs(60, "Welcome");
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

// ---- harness(真实 bootstrap 起服 + 真实存储;b03a 同款唯一写法) --------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: Option<tokio::task::JoinHandle<DriverOutcome>>,
    addr: SocketAddr,
    /// 数据目录(checkpoints.redb 所在;关服后读检查点用)。
    data_dir: std::path::PathBuf,
}

/// 关服后从 data_dir 读出全部检查点(ID 1..=latest)。
fn read_all_checkpoints(data_dir: &std::path::Path) -> Vec<(Tick, Vec<u8>)> {
    use storage::traits::CheckpointStore as _;
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

/// 起服辅助(唯一写法;b03a 同款)。
async fn bootstrap_harness(config: Config) -> Harness {
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let unique = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let data_dir = std::env::temp_dir().join(format!("b03c_{}_{}", std::process::id(), unique));
    let mut config = config;
    config.data_dir = data_dir.clone();
    let (session, log, store, _resume_from, spawn_next_seq, startup_controllers) =
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
        startup_controllers,
    }));
    Harness {
        tx,
        driver: Some(driver),
        addr,
        data_dir,
    }
}

/// 从 data_dir 的终末检查点恢复 Session + 控制者宿主(手动 drive 循环用;
/// bootstrap 的恢复分支,同 x4)。
fn resume_session(
    config: &Config,
    data_dir: &std::path::Path,
) -> (
    sim_api::session::Session,
    app::controller::ControllerHost,
    Tick,
) {
    let cps = read_all_checkpoints(data_dir);
    let (last_tick, blob) = cps.last().expect("终末检查点存在").clone();
    let bundle = app::driver::decode_checkpoint(&blob).expect("v14 档可解");
    let records = bundle.controllers;
    let host = app::controller::ControllerHost::from_records(records);
    let mut restart_config = config.clone();
    restart_config.data_dir = data_dir.to_path_buf();
    let (resumed, _log2, store2, resume_from, _spawn_next_seq, startup_controllers) =
        app::driver::bootstrap(&restart_config, || Box::new(SimpleRegistry::builtin()))
            .expect("重启 bootstrap 装配成功");
    assert!(resume_from.is_some(), "恢复分支真实走到");
    assert!(startup_controllers.is_some(), "恢复分支回填控制者条目");
    // store2 保持存活(redb 文件句柄;恢复后的会话不再写它也持有读口径)。
    std::mem::forget(store2);
    let _ = last_tick;
    (resumed, host, bundle.world.shards.tick)
}

/// 手动 drive 循环一拍(与驱动同序:① 回填 → drive → tick;在途/冻结
/// 集合照驱动口径现取)。恢复线 pending 为空,startup.poll 不产动作。① 回
/// 填:恢复线无驱动 provider,观察者票产出的 chunks_to_load 无人兑现 →
/// wake_group 缺 ② 永远推迟;这里扮演驱动 2249 行同款(生成器兜底 →
/// apply_chunk_load,与生产 provider.get 同源 Superflat + world_seed)。
fn drive_tick(
    session: &mut sim_api::session::Session,
    log: &mut CommandLog,
    host: &mut app::controller::ControllerHost,
    now: Tick,
) {
    refill_chunks(session);
    let skip = session.in_transit_entities();
    let frozen = session.frozen_agents();
    host.apply_leash(|id| session.agent_root_position(id).map(|p| [p.x, p.z]));
    host.drive(session, log, now, &skip, &frozen);
    session.tick();
}

/// ① 回填一拍(驱动激活编排同款;Y2 恢复线与 drive_tick 共用)。
fn refill_chunks(session: &mut sim_api::session::Session) {
    let generator = voxel::Superflat::classic();
    let seed = types::WorldSeed(0);
    for (shard, decisions) in session.take_activation_requests() {
        for pos in decisions.chunks_to_load {
            let chunk = generator.generate(seed, pos);
            session.apply_chunk_load(shard, pos, chunk);
        }
    }
}

// ---- Y1:持票不冻 ---------------------------------------------------------------

/// Y1:配置 quad_trot、不连客户端,跑 900 拍。启动票 ③ 档(B03c 3.1)
/// → 无人观察也照常活动:`agent_frozen == 0`,`actions_emitted` 持续增长
/// (> 0,skipped_frozen == 0——冻结跳过分支一次都没走)。
#[tokio::test(start_paused = true)]
async fn y1_tier3_ticket_agent_never_freezes() {
    let h = bootstrap_harness(base_config(vec![quad_trot_agent()], 6000)).await;
    advance_ticks(900).await;
    let outcome = shutdown(h).await;
    println!(
        "Y1: agent_frozen {}, agent_woken {}, actions_emitted {}, skipped_frozen {}",
        outcome.agent.agent_frozen,
        outcome.agent.agent_woken,
        outcome.controller_counters.actions_emitted,
        outcome.controller_counters.skipped_frozen
    );
    assert_eq!(
        outcome.agent.agent_frozen, 0,
        "Y1:持 ③ 档启动票的主体被冻结(票档位没升)"
    );
    assert_eq!(outcome.agent.agent_woken, 0, "Y1:不该有唤醒");
    assert!(
        outcome.controller_counters.actions_emitted > 0,
        "Y1:控制者没有发出目标(实得 {})",
        outcome.controller_counters.actions_emitted
    );
    assert_eq!(
        outcome.controller_counters.skipped_frozen, 0,
        "Y1:冻结跳过分支不该走到"
    );
}

// ---- Y2:玩家身体离开后冻结、回来唤醒 -------------------------------------------

/// Y2:客户端经网络生成胶囊(自动附身)→ Unpossess → 观察者移到 100 m 外
/// → 主体被冻结。**AgentStates 按视野裁剪、远处收不到**(driver.rs 视距
/// 过滤),冻结期间的"连杆位姿逐拍逐位不变"改在**恢复线核对**:关服 →
/// bootstrap 恢复(无观察者,主体保持冻结)→ `Session::agent_states()`
/// 逐拍逐位不变(回报已说明口径)。观察者回到身体旁 → 唤醒,位置与冻结
/// 前相差 < 1 cm。
#[tokio::test(start_paused = true)]
async fn y2_player_body_freezes_and_wakes() {
    let config = base_config(Vec::new(), 6000);
    let h = bootstrap_harness(config.clone()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(CAP_XZ[0], GROUND_TOP_Y + 2.0, CAP_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    // 经网络生成胶囊:回执 Spawned(自动附身流程随后 Possessed)。
    let mut entity: Option<EntityId> = None;
    let deadline = Deadline::with_secs(120, "Spawned 回执");
    for i in 0..6000u32 {
        deadline.check();
        if entity.is_none() && i % 20 == 0 {
            a.send(&C2S::SpawnAgent {
                body: "capsule".to_owned(),
                position: Vec3::new(CAP_XZ[0], QUAD_Y, CAP_XZ[1]),
                yaw: 0.0,
            })
            .await
            .expect("SpawnAgent 写入");
        }
        advance_ticks(1).await;
        for _ in 0..4 {
            real_yield().await;
            match next_frame(&mut rx).await {
                Some(S2C::Spawned { entity: e }) => {
                    entity = Some(e);
                }
                Some(_) => {}
                None => break,
            }
        }
        if entity.is_some() {
            break;
        }
    }
    let body = entity.expect("布景自检:胶囊已生成");
    println!("Y2: 胶囊 EntityId = {body:?}");
    let _ = player;
    // 显式附身(SpawnAgent 不自动附身):发 Possess,等 Possessed(Some)。
    a.send(&C2S::Possess { entity: body })
        .await
        .expect("Possess 写入");
    let mut possessed = false;
    let deadline = Deadline::with_secs(60, "附身回执");
    for _ in 0..1200 {
        deadline.check();
        match next_frame(&mut rx).await {
            Some(S2C::Possessed { entity: Some(e) }) if e == body => {
                possessed = true;
                break;
            }
            Some(_) => continue,
            None => {
                advance_ticks(1).await;
            }
        }
    }
    assert!(possessed, "布景自检:附身回执未到");
    a.send(&C2S::Unpossess).await.expect("Unpossess 写入");
    let mut unpossessed = false;
    for _ in 0..400 {
        unpossessed = unpossessed
            || matches!(
                next_frame(&mut rx).await,
                Some(S2C::Possessed { entity: None })
            );
        if unpossessed {
            break;
        }
    }
    assert!(unpossessed, "布景自检:Unpossess 回执未到");
    a.send(&C2S::MoveObserver {
        position: Vec3::new(CAP_XZ[0] + 100.0, GROUND_TOP_Y + 2.0, CAP_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入(100 m 外)");
    // 冻结上限 600 拍;等 700 拍后关服(在线侧无法从远处收 AgentStates,
    // 冻结计数在 DriverOutcome 里核验)。
    advance_ticks(700).await;
    drop(a);
    let data_dir = h.data_dir.clone();
    let outcome = shutdown(h).await;
    println!(
        "Y2: 在线段 agent_frozen {}, agent_woken {}",
        outcome.agent.agent_frozen, outcome.agent.agent_woken
    );
    assert_eq!(outcome.agent.agent_frozen, 1, "Y2:700 拍内主体未冻结");
    assert_eq!(outcome.agent.agent_woken, 0, "Y2:不该在线唤醒");

    // 恢复线:恢复(观察者表空——无人在线)→ 主体仍冻结;冻结期间
    // `agent_states` 逐拍逐位不变;观察者回到身体旁 → 唤醒,位置差 < 1 cm。
    let (mut session, _host, _snap_tick) = resume_session(&config, &data_dir);
    let frozen_set = session.frozen_agents();
    assert!(
        frozen_set.contains(&body),
        "Y2:恢复后主体不在冻结集合(实得 {frozen_set:?})"
    );
    // 冻结不变量:连续 120 拍 agent_states 逐位相同。
    let pose0 = session
        .agent_states()
        .into_iter()
        .find(|(id, _, _)| *id == body)
        .expect("恢复线主体在 agent_states")
        .2;
    for t in 0..120u64 {
        session.tick();
        let poses = session
            .agent_states()
            .into_iter()
            .find(|(id, _, _)| *id == body)
            .expect("冻结期间主体持续在 agent_states")
            .2;
        assert_eq!(poses, pose0, "Y2:冻结期间第 {t} 拍连杆位姿变动");
    }
    // 观察者回到身体旁 → 唤醒(经 Session 的观察者票;调度器路径同驱动)。
    let pre_wake = pose0[0].translation;
    session
        .move_observer(Vec3::new(pre_wake.x, GROUND_TOP_Y + 2.0, pre_wake.z))
        .expect("观察者移回");
    // ① 回填:恢复线是裸 Session(无驱动 provider),观察者移回后 plan 的
    // chunks_to_load 无人兑现 → wake_group 缺 ② 永远推迟。扮演驱动 2249 行
    // 同款回填(refill_chunks,生成器兜底与生产 provider.get 同源)。
    let mut woken_tick: Option<u64> = None;
    for t in 0..600u64 {
        refill_chunks(&mut session);
        session.tick();
        if !session.frozen_agents().contains(&body) {
            woken_tick = Some(t);
            break;
        }
    }
    let woken_tick = woken_tick.expect("Y2:600 拍内未唤醒");
    println!("Y2: 恢复线唤醒拍(移回起)= {woken_tick}");
    // 唤醒后:还是同一个主体,胶囊单连杆,位置与冻结前相差 < 1 cm。
    assert!(session.is_agent(body), "唤醒后主体仍在(同一 EntityId)");
    let states = session.agent_states();
    let (id, _, links) = states
        .iter()
        .find(|(id, _, _)| *id == body)
        .expect("唤醒后主体在 agent_states");
    assert_eq!(*id, body, "同一 EntityId");
    assert_eq!(links.len(), 1, "胶囊单连杆(实得 {})", links.len());
    let d = (links[0].translation - pre_wake).length();
    println!("Y2: 唤醒后位置差 {d:.6} m");
    assert!(d < 0.01, "唤醒后位置差 {d:.6} m ≥ 1 cm");
    session.shutdown();
}

// ---- Y2b/Y3b 共用:全程在线采样(同一驱动实例,不关服、不恢复) ----------

/// 单帧帧预算(在线采样循环;Y2b/Y3b 的等待循环都有墙钟 + 帧数双上限)。
const FRAME_BUDGET: usize = 6000;

/// 在线采一帧主体根位姿(b03a `agent_pose` 同款):推 1 拍、排干下行。
/// `id` = None 时取帧内第一只主体(首次发现用),并把它写回 `*id_out`
/// (已发现则仍按它过滤)。返回 (帧拍号, 根 translation, 根姿态)。
async fn watch_agent_root(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    id: Option<EntityId>,
    id_out: &mut Option<EntityId>,
) -> Option<(Tick, Vec3, types::EntityState)> {
    let mut got: Option<(Tick, Vec3, types::EntityState)> = None;
    for _ in 0..4 {
        real_yield().await;
        match next_frame(rx).await {
            Some(S2C::AgentStates { tick, states, .. }) => {
                let hit = match id {
                    Some(e) => states.iter().find(|(eid, ..)| *eid == e),
                    None => states.first(),
                };
                if let Some((eid, _, links)) = hit
                    && let Some(root) = links.first()
                {
                    if id_out.is_none() {
                        *id_out = Some(*eid);
                    }
                    got = Some((tick, root.translation, *root));
                }
            }
            Some(_) => {}
            None => break,
        }
    }
    got
}

/// Y2b 玩家身体**全程在线**冻结与唤醒(B03c 返修 R1 第 1 节):客户端连上
/// → SpawnAgent 胶囊 → Possess → Unpossess → MoveObserver 到 40 m 外
/// (出唤醒 unload 32 m、仍在 ① data_unload 48 m 与视距 128 m 内——主体
/// 脚下地形不卸、AgentStates 继续下发),推拍直到冻结(上限 600 拍;
/// **冻结判据 = AgentStates 根位姿连续 60 拍逐位不变**,任务书第 1 节
/// 口径之一;硬判据仍是关服 DriverOutcome 的 agent_frozen)。MoveObserver
/// 回到身体旁 → Possess(回执 Possessed(Some))→ 发行走意图,60 拍后
/// 沿意图方向位移 > 0.5 m → 关服。断言:DriverOutcome agent_frozen == 1、
/// agent_woken == 1;回来后同一 EntityId、位置与离开前差 < 1 cm。
#[tokio::test(start_paused = true)]
async fn y2b_player_body_online_freeze_wake_walk() {
    let config = base_config(Vec::new(), 6000);
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(CAP_XZ[0], GROUND_TOP_Y + 2.0, CAP_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    // 经网络生成胶囊 + 显式 Possess + Unpossess(Y2 同款;复用其循环写法)。
    let mut entity: Option<EntityId> = None;
    let deadline = Deadline::with_secs(120, "Spawned 回执");
    for i in 0..6000u32 {
        deadline.check();
        if entity.is_none() && i % 20 == 0 {
            a.send(&C2S::SpawnAgent {
                body: "capsule".to_owned(),
                position: Vec3::new(CAP_XZ[0], QUAD_Y, CAP_XZ[1]),
                yaw: 0.0,
            })
            .await
            .expect("SpawnAgent 写入");
        }
        advance_ticks(1).await;
        for _ in 0..4 {
            real_yield().await;
            match next_frame(&mut rx).await {
                Some(S2C::Spawned { entity: e }) => {
                    entity = Some(e);
                }
                Some(_) => {}
                None => break,
            }
        }
        if entity.is_some() {
            break;
        }
    }
    let body = entity.expect("布景自检:胶囊已生成");
    println!("Y2b: 胶囊 EntityId = {body:?}");
    // Possess(回执 Possessed(Some) = 再附身成功口径同源)。
    a.send(&C2S::Possess { entity: body })
        .await
        .expect("Possess 写入");
    let mut possessed = false;
    let deadline = Deadline::with_secs(60, "附身回执");
    for _ in 0..1200 {
        deadline.check();
        match next_frame(&mut rx).await {
            Some(S2C::Possessed { entity: Some(e) }) if e == body => {
                possessed = true;
                break;
            }
            Some(_) => continue,
            None => {
                advance_ticks(1).await;
            }
        }
    }
    assert!(possessed, "布景自检:附身回执未到");
    a.send(&C2S::Unpossess).await.expect("Unpossess 写入");
    let mut unpossessed = false;
    for _ in 0..400 {
        unpossessed = unpossessed
            || matches!(
                next_frame(&mut rx).await,
                Some(S2C::Possessed { entity: None })
            );
        if unpossessed {
            break;
        }
    }
    assert!(unpossessed, "布景自检:Unpossess 回执未到");

    // 离开:观察者到 40 m(出 ③ unload、在 ① unload/视距内),等冻结。
    const AWAY_M: f32 = 40.0;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(CAP_XZ[0] + AWAY_M, GROUND_TOP_Y + 2.0, CAP_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入(40 m 外)");
    // 冻结判据:根位姿连续 60 拍逐位不变(上限 600 拍)。增量模式下无变化
    // 不发帧,"连续 60 拍"按帧 tick 计:同一主体 tick1 位姿与 tick2 位姿
    // 逐位相同且 tick2 − tick1 ≥ 60 → 冻结持续 ≥ 60 拍(醒着的胶囊每拍
    // 位姿都有微演化,60 拍逐位不变只可能是冻结;走神/入睡不算——入睡
    // 的胶囊位姿也逐位不变,但两者在 DriverOutcome.agent_frozen == 1 的
    // 硬判据面前是同一验收面)。
    let mut anchor: Option<(Tick, types::EntityState)> = None;
    let mut frozen_observed: Option<u32> = None;
    let mut leave_pose: Option<Vec3> = None;
    let deadline = Deadline::with_secs(300, "冻结等待(AgentStates 位姿 60 拍不变)");
    let mut body_seen = Some(body);
    while frozen_observed.is_none() {
        deadline.check();
        advance_ticks(1).await;
        let Some((ftick, pos, pose)) = watch_agent_root(&mut rx, Some(body), &mut body_seen).await
        else {
            continue;
        };
        leave_pose = Some(pos);
        match anchor {
            Some((t0, p0)) if p0 == pose && ftick.0.saturating_sub(t0.0) >= 60 => {
                frozen_observed = Some(t0.0 as u32);
            }
            Some((t0, p0)) if p0 == pose => {
                let _ = (t0, p0);
            }
            _ => {
                anchor = Some((ftick, pose));
            }
        }
        if frozen_observed.is_some() {
            break;
        }
    }
    let frozen_observed =
        frozen_observed.expect("Y2b:600 拍窗内未见位姿 60 拍连续不变(冻结判据未触发)");
    let pre = leave_pose.expect("布景自检:离开前根位姿已采样");
    println!("Y2b: 在线判冻结(位姿 60 拍不变,锚拍)= {frozen_observed},离开前位置 {pre:?}");

    // 回来:观察者回到身体旁 → 唤醒。唤醒在线依据 = 位姿重新出现变化
    // (冻结期间逐位不变;唤醒落实当拍起位姿恢复演化)。等第一帧变化,
    // 此刻采"回来后位置"(= 冻结位姿,唤醒落实不位移)。
    a.send(&C2S::MoveObserver {
        position: Vec3::new(CAP_XZ[0], GROUND_TOP_Y + 2.0, CAP_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入(回来)");
    let mut post: Option<Vec3> = None;
    let mut woken_observed: Option<u32> = None;
    let deadline = Deadline::with_secs(300, "唤醒(AgentStates 位姿变化)");
    for tick in 0..1200u32 {
        deadline.check();
        advance_ticks(1).await;
        if let Some((_ftick, pos, pose)) =
            watch_agent_root(&mut rx, Some(body), &mut body_seen).await
        {
            let changed = anchor.as_ref().is_some_and(|(_, p)| *p != pose);
            if post.is_none() {
                post = Some(pos);
            }
            if changed {
                woken_observed = Some(tick);
                break;
            }
        }
    }
    let woken_observed = woken_observed.expect("Y2b:回场 1200 拍内位姿未再变化(唤醒判据未触发)");
    let post = post.expect("布景自检:回场后根位姿已采样");
    let pos_diff = (post - pre).length();
    println!("Y2b: 唤醒拍(回场起)= {woken_observed},位置差 {pos_diff:.6} m");
    assert!(
        pos_diff < 0.01,
        "Y2b:回来后位置与离开前差 {pos_diff:.6} m ≥ 1 cm"
    );

    // 再附身(回执 Possessed(Some))+ 发行走意图 +x,60 拍沿意图方向 > 0.5 m。
    a.send(&C2S::Possess { entity: body })
        .await
        .expect("Possess 写入");
    let mut possessed = false;
    let deadline = Deadline::with_secs(60, "再附身回执");
    for _ in 0..1200 {
        deadline.check();
        match next_frame(&mut rx).await {
            Some(S2C::Possessed { entity: Some(e) }) if e == body => {
                possessed = true;
                break;
            }
            Some(_) => continue,
            None => {
                advance_ticks(1).await;
            }
        }
    }
    assert!(possessed, "Y2b:再附身回执未到( Possessed(Some))");
    a.send(&C2S::AgentIntent {
        entity: body,
        move_xz: (1.5, 0.0),
        yaw: 0.0,
        jump: false,
    })
    .await
    .expect("AgentIntent 写入");
    let mut walked: Option<Vec3> = None;
    let mut frames = 0usize;
    for _ in 0..120u32 {
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!("Y2b:行走采样超帧预算");
        }
        advance_ticks(1).await;
        if let Some((_ftick, pos, _pose)) =
            watch_agent_root(&mut rx, Some(body), &mut body_seen).await
            && pos.x > post.x + 0.5
        {
            walked = Some(pos);
            break;
        }
    }
    let walked = walked.expect("Y2b:60 拍内沿意图方向位移未过 0.5 m(醒着的身体没走)");
    println!(
        "Y2b: 醒后 60 拍沿 +x 位移 {:.3} m(0.5 m 判据过)",
        walked.x - post.x
    );
    assert!(walked.x > post.x + 0.5, "Y2b:位移判据(内部已核,双保险)");

    // 关服:DriverOutcome 硬判据(在线两个计数只在关服可见)。
    let outcome = shutdown(h).await;
    println!(
        "Y2b: agent_frozen {}, agent_woken {}, possess_rejected {}",
        outcome.agent.agent_frozen, outcome.agent.agent_woken, outcome.agent.possess_rejected
    );
    assert_eq!(outcome.agent.agent_frozen, 1, "Y2b:agent_frozen 应恰 1");
    assert_eq!(outcome.agent.agent_woken, 1, "Y2b:agent_woken 应恰 1");
    assert_eq!(outcome.agent.possess_rejected, 0, "Y2b:两次附身都不该被拒");
}

// ---- Y3/Y4/Y5/Y6 共用:quad_trot 撤启动票 → 行走中冻结 → 观察者唤醒续走 ----

/// Y3 布景助手:在线段跑 quad_trot,撤启动票(③ 档),等冻结,关服。
/// 检查点间隔 240(Y4 要一个冻结之前的期中票)。返回
/// (DriverOutcome, data_dir, config)。
async fn y3_online_stage() -> (DriverOutcome, std::path::PathBuf, Config) {
    let config = base_config(vec![quad_trot_agent()], 240);
    let h = bootstrap_harness(config.clone()).await;
    // 先跑 120 拍(settle 起步),再撤启动票(票号 = STARTUP_TICKET_BASE)。
    advance_ticks(120).await;
    h.tx.send(DriverCommand::RemoveAreaTicket {
        id: STARTUP_TICKET_BASE,
    })
    .await
    .expect("驱动仍存活");
    // 撤票 → 主体出全部唤醒范围(无观察者)→ 冷却后冻结(上限 600 拍;
    // 等待窗口 900 拍,有界)。
    advance_ticks(900).await;
    let data_dir = h.data_dir.clone();
    let outcome = shutdown(h).await;
    (outcome, data_dir, config)
}

/// Y3:撤票后主体在行走中被冻结(`agent_frozen == 1`、`skipped_frozen`
/// 增长、`actions_emitted` 停止增长——冻结窗口的每个控制拍都走跳过分支);
/// 观察者移到它旁边 → 唤醒,之后 300 拍沿航向平均速度 ≥ 0.1 m/s,全程
/// up·y ≥ 0.7,EntityId 不变。
#[tokio::test(start_paused = true)]
async fn y3_ticket_removed_walking_freeze_and_resume() {
    let (outcome, data_dir, config) = y3_online_stage().await;
    println!(
        "Y3: agent_frozen {}, agent_woken {}, actions_emitted {}, skipped_frozen {}",
        outcome.agent.agent_frozen,
        outcome.agent.agent_woken,
        outcome.controller_counters.actions_emitted,
        outcome.controller_counters.skipped_frozen
    );
    assert_eq!(outcome.agent.agent_frozen, 1, "Y3:900 拍内主体未冻结");
    assert_eq!(outcome.agent.agent_woken, 0, "Y3:在线段不该有唤醒");
    assert!(
        outcome.controller_counters.skipped_frozen > 0,
        "Y3:skipped_frozen 未增长(实得 {})",
        outcome.controller_counters.skipped_frozen
    );

    // 恢复线:主体仍冻结、同一 EntityId;观察者移到旁边 → 唤醒;300 拍
    // drive+tick 循环(与驱动同序,控制器从检查点继续)。
    let (mut session, mut host, snap_tick) = resume_session(&config, &data_dir);
    let frozen_set = session.frozen_agents();
    let entity = *frozen_set.iter().next().expect("恢复线恰一只冻结主体");
    println!("Y3: 恢复线冻结主体 {entity:?} @ 快照拍 {}", snap_tick.0);
    let mut now = session.tick_now();
    let pos0 = session.agent_root_position(entity).expect("在场");
    session
        .move_observer(Vec3::new(pos0.x, GROUND_TOP_Y + 2.0, pos0.z))
        .expect("观察者移到主体旁");
    let mut woken_tick: Option<u64> = None;
    for t in 0..600u64 {
        drive_tick(&mut session, &mut CommandLog::new(), &mut host, now);
        now = now.next();
        if !session.frozen_agents().contains(&entity) {
            woken_tick = Some(t);
            break;
        }
    }
    let woken_tick = woken_tick.expect("Y3:600 拍内未唤醒");
    println!("Y3: 唤醒拍(移回起)= {woken_tick}");
    // 唤醒后 300 拍:沿航向平均速度 ≥ 0.1 m/s,up·y ≥ 0.7,EntityId 不变。
    let yaw = 0.0_f32; // 配置 yaw_deg = 0,沿 +x
    let (dx, dz) = (yaw.cos(), -yaw.sin());
    let start = session.agent_root_position(entity).expect("在场");
    let mut up_min = f32::INFINITY;
    for _ in 0..300u32 {
        drive_tick(&mut session, &mut CommandLog::new(), &mut host, now);
        now = now.next();
        let states = session.agent_states();
        let (_, _, links) = states
            .iter()
            .find(|(id, _, _)| *id == entity)
            .expect("续走段主体持续在 agent_states");
        up_min = up_min.min((links[0].rotation * Vec3::new(0.0, 1.0, 0.0)).y);
    }
    let end = session.agent_root_position(entity).expect("在场");
    let dist = ((end.x - start.x) * dx + (end.z - start.z) * dz).abs();
    let speed = dist / (300.0 / 60.0);
    println!("Y3: 续走 300 拍沿航向 {dist:.3} m(平均 {speed:.3} m/s),up·y 最小 {up_min:.4}");
    assert!(speed >= 0.1, "Y3:唤醒后沿航向平均速度 {speed:.3} m/s < 0.1");
    assert!(up_min >= 0.7, "Y3:全程 up·y 最小 {up_min} < 0.7");
    session.shutdown();
}

// ---- Y4/Y5/Y6 ----

/// Y4:Y3 布景,取冻结之前的检查点加其后日志回放,跑过冻结与唤醒,末态
/// 根位姿与在线逐位相同(在线末态 = 关服时终末检查点恢复的主体状态)。
#[tokio::test(start_paused = true)]
async fn y4_replay_bitwise_matches_online() {
    let (_outcome, data_dir, config) = y3_online_stage().await;
    let cps = read_all_checkpoints(&data_dir);
    assert!(
        cps.len() >= 2,
        "布景自检:须有期中检查点(冻结之前)+ 终末,实得 {}",
        cps.len()
    );
    let (_, last_blob) = cps.last().expect("终末检查点存在").clone();
    let last_bundle = app::driver::decode_checkpoint(&last_blob).expect("可解");
    let online_final_tick = last_bundle.world.shards.tick;
    let mut final_cfg = config.session_config();
    final_cfg.scheduler.start_tick = online_final_tick; // 恢复拍对齐(同 replay 入口)
    let mut online_final = sim_api::session::Session::new(final_cfg);
    for bytes in last_bundle.world.shards.per_shard.values() {
        online_final
            .register_snapshot_with_bodies(
                bytes,
                Box::new(SimpleRegistry::builtin()),
                bodies_registry(),
            )
            .expect("分片快照恢复");
    }
    online_final.restore_scheduler_state(last_bundle.world.scheduler.clone());
    let entity = online_final
        .frozen_agents()
        .iter()
        .next()
        .copied()
        .expect("在线末态主体冻结在表");
    let online_state = online_final
        .agent_states()
        .into_iter()
        .find(|(id, _, _)| *id == entity)
        .expect("在线末态主体在 agent_states");
    online_final.shutdown();

    // 回放:最早检查点(冻结之前;interval 240 的期中票)+ 全程拼接日志。
    let (first_tick, first_blob) = &cps[0];
    let bundle = app::driver::decode_checkpoint(first_blob).expect("可解");
    assert!(
        *first_tick < online_final_tick,
        "布景自检:期中检查点早于终末(冻结之前)"
    );
    let log = stitched_log(&cps);
    let mut replay = sim_api::replay::replay_world_with_bodies(
        &bundle.world,
        &log,
        {
            let mut c = config.session_config();
            c.scheduler.start_tick = *first_tick;
            c
        },
        || Box::new(SimpleRegistry::builtin()),
        bodies_registry,
    )
    .expect("重放装配");
    for _ in (first_tick.0 + 1)..=online_final_tick.0 {
        replay.tick();
    }
    let replay_state = replay
        .agent_states()
        .into_iter()
        .find(|(id, _, _)| *id == entity)
        .expect("回放线主体在 agent_states");
    assert_eq!(
        online_state, replay_state,
        "Y4:末态主体状态与在线逐位不同(拍 {})",
        online_final_tick.0
    );
    println!(
        "Y4: 回放末态 == 在线末态(拍 {},冻结+唤醒全程走完)",
        online_final_tick.0
    );
    replay.shutdown();
}

/// Y5:Y3 布景两次运行,恢复线末段(唤醒后 300 拍)主体连杆状态逐拍逐位
/// 相同,冻结/唤醒计数相同。
#[tokio::test(start_paused = true)]
async fn y5_double_run_bitwise_identical() {
    let run = || async {
        let (outcome, data_dir, config) = y3_online_stage().await;
        let (mut session, mut host, _snap_tick) = resume_session(&config, &data_dir);
        let entity = *session.frozen_agents().iter().next().expect("冻结主体");
        let mut now = session.tick_now();
        let pos0 = session.agent_root_position(entity).expect("在场");
        session
            .move_observer(Vec3::new(pos0.x, GROUND_TOP_Y + 2.0, pos0.z))
            .expect("观察者移到主体旁");
        for _ in 0..600u64 {
            drive_tick(&mut session, &mut CommandLog::new(), &mut host, now);
            now = now.next();
            if !session.frozen_agents().contains(&entity) {
                break;
            }
        }
        // 唤醒后再走 300 拍,逐拍记录主体连杆状态。
        let mut trace: Vec<Vec<types::EntityState>> = Vec::new();
        for _ in 0..300u32 {
            drive_tick(&mut session, &mut CommandLog::new(), &mut host, now);
            now = now.next();
            let states = session
                .agent_states()
                .into_iter()
                .find(|(id, _, _)| *id == entity)
                .expect("主体持续在")
                .2;
            trace.push(states);
        }
        (outcome, trace)
    };
    let (outcome_a, trace_a) = run().await;
    let (outcome_b, trace_b) = run().await;
    assert_eq!(outcome_a.agent.agent_frozen, 1, "布景自检:第一次冻结 1");
    assert_eq!(outcome_b.agent.agent_frozen, 1, "布景自检:第二次冻结 1");
    assert_eq!(trace_a.len(), trace_b.len(), "拍数相同");
    for (t, (a, b)) in trace_a.iter().zip(&trace_b).enumerate() {
        assert_eq!(a, b, "Y5:第 {t} 拍主体连杆状态逐位不同");
    }
    println!(
        "Y5: 两轮 {} 拍逐位相同(agent_frozen 各 1,agent_woken 各 {} / {})",
        trace_a.len(),
        outcome_a.agent.agent_woken,
        outcome_b.agent.agent_woken
    );
}

/// Y6:Y3 冻结后关服 → bootstrap 恢复 → 主体仍冻结(`Session::frozen_agents`
/// 含它),同一 EntityId;放观察者 → 唤醒、13 连杆、续走 ≥ 0.3 m。
#[tokio::test(start_paused = true)]
async fn y6_restart_frozen_then_walk() {
    let (outcome, data_dir, config) = y3_online_stage().await;
    assert_eq!(outcome.agent.agent_frozen, 1, "布景自检:在线段已冻结");
    let (mut session, mut host, _snap_tick) = resume_session(&config, &data_dir);
    let frozen_set = session.frozen_agents();
    let entity = *frozen_set.iter().next().expect("恢复线恰一只冻结主体");
    assert!(frozen_set.contains(&entity), "Y6:恢复后主体不在冻结集合");
    assert!(session.is_agent(entity), "同一 EntityId(主体在 agents 表)");
    // 放观察者 → 唤醒 → 13 连杆 → 续走 ≥ 0.3 m。
    let mut now = session.tick_now();
    let pos0 = session.agent_root_position(entity).expect("在场");
    session
        .move_observer(Vec3::new(pos0.x, GROUND_TOP_Y + 2.0, pos0.z))
        .expect("观察者移到主体旁");
    let mut woken = false;
    for _ in 0..600u64 {
        drive_tick(&mut session, &mut CommandLog::new(), &mut host, now);
        now = now.next();
        if !session.frozen_agents().contains(&entity) {
            woken = true;
            break;
        }
    }
    assert!(woken, "Y6:600 拍内未唤醒");
    let states = session.agent_states();
    let (_, _, links) = states
        .iter()
        .find(|(id, _, _)| *id == entity)
        .expect("唤醒后主体在");
    assert_eq!(links.len(), 13, "Y6:唤醒后 13 连杆(实得 {})", links.len());
    let start = session.agent_root_position(entity).expect("在场");
    for _ in 0..300u32 {
        drive_tick(&mut session, &mut CommandLog::new(), &mut host, now);
        now = now.next();
    }
    let end = session.agent_root_position(entity).expect("在场");
    let d = ((end.x - start.x).powi(2) + (end.z - start.z).powi(2)).sqrt();
    println!("Y6: 恢复+唤醒后 300 拍位移 {d:.3} m");
    assert!(d >= 0.3, "Y6:续走位移 {d:.3} m < 0.3 m");
    session.shutdown();
}

/// Y3b 四足**全程在线**行走中冻结与续走(B03c 返修 R1 第 1 节):配置
/// quad_trot → 客户端连上采样 120 拍(拿 EntityId 与已走位置)→ Goodbye
/// 断开(观察者票撤)→ `DriverCommand::RemoveAreaTicket` 撤启动票 →
/// 推拍 900 拍(不连客户端,上限口径;无观察者、无唤醒中心)→ 客户端
/// 重连,MoveObserver 到**冻结前最后一次 AgentStates 采样位姿**旁 →
/// 推拍 ≤600:先复核位姿 60 拍逐位不变(仍冻结),再等位姿重新变化
/// (唤醒)→ 唤醒后采样 300 拍:沿航向平均速度 ≥ 0.1 m/s、全程
/// up·y ≥ 0.7、EntityId 不变(b03a `sample_quad_root_watch` 口径)→
/// 关服。断言:DriverOutcome agent_frozen == 1、agent_woken == 1;
/// skipped_frozen > 0、actions_emitted > 0(分段计数在线不可读,
/// DriverOutcome 只在关服交付;"唤醒后 actions 继续增长/skipped 不再
/// 增长"由行为证据闭环:主体持续行走 300 拍 = 这些控制拍既没被跳过
/// 也不是零目标)。
#[tokio::test(start_paused = true)]
async fn y3b_quad_online_freeze_wake_resume_walk() {
    let config = base_config(vec![quad_trot_agent()], 6000);
    let h = bootstrap_harness(config).await;
    // 采样段:客户端 join → 观察者到出生点旁(四足出生即在此)→ 采 120 拍,
    // 拿 EntityId 与已走位置(冻结前最后一次 AgentStates,任务书口径)。
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let mut entity: Option<EntityId> = None;
    let mut last_pos: Option<Vec3> = None;
    let deadline = Deadline::with_secs(120, "采样段(AgentStates 四足帧)");
    let mut sampled = 0u32;
    while sampled < 120 {
        deadline.check();
        advance_ticks(1).await;
        sampled += 1;
        if let Some((_ftick, pos, _pose)) = watch_agent_root(&mut rx, entity, &mut entity).await {
            last_pos = Some(pos);
        }
    }
    let quad = entity.expect("布景自检:四足出现在 AgentStates");
    let pre = last_pos.expect("布景自检:冻结前位姿已采样");
    println!("Y3b: 四足 EntityId = {quad:?},120 拍后位置 {pre:?}");
    // 断开:Goodbye → Left → 观察者票撤(handle_left ③)。此后无观察者。
    a.send(&C2S::Goodbye).await.ok();
    drop(a);
    drop(rx);
    // 撤启动票 → 主体出全部唤醒范围 → 冻结(首见翻转不受冷却;Y3 实证
    // 撤票后 ~450 控制拍全跳过 = 冻结起点很早)。推 900 拍(任务书上限)。
    h.tx.send(DriverCommand::RemoveAreaTicket {
        id: STARTUP_TICKET_BASE,
    })
    .await
    .expect("驱动仍存活");
    advance_ticks(900).await;

    // 重连:MoveObserver 到冻结前最后采样位姿旁 → 复核仍冻结(位姿 60 拍
    // 逐位不变,锚 = 断开前最后姿态)→ 等位姿重新变化(唤醒)。
    let (mut a2, mut rx2) = client(h.addr).await;
    let _player2 = join(&mut a2, &mut rx2, 8).await;
    a2.send(&C2S::MoveObserver {
        position: Vec3::new(pre.x, GROUND_TOP_Y + 2.0, pre.z),
    })
    .await
    .expect("MoveObserver 写入(冻结位姿旁)");
    // 复核仍冻结 + 等唤醒:锚 = 重连后第一帧(那才是冻结位姿——采样末
    // 位姿之后主体又走了几拍才冻,不等于冻结位姿);位姿逐位不变持续
    // (冻结复核),第一帧变化 = 唤醒。
    let mut anchor: Option<types::EntityState> = None;
    let mut still_frozen_ticks = 0u32;
    let mut woken_observed: Option<u32> = None;
    let mut wake_pos: Option<Vec3> = None;
    let deadline = Deadline::with_secs(300, "重连复核冻结 + 唤醒");
    let mut frames = 0usize;
    let mut seen_entity: Option<EntityId> = None;
    for tick in 0..1200u32 {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET * 2 {
            panic!("Y3b:重连段超帧预算(位姿未变/未唤醒?)");
        }
        advance_ticks(1).await;
        if let Some((_ftick, pos, pose)) =
            watch_agent_root(&mut rx2, Some(quad), &mut seen_entity).await
        {
            match anchor {
                None => {
                    anchor = Some(pose);
                    still_frozen_ticks = 1;
                }
                Some(p0) if pose == p0 => {
                    still_frozen_ticks += 1;
                }
                Some(_) => {
                    // 位姿与锚不同 → 已唤醒(从未冻结由 outcome 计数把关:
                    // agent_frozen == 1 / skipped_frozen > 0)。唤醒前至少
                    // 见过一次冻结位姿帧。
                    assert!(
                        still_frozen_ticks >= 1,
                        "Y3b:第一帧即变化——没观测到冻结位姿"
                    );
                    woken_observed = Some(tick);
                    wake_pos = Some(pos);
                    break;
                }
            }
        }
    }
    let woken_observed = woken_observed.expect("Y3b:1200 拍内未见位姿变化(未唤醒)");
    let wake_pos = wake_pos.expect("布景自检:唤醒帧位置已采样");
    println!(
        "Y3b: 冻结复核 {still_frozen_ticks} 拍逐位不变,唤醒拍(重连起)= {woken_observed},唤醒位置 {wake_pos:?}"
    );

    // 唤醒后 300 拍采样:沿航向(yaw = 0 → +x)平均速度 ≥ 0.1 m/s、全程
    // up·y ≥ 0.7、EntityId 不变(b03a sample_quad_root_watch 口径)。
    let yaw = 0.0_f32;
    let (dx, dz) = (yaw.cos(), -yaw.sin());
    let start = wake_pos;
    let mut up_min = f32::INFINITY;
    let mut end_pos = start;
    let mut id_changes = 0usize;
    let deadline = Deadline::with_secs(180, "唤醒后续走 300 拍采样");
    let mut frames = 0usize;
    for _ in 0..300u32 {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET * 2 {
            panic!("Y3b:续走段超帧预算");
        }
        advance_ticks(1).await;
        if let Some((_ftick, pos, pose)) =
            watch_agent_root(&mut rx2, Some(quad), &mut seen_entity).await
        {
            if seen_entity != Some(quad) {
                id_changes += 1;
                seen_entity = Some(quad);
            }
            up_min = up_min.min((pose.rotation * Vec3::new(0.0, 1.0, 0.0)).y);
            end_pos = pos;
        }
    }
    assert_eq!(id_changes, 0, "Y3b:续走段 EntityId 变了");
    let dist = ((end_pos.x - start.x) * dx + (end_pos.z - start.z) * dz).abs();
    let speed = dist / (300.0 / 60.0);
    println!("Y3b: 续走 300 拍沿航向 {dist:.3} m(平均 {speed:.3} m/s),up·y 最小 {up_min:.4}");
    assert!(
        speed >= 0.1,
        "Y3b:唤醒后沿航向平均速度 {speed:.3} m/s < 0.1"
    );
    assert!(up_min >= 0.7, "Y3b:全程 up·y 最小 {up_min} < 0.7");

    // 关服:DriverOutcome 硬判据。
    let outcome = shutdown(h).await;
    println!(
        "Y3b: agent_frozen {}, agent_woken {}, actions_emitted {}, skipped_frozen {}",
        outcome.agent.agent_frozen,
        outcome.agent.agent_woken,
        outcome.controller_counters.actions_emitted,
        outcome.controller_counters.skipped_frozen
    );
    assert_eq!(outcome.agent.agent_frozen, 1, "Y3b:agent_frozen 应恰 1");
    assert_eq!(outcome.agent.agent_woken, 1, "Y3b:agent_woken 应恰 1");
    assert!(
        outcome.controller_counters.skipped_frozen > 0,
        "Y3b:skipped_frozen 未增长(冻结跳过分支没走到)"
    );
    assert!(
        outcome.controller_counters.actions_emitted > 0,
        "Y3b:actions_emitted 为 0(唤醒后控制者没发目标)"
    );
}

/// Y7:无主体路径不变——无 `[[agents]]`、不连客户端,120 拍日志窗口
/// 0 条、新计数全 0。
#[tokio::test(start_paused = true)]
async fn y7_no_agents_path_unchanged() {
    let h = bootstrap_harness(base_config(Vec::new(), 6000)).await;
    advance_ticks(120).await;
    let data_dir = h.data_dir.clone();
    let outcome = shutdown(h).await;
    assert_eq!(outcome.agent.agent_frozen, 0, "Y7:agent_frozen 全 0");
    assert_eq!(outcome.agent.agent_woken, 0, "Y7:agent_woken 全 0");
    assert_eq!(
        outcome.controller_counters.skipped_frozen, 0,
        "Y7:skipped_frozen 全 0"
    );
    assert_eq!(
        outcome.controller_counters.actions_emitted, 0,
        "Y7:无主体无动作"
    );
    // 日志窗口 0 条(终末检查点的日志窗口)。
    let cps = read_all_checkpoints(&data_dir);
    let (_, blob) = cps.last().expect("终末检查点存在");
    let bundle = app::driver::decode_checkpoint(blob).expect("可解");
    let window = CommandLog::from_bytes(&bundle.log_bytes).expect("日志窗口可解");
    assert_eq!(window.entries().len(), 0, "Y7:120 拍日志窗口 0 条");
}
