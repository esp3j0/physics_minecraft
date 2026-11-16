//! REQ-012 D01b-1(#25):策略控制者接进驱动层(同步推理)。
//!
//! 口径(模板 = `b02b_startup_agents.rs`):真实网络栈(TCP 客户端 + 驱动 +
//! Session);`[[agents]]` 配置走 `Config.agents`,驱动每拍
//! `startup.poll → host.apply_leash → host.drive → host.drive_policies →
//! session.tick → retain_alive`。策略包目录用 `TempDir` 现拼:夹具 ONNX
//! 复制成 `<dir>/<名>/policy.onnx`,`policy.toml` 用字符串写,`body_hash`
//! 用 quad 身体的 `hash().hex()` 现算。
//!
//! **等待循环一律有界**(a06 R1 §4):墙钟 60 s 上限 + 帧数预算,超限
//! panic 并写明在等什么——挂死不算红。

// 期望值照任务书手算字面量 verbatim(如 1.64719278),不截断(policy act.rs 同款)。
#![allow(clippy::excessive_precision)]

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::{Config, StartupAgentConfig, StartupController};
use app::controller::PolicyKey;
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::InputSource;
use physics_sim::BodyRegistry;
use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use storage::traits::CheckpointStore as _;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{EntityId, Tick};
use voxel::SimpleRegistry;

/// Superflat classic 地面顶面 world y(b02b 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;
/// 软窗(b02b 同款)。
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
        policy: None,
        intent: [0.0; 3],
    }
}

fn quad_config(agents: Vec<StartupAgentConfig>) -> Config {
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 6000, // 默认不落期中票;Q4/Q7 显式改
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

/// 单个等待循环的墙钟上限(b02b 同款)。
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
            #[allow(clippy::disallowed_methods)] // 测试等待墙钟(b02b 同款授权面)
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

// ---- 迷你客户端(b02b 同款) --------------------------------------------------

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

/// Hello + Subscribe → Welcome(b02b join 同款)。
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

/// 四足 AgentStates 采样(b02b 同款):逐帧推拍,收集逐拍根记录。
#[derive(Debug, Clone, Copy)]
struct RootSample {
    tick: Tick,
    pos: Vec3,
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

// ---- harness(真实 bootstrap 起服 + 真实存储;b02b 同款起跑) ----------------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: Option<tokio::task::JoinHandle<DriverOutcome>>,
    addr: SocketAddr,
    /// 数据目录的持有者(checkpoints.redb 所在;最后一个持有者 drop 时
    /// 目录被删除)。重启段裸调 `app::driver::bootstrap` 时用 `h.data.clone()`
    /// 克隆 Arc 把目录续命到第二次起服。
    data: Arc<tempfile::TempDir>,
}

async fn bootstrap_harness(config: Config) -> Harness {
    let data = Arc::new(
        tempfile::Builder::new()
            .prefix("d01b_")
            .tempdir()
            .expect("临时目录"),
    );
    bootstrap_harness_in(config, data).await
}

/// 在给定的数据目录上起服(重启类测试用:同一目录可再次起服)。
async fn bootstrap_harness_in(config: Config, data: Arc<tempfile::TempDir>) -> Harness {
    let mut config = config;
    config.data_dir = data.path().to_path_buf();
    let (session, log, store, _resume_from, spawn_next_seq, resume) =
        app::driver::bootstrap(&config, || Box::new(SimpleRegistry::builtin()))
            .expect("bootstrap 装配成功");

    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 3600,
        observe_interval_ticks: 1,
        // 全量周期 1:站立主体姿态收敛后差分无变化不发包(b02b 的 trot
        // 一直在动不受影响),本单要逐拍采根位姿,用每拍全量。
        full_snapshot_interval_ticks: 1,
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
        data,
    }
}

async fn shutdown(mut h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    let driver = h.driver.take().expect("驱动句柄只在 shutdown 取走一次");
    driver.await.expect("驱动不 panic")
}

// ---- Q1:关节状态透传 --------------------------------------------------------

/// Q1:一只 `quad_stand` 四足,推 300 拍 → `agent_joint_state` 长度 12、
/// 全部有限、各关节角与 `default_angle` 差 < 0.35(大致站姿粗检);读数
/// 时刻主体已入睡,根线速度/角速度与 12 个关节速度逐位 0(D01b-1 裁定);
/// `agent_root_state` 位置与 `agent_root_position` 逐位相同;不存在的实体
/// 两法皆 None。
/// (Session 由关服后的恢复启动取回——驱动持有在线 Session,测试经
/// `bootstrap` 恢复分支拿到逐位相同的末态会话。)
#[tokio::test(start_paused = true)]
async fn q1_joint_state_passthrough() {
    let config = quad_config(vec![quad_agent(StartupController::QuadStand, 16.0)]);
    let h = bootstrap_harness(config.clone()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity, nlinks, samples) = sample_quad_root(&mut rx, 300).await;
    assert_eq!(nlinks, 13, "四足 13 连杆(实得 {nlinks})");
    let mut last_move = 0u64;
    for w in samples.windows(2) {
        if w[0].pos != w[1].pos {
            last_move = w[1].tick.0;
        }
    }
    let data = h.data.clone();
    let _outcome = shutdown(h).await;

    // 恢复启动取回末态 Session(回放逐位再生)。
    let mut resume_config = config;
    resume_config.data_dir = data.path().to_path_buf();
    let (session, _log, _store, at, _seq, _resume) =
        app::driver::bootstrap(&resume_config, || Box::new(SimpleRegistry::builtin()))
            .expect("恢复装配成功");
    let joints = session.agent_joint_state(entity).expect("主体关节状态可读");
    assert_eq!(joints.positions.len(), 12, "positions 长度 12");
    assert_eq!(joints.velocities.len(), 12, "velocities 长度 12");
    let bodies = quad_registry();
    let def = bodies
        .get(&bodies.hash_of("quad").expect("quad 在注册表"))
        .expect("哈希必在");
    println!(
        "Q1 实测:恢复拍 {:?},末采样拍 {:?},根位姿最后变化拍 {last_move}",
        at,
        samples.last().expect("非空").tick
    );
    for (i, joint) in def.dof_joints().enumerate() {
        let default = joint.kind.params().default_angle.unwrap_or(0.0);
        println!(
            "Q1 实测:关节 {i} q {:.6} default {default:.6} 差 {:.6} qd {:.6}",
            joints.positions[i],
            (joints.positions[i] - default).abs(),
            joints.velocities[i]
        );
    }
    let root = session.agent_root_state(entity).expect("根状态可读");
    println!(
        "Q1 实测:根 linvel {:?} angvel {:?}",
        root.linvel, root.angvel
    );
    assert!(
        joints.positions.iter().all(|v| v.is_finite())
            && joints.velocities.iter().all(|v| v.is_finite()),
        "关节角与关节速度全部有限"
    );
    for (i, joint) in def.dof_joints().enumerate() {
        let default = joint.kind.params().default_angle.unwrap_or(0.0);
        let dq = (joints.positions[i] - default).abs();
        assert!(
            dq < 0.35,
            "关节 {i}:角 {} 与 default_angle {default} 差 {dq} ≥ 0.35(大致保持站姿)",
            joints.positions[i]
        );
    }
    // 读数时刻主体已入睡(根位姿自 tick 138 起不变):根速度与关节速度全 0。
    assert_eq!(root.linvel, Vec3::ZERO, "入睡后根线速度逐位 0");
    assert_eq!(root.angvel, Vec3::ZERO, "入睡后根角速度逐位 0");
    assert_eq!(
        joints.velocities,
        vec![0.0; 12],
        "入睡后 12 个关节速度逐位 0.0"
    );
    let root_pos = session.agent_root_position(entity).expect("根位置可读");
    assert_eq!(root.translation, root_pos, "根状态位置与根位置逐位相同");
    let absent = EntityId::from_parts(0, 999_999);
    assert!(session.agent_joint_state(absent).is_none(), "不存在 → None");
    assert!(session.agent_root_state(absent).is_none(), "不存在 → None");
    session.shutdown();
}
// ---- 策略包夹具(D01b-1 §2:TempDir 现拼,夹具 ONNX + 字符串 toml) ---------

const STAND_ONNX: &[u8] = include_bytes!("../../../assets/policies/test_fixtures/zero45x12.onnx");
const WOBBLE_ONNX: &[u8] = include_bytes!("../../../assets/policies/test_fixtures/lin6x12.onnx");
const BROKEN_ONNX: &[u8] = include_bytes!("../../../assets/policies/test_fixtures/nan6x12.onnx");

fn obs_terms(terms: &[&str]) -> String {
    let mut s = String::new();
    for t in terms {
        s.push_str(&format!("[[obs]]\nterm = \"{t}\"\n"));
    }
    s
}

fn stand_toml(hash_hex: &str) -> String {
    format!(
        "name = \"stand\"\nbody_hash = \"{hash_hex}\"\ncontrol_period_ticks = 2\naction_scale = 0.25\n{}",
        obs_terms(&[
            "base_lin_vel",
            "base_ang_vel",
            "projected_gravity",
            "joint_pos",
            "joint_vel",
            "last_action"
        ])
    )
}

fn wobble_toml(hash_hex: &str) -> String {
    format!(
        "name = \"wobble\"\nbody_hash = \"{hash_hex}\"\ncontrol_period_ticks = 2\naction_scale = 0.1\naction_clip = 1.0\n{}",
        obs_terms(&["base_ang_vel", "intent"])
    )
}

fn broken_toml(hash_hex: &str) -> String {
    format!(
        "name = \"broken\"\nbody_hash = \"{hash_hex}\"\ncontrol_period_ticks = 2\naction_scale = 0.25\n{}",
        obs_terms(&["base_ang_vel", "intent"])
    )
}

/// Q9:与 `wobble` 同 manifest,只把 `action_scale` 改成 100.0(大幅越限)。
fn wobble_big_toml(hash_hex: &str) -> String {
    format!(
        "name = \"wobble_big\"\nbody_hash = \"{hash_hex}\"\ncontrol_period_ticks = 2\naction_scale = 100.0\naction_clip = 1.0\n{}",
        obs_terms(&["base_ang_vel", "intent"])
    )
}

fn write_pack(dir: &std::path::Path, name: &str, onnx: &[u8], toml: &str) {
    let p = dir.join(name);
    std::fs::create_dir_all(&p).expect("策略包目录可建");
    std::fs::write(p.join("policy.onnx"), onnx).expect("policy.onnx 可写");
    std::fs::write(p.join("policy.toml"), toml).expect("policy.toml 可写");
}

fn quad_hash_hex() -> String {
    let bodies = quad_registry();
    bodies.hash_of("quad").expect("quad 在注册表").hex()
}

/// stand / wobble / broken 三包(quad 哈希现算)。
fn policy_dir() -> Arc<tempfile::TempDir> {
    let dir = Arc::new(
        tempfile::Builder::new()
            .prefix("d01b_policies_")
            .tempdir()
            .expect("临时目录"),
    );
    let h = quad_hash_hex();
    write_pack(dir.path(), "stand", STAND_ONNX, &stand_toml(&h));
    write_pack(dir.path(), "wobble", WOBBLE_ONNX, &wobble_toml(&h));
    write_pack(dir.path(), "broken", BROKEN_ONNX, &broken_toml(&h));
    dir
}

fn policy_agent(name: &str, intent: [f32; 3]) -> StartupAgentConfig {
    StartupAgentConfig {
        body: "quad".to_owned(),
        position: [QUAD_XZ[0], QUAD_Y, QUAD_XZ[1]],
        yaw_deg: 0.0,
        controller: StartupController::Policy,
        leash_m: 16.0,
        policy: Some(name.to_owned()),
        intent,
    }
}

fn config_with_policies(agents: Vec<StartupAgentConfig>, dir: &tempfile::TempDir) -> Config {
    let mut c = quad_config(agents);
    c.policies_dir = dir.path().to_path_buf();
    c
}

// ---- 检查点 / 命令日志读取(b02b 同款) -------------------------------------

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

fn last_joint_targets(checkpoints: &[(Tick, Vec<u8>)], entity: EntityId) -> Option<Vec<f32>> {
    let log = stitched_log(checkpoints);
    log.entries().iter().rev().find_map(|e| match &e.command {
        messaging::SimCommand::AgentJointTargets {
            entity: id,
            targets,
        } if *id == entity => Some(targets.clone()),
        _ => None,
    })
}

fn count_joint_targets(checkpoints: &[(Tick, Vec<u8>)], entity: EntityId) -> usize {
    stitched_log(checkpoints)
        .entries()
        .iter()
        .filter(|e| {
            matches!(&e.command,
                messaging::SimCommand::AgentJointTargets { entity: id, .. } if *id == entity)
        })
        .count()
}

/// 脚本站立参考根高度(Q2/Q5 的对照基线;同 Q1 布景)。
async fn script_stand_height(ticks: u32) -> f32 {
    let config = quad_config(vec![quad_agent(StartupController::QuadStand, 16.0)]);
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (_entity, _n, samples) = sample_quad_root(&mut rx, ticks).await;
    let y = samples.last().expect("非空").pos.y;
    drop(a);
    let _outcome = shutdown(h).await;
    y
}

/// 一帧 AgentStates 的主体表(逐连杆姿态)。
type AgentFrame = Vec<(EntityId, types::BodyHash, Vec<types::EntityState>)>;

/// Q4 采样:推 want_ticks 拍,记住最后一帧非空 AgentStates(全量,逐连杆)。
async fn sample_last_agent_states(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    want_ticks: u32,
) -> (Tick, AgentFrame) {
    let deadline = Deadline::with_secs(180, "AgentStates 全量帧(Q4 逐连杆)");
    let mut frames = 0usize;
    let mut ticks_seen = 0usize;
    let mut last: Option<(Tick, AgentFrame)> = None;
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!("等 AgentStates 超预算:{FRAME_BUDGET} 帧({ticks_seen}/{want_ticks} 拍)");
        }
        advance_ticks(1).await;
        for _ in 0..4 {
            real_yield().await;
            match next_frame(rx).await {
                Some(S2C::AgentStates { tick, states, .. }) => {
                    if !states.is_empty() {
                        if last.as_ref().map(|(t, _)| t.0) != Some(tick.0) {
                            ticks_seen += 1;
                        }
                        last = Some((tick, states));
                    }
                }
                Some(_) => {}
                None => break,
            }
        }
        if ticks_seen >= want_ticks as usize {
            break;
        }
    }
    last.expect("至少见过一帧非空 AgentStates")
}

/// bootstrap 拒绝启动且错误文本含全部 needles(新数据目录)。
fn expect_bootstrap_err(mut config: Config, needles: &[&str]) {
    let data = tempfile::Builder::new()
        .prefix("d01b_q6_")
        .tempdir()
        .expect("临时目录");
    config.data_dir = data.path().to_path_buf();
    let err = match app::driver::bootstrap(&config, || Box::new(SimpleRegistry::builtin())) {
        Err(e) => e,
        Ok(_) => panic!("bootstrap 应拒绝:{needles:?}"),
    };
    let msg = format!("{err}");
    for n in needles {
        assert!(msg.contains(n), "错误文本含 {n:?}:{msg}");
    }
}

async fn observe_at_quad(a: &mut TestClient) {
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
}

// ---- Q2:策略站立 -----------------------------------------------------------

/// Q2:`stand` 策略推 600 拍——主体还在;根高度与脚本站立 600 拍差 < 0.02 m;
/// `policy_inferences ∈ [290,300]`、`policy_errors = 0`、`actions_emitted =
/// policy_inferences`。
#[tokio::test(start_paused = true)]
async fn q2_policy_stand() {
    let y_ref = script_stand_height(600).await;
    let dir = policy_dir();
    let config = config_with_policies(vec![policy_agent("stand", [0.0; 3])], &dir);
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    observe_at_quad(&mut a).await;
    let (entity, nlinks, samples) = sample_quad_root(&mut rx, 600).await;
    assert_eq!(nlinks, 13, "四足 13 连杆(主体还在)");
    let y = samples.last().expect("非空").pos.y;
    drop(a);
    let outcome = shutdown(h).await;
    let c = outcome.controller_counters;
    println!(
        "Q2 实测:脚本参考高度 {y_ref:.6},策略高度 {y:.6}(差 {:.6});\
         inferences {} errors {} emitted {} entity {entity:?}",
        (y - y_ref).abs(),
        c.policy_inferences,
        c.policy_errors,
        c.actions_emitted
    );
    assert!(
        (y - y_ref).abs() < 0.02,
        "根高度与脚本站立差 ≥ 0.02:策略 {y} vs 脚本 {y_ref}"
    );
    assert!(
        (290..=300).contains(&c.policy_inferences),
        "policy_inferences {} 不在 [290,300]",
        c.policy_inferences
    );
    assert_eq!(c.policy_errors, 0, "policy_errors 应为 0");
    assert_eq!(
        c.actions_emitted, c.policy_inferences,
        "actions_emitted = policy_inferences"
    );
    assert_eq!(
        c.policy_targets_clamped, 0,
        "stand 目标即默认站姿,不可能越限(clamped 应为 0)"
    );
}

// ---- Q3:观测进模型(intent 驱动关节) ---------------------------------------

async fn wobble_last_targets(intent: [f32; 3], dir: &tempfile::TempDir) -> Vec<f32> {
    let config = config_with_policies(vec![policy_agent("wobble", intent)], dir);
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    observe_at_quad(&mut a).await;
    let (entity, _n, _samples) = sample_quad_root(&mut rx, 120).await;
    drop(a);
    let data = h.data.clone();
    let _outcome = shutdown(h).await;
    last_joint_targets(&read_all_checkpoints(data.path()), entity)
        .expect("日志里有该主体的 AgentJointTargets")
}

/// Q3:`wobble` + intent [1,0,-1] 推 120 拍——日志末条 AgentJointTargets 的
/// 关节 3 = 0.04、关节 5 = 1.64719278(各差 < 1e-5);intent 全 0 时关节 3 = 0。
#[tokio::test(start_paused = true)]
async fn q3_wobble_moves_joints() {
    let dir = policy_dir();
    let t = wobble_last_targets([1.0, 0.0, -1.0], &dir).await;
    println!("Q3 实测:关节3 {:.8} 关节5 {:.8}", t[3], t[5]);
    assert!(
        (t[3] - 0.04).abs() < 1e-5,
        "关节3 目标 {} 与 0.04 差 ≥ 1e-5",
        t[3]
    );
    assert!(
        (t[5] - 1.64719278).abs() < 1e-5,
        "关节5 目标 {} 与 1.64719278 差 ≥ 1e-5",
        t[5]
    );
    let t0 = wobble_last_targets([0.0; 3], &dir).await;
    assert_eq!(t0[3], 0.0, "intent 全 0 时关节3 目标应为 0(实得 {})", t0[3]);
}

// ---- Q4:回放不依赖模型(AC-D2) ---------------------------------------------

/// Q4:`wobble` 在线推 300 拍,取最早检查点 + 拼接日志回放;回放前把策略包
/// 临时目录整个删掉(回放侧不建 PolicyTable、不读 .onnx)。回放末态全部主体
/// 各连杆状态与在线逐位相同;日志 AgentJointTargets 条数 = policy_inferences。
#[tokio::test(start_paused = true)]
async fn q4_replay_without_model() {
    let dir = policy_dir();
    let mut config = config_with_policies(vec![policy_agent("wobble", [1.0, 0.0, -1.0])], &dir);
    config.checkpoint_interval_ticks = 240;
    let h = bootstrap_harness(config.clone()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    observe_at_quad(&mut a).await;
    let (end_tick, online_states) = sample_last_agent_states(&mut rx, 300).await;
    drop(a);
    let data = h.data.clone();
    let outcome = shutdown(h).await;

    let cps = read_all_checkpoints(data.path());
    assert!(cps.len() >= 2, "期中 + 终末检查点都有(实得 {})", cps.len());
    let (first_tick, first_blob) = &cps[0];
    let bundle = app::driver::decode_checkpoint(first_blob).expect("可解");
    assert_eq!(bundle.world.shards.tick, *first_tick);
    let log = stitched_log(&cps);
    let entity = online_states[0].0;
    let n_targets = count_joint_targets(&cps, entity);
    assert_eq!(
        n_targets as u64, outcome.controller_counters.policy_inferences,
        "日志 AgentJointTargets 条数 {n_targets} ≠ policy_inferences {}",
        outcome.controller_counters.policy_inferences
    );

    // 回放之前删掉策略包临时目录:证明回放侧不读任何 .onnx。
    let dir_path = dir.path().to_path_buf();
    drop(dir);
    assert!(!dir_path.exists(), "策略包临时目录应已删除");

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
    .expect("回放装配(无策略表)");
    for _ in (first_tick.0 + 1)..=end_tick.0 {
        replay.tick();
    }
    let mut online = online_states.clone();
    let mut replay_states = replay.agent_states();
    online.sort_by_key(|(id, _, _)| *id);
    replay_states.sort_by_key(|(id, _, _)| *id);
    assert_eq!(
        online, replay_states,
        "回放末态全部主体各连杆与在线逐位不同(拍 {})",
        end_tick.0
    );
    replay.shutdown();
}

// ---- Q5:坏策略保持默认目标 --------------------------------------------------

/// Q5:`broken`(NaN 输出)推 300 拍——policy_errors ∈ [145,150]、
/// inferences = 0、emitted = 0;日志无该主体 AgentJointTargets;主体还在,
/// 根高度与 Q1(脚本 300 拍)差 < 0.02 m。
#[tokio::test(start_paused = true)]
async fn q5_broken_policy_holds() {
    let y_ref = script_stand_height(300).await;
    let dir = policy_dir();
    let config = config_with_policies(vec![policy_agent("broken", [0.0; 3])], &dir);
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    observe_at_quad(&mut a).await;
    let (entity, nlinks, samples) = sample_quad_root(&mut rx, 300).await;
    assert_eq!(nlinks, 13, "主体还在");
    let y = samples.last().expect("非空").pos.y;
    drop(a);
    let data = h.data.clone();
    let outcome = shutdown(h).await;
    let c = outcome.controller_counters;
    println!(
        "Q5 实测:参考高度 {y_ref:.6},broken 高度 {y:.6}(差 {:.6});\
         errors {} inferences {} emitted {}",
        (y - y_ref).abs(),
        c.policy_errors,
        c.policy_inferences,
        c.actions_emitted
    );
    assert!(
        (145..=150).contains(&c.policy_errors),
        "policy_errors {} 不在 [145,150]",
        c.policy_errors
    );
    assert_eq!(c.policy_inferences, 0, "坏策略不推理");
    assert_eq!(c.actions_emitted, 0, "坏策略不发目标");
    assert_eq!(
        count_joint_targets(&read_all_checkpoints(data.path()), entity),
        0,
        "日志里没有该主体的 AgentJointTargets"
    );
    assert!(
        (y - y_ref).abs() < 0.02,
        "根高度与 Q1 差 ≥ 0.02:broken {y} vs 脚本 {y_ref}"
    );
}

// ---- Q6:启动拒绝(AC-D1 服务器层面) -----------------------------------------

/// Q6:策略包不合格 / 配置错误 → 拒绝启动,错误文本含指定内容。
/// 包级三条走 `bootstrap`;配置级四条在解析阶段校验(`Config::from_toml_str`,
/// g3 同口径——bootstrap 不再重复纯配置校验)。
#[test]
fn q6_bootstrap_rejects() {
    let bodies = quad_registry();
    let quad_hex = bodies.hash_of("quad").expect("quad").hex();
    let capsule_hex = bodies.hash_of("capsule").expect("capsule").hex();

    // ① body_hash 全 0 → 两个哈希都在错误文本里。
    let dir = tempfile::Builder::new()
        .prefix("d01b_q6a_")
        .tempdir()
        .expect("临时目录");
    write_pack(
        dir.path(),
        "stand",
        STAND_ONNX,
        &stand_toml("0000000000000000"),
    );
    let c = config_with_policies(vec![policy_agent("stand", [0.0; 3])], &dir);
    expect_bootstrap_err(c, &["0000000000000000", &quad_hex]);

    // ② policy = "nope"(目录不存在)。
    let dir2 = tempfile::Builder::new()
        .prefix("d01b_q6b_")
        .tempdir()
        .expect("临时目录");
    let c2 = config_with_policies(vec![policy_agent("nope", [0.0; 3])], &dir2);
    expect_bootstrap_err(c2, &["nope"]);

    // ③ stand 配给 capsule(包哈希写 capsule 的,越过哈希校验到 NoDof)。
    let dir3 = tempfile::Builder::new()
        .prefix("d01b_q6c_")
        .tempdir()
        .expect("临时目录");
    write_pack(dir3.path(), "stand", STAND_ONNX, &stand_toml(&capsule_hex));
    let mut cap_agent = policy_agent("stand", [0.0; 3]);
    cap_agent.body = "capsule".to_owned();
    let c3 = config_with_policies(vec![cap_agent], &dir3);
    expect_bootstrap_err(c3, &["没有可动关节"]);

    // ④ 配置级错误(解析阶段拒绝)。
    let head = "[[agents]]\nbody = \"quad\"\nposition = [64.0, -59.5, 64.0]\n";
    for (toml, needle) in [
        (
            format!("{head}controller = \"policy\"\n"),
            "必须写 policy 名",
        ),
        (
            format!("{head}controller = \"quad_stand\"\npolicy = \"stand\"\n"),
            "不许写 policy",
        ),
        (
            format!("{head}controller = \"policy\"\npolicy = \"../x\"\n"),
            "只许",
        ),
        (
            format!(
                "{head}controller = \"policy\"\npolicy = \"stand\"\nintent = [2.0, 0.0, 0.0]\n"
            ),
            "intent",
        ),
    ] {
        let err = Config::from_toml_str(&toml).expect_err("配置应拒绝");
        let msg = format!("{err}");
        assert!(msg.contains(needle), "错误文本含 {needle:?}:{msg}");
    }
}

// ---- Q7:检查点恢复 ----------------------------------------------------------

/// Q7:`stand` 推 120 拍关服,同目录同配置再起——槽位 Policy(of("stand"))、
/// intent 保留;再推 120 拍 inferences ∈ [55,60]、主体还在。换没有这条
/// [[agents]] 的配置在同目录起服 → 拒绝,文本含“策略”。
#[tokio::test(start_paused = true)]
async fn q7_checkpoint_resume() {
    let dir = policy_dir();
    let intent = [1.0, 0.0, -1.0];
    let config = config_with_policies(vec![policy_agent("stand", intent)], &dir);
    let h = bootstrap_harness(config.clone()).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    observe_at_quad(&mut a).await;
    let (entity, _n, _s) = sample_quad_root(&mut rx, 120).await;
    drop(a);
    let data = h.data.clone();
    let _o1 = shutdown(h).await;

    // 恢复装配:槽位种类 + intent 保留。
    let mut rc = config.clone();
    rc.data_dir = data.path().to_path_buf();
    let (session, _log, store, at, _seq, resume) =
        app::driver::bootstrap(&rc, || Box::new(SimpleRegistry::builtin())).expect("恢复装配成功");
    let resume = resume.expect("恢复启动(Some)");
    let rec = resume
        .controllers
        .iter()
        .find(|r| r.entity == entity)
        .expect("检查点里有该主体的控制者条目");
    println!(
        "Q7 诊断:恢复拍 {at:?},attached_at {:?},period {}",
        rec.slot.attached_at, rec.slot.period_ticks
    );
    assert_eq!(
        rec.slot.kind,
        app::controller::ControllerKind::Policy(PolicyKey::of("stand")),
        "恢复槽位仍是 Policy(of(\"stand\"))"
    );
    assert_eq!(rec.slot.intent, intent, "intent 保留");
    session.shutdown();
    drop(store); // 释放 redb 文件锁,第二段起服才能再开

    // 第二段 +120 拍。
    let h2 = bootstrap_harness_in(config.clone(), data.clone()).await;
    let (mut a2, mut rx2) = client(h2.addr).await;
    let _p2 = join(&mut a2, &mut rx2, 8).await;
    observe_at_quad(&mut a2).await;
    let (e2, n2, s2) = sample_quad_root(&mut rx2, 120).await;
    assert_eq!(e2, entity, "恢复后同一主体");
    assert_eq!(n2, 13, "主体还在");
    let t0 = s2.first().expect("非空").tick;
    drop(a2);
    let o2 = shutdown(h2).await;
    // 累计计数含关服滞后段(驱动处理 Shutdown 前还在推拍),超出"推 120 拍"
    // 窗口;用带拍号的日志精确框住 [t0, t0+120) 这 120 拍内的推理数
    // (每条推理恰落一条 AgentJointTargets,Q4 已证 1:1)。恢复段末检查点
    // 的日志含恢复以来全量(还原上一窗口再续跑),只取最新一条,不拼接。
    let cps2 = read_all_checkpoints(data.path());
    let latest = cps2.last().expect("至少一个检查点");
    let bundle2 = app::driver::decode_checkpoint(&latest.1).expect("可解");
    let log2 = CommandLog::from_bytes(&bundle2.log_bytes).expect("日志窗口可解");
    let window_inf = log2
        .entries()
        .iter()
        .filter(|e| {
            matches!(&e.command,
                messaging::SimCommand::AgentJointTargets { entity: id, .. } if *id == e2)
                && e.tick.0 >= t0.0
                && e.tick.0 < t0.0 + 120
        })
        .count();
    println!(
        "Q7 实测:120 拍窗口 [{t0:?},+120) 推理 {window_inf};累计计数 {}(含关服段);\
         首样拍 {t0:?} 末样拍 {:?}(采样 {} 拍)",
        o2.controller_counters.policy_inferences,
        s2.last().expect("非空").tick,
        s2.len()
    );
    assert!(
        (55..=60).contains(&window_inf),
        "恢复后 120 拍窗口推理 {window_inf} 不在 [55,60]"
    );

    // 没有这条 [[agents]] 的配置在同目录起服 → 拒绝(检查点 Policy 键不在表)。
    let mut bad = config.clone();
    bad.agents = Vec::new();
    bad.data_dir = data.path().to_path_buf();
    let msg = match app::driver::bootstrap(&bad, || Box::new(SimpleRegistry::builtin())) {
        Err(e) => format!("{e}"),
        Ok(_) => panic!("无对应策略配置应拒绝恢复"),
    };
    assert!(msg.contains("策略"), "错误文本含“策略”:{msg}");
}

// ---- Q9:越限计数(policy_targets_clamped) ----------------------------------

/// Q9:`wobble_big`(lin6x12,action_scale=100)+ intent [1,0,-1] 推 300 拍。
/// 关节 3/9 必夹上限、5/11 必夹下限、4/10 必不夹(乘 intent[1]=0);设
/// N = policy_inferences,要求 N ≥ 145 且 4·N ≤ policy_targets_clamped ≤ 10·N;
/// 日志末条目标里 3/9 = 各自上限、5/11 = 各自下限(从身体定义读,逐位相等),
/// 4/10 = 默认角(差 < 1e-6)。主体是否还站着不作要求。
#[tokio::test(start_paused = true)]
async fn q9_clamped_counted() {
    let dir = Arc::new(
        tempfile::Builder::new()
            .prefix("d01b_q9_")
            .tempdir()
            .expect("临时目录"),
    );
    write_pack(
        dir.path(),
        "wobble_big",
        WOBBLE_ONNX,
        &wobble_big_toml(&quad_hash_hex()),
    );
    let config = config_with_policies(vec![policy_agent("wobble_big", [1.0, 0.0, -1.0])], &dir);
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    observe_at_quad(&mut a).await;
    let (entity, _n, _samples) = sample_quad_root(&mut rx, 300).await;
    drop(a);
    let data = h.data.clone();
    let outcome = shutdown(h).await;
    let c = outcome.controller_counters;
    let n = c.policy_inferences;
    let clamped = c.policy_targets_clamped;
    let t = last_joint_targets(&read_all_checkpoints(data.path()), entity)
        .expect("日志里有该主体的 AgentJointTargets");

    // 上下限 / 默认角从身体定义读(dof 顺序与目标向量一致)。
    let bodies = quad_registry();
    let hash = bodies.hash_of("quad").expect("quad 哈希");
    let quad = bodies.get(&hash).expect("quad 定义");
    let params: Vec<_> = quad.dof_joints().map(|j| j.kind.params()).collect();
    let upper = |i: usize| params[i].limits.expect("关节有限位").upper;
    let lower = |i: usize| params[i].limits.expect("关节有限位").lower;
    let default = |i: usize| params[i].default_angle.unwrap_or(0.0);

    println!(
        "Q9 实测:N={n} clamped={clamped}(4N={} 10N={})",
        4 * n,
        10 * n
    );
    println!(
        "Q9 末条目标:关节3 {:.8}(上限 {:.8})关节9 {:.8}(上限 {:.8})\
         关节5 {:.8}(下限 {:.8})关节11 {:.8}(下限 {:.8})关节4 {:.8} 关节10 {:.8}",
        t[3],
        upper(3),
        t[9],
        upper(9),
        t[5],
        lower(5),
        t[11],
        lower(11),
        t[4],
        t[10]
    );
    assert!(n >= 145, "policy_inferences {n} < 145(主体可能中途被移除)");
    assert!(
        4 * n <= clamped && clamped <= 10 * n,
        "policy_targets_clamped {clamped} 不在 [4N={},10N={}]",
        4 * n,
        10 * n
    );
    assert_eq!(t[3], upper(3), "关节3 目标应逐位等于上限");
    assert_eq!(t[9], upper(9), "关节9 目标应逐位等于上限");
    assert_eq!(t[5], lower(5), "关节5 目标应逐位等于下限");
    assert_eq!(t[11], lower(11), "关节11 目标应逐位等于下限");
    assert!(
        (t[4] - default(4)).abs() < 1e-6,
        "关节4 目标 {} 与默认角 {} 差 ≥ 1e-6",
        t[4],
        default(4)
    );
    assert!(
        (t[10] - default(10)).abs() < 1e-6,
        "关节10 目标 {} 与默认角 {} 差 ≥ 1e-6",
        t[10],
        default(10)
    );
}
