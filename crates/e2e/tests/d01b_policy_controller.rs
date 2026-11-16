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

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::{Config, StartupAgentConfig, StartupController};
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::InputSource;
use physics_sim::BodyRegistry;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
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
