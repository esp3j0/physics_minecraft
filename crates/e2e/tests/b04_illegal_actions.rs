//! REQ-012 B04 第 1 节(#25):非法动作计数上报(AC-B5)端到端。
//!
//! 口径(夹具 = `b02b_startup_agents.rs` 同款):真实网络栈(TCP 客户端 +
//! 驱动 + Session,a03 同款装配)。V1:配置一条 `quad_stand`,生成站稳后
//! 经 `DriverCommand::Send`(本地受信通道)依次发 5 条
//! `SimCommand::AgentJointTargets`(前 4 条非法),断言驱动计数、四足
//! 存活、非法拍位姿与对照运行逐位相同。V2:600 拍 `quad_trot` 零非法,
//! 两个计数恒 0。

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::{Config, StartupAgentConfig, StartupController};
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::SimCommand;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{DimensionId, EntityId, ShardId, ShardPos, Tick, WorldId};

/// Superflat classic 地面顶面 world y(b02b 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;
/// 软窗(b02b 同款)。
const SOFT: Duration = Duration::from_millis(50);
/// 收帧上限(条)。
const FRAME_BUDGET: usize = 6000;
/// 等待墙钟上限(单循环)。
const WAIT_WALL_SECS: u64 = 60;
/// 出生点:分片 (0,0) 中心 (64,64),地面顶面 + 0.45(b02b 同口径)。
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
        checkpoint_interval_ticks: 6000,
        ..Config::default()
    };
    config.bodies_dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies");
    config.agents = agents;
    config
}

fn shard00() -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0))
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
            #[allow(clippy::disallowed_methods)] // 同 b02b:计时只属测量
            at: std::time::Instant::now() + Duration::from_secs(secs),
            what,
        }
    }

    fn check(&self) {
        #[allow(clippy::disallowed_methods)] // 同上
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

use messaging::InputSource;

// ---- harness(b02b 同款) ------------------------------------------------------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: Option<tokio::task::JoinHandle<DriverOutcome>>,
    addr: SocketAddr,
}

async fn bootstrap_harness(config: Config) -> Harness {
    // 目录唯一性:原子计数(并行测试同纳秒会撞 redb 文件锁)。
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let unique = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let data_dir = std::env::temp_dir().join(format!("b04_ia_{}_{}", std::process::id(), unique));
    let mut config = config;
    config.data_dir = data_dir;
    let (session, log, store, _resume_from, spawn_next_seq, resume) =
        app::driver::bootstrap(&config, || Box::new(voxel::SimpleRegistry::builtin()))
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
            Box::new(voxel::SimpleRegistry::builtin())
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
    }
}

async fn shutdown(mut h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    let driver = h.driver.take().expect("驱动句柄只在 shutdown 取走一次");
    driver.await.expect("驱动不 panic")
}

// ---- 采样(四足全连杆位姿,逐拍) ----------------------------------------------

#[derive(Debug, Clone)]
struct Snap {
    tick: Tick,
    links: Vec<types::EntityState>,
}

/// 逐帧推拍,采够 `want` 帧四足全连杆位姿即返回(帧按拍去重);
/// 预算超限 panic(b02b sample_quad_root 同口径,但保留全部连杆)。
async fn collect_snaps(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    want: usize,
) -> (EntityId, Vec<Snap>) {
    let deadline = Deadline::with_secs(180, "AgentStates 主体帧(四足全连杆位姿)");
    let mut frames = 0usize;
    let mut entity: Option<EntityId> = None;
    let mut snaps: Vec<Snap> = Vec::new();
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!(
                "等 AgentStates 超预算:{FRAME_BUDGET} 帧内主体帧不足 \
                 ({}/{want})(在等:配置生成的四足出现在 AgentStates 下行)",
                snaps.len()
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
                        }
                        if Some(id) == entity && snaps.last().map(|s| s.tick.0) != Some(tick.0) {
                            snaps.push(Snap { tick, links });
                        }
                    }
                }
                Some(_) => {}
                None => break,
            }
        }
        if snaps.len() >= want {
            break;
        }
    }
    (entity.expect("至少见过一次主体帧"), snaps)
}

/// 逐位相等(f32 按 to_bits;含 −0/+0 区分,不含 NaN——位姿应有限)。
fn bits_eq(a: &types::EntityState, b: &types::EntityState) -> bool {
    let v = |x: &Vec3| [x.x.to_bits(), x.y.to_bits(), x.z.to_bits()];
    let q = |r: &types::glam::Quat| [r.x.to_bits(), r.y.to_bits(), r.z.to_bits(), r.w.to_bits()];
    v(&a.translation) == v(&b.translation)
        && q(&a.rotation) == q(&b.rotation)
        && v(&a.linvel) == v(&b.linvel)
        && v(&a.angvel) == v(&b.angvel)
}

fn links_finite(links: &[types::EntityState]) -> bool {
    let v_ok = |v: &Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
    links.iter().all(|s| {
        v_ok(&s.translation)
            && v_ok(&s.linvel)
            && v_ok(&s.angvel)
            && s.rotation.w.is_finite()
            && s.rotation.x.is_finite()
            && s.rotation.y.is_finite()
            && s.rotation.z.is_finite()
    })
}

fn up_y(state: &types::EntityState) -> f32 {
    (state.rotation * Vec3::new(0.0, 1.0, 0.0)).y
}

// ---- V1:非法关节目标(拒 4 + 钳 12),非法拍位姿与对照逐位相同 --------------------

/// V1(任务书 B04 §1.2;B04 返修 R1 §0 裁决):quad_stand 站稳 120 拍后,
/// 经 `DriverCommand::Send` 依次发 5 条 AgentJointTargets:
/// 1. 长度 11(少一个)→ **内核拒**(`joint_targets_rejected`,驱动前置
///    校验不查长度——两道防线里只有这条能走到内核);
/// 2. 长度 12、其一 NaN → **驱动前置拦**(`commands_rejected`,不进日志、
///    不进内核——AC-B5 的"不会进入内核");
/// 3. 长度 12、其一 +∞ → **驱动前置拦**(同上);
/// 4. 目标实体不存在 → **驱动前置拦**(同上);
/// 5. 全 100.0(全超限)→ 合法生效,内核钳 12(`joint_targets_clamped`)。
/// 每条之间推 1 拍,发完再推 120 拍后关服。断言:rejected == 1、
/// commands_rejected == 3、clamped == 12;驱动正常退出;四足还在(13 连杆、
/// 同一 EntityId、根位姿全有限、up·y ≥ 0.7);前 4 条发出后的那一拍位姿与
/// "不发非法目标的对照运行"逐位相同(第 5 条不比,它合法生效);对照运行
/// 三个计数全 0。
#[tokio::test(start_paused = true)]
async fn v1_illegal_joint_targets_rejected_and_clamped() {
    // ---- 主运行:发 5 条(前 4 条非法)----
    let h = bootstrap_harness(quad_config(vec![quad_agent(
        StartupController::QuadStand,
        16.0,
    )]))
    .await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("MoveObserver 写入");
    // 站稳与起步 120 拍(生成 + settle;E1 口径)。
    let (quad, settle) = collect_snaps(&mut rx, 120).await;
    assert_eq!(
        settle.last().expect("非空").links.len(),
        13,
        "四足 13 连杆(布景前提)"
    );

    let cases: [(EntityId, Vec<f32>); 4] = [
        (quad, vec![0.0; 11]), // 1. 长度不符(dof = 12)
        (
            quad,
            {
                let mut v = vec![0.0f32; 12];
                v[3] = f32::NAN;
                v
            }, // 2. NaN
        ),
        (
            quad,
            {
                let mut v = vec![0.0f32; 12];
                v[5] = f32::INFINITY;
                v
            }, // 3. +∞
        ),
        // 4. 不存在的 EntityId(生成序号远小于 4242)。
        (EntityId::from_parts(0, 4242), vec![0.0; 12]),
    ];
    let mut main_snaps: Vec<Snap> = Vec::new();
    for (entity, targets) in cases {
        h.tx.send(DriverCommand::Send {
            shard: shard00(),
            command: SimCommand::AgentJointTargets { entity, targets },
        })
        .await
        .expect("commands 通道 Send");
        advance_ticks(1).await;
        let (_, snaps) = collect_snaps(&mut rx, 1).await;
        main_snaps.push(snaps.into_iter().next_back().expect("恰 1 帧"));
    }
    // 5. 全部超限(合法;内核钳 12)。
    h.tx.send(DriverCommand::Send {
        shard: shard00(),
        command: SimCommand::AgentJointTargets {
            entity: quad,
            targets: vec![100.0; 12],
        },
    })
    .await
    .expect("commands 通道 Send");
    let (_, final_snaps) = collect_snaps(&mut rx, 120).await;
    let last = final_snaps.last().expect("非空").clone();
    let outcome = shutdown(h).await;

    // ---- 对照运行:同配置同拍数,不发非法目标 ----
    let hc = bootstrap_harness(quad_config(vec![quad_agent(
        StartupController::QuadStand,
        16.0,
    )]))
    .await;
    let (mut ac, mut rxc) = client(hc.addr).await;
    let _ = join(&mut ac, &mut rxc, 8).await;
    ac.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_XZ[0], GROUND_TOP_Y + 2.0, QUAD_XZ[1]),
    })
    .await
    .expect("对照 MoveObserver 写入");
    let (quad_c, settle_c) = collect_snaps(&mut rxc, 120).await;
    assert_eq!(quad_c, quad, "对照运行同一 EntityId(同布景同分配)");
    let mut ctrl_snaps: Vec<Snap> = Vec::new();
    for _ in 0..4 {
        advance_ticks(1).await;
        let (_, snaps) = collect_snaps(&mut rxc, 1).await;
        ctrl_snaps.push(snaps.into_iter().next_back().expect("恰 1 帧"));
    }
    let _ = collect_snaps(&mut rxc, 120).await;
    let outcome_c = shutdown(hc).await;

    // ---- 断言 ----
    // 拍号对齐(两次运行逐拍同步推进,错位即布景问题)。
    assert_eq!(
        settle.last().expect("非空").tick,
        settle_c.last().expect("非空").tick,
        "站稳末拍对齐"
    );
    for (k, (m, c)) in main_snaps.iter().zip(&ctrl_snaps).enumerate() {
        assert_eq!(m.tick, c.tick, "第 {} 条非法后的拍号对齐", k + 1);
        assert_eq!(m.links.len(), c.links.len(), "连杆数一致");
        for (i, (ml, cl)) in m.links.iter().zip(&c.links).enumerate() {
            assert!(
                bits_eq(ml, cl),
                "第 {} 条非法后连杆 {i} 位姿不逐位:\n主 {:?}\n对照 {:?}",
                k + 1,
                ml,
                cl
            );
        }
    }
    // 计数(返修 R1 §0 裁决口径:长度不符那条进内核计数,NaN/∞/不存在
    // 被驱动前置校验拦下进 commands_rejected)。
    println!(
        "V1: rejected {} clamped {} commands_rejected {}(对照 rejected {} clamped {} commands_rejected {})",
        outcome.agent.joint_targets_rejected,
        outcome.agent.joint_targets_clamped,
        outcome.commands_rejected,
        outcome_c.agent.joint_targets_rejected,
        outcome_c.agent.joint_targets_clamped,
        outcome_c.commands_rejected,
    );
    assert_eq!(
        outcome.agent.joint_targets_rejected, 1,
        "rejected == 1(实得 {};仅长度不符那条到内核)",
        outcome.agent.joint_targets_rejected
    );
    assert_eq!(
        outcome.agent.joint_targets_clamped, 12,
        "clamped == 12(实得 {})",
        outcome.agent.joint_targets_clamped
    );
    assert_eq!(
        outcome.commands_rejected, 3,
        "commands_rejected == 3(实得 {};NaN/∞/不存在被驱动前置拦)",
        outcome.commands_rejected
    );
    // 对照运行:三个计数全 0。
    assert_eq!(outcome_c.agent.joint_targets_rejected, 0, "对照零 rejected");
    assert_eq!(outcome_c.agent.joint_targets_clamped, 0, "对照零 clamped");
    assert_eq!(outcome_c.commands_rejected, 0, "对照零 commands_rejected");
    // 四足还在:末帧 13 连杆、同一 EntityId、位姿全有限、up·y ≥ 0.7。
    assert_eq!(last.links.len(), 13, "四足还在(13 连杆)");
    assert!(links_finite(&last.links), "全连杆位姿有限");
    let uy = up_y(&last.links[0]);
    assert!(uy >= 0.7, "up·y = {uy} ≥ 0.7");
}

// ---- V2:600 拍 trot 零非法 ----------------------------------------------------

/// V2(任务书 B04 §1.2):没有任何非法目标的 600 拍 `quad_trot` 运行 →
/// 两个计数都是 0(脚本控制者自己不产生非法或超限目标)。clamped > 0
/// 不改断言——那说明小跑步态在撞限位,是要记下来的发现。
#[tokio::test(start_paused = true)]
async fn v2_trot_no_illegal_actions_zero_counters() {
    let h = bootstrap_harness(quad_config(vec![quad_agent(
        StartupController::QuadTrot,
        16.0,
    )]))
    .await;
    advance_ticks(600).await;
    let outcome = shutdown(h).await;
    println!(
        "V2: rejected {} clamped {} actions_emitted {}",
        outcome.agent.joint_targets_rejected,
        outcome.agent.joint_targets_clamped,
        outcome.controller_counters.actions_emitted,
    );
    assert_eq!(
        outcome.agent.joint_targets_rejected, 0,
        "trot 零拒绝(实得 {})",
        outcome.agent.joint_targets_rejected
    );
    assert_eq!(
        outcome.agent.joint_targets_clamped, 0,
        "trot 零钳制(实得 {};> 0 = 小跑撞限位,照实上报)",
        outcome.agent.joint_targets_clamped
    );
}
