//! REQ-012 B04 第 1 节(#25):非法动作计数上报(AC-B5)端到端。
//!
//! 口径(夹具 = `b02b_startup_agents.rs` 同款):真实网络栈(TCP 客户端 +
//! 驱动 + Session,a03 同款装配)。V1:配置一条 `quad_stand`,生成站稳后
//! 经 `DriverCommand::Send`(本地受信通道)依次发 5 条
//! `SimCommand::AgentJointTargets`(前 4 条非法),断言驱动计数、四足
//! 存活、非法拍位姿与对照运行逐位相同。V2:600 拍 `quad_trot` 零非法,
//! 两个计数恒 0。

use std::sync::Arc;
use std::time::Duration;

use app::config::{Config, StartupAgentConfig, StartupController};
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use tokio::sync::mpsc;

/// Superflat classic 地面顶面 world y(b02b 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;
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

async fn advance_ticks(ticks: u32) {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: Option<tokio::task::JoinHandle<DriverOutcome>>,
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
    serve(net_cfg, up_tx, bcast_rx, hub.clone())
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
    }
}

async fn shutdown(mut h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    let driver = h.driver.take().expect("驱动句柄只在 shutdown 取走一次");
    driver.await.expect("驱动不 panic")
}

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
