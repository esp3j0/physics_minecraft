//! REQ-013 §6 AC5:单机进程内宿主(路径 L)vs 真实 TCP 网络服务器(路径 N)双路径位级一致。
//!
//! 两条路径在同一个 `#[tokio::test(start_paused = true)]` runtime 下同逻辑拍锁步推进,
//! 使用真实 `assets/policies/quad_gpu1` 策略包与 `assets/bodies/quad.body.toml` 身体定义,
//! 验证:
//! - A1 拼接命令日志逐条 bincode 字节相等;
//! - A2 两侧 `controller_counters` 零超时/零迟到/零丢弃/零错误/零拒绝且 `policy_inferences >= 400`;
//! - A3 回放末态根位姿/速度与关节角逐分量 `to_bits()` 相等;
//! - A4 意图段 300 拍内根位置 `+x` 位移 `>= 1.0 m`。

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use storage::traits::CheckpointStore as _;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{ChunkPos, EntityId, Tick, WorldSeed};
use voxel::SimpleRegistry;

use crate::config::{Config, WorldKind};
use crate::driver::{
    self, DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, decode_checkpoint,
    run,
};
use crate::local::{LocalConfig, spawn_in_current_runtime};
use crate::net::chunk_stream::PumpConfig;
use crate::net::{NetConfig, NetUp, OutboundHub, serve};

fn assets_path(sub: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets")
        .join(sub)
}

fn shared_config(data_dir: &Path) -> Config {
    Config {
        worker_count: 2,
        world_seed: WorldSeed(3),
        world_kind: WorldKind::Superflat,
        bodies_dir: assets_path("bodies"),
        policies_dir: assets_path("policies"),
        default_policy: Some("quad_gpu1".to_owned()),
        data_dir: data_dir.to_path_buf(),
        agents: Vec::new(),
        ..Config::default()
    }
}

struct NetHarness {
    tx: mpsc::Sender<DriverCommand>,
    driver: tokio::task::JoinHandle<DriverOutcome>,
    addr: SocketAddr,
    _bcast_tx: tokio::sync::broadcast::Sender<()>,
}

async fn spawn_net_harness(config: Config) -> NetHarness {
    let (session, log, store, _resume_from, spawn_next_seq, resume) =
        driver::bootstrap(&config, || Box::new(SimpleRegistry::builtin()))
            .expect("N bootstrap 成功");
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_owned(),
        read_timeout_secs: 3600,
        observe_interval_ticks: 1,
        full_snapshot_interval_ticks: 1,
        pump: PumpConfig::default(),
    };
    let cadence = ObserveCadence::from(&net_cfg);
    let (up_tx, up_rx) = mpsc::channel::<NetUp>(256);
    let hub = Arc::new(OutboundHub::new());
    let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
    let addr = serve(net_cfg, up_tx, bcast_rx, hub.clone())
        .await
        .expect("N 监听成功");
    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let (provider, new_sim, bodies) =
        driver::wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("N 装配成功");
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
        bodies,
        resume,
    }));
    NetHarness {
        tx,
        driver,
        addr,
        _bcast_tx: bcast_tx,
    }
}

struct TestClient {
    w: tokio::net::tcp::OwnedWriteHalf,
}

impl TestClient {
    async fn send(&mut self, c2s: &C2S) -> std::io::Result<()> {
        self.w.write_all(&encode_c2s_frame(c2s)).await
    }
}

async fn connect_client(addr: SocketAddr) -> (TestClient, mpsc::UnboundedReceiver<S2C>) {
    let stream = TcpStream::connect(addr).await.expect("N TCP 连入");
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

/// 不推进虚拟时钟,仅让 OS 回环 TCP 与 mio 反应堆完成帧投递到驱动 `up_rx`。
async fn flush_tcp() {
    for _ in 0..10 {
        tokio::task::yield_now().await;
        std::thread::sleep(Duration::from_millis(1));
    }
    for _ in 0..8 {
        tokio::task::yield_now().await;
    }
}

/// 精确推进 1 个逻辑模拟拍(tokio 定时器轮为 1 ms 粒度,dt = 16.667 ms 向上取整 17 ms,
/// 推进 17 ms 恰好触发两侧各 1 次 `MissedTickBehavior::Skip` interval)。
async fn advance_one_tick() {
    tokio::time::advance(Duration::from_millis(17)).await;
    tokio::task::yield_now().await;
}

fn read_all_checkpoints(data_dir: &Path) -> Vec<(Tick, Vec<u8>)> {
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
        let bundle = decode_checkpoint(blob).expect("checkpoint 可解");
        let window = CommandLog::from_bytes(&bundle.log_bytes).expect("日志窗口可解");
        for entry in window.entries() {
            log.append(entry.clone());
        }
    }
    log
}

#[derive(Default)]
struct PathObserver {
    saw_chunk_0_0: bool,
    spawned: Option<EntityId>,
    latest_root_x: Option<f32>,
}

impl PathObserver {
    fn feed(&mut self, msg: S2C, tracked: Option<EntityId>) {
        match msg {
            S2C::ChunkData { pos, .. } if pos == ChunkPos::new(0, 0) => {
                self.saw_chunk_0_0 = true;
            }
            S2C::Spawned { entity } => {
                if self.spawned.is_none() {
                    self.spawned = Some(entity);
                }
            }
            S2C::AgentStates { states, .. } => {
                if let Some(target) = tracked.or(self.spawned) {
                    for (id, _, links) in states {
                        if id == target
                            && let Some(root) = links.first()
                        {
                            self.latest_root_x = Some(root.translation.x);
                        }
                    }
                }
            }
            _ => {}
        }
    }
}

fn drain_n(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    obs: &mut PathObserver,
    tracked: Option<EntityId>,
) {
    while let Ok(msg) = rx.try_recv() {
        obs.feed(msg, tracked);
    }
}

fn drain_l(handle: &crate::local::LocalHandle, obs: &mut PathObserver, tracked: Option<EntityId>) {
    for msg in handle.drain() {
        obs.feed(msg, tracked);
    }
}

#[tokio::test(start_paused = true)]
async fn ac5_dual_path_bit_exact_consistency() {
    let dir_l = tempfile::TempDir::new().expect("TempDir L");
    let dir_n = tempfile::TempDir::new().expect("TempDir N");

    let cfg_l = shared_config(dir_l.path());
    let cfg_n = shared_config(dir_n.path());

    // 启动路径 L 与路径 N(同一虚拟时钟起点 t=0)。
    let (l_handle, l_driver) = spawn_in_current_runtime(LocalConfig {
        config: cfg_l.clone(),
        view_distance: 2,
    })
    .await
    .expect("路径 L 启动");
    let n_harness = spawn_net_harness(cfg_n.clone()).await;
    let (mut n_client, mut n_rx) = connect_client(n_harness.addr).await;

    // 1. N 侧 Hello + Subscribe(2);L 侧装配等效。不推进虚拟时钟先等回环投递。
    n_client
        .send(&C2S::Hello { client_id: 1 })
        .await
        .expect("N Hello");
    n_client
        .send(&C2S::Subscribe { view_distance: 2 })
        .await
        .expect("N Subscribe");
    flush_tcp().await;

    let mut obs_l = PathObserver::default();
    let mut obs_n = PathObserver::default();
    let mut tick: u64 = 0;

    // 各推进 120 拍。
    for _ in 0..120 {
        advance_one_tick().await;
        tick += 1;
        drain_l(&l_handle, &mut obs_l, None);
        drain_n(&mut n_rx, &mut obs_n, None);
    }
    flush_tcp().await;
    drain_l(&l_handle, &mut obs_l, None);
    drain_n(&mut n_rx, &mut obs_n, None);

    // 2. 两边同拍发 MoveObserver(8, -60, 8),推进 60 拍;各自确认收到 chunk(0,0) 的 ChunkData(上限再 200 拍)。
    let move_cmd = C2S::MoveObserver {
        position: Vec3::new(8.0, -60.0, 8.0),
    };
    l_handle.send(move_cmd.clone());
    n_client.send(&move_cmd).await.expect("N MoveObserver");
    flush_tcp().await;

    for _ in 0..60 {
        advance_one_tick().await;
        tick += 1;
        drain_l(&l_handle, &mut obs_l, None);
        drain_n(&mut n_rx, &mut obs_n, None);
    }
    flush_tcp().await;
    drain_l(&l_handle, &mut obs_l, None);
    drain_n(&mut n_rx, &mut obs_n, None);

    let mut extra_wait = 0u32;
    while (!obs_l.saw_chunk_0_0 || !obs_n.saw_chunk_0_0) && extra_wait < 200 {
        advance_one_tick().await;
        tick += 1;
        extra_wait += 1;
        if extra_wait.is_multiple_of(5) {
            flush_tcp().await;
        }
        drain_l(&l_handle, &mut obs_l, None);
        drain_n(&mut n_rx, &mut obs_n, None);
    }
    assert!(
        obs_l.saw_chunk_0_0 && obs_n.saw_chunk_0_0,
        "两边都必须收到 chunk(0,0) 的 ChunkData (L={}, N={}, tick={tick})",
        obs_l.saw_chunk_0_0,
        obs_n.saw_chunk_0_0
    );

    // 3. 对齐到公共拍 T=200 后发 SpawnAgent quad。
    assert!(tick <= 200, "chunk(0,0) 到达拍 {tick} 超出公共对齐拍 T=200");
    while tick < 200 {
        advance_one_tick().await;
        tick += 1;
        drain_l(&l_handle, &mut obs_l, None);
        drain_n(&mut n_rx, &mut obs_n, None);
    }
    assert_eq!(tick, 200, "发 SpawnAgent 前两侧必须对齐到公共拍 T=200");

    let spawn_cmd = C2S::SpawnAgent {
        body: "quad".to_owned(),
        position: Vec3::new(8.5, -59.5, 8.5),
        yaw: 0.0,
    };
    l_handle.send(spawn_cmd.clone());
    n_client.send(&spawn_cmd).await.expect("N SpawnAgent");
    flush_tcp().await;

    for _ in 0..20 {
        advance_one_tick().await;
        tick += 1;
        flush_tcp().await;
        drain_l(&l_handle, &mut obs_l, None);
        drain_n(&mut n_rx, &mut obs_n, None);
        if obs_l.spawned.is_some() && obs_n.spawned.is_some() {
            break;
        }
    }
    let entity_l = obs_l.spawned.expect("路径 L 必须收到 Spawned");
    let entity_n = obs_n.spawned.expect("路径 N 必须收到 Spawned");
    assert_eq!(entity_l, entity_n, "两侧生成的 EntityId 必须相等");
    let entity = entity_l;

    // 4. 推进 600 拍(策略站立平衡)。
    for _ in 0..600 {
        advance_one_tick().await;
        tick += 1;
        drain_l(&l_handle, &mut obs_l, Some(entity));
        drain_n(&mut n_rx, &mut obs_n, Some(entity));
    }

    // 5. Possess{entity} + 推进 60 拍 + AgentIntent(0.5, 0) + 推进 300 拍(行走)。
    let possess_cmd = C2S::Possess { entity };
    l_handle.send(possess_cmd.clone());
    n_client.send(&possess_cmd).await.expect("N Possess");
    flush_tcp().await;

    for _ in 0..60 {
        advance_one_tick().await;
        tick += 1;
        drain_l(&l_handle, &mut obs_l, Some(entity));
        drain_n(&mut n_rx, &mut obs_n, Some(entity));
    }
    flush_tcp().await;
    drain_l(&l_handle, &mut obs_l, Some(entity));
    drain_n(&mut n_rx, &mut obs_n, Some(entity));
    let start_x_l = obs_l.latest_root_x.expect("路径 L 意图段起点必有根 x");
    let start_x_n = obs_n.latest_root_x.expect("路径 N 意图段起点必有根 x");

    let intent_cmd = C2S::AgentIntent {
        entity,
        move_xz: (0.5, 0.0),
        yaw: 0.0,
        jump: false,
    };
    l_handle.send(intent_cmd.clone());
    n_client.send(&intent_cmd).await.expect("N AgentIntent");
    flush_tcp().await;

    for _ in 0..300 {
        advance_one_tick().await;
        tick += 1;
        drain_l(&l_handle, &mut obs_l, Some(entity));
        drain_n(&mut n_rx, &mut obs_n, Some(entity));
    }
    flush_tcp().await;
    drain_l(&l_handle, &mut obs_l, Some(entity));
    drain_n(&mut n_rx, &mut obs_n, Some(entity));
    let end_x_l = obs_l.latest_root_x.expect("路径 L 意图段末点必有根 x");
    let end_x_n = obs_n.latest_root_x.expect("路径 N 意图段末点必有根 x");

    // 6. Unpossess + 推进 120 拍(零意图恢复站立)。
    l_handle.send(C2S::Unpossess);
    n_client.send(&C2S::Unpossess).await.expect("N Unpossess");
    flush_tcp().await;

    for _ in 0..120 {
        advance_one_tick().await;
        tick += 1;
        drain_l(&l_handle, &mut obs_l, Some(entity));
        drain_n(&mut n_rx, &mut obs_n, Some(entity));
    }

    // 7. 同拍关机,取两边 DriverOutcome(不额外推进 tick,确保两侧终末检查点对齐且实体存活)。
    l_handle.send(C2S::Goodbye);
    n_harness
        .tx
        .send(DriverCommand::Shutdown)
        .await
        .expect("N Shutdown");
    let outcome_l = l_driver.await.expect("路径 L 驱动退出");
    let outcome_n = n_harness.driver.await.expect("路径 N 驱动退出");
    drop(n_client);

    // ---- A1:拼接命令日志逐条 bincode 字节相等 ----------------------------------
    let cps_l = read_all_checkpoints(dir_l.path());
    let cps_n = read_all_checkpoints(dir_n.path());
    let log_l = stitched_log(&cps_l);
    let log_n = stitched_log(&cps_n);
    let entries_l = log_l.entries();
    let entries_n = log_n.entries();
    println!(
        "AC5 实测:tick={tick}, L 日志 {} 条 / N 日志 {} 条, L ticks_run={} / N ticks_run={}",
        entries_l.len(),
        entries_n.len(),
        outcome_l.ticks_run,
        outcome_n.ticks_run
    );
    let common = entries_l.len().min(entries_n.len());
    for (idx, (le, ne)) in entries_l.iter().zip(entries_n.iter()).enumerate() {
        let lb = bincode::serde::encode_to_vec(le, bincode::config::standard()).expect("bincode L");
        let nb = bincode::serde::encode_to_vec(ne, bincode::config::standard()).expect("bincode N");
        assert_eq!(
            lb,
            nb,
            "AC5 A1 首个不等命令日志条目 #{idx}(L 共 {} 条, N 共 {} 条):\n  L = {le:?}\n  N = {ne:?}",
            entries_l.len(),
            entries_n.len()
        );
    }
    assert_eq!(
        entries_l.len(),
        entries_n.len(),
        "AC5 A1 命令日志总条数不等(L={}, N={}, 前 {common} 条均相等):\n  L 额外首条 = {:?}\n  N 额外首条 = {:?}",
        entries_l.len(),
        entries_n.len(),
        entries_l.get(common),
        entries_n.get(common)
    );

    // ---- A2:controller_counters ------------------------------------------------
    for (label, c) in [
        ("L", outcome_l.controller_counters),
        ("N", outcome_n.controller_counters),
    ] {
        println!(
            "AC5 A2({label}): inferences={} timeouts={} late={} discarded={} errors={} send_rejected={}",
            c.policy_inferences,
            c.policy_timeouts,
            c.policy_late,
            c.policy_discarded,
            c.policy_errors,
            c.send_rejected
        );
        assert_eq!(c.policy_timeouts, 0, "{label}: policy_timeouts 必须为 0");
        assert_eq!(c.policy_late, 0, "{label}: policy_late 必须为 0");
        assert_eq!(c.policy_discarded, 0, "{label}: policy_discarded 必须为 0");
        assert_eq!(c.policy_errors, 0, "{label}: policy_errors 必须为 0");
        assert_eq!(c.send_rejected, 0, "{label}: send_rejected 必须为 0");
        assert!(
            c.policy_inferences >= 400,
            "{label}: policy_inferences 须 >= 400,实测 {}",
            c.policy_inferences
        );
    }

    // ---- A4:行走实证(意图段 300 拍内 +x 位移 >= 1.0 m) -------------------------
    let dx_l = end_x_l - start_x_l;
    let dx_n = end_x_n - start_x_n;
    println!(
        "AC5 A4: L dx={dx_l:.4} m ({start_x_l:.4} -> {end_x_l:.4}), N dx={dx_n:.4} m ({start_x_n:.4} -> {end_x_n:.4})"
    );
    assert!(
        dx_l >= 1.0,
        "路径 L 意图段 300 拍 +x 位移须 >= 1.0 m,实测 {dx_l:.4} m"
    );
    assert!(
        dx_n >= 1.0,
        "路径 N 意图段 300 拍 +x 位移须 >= 1.0 m,实测 {dx_n:.4} m"
    );

    // ---- A3:回放末态逐分量 to_bits() 相等 --------------------------------------
    let (sess_l, _ll, store_l, at_l, _, _) =
        driver::bootstrap(&cfg_l, || Box::new(SimpleRegistry::builtin())).expect("L 恢复启动");
    let (sess_n, _ln, store_n, at_n, _, _) =
        driver::bootstrap(&cfg_n, || Box::new(SimpleRegistry::builtin())).expect("N 恢复启动");
    assert_eq!(at_l, at_n, "两侧恢复拍号必须相等");

    let root_l = sess_l.agent_root_state(entity).expect("L 根状态可读");
    let root_n = sess_n.agent_root_state(entity).expect("N 根状态可读");
    let joints_l = sess_l.agent_joint_state(entity).expect("L 关节状态可读");
    let joints_n = sess_n.agent_joint_state(entity).expect("N 关节状态可读");

    let vec3_bits = |v: Vec3| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
    let quat_bits =
        |q: types::glam::Quat| [q.x.to_bits(), q.y.to_bits(), q.z.to_bits(), q.w.to_bits()];
    assert_eq!(
        vec3_bits(root_l.translation),
        vec3_bits(root_n.translation),
        "A3 root.translation 逐位不等: L={:?} N={:?}",
        root_l.translation,
        root_n.translation
    );
    assert_eq!(
        quat_bits(root_l.rotation),
        quat_bits(root_n.rotation),
        "A3 root.rotation 逐位不等: L={:?} N={:?}",
        root_l.rotation,
        root_n.rotation
    );
    assert_eq!(
        vec3_bits(root_l.linvel),
        vec3_bits(root_n.linvel),
        "A3 root.linvel 逐位不等: L={:?} N={:?}",
        root_l.linvel,
        root_n.linvel
    );
    assert_eq!(
        vec3_bits(root_l.angvel),
        vec3_bits(root_n.angvel),
        "A3 root.angvel 逐位不等: L={:?} N={:?}",
        root_l.angvel,
        root_n.angvel
    );
    assert_eq!(
        joints_l.positions.len(),
        joints_n.positions.len(),
        "A3 关节数不等"
    );
    let pos_bits_l: Vec<u32> = joints_l.positions.iter().map(|v| v.to_bits()).collect();
    let pos_bits_n: Vec<u32> = joints_n.positions.iter().map(|v| v.to_bits()).collect();
    assert_eq!(
        pos_bits_l, pos_bits_n,
        "A3 agent_joint_state.positions 逐分量 to_bits() 不等: L={:?} N={:?}",
        joints_l.positions, joints_n.positions
    );

    sess_l.shutdown();
    sess_n.shutdown();
    drop(store_l);
    drop(store_n);
}
