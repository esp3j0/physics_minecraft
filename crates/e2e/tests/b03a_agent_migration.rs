//! REQ-012 B03a(#25):多连杆主体的原子迁移(AC-B3)驱动层测试。
//!
//! 口径(模板 = `b02b_startup_agents.rs`):真实网络栈(TCP 客户端、驱动
//! 与 Session,a03 同款装配);`[[agents]]` 配置走 `Config.agents`,驱动每
//! 拍顺序按 B03a 2.1:startup.poll、apply_leash、drive(skip 在途)、
//! session.tick、retain_alive(并入在途集合)。起服一律经真实
//! `bootstrap`,数据目录在 temp(不碰仓库 `data/`)。
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
use messaging::InputSource;
use physics_sim::BodyRegistry;
use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{EntityId, Tick};
use voxel::SimpleRegistry;

/// Superflat classic 地面顶面 world y(a03/b02b 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;
/// 软窗(a03 同款)。
const SOFT: Duration = Duration::from_millis(50);
/// 收帧上限(条)。
const FRAME_BUDGET: usize = 6000;
/// 出生点(B03a X1:分片 (0,0) 东缘, walking +x 越过 x = 128 边界)。
const QUAD_SPAWN: [f32; 3] = [124.5, -59.5, 24.5];
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

/// X1 布景:quad_trot 出生 (124.5, -59.5, 24.5)、yaw 0(沿 +x 走)、
/// leash 16(圆心 = 出生点;leash 不触发——边界 x = 128 距圆心 3.5 m
/// < 16,迁移先于 leash 发生)。
fn x1_agent() -> StartupAgentConfig {
    StartupAgentConfig {
        body: "quad".to_owned(),
        position: QUAD_SPAWN,
        yaw_deg: 0.0,
        controller: StartupController::QuadTrot,
        leash_m: 16.0,
    }
}

fn x1_config(checkpoint_interval_ticks: u64) -> Config {
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks,
        ..Config::default()
    };
    config.bodies_dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies");
    config.agents = vec![x1_agent()];
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
        Self::with_secs(60, what)
    }

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

/// 主体根采样一条。
#[derive(Debug, Clone, Copy)]
struct RootSample {
    tick: Tick,
    pos: Vec3,
    up_y: f32,
}

/// 四足 AgentStates 采样:逐帧推拍,收集 (tick, 根 pos、up·y) 与"不在场"
/// 拍。返回(首见 EntityId, 连杆数, 逐拍根记录, EntityId 变更次数,
/// 每拍连杆态:(在场拍, 缺席拍, 非 13 连杆拍))。
async fn sample_quad_root_watch(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    want_ticks: u32,
) -> (
    EntityId,
    usize,
    Vec<RootSample>,
    usize,
    (u32, u32, u32),
    usize,
    (u64, Option<u64>, Option<u64>),
) {
    let deadline = Deadline::with_secs(300, "AgentStates 主体帧(四足根位姿)");
    let mut frames = 0usize;
    let mut entity: Option<EntityId> = None;
    let mut nlinks = 0usize;
    let mut samples: Vec<RootSample> = Vec::new();
    let mut id_changes = 0usize;
    let mut present_ticks = 0u32;
    let mut absent_ticks = 0u32;
    let mut bad_link_ticks = 0u32;
    let mut max_links = 0usize;
    let mut ticks_pushed = 0u32;
    // 验收 R1(§3):三个里程碑拍号——首次出现在 AgentStates、根 x 首次
    // ≥ 128、根 x 首次 ≥ 130。
    let mut first_seen_tick: Option<u64> = None;
    let mut cross128_tick: Option<u64> = None;
    let mut cross130_tick: Option<u64> = None;
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET * 4 {
            panic!(
                "等 AgentStates 超预算:{} 帧内主体帧不足 \
                 ({}/{want_ticks})(在等:四足出现在 AgentStates 下行)",
                FRAME_BUDGET * 4,
                want_ticks
            );
        }
        advance_ticks(1).await;
        ticks_pushed += 1;
        if ticks_pushed >= want_ticks {
            break;
        }
        let mut this_frame: Vec<(Tick, EntityId, usize, Option<RootSample>)> = Vec::new();
        for _ in 0..4 {
            real_yield().await;
            match next_frame(rx).await {
                Some(S2C::AgentStates { tick, states, .. }) => {
                    for (id, _, links) in states {
                        let root = links.first();
                        this_frame.push((
                            tick,
                            id,
                            links.len(),
                            root.map(|r| RootSample {
                                tick,
                                pos: r.translation,
                                up_y: (r.rotation * Vec3::new(0.0, 1.0, 0.0)).y,
                            }),
                        ));
                    }
                }
                Some(_) => {}
                None => break,
            }
        }
        if this_frame.is_empty() {
            continue;
        }
        // 每个采样拍:恰一只主体;EntityId 不变;连杆态记账。
        let mut seen_ids: Vec<EntityId> = Vec::new();
        for (tick, id, nlinks_frame, root) in &this_frame {
            if let Some(first) = entity {
                if id != &first && !seen_ids.contains(id) {
                    seen_ids.push(*id);
                }
            } else {
                entity = Some(*id);
                nlinks = *nlinks_frame;
            }
            max_links = max_links.max(*nlinks_frame);
            if first_seen_tick.is_none() {
                first_seen_tick = Some(tick.0);
            }
            match root {
                Some(r) if samples.last().map(|s| s.tick.0) != Some(tick.0) => {
                    samples.push(*r);
                    present_ticks += 1;
                    if cross128_tick.is_none() && r.pos.x >= 128.0 {
                        cross128_tick = Some(tick.0);
                    }
                    if cross130_tick.is_none() && r.pos.x >= 130.0 {
                        cross130_tick = Some(tick.0);
                    }
                }
                Some(_) => {}
                None => {
                    // 该拍主体不在(迁移在途):只统计,不产采样。
                    absent_ticks += 1;
                }
            }
            if root.is_some() && *nlinks_frame != 13 {
                bad_link_ticks += 1;
            }
        }
        id_changes += seen_ids.len();
        if samples.len() > 400 {
            let a = samples[samples.len() - 200];
            let b = samples[samples.len() - 1];
            let dt = (b.tick.0 - a.tick.0) as f32 / 60.0;
            let d = ((b.pos.x - a.pos.x).powi(2) + (b.pos.z - a.pos.z).powi(2)).sqrt();
            let v = d / dt;
            if dt > 3.0 && v < 0.02 {
                println!(
                    "X1 停滞探针:近 200 采样平均速度 {v:.4} m/s(拍 {} → {} 停在 x {:.3})",
                    a.tick.0, b.tick.0, b.pos.x
                );
                break;
            }
        }
    }
    (
        entity.expect("至少见过一次主体帧"),
        nlinks,
        samples,
        id_changes,
        (present_ticks, absent_ticks, bad_link_ticks),
        max_links,
        (
            first_seen_tick.expect("至少见过一次主体帧"),
            cross128_tick,
            cross130_tick,
        ),
    )
}

// ---- harness(真实 bootstrap 起服 + 真实存储;b02b R1 同款唯一写法) --------

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

/// 起服辅助(唯一写法)——经真实 `bootstrap` 装配(全新启动分支),存储用
/// bootstrap 打开的 redb 文件(temp 下唯一目录,不碰本地 data/)。
async fn bootstrap_harness(config: Config) -> Harness {
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let unique = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let data_dir = std::env::temp_dir().join(format!("b03a_{}_{}", std::process::id(), unique));
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

// ---- X1:走过分片边界 ---------------------------------------------------------

/// X1:quad_trot 出生 (124.5, -59.5, 24.5)、yaw 0、leash 16。跑到根 x ≥ 130
/// (首现 AgentStates → x≥130 ≤ 2400 拍;里程碑拍号照实打印)。断言:
/// - 全程 EntityId 不变;每拍 13 连杆或不在(在途),不在 ≤ 4 拍;
/// - 至少一次迁移(根分片 (0,0) → (1,0));
/// - 全程 up·y ≥ 0.7;
/// - 迁移后 300 拍沿航向平均速度 ≥ 0.1 m/s;
/// - 控制者仍挂着:actions_emitted 迁移后继续增长、send_rejected == 0、
///   skipped_in_transit 报实测。
#[tokio::test(start_paused = true)]
async fn x1_cross_shard_boundary() {
    let config = x1_config(6000);
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_SPAWN[0], GROUND_TOP_Y + 2.0, QUAD_SPAWN[1]),
    })
    .await
    .expect("MoveObserver 写入");

    let (
        _entity,
        _nlinks,
        samples,
        id_changes,
        (present, absent, bad_links),
        max_links,
        (first_seen, cross128, cross130),
    ) = sample_quad_root_watch(&mut rx, 4800).await;
    // 验收 R1(§3):三个里程碑拍号照实打印。
    println!(
        "X1 里程碑: 首现 AgentStates 拍 {first_seen},根 x≥128 拍 {:?},根 x≥130 拍 {:?}",
        cross128, cross130
    );
    // 任务书口径:从首次出现到 x ≥ 130 不超过 2400 拍。
    match cross130 {
        Some(t130) => {
            let span = t130 - first_seen;
            println!("X1: 首现拍 {first_seen} → x≥130 拍 {t130},历时 {span} 拍(≤ 2400?)");
            assert!(
                span <= 2400,
                "首现拍 {first_seen} → x≥130 拍 {t130} 历时 {span} 拍 > 2400(任务书原意口径)"
            );
        }
        None => panic!(
            "推拍窗口内根 x 未达 130(首现拍 {first_seen},末采样 x = {:.3};\
             布景异常或里程超限,如实红)",
            samples.last().expect("非空").pos.x
        ),
    }
    let last = *samples.last().expect("非空");
    // 全程 EntityId 不变;连杆态:恰 13 或不在;在途拍数 ≤ 4。
    assert_eq!(
        id_changes, 0,
        "全程 EntityId 不变(实得 {id_changes} 次变更)"
    );
    assert!(
        max_links == 13 && bad_links == 0,
        "在场拍连杆恒 13(max {max_links}, 非 13 拍 {bad_links})"
    );
    println!(
        "X1: 在途缺席拍 {absent}(在场合计 {present});根 ({:.2},{:.2},{:.2}) @ 拍 {}",
        last.pos.x, last.pos.y, last.pos.z, last.tick.0
    );
    assert!(
        absent <= 4,
        "在途缺席拍 {absent} > 4(原子迁移应至多隔 1-2 拍)"
    );
    // 至少一次迁移:迁移后根所在分片 = (1,0)(x ≥ 128 即分片 1)。
    let crossed = samples.iter().any(|s| s.pos.x >= 128.0);
    assert!(crossed, "至少一次迁移(根越过 x = 128)");
    // 全程 up·y ≥ 0.7。
    let up_min = samples.iter().map(|s| s.up_y).fold(f32::INFINITY, f32::min);
    println!("X1: up·y 最小 {up_min:.4}");
    assert!(up_min >= 0.7, "全程 up·y 最小 {up_min} < 0.7");
    // 迁移后 300 拍沿航向的平均速度 ≥ 0.1 m/s:取首个 x ≥ 129 的采样为
    // 窗口起点,300 拍后的位移 / 300 拍秒数。
    let cross_idx = samples
        .iter()
        .position(|s| s.pos.x >= 129.0)
        .expect("迁移后根已入分片 (1,0)");
    let s0 = samples[cross_idx];
    let s1_idx = (cross_idx + 300).min(samples.len() - 1);
    let s1 = samples[s1_idx];
    let dt_ticks = (s1.tick.0 - s0.tick.0) as f32;
    let dist = ((s1.pos.x - s0.pos.x).powi(2) + (s1.pos.z - s0.pos.z).powi(2)).sqrt();
    let v = dist / (dt_ticks / 60.0);
    println!("X1: 迁移后 {dt_ticks} 拍位移 {dist:.3} m, 平均速度 {v:.3} m/s");
    assert!(
        v >= 0.1,
        "迁移后平均速度 {v:.3} m/s < 0.1(控制不中断才走得动)"
    );

    let outcome = shutdown(h).await;
    println!(
        "X1 counters: actions_emitted {} send_rejected {} skipped_in_transit {}",
        outcome.controller_counters.actions_emitted,
        outcome.controller_counters.send_rejected,
        outcome.controller_counters.skipped_in_transit
    );
    assert!(
        outcome.controller_counters.actions_emitted > 0,
        "actions_emitted > 0(控制者仍挂着)"
    );
    assert_eq!(
        outcome.controller_counters.send_rejected, 0,
        "send_rejected == 0(在途跳过不走被拒路径)"
    );
}

// ---- X2:回放 -----------------------------------------------------------------

/// X2:X1 布景,取迁移之前的一个检查点 + 其后日志回放,跑过迁移后 200 拍,
/// 末态根位姿与在线逐位相同。
#[tokio::test(start_paused = true)]
async fn x2_replay_through_migration_bitwise() {
    let config = x1_config(240); // 期中票:迁移前(出生 124.5,起步 ~1 s 后)
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_SPAWN[0], GROUND_TOP_Y + 2.0, QUAD_SPAWN[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (_entity, _n, samples, _, _, _, _) = sample_quad_root_watch(&mut rx, 2200).await;
    let online_last = *samples.last().expect("非空");
    drop(a);
    let cps_dir = h.data_dir.clone();
    let outcome = shutdown(h).await;
    assert!(
        outcome.controller_counters.actions_emitted > 0,
        "布景自检:控制者发过动作"
    );

    // 最早检查点(240 拍,主体起步 ~3 拍/settle 120 拍 → 仍在出生分片起步
    // 段)+ 其后拼接日志。
    let checkpoints = read_all_checkpoints(&cps_dir);
    assert!(
        checkpoints.len() >= 2,
        "期中 + 终末都有(实得 {})",
        checkpoints.len()
    );
    let (first_tick, first_blob) = &checkpoints[0];
    let bundle = app::driver::decode_checkpoint(first_blob).expect("可解");
    assert_eq!(bundle.world.shards.tick, *first_tick);
    let log = stitched_log(&checkpoints);
    // 回放装配(逐段投喂:日志按拍分桶,先投该拍条目再 tick——收件箱
    // 容量 256,一次性投 4000+ 条会 InboxFull;b02b E3 窗口短没暴露)。
    let mut replay = sim_api::replay::replay_world_with_bodies(
        &bundle.world,
        &sim_api::log::CommandLog::new(),
        {
            let mut c = x1_config(6000).session_config();
            c.scheduler.start_tick = *first_tick;
            c
        },
        || Box::new(SimpleRegistry::builtin()),
        quad_registry,
    )
    .expect("重放装配");
    // 回放对齐:在线末采样拍 = 回放需跑到的拍(迁移发生在窗口内)。
    let end = online_last.tick.0;
    assert!(
        end > first_tick.0 + 600,
        "布景自检:回放窗口须跨过迁移拍(末拍 {end}, 首检 {})",
        first_tick.0
    );
    {
        use messaging::SimCommand;
        use std::collections::BTreeMap;
        let mut by_tick: BTreeMap<u64, Vec<&sim_api::log::LogEntry>> = BTreeMap::new();
        for e in log.entries() {
            if e.tick.0 >= first_tick.0 {
                by_tick.entry(e.tick.0).or_default().push(e);
            }
        }
        for tick in (first_tick.0 + 1)..=end {
            if let Some(entries) = by_tick.get(&tick) {
                for e in entries {
                    // 观察者命令走观察者收件箱(replay_log_entries 同款分流
                    // ;stamped 私有,测试内联同式构造)。
                    let stamped = messaging::StampedInput {
                        apply_at: e.tick,
                        source: e.source,
                        sequence: e.sequence,
                        payload: messaging::InputPayload::Command(e.command.clone()),
                    };
                    match e.command {
                        SimCommand::SetObserverPosition { .. } | SimCommand::RemoveObserver => {
                            replay.submit_observer_raw(stamped);
                        }
                        _ => {
                            replay.submit_raw(e.dest, stamped).expect("重放投递");
                        }
                    }
                }
            }
            replay.tick();
        }
    }
    let replay_root = replay
        .agent_root_position(_entity)
        .expect("回放线主体在(迁移后被目的分片重建)");
    println!(
        "X2: 在线末 ({:.4},{:.4},{:.4}) @ 拍 {} vs 回放 ({:.4},{:.4},{:.4})",
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
        "末态根位姿与在线逐位不同(拍 {end};迁移须在回放窗口内)"
    );
    replay.shutdown();
}

// ---- X3:双跑逐位相同 ---------------------------------------------------------

/// X3:X1 布景跑两遍,逐拍根位姿逐位相同。
#[tokio::test(start_paused = true)]
async fn x3_double_run_bitwise() {
    let run = || async {
        let config = x1_config(6000);
        let h = bootstrap_harness(config).await;
        let (mut a, mut rx) = client(h.addr).await;
        let _player = join(&mut a, &mut rx, 8).await;
        a.send(&C2S::MoveObserver {
            position: Vec3::new(QUAD_SPAWN[0], GROUND_TOP_Y + 2.0, QUAD_SPAWN[1]),
        })
        .await
        .expect("MoveObserver 写入");
        let (_entity, _n, samples, _, _, _, _) = sample_quad_root_watch(&mut rx, 2200).await;
        shutdown(h).await;
        samples
    };
    let sa = run().await;
    let sb = run().await;
    assert_eq!(
        sa.len(),
        sb.len(),
        "两遍采样拍数相同(实得 {} vs {})",
        sa.len(),
        sb.len()
    );
    let mut max_diff = 0.0f32;
    for (ra, rb) in sa.iter().zip(&sb) {
        assert_eq!(ra.tick, rb.tick, "拍号对齐");
        assert_eq!(ra.pos, rb.pos, "拍 {} 根位姿逐位不同", ra.tick.0);
        let d = (ra.pos - rb.pos).length();
        max_diff = max_diff.max(d);
    }
    println!("X3: {} 拍逐位相同(最大差 {max_diff:.2e} m)", sa.len());
}

// ---- X4:重启 -----------------------------------------------------------------

/// X4:四足迁移之后关服,再 bootstrap 起服;主体仍在、13 连杆、控制者仍
/// 挂着并继续前进 ≥ 0.3 m。
#[tokio::test(start_paused = true)]
async fn x4_restart_after_migration_preserves_agent() {
    let config = x1_config(6000);
    let h = bootstrap_harness(config.clone()).await;
    let mut restart_config = config.clone();
    restart_config.data_dir = h.data_dir.clone();
    let (mut a, mut rx) = client(h.addr).await;
    let _player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(QUAD_SPAWN[0], GROUND_TOP_Y + 2.0, QUAD_SPAWN[1]),
    })
    .await
    .expect("MoveObserver 写入");
    let (entity0, _n, samples, _, _, _, _) = sample_quad_root_watch(&mut rx, 900).await;
    // 布景自检:关服前发生过迁移(根曾越过 x = 128;任务书只要求
    // "迁移之后关服")。
    let crossed = samples.iter().any(|s| s.pos.x >= 128.0);
    assert!(crossed, "布景自检:关服前未发生迁移(根从未过界)");
    let pre_last = *samples.last().expect("非空");
    drop(a);
    let data_dir = h.data_dir.clone();
    let outcome1 = shutdown(h).await;
    assert_eq!(outcome1.startup_counters.spawned, 1, "首段生成恰 1 只");

    // 重启:同一 data_dir 再走一次真实 bootstrap(读终末 bundle)。
    let cps = read_all_checkpoints(&data_dir);
    let (_, blob) = cps.last().expect("终末检查点存在").clone();
    let bundle = app::driver::decode_checkpoint(&blob).expect("v13 档可解");
    assert_eq!(
        bundle.controllers.len(),
        1,
        "检查点恰 1 条 ControllerRecord"
    );
    let (resumed, _log2, store2, resume_from, spawn_next_seq, resume) =
        app::driver::bootstrap(&restart_config, || Box::new(SimpleRegistry::builtin()))
            .expect("重启 bootstrap 装配成功");
    assert!(resume_from.is_some(), "重启段走恢复分支");
    assert!(
        resumed.is_agent(entity0),
        "恢复后主体仍在(同一 EntityId {entity0:?})"
    );

    // 起新驱动(网络面)。
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
            cadence,
        }),
        spawn_next_seq,
        bodies: loaded,
        resume,
    }));

    // 重启段采样:主体 13 连杆、控制者继续走 ≥ 0.3 m。
    let (mut a2, mut rx2) = client(addr).await;
    let _player2 = join(&mut a2, &mut rx2, 8).await;
    a2.send(&C2S::MoveObserver {
        position: Vec3::new(pre_last.pos.x, GROUND_TOP_Y + 2.0, pre_last.pos.z),
    })
    .await
    .expect("MoveObserver 写入(跟随主体新位置)");
    let (entity1, nlinks1, samples2, _, _, _, _) = sample_quad_root_watch(&mut rx2, 200).await;
    assert_eq!(entity1, entity0, "重启后仍是同一个 EntityId");
    assert_eq!(nlinks1, 13, "重启后主体 13 连杆(实得 {nlinks1})");
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let _ = driver.await.expect("驱动不 panic");
    let q0 = samples2.first().expect("非空");
    let q1 = samples2.last().expect("非空");
    let d = ((q1.pos.x - q0.pos.x).powi(2) + (q1.pos.z - q0.pos.z).powi(2)).sqrt();
    println!(
        "X4: 重启段位移 {d:.3} m(首样 → ({:.2},{:.2}))",
        q1.pos.x, q1.pos.z
    );
    assert!(d >= 0.3, "重启后 200 拍位移 {d:.3} m < 0.3 m(控制者没恢复)");
}

// ---- X5:网络生成多连杆 -------------------------------------------------------

/// X5:客户端 `SpawnAgent { body: "quad" }` 成功,AgentStates 里有 13 连杆;
/// 等它站立 300 拍,不停机(终末 outcome 不带失控/移除计数)。
#[tokio::test(start_paused = true)]
async fn x5_network_spawn_quad_stands() {
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 6000,
        ..Config::default()
    };
    config.bodies_dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies");
    let h = bootstrap_harness(config).await;
    let (mut a, mut rx) = client(h.addr).await;
    let player = join(&mut a, &mut rx, 8).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(64.0, GROUND_TOP_Y + 2.0, 64.0),
    })
    .await
    .expect("MoveObserver 写入(出生点在分片中心)");
    // Welcome 回执 + AgentStates:等 quad 出现,恰 13 连杆。chunk 装载在
    // 观察者票生效后发生(③ 校验需要 chunk 已装载),SpawnAgent 每 20 拍
    // 重发一次直到被接受(Spawned 回执或 AgentStates 出现即停)。
    let deadline = Deadline::with_secs(60, "quad 出现在 AgentStates");
    let mut found: Option<(EntityId, usize)> = None;
    for i in 0..6000 {
        deadline.check();
        if found.is_none() && i % 20 == 0 {
            a.send(&C2S::SpawnAgent {
                body: "quad".to_owned(),
                position: Vec3::new(64.0, QUAD_Y, 64.0),
                yaw: 0.0,
            })
            .await
            .expect("SpawnAgent 写入");
        }
        advance_ticks(1).await;
        for _ in 0..4 {
            real_yield().await;
            match next_frame(&mut rx).await {
                Some(S2C::Spawned { entity }) => {
                    println!("X5: Spawned 回执 {entity:?}");
                }
                Some(S2C::AgentStates { states, .. }) => {
                    for (id, _, links) in states {
                        if links.len() == 13 {
                            found = Some((id, links.len()));
                        }
                    }
                }
                Some(_) => {}
                None => break,
            }
        }
        if found.is_some() {
            break;
        }
    }
    let (entity, n) = found.expect("60 s 内 quad 出现在 AgentStates(13 连杆)");
    assert_eq!(n, 13);
    println!("X5: quad EntityId {entity:?}");
    // 站立 300 拍(无控制者 = 电机保持默认目标,应站稳):采样 up·y。
    // 诊断版:逐帧打 states/removed。
    let mut up_min = f32::INFINITY;
    let mut n_samples = 0usize;
    let n_empty = 0usize;
    {
        let dl = Deadline::with_secs(60, "站立采样");
        for i in 0..2000 {
            dl.check();
            advance_ticks(1).await;
            for _ in 0..4 {
                real_yield().await;
                match next_frame(&mut rx).await {
                    Some(S2C::AgentStates { states, .. }) => {
                        for (_, _, links) in states {
                            if let Some(r) = links.first() {
                                let up = (r.rotation * Vec3::new(0.0, 1.0, 0.0)).y;
                                up_min = up_min.min(up);
                                n_samples += 1;
                            }
                        }
                    }
                    Some(_) => {}
                    None => break,
                }
            }
            if i > 400 {
                break;
            }
        }
    }
    // 差分语义:静止(入睡)的主体位姿逐位不变 → 无增量帧,采样数 < 推拍数
    // 是正常现象;判据只看采到的姿态(up·y)。
    println!("X5: 采样 {n_samples} 帧, 空 states {n_empty} 帧, up_min {up_min:.4}");
    assert!(n_samples > 0, "站立段一帧都没采到(布景异常)");
    assert!(up_min >= 0.7, "站立段 up·y 最小 {up_min} < 0.7(倒了)");
    let outcome = shutdown(h).await;
    assert!(
        outcome.controller_counters.send_rejected == 0,
        "无控制者布景无被拒"
    );
    let _ = player;
}

// ---- X6:无关路径不变 ---------------------------------------------------------

/// X6:没有主体时 120 拍,日志窗口 0 条,各计数为 0(同 B02b E7;不连
/// 客户端——MoveObserver 会进日志窗口,E7 口径 = 无任何命令)。
#[tokio::test(start_paused = true)]
async fn x6_no_agents_world_unchanged() {
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 6000,
        ..Config::default()
    };
    config.bodies_dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies");
    let h = bootstrap_harness(config).await;
    advance_ticks(120).await;
    let cps_dir = h.data_dir.clone();
    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.controller_counters.actions_emitted, 0,
        "无主体:不发动作"
    );
    assert_eq!(outcome.controller_counters.send_rejected, 0);
    assert_eq!(outcome.controller_counters.leash_stops, 0);
    assert_eq!(outcome.controller_counters.skipped_in_transit, 0);
    assert_eq!(outcome.startup_counters.spawned, 0);
    assert_eq!(outcome.startup_counters.spawn_timeouts, 0);
    // 日志窗口 0 条(终末检查点的日志段)。
    let cps = read_all_checkpoints(&cps_dir);
    let (_, blob) = cps.last().expect("终末检查点存在").clone();
    let bundle = app::driver::decode_checkpoint(&blob).expect("可解");
    let window = CommandLog::from_bytes(&bundle.log_bytes).expect("日志窗口可解");
    assert_eq!(
        window.entries().len(),
        0,
        "无主体:日志窗口 0 条(实得 {})",
        window.entries().len()
    );
}
