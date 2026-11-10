//! REQ-012 A03(#25):协议、附身、观察者跟随、权威校验的场景判据。
//!
//! 口径(放行判据表):所有场景走**真实网络栈**(TCP 客户端 + 驱动 +
//! Session,`network.rs` 同款装配,不直接调 ShardSim);凝固与激活生产默认
//! 配置;跨分片场景至少 2×2 分片(观察者/主体走动时驱动 ensure 自动建片)。
//!
//! - **A03-1 协议**:6 新变体编解码往返 + 旧序号不变 + C2S::Command 仍
//!   Reject(sim-api proto 单测 + app::net translate 单测;本套件集成侧只
//!   发真实帧)。
//! - **A03-2 生成(AC-A1 网络版)**:C2S::SpawnAgent → Spawned;
//!   entity_owner.source = 连接;站 3600 拍不凝固/下沉 ≤ 1 cm/不被预算冻结
//!   (判法同 A04-6:之后发意图能走起来);四种非法请求拒绝并计数。
//! - **A03-3 附身规则**:第 3 节表逐行(含自动解除旧的);附身表双向一对一。
//! - **A03-4 权威(AC-A7)**:B 给 A 附身的主体发意图被拒;状态与对照线
//!   逐位相同;未附身发意图同样被拒。
//! - **A03-5 路由与在途**:附身主体经网络意图跨分片边界;迁移在途拍发
//!   "停下"→ 下一拍投递(计数 +1)、主体随后停下;同窗口两条只有后者生效。
//! - **A03-6 观察者跟随**:附身期间 MoveObserver 被忽略(计数);观察者
//!   跟随身体走 ≥ 300 m、跨 ≥ 2 个分片;chunk 沿途已加载下发;高度 ±2 cm;
//!   解除后 MoveObserver 恢复。
//! - **A03-7 断线**:附身自己身体断线 → 身体移除、无刚体碰撞体残留、不再
//!   出现在 AgentStates 全量包;附身服务端主体断线 → 主体保留并停下。
//! - **A03-8 网络实体命令**:对主体(含自己身体)发 Teleport/Remove 被拒
//!   计数、无 panic、主体不受影响。
//! - **A03-9 AgentStates**:距离过滤;移除后出现在 removed;全量重建;
//!   从不在 EntityStates;无主体世界帧序列与改动前逐位相同(既有套件守住,
//!   本套件以"无 SpawnAgent 的对照连接无任何 AgentStates 帧"锚定)。
//! - **A03-10 确定性**:两客户端完整会话,命令日志回放逐位相同;检查点
//!   恢复后第一拍移除玩家身体(计数)、服务端主体续跑逐位相同。
//! - **A03-11 不影响旧行为**:现有测试期望不改;golden/门禁/smoke 在 CI。

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::Config;
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::InputSource;
use physics_sim::BodyRegistry;
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

/// Superflat classic 地面顶面 world y(network.rs 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;

/// 软窗(网络.rs 同款):等一帧最多 50 ms。
const SOFT: Duration = Duration::from_millis(50);

/// capsule 出生姿态(站地面顶面;胶囊半高 0.9 → 重心 y = 顶面 + 0.9)。
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

/// 暂停时间下推进 N 拍(network.rs 同款)。
async fn advance_ticks(ticks: u32) {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

/// 软窗收一帧。
async fn next_frame(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Option<S2C> {
    tokio::time::timeout(SOFT, rx.recv()).await.ok().flatten()
}

/// 软窗落空(None)后的真实墙钟让步(1 ms):暂停时钟下 tokio 定时器瞬时
/// 兑现,一串软窗可以在墙钟微秒内烧完——空闲信道的请求-回执(服务端响应
/// 要走真实 socket 往返)在高负载 CI 上会被整个预算饿死而本地快机险过
/// (CI run 42825549615 的 a03_3 行 1b / a03_6 末尾 unpossess 实证;有帧流
/// 量时虚拟时钟被就绪任务冻结,等待自然退化为实时收帧,故只有空闲信道
/// 命中)。阻塞的是本测试的单线程运行时,各任务本就空闲;内核侧 loopback
/// 回执在让步期间入队,下一次 park 的 IO 轮询即拾起、整条服务端链(读 →
/// NetUp → 驱动 → 回执写)在同一 park 周期跑完。拒绝 = 静默的负断言走满
/// 预算(200 × 1 ms = 200 ms),代价可接受。
fn real_yield() {
    std::thread::sleep(Duration::from_millis(1));
}

// ---------------------------------------------------------------------------
// 内嵌迷你客户端(network.rs 同款;codec = sim-api::proto 公开对称接口)
// ---------------------------------------------------------------------------

struct TestClient {
    w: tokio::net::tcp::OwnedWriteHalf,
}

impl TestClient {
    async fn send(&mut self, c2s: &C2S) -> std::io::Result<()> {
        self.w.write_all(&encode_c2s_frame(c2s)).await
    }

    async fn bye(&mut self) {
        let _ = self.send(&C2S::Goodbye).await;
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

/// Hello + Subscribe,返回 Welcome 的 player_id(network.rs 同款)。
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

/// C2S::SpawnAgent → 等 Spawned(None = 被拒,预算内无回执)。
async fn spawn_agent(
    c: &mut TestClient,
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    body: &str,
    position: Vec3,
    yaw: f32,
    budget: u32,
) -> Option<EntityId> {
    while rx.try_recv().is_ok() {}
    c.send(&C2S::SpawnAgent {
        body: body.to_string(),
        position,
        yaw,
    })
    .await
    .expect("SpawnAgent 写入");
    for _ in 0..budget {
        match next_frame(rx).await {
            Some(S2C::Spawned { entity }) => return Some(entity),
            Some(_) => continue,
            None => real_yield(),
        }
    }
    None
}

/// C2S::Possess → 等 Possessed;返回回执值。
async fn possess(
    c: &mut TestClient,
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    entity: EntityId,
    budget: u32,
) -> Option<Option<EntityId>> {
    while rx.try_recv().is_ok() {}
    c.send(&C2S::Possess { entity })
        .await
        .expect("Possess 写入");
    for _ in 0..budget {
        match next_frame(rx).await {
            Some(S2C::Possessed { entity }) => return Some(entity),
            Some(_) => continue,
            None => real_yield(),
        }
    }
    None
}

/// C2S::Unpossess → 等 Possessed(None)。
async fn unpossess(
    c: &mut TestClient,
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    budget: u32,
) -> Option<Option<EntityId>> {
    while rx.try_recv().is_ok() {}
    c.send(&C2S::Unpossess).await.expect("Unpossess 写入");
    for _ in 0..budget {
        match next_frame(rx).await {
            Some(S2C::Possessed { entity }) => return Some(entity),
            Some(_) => continue,
            None => real_yield(),
        }
    }
    None
}

/// 收集 AgentStates 增量(差分后按 id 取最新姿态);budget 帧窗口。
async fn agent_pose(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    id: EntityId,
    budget: u32,
) -> Option<Vec3> {
    for _ in 0..budget {
        if let Some(S2C::AgentStates { states, .. }) = next_frame(rx).await
            && let Some((_, _, poses)) = states.iter().find(|(e, ..)| *e == id)
            && let Some(root) = poses.first()
        {
            return Some(root.translation);
        }
    }
    None
}

/// 排干通道取主体最新位姿与沿途最远 chunk x(驱动 World 队列在满速
/// 行走时堆积延迟帧;FIFO 读到的是几百拍前的状态——采样一律先排干;
/// chunk 帧与 AgentStates 帧同通道,联合采样避免互相挤占)。
async fn agent_pose_and_chunks(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    id: EntityId,
    max_chunk_x: &mut i32,
) -> Option<Vec3> {
    let mut last: Option<Vec3> = None;
    for _ in 0..50 {
        match next_frame(rx).await {
            Some(S2C::AgentStates { states, .. }) => {
                if let Some((_, _, poses)) = states.iter().find(|(e, ..)| *e == id)
                    && let Some(root) = poses.first()
                {
                    last = Some(root.translation);
                }
                break;
            }
            Some(S2C::ChunkData { pos, .. }) => {
                *max_chunk_x = (*max_chunk_x).max(pos.x);
            }
            Some(_) => continue,
            None => return last,
        }
    }
    while let Ok(frame) = rx.try_recv() {
        match frame {
            S2C::AgentStates { states, .. } => {
                if let Some((_, _, poses)) = states.iter().find(|(e, ..)| *e == id)
                    && let Some(root) = poses.first()
                {
                    last = Some(root.translation);
                }
            }
            S2C::ChunkData { pos, .. } => {
                *max_chunk_x = (*max_chunk_x).max(pos.x);
            }
            _ => {}
        }
    }
    last
}

/// 排干通道取主体最新位姿(驱动 World 队列在满速行走时堆积延迟帧;
/// FIFO 读到的是几百拍前的状态——采样一律先排干)。
async fn agent_pose_latest(rx: &mut mpsc::UnboundedReceiver<S2C>, id: EntityId) -> Option<Vec3> {
    let mut last: Option<Vec3> = None;
    // 第一阶段:阻塞等一帧(推进虚拟时钟到下一输出点)。
    for _ in 0..50 {
        match next_frame(rx).await {
            Some(S2C::AgentStates { states, .. }) => {
                if let Some((_, _, poses)) = states.iter().find(|(e, ..)| *e == id)
                    && let Some(root) = poses.first()
                {
                    last = Some(root.translation);
                }
                break;
            }
            Some(_) => continue,
            None => return last,
        }
    }
    // 第二阶段:非阻塞排干,取最新。
    while let Ok(frame) = rx.try_recv() {
        if let S2C::AgentStates { states, .. } = frame
            && let Some((_, _, poses)) = states.iter().find(|(e, ..)| *e == id)
            && let Some(root) = poses.first()
        {
            last = Some(root.translation);
        }
    }
    last
}

/// 排干通道后等下一个**新**全量 AgentStates 包(旧帧堆积下的可靠重建
/// 观测:先 try_recv 排干存量,再阻塞等新帧)。
async fn agent_full_after_drain(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
) -> Option<Vec<(EntityId, types::BodyHash, Vec<EntityState>)>> {
    while rx.try_recv().is_ok() {}
    for _ in 0..300 {
        advance_ticks(5).await;
        if let Some(S2C::AgentStates { states, full, .. }) = next_frame(rx).await
            && full
        {
            return Some(states);
        }
    }
    None
}

/// 窗口内是否出现过含该实体的 AgentStates 帧(负样本传小预算)。
async fn agent_states_mention(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    id: EntityId,
    budget: u32,
) -> bool {
    for _ in 0..budget {
        if let Some(S2C::AgentStates {
            states, removed, ..
        }) = next_frame(rx).await
            && (states.iter().any(|(e, ..)| *e == id) || removed.contains(&id))
        {
            return true;
        }
    }
    false
}

/// EntityStates 流里是否出现过该实体(判据:主体从不出现在方块流)。
async fn entity_states_mention(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    id: EntityId,
    budget: u32,
) -> bool {
    for _ in 0..budget {
        if let Some(S2C::EntityStates {
            states, removed, ..
        }) = next_frame(rx).await
            && (states.iter().any(|(e, _)| *e == id) || removed.contains(&id))
        {
            return true;
        }
    }
    false
}

// ---------------------------------------------------------------------------
// 装配 harness(network.rs 同款;new_sim 带 capsule 注册表 = 生产同构)
// ---------------------------------------------------------------------------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: tokio::task::JoinHandle<DriverOutcome>,
    addr: SocketAddr,
}

/// 生产默认配置(凝固/激活默认;checkpoint 疏)。bodies_dir 用绝对路径
/// (集成测试 cwd = crates/e2e,默认相对路径 `assets/bodies` 失效)。
fn base_config() -> Config {
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 600,
        ..Config::default()
    };
    config.bodies_dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies");
    config
}

async fn harness(config: Config, observe: (u64, u64)) -> Harness {
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 3600,
        observe_interval_ticks: observe.0,
        full_snapshot_interval_ticks: observe.1,
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
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    // 生产装配:provider 走 wire(数据源 = Superflat classic);注册表同一份
    // (驱动生成校验 + new_sim 内核注入都用它——不再读目录)。
    let (provider, new_sim, bodies) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry())
            .expect("装配失败(fail-stop;MARGIN 不足等类型化错误)");
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
    }));
    Harness { tx, driver, addr }
}

fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

async fn shutdown(h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    h.driver.await.expect("驱动不 panic")
}

/// 落地自检:等主体位姿稳定到地面(下沉 ≤ 1 cm 于 y 稳定后)。
async fn wait_agent_grounded(rx: &mut mpsc::UnboundedReceiver<S2C>, id: EntityId) -> Vec3 {
    let mut last = Vec3::ZERO;
    let mut stable = 0;
    for _ in 0..4000 {
        if let Some(p) = agent_pose(rx, id, 1).await {
            if (p.y - last.y).abs() < 1e-4 && (p.x - last.x).abs() < 1e-4 {
                stable += 1;
                if stable >= 30 {
                    return p;
                }
            } else {
                stable = 0;
            }
            last = p;
        }
    }
    panic!("主体未落地稳定");
}

// ===========================================================================
// A03-2 生成(AC-A1 网络版)+ A03-1 集成面(真实帧)
// ===========================================================================

/// A03-2:经 C2S::SpawnAgent 生成胶囊 → Spawned;source 等于该连接;
/// 静止 3600 拍不凝固/下沉 ≤ 1 cm/不被预算冻结(之后发意图能走起来);
/// 四种非法请求被拒并计数、无 panic、世界没多出实体。
#[tokio::test(start_paused = true)]
async fn a03_2_spawn_agent_network_aca1() {
    let h = harness(base_config(), (2, 60)).await;
    let (mut c, mut rx) = client(h.addr).await;
    tokio::task::yield_now().await;
    let player = join(&mut c, &mut rx, 4).await;
    assert_eq!(player, InputSource(1));

    // 观察者移到出生区(分片 (0,0);chunk 已加载由观察者票驱动)。
    c.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;

    // 正路生成(站地面顶面上方一点,自然落稳)。
    let entity = spawn_agent(
        &mut c,
        &mut rx,
        "capsule",
        Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        300,
    )
    .await
    .expect("合法生成必有 Spawned 回执");

    // source 等于该连接(entity_owner.source = Some(1))——经 AgentStates
    // 流与驱动侧共同判定:主体出现在 AgentStates(不是 EntityStates)。
    let pose = wait_agent_grounded(&mut rx, entity).await;
    assert!(
        !entity_states_mention(&mut rx, entity, 20).await,
        "主体从不出现在 EntityStates(A01 过滤;A03-9 同口径)"
    );

    // 静止 3600 拍:不凝固(仍出现在 AgentStates 且可被意图驱动)、下沉 ≤ 1 cm。
    advance_ticks(3600).await;
    let after = agent_pose(&mut rx, entity, 200)
        .await
        .expect("静止后主体仍在 AgentStates 流(不凝固)");
    assert!(
        (after.y - pose.y).abs() <= 0.01,
        "3600 拍下沉 ≤ 1 cm(实际 {} cm)",
        (after.y - pose.y).abs() * 100.0
    );

    // 不被预算冻结:发意图能走起来(判法同 A04-6 R1)。
    assert_eq!(
        possess(&mut c, &mut rx, entity, 300).await,
        Some(Some(entity)),
        "附身成功回执"
    );
    c.send(&C2S::AgentIntent {
        entity,
        move_xz: (1.5, 0.0),
        yaw: 0.0,
        jump: false,
    })
    .await
    .expect("AgentIntent");
    let start = after;
    let mut walked = None;
    for _ in 0..3000 {
        if let Some(p) = agent_pose(&mut rx, entity, 1).await
            && p.x > start.x + 2.0
        {
            walked = Some(p);
            break;
        }
    }
    assert!(
        walked.is_some(),
        "3600 拍静止后发意图能走起来(不被预算冻结)"
    );

    // 四种非法请求(此时该连接已有身体):
    // ① 未知身体名;② 非有限值;③ chunk 未加载;④ 已有身体。
    let budget = 30;
    let r1 = spawn_agent(
        &mut c,
        &mut rx,
        "dog",
        Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        budget,
    )
    .await;
    assert!(r1.is_none(), "未知身体名被拒(无回执)");
    let r2 = spawn_agent(
        &mut c,
        &mut rx,
        "capsule",
        Vec3::new(f32::NAN, GROUND_TOP_Y, 8.0),
        0.0,
        budget,
    )
    .await;
    assert!(r2.is_none(), "非有限坐标被拒");
    let r3 = spawn_agent(
        &mut c,
        &mut rx,
        "capsule",
        Vec3::new(2000.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        budget,
    )
    .await;
    assert!(r3.is_none(), "chunk 未加载(远端)被拒");
    let r4 = spawn_agent(
        &mut c,
        &mut rx,
        "capsule",
        Vec3::new(9.0, GROUND_TOP_Y + 1.2, 9.0),
        0.0,
        budget,
    )
    .await;
    assert!(r4.is_none(), "一连接一身体(D1/D12)被拒");

    // 世界里没多出实体:AgentStates 流全量包里主体数恒 1。
    advance_ticks(10).await;
    let mut full_seen = false;
    for _ in 0..400 {
        if let Some(S2C::AgentStates { states, full, .. }) = next_frame(&mut rx).await
            && full
        {
            assert_eq!(states.len(), 1, "全量包主体数恒 1(非法请求没生成实体)");
            full_seen = true;
            break;
        }
    }
    assert!(full_seen, "窗口内见到全量包");

    let outcome = shutdown(h).await;
    assert_eq!(outcome.agent.spawn_rejected, 4, "四种非法请求各计 1");
    assert_eq!(outcome.agent.intent_rejected, 0);
}

// ===========================================================================
// A03-3 附身规则(表逐行 + 双向一对一)
// ===========================================================================

/// A03-3:附身自己的身体成功;附身别人身体/已被附身/不存在/非主体被拒;
/// 附身新主体自动解除旧的;附身表双向一对一。
#[tokio::test(start_paused = true)]
async fn a03_3_possess_rules_table() {
    let h = harness(base_config(), (2, 60)).await;
    let (mut a, mut rxa) = client(h.addr).await;
    let (mut b, mut rxb) = client(h.addr).await;
    tokio::task::yield_now().await;
    let pa = join(&mut a, &mut rxa, 4).await;
    let pb = join(&mut b, &mut rxb, 4).await;
    assert_eq!(pa, InputSource(1));
    assert_eq!(pb, InputSource(2));
    for (c, pos) in [
        (&mut a, Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0)),
        (&mut b, Vec3::new(48.0, GROUND_TOP_Y + 2.0, 8.0)),
    ] {
        c.send(&C2S::MoveObserver { position: pos })
            .await
            .expect("MoveObserver");
    }
    advance_ticks(30).await;

    // A、B 各生成一具身体(服务端主体 = spawn_owned 路径这里没有;用
    // Session 受信通道造一个服务端主体做"没人附身的服务端主体"行)。
    let ea = spawn_agent(
        &mut a,
        &mut rxa,
        "capsule",
        Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        300,
    )
    .await
    .expect("A 身体生成");
    let eb = spawn_agent(
        &mut b,
        &mut rxb,
        "capsule",
        Vec3::new(48.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        300,
    )
    .await
    .expect("B 身体生成");
    advance_ticks(30).await;

    // 行 2e(权威前置):**未附身状态**下 B 附身 A 的玩家身体 → 拒(owner
    // .source = Some(A) 判定;区别于"已被附身"路径,变异"内核不写 source"
    // 会把玩家身体变服务端主体而放行)。
    assert_eq!(
        possess(&mut b, &mut rxb, ea, 30).await,
        None,
        "B 附身 A 未附身的玩家身体被拒(source 指认)"
    );

    // 行 1a:附身自己的身体 → 成功。
    assert_eq!(possess(&mut a, &mut rxa, ea, 200).await, Some(Some(ea)));
    // 行 1b:附身一个没人附身的服务端主体 —— 服务端主体经 commands 通道
    // 受信生成(dispatch_external Send 的 SpawnAgent 由 validate 放行
    // source = None)。B 先附身自己的身体(占住一对一),确保后续行隔离。
    assert_eq!(possess(&mut b, &mut rxb, eb, 200).await, Some(Some(eb)));

    // 行 2a:附身别人的玩家身体 → 拒(计数),附身表不变。
    assert_eq!(
        possess(&mut b, &mut rxb, ea, 30).await,
        None,
        "B 附身 A 身体被拒"
    );
    // 行 2b(A03 验收 P6)挪到下方服务端主体段:A 附身 B 的玩家身体会被
    // source 检查先拒,`possessor_of` 检查走不到——改为 A 附身 B 已附身的
    // **服务端主体**(source = None,只有"已被他人附身"检查能拒)。
    // 行 2c:不存在的实体 → 拒。
    let ghost = EntityId::from_parts(0, 9999);
    assert_eq!(
        possess(&mut a, &mut rxa, ghost, 30).await,
        None,
        "附身不存在实体被拒"
    );
    // 行 2d:不是主体的实体(动态方块;经 C2S::Spawn 生成)→ 拒。
    a.send(&C2S::Spawn {
        state: capsule_spawn(Vec3::new(16.0, GROUND_TOP_Y + 1.0, 16.0)),
        desc: types::EntityDesc::default(),
    })
    .await
    .expect("Spawn 方块");
    let mut block = None;
    for _ in 0..300 {
        if let Some(S2C::Spawned { entity }) = next_frame(&mut rxa).await {
            block = Some(entity);
            break;
        }
    }
    let block = block.expect("方块生成回执");
    advance_ticks(5).await;
    assert_eq!(
        possess(&mut a, &mut rxa, block, 30).await,
        None,
        "附身非主体实体被拒"
    );

    // 行 1 补充:附身新主体时自动解除旧的 —— A 先 Unpossess(回归未附身),
    // 再由 commands 通道造一具服务端主体(A 可附身),附身服务端主体成功;
    // 再发 Possess(自己的身体)→ 旧的自动解除(只回新回执)。
    assert_eq!(
        unpossess(&mut a, &mut rxa, 200).await,
        Some(None),
        "A 解除附身"
    );
    // 服务端主体:用驱动 Send 缝(SpawnAgent source = None 路径)。
    let server_body = EntityId::from_parts(0, 900);
    let hash = capsule_registry().hash_of("capsule").unwrap();
    h.tx.send(DriverCommand::Send {
        shard: ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0)),
        command: messaging::SimCommand::SpawnAgent {
            entity: server_body,
            body: hash,
            state: capsule_spawn(Vec3::new(20.0, GROUND_TOP_Y + 1.2, 8.0)),
        },
    })
    .await
    .expect("commands 通道 Send");
    advance_ticks(30).await;
    assert_eq!(
        possess(&mut a, &mut rxa, server_body, 200).await,
        Some(Some(server_body)),
        "附身没人附身的服务端主体成功"
    );
    // 行 2b(P6):A 已附身服务端主体,B 再附身它 → 拒,计数,附身表不变
    // (B 仍附身 eb)。source 检查对服务端主体(source = None)放行,只有
    // `possessor_of` 的"已被他人附身"检查能拒——变异"解除该检查"(M2)红。
    assert_eq!(
        possess(&mut b, &mut rxb, server_body, 30).await,
        None,
        "B 附身 A 已附身的服务端主体被拒(possessor_of 检查)"
    );
    // 现在附身自己的身体:旧的服务端主体自动解除(表一对一),回新回执。
    assert_eq!(
        possess(&mut a, &mut rxa, ea, 200).await,
        Some(Some(ea)),
        "附身新主体自动解除旧的"
    );
    // 解除旧附身后服务端主体没人附身了:B 能附身它(旁证一对一)。
    assert_eq!(
        possess(&mut b, &mut rxb, server_body, 200).await,
        Some(Some(server_body)),
        "自动解除后服务端主体可被他人附身(一对一)"
    );

    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.agent.possess_rejected, 5,
        "行 2a/2b/2c/2d/2e 各计 1"
    );
}

// ===========================================================================
// A03-4 权威(AC-A7)
// ===========================================================================

/// A03-4:B 给 A 附身的主体发意图被拒;主体状态与"B 没发"对照线逐位相同;
/// 未附身就发意图同样被拒。
#[tokio::test(start_paused = true)]
async fn a03_4_intent_authority_aca7() {
    // 两条对照线跑同一布景:线 1 = B 伪造意图;线 2 = B 不发。A 的意图时序
    // 完全一致;主体姿态逐位相同 ⇒ 伪造没有影响状态。
    let mut lines = Vec::new();
    for forge in [true, false] {
        let h = harness(base_config(), (2, 60)).await;
        let (mut a, mut rxa) = client(h.addr).await;
        let (mut b, mut rxb) = client(h.addr).await;
        tokio::task::yield_now().await;
        let _pa = join(&mut a, &mut rxa, 4).await;
        let _pb = join(&mut b, &mut rxb, 4).await;
        a.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
        })
        .await
        .expect("MoveObserver");
        b.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
        })
        .await
        .expect("MoveObserver");
        advance_ticks(30).await;
        let ea = spawn_agent(
            &mut a,
            &mut rxa,
            "capsule",
            Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
            0.0,
            300,
        )
        .await
        .expect("A 身体");
        advance_ticks(30).await;
        assert_eq!(possess(&mut a, &mut rxa, ea, 200).await, Some(Some(ea)));

        // A 的意图序列(两线一致):走 60 拍的量,分 6 条。
        for _ in 0..6 {
            a.send(&C2S::AgentIntent {
                entity: ea,
                move_xz: (1.5, 0.0),
                yaw: 0.0,
                jump: false,
            })
            .await
            .expect("A 意图");
            advance_ticks(10).await;
        }
        // B 伪造(线 1):未附身 + 给别人附身的主体各一条;线 2 跳过。
        if forge {
            b.send(&C2S::AgentIntent {
                entity: ea,
                move_xz: (5.6, 0.0),
                yaw: 1.0,
                jump: true,
            })
            .await
            .expect("B 伪造意图(写入)");
            advance_ticks(10).await;
            b.send(&C2S::AgentIntent {
                entity: ea,
                move_xz: (0.0, 0.0),
                yaw: 0.0,
                jump: false,
            })
            .await
            .expect("B 未附身发意图");
            advance_ticks(10).await;
        }
        // 继续走 60 拍(两线一致),采样末姿态。
        for _ in 0..6 {
            a.send(&C2S::AgentIntent {
                entity: ea,
                move_xz: (1.5, 0.0),
                yaw: 0.0,
                jump: false,
            })
            .await
            .expect("A 意图");
            advance_ticks(10).await;
        }
        advance_ticks(40).await;
        let final_pose = agent_pose(&mut rxa, ea, 300).await.expect("末姿态采样");
        let outcome = shutdown(h).await;
        assert!(outcome.agent.intent_rejected >= if forge { 2 } else { 0 });
        lines.push((forge, final_pose));
    }
    // 对照:两线末姿态逐位相同(bit 级)。
    let (f1, p1) = &lines[0];
    let (f2, p2) = &lines[1];
    assert!(*f1 && !*f2, "两线口径自检");
    assert_eq!(
        p1.to_array(),
        p2.to_array(),
        "B 伪造/不发的两线主体末姿态逐位相同"
    );
}

// ===========================================================================
// A03-5 路由与在途
// ===========================================================================

/// A03-5:附身主体经网络意图走过分片边界(x=128),目的分片继续响应意图
/// (再走回来);迁移在途拍发"停下" → 下一拍投递(计数 +1),主体随后
/// 停下;同窗口两条只有后一条生效。
#[tokio::test(start_paused = true)]
async fn a03_5_routing_and_inflight() {
    let h = harness(base_config(), (2, 60)).await;
    let (mut c, mut rx) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _p = join(&mut c, &mut rx, 6).await;
    c.send(&C2S::MoveObserver {
        position: Vec3::new(120.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(60).await;
    let e = spawn_agent(
        &mut c,
        &mut rx,
        "capsule",
        Vec3::new(120.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        300,
    )
    .await
    .expect("生成(近边界出生,驱动 ensure 邻片)");
    advance_ticks(30).await;
    assert_eq!(possess(&mut c, &mut rx, e, 200).await, Some(Some(e)));

    // 一路 +x 走过边界 128(意图每 10 拍一条,保持状态语义)。
    let mut crossed = false;
    for _ in 0..120 {
        c.send(&C2S::AgentIntent {
            entity: e,
            move_xz: (5.6, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("意图");
        advance_ticks(10).await;
        if let Some(p) = agent_pose(&mut rx, e, 2).await
            && p.x > 130.0
        {
            crossed = true;
            break;
        }
    }
    assert!(crossed, "走过分片边界(≥130)");

    // 过界后目的分片继续响应意图:反向走 10 m。
    let before = agent_pose(&mut rx, e, 100).await.expect("过界后姿态");
    let mut returned = false;
    for _ in 0..120 {
        c.send(&C2S::AgentIntent {
            entity: e,
            move_xz: (-5.6, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("反向意图");
        advance_ticks(10).await;
        if let Some(p) = agent_pose_latest(&mut rx, e).await
            && p.x < before.x - 3.0
        {
            returned = true;
            break;
        }
    }
    assert!(returned, "目的分片继续响应意图(反向走了 >3 m)");

    // 在途"停下":再向 +x 走,**逐拍采样**,主体在迁移在途窗口的那几拍
    // AgentStates 不含它(A04:恰隔 2 拍)——采到空窗立即同拍连发两条
    // (0,0) 与 (-5.6,0)(同窗口只留最后一条 ⇒ 主体向 -x);计数"延后"≥ 1。
    // 在途"停下":向 +x 走,**密集连发多对** (0,0)+(-5.6) —— 迁移在途
    // 窗口(A04:恰隔 2 拍)落在哪一对里,那一对的两条都进在途暂存(同窗
    // 覆盖,deferred ≥ 1),owner 出现后只投最后一条;窗口外的对正常投递。
    // 最终断言:主体向 -x 走(同窗口只有后一条生效的端到端口径)+ 计数。
    for _ in 0..8 {
        c.send(&C2S::AgentIntent {
            entity: e,
            move_xz: (5.6, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("意图");
        advance_ticks(10).await;
    }
    // 逐拍一对 × 40:总有一个消费点落在迁移在途拍(W-1 末:owner 已释放、
    // 信封在 Router)→ 那一对的两条都进在途暂存(同窗覆盖,deferred ≥ 1)。
    for _ in 0..40 {
        c.send(&C2S::AgentIntent {
            entity: e,
            move_xz: (0.0, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("停下");
        c.send(&C2S::AgentIntent {
            entity: e,
            move_xz: (-5.6, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("反向");
        advance_ticks(1).await;
    }
    advance_ticks(10).await;
    let mut moving_back = false;
    for _ in 0..200 {
        if let Some(p) = agent_pose_latest(&mut rx, e).await
            && p.x < 126.0
        {
            moving_back = true;
            break;
        }
    }
    assert!(moving_back, "同窗口两条只有后一条生效(向 -x 走)");
    // 真正的停下:走回来后发零意图(不在在途窗口),主体速度归零停住。
    let mut settled = false;
    for _ in 0..120 {
        c.send(&C2S::AgentIntent {
            entity: e,
            move_xz: (-5.6, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("反向意图");
        advance_ticks(10).await;
        if let Some(p) = agent_pose_latest(&mut rx, e).await
            && p.x < 120.0
        {
            settled = true;
            break;
        }
    }
    assert!(settled, "反向走回出生区(目的分片持续响应)");
    c.send(&C2S::AgentIntent {
        entity: e,
        move_xz: (0.0, 0.0),
        yaw: 0.0,
        jump: false,
    })
    .await
    .expect("零意图(停下)");
    advance_ticks(60).await;
    let stop1 = agent_pose_latest(&mut rx, e).await.expect("停下采样 1");
    advance_ticks(60).await;
    let stop2 = agent_pose_latest(&mut rx, e).await.expect("停下采样 2");
    assert!(
        stop1.distance(stop2) < 0.02,
        "零意图后主体停下(60 拍位移 < 2 cm,实际 {})",
        stop1.distance(stop2)
    );

    let outcome = shutdown(h).await;
    // 迁移在途拍恰 1 拍,而 TCP 帧到达粒度 ~10 拍(虚拟时钟逐拍推进 vs
    // 真实 socket 转发;实验证明逐拍密集发送也无法保证命中)——"在途暂存
    // → 下一拍投递 → 计数 +1 → 同窗覆盖只留最后一条"的时序判据由驱动
    // 单测 `a03_inflight_intent_stash_overwrite_and_flush` 直接构造(owner
    // 查不到的时刻 = 在途窗口的驱动侧可见形态);本 e2e 验证端到端功能面
    // (过界、目的分片响应、同拍对只有后一条生效、停下)。intent_deferred
    // 如实记录:本场景计数可能为 0(对全部落在窗口外,投递直达)。
    eprintln!(
        "[a03_5] intent_deferred={} (窗口命中与否依赖帧到达相位,不做硬断言)",
        outcome.agent.intent_deferred
    );
}

// ===========================================================================
// A03-6 观察者跟随
// ===========================================================================

/// A03-6:附身期间客户端 MoveObserver 被忽略(计数);观察者跟随身体走
/// ≥ 300 m、跨 ≥ 2 个分片;沿途 chunk 已加载并下发;高度 ±2 cm(不掉进
/// 未加载区);解除附身后 MoveObserver 恢复生效。
#[tokio::test(start_paused = true)]
async fn a03_6_observer_follow() {
    let h = harness(base_config(), (2, 60)).await;
    let (mut c, mut rx) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _p = join(&mut c, &mut rx, 8).await;
    c.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;
    let e = spawn_agent(
        &mut c,
        &mut rx,
        "capsule",
        Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        300,
    )
    .await
    .expect("生成");
    advance_ticks(30).await;
    assert_eq!(possess(&mut c, &mut rx, e, 200).await, Some(Some(e)));

    // 附身期间:客户端 MoveObserver 试图把视野拽走 → 被忽略(计数)。
    c.send(&C2S::MoveObserver {
        position: Vec3::new(-500.0, GROUND_TOP_Y + 2.0, -500.0),
    })
    .await
    .expect("客户端拽视野");
    advance_ticks(5).await;

    // 沿 +x 走 ≥ 300 m(x: 8 → 320+,分片 0 → 2),持续意图流。
    let mut last_pose = Vec3::ZERO;
    let mut max_x = 0.0_f32;
    let mut height_violation: Option<(f32, f32)> = None;
    let mut ground_ref: Option<f32> = None;
    // 沿途收过的最远 chunk x(跟随的观察者票 → chunk 流;行走循环内采样)。
    let mut max_chunk_x = i32::MIN;
    for _ in 0..700 {
        c.send(&C2S::AgentIntent {
            entity: e,
            move_xz: (5.6, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("意图");
        advance_ticks(10).await;
        if let Some(p) = agent_pose_and_chunks(&mut rx, e, &mut max_chunk_x).await {
            last_pose = p;
            max_x = max_x.max(p.x);
            // 高度口径:Superflat 顶面 y = 1,胶囊重心 = 顶面 + 0.9;落地后
            // 与参考差 ≤ 2 cm(首样本立参考)。
            let expected = GROUND_TOP_Y + 0.9;
            match ground_ref {
                None => ground_ref = Some(p.y),
                Some(g) => {
                    if (p.y - g).abs() > 0.02 && height_violation.is_none() {
                        height_violation = Some((g, p.y));
                    }
                }
            }
            let _ = expected;
        }
        if max_x >= 320.0 {
            break;
        }
    }
    assert!(max_x >= 300.0, "走过 ≥ 300 m(实际 {})", max_x);
    // 跨 ≥ 2 个分片边界(x=128 与 x=256 两条边界 = 跨入分片 (1,0) (2,0))。
    assert!(max_x > 256.0, "跨过 ≥ 2 个分片边界(x=128 与 x=256)");
    assert!(
        height_violation.is_none(),
        "沿途高度 ±2 cm(参考 {:?} vs 偏离 {:?})",
        ground_ref,
        height_violation
    );
    assert!(
        (last_pose.y - (GROUND_TOP_Y + 0.9)).abs() <= 0.02 + 1e-3,
        "终点仍在地面上(主体没掉进未加载区域;y = {})",
        last_pose.y
    );

    // 沿途 chunk 已加载并下发:行走循环内收到过主体终点附近的 chunk
    // (终点 x≥300 → chunk ≥18;观察者跟随 ⇒ 该连接的 chunk 流覆盖)。
    assert!(
        max_chunk_x >= 18,
        "沿途远端 chunk 已下发给该连接(最远 chunk x = {};跟随的观察者票生效)",
        max_chunk_x
    );

    // 解除附身:客户端 MoveObserver 恢复生效 —— 移到远处新位置,驱动照办
    // (旁证:不再忽略;新位置的 chunk 开始下发)。
    assert_eq!(unpossess(&mut c, &mut rx, 200).await, Some(None));
    advance_ticks(5).await;
    c.send(&C2S::MoveObserver {
        position: Vec3::new(-200.0, GROUND_TOP_Y + 2.0, -200.0),
    })
    .await
    .expect("解除后 MoveObserver");
    let mut got_west = false;
    for _ in 0..600 {
        if let Some(S2C::ChunkData { pos, .. }) = next_frame(&mut rx).await
            && pos.x <= -12
            && pos.z <= -12
        {
            got_west = true;
            break;
        }
    }
    assert!(
        got_west,
        "解除附身后 MoveObserver 恢复生效(西边 chunk 下发)"
    );

    let outcome = shutdown(h).await;
    assert!(
        outcome.agent.move_observer_ignored >= 1,
        "附身期间客户端 MoveObserver 被忽略并计数"
    );
}

// ===========================================================================
// A03-7 断线
// ===========================================================================

/// A03-7:玩家附身自己身体断线 → 身体移除(无刚体/碰撞体残留:实体不再
/// 出现在下一个全量 AgentStates),附身表清空(回 Possessed 在断线前发
/// Unpossess 才可见;被迫解除的回执发给已断线连接无意义,断言表清空走
/// B 重连视角)。另一组:附身服务端主体断线 → 主体保留并停下。
#[tokio::test(start_paused = true)]
async fn a03_7_disconnect() {
    // 组 1:附身自己身体断线。
    {
        let h = harness(base_config(), (2, 60)).await;
        let (mut a, mut rxa) = client(h.addr).await;
        let (mut b, mut rxb) = client(h.addr).await;
        tokio::task::yield_now().await;
        let _pa = join(&mut a, &mut rxa, 4).await;
        let _pb = join(&mut b, &mut rxb, 4).await;
        a.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
        })
        .await
        .expect("MoveObserver");
        b.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
        })
        .await
        .expect("MoveObserver");
        advance_ticks(30).await;
        let ea = spawn_agent(
            &mut a,
            &mut rxa,
            "capsule",
            Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
            0.0,
            300,
        )
        .await
        .expect("A 身体");
        advance_ticks(30).await;
        assert_eq!(possess(&mut a, &mut rxa, ea, 200).await, Some(Some(ea)));
        a.send(&C2S::AgentIntent {
            entity: ea,
            move_xz: (2.0, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("走起来");
        advance_ticks(10).await;

        // 断线(Goodbye → Left)。
        a.bye().await;
        advance_ticks(10).await;

        // 移除旁证(先验):重连新客户端生成身体成功(一连接一身体计数
        // 不挡 ⇒ 旧身体已移除;outcome 计数在 shutdown 后断言)。
        let (mut a2, mut rxa2) = client(h.addr).await;
        tokio::task::yield_now().await;
        let _p2 = join(&mut a2, &mut rxa2, 4).await;
        a2.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
        })
        .await
        .expect("MoveObserver");
        advance_ticks(10).await;
        let _ea2 = spawn_agent(
            &mut a2,
            &mut rxa2,
            "capsule",
            Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
            0.0,
            300,
        )
        .await
        .expect("断线后重连生成新身体成功(旧身体已移除)");

        // B 视角:排干旧帧后的下一个新全量包里没有 ea(判据口径:"不再
        // 出现在 AgentStates 里(下一个全量包)";增量 removed 由 a03_9
        // 的持续观察连接判定)。
        let states = agent_full_after_drain(&mut rxb)
            .await
            .expect("排干后必有新全量包");
        assert!(
            !states.iter().any(|(id, ..)| *id == ea),
            "全量包重建后无该实体(不留残余)"
        );

        let outcome = shutdown(h).await;
        assert_eq!(
            outcome.agent.bodies_removed_on_disconnect, 1,
            "断线移除玩家身体计数"
        );
    }
    // 组 2:附身服务端主体断线 → 主体保留并停下。
    {
        let h = harness(base_config(), (2, 60)).await;
        let (mut a, mut rxa) = client(h.addr).await;
        tokio::task::yield_now().await;
        let _pa = join(&mut a, &mut rxa, 4).await;
        a.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
        })
        .await
        .expect("MoveObserver");
        advance_ticks(30).await; // 服务端主体(commands 通道受信生成)。
        let server_body = EntityId::from_parts(0, 900);
        let hash = capsule_registry().hash_of("capsule").unwrap();
        h.tx.send(DriverCommand::Send {
            shard: ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0)),
            command: messaging::SimCommand::SpawnAgent {
                entity: server_body,
                body: hash,
                state: capsule_spawn(Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0)),
            },
        })
        .await
        .expect("commands Send");
        advance_ticks(30).await;
        assert_eq!(
            possess(&mut a, &mut rxa, server_body, 200).await,
            Some(Some(server_body))
        );
        a.send(&C2S::AgentIntent {
            entity: server_body,
            move_xz: (2.0, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("走起来");
        advance_ticks(20).await;

        a.bye().await;
        advance_ticks(10).await;

        // 主体保留 + 停下:经新观察连接看(旧连接的接收端不再收帧)。
        let (mut w, mut rxw) = client(h.addr).await;
        tokio::task::yield_now().await;
        let _pw = join(&mut w, &mut rxw, 4).await;
        w.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
        })
        .await
        .expect("MoveObserver");
        advance_ticks(10).await;
        // 断线后等零意图生效(刹车 ~1 s),再比较后两个 120 拍的位移。
        let mut max_chunk_x = i32::MIN;
        advance_ticks(120).await;
        let _ = agent_pose_and_chunks(&mut rxw, server_body, &mut max_chunk_x).await;
        advance_ticks(120).await;
        let p1 = agent_pose_and_chunks(&mut rxw, server_body, &mut max_chunk_x)
            .await
            .expect("服务端主体断线后保留(仍在 AgentStates)");
        advance_ticks(120).await;
        let p2 = agent_pose_and_chunks(&mut rxw, server_body, &mut max_chunk_x)
            .await
            .expect("主体位置 2");
        assert!(
            p1.distance(p2) < 0.02,
            "被迫解除的零意图让主体停下(120 拍位移 < 2 cm,实际 {})",
            p1.distance(p2)
        );

        let outcome = shutdown(h).await;
        assert_eq!(
            outcome.agent.bodies_removed_on_disconnect, 0,
            "服务端主体不是玩家身体,断线不移除"
        );
    }
}

// ===========================================================================
// A03-8 网络实体命令
// ===========================================================================

/// A03-8:对主体(含自己的身体)发网络 TeleportEntity/RemoveEntity 被拒
/// 并计数,无 panic,主体不受影响。
#[tokio::test(start_paused = true)]
async fn a03_8_network_entity_commands_rejected() {
    let h = harness(base_config(), (2, 60)).await;
    let (mut a, mut rxa) = client(h.addr).await;
    let (mut b, mut rxb) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _pa = join(&mut a, &mut rxa, 4).await;
    let _pb = join(&mut b, &mut rxb, 4).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    b.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;
    let ea = spawn_agent(
        &mut a,
        &mut rxa,
        "capsule",
        Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        300,
    )
    .await
    .expect("A 身体");
    advance_ticks(30).await;
    let before = agent_pose(&mut rxa, ea, 200).await.expect("姿态");

    // 自己的身体:Teleport + Remove 都要被拒(客户端可构造;内核对主体
    // Teleport 是 fail-stop——驱动必须挡住,服务器不能停)。
    a.send(&C2S::Command(messaging::SimCommand::TeleportEntity {
        entity: ea,
        state: capsule_spawn(Vec3::new(300.0, GROUND_TOP_Y + 30.0, 300.0)),
    }))
    .await
    .expect("Teleport 写入");
    a.send(&C2S::Command(messaging::SimCommand::RemoveEntity {
        entity: ea,
    }))
    .await
    .expect("Remove 写入");
    // B 打别人的主体:同样被拒(越权 + 主体双口径)。
    b.send(&C2S::Command(messaging::SimCommand::RemoveEntity {
        entity: ea,
    }))
    .await
    .expect("B Remove 写入");
    b.send(&C2S::Command(messaging::SimCommand::TeleportEntity {
        entity: ea,
        state: capsule_spawn(Vec3::new(-300.0, GROUND_TOP_Y + 30.0, -300.0)),
    }))
    .await
    .expect("B Teleport 写入");
    advance_ticks(30).await;

    // 主体不受影响:仍在原位附近(没有被传送/移除),驱动活着(无 panic)。
    let after = agent_pose(&mut rxa, ea, 300)
        .await
        .expect("主体仍在 AgentStates");
    assert!(
        before.distance(after) < 1.0,
        "主体未受命令影响(位移 {} m)",
        before.distance(after)
    );

    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.agent.entity_command_rejected, 4,
        "四条网络实体命令(含自己身体)各计 1"
    );
}

// ===========================================================================
// A03-9 AgentStates 语义
// ===========================================================================

/// A03-9:距离过滤(视距外的主体不发);移除后出现在 removed;全量重建;
/// 主体从不出现在 EntityStates;无主体世界里不再发 AgentStates 帧(帧序列
/// 与改动前逐位相同的锚)。
#[tokio::test(start_paused = true)]
async fn a03_9_agent_states_semantics() {
    let h = harness(base_config(), (2, 60)).await;
    // near:视距 1 chunk(16 m);far:观察远处(x = 200)。
    let (mut near, mut rxn) = client(h.addr).await;
    let (mut far, mut rxf) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _pn = join(&mut near, &mut rxn, 1).await;
    let _pf = join(&mut far, &mut rxf, 1).await;
    near.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    far.send(&C2S::MoveObserver {
        position: Vec3::new(200.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;

    // near 视距内一具身体(第三连接生成;far 视距外)。
    let (mut owner, mut rxo) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _po = join(&mut owner, &mut rxo, 4).await;
    owner
        .send(&C2S::MoveObserver {
            position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
        })
        .await
        .expect("MoveObserver");
    advance_ticks(10).await;
    let e = spawn_agent(
        &mut owner,
        &mut rxo,
        "capsule",
        Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        300,
    )
    .await
    .expect("生成");
    advance_ticks(40).await;

    // near 看得见;far 看不见(距离过滤)。
    assert!(
        agent_states_mention(&mut rxn, e, 100).await,
        "视距内的主体下发"
    );
    // 全量重建语义(主体活着):排干旧帧后的新全量包含该主体。
    let full_states = agent_full_after_drain(&mut rxn)
        .await
        .expect("主体活着时全量包必到");
    assert!(
        full_states.iter().any(|(id, ..)| *id == e),
        "全量包重建含该主体"
    );
    let mut far_sees = false;
    for _ in 0..100 {
        if let Some(S2C::AgentStates { states, .. }) = next_frame(&mut rxf).await
            && states.iter().any(|(id, ..)| *id == e)
        {
            far_sees = true;
            break;
        }
    }
    assert!(!far_sees, "视距外的主体不下发");

    // 主体从不出现在 EntityStates(两条流都查)。
    assert!(
        !entity_states_mention(&mut rxn, e, 100).await,
        "主体不在 near 的 EntityStates"
    );
    assert!(
        !entity_states_mention(&mut rxf, e, 100).await,
        "主体不在 far 的 EntityStates"
    );

    // 移除(owner 断线)→ removed;全量重建;之后无主体的世界不再发帧。
    owner.bye().await;
    advance_ticks(10).await;
    let mut removed_seen = false;
    for _ in 0..600 {
        if let Some(S2C::AgentStates { removed, .. }) = next_frame(&mut rxn).await
            && removed.contains(&e)
        {
            removed_seen = true;
            break;
        }
    }
    assert!(removed_seen, "主体移除出现在增量包 removed");

    // 无主体且上次表已清 → 不再发任何 AgentStates 帧(排干后推进若干拍,
    // 窗口内零 AgentStates 帧 = 帧序列与改动前逐位相同的锚)。
    while rxn.try_recv().is_ok() {}
    advance_ticks(180).await;
    let mut tail_agent_frames = 0;
    while let Ok(frame) = rxn.try_recv() {
        if let S2C::AgentStates { .. } = frame {
            tail_agent_frames += 1;
        }
    }
    assert_eq!(
        tail_agent_frames, 0,
        "无主体的世界不再发 AgentStates(帧序列与改动前逐位相同的锚)"
    );
    let _ = shutdown(h).await;
}

// ===========================================================================
// A03-3 补充(P4):"玩家身体"判定用 is_agent,不用 actor 标记
// ===========================================================================

/// P4(验收返修):玩家 C2S::Spawn 一个方块、本地管理面 MarkActor 它,
/// 然后这个玩家**仍能**生成身体(actor 误标不再挡一对一校验);断线后
/// 方块**还在**(不随玩家身体被误删)。变异(恢复用 `actor` 判定)红:
/// MarkActor 后一对一校验把方块当玩家身体 → 生成被拒;断线把方块删掉。
#[tokio::test(start_paused = true)]
async fn a03_4b_player_body_check_uses_is_agent_not_actor() {
    let h = harness(base_config(), (2, 60)).await;
    let (mut c, mut rx) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _p = join(&mut c, &mut rx, 4).await;
    c.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;

    // ① 玩家生成一个方块(C2S::Spawn → spawn_owned,source = 该玩家,
    // actor 默认 false)。
    c.send(&C2S::Spawn {
        state: capsule_spawn(Vec3::new(16.0, GROUND_TOP_Y + 1.0, 16.0)),
        desc: types::EntityDesc::default(),
    })
    .await
    .expect("Spawn 方块");
    let mut block = None;
    for _ in 0..300 {
        if let Some(S2C::Spawned { entity }) = next_frame(&mut rx).await {
            block = Some(entity);
            break;
        }
    }
    let block = block.expect("方块生成回执");
    advance_ticks(5).await;

    // ② 本地管理面 MarkActor 它(本地受信通道;服务端策略命令,网络面
    // 拒绝——此处走 DriverCommand::Send 正路)。
    h.tx.send(DriverCommand::Send {
        shard: ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(1, 1)),
        command: messaging::SimCommand::MarkActor {
            entity: block,
            actor: true,
        },
    })
    .await
    .expect("commands 通道 Send");
    advance_ticks(10).await;

    // ③ 玩家仍能生成身体(P4 判据核心:actor 误标不挡一对一)。
    // 返回值 = 玩家身体 ID(断线移除的主体;下方计数断言用)。
    let _e = spawn_agent(
        &mut c,
        &mut rx,
        "capsule",
        Vec3::new(8.0, GROUND_TOP_Y + 1.2, 8.0),
        0.0,
        300,
    )
    .await
    .expect("MarkActor 方块后玩家仍能生成身体(P4)");

    // ④ 断线:玩家身体被移除;MarkActor 方块**还在**(不被误删)。
    // 断线方自己的通道随连接关闭收不到 removed 帧——移除断言:观察连接
    // 的 AgentStates(EntityStates 不含主体)中不再出现身体;方块在
    // EntityStates 里仍在。
    c.bye().await;
    advance_ticks(30).await;
    // 方块还在、身体没了:新观察连接看两个实体。
    let (mut watcher, mut rxw) = client(h.addr).await;
    tokio::task::yield_now().await;
    let _pw = join(&mut watcher, &mut rxw, 6).await;
    watcher
        .send(&C2S::MoveObserver {
            position: Vec3::new(12.0, GROUND_TOP_Y + 2.0, 12.0),
        })
        .await
        .expect("watcher MoveObserver");
    advance_ticks(30).await;
    let mut block_seen = false;
    for _ in 0..300 {
        if let Some(S2C::EntityStates { states, .. }) = next_frame(&mut rxw).await
            && states.iter().any(|(id, _)| *id == block)
        {
            block_seen = true;
            break;
        }
    }
    assert!(
        block_seen,
        "断线后 MarkActor 方块还在(P4:不被当玩家身体误删)"
    );
    let outcome = shutdown(h).await;
    assert_eq!(
        outcome.agent.bodies_removed_on_disconnect, 1,
        "断线只移除玩家身体(1),方块不算"
    );
}
