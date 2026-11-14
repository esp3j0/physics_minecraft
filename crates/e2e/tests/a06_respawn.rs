//! REQ-012 A06 R1(#25):玩家回出生点的场景判据(放行判据 A06-4/5/6)。
//!
//! 口径:所有场景走**真实网络栈**(TCP 客户端 + 驱动 + Session,a03_agents_net
//! 同款装配)。挖穿 = 对地面格发 `SetBlock AIR`(经网络命令,与玩家挖除同
//! 路径);主体掉到虚空移除深度(y < WORLD_Y_MIN − 16)内核移除,驱动在
//! 出生点重生并自动附身。
//!
//! - **A06-4(=T2)**:同一连接连续掉 3 次,每次都重生并收到
//!   `Possessed(Some(新 ID))`;`player_respawned` = 3、`player_respawn_failed`
//!   = 0;新身体的意图能驱动它走。对照:AI 主体(source = None)掉虚空只
//!   移除不重生。
//! - **A06-5(=T3)**:恢复清扫**真的重启**(带玩家身体写检查点 → 关停 →
//!   同一存储起新驱动 → 第一拍清扫移除 → 再跑 60 拍),`player_respawned`
//!   = 0、`bodies_removed_on_recovery` ≥ 1;断线移除不重生。R3(§6 修订)
//!   起"出生点不可用"改为**等区块装载 + 安全落点**:走远掉出世界 → 观察
//!   者移回出生点 → chunk 重载 → 重生(E1);洞在出生点正下方 → 换列落点
//!   (E2);600 拍超时失败口径在驱动层单测(app D2)。
//! - **A06-6(=T5)**:含一次重生的会话,拼接日志**从头回放两次**,逐拍
//!   比较全部主体状态逐位相同;并与在线会话末态逐位相同。
//!
//! **等待循环一律有界**(R1 §4):收帧按条数封顶,超限 `panic!` 并写明
//! 在等什么;任何回执不来都在 60 s 内变红。

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
use storage::traits::CheckpointStore as _;
use storage::worker::StoreWorker;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{BlockPos, DimensionId, EntityId, WorldId};
use voxel::{BlockRegistry, SimpleRegistry};

/// Superflat classic 地面顶面 world y(a03 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;
/// 软窗(a03 同款)。
const SOFT: Duration = Duration::from_millis(50);
/// 收帧上限(条):一轮"等重生"窗口最多消费的下帧数。超限 panic(写明
/// 在等什么)——挂死不算红(R1 §4)。
const FRAME_BUDGET: usize = 4000;
/// 等待墙钟上限(R2-1):单个等待循环从建 `Deadline` 起最多 60 s 真实
/// 时间。帧数预算在 CI 受限 runner 上换算成墙钟不可控(每帧内层
/// real_yield 的真实 sleep 被调度放大,T4 按 4000 帧实测 344 s)——墙钟
/// 上限保证"回执不来 60 s 内变红"。
const WAIT_WALL_SECS: u64 = 60;

/// 单个等待循环的墙钟上限(R2-1 设计):循环开头 `new`,每轮迭代 `check`;
/// 过期 panic 带上在等什么。
struct Deadline {
    at: std::time::Instant,
    what: &'static str,
}

impl Deadline {
    fn new(what: &'static str) -> Self {
        Self {
            #[allow(clippy::disallowed_methods)] // e2e 测试墙钟上限(R2-1 授权)
            at: std::time::Instant::now() + Duration::from_secs(WAIT_WALL_SECS),
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

fn capsule_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
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

/// 逐帧推拍并消费"非 Possessed(新)"的杂帧,直到收到 `Possessed(Some(x))`
/// 且 x ≠ old(重生+自动附身完成)或预算烧完(panic 写明在等什么)。
/// 返回 (新身体, 期间记录的 removed 实体, 期间记录的 Spawned 实体)。
/// `expect_removed` = 是否要求途中看到 old 出现在 removed(掉洞线)。
async fn wait_respawn_possessed(
    a: &mut TestClient,
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    old: EntityId,
) -> (EntityId, bool, Vec<EntityId>) {
    let mut removed_seen = false;
    let mut spawned: Vec<EntityId> = Vec::new();
    let mut frames = 0usize;
    let deadline = Deadline::new("虚空移除→重生→附身回执(Possessed(Some(≠旧身体)))");
    loop {
        deadline.check();
        if frames >= FRAME_BUDGET {
            panic!(
                "等重生自动附身超预算:{FRAME_BUDGET} 帧内未见 Possessed(Some(≠{old:?}));\
                 removed_seen={removed_seen} spawned={spawned:?}(在等:虚空移除→重生→附身回执)"
            );
        }
        frames += 1;
        advance_ticks(1).await;
        // 收干本拍可用帧(有界:每次 advance 后最多再收 32 条;收不到就回
        // 主循环推下一拍——推进拍数本身就是时间上限)。
        for _ in 0..32 {
            real_yield().await;
            match next_frame(rx).await {
                Some(S2C::AgentStates { removed, .. }) => {
                    if removed.contains(&old) {
                        removed_seen = true;
                    }
                }
                Some(S2C::Spawned { entity }) => spawned.push(entity),
                Some(S2C::Possessed { entity: Some(x) }) if x != old => {
                    return (x, removed_seen, spawned);
                }
                Some(_) => {}
                None => break,
            }
        }
        let _ = a; // 保留参数形状(未来需要补发意图时用)
    }
}

// ---------------------------------------------------------------------------
// chunk 流装卸帧等待(R3:chunk 流装卸证据的辅助)
// ---------------------------------------------------------------------------

/// 排干后等某 chunk 的 `ChunkUnload` 帧(客户端流侧卸载证据;泵的卸载请求
/// 与服务端 `apply_chunk_unload` 同一驱动路径)。
async fn wait_chunk_unload(rx: &mut mpsc::UnboundedReceiver<S2C>, pos: types::ChunkPos) {
    let deadline = Deadline::new("出生点 chunk 的 ChunkUnload 帧(客户端流侧卸载证据)");
    let mut frames = 0usize;
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!("等 ChunkUnload({pos:?}) 超预算(在等:客户端流卸载出生点 chunk)");
        }
        advance_ticks(1).await;
        for _ in 0..32 {
            real_yield().await;
            match next_frame(rx).await {
                Some(S2C::ChunkUnload { pos: p }) if p == pos => return,
                Some(_) => {}
                None => break,
            }
        }
    }
}

/// 排干后等某 chunk 的 `ChunkData` 帧(重生等待期间观察者被移回出生点、
/// chunk 重载的结构证据;只在驱动 `apply_chunk_load` 之后由泵发出)。
async fn wait_chunk_reload(rx: &mut mpsc::UnboundedReceiver<S2C>, pos: types::ChunkPos) {
    let deadline = Deadline::new("出生点 chunk 重载(ChunkData 重到)");
    let mut frames = 0usize;
    loop {
        deadline.check();
        frames += 1;
        if frames >= FRAME_BUDGET {
            panic!("等 ChunkData({pos:?}) 超预算(在等:重生等待把观察者移回出生点后 chunk 重载)");
        }
        advance_ticks(1).await;
        for _ in 0..32 {
            real_yield().await;
            match next_frame(rx).await {
                Some(S2C::ChunkData { pos: p, .. }) if p == pos => return,
                Some(_) => {}
                None => break,
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 迷你客户端(a03_agents_net 同款)
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

/// Hello + Subscribe,返回 player_id。
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

/// SpawnAgent → Spawned(预算内无回执 = panic)。
async fn spawn_agent_expect(
    c: &mut TestClient,
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    position: Vec3,
    yaw: f32,
) -> EntityId {
    while rx.try_recv().is_ok() {}
    c.send(&C2S::SpawnAgent {
        body: "capsule".to_string(),
        position,
        yaw,
    })
    .await
    .expect("SpawnAgent 写入");
    for _ in 0..300 {
        match next_frame(rx).await {
            Some(S2C::Spawned { entity }) => return entity,
            Some(_) => continue,
            None => real_yield().await,
        }
    }
    panic!("生成回执未到")
}

/// Possess → Possessed(Some)(预算内无回执 = panic)。
async fn possess_expect(
    c: &mut TestClient,
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    entity: EntityId,
) {
    while rx.try_recv().is_ok() {}
    c.send(&C2S::Possess { entity })
        .await
        .expect("Possess 写入");
    for _ in 0..300 {
        match next_frame(rx).await {
            Some(S2C::Possessed { entity: Some(x) }) if x == entity => return,
            Some(_) => continue,
            None => real_yield().await,
        }
    }
    panic!("附身回执未到")
}

/// SetBlock 经网络命令(与玩家挖除同路径)。
async fn set_block(c: &mut TestClient, pos: BlockPos, block: types::BlockId) {
    use messaging::SimCommand;
    c.send(&C2S::Command(SimCommand::SetBlock { pos, block }))
        .await
        .expect("SetBlock 写入");
}

/// 排干后等下一个 AgentStates(增量/全量),取该实体的最新根位姿。
async fn agent_pose_drained(rx: &mut mpsc::UnboundedReceiver<S2C>, id: EntityId) -> Option<Vec3> {
    while rx.try_recv().is_ok() {}
    let mut last = None;
    for _ in 0..200 {
        // start_paused 虚拟时钟的 timeout 不耗真实时间,服务器任务要在
        // 真实 CPU 窗口里才能投帧(a03 系 real_yield 同款);无 real_yield
        // 时 CI 受限 runner 上等待窗口瞬时烧完,一次帧都等不到。200 帧 ×
        // 1 ms 真实 sleep = 单次 drained 至少 200 ms 真实让出窗。
        real_yield().await;
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

// ---------------------------------------------------------------------------
// 装配(a03_agents_net 同款)
// ---------------------------------------------------------------------------

/// 测试用检查点存储(全部 blob 记进共享 Vec;a03_session_determinism 同款)。
type CheckpointRecords = Arc<std::sync::Mutex<Vec<(types::Tick, Vec<u8>)>>>;

#[derive(Default, Clone)]
struct RecordingCheckpointStore {
    records: CheckpointRecords,
}

impl RecordingCheckpointStore {
    fn read_all(&self) -> Vec<(types::Tick, Vec<u8>)> {
        self.records.lock().expect("锁不毒").clone()
    }
}

impl storage::traits::CheckpointStore for RecordingCheckpointStore {
    fn append(
        &mut self,
        at: types::Tick,
        blob: &[u8],
    ) -> Result<storage::traits::CheckpointId, storage::traits::StoreError> {
        let mut records = self.records.lock().expect("锁不毒");
        records.push((at, blob.to_vec()));
        Ok(storage::traits::CheckpointId(records.len() as u64))
    }

    fn read_latest(&self) -> Result<Option<(types::Tick, Vec<u8>)>, storage::traits::StoreError> {
        Ok(self.records.lock().expect("锁不毒").last().cloned())
    }

    fn read(
        &self,
        id: storage::traits::CheckpointId,
    ) -> Result<Option<(types::Tick, Vec<u8>)>, storage::traits::StoreError> {
        Ok(self
            .records
            .lock()
            .expect("锁不毒")
            .iter()
            .find(|(i, _)| storage::traits::CheckpointId(i.0 + 1) == id)
            .cloned())
    }
}

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    /// Some = 待 join;显式 shutdown 时 take(Drop 类型禁止 move 字段,
    /// take 走 Option::replace 语义合法)。
    driver: Option<tokio::task::JoinHandle<DriverOutcome>>,
    addr: SocketAddr,
    checkpoints: RecordingCheckpointStore,
}

/// 兜底关停(R2-1,a06_6 挂死根因):测试 body panic(等待判据红)时,
/// 测试任务 unwind,**没人发 Shutdown**——驱动任务在 start_paused 下
/// `ticker.tick()` auto-advance 全速空转,每 `checkpoint_interval_ticks`
/// 拍写一次 checkpoint(bincode + lz4 压缩,栈证据:
/// `write_checkpoint → encode_checkpoint → lz4_flex compress`),测试
/// 进程永不退出(实测 25 min 不结束,CPU 满核)。Drop 先 try_send 关停,
/// 驱动收尾(终末 checkpoint + flush + shutdown)照常走完;JoinHandle
/// 留在原地被 drop(detach)不阻塞 unwind。
impl Drop for Harness {
    fn drop(&mut self) {
        let _ = self.tx.try_send(DriverCommand::Shutdown);
    }
}

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

async fn harness(config: Config) -> Harness {
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
    let (provider, new_sim, bodies) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry())
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
            hub: hub.clone(),
            cadence,
        }),
        spawn_next_seq: 0,
        bodies,
        startup_controllers: Vec::new(),
    }));
    Harness {
        tx,
        driver: Some(driver),
        addr,
        checkpoints,
    }
}

fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

async fn shutdown(mut h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    let driver = h.driver.take().expect("驱动句柄只在 shutdown 取走一次");
    driver.await.expect("驱动不 panic")
}

/// DriverOutcome 的 player_respawned 计数(测试断言用)。
fn respawned(outcome: &DriverOutcome) -> u64 {
    outcome.agent.player_respawned
}

fn respawn_failed(outcome: &DriverOutcome) -> u64 {
    outcome.agent.player_respawn_failed
}

// ---------------------------------------------------------------------------
// A06-4(=T2):同一连接连续掉 3 次,每次都重生并自动附身;AI 主体对照只移除
// ---------------------------------------------------------------------------

/// 挖穿柱(x=24,z∈{8,9},y 全深 AIR;与出生点隔开,塌落证据不与出生点重叠)。
async fn dig_shaft(a: &mut TestClient) {
    let top_cell = (GROUND_TOP_Y - 1.0).floor() as i32;
    for z in [8, 9] {
        for y in (types::WORLD_Y_MIN..=top_cell).rev() {
            set_block(a, BlockPos::new(24, y, z), types::BlockId::AIR).await;
        }
    }
}

/// 走进洞:逐拍发意图 (0, −4.3) 共 50 拍(足够走出边沿并悬空下坠)。
/// "是否掉洞"由 [`wait_respawn_possessed`] 的结构判据承担(旧身体的
/// removed 帧 = 虚空移除发生;在线帧采样不做判定依据——CI 受限 runner 上
/// 帧投递时序与本地不可比,R1/R2 两轮 CI 实证)。
async fn walk_into_shaft(a: &mut TestClient, body: EntityId) {
    for _ in 0..50 {
        a.send(&C2S::AgentIntent {
            entity: body,
            move_xz: (0.0, -4.3),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("走向洞");
        advance_ticks(1).await;
    }
}

/// 场景(T2,R1 §4):同一连接**连续掉 3 次**——第一次在出生点生成附身,
/// 走进洞掉虚空 → 重生 + 自动附身;新身体再次走进洞(洞已在出生点旁,
/// 第二、三次生成后直接发意图走洞)→ 再重生;第三次同款。每次都收到
/// `Possessed(Some(新 ID))`;关停计数 `player_respawned` = 3、
/// `player_respawn_failed` = 0。变红实现:P1 的全局计数器(第二次重生
/// 第一拍计数到 3 直接判失败)、虚空移除不重生、重生后不自动附身。
#[tokio::test(start_paused = true)]
async fn a06_4_respawn_three_times_same_connection() {
    let h = harness(base_config()).await;
    let (mut a, mut rxa) = client(h.addr).await;
    tokio::task::yield_now().await;
    join(&mut a, &mut rxa, 4).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(24.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;

    // 洞先挖好(出生点 (24.5, ·, 12.5) 旁 z=8/9 柱;重生点完好)。
    dig_shaft(&mut a).await;
    advance_ticks(10).await;

    let spawn_pos = Vec3::new(24.5, GROUND_TOP_Y + 1.2, 12.5);
    let mut body = spawn_agent_expect(&mut a, &mut rxa, spawn_pos, 0.0).await;
    advance_ticks(30).await;
    possess_expect(&mut a, &mut rxa, body).await;
    // 落稳。
    for _ in 0..60 {
        advance_ticks(1).await;
        let _ = agent_pose_drained(&mut rxa, body).await;
    }

    let mut removed_first = false;
    for round in 1..=3 {
        // 走进洞 + 掉落 + 重生 + 自动附身(有界等待;第二轮起身体就出生在
        // 出生点,直接发意图走洞)。
        walk_into_shaft(&mut a, body).await;
        let (new_body, removed_seen, spawned) =
            wait_respawn_possessed(&mut a, &mut rxa, body).await;
        assert_ne!(new_body, body, "第 {round} 次:实体 ID 会变(§6)");
        if round == 1 {
            removed_first = removed_seen;
            assert!(
                spawned.contains(&new_body),
                "重生 Spawned 回执按顺序到达(实得 {spawned:?})"
            );
        }
        eprintln!("A06-4 T2 第 {round} 次重生:{body:?} → {new_body:?}");
        body = new_body;
        // 落稳后再进下一轮。
        for _ in 0..30 {
            advance_ticks(1).await;
            let _ = agent_pose_drained(&mut rxa, body).await;
        }
    }
    assert!(
        removed_first,
        "第 1 次掉洞:旧身体应出现在 AgentStates.removed"
    );

    // 关停取计数。
    drop(a);
    let outcome = shutdown(h).await;
    assert_eq!(
        respawned(&outcome),
        3,
        "player_respawned = 3(实得 {})",
        outcome.agent.player_respawned
    );
    assert_eq!(
        respawn_failed(&outcome),
        0,
        "player_respawn_failed = 0(实得 {})",
        outcome.agent.player_respawn_failed
    );
}

/// 对照线:AI 主体(source = None)掉虚空只移除,不重生(§6 表)。
/// 网络路径的 SpawnAgent 必带连接章(source = Some)——AI 主体只能走本地
/// 受信通道 `DriverCommand::Send`(SimCommand::SpawnAgent,SCHEDULER 语义
/// = source None;a03_session_determinism 的服务端主体同款)。装配同
/// harness 但不走网络面:驱动 commands 通道直注。
#[tokio::test(start_paused = true)]
async fn a06_4_ai_agent_void_removed_no_respawn() {
    let config = base_config();
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
    let session = Session::new(config.session_config());
    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    let (provider, new_sim, bodies) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry())
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
        bodies,
        startup_controllers: Vec::new(),
    }));

    // 布景:AI 主体在 (8.5, 顶面+1.2, 8.5) 生成(SCHEDULER 盖章 = source
    // None),激活观察者放它附近(走网络 MoveObserver 一样要连接——这里用
    // DriverCommand::MoveObserver 本地载体)。
    let entity = types::EntityId::from_parts(0, 900_001);
    let hash = capsule_registry().hash_of("capsule").expect("capsule");
    tx.send(DriverCommand::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver 载体");
    tx.send(DriverCommand::Send {
        shard: types::ShardId::new(
            WorldId(0),
            DimensionId(0),
            types::BlockPos::new(8, types::WORLD_Y_MIN + 1, 8)
                .chunk_of()
                .shard_of(),
        ),
        command: messaging::SimCommand::SpawnAgent {
            entity,
            body: hash,
            state: types::EntityState {
                translation: Vec3::new(8.5, GROUND_TOP_Y + 1.2, 8.5),
                ..types::EntityState::default()
            },
        },
    })
    .await
    .expect("AI 主体生成载体");
    advance_ticks(60).await;

    // 挖穿脚下(本地 SimCommand 同载体)。
    let top_cell = (GROUND_TOP_Y - 1.0).floor() as i32;
    for y in (types::WORLD_Y_MIN..=top_cell).rev() {
        tx.send(DriverCommand::Send {
            shard: types::ShardId::new(
                WorldId(0),
                DimensionId(0),
                types::BlockPos::new(8, y, 8).chunk_of().shard_of(),
            ),
            command: messaging::SimCommand::SetBlock {
                pos: BlockPos::new(8, y, 8),
                block: types::BlockId::AIR,
            },
        })
        .await
        .expect("挖洞载体");
    }
    // 等掉落 + 虚空移除(~120 拍裕量)。
    advance_ticks(240).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome = driver.await.expect("驱动不 panic");
    // AI 主体已移除(无重生计数);对照断言:player_respawned = 0。
    assert_eq!(respawned(&outcome), 0, "AI 主体(source = None)掉虚空不重生");
}

// ---------------------------------------------------------------------------
// A06-5:不重生的情形
// ---------------------------------------------------------------------------

/// 断线移除:玩家附身身体后断线 → 身体移除(bodies_removed_on_disconnect),
/// 不重生(player_respawned = 0)。
#[tokio::test(start_paused = true)]
async fn a06_5_disconnect_no_respawn() {
    let h = harness(base_config()).await;
    let (mut a, mut rxa) = client(h.addr).await;
    tokio::task::yield_now().await;
    join(&mut a, &mut rxa, 4).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;
    let body = spawn_agent_expect(
        &mut a,
        &mut rxa,
        Vec3::new(8.5, GROUND_TOP_Y + 1.2, 8.5),
        0.0,
    )
    .await;
    advance_ticks(30).await;
    possess_expect(&mut a, &mut rxa, body).await;
    advance_ticks(10).await;
    // 断线。
    a.bye().await;
    advance_ticks(60).await;
    let outcome = shutdown(h).await;
    assert_eq!(respawned(&outcome), 0, "断线移除不重生");
    assert!(
        outcome.agent.bodies_removed_on_disconnect >= 1,
        "断线移除计数(实得 {})",
        outcome.agent.bodies_removed_on_disconnect
    );
}

/// T3(R1 §4):恢复清扫**真的重启**——带玩家身体写检查点 → 关停 → 用
/// 同一存储起新驱动 → 第一拍清扫移除 → 再跑 60 拍,`player_respawned` = 0、
/// `bodies_removed_on_recovery` ≥ 1。变红实现:断线移除也重生(A06-5,
/// 放行变异 8——清扫路径等同"玩家断线后的身体",若清扫调重生这条先红)。
///
/// 装配(a03_10b 恢复线同款):Session 层造"玩家身体活着"的快照落检查点,
/// 新驱动 `run()` 从快照恢复(bootstrap 语义 = 所有玩家同时断线)。
#[tokio::test(start_paused = true)]
async fn a06_5_recovery_sweep_no_respawn() {
    use shard::SchedulerConfig;
    use sim_api::session::SessionConfig;
    use types::{ShardId, ShardPos, WorldId};

    let bodies = capsule_registry();
    let hash = bodies.hash_of("capsule").expect("capsule 已装");
    let shard = ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0));

    // —— 第一段:造"玩家身体活着"的会话,写检查点进 RecordingCheckpointStore
    //    (write_checkpoint 同款 bundle 编码;玩家身体 source = Some(1))——
    let checkpoints_store = RecordingCheckpointStore::default();
    let player_body = EntityId::from_parts(0, 1);
    let snap_tick;
    {
        let mut session = Session::new(SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        });
        session.register(physics_sim::ShardSim::new_with_bodies(
            physics_sim::ShardSimConfig::new(shard),
            Box::new(SimpleRegistry::builtin()),
            bodies.clone(),
        ));
        // 出生点 chunk(0,0)装载(平板一格地面;GROUND_TOP_Y 不适用——
        // 这是手工平板,y=64 顶面,即 GROUND_TOP_Y 与手工世界无关,直接用
        // 已知顶面)。
        let mut chunk = voxel::Chunk::air();
        let stone = types::BlockId(1);
        for x in 0..voxel::CHUNK_X {
            for z in 0..voxel::CHUNK_Z {
                chunk.set(x, 64, z, stone);
            }
        }
        session.apply_chunk_load(shard, types::ChunkPos::new(0, 0), chunk);
        let entry = session
            .spawn_agent_from(
                InputSource(1),
                shard,
                player_body,
                hash,
                types::EntityState {
                    translation: Vec3::new(12.0, 65.2, 12.0),
                    ..types::EntityState::default()
                },
            )
            .expect("玩家身体生成");
        let mut log = CommandLog::new();
        log.append(entry);
        session.tick();
        // 快照 + bundle 编码(写检查点;spawn 高水位 = 已用 2)。
        let world = session.world_snapshot();
        snap_tick = world.shards.tick;
        let bundle = app::driver::CheckpointBundle {
            format_version: app::driver::CHECKPOINT_FORMAT_VERSION,
            world,
            log_bytes: log.to_bytes(),
            spawn_next_seq: 2,
            controllers: Vec::new(),
        };
        let mut store = checkpoints_store.clone();
        storage::traits::CheckpointStore::append(
            &mut store,
            snap_tick,
            &app::driver::encode_checkpoint(&bundle),
        )
        .expect("检查点写入");
    }

    // —— 第二段:同一存储起新驱动(replay 恢复 + run;observe 关闭 = 无
    //    网络面,清扫照做)——
    let resumed = {
        let latest = checkpoints_store
            .read_latest()
            .expect("读回")
            .expect("有检查点");
        let bundle = app::driver::decode_checkpoint(&latest.1).expect("checkpoint 可解");
        sim_api::replay::replay_world_with_bodies(
            &bundle.world,
            &CommandLog::new(),
            SessionConfig {
                scheduler: SchedulerConfig {
                    worker_count: 2,
                    start_tick: snap_tick,
                    ..SchedulerConfig::default()
                },
                ..SessionConfig::default()
            },
            || Box::new(SimpleRegistry::builtin()),
            || bodies.clone(),
        )
        .expect("恢复")
    };
    let config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 6000,
        ..Config::default()
    };
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(checkpoints_store.clone()),
    );
    let (provider, new_sim, bodies2) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry())
            .expect("装配失败");
    let (tx, rx) = mpsc::channel::<DriverCommand>(16);
    let driver = tokio::spawn(run(DriverArgs {
        config,
        session: resumed,
        store,
        log: CommandLog::new(),
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: None,
        spawn_next_seq: 2,
        bodies: bodies2,
        startup_controllers: Vec::new(),
    }));
    // 再跑 60 拍(清扫在第一拍)。
    advance_ticks(60).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome = driver.await.expect("驱动不 panic");
    assert_eq!(
        outcome.agent.bodies_removed_on_recovery, 1,
        "第一拍清扫移除玩家身体(实得 {})",
        outcome.agent.bodies_removed_on_recovery
    );
    assert_eq!(
        respawned(&outcome),
        0,
        "恢复清扫不重生(player_respawned = 0,实得 {})",
        outcome.agent.player_respawned
    );
    assert_eq!(respawn_failed(&outcome), 0);
}

// ---------------------------------------------------------------------------
// A06-6(=T5):确定性(含一次重生的会话,日志回放两次逐位同)
// ---------------------------------------------------------------------------

/// 含一次虚空移除 + 重生的会话(T5,R1 §4):
/// ① 重生动作在日志里——日志含**两条** `SimCommand::SpawnAgent`(原始
///    生成 + 重生,与网络请求同一条路径),重生那条的实体 = 回执的新身体;
/// ② **日志回放两次,逐拍比较全部主体状态,逐位相同**(放行 A06-6 原文);
/// ③ 回放末态与**在线会话末态**(终末 checkpoint 的世界快照)逐位相同。
///    变红实现:重生不写日志(放行变异 11——回放线没有重生动作,末态少
///    一次重生,与在线末态不一致)。
///
/// 回放口径:取**第一个检查点**的世界快照 + **拼接日志**(窗内 `>=` 回放),
/// tick 到在线末拍(关停时的 ticks_run),逐拍记录 `agent_states()`。
#[tokio::test(start_paused = true)]
async fn a06_6_replay_twice_bitwise_identical() {
    use shard::SchedulerConfig;
    use sim_api::session::SessionConfig;

    let mut config = base_config();
    config.checkpoint_interval_ticks = 30;
    let mut h = harness(config.clone()).await;
    let (mut a, mut rxa) = client(h.addr).await;
    tokio::task::yield_now().await;
    join(&mut a, &mut rxa, 4).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(24.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;
    let spawn_pos = Vec3::new(24.5, GROUND_TOP_Y + 1.2, 12.5);
    let body = spawn_agent_expect(&mut a, &mut rxa, spawn_pos, 0.0).await;
    advance_ticks(30).await;
    possess_expect(&mut a, &mut rxa, body).await;
    for _ in 0..60 {
        advance_ticks(1).await;
        let _ = agent_pose_drained(&mut rxa, body).await;
    }
    // 挖穿旁柱 + 走进洞 + 掉落 + 重生(T2 同款;出生点不在挖的柱上,
    // 重生点完好 → 恰一次重生)。
    dig_shaft(&mut a).await;
    advance_ticks(10).await;
    walk_into_shaft(&mut a, body).await;
    // 等虚空移除 + 重生(有界;Possessed(Some(非旧身体)) 为终点)。
    let (new_body, _, _) = wait_respawn_possessed(&mut a, &mut rxa, body).await;
    advance_ticks(60).await;
    drop(a);
    // 不解构 Harness(其 Drop 是 panic 路径的兜底关停);显式关停取计数。
    let checkpoints = h.checkpoints.clone();
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    let driver = h.driver.take().expect("驱动句柄只在 shutdown 取走一次");
    let outcome = driver.await.expect("驱动不 panic");
    assert_eq!(respawned(&outcome), 1, "重生恰好一次");
    let online_ticks = outcome.ticks_run;

    // 拼接日志(检查点窗拼接;a03_10 同法)+ 结构断言(①)。
    let checkpoints = checkpoints.read_all();
    assert!(!checkpoints.is_empty(), "checkpoint 已写");
    let log = stitched_log(&checkpoints);
    let mut spawn_agent_count = 0usize;
    let mut respawn_spawn_seen = false;
    for entry in log.entries() {
        if let messaging::SimCommand::SpawnAgent { entity, .. } = entry.command {
            spawn_agent_count += 1;
            if entity == new_body {
                respawn_spawn_seen = true;
            }
        }
    }
    assert_eq!(
        spawn_agent_count, 2,
        "日志含两条 SpawnAgent(原始生成 + 重生;同一条校验/日志路径)"
    );
    assert!(
        respawn_spawn_seen,
        "重生那条 SpawnAgent 的实体 = 回执的新身体 {new_body:?}"
    );

    // —— ② 回放两次,逐拍比较全部主体状态(逐位)——
    // 第一个检查点的世界快照(起点)+ 拼接日志(`>=` 窗口回放)。
    let (_, first_blob) = &checkpoints[0];
    let first_bundle = app::driver::decode_checkpoint(first_blob).expect("checkpoint 可解");
    let bodies = capsule_registry();
    let mk_replay = || -> Session {
        sim_api::replay::replay_world_with_bodies(
            &first_bundle.world,
            &log,
            SessionConfig {
                scheduler: SchedulerConfig {
                    worker_count: 2,
                    start_tick: first_bundle.world.shards.tick,
                    ..SchedulerConfig::default()
                },
                ..SessionConfig::default()
            },
            || Box::new(SimpleRegistry::builtin()),
            || bodies.clone(),
        )
        .expect("回放会话重建")
    };
    let end_tick = snap_tick_of(&checkpoints, online_ticks);
    let sample = |session: &mut Session| -> Vec<u8> {
        // 逐拍主体状态序列化(EntityId 升序 + BodyHash + 全连杆 EntityState
        // 的 bincode;EntityState 含位姿/速度全部字段 = 逐位比较)。
        let mut buf = Vec::new();
        for (id, hash, poses) in session.agent_states() {
            buf.extend_from_slice(&id.to_bits().to_le_bytes());
            buf.extend_from_slice(&hash.0);
            for pose in &poses {
                let bytes = bincode::serde::encode_to_vec(pose, bincode::config::standard())
                    .expect("EntityState 序列化");
                buf.extend_from_slice(&bytes);
            }
        }
        buf
    };
    let mut replay_samples = Vec::new();
    for _rep in 0..2 {
        let mut session = mk_replay();
        let mut reps = Vec::new();
        while session.tick_now().0 < end_tick.0 {
            session.tick();
            reps.push(sample(&mut session));
        }
        replay_samples.push(reps);
        session.shutdown();
    }
    assert_eq!(
        replay_samples[0], replay_samples[1],
        "回放两次:逐拍全部主体状态逐位相同"
    );

    // —— ③ 回放末态与在线会话末态逐位相同——
    // 在线末态 = 终末 checkpoint 的世界快照,恢复出会话采样主体状态。
    let (_, last_blob) = checkpoints.last().expect("终末 checkpoint");
    let last_bundle = app::driver::decode_checkpoint(last_blob).expect("终末可解");
    let mut online = sim_api::replay::replay_world_with_bodies(
        &last_bundle.world,
        &CommandLog::new(),
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                start_tick: last_bundle.world.shards.tick,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        },
        || Box::new(SimpleRegistry::builtin()),
        || bodies.clone(),
    )
    .expect("在线末态恢复");
    let online_final = sample(&mut online);
    let replay_final = {
        let mut session = mk_replay();
        while session.tick_now().0 < end_tick.0 {
            session.tick();
        }
        let s = sample(&mut session);
        session.shutdown();
        s
    };
    online.shutdown();
    assert_eq!(
        replay_final, online_final,
        "回放末态与在线末态逐位相同(重生动作经日志自再生)"
    );
    assert!(!replay_final.is_empty(), "末态有主体(重生的新身体活着)");
}

/// 在线会话关停时的末拍(终末 checkpoint 的 tick;ticks_run 是相对值,
/// 终末 bundle 的 tick = 下一待执行拍 = 回放终点)。
fn snap_tick_of(checkpoints: &[(types::Tick, Vec<u8>)], _online_ticks: u64) -> types::Tick {
    let (_, blob) = checkpoints.last().expect("终末 checkpoint");
    let bundle = app::driver::decode_checkpoint(blob).expect("终末可解");
    bundle.world.shards.tick
}

/// 日志窗拼接(a03_session_determinism 同法)。
fn stitched_log(checkpoints: &[(types::Tick, Vec<u8>)]) -> CommandLog {
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

// ---------------------------------------------------------------------------
// A06 R3(§6 修订):E1 等区块装载 / E2 安全落点
// ---------------------------------------------------------------------------

/// E1(§6"等区块装载";替代旧 T4"出生点不可用"——返修单第 3 节授权
/// "按新条文改成 E1"):走远后出生点 chunk 卸载,掉出世界能回来。
///
/// 流程:view=1 join → 出生点 (8.5,·,8.5) 生成附身 → 意图 +x 走 ~100 m
/// (1400 拍)→ 停稳 → **对照线:先断言出生点 chunk 已卸载**(客户端流的
/// `ChunkUnload(0,0)` 帧;泵的卸载请求与驱动 `apply_chunk_unload` 同一
/// 路径)→ 洞挖在停点 → 身体掉虚空 → 驱动把观察者移回出生点(等待期间
/// 该连接的 `MoveObserver` 被忽略)→ chunk 重载(收到 `ChunkData(0,0)`)
/// → 安全落点重生 → 自动附身 `Possessed(Some)`、新身体在出生点附近。
#[tokio::test(start_paused = true)]
async fn a06_r3_e1_walk_far_chunk_unloaded_respawn() {
    let h = harness(base_config()).await;
    let (mut a, mut rxa) = client(h.addr).await;
    tokio::task::yield_now().await;
    join(&mut a, &mut rxa, 1).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;
    let spawn_pos = Vec3::new(8.5, GROUND_TOP_Y + 1.2, 8.5);
    let body = spawn_agent_expect(&mut a, &mut rxa, spawn_pos, 0.0).await;
    advance_ticks(30).await;
    possess_expect(&mut a, &mut rxa, body).await;
    for _ in 0..60 {
        advance_ticks(1).await;
        let _ = agent_pose_drained(&mut rxa, body).await;
    }

    // 走远:逐拍发意图 +x,1400 拍(4.3 m/s × 1400/60 ≈ 100 m)。
    for _ in 0..1400 {
        a.send(&C2S::AgentIntent {
            entity: body,
            move_xz: (4.3, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("走远");
        advance_ticks(1).await;
    }
    // 停住(刹车 ~120 拍)+ 留数据卸载决策与冷却拍数(旧 T4 同款 600)。
    a.send(&C2S::AgentIntent {
        entity: body,
        move_xz: (0.0, 0.0),
        yaw: 0.0,
        jump: false,
    })
    .await
    .expect("停住");
    advance_ticks(120).await;
    advance_ticks(600).await;

    // 对照线(先断言):出生点 chunk 已卸载(客户端流 ChunkUnload 帧)。
    wait_chunk_unload(&mut rxa, types::ChunkPos::new(0, 0)).await;

    // 读身体实际停点;断言确实走出了 48 m 数据卸载圈。
    let stopped = agent_pose_drained(&mut rxa, body)
        .await
        .unwrap_or_else(|| panic!("读身体停点失败(在等:身体停稳后的 AgentStates 帧)"));
    let walked = stopped.x - spawn_pos.x;
    eprintln!(
        "E1:身体停点 ({:.2}, {:.2}),距出生点 {walked:.1} m",
        stopped.x, stopped.z
    );
    assert!(
        walked > 48.0,
        "身体应走出数据卸载圈 48 m(实走 {walked:.1} m)"
    );

    // 洞挖在停点(x ± 1、z 两列,世界底全深)→ 身体悬空掉虚空。
    let bx = stopped.x.floor() as i32;
    let bz = stopped.z.floor() as i32;
    let top_cell = (GROUND_TOP_Y - 1.0).floor() as i32;
    for x in (bx - 1)..=(bx + 1) {
        for z in bz..=(bz + 1) {
            for y in (types::WORLD_Y_MIN..=top_cell).rev() {
                set_block(&mut a, BlockPos::new(x, y, z), types::BlockId::AIR).await;
            }
        }
    }

    // 等虚空移除 → 等区块装载 → 重生 → 自动附身(Possessed(Some(≠旧)))。
    let mut removed_seen = false;
    let mut frames = 0usize;
    let deadline = Deadline::new("虚空移除 → 等区块装载 → 重生 → Possessed(Some)");
    let new_body = loop {
        deadline.check();
        if frames >= FRAME_BUDGET {
            panic!(
                "等重生自动附身超预算:{FRAME_BUDGET} 帧;removed_seen={removed_seen} \
                 (在等:虚空移除 → 等区块装载 → 重生 → Possessed(Some(≠{body:?})))"
            );
        }
        frames += 1;
        advance_ticks(1).await;
        let mut done = None;
        for _ in 0..32 {
            real_yield().await;
            match next_frame(&mut rxa).await {
                Some(S2C::AgentStates { removed, .. }) => {
                    if removed.contains(&body) {
                        removed_seen = true;
                    }
                }
                Some(S2C::Possessed { entity: Some(x) }) if x != body => {
                    done = Some(x);
                    break;
                }
                Some(_) => {}
                None => break,
            }
        }
        if let Some(x) = done {
            break x;
        }
    };
    assert!(removed_seen, "旧身体应出现在 AgentStates.removed(虚空移除)");

    // 出生点 chunk 重载证据:ChunkData(0,0) 重到(驱动先 apply_chunk_load,
    // 泵才发 ChunkData——装载好了才可能生成)。
    wait_chunk_reload(&mut rxa, types::ChunkPos::new(0, 0)).await;

    // 新身体在出生点附近(原位置落点 = Found(point) 原样,水平差 < 2 m)。
    let root = agent_pose_drained(&mut rxa, new_body)
        .await
        .unwrap_or_else(|| panic!("读新身体位置失败(在等:新身体的 AgentStates 帧)"));
    eprintln!("E1:重生落点 ({:.2}, {:.2}, {:.2})", root.x, root.y, root.z);
    assert!(
        (root.x - spawn_pos.x).abs() <= 2.0 && (root.z - spawn_pos.z).abs() <= 2.0,
        "新身体应在出生点附近(根位置 {root:?} vs 出生点 {spawn_pos:?})"
    );

    drop(a);
    let outcome = shutdown(h).await;
    assert_eq!(
        respawned(&outcome),
        1,
        "player_respawned = 1(实得 {})",
        outcome.agent.player_respawned
    );
    assert_eq!(
        respawn_failed(&outcome),
        0,
        "player_respawn_failed = 0(实得 {})",
        outcome.agent.player_respawn_failed
    );
}

/// E2(§6"安全落点";用户人工验收复现的无限重生 bug):洞挖在出生点
/// 正下方 → 掉出世界后重生在**邻列**,之后 300 拍内没有第二次虚空移除
/// (修订前 ~124 拍一轮无限重生停不下来)。`player_respawned` = 1。
/// 变红实现:`safe_spawn_position` 恒返回 `Found(point)`(原地重生 →
/// 再掉 → 循环)。
#[tokio::test(start_paused = true)]
async fn a06_r3_e2_hole_under_spawn_respawn_neighbor_no_loop() {
    let h = harness(base_config()).await;
    let (mut a, mut rxa) = client(h.addr).await;
    tokio::task::yield_now().await;
    join(&mut a, &mut rxa, 4).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;
    let spawn_pos = Vec3::new(8.5, GROUND_TOP_Y + 1.2, 8.5);
    let body = spawn_agent_expect(&mut a, &mut rxa, spawn_pos, 0.0).await;
    advance_ticks(30).await;
    possess_expect(&mut a, &mut rxa, body).await;
    for _ in 0..60 {
        advance_ticks(1).await;
        let _ = agent_pose_drained(&mut rxa, body).await;
    }

    // 洞挖在出生点正下方(出生点列 (8,·,8) 世界底全深;观察者就在出生点,
    // chunk 全程装载)。
    let top_cell = (GROUND_TOP_Y - 1.0).floor() as i32;
    for y in (types::WORLD_Y_MIN..=top_cell).rev() {
        set_block(&mut a, BlockPos::new(8, y, 8), types::BlockId::AIR).await;
    }

    // 掉落 → 虚空移除 → 安全落点重生 → 自动附身。
    let mut removed_seen = false;
    let mut frames = 0usize;
    let deadline = Deadline::new("虚空移除 → 安全落点重生 → Possessed(Some)");
    let new_body = loop {
        deadline.check();
        if frames >= FRAME_BUDGET {
            panic!(
                "等重生超预算:{FRAME_BUDGET} 帧;removed_seen={removed_seen} \
                 (在等:虚空移除 → 安全落点重生 → Possessed(Some(≠{body:?})))"
            );
        }
        frames += 1;
        advance_ticks(1).await;
        let mut done = None;
        for _ in 0..32 {
            real_yield().await;
            match next_frame(&mut rxa).await {
                Some(S2C::AgentStates { removed, .. }) => {
                    if removed.contains(&body) {
                        removed_seen = true;
                    }
                }
                Some(S2C::Possessed { entity: Some(x) }) if x != body => {
                    done = Some(x);
                    break;
                }
                Some(_) => {}
                None => break,
            }
        }
        if let Some(x) = done {
            break x;
        }
    };
    assert!(removed_seen, "旧身体应出现在 AgentStates.removed(虚空移除)");

    // 新身体落在邻列(第 1 环第一列 (7,7):cx=8、cz=8;落点 = 顶格 -61
    // + 1 + 1.0 = -59.0,水平 (7.5, 7.5))。
    let root = agent_pose_drained(&mut rxa, new_body)
        .await
        .unwrap_or_else(|| panic!("读新身体位置失败(在等:新身体的 AgentStates 帧)"));
    eprintln!("E2:重生落点 ({:.2}, {:.2}, {:.2})", root.x, root.y, root.z);
    assert!(
        (root.x - 7.5).abs() <= 0.3 && (root.z - 7.5).abs() <= 0.3,
        "重生应落在邻列 (7.5, ·, 7.5)(实得 {root:?};出生点列已挖穿,不得原地重生)"
    );

    // 300 拍内没有第二次虚空移除(修订前的循环 ≈124 拍一轮)。
    for i in 0..300 {
        advance_ticks(1).await;
        for _ in 0..32 {
            real_yield().await;
            match next_frame(&mut rxa).await {
                Some(S2C::AgentStates { removed, .. }) => {
                    assert!(
                        !removed.contains(&new_body),
                        "拍 {i}:新身体再次被虚空移除(重生应已跳出洞,不再循环)"
                    );
                }
                Some(_) => {}
                None => break,
            }
        }
    }

    drop(a);
    let outcome = shutdown(h).await;
    assert_eq!(
        respawned(&outcome),
        1,
        "player_respawned = 1(实得 {})",
        outcome.agent.player_respawned
    );
    assert_eq!(
        respawn_failed(&outcome),
        0,
        "player_respawn_failed = 0(实得 {})",
        outcome.agent.player_respawn_failed
    );
}
