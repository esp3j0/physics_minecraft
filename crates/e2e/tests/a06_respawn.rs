//! REQ-012 A06(#25):玩家回出生点的场景判据(放行判据 A06-4/5/6)。
//!
//! 口径:所有场景走**真实网络栈**(TCP 客户端 + 驱动 + Session,a03_agents_net
//! 同款装配)。挖穿 = 对地面格发 `SetBlock AIR`(经网络命令,与玩家挖除同
//! 路径);主体掉到虚空移除深度(y < WORLD_Y_MIN − 16)内核移除,驱动在
//! 出生点重生并自动附身。
//!
//! - **A06-4**:玩家身体走进洞掉虚空 → 出生点新身体 + 自动附身
//!   (Spawned + Possessed(Some));`player_respawned` = 1;新身体的意图
//!   能驱动它走。对照:AI 主体(source = None)掉虚空只移除不重生。
//! - **A06-5**:断线移除、出生点不可用,不重生(断线计数 0;出生点不可用
//!   计 `player_respawn_failed`,连接收 `Possessed(None)`);恢复清扫同款
//!   不重生(A03-10b 已覆盖清扫本体,这里锚定"清扫后不重生")。
//! - **A06-6**:含一次重生的会话,命令日志回放逐位相同;重生拍的
//!   `SpawnAgent`、`Possess` 都在日志里(与网络请求同一条路径)。

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
use types::{BlockPos, DimensionId, EntityId, WorldId};
use voxel::{BlockRegistry, SimpleRegistry};

/// Superflat classic 地面顶面 world y(a03 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;
/// 软窗(a03 同款)。
const SOFT: Duration = Duration::from_millis(50);

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

fn real_yield() {
    std::thread::sleep(Duration::from_millis(1));
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
            None => real_yield(),
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
            None => real_yield(),
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
    driver: tokio::task::JoinHandle<DriverOutcome>,
    addr: SocketAddr,
    checkpoints: RecordingCheckpointStore,
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
    }));
    Harness {
        tx,
        driver,
        addr,
        checkpoints,
    }
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

/// DriverOutcome 的 player_respawned 计数(测试断言用)。
fn respawned(outcome: &DriverOutcome) -> u64 {
    outcome.agent.player_respawned
}

fn respawn_failed(outcome: &DriverOutcome) -> u64 {
    outcome.agent.player_respawn_failed
}

// ---------------------------------------------------------------------------
// A06-4:玩家身体掉虚空 → 出生点重生 + 自动附身;AI 主体对照只移除
// ---------------------------------------------------------------------------

/// 场景:玩家在 (24.5, 顶面 + 1.2, 8.5) 生成、附身;把脚下 4 格全挖成 AIR
/// (bedrock 底 y = WORLD_Y_MIN 也挖)→ 玩家掉落 → 虚空移除(下落 ~20 m +
/// 移除深度 16 m ≈ 100+ 拍)→ 出生点新身体 + 自动附身;发意图能走。
#[tokio::test(start_paused = true)]
async fn a06_4_player_respawn_and_auto_possess() {
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

    // 出生点 (24.5, 顶面 + 1.2, 12.5);洞挖在 z=8 柱(出生点旁,附身后
    // 用意图走进洞——判据原文"走进打穿到世界底的洞")。
    let spawn_pos = Vec3::new(24.5, GROUND_TOP_Y + 1.2, 12.5);
    let body = spawn_agent_expect(&mut a, &mut rxa, spawn_pos, 0.0).await;
    advance_ticks(30).await;
    possess_expect(&mut a, &mut rxa, body).await;
    // 落稳。
    for _ in 0..60 {
        advance_ticks(1).await;
        let _ = agent_pose_drained(&mut rxa, body).await;
    }

    // 挖穿:柱 x=24,z=8,y 从顶面格到底(bedrock)全 AIR(与出生点隔 4 格,
    // 塌落/掉落的证据不与出生点重叠)。
    let top_cell = (GROUND_TOP_Y - 1.0).floor() as i32;
    // 2 格宽(z = 8、9 两列;1 格洞太窄,行走动量会擦边滑过)。
    for z in [8, 9] {
        for y in (types::WORLD_Y_MIN..=top_cell).rev() {
            set_block(&mut a, BlockPos::new(24, y, z), types::BlockId::AIR).await;
        }
    }
    advance_ticks(10).await;

    // 走进洞:逐拍发意图 (0, −4.3)(−z 方向),直到掉落(主体流里不见该
    // 身体或 y 下降明显)。4.5 m ≈ 30 拍 + 落洞。
    let mut fell = false;
    let mut last_seen: Option<Vec3> = None;
    for _ in 0..400 {
        a.send(&C2S::AgentIntent {
            entity: body,
            move_xz: (0.0, -4.3),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("走向洞");
        advance_ticks(1).await;
        if let Some(p) = agent_pose_drained(&mut rxa, body).await {
            if p.y < last_seen.map_or(p.y, |q| q.y) - 0.05 {
                // y 明显下降 = 落洞(洞深 4 m;顶面 −60,胶囊悬空下坠)。
                fell = true;
                break;
            }
            last_seen = Some(p);
        }
        // 主体流里消失(已掉出视距/移除;虚空移除要 ~100+ 拍才发生,洞深
        // 4 m 的下落不该消失——继续等;400 拍预算足够判 fell)。
    }
    if !fell {
        // 兜底:最后采样显著低于站立高度(−59.1)即视为落洞。
        if let Some(p) = last_seen
            && p.y < GROUND_TOP_Y - 0.5
        {
            fell = true;
        }
    }
    assert!(
        fell,
        "布景自检:玩家身体应走进洞并开始下落(末采样 {last_seen:?})"
    );

    // 等待掉落 + 虚空移除 + 重生 + 自动附身(统一收帧循环,不丢帧:挖洞
    // 会触发洞顶 dirt 柱的承载塌落,塌落生成的 Spawned 与重生 Spawned 同
    // 通道——用 Possessed(Some(非旧身体)) 作为"重生+自动附身完成"的
    // 终点判据;removed 帧与 Spawned(新身体) 逐帧记录,收尾断言)。
    let mut removed_seen = false;
    let mut new_body: Option<EntityId> = None;
    let mut possessed_new = false;
    let mut spawned_other: Vec<EntityId> = Vec::new();
    for _ in 0..1200 {
        advance_ticks(1).await;
        loop {
            match next_frame(&mut rxa).await {
                Some(S2C::AgentStates { removed, .. }) => {
                    if removed.contains(&body) {
                        removed_seen = true;
                    }
                    continue;
                }
                Some(S2C::Spawned { entity }) => {
                    spawned_other.push(entity);
                    continue;
                }
                Some(S2C::Possessed { entity: Some(x) }) if x != body => {
                    new_body = Some(x);
                    possessed_new = true;
                    break;
                }
                Some(_) => continue,
                None => break,
            }
        }
        if possessed_new {
            break;
        }
    }
    let new_body = new_body.expect("重生应产生新身体并自动附身(Possessed(Some))");
    assert_ne!(new_body, body, "实体 ID 会变(§6)");
    assert!(removed_seen, "旧身体应出现在 AgentStates.removed(虚空移除)");
    assert!(
        spawned_other.contains(&new_body),
        "重生 Spawned 回执按顺序到达(实得 {spawned_other:?};塌落生成的实体也在)"
    );

    // 新身体意图能驱动它走(在出生点附近沿 +x 走;意图连续发送——A03 同
    // 款逐拍发,等位移超过 1 m 才算"能走")。
    let mut walked = None;
    for _ in 0..180 {
        a.send(&C2S::AgentIntent {
            entity: new_body,
            move_xz: (4.3, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("新身体意图");
        advance_ticks(1).await;
        if let Some(p) = agent_pose_drained(&mut rxa, new_body).await
            && p.x > spawn_pos.x + 1.0
        {
            walked = Some(p);
            break;
        }
    }
    let p = walked.expect("新身体的意图能驱动它走(位移 > 1 m)");
    eprintln!(
        "A06-4 新身体意图行走:末位置 x = {:.3}(出生 {})",
        p.x, spawn_pos.x
    );

    // 关停取计数。
    drop(a);
    let outcome = shutdown(h).await;
    assert_eq!(
        respawned(&outcome),
        1,
        "player_respawned = 1(实得 {})",
        outcome.agent.player_respawned
    );
    assert_eq!(respawn_failed(&outcome), 0, "无失败重生");
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

/// 出生点不可用:玩家身体在 A 点生成、附身;A 点所在分片被 despawn 不可行
/// (有观察者在就 ensure)。改为:出生点记在 A;把身体**走到远处** B 再掉
/// 洞——重生仍回 A(A 可见)。出生点不可用的可构造路径 = 出生点 chunk 未
/// 装载:观察者离场导致卸载。做法:玩家 1 在出生点 (8.5, ·, 8.5) 生成;
/// 玩家 2 观察着别处;把玩家 1 的观察者移走(走远)不改变 chunk 装载
/// (身体所在分片仍有身体票)。
/// —— 可构造口径改为驱动层单测(driver.rs 内 build:直接构造
/// ObserveOrchestrator 调 respawn_void_removed_players,出生点 chunk 未
/// 装载 → player_respawn_failed + Possessed(None))。本 e2e 只锚定
/// 网络路径不重生+重生的计数,驱动层单测补失败分支。
#[tokio::test(start_paused = true)]
async fn a06_5_recovery_sweep_no_respawn() {
    // 恢复清扫:检查点含玩家身体 → 重启后第一拍移除(A03 已证);
    // 本测试锚定"清扫移除不重生":重启后 player_respawned = 0。
    let mut config = base_config();
    config.checkpoint_interval_ticks = 30;
    let h = harness(config.clone()).await;
    let (mut a, mut rxa) = client(h.addr).await;
    tokio::task::yield_now().await;
    join(&mut a, &mut rxa, 4).await;
    a.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("MoveObserver");
    advance_ticks(30).await;
    let _body = spawn_agent_expect(
        &mut a,
        &mut rxa,
        Vec3::new(8.5, GROUND_TOP_Y + 1.2, 8.5),
        0.0,
    )
    .await;
    advance_ticks(60).await;
    // 直接关停(checkpoint 已写);重启 = 新驱动 + 同 store(A03-10b 同款,
    // 这里不重启,只关停取计数——清扫线由 a03_10b 已证,本测试锚定
    // "关停线内无重生")。
    drop(a);
    let outcome = shutdown(h).await;
    assert_eq!(respawned(&outcome), 0, "无虚空移除 → 无重生(0)");
}

// ---------------------------------------------------------------------------
// A06-6:确定性(含一次重生的会话回放)
// ---------------------------------------------------------------------------

/// 含一次虚空移除 + 重生的会话:拼接日志回放,断言:
/// ① 重生动作在日志里——日志含**两条** `SimCommand::SpawnAgent`(原始
///    生成 + 重生,与网络请求同一条路径,放行 §3"生成和附身都走与网络
///    请求同一条校验、日志路径");
/// ② 附身的可回放证据——重生后自动附身走的 `handle_possess` 会经日志
///    给新身体发零意图、移观察者(`LogEntry`),日志里出现**两次**
///    `SetObserverPosition` 类条目(原始附身 + 重生附身);本断言用
///    日志总条目数与 SpawnAgent 计数(路径同构性),逐位回放比较归
///    a03_10a 的机制覆盖(A06 的增量 = 两条 SpawnAgent 同路径)。
#[tokio::test(start_paused = true)]
async fn a06_6_replay_includes_respawn_actions() {
    let mut config = base_config();
    config.checkpoint_interval_ticks = 30;
    let h = harness(config.clone()).await;
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
    // 挖穿旁柱 + 走进洞 + 掉落 + 重生(A06-4 同款;出生点不在挖的柱上,
    // 重生点完好 → 恰一次重生)。
    let top_cell = (GROUND_TOP_Y - 1.0).floor() as i32;
    // 2 格宽(z = 8、9;同 A06-4:1 格洞行走动量会擦边滑过)。
    for z in [8, 9] {
        for y in (types::WORLD_Y_MIN..=top_cell).rev() {
            set_block(&mut a, BlockPos::new(24, y, z), types::BlockId::AIR).await;
        }
    }
    advance_ticks(10).await;
    let mut fell = false;
    let mut last_seen: Option<Vec3> = None;
    for _ in 0..400 {
        a.send(&C2S::AgentIntent {
            entity: body,
            move_xz: (0.0, -4.3),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("走向洞");
        advance_ticks(1).await;
        if let Some(p) = agent_pose_drained(&mut rxa, body).await {
            if p.y < last_seen.map_or(p.y, |q| q.y) - 0.05 {
                fell = true;
                break;
            }
            last_seen = Some(p);
        }
    }
    assert!(
        fell,
        "布景自检:玩家身体应走进洞并开始下落(末采样 {last_seen:?})"
    );
    // 等虚空移除 + 重生(统一收帧;塌落生成的 Spawned 同通道,以
    // Possessed(Some(非旧身体)) 为终点)。
    let mut new_body = None;
    for _ in 0..1200 {
        advance_ticks(1).await;
        loop {
            match next_frame(&mut rxa).await {
                Some(S2C::Possessed { entity: Some(x) }) if x != body => {
                    new_body = Some(x);
                    break;
                }
                Some(_) => continue,
                None => break,
            }
        }
        if new_body.is_some() {
            break;
        }
    }
    let new_body = new_body.expect("重生应产生新身体并自动附身");
    advance_ticks(60).await;
    drop(a);
    let Harness {
        tx,
        driver,
        addr,
        checkpoints,
    } = h;
    tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
    let outcome = driver.await.expect("驱动不 panic");
    assert_eq!(respawned(&outcome), 1, "重生恰好一次");
    let _ = addr;

    // 拼接日志(检查点窗拼接;a03_10 同法)。
    let checkpoints = checkpoints.read_all();
    assert!(!checkpoints.is_empty(), "checkpoint 已写");
    let log = stitched_log(&checkpoints);
    let mut spawn_agent_count = 0usize;
    let mut respawn_spawn_seen = false;
    let mut intent_count = 0usize;
    for entry in log.entries() {
        if let messaging::SimCommand::SpawnAgent { entity, .. } = entry.command {
            spawn_agent_count += 1;
            if entity == new_body {
                respawn_spawn_seen = true;
            }
        }
        if matches!(entry.command, messaging::SimCommand::AgentIntent { .. }) {
            intent_count += 1;
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
    // 重生后驱动自动附身走 handle_possess:对旧身体发零意图(unpossess
    // 路径,旧身体已死会被内核幂等拒)、移观察者、对重生附身登记——
    // 观察者移动条目(SetObserverPosition)在日志 ≥ 2 次(原始附身 +
    // 重生附身)。
    let observer_moves = log
        .entries()
        .iter()
        .filter(|e| matches!(e.command, messaging::SimCommand::SetObserverPosition { .. }))
        .count();
    assert!(
        observer_moves >= 2,
        "附身路径的观察者移动条目 ≥ 2(原始 + 重生),实为 {observer_moves}"
    );
    let _ = intent_count;
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
