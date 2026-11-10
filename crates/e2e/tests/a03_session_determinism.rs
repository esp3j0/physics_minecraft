//! REQ-012 A03 判据 A03-10(P3 重做):确定性。
//!
//! - **10a**:真实网络栈 + 驱动跑两客户端会话(生成、附身、走过边界、
//!   一次伪造、一次"停下"、断线),取驱动写出的命令日志(checkpoint 增量
//!   窗口拼接)用 `replay` 重放,逐拍采样比较主体状态(f32 逐位)。
//!   在线侧逐拍状态取 AgentStates 下行帧(f32 原值,帧带拍号);回放侧
//!   `Session::agent_root_position` 同拍采样。
//! - **10b**:检查点在玩家身体**还活着**时落下(快照含玩家身体);恢复
//!   走 `run` 的启动清扫(计数 = 1);服务端主体续跑与不中断的那一线
//!   **逐位相同**(两线同一拍移除玩家身体后,终态 world 快照的分片字节
//!   逐位比较——任何一拍的任何差异都会传播进终态字节)。
//!
//! 真实网络栈构造不出 1 拍迁移在途窗口(TCP 帧粒度 ~10 拍),"在途那条
//! 停下"的时序判据由 P2 的驱动用例(真实迁移窗口)承担;本会话在边界
//! 附近密集发"停下",不硬断言在途命中(如实口径)。

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use app::config::Config;
use app::driver::{DriverArgs, DriverOutcome, ObserveCadence, ObserveLink, decode_checkpoint, run};
use app::net::chunk_stream::PumpConfig;
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::InputSource;
use physics_sim::{BodyRegistry, ShardSim};
use shard::SchedulerConfig;
use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use sim_api::session::{Session, SessionConfig};
use storage::mem::MemChunkStore;
use storage::traits::{CheckpointId, CheckpointStore, StoreError};
use storage::worker::StoreWorker;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{ChunkPos, DimensionId, EntityId, EntityState, ShardId, ShardPos, Tick, WorldId};
use voxel::SimpleRegistry;

/// Superflat classic 地面顶面 world y(a03_agents_net 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;

/// 软窗(a03_agents_net 同款):等一帧最多 50 ms。
const SOFT: std::time::Duration = std::time::Duration::from_millis(50);

/// 空闲信道软窗落空后的真实墙钟让步(a03_agents_net 同款)。
fn real_yield() {
    std::thread::sleep(std::time::Duration::from_millis(1));
}

fn capsule_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
}

/// 自制平板的顶面 world y(a04 同款:层 64 → world y = 0)。
const REPLAY_TOP_Y: f32 = 0.0;

fn flat_ground_chunk() -> voxel::Chunk {
    let mut chunk = voxel::Chunk::air();
    let stone = types::BlockId(1);
    for x in 0..voxel::CHUNK_X {
        for z in 0..voxel::CHUNK_Z {
            chunk.set(x, 64, z, stone);
        }
    }
    chunk
}

fn replay_sim(shard: ShardId, bodies: BodyRegistry) -> ShardSim {
    let cfg = physics_sim::ShardSimConfig::new(shard);
    ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), bodies)
}

/// 2×2 平板世界(a04 同款:四片先注册后装载;10b 布景用)。
fn four_shard_session(bodies: BodyRegistry) -> Session {
    let mut session = Session::new(SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    });
    for &(sx, sz) in &[(0i32, 0i32), (1, 0), (0, 1), (1, 1)] {
        session.register(replay_sim(
            ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(sx, sz)),
            bodies.clone(),
        ));
    }
    for &(sx, sz) in &[(0i32, 0i32), (1, 0), (0, 1), (1, 1)] {
        let shard = ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(sx, sz));
        for cx in (8 * sx)..(8 * sx + 8) {
            for cz in (8 * sz)..(8 * sz + 8) {
                session.apply_chunk_load(shard, ChunkPos::new(cx, cz), flat_ground_chunk());
            }
        }
    }
    session
}

fn capsule_spawn(pos: Vec3) -> EntityState {
    EntityState {
        translation: pos,
        ..EntityState::default()
    }
}

// ---------------------------------------------------------------------------
// 10a:两客户端完整会话 → 驱动日志回放逐拍逐位
// ---------------------------------------------------------------------------

/// 测试用检查点存储:全部 blob 记进共享 Vec(shutdown 后测试读回拼接日志)。
type CheckpointRecords = Arc<Mutex<Vec<(Tick, Vec<u8>)>>>;

#[derive(Default, Clone)]
struct RecordingCheckpointStore {
    records: CheckpointRecords,
}

impl RecordingCheckpointStore {
    fn read_all(&self) -> Vec<(Tick, Vec<u8>)> {
        self.records.lock().expect("锁不毒").clone()
    }
}

impl CheckpointStore for RecordingCheckpointStore {
    fn append(&mut self, at: Tick, blob: &[u8]) -> Result<CheckpointId, StoreError> {
        let mut records = self.records.lock().expect("锁不毒");
        records.push((at, blob.to_vec()));
        Ok(CheckpointId(records.len() as u64))
    }

    fn read_latest(&self) -> Result<Option<(Tick, Vec<u8>)>, StoreError> {
        Ok(self.records.lock().expect("锁不毒").last().cloned())
    }

    fn read(&self, id: CheckpointId) -> Result<Option<(Tick, Vec<u8>)>, StoreError> {
        Ok(self
            .records
            .lock()
            .expect("锁不毒")
            .iter()
            .find(|(i, _)| CheckpointId(i.0 + 1) == id)
            .cloned())
    }
}

/// 迷你客户端(a03_agents_net 同款)。
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

/// Hello + Subscribe + 等 Welcome(join;a03_agents_net 同款)。
async fn join(c: &mut TestClient, rx: &mut mpsc::UnboundedReceiver<S2C>) {
    c.send(&C2S::Hello { client_id: 1 })
        .await
        .expect("Hello 写入");
    c.send(&C2S::Subscribe { view_distance: 6 })
        .await
        .expect("Subscribe 写入");
    for _ in 0..300 {
        match next_frame(rx).await {
            Some(S2C::Welcome { .. }) => return,
            Some(_) => continue,
            None => real_yield(),
        }
    }
    panic!("Welcome 未到(join 失败)");
}

async fn next_frame(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Option<S2C> {
    tokio::time::timeout(SOFT, rx.recv()).await.ok().flatten()
}

/// 暂停时间下推进 N 拍(a03_agents_net 同款)。
async fn advance_ticks(ticks: u32) {
    let dt = std::time::Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

/// 逐拍收 AgentStates 帧并记录采样(推进一拍;返回本拍采到的根位姿)。
/// cadence (1,1) 下每输出点一帧;无变化时增量不发包,但全量周期 = 1 拍
/// 恒发(full = full_due || 空表)——视距内有主体的拍恒有帧。
async fn sample_agent_states(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    id: EntityId,
    samples: &mut Vec<(Tick, [f32; 3])>,
) {
    for _ in 0..50 {
        match next_frame(rx).await {
            Some(S2C::AgentStates { tick, states, .. }) => {
                if let Some((_, _, poses)) = states.iter().find(|(e, ..)| *e == id)
                    && let Some(root) = poses.first()
                {
                    samples.push((tick, root.translation.to_array()));
                    return;
                }
            }
            Some(_) => continue,
            None => {
                real_yield();
            }
        }
    }
    panic!("一拍窗口内没有 AgentStates 帧(采样失败;cadence (1,1) 全量每拍必发)");
}

struct Harness {
    tx: mpsc::Sender<app::driver::DriverCommand>,
    driver: tokio::task::JoinHandle<DriverOutcome>,
    addr: SocketAddr,
    checkpoints: RecordingCheckpointStore,
}

/// 10a 专用 harness:真实网络栈 + 驱动,checkpoint 落测试的记录型存储
/// (shutdown 后读回日志);observe (1,1) = 每拍增量+全量(逐拍采样)。
async fn harness_recording(config: Config) -> Harness {
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
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
        .expect("监听成功");
    std::mem::forget(bcast_tx);

    let session = Session::new(config.session_config());
    let (tx, rx) = mpsc::channel::<app::driver::DriverCommand>(256);
    let checkpoints = RecordingCheckpointStore::default();
    let store = StoreWorker::new(
        Box::new(MemChunkStore::new()),
        Box::new(RecordingCheckpointStore {
            records: Arc::clone(&checkpoints.records),
        }),
    );
    let (provider, new_sim, bodies) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        })
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
    Harness {
        tx,
        driver,
        addr,
        checkpoints,
    }
}

/// checkpoint 窗口按落盘序拼接成完整命令日志(每窗口 = 上次 checkpoint
/// 后的增量;log.take() 清空语义保证窗口不重不漏,按 tick 升序)。
fn stitched_log(checkpoints: &[(Tick, Vec<u8>)]) -> CommandLog {
    let mut log = CommandLog::new();
    for (_, blob) in checkpoints {
        let bundle = decode_checkpoint(blob).expect("checkpoint 可解(驱动写出的)");
        let window = CommandLog::from_bytes(&bundle.log_bytes).expect("日志窗口可解");
        for entry in window.entries() {
            log.append(entry.clone());
        }
    }
    log
}

/// A03-10a:两客户端完整会话(生成、附身、走过边界、一次伪造、一次
/// "停下"、断线)与命令日志回放逐拍逐位相同。在线侧逐拍状态 = AgentStates
/// 下行帧(f32 原值);回放侧 = `replay_world_with_bodies` 后逐拍
/// `agent_root_position`(同源 f32)。
#[tokio::test(start_paused = true)]
async fn a03_10a_replay_bit_identical() {
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 30,
        ..Config::default()
    };
    config.bodies_dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies");
    let h = harness_recording(config.clone()).await;
    let (mut a, mut rxa) = client(h.addr).await;
    let (mut b, mut rxb) = client(h.addr).await;
    tokio::task::yield_now().await;
    join(&mut a, &mut rxa).await;
    join(&mut b, &mut rxb).await;
    // A MoveObserver(中心覆盖行程全程,激活生产口径)。
    a.send(&C2S::MoveObserver {
        position: Vec3::new(110.0, GROUND_TOP_Y + 2.0, 8.0),
    })
    .await
    .expect("A MoveObserver");
    advance_ticks(30).await;

    // ① 生成(A 身体,近边界出生)。
    let e = {
        while rxa.try_recv().is_ok() {}
        a.send(&C2S::SpawnAgent {
            body: "capsule".to_string(),
            position: Vec3::new(110.0, GROUND_TOP_Y + 1.2, 8.0),
            yaw: 0.0,
        })
        .await
        .expect("SpawnAgent 写入");
        let mut found = None;
        for _ in 0..300 {
            match next_frame(&mut rxa).await {
                Some(S2C::Spawned { entity }) => {
                    found = Some(entity);
                    break;
                }
                Some(_) => continue,
                None => real_yield(),
            }
        }
        found.expect("A 身体生成回执")
    };
    advance_ticks(30).await;
    // ② 附身。
    {
        a.send(&C2S::Possess { entity: e })
            .await
            .expect("Possess 写入");
        let mut ok = false;
        for _ in 0..300 {
            match next_frame(&mut rxa).await {
                Some(S2C::Possessed { entity: Some(x) }) if x == e => {
                    ok = true;
                    break;
                }
                Some(_) => continue,
                None => real_yield(),
            }
        }
        assert!(ok, "附身成功");
    }
    advance_ticks(10).await;

    // ③ 逐拍采样起点(生成+附身后的稳态起)。
    let mut samples: Vec<(Tick, [f32; 3])> = Vec::new();

    // ④ 走过边界:意图每拍一条(B 在中段伪造一次);A 客户端逐拍收帧采样。
    // 出生 110 → 边界 128:加速 τ 0.2 s(~12 拍)+ 18 m ≈ 200+ 拍(中途
    // 迁移在途窗口的拍 AgentStates 不含该主体,采样帧缺失 = 正常,按拍号
    // 对齐比较)。
    for i in 0..260u32 {
        a.send(&C2S::AgentIntent {
            entity: e,
            move_xz: (5.6, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("意图");
        if i == 30 {
            // 伪造:B 给 A 附身的主体发意图(权威校验拒绝,不进日志)。
            b.send(&C2S::AgentIntent {
                entity: e,
                move_xz: (9.9, 0.0),
                yaw: 0.0,
                jump: false,
            })
            .await
            .expect("B 伪造意图写入");
        }
        advance_ticks(1).await;
        sample_agent_states(&mut rxa, e, &mut samples).await;
    }
    let crossed = samples.last().expect("有采样").1[0] > 126.0;
    assert!(crossed, "布景自检:260 拍采样末应已走到边界附近(>126)");

    // ⑤ "停下":边界附近密集发零意图(在途命中与否依赖帧相位,不硬断言;
    // 时序判据由 P2 驱动用例承担)。逐拍采样继续。
    for _ in 0..30 {
        a.send(&C2S::AgentIntent {
            entity: e,
            move_xz: (0.0, 0.0),
            yaw: 0.0,
            jump: false,
        })
        .await
        .expect("停下");
        advance_ticks(1).await;
        sample_agent_states(&mut rxa, e, &mut samples).await;
    }

    // ⑥ 断线(bye → 身体移除;采样继续到移除帧后结束)。
    a.bye().await;
    for _ in 0..40 {
        advance_ticks(1).await;
        // 移除后不再有该主体的帧;窗口耗尽即止。
        let mut got = false;
        for _ in 0..50 {
            match next_frame(&mut rxa).await {
                Some(S2C::AgentStates { tick, states, .. }) => {
                    if let Some((_, _, poses)) = states.iter().find(|(x, ..)| *x == e)
                        && let Some(root) = poses.first()
                    {
                        samples.push((tick, root.translation.to_array()));
                        got = true;
                    }
                    break;
                }
                Some(_) => continue,
                None => {
                    real_yield();
                }
            }
        }
        if !got {
            break;
        }
    }
    advance_ticks(30).await;
    drop(b);
    let Harness {
        tx,
        driver,
        addr,
        checkpoints,
    } = h;
    tx.send(app::driver::DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    let outcome = driver.await.expect("驱动不 panic");
    assert!(outcome.ticks_run > 0);
    let _ = addr;

    // —— 回放线:最早 checkpoint + 全量拼接日志重放,逐拍采样对比 ——
    let checkpoints = checkpoints.read_all();
    assert!(
        checkpoints.len() >= 2,
        "布景自检:期中 + 终末 checkpoint 应都有(实为 {})",
        checkpoints.len()
    );
    let log = stitched_log(&checkpoints);
    // 日志非空(生成/意图/断线移除都经日志)。
    assert!(!log.is_empty(), "驱动写出的命令日志非空");
    let (first_tick, first_blob) = &checkpoints[0];
    let bundle = decode_checkpoint(first_blob).expect("checkpoint 可解");
    assert_eq!(bundle.world.shards.tick, *first_tick);
    let mut replay = sim_api::replay::replay_world_with_bodies(
        &bundle.world,
        &log,
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        },
        || Box::new(SimpleRegistry::builtin()),
        capsule_registry,
    )
    .expect("重放装配成功");
    // 回放从快照拍续跑到最后采样拍,逐拍采样(同源 f32)。在线帧的 tick
    // 标签 = 输出点拍号(session.tick() 之后取 tick_now() = 已完成拍 + 1),
    // 帧里状态 = 已完成拍末——回放侧 tick 后(已完成拍 t-1)的采样与在线
    // 帧 tick = t 对齐。
    let start = first_tick.0;
    let end = samples.last().expect("非空").0.0;
    assert!(end > start, "采样应跨越 checkpoint 之后");
    let mut replay_samples: Vec<(Tick, [f32; 3])> = Vec::new();
    for t in (start + 1)..=end {
        replay.tick();
        if let Some(p) = replay.agent_root_position(e) {
            replay_samples.push((Tick(t), p.to_array()));
        }
    }
    // 对齐比较:在线采样 (tick, pos) 与回放同拍逐位相同。
    let mut misses = 0usize;
    for (tick, pos) in &samples {
        let Some((_, rp)) = replay_samples.iter().find(|(t, _)| t == tick) else {
            misses += 1;
            continue;
        };
        assert_eq!(pos, rp, "拍 {}:在线与回放主体状态逐位不同", tick.0);
    }
    assert_eq!(
        misses, 0,
        "在线采样中有 {misses} 拍在回放侧无对应点(帧 tick 对齐失败)"
    );
    replay.shutdown();
}

// ---------------------------------------------------------------------------
// 10b:检查点在玩家身体存活时落下;恢复清扫;续跑逐位
// ---------------------------------------------------------------------------

/// A03-10b:检查点必须在玩家身体还活着时落下(快照含玩家身体,恢复后
/// `is_agent` 为真);恢复走 `run` 的启动清扫——第一拍移除(计数 = 1);
/// 服务端主体续跑与不中断的那一线逐位相同(不中断线在同一拍移除玩家
/// 身体,两线终态分片字节逐位比较)。
#[tokio::test(start_paused = true)]
async fn a03_10b_checkpoint_resume_server_agent() {
    let bodies = capsule_registry();
    let hash = bodies.hash_of("capsule").expect("capsule 已装");

    // 布景:玩家身体(source = Some(1))+ 服务端主体(每 20 拍一条意图)。
    // tick 到 CP(120)落快照——玩家身体**还活着**。
    const CP: u64 = 120;
    const END: u64 = 360;
    let mk = || -> (Session, EntityId, EntityId, CommandLog) {
        let mut session = four_shard_session(bodies.clone());
        let player_body = EntityId::from_parts(0, 1);
        let server = EntityId::from_parts(0, 5);
        let mut log = CommandLog::new();
        let e1 = session
            .spawn_agent_from(
                InputSource(1),
                ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0)),
                player_body,
                hash,
                capsule_spawn(Vec3::new(12.0, REPLAY_TOP_Y + 1.2, 12.0)),
            )
            .expect("玩家身体生成");
        log.append(e1);
        let e2 = session
            .spawn_agent_from(
                InputSource::SCHEDULER,
                ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0)),
                server,
                hash,
                capsule_spawn(Vec3::new(8.0, REPLAY_TOP_Y + 1.2, 8.0)),
            )
            .expect("服务端主体生成(source = None)");
        log.append(e2);
        session.tick();
        (session, player_body, server, log)
    };
    // 每 20 拍一条服务端主体意图(两线共用的时间线;`from` = 起始拍)。
    let feed = |s: &mut Session, log: &mut CommandLog, i: u64, server: EntityId| {
        if i.is_multiple_of(20) {
            let entry = s
                .send_agent_intent_to(InputSource::SCHEDULER, server, (2.0, 0.0), 0.0, false)
                .expect("服务端意图入日志");
            log.append(entry);
        }
    };

    // 线 1(不中断):拍 CP 落快照(玩家身体活着);拍 CP+1 发恢复线清扫
    // 的同一命令(SCHEDULER 章、apply_at = CP+1 = 恢复线 run 清扫的章);
    // 跑到 END,逐拍采样 + 终态快照。
    let (mut cont, player_body, server, mut log1) = mk();
    let mut mid_samples = Vec::new();
    for i in 0..CP {
        feed(&mut cont, &mut log1, i, server);
        cont.tick();
        if (i + 1) % 30 == 0
            && let Some(p) = cont.agent_root_position(server)
        {
            mid_samples.push(p.to_array());
        }
    }
    let snap = cont.world_snapshot();
    // 布景自检(判据核心):快照里玩家身体**活着**(恢复后 is_agent 为真)。
    {
        let resumed_probe = sim_api::replay::replay_world_with_bodies(
            &snap,
            &CommandLog::new(),
            SessionConfig {
                scheduler: SchedulerConfig {
                    worker_count: 2,
                    start_tick: snap.shards.tick,
                    ..SchedulerConfig::default()
                },
                ..SessionConfig::default()
            },
            || Box::new(SimpleRegistry::builtin()),
            || bodies.clone(),
        )
        .expect("探针恢复");
        assert!(
            resumed_probe.is_agent(player_body),
            "检查点必须在玩家身体活着时落下(快照恢复后 is_agent 为真)"
        );
        assert!(resumed_probe.is_agent(server), "服务端主体也在快照里");
        resumed_probe.shutdown();
    }
    // 恢复线 run 清扫发出的命令 = send_to_entity(SCHEDULER, RemoveEntity),
    // apply_at = 恢复拍(CP+1)。不中断线在同一拍发同一条(两线才可比)。
    let entry = cont
        .send_to_entity(
            InputSource::SCHEDULER,
            player_body,
            messaging::SimCommand::RemoveEntity {
                entity: player_body,
            },
        )
        .expect("不中断线的移除命令");
    log1.append(entry);
    let mut s1 = Vec::new();
    for i in CP..END {
        feed(&mut cont, &mut log1, i, server);
        cont.tick();
        if let Some(p) = cont.agent_root_position(server) {
            s1.push(p.to_array());
        }
    }
    let cont_final = cont.world_snapshot();
    cont.shutdown();

    // 恢复线 run 级:replay 恢复(空增量日志:快照拍 CP 之前的命令已烙进
    // 快照;CP 拍及其后无外部命令——线 1 的 RemoveEntity 是线 1 自己发的,
    // 恢复线由 run 启动清扫发)→ 喂 run(observe 关闭)→ 启动清扫第一拍
    // (恢复拍 CP+1)移除玩家身体,计数 = 1;服务端主体保留。
    {
        let resumed = sim_api::replay::replay_world_with_bodies(
            &snap,
            &CommandLog::new(),
            SessionConfig {
                scheduler: SchedulerConfig {
                    worker_count: 2,
                    ..SchedulerConfig::default()
                },
                ..SessionConfig::default()
            },
            || Box::new(SimpleRegistry::builtin()),
            || bodies.clone(),
        )
        .expect("恢复");
        let config = app::config::Config {
            worker_count: 2,
            checkpoint_interval_ticks: END * 2,
            ..app::config::Config::default()
        };
        let store = StoreWorker::new(
            Box::new(MemChunkStore::new()),
            Box::new(storage::mem::MemCheckpointStore::new()),
        );
        let (provider, new_sim, _bodies) =
            app::driver::wire_provider_and_new_sim(config.clone(), &store, || {
                Box::new(SimpleRegistry::builtin())
            })
            .expect("装配失败(fail-stop;MARGIN 不足等类型化错误)");
        let (tx, rx) = tokio::sync::mpsc::channel::<app::driver::DriverCommand>(16);
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
            spawn_next_seq: 0,
            bodies: BodyRegistry::new(),
        }));
        // 推进 30 拍(清扫在第一拍;a03_recovery_sweep 同款推进)。
        for _ in 0..30 {
            tokio::time::sleep(std::time::Duration::from_secs_f32(1.0 / 60.0)).await;
            tokio::task::yield_now().await;
        }
        tx.send(app::driver::DriverCommand::Shutdown)
            .await
            .expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(
            outcome.agent.bodies_removed_on_recovery, 1,
            "恢复清扫第一拍移除玩家身体(计数 = 1)"
        );
        assert_eq!(outcome.agent.bodies_removed_on_disconnect, 0);
    }

    // 恢复线续跑(Session 层,与 run 清扫同一条命令:send_to_entity(
    // SCHEDULER, RemoveEntity) apply_at = 恢复拍 CP+1——run 清扫的命令
    // 构造即此式,计数语义由上一段 run 级断言守住):跑同样时间线,终态
    // 分片字节与不中断线逐位比较。
    let mut resumed = sim_api::replay::replay_world_with_bodies(
        &snap,
        &CommandLog::new(),
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        },
        || Box::new(SimpleRegistry::builtin()),
        || bodies.clone(),
    )
    .expect("恢复(续跑线)");
    let entry = resumed
        .send_to_entity(
            InputSource::SCHEDULER,
            player_body,
            messaging::SimCommand::RemoveEntity {
                entity: player_body,
            },
        )
        .expect("恢复线清扫命令(与 run 启动清扫同章)");
    let mut log2 = CommandLog::new();
    log2.append(entry);
    let mut s2 = Vec::new();
    for i in CP..END {
        feed(&mut resumed, &mut log2, i, server);
        resumed.tick();
        if let Some(p) = resumed.agent_root_position(server) {
            s2.push(p.to_array());
        }
    }
    let resumed_final = resumed.world_snapshot();
    resumed.shutdown();

    // 逐拍采样逐位相同(服务端主体续跑;快照前段两线同源不比,只比续跑段)。
    assert_eq!(s1.len(), s2.len(), "续跑段采样数一致");
    for (t, (x, y)) in s1.iter().zip(s2.iter()).enumerate() {
        assert_eq!(
            x,
            y,
            "拍 {}:恢复线与不中断线服务端主体逐位不同",
            CP + t as u64
        );
    }
    // 终态世界(全分片字节)逐位相同(比逐拍采样更强:任何一拍任何差异
    // 都会传播进终态字节)。
    assert_eq!(
        cont_final.shards.tick, resumed_final.shards.tick,
        "两线终态拍一致"
    );
    assert_eq!(
        cont_final.shards.per_shard, resumed_final.shards.per_shard,
        "两线终态全分片字节逐位相同"
    );
    // 布景自检:续跑段确实发生(主体移动过)。
    assert!(s1.len() > 10, "布景自检:续跑段采样非空");
    // 布景自检:恢复线里玩家身体已被移除(清扫生效;服务端主体在场)。
    {
        let probe = sim_api::replay::replay_world_with_bodies(
            &resumed_final,
            &CommandLog::new(),
            SessionConfig::default(),
            || Box::new(SimpleRegistry::builtin()),
            || bodies.clone(),
        )
        .expect("终态探针恢复");
        assert!(
            !probe.is_agent(player_body),
            "恢复线终态无玩家身体(清扫生效)"
        );
        assert!(probe.is_agent(server), "恢复线终态服务端主体在场");
        probe.shutdown();
    }
    let _ = (mid_samples, log2);
}
