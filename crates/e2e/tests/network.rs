//! REQ-003 验收套件(任务书 req003-net/04):AC-N1~N6 端到端。
//!
//! 工程取舍(任务书「工程细节」段的实测结论,notes 同步汇报):
//! - 全部测试 `#[tokio::test(start_paused = true)]`:驱动/泵/超时走 tokio 虚拟
//!   时钟(advance_ticks 逐拍推进,暂停时钟由休眠自动兑现),网络 = 真 TCP
//!   (127.0.0.1:0 实际端口,README 决策 10;暂停时钟不影响 socket 真实 IO,
//!   任务 03 的 app 内观察测试同款组合已实证);
//! - keepalive 读超时与暂停时钟的冲突(任务书预告的取舍):read_timeout 走
//!   tokio 虚拟时钟,默认 10s 在物理等待动辄数十虚拟秒的 AC 场景会把客户端
//!   踢下线(keepalive_timeout_disconnects 实证了该机制)——本套件把
//!   read_timeout_secs 调大(测试参数,真实部署语义仍 10s 墙钟,该语义由
//!   任务 01 专项测试覆盖);
//! - 慢客户端断连机制(任务 01 实测):出站冲刷写超时(FLUSH_WRITE_TIMEOUT=5s,
//!   虚拟时钟)→ SocketClosed 收口;Critical 队列满(64)字面支路需冲刷不阻塞
//!   而队列独积,真 socket 上不可自然构造——其单元口径由 app 内任务 01/03
//!   测试锚定,本套件断言 AC-N5 语义面(丢最旧降级 + 断连收口 + tick 不减);
//! - 断连→票撤(AC-N6):Left 上行 → 驱动 `Session::remove_observer`
//!   (评审修复②)——观察者表条目随断连移除,空片无票即可 despawn,
//!   缩短配置下端到端验证;
//! - 帧 codec:复用 sim-api::proto 对称公开接口(encode_c2s_frame / decode_frame);
//!   MiniClient 为任务书 04 授权的内嵌副本(net::tests 版 = cfg(test) pub(crate),
//!   集成测试不可达)。

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use app::config::Config;
use app::driver::{DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink, run};
use app::net::chunk_stream::{MAX_LOADS_PER_PUMP, PumpConfig, chebyshev, view_square};
use app::net::{NetConfig, NetUp, OutboundHub, serve};
use messaging::{InputSource, SimCommand};
use physics_sim::{ShardSim, SolidifyConfig};
use sim_api::log::CommandLog;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use sim_api::session::Session;
use storage::mem::MemCheckpointStore;
use storage::worker::StoreWorker;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::Vec3;
use types::{
    BlockId, BlockPos, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos,
    WorldId,
};
use voxel::{BlockRegistry, Chunk, SimpleRegistry, Superflat, WorldGenerator};

/// Superflat classic 地面顶面 world y(-64 + 层厚 4;world_loading.rs 同口径)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;

/// 软窗:等一帧最多 50ms(虚拟时钟;服务端静默即落空返回,不挂死——
/// 任务 03 测试同款)。
const SOFT: Duration = Duration::from_millis(50);

/// 实体身份(origin = 分片 (1,0) 序号;AC-N3 的实体出生位在世界 x=200 区
/// (分片 (1,0))——翻译层对 TeleportEntity/RemoveEntity 按 origin 序号做
/// **信息性路由**(`net.rs command_shard`:origin → ShardPos(origin,0)),
/// origin 必须与所在注册分片一致,否则调度器 fail-stop)。
fn eid(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

/// 测试用分片身份(与 world_loading / driver 测试同风格)。
fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// registry 工厂(world_loading.rs 同款)。
fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

/// 暂停时间下推进 N 拍(auto-advance;world_loading.rs 同款)。
async fn advance_ticks(ticks: u32) {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

// ---------------------------------------------------------------------------
// 内嵌迷你客户端(任务书 04 授权复制;codec = sim-api::proto 公开对称接口)
// ---------------------------------------------------------------------------

/// 写端(帧 = 4B LE 长度 + bincode,README 决策 2)。
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

/// 连接 + 读通道:reader task 解帧转发无界通道;EOF/出错 = 服务端断连
/// (reader 退出 → 发送端 drop → `rx.is_closed()` 成为测试侧断连观测)。
async fn client(addr: SocketAddr) -> (TestClient, mpsc::UnboundedReceiver<S2C>) {
    let stream = TcpStream::connect(addr).await.expect("TCP 连入");
    let (mut r, w) = stream.into_split();
    let (tx, rx) = mpsc::unbounded_channel::<S2C>();
    tokio::spawn(async move {
        let mut buf: Vec<u8> = Vec::with_capacity(16 * 1024);
        let mut chunk = [0u8; 16 * 1024];
        loop {
            // 凑一帧(4B 前缀 + 载荷),解码复用 proto::decode_frame(帧格式
            // 单一事实源,README 决策 10)。
            let frame = loop {
                if buf.len() >= 4 {
                    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize + 4;
                    if buf.len() >= len {
                        break buf.drain(..len).collect::<Vec<u8>>();
                    }
                }
                match r.read(&mut chunk).await {
                    Ok(0) | Err(_) => return, // EOF/出错:链路死亡
                    Ok(n) => buf.extend_from_slice(&chunk[..n]),
                }
            };
            if let Some(s2c) = decode_frame(&frame)
                && tx.send(s2c).is_err()
            {
                return; // 测试侧已退场
            }
        }
    });
    (TestClient { w }, rx)
}

/// Hello + Subscribe,返回 Welcome 的 player_id。
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
            Some(_) => continue, // Welcome 先行冲刷,理论无前置帧;防御式跳过
            None => panic!("软窗预算内未见 Welcome(首帧即连接层 Welcome)"),
        }
    }
    panic!("64 软窗内未收到 Welcome")
}

/// 外部生成(REQ-006 任务 02):发 `C2S::Spawn` → 等 `S2C::Spawned`,返回
/// 服务端分配的 ID;其它观察帧跳过。
async fn spawn_entity(
    c: &mut TestClient,
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    translation: Vec3,
) -> EntityId {
    c.send(&C2S::Spawn {
        state: EntityState {
            translation,
            ..EntityState::default()
        },
        desc: EntityDesc::default(),
    })
    .await
    .expect("Spawn 写入");
    for _ in 0..300 {
        match next_frame(rx).await {
            Some(S2C::Spawned { entity }) => return entity,
            Some(_) => continue,
            None => break,
        }
    }
    panic!("300 软窗内未收到 Spawned 回执");
}

/// 软窗收一帧(None = 窗口内无帧;虚拟时钟已推进 ~3 拍)。
async fn next_frame(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Option<S2C> {
    tokio::time::timeout(SOFT, rx.recv()).await.ok().flatten()
}

/// 非阻塞收一帧(不推进时钟)。
fn try_frame(rx: &mut mpsc::UnboundedReceiver<S2C>) -> Option<S2C> {
    rx.try_recv().ok()
}

/// 收集 ChunkData 到 want 个(软窗 × budget 上限;不足原样带回,数量判定
/// 归调用方)。泵「等回执才续下一轮 + 每 cadence 至多一 diff」的串行性保证:
/// want = MAX_LOADS_PER_PUMP 时收集恰为一泵之量(不混入下一泵)。
async fn collect_chunk_loads(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    want: usize,
    budget: u32,
) -> Vec<(ChunkPos, Chunk)> {
    let mut out = Vec::new();
    for _ in 0..budget {
        if let Some(S2C::ChunkData { pos, chunk }) = next_frame(rx).await {
            out.push((pos, chunk));
            if out.len() >= want {
                break;
            }
        }
    }
    out
}

/// 差分窗口:总帧数到 min_frames 后连续 2 静默窗收口(装卸分桶,其余帧跳过)。
async fn settle_load_unload(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    min_frames: usize,
    budget: u32,
) -> (Vec<ChunkPos>, Vec<ChunkPos>) {
    let mut loads: Vec<ChunkPos> = Vec::new();
    let mut unloads: Vec<ChunkPos> = Vec::new();
    let mut quiet = 0;
    for _ in 0..budget {
        match next_frame(rx).await {
            Some(S2C::ChunkData { pos, .. }) => {
                loads.push(pos);
                quiet = 0;
            }
            Some(S2C::ChunkUnload { pos }) => {
                unloads.push(pos);
                quiet = 0;
            }
            Some(_) => quiet = 0,
            None => {
                quiet += 1;
                if quiet >= 2 && loads.len() + unloads.len() >= min_frames {
                    break;
                }
            }
        }
    }
    (loads, unloads)
}

/// 观察输出侧的实体位姿监视:budget 帧窗口内,EntityStates 含 id 且谓词成立
/// → 返回该状态。
async fn watch_entity(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    id: EntityId,
    budget: u32,
    mut pred: impl FnMut(&EntityState) -> bool,
) -> Option<EntityState> {
    for _ in 0..budget {
        if let Some(S2C::EntityStates { states, .. }) = next_frame(rx).await
            && let Some((_, st)) = states.iter().find(|(e, _)| *e == id)
            && pred(st)
        {
            return Some(*st);
        }
    }
    None
}

/// budget 窗内 EntityStates 是否出现过该实体(正/负样本共用;负样本传小预算)。
async fn entity_appears(rx: &mut mpsc::UnboundedReceiver<S2C>, id: EntityId, budget: u32) -> bool {
    for _ in 0..budget {
        if let Some(S2C::EntityStates { states, .. }) = next_frame(rx).await
            && states.iter().any(|(e, _)| *e == id)
        {
            return true;
        }
    }
    false
}

/// 等待 BlockUpdates 逐格收账(同拍合并成单包)。
async fn wait_cells(
    rx: &mut mpsc::UnboundedReceiver<S2C>,
    cells: &[(BlockPos, BlockId)],
    budget: u32,
) -> bool {
    let mut hit = vec![false; cells.len()];
    for _ in 0..budget {
        if let Some(S2C::BlockUpdates { updates }) = next_frame(rx).await {
            for (i, (pos, block)) in cells.iter().enumerate() {
                if !hit[i] && updates.contains(&(*pos, *block)) {
                    hit[i] = true;
                }
            }
            if hit.iter().all(|h| *h) {
                return true;
            }
        }
    }
    false
}

/// 视距方形的 center-out 升序表(纯函数口径,断言基准;键 =
/// (切比雪夫距, x, z),泵内 diff 同式)。
fn sorted_square(center: ChunkPos, view: u8) -> Vec<ChunkPos> {
    let mut v: Vec<ChunkPos> = view_square(center, view).into_iter().collect();
    v.sort_by_key(|p| (chebyshev(center, *p), p.x, p.z));
    v
}

// ---------------------------------------------------------------------------
// 装配 harness:真驱动 + 真网络(observe 缝真接)+ 127.0.0.1:0 实际端口
// ---------------------------------------------------------------------------

struct Harness {
    tx: mpsc::Sender<DriverCommand>,
    driver: tokio::task::JoinHandle<DriverOutcome>,
    hub: Arc<OutboundHub>,
    addr: SocketAddr,
}

/// 测试通用配置:2 worker、checkpoint 疏(600,终末必写)、其余默认。
fn base_config() -> Config {
    Config {
        worker_count: 2,
        checkpoint_interval_ticks: 600,
        ..Config::default()
    }
}

/// 起驱动 + 网络(observe 三缝真接;真 provider = worker 通道 + 生成器 + seed):
/// - pump:chunk 流泵节奏(AC-N2 用 interval=200ms/catchup=1 得「单泵恰 ≤16」
///   的可观测口径,其余默认 10 Hz);
/// - observe:增量/全量周期(拍);
/// - zero_g:测试分片零重力(AC-N4「两包位级 ==」需绝对静止;物理生效面由
///   AC-N3 以真实重力覆盖);
/// - 凝固桥一律关闭(本套件断言命令生效,不是凝固落定;world_loading
///   AC-G3 同款口径);
/// - read_timeout_secs = 600:keepalive 读超时在 start_paused 下是**虚拟** 10s,
///   物理等待动辄数十虚拟秒会误踢客户端(取舍见模块头;keepalive 本体语义
///   由任务 01 专项测试覆盖)。
async fn harness(config: Config, pump: PumpConfig, observe: (u64, u64), zero_g: bool) -> Harness {
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 600,
        observe_interval_ticks: observe.0,
        full_snapshot_interval_ticks: observe.1,
        pump,
    };
    let cadence = ObserveCadence::from(&net_cfg);
    let (up_tx, up_rx) = mpsc::channel::<NetUp>(256);
    let hub = Arc::new(OutboundHub::new());
    let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
    let addr = serve(net_cfg, up_tx, bcast_rx, hub.clone())
        .await
        .expect("监听成功(127.0.0.1:0)");
    // 关机广播发送端随测试存续(accept 循环以其存活为常驻条件;driver
    // observe_harness 同款 std::mem::forget 手法)。
    std::mem::forget(bcast_tx);

    let session = Session::new(config.session_config());
    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    // provider 走生产装配(worker 通道 + 生成器按 world_kind + seed);
    // new_sim 自造(凝固桥关;zero_g 可选)。
    let (provider, _) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let sim_source = config.clone();
    let new_sim = Box::new(move |shard: ShardId| {
        let mut cfg = sim_source.shard_sim_config(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        if zero_g {
            cfg.gravity = Vec3::ZERO;
        }
        ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
    });
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
    }));
    Harness {
        tx,
        driver,
        hub,
        addr,
    }
}

/// 收口:Shutdown → join 驱动(panic = 红)。
async fn shutdown(h: Harness) -> DriverOutcome {
    h.tx.send(DriverCommand::Shutdown)
        .await
        .expect("驱动仍存活");
    h.driver.await.expect("驱动不 panic")
}

// ---------------------------------------------------------------------------
// AC-N1 连接闭环
// ---------------------------------------------------------------------------

/// AC-N1:客户端连入 → Welcome(身份 1、2 递增)→ Subscribe(hub 注册)→
/// MoveObserver → 世界该区域分片被创建/装载(经驱动编排,outcome 计数),
/// chunk 流随观察者中心迁移;Goodbye → 出站清理,双客户端互不影响。
#[tokio::test(start_paused = true)]
async fn acn1_connect_join_and_observe() {
    let h = harness(base_config(), PumpConfig::default(), (2, 60), false).await;

    let (mut a, mut rx_a) = client(h.addr).await;
    let (mut b, mut rx_b) = client(h.addr).await;
    // 让出事件循环:accept 与连接 task 就绪后才能入会。
    tokio::task::yield_now().await;

    let id_a = join(&mut a, &mut rx_a, 2).await;
    let id_b = join(&mut b, &mut rx_b, 0).await;
    assert_eq!(id_a, InputSource(1), "身份从 1 起(0 保留)");
    assert_eq!(id_b, InputSource(2), "身份递增");
    assert!(
        h.hub.subscribers().contains(&id_a) && h.hub.subscribers().contains(&id_b),
        "Subscribe 完成后双方注册进出站 hub(观察输出面)"
    );

    // MoveObserver 进新区:chunk (12,0) → 分片 (1,0)。不钉首泵中心——订阅帧
    // 与移动帧到达序对首个泵 cadence 存在竞先,latest-wins 下首泵落旧心也被
    // 后续差分自然纠正;AC-N1 断言面 = 观察者新区 chunk 经协议 + 驱动取数
    // 到位(真数据源位级)。
    a.send(&C2S::MoveObserver {
        position: Vec3::new(200.0, -58.0, 8.0),
    })
    .await
    .expect("MoveObserver 写入");
    let new_center = ChunkPos::new(12, 0);
    let mut new_chunk = None;
    for _ in 0..60 {
        if let Some(S2C::ChunkData { pos, chunk }) = next_frame(&mut rx_a).await
            && pos == new_center
        {
            new_chunk = Some(chunk);
            break;
        }
    }
    let chunk = new_chunk.expect("观察者新区 chunk (12,0) 经流到达");
    assert_eq!(
        chunk,
        Superflat::classic().generate(types::WorldSeed(0), new_center),
        "新区 chunk = 真数据源生成值(位级)"
    );
    // 再推进让编排计数字兑现(观察者应用 → ensure → 激活装载)。
    advance_ticks(120).await;

    // A 主动 Goodbye → Left 上行 → 出站注册清理;B 不受影响。
    a.bye().await;
    advance_ticks(3).await;
    assert!(
        !h.hub.subscribers().contains(&id_a),
        "Goodbye → 出站注册清理"
    );
    assert!(
        h.hub.subscribers().contains(&id_b),
        "双客户端:B 不受 A 断连影响"
    );

    let outcome = shutdown(h).await;
    assert!(
        outcome.shards_spawned >= 1,
        "MoveObserver → 分片经驱动编排 ensure 创建"
    );
    assert!(
        outcome.chunks_loaded >= 1,
        "激活装载 + 流取数经 provider 真路径"
    );
}

// ---------------------------------------------------------------------------
// AC-N2 chunk 流
// ---------------------------------------------------------------------------

/// AC-N2:新区首流按切比雪夫 center-out 序、每泵恰 16(catchup=1 可观测口径),
/// 多泵补齐;走「入新区域」= 最小差分(新列进/旧列退,远离侧收 ChunkUnload);
/// 视距钳制 200→32(首泵 = 32 方形前 16);ChunkData = 真数据源生成值(位级)。
#[tokio::test(start_paused = true)]
async fn acn2_chunk_stream_center_out_capped() {
    let pump = PumpConfig {
        interval_ms: 200,
        max_catchup_rounds: 1,
    };
    let config = base_config();
    let h = harness(config.clone(), pump, (2, 60), false).await;

    // —— A:视距 2(中心 = 初始 (0,0))。首泵恰 16(center-out)——
    let (mut a, mut rx_a) = client(h.addr).await;
    join(&mut a, &mut rx_a, 2).await;
    let expected = sorted_square(ChunkPos::new(0, 0), 2);
    let first_pump = collect_chunk_loads(&mut rx_a, MAX_LOADS_PER_PUMP, 40).await;
    assert_eq!(
        first_pump.len(),
        MAX_LOADS_PER_PUMP,
        "首泵恰 16(每泵限额;实 {})",
        first_pump.len()
    );
    assert_eq!(
        first_pump.iter().map(|(p, _)| *p).collect::<Vec<_>>(),
        expected[..MAX_LOADS_PER_PUMP],
        "首泵 center-out 升序(切比雪夫距,再 x,z)"
    );
    // 真数据源位级校验:(0,0) chunk == Superflat 生成值。
    let chunk_00 = &first_pump
        .iter()
        .find(|(p, _)| *p == ChunkPos::new(0, 0))
        .expect("中心 chunk 必在首泵")
        .1;
    assert_eq!(
        *chunk_00,
        Superflat::classic().generate(config.world_seed, ChunkPos::new(0, 0)),
        "ChunkData = 真数据源生成值(位级 ==)"
    );
    // 二泵补齐余 9(仍 center-out 续序)。
    advance_ticks(48).await; // ≥ 下一 cadence(+200ms)
    let second_pump = collect_chunk_loads(&mut rx_a, 9, 40).await;
    assert_eq!(
        second_pump.iter().map(|(p, _)| *p).collect::<Vec<_>>(),
        expected[MAX_LOADS_PER_PUMP..],
        "次泵续装余 9,续序不变(多泵补齐)"
    );

    // —— 走「入新区域」:中心 → chunk (1,0)(world x=20):新列 5 进 / 旧列 5 退 ——
    a.send(&C2S::MoveObserver {
        position: Vec3::new(20.0, -58.0, 8.0),
    })
    .await
    .expect("MoveObserver 写入");
    advance_ticks(48).await; // 覆盖下一 cadence 的差分
    let (loads2, unloads2) = settle_load_unload(&mut rx_a, 10, 40).await;
    let new_square = view_square(ChunkPos::new(1, 0), 2);
    let old_square = view_square(ChunkPos::new(0, 0), 2);
    assert!(
        loads2.iter().all(|p| new_square.contains(p)),
        "新装 chunk 全在新中心方形内: {loads2:?}"
    );
    let mut final_loaded: std::collections::BTreeSet<ChunkPos> = expected.iter().copied().collect();
    for p in &unloads2 {
        assert!(
            old_square.contains(p) && !new_square.contains(p),
            "ChunkUnload 只卸出视距的旧 chunk:{p:?}"
        );
        final_loaded.remove(p);
    }
    final_loaded.extend(loads2.iter().copied());
    assert_eq!(
        final_loaded, new_square,
        "差分闭合:终态 = 新视距方形(装 {loads2:?} / 卸 {unloads2:?})"
    );

    // —— 钳制:Subscribe{200} → 泵按 32 方形计(首泵 = 前 16 center-out)——
    let (mut c, mut rx_c) = client(h.addr).await;
    join(&mut c, &mut rx_c, 200).await;
    let expected32 = sorted_square(ChunkPos::new(0, 0), 32);
    let clamped_first = collect_chunk_loads(&mut rx_c, MAX_LOADS_PER_PUMP, 60).await;
    assert_eq!(
        clamped_first.iter().map(|(p, _)| *p).collect::<Vec<_>>(),
        expected32[..MAX_LOADS_PER_PUMP],
        "Subscribe 200 → 钳 32:首泵 = 32 方形前 16(端到端闭合)"
    );
    c.bye().await;
    advance_ticks(2).await;

    shutdown(h).await;
}

// ---------------------------------------------------------------------------
// AC-N3 命令往返
// ---------------------------------------------------------------------------

/// AC-N3(真重力,物理生效面):生成(经协议)→ 实体出现并落上生成地面;
/// 挖(SetBlock air)→ BlockUpdates 到达 + 实体下陷;放(同格石头)→ 抬回地面;
/// 爆炸(Detonate)→ linvel 激增(被炸飞;物理量按容差)。
#[tokio::test(start_paused = true)]
async fn acn3_command_roundtrip() {
    let h = harness(base_config(), PumpConfig::default(), (2, 60), false).await;
    let (mut a, mut rx) = client(h.addr).await;
    join(&mut a, &mut rx, 4).await; // 视距 4 chunk = 64m

    // 观察者进分片 (1,0) 区域;留 8 拍:观察者应用 + 编排 ① ensure + 中心
    // chunk 装载(未注册分片 Send = 调度器 fail-stop,命令必须晚于 ensure)。
    a.send(&C2S::MoveObserver {
        position: Vec3::new(200.0, -58.0, 8.0),
    })
    .await
    .expect("MoveObserver 写入");
    advance_ticks(8).await;

    // 生成经协议:Spawn(出生 y=-55,自由落体)。几何取 .0 位:实体盒
    // [207.5,208.5]×[7.5,8.5] 整占格 x∈{207,208}, z∈{7,8}——挖/放必须覆盖
    // 全部承托格,否则边列留活(几何口径在挖段注释)。ID 由服务端分配。
    let e = spawn_entity(&mut a, &mut rx, Vec3::new(208.0, -55.0, 8.0)).await;
    assert!(
        watch_entity(&mut rx, e, 300, |st| (st.translation
            - Vec3::new(208.0, -55.0, 8.0))
        .length()
            < 0.75)
        .await
        .is_some(),
        "生成命令经协议送达模拟:实体出现于 EntityStates"
    );
    assert!(
        watch_entity(&mut rx, e, 400, |st| {
            st.translation.y > GROUND_TOP_Y && st.translation.y < GROUND_TOP_Y + 1.5
        })
        .await
        .is_some(),
        "实体落在生成地面上被支撑(驻留带,容差)"
    );

    // 挖:碰撞盒整占格 (207..=208, -61, 7..=8) 的 dirt → air(承托全部移除);
    // BlockUpdates 收账(四格合并单包)。落稳体已被引擎睡眠(拆支撑不唤醒,
    // 引擎标准语义)→ 物理生效经协议 TeleportEntity 重新空投检验:无支撑
    // 即下陷一格,驻留带整体降 1m(新顶面 -61 上方;与原位驻留带互斥——
    // 唯一解释 = 支撑确被移除,命令在模拟物理生效)。
    let hole: Vec<(BlockPos, BlockId)> = [
        BlockPos::new(207, -61, 7),
        BlockPos::new(208, -61, 7),
        BlockPos::new(207, -61, 8),
        BlockPos::new(208, -61, 8),
    ]
    .into_iter()
    .map(|pos| (pos, BlockId::AIR))
    .collect();
    for &(pos, block) in &hole {
        a.send(&C2S::Command(SimCommand::SetBlock { pos, block }))
            .await
            .expect("SetBlock(挖)写入");
    }
    assert!(
        wait_cells(&mut rx, &hole, 60).await,
        "方块变更经协议回流 BlockUpdates(含所挖四格)"
    );
    a.send(&C2S::Command(SimCommand::TeleportEntity {
        entity: e,
        state: EntityState {
            translation: Vec3::new(208.0, -55.0, 8.0),
            ..EntityState::default()
        },
    }))
    .await
    .expect("TeleportEntity(重投)写入");
    assert!(
        watch_entity(&mut rx, e, 400, |st| {
            st.translation.y < GROUND_TOP_Y && st.translation.y > GROUND_TOP_Y - 1.6
        })
        .await
        .is_some(),
        "挖而后陷:重投实体落入洞内(新顶面 -61 上方,容差)"
    );

    // 放:同四格石头回置 → 物理生效同样经重投检验:清醒体落回列上,被新
    // 放置层承托回原地面上方(若未生效则穿洞坠到 -61 顶带——两带互斥)。
    let fill: Vec<(BlockPos, BlockId)> = hole.iter().map(|(pos, _)| (*pos, BlockId(1))).collect();
    for &(pos, block) in &fill {
        a.send(&C2S::Command(SimCommand::SetBlock { pos, block }))
            .await
            .expect("SetBlock(放)写入");
    }
    assert!(
        wait_cells(&mut rx, &fill, 60).await,
        "放置同样经 BlockUpdates 广播"
    );
    a.send(&C2S::Command(SimCommand::TeleportEntity {
        entity: e,
        state: EntityState {
            translation: Vec3::new(208.0, -55.0, 8.0),
            ..EntityState::default()
        },
    }))
    .await
    .expect("TeleportEntity(重投 2)写入");
    assert!(
        watch_entity(&mut rx, e, 400, |st| {
            st.translation.y > GROUND_TOP_Y && st.translation.y < GROUND_TOP_Y + 1.5
        })
        .await
        .is_some(),
        "放置生效:重投实体被新层承托回地面驻留带"
    );

    // 爆炸:Detonate 经协议 → 冲量 ≈ power×(1−d/r)(m=1)→ linvel 激增。
    a.send(&C2S::Command(SimCommand::Detonate {
        center: Vec3::new(208.0, -59.3, 8.0),
        power: 8.0,
        radius: 30.0,
    }))
    .await
    .expect("Detonate 写入");
    assert!(
        watch_entity(&mut rx, e, 60, |st| st.linvel.length() > 0.5)
            .await
            .is_some(),
        "爆炸经协议生效:linvel 激增(被炸飞,|v| > 0.5)"
    );

    let outcome = shutdown(h).await;
    assert!(outcome.ticks_run > 0, "场景确有推进");
}

// ---------------------------------------------------------------------------
// AC-N4 观察输出一致性
// ---------------------------------------------------------------------------

/// AC-N4(零重力绝对静止场景——协议观察面两包位级一致性只在物理静止时成立,
/// 物理生效面由 AC-N3 覆盖):订阅全量打底 → 传送(变化)→ 增量单包更新;
/// 全量周期到 = 整表且 **与增量累积终态逐实体位级 ==**;方块变更到达。
#[tokio::test(start_paused = true)]
async fn acn4_incremental_matches_full() {
    let h = harness(base_config(), PumpConfig::default(), (2, 60), true).await;
    let (mut a, mut rx) = client(h.addr).await;
    join(&mut a, &mut rx, 2).await; // 过滤半径 32m(小视距:chunk 流不淹没观察帧)
    a.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, 3.0, 8.0),
    })
    .await
    .expect("MoveObserver 写入");
    advance_ticks(4).await; // 观察者应用 + ensure

    // 原点区实体:服务端分配 ID(REQ-006 任务 02;客户端不自选)。
    let e = spawn_entity(&mut a, &mut rx, Vec3::new(10.0, 3.0, 10.0)).await;
    // 订阅后首见 = 全量打底(removed 恒空;增量不含未变化实体)。
    let first = watch_entity(&mut rx, e, 300, |_| true)
        .await
        .expect("订阅后全量打底含出生实体");
    let mut accum: std::collections::BTreeMap<EntityId, EntityState> =
        std::collections::BTreeMap::new();
    accum.insert(e, first);

    // 变化(TeleportEntity;零重力 → 到位后绝对静止)。
    a.send(&C2S::Command(SimCommand::TeleportEntity {
        entity: e,
        state: EntityState {
            translation: Vec3::new(20.0, 3.0, 10.0),
            ..EntityState::default()
        },
    }))
    .await
    .expect("TeleportEntity 写入");
    let moved = watch_entity(&mut rx, e, 300, |st| {
        (st.translation - Vec3::new(20.0, 3.0, 10.0)).length() < 0.05
    })
    .await
    .expect("传送后增量含新位置");
    accum.insert(e, moved);

    // 全量周期(60 拍)到 → 整表;与增量累积终态逐实体位级 ==。
    advance_ticks(70).await;
    let mut saw_full = false;
    for _ in 0..400 {
        if let Some(S2C::EntityStates {
            ref states,
            ref removed,
            ..
        }) = next_frame(&mut rx).await
            && let Some((_, st)) = states.iter().find(|(id, _)| *id == e)
        {
            // 静止实体再现 = 增量不含未变化实体 → 此包必为全量。
            assert!(!removed.contains(&e), "全量包 removed 恒不含在场实体");
            let acc = accum.get(&e).expect("增量累积含该实体");
            assert_eq!(st, acc, "AC-N4:全量 == 增量累积终态(逐实体位级)");
            saw_full = true;
            break;
        }
    }
    assert!(saw_full, "60 拍全量周期内须见整表包(含静止实体)");

    // 方块变更到达(驱动镜像外部 SetBlock → 按视距覆盖广播)。
    let cell = (BlockPos::new(4, 3, 4), BlockId(1));
    a.send(&C2S::Command(SimCommand::SetBlock {
        pos: cell.0,
        block: cell.1,
    }))
    .await
    .expect("SetBlock 写入");
    assert!(
        wait_cells(&mut rx, std::slice::from_ref(&cell), 60).await,
        "方块变更经 BlockUpdates 到达订阅者"
    );

    shutdown(h).await;
}

// ---------------------------------------------------------------------------
// AC-N5 背压
// ---------------------------------------------------------------------------

/// AC-N5:慢客户端(视距 32 = 4225 chunk 洪水,全程不读 socket):
/// World 队列丢最旧(hub 计数增长 = 降级不断服)、连接被服务端收口(冲刷写
/// 超时 → Left → 注册清理)、正常客户端全程体验不受影响、tick 计数不受拖累
/// (虚拟时长足量兑现拍数)。
#[tokio::test(start_paused = true)]
async fn acn5_slow_client_backpressure() {
    let h = harness(base_config(), PumpConfig::default(), (2, 60), false).await;

    // 正常客户端:视距 2,观察者落原点,reader 全程真收帧。
    let (mut normal, mut rxn) = client(h.addr).await;
    let id_normal = join(&mut normal, &mut rxn, 2).await;
    normal
        .send(&C2S::MoveObserver {
            position: Vec3::new(8.0, -58.0, 8.0),
        })
        .await
        .expect("MoveObserver 写入");

    // 慢客户端:入会后**永不读 socket**(大视距 → chunk 流洪水灌满 World 队列
    // 与 socket 缓冲;写侧由冲刷写超时 5s 收口,任务 01 实测机制)。
    let mut slow = TcpStream::connect(h.addr).await.expect("慢客户端连入");
    slow.write_all(&encode_c2s_frame(&C2S::Hello { client_id: 9 }))
        .await
        .expect("Hello 写入");
    slow.write_all(&encode_c2s_frame(&C2S::Subscribe { view_distance: 32 }))
        .await
        .expect("Subscribe 写入(大视距洪水)");
    advance_ticks(2).await; // 兑现 Joined/Subscribed 上行 → hub 注册
    let id_slow = *h
        .hub
        .subscribers()
        .iter()
        .find(|p| **p != id_normal)
        .expect("慢客户端已注册");

    // 灌洪水 ≈ 7.7s 虚拟(泵 16/轮 × 快进;4225 ≫ 512 容量必触顶;
    // 冲刷写超时 5s → 断连收口)。
    advance_ticks(460).await;

    // ① 降级不断服:World 满 → 丢最旧。
    assert!(
        h.hub.world_dropped() > 0,
        "慢客户端 → World 队列丢最旧(hub 计数增长)"
    );
    // ② 出站堵死的连接被服务端收口(不养僵尸;写超时 → Left → 注册清理)。
    assert!(
        !h.hub.subscribers().contains(&id_slow),
        "慢客户端被服务端收口(Critical/冲刷堵塞 → 断连,AC-N5/README 决策 6)"
    );
    // ③ 正常客户端体验不受影响:在场且收得到帧(其 chunk 流/观察帧在途)。
    assert!(!rxn.is_closed(), "正常客户端全程在场");
    let mut got_frame = false;
    for _ in 0..40 {
        if next_frame(&mut rxn).await.is_some() {
            got_frame = true;
            break;
        }
    }
    assert!(got_frame, "正常客户端在洪水期仍能收帧(体验不受拖累)");

    // ④ tick 不减:虚拟 460 拍 → ≥ 400(被拖累则远低于应得值;任务 03 同款
    // 判据)。
    let outcome = shutdown(h).await;
    assert!(
        outcome.ticks_run >= 400,
        "慢客户端全程:tick 计数不减(实 {})",
        outcome.ticks_run
    );
}

// ---------------------------------------------------------------------------
// AC-N6 断线重连 + 双客户端隔离
// ---------------------------------------------------------------------------

/// AC-N6:A1/A2(同区域双客户端)与 B(远区)并发互不干扰(实体流按视距
/// 隔离);A1 断连 → 注册清理 + EOF + 观察者移除(票撤,评审修复②)→
/// 缩短 despawn 配置下分片回收;重连(新身份)→ ensure 复活分片、chunk 流
/// 与实体观察恢复;B 全程不受 A 域扰动。
#[tokio::test(start_paused = true)]
async fn acn6_disconnect_reconnect() {
    let mut config = base_config();
    config.despawn_after_secs = 0; // despawn 窗口 0(任务书工程细节:缩短配置)
    let h = harness(config, PumpConfig::default(), (2, 60), true).await;

    let (mut a1, mut rx_a1) = client(h.addr).await;
    let (mut a2, mut rx_a2) = client(h.addr).await;
    let (mut b, mut rx_b) = client(h.addr).await;
    tokio::task::yield_now().await;

    let id_a1 = join(&mut a1, &mut rx_a1, 1).await;
    let id_a2 = join(&mut a2, &mut rx_a2, 1).await;
    let id_b = join(&mut b, &mut rx_b, 1).await;
    assert_eq!(
        (id_a1, id_a2, id_b),
        (InputSource(1), InputSource(2), InputSource(3)),
        "身份 1/2/3 递增"
    );

    // 同区域双客户端 + 远区 B:观察者就位。
    for c in [&mut a1, &mut a2] {
        c.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, -58.0, 8.0),
        })
        .await
        .expect("A 域移动写入");
    }
    b.send(&C2S::MoveObserver {
        position: Vec3::new(1000.0, -58.0, 8.0),
    })
    .await
    .expect("B 域移动写入");
    advance_ticks(12).await; // 两片 ensure + 激活装载(Spawn 的先决)

    // 两域各一实体(经驱动命令通道注入,源 = 会话身份)。
    let e_a = eid(1);
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnDynamic {
            entity: e_a,
            state: EntityState {
                translation: Vec3::new(8.5, -55.0, 8.5),
                ..EntityState::default()
            },
            desc: EntityDesc::default(),
        },
    })
    .await
    .expect("驱动存活");
    let e_b = eid(2); // 出生 x=1000.5 → chunk 62 → 分片 (7,0)
    h.tx.send(DriverCommand::Send {
        shard: shard_id(7, 0),
        command: SimCommand::SpawnDynamic {
            entity: e_b,
            state: EntityState {
                translation: Vec3::new(1000.5, -55.0, 8.5),
                ..EntityState::default()
            },
            desc: EntityDesc::default(),
        },
    })
    .await
    .expect("驱动存活");
    advance_ticks(6).await;

    // 隔离:A1/A2 见 e_a 不见 e_b;B 反之(负样本 = 小预算窗内从未出现)。
    assert!(
        entity_appears(&mut rx_a1, e_a, 100).await,
        "A1 见本域实体 e_a"
    );
    assert!(
        entity_appears(&mut rx_a2, e_a, 100).await,
        "A2 见本域实体 e_a(同区域双客户端共存)"
    );
    assert!(
        !entity_appears(&mut rx_a1, e_b, 30).await,
        "A1 不得见 B 域实体(双客户端互不干扰)"
    );
    assert!(
        entity_appears(&mut rx_b, e_b, 100).await,
        "B 见本域实体 e_b"
    );
    assert!(
        !entity_appears(&mut rx_b, e_a, 30).await,
        "B 不得见 A 域实体(双客户端互不干扰)"
    );

    // 离场:A1/A2 双双 Goodbye(票撤由 Left → remove_observer 完成,
    // 评审修复②;A2 无需自挪)。
    a1.bye().await;
    a2.bye().await;
    advance_ticks(3).await;
    assert!(
        !h.hub.subscribers().contains(&id_a1) && !h.hub.subscribers().contains(&id_a2),
        "断连 → 双方出站注册清理"
    );
    // A1 读通道 EOF(reader 随对端关闭退出 → 发送端 drop;先吃掉缓冲帧)。
    let mut a1_eof = false;
    for _ in 0..8 {
        while try_frame(&mut rx_a1).is_some() {}
        if rx_a1.is_closed() {
            a1_eof = true;
            break;
        }
        advance_ticks(1).await;
    }
    assert!(a1_eof, "断连 → 客户端读通道 EOF");

    // A 域实体清场(观察者已随断连移除;despawn 前先移除片内实体——空片
    // 才是回收候选)→ 片 (0,0) 空 + 无票 + idle(窗口 0)→ 编排 ③ 回收。
    h.tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::RemoveEntity { entity: e_a },
    })
    .await
    .expect("驱动存活");
    advance_ticks(8).await;

    // 重连:新客户端 A2b(新身份 4)入同一区域 → ensure 复活分片、chunk 流
    // 恢复、实体观察恢复。
    let (mut a2b, mut rx_a2b) = client(h.addr).await;
    let id_a2b = join(&mut a2b, &mut rx_a2b, 1).await;
    assert_eq!(id_a2b, InputSource(4), "重连分配新身份(单调递增)");
    a2b.send(&C2S::MoveObserver {
        position: Vec3::new(8.0, -58.0, 8.0),
    })
    .await
    .expect("重连移动写入");
    advance_ticks(30).await; // re-ensure + 激活装载 + 泵流(9 chunk 一泵齐)
    let loads = collect_chunk_loads(&mut rx_a2b, 9, 20).await;
    let square0 = view_square(ChunkPos::new(0, 0), 1);
    assert!(
        !loads.is_empty() && loads.iter().all(|(p, _)| square0.contains(p)),
        "重连后 chunk 流恢复且在新观察中心方形内: {:?}",
        loads.iter().map(|(p, _)| *p).collect::<Vec<_>>()
    );
    // 实体观察恢复:重连者经协议生成新实体 → 可见(ID 由服务端分配)。
    let e = spawn_entity(&mut a2b, &mut rx_a2b, Vec3::new(8.5, -55.0, 8.5)).await;
    advance_ticks(4).await;
    assert!(
        entity_appears(&mut rx_a2b, e, 100).await,
        "重连后观察输出恢复(新实体可见)"
    );

    // B 不受 A 域断连/重连扰动:在场且仍能收帧(其区域数据在途/已缓冲)。
    assert!(!rx_b.is_closed(), "B 全程在场");
    let mut b_alive = false;
    for _ in 0..40 {
        if next_frame(&mut rx_b).await.is_some() {
            b_alive = true;
            break;
        }
    }
    assert!(b_alive, "B 仍能收帧(A 域生命周期事件零外溢)");

    let outcome = shutdown(h).await;
    // REQ-005 D5(任务 00 连带):观察者贴分片角落 → 邻片同拍 ensure;断连
    // 票撤后四分片(本片 + 三邻片)皆空+无票回收。原"恰 1"口径随邻片
    // ensure 更新;孤儿扩张副本由 owner despawn 一并撤销(不阻塞回收)。
    assert_eq!(
        outcome.shards_despawned, 4,
        "断连票撤(Left → remove_observer)→ 缩短窗口下角落四分片回收(邻片 ensure,REQ-005 D5)"
    );
    assert!(outcome.chunks_unloaded >= 1, "回收分片的 chunk 被卸载落库");
    assert!(outcome.shards_spawned >= 2, "初生 + 复活各一次 ensure(≥2)");
}
