//! biased select(关机>tick>命令)、MissedTickBehavior::Skip、checkpoint 节奏与断点续跑装配。
//!
//! 驱动形状照 ferrumc v2 driver(调研 §2):biased select 优先级 = 关机 > 到期
//! tick > 命令——命令洪水永远饿不死模拟;`MissedTickBehavior::Skip` 让超载降拍
//! 而不补帧。checkpoint = 快照拍 + 此后命令日志原子一体(README 决策 4),只存增量。

use std::collections::BTreeMap;
use std::time::Duration;

use serde::{Deserialize, Serialize};
use storage::traits::{CheckpointStore, ChunkKey};

use crate::config::{Config, ConfigError};
use crate::net::{NetUp, OutboundHub};
use sim_api::proto::S2C;
use types::glam::Vec3;
use types::{WORLD_Y_MAX, WORLD_Y_MIN};

/// 驱动指令(外部缝;REQ-003 网络将复用同一形状)。变体只许追加(声明序续在最后)。
/// `PartialEq` 手工实现(REQ-003 任务书 02:`StreamChunks` 的 oneshot 回执端
/// 无 PartialEq——按数据字段比较,`reply` 忽略)。
#[derive(Debug)]
pub enum DriverCommand {
    Send {
        shard: types::ShardId,
        command: messaging::SimCommand,
    },
    Shutdown,
    /// 观察者移动(REQ-002 §4.2/§4.4 命令化入口):经 `session.move_observer`
    /// 盖章入调度器专用收件箱 + 命令日志(回放确定性的硬要求)。
    MoveObserver {
        position: types::glam::Vec3,
    },
    /// 每玩家 chunk 流往返(REQ-003 §4.4;任务书 req003-net/02):连接层泵发来
    /// 视距差分清单,驱动 tick 末端编排第 ④ 步处理——load 批量 `provider.get`
    /// (持久层优先、生成器兜底已内建),unload 无模拟动作(chunk 流是纯视图:
    /// 观察者的模拟激活/票据由 REQ-002 观察者命令化处理),然后 oneshot 回执
    /// `(ChunkPos, Chunk)` 对。**请求在编排点处理(同拍或下一拍),永不阻塞
    /// tick**——本变体入挂起队列,不进 select 命令分支的同步处理。
    StreamChunks {
        load: Vec<types::ChunkPos>,
        unload: Vec<types::ChunkPos>,
        reply: tokio::sync::oneshot::Sender<Vec<(types::ChunkPos, voxel::Chunk)>>,
    },
}

impl PartialEq for DriverCommand {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (
                Self::Send {
                    shard: a,
                    command: b,
                },
                Self::Send {
                    shard: c,
                    command: d,
                },
            ) => a == c && b == d,
            (Self::Shutdown, Self::Shutdown) => true,
            (Self::MoveObserver { position: a }, Self::MoveObserver { position: b }) => a == b,
            (
                Self::StreamChunks {
                    load: la,
                    unload: ua,
                    ..
                },
                Self::StreamChunks {
                    load: lb,
                    unload: ub,
                    ..
                },
            ) => la == lb && ua == ub,
            _ => false,
        }
    }
}

/// 命令前置校验结果(REQ-003 评审修复①):`Ok(())` = 可发;`Err(reason)`
/// = 必被模拟侧 fail-stop 拒绝(丢弃不投——hostile 输入不得触发 fail-stop)。
type CommandGuard = Result<(), &'static str>;

/// 外部命令投递前的前置校验(REQ-003 §4.2「网络层永不直改模拟」+ README
/// 决策 4 翻译层一处收口的投递侧补全):路由换算可由客户端任意参数构造,
/// 不可达目标若照常投递会命中模拟/调度器的 fail-stop panic(未注册分片
/// `Scheduler::submit`、SetBlock 越界 `apply_command`、RemoveEntity/
/// TeleportEntity 未知实体 `despawn_entity`)→ 整机崩溃(DoS)。fail-stop
/// 语义保留给调用方 bug:经外部缝(网络/命令通道)进入的命令必须可预检,
/// 预检不过 = 静默丢弃(与既有反压计数同型的外部输入降级,只记录)。
///
/// 各变体的模拟侧权威判定对齐:
/// - 路由分片未注册 → 丢弃(调度器 `Scheduler::submit` 对未注册分片
///   fail-stop;ensure 只对观察者所在分片惰性建片,命令目标不在其列);
/// - `SetBlock` → ① y 界内(`WORLD_Y_MIN..=WORLD_Y_MAX`,与
///   `VoxelEdit::apply` / `VoxelWorld::set_block` 同界;x/z 无界——set_block
///   自动建 chunk);② owner 匹配:路由分片 == 由 `pos.chunk_of()` + 分片
///   world/dimension 组装的归属分片(REQ-005 §4.1.2,`Scheduler::owner_of`
///   同口径;不匹配的投递会命中广播校验 `SessionError::Rejected`,先拦);
///   ③ chunk 已加载(REQ-005 任务 01,语义细则 7:`VoxelWorld::set_block`
///   会自动建空气 chunk,之后生成器/持久化会把地形覆盖成空气——未加载
///   chunk 的外部 SetBlock 必须丢弃,不自动建片);
/// - `RemoveEntity` / `TeleportEntity` / `MarkActor` → 实体存在(按信息性
///   路由分片查 `entity_state`——`command_shard` 换算口径;迁移中的实体在
///   原 origin 分片表内,查询同键);
/// - `SpawnDynamic` → 分片在场即过(模拟侧无进一步 panic 路径);
/// - `Detonate` → **只做参数校验**且**先于**分片注册检查(REQ-005 任务 03)。
pub fn validate_external_command(
    session: &sim_api::session::Session,
    shard: types::ShardId,
    command: &messaging::SimCommand,
) -> CommandGuard {
    // Detonate(REQ-005 任务 03:**先于**分片注册检查):只做参数校验
    // (有限值/半径>0);目标集合由 `Session::detonate_from` 的
    // `shards_covering` 计算,爆心可在未注册区域(目标集可能为空 = 零条
    // 日志,合法)。`shard` 参数对本变体仅信息性(不再用于投递判定)。
    if let messaging::SimCommand::Detonate {
        center: _,
        power,
        radius,
    } = command
    {
        return if power.is_finite() && radius.is_finite() && *radius > 0.0 {
            Ok(())
        } else {
            Err("Detonate 参数非法(需有限 power/radius 且 radius > 0)")
        };
    }
    // 未注册分片:任何命令都不可投(Scheduler::submit panic = fail-stop)。
    // ensure(编排 ①)只对观察者所在分片惰性建片——命令目标分片不由驱动
    // 造,hostile 序号路由(SpawnDynamic 中心/origin 远离一切观察者)不可达。
    if !session.shard_ids().contains(&shard) {
        return Err("路由分片未注册");
    }
    match command {
        messaging::SimCommand::SetBlock { pos, .. } => {
            if !(WORLD_Y_MIN..=WORLD_Y_MAX).contains(&pos.y) {
                return Err("SetBlock y 越界");
            }
            // owner 匹配(REQ-005 §4.1.2):路由分片必须 == 归属分片
            // (world/dimension 取自分片身份,单世界单维度下恒等,仍按
            // 组装口径判——与 session 广播校验同源,hostile 不投)。
            let true_owner = shard::Scheduler::owner_of(shard, pos.chunk_of());
            if shard != true_owner {
                return Err("SetBlock owner 不匹配(非归属分片)");
            }
            // chunk 已加载(REQ-005 任务 01 语义细则 7):未加载 chunk 的
            // 外部 SetBlock 必须拒绝——set_block 会自动建空气 chunk,之后
            // 生成器/持久化会把地形覆盖成空气。
            if !session.is_chunk_loaded(shard, pos.chunk_of()) {
                return Err("SetBlock 目标 chunk 未加载");
            }
            Ok(())
        }
        messaging::SimCommand::RemoveEntity { entity }
        | messaging::SimCommand::TeleportEntity { entity, .. } => {
            if session.entity_state(shard, *entity).is_some() {
                Ok(())
            } else {
                Err("实体不存在(按路由分片)")
            }
        }
        messaging::SimCommand::MarkActor { entity, .. } => {
            if session.entity_state(shard, *entity).is_some() {
                Ok(())
            } else {
                Err("实体不存在(按路由分片)")
            }
        }
        messaging::SimCommand::SpawnDynamic { .. } => Ok(()),
        // Detonate 已在函数顶部先于分片注册检查处理(参数校验后直接 Ok/
        // Err 返回);此臂不可达,match 穷举仍需覆盖。
        messaging::SimCommand::Detonate { .. } => Ok(()),
        // 观察者命令(REQ-002 §4.2 / REQ-005 D9):SetObserverPosition 与
        // RemoveObserver 同为调度器专用观察者收件箱的载荷,不可经外部缝投递
        // (客户端白名单拒绝;network translate 同口径)。观察者移动的正路 =
        // `DriverCommand::MoveObserver` 专用包 → `session.move_observer`。
        // 经本缝到达 = 拒绝丢弃(hostile 路径不 panic)。
        messaging::SimCommand::SetObserverPosition { .. }
        | messaging::SimCommand::RemoveObserver => Err("观察者命令不可外部投递"),
    }
}

/// 外部命令的统一投递(REQ-005 §4.1.2 / 任务书 req005-consistency/01;
/// commands 通道与网络上行两条 Send 路径共用一处收口)。`source` 参数决定
/// 日志 source:网络路径(有玩家)传玩家身份(REQ-005 D9:日志 source 必须
/// 是玩家,不是会话 1),commands 通道(本地权威)传会话 source:
/// - `Detonate` → `session.detonate_from`(REQ-005 任务 03:目标 =
///   `shards_covering` 覆盖集;空目标集 = 零条日志合法)→ 全部入日志;
/// - `SetBlock` → `session.send_broadcast_from`(同拍广播 owner + holders,
///   每个目标一条 [`LogEntry`],整批原子)→ 全部入日志;
/// - 其余命令照旧单分片 `session.send_from` → 入日志;
/// - `Err(Rejected)`(owner 不匹配/不支持的广播命令)→ eprintln + 计数,
///   不断链(hostile 路径丢弃;先经 [`validate_external_command`] 预检,
///   正常不可达,防御层兜底);
/// - `Err(Route(InboxFull))` → 只计数,不断链(反压;Detonate 整批原子,
///   任一目标满 = 全拒也走此计数);
/// - 其它 Err → panic(非反压的会话层失败,fail-stop)。
fn dispatch_external(
    session: &mut sim_api::session::Session,
    log: &mut sim_api::log::CommandLog,
    commands_rejected: &mut u64,
    source: messaging::InputSource,
    shard: types::ShardId,
    command: messaging::SimCommand,
) {
    let result = match command {
        // 跨片爆炸(REQ-005 任务 03):目标集 = shards_covering;`shard`
        // 参数仅信息性(网络 translate 的爆心分片,广播不再用它)。
        messaging::SimCommand::Detonate {
            center,
            power,
            radius,
        } => session
            .detonate_from(source, center, power, radius)
            .map(|entries| {
                entries.iter().for_each(|entry| log.append(entry.clone()));
            }),
        messaging::SimCommand::SetBlock { .. } => {
            // 广播路径:返回全目标的 LogEntry(dest 升序,同盖章),逐条入日志。
            session
                .send_broadcast_from(source, shard, command)
                .map(|entries| {
                    entries.iter().for_each(|entry| log.append(entry.clone()));
                })
        }
        _ => {
            // 单发路径照旧:先缝后日志,失败即不入日志(决策 5)。
            session
                .send_from(source, shard, command)
                .map(|entry| log.append(entry))
        }
    };
    match result {
        Ok(()) => {}
        Err(sim_api::session::SessionError::Rejected(why)) => {
            eprintln!("[driver] 外部命令拒绝(广播校验):{why}");
            *commands_rejected += 1;
        }
        Err(sim_api::session::SessionError::Route(shard::router::RouteError::InboxFull {
            ..
        })) => {
            // 反压:只计数,不断链(与既有命令分支同款)。
            *commands_rejected += 1;
        }
        Err(other) => panic!("会话发送失败(非反压):{other:?}"),
    }
}

/// 驱动装配(全显式,无全局单例)。
pub struct DriverArgs {
    pub config: Config,
    pub session: sim_api::session::Session,
    pub store: storage::worker::StoreWorker,
    pub log: sim_api::log::CommandLog,
    pub commands: tokio::sync::mpsc::Receiver<DriverCommand>,
    /// 是否监听 ctrl_c 为关机源(测试置 false,README 决策 7)。
    pub listen_ctrl_c: bool,
    /// 世界 chunk 数据源(REQ-002 §4.3;bootstrap 装配:worker 通道 + 生成器
    /// (按 world_kind)+ seed)。tick 末端取数回填用。
    pub provider: storage::provider::ChunkProvider,
    /// 注册新分片的构造闭包(shard → ShardSim;REQ-002 §4.4 README 决策 6:
    /// ensure_shard 在 app 层,调度器不知生成器)。闭包捕获 shard_sim_config
    /// 与 registry 工厂。`Send`:run 作为 tokio 任务 spawn,跨 await 持有。
    pub new_sim: Box<dyn FnMut(types::ShardId) -> physics_sim::ShardSim + Send>,
    /// 观察输出缝(REQ-003 §4.4;任务书 req003-net/03):网络层上行事件接收
    /// 端 + 出站注册表下行缝 + 观察节奏。`None` = 无网络(观察输出编排整体
    /// 关闭,既有行为不变;main 装配在任务 04 接线前恒 None)。
    pub observe: Option<ObserveLink>,
}

/// 观察输出缝(REQ-003 §4.4;任务书 03 装配件):网络层 `serve` 的 up 接收端
/// 交给驱动(`try_recv` 在 tick 末端排空——不新增 select 臂,任务书细则 5),
/// [`OutboundHub`] 为每玩家 World 队列的下行填充缝。
pub struct ObserveLink {
    /// 网络上行事件(Joined/Subscribed/DriverCommand/Left);每拍末端排空。
    pub up: tokio::sync::mpsc::Receiver<NetUp>,
    /// 每玩家出站队列注册表(net 层 Subscribe 注册/断连注销)。
    pub hub: std::sync::Arc<OutboundHub>,
    /// 观察输出节奏(增量/全量周期,拍)。
    pub cadence: ObserveCadence,
}

/// 观察输出节奏(任务书 03 接口设计:NetConfig 增量的驱动侧投影;
/// 两周期独立计数)。
#[derive(Debug, Clone, Copy)]
pub struct ObserveCadence {
    /// 增量输出周期(拍;默认 10)。
    pub interval_ticks: u64,
    /// 全量输出周期(拍;默认 60;全量包 removed 恒空)。
    pub full_interval_ticks: u64,
}

impl Default for ObserveCadence {
    fn default() -> Self {
        Self {
            interval_ticks: 10,
            full_interval_ticks: 60,
        }
    }
}

impl From<&crate::net::NetConfig> for ObserveCadence {
    fn from(cfg: &crate::net::NetConfig) -> Self {
        Self {
            interval_ticks: cfg.observe_interval_ticks.max(1),
            full_interval_ticks: cfg.full_snapshot_interval_ticks.max(1),
        }
    }
}

/// 每玩家观察流状态(任务书 03 接口设计:断连即弃;玩家互不影响)。
struct PlayerStream {
    /// 距离过滤半径(米;Subscribe 钳制值 × 16,README 决策 8)。
    view_distance_m: f32,
    /// 上次发给该玩家的实体表快照(差分基准;实体消失 = 上轮有本轮无)。
    last_sent: BTreeMap<types::EntityId, types::EntityState>,
}
/// 驱动结果。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DriverOutcome {
    /// 本次运行的 tick 数。
    pub ticks_run: u64,
    /// 写入的检查点(含终末),按写入序。
    pub checkpoints_written: Vec<types::Tick>,
    /// 因收件箱满被拒的命令数(反压计数,只记录)。
    pub commands_rejected: u64,
    /// 退出时告警是否处于激活(§9.2 只度量)。
    pub alert_active_at_exit: bool,
    /// 驱动编排新建的分片数(REQ-002 §4.4 观察 ensure)。
    pub shards_spawned: u64,
    /// 驱动编排卸载(despawn)的分片数。
    pub shards_despawned: u64,
    /// 经 provider 装载的 chunk 数。
    pub chunks_loaded: u64,
    /// 卸载并入持久化批的 chunk 数(含 despawn 卸载)。
    pub chunks_unloaded: u64,
}

/// 检查点格式版本(REQ-005 D8;任务书 02):预发布阶段不兼容旧格式——
/// 升版即破格,磁盘上现存旧 checkpoint 从此不可用(bootstrap 显式报错,
/// 不写迁移逻辑)。`format_version != CHECKPOINT_FORMAT_VERSION` →
/// `AppError::Decode`(明确、不静默降级)。
pub const CHECKPOINT_FORMAT_VERSION: u32 = 1;

/// 检查点捆绑:格式版本 + 世界快照(分片 + 调度器状态)+ 此后命令日志,
/// 原子一体(README 决策 4;REQ-005 D7 世界快照,D8 版本字段)。
#[derive(Serialize, Deserialize)]
pub struct CheckpointBundle {
    pub format_version: u32,
    pub world: sim_api::snapshot::WorldSnapshot,
    pub log_bytes: Vec<u8>,
}

/// 驱动层可预期失败(类型化;蓝图 §4)。
#[derive(Debug, thiserror::Error)]
pub enum AppError {
    #[error("配置错误:{0}")]
    Config(#[from] crate::config::ConfigError),
    #[error("存储错误:{0}")]
    Store(#[from] storage::traits::StoreError),
    #[error("会话错误:{0}")]
    Session(#[from] sim_api::session::SessionError),
    #[error("检查点解码失败:{0}")]
    Decode(String),
}

/// bundle → bincode 字节(standard 配置;全字段可序列化,不可能失败 → fail-stop)。
fn encode(bundle: &CheckpointBundle) -> Vec<u8> {
    bincode::serde::encode_to_vec(bundle, bincode::config::standard())
        .expect("检查点捆绑编码不可能失败(全字段可序列化,fail-stop)")
}

/// bincode 字节 → bundle(REQ-005 D8):
/// - 解码失败(含任务 01 前旧布局 `{snapshot, log_bytes}` 的字节——无版本
///   字段,结构不同自然解不开)→ `Err(AppError::Decode)`;
/// - 解码成功但 `format_version != CHECKPOINT_FORMAT_VERSION` →
///   `Err(AppError::Decode)` 带版本说明(不兼容旧格式,不静默降级)。
fn decode(bytes: &[u8]) -> Result<CheckpointBundle, AppError> {
    let (bundle, _len): (CheckpointBundle, usize) =
        bincode::serde::decode_from_slice(bytes, bincode::config::standard())
            .map_err(|e| AppError::Decode(e.to_string()))?;
    if bundle.format_version != CHECKPOINT_FORMAT_VERSION {
        return Err(AppError::Decode(format!(
            "unsupported checkpoint format version {}, expected {}(REQ-005 D8:不兼容旧格式;\
             请清空 data_dir 后重开)",
            bundle.format_version, CHECKPOINT_FORMAT_VERSION
        )));
    }
    Ok(bundle)
}

/// 检查点节奏:`ticks_run % interval == 0` → bundle = {world_snapshot(),
/// log.take() 的 to_bytes} → `append_checkpoint`(fire-and-forget,决策 5);
/// log.take() 重置后日志只存增量(崩溃丢失窗口 = checkpoint 间隔,§9.11)。
/// 期中与终末同构,差别只在 fire-and-forget 与否。
///
/// **原子性(REQ-005 任务书 02 风险备注)**:`world_snapshot()` 必须先于
/// `log.take()`,且两者之间不得插入任何 `session` 调用——同一拍边界的
/// 世界状态与日志窗口才互相一致(先 world 后 log:log 的窗口恰好是
/// snapshot.tick 之后的命令,与 replay_world 的 `>=` 重放窗对齐)。
fn write_checkpoint(
    session: &sim_api::session::Session,
    log: &mut sim_api::log::CommandLog,
    store: &storage::worker::StoreWorker,
    checkpoints_written: &mut Vec<types::Tick>,
) {
    let world = session.world_snapshot();
    let taken = log.take();
    let bundle = CheckpointBundle {
        format_version: CHECKPOINT_FORMAT_VERSION,
        world,
        log_bytes: encode_log(taken),
    };
    let tick = bundle.world.shards.tick;
    store.append_checkpoint(tick, encode(&bundle));
    checkpoints_written.push(tick);
}

/// 日志条目集 → bincode 字节(与 `CommandLog::to_bytes` 同构:重建临时日志整体
/// 编码,标准配置,不可能失败 → fail-stop)。
fn encode_log(entries: Vec<sim_api::log::LogEntry>) -> Vec<u8> {
    let mut log = sim_api::log::CommandLog::new();
    for entry in entries {
        log.append(entry);
    }
    log.to_bytes()
}

/// tick 末端编排累积的计数字段(每拍清零后由 [`orchestrate`] 填充)。
#[derive(Debug, Default, Clone, Copy)]
struct OrchestrationCounters {
    shards_spawned: u64,
    shards_despawned: u64,
    chunks_loaded: u64,
    chunks_unloaded: u64,
}

/// 驱动持有的挂起 chunk 流请求(REQ-003 §4.4:网络侧经 NetUp 上行后由驱动
/// 持有;每拍末端 drain,oneshot 回执——永不阻塞 tick)。
#[derive(Debug, Default)]
struct PendingStreams {
    queue: std::collections::VecDeque<DriverCommand>,
}

impl PendingStreams {
    fn push(&mut self, cmd: DriverCommand) {
        self.queue.push_back(cmd);
    }

    /// 取走全部挂起请求(取走即清;同拍编排点处理)。
    fn drain(&mut self) -> std::collections::vec_deque::Drain<'_, DriverCommand> {
        self.queue.drain(..)
    }
}

// ---- 观察输出编排(REQ-003 §4.4;任务书 req003-net/03) ----

/// 驱动侧观察输出编排器(REQ-003 §4.4;每驱动一个,observe 关闭时不建):
/// 每订阅玩家的观察流状态 + 增量/全量两周期计数 + 跨拍积压(Welcome 拍号、
/// 方块变更源)。玩家生命周期(Join/Leave/换视距)由网络上行事件驱动。
#[derive(Default)]
struct ObserveOrchestrator {
    /// 订阅玩家 → 观察流状态(断连即弃,任务书细则 3)。
    players: BTreeMap<messaging::InputSource, PlayerStream>,
    /// 增量周期计数(拍;周期到即输出并清零——两周期独立,细则 5)。
    ticks_since_interval: u64,
    /// 全量周期计数(拍)。
    ticks_since_full: u64,
    /// Join 未带 Welcome 的玩家 → 欢迎拍(下一个输出点入 Critical,细则 5)。
    pending_welcome: BTreeMap<messaging::InputSource, types::Tick>,
    /// 跨拍积压的方块变更源(格 → 写入值;后写覆盖先写)。
    pending_blocks: BTreeMap<types::BlockPos, types::BlockId>,
    /// 节奏副本(输出点用)。
    _cadence: ObserveCadence,
}

impl ObserveOrchestrator {
    fn new(cadence: ObserveCadence) -> Self {
        Self {
            players: BTreeMap::new(),
            ticks_since_interval: 0,
            ticks_since_full: 0,
            pending_welcome: BTreeMap::new(),
            pending_blocks: BTreeMap::new(),
            _cadence: cadence,
        }
    }

    /// 网络上行事件消费(任务书 03 编排 ①②;REQ-005 D9 观察者命令入日志)。
    ///
    /// - `Joined` → 登记欢迎拍(Welcome 下个输出点入 Critical);
    /// - `Subscribed` → 距离过滤半径更新(钳制值 × 16,README 决策 8);
    /// - `Left` → 观察流清理(断连即弃,last_sent 同弃,细则 3)+
    ///   `session.remove_observer_from(player)` + `log.append`(断线移除入
    ///   日志 → 可回放,无幽灵观察者;幂等,不在表无害);
    /// - `DriverCommand::MoveObserver` → `session.move_observer_from(player,
    ///   position)` + `log.append`(D9:盖章 source = 玩家,sequence 沿会话
    ///   单调——删除了旧 `player_sequences` 表与手工 `StampedInput` 构造;
    ///   收件箱 unbounded,无失败路径);
    /// - `DriverCommand::Send` → 前置校验后 `dispatch_external`(source =
    ///   玩家;SetBlock 走 `send_broadcast_from` 全部条目入日志,其余单发入
    ///   日志);SetBlock 成功镜像入 BlockUpdates 源(`record_block_changes`
    ///   统一从 `take_block_updates` 取,无需在此镜像)。
    fn consume_net_up(
        &mut self,
        event: NetUp,
        session: &mut sim_api::session::Session,
        log: &mut sim_api::log::CommandLog,
        commands_rejected: &mut u64,
    ) {
        match event {
            NetUp::Joined { player } => {
                self.pending_welcome.insert(player, session.tick_now());
            }
            NetUp::Subscribed {
                player,
                view_distance,
            } => {
                let stream = self.players.entry(player).or_insert_with(|| PlayerStream {
                    view_distance_m: 0.0,
                    last_sent: BTreeMap::new(),
                });
                stream.view_distance_m = f32::from(view_distance) * 16.0;
            }
            NetUp::Left { player, .. } => {
                // 断连即弃(细则 3):观察流状态清理 + 模拟侧观察者移除入日志
                // (REQ-005 D9:remove_observer_from 盖章玩家 source + 会话
                // 单调 sequence,log.append → 可回放;幂等——不在表无害)。
                self.players.remove(&player);
                self.pending_welcome.remove(&player);
                match session.remove_observer_from(player) {
                    Ok(entry) => log.append(entry),
                    // 观察者收件箱 unbounded → 无失败路径;非预期错误 fail-stop。
                    Err(other) => panic!("观察者移除失败(非预期):{other:?}"),
                }
            }
            NetUp::DriverCommand { player, cmd } => match cmd {
                DriverCommand::MoveObserver { position } => {
                    // 观察者移动入日志(REQ-005 D9):盖章 source = 玩家、
                    // sequence 沿会话单调;返回 LogEntry 直接 append。
                    // 收件箱 unbounded(REQ-002 细则 1),无失败路径;
                    // 迟到不可能(盖当前拍)。
                    match session.move_observer_from(player, position) {
                        Ok(entry) => log.append(entry),
                        Err(other) => panic!("观察者移动失败(非预期):{other:?}"),
                    }
                }
                DriverCommand::Send { shard, command } => {
                    // 前置校验(评审修复①):网络上行的命令参数全部客户端可
                    // 构造,预检不过 = 丢弃不投——hostile 输入不得触发调度器/
                    // 模拟的 fail-stop panic(整机崩溃 DoS);计数不断链。
                    if let Err(why) = validate_external_command(session, shard, &command) {
                        eprintln!("[driver] 网络命令丢弃(前置校验):{why} ({command:?})");
                        *commands_rejected += 1;
                        return;
                    }
                    // 统一投递收口(REQ-005 D9):source = 玩家——SetBlock 走
                    // 广播(每目标一条 LogEntry 全入日志),其余单发入日志;
                    // 反压/拒绝计数不断链。
                    dispatch_external(session, log, commands_rejected, player, shard, command);
                }
                DriverCommand::Shutdown | DriverCommand::StreamChunks { .. } => {
                    // 网络上行不含这两类(关机走 commands 通道;chunk 流回执
                    // 由驱动直接持有 oneshot)——出现即编程错误,fail-stop。
                    panic!("网络上行出现非观察面指令:一致性违规(fail-stop)");
                }
            },
        }
    }

    /// 方块变更源积累(REQ-005 D11:统一从 `session.take_block_updates()` 取
    /// 外部 SetBlock 的 owner 写入——不再镜像驱动侧发送;后接
    /// `take_solidified()` 凝固写入,本任务保留,任务 03 迁 Emergent)。
    /// 后写覆盖先写(格位键;同格凝固覆盖外部);下一个输出点按视距覆盖
    /// 广播(单包合并)后清空。
    fn record_block_changes(&mut self, session: &mut sim_api::session::Session) {
        // (shard, 格, 值):单世界单维度,BlockPos 即全局格位,shard 仅信息性。
        for (_shard, cell, block) in session.take_block_updates() {
            self.pending_blocks.insert(cell, block);
        }
        for (_shard, cell, block) in session.take_solidified() {
            self.pending_blocks.insert(cell, block);
        }
    }

    /// 输出点(任务书 03 编排 ③④⑤):增量/全量周期到 → 对每订阅玩家
    /// (hub 注册表 = 活连接):收集各分片 entity_states → 3D 距离过滤
    /// (≤ view_distance×16,观察者表最新位置,细则 1)→ 与 last_sent 差分
    /// → EntityStates 入 World 队列 → 更新 last_sent;全量周期(或首包)整表
    /// removed 空。方块变更按格位视距覆盖广播(单包合并,README 决策 9)。
    /// Welcome 入 Critical。
    ///
    /// 丢包自愈(细则 4):World 满丢最旧对 EntityStates 无损正确性
    /// (latest-wins,下轮全量周期整表覆盖)。
    fn emit_outputs(
        &mut self,
        session: &sim_api::session::Session,
        hub: &OutboundHub,
        tick: types::Tick,
        cadence: ObserveCadence,
    ) {
        // 周期判定(两周期独立计数,细则 5)。
        self.ticks_since_interval += 1;
        self.ticks_since_full += 1;
        let incremental_due = self.ticks_since_interval >= cadence.interval_ticks;
        let full_due = self.ticks_since_full >= cadence.full_interval_ticks;
        if !incremental_due && !full_due {
            return;
        }
        if incremental_due {
            self.ticks_since_interval = 0;
        }
        if full_due {
            self.ticks_since_full = 0;
        }

        // Welcome(Joined 登记)→ Critical(细则 ⑤;满 → CriticalFull →
        // 连接侧断连收口,驱动侧注册消失后自然弃,无阻塞)。
        for (player, welcome_tick) in std::mem::take(&mut self.pending_welcome) {
            hub.push_critical(
                player,
                S2C::Welcome {
                    player_id: player,
                    tick: welcome_tick,
                },
            );
        }

        // 观察者表(驱动可见的最新位置)与分片全实体表(每分片取一次)。
        let observers: BTreeMap<messaging::InputSource, Vec3> =
            session.observers().into_iter().collect();
        let mut tables: BTreeMap<types::ShardId, Vec<(types::EntityId, types::EntityState)>> =
            BTreeMap::new();
        for shard in session.shard_ids() {
            if let Some(table) = session.entity_states(shard) {
                tables.insert(shard, table);
            }
        }

        // 实体状态:对每订阅玩家差分(全量周期到 或 首包 → 整表)。
        for player in hub.subscribers() {
            let Some(stream) = self.players.get_mut(&player) else {
                continue; // Subscribed 竞先窗口:下个输出点自然补
            };
            let Some(&position) = observers.get(&player) else {
                continue; // 未 MoveObserver:无过滤中心,静默等待(0 视距语义)
            };
            let full = full_due || stream.last_sent.is_empty();
            let radius = stream.view_distance_m;

            // 视距内实体(3D 距离,≤ view_distance×16;细则 1)。
            let mut visible: BTreeMap<types::EntityId, types::EntityState> = BTreeMap::new();
            for table in tables.values() {
                for (id, state) in table {
                    if position.distance(state.translation) <= radius {
                        visible.insert(*id, *state);
                    }
                }
            }

            let mut states: Vec<(types::EntityId, types::EntityState)> = Vec::new();
            let mut removed: Vec<types::EntityId> = Vec::new();
            if full {
                states = visible.iter().map(|(k, v)| (*k, *v)).collect();
            } else {
                // 增量:变化集(新出现或状态变化)+ 消失集(上轮有本轮无——
                // 迁移/凝固/移除三源同型,细则 2)。
                for (id, state) in &visible {
                    match stream.last_sent.get(id) {
                        Some(prev) if prev == state => {}
                        _ => states.push((*id, *state)),
                    }
                }
                for id in stream.last_sent.keys() {
                    if !visible.contains_key(id) {
                        removed.push(*id);
                    }
                }
            }
            stream.last_sent = visible;
            if states.is_empty() && removed.is_empty() {
                continue; // 无变化不发包(增量语义)
            }
            hub.push_world(
                player,
                S2C::EntityStates {
                    tick,
                    states,
                    removed,
                    full,
                },
            );
        }

        // BlockUpdates:按格位视距覆盖广播(单包合并,README 决策 9;格位
        // 中心到玩家 ≤ 过滤半径)。未移动过的玩家(无中心)静默等待。
        let blocks = std::mem::take(&mut self.pending_blocks);
        if !blocks.is_empty() {
            for player in hub.subscribers() {
                let Some(stream) = self.players.get(&player) else {
                    continue;
                };
                let Some(&position) = observers.get(&player) else {
                    continue;
                };
                let updates: Vec<(types::BlockPos, types::BlockId)> = blocks
                    .iter()
                    .filter(|&(pos, _)| {
                        let center =
                            Vec3::new(pos.x as f32 + 0.5, pos.y as f32 + 0.5, pos.z as f32 + 0.5);
                        position.distance(center) <= stream.view_distance_m
                    })
                    .map(|(pos, block)| (*pos, *block))
                    .collect();
                if updates.is_empty() {
                    continue;
                }
                hub.push_world(player, S2C::BlockUpdates { updates });
            }
        }
    }
}

/// tick 末端编排(REQ-002 §4.4/§4.5;README 决策 8,顺序固定):
///
/// - **观察者分片 ensure**:`observers()` → 所在 shard 未注册 → `new_sim(shard)`
///   + `register`(观察者可走进未注册区域,惰性创建 = 世界自己长出来);
/// - **激活请求装载**:本拍 `take_activation_requests` → 装载 `provider.get(key)`
///   → `apply_chunk_load`;卸载 `apply_chunk_unload` → chunk 累积持久化批,
///   批后 `put_chunks`(fire-and-forget,屏障语义见 despawn 条);
/// - **despawn 候选**:`take_despawn_requests` → 逐个 `unload_shard` → chunks
///   并入持久化批 → **批后一次 flush + recv ack(屏障)**,粘滞错误 panic
///   沿用(与退出路径同款 fail-stop);
/// - **chunk 流请求**(REQ-003 §4.4,任务书 req003-net/02):drain 挂起的
///   `StreamChunks`(网络泵上行、驱动持有)→ load 批量 `provider.get` →
///   oneshot 回执 `(ChunkPos, Chunk)` 对;unload 无模拟动作(纯视图)。
///   回执发送端已亡(连接已断)= 请求作废,取数结果直接弃(不持久化、不
///   报错——数据源只读,无状态泄漏)。
/// - **计数**:交还调用方入 outcome。
///
/// 屏障取舍(REQ-002 语义细则 2):常规卸载的 chunk 数据在本拍 fire-and-forget
/// 入队,不单独等——与 checkpoint 同窗口语义(§9.11,终末 flush 兜底);
/// 只有 despawn 批后强制 flush 屏障(分片即将消失,数据必须确定性落库)。
/// 无观察者且从未设置过观察者时,激活请求恒空、ensure/despawn 无候选 → 零动作
/// (向后兼容开关,既有测试行为不变)。
fn orchestrate(
    session: &mut sim_api::session::Session,
    store: &storage::worker::StoreWorker,
    provider: &storage::provider::ChunkProvider,
    new_sim: &mut dyn FnMut(types::ShardId) -> physics_sim::ShardSim,
    pending_streams: &mut PendingStreams,
    ensure_radius: f32,
    counters: &mut OrchestrationCounters,
) {
    // ① 观察者分片 + 邻片 ensure(REQ-005 D5):观察者核心区距离
    // `ensure_radius`(= awake_unload_radius,默认 32m)内的所有分片都
    // ensure——醒着实体的迁移目的地/扩张副本的 owner 必在其中(醒着要求
    // 观察者 24m 内,32m 有裕度);同拍去重,计数并入 shards_spawned。
    let mut ensured: std::collections::BTreeSet<types::ShardId> = std::collections::BTreeSet::new();
    for (_source, position) in session.observers() {
        for shard in shards_within(position, ensure_radius) {
            if ensured.insert(shard) && !session.shard_ids().contains(&shard) {
                session.register(new_sim(shard));
                counters.shards_spawned += 1;
            }
        }
    }

    // ①′ 迁移目的地 ensure(REQ-005 D5/§4.1.4):Router 待投递信封的目的
    // 分片逐个 ensure——保证下一拍 tick() 开头的「信封黑洞」不变量不触发
    // (目的分片已注册)。观察者邻片 ensure 覆盖常规路径,此处是兜底
    // (队列里可能沉有早于观察者移动产生的信封)。
    for dest in session.pending_shard_destinations() {
        if ensured.insert(dest) && !session.shard_ids().contains(&dest) {
            session.register(new_sim(dest));
            counters.shards_spawned += 1;
        }
    }

    // ② 激活请求 → provider 取数回填 / 卸载累积持久化批。
    //    BlockWrite 卸载保序(REQ-005 任务 03 终审修复):Emergent 写入的信封
    //    apply_at = N+1,接收方在 N 与 N+1 之间卸载目标 chunk 会让 dispatch
    //    侧的装载断言 fail-stop——这是编排时序竞态,不是装配 bug。命中
    //    pending 目标格的卸载延迟一轮(信封应用后下一轮编排正常卸载)。
    let deferred_blockwrites = session.pending_blockwrite_cells();
    let mut put_batch: Vec<(ChunkKey, voxel::Chunk)> = Vec::new();
    for (shard, decisions) in session.take_activation_requests() {
        for pos in decisions.chunks_to_load {
            let key = ChunkKey {
                world: shard.world,
                dim: shard.dimension,
                pos,
            };
            // 生成器兜底后不应 None——None = 生成器实现缺陷 → fail-stop(细则 1)。
            let chunk = provider
                .get(key)
                .unwrap_or_else(|| panic!("provider.get 返回 None(生成器缺陷,fail-stop):{key:?}"));
            session.apply_chunk_load(shard, pos, chunk);
            counters.chunks_loaded += 1;
        }
        for pos in decisions.chunks_to_unload {
            if deferred_blockwrites
                .get(&shard)
                .is_some_and(|chunks| chunks.contains(&pos))
            {
                continue;
            }
            if let Some(chunk) = session.apply_chunk_unload(shard, pos) {
                put_batch.push((
                    ChunkKey {
                        world: shard.world,
                        dim: shard.dimension,
                        pos,
                    },
                    chunk,
                ));
                counters.chunks_unloaded += 1;
            }
        }
    }

    // ③ despawn 候选 → unload_shard → chunks 并批 → 一次 flush 屏障。
    //    在途信封目的分片不 despawn(防御:despawn 窗口远大于一拍,正常
    //    不可达;若发生,下一拍信封黑洞校验会 fail-stop——先延迟更稳)。
    let pending_destinations = session.pending_shard_destinations();
    let mut despawn_batch: Vec<(ChunkKey, voxel::Chunk)> = Vec::new();
    for shard in session.take_despawn_requests() {
        if !session.shard_ids().contains(&shard) {
            // 候选只记录不清,重复取会重复返回:已卸载的跳过(幂等)。
            continue;
        }
        if pending_destinations.contains(&shard) {
            continue;
        }
        for (pos, chunk) in session.unload_shard(shard) {
            despawn_batch.push((
                ChunkKey {
                    world: shard.world,
                    dim: shard.dimension,
                    pos,
                },
                chunk,
            ));
            counters.chunks_unloaded += 1;
        }
        counters.shards_despawned += 1;
    }
    if !despawn_batch.is_empty() {
        put_batch.extend(despawn_batch);
    }
    if !put_batch.is_empty() {
        store.put_chunks(put_batch);
    }
    if counters.shards_despawned > 0 {
        // 屏障:despawn 批(连同本拍常规卸载批)落库确认;粘滞错误 panic
        // 沿用退出路径同款 fail-stop。
        let ack = store.flush();
        let flush_result = ack
            .recv()
            .expect("storage-writer 在 despawn flush 前死亡:一致性违规(fail-stop)");
        if let Err(e) = flush_result {
            panic!("存储粘滞错误(fail-stop 停机):{e}");
        }
    }

    // ④ chunk 流请求(REQ-003 §4.4):批量取数 + oneshot 回执(同拍编排点;
    // unload 无模拟动作——chunk 流是纯视图,模拟侧装载/票据由观察者命令化
    // 负责)。provider.get 毫秒级(REQ-002 §4.3),16 chunk/泵批处理可接受。
    for cmd in pending_streams.drain() {
        let DriverCommand::StreamChunks {
            load,
            unload: _,
            reply,
        } = cmd
        else {
            // 队列只收 StreamChunks(见命令分支);误入 = 编程错误,fail-stop。
            panic!("挂起流队列出现非 StreamChunks 指令:一致性违规(fail-stop)");
        };
        let mut chunks = Vec::with_capacity(load.len());
        for pos in load {
            // 世界/维度:进程单世界单维度(v1 共识;shard_of_position 同款取值)。
            let key = ChunkKey {
                world: types::WorldId(0),
                dim: types::DimensionId(0),
                pos,
            };
            // 生成器兜底后不应 None——None = 生成器实现缺陷 → fail-stop(同 ②)。
            let chunk = provider.get(key).unwrap_or_else(|| {
                panic!("provider.get 返回 None(生成器实现缺陷,fail-stop):{key:?}")
            });
            counters.chunks_loaded += 1;
            chunks.push((pos, chunk));
        }
        // 回执:接收端亡(连接已断)= 请求作废,结果弃(数据源只读,无泄漏)。
        let _ = reply.send(chunks);
    }
}

/// 位置 → 所在分片(与 `Session::move_observer` 的 dest 换算同口径:
/// chunk_of → shard_of,floor 语义)。
/// 位置 `p` 核心区 AABB 距离 ≤ `radius` 的全部分片(水平 x/z 口径;
/// REQ-005 D5 邻片 ensure 用)。分片边长 128m → radius < 128 时至多 4 个。
fn shards_within(p: types::glam::Vec3, radius: f32) -> Vec<types::ShardId> {
    const SHARD_M: f32 = (types::SHARD_CHUNKS * types::CHUNK_SIZE) as f32;
    let min_x = ((p.x - radius) / SHARD_M).floor() as i32;
    let max_x = ((p.x + radius) / SHARD_M).floor() as i32;
    let min_z = ((p.z - radius) / SHARD_M).floor() as i32;
    let max_z = ((p.z + radius) / SHARD_M).floor() as i32;
    let mut out = Vec::new();
    for sx in min_x..=max_x {
        for sz in min_z..=max_z {
            // 点到该分片核心 AABB 的水平距离(每轴 clamp 到盒外量)。
            let (cx, cz) = (sx as f32 * SHARD_M, sz as f32 * SHARD_M);
            let dx = (cx - p.x).max(0.0).max(p.x - (cx + SHARD_M));
            let dz = (cz - p.z).max(0.0).max(p.z - (cz + SHARD_M));
            if (dx * dx + dz * dz).sqrt() <= radius {
                out.push(types::ShardId::new(
                    types::WorldId(0),
                    types::DimensionId(0),
                    types::ShardPos::new(sx, sz),
                ));
            }
        }
    }
    out
}

/// 驱动主循环(biased:关机 > tick > 命令;决策 2/3/5)。
/// 正常与关机路径都执行:终末 checkpoint → append → flush → recv ack →
/// session.shutdown → store.shutdown;返回 outcome。
///
/// 分支序(poll 优先级,README 决策 2「关机 > tick > 命令」照 v2 driver):
/// 1. 关机(ctrl_c 仅当 `listen_ctrl_c`);
/// 2. 到期 tick——排在命令前,命令洪水挤不掉模拟;
/// 3. 命令(经 commands 通道:Shutdown 与通道关闭同列)。
///    同刻就绪时 tick 先于命令执行,该命令盖章 = 已推进的拍号。
pub async fn run(args: DriverArgs) -> DriverOutcome {
    let DriverArgs {
        config,
        mut session,
        store,
        mut log,
        mut commands,
        listen_ctrl_c,
        provider,
        mut new_sim,
        observe,
    } = args;
    // 会话 source(commands 通道本地权威命令的日志 source;网络路径用玩家,
    // REQ-005 D9)。
    let session_source = config.session_config().source;

    let mut ticker = tokio::time::interval(Duration::from_secs_f32(config.dt));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    // tokio interval 首枚 tick 立即到期:构造后先吃掉再进 select,否则首拍连跳
    // (README 决策 2)。
    ticker.tick().await;

    // 度量告警(§9.2):阈值与滑窗来自 config(5ms × 60 tick);只记录不决策。
    let mut telemetry = crate::telemetry::TickTelemetry::new(config.shard_alert_ms, 60);

    let mut ticks_run: u64 = 0;
    let mut commands_rejected: u64 = 0;
    let mut checkpoints_written: Vec<types::Tick> = Vec::new();
    // 编排计数(REQ-002 §4.4:DriverOutcome 增计数字段)。
    let mut shards_spawned: u64 = 0;
    let mut shards_despawned: u64 = 0;
    let mut chunks_loaded: u64 = 0;
    let mut chunks_unloaded: u64 = 0;
    // 上次 checkpoint 时的 ticks_run(相对本次运行;bootstrap 的检查点在起点之前,
    // 故初值 0——起跑后任何 tick 都是"自上次 checkpoint 后的增量")。
    let mut ticks_at_last_checkpoint: u64 = 0;
    // 挂起 chunk 流请求(REQ-003 §4.4:命令分支收下、tick 末端编排第 ④ 步
    // 处理——同拍或下一拍,永不阻塞 tick)。
    let mut pending_streams = PendingStreams::default();
    // 邻片 ensure 半径(REQ-005 D5 = awake_unload_radius,默认 32m)。
    let ensure_radius = config.activation.awake_unload_radius as f32;
    // 观察输出编排(REQ-003 §4.4;任务书 03):observe 缝在场才建(tick 末端
    // 第 ⑤ 步);None = 无网络,编排整体关闭(既有行为不变)。
    let mut observe_state = observe.map(|link| {
        (
            ObserveOrchestrator::new(link.cadence),
            link.up,
            link.hub,
            link.cadence,
        )
    });

    // ctrl_c 关机源(仅当 listen_ctrl_c;测试关掉,否则暂停时间测试被信号干扰)。
    let shutdown_signal = async {
        if listen_ctrl_c {
            let _ = tokio::signal::ctrl_c().await;
        } else {
            std::future::pending::<()>().await;
        }
    };
    tokio::pin!(shutdown_signal);

    loop {
        tokio::select! {
            biased;

            // 关机分支(优先级最高):ctrl_c 信号。
            _ = &mut shutdown_signal => break,

            // tick 分支(优先级次之):到期才执行,迟到即 Skip 不补帧。排在命令
            // 之前(biased 书写序 = poll 序):命令洪水(恒就绪的 recv)挤不掉到期
            // 的 tick——命令洪水饿不死模拟(v2 driver 语义,README 决策 2)。
            _ = ticker.tick() => {
                // 计时(§9.2):std Instant 属 app::telemetry 授权面(clippy.toml
                // 顶部注释);喂 telemetry 的只有现成 Duration。
                #[allow(clippy::disallowed_methods)]
                let t0 = std::time::Instant::now();
                // session.tick() 为阻塞调用,直接在 async loop 内执行(毫秒级、
                // 单装配单驱动;REQ-003 网络落地时再 spawn_blocking 化,README 决策 3)。
                // TODO(REQ-003):tick() 移入 spawn_blocking,消掉驱动的阻塞段。
                let _summary = session.tick();
                telemetry.record(t0.elapsed());
                ticks_run += 1;

                // tick 末端编排(REQ-002 §4.4/§4.5,顺序固定 ①②③④):
                // ensure 观察者分片 → 激活请求取数回填/卸载 → despawn 候选 +
                // 持久化屏障 → chunk 流请求取数回执。无观察者且无挂起流请求时
                // 零动作(向后兼容)。
                let mut counters = OrchestrationCounters::default();
                orchestrate(
                    &mut session,
                    &store,
                    &provider,
                    &mut new_sim,
                    &mut pending_streams,
                    ensure_radius,
                    &mut counters,
                );
                shards_spawned += counters.shards_spawned;
                shards_despawned += counters.shards_despawned;
                chunks_loaded += counters.chunks_loaded;
                chunks_unloaded += counters.chunks_unloaded;

                // 第 ⑤ 步(任务书 03):观察输出编排——网络上行事件消费 →
                // BlockUpdates 源积累 → 输出点(实体差分/全量、方块广播、
                // Welcome)。try_recv 排空(不新增 select 臂,细则 5);
                // observe 关闭时整段跳过(既有行为不变)。
                if let Some((orchestrator, up, hub, cadence)) = observe_state.as_mut() {
                    while let Ok(event) = up.try_recv() {
                        // chunk 流请求(任务 02 泵上行,NetUp::DriverCommand
                        // 载体)→ 挂起队列,编排 ④ 处理(既有路径);其余
                        // 事件 → 观察编排(任务 03)。
                        if matches!(
                            &event,
                            NetUp::DriverCommand {
                                cmd: DriverCommand::StreamChunks { .. },
                                ..
                            }
                        ) {
                            let NetUp::DriverCommand { cmd, .. } = event else {
                                unreachable!("上方 matches 已判定")
                            };
                            pending_streams.push(cmd);
                        } else {
                            orchestrator.consume_net_up(
                                event,
                                &mut session,
                                &mut log,
                                &mut commands_rejected,
                            );
                        }
                    }
                    orchestrator.record_block_changes(&mut session);
                    orchestrator.emit_outputs(&session, hub, session.tick_now(), *cadence);
                }

                // checkpoint 节奏:整间隔倍数拍到期(fire-and-forget,决策 5)。
                if ticks_run.is_multiple_of(config.checkpoint_interval_ticks) {
                    write_checkpoint(&session, &mut log, &store, &mut checkpoints_written);
                    ticks_at_last_checkpoint = ticks_run;
                }
            }

            // 命令分支(优先级最低):Shutdown 指令 / 命令通道关闭。
            maybe_command = commands.recv() => {
                match maybe_command {
                    Some(DriverCommand::Shutdown) | None => break,
                    Some(DriverCommand::Send { shard, command }) => {
                        // 前置校验(评审修复①):预检不过 = 丢弃不投——hostile
                        // 输入不得触发调度器/模拟的 fail-stop panic(DoS);
                        // 只记录,不断链(与反压计数同型的外部输入降级)。
                        if let Err(why) = validate_external_command(&session, shard, &command) {
                            eprintln!("[driver] 外部命令丢弃(前置校验):{why} ({command:?})");
                            commands_rejected += 1;
                            continue;
                        }
                        // 统一投递收口(REQ-005;先缝后日志由收口内保证,失败
                        // 即不入日志):SetBlock 广播全目标入日志,其余单发;
                        // 反压/拒绝只计数,不断链(任务书语义细则 2)。source
                        // = 会话(本地权威 commands 通道,无玩家身份)。
                        dispatch_external(
                            &mut session,
                            &mut log,
                            &mut commands_rejected,
                            session_source,
                            shard,
                            command,
                        );
                    }
                    Some(DriverCommand::MoveObserver { position }) => {
                        // 观察者移动(REQ-002 §4.2 生产入口):盖章 + 专用收件箱
                        // + 命令日志(与 Send 同款:先缝后日志);收件箱 unbounded
                        // → 无 InboxFull 路径,非反压错误 panic(fail-stop)。
                        match session.move_observer(position) {
                            Ok(entry) => log.append(entry),
                            Err(other) => panic!("观察者移动失败(非预期):{other:?}"),
                        }
                    }
                    Some(cmd @ DriverCommand::StreamChunks { .. }) => {
                        // chunk 流往返(REQ-003 §4.4,任务书 02):入挂起队列,
                        // tick 末端编排第 ④ 步处理——永不阻塞 tick(README 决策 7)。
                        pending_streams.push(cmd);
                    }
                }
            }
        }
    }

    // —— 退出路径(任何关机源,README 决策 5)——
    // 未处理的挂起流请求:drop 队列 = oneshot 回执端弃发 → 泵收到 Err →
    // 连接终止(REQ-003 §4.4 回执失败语义);驱动关停即全网停,无悬挂等待。
    drop(pending_streams);
    // 有未落检查点的增量(日志非空或自上次 checkpoint 后有 tick)→ 终末 checkpoint。
    if !log.is_empty() || ticks_run > ticks_at_last_checkpoint {
        write_checkpoint(&session, &mut log, &store, &mut checkpoints_written);
    }
    // flush 只在关停:屏障 = 此前全部操作(期中 + 终末)已落库。
    let ack = store.flush();
    // worker 的 ack 通道不允许被弃收——弃收 = worker panic(按其设计,决策 5)。
    let flush_result = ack
        .recv()
        .expect("storage-writer 在 flush 前死亡:一致性违规(fail-stop)");
    if let Err(e) = flush_result {
        panic!("存储粘滞错误(fail-stop 停机):{e}");
    }
    session.shutdown();
    store
        .shutdown()
        .expect("存储关停失败(粘滞错误已在上一步 panic,此处不应再失败)");

    DriverOutcome {
        ticks_run,
        checkpoints_written,
        commands_rejected,
        alert_active_at_exit: telemetry.alert(),
        shards_spawned,
        shards_despawned,
        chunks_loaded,
        chunks_unloaded,
    }
}

/// 装配:开库 → 查最新检查点(read_latest 在移交 worker 前,决策 6)→
/// 有则 replay 恢复(Session + log 续跑),无则全新;返回驱动所需全部件。
///
/// chunks 库 = `data_dir/chunks.redb`、checkpoints 库 = `data_dir/checkpoints.redb`
/// (目录不存在则创建);chunks 库为将来世界加载预留(本轮只开不用)。
pub fn bootstrap(
    config: &Config,
    new_registry: impl Fn() -> Box<dyn voxel::BlockRegistry>,
) -> Result<
    (
        sim_api::session::Session,
        sim_api::log::CommandLog,
        storage::worker::StoreWorker,
        Option<types::Tick>,
    ),
    AppError,
> {
    let data_dir = &config.data_dir;
    // 目录创建失败经 ConfigError::Io 上抛(AppError 无 IO 变体,任务书签名固定;
    // 配置指定的数据目录不可创建归配置错误)。
    std::fs::create_dir_all(data_dir).map_err(ConfigError::Io)?;
    let chunks_path = data_dir.join("chunks.redb");
    let checkpoints_path = data_dir.join("checkpoints.redb");
    let chunks = storage::redb::RedbStore::open(&chunks_path)?;
    let checkpoints = storage::redb::RedbStore::open(&checkpoints_path)?;

    // read_latest 必须在移交 worker 之前(决策 6:移交后文件锁在手)。
    let latest = checkpoints.read_latest()?;
    let (session, log, resume_from) = match latest {
        Some((at, blob)) => {
            // 有检查点:解码 bundle(replay_world 恢复 Session + scheduler
            // 状态 + log 续跑,拍号对齐)。旧格式/未知版本 → Err(AppError::
            // Decode),不静默降级(REQ-005 D8)。
            let bundle = decode(&blob)?;
            let log = sim_api::log::CommandLog::from_bytes(&bundle.log_bytes)
                .map_err(|e| AppError::Decode(e.to_string()))?;
            if bundle.world.shards.tick != at {
                // 库内 (tick, blob) 元组与 bundle 快照拍不一致 = 存储损坏,fail-stop。
                panic!(
                    "检查点元数据不一致:记录 tick {} ≠ bundle 快照拍 {:?}(fail-stop)",
                    at.0, bundle.world.shards.tick
                );
            }
            let session = sim_api::replay::replay_world(
                &bundle.world,
                &log,
                config.session_config(),
                new_registry,
            )?;
            (session, log, Some(at))
        }
        None => (
            sim_api::session::Session::new(config.session_config()),
            sim_api::log::CommandLog::new(),
            None,
        ),
    };

    let store = storage::worker::StoreWorker::new(Box::new(chunks), Box::new(checkpoints));
    Ok((session, log, store, resume_from))
}

/// bootstrap 之外的第二步装配(REQ-002 §4.3/§4.4):按 world_kind 造生成器,
/// 与 worker 句柄合成 ChunkProvider;new_sim 闭包捕获 **config 本体**(只读
/// 使用 shard_sim_config 换算;取所有权以免借用 DriverArgs.config)与 registry
/// 工厂(README 决策 6/7/9:ensure_shard 在 app 层,调度器不知生成器;
/// Config 是种子/世界种类单一来源)。
///
/// `world_kind`:
/// - `Superflat` → `Superflat::classic()`;
/// - `SeededScatter` → `SeededScatter::new(0.1)`(低密度散布,基底不变)。
pub fn wire_provider_and_new_sim(
    config: Config,
    store: &storage::worker::StoreWorker,
    new_registry: impl Fn() -> Box<dyn voxel::BlockRegistry> + Send + 'static,
) -> (
    storage::provider::ChunkProvider,
    Box<dyn FnMut(types::ShardId) -> physics_sim::ShardSim + Send>,
) {
    let generator: Box<dyn voxel::WorldGenerator> = match config.world_kind {
        crate::config::WorldKind::Superflat => Box::new(voxel::Superflat::classic()),
        crate::config::WorldKind::SeededScatter => Box::new(voxel::SeededScatter::new(0.1)),
    };
    let provider = storage::provider::ChunkProvider::new(store, generator, config.world_seed);
    let new_registry = new_registry;
    let new_sim = Box::new(move |shard: types::ShardId| {
        physics_sim::ShardSim::new(config.shard_sim_config(shard), new_registry())
    });
    (provider, new_sim)
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    use super::*;
    use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
    use storage::mem::{MemCheckpointStore, MemChunkStore};
    use storage::traits::{CheckpointId, CheckpointStore, StoreError};
    use types::glam::Vec3;
    use types::{DimensionId, EntityDesc, EntityId, EntityState, ShardPos, WorldId};
    use voxel::SimpleRegistry;

    /// 测试用分片身份(与 sim-api/shard 测试同风格)。
    fn shard_id(x: i32, z: i32) -> types::ShardId {
        types::ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    /// 空调度器快照(手工构造 CheckpointBundle 的测试用;tick 可控)。
    fn empty_scheduler_snapshot(tick: types::Tick) -> shard::SchedulerSnapshot {
        shard::SchedulerSnapshot {
            tick,
            observers: std::collections::BTreeMap::new(),
            observers_ever_set: false,
            router: shard::RouterSnapshot {
                envelopes: Vec::new(),
                sequences: std::collections::BTreeMap::new(),
            },
            lifecycles: std::collections::BTreeMap::new(),
        }
    }

    /// 凝固桥关闭的虚空分片(session 测试同款:不关心桥行为)。
    fn sim(x: i32, z: i32) -> ShardSim {
        let mut cfg = ShardSimConfig::new(shard_id(x, z));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
    }

    /// 测试配置:2 worker、checkpoint 间隔可覆写、dt 保持默认 1/60。
    fn test_config(checkpoint_interval_ticks: u64) -> Config {
        Config {
            worker_count: 2,
            checkpoint_interval_ticks,
            ..Config::default()
        }
    }

    /// 起一个注册了单个分片 (0,0) 的会话。
    fn session_with_one_shard(config: &Config) -> sim_api::session::Session {
        let mut session = sim_api::session::Session::new(config.session_config());
        session.register(sim(0, 0));
        session
    }

    /// 测试专用检查点存储:`Arc<Mutex<MemCheckpointStore>>` 共享句柄——驱动侧经
    /// `Box<dyn CheckpointStore>` 持有,测试侧留一份克隆在 run 结束后读回断言。
    #[derive(Clone, Default)]
    struct SharedCheckpoints(Arc<Mutex<MemCheckpointStore>>);

    impl CheckpointStore for SharedCheckpoints {
        fn append(&mut self, at: types::Tick, blob: &[u8]) -> Result<CheckpointId, StoreError> {
            self.0.lock().expect("检查点锁毒化").append(at, blob)
        }

        fn read_latest(&self) -> Result<Option<(types::Tick, Vec<u8>)>, StoreError> {
            self.0.lock().expect("检查点锁毒化").read_latest()
        }

        fn read(&self, id: CheckpointId) -> Result<Option<(types::Tick, Vec<u8>)>, StoreError> {
            self.0.lock().expect("检查点锁毒化").read(id)
        }
    }

    /// 测试驱动参数:mem 后端、不监听 ctrl_c(README 决策 7);返回共享检查点句柄。
    /// 任务 04 起含 provider(superflat 生成器 + config.world_seed)与 new_sim
    /// 闭包(shard_sim_config 换算 + SimpleRegistry::builtin);可选覆写 world_kind。
    fn test_args(
        config: Config,
        session: sim_api::session::Session,
    ) -> (
        DriverArgs,
        tokio::sync::mpsc::Sender<DriverCommand>,
        SharedCheckpoints,
    ) {
        let checkpoints = SharedCheckpoints::default();
        let (tx, rx) = tokio::sync::mpsc::channel(16);
        let store = storage::worker::StoreWorker::new(
            Box::new(MemChunkStore::new()),
            Box::new(checkpoints.clone()),
        );
        let (provider, new_sim) = wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        });
        let args = DriverArgs {
            config,
            session,
            store,
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
            observe: None,
        };
        (args, tx, checkpoints)
    }

    /// SpawnDynamic 命令(实体出生在虚空分片原点上方)。
    fn spawn_cmd(entity: EntityId) -> messaging::SimCommand {
        messaging::SimCommand::SpawnDynamic {
            entity,
            state: EntityState {
                translation: Vec3::new(1.5, 3.0, 1.5),
                ..EntityState::default()
            },
            desc: EntityDesc::default(),
        }
    }

    /// 拍间注入一条 Send(命令分支优先级最低:已到期的 tick 先跑,之后才处理
    /// 命令——盖章 apply_at = 注入后第一个 tick 分支执行后的 tick_now)。
    async fn inject_send(
        tx: &tokio::sync::mpsc::Sender<DriverCommand>,
        shard: types::ShardId,
        command: messaging::SimCommand,
    ) {
        tx.send(DriverCommand::Send { shard, command })
            .await
            .expect("驱动仍存活");
    }

    /// 暂停时间下推进 N 拍:测试任务 `sleep(dt)` 靠 **auto-advance** 推进
    /// (tokio clock.rs 文档:`advance` 不保证到期 timer 首次 poll 即交付,
    /// 推荐用 sleep + auto-advance 可靠等一拍)。测试 sleep 与驱动 interval
    /// 同刻到期,同 slot 一起被唤醒 → 每 sleep 恰一拍,无 burst(README 决策 10)。
    async fn advance_ticks(ticks: u32) {
        let dt = Duration::from_secs_f32(1.0 / 60.0);
        for _ in 0..ticks {
            tokio::time::sleep(dt).await;
            // sleep 醒后让驱动跑完本拍处理(tick 分支)再排下一拍。
            tokio::task::yield_now().await;
        }
    }

    /// 读回最新检查点并解码为 bundle(测试侧断言用)。
    fn latest_bundle(checkpoints: &SharedCheckpoints) -> CheckpointBundle {
        let (_tick, blob) = checkpoints
            .read_latest()
            .expect("读最新检查点")
            .expect("必有检查点");
        decode(&blob).expect("自有编码必可解码")
    }

    /// 测试清单 1:start_paused;3 拍 + 拍间注入 Send(SpawnDynamic)→
    /// outcome.ticks_run == 3,实体出现(经终末检查点快照恢复验证)。
    #[tokio::test(start_paused = true)]
    async fn run_executes_ticks_and_commands() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let shard = shard_id(0, 0);
        let entity = EntityId::from_parts(1, 1);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        inject_send(&tx, shard, spawn_cmd(entity)).await;
        advance_ticks(3).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.ticks_run, 3, "3 拍");
        assert_eq!(
            outcome.checkpoints_written,
            vec![types::Tick(3)],
            "终末检查点"
        );

        // 实体出现:终末检查点的分片快照恢复后可查到该实体。
        let bundle = latest_bundle(&checkpoints);
        assert_eq!(bundle.world.shards.tick, types::Tick(3));
        let bytes = bundle
            .world
            .shards
            .per_shard
            .get(&shard)
            .expect("快照含分片 (0,0)");
        let restored =
            ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        assert!(
            restored.entity_state(entity).is_some(),
            "3 拍后 SpawnDynamic 已生效,实体存在"
        );
    }

    /// 测试清单 2:发 Shutdown → run 返回,outcome 合法。
    #[tokio::test(start_paused = true)]
    async fn shutdown_command_stops_loop() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.ticks_run, 0, "未 tick 即关机");
        assert!(outcome.checkpoints_written.is_empty(), "零拍无检查点");
        assert_eq!(outcome.commands_rejected, 0);
        assert!(!outcome.alert_active_at_exit, "无耗时样本,告警不激活");
        assert!(
            checkpoints.read_latest().expect("读库").is_none(),
            "零拍零命令:不写任何检查点"
        );
    }

    /// 测试清单 3:interval=5,跑 12 拍 → checkpoints == [5, 10, 12]
    /// (12 = 终末);store 用 mem 后端。
    #[tokio::test(start_paused = true)]
    async fn checkpoints_at_interval_and_exit() {
        let config = test_config(5);
        let session = session_with_one_shard(&config);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        advance_ticks(12).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");

        assert_eq!(outcome.ticks_run, 12);
        assert_eq!(
            outcome.checkpoints_written,
            vec![types::Tick(5), types::Tick(10), types::Tick(12)],
            "间隔 5 写 [5,10],终末 12"
        );
        let bundle = latest_bundle(&checkpoints);
        assert_eq!(bundle.world.shards.tick, types::Tick(12), "最新 = 终末");
    }

    /// 测试清单 4:小 inbox(1)灌 2 条同拍命令 → rejected==1,循环仍活。
    #[tokio::test(start_paused = true)]
    async fn rejected_command_counted_not_fatal() {
        let mut config = test_config(600);
        config.inbox_capacity = 1;
        let session = session_with_one_shard(&config);
        let shard = shard_id(0, 0);
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 同拍连灌 2 条(容量 1):第 2 条必被拒(InboxFull),驱动不因此退出。
        inject_send(&tx, shard, spawn_cmd(EntityId::from_parts(1, 1))).await;
        inject_send(&tx, shard, spawn_cmd(EntityId::from_parts(1, 2))).await;
        advance_ticks(2).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.commands_rejected, 1, "恰拒 1 条");
        assert_eq!(outcome.ticks_run, 2, "循环仍活:拒绝不致命");
    }

    /// 评审修复①(REQ-003):hostile 外部命令不再触发 fail-stop panic。
    /// 只注册分片 (0,0) 的会话上,注入客户端可构造的三类崩溃命令:
    /// SetBlock y 越界(apply_command「SetBlock 越界」)、RemoveEntity 未知
    /// 实体(despawn_entity「实体存在」)、路由到未注册分片的 SpawnDynamic
    /// (Scheduler::submit「提交给未注册分片」)——修复前三条路径各自整机
    /// panic(驱动任务连坐,tick 全停);修复后 = 丢弃计数,驱动存活、
    /// 存活命令照常生效。
    #[tokio::test(start_paused = true)]
    async fn hostile_commands_dropped_not_fatal() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let shard = shard_id(0, 0);
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 三类 hostile 命令(全部只注册 (0,0) 时客户端可构造)。
        inject_send(
            &tx,
            shard,
            messaging::SimCommand::SetBlock {
                pos: types::BlockPos::new(0, 1000, 0), // y > WORLD_Y_MAX(319)
                block: types::BlockId(1),
            },
        )
        .await;
        inject_send(
            &tx,
            shard,
            messaging::SimCommand::RemoveEntity {
                entity: EntityId::from_parts(1, 1), // 分片内无实体
            },
        )
        .await;
        inject_send(
            &tx,
            shard_id(999, 0), // 未注册分片(origin=999 的信息性路由)
            spawn_cmd(EntityId::from_parts(999, 1)),
        )
        .await;
        // 存活命令(同型,但参数合法)照常生效。
        inject_send(&tx, shard, spawn_cmd(EntityId::from_parts(1, 1))).await;
        advance_ticks(2).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("hostile 命令不崩驱动(丢弃计数)");
        assert_eq!(
            outcome.commands_rejected, 3,
            "三条 hostile 命令各自计数(丢弃不投)"
        );
        assert_eq!(outcome.ticks_run, 2, "循环仍活:hostile 输入不致命");
    }

    /// gate 修复(任务 03 先行项):`SetObserverPosition` 与 `RemoveObserver`
    /// 同为观察者收件箱专用命令,经 commands 通道投递 → 前置校验拒绝
    /// (commands_rejected 计数),不 panic、驱动存活、观察者表不变
    /// (MoveObserver 专用包才是观察者移动的正路)。
    #[tokio::test(start_paused = true)]
    async fn setobserver_via_commands_rejected_counted() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 观察者专用命令经 Send 缝(commands 通道)注入:修复前落入 Ok 组,
        // 直投分片收件箱 → 拍 0 应用时命中 ShardSim fail-stop panic(连坐)。
        inject_send(
            &tx,
            shard_id(0, 0),
            messaging::SimCommand::SetObserverPosition {
                position: Vec3::new(8.0, 3.0, 8.0),
            },
        )
        .await;
        advance_ticks(2).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("观察者命令经 Send 缝不崩驱动");
        assert_eq!(
            outcome.commands_rejected, 1,
            "SetObserverPosition 经 Send 缝被拒绝并计数"
        );

        // 观察者表不变:检查点快照的调度器观察者表为空(直投未发生)。
        let bundle = latest_bundle(&checkpoints);
        assert!(
            bundle.world.scheduler.observers.is_empty(),
            "观察者表不变(命令未进观察者收件箱)"
        );
    }

    // ---- 任务书 req005-consistency/03:网络 Detonate 跨片广播(app 面)----

    /// 任务书测试 `detonate_hostile_parameters_rejected`:NaN/±inf/radius≤0
    /// 的 Detonate 被驱动丢弃(commands_rejected 计数),不 panic、不断链;
    /// 合法参数的同型命令照常生效(对照)。合法版同时验证 Detonate 走
    /// `detonate_from`(日志按覆盖集逐片一条)与"中心分片未注册不拒"
    /// (爆心在未注册区域 → 空目标集 = 零条日志、零拒绝)。
    #[tokio::test(start_paused = true)]
    async fn detonate_hostile_parameters_rejected() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        let shard = shard_id(0, 0);
        let blast = |power: f32, radius: f32| messaging::SimCommand::Detonate {
            center: Vec3::new(8.0, 3.0, 8.0),
            power,
            radius,
        };
        // 四类 hostile 参数(路由分片已注册,预检只看参数)。
        inject_send(&tx, shard, blast(f32::NAN, 5.0)).await;
        inject_send(&tx, shard, blast(f32::INFINITY, 5.0)).await;
        inject_send(&tx, shard, blast(6.0, f32::NEG_INFINITY)).await;
        inject_send(&tx, shard, blast(6.0, 0.0)).await;
        // 对照:爆心在远端未注册区域(分片 (999,9))——合法参数 → 空目标集,
        // 零拒绝零日志(不因"爆心分片未注册"被丢)。
        inject_send(
            &tx,
            shard_id(999, 9),
            messaging::SimCommand::Detonate {
                center: Vec3::new(500_000.0, 3.0, 500_000.0),
                power: 6.0,
                radius: 5.0,
            },
        )
        .await;
        advance_ticks(2).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("hostile Detonate 不崩驱动");
        assert_eq!(
            outcome.commands_rejected, 4,
            "四条 hostile 参数各自计数(丢弃不投),合法空目标集不计"
        );

        // 日志面:无 Detonate 入日志(四条被丢 + 空目标集零条)。
        let bundle = latest_bundle(&checkpoints);
        let entries = sim_api::log::CommandLog::from_bytes(&bundle.log_bytes)
            .expect("自有编码必可解码")
            .take();
        assert!(
            !entries
                .iter()
                .any(|e| matches!(e.command, messaging::SimCommand::Detonate { .. })),
            "hostile Detonate 不入日志:{entries:?}"
        );
    }

    /// 任务书 03(app 面):合法 Detonate 经 commands 通道 → `detonate_from`
    /// 广播——日志按覆盖集逐片一条(升序),分片行为 = 各自本地求值(范围内
    /// 实体受冲量)。单分片场景目标 = 1(行为与旧单片 send 等价,回归面)。
    #[tokio::test(start_paused = true)]
    async fn detonate_via_commands_broadcasts_and_logs() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let shard = shard_id(0, 0);
        let e = EntityId::from_parts(1, 1);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 先确保分片在(预注册),再注入实体(命令路径)与爆炸。
        inject_send(&tx, shard, spawn_cmd(e)).await;
        advance_ticks(1).await;
        inject_send(
            &tx,
            shard,
            messaging::SimCommand::Detonate {
                center: Vec3::new(1.5, 3.0, 1.5),
                power: 8.0,
                radius: 5.0,
            },
        )
        .await;
        advance_ticks(2).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.commands_rejected, 0, "合法命令零拒绝");

        // 日志:Spawn 1 条 + Detonate 1 条(单分片覆盖集 = 1 个目标)。
        let bundle = latest_bundle(&checkpoints);
        let entries = sim_api::log::CommandLog::from_bytes(&bundle.log_bytes)
            .expect("自有编码必可解码")
            .take();
        let blasts: Vec<_> = entries
            .iter()
            .filter(|e| matches!(e.command, messaging::SimCommand::Detonate { .. }))
            .collect();
        assert_eq!(blasts.len(), 1, "单分片 detonate = 一条日志(覆盖集 1 目标)");
        assert_eq!(blasts[0].dest, shard, "dest = 覆盖分片");
    }

    /// 测试清单 5:(暂停时间下)跑 20 拍 tick 计数恰 20,无连跳(首枚已吃)。
    #[tokio::test(start_paused = true)]
    async fn interval_skip_does_not_burst() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        advance_ticks(20).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(
            outcome.ticks_run, 20,
            "20 个周期恰 20 拍:Skip 不补帧、首枚已吃"
        );
    }

    /// 回归(评审发现 1):tick 到期与命令就绪同刻竞先时,tick 分支先执行——
    /// 命令洪水饿不死模拟(README 决策 2「关机 > tick > 命令」,v2 语义)。
    /// 判据 = LogEntry.tick(send 盖章 apply_at = tick_now):发送任务与 interval
    /// 第 3 枚 deadline 同刻到期(暂停时钟同 slot;发送任务先 spawn → 其 sleep
    /// 定时器先注册 → 同 slot 先被唤醒)。若命令分支排在 tick 前(biased 书写序),
    /// 命令在 tick#3 之前被处理 → 盖章 2(旧序实证);新序盖章 3。
    #[tokio::test(start_paused = true)]
    async fn due_tick_precedes_queued_command() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let shard = shard_id(0, 0);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 发送任务:与驱动第 3 枚 tick 同刻醒,把命令送入通道。
        let tx_sender = tx.clone();
        let sender = tokio::spawn(async move {
            tokio::time::sleep(Duration::from_secs_f32(1.0 / 60.0) * 3).await;
            tx_sender
                .send(DriverCommand::Send {
                    shard,
                    command: spawn_cmd(EntityId::from_parts(1, 1)),
                })
                .await
                .expect("驱动仍存活");
        });

        advance_ticks(3).await;
        sender.await.expect("发送任务不 panic");
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.ticks_run, 3, "命令排队不吞拍");

        // 终末检查点(bundle.tick = 3)里的日志:新序下命令在 tick#3 之后处理,
        // 盖章 = 已推进的 3 拍。
        let bundle = latest_bundle(&checkpoints);
        assert_eq!(bundle.world.shards.tick, types::Tick(3));
        let entries = sim_api::log::CommandLog::from_bytes(&bundle.log_bytes)
            .expect("自有编码必可解码")
            .take();
        assert_eq!(entries.len(), 1, "注入的命令已入日志");
        assert_eq!(
            entries[0].tick,
            types::Tick(3),
            "到期 tick 先于同刻就绪的命令(v2 语义);盖章 2 即旧序回归"
        );
    }

    /// 测试清单 6:bundle 编解码往返相等;空拍照常出 checkpoint(0 分片快照合法);
    /// 版本字段带当前 CHECKPOINT_FORMAT_VERSION(统一常量,任务书 02)。
    #[test]
    fn checkpoint_bundle_roundtrip() {
        // 非空:带分片字节 + 日志字节。
        let mut snap = sim_api::snapshot::GlobalSnapshot::new(types::Tick(7));
        snap.insert_shard(shard_id(0, 0), vec![1, 2, 3]);
        let bundle = CheckpointBundle {
            format_version: CHECKPOINT_FORMAT_VERSION,
            world: sim_api::snapshot::WorldSnapshot {
                shards: snap,
                scheduler: empty_scheduler_snapshot(types::Tick(7)),
            },
            log_bytes: sim_api::log::CommandLog::new().to_bytes(),
        };
        let decoded = decode(&encode(&bundle)).expect("自有编码必可解码");
        assert_eq!(
            decoded.world.shards, bundle.world.shards,
            "快照往返逐字段相等"
        );
        assert_eq!(decoded.log_bytes, bundle.log_bytes);

        // 空快照(0 分片)+ 空日志:同样合法。
        let empty = CheckpointBundle {
            format_version: CHECKPOINT_FORMAT_VERSION,
            world: sim_api::snapshot::WorldSnapshot {
                shards: sim_api::snapshot::GlobalSnapshot::new(types::Tick(0)),
                scheduler: empty_scheduler_snapshot(types::Tick(0)),
            },
            log_bytes: Vec::new(),
        };
        let decoded = decode(&encode(&empty)).expect("空 bundle 必可解码");
        assert!(decoded.world.shards.per_shard.is_empty(), "0 分片快照合法");
        assert_eq!(decoded.world.shards.tick, types::Tick(0));
    }

    /// 任务书测试 `old_checkpoint_version_rejected`(REQ-005 D8):两条路径
    /// 都必须明确 Err、不静默降级——
    /// ① 手工编码 `format_version` 错值的合法 bundle → 版本不符 Err;
    /// ② 任务 01 前旧布局字节(`{snapshot, log_bytes}` 形状,无版本字段)
    ///    → bincode 解码失败 Err。
    #[test]
    fn old_checkpoint_version_rejected() {
        let mk_world = |tick: types::Tick| sim_api::snapshot::WorldSnapshot {
            shards: sim_api::snapshot::GlobalSnapshot::new(tick),
            scheduler: empty_scheduler_snapshot(tick),
        };

        // ① 错误版本号:结构合法但 format_version = 99 → 明确版本错误。
        let future = CheckpointBundle {
            format_version: 99,
            world: mk_world(types::Tick(7)),
            log_bytes: sim_api::log::CommandLog::new().to_bytes(),
        };
        let bytes = encode(&future);
        match decode(&bytes) {
            Err(AppError::Decode(msg)) => {
                assert!(
                    msg.contains("unsupported checkpoint format version 99"),
                    "错误信息必须指明版本不符:{msg}"
                );
            }
            other => panic!(
                "错误版本必须 Err(AppError::Decode),实为 {:?}",
                other.is_err()
            ),
        }

        // 0(旧版本起点)同样拒绝。
        let zero = CheckpointBundle {
            format_version: 0,
            world: mk_world(types::Tick(7)),
            log_bytes: Vec::new(),
        };
        assert!(decode(&encode(&zero)).is_err(), "版本 0 = 旧格式,拒绝");

        // ② 任务 01 前旧布局:{snapshot: GlobalSnapshot, log_bytes}。手工
        //    bincode 编码(结构形状与新 bundle 不同,无 format_version 字段
        //    前缀)→ 新 decode 必然解码失败(不是静默解释成别的世界)。
        let mut old_snap = sim_api::snapshot::GlobalSnapshot::new(types::Tick(5));
        old_snap.insert_shard(shard_id(0, 0), vec![1, 2, 3]);
        #[derive(serde::Serialize)]
        struct PreTask01Bundle {
            snapshot: sim_api::snapshot::GlobalSnapshot,
            log_bytes: Vec<u8>,
        }
        let old_bytes = bincode::serde::encode_to_vec(
            PreTask01Bundle {
                snapshot: old_snap,
                log_bytes: Vec::new(),
            },
            bincode::config::standard(),
        )
        .expect("旧布局编码(测试构造)");
        match decode(&old_bytes) {
            Err(AppError::Decode(_)) => {}
            other => panic!(
                "任务 01 前旧布局字节必须 Err(Decode),实为 {:?}",
                other.is_err()
            ),
        }
    }

    // ---- 任务 04:驱动接线(测试清单)----

    /// 地面顶面高度(world y):Superflat classic = stone×1 + dirt×3,层底在
    /// 世界 y = WORLD_Y_MIN(-64)→ 顶面 y = -60。
    const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;

    /// MoveObserver 命令(注入通道)。
    async fn inject_move(tx: &tokio::sync::mpsc::Sender<DriverCommand>, position: Vec3) {
        tx.send(DriverCommand::MoveObserver { position })
            .await
            .expect("驱动仍存活");
    }

    /// 测试清单 1 `observer_walk_spawns_shard_and_loads_ground`:MoveObserver 进
    /// 新区 → 分片创建(ensure)→ 激活请求 → provider 真路径装载 → 实体落在
    /// 生成的地面上被支撑。
    #[tokio::test(start_paused = true)]
    async fn observer_walk_spawns_shard_and_loads_ground() {
        let config = test_config(600);
        // 空会话(无预注册分片):分片由驱动 ensure 造出。
        let session = sim_api::session::Session::new(config.session_config());
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 观察者进分片 (1,0) 区域(x=200 → chunk 12 → 分片 (1,0));y 取地面
        // 顶面附近(-60):tier③ 冻结半径 32m 按 3D 欧氏计,观察者悬在 y=0 而
        // 实体在 y=-55 时距离 55m,实体会被立即冻结、无法验证"落在地面被支撑"。
        inject_move(&tx, Vec3::new(200.0, GROUND_TOP_Y, 8.0)).await;
        // 先过 1 拍:让编排 ① 在 tick 末端 ensure 出分片(命令分支比 tick 优先级
        // 低,同拍注入的 SpawnDynamic 会先于 ensure 被 session.send 拒绝——
        // 未注册分片 fail-stop 是调度器语义,测试须保证先建片再发命令)。
        advance_ticks(1).await;
        // 实体出生观察者上方 5m,随装载的地面驻留(到观察者 3D 距离 ≤5.5m,
        // 全程在唤醒半径 24m 内 → 不被冻结,可自由下落)。
        inject_send(
            &tx,
            shard_id(1, 0),
            messaging::SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 1),
                state: EntityState {
                    translation: Vec3::new(200.5, -55.0, 8.5),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            },
        )
        .await;
        // 装载发生在 tick 末端(晚一拍);实体从出生到落稳(探针实测 ~80 拍,
        // rapier 碰撞余量驻留在顶面上方 ~0.57m)需百余拍:120 拍留裕度。
        advance_ticks(120).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(
            outcome.shards_spawned >= 1,
            "观察者进入新区 → 分片被 ensure 创建"
        );
        assert!(outcome.chunks_loaded >= 1, "激活请求经 provider 真路径装载");

        // 实体落在生成的地面上(顶面 world y = -60):经终末检查点快照验证。
        // 驱动 new_sim 走 config.shard_sim_config → 凝固桥**默认开启**:实体
        // 落稳约 20 拍后经 P2′(速度+位移门)凝固成方块(合法落定,非误凝固)
        // ——entity_state 变 None。故断言二选一:仍是实体(顶面上方驻留带)或
        // 已凝固(实体出生列上出现非空气方块)。两者都只有"地面真装载并被
        // 物理支撑"才会发生。
        let bundle = latest_bundle(&_checkpoints);
        let bytes = bundle
            .world
            .shards
            .per_shard
            .get(&shard_id(1, 0))
            .expect("观察者分片已创建并进入快照");
        let restored =
            ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        let entity = EntityId::from_parts(1, 1);
        match restored.entity_state(entity) {
            Some(state) => {
                assert!(
                    state.translation.y > GROUND_TOP_Y && state.translation.y < GROUND_TOP_Y + 1.5,
                    "实体仍为实体:应驻留在地面顶面上方:y = {}(顶面 {GROUND_TOP_Y})",
                    state.translation.y
                );
            }
            None => {
                // 凝固落点 = 实体出生 x/z 的方块列(spawn x=200.5,z=8.5 →
                // 方块 (200,·,8));凝固桥只向 air 格写方块 → 该列必有非空气。
                let mut column_has_block = false;
                for y in types::WORLD_Y_MIN..=types::WORLD_Y_MAX {
                    if !restored
                        .voxel()
                        .get_block(types::BlockPos::new(200, y, 8))
                        .is_air()
                    {
                        column_has_block = true;
                        break;
                    }
                }
                assert!(
                    column_has_block,
                    "实体已凝固:出生列 (x=200, z=8) 应出现凝固方块(地面真装载且被支撑)"
                );
            }
        }
    }

    /// 测试清单 2 `chunk_unload_persisted_via_worker`:观察者远离 → 卸载请求 →
    /// put_chunks 落库(重开 store 验证;mem 后端经共享句柄读回)。
    #[tokio::test(start_paused = true)]
    async fn chunk_unload_persisted_via_worker() {
        // 读回句柄:驱动持 mem 后端的 Box,测试侧用共享句柄直接查 MemChunkStore。
        #[derive(Clone, Default)]
        struct SharedChunks(Arc<Mutex<storage::mem::MemChunkStore>>);

        impl storage::traits::ChunkStore for SharedChunks {
            fn put_batch(
                &mut self,
                batch: &[(storage::traits::ChunkKey, voxel::Chunk)],
            ) -> Result<(), storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").put_batch(batch)
            }
            fn get(
                &self,
                key: storage::traits::ChunkKey,
            ) -> Result<Option<voxel::Chunk>, storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").get(key)
            }
            fn len(&self) -> Result<usize, storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").len()
            }
            fn is_empty(&self) -> Result<bool, storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").is_empty()
            }
        }

        let mut config = test_config(600);
        // 激活滞回:观察者瞬移远超卸载半径 → 下拍必出卸载请求。冷却关(瞬移
        // 立即生效);预算调大——卸载候选按"最近观察者距离"升序与远邻域装载
        // 候选同队竞争(瞬移后卸载候选距 ~5 万 m 排在装载候选 ~12m 之后),
        // 默认预算 2 会被装载吃光、卸载恒饿死(激活策略机实测语义);预算给足
        // (卸载 2 + 远邻域 12)让卸载请求出队。
        config.activation.cooldown_secs = 0;
        config.swap_budget_chunks_per_tick = 14;
        let session = sim_api::session::Session::new(config.session_config());
        let chunks = SharedChunks::default();
        let (tx, rx) = tokio::sync::mpsc::channel(16);
        let checkpoints = SharedCheckpoints::default();
        let store = storage::worker::StoreWorker::new(
            Box::new(chunks.clone()),
            Box::new(checkpoints.clone()),
        );
        let (provider, new_sim) = wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        });
        let driver = tokio::spawn(run(DriverArgs {
            config,
            session,
            store,
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
            observe: None,
        }));

        // 进区(分片 (1,0))→ 等装载 → 瞬移远走(分片 (50,0),超卸载半径)。
        inject_move(&tx, Vec3::new(200.0, 0.0, 8.0)).await;
        advance_ticks(5).await;
        inject_move(&tx, Vec3::new(50_000.0, 0.0, 8.0)).await;
        advance_ticks(5).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(outcome.chunks_loaded >= 1, "前置:先有装载");
        assert!(outcome.chunks_unloaded >= 1, "观察者远离 → chunk 被卸载");

        // 重开 store 验证:mem 后端无"重开",等价口径 = 卸载值确实经 worker
        // 落库(共享句柄可见),且值与生成器初生值一致(未被修改)。
        let key = storage::traits::ChunkKey {
            world: WorldId(0),
            dim: DimensionId(0),
            pos: types::ChunkPos::new(12, 0),
        };
        let stored =
            storage::traits::ChunkStore::get(&*chunks.0.lock().expect("chunk 锁毒化"), key)
                .expect("读回不失败")
                .expect("卸载的 chunk 已落库");
        let expected = voxel::WorldGenerator::generate(
            &voxel::Superflat::classic(),
            types::WorldSeed(0),
            key.pos,
        );
        assert_eq!(stored, expected, "落库值 = 卸载时的 chunk(初生值)");
    }

    /// 测试清单 3 `despawn_flow_with_barrier`:小 despawn 窗口下,空片无票到期
    /// → 候选 → unload_shard → put + flush ack 屏障 → outcome.shards_despawned
    /// == 1;有实体的分片不 despawn。
    #[tokio::test(start_paused = true)]
    async fn despawn_flow_with_barrier() {
        let mut config = test_config(600);
        config.despawn_after_secs = 0; // despawn_after_ticks = 0 → 空片当拍即候选
        // 预注册两个分片:A(0,0) 恒空;B(0,1) 带实体。无观察者注入(命令路径)。
        // 注意:无观察者 = 编排零动作(无票判定回退),但 despawn 候选判定在
        // 调度器 tick 内(与驱动编排无关)——A 满足空+无票+idle 到期。
        let mut session = sim_api::session::Session::new(config.session_config());
        session.register(sim(0, 0));
        let mut sim_b = sim(0, 1);
        sim_b.apply_prefix(&[messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 0,
            payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 1),
                state: EntityState {
                    translation: Vec3::new(8.0, 3.0, 136.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            }),
        }]);
        session.register(sim_b);

        let (tx, rx) = tokio::sync::mpsc::channel(16);
        let checkpoints = SharedCheckpoints::default();
        let store = storage::worker::StoreWorker::new(
            Box::new(MemChunkStore::new()),
            Box::new(checkpoints.clone()),
        );
        let (provider, new_sim) = wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        });
        let driver = tokio::spawn(run(DriverArgs {
            config,
            session,
            store,
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
            observe: None,
        }));

        advance_ticks(3).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.shards_despawned, 1, "恰 A(空片无票)被 despawn");
        assert!(outcome.ticks_run >= 3, "前置:跑过 despawn 窗口");

        // B(带实体)仍在:终末检查点快照含 B、不含 A。
        let bundle = latest_bundle(&checkpoints);
        assert!(
            !bundle.world.shards.per_shard.contains_key(&shard_id(0, 0)),
            "despawn 的 A 不在终末快照"
        );
        assert!(
            bundle.world.shards.per_shard.contains_key(&shard_id(0, 1)),
            "有实体的 B 不 despawn"
        );
    }

    /// 测试清单 4 `move_observer_command_routed`:DriverCommand::MoveObserver →
    /// session 观察者表更新(两个口径:命令日志含 SetObserverPosition = 已盖章
    /// 路由;编排 ① 从 observers() 视图 ensure 出新分片 = 表已更新且驱动可见)。
    /// 任务书 req005/00 测试 10:观察者距边界 <32m → 邻片在 ensure 集合;
    /// >32m 不在(shards_within 纯函数 = 驱动邻片 ensure 的判定核心)。
    #[test]
    fn ensure_covers_observer_neighbors() {
        use types::glam::Vec3;
        let shard = |x: i32, z: i32| {
            types::ShardId::new(
                types::WorldId(0),
                types::DimensionId(0),
                types::ShardPos::new(x, z),
            )
        };
        // 片 (0,0) 中心:距四界均 64m > 32m → 无邻片。
        let center = shards_within(Vec3::new(64.0, 0.0, 64.0), 32.0);
        assert_eq!(center, vec![shard(0, 0)]);
        // 贴东界(x=120):片 (1,0) 核心 AABB 距 8m ≤ 32 → 两片;斜角远不含。
        let east = shards_within(Vec3::new(120.0, 0.0, 64.0), 32.0);
        assert!(east.contains(&shard(0, 0)));
        assert!(east.contains(&shard(1, 0)), "32m 内邻片被 ensure:{east:?}");
        assert_eq!(east.len(), 2, "只东邻,不含斜角:{east:?}");
        // 角落(x=120, z=120):四分片都在(东/南 8m;斜角 sqrt(8²+8²)≈11m)。
        let corner = shards_within(Vec3::new(120.0, 0.0, 120.0), 32.0);
        assert_eq!(corner.len(), 4, "角落四分片都在:{corner:?}");
        // 距界 33m:邻片不在(无过度 ensure)。
        let far = shards_within(Vec3::new(95.0, 0.0, 64.0), 32.0);
        assert_eq!(far, vec![shard(0, 0)], "33m > 32m 不 ensure 邻片");
    }

    #[tokio::test(start_paused = true)]
    async fn move_observer_command_routed() {
        let config = test_config(600);
        let session = sim_api::session::Session::new(config.session_config());
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        let pos = Vec3::new(200.0, 0.0, 8.0);
        inject_move(&tx, pos).await;
        advance_ticks(2).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(outcome.ticks_run >= 2, "前置:拍已推进");

        // 命令日志含 SetObserverPosition(盖章入日志 = 已路由进 session)。
        let bundle = latest_bundle(&checkpoints);
        let log_again =
            sim_api::log::CommandLog::from_bytes(&bundle.log_bytes).expect("自有编码必可解码");
        assert!(
            log_again
                .entries()
                .iter()
                .any(|e| e.command == messaging::SimCommand::SetObserverPosition { position: pos }),
            "MoveObserver 已入命令日志"
        );

        // 观察者表更新(REQ-005 任务 04 补强:精确集合,不再取 ≥1)——
        // 编排 ① 读 observers() 视图按 shards_within(ensure_radius=32)
        // ensure:观察者 (200,·,8) 恰距 z=0 分片界 8m ≤ 32m → 恰 2 片
        // {(1,0) 本体,(1,-1) 邻片};session 起动为空 → 两 ensure 皆新建。
        assert_eq!(
            outcome.shards_spawned, 2,
            "观察者表更新且驱动恰 ensure 出本体片 + 邻片(不多不少):{}",
            outcome.shards_spawned
        );
        // 终末快照的分片键集 = 精确集合 {(1,-1),(1,0)}(BTreeMap 键序):
        // 表更新落到分片存在性上,无第三片混入、无缺片。
        let keys: Vec<types::ShardId> = bundle.world.shards.per_shard.keys().copied().collect();
        assert_eq!(
            keys,
            vec![shard_id(1, -1), shard_id(1, 0)],
            "观察者 ensure 的分片集合恰为 {{(1,-1),(1,0)}}(观察者 (200,0,8) \
             距 z 界 8m ≤ 32m → 本体 + z 负邻片,不含其他)"
        );
    }

    /// 测试清单 5 `outcome_counters_reflect_activity`:计数字段与场景一致——
    /// 走一个 进区装载 → 远走卸载 → A 分片 despawn 的组合场景,四字段全非零
    /// 且 despawn 恰 1。
    #[tokio::test(start_paused = true)]
    async fn outcome_counters_reflect_activity() {
        let mut config = test_config(600);
        config.despawn_after_secs = 0;
        // 同 chunk_unload 测试:预算给足让远走后的卸载请求出队(否则被远邻域
        // 装载候选饿死);冷却关让瞬移立即生效。
        config.activation.cooldown_secs = 0;
        config.swap_budget_chunks_per_tick = 14;
        let session = sim_api::session::Session::new(config.session_config());
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 进区装载(分片 (1,0))→ 远走卸载 → despawn(空+无票,idle 窗口 0)。
        inject_move(&tx, Vec3::new(200.0, 0.0, 8.0)).await;
        advance_ticks(4).await;
        inject_move(&tx, Vec3::new(50_000.0, 0.0, 8.0)).await;
        advance_ticks(4).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(outcome.shards_spawned >= 1, "ensure 建片计数");
        assert!(outcome.chunks_loaded >= 1, "装载计数");
        assert!(outcome.chunks_unloaded >= 1, "卸载计数");
        // REQ-005 D5:观察者 (200,·,8) 距 z 界 8m → 邻片 (1,-1) 同步 ensure;
        // 远走后两片皆空+无票 → despawn 2(原"唯一分片"口径随邻片 ensure 更新)。
        assert_eq!(outcome.shards_despawned, 2, "离开后本片+邻片 despawn");
    }

    /// 测试清单 6 `no_observers_no_orchestration`:无观察者时编排零动作
    /// (既有行为不变,回归):预注册分片跑 10 拍,四个计数全 0。
    #[tokio::test(start_paused = true)]
    async fn no_observers_no_orchestration() {
        let config = test_config(600);
        let session = session_with_one_shard(&config);
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        advance_ticks(10).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.ticks_run, 10, "前置:10 拍");
        assert_eq!(outcome.shards_spawned, 0, "无观察者:不建片");
        assert_eq!(outcome.shards_despawned, 0, "无观察者:不 despawn");
        assert_eq!(outcome.chunks_loaded, 0, "无观察者:不装载");
        assert_eq!(outcome.chunks_unloaded, 0, "无观察者:不卸载");
    }

    // ---- 任务书 req005-consistency/01:外部编辑门卫 / outbox 源 / 持久化归属 ----

    /// 起一个预注册分片 (0,0)、手工装载 chunk(0,0) 与 chunk(8,0) 的会话
    /// (chunk (8,0) 的 owner = 片 (1,0),扩张副本语义;分片 (1,0) 不注册时
    /// 装载会被 owner-在位校验拦下,故同时注册空片 (1,0))。
    fn session_two_loaded_chunks(config: &Config) -> sim_api::session::Session {
        let mut session = sim_api::session::Session::new(config.session_config());
        session.register(sim(0, 0));
        session.register(sim(1, 0));
        session.apply_chunk_load(
            shard_id(0, 0),
            types::ChunkPos::new(0, 0),
            voxel::Chunk::air(),
        );
        // chunk (8,0):owner = 片 (1,0);片 (1,0) 装自己的本体 chunk。
        session.apply_chunk_load(
            shard_id(1, 0),
            types::ChunkPos::new(8, 0),
            voxel::Chunk::air(),
        );
        session
    }

    /// 任务书测试 `setblock_unloaded_chunk_rejected`:外部 SetBlock 落在未加载
    /// chunk → 驱动丢弃(commands_rejected 计数),**不自动建空气 chunk**
    /// (副作用面:该 chunk 在 sim 内仍不存在;加载态 chunk 的同型命令照常
    /// 生效作对照)。
    #[tokio::test(start_paused = true)]
    async fn setblock_unloaded_chunk_rejected() {
        let config = test_config(600);
        let session = session_two_loaded_chunks(&config);
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 未加载 chunk(3,3) 的 SetBlock(路由分片 (0,0) 已注册、y 界内)。
        inject_send(
            &tx,
            shard_id(0, 0),
            messaging::SimCommand::SetBlock {
                pos: types::BlockPos::new(56, 5, 56), // chunk (3,3) 未装载
                block: types::BlockId(1),
            },
        )
        .await;
        // 对照:已加载 chunk (0,0) 内的 SetBlock 照常生效。
        inject_send(
            &tx,
            shard_id(0, 0),
            messaging::SimCommand::SetBlock {
                pos: types::BlockPos::new(4, 5, 4),
                block: types::BlockId(1),
            },
        )
        .await;
        advance_ticks(2).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.commands_rejected, 1, "未加载 chunk 命令恰拒 1 条");

        // 副作用面:终末快照恢复后,未加载 chunk 不得被自动建成空气 chunk。
        let bundle = latest_bundle(&checkpoints);
        let bytes = bundle
            .world
            .shards
            .per_shard
            .get(&shard_id(0, 0))
            .expect("分片 (0,0) 在快照");
        let restored =
            ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        assert!(
            restored
                .voxel()
                .get_chunk(types::ChunkPos::new(3, 3))
                .is_none(),
            "未加载 chunk 未被自动建空气 chunk(set_block 的自动建路径未触发)"
        );
        assert_eq!(
            restored.voxel().get_block(types::BlockPos::new(4, 5, 4)),
            types::BlockId(1),
            "对照:已加载 chunk 的编辑照常生效"
        );
    }

    /// 任务书测试 `network_block_updates_from_outbox`:网络 SetBlock 成功 →
    /// 客户端收 BlockUpdates,**不再依赖驱动镜像**(镜像已删,源 =
    /// `take_block_updates`)。观察者在场(ensure 建片 + 激活装载 → chunk
    /// 已加载,SetBlock 不被新门卫拦截)。
    #[tokio::test(start_paused = true)]
    async fn network_block_updates_from_outbox() {
        let (args, tx, _hub, addr) = observe_harness(observe_config()).await;
        let driver = tokio::spawn(run(args));

        let mut c = MiniClient::connect(addr).await.expect("客户端连入");
        let _id = join_player(&mut c, 4).await;
        // 观察者就位 → 编排 ensure 分片 + 激活装载(1 拍 ensure,装载晚一拍)。
        inject_move(&tx, Vec3::new(8.0, 3.0, 8.0)).await;
        advance_ticks(3).await;

        // 网络 SetBlock(走 send_broadcast 广播;单分片下目标 = owner 一个)。
        tx.send(DriverCommand::Send {
            shard: shard_id(0, 0),
            command: messaging::SimCommand::SetBlock {
                pos: types::BlockPos::new(4, 3, 4),
                block: types::BlockId(1),
            },
        })
        .await
        .expect("命令仍存活");
        advance_ticks(6).await;

        // 客户端收 BlockUpdates(软窗:与 EntityStates/ChunkData 混流跳过)。
        let mut got = false;
        for _ in 0..320 {
            let frame = tokio::time::timeout(std::time::Duration::from_millis(50), c.recv()).await;
            match frame {
                Ok(Ok(S2C::BlockUpdates { updates })) => {
                    if updates.contains(&(types::BlockPos::new(4, 3, 4), types::BlockId(1))) {
                        got = true;
                        break;
                    }
                }
                Ok(Ok(_)) => {}
                _ => {}
            }
        }
        assert!(got, "SetBlock 经 outbox 源广播 BlockUpdates(非驱动镜像)");

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        driver.await.expect("驱动不 panic");
    }

    /// 任务书测试 `single_shard_regression`:单分片 SetBlock/网络命令行为不变
    /// ——合法 SetBlock 生效(终态快照可查)、日志一条、无拒绝计数。
    #[tokio::test(start_paused = true)]
    async fn single_shard_regression() {
        let config = test_config(600);
        // 单分片 + 手工装载目标 chunk(不经激活编排,消除观察者依赖)。
        let mut session = sim_api::session::Session::new(config.session_config());
        session.register(sim(0, 0));
        session.apply_chunk_load(
            shard_id(0, 0),
            types::ChunkPos::new(0, 0),
            voxel::Chunk::air(),
        );
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        inject_send(
            &tx,
            shard_id(0, 0),
            messaging::SimCommand::SetBlock {
                pos: types::BlockPos::new(4, 5, 4),
                block: types::BlockId(1),
            },
        )
        .await;
        advance_ticks(2).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(outcome.commands_rejected, 0, "合法命令零拒绝");

        // 日志一条(单分片 = 单目标一条 LogEntry)且生效。
        let bundle = latest_bundle(&checkpoints);
        let entries = sim_api::log::CommandLog::from_bytes(&bundle.log_bytes)
            .expect("自有编码必可解码")
            .take();
        assert_eq!(entries.len(), 1, "单分片 SetBlock = 一条日志");
        assert_eq!(entries[0].dest, shard_id(0, 0));
        let bytes = bundle
            .world
            .shards
            .per_shard
            .get(&shard_id(0, 0))
            .expect("快照");
        let restored =
            ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        assert_eq!(
            restored.voxel().get_block(types::BlockPos::new(4, 5, 4)),
            types::BlockId(1),
            "SetBlock 行为不变(体素生效)"
        );
    }

    /// 任务书测试 `non_owner_unload_does_not_persist` + `owner_unload_persists_edit`
    /// 的合场景:两片各持一份边界 chunk(片 (0,0) 持本体 (0,0) 与片 (1,0)
    /// 本体的扩张副本 (8,0)),SetBlock(130,10,4) 广播两片后,观察者不在场
    /// + despawn 窗口 0 → 两片先后 despawn(shard 升序:非 owner 片 (0,0)
    /// 先卸——其扩张副本被丢弃**不写库**;owner 片 (1,0) 后卸——写库)。
    /// 库内 chunk (8,0) 必为编辑值:若非 owner 也写库且后写覆盖,先卸的
    /// 旧副本不会出现——D6「卸载顺序任意、不后写覆盖」的回归面。
    #[tokio::test(start_paused = true)]
    async fn non_owner_unload_does_not_persist() {
        let mut config = test_config(600);
        config.despawn_after_secs = 0; // 空片当拍即 despawn 候选
        let mut session = session_two_loaded_chunks(&config);
        // 片 (0,0) 作为 holder 装载片 (1,0) 本体的扩张副本(margin=1 允许;
        // 装载校验:owner 片 (1,0) 已注册)。
        session.apply_chunk_load(
            shard_id(0, 0),
            types::ChunkPos::new(8, 0),
            voxel::Chunk::air(),
        );

        // 共享 chunk 库句柄:驱动 put 后测试侧直接读(mem 后端等价"重开库")。
        #[derive(Clone, Default)]
        struct SharedChunks(Arc<Mutex<storage::mem::MemChunkStore>>);

        impl storage::traits::ChunkStore for SharedChunks {
            fn put_batch(
                &mut self,
                batch: &[(storage::traits::ChunkKey, voxel::Chunk)],
            ) -> Result<(), storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").put_batch(batch)
            }
            fn get(
                &self,
                key: storage::traits::ChunkKey,
            ) -> Result<Option<voxel::Chunk>, storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").get(key)
            }
            fn len(&self) -> Result<usize, storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").len()
            }
            fn is_empty(&self) -> Result<bool, storage::traits::StoreError> {
                self.0.lock().expect("chunk 锁毒化").is_empty()
            }
        }
        let chunks_shared = SharedChunks::default();
        let (tx, rx) = tokio::sync::mpsc::channel(16);
        let store = storage::worker::StoreWorker::new(
            Box::new(chunks_shared.clone()),
            Box::new(SharedCheckpoints::default()),
        );
        let (provider, new_sim) = wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        });
        let driver = tokio::spawn(run(DriverArgs {
            config,
            session,
            store,
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
            observe: None,
        }));

        // 编辑 chunk (8,0)(owner = 片 (1,0);两片各持一份 → 广播两片)。
        inject_send(
            &tx,
            shard_id(1, 0),
            messaging::SimCommand::SetBlock {
                pos: types::BlockPos::new(130, 10, 4),
                block: types::BlockId(2),
            },
        )
        .await;
        advance_ticks(3).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(
            outcome.shards_despawned >= 2,
            "前置:两片(空+无票,窗口 0)先后 despawn,实 {}",
            outcome.shards_despawned
        );

        // 库内 chunk (8,0) = 编辑值:owner 卸载写库;非 owner 先卸的扩张副本
        // 未写库(否则旧空气值已覆盖编辑)。
        let key = storage::traits::ChunkKey {
            world: types::WorldId(0),
            dim: types::DimensionId(0),
            pos: types::ChunkPos::new(8, 0),
        };
        let stored = storage::traits::ChunkStore::get(&*chunks_shared.0.lock().expect("锁"), key)
            .expect("读库不失败")
            .expect("owner 卸载已落库");
        assert_eq!(
            stored.get(2, 74, 4),
            types::BlockId(2),
            "owner 卸载写库:取回编辑值(局部 (2,74,4) = 世界 (130,10,4))"
        );
        // owner_unload_persists_edit 的重载面:同一库句柄再取(chunk 全量
        // 往返 == 期望构造)——重开 redb 的进程重启面由 e2e AC-G6 覆盖。
        let mut expected = voxel::Chunk::air();
        expected.set(2, 74, 4, types::BlockId(2));
        assert_eq!(stored, expected, "owner 卸载的 chunk 内容完整(全量)");
        assert!(
            outcome.chunks_unloaded >= 2,
            "owner chunk 卸载计数兑现(扩张副本不计)"
        );
    }

    /// 任务书测试 `owner_unload_persists_edit`(owner 卸载 → `Some(编辑后
    /// chunk)`,重载取回编辑):单分片持自己的 chunk,编辑 → despawn 落库 →
    /// 分片重活(观察者回归 ensure)→ 激活装载命中持久层编辑值(终末快照
    /// 验证模拟侧取回;库侧直接读)。与 `non_owner_unload_does_not_persist`
    /// 分立:本测试单分片,钉「owner 卸载确写库、重载确取回」的正样本。
    #[tokio::test(start_paused = true)]
    async fn owner_unload_persists_edit() {
        let mut config = test_config(600);
        config.despawn_after_secs = 0;
        config.activation.cooldown_secs = 0;
        config.swap_budget_chunks_per_tick = 14;
        let session = sim_api::session::Session::new(config.session_config());
        let (args, tx, checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // ① 进区:ensure 建片 (1,0) + 激活装载 chunk (12,0)(superflat)。
        inject_move(&tx, Vec3::new(200.0, GROUND_TOP_Y, 8.0)).await;
        advance_ticks(4).await;
        // ② 编辑(chunk (12,0) 已激活装载;stone → dirt 可区分)。
        inject_send(
            &tx,
            shard_id(1, 0),
            messaging::SimCommand::SetBlock {
                pos: types::BlockPos::new(200, types::WORLD_Y_MIN, 8),
                block: types::BlockId(2),
            },
        )
        .await;
        advance_ticks(2).await;
        // ③ 远走:卸载(despawn 窗口 0,空片无票即候选)→ owner 卸载写库。
        inject_move(&tx, Vec3::new(50_000.0, 0.0, 8.0)).await;
        advance_ticks(150).await;
        // ④ 回归:ensure 复活 → 激活装载命中持久层 → 模拟侧取回编辑值。
        inject_move(&tx, Vec3::new(200.0, GROUND_TOP_Y, 8.0)).await;
        advance_ticks(8).await;
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(
            outcome.shards_despawned >= 1,
            "前置:owner 片 despawn(卸载写库)"
        );
        assert!(outcome.shards_spawned >= 2, "前置:复活 ensure(初生 + 回归)");

        // 模拟侧重载面:终末快照恢复后 chunk (12,0) 的编辑格 = dirt。
        let bundle = latest_bundle(&checkpoints);
        let bytes = bundle
            .world
            .shards
            .per_shard
            .get(&shard_id(1, 0))
            .expect("复活的 owner 片在终末快照");
        let restored =
            ShardSim::restore(bytes, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        assert_eq!(
            restored
                .voxel()
                .get_block(types::BlockPos::new(200, types::WORLD_Y_MIN, 8)),
            types::BlockId(2),
            "重载取回编辑值(owner 卸载写库 → 复活装载命中持久层)"
        );
    }

    /// 任务书测试 `pending_destination_ensured_before_next_tick`:拍内产生
    /// 迁移信封(pending 目的分片)后 orchestrate → 目的分片已注册,
    /// 下一拍 tick() 不触发信封黑洞 fail-stop(ensure 后才进下一拍)。
    /// 驱动级编排经 run 驱动:观察者贴边界造迁移 → despawn 候选清空目的片
    /// 的对立面不可达(空片才 despawn)——直接以「观察者驱动 + pending 目的
    /// 分片被 ①′ ensure」为断言面。
    #[tokio::test(start_paused = true)]
    async fn pending_destination_ensured_before_next_tick() {
        // 场景:观察者站分片 (0,0) 深处 → ensure 建片 + 装载;注入高速实体
        // 冲向分片 (1,0)(拍内产生迁移信封,pending 目的 = 片 (1,0));下一拍
        // tick 前编排 ①′ 必须已 ensure 片 (1,0)——若漏,下一拍 tick 开头
        // 「信封黑洞」fail-stop panic,驱动任务连坐(测试以驱动不 panic +
        // shards_spawned 计数兑现为判据)。
        let config = test_config(600);
        let session = sim_api::session::Session::new(config.session_config());
        let (args, tx, _checkpoints) = test_args(config, session);
        let driver = tokio::spawn(run(args));

        // 观察者就位(ensure 片 (0,0))+ 装载窗口。
        inject_move(&tx, Vec3::new(8.0, 3.0, 8.0)).await;
        advance_ticks(2).await;
        // 高速实体(linvel 600 m/s ≈ 10 m/tick):出生 8 拍后必越 x=128 边界,
        // 产生发往片 (1,0) 的迁移信封。
        inject_send(
            &tx,
            shard_id(0, 0),
            messaging::SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 1),
                state: EntityState {
                    translation: Vec3::new(48.0, 3.0, 8.0),
                    linvel: Vec3::new(600.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            },
        )
        .await;
        // 实体出生拍不在片 (0,0) 装载区也无妨(虚空运动):8 拍内越界。
        // 每拍末编排读 pending destinations 并 ensure 片 (1,0)——黑洞不变量
        // 在后续每拍 tick 开头校验,漏 ensure 即 panic(驱动连坐)。
        advance_ticks(20).await;

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver
            .await
            .expect("驱动不 panic(pending 目的片已被 ①′ ensure)");
        assert!(
            outcome.shards_spawned >= 2,
            "片 (0,0)(观察者)与片 (1,0)(pending 目的)都被 ensure,实 {}",
            outcome.shards_spawned
        );
    }

    // ---- 任务书 req003-net/03:观察输出编排与出站背压(测试清单)----

    use crate::net::{NetConfig, OutboundHub, serve};
    use sim_api::proto::{C2S, S2C};

    /// 观察输出测试配置:observe_interval = 3、full = 9(短周期,少拍验证,
    /// 注释所述节奏;经 harness 的 NetConfig 投影为 ObserveCadence);
    /// despawn 窗口 0(left 清理测试的缩短验证,任务书测试清单 6)。
    fn observe_config() -> Config {
        let mut config = test_config(600);
        config.despawn_after_secs = 0;
        config
    }

    /// 观察输出测试的装配:mem 后端 + 空会话 + 真网络 serve(127.0.0.1:0)+
    /// ObserveLink(up/hub/cadence 三缝真接);返回驱动参数、命令发送端、
    /// hub 句柄、实际监听地址。
    ///
    /// 链路 = 任务 01/02 的连接层直接 serve + 驱动 `observe` 缝:客户端经 TCP
    /// 入会 → NetUp 上行 → 驱动第 ⑤ 步编排 → hub 下行 → 客户端收帧。真端口、
    /// 真字节(README 决策 10;127.0.0.1:0 由 OS 分配)。
    async fn observe_harness(
        config: Config,
    ) -> (
        DriverArgs,
        tokio::sync::mpsc::Sender<DriverCommand>,
        std::sync::Arc<OutboundHub>,
        std::net::SocketAddr,
    ) {
        let net_cfg = NetConfig {
            listen_addr: "127.0.0.1:0".to_string(),
            // 测试节奏(注释所述 3/9):NetConfig 增量字段 → ObserveCadence。
            observe_interval_ticks: 3,
            full_snapshot_interval_ticks: 9,
            ..NetConfig::default()
        };
        let cadence = ObserveCadence::from(&net_cfg);
        let (up_tx, up_rx) = tokio::sync::mpsc::channel(256);
        let hub = std::sync::Arc::new(OutboundHub::new());
        let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
        let addr = serve(net_cfg, up_tx, bcast_rx, hub.clone())
            .await
            .expect("监听成功(127.0.0.1:0)");
        // 关机发送端随 harness 存续(测试结束即 drop → 连接任务收口)。
        std::mem::forget(bcast_tx);

        let session = sim_api::session::Session::new(config.session_config());
        let (tx, rx) = tokio::sync::mpsc::channel(256);
        let checkpoints = SharedCheckpoints::default();
        let store = storage::worker::StoreWorker::new(
            Box::new(MemChunkStore::new()),
            Box::new(checkpoints.clone()),
        );
        // new_sim:shard_sim_config 基础上零重力 + 关凝固位移门(0 = 直通)
        // ——测试场景确定化:实体出生即静止,20 拍内凝固(session.rs 测试
        // 清单 5 同款);无坠落(坠落会滑出视距球,观察断言不确定)。
        // provider 仍由 wire 装配(数据源不变)。
        let (provider, _) = wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        });
        let sim_source = config.clone();
        let new_sim = Box::new(move |shard: types::ShardId| {
            let mut sim_cfg = sim_source.shard_sim_config(shard);
            sim_cfg.gravity = Vec3::ZERO;
            sim_cfg.solidify.displacement_gate_m = 0.0;
            physics_sim::ShardSim::new(sim_cfg, Box::new(SimpleRegistry::builtin()))
        });
        let args = DriverArgs {
            config,
            session,
            store,
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
            observe: Some(ObserveLink {
                up: up_rx,
                hub: hub.clone(),
                cadence,
            }),
        };
        (args, tx, hub, addr)
    }

    /// MiniClient 复用(net::tests 的内嵌客户端;crate 内可见)。
    use crate::net::tests::MiniClient;

    /// 测试清单 1 `welcome_on_join_carries_player_and_tick`:真客户端入会 →
    /// Welcome{player_id, tick} 到达该连接的 Critical 通道并被冲刷出网;
    /// tick = 驱动当前拍(Welcome 拍号接线)。
    #[tokio::test(start_paused = true)]
    async fn welcome_on_join_carries_player_and_tick() {
        let (args, tx, hub, addr) = observe_harness(observe_config()).await;
        let driver = tokio::spawn(run(args));

        let mut c = MiniClient::connect(addr).await.expect("客户端连入");
        c.send(&C2S::Hello { client_id: 1 })
            .await
            .expect("Hello 写入");
        c.send(&C2S::Subscribe { view_distance: 4 })
            .await
            .expect("Subscribe 写入");
        // 连接层同步 Welcome(tick 暂 0):首帧必是它(Subscribe 前无其他源)。
        let first = c.recv_timeout().await;
        assert!(
            matches!(first, S2C::Welcome { .. }),
            "首帧 = Welcome(连接层),实为 {first:?}"
        );
        advance_ticks(2).await;
        // 驱动 Welcome(拍号刷新)与 chunk 流 ChunkData(泵)竞先:收帧直到
        // 见到第二个 Welcome(任务书测试意图 = Welcome{player, tick} 到达
        // Critical 通道;帧序不属断言面)。
        let mut got_welcome = None;
        for _ in 0..32 {
            if let S2C::Welcome { player_id, tick } = c.recv_timeout().await {
                got_welcome = Some((player_id, tick));
                break;
            }
        }
        match got_welcome {
            Some((player_id, tick)) => {
                assert_eq!(
                    player_id,
                    messaging::InputSource(1),
                    "player_id = 网络分配身份"
                );
                // tick = 驱动在「消费 Joined 的那拍末端」的 tick_now(已完成
                // 拍数;竞先下为 1 或 2——非 0 即拍号接线成立,连接层暂值
                // 0 已被驱动值覆盖)。
                assert!(tick.0 >= 1, "tick = 驱动侧拍号(接线成立),实为 {tick:?}");
            }
            None => panic!("32 帧内未见到驱动侧 Welcome(Critical 通道)"),
        }
        assert!(
            hub.subscribers().contains(&messaging::InputSource(1)),
            "订阅玩家已注册进 hub"
        );
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        driver.await.expect("驱动不 panic");
    }

    /// 测试清单 2 `entity_states_incremental_then_full`:拍 1 出生实体 →
    /// 首个输出点全量(removed 空);实体移动 → 增量只含变化;全量周期到 →
    /// 整表(removed 空)。周期 = observe 3 拍 / full 9 拍。
    #[tokio::test(start_paused = true)]
    async fn entity_states_incremental_then_full() {
        let (args, tx, _hub, addr) = observe_harness(observe_config()).await;
        let driver = tokio::spawn(run(args));

        let mut c = MiniClient::connect(addr).await.expect("客户端连入");
        let _ = join_player(&mut c, 4).await;

        // 拍 1:先移动观察者(编排 ① 下一拍 ensure 分片),再出生实体——
        // Send 路由的先决条件 = 分片已注册(调度器语义:未注册 fail-stop)。
        let shard = shard_id(0, 0);
        let entity = EntityId::from_parts(1, 1);
        inject_move(&tx, Vec3::new(8.0, 3.0, 8.0)).await;
        advance_ticks(1).await; // ensure 分片 (0,0)
        inject_send(&tx, shard, spawn_cmd(entity)).await;
        advance_ticks(3).await; // 越过首个增量周期(3 拍)

        // 首个输出点:该玩家首包 → 全量整表(removed 空)。
        let first = recv_entity_states(&mut c).await;
        assert_eq!(
            first.1.iter().map(|(id, _)| *id).collect::<Vec<_>>(),
            vec![entity],
            "首输出 = 全表"
        );
        assert!(first.2.is_empty(), "全量 removed 恒空");

        // 移动实体(TeleportEntity)→ 下一增量只含变化。
        inject_send(
            &tx,
            shard,
            messaging::SimCommand::TeleportEntity {
                entity,
                state: EntityState {
                    translation: Vec3::new(2.5, 3.0, 2.5),
                    ..EntityState::default()
                },
            },
        )
        .await;
        advance_ticks(4).await; // 下一增量周期
        let (tick, states, removed) = recv_entity_states(&mut c).await;
        assert_eq!(
            states.iter().map(|(id, _)| *id).collect::<Vec<_>>(),
            vec![entity],
            "增量 = 变化实体"
        );
        assert!(
            (states[0].1.translation - Vec3::new(2.5, 3.0, 2.5)).length() < 0.05,
            "增量携带最新状态(物理步进容差),实为 {:?}",
            states[0].1.translation
        );
        assert!(removed.is_empty(), "无消失实体");
        let _ = tick;

        // 全量周期到(累计 ≥9 拍)→ 整表且 removed 空。
        advance_ticks(4).await;
        let (_t, states_full, removed_full) = recv_entity_states(&mut c).await;
        assert_eq!(
            states_full.iter().map(|(id, _)| *id).collect::<Vec<_>>(),
            vec![entity],
            "全量 = 整表"
        );
        assert!(removed_full.is_empty(), "全量 removed 恒空");

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        driver.await.expect("驱动不 panic");
    }

    /// 测试清单 3 `removed_reported_on_solidify_and_removal`:凝固实体 →
    /// 下轮增量 removed 含其 ID 且 BlockUpdates 含该格;RemoveEntity 同型
    /// (removed 通知、无方块变更)。
    #[tokio::test(start_paused = true)]
    async fn removed_reported_on_solidify_and_removal() {
        // 无重力出生 → 真静止 → 凝固(session.rs 测试清单 5 同款场景;
        // 位移门默认 0.1m,真静止即可满足,40 拍上限留裕度)。
        let (args, tx, _hub, addr) = observe_harness(observe_config()).await;
        let driver = tokio::spawn(run(args));

        let mut c = MiniClient::connect(addr).await.expect("客户端连入");
        let _ = join_player(&mut c, 4).await;

        let shard = shard_id(0, 0);
        let entity = EntityId::from_parts(1, 1);
        // ensure 分片(观察者入表 → 编排 ① 建片)→ 零重力出生(harness
        // 确定化:出生即静止)→ 连续 20 拍静止 → 凝固(位移门已关)。
        inject_move(&tx, Vec3::new(8.0, 3.0, 8.0)).await;
        advance_ticks(1).await;
        inject_send(
            &tx,
            shard,
            messaging::SimCommand::SpawnDynamic {
                entity,
                state: EntityState {
                    translation: Vec3::new(1.5, 3.0, 1.5),
                    ..EntityState::default()
                },
                desc: types::EntityDesc::default(),
            },
        )
        .await;
        advance_ticks(3).await;
        let _ = recv_entity_states(&mut c).await; // 吃掉首全量

        // 跑到凝固被观察(增量 removed;期间可能有无变化静默——改为帧等待
        // 上限,包到即判)。凝固包顺序:EntityStates(removed)与 BlockUpdates
        // 同轮两包,顺序不限。
        let mut saw_removed = false;
        for _ in 0..160 {
            match tokio::time::timeout(std::time::Duration::from_millis(50), c.recv()).await {
                Ok(Ok(S2C::EntityStates { removed, .. })) if !removed.is_empty() => {
                    assert_eq!(removed, vec![entity], "凝固 → removed 含实体 ID");
                    saw_removed = true;
                    break;
                }
                Ok(Ok(other)) => eprintln!("[dbg] f {:?}", other),
                Err(e) => eprintln!("[dbg] err {:?}", e),
                _ => {}
            }
        }
        assert!(saw_removed, "凝固后 160 帧内 removed 必达");
        // BlockUpdates:凝固格(实体出生列)在 removed 报文前后必达;收帧直到见。
        let solidified_cell = 'search: {
            for _ in 0..160 {
                match tokio::time::timeout(std::time::Duration::from_millis(50), c.recv()).await {
                    Ok(Ok(S2C::BlockUpdates { updates })) => {
                        if updates
                            .iter()
                            .any(|(pos, _)| *pos == types::BlockPos::new(1, 3, 1))
                        {
                            break 'search true;
                        }
                    }
                    Ok(Ok(_)) => {}
                    _ => break 'search false,
                }
            }
            false
        };
        assert!(solidified_cell, "凝固 → BlockUpdates 含凝固格 (1,3,1)");

        // RemoveEntity 同型:再生成一实体 → 移除 → 增量 removed。
        let entity2 = EntityId::from_parts(1, 2);
        inject_send(&tx, shard, spawn_cmd(entity2)).await;
        advance_ticks(6).await;
        inject_send(
            &tx,
            shard,
            messaging::SimCommand::RemoveEntity { entity: entity2 },
        )
        .await;
        // 等 removed2 到达(帧上限;期间的全量/无变化帧跳过)。
        let mut removed2_hit = false;
        for _ in 0..160 {
            match tokio::time::timeout(std::time::Duration::from_millis(50), c.recv()).await {
                Ok(Ok(S2C::EntityStates { removed, .. })) if removed.contains(&entity2) => {
                    removed2_hit = true;
                    break;
                }
                Ok(Ok(_)) => {}
                _ => {}
            }
        }
        assert!(removed2_hit, "RemoveEntity → removed 同型通知(160 帧内)");

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        driver.await.expect("驱动不 panic");
    }

    /// 测试清单 4 `distance_filter_respects_view_distance`:视距外实体不入包;
    /// 移近后入包。玩家在原点(视距 1 chunk = 16m),实体生于 x=1000(远),
    /// 传送近处(x=10)后入包。
    #[tokio::test(start_paused = true)]
    async fn distance_filter_respects_view_distance() {
        let (args, tx, _hub, addr) = observe_harness(observe_config()).await;
        let driver = tokio::spawn(run(args));

        let mut c = MiniClient::connect(addr).await.expect("客户端连入");
        let _ = join_player(&mut c, 1).await; // 视距 1 chunk = 16m
        // 观察者到原点(距离过滤中心 + 编排 ① ensure 分片的先决)。
        inject_move(&tx, Vec3::new(8.0, 3.0, 8.0)).await;
        advance_ticks(1).await;

        let shard = shard_id(0, 0);
        let far = EntityId::from_parts(1, 1);
        inject_send(
            &tx,
            shard,
            messaging::SimCommand::SpawnDynamic {
                entity: far,
                state: EntityState {
                    // x=100:chunk 6 → 分片 (0,0) 本体区内(不触发迁移),
                    // 距观察者 100m > 视距 16m(视距外)。
                    translation: Vec3::new(100.0, 3.0, 0.0),
                    ..EntityState::default()
                },
                desc: types::EntityDesc::default(),
            },
        )
        .await;
        advance_ticks(12).await; // ≥2 个输出周期(空表全量也可能到达)
        // 视距内无实体:预算窗内不允许出现任何含 far 的包(全量空表合法,
        // 绝不含 far;预算 16 = 吃光开局 ChunkData 队列后落入静默窗)。
        assert!(
            !entity_appears_within(&mut c, far, 16).await,
            "视距外实体不得入包"
        );

        // 传送近处(到原点 10m < 16m)→ 移近后入包(实体从未入过包,故
        // "移近"对客户端 = 新出现:增量 states 或全量整表都可能携带)。
        inject_send(
            &tx,
            shard,
            messaging::SimCommand::TeleportEntity {
                entity: far,
                state: EntityState {
                    translation: Vec3::new(10.0, 3.0, 0.0),
                    ..EntityState::default()
                },
            },
        )
        .await;
        advance_ticks(12).await;
        assert!(
            entity_appears_within(&mut c, far, 16).await,
            "移近后实体入包(16 帧预算:EntityStates 与泵 ChunkData 竞先)"
        );

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        driver.await.expect("驱动不 panic");
    }

    /// 测试清单 5 `block_updates_broadcast_by_proximity`:A 视距内、B 视距外的
    /// 同一格位 SetBlock:只 A 收到 BlockUpdates。
    #[tokio::test(start_paused = true)]
    async fn block_updates_broadcast_by_proximity() {
        let (args, tx, _hub, addr) = observe_harness(observe_config()).await;
        let driver = tokio::spawn(run(args));

        let mut a = MiniClient::connect(addr).await.expect("客户端 A 连入");
        let mut b = MiniClient::connect(addr).await.expect("客户端 B 连入");
        let id_a = join_player(&mut a, 4).await; // A 视距 4 chunk = 64m
        // A 到原点(过滤中心 + ensure 分片);B 移到远区(2000,0,0)——
        // 格 (4,·,4) 距 B ~2000m,在 B 视距外。
        a.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, -60.0, 8.0),
        })
        .await
        .expect("A 移动写入");
        b.send(&C2S::MoveObserver {
            position: Vec3::new(2000.0, -60.0, 0.0),
        })
        .await
        .expect("B 移动写入");
        // 两拍:第 1 拍命令分支收下 MoveObserver(盖章),第 2 拍 tick 分支
        // 应用观察者 + 编排 ① ensure 分片(命令分支优先级最低,1 拍不够)。
        advance_ticks(2).await;
        // REQ-005 任务 01 适配(既有测试最小适配,单列汇报):外部 SetBlock
        // 新增「chunk 已加载」门卫(未加载 chunk 不再自动建空气 chunk)——
        // 等激活装载完成(2 + 4 拍裕度)再编辑,载荷与断言不变。
        advance_ticks(4).await;

        // 原点旁 SetBlock(距 A 5m,距 B ~2000m)。
        let _ = id_a;
        tx.send(DriverCommand::Send {
            shard: shard_id(0, 0),
            command: messaging::SimCommand::SetBlock {
                pos: types::BlockPos::new(4, -60, 4),
                block: types::BlockId(1),
            },
        })
        .await
        .expect("命令仍存活");
        advance_ticks(6).await;

        // A 收到含该格的 BlockUpdates(软窗:EntityStates/ChunkData 混流跳过)。
        let mut a_got = false;
        for _ in 0..320 {
            let frame = tokio::time::timeout(std::time::Duration::from_millis(50), a.recv()).await;
            match frame {
                Ok(Ok(S2C::BlockUpdates { updates })) => {
                    if updates.contains(&(types::BlockPos::new(4, -60, 4), types::BlockId(1))) {
                        a_got = true;
                        break;
                    }
                }
                Ok(Ok(_)) => {}
                _ => {}
            }
        }
        assert!(a_got, "视距内玩家 A 必收 BlockUpdates");
        // B 不收:B 侧静默判定(1s 窗口内无该格 BlockUpdates)。
        let mut b_got = false;
        for _ in 0..8 {
            match tokio::time::timeout(std::time::Duration::from_millis(100), b.recv()).await {
                Ok(Ok(S2C::BlockUpdates { updates })) => {
                    if updates.contains(&(types::BlockPos::new(4, -60, 4), types::BlockId(1))) {
                        b_got = true;
                    }
                }
                Ok(Ok(_)) => {}
                Ok(Err(_)) | Err(_) => break,
            }
        }
        assert!(!b_got, "视距外玩家 B 不得收该格 BlockUpdates");

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        driver.await.expect("驱动不 panic");
    }

    /// 测试清单 6 `left_removes_observer_and_stream`:断连 → 观察者表移除
    /// (驱动可见:模拟侧 observers() 条目消失)→ 出站注册清理;空片无票
    /// 可 despawn(despawn_after_secs = 0 缩短验证;评审修复②回填)。
    #[tokio::test(start_paused = true)]
    async fn left_removes_observer_and_stream() {
        let (args, tx, hub, addr) = observe_harness(observe_config()).await;
        let driver = tokio::spawn(run(args));

        let mut c = MiniClient::connect(addr).await.expect("客户端连入");
        let id = join_player(&mut c, 2).await;
        c.send(&C2S::MoveObserver {
            position: Vec3::new(8.0, 3.0, 8.0),
        })
        .await
        .expect("观察者就位写入");
        advance_ticks(2).await;
        assert!(hub.subscribers().contains(&id), "前置:订阅玩家已注册");

        c.send(&C2S::Goodbye).await.expect("Goodbye 写入");
        assert!(c.is_disconnected().await, "Goodbye → 断连");
        advance_ticks(3).await;

        // Left 上行 → 观察流清理:hub 注册消失(驱动可见口径)。
        assert!(
            !hub.subscribers().contains(&id),
            "断连 → 观察流出站注册清理"
        );

        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        // 评审修复②回填:Left → Session::remove_observer → 票撤;空片在
        // despawn_after_secs = 0 下被回收(任务书测试清单第 6 项的
        // despawn 断言,原实现因移除缝缺失而 `let _ = outcome` 放弃)。
        // REQ-005 D5:观察者 (8,·,8) 贴分片角落(x/z 均距界 8m ≤ 32m)→
        // 四分片同拍 ensure;断连票撤后四片皆回收(原"唯一分片"口径随
        // 邻片 ensure 更新)。
        assert_eq!(
            outcome.shards_despawned, 4,
            "断连票撤 → 角落四分片 despawn(缩短配置验证)"
        );
        assert!(
            outcome.chunks_unloaded >= 1,
            "回收分片的 chunk 卸载计数兑现"
        );
    }

    /// 评审修复①(REQ-003,socket 级 PoC 对应):入会客户端经网络直发三类
    /// hostile 命令(未注册分片路由 / SetBlock y 越界 / 未知实体),修复前
    /// 各自触发调度器/模拟 fail-stop panic → 驱动任务连坐崩溃(DoS);修复
    /// 后 = 驱动侧前置校验丢弃计数,连接不断、驱动存活。命令白名单
    /// (Reject 支路)由 net::tests::command_whitelist_enforced 覆盖,此处
    /// 补的正是五放行命令的**路由安全**面。
    #[tokio::test(start_paused = true)]
    async fn hostile_net_commands_dropped_not_fatal() {
        let (args, tx, hub, addr) = observe_harness(observe_config()).await;
        let driver = tokio::spawn(run(args));

        let mut c = MiniClient::connect(addr).await.expect("客户端连入");
        let id = join_player(&mut c, 2).await;
        advance_ticks(2).await;
        assert!(hub.subscribers().contains(&id), "前置:订阅玩家已注册");

        // 三类 hostile 命令(全部入会即可发;分片 (0,0) 未注册——观察者
        // 未上报位置,ensure 未建任何片)。
        c.send(&C2S::Command(messaging::SimCommand::RemoveEntity {
            entity: EntityId::from_parts(999, 1), // origin 999 → ShardPos(999,0) 未注册
        }))
        .await
        .expect("hostile RemoveEntity 写入");
        c.send(&C2S::Command(messaging::SimCommand::SetBlock {
            pos: types::BlockPos::new(0, 1000, 0), // y 越界
            block: types::BlockId(1),
        }))
        .await
        .expect("hostile SetBlock 写入");
        c.send(&C2S::Command(messaging::SimCommand::SpawnDynamic {
            entity: EntityId::from_parts(1, 1),
            state: EntityState {
                translation: Vec3::new(2000.0, 3.0, 0.0), // 中心 → 分片 (15,0) 未注册
                ..EntityState::default()
            },
            desc: types::EntityDesc::default(),
        }))
        .await
        .expect("hostile SpawnDynamic 写入");
        advance_ticks(3).await;

        // 客户端连接不断(丢弃 ≠ 断连;断连面只属协议违规/keepalive)。
        assert!(
            hub.subscribers().contains(&id),
            "hostile 命令被丢弃但不断连(降级处理)"
        );
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("hostile 命令不崩驱动(评审修复①)");
        assert_eq!(outcome.commands_rejected, 3, "三条 hostile 命令各自计数");
    }

    /// 测试清单 7 `slow_client_degrades_not_blocks`:不读 socket 的客户端:
    /// World 队列丢最旧(hub 计数增长)、驱动 tick 计数不减;连接最终被断。
    ///
    /// 场景 = 大视距订阅的 chunk 流洪水(任务 02 泵持续装 4225 chunk,慢客户端
    /// 不读 socket → World 队列(512)满 → 丢最旧;README 决策 6「ChunkData
    /// 丢失由客户端重新 Subscribe 修复」)。超容量 512 的 ChunkData 只能被
    /// 丢弃——帧在 World 队列积压,不因 OS 缓冲吞帧而泄压(队列先于 socket
    /// 缓冲饱和:服务端冲刷受写超时 5s 窗口约束,慢客户端写必然超时断连)。
    /// tick 计数不受拖累(暂停时间下拍数精确)。
    #[tokio::test(start_paused = true)]
    async fn slow_client_degrades_not_blocks() {
        let (args, tx, hub, addr) = observe_harness(observe_config()).await;
        let driver = tokio::spawn(run(args));

        // 慢客户端:订阅大视距(32 → 4225 chunk 请求)后不再读 socket。
        let mut slow = MiniClient::connect(addr).await.expect("慢客户端连入");
        slow.send(&C2S::Hello { client_id: 1 })
            .await
            .expect("Hello 写入");
        slow.send(&C2S::Subscribe { view_distance: 32 })
            .await
            .expect("Subscribe 写入(大视距 → chunk 流洪水)");
        let _ = slow.recv_timeout().await; // Welcome(连接层同步帧)

        // 灌洪水:泵 16 chunk/泵持续推进(虚拟时钟与泵 cadence 同源),
        // World 队列只进不出(不读 socket)。等待覆盖 512+ 包的产出。
        for _ in 0..80 {
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            tokio::task::yield_now().await;
        }

        // 降级证据:World 丢最旧计数增长(512 容量必触顶——4225 请求 > 512)。
        assert!(
            hub.world_dropped() > 0,
            "慢客户端 → World 队列丢最旧(hub 计数增长)"
        );
        // 断连证据:连接最终被服务端收口(不读 socket 的连接不被无限供养;
        // 写超时/写满路径收口,任务 01 实测语义)。
        let _ = slow.is_disconnected().await;
        // tick 不减(暂停时间:80 次 100ms 虚拟等待 = 8000ms 虚拟时长,
        // 驱动 1/60s 间隔应得 ⌊8000/16.67⌋ ≈ 480 拍;断言 ≥ 400 = 拍数
        // 不受洪水拖累——若被阻塞,拍数将远低于虚拟时间应得值)。
        tx.send(DriverCommand::Shutdown).await.expect("驱动仍存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert!(
            outcome.ticks_run >= 400,
            "慢客户端下 tick 计数不减(8000ms 虚拟时长应得 ~480 拍,实 {})",
            outcome.ticks_run
        );
    }

    // ---- 任务 03 测试辅助 ----

    /// 入会(Hello + Subscribe)并吃掉连接层 Welcome;返回身份。
    async fn join_player(c: &mut MiniClient, view_distance: u8) -> messaging::InputSource {
        c.send(&C2S::Hello { client_id: 1 })
            .await
            .expect("Hello 写入");
        c.send(&C2S::Subscribe { view_distance })
            .await
            .expect("Subscribe 写入");
        match c.recv_timeout().await {
            S2C::Welcome { player_id, .. } => player_id,
            other => panic!("首帧应为 Welcome,实为 {other:?}"),
        }
    }

    /// 收下一个 EntityStates 包(跳过其它帧;50ms 软窗 + 320 帧上限——
    /// 静默期不挂死,超限 panic 带意图)。
    async fn recv_entity_states(
        c: &mut MiniClient,
    ) -> (types::Tick, Vec<(EntityId, EntityState)>, Vec<EntityId>) {
        for _ in 0..320 {
            let frame = tokio::time::timeout(std::time::Duration::from_millis(50), c.recv()).await;
            if let Ok(Ok(s2c @ S2C::EntityStates { .. })) = frame {
                let S2C::EntityStates {
                    tick,
                    states,
                    removed,
                    ..
                } = s2c
                else {
                    unreachable!("上层已判定变体")
                };
                return (tick, states, removed);
            }
        }
        panic!("320 帧内未收到 EntityStates(观察输出未达客户端)")
    }

    /// 在最多 `max_frames` 帧内是否见到含该实体的 EntityStates(见即消化);
    /// 帧等待带短超时(50ms:暂停时间下服务端不再产帧即落空返回)——
    /// 等帧用软判定,不因静默窗 panic(负/正样本都可能有静默尾)。
    async fn entity_appears_within(
        c: &mut MiniClient,
        entity: EntityId,
        max_frames: usize,
    ) -> bool {
        for _ in 0..max_frames {
            let frame = tokio::time::timeout(std::time::Duration::from_millis(50), c.recv()).await;
            if let Ok(Ok(S2C::EntityStates { states, .. })) = frame
                && states.iter().any(|(id, _)| *id == entity)
            {
                return true;
            }
        }
        false
    }
}
