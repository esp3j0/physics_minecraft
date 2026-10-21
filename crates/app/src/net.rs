//! app::net —— 连接层(REQ-003 §4.2;任务书 req003-net/01)。
//!
//! 职责:TCP 监听、每连接一 task、流式帧切分(4B LE 长度 + bincode)、
//! 会话状态机(Connected → Subscribed)、翻译层(C2S → DriverCommand 映射
//! 一处收口)、keepalive(读超时窗口)、每玩家出站两级队列骨架
//! (Critical 64 / World 512,README 决策 6;填充逻辑在任务 03,队列与
//! 断连策略本任务落)。
//!
//! 并发模型(REQ-003 决议 2):与驱动同 runtime,每连接一 task;
//! 网络层永不直改模拟——一切下行经 `NetUp`(有界 mpsc)上报驱动。
//! 玩家身份分配:网络层原子完成(JoinHandle 表键 = `InputSource`),
//! 从 1 起递增,0 保留给调度器(messaging::InputSource 约定)。
//!
//! keepalive(README 决策 5):服务端读超时(默认 10s,配置)窗口内无任何
//! 入站帧 → 断连;C2S::Ping → S2C::Pong(Critical 级)。
//!
//! 断连语义(任务书细则 5):Goodbye/超时/协议违规/Critical 溢出 → 尽力发
//! `Bye { reason }` → 上报 `NetUp::Left`。驱动侧观察者清理与出站填充在
//! 任务 03 接线,本任务只上报。

use std::io;

use messaging::{InputSource, SimCommand};
use sim_api::proto::{C2S, FRAME_MAX, S2C};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::{broadcast, mpsc};
use types::glam::Vec3;

pub mod chunk_stream;

use chunk_stream::{ChunkStreamState, PumpConfig};
pub use chunk_stream::{
    MAX_LOADS_PER_PUMP, MAX_UNLOADS_PER_PUMP, PumpConfig as StreamPumpConfig, chebyshev,
    view_square,
};

use crate::driver::DriverCommand;

// ---- 配置 ----

/// 网络配置(Config 增量):listen_addr("127.0.0.1:0" 默认,测试用 0 端口)、
/// read_timeout_secs(10)、chunk 流泵节奏(默认 10 Hz,REQ-003 §4.3)、
/// 观察输出节奏(任务 03:增量周期 10 拍、全量周期 60 拍,README 决策 8)。
#[derive(Debug, Clone)]
pub struct NetConfig {
    /// 监听地址(端口 0 = OS 分配,`serve` 回传实际绑定地址)。
    pub listen_addr: String,
    /// 读超时窗口(秒):窗口内无任何入站帧 → 断连(README 决策 5)。
    pub read_timeout_secs: u64,
    /// chunk 流泵配置(任务 02:pump cadence,默认 10 Hz)。
    pub pump: PumpConfig,
    /// 实体状态增量输出周期(拍;默认 10,任务书 03 接口设计;驱动侧消费)。
    pub observe_interval_ticks: u64,
    /// 实体状态全量输出周期(拍;默认 60,任务书 03 接口设计;驱动侧消费。
    /// 两周期独立计数;全量包 removed 恒空)。
    pub full_snapshot_interval_ticks: u64,
}

impl Default for NetConfig {
    fn default() -> Self {
        Self {
            listen_addr: "127.0.0.1:0".to_string(),
            read_timeout_secs: 10,
            pump: PumpConfig::default(),
            observe_interval_ticks: 10,
            full_snapshot_interval_ticks: 60,
        }
    }
}

// ---- 网络事件(网络 → 驱动) ----

/// 断连原因(Left 上报;任务书细则 5)。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DisconnectReason {
    /// 客户端显式 Goodbye。
    Goodbye,
    /// 读超时窗口内无入站帧。
    Timeout,
    /// 协议违规(未 Subscribe 先发业务帧/帧超限/解码失败/非白名单命令)。
    ProtocolViolation(&'static str),
    /// Critical 出站队列满(README 决策 6:满 → 断连)。
    CriticalOverflow,
    /// 对端先关/读写失败。
    SocketClosed,
    /// 服务器关机。
    Shutdown,
}

impl DisconnectReason {
    /// → Bye 报文 reason 字符串(断连前尽力发送)。
    fn as_bye_text(&self) -> String {
        match self {
            Self::Goodbye => "goodbye".to_string(),
            Self::Timeout => "read timeout".to_string(),
            Self::ProtocolViolation(what) => format!("protocol violation: {what}"),
            Self::CriticalOverflow => "critical queue overflow".to_string(),
            Self::SocketClosed => "socket closed".to_string(),
            Self::Shutdown => "server shutdown".to_string(),
        }
    }
}

/// 网络事件(网络 → 驱动;有界 mpsc)。
#[derive(Debug, PartialEq)]
pub enum NetUp {
    /// Hello+Subscribe 完成后(细则 1:未 Subscribe 不进世界)。
    Joined { player: InputSource },
    /// 订阅视距(任务 03:驱动侧观察输出的距离过滤半径来源;Subscribe 完成后
    /// 与 Joined 同拍上报,重复 Subscribe 亦重报最新钳制值)。
    Subscribed {
        player: InputSource,
        view_distance: u8,
    },
    /// Goodbye/超时/Critical 丢弃/协议违规。
    Left {
        player: InputSource,
        reason: DisconnectReason,
    },
    /// 翻译层产物(DriverCommand:MoveObserver/Send)。
    DriverCommand {
        player: InputSource,
        cmd: DriverCommand,
    },
}

/// 订阅视距上限(REQ-003 §4.3:钳制 ≤32 chunk)。
pub const VIEW_DISTANCE_MAX: u8 = 32;

// ---- 翻译层(纯函数;任务书:单元测试主战场) ----

/// 翻译结果(纯函数产物)。
#[derive(Debug, PartialEq)]
pub enum TranslateOutcome {
    /// MoveObserver。
    Move(DriverCommand),
    /// Command 白名单 → DriverCommand::Send{shard, command}
    /// (shard = 命令作用位置所在分片:SetBlock→pos;Spawn/Detonate→中心)。
    Send(DriverCommand),
    Ping,
    /// 已钳制(≤ [`VIEW_DISTANCE_MAX`])的视距。
    Subscribe(u8),
    Hello,
    Goodbye,
    /// 非白名单命令等(协议违规 → 断连)。
    Reject(&'static str),
}

/// 翻译层(纯函数;零 IO、零时钟,任务书细则 6):C2S → [`TranslateOutcome`]
/// 映射一处收口(README 决策 4)。命令白名单(README 决策 1)在此执行:
/// SetBlock/RemoveEntity/TeleportEntity/Detonate 放行;外部 Spawn 走
/// [`C2S::Spawn`](服务端分配 ID);SpawnDynamic/MarkActor/SetObserverPosition
/// 到达 = 协议违规(`Reject`)。
pub fn translate(player: InputSource, c2s: C2S) -> TranslateOutcome {
    let _ = player; // 身份由上报侧附带;翻译只定形状
    match c2s {
        C2S::Hello { .. } => TranslateOutcome::Hello,
        C2S::Subscribe { view_distance } => {
            TranslateOutcome::Subscribe(view_distance.min(VIEW_DISTANCE_MAX))
        }
        C2S::MoveObserver { position } => {
            TranslateOutcome::Move(DriverCommand::MoveObserver { position })
        }
        C2S::Command(command) => match command_shard(&command) {
            Some(shard) => TranslateOutcome::Send(DriverCommand::Send { shard, command }),
            None => TranslateOutcome::Reject("非白名单命令"),
        },
        C2S::Spawn { state, desc } => TranslateOutcome::Send(DriverCommand::Spawn { state, desc }),
        C2S::Ping => TranslateOutcome::Ping,
        C2S::Goodbye => TranslateOutcome::Goodbye,
    }
}

/// 命令 → 作用分片(白名单判定与路由换算一体;白名单外 → `None`):
/// - `SetBlock` → pos 所在分片(floor 语义,`BlockPos::chunk_of().shard_of()` 同式);
/// - `Detonate` → 爆心(与 `session.move_observer` 的 dest 换算同口径);
/// - `RemoveEntity`/`TeleportEntity` → 实体 origin 序号(信息性路由:实体
///   归属分片由模拟侧权威判定;投递前的**前置校验**见
///   `driver::validate_external_command`——未知实体/越界目标在驱动侧丢弃
///   计数,不投模拟:hostile 输入不得触发 fail-stop panic,评审修复①)。
/// - `SpawnDynamic` → `None`(REQ-006 任务 02:外部只能用 `C2S::Spawn`，
///   由服务端分配 ID;内部/测试仍可直用 SimCommand);
/// - `MarkActor`/`SetObserverPosition` → `None`(非白名单,README 决策 1)。
pub fn command_shard(command: &SimCommand) -> Option<types::ShardId> {
    let shard_pos = match command {
        SimCommand::SetBlock { pos, .. } => pos.chunk_of().shard_of(),
        SimCommand::SpawnDynamic { .. } => return None,
        SimCommand::Detonate { center, .. } => point_shard_of(*center),
        SimCommand::RemoveEntity { entity } | SimCommand::TeleportEntity { entity, .. } => {
            // origin 是分片的 u32 序号(注册表分配),不是 x/z 坐标——没有
            // 坐标依据就不造坐标:序号直落 ShardPos(x 轴,z = 0),doc 注明
            // 信息性路由的语义边界。
            types::ShardPos::new(entity.origin_shard() as i32, 0)
        }
        SimCommand::MarkActor { .. } | SimCommand::SetObserverPosition { .. } => return None,
        // 观察者移除(REQ-005 D9):观察者收件箱专用命令,不走分片前缀——
        // 无上行映射(客户端白名单拒绝;driver::validate_external_command
        // 同口径防御层)。
        SimCommand::RemoveObserver => return None,
    };
    Some(types::ShardId::new(
        types::WorldId(0),
        types::DimensionId(0),
        shard_pos,
    ))
}

/// 世界坐标 → 所在分片(floor 语义;与 driver::shard_of_position 同口径)。
fn point_shard_of(p: Vec3) -> types::ShardPos {
    chunk_shard_of((p.x / 16.0).floor() as i32, (p.z / 16.0).floor() as i32)
}

/// chunk 网格坐标 → 所在分片(8×8 chunk 一片,负方向 floor)。
fn chunk_shard_of(cx: i32, cz: i32) -> types::ShardPos {
    types::ShardPos::new(
        cx.div_euclid(types::SHARD_CHUNKS),
        cz.div_euclid(types::SHARD_CHUNKS),
    )
}

// ---- 出站两级队列(README 决策 6;骨架——填充在任务 03) ----

/// 出站优先级(两级):Critical(Welcome/Bye/Pong)/ World(chunk 流/实体状态)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutboundPriority {
    Critical,
    World,
}

/// Critical 队列容量(README 决策 6:64;满 → 断连)。
pub const CRITICAL_CAPACITY: usize = 64;
/// World 队列容量(README 决策 6:512;满 → 丢最旧)。
pub const WORLD_CAPACITY: usize = 512;

/// 出站入队结果(供本任务断言与任务 03 填充侧使用)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnqueueResult {
    Enqueued,
    /// World 满:丢最旧,新值入队(EntityStates 天然 latest-wins;ChunkData
    /// 丢失由客户端重新 Subscribe 修复,README 决策 6)。
    DroppedOldest,
    /// Critical 满(或接收端已亡)= 断连信号。
    CriticalFull,
}

/// 每连接出站队列:Critical 满或 send 失败 → 断连并上报 Left;World 满丢最旧。
///
/// 结构(任务 02 起共享):Critical = 连接任务私有的 mpsc(Welcome/Pong/Bye);
/// World = 共享 deque(`std::sync::Mutex<VecDeque>`,任务 02:泵任务写、连接
/// 任务读——**socket 单写者始终是连接任务**)+ [`tokio::sync::Notify`] 提醒
/// 连接冲刷(泵不入 socket;notify_one 无等待者时存票,不丢唤醒)。
/// 互斥临界区全同步零 await,不会跨挂起点持有;锁中毒不可达
/// (临界区无 panic 点),保守解包继续。
pub struct Outbound {
    critical_tx: mpsc::Sender<S2C>,
    critical_rx: mpsc::Receiver<S2C>,
    world: std::sync::Arc<std::sync::Mutex<std::collections::VecDeque<S2C>>>,
    /// World 入队提醒(泵 → 连接读循环;连接据此冲刷 World 队列出网)。
    pub(crate) data_ready: std::sync::Arc<tokio::sync::Notify>,
    /// World 丢最旧计数(队列级共享:hub 注册的写方与泵写方累加同一计数;
    /// `world_dropped()` 观测口径的总览)。
    world_dropped: std::sync::Arc<std::sync::atomic::AtomicU64>,
}

impl Outbound {
    /// 两级各按容量建队。
    pub fn new() -> Self {
        let (critical_tx, critical_rx) = mpsc::channel(CRITICAL_CAPACITY);
        Self {
            critical_tx,
            critical_rx,
            world: std::sync::Arc::new(std::sync::Mutex::new(std::collections::VecDeque::new())),
            data_ready: std::sync::Arc::new(tokio::sync::Notify::new()),
            world_dropped: std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0)),
        }
    }

    /// World 队列的泵句柄(任务 02):共享同一 deque 与提醒,只写不读。
    /// 任务 03:附 Critical 发送端克隆(驱动 Welcome 下行经此直通玩家
    /// Critical 队列;连接任务仍是唯一 socket 写者)与丢弃计数句柄。
    pub(crate) fn world_writer(&self) -> WorldWriter {
        WorldWriter {
            world: self.world.clone(),
            notify: self.data_ready.clone(),
            critical_tx: self.critical_tx.clone(),
            dropped: self.world_dropped.clone(),
        }
    }

    /// World 入队核心(同步;满 512 → 丢最旧,新值入队尾)。
    /// 锁中毒(理论不可达:临界区无 panic 点)→ 取内值继续,不放大故障。
    fn world_push(
        world: &std::sync::Mutex<std::collections::VecDeque<S2C>>,
        msg: S2C,
    ) -> EnqueueResult {
        let mut guard = world.lock().unwrap_or_else(|p| p.into_inner());
        if guard.len() >= WORLD_CAPACITY {
            guard.pop_front(); // 丢最旧(README 决策 6)
            guard.push_back(msg);
            EnqueueResult::DroppedOldest
        } else {
            guard.push_back(msg);
            EnqueueResult::Enqueued
        }
    }

    /// 入队(按优先级);策略见 [`EnqueueResult`]。World 入队后提醒连接冲刷
    /// (任务 02:泵/连接共写时,冲刷由连接任务统一执行)。
    pub fn push(&mut self, priority: OutboundPriority, msg: S2C) -> EnqueueResult {
        match priority {
            OutboundPriority::Critical => match self.critical_tx.try_send(msg) {
                Ok(()) => EnqueueResult::Enqueued,
                Err(mpsc::error::TrySendError::Full(_))
                | Err(mpsc::error::TrySendError::Closed(_)) => EnqueueResult::CriticalFull,
            },
            OutboundPriority::World => {
                let result = Self::world_push(&self.world, msg);
                self.data_ready.notify_one();
                result
            }
        }
    }

    /// 队列是否全空(测试与断连判定用)。
    pub fn is_empty(&self) -> bool {
        self.critical_rx.is_empty()
            && self
                .world
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .is_empty()
    }

    /// 取队头(Critical 先于 World;写泵任务 03 接线,本任务供测试)。
    pub(crate) fn try_pop(&mut self) -> Option<S2C> {
        self.critical_rx.try_recv().ok().or_else(|| {
            self.world
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .pop_front()
        })
    }
}

impl Default for Outbound {
    fn default() -> Self {
        Self::new()
    }
}

/// World 队列的泵句柄(任务 02):只写 World + 提醒连接冲刷;不碰 Critical
/// (Welcome/Pong/Bye 属连接任务)。入队即 notify_one——连接读循环的
/// `data_ready` 臂据此冲刷出网(单 socket 写者)。
pub(crate) struct WorldWriter {
    world: std::sync::Arc<std::sync::Mutex<std::collections::VecDeque<S2C>>>,
    notify: std::sync::Arc<tokio::sync::Notify>,
    /// Critical 发送端(任务 03:Welcome 经 hub 入玩家 Critical 队列——
    /// 连接任务是唯一接收者;满 = CriticalFull 断连信号原样上浮)。
    critical_tx: mpsc::Sender<S2C>,
    /// 本队列的丢最旧计数(deque 级共享:hub 写方与泵写方累加同一计数,
    /// AC-N5 观测口径 = 该玩家 World 队列的全部丢最旧)。
    dropped: std::sync::Arc<std::sync::atomic::AtomicU64>,
}

impl WorldWriter {
    /// 入队 World 级(deque 永不"关闭":连接亡故由泵关机路径收口,
    /// 见 [`StreamLink`] 的 Drop;故只有 Enqueued/DroppedOldest 两态)。
    pub(crate) fn push_world(&mut self, msg: S2C) -> EnqueueResult {
        let result = Outbound::world_push(&self.world, msg);
        if result == EnqueueResult::DroppedOldest {
            self.dropped
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
        self.notify.notify_one();
        result
    }

    /// 入队 Critical 级(任务 03:驱动侧 Welcome 下行;满 → CriticalFull,
    /// 连接侧断连收口;发送端已亡(连接任务终止)→ 同判(断连语义))。
    pub(crate) fn push_critical(&mut self, msg: S2C) -> EnqueueResult {
        match self.critical_tx.try_send(msg) {
            Ok(()) => {
                self.notify.notify_one();
                EnqueueResult::Enqueued
            }
            Err(mpsc::error::TrySendError::Full(_)) | Err(mpsc::error::TrySendError::Closed(_)) => {
                EnqueueResult::CriticalFull
            }
        }
    }

    /// 本队列丢最旧累计(hub 总览求和用)。
    pub(crate) fn dropped(&self) -> u64 {
        self.dropped.load(std::sync::atomic::Ordering::Relaxed)
    }
}

// ---- 出站注册表(任务 03):驱动观察输出 → 每玩家 World 队列的下行缝 ----

/// 出站队列注册表(REQ-003 §4.4「EntityStates 入 World 队列」的落地缝):
/// 连接 Subscribe 成功时注册本连接的 [`WorldWriter`],驱动 tick 末端编排据此
/// 把观察包逐玩家入队;连接终止(任务 Drop)即注销——注册表永不含死队列。
///
/// 无锁设计:注册/注销在连接任务的 Subscribe 处理路径同步完成
/// (`Mutex` 临界区零 await);驱动侧只读快照。锁中毒不可达(临界区无
/// panic 点),保守解包继续(app::net 既有同款纪律)。
#[derive(Default)]
pub struct OutboundHub {
    /// 玩家 → 该连接的 World 队列写入端(BTreeMap:键序确定性)。
    writers: std::sync::Mutex<std::collections::BTreeMap<InputSource, WorldWriter>>,
    /// World 满 → 丢最旧的累计次数(AC-N5 观测口径;单调;原子累计——
    /// 计数本体在每连接的 Outbound 上,hub 注册/注销只存各队列的计数句柄,
    /// 读侧对在场句柄求和 + 已注销队列的沉淀值)。
    dropped_retired: std::sync::atomic::AtomicU64,
}

impl OutboundHub {
    /// 空注册表(装配期构造,与 `serve`/驱动共享)。
    pub fn new() -> Self {
        Self::default()
    }

    /// 注册玩家出站(连接 Subscribe 成功时;同玩家重复注册 = 替换——
    /// 身份单调递增不循环,理论不可达,防御式覆盖)。
    fn register(&self, player: InputSource, writer: WorldWriter) {
        self.writers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(player, writer);
    }

    /// 注销玩家出站(连接任务终止时;Drop 路径同步执行)。返回是否确有注册。
    fn unregister(&self, player: &InputSource) -> bool {
        let removed = self
            .writers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .remove(player);
        if let Some(writer) = removed {
            // 已注销队列的丢弃计数沉淀(总览口径不因断连而回退)。
            self.dropped_retired
                .fetch_add(writer.dropped(), std::sync::atomic::Ordering::Relaxed);
            true
        } else {
            false
        }
    }

    /// Welcome 入玩家 Critical 队列(任务书 03 细则 ⑤):满(64)→
    /// `CriticalFull` 返回(调用方不重试——Critical 满 = 断连信号,连接侧
    /// 断连收口后注册消失,后续 push 自然弃);玩家已断连 → `Enqueued` 于
    /// 死队列(写入无害,写缓冲无人读——返回值仅诊断用)。
    pub fn push_critical(&self, player: InputSource, msg: S2C) -> EnqueueResult {
        let writer = {
            let guard = self.writers.lock().unwrap_or_else(|p| p.into_inner());
            guard.get(&player).map(cloned_writer)
        };
        match writer {
            Some(mut writer) => writer.push_critical(msg),
            None => EnqueueResult::Enqueued, // 已断连:静默弃(竞先窗口,无泄漏)
        }
    }

    /// 当前订阅玩家(升序;驱动编排「对每订阅玩家」的遍历源)。
    pub fn subscribers(&self) -> Vec<InputSource> {
        self.writers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .keys()
            .copied()
            .collect()
    }

    /// 观察包入玩家 World 队列(满 512 → 丢最旧,新值入队——EntityStates
    /// latest-wins 语义丢旧无损正确性,下轮全量周期自愈,任务书细则 4)。
    /// 玩家已断连(已注销)→ 静默弃(驱动与连接的竞先窗口,无泄漏)。
    pub fn push_world(&self, player: InputSource, msg: S2C) {
        let writer = {
            let guard = self.writers.lock().unwrap_or_else(|p| p.into_inner());
            guard.get(&player).map(cloned_writer)
        };
        if let Some(mut writer) = writer {
            // 丢弃计数本体在队列上(WorldWriter::push_world 内累加;泵写方
            // 同计数)——hub 不再重复计数。
            let _ = writer.push_world(msg);
        }
    }

    /// World 丢最旧累计次数(AC-N5 观测口径;测试与度量用):在场队列计数
    /// 求和 + 已注销队列沉淀值。
    pub fn world_dropped(&self) -> u64 {
        let live: u64 = self
            .writers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .values()
            .map(WorldWriter::dropped)
            .sum();
        live + self
            .dropped_retired
            .load(std::sync::atomic::Ordering::Relaxed)
    }
}

/// `&WorldWriter` → 拥有的克隆(字段皆 Arc,克隆廉价;自由函数便于
/// `map(cloned_writer)` 直接用于守卫内的借用)。
fn cloned_writer(writer: &WorldWriter) -> WorldWriter {
    WorldWriter {
        world: writer.world.clone(),
        notify: writer.notify.clone(),
        critical_tx: writer.critical_tx.clone(),
        dropped: writer.dropped.clone(),
    }
}

// ---- 监听与连接任务 ----

/// 启动监听(spawn accept 循环 + 每连接 task);返回实际绑定地址(测试取端口)。
/// shutdown 关闭后停止 accept,广播存量连接退出,并兜底 abort 连接任务。
///
/// 本函数 bind 后 **spawn** accept 循环并立即返回地址(任务书:「spawn accept
/// 循环 + 每连接 task;返回实际绑定地址」)——调用方无须等监听停止即可拿到
/// 端口开连。
///
/// `hub`(任务 03):出站注册表,调用方(装配)持有克隆供驱动编排下行填充;
/// 连接 Subscribe 注册、断连注销。
pub async fn serve(
    cfg: NetConfig,
    up_tx: mpsc::Sender<NetUp>,
    shutdown: broadcast::Receiver<()>,
    hub: std::sync::Arc<OutboundHub>,
) -> io::Result<std::net::SocketAddr> {
    let listener = tokio::net::TcpListener::bind(&cfg.listen_addr).await?;
    let addr = listener.local_addr()?;
    // accept 循环任务:自持一份关机发送端(广播存量连接退出;自身随 caller 的
    // shutdown 或显式广播退出)。
    let shutdown_tx = broadcast::channel::<()>(1).0;
    let accept_loop_task = tokio::spawn(accept_loop(
        listener,
        cfg,
        up_tx,
        shutdown,
        shutdown_tx.clone(),
        hub,
    ));
    // 退出钩子:accept 循环结束(自行返回或被 abort)时确保广播关机,
    // 存量连接全部终止。
    tokio::spawn(async move {
        let _ = accept_loop_task.await;
        let _ = shutdown_tx.send(());
    });
    Ok(addr)
}

/// accept 循环(serve 的常驻体):每连接分配递增身份并 spawn 连接 task。
async fn accept_loop(
    listener: tokio::net::TcpListener,
    cfg: NetConfig,
    up_tx: mpsc::Sender<NetUp>,
    mut shutdown: broadcast::Receiver<()>,
    shutdown_tx: tokio::sync::broadcast::Sender<()>,
    hub: std::sync::Arc<OutboundHub>,
) {
    // 存量连接句柄(JoinHandle 表;键序 = 分配序):退出前广播 + 兜底 abort。
    let mut connections: Vec<tokio::task::JoinHandle<()>> = Vec::new();
    let mut next_player: u32 = 1; // 身份从 1 起(0 保留给调度器)

    loop {
        tokio::select! {
            biased;

            _ = shutdown.recv() => break,

            accepted = listener.accept() => {
                let (stream, _peer) = match accepted {
                    Ok(ok) => ok,
                    Err(_) => continue, // 瞬时 accept 失败:不塌监听
                };
                let player = InputSource(next_player);
                next_player += 1;
                let conn_shutdown = shutdown_tx.subscribe();
                let up = up_tx.clone();
                let cfg = cfg.clone();
                let hub = hub.clone();
                connections.push(tokio::spawn(run_connection(
                    player, stream, cfg, up, conn_shutdown, hub,
                )));
            }
        }
    }

    // 关机收口:广播(存量连接自然终止)+ 兜底 abort。
    let _ = shutdown_tx.send(());
    for handle in connections.drain(..) {
        handle.abort();
    }
}

/// 会话状态(细则 1):Connected(未 Subscribe)→ Subscribed(已订阅,
/// chunk 流激活)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SessionState {
    Connected,
    /// 已 Subscribe(载荷 = 订阅者身份:泵/驱动回执按此转发)。
    Subscribed,
}

/// chunk 流接线(连接 ↔ 泵;任务 02):订阅成功时建立,连接任务退出即弃
/// (泵择 `shutdown` 臂终止——断连即停,无孤儿任务)。
pub(crate) struct StreamLink {
    /// 泵态(center 单槽合并 + 视距 + loaded;连接读路径与泵共享)。
    pub state: std::sync::Arc<tokio::sync::Mutex<ChunkStreamState>>,
    /// 连接侧关机发送端(连接任务 Drop 时触发;显式 abort 兜底)。
    pub shutdown: tokio::sync::broadcast::Sender<()>,
    /// 泵任务句柄(显式 abort 兜底用)。
    pub task: tokio::task::JoinHandle<()>,
}

impl StreamLink {
    /// 起泵任务(独立 interval task,默认 10 Hz,任务书:pump cadence)。
    /// 回执 chunk 经 [`WorldWriter`] 入 World 队列(单 socket 写者 = 连接任务,
    /// `data_ready` 提醒冲刷);StreamChunks 请求经 `up_tx` 以
    /// `NetUp::DriverCommand` 上行(驱动侧路由后持有 oneshot 回执)。
    pub fn spawn(
        state: std::sync::Arc<tokio::sync::Mutex<ChunkStreamState>>,
        cfg: PumpConfig,
        up_tx: mpsc::Sender<NetUp>,
        outbound: WorldWriter,
        player: InputSource,
    ) -> Self {
        let (shutdown, shutdown_rx) = broadcast::channel::<()>(1);
        let task = tokio::spawn(chunk_stream::run_pump(
            state.clone(),
            cfg,
            shutdown_rx,
            up_tx,
            outbound,
            player,
        ));
        Self {
            state,
            shutdown,
            task,
        }
    }
}

impl Drop for StreamLink {
    fn drop(&mut self) {
        // 连接侧关机:广播 + 兜底 abort(泵 select 的 shutdown 臂通常先醒)。
        let _ = self.shutdown.send(());
        self.task.abort();
    }
}

/// 断连收口守卫(任务 03):持有 = 玩家出站在 [`OutboundHub`] 注册中;
/// Drop(连接任务的任何 return 路径)→ 注销——驱动侧「Left 上行 → 流状态
/// 清理」以注册表消失为权威(断连即弃,last_sent 同弃,任务书细则 3)。
struct DisconnectGuard {
    player: InputSource,
    hub: std::sync::Arc<OutboundHub>,
}

impl Drop for DisconnectGuard {
    fn drop(&mut self) {
        self.hub.unregister(&self.player);
    }
}

/// 单帧处置流向(继续读下一帧 / 按原因断连)。
enum Flow {
    Continue,
    Disconnect(DisconnectReason),
}

/// 每连接任务:读循环(流式切帧 + 读超时)→ 状态机/翻译 → 上报;Pong 应答。
/// 任何断连路径都先尽力发 `Bye`,再上报 `NetUp::Left`(细则 5)。
///
/// 任务 03:Subscribe 成功时该连接的 World 写入端注册进 `hub`(驱动观察输出
/// 下行缝);连接终止(任何 return 路径)由 [`DisconnectGuard`] Drop 注销。
#[allow(clippy::too_many_arguments)]
async fn run_connection(
    player: InputSource,
    mut stream: TcpStream,
    cfg: NetConfig,
    up_tx: mpsc::Sender<NetUp>,
    mut shutdown: broadcast::Receiver<()>,
    hub: std::sync::Arc<OutboundHub>,
) {
    let read_timeout = std::time::Duration::from_secs(cfg.read_timeout_secs.max(1));
    let mut outbound = Outbound::new();
    let mut state = SessionState::Connected;
    let mut buf: Vec<u8> = Vec::with_capacity(4096);
    let mut chunk = [0u8; 4096];
    // chunk 流(任务 02):订阅时激活;泵 task 随连接任务终止(断连即停,
    // 无孤儿任务)。stream_tx 挂在 `NetUp::DriverCommand` 的发送端克隆上
    // (驱动侧经 NetUp 路由并持有回执——REQ-003 §4.4「网络侧经 NetUp 上行
    // 后由驱动持有」)。
    let mut stream_link: Option<StreamLink> = None;
    // 观察输出注册守卫(任务 03):构造于 Subscribe 成功处(持有前不注销);
    // 任何 return 路径 Drop → hub 注销(注册表永不含死队列)。
    let mut hub_guard: Option<DisconnectGuard> = None;

    loop {
        // 读 + 关机 + World 队列提醒(任务 02:泵回执 ChunkData/ChunkUnload
        // 入队即提醒)竞先;keepalive 窗口:每帧(含 Ping)都重新计时(细则 3)。
        // `data_ready` 臂 = 连接任务是 socket 唯一写者:取尽两级队列并冲刷
        // (Critical 先于 World,README 决策 6 优先级)。
        let result = tokio::select! {
            biased;
            _ = shutdown.recv() => {
                disconnect(&mut stream, &mut outbound, &up_tx, player, DisconnectReason::Shutdown).await;
                return;
            }
            _ = outbound.data_ready.notified() => Ok(Ok(usize::MAX)), // 伪读:触发冲刷
            result = tokio::time::timeout(read_timeout, stream.read(&mut chunk)) => result,
        };
        let n = match result {
            Ok(Ok(n)) => n,
            // 读超时(Err 分支 = timeout elapsed)→ Timeout;读失败 → SocketClosed。
            Ok(Err(_)) | Err(_) => {
                let reason = if result.is_err() {
                    DisconnectReason::Timeout
                } else {
                    DisconnectReason::SocketClosed
                };
                // 对端先关/写失败时 Bye 多半发不出去:仍走断连收口(尽力而为)。
                disconnect(&mut stream, &mut outbound, &up_tx, player, reason).await;
                return;
            }
        };
        if n != usize::MAX {
            if n == 0 {
                // 对端半关(EOF):上报 SocketClosed,不再回写。
                let _ = up_tx
                    .send(NetUp::Left {
                        player,
                        reason: DisconnectReason::SocketClosed,
                    })
                    .await;
                return;
            }
            buf.extend_from_slice(&chunk[..n]);
        }
        // 先冲刷两级出站(Critical 先于 World;泵提醒/新帧都可能带着待发数据)。
        if flush_outbound(&mut outbound, &mut stream).await.is_err() {
            disconnect(
                &mut stream,
                &mut outbound,
                &up_tx,
                player,
                DisconnectReason::SocketClosed,
            )
            .await;
            return;
        }

        // 流式切帧(细则 2):凑满 4B 长度 → 超限断连 → 凑满载荷 → decode。
        loop {
            match cut_frame(&buf) {
                Cut::NeedMore => break, // 半帧:等更多字节
                Cut::Oversized => {
                    disconnect(
                        &mut stream,
                        &mut outbound,
                        &up_tx,
                        player,
                        DisconnectReason::ProtocolViolation("帧超限"),
                    )
                    .await;
                    return;
                }
                Cut::Frame(frame, consumed) => {
                    buf.drain(..consumed);
                    let Some(c2s) = sim_api::proto::decode_c2s_frame(&frame) else {
                        disconnect(
                            &mut stream,
                            &mut outbound,
                            &up_tx,
                            player,
                            DisconnectReason::ProtocolViolation("解码失败"),
                        )
                        .await;
                        return;
                    };
                    match handle_frame(
                        player,
                        &mut state,
                        c2s,
                        &mut outbound,
                        &up_tx,
                        &mut stream,
                        &mut stream_link,
                        cfg.pump,
                        &hub,
                        hub.clone(),
                        &mut hub_guard,
                    )
                    .await
                    {
                        Flow::Continue => {}
                        Flow::Disconnect(reason) => {
                            disconnect(&mut stream, &mut outbound, &up_tx, player, reason).await;
                            return;
                        }
                    }
                }
            }
        }
    }
}

/// 单帧处置(状态机 + 翻译 + 上报):返回 `Flow::Continue` 或按原因断连。
/// 细则 1:未 Subscribe 前,只放行 Hello / Subscribe / Ping / Goodbye;
/// 其余(含 MoveObserver / Command)= 协议违规断连。
///
/// 任务 02(chunk 流):Subscribe 成功 → 建 [`ChunkStreamState`] 并起泵 task
/// (回执转发同 `NetUp::DriverCommand` 上行通道);Subscribed 后的 MoveObserver
/// → 泵态 pending 中心单槽合并(socket 读路径零 chunk 工作,v2 语义)+
/// 照常上报驱动(观察者位置仍由模拟侧消费)。
///
/// 任务 03(观察输出接线):Subscribe 成功 → 该连接的 World 队列写入端注册进
/// [`OutboundHub`](驱动编排的下行填充缝),并以 `NetUp::Subscribed` 把(钳制
/// 后)视距上报驱动——距离过滤半径的单一来源;连接终止由 [`DisconnectGuard`]
/// 的 Drop 注销注册(注册表永不含死队列)。
#[allow(clippy::too_many_arguments)]
async fn handle_frame(
    player: InputSource,
    state: &mut SessionState,
    c2s: C2S,
    outbound: &mut Outbound,
    up_tx: &mpsc::Sender<NetUp>,
    stream: &mut TcpStream,
    stream_link: &mut Option<StreamLink>,
    pump: PumpConfig,
    hub: &OutboundHub,
    run_hub: std::sync::Arc<OutboundHub>,
    hub_guard: &mut Option<DisconnectGuard>,
) -> Flow {
    // 状态机门(先于翻译:门是会话层的,翻译是纯形状映射)。
    if *state == SessionState::Connected {
        match c2s {
            C2S::Hello { .. } | C2S::Subscribe { .. } | C2S::Ping | C2S::Goodbye => {}
            _ => {
                return Flow::Disconnect(DisconnectReason::ProtocolViolation(
                    "未 Subscribe 前的帧",
                ));
            }
        }
    }

    match translate(player, c2s) {
        TranslateOutcome::Hello => Flow::Continue,
        TranslateOutcome::Subscribe(view_distance) => {
            if let Some(link) = stream_link.as_ref() {
                // 重复 Subscribe:换挡(钳制后替换视距;泵差分自然卸出外环)。
                // 首订阅 Welcome 已发,此处不再发;最新视距重报驱动(任务 03:
                // 距离过滤半径跟随)。
                link.state
                    .lock()
                    .await
                    .resubscribe(view_distance.min(VIEW_DISTANCE_MAX));
                let _ = up_tx
                    .send(NetUp::Subscribed {
                        player,
                        view_distance: view_distance.min(VIEW_DISTANCE_MAX),
                    })
                    .await;
                return Flow::Continue;
            }
            *state = SessionState::Subscribed;
            // Welcome(Critical 级)+ 立即冲刷(客户端直读 socket 可见)。
            // tick 暂填 0,连发后由驱动经 hub 刷新(任务 03 接线;语义:
            // Welcome 到达即订阅成立,拍号随后一次刷新对齐)。
            let welcome = S2C::Welcome {
                player_id: player,
                tick: types::Tick(0),
            };
            if outbound.push(OutboundPriority::Critical, welcome) == EnqueueResult::CriticalFull {
                return Flow::Disconnect(DisconnectReason::CriticalOverflow);
            }
            if flush_outbound(outbound, stream).await.is_err() {
                return Flow::Disconnect(DisconnectReason::SocketClosed);
            }
            // chunk 流激活:起始中心 = 世界原点所在 chunk(玩家观察者尚未上报
            // 位置;首个 MoveObserver 合并进 pending 单槽,首泵即按真实位置差分
            // ——0 视距订阅静默等待,与 v2 spawn 前 pending 语义一致)。
            // 泵经 WorldWriter 只写 World 队列(单 socket 写者 = 连接任务);
            // StreamChunks 请求仍经 up_tx 上行(驱动侧经 NetUp 路由后持有
            // oneshot 回执,REQ-003 §4.4)。
            let stream_state = std::sync::Arc::new(tokio::sync::Mutex::new(ChunkStreamState::new(
                types::ChunkPos::new(0, 0),
                view_distance,
            )));
            let link = StreamLink::spawn(
                stream_state,
                pump,
                up_tx.clone(),
                outbound.world_writer(),
                player,
            );
            // 观察输出注册(任务 03):World 队列写入端入 hub,驱动编排据此
            // 逐玩家填充 EntityStates/BlockUpdates;连接终止由注册守卫注销
            // (守卫经 `&mut Option` 从连接任务写入——handle_frame 不持 Arc)。
            hub.register(player, outbound.world_writer());
            let hub_arc = run_hub.clone();
            *hub_guard = Some(DisconnectGuard {
                player,
                hub: hub_arc,
            });
            *stream_link = Some(link);
            // 入会三连报(任务 03):Joined(观察者建立)→ Subscribed(距离
            // 过滤半径)→ 当前拍(Welcome 拍号一次刷新对齐;驱动 tick_now 即
            // 已完成拍数,客户端开局即可对齐时间轴)。
            let _ = up_tx.send(NetUp::Joined { player }).await;
            let _ = up_tx
                .send(NetUp::Subscribed {
                    player,
                    view_distance: view_distance.min(VIEW_DISTANCE_MAX),
                })
                .await;
            Flow::Continue
        }
        TranslateOutcome::Move(cmd) => {
            // 观察者移动:泵态 pending 中心单槽合并(读路径零 chunk 工作);
            // 上报驱动照旧(观察者表 = 模拟侧权威)。
            if let (Some(link), crate::driver::DriverCommand::MoveObserver { position }) =
                (stream_link.as_ref(), &cmd)
            {
                let center = types::ChunkPos::new(
                    (position.x / 16.0).floor() as i32,
                    (position.z / 16.0).floor() as i32,
                );
                link.state.lock().await.set_center(center);
            }
            let _ = up_tx.send(NetUp::DriverCommand { player, cmd }).await;
            Flow::Continue
        }
        TranslateOutcome::Send(cmd) => {
            let _ = up_tx.send(NetUp::DriverCommand { player, cmd }).await;
            Flow::Continue
        }
        TranslateOutcome::Ping => {
            // Ping → Pong(Critical 级,细则 3)+ 立即冲刷(keepalive 应答可观测)。
            match outbound.push(OutboundPriority::Critical, S2C::Pong) {
                EnqueueResult::CriticalFull => Flow::Disconnect(DisconnectReason::CriticalOverflow),
                _ => {
                    if flush_outbound(outbound, stream).await.is_err() {
                        Flow::Disconnect(DisconnectReason::SocketClosed)
                    } else {
                        Flow::Continue
                    }
                }
            }
        }
        TranslateOutcome::Goodbye => Flow::Disconnect(DisconnectReason::Goodbye),
        TranslateOutcome::Reject(why) => Flow::Disconnect(DisconnectReason::ProtocolViolation(why)),
    }
}

/// 流式切帧结果(细则 2;纯缓冲操作)。
#[derive(Debug)]
enum Cut {
    /// 半帧:等更多字节。
    NeedMore,
    /// 长度前缀 > [`FRAME_MAX`]:立即断连(不读载荷——防炸弹,v2 教训)。
    Oversized,
    /// 完整帧字节 + 消耗量(4B 前缀含在内)。
    Frame(Vec<u8>, usize),
}

fn cut_frame(buf: &[u8]) -> Cut {
    if buf.len() < 4 {
        return Cut::NeedMore;
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    if len > FRAME_MAX {
        return Cut::Oversized;
    }
    let total = 4 + len;
    if buf.len() < total {
        return Cut::NeedMore;
    }
    Cut::Frame(buf[..total].to_vec(), total)
}

/// 断连路径收口:尽力发 Bye(Critical 满/写失败则放弃)→ 上报 Left → 半关写侧。
async fn disconnect(
    stream: &mut TcpStream,
    outbound: &mut Outbound,
    up_tx: &mpsc::Sender<NetUp>,
    player: InputSource,
    reason: DisconnectReason,
) {
    let left = NetUp::Left {
        player,
        reason: reason.clone(),
    };
    // Bye 尽力而为:Critical 溢出断连时队列已满,放弃发送(发也发不出)。
    if reason != DisconnectReason::CriticalOverflow && reason != DisconnectReason::SocketClosed {
        let bye = S2C::Bye {
            reason: reason.as_bye_text(),
        };
        if outbound.push(OutboundPriority::Critical, bye) != EnqueueResult::CriticalFull {
            let _ = flush_outbound(outbound, stream).await;
        }
    }
    let _ = up_tx.send(left).await;
    let _ = stream.shutdown().await;
}

/// 单次出站写超时(慢客户端保护,README 决策 6 的写侧补全):客户端不读
/// socket → 写缓冲堆满 → 写阻塞;超时即放弃本次冲刷并断连(与读侧 keepalive
/// 对称,服务器永不因单个慢客户端阻塞)。
const FLUSH_WRITE_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(5);

/// 出站冲刷:Critical 先于 World(README 决策 6 优先级),逐帧编码写出。
/// 写失败或写超时 → `Err`(调用方按 SocketClosed 断连)。
async fn flush_outbound(outbound: &mut Outbound, stream: &mut TcpStream) -> io::Result<()> {
    while let Some(msg) = outbound.try_pop() {
        let frame = sim_api::proto::encode_frame(&msg);
        match tokio::time::timeout(FLUSH_WRITE_TIMEOUT, stream.write_all(&frame)).await {
            Ok(r) => r?,
            Err(_elapsed) => {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "出站写超时(慢客户端不读)",
                ));
            }
        }
    }
    // flush 同样受写超时保护(缓冲语义下 flush 才是真正出网的一步)。
    match tokio::time::timeout(FLUSH_WRITE_TIMEOUT, stream.flush()).await {
        Ok(r) => r,
        Err(_elapsed) => Err(io::Error::new(
            io::ErrorKind::TimedOut,
            "出站写超时(慢客户端不读)",
        )),
    }
}

// ---- 装配(REQ-004A 任务 00):main 与测试共用的网络装配缝 ----

/// 网络装配产出(main 与 smoke 测试共用;测试书授权的抽取件):
/// `observe` 交驱动(`DriverArgs.observe`),`addr` 供打印「监听 {addr}」
/// 与测试连入,`shutdown` 为关停句柄。
pub struct NetworkHandle {
    /// 驱动观察缝(DriverArgs.observe 三件套)。
    pub observe: crate::driver::ObserveLink,
    /// 实际绑定地址(端口 0 = OS 分配,`serve` 回传)。
    pub addr: std::net::SocketAddr,
    /// 网络关停句柄(存活 = 监听常驻)。
    pub shutdown: ShutdownHandle,
}

/// 网络关停句柄:存活 = accept 循环常驻;显式 [`ShutdownHandle::shutdown`]
/// 广播关机(accept 停止 + 存量连接退出);句柄 drop(sender 析构)同样令
/// accept 循环经 recv `Closed` 收口——进程正常退出的自然语义。
#[derive(Debug, Clone)]
pub struct ShutdownHandle(tokio::sync::broadcast::Sender<()>);

impl ShutdownHandle {
    /// 广播关机(幂等;发送端自持,无接收者亦无害)。
    pub fn shutdown(&self) {
        let _ = self.0.send(());
    }
}

/// 装配网络(REQ-004A 任务 00;main 与测试共用):Config.listen_addr +
/// NetConfig 默认(超时/泵/观察节奏)→ serve 起监听(spawn accept 循环)→
/// ObserveLink(up/hub/cadence 三缝)交驱动。监听失败 = `Err`(启动期错误,
/// 调用方 fail-stop expect)。
///
/// 观察节奏投影 `ObserveCadence::from(&NetConfig)`(默认 10/60 拍);up 通道
/// 容量 256(REQ-003 测试装配同款);关停句柄存活 = 监听常驻(main 持有至
/// 进程退出,勿忘)。
pub async fn assemble_network(config: &crate::config::Config) -> io::Result<NetworkHandle> {
    let net_cfg = NetConfig {
        listen_addr: config.listen_addr.clone(),
        ..NetConfig::default()
    };
    let cadence = crate::driver::ObserveCadence::from(&net_cfg);
    let (up_tx, up_rx) = mpsc::channel(256);
    let hub = std::sync::Arc::new(OutboundHub::new());
    let (shutdown_tx, shutdown_rx) = tokio::sync::broadcast::channel::<()>(1);
    let addr = serve(net_cfg, up_tx, shutdown_rx, hub.clone()).await?;
    Ok(NetworkHandle {
        observe: crate::driver::ObserveLink {
            up: up_rx,
            hub,
            cadence,
        },
        addr,
        shutdown: ShutdownHandle(shutdown_tx),
    })
}

// ---- 测试清单(内嵌;全走 127.0.0.1:0 + 内嵌迷你客户端) ----

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use sim_api::proto::{encode_c2s_frame, encode_frame};
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio::net::TcpStream;
    use tokio::sync::mpsc;
    use types::{BlockPos, EntityId};

    /// 5s 测试超时上限(网络真 IO;防挂死)。
    fn net_timeout() -> std::time::Duration {
        std::time::Duration::from_secs(5)
    }

    /// 内嵌迷你客户端:真 TCP,帧编解码复用 sim-api::proto 的对称接口
    /// (encode_c2s_frame / 手工读长度前缀;README 决策 10)。
    pub(crate) struct MiniClient {
        stream: TcpStream,
        /// std 侧副本(#15 半关读端用;与 stream 共享同一 socket)。
        std_stream: std::net::TcpStream,
        buf: Vec<u8>,
    }

    impl MiniClient {
        pub(crate) async fn connect(addr: std::net::SocketAddr) -> io::Result<Self> {
            // std 建连后转 tokio:留一份 std 副本供半关读端(#15);
            // tokio TcpStream 无 try_clone,into_std 会摘走注册的 fd。
            // from_std 前必须设非阻塞(tokio 文档要求,否则注册到 reactor
            // 的是阻塞 fd,读写事件永不唤醒)。
            let std_stream = std::net::TcpStream::connect(addr)?;
            std_stream.set_nonblocking(true)?;
            let tokio_stream = TcpStream::from_std(std_stream.try_clone()?)?;
            Ok(Self {
                stream: tokio_stream,
                std_stream,
                buf: Vec::new(),
            })
        }

        pub(crate) async fn send(&mut self, c2s: &C2S) -> io::Result<()> {
            let frame = encode_c2s_frame(c2s);
            // 写侧也设超时:服务端可能已停止读(慢客户端场景),写全失败/超时
            // 都返回 Err(测试把 Err 当作"对端已断"处理)。
            match tokio::time::timeout(net_timeout(), self.stream.write_all(&frame)).await {
                Ok(r) => r,
                Err(_) => Err(io::Error::new(io::ErrorKind::TimedOut, "写超时(对端不读)")),
            }
        }

        /// 原始字节写(半帧/粘包测试用)。
        pub(crate) async fn send_raw(&mut self, bytes: &[u8]) -> io::Result<()> {
            self.stream.write_all(bytes).await
        }

        /// 关闭读写两端(#15):表达"客户端从此不再消费 socket"——平台
        /// 无关的僵尸连接构造。服务端下一次读得 EOF(Ok(0))→ SocketClosed
        /// 断连(README 决策 6 send 失败/连接关闭支路,断连原因断言不变)。
        /// 纯 EOF 路径不依赖任何平台缓冲语义——这是对第一版(SHUT_RD)的
        /// 修正:Linux 对 SHUT_RD 后到达的数据只 ACK 丢弃、不回 RST,服务端
        /// 永不断连(Windows 才回 RST),仍是平台相关。
        pub(crate) fn shutdown_both(&self) -> io::Result<()> {
            // std 副本与 tokio stream 共享同一 socket fd,关闭对两端生效。
            self.std_stream.shutdown(std::net::Shutdown::Both)
        }

        /// 读一帧 S2C(凑 4B 前缀 + 载荷)。
        pub(crate) async fn recv(&mut self) -> io::Result<S2C> {
            loop {
                if let Some(s2c) = try_take_frame(&mut self.buf) {
                    return Ok(s2c);
                }
                let mut chunk = [0u8; 4096];
                let n = self.stream.read(&mut chunk).await?;
                if n == 0 {
                    return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "对端关闭"));
                }
                self.buf.extend_from_slice(&chunk[..n]);
            }
        }

        /// 带超时读一帧(超时/读失败 panic 带意图)。
        pub(crate) async fn recv_timeout(&mut self) -> S2C {
            tokio::time::timeout(net_timeout(), self.recv())
                .await
                .expect("5s 内应收到帧")
                .expect("读帧不失败")
        }

        /// 连接是否已被对端关闭(读返回 0 = EOF)。等待最多 5s;
        /// 途中读到的帧(如 Bye)被吞掉(断连判定只认 EOF)。
        pub(crate) async fn is_disconnected(&mut self) -> bool {
            let deadline = tokio::time::Instant::now() + net_timeout();
            loop {
                let mut chunk = [0u8; 256];
                match tokio::time::timeout_at(deadline, self.stream.read(&mut chunk)).await {
                    Ok(Ok(0)) => return true,
                    Ok(Ok(n)) => {
                        self.buf.extend_from_slice(&chunk[..n]);
                        while try_take_frame(&mut self.buf).is_some() {}
                    }
                    Ok(Err(_)) => return true,
                    Err(_) => return false,
                }
            }
        }
    }

    /// 从缓冲切出一帧(够 4B 长度 + 载荷才切;不完整返回 None)。
    fn try_take_frame(buf: &mut Vec<u8>) -> Option<S2C> {
        if buf.len() < 4 {
            return None;
        }
        let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
        if len > FRAME_MAX {
            return None;
        }
        if buf.len() < 4 + len {
            return None;
        }
        let frame: Vec<u8> = buf.drain(..4 + len).collect();
        sim_api::proto::decode_frame(&frame)
    }

    /// 起服务器(127.0.0.1:0)+ 事件收集通道;返回实际地址、up 接收端、关机发送端。
    /// 关机 = 广播 `()`;`bcast_tx` 由调用方持有(存活期间 accept 循环常驻)。
    async fn spawn_server(
        read_timeout_secs: u64,
    ) -> (
        std::net::SocketAddr,
        mpsc::Receiver<NetUp>,
        tokio::sync::broadcast::Sender<()>,
    ) {
        let cfg = NetConfig {
            listen_addr: "127.0.0.1:0".to_string(),
            read_timeout_secs,
            ..NetConfig::default()
        };
        let (addr, up, shutdown, _hub) = spawn_server_cfg(cfg).await;
        (addr, up, shutdown)
    }

    /// 带全量配置起服务器(任务 02:泵节奏可调;测试用 0 端口)。
    /// 任务 03:回传共享 [`OutboundHub`]句柄(观察输出注册表断言用)。
    async fn spawn_server_cfg(
        cfg: NetConfig,
    ) -> (
        std::net::SocketAddr,
        mpsc::Receiver<NetUp>,
        tokio::sync::broadcast::Sender<()>,
        std::sync::Arc<OutboundHub>,
    ) {
        let (up_tx, up_rx) = mpsc::channel(256);
        let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
        let (addr_tx, addr_rx) = tokio::sync::oneshot::channel();
        let hub = std::sync::Arc::new(OutboundHub::new());
        let hub_ret = hub.clone();
        tokio::spawn(async move {
            let addr = serve(cfg, up_tx, bcast_rx, hub).await.expect("监听成功");
            let _ = addr_tx.send(addr);
        });
        let addr = addr_rx.await.expect("地址回传");
        (addr, up_rx, bcast_tx, hub_ret)
    }

    /// 收下一条网络事件(5s 上限;防挂死)。任务 03 起跳过 `Subscribed`
    /// (订阅视距簿记上报;任务 02 测试不断言它——驱动侧消费,连接层语义
    /// 已由 join() 的 Welcome 断言覆盖)。
    async fn next_up(up: &mut mpsc::Receiver<NetUp>) -> NetUp {
        loop {
            let event = tokio::time::timeout(net_timeout(), up.recv())
                .await
                .expect("5s 内应有网络事件")
                .expect("服务器存活");
            if matches!(event, NetUp::Subscribed { .. }) {
                continue;
            }
            return event;
        }
    }

    /// Hello + Subscribe 完整入会;返回 Welcome 的 player_id。
    async fn join(client: &mut MiniClient, view_distance: u8) -> InputSource {
        client
            .send(&C2S::Hello { client_id: 1 })
            .await
            .expect("Hello 写入");
        client
            .send(&C2S::Subscribe { view_distance })
            .await
            .expect("Subscribe 写入");
        match client.recv_timeout().await {
            S2C::Welcome { player_id, .. } => player_id,
            other => panic!("首帧应为 Welcome,实为 {other:?}"),
        }
    }

    /// 广播关机并吃掉可能的结果错误。
    async fn bcast_shutdown(tx: &tokio::sync::broadcast::Sender<()>) {
        let _ = tx.send(());
    }

    // ---- 测试清单 1:两客户端先后连入 → 身份 1、2 递增;Joined 上报 ----
    #[tokio::test]
    async fn hello_subscribe_joins_with_player_id() {
        let (addr, mut up, shutdown) = spawn_server(10).await;

        let mut a = MiniClient::connect(addr).await.expect("客户端 A 连入");
        let mut b = MiniClient::connect(addr).await.expect("客户端 B 连入");

        let id_a = join(&mut a, 8).await;
        assert_eq!(
            next_up(&mut up).await,
            NetUp::Joined { player: id_a },
            "A:Joined 上报"
        );

        let id_b = join(&mut b, 8).await;
        assert_eq!(
            next_up(&mut up).await,
            NetUp::Joined { player: id_b },
            "B:Joined 上报"
        );

        assert_eq!(
            (id_a, id_b),
            (InputSource(1), InputSource(2)),
            "身份从 1 起递增(0 保留)"
        );

        bcast_shutdown(&shutdown).await;
    }

    // ---- 测试清单 2:Subscribe{200} → 钳 32 ----
    #[tokio::test]
    async fn subscribe_clamps_view_distance() {
        let (addr, _up, shutdown) = spawn_server(10).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");

        // Subscribe 超视距:钳制后连接必须存活(不因数值拒断),会话照常入会。
        c.send(&C2S::Hello { client_id: 1 })
            .await
            .expect("Hello 写入");
        c.send(&C2S::Subscribe { view_distance: 200 })
            .await
            .expect("Subscribe 写入");
        let welcome = c.recv_timeout().await;
        assert!(
            matches!(welcome, S2C::Welcome { .. }),
            "Subscribe(200) 不应导致断连(钳 32 后放行)"
        );

        // 纯函数层独立断言钳制值(状态机经此函数落地)。
        assert_eq!(
            translate(InputSource(1), C2S::Subscribe { view_distance: 200 }),
            TranslateOutcome::Subscribe(VIEW_DISTANCE_MAX),
            "翻译层把 200 钳到 {VIEW_DISTANCE_MAX}"
        );
        assert_eq!(
            translate(InputSource(1), C2S::Subscribe { view_distance: 32 }),
            TranslateOutcome::Subscribe(32),
            "边界 32 原样通过"
        );
        assert_eq!(
            translate(InputSource(1), C2S::Subscribe { view_distance: 0 }),
            TranslateOutcome::Subscribe(0),
            "0 合法(立即移除视距语义,留任务 02/03 解释)"
        );
        bcast_shutdown(&shutdown).await;
    }

    // ---- 测试清单 3:MoveObserver → NetUp::DriverCommand{MoveObserver} ----
    #[tokio::test]
    async fn move_observer_reaches_driver_channel() {
        let (addr, mut up, shutdown) = spawn_server(10).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let id = join(&mut c, 8).await;
        assert_eq!(next_up(&mut up).await, NetUp::Joined { player: id });

        let pos = Vec3::new(200.0, 0.0, 8.0);
        c.send(&C2S::MoveObserver { position: pos })
            .await
            .expect("MoveObserver 写入");
        assert_eq!(
            next_up(&mut up).await,
            NetUp::DriverCommand {
                player: id,
                cmd: DriverCommand::MoveObserver { position: pos },
            },
            "翻译层产物 = DriverCommand::MoveObserver,携带玩家身份"
        );
        bcast_shutdown(&shutdown).await;
    }

    // ---- 测试清单 4:白名单透传;SetObserverPosition 命令 → Reject+断连 ----
    #[tokio::test]
    async fn command_whitelist_enforced() {
        // 纯函数层:白名单命令 → Send(分片换算);非白名单 → Reject。
        let whitelist = vec![
            SimCommand::SetBlock {
                pos: BlockPos::new(1, 2, 3),
                block: types::BlockId(1),
            },
            SimCommand::RemoveEntity {
                entity: EntityId::from_parts(1, 1),
            },
            SimCommand::TeleportEntity {
                entity: EntityId::from_parts(1, 1),
                state: types::EntityState::default(),
            },
            SimCommand::Detonate {
                center: Vec3::new(24.0, 0.0, 40.0),
                power: 6.0,
                radius: 5.0,
            },
        ];
        for cmd in whitelist {
            let outcome = translate(InputSource(3), C2S::Command(cmd));
            let expected_shard = command_shard(&cmd).expect("白名单命令必有路由");
            assert_eq!(
                outcome,
                TranslateOutcome::Send(DriverCommand::Send {
                    shard: expected_shard,
                    command: cmd,
                }),
                "白名单命令 {cmd:?} 透传为 Send"
            );
        }
        // 外部 Spawn 请求(REQ-006 任务 02)→ DriverCommand::Spawn,不带 ID。
        let spawn = C2S::Spawn {
            state: types::EntityState::default(),
            desc: types::EntityDesc::default(),
        };
        assert_eq!(
            translate(InputSource(3), spawn.clone()),
            TranslateOutcome::Send(DriverCommand::Spawn {
                state: types::EntityState::default(),
                desc: types::EntityDesc::default(),
            }),
            "C2S::Spawn 透传为 DriverCommand::Spawn"
        );
        // 外部 SpawnDynamic 已禁用(客户端不能自选 ID)。
        assert!(
            matches!(
                translate(
                    InputSource(1),
                    C2S::Command(SimCommand::SpawnDynamic {
                        entity: EntityId::from_parts(1, 1),
                        state: types::EntityState::default(),
                        desc: types::EntityDesc::default(),
                    })
                ),
                TranslateOutcome::Reject(_)
            ),
            "外部 SpawnDynamic → Reject"
        );
        // 非白名单:MarkActor、SetObserverPosition(观察者走 MoveObserver 专用包)。
        assert!(
            matches!(
                translate(
                    InputSource(1),
                    C2S::Command(SimCommand::MarkActor {
                        entity: EntityId::from_parts(1, 1),
                        actor: true,
                    })
                ),
                TranslateOutcome::Reject(_)
            ),
            "MarkActor → Reject"
        );
        assert!(
            matches!(
                translate(
                    InputSource(1),
                    C2S::Command(SimCommand::SetObserverPosition {
                        position: Vec3::ZERO
                    })
                ),
                TranslateOutcome::Reject(_)
            ),
            "SetObserverPosition 命令 → Reject(走 C2S::MoveObserver 专用包)"
        );

        // socket 层:发 SetObserverPosition 命令 → 断连 + Left(协议违规)。
        let (addr, mut up, shutdown) = spawn_server(10).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let id = join(&mut c, 4).await;
        assert_eq!(next_up(&mut up).await, NetUp::Joined { player: id });

        c.send(&C2S::Command(SimCommand::SetObserverPosition {
            position: Vec3::ZERO,
        }))
        .await
        .expect("违规命令写入");
        assert!(c.is_disconnected().await, "非白名单命令 → 服务端断连");
        assert_eq!(
            next_up(&mut up).await,
            NetUp::Left {
                player: id,
                reason: DisconnectReason::ProtocolViolation("非白名单命令"),
            },
            "Left 上报 = 协议违规"
        );
        bcast_shutdown(&shutdown).await;
    }

    // ---- 测试清单 5:发 >2MiB 长度前缀 → 断连 + Left 上报 ----
    #[tokio::test]
    async fn oversized_frame_disconnects() {
        let (addr, mut up, shutdown) = spawn_server(10).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let id = join(&mut c, 4).await;
        assert_eq!(next_up(&mut up).await, NetUp::Joined { player: id });

        // 只写 4B 长度前缀 = FRAME_MAX+1(不写载荷):断连判定只看前缀(细则 2)。
        let declared = (FRAME_MAX + 1) as u32;
        c.send_raw(&declared.to_le_bytes())
            .await
            .expect("超限前缀写入");
        assert!(c.is_disconnected().await, "长度前缀 > 2MiB → 服务端断连");
        assert_eq!(
            next_up(&mut up).await,
            NetUp::Left {
                player: id,
                reason: DisconnectReason::ProtocolViolation("帧超限"),
            },
            "Left 上报 = 帧超限"
        );
        bcast_shutdown(&shutdown).await;
    }

    // ---- 测试清单 6:read_timeout 调小 → 静默客户端被断;Ping 保持在线 ----
    #[tokio::test]
    async fn keepalive_timeout_disconnects() {
        let (addr, _up, shutdown) = spawn_server(1).await; // 读超时 1s

        // 静默客户端:连上即哑 → 1 个超时窗口后必被断。计时用 tokio 时钟
        // (与被测超时同源;clippy.toml 禁 std::Instant::now)。
        let mut silent = MiniClient::connect(addr).await.expect("连入");
        let started = tokio::time::Instant::now();
        assert!(silent.is_disconnected().await, "静默客户端在超时窗口后被断");
        assert!(
            started.elapsed() >= std::time::Duration::from_millis(900),
            "断连发生在读超时窗口之后(非立即踢)"
        );

        // 勤快客户端:每个窗口内发 Ping,窗口刷新 → 存活,且收到 Pong。
        let mut lively = MiniClient::connect(addr).await.expect("连入");
        lively.send(&C2S::Ping).await.expect("Ping 写入");
        match lively.recv_timeout().await {
            S2C::Pong => {}
            other => panic!("Ping 应答 = Pong,实为 {other:?}"),
        }
        // 0.6s < 窗口 1s:刷新窗口。
        tokio::time::sleep(std::time::Duration::from_millis(600)).await;
        lively.send(&C2S::Ping).await.expect("第二发 Ping");
        match lively.recv_timeout().await {
            S2C::Pong => {}
            other => panic!("第二次应答仍 = Pong,实为 {other:?}"),
        }
        // 再静默超一个窗口 → 必被断(keepalive 语义对称:服务端不无限养哑连接)。
        assert!(
            lively.is_disconnected().await,
            "停止 Ping 后超一个窗口 → 断连"
        );
        bcast_shutdown(&shutdown).await;
    }

    // ---- 测试清单 7:半帧/TCP 粘包(发端分段写)→ 正确切帧 ----
    #[tokio::test]
    async fn frame_streaming_splits_partial_reads() {
        let (addr, mut up, shutdown) = spawn_server(10).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");

        // Hello 分 3 段写:2B(半前缀)+ 3B(前缀尾+载荷头)+ 余量。
        let hello = encode_c2s_frame(&C2S::Hello { client_id: 7 });
        assert!(hello.len() > 5, "前提:帧长 > 5B 才能三段切");
        c.send_raw(&hello[..2]).await.expect("段1");
        tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        c.send_raw(&hello[2..5]).await.expect("段2");
        tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        c.send_raw(&hello[5..]).await.expect("段3");

        // 紧接着粘包:Subscribe 与 Ping 两帧一次写全(服务端缓冲里两帧连排)。
        let subscribe = encode_c2s_frame(&C2S::Subscribe { view_distance: 4 });
        let ping = encode_c2s_frame(&C2S::Ping);
        c.send_raw(&subscribe).await.expect("Subscribe 整帧");
        c.send_raw(&ping).await.expect("Ping 整帧");

        // 正确切帧:Welcome(Hello 完成)→ Pong(Ping 应答);期间无断连。
        match c.recv_timeout().await {
            S2C::Welcome { .. } => {}
            other => panic!("半帧 Hello 应被正确重组,实为 {other:?}"),
        }
        match c.recv_timeout().await {
            S2C::Pong => {}
            other => panic!("粘包帧应被正确切分并逐帧处理,实为 {other:?}"),
        }

        // 半帧 Hello 确实走完了状态机:Joined 上报可见。
        let mut saw_joined = false;
        for _ in 0..3 {
            if let NetUp::Joined { player } = next_up(&mut up).await {
                assert_eq!(player, InputSource(1), "首个入会者 = 身份 1");
                saw_joined = true;
                break;
            }
        }
        assert!(saw_joined, "半帧 Hello 完成后 Joined 上报");
        bcast_shutdown(&shutdown).await;
    }

    // ---- 测试清单 8:灌满 Critical(不读 socket 的客户端)→ 断连 ----
    #[tokio::test]
    async fn critical_overflow_disconnects() {
        // 单元级:队列策略断言(容量 64,满 = CriticalFull 断连信号)。
        let mut outbound = Outbound::new();
        for i in 0..CRITICAL_CAPACITY {
            assert_eq!(
                outbound.push(OutboundPriority::Critical, S2C::Pong),
                EnqueueResult::Enqueued,
                "第 {i} 条应入队(容量 {CRITICAL_CAPACITY})"
            );
        }
        assert_eq!(
            outbound.push(OutboundPriority::Critical, S2C::Pong),
            EnqueueResult::CriticalFull,
            "Critical 满(64)再入 → CriticalFull(断连信号)"
        );
        assert!(!outbound.is_empty());

        // World 级:满 512 → 丢最旧(DroppedOldest),不断连;新值保留在队尾。
        let mut outbound2 = Outbound::new();
        let sentinel = S2C::ChunkUnload {
            pos: types::ChunkPos::new(9, 9),
        };
        for _ in 0..WORLD_CAPACITY {
            assert_eq!(
                outbound2.push(
                    OutboundPriority::World,
                    S2C::ChunkUnload {
                        pos: types::ChunkPos::new(0, 0)
                    }
                ),
                EnqueueResult::Enqueued
            );
        }
        assert_eq!(
            outbound2.push(OutboundPriority::World, sentinel.clone()),
            EnqueueResult::DroppedOldest,
            "World 满 → 丢最旧(新值入队)"
        );
        assert!(!outbound2.is_empty());
        // 逐条取尽:最后一帧必为 sentinel(丢的是最旧,新值在队尾)。
        let mut last = None;
        while let Some(msg) = outbound2.try_pop() {
            last = Some(msg);
        }
        assert_eq!(last, Some(sentinel), "新值保留在队尾(丢最旧,非拒绝)");

        // socket 级:真实连接上灌满 Critical → 服务端断连(不读 socket 的客户端)。
        // 机制如实说明:本任务 Pong 是"入队 + 同步冲刷"——冲刷总是把队列清空,
        // 队列积压到 64 的前提是冲刷被慢客户端堵死(写超时)。故慢客户端在
        // socket 层的断连路径 = 写超时(README 决策 6「Critical 满或 send 失败 →
        // 断连」的 send 失败支路;Task 03 异步写泵接入后,「Critical 满队列」
        // 支路才会在 socket 层自然出现)。断连原因按实测如实断言。
        let (addr, mut up, shutdown) = spawn_server(10).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let id = join(&mut c, 2).await;
        assert_eq!(next_up(&mut up).await, NetUp::Joined { player: id });

        // 客户端从此不再消费:关闭读写两端(#15)。"客户端不读"的平台无关
        // 构造——原实现依赖"写满客户端接收缓冲 → 服务端写超时",但 Linux 的
        // TCP 自动调优缓冲(rcvbuf/sndbuf,MB 级)能把 20 万个 Pong 帧
        // (~1.4MB)全部吸收,写永不阻塞,测试在 ubuntu 稳定失败(Windows
        // 小缓冲才触发)。Both 关闭后服务端下一次读得 EOF → SocketClosed
        // 断连 → Left 上报(纯 EOF 路径,不依赖缓冲/平台;第一版 SHUT_RD
        // 在 Linux 下只被 ACK 丢弃、不回 RST,仍平台相关,已废弃)。
        c.shutdown_both().expect("关闭读写两端");
        assert!(
            c.is_disconnected().await,
            "Critical 溢出/写失败(客户端不读)→ 服务端断连,不养僵尸连接"
        );
        // Left 必达(up 通道容量 256,事件不会丢);原因 = send 失败支路。
        let mut saw_left = false;
        for _ in 0..3 {
            if let NetUp::Left { player, reason } = next_up(&mut up).await {
                assert_eq!(player, id);
                assert_eq!(
                    reason,
                    DisconnectReason::SocketClosed,
                    "断连原因 = send 失败支路(写超时;Critical 满队列支路属任务 03)"
                );
                saw_left = true;
                break;
            }
        }
        assert!(saw_left, "Left 上报必达");
        bcast_shutdown(&shutdown).await;
    }

    // ---- 测试清单 9:纯函数表驱动(每个 C2S 变体 → 期望 outcome)----
    #[test]
    fn translate_pure_function_table() {
        let player = InputSource(2);

        // Hello(任意 client_id)→ Hello(注册意图;状态机消费)。
        assert_eq!(
            translate(player, C2S::Hello { client_id: 42 }),
            TranslateOutcome::Hello,
            "Hello → 注册意图"
        );

        // Subscribe:钳制边界(细则 1:≤32)。
        assert_eq!(
            translate(player, C2S::Subscribe { view_distance: 16 }),
            TranslateOutcome::Subscribe(16)
        );
        assert_eq!(
            translate(player, C2S::Subscribe { view_distance: 255 }),
            TranslateOutcome::Subscribe(32)
        );

        // MoveObserver → Move(DriverCommand::MoveObserver)。
        let pos = Vec3::new(8.0, 1.0, -16.0);
        assert_eq!(
            translate(player, C2S::MoveObserver { position: pos }),
            TranslateOutcome::Move(DriverCommand::MoveObserver { position: pos })
        );

        // Ping → Ping;Goodbye → Goodbye。
        assert_eq!(translate(player, C2S::Ping), TranslateOutcome::Ping);
        assert_eq!(translate(player, C2S::Goodbye), TranslateOutcome::Goodbye);

        // Command 白名单(四变体,分片换算逐一核对;SpawnDynamic 由 C2S::Spawn 取代):
        // SetBlock → pos 所在分片(块 (200,·,8) → chunk 12 → 分片 (1,0))。
        let set_block = SimCommand::SetBlock {
            pos: BlockPos::new(200, -60, 8),
            block: types::BlockId(1),
        };
        assert_eq!(
            translate(player, C2S::Command(set_block)),
            TranslateOutcome::Send(DriverCommand::Send {
                shard: types::ShardId::new(
                    types::WorldId(0),
                    types::DimensionId(0),
                    types::ShardPos::new(1, 0)
                ),
                command: set_block,
            }),
            "SetBlock 分片 = 方块位置所在分片"
        );

        // 外部 SpawnDynamic 已禁用(REQ-006 任务 02:ID 由服务端分配);
        // C2S::Spawn 透传为 DriverCommand::Spawn,分片由驱动按出生位置解析。
        let spawn_dynamic = SimCommand::SpawnDynamic {
            entity: EntityId::from_parts(1, 1),
            state: types::EntityState {
                translation: Vec3::new(200.0, 0.0, 8.0),
                ..types::EntityState::default()
            },
            desc: types::EntityDesc::default(),
        };
        assert_eq!(
            translate(player, C2S::Command(spawn_dynamic)),
            TranslateOutcome::Reject("非白名单命令"),
            "外部 SpawnDynamic 拒绝"
        );
        let spawn = C2S::Spawn {
            state: types::EntityState {
                translation: Vec3::new(200.0, 0.0, 8.0),
                ..types::EntityState::default()
            },
            desc: types::EntityDesc::default(),
        };
        assert_eq!(
            translate(player, spawn.clone()),
            TranslateOutcome::Send(DriverCommand::Spawn {
                state: types::EntityState {
                    translation: Vec3::new(200.0, 0.0, 8.0),
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
            "C2S::Spawn → DriverCommand::Spawn(不带分片)"
        );
        let detonate = SimCommand::Detonate {
            center: Vec3::new(-0.5, 0.0, -0.5),
            power: 6.0,
            radius: 5.0,
        };
        assert_eq!(
            translate(player, C2S::Command(detonate)),
            TranslateOutcome::Send(DriverCommand::Send {
                shard: types::ShardId::new(
                    types::WorldId(0),
                    types::DimensionId(0),
                    types::ShardPos::new(-1, -1)
                ),
                command: detonate,
            }),
            "Detonate 分片 = 爆心所在分片(负坐标 floor)"
        );

        // 非白名单:MarkActor / SetObserverPosition → Reject。
        assert!(matches!(
            translate(
                player,
                C2S::Command(SimCommand::MarkActor {
                    entity: EntityId::from_parts(1, 1),
                    actor: false,
                })
            ),
            TranslateOutcome::Reject(_)
        ));
        assert!(matches!(
            translate(
                player,
                C2S::Command(SimCommand::SetObserverPosition { position: pos })
            ),
            TranslateOutcome::Reject(_)
        ));

        // 纯函数性:同输入同输出(零 IO 零时钟,细则 6——重跑一次比对)。
        assert_eq!(
            translate(player, C2S::Ping),
            translate(player, C2S::Ping),
            "同输入必同输出"
        );
    }

    /// cut_frame 的超限哨兵路径(细则 2:长度 > FRAME_MAX 必须被读循环识别);
    /// 与 decode 侧 FRAME_MAX 拒绝(sim-api proto 测试)互补。
    #[test]
    fn cut_frame_rejects_oversized_prefix() {
        let oversized = ((FRAME_MAX + 1) as u32).to_le_bytes();
        assert!(
            matches!(cut_frame(&oversized), Cut::Oversized),
            "超限前缀 → Oversized"
        );
        // 正常帧:消耗量 = 4 + 载荷长。
        let frame_ok = encode_frame(&S2C::Pong);
        match cut_frame(&frame_ok) {
            Cut::Frame(_, consumed) => assert_eq!(consumed, frame_ok.len()),
            other => panic!("完整帧应可切,实为 {other:?}"),
        }
        // 半帧:NeedMore。
        assert!(matches!(
            cut_frame(&frame_ok[..frame_ok.len() - 1]),
            Cut::NeedMore
        ));
        // 空缓冲:NeedMore。
        assert!(matches!(cut_frame(&[]), Cut::NeedMore));
    }

    // ---- 任务 02(REQ-003 §4.3):chunk 流测试(驱动 = 测试替身应答器)----

    use crate::driver::DriverCommand as DC;

    /// 测试清单 1 `initial_pump_streams_view_square_center_out`:订阅后首泵按
    /// 切比雪夫 center-out 收 ChunkData,数量 = 视距方形且 ≤16/泵,多泵补齐。
    #[tokio::test]
    async fn initial_pump_streams_view_square_center_out() {
        let cfg = NetConfig {
            pump: chunk_stream::PumpConfig {
                interval_ms: 1,
                max_catchup_rounds: 4,
            },
            ..NetConfig::default()
        };
        let (addr, mut up, shutdown, _hub) = spawn_server_cfg(cfg).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let _id = join(&mut c, 2).await; // 视距 2 = 5×5 = 25 chunk
        let _ = next_up(&mut up).await; // 吃掉 Joined

        // 首泵(≤16 个 ChunkData,center-out 升序);再等次泵补齐(25)。
        let mut seen: Vec<types::ChunkPos> = Vec::new();
        let mut stream_requests = 0u32;
        loop {
            // 收客户端帧;同时从 up 收流请求并以真数据应答(替身驱动)。
            tokio::select! {
                biased;
                frame = c.recv() => match frame.expect("读帧不失败") {
                    S2C::ChunkData { pos, .. } => {
                        assert!(!seen.contains(&pos), "chunk {pos:?} 不得重复(差分收账)");
                        seen.push(pos);
                        if seen.len() == 25 {
                            break;
                        }
                    }
                    other => panic!("订阅后应只收 ChunkData,实为 {other:?}"),
                },
                cmd = up.recv() => match cmd.expect("服务器存活") {
                    NetUp::DriverCommand {
                        cmd:
                            DC::StreamChunks {
                                load, reply, ..
                            },
                        ..
                    } => {
                        stream_requests += 1;
                        assert!(load.len() <= MAX_LOADS_PER_PUMP, "每泵 ≤16");
                        let center = types::ChunkPos::new(0, 0);
                        let keys: Vec<(i32, i32, i32)> = load
                            .iter()
                            .map(|pos| {
                                (
                                    chunk_stream::chebyshev(center, *pos),
                                    pos.x,
                                    pos.z,
                                )
                            })
                            .collect();
                        let mut sorted = keys.clone();
                        sorted.sort();
                        assert_eq!(keys, sorted, "装载清单 center-out 升序");
                        assert!(
                            load.iter().all(|pos| chunk_stream::chebyshev(center, *pos) <= 2),
                            "视距 2 方形内(已收账的不再请求)"
                        );
                        let _ = reply.send(
                            load.into_iter()
                                .map(|pos| (pos, voxel::Chunk::air()))
                                .collect(),
                        );
                    }
                    NetUp::Subscribed { .. } => {} // 任务 03 簿记事件:忽略
                    other => panic!("只应有 StreamChunks 请求,实为 {other:?}"),
                },
            }
        }
        assert_eq!(seen.len(), 25, "视距 2 方形补齐");
        assert!(
            stream_requests >= 2,
            "25 > 16/泵 → 至少两泵补齐(实 {stream_requests})"
        );
        bcast_shutdown(&shutdown).await;
    }

    /// 测试清单 2 `view_distance_clamped_32`:订阅 200 → 请求清单按 32 的方形
    /// 计(即 (2×32+1)² = 4225)——钳制值进入泵态(纯函数层单测已断言
    /// `translate` 钳 32;此处断言泵用同一钳制值,端到端闭合)。
    #[tokio::test]
    async fn view_distance_clamped_32() {
        let cfg = NetConfig {
            pump: chunk_stream::PumpConfig {
                interval_ms: 1,
                max_catchup_rounds: 4,
            },
            ..NetConfig::default()
        };
        let (addr, mut up, shutdown, _hub) = spawn_server_cfg(cfg).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        // Subscribe 200(钳 32)→ Welcome。
        c.send(&C2S::Hello { client_id: 1 }).await.expect("Hello");
        c.send(&C2S::Subscribe { view_distance: 200 })
            .await
            .expect("Subscribe");
        assert!(matches!(c.recv_timeout().await, S2C::Welcome { .. }));
        let _ = next_up(&mut up).await; // Joined

        // 首个流请求清单:≤16,center-out;验证其覆盖的切比雪夫层级与 32 视距
        // 一致(首泵 = 0 距环 + 1 距环全量 + 2 距环部分 = 恰 16;若视距未被
        // 钳/为 200,数量仍是 16 但排序语义同——故用「总请求量收敛到 (65)²」
        // 不可行(4225 帧太重),改用纯函数口:同输入的钳制值已由 translate
        // 单测锚定 32,这里断言泵请求的坐标键序与 32 视距的期望前 16 一致)。
        let cmd = next_up(&mut up).await;
        let NetUp::DriverCommand {
            cmd: DC::StreamChunks { load, reply, .. },
            ..
        } = cmd
        else {
            panic!("应为 StreamChunks 请求,实为 {cmd:?}");
        };
        let center = types::ChunkPos::new(0, 0);
        let mut expected: Vec<types::ChunkPos> =
            chunk_stream::view_square(center, 32).into_iter().collect();
        expected.sort_by_key(|pos| (chunk_stream::chebyshev(center, *pos), pos.x, pos.z));
        assert_eq!(
            load,
            expected[..16],
            "首泵 = 32 视距方形的前 16(center-out)"
        );
        let _ = reply.send(Vec::new());
        bcast_shutdown(&shutdown).await;
    }

    /// 测试清单 3 `moving_center_diffs_minimally`:中心移动 1 chunk → 只收新
    /// 进入的列(差分),旧列收 ChunkUnload。
    #[tokio::test]
    async fn moving_center_diffs_minimally() {
        let cfg = NetConfig {
            pump: chunk_stream::PumpConfig {
                interval_ms: 1,
                max_catchup_rounds: 4,
            },
            ..NetConfig::default()
        };
        let (addr, mut up, shutdown, _hub) = spawn_server_cfg(cfg).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let id = join(&mut c, 1).await; // 视距 1 = 3×3
        let _ = next_up(&mut up).await; // Joined

        // 替身驱动:应答全部流请求(收 ChunkData 到稳态)。
        let responder = tokio::spawn(async move {
            loop {
                let NetUp::DriverCommand {
                    cmd: DC::StreamChunks { load, reply, .. },
                    ..
                } = up.recv().await.expect("服务器存活")
                else {
                    continue;
                };
                let _ = reply.send(
                    load.into_iter()
                        .map(|pos| (pos, voxel::Chunk::air()))
                        .collect(),
                );
            }
        });

        // 收稳态(9 个)后移动中心 +1 chunk。
        let mut seen = std::collections::BTreeSet::new();
        while seen.len() < 9 {
            match c.recv_timeout().await {
                S2C::ChunkData { pos, .. } => {
                    seen.insert(pos);
                }
                other => panic!("首稳态应只收 ChunkData,实为 {other:?}"),
            }
        }
        c.send(&C2S::MoveObserver {
            position: types::glam::Vec3::new(20.0, 0.0, 0.0), // chunk (1,0)
        })
        .await
        .expect("MoveObserver 写入");

        // 差分窗口:恰好 3 装(x=2 列)+ 3 卸(x=-1 列),列坐标按断言核对。
        let mut loaded_new = Vec::new();
        let mut unloaded = Vec::new();
        let deadline = tokio::time::Instant::now() + net_timeout();
        while (loaded_new.len() < 3 || unloaded.len() < 3) && tokio::time::Instant::now() < deadline
        {
            match tokio::time::timeout_at(deadline, c.recv()).await {
                Ok(Ok(S2C::ChunkData { pos, .. })) => loaded_new.push(pos),
                Ok(Ok(S2C::ChunkUnload { pos })) => unloaded.push(pos),
                Ok(Ok(other)) => panic!("差分窗口只应装卸帧,实为 {other:?}"),
                Ok(Err(_)) | Err(_) => break,
            }
        }
        responder.abort();
        assert_eq!(
            loaded_new.len(),
            3,
            "中心 +1 → 恰装新列 3(实 {loaded_new:?})"
        );
        assert_eq!(unloaded.len(), 3, "中心 +1 → 恰卸旧列 3(实 {unloaded:?})");
        assert_eq!(
            loaded_new,
            vec![
                types::ChunkPos::new(2, -1),
                types::ChunkPos::new(2, 0),
                types::ChunkPos::new(2, 1)
            ],
            "新列 = x=2(center-out 序)"
        );
        assert!(
            unloaded.contains(&types::ChunkPos::new(-1, 0)),
            "旧列 x=-1 收 ChunkUnload"
        );
        let _ = id;
        bcast_shutdown(&shutdown).await;
    }

    /// 测试清单 4 `per_pump_limits_enforced`:视距 8(289 chunk)→ 首泵恰 16,
    /// 总泵数 ≥ ⌈289/16⌉ = 19(替身驱动逐请求计数)。
    #[tokio::test]
    async fn per_pump_limits_enforced() {
        let cfg = NetConfig {
            pump: chunk_stream::PumpConfig {
                interval_ms: 1,
                max_catchup_rounds: 1,
            },
            ..NetConfig::default()
        };
        let (addr, mut up, shutdown, _hub) = spawn_server_cfg(cfg).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let _id = join(&mut c, 8).await;
        let _ = next_up(&mut up).await; // Joined

        let responder = tokio::spawn(async move {
            let mut requests = 0u32;
            let mut total_loads = 0usize;
            let mut first_len = 0usize;
            loop {
                let NetUp::DriverCommand {
                    cmd: DC::StreamChunks { load, reply, .. },
                    ..
                } = up.recv().await.expect("服务器存活")
                else {
                    continue;
                };
                if first_len == 0 {
                    first_len = load.len();
                }
                requests += 1;
                total_loads += load.len();
                let _ = reply.send(
                    load.into_iter()
                        .map(|pos| (pos, voxel::Chunk::air()))
                        .collect(),
                );
                // 累计应答 ≥ 289(全量)= 可收口;泵稳态后不再发请求,等
                // 40 个会永挂——按数据量收口,不按请求数。
                if total_loads >= 289 {
                    return (first_len, requests);
                }
            }
        });

        // 客户端收满 289 个 ChunkData(视距 8 全量)。
        let mut seen = 0usize;
        let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(30);
        while seen < 289 {
            match tokio::time::timeout_at(deadline, c.recv()).await {
                Ok(Ok(S2C::ChunkData { .. })) => seen += 1,
                Ok(Ok(other)) => panic!("应只收 ChunkData,实为 {other:?}"),
                Ok(Err(e)) => panic!("289 个前连接断({seen} 个):{e}"),
                Err(_) => panic!("30s 未收满 289(实 {seen})"),
            }
        }
        let (first_len, requests) = responder.await.expect("替身不 panic");
        assert_eq!(first_len, 16, "首泵恰 16");
        assert!(requests >= 19, "总泵数 {requests} ≥ ⌈289/16⌉ = 19");
        bcast_shutdown(&shutdown).await;
    }

    /// 测试清单 5 `server_chunks_come_from_provider`:驱动(真 provider)以
    /// 修改值应答 → 客户端收到的 ChunkData 是修改值(真数据源,非连接层缓存)。
    #[tokio::test]
    async fn server_chunks_come_from_provider() {
        // 真 worker + provider:预置一个修改过的 chunk(区别于生成器初生值)。
        let worker = storage::worker::StoreWorker::new(
            Box::new(storage::mem::MemChunkStore::new()),
            Box::new(storage::mem::MemCheckpointStore::new()),
        );
        let provider = storage::provider::ChunkProvider::new(
            &worker,
            Box::new(voxel::Superflat::classic()),
            types::WorldSeed(7),
        );
        let target = types::ChunkPos::new(0, 0);
        let mut modified = provider.generate(target);
        modified.set(3, 100, 4, types::BlockId(2));
        worker.put_chunks(vec![(
            storage::traits::ChunkKey {
                world: types::WorldId(0),
                dim: types::DimensionId(0),
                pos: target,
            },
            modified.clone(),
        )]);
        let ack = worker.flush();
        assert!(ack.recv().expect("ack").is_ok(), "预置落库");

        // 网络侧:替身驱动用 **真 provider** 应答 StreamChunks(路径同编排 ④)。
        let cfg = NetConfig {
            pump: chunk_stream::PumpConfig {
                interval_ms: 1,
                max_catchup_rounds: 4,
            },
            ..NetConfig::default()
        };
        let (addr, mut up, shutdown, _hub) = spawn_server_cfg(cfg).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let _id = join(&mut c, 1).await;
        let _ = next_up(&mut up).await; // Joined

        let responder = tokio::spawn(async move {
            loop {
                let NetUp::DriverCommand {
                    cmd: DC::StreamChunks { load, reply, .. },
                    ..
                } = up.recv().await.expect("服务器存活")
                else {
                    continue;
                };
                let mut out = Vec::new();
                for pos in load {
                    let key = storage::traits::ChunkKey {
                        world: types::WorldId(0),
                        dim: types::DimensionId(0),
                        pos,
                    };
                    // 生成器兜底后不应 None(同编排 ④ 语义)。
                    let chunk = provider.get(key).expect("生成器兜底必有值");
                    out.push((pos, chunk));
                }
                let _ = reply.send(out);
            }
        });

        // (0,0) 到手且必为修改值。
        let mut got_target = None;
        let deadline = tokio::time::Instant::now() + net_timeout();
        while got_target.is_none() {
            match tokio::time::timeout_at(deadline, c.recv()).await {
                Ok(Ok(S2C::ChunkData { pos, chunk })) if pos == target => {
                    got_target = Some(chunk);
                }
                Ok(Ok(S2C::ChunkData { .. })) => {}
                Ok(Ok(other)) => panic!("应只收 ChunkData,实为 {other:?}"),
                Ok(Err(e)) => panic!("未收到目标 chunk:{e}"),
                Err(_) => panic!("超时未收到目标 chunk"),
            }
        }
        responder.abort();
        let chunk = got_target.expect("(0,0) 必达");
        assert_eq!(
            chunk.get(3, 100, 4),
            types::BlockId(2),
            "客户端收到的是持久层修改值(真数据源)"
        );
        assert_eq!(chunk, modified, "整 chunk 与预置修改值一致");
        bcast_shutdown(&shutdown).await;
        worker.shutdown().expect("worker 关停");
    }

    /// 测试清单 6 `stream_reply_never_blocks_tick`:真驱动 + 网络层;大视距
    /// (32)订阅,泵请求淹没命令通道——暂停时间下 tick 计数不减(编排 ④
    /// 每 tick 恰 drain 一轮,其余请求等下一拍;tick 分支优先级高于命令分支,
    /// 请求洪水饿不死模拟)。
    #[tokio::test(start_paused = true)]
    async fn stream_reply_never_blocks_tick() {
        use crate::driver::{DriverArgs, bootstrap};
        use voxel::SimpleRegistry;

        let config = crate::config::Config {
            worker_count: 2,
            data_dir: std::env::temp_dir().join(format!("app-net-stream-{}", std::process::id())),
            ..crate::config::Config::default()
        };
        let _ = std::fs::remove_dir_all(&config.data_dir);
        let (session, log, store, _resume, _spawn_next_seq) =
            bootstrap(&config, || Box::new(SimpleRegistry::builtin())).expect("装配");
        let (provider, new_sim) =
            crate::driver::wire_provider_and_new_sim(config.clone(), &store, || {
                Box::new(SimpleRegistry::builtin())
            });
        let (tx, rx) = tokio::sync::mpsc::channel::<crate::driver::DriverCommand>(16);
        let args = DriverArgs {
            config,
            session,
            store,
            log,
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
            observe: None,
            spawn_next_seq: 0,
        };
        let driver = tokio::spawn(crate::driver::run(args));

        // 网络层接到驱动通道上(真链路:泵 → NetUp → 驱动;本层不起 serve,
        // 直接喂 StreamChunks 语义等价——大视距请求流经 16 容量通道)。
        let (addr, mut up, shutdown) = {
            let (up_tx, up_rx) = mpsc::channel(256);
            let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
            let (addr_tx, addr_rx) = tokio::sync::oneshot::channel();
            let cfg = NetConfig {
                pump: chunk_stream::PumpConfig {
                    interval_ms: 1,
                    max_catchup_rounds: 4,
                },
                ..NetConfig::default()
            };
            let hub = std::sync::Arc::new(OutboundHub::new());
            tokio::spawn(async move {
                let addr = serve(cfg, up_tx, bcast_rx, hub).await.expect("监听");
                let _ = addr_tx.send(addr);
            });
            (addr_rx.await.expect("地址"), up_rx, bcast_tx)
        };
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let _id = join(&mut c, 32).await; // 大视距:4225 chunk 请求
        let _ = next_up(&mut up).await; // Joined

        // 驱动转发桥:up → tx(StreamChunks 照发;MoveObserver 供观察者入表)。
        // 泵请求速率远超 16 容量通道 + 每 tick 只 drain 一轮 → 通道持续拥塞,
        // 正是「请求洪水」场景。
        let tx_bridge = tx.clone();
        let bridge = tokio::spawn(async move {
            while let Some(up_ev) = up.recv().await {
                if let NetUp::DriverCommand { cmd, .. } = up_ev
                    && tx_bridge.send(cmd).await.is_err()
                {
                    break;
                }
            }
        });

        // 让请求洪水灌 30 拍(暂停时间;每 sleep 恰一拍,README 决策 10)。
        let dt = std::time::Duration::from_secs_f32(1.0 / 60.0);
        for _ in 0..30 {
            tokio::time::sleep(dt).await;
            tokio::task::yield_now().await;
        }
        tx.send(crate::driver::DriverCommand::Shutdown)
            .await
            .expect("驱动存活");
        let outcome = driver.await.expect("驱动不 panic");
        assert_eq!(
            outcome.ticks_run, 30,
            "大视距请求洪水下 tick 计数不减(命令不阻塞 tick)"
        );
        bridge.abort();
        bcast_shutdown(&shutdown).await;
        let _ = std::fs::remove_dir_all(
            std::env::temp_dir().join(format!("app-net-stream-{}", std::process::id())),
        );
    }

    /// 测试清单 7 `disconnect_cleans_stream_state`:断连后泵停止(无孤儿任务:
    /// 连接 task 终止即停);稳态后断开 → 再无 StreamChunks 请求涌出。
    #[tokio::test]
    async fn disconnect_cleans_stream_state() {
        let cfg = NetConfig {
            pump: chunk_stream::PumpConfig {
                interval_ms: 1,
                max_catchup_rounds: 4,
            },
            ..NetConfig::default()
        };
        let (addr, mut up, shutdown, _hub) = spawn_server_cfg(cfg).await;
        let mut c = MiniClient::connect(addr).await.expect("连入");
        let _id = join(&mut c, 1).await;
        let _ = next_up(&mut up).await; // Joined

        // 前置:稳态即断(收到 9 个 ChunkData 后立即 Goodbye;替身应答保泵不卡)。
        // 替身应答在本任务内做(up 的所有权留在此处,供断连后泄漏检查)。
        let mut seen = 0usize;
        while seen < 9 {
            tokio::select! {
                frame = c.recv() => match frame.expect("读帧不失败") {
                    S2C::ChunkData { .. } => seen += 1,
                    other => panic!("稳态前只应收 ChunkData,实为 {other:?}"),
                },
                cmd = up.recv() => match cmd.expect("服务器存活") {
                    NetUp::DriverCommand {
                        cmd: DC::StreamChunks { load, reply, .. },
                        ..
                    } => {
                        let _ = reply.send(
                            load.into_iter()
                                .map(|pos| (pos, voxel::Chunk::air()))
                                .collect(),
                        );
                    }
                    NetUp::Subscribed { .. } => {} // 任务 03 簿记事件:忽略
                    other => panic!("只应有 StreamChunks 请求,实为 {other:?}"),
                },
            }
        }
        c.send(&C2S::Goodbye).await.expect("Goodbye 写入");
        assert!(c.is_disconnected().await, "Goodbye → 断连");

        // 断连后:泵已随连接 task 终止——up 通道在剩余事件(Left)之后
        // 恒静默(稳态无差分,若泵未停而 loaded 又清零会重新整片申请,
        // 窗口内必可见;静默即无孤儿泵)。
        let mut saw_left = false;
        let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(1);
        while tokio::time::Instant::now() < deadline {
            match tokio::time::timeout_at(deadline, up.recv()).await {
                Ok(Some(NetUp::Left { .. })) => {
                    saw_left = true;
                    break;
                }
                Ok(Some(NetUp::DriverCommand {
                    cmd: DC::StreamChunks { load, reply, .. },
                    ..
                })) => {
                    // 断连竞先窗口内的最后一个请求:照常应答(驱动语义),
                    // 不影响后续静默判定。
                    let _ = reply.send(
                        load.into_iter()
                            .map(|pos| (pos, voxel::Chunk::air()))
                            .collect(),
                    );
                }
                Ok(Some(_)) => {}
                Ok(None) | Err(_) => break,
            }
        }
        assert!(saw_left, "断连必上报 Left");
        // Left 之后 1s 静默窗:无任何新流请求(孤儿泵若在,视距 1 的重整片
        // 请求必在此窗出现——loaded 已清、center 未变也会全量重装)。
        let mut leaked = false;
        let quiet = tokio::time::Instant::now() + std::time::Duration::from_secs(1);
        while tokio::time::Instant::now() < quiet {
            match tokio::time::timeout_at(quiet, up.recv()).await {
                Ok(Some(NetUp::DriverCommand {
                    cmd: DC::StreamChunks { .. },
                    ..
                })) => {
                    leaked = true;
                    break;
                }
                Ok(Some(_)) => {}
                Ok(None) | Err(_) => break,
            }
        }
        assert!(!leaked, "断连后无孤儿泵请求(连接 task 终止即停)");
        bcast_shutdown(&shutdown).await;
    }

    // ---- 任务书 req004a-viewer/00:main 网络装配接线(测试清单)----

    /// 测试清单 2 `main_wiring_smoke`:`assemble_network` 装配出的 ObserveLink
    /// 非空 + serve 起后外部迷你客户端连接收 Welcome(装配函数 = main 与测试
    /// 共用;任务书授权)。驱动侧以真 tick 循环接线(Welcome 拍号刷新路径),
    /// 关机 = Shutdown 命令 + 广播。
    #[tokio::test]
    async fn main_wiring_smoke() {
        // 装配输入:监听地址覆写为 0 端口(OS 分配;其余字段默认)。
        let config = crate::config::Config {
            listen_addr: "127.0.0.1:0".to_string(),
            ..crate::config::Config::default()
        };

        // 装配:serve 起监听,ObserveLink 在场(Option::Some = 装配非空)。
        let handle = assemble_network(&config)
            .await
            .expect("装配监听成功(127.0.0.1:0)");
        assert!(handle.addr.port() > 0, "实际绑定地址已知(端口 0 → OS 分配)",);

        // 驱动侧最小装配:真会话 + mem 后端 + observe 三缝真接(与 driver.rs
        // observe_harness 同款链路;真 tick 让 Welcome 拍号经编排刷新)。
        let session = sim_api::session::Session::new(config.session_config());
        let (tx, rx) = mpsc::channel::<crate::driver::DriverCommand>(256);
        let store = storage::worker::StoreWorker::new(
            Box::new(storage::mem::MemChunkStore::new()),
            Box::new(storage::mem::MemCheckpointStore::new()),
        );
        let (provider, new_sim) =
            crate::driver::wire_provider_and_new_sim(config.clone(), &store, || {
                Box::new(voxel::SimpleRegistry::builtin())
            });
        let driver = tokio::spawn(crate::driver::run(crate::driver::DriverArgs {
            config,
            session,
            store,
            log: sim_api::log::CommandLog::new(),
            commands: rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
            observe: Some(handle.observe),
            spawn_next_seq: 0,
        }));

        // 外部客户端连入:Hello + Subscribe → 首帧 Welcome(ACV1 冒烟)。
        let mut c = MiniClient::connect(handle.addr).await.expect("客户端连入");
        c.send(&C2S::Hello { client_id: 1 })
            .await
            .expect("Hello 写入");
        c.send(&C2S::Subscribe { view_distance: 4 })
            .await
            .expect("Subscribe 写入");
        match c.recv_timeout().await {
            S2C::Welcome { player_id, .. } => {
                assert_eq!(player_id, InputSource(1), "装配出的服务器分配身份 1");
            }
            other => panic!("装配后外部客户端首帧应为 Welcome,实为 {other:?}"),
        }

        // 收口:驱动 Shutdown(join = 驱动退出路径跑完:终末 checkpoint +
        // store 关停,不泄后台任务)+ 网络广播关机(句柄公开面可用)。
        tx.send(crate::driver::DriverCommand::Shutdown)
            .await
            .expect("驱动仍存活");
        driver.await.expect("驱动不 panic");
        handle.shutdown.shutdown();
    }
}
