//! client —— 协议客户端层(REQ-004A 任务 01 接口设计)。纯 tokio +
//! sim-api::proto(帧编解码全部复用,不重复实现协议);**零 macroquad 依赖**
//! (模块级 use 纪律,README 决策 1;e2e 可复用)。
//!
//! 线程模型(README 决策 3):[`ViewerClient::spawn`] 自带独立 tokio current_thread
//! runtime 跑连接循环(与 macroquad 主循环解耦),S2C → [`ViewEvent`]
//! 经 std mpsc 桥给主循环 `try_recv` 排空;控制端发 [`C2S::MoveObserver`] /
//! [`C2S::Goodbye`](有界通道,满即弃——观察者位姿 latest-wins,不背压渲染)。

use std::io;
use std::net::SocketAddr;
use std::sync::mpsc as std_mpsc;
use std::time::Duration;

use messaging::InputSource;
use sim_api::proto::{C2S, S2C, decode_frame, encode_c2s_frame};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use types::glam::Vec3;
use types::{BlockId, BlockPos, ChunkPos, EntityId, EntityState, Tick};
use voxel::Chunk;

/// 事件通道容量(主循环每帧排空;正常节奏不会积满)。
pub(crate) const EVENT_CAPACITY: usize = 4096;
/// 控制通道容量(移动节流后不会积满;满即弃,latest-wins 语义)。
pub(crate) const CONTROL_CAPACITY: usize = 64;
/// 连接握手等待上限(spawn 同步返回;超时 = Err)。
pub(crate) const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);
/// 事件(client 连接循环 → 主循环;通道桥)。
#[derive(Debug, Clone, PartialEq)]
pub enum ViewEvent {
    /// 入会完成(Subscribe 成功;player_id = 服务器分配的观察者身份)。
    Welcome { player_id: InputSource, tick: Tick },
    /// chunk 装载(整块替换)。
    Chunk { pos: ChunkPos, chunk: Chunk },
    /// chunk 卸载(移出视距)。
    ChunkGone { pos: ChunkPos },
    /// 实体流(增量 = 变化 + 消失;全量 = 整表对账,`full` 时以 states 为准
    /// 重建实体表——驱散幽灵实体,协议 EntityStates.full 同语义)。
    Entities {
        tick: Tick,
        states: Vec<(EntityId, EntityState)>,
        removed: Vec<EntityId>,
        full: bool,
    },
    /// 方块增量(凝固/挖掘/放置的渲染面)。
    Blocks { updates: Vec<(BlockPos, BlockId)> },
    /// 服务端分配的外部生成 ID(REQ-006 任务 02)。
    Spawned { entity: EntityId },
    /// 复合实体形状(REQ-010 §10 修订 4)。
    Shapes {
        shapes: Vec<(EntityId, types::CompoundShape)>,
    },
    /// 调试遥测包(REQ-011 §3.5.2;W04):仅交互模式订阅后收到;脚本模式
    /// 不订阅、收到也忽略。
    DebugStats {
        tick: Tick,
        counters: sim_api::proto::DebugCounters,
        recent: Vec<(Tick, sim_api::proto::FlowEventWire)>,
    },
    /// 断连(对端 Bye/EOF/IO 失败;reason = 尽力还原的说明)。
    Disconnected { reason: String },
    /// 主体状态流(REQ-012 A06):`(EntityId, BodyHash, 连杆姿态列表)`;
    /// 增量/全量/removed 语义与 [`ViewEvent::Entities`] 同款。
    Agents {
        tick: Tick,
        states: Vec<(EntityId, types::BodyHash, Vec<EntityState>)>,
        removed: Vec<EntityId>,
        full: bool,
    },
    /// 附身回执(REQ-012 A06):`Some` = 附身的主体;`None` = 回到飞行相机
    /// (主动解除或被服务器被迫解除)。
    Possessed { entity: Option<EntityId> },
}

/// 控制端(主循环 → 连接循环):观察者移动与主动离场。
/// REQ-010 §5.4:Command/Spawn 的载荷不再 `Copy`,本枚举随之只有 `Clone`。
#[derive(Debug, Clone, PartialEq)]
pub enum Control {
    /// 观察者位姿(渲染节流后送达;连接循环原样发 `C2S::MoveObserver`)。
    Move(Vec3),
    /// 白名单命令(引爆等;连接循环发 `C2S::Command`,翻译层在服务器侧)。
    /// ACV4 ③④ 的交互入口(评审后补,Refs #6)。
    Command(messaging::SimCommand),
    /// 同帧命令批(REQ-010 终审返修 1):脚本引擎一帧产出的一组命令(一次
    /// fill 的逐格 SetBlock 展开)是**一次原子编辑**——通道一条消息即天然的
    /// 批边界,连接循环一次 `write_all` 发出,服务器同一轮 read 收全、同一拍
    /// 落地。拆成多条 Command 会在桥线程搬运与 socket 写之间被打断,批裂开
    /// 后中间拍看到半个结构,承载判定对不完整视图求值误判塌落(door-lintel
    /// 石门楣间歇性塌落的根因之一)。
    Batch(Vec<messaging::SimCommand>),
    /// 外部生成请求(连接循环发 `C2S::Spawn`;服务端分配 ID 并回
    /// [`ViewEvent::Spawned`])。
    Spawn {
        state: EntityState,
        desc: types::EntityDesc,
    },
    /// 调试遥测订阅(REQ-011 §3.5.2;W04:连接循环发 `C2S::SubscribeDebug`,
    /// 服务端开始按 10 拍下发 [`S2C::DebugStats`];重复发送幂等)。
    SubscribeDebug,
    /// 主体生成(A06 §2:连接循环发 `C2S::SpawnAgent`;回执复用
    /// [`ViewEvent::Spawned`]——与方块 Spawn 按请求顺序对应,见
    /// [`PossessState`] 的请求队列)。
    SpawnAgent {
        body: String,
        position: Vec3,
        yaw: f32,
    },
    /// 附身(A06 §2;连接循环发 `C2S::Possess`)。
    Possess { entity: EntityId },
    /// 解除附身(A06 §2;连接循环发 `C2S::Unpossess`)。
    Unpossess,
    /// 主体意图(A06 §2;连接循环发 `C2S::AgentIntent`;move_xz 为**世界系**
    /// 水平速度;yaw 弧度)。
    AgentIntent {
        entity: EntityId,
        move_xz: (f32, f32),
        yaw: f32,
        jump: bool,
    },
    /// 主动离场(`C2S::Goodbye`)。
    Goodbye,
}

/// 控制句柄(主循环持有;发送失败 = 连接已亡,静默弃——断连经 ViewEvent 呈现)。
/// 内部 = std 通道 + 桥线程:std `try_send` 恒不阻塞渲染;桥线程把命令搬进
/// tokio 通道,连接循环 `select!` 才能同时等 socket 读与控制命令(轮询式
/// try_recv 会在阻塞 read 期间饿死控制面——实测挂死教训)。
#[derive(Clone)]
pub struct ControlHandle {
    tx: std_mpsc::SyncSender<Control>,
}

impl ControlHandle {
    /// 发观察者位姿(latest-wins:满即弃上一帧,渲染侧不做背压)。
    pub fn move_observer(&self, position: Vec3) {
        let _ = self.tx.try_send(Control::Move(position));
    }

    /// 发白名单命令(引爆;ACV4 ③④ 交互入口)。
    pub fn command(&self, command: messaging::SimCommand) {
        let _ = self.tx.try_send(Control::Command(command));
    }

    /// 发同帧命令批(REQ-010 终审返修 1):一次 fill 的逐格 SetBlock 是一次
    /// 原子编辑,须单条消息入队、单次 TCP 写发出。空批不占通道。
    pub fn command_batch(&self, commands: Vec<messaging::SimCommand>) {
        if !commands.is_empty() {
            let _ = self.tx.try_send(Control::Batch(commands));
        }
    }

    /// 发外部生成请求(REQ-006 任务 02:服务端分配 ID,客户端不选 ID)。
    pub fn spawn(&self, state: EntityState, desc: types::EntityDesc) {
        let _ = self.tx.try_send(Control::Spawn { state, desc });
    }

    /// 发调试遥测订阅(REQ-011 §3.5.2;W04:交互模式连上后发一次;服务端
    /// 每 10 拍回 [`S2C::DebugStats`] → [`ViewEvent::DebugStats`])。
    pub fn subscribe_debug(&self) {
        let _ = self.tx.try_send(Control::SubscribeDebug);
    }

    /// 发主体生成请求(A06 §2;服务端校验分配 ID,回执 `Spawned`)。
    pub fn spawn_agent(&self, body: String, position: Vec3, yaw: f32) {
        let _ = self.tx.try_send(Control::SpawnAgent {
            body,
            position,
            yaw,
        });
    }

    /// 发附身请求(A06 §2;回执 `Possessed`)。
    pub fn possess(&self, entity: EntityId) {
        let _ = self.tx.try_send(Control::Possess { entity });
    }

    /// 发解除附身(A06 §2;回执 `Possessed(None)`)。
    pub fn unpossess(&self) {
        let _ = self.tx.try_send(Control::Unpossess);
    }

    /// 发主体意图(A06 §2;保持状态——服务端直到下一条意图;move_xz 世界系)。
    pub fn agent_intent(&self, entity: EntityId, move_xz: (f32, f32), yaw: f32, jump: bool) {
        let _ = self.tx.try_send(Control::AgentIntent {
            entity,
            move_xz,
            yaw,
            jump,
        });
    }

    /// 主动离场(尽力而为;连接亡故时静默)。
    pub fn goodbye(&self) {
        let _ = self.tx.try_send(Control::Goodbye);
    }

    /// 测试构造(供 viewer / client 内部测试):有界内存通道(容量同生产 64),收端
    /// 原样读回 [`Control`] 消息——`forward_commands` 的发送层断言用,不起线程。
    #[doc(hidden)]
    pub fn for_test() -> (Self, std_mpsc::Receiver<Control>) {
        let (tx, rx) = std_mpsc::sync_channel(CONTROL_CAPACITY);
        (Self { tx }, rx)
    }
}

/// 协议客户端:S2C 帧 → [`ViewEvent`] 泵 + C2S 控制通道的独立线程封装。
pub struct ViewerClient;

/// Hello 的 client_id(任务书接口设计:固定 0——viewer 非权威输入,
/// 不需要唯一身份;服务器分配的 InputSource 才是观察者身份)。
pub const HELLO_CLIENT_ID: u64 = 0;

/// 读循环的 socket 缓冲块大小(4 KiB,app::net 迷你客户端同量级)。
const READ_CHUNK: usize = 4096;

/// 断连原因英文化(W04,W01 遗留):交互模式 HUD 只认 ASCII(macroquad
/// 位图字体不含 CJK)。
/// - 已知客户端原因映射成英文(`连接关闭`/`帧解码失败`/`连接失败:…`);
/// - `io::Error` 文本可能是系统语言,非 ASCII 字符一律替换 `?`;
/// - 服务器 `Bye` 原因本来就是英文,透传。脚本模式的 `draw_hud` 与
///   manifest 里的原因文本不动(不经此函数)。
#[must_use]
pub fn localize_disconnect_reason(reason: &str) -> String {
    let known = match reason.strip_prefix("连接失败:") {
        Some(rest) => Some(format!("connect failed: {}", ascii_escape(rest))),
        None => match reason {
            "连接关闭" => Some("connection closed".to_owned()),
            "帧解码失败" => Some("frame decode failed".to_owned()),
            _ => None,
        },
    };
    known.unwrap_or_else(|| ascii_escape(reason))
}

/// 非 ASCII 字符替换 `?`(io::Error 文本的系统语言防御;ASCII 原样保留)。
fn ascii_escape(text: &str) -> String {
    text.chars()
        .map(|c| if c.is_ascii() { c } else { '?' })
        .collect()
}

/// 断连原因 → 事件说明文本(尽力还原;EOF/IO 失败同路)。
fn disconnect_reason(err: &io::Error) -> String {
    format!("连接失败:{}", err)
}

/// S2C 帧 → [`ViewEvent`] 映射(纯函数;连接循环与 codec 一致性测试共用,
/// 单一事实源)。Pong(keepalive 应答)对渲染无意义 → `None` 吞掉。
pub fn s2c_to_event(s2c: S2C) -> Option<ViewEvent> {
    match s2c {
        S2C::Welcome { player_id, tick } => Some(ViewEvent::Welcome { player_id, tick }),
        S2C::ChunkData { pos, chunk } => Some(ViewEvent::Chunk { pos, chunk }),
        S2C::ChunkUnload { pos } => Some(ViewEvent::ChunkGone { pos }),
        S2C::EntityStates {
            tick,
            states,
            removed,
            full,
        } => Some(ViewEvent::Entities {
            tick,
            states,
            removed,
            full,
        }),
        S2C::BlockUpdates { updates } => Some(ViewEvent::Blocks { updates }),
        S2C::Spawned { entity } => Some(ViewEvent::Spawned { entity }),
        S2C::EntityShapes { shapes } => Some(ViewEvent::Shapes { shapes }),
        S2C::Pong => None,
        S2C::Bye { reason } => Some(ViewEvent::Disconnected { reason }),
        // 调试遥测(REQ-011 §3.5.2;W04):交互模式订阅后接收,映射成事件;
        // 脚本模式不订阅,不会收到。
        S2C::DebugStats {
            tick,
            counters,
            recent,
        } => Some(ViewEvent::DebugStats {
            tick,
            counters,
            recent,
        }),
        // 主体协议面(REQ-012 A03):A06 接附身模式与主体渲染——
        // AgentStates/Possessed 一比一映射成事件(与 Pong 不同:渲染/附身
        // 状态机要消费)。
        S2C::AgentStates {
            tick,
            states,
            removed,
            full,
        } => Some(ViewEvent::Agents {
            tick,
            states,
            removed,
            full,
        }),
        S2C::Possessed { entity } => Some(ViewEvent::Possessed { entity }),
        // 多人预留包(REQ-013 §3.3)与性能遥测包(C01b-3;仅 bevy client 消费,viewer 不消费)。
        S2C::InputAck { .. } | S2C::Correction { .. } | S2C::PerfStats { .. } => None,
    }
}

/// 缓冲切帧:凑满 4B LE 长度前缀 + 载荷才切(半帧等待;proto::decode_frame
/// 负责校验与解码,这里只做流式切分,与 app::net cut_frame 同式)。
fn try_cut_frame(buf: &mut Vec<u8>) -> Option<Vec<u8>> {
    if buf.len() < 4 {
        return None;
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    if buf.len() < 4 + len {
        return None;
    }
    Some(buf.drain(..4 + len).collect())
}

/// `Control` → `Vec<C2S>` 纯翻译(REQ-014 M01 §2.3):`Batch` 展开为多元素 vec
/// (保 REQ-010 批原子性:下游一次 `write_all`),其余为单元素 vec。
fn control_to_c2s(command: Control) -> Vec<C2S> {
    match command {
        Control::Move(position) => vec![C2S::MoveObserver { position }],
        Control::Command(cmd) => vec![C2S::Command(cmd)],
        Control::Batch(commands) => commands.into_iter().map(C2S::Command).collect(),
        Control::Spawn { state, desc } => vec![C2S::Spawn { state, desc }],
        Control::SubscribeDebug => vec![C2S::SubscribeDebug],
        Control::SpawnAgent {
            body,
            position,
            yaw,
        } => vec![C2S::SpawnAgent {
            body,
            position,
            yaw,
        }],
        Control::Possess { entity } => vec![C2S::Possess { entity }],
        Control::Unpossess => vec![C2S::Unpossess],
        Control::AgentIntent {
            entity,
            move_xz,
            yaw,
            jump,
        } => vec![C2S::AgentIntent {
            entity,
            move_xz,
            yaw,
            jump,
        }],
        Control::Goodbye => vec![C2S::Goodbye],
    }
}

impl ViewerClient {
    /// 在独立 tokio 线程跑连接循环(自带 current_thread runtime),
    /// 返回 (事件接收端, 控制端)。连接失败 = `Err`(启动期错误,主循环
    /// fail-stop 自决);连接中途断连经 `ViewEvent::Disconnected` 呈现。
    ///
    /// 入会序列:Hello(固定 client_id)→ Subscribe{view_distance} → 等
    /// Welcome;之后经共用 `run_raw_pump` 收发原始帧,转换线程映射 `S2C → ViewEvent`,
    /// Pong 吞掉,Bye/EOF/IO 错 → Disconnected 收口(通道随后自然枯竭)。
    pub fn spawn(
        addr: SocketAddr,
        view_distance: u8,
    ) -> io::Result<(std_mpsc::Receiver<ViewEvent>, ControlHandle)> {
        // 整条连接生命周期(连接 + 收发泵)自持一个 current_thread runtime,
        // 建在独立 OS 线程上(不在调用线程建 runtime:测试/宿主进程可能已有
        // tokio 上下文,嵌套 block_on 会 panic;线程边界 = 干净上下文)。
        // 连接失败经 oneshot 回传(启动期 `Err`);成功后事件/控制双通道交接。
        let (addr_tx, addr_rx) = std_mpsc::sync_channel::<io::Result<()>>(1);
        let (event_tx, event_rx) = std_mpsc::sync_channel::<ViewEvent>(EVENT_CAPACITY);
        let (down_tx, down_rx) = std_mpsc::sync_channel::<S2C>(EVENT_CAPACITY);
        let (control_std_tx, control_std_rx) = std_mpsc::sync_channel::<Control>(CONTROL_CAPACITY);
        let (up_tx, up_rx) = tokio::sync::mpsc::channel::<Vec<C2S>>(CONTROL_CAPACITY);

        // 入会首帧先入 up 通道(裸帧不包装,同现网;保序位于所有后续控制帧之前)。
        up_tx
            .try_send(vec![
                C2S::Hello {
                    client_id: HELLO_CLIENT_ID,
                },
                C2S::Subscribe { view_distance },
            ])
            .expect("新建 up 通道容量足够容纳入会首帧");

        std::thread::Builder::new()
            .name("viewer-client".to_owned())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("client runtime 构建失败");
                rt.block_on(async move {
                    match TcpStream::connect(addr).await {
                        // 直连成功:开启 TCP_NODELAY 并回执后进共用原始泵(connect 与泵同
                        // runtime,socket 注册不跨 runtime)。
                        Ok(stream) => {
                            if let Err(err) = stream.set_nodelay(true) {
                                let _ = addr_tx.send(Err(err));
                                return;
                            }
                            let _ = addr_tx.send(Ok(()));
                            drop(addr_tx);
                            run_raw_pump(stream, up_rx, down_tx).await;
                        }
                        Err(err) => {
                            let _ = addr_tx.send(Err(err));
                        }
                    }
                });
            })
            .expect("client 线程 spawn 失败");

        // 等连接回执(失败 = Err 上抛,不产生半死事件通道)。
        addr_rx
            .recv_timeout(CONNECT_TIMEOUT)
            .map_err(|_| io::Error::new(io::ErrorKind::TimedOut, "连接超时"))??;

        // 控制面桥线程:std 通道收 Control → control_to_c2s 翻译 → tokio up 通道。
        std::thread::Builder::new()
            .name("viewer-control-bridge".to_owned())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("控制桥 runtime 构建失败");
                rt.block_on(async {
                    while let Ok(command) = control_std_rx.recv() {
                        let msgs = control_to_c2s(command);
                        if up_tx.send(msgs).await.is_err() {
                            return; // 连接循环已收口
                        }
                    }
                });
            })
            .expect("控制桥线程 spawn 失败");

        // 下行转换线程:down_rx → s2c_to_event → event_tx(满则阻塞,宁慢不丢);
        // down 通道枯竭且未发出过 Disconnected 时补发 Disconnected{"goodbye"}(保现网 :548 语义)。
        std::thread::Builder::new()
            .name("viewer-event-convert".to_owned())
            .spawn(move || {
                let mut sent_disconnected = false;
                while let Ok(s2c) = down_rx.recv() {
                    if let Some(event) = s2c_to_event(s2c) {
                        if matches!(event, ViewEvent::Disconnected { .. }) {
                            sent_disconnected = true;
                        }
                        if event_tx.send(event).is_err() {
                            return;
                        }
                    }
                }
                if !sent_disconnected {
                    let _ = event_tx.send(ViewEvent::Disconnected {
                        reason: "goodbye".to_owned(),
                    });
                }
            })
            .expect("事件转换线程 spawn 失败");

        Ok((event_rx, ControlHandle { tx: control_std_tx }))
    }
}

/// 原始协议收发泵(REQ-014 M01 §2.1;`ViewerClient` 与 `NetTransport` 共用):
/// `select!(biased)` 同等 socket 读 / `up_rx` 上行批 / 3s Ping 心跳;
/// 下行直接输出原始 `S2C`(满则阻塞,宁慢不丢),任何断连路径均发 `S2C::Bye` 后收口。
pub(crate) async fn run_raw_pump<S>(
    mut stream: S,
    mut up_rx: tokio::sync::mpsc::Receiver<Vec<C2S>>,
    down_tx: std_mpsc::SyncSender<S2C>,
) where
    S: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin,
{
    let mut buf: Vec<u8> = Vec::with_capacity(READ_CHUNK * 2);
    let mut chunk = vec![0u8; READ_CHUNK];

    // keepalive:服务器读超时按"任何入站帧"计时,须周期 Ping 自保(3s < 超时 10s)。
    let mut ping = tokio::time::interval(Duration::from_secs(3));
    ping.tick().await; // 吃掉首枚立即到期(同现网)

    loop {
        tokio::select! {
            biased;
            read = stream.read(&mut chunk) => match read {
                Ok(0) | Err(_) => {
                    // EOF/IO 错:对端断(服务器 Bye 通常先到;此处兜底)。
                    let _ = down_tx.send(S2C::Bye {
                        reason: "连接关闭".to_owned(),
                    });
                    return;
                }
                Ok(n) => buf.extend_from_slice(&chunk[..n]),
            },
            up = up_rx.recv() => match up {
                Some(msgs) => {
                    if msgs.is_empty() {
                        continue;
                    }
                    // 单条消息内多帧拼成一个缓冲单次 write_all(保 REQ-010 批原子性)。
                    let mut batch: Vec<u8> = Vec::with_capacity(msgs.len() * 16);
                    let mut has_goodbye = false;
                    for msg in &msgs {
                        if matches!(msg, C2S::Goodbye) {
                            has_goodbye = true;
                        }
                        batch.extend_from_slice(&encode_c2s_frame(msg));
                    }
                    if let Err(err) = write_all(&mut stream, &batch).await {
                        let _ = down_tx.send(S2C::Bye {
                            reason: disconnect_reason(&err),
                        });
                        return;
                    }
                    if has_goodbye {
                        finish_after_goodbye(&mut stream, &mut buf, &mut chunk, &down_tx).await;
                        return;
                    }
                }
                None => {
                    // up 通道关闭:尽力发 Goodbye,再等 Bye / EOF / 2s 超时收口。
                    let frame = encode_c2s_frame(&C2S::Goodbye);
                    let _ = write_all(&mut stream, &frame).await;
                    finish_after_goodbye(&mut stream, &mut buf, &mut chunk, &down_tx).await;
                    return;
                }
            },
            _ = ping.tick() => {
                let frame = encode_c2s_frame(&C2S::Ping);
                if let Err(err) = write_all(&mut stream, &frame).await {
                    let _ = down_tx.send(S2C::Bye {
                        reason: disconnect_reason(&err),
                    });
                    return;
                }
            },
        }

        // 切尽缓冲里的完整帧。
        while let Some(frame) = try_cut_frame(&mut buf) {
            let Some(s2c) = decode_frame(&frame) else {
                // 解码失败:协议错位,断连收口(不猜续帧)。
                let _ = down_tx.send(S2C::Bye {
                    reason: "帧解码失败".to_owned(),
                });
                return;
            };
            // 下行通道满则阻塞等待:宁可慢不丢;接收端亡则直接结束泵。
            if down_tx.send(s2c).is_err() {
                return;
            }
        }
    }
}

/// 发 `C2S::Goodbye` 后的收口读循环:继续读直到收到 `S2C::Bye`、EOF/IO 错或 2s 超时;
/// 若未从对端收到 `Bye` 则合成 `S2C::Bye { reason: "goodbye" }` 后返回。
async fn finish_after_goodbye<S>(
    stream: &mut S,
    buf: &mut Vec<u8>,
    chunk: &mut [u8],
    down_tx: &std_mpsc::SyncSender<S2C>,
) where
    S: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin,
{
    let delivered_bye = tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            while let Some(frame) = try_cut_frame(buf) {
                let Some(s2c) = decode_frame(&frame) else {
                    let _ = down_tx.send(S2C::Bye {
                        reason: "帧解码失败".to_owned(),
                    });
                    return true;
                };
                let is_bye = matches!(s2c, S2C::Bye { .. });
                if down_tx.send(s2c).is_err() {
                    return true;
                }
                if is_bye {
                    return true;
                }
            }
            match stream.read(chunk).await {
                Ok(0) | Err(_) => return false,
                Ok(n) => buf.extend_from_slice(&chunk[..n]),
            }
        }
    })
    .await;

    if !matches!(delivered_bye, Ok(true)) {
        let _ = down_tx.send(S2C::Bye {
            reason: "goodbye".to_owned(),
        });
    }
}

/// 单帧写全(socket 半写由 write_all 兜住)。
async fn write_all<S>(stream: &mut S, frame: &[u8]) -> io::Result<()>
where
    S: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin,
{
    stream.write_all(frame).await?;
    stream.flush().await
}

// ---- 测试清单(任务书 01)----

#[cfg(test)]
mod tests {
    use super::*;
    use messaging::InputSource;
    use std::net::TcpListener;
    use std::time::Duration;
    use types::glam::Quat;
    use types::{EntityId, Tick};

    /// 测试收发超时上限(真 socket 测试防挂死)。
    fn net_timeout() -> Duration {
        Duration::from_secs(5)
    }

    /// 测试清单 `client_frame_roundtrip_against_codec`:构造 S2C 全变体帧,
    /// 经 proto codec 往返(decode_frame(encode_frame(x)) == x)后喂
    /// [`s2c_to_event`](连接循环共用的映射口,单一事实源),逐变体断言
    /// ViewEvent 形状一致;Pong → None(吞掉,不产事件)。
    #[test]
    fn client_frame_roundtrip_against_codec() {
        let mut chunk = voxel::Chunk::air();
        chunk.set(1, 2, 3, types::BlockId(2));
        let cases: Vec<S2C> = vec![
            S2C::Welcome {
                player_id: InputSource(3),
                tick: Tick(42),
            },
            S2C::ChunkData {
                pos: types::ChunkPos::new(-3, 7),
                chunk: chunk.clone(),
            },
            S2C::ChunkUnload {
                pos: types::ChunkPos::new(9, -1),
            },
            S2C::EntityStates {
                tick: Tick(7),
                states: vec![(
                    EntityId::from_parts(1, 2),
                    EntityState {
                        translation: Vec3::new(1.5, 2.5, 3.5),
                        rotation: Quat::IDENTITY,
                        linvel: Vec3::X,
                        angvel: Vec3::ZERO,
                    },
                )],
                removed: vec![EntityId::from_parts(9, 9)],
                full: false,
            },
            S2C::BlockUpdates {
                updates: vec![
                    (types::BlockPos::new(-1, 5, -2), types::BlockId(1)),
                    (types::BlockPos::new(4, 6, 8), types::BlockId::AIR),
                ],
            },
            S2C::Pong,
            S2C::Bye {
                reason: "shutdown".to_owned(),
            },
            S2C::Spawned {
                entity: EntityId::from_parts(0, 3),
            },
            S2C::EntityShapes {
                shapes: vec![(
                    EntityId::from_parts(0, 5),
                    types::CompoundShape::new(vec![
                        types::ShapeCell {
                            offset: [0, 0, 0],
                            block: types::BlockId(1),
                        },
                        types::ShapeCell {
                            offset: [1, 0, 0],
                            block: types::BlockId(1),
                        },
                    ])
                    .expect("2 格梁"),
                )],
            },
            // W04:调试遥测包(协议镜像 → 事件一比一)。
            S2C::DebugStats {
                tick: Tick(120),
                counters: sim_api::proto::DebugCounters {
                    impact_flow_events: 3,
                    impact_flow_breaks: 1,
                    ..sim_api::proto::DebugCounters::default()
                },
                recent: vec![(
                    Tick(118),
                    sim_api::proto::FlowEventWire {
                        impact: true,
                        supply: 9.217,
                        cut: 7.0,
                        broke: true,
                        detached_cells: 1,
                        position: Vec3::new(10.5, 5.5, 8.5),
                    },
                )],
            },
            // A06:主体流(增量 + 全量空包)与附身回执(Some/None)。
            S2C::AgentStates {
                tick: Tick(50),
                states: vec![(
                    EntityId::from_parts(0, 7),
                    types::BodyHash::fnv1a(b"capsule"),
                    vec![EntityState {
                        translation: Vec3::new(1.0, 2.0, 3.0),
                        ..EntityState::default()
                    }],
                )],
                removed: vec![EntityId::from_parts(0, 8)],
                full: false,
            },
            S2C::AgentStates {
                tick: Tick(51),
                states: vec![],
                removed: vec![],
                full: true,
            },
            S2C::Possessed {
                entity: Some(EntityId::from_parts(0, 7)),
            },
            S2C::Possessed { entity: None },
        ];

        for case in cases {
            // 先证 codec 往返本身无损(喂编码帧的前提)。
            let frame = sim_api::proto::encode_frame(&case);
            let decoded = decode_frame(&frame).expect("全变体帧可解码");
            assert_eq!(decoded, case, "codec 往返无损:{case:?}");

            // 映射口:逐变体断言 ViewEvent 形状(协议类型 → 事件类型一比一)。
            match (case, s2c_to_event(decoded)) {
                (
                    S2C::Welcome { player_id, tick },
                    Some(ViewEvent::Welcome {
                        player_id: p,
                        tick: t,
                    }),
                ) => {
                    assert_eq!((p, t), (player_id, tick), "Welcome 字段一致");
                }
                (
                    S2C::ChunkData { pos, chunk: c },
                    Some(ViewEvent::Chunk { pos: p, chunk: rc }),
                ) => {
                    assert_eq!((p, rc), (pos, c), "ChunkData 字段一致");
                }
                (S2C::ChunkUnload { pos }, Some(ViewEvent::ChunkGone { pos: p })) => {
                    assert_eq!(p, pos, "ChunkUnload 字段一致");
                }
                (
                    S2C::EntityStates {
                        tick,
                        states,
                        removed,
                        full,
                    },
                    Some(ViewEvent::Entities {
                        tick: t,
                        states: s,
                        removed: r,
                        full: f,
                    }),
                ) => {
                    assert_eq!(
                        (t, s, r, f),
                        (tick, states, removed, full),
                        "EntityStates 字段一致"
                    );
                }
                (S2C::BlockUpdates { updates }, Some(ViewEvent::Blocks { updates: u })) => {
                    assert_eq!(u, updates, "BlockUpdates 字段一致");
                }
                (S2C::Bye { reason }, Some(ViewEvent::Disconnected { reason: r })) => {
                    assert_eq!(r, reason, "Bye → Disconnected 携带原因");
                }
                (S2C::Spawned { entity }, Some(ViewEvent::Spawned { entity: e })) => {
                    assert_eq!(e, entity, "Spawned 回执字段一致");
                }
                (S2C::EntityShapes { shapes }, Some(ViewEvent::Shapes { shapes: s })) => {
                    assert_eq!(s, shapes, "EntityShapes 字段一致(内容原样)");
                }
                (
                    S2C::DebugStats {
                        tick,
                        counters,
                        recent,
                    },
                    Some(ViewEvent::DebugStats {
                        tick: t,
                        counters: c,
                        recent: r,
                    }),
                ) => {
                    assert_eq!((t, c, r), (tick, counters, recent), "DebugStats 字段一致");
                }
                (
                    S2C::AgentStates {
                        tick,
                        states,
                        removed,
                        full,
                    },
                    Some(ViewEvent::Agents {
                        tick: t,
                        states: s,
                        removed: r,
                        full: f,
                    }),
                ) => {
                    assert_eq!(
                        (t, s, r, f),
                        (tick, states, removed, full),
                        "AgentStates 一致"
                    );
                }
                (S2C::Possessed { entity }, Some(ViewEvent::Possessed { entity: e })) => {
                    assert_eq!(e, entity, "Possessed 字段一致");
                }
                (S2C::Pong, None) => {}
                (original, event) => panic!("变体 {original:?} 映射不符: {event:?}"),
            }
        }
    }

    /// 测试清单 `client_spawn_receives_welcome_and_move_roundtrip`:
    /// [`ViewerClient::spawn`] 对真 TCP(127.0.0.1:0 实际端口)的端到端小回环——
    /// 迷你服务器(同步 socket + proto 对称 codec)收 Hello/Subscribe、发
    /// Welcome;客户端事件通道收到 Welcome(player_id/tick 一致);控制端
    /// MoveObserver 到达服务器侧为同值 MoveObserver 帧;服务器 Bye → 客户端
    /// Disconnected 携带原因。(连接行为非窗口类,属可自动化网络测试。)
    #[test]
    fn client_spawn_receives_welcome_and_move_roundtrip() {
        // 迷你服务器:监听 0 端口(OS 分配),线程内同步收发。
        let listener = TcpListener::bind("127.0.0.1:0").expect("绑定 127.0.0.1:0");
        let addr = listener.local_addr().expect("实际端口");
        let server = std::thread::spawn(move || {
            use std::io::Read;
            let (mut sock, _) = listener.accept().expect("accept");
            // 收两帧(Hello + Subscribe;半帧可能,循环凑)。
            let mut buf = Vec::new();
            let mut incoming = Vec::new();
            while incoming.len() < 2 {
                let mut chunk = [0u8; 1024];
                let n = sock.read(&mut chunk).expect("读入会帧");
                assert!(n > 0, "客户端在入会前断开");
                buf.extend_from_slice(&chunk[..n]);
                while buf.len() >= 4 {
                    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
                    if buf.len() < 4 + len {
                        break;
                    }
                    let frame: Vec<u8> = buf.drain(..4 + len).collect();
                    let c2s = sim_api::proto::decode_c2s_frame(&frame).expect("入会帧可解码");
                    match c2s {
                        C2S::Hello { client_id } => {
                            assert_eq!(client_id, HELLO_CLIENT_ID, "client_id 固定 0");
                            incoming.push(c2s);
                        }
                        C2S::Subscribe { view_distance } => {
                            assert_eq!(view_distance, 4, "视距 = spawn 入参");
                            incoming.push(c2s);
                        }
                        other => panic!("入会期只应有两帧,实为 {other:?}"),
                    }
                }
            }
            // 发 Welcome。
            let welcome = sim_api::proto::encode_frame(&S2C::Welcome {
                player_id: InputSource(2),
                tick: Tick(7),
            });
            use std::io::Write;
            sock.write_all(&welcome).expect("写 Welcome");
            // 收 MoveObserver 帧(控制端 roundtrip)与 SubscribeDebug(W04:
            // 到达顺序不定——两帧都到才收口;SubscribeDebug 恰一帧)。
            let mut move_seen = None;
            let mut debug_subs = 0usize;
            let mut buf = Vec::new();
            while move_seen.is_none() || debug_subs < 1 {
                let mut chunk = [0u8; 1024];
                let n = sock.read(&mut chunk).expect("读 MoveObserver");
                assert!(n > 0, "等 MoveObserver 期间客户端断开");
                buf.extend_from_slice(&chunk[..n]);
                while buf.len() >= 4 {
                    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
                    if buf.len() < 4 + len {
                        break;
                    }
                    let frame: Vec<u8> = buf.drain(..4 + len).collect();
                    match sim_api::proto::decode_c2s_frame(&frame).expect("帧可解码") {
                        C2S::MoveObserver { position } => move_seen = Some(position),
                        C2S::SubscribeDebug => debug_subs += 1,
                        _ => continue,
                    }
                }
            }
            assert_eq!(
                debug_subs, 1,
                "控制端 subscribe_debug → 服务器恰一帧(幂等订阅一次)"
            );
            // 服务器回一包 DebugStats(客户端映射成 ViewEvent::DebugStats 的
            // 收端验证;W03 协议在 e2e 已覆盖,这里只证映射链路)。
            let stats = sim_api::proto::encode_frame(&S2C::DebugStats {
                tick: Tick(9),
                counters: sim_api::proto::DebugCounters::default(),
                recent: Vec::new(),
            });
            sock.write_all(&stats).expect("写 DebugStats");
            // 发 Bye(断连呈现)并关闭。
            let bye = sim_api::proto::encode_frame(&S2C::Bye {
                reason: "server-shutdown".to_owned(),
            });
            let _ = sock.write_all(&bye);
            drop(sock);
            move_seen.expect("控制端 Move 必达服务器")
        });

        // 客户端侧:spawn → Welcome 事件 → 控制 Move → Bye 断连事件。
        let (events, control) = ViewerClient::spawn(addr, 4).expect("连接 127.0.0.1:0 迷你服务器");
        match events
            .recv_timeout(net_timeout())
            .expect("5s 内收到 Welcome 事件")
        {
            ViewEvent::Welcome { player_id, tick } => {
                assert_eq!(player_id, InputSource(2), "Welcome 身份一致");
                assert_eq!(tick, Tick(7), "Welcome 拍号一致");
            }
            other => panic!("首事件应为 Welcome,实为 {other:?}"),
        }
        control.move_observer(Vec3::new(24.0, -58.0, 8.0));
        control.subscribe_debug();
        let moved = server.join().expect("迷你服务器线程不 panic");
        assert_eq!(
            moved,
            Vec3::new(24.0, -58.0, 8.0),
            "控制端 move_observer → 服务器侧同值 MoveObserver 帧"
        );
        // 服务器回的 DebugStats → ViewEvent::DebugStats(W04 映射链路)。
        match events
            .recv_timeout(net_timeout())
            .expect("DebugStats 事件必达")
        {
            ViewEvent::DebugStats {
                tick,
                counters,
                recent,
            } => {
                assert_eq!(tick, Tick(9), "DebugStats 拍号一致");
                assert_eq!(counters, sim_api::proto::DebugCounters::default());
                assert!(recent.is_empty());
            }
            other => panic!("应为 DebugStats,实为 {other:?}"),
        }
        // 服务器 Bye → Disconnected(reason 透传)。
        match events.recv_timeout(net_timeout()).expect("断连事件必达") {
            ViewEvent::Disconnected { reason } => {
                assert_eq!(reason, "server-shutdown", "Bye reason 透传");
            }
            other => panic!("断连后应为 Disconnected,实为 {other:?}"),
        }
    }

    /// M01 §2.3:`control_to_c2s` 纯翻译——Batch 展为多元素 vec,其余单元素 vec。
    #[test]
    fn m01_control_to_c2s_pure() {
        let cmd_a = messaging::SimCommand::SetBlock {
            pos: BlockPos::new(0, -60, 0),
            block: BlockId(1),
        };
        let cmd_b = messaging::SimCommand::SetBlock {
            pos: BlockPos::new(1, -60, 0),
            block: BlockId(2),
        };
        assert_eq!(
            control_to_c2s(Control::Batch(vec![cmd_a.clone(), cmd_b.clone()])),
            vec![C2S::Command(cmd_a), C2S::Command(cmd_b)]
        );
        assert_eq!(
            control_to_c2s(Control::Move(Vec3::new(1.0, 2.0, 3.0))),
            vec![C2S::MoveObserver {
                position: Vec3::new(1.0, 2.0, 3.0),
            }]
        );
        assert_eq!(control_to_c2s(Control::Goodbye), vec![C2S::Goodbye]);
    }

    /// REQ-014 M03 T7(MF 上行批单次 write_all 边界锚):
    /// `up_tx` 发送一条含 3 个 Ping 的批消息,记录器断言恰触发 1 次 `poll_write`,
    /// 且字节与 3 帧顺序拼接逐位相同;关闭 `up_tx` 后泵写出恰 1 条 Goodbye 并正常收口。
    #[tokio::test(start_paused = true)]
    async fn m03_t7_up_batch_single_write() {
        use std::pin::Pin;
        use std::sync::{Arc, Mutex};
        use std::task::{Context, Poll};

        struct WriteRecorder {
            writes: Arc<Mutex<Vec<Vec<u8>>>>,
        }

        impl tokio::io::AsyncWrite for WriteRecorder {
            fn poll_write(
                self: Pin<&mut Self>,
                _cx: &mut Context<'_>,
                buf: &[u8],
            ) -> Poll<io::Result<usize>> {
                self.writes.lock().expect("writes lock").push(buf.to_vec());
                Poll::Ready(Ok(buf.len()))
            }

            fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
                Poll::Ready(Ok(()))
            }

            fn poll_shutdown(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
                Poll::Ready(Ok(()))
            }
        }

        let writes: Arc<Mutex<Vec<Vec<u8>>>> = Arc::new(Mutex::new(Vec::new()));
        let recorder = WriteRecorder {
            writes: writes.clone(),
        };
        let (duplex_read, _duplex_keep_alive) = tokio::io::duplex(4096);
        let io = tokio::io::join(duplex_read, recorder);
        let (up_tx, up_rx) = tokio::sync::mpsc::channel(4);
        let (down_tx, _down_rx) = std_mpsc::sync_channel(16);

        let task = tokio::spawn(run_raw_pump(io, up_rx, down_tx));

        up_tx
            .send(vec![C2S::Ping, C2S::Ping, C2S::Ping])
            .await
            .expect("发送三帧批消息");

        let mut seen = false;
        for _ in 0..500 {
            if !writes.lock().expect("writes lock").is_empty() {
                seen = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        assert!(seen, "上行批未写出");
        tokio::task::yield_now().await;

        let snapshot = writes.lock().expect("writes lock").clone();
        assert_eq!(
            snapshot.len(),
            1,
            "锚 A: 一条消息三帧必须恰 1 次 write_all,实为 {} 次",
            snapshot.len()
        );
        let expected: Vec<u8> = [C2S::Ping, C2S::Ping, C2S::Ping]
            .iter()
            .flat_map(encode_c2s_frame)
            .collect();
        assert_eq!(
            snapshot[0], expected,
            "锚 B: 单次写入字节必须与三帧 Ping 顺序拼接逐位相同"
        );

        drop(up_tx);
        task.await.expect("run_raw_pump 正常收口");
        drop(_duplex_keep_alive);

        let all_writes = writes.lock().expect("writes lock");
        let goodbye = encode_c2s_frame(&C2S::Goodbye);
        let goodbye_count = all_writes.iter().filter(|w| **w == goodbye).count();
        assert_eq!(goodbye_count, 1, "收尾 Goodbye 帧应恰 1 条");
    }
}
