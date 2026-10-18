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
const EVENT_CAPACITY: usize = 4096;
/// 控制通道容量(移动节流后不会积满;满即弃,latest-wins 语义)。
const CONTROL_CAPACITY: usize = 64;
/// 连接握手等待上限(spawn 同步返回;超时 = Err)。
const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);
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
    /// 断连(对端 Bye/EOF/IO 失败;reason = 尽力还原的说明)。
    Disconnected { reason: String },
}

/// 控制端(主循环 → 连接循环):观察者移动与主动离场。
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Control {
    /// 观察者位姿(渲染节流后送达;连接循环原样发 `C2S::MoveObserver`)。
    Move(Vec3),
    /// 白名单命令(生成/引爆等;连接循环发 `C2S::Command`,翻译层在服务器侧)。
    /// ACV4 ③④ 的交互入口(评审后补,Refs #6)。
    Command(messaging::SimCommand),
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

    /// 发白名单命令(生成/引爆;ACV4 ③④ 交互入口)。
    pub fn command(&self, command: messaging::SimCommand) {
        let _ = self.tx.try_send(Control::Command(command));
    }

    /// 主动离场(尽力而为;连接亡故时静默)。
    pub fn goodbye(&self) {
        let _ = self.tx.try_send(Control::Goodbye);
    }
}

/// 协议客户端:S2C 帧 → [`ViewEvent`] 泵 + C2S 控制通道的独立线程封装。
pub struct ViewerClient;

/// Hello 的 client_id(任务书接口设计:固定 0——viewer 非权威输入,
/// 不需要唯一身份;服务器分配的 InputSource 才是观察者身份)。
const HELLO_CLIENT_ID: u64 = 0;

/// 读循环的 socket 缓冲块大小(4 KiB,app::net 迷你客户端同量级)。
const READ_CHUNK: usize = 4096;

/// 断连原因 → 事件说明文本(尽力还原;EOF/IO 失败同路)。
fn disconnect_reason(err: &io::Error) -> String {
    format!("连接失败:{err}")
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
        S2C::Bye { reason } => Some(ViewEvent::Disconnected { reason }),
        S2C::Pong => None,
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

impl ViewerClient {
    /// 在独立 tokio 线程跑连接循环(自带 current_thread runtime),
    /// 返回 (事件接收端, 控制端)。连接失败 = `Err`(启动期错误,主循环
    /// fail-stop 自决);连接中途断连经 `ViewEvent::Disconnected` 呈现。
    ///
    /// 入会序列:Hello(固定 client_id)→ Subscribe{view_distance} → 等
    /// Welcome;之后 Pump 帧 → 事件通道,Pong 吞掉,Bye/EOF/IO 错 →
    /// Disconnected 收口(通道随后自然枯竭,主循环以 Disconnected 判终)。
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
        // 控制面:渲染侧 std 通道(try_send 恒不阻塞)+ 桥线程搬进 tokio 通道
        // (连接循环 select! 同时等 socket 读与控制命令;轮询式 try_recv 会被
        // 阻塞 read 饿死——挂死教训,见 ControlHandle 注释)。
        let (control_std_tx, control_std_rx) = std_mpsc::sync_channel::<Control>(CONTROL_CAPACITY);
        let (control_tx, control_rx) = tokio::sync::mpsc::channel::<Control>(CONTROL_CAPACITY);
        std::thread::Builder::new()
            .name("viewer-control-bridge".to_owned())
            .spawn(move || {
                // 阻塞 recv 搬运;连接亡故(tokio 接收端 drop)→ 桥线程收口。
                // 搬进 tokio 通道需 async send → 自持微型 runtime(独立线程,
                // 同 spawn 主线程的隔离理由)。
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("控制桥 runtime 构建失败");
                rt.block_on(async {
                    while let Ok(command) = control_std_rx.recv() {
                        if control_tx.send(command).await.is_err() {
                            return; // 连接循环已收口
                        }
                    }
                });
            })
            .expect("控制桥线程 spawn 失败");

        std::thread::Builder::new()
            .name("viewer-client".to_owned())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("client runtime 构建失败");
                rt.block_on(async move {
                    match TcpStream::connect(addr).await {
                        // 直连成功:回执后进泵(connect 与泵同 runtime,socket
                        // 注册不跨 runtime——tokio TcpStream 的mio 注册绑定
                        // 建立 runtime,换 runtime 会得到「context shutdown」错)。
                        Ok(stream) => {
                            let _ = addr_tx.send(Ok(()));
                            drop(addr_tx);
                            run_connection(stream, view_distance, event_tx, control_rx).await;
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
        Ok((event_rx, ControlHandle { tx: control_std_tx }))
    }
}

/// 连接循环体(独立线程的 current_thread runtime 上跑):入会 → select 泵。
/// 任何断连路径:先发 Disconnected 事件,再收口(通道随后自然死亡)。
async fn run_connection(
    mut stream: TcpStream,
    view_distance: u8,
    event_tx: std_mpsc::SyncSender<ViewEvent>,
    mut control_rx: tokio::sync::mpsc::Receiver<Control>,
) {
    let mut buf: Vec<u8> = Vec::with_capacity(READ_CHUNK * 2);
    let mut chunk = vec![0u8; READ_CHUNK];

    // 入会:Hello → Subscribe → 等 Welcome 帧(状态机:未 Subscribe 不进世界,
    // 首帧必 Welcome;超时语义交由服务器 keepalive,此处尽力读写)。
    let hello = encode_c2s_frame(&C2S::Hello {
        client_id: HELLO_CLIENT_ID,
    });
    let subscribe = encode_c2s_frame(&C2S::Subscribe { view_distance });
    if let Err(err) = write_all_frames(&mut stream, &[&hello, &subscribe]).await {
        let _ = event_tx.send(ViewEvent::Disconnected {
            reason: disconnect_reason(&err),
        });
        return;
    }

    // 读侧主循环:socket 读与控制命令同一 select(控制面不会被阻塞 read 饿死;
    // biased 下 socket 读优先——数据流是主生产物)。
    // keepalive:服务器读超时按"任何入站帧"计时——观察者解耦/静止时无其它
    // 流量,须周期 Ping 自保(评审发现 3,架构师修复;间隔 3s < 超时 10s)。
    let mut ping = tokio::time::interval(std::time::Duration::from_secs(3));
    ping.tick().await; // 吃掉首枚立即到期(照 driver 的 interval 处理)
    // 控制臂开关:recv 对已关闭通道恒立即可返,继续轮询会 100% CPU 空转
    // (评审发现 4,架构师修复)——None 后停询本臂。
    let mut control_open = true;
    'read: loop {
        tokio::select! {
            biased;
            read = stream.read(&mut chunk) => match read {
                Ok(0) | Err(_) => {
                    // EOF/IO 错:对端断(服务器 Bye 通常先到;此处兜底)。
                    let _ = event_tx.send(ViewEvent::Disconnected {
                        reason: "连接关闭".to_owned(),
                    });
                    return;
                }
                Ok(n) => buf.extend_from_slice(&chunk[..n]),
            },
            command = control_rx.recv(), if control_open => match command {
                Some(Control::Move(position)) => {
                    let frame = encode_c2s_frame(&C2S::MoveObserver { position });
                    if write_all(&mut stream, &frame).await.is_err() {
                        break 'read;
                    }
                }
                Some(Control::Command(command)) => {
                    let frame = encode_c2s_frame(&C2S::Command(command));
                    if write_all(&mut stream, &frame).await.is_err() {
                        break 'read;
                    }
                }
                Some(Control::Goodbye) => {
                    let frame = encode_c2s_frame(&C2S::Goodbye);
                    let _ = write_all(&mut stream, &frame).await; // 尽力;随后读 EOF
                    break 'read;
                }
                None => {
                    // 控制桥亡故(主循环弃控制端):不主动断(窗口关闭即进程
                    // 退),停询本臂,读至断连呈现。
                    control_open = false;
                }
            },
            _ = ping.tick() => {
                let frame = encode_c2s_frame(&C2S::Ping);
                if write_all(&mut stream, &frame).await.is_err() {
                    break 'read;
                }
            },
        }

        // 切尽缓冲里的完整帧。
        while let Some(frame) = try_cut_frame(&mut buf) {
            let Some(s2c) = decode_frame(&frame) else {
                // 解码失败:协议错位,断连收口(不猜续帧)。
                let _ = event_tx.send(ViewEvent::Disconnected {
                    reason: "帧解码失败".to_owned(),
                });
                return;
            };
            if let Some(event) = s2c_to_event(s2c) {
                // 事件通道满(主循环卡死)则阻塞等待:宁可慢不发丢;
                // 接收端死亡(窗口关)→ 结束连接循环。
                if event_tx.send(event).is_err() {
                    return;
                }
            }
        }
    }

    // 主动离场路径:等服务器 Bye/EOF 短窗后收口(读循环已退;尽力收尾)。
    let _ = event_tx.send(ViewEvent::Disconnected {
        reason: "goodbye".to_owned(),
    });
}

/// 顺序写多帧(入会两连写)。
async fn write_all_frames(stream: &mut TcpStream, frames: &[&[u8]]) -> io::Result<()> {
    for frame in frames {
        write_all(stream, frame).await?;
    }
    Ok(())
}

/// 单帧写全(socket 半写由 write_all 兜住)。
async fn write_all(stream: &mut TcpStream, frame: &[u8]) -> io::Result<()> {
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
            // 收 MoveObserver 帧(控制端 roundtrip)。
            let mut move_seen = None;
            let mut buf = Vec::new();
            while move_seen.is_none() {
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
                        _ => continue,
                    }
                }
            }
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
        let moved = server.join().expect("迷你服务器线程不 panic");
        assert_eq!(
            moved,
            Vec3::new(24.0, -58.0, 8.0),
            "控制端 move_observer → 服务器侧同值 MoveObserver 帧"
        );
        // 服务器 Bye → Disconnected(reason 透传)。
        match events.recv_timeout(net_timeout()).expect("断连事件必达") {
            ViewEvent::Disconnected { reason } => {
                assert_eq!(reason, "server-shutdown", "Bye reason 透传");
            }
            other => panic!("断连后应为 Disconnected,实为 {other:?}"),
        }
    }
}
