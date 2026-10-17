# 任务 01:连接层(app::net)

## 目标

REQ-003 §4.2:TCP 监听、每连接一 task、流式帧切分、会话状态机、翻译层、
keepalive——到本任务结束,客户端能连入、被分配玩家身份、移动观察者、
被健康检查(还没有 chunk 流与观察输出,属任务 02/03)。

## 必读

- `docs/requirements/REQ-003-network.md` §4.1/§4.2、§5(与 v1/v2 差异表)
- `docs/tasks/req003-net/README.md` 决策 2/4/5
- `crates/sim-api/src/proto.rs`(任务 00 产出)
- `crates/app/src/driver.rs`(DriverCommand 通道;MoveObserver 已有)

## 交付物

`crates/app/src/net/mod.rs`(或 net.rs 单文件 + 子模块,自选)+ lib.rs 导出,
含:监听器、连接 task、流式帧 codec、会话状态机、翻译层、keepalive、
出站两级队列骨架(填充逻辑在任务 03,队列与断连策略本任务落)。

## 接口设计(要点固定)

```rust
/// 网络配置(Config 增量):listen_addr("127.0.0.1:0" 默认,测试用 0 端口)、
/// read_timeout_secs(10)、keepalive 间隔(2s)。
pub struct NetConfig { pub listen_addr: String, pub read_timeout_secs: u64, ... }

/// 网络事件(网络 → 驱动;有界 mpsc):
pub enum NetUp {
    Joined { player: InputSource },           // Hello+Subscribe 完成后
    Left { player: InputSource, reason: ... }, // Goodbye/超时/Critical 丢弃/协议违规
    DriverCommand { player: InputSource, cmd: DriverCommand }, // 翻译层产物
}
/// 驱动 → 网络:每玩家出站句柄(入两级队列);玩家身份分配 = InputSource 递增
/// (从 1 起;0 保留)。身份分配在网络层原子完成(JoinHandle 表)。

/// 翻译层(纯函数,单元测试主战场):
pub fn translate(player: InputSource, c2s: C2S) -> TranslateOutcome;
pub enum TranslateOutcome {
    Move(DriverCommand),           // MoveObserver
    Send(DriverCommand),           // Command 白名单 → DriverCommand::Send{shard,command}
                                   // shard = 命令作用位置所在分片(SetBlock→pos;Spawn/Detonate→中心)
    Ping, Subscribe(u8), Hello(..), Goodbye, Reject(&'static str), // 非白名单命令等
}

/// 启动监听(spawn accept 循环 + 每连接 task);返回实际绑定地址(测试取端口)。
pub async fn serve(cfg: NetConfig, up_tx: mpsc::Sender<NetUp>,
                   shutdown: oneshot/broadcast) -> io::Result<SocketAddr>;
```

## 语义细则

1. 会话状态机:TCP 连上 → 收 Hello → (分配身份) → 收 Subscribe(钳制 ≤32)
   → Joined 上报;未 Subscribe 前其它 C2S = 协议违规断连;
2. 帧切分:读循环凑满 4B 长度 → 长度 > FRAME_MAX 断连 → 读满载荷 → decode;
3. keepalive:读超时窗口(config)内无任何入站帧 → 断连;Ping → Pong(Critical);
4. 出站队列:Critical 64 / World 512(决策 6);Critical 满或 send 失败 → 断连
   并上报 Left;World 满丢最旧(本任务只需队列与丢弃策略正确,消息源在 03);
5. Goodbye/断连 → Left 上报(驱动侧观察者清理由任务 03 接线,本任务只上报);
6. 翻译层零 IO、零时钟——纯函数。

## 测试清单(内嵌,全走 127.0.0.1:0 + 内嵌迷你客户端)

| 测试名 | 断言意图 |
| --- | --- |
| `hello_subscribe_joins_with_player_id` | 两客户端先后连入 → 身份 1、2 递增;Joined 上报 |
| `subscribe_clamps_view_distance` | Subscribe{200} → 钳 32(状态机内部断言或经后续流验证) |
| `move_observer_reaches_driver_channel` | MoveObserver → NetUp::DriverCommand{MoveObserver} |
| `command_whitelist_enforced` | 白名单命令透传;SetObserverPosition 命令 → Reject+断连 |
| `oversized_frame_disconnects` | 发 >2MiB 长度前缀 → 断连 + Left 上报 |
| `keepalive_timeout_disconnects` | read_timeout 调小 → 静默客户端被断;Ping 保持在线 |
| `frame_streaming_splits_partial_reads` | 半帧/TCP 粘包(发端分段写)→ 正确切帧 |
| `critical_overflow_disconnects` | 灌满 Critical(不读 socket 的客户端)→ 断连 |
| `translate_pure_function_table` | 纯函数:每个 C2S 变体 → 期望 outcome(表驱动) |

## 完成定义

见 [README 公共完成定义](README.md)。
