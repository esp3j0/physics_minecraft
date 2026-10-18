# 任务 00:app main 网络装配接线

## 目标

REQ-003 交付的 `app::net::serve`/`OutboundHub`/`ObserveLink` 在 main.rs 尚未接线
(`observe: None` 占位)。本任务让**服务器启动即监听**,外部客户端可连——viewer 的前置。

## 必读

- `crates/app/src/driver.rs`(ObserveLink 结构与 consume_net_up;以源码为准)
- `crates/app/src/net.rs`(serve 签名/NetConfig/NetUp)
- `crates/app/tests/app_e2e.rs` 与 `crates/e2e/tests/network.rs`(装配与客户端测试风格)
- `docs/tasks/req004a-viewer/README.md` 决策 6

## 交付物

`crates/app/src/config.rs`(listen_addr 字段)+ `main.rs`(接线)+ 内嵌/集成测试。

## 接口设计(要点固定)

```rust
// Config 增:pub listen_addr: String(默认 "127.0.0.1:25565",#[serde(default)] 风格同其它字段)
// main.rs:
// 1. NetConfig::from(config)(listen_addr + 默认超时);
// 2. 构造 NetUp 通道 + OutboundHub(照 net 模块公开面与 ObserveLink 形状接线);
// 3. serve(...) 取实际绑定地址并打印一行"监听 {addr}";
// 4. DriverArgs.observe = Some(真实链接);其余装配不变。
// 监听失败 = fail-stop(expect,启动期错误)。
```

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `config_listen_addr_default_and_override` | 默认值 + toml 覆写 |
| `main_wiring_smoke`(集成测试或抽装配函数测) | 装配出的 ObserveLink 非空;serve 起后外部测试客户端连接收 Welcome(复用 net 测试的迷你客户端风格;若 main 装配不可从测试触达,抽 `pub fn assemble_network(...) -> (ObserveLink, SocketAddr, 关停句柄)` 供 main 与测试共用,授权) |

## 完成定义

见 [README](README.md);门禁 `-p app`;既有 301 测试零回归。
