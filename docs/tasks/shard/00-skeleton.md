# 任务 00:shard crate 骨架

## 目标

crate `shard` 目前只有 lib.rs 文档注释。建立编译通过的模块骨架:依赖声明 +
5 个模块文件(仅文档注释),不写任何逻辑与测试。

## 必读

- `docs/design/codebase-layout.md` §2(依赖表)与 §3 shard 图纸
- `crates/shard/src/lib.rs`(现有文档注释,保留)
- `crates/physics-sim/Cargo.toml`(依赖声明风格参照)

## 交付物

1. `crates/shard/Cargo.toml`:在 `[lints]` 前添加

   ```toml
   [dependencies]
   types = { path = "../types" }
   messaging = { path = "../messaging" }
   physics-sim = { path = "../physics-sim" }
   ```

   不加 serde/thiserror 等其它依赖(后续任务按需加)。

2. `crates/shard/src/lib.rs`:保留现有 `//!` 文档,追加模块声明:

   ```rust
   pub mod coordinator;
   pub mod lifecycle;
   pub mod ownership;
   pub mod router;
   pub mod scheduler;
   ```

3. 新建 5 个文件,各只含一行 `//!` 文档注释(内容照蓝图 §3 对应行,写清模块职责):

   - `src/coordinator.rs` —— TickCoordinator:逻辑 tick 单调 +1、不看墙钟、lag 只上报
   - `src/ownership.rs` —— OwnershipMap:claim/release/owner,重复 claim fail-stop
   - `src/lifecycle.rs` —— 生命周期:Active→Draining→Stopped、懒创建、空闲 despawn 判定
   - `src/router.rs` —— 命令路由:有界收件箱、满则断开权威输入;信封盖章与分发
   - `src/scheduler.rs` —— 固定 worker 池:capacity-one 通道、每拍全员屏障、所有权转移

## 完成定义

- `cargo test -p shard --all-targets` 绿(0 个测试);
- `cargo clippy -p shard --all-targets -- -D warnings` 绿(空模块不得引 clippy 告警,
  例如多余的 `pub use`);
- 不改动本任务书未列出的任何文件。
