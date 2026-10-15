# 任务 00:sim-api crate 骨架

## 目标

crate `sim-api` 目前只有 lib.rs 文档注释。建立编译通过的模块骨架:依赖声明 +
4 个模块文件(仅文档注释),不写任何逻辑与测试。

## 必读

- `docs/design/codebase-layout.md` §2(依赖表,注意 README 决策 1 的修订)与 §3 sim-api 图纸
- `crates/sim-api/src/lib.rs`(现有文档注释,保留)
- `crates/storage/Cargo.toml`(依赖声明风格参照)

## 交付物

1. `crates/sim-api/Cargo.toml`:

   ```toml
   [dependencies]
   types = { path = "../types" }
   messaging = { path = "../messaging" }
   shard = { path = "../shard" }
   storage = { path = "../storage" }
   physics-sim = { path = "../physics-sim" }
   voxel = { path = "../voxel" }
   serde = { version = "1", features = ["derive"] }
   bincode = { version = "2", features = ["serde"] }
   thiserror = "2"

   [dev-dependencies]
   tempfile = "3"

   [lints]
   workspace = true
   ```

2. `crates/sim-api/src/lib.rs`:保留现有 `//!` 文档(顺带把依赖边界一句按 README
   决策 1 修订:允许 physics-sim/voxel 的用途如上),追加:

   ```rust
   pub mod log;
   pub mod replay;
   pub mod session;
   pub mod snapshot;
   ```

3. 新建 4 个文件,各只含一行 `//!` 文档注释(照蓝图 §3 对应行):

   - `src/session.rs` —— Session:命令进/快照出的 headless 缝(接口形状=网络形状)
   - `src/snapshot.rs` —— GlobalSnapshot:tick + 每分片不透明字节
   - `src/log.rs` —— 外部命令日志:按 tick 分段(bincode);信封/事件/迁移不落日志
   - `src/replay.rs` —— restore checkpoint + 重放命令日志;终态位级一致(AC8)

## 完成定义

- `cargo test -p sim-api --all-targets` 绿(0 个测试);
- `cargo clippy -p sim-api --all-targets -- -D warnings` 绿;
- 不改动本任务书未列出的任何文件。
