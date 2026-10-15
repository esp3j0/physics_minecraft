# 任务 00:app crate 骨架(lib + bin 拆分)

## 目标

app 现为纯 bin(只打印版本)。拆出 lib 承载逻辑,建立编译通过的模块骨架,
不写任何逻辑与测试。

## 必读

- `docs/design/codebase-layout.md` §3 app、§6(配置表)
- `docs/tasks/app/README.md` 决策 1
- `crates/sim-api/Cargo.toml`(依赖声明风格)

## 交付物

1. `crates/app/Cargo.toml`:

   ```toml
   [dependencies]
   types = { path = "../types" }
   messaging = { path = "../messaging" }
   voxel = { path = "../voxel" }
   physics-sim = { path = "../physics-sim" }
   shard = { path = "../shard" }
   storage = { path = "../storage" }
   sim-api = { path = "../sim-api" }
   tokio = { version = "1", features = ["macros", "rt", "time", "sync", "signal"] }
   serde = { version = "1", features = ["derive"] }
   toml = "1"
   bincode = { version = "2", features = ["serde"] }
   thiserror = "2"

   [dev-dependencies]
   tempfile = "3"
   tokio = { version = "1", features = ["test-util"] }

   [lints]
   workspace = true
   ```

   (dev 的 tokio 条目只为并入 `test-util` feature,cargo 会合并。)

2. `crates/app/src/lib.rs`:保留现有 `//!` 文档(从 main.rs 迁来,职责句不变),
   追加 `pub mod config; pub mod driver; pub mod telemetry;`

3. 新建 3 个模块文件,各只含一行 `//!` 文档注释(照蓝图 §3 对应行):

   - `src/config.rs` —— 全部参数(§6)+ toml 可覆写;唯一默认值来源
   - `src/driver.rs` —— biased select(关机>tick>命令)、MissedTickBehavior::Skip、
     checkpoint 节奏与断点续跑装配
   - `src/telemetry.rs` —— tick 耗时 P95>阈值×60tick 告警;只度量不决策

4. `crates/app/src/main.rs`:改为一行 `fn main()` 调用占位(`println!` 保留版本行
   即可,装配在任务 03 落地)。

## 完成定义

- `cargo test -p app --all-targets` 绿(0 个测试);
- `cargo clippy -p app --all-targets -- -D warnings` 绿;
- 不改动本任务书未列出的任何文件。
