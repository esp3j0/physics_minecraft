# 任务 03:驱动 loop 与装配(driver.rs + main.rs)

## 目标

蓝图 §3 driver:biased select(关机>tick>命令)、MissedTickBehavior::Skip、
checkpoint 节奏、断点续跑装配;§3 main:Config→各 crate→驱动 loop。
这是把 storage/sim-api/shard 全部缝起来的最终装配。

## 必读

- `docs/tasks/app/README.md` 决策 2-7(全部核心决策在这)
- `crates/sim-api/src/session.rs` / `replay.rs` / `log.rs` / `snapshot.rs`
- `crates/storage/src/worker.rs`(flush ack 必须被 recv!)/ `redb.rs` / `traits.rs`
- `crates/shard/tests/two_shards.rs`(构造分片的帮助函数风格)
- `docs/research/ferrumc-v2-survey.md` driver 部分(biased select 原型)
- `crates/app/src/config.rs`(任务 01 产出)

## 交付物

`crates/app/src/driver.rs`(类型 + 内嵌测试)与 `crates/app/src/main.rs`(装配)。

## 接口设计(签名固定)

```rust
/// 驱动指令(外部缝;REQ-003 网络将复用同一形状)。
#[derive(Debug)]
pub enum DriverCommand {
    Send { shard: ShardId, command: SimCommand },
    Shutdown,
}

/// 驱动装配(全显式,无全局单例)。
pub struct DriverArgs {
    pub config: Config,
    pub session: Session,
    pub store: StoreWorker,
    pub log: CommandLog,
    pub commands: tokio::sync::mpsc::Receiver<DriverCommand>,
    /// 是否监听 ctrl_c 为关机源(测试置 false,README 决策 7)。
    pub listen_ctrl_c: bool,
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
}

/// 检查点捆绑:快照拍 + 此后命令日志,原子一体(README 决策 4)。
#[derive(Serialize, Deserialize)]
pub struct CheckpointBundle {
    pub snapshot: sim_api::GlobalSnapshot,
    pub log_bytes: Vec<u8>,
}

#[derive(Debug, thiserror::Error)]
pub enum AppError {
    #[error("配置错误:{0}")]
    Config(#[from] ConfigError),
    #[error("存储错误:{0}")]
    Store(#[from] storage::traits::StoreError),
    #[error("会话错误:{0}")]
    Session(#[from] sim_api::SessionError),
    #[error("检查点解码失败:{0}")]
    Decode(String),
}

/// 驱动主循环(biased:关机 > tick > 命令;决策 2/3/5)。
/// 正常与关机路径都执行:终末 checkpoint → append → flush → recv ack →
/// session.shutdown → store.shutdown;返回 outcome。
pub async fn run(args: DriverArgs) -> DriverOutcome;

/// 装配:开库 → 查最新检查点(read_latest 在移交 worker 前,决策 6)→
/// 有则 replay 恢复(Session + log 续跑),无则全新;返回驱动所需全部件。
pub fn bootstrap(
    config: &Config,
    new_registry: impl Fn() -> Box<dyn voxel::BlockRegistry>,
) -> Result<
    (Session, CommandLog, StoreWorker, Option<types::Tick> /* 续跑起始拍 */),
    AppError,
>;
```

`main.rs`:读命令行第一个参数为配置路径(无则 None)→ `Config::load` →
`bootstrap` → 起 mpsc 通道 + `tokio::spawn(run)` → 等待完成 → 打印 outcome
(一行 tick/检查点/告警摘要)。registry 用 `SimpleRegistry::builtin`。

## 语义细则(逐条照做)

1. **interval**:周期 `Duration::from_secs_f32(config.dt)`,
   `set_missed_tick_behavior(MissedTickBehavior::Skip)`,构造后**先 `tick().await`
   吃掉立即到期的首枚**再进 select(决策 2);
2. **select 分支序(biased)**:ctrl_c(仅 `listen_ctrl_c`)→ `commands.recv()`
   → `interval.tick()`;命令分支:`Send` → `session.send` + `log.append`
   (InboxFull → `commands_rejected += 1`,只记录);`Shutdown`/通道关闭 → 退出;
3. **tick 分支**:计时(t0=Instant → elapsed,喂 telemetry)→ `session.tick()`
   → `ticks_run += 1`;**checkpoint 节奏**:`ticks_run % checkpoint_interval_ticks
   == 0` 时:bundle = {global_snapshot(), log.take() 的 to_bytes} →
   `store.append_checkpoint(tick, bincode(bundle))`(fire-and-forget,决策 5);
4. **退出路径**(任何关机源):若 `log` 非空或自上次 checkpoint 后有 tick →
   终末 checkpoint(同上)→ `flush()` → **`recv()` ack**(阻塞可接受,决策 3
   同 rationale)→ `session.shutdown()` → `store.shutdown()`;
5. **bootstrap**:chunks 库 = `data_dir/chunks.redb`、checkpoints 库 =
   `data_dir/checkpoints.redb`(目录不存在则 `fs::create_dir_all`);
   checkpoints `read_latest` → Some 则 `sim_api::replay(&snapshot, &log_from_bytes,
   config.session_config(), new_registry)`(决策 6);None 则
   `Session::new(config.session_config())` + 空 log;
   最后 `StoreWorker::new(chunks, checkpoints)`(注意:read 在移交前);
6. telemetry 的 `threshold_ms`/窗口来自 config(§9.2:5ms × 60 tick);
7. 驱动内除 cargo 外不运行外部命令;不用 HashMap(被 deny)。

## 测试清单(内嵌 driver.rs,tokio 暂停时间)

| 测试名 | 断言意图 |
| --- | --- |
| `run_executes_ticks_and_commands` | start_paused;3 拍 + 拍间注入 Send(SpawnDynamic)→ outcome.ticks_run==3,实体出现 |
| `shutdown_command_stops_loop` | 发 Shutdown → run 返回,outcome 合法 |
| `checkpoints_at_interval_and_exit` | interval=5,跑 12 拍 → checkpoints == [5, 10, 12](12 = 终末);store 用 mem 后端 |
| `rejected_command_counted_not_fatal` | 小 inbox(1)灌 2 条同拍命令 → rejected==1,循环仍活 |
| `interval_skip_does_not_burst` | (暂停时间下)跑 20 拍 tick 计数恰 20,无连跳(首枚已吃) |
| `checkpoint_bundle_roundtrip` | bundle 编解码往返相等;空拍照常出 checkpoint(0 分片快照合法) |

bootstrap 的真库续跑验收归任务 04(跨进程文件锁语义在集成侧更自然)。

## 完成定义

见 [README 公共完成定义](README.md)。
