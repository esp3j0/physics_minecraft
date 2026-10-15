# app crate 实现任务书(总览)

- 状态:已定稿(架构师签发)
- 日期:2026-10-16
- 上游:[REQ-001 总体架构](../../requirements/REQ-001-architecture.md)、[代码布局蓝图 §3 app / §6 配置表](../../design/codebase-layout.md)
- 实现方式:与前两轮相同——glm-5.3-flash 写核心(01-03),qwen 写骨架/集成测试(00/04)
  + 独立只读评审;cargo 门禁;架构师终审。**这是 REQ-001 主体最后一轮。**

## 任务顺序(依赖序,串行执行)

| # | 任务书 | 模块 | 实现模型 | 内容 |
| --- | --- | --- | --- | --- |
| 0 | [00-skeleton.md](00-skeleton.md) | 全 crate | qwen | lib+bin 拆分、依赖、模块骨架 |
| 1 | [01-config.md](01-config.md) | config.rs | glm | §6 全参数表 + toml 部分覆写 + 子配置换算 |
| 2 | [02-telemetry.md](02-telemetry.md) | telemetry.rs | glm | tick 耗时 P95 滑窗告警(只度量不决策) |
| 3 | [03-driver.md](03-driver.md) | driver.rs + main.rs | glm | biased select 驱动、checkpoint、断点续跑装配 |
| 4 | [04-integration.md](04-integration.md) | tests/ | qwen | 真库断点续跑端到端、toml 文件覆写 |

## 每份任务书的公共完成定义

1. 严格 TDD:先写【测试清单】全部测试 + 最小 `todo!()` 桩,`cargo test -p app` 见红
   (编译通过、测试失败),再实现到绿;
2. 门禁(机器判定,不可绕过):
   - `cargo test -p app --all-targets` 退出码 0;
   - `cargo clippy -p app --all-targets -- -D warnings` 退出码 0;
3. 只改任务书【交付物】与【授权改动】列出的文件。

## 架构师在本轮做出的具体决策(实现时不得偏离)

1. **lib + bin 拆分**:app 现为纯 bin;改为 `src/lib.rs`(`pub mod config/driver/
   telemetry`)承载全部逻辑(集成测试可测),`main.rs` 只做装配与打印;
2. **tokio 驱动**:`tokio::time::interval`(周期 = `config.dt`)+
   `MissedTickBehavior::Skip` + `tokio::select! { biased; 关机 > tick > 命令 }`
   (照 v2 driver);**tokio interval 首枚 tick 立即到期——构造后先 `tick().await`
   吃掉再进 select**,否则首拍连跳;
3. **`session.tick()` 为阻塞调用,直接在 async loop 内执行**(毫秒级、单装配单
   驱动;REQ-003 网络落地时再 spawn_blocking 化,注释留 TODO);
4. **CheckpointBundle = bincode(`{ snapshot: GlobalSnapshot, log_bytes: Vec<u8> }`)**:
   快照拍 + 此后命令日志原子一体存 `CheckpointStore`;checkpoint 时 `log.take()`
   重置(日志只存增量);崩溃丢失窗口 = `checkpoint_interval_ticks`(§9.11 语义);
5. **flush 只在关停**:checkpoint 时仅 `append_checkpoint`(fire-and-forget);
   退出路径 = 终末 checkpoint → `append` → `flush()` → **必须 `recv()` ack**
   (worker 的 ack 通道不允许被弃收——弃收 = worker panic,按其设计)→
   `session.shutdown()` → `store.shutdown()`;
6. **断点续跑(bootstrap)**:先开 chunks/checkpoints 两个 redb 库,
   `read_latest` 检查点(**在移交给 StoreWorker 之前读**——移交后文件锁在手),
   有则解码 bundle → `sim_api::replay` 恢复 Session+log(续跑拍号对齐),
   无则全新 Session;chunks 库为将来世界加载预留(本轮只开不用);
7. **ctrl_c 作为关机源**(`tokio::signal::ctrl_c`,Windows 支持);测试可关
   (`DriverArgs.listen_ctrl_c = false`),否则暂停时间测试会被信号干扰;
8. **telemetry 计时**:std `Instant`(局部 `#[allow(clippy::disallowed_methods)]`
   + 理由注释——clippy.toml 顶部注释已授权"计时只属于 app::telemetry");
   P95 滑窗 60 tick,`> shard_alert_ms` 报警;只记录/上报,不影响驱动决策;
9. **config 全字段 `#[serde(default)]`**(toml 文件可只写要覆写的字段),
   `Default` = 蓝图 §6 表;app 级装配参数一并入表:`worker_count=4`、
   `data_dir="data"`、`inbox_capacity=256`(shard README 决策 1);
   `despawn_after_ticks = round(despawn_after_secs / dt)`;
10. **驱动测试用 `#[tokio::test(start_paused = true)]`**(tokio test-util,
    暂停时间自动推进 interval,确定性);telemetry 用合成 Duration 喂,不依赖时钟。
