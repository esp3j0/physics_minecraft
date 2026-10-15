# 任务 05:固定 worker 池调度器(本 crate 核心)

## 目标

蓝图 §3:"固定 worker 池(OS 线程,capacity-one 通道,每 tick 全员汇合屏障,分片
所有权转移;空片跳过但收输入必恢复)"。这是全项目**唯一允许创建线程的地方**。

核心机制(AC5 的结构保证):`ShardSim` 实例本身在 worker 间**按拍转移所有权**——
调度器把 sim 连同本拍输入前缀一起发进 worker 的 capacity-one 通道,worker 处理完
把 sim 连同 outbox 发回;调度器收齐全部结果才推进下一拍(全屏障)。任一时刻一个
分片只在一个线程手里。

## 必读

- `crates/physics-sim/src/sim.rs`(`ShardSim` 全部公开 API:`new`/`shard`/
  `apply_prefix`/`step`/`take_outbox`/`entity_state`/`entity_count`/`load_chunk`;
  `Outbox { migrations, dirty_chunks, solidified }`)
- `crates/physics-sim/tests/shard_sim.rs`(现有用法示例)
- `crates/shard/src/` 已完成的 coordinator/ownership/lifecycle/router(直接复用)
- `docs/research/ferrumc-v2-survey.md`(worker 池/屏障/轮转的调研结论,含文件行号)
- `references/ferrumc-v2/` 对应源码(按调研文档指路读,仅作参照)
- `docs/design/codebase-layout.md` §3 scheduler 行、§4(确定性纪律)
- `docs/tasks/shard/README.md` 决策 4/5/6/7

## 授权改动(超出 shard crate 的部分,仅此一处)

`crates/voxel/src/block.rs`:给 `BlockRegistry` trait 增加 `Send` supertrait
(`pub trait BlockRegistry: Send { ... }`)。这是让 `ShardSim`(内含
`Box<dyn BlockRegistry>`)能跨线程转移的最小改动;`SimpleRegistry` 纯数据自动满足,
voxel 现有测试必须保持全绿。

## 交付物

1. `crates/shard/src/scheduler.rs`:类型 + 内嵌单元测试;
2. `crates/shard/tests/scheduler_basic.rs`:crate 级行为测试(见测试清单下半);
3. `crates/shard/src/lib.rs`:re-export 公开入口
   `pub use scheduler::{Scheduler, SchedulerConfig, TickSummary};`(其余模块的
   re-export 保持各任务已定状态)。

## 接口设计(签名固定)

```rust
#[derive(Debug, Clone)]
pub struct SchedulerConfig {
    /// worker 数,≥ 1。测试建议 2。
    pub worker_count: usize,
    pub router: RouterConfig,
    /// 传给各分片 lifecycle(本轮只接线,不自动 despawn;README 决策 4)。
    pub despawn_after_ticks: u64,
}
// Default:{ worker_count: 4, router: Default::default(), despawn_after_ticks: 1800 }

/// 一拍的执行摘要。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TickSummary {
    /// 本拍执行的拍号(tick() 第 k 次调用返回 k-1,README 决策 7)。
    pub tick: Tick,
    /// 本拍 step 的分片数。
    pub stepped_shards: usize,
    /// 本拍成功入队的迁移信封数。
    pub migrations_admitted: usize,
}

pub struct Scheduler { /* 私有字段自定 */ }

impl Scheduler {
    /// 起 worker 线程池。worker_count == 0 → panic(配置错误)。
    pub fn new(cfg: SchedulerConfig) -> Self;

    /// 注册一个分片(以构造好的 ShardSim 形式进入)。已注册 → panic;
    /// 同时 ownership.claim + lifecycle.activate(懒创建语义由注册方掌握)。
    pub fn register(&mut self, sim: ShardSim);

    /// 本地权威命令入箱(经 Router)。未注册分片 → panic(README 决策 2);
    /// apply_at < 当前 tick → panic(迟到输入,README 决策 6);
    /// 收件箱满 → Err(InboxFull)(反压,调用方断开权威输入)。
    pub fn submit(&mut self, shard: ShardId, input: StampedInput) -> Result<(), RouteError>;

    /// 执行一拍(全屏障,流程见下),返回摘要。
    pub fn tick(&mut self) -> TickSummary;

    /// 查询:未注册或实体不在该分片 → None。
    pub fn entity_state(&self, shard: ShardId, entity: EntityId) -> Option<EntityState>;

    pub fn shard_ids(&self) -> Vec<ShardId>; // ShardId 升序
    pub fn shard_count(&self) -> usize;
    pub fn tick_now(&self) -> Tick;

    /// 关停:释放全部 sim,join 全部 worker(Draining)。幂等。
    pub fn shutdown(mut self);
}
// Drop 同样 join(用 Option::take 保证幂等),防线程泄漏。
```

## tick() 流程(严格按序,fail-stop)

1. `let at = coordinator.tick();`(本拍号)
2. 对每个已注册分片(ShardId 升序):`prefix = router.dispatch_prefix(shard, at)`
3. **静态轮转分配**(WorkerSlot,照 v2 scheduler.rs 的 round-robin):维护
   `next_slot: usize`,每分配一个分片后 `next_slot = (next_slot + 1) % worker_count`;
   同一 worker 本拍分到的多个分片打包成**一个**任务消息;
4. 经每 worker 的 **capacity-one** 通道(`std::sync::mpsc::sync_channel::<WorkerJob>(1)`)
   发送任务;worker 处理完经结果通道回传;
5. **屏障**:收齐全部已派发 worker 的结果;结果收进 `BTreeMap<ShardId, (ShardSim,
   Outbox)>`——回传完成顺序不定,BTreeMap 归一(确定性);
6. sim 放回自有表;按 ShardId 升序处理各 outbox:`router.collect_intents(shard,
   &outbox.migrations, at)`——`AdmitError::QueueFull` 时 panic(fail-stop,注释留
   TODO:反压策略);其余 AdmitError = 编程错误 panic;
7. lifecycle 接线:本拍有输入前缀非空或 `entity_count() > 0` 的分片
   `note_activity(at)`(空片不记,空闲计时才走得动);
8. `coordinator.advance();` 返回 `TickSummary { tick: at, stepped_shards,
   migrations_admitted }`。

**worker 线程函数**:循环 `recv()` 任务;对任务里每个 `(shard, mut sim, prefix)`:
`sim.apply_prefix(&prefix); sim.step(); let outbox = sim.take_outbox();` 回传
`Vec<(ShardId, ShardSim, Outbox)>`;`recv` 返回 Err(通道关闭)= 正常退出。
内部通道 send/recv 失败 = 一致性违规 → panic(fail-stop)。

**确定性要点**:worker 完成顺序不影响结果(第 5/6 步按序归一);worker 内不读
墙钟/不随机;分片处理顺序 = ShardId 升序。

## 测试清单

内嵌 `src/scheduler.rs`(纯逻辑部分):

| 测试名 | 断言意图 |
| --- | --- |
| `config_zero_workers_panics` | `#[should_panic]` worker_count=0 |
| `tick_summary_reports_stepped_shards` | 注册 2 片 tick 一次:stepped_shards==2,tick==起始拍 |

`tests/scheduler_basic.rs`(跨线程行为):

| 测试名 | 断言意图 |
| --- | --- |
| `tick_is_monotonic_and_barriered` | 连续 3 次 tick:tick_now 从 0 → 3,summary.tick == 0,1,2 |
| `register_twice_panics` | `#[should_panic]` |
| `submit_to_unknown_shard_panics` | `#[should_panic]` |
| `submit_stale_input_panics` | `#[should_panic]` apply_at < 当前 tick |
| `entity_spawned_via_command_is_steppable` | submit SpawnDynamic(apply_at=下一拍)→ tick 若干次 → entity_state 有值且位置在动 |
| `shutdown_joins_all_workers` | tick 后 shutdown 正常返回、进程退出无挂起 |

测试构造 sim 参照 `crates/physics-sim/tests/shard_sim.rs` 的帮助函数风格
(`ShardSimConfig::new(shard)` + `Box::new(SimpleRegistry…)`;solidify 可关)。

## 完成定义

见 [README 公共完成定义](README.md);此外 voxel 既有测试必须保持全绿(Send 改动)。
