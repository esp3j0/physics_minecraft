# 任务 03:会话缝与回放器

## 目标

`session.rs`:headless 缝——命令进、tick 推进、快照出(接口形状=未来网络形状,
共识 3);`replay.rs`:从全局快照 + 命令日志重建会话(AC8 的执行侧)。

## 必读

- `crates/shard/src/scheduler.rs`(register/submit/tick/snapshot_shard/tick_now/
  SchedulerConfig.start_tick——全部已有)
- `crates/physics-sim/src/sim.rs` 的 snapshot/restore(任务 01 产出)
- `crates/sim-api/src/log.rs`、`snapshot.rs`(任务 02 产出)
- `docs/tasks/sim-api/README.md` 决策 5/6/8

## 交付物

`crates/sim-api/src/session.rs` 与 `crates/sim-api/src/replay.rs`,各含类型 + 内嵌测试。

## 接口设计(签名固定)

`session.rs`:

```rust
#[derive(Debug, Clone)]
pub struct SessionConfig {
    pub scheduler: SchedulerConfig,
    /// 会话作为输入源(默认 InputSource(1);0 保留给调度器,决策 5)。
    pub source: InputSource,
}
// Default:scheduler 默认 + InputSource(1)

#[derive(Debug, thiserror::Error)]
pub enum SessionError {
    #[error("路由失败:{0}")]
    Route(#[from] RouteError),
    #[error("快照恢复失败:{0}")]
    Snapshot(#[from] SimSnapshotError),
}

/// headless 会话:命令进/快照出(蓝图 §3 session;网络形状后置 REQ-003)。
pub struct Session {
    scheduler: Scheduler,
    source: InputSource,
    next_sequence: u64,
}

impl Session {
    pub fn new(cfg: SessionConfig) -> Self;

    /// 直接注册现成分片(装配/测试用)。
    pub fn register(&mut self, sim: ShardSim);

    /// 从分片快照恢复并注册(README 决策 1:registry 由调用方注入)。
    pub fn register_snapshot(
        &mut self,
        bytes: &[u8],
        registry: Box<dyn BlockRegistry>,
    ) -> Result<ShardId, SessionError>;

    /// 命令进入缝:盖 apply_at = tick_now()(下一次 tick() 生效)、
    /// source = 本会话、sequence 单调;返回 LogEntry 供调用方记录(决策 5)。
    pub fn send(&mut self, shard: ShardId, command: SimCommand) -> Result<LogEntry, SessionError>;

    /// 原章输入直投(回放专用;不得用于普通路径,决策 8)。
    pub fn submit_raw(&mut self, input: StampedInput) -> Result<(), SessionError>;

    pub fn tick(&mut self) -> TickSummary;
    pub fn tick_now(&self) -> Tick;
    pub fn shard_ids(&self) -> Vec<ShardId>;
    pub fn entity_state(&self, shard: ShardId, entity: EntityId) -> Option<EntityState>;

    /// 全局快照(两拍之间;tick + 各分片字节,ShardId 升序)。
    pub fn global_snapshot(&self) -> GlobalSnapshot;

    pub fn shutdown(self);
}
```

`replay.rs`:

```rust
/// 从全局快照 + 命令日志重建会话(AC8 执行侧)。
/// - start_tick 对齐 snapshot.tick(决策 6);
/// - 只重放 tick > snapshot.tick 的条目(≤ 的已烙进快照),按日志序原章直投;
/// - registry 经工厂逐分片注入(不序列化,任务 01 决策)。
/// 返回的 Session 已注册全部分片并投递完命令;调用方 tick 到目标拍后比对终态。
pub fn replay(
    snapshot: &GlobalSnapshot,
    log: &CommandLog,
    cfg: SessionConfig,
    new_registry: impl Fn() -> Box<dyn BlockRegistry>,
) -> Result<Session, SessionError>;
```

## 语义细则

1. `send` 的 LogEntry:tick = apply_at(= tick_now),dest/source/sequence 照章;
2. `replay` 会 **覆盖** cfg.scheduler.start_tick = snapshot.tick(以快照为准);
3. `replay` 重放顺序 = 日志条目序(append 序);不重新盖章、不改 sequence(决策 8);
4. `global_snapshot` 在两拍之间调用(doc 注明);`per_shard` 经
   `Scheduler::snapshot_shard` 逐分片取(升序);
5. Session/shutdown 不吞错误:Route/Snapshot 透传。

## 测试清单(内嵌)

| 测试名 | 断言意图 |
| --- | --- |
| `send_stamps_apply_at_current_tick` | send 后 tick_now 为 N → LogEntry.tick == N;下一次 tick() 后实体已出现 |
| `send_sequence_is_monotonic` | 同 session 连续 send:sequence 0,1,2;不同 source 互不影响(config 换 source) |
| `register_snapshot_restores_shard` | 快照 → register_snapshot → entity_state 可查、tick 可跑 |
| `global_snapshot_contains_all_shards_sorted` | 两分片 → per_shard 键升序、tick 正确 |
| `replay_skips_entries_at_or_before_snapshot_tick` | 快照拍 20;日志含 tick 10/20/21 → 只重放 21(观察:重放后 send 的 next_sequence 不受干扰、tick_now == 20) |
| `replay_restores_from_start_tick` | replay 后 tick_now == snapshot.tick;再 tick() 的 summary.tick == snapshot.tick |

(端到端位级一致由任务 04 验收,本任务不做全链路。)

## 门禁包

`-p sim-api`。

## 完成定义

见 [README 公共完成定义](README.md)。
