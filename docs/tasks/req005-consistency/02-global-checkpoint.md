# 任务 02:全局快照、观察者回放与断点续跑

## 目标

兑现 REQ-005 §4.2 / D7/D8/D9:把 checkpoint 从"每分片快照"升级为
**WorldSnapshot = 分片快照 + 调度器状态**,并让网络观察者命令进入命令日志。
完成后:

- checkpoint 落在"待迁移信封 + 多观察者 + 生命周期计时"的任意 tick,
  恢复 + 重放到终态仍位级一致;实体不丢;票/despawn 计时不被重置;
- 网络 `MoveObserver`/`RemoveObserver` 可回放;断线不会留下幽灵观察者;
- `bootstrap` 对旧 checkpoint 明确报错,不静默降级。

## 必读

- `docs/requirements/REQ-005-consistency.md` §4.2、AC-C5/C6
- `docs/requirements/REQ-005-decisions.md` D7/D8/D9
- `crates/sim-api/src/snapshot.rs`、`replay.rs`、`session.rs`、`log.rs`
- `crates/shard/src/scheduler.rs`(字段清单/register/tick)、`router.rs`、
  `lifecycle.rs`
- `crates/messaging/src/envelope.rs`、`command.rs`
- `crates/app/src/driver.rs`(`CheckpointBundle`/`write_checkpoint`/
  `bootstrap`/`ObserveOrchestrator::consume_net_up`)
- `crates/e2e/tests/world_loading.rs`(AC-G5 回放风格)、
  `crates/sim-api/tests/long_replay.rs`

## 授权改动

- `crates/messaging/src/command.rs`:`SimCommand::RemoveObserver`(追加,
  ordinal 7);`apply_command` 里 panic(观察者命令永不进分片前缀);
- `crates/messaging/src/envelope.rs`:`CrossShardQueue::snapshot()/restore()`
  (或等价);`Envelope` 已 serde,必要时补 `Eq`(测试用);
- `crates/shard/src/lifecycle.rs`:`LifecycleSnapshot` + snapshot/restore;
  `Stage` serde;
- `crates/shard/src/lib.rs`:re-export `SchedulerSnapshot`/`RouterSnapshot`/
  `LifecycleSnapshot`(sim-api 消费);
- `crates/shard/src/router.rs`:`RouterSnapshot { envelopes, sequences }` +
  snapshot/restore;inboxes 不入快照;
- `crates/shard/src/scheduler.rs`:`SchedulerSnapshot` + snapshot_state/
  restore_state;观察者收件箱处理 `RemoveObserver`;rebuild holders;
- `crates/sim-api/src/snapshot.rs`:`WorldSnapshot`、`SchedulerSnapshot`;
  `GlobalSnapshot` 保持不变;
- `crates/sim-api/src/session.rs`:`world_snapshot`、`move_observer_from`、
  `remove_observer_from`、`send_from`、`send_broadcast_from`;
- `crates/sim-api/src/replay.rs`:`replay_world`(恢复 scheduler + shards +
  观察者命令路由);`replay` 保留给既有 GlobalSnapshot 测试;
- `crates/app/src/driver.rs`:`CheckpointBundle { format_version, world,
  log_bytes }`;bootstrap 用 `replay_world`;网络 Move/Remove/Send 入日志;
- `crates/app/src/config.rs`:若需要,加 checkpoint 版本常量/错误文案;
- 测试文件按需更新(授权)。

## 接口设计(签名固定)

```rust
// messaging
pub enum SimCommand {
    // ...既有变体...
    /// 观察者移除(source 由 StampedInput.source 隐含;不走分片前缀)。
    RemoveObserver,
}

// shard::lifecycle
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifecycleSnapshot {
    pub stage: Stage,
    pub last_activity: Tick,
    pub drained: bool,
}
impl ShardLifecycle {
    pub fn snapshot(&self) -> LifecycleSnapshot;
    pub fn restore(snap: LifecycleSnapshot, cfg: LifecycleConfig) -> Self;
}

// shard::router
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouterSnapshot {
    pub envelopes: Vec<Envelope>,
    pub sequences: BTreeMap<ShardId, u64>,
}
impl Router {
    pub fn snapshot(&self) -> RouterSnapshot;
    pub fn restore(&mut self, snap: RouterSnapshot);
}

// shard::scheduler
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SchedulerSnapshot {
    pub tick: Tick,
    pub observers: BTreeMap<InputSource, Vec3>,
    pub observers_ever_set: bool,
    pub router: RouterSnapshot,
    pub lifecycles: BTreeMap<ShardId, LifecycleSnapshot>,
}
impl Scheduler {
    pub fn snapshot_state(&self) -> SchedulerSnapshot;
    pub fn restore_state(&mut self, snap: SchedulerSnapshot);
}

// sim-api
#[derive(Debug, Clone, PartialEq)]
pub struct WorldSnapshot {
    pub shards: GlobalSnapshot,
    pub scheduler: SchedulerSnapshot,
}
impl Session {
    pub fn world_snapshot(&self) -> WorldSnapshot;
    pub fn move_observer_from(
        &mut self,
        source: InputSource,
        position: Vec3,
    ) -> Result<LogEntry, SessionError>;
    pub fn remove_observer_from(
        &mut self,
        source: InputSource,
    ) -> Result<LogEntry, SessionError>;
    pub fn send_from(
        &mut self,
        source: InputSource,
        shard: ShardId,
        command: SimCommand,
    ) -> Result<LogEntry, SessionError>;
    pub fn send_broadcast_from(
        &mut self,
        source: InputSource,
        owner: ShardId,
        command: SimCommand,
    ) -> Result<Vec<LogEntry>, SessionError>;
}
pub fn replay_world(
    world: &WorldSnapshot,
    log: &CommandLog,
    cfg: SessionConfig,
    new_registry: impl Fn() -> Box<dyn BlockRegistry>,
) -> Result<Session, SessionError>;

// app
pub const CHECKPOINT_FORMAT_VERSION: u32 = 1;
#[derive(Serialize, Deserialize)]
pub struct CheckpointBundle {
    pub format_version: u32,
    pub world: WorldSnapshot,
    pub log_bytes: Vec<u8>,
}
```

## 语义细则

1. **SchedulerSnapshot 内容**:
   - `observers`/`observers_ever_set`/`router`/`lifecycles` 四项;`tick` 必须
     等于 `GlobalSnapshot.tick`;
   - `lifecycles` 只存当前 `sims` 键集;`unload_shard` 留下的 `Stopped`
     记录是观测残留,不入快照(恢复时按已注册分片重建 Active 记录);
   - `holders` 不入快照;`restore_state` 里从每个 `ShardSim` 的
     `voxels.chunk_positions()` 重建(调用任务 00 的 holder 表重建);
   - `observer_inbox`/`router.inboxes`/`activation_requests`/
     `despawn_requests`/`solidified` 不入快照:
     - 观察者未来命令由日志回放;
     - 分片未来命令由日志回放;
     - 其余是每拍派生物,下一拍重算。
2. **Router 快照**:
   - `CrossShardQueue::snapshot()` 返回全部信封(按 `(destination, apply_at,
   payload ordinal, source, sequence)` 排序;桶内已 canonical,汇总再排序);
   - `restore` 按容量与 `apply_at == produced_at + 1` 校验后放回桶;
   - `sequences` 原样恢复;不恢复会改变后续 migration 的 source_sequence,
     破坏 canonical 序。
3. **WorldSnapshot**:
   - `Session::world_snapshot` 在**两拍之间**调用;`scheduler.snapshot_state()`
     + `global_snapshot()`;断言 tick 相等;
   - `CheckpointBundle` 原子写 `format_version + world + log_bytes`;
   - `format_version` 不匹配 → `AppError::Decode("unsupported checkpoint
     format ...")`;不要尝试兼容旧格式(见 D8)。
4. **观察者命令**:
   - `SimCommand::RemoveObserver` 追加在最后(ordinal 7);客户端白名单拒绝
     (`translate`/`validate_external_command`);
   - `move_observer_from`/`remove_observer_from` 盖章 `source`/`sequence`,
     提交调度器观察者收件箱,返回 `LogEntry`;`move_observer`/`remove_observer`
     保留为"以会话 source 直呼"的测试/装配入口;
   - `Scheduler::tick` 的观察者收件箱处理两种命令:
     - `SetObserverPosition` → 插入 `(source, position)`;
     - `RemoveObserver` → 删除 source(不存在 = 幂等,不 panic);
   - `replay_world` 对两种观察者命令走 `submit_observer_raw`;其它走
     `submit_raw(dest)`。
5. **网络驱动**:
   - `NetUp::DriverCommand` 的 `MoveObserver` → `session.move_observer_from
     (player, position)`;`log.append(entry)`;删除 `player_sequences` /
     手工 `StampedInput` 构造;
   - `Left` → `session.remove_observer_from(player)`;`log.append(entry)`;
     幂等移除;
   - `Send` → `session.send_from(player, shard, command)`;SetBlock 走
     `send_broadcast_from(player, owner, command)` 并 append 全部 entries;
   - 日志 source 现在是真实玩家;回放的 canonical 序与观察者来源一致。
6. **恢复顺序**:
   - `replay_world`: `Scheduler::new(start_tick = world.shards.tick)` →
     逐片 `register_snapshot` → `restore_state(world.scheduler)` →
     重建 holders → 回放日志;
   - `restore_state` 校验 tick、lifecycle keys、信封目的地;不满足 →
     `SessionError` 或 panic(fail-stop,内部状态)。
7. **重放窗口(修正)**:
   - checkpoint 的 `snapshot.tick` 是"下一待执行拍"(`tick_now()`);因此
     `entry.tick >= snapshot.tick` 的条目都要回放,`entry.tick <
     snapshot.tick` 的已进快照;
   - 现有 `replay`/`replay_world` 的 `entry.tick > snapshot.tick` 会丢掉
     "checkpoint 后、下一拍前到达"的命令;本任务统一改为 `>=`,并更新
     `replay_skips_entries_at_or_before_snapshot_tick` 测试语义;
   - `replay` 旧函数保留,但过滤规则与 `replay_world` 一致;`bootstrap` 用
     `replay_world`。

## 测试清单(先红后绿)

| 测试名 | 断言意图 |
| --- | --- |
| `scheduler_snapshot_roundtrip` | 观察者/信封/sequences/lifecycle 快照→恢复逐字段相等 |
| `world_snapshot_tick_consistency` | `world.shards.tick == world.scheduler.tick`;不等 → panic/Err |
| `checkpoint_pending_migration_entity_survives` | tick N 末有迁移信封时快照;replay_world 到 N+2 实体在目的片 |
| `checkpoint_command_at_snapshot_tick_replayed` | `entry.tick == snapshot.tick` 的命令在 checkpoint 后、下一拍前到达;重放必须应用,不丢 |
| `replay_skips_only_entries_before_snapshot_tick` | `< snapshot.tick` 跳过;`==` 与 `>` 都回放(替换旧 `..._at_or_before` 断言) |
| `checkpoint_restores_observers_without_move` | 快照后恢复,不再发 MoveObserver;observers() 非空且激活继续 |
| `checkpoint_restores_lifecycle_timers` | 空闲计时跨 checkpoint 不重置;despawn 候选拍不变 |
| `network_move_observer_logged_and_replayed` | `move_observer_from` 产生日志;replay_world 后观察者表一致 |
| `network_remove_observer_logged_and_replayed` | 断线移除入日志;重放后无幽灵;票撤 |
| `send_from_logs_player_source` | 网络 Send 的 LogEntry.source = 玩家,不是会话 1 |
| `checkpoint_bytes_deterministic` | 同状态两次 world_snapshot 字节相等 |
| `old_checkpoint_version_rejected` | 旧/未知 format_version → 明确 Err,不静默 |
| `replay_world_600_ticks_matches` | 含迁移/多观察者/断线的 600 拍场景,重放窗逐拍采样位级一致 |

## 完成定义

- 门禁:`cargo test -p messaging -p shard -p sim-api -p app --all-targets` +
  `cargo clippy -p messaging -p shard -p sim-api -p app --all-targets
  -- -D warnings` 退出码 0;
- 既有 `replay`/`long_replay`/AC-G5 测试零回归(旧函数保留);
- 不实现事件广播/block write(任务 03);不实现实体所有权(D12)。

## 风险与备注

- **checkpoint 原子性**:`world_snapshot` 与 `log.take()` 必须在同一拍边界
  完成;当前驱动在 tick 分支末尾写,commands 分支不会并发进来(单线程
  select),但仍要保证 `world_snapshot` 在 `log.take` 之前/之后一致。建议
  先取 world,再 `log.take`;两者之间不能有 `session.send`。
- **旧测试的 `CheckpointBundle` 构造**:`driver.rs` 内部测试有手工构造;
  统一加 `format_version` 常量,更新测试。
- **观察者日志膨胀**:网络 MoveObserver 节流在 viewer 侧(≥100ms),但恶意
  客户端可高频发送;本任务只保证可回放,限流是 REQ-006/网络加固的范围。
  上报 `LogEntry` 数量指标,便于后续评估。
- **RemoveObserver 的 dest**:观察者命令不看 dest;`LogEntry.dest` 用玩家
  最后已知分片或 `WorldId(0)/DimensionId(0)/ShardPos(0,0)` 哨兵,doc 注明。
