# 任务 03:跨片事件广播与涌现 block write

## 目标

兑现 REQ-005 §4.3 / D10/D11(事件部分):把"爆炸只在爆心分片生效"升级为
真正的跨片广播;把凝固桥/未来破坏产生的**涌现 block write** 通过信封
转发到 owner + holders。完成后:

- 边界 `Detonate` 经 `Session::detonate` / 网络路径覆盖所有相关分片,
  合并结果与单世界位级一致;
- 涌现 `CrossEvent` 经 `EnvelopePayload::Event` 在 `apply_at = N+1` 广播,
  每个目标恰好一次;
- 凝固桥在边界产生的 block write 经 `EnvelopePayload::BlockWrite` 转发,
  owner 持久化、各副本一致;
- 回放:外部 Detonate 按日志逐片重投;涌现事件由重放过程确定性再生
  (不落日志)。

## 必读

- `docs/requirements/REQ-005-consistency.md` §4.3、AC-C7
- `docs/requirements/REQ-005-decisions.md` D10/D11
- `crates/messaging/src/event.rs`、`envelope.rs`、`ordering.rs`
- `crates/shard/src/scheduler.rs`、`router.rs`、`chunks.rs`(任务 00)
- `crates/physics-sim/src/sim.rs`(`apply_prefix`/`detonate`/`Outbox`)
- `crates/sim-api/src/session.rs`、`replay.rs`
- `crates/app/src/net.rs`(`translate`/`command_shard`)、`driver.rs`
  (`consume_net_up` 的 Send 分支、`validate_external_command`)
- `crates/e2e/tests/explosion_equivalence.rs`(手工两片广播的对照)

## 授权改动

- `crates/messaging/src/envelope.rs`:`EnvelopePayload::Event(CrossEvent)`、
  `BlockWrite { pos, block }`(追加;ordinal 续最后);
- `crates/physics-sim/src/sim.rs`:`Outbox.block_updates` 扩为
  `BlockWrite { pos, block, kind }`(任务 01 引入;本任务补 `Emergent` 分支);
  `apply_prefix` 处理 `EnvelopePayload::Event/BlockWrite`;凝固桥写标
  `Emergent`;测试;
- `crates/shard/src/scheduler.rs`:`shards_covering`、`broadcast_event`、
  tick 末把 `Outbox.events`/`block_updates(Emergent)` 转信封;测试;
- `crates/shard/src/router.rs`:复用 `collect_intents`;必要时加
  `pending_destinations`(任务 00 已有);
- `crates/sim-api/src/session.rs`:`detonate`/`detonate_from`(逐目标分片
  复制命令 + 日志);测试;
- `crates/app/src/driver.rs`:网络 Detonate 走 `session.detonate_from`;
  `validate_external_command` 对 Detonate 做有限值/半径>0 校验;
- `crates/app/src/net.rs`:仅当 Detonate 需要新的 DriverCommand/校验时改;
  不改协议枚举;
- 测试文件按需更新。

## 接口设计(签名固定)

```rust
// messaging
pub enum EnvelopePayload {
    MigrateEntity { entity: EntityId, state: EntityState, desc: EntityDesc },
    /// 涌现事件:共享参数,各片本地求值;apply_at = N+1。
    Event(CrossEvent),
    /// 涌现 block write:owner + holders 应用;apply_at = N+1。
    BlockWrite { pos: BlockPos, block: BlockId },
}

// physics-sim
pub enum BlockWriteKind { External, Emergent }
pub struct BlockWrite {
    pub pos: BlockPos,
    pub block: BlockId,
    pub kind: BlockWriteKind,
}
pub struct Outbox {
    // ...
    pub block_updates: Vec<BlockWrite>,
    /// 未来涌现事件生产口(本任务只保留;测试可直接调 Scheduler::broadcast_event)。
    pub events: Vec<CrossEvent>,
}

// shard
impl Scheduler {
    /// 核心区或 holder 与事件 AABB(center ± radius)相交的已注册分片,
    /// ShardId 升序。
    pub fn shards_covering(&self, center: Vec3, radius: f32) -> BTreeSet<ShardId>;

    /// 涌现事件广播(tick N 末/内调用):对目标分片逐个 stamp 信封,
    /// `apply_at = produced_at + 1`;origin 自身跳过。
    /// 调度器在 `tick()` 收 outbox 时传 `produced_at = at`;测试可传
    /// `tick_now()`(此时 apply_at = 下一拍)。
    pub fn broadcast_event(
        &mut self,
        origin: ShardId,
        event: CrossEvent,
        produced_at: Tick,
    ) -> Result<(), AdmitError>;
}

// sim-api
impl Session {
    /// 外部爆炸:目标 = shards_covering;每个目标一条 Detonate 命令 +
    /// 一条 LogEntry(同一 apply_at/source/sequence);整批原子。
    pub fn detonate_from(
        &mut self,
        source: InputSource,
        center: Vec3,
        power: f32,
        radius: f32,
    ) -> Result<Vec<LogEntry>, SessionError>;
    pub fn detonate(
        &mut self,
        center: Vec3,
        power: f32,
        radius: f32,
    ) -> Result<Vec<LogEntry>, SessionError>;
}
```

## 语义细则

1. **外部 Detonate(同拍,日志)**:
   - `Session::detonate_from` 计算目标分片,构造同一 `StampedInput`
     (source/sequence/apply_at 相同),`scheduler.submit_batch` 原子投递;
     每个目标一条 `LogEntry`(dest = 该分片);
   - 目标分片 = `shards_covering(center, radius)`;没有目标 → 返回空 Vec
     (不报错;例如爆心在未注册区域);
   - 回放按日志逐条 `submit_raw(dest)` 重投;不引入新的日志格式;
   - 网络 `Send { command: Detonate }` 在驱动 `consume_net_up` 里**先于**
     `validate_external_command` 的分片检查识别:调用 `detonate_from(player,
     ...)`;`validate_external_command` 对 Detonate 只做参数校验
     (`power.is_finite() && radius.is_finite() && radius > 0.0`),不要求
     "爆心分片已注册"(目标集合由 broadcast 计算,中心分片可能是空的);
     其它命令仍走原分片门卫;
2. **涌现事件(下一拍,不落日志)**:
   - `Scheduler::tick()` 处理 outbox 时(`at` = 本拍)对 `Outbox.events` 逐个
     调 `broadcast_event(origin, event, at)`;
   - `broadcast_event(origin, event, produced_at)`:
     - `targets = shards_covering(event.center, event.radius) - {origin}`;
     - 每个目标 `Intent { destination, payload: Event(event) }`,
       经 `Router::collect_intents(origin, ..., produced_at)` 盖章;
       `apply_at = produced_at + 1`;
     - 队列满 → `Err(AdmitError::QueueFull)`;调用方按 fail-stop/降载策略
       处理(本轮 panic,记录待办);
     - `origin` 已在 step 内本地求值,不给自己发;
   - `ShardSim::apply_prefix` 收到 `EnvelopePayload::Event(e)` → 本地
     `detonate`(复用同一纯函数);冻结体仍免疫(§9.12);
   - 事件恰好一次:一个目标一个信封;`take_batch` 取走即清;canonical 序由
     envelope 的 `(payload.ordinal, source, source_sequence)` 保证;
   - `Outbox.events` 是未来"撞击触发爆炸"的生产口;本任务不实现触发逻辑,
     只留字段 + 用 `Scheduler::broadcast_event` 的合成测试。
3. **涌现 block write(下一拍,不落日志)**:
   - 任务 01 已把外部 SetBlock 标 `BlockWriteKind::External`;本任务把凝固桥
     写入标 `Emergent`(未来破坏同);
   - 调度器 tick 末:
     - `External`:只发布 owner 的写入(`take_block_updates`),不做信封;
     - `Emergent`:发布产生者的写入;对 `owner + holders - {producer}` 逐个
       stamp `EnvelopePayload::BlockWrite`(apply_at N+1);owner 收到后应用
       并持久化,其他 holder 应用以保持副本一致;
   - `ShardSim::apply_prefix` 收到 `EnvelopePayload::BlockWrite` → 先断言
     目标 chunk 已加载(否则 panic,内部一致性违规),再直接写体素 + 重建
     collider,**不再推 `outbox.block_updates`**(防广播回环);
   - owner 必须已是该 chunk 的 holder;任务 00 的 ensure/区域规则保证;
     不满足 → panic(内部一致性违规);
   - 网络 `BlockUpdates` 在产生者发布时发出;owner 在 N+1 应用/持久化。
     一拍可见性差异可接受,doc 注明;若未来要求严格同拍,改走"owner 发布"。
4. **事件与迁移/命令的 canonical 序**:
   - `EnvelopePayload::ordinal`:MigrateEntity=0,Event=1,BlockWrite=2;
   - 同一目标同拍,信封排在命令后(既有 `dispatch_prefix` 逻辑);事件/迁移/
      block write 之间按 payload ordinal → source → sequence 排序;
   - 不得在同一 tick 对同一实体重复施加同一 Detonate(测试覆盖)。
5. **回放**:
   - 外部 Detonate 是命令,日志逐片重投;原线与重放的目标集合由日志固定;
   - 涌现事件/BlockWrite 不落日志,由重放中的分片状态/凝固桥确定性再生;
     若由此产生新信封,入 Router,apply_at = 产生拍+1;
   - checkpoint 在事件产生前后都能恢复(依赖任务 02 的 Router 队列快照)。

## 测试清单(先红后绿)

| 测试名 | 断言意图 |
| --- | --- |
| `detonate_targets_all_covering_shards` | 边界爆心;两片都在目标集;每片收到一次 |
| `detonate_excludes_far_shards` | 远离爆心的已注册分片不在目标集 |
| `detonate_logs_one_entry_per_shard` | 返回/日志按 ShardId 升序,同 tick/sequence |
| `boundary_detonate_matches_single_world` | 两片各一个实体 vs 单世界;位级/容差一致 |
| `detonate_batch_atomic_on_inbox_full` | 一个目标满 → 全拒;无半投 |
| `detonate_hostile_parameters_rejected` | NaN/inf/radius<=0 被驱动丢弃,不 panic |
| `detonate_center_in_empty_shard_hits_registered_neighbor` | 爆心所在分片未注册,但邻片有实体 → 仍广播到邻片(不要求中心分片存在) |
| `emergent_event_envelope_next_tick` | `broadcast_event` 后目标在 N+1 收到 Event;origin 不重复 |
| `emergent_event_canonical_and_once` | 同拍多事件 canonical 序;每目标恰一次 |
| `blockwrite_envelope_reaches_owner_and_holders` | 非 owner 凝固写:owner/holders 副本一致;owner 持久化 |
| `blockwrite_does_not_rebroadcast` | 应用 BlockWrite 的 owner 不再产生新信封(无回环) |
| `event_queue_full_is_typed` | 队列满 → `AdmitError`;不静默丢 |
| `detonate_replay_matches_original` | 日志逐片重投;重放窗位级一致 |

## 完成定义

- 门禁:`cargo test -p messaging -p physics-sim -p shard -p sim-api -p app
  --all-targets` + 对应 clippy `-D warnings` 退出码 0;
- 既有 AC3/AC-N3/AC-G5 测试零回归(手工两片 Detonate 广播模式仍保留为
  crate 级对照;生产路径改走 `Session::detonate`);
- 不实现撞击触发/毁方块/结构坍塌的玩法语义(只留 `Outbox.events` +
  `BlockWrite` 通道);不实现实体所有权(D12)。

## 风险与备注

- **Detonate 日志变多**:一次用户动作 1–4 条;回放逐片,符合 AC8;
- **事件 fanout**:目标集按 core∪holder 计算;分片数大时信封数线性增长,
  但每个分片只收一个;本轮不做事件合并;
- **BlockWrite 边界**:owner 必须已持有该 chunk;若任务 00 的 ensure 漏掉,
  这里会 fail-stop。测试要覆盖"观察者在边界 32m 内 + 凝固格跨到邻片"。
- **未来破坏**:各片对同一事件应用同一纯破坏函数;本地体素副本一致由
  BlockWrite/事件广播保证;破坏 REQ 不能绕过这条通道。
