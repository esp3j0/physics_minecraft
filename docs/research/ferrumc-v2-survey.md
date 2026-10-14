# ferrumc v2 架构调研(面向"物理版 Minecraft"多 Rapier 分片架构)

> 由探索 agent 于 2026-10-14 调研产出,所有行号来自本地
> `references/ferrumc-v2`(v2,rework/v2-skeleton)与 `references/ferrumc`(v1,master)。

## 1. 八篇 ADR 结论(docs/adr/)

| ADR | 结论 | 对物理分片架构 |
|---|---|---|
| 0001 Clean Rewrite | 抛弃 v1 的 Bevy ECS 全局状态、DashMap、LMDB 耦合,改为 actor 分片 + 消息传递 + 所有权式 chunk map;"Basic scheduler → tick coordinator with backpressure" | **照搬**:与"多 PhysicsWorld 分片 + tick 边界消息"完全同构 |
| 0002 Actor-Sharded Simulation | 每个 shard 独占 8×8 chunk 的 chunks/entities,**无锁并行 tick**;shard 内 SlotMap+ComponentVec 的 ECS 式存储,系统是普通 `fn(ctx: &mut ShardTickCtx)`;"Multiple Bevy worlds fights the framework"——显式函数调用替代隐式调度 | **照搬**:Rapier PhysicsWorld 每 shard 一个、独占所有权,正是此决策的物理版 |
| 0003 Single Tokio Runtime | 单 tokio runtime 只管网络 I/O 与异步编排;**模拟跑在专用线程池(非 Tokio task)**,存储走专用 worker 线程 | **照搬(关键)**:物理分片 worker 用专用 OS 线程 |
| 0004 Storage Backend | 存储收敛在 trait 边界(WorldStore 等),redb 起步;单写者 + 批量写 | **改造**:地形持久化可沿用"trait 边界 + 单写 worker + tick 末端批量 flush" |
| 0005 Compiled Rust Plugins First | 插件能力制(WorldView 只读、!Send、不跨 await) | **不适用/可选**;其"只读视图 + intent"边界值得借鉴 |
| 0006 C ABI Dynamic Plugins | cdylib + vtable 解决 Rust ABI 不稳 | **不适用** |
| 0007 redb Storage | RedbStore 落地,trait 不变可换后端;事务在 spawn_blocking,批量单事务提交 | **不适用**(模式可参考) |
| 0008 Trusted Native Plugins | 非沙箱、不可热卸载、fail-stop 的信任模型 | **不适用** |

## 2. driver.rs(tick 驱动)【已解决】

- **biased select 排序(关机 > 到期 tick > 命令)**:`server/app/src/driver.rs` L719-784:
  `tokio::select! { biased; _ = shutdown.changed() => {...} // L721
  _ = ticker.tick() => { run_tick(...) } // L742
  maybe_command = commands.recv() => ... // L762`
  L662-668 文档:"The loop prioritises shutdown, then a due tick, then a pending command,
  so a command flood can never starve the simulation."
- **MissedTickBehavior::Skip**:L696-698:"Lag must not trigger catch-up ticks: skip missed
  deadlines instead."
- **SimCommand 有界通道**:L4-5"Connection tasks reach it only through a bounded SimCommand
  channel";L212 起 enum 定义,"the only way a connection influences simulation state; it
  carries no sockets and no shard handles"。满时策略:tick 内 `drain_shard_inputs` 只取
  inbox 能容纳的量,多余留在有界 router 通道下 tick 重试(L1465-1468)。
- **收件箱满断开玩家**:L893-939 `enforce_input_delivery`:`SessionError::ShardInboxFull` 时
  `DeliveryPolicy::Authoritative | Coalescable` 走 `router.disconnect_player_owned(player)`
  (L928-937);BestEffort 容忍。
- **重要限定**:app 层 driver 只持有**单个** `SimShard`(L681;server.rs L547
  `SimShard::new(ShardPos::new(0, 0))`;server.rs L251"one authoritative simulation owner")。
  **多分片调度器存在但未接入 app**(shadow 代码)。

## 3. 分片定义(shard_pos.rs / ownership.rs)【已解决】

- `ferrumc-math/src/shard_pos.rs` L9:`CHUNK_SHIFT: i32 = 3`(8×8 chunk/shard);
  L44:`ChunkPos::new(self.x << CHUNK_SHIFT, self.z << CHUNK_SHIFT)`。
- `ferrumc-sim/src/ownership.rs` L19:`SHARD_WIDTH_CHUNKS = 8`;L36-40:`ShardId { world,
  dimension, position }`——身份含世界/维度。L225-248 `ShardPartitioner`(无状态坐标→shard);
  L256-331 `ShardOwnershipMap::claim/release/owner`(BTreeMap,重复 claim 报
  `OverlappingShardOwnership`)。
- 持有者:逻辑上 `ShardScheduler` 持 `BTreeMap<ShardId, ScheduledShard>`;当前 app 由
  driver task 独占一个 `SimShard`(ownership.rs L4-7 注释明言服务于"non-default shadow
  scheduler")。

## 4. 跨分片交互【部分解决——消息已实现但未接线;实体迁移未实现】

**结论:跨片"消息"已实现(crate 级、确定性、有界、tick 边界应用),但标注为 crate-private
shadow 代码、没有 app 接线;跨片"实体迁移/handoff"完全未实现(migration/migrate/handoff/
transfer 在 ferrumc-sim 与全部 ADR 中 0 命中)。**

- `ferrumc-sim/src/cross_shard.rs`(546 行):
  - L8-11:`#![allow(dead_code, reason = "the Packet 42 cross-shard carrier is
    crate-private shadow plumbing without app wiring")]`——官方自认未接线。
  - 机制:`CrossShardIntent { destination, payload }`(L38)→ 调度器盖章为
    `CrossShardEnvelope { produced_at, apply_at, source, destination, source_sequence,
    payload }`(L69);**apply_at ≡ produced_at + 1**(L84-102:"Accepted envelopes always
    target exactly the tick after they were produced")。
  - 目前唯一 payload 是 `ApplyInput(GameInput)`(L22-25)——转发一条模拟输入,**不是实体搬运**。
  - `CrossShardQueue`(L245-371):有界(默认 1024),准入**不驱逐旧条目**,确定性排序键
    `(destination, source, source_sequence)`(L135-137),拒绝原因枚举
    QueueFull/SameShard/UnknownDestination/DestinationNotActive/NoNextTick(L158-175),
    prepare/commit 两阶段防陈旧(L335-370)。
- 规划痕迹:INVARIANTS.md L63-67:"Cross-shard workers emit destination-only intents; only
  the scheduler may stamp... Accepted cross-shard envelopes produced in tick N apply
  exactly once as a **separate prefix** to the destination's tick N+1 execution";
  scheduler.rs L1502:"test instrumentation for the Packet 42 carrier until a gameplay..."
- teleport 在 app 中只是单 shard 内的路由移动(`SimCommand::TeleportPlayer` →
  `router.teleport_player_owned`,driver.rs L1172-1177)。
- **启示**:v2 的"intent → 调度器盖章 → next-tick 边界前缀应用 + 有界确定性队列"可直接
  照搬为物理实体跨片消息;**实体迁移(位置快照 + 原片删除 + 新片插入)需自行设计**。

## 5. 分片生命周期、worker 分配与 tick 对齐【部分解决(机制完整,未投产)】

- 生命周期(ownership.rs L333-407):`Created → Active → Draining → Stopped` 单向状态机;
  Draining 拒新输入、跑完已接纳工作(scheduler.rs L585-590;INVARIANTS L59-61)。
- 创建:scheduler.rs L1073 `register(shard)`。**销毁:无显式 despawn API**;Draining→Stopped
  为终态不可复活(L346-347)。app 中 shard 启动时一次性创建(server.rs L547)。
- **空片照常 tick**(未发现跳过逻辑;INVARIANTS L54"每个 runnable shard 每 tick 恰好访问一次")。
- shard→线程:`WorkerSlot(ordinal % worker_slots.get())`(scheduler.rs L124-127)——按 ShardId
  规范序**静态轮转**,无运行时负载均衡/工作窃取。worker 为持久 OS 线程
  `ferrumc-sim-worker-{index}`(L700 附近),**capacity-one rendezvous 通道**
  (`WORKER_CHANNEL_CAPACITY = 1`,"larger queue would only permit accidental tick overlap"),
  上限 `MAX_SHARD_WORKERS = 64`。
- **每 tick 全 worker 汇合屏障**:scheduler.rs L15-21:"transfers owned shard batches to a
  fixed group of worker threads... **the scheduler waits for every batch to return** before
  the next tick... the same shard can never be entered concurrently"——所有权转移式并行,
  而非共享内存屏障。默认模式 `AuthoritativeInline`(L51-54);L621:"the shadow seam is a
  correctness path, not an app configuration surface"。
- 全局 tick:`TickCoordinator` 单一权威推进,"never consults a wall clock";INVARIANTS
  L16-17:"tick counter advances by exactly one per advance... **no catch-up ticks**: lag
  is reported, never replayed"。

## 6. chunk_stream.rs(每玩家 chunk 流)【已解决】

- 每泵装/卸上限 16:L33 `MAX_CHUNK_LOADS_PER_UPDATE = 16`;L41 `MAX_CHUNK_UNLOADS_PER_UPDATE
  = 16`(注释:单泵最多 32 次 chunk 变更)。
- 切比雪夫距离 center-out:L379-383 `chebyshev_distance`;L408-410
  `to_load.sort_by_key(|pos| (chebyshev_distance(center, *pos), pos.x(), pos.z()))` +
  truncate。
- 视距差分 + latest-wins:L408-415 `desired.difference(loaded)` / `loaded.difference(desired)`;
  每泵从最新 center 重算、不保留队列;pending_position 单槽合并(L69-70);视距钳制
  `STREAM_VIEW_DISTANCE_MAX = 32`(L49)。
- chunk 票据:`ferrumc-sim/src/ticket.rs` L58-67 `TicketReason { Spawn, Player, Forced }`,
  "Spawn and forced chunks tick";玩家票据 driver.rs L1347;出生点票据 radius-2 方阵
  (chunk_stream.rs L87-96)。
- 卸载前持久化屏障:driver.rs L1396-1443 `release_chunks_acked`——先向 storage worker 发
  oneshot ack 的 flush 并 await,提交后才 drop 票据("Bug A barrier",L1400-1409)。

## 7. v1 game_loop.rs【已解决】

- 多周期调度器:L243-329 `build_timed_scheduler`:tick(1/TPS,**Burst** + max_catch_up 5,
  L271-275)、world_sync(15s,Skip)、chunk_gc(5s,Skip)、keepalive(1s,Skip + 相位偏移 250ms,
  L314-318);小顶堆 peek/pop 到期任务、`park_until_next_due()`(L137-213);全局补帧上限
  `MAX_GLOBAL_CATCH_UP = 64`。
- 网络独立线程:L378-387 专用 `TokioNetworkThread`(独立 current_thread runtime),与主循环
  crossbeam channel 传新连接(L67)。
- 无锁队列:`crossbeam_queue::SegQueue`(src/lib/core/src/mq.rs L4-11)。
- **对比**:v1"多周期 + 补帧"单线程 Bevy;v2 反其道"单周期 20TPS + Skip 不补帧"——物理分片
  应采纳 v2 语义:超载降 TPS 而不追赶。

## 总体判断

1. **可照搬的骨架**:8×8 分片 + 独占所有权无锁 tick(ADR-0002)、专用线程池 + capacity-one
   通道 + 每 tick 全员汇合屏障、TickCoordinator 单一全局 tick 号 + Skip 不补帧、biased
   select、有界 SimCommand + inbox 满断开玩家、卸载前持久化屏障。
2. **需自行补齐**:跨片能力止步于"确定性有界消息队列(next-tick 前缀应用)"且是未接线 shadow
   代码;**实体迁移/handoff、动态负载均衡(现为静态轮转)、空片跳过均无现成实现**。可复用
   CrossShardEnvelope 的"调度器盖章 + canonical 排序 + 恰好下一 tick 应用",其上装实体状态
   快照载荷。
3. **风险**:v2 多分片路径从未在 app 投产(单 shard 权威路径),确定性声明由 crate 测试与
   INVARIANTS.md 支撑而非生产验证;照搬时应把"shadow→权威切换"当作自身项目的一等里程碑。
