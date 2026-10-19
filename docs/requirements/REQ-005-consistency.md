# REQ-005:分片静态一致性、全局快照与跨片事件

- 状态:**已完成**(2026-10-19 任务 00–04 全部验收,AC-C1~C9 通过;2026-10-18 拍板开工)
- GitHub issue:[#7](https://github.com/esp3j0/physics_minecraft/issues/7)
- 提出日期:2026-10-18
- 上游:
  - [REQ-001](REQ-001-architecture.md) §4.1(分片/所有权)、§4.2(跨片信封/事件/迁移)、
    §9.6(canonical 序)、§10(AC2/AC3/AC8)
  - [REQ-002](REQ-002-world-loading.md) §4.4(装载编排)、§4.5(despawn/票据)、
    §10(AC-G5 回放兼容)
  - [REQ-003](REQ-003-network.md) §4.2/§4.3/§4.4(观察者、命令、观察输出)、
    §9(AC-N1/N3/N6)
- 关联代码:`crates/shard`、`crates/physics-sim`、`crates/sim-api`、`crates/messaging`、
  `crates/app`、`crates/e2e`
- 编号说明:**REQ-004A 已用于最小 viewer**;本需求是模拟核的一致性收口,独立编号
  REQ-005,不塞进 viewer 需求,也不叫"REQ-003 挂账"(它不是网络轮的小修,而是
  三条架构承诺的兑现)。

---

## 1. 项目定义

REQ-003 完成后,网络链路已经能把客户端接进世界、看 chunk 流和实体流、改方块、
引爆;但代码里有三处"架构承诺"还没有真正闭环:

1. **chunk 归属与扩张区一致性**——目前只有分片所有权,没有 chunk 归属;激活
   只按观察者距离算,没有按分片本体区过滤;多个分片会各自装载同一批 chunk;
   一个 `SetBlock` 只改一个副本;持久化可能后写覆盖先写。
2. **全局快照**——`GlobalSnapshot` 只含每分片字节;观察者表、Router 待投递
   信封、生命周期计时、网络观察者移动都不在 checkpoint 里。AC8 只在"特意
   避开这些状态"的测试场景成立。
3. **跨片事件广播**——`CrossEvent` 的本地求值有了,但调度器没有广播编排;
   网络 `Detonate` 只发给爆心所在的一个分片;AC3 的等价性测试是手工向两片
   各投一次,不是生产路径。

本需求把这三件事做成**可测试的既有决议兑现**,不改变 REQ-001 的总体框架
(九 crate、五层依赖、确定性核、tick 边界交互、命令进/快照出)。

一句话:**让"分片"在静态世界上真正像同一个世界,让 checkpoint 真正能续跑,
让跨片事件真正能跨片。**

## 2. 既定共识(继承,不再辩论)

1. REQ-001/002/003 全部决议沿用;本需求需要变更其中任何一条时,先修订其 §9;
2. 体素数组仍是唯一真相;collider 仍是派生缓存;一切位素写入必须可追溯到
   owner chunk;
3. 模拟核确定性纪律不变:无墙钟/随机/IO,容器用 `BTreeMap/BTreeSet`,
   输入由调度器盖章;
4. 跨片只发生在 tick 边界:信封 `apply_at = produced_at + 1`;实体迁移 =
   删除 + 重建;事件 = 共享参数 + 各片本地确定性求值;
5. 客户端输入是 hostile 的:失败必须是"拒绝/丢弃/断连",不能变成模拟核
   fail-stop panic(REQ-003 评审已确立的门卫语义);
6. viewer(REQ-004A)只是观察者,不参与本需求的模拟侧一致性判定。

## 3. 已验证事实与当前缺口(代码证据)

### 3.1 已经有的

- `OwnershipMap` 管**分片** claim/release;`Scheduler` 每拍对已注册分片
  全量派活、barrier 回收(AC5 结构保证);
- `ActivationController` 有滞回/冷却/预算;`take_activation_requests()` 把
  tier①② 决策外露,驱动经 `ChunkProvider` 回填;
- `Scheduler::take_solidified()`/`entity_states()` 观察输出全链;
- `GlobalSnapshot` + `replay` + checkpoint 已有 AC8 测试;AC-G5 有观察者/
  装载/爆炸的 600 拍回放;
- `CrossEvent::Detonate` 本地求值位级等价(demo D / AC3);
- `Session::remove_observer` 已补;`validate_external_command` 已在驱动侧
  拒绝 hostile 输入(REQ-003 评审修复)。

### 3.2 三个缺口(本需求要闭合)

**缺口 A:chunk 归属/扩张区没有运行时规则**

- `crates/shard/src/ownership.rs` 只有 `ShardId` 的 claim/release,没有
  chunk 归属表,也没有"哪些分片持有一份副本"的登记;
- `ActivationController::plan` 的候选集只由观察者距离和 `loaded` 决定,
  没有分片本体区/扩张区过滤;`Scheduler::tick` 又对每个已注册分片调用
  `apply_activation` 且传入同一批观察者——多个分片会各自请求装载观察者
  附近的同一批 chunk;
- 两分片测试的装载是**手工**把 0..=8 / 7..=15 的 chunk 塞进各自的 sim
  (`e2e/tests/migration.rs`、`shard/tests/two_shards.rs`),没有验证运行时
  归属;
- `SetBlock` 只投给位置所在分片,邻片/扩张区的副本不更新;卸载时每个持有
  副本的分片都可能 `put_chunks`,存在后写覆盖;
- `margin_meters` 只在 `app::Config` 和默认值测试里出现,没有接进激活/
  迁移/扩张区。

**缺口 B:checkpoint 不是"全局"的**

- `GlobalSnapshot` 只有 `tick + per_shard bytes`;`CheckpointBundle` 只有
  `snapshot + log_bytes`;
- `Scheduler` 的观察者表、`observers_ever_set`、Router 的待投递信封和
  per-source sequence、生命周期计时都不在 checkpoint;
- 网络 `MoveObserver` 在 `ObserveOrchestrator::consume_net_up` 里以
  `submit_observer_raw` 原章直投,**不进命令日志**;`Left` 的
  `remove_observer` 也不进日志;
- `replay` 只回放 `entry.tick > snapshot.tick` 的日志条目——checkpoint 之前
  的观察者命令被过滤,观察者表没有别的来源,恢复后为空;
- 如果 checkpoint 恰好落在"tick N 末已产生迁移信封、apply_at = N+1"的窗口:
  源分片实体已删除,目的分片还没收到,信封只在 Router 内存里;快照没有
  Router 队列 → 恢复后实体消失。现有 AC8 长程测试特意让迁移发生在快照前,
  没覆盖这个窗口。

**缺口 C:跨片事件没有广播编排**

- `CrossEvent::Detonate` 只在 `ShardSim::apply_prefix` 里本地求值;
  `Scheduler`/`Router` 没有 `broadcast_event` 或等价的广播入口;
- 网络 `translate` 把 `C2S::Command(Detonate)` 路由到爆心所在分片,
  [net.rs](../../crates/app/src/net.rs) 的 `command_shard` 只返回一个
  `ShardId`;
- AC3/e2e 的"广播"是测试手工把同一个 `StampedInput` 分别 submit 给两个
  分片;生产路径没有这个动作,爆心在分片边界时邻片实体不会受影响;
- 未来"爆炸毁方块"若各片只改自己的体素副本,没有广播/归属规则就无法保证
  边界一致。

## 4. 架构

### 4.1 chunk 归属、扩张区与持久化

**4.1.1 归属 = 本体区分片(纯函数)**

- 一个 chunk 的 **owner** = `chunk.pos.shard_of()` 所在分片(连同 world/dim);
  这是纯函数,不做动态 chunk claim/release,也不做动态重划分片;
- 一个 chunk 可以被多个分片**持有**(holder):owner 持有本体副本;边界
  扩张区的邻片持有只读缓存副本;holder 集合是唯一需要登记的运行时状态;
- `margin_meters` 换算为 `margin_chunks = ceil(margin_meters / 16)`(默认 1);
  分片的静态范围 = `core ∪ margin_ring`;激活候选必须落在这个范围内;
- 扩张区副本只为边界 collider/物理服务,不承载实体;实体仍唯一归属本体区。

**4.1.2 外部编辑:同拍广播到所有 holder**

- 外部 `SetBlock` 是命令,不是涌现事件;它在 `apply_at` 拍对 owner + 所有
  holder **同时**生效,避免"owner 改了、邻片下一拍才改"的接缝;
- 实现形状:调度器按 `pos` 找到 owner + holder 集合,原子地给每个分片投递
  同一盖章命令,每个分片各记一条 `LogEntry`(dest = 该分片);回放按日志
  逐分片重投,不引入新的日志格式;
- 任何分片收件箱满 → 整批拒绝(不半投),返回类型化反压错误;
- **涌现写入**(凝固桥、未来的爆炸毁方块)不同:它发生在 tick N 内,只能
  在 tick N 末盖章给其他 holder/受影响分片,`apply_at = N+1`;这是 REQ-001
  §4.2 已接受的"涌现事件晚一拍"语义,必须文档化、测试化。`EnvelopePayload`
  追加 `BlockWrite { pos, block }`;各 holder 应用同一写入,owner 持久化,
  应用 `BlockWrite` 不再产 `block_updates`(防广播回环)。

**4.1.3 持久化:只由 owner 落盘**

- `apply_chunk_unload(shard, pos)` 仅在 `shard == owner(pos)` 时把 chunk 交给
  驱动持久化;非 owner 的副本卸载直接丢弃(不写库);
- `unload_shard(shard)` 只返回该分片作为 owner 的 chunk;扩张副本丢弃;
- 分片 despawn 的持久化屏障只覆盖 owner chunk;不会再用"后写覆盖先写";
- owner 分片必须存在:如果某个 holder 持有一个 chunk,而 owner 分片没注册,
  这是编排 bug;ensure 规则必须保证它不会发生。

**4.1.4 邻片 ensure 与迁移目的地**

- 驱动 `orchestrate` 目前只 ensure 观察者所在分片;迁移目的地/扩张区所在
  邻片可能是"黑洞";
- 本需求:对每个观察者,ensure 其所在分片 + 核心区距离
  `awake_unload_radius` 内的所有分片(默认 32m;至多 4 个);另外在 tick 末
  从 Router 待投递信封中取 `pending_shard_destinations()`,ensure 这些目的
  分片,保证 `apply_at` 到来前分片已注册;
- 若目的分片仍不存在 → 内部一致性违规,fail-stop(不是 hostile 输入路径);
  不允许信封永久沉在桶里。

### 4.2 全局快照与回放

**4.2.1 快照形状**

保留 `GlobalSnapshot`(每分片不透明字节,兼容既有测试),新增:

```rust
pub struct WorldSnapshot {
    pub shards: GlobalSnapshot,
    pub scheduler: SchedulerSnapshot,
}

pub struct RouterSnapshot {
    pub envelopes: Vec<Envelope>,
    pub sequences: BTreeMap<ShardId, u64>,
}

pub struct SchedulerSnapshot {
    pub tick: Tick,
    pub observers: BTreeMap<InputSource, Vec3>,
    pub observers_ever_set: bool,
    pub router: RouterSnapshot,                   // 待投递队列 + per-source sequence
    pub lifecycles: BTreeMap<ShardId, LifecycleSnapshot>,
}
```

- `holders` 不入快照:恢复时从每个 `ShardSim` 的已加载 chunk 重建(load/unload
  已登记,纯函数可重建);
- `lifecycles` 只存当前已注册分片;`Stopped` 观测残留不入快照;
- `Router.inboxes` 不入快照:外部命令都在命令日志里,回放按 `dest` 重投;
  `observer_inbox` 的未来条目也由日志覆盖(见 4.2.2);
- `activation_requests`/`despawn_requests`/`solidified` 等每拍派生物不入快照;
- `CheckpointBundle` 增 `format_version`;旧格式/未知版本 → 明确错误,不静默
  降级到一个"看起来能跑"的世界。

**4.2.2 观察者与网络命令必须可回放**

- 新增 `Session::move_observer_from(source, position)`、`remove_observer_from(source)`:
  盖章、入观察者收件箱、返回 `LogEntry`;网络驱动必须 `log.append`;
- 新增追加式 `SimCommand::RemoveObserver`(无字段,source 隐含):观察者收件箱
  按 source 删除;分片前缀永不出现(出现即 fail-stop);客户端白名单拒绝;
- `Session::send_from(source, shard, command)` 让网络命令日志带真实玩家 source
  (`send` 保留为会话 source 的测试/装配入口);
- `replay` 对 `SetObserverPosition`/`RemoveObserver` → 观察者收件箱原章直投;
  其余 → `submit_raw(dest)`;**重放窗口是 `entry.tick >= snapshot.tick`**:
  checkpoint 的 `tick` 是"下一待执行拍"(`tick_now()`),等于该拍的命令尚未
  应用,必须回放;`entry.tick < snapshot.tick` 的已进快照,不再重放。现有
  `entry.tick > snapshot.tick` 会把 `== snapshot.tick` 的待执行命令排除,
  本需求修正;
- 快照里的观察者表是"checkpoint 时刻"的兜底;日志是 checkpoint 之后的
  权威来源。两者叠加,网络 session 也能 AC8 位级回放。

**4.2.3 恢复顺序**

1. `Scheduler::new(start_tick = snapshot.tick)`;
2. 逐分片 `register(ShardSim::restore(...))`(keys = ownership);
3. `scheduler.restore_state(scheduler_snapshot)`(观察者表/Router/生命周期);
4. 从每片已加载 chunk 重建 holder 表;
5. 回放日志 `tick >= snapshot.tick`(等于该拍的待执行命令尚未应用);
6. 驱动照常 ensure/装载/despawn。

恢复时校验:`scheduler.tick == shards.tick`;lifecycle keys = sim keys;
Router 信封的 destination 要么已注册、要么可被 ensure;不满足 → fail-stop。

### 4.3 跨片事件广播

**4.3.1 外部命令触发的爆炸:按分片复制命令**

- `Session::detonate(center, power, radius) -> Vec<LogEntry>`:
  - 目标分片 = 所有已注册分片中,核心区 AABB 或持有的 chunk 与事件 AABB
    (`center ± radius`)相交者(holder 规则为未来毁方块预留);
  - 对每个目标分片投递同一 `SimCommand::Detonate`,同一 `apply_at`,每个
    分片一条 `LogEntry`;整批原子(任一收件箱满 → 全拒);
  - 回放按日志逐分片重投,不引入新的日志格式/scope 标志;
- 网络 `Detonate` 走 `Session::detonate`,不再只投爆心分片;
- 这与现有 AC3 测试的"手工两片各投一次"语义相同,只是把动作收进生产 API。

**4.3.2 涌现事件:调度器广播**

- `EnvelopePayload` 追加 `Event(CrossEvent)`(ordinal 追加在最后);`ShardSim`
  在输入前缀收到 `EnvelopePayload::Event` 时本地求值(复用 `detonate`);
- `Scheduler::broadcast_event(origin, event, produced_at)`(tick 末/内调用)对
  目标分片逐个 `Envelope::stamp`(`apply_at = produced_at + 1`),入有界队列;
- 目标集合规则:已注册分片中,核心区 AABB 或持有 chunk 与事件 AABB 相交者;
  按 `ShardId` 升序,信封序列由 origin 的 per-source counter 保证 canonical 序;
- 未来 `Outbox.events` 作为涌现事件的生产口(本需求只留接口 + 用合成事件
  测试广播链,不实现撞击触发/毁方块);
- 事件恰好一次:每个目标一个信封;`take_batch` 取走即清;同拍不重复。

## 5. 含糊点与决议

见 §9 与 [REQ-005-decisions.md](REQ-005-decisions.md)。本需求把每条含糊点都
落成"推荐选项 + 备选 + 影响",拍板后回写本节。

## 6. 模块归属(蓝图 §7 增量)

| 新东西 | 去处 |
| --- | --- |
| `ChunkRegion`、region ↔ core 换算 | `types`(坐标) |
| 激活候选区域过滤、margin 换算 | `physics-sim::activation` + `ShardSim` |
| holder 表、owner/holders 查询、批量投递、pending destinations、广播事件 | `shard`(scheduler/router/ownership) |
| `WorldSnapshot`/`SchedulerSnapshot`、`replay_world`、观察者来源 API | `sim-api`(snapshot/replay/session) |
| `EnvelopePayload::Event`/`BlockWrite`、`SimCommand::RemoveObserver` | `messaging` |
| `Outbox.block_updates`/`BlockWriteKind`、事件/block write 本地应用 | `physics-sim` |
| 驱动 ensure/持久化 owner/网络 MoveObserver 入日志/Detonate 广播 | `app`(driver/net/config) |
| AC-C1~C9 验收 | `e2e` |

## 7. 非目标(本需求明确不做)

- **实体所有权表与外部实体命令的权威路由**(`RemoveEntity`/`TeleportEntity`
  跨片仍可能被 `validate_external_command` 丢弃;viewer 只观察,不受影响)——
  REQ-006 候选;
- **方块破坏/结构坍塌/流体**——本需求只把 block write/event 的通道和归属
  规则打好;破坏语义单独立 REQ;
- 玩家物理实体化、操控模型;
- 动态重划分片、动态负载均衡;
- 真 MC 协议、压缩/加密、WAN 优化;
- viewer 渲染本身(REQ-004A);本需求只在验收里做"服务器 + viewer 可连"冒烟;
- 旧 checkpoint 的自动迁移(决议 D8)。

## 8. 验收标准(逐条可测试)

- **AC-C1 归属与区域**:任意 tick,每个已加载 chunk 的 owner 唯一且等于
  `chunk.pos.shard_of()`;holder 集合只出现在 `core ∪ margin_ring`;任何分片
  不加载自己区域外的 chunk;`margin_meters` 从 Config 到激活的实际换算生效。
- **AC-C2 外部编辑一致**:边界 `SetBlock` 对所有 owner+holder 在同 `apply_at`
  生效;各副本逐字节相等;双片边界场景与单世界参考的轨迹误差 < 1e-4 m
  (在 AC2 参考世界上加同一编辑)。
- **AC-C3 持久化归属**:非 owner 卸载不写库;owner 卸载写库;重新装载取回
  编辑值;双副本卸载顺序任意,不会出现后写覆盖。
- **AC-C4 迁移目的地**:观察者站在分片边界 32m 内,实体跨片时目的分片必已
  存在;无信封黑洞;迁移误差满足 AC2;`pending_shard_destinations` 为空后
  才进下一拍。
- **AC-C5 全局快照**:在"待迁移信封 + 多观察者 + 生命周期计时"状态下拍
  checkpoint,恢复 + 重放到终态:实体/分片键集/观察者表/tick 位级一致;
  实体不丢失;despawn 计时不被重置;快照字节同输入确定;`entry.tick ==
  snapshot.tick` 的待执行命令必须被回放,不能丢。
- **AC-C6 观察者可回放**:网络 `MoveObserver`/`RemoveObserver` 入日志;
  多玩家移动/断线/重连场景从 checkpoint 重放,观察者表/ticket/激活结果与
  原线一致;无幽灵观察者;`replay` 不重复应用。
- **AC-C7 跨片事件**:边界 `Detonate` 经 `Session::detonate`/网络路径覆盖
  所有相关分片;合并结果与单世界位级一致;每个目标恰好一次;canonical 序;
  涌现事件 `broadcast_event` 走 `EnvelopePayload::Event`、`apply_at = N+1`。
- **AC-C8 回归**:`cargo test --workspace` 全绿(既有 301+ 测试零回归);
  `cargo clippy --workspace --all-targets -- -D warnings`;`cargo fmt --all
  --check`;REQ-004A viewer 可连服务器收到 Welcome/ChunkData。
- **AC-C9 规模不变量**:同一 chunk 不被同一分片重复登记 holder;观察者附近
  不会出现"每个已注册分片都装载同一批 chunk";事件 fanout ≤ 目标分片数;
  checkpoint 字节大小随状态线性、无重复副本。

## 9. 决议(2026-10-18 拍板:D1–D12 全部按推荐 A)

逐条清单、推荐选项、备选与影响见 [REQ-005-decisions.md](REQ-005-decisions.md)。
拍板后本节回写为最终决议(编号 D1–D12),并同步任务书
[docs/tasks/req005-consistency/README.md](../tasks/req005-consistency/README.md)。

| ID | 议题 | 决议(= 推荐) |
| --- | --- | --- |
| D1 | 范围与编号 | REQ-005 只做三件事 + 观察者可回放;实体所有权/破坏延后 |
| D2 | chunk owner 语义 | 静态 owner = 本体分片;holder 表可变;不做动态 claim |
| D3 | 外部编辑一致性 | 同拍复制到 owner+holders;每片一条日志;整批原子 |
| D4 | 激活区域 | `core ∪ margin_ring` 过滤;`margin_meters` 生效 |
| D5 | 邻片 ensure | 观察者 32m 内所有分片 + pending 迁移目的地 |
| D6 | 持久化归属 | 只有 owner 落盘;非 owner 卸载丢弃 |
| D7 | 全局快照形状 | `WorldSnapshot = GlobalSnapshot + SchedulerSnapshot`;holders 重建 |
| D8 | checkpoint 兼容 | 预发布破旧格式;`format_version`;旧库明确报错 |
| D9 | 观察者回放 | 网络 Move/Remove 入日志 + 快照观察者表兜底 |
| D10 | 事件语义 | 外部 Detonate 复制命令;涌现事件走 `EnvelopePayload::Event` |
| D11 | block write 通道 | 新增 `Outbox.block_updates`;网络/持久化统一从它取 |
| D12 | 实体所有权 | 延后 REQ-006;本需求保持"校验/丢弃" |

## 10. Demo

本需求是系统一致性/回放/广播逻辑,没有新的物理数值未知项;按 REQ-002/003
先例**不新设 Python demo**。验证手段:

- 现有 demo A(迁移)/D(爆炸)结论继续作为位级等价依据;
- 新增 e2e 场景(见 AC-C1~C9)覆盖归属、扩张、快照、事件;
- viewer(REQ-004A)作为人工冒烟:边界编辑/迁移/爆炸无可感知闪烁。
