# REQ-005 决策清单(待拍板)

- 状态:**待拍板**(2026-10-18)
- GitHub issue:[#7](https://github.com/esp3j0/physics_minecraft/issues/7)
- 用途:给用户逐条选择;拍板后回写 [REQ-005-consistency.md](REQ-005-consistency.md) §9,
  并同步 [任务书](../tasks/req005-consistency/README.md)。
- 拍板方式:回复"全部按推荐"即可;或按 `D1=B, D3=A, ...` 只改个别项。
- 默认建议:**D1–D12 全部按推荐(A)**。推荐项按"最小改动 + 确定性 + 可测试"
  选出,不改变 REQ-001 的总体框架。

## 一览表

| ID | 议题 | 推荐(A) | 备选(B/C) |
| --- | --- | --- | --- |
| D1 | 范围与编号 | REQ-005 只做三件事 + 观察者可回放 | 004B / 拆两个 REQ / 把实体所有权也塞进来 |
| D2 | chunk owner 语义 | 静态 owner = 本体分片,holder 表可变 | 动态 claim/release;共享静态世界 |
| D3 | 外部编辑一致性 | 同拍复制到 owner+holders,每片一条日志,整批原子 | owner 先写、下一拍 ChunkSync;懒刷新 |
| D4 | 激活区域 | `core ∪ margin_ring` 过滤,`margin_meters` 生效 | 全局激活规划器;保持现状 |
| D5 | 邻片 ensure | 观察者 32m 内所有分片 + pending 迁移目的地 | 只 ensure 16m;只 ensure 迁移目的地 |
| D6 | 持久化归属 | 只有 owner 落盘;非 owner 卸载丢弃 | 任意 holder 落盘(后写覆盖) |
| D7 | 全局快照形状 | `WorldSnapshot = GlobalSnapshot + SchedulerSnapshot`;holders 重建 | 直接扩 `GlobalSnapshot`;拆开存 |
| D8 | checkpoint 兼容 | 预发布破旧格式,`format_version`,旧库明确报错 | serde 默认兼容;迁移工具 |
| D9 | 观察者回放 | 网络 Move/Remove 入日志 + 快照观察者表兜底 | 只快照;独立观察者日志 |
| D10 | 事件语义 | 外部 Detonate 复制命令;涌现事件 `EnvelopePayload::Event` | 单分片 + 广播信封;新日志 scope 标志 |
| D11 | block write 通道 | 新增 `Outbox.block_updates`,网络/持久化统一从它取 | 保留 `solidified` + 驱动镜像;以后再加 |
| D12 | 实体所有权 | 延后 REQ-006;本需求保持校验/丢弃 | 本轮一起做 |

---

## D1 范围与编号

**问题**:这三件事(静态一致性/全局快照/事件广播)放在哪个需求里?

- **A(推荐)**:新建 **REQ-005**,只做三件事 + "观察者/网络命令可回放"这个
  必要子项。实体所有权、方块破坏、流体、玩家实体化全部延后。
- B:叫 REQ-004B,挂在 viewer 需求下。
- C:拆成 REQ-005(静态一致性)+ REQ-006(快照/事件)两个需求。
- D:把实体所有权/命令路由也一起做(范围明显变大)。

**推荐理由**:三件事共享同一套 chunk/holder 状态,拆开会让快照和事件反复改
同一批结构;但实体所有权是另一条独立主线,塞进来会拖慢 viewer 之后的第一轮
修复。编号用 REQ-005 是为了不和 REQ-004A(viewer)混在一起。

**拍板**:[ ] A  [ ] B  [ ] C  [ ] D

---

## D2 chunk owner 语义

**问题**:REQ-001 写的是"chunk 同一 tick 只属于一个分片(claim/release)"。
实现上要不要真的做一张动态 claim 表?

- **A(推荐)**:**owner = 本体区分片**,由 `chunk.pos.shard_of()` 纯函数决定;
  不做动态 chunk claim/release(也不做动态重划分片)。运行时只登记
  **holder 集合**(哪些分片持有一份副本)。owner 永远在 holder 里。
- B:做动态 claim/release 表,允许 chunk 换 owner(为未来动态重划留口)。
- C:不做副本,所有分片共享一份静态世界(共享内存/Arc),各自派生 collider。

**推荐理由**:A 最符合"运行时不重划"的既定决议;owner 是纯函数,快照/回放
不需要存归属表,holder 可从各片已加载 chunk 重建。B 会引入 owner 迁移协议,
本轮不值得;C 与"worker 间不共享内存"的并发模型冲突。

**影响**:REQ-001 的"claim/release"在本项目里解释为"owner 静态 + holder 登记";
若用户坚持字面 claim 表,选 B,任务书 00 要加一张 owner 迁移表。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D3 外部编辑一致性(SetBlock)

**问题**:玩家在分片边界改一个方块,owner 和邻片扩张副本怎么保持一致?

- **A(推荐)**:**同拍复制命令**。调度器按 `pos` 找 owner + holders,在同一
  `apply_at` 给每个分片投同一盖章 `SetBlock`;每个分片各记一条 `LogEntry`
  (dest = 该分片);任一收件箱满 → 整批拒绝,不半投。回放按日志逐片重投,
  不需要新的日志格式。
- B:owner 先写;tick N 末给其他 holder 发 `ChunkSync`/`BlockUpdate` 信封,
  `apply_at = N+1`(边界有一拍旧 collider)。
- C:非 owner 副本懒刷新,读到 owner 时再取(跨片查询,违反"不跨世界查询")。

**推荐理由**:A 没有接缝,且复用现有"Detonate 手工两片各投一次"的日志模式
(REQ-003 测试已证明可回放);B 会在一拍内让两边 collider 不一致,正好是
AC2 最敏感的接触态;C 直接违反架构边界。

**影响**:一条用户编辑会变成 1–2 条日志(holder 数),日志略增;但回放简单、
无新协议。整批原子需要 `Router::route_batch` 预检容量。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D4 激活区域

**问题**:现在每个已注册分片都用同一批观察者算激活,导致观察者附近的 chunk
被多个分片重复装载。怎么限制?

- **A(推荐)**:每个分片有 `core = 8×8 chunk`;`margin_meters` 换算成
  `margin_chunks`(默认 16m → 1);激活候选只允许落在 `core ∪ margin_ring`。
  加载仍按观察者距离/滞回/冷却/预算;区域外的 chunk 一律不请求。
- B:保持"全局观察者半径"候选,但在 `Scheduler` 层做一次全局规划,把每个
  chunk 分配给 owner + 必要的 holder(更复杂,但全局预算更准)。
- C:保持现状(每个分片都装同一批),只加去重日志。

**推荐理由**:A 是改动最小、语义最直白的修复;`margin_meters` 终于被用上;
holder 只会出现在邻片扩张环,不会出现"分片 A 装载 500m 外 chunk"。
B 是更干净的长期形态,但会重写激活策略机,超出本轮。

**影响**:两分片/多分片测试的装载计数会变化(这是预期修复);单分片行为
基本不变(多一圈 margin)。每片预算仍独立,全局预算暂不引入。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D5 邻片 ensure

**问题**:驱动现在只 ensure 观察者所在分片;实体跨到未注册的邻片会变成
"信封黑洞"。怎么保证目的分片存在?

- **A(推荐)**:对每个观察者,ensure 其所在分片 + 核心区距离
  `awake_unload_radius`(默认 32m)内的所有分片(至多 4 个);另外 tick 末从
  Router 待投递信封取 `pending_shard_destinations()`,ensure 这些目的分片。
  仍不存在 → 内部一致性违规 fail-stop。
- B:只 ensure `margin_meters`(16m)内的邻片。
- C:只在迁移意图产生时 ensure 目的地,不做常规邻片 ensure。

**推荐理由**:A 覆盖"实体醒着才会迁移"(醒着要求观察者 24m 内,32m 有裕度),
同时让扩张副本有 owner;pending destinations 是兜底,确保不会黑洞。
B 在观察者 20m 时可能漏;C 会让扩张副本在迁移前不存在,边界物理仍有空窗。

**影响**:观察者靠近边界时会多注册 1–3 个分片;这些分片本来就需要扩张区,
不是额外开销。despawn 票据仍按 48m 判定,不会误杀。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D6 持久化归属

**问题**:一个 chunk 在 owner 和邻片各有一份;卸载时谁写库?

- **A(推荐)**:只有 **owner** 写库;非 owner 的副本卸载直接丢弃,不写。
  `unload_shard` 也只返回 owner chunk。
- B:任意 holder 都写,后写覆盖(现状);靠"副本一致"保证最终值相同。
- C:owner 写主副本,holder 写"覆盖层"(更复杂)。

**推荐理由**:A 是唯一能保证"不会用旧副本覆盖新编辑"的简单规则;B 在副本
不一致或卸载顺序不同时必出问题;C 是分布式存储的复杂度,本轮不需要。

**影响**:非 owner 的卸载不再产生 `put_chunks`;测试里的卸载计数会变。
owner despawn 的屏障只覆盖 owner chunk,语义更准确。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D7 全局快照形状

**问题**:checkpoint 怎么装下"分片字节 + 观察者表 + Router 信封 + 生命周期"?

- **A(推荐)**:保留 `GlobalSnapshot`;新增
  `WorldSnapshot { shards: GlobalSnapshot, scheduler: SchedulerSnapshot }`。
  `SchedulerSnapshot` 含观察者表/`observers_ever_set`/待投递信封/
  per-source sequence/生命周期;holders 从各片已加载 chunk 重建;
  `activation_requests`/`despawn_requests`/`solidified` 等每拍派生物不存。
- B:直接给 `GlobalSnapshot` 加字段(破坏现有构造/测试)。
- C:checkpoint 里分两个独立字段存(失去原子性)。

**推荐理由**:A 保留现有 `GlobalSnapshot` API/测试,新类型命名清楚,原子性
由 `CheckpointBundle` 保证。holders 可重建,不必存,避免两套归属状态。

**影响**:`CheckpointBundle` 形态变化;`replay` 新增 `replay_world`;
`bootstrap` 读新格式。旧 checkpoint 见 D8。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D8 checkpoint 兼容

**问题**:新快照格式与旧 `data/checkpoints.redb` 不兼容,怎么处理?

- **A(推荐)**:预发布阶段**不兼容**;`CheckpointBundle` 加 `format_version`;
  旧格式/未知版本 → 明确 `AppError::Decode`/启动失败,提示清空 `data_dir`;
  绝不静默降级。
- B:serde 默认字段,尽量兼容旧 checkpoint(可能读到半状态)。
- C:写迁移工具。

**推荐理由**:A 最安全;静默兼容会让 AC8 在一个"看起来能跑"的错误状态上
通过。项目还没有生产数据,破格式的成本最低。

**影响**:升级后首次启动需要清空/换 `data_dir`;文档要写。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D9 观察者回放

**问题**:网络 `MoveObserver` 现在原章直投、不进日志;checkpoint 也不含
观察者表。重放必然丢观察状态。怎么补?

- **A(推荐)**:新增 `Session::move_observer_from(source, pos)` /
  `remove_observer_from(source)`,网络驱动入日志;`SimCommand` 追加
  `RemoveObserver`;`replay` 把两种观察者命令路由到观察者收件箱。
  同时把观察者表放进 `SchedulerSnapshot` 作为 checkpoint 时刻的兜底。
- B:只把观察者表放进快照,不记日志——checkpoint 之后的移动仍无法重放。
- C:单独维护一份观察者日志(不混命令日志)。

**推荐理由**:A 才能让"多玩家 + 断线重连 + checkpoint 之后继续移动"整体
可回放;快照兜底让 checkpoint 时刻的观察状态不依赖日志窗口。C 多一个文件
和一套原子性,收益不大。

**影响**:网络 `MoveObserver`/`Left` 会多日志条目;`RemoveObserver` 是追加
枚举,不破坏协议;客户端白名单要拒绝它。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D10 事件语义

**问题**:玩家引爆炸弹,和分片模拟中涌现的爆炸,分别怎么广播?

- **A(推荐)**:
  - **外部命令 Detonate**:`Session::detonate` 按目标分片**复制命令**,同一
    `apply_at`,每片一条 `LogEntry`;回放逐片重投,无新日志格式;
  - **涌现事件**(未来撞击触发等):`Scheduler::broadcast_event` 在 tick N
    末给目标分片盖 `EnvelopePayload::Event(CrossEvent)`,`apply_at = N+1`;
    `ShardSim` 本地求值;`apply_at` 晚一拍是 REQ-001 已接受的固有语义。
- B:外部 Detonate 也只发一个分片,靠新 `EnvelopePayload::Event` 广播——会
  变成"下一拍才炸到邻片",与"命令调度事件同拍"语义冲突。
- C:给 `LogEntry` 加 `broadcast` 标志,回放时重算目标集合——日志格式变复杂,
  且目标集合可能随状态变化。

**推荐理由**:A 区分了"外部命令 = 同拍"与"涌现事件 = 下一拍",和 REQ-001
§4.2 完全一致;外部命令复制复用现有 AC3 测试模式,回放不需要新概念。

**影响**:Detonate 一次用户动作产生多片日志;目标集合由 holder/core 计算。
涌现事件本轮只留接口 + 合成测试,不实现撞击触发。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D11 block write 通道

**问题**:网络 `BlockUpdates` 现在靠驱动"镜像外部 SetBlock + take_solidified";
未来毁方块没有统一来源。要不要现在收口?

- **A(推荐)**:`Outbox` 增加 `block_updates: Vec<(BlockPos, BlockId)>`;外部
  SetBlock、凝固桥、未来破坏都写它;`Scheduler::take_block_updates()` 只发布
  owner 的写入;网络/持久化从同一处取。`take_solidified` 保留为兼容包装。
- B:保留现状(`solidified` + 驱动镜像),等破坏 REQ 再加。
- C:只给网络加一个 `BlockUpdates` outbox,持久化仍走 `solidified`。

**推荐理由**:A 是未来"爆炸毁方块/结构坍塌"的必要通道;现在收口成本低,
避免网络/持久化两套来源。B 会在破坏 REQ 里再改一次。

**影响**:`Outbox` 快照字段变化(旧 checkpoint 见 D8);`take_solidified` 的
行为可由新通道实现,测试保持兼容。

**拍板**:[ ] A  [ ] B  [ ] C

---

## D12 实体所有权

**问题**:外部 `RemoveEntity`/`TeleportEntity` 目前按 `origin_shard` 路由,
实体迁移后可能被丢弃;要不要本轮一起做实体所有权表?

- **A(推荐)**:**延后 REQ-006**。本轮保持 `validate_external_command` 的
  "校验不过就丢弃"语义;viewer 只观察,不受影响;文档标注该限制。
- B:本轮加实体所有权表 + 命令权威路由(范围 +1 个大任务)。

**推荐理由**:三件事已经足够大;实体所有权涉及 spawn/迁移/despawn 全链,
单独一拍更稳。A 不会让现有 AC 回归,只是保留已知限制。

**影响**:客户端对迁移后实体的 Remove/Teleport 可能被静默丢弃;若近期要做
"玩家物理实体/交互",应尽快排 REQ-006。

**拍板**:[ ] A  [ ] B

---

## 拍板记录

| ID | 选择 | 备注 |
| --- | --- | --- |
| D1 |  |  |
| D2 |  |  |
| D3 |  |  |
| D4 |  |  |
| D5 |  |  |
| D6 |  |  |
| D7 |  |  |
| D8 |  |  |
| D9 |  |  |
| D10 |  |  |
| D11 |  |  |
| D12 |  |  |

> 拍板后:把上表填好 → 回写 REQ-005 §9 → 任务书 README 的"架构师决策"
> 去掉"待拍板",即可交 zcode 工作流。
