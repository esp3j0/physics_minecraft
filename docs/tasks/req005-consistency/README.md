# REQ-005 实现任务书(总览)

- 状态:**已定稿**。需求 [REQ-005](../../requirements/REQ-005-consistency.md)
  与 [决策清单](../../requirements/REQ-005-decisions.md) 已拍板(2026-10-18,
  D1–D12 全部按推荐 A);按最终决议执行。
- GitHub issue:[#7](https://github.com/esp3j0/physics_minecraft/issues/7)
- 实现方式:glm 写核心(00–03),qwen 写验收(04)+ 独立评审;cargo 门禁;
  架构师终审。
- 编号:REQ-004A 是 viewer,本任务是 **REQ-005**;不要与 REQ-003 网络轮混跑。
- 启动时机:等 zcode 的 REQ-004A(viewer)收尾后再开;本任务书与 viewer 改动
  范围不重叠(本任务只碰模拟核/shard/sim-api/app 的既有文件)。

## 任务顺序(依赖序,串行)

| # | 任务书 | 内容 | 实现模型 | 门禁包 |
| --- | --- | --- | --- | --- |
| 0 | [00-chunk-ownership.md](00-chunk-ownership.md) | chunk owner/holder、区域过滤激活、邻片 ensure、owner-only 持久化 | glm | types physics-sim shard app |
| 1 | [01-expansion-consistency.md](01-expansion-consistency.md) | 外部 SetBlock 同拍广播、副本一致、BlockUpdates outbox | glm | physics-sim shard sim-api app |
| 2 | [02-global-checkpoint.md](02-global-checkpoint.md) | `WorldSnapshot`/`SchedulerSnapshot`、观察者入日志、replay/bootstrap | glm | messaging shard sim-api app |
| 3 | [03-event-broadcast.md](03-event-broadcast.md) | Detonate 复制命令、涌现事件信封广播、事件验收 | glm | messaging physics-sim shard sim-api app |
| 4 | [04-acceptance.md](04-acceptance.md) | AC-C1~C9 端到端套件 + 独立评审 | qwen | e2e |

## 公共完成定义

1. 严格 TDD:先写【测试清单】全部测试 + 最小 `todo!()` 桩,门禁包见红
   (编译通过、测试失败),再实现到绿;
2. 门禁(机器判定,不可绕过):门禁包 `cargo test --all-targets` 与
   `cargo clippy --all-targets -- -D warnings` 退出码均 0;
3. 只改任务书【交付物】与【授权改动】列出的文件;不 `git commit`/`push`;
4. `cargo fmt` 只格式化你改动的文件;
5. 既有测试零回归(当前全仓 301+ 测试;含 viewer 的测试).
6. 诚实汇报:没跑过的检查不说跑过;做不到/有矛盾如实说明并 escalate,
   不绕过门禁、不弱化断言。

## 架构师决策(拍板后不得偏离)

以 [REQ-005-decisions.md](../../requirements/REQ-005-decisions.md) 的
**D1–D12 最终选择(全 A)**为准:

1. **D2**:owner = 本体分片纯函数;holder 表可变;不做动态 claim;
2. **D3**:外部 `SetBlock` 同拍复制到 owner+holders,每片一条日志,整批原子;
3. **D4**:激活候选 = `core ∪ margin_ring`;`margin_meters` 换算为
   `margin_chunks`(默认 16m → 1);
4. **D5**:观察者 32m 内所有分片 + `pending_shard_destinations` 都 ensure;
5. **D6**:只有 owner 落盘;非 owner 卸载丢弃;
6. **D7**:`WorldSnapshot = GlobalSnapshot + SchedulerSnapshot`;holders 从
   已加载 chunk 重建;不存每拍派生物;
7. **D8**:`format_version`;旧 checkpoint 明确报错,不静默;
8. **D9**:网络 `MoveObserver`/`RemoveObserver` 入日志 + 快照观察者表兜底;
   `SimCommand::RemoveObserver` 追加,客户端白名单拒绝;
9. **D10**:外部 Detonate = 复制命令(同拍、每片一条日志);涌现事件 =
   `EnvelopePayload::Event`(apply_at = N+1);
10. **D11**:`Outbox.block_updates` 统一 block write;`take_solidified` 保留
    兼容包装;
11. **D12**:实体所有权/外部实体命令路由延后 REQ-006;本任务保持校验/丢弃。

## 跨任务纪律

- **不要改 REQ-001/002/003 的既定语义**:本任务只兑现"归属/快照/广播",
  不改 tick 纪律、迁移协议、凝固桥参数、网络协议版本;
- **确定性**:核 crate 仍禁墙钟/随机/IO/无序容器;新增 `serde` 结构字段序
  固定;holders/信封/lifecycle 都用 `BTreeMap/BTreeSet`;
- **fail-stop 边界**:内部一致性违规(owner 不存在、信封目的地未注册、
  snapshot tick 不一致)panic;hostile 客户端输入只能拒绝/丢弃/断连;
- **API 变更**:跨 crate API 先改测试;新增 `pub` 需在汇报里说明消费者;
- **文件规模**:单文件超 ~500 行必须拆;本任务已有多文件接近上限,优先在
  现有模块内加小模块,不要继续堆 `scheduler.rs`。

## 验收与评审

- 04 的 e2e 套件是最终契约;架构师终审以 04 + 独立评审为准;
- 评审重点:副本一致性、owner-only 持久化、快照完整性、事件恰好一次、
  hostile 输入不 panic、既有 301+ 测试零回归;
- viewer 冒烟:AC-C8 里只要求"服务器 + viewer 可连、收 Welcome/ChunkData",
  渲染细节仍归 REQ-004A 的 ACV4 手动清单。
