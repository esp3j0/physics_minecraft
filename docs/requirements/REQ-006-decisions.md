# REQ-006 决策清单(待拍板)

- 状态:**待拍板**(2026-10-19)
- 用途:逐条选择;拍板后回写 [REQ-006-entity-authority.md](REQ-006-entity-authority.md) §8。
- 默认建议:**D1–D5 全部 A**。

## 一览表

| ID | 议题 | 推荐(A) | 备选(B/C) |
| --- | --- | --- | --- |
| D1 | EntityId 分配 | 外部 spawn 服务端分配 | 玩家命名空间 + 唯一校验;仅唯一校验 |
| D2 | spawn 协议 | 新增 `C2S::Spawn` + `S2C::Spawned` | 只加回执不加请求;无协议(客户端 ID) |
| D3 | ownership 持久化 | 不入快照，恢复重建 | 入快照，重建校验 |
| D4 | 迁移/传送保真 | actor + source 入迁移 payload;Teleport 保留 | actor 入 EntityDesc;只保 desc |
| D5 | 外部 spawn 语义 | 仅 `Spawn` 请求;外部 `SpawnDynamic` 拒绝 | 接受客户端 ID但命名空间校验;保留现状 |

---

## D1 EntityId 分配

- **A(推荐)**:外部 spawn 由服务端分配 ID。`C2S::Spawn` 只带 state/desc；
  服务端用 server namespace(origin 0) + 单调 sequence 分配，回执 `S2C::Spawned`。
  内部/测试/RL 仍可显式带 ID。
- B:客户端自选 ID，但用玩家 `InputSource` 做命名空间 + 服务端全局唯一校验。
- C:只加全局唯一校验，仍允许任意客户端 ID。

**推荐理由**:A 从协议层消灭撞 ID/抢 ID，权威边界最清楚;代价是协议和 viewer
适配。B 改动小，但 ID 语义被玩家 ID 污染;C 不适合多人。

## D2 spawn 协议

- **A(推荐)**:`C2S::Spawn { state, desc }` + `S2C::Spawned { entity }`。
  Remove/Teleport 暂不加通用回执(错误计数/日志;后续可加 `CommandResult`)。
- B:只加 `Spawned` 回执，但仍让客户端带 ID。
- C:无协议，维持客户端自选 ID。

**推荐理由**:A 与 D1 配套，且只追加变体，不破坏旧编码。

## D3 ownership 持久化

- **A(推荐)**:不入快照;恢复时从各分片实体表重建并校验全局唯一。
- B:入 `WorldSnapshot`，恢复时再校验。

**推荐理由**:A 避免两套状态;实体表是真相，ownership 是索引。性能需要时再缓存。

## D4 迁移/传送保真

- **A(推荐)**:`EnvelopePayload::MigrateEntity` 增加 `actor` + `source`；
  `TeleportEntity` 在分片内 despawn+spawn 时保留二者。
- B:actor 放入 `EntityDesc`，source 单独。
- C:只保 desc(现状)。

**推荐理由**:actor 是政策状态，不是物理描述;迁移 payload 是最小传递面。

## D5 外部 spawn 语义

- **A(推荐)**:外部只允许 `C2S::Spawn`;`C2S::Command(SpawnDynamic)` 从白名单
  拒绝。内部/测试仍可用 `SpawnDynamic`。
- B:允许客户端 `SpawnDynamic`，但校验命名空间/唯一性。
- C:保留现状。

**推荐理由**:A 与 D1/D2 一致，外部无法抢 ID;内部路径保持可测可控。

## 拍板记录

| ID | 选择 | 备注 |
| --- | --- | --- |
| D1 |  |  |
| D2 |  |  |
| D3 |  |  |
| D4 |  |  |
| D5 |  |  |

> 拍板后:回写 REQ-006 §8，更新任务书 README，再交实现。
