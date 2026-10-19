# REQ-006:实体所有权与外部实体命令权威路由

- 状态:**草案**(2026-10-19 起草;D1–D5 决策待拍板,架构师默认全 A)
- 上游:
  - [REQ-001](REQ-001-architecture.md) §4.1(实体唯一归属)、§9.7(EntityId)、
    §9.13(跨片实体接触限制)
  - [REQ-003](REQ-003-network.md) §4.1(命令白名单)、§4.2(翻译层)
  - [REQ-005](REQ-005-consistency.md) D12(实体所有权延后到本 REQ)
- 关联 issue:[#10](https://github.com/esp3j0/physics_minecraft/issues/10)
- 关联代码:`crates/shard`、`crates/physics-sim`、`crates/sim-api`、
  `crates/messaging`、`crates/app`、`crates/viewer`、`crates/e2e`

---

## 1. 项目定义

把实体从“各分片本地各管各的”升级为**全局唯一归属 + 服务端权威路由**：

- 任意 `EntityId`，服务端能可靠回答“它现在归哪个分片、归哪个玩家/服务端”；
- 外部 `RemoveEntity`/`TeleportEntity` 不再按 `origin_shard` 伪造路由；
- 外部 spawn 由服务端分配 ID 并回执，客户端不能撞 ID/抢别人的实体；
- 迁移/传送保留 actor/owner 等政策状态；
- 快照恢复重建 ownership 并校验全局唯一。

一句话:**多人交互/玩法（玩家实体、抓取、推拉、删除/传送、后续 RL）的实体权威地基。**

## 2. 既定共识(继承)

1. REQ-001/002/003/005 全部决议沿用；
2. 模拟核确定性纪律不变：核 crate 无墙钟/随机/IO；实体命令仍走盖章输入；
3. 实体唯一归属一个分片；跨片迁移仍是 tick 边界“删除 + 重建”；
4. 服务器权威：客户端只发请求/命令，不发送权威实体位置；
5. viewer 是 demo，不是未来客户端；但 REQ-006 的 spawn 回执需要 viewer 适配；
6. 旧 checkpoint 兼容策略沿用 REQ-005 `format_version`：**不做迁移，显式拒绝**。

## 3. 现状缺口(代码证据)

1. **无全局实体所有权表**
   `Scheduler` 只有分片/chunk holder；`EntityId → ShardId` 不存在。实体表在每个
   `ShardSim` 内部，迁移后无法 O(1) 找 owner。

2. **外部实体命令按 `origin_shard` 伪造路由**
   `crates/app/src/net.rs::command_shard` 对 `RemoveEntity`/`TeleportEntity` 用
   `entity.origin_shard() as i32` 当 `ShardPos.x`。`origin_shard` 是分片注册序号，
   不是坐标;实体迁移后 owner 也变。结果命令被投到错误分片，
   `validate_external_command` 按“该分片实体存在”校验 → 丢弃；绕过校验则
   `ShardSim::apply_command` 的未知实体 panic。

3. **客户端自选 EntityId**
   `C2S::Command(SpawnDynamic { entity, .. })` 让客户端决定 ID；只在单分片
   `spawn_entity` 内断言重复，没有全局唯一性/归属校验。两个玩家可撞 ID，或
   跨分片重复；客户端也可猜别人的 ID 发 Remove/Teleport。

4. **迁移/传送丢政策状态**
   `EnvelopePayload::MigrateEntity` 只带 `state + desc`，不带 `actor`/`owner`；
   `TeleportEntity` 走 despawn+spawn，actor/window 重置。演员豁免跨片/传送后丢失。

5. **没有 spawn 回执**
   外部 `SpawnDynamic` 由客户端选 ID，服务端没有“已分配/已接受”的回执；
   Remove/Teleport 也没有成功/拒绝回执。

6. **快照恢复无全局唯一性校验**
   `WorldSnapshot` 逐分片恢复实体表；若历史状态有重复 EntityId，没有全局
   fail-stop。

## 4. 架构

### 4.1 EntityOwnership 表

在 `shard` 新增:

```rust
pub struct EntityOwner {
    pub shard: ShardId,
    /// `Some(player)` = 外部 spawn 的玩家;`None` = 服务端/内部实体。
    pub source: Option<InputSource>,
    /// 演员政策标记(REQ-002 决议 4b);迁移/传送必须保留。
    pub actor: bool,
}

pub struct EntityOwnership {
    owners: BTreeMap<EntityId, EntityOwner>,
}
```

- `owner_of(entity) -> Option<EntityOwner>`；
- `entities_of(shard) -> BTreeSet<EntityId>`(可选)；
- `claim(entity, owner)`：已存在 → panic(fail-stop，全局唯一)；
- `release(entity)`：不存在 → panic(内部一致性)；
- `rebuild_from_shards(...)`：恢复时从各分片实体表重建，重复 ID → panic；
- 表**不入快照**：从 `ShardSim` 实体表重建；若未来性能需要再入快照(决策 D3)。

### 4.2 生命周期更新点

`ShardSim` 的 `Outbox` 追加实体生命周期记录:

- `spawned: Vec<EntityId>`：新实体(spawn / 迁移目的重建 / 传送重建)；
- `despawned: Vec<EntityId>`：实体移除(Remove / 凝固 / 迁移源删除)；
- `migrating` 已有 `Intent`；迁移源推 `despawned`，目的分片下一拍 spawn 推
  `spawned`；迁移窗口内 owner 暂时不存在(外部命令拒绝，不投错片)。

`Scheduler` 每拍 outbox 处理顺序:

1. 全部 `despawned` → `release`；
2. 全部 `spawned` → `claim`(若重复 = fail-stop)；
3. 迁移 `Intent` 盖章入队(不改 owner，等目的片 spawn 再 claim)。

`TeleportEntity` 在 `ShardSim` 内 despawn+spawn，outbox 同时有 `despawned` 和
`spawned`；按“先 despawn 后 spawn”处理，owner 不丢。

### 4.3 外部实体命令路由

- `Session::entity_owner(entity) -> Option<EntityOwner>` 透传；
- `Session::send_to_entity(source, entity, command)`:
  - 解析 owner；
  - 不存在 → `SessionError::Rejected`(hostile 不 panic)；
  - `TeleportEntity`/`RemoveEntity` 投 owner 分片；日志 `dest = owner`；
- 驱动 `validate_external_command`:
  - `RemoveEntity`/`TeleportEntity`：owner 存在 + 权限；
  - 网络来源 `player`：`owner.source == Some(player)` 才允许；`None`(服务端
    实体)默认拒绝(除非未来玩法策略显式允许)；
  - commands 通道(本地权威)：可传 `None`(受信)；
- `MarkActor` 仍服务端专用，客户端白名单拒绝；
- `net::command_shard` 不再为实体命令伪造坐标；理想形状是
  `DriverCommand::EntityCommand { command }`，由驱动解析 owner；至少要做到
  “实体命令的 shard 参数被忽略，路由以 `Session::entity_owner` 为准”。

### 4.4 外部 spawn:服务端分配 + 回执(D1/D2 推荐 A)

- 新增 `C2S::Spawn { state: EntityState, desc: EntityDesc }`(不带 EntityId)；
- 服务端分配 `EntityId`(origin 0 = server，local_seq 单调；0 保留给服务端)；
- 驱动:
  1. 目标分片 = 出生位置所属分片；未注册/未加载 → 拒绝(不自动建空气 chunk)；
  2. 分配 ID，`Session::spawn_owned(owner, id, state, desc, source)`；
  3. 成功 → `S2C::Spawned { entity: id }` 入该玩家 Critical；
  4. 失败 → `S2C::CommandRejected { reason }`(或计数丢弃，v1 可选)；
- `S2C::Spawned` 是**输出协议追加**，不参与 canonical 输入排序；
- 内部/测试/RL 仍可用 `SimCommand::SpawnDynamic` 显式带 ID(受信路径)；
- 外部 `C2S::Command(SpawnDynamic)` 从白名单移除/拒绝(避免客户端抢 ID)。

### 4.5 迁移/传送保真

- `EnvelopePayload::MigrateEntity` 追加:
  - `actor: bool`
  - `source: Option<InputSource>`
  - (可选 `slow_ticks/window` 不迁移，重建时重置)
- 目的 `spawn_entity` 用 payload 的 actor/source 重建 `EntityRecord`；
- `TeleportEntity` 在 `despawn_entity` 时保留 `EntityRecord` 的 actor/source，
  重建时写回；
- 迁移/传送后 owner 表 claim 到新分片/同一分片，重复 claim fail-stop。

### 4.6 快照/恢复

- `SnapshotEntityRecord` 增加 `source: Option<InputSource>`(actor 已有)；
- 恢复 `ShardSim` 后，`Scheduler::restore_state` 末尾
  `rebuild_entity_ownership()`；
- 全局重复 EntityId → panic(fail-stop)；
- `WorldSnapshot`/`format_version` 若字段变化，按 REQ-005 D8 升版并显式拒绝旧档；
- 回放：实体命令日志 `dest` 以 owner 为准；重放时 owner 表由恢复后的实体表重建。

## 5. 模块归属(蓝图 §7 增量)

| 新东西 | 去处 |
| --- | --- |
| `EntityOwner`/`EntityOwnership`、owner 查询/claim/release/rebuild | `shard`(新 `entities.rs`) |
| `Outbox.spawned/despawned`、迁移 payload actor/source、传送保真 | `physics-sim` |
| `Session::entity_owner`/`send_to_entity`/`spawn_owned` | `sim-api` |
| `C2S::Spawn`/`S2C::Spawned`、ID 分配器 | `sim-api`(proto)/`app` |
| 驱动实体命令路由/权限、spawn 回执 | `app`(driver/net) |
| viewer 适配 Spawn/Spawned | `viewer` |
| AC-E1~E8 验收 | `e2e` |

## 6. 非目标

- 玩家物理实体化/操控模型；
- 客户端预测/本地 sim(#9)；
- 方块破坏/结构坍塌/流体；
- 动态重划分片；
- 跨片实体接触(REQ-001 §9.13 已接受限制)；
- 完整作弊防护/鉴权(仅做 owner 权限校验)。

## 7. 验收标准(逐条可测)

- **AC-E1 全局唯一**:任意时刻每个 EntityId 至多一个 owner；重复 spawn →
  fail-stop；`owner_of` 与各分片实体表一致。
- **AC-E2 外部命令路由**:实体迁移到另一分片后，`RemoveEntity`/
  `TeleportEntity` 仍命中新 owner；错误 owner/未知实体 → 类型化拒绝/丢弃，
  不 panic；不再使用 `origin_shard` 路由。
- **AC-E3 spawn 分配/回执**:外部 `Spawn` → 服务端分配唯一 ID → `Spawned`
  回执；两个客户端并发 spawn 不撞 ID；客户端不能自选 ID 抢实体。
- **AC-E4 迁移保真**:迁移后 actor/source 保留；迁移窗口内外部命令拒绝而非
  投错片；目的片实体存在且唯一。
- **AC-E5 快照/恢复**:checkpoint 恢复后 ownership 重建、全局唯一；replay
  终态位级一致；旧格式显式拒绝。
- **AC-E6 权限/多玩家**:玩家 A 不能 Remove/Teleport 玩家 B 的实体；B 可以；
  服务端实体默认拒绝外部命令。
- **AC-E7 协议兼容**:C2S/S2C 只追加；旧变体编码不变；viewer 适配后能
  spawn/看到自己的实体。
- **AC-E8 回归**:全仓 `cargo test --workspace`、clippy、fmt 全绿；既有
  412+ 测试零回归。

## 8. 决议(待拍板)

见 [REQ-006-decisions.md](REQ-006-decisions.md)。推荐:

| ID | 议题 | 推荐 |
| --- | --- | --- |
| D1 | EntityId 分配 | 服务端分配(外部 spawn) |
| D2 | spawn 协议 | 新增 `C2S::Spawn` + `S2C::Spawned` |
| D3 | ownership 持久化 | 不入快照，恢复重建 |
| D4 | 迁移/传送保真 | actor/source 入迁移 payload;Teleport 保留 |
| D5 | 外部 spawn 语义 | 仅 `Spawn` 请求;外部 `SpawnDynamic` 拒绝 |
