# 任务 02:服务端 ID 分配与 Spawn/Spawned 协议

## 目标

外部 spawn 由服务端分配唯一 `EntityId`，客户端发 `C2S::Spawn { state, desc }`，
服务端回 `S2C::Spawned { entity }`；viewer 适配，继续能生成实体。

## 必读

- `docs/requirements/REQ-006-entity-authority.md` §4.4/§4.5
- `crates/sim-api/src/proto.rs`
- `crates/app/src/net.rs`(translate/NetUp/OutboundHub)
- `crates/app/src/driver.rs`(dispatch/observe)
- `crates/viewer/src/render.rs`/`client.rs`(B 键 spawn)
- `crates/sim-api/src/session.rs`(spawn_owned)

## 授权改动

- `crates/sim-api/src/proto.rs`:
  - `C2S::Spawn { state: EntityState, desc: EntityDesc }` 追加；
  - `S2C::Spawned { entity: EntityId }` 追加；
  - 协议只追加，声明序不变；
- `crates/sim-api/src/session.rs`:
  - `spawn_owned(shard, entity, state, desc, source) -> Result<LogEntry, SessionError>`;
  - 目标分片 = 出生位置分片；未注册/未加载 → Rejected；
- `crates/app/src/driver.rs`:
  - `DriverCommand::Spawn { player, state, desc }`(或 NetUp 直通)；
  - ID 分配器：server namespace(origin 0) + 单调 sequence；不得复用；
  - 校验出生位置/分片；`session.spawn_owned`；成功 → `S2C::Spawned` 入
    player Critical；失败 → 计数/拒绝；
- `crates/app/src/net.rs`:
  - translate `C2S::Spawn`；外部 `C2S::Command(SpawnDynamic)` 从白名单移除
    (Reject)；
  - 出站 `Spawned` 经 OutboundHub；
- `crates/viewer/src/render.rs`/`client.rs`:
  - B 键发 `C2S::Spawn` 而不是 `SpawnDynamic`；
  - 收 `S2C::Spawned` 后记录自己的实体 id，用于后续操作/显示；
  - 不改变 demo 的其它行为；
- 测试。

## 接口设计

```rust
// proto
pub enum C2S {
    // ...
    Spawn { state: EntityState, desc: EntityDesc },
}
pub enum S2C {
    // ...
    Spawned { entity: EntityId },
}

impl Session {
    pub fn spawn_owned(
        &mut self,
        shard: ShardId,
        entity: EntityId,
        state: EntityState,
        desc: EntityDesc,
        source: InputSource,
    ) -> Result<LogEntry, SessionError>;
}

// app
pub enum DriverCommand {
    Spawn { player: InputSource, state: EntityState, desc: EntityDesc },
}
```

## 语义细则

1. **ID 分配**：server namespace `origin=0`，`local_seq` 单调从 0 起；
   `EntityId::from_parts(0, seq)`；超 u32 理论边界 panic(fail-stop)。
2. **出生分片**：`state.translation.chunk_of().shard_of()`；未注册/未加载
   → 拒绝(不自动建空气 chunk)；驱动 ensure? 外部 spawn 只允许已加载区域，
   viewer 相机所在区域已加载。
3. **归属**：`spawn_owned` 的 `source` = player；outbox spawned 让 Scheduler
   claim owner;`Spawned` 回执携带 ID。
4. **外部命令白名单**：`C2S::Command(SpawnDynamic)` → Reject；内部/测试/
   RL 仍可用 `SimCommand::SpawnDynamic`。
5. **viewer**：收到 Spawned 后记录；不把 ID 作为本地权威，仅显示/操作句柄。
6. **回放**：外部 spawn 入日志(实体 ID 由服务端分配);日志含 ID/tick/source;
   replay 时 `submit_raw` 到 owner;owner 表由恢复实体表重建。
7. **多客户端并发**：ID 分配单线程驱动,天然不撞;两个 Spawned 回执各自
   自己的连接。

## 测试清单

| 测试名 | 断言 |
| --- | --- |
| `proto_spawn_spawned_roundtrip` | 新变体 bincode 往返;旧变体编码不变 |
| `external_spawn_gets_server_id` | 客户端 Spawn → Spawned ID origin=0/唯一 |
| `two_clients_spawn_unique_ids` | 两连接并发 spawn → ID 不撞 |
| `spawn_rejected_unloaded_area` | 未加载/未注册 → 拒绝,不建空气 chunk |
| `spawn_dynamic_external_rejected` | 外部 SpawnDynamic → Reject |
| `viewer_spawn_uses_spawn_packet` | viewer client 发 Spawn/收 Spawned |
| `external_spawn_logged_and_replayed` | 日志/回放终态一致 |

## 完成定义

- 门禁:`cargo test -p sim-api -p app -p viewer -p e2e --all-targets` + clippy;
- 既有 network/viewer 测试零回归;
- 旧协议变体编码不变;只追加。

## 风险

- 这是 REQ-006 唯一的协议变更;必须只追加、旧变体不变。
- viewer 适配不能扩成未来客户端;只改 spawn 入口/ID 记录。
- `Spawned` 回执在 Critical 队列;慢客户端满 → 已有断连策略。
- 服务端 ID 分配器要单调且可回放;不要用墙钟/随机。
