# 任务 01:外部实体命令按 owner 路由与权限

## 目标

外部 `RemoveEntity`/`TeleportEntity` 不再按 `origin_shard` 伪造路由；由
`Session::entity_owner` 解析真实 owner，校验权限后投递。未知/错片/越权 →
类型化拒绝/丢弃，不 panic。

## 必读

- `docs/requirements/REQ-006-entity-authority.md` §4.3/§4.4
- `crates/sim-api/src/session.rs`
- `crates/app/src/driver.rs`(validate_external_command/dispatch_external)
- `crates/app/src/net.rs`(translate/command_shard)
- `crates/shard/src/scheduler.rs`(owner_of)

## 授权改动

- `crates/sim-api/src/session.rs`:
  - `entity_owner(entity) -> Option<EntityOwner>`;
  - `send_to_entity(source, entity, command) -> Result<LogEntry, SessionError>`;
  - `SessionError::Rejected` 已有；
- `crates/app/src/driver.rs`:
  - `validate_external_command` 增加 `source: Option<InputSource>` 参数
    (commands 通道传 None，网络传 Some(player));
  - Remove/Teleport：owner 必须存在;若 `source=Some(player)` 则
    `owner.source == Some(player)`;服务端实体(source=None)默认拒绝外部；
  - `dispatch_external` 对实体命令走 `session.send_to_entity`，忽略网络
    传来的 `shard`；
  - 计数/丢弃，不 panic；
- `crates/app/src/net.rs`:
  - `command_shard` 对 Remove/Teleport 不再伪造 origin 坐标;可返回
    `None` + `TranslateOutcome::Reject`? 或保留信息性 shard 但驱动忽略——
    **优先**：新增 `DriverCommand::EntityCommand { command }`，翻译层不决定分片；
  - 保持 `MarkActor`/观察者命令拒绝；
- 测试更新：net translate/command_shard、driver validate/dispatch。

## 接口设计

```rust
impl Session {
    pub fn entity_owner(&self, entity: EntityId) -> Option<shard::EntityOwner>;
    pub fn send_to_entity(
        &mut self,
        source: InputSource,
        entity: EntityId,
        command: SimCommand,
    ) -> Result<LogEntry, SessionError>;
}

// app
pub enum DriverCommand {
    // ...
    EntityCommand { command: SimCommand }, // 网络实体命令;owner 由驱动解析
}

pub fn validate_external_command(
    session: &Session,
    source: Option<InputSource>,
    shard: ShardId, // 非实体命令仍用;实体命令忽略
    command: &SimCommand,
) -> CommandGuard;
```

## 语义细则

1. **路由**：Remove/Teleport → `session.entity_owner(entity)`；返回 `None`
   → `Rejected("实体不存在/迁移窗口")`；`Some` → 投 owner 分片。
2. **权限**：
   - 网络（Some(player)）：`owner.source == Some(player)` 才允许；
   - 服务端实体（None）：外部拒绝；commands 通道（None）允许；
   - 未来玩法策略可扩展，本任务不开放“管理员”后门。
3. **不再使用 origin**：`net::command_shard` 对实体命令不再按
   `origin_shard` 造坐标；若保留旧形状，驱动必须忽略该 shard。
4. **回执**：本任务只保证路由/拒绝计数；spawn 回执在任务 02。
5. **Teleport 保真**：依赖任务 00 的 actor/source 保留；测试覆盖。
6. **hostile 边界**：未知实体、错 owner、越权 → 丢弃/计数，不 panic；
   不会触发 `ShardSim::despawn_entity` 的 expect。
7. **日志**：成功的 Remove/Teleport 入日志，source=玩家/会话，dest=owner；
   回放按日志重投。

## 测试清单

| 测试名 | 断言 |
| --- | --- |
| `entity_owner_lookup_after_spawn` | spawn 后 owner=目标片/source/actor |
| `remove_routes_to_current_owner` | 实体迁移后 Remove 仍命中新 owner |
| `teleport_routes_to_current_owner` | 迁移后 Teleport 命中新 owner,actor 保留 |
| `wrong_owner_command_rejected` | 错片/未知实体 → 拒绝,不 panic |
| `player_cannot_remove_other_entity` | A 删 B 的实体 → 拒绝;B 删 → 成功 |
| `server_owned_entity_rejected_external` | source=None 的实体外部命令拒绝 |
| `commands_channel_can_command_server_entity` | 本地权威通道允许 |
| `translate_no_origin_shard_routing` | 实体命令不再产生 origin 坐标路由 |
| `entity_command_log_dest_owner` | LogEntry.dest = owner |

## 完成定义

- 门禁:`cargo test -p sim-api -p app --all-targets` + clippy `-D warnings`;
- 既有 412+ 测试零回归;网络旧协议测试按需更新;
- 不实现 spawn 协议/ID 分配(任务 02)。

## 风险

- `validate_external_command` 签名变更会影响 app 测试/调用点;全量更新。
- commands 通道传 None 的权限语义要写清：本地权威不是“绕过所有规则”，
  只是允许操作服务端实体。
- Teleport 跨片目标位置：命令先投当前 owner；owner 内 despawn+spawn 后由
  下一拍迁移；不要在网络层预先改 owner。
