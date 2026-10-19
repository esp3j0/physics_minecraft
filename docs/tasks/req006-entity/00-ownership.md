# 任务 00:EntityOwnership 表与实体生命周期

## 目标

建立全局唯一的 `EntityId → (ShardId, source, actor)` 归属表，并在 spawn /
despawn / migrate / teleport 生命周期上维护；恢复时从各分片实体表重建。

## 必读

- `docs/requirements/REQ-006-entity-authority.md` §4.1/§4.2
- `docs/tasks/req006-entity/README.md`
- `crates/physics-sim/src/sim.rs`(spawn/despawn/apply_prefix/Outbox/EntityRecord)
- `crates/physics-sim/src/snapshot.rs`(SnapshotEntityRecord)
- `crates/shard/src/scheduler.rs`(register/tick/outbox)
- `crates/messaging/src/envelope.rs`(MigrateEntity)

## 授权改动

- `crates/messaging/src/envelope.rs`:`MigrateEntity { actor, source }` 字段追加；
- `crates/physics-sim/src/sim.rs`:
  - `EntityRecord` 增 `source: Option<InputSource>`(actor 已有)；
  - `Outbox` 增 `spawned: Vec<EntityId>` / `despawned: Vec<EntityId>`；
  - `spawn_entity` 接收 source/actor;`despawn_entity` 返回/记录 source/actor；
  - `apply_prefix` 的 Command 路径传 `input.source`；
  - 迁移 intent 带 actor/source;目的 spawn 恢复；
  - `entity_owners()` 只读接口(重建用)；
- `crates/physics-sim/src/snapshot.rs`:`SnapshotEntityRecord` 增 source；
- `crates/shard/src/entities.rs`(新):`EntityOwner`/`EntityOwnership`；
- `crates/shard/src/lib.rs` re-export；
- `crates/shard/src/scheduler.rs`:
  - 字段 `entity_ownership`；
  - `register`/`restore_state` 末尾 `rebuild_entity_ownership()`；
  - tick outbox 处理：先全部 despawned → release，再 spawned → claim(重复 panic)；
  - `owner_of(entity)`/`entity_owners_of(shard)` 查询；
- 测试各 crate。

## 接口设计(签名固定)

```rust
// shard::entities
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntityOwner {
    pub shard: ShardId,
    pub source: Option<InputSource>,
    pub actor: bool,
}
#[derive(Debug, Default, Clone)]
pub struct EntityOwnership { /* BTreeMap */ }
impl EntityOwnership {
    pub fn owner_of(&self, entity: EntityId) -> Option<EntityOwner>;
    pub fn claim(&mut self, entity: EntityId, owner: EntityOwner); // duplicate -> panic
    pub fn release(&mut self, entity: EntityId);                   // absent -> panic
    pub fn rebuild(&mut self, entries: impl Iterator<Item = (EntityId, EntityOwner)>);
    pub fn len(&self) -> usize;
}

// physics-sim
pub struct Outbox {
    // ...
    pub spawned: Vec<EntityId>,
    pub despawned: Vec<EntityId>,
}
impl ShardSim {
    /// 重建/测试用:(entity, owner_source, actor);不暴露内部 handle。
    pub fn entity_owners(&self) -> Vec<(EntityId, Option<InputSource>, bool)>;
}

// shard
impl Scheduler {
    pub fn owner_of(&self, entity: EntityId) -> Option<EntityOwner>;
    pub fn entity_owners_of(&self, shard: ShardId) -> BTreeSet<EntityId>;
}
```

## 语义细则

1. **唯一性**:`claim` 已存在 → panic(fail-stop);`release` 不存在 → panic
   (内部一致性;hostile 路径由上层先查 owner，不直接 release)。
2. **顺序**:每拍先处理所有 `despawned`(按 ShardId 升序)，再处理 `spawned`;
   Teleport 同拍 despawn+spawn → 最终 owner 正确。
3. **迁移**:源片 `despawned` 删除 owner;目的片下一拍 `spawned` claim;迁移窗口
   owner 缺失，查询返回 None(外部命令拒绝)。
4. **source/actor**:
   - 外部 spawn 的 source = 玩家;内部 spawn = None；
   - actor 来自 `MarkActor`/迁移 payload;Teleport 保留；
5. **重建**:`register`/`restore_state` 末尾扫描各 `sim.entity_owners()`;
   重复 EntityId → panic;不得从快照单独恢复 ownership。
6. **快照**:`SnapshotEntityRecord` 增 source;旧格式按 REQ-005 `format_version`
   策略拒绝(若字段变更，任务 03 统一升版)。

## 测试清单

| 测试名 | 断言 |
| --- | --- |
| `entity_owners_claim_release_unique` | claim/查询/release;重复 claim panic |
| `spawn_claims_despawn_releases` | 一 spawn 一 despawn 后表空 |
| `teleport_keeps_owner_and_actor` | Teleport 后 owner shard 不变、actor/source 保留 |
| `migrate_moves_owner_next_tick` | 源 despawn 后 owner None;目的 spawn 后 owner=dest |
| `duplicate_spawn_across_shards_panics` | 两片同 ID spawn → 第二拍 claim panic |
| `rebuild_from_shards_detects_duplicate` | 两片实体表同 ID → rebuild panic |
| `spawned_despawned_outbox_survive_snapshot` | 未消费生命周期记录随快照往返 |
| `migrate_payload_preserves_actor_source` | 信封字段与目的记录一致 |

## 完成定义

- 门禁:`cargo test -p messaging -p physics-sim -p shard --all-targets` +
  `cargo clippy -p messaging -p physics-sim -p shard --all-targets -- -D warnings`;
- 既有测试零回归(Outbox 新字段导致构造变化需更新，notes 说明);
- 不实现外部命令路由/spawn 协议(任务 01/02)。

## 风险

- `Outbox`/`EnvelopePayload` 字段变更会破坏旧 checkpoint;任务 03 统一升
  `format_version`，本任务先如实注释。
- `EntityRecord` 增 source 会改快照布局;测试必须覆盖 restore。
- 迁移窗口 owner 缺失是**设计语义**，不要用旧 owner 兜底，否则会投错片。
