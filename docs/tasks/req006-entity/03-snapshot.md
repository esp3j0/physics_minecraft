# 任务 03:ownership 重建、快照/回放与格式版本

- 状态:**实现完成,门禁通过(2026-10-19)**。

## 目标

checkpoint/restore/replay 时从各分片实体表重建 ownership 并校验全局唯一；
`EntityRecord` 的 source/actor 随快照走；格式变化用 `format_version` 显式拒绝旧档。

## 必读

- `docs/requirements/REQ-006-entity-authority.md` §4.6
- `crates/sim-api/src/snapshot.rs`/`replay.rs`/`session.rs`
- `crates/shard/src/scheduler.rs`(snapshot_state/restore_state/rebuild)
- `crates/app/src/driver.rs`(CheckpointBundle/format_version)
- REQ-005 任务 02 的现有 WorldSnapshot/format_version

## 授权改动

- `crates/physics-sim/src/snapshot.rs`:`SnapshotEntityRecord` 增 source；
- `crates/shard/src/scheduler.rs`:`restore_state` 末尾
  `rebuild_entity_ownership()`;重复 ID → panic；
- `crates/sim-api/src/snapshot.rs`/`session.rs`:若字段变化，更新
  `WorldSnapshot`/`Session::world_snapshot` 注释；
- `crates/app/src/driver.rs`:`CHECKPOINT_FORMAT_VERSION` 升版(若快照布局变化);
  `decode` 拒绝旧版并提示清 `data_dir`；
- 测试。

## 语义细则

1. `restore_state` 顺序：注册分片 → 恢复 scheduler 状态 → 重建 holders →
   **重建 ownership**;任何重复 EntityId → panic(fail-stop)。
2. `SnapshotEntityRecord` 增 `source: Option<InputSource>`;actor 已有;
   restore 后 `EntityOwnership` 的 source/actor 与记录一致。
3. `format_version`：REQ-006 改变 `Outbox`/`EntityRecord`/Envelope 布局 →
   升到 2;旧检查点显式 `AppError::Decode`;不静默。
4. `replay_world`：实体命令日志 `dest` 以 owner 为准;重放后 ownership
   从恢复实体表重建;外部 spawn 日志含服务端 ID,直接 `submit_raw`。
5. 迁移 pending envelope：恢复后 owner 缺失(源已 despawn、目的未 spawn)
   是合法窗口;`entity_owner` 返回 None;外部命令拒绝。
6. 不允许把 `EntityOwnership` 单独存快照后再重建;两套状态会漂移。

## 测试清单

| 测试名 | 断言 |
| --- | --- |
| `restore_rebuilds_ownership_unique` | restore 后 owner 表与实体表一致 |
| `restore_duplicate_entity_panics` | 两片快照同 ID → panic |
| `snapshot_preserves_actor_source` | 快照往返 source/actor 一致 |
| `old_format_version_rejected` | format_version 1/未知 → 显式拒绝 |
| `replay_entity_command_after_migration` | 迁移后 Remove/Teleport 重放命中 owner |
| `replay_external_spawn_server_id` | Spawn 日志重放,ID/终态一致 |
| `checkpoint_pending_migration_owner_absent` | 迁移窗口 owner None,不投错片 |

## 完成定义

- 门禁:`cargo test -p shard -p sim-api -p app --all-targets` + clippy;
- 既有 REQ-005 快照/回放测试零回归(格式版本更新除外);
- 不实现 spawn 协议(任务 02)。

## 风险

- 升 `format_version` = 旧 data_dir 不可用;文档提示清库。
- 迁移窗口 owner 缺失要测试，不要用“最后已知 owner”兜底。
- `SnapshotEntityRecord` 字段变更会破旧档;本任务统一收口。
