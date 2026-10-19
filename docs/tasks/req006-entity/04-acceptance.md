# 任务 04:AC-E1~E8 验收与独立评审

## 目标

用端到端测试证明“实体所有权 + 外部命令路由 + spawn 分配 + 迁移保真 + 回放”
在真驱动/真网络/真持久层上闭环；不接受单测通过但生产路径漏。

## 必读

- `docs/requirements/REQ-006-entity-authority.md` §7(AC-E1~E8)
- `docs/tasks/req006-entity/README.md`
- `crates/e2e/tests/consistency.rs`(REQ-005 验收风格)
- `crates/e2e/tests/network.rs`(真 TCP)
- `crates/e2e/tests/world_loading.rs`(真驱动/暂停时间)
- 00–03 源码(以最终实现为准)

## 交付物

`crates/e2e/tests/entity_authority.rs`(新;必要时拆子文件,总入口保持一个)。

## 测试清单

| 测试名 | 断言(AC) |
| --- | --- |
| `ace1_ownership_unique_after_spawn_and_migrate` | 任意时刻每 ID 一个 owner;迁移后 owner=dest;重建唯一 |
| `ace2_remove_after_migration_routes_to_owner` | 实体跨片后 Remove 成功,目标片实体消失,日志 dest=owner |
| `ace2_teleport_after_migration_routes_to_owner` | Teleport 后实体在新位置,owner 正确,actor/source 保留 |
| `ace2_wrong_owner_rejected_no_panic` | 错 owner/未知实体/越权 → 拒绝/计数,驱动不 panic |
| `ace3_spawn_assigned_ids_unique` | 真 TCP 两客户端 Spawn → 两个不同 ID;回执到达 |
| `ace3_spawn_unloaded_rejected` | 未加载区域 Spawn → 拒绝,不建空气 chunk |
| `ace4_actor_source_survive_migration` | 迁移/传送后 actor/source 保留;凝固豁免仍生效 |
| `ace5_checkpoint_restore_ownership` | checkpoint/restore 后 ownership 与实体表一致;replay 终态一致 |
| `ace6_player_cannot_command_other_entity` | A 删/传送 B 的实体 → 拒绝;B 可以 |
| `ace7_old_protocol_variants_still_decode` | 旧 C2S/S2C 变体编码不变;新变体只追加 |
| `ace8_workspace_regression` | 全仓 test/clippy/fmt;既有 412+ 测试零回归 |

## 真实性红线

- 真驱动、真 TCP、真 redb(或既有 Shared* 镜像 + 真 ChunkProvider);
- 位级 `==`、精确集合、给定容差;禁止“非空/不 panic/计数>=1”式弱断言;
- 必须制造“实体迁移后再 Remove/Teleport”的场景,不能只测未迁移实体;
- 必须测“玩家 A 不能操作 B 的实体”；
- 外部 Spawn 必须走新协议,不能绕过用 Service 内部命令；
- 如果某条不可构造,如实 escalate + 结构论证,不许删测试/改断言。

## 独立评审

至少覆盖:

1. ownership 唯一性/迁移窗口/source/actor;
2. 外部命令是否还有 `origin_shard` 路由残留;
3. 权限:外部能否删/传送别人的实体;服务端实体是否被外部操作;
4. spawn ID 是否服务端分配、是否可能撞/复用;
5. 快照/恢复/回放 ownership;
6. 协议只追加、旧变体兼容、viewer 适配;
7. 回归与测试质量。

## 完成定义

- `cargo test -p e2e --all-targets` 0;
- `cargo test --workspace`、clippy、fmt 0;
- 独立评审无 medium/high 未解决;架构师终审 AC-E1~E8;
- 通过后 REQ-006 标完成、关闭 #10;不关闭 #8/#9。
