# REQ-006 实现任务书(总览)

- 状态:**已确认(2026-10-19 用户确认 D1–D5 全 A);任务 00 实现中**。需求 [REQ-006](../../requirements/REQ-006-entity-authority.md)
  与 [决策清单](../../requirements/REQ-006-decisions.md) 已拍板。
- 上游:REQ-001/002/003/005;issue [#10](https://github.com/esp3j0/physics_minecraft/issues/10)。
- 实现顺序:00 → 01 → 02 → 03 → 04,串行;每任务独立门禁。

## 任务表

进度:**00 ✅ 完成;01 ✅ 完成;02 ✅ 完成;03 ✅ 完成;04 ✅ 完成(评审 APPROVE);非阻塞 follow-up #11**。

| # | 任务书 | 内容 | 门禁包 |
| --- | --- | --- | --- |
| 0 | [00-ownership.md](00-ownership.md) | EntityOwnership 表、生命周期 outbox、owner 查询/重建/唯一性 | messaging physics-sim shard ✅ |
| 1 | [01-routing.md](01-routing.md) | 外部实体命令按 owner 路由、权限、actor/传送保真 | sim-api app ✅ |
| 2 | [02-spawn-protocol.md](02-spawn-protocol.md) | 服务端 ID 分配、`C2S::Spawn`/`S2C::Spawned`、viewer 适配 | sim-api app viewer e2e ✅ |
| 3 | [03-snapshot.md](03-snapshot.md) | ownership 重建、快照/回放校验、format_version | shard sim-api app ✅ |
| 4 | [04-acceptance.md](04-acceptance.md) | AC-E1~E8 端到端验收 + 独立评审 | e2e(7 条,B1 修复,APPROVE) |

## 公共完成定义

1. 严格 TDD:先写测试清单 + 最小桩,门禁包见红,再实现到绿;
2. 门禁:`cargo test --all-targets`、`cargo clippy --all-targets -- -D warnings`
   退出码 0;
3. 只改任务书【交付物】与【授权改动】列出的文件;
4. 全仓既有 412+ 测试零回归(除非任务书明确授权更新断言,需说明理由);
5. 提交 `Refs #10`;不关闭 #10;每任务回评后等架构师放行。

## 架构师决策(拍板后不得偏离)

以 [REQ-006-decisions.md](../../requirements/REQ-006-decisions.md) 的最终选择为准。
默认推荐:**D1=A、D2=A、D3=A、D4=A、D5=A**。

## 关键纪律

- **EntityId 全局唯一**:重复 claim/spawn = fail-stop;外部输入不得触发 panic。
- **ownership 是索引，不是真相**:恢复时从各分片实体表重建;不要造第二套状态。
- **实体命令不再用 `origin_shard` 路由**:owner 由 `Session::entity_owner` 解析。
- **迁移窗口**:源片 despawn 后 owner 暂时不存在;外部命令拒绝，不投错片。
- **actor/source 是政策状态**:迁移/传送必须保留;不要塞进物理 desc 混用。
- **协议只追加**:C2S/S2C 变体声明序不变;旧编码保持可解。
- **viewer 只是适配**:`Spawn/Spawned` 让 demo 继续能生成实体;不扩成未来客户端。
