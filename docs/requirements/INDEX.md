# 需求索引:REQ ↔ issue ↔ 提交

- 建立日期:2026-10-18
- 用途:仓库内的需求/issue/提交总表;GitHub 上的置顶索引是
  [#8](https://github.com/esp3j0/physics_minecraft/issues/8)。
- 维护规则见 [docs/process.md](../process.md) 的 **阶段 4:GitHub 记录**。

| REQ | 状态 | Issue | 需求文档 | 任务书 | 关键提交 |
| --- | --- | --- | --- | --- | --- |
| REQ-001 | ✅ 完成 | [#3](https://github.com/esp3j0/physics_minecraft/issues/3) | [REQ-001](REQ-001-architecture.md) | [codebase-layout](../design/codebase-layout.md) | [83bb15a](https://github.com/esp3j0/physics_minecraft/commit/83bb15a) 决议;[5cbe443](https://github.com/esp3j0/physics_minecraft/commit/5cbe443) AC1–AC8 |
| REQ-002 | ✅ 完成 | [#4](https://github.com/esp3j0/physics_minecraft/issues/4) | [REQ-002](REQ-002-world-loading.md) | [req002-world](../tasks/req002-world) | [df813c7](https://github.com/esp3j0/physics_minecraft/commit/df813c7) 定稿;[7019ff9](https://github.com/esp3j0/physics_minecraft/commit/7019ff9) 落地 |
| REQ-003 | ✅ 完成 | [#5](https://github.com/esp3j0/physics_minecraft/issues/5) | [REQ-003](REQ-003-network.md) | [req003-net](../tasks/req003-net) | [f03230c](https://github.com/esp3j0/physics_minecraft/commit/f03230c) 定稿;[ea581df](https://github.com/esp3j0/physics_minecraft/commit/ea581df) 落地 |
| REQ-004A | ✅ 完成（ACV4 通过） | [#6](https://github.com/esp3j0/physics_minecraft/issues/6) | [REQ-004A](REQ-004A-visual-viewer.md) | [req004a-viewer](../tasks/req004a-viewer) | [3299ef6](https://github.com/esp3j0/physics_minecraft/commit/3299ef6) 实现;[e0918e2](https://github.com/esp3j0/physics_minecraft/commit/e0918e2)、[8dc911a](https://github.com/esp3j0/physics_minecraft/commit/8dc911a) ACV4 修复 |
| REQ-005 | ✅ 完成 | [#7](https://github.com/esp3j0/physics_minecraft/issues/7) | [REQ-005](REQ-005-consistency.md) | [req005-consistency](../tasks/req005-consistency) | [92fa4da](https://github.com/esp3j0/physics_minecraft/commit/92fa4da) 任务 04 验收;[c3c02df](https://github.com/esp3j0/physics_minecraft/commit/c3c02df) 任务 03;[7050979](https://github.com/esp3j0/physics_minecraft/commit/7050979) 任务 02 |
| REQ-006 | ✅ 完成（评审 APPROVE；#12 已完成） | [#10](https://github.com/esp3j0/physics_minecraft/issues/10) | [REQ-006](REQ-006-entity-authority.md) | [req006-entity](../tasks/req006-entity) | [d39a44c](https://github.com/esp3j0/physics_minecraft/commit/d39a44c) #12 测试收口;[ea298a0](https://github.com/esp3j0/physics_minecraft/commit/ea298a0) #12 高水位;[6dd0488](https://github.com/esp3j0/physics_minecraft/commit/6dd0488) B1 修复;[94c4cda](https://github.com/esp3j0/physics_minecraft/commit/94c4cda) 任务 04;[a927649](https://github.com/esp3j0/physics_minecraft/commit/a927649) 任务 03;[1e1c325](https://github.com/esp3j0/physics_minecraft/commit/1e1c325) 任务 02;[c417691](https://github.com/esp3j0/physics_minecraft/commit/c417691) 任务 01;[bd565ff](https://github.com/esp3j0/physics_minecraft/commit/bd565ff) 任务 00 |

## 说明

- **一个 REQ 一个 issue**:需求、任务书、AC、提交记录都挂在 issue;仓库文档是
  单一事实来源。
- **历史提交补录**:2026-10-18 之前的提交已在对应 issue 正文里补录 commit 链接;
  不改写历史、不 rebase。
- **新提交**:commit message 末尾加 `Refs #<issue>`;PR 用 `Closes #<issue>`。
- **完成**:REQ 文档状态改“已完成” → 关闭 issue(completed)→ 更新本表。

## 设计讨论/草案

- [#10 REQ-006 实体所有权与外部实体命令权威路由(草案)](https://github.com/esp3j0/physics_minecraft/issues/10)
  —— 待 zcode read-only 审计 + 架构师写需求/任务书;未拍板前不实现。
- [#9 未来客户端架构:服务器权威 / 复制 / 预测 / 客户端侧物理](https://github.com/esp3j0/physics_minecraft/issues/9)
  —— 讨论“客户端能否跑 sim 内核 / 只发碰撞事件 / 本地生成物体”;结论收敛后
  再决定是否立客户端 REQ。REQ-005/006 实现不受本讨论阻塞。

