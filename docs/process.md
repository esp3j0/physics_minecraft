# 开发流程:需求驱动 + 测试驱动 + demo 驱动

本项目把三种开发模型组合成一条流水线,任何功能开发都按它执行:

```
确认需求 ──► 写需求文档 ──► Python demo 验证 ──► 发现问题?回写需求文档
                                        │ 没问题
                                        ▼
                          验收标准写成测试 ──► Rust 正式实现 ──► 测试全绿 = 完成
```

## 阶段 1:需求(需求驱动)

- 每个功能先在 `docs/requirements/` 写详细需求文档,命名 `REQ-<三位编号>-<短名>.md`,内容用 [TEMPLATE.md](requirements/TEMPLATE.md)
- 需求文档必须包含**逐条可验证的验收标准**——每一条都要能对应一个测试
- 状态字段流转:`草案 → 已确认 → 开发中 → 已完成`(或 `已搁置`)
- 需求未经确认,不写正式代码

## 阶段 2:Demo(demo 驱动)

- 在 `demos/<主题>/` 用 Python 快速实现核心算法/猜想,目的是**低成本发现需求与设计问题**
- demo 允许脏、允许硬编码:它验证的是"需求是否成立、算法是否可行",不是代码质量
- demo 过程中发现的问题回写需求文档(修订需求、补充边界条件),必要时重走阶段 1

## 阶段 3:实现(测试驱动)

- 先把需求文档的验收标准逐条写成 Rust 单元测试(先写测试,跑红)
- 在 `crates/` 对应模块实现,让测试转绿;demo 验证过的算法直接照搬思路
- 全部验收测试通过 + `cargo test --workspace` 全绿,需求文档状态改为 `已完成`

## 目录约定

| 目录 | 用途 | 语言 |
| --- | --- | --- |
| `docs/requirements/` | 需求文档(单一事实来源) | Markdown |
| `demos/` | 验证性 demo,一个主题一个子目录 | Python(辅助) |
| `crates/` | 正式项目代码 | Rust |

## 阶段 4:GitHub 记录(2026-10-18 起)

文档和代码之外,GitHub issue 是**需求进度的公开账本**;仓库文档仍是单一事实来源,
issue 只做索引/状态/提交关联,不复制需求正文。

1. **一个 REQ 一个 issue**:issue 标题 = REQ 标题;打 `req-XXX` 标签,挂同名
   milestone;正文写状态、需求/任务书链接、AC、提交记录。
2. **提交关联**:实现提交的 message 末尾加 `Refs #<issue>`;PR 用
   `Closes #<issue>`。GitHub 自动在 issue 产生 cross-reference。
3. **历史补录**:已经发生的提交不改写/rebase;在对应 issue 正文里补录 commit
   链接(短 SHA + 一行说明)。
4. **完成**:REQ 文档状态改 `已完成` → 任务书收尾 → 关闭 issue(reason:
   completed)→ 更新 [docs/requirements/INDEX.md](requirements/INDEX.md)。
5. **索引**:置顶 issue
   [#8](https://github.com/esp3j0/physics_minecraft/issues/8) 与
   [INDEX.md](requirements/INDEX.md) 保持同一张表;新 REQ 先建 issue 再写文档。
6. **挂账/后续候选**:不混入当前 REQ 的 issue;单独建 issue 或在 REQ 文档
   §7/§9 明确"后续",候选不等于承诺。

