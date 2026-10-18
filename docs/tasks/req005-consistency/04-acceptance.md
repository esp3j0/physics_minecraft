# 任务 04:AC-C1~C9 验收套件与独立评审

## 目标

用端到端测试证明 REQ-005 的三件事真的闭环,而不是"单元测试通过、生产路径
仍然漏"。本任务由 qwen 执笔,与 00–03 的实现交叉;测试是最终契约。

## 必读

- `docs/requirements/REQ-005-consistency.md` §8(AC-C1~C9)
- `docs/tasks/req005-consistency/README.md` 架构师决策
- `crates/e2e/tests/world_loading.rs`(真驱动 + 暂停时间 + 真 redb)
- `crates/e2e/tests/network.rs`(真 TCP + 内嵌客户端)
- `crates/e2e/tests/explosion_equivalence.rs`(单世界对照)
- `crates/e2e/tests/migration.rs`(边界迁移/扩张带)
- `crates/sim-api/tests/long_replay.rs`(600 拍 checkpoint 回放)
- 00–03 的源码(以最终实现为准;任务书的签名是契约,细节以源码为准)

## 交付物

`crates/e2e/tests/consistency.rs`(新;必要时拆 `consistency_chunks.rs` /
`consistency_replay.rs`,但总入口保持一个套件)。

允许的 dev-deps 增补:与 `world_loading.rs`/`network.rs` 同款已有依赖优先;
确需新增(如 `tempfile`/`redb`)在 notes 说明。

## 测试清单(逐条对 AC)

| 测试名 | 断言(AC) |
| --- | --- |
| `acc1_chunk_ownership_region_and_holders` | 两分片 + 一观察者:holder 只出现在 core∪margin;同一 chunk 不会被两片当成本体重复装载;owner 唯一(AC-C1/C9) |
| `acc2_boundary_edit_same_tick` | 边界 SetBlock 后两片该 chunk 逐字节相等;两侧 collider 同拍更新(AC-C2) |
| `acc2_boundary_edit_physics_vs_single_world` | 编辑后的边界上滚球,双片 vs 单世界参考 <1e-4 m(AC-C2) |
| `acc3_owner_only_persistence` | 非 owner 卸载不写库;owner 卸载写库;重载取回编辑;双副本任意卸载顺序无覆盖(AC-C3) |
| `acc4_migration_destination_ensured` | 观察者距边界 <32m,实体跨片:目的片已创建、实体下一拍出现、无 pending 信封(AC-C4) |
| `acc4_no_envelope_black_hole` | 构造目的片未注册的迁移信封;驱动 tick 末 ensure;下一拍 `pending_shard_destinations` 为空(AC-C4) |
| `acc5_checkpoint_pending_migration` | 在 tick N 末有迁移信封时拍 checkpoint;`replay_world` 到 N+2 实体在目的片,终态位级一致(AC-C5) |
| `acc5_checkpoint_observers_and_lifecycle` | 多观察者 + 空闲分片计时中拍 checkpoint;恢复后不发新 Move 仍继续激活;despawn 候选拍不被重置(AC-C5) |
| `acc6_network_observer_replay` | 网络 Move/Remove 入日志;checkpoint 后重放,观察者表/票/激活与原线一致;无幽灵(AC-C6) |
| `acc7_boundary_detonate_broadcast` | 边界 Detonate 经 `Session::detonate`/网络:两片实体都受冲量;合并 vs 单世界位级一致;每片恰一次(AC-C7) |
| `acc7_emergent_event_next_tick` | `Scheduler::broadcast_event` 目标 N+1 生效;origin 不重复;canonical 序(AC-C7) |
| `acc7_blockwrite_across_boundary` | 凝固格跨片:owner/holders 副本一致;owner 持久化;无广播回环(AC-C7/D11) |
| `acc8_workspace_regression_and_viewer_smoke` | `cargo test --workspace` + clippy + fmt 全绿;viewer::client 连真服务器收 Welcome/ChunkData(AC-C8) |
| `acc9_no_duplicate_loads_or_fanout` | 同拍装载计数无重复;事件 fanout = 目标分片数;checkpoint 字节确定(AC-C9) |

> 测试真实性红线:
> - 必须用**真驱动**、**真 redb**(或既有 `SharedCheckpoints`)、**真 TCP**;
>   不许 mock 掉被验收的那一层;
> - 不许把断言弱化为"不 panic/非空";AC-C2/C5/C7 的物理/回放必须位级或
>   给定容差;
> - AC-C5 必须**制造** pending 信封/观察者/生命周期非空的状态,不能挑一个
>   恰好干净的 tick;
> - 如果某条做不到,如实报告并 escalate,不许删测试或改断言。

## 独立评审(qwen)

与实现交叉的只读评审,至少覆盖:

1. **归属/区域**:是否有分片仍能加载区域外 chunk;holder 是否可能重复/
   泄漏;owner 不存在时是否 fail-stop;`margin_meters` 是否真的生效;
2. **副本一致**:外部 SetBlock 是否同拍到所有 holder;非 owner 卸载是否
   真的不写;block write 是否可能回环/重复;
3. **快照完整**:观察者表/Router 信封/sequences/lifecycle 是否都进
   `WorldSnapshot`;holders 重建是否等价;旧 checkpoint 是否明确报错;
4. **观察者回放**:网络 Move/Remove 是否入日志;replay 是否重复/漏;
   `RemoveObserver` 是否可能被客户端注入;断开是否幂等;
5. **事件**:目标集是否覆盖 core∪holder;origin 是否重复;事件是否恰好
   一次;Detonate 参数校验是否挡住 NaN/inf;日志/回放是否一致;
6. **回归**:既有 301+ 测试、viewer 冒烟、clippy/fmt;测试弱化/造假 =
   high。

报告格式:路径:行号 + 证据 + 任务书条款;只报有证据的问题;无问题返回
空数组。

## 完成定义

- `cargo test -p e2e --all-targets` 退出码 0;
- `cargo test --workspace`、`cargo clippy --workspace --all-targets -- -D
  warnings`、`cargo fmt --all --check` 退出码 0;
- 独立评审无 medium/high 未解决项;low 项记录挂账;
- 架构师终审:AC-C1~C9 逐条勾选,`REQ-005` 状态可改 `已完成`。

## 备注

- AC-C8 的 viewer 冒烟只要求"能连、收 Welcome/ChunkData",不要求渲染;
  渲染手动清单归 REQ-004A ACV4;
- 性能只做"无重复/有界"的结构断言,不做墙钟基准(§7 真实长跑校准另行);
- 测试命名保持 `accN_` 前缀,便于按 AC 过滤。
