# 任务 01:凝固桥 P2′ 位移门 + 演员豁免

## 目标

落地 REQ-001 §9.5 修订(2026-10-17,依据 demos/solidify-slopes):自动凝固
谓词加位移门(挡受驱慢爬行误凝固),演员实体政策层豁免。

## 必读

- `demos/solidify-slopes/README.md`(实验数据:为何 3mm、为何演员豁免必要)
- `crates/physics-sim/src/sim.rs`(step 的凝固检查、EntityRecord、SolidifyConfig)
- `crates/physics-sim/src/snapshot.rs`(EntityRecord 镜像)
- `crates/messaging/src/command.rs`(枚举追加)
- `docs/tasks/req002-world/README.md` 决策 1/2

## 交付物

1. `crates/physics-sim/src/sim.rs` + `snapshot.rs`:位移门 + 演员标记;
2. `crates/messaging/src/command.rs`:`MarkActor` 变体追加(含 ordinal)。

## 接口设计(要点固定)

```rust
// SolidifyConfig 增字段:
/// 自动触发位移门:最近 20 tick 位移 ≤ 此值才允许凝固(0 = 关闭门,兼容开关)。
/// 默认 0.003(3mm;实验:合法落定 d20 ≤1.4mm,受驱爬行 ≥6.3mm)。
pub displacement_gate_m: f32,

// EntityRecord 增:pub(crate) actor: bool(默认 false)
//                 窗口:最近 21 个 translation(VecDeque,滚动)

// SimCommand 追加(声明序最后,ordinal 续):
MarkActor { entity: EntityId, actor: bool },
```

## 语义细则

1. 位移门**同时约束两条自动路径**(P2 计数触发与睡眠兜底触发);
   滚动窗口首尾距离 ≤ displacement_gate_m 才放行;窗口在实体存在期间持续
   滚动(凝固/迁移重建后重置为单点);
2. **演员实体完全豁免自动凝固**(两条路径),仅显式命令将来触发;
   `MarkActor` 触及不存在实体 → panic(fail-stop);
3. 位移门 = 0.0 时视为关闭(直通),便于既有测试与对照;
4. 快照:actor 标记与位移窗口随快照走(既有 idempotence 测试保持绿;
   窗口序列化注意 restore 后行为一致);
5. 爆炸/迁移/激活对演员实体行为不变(豁免只影响凝固)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `gate_blocks_sustained_slow_drift` | 虚空实体 linvel=0.08(<0.1 不睡眠)漂 600 tick:位移门挡住,永不凝固(实体仍在,P2 计数曾达 20) |
| `gate_passes_true_rest` | 真静止 20 tick 照常凝固(门不阻合法);snap 正常 |
| `sleeping_body_freezes_only_after_stop` | linvel=0.03 → 引擎睡眠后窗口位移衰减 → 凝固发生在睡眠 ≥20 tick 后(不再截断滑行,demo 场景 C 语义) |
| `mark_actor_exempts_all_auto_paths` | MarkActor{true} 后真静止 600 tick 不凝固;MarkActor{false} 后恢复凝固 |
| `mark_actor_unknown_entity_panics` | `#[should_panic]` |
| `snapshot_keeps_actor_and_window` | 演员标记 + 窗口随快照往返,行为延续 |
| `gate_disabled_when_zero` | displacement_gate_m=0:慢漂实体照旧凝固(直通开关) |

## 完成定义

见 [README 公共完成定义](README.md)。
