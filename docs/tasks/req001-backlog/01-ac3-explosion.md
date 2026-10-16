# 任务 01:AC3 爆炸等价(位级)+ AC8 长程(600 拍)

## 目标

- AC3:"任意爆炸场景,分片各自求值的合并结果与单世界参考**位级一致**";
- AC8 长程:600 拍 + 中途快照(拍 300)+ 重放,终态位级一致(§9.11 的
  checkpoint 周期即 600)。

## 必读

- `docs/tasks/req001-backlog/README.md` 决策 3/4
- `crates/e2e/tests/migration.rs`(任务 00 的参考世界构造,复用其帮助函数)
- `crates/sim-api/tests/replay.rs`(AC8 场景与断言风格;600 拍测试加在这里)
- `crates/messaging/src/event.rs`(CrossEvent::Detonate)

## 交付物

1. `crates/e2e/tests/explosion_equivalence.rs`(AC3);
2. `crates/sim-api/tests/replay.rs` 增补一个 600 拍长程测试(或同 crate 新文件
   `long_replay.rs`,自选并保持风格一致)。

## AC3 场景设计(固定)

- **无地面、无互触**(决策 3:纯弹道,位级等价的前提):8 个立方体悬空,
  分片 (0,0) 4 个(x∈[16,112])、分片 (1,0) 4 个(x∈[144,240]),z=8 一线,
  各间隔 ≥16m(互不接触、互不落同片爆炸半径边缘重叠即可);
- 爆炸:center=(128, 8, 8)(恰在边界上),power/radius 取值照
  `physics-sim/tests/shard_sim.rs::detonate_event_applies_symmetric_impulses` 的量级;
- 分片侧:**同一 CrossEvent::Detonate 以同 apply_at 分别 submit 给两个分片**
  (广播语义);跑 90 拍(冲量+弹道演化);
- 参考侧:裸 PhysicsWorld(同法构造 8 个刚体,同参数),同拍 Detonate 一次性
  施加全部实体,同 90 步。

## 测试清单(AC3)

| 测试名 | 断言意图 |
| --- | --- |
| `ac3_explosion_bitwise_equivalent` | 8 个实体两侧终态 EntityState 位级 `==`(translation/rotation/linvel/angvel 全四向量) |
| `ac3_blast_actually_moved_everyone` | 每个实体参考侧位移 > 1m(非退化保障) |

## AC8 长程场景(sim-api)

| 测试名 | 断言意图 |
| --- | --- |
| `ac8_long_replay_600_ticks_bitwise` | 照既有 AC8 场景(2 分片/出生/穿越/爆炸)拉伸到 600 拍;拍 300 中途快照;原线继续到 600;重放(快照 + tick>300 日志)到 600;终态位级 `==`;运行时长可控(全仓测试预算内,实测报告耗时) |

## 完成定义

见 [README 公共完成定义](README.md);门禁包 `-p e2e -p sim-api`。
