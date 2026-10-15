# 任务 04:AC8 回放端到端集成测试

## 目标

把全链路串起来验证 AC8:**中途快照 → 原线继续 → 重放 → 终态位级一致**;
外加 checkpoint 经 storage 落盘再回放(快照 blob 走 CheckpointStore 的真实路径)。
本任务由 qwen 执笔,与实现(glm)交叉。

## 必读

- `docs/tasks/sim-api/README.md` 决策 7/8(AC8 验收形态;原章重放)
- `crates/shard/tests/two_shards.rs`(场景构造风格:ground_chunk/entity/60m/s 穿越)
- `crates/sim-api/src/session.rs`、`replay.rs`、`log.rs`、`snapshot.rs`(以源码为准)
- `crates/storage/src/traits.rs`(CheckpointStore)

## 交付物

`crates/sim-api/tests/replay.rs`。

## 场景设计(固定)

- 2 分片 (0,0)/(1,0),地面 chunk(照 two_shards.rs 帮助函数,凝固桥关闭);
- 实体:x=110 出生,linvel=(60,0,0)(≈1 m/tick,约拍 19 前后过界 → 迁移发生在
  快照之后,重放必须重建信封流);
- 命令时间线(全部经 session.send 并 append 进 CommandLog):
  - tick 1:SpawnDynamic;
  - tick 15:第二个 SpawnDynamic(静止实体,凑双分片多实体);
  - tick 25:SimCommand::Detonate(在两实体中点,让重放含力学期);
- 原线跑到拍 **20** → global_snapshot(迁移尚未发生)→ 原线继续到拍 **40**;
- 重放:replay(snapshot@20, log, 同 SessionConfig, SimpleRegistry 工厂)→
  tick 到拍 40。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `ac8_final_state_is_bitwise_identical_after_replay` | 两线到拍 40:逐分片逐实体 EntityState 位级 `==`(不许近似比较);tick_now 相等;shard_ids 相等 |
| `replay_reproduces_migration_envelope_flow` | 原线与重放线逐拍 tick_now 采样:实体从源片消失/目的片出现的拍完全一致(信封流被忠实重建) |
| `checkpoint_via_storage_roundtrips_into_replay` | snapshot.to_bytes() 经 MemCheckpointStore append/read → from_bytes → replay → 终态仍位级一致(blob 过真实存储路径) |
| `log_bytes_roundtrip_replays_identically` | CommandLog.to_bytes → from_bytes → 用解码后的日志重放 → 终态仍位级一致 |
| `determinism_baseline_two_fresh_runs` | 同场景两条原线(不做快照/重放)终态位级一致(AC8 的对照基线:不稳定的话上面的位级断言无意义) |

工程细节:每条线独立构造(无共享可变状态);位级断言用 `assert_eq!`(EntityState
是 PartialEq);快照/重放之间 registry 都用 `SimpleRegistry::builtin()`。

## 门禁包

`-p sim-api`。

## 完成定义

见 [README 公共完成定义](README.md)。
