# 任务 06:双分片迁移端到端集成测试

## 目标

把全链路串起来验证:命令入箱 → 调度器分拍 → worker 执行 → outbox 迁移意图 →
Router 盖章入队 → 下一拍成为目的分片输入前缀 → 实体在邻片重建继续运动。
这是 REQ-001 AC2/AC5 的调度器级前哨战(demo A 结论的 Rust 全链路复现);
外加全链路确定性(AC1 的多线程侧)。

## 必读

- `crates/physics-sim/tests/shard_sim.rs` 的 `entity_migrates_across_boundary`
  与帮助函数(`ground_chunk`/`entity`/`shard_id`;参数与断言风格照此adapt)
- `crates/shard/src/scheduler.rs`(任务 05 产出的 API)
- `docs/tasks/shard/README.md` 决策 7(tick 语义:第 k 次调用执行第 k-1 拍)

## 交付物

`crates/shard/tests/two_shards.rs`。

## 场景设计(固定)

- 2 个 worker;注册 shard(0,0)(x∈[0,128))与 shard(1,0)(x∈[128,256)),
  各自带地面 chunk(直接构造 `ShardSim` 后 `load_chunk`,再 `register`);
- 实体:立方体,出生在 x≈110、地面上方,`linvel = (60, 0, 0)`(≈1 m/tick),
  SpawnDynamic 命令 `apply_at = Tick(1)`;
- `tick()` 循环 31 次(执行第 0..30 拍;实体第 1 拍出生,飞行约 18 拍后过界)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `entity_migrates_across_scheduler_boundary` | 结束时:源片 `entity_state == None`;目的片 `Some`,translation.x > 128 且 < 160;\|linvel.x − 60\| < 2(动量近似保持);`tick_now() == Tick(31)` |
| `migration_flows_through_envelope_prefix` | 逐拍记录 summary:`migrations_admitted` 恰在过界拍为 1,其余拍为 0;实体从源片消失的拍与目的片出现的拍相差恰 1(apply_at ≡ produced_at + 1) |
| `full_pipeline_is_deterministic` | 同场景独立跑两个 Scheduler,同输入同拍数:每拍 TickSummary 序列相等;终态两侧 `entity_state` 位级相等(EntityState 支持 PartialEq) |

## 语义细则

- 实体消失/出现的"拍差 1"用逐拍 `entity_state(源/目的)` 采样判定;
- 两个 Scheduler 的 tick 循环之间不得有共享可变状态;
- 若断言需要浮点比较,用绝对误差 `< 2.0`(物理量),确定性断言必须用 `==`
  (位级),不许对 EntityState 用近似比较。

## 完成定义

见 [README 公共完成定义](README.md)。
