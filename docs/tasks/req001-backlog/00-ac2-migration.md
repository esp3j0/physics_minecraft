# 任务 00:e2e crate + AC2 迁移等价(接触态,<1e-4m)

## 目标

REQ-001 §10 AC2:"实体跨分片迁移前后轨迹与不迁移对照误差 < 1e-4 m(接触态)"。
对照 = 裸 PhysicsWorld 单世界参考(demo A 方法论,README 决策 2):同地面、
同刚体、同输入,**没有所有权边界**的连续路径。

## 必读

- `docs/tasks/req001-backlog/README.md` 决策 1/2
- `crates/physics-sim/src/collider_build.rs`(greedy_colliders/NeighborChunks——
  参考世界的地面 collider 用它构造;若未 pub,授权改 pub)
- `crates/physics-sim/src/sim.rs`(spawn_entity 的刚体参数:friction 0.6/
  restitution 0.1/density 1.0/half 0.5——参考世界必须逐项一致)
- `crates/shard/tests/two_shards.rs`(分片侧场景构造与断言风格)
- `demos/shard-migration/README.md`(demo A 的对照方法论)

## 交付物

1. `crates/e2e/Cargo.toml`(workspace 成员;`crates/*` 自动收录):

   ```toml
   [package]
   name = "e2e"
   version.workspace = true
   edition.workspace = true

   [dependencies]

   [dev-dependencies]
   types = { path = "../types" }
   voxel = { path = "../voxel" }
  messaging = { path = "../messaging" }
   physics-sim = { path = "../physics-sim" }
   shard = { path = "../shard" }
   rapier3d = "0.36.1"

   [lints]
   workspace = true
   ```

   `src/lib.rs` 一行 `//!` 文档(等价性验收套件,README 决策 1)。

2. `crates/e2e/tests/migration.rs`(AC2)。

## 场景设计(固定)

- 走廊 x∈[0,256)、z∈[0,16),整层地面(顶层 world y=64,照 two_shards.rs
  的 ground_chunk 帮助函数风格;凝固桥关闭);
- 实体:0.5 半边立方体,出生 (100, 65.5, 8) 贴地,**linvel=(60,0,0)**
  (接触滑动/翻滚,跨 x=128 边界;SpawnDynamic 命令 apply_at=Tick(1));
- 分片侧:Scheduler 2 分片 (0,0)/(1,0),地面 chunk 各自装载(含扩张带重叠),
  跑 45 拍(实体过界后仍滑行一段);
- 参考侧:裸 PhysicsWorld(gravity/dt 同默认),地面 = 走廊全部 chunk 经
  `greedy_colliders` 生成的 compound(fixed),刚体 = 与 spawn_entity 逐参数
  一致的 dynamic + cuboid,同出生态,同 45 步。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `ac2_contact_crossing_matches_single_world` | 两侧终态 translation 各分量差 < 1e-4;linvel 差 < 1e-3(速度对照,辅助) |
| `ac2_error_far_below_tolerance` | 同场景误差实测上界断言 < 1e-5(防回归恶化;若实测不满足,如实报告并只保留 1e-4 断言,notes 说明) |
| `crossing_actually_happened` | 分片侧逐拍采样证明实体确实迁移(源片 None/目的片 Some,拍差 1)——等价性断言的非退化保障 |

## 完成定义

见 [README 公共完成定义](README.md);门禁包 `-p e2e`。
