# 任务 04:驱动接线(app)

## 目标

REQ-002 §4.4/§4.5 的 app 侧:分片按需创建、激活请求取数回填、despawn 持久化
屏障、观察者命令入口、世界种子配置。完成后"给种子与观察者,世界自己长出来"。

## 必读

- `docs/requirements/REQ-002-world-loading.md` §4 全部
- `docs/tasks/req002-world/README.md` 决策 6/8/9
- `crates/app/src/driver.rs`(run/bootstrap/退出路径现状)
- 任务 00-03 产出的 API(以源码为准)

## 交付物

`crates/app/src/config.rs` + `driver.rs` + `main.rs` 的增量接线 + 内嵌测试。

## 接口设计(要点固定)

```rust
// config.rs 增:
pub world_seed: types::WorldSeed,   // 默认 WorldSeed(0)
pub world_kind: WorldKind,          // enum { Superflat, SeededScatter },默认 Superflat
// Default/serde(default)/§6 表同步;shard_sim_config 不变。

// driver.rs:
/// DriverCommand 追加:
MoveObserver { position: Vec3 },   // → session.move_observer

/// DriverArgs 增:pub provider: storage::provider::ChunkProvider,
///               pub new_sim: 注册新分片的构造闭包(shard -> ShardSim)
// (bootstrap 装配 provider:worker 通道 + 生成器(按 world_kind)+ seed;
//  new_sim = 闭包捕获 shard_sim_config 与 registry 工厂)

/// DriverOutcome 增:
pub shards_spawned: u64, pub shards_despawned: u64,
pub chunks_loaded: u64, pub chunks_unloaded: u64,

/// tick 末端编排(run 的 tick 分支内,session.tick() 之后,顺序固定):
/// ① observers() → 各观察者所在 shard 未注册 → new_sim(shard) + register;
/// ② take_activation_requests → 装载:provider.get(ChunkKey{world,dim,pos})
///    → apply_chunk_load(shard, pos, chunk);卸载:apply_chunk_unload →
///    累积 (key, chunk) → store.put_chunks(一批 fire-and-forget);
/// ③ take_despawn_requests → 逐个 unload_shard → chunks 并入 put 批 →
///    批后一次 flush() + recv ack(屏障;粘滞错误 panic 沿用);
/// ④ 计数入 outcome。
```

## 语义细则

1. ②中 provider.get 返回 None 的处理:生成器兜底后不应 None——若 None
   (生成器实现缺陷)→ panic fail-stop;
2. 卸载的 chunk 数据在本拍 fire-and-forget 入队,despawn 批后 flush 保证
   屏障;常规卸载依赖终末 flush(与 checkpoint 同窗口语义,§9.11);
3. ChunkKey 的 world/dim 取自分片 ShardId;
4. main.rs:装配 provider 与 new_sim 闭包(registry 工厂 + config.shard_sim_config),
   其余不变;
5. 既有测试(断点续跑/e2e 套件)必须保持绿(向后兼容:不注入观察者时
   编排零动作)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `observer_walk_spawns_shard_and_loads_ground` | MoveObserver 进新区 → 分片创建 → 激活请求 → 地面真装载(provider 真路径),实体落上被支撑 |
| `chunk_unload_persisted_via_worker` | 观察者远离 → 卸载请求 → put_chunks 落库(重开 store 验证) |
| `despawn_flow_with_barrier` | 空片无票到期 → 候选 → unload_shard → put + flush ack → outcome.shards_despawned==1;实体片不 despawn |
| `move_observer_command_routed` | DriverCommand::MoveObserver → session 观察者表更新 |
| `outcome_counters_reflect_activity` | 计数字段与场景一致 |
| `no_observers_no_orchestration` | 无观察者时编排零动作(既有行为不变,回归) |

## 完成定义

见 [README 公共完成定义](README.md)。
