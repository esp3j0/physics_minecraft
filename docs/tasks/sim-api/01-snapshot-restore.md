# 任务 01:ShardSim 快照/恢复 + 调度器缝(跨 crate)

## 目标

AC8(回放位级一致)的地基:把一个分片的全部确定性状态压成字节、再无损还原。
rapier 0.36 的 `serde-serialize` feature 使 `PhysicsWorld` 整体可序列化
(官方把 pipeline/ccd_solver 等纯工作区 `serde(skip)`,见
references/rapier/src/pipeline/physics_world.rs L64-113),暖启动/岛屿/接触状态
全部随快照走——这是"位级"的关键,不得用"重建刚体"式近似替代。

## 必读

- `crates/physics-sim/src/sim.rs`(ShardSim 全部字段与构造)
- `crates/shard/src/scheduler.rs`(SchedulerConfig/tick_now/register)
- `references/rapier/src/pipeline/physics_world.rs`(serde 派生与 skip 字段)
- `docs/tasks/sim-api/README.md` 决策 2/6

## 授权改动(仅此范围)

1. `crates/physics-sim/Cargo.toml`:rapier3d 加 `features = ["serde-serialize"]`;
   新增 `serde = { version = "1", features = ["derive"] }`;
2. `crates/voxel/src/world.rs`:`VoxelWorld` 补 `Serialize, Deserialize` derive
   (若 `ChunkPos` 尚无 serde derive,一并补上——先查 coords.rs);
3. `crates/shard/src/scheduler.rs`:`SchedulerConfig` 增 `start_tick: Tick` 字段
   (Default 为 `Tick::ZERO`,`new` 用它初始化 coordinator)。

## 交付物

1. `crates/physics-sim/src/sim.rs`(或同 crate 新文件 `snapshot.rs`,自选):
   `ShardSim::snapshot/restore` + `SimSnapshotError`;内嵌测试;
2. `crates/shard/src/scheduler.rs`:`snapshot_shard` 方法 + 测试。

## 接口设计(签名固定)

```rust
// physics-sim
#[derive(Debug, thiserror::Error)]
pub enum SimSnapshotError {
    #[error("快照解码失败:{0}")]
    Decode(String),
}

impl ShardSim {
    /// 全状态快照(确定性字节:bincode standard 配置,字段序固定)。
    /// registry 不入快照;restore 重注入(README 决策 2)。
    pub fn snapshot(&self) -> Vec<u8>;

    /// 从快照恢复。registry 必须与原实例语义一致(SimpleRegistry::builtin 稳定)。
    pub fn restore(bytes: &[u8], registry: Box<dyn BlockRegistry>) -> Result<Self, SimSnapshotError>;
}

// shard
impl Scheduler {
    /// 分片快照(两拍之间调用;sims 归位时)。未注册 → None。
    pub fn snapshot_shard(&self, shard: ShardId) -> Option<Vec<u8>>;
}
```

## 语义细则

1. 快照私有结构含:`cfg`、`voxels`、`phys: &PhysicsWorld`、`entities`
   (RigidBodyHandle 在 rapier serde feature 下可序列化;EntityDesc/slow_ticks 原样)、
   `chunk_bodies`、`outbox`;**registry 不进**;
2. `ShardSimConfig` 需补 `Serialize, Deserialize` derive(gram: gravity 为 glam
   Vec3,types 的 glam 已开 serde);
3. 序列化统一 bincode 2 + `config::standard()`(与 storage 同款);
4. restore 对残缺/错误字节返回 `Err(Decode)`,不得 panic;
5. `snapshot_shard` 只在 sims 归位时可用:tick 进行中不存在该窗口(单线程调用
   约定),doc 注明"两拍之间调用"。

## 测试清单(内嵌各 crate)

| 测试名 | 断言意图 |
| --- | --- |
| `snapshot_restore_then_step_is_bitwise` | 地形+实体跑 10 拍 → 快照 → restore → 两实例再各跑 10 拍 → 逐实体 EntityState 位级 `==` |
| `snapshot_bytes_are_idempotent` | `restore(s).snapshot() == s`(serde 字段序稳定,防引入非确定性字段) |
| `restore_garbage_errors` | `restore(&[0xFF; 8], …)` → `Err(Decode)`,不 panic |
| `snapshot_excludes_registry_but_restores_behavior` | restore 后 SetBlock/凝固行为正常(冒烟:再跑 5 拍不炸) |
| shard:`start_tick_advances_from_given_tick` | `start_tick = Tick(100)` → 首次 tick() 的 summary.tick == Tick(100),tick_now 从 100 走到 101 |
| shard:`snapshot_shard_roundtrip_via_restore` | 注册 sim → snapshot_shard → restore → 再注册可跑;未注册 → None |

## 门禁包

`-p physics-sim -p shard -p voxel`(voxel 既有测试必须保持全绿)。

## 完成定义

见 [README 公共完成定义](README.md)。
