# 任务 02:激活策略机 + ShardSim 接线(AC4 核心)

## 目标

REQ-001 §4.3 第 3 层:单世界激活。**策略纯函数化**(`ActivationController`) +
ShardSim 落实 tier③(冻结/唤醒施体类型),§9.4 超预算冻结、§9.12 冻结体事件
免疫与命令唤醒,全部带不变量测试。

## 必读

- `docs/requirements/REQ-001-architecture.md` §4.3(三档表、滞回、预算、冻结语义)
- `docs/tasks/req001-backlog/README.md` 决策 5-8
- `crates/physics-sim/src/sim.rs`(实体表/step/detonate/spawn_entity/
  snapshot——接线点;`set_body_type` 的 rapier 用法以
  references/rapier 源码为准)
- `crates/shard/src/scheduler.rs`(tick 流程步骤 2 之前的接线位)

## 授权改动

- `crates/physics-sim/src/lib.rs`:`pub mod activation;`
- `crates/shard/src/scheduler.rs`:`set_observers` + tick 内接线(细则 6);
- `crates/physics-sim/src/sim.rs`:新增方法与冻结体跳过逻辑;
- `crates/physics-sim/src/snapshot.rs`:快照结构扩展 activation 簿记。

## 交付物

1. `crates/physics-sim/src/activation.rs`:策略机 + 内嵌不变量测试;
2. sim.rs / snapshot.rs / scheduler.rs 的接线 + 测试。

## 接口设计(签名固定)

```rust
// activation.rs
/// 激活配置(§4.3 三档;核内独立结构,app::Config 注入,无全局单例)。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ActivationConfig {
    pub chunk_load_radius: f32,            // 32
    pub chunk_unload_radius: f32,          // 48
    pub awake_load_radius: f32,            // 24
    pub awake_unload_radius: f32,          // 32
    pub cooldown_ticks: u64,               // 180(3s × 60Hz)
    pub max_awake_dynamic: usize,          // 512(§9.4)
    pub swap_budget_chunks_per_tick: usize, // 2(装+卸合计)
}
// Default 按上列默认值。

/// 一轮激活决策(策略纯输出;执行方 = ShardSim / 调用方)。
#[derive(Debug, Default, PartialEq)]
pub struct ActivationDecisions {
    /// 请求装载的 chunk(按到最近演员距离升序;与卸载合计已截断到预算)。
    pub chunks_to_load: Vec<ChunkPos>,
    /// 请求卸载的 chunk(同上)。
    pub chunks_to_unload: Vec<ChunkPos>,
    pub entities_to_wake: Vec<EntityId>,
    pub entities_to_freeze: Vec<EntityId>,
}

/// 激活策略机:滞回/冷却/预算的簿记与决策(纯内存,serde 可快照)。
#[derive(Serialize, Deserialize)]
pub struct ActivationController { cfg: ActivationConfig,
    chunk_last_swap: BTreeMap<ChunkPos, Tick>,
    entity_last_flip: BTreeMap<EntityId, Tick> }

impl ActivationController {
    pub fn new(cfg: ActivationConfig) -> Self;

    /// 一轮决策(README 决策 5/7):
    /// - chunk:未载 && 最近距离 ≤ load → 装载;已载 && > unload → 卸载;
    ///   两者均需冷却(该 chunk 上次换装 ≥ cooldown_ticks 拍;首见视为可换);
    ///   装载候选 = 各观察者周围 ceil(unload/16)+2 chunk 网格内未载者;
    ///   装卸合并按距离排序后截断到 swap_budget 合计;
    /// - 实体:醒 && > awake_unload && 冷却 → 冻结;冻 && ≤ awake_load && 冷却 → 唤醒;
    /// - 超预算(醒数 > max_awake_dynamic):按到最近演员距离降序立即冻结
    ///   超出者(不受冷却,§9.4)。
    /// 决策即簿记:被采纳的换装/翻转记入冷却表。
    pub fn plan(&mut self, observers: &[Vec3], loaded: &[ChunkPos],
                entities: &[(EntityId, Vec3, bool /*frozen*/)>, now: Tick)
        -> ActivationDecisions;

    /// 预算强制(§9.12 命令唤醒路径):醒数超限时按距离降序给出应冻结者。
    /// 独立于 plan 的冷却(显式命令触及 = 立即生效)。
    pub fn enforce_budget(&self, observers: &[Vec3],
                          awake: &[(EntityId, Vec3)]) -> Vec<EntityId>;
}
```

```rust
// sim.rs 接线(签名固定)
impl ShardSim {
    /// 激活刷新(README 决策 5):tier③ 落实(冻结 = set_body_type
    /// KinematicPositionBased + 清速度;唤醒 = Dynamic + wake_up),
    /// tier①② 以决策返回(chunk 数据源在 REQ-002,本轮调用方可忽略)。
    /// 冻结体跳过:detonate(§9.12 事件免疫)、凝固桥、迁移检测(防御性)。
    pub fn apply_activation(&mut self, observers: &[Vec3]) -> ActivationDecisions;

    pub fn activation_config(&self) -> &ActivationConfig; // 测试/注入用
}
// ShardSimConfig 增加 pub activation: ActivationConfig(Default 默认)。
```

```rust
// scheduler.rs 接线(签名固定)
impl Scheduler {
    /// 设置激活观察者(演员位置;两拍之间调用,下一 tick 生效)。
    pub fn set_observers(&mut self, observers: Vec<Vec3>);
}
// tick() 流程:步骤 2(取前缀)之前,对每分片 apply_activation(忽略返回的
// chunk 请求——数据源属 REQ-002;冻结在本拍 step 生效)。
```

## 语义细则

1. 几何口径照 README 决策 7(chunk 中心水平距离 / 实体 3D / 均取最近演员);
2. `plan` 为**纯决策**:不改施体、不做 IO;同一输入两次调用,第二次因冷却簿记
   已更新而输出空(决策即簿记)——这是有意语义,doc 注明;
3. 冻结时清零线/角速度(§9.4 kinematic 化语义;解冻后从静止继续);
4. 快照扩展:ActivationController(含两张簿记表)入 ShardSimSnapshot——
   既有 `snapshot_bytes_are_idempotent` 必须保持绿;
5. 无观察者(set_observers 空/未设)时 plan 输出:不冻结任何实体?
   **否**——无演员 = 无激活中心,tier③ 全冻结(§4.3"激活中心 = 演员集合";
   但既有测试不调用 set_observers 就不受影响:apply_activation 只在
   tick 接线里调用,而接线仅在观察者**已设置过**时启用(Option 状态,
   未设置 = 行为与现在完全一致)。doc 注明此向后兼容开关。

## 测试清单(activation.rs 内嵌)

| 测试名 | 断言意图 |
| --- | --- |
| `hysteresis_middle_band_holds` | 距离 35(32<35<48):已载不卸、未载不装 |
| `cooldown_blocks_flip_back` | 装载后立刻移远(>48)→ 179 拍内不卸,第 180 拍可卸 |
| `swap_budget_caps_combined_load_unload` | 构造 3 装 + 3 卸候选 → 合计 ≤ 2 且按距离序 |
| `over_budget_freezes_farthest_immediately` | max=2、3 醒实体 → 最远者冻结,不受冷却 |
| `entity_wake_freeze_hysteresis` | 25(24<25<32)保持;>32 冷却后冻;回 ≤24 冷却后醒 |
| `plan_is_bookkeeping_idempotent` | 同输入第二次 plan 输出空(决策即簿记) |
| `enforce_budget_picks_farthest` | 命令唤醒路径的超限淘汰按距离降序 |

sim.rs / scheduler.rs 接线测试:

| 测试名 | 断言意图 |
| --- | --- |
| `frozen_body_ignores_detonate` | apply_activation 冻结远处实体后 Detonate:其状态位级不变;近处实体受冲量 |
| `frozen_body_excluded_from_solidify_and_migration` | 冻结体不凝固、不产生迁移意图(跑 200 拍冒烟) |
| `snapshot_roundtrip_keeps_activation_state` | 冷却簿记随快照往返(装→快照→恢复→冷却窗口仍在) |
| shard:`observers_drive_freeze_via_scheduler` | set_observers 后 tick:远处实体冻结(经 TickSummary 或查询验证) |
| shard:`no_observers_set_behavior_unchanged` | 未设置观察者:tick 行为与现状一致(既有测试即证,补一个显式对照) |

## 完成定义

见 [README 公共完成定义](README.md);门禁包 `-p physics-sim -p shard -p e2e`。
