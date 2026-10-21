# 任务 01:①② 拆分(数据 / collider 独立档)

- 状态:待放行。
- 对应 AC:AC-T2(部分:I1、①② 预算独立)、AC-T1(重跑基线,①② 集仍与基线一致)。
- 涉及 §9 白名单:**第 3 条**(`load_chunk` 同时建 collider 的用例改为 ① 后 ②)。
- 涉及 §14 检查项:7(I1 debug 断言)。

## 目标与范围

把现状合一的 chunk 装载拆为两档:① 体素数据(provider IO,驱动拍末执行)、② collider
(内存 ① 的纯函数派生,`ShardSim` 本地构建/拆除)。两档独立半径、独立预算、独立冷却簿记。
**不含票据**(plan 的输入仍是观察者坐标;票据视图改造在 02)。

## 交付物

1. `ActivationConfig` 拆出 ①② 参数;`ActivationController` 双档预算与冷却簿记;
2. `ShardSim`:② 本地派生(构建/拆除/邻居面剔除重建);I1 断言;
3. `Session::apply_chunk_load` 语义改为**只装 ①**(② 随激活刷新自建);
4. `app::Config` 映射与配置校验(`collider_*_radius > data_*_radius` 报错);
5. I2 **简化版**:实体中心所在柱的 ② 存在性断言(完整足迹在 03,见 README §偏离 3)。

## 授权改动

- `crates/physics-sim/src/activation.rs`:config/decisions/controller(详见签名);
- `crates/physics-sim/src/sim.rs`:② 派生、I1/I2 断言、`load_chunk`/`unload_chunk` 调整;
- `crates/physics-sim/src/snapshot.rs`:①② 两套冷却簿记入快照(版本号暂不变,见 README §偏离 4);
- `crates/sim-api/src/session.rs`:`apply_chunk_load` 文档与透传(签名不变);
- `crates/app/src/config.rs`:新键 + 别名 + 校验;
- 测试:`activation.rs` 内联测试(改动明细见【偏离与授权请求】)、`sim.rs` 内联测试、
  `crates/sim-api/tests/`、`crates/app/tests/` 相关用例;
- **不授权**:messaging/shard/viewer。

## 接口设计(签名固定)

```rust
// activation.rs
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ActivationConfig {
    /// ① 数据加载/卸载半径(m)。默认 32/48。
    pub data_load_radius: f32,
    pub data_unload_radius: f32,
    /// ② collider 加载/卸载半径(m)。默认 32/48;校验 ≤ ① 对应值。
    pub collider_load_radius: f32,
    pub collider_unload_radius: f32,
    /// ③(不变)。
    pub awake_load_radius: f32,
    pub awake_unload_radius: f32,
    pub cooldown_ticks: u64,
    pub max_awake_dynamic: usize,
    /// ① 每拍装+卸合计预算。默认 2(继承 swap_budget_chunks_per_tick)。
    pub data_budget_per_tick: usize,
    /// ② 每拍构建预算(拆除不计)。默认 2。
    pub collider_budget_per_tick: usize,
    pub region_margin_chunks: i32,
}
// 旧键别名映射:load_radius/unload_radius → data_* 与 collider_* 同值;
// swap_budget_chunks_per_tick → data_budget_per_tick(app::Config 层)。

#[derive(Debug, Default, PartialEq)]
pub struct ActivationDecisions {
    pub data_to_load: Vec<ChunkPos>,       // ①(原 chunks_to_load)
    pub data_to_unload: Vec<ChunkPos>,     // ①
    pub colliders_to_build: Vec<ChunkPos>, // ②(决策外露仅供测试/遥测;执行在 ShardSim)
    pub entities_to_wake: Vec<EntityId>,   // ③(不变)
    pub entities_to_freeze: Vec<EntityId>, // ③(不变)
}

impl ActivationController {
    // plan 签名不变;内部:
    // - ① 候选:锚点 = 观察者,半径 data_*;预算 data_budget_per_tick;
    //   冷却簿记 chunk_last_swap_data
    // - ② 候选:同锚点,半径 collider_*;预算 collider_budget_per_tick;
    //   冷却簿记 chunk_last_swap_collider(输出 colliders_to_build 供测试/遥测)
    // - ③ 不变
    // 旧字段 chunks_to_load/unload 删除(编译期暴露全部调用点,防止漏改)。
}

// sim.rs
impl ShardSim {
    /// apply_activation 开头(② 派生,偏离说明见 README §偏离 1):
    /// 对每个已装 ① 且未建 ② 且距最近锚点 ≤ collider_load_radius 且冷却已过的
    /// chunk(ChunkPos 升序)构建 ②,预算 collider_budget_per_tick;
    /// 记录 colliders_built 集合(新簿记字段,入快照)。
    /// ① 卸载点同步拆除 ②(I1);邻居 ① 装/卸时已建 ② 的邻 chunk 重建
    /// (expand_boundary 语义,不计预算)。
    /// 随后原 ③ 逻辑不变。
    pub fn has_collider(&self, cp: ChunkPos) -> bool;         // 测试/断言用
    pub fn collider_loaded_chunks(&self) -> BTreeSet<ChunkPos>; // 快照重建用
}
```

## 语义细则

1. **I1(②⊆①,fail-stop)**:`unload_chunk` 拆 ① 时必须同步拆 ②;debug 断言
   任意时刻"有 ② 必有 ①"(在 load/unload/SetBlock 路径出口检查)。
2. **② 构建前提**:① 已装。无 ① 的 chunk 永不建 ②(即使半径内)。
3. **② 重建**:邻居 ① 装/卸、SetBlock/BlockWrite 命中**已建 ②** 的 chunk → 重建该
   ②(面剔除沿用 `neighbors_of`),**不计 ② 预算**、不进冷却簿记(同 SetBlock 现状)。
4. **有 ① 无 ② 的 chunk 收到 SetBlock/BlockWrite**:写体素,不建本 chunk ②,已有
   ② 的邻居照常重建。
5. **预算**:①② 各自独立;② 的"决策即簿记"语义与 ① 一致。
6. **冷却**:两套簿记表互不影响;旧 `chunk_last_swap` 单表拆成两张。
7. **I2 简化版**:醒着 Dynamic 实体中心所在柱有 ① 无 ②(且该 ② 处于待构建状态)时,
   debug 断言放行(容忍一拍构建延迟),但**实体唤醒前置**要求所在柱 ② 已建——现状
   唤醒路径不查 ②,本任务先不加"待地形"状态(03 做),只保证唤醒发生的当拍柱内 ②
   已就绪,否则推迟唤醒(免冷却请求 ①/②,排序暂按距离)。此为 I2 的窄化实现,
   完整足迹与待地形状态在 03。
8. **快照**:② 集合与两套冷却簿记进快照;恢复时按"快照里的 ② 集合"重建 collider,
   不再"凡 ① 皆建 ②"(§4.14 的提前落地;format_version 暂保持 3,旧档自然拒绝)。
9. **`apply_chunk_load`**:语义 = 装 ①;② 由下一拍激活刷新构建(N 决策 → N+1 ① →
   N+1 apply_activation 建 ②,与现状"N+1 同时可用"时序一致)。

## 测试清单

| 测试名(文件::函数) | 断言 | AC |
| --- | --- | --- |
| `activation::i1_unload_data_drops_collider` | 卸 ① 后 `has_collider == false`;再卸同 ① 幂等 | T2 |
| `activation::i1_assertion_catches_orphan_collider` | 测试钩子造"有 ② 无 ①"→ panic | T2 / §14-7 |
| `activation::collider_budget_independent_of_data` | ① 预算 0(不装 ①)时 ② 不建;① 预算 1、② 预算 1 时两档各自推进 | T2 |
| `activation::collider_cooldown_separate_from_data` | ② 构建后立即 SetBlock 重建不受 ② 冷却;重建不计预算 | T2 |
| `activation::setblock_on_data_without_collider` | 有 ① 无 ② chunk 写块:体素生效、本 chunk 不建 ②、邻居 ② 重建 | T2 |
| `sim::wake_requires_footprint_collider_narrow` | 实体所在柱 ② 缺 → 唤醒推迟;② 建后下一拍醒 | I2(窄化) |
| `sim::snapshot_roundtrips_collider_set_and_cooldowns` | ② 集合 + 两套冷却簿记快照往返;恢复后不"凡 ① 皆建 ②" | T2 / §4.14 |
| `e2e/activation.rs::ac_t1_rerun_matches_baseline` | **占位**(02 完成对比断言;本任务先断言场景可跑、①② 分离计数输出) | T1 |
| `app::config::collider_radius_validation` | collider 半径 > data 半径 → 配置错误;别名键映射正确 | T22(部分) |
| 既有 §9-3 迁移:`load_chunk` 合一用例改断言(明细见下) | ① 后 ② 顺序生效 | §9-3 |

## §9 白名单对应

- 第 3 条:`sim.rs`/`shard` 中"`load_chunk` 同时建 collider"断言改为 ① 后 ②;
  涉及测试在提交说明逐个列出。

## 偏离与授权请求(需总架构师放行本任务书时确认)

`activation.rs` 既有测试改动**不在 §9 六条白名单内**,源于本任务的结构改动
(Decisions 字段改名/② 决策新增),请求一并授权(改动 = 机械适配,断言意图不变):

| 测试 | 改动 | 原因 |
| --- | --- | --- |
| `activation_never_loads_outside_region` | `chunks_to_load` → `data_to_load` | 字段改名 |
| `hysteresis_middle_band_holds` | 同上 | 同上 |
| `cooldown_blocks_flip_back` | 同上 + 冷却表拆分后仍单循环 | 同上 |
| `swap_budget_caps_combined_load_unload` | 断言拆两条(① 预算合计不变;② 独立) | 预算拆分 |
| `over_budget_freezes_farthest_immediately` / `overflow_counts_decided_freezes` | 实体档不动,仅构造适配 | 字段改名 |
| `entity_wake_freeze_hysteresis` / `plan_is_bookkeeping_idempotent` / `enforce_budget_picks_farthest` | 同上 | 同上 |

## §14 检查项自查范围

- 7:I1 debug 断言落地(测试 `i1_assertion_catches_orphan_collider`)。
  其余各项不涉及。

## 门禁包

标准三条(workspace 全量)。

## 完成定义

- 测试清单全绿;00 基线场景可跑(对比断言占位);无 messaging/shard/viewer 改动;
- 提交说明含 §9-3 迁移明细 + 上表授权请求对照。

## 风险

- ② 本地派生使 `apply_activation` 变重(0.3–1.4 ms/chunk × 预算 2)——超 shard_alert_ms
  时告警,不改语义;
- 旧 checkpoint 在本任务即开始不兼容(② 集合字段);如实注释,版本统一 06 升。
