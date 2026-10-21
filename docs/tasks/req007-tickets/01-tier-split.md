# 任务 01:①② 拆分(数据 / collider 独立档)

- 状态:待放行(修订 2:按审核 B2/B3/B5/S4 修订)。
- 对应 AC:AC-T2(部分:I1、①② 预算独立)、AC-T1(**四项严格相等比对**,B4.4)、
  AC-T24(部分:D17 开关分支)。
- 涉及 §9 白名单:**第 3 条**(`load_chunk` 同时建 collider 的用例改为 ① 后 ②;仅限
  激活**已启用**路径;未启用路径按 §9-7 零改动)、**第 7 条**(未启用模式用例零改动)。
- 涉及 §14 检查项:7(I1 debug 断言)、9(未启用行为与 `8702670` 一致)。

## 目标与范围

把现状合一的 chunk 装载拆为两档:① 体素数据(provider IO,驱动拍末执行)、② collider
(内存 ① 的纯函数派生)。两档独立半径、独立预算、独立冷却簿记。**plan 是 ② 的唯一
决策者**(B3)。落地 D17 激活启用开关的装载分支。**不含票据**(plan 的锚点输入仍是
观察者坐标;票据视图改造在 02)、**不含 I2/待地形**(S4:03/04)。

## 交付物

1. `ActivationConfig` 拆出 ①② 参数;`ActivationController` 双档预算与冷却簿记;
   `ActivationDecisions` 新增 ② 决策字段(**不改名既有字段**,B2);
2. `ShardSim::apply_activation`:拿 ② 决策当场执行(构建/拆除/邻居面剔除重建);I1 断言;
3. `Session::apply_chunk_load`:启用时只装 ①(② 随激活决策构建);未启用时合一(现状);
4. D17 开关:调度器"曾设观察者或区域票"单向启用;未启用时全部行为与 `8702670` 一致;
5. `CHECKPOINT_FORMAT_VERSION` **3 → 4(B5,本任务起)**;
6. `app::Config` 映射与配置校验(`collider_*_radius > data_*_radius` 报错);
7. 基线四项比对测试(见测试清单)。

## 授权改动

- `crates/physics-sim/src/activation.rs`:config/decisions/controller(详见签名);
- `crates/physics-sim/src/sim.rs`:② 执行、I1 断言、`load_chunk`/`unload_chunk` 调整、
  D17 开关的 sim 侧分支;
- `crates/physics-sim/src/snapshot.rs`:①② 两套冷却簿记、② 集合、开关状态入快照;
- `crates/sim-api/src/session.rs`:`apply_chunk_load` 文档与透传(签名不变);
- `crates/shard/src/scheduler.rs`:**仅** D17 开关字段(`observed_or_ticketed`,推广
  现有 `observers_ever_set`)、激活请求的开关透传、`unload_shard`/`apply_chunk_load`
  的合一分支(未启用路径);既有测试**零改动**(§9-7);
- `crates/app/src/config.rs`:新键 + 别名 + 校验;
- `crates/app/src/driver.rs`:**仅** `CHECKPOINT_FORMAT_VERSION` 常量与 ① 决策字段名
  适配(**无字段改名,应为零适配**;若编译器证明有遗漏调用点,在回评说明);
- `crates/e2e/tests/activation_baseline.rs`:四项比对落地(B4.4);
- 测试:`activation.rs` 内联(仅批准的 1 条)、`sim.rs` 内联、`crates/app/tests/` v4 用例;
- **不授权**:messaging/viewer;`e2e/tests/activation.rs` 等 §9-7 用例不授权。

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
// 旧键别名映射(app::Config 层):load_radius/unload_radius → data_* 与 collider_* 同值;
// swap_budget_chunks_per_tick → data_budget_per_tick。

#[derive(Debug, Default, PartialEq)]
pub struct ActivationDecisions {
    /// ① 装载/卸载(**保留原名**,B2;语义 = ①,更新文档注释)。
    pub chunks_to_load: Vec<ChunkPos>,
    pub chunks_to_unload: Vec<ChunkPos>,
    /// ② 构建/拆除决策(B3:plan 是唯一决策者;半径/滞回/预算/冷却全在 plan)。
    pub colliders_to_build: Vec<ChunkPos>,
    pub colliders_to_drop: Vec<ChunkPos>,
    /// ③(不变)。
    pub entities_to_wake: Vec<EntityId>,
    pub entities_to_freeze: Vec<EntityId>,
}

impl ActivationController {
    // plan 签名不变(02 才改票据视图)。内部:
    // - ① 候选:锚点 = 观察者,半径 data_*;预算 data_budget_per_tick;
    //   冷却簿记 chunk_last_swap_data
    // - ② 候选:同锚点,半径 collider_*;构建前提 = ① 已载(输入 loaded 集合判定);
    //   预算 collider_budget_per_tick(拆除不计);冷却簿记 chunk_last_swap_collider;
    //   拆除候选 = 已建 ② ∧ 距离 > collider_unload_radius(B3 补漏:两档半径可不同)
    // - ③ 不变
}
```

```rust
// sim.rs
impl ShardSim {
    /// apply_activation(observers, decisions):拿 ② 决策当场执行(B3,与冻结/唤醒
    /// 同方式;驱动不执行 ②):
    /// - build:按 decisions.colliders_to_build 顺序建 ②(体素 → compound,面剔除
    ///   沿用 neighbors_of);
    /// - drop:按 decisions.colliders_to_drop 拆 ②;
    /// - 邻居 ① 装/卸、SetBlock/BlockWrite 命中已建 ② → 重建该 ②(不计预算、不进冷却);
    /// - I1 断言(见细则 1)。
    /// 未启用(D17)时本函数整体跳过(激活不生效,现状语义)。
    pub fn has_collider(&self, cp: ChunkPos) -> bool;           // 测试/断言用
    pub fn collider_loaded_chunks(&self) -> BTreeSet<ChunkPos>; // 快照重建用
}

// shard/src/scheduler.rs(D17 开关)
impl Scheduler {
    /// 推广现 observers_ever_set:曾设观察者或区域票(02 接区域票)即启用。
    /// 单向;随快照保存(沿用现快照位置)。未启用 → tick 的激活段整体跳过。
    pub fn activation_enabled(&self) -> bool;
}
```

```rust
// app/driver.rs
pub const CHECKPOINT_FORMAT_VERSION: u32 = 4;   // 3 → 4(B5,01 起)
```

## 语义细则

1. **I1(②⊆①,fail-stop)**:`unload_chunk` 拆 ① 时必须同步拆 ②(启用模式);
   debug 断言任意时刻"有 ② 必有 ①"(load/unload/SetBlock 路径出口检查)。
   未启用模式无独立 ② 状态,断言天然成立。
2. **② 构建前提**:① 已载。无 ① 的 chunk 永不建 ②(即使半径内)——plan 在候选阶段
   过滤。
3. **② 拆除(B3 补漏)**:距离 > `collider_unload_radius` 且 ① 仍在 → 拆 ② 留 ①
   (有 ① 无 ② 状态);① 卸载必同步拆 ②(I1)。默认参数两档半径相同,此路径默认
   触不到,由专用测试覆盖。
4. **② 重建**:邻居 ① 装/卸、SetBlock/BlockWrite 命中已建 ② 的 chunk → 重建该 ②,
   不计 ② 预算、不进冷却簿记。
5. **有 ① 无 ② 的 chunk 收到 SetBlock/BlockWrite**:写体素,不建本 chunk ②,已有
   ② 的邻居照常重建。
6. **预算**:①② 各自独立;"决策即簿记"语义两档一致。
7. **冷却**:两套簿记互不影响。
8. **D17 开关**:
   - 启用 = 曾设观察者或区域票;单向;随快照(沿用 `observers_ever_set` 位置);
   - **未启用**:`load_chunk`/`apply_chunk_load` 合一建①②(现状);激活跳过;无 ②
     独立簿记;行为与 `8702670` 位级一致(§9-7 用例零改动通过);
   - **启用当拍**(bootstrap,03 完成补签票据;本任务先落开关与装载分支):已建
     碰撞体的 chunk 即初始 ② 集合;
   - 本任务 `activation_enabled()` 只反映观察者侧;区域票并入在 02。
9. **I2 不在本任务(S4)**:不做实体唤醒的 ② 前置检查、不做待地形、不做阻塞请求;
   基线比对因此不允许出现"推迟唤醒"差异。
10. **快照**:② 集合、两套冷却簿记、开关状态入快照;恢复时按快照 ② 集合重建
    collider(不再"凡 ① 皆建 ②");`CHECKPOINT_FORMAT_VERSION = 4`(B5)。
11. **`apply_chunk_load`(启用)**:装 ①;② 由下一拍激活决策构建(N 决策 → N+1 ① →
    N+1 apply_activation 建 ②,与现状"N+1 同时可用"时序一致)。
12. **字段不改名(B2)**:`chunks_to_load/unload` 保留,语义 = ①;所有既有调用点
    (driver.rs、scheduler.rs 测试、e2e 三套件)零改动。

## 测试清单

| 测试名(文件::函数) | 断言 | AC |
| --- | --- | --- |
| `activation::i1_unload_data_drops_collider` | 启用模式:卸 ① 后 `has_collider == false`;同 ① 再卸幂等 | T2 |
| `activation::i1_assertion_catches_orphan_collider` | 测试钩子造"有 ② 无 ①"→ panic | T2 / §14-7 |
| `activation::collider_drop_before_data_drop` | 两档半径不同(② 卸 32 < ① 卸 48):32–48 m 环带内 ② 先拆、① 保留(有①无②) | B3 补漏 |
| `activation::collider_budget_independent_of_data` | ① 预算 0 时 ② 不建(候选过滤);①② 预算各 1 时两档独立推进 | T2 |
| `activation::collider_cooldown_separate_from_data` | ② 构建后立即 SetBlock 重建不受 ② 冷却;重建不计预算 | T2 |
| `activation::setblock_on_data_without_collider` | 有 ① 无 ② chunk 写块:体素生效、本 chunk 不建 ②、邻居 ② 重建 | T2 |
| `sim::apply_activation_executes_collider_decisions` | 决策进、collider 出:build/drop 当场生效(单点执行,无二次派生) | B3 |
| `sim::load_chunk_legacy_mode_unified`(§9-7 锚) | 未启用:load 后 ①② 同时具备,与旧行为一致 | T24/§9-7 |
| `sim::activation_enabled_flag_semantics` | 未设观察者 → false;设置一次 → 永久 true(单向);快照往返保持 | T24 |
| `sim::snapshot_roundtrips_collider_set_and_cooldowns` | ② 集合 + 两套冷却 + 开关快照往返;恢复后不"凡 ① 皆建 ②" | T2/§4.14 |
| `e2e/activation_baseline::ac_t1_matches_golden_after_tier_split` | **四项严格相等(B4.4)**:① 集 == golden chunk 集;② 集 == golden chunk 集;冻结/唤醒集 == golden;分片集 == golden;零"推迟唤醒"差异 | T1 |
| `app::config::collider_radius_validation` | collider 半径 > data 半径 → 配置错误;别名键映射正确 | T22(部分) |
| `app::checkpoint_v4_rejects_v3` | v3 旧档显式拒绝(v2/v1/v0 拒绝测试在 06 补齐) | T21(部分) |
| 既有 §9-3 迁移:`load_chunk` 合一用例(启用路径)改断言 | ① 后 ② 顺序生效 | §9-3 |

## §9 白名单对应

- 第 3 条:启用路径的"`load_chunk` 同时建 collider"断言改为 ① 后 ②;涉及测试在
  提交说明逐个列出;
- 第 7 条:未启用模式用例(shard_sim.rs、two_shards.rs、migration.rs、replay.rs、
  app_e2e 等)**零改动**,以其全绿作为本条的证据。

## 偏离与授权请求(无)

- B2 已裁决:字段不改名,原"8 测试机械适配"授权请求撤回;仅
  `swap_budget_caps_combined_load_unload` 因 ①② 预算拆分调整断言(总架构师已批准):
  断言拆为"① 合计预算语义不变"+"② 独立预算",在提交说明对应此条批准。

## §14 检查项自查范围

- 7:I1 debug 断言落地;9:未启用行为 == `8702670`(§9-7 用例零改动 + 
  `load_chunk_legacy_mode_unified` 显式锚定)。其余各项不涉及。

## 门禁包

标准三条(workspace 全量)。

## 完成定义

- 测试清单全绿;基线四项比对绿(允许的机械适配仅限比对测试自身);无 messaging/viewer
  改动;§9-7 用例零改动;
- 提交说明含 §9-3 迁移明细 + 批准的 1 条断言调整对照;
- **回评醒目提醒:checkpoint 布局已变、版本已升 4,旧存档(v3)不可用,需清 data_dir**。

## 风险

- ② 决策进 `apply_activation` 后该函数变重(0.3–1.4 ms/chunk × 预算 2)——超
  shard_alert_ms 时告警,不改语义;
- 未启用分支若与旧行为有任何位级偏差,§9-7 用例会立即暴露——这是验收手段,不是风险;
- 快照布局在 02–05 还会变:版本号保持 4 不再逐任务升,每任务回评提醒清 data_dir。
