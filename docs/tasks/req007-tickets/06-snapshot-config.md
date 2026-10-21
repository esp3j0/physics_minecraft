# 任务 06:快照收口终稿、I3 不变量、幂等收口、全参数配置、遥测

- 状态:待放行(修订 2:按审核 B5 与 D17 修订)。
- 对应 AC:AC-T19(I3 与幂等全项)、T21(快照/回放)、T22(配置)、AC-T24(开关快照)。
- 涉及 §9 白名单:**第 4 条**(快照格式与 format_version 用例更新到 v4)、
  **第 7 条**(未启用模式用例零改动)。
- 涉及 §14 检查项:7(I3 每拍末校验)、1/2/3/4(复核 02 起的落地)、5、6、8、9(全量自查)。

## 目标与范围

收口三类横切面:(1)**I3 不变量**每拍末校验 + 测试钩子制造违反 → fail-stop;
(2)**快照/回放**把 02–05 分批进入的字段整理收口,`CHECKPOINT_FORMAT_VERSION`
3 → 4,旧档显式拒绝,600 拍中途 checkpoint 恢复重放位级一致;(3)**§4.15 全参数**
经 `app::Config` 注入 + 默认值锚定 + 旧键别名;(4)**遥测**(§4.16)。

## 交付物

1. `shard`:I3 校验器(每拍末,ticket_ops 与终结判定之后);测试钩子
   (`#[cfg(test)]` 或 `pub(crate)` 可见性,制造缺票/错种票/冻结持票);
2. 快照:SchedulerSnapshot/ShardSim 快照字段终稿(**含 D17 开关**——沿用
   `observers_ever_set` 快照位置推广为"曾观察者或区域票";票据表、收件箱、② 集合、
   两套冷却、待地形、静止窗口、pending_terrain_wakes、补偿种子、未消费 ticket_ops);
   `app/driver.rs` 的 `CHECKPOINT_FORMAT_VERSION` 保持 4(**01 已升,B5**);本任务
   收口布局终稿并补齐 v3/v2/v1/v0 拒绝测试;
3. `app::Config`:§4.15 全部参数;默认值锚定测试;配置校验(collider ≤ data、
   `area_ticket_max_*`);别名映射测试;
4. 遥测:TickSummary 扩展(按 §4.16,计数全整数、逐拍重算);
5. D14 补漏复核:常驻体超预算冻结 End ActorMotion(04 已做)、全部操作幂等收口测试。

## 授权改动

- `crates/shard/src/scheduler.rs`(I3/快照/遥测)、`crates/shard/src/tickets.rs`;
- `crates/physics-sim/src/snapshot.rs`、`sim.rs`(遥测输出);
- `crates/app/src/driver.rs`(**仅** CHECKPOINT_FORMAT_VERSION 常量、配置/遥测接线)、
  `crates/app/src/config.rs`、`crates/app/src/main.rs`(如有 Config 透传);
- `crates/sim-api/src/session.rs`(快照透传如有新字段);
- 测试:`app`(driver 快照/v4 用例)、`sim-api`(long_replay 等)、`shard`、`e2e`。

## 接口设计(签名固定)

```rust
// app/driver.rs
pub const CHECKPOINT_FORMAT_VERSION: u32 = 4;   // 3 → 4;旧档显式拒绝

// shard::tickets
impl TicketTable {
    /// I3(§4.3.6):violation → panic(fail-stop)。输入:
    /// - alive_or_in_transit: 存活(实体表)∪ 在途(Router 信封)实体及其 actor;
    /// - frozen_actors: 冻结的常驻体集;
    /// - suspended_carried: 承载静止冻结的瞬态体集(应持 Suspended 票)。
    pub fn check_invariants_i3(
        &self,
        alive_or_in_transit: &BTreeMap<EntityId, bool>,  // entity → actor
        frozen_actors: &BTreeSet<EntityId>,
        suspended_carried: &BTreeSet<EntityId>,
    );
}
// 调用点:scheduler tick 末(应用 ticket_ops 与终结判定之后);每次 tick 恰一次。

// TickSummary 扩展(只追加字段)
pub struct TickSummary {
    // tick / stepped_shards / migrations_admitted(不变)
    pub tickets_active: usize, pub tickets_paused: usize, pub tickets_suspended: usize,
    pub chunks_data_loaded: usize, pub chunks_collider_loaded: usize,
    pub entities_awake: usize, pub entities_awaiting_terrain: usize,
    pub entities_frozen: usize, pub entities_carried: usize,
    pub forced_landings: u64, pub void_removals: u64,
    pub wakes_contact_pred: u64, pub wakes_contact_comp: u64,
    pub wakes_terrain: u64, pub wakes_closure_deferred: u64,
    pub contact_wake_skipped_no_terrain: u64,
    pub actor_motion_capped: u64,
}
```

## 语义细则

1. **I3 四条**(§4.3.6,违反即 panic,消息含实体/票键):
   1. 存活或在途瞬态体恰持一张 TransientMotion;
   2. 实体票持有者 ∈ 存活 ∪ 在途;
   3. 瞬态不持 ActorMotion、常驻不持 TransientMotion;
   4. 冻结常驻体不持 ActorMotion;承载静止冻结的瞬态持 Suspended 票。
   "存活"以 EntityOwnership(REQ-006)为准,"在途"以 Router 待投递迁移信封为准。
2. **幂等收口**:全部操作(Issue/End/Pause/Unpause/Suspend/Resume)在 TicketTable
   层面幂等;02 的单测补齐 Resume/Unpause 分支;同拍同票多操作(不同路径)结果 =
   单次操作。
3. **format v4(B5 收口)**:版本常量已在 01 升 4;本任务收口布局终稿,v3/v2/v1/v0
   拒绝测试补齐(含"v2 三字段旧字节 Decode UnexpectedEnd"的显式验证,防静默);
   01–05 各任务回评均已逐次提醒清 data_dir,本任务回评再汇总提醒一次。
4. **快照终稿**:逐字段核对 §4.14 清单 + **D17 开关**(沿用 `observers_ever_set`
   快照位置;恢复后开关状态保持单向);恢复后"票据表 + ② 集合 + 待地形 + 静止窗口
   + 补偿种子 + 未消费 ticket_ops"与快照前位级一致;回放:区域票命令走日志重放,
   派生票/内部指令/唤醒全部由状态再生;**AC-T21 场景**:四种票 + 全状态 + 600 拍
   中途 checkpoint,恢复重放终态位级一致。
5. **配置(§4.15 全表)**:22 个参数逐一可注入;`App Toml` 键 = 参数名(snake);
   旧键别名:`load_radius/unload_radius → data_* 与 collider_* 同值`、
   `swap_budget_chunks_per_tick → data_budget_per_tick`、`cooldown_secs →
   cooldown_ticks`(换算);校验:collider_* ≤ data_*、area_ticket 上限非负;
   默认值逐一有锚定测试(显式断言数字,不靠 Default 派生)。
6. **遥测**:纯计数、每拍重算、无累加状态(快照不携带);viewer HUD 不接(非目标)。

## 测试清单

| 测试名(文件::函数) | 断言 | AC |
| --- | --- | --- |
| `shard/tickets::i3_transient_without_ticket_panics` | 钩子删存活瞬态体的票 → tick panic | T19/§14-7 |
| `shard/tickets::i3_actor_with_transient_ticket_panics` | 钩子给常驻体塞瞬态票 → panic | T19 |
| `shard/tickets::i3_ticket_to_dead_entity_panics` | 钩子留指向已终结实体的票 → panic | T19 |
| `shard/tickets::i3_frozen_actor_with_motion_ticket_panics` | 冻结常驻持 ActorMotion → panic | T19 |
| `shard/tickets::i3_holds_on_all_e2e_scenarios` | 全部 e2e 场景逐拍不触发(冒烟级,不打断断言) | T19 |
| `shard/tickets::all_ops_idempotent_matrix` | 6 操作 × (已存在/不存在/已在目标态) 全矩阵无 panic、状态/时钟符合预期 | T19 |
| `app::checkpoint_v4_rejects_v3_and_v2` | v3/v2/v1/v0 全拒;v2 旧字节 UnexpectedEnd 非静默 | T21 |
| `app::snapshot_roundtrip_full_ticket_state` | §4.14 全字段往返位级相等 | T21 |
| `e2e/consistency(新增用例)::ac_t21_midway_checkpoint_replay` | 四票全状态场景 600 拍中途 checkpoint → 恢复重放终态位级一致 | T21 |
| `app::config::all_req007_params_injectable` | §4.15 22 参数逐一注入生效 | T22 |
| `app::config::defaults_anchored` | 22 个默认值逐一断言(§4.15 表) | T22 |
| `app::config::legacy_keys_alias_mapping` | 别名键读入映射正确;新旧键同给时新键优先 | T22/§4.15 |
| `app::config::validation_rules` | collider > data 报错等全部校验 | T22 |
| `shard::tick_summary_counts` | 构造场景逐字段核对计数(票据四态、实体四态、落地/虚空、唤醒四源) | §4.16 |
| `shard::activation_enabled_survives_snapshot` | 开关随快照往返:启用前快照恢复后仍未启用;启用后快照恢复后仍启用(单向不回退) | T24 |

## §9 白名单对应

- 第 4 条:快照 format 用例更新 v4(`app`、`sim-api` 中 format_version 相关全部用例),
  提交说明逐个列出。

## §14 检查项自查范围

1–8 全量复核:每项写"满足 / 不涉及"并附证据(测试名或代码位置),进回评。

## 偏离需求示意之处

1. I3 的"冻结常驻体不持 ActorMotion"判定需要调度器知道谁被冻结:由各分片
   TickSummary/实体状态查询汇聚(现状 `entity_states` 已有冻结信息),不新增快照字段;
2. 遥测字段名与 §4.16 一一对应但打平进 TickSummary(需求示意"TickSummary/telemetry"
   二选一,选 TickSummary 零新通道)。

## 门禁包

标准三条(workspace 全量)。

## 完成定义

- 测试清单全绿;布局终稿收口(v4);I3 钩子可制造全部四类违反;AC-T21 位级一致;
- **回评醒目位置汇总提醒:01 起旧 checkpoint(≤ v3)即不可用,需清理 data_dir**。

## 风险

- I3 的"在途"口径:Router 信封属调度器内部状态,校验时点(tick 末)信封已投递或
  仍在队列——以 `pending_shard_destinations()` 已有口径为准,实现时对齐;
- 快照字段终稿若与 02–05 的临时结构有出入,允许本任务内调整中间结构签名
  (都在授权清单内),但**语义不变**。
