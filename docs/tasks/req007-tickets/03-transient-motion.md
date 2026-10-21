# 任务 03:瞬态运动票、待地形、强制落地、承载、阻塞请求

- 状态:待放行。
- 对应 AC:AC-T3、T4、T5(含阻塞优先)、T6、T7、T8(瞬态半边)、T9、T20;
  AC-T2 的 I2 升级为完整足迹口径。
- 涉及 §9 白名单:**第 1 条(瞬态半边)**(远冻结瞬态用例改期望:不冻结/强制落地/凝固,
  另补常驻对照在 04)、**第 2 条(瞬态半边)**(超预算瞬态 → 强制落地)。
- 涉及 §14 检查项:6(ticket_ops 键序、N 拍产生 N+1 生效)、8(阻塞请求只依实体状态)、
  7(I2 完整口径断言)。

## 目标与范围

落地瞬态体(actor=false)的运动票生命周期:签发/承载暂停挂起/终结;当前/预载两种足迹
(D13);待地形状态(I2 完整执行);强制落地与虚空移除(§4.8);凝固前提修订(§4.7);
阻塞请求优先(D15)。**不含常驻票**(04)、**不含接触唤醒**(05)。

## 交付物

1. `physics-sim`:
   - 足迹计算(当前/预载)、支撑判定(接触法向)、待地形状态机、强制落地(纯体素)、
     虚空移除、凝固前提(无实体支撑)、`Outbox.ticket_ops`;
   - `plan` 的实体档改造:瞬态不再按距离冻结;超预算瞬态 → 强制落地候选;
2. `shard`:ticket_ops 应用(键序,N+1 生效)、实体票到期内部指令
   (`InputSource::SCHEDULER`,不入外部日志)、承载事件的 Pause/Unpause/Suspend;
3. `app`:到期内部指令的投递接线(不盖日志章);`transient_settle_deadline_ticks` 等新配置;
4. 快照:待地形速度、未消费 ticket_ops、强制落地/虚空移除簿记。

## 授权改动

- `crates/physics-sim/src/sim.rs`(状态机/支撑/强制落地/凝固前提/Outbox)、
  `crates/physics-sim/src/activation.rs`(实体档)、
  `crates/physics-sim/src/settle.rs`(新,强制落地 + 足迹,纯函数)、
  `crates/physics-sim/src/snapshot.rs`;
- `crates/shard/src/scheduler.rs`(ticket_ops/内部指令/承载)、`crates/shard/src/tickets.rs`;
- `crates/messaging/src/envelope.rs`(内部指令信封或 outbox 直通,实现定,语义不变);
- `crates/app/src/config.rs` + `driver.rs`(配置、接线);
- `crates/types/src/state.rs`(**仅当**待地形标记需要进 `EntityState` 的旁路结构时;
  默认放 physics-sim 内部记录 + 快照,types 不动);
- 测试各 crate + e2e。

## 接口设计(签名固定)

```rust
// physics-sim::settle.rs(新;纯函数,rapier 类型不出)
/// 当前足迹:AABB(旋转包络 half_size·√3)外扩 terrain_guard_m 的水平 chunk 集。
pub fn current_footprint(pos: Vec3, half_size: f32, guard_m: f32) -> BTreeSet<ChunkPos>;
/// 预载足迹 = 当前足迹 ∪ 前视(水平速度扫掠 lookahead_ticks,截 max_chunks,
/// 按到实体距离升序截断)。只用于装载请求。
pub fn preload_footprint(
    pos: Vec3, half_size: f32, guard_m: f32,
    linvel: Vec3, lookahead_ticks: u64, max_chunks: usize,
) -> BTreeSet<ChunkPos>;
/// 强制落地(§4.8):返回落点格或 None(虚空移除)。
/// 只读该柱体素;overlap 判定由调用方传入其他实体 AABB。
pub fn forced_landing_cell(
    column: &dyn Fn(i32, i32) -> Option<BlockId>,
    x: i32, y: i32, z: i32,
    world_y_min: i32, world_y_max: i32,
    overlaps: &dyn Fn(BlockPos) -> bool,
) -> Option<BlockPos>;

// sim.rs / Outbox(只追加)
pub struct Outbox {
    // ...
    pub ticket_ops: Vec<TicketOp>,        // 分片判定,调度器应用
    pub forced_landings: Vec<(EntityId, BlockPos)>, // 与凝固同型:Emergent BlockWrite + despawn
    pub void_removed: Vec<EntityId>,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TicketOp {   // 只追加;操作幂等由 TicketTable 保证
    IssueTransient,
    End,
    Pause,
    Unpause,
    Suspend,
    Resume,
}
// TicketOp 不带实体号:调度器按 (ShardId, EntityId) 归属;struct 装载:
pub struct TicketOpRecord { pub entity: EntityId, pub op: TicketOp }

// 实体状态标记(sim 内部 + 快照;不进 EntityState)
pub enum LocomotionState { Awake, AwaitingTerrain { saved_linvel: Vec3, saved_angvel: Vec3 } }
```

## 语义细则

1. **瞬态票签发**:新实体 claim(spawn/迁移重建/传送重建)时表中无其票 → 分片产出
   `IssueTransient`(N 拍)→ 调度器 N+1 生效。迁移/传送不产生新票(键不变,时钟继续)。
2. **足迹**:`half_size·√3` 旋转包络 + `terrain_guard_m`(默认 1.0)外扩,水平投影
   chunk 柱。I2 口径:**当前足迹 ∩ 本分片区域(core ∪ margin_ring)内每个 chunk 都有 ②**。
3. **待地形**:
   - 进入:醒着 Dynamic 的当前足迹缺 ②(判定点:apply_activation 与 spawn_entity);
   - 期间:Kinematic、速度保存(位级)、不计醒数、不凝固不迁移、事件免疫、不参与
     接触唤醒(05 落地时同口径)、票据时钟**照常计时**(§4.6);
   - 恢复:足迹 ② 全就绪 → 切回 Dynamic、原样写回速度;恢复属于唤醒语义 → 重置静止判据
     (§4.4.1),但**不产生** Issue(票已存在);
   - 阻塞请求:缺的 ①/② 以阻塞身份请求(免冷却、排序最前、仍占预算)。
4. **支撑判定**:step 后检查接触对;`n·û ≥ support_normal_min`(默认 0.7,û = 重力
   反向);对方静态体 = 地形支撑、实体 = 实体支撑;零重力不判定(既有零重力用例不变,
   §9-6)。被承载 = 有实体支撑。
5. **凝固前提**:现有两路(低速计数 / 睡眠兜底)追加"本拍无实体支撑";零重力场景
   不判定支撑 → 前提恒真(行为不变)。
6. **承载规则**(§4.4.2 表):瞬态被承载:运动票 Pause;静止且不在唤醒范围 → 冻结 +
   Suspend;承载结束 → Unpause;冻结被唤醒 → Resume(仍被承载则 Paused)。
   Suspend 后票不占区域(锚点消失风险:被承载体无独立装载需求,挂靠承载者的票)。
7. **强制落地**:触发 = 票到期(`active_ticks ≥ transient_settle_deadline_ticks`)
   与超预算淘汰(§4.12);前置 = 中心柱 ① 在内存,缺 → 阻塞请求 + 推迟;
   落点算法按 §4.8 五步;产出 = `Emergent BlockWrite + despawned`(复用 REQ-005 广播
   持久化)+ `End`(调度器终结判定自然 End)。
8. **虚空移除**:所有实体 `y < WORLD_Y_MIN − void_despawn_depth_m`(默认 16)→ 删除
   (无 BlockWrite);每拍 step 末判定。
9. **超预算淘汰**(本任务瞬态半边):醒数超限 → 距离降序;瞬态 → 强制落地候选
   (产出同上);§9-2。常驻半边 04。
10. **到期内部指令**:调度器发现实体票到期 → 投 `SCHEDULER` 来源内部指令
    (`apply_at = N+1`,**不进外部命令日志**,回放再生)→ 分片执行强制落地。
11. **阻塞请求(D15)**:三来源(待地形足迹缺 ②;推迟唤醒足迹缺 ①/②——本任务含
    待地形恢复与 I2 窄化唤醒;强制落地柱缺 ①)。请求属性 `(ChunkPos, is_blocked)` 进
    决策输出;排序键 `(is_blocked, priority, dist, ChunkPos)`;卸载候选不得卸任何
    阻塞请求需要的 chunk。
12. **新实体免冷却保护**:翻转簿记不再需要为"新实体"设防——冻结要求静止(§4.4.2),
    新碎片运动中不冻(蓝方块修复)。现状"实体冷却 180 拍"仅约束翻转频率(③ 滞回),
    冻结判定新增"静止 ∧ 常驻/被承载"前置(常驻判定 04 完整落地;本任务瞬态体从
    滞回冻结路径整体移除)。
13. **快照**:LocomotionState(待地形速度)、未消费 ticket_ops、强制落地/虚空计数;
    待处理的"阻塞中的柱"不入快照(每拍由状态重算)。

## 测试清单

| 测试名(文件::函数) | 断言 | AC |
| --- | --- | --- |
| `settle::footprint_current_and_preload` | 包络/guard/前视的 chunk 集正确;前视截断按距离;前视不进 I2 | T6 |
| `settle::forced_landing_column_scan` | 实心向上/悬空向下/到 WORLD_Y_MIN 虚空/重叠让位 | T7 |
| `settle::forced_landing_requires_column_data` | 柱 ① 缺 → 返回"推迟"(调用方产阻塞请求) | D15 |
| `sim::transient_keeps_ticket_while_moving` | 抛出瞬态体(落点 > 60 m,无观察者):全程醒着、票 Active、active_ticks 递增 | T3 |
| `sim::transient_lands_solidifies_ticket_ends` | 落地凝固:Emergent BlockWrite + despawned + 票 End;分片可 despawn | T3 |
| `sim::transient_not_frozen_when_far`(§9-1 迁移) | 远处运动瞬态体不被滞回冻结(原 frozen_body_excluded... 类用例翻转期望或改常驻) | T3/§9-1 |
| `sim::awaiting_terrain_enters_and_recovers` | ② 预算 0 → 待地形(Kinematic、位置不变、事件免疫);② 就绪 → 醒、速度位级写回 | T5 |
| `sim::support_detection_normal_and_entity` | 法向 0.7 边界;静态体/实体支撑区分;零重力恒无支撑 | T9/§9-6 |
| `sim::solidify_requires_no_entity_support` | 压在常驻体上的瞬态不凝固(常驻走开前);落地后凝固(常驻对照 04 补) | T9/§4.7 |
| `sim::carried_ticket_pause_suspend_resume` | 承载:Pause → 静止冻结 Suspend → 唤醒 Resume | T9 |
| `sim::carried_clock_paused_no_forced_landing` | 被承载超 deadline 不强制落地(active_ticks 不增) | T9 |
| `sim::over_budget_transient_forced_landing` | 预算超限瞬态 → 强制落地(非冻结);顺序距离降序 | T8/§9-2 |
| `sim::void_removal_all_entities` | y < 阈值删除;常驻同样生效 | T7 |
| `sim::ticket_deadline_forced_landing` | 关凝固零重力漂移 → 到期强制落地 | T7 |
| `shard/tickets::ticket_ops_applied_in_key_order_n_plus_1` | 同拍多 ops 按键序;N 产生 N+1 生效(中间拍查询可见时序) | §14-6 |
| `shard/scheduler::deadline_internal_command_not_logged` | 到期指令 SCHEDULER 来源、apply_at N+1、外部命令日志无记录;回放再生一致 | §4.3.2 |
| `shard/scheduler::migration_keeps_ticket_clock` | 迁移在途票保留、active_ticks 连续;目的 claim 后不重签(幂等) | T4 |
| `shard/scheduler::solidify_end_vs_migrate_keep` | 同拍凝固 despawn → End;迁移 despawn → 保留 | T19(前半,提前落地) |
| `e2e/activation_tickets::ac_t3_no_hanging_blue_blocks` | 观察者原点、瞬态抛 > 60 m:不冻结、落点成块、表清、观察者移除后分片 despawn | T3 |
| `e2e/activation_tickets::ac_t4_cross_shard_landing` | 抛进未注册分片:目的分片 ensure、目的分片凝固、active_ticks 连续 | T4 |
| `e2e/activation_tickets::ac_t5_awaiting_terrain_and_block_priority` | T5 全流程 + 阻塞优先:② 预算恢复 1、同拍有观察者装载候选 → 先装待地形缺的 chunk | T5/D15 |
| `e2e/activation_tickets::ac_t6_lookahead_not_awaiting` | 前视缺、当前足迹齐 → 不待地形;前视跨片由 owner 装载、不触发待地形 | T6 |
| `e2e/activation_tickets::ac_t7_deadline_and_void` | 截止拍强制落点正确;无支撑柱 → 虚空;坠落越线 → 立即移除 | T7 |
| `e2e/activation_tickets::ac_t9_carry_full_arc` | 方块落冻结常驻体:不凝固 → 冻结 Suspended → 唤醒同组 → 推走 → 下落凝固;重叠让位 | T9 |
| `e2e/activation_tickets::ac_t20_lifecycle` | 运动票/区域票保活;Suspended 不保活;票清实体终 → idle despawn | T20 |

## §9 白名单对应

- 第 1 条(瞬态半边):`frozen_body_excluded_from_solidify_and_migration`、
  `event_envelope_detonates_locally_frozen_immune`、`e2e/activation.rs` 远冻结瞬态用例
  → 改期望为"不冻结 / 到期落地";常驻对照保留到 04 迁移;
- 第 2 条(瞬态半边):超预算瞬态用例改强制落地;
- 第 6 条:零重力凝固用例断言**不改动**(支撑前提在零重力下恒真,作为回归锚)。

## §14 检查项自查范围

- 6:ticket_ops 键序 + N+1 生效(测试见清单);
- 7:I2 完整口径 debug 断言;
- 8:阻塞请求只依实体状态(`settle::` 纯函数无票种参数)。

## 偏离需求示意之处

1. `TicketOp` 不带实体号、由 `TicketOpRecord` 包装:调度器需 (ShardId, EntityId) 路由,
   比需求"Outbox.ticket_ops"示意多一层归属结构,语义一致;
2. 蓝方块修复在 12 条细化了"新实体免冷却"的机制口径:现状的实体翻转冷却保留为
   滞回频率约束,冻结新增静止前置——需求 §5 修订行"冻结只在静止时"的直接落实,
   需求未明说旧簿记去留;
3. 虚空移除放在本任务(需求 §4.8 末段"对所有实体生效",实现上与强制落地同文件同拍点)。

## 门禁包

标准三条(workspace 全量)。

## 完成定义

- 测试清单全绿;蓝方块场景(T3)在 e2e 可复现修复;既有零重力用例零改动通过;
- 提交说明含 §9-1/2(瞬态半边)迁移明细。

## 风险

- 支撑判定依赖 rapier 接触对查询(kinematic–static 也有接触对;实现时验证,不足则用
  `contact_wake_skin_m` 外扩 AABB 的几何兜底);
- 强制落地与迁移窗口的交互(实体在途时到期):在途时钟继续、落地指令按 owner 分片
  投递——claim 后到达才执行,测试覆盖。
