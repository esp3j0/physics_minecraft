# 任务 04:常驻运动票、统一唤醒、地形变更唤醒、超预算常驻冻结

- 状态:待放行。
- 对应 AC:AC-T10、T11、T12(支撑闭包)、T13;AC-T8(常驻半边);§9-1 常驻半边迁移。
- 涉及 §9 白名单:**第 1 条(常驻半边)**(远冻结常驻用例:改 actor=true 且先静止,
  保留原意图;瞬态对照已在 03)、**第 2 条(常驻半边)**(超预算常驻 → 冻结)、
  **第 7 条**(未启用模式用例零改动;新测试"无观察者"场景先启用激活,S6)。
- 涉及 §14 检查项:7(I2/I1 持续)、8(阻塞请求)、6(内部指令时序)。

## 目标与范围

落地常驻体(actor=true)运动票与统一唤醒规则(§4.9):五种来源中的三种在本任务接通
(观察者/区域票半径唤醒已有;**地形变更唤醒**新增;**命令唤醒**改走统一前置);
**向上支撑闭包**整组原子唤醒;静止判据与重置;超预算常驻冻结(+ `End ActorMotion`,
D14 补漏)。**不含接触唤醒**(05)。

## 交付物

1. `physics-sim`:
   - 静止判据(低速连续 + 位移门 / rapier 睡眠)与**唤醒重置**;
   - 常驻票签发/终结(§4.3.4 常驻各行);
   - 统一唤醒执行器:前置(足迹 ② 齐,否则推迟 + 阻塞请求)、支撑闭包成组、
     整组原子、重置静止、翻转簿记、票据签发/恢复;
   - 地形变更唤醒:体素写入登记 → 下一拍唤醒 AABB 相交冻结体;
   - 超预算路径:常驻 → 冻结 + End ActorMotion;瞬态 → 强制落地(03 已有);
2. `shard`:到期常驻票内部指令(撤票);预算淘汰的 op 产出;
3. `app`:新配置键(`actor_motion_max_ticks`、`support_normal_min`、
   `contact_wake_skin_m` 的地形变更共用、`void_despawn_depth_m` 已在 03);
4. 快照:静止判据窗口、地形变更唤醒格登记、补偿无关(05)。

## 授权改动

- `crates/physics-sim/src/sim.rs`(静止/闭包/地形变更/预算)、`activation.rs`(③ 档)、
  `crates/physics-sim/src/wake.rs`(新,统一唤醒 + 闭包,纯函数)、`snapshot.rs`;
- `crates/shard/src/scheduler.rs`、`crates/shard/src/tickets.rs`;
- `crates/app/src/config.rs`、`driver.rs`;
- 测试各 crate + e2e。

## 接口设计(签名固定)

```rust
// physics-sim::wake.rs(新;纯函数)
/// 向上支撑闭包:seeds 之上、贴靠(间距 ≤ skin_m)且中心更高的冻结实体,递归并入。
/// 邻接判定 = 旋转包络 AABB 外扩 skin_m 相交(几何,不依赖 rapier 接触对)。
pub fn upward_closure(
    frozen: &[(EntityId, Vec3, Vec3)],        // (id, center, half_extent)
    seeds: &[EntityId],
    skin_m: f32,
) -> Vec<EntityId>;                          // EntityId 升序;含 seeds

/// 地形变更唤醒:改写格 c 与哪些冻结体 AABB(外扩 skin_m)相交。
pub fn frozen_intersecting_cell(
    frozen: &[(EntityId, Vec3, Vec3)],
    c: BlockPos,
    skin_m: f32,
) -> Vec<EntityId>;

// sim.rs
pub struct ShardSim {
    // ...
    pending_terrain_wakes: BTreeSet<BlockPos>,  // 本拍体素写入登记;下一拍消费
}
impl ShardSim {
    /// 统一唤醒入口(观察者/区域半径、命令、地形变更、闭包传播共用):
    /// 前置不满足 → 推迟并返回阻塞请求;满足 → 整组 Dynamic + 重置静止 + 簿记。
    fn wake_group(&mut self, group: &[EntityId]) -> WakeOutcome;
}
```

## 语义细则

1. **常驻票**:醒着 ∧ 运动中 ∧ 不在任何唤醒范围 → `Issue ActorMotion`(幂等);
   唤醒(接触/地形变更/闭包)同签发;静止 ∨ 进入唤醒范围 ∨ 到达上限 ∨ 超预算冻结
   → `End`;到期内部指令 = 撤票(非强制落地)。
2. **运动中不冻结(§4.4.2)**:常驻体滞回冻结路径加"静止"前置;运动中离开范围
   → 持票继续醒(蓝方块的常驻版修复)。
3. **静止判据**:低速(lin < linear_threshold ∧ ang < angular_threshold,沿用凝固桥
   常数)连续 `consecutive_ticks`(20)且位移门通过,或 rapier 睡眠;**与
   `solidify.enabled` 无关**;任何唤醒重置(`slow_ticks = 0`、位移窗口只留当前位置)。
4. **统一唤醒共同规则(§4.9)**:
   - 前置:被唤醒者当前足迹 ∩ 本分片区域 ② 齐;缺 → 整组推迟 + 每缺 chunk 阻塞请求
     (免冷却);
   - 支撑闭包:向上递归(几何判定,rapier 不为 kinematic–kinematic 生成接触);
   - 整组原子:组内全部满足前置才醒,否则整组推迟(不许"下面的醒了走开");
   - 重置静止;翻转簿记;票据按 §4.3.4。
5. **地形变更唤醒**:本分片任何体素写入(SetBlock、BlockWrite 信封、凝固、强制落地)
   改写格 c → 登记 c;**下一拍** apply_activation 对 AABB(外扩 skin)相交冻结体
   走统一唤醒:不受冷却、**不受预算截断**(当拍允许超限,后续常规淘汰);闭包同组。
6. **命令唤醒**:Spawn/Teleport 重建即醒(现状),改走统一前置(缺 ② → 待地形,
   03 已接);同拍预算淘汰路径 `enforce_budget_after_command_wake` 按新规则
   (瞬态落地/常驻冻结)。
7. **超预算淘汰**(§4.12):醒数超限 → 到最近**唤醒中心**(观察者 + 区域票锚点)距离
   降序,同距 EntityId 升序;瞬态 → 强制落地、常驻 → 冻结 + End ActorMotion;
   当拍地形变更唤醒的不进淘汰池。
8. **预算降级的例外标注**:常驻体可能在运动中被冻结(计入遥测 `actor_motion_capped`
   相邻路径,遥测字段 06 统一)。
9. **快照**:静止窗口(slow_ticks、位移窗口)、pending_terrain_wakes 入快照。
10. **常驻体推迟唤醒(S4)**:统一唤醒的前置检查(足迹 ② 缺 → 整组推迟 + 阻塞请求)
    在本任务对常驻体完整生效;未启用模式无推迟唤醒(D17,§9-7)。
11. **S6 自查**:新增测试凡"无观察者"场景先启用激活(远处区域票或设后移除观察者);
    既有未启用用例零改动。

## 测试清单

| 测试名(文件::函数) | 断言 | AC |
| --- | --- | --- |
| `wake::upward_closure_stacks_and_gaps` | 竖叠全并;间距 > skin 断开;更高才并入;含 seeds、EntityId 序 | T12 |
| `wake::closure_atomic_when_budget_short` | 空闲预算不足 → 整组推迟,无一单独醒 | T12 |
| `wake::terrain_wake_next_tick_no_cooldown_no_budget` | 写格登记 → 下一拍醒;冷却满、醒数满仍醒(超限) | T13 |
| `sim::actor_keeps_awake_while_moving_out_of_range` | 常驻体运动出范围:持票醒着(active_ticks 递增);静止后(冷却过)冻结 | T11 |
| `sim::actor_motion_ticket_issue_end_lifecycle` | 运动中 Issue;静止 End;再动再 Issue(新时钟) | T11/§4.3.4 |
| `sim::wake_resets_stillness_window` | 唤醒后 ≥ consecutive_ticks 内不判静止(伪造旧窗口被清) | T11 |
| `sim::terrain_wake_under_frozen_actor` | 移除冻结常驻体脚下格 → 下一拍醒并下落 | T13 |
| `sim::over_budget_actor_freezes_and_ends_ticket` | 超预算常驻 → 冻结 + 票 End(I3 前置一致);瞬态对照落地 | T8/§9-2 |
| `sim::deferred_wake_blocks_on_footprint` | 足迹 ② 缺 → 推迟(保持冻结)+ 阻塞请求;② 齐后醒 | T10 |
| `shard/scheduler::actor_ticket_deadline_ends_ticket_only` | 常驻票到期:撤票不落地;实体保持冻结态 | §4.3.2 |
| `e2e/activation_tickets::ac_t10_roundtrip_no_falling_through` | 复现 §3.3-3:常驻体折返,足迹 ② 就绪前不醒,全程 y ≥ 地表 | T10 |
| `e2e/activation_tickets::ac_t11_actor_ticket_full` | T11 三项:推走观察者不冻结、上限撤票计数、静止重置 | T11 |
| `e2e/activation_tickets::ac_t12_closure_group_wake` | 一叠冻结体唤醒底层 → 整叠同拍;预算不足整叠推迟 | T12 |
| `e2e/activation_tickets::ac_t13_terrain_change_wake` | SetBlock 移除脚下 → 下一拍醒;预算满仍醒;之后常规淘汰 | T13 |
| `e2e/activation.rs::far_actor_still_freezes_after_rest`(§9-1 迁移) | 原远冻结用例改:actor=true + 先静止 → 冻结语义保留 | §9-1 |
| (§9-7 回归)未启用模式既有用例零改动全绿 | — | §9-7/T24 |

## §9 白名单对应

- 第 1 条(常驻半边):`frozen_body_ignores_detonate`、
  `observers_drive_freeze_via_scheduler`、`activation.rs` 实体冻结/唤醒用例、
  `e2e/activation.rs` 相关用例 → 被测实体改 `actor = true` 且期望冻结前先静止
  (零重力零速生成推进 ≥ 21 拍,或落地沉降);瞬态新用例 03 已补;
- 第 2 条(常驻半边):超预算冻结期望保留,补 End ActorMotion 断言。

## §14 检查项自查范围

- 6:常驻到期内部指令时序;7:I1/I2 断言随新路径持续成立;
- 8:闭包/地形变更的阻塞请求只依实体状态。

## 偏离需求示意之处

1. 支撑闭包邻接用几何 AABB(外扩 skin)而非接触对——需求 §4.9 已写明"几何判定",
   本任务书固化其为 `wake.rs` 纯函数(rapier 类型不出 crate);
2. `pending_terrain_wakes` 以 `BTreeSet<BlockPos>` 登记(需求示意"待处理的地形变更
   唤醒格"),去重语义:同拍同格多次写入只醒一次。

## 门禁包

标准三条(workspace 全量)。

## 完成定义

- 测试清单全绿;T10 折返竞态在 e2e 复现并修复;§9-1 常驻半边迁移完成;
- 提交说明含迁移明细;
- **回评提醒:checkpoint 布局再变,旧存档不可用,需清 data_dir**(B5)。

## 风险

- "静止但被预算淘汰"与"静止冻结"的簿记交互(淘汰冻结也要求翻转簿记,防抖);
- 地形变更唤醒"不受预算截断"可能瞬时超限很多(大范围爆破)——按 §4.9 允许超限,
  下一拍常规淘汰收敛;测试锚定不失控(≤ 登记格数)。
