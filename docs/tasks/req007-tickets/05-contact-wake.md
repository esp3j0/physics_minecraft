# 任务 05:接触唤醒(撞前预测 + 动量传播 + 补偿)

- 状态:待放行。
- 对应 AC:AC-T14、T15、T16、T17。
- 涉及 §9 白名单:不涉及(全新能力,无既有断言改动;§9-1 迁移后的远冻结常驻用例
  若受接触唤醒影响,在 T16 静止对照中锚定不变)。
- 涉及 §14 检查项:3(图遍历确定性)、7(不变量)、5(rapier 类型不出 crate;
  `ContactPair` 冲量读取封在 physics-sim 内)。

## 目标与范围

落地 §4.10 接触唤醒:**撞前预测**为主路径(step 开头、物理步进前),冻结邻接图上
最大动量传播,成组截断后走 04 的统一唤醒;**补偿路径**兜底(step 后从 rapier 接触对
收种子,下一拍传播,种子入快照)。

## 交付物

1. `physics-sim/src/wake.rs` 扩展:种子筛选、邻接图、传播、成组;
2. `ShardSim::step`:预测钩子(apply_prefix 后、步进前)与补偿钩子(步进后);
3. 新配置键五个 + 快照字段(补偿种子);
4. 参数示例锚定:decay 0.5、门槛 1.0 时 p=5 恰醒 3 个(§4.10.3 示例 = 测试)。

## 授权改动

- `crates/physics-sim/src/wake.rs`、`sim.rs`、`snapshot.rs`;
- `crates/app/src/config.rs`、`driver.rs`(配置透传);
- 测试:`physics-sim` 内联 + `crates/e2e/tests/activation_tickets.rs` 追加。

## 接口设计(签名固定)

```rust
// physics-sim::wake.rs(扩展)
/// 种子:醒着实体 A 的扫掠体(旋转包络 AABB 沿 v 扫掠 lookahead 拍,外扩 skin)
/// 相交的冻结体;冲击动量 p_AB = m_A · max(0, v_A · n̂_AB),同 B 取最大;
/// p ≥ min_momentum 才入选。返回 (seed_entity, momentum),按 momentum 降序、
/// EntityId 升序。
pub fn contact_wake_seeds(
    awake: &[(EntityId, Vec3, Vec3, f32, Vec3)],  // (id, center, half, mass, linvel)
    frozen: &[(EntityId, Vec3, Vec3)],            // (id, center, half)
    lookahead_ticks: u64, skin_m: f32, min_momentum: f32, dt: f32,
) -> Vec<(EntityId, EntityId, f32)>;              // (A, B, p_AB)

/// 冻结邻接图:碰撞形状间距 ≤ skin_m 即相邻(几何)。
pub fn frozen_adjacency(frozen: &[(EntityId, Vec3, Vec3)], skin_m: f32)
    -> BTreeMap<EntityId, BTreeSet<EntityId>>;

/// 最大动量传播:p(子) = decay · p(父),每节点取最大;优先队列
/// (p 降序、同 p EntityId 升序);p < min_momentum 或 hops > max_hops 停。
/// 返回 (entity, p) 按 p 降序、EntityId 升序。
pub fn propagate_momentum(
    graph: &BTreeMap<EntityId, BTreeSet<EntityId>>,
    seeds: &[(EntityId, f32)],
    decay: f32, min_momentum: f32, max_hops: usize,
) -> Vec<(EntityId, f32)>;
```

## 语义细则

1. **时机**:预测在 `step` 内 `apply_prefix` 之后、`rapier` 步进之前(命令与爆炸冲量
   已施加);补偿在步进之后。
2. **候选冻结体**:常驻体或被承载的瞬态体(与 §4.4.2 冻结资格一致);待地形体**不**
   被(§4.10.5)。
3. **门槛量纲**:`contact_wake_min_momentum`(kg·m/s);静止接触 v≈0 天然不触发(T16)。
4. **成组与截断**:每个结果节点走 04 统一唤醒(前置/闭包/重置/簿记/票据);组按组内
   最大 p 降序、最小 EntityId 升序;截断至 `contact_wake_max_bodies_per_tick`(64)
   且**只占空闲醒体预算**,不淘汰已醒者;放不下的组整组跳过。
5. **落实速度**:被唤醒者当拍切 Dynamic、**零速度起步**(动量交换交给 rapier 同步
   解算——这正是撞前预测的目的)。
6. **补偿路径**:step 后收集"Dynamic ↔ 冻结"接触对,法向冲量合计 ≥ 门槛 → 以该冲量
   为种子,并入**下一拍**的传播;补偿种子集合入快照(消费即清)。
7. **确定性**:图遍历全部排序;`total_impulse_magnitude()` 只读标量。
8. **爆炸免疫不变**:detonate 冲量施加发生在冻结体上即跳过(§9.12 既有语义),接触
   唤醒不改它;但醒着的撞击者撞冻结体的场景由本任务覆盖。

## 测试清单

| 测试名(文件::函数) | 断言 | AC |
| --- | --- | --- |
| `wake::seeds_require_momentum_and_sweep` | 慢速/静止不种子;扫掠命中;同 B 多 A 取最大 | T14/T16 |
| `wake::adjacency_and_propagation_example` | §4.10.3 示例:p0=5、decay 0.5、门槛 1 → 恰 3 个;跳数上限生效 | T15 |
| `wake::propagation_order_independent` | 种子集合同拍多源,结果与处理顺序无关 | T15 |
| `wake::per_tick_cap_and_idle_budget` | 每拍上限截断;空闲预算不足整组跳过;不淘汰已醒实体 | T15 |
| `sim::prestep_wake_momentum_exchange` | 1 kg @5 m/s 撞冻结 1 kg:同拍双 Dynamic,被撞 > 1 m/s、撞击者降速且不反向弹回 | T14 |
| `sim::contact_wake_off_control_group` | 关闭接触唤醒(配置):撞击者反弹、被撞者不动(对照) | T14 |
| `sim::compensation_next_tick_wake` | 预测漏判(第三方中途加速)→ 下一拍醒;补偿种子随快照往返,恢复后行为一致 | T17 |
| `sim::resting_contact_no_wake` | 静止贴靠 600 拍不触发 | T16 |
| `sim::awaiting_terrain_never_contact_wake` | 待地形体既不当撞击者也不被唤醒 | §4.10.5 |
| `e2e/activation_tickets::ac_t14_momentum_exchange` | e2e 全链路(真驱动):T14 双向断言 + 对照组 | T14 |
| `e2e/activation_tickets::ac_t15_row_of_five` | 5 连排:恰醒 3;跳数/每拍上限/空闲预算三重截断;多源顺序无关 | T15 |
| `e2e/activation_tickets::ac_t17_compensation_e2e` | 补偿路径 e2e + checkpoint 恢复一致 | T17 |

## §9 白名单对应

不涉及。若 §9-1 迁移后的用例(`frozen_body_ignores_detonate` 等)因接触唤醒改变
行为:爆炸路径对冻结体免疫不变,不应受影响;万一受影响,按 T16 静止对照排查,
属 bug 不属白名单。

## §14 检查项自查范围

- 3:遍历全排序、无随机;5:`total_impulse_magnitude` 使用封在 physics-sim;
- 7:唤醒走 04 统一入口,I1/I2 持续。

## 偏离需求示意之处

1. 预测种子只考虑 `linvel` 扫掠、不含角速度包络膨胀(需求示意"旋转包络 AABB 沿 v
   扫掠");高速旋转的慢移实体可能漏预测 → 由补偿路径兜底,符合 §4.10.4 设计意图。
2. `contact_wake_seeds` 需要质量 `mass`:从 EntityDesc 体积 × 密度(既有常数)派生,
   不新增配置。

## 门禁包

标准三条(workspace 全量)。

## 完成定义

- 测试清单全绿;T14 对照组锚定"关掉新能力 = 旧行为";示例数值(p=5 醒 3)被测试
  锚定,防参数漂移。

## 风险

- rapier 接触对在 kinematic–dynamic 同步解算时的冲量口径需实测验证(§4.10.4 依赖
  `total_impulse_magnitude()`;0.36.1 API 存在,量纲验证放首个测试);
- 预测窗口 2 拍内多次碰撞的排序敏感性:传播的优先队列已按 (p 降序, EntityId 升序)
  定序,AC1 位级纪律下可复现。
