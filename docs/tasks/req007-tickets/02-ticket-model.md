# 任务 02:票据模型、票据表、票据收件箱;观察者票 + 区域票

- 状态:待放行(修订 2:按审核 B3/B4.4/S1/S2/S3 与 D17 修订)。
- 对应 AC:**AC-T1 再比对**(重跑 00 场景,四项严格相等)、AC-T18(区域票)、
  AC-T24(部分:开关推广到区域票)。
- 涉及 §9 白名单:**第 5 条**(观察者收件箱推广为票据收件箱,观察者命令语义不变,
  只允许改类型名/接口名)、**第 7 条**(未启用模式用例零改动)。
- 涉及 §14 检查项:1(决策层无 TicketKind 分支——S1 后由结构保证)、2(枚举只追加)、
  3(键序遍历/无墙钟随机 IO)、4(只有观察者/区域票命令入日志)、6(时序)、9(未启用一致)。

## 目标与范围

落地 §4.2 票据模型与 §4.3 操作语义,实现两种**外部输入票**:观察者票(等价改造现状)
与区域票(新命令)。`plan` 改读**票据视图**;ensure、分片保活改读票据;分片 despawn
条件改"空片 + 无票 + idle";D17 开关推广为"观察者**或区域票**"。**不含实体运动票**
(03/04)。② 的决策仍在 `plan`(B3 唯一出处不变;本任务把锚点从观察者坐标推广为票据)。

## 交付物

1. `physics-sim`(新文件 `tickets.rs` 视图类型):`TicketHolder/TicketKind/TicketView/
   RegionSpec/WakeSpec/TicketClock/TicketState`;
2. `shard`(新文件 `tickets.rs`):`TicketTable` + 票据操作(全幂等;区域票覆盖续期)+
   收件箱推广 + ensure/保活/despawn 改读票据 + 锚点派生;
3. `activation.rs::plan` 改签名:锚点输入从 `observers: &[Vec3]` 改为 `tickets: &[TicketView]`
   (扁平视图,含不透明排序键,见 S1);
4. `messaging`:`SimCommand::SetAreaTicket/RemoveAreaTicket`(只追加,ordinal 8/9);
5. `sim-api`:`Session::set_area_ticket/remove_area_ticket`(盖章入日志;超限类型化拒绝);
6. `app`:票据收件箱接线(观察者 + 区域票命令走调度器级收件箱,永不进分片前缀)、
   `translate` 白名单(**区域票命令不映射 → 拒绝**)、ensure/despawn 改读票据;
7. 快照:票据表 + 收件箱未到期条目入快照(版本号保持 4,B5)。

## 授权改动

- `crates/physics-sim/src/tickets.rs`(新)、`activation.rs`、`sim.rs`(`apply_activation`
  传票视图)、`lib.rs` re-export;
- `crates/shard/src/tickets.rs`(新)、`scheduler.rs`(收件箱/ensure/保活/despawn/快照/
  开关推广)、`lib.rs`;
- `crates/messaging/src/command.rs`(两命令变体 + ordinal);
- `crates/sim-api/src/session.rs`(区域票 API);
- `crates/app/src/driver.rs`(收件箱接线、ensure/despawn 改读票据、区域票本地通道)、
  `crates/app/src/net.rs`(translate 拒绝)、`crates/app/src/config.rs`(区域票上限两键);
- 测试各 crate + `crates/e2e/tests/`(AC-T1 再比对、区域票用例)。

## 接口设计(签名固定)

```rust
// physics-sim::tickets(视图类型,plan 输入;serde 只追加)
pub type AreaId = u64;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum TicketHolder {
    Observer(InputSource),
    Entity(EntityId),   // 本任务无生产者;03/04 用
    Area(AreaId),
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum TicketKind { Observer, Area, TransientMotion, ActorMotion } // 只追加

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RegionSpec {  // 只追加
    None,
    Radius { load_m: f32, unload_m: f32 },   // 水平,锚点圆心,带滞回
    Footprint { lookahead_ticks: u64, max_lookahead_chunks: usize }, // 03 语义
    Chunks(BTreeSet<ChunkPos>),
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum WakeSpec {    // 只追加
    None,
    SelfOnly,
    Radius { load_m: f32, unload_m: f32 },   // 3D
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TicketClock {
    pub issued_at: Tick,
    pub active_ticks: u64,
    pub limit: Option<u64>,
    pub starts_at: Option<Tick>,   // 恒 None(预留)
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TicketState { Active, Paused, Suspended }

/// 决策层唯一可见形状。**不含 holder/kind**(S1):票种语义在结构上进不了决策层;
/// `order` 是票据表键序中的名次(0 起,连续),仅用于确定性排序与平局决胜。
#[derive(Debug, Clone)]
pub struct TicketView {
    pub order: u32,
    pub priority: u8,
    pub data: RegionSpec,
    pub collider: RegionSpec,
    pub wake: WakeSpec,
    pub state: TicketState,
    pub anchor: Vec3,                     // 每拍派生(见下)
}

// shard::tickets
pub type TicketKey = (TicketHolder, TicketKind);
#[derive(Debug, Default)]
pub struct TicketTable { /* BTreeMap<TicketKey, StoredTicket> */ }
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StoredTicket {
    pub priority: u8,
    pub data: RegionSpec,
    pub collider: RegionSpec,
    pub wake: WakeSpec,
    pub clock: TicketClock,
    pub state: TicketState,
    pub center: Option<Vec3>,  // 区域票锚点;其他 None(锚点由持有者派生)
}
impl TicketTable {
    /// 幂等语义(D14 + S3):
    /// - 观察者票/区域票:键已在 → 覆盖规格与 TTL(**区域票时钟重置 = 续期,S3**;
    ///   观察者票无 limit,时钟重置与否不可观测,统一按续期实现);
    /// - 实体票:键已在 → 无操作(不重置时钟、不改状态);
    /// - 不存在 → 新建(时钟 issued_at = 当前拍,active_ticks = 0)。
    pub fn issue(&mut self, key: TicketKey, ticket: StoredTicket, now: Tick);
    /// 全部幂等:不存在 → 无操作。
    pub fn end(&mut self, key: TicketKey);
    /// 幂等:不存在或已在目标态 → 无操作。
    pub fn set_state(&mut self, key: TicketKey, s: TicketState);
    pub fn advance_clocks(&mut self);                          // Active 时钟 +1
    pub fn expired(&self) -> Vec<TicketKey>;                   // 到期(键序)
    /// 视图导出:锚点由调用方按 holder 派生;order = 键序名次;Entity/观察者票
    /// 无锚(03 接)时跳过。Suspended 不导出。
    pub fn views(&self, anchor_of: impl Fn(&TicketHolder) -> Option<Vec3>) -> Vec<TicketView>;
    pub fn iter(&self) -> impl Iterator<Item = (TicketKey, &StoredTicket)>; // 键序
    pub fn order_of(&self, key: TicketKey) -> Option<u32>;
}
```

`plan` 新签名(02 内迁移完毕,旧 observers 入口删除;既有测试改用"构造一张观察者票"
辅助函数,断言不变——§9-5):

```rust
pub fn plan(
    &mut self,
    tickets: &[TicketView],
    loaded_data: &[ChunkPos],
    loaded_collider: &[ChunkPos],
    entities: &[(EntityId, Vec3, bool)],
    now: Tick,
    region: ChunkRegion,
) -> ActivationDecisions;
// 候选 = Σ Active/Paused 票的 ①/② 规格;排序键 = (priority, 距锚点距离, ChunkPos);
// ①② 各自预算;② 决策仍输出 colliders_to_build/colliders_to_drop(B3 唯一出处不变)。
```

## 语义细则

1. **收件箱**:观察者 Set/Remove 与区域票 Set/Remove 走调度器级票据收件箱
   (`observer_inbox` 推广;§9-5 只允许改名/接口名),N 拍开头应用,**当拍生效**;
   永不进分片输入前缀(出现即一致性违规 fail-stop,现状断言保留)。
2. **开关推广(D17)**:`activation_enabled` = 曾设观察者**或**区域票;单向;随快照;
   未启用时本任务新增的一切(票据派生、决策、区域票处理)都不发生,行为与
   `8702670` 一致。
3. **锚点派生**:观察者票 = 观察者坐标;区域票 = `StoredTicket.center`;实体票 =
   实体当前位置(在途取迁移载荷,03 接)。无锚可派生(观察者已删)→ 票应已被 End;
   出现无锚票 = 一致性违规 panic(06 的 I3 前置防御)。
4. **plan 消费**:装载候选 = Σ Active/Paused 票 ① Radius 的"进"区域 − 已载;
   卸载候选 = 已载 ∧ 不在任何 Active/Paused 票 ① "出"区域 ∧ 冷却;排序
   `(priority, 距锚点距离, ChunkPos)` 升序;①② 按各自预算(01)。多票覆盖同一
   chunk 时取最优(阻塞前缀在 03 引入;本任务 = 最小 priority、其次最近距离)。
5. **③ 唤醒中心**:WakeSpec::Radius 的票;醒数超限按"到最近唤醒中心距离"降序淘汰
   (无中心 = +∞);仅观察者票时与现状数值等价(半径同 24/32)。
6. **实体票签发在本任务不发生**(03/04);TicketTable 对实体票的幂等语义先行落地并
   测试(D14 矩阵的前半)。
7. **ensure**(driver):每张 Active/Paused 票 ① 进区域内 chunk 的 owner 分片必须注册;
   观察者票另保留现状 32 m 邻片 ensure(驱动侧行为不变)。区域票的 ensure = 其
   "进"区域覆盖的 owner 分片注册。
8. **保活**(shard despawn 候选):分片有票 ⇔ 任一 Active/Paused 票 ① 出区域与该分片
   核心相交;观察者票按此规则 == 现状"到 AABB ≤ 48 m"。Suspended 票不保活、不
   ensure、不占区域。
9. **despawn 条件**:空片 + 无票 + idle(语义不变,票源换成票据表)。
10. **区域票约束**:`radius_m ≤ area_ticket_max_radius_m`(默认 128)、在票数
    ≤ `area_ticket_max_count`(默认 64);超限 → **类型化拒绝**
    (`Result<(), AreaTicketError>`,枚举 `RadiusTooLarge { max }` / `TooManyTickets
    { max }`),不 panic。同 id 覆盖 = 续期(S3),计数不变。
11. **区域票不入 C2S**:`translate` 对 `SetAreaTicket/RemoveAreaTicket` 返回
    `Reject`;`command_shard` 返回 `None`。仅本地受信通道(driver commands 通道 /
    `Session`)。
12. **入日志**:观察者/区域票命令照常盖章入外部命令日志(回放再生);票据表本体
    不入日志、入快照。
13. **AC-T1 再比对(B4.4/S2)**:重跑 00 场景,四项**严格相等**——① 集 == golden
    chunk 集、② 集 == golden chunk 集、冻结/唤醒集 == golden、分片集 == golden;
    零"推迟唤醒"差异(待地形在 03 才引入;本任务若出现差异即 bug)。

## 测试清单

| 测试名(文件::函数) | 断言 | AC |
| --- | --- | --- |
| `shard/tickets::issue_end_idempotent` | 实体票 Issue 已在:无操作、时钟不变;End 不存在:无操作 | D14 |
| `shard/tickets::area_ticket_renewal_on_override` | 同 id 覆盖:规格/TTL 更新、**时钟重置(S3)**;计数不变 | T18/S3 |
| `shard/tickets::table_key_order_and_views` | 遍历键序;order = 键序名次(0 起连续);views 派生锚点;Suspended 不导出 | §14-3 |
| `shard/tickets::clocks_advance_and_expire` | Active +1;Paused/Suspended 停;到期出 `expired()` | §4.3.2 |
| `shard/tickets::observer_ticket_equivalence` | 观察者票 ①②③ 规格 == 现状半径(32/48、24/32);保活判定 == AABB 48 m | T1 |
| `shard/tickets::area_ticket_radius_and_count_caps` | 超半径/超数量 → `AreaTicketError` 两变体;边界值通过;覆盖不占新名额 | T18 |
| `shard/tickets::area_ticket_ttl_expires` | TTL 到期票 End;分片恢复 despawn 资格;续期后 TTL 重新起算 | T18/S3 |
| `shard/tickets::suspended_ticket_not_keepalive` | Suspended 票不保活、不 ensure、不占区域 | T20(前半) |
| `shard/scheduler::despawn_reads_ticket_table` | 有 Active/Paused 票的分片不进 despawn 候选;票清 + 空 + idle → 候选出现 | T18/T20 |
| `shard/scheduler::activation_enabled_by_area_ticket` | 未设观察者、SetAreaTicket → 启用(激活开始生效);Remove 后仍启用(单向) | T24/D17 |
| `sim::apply_activation_reads_ticket_views` | plan 输入票视图;观察者票 == 旧 observers 路径(对拍:同坐标票同输出) | T1 |
| `sim::observer_and_area_same_spec_same_decision`(S1 对拍) | Observer 与 Area 两票同 RegionSpec/WakeSpec/priority/anchor → 决策逐字段相等 | §14-1/S1 |
| `sim::ticket_view_has_no_kind`(结构检查) | TicketView 无 holder/kind 字段(编译期保证;测试断言字段集) | §14-1/S1 |
| `messaging::area_ticket_ordinals_appended` | 新变体 ordinal = 8/9;旧 0–7 不变;旧编码可解 | 只追加 |
| `app::net::translate_rejects_area_ticket` | C2S 承载区域票命令 → Reject(类型化文案);`command_shard` None | T18 |
| `app::driver::area_ticket_via_local_channel_only` | driver commands 通道 Set/Remove 生效(入日志、票据表变化);网络路径无入口 | T18 |
| `session::set_area_ticket_logs_command` | 盖章入日志;回放后票据表一致 | T18 |
| `e2e/activation_baseline::ac_t1_matches_golden_after_tickets` | **四项严格相等**(同 01 的四项;02 再比一次,B4.4) | T1/S2 |
| `e2e/activation_tickets::area_ticket_keeps_region_alive` | 无观察者 SetAreaTicket(①②③):chunk 装载、范围内冻结体唤醒、分片不 despawn;Remove 后 idle 正常 despawn;命令入日志回放一致 | T18 |

## §9 白名单对应

- 第 5 条:观察者收件箱 → 票据收件箱;涉及 `scheduler.rs` 观察者收件箱相关测试,
  只允许类型名/接口名改动,提交说明逐个列出;
- 第 7 条:未启用模式用例零改动(区域票路径全部要求先启用)。

## §14 检查项自查范围

- 1:结构保证(TicketView 无票种字段)+ 同规格对拍测试;
- 2/3/4/6/9:测试见清单;5(rapier 不出 crate)不涉及。

## 偏离需求示意之处(修订后余留)

1. `StoredTicket.center` 只服务区域票;需求示意 RegionSpec::Radius 自带中心,此处把
   锚点与规格分离(观察者/实体票锚点动态派生,区域票锚点静态)。已获同意。
2. `TicketView.order`(S1 裁定)替代需求示意中"键含 holder/kind"的排序输入:
   不透明名次,结构上杜绝决策层票种分支。已获同意。

## 门禁包

标准三条(workspace 全量)。

## 完成定义

- 测试清单全绿;AC-T1 四项再比对绿;区域票全链路(本地通道 → 日志 → 票据表 →
  装载/唤醒/保活 → TTL → 续期)可用;C2S 无区域票入口;开关推广完成;
- 提交说明含 §9-5 迁移明细;
- **回评提醒:checkpoint 布局再变(票据表/收件箱入快照),旧存档不可用,需清 data_dir**。

## 风险

- plan 签名变更波及面大(sim/scheduler/全部既有激活测试)——既有测试改为"构造一张
  观察者票"的辅助函数,断言不变(§9-5);若个别用例超出改名范围,回评报告,不顺手改。
