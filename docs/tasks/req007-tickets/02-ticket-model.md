# 任务 02:票据模型、票据表、票据收件箱;观察者票 + 区域票

- 状态:待放行。
- 对应 AC:**AC-T1 全绿**(重跑 00 场景逐拍对比)、AC-T18(区域票)。
- 涉及 §9 白名单:**第 5 条**(观察者收件箱推广为票据收件箱,观察者命令语义不变,
  只允许改类型名/接口名)。
- 涉及 §14 检查项:1(决策层无 TicketKind 分支)、2(枚举只追加)、3(键序遍历/无墙钟
  随机 IO)、4(只有观察者/区域票命令入日志)、6(N 拍产生 N+1 生效/收件箱当拍生效)。

## 目标与范围

落地 §4.2 票据模型与 §4.3 操作语义,实现两种**外部输入票**:观察者票(等价改造现状)
与区域票(新命令)。`plan`、ensure、分片保活改读**票据视图**;分片 despawn 条件改
"空片 + 无票 + idle"。**不含实体运动票**(03/04)。

## 交付物

1. `physics-sim`(新文件 `tickets.rs` 视图类型):`TicketHolder/TicketKind/TicketView/
   RegionSpec/WakeSpec/TicketClock/TicketState`;
2. `shard`(新文件 `tickets.rs`):`TicketTable`(调度器级全局唯一一张) + 票据操作
   (全幂等)+ 收件箱推广 + ensure/保活/despawn 改读票据 + 锚点派生;
3. `activation.rs::plan` 改签名:输入从 `observers: &[Vec3]` 改为 `tickets: &[TicketView]`
   (扁平视图,决策层不可见 TicketKind);
4. `messaging`:`SimCommand::SetAreaTicket/RemoveAreaTicket`(只追加,ordinal 8/9);
5. `sim-api`:`Session::set_area_ticket/remove_area_ticket`(盖章入日志);
6. `app`:票据收件箱接线(观察者 + 区域票命令走调度器级收件箱,永不进分片前缀)、
   `translate` 白名单(**区域票命令不映射 → 拒绝**)、ensure/despawn 改读票据;
7. 快照:票据表 + 收件箱未到期条目入快照。

## 授权改动

- `crates/physics-sim/src/tickets.rs`(新)、`activation.rs`、`sim.rs`(`apply_activation`
  传票视图)、`lib.rs` re-export;
- `crates/shard/src/tickets.rs`(新)、`scheduler.rs`(收件箱/ensure/保活/despawn/快照)、
  `lib.rs`;
- `crates/messaging/src/command.rs`(两命令变体 + ordinal);
- `crates/sim-api/src/session.rs`(区域票 API);
- `crates/app/src/driver.rs`(收件箱接线、ensure/despawn 改读票据、区域票本地通道)、
  `crates/app/src/net.rs`(translate 拒绝)、`crates/app/src/config.rs`(区域票上限两键);
- 测试各 crate + `crates/e2e/tests/`(AC-T1 对比断言落地、区域票用例)。

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

/// 决策层唯一可见形状(不含 kind/holder 语义;§4.2.2 承重墙)。
#[derive(Debug, Clone)]
pub struct TicketView {
    pub key: (TicketHolder, TicketKind),  // 仅供排序/日志;决策逻辑不得匹配其内容
    pub priority: u8,
    pub data: RegionSpec,
    pub collider: RegionSpec,
    pub wake: WakeSpec,
    pub state: TicketState,
    pub anchor: Vec3,                     // 每拍派生(见下)
}

// shard::tickets
#[derive(Debug, Default)]
pub struct TicketTable { /* BTreeMap<(TicketHolder, TicketKind), StoredTicket> */ }
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StoredTicket {  // TicketView + clock(时钟只存表内,视图派生)
    pub priority: u8,
    pub data: RegionSpec,
    pub collider: RegionSpec,
    pub wake: WakeSpec,
    pub clock: TicketClock,
    pub state: TicketState,
    pub center: Option<Vec3>,  // 区域票锚点;其他 None(锚点由持有者派生)
}
impl TicketTable {
    pub fn issue(&mut self, key: TicketKey, ticket: StoredTicket); // 幂等语义见下
    pub fn end(&mut self, key: TicketKey);                         // 幂等
    pub fn set_state(&mut self, key: TicketKey, s: TicketState);   // 幂等
    pub fn advance_clocks(&mut self, now: Tick);                   // Active 时钟 +1
    pub fn expired(&self) -> Vec<TicketKey>;                       // 到期(键序)
    pub fn views(&self, anchor_of: impl Fn(&TicketHolder) -> Option<Vec3>) -> Vec<TicketView>;
    pub fn iter(&self) -> impl Iterator<Item = (TicketKey, &StoredTicket)>; // 键序
}
pub type TicketKey = (TicketHolder, TicketKind);
```

`plan` 新签名(旧 observers 入口保留一个适配重载给 03 前的过渡测试;02 内迁移完毕后
仅留票据版):

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
```

## 语义细则

1. **收件箱**:观察者 Set/Remove 与区域票 Set/Remove 走调度器级票据收件箱
   (`observer_inbox` 推广;§9-5 只允许改名/接口名),N 拍开头应用,**当拍生效**;
   永不进分片输入前缀(分片前缀出现即一致性违规 fail-stop,现状断言保留)。
2. **实体票操作幂等(D14)**:`issue` 键已在 → 无操作(不重置时钟、不改状态;
   观察者/区域票例外 = 覆盖更新规格与 TTL,时钟**保留**——覆盖不等于重签);
   `end`/`set_state` 不存在 → 无操作。I3 校验在 06,本任务先按幂等实现。
3. **时钟**:调度器每拍末对 Active 票 `active_ticks += 1`;`limit` 到期:区域票直接
   `End`(本任务);实体票到期投递内部指令(03/04)。
4. **锚点派生**:观察者票 = 观察者坐标;区域票 = `StoredTicket.center`;实体票 =
   实体当前位置(在途取迁移载荷,03 接)。无锚可派生(观察者已删)→ 票应已被 End;
   出现无锚票 = 一致性违规 panic(06 的 I3 前置防御)。
5. **plan 消费**:装载候选 = Σ Active/Paused 票 ① Radius/Footprint 的"进"区域 −
   已载;卸载候选 = 已载 ∧ 不在任何 Active/Paused 票 ① "出"区域 ∧ 冷却;
   排序 `(priority, 距锚点距离, ChunkPos)` 升序;预算按档(01 的两档)。
   Suspended 票不占区域、不保活、不 ensure。
6. **③ 唤醒中心**:WakeSpec::Radius 的票;醒数超限按"到最近唤醒中心距离"降序淘汰
   (无中心 = +∞);仅观察者票时与现状数值等价(半径同 24/32)。
7. **ensure**(driver):每张 Active/Paused 票 ① 进区域内 chunk 的 owner 分片必须注册;
   观察者票另保留现状 32 m 邻片 ensure(驱动侧行为不变)。
8. **保活**(shard despawn 候选):分片有票 ⇔ 任一 Active/Paused 票 ① 出区域与该分片
   核心相交;观察者票按此规则 == 现状"到 AABB ≤ 48 m"。
9. **despawn 条件**:空片 + 无票 + idle(语义不变,票源换成票据表)。
10. **区域票约束**:`radius_m ≤ area_ticket_max_radius_m`(默认 128)、在票数
    ≤ `area_ticket_max_count`(默认 64);超限 → **类型化拒绝**
    (`Session::set_area_ticket` 返回 `Result<(), AreaTicketError>`,枚举
    `RadiusTooLarge { max }` / `TooManyTickets { max }`),不 panic。
11. **区域票不入 C2S**:`translate` 对 `SetAreaTicket/RemoveAreaTicket` 返回
    `Reject`;`command_shard` 返回 `None`。仅本地受信通道(driver commands 通道 /
    `Session`)。
12. **入日志**:观察者/区域票命令照常盖章入外部命令日志(回放再生);票据表本体
    不入日志、入快照。
13. **AC-T1 对比**:重跑 00 场景,逐拍 `data_to_load ∪ colliders` 集合 == golden
    chunk 集(② 集合 ⊆ ① 集合,差异 = 观察者 32 m 内 ② 全建时两者相等);
    冻结/唤醒集 == golden;分片集 == golden;唯一允许差异 = §4.6 推迟唤醒拍
    (本任务应为零,03 引入待地形后才可能出现)。

## 测试清单

| 测试名(文件::函数) | 断言 | AC |
| --- | --- | --- |
| `shard/tickets::issue_end_idempotent` | 实体票 Issue 已在:无操作、时钟不变;End 不存在:无操作;观察者/区域覆盖更新保时钟 | D14 |
| `shard/tickets::table_key_order_and_views` | 遍历键序;views 派生锚点正确;Suspended 不产视图 | §14-3 |
| `shard/tickets::clocks_advance_and_expire` | Active +1;Paused/Suspended 停;到期出 `expired()` | §4.3.2 |
| `shard/tickets::observer_ticket_equivalence` | 观察者票 ①②③ 规格 == 现状半径(32/48、24/32);保活判定 == AABB 48 m | T1 |
| `shard/tickets::area_ticket_radius_and_count_caps` | 超半径/超数量 → `AreaTicketError` 两变体;边界值通过 | T18 |
| `shard/tickets::area_ticket_ttl_expires` | TTL 到期票 End;分片恢复 despawn 资格 | T18 |
| `shard/tickets::suspended_ticket_not_keepalive` | Suspended 票不保活、不 ensure、不占区域 | T20(前半) |
| `shard/scheduler::despawn_reads_ticket_table` | 有 Active/Paused 票的分片不进 despawn 候选;票清 + 空 + idle → 候选出现 | T18/T20 |
| `sim::apply_activation_reads_ticket_views` | plan 输入为票视图;观察者票等价旧 observers 路径(对拍:同坐标票 == 旧输出) | T1 |
| `sim::no_ticket_kind_branch_in_plan`(评审检查) | 决策层仅消费 RegionSpec/WakeSpec/priority/state:以两票种同规格产同决策(Observer vs Area 同规格同输出) | §14-1 |
| `messaging::area_ticket_ordinals_appended` | 新变体 ordinal = 8/9;旧 0–7 不变;旧编码可解 | 只追加 |
| `app::net::translate_rejects_area_ticket` | C2S 承载区域票命令 → Reject(类型化文案);`command_shard` None | T18 |
| `app::driver::area_ticket_via_local_channel_only` | driver commands 通道 Set/Remove 生效(入日志、票据表变化);网络路径无入口 | T18 |
| `session::set_area_ticket_logs_command` | 盖章入日志;回放后票据表一致 | T18 |
| `e2e/activation_baseline::ac_t1_matches_golden` | **AC-T1 全绿**:重跑 00 场景,chunk/冻结唤醒/分片三集逐拍 == golden;无推迟唤醒差异 | T1 |
| `e2e/activation_tickets::area_ticket_keeps_region_alive` | 无观察者 SetAreaTicket(①②③):chunk 装载、范围内冻结体唤醒、分片不 despawn;Remove 后 idle 正常 despawn;命令入日志回放一致 | T18 |

## §9 白名单对应

- 第 5 条:观察者收件箱 → 票据收件箱;涉及 `scheduler.rs` 观察者收件箱相关测试,
  只允许类型名/接口名改动,提交说明逐个列出。

## §14 检查项自查范围

- 1:`no_ticket_kind_branch_in_plan` + 评审人工核对决策代码;
- 2/3/4/6:测试见清单;rapier 不出 crate(5)本任务不涉及。

## 偏离需求示意之处

1. `TicketView.key` 仍携带 `(TicketHolder, TicketKind)` 用于排序与日志,但决策逻辑
   不得读其内容——比需求"键不进视图"的示意更严格地保序,§14-1 以
   `no_ticket_kind_branch_in_plan` 对拍测试 + 人工评审兜底;
2. `StoredTicket.center` 只服务区域票;需求示意 RegionSpec::Radius 自带中心,此处把
   锚点与规格分离(观察者/实体票锚点动态派生,区域票锚点静态)。
3. 观察者/区域票覆盖更新保留时钟(需求未明说;若覆盖即重签,TTL 刷新会造成无限续期,
   与"同 id 覆盖"的运维语义不符)。

## 门禁包

标准三条(workspace 全量)。

## 完成定义

- 测试清单全绿;AC-T1 全绿;区域票全链路(本地通道 → 日志 → 票据表 → 装载/唤醒/保活
  → TTL)可用;C2S 无区域票入口;
- 提交说明含 §9-5 迁移明细。

## 风险

- plan 签名变更波及面大(sim/scheduler/全部既有激活测试)——既有测试多数改为
  "构造一张观察者票"的辅助函数,断言不变;
- 票据表快照字段使旧 checkpoint 拒绝(bincode 定长);如实注释。
