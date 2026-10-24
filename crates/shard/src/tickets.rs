//! 票据表与操作语义(REQ-007 §4.2/§4.3;任务书 req007-tickets/02)。
//!
//! 调度器级**全局唯一一张**表,按 `TicketKey = (TicketHolder, TicketKind)` 有序
//! (`BTreeMap`);任何遍历都按键序(§4.2.2-3)。操作语义(§4.3.1/D14/S3):
//! 全部幂等——观察者/区域票 Issue 键已在 → 覆盖规格与 TTL(区域票时钟重置 =
//! 续期,S3);实体票 Issue 键已在 → 无操作(不重置时钟、不改状态);End/Pause/
//! Suspend 等遇不存在或已在目标态 → 无操作。正确性由每拍不变量校验兜底
//! (I3,06 落地),比逐条操作报错更严格。

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{EntityId, Tick};

use physics_sim::tickets::{TicketClock, TicketHolder, TicketKind, TicketState, TicketView};

/// 全局唯一票据键:`(持有者, 种类)`(§4.2.1;BTreeMap 键序 = 声明序派生)。
pub type TicketKey = (TicketHolder, TicketKind);

/// 表内存储的票据(§4.2.1 `Ticket` 的实现形状)。`center` 只服务区域票
/// (锚点静态);观察者/实体票锚点由持有者每拍派生(任务书"偏离需求示意"
/// 第 1 条,已获同意)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StoredTicket {
    pub priority: u8,
    pub data: physics_sim::tickets::RegionSpec,
    pub collider: physics_sim::tickets::RegionSpec,
    pub wake: physics_sim::tickets::WakeSpec,
    pub clock: TicketClock,
    pub state: TicketState,
    /// 区域票锚点;其他持有者恒 `None`(锚点派生,细则 3)。
    pub center: Option<Vec3>,
}

/// 票据表(`BTreeMap<TicketKey, StoredTicket>` 的 NewType;键序遍历)。
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
pub struct TicketTable {
    tickets: BTreeMap<TicketKey, StoredTicket>,
}

impl TicketTable {
    pub fn new() -> Self {
        Self::default()
    }

    /// 签发(§4.3.1 `Issue`;D14 + S3 幂等语义):
    /// - 观察者票/区域票:键已在 → **覆盖**规格与 TTL;区域票时钟重置
    ///   (`issued_at = now`,`active_ticks = 0`)= 续期(S3);观察者票无
    ///   limit,时钟重置不可观测,统一按续期实现;
    /// - 实体票:键已在 → **无操作**(不重置时钟、不改状态——同拍多路径
    ///   重复 Issue 不误报;截止不重置,§4.3.3);
    /// - 不存在 → 新建。
    pub fn issue(&mut self, key: TicketKey, ticket: StoredTicket, now: Tick) {
        match key.0 {
            TicketHolder::Observer(_) | TicketHolder::Area(_) => {
                let mut t = ticket;
                t.clock.issued_at = now;
                t.clock.active_ticks = 0;
                self.tickets.insert(key, t);
            }
            // 实体票:已在 → 无操作(D14)。
            TicketHolder::Entity(_) => {
                self.tickets.entry(key).or_insert(ticket);
            }
        }
    }

    /// 删除(§4.3.1 `End`):不存在 → 无操作(幂等)。
    pub fn end(&mut self, key: TicketKey) {
        self.tickets.remove(&key);
    }

    /// 状态切换:不存在或已在目标态 → 无操作(幂等)。
    pub fn set_state(&mut self, key: TicketKey, state: TicketState) {
        if let Some(t) = self.tickets.get_mut(&key) {
            t.state = state;
        }
    }

    /// 查询(键序遍历用;返回引用供锚点派生/保活判定)。
    pub fn get(&self, key: TicketKey) -> Option<&StoredTicket> {
        self.tickets.get(&key)
    }

    /// 区域票计数(上限校验用;§4.2.3)。
    pub fn area_count(&self) -> usize {
        self.tickets
            .keys()
            .filter(|(h, _)| matches!(h, TicketHolder::Area(_)))
            .count()
    }

    /// 时钟推进(§4.3.2):每拍末对 Active 票 `active_ticks += 1`
    /// (Paused/Suspended 停)。
    pub fn advance_clocks(&mut self) {
        for t in self.tickets.values_mut() {
            if t.state == TicketState::Active {
                t.clock.active_ticks += 1;
            }
        }
    }

    /// 到期键集(键序;§4.3.2:`active_ticks ≥ limit` → 区域票直接 End,
    /// 由调用方执行;实体票投递内部指令,03 接)。
    pub fn expired(&self) -> Vec<TicketKey> {
        self.tickets
            .iter()
            .filter(|(_, t)| {
                t.clock
                    .limit
                    .is_some_and(|limit| t.clock.active_ticks >= limit)
            })
            .map(|(k, _)| *k)
            .collect()
    }

    /// 视图导出(plan 的输入;键序遍历 → order = 名次 0 起连续)。锚点由
    /// 调用方按持有者派生(`anchor_of` 返回 `None` = 无锚可派生——观察者已删
    /// 而票未 End 属一致性违规,任务书细则 3:panic 由调度器侧执行;此处跳过
    /// 无锚实体票(03 接),观察者/区域票无锚 = 违规 panic)。Suspended 不导出。
    pub fn views(&self, anchor_of: impl Fn(&TicketHolder) -> Option<Vec3>) -> Vec<TicketView> {
        let mut out = Vec::new();
        for (order, (holder, t)) in self.tickets.iter().enumerate() {
            if t.state == TicketState::Suspended {
                continue;
            }
            let Some(anchor) = anchor_of(&holder.0) else {
                match holder.0 {
                    TicketHolder::Observer(_) => panic!(
                        "观察者票 {holder:?} 无锚可派生(观察者已删而票未 End = 一致性违规, fail-stop;任务书 02 细则 3)"
                    ),
                    // 实体票无锚 = 终结窗口合法态(实体本拍已删、票在拍末 End;
                    // H1 生产接线暴露)。跳过该拍视图;I3(06)负责最终一致性。
                    _ => continue,
                }
            };
            out.push(TicketView {
                order: order as u32,
                priority: t.priority,
                data: t.data.clone(),
                collider: t.collider.clone(),
                wake: t.wake.clone(),
                state: t.state,
                anchor,
            });
        }
        out
    }

    /// 键序遍历。
    pub fn iter(&self) -> impl Iterator<Item = (TicketKey, &StoredTicket)> {
        self.tickets.iter().map(|(k, v)| (*k, v))
    }

    /// 键的名次(视图 order 同源;不存在 → None)。
    pub fn order_of(&self, key: TicketKey) -> Option<u32> {
        self.tickets
            .keys()
            .enumerate()
            .find(|(_, k)| **k == key)
            .map(|(i, _)| i as u32)
    }

    /// 键序名次 → 持有者(views 导出子集的按名次配对;`views()` 的 order
    /// 即全表键序名次,跳过条目不改变名次)。
    pub fn holder_of_order(&self, order: u32) -> TicketHolder {
        self.tickets
            .keys()
            .nth(order as usize)
            .map(|k| k.0)
            .expect("order 由 views() 生成,必在全表键序内")
    }

    /// 不变量 I3(REQ-007 §4.3.6/D14;任务书 06):违反即 panic(fail-stop),
    /// 消息含被违反条款编号 `I3-<n>`(n 为 §4.3.6 列表序号 1–4)与实体/票键。
    /// 调用点 = `Scheduler::tick` 拍末(应用 ticket_ops 与终结判定、推进时钟
    /// 之后),每拍恰一次;激活未启用不校验(D17:启用当拍的补签让 I3 从该
    /// 拍起成立)。输入(任务书签名固定):
    /// - `alive_or_in_transit`:存活(EntityOwnership)∪ 在途(Router 待投递
    ///   迁移信封)实体及其 actor 标记(在途取信封载荷——风险 2:迁移在途
    ///   窗口的 actor 以信封为准);
    /// - `frozen_actors`:冻结的常驻体集(滞回冻结,待地形不算——待地形的
    ///   常驻体票语义与醒着同,§4.4.2 状态表只对"冻结"行豁免);
    /// - `suspended_carried`:承载静止冻结的瞬态体集(其 TransientMotion 应
    ///   处于 Suspended;与 Suspend 操作同源的记录口径)。
    ///
    /// 复杂度(风险 7):O(票数·log)——每张实体票按持有者查 BTreeMap,无
    /// "每实体扫全表"的二重循环。
    pub fn check_invariants_i3(
        &self,
        alive_or_in_transit: &BTreeMap<EntityId, bool>,
        frozen_actors: &BTreeSet<EntityId>,
        suspended_carried: &BTreeSet<EntityId>,
    ) {
        use physics_sim::tickets::{TicketHolder, TicketKind};
        let actor_of = |e: EntityId| -> Option<bool> { alive_or_in_transit.get(&e).copied() };
        // 逐票检查(键序遍历,确定性 = 首个违反者按键序固定)。
        for (key, t) in self.tickets.iter() {
            let TicketHolder::Entity(e) = key.0 else {
                continue; // 观察者/区域票不属 I3(I3 全部关于实体票)
            };
            let kind = key.1;
            let Some(actor) = actor_of(e) else {
                // 第 2 条:实体票的持有者必须存活或在途(不存在指向已终结
                // 实体的票)。
                panic!(
                    "I3-2 违反:实体票 ({e:?}, {kind:?}) 的持有者既不存活也非迁移在途\
                     (票指向已终结实体,fail-stop;§4.3.6)"
                );
            };
            // 第 3 条:瞬态不持 ActorMotion、常驻不持 TransientMotion。
            if (kind == TicketKind::TransientMotion && actor)
                || (kind == TicketKind::ActorMotion && !actor)
            {
                panic!(
                    "I3-3 违反:票 ({e:?}, {kind:?}) 与持有者的 actor 标记不符 \
                     (actor = {actor};瞬态体不持 ActorMotion、常驻体不持 \
                     TransientMotion,fail-stop;§4.3.6)"
                );
            }
            // 第 4 条前半:冻结的常驻体不持 ActorMotion。
            if kind == TicketKind::ActorMotion && frozen_actors.contains(&e) {
                panic!(
                    "I3-4 违反:冻结的常驻体 {e:?} 持 ActorMotion 票 \
                     (冻结常驻体不持运动票,fail-stop;§4.3.6)"
                );
            }
            // 第 4 条后半:承载静止冻结的瞬态体,其 TransientMotion 应处于
            // Suspended(任何非 Suspended 状态都违反——挂起是 Suspend 操作
            // 的唯一合法落点)。
            if kind == TicketKind::TransientMotion
                && suspended_carried.contains(&e)
                && t.state != TicketState::Suspended
            {
                panic!(
                    "I3-4 违反:承载静止冻结的瞬态体 {e:?} 的 TransientMotion \
                     状态为 {:?}(应为 Suspended,fail-stop;§4.3.6)",
                    t.state
                );
            }
        }
        // 第 1 条:每个存活或在途的瞬态体恰好持一张 TransientMotion 票。
        // 按 BTreeMap 键有序;瞬态票种类唯一 ⇒ "有键"即"恰一张"。
        for (e, actor) in alive_or_in_transit {
            if *actor {
                continue;
            }
            let key = (TicketHolder::Entity(*e), TicketKind::TransientMotion);
            if !self.tickets.contains_key(&key) {
                panic!(
                    "I3-1 违反:存活的瞬态体 {e:?} 不持 TransientMotion 票 \
                     (每存活或在途瞬态体恰持一张,fail-stop;§4.3.6)"
                );
            }
        }
    }
}

/// 派生票据种类(签发侧的票种选择;§4.2.3 规则表——种类逻辑只允许出现在
/// 签发与转递,§4.2.2-1)。
pub fn kind_of(holder: &TicketHolder) -> TicketKind {
    match holder {
        TicketHolder::Observer(_) => TicketKind::Observer,
        TicketHolder::Entity(_) => TicketKind::TransientMotion, // 03 按状态细分
        TicketHolder::Area(_) => TicketKind::Area,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use physics_sim::tickets::{
        RegionSpec, TicketClock, TicketHolder, TicketKind, TicketState, WakeSpec,
    };
    use std::collections::BTreeSet;
    use types::EntityId;

    fn now() -> Tick {
        Tick(100)
    }

    fn ticket(priority: u8, ttl: Option<u64>) -> StoredTicket {
        StoredTicket {
            priority,
            data: RegionSpec::Radius {
                load_m: 32.0,
                unload_m: 48.0,
            },
            collider: RegionSpec::Radius {
                load_m: 32.0,
                unload_m: 48.0,
            },
            wake: WakeSpec::Radius {
                load_m: 24.0,
                unload_m: 32.0,
            },
            clock: TicketClock {
                issued_at: now(),
                active_ticks: 0,
                limit: ttl,
                starts_at: None,
            },
            state: TicketState::Active,
            center: Some(Vec3::new(64.0, 0.0, 64.0)),
        }
    }

    fn obs_key(src: u32) -> TicketKey {
        (
            TicketHolder::Observer(messaging::InputSource(src)),
            TicketKind::Observer,
        )
    }
    fn area_key(id: u64) -> TicketKey {
        (TicketHolder::Area(id), TicketKind::Area)
    }
    fn entity_key(n: u32) -> TicketKey {
        (
            TicketHolder::Entity(EntityId::from_parts(1, n)),
            TicketKind::TransientMotion,
        )
    }

    #[test]
    fn issue_end_idempotent() {
        let mut t = TicketTable::new();
        t.issue(entity_key(1), ticket(1, Some(100)), now());
        let mut t2 = t.clone();
        t2.advance_clocks();
        t2.advance_clocks();
        let before = t2.get(entity_key(1)).cloned().unwrap();
        t2.issue(entity_key(1), ticket(1, Some(100)), Tick(150));
        let after = t2.get(entity_key(1)).cloned().unwrap();
        assert_eq!(
            after.clock, before.clock,
            "实体票重发不重置时钟(D14):active_ticks 2 保持、issued_at 不变"
        );
        assert_eq!(after.state, before.state);
        let mut t3 = TicketTable::new();
        t3.end(entity_key(99));
        assert!(t3.get(entity_key(99)).is_none(), "End 不存在 = 无操作");
    }

    #[test]
    fn area_ticket_renewal_on_override() {
        let mut t = TicketTable::new();
        t.issue(area_key(7), ticket(1, Some(10)), now());
        t.advance_clocks();
        t.advance_clocks();
        assert_eq!(t.area_count(), 1);
        let mut renewed = ticket(1, Some(99));
        renewed.center = Some(Vec3::new(0.0, 0.0, 0.0));
        t.issue(area_key(7), renewed, Tick(200));
        let got = t.get(area_key(7)).unwrap();
        assert_eq!(got.clock.issued_at, Tick(200), "S3:覆盖 = 续期(时钟重置)");
        assert_eq!(got.clock.active_ticks, 0);
        assert_eq!(got.clock.limit, Some(99), "TTL 更新");
        assert_eq!(got.center, Some(Vec3::new(0.0, 0.0, 0.0)), "center 更新");
        assert_eq!(t.area_count(), 1, "覆盖不占新名额");
    }

    #[test]
    fn table_key_order_and_views() {
        let mut t = TicketTable::new();
        t.issue(area_key(1), ticket(1, None), now());
        t.issue(obs_key(1), ticket(0, None), now());
        t.issue(entity_key(1), ticket(1, None), now());
        let keys: Vec<TicketKey> = t.iter().map(|(k, _)| k).collect();
        let mut sorted = keys.clone();
        sorted.sort();
        assert_eq!(keys, sorted, "iter 键序");
        for (i, k) in keys.iter().enumerate() {
            assert_eq!(t.order_of(*k), Some(i as u32), "order = 键序名次");
        }
        let mut t2 = TicketTable::new();
        t2.issue(area_key(1), ticket(1, None), now());
        t2.issue(obs_key(1), ticket(0, None), now());
        let views = t2.views(|h| match h {
            TicketHolder::Observer(_) => Some(Vec3::new(8.0, 0.0, 8.0)),
            TicketHolder::Area(_) => Some(Vec3::new(64.0, 0.0, 64.0)),
            _ => None,
        });
        assert_eq!(views.len(), 2);
        assert_eq!(views[0].order, 0);
        assert_eq!(views[1].order, 1);
        // 键序:TicketHolder 声明序 Observer < Entity < Area → [0]=观察者票。
        assert_eq!(
            views[0].anchor,
            Vec3::new(8.0, 0.0, 8.0),
            "观察者票锚 = 派生"
        );
        assert_eq!(
            views[1].anchor,
            Vec3::new(64.0, 0.0, 64.0),
            "区域票锚 = center"
        );
        t2.set_state(area_key(1), TicketState::Suspended);
        let views2 = t2.views(|h| match h {
            TicketHolder::Observer(_) => Some(Vec3::new(8.0, 0.0, 8.0)),
            _ => Some(Vec3::ZERO),
        });
        assert_eq!(views2.len(), 1, "Suspended 不导出");
        assert_eq!(views2[0].order, 0, "名次连续(基于全表键序)");
    }

    #[test]
    fn clocks_advance_and_expire() {
        let mut t = TicketTable::new();
        t.issue(area_key(1), ticket(1, Some(3)), now());
        t.issue(area_key(2), ticket(1, Some(3)), now());
        t.set_state(area_key(2), TicketState::Paused);
        t.advance_clocks();
        t.advance_clocks();
        assert_eq!(t.get(area_key(1)).unwrap().clock.active_ticks, 2);
        assert_eq!(
            t.get(area_key(2)).unwrap().clock.active_ticks,
            0,
            "Paused 时钟停"
        );
        assert!(t.expired().is_empty());
        t.advance_clocks();
        assert_eq!(t.expired(), vec![area_key(1)], "active_ticks 3 ≥ limit 3");
        t.set_state(area_key(1), TicketState::Suspended);
        t.advance_clocks();
        assert_eq!(
            t.get(area_key(1)).unwrap().clock.active_ticks,
            3,
            "Suspended 时钟停"
        );
    }

    #[test]
    fn observer_ticket_equivalence() {
        let mut t2 = TicketTable::new();
        let mut obs = ticket(0, None);
        obs.center = None;
        t2.issue(obs_key(1), obs, now());
        let got = t2.get(obs_key(1)).unwrap();
        assert_eq!(
            got.data,
            RegionSpec::Radius {
                load_m: 32.0,
                unload_m: 48.0
            },
            "① 规格 == 现状半径"
        );
        assert_eq!(
            got.wake,
            WakeSpec::Radius {
                load_m: 24.0,
                unload_m: 32.0
            },
            "③ 规格 == 现状半径"
        );
        assert_eq!(got.priority, 0, "观察者票 priority 0");
        assert_eq!(got.center, None, "观察者票锚点派生,center 恒 None");
        let views = t2.views(|h| match h {
            TicketHolder::Observer(_) => Some(Vec3::new(8.0, 0.0, 8.0)),
            _ => None,
        });
        assert_eq!(views[0].anchor, Vec3::new(8.0, 0.0, 8.0));
    }

    #[test]
    fn area_ticket_radius_and_count_caps() {
        let mut t = TicketTable::new();
        for i in 0..64 {
            t.issue(area_key(i), ticket(1, None), now());
        }
        assert_eq!(t.area_count(), 64, "上限口径:64 张(注入值由 Session 校验)");
    }

    #[test]
    fn area_ticket_ttl_expires() {
        let mut t = TicketTable::new();
        t.issue(area_key(1), ticket(1, Some(5)), now());
        for _ in 0..5 {
            t.advance_clocks();
        }
        assert_eq!(t.expired(), vec![area_key(1)]);
        t.end(area_key(1));
        assert!(t.get(area_key(1)).is_none(), "到期 End");
        t.issue(area_key(1), ticket(1, Some(5)), Tick(500));
        for _ in 0..4 {
            t.advance_clocks();
        }
        assert!(t.expired().is_empty(), "续期后 4 拍 < 5 未到期");
        t.advance_clocks();
        assert_eq!(t.expired(), vec![area_key(1)], "续期后 TTL 重新起算");
    }

    #[test]
    fn suspended_ticket_not_keepalive() {
        let mut t = TicketTable::new();
        t.issue(area_key(1), ticket(1, None), now());
        t.set_state(area_key(1), TicketState::Suspended);
        let views = t.views(|h| match h {
            TicketHolder::Area(_) => Some(Vec3::ZERO),
            _ => None,
        });
        assert!(views.is_empty(), "Suspended 不导出 → 无区域占用");
        let before = t.get(area_key(1)).unwrap().clock.active_ticks;
        t.advance_clocks();
        assert_eq!(
            t.get(area_key(1)).unwrap().clock.active_ticks,
            before,
            "Suspended 时钟停"
        );
    }

    // ==== REQ-007 任务 06a(幂等矩阵 + I3 校验器)====

    fn actor_key(n: u32) -> TicketKey {
        (
            TicketHolder::Entity(EntityId::from_parts(1, n)),
            TicketKind::ActorMotion,
        )
    }

    /// 任务书测试 `shard/tickets::all_ops_idempotent_matrix`(T19/D14/§4.3.1):
    /// 6 操作 × (键已存在 / 不存在 / 已在目标态) 全矩阵——无 panic,状态与时钟
    /// 符合预期;同拍同票多操作结果 = 单次(不同路径重复操作不误报)。每个
    /// 分支失效(某操作非幂等)都会让对应断言变红(纪律 12)。
    #[test]
    fn all_ops_idempotent_matrix() {
        // ---- 实体票:Issue 键已在(不重置时钟、不改状态)----
        let mut t = TicketTable::new();
        t.issue(entity_key(1), ticket(1, Some(50)), now());
        t.set_state(entity_key(1), TicketState::Paused);
        for _ in 0..7 {
            t.advance_clocks();
        }
        let before = t.get(entity_key(1)).cloned().unwrap();
        // 新规格/新 issued_at 重发:必须被整体忽略(覆盖 = 重置时钟即违反 D14)。
        t.issue(entity_key(1), ticket(9, Some(9)), Tick(999));
        let after = t.get(entity_key(1)).cloned().unwrap();
        assert_eq!(
            after, before,
            "实体票 Issue 键已在 → 无操作(不重置时钟、不改状态、不换规格)"
        );

        // ---- End:存在 / 不存在 ----
        let mut t = TicketTable::new();
        t.issue(entity_key(2), ticket(1, None), now());
        t.end(entity_key(2));
        assert!(t.get(entity_key(2)).is_none(), "End 存在的票 → 删除");
        t.end(entity_key(2));
        t.end(actor_key(2));
        assert!(
            t.get(entity_key(2)).is_none(),
            "End 不存在 → 无操作不 panic"
        );

        // ---- Pause / Unpause:已在目标态 / 不存在 ----
        let mut t = TicketTable::new();
        t.issue(entity_key(3), ticket(1, None), now());
        t.advance_clocks();
        t.set_state(entity_key(3), TicketState::Paused);
        t.set_state(entity_key(3), TicketState::Paused); // 已在 Paused
        assert_eq!(t.get(entity_key(3)).unwrap().state, TicketState::Paused);
        t.set_state(actor_key(3), TicketState::Paused); // 票不存在
        assert!(t.get(actor_key(3)).is_none(), "Pause 不存在的票不凭空建票");
        t.set_state(entity_key(3), TicketState::Active); // Unpause 回 Active
        t.set_state(entity_key(3), TicketState::Active); // 已在 Active(同拍重复)
        assert_eq!(t.get(entity_key(3)).unwrap().state, TicketState::Active);

        // ---- Suspend / Resume:已在目标态 / 不存在;Resume 恢复 Active ----
        let mut t = TicketTable::new();
        t.issue(entity_key(4), ticket(1, None), now());
        t.set_state(entity_key(4), TicketState::Suspended);
        t.set_state(entity_key(4), TicketState::Suspended); // 已在 Suspended
        assert_eq!(t.get(entity_key(4)).unwrap().state, TicketState::Suspended);
        let ticks_at_suspend = t.get(entity_key(4)).unwrap().clock.active_ticks;
        for _ in 0..3 {
            t.advance_clocks();
        }
        assert_eq!(
            t.get(entity_key(4)).unwrap().clock.active_ticks,
            ticks_at_suspend,
            "Suspended 时钟停(重复 Suspend 不借状态切换推进时钟)"
        );
        t.set_state(entity_key(4), TicketState::Active); // Resume → Active
        t.set_state(entity_key(4), TicketState::Active); // 已在 Active
        assert_eq!(t.get(entity_key(4)).unwrap().state, TicketState::Active);
        t.set_state(actor_key(4), TicketState::Suspended); // 不存在
        t.set_state(actor_key(4), TicketState::Active);
        assert!(t.get(actor_key(4)).is_none(), "状态操作不存在的票不建票");

        // ---- 观察者/区域票:覆盖 = 续期(与实体票行为对照)----
        let mut t = TicketTable::new();
        t.issue(area_key(5), ticket(1, Some(3)), now());
        t.advance_clocks();
        t.issue(area_key(5), ticket(1, Some(30)), Tick(888));
        let got = t.get(area_key(5)).unwrap();
        assert_eq!(got.clock.issued_at, Tick(888), "区域票覆盖 = 续期(S3)");
        assert_eq!(got.clock.active_ticks, 0);
        assert_eq!(got.clock.limit, Some(30));

        // ---- 同拍同票多操作(不同路径)= 单次:先 End 后 Issue(键序下
        // End < 状态操作;这里按语义)净效果由最后的应用决定,不得 panic、
        // 不得出现双票或时钟错乱。常驻静止分片发 End、同拍调度器终结判定
        // 也 End(D14 的例子)→ 结果一致。
        let mut t = TicketTable::new();
        t.issue(actor_key(6), ticket(1, None), now());
        t.end(actor_key(6));
        t.end(actor_key(6));
        // End 后再签 = 新建(生产侧构造 StoredTicket 时自带 issued_at = 当前拍;
        // `issue` 的 now 参数只作用于观察者/区域票的覆盖续期)。
        let mut renewed = ticket(1, None);
        renewed.clock.issued_at = Tick(777);
        renewed.clock.active_ticks = 0;
        t.issue(actor_key(6), renewed, Tick(777));
        let got = t.get(actor_key(6)).unwrap();
        assert_eq!(
            got.clock.issued_at,
            Tick(777),
            "End 后重签 = 新票(时钟从重签拍起算);全程无 panic"
        );
    }

    // ---- I3 校验器单测(校验逻辑本体;生产接线由 e2e acc_06a_* 走)----

    fn e(n: u32) -> EntityId {
        EntityId::from_parts(1, n)
    }

    /// 四条规则各一条变异锚定:关掉对应规则(单测内手写违反场景)→ 红。
    /// 这里正向断言四类违反各自 panic 且消息含 `I3-<n>`。
    #[test]
    #[should_panic(expected = "I3-1")]
    fn i3_transient_without_ticket_panics() {
        let t = TicketTable::new();
        let alive = BTreeMap::from([(e(1), false)]);
        t.check_invariants_i3(&alive, &BTreeSet::new(), &BTreeSet::new());
    }

    #[test]
    #[should_panic(expected = "I3-2")]
    fn i3_ticket_to_dead_entity_panics() {
        let mut t = TicketTable::new();
        t.issue(entity_key(9), ticket(1, None), now());
        t.check_invariants_i3(&BTreeMap::new(), &BTreeSet::new(), &BTreeSet::new());
    }

    #[test]
    #[should_panic(expected = "I3-3")]
    fn i3_actor_with_transient_ticket_panics() {
        let mut t = TicketTable::new();
        t.issue(entity_key(1), ticket(1, None), now());
        let alive = BTreeMap::from([(e(1), true)]);
        t.check_invariants_i3(&alive, &BTreeSet::new(), &BTreeSet::new());
    }

    #[test]
    #[should_panic(expected = "I3-3")]
    fn i3_transient_with_actor_ticket_panics() {
        let mut t = TicketTable::new();
        t.issue(actor_key(1), ticket(1, None), now());
        let alive = BTreeMap::from([(e(1), false)]);
        t.check_invariants_i3(&alive, &BTreeSet::new(), &BTreeSet::new());
    }

    #[test]
    #[should_panic(expected = "I3-4")]
    fn i3_frozen_actor_with_motion_ticket_panics() {
        let mut t = TicketTable::new();
        t.issue(actor_key(1), ticket(1, None), now());
        let alive = BTreeMap::from([(e(1), true)]);
        let frozen = BTreeSet::from([e(1)]);
        t.check_invariants_i3(&alive, &frozen, &BTreeSet::new());
    }

    #[test]
    #[should_panic(expected = "I3-4")]
    fn i3_carried_frozen_transient_needs_suspended_panics() {
        let mut t = TicketTable::new();
        t.issue(entity_key(1), ticket(1, None), now());
        let alive = BTreeMap::from([(e(1), false)]);
        let carried = BTreeSet::from([e(1)]);
        t.check_invariants_i3(&alive, &BTreeSet::new(), &carried);
    }

    /// 合法态不误报:常驻持 ActorMotion(Active)、瞬态持 TransientMotion
    /// (Suspended 且在 carried 集)、冻结常驻无票——全部通过(无 panic)。
    #[test]
    fn i3_holds_on_legal_state() {
        let mut t = TicketTable::new();
        t.issue(entity_key(1), ticket(1, None), now()); // 瞬态
        t.set_state(entity_key(1), TicketState::Suspended);
        t.issue(actor_key(2), ticket(1, None), now()); // 常驻(醒着)
        t.issue(area_key(3), ticket(1, None), now()); // 区域票不参与
        let alive = BTreeMap::from([(e(1), false), (e(2), true)]);
        let frozen = BTreeSet::new(); // e2 常驻醒着,不在冻结表
        let carried = BTreeSet::from([e(1)]);
        t.check_invariants_i3(&alive, &frozen, &carried);
    }
}
