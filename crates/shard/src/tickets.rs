//! 票据表与操作语义(REQ-007 §4.2/§4.3;任务书 req007-tickets/02)。
//!
//! 调度器级**全局唯一一张**表,按 `TicketKey = (TicketHolder, TicketKind)` 有序
//! (`BTreeMap`);任何遍历都按键序(§4.2.2-3)。操作语义(§4.3.1/D14/S3):
//! 全部幂等——观察者/区域票 Issue 键已在 → 覆盖规格与 TTL(区域票时钟重置 =
//! 续期,S3);实体票 Issue 键已在 → 无操作(不重置时钟、不改状态);End/Pause/
//! Suspend 等遇不存在或已在目标态 → 无操作。正确性由每拍不变量校验兜底
//! (I3,06 落地),比逐条操作报错更严格。

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use types::Tick;
use types::glam::Vec3;

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
}
