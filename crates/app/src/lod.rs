//! REQ-014 M04b-1 服务器 LOD 状态机与量化打包纯函数(零网络、零内核依赖)。

use std::collections::{BTreeMap, BTreeSet};

use sim_api::proto::{CompactAgentPose, LiteAgentPose};
use sim_api::quant;
use types::glam::Vec3;
use types::{EntityId, EntityState};

pub const K_L1: usize = 8;
pub const DWELL_TICKS: u64 = 30;
pub const L1_PERIOD: u64 = 6; // 10Hz
pub const L2_PERIOD: u64 = 30; // 2Hz
pub const FULL_PERIOD: u64 = 60; // L1 full / Meta 自愈周期
pub const L1_IN: f32 = 32.0;
pub const L1_OUT: f32 = 40.0;
pub const SWAP_MARGIN: f32 = 4.0;
pub const L2_OUT_MARGIN: f32 = 8.0;

/// 每连接 LOD 表。candidates 每拍全量传入(含视距外),消失即移除。
#[derive(Debug, Clone)]
pub struct LodTable {
    pub k_l1: usize,
    pub tier_of: BTreeMap<EntityId, u8>,
    pub lock_until: BTreeMap<EntityId, u64>,
    pub meta_sent: BTreeSet<EntityId>,
    pub force_send: BTreeSet<EntityId>,
    pub pending_removed: BTreeMap<u8, Vec<EntityId>>,
}

impl Default for LodTable {
    fn default() -> Self {
        Self::new(K_L1)
    }
}

/// 单拍 LOD 状态机事件输出(按 EntityId 升序)。
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LodEvents {
    pub need_meta: Vec<EntityId>,
    pub upgrades: Vec<EntityId>,
}

impl LodTable {
    /// 构造空 LOD 表,指定 L1 人群硬帽 `k_l1`。
    pub fn new(k_l1: usize) -> Self {
        Self {
            k_l1,
            tier_of: BTreeMap::new(),
            lock_until: BTreeMap::new(),
            meta_sent: BTreeSet::new(),
            force_send: BTreeSet::new(),
            pending_removed: BTreeMap::new(),
        }
    }

    /// 每拍一步。candidates = (id, 到观察者 3D 距离米),按 EntityId 升序;
    /// possessed = 本连接附身实体(可不在 candidates 中,忽略);view_m = R。
    /// 返回本拍事件(供接线发 Meta/removed)。幂等:同输入同输出。
    pub fn step(
        &mut self,
        tick: u64,
        candidates: &[(EntityId, f32)],
        possessed: Option<EntityId>,
        view_m: f32,
    ) -> LodEvents {
        let l1_in = L1_IN.min(view_m);
        let l1_out = L1_OUT.min(view_m + L2_OUT_MARGIN);
        let l2_in = view_m;
        let l2_out = view_m + L2_OUT_MARGIN;

        let cand_map: BTreeMap<EntityId, f32> = candidates.iter().copied().collect();

        // 1. 期望档(纯距离 + K_L1)
        let mut desired: BTreeMap<EntityId, Option<u8>> = BTreeMap::new();
        let possessed_present = possessed.filter(|id| cand_map.contains_key(id));
        if let Some(pid) = possessed_present {
            desired.insert(pid, Some(0));
        }

        // L1 集合选择(排除 possessed):
        // a. 成员 = 上拍 tier==1 且本拍 d < l1_out 者(32<d<40 留任=死区)。
        let mut members: Vec<(f32, EntityId)> = Vec::new();
        let mut member_ids: BTreeSet<EntityId> = BTreeSet::new();
        for (&id, &d) in &cand_map {
            if Some(id) == possessed_present {
                continue;
            }
            if self.tier_of.get(&id) == Some(&1) && d < l1_out {
                members.push((d, id));
                member_ids.insert(id);
            }
        }

        // b. 挑战者 = 非成员且 d <= l1_in,按 (d, EntityId) 升序。
        let mut challengers: Vec<(f32, EntityId)> = cand_map
            .iter()
            .filter(|&(&id, &d)| {
                Some(id) != possessed_present && !member_ids.contains(&id) && d <= l1_in
            })
            .map(|(&id, &d)| (d, id))
            .collect();
        challengers.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));

        // c. 逐个挑战者:若 |成员| < k_l1 -> 加入;否则取最远成员 f(按 (d, EntityId) 最大),
        //    若 d_ch + SWAP_MARGIN < d_f -> 逐 f 加 ch,否则 ch 不入。
        for (d_ch, id_ch) in challengers {
            if members.len() < self.k_l1 {
                members.push((d_ch, id_ch));
            } else if let Some((max_idx, &(d_f, _))) = members
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)))
                && d_ch + SWAP_MARGIN < d_f
            {
                members[max_idx] = (d_ch, id_ch);
            }
        }

        let l1_chosen: BTreeSet<EntityId> = members.iter().map(|&(_, id)| id).collect();
        for (&id, &d) in &cand_map {
            if Some(id) == possessed_present {
                continue;
            }
            if l1_chosen.contains(&id) {
                desired.insert(id, Some(1));
            } else {
                let prev = self.tier_of.get(&id).copied();
                if (prev == Some(2) && d < l2_out) || d <= l2_in {
                    desired.insert(id, Some(2));
                } else {
                    desired.insert(id, None);
                }
            }
        }

        // 2. 驻留锁(仅 L1<->L2) & 3. 事件与表更新
        let mut events = LodEvents::default();
        let mut all_ids: BTreeSet<EntityId> = self.tier_of.keys().copied().collect();
        all_ids.extend(cand_map.keys().copied());
        let mut touched_removed_tiers: BTreeSet<u8> = BTreeSet::new();

        for id in all_ids {
            let old_tier = self.tier_of.get(&id).copied();
            let des_tier = if cand_map.contains_key(&id) {
                desired.get(&id).copied().flatten()
            } else {
                None
            };

            let new_tier = match (old_tier, des_tier) {
                (Some(old @ (1 | 2)), Some(des @ (1 | 2)))
                    if old != des
                        && self.lock_until.get(&id).is_some_and(|&until| tick < until) =>
                {
                    Some(old)
                }
                _ => des_tier,
            };

            if new_tier != old_tier {
                if matches!(
                    (old_tier, new_tier),
                    (Some(1), Some(2)) | (Some(2), Some(1))
                ) {
                    self.lock_until.insert(id, tick.saturating_add(DWELL_TICKS));
                }

                match (old_tier, new_tier) {
                    (Some(old), None) => {
                        self.pending_removed.entry(old).or_default().push(id);
                        touched_removed_tiers.insert(old);
                        self.meta_sent.remove(&id);
                        self.force_send.remove(&id);
                        self.lock_until.remove(&id);
                        self.tier_of.remove(&id);
                    }
                    (old, Some(new)) => {
                        if new == 1 {
                            self.force_send.insert(id);
                            if old != Some(0) {
                                events.upgrades.push(id);
                            }
                        } else {
                            self.force_send.remove(&id);
                        }
                        if old.is_none() {
                            if !self.meta_sent.contains(&id) {
                                events.need_meta.push(id);
                            }
                            self.meta_sent.insert(id);
                        }
                        self.tier_of.insert(id, new);
                    }
                    (None, None) => {}
                }
            }
        }

        // 4. 输出顺序:need_meta/upgrades 已按 BTreeSet 升序;pending_removed 各档内按 EntityId 升序。
        for tier in touched_removed_tiers {
            if let Some(list) = self.pending_removed.get_mut(&tier) {
                list.sort();
                list.dedup();
            }
        }

        events
    }
}

/// L1 发射拍:`(tick + phase) % 6 == 0`。
pub fn l1_due(tick: u64, phase: u64) -> bool {
    tick.wrapping_add(phase).is_multiple_of(L1_PERIOD)
}

/// L1 full 拍:`(tick + phase) % 60 == 0`。
pub fn l1_full(tick: u64, phase: u64) -> bool {
    tick.wrapping_add(phase).is_multiple_of(FULL_PERIOD)
}

/// L2 发射拍:`(tick + phase) % 30 == 0`。
pub fn l2_due(tick: u64, phase: u64) -> bool {
    tick.wrapping_add(phase).is_multiple_of(L2_PERIOD)
}

/// Meta 周期自愈拍:`(tick + phase) % 60 == 0`。
pub fn meta_due(tick: u64, phase: u64) -> bool {
    tick.wrapping_add(phase).is_multiple_of(FULL_PERIOD)
}

/// 观察者世界坐标 → Lite 基准 chunk `(cx, cz)`(floor 向负无穷,非 trunc)。
pub fn observer_origin_chunk(observer: Vec3) -> (i32, i32) {
    (
        (observer.x / 16.0).floor() as i32,
        (observer.z / 16.0).floor() as i32,
    )
}

/// poses 非空(内核保证 >=1 连杆)。root = poses[0]。
pub fn pack_compact(entity: EntityId, poses: &[EntityState]) -> CompactAgentPose {
    let root = poses[0];
    let inv = root.rotation.conjugate();
    let child_links = poses[1..]
        .iter()
        .map(|link| {
            let rel_pos = inv * (link.translation - root.translation);
            let rel_rot = inv * link.rotation;
            quant::pack_child_link(rel_pos, rel_rot)
        })
        .collect();
    CompactAgentPose {
        entity,
        root_pos: root.translation.to_array(),
        root_rot: quant::pack_quat(root.rotation),
        root_linvel: quant::pack_linvel(root.linvel),
        child_links,
    }
}

/// origin_chunk = (cx, cz);frozen 来自 `session.frozen_agents()`。
pub fn pack_lite(
    entity: EntityId,
    root: &EntityState,
    cx: i32,
    cz: i32,
    frozen: bool,
) -> LiteAgentPose {
    let origin = Vec3::new(cx as f32 * 16.0, 0.0, cz as f32 * 16.0);
    let rel = root.translation - origin;
    let forward = root.rotation * Vec3::X;
    let yaw = (-forward.z).atan2(forward.x);
    let horizontal_speed = (root.linvel.x * root.linvel.x + root.linvel.z * root.linvel.z).sqrt();
    LiteAgentPose {
        entity,
        rel_pos: quant::pack_lite_pos(rel),
        yaw: quant::pack_yaw(yaw),
        motion_flags: quant::pack_motion_flags(horizontal_speed, frozen),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::glam::Quat;

    fn eid(seq: u32) -> EntityId {
        EntityId::from_parts(0, seq)
    }

    /// REQ-014 M04b-1 T1:档位与边界(k=8,R=128 及小视距 R=20)。
    #[test]
    fn m04b1_t1_tiers_and_boundaries() {
        let mut t = LodTable::new(8);
        let e1 = eid(1);
        let e2 = eid(2);
        let e3 = eid(3);
        let e4 = eid(4);
        let e5 = eid(5);
        let e_pos = eid(6);
        let e_absent = eid(7);

        t.step(
            1,
            &[
                (e1, 20.0),
                (e2, 100.0),
                (e3, 140.0),
                (e4, 32.0),
                (e5, 32.5),
                (e_pos, 200.0),
            ],
            Some(e_pos),
            128.0,
        );
        assert_eq!(t.tier_of.get(&e1), Some(&1), "d=20 -> L1");
        assert_eq!(t.tier_of.get(&e2), Some(&2), "d=100 -> L2");
        assert_eq!(t.tier_of.get(&e3), None, "d=140 -> L3");
        assert_eq!(t.tier_of.get(&e4), Some(&1), "d=32(<=) -> L1");
        assert_eq!(t.tier_of.get(&e5), Some(&2), "d=32.5 -> L2");
        assert_eq!(t.tier_of.get(&e_pos), Some(&0), "possessed d=200 -> L0");

        // possessed 不在 candidates -> 忽略。
        let mut t_abs = LodTable::new(8);
        t_abs.step(1, &[(e1, 20.0)], Some(e_absent), 128.0);
        assert_eq!(t_abs.tier_of.get(&e_absent), None, "缺席 possessed 忽略");
        assert_eq!(t_abs.tier_of.get(&e1), Some(&1));

        // 上拍 L1 者 d=39.9 -> 留 1;d=40 -> 2(>= 退出)。
        let mut t_l1 = LodTable::new(8);
        t_l1.step(1, &[(e1, 20.0), (e2, 20.0)], None, 128.0);
        t_l1.step(2, &[(e1, 39.9), (e2, 40.0)], None, 128.0);
        assert_eq!(t_l1.tier_of.get(&e1), Some(&1), "L1 d=39.9 留 1");
        assert_eq!(t_l1.tier_of.get(&e2), Some(&2), "L1 d=40 退 2");

        // 上拍 L2 者 d=135.9 -> 留 2;d=136 -> L3;新入 d=128 -> 2(<=R 进入)。
        let mut t_l2 = LodTable::new(8);
        t_l2.step(1, &[(e1, 100.0), (e2, 100.0)], None, 128.0);
        t_l2.step(2, &[(e1, 135.9), (e2, 136.0), (e3, 128.0)], None, 128.0);
        assert_eq!(t_l2.tier_of.get(&e1), Some(&2), "L2 d=135.9 留 2");
        assert_eq!(t_l2.tier_of.get(&e2), None, "L2 d=136 退 L3");
        assert_eq!(t_l2.tier_of.get(&e3), Some(&2), "d=128(<=R) 入 L2");

        // 小视距 R=20:l1_in=20, l1_out=28, l2_in=20, l2_out=28。
        // 上拍 L1 者 d=25 -> 留 1;d=29 -> L3;上拍 L2 者 d=27 -> 留 2;d=28 -> L3。
        let mut t_small = LodTable::new(8);
        t_small.step(1, &[(e1, 10.0), (e2, 10.0)], None, 20.0);
        assert_eq!(t_small.tier_of.get(&e1), Some(&1));
        assert_eq!(t_small.tier_of.get(&e2), Some(&1));
        t_small.step(2, &[(e1, 25.0), (e2, 29.0)], None, 20.0);
        assert_eq!(t_small.tier_of.get(&e1), Some(&1), "R=20 L1 d=25 留 1");
        assert_eq!(t_small.tier_of.get(&e2), None, "R=20 L1 d=29 退 L3");

        // 构造 R=20 下上拍 L2(用 k_l1=0 或通过已有 L2 状态:k=1 时第 2 只溢出 L2)。
        let mut t_small_l2 = LodTable::new(1);
        t_small_l2.step(1, &[(e1, 5.0), (e2, 15.0), (e3, 16.0)], None, 20.0);
        assert_eq!(t_small_l2.tier_of.get(&e2), Some(&2));
        assert_eq!(t_small_l2.tier_of.get(&e3), Some(&2));
        t_small_l2.step(2, &[(e1, 5.0), (e2, 27.0), (e3, 28.0)], None, 20.0);
        assert_eq!(t_small_l2.tier_of.get(&e2), Some(&2), "R=20 L2 d=27 留 2");
        assert_eq!(t_small_l2.tier_of.get(&e3), None, "R=20 L2 d=28 退 L3");
    }

    /// REQ-014 M04b-1 T2:驻留锁(k=8,R=128;仅 L1<->L2,30 拍;L0/L3 立即)。
    #[test]
    fn m04b1_t2_dwell_lock() {
        let mut t = LodTable::new(8);
        let e = eid(1);

        // e 在 d=20 入 L1(L3->L1 立即,无锁)。
        t.step(50, &[(e, 20.0)], None, 128.0);
        assert_eq!(t.tier_of.get(&e), Some(&1));
        assert_eq!(t.lock_until.get(&e), None, "L3->L1 不置锁");

        // tick=100 起 d=45:当拍即转 L2(此前无锁),lock_until=130。
        t.step(100, &[(e, 45.0)], None, 128.0);
        assert_eq!(t.tier_of.get(&e), Some(&2), "tick=100 应转 L2");
        assert_eq!(t.lock_until.get(&e), Some(&130));

        // tick=101 回 d=20:期望 L1 但被锁 -> 维持 L2 至 tick=129。
        for tick in 101..=129 {
            t.step(tick, &[(e, 20.0)], None, 128.0);
            assert_eq!(t.tier_of.get(&e), Some(&2), "tick={tick} 应维持 L2");
        }
        // tick=130 转 L1,lock_until=160。
        t.step(130, &[(e, 20.0)], None, 128.0);
        assert_eq!(t.tier_of.get(&e), Some(&1), "tick=130 应转 L1");
        assert_eq!(t.lock_until.get(&e), Some(&160));

        // tick=131 又 d=45:维持 L1 至 tick=159;tick=160 转 L2。
        for tick in 131..=159 {
            t.step(tick, &[(e, 45.0)], None, 128.0);
            assert_eq!(t.tier_of.get(&e), Some(&1), "tick={tick} 应维持 L1");
        }
        t.step(160, &[(e, 45.0)], None, 128.0);
        assert_eq!(t.tier_of.get(&e), Some(&2), "tick=160 应转 L2");

        // 对照:同场景 possessed 切换不受锁影响(L0 立即);d=200(->L3)亦无视锁立即 removed。
        t.step(161, &[(e, 45.0)], Some(e), 128.0);
        assert_eq!(t.tier_of.get(&e), Some(&0), "L0 绕过驻留锁立即生效");
        t.step(162, &[(e, 20.0)], None, 128.0);
        assert_eq!(t.tier_of.get(&e), Some(&1));

        // 构造带锁 L1 再骤移 d=200 -> 立即 removed。
        let mut t_l3 = LodTable::new(8);
        t_l3.step(1, &[(e, 100.0)], None, 128.0);
        t_l3.step(2, &[(e, 20.0)], None, 128.0);
        assert_eq!(t_l3.tier_of.get(&e), Some(&1));
        assert_eq!(t_l3.lock_until.get(&e), Some(&32));
        t_l3.step(3, &[(e, 200.0)], None, 128.0);
        assert_eq!(t_l3.tier_of.get(&e), None, "->L3 无视锁立即退出");
        assert_eq!(t_l3.pending_removed.get(&1), Some(&vec![e]));
    }

    /// REQ-014 M04b-1 T3:K_L1 帽与 4m 互换(k=2,R=128)。
    #[test]
    fn m04b1_t3_k_l1_cap_and_swap_margin() {
        let a = eid(1);
        let b = eid(2);
        let c = eid(3);

        // a(d=1), b(d=2) 入 L1;c(d=3) -> L2(帽满)。
        let mut t = LodTable::new(2);
        t.step(1, &[(a, 1.0), (b, 2.0), (c, 3.0)], None, 128.0);
        assert_eq!(t.tier_of.get(&a), Some(&1));
        assert_eq!(t.tier_of.get(&b), Some(&1));
        assert_eq!(t.tier_of.get(&c), Some(&2));

        // c 移到 d=0.5: 0.5+4=4.5 > 2 -> 不互换,c 仍 L2(负例)。
        t.step(2, &[(a, 1.0), (b, 2.0), (c, 0.5)], None, 128.0);
        assert_eq!(t.tier_of.get(&a), Some(&1));
        assert_eq!(t.tier_of.get(&b), Some(&1));
        assert_eq!(t.tier_of.get(&c), Some(&2), "0.5+4 > 2 不应互换");

        // 换帽场景:a(d=1), b(d=8) 入 L1;c(d=3):3+4=7 < 8 -> 逐 b 加 c;b->L2, pending_removed 空。
        let mut t2 = LodTable::new(2);
        t2.step(1, &[(a, 1.0), (b, 8.0)], None, 128.0);
        assert_eq!(t2.tier_of.get(&a), Some(&1));
        assert_eq!(t2.tier_of.get(&b), Some(&1));
        t2.step(2, &[(a, 1.0), (b, 8.0), (c, 3.0)], None, 128.0);
        assert_eq!(t2.tier_of.get(&a), Some(&1));
        assert_eq!(t2.tier_of.get(&c), Some(&1), "3+4 < 8 应逐 b 加 c");
        assert_eq!(t2.tier_of.get(&b), Some(&2), "b 降至 L2");
        assert!(t2.pending_removed.is_empty(), "换档不发 removed");

        // 平局:d 相同按 EntityId 升序优先。
        let mut t3 = LodTable::new(2);
        t3.step(1, &[(a, 5.0), (b, 5.0), (c, 5.0)], None, 128.0);
        assert_eq!(t3.tier_of.get(&a), Some(&1));
        assert_eq!(t3.tier_of.get(&b), Some(&1));
        assert_eq!(t3.tier_of.get(&c), Some(&2), "同距按 EntityId 升序优先");
    }

    /// REQ-014 M04b-1 T4:事件语义(need_meta/upgrades/force_send/removed/重入)与周期判定。
    #[test]
    fn m04b1_t4_event_semantics() {
        let mut t = LodTable::new(8);
        let e = eid(10);

        // 新实体 d=100 -> need_meta=[e], tier 2;下一拍同输入 -> need_meta 空(幂等)。
        let ev1 = t.step(1, &[(e, 100.0)], None, 128.0);
        assert_eq!(ev1.need_meta, vec![e]);
        assert!(ev1.upgrades.is_empty());
        assert_eq!(t.tier_of.get(&e), Some(&2));
        let ev2 = t.step(2, &[(e, 100.0)], None, 128.0);
        assert!(ev2.need_meta.is_empty(), "同输入 need_meta 幂等为空");

        // e 从 L2(d=100) -> d=20: upgrades=[e], force_send 含 e, pending_removed 空。
        let ev3 = t.step(3, &[(e, 20.0)], None, 128.0);
        assert!(ev3.need_meta.is_empty());
        assert_eq!(ev3.upgrades, vec![e]);
        assert!(t.force_send.contains(&e));
        assert!(t.pending_removed.is_empty());

        // e 从 L1 -> d=200: pending_removed[1]=[e], meta_sent 不含 e, tier_of 无 e。
        let ev4 = t.step(4, &[(e, 200.0)], None, 128.0);
        assert!(ev4.need_meta.is_empty());
        assert!(ev4.upgrades.is_empty());
        assert_eq!(t.pending_removed.get(&1), Some(&vec![e]));
        assert!(!t.meta_sent.contains(&e));
        assert!(!t.tier_of.contains_key(&e));
        assert!(!t.force_send.contains(&e));

        // e 再入 d=100: need_meta 再次=[e](L3 重入重发 Meta)。
        let ev5 = t.step(5, &[(e, 100.0)], None, 128.0);
        assert_eq!(ev5.need_meta, vec![e]);
        assert_eq!(t.tier_of.get(&e), Some(&2));

        // e 消失(candidates 不含,且带有驻留锁): pending_removed[旧档]=[e], 立即无视锁。
        t.step(6, &[(e, 20.0)], None, 128.0);
        assert_eq!(t.tier_of.get(&e), Some(&1));
        assert_eq!(t.lock_until.get(&e), Some(&36));
        t.pending_removed.clear();
        let ev7 = t.step(7, &[], None, 128.0);
        assert!(ev7.need_meta.is_empty());
        assert_eq!(t.pending_removed.get(&1), Some(&vec![e]));
        assert!(!t.tier_of.contains_key(&e));

        // R1 B1a: 0->1(解除附身)置 force_send 补闪,但不进 upgrades(降档)。
        t.step(8, &[(e, 20.0)], Some(e), 128.0);
        assert_eq!(t.tier_of.get(&e), Some(&0));
        let ev9 = t.step(9, &[(e, 20.0)], None, 128.0);
        assert_eq!(t.tier_of.get(&e), Some(&1));
        assert!(t.force_send.contains(&e), "0->1 必须置 force_send 补闪");
        assert!(ev9.upgrades.is_empty(), "0->1 是降档,不进 upgrades");

        // 周期纯函数校验。
        assert!(l1_due(5, 1));
        assert!(!l1_due(6, 1));
        assert!(l1_full(59, 1));
        assert!(!l1_full(60, 1));
        assert!(l2_due(29, 1));
        assert!(!l2_due(30, 1));
        assert!(meta_due(59, 1));
        assert!(!meta_due(60, 1));
    }

    /// REQ-014 M04b-1 T5:pack_compact golden(手算,M04a 锚复用)。
    #[test]
    fn m04b1_t5_pack_compact_golden() {
        // 1) capsule 单连杆。
        let root_cap = EntityState {
            translation: Vec3::new(8.0, -60.0, 8.0),
            rotation: Quat::IDENTITY,
            linvel: Vec3::new(1.2345, -0.0005, 32.9),
            angvel: Vec3::ZERO,
        };
        let cap = pack_compact(eid(1), &[root_cap]);
        assert_eq!(cap.entity, eid(1));
        assert_eq!(cap.root_pos, [8.0, -60.0, 8.0]);
        assert_eq!(cap.root_rot, [0x03, 0x00, 0x01, 0x80, 0x00, 0x40]);
        assert_eq!(cap.root_linvel, [0xD3, 0x04, 0xFF, 0xFF, 0xFF, 0x7F]);
        assert!(cap.child_links.is_empty());

        // 2) 双连杆:root IDENTITY 在原点,child translation (0, 0.5, -16.384) rotation IDENTITY。
        let root_id = EntityState::default();
        let child_id = EntityState {
            translation: Vec3::new(0.0, 0.5, -16.384),
            rotation: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        };
        let dual = pack_compact(eid(2), &[root_id, child_id]);
        assert_eq!(dual.child_links.len(), 1);
        assert_eq!(
            dual.child_links[0],
            [
                0x00, 0x80, 0xE8, 0x83, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00, 0x40
            ]
        );

        // 3) 根旋转非恒等:root rotation = from_rotation_y(pi/2) 在原点,
        //    child 世界位置 = (0, 0, -1), child rotation = root_rot。
        let r90 = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        let root_rot90 = EntityState {
            translation: Vec3::ZERO,
            rotation: r90,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        };
        let child_rot90 = EntityState {
            translation: Vec3::new(0.0, 0.0, -1.0),
            rotation: r90,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        };
        let rot_packed = pack_compact(eid(3), &[root_rot90, child_rot90]);
        assert_eq!(rot_packed.root_rot, [0x01, 0x00, 0x01, 0x80, 0xFF, 0x7F]);
        assert_eq!(rot_packed.child_links.len(), 1);
        assert_eq!(
            rot_packed.child_links[0],
            [
                0xD0, 0x87, 0x00, 0x80, 0x00, 0x80, 0x03, 0x00, 0x01, 0x80, 0x00, 0x40
            ]
        );
    }

    /// REQ-014 M04b-1 T6:pack_lite golden(手算)与负坐标 floor。
    #[test]
    fn m04b1_t6_pack_lite_golden() {
        let root = EntityState {
            translation: Vec3::new(-46.5, -2.25, 113.5),
            rotation: Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
            linvel: Vec3::new(1.0, 5.0, 0.0),
            angvel: Vec3::ZERO,
        };
        let entity = EntityId::from_parts(0, 300);
        let pose = pack_lite(entity, &root, -3, 7, true);
        assert_eq!(pose.entity, entity);
        assert_eq!(pose.rel_pos, [0x30, 0x80, 0xB8, 0x7F, 0x30, 0x80]);
        assert_eq!(pose.yaw, [0x00, 0x40]); // R1:yaw=真实 Y 旋转角(原 atan2(z,x) 符号反,架构师 M04b-1 公式错误)
        assert_eq!(pose.motion_flags, 0x94);
        assert_eq!(
            quant::pack_origin_chunk(-3, 7),
            [0xFD, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            observer_origin_chunk(Vec3::new(-0.5, 0.0, -0.5)),
            (-1, -1),
            "负坐标必须向负无穷 floor"
        );
    }
}
