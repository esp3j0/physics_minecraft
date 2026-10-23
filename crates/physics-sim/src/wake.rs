//! 统一唤醒的几何纯函数(REQ-007 任务 04b;任务书 04 接口固定)。
//!
//! rapier 不为 kinematic–kinematic 对生成接触——向上支撑闭包只能按几何判定:
//! 旋转包络 AABB(`half·√3` 外扩 `skin_m`)相交 + 中心更高,自种子递归向上。
//! rapier 类型不出 crate(§14 第 5 条)。

use std::collections::{BTreeMap, BTreeSet};

use types::{EntityId, glam::Vec3};

/// 旋转包络外扩的 AABB 半高(`half·√3`;stable 无 f32::consts::SQRT_3,常量展开)。
fn envelope_half(half: f32) -> f32 {
    half * SQRT_3
}

const SQRT_3: f32 = 1.732_050_8;

/// 向上支撑闭包(任务书 04 接口设计;§4.9 共同规则 2):
/// `seeds` 之上、贴靠(AABB 外扩 `skin_m` 相交)且中心更高的**冻结**实体,
/// 递归并入。邻接判定 = 旋转包络 AABB(`half·√3` 外扩 skin_m)相交(几何,
/// 不依赖 rapier 接触对)。返回 EntityId 升序(含 seeds;seeds 中不在 frozen
/// 表内的条目忽略)。
///
/// `frozen`:全部冻结实体的 `(id, center, half_extent)`;调用方保证不含醒着
/// 体/待地形体(风险 3:闭包成员只有冻结体)。
pub fn upward_closure(
    frozen: &[(EntityId, Vec3, f32)],
    seeds: &[EntityId],
    skin_m: f32,
) -> Vec<EntityId> {
    let by_id: std::collections::BTreeMap<EntityId, (Vec3, f32)> =
        frozen.iter().map(|(id, c, h)| (*id, (*c, *h))).collect();
    let mut group: BTreeSet<EntityId> = seeds
        .iter()
        .filter(|id| by_id.contains_key(id))
        .copied()
        .collect();
    // BFS 递归向上:组内任一成员的"上方贴靠体"并入,直到不动点。
    loop {
        let mut grew = false;
        let group_snapshot: Vec<EntityId> = group.iter().copied().collect();
        for member in group_snapshot {
            let Some((mc, mh)) = by_id.get(&member) else {
                continue;
            };
            for (id, (c, h)) in &by_id {
                if group.contains(id) {
                    continue;
                }
                if c.y <= mc.y {
                    continue; // 中心更高才并入(风险 3:不拉下方/旁侧)
                }
                // AABB(外扩 skin)相交:三轴都重叠。
                let overlap = |a: f32, ha: f32, b: f32, hb: f32| (a - b).abs() <= ha + hb + skin_m;
                if overlap(mc.x, envelope_half(*mh), c.x, envelope_half(*h))
                    && overlap(mc.y, *mh, c.y, envelope_half(*h))
                    && overlap(mc.z, envelope_half(*mh), c.z, envelope_half(*h))
                {
                    group.insert(*id);
                    grew = true;
                }
            }
        }
        if !grew {
            break;
        }
    }
    group.into_iter().collect()
}

/// 接触唤醒种子(任务 05a;任务书 05 接口设计;需求 §4.10.2):醒着实体 A 的
/// 扫掠体——旋转包络 AABB(`half·√3`)沿 `v_A` 扫掠 `lookahead_ticks` 拍、
/// 外扩 `skin_m`——与之相交的冻结体 B 为候选;冲击动量
/// `p_AB = m_A · max(0, v_A · n̂_AB)`(`n̂_AB` = A 中心指向 B 中心),同一 B
/// 被多个 A 命中取最大值;`p ≥ min_momentum` 才入选。静止接触 v≈0、p≈0,
/// 天然不触发(AC-T16)。返回 `(A, B, p_AB)`,按 B 升序(确定性;同一 B 恰一条)。
///
/// 扫掠体 = 起点与终点(沿 v 平移 `v·lookahead·dt`)两 AABB 的并包盒
/// (逐轴 min/max);零速度退化为静止 AABB 外扩。A、B 都用包络 AABB 判定
/// (宁可多选候选,不漏判——调用方还有动量门槛过滤)。
pub fn contact_wake_seeds(
    awake: &[(EntityId, Vec3, Vec3, f32, Vec3)], // (id, center, half, mass, linvel)
    frozen: &[(EntityId, Vec3, Vec3)],           // (id, center, half)
    lookahead_ticks: u64,
    skin_m: f32,
    min_momentum: f32,
    dt: f32,
) -> Vec<(EntityId, EntityId, f32)> {
    // AABB 相交(中心/半高对;skin 已并入外扩半高)。
    let overlap = |a: Vec3, ha: Vec3, b: Vec3, hb: Vec3| {
        (a.x - b.x).abs() <= ha.x + hb.x
            && (a.y - b.y).abs() <= ha.y + hb.y
            && (a.z - b.z).abs() <= ha.z + hb.z
    };
    let mut best: BTreeMap<EntityId, (EntityId, f32)> = BTreeMap::new();
    for (a_id, a_center, a_half, mass, v) in awake {
        let sweep = *v * (lookahead_ticks as f32 * dt);
        let reach = Vec3::new(sweep.x.abs(), sweep.y.abs(), sweep.z.abs());
        // 扫掠并包盒:静止包络 + 沿 v 两端的位移,再外扩 skin。
        let a_h = Vec3::new(
            a_half.x * SQRT_3 + reach.x + skin_m,
            a_half.y * SQRT_3 + reach.y + skin_m,
            a_half.z * SQRT_3 + reach.z + skin_m,
        );
        for (b_id, b_center, b_half) in frozen {
            let b_h = Vec3::new(
                b_half.x * SQRT_3 + skin_m,
                b_half.y * SQRT_3 + skin_m,
                b_half.z * SQRT_3 + skin_m,
            );
            if !overlap(*a_center, a_h, *b_center, b_h) {
                continue;
            }
            // 冲击动量:m·max(0, v·n̂),n̂ = A→B。相距过近(重合)时方向
            // 无定义,保守取 |v|(撞进同格 = 必撞)。
            let dir = *b_center - *a_center;
            let closing = if dir.length_squared() > 1e-12 {
                let n = dir / dir.length();
                v.dot(n).max(0.0)
            } else {
                v.length()
            };
            let p = mass * closing;
            if p < min_momentum {
                continue;
            }
            best.entry(*b_id)
                .and_modify(|(_, cur)| {
                    if p > *cur {
                        *cur = p;
                    }
                })
                .or_insert((*a_id, p));
        }
    }
    best.into_iter().map(|(b, (a, p))| (a, b, p)).collect()
}

/// 地形变更唤醒辅助(任务书 04 接口设计;04c 接线):改写格 `c` 与哪些
/// 冻结体旋转包络 AABB(外扩 `skin_m`)相交。命中表 EntityId 升序。
pub fn frozen_intersecting_cell(
    frozen: &[(EntityId, Vec3, f32)],
    c: types::BlockPos,
    skin_m: f32,
) -> Vec<EntityId> {
    let cell_center = Vec3::new(c.x as f32 + 0.5, c.y as f32 + 0.5, c.z as f32 + 0.5);
    let mut out = Vec::new();
    for (id, center, half) in frozen {
        let (ex, ey, ez) = (
            envelope_half(*half),
            envelope_half(*half),
            envelope_half(*half),
        );
        let overlap = |a: f32, b: f32, hb: f32| (a - b).abs() <= hb + 0.5 + skin_m;
        if overlap(center.x, cell_center.x, ex)
            && overlap(center.y, cell_center.y, ey)
            && overlap(center.z, cell_center.z, ez)
        {
            out.push(*id);
        }
    }
    out.sort_by_key(|id| *id);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::{EntityId, glam::Vec3};

    fn id(n: u32) -> EntityId {
        EntityId::from_parts(1, n)
    }

    /// 任务书测试 `wake::seeds_require_momentum_and_sweep`(05a;T14/T16):
    /// 扫掠命中;慢速/静止/远离不种子;动量门槛;同 B 多 A 取最大。
    /// 会变红的实现:扫掠体漏算位移(高速撞不到)、p 忘取 max(0,·)(远离
    /// 体的负点积成种子)、同 B 多 A 未取最大、门槛比较反向。
    #[test]
    fn seeds_require_momentum_and_sweep() {
        let dt = 1.0 / 60.0;
        let look = 2;
        let skin = 0.05_f32;
        let min_p = 1.0_f32;
        let b = (id(2), Vec3::new(5.0, 0.0, 0.0), Vec3::new(0.5, 0.5, 0.5));
        let frozen = [b];

        // 高速扫掠命中:A 静止 AABB(包络 0.866+skin)够不着 5 m 外的 B,
        // 沿 v 扫掠 2 拍(150 m/s → 5 m)后相交;p = 1 kg × 150 m/s。
        let fast = [(
            id(1),
            Vec3::ZERO,
            Vec3::new(0.5, 0.5, 0.5),
            1.0_f32,
            Vec3::new(150.0, 0.0, 0.0),
        )];
        assert_eq!(
            contact_wake_seeds(&fast, &frozen, look, skin, min_p, dt),
            vec![(id(1), id(2), 150.0)],
            "扫掠命中 + 动量 = m·v"
        );

        // 静止(同几何,v=0):扫掠退化为静止 AABB → 不相交 → 无种子。
        let still = [(
            id(1),
            Vec3::ZERO,
            Vec3::new(0.5, 0.5, 0.5),
            1.0_f32,
            Vec3::ZERO,
        )];
        assert!(
            contact_wake_seeds(&still, &frozen, look, skin, min_p, dt).is_empty(),
            "静止体够不着 5 m 外的 B(无扫掠不误报)"
        );

        // 贴靠静止(静止 AABB 相交,v=0 → p=0 < 门槛):T16 静止接触不唤醒。
        let touching = [(id(2), Vec3::new(1.5, 0.0, 0.0), Vec3::new(0.5, 0.5, 0.5))];
        assert!(
            contact_wake_seeds(&still, &touching, look, skin, min_p, dt).is_empty(),
            "静止贴靠相交但 p≈0,不成为种子"
        );

        // 慢速(扫掠后仍够不着):1 m/s 两拍位移 0.033 m,0.866+0.033+0.05 < 2。
        let far = [(id(2), Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.5, 0.5, 0.5))];
        let slow = [(
            id(1),
            Vec3::ZERO,
            Vec3::new(0.5, 0.5, 0.5),
            1.0_f32,
            Vec3::new(1.0, 0.0, 0.0),
        )];
        assert!(
            contact_wake_seeds(&slow, &far, look, skin, min_p, dt).is_empty(),
            "慢速扫掠仍够不着 → 无种子"
        );

        // 慢速但已贴近(相交)且动量足:p = 30 ≥ 1 → 种子(门槛量纲生效)。
        let near = [(id(2), Vec3::new(1.5, 0.0, 0.0), Vec3::new(0.5, 0.5, 0.5))];
        let slow_near = [(
            id(1),
            Vec3::ZERO,
            Vec3::new(0.5, 0.5, 0.5),
            1.0_f32,
            Vec3::new(30.0, 0.0, 0.0),
        )];
        assert_eq!(
            contact_wake_seeds(&slow_near, &near, look, skin, min_p, dt),
            vec![(id(1), id(2), 30.0)],
            "贴近 + 动量足 → 种子"
        );

        // 门槛:0.9 < 1 → 不入选(同一几何,只降速度)。
        let below = [(
            id(1),
            Vec3::ZERO,
            Vec3::new(0.5, 0.5, 0.5),
            1.0_f32,
            Vec3::new(0.9, 0.0, 0.0),
        )];
        assert!(
            contact_wake_seeds(&below, &near, look, skin, min_p, dt).is_empty(),
            "p < min_momentum → 不入选"
        );

        // 远离(v 背向 B):点积为负,max(0,·) 归零 → 无种子。
        let away = [(
            id(1),
            Vec3::ZERO,
            Vec3::new(0.5, 0.5, 0.5),
            1.0_f32,
            Vec3::new(-30.0, 0.0, 0.0),
        )];
        assert!(
            contact_wake_seeds(&away, &near, look, skin, min_p, dt).is_empty(),
            "远离 B 的实体不产生种子(负点积不取)"
        );

        // 同 B 多 A 取最大:A1 正对 30 m/s(m=1→p=30),A2 同速 m=0.5→p=15;
        // 返回恰一条且动量/撞击者取 p 更大者。
        let two = [
            (
                id(1),
                Vec3::ZERO,
                Vec3::new(0.5, 0.5, 0.5),
                1.0_f32,
                Vec3::new(30.0, 0.0, 0.0),
            ),
            (
                id(3),
                Vec3::ZERO,
                Vec3::new(0.5, 0.5, 0.5),
                0.5_f32,
                Vec3::new(30.0, 0.0, 0.0),
            ),
        ];
        assert_eq!(
            contact_wake_seeds(&two, &near, look, skin, min_p, dt),
            vec![(id(1), id(2), 30.0)],
            "同 B 多 A 取最大 p(与遍历顺序无关)"
        );
    }

    /// 任务书测试 `wake::upward_closure_stacks_and_gaps`(T12):竖叠全并;
    /// 间距 > skin 断开;更高才并入;含 seeds、EntityId 序。
    #[test]
    fn upward_closure_stacks_and_gaps() {
        let skin = 0.075_f32;
        // 竖叠:half 0.5,A @ y=10,B @ 11(贴靠),C @ 12(与 B 贴靠),
        // D @ 13.5(与 C 间隙 0.5 > skin → 断开),E @ 9(在 A 下方,不并入),
        // F @ (60, 11, 60)(旁侧,不并入)。
        let frozen = [
            (id(1), Vec3::new(8.0, 10.0, 8.0), 0.5_f32), // A(种子)
            (id(2), Vec3::new(8.0, 11.0, 8.0), 0.5),     // B(贴靠 A,更高)
            (id(3), Vec3::new(8.0, 12.0, 8.0), 0.5),     // C(贴靠 B,更高)
            (id(4), Vec3::new(8.0, 13.6, 8.0), 0.5),     // D(与 C 间隙 > skin)
            (id(5), Vec3::new(8.0, 9.0, 8.0), 0.5),      // E(A 下方)
            (id(6), Vec3::new(60.0, 11.0, 60.0), 0.5),   // F(旁侧)
        ];
        let out = upward_closure(&frozen, &[id(1)], skin);
        assert_eq!(
            out,
            vec![id(1), id(2), id(3)],
            "竖叠全并;间隙/下方/旁侧不并入;EntityId 升序"
        );
        // 多种子:seeds 含 E(下方)时 E 在组内(种子直接计入),但 F 仍不入。
        let out2 = upward_closure(&frozen, &[id(1), id(5)], skin);
        assert_eq!(out2, vec![id(1), id(2), id(3), id(5)], "含全部 seeds");
        // 种子不在 frozen 表内 → 忽略。
        let out3 = upward_closure(&frozen, &[id(9)], skin);
        assert!(out3.is_empty(), "种子必须为冻结体");
    }

    /// 任务书测试 `wake::closure_atomic_when_budget_short` 的几何前置:
    /// 大 half 实体贴靠判定(包络 half·√3 外扩)。
    #[test]
    fn closure_envelope_contact() {
        let skin = 0.075_f32;
        // half 0.25 的两块贴靠(中心距 0.5 = 2×half):包络半高 0.433,
        // y 重叠 |10−10.5| = 0.5 ≤ 0.433+0.433+skin ✓。
        let frozen = [
            (id(1), Vec3::new(0.0, 10.0, 0.0), 0.25_f32),
            (id(2), Vec3::new(0.0, 10.5, 0.0), 0.25),
        ];
        let out = upward_closure(&frozen, &[id(1)], skin);
        assert_eq!(out, vec![id(1), id(2)], "小方块贴靠并入(包络判定)");
        // 拉开 0.2(总间距 0.7 > 2×0.433+0.075=0.941?否——0.7 < 0.941 相交)。
        // 拉开到 1.0:|10−11| = 1.0 > 0.941 → 断开。
        let far = [
            (id(1), Vec3::new(0.0, 10.0, 0.0), 0.25_f32),
            (id(2), Vec3::new(0.0, 11.0, 0.0), 0.25),
        ];
        let out2 = upward_closure(&far, &[id(1)], skin);
        assert_eq!(out2, vec![id(1)], "间距超出包络+skin → 断开");
    }
}
