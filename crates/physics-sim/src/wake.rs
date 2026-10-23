//! 统一唤醒的几何纯函数(REQ-007 任务 04b;任务书 04 接口固定)。
//!
//! rapier 不为 kinematic–kinematic 对生成接触——向上支撑闭包只能按几何判定:
//! 旋转包络 AABB(`half·√3` 外扩 `skin_m`)相交 + 中心更高,自种子递归向上。
//! rapier 类型不出 crate(§14 第 5 条)。

use std::collections::BTreeSet;

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
