//! agent_pick —— 准星射线拾取已有主体与方块遮挡比较(REQ-013 C08 / V01 改写)。
//!
//! 纯逻辑、零渲染依赖:对每个主体的每个连杆每个碰撞形状取连杆局部 AABB,
//! 将世界射线经连杆位姿逆变换到连杆局部系后做 slab 相交测试,取最近命中。
//! 任何输入不 panic。

use types::glam::{Quat, Vec3};
use types::{BodyHash, EntityId, EntityState};

use crate::bodies::{BodyShapeTable, LinkShape};

/// 主体射线拾取命中。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AgentHit {
    pub entity: EntityId,
    pub dist: f32,
}

/// 准星综合拾取目标(方块 vs 主体)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Target {
    Block,
    Agent(AgentHit),
}

/// 计算连杆局部系下单个形状的 AABB `(min, max)`。
/// 凸包顶点为空或含非有限值时返回 `None`,调用方跳过而不 panic。
fn shape_local_aabb(shape: &LinkShape) -> Option<(Vec3, Vec3)> {
    let (min, max) = match shape {
        LinkShape::Box(half, off) => {
            let o = Vec3::from_array(*off);
            let h = Vec3::from_array(*half).abs();
            (o - h, o + h)
        }
        LinkShape::Sphere(r, off) => {
            let o = Vec3::from_array(*off);
            let h = Vec3::splat(r.abs());
            (o - h, o + h)
        }
        LinkShape::CapsuleY {
            half_height,
            radius,
            offset,
        } => {
            let o = Vec3::from_array(*offset);
            let r = radius.abs();
            let hy = half_height.abs() + r;
            let h = Vec3::new(r, hy, r);
            (o - h, o + h)
        }
        LinkShape::ConvexHull(points, off) => {
            let first = *points.first()?;
            let mut min = Vec3::from_array(first);
            let mut max = min;
            for p in points.iter().skip(1) {
                let v = Vec3::from_array(*p);
                min = min.min(v);
                max = max.max(v);
            }
            let o = Vec3::from_array(*off);
            (min + o, max + o)
        }
    };
    if min.is_finite() && max.is_finite() {
        Some((min, max))
    } else {
        None
    }
}

/// 射线 `o + t * d` (`t in [0, max_dist]`) 与轴对齐盒 `[min, max]` 的 slab 测试,返回进入 `t`。
fn ray_aabb_enter(origin: Vec3, dir: Vec3, min: Vec3, max: Vec3, max_dist: f32) -> Option<f32> {
    let mut tmin = 0.0_f32;
    let mut tmax = max_dist;
    for (o, d, mn, mx) in [
        (origin.x, dir.x, min.x, max.x),
        (origin.y, dir.y, min.y, max.y),
        (origin.z, dir.z, min.z, max.z),
    ] {
        if d == 0.0 {
            if o < mn || o > mx {
                return None;
            }
        } else {
            let inv_d = 1.0 / d;
            let mut t0 = (mn - o) * inv_d;
            let mut t1 = (mx - o) * inv_d;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            tmin = tmin.max(t0);
            tmax = tmax.min(t1);
            if tmin > tmax {
                return None;
            }
        }
    }
    if tmin.is_finite() && tmin >= 0.0 && tmin <= max_dist {
        Some(tmin)
    } else {
        None
    }
}

/// 沿世界射线 `p = origin + t * dir` (内部单位化;零向量 -> None),在 `t in [0, max_dist]`
/// 范围内拾取最近的主体;多主体平局取 `EntityId` 较小者;查不到身体哈希时跳过。
#[must_use]
pub fn pick_agent(
    agents: &[(EntityId, BodyHash, &[EntityState])],
    table: &BodyShapeTable,
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
) -> Option<AgentHit> {
    if !origin.is_finite() || !dir.is_finite() || !max_dist.is_finite() || max_dist < 0.0 {
        return None;
    }
    let len_sq = dir.length_squared();
    if len_sq <= 0.0 || !len_sq.is_finite() {
        return None;
    }
    let dir = dir / len_sq.sqrt();

    let mut best: Option<AgentHit> = None;

    for &(entity, hash, poses) in agents {
        let Some(link_shapes) = table.get(&hash) else {
            continue;
        };
        let mut agent_min: Option<f32> = None;

        for (pose, shapes) in poses.iter().zip(link_shapes.iter()) {
            if !pose.translation.is_finite() || !pose.rotation.is_finite() {
                continue;
            }
            let q_len_sq = pose.rotation.length_squared();
            let q_inv = if q_len_sq > 0.0 && q_len_sq.is_finite() {
                (pose.rotation / q_len_sq.sqrt()).conjugate()
            } else {
                Quat::IDENTITY
            };
            let o_local = q_inv * (origin - pose.translation);
            let d_local = q_inv * dir;

            for shape in shapes {
                let Some((min, max)) = shape_local_aabb(shape) else {
                    continue;
                };
                if let Some(t) = ray_aabb_enter(o_local, d_local, min, max, max_dist) {
                    agent_min = Some(match agent_min {
                        Some(cur) => cur.min(t),
                        None => t,
                    });
                }
            }
        }

        if let Some(dist) = agent_min {
            let better = match best {
                None => true,
                Some(cur) => dist < cur.dist || (dist == cur.dist && entity < cur.entity),
            };
            if better {
                best = Some(AgentHit { entity, dist });
            }
        }
    }

    best
}

/// 综合方块距离与主体命中:当且仅当 `agent.dist < block_dist.unwrap_or(f32::INFINITY)`
/// 时判定命中主体,否则为方块。
#[must_use]
pub fn pick_target(block_dist: Option<f32>, agent: Option<AgentHit>) -> Target {
    match agent {
        Some(hit) if hit.dist < block_dist.unwrap_or(f32::INFINITY) => Target::Agent(hit),
        _ => Target::Block,
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use types::{BodyDef, EntityId, EntityState};

    use super::*;

    fn eid(n: u32) -> EntityId {
        EntityId::from_parts(0, n)
    }

    fn pose(t: Vec3, q: Quat) -> EntityState {
        EntityState {
            translation: t,
            rotation: q,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        }
    }

    fn def_and_table(toml: &str) -> (BodyHash, BodyShapeTable) {
        let def = BodyDef::from_toml_str(toml).expect("测试 BodyDef TOML 合法");
        let hash = def.hash();
        let table = BodyShapeTable::from_defs([&def]);
        (hash, table)
    }

    const BOX_1_TOML: &str = r#"
name = "box1"
[[links]]
name = "root"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [1.0, 1.0, 1.0]
"#;

    const SPHERE_1_TOML: &str = r#"
name = "sphere1"
[[links]]
name = "root"
mass = 1.0
[[links.shapes]]
kind = "sphere"
radius = 1.0
"#;

    const BOX_211_TOML: &str = r#"
name = "box211"
[[links]]
name = "root"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [2.0, 1.0, 1.0]
"#;

    /// T1 盒:单连杆 box half=[1,1,1] off 0,连杆 t=(0,0,0) q=identity;
    /// origin=(-5,0,0) dir=(1,0,0) -> t=4。
    #[test]
    fn t1_box_hit_distance() {
        let (hash, table) = def_and_table(BOX_1_TOML);
        let poses = [pose(Vec3::ZERO, Quat::IDENTITY)];
        let agents = [(eid(1), hash, poses.as_slice())];
        let hit = pick_agent(
            &agents,
            &table,
            Vec3::new(-5.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            48.0,
        )
        .expect("T1 必命中");
        assert_eq!(hit.entity, eid(1));
        assert!(
            (hit.dist - 4.0).abs() <= 1e-5,
            "T1 期望 t=4,实得 {}",
            hit.dist
        );
    }

    /// T2 球:sphere r=1 off 0,连杆 t=(10,0,0);origin=(0,0,0) dir=(1,0,0) -> t=9。
    #[test]
    fn t2_sphere_hit_distance() {
        let (hash, table) = def_and_table(SPHERE_1_TOML);
        let poses = [pose(Vec3::new(10.0, 0.0, 0.0), Quat::IDENTITY)];
        let agents = [(eid(2), hash, poses.as_slice())];
        let hit = pick_agent(&agents, &table, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 48.0)
            .expect("T2 必命中");
        assert_eq!(hit.entity, eid(2));
        assert!(
            (hit.dist - 9.0).abs() <= 1e-5,
            "T2 期望 t=9,实得 {}",
            hit.dist
        );
    }

    /// T3 旋转盒:box half=[2,1,1],q=Quat::from_rotation_y(PI/2),t=0;
    /// origin=(0,0,-5) dir=(0,0,1) -> 局部 o'=(5,0,0)、d'=(-1,0,0),x slab 进入 (5-2)/1=3。
    #[test]
    fn t3_rotated_box_hit_distance() {
        let (hash, table) = def_and_table(BOX_211_TOML);
        let poses = [pose(Vec3::ZERO, Quat::from_rotation_y(FRAC_PI_2))];
        let agents = [(eid(3), hash, poses.as_slice())];
        let hit = pick_agent(
            &agents,
            &table,
            Vec3::new(0.0, 0.0, -5.0),
            Vec3::new(0.0, 0.0, 1.0),
            48.0,
        )
        .expect("T3 必命中");
        assert_eq!(hit.entity, eid(3));
        assert!(
            (hit.dist - 3.0).abs() <= 1e-4,
            "T3 期望 t=3,实得 {}",
            hit.dist
        );
    }

    /// T4 最近优先:两主体同轴盒中心 x=10 与 x=20(半长 1) -> 命中 x=10,t=9。
    #[test]
    fn t4_nearest_agent_wins() {
        let (hash, table) = def_and_table(BOX_1_TOML);
        let poses_far = [pose(Vec3::new(20.0, 0.0, 0.0), Quat::IDENTITY)];
        let poses_near = [pose(Vec3::new(10.0, 0.0, 0.0), Quat::IDENTITY)];
        // 刻意把远主体放前面,验证不是取第一个命中。
        let agents = [
            (eid(10), hash, poses_far.as_slice()),
            (eid(20), hash, poses_near.as_slice()),
        ];
        let hit = pick_agent(&agents, &table, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 48.0)
            .expect("T4 必命中");
        assert_eq!(hit.entity, eid(20));
        assert!(
            (hit.dist - 9.0).abs() <= 1e-5,
            "T4 期望 t=9,实得 {}",
            hit.dist
        );
    }

    /// T5 上限:T1 几何,max_dist=3.9 -> None;4.1 -> Some(t=4)。
    #[test]
    fn t5_max_dist_cutoff() {
        let (hash, table) = def_and_table(BOX_1_TOML);
        let poses = [pose(Vec3::ZERO, Quat::IDENTITY)];
        let agents = [(eid(5), hash, poses.as_slice())];
        assert!(
            pick_agent(
                &agents,
                &table,
                Vec3::new(-5.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                3.9,
            )
            .is_none(),
            "max_dist=3.9 不应命中 t=4 的盒"
        );
        let hit = pick_agent(
            &agents,
            &table,
            Vec3::new(-5.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            4.1,
        )
        .expect("max_dist=4.1 必命中");
        assert!((hit.dist - 4.0).abs() <= 1e-5);
    }

    /// T6 遮挡:block_dist=Some(3.0)+agent t=4 -> Block;None+agent -> Agent;
    /// Some(5.0)+agent t=4 -> Agent。
    #[test]
    fn t6_block_occlusion_comparison() {
        let agent = AgentHit {
            entity: eid(6),
            dist: 4.0,
        };
        assert_eq!(pick_target(Some(3.0), Some(agent)), Target::Block);
        assert_eq!(pick_target(None, Some(agent)), Target::Agent(agent));
        assert_eq!(pick_target(Some(5.0), Some(agent)), Target::Agent(agent));
        assert_eq!(pick_target(None, None), Target::Block);
    }

    /// T7 零向量/空表/未知哈希:不 panic 且返回 None。
    #[test]
    fn t7_zero_dir_and_empty_inputs_never_panic() {
        let (hash, table) = def_and_table(BOX_1_TOML);
        let poses = [pose(Vec3::ZERO, Quat::IDENTITY)];
        let agents = [(eid(7), hash, poses.as_slice())];

        assert!(pick_agent(&agents, &table, Vec3::new(-5.0, 0.0, 0.0), Vec3::ZERO, 48.0).is_none());
        assert!(pick_agent(&[], &table, Vec3::new(-5.0, 0.0, 0.0), Vec3::X, 48.0).is_none());

        let unknown = BodyHash::fnv1a(b"unknown-body");
        let unknown_agents = [(eid(8), unknown, poses.as_slice())];
        assert!(
            pick_agent(
                &unknown_agents,
                &table,
                Vec3::new(-5.0, 0.0, 0.0),
                Vec3::X,
                48.0,
            )
            .is_none()
        );
    }
}
