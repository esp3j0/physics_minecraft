//! present —— 客户端主体表现状态机(REQ-014 M04b-2)。
//!
//! 纯逻辑、零 bevy、零墙钟(`dt_secs` 由调用方传入):
//! - tier 2(Lite)按水平速率本地积分摆腿相位并经 [`crate::bodies::swing_link_transforms`] 展开连杆;
//! - 档位变化时在 [`BLEND_SECS`] (150ms) 内逐连杆 lerp/slerp 平滑混合;
//! - 首帧无混合,tier 0/1 无摆腿(单机/附身旁路逐位等价)。

use std::collections::{BTreeMap, BTreeSet};
use std::f32::consts::TAU;

use types::glam::Vec3;
use types::{BodyDef, EntityId, EntityState};

use crate::bodies::{STRIDE_LENGTH, swing_link_transforms};
use crate::entities::AgentView;

/// 换档平滑混合时长(秒;REQ-014 §4 ≤ 150ms)。
pub const BLEND_SECS: f32 = 0.15;

/// 单个主体的表现状态(当前档位、跨档保留摆腿相位、进行中的换档混合、上帧输出)。
#[derive(Debug, Clone)]
pub struct AgentPresentation {
    tier: u8,
    phase: f32,
    blend: Option<(Vec<EntityState>, f32)>,
    last: Vec<EntityState>,
}

/// 全部主体的表现状态表。
#[derive(Debug, Default, Clone)]
pub struct PresentationTable {
    pub agents: BTreeMap<EntityId, AgentPresentation>,
}

impl PresentationTable {
    /// 推进单主体一帧表现状态并返回渲染用连杆姿态。
    pub fn advance(
        &mut self,
        id: EntityId,
        resolved: &AgentView,
        tier: u8,
        speed: f32,
        def: Option<&BodyDef>,
        dt_secs: f32,
    ) -> Vec<EntityState> {
        let entry = self.agents.entry(id).or_insert_with(|| AgentPresentation {
            tier,
            phase: 0.0,
            blend: None,
            last: Vec::new(),
        });

        let target: Vec<EntityState> = if tier == 2
            && let Some(def) = def
            && resolved.poses.len() == def.links.len()
            && !def.links.is_empty()
        {
            entry.phase = (entry.phase + speed * TAU / STRIDE_LENGTH * dt_secs) % TAU;
            let rel = swing_link_transforms(def, entry.phase);
            let root = resolved.poses[0];
            let mut poses = Vec::with_capacity(rel.len());
            poses.push(root);
            for &(rel_pos, rel_rot) in rel.iter().skip(1) {
                poses.push(EntityState {
                    translation: root.translation + root.rotation * rel_pos,
                    rotation: root.rotation * rel_rot,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                });
            }
            poses
        } else {
            resolved.poses.clone()
        };

        if entry.tier != tier {
            if entry.last.len() == target.len() && !target.is_empty() {
                entry.blend = Some((entry.last.clone(), 0.0));
            } else {
                entry.blend = None;
            }
            entry.tier = tier;
        }

        let out = if let Some((from, mut elapsed)) = entry.blend.take() {
            if from.len() == target.len() {
                elapsed += dt_secs;
                let alpha = (elapsed / BLEND_SECS).min(1.0);
                if alpha >= 1.0 {
                    target
                } else {
                    let blended: Vec<EntityState> = from
                        .iter()
                        .zip(&target)
                        .map(|(a, b)| EntityState {
                            translation: a.translation.lerp(b.translation, alpha),
                            rotation: a.rotation.slerp(b.rotation, alpha),
                            linvel: b.linvel,
                            angvel: b.angvel,
                        })
                        .collect();
                    entry.blend = Some((from, elapsed));
                    blended
                }
            } else {
                target
            }
        } else {
            target
        };

        entry.last = out.clone();
        out
    }

    /// 移除指定主体的表现状态。
    pub fn remove(&mut self, id: EntityId) {
        self.agents.remove(&id);
    }

    /// 仅保留 `keep` 集合内的主体表现状态。
    pub fn retain(&mut self, keep: &BTreeSet<EntityId>) {
        self.agents.retain(|id, _| keep.contains(id));
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::{FRAC_PI_2, PI};

    use types::BodyHash;
    use types::glam::Quat;

    use super::*;

    fn eid(n: u32) -> EntityId {
        EntityId::from_parts(0, n)
    }

    fn two_link_def() -> BodyDef {
        let toml = r#"
name = "two_link_present"
[[links]]
name = "base"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[links]]
name = "leg"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[joints]]
parent = "base"
child = "leg"
type = "revolute"
axis = [0.0, 1.0, 0.0]
anchor = [0.0, 0.0, 0.0]
child_anchor = [-1.0, 0.0, 0.0]
default_angle = 0.0
"#;
        BodyDef::from_toml_str(toml).expect("two_link_present 合法")
    }

    fn single_view(x: f32, rot: Quat, linvel: Vec3, angvel: Vec3) -> AgentView {
        AgentView {
            hash: BodyHash::fnv1a(b"single"),
            poses: vec![EntityState {
                translation: Vec3::new(x, 0.0, 0.0),
                rotation: rot,
                linvel,
                angvel,
            }],
        }
    }

    fn assert_vec3_near(actual: Vec3, expected: Vec3, tol: f32, label: &str) {
        let d = (actual - expected).abs();
        assert!(
            d.x <= tol && d.y <= tol && d.z <= tol,
            "{label}: actual={actual:?}, expected={expected:?}, tol={tol}"
        );
    }

    fn assert_quat_near(actual: Quat, expected: Quat, tol: f32, label: &str) {
        assert!(
            (actual.x - expected.x).abs() <= tol
                && (actual.y - expected.y).abs() <= tol
                && (actual.z - expected.z).abs() <= tol
                && (actual.w - expected.w).abs() <= tol,
            "{label}: actual={actual:?}, expected={expected:?}, tol={tol}"
        );
    }

    /// REQ-014 M04b-2 T10:换档 150ms 分三步 lerp/slerp 混合与速度取 target。
    #[test]
    fn blend_thirds_and_slerp() {
        let mut table = PresentationTable::default();
        let id = eid(1);
        let lv = Vec3::new(1.5, -0.5, 2.0);
        let av = Vec3::new(0.0, 0.25, 0.0);

        let v0 = single_view(0.0, Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);
        let out0 = table.advance(id, &v0, 1, 0.0, None, 0.05);
        assert_eq!(out0[0].translation.x, 0.0);

        let v1 = single_view(10.0, Quat::from_rotation_y(FRAC_PI_2), lv, av);
        let out1 = table.advance(id, &v1, 2, 0.0, None, 0.05);
        assert!((out1[0].translation.x - 10.0 / 3.0).abs() <= 1e-4);
        assert_quat_near(
            out1[0].rotation,
            Quat::from_rotation_y(FRAC_PI_2 / 3.0),
            1e-4,
            "alpha=1/3 rot 30deg",
        );
        assert_eq!(out1[0].linvel, lv, "linvel 取 target 逐位");
        assert_eq!(out1[0].angvel, av, "angvel 取 target 逐位");

        let out2 = table.advance(id, &v1, 2, 0.0, None, 0.05);
        assert!((out2[0].translation.x - 20.0 / 3.0).abs() <= 1e-4);

        let out3 = table.advance(id, &v1, 2, 0.0, None, 0.05);
        assert!((out3[0].translation.x - 10.0).abs() <= 1e-4);
        assert!(table.agents[&id].blend.is_none(), "alpha>=1 后 blend 结束");

        let out4 = table.advance(id, &v1, 2, 0.0, None, 0.05);
        assert!((out4[0].translation.x - 10.0).abs() <= 1e-4);
    }

    /// REQ-014 M04b-2 T11:换档时连杆数突变直接切换,不做混合。
    #[test]
    fn blend_len_mismatch_snaps() {
        let mut table = PresentationTable::default();
        let id = eid(2);
        let v1 = single_view(0.0, Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);
        let _ = table.advance(id, &v1, 1, 0.0, None, 0.05);

        let v2 = AgentView {
            hash: BodyHash::fnv1a(b"two"),
            poses: vec![
                EntityState {
                    translation: Vec3::new(10.0, 0.0, 0.0),
                    rotation: Quat::IDENTITY,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                },
                EntityState {
                    translation: Vec3::new(11.0, 0.0, 0.0),
                    rotation: Quat::IDENTITY,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                },
            ],
        };
        let out = table.advance(id, &v2, 2, 0.0, None, 0.05);
        assert_eq!(out, v2.poses, "长度突变无混合,逐位等于 target");
    }

    /// REQ-014 M04b-2 T12:tier2 摆腿相位按 speed*TAU/STRIDE_LENGTH*dt 积分,speed=0 冻结。
    #[test]
    fn swing_phase_accumulates() {
        let def = two_link_def();
        let hash = def.hash();
        let mut table = PresentationTable::default();
        let id = eid(3);
        let resolved = AgentView {
            hash,
            poses: vec![
                EntityState {
                    translation: Vec3::new(5.0, 0.0, 0.0),
                    rotation: Quat::IDENTITY,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                },
                EntityState {
                    translation: Vec3::new(6.0, 0.0, 0.0),
                    rotation: Quat::IDENTITY,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                },
            ],
        };

        let out1 = table.advance(id, &resolved, 2, 0.8, Some(&def), 0.1);
        let theta1 = -0.35_f32 * (PI / 5.0).sin();
        assert_vec3_near(
            out1[1].translation,
            Vec3::new(5.0 + theta1.cos(), 0.0, -theta1.sin()),
            1e-4,
            "phase=pi/5 child",
        );

        let out2 = table.advance(id, &resolved, 2, 0.8, Some(&def), 0.1);
        let theta2 = -0.35_f32 * (2.0 * PI / 5.0).sin();
        assert_vec3_near(
            out2[1].translation,
            Vec3::new(5.0 + theta2.cos(), 0.0, -theta2.sin()),
            1e-4,
            "phase=2pi/5 child",
        );

        let out3 = table.advance(id, &resolved, 2, 0.0, Some(&def), 0.1);
        assert_eq!(out3[1], out2[1], "speed=0 时相位冻结,child 逐位不变");
    }

    /// REQ-014 M04b-2 T13:tier 0 旁路逐位等价(含非零 linvel/angvel)。
    #[test]
    fn tier0_passthrough_bitwise() {
        let def = two_link_def();
        let mut table = PresentationTable::default();
        let id = eid(4);
        let resolved = AgentView {
            hash: def.hash(),
            poses: vec![
                EntityState {
                    translation: Vec3::new(1.25, -2.5, 3.75),
                    rotation: Quat::from_rotation_y(0.4),
                    linvel: Vec3::new(1.0, 2.0, 3.0),
                    angvel: Vec3::new(-0.5, 0.25, 0.75),
                },
                EntityState {
                    translation: Vec3::new(2.25, -2.5, 3.75),
                    rotation: Quat::from_rotation_y(0.8),
                    linvel: Vec3::new(0.1, 0.2, 0.3),
                    angvel: Vec3::new(0.4, 0.5, 0.6),
                },
            ],
        };
        let out = table.advance(id, &resolved, 0, 2.0, Some(&def), 0.05);
        assert_eq!(out, resolved.poses, "tier 0 必须与输入逐位相等");
    }

    /// REQ-014 M04b-2 T15:首帧以当前档初始化,无混合起点。
    #[test]
    fn first_entry_no_blend() {
        let def = two_link_def();
        let mut table = PresentationTable::default();
        let id = eid(5);
        let resolved = AgentView {
            hash: def.hash(),
            poses: vec![
                EntityState {
                    translation: Vec3::new(3.0, 0.0, 0.0),
                    rotation: Quat::IDENTITY,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                },
                EntityState {
                    translation: Vec3::new(4.0, 0.0, 0.0),
                    rotation: Quat::IDENTITY,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                },
            ],
        };
        let out = table.advance(id, &resolved, 2, 0.0, Some(&def), 0.05);
        assert!(table.agents[&id].blend.is_none());
        assert_vec3_near(out[0].translation, Vec3::new(3.0, 0.0, 0.0), 1e-6, "root");
        assert_vec3_near(out[1].translation, Vec3::new(4.0, 0.0, 0.0), 1e-6, "child");
    }

    /// REQ-014 M04b-2 T14:remove 后重建首帧无混合;retain 仅保留 keep 集。
    #[test]
    fn remove_retain() {
        let mut table = PresentationTable::default();
        let id1 = eid(10);
        let id2 = eid(20);
        let v0 = single_view(0.0, Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);
        let v10 = single_view(10.0, Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);

        let _ = table.advance(id1, &v0, 1, 0.0, None, 0.05);
        let _ = table.advance(id2, &v0, 1, 0.0, None, 0.05);
        assert_eq!(table.agents.len(), 2);

        table.remove(id1);
        assert!(!table.agents.contains_key(&id1));
        // remove 后再次 advance(tier2) 应视为首帧无混合,直出 x=10。
        let out = table.advance(id1, &v10, 2, 0.0, None, 0.05);
        assert_eq!(out[0].translation.x, 10.0);

        let keep = BTreeSet::from([id2]);
        table.retain(&keep);
        assert_eq!(table.agents.keys().copied().collect::<Vec<_>>(), vec![id2]);
    }

    /// REQ-014 M04b-2 R2 T23:相位跨档 2→1→2 保留不归零 + tier 1 无摆腿(混合感知四步序列)。
    #[test]
    fn phase_survives_tier_roundtrip() {
        let def = two_link_def();
        let hash = def.hash();
        let mut table = PresentationTable::default();
        let id = eid(30);
        let resolved = AgentView {
            hash,
            poses: vec![
                EntityState {
                    translation: Vec3::new(5.0, 0.0, 0.0),
                    rotation: Quat::IDENTITY,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                },
                EntityState {
                    translation: Vec3::new(6.0, 0.0, 0.0),
                    rotation: Quat::IDENTITY,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                },
            ],
        };

        let theta1 = -0.35_f32 * (PI / 5.0).sin();
        let child1 = Vec3::new(5.0 + theta1.cos(), 0.0, -theta1.sin());
        let theta2 = -0.35_f32 * (2.0 * PI / 5.0).sin();
        let child2 = Vec3::new(5.0 + theta2.cos(), 0.0, -theta2.sin());

        let out1 = table.advance(id, &resolved, 2, 0.8, Some(&def), 0.1);
        assert_vec3_near(out1[1].translation, child1, 1e-4, "step1 phase=pi/5 child");

        let out2 = table.advance(id, &resolved, 1, 0.0, Some(&def), 0.15);
        assert_eq!(
            out2, resolved.poses,
            "step2 tier 1 混合完成逐位等于 resolved.poses"
        );

        let out3 = table.advance(id, &resolved, 2, 0.0, Some(&def), 0.15);
        assert_vec3_near(
            out3[1].translation,
            child1,
            1e-4,
            "step3 tier 2 speed=0 相位跨档保留仍为 theta1",
        );

        let out4 = table.advance(id, &resolved, 2, 0.8, Some(&def), 0.1);
        assert_vec3_near(
            out4[1].translation,
            child2,
            1e-4,
            "step4 phase=2pi/5 child 累积不归零",
        );
    }
}
