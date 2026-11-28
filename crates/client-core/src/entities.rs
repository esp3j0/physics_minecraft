//! entities —— 客户端主体与动态刚体/脱落簇纯逻辑镜像(REQ-013 C03-1 D2)。
//!
//! 零 bevy、零墙钟,无头可测:消费 `S2C::AgentStates`/`S2C::EntityStates`/`S2C::EntityShapes`,
//! 维护 `agents`/`solids`/`shapes` 三张权威镜像表供渲染层每帧对账。

use std::collections::{BTreeMap, BTreeSet};

use sim_api::proto::S2C;
use sim_api::quant;
use types::glam::{Quat, Vec3};
use types::{BodyDef, BodyHash, CompoundShape, EntityId, EntityState};

/// 动态实体单格渲染边长(m;镜像 `viewer/src/render.rs:55` `ENTITY_CUBE_SIZE`,同源)。
pub const ENTITY_CUBE_SIZE: f32 = 0.9;

/// 主体视图:身体哈希 + 按 `BodyDef.links` 排序的最新连杆位姿。
#[derive(Debug, Clone, PartialEq)]
pub struct AgentView {
    pub hash: BodyHash,
    pub poses: Vec<EntityState>,
}

/// 动态刚体/脱落簇视图:最新位姿 + 可选复合体形状(`None` 表示单格方块实体)。
#[derive(Debug, Clone, PartialEq)]
pub struct SolidView {
    pub state: EntityState,
    pub compound: Option<CompoundShape>,
}

/// 主体/刚体/复合形状镜像(全量重建、增量 upsert、removed 同步清表,支持 REQ-014 分档 LOD 流)。
#[derive(Debug, Default, Clone)]
pub struct EntityMirror {
    pub agents: BTreeMap<EntityId, AgentView>,
    pub solids: BTreeMap<EntityId, SolidView>,
    pub shapes: BTreeMap<EntityId, CompoundShape>,
    pub meta: BTreeMap<EntityId, BodyHash>,
    pub agent_tiers: BTreeMap<EntityId, u8>,
    pub agent_speed: BTreeMap<EntityId, f32>,
    #[allow(dead_code)]
    defs: BTreeMap<BodyHash, BodyDef>,
    statics: BTreeMap<BodyHash, Vec<(Vec3, Quat)>>,
}

impl EntityMirror {
    pub fn new() -> Self {
        Self::default()
    }

    /// 注入身体定义(启动时一次):存 `defs` 并预算 `statics = bodies::static_link_transforms(def)`。
    pub fn set_defs(&mut self, defs: impl IntoIterator<Item = (BodyHash, BodyDef)>) {
        self.defs.clear();
        self.statics.clear();
        for (hash, def) in defs {
            let static_xf = crate::bodies::static_link_transforms(&def);
            self.statics.insert(hash, static_xf);
            self.defs.insert(hash, def);
        }
    }

    /// 消费单条 S2C 消息更新镜像(协议注释语义,其余消息 no-op)。
    ///
    /// - `AgentStates`:`full=true` 时以 `states` 重建 `agents` 并清辅助表;增量时 upsert 并按 `removed` 清四表。
    /// - `AgentMeta`:upsert `meta` 映射表,不动 `agents`。
    /// - `AgentStatesCompact`:分档 `full` 只清本 `tier`;缺 meta 跳过;解码完整连杆;按 `removed` 清四表。
    /// - `AgentStatesLite`:分档 `full` 只清档 2;缺 meta 跳过;按 `statics` 静姿展开(未知身体仅根);按 `removed` 清四表。
    /// - `EntityStates`/`EntityShapes`:刚体与复合形状镜像语义不变。
    pub fn apply(&mut self, s2c: &S2C) {
        match s2c {
            S2C::AgentStates {
                states,
                removed,
                full,
                ..
            } => {
                if *full {
                    self.agents.clear();
                    self.agent_tiers.clear();
                    self.agent_speed.clear();
                    self.meta.clear();
                    for (id, hash, poses) in states {
                        self.agents.insert(
                            *id,
                            AgentView {
                                hash: *hash,
                                poses: poses.clone(),
                            },
                        );
                    }
                } else {
                    for (id, hash, poses) in states {
                        self.agents.insert(
                            *id,
                            AgentView {
                                hash: *hash,
                                poses: poses.clone(),
                            },
                        );
                    }
                    for id in removed {
                        self.agents.remove(id);
                        self.agent_tiers.remove(id);
                        self.agent_speed.remove(id);
                        self.meta.remove(id);
                    }
                }
            }
            S2C::EntityStates {
                states,
                removed,
                full,
                ..
            } => {
                if *full {
                    self.solids.clear();
                    for (id, state) in states {
                        let compound = self.shapes.get(id).cloned();
                        self.solids.insert(
                            *id,
                            SolidView {
                                state: *state,
                                compound,
                            },
                        );
                    }
                } else {
                    for (id, state) in states {
                        let compound = self.shapes.get(id).cloned();
                        self.solids.insert(
                            *id,
                            SolidView {
                                state: *state,
                                compound,
                            },
                        );
                    }
                    for id in removed {
                        self.solids.remove(id);
                        self.shapes.remove(id);
                    }
                }
            }
            S2C::EntityShapes { shapes } => {
                for (id, shape) in shapes {
                    self.shapes.insert(*id, shape.clone());
                    if let Some(solid) = self.solids.get_mut(id) {
                        solid.compound = Some(shape.clone());
                    }
                }
            }
            S2C::AgentMeta { entries } => {
                for (id, hash) in entries {
                    self.meta.insert(*id, *hash);
                }
            }
            S2C::AgentStatesCompact {
                tier,
                states,
                removed,
                full,
                ..
            } => {
                if *full {
                    let present: BTreeSet<EntityId> = states.iter().map(|s| s.entity).collect();
                    let stale: Vec<EntityId> = self
                        .agent_tiers
                        .iter()
                        .filter_map(|(&id, &t)| {
                            (t == *tier && !present.contains(&id)).then_some(id)
                        })
                        .collect();
                    for id in stale {
                        self.agents.remove(&id);
                        self.agent_tiers.remove(&id);
                        self.agent_speed.remove(&id);
                    }
                }
                for s in states {
                    let Some(&hash) = self.meta.get(&s.entity) else {
                        continue;
                    };
                    let root_pos = Vec3::from_array(s.root_pos);
                    let root_rot = quant::unpack_quat(&s.root_rot);
                    let root_linvel = quant::unpack_linvel(&s.root_linvel);
                    let mut poses = Vec::with_capacity(1 + s.child_links.len());
                    poses.push(EntityState {
                        translation: root_pos,
                        rotation: root_rot,
                        linvel: root_linvel,
                        angvel: Vec3::ZERO,
                    });
                    for child in &s.child_links {
                        let (rel_pos, rel_rot) = quant::unpack_child_link(child);
                        poses.push(EntityState {
                            translation: root_pos + root_rot * rel_pos,
                            rotation: root_rot * rel_rot,
                            linvel: Vec3::ZERO,
                            angvel: Vec3::ZERO,
                        });
                    }
                    self.agents.insert(s.entity, AgentView { hash, poses });
                    self.agent_tiers.insert(s.entity, *tier);
                    self.agent_speed.remove(&s.entity);
                }
                for id in removed {
                    self.agents.remove(id);
                    self.agent_tiers.remove(id);
                    self.agent_speed.remove(id);
                    self.meta.remove(id);
                }
            }
            S2C::AgentStatesLite {
                origin_chunk,
                states,
                removed,
                full,
                ..
            } => {
                if *full {
                    let present: BTreeSet<EntityId> = states.iter().map(|s| s.entity).collect();
                    let stale: Vec<EntityId> = self
                        .agent_tiers
                        .iter()
                        .filter_map(|(&id, &t)| (t == 2 && !present.contains(&id)).then_some(id))
                        .collect();
                    for id in stale {
                        self.agents.remove(&id);
                        self.agent_tiers.remove(&id);
                        self.agent_speed.remove(&id);
                    }
                }
                let (cx, cz) = quant::unpack_origin_chunk(origin_chunk);
                let base = Vec3::new(cx as f32 * 16.0, 0.0, cz as f32 * 16.0);
                for s in states {
                    let Some(&hash) = self.meta.get(&s.entity) else {
                        continue;
                    };
                    let t = base + quant::unpack_lite_pos(&s.rel_pos);
                    let yaw = quant::unpack_yaw(&s.yaw);
                    let rot = Quat::from_rotation_y(yaw);
                    let (speed_q, frozen) = quant::unpack_motion_flags(s.motion_flags);
                    let speed = if frozen { 0.0 } else { speed_q };
                    let linvel = if frozen {
                        Vec3::ZERO
                    } else {
                        rot * Vec3::X * speed
                    };
                    let root = EntityState {
                        translation: t,
                        rotation: rot,
                        linvel,
                        angvel: Vec3::ZERO,
                    };
                    let poses = match self.statics.get(&hash) {
                        Some(rel_links) if !rel_links.is_empty() => {
                            let mut v = Vec::with_capacity(rel_links.len());
                            v.push(root);
                            for &(rel_pos, rel_rot) in rel_links.iter().skip(1) {
                                v.push(EntityState {
                                    translation: t + rot * rel_pos,
                                    rotation: rot * rel_rot,
                                    linvel: Vec3::ZERO,
                                    angvel: Vec3::ZERO,
                                });
                            }
                            v
                        }
                        _ => vec![root],
                    };
                    self.agents.insert(s.entity, AgentView { hash, poses });
                    self.agent_tiers.insert(s.entity, 2);
                    self.agent_speed.insert(s.entity, speed);
                }
                for id in removed {
                    self.agents.remove(id);
                    self.agent_tiers.remove(id);
                    self.agent_speed.remove(id);
                    self.meta.remove(id);
                }
            }
            _ => {}
        }
    }
}

/// 相机根位置表对账(Compact/Lite 共用):`full` 时 retain 到 `mirror.agents` 键集(跨档保留);
/// `ids` 中镜像存在者取 `poses[0].translation` upsert;`removed` 逐条删除。
pub fn sync_camera_roots(
    roots: &mut BTreeMap<EntityId, Vec3>,
    mirror: &EntityMirror,
    ids: &[EntityId],
    removed: &[EntityId],
    full: bool,
) {
    if full {
        roots.retain(|id, _| mirror.agents.contains_key(id));
    }
    for id in ids {
        if let Some(root) = mirror.agents.get(id).and_then(|v| v.poses.first()) {
            roots.insert(*id, root.translation);
        }
    }
    for id in removed {
        roots.remove(id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::glam::{Quat, Vec3};
    use types::{BlockId, ShapeCell, Tick};

    fn eid(n: u32) -> EntityId {
        EntityId::from_parts(0, n)
    }

    fn pose_at(x: f32, y: f32, z: f32) -> EntityState {
        EntityState {
            translation: Vec3::new(x, y, z),
            rotation: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        }
    }

    fn three_cell_column() -> CompoundShape {
        CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [0, 1, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [0, 2, 0],
                block: BlockId(1),
            },
        ])
        .expect("三格复合形合法")
    }

    /// MS27 t1(REQ-013 C03-1 D2 / PM33 靶点):full 重建丢弃旧 id
    /// (先增量塞 id=1,再 full 只含 id=2 → agents/solids 仅剩 id=2)。
    #[test]
    fn ms27_t1_full_rebuilds_and_drops_stale_ids() {
        let mut mirror = EntityMirror::new();
        let hash = BodyHash::fnv1a(b"capsule");

        mirror.apply(&S2C::AgentStates {
            tick: Tick(1),
            states: vec![(eid(1), hash, vec![pose_at(1.0, 2.0, 3.0)])],
            removed: vec![],
            full: false,
        });
        mirror.apply(&S2C::EntityStates {
            tick: Tick(1),
            states: vec![(eid(1), pose_at(4.0, 5.0, 6.0))],
            removed: vec![],
            full: false,
        });
        assert!(mirror.agents.contains_key(&eid(1)));
        assert!(mirror.solids.contains_key(&eid(1)));

        mirror.apply(&S2C::AgentStates {
            tick: Tick(2),
            states: vec![(eid(2), hash, vec![pose_at(7.0, 8.0, 9.0)])],
            removed: vec![],
            full: true,
        });
        mirror.apply(&S2C::EntityStates {
            tick: Tick(2),
            states: vec![(eid(2), pose_at(10.0, 11.0, 12.0))],
            removed: vec![],
            full: true,
        });

        assert_eq!(
            mirror.agents.keys().copied().collect::<Vec<_>>(),
            vec![eid(2)]
        );
        assert_eq!(
            mirror.solids.keys().copied().collect::<Vec<_>>(),
            vec![eid(2)]
        );
    }

    /// MS27 t2(REQ-013 C03-1 D2):增量 upsert 保留无关 id 并更新目标 id。
    #[test]
    fn ms27_t2_incremental_upsert_preserves_other_ids() {
        let mut mirror = EntityMirror::new();
        let hash = BodyHash::fnv1a(b"capsule");

        mirror.apply(&S2C::AgentStates {
            tick: Tick(1),
            states: vec![(eid(1), hash, vec![pose_at(1.0, 0.0, 0.0)])],
            removed: vec![],
            full: false,
        });
        mirror.apply(&S2C::AgentStates {
            tick: Tick(2),
            states: vec![(eid(2), hash, vec![pose_at(2.0, 0.0, 0.0)])],
            removed: vec![],
            full: false,
        });
        assert_eq!(
            mirror.agents.keys().copied().collect::<Vec<_>>(),
            vec![eid(1), eid(2)]
        );

        mirror.apply(&S2C::EntityStates {
            tick: Tick(1),
            states: vec![(eid(10), pose_at(1.0, 0.0, 0.0))],
            removed: vec![],
            full: false,
        });
        mirror.apply(&S2C::EntityStates {
            tick: Tick(2),
            states: vec![(eid(11), pose_at(2.0, 0.0, 0.0))],
            removed: vec![],
            full: false,
        });
        assert_eq!(
            mirror.solids.keys().copied().collect::<Vec<_>>(),
            vec![eid(10), eid(11)]
        );
        assert_eq!(mirror.solids[&eid(10)].compound, None);
    }

    /// MS27 t3(REQ-013 C03-1 D2 / PM32 & N16 靶点):removed 同步清 agent/solid/shape 三表。
    #[test]
    fn ms27_t3_removed_clears_agent_solid_and_shape() {
        let mut mirror = EntityMirror::new();
        let hash = BodyHash::fnv1a(b"capsule");
        let shape = three_cell_column();

        mirror.apply(&S2C::AgentStates {
            tick: Tick(1),
            states: vec![(eid(1), hash, vec![pose_at(0.0, 0.0, 0.0)])],
            removed: vec![],
            full: false,
        });
        mirror.apply(&S2C::EntityShapes {
            shapes: vec![(eid(2), shape)],
        });
        mirror.apply(&S2C::EntityStates {
            tick: Tick(1),
            states: vec![(eid(2), pose_at(0.0, 0.0, 0.0))],
            removed: vec![],
            full: false,
        });
        assert_eq!(mirror.agents.len(), 1);
        assert_eq!(mirror.solids.len(), 1);
        assert_eq!(mirror.shapes.len(), 1);

        mirror.apply(&S2C::AgentStates {
            tick: Tick(2),
            states: vec![],
            removed: vec![eid(1)],
            full: false,
        });
        mirror.apply(&S2C::EntityStates {
            tick: Tick(2),
            states: vec![],
            removed: vec![eid(2)],
            full: false,
        });
        assert!(mirror.agents.is_empty(), "removed 必须清 agents");
        assert!(mirror.solids.is_empty(), "removed 必须清 solids");
        assert!(mirror.shapes.is_empty(), "removed 必须同步清 shapes");
    }

    /// MS27 t4(REQ-013 C03-1 D2):EntityShapes 先到、EntityStates 后到 →
    /// solids[id].compound == 发送的 cells。
    #[test]
    fn ms27_t4_shapes_before_states_attaches_compound() {
        let mut mirror = EntityMirror::new();
        let shape = three_cell_column();

        mirror.apply(&S2C::EntityShapes {
            shapes: vec![(eid(7), shape.clone())],
        });
        mirror.apply(&S2C::EntityStates {
            tick: Tick(5),
            states: vec![(eid(7), pose_at(8.5, -58.5, 8.5))],
            removed: vec![],
            full: false,
        });

        let solid = mirror.solids.get(&eid(7)).expect("solid 应存在");
        assert_eq!(
            solid.compound.as_ref().map(CompoundShape::cells),
            Some(shape.cells()),
            "先到达的 EntityShapes 必须挂载到后续到达的 EntityStates 对应 solid"
        );
    }

    fn two_link_def() -> BodyDef {
        let toml = r#"
name = "two_link_lod"
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
        BodyDef::from_toml_str(toml).expect("two_link_lod 合法")
    }

    fn assert_vec3_near(actual: Vec3, expected: Vec3, tol: f32, label: &str) {
        let d = (actual - expected).abs();
        assert!(
            d.x <= tol && d.y <= tol && d.z <= tol,
            "{label}: actual={actual:?}, expected={expected:?}, diff={d:?}, tol={tol}"
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

    /// REQ-014 M04b-2 T1:Compact 量化帧解码到世界系完整 N 连杆姿态。
    #[test]
    fn ms31_t1_compact_decode_world() {
        use sim_api::proto::CompactAgentPose;

        let def = two_link_def();
        let hash = def.hash();
        let mut mirror = EntityMirror::new();
        mirror.apply(&S2C::AgentMeta {
            entries: vec![(eid(1), hash)],
        });
        mirror.apply(&S2C::AgentStatesCompact {
            tick: Tick(10),
            tier: 1,
            states: vec![CompactAgentPose {
                entity: eid(1),
                root_pos: [10.0, 20.0, 30.0],
                root_rot: [0x01, 0x00, 0x01, 0x80, 0xFF, 0x7F],
                root_linvel: [0x70, 0x17, 0x00, 0x00, 0x00, 0x00],
                child_links: vec![[
                    0xD0, 0x87, 0x00, 0x80, 0x00, 0x80, 0x03, 0x00, 0x01, 0x80, 0x00, 0x40,
                ]],
            }],
            removed: vec![],
            full: false,
        });

        let view = mirror.agents.get(&eid(1)).expect("id1 应已解码");
        let q90 = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        assert_eq!(view.hash, hash);
        assert_eq!(view.poses.len(), 2);
        assert_vec3_near(
            view.poses[0].translation,
            Vec3::new(10.0, 20.0, 30.0),
            1e-4,
            "root pos",
        );
        assert_vec3_near(
            view.poses[0].linvel,
            Vec3::new(6.0, 0.0, 0.0),
            1e-4,
            "root linvel",
        );
        assert_quat_near(view.poses[0].rotation, q90, 1e-4, "root rot");
        assert_vec3_near(
            view.poses[1].translation,
            Vec3::new(10.0, 20.0, 29.0),
            1e-4,
            "child world pos",
        );
        assert_quat_near(view.poses[1].rotation, q90, 1e-4, "child world rot");
        assert_eq!(view.poses[1].linvel, Vec3::ZERO, "子连杆 linvel 逐位零");
        assert_eq!(view.poses[1].angvel, Vec3::ZERO, "子连杆 angvel 逐位零");
        assert_eq!(mirror.agent_tiers.get(&eid(1)), Some(&1));
    }

    /// REQ-014 M04b-2 T2:Lite 解码 + 静姿 FK 展开 + frozen 速率归零 + 未知哈希降级仅根。
    #[test]
    fn ms31_t2_lite_decode_expand() {
        use sim_api::proto::LiteAgentPose;

        let def = two_link_def();
        let hash = def.hash();
        let mut mirror = EntityMirror::new();
        mirror.set_defs([(hash, def)]);
        let unknown_hash = BodyHash::fnv1a(b"nope");
        mirror.apply(&S2C::AgentMeta {
            entries: vec![(eid(1), hash), (eid(2), unknown_hash)],
        });

        let origin = [0xFD, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00];
        let rel = [0x30, 0x80, 0xB8, 0x7F, 0x30, 0x80];
        let yaw_neg90 = [0x00, 0xC0];

        mirror.apply(&S2C::AgentStatesLite {
            tick: Tick(20),
            origin_chunk: origin,
            states: vec![
                LiteAgentPose {
                    entity: eid(1),
                    rel_pos: rel,
                    yaw: yaw_neg90,
                    motion_flags: 0x14,
                },
                LiteAgentPose {
                    entity: eid(2),
                    rel_pos: rel,
                    yaw: yaw_neg90,
                    motion_flags: 0x14,
                },
            ],
            removed: vec![],
            full: true,
        });

        let v1 = mirror.agents.get(&eid(1)).expect("id1 应已解码");
        assert_eq!(v1.poses.len(), 2, "已知身体静姿展开为 N=2 连杆");
        assert_vec3_near(
            v1.poses[0].translation,
            Vec3::new(-46.5, -2.25, 113.5),
            1e-5,
            "lite root pos",
        );
        assert_vec3_near(
            v1.poses[1].translation,
            Vec3::new(-46.5, -2.25, 114.5),
            1e-5,
            "lite child pos",
        );
        assert_eq!(v1.poses[1].rotation, v1.poses[0].rotation);
        assert_vec3_near(
            v1.poses[0].rotation * Vec3::X,
            Vec3::new(0.0, 0.0, 1.0),
            1e-5,
            "lite yaw forward",
        );
        assert_vec3_near(
            v1.poses[0].linvel,
            Vec3::new(0.0, 0.0, 1.0),
            1e-5,
            "lite linvel",
        );
        assert_eq!(mirror.agent_tiers.get(&eid(1)), Some(&2));
        assert!((mirror.agent_speed[&eid(1)] - 1.0).abs() <= 1e-6);

        let v2 = mirror.agents.get(&eid(2)).expect("id2 应已解码");
        assert_eq!(v2.poses.len(), 1, "未知哈希仅根 1 pose");

        // frozen 变体: flags 0x94 -> linvel 逐位零, agent_speed 0.0, translation 不变。
        mirror.apply(&S2C::AgentStatesLite {
            tick: Tick(30),
            origin_chunk: origin,
            states: vec![LiteAgentPose {
                entity: eid(1),
                rel_pos: rel,
                yaw: yaw_neg90,
                motion_flags: 0x94,
            }],
            removed: vec![],
            full: true,
        });
        let v1_f = mirror.agents.get(&eid(1)).expect("id1 frozen 存在");
        assert_eq!(v1_f.poses[0].linvel, Vec3::ZERO, "frozen linvel 逐位零");
        assert_eq!(mirror.agent_speed[&eid(1)], 0.0, "frozen speed 为 0");
        assert_vec3_near(
            v1_f.poses[0].translation,
            Vec3::new(-46.5, -2.25, 113.5),
            1e-5,
            "frozen pos 不变",
        );
    }

    /// REQ-014 M04b-2 T3:分档 full 只清本档 + removed 同步清四表。
    #[test]
    fn ms31_t3_per_tier_full_and_removed() {
        use sim_api::proto::{CompactAgentPose, LiteAgentPose};

        let def = two_link_def();
        let hash = def.hash();
        let mut mirror = EntityMirror::new();
        mirror.set_defs([(hash, def)]);
        mirror.apply(&S2C::AgentMeta {
            entries: vec![
                (eid(1), hash),
                (eid(2), hash),
                (eid(3), hash),
                (eid(4), hash),
            ],
        });

        let id_quat = [0x03, 0x00, 0x01, 0x80, 0x00, 0x40];
        let zero6 = [0u8; 6];
        let zero_rel = [0x00, 0x80, 0x00, 0x80, 0x00, 0x80];

        // 三档初始: id3@tier0, id1@tier1, id2@tier2。
        mirror.apply(&S2C::AgentStatesCompact {
            tick: Tick(1),
            tier: 0,
            states: vec![CompactAgentPose {
                entity: eid(3),
                root_pos: [3.0, 0.0, 0.0],
                root_rot: id_quat,
                root_linvel: zero6,
                child_links: vec![],
            }],
            removed: vec![],
            full: false,
        });
        mirror.apply(&S2C::AgentStatesCompact {
            tick: Tick(1),
            tier: 1,
            states: vec![CompactAgentPose {
                entity: eid(1),
                root_pos: [1.0, 0.0, 0.0],
                root_rot: id_quat,
                root_linvel: zero6,
                child_links: vec![],
            }],
            removed: vec![],
            full: false,
        });
        mirror.apply(&S2C::AgentStatesLite {
            tick: Tick(1),
            origin_chunk: [0u8; 8],
            states: vec![LiteAgentPose {
                entity: eid(2),
                rel_pos: zero_rel,
                yaw: [0, 0],
                motion_flags: 0,
            }],
            removed: vec![],
            full: true,
        });

        // Compact tier=1 full=true states=[id4] -> id1 清, id2/id3 跨档保留。
        mirror.apply(&S2C::AgentStatesCompact {
            tick: Tick(2),
            tier: 1,
            states: vec![CompactAgentPose {
                entity: eid(4),
                root_pos: [4.0, 0.0, 0.0],
                root_rot: id_quat,
                root_linvel: zero6,
                child_links: vec![],
            }],
            removed: vec![],
            full: true,
        });
        assert_eq!(
            mirror.agents.keys().copied().collect::<Vec<_>>(),
            vec![eid(2), eid(3), eid(4)],
            "tier1 full 只清 tier1 的 id1,跨档保留 id2/id3"
        );
        assert!(!mirror.agent_tiers.contains_key(&eid(1)));

        // Lite full=true states=[] -> id2 清, 剩 {id3, id4}。
        mirror.apply(&S2C::AgentStatesLite {
            tick: Tick(3),
            origin_chunk: [0u8; 8],
            states: vec![],
            removed: vec![],
            full: true,
        });
        assert_eq!(
            mirror.agents.keys().copied().collect::<Vec<_>>(),
            vec![eid(3), eid(4)]
        );
        assert!(!mirror.agent_tiers.contains_key(&eid(2)));
        assert!(!mirror.agent_speed.contains_key(&eid(2)));

        // Compact tier=0 removed=[id3] -> 剩 {id4}, 四表(含 meta)同步清 id3。
        mirror.apply(&S2C::AgentStatesCompact {
            tick: Tick(4),
            tier: 0,
            states: vec![],
            removed: vec![eid(3)],
            full: false,
        });
        assert_eq!(
            mirror.agents.keys().copied().collect::<Vec<_>>(),
            vec![eid(4)]
        );
        assert!(!mirror.agent_tiers.contains_key(&eid(3)));
        assert!(!mirror.agent_speed.contains_key(&eid(3)));
        assert!(
            !mirror.meta.contains_key(&eid(3)),
            "removed 必须同步清 meta"
        );
    }

    /// REQ-014 M04b-2 T4:缺 meta 时 Compact 与 Lite 跳过,补发 AgentMeta 后恢复解码。
    #[test]
    fn ms31_t4_meta_missing_skip() {
        use sim_api::proto::{CompactAgentPose, LiteAgentPose};

        let hash = BodyHash::fnv1a(b"body9");
        let mut mirror = EntityMirror::new();
        let id_quat = [0x03, 0x00, 0x01, 0x80, 0x00, 0x40];

        mirror.apply(&S2C::AgentStatesCompact {
            tick: Tick(1),
            tier: 1,
            states: vec![CompactAgentPose {
                entity: eid(9),
                root_pos: [1.0, 2.0, 3.0],
                root_rot: id_quat,
                root_linvel: [0u8; 6],
                child_links: vec![],
            }],
            removed: vec![],
            full: false,
        });
        assert!(mirror.agents.is_empty());
        assert!(mirror.agent_tiers.is_empty());

        mirror.apply(&S2C::AgentStatesLite {
            tick: Tick(2),
            origin_chunk: [0u8; 8],
            states: vec![LiteAgentPose {
                entity: eid(9),
                rel_pos: [0x00, 0x80, 0x00, 0x80, 0x00, 0x80],
                yaw: [0, 0],
                motion_flags: 0x14,
            }],
            removed: vec![],
            full: true,
        });
        assert!(mirror.agents.is_empty());
        assert!(mirror.agent_tiers.is_empty());
        assert!(mirror.agent_speed.is_empty());

        mirror.apply(&S2C::AgentMeta {
            entries: vec![(eid(9), hash)],
        });
        mirror.apply(&S2C::AgentStatesCompact {
            tick: Tick(3),
            tier: 1,
            states: vec![CompactAgentPose {
                entity: eid(9),
                root_pos: [1.0, 2.0, 3.0],
                root_rot: id_quat,
                root_linvel: [0u8; 6],
                child_links: vec![],
            }],
            removed: vec![],
            full: false,
        });
        assert!(mirror.agents.contains_key(&eid(9)));
        assert_eq!(mirror.agents[&eid(9)].hash, hash);
    }

    /// REQ-014 M04b-2 T5:legacy `AgentStates` full 清辅助三表 + removed 清四表。
    #[test]
    fn ms31_t5_legacy_full_clears_aux() {
        use sim_api::proto::LiteAgentPose;

        let hash = BodyHash::fnv1a(b"legacy");
        let mut mirror = EntityMirror::new();
        mirror.apply(&S2C::AgentMeta {
            entries: vec![(eid(1), hash)],
        });
        mirror.apply(&S2C::AgentStatesLite {
            tick: Tick(1),
            origin_chunk: [0u8; 8],
            states: vec![LiteAgentPose {
                entity: eid(1),
                rel_pos: [0x00, 0x80, 0x00, 0x80, 0x00, 0x80],
                yaw: [0, 0],
                motion_flags: 0x14,
            }],
            removed: vec![],
            full: true,
        });
        assert!(!mirror.agent_tiers.is_empty());
        assert!(!mirror.agent_speed.is_empty());
        assert!(!mirror.meta.is_empty());

        mirror.apply(&S2C::AgentStates {
            tick: Tick(2),
            states: vec![(eid(5), hash, vec![pose_at(5.0, 0.0, 0.0)])],
            removed: vec![],
            full: true,
        });
        assert_eq!(
            mirror.agents.keys().copied().collect::<Vec<_>>(),
            vec![eid(5)]
        );
        assert!(mirror.agent_tiers.is_empty());
        assert!(mirror.agent_speed.is_empty());
        assert!(mirror.meta.is_empty());

        mirror.apply(&S2C::AgentStates {
            tick: Tick(3),
            states: vec![],
            removed: vec![eid(5)],
            full: false,
        });
        assert!(mirror.agents.is_empty());
        assert!(mirror.agent_tiers.is_empty());
        assert!(mirror.agent_speed.is_empty());
        assert!(mirror.meta.is_empty());
    }

    /// REQ-014 M04b-2 R1 T22:相机根位置表分档 full 跨档保留 + removed 删除。
    #[test]
    fn ms31_t6_camera_roots_per_tier_full() {
        use sim_api::proto::{CompactAgentPose, LiteAgentPose};

        let hash = BodyHash::fnv1a(b"cam_roots");
        let id_quat = [0x03, 0x00, 0x01, 0x80, 0x00, 0x40];
        let mut mirror = EntityMirror::new();
        mirror.apply(&S2C::AgentMeta {
            entries: vec![(eid(1), hash), (eid(2), hash)],
        });
        mirror.apply(&S2C::AgentStatesCompact {
            tick: Tick(1),
            tier: 1,
            states: vec![CompactAgentPose {
                entity: eid(1),
                root_pos: [1.0, 2.0, 3.0],
                root_rot: id_quat,
                root_linvel: [0u8; 6],
                child_links: vec![],
            }],
            removed: vec![],
            full: false,
        });
        mirror.apply(&S2C::AgentStatesLite {
            tick: Tick(1),
            origin_chunk: [0u8; 8],
            states: vec![LiteAgentPose {
                entity: eid(2),
                rel_pos: [0x80, 0x80, 0xA0, 0x80, 0xC0, 0x80],
                yaw: [0, 0],
                motion_flags: 0x80,
            }],
            removed: vec![],
            full: true,
        });

        let mut roots = BTreeMap::from([
            (eid(1), Vec3::ZERO),
            (eid(2), Vec3::new(4.0, 5.0, 6.0)),
            (eid(3), Vec3::new(9.0, 9.0, 9.0)),
        ]);
        sync_camera_roots(&mut roots, &mirror, &[eid(1)], &[], true);
        assert_eq!(
            roots,
            BTreeMap::from([
                (eid(1), Vec3::new(1.0, 2.0, 3.0)),
                (eid(2), Vec3::new(4.0, 5.0, 6.0)),
            ])
        );

        sync_camera_roots(&mut roots, &mirror, &[], &[eid(2)], false);
        assert_eq!(roots, BTreeMap::from([(eid(1), Vec3::new(1.0, 2.0, 3.0))]));
    }
}
