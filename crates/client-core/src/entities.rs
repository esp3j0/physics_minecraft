//! entities —— 客户端主体与动态刚体/脱落簇纯逻辑镜像(REQ-013 C03-1 D2)。
//!
//! 零 bevy、零墙钟,无头可测:消费 `S2C::AgentStates`/`S2C::EntityStates`/`S2C::EntityShapes`,
//! 维护 `agents`/`solids`/`shapes` 三张权威镜像表供渲染层每帧对账。

use std::collections::BTreeMap;

use sim_api::proto::S2C;
use types::{BodyHash, CompoundShape, EntityId, EntityState};

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

/// 主体/刚体/复合形状镜像(全量重建、增量 upsert、removed 同步清三表)。
#[derive(Debug, Default, Clone)]
pub struct EntityMirror {
    pub agents: BTreeMap<EntityId, AgentView>,
    pub solids: BTreeMap<EntityId, SolidView>,
    pub shapes: BTreeMap<EntityId, CompoundShape>,
}

impl EntityMirror {
    pub fn new() -> Self {
        Self::default()
    }

    /// 消费单条 S2C 消息更新镜像(协议注释语义,其余消息 no-op)。
    ///
    /// - `AgentStates`:`full=true` 时以 `states` 重建 `agents`(忽略 `removed`,PM33 靶点);
    ///   增量时 upsert 并按 `removed` 删除(PM32 靶点)。
    /// - `EntityStates`:`full=true` 时以 `states` 重建 `solids`(`compound` 查 `shapes`,缺为 `None`);
    ///   增量时 upsert 并按 `removed` 同步清除 `solids` 与 `shapes`(PM32/N16 靶点)。
    /// - `EntityShapes`:upsert `shapes`(先于状态包到达;若 `solids` 已存在同 id 亦同步挂载)。
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
            _ => {}
        }
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
}
