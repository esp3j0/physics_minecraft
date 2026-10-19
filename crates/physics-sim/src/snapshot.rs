//! `ShardSim` 全状态快照/恢复(任务书 01,AC8 地基)。
//!
//! 快照 = 私有结构(`ShardSimSnapshot`)的 bincode 2 `config::standard()` 字节
//! (与 storage 同款):`cfg` / `voxels` / `phys: &PhysicsWorld` / `entities` /
//! `chunk_bodies` / `outbox`,字段序固定 → 确定性字节。**registry 不入快照**,
//! `restore` 重注入(README 决策 2:存档不绑注册表实例)。
//!
//! rapier 0.36 的 `serde-serialize` feature 使 `PhysicsWorld` 整体可序列化
//! (pipeline/ccd_solver 等纯工作区 `serde(skip)`,见官方
//! `pipeline/physics_world.rs`):暖启动/岛屿/接触状态全部随快照走——
//! 这是"位级"的关键,不得用"重建刚体"式近似替代。

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use voxel::BlockRegistry;

use crate::activation::ActivationController;
use crate::sim::{Outbox, ShardSim, ShardSimConfig};
use rapier3d::prelude::*;
use types::{ChunkPos, EntityDesc, EntityId, Tick};

/// 快照层可预期失败:残缺/错误字节(一致性违规仍走 panic,蓝图 §4)。
#[derive(Debug, thiserror::Error)]
pub enum SimSnapshotError {
    #[error("快照解码失败:{0}")]
    Decode(String),
}

/// 快照私有结构(接口细则 1):字段顺序即 bincode 序列化顺序,不得调整。
///
/// `phys` 直接序列化官方 `PhysicsWorld`(serde-serialize feature 派生):字节
/// 布局即官方 serde 布局,rapier 升级若新增含状态字段会自动随之入快照——
/// 不得用手工镜像字段清单(升级后静默漏存新状态,AC8 地基失效)。
#[derive(Serialize, Deserialize)]
struct ShardSimSnapshot {
    cfg: ShardSimConfig,
    voxels: voxel::VoxelWorld,
    phys: PhysicsWorld,
    entities: BTreeMap<EntityId, SnapshotEntityRecord>,
    chunk_bodies: BTreeMap<ChunkPos, RigidBodyHandle>,
    outbox: Outbox,
    /// 激活簿记(两张冷却表;任务书 02 细则 4:随快照走,恢复后冷却窗口仍在)。
    activation: ActivationController,
    /// 激活内部拍(`apply_activation` 已执行次数;与冷却表同源,恢复后相对差不变)。
    activation_now: Tick,
    /// 观察者注册状态(命令唤醒预算强制的前提;`None` = 从未注册 → 行为与
    /// 历史一致;`Some` = 最近一次 `apply_activation` 的观察者表。随快照走:
    /// 恢复后激活生效状态与距离中心不变)。
    observed: Option<Vec<types::glam::Vec3>>,
}

/// 快照用的世界克隆:状态字段逐域拷贝,三个 serde(skip) 工作区字段取新构造
/// (rapier 未提供整体 `Clone`;与官方 Default 构造同款字段)。
fn clone_world(w: &PhysicsWorld) -> PhysicsWorld {
    PhysicsWorld {
        physics_pipeline: PhysicsPipeline::new(),
        collision_pipeline: CollisionPipeline::new(),
        ccd_solver: CCDSolver::new(),
        gravity: w.gravity,
        integration_parameters: w.integration_parameters,
        islands: w.islands.clone(),
        broad_phase: w.broad_phase.clone(),
        narrow_phase: w.narrow_phase.clone(),
        bodies: w.bodies.clone(),
        colliders: w.colliders.clone(),
        impulse_joints: w.impulse_joints.clone(),
        multibody_joints: w.multibody_joints.clone(),
        soft_bodies: w.soft_bodies.clone(),
    }
}

/// `EntityRecord` 的可序列化镜像(`slow_ticks` 原样,细则 1;actor 与位移窗口
/// 随快照走,任务书 01 细则 4)。
#[derive(Serialize, Deserialize)]
struct SnapshotEntityRecord {
    handle: RigidBodyHandle,
    desc: EntityDesc,
    slow_ticks: u32,
    actor: bool,
    /// REQ-006:外部 spawn 来源(玩家) / 服务端 None。
    source: Option<messaging::InputSource>,
    window: std::collections::VecDeque<types::glam::Vec3>,
}

impl ShardSim {
    /// 全状态快照(确定性字节:bincode standard 配置,字段序固定)。
    /// registry 不入快照;restore 重注入(README 决策 2)。
    pub fn snapshot(&self) -> Vec<u8> {
        let snap = ShardSimSnapshot {
            cfg: self.cfg.clone(),
            voxels: self.voxels.clone(),
            phys: clone_world(&self.phys),
            entities: self
                .entities
                .iter()
                .map(|(id, rec)| {
                    (
                        *id,
                        SnapshotEntityRecord {
                            handle: rec.handle,
                            desc: rec.desc,
                            slow_ticks: rec.slow_ticks,
                            actor: rec.actor,
                            source: rec.source,
                            window: rec.window.clone(),
                        },
                    )
                })
                .collect(),
            chunk_bodies: self.chunk_bodies.clone(),
            outbox: self.outbox.clone(),
            activation: self.activation.clone(),
            activation_now: self.activation_now,
            observed: self.observed.clone(),
        };
        bincode::serde::encode_to_vec(&snap, bincode::config::standard())
            .expect("快照编码不可能失败(全字段可序列化,fail-stop)")
    }

    /// 从快照恢复。registry 必须与原实例语义一致(SimpleRegistry::builtin 稳定)。
    pub fn restore(
        bytes: &[u8],
        registry: Box<dyn BlockRegistry>,
    ) -> Result<Self, SimSnapshotError> {
        let (snap, _len): (ShardSimSnapshot, usize) =
            bincode::serde::decode_from_slice(bytes, bincode::config::standard())
                .map_err(|e| SimSnapshotError::Decode(e.to_string()))?;
        // 官方反序列化:serde(skip) 工作区字段经 Default 补齐,状态字段全量还原。
        let phys = snap.phys;
        let entities: BTreeMap<EntityId, crate::sim::EntityRecord> = snap
            .entities
            .into_iter()
            .map(|(id, rec)| {
                (
                    id,
                    crate::sim::EntityRecord {
                        handle: rec.handle,
                        desc: rec.desc,
                        slow_ticks: rec.slow_ticks,
                        actor: rec.actor,
                        source: rec.source,
                        window: rec.window,
                    },
                )
            })
            .collect();
        Ok(Self {
            cfg: snap.cfg,
            registry,
            voxels: snap.voxels,
            phys,
            entities,
            chunk_bodies: snap.chunk_bodies,
            outbox: snap.outbox,
            activation: snap.activation,
            activation_now: snap.activation_now,
            observed: snap.observed,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{ShardSimConfig, SolidifyConfig};
    use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
    use types::BlockPos;
    use types::glam::Vec3;
    use types::{DimensionId, EntityState, ShardPos, WorldId};
    use voxel::SimpleRegistry;

    /// 测试用分片身份(固定值,不引入随机)。
    fn shard_id() -> types::ShardId {
        types::ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0))
    }

    /// 凝固桥关闭的测试配置(与既有 physics-sim 测试同风格)。
    fn cfg() -> ShardSimConfig {
        let mut c = ShardSimConfig::new(shard_id());
        c.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        c
    }

    /// 一条盖章的 SpawnDynamic 输入。
    fn spawn(id: u32, x: f32, y: f32) -> StampedInput {
        StampedInput {
            apply_at: types::Tick::ZERO,
            source: InputSource(1),
            sequence: u64::from(id),
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: types::EntityId::from_parts(1, id),
                state: EntityState {
                    translation: Vec3::new(x, y, x),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            }),
        }
    }

    /// 建 3 个 chunk 的平板地形(层 0 全石)+ 2 个下落实体。
    fn sim_with_terrain_and_entities() -> ShardSim {
        let mut sim = ShardSim::new(cfg(), Box::new(SimpleRegistry::builtin()));
        let stone = types::BlockId(1);
        for (cx, cz) in [(0, 0), (1, 0), (0, 1)] {
            let mut chunk = voxel::Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    // 世界 y = 0 ↔ 局部 y = WORLD_Y_MIN 便宜量(64)。
                    chunk.set(x, 64, z, stone);
                }
            }
            sim.load_chunk(types::ChunkPos::new(cx, cz), chunk);
        }
        sim.apply_prefix(&[spawn(1, 8.0, 3.0), spawn(2, 24.0, 3.0)]);
        sim
    }

    /// 逐实体比较状态(位级 `==`:EntityState 的 f32 字段直接相等)。
    /// `ids` 为两实例都应有的实体表(测试内已知)。
    fn assert_states_bitwise(a: &ShardSim, b: &ShardSim, ids: &[EntityId]) {
        assert!(!ids.is_empty(), "至少要有一个实体可比");
        for id in ids {
            assert_eq!(
                a.entity_state(*id),
                b.entity_state(*id),
                "实体 {id:?} 状态位级不一致"
            );
        }
    }

    /// 测试用实体表(spawn(1)/(2) 所得)。
    fn entity_ids() -> Vec<EntityId> {
        vec![
            types::EntityId::from_parts(1, 1),
            types::EntityId::from_parts(1, 2),
        ]
    }

    /// 测试清单 1:地形+实体跑 10 拍 → 快照 → restore → 两实例再各跑 10 拍
    /// → 逐实体 EntityState 位级 `==`。
    #[test]
    fn snapshot_restore_then_step_is_bitwise() {
        let mut a = sim_with_terrain_and_entities();
        for _ in 0..10 {
            a.step();
        }
        let bytes = a.snapshot();
        let mut b = ShardSim::restore(&bytes, Box::new(SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        for _ in 0..10 {
            a.step();
            b.step();
        }
        assert_states_bitwise(&a, &b, &entity_ids());
    }

    /// 测试清单 2:`restore(s).snapshot() == s`(serde 字段序稳定,
    /// 防引入非确定性字段)。
    #[test]
    fn snapshot_bytes_are_idempotent() {
        let a = sim_with_terrain_and_entities();
        let s1 = a.snapshot();
        let b =
            ShardSim::restore(&s1, Box::new(SimpleRegistry::builtin())).expect("自有快照必可解码");
        let s2 = b.snapshot();
        assert_eq!(s1, s2, "restore→snapshot 必须得到相同字节");
    }

    /// 测试清单 3:`restore(&[0xFF; 8], …)` → `Err(Decode)`,不 panic。
    #[test]
    fn restore_garbage_errors() {
        let result = ShardSim::restore(&[0xFF; 8], Box::new(SimpleRegistry::builtin()));
        assert!(
            matches!(result, Err(SimSnapshotError::Decode(_))),
            "垃圾字节必须返回 Err(Decode)"
        );
    }

    /// 测试清单 4:restore 后 SetBlock/凝固行为正常(冒烟:再跑 5 拍不炸,
    /// 且 SetBlock 仍能经 registry 正常落块)。
    #[test]
    fn snapshot_excludes_registry_but_restores_behavior() {
        let mut a = sim_with_terrain_and_entities();
        for _ in 0..3 {
            a.step();
        }
        let bytes = a.snapshot();
        let mut b = ShardSim::restore(&bytes, Box::new(SimpleRegistry::builtin()))
            .expect("自有快照必可解码");

        // SetBlock 走 registry::is_solid → collider 重建路径:恢复实例照常工作。
        b.apply_prefix(&[StampedInput {
            apply_at: types::Tick::ZERO,
            source: InputSource(1),
            sequence: 9,
            payload: InputPayload::Command(SimCommand::SetBlock {
                pos: BlockPos::new(8, 1, 8),
                block: types::BlockId(1),
            }),
        }]);
        for _ in 0..5 {
            b.step();
        }
        assert_eq!(
            b.voxel().get_block(BlockPos::new(8, 1, 8)),
            types::BlockId(1),
            "restore 后 SetBlock 经重注入 registry 生效"
        );
    }

    /// 测试清单 5(任务书 02):冷却簿记随快照往返——装→快照→恢复→
    /// 冷却窗口仍在。场景:远处实体冻结(簿记 flip@0,激活拍 +1)→ 快照 →
    /// 恢复实例换近观察者刷新:唤醒候选但冷却未过 → 不醒;满 180 拍后
    /// 同输入 → 醒(证明簿记与激活拍计数均随快照走)。
    #[test]
    fn snapshot_roundtrip_keeps_activation_state() {
        let e = types::EntityId::from_parts(1, 1);
        let mut a = ShardSim::new(cfg(), Box::new(SimpleRegistry::builtin()));
        a.apply_prefix(&[StampedInput {
            apply_at: types::Tick::ZERO,
            source: InputSource(1),
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(100.0, 3.0, 100.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            }),
        }]);
        // 拍 0(远观察者):实体冻结,簿记 flip@0,激活拍 → 1。
        let d = a.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.entities_to_freeze, vec![e], "远处实体被冻结");

        let bytes = a.snapshot();
        let mut b = ShardSim::restore(&bytes, Box::new(SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        assert_eq!(
            b.activation_config(),
            a.activation_config(),
            "激活配置随快照走"
        );

        // 恢复实例,激活拍 1:换近观察者(距实体 ≈5.7 ≤ 24)→ 唤醒候选,
        // 但冷却未过(簿记 flip@0 随快照走)→ 不醒。
        // (tier①② 的 chunk 装载请求与实体冷却无关,空世界照常产生装载
        // 候选是策略的正确行为,不在此断言。)
        let d = b.apply_activation(&[Vec3::new(96.0, 0.0, 96.0)]);
        assert!(
            d.entities_to_wake.is_empty() && d.entities_to_freeze.is_empty(),
            "冷却簿记随快照保留:唤醒被挡下"
        );

        // 再刷 179 次(激活拍 2..=180):满 180 拍冷却已过 → 醒(§9.12 冻结
        // 须显式激活唤醒;此处由策略冷却窗证明簿记完整)。
        let mut last = crate::ActivationDecisions::default();
        for _ in 0..179 {
            last = b.apply_activation(&[Vec3::new(96.0, 0.0, 96.0)]);
        }
        assert_eq!(last.entities_to_wake, vec![e], "180 拍冷却满 → 唤醒");
        assert!(last.entities_to_freeze.is_empty());
    }
}
