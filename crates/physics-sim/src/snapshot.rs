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
use std::collections::BTreeSet;

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
    /// ② 已建集合(修订 5 D18 独立状态;与 chunk_bodies 分离——全空气 chunk
    /// 无碰撞盒但 ② 已建)。
    built_colliders: BTreeSet<ChunkPos>,
    /// R1"装载时附带建 ②"暂存集合(N 拍决策、apply_chunk_load 兑现;未兑现
    /// 下次 apply_activation 作废)。随快照走:恢复后驱动继续送 ①,集合在场
    /// 即兑现——不进快照会有"有①无②"一拍窗口,违反 D18 逐拍相等。
    pending_collider_builds: BTreeSet<ChunkPos>,
    /// 待地形状态(REQ-007 任务 03/R10:保存的速度必须存,恢复后原样写回)。
    locomotion: BTreeMap<EntityId, crate::sim::LocomotionState>,
    /// 待执行强制落地(R10:到期/超预算簿记)。
    forced_pending: BTreeSet<EntityId>,
    /// 待执行上限兜底冻结(04a)。
    capped_freeze_pending: BTreeSet<EntityId>,
    outbox: Outbox,
    /// 激活簿记(两张冷却表;任务书 02 细则 4:随快照走,恢复后冷却窗口仍在)。
    activation: ActivationController,
    /// 激活内部拍(`apply_activation` 已执行次数;与冷却表同源,恢复后相对差不变)。
    activation_now: Tick,
    /// 观察者注册状态(命令唤醒预算强制的前提;`None` = 从未注册 → 行为与
    /// 历史一致;`Some` = 最近一次 `apply_activation` 的观察者表。随快照走:
    /// 恢复后激活生效状态与距离中心不变)。
    observed: Option<Vec<types::glam::Vec3>>,
    /// 唤醒中心(WakeSpec::Radius 票锚;04a;与 observed 分离)。随快照走。
    wake_centers: Option<Vec<types::glam::Vec3>>,
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
    collider: ColliderHandle,
    desc: EntityDesc,
    slow_ticks: u32,
    actor: bool,
    /// REQ-006:外部 spawn 来源(玩家) / 服务端 None。
    source: Option<messaging::InputSource>,
    window: std::collections::VecDeque<types::glam::Vec3>,
    /// 被承载/静止标志(K1:分片记录侧;随快照走)。
    carried: bool,
    at_rest: bool,
    /// 常驻运动票持票拍数记账(04a;含已触顶标记;随快照走)。
    actor_motion_ticks: Option<u64>,
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
                            collider: rec.collider,
                            desc: rec.desc,
                            slow_ticks: rec.slow_ticks,
                            actor: rec.actor,
                            source: rec.source,
                            window: rec.window.clone(),
                            carried: rec.carried,
                            at_rest: rec.at_rest,
                            actor_motion_ticks: rec.actor_motion_ticks,
                        },
                    )
                })
                .collect(),
            chunk_bodies: self.chunk_bodies.clone(),
            built_colliders: self.built_colliders.clone(),
            pending_collider_builds: self.pending_collider_builds.clone(),
            locomotion: self.locomotion.clone(),
            forced_pending: self.forced_pending.clone(),
            capped_freeze_pending: self.capped_freeze_pending.clone(),
            outbox: self.outbox.clone(),
            activation: self.activation.clone(),
            activation_now: self.activation_now,
            observed: self.observed.clone(),
            wake_centers: self.wake_centers.clone(),
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
                        collider: rec.collider,
                        desc: rec.desc,
                        slow_ticks: rec.slow_ticks,
                        actor: rec.actor,
                        source: rec.source,
                        window: rec.window,
                        carried: rec.carried,
                        at_rest: rec.at_rest,
                        actor_motion_ticks: rec.actor_motion_ticks,
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
            built_colliders: snap.built_colliders,
            pending_collider_builds: snap.pending_collider_builds,
            locomotion: snap.locomotion,
            forced_pending: snap.forced_pending,
            capped_freeze_pending: snap.capped_freeze_pending,
            outbox: snap.outbox,
            activation: snap.activation,
            activation_now: snap.activation_now,
            observed: snap.observed,
            wake_centers: snap.wake_centers,
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
        // §9-1(常驻半边)迁移:零重力(出生零速 = 立即静止;有重力虚空出生
        // 在下落,"冻结只在静止时"永不冻结)。
        let mut a = ShardSim::new(
            ShardSimConfig {
                gravity: Vec3::ZERO,
                ..cfg()
            },
            Box::new(SimpleRegistry::builtin()),
        );
        a.apply_prefix(&[
            StampedInput {
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
            },
            // §9-1 迁移:常驻化(瞬态不滞回冻结,任务 03)。
            StampedInput {
                apply_at: types::Tick::ZERO,
                source: InputSource(1),
                sequence: 2,
                payload: InputPayload::Command(SimCommand::MarkActor {
                    entity: e,
                    actor: true,
                }),
            },
        ]);
        // §9-1:零重力零速生成推进 ≥ 21 拍(慢速计数达标 = "先静止")。
        for _ in 0..21 {
            a.step();
        }
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

    /// 任务书测试 `snapshot_roundtrips_collider_set_and_cooldowns`(REQ-007
    /// 任务 01/细则 10):② 集合 + 两套冷却簿记随快照往返;恢复**不"凡 ①
    /// 皆建 ②"**;恢复后策略照常(状态 ≠ 策略冻结)。
    /// 判别器(② 簿记):远锚拍 0 拆 (0,0) ②、建 (1,0) ②(apply_activation
    /// 当场执行,簿记均 @0)→ 快照 → 恢复 → 回近锚:(0,0) 有①无② 且半径内
    /// 本应立即成为构建候选,被 ② 簿记 @0 挡下 → 全空;簿记丢失则决策立即
    /// 重现。满 180 拍后决策重现(冷却窗口随档)。
    #[test]
    fn snapshot_roundtrips_collider_set_and_cooldowns() {
        // ---- 场景 A:② 侧(② 集合 + ② 冷却簿记)----
        let cfg_a = ShardSimConfig {
            activation: crate::ActivationConfig {
                collider_unload_radius: 32.0,
                data_load_radius: 0.0, // 关装载候选 → pending 恒空,聚焦单独构建/拆除
                ..crate::ActivationConfig::default()
            },
            ..cfg()
        };
        let mut a = ShardSim::new(cfg_a, Box::new(SimpleRegistry::builtin()));
        let solid = {
            let mut ch = voxel::Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    ch.set(x, 64, z, types::BlockId(1));
                }
            }
            ch
        };
        a.load_chunk(types::ChunkPos::new(0, 0), solid.clone());
        a.load_data_only(types::ChunkPos::new(1, 0), solid.clone());
        // 拍 0 远锚 (48,·,8):(0,0) 中心距 40 > 32 且 ① 留 → 拆 ②;
        // (1,0) 中心距 24 ≤ 32 且 ① 在未建 → 建 ②。两者当场执行。
        let far = [Vec3::new(48.0, 0.0, 8.0)];
        let d = a.apply_activation(&far);
        assert_eq!(d.colliders_to_drop, vec![types::ChunkPos::new(0, 0)]);
        assert_eq!(d.colliders_to_build, vec![types::ChunkPos::new(1, 0)]);
        assert!(!a.has_collider(types::ChunkPos::new(0, 0)), "拆除已执行");
        assert!(a.has_collider(types::ChunkPos::new(1, 0)), "构建已执行");

        let bytes = a.snapshot();
        let mut b = ShardSim::restore(&bytes, Box::new(SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        // ② 集合逐位往返;恢复不"凡 ① 皆建 ②"((0,0) 有①无② 原样)。
        assert_eq!(b.collider_loaded_chunks(), a.collider_loaded_chunks());
        assert_eq!(
            b.collider_loaded_chunks(),
            std::iter::once(types::ChunkPos::new(1, 0)).collect(),
            "② 集合 = {{(1,0)}};(0,0) 不被恢复补建"
        );

        // 回近锚 (8,·,8)(激活拍 1):(0,0) ① 在无 ② 距 0 ≤ 32 → 构建候选,
        // 被 ② 簿记 @0(拆除记的)挡下 → 全空;簿记丢失 → 立即重现构建决策。
        let near = [Vec3::new(8.0, 0.0, 8.0)];
        let d = b.apply_activation(&near);
        assert!(
            d.colliders_to_build.is_empty() && d.colliders_to_drop.is_empty(),
            "② 簿记随快照往返:回锚重刷全空(实得 build={:?} drop={:?})",
            d.colliders_to_build,
            d.colliders_to_drop
        );
        // 满 180 拍(激活拍 180,now-last ≥ cooldown)→ 构建决策重现。
        let mut last = crate::ActivationDecisions::default();
        for _ in 0..179 {
            last = b.apply_activation(&near);
        }
        assert_eq!(
            last.colliders_to_build,
            vec![types::ChunkPos::new(0, 0)],
            "冷却满 → 构建决策重现(策略未随快照冻结)"
        );

        // ---- 场景 A′:R1 pending 集合随档(恢复后驱动送 ① 当场兑现)----
        let cfg_p = ShardSimConfig {
            activation: crate::ActivationConfig {
                data_load_radius: 0.0, // 只有锚点所在 chunk 是装载候选
                ..crate::ActivationConfig::default()
            },
            ..cfg()
        };
        let mut c = ShardSim::new(cfg_p, Box::new(SimpleRegistry::builtin()));
        let dp = c.apply_activation(&[Vec3::new(8.0, 0.0, 8.0)]);
        assert_eq!(dp.chunks_to_load, vec![types::ChunkPos::new(0, 0)]);
        assert_eq!(
            dp.colliders_pending_build,
            vec![types::ChunkPos::new(0, 0)],
            "N 拍 pending 决策"
        );
        let bytes_p = c.snapshot();
        let mut r = ShardSim::restore(&bytes_p, Box::new(SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        // 恢复实例上驱动送 ① → pending 兑现(同点建 ②)。若 pending 不入快照,
        // 这里 has_collider 为 false = "有①无②"一拍窗口,违反 D18。
        r.load_data_only(types::ChunkPos::new(0, 0), solid.clone());
        assert!(
            r.has_collider(types::ChunkPos::new(0, 0)),
            "pending 随快照:恢复后 ① 到场当场兑现 ②(D18 逐拍相等)"
        );

        // ---- 场景 B:① 侧簿记(同法)----
        let mut c = ShardSim::new(cfg(), Box::new(SimpleRegistry::builtin()));
        c.load_data_only(types::ChunkPos::new(1, 0), solid);
        let distant = [Vec3::new(200.0, 0.0, 8.0)]; // (1,0) 距 176 > 48 → ① 卸
        let d = c.apply_activation(&distant);
        assert_eq!(d.chunks_to_unload, vec![types::ChunkPos::new(1, 0)]);
        let bytes = c.snapshot();
        let d2 = ShardSim::restore(&bytes, Box::new(SimpleRegistry::builtin()))
            .expect("自有快照必可解码")
            .apply_activation(&distant);
        assert!(
            d2.chunks_to_unload.is_empty() && d2.chunks_to_load.is_empty(),
            "① 簿记随快照往返:立即重刷全空"
        );
    }
}
