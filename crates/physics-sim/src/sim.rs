//! `ShardSim`:单分片确定性物理核(rapier 封装,REQ-001 §4.1/§4.3)。
//!
//! 单个分片 = 一个 rapier `PhysicsWorld` + 自己的体素 chunk(`VoxelWorld`)+ 实体表。
//! 一切状态变更只经 `apply_prefix`(盖章输入);`step` 推进一拍后在 tick 末做
//! 凝固桥检测与迁移检测(demo A/C 协议);产出经 `take_outbox` 交给调度器。
//! rapier 类型不出本模块的公开接口(防腐层,蓝图 §0)。

use std::collections::{BTreeMap, BTreeSet};

use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{
    BlockPos, CHUNK_SIZE, ChunkPos, EntityDesc, EntityId, EntityState, SHARD_CHUNKS, ShardId, Tick,
};
use voxel::{BlockRegistry, Chunk, VoxelEdit, VoxelWorld};

use crate::activation::{ActivationConfig, ActivationDecisions};
use crate::collider_build::{BoxCollider, NeighborChunks, greedy_colliders};
use crate::conv;
use crate::solidify;
use messaging::{CrossEvent, EnvelopePayload, InputPayload, Intent, SimCommand, StampedInput};

/// 凝固桥触发参数(P2 + 睡眠兜底,REQ-001 §9.5 / demo C)。可序列化(快照)。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolidifyConfig {
    /// 总开关。几何测试等场景可关闭;生产恒开(共识 6)。
    pub enabled: bool,
    /// 线速度阈值(m/tick)。
    pub linear_threshold: f32,
    /// 角速度阈值(rad/tick)。
    pub angular_threshold: f32,
    /// 连续低于阈值的 tick 数。
    pub consecutive_ticks: u32,
    /// snap 位移超过此值(格)→ 升级为等睡眠。
    pub max_snap_blocks: f32,
    /// snap 角度误差超过此值 → 升级为等睡眠。
    pub max_snap_angle_deg: f32,
}

impl Default for SolidifyConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            linear_threshold: 0.1,
            angular_threshold: 0.05,
            consecutive_ticks: 20,
            max_snap_blocks: 0.5,
            max_snap_angle_deg: 45.0,
        }
    }
}

/// 单分片配置。可序列化(快照,任务书 01 细则 2:gravity 为 glam Vec3,
/// types 的 glam 已开 serde)。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShardSimConfig {
    pub shard: ShardId,
    pub gravity: Vec3,
    pub dt: f32,
    pub solidify: SolidifyConfig,
    /// 激活参数(§4.3;Default 默认,任务书 02)。
    pub activation: crate::activation::ActivationConfig,
}

impl ShardSimConfig {
    pub fn new(shard: ShardId) -> Self {
        Self {
            shard,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            dt: 1.0 / 60.0,
            solidify: SolidifyConfig::default(),
            activation: crate::activation::ActivationConfig::default(),
        }
    }
}

/// 本 tick 产出(调度器消费):迁移意图、collider 重建过的 chunk、凝固记录。
/// 可序列化 + 可克隆(快照携带未消费的产出,任务书 01 细则 1)。
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct Outbox {
    /// 迁移意图(调度器盖章成信封,REQ-001 §4.2 (b))。
    pub migrations: Vec<Intent>,
    /// 本 tick collider 重建过的 chunk(可见性/换装统计用)。
    pub dirty_chunks: BTreeSet<ChunkPos>,
    /// 凝固完成记录:(实体, 写入的格)。
    pub solidified: Vec<(EntityId, BlockPos)>,
}

/// 实体的分片侧记录(crate 内可见:snapshot.rs 的快照结构复用)。
pub(crate) struct EntityRecord {
    pub(crate) handle: RigidBodyHandle,
    pub(crate) desc: EntityDesc,
    /// 连续"低速"tick 计数(P2 触发用)。
    pub(crate) slow_ticks: u32,
}

/// 单分片模拟器(字段 crate 内可见:snapshot.rs 读写快照结构)。
pub struct ShardSim {
    pub(crate) cfg: ShardSimConfig,
    pub(crate) registry: Box<dyn BlockRegistry>,
    pub(crate) voxels: VoxelWorld,
    pub(crate) phys: PhysicsWorld,
    pub(crate) entities: BTreeMap<EntityId, EntityRecord>,
    pub(crate) chunk_bodies: BTreeMap<ChunkPos, RigidBodyHandle>,
    pub(crate) outbox: Outbox,
    /// 激活策略机(滞回/冷却/预算簿记;任务书 02)。
    pub(crate) activation: crate::activation::ActivationController,
    /// 激活内部拍:`apply_activation` 已执行次数(随快照走,恢复后冷却
    /// 相对差不变;`apply_activation` 签名无 tick 参数,由本计数供拍)。
    pub(crate) activation_now: Tick,
    /// 观察者注册状态(README 决策 6 的命令唤醒淘汰前提;随快照走,语义同
    /// 任务书 02 细则 5 的 Option 开关:`None` = 从未注册 → 激活完全不生效,
    /// 行为与历史一致;`Some` = 最近一次 `apply_activation` 的观察者表,
    /// 命令唤醒路径的预算强制以其为距离中心)。
    pub(crate) observed: Option<Vec<Vec3>>,
}

impl ShardSim {
    pub fn new(cfg: ShardSimConfig, registry: Box<dyn BlockRegistry>) -> Self {
        let mut phys = PhysicsWorld::new();
        phys.gravity = conv::vec3_to_rapier(cfg.gravity);
        phys.integration_parameters.dt = cfg.dt;
        let activation = crate::activation::ActivationController::new(cfg.activation);
        Self {
            cfg,
            registry,
            voxels: VoxelWorld::new(),
            phys,
            entities: BTreeMap::new(),
            chunk_bodies: BTreeMap::new(),
            outbox: Outbox::default(),
            activation,
            activation_now: Tick::ZERO,
            observed: None,
        }
    }

    pub fn shard(&self) -> ShardId {
        self.cfg.shard
    }

    /// 体素世界只读视图(测试/导出用)。
    pub fn voxel(&self) -> &VoxelWorld {
        &self.voxels
    }

    pub fn entity_count(&self) -> usize {
        self.entities.len()
    }

    /// 实体当前状态;不存在(已迁移/已凝固)返回 None。
    pub fn entity_state(&self, id: EntityId) -> Option<EntityState> {
        let rec = self.entities.get(&id)?;
        Some(self.read_state(rec))
    }

    /// 加载一个体素 chunk 并(与已加载邻居一起)重建边界 collider。
    pub fn load_chunk(&mut self, cp: ChunkPos, chunk: Chunk) {
        self.voxels.insert_chunk(cp, chunk);
        for cp in expand_boundary([cp], &self.voxels) {
            self.rebuild_chunk_collider(cp);
        }
    }

    /// 卸载一个 chunk(返回它供持久化屏障),重建邻居边界 collider。
    pub fn unload_chunk(&mut self, cp: ChunkPos) -> Option<Chunk> {
        let removed = self.voxels.remove_chunk(cp);
        if let Some(handle) = self.chunk_bodies.remove(&cp) {
            self.phys.remove_body(handle);
        }
        for ncp in expand_boundary([cp], &self.voxels) {
            self.rebuild_chunk_collider(ncp);
        }
        removed
    }

    /// 应用一个 tick 的输入前缀(命令/事件/信封,按 canonical 序)。
    ///
    /// README 决策 6(绑定):命令唤醒路径(Spawn/Teleport 重建即醒,§9.12)
    /// 超预算时**同一拍立即**按距离淘汰(`enforce_budget` 语义;hook 在 apply
    /// 末尾、step 之前,淘汰与 step 同拍生效)。淘汰 = 冻结落实(清速 +
    /// kinematic)+ 按决策即簿记记入冷却表;追加产出并入 outbox 无额外通道。
    /// 预算内 → 零额外动作(唤醒当拍行为与历史完全一致)。
    pub fn apply_prefix(&mut self, inputs: &[StampedInput]) {
        let mut sorted: Vec<&StampedInput> = inputs.iter().collect();
        sorted.sort_by_key(|i| i.canonical_key());
        for input in sorted {
            match input.payload {
                InputPayload::Command(cmd) => self.apply_command(cmd),
                InputPayload::Event(CrossEvent::Detonate {
                    center,
                    power,
                    radius,
                }) => self.detonate(center, power, radius),
                InputPayload::Envelope(e) => match e.payload {
                    EnvelopePayload::MigrateEntity {
                        entity,
                        state,
                        desc,
                    } => self.spawn_entity(entity, state, desc),
                },
            }
        }
        self.enforce_budget_after_command_wake();
    }

    /// 命令唤醒路径的预算强制(README 决策 6):观察者已注册过(激活生效中)
    /// 且醒数超限时,按到最近观察者距离降序给出应冻结者并**立即落实**。
    /// 淘汰条目照 plan 的超预算路径记入冷却簿记(唤醒须等冷却/显式命令)。
    /// 从未注册观察者 → 恒空,行为与历史一致(向后兼容开关,任务书 02 细则 5)。
    fn enforce_budget_after_command_wake(&mut self) {
        let Some(observers) = self.observed.clone() else {
            return;
        };
        let awake: Vec<(EntityId, Vec3)> = self
            .entities
            .iter()
            .map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                (*id, conv::rapier_to_vec3(b.translation()))
            })
            .collect();
        for id in self.activation.enforce_budget(&observers, &awake) {
            let now = self.activation_now;
            self.freeze_entity(&id);
            self.activation.note_flip(&id, now);
        }
    }

    /// 冻结落实(tier③;apply_activation 与命令唤醒淘汰共用):
    /// rapier 0.36 语义:KinematicPositionBased 下 set_linvel/set_angvel 是
    /// no-op → 必须先清速度再切类型。
    fn freeze_entity(&mut self, id: &EntityId) {
        let rec = self
            .entities
            .get(id)
            .expect("决策实体必在实体表(输入即来源)");
        let body = self.phys.bodies.get_mut(rec.handle).expect("handle 有效");
        body.set_linvel(conv::vec3_to_rapier(Vec3::ZERO), true);
        body.set_angvel(conv::vec3_to_rapier(Vec3::ZERO), true);
        body.set_body_type(RigidBodyType::KinematicPositionBased, true);
    }

    /// 推进一拍:物理步进 → 凝固桥检测 → 迁移检测(demo A/C 时序)。
    pub fn step(&mut self) {
        self.phys.step();

        let ids: Vec<EntityId> = self.entities.keys().copied().collect();
        let orients = solidify::axis_aligned_orientations();
        let cfg = self.cfg.solidify.clone();
        for id in ids {
            let Some(rec) = self.entities.get(&id) else {
                continue;
            };
            let (handle, desc) = (rec.handle, rec.desc);
            let state = self.read_state(rec);
            let is_sleeping = self
                .phys
                .bodies
                .get(handle)
                .expect("handle 有效")
                .is_sleeping();
            // 冻结体(kinematic)跳过凝固桥与迁移检测(§9.12/任务书 02):
            // 不凝固(冻结清零速度会使 slow 判据恒真)、不迁移(防御性;
            // kinematic 零速度本就静止,两跳为不变量的显式保证)。
            let is_frozen = self
                .phys
                .bodies
                .get(handle)
                .expect("handle 有效")
                .is_kinematic();
            if is_frozen {
                continue;
            }

            // ---- 凝固桥(P2 + 睡眠兜底,§9.5)----
            let slow = state.linvel.length() < cfg.linear_threshold
                && state.angvel.length() < cfg.angular_threshold;
            let slow_ticks = if slow { rec.slow_ticks + 1 } else { 0 };
            self.entities.get_mut(&id).expect("存在").slow_ticks = slow_ticks;
            if cfg.enabled && (slow_ticks >= cfg.consecutive_ticks || is_sleeping) {
                let snapped = solidify::nearest_orientation(state.rotation, &orients);
                let (cell, _center, disp) = solidify::snap_position(state.translation);
                let ang_err = solidify::snap_angle_error(state.rotation, snapped);
                let too_far =
                    disp > cfg.max_snap_blocks || ang_err > cfg.max_snap_angle_deg.to_radians();
                if !too_far || is_sleeping {
                    // 睡眠兜底:睡着即无条件凝固;未睡但误差过大 → 继续等(升级路径)
                    if self.voxels.set_block(cell, desc.block).is_ok() {
                        for cp in expand_boundary([cell.chunk_of()], &self.voxels) {
                            self.rebuild_chunk_collider(cp);
                        }
                        self.entities.remove(&id).expect("存在");
                        self.phys.remove_body(handle);
                        self.outbox.solidified.push((id, cell));
                        continue;
                    }
                }
            }

            // ---- 迁移检测(实体中心离开本体区,demo A)----
            if !self.core_contains(state.translation) {
                let dest = ShardId::new(
                    self.cfg.shard.world,
                    self.cfg.shard.dimension,
                    pos_chunk(state.translation).shard_of(),
                );
                self.entities.remove(&id).expect("存在");
                self.phys.remove_body(handle);
                self.outbox.migrations.push(Intent {
                    destination: dest,
                    payload: EnvelopePayload::MigrateEntity {
                        entity: id,
                        state,
                        desc,
                    },
                });
            }
        }
    }

    /// 取走并清空本 tick 产出。
    pub fn take_outbox(&mut self) -> Outbox {
        std::mem::take(&mut self.outbox)
    }

    /// 激活刷新(README 决策 5):tier③ 落实(冻结 = set_body_type
    /// KinematicPositionBased + 清速度;唤醒 = Dynamic + wake_up),
    /// tier①② 以决策返回(chunk 数据源在 REQ-002,本轮调用方可忽略)。
    /// 冻结体跳过:detonate(§9.12 事件免疫)、凝固桥、迁移检测(防御性)。
    ///
    /// 供拍的激活时钟 = 本方法已执行次数(随快照走,恢复后冷却相对差不变);
    /// 每 tick 恰好调用一次时即全局拍号。同输入两次调用第二次输出空
    /// (决策即簿记,见 `ActivationController::plan`)。首次调用即注册
    /// 观察者(`observed = Some`,覆盖式更新)→ 命令唤醒路径的预算强制
    /// 随之生效(README 决策 6)。
    pub fn apply_activation(&mut self, observers: &[Vec3]) -> ActivationDecisions {
        let now = self.activation_now;
        self.activation_now = now.next();
        self.observed = Some(observers.to_vec());
        let loaded: Vec<ChunkPos> = self.voxels.chunk_positions().collect();
        let entities: Vec<(EntityId, Vec3, bool)> = self
            .entities
            .iter()
            .map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                (*id, conv::rapier_to_vec3(b.translation()), b.is_kinematic())
            })
            .collect();
        let decisions = self.activation.plan(observers, &loaded, &entities, now);

        // tier③ 落实:施体类型切换。
        for id in &decisions.entities_to_freeze {
            self.freeze_entity(id);
        }
        for id in &decisions.entities_to_wake {
            let rec = self
                .entities
                .get(id)
                .expect("决策实体必在实体表(输入即来源)");
            let body = self.phys.bodies.get_mut(rec.handle).expect("handle 有效");
            body.set_body_type(RigidBodyType::Dynamic, true);
        }
        decisions
    }

    /// 激活配置(测试/注入用)。
    pub fn activation_config(&self) -> &ActivationConfig {
        &self.cfg.activation
    }

    // ---- 内部 ----

    fn apply_command(&mut self, cmd: SimCommand) {
        match cmd {
            SimCommand::SetBlock { pos, block } => {
                let edit = VoxelEdit::new().set(pos, block);
                match edit.apply(&mut self.voxels) {
                    Ok(dirty) => {
                        for cp in expand_boundary(dirty, &self.voxels) {
                            self.rebuild_chunk_collider(cp);
                        }
                    }
                    Err(e) => panic!("SetBlock 越界(fail-stop):{e}"),
                }
            }
            SimCommand::SpawnDynamic {
                entity,
                state,
                desc,
            } => self.spawn_entity(entity, state, desc),
            SimCommand::RemoveEntity { entity } => {
                self.despawn_entity(entity);
            }
            SimCommand::TeleportEntity { entity, state } => {
                // 一切位姿变更 = 删除 + 重建(REQ-001 §5 行 10)
                let (_, desc) = self.despawn_entity(entity);
                self.spawn_entity(entity, state, desc);
            }
            SimCommand::Detonate {
                center,
                power,
                radius,
            } => self.detonate(center, power, radius),
        }
    }

    fn rebuild_chunk_collider(&mut self, cp: ChunkPos) {
        if let Some(handle) = self.chunk_bodies.remove(&cp) {
            self.phys.remove_body(handle);
        }
        let Some(chunk) = self.voxels.get_chunk(cp) else {
            return;
        };
        let boxes: Vec<BoxCollider> =
            greedy_colliders(chunk, cp, &neighbors_of(&self.voxels, cp), |b| {
                self.registry.is_solid(b)
            });
        if boxes.is_empty() {
            return;
        }
        let parts: Vec<(Pose, SharedShape)> = boxes
            .iter()
            .map(|b| {
                (
                    conv::pose_to_rapier(b.center, types::glam::Quat::IDENTITY),
                    SharedShape::cuboid(b.half.x, b.half.y, b.half.z),
                )
            })
            .collect();
        let (handle, _) = self.phys.insert(
            RigidBodyBuilder::fixed(),
            ColliderBuilder::compound(parts)
                .friction(0.6)
                .restitution(0.0),
        );
        self.chunk_bodies.insert(cp, handle);
        self.outbox.dirty_chunks.insert(cp);
    }

    fn spawn_entity(&mut self, id: EntityId, state: EntityState, desc: EntityDesc) {
        assert!(
            !self.entities.contains_key(&id),
            "实体重复 spawn(fail-stop):{id:?}"
        );
        let h = desc.half_size;
        let (handle, _) = self.phys.insert(
            RigidBodyBuilder::dynamic()
                .pose(conv::pose_to_rapier(state.translation, state.rotation))
                .linvel(conv::vec3_to_rapier(state.linvel))
                .angvel(conv::vec3_to_rapier(state.angvel)),
            ColliderBuilder::cuboid(h, h, h)
                .density(1.0)
                .friction(0.6)
                .restitution(0.1),
        );
        self.entities.insert(
            id,
            EntityRecord {
                handle,
                desc,
                slow_ticks: 0,
            },
        );
    }

    /// 删除实体(迁移/传送/凝固),返回最后状态与描述。
    fn despawn_entity(&mut self, id: EntityId) -> (EntityState, EntityDesc) {
        let rec = self.entities.remove(&id).expect("实体存在(fail-stop)");
        let state = self.read_state(&rec);
        self.phys.remove_body(rec.handle);
        (state, rec.desc)
    }

    fn read_state(&self, rec: &EntityRecord) -> EntityState {
        let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
        EntityState {
            translation: conv::rapier_to_vec3(b.translation()),
            rotation: conv::rapier_to_quat(b.rotation()),
            linvel: conv::rapier_to_vec3(b.linvel()),
            angvel: conv::rapier_to_vec3(b.angvel()),
        }
    }

    fn detonate(&mut self, center: Vec3, power: f32, radius: f32) {
        let ids: Vec<EntityId> = self.entities.keys().copied().collect();
        for id in ids {
            let rec = self.entities.get(&id).expect("存在");
            let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
            // §9.12 事件免疫:冻结体不响应事件(爆炸对其无效)。
            if body.is_kinematic() {
                continue;
            }
            let pos = conv::rapier_to_vec3(body.translation());
            let Some(impulse) = crate::explosion::explosion_impulse(center, power, radius, pos)
            else {
                continue;
            };
            self.phys
                .bodies
                .get_mut(rec.handle)
                .expect("handle 有效")
                .apply_impulse(conv::vec3_to_rapier(impulse), true);
        }
    }

    /// 实体中心是否在本体区内(8×8 chunk,REQ-001 §4.1)。
    fn core_contains(&self, pos: Vec3) -> bool {
        let min = self.cfg.shard.pos.min_chunk().block_min();
        let max_x = min.x + SHARD_CHUNKS * CHUNK_SIZE;
        let max_z = min.z + SHARD_CHUNKS * CHUNK_SIZE;
        pos.x >= min.x as f32
            && pos.x < max_x as f32
            && pos.z >= min.z as f32
            && pos.z < max_z as f32
    }
}

/// 位置 → 所在 chunk(floor 语义,负方向向下取整)。
fn pos_chunk(pos: Vec3) -> ChunkPos {
    ChunkPos::new((pos.x / 16.0).floor() as i32, (pos.z / 16.0).floor() as i32)
}

/// 从 VoxelWorld 取四邻居(collider 重建用)。
fn neighbors_of(world: &VoxelWorld, cp: ChunkPos) -> NeighborChunks<'_> {
    NeighborChunks {
        px: world.get_chunk(ChunkPos::new(cp.x + 1, cp.z)),
        nx: world.get_chunk(ChunkPos::new(cp.x - 1, cp.z)),
        pz: world.get_chunk(ChunkPos::new(cp.x, cp.z + 1)),
        nz: world.get_chunk(ChunkPos::new(cp.x, cp.z - 1)),
    }
}

/// collider 需要重建的 chunk 集合:自身 + 已加载的边界邻居。
fn expand_boundary(
    dirty: impl IntoIterator<Item = ChunkPos>,
    world: &VoxelWorld,
) -> BTreeSet<ChunkPos> {
    let mut out = BTreeSet::new();
    for cp in dirty {
        out.insert(cp);
        for (dx, dz) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let ncp = ChunkPos::new(cp.x + dx, cp.z + dz);
            if world.get_chunk(ncp).is_some() {
                out.insert(ncp);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::{DimensionId, ShardPos, WorldId};

    fn shard_id(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    fn sim(x: i32, z: i32) -> ShardSim {
        ShardSim::new(
            ShardSimConfig::new(shard_id(x, z)),
            Box::new(voxel::SimpleRegistry::builtin()),
        )
    }

    fn entity(n: u32) -> EntityId {
        EntityId::from_parts(1, n)
    }

    fn spawn_input(seq: u64, id: EntityId, pos: Vec3) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: pos,
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            }),
        }
    }

    fn detonate_input(seq: u64, center: Vec3, power: f32, radius: f32) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::Detonate {
                center,
                power,
                radius,
            }),
        }
    }

    fn teleport_input(seq: u64, id: EntityId, pos: Vec3) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::TeleportEntity {
                entity: id,
                state: EntityState {
                    translation: pos,
                    ..EntityState::default()
                },
            }),
        }
    }

    /// 测试清单(sim)1:apply_activation 冻结远处实体后 Detonate:
    /// 冻结体状态位级不变(§9.12 事件免疫),近处实体受冲量。
    #[test]
    fn frozen_body_ignores_detonate() {
        let mut s = sim(0, 0);
        let (near, far) = (entity(1), entity(2));
        // 近 (8,3,8)、远 (100,3,100)——爆炸中心取两者正中,几何对称。
        s.apply_prefix(&[
            spawn_input(0, near, Vec3::new(8.0, 3.0, 8.0)),
            spawn_input(1, far, Vec3::new(100.0, 3.0, 100.0)),
        ]);
        let decisions = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert!(
            decisions.entities_to_freeze.contains(&far),
            "远处实体(距 ≈130 > 32)必须被冻结:{decisions:?}"
        );
        assert!(!decisions.entities_to_freeze.contains(&near));
        let far_before = s.entity_state(far).expect("冻结体仍在实体表");

        // 对称爆炸:两者到爆心等距(≈65 < 70)。
        s.apply_prefix(&[detonate_input(2, Vec3::new(54.0, 3.0, 54.0), 30.0, 70.0)]);

        let far_after = s.entity_state(far).expect("冻结体未被移除");
        assert_eq!(far_after, far_before, "冻结体对爆炸免疫:状态位级不变");
        let near_after = s.entity_state(near).expect("近处实体仍在");
        assert!(
            near_after.linvel.length() > 2.0 && near_after.linvel.x < 0.0,
            "近处实体受冲量且方向背离爆心:{:?}",
            near_after.linvel
        );
    }

    /// 测试清单(sim)2:冻结体不凝固、不产生迁移意图(200 拍冒烟)。
    /// 实体放本体区外 (130,·,100)(若不冻结第 1 拍即迁移),速度冻结后清零
    /// (若不跳过凝固,slow 判据恒真 → ≤20 拍即凝固移除);另设无激活对照
    /// 证明场景非退化。
    #[test]
    fn frozen_body_excluded_from_solidify_and_migration() {
        let e = entity(1);
        // 对照:不做激活 → 第 1 拍即迁出(场景确实会触发两条路径)。
        let mut control = sim(0, 0);
        control.apply_prefix(&[spawn_input(0, e, Vec3::new(130.0, 3.0, 100.0))]);
        control.step();
        assert!(
            control.entity_state(e).is_none(),
            "非退化前提:动态体在本体区外第 1 拍即迁移"
        );
        assert_eq!(control.take_outbox().migrations.len(), 1);

        // 冻结路径:apply_activation → 200 拍仍原位原态,outbox 全空。
        let mut s = sim(0, 0);
        s.apply_prefix(&[spawn_input(0, e, Vec3::new(130.0, 3.0, 100.0))]);
        let decisions = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(decisions.entities_to_freeze, vec![e], "远处实体被冻结");
        let frozen_state = s.entity_state(e).expect("冻结后仍在表");
        for _ in 0..200 {
            s.step();
        }
        assert_eq!(
            s.entity_state(e),
            Some(frozen_state),
            "冻结体 200 拍位级不变"
        );
        let outbox = s.take_outbox();
        assert!(outbox.migrations.is_empty(), "冻结体不迁移(防御性跳过)");
        assert!(outbox.solidified.is_empty(), "冻结体不凝固");
        assert_eq!(s.voxel().chunk_count(), 0, "凝固桥未写入任何体素");
    }

    /// activation_config 返回注入配置(非默认值可辨)。
    #[test]
    fn activation_config_reflects_injection() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.activation.max_awake_dynamic = 7;
        let s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        assert_eq!(s.activation_config().max_awake_dynamic, 7);
        let expected = ActivationConfig {
            max_awake_dynamic: 7,
            ..ActivationConfig::default()
        };
        assert_eq!(s.activation_config(), &expected);
    }

    /// sim 级配置:关闭凝固桥、压低醒预算(其余默认)。
    fn budget_cfg(max_awake: usize) -> ShardSimConfig {
        let mut c = ShardSimConfig::new(shard_id(0, 0));
        c.solidify.enabled = false;
        c.activation.max_awake_dynamic = max_awake;
        c
    }

    /// 测试清单(sim)3(README 决策 6,绑定):命令唤醒(Teleport 重建即醒)
    /// 超预算 → **同一拍立即**按距离淘汰(enforce_budget 语义),不得把超限
    /// 窗口拖到下一拍 plan。场景:预算 1,观察者 (8,3,8);拍 0 生成 e1(距 5)、
    /// e3(距 ≈130,当拍冻结);拍 2 前缀 Teleport(e3 → 距 2)→ 重建即醒,
    /// 醒数 {e1@5, e3@2} = 2 > 1 → 同拍淘汰最远者 e1:step 前 e1 已清速 +
    /// kinematic → 此后逐拍位级悬停;命令唤醒者 e3 照常下落。
    #[test]
    fn command_wake_culls_immediately_same_tick() {
        let mut s = ShardSim::new(budget_cfg(1), Box::new(voxel::SimpleRegistry::builtin()));
        let (e1, e3) = (entity(1), entity(3));
        s.apply_prefix(&[
            spawn_input(0, e1, Vec3::new(13.0, 3.0, 8.0)),
            spawn_input(1, e3, Vec3::new(100.0, 3.0, 100.0)),
        ]);
        // 拍 0(观察者已设):e3 距 ≈130 > 32 → 冻结;同时注册观察者供
        // 后续命令唤醒路径复用。醒数 {e1} = 1 ≤ 预算 1,不淘汰。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.entities_to_freeze, vec![e3], "非退化:e3 先被滞回冻结");
        s.step(); // 拍 1:无输入,e1 继续下落。

        // 拍 2:命令唤醒 e3(Teleport 重建,距 2 ≤ 24)→ 醒数 2 > 1 →
        // 同拍淘汰最远醒者 e1(距 5)。
        s.apply_prefix(&[teleport_input(2, e3, Vec3::new(10.0, 3.0, 8.0))]);
        s.step();
        let e3_after = s.entity_state(e3).expect("e3 命令唤醒存活");
        assert_eq!(e3_after.translation.x, 10.0, "传送落点位级生效");
        assert!(
            e3_after.translation.y < 3.0,
            "命令唤醒者当拍已受重力下落:y={}",
            e3_after.translation.y
        );
        let frozen1 = s.entity_state(e1).expect("e1 被淘汰,仍在实体表");
        assert_eq!(frozen1.linvel, Vec3::ZERO, "e1 淘汰:速度清零");
        for k in 3..8 {
            s.step();
            assert_eq!(
                s.entity_state(e1),
                Some(frozen1),
                "拍 {k}:e1 在唤醒拍 2 已被淘汰,位级悬停"
            );
        }
        // 后续拍观察者刷新(plan 路径)无残余决策:醒数已回预算内,
        // 淘汰条目在冷却窗内不得被唤醒。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert!(
            d.entities_to_freeze.is_empty() && d.entities_to_wake.is_empty(),
            "同拍淘汰后 plan 无残余决策:{d:?}"
        );
    }

    /// 测试清单(sim)4:命令唤醒不超预算 → 零额外决策(淘汰只发生在超限拍)。
    #[test]
    fn command_wake_within_budget_no_cull() {
        let mut s = ShardSim::new(budget_cfg(2), Box::new(voxel::SimpleRegistry::builtin()));
        let (e1, e2) = (entity(1), entity(2));
        s.apply_prefix(&[
            spawn_input(0, e1, Vec3::new(13.0, 3.0, 8.0)),
            spawn_input(1, e2, Vec3::new(100.0, 3.0, 100.0)),
        ]);
        // 注册观察者(e2 远 → 冻结);醒数 {e1} = 1 ≤ 预算 2。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.entities_to_freeze, vec![e2]);
        s.step();
        // 命令唤醒 e2(重建至近处):醒数 {e1@5, e2@2} = 2 ≤ 预算 2 → 不淘汰。
        s.apply_prefix(&[teleport_input(2, e2, Vec3::new(10.0, 3.0, 8.0))]);
        s.step();
        let s2 = s.entity_state(e2).expect("e2 唤醒存活");
        assert!(
            s2.translation.y < 3.0,
            "e2 命令唤醒后正常下落:y={}",
            s2.translation.y
        );
        assert!(
            s.entity_state(e1).expect("e1 存活").translation.y < 3.0,
            "e1 未被淘汰,正常下落"
        );
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert!(
            d.entities_to_freeze.is_empty(),
            "预算内命令唤醒不得触发淘汰:{d:?}"
        );
    }

    /// 测试清单(sim)5:命令唤醒路径的淘汰条目按「决策即簿记」记冷却 →
    /// 窗口内 plan 不得把它唤醒(§9.4 kinematic 化语义:解冻只走显式命令
    /// 或冷却满两条路)。
    #[test]
    fn command_wake_cull_respects_cooldown() {
        let mut s = ShardSim::new(budget_cfg(1), Box::new(voxel::SimpleRegistry::builtin()));
        let (e1, e2) = (entity(1), entity(2));
        s.apply_prefix(&[
            spawn_input(0, e1, Vec3::new(13.0, 3.0, 8.0)),
            spawn_input(1, e2, Vec3::new(100.0, 3.0, 100.0)),
        ]);
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.entities_to_freeze, vec![e2], "e2 远 → 冻结(观察者已注册)");
        s.step();
        // 拍 2:命令唤醒 e2 → 醒数 {e1, e2} = 2 > 1 → 同拍淘汰 e1(距 5 最远)。
        s.apply_prefix(&[teleport_input(2, e2, Vec3::new(10.0, 3.0, 8.0))]);
        s.step();
        assert_eq!(
            s.entity_state(e1).expect("e1 已淘汰").linvel,
            Vec3::ZERO,
            "e1 淘汰落实"
        );
        // 紧接的观察者刷新:e1 距 5 ≤ 24(唤醒半径)但淘汰簿记刚记 →
        // 冷却窗内 plan 不得唤醒。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert!(
            d.entities_to_wake.is_empty(),
            "淘汰条目入冷却簿记:窗口内 plan 不得唤醒:{d:?}"
        );
    }
}
