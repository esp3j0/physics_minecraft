//! `ShardSim`:单分片确定性物理核(rapier 封装,REQ-001 §4.1/§4.3)。
//!
//! 单个分片 = 一个 rapier `PhysicsWorld` + 自己的体素 chunk(`VoxelWorld`)+ 实体表。
//! 一切状态变更只经 `apply_prefix`(盖章输入);`step` 推进一拍后在 tick 末做
//! 凝固桥检测与迁移检测(demo A/C 协议);产出经 `take_outbox` 交给调度器。
//! rapier 类型不出本模块的公开接口(防腐层,蓝图 §0)。

use std::collections::{BTreeMap, BTreeSet};

use rapier3d::prelude::*;
use types::glam::Vec3;
use types::{
    BlockPos, CHUNK_SIZE, ChunkPos, EntityDesc, EntityId, EntityState, SHARD_CHUNKS, ShardId,
};
use voxel::{BlockRegistry, Chunk, VoxelEdit, VoxelWorld};

use crate::collider_build::{BoxCollider, NeighborChunks, greedy_colliders};
use crate::conv;
use crate::solidify;
use messaging::{CrossEvent, EnvelopePayload, InputPayload, Intent, SimCommand, StampedInput};

/// 凝固桥触发参数(P2 + 睡眠兜底,REQ-001 §9.5 / demo C)。
#[derive(Debug, Clone, Copy)]
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

/// 单分片配置。
#[derive(Debug, Clone)]
pub struct ShardSimConfig {
    pub shard: ShardId,
    pub gravity: Vec3,
    pub dt: f32,
    pub solidify: SolidifyConfig,
}

impl ShardSimConfig {
    pub fn new(shard: ShardId) -> Self {
        Self {
            shard,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            dt: 1.0 / 60.0,
            solidify: SolidifyConfig::default(),
        }
    }
}

/// 本 tick 产出(调度器消费):迁移意图、collider 重建过的 chunk、凝固记录。
#[derive(Debug, Default, PartialEq)]
pub struct Outbox {
    /// 迁移意图(调度器盖章成信封,REQ-001 §4.2 (b))。
    pub migrations: Vec<Intent>,
    /// 本 tick collider 重建过的 chunk(可见性/换装统计用)。
    pub dirty_chunks: BTreeSet<ChunkPos>,
    /// 凝固完成记录:(实体, 写入的格)。
    pub solidified: Vec<(EntityId, BlockPos)>,
}

/// 实体的分片侧记录。
struct EntityRecord {
    handle: RigidBodyHandle,
    desc: EntityDesc,
    /// 连续"低速"tick 计数(P2 触发用)。
    slow_ticks: u32,
}

/// 单分片模拟器。
pub struct ShardSim {
    cfg: ShardSimConfig,
    registry: Box<dyn BlockRegistry>,
    voxels: VoxelWorld,
    phys: PhysicsWorld,
    entities: BTreeMap<EntityId, EntityRecord>,
    chunk_bodies: BTreeMap<ChunkPos, RigidBodyHandle>,
    outbox: Outbox,
}

impl ShardSim {
    pub fn new(cfg: ShardSimConfig, registry: Box<dyn BlockRegistry>) -> Self {
        let mut phys = PhysicsWorld::new();
        phys.gravity = conv::vec3_to_rapier(cfg.gravity);
        phys.integration_parameters.dt = cfg.dt;
        Self {
            cfg,
            registry,
            voxels: VoxelWorld::new(),
            phys,
            entities: BTreeMap::new(),
            chunk_bodies: BTreeMap::new(),
            outbox: Outbox::default(),
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
    }

    /// 推进一拍:物理步进 → 凝固桥检测 → 迁移检测(demo A/C 时序)。
    pub fn step(&mut self) {
        self.phys.step();

        let ids: Vec<EntityId> = self.entities.keys().copied().collect();
        let orients = solidify::axis_aligned_orientations();
        let cfg = self.cfg.solidify;
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
            let pos = conv::rapier_to_vec3(
                self.phys
                    .bodies
                    .get(rec.handle)
                    .expect("handle 有效")
                    .translation(),
            );
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
