//! AC2 迁移等价(REQ-001 §10;任务书 00):实体跨分片迁移前后轨迹与不迁移
//! 对照(裸 PhysicsWorld 单世界,README 决策 2)误差 < 1e-4 m(接触态)。
//!
//! 固定场景(任务书【场景设计】):走廊 x∈[0,256)、z∈[0,16),整层地面,
//! 顶层 world y=64——照 two_shards.rs 注释口径"顶层实心面 world y = top_ly - 64"
//! 取 `ground_chunk(128)`(顶面平面 y=64,薄片面 [63.925, 64.075]),凝固桥关闭;
//! 实体 = 默认描述(0.5 半边立方体)出生 (100, 65.5, 8)、linvel=(60,0,0),
//! 触地后接触滑动/弹跳滚动,拍 ~29 跨 x=128 分片边界;SpawnDynamic apply_at=Tick(1);
//! 跑 45 拍(执行第 0..44 拍,实体在拍 1..44 共 44 步物理)。
//!
//! 分片侧:Scheduler 两分片 (0,0)/(1,0),地面 chunk 各自装载并含扩张带重叠
//! (片 0 = chunk x∈0..=8,片 1 = chunk x∈7..=15);参考侧:走廊全部 chunk 经
//! `greedy_colliders` 生成的单一 compound(fixed,friction 0.6/restitution 0.0,
//! 照 `sim.rs::rebuild_chunk_collider`)+ 逐参数一致的 dynamic 刚体 + 44 步。

use std::collections::BTreeMap;

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::collider_build::{NeighborChunks, greedy_colliders};
use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
use rapier3d::prelude::*;
use shard::{Scheduler, SchedulerConfig};
use types::glam::{Quat, Vec3};
use types::{
    BlockId, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, Tick,
    WorldId,
};
use voxel::{BlockRegistry, Chunk, SimpleRegistry};

/// 场景总拍数:执行第 0..44 拍(README 决策 7:第 k 次 tick() 执行第 k-1 拍)。
const TICKS: usize = 45;
/// 实体物理步数:apply_at=Tick(1) → 拍 1..44 各一拍一步。
const ENTITY_STEPS: usize = TICKS - 1;
/// 地面顶层:`ground_chunk(128)` 顶面实心面 world y = 128 - 64 = 64(任务书口径)。
const GROUND_TOP_LY: usize = 128;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn entity() -> EntityId {
    EntityId::from_parts(1, 1)
}

/// 顶层实心面在 world y = top_ly - 64 的地形 chunk(照 two_shards.rs 帮助函数)。
fn ground_chunk(top_ly: usize) -> Chunk {
    let mut c = Chunk::air();
    for y in 0..top_ly {
        for x in 0..16 {
            for z in 0..16 {
                c.set(x, y, z, BlockId(1));
            }
        }
    }
    c
}

/// 出生/迁移后的实体出生态(两侧共同的唯一初值,任务书固定)。
fn spawn_state() -> EntityState {
    EntityState {
        translation: Vec3::new(100.0, 65.5, 8.0),
        rotation: Quat::IDENTITY,
        linvel: Vec3::new(60.0, 0.0, 0.0),
        angvel: Vec3::ZERO,
    }
}

/// 分片侧一次完整跑动:逐拍双片采样(非退化保障 + 终态)。
struct ShardRun {
    /// 第 k 拍执行完后源片 (0,0) 的实体状态。
    src_at: Vec<Option<EntityState>>,
    /// 第 k 拍执行完后目的片 (1,0) 的实体状态。
    dst_at: Vec<Option<EntityState>>,
}

/// 带地面的分片模拟核:凝固桥关闭(任务书"凝固桥关闭"),chunk 全量装载。
fn ground_sim(shard_x: i32, chunk_xs: std::ops::Range<i32>) -> ShardSim {
    let mut s = ShardSim::new(
        ShardSimConfig {
            solidify: SolidifyConfig {
                enabled: false,
                ..SolidifyConfig::default()
            },
            ..ShardSimConfig::new(shard_id(shard_x, 0))
        },
        Box::new(SimpleRegistry::builtin()),
    );
    for cx in chunk_xs {
        s.load_chunk(ChunkPos::new(cx, 0), ground_chunk(GROUND_TOP_LY));
    }
    s
}

/// 分片侧:Scheduler 两分片 + 扩张带重叠装载 + 出生命令 + 45 拍逐拍采样。
fn run_shards() -> ShardRun {
    let (src, dst) = (shard_id(0, 0), shard_id(1, 0));
    let mut sched = Scheduler::new(SchedulerConfig {
        worker_count: 2,
        ..SchedulerConfig::default()
    });
    // 本体区:片 0 = chunk 0..8,片 1 = chunk 8..16;扩张带重叠:各多载邻列。
    sched.register(ground_sim(0, 0..9));
    sched.register(ground_sim(1, 7..16));

    sched
        .submit(
            src,
            StampedInput {
                apply_at: Tick(1),
                source: InputSource(7),
                sequence: 0,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: entity(),
                    state: spawn_state(),
                    desc: EntityDesc::default(),
                }),
            },
        )
        .expect("收件箱未满");

    let mut src_at = Vec::with_capacity(TICKS);
    let mut dst_at = Vec::with_capacity(TICKS);
    for _ in 0..TICKS {
        sched.tick();
        src_at.push(sched.entity_state(src, entity()));
        dst_at.push(sched.entity_state(dst, entity()));
    }
    sched.shutdown();
    ShardRun { src_at, dst_at }
}

/// 走廊全部 chunk(x∈0..16,z=0)体素表(参考世界唯一地面数据源)。
fn corridor_chunks() -> BTreeMap<ChunkPos, Chunk> {
    let mut chunks = BTreeMap::new();
    for cx in 0..16 {
        chunks.insert(ChunkPos::new(cx, 0), ground_chunk(GROUND_TOP_LY));
    }
    chunks
}

/// 参考侧:裸 PhysicsWorld(gravity/dt 同 `ShardSimConfig::new` 默认),
/// 地面 = 全部 chunk 的 greedy_colliders 合成单一 compound(fixed),
/// 实体 = 与 `spawn_entity` 逐参数一致的 dynamic + cuboid,同初态跑 44 步。
fn run_reference() -> EntityState {
    let mut phys = PhysicsWorld::new();
    phys.gravity = Vec3::new(0.0, -9.81, 0.0);
    phys.integration_parameters.dt = 1.0 / 60.0;

    // 地面:与分片侧同一纯函数、同一 solid 判据、同一 compound 装配参数
    //(照 sim.rs::rebuild_chunk_collider:friction 0.6 / restitution 0.0)。
    let chunks = corridor_chunks();
    let registry = SimpleRegistry::builtin();
    let get = |x: i32, z: i32| chunks.get(&ChunkPos::new(x, z));
    let mut parts: Vec<(Pose, SharedShape)> = Vec::new();
    for cx in 0..16 {
        let cp = ChunkPos::new(cx, 0);
        let neighbors = NeighborChunks {
            px: get(cx + 1, 0),
            nx: get(cx - 1, 0),
            pz: get(cx, 1),
            nz: get(cx, -1),
        };
        for b in greedy_colliders(
            get(cx, 0).expect("走廊 chunk 必在"),
            cp,
            &neighbors,
            |b| registry.is_solid(b),
        ) {
            parts.push((
                Pose {
                    rotation: Quat::IDENTITY,
                    translation: b.center,
                    ..Pose::IDENTITY
                },
                SharedShape::cuboid(b.half.x, b.half.y, b.half.z),
            ));
        }
    }
    phys.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::compound(parts)
            .friction(0.6)
            .restitution(0.0),
    );

    // 实体:与 sim.rs::spawn_entity 逐参数一致(密度 1/摩擦 0.6/弹性 0.1)。
    let s = spawn_state();
    let h = EntityDesc::default().half_size;
    let (handle, _) = phys.insert(
        RigidBodyBuilder::dynamic()
            .pose(Pose {
                rotation: s.rotation,
                translation: s.translation,
                ..Pose::IDENTITY
            })
            .linvel(s.linvel)
            .angvel(s.angvel),
        ColliderBuilder::cuboid(h, h, h)
            .density(1.0)
            .friction(0.6)
            .restitution(0.1),
    );
    for _ in 0..ENTITY_STEPS {
        phys.step();
    }
    let b = phys.bodies.get(handle).expect("刚体在自家世界");
    EntityState {
        translation: b.translation(),
        rotation: *b.rotation(),
        linvel: b.linvel(),
        angvel: b.angvel(),
    }
}

/// 迁移确实发生:源片 Some→None 拍与目的片首次 Some 拍差恰 1。
fn crossing_tick(r: &ShardRun) -> u64 {
    let vanish = r
        .src_at
        .windows(2)
        .position(|w| w[0].is_some() && w[1].is_none())
        .expect("实体必须先从源片消失") as u64
        + 1;
    let appear = r
        .dst_at
        .iter()
        .position(|s| s.is_some())
        .expect("实体必须随后在目的片出现") as u64;
    assert_eq!(
        appear,
        vanish + 1,
        "迁移拍差必为 1(apply_at ≡ produced_at + 1):vanish={vanish} appear={appear}"
    );
    vanish
}

/// AC2 主断言:两侧终态 translation 各分量差 < 1e-4 m;linvel 差 < 1e-3 m/s。
#[test]
fn ac2_contact_crossing_matches_single_world() {
    let r = run_shards();
    let _ = crossing_tick(&r); // 非退化保障:等价断言必须建立在迁移确实发生之上
    let shard_end = r.dst_at[TICKS - 1].expect("终态实体在目的片");
    let reference = run_reference();

    let d = shard_end.translation - reference.translation;
    assert!(
        d.x.abs() < 1e-4 && d.y.abs() < 1e-4 && d.z.abs() < 1e-4,
        "AC2 接触态迁移轨迹误差超限:Δ=({},{},{}) m,分片侧={:?} 参考={:?}",
        d.x,
        d.y,
        d.z,
        shard_end.translation,
        reference.translation
    );
    let dv = shard_end.linvel - reference.linvel;
    assert!(
        dv.x.abs() < 1e-3 && dv.y.abs() < 1e-3 && dv.z.abs() < 1e-3,
        "速度对照超限:Δv=({},{},{}) m/s",
        dv.x,
        dv.y,
        dv.z
    );
}

/// 防回归恶化:同场景 translation 误差实测上界 < 1e-5 m(demo A 基线 ~5e-6)。
/// 若实测不满足,任务书允许只保留 1e-4 断言并在 notes 说明——本断言先行如实报数。
#[test]
fn ac2_error_far_below_tolerance() {
    let r = run_shards();
    let _ = crossing_tick(&r);
    let shard_end = r.dst_at[TICKS - 1].expect("终态实体在目的片");
    let reference = run_reference();
    let d = shard_end.translation - reference.translation;
    let max_err = d.x.abs().max(d.y.abs()).max(d.z.abs());
    assert!(
        max_err < 1e-5,
        "实测误差上界 {max_err:.3e} m 未达 1e-5(demo A 基线 ~5e-6):如实报数"
    );
}

/// 非退化保障:分片侧逐拍采样证明实体确实迁移(源片 None / 目的片 Some,拍差 1)。
#[test]
fn crossing_actually_happened() {
    let r = run_shards();
    let vanish = crossing_tick(&r);
    assert!(
        vanish >= 2 && vanish < TICKS as u64,
        "迁移必须发生在场景中段(拍 {vanish}/共 {TICKS} 拍)"
    );
    assert!(
        r.dst_at[TICKS - 1].is_some(),
        "迁移后实体留在目的片(场景内不再越界)"
    );
}
