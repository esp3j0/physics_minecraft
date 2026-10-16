//! AC3 爆炸等价(REQ-001 §4.3 / §10;任务书 01):同一 `CrossEvent::Detonate`
//! 广播给两个分片各自求值,合并终态与单世界参考(裸 PhysicsWorld,README 决策 2)
//! **位级一致**(README 决策 3:纯弹道,每实体冲量与积分在两个世界里逐位独立)。
//!
//! 固定场景(任务书【AC3 场景设计】,坐标为任务书区间 [16,112]/[144,240] 内的固定选点):
//! - 无地面、无互触:8 个默认描述立方体(0.5 半边)悬空,z=8 一线 y=8,
//!   分片 (0,0) 4 个 x∈{64,80,96,112}、分片 (1,0) 4 个 x∈{144,160,176,192},
//!   相邻间隔 16 m(互不接触;分片本体区 x∈[0,128)/[128,256) 与 x 归属吻合);
//! - 爆心 (128, 8, 8) 恰在两分片边界上;实体到爆心距离 = {16,32,48,64}×两侧,
//!   全落在半径内且无一恰在半径边缘(任务书"不落爆炸半径边缘重叠");
//!   power/radius 照 `detonate_event_applies_symmetric_impulses` 的量级
//!   (physics-sim/tests/shard_sim.rs:238-270,power 6/实体距心 1/速度 4.8):
//!   本场景最大距离 64 m 决定 radius=72(>64 且 ≠ 任何整距离),power=12(同量级,
//!   最近实体获得 12×(1−16/72)≈9.3 m/s,与参考测试的 4.8 m/s 同量级);
//! - 分片侧:SpawnDynamic×8 + 同一 `StampedInput` 的 Event(Detonate) 以同
//!   apply_at=Tick(0) 分别 submit 给两分片(广播语义),跑 90 拍(冲量+弹道演化);
//! - 参考侧:裸 PhysicsWorld(gravity/dt 同 `ShardSimConfig::new` 默认),同参数
//!   dynamic 刚体×8,先 spawn 后 detonate(与分片侧 canonical 序一致:Command 先于
//!   Event;冲量施加序 = EntityId 升序 = 出生 x 升序,两侧一致),同 90 步。
//!
//! 非退化保障:① 参考侧每实体位移 > 1 m(任务书);② 每实体终态 linvel.x ≠ 0
//! (重力只改 y,零冲量则 x 分量逐位保持出生 0.0——x≠0 是冲量真实生效的位级证据);
//! ③ 全程零迁移(爆心在边界、冲量方向背离边界,弹道不出本体区,
//!   合并终态 = 两分片并集,等价断言不含信封链路)。

use messaging::{CrossEvent, InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
use rapier3d::prelude::*;
use shard::{Scheduler, SchedulerConfig};
use types::glam::{Quat, Vec3};
use types::{DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, Tick, WorldId};
use voxel::SimpleRegistry;

/// 场景总拍数 = 总物理步数(spawn/detonate 均在拍 0 前缀,拍 0..89 各步一次)。
const TICKS: usize = 90;
/// 爆心(任务书固定,恰在分片 (0,0)/(1,0) 边界;实体一线 y=z=8 与其共面,
/// 冲量纯水平,竖直方向只剩重力——两侧独立积分仍可对照)。
const BLAST_CENTER: Vec3 = Vec3::new(128.0, 8.0, 8.0);
/// 爆轰参数(量级依据见模块头注释)。
const BLAST_POWER: f32 = 12.0;
const BLAST_RADIUS: f32 = 72.0;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn entity(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

/// 8 个实体的出生 x(EntityId n ↔ 下标 n-1;升序 = EntityId 升序,两侧同序
/// spawn,canonical 序与 detonate 内部遍历序天然一致)。
fn spawn_xs() -> [f32; 8] {
    [64.0, 80.0, 96.0, 112.0, 144.0, 160.0, 176.0, 192.0]
}

/// 实体 n 的出生态(悬空静止,z=8 一线;任务书场景)。
fn spawn_state(n: u32) -> EntityState {
    EntityState {
        translation: Vec3::new(spawn_xs()[(n - 1) as usize], 8.0, 8.0),
        rotation: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
    }
}

/// 实体 n 所在分片(x < 128 → (0,0),否则 (1,0);与本体区 [0,128)/[128,256) 吻合)。
fn shard_of(n: u32) -> ShardId {
    shard_id(
        if spawn_xs()[(n - 1) as usize] < 128.0 {
            0
        } else {
            1
        },
        0,
    )
}

/// 实体 n 的非归属分片(迁移检查用:出现在这里 = 场景退化)。
fn foreign_shard_of(n: u32) -> ShardId {
    shard_id(
        if spawn_xs()[(n - 1) as usize] < 128.0 {
            1
        } else {
            0
        },
        0,
    )
}

/// 虚空分片(无地面 chunk;凝固桥关闭——悬空体受重力低速窗口若开桥会被凝固)。
fn void_sim(x: i32, z: i32) -> ShardSim {
    let mut cfg = ShardSimConfig::new(shard_id(x, z));
    cfg.solidify = SolidifyConfig {
        enabled: false,
        ..SolidifyConfig::default()
    };
    ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
}

/// 盖章输入(apply_at = 拍 0):spawn 命令与爆炸事件同一前缀;分片侧 canonical 序
/// 保证 Command×4(按 sequence 升序)先于 Event(类型序号,ordering.rs KIND 位)。
fn input(sequence: u64, payload: InputPayload) -> StampedInput {
    StampedInput {
        apply_at: Tick(0),
        source: InputSource::SCHEDULER,
        sequence,
        payload,
    }
}

fn detonate_event() -> CrossEvent {
    CrossEvent::Detonate {
        center: BLAST_CENTER,
        power: BLAST_POWER,
        radius: BLAST_RADIUS,
    }
}

/// 分片侧一次完整跑动。
struct ShardRun {
    /// 终态:n → 归属分片终态(本场景必为 Some,None = 迁移/消失,断言兜底)。
    final_states: [Option<EntityState>; 8],
    /// 逐拍"实体出现在非归属分片"计数(非退化保障③:必为 0 = 零迁移)。
    migrations: usize,
}

/// 分片侧:Scheduler 两虚空分片 + 8×Spawn + 同一 Detonate 广播 submit + 90 拍。
fn run_shards() -> ShardRun {
    let mut sched = Scheduler::new(SchedulerConfig {
        worker_count: 2,
        ..SchedulerConfig::default()
    });
    sched.register(void_sim(0, 0));
    sched.register(void_sim(1, 0));

    for n in 1..=8u32 {
        sched
            .submit(
                shard_of(n),
                input(
                    u64::from(n),
                    InputPayload::Command(SimCommand::SpawnDynamic {
                        entity: entity(n),
                        state: spawn_state(n),
                        desc: EntityDesc::default(),
                    }),
                ),
            )
            .expect("收件箱未满");
    }
    // 广播语义:同一 CrossEvent::Detonate 以同 apply_at 分别 submit 给两个分片。
    let blast = input(100, InputPayload::Event(detonate_event()));
    for n_shard in [shard_id(0, 0), shard_id(1, 0)] {
        sched.submit(n_shard, blast.clone()).expect("收件箱未满");
    }

    let mut migrations = 0usize;
    for _ in 0..TICKS {
        sched.tick();
        for n in 1..=8u32 {
            // 非归属分片查出实体 = 发生过迁移(场景必须零迁移,见头注释③)。
            if sched.entity_state(foreign_shard_of(n), entity(n)).is_some() {
                migrations += 1;
            }
        }
    }
    let final_states =
        std::array::from_fn(|i| sched.entity_state(shard_of(i as u32 + 1), entity(i as u32 + 1)));
    sched.shutdown();
    ShardRun {
        final_states,
        migrations,
    }
}

/// 参考侧:裸 PhysicsWorld(同 gravity/dt/刚体参数),spawn×8 → Detonate → 90 步。
fn run_reference() -> [EntityState; 8] {
    let cfg = ShardSimConfig::new(shard_id(0, 0));
    let mut phys = PhysicsWorld::new();
    phys.gravity = cfg.gravity;
    phys.integration_parameters.dt = cfg.dt;

    let mut handles = Vec::new();
    for n in 1..=8u32 {
        let s = spawn_state(n);
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
        handles.push(handle);
    }
    // 与分片侧 canonical 序一致:全部 spawn 先于 detonate;冲量施加序 = 出生 x
    // 升序(分片侧 detonate 按 entities BTreeMap 的 EntityId 升序遍历,同序)。
    let CrossEvent::Detonate {
        center,
        power,
        radius,
    } = detonate_event();
    for handle in &handles {
        let pos = phys.bodies.get(*handle).expect("自家刚体").translation();
        if let Some(impulse) = physics_sim::explosion::explosion_impulse(center, power, radius, pos)
        {
            phys.bodies
                .get_mut(*handle)
                .expect("自家刚体")
                .apply_impulse(impulse, true);
        }
    }
    for _ in 0..TICKS {
        phys.step();
    }
    std::array::from_fn(|i| {
        let b = phys.bodies.get(handles[i]).expect("自家刚体");
        EntityState {
            translation: b.translation(),
            rotation: *b.rotation(),
            linvel: b.linvel(),
            angvel: b.angvel(),
        }
    })
}

/// AC3 主断言:8 个实体两侧终态 EntityState 位级 `==`
/// (translation/rotation/linvel/angvel 四向量逐位,不许近似)。
#[test]
fn ac3_explosion_bitwise_equivalent() {
    let shard_run = run_shards();
    let reference = run_reference();
    // 非退化保障③:零迁移——合并终态 = 两分片并集,等价断言不含信封链路。
    assert_eq!(shard_run.migrations, 0, "场景必须零迁移(弹道不出本体区)");
    for n in 1..=8u32 {
        let shard_side =
            shard_run.final_states[(n - 1) as usize].expect("场景保证实体不迁移不消失");
        assert_eq!(
            shard_side,
            reference[(n - 1) as usize],
            "实体 {n}:分片侧(各自求值)与单世界参考必须位级一致"
        );
        // 非退化保障②:重力只改 y,x 分量逐位保持出生 0.0 当且仅当冲量未生效。
        assert!(
            shard_side.linvel.x != 0.0,
            "实体 {n}:终态 linvel.x = 0 → 爆炸冲量未生效,场景退化"
        );
    }
}

/// 非退化保障(任务书):参考侧每实体位移 > 1 m(爆炸+弹道确实改变轨迹)。
#[test]
fn ac3_blast_actually_moved_everyone() {
    let reference = run_reference();
    for n in 1..=8u32 {
        let start = spawn_state(n).translation;
        let end = reference[(n - 1) as usize].translation;
        let disp = (end - start).length();
        assert!(
            disp > 1.0,
            "实体 {n}:参考侧位移 {disp:.3} m 必须 > 1 m(场景真实演化)"
        );
    }
}
