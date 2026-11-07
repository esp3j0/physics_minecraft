//! REQ-012 A00-3 demo `migrate`(2026-11-07)。
//!
//! 问题(A00-3 放行原文):丢掉接触缓存重建后,四足会不会跳变或摔倒?
//! 要验证的设计:迁移信封只带身体哈希、根状态、关节坐标、关节速度、最后的
//! 目标,整个主体同一拍迁移。
//!
//! 重建方式三种(都跑):`same` = 同一世界内移除再重建(只隔离"重建 +
//! 丢缓存"的影响);`cross` = 移到第二个世界(地形相同,两个世界同拍步进);
//! `cross_hist` = cross + 目标世界先插入、删除过 200 个刚体 + 20 m 外
//! 3×3×3 无关方块堆(暴露句柄历史和求解顺序的影响)。
//!
//! 状态快照两种(都跑):`full` = 完整快照(根位姿、根速度、关节坐标、
//! 关节速度、目标);`novec` = 不带速度(速度清零,说明速度是否必须带)。
//!
//! 统一口径(A00-3 放行):60 Hz、生产求解参数(只覆写 dt)、默认步态
//! (kp 100、kd 3、3 Hz)、k = 1/100、阻尼按 k 缩放、30 Hz 更新目标、
//! 不开 extra。
//!
//! `cross` / `cross_hist` 的轨迹是否相同只报数(diverge_tick:与同场景
//! `same_full` 行的逐拍状态哈希比较的首个分歧拍;不是判据,由总架构师
//! 裁定生产迁移的确定性定义)。

use std::collections::BTreeMap;
use std::time::Instant;

use physics_sim::collider_build::{greedy_colliders, BoxCollider, NeighborChunks};
use quad::{
    default_gait, ik, production_params, read_angle, restore_robot, set_target, snapshot_robot,
    spawn_robot, DampMode, GROUND_RESTITUTION, N_JOINTS, TAU_LIMIT,
};
use rapier3d::dynamics::{RigidBodyBuilder, RigidBodyHandle};
use rapier3d::geometry::{ColliderBuilder, ColliderHandle, PairContacts, SharedShape};
use rapier3d::math::{Pose, Vector};
use rapier3d::pipeline::PhysicsWorld;
use types::{ChunkPos, WorldSeed};
use voxel::{BlockRegistry as _, WorldGenerator as _};
use voxel::{Chunk, SimpleRegistry, Superflat};

/// 地面顶面(T1 体素口径,同 terrain:世界 y = −60;T0 平面顶面同高)。
const GROUND_TOP: f32 = -60.0;
/// chunk x ∈ −2..CHUNK_SPAN(同 terrain T1:出生 x=10,跨 x=16/32 缝)。
const CHUNK_SPAN: i32 = 14;
const CHUNK_Z_LO: i32 = -2;
const CHUNK_Z_HI: i32 = 2;
/// 站立序列结束拍(1 s 直腿降 + 2 s 稳;同 mass/terrain)。
const STAND_TICKS: usize = 180;
/// 迁移后跟踪窗口(判据 2)。
const POST_TICKS: usize = 120;
/// 单次迁移组总拍数(站立 180 + 行走 1320 ≈ 25 s)。PingPong 固定 3600(放行)。
const WALK_TICKS: usize = STAND_TICKS + 1320;
/// 来回迁移组总拍数。
const PINGPONG_TICKS: usize = 3600;
/// 质量比例 k(压缩档,§4;A00-3 统一口径)。
const K: f32 = 1.0 / 100.0;

/// 重建方式。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Rebuild {
    /// 同一世界内移除再重建。
    Same,
    /// 移到第二个世界(地形相同,两世界同拍步进)。
    Cross,
    /// cross + 目标世界历史(插入/删除 200 刚体 + 20 m 外 3×3×3 堆)。
    CrossHist,
}

impl Rebuild {
    fn tag(self) -> &'static str {
        match self {
            Rebuild::Same => "same",
            Rebuild::Cross => "cross",
            Rebuild::CrossHist => "cross_hist",
        }
    }
}

/// 快照口径。
#[derive(Clone, Copy, PartialEq, Eq)]
enum SnapKind {
    /// 完整快照(根位姿、根速度、关节坐标、关节速度、目标)。
    Full,
    /// 不带速度(根速度与关节速度清零)。
    Novec,
}

impl SnapKind {
    fn tag(self) -> &'static str {
        match self {
            SnapKind::Full => "full",
            SnapKind::Novec => "novec",
        }
    }
}

/// 场景。
#[derive(Clone, Copy, PartialEq)]
enum Scene {
    /// 行走中迁移:步态周期(f3 = 20 拍)内 12 个相位。
    WalkPhase { phase: usize },
    /// 醒着站立(每拍 wake_up)。
    StandAwake,
    /// 已睡眠的站立(站稳后不再唤醒,睡满 600 拍迁移)。
    StandAsleep,
    /// 落地瞬间(至少一只脚刚接触的那一拍;直腿出生降落首次接触)。
    LandTick,
    /// 连续来回迁移:间隔 gap 拍,跑 3600 拍。
    PingPong { gap: usize },
    /// 对照:同参数、不迁移的同一条轨迹(站立窗口;基点 walk0)。
    Control,
    /// 行走对照:不迁移、步行目标,窗口基点 = 对应相位的迁移拍。
    ControlWalk { phase: usize },
    /// 落地对照:不迁移、落地布景同款(瞬移 0.5 m 落下),基点 = 落地拍。
    ControlLand,
    /// 8 只同拍迁移(same/full,T0)。
    Eight,
}

struct Cfg {
    name: String,
    scene: Scene,
    rebuild: Rebuild,
    snap: SnapKind,
    /// T0 = 大平面;T1 = chunk 接缝。
    t1: bool,
    /// 8 只组(独立路径,见 run_eight)。
    eight: bool,
}

fn hash_f32(h: u64, v: f32) -> u64 {
    let mut h = h;
    for b in &v.to_ne_bytes() {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

// ── 地形(T1 与 terrain 完全同构) ──────────────────────────────────────

fn build_voxels_t1() -> Vec<(ChunkPos, Chunk)> {
    let gen = Superflat::classic();
    let mut out = Vec::new();
    for cx in -2..CHUNK_SPAN {
        for cz in CHUNK_Z_LO..=CHUNK_Z_HI {
            let cp = ChunkPos::new(cx, cz);
            out.push((cp, gen.generate(WorldSeed(0), cp)));
        }
    }
    out
}

fn neighbor_lookup(voxels: &[(ChunkPos, Chunk)], x: i32, z: i32) -> Option<&Chunk> {
    voxels
        .iter()
        .find(|(c, _)| c.x == x && c.z == z)
        .map(|(_, ch)| ch)
}

/// 与生产 rebuild_chunk_collider 相同:greedy_colliders → fixed compound,
/// friction 0.6 / restitution 0。返回全部地形 collider。
fn build_terrain(world: &mut PhysicsWorld, t1: bool) -> Vec<ColliderHandle> {
    if !t1 {
        let (_, collider) = world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, GROUND_TOP - 0.5, 0.0)),
            ColliderBuilder::cuboid(400.0, 0.5, 400.0)
                .friction(quad::GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
        );
        return vec![collider];
    }
    let reg = SimpleRegistry::builtin();
    let voxels = build_voxels_t1();
    let mut handles = Vec::new();
    for (cp, chunk) in &voxels {
        let neighbors = NeighborChunks {
            px: neighbor_lookup(&voxels, cp.x + 1, cp.z),
            nx: neighbor_lookup(&voxels, cp.x - 1, cp.z),
            pz: neighbor_lookup(&voxels, cp.x, cp.z + 1),
            nz: neighbor_lookup(&voxels, cp.x, cp.z - 1),
        };
        let boxes: Vec<BoxCollider> = greedy_colliders(chunk, *cp, &neighbors, |b| reg.is_solid(b));
        if boxes.is_empty() {
            continue;
        }
        let parts: Vec<(Pose, SharedShape)> = boxes
            .iter()
            .map(|b| {
                (
                    Pose {
                        translation: Vector::new(b.center.x, b.center.y, b.center.z),
                        ..Pose::IDENTITY
                    },
                    SharedShape::cuboid(b.half.x, b.half.y, b.half.z),
                )
            })
            .collect();
        let (_body, collider) = world.insert(
            RigidBodyBuilder::fixed(),
            ColliderBuilder::compound(parts)
                .friction(quad::GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
        );
        handles.push(collider);
    }
    handles
}

/// cross_hist 的历史污染:插入、删除 200 个刚体(句柄历史)+ 20 m 外
/// 3×3×3 无关方块堆(求解顺序;生产口径同 mass Isolation)。
fn pollute_history(world: &mut PhysicsWorld) {
    let mut tmp: Vec<RigidBodyHandle> = Vec::new();
    for i in 0..200u32 {
        let (h, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(
                -50.0 + ((i % 50) as f32),
                5.0 + ((i / 50) as f32) * 2.0,
                0.0,
            )),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5).density(1.0),
        );
        tmp.push(h);
    }
    for h in tmp {
        world.remove_body(h);
    }
    for ix in 0..3i32 {
        for iy in 0..3i32 {
            for iz in 0..3i32 {
                let pos = Vector::new(
                    30.0 + ix as f32,
                    GROUND_TOP + 0.5 + 1.0 + iy as f32,
                    iz as f32,
                );
                world.insert(
                    RigidBodyBuilder::dynamic().translation(pos),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5)
                        .density(1.0)
                        .friction(0.6)
                        .restitution(0.1),
                );
            }
        }
    }
}

// ── 公共运行件 ─────────────────────────────────────────────────────────

/// 一侧世界:地形 + 可选机器人。
struct Side {
    world: PhysicsWorld,
    robot: Option<quad::Robot>,
    terrain: Vec<ColliderHandle>,
}

fn make_side(t1: bool) -> Side {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = production_params(1.0 / 60.0);
    let terrain = build_terrain(&mut world, t1);
    Side {
        world,
        robot: None,
        terrain,
    }
}

fn spawn_into(side: &mut Side) -> quad::Robot {
    let y0 = GROUND_TOP + 2.0 * quad::L + quad::FOOT_R + 0.002;
    spawn_robot(
        &mut side.world,
        Vector::new(10.0, y0, 0.0),
        K,
        quad::GROUND_FRICTION,
        DampMode::Scaled,
    )
}

/// 步态目标(站立或行走 tick 的 12 关节目标;与 terrain 同口径)。
/// `walking = false` 的场景(站立/睡眠/落地/对照)全程保持站姿目标。
fn targets_at(gait: &quad::Gait, tick: usize, walk0: usize, walking: bool) -> [f32; N_JOINTS] {
    let mut targets = [0.0f32; N_JOINTS];
    let walk_start = if walking { walk0 } else { usize::MAX };
    for leg in 0..4 {
        let foot = if tick < 60 {
            let a = tick as f32 / 60.0;
            let sm = a * a * (3.0 - 2.0 * a);
            let (x, _) = gait.stand_foot();
            (
                x,
                quad::FULL_STRETCH_DY + (-gait.h - quad::FULL_STRETCH_DY) * sm,
            )
        } else if tick < walk_start {
            gait.stand_foot()
        } else {
            let walk_t = (tick - walk_start) as f32 / 60.0;
            gait.foot_target(leg, walk_t)
        };
        let (dx, dy) = foot;
        let (a, k) = ik(dx, dy);
        targets[leg * 3] = 0.0;
        targets[leg * 3 + 1] = a;
        targets[leg * 3 + 2] = k;
    }
    targets
}

fn write_targets(world: &mut PhysicsWorld, robot: &quad::Robot, targets: &[f32; N_JOINTS]) {
    let kp = K * quad::DEFAULT_KP;
    let kd = K * quad::DEFAULT_KD;
    let tau = K * TAU_LIMIT;
    for (dof, tgt) in targets.iter().enumerate() {
        set_target(world, robot, dof, *tgt, kp, kd, tau);
    }
}

fn wake_all(world: &mut PhysicsWorld, robot: &quad::Robot) {
    for h in robot
        .links
        .iter()
        .copied()
        .chain(std::iter::once(robot.body))
    {
        if let Some(rb) = world.bodies.get_mut(h) {
            rb.wake_up(true);
        }
    }
}

/// 移除整只(remove_body 连带拆 multibody joint 与 collider)。
fn remove_robot(side: &mut Side, robot: &quad::Robot) {
    let mut all: Vec<RigidBodyHandle> = robot.links.clone();
    all.push(robot.body);
    for h in all {
        side.world.remove_body(h);
    }
}

/// 任一脚对地形有真实接触(dist<1mm 流形)。
fn foot_touching(world: &PhysicsWorld, r: &quad::Robot, terrain: &[ColliderHandle]) -> bool {
    for pair in world.narrow_phase.contact_graph().interactions() {
        if let PairContacts::Rigid(rigid) = &pair.contacts {
            let robot_side =
                r.colliders.contains(&pair.collider1) || r.colliders.contains(&pair.collider2);
            let ground_side =
                terrain.contains(&pair.collider1) || terrain.contains(&pair.collider2);
            if !(robot_side && ground_side) {
                continue;
            }
            for manifold in &rigid.manifolds {
                if manifold.points.is_empty() {
                    continue;
                }
                if manifold
                    .points
                    .iter()
                    .map(|p| p.dist)
                    .reduce(f32::max)
                    .unwrap_or(f32::INFINITY)
                    < 0.001
                {
                    return true;
                }
            }
        }
    }
    false
}

struct Outcome {
    name: String,
    /// 重建后、步进前:(关节坐标 rad,连杆位置 m,连杆转角 rad,速度)。
    err_coord: f32,
    err_pos: f32,
    err_rot: f32,
    err_vel: f32,
    /// 迁移后 POST_TICKS 窗口。
    post_upy_min: f32,
    post_max_vy: f32,
    post_link_max: f32,
    post_pen_max: f32,
    /// 迁移后首次真实脚-地接触(dist<1mm)相对迁移拍。
    contact_recover_tick: Option<usize>,
    /// 全程 vfwd(行走段均值,m/s;对照同窗口)。
    vfwd: f32,
    fell_tick: Option<usize>,
    nan: bool,
    #[allow(dead_code)] // CSV 列占位(migrate 无 panic 隔离需求;freeze 同列序对齐)
    panicked: bool,
    panic_msg: String,
    /// 单次迁移耗时(µs;restore_robot 调用,Eight 组报 8 只平均)。
    migrate_us: f32,
    /// 与同场景 same_full 行逐拍哈希的首个分歧拍(cross/cross_hist;None = 无)。
    diverge_tick: Option<usize>,
    /// 落地组:实际迁移拍。
    land_tick: Option<usize>,
    state_hash: u64,
}

fn outcome_csv(o: &Outcome) -> String {
    format!(
        "{},{:.3e},{:.3e},{:.3e},{:.3e},{:.4},{:.4},{:.4},{:.4},{},{:.4},{},{},false,{},{:.1},{},{},{:016x}",
        o.name,
        o.err_coord,
        o.err_pos,
        o.err_rot,
        o.err_vel,
        o.post_upy_min,
        o.post_max_vy,
        o.post_link_max,
        o.post_pen_max,
        o.contact_recover_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.vfwd,
        o.fell_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.nan,
        o.panic_msg,
        o.migrate_us,
        o.diverge_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.land_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.state_hash,
    )
}

/// 场景键(diverge 存档;same_full 行产出,cross/cross_hist 行消费)。
fn scene_key(cfg: &Cfg) -> Option<String> {
    let t = if cfg.t1 { "T1" } else { "T0" };
    match cfg.scene {
        Scene::WalkPhase { phase } => Some(format!("walk_p{phase:02}_{t}")),
        Scene::StandAwake => Some(format!("stand_awake_{t}")),
        Scene::StandAsleep => Some(format!("stand_asleep_{t}")),
        Scene::LandTick => Some(format!("land_{t}")),
        _ => None,
    }
}

fn run(cfg: &Cfg, ref_hashes: &[u64]) -> (Outcome, Vec<u64>) {
    let gait = default_gait();
    let period = (60.0 / gait.freq).round() as usize;
    let walk0 = STAND_TICKS;
    // 行走口径只属于 WalkPhase / PingPong / 行走对照;站立、睡眠、落地、
    // 站立对照全程站姿。
    let walking = matches!(
        cfg.scene,
        Scene::WalkPhase { .. } | Scene::PingPong { .. } | Scene::ControlWalk { .. }
    );
    let total_ticks = match cfg.scene {
        Scene::PingPong { .. } => PINGPONG_TICKS,
        _ => WALK_TICKS,
    };

    let mut migrate_ticks: Vec<usize> = Vec::new();
    match cfg.scene {
        Scene::WalkPhase { phase } => migrate_ticks.push(walk0 + phase * period / 12),
        Scene::StandAwake => migrate_ticks.push(walk0),
        Scene::StandAsleep => migrate_ticks.push(walk0 + 600),
        Scene::LandTick => migrate_ticks.push(usize::MAX),
        _ => {}
    }
    // 对照窗口基点:与被对照场景的迁移拍同拍(对照组不迁移,但统计窗口
    // 对齐,比较才公平)。
    let control_base: usize = match cfg.scene {
        Scene::ControlWalk { phase } => walk0 + phase * period / 12,
        Scene::Control => walk0,
        Scene::ControlLand => 61, // land 实测落地拍(T0/T1 同)
        _ => 0,
    };

    // land 布景(站稳后 → 抬到 0.5 m 自由落体 → 刚接触那拍迁移):
    // tick 60 站姿稳定后,把整只瞬移到 GROUND_TOP+0.5(位置直写根刚体,
    // 速度清零),之后任意一脚 dist<1mm 的首拍 = 落地瞬间。
    // 用迁移组自身的 loop 状态机实现,见 land_stage。
    let mut land_stage = 0usize; // 0=站稳中 1=悬空下落 2=已迁移

    let two_worlds = matches!(cfg.rebuild, Rebuild::Cross | Rebuild::CrossHist);
    let mut a = make_side(cfg.t1);
    let mut b = if two_worlds {
        let mut s = make_side(cfg.t1);
        if cfg.rebuild == Rebuild::CrossHist {
            pollute_history(&mut s.world);
        }
        s
    } else {
        make_side(cfg.t1)
    };
    let mut in_a = true;
    let robot0 = spawn_into(&mut a);
    a.robot = Some(robot0);

    let mut err = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    let mut migrated = false;
    let mut post_base = 0usize;
    let mut migrate_us = 0.0f32;
    let mut land_tick: Option<usize> = None;
    let mut next_ping: Option<usize> = None;
    let mut cur_targets = [0.0f32; N_JOINTS];

    let mut upy_min = 1.0f32;
    let mut max_vy = 0.0f32;
    let mut link_max = 0.0f32;
    let mut pen_max = 0.0f32;
    let mut contact_recover: Option<usize> = None;
    let mut vfwd_sum = 0.0f64;
    let mut vfwd_n = 0u64;
    let mut fell_tick = None;
    let mut nan = false;
    let mut diverge_tick: Option<usize> = None;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut tick_hashes: Vec<u64> = Vec::new();

    for tick in 0..total_ticks {
        // 落地布景(land + 落地对照):站稳(60 拍)→ 抬到 0.5 m 落下;
        // LandTick 在刚接触那拍迁移,ControlLand 不迁移照跑。
        if matches!(cfg.scene, Scene::LandTick | Scene::ControlLand) {
            let (r_world_ref, terrain_ref) = if in_a {
                (&a.world, &a.terrain)
            } else {
                (&b.world, &b.terrain)
            };
            let r_ref = if in_a {
                a.robot.as_ref()
            } else {
                b.robot.as_ref()
            }
            .expect("alive");
            if land_stage == 0 && tick == 60 {
                // 瞬移到 0.5 m 悬空:根刚体位置直写 + 速度清零 + 强唤醒。
                let r_mut = if in_a {
                    a.robot.as_mut()
                } else {
                    b.robot.as_mut()
                }
                .expect("alive");
                let world_mut = if in_a { &mut a.world } else { &mut b.world };
                let y0 = GROUND_TOP + 0.5 + 2.0 * quad::L + quad::FOOT_R;
                if let Some(rb) = world_mut.bodies.get_mut(r_mut.body) {
                    let cur = *rb.rotation();
                    rb.set_position(
                        Pose {
                            translation: Vector::new(10.0, y0, 0.0),
                            rotation: cur,
                            ..Pose::IDENTITY
                        },
                        false,
                    );
                    rb.set_linvel(Vector::ZERO, false);
                    rb.set_angvel(Vector::ZERO, false);
                }
                wake_all(world_mut, r_mut);
                land_stage = 1;
            } else if land_stage == 1 && foot_touching(r_world_ref, r_ref, terrain_ref) {
                land_tick = Some(tick);
                if cfg.scene == Scene::LandTick {
                    migrate_ticks = vec![tick];
                }
                land_stage = 2;
            }
        }
        if tick % 2 == 0 {
            cur_targets = targets_at(&gait, tick, walk0, walking);
        }
        // PingPong 首次排程(站立稳定 60 拍后开始;之后每 gap 拍一次)。
        if let Scene::PingPong { gap } = cfg.scene {
            if next_ping.is_none() && tick >= walk0 + 60 {
                next_ping = Some(tick);
                let _ = gap;
            }
        }
        let do_now = match cfg.scene {
            Scene::PingPong { .. } => next_ping == Some(tick),
            _ => migrate_ticks.first() == Some(&tick) && !migrated,
        };
        if do_now && tick > 0 {
            // 快照(读活跃侧)→ 移除 → 重建(目标侧)。
            let snap = {
                let (src, _dst): (&Side, &mut Side) =
                    if in_a { (&a, &mut b) } else { (&b, &mut a) };
                let r = src.robot.as_ref().expect("alive");
                snapshot_robot(&src.world, r, &cur_targets, cfg.snap == SnapKind::Full)
            };
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            // 目标侧重建(cross / PingPong 落到对侧;same 落回同侧——快照
            // 已完成,先移除再重建安全)。
            if two_worlds || matches!(cfg.scene, Scene::PingPong { .. }) {
                // 顺序:先移除活跃侧,再在对侧重建(借用分离)。
                let old = if in_a { a.robot.take() } else { b.robot.take() }.expect("alive");
                let (src_side, dst_side): (&mut Side, &mut Side) = if in_a {
                    (&mut a, &mut b)
                } else {
                    (&mut b, &mut a)
                };
                remove_robot(src_side, &old);
                let new_robot = restore_robot(
                    &mut dst_side.world,
                    &snap,
                    K,
                    quad::GROUND_FRICTION,
                    DampMode::Scaled,
                );
                migrate_us = t0.elapsed().as_secs_f32() * 1e6;
                err = quad::measure_restore_error(&dst_side.world, &new_robot, &snap);
                dst_side.robot = Some(new_robot);
                in_a = !in_a;
            } else {
                let old = a.robot.take().expect("alive");
                remove_robot(&mut a, &old);
                let new_robot = restore_robot(
                    &mut a.world,
                    &snap,
                    K,
                    quad::GROUND_FRICTION,
                    DampMode::Scaled,
                );
                migrate_us = t0.elapsed().as_secs_f32() * 1e6;
                err = quad::measure_restore_error(&a.world, &new_robot, &snap);
                a.robot = Some(new_robot);
            }
            migrated = true;
            post_base = tick;
            if let Scene::PingPong { gap } = cfg.scene {
                next_ping = Some(tick + gap);
            }
            continue;
        }

        let (active, idle): (&mut Side, &mut Side) = if in_a {
            (&mut a, &mut b)
        } else {
            (&mut b, &mut a)
        };
        let Some(r) = active.robot.as_ref() else {
            break;
        };
        if tick % 2 == 0 {
            write_targets(&mut active.world, r, &cur_targets);
        }
        if cfg.scene == Scene::StandAwake {
            wake_all(&mut active.world, r);
        }
        active.world.step();
        if two_worlds {
            idle.world.step();
        }

        let r = active.robot.as_ref().expect("alive");
        let body = active.world.bodies.get(r.body).expect("alive");
        let p = body.translation();
        if !p.x.is_finite() || !p.y.is_finite() {
            nan = true;
            break;
        }
        let mut th: u64 = 0xcbf2_9ce4_8422_2325;
        for v in [p.x, p.y, p.z] {
            hash = hash_f32(hash, v);
            th = hash_f32(th, v);
        }
        let rot = body.rotation();
        for c in [rot.w, rot.x, rot.y, rot.z] {
            hash = hash_f32(hash, c);
            th = hash_f32(th, c);
        }
        for dof in 0..N_JOINTS {
            let q = read_angle(&active.world.multibody_joints, r, dof);
            hash = hash_f32(hash, q);
            th = hash_f32(th, q);
        }
        let idx = tick_hashes.len();
        tick_hashes.push(th);
        if diverge_tick.is_none() && !ref_hashes.is_empty() {
            if let Some(rh) = ref_hashes.get(idx) {
                if *rh != th {
                    diverge_tick = Some(idx);
                }
            }
        }

        let upy = (rot * Vector::new(0.0, 1.0, 0.0)).y;
        let vy = body.linvel().y.abs();
        let mut tick_link_max = 0.0f32;
        for lh in r.links.iter().copied().chain(std::iter::once(r.body)) {
            let rb = active.world.bodies.get(lh).expect("alive");
            tick_link_max = tick_link_max.max(rb.linvel().length());
        }
        link_max = link_max.max(tick_link_max);
        for pair in active.world.narrow_phase.contact_graph().interactions() {
            if let PairContacts::Rigid(rigid) = &pair.contacts {
                let robot_side =
                    r.colliders.contains(&pair.collider1) || r.colliders.contains(&pair.collider2);
                let ground_side = active.terrain.contains(&pair.collider1)
                    || active.terrain.contains(&pair.collider2);
                if !(robot_side && ground_side) {
                    continue;
                }
                for manifold in &rigid.manifolds {
                    if manifold.points.is_empty() {
                        continue;
                    }
                    let deepest = manifold
                        .points
                        .iter()
                        .map(|pt| pt.dist)
                        .reduce(f32::max)
                        .expect("nonempty");
                    pen_max = pen_max.min(deepest);
                    if contact_recover.is_none()
                        && migrated
                        && tick >= post_base
                        && r.feet
                            .iter()
                            .any(|f| *f == pair.collider1 || *f == pair.collider2)
                        && deepest < 0.001
                    {
                        contact_recover = Some(tick - post_base);
                    }
                }
            }
        }
        if tick >= walk0 {
            let fwd = rot * Vector::new(1.0, 0.0, 0.0);
            let fh = Vector::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
            vfwd_sum += f64::from(body.linvel().dot(fh));
            vfwd_n += 1;
        }
        // 迁移后窗口(判据 2/3);对照组窗口基点 = 对应场景迁移拍
        // (control_base,窗口对齐才可比)。
        let in_window = if migrated {
            tick >= post_base && tick < post_base + POST_TICKS
        } else {
            tick >= control_base && tick < control_base + POST_TICKS
        };
        if in_window {
            upy_min = upy_min.min(upy);
            max_vy = max_vy.max(vy);
            if upy < 0.7 && fell_tick.is_none() {
                fell_tick =
                    Some(tick.saturating_sub(if migrated { post_base } else { control_base }));
            }
        }
        if !migrated && !in_window && upy < 0.7 && tick > walk0 && fell_tick.is_none() {
            fell_tick = Some(tick);
        }
    }

    let vfwd = (vfwd_sum / vfwd_n.max(1) as f64) as f32;
    let outcome = Outcome {
        name: cfg.name.clone(),
        err_coord: err.0,
        err_pos: err.1,
        err_rot: err.2,
        err_vel: err.3,
        post_upy_min: upy_min,
        post_max_vy: max_vy,
        post_link_max: link_max,
        post_pen_max: pen_max,
        contact_recover_tick: contact_recover,
        vfwd,
        fell_tick,
        nan,
        panicked: false,
        panic_msg: String::new(),
        migrate_us,
        diverge_tick,
        land_tick,
        state_hash: hash,
    };
    (outcome, tick_hashes)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("--list") {
        for c in build_matrix() {
            println!("{}", c.name);
        }
        return;
    }
    let det_mode = args.iter().any(|a| a == "det");
    let only: Vec<&String> = args.iter().filter(|a| a.as_str() != "det").collect();
    let cfgs = build_matrix();

    let mut out = String::new();
    out.push_str(
        "name,err_coord_rad,err_pos_m,err_rot_rad,err_vel,post_upy_min,post_max_vy,\
         post_link_max,post_pen_max,contact_recover_tick,vfwd,fell_tick,nan,panicked,\
         panic_msg,migrate_us,diverge_tick,land_tick,state_hash\n",
    );
    let mut det_out = String::new();
    let mut same_store: BTreeMap<String, Vec<u64>> = BTreeMap::new();

    for cfg in &cfgs {
        if cfg.eight {
            // 8 只同拍迁移(耗时组;不放主矩阵循环)。
            if det_mode && !cfg.name.contains("same_full") {
                continue;
            }
            let selected = only.is_empty() || only.iter().any(|a| cfg.name.contains(a.as_str()));
            if !selected {
                continue;
            }
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            let (o, _) = run_eight(cfg);
            let line = outcome_csv(&o);
            println!("{line} | {:.1}s", t0.elapsed().as_secs_f32());
            out.push_str(&line);
            out.push('\n');
            continue;
        }
        let selected = if det_mode {
            cfg.name.contains("walk_p00_same_full_T0")
                || cfg.name.contains("walk_p06_same_full_T0")
                || cfg.name.contains("walk_p06_cross_full_T1")
                || cfg.name.contains("pingpong_g30_same_full_T0")
        } else if only.is_empty() {
            true
        } else {
            only.iter().any(|a| cfg.name.contains(a.as_str()))
        };
        if !selected {
            continue;
        }
        // diverge 基准:cross/cross_hist 取同场景 same_full 存档。
        let is_same_full = cfg.rebuild == Rebuild::Same && cfg.snap == SnapKind::Full;
        let key = scene_key(cfg);
        let mut ref_hashes: Vec<u64> = Vec::new();
        if !is_same_full && cfg.scene != Scene::Control {
            if let (true, Some(k)) = (
                matches!(cfg.rebuild, Rebuild::Cross | Rebuild::CrossHist),
                &key,
            ) {
                ref_hashes = same_store.get(k).cloned().unwrap_or_default();
            }
        }
        #[allow(clippy::disallowed_methods)]
        let t0 = Instant::now();
        let (o, ths) = run(cfg, &ref_hashes);
        if is_same_full {
            if let Some(k) = key {
                same_store.insert(k, ths);
            }
        }
        let line = outcome_csv(&o);
        println!("{line} | {:.1}s", t0.elapsed().as_secs_f32());
        out.push_str(&line);
        out.push('\n');
        if det_mode {
            let (o2, _) = run(cfg, &[]);
            let ok = o.state_hash == o2.state_hash;
            println!(
                "determinism: {} -> {}",
                o.name,
                if ok { "IDENTICAL" } else { "DIFFER" }
            );
            det_out.push_str(&format!(
                "{},{:016x} vs {:016x},{}\n",
                o.name, o.state_hash, o2.state_hash, ok
            ));
        }
    }
    let _ = std::fs::create_dir_all("out");
    let _ = std::fs::write("out/run_summary.csv", &out);
    if det_mode {
        let _ = std::fs::write("out/run_det.txt", &det_out);
    }
}

/// 8 只同拍迁移(same/full,T0):8 只沿 x 排开(间隔 2.5 m),站立后
/// 同时迁移(各自移除重建),报 8 只平均迁移耗时与全体 up·y。
fn run_eight(cfg: &Cfg) -> (Outcome, Vec<u64>) {
    let _ = cfg.snap;
    let gait = default_gait();
    let walk0 = STAND_TICKS;
    let total_ticks = WALK_TICKS;
    let migrate_tick = walk0 + 60; // 行走 1 s 后

    let mut side = make_side(cfg.t1);
    let n = 8;
    let mut robots: Vec<quad::Robot> = Vec::new();
    for i in 0..n {
        let y0 = GROUND_TOP + 2.0 * quad::L + quad::FOOT_R + 0.002;
        let r = spawn_robot(
            &mut side.world,
            Vector::new(10.0 + i as f32 * 2.5, y0, 0.0),
            K,
            quad::GROUND_FRICTION,
            DampMode::Scaled,
        );
        robots.push(r);
    }

    let mut upy_min = 1.0f32;
    let mut max_vy = 0.0f32;
    let mut link_max = 0.0f32;
    let pen_max = 0.0f32;
    let mut vfwd_sum = 0.0f64;
    let mut vfwd_n = 0u64;
    let mut fell_tick = None;
    let mut nan = false;
    let mut migrate_us = 0.0f32;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut cur_targets = [0.0f32; N_JOINTS];

    for tick in 0..total_ticks {
        if tick % 2 == 0 {
            cur_targets = targets_at(&gait, tick, walk0, true);
        }
        if tick % 2 == 0 {
            for r in &robots {
                write_targets(&mut side.world, r, &cur_targets);
            }
        }
        // 同拍迁移:全部 8 只快照 → 移除 → 重建。
        if tick == migrate_tick {
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            let snaps: Vec<_> = robots
                .iter()
                .map(|r| snapshot_robot(&side.world, r, &cur_targets, true))
                .collect();
            for r in &robots {
                remove_robot(&mut side, r);
            }
            robots.clear();
            for snap in &snaps {
                robots.push(restore_robot(
                    &mut side.world,
                    snap,
                    K,
                    quad::GROUND_FRICTION,
                    DampMode::Scaled,
                ));
            }
            migrate_us = t0.elapsed().as_secs_f32() * 1e6 / n as f32;
        }

        side.world.step();

        for (ri, r) in robots.iter().enumerate() {
            let body = side.world.bodies.get(r.body).expect("alive");
            let p = body.translation();
            if !p.x.is_finite() {
                nan = true;
            }
            if ri == 0 {
                for v in [p.x, p.y, p.z] {
                    hash = hash_f32(hash, v);
                }
                let rot = body.rotation();
                for c in [rot.w, rot.x, rot.y, rot.z] {
                    hash = hash_f32(hash, c);
                }
                for dof in 0..N_JOINTS {
                    hash = hash_f32(hash, read_angle(&side.world.multibody_joints, r, dof));
                }
            }
            let rot = body.rotation();
            let upy = (rot * Vector::new(0.0, 1.0, 0.0)).y;
            let vy = body.linvel().y.abs();
            let mut tick_link_max = 0.0f32;
            for lh in r.links.iter().copied().chain(std::iter::once(r.body)) {
                let rb = side.world.bodies.get(lh).expect("alive");
                tick_link_max = tick_link_max.max(rb.linvel().length());
            }
            link_max = link_max.max(tick_link_max);
            if tick >= migrate_tick && tick < migrate_tick + POST_TICKS {
                upy_min = upy_min.min(upy);
                max_vy = max_vy.max(vy);
                if upy < 0.7 && fell_tick.is_none() {
                    fell_tick = Some(tick - migrate_tick);
                }
            }
        }
        if let Some(r0) = robots.first() {
            if tick >= walk0 {
                let body = side.world.bodies.get(r0.body).expect("alive");
                let rot = body.rotation();
                let fwd = rot * Vector::new(1.0, 0.0, 0.0);
                let fh = Vector::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
                vfwd_sum += f64::from(body.linvel().dot(fh));
                vfwd_n += 1;
            }
        }
    }

    let vfwd = (vfwd_sum / vfwd_n.max(1) as f64) as f32;
    let outcome = Outcome {
        name: cfg.name.clone(),
        err_coord: 0.0,
        err_pos: 0.0,
        err_rot: 0.0,
        err_vel: 0.0,
        post_upy_min: upy_min,
        post_max_vy: max_vy,
        post_link_max: link_max,
        post_pen_max: pen_max,
        contact_recover_tick: None,
        vfwd,
        fell_tick,
        nan,
        panicked: false,
        panic_msg: String::new(),
        migrate_us,
        diverge_tick: None,
        land_tick: None,
        state_hash: hash,
    };
    (outcome, Vec::new())
}

/// 全矩阵(放行工况 × 三重建 × 两快照 × 两地形;control / PingPong / Eight
/// 口径见 build_matrix 内注释)。
fn build_matrix() -> Vec<Cfg> {
    let mut all: Vec<Cfg> = Vec::new();
    // 对照(不迁移):站立 control_*、落地 control_land_*、行走
    // control_walk_pXX_*(与迁移组窗口基点对齐;T0/T1)。
    for t1 in [false, true] {
        let t = if t1 { "T1" } else { "T0" };
        all.push(Cfg {
            name: format!("control_{t}"),
            scene: Scene::Control,
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1,
            eight: false,
        });
        all.push(Cfg {
            name: format!("control_land_{t}"),
            scene: Scene::ControlLand,
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1,
            eight: false,
        });
        // 行走对照只跑名义相位 p06(其余相位迁移组与 p06 窗口统计同型,
        // 对照用同一组 gait 频率,单相位对照已给比较基准)。
        all.push(Cfg {
            name: format!("control_walk_p06_{t}"),
            scene: Scene::ControlWalk { phase: 6 },
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1,
            eight: false,
        });
    }
    // 行走中迁移:12 相位 × 三重建 × 两快照 × 两地形 = 144。
    for phase in 0..12usize {
        for rb in [Rebuild::Same, Rebuild::Cross, Rebuild::CrossHist] {
            for sk in [SnapKind::Full, SnapKind::Novec] {
                for t1 in [false, true] {
                    all.push(Cfg {
                        name: format!(
                            "walk_p{:02}_{}_{}_{}",
                            phase,
                            rb.tag(),
                            sk.tag(),
                            if t1 { "T1" } else { "T0" }
                        ),
                        scene: Scene::WalkPhase { phase },
                        rebuild: rb,
                        snap: sk,
                        t1,
                        eight: false,
                    });
                }
            }
        }
    }
    // 醒着站立 / 已睡眠站立 × 三重建 × 两快照 × 两地形 = 24。
    for (scene, base) in [
        (Scene::StandAwake, "stand_awake"),
        (Scene::StandAsleep, "stand_asleep"),
    ] {
        for rb in [Rebuild::Same, Rebuild::Cross, Rebuild::CrossHist] {
            for sk in [SnapKind::Full, SnapKind::Novec] {
                for t1 in [false, true] {
                    all.push(Cfg {
                        name: format!(
                            "{base}_{}_{}_{}",
                            rb.tag(),
                            sk.tag(),
                            if t1 { "T1" } else { "T0" }
                        ),
                        scene,
                        rebuild: rb,
                        snap: sk,
                        t1,
                        eight: false,
                    });
                }
            }
        }
    }
    // 落地瞬间 × 三重建 × 两快照 × 两地形 = 12。
    for rb in [Rebuild::Same, Rebuild::Cross, Rebuild::CrossHist] {
        for sk in [SnapKind::Full, SnapKind::Novec] {
            for t1 in [false, true] {
                all.push(Cfg {
                    name: format!(
                        "land_{}_{}_{}",
                        rb.tag(),
                        sk.tag(),
                        if t1 { "T1" } else { "T0" }
                    ),
                    scene: Scene::LandTick,
                    rebuild: rb,
                    snap: sk,
                    t1,
                    eight: false,
                });
            }
        }
    }
    // 连续来回迁移:间隔 2、5、30、120(same/full,T0);cross/cross_hist
    // 只跑 g30 一组对照。
    for gap in [2usize, 5, 30, 120] {
        all.push(Cfg {
            name: format!("pingpong_g{gap}_same_full_T0"),
            scene: Scene::PingPong { gap },
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1: false,
            eight: false,
        });
    }
    for rb in [Rebuild::Cross, Rebuild::CrossHist] {
        all.push(Cfg {
            name: format!("pingpong_g30_{}_full_T0", rb.tag()),
            scene: Scene::PingPong { gap: 30 },
            rebuild: rb,
            snap: SnapKind::Full,
            t1: false,
            eight: false,
        });
    }
    // 8 只同拍迁移(same/full,T0)。
    all.push(Cfg {
        name: "eight_same_full_T0".into(),
        scene: Scene::Eight,
        rebuild: Rebuild::Same,
        snap: SnapKind::Full,
        t1: false,
        eight: true,
    });
    all
}
