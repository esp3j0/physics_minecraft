//! REQ-012 A00-3 demo `migrate`(2026-11-07,R1 返修)。
//!
//! 问题(A00-3 放行原文):丢掉接触缓存重建后,四足会不会跳变或摔倒?
//! 要验证的设计:迁移信封只带身体哈希、根状态、关节坐标、关节速度、最后的
//! 目标,整个主体同一拍迁移。
//!
//! R1 修正(A00-3 验收 M1–M10):
//! - M1 `restore_robot` 写回 `snap.targets`(quad 侧修,重建后下一步进前
//!   电机在位);
//! - M2 迁移拍照常写目标、照常步进(不再 `continue`);
//! - M3/M4 `post_*` 只统计迁移/唤醒后 POST_TICKS 窗口(不再混入出生段,
//!   对照窗口基点 = 同相位迁移拍);
//! - M5 摔倒从迁移拍到运行结束全程检查;
//! - M6 新增 `post_body_step_max`(窗口内机身相邻拍位移最大值);
//! - M7 行走对照 12 相位各一条(T0/T1),站立对照强制醒着/不强制各一条;
//! - M8 pingpong 对照同 3600 拍;
//! - M9 `diverge_tick` 与相同快照类型的 `same` 行比;
//! - M10 8 只组逐只报数(不写死 0)。
//!
//! 新增组(R1-5/6/7):`cross_hist` 拆开——只有句柄历史(histonly)/只有
//! 方块堆(堆已睡着 pilesleep / 堆醒着 pileawake);`order_*` 连杆顺序表
//! (主体直接出生不迁移,在主体前/后插入刚体,报活跃集顺序);迁移后顺序
//! 变化(`seq_*`:先 4 方块打乱顺序 → 行走 300 拍 → same 迁移 → 再走
//! 1020 拍,报迁移前后顺序与前后段 vfwd/航向速率)。
//!
//! 重建方式三种(都跑):`same` = 同一世界内移除再重建;`cross` = 移到第二个
//! 世界(地形相同,两世界同拍步进);`cross_hist` = cross + 目标世界历史
//! (R1 拆分见上)。
//!
//! 状态快照两种(都跑):`full` / `novec`(速度清零)。
//!
//! 统一口径(A00-3 放行):60 Hz、生产求解参数(只覆写 dt)、默认步态
//! (kp 100、kd 3、3 Hz)、k = 1/100、阻尼按 k 缩放、30 Hz 更新目标、
//! 不开 extra。
//!
//! `cross` / `cross_hist` 的轨迹分歧(diverge_tick,与相同快照类型的
//! `same` 行比)只报数,不是判据。
//!
//! R2 修正(A00-3 R1 验收 MR1–MR6):
//! - MR2 `active_order_end` 读主体最后所在的世界;新增 `active_order_post`
//!   (迁移后第 120 拍、主体所在世界的顺序);
//! - MR3 新增 `dst_awake_at_migrate`(迁移拍目标世界醒着的非主体动态刚体
//!   数;同世界写 -);`pilesleep` 迁移门槛 = 堆全部睡着且连续 60 拍保持
//!   全睡,才在下一个对应相位的拍迁移,不为 0 panic;新增
//!   `dst_rewake_tick`(迁移后堆重新醒来的拍);
//! - MR4 去重复行(拆分家族只跑 full);
//! - MR5 窗口只收迁移后(对照行用自己的基点,不再把迁移前拍混进同一统计;
//!   order 行无迁移窗口写 -);
//! - MR6 新增 `yaw_change_deg`(全程累计航向变化,展开 ±180°)、
//!   `path_len_m`(机身水平路径长度)、`slept`(主体是否睡过);新增
//!   `seq_control_T0`(seq 前后两段同时长的干净世界对照)。

use std::collections::BTreeMap;
use std::time::Instant;

use physics_sim::collider_build::{greedy_colliders, BoxCollider, NeighborChunks};
use quad::{
    active_order, default_gait, ik, production_params, read_angle, restore_robot, set_target,
    snapshot_robot, spawn_robot, DampMode, GROUND_RESTITUTION, N_JOINTS, TAU_LIMIT,
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
/// pilesleep 迁移门槛:堆全部睡着且连续保持这么多拍(MR3)。
const PILE_ALL_ASLEEP_STREAK: usize = 60;

/// 重建方式。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Rebuild {
    /// 同一世界内移除再重建。
    Same,
    /// 移到第二个世界(地形相同,两世界同拍步进)。
    Cross,
    /// cross + 目标世界完整历史(句柄历史 + 20 m 外 3×3×3 堆;R1 之前口径)。
    CrossHist,
    /// cross + 只有句柄历史(R1-5 拆分:200 个刚体插入再删除,无方块堆)。
    HistOnly,
    /// cross + 只有方块堆,且迁移拍堆已全部睡着(R1-5 拆分)。
    PileSleep,
    /// cross + 只有方块堆,迁移拍堆还有醒着的(每拍 wake_up;
    /// R1-5 拆分:对照验收"目标世界有堆且迁移拍已睡着时与干净世界逐位一致
    /// 到显示精度")。
    PileAwake,
}

impl Rebuild {
    fn tag(self) -> &'static str {
        match self {
            Rebuild::Same => "same",
            Rebuild::Cross => "cross",
            Rebuild::CrossHist => "cross_hist",
            Rebuild::HistOnly => "histonly",
            Rebuild::PileSleep => "pilesleep",
            Rebuild::PileAwake => "pileawake",
        }
    }
    fn is_cross(self) -> bool {
        !matches!(self, Rebuild::Same)
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
    /// 8 只同拍迁移(same/full,T0)。
    Eight,
    /// 连杆顺序表(R1-6):主体直接出生,不迁移,180+1320 拍行走,
    /// 场景按 Cfg.order 配置布景。
    Order,
    /// 迁移后顺序变化(R1-7):4 方块打乱顺序 → 行走 300 拍 → same 迁移
    /// → 再走 1020 拍。
    SeqShuffle,
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
    /// Order 场景布景:在主体之前插入的刚体数量(方块/固定/无碰撞体/睡着)。
    order: Option<OrderSetup>,
    /// 对照行(不迁移;窗口基点与被对照场景的迁移拍对齐)。
    control_like: bool,
}

/// Order 场景的布景描述(R1-6)。
#[derive(Clone, Copy)]
struct OrderSetup {
    /// 插入刚体数量。
    n: usize,
    /// 插入位置:主体之前 / 之后。
    before: bool,
    /// 类型:动态方块(会睡着)/ 固定刚体 / 无碰撞体刚体 / 出生就睡着。
    kind: OrderKind,
    /// 方块堆摆放中心 x 偏移(±200 m 远置组用)。
    dx: f32,
}

#[derive(Clone, Copy, PartialEq)]
enum OrderKind {
    Block,
    Fixed,
    NoCollider,
    Asleep,
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

/// cross_hist 的完整历史污染:插入、删除 200 个刚体(句柄历史)。
fn pollute_handles(world: &mut PhysicsWorld) {
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
}

/// 20 m 外 3×3×3 无关方块堆(mass Isolation 同构)。
fn spawn_pile(world: &mut PhysicsWorld) -> Vec<RigidBodyHandle> {
    let mut out = Vec::new();
    for ix in 0..3i32 {
        for iy in 0..3i32 {
            for iz in 0..3i32 {
                let pos = Vector::new(
                    30.0 + ix as f32,
                    GROUND_TOP + 0.5 + 1.0 + iy as f32,
                    iz as f32,
                );
                let (h, _) = world.insert(
                    RigidBodyBuilder::dynamic().translation(pos),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5)
                        .density(1.0)
                        .friction(0.6)
                        .restitution(0.1),
                );
                out.push(h);
            }
        }
    }
    out
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

fn spawn_into(side: &mut Side, x: f32) -> quad::Robot {
    let y0 = GROUND_TOP + 2.0 * quad::L + quad::FOOT_R + 0.002;
    spawn_robot(
        &mut side.world,
        Vector::new(x, y0, 0.0),
        K,
        quad::GROUND_FRICTION,
        DampMode::Scaled,
    )
}

/// 步态目标(站立或行走 tick 的 12 关节目标;与 terrain 同口径)。
/// `walking = false` 的场景(站立/睡眠/落地)全程保持站姿目标。
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

/// 对地穿透最深值(全部连杆对地形;流形非空)。
fn ground_pen(world: &PhysicsWorld, r: &quad::Robot, terrain: &[ColliderHandle]) -> f32 {
    let mut pen = 0.0f32;
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
                if let Some(deepest) = manifold.points.iter().map(|p| p.dist).reduce(f32::max) {
                    pen = pen.min(deepest);
                }
            }
        }
    }
    pen
}

struct Outcome {
    name: String,
    /// 重建后、步进前:(关节坐标 rad,连杆位置 m,连杆转角 rad,速度)。
    err_coord: f32,
    err_pos: f32,
    err_rot: f32,
    err_vel: f32,
    /// 迁移/唤醒后 POST_TICKS 真实窗口(M3/M4:只含窗口内拍)。
    post_upy_min: f32,
    post_max_vy: f32,
    post_link_max: f32,
    post_pen_max: f32,
    /// M6:窗口内机身相邻拍位移最大值(m)。
    post_body_step_max: f32,
    /// 迁移后首次真实脚-地接触(dist<1mm)相对迁移拍。
    contact_recover_tick: Option<usize>,
    /// 全程 vfwd(行走段均值,m/s;对照同窗口)。
    vfwd: f32,
    /// M5:从迁移/唤醒拍起全程检查的摔倒拍(相对迁移拍)。
    fell_tick: Option<usize>,
    nan: bool,
    /// CSV 列占位(与 freeze 列序对齐;migrate 的重建路径无 panic 源,
    /// 恒 false,不写入文件)。
    #[allow(dead_code)]
    panicked: bool,
    panic_msg: String,
    /// 单次迁移耗时(µs;restore_robot 调用,Eight 组报 8 只平均)。
    migrate_us: f32,
    /// 与相同快照类型的 same 行逐拍哈希的首个分歧拍(M9;None = 无)。
    diverge_tick: Option<usize>,
    /// 落地组:实际迁移拍。
    land_tick: Option<usize>,
    /// R1-7:迁移前/后的活跃集顺序与前后段 vfwd、航向速率(SeqShuffle 组)。
    seq_pre_order: String,
    seq_post_order: String,
    /// R1 通用:迁移/起跑前与运行结束的活跃集顺序(全部行;MR2:end 读
    /// 主体最后所在的世界)。
    active_order_start: String,
    active_order_end: String,
    /// MR2:迁移后第 120 拍、主体所在世界的活跃集顺序(非迁移行 = -)。
    active_order_post: String,
    /// MR3:迁移拍目标世界里醒着的非主体动态刚体数(同世界行 = -)。
    dst_awake_at_migrate: Option<usize>,
    /// MR3:迁移后目标世界堆重新醒来的拍(相对迁移拍;未复醒 = -)。
    dst_rewake_tick: Option<usize>,
    seq_pre_vfwd: f32,
    seq_post_vfwd: f32,
    /// 航向速率(度/s;pre/post 段)。
    seq_pre_yaw_rate: f32,
    seq_post_yaw_rate: f32,
    /// MR6:全程累计航向变化(度,展开过 ±180°)。
    yaw_change_deg: f32,
    /// MR6:机身水平路径长度(m)。
    path_len_m: f32,
    /// MR6:主体是否睡过(全程)。
    slept: bool,
    state_hash: u64,
}

fn outcome_csv(o: &Outcome) -> String {
    let fnum = |v: f32| {
        if v.is_finite() {
            format!("{v:.4}")
        } else {
            "inf".to_string()
        }
    };
    let num_or = |v: Option<usize>| v.map_or_else(|| "-".to_string(), |t| t.to_string());
    format!(
        "{},{:.3e},{:.3e},{:.3e},{:.3e},{},{},{},{},{},{},{},{},{},false,{},{:.1},{},{},{},{},{},{},{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{:016x}",
        o.name,
        o.err_coord,
        o.err_pos,
        o.err_rot,
        o.err_vel,
        fnum(o.post_upy_min),
        fnum(o.post_max_vy),
        fnum(o.post_link_max),
        fnum(o.post_pen_max),
        fnum(o.post_body_step_max),
        o.contact_recover_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        fnum(o.vfwd),
        o.fell_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.nan,
        o.panic_msg,
        o.migrate_us,
        o.diverge_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.land_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.seq_pre_order,
        o.seq_post_order,
        o.active_order_start,
        o.active_order_end,
        o.active_order_post,
        num_or(o.dst_awake_at_migrate),
        num_or(o.dst_rewake_tick),
        fnum(o.seq_pre_vfwd),
        fnum(o.seq_post_vfwd),
        fnum(o.seq_pre_yaw_rate),
        fnum(o.seq_post_yaw_rate),
        fnum(o.yaw_change_deg),
        fnum(o.path_len_m),
        o.slept,
        o.state_hash,
    )
}

/// 场景键(diverge 存档;same 行产出,cross 家族行消费;M9:按快照类型)。
fn scene_key(cfg: &Cfg) -> Option<String> {
    let t = if cfg.t1 { "T1" } else { "T0" };
    let sk = cfg.snap.tag();
    match cfg.scene {
        Scene::WalkPhase { phase } => Some(format!("walk_p{phase:02}_{t}_{sk}")),
        Scene::StandAwake => Some(format!("stand_awake_{t}_{sk}")),
        Scene::StandAsleep => Some(format!("stand_asleep_{t}_{sk}")),
        Scene::LandTick => Some(format!("land_{t}_{sk}")),
        _ => None,
    }
}

/// 窗口内统计累积器(M3/M4:只收窗口内拍)。
#[derive(Default)]
struct WinStats {
    upy_min: f32,
    max_vy: f32,
    link_max: f32,
    pen_max: f32,
    body_step_max: f32,
    prev_body: Option<Vector>,
}

impl WinStats {
    fn new() -> Self {
        WinStats {
            upy_min: 1.0,
            max_vy: 0.0,
            link_max: 0.0,
            pen_max: 0.0,
            body_step_max: 0.0,
            prev_body: None,
        }
    }
    fn push(&mut self, upy: f32, vy: f32, link_max_tick: f32, pen: f32, body_pos: Vector) {
        self.upy_min = self.upy_min.min(upy);
        self.max_vy = self.max_vy.max(vy);
        self.link_max = self.link_max.max(link_max_tick);
        self.pen_max = self.pen_max.min(pen);
        if let Some(prev) = self.prev_body {
            self.body_step_max = self.body_step_max.max((body_pos - prev).length());
        }
        self.prev_body = Some(body_pos);
    }
}

fn run(cfg: &Cfg, ref_hashes: &[u64]) -> (Outcome, Vec<u64>) {
    let gait = default_gait();
    let period = (60.0 / gait.freq).round() as usize;
    let walk0 = STAND_TICKS;
    // 行走口径只属于 WalkPhase / PingPong / SeqShuffle / Order;站立、睡眠、
    // 落地全程站姿。
    let walking = matches!(
        cfg.scene,
        Scene::WalkPhase { .. } | Scene::PingPong { .. } | Scene::SeqShuffle | Scene::Order
    );
    let total_ticks = match cfg.scene {
        Scene::PingPong { .. } => PINGPONG_TICKS,
        Scene::Order => WALK_TICKS,
        Scene::SeqShuffle => STAND_TICKS + 300 + 1020,
        _ => WALK_TICKS,
    };

    // MR3(pilesleep):迁移拍推迟到"堆全部睡着且连续 60 拍保持全睡"之后的
    // 第一个对应相位拍。名义迁移拍 = migrate_ticks[0](Deferred 由状态机覆写)。
    let mut migrate_ticks: Vec<usize> = Vec::new();
    match cfg.scene {
        Scene::WalkPhase { phase } => migrate_ticks.push(walk0 + phase * period / 12),
        Scene::StandAwake => migrate_ticks.push(walk0),
        Scene::StandAsleep => migrate_ticks.push(walk0 + 600),
        Scene::LandTick => migrate_ticks.push(usize::MAX),
        Scene::SeqShuffle => migrate_ticks.push(walk0 + 300),
        _ => {}
    }

    // land 布景(站稳后 → 抬到 0.5 m 自由落体 → 刚接触那拍迁移):
    let mut land_stage = 0usize; // 0=站稳中 1=悬空下落 2=已迁移/落地

    let two_worlds = cfg.rebuild.is_cross();
    let mut a = make_side(cfg.t1);
    let mut b = if two_worlds {
        let mut s = make_side(cfg.t1);
        match cfg.rebuild {
            Rebuild::CrossHist => {
                pollute_handles(&mut s.world);
                spawn_pile(&mut s.world);
            }
            Rebuild::HistOnly => pollute_handles(&mut s.world),
            Rebuild::PileSleep | Rebuild::PileAwake => {
                spawn_pile(&mut s.world);
            }
            _ => {}
        }
        s
    } else {
        make_side(cfg.t1)
    };
    // Order 布景(R1-6):在主体之前/之后**插入刚体**(句柄序;机制见
    // #25 总架构师验收:睡着时 island 用 swap_remove 移出活跃集,主体之前
    // 插入的刚体会把主体连杆搬到活跃集前面)。"之前插入" = 先于机器人生成
    // (句柄更小);"之后插入" = 晚于机器人生成。刚体放在行走带侧旁
    // (z 偏移 ±2.5 m 起),不挡路——被测变量是插入顺序,不是碰撞。
    if let Some(os) = cfg.order.filter(|o| o.before) {
        spawn_order_bodies(&mut a.world, os);
    }
    let mut in_a = true;
    let robot0 = spawn_into(&mut a, 10.0);
    a.robot = Some(robot0);
    if let Some(os) = cfg.order.filter(|o| !o.before) {
        spawn_order_bodies(&mut a.world, os);
    }

    // PileAwake 组:目标世界的方块堆迁移拍前每拍唤醒(醒着的堆参与求解,
    // PileSleep 对照堆睡着;唤醒窗口见主循环 pile_wake_now)。
    let pile_awake = cfg.rebuild == Rebuild::PileAwake;
    // MR3(pilesleep):全部睡着且连续 PILE_ALL_ASLEEP_STREAK 拍 → 允许迁移;
    // 名义拍在门槛满足后的第一个"相位对齐"拍执行。
    let pile_sleep_gate = cfg.rebuild == Rebuild::PileSleep;
    let mut pile_asleep_streak = 0usize;
    // MR3:目标世界堆复醒检测(迁移后第几拍;pileawake 组也报)。
    let mut dst_rewake_tick: Option<usize> = None;
    let mut err = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    let mut migrated = false;
    let mut post_base = 0usize;
    // MR2:迁移后第 POST_TICKS 拍采样的 active_order_post。
    let mut order_post = String::new();
    let mut order_post_done = false;
    // MR3:迁移拍目标世界醒着的非主体动态刚体数。
    let mut dst_awake_at_migrate: Option<usize> = None;
    // R1 通用要求:每行报 active_order_start(迁移/起跑前)与
    // active_order_end(运行结束时;MR2 = 主体最后所在的世界)。
    let mut order_start = String::new();
    let mut order_end = String::new();
    let mut migrate_us = 0.0f32;
    let mut land_tick: Option<usize> = None;
    let mut next_ping: Option<usize> = None;
    let mut cur_targets = [0.0f32; N_JOINTS];

    let mut win = WinStats::new();
    let mut contact_recover: Option<usize> = None;
    let mut vfwd_sum = 0.0f64;
    let mut vfwd_n = 0u64;
    // M5:摔倒从迁移/唤醒拍起全程检查(含窗口外)。
    let mut fell_tick = None;
    let mut nan = false;
    let mut diverge_tick: Option<usize> = None;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut tick_hashes: Vec<u64> = Vec::new();
    // R1-7 SeqShuffle:迁移前后段 vfwd/航向速率与顺序。
    let mut seq_pre_order = String::new();
    let mut seq_post_order = String::new();
    let mut seq_pre_vfwd = 0.0f64;
    let mut seq_pre_n = 0u64;
    let mut seq_post_vfwd = 0.0f64;
    let mut seq_post_n = 0u64;
    let mut seq_pre_yaw0: Option<f32> = None;
    let mut seq_pre_yaw1: Option<f32> = None;
    let mut seq_post_yaw0: Option<f32> = None;
    let mut seq_post_yaw1: Option<f32> = None;
    let seq_pre_span = (walk0, walk0 + 300);
    let seq_post_span = (walk0 + 300, total_ticks);
    // MR6:全程航向变化(展开)与水平路径长度、睡眠。
    let mut yaw_prev: Option<f32> = None;
    let mut yaw_total = 0.0f32;
    let mut path_len = 0.0f32;
    let mut body_prev: Option<Vector> = None;
    let mut slept = false;

    for tick in 0..total_ticks {
        // 落地布景(land):站稳(60 拍)→ 抬到 0.5 m 落下;在刚接触那拍迁移。
        if matches!(cfg.scene, Scene::LandTick) {
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
                migrate_ticks = vec![tick];
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
        // MR3(pilesleep):迁移拍 = 堆全睡连续 60 拍后的第一个对应相位拍。
        // 名义相位拍集合 = walk0 + k·period/12(与该组 phase 对齐的拍 =
        // (tick - 名义拍) % (period/12) == 0)。
        let do_now = match cfg.scene {
            Scene::PingPong { .. } => next_ping == Some(tick),
            _ => {
                !cfg.control_like
                    && migrate_ticks.first() == Some(&tick)
                    && !migrated
                    && pile_gate_ok(pile_sleep_gate, pile_asleep_streak)
            }
        };
        // pilesleep 门槛未满足时把迁移拍顺延一个相位格(period/12),保持
        // 相位对齐(状态机在下面 pile_streak 更新处推进)。
        if pile_sleep_gate && !migrated && migrate_ticks.first() == Some(&tick) && !do_now {
            let phase_step = period / 12;
            migrate_ticks[0] = tick + phase_step;
        }
        if do_now && tick > 0 {
            // 快照(读活跃侧)→ 移除 → 重建(目标侧)。迁移拍照常写目标、
            // 照常步进(M2:不再 continue)。
            let snap = {
                let src = if in_a { &a } else { &b };
                let r = src.robot.as_ref().expect("alive");
                snapshot_robot(&src.world, r, &cur_targets, cfg.snap == SnapKind::Full)
            };
            // MR3:迁移拍目标世界里醒着的非主体动态刚体数(同世界 = None)。
            // pilewasawake 用于复醒列语义(pileawake 组迁移前堆醒着是布景预期)。
            if two_worlds {
                let dst = if in_a { &b } else { &a };
                let n = count_awake_dynamic_excluding(dst, &[]);
                dst_awake_at_migrate = Some(n);
                if cfg.rebuild == Rebuild::PileSleep && n != 0 {
                    panic!(
                        "pilesleep gate violated at migrate: {} awake bodies in dst (name={})",
                        n, cfg.name
                    );
                }
            }
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            if two_worlds {
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
        }
        // R1-7:迁移前的顺序(每拍更新,取迁移前最后一拍值)。
        if cfg.scene == Scene::SeqShuffle && !migrated && tick >= walk0 {
            if let Some(r) = a.robot.as_ref() {
                seq_pre_order = active_order(&a.world, r);
            }
        }
        // R1 通用:迁移/起跑前的活跃集顺序(active_order_start;起步第一拍
        // 捕获,站立平衡后。行走组已写入目标、世界未步进 = 本拍求解前的顺序;
        // SeqShuffle 用 seq_pre_order,不在此时覆盖)。
        if order_start.is_empty()
            && tick == walk0
            && cfg.scene != Scene::SeqShuffle
            && !cfg.control_like
        {
            if let Some(r) = a.robot.as_ref() {
                order_start = active_order(&a.world, r);
            }
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
        // PileAwake:迁移拍之前目标世界的无关刚体(方块堆)保持醒着
        // (R1-5 对照口径;机器人只存在于活跃侧,目标世界无机器人)。
        let pile_wake_now = pile_awake && !migrated && tick <= post_base_min(&migrate_ticks, walk0);
        active.world.step();
        if two_worlds {
            if pile_wake_now {
                for (_, rb) in idle.world.bodies.iter_mut() {
                    rb.wake_up(true);
                }
            }
            idle.world.step();
        }

        let r = active.robot.as_ref().expect("alive");
        let body = active.world.bodies.get(r.body).expect("alive");
        let p = body.translation();
        if !p.x.is_finite() || !p.y.is_finite() {
            nan = true;
            break;
        }
        // MR6:航向累计(展开 ±180°)与水平路径。
        let rot0 = body.rotation();
        let yaw = yaw_of(rot0);
        if let Some(prev) = yaw_prev {
            let d = (yaw - prev).rem_euclid(std::f32::consts::TAU);
            let d = if d > std::f32::consts::PI {
                d - std::f32::consts::TAU
            } else {
                d
            };
            yaw_total += d;
        }
        yaw_prev = Some(yaw);
        if let Some(prev) = body_prev {
            let d = p - prev;
            path_len += (d.x * d.x + d.z * d.z).sqrt();
        }
        body_prev = Some(p);
        // MR6:slept = 主体任一连杆睡过(含机身)。
        if !slept {
            slept = active
                .world
                .bodies
                .get(r.body)
                .map(|b| b.is_sleeping())
                .unwrap_or(false)
                || r.links.iter().any(|lh| {
                    active
                        .world
                        .bodies
                        .get(*lh)
                        .map(|b| b.is_sleeping())
                        .unwrap_or(false)
                });
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
        let pen = ground_pen(&active.world, r, &active.terrain);
        // M3/M4 + MR5:窗口 = 迁移/对照基点后 POST_TICKS 拍,只统计窗口内。
        // 对照行用自己的基点(不与迁移行混统计);迁移前行不进窗口。
        let in_window = if migrated {
            tick >= post_base && tick < post_base + POST_TICKS
        } else {
            !cfg.control_like && cfg.scene == Scene::Eight
                || control_like_window(cfg, tick, period, walk0, land_tick)
        };
        if in_window {
            win.push(upy, vy, tick_link_max, pen, p);
        }
        // MR2:迁移后第 POST_TICKS 拍(窗口末拍)、step 后采样目标世界顺序。
        if migrated && !order_post_done && two_worlds && tick == post_base + POST_TICKS {
            if let Some(rr) = active.robot.as_ref() {
                order_post = active_order(&active.world, rr);
                order_post_done = true;
            }
        }
        // 接触恢复(迁移组):迁移后首次真实脚-地接触。
        if contact_recover.is_none()
            && migrated
            && tick >= post_base
            && foot_touching(&active.world, r, &active.terrain)
        {
            contact_recover = Some(tick - post_base);
        }
        // M5:摔倒从迁移/对照基点起全程检查(land 组窗口覆盖落地冲击本身,
        // up·y 低是物理结果,不判摔、只报窗口指标)。
        let fell_base = if migrated {
            post_base
        } else {
            control_base_of(cfg, period, walk0, land_tick)
        };
        let fell_watch = !matches!(cfg.scene, Scene::LandTick);
        if fell_watch && tick >= fell_base && upy < 0.7 && fell_tick.is_none() {
            fell_tick = Some(tick - fell_base);
        }
        if tick >= walk0 {
            let fwd = rot * Vector::new(1.0, 0.0, 0.0);
            let fh = Vector::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
            let v_fwd = body.linvel().dot(fh);
            vfwd_sum += f64::from(v_fwd);
            vfwd_n += 1;
            if cfg.scene == Scene::SeqShuffle {
                let yaw = yaw_of(rot);
                if tick >= seq_pre_span.0 && tick < seq_pre_span.1 {
                    seq_pre_vfwd += f64::from(v_fwd);
                    seq_pre_n += 1;
                    if seq_pre_yaw0.is_none() {
                        seq_pre_yaw0 = Some(yaw);
                    }
                    seq_pre_yaw1 = Some(yaw);
                }
                if tick >= seq_post_span.0 && tick < seq_post_span.1 {
                    seq_post_vfwd += f64::from(v_fwd);
                    seq_post_n += 1;
                    if seq_post_yaw0.is_none() {
                        seq_post_yaw0 = Some(yaw);
                    }
                    seq_post_yaw1 = Some(yaw);
                }
                if tick == seq_post_span.0 {
                    if let Some(rr) = active.robot.as_ref() {
                        seq_post_order = active_order(&active.world, rr);
                    }
                }
            }
        }
        // MR3(pilesleep 状态机):步进后统计堆睡眠连拍;名义迁移拍到达但
        // 门槛未满足时,do_now 已把 migrate_ticks[0] 顺延一个相位格。
        if pile_sleep_gate && !migrated && pile_all_asleep(idle_world(in_a, &a, &b)) {
            pile_asleep_streak += 1;
        } else if pile_sleep_gate && !migrated {
            pile_asleep_streak = 0;
        }
        // MR3:堆复醒检测(迁移后第几拍;pileawake/pilesleep/cross_hist 都
        // 有堆。堆在**主体迁移后的目标世界** = 活跃侧;判定 = 出现醒着的
        // 动态刚体(排除主体自身),记第一拍。放在循环末,避开 active
        // 可变借用区)。
        if migrated
            && dst_rewake_tick.is_none()
            && matches!(
                cfg.rebuild,
                Rebuild::PileSleep | Rebuild::PileAwake | Rebuild::CrossHist
            )
        {
            let robot_handles: Vec<RigidBodyHandle> = active_robot_handles(in_a, &a, &b);
            if pile_has_awake(active_world(in_a, &a, &b), &robot_handles) {
                dst_rewake_tick = Some(tick - post_base);
            }
        }
    }

    // R1 通用 + MR2:运行结束时的活跃集顺序(active_order_end 读主体
    // 最后所在的世界;跨世界迁移后主体在 b)。
    let final_robot = a.robot.as_ref().or(b.robot.as_ref());
    if let Some(r) = final_robot {
        let world = if a.robot.is_some() {
            &a.world
        } else {
            &b.world
        };
        order_end = active_order(world, r);
    }
    // MR2:order_post 没采到(总拍数 < post_base+120、非迁移行、同世界行)
    // 一律写 -。
    if !order_post_done {
        order_post = "-".to_string();
    }

    let vfwd = (vfwd_sum / vfwd_n.max(1) as f64) as f32;
    let dt = 1.0 / 60.0;
    let (seq_pre_rate, seq_post_rate) = (
        yaw_rate(seq_pre_yaw0, seq_pre_yaw1, seq_pre_n as f32 * dt),
        yaw_rate(seq_post_yaw0, seq_post_yaw1, seq_post_n as f32 * dt),
    );
    let outcome = Outcome {
        name: cfg.name.clone(),
        err_coord: err.0,
        err_pos: err.1,
        err_rot: err.2,
        err_vel: err.3,
        post_upy_min: win.upy_min,
        post_max_vy: win.max_vy,
        post_link_max: win.link_max,
        post_pen_max: win.pen_max,
        post_body_step_max: win.body_step_max,
        contact_recover_tick: contact_recover,
        vfwd,
        fell_tick,
        nan,
        panicked: false, // migrate 的重建路径无 panic 源(全连杆切型属 freeze)

        panic_msg: String::new(),
        migrate_us,
        diverge_tick,
        land_tick,
        seq_pre_order,
        seq_post_order,
        active_order_start: order_start,
        active_order_end: order_end,
        active_order_post: order_post,
        dst_awake_at_migrate,
        dst_rewake_tick,
        seq_pre_vfwd: (seq_pre_vfwd / seq_pre_n.max(1) as f64) as f32,
        seq_post_vfwd: (seq_post_vfwd / seq_post_n.max(1) as f64) as f32,
        seq_pre_yaw_rate: seq_pre_rate,
        seq_post_yaw_rate: seq_post_rate,
        yaw_change_deg: yaw_total * 180.0 / std::f32::consts::PI,
        path_len_m: path_len,
        slept,
        state_hash: hash,
    };
    (outcome, tick_hashes)
}

/// PileAwake 的唤醒窗口上界:迁移拍(单次迁移场景)。
fn post_base_min(migrate_ticks: &[usize], walk0: usize) -> usize {
    migrate_ticks.first().copied().unwrap_or(walk0)
}

/// R1-6 Order 布景:按 OrderSetup 在世界里插入刚体(调用方控制先于/晚于
/// 机器人生成,实现"在主体之前/之后插入")。刚体摆在行走带侧旁(z 从
/// ±2.5 m 起每 1.2 m 一个),不挡路;远置组(±200 m)按 os.dx 平移。
fn spawn_order_bodies(world: &mut PhysicsWorld, os: OrderSetup) {
    let z_sign: f32 = if os.before { 2.5 } else { -2.5 };
    for i in 0..os.n {
        let x = os.dx;
        let z = z_sign + i as f32 * 1.2;
        match os.kind {
            OrderKind::Block => {
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(x, GROUND_TOP + 0.5, z)),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5)
                        .density(1.0)
                        .friction(0.6)
                        .restitution(0.1),
                );
            }
            OrderKind::Fixed => {
                world.insert(
                    RigidBodyBuilder::fixed().translation(Vector::new(x, GROUND_TOP + 0.5, z)),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5),
                );
            }
            OrderKind::NoCollider => {
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(x, GROUND_TOP + 3.0, z)),
                    ColliderBuilder::ball(0.01),
                );
            }
            OrderKind::Asleep => {
                world.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(x, GROUND_TOP + 0.5, z))
                        .sleeping(true),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5)
                        .density(1.0)
                        .friction(0.6)
                        .restitution(0.1),
                );
            }
        }
    }
}

fn yaw_rate(y0: Option<f32>, y1: Option<f32>, secs: f32) -> f32 {
    match (y0, y1) {
        (Some(a), Some(b)) if secs > 0.0 => {
            let d = (b - a).rem_euclid(std::f32::consts::TAU);
            let d = if d > std::f32::consts::PI {
                d - std::f32::consts::TAU
            } else {
                d
            };
            d / secs * 180.0 / std::f32::consts::PI
        }
        _ => 0.0,
    }
}

fn yaw_of(rot: &rapier3d::math::Rotation) -> f32 {
    // 绕世界 Y 的欧拉角(机身出生只绕 Y 旋转 + 行走小倾角,取投影角)。
    let fwd = *rot * Vector::new(1.0, 0.0, 0.0);
    fwd.z.atan2(fwd.x)
}

/// 对照组的窗口基点(M3/M7:与被对照场景的迁移拍同拍对齐)。
fn control_base_of(cfg: &Cfg, period: usize, walk0: usize, land_tick: Option<usize>) -> usize {
    match cfg.scene {
        Scene::WalkPhase { phase } => walk0 + phase * period / 12,
        Scene::StandAwake => walk0,
        Scene::StandAsleep => walk0 + 600,
        Scene::LandTick => land_tick.unwrap_or(61),
        Scene::PingPong { gap } => {
            let _ = gap;
            walk0 + 60
        }
        // MR5:order / seq 行没有对齐的迁移窗口,列写 -(不进窗口统计)。
        Scene::Order | Scene::SeqShuffle => usize::MAX,
        _ => 0,
    }
}

/// 对照行是否处于自己的统计窗口(MR5:对照行用自己的基点,不与迁移行混;
/// order/seq 行基点 = usize::MAX 恒不在窗口)。
fn control_like_window(
    cfg: &Cfg,
    tick: usize,
    period: usize,
    walk0: usize,
    land_tick: Option<usize>,
) -> bool {
    let base = control_base_of(cfg, period, walk0, land_tick);
    tick >= base && tick < base + POST_TICKS
}

/// MR3(pilesleep):迁移门槛 = 堆全睡连续 60 拍。非 pilesleep 行恒 true。
fn pile_gate_ok(gate: bool, streak: usize) -> bool {
    !gate || streak >= PILE_ALL_ASLEEP_STREAK
}

/// 目标世界(非活跃侧)是否全睡(动态刚体全部 sleeping;MR3 门槛用)。
fn pile_all_asleep(world: &PhysicsWorld) -> bool {
    world
        .bodies
        .iter()
        .all(|(_, rb)| rb.is_fixed() || rb.is_sleeping())
}

/// 目标世界是否有醒着的动态刚体(MR3 复醒检测用;exclude = 主体句柄,
/// 主体醒着行走不算堆复醒)。
fn pile_has_awake(world: &PhysicsWorld, exclude: &[RigidBodyHandle]) -> bool {
    world
        .bodies
        .iter()
        .any(|(h, rb)| !rb.is_fixed() && !exclude.contains(&h) && !rb.is_sleeping())
}

/// 目标世界(非活跃侧)引用。
fn idle_world<'w>(in_a: bool, a: &'w Side, b: &'w Side) -> &'w PhysicsWorld {
    if in_a {
        &b.world
    } else {
        &a.world
    }
}

/// 主体所在世界(活跃侧)引用。迁移后堆所在的目标世界 = 活跃侧。
fn active_world<'w>(in_a: bool, a: &'w Side, b: &'w Side) -> &'w PhysicsWorld {
    if in_a {
        &a.world
    } else {
        &b.world
    }
}

/// 主体所在世界的主体句柄组(机身 + 连杆;空 = 无主体)。
fn active_robot_handles(in_a: bool, a: &Side, b: &Side) -> Vec<RigidBodyHandle> {
    let side = if in_a { a } else { b };
    match side.robot.as_ref() {
        Some(r) => std::iter::once(r.body)
            .chain(r.links.iter().copied())
            .collect(),
        None => Vec::new(),
    }
}

/// 世界里醒着的非主体动态刚体数(MR3;exclude = 主体连杆/机身句柄)。
fn count_awake_dynamic_excluding(side: &Side, exclude: &[RigidBodyHandle]) -> usize {
    let mut n = 0usize;
    for (h, rb) in side.world.bodies.iter() {
        if rb.is_fixed() || exclude.contains(&h) {
            continue;
        }
        if !rb.is_sleeping() {
            n += 1;
        }
    }
    n
}

/// 8 只同拍迁移(same/full,T0):8 只沿 x 排开(间隔 2.5 m),站立后
/// 同时迁移,逐只报误差/窗口指标/vfwd(M10)。返回(汇总行, 逐只行)。
fn run_eight(cfg: &Cfg) -> (Outcome, Vec<String>) {
    let _ = cfg.snap;
    let gait = default_gait();
    let walk0 = STAND_TICKS;
    let total_ticks = WALK_TICKS;
    let migrate_tick = walk0 + 60; // 行走 1 s 后
                                   // 对照行(M10/eight_control_T0)不迁移:migrate_us 无值,行名用 cfg.name。
    let is_control = cfg.control_like;

    let mut side = make_side(cfg.t1);
    let n = 8;
    let mut robots: Vec<quad::Robot> = Vec::new();
    for i in 0..n {
        robots.push(spawn_into(&mut side, 10.0 + i as f32 * 2.5));
    }

    // 逐只统计(M10)。
    struct PerRobot {
        err: (f32, f32, f32, f32),
        win: WinStats,
        vfwd_sum: f64,
        vfwd_n: u64,
    }
    let mut per: Vec<PerRobot> = (0..n)
        .map(|_| PerRobot {
            err: (0.0, 0.0, 0.0, 0.0),
            win: WinStats::new(),
            vfwd_sum: 0.0,
            vfwd_n: 0,
        })
        .collect();
    let mut migrate_us = 0.0f32;
    let mut nan = false;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut cur_targets = [0.0f32; N_JOINTS];

    for tick in 0..total_ticks {
        if tick % 2 == 0 {
            cur_targets = targets_at(&gait, tick, walk0, true);
            for r in &robots {
                write_targets(&mut side.world, r, &cur_targets);
            }
        }
        // 同拍迁移:全部 8 只快照 → 移除 → 重建。对照行(eight_control_T0)
        // 不迁移(R2/M10:对照行必须没做被测操作,migrate_us 列 = 0)。
        if tick == migrate_tick && !is_control {
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
                let nr = restore_robot(
                    &mut side.world,
                    snap,
                    K,
                    quad::GROUND_FRICTION,
                    DampMode::Scaled,
                );
                let i = robots.len();
                per[i].err = quad::measure_restore_error(&side.world, &nr, snap);
                robots.push(nr);
            }
            migrate_us = t0.elapsed().as_secs_f32() * 1e6 / n as f32;
        }

        side.world.step();

        for (ri, r) in robots.iter().enumerate() {
            let body = side.world.bodies.get(r.body).expect("alive");
            let p = body.translation();
            if !p.x.is_finite() || !p.y.is_finite() {
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
            let pen = ground_pen(&side.world, r, &side.terrain);
            if tick >= migrate_tick && tick < migrate_tick + POST_TICKS {
                per[ri].win.push(upy, vy, tick_link_max, pen, p);
            }
            if tick >= walk0 {
                let fwd = rot * Vector::new(1.0, 0.0, 0.0);
                let fh = Vector::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
                per[ri].vfwd_sum += f64::from(body.linvel().dot(fh));
                per[ri].vfwd_n += 1;
            }
        }
    }

    // 逐只行(M10)+ 汇总行(全体窗口指标;逐只明细在 per 行)。
    // 行名带 cfg.name(区分迁移组与不迁移对照)。
    let mut lines = Vec::new();
    for (ri, pr) in per.iter().enumerate() {
        lines.push(format!(
            "{}_r{ri},{:.3e},{:.3e},{:.3e},{:.3e},{},{},{},{},{},{},{:.4},{},false,,{:.1},-,-,,,,,-,-,-,-,-,-,-,-,false,{:016x}",
            cfg.name,
            pr.err.0,
            pr.err.1,
            pr.err.2,
            pr.err.3,
            fnum_or(pr.win.upy_min),
            fnum_or(pr.win.max_vy),
            fnum_or(pr.win.link_max),
            fnum_or(pr.win.pen_max),
            fnum_or(pr.win.body_step_max),
            "-",
            (pr.vfwd_sum / pr.vfwd_n.max(1) as f64) as f32,
            "-",
            migrate_us,
            hash,
        ));
    }
    // 汇总 = 8 只最坏值。
    let agg_upy = per.iter().map(|p| p.win.upy_min).fold(1.0, f32::min);
    let agg_vy = per.iter().map(|p| p.win.max_vy).fold(0.0, f32::max);
    let agg_link = per.iter().map(|p| p.win.link_max).fold(0.0, f32::max);
    let agg_pen = per.iter().map(|p| p.win.pen_max).fold(0.0, f32::min);
    let agg_step = per.iter().map(|p| p.win.body_step_max).fold(0.0, f32::max);
    let vfwd_all = per
        .iter()
        .map(|p| p.vfwd_sum / p.vfwd_n.max(1) as f64)
        .sum::<f64>()
        / n as f64;
    let summary = Outcome {
        name: cfg.name.clone(),
        err_coord: 0.0,
        err_pos: 0.0,
        err_rot: 0.0,
        err_vel: 0.0,
        post_upy_min: agg_upy,
        post_max_vy: agg_vy,
        post_link_max: agg_link,
        post_pen_max: agg_pen,
        post_body_step_max: agg_step,
        contact_recover_tick: None,
        vfwd: vfwd_all as f32,
        fell_tick: None,
        nan,
        panicked: false,
        panic_msg: String::new(),
        migrate_us,
        diverge_tick: None,
        land_tick: None,
        seq_pre_order: String::new(),
        seq_post_order: String::new(),
        active_order_start: String::new(),
        active_order_end: String::new(),
        active_order_post: "-".into(),
        dst_awake_at_migrate: None,
        dst_rewake_tick: None,
        seq_pre_vfwd: 0.0,
        seq_post_vfwd: 0.0,
        seq_pre_yaw_rate: 0.0,
        seq_post_yaw_rate: 0.0,
        yaw_change_deg: 0.0,
        path_len_m: 0.0,
        slept: false,
        state_hash: hash,
    };
    (summary, lines)
}

fn fnum_or(v: f32) -> String {
    if v.is_finite() {
        format!("{v:.4}")
    } else {
        "inf".to_string()
    }
}

/// 全矩阵。行序:对照(12 相位行走 + 落地 + 站立醒/眠)先行,迁移组消费;
/// cross_hist 拆分组;order_* 顺序表;seq_* 迁移后顺序。
fn build_matrix() -> Vec<Cfg> {
    let mut all: Vec<Cfg> = Vec::new();
    for t1 in [false, true] {
        let t = if t1 { "T1" } else { "T0" };
        // M7:行走对照 12 相位各一条(窗口与对应迁移组对齐)。
        for phase in 0..12usize {
            all.push(Cfg {
                name: format!("control_walk_p{phase:02}_{t}"),
                scene: Scene::WalkPhase { phase },
                rebuild: Rebuild::Same,
                snap: SnapKind::Full,
                t1,
                eight: false,
                order: None,
                control_like: true,
            });
        }
        // 站立对照:强制醒着 + 不强制(睡眠口径)各一条。
        all.push(Cfg {
            name: format!("control_stand_awake_{t}"),
            scene: Scene::StandAwake,
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1,
            eight: false,
            order: None,
            control_like: true,
        });
        all.push(Cfg {
            name: format!("control_stand_sleep_{t}"),
            scene: Scene::StandAsleep,
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1,
            eight: false,
            order: None,
            control_like: true,
        });
        // 落地对照。
        all.push(Cfg {
            name: format!("control_land_{t}"),
            scene: Scene::LandTick,
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1,
            eight: false,
            order: None,
            control_like: true,
        });
        // M8:pingpong 对照(同 3600 拍,窗口基点 = 首次迁移拍)。
        all.push(Cfg {
            name: format!("control_pingpong_{t}"),
            scene: Scene::PingPong { gap: 30 },
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1,
            eight: false,
            order: None,
            control_like: true,
        });
    }
    // 行走中迁移:12 相位 × 重建家族 × 两快照 × 两地形。
    // MR4:full/novec 只属 same/cross/cross_hist;拆分家族(histonly/
    // pilesleep/pileawake)按 R1 放行原文只跑 full,不重复推进。
    let rebuilds = [
        Rebuild::Same,
        Rebuild::Cross,
        Rebuild::CrossHist,
        Rebuild::HistOnly,
        Rebuild::PileSleep,
        Rebuild::PileAwake,
    ];
    for phase in 0..12usize {
        for rb in rebuilds {
            let snap_list: &[SnapKind] =
                if matches!(rb, Rebuild::Same | Rebuild::Cross | Rebuild::CrossHist) {
                    &[SnapKind::Full, SnapKind::Novec]
                } else {
                    &[SnapKind::Full]
                };
            for sk in snap_list {
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
                        snap: *sk,
                        t1,
                        eight: false,
                        order: None,
                        control_like: false,
                    });
                }
            }
        }
    }
    // 站立醒/眠 × 重建家族 × 两快照 × 两地形。
    for (scene, base) in [
        (Scene::StandAwake, "stand_awake"),
        (Scene::StandAsleep, "stand_asleep"),
    ] {
        for rb in rebuilds {
            let snap_list: &[SnapKind] =
                if matches!(rb, Rebuild::Same | Rebuild::Cross | Rebuild::CrossHist) {
                    &[SnapKind::Full, SnapKind::Novec]
                } else {
                    &[SnapKind::Full]
                };
            for sk in snap_list {
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
                        snap: *sk,
                        t1,
                        eight: false,
                        order: None,
                        control_like: false,
                    });
                }
            }
        }
    }
    // 落地瞬间 × 重建家族 × 两快照 × 两地形。
    for rb in rebuilds {
        let snap_list: &[SnapKind] =
            if matches!(rb, Rebuild::Same | Rebuild::Cross | Rebuild::CrossHist) {
                &[SnapKind::Full, SnapKind::Novec]
            } else {
                &[SnapKind::Full]
            };
        for sk in snap_list {
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
                    snap: *sk,
                    t1,
                    eight: false,
                    order: None,
                    control_like: false,
                });
            }
        }
    }
    // 连续来回迁移:间隔 2、5、30、120(same/full,T0);cross 家族只跑 g30。
    for gap in [2usize, 5, 30, 120] {
        all.push(Cfg {
            name: format!("pingpong_g{gap}_same_full_T0"),
            scene: Scene::PingPong { gap },
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1: false,
            eight: false,
            order: None,
            control_like: false,
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
            order: None,
            control_like: false,
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
        order: None,
        control_like: false,
    });
    // 8 只不迁移对照(M10 要求的对照;same 路径不迁移)。
    all.push(Cfg {
        name: "eight_control_T0".into(),
        scene: Scene::Eight,
        rebuild: Rebuild::Same,
        snap: SnapKind::Full,
        t1: false,
        eight: true,
        order: None,
        control_like: true,
    });
    // R1-6 连杆顺序表(order_*;主体直接出生,不迁移,T0)。
    let mut push_order = |name: String, os: OrderSetup| {
        all.push(Cfg {
            name,
            scene: Scene::Order,
            rebuild: Rebuild::Same,
            snap: SnapKind::Full,
            t1: false,
            eight: false,
            order: Some(os),
            control_like: false,
        });
    };
    for n in [0usize, 1, 2, 3, 4, 8, 13, 16, 27, 64] {
        push_order(
            format!("order_before_block{n}"),
            OrderSetup {
                n,
                before: true,
                kind: OrderKind::Block,
                dx: 0.0,
            },
        );
    }
    for n in [1usize, 4, 27] {
        push_order(
            format!("order_after_block{n}"),
            OrderSetup {
                n,
                before: false,
                kind: OrderKind::Block,
                dx: 0.0,
            },
        );
    }
    push_order(
        "order_before_fixed64".into(),
        OrderSetup {
            n: 64,
            before: true,
            kind: OrderKind::Fixed,
            dx: 0.0,
        },
    );
    for n in [4usize, 16] {
        push_order(
            format!("order_before_nocollider{n}"),
            OrderSetup {
                n,
                before: true,
                kind: OrderKind::NoCollider,
                dx: 0.0,
            },
        );
    }
    push_order(
        "order_before_asleep16".into(),
        OrderSetup {
            n: 16,
            before: true,
            kind: OrderKind::Asleep,
            dx: 0.0,
        },
    );
    push_order(
        "order_before_block16_m200".into(),
        OrderSetup {
            n: 16,
            before: true,
            kind: OrderKind::Block,
            dx: -200.0,
        },
    );
    push_order(
        "order_before_block16_p200".into(),
        OrderSetup {
            n: 16,
            before: true,
            kind: OrderKind::Block,
            dx: 200.0,
        },
    );
    // R1-7 顺序和重建的关系:4 方块打乱 → 行走 300 拍 → same 迁移 → 1020 拍。
    all.push(Cfg {
        name: "seq_shuffle4_same_full_T0".into(),
        scene: Scene::SeqShuffle,
        rebuild: Rebuild::Same,
        snap: SnapKind::Full,
        t1: false,
        eight: false,
        order: Some(OrderSetup {
            n: 4,
            before: true,
            kind: OrderKind::Block,
            dx: 0.0,
        }),
        control_like: false,
    });
    // R2(MR6):seq 的干净世界对照(T0,不插方块,不迁移),报同样两段的
    // vfwd 与航向速率。
    all.push(Cfg {
        name: "seq_control_T0".into(),
        scene: Scene::SeqShuffle,
        rebuild: Rebuild::Same,
        snap: SnapKind::Full,
        t1: false,
        eight: false,
        order: None,
        control_like: true,
    });
    all
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
         post_link_max,post_pen_max,post_body_step_max,contact_recover_tick,vfwd,fell_tick,\
         nan,panicked,panic_msg,migrate_us,diverge_tick,land_tick,seq_pre_order,seq_post_order,\
         active_order_start,active_order_end,active_order_post,dst_awake_at_migrate,\
         dst_rewake_tick,seq_pre_vfwd,seq_post_vfwd,seq_pre_yaw_rate,seq_post_yaw_rate,\
         yaw_change_deg,path_len_m,slept,state_hash\n",
    );
    let mut extra_out = String::new();
    let mut det_out = String::new();
    let mut same_store: BTreeMap<String, Vec<u64>> = BTreeMap::new();

    for cfg in &cfgs {
        let selected = if det_mode {
            // det 全矩阵(R1 通用要求):全部行双跑。
            true
        } else if only.is_empty() {
            true
        } else {
            only.iter().any(|a| cfg.name.contains(a.as_str()))
        };
        if !selected {
            continue;
        }
        if cfg.eight {
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            let (o, per_lines) = run_eight(cfg);
            if det_mode {
                let (o2, _) = run_eight(cfg);
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
            let line = outcome_csv(&o);
            println!("{line} | {:.1}s", t0.elapsed().as_secs_f32());
            out.push_str(&line);
            out.push('\n');
            for l in per_lines {
                extra_out.push_str(&l);
                extra_out.push('\n');
            }
            continue;
        }
        // diverge 基准(M9):同场景 + 相同快照类型的 same 行存档。
        let is_same = cfg.rebuild == Rebuild::Same && !cfg.control_like;
        let key = scene_key(cfg);
        let mut ref_hashes: Vec<u64> = Vec::new();
        if !is_same {
            if let (true, Some(k)) = (cfg.rebuild.is_cross(), &key) {
                ref_hashes = same_store.get(k).cloned().unwrap_or_default();
            }
        }
        #[allow(clippy::disallowed_methods)]
        let t0 = Instant::now();
        let (o, ths) = run(cfg, &ref_hashes);
        if is_same {
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
    if !extra_out.is_empty() {
        let _ = std::fs::write("out/run_eight_per_robot.csv", &extra_out);
    }
    if det_mode {
        let _ = std::fs::write("out/run_det.txt", &det_out);
    }
}
