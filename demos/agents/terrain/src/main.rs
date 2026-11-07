//! REQ-012 A00-2 demo `terrain`(2026-11-06;A00-2 R1 重做布局与接触口径)。
//!
//! 问题:足端半径 2.2 cm 的小脚走过贪心合并的体素碰撞体时,共面盒子接缝
//! 会不会卡脚、抖动或被弹起(A00-2 放行;REQ-012 §9 第 2 项)。
//!
//! 地形与生产一致(放行硬要求):路径依赖 `physics-sim`,直接调用
//! `collider_build::greedy_colliders`,每 chunk 一个 fixed 复合碰撞体,
//! friction 0.6 / restitution 0(同 `rebuild_chunk_collider`,sim.rs:4179)。
//! 体素 = Superflat classic(bedrock×1 + dirt×3,世界 y −64..−61),
//! 地面顶面 = 世界 y = −60(生产布景纪律,同 sim.rs drop_origin_y 注释)。
//!
//! R1 修正(验收评论 5 条 + 放行 R1-1…R1-6):
//! - 航向符号:`from_rotation_y(θ)` 把 +x 转向 **−z**,前进方向
//!   `fwd0 = (cos θ, 0, −sin θ)`(旧版取 +sin θ,T3 实际朝 (0.70,−0.71) 走,
//!   行走带轴线却按 +z 算,5 次全在 657 拍离带);
//! - 接触统计窄口径(R1-2):只统计**脚 collider 对地形**、流形最深点
//!   dist < 1 mm 的接触;旧口径混入机器人自身连杆对(T0 名义组 13358 次
//!   "倾斜"全部是自身连杆对,球脚与大平面的法线恒竖直,真倾斜为 0);
//! - 工况布局(R1-3):T1 出生 x=10(跨 x=16/32 chunk 缝)、带半宽 12 m;
//!   T2 沟只挖 |z|>12(脚不进沟)、离带阈值 11.7 m;T3 沿实际 45° 方向
//!   半宽 4 m 的带、只挖"最近点也在带外"的格子;T4 带半宽 12 m、5 次
//!   都要走到台阶沿 x=32;
//! - 每次报路程与脚跨接缝次数(R1-4);颠簸事件表(R1-5);8 只组新布局
//!   重跑并核对旧 `eight_T2` up·y 0.83 事件(R1-6)。

use std::time::Instant;

use physics_sim::collider_build::{greedy_colliders, BoxCollider, NeighborChunks};
use quad::{
    default_gait, ik, production_params, read_angle, set_target, spawn_robot_yawed, DampMode,
    GROUND_RESTITUTION, N_JOINTS, TAU_LIMIT,
};
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::{ColliderBuilder, ColliderHandle, PairContacts, SharedShape};
use rapier3d::math::{Pose, Vector};
use rapier3d::pipeline::PhysicsWorld;
use types::{BlockId, ChunkPos, WorldSeed};
use voxel::{builtin, Chunk, SimpleRegistry, Superflat, CHUNK_X, CHUNK_Z};
use voxel::{BlockRegistry as _, WorldGenerator as _};

/// 地面顶面(世界 y;Superflat classic 层厚 4:世界 −64..−61)。
const GROUND_TOP: f32 = -60.0;
/// chunk x ∈ −2..CHUNK_SPAN(世界 x ∈ [−32, 224));T1 出生 x=10 走 ~33 m
/// 到 x≈43,跨过 x=16 与 x=32 两条 chunk 缝(R1-3)。
const CHUNK_SPAN: i32 = 14;
/// chunk z ∈ −2..=2(世界 z ∈ [−32, 32))。
const CHUNK_Z_LO: i32 = -2;
const CHUNK_Z_HI: i32 = 2;
/// 掉坑判定(y 低于地面顶面 3 m)。
const FALL_DROP: f32 = 3.0;
/// T2 沟只挖 |z| > 12 格(R1-3:保证脚不进沟——行走带半宽 12 m)。
const T2_DITCH_MARGIN: i32 = 12;
/// T3 带半宽(格/m 同尺寸;R1-3)。
const T3_LANE_HALF: f32 = 4.0;
/// 颠簸事件阈值(R1-5)。
const BUMP_VY: f32 = 0.35;
const BUMP_UPY: f32 = 0.97;

/// 工况。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Workload {
    /// 大平面(同 gait 的地面 cuboid)。
    T0,
    /// chunk 接缝:平地跨 CHUNK_SPAN 个 chunk。
    T1,
    /// 密集接缝:|z|>12 挖沟打断贪心合并,接缝穿过行走带(带半宽 12 m)。
    T2,
    /// T2 式挖沟(只挖最近点在带外的格子),45° 初始航向,带半宽 4 m。
    T3,
    /// 1 格台阶下落:x ∈ [16, 32) 高台 +1 格,出生台上,走到 x=32 下台阶。
    T4,
}

/// 工况的布局参数(R1-3)。
struct Layout {
    /// 行走带半宽(m)。
    lane_half: f32,
    /// 离带阈值(|横偏| > leave 即结束)。
    leave: f32,
    /// 出生 x。
    origin_x: f32,
    /// 基准航向(弧度;偏置 ±1°/±2° 叠加)。
    heading: f32,
}

fn layout(w: Workload) -> Layout {
    match w {
        Workload::T0 | Workload::T1 | Workload::T2 => Layout {
            lane_half: 12.0,
            leave: 11.7,
            origin_x: 10.0,
            heading: 0.0,
        },
        Workload::T3 => Layout {
            lane_half: T3_LANE_HALF,
            leave: 3.7,
            origin_x: 10.0,
            heading: std::f32::consts::FRAC_PI_4,
        },
        Workload::T4 => Layout {
            lane_half: 12.0,
            leave: 11.7,
            origin_x: 20.0, // 高台上(x ∈ [16,32))
            heading: 0.0,
        },
    }
}

/// 体素世界:chunk x ∈ −2..CHUNK_SPAN、z ∈ −2..=2,Superflat classic,
/// T2/T3 挖沟、T4 高台。纯函数(无随机);同工况产出位级相同。
fn build_voxels(w: Workload) -> Vec<(ChunkPos, Chunk)> {
    let gen = Superflat::classic();
    let mut out = Vec::new();
    for cx in -2..CHUNK_SPAN {
        for cz in CHUNK_Z_LO..=CHUNK_Z_HI {
            let cp = ChunkPos::new(cx, cz);
            let mut chunk = gen.generate(WorldSeed(0), cp);
            if matches!(w, Workload::T2) {
                // T2:1 格宽(x 向)空气槽,贯穿 z,深 3 格(挖穿 dirt 层到
                // bedrock 顶)。位置随 chunk 交错;**只挖 |wz| > 12**(R1-3),
                // 行走带半宽 12 m 内没有沟,脚不进沟。
                let base_x = if cp.x.rem_euclid(2) == 0 { 1 } else { 3 };
                for lx in (base_x..CHUNK_X as i32).step_by(4) {
                    for lz in 0..CHUNK_Z {
                        let wz = cp.block_min().z + lz as i32;
                        if wz.abs() <= T2_DITCH_MARGIN {
                            continue;
                        }
                        for ly in 1..4usize {
                            chunk.set(lx as usize, ly, lz, BlockId::AIR);
                        }
                    }
                }
            }
            if w == Workload::T3 {
                // T3:同样每 4 列的沟,但**只挖"最近点也在带外"的格子**
                // (R1-3:格子角点会伸进带内)。带 = 过出生点 (origin_x, 0)、
                // 方向 45° 基准航向、半宽 4 m 的走廊。点到轴线距离 = |2D 叉积|。
                let lay = layout(w);
                let dx = lay.heading.cos();
                let dz = -lay.heading.sin();
                let base_x = if cp.x.rem_euclid(2) == 0 { 1 } else { 3 };
                for lx in (base_x..CHUNK_X as i32).step_by(4) {
                    for lz in 0..CHUNK_Z {
                        let wx = (cp.block_min().x + lx) as f32;
                        let wz = (cp.block_min().z + lz as i32) as f32;
                        // 格子 4 个 (x,z) 角点到带轴线的最小 |叉积|。
                        let mut min_cross = f32::INFINITY;
                        for ox in [0.0f32, 1.0] {
                            for oz in [0.0f32, 1.0] {
                                let px = wx + ox - lay.origin_x;
                                let pz = wz + oz;
                                let cross = px * dz - pz * dx;
                                min_cross = min_cross.min(cross.abs());
                            }
                        }
                        if min_cross <= lay.lane_half {
                            continue;
                        }
                        for ly in 1..4usize {
                            chunk.set(lx as usize, ly, lz, BlockId::AIR);
                        }
                    }
                }
            }
            if w == Workload::T4 && cx == 1 {
                // T4:x ∈ [16, 32) 高台加高 1 格(dirt),其余平地。
                // 高台顶面 = GROUND_TOP + 1;机器人从台上走下 x=32 台阶。
                for lz in 0..CHUNK_Z {
                    for lx in 0..CHUNK_X {
                        chunk.set(lx, 4, lz, builtin::DIRT);
                    }
                }
            }
            out.push((cp, chunk));
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

/// 与生产 rebuild_chunk_collider 相同的组装:greedy_colliders → fixed
/// compound,friction 0.6 / restitution 0。solid 判据用 SimpleRegistry。
/// 返回 (每 chunk 的 collider 数, 全部地形 collider 句柄)。
fn build_terrain(
    world: &mut PhysicsWorld,
    w: Workload,
) -> (Vec<(ChunkPos, usize)>, Vec<ColliderHandle>) {
    let reg = SimpleRegistry::builtin();
    let voxels = build_voxels(w);
    let mut per_chunk = Vec::new();
    let mut handles: Vec<ColliderHandle> = Vec::new();
    for (cp, chunk) in &voxels {
        let neighbors = NeighborChunks {
            px: neighbor_lookup(&voxels, cp.x + 1, cp.z),
            nx: neighbor_lookup(&voxels, cp.x - 1, cp.z),
            pz: neighbor_lookup(&voxels, cp.x, cp.z + 1),
            nz: neighbor_lookup(&voxels, cp.x, cp.z - 1),
        };
        let boxes: Vec<BoxCollider> = greedy_colliders(chunk, *cp, &neighbors, |b| reg.is_solid(b));
        per_chunk.push((*cp, boxes.len()));
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
    (per_chunk, handles)
}

/// 收集一个工况的全部 collider 几何(接缝统计用;纯函数,不带 rapier)。
fn collider_geoms(w: Workload) -> Vec<BoxCollider> {
    let reg = SimpleRegistry::builtin();
    let voxels = build_voxels(w);
    let mut out = Vec::new();
    for (cp, chunk) in &voxels {
        let neighbors = NeighborChunks {
            px: neighbor_lookup(&voxels, cp.x + 1, cp.z),
            nx: neighbor_lookup(&voxels, cp.x - 1, cp.z),
            pz: neighbor_lookup(&voxels, cp.x, cp.z + 1),
            nz: neighbor_lookup(&voxels, cp.x, cp.z - 1),
        };
        out.extend(greedy_colliders(chunk, *cp, &neighbors, |b| {
            reg.is_solid(b)
        }));
    }
    out
}

/// 接缝线:axis 0 = x 向竖线(法线沿 x,c 为 x 坐标);axis 1 = z 向横线。
#[derive(Clone, Copy, PartialEq)]
struct SeamLine {
    axis: usize,
    c: f32,
}

/// 点到带轴线的横偏(带轴过 origin,方向 (cos θ, −sin θ);2D 叉积)。
fn lateral_of(px: f32, pz: f32, origin_x: f32, heading: f32) -> f32 {
    let dx = heading.cos();
    let dz = -heading.sin();
    (px - origin_x) * dz - pz * dx
}

/// 收集行走带内的接缝线(R1-3:0.5 m 网格去重;x 向 + z 向)。
/// 盒与带相交 ⇔ 盒 4 个 (x,z) 角点的最小 |横偏| ≤ 带半宽。
fn collect_seam_lines(w: Workload) -> Vec<SeamLine> {
    let lay = layout(w);
    let geoms = collider_geoms(w);
    let mut xs: Vec<i32> = Vec::new();
    let mut zs: Vec<i32> = Vec::new();
    for b in &geoms {
        let top = b.center.y + b.half.y;
        if (top - GROUND_TOP).abs() > 1e-3 {
            continue; // 只统计地表平面(顶面 = GROUND_TOP)的盒
        }
        // 盒 4 角点最小 |横偏| ≤ 带半宽 → 盒与带相交。
        let mut min_lat = f32::INFINITY;
        for sx in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                let lat = lateral_of(
                    b.center.x + sx * b.half.x,
                    b.center.z + sz * b.half.z,
                    lay.origin_x,
                    lay.heading,
                );
                min_lat = min_lat.min(lat.abs());
            }
        }
        if min_lat > lay.lane_half {
            continue;
        }
        let in_domain = |edge: f32, lo: i32, hi: i32| edge > lo as f32 && edge < hi as f32;
        for edge in [b.center.x - b.half.x, b.center.x + b.half.x] {
            // x 向竖线(x 向边界;0.5 m 网格去重),去掉域最外缘。
            if in_domain(edge, -31, 223) {
                xs.push((edge * 2.0).round() as i32);
            }
        }
        for edge in [b.center.z - b.half.z, b.center.z + b.half.z] {
            if in_domain(edge, -31, 31) {
                zs.push((edge * 2.0).round() as i32);
            }
        }
    }
    xs.sort_unstable();
    xs.dedup();
    zs.sort_unstable();
    zs.dedup();
    let mut out: Vec<SeamLine> = xs
        .into_iter()
        .map(|qx| SeamLine {
            axis: 0,
            c: qx as f32 / 2.0,
        })
        .collect();
    out.extend(zs.into_iter().map(|qz| SeamLine {
        axis: 1,
        c: qz as f32 / 2.0,
    }));
    out
}

/// 接缝统计:条数与线密度(条/m;分母 = 带内可行走长度,按工况口径)。
fn seam_stats(w: Workload, lines: &[SeamLine]) -> (usize, f32, usize, usize) {
    let nx = lines.iter().filter(|l| l.axis == 0).count();
    let nz = lines.iter().filter(|l| l.axis == 1).count();
    let span = match w {
        Workload::T3 => 45.0, // 45° 带在 z ∈ [0,−32) 域内的可行走长度
        _ => 214.0,           // 出生 x=10 到域尾 224
    };
    (lines.len(), lines.len() as f32 / span, nx, nz)
}

/// 沟中心线 x 坐标集合(与 build_voxels 的挖沟位置一致;归因用)。
fn ditch_xs(w: Workload) -> Vec<f32> {
    if !matches!(w, Workload::T2 | Workload::T3) {
        return Vec::new();
    }
    let mut out = Vec::new();
    for cx in -2..CHUNK_SPAN {
        let base_x = if cx.rem_euclid(2) == 0 { 1 } else { 3 };
        for lx in (base_x..CHUNK_X as i32).step_by(4) {
            out.push((cx * 16 + lx) as f32 + 0.5);
        }
    }
    out
}

/// 颠簸事件段在进行中的聚合状态:
/// (robot_idx, 拍数, max |vy|, min up·y, x, z, 各脚是否接触, min |n.y|)。
type BumpAcc = (usize, usize, f32, f32, f32, f32, [bool; 4], f32);

// ── 单次运行 ────────────────────────────────────────────────────────────

struct Cfg {
    name: &'static str,
    w: Workload,
    /// 初始航向(弧度;含 ±1°/±2° 偏置)。
    heading: f32,
    update_every: usize,
    total_ticks: usize,
    /// 8 只同世界(耗时测量 + 事件核对)。
    eight: bool,
}

/// 颠簸事件段(R1-5):连续满足条件的拍(间隔 ≤ 5 拍)合并。
struct BumpEvent {
    tick: usize,
    ticks: usize,
    max_vy: f32,
    min_upy: f32,
    x: f32,
    z: f32,
    /// 段内有 dist<1mm 脚-地形接触的脚(0..4)。
    feet: Vec<usize>,
    min_abs_ny: f32,
    /// 最近地形特征的描述("seam x=16 / d 0.02"、"ditch x=13.5 / d 11.9"、
    /// "step x=32 / d 0.3"、"none")。
    nearest: String,
}

struct Outcome {
    name: String,
    end_tick: usize,
    /// completed / left_lane / fell_pit / nan。
    end_reason: &'static str,
    /// 沿航向路程(机身位置投影的净位移,m)。
    distance: f32,
    /// 脚跨接缝次数(R1-4;4 脚 × 全部带内接缝线)。
    seam_crossings: u64,
    up_y_min: f32,
    height_std: f32,
    fwd_speed_mean: f32,
    max_body_vy: f32,
    max_link_speed: f32,
    max_pen_ground: f32,
    /// 窄口径(R1-2):脚对地形、最深点 dist<1mm 的流形。
    foot_manifolds: u64,
    foot_tilt: u64,
    foot_tilt_near_seam: u64,
    foot_min_abs_ny: f32,
    fell_tick: Option<usize>,
    nan: bool,
    bumps: Vec<BumpEvent>,
    /// T4 特有:下台阶拍(机身过 x=32)与下落过程(其后 60 拍)峰值。
    t4_edge_tick: Option<usize>,
    t4_fall_link_v: f32,
    t4_fall_pen: f32,
    state_hash: u64,
    step_ms_p50: f32,
    step_ms_p95: f32,
    step_ms_max: f32,
}

fn hash_f32(h: u64, v: f32) -> u64 {
    let mut h = h;
    for b in &v.to_ne_bytes() {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

fn run(cfg: &Cfg) -> Outcome {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = production_params(1.0 / 60.0);

    let lay = layout(cfg.w);
    let terrain_handles = if cfg.w == Workload::T0 {
        let (_, collider) = world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, GROUND_TOP - 0.5, 0.0)),
            ColliderBuilder::cuboid(400.0, 0.5, 400.0)
                .friction(quad::GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
        );
        vec![collider]
    } else {
        let (_, handles) = build_terrain(&mut world, cfg.w);
        handles
    };

    // 步态 = 默认组(kp100 kd3 f3 s0.15 l0.06,30 Hz 目标更新)。
    let gait = default_gait();
    let y0 = if cfg.w == Workload::T4 {
        GROUND_TOP + 1.0 + 2.0 * quad::L + quad::FOOT_R + 0.002
    } else {
        GROUND_TOP + 2.0 * quad::L + quad::FOOT_R + 0.002
    };
    // 8 只沿 x 排开(间隔 2.5 m,互不接触;T2 沟在 |z|>12,z 向排开会进沟)。
    let n_bots = if cfg.eight { 8 } else { 1 };
    let mut robots = Vec::new();
    for i in 0..n_bots {
        let off = Vector::new(i as f32 * 2.5, 0.0, 0.0);
        let spawn = Vector::new(lay.origin_x, y0, 0.0) + off;
        let r = spawn_robot_yawed(
            &mut world,
            spawn,
            quad::yaw_quat(cfg.heading),
            1.0,
            quad::GROUND_FRICTION,
            DampMode::Scaled,
        );
        robots.push(r);
    }
    let robot = &robots[0];
    // R1-1:from_rotation_y(θ) 把 +x 转向 −z → 前进方向 (cos θ, 0, −sin θ);
    // 横偏/路程统一走 lateral_of / lateral_of_at。
    let seam_lines = if cfg.w == Workload::T0 {
        Vec::new()
    } else {
        collect_seam_lines(cfg.w)
    };
    let ditch_xs = ditch_xs(cfg.w);
    let step_edges: Vec<f32> = if cfg.w == Workload::T4 {
        vec![16.0, 32.0]
    } else {
        Vec::new()
    };

    let tau = TAU_LIMIT;
    let kp = quad::DEFAULT_KP;
    let kd = quad::DEFAULT_KD;
    let rise_ticks = 60usize; // 1 s 直腿降到站姿
    let settle_ticks = 2 * 60;
    let walk0 = rise_ticks + settle_ticks;

    let mut targets = [0.0f32; N_JOINTS];

    let mut up_min = 1.0f32;
    let mut h_sum = 0.0f64;
    let mut h_sq = 0.0f64;
    let mut h_n = 0u64;
    let mut vfwd_sum = 0.0f64;
    let mut vel_n = 0u64;
    let mut max_vy = 0.0f32;
    let mut max_link_speed = 0.0f32;
    let mut max_pen = 0.0f32;
    // 窄口径接触统计(R1-2)。
    let mut foot_manifolds = 0u64;
    let mut foot_tilt = 0u64;
    let mut foot_tilt_near = 0u64;
    let mut foot_min_abs_ny = 1.0f32;
    // 跨缝计数(R1-4):每脚 × 每线上一拍的带符号距离符号。
    let mut seam_prev: Vec<[f32; 4]> = Vec::new();
    let mut seam_crossings = 0u64;
    // 颠簸事件(R1-5)。
    let mut bumps: Vec<BumpEvent> = Vec::new();
    let mut cur_bump: Option<BumpAcc> = None;
    // T4:下台阶与摔倒过程。
    let mut t4_edge_tick = None;
    let mut t4_fall_link_v = 0.0f32;
    let mut t4_fall_pen = 0.0f32;
    let mut fell_tick = None;
    let mut fell_link_v = 0.0f32;
    let mut fell_pen = 0.0f32;
    let mut nan = false;
    let mut end_reason = "completed";
    let mut end_tick = cfg.total_ticks;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let colliders = &robot.colliders;
    let feet = &robot.feet;
    let is_robot_collider = |h: ColliderHandle| colliders.contains(&h);
    let is_foot = |h: ColliderHandle| feet.contains(&h);
    let mut step_ms: Vec<f32> = Vec::new();
    let mut bump_unexplained: Vec<String> = Vec::new();
    let _ = &mut bump_unexplained;

    for tick in 0..cfg.total_ticks {
        #[allow(clippy::disallowed_methods)] // 8 只耗时组;demo 侧计时,纪律自束
        let t0 = Instant::now();
        if tick % cfg.update_every == 0 {
            for robot in robots.iter() {
                for leg in 0..4 {
                    let foot = if tick < rise_ticks {
                        let a = tick as f32 / rise_ticks as f32;
                        let sm = a * a * (3.0 - 2.0 * a);
                        let (x, _) = gait.stand_foot();
                        (
                            x,
                            quad::FULL_STRETCH_DY + (-gait.h - quad::FULL_STRETCH_DY) * sm,
                        )
                    } else if tick < walk0 {
                        gait.stand_foot()
                    } else {
                        let walk_t = (tick - walk0) as f32 / 60.0;
                        gait.foot_target(leg, walk_t)
                    };
                    let (dx, dy) = foot;
                    let (a, k) = ik(dx, dy);
                    targets[leg * 3] = 0.0;
                    targets[leg * 3 + 1] = a;
                    targets[leg * 3 + 2] = k;
                }
                for (dof, tgt) in targets.iter().enumerate() {
                    set_target(&mut world, robot, dof, *tgt, kp, kd, tau);
                }
            }
        }

        world.step();
        step_ms.push(t0.elapsed().as_secs_f32() * 1000.0);

        let body = world.bodies.get(robot.body).expect("alive");
        let p = body.translation();
        let up = body.rotation() * Vector::new(0.0, 1.0, 0.0);
        if !p.x.is_finite() || !p.y.is_finite() {
            nan = true;
            end_tick = tick;
            end_reason = "nan";
            break;
        }
        for v in [p.x, p.y, p.z] {
            hash = hash_f32(hash, v);
        }
        let rot = body.rotation();
        for c in [rot.w, rot.x, rot.y, rot.z] {
            hash = hash_f32(hash, c);
        }
        for dof in 0..N_JOINTS {
            hash = hash_f32(hash, read_angle(&world.multibody_joints, robot, dof));
        }

        // 横偏 = 相对"出生点 + 初始航向"轴线的 2D 叉积(R1-1 修正符号后
        // fwd0 与 yaw 一致)。
        let lateral = lateral_of(p.x, p.z, lay.origin_x, cfg.heading);

        // 脚世界坐标(球心):每拍都要(跨缝、接触统计、事件归因)。
        let mut foot_pos = [Vector::ZERO; 4];
        for (i, sh) in robot.shanks.iter().enumerate() {
            let rb = world.bodies.get(*sh).expect("alive");
            foot_pos[i] = rb.translation() + rb.rotation() * Vector::new(0.0, -quad::L / 2.0, 0.0);
        }
        // 跨缝计数(R1-4):脚位置在每条带内接缝线的带符号距离变号。
        if tick >= walk0 && !seam_lines.is_empty() {
            let mut cur: Vec<[f32; 4]> = Vec::with_capacity(seam_lines.len());
            for (li, line) in seam_lines.iter().enumerate() {
                let mut signs = [0.0f32; 4];
                for (fi, fp) in foot_pos.iter().enumerate() {
                    signs[fi] = if line.axis == 0 {
                        fp.x - line.c
                    } else {
                        fp.z - line.c
                    };
                }
                if let Some(prev) = seam_prev.get(li) {
                    for fi in 0..4 {
                        if prev[fi] < 0.0 && signs[fi] >= 0.0 || prev[fi] > 0.0 && signs[fi] <= 0.0
                        {
                            seam_crossings += 1;
                        }
                    }
                }
                cur.push(signs);
            }
            seam_prev = cur;
        }

        if tick >= walk0 {
            if up.y < 0.7 && fell_tick.is_none() {
                fell_tick = Some(tick - walk0);
            }
            up_min = up_min.min(up.y);
            let py = f64::from(p.y);
            h_sum += py;
            h_sq += py * py;
            h_n += 1;
            let lv = body.linvel();
            max_vy = max_vy.max(lv.y.abs());
            let fwd = body.rotation() * Vector::new(1.0, 0.0, 0.0);
            let fh = Vector::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
            vfwd_sum += f64::from(lv.dot(fh));
            vel_n += 1;
            // 最大连杆速度:全部连杆 + 机身(R1 通用要求)。
            for lh in robot
                .links
                .iter()
                .copied()
                .chain(std::iter::once(robot.body))
            {
                let rb = world.bodies.get(lh).expect("alive");
                max_link_speed = max_link_speed.max(rb.linvel().length());
            }
            // 接触统计:连杆侧对地穿透(自接触已关;地形 = fixed)。
            // 窄口径(R1-2):脚对地形、流形最深点 dist<1mm。
            for pair in world.narrow_phase.contact_graph().interactions() {
                if let PairContacts::Rigid(rigid) = &pair.contacts {
                    if !(is_robot_collider(pair.collider1) || is_robot_collider(pair.collider2)) {
                        continue;
                    }
                    let foot_side: Option<ColliderHandle> = if is_foot(pair.collider1) {
                        Some(pair.collider1)
                    } else if is_foot(pair.collider2) {
                        Some(pair.collider2)
                    } else {
                        None
                    };
                    let vs_terrain = terrain_handles.contains(&pair.collider1)
                        || terrain_handles.contains(&pair.collider2);
                    for manifold in &rigid.manifolds {
                        let deepest = manifold
                            .points
                            .iter()
                            .map(|pt| pt.dist)
                            .reduce(f32::max)
                            .unwrap_or(0.0);
                        if vs_terrain {
                            max_pen = max_pen.min(deepest);
                        }
                        // 摔倒 / 下落过程穿透(T4)。
                        if let Some(ft) = fell_tick {
                            if tick >= walk0 + ft {
                                fell_pen = fell_pen.min(deepest);
                            }
                        }
                        if let Some(et) = t4_edge_tick {
                            if tick >= et && tick < et + 60 {
                                t4_fall_pen = t4_fall_pen.min(deepest);
                            }
                        }
                        // 窄口径:脚对地形、dist < 1 mm。
                        if let Some(_foot) = foot_side {
                            if vs_terrain && deepest < 0.001 {
                                foot_manifolds += 1;
                                let abs_ny = manifold.data.normal.y.abs();
                                foot_min_abs_ny = foot_min_abs_ny.min(abs_ny);
                                if abs_ny < 0.9 {
                                    foot_tilt += 1;
                                    // 接触点 ≈ 脚球心 − n·(R − dist)(球面接触点)。
                                    let fp = foot_pos
                                        [robot.feet.iter().position(|f| *f == _foot).unwrap_or(0)];
                                    let n = manifold.data.normal;
                                    let cp = fp - n * (quad::FOOT_R - deepest.max(0.0));
                                    let near = seam_lines.iter().any(|line| {
                                        let d = if line.axis == 0 {
                                            (cp.x - line.c).abs()
                                        } else {
                                            (cp.z - line.c).abs()
                                        };
                                        d <= quad::FOOT_R
                                    });
                                    if near {
                                        foot_tilt_near += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            // T4:下台阶检测(机身过 x=32)与下落 60 拍峰值。
            if cfg.w == Workload::T4 && t4_edge_tick.is_none() && p.x >= 32.0 {
                t4_edge_tick = Some(tick);
            }
            if let Some(et) = t4_edge_tick {
                if tick < et + 60 {
                    for lh in robot
                        .links
                        .iter()
                        .copied()
                        .chain(std::iter::once(robot.body))
                    {
                        let rb = world.bodies.get(lh).expect("alive");
                        t4_fall_link_v = t4_fall_link_v.max(rb.linvel().length());
                    }
                }
            }
            if let Some(ft) = fell_tick {
                if tick < walk0 + ft + 60 {
                    for lh in robot
                        .links
                        .iter()
                        .copied()
                        .chain(std::iter::once(robot.body))
                    {
                        let rb = world.bodies.get(lh).expect("alive");
                        fell_link_v = fell_link_v.max(rb.linvel().length());
                    }
                }
            }
            // 颠簸事件(R1-5):|vy| > 0.35 或 up·y < 0.97。多只时每只都
            // 检测(事件归到该组;位置/脚取对应个体)。这里逐只:
            for (ri, r) in robots.iter().enumerate() {
                let b = world.bodies.get(r.body).expect("alive");
                let bp = b.translation();
                let bup = (b.rotation() * Vector::new(0.0, 1.0, 0.0)).y;
                let bvy = b.linvel().y;
                let active = bvy.abs() > BUMP_VY || bup < BUMP_UPY;
                // 该个体当前接触(dist<1mm)的脚与最斜法线。
                let mut feet_touch = [false; 4];
                let mut min_ny = 1.0f32;
                if active {
                    for pair in world.narrow_phase.contact_graph().interactions() {
                        if let PairContacts::Rigid(rigid) = &pair.contacts {
                            for (fi, fc) in r.feet.iter().enumerate() {
                                if *fc == pair.collider1 || *fc == pair.collider2 {
                                    for manifold in &rigid.manifolds {
                                        let deepest = manifold
                                            .points
                                            .iter()
                                            .map(|pt| pt.dist)
                                            .reduce(f32::max)
                                            .unwrap_or(0.0);
                                        if deepest < 0.001 {
                                            feet_touch[fi] = true;
                                            min_ny = min_ny.min(manifold.data.normal.y.abs());
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                let open = cur_bump.as_mut();
                if active {
                    match open {
                        Some((_, _cnt, mv, mu, _x, _z, ftouch, mny)) => {
                            *_cnt += 1;
                            *mv = mv.max(bvy.abs());
                            *mu = mu.min(bup);
                            for (i, t) in ftouch.iter_mut().enumerate() {
                                *t |= feet_touch[i];
                            }
                            *mny = mny.min(min_ny);
                        }
                        None => {
                            cur_bump =
                                Some((ri, 1, bvy.abs(), bup, bp.x, bp.z, feet_touch, min_ny));
                        }
                    }
                } else if let Some((bri, cnt, mv, mu, bx, bz, ftouch, mny)) = cur_bump.take() {
                    if cnt >= 2 {
                        // 归因:最近的接缝线 / 沟中心 / 台阶沿。
                        let mut nearest = String::from("none");
                        let mut best_d = f32::INFINITY;
                        for line in &seam_lines {
                            let d = if line.axis == 0 {
                                (bx - line.c).abs()
                            } else {
                                (bz - line.c).abs()
                            };
                            if d < best_d {
                                best_d = d;
                                nearest = format!(
                                    "seam{}={}/d{:.2}",
                                    if line.axis == 0 { "x" } else { "z" },
                                    line.c,
                                    d
                                );
                            }
                        }
                        for dx0 in &ditch_xs {
                            let dxy = ((bx - dx0).abs()).hypot(12.0f32.min(bp.z.abs()).max(0.0));
                            // 沟在 |z|>12;机身到沟的水平距离。
                            let d = ((bx - dx0) * (bx - dx0)
                                + ((bp.z.abs() - T2_DITCH_MARGIN as f32).max(0.0)).powi(2))
                            .sqrt();
                            let _ = dxy;
                            if d < best_d {
                                best_d = d;
                                nearest = format!("ditch_x={}/d{:.2}", dx0, d);
                            }
                        }
                        for se in &step_edges {
                            let d = (bx - se).abs();
                            if d < best_d {
                                best_d = d;
                                nearest = format!("step_x={}/d{:.2}", se, d);
                            }
                        }
                        bumps.push(BumpEvent {
                            tick,
                            ticks: cnt,
                            max_vy: mv,
                            min_upy: mu,
                            x: bx,
                            z: bz,
                            feet: ftouch
                                .iter()
                                .enumerate()
                                .filter(|(_, t)| **t)
                                .map(|(i, _)| i)
                                .collect(),
                            min_abs_ny: mny,
                            nearest,
                        });
                        let _ = bri;
                    }
                }
                let _ = ri;
            }
            if lateral.abs() > lay.leave {
                end_reason = "left_lane";
                end_tick = tick;
                break;
            }
            if p.y < GROUND_TOP - FALL_DROP {
                end_reason = "fell_pit";
                end_tick = tick;
                break;
            }
        }
    }

    // 事件段闭合(跑到总拍数仍 active)。
    if let Some((_ri, cnt, mv, mu, bx, bz, ftouch, mny)) = cur_bump.take() {
        if cnt >= 2 {
            bumps.push(BumpEvent {
                tick: end_tick,
                ticks: cnt,
                max_vy: mv,
                min_upy: mu,
                x: bx,
                z: bz,
                feet: ftouch
                    .iter()
                    .enumerate()
                    .filter(|(_, t)| **t)
                    .map(|(i, _)| i)
                    .collect(),
                min_abs_ny: mny,
                nearest: "tail".to_string(),
            });
        }
    }

    let height_std = if h_n > 0 {
        let mean = h_sum / h_n as f64;
        ((h_sq / h_n as f64 - mean * mean).max(0.0)).sqrt() as f32
    } else {
        0.0
    };
    let fwd_speed_mean = (vfwd_sum / vel_n.max(1) as f64) as f32;
    // 路程 = 沿初始航向的净位移(R1-4)。
    let end_body = world.bodies.get(robot.body).expect("alive");
    let distance = lateral_of_at(end_body.translation(), lay.origin_x, cfg.heading);

    step_ms.sort_by(|a, b| a.total_cmp(b));
    let p50 = percentile_of(&step_ms, 0.5);
    let p95 = percentile_of(&step_ms, 0.95);
    let mx = step_ms.last().copied().unwrap_or(0.0);
    let _ = fell_link_v;
    let _ = fell_pen;
    let _ = bump_unexplained;

    Outcome {
        name: cfg.name.to_string(),
        end_tick,
        end_reason,
        distance,
        seam_crossings,
        up_y_min: up_min,
        height_std,
        fwd_speed_mean,
        max_body_vy: max_vy,
        max_link_speed,
        max_pen_ground: max_pen,
        foot_manifolds,
        foot_tilt,
        foot_tilt_near_seam: foot_tilt_near,
        foot_min_abs_ny,
        fell_tick,
        nan,
        bumps,
        t4_edge_tick,
        t4_fall_link_v,
        t4_fall_pen,
        state_hash: hash,
        step_ms_p50: p50,
        step_ms_p95: p95,
        step_ms_max: mx,
    }
}

/// 终点横坐标投影(路程 = 沿航向轴的净位移)。
fn lateral_of_at(p: Vector, origin_x: f32, heading: f32) -> f32 {
    let dx = heading.cos();
    let dz = -heading.sin();
    (p.x - origin_x) * dx + p.z * dz
}

/// 已排序切片的分位(线性插值;空返回 0)。
fn percentile_of(sorted: &[f32], p: f64) -> f32 {
    if sorted.is_empty() {
        return 0.0;
    }
    let n = sorted.len();
    let idx = ((n as f64 - 1.0) * p).round() as usize;
    sorted[idx.min(n - 1)]
}

fn outcome_csv(o: &Outcome) -> String {
    format!(
        "{},{},{},{:.2},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.5},{},{},{},{},{},{:.4},{:.4},{},{:016x},{:.3},{:.3},{:.3}",
        o.name,
        o.end_tick,
        o.end_reason,
        o.distance,
        o.seam_crossings,
        o.up_y_min,
        o.height_std,
        o.fwd_speed_mean,
        o.max_body_vy,
        o.max_link_speed,
        o.max_pen_ground,
        o.foot_min_abs_ny,
        o.foot_manifolds,
        o.foot_tilt,
        o.foot_tilt_near_seam,
        o.fell_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.t4_edge_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.t4_fall_link_v,
        o.t4_fall_pen,
        o.nan,
        o.state_hash,
        o.step_ms_p50,
        o.step_ms_p95,
        o.step_ms_max,
    )
}

fn bumps_txt(name: &str, o: &Outcome) -> String {
    let mut s = format!("# bumps: {} ({} events)\n", o.name, o.bumps.len());
    let _ = name;
    s.push_str("# tick,ticks,max_vy,min_upy,x,z,feet,min_abs_ny,nearest\n");
    for b in &o.bumps {
        s.push_str(&format!(
            "{},{},{:.3},{:.4},{:.2},{:.2},{},{:.4},{}\n",
            b.tick,
            b.ticks,
            b.max_vy,
            b.min_upy,
            b.x,
            b.z,
            if b.feet.is_empty() {
                "-".to_string()
            } else {
                b.feet
                    .iter()
                    .map(|i| i.to_string())
                    .collect::<Vec<_>>()
                    .join("|")
            },
            if b.min_abs_ny < 1.0 {
                format!("{:.4}", b.min_abs_ny)
            } else {
                "-".to_string()
            },
            b.nearest,
        ));
    }
    s
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("--list") {
        println!("T0_h0..4 T1_h0..4 T2_h0..4 T3_h0..4 T4_h0..4 (h2 = nominal heading)");
        println!("T2_h0 T2_h1 ... T2_h4 = heading offsets -2/-1/0/+1/+2 deg");
        println!("eight = 8 bots on T2 (R1-6)");
        return;
    }
    let det_mode = args.first().map(String::as_str) == Some("det");
    let only: Vec<&String> = if det_mode || args.is_empty() {
        Vec::new()
    } else {
        args.iter().collect()
    };

    // 接缝密度报告(R1-3:按新布局;T3 含 x 向与 z 向)。
    for (tag, w) in [
        ("T1", Workload::T1),
        ("T2", Workload::T2),
        ("T3", Workload::T3),
    ] {
        let lines = collect_seam_lines(w);
        let (n, dens, nx, nz) = seam_stats(w, &lines);
        println!("seam lines {tag}: {n} total (x-facing {nx}, z-facing {nz}) / {dens:.3} per m");
    }

    let yaw_offsets = [-2.0f32, -1.0, 0.0, 1.0, 2.0];
    let mut cfgs: Vec<Cfg> = Vec::new();
    let workloads = [
        ("T0", Workload::T0),
        ("T1", Workload::T1),
        ("T2", Workload::T2),
        ("T3", Workload::T3),
        ("T4", Workload::T4),
    ];
    for (tag, w) in workloads {
        for (i, dy) in yaw_offsets.iter().enumerate() {
            let heading = layout(w).heading + dy.to_radians();
            let name = Box::leak(format!("{tag}_h{i}_dy{dy:+.0}").into_boxed_str());
            cfgs.push(Cfg {
                name,
                w,
                heading,
                update_every: 2,
                total_ticks: 3600 + 3 * 60,
                eight: false,
            });
        }
    }
    // 8 只同世界耗时组(T2 新布局,沿 x 排开;R1-6 事件核对)。
    cfgs.push(Cfg {
        name: "eight_T2",
        w: Workload::T2,
        heading: 0.0,
        update_every: 2,
        total_ticks: 1800 + 3 * 60,
        eight: true,
    });

    let mut out = String::new();
    out.push_str(
        "name,end_tick,end_reason,distance,seam_crossings,up_y_min,height_std,\
         fwd_speed_mean,max_body_vy,max_link_speed,max_pen_ground,foot_min_abs_ny,\
         foot_manifolds,foot_tilt,foot_tilt_near_seam,fell_tick,t4_edge_tick,\
         t4_fall_link_v,t4_fall_pen,nan,state_hash,step_ms_p50,step_ms_p95,step_ms_max\n",
    );
    let mut bump_out = String::new();
    for cfg in &cfgs {
        let only_has_eight = only.iter().any(|a| a.as_str() == "eight");
        let selected = if det_mode {
            cfg.name.starts_with("T2_h2") || cfg.eight
        } else if cfg.eight {
            only_has_eight
        } else {
            only.is_empty() && !cfg.eight
                || (!only.is_empty()
                    && !only_has_eight
                    && only.iter().any(|a| cfg.name.starts_with(a.as_str())))
        };
        if !selected {
            continue;
        }
        // demo 自身要报每工况耗时;不在 workspace lints 下,allow 是纪律自束
        // (与 gait 同口径)。
        #[allow(clippy::disallowed_methods)]
        let t0 = Instant::now();
        let o = run(cfg);
        println!("{} | {:.1}s", outcome_csv(&o), t0.elapsed().as_secs_f32());
        out.push_str(&outcome_csv(&o));
        out.push('\n');
        bump_out.push_str(&bumps_txt(cfg.name, &o));
        // det 模式只跑 T2 一组双跑(放行判据 5)。
        if det_mode && cfg.name.starts_with("T2_h2") {
            let o2 = run(cfg);
            println!(
                "determinism: {:016x} vs {:016x} -> {}",
                o.state_hash,
                o2.state_hash,
                if o.state_hash == o2.state_hash {
                    "IDENTICAL"
                } else {
                    "DIFFER"
                }
            );
        }
    }
    let _ = std::fs::create_dir_all("out");
    let _ = std::fs::write("out/run_summary.csv", &out);
    // 颠簸事件表(R1-5)写到提交文件;默认运行会重写,提交前用 --submit 语义
    // 由提交流程保证(与 gait 的 matrix_full 同口径:本文件即提交数据)。
    let _ = std::fs::write("out/bumps_submitted.txt", &bump_out);
}
