//! REQ-012 A00-2 demo `terrain`(2026-11-06)。
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
//! 工况(A00-2 放行表):
//! - T0 基线:一块大 cuboid(同 gait);
//! - T1 chunk 接缝:平地跨多个 chunk,只有 chunk 边界有接缝;
//! - T2 密集接缝:行走带外挖坑(1 格宽、3 格深的空气槽)打断贪心合并,
//!   让合并边界(接缝)穿过行走带;统计行走带内的接缝密度
//!   (目标 ≥ 每 2 m 一条,即 ≥ 0.5 条/m);
//! - T3 斜穿:T2 地形,初始航向 45°;
//! - T4 落差:从 1 格(1 m)高台走下到平地,只看跌落有无穿透/NaN/弹飞。
//!
//! 每个工况 5 次(初始航向 −2°/−1°/0/+1°/+2°),行走 3600 拍(前 3 s 站立
//! 序列不计),离开行走带(|横偏| > 4 m)或掉坑(y < 地面顶面 − 3 m)
//! 提前结束。步态 = 默认组(kp100 kd3 f3 s0.15 l0.06,30 Hz 目标更新)。

use std::time::Instant;

use physics_sim::collider_build::{greedy_colliders, BoxCollider, NeighborChunks};
use quad::{
    default_gait, ik, production_params, read_angle, set_target, spawn_robot_yawed, DampMode,
    GROUND_RESTITUTION, N_JOINTS,
};
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::{ColliderBuilder, PairContacts, SharedShape};
use rapier3d::math::{Pose, Vector};
use rapier3d::pipeline::PhysicsWorld;
use types::{BlockId, ChunkPos, WorldSeed};
use voxel::{builtin, Chunk, SimpleRegistry, Superflat, CHUNK_X, CHUNK_Z};
use voxel::{BlockRegistry as _, WorldGenerator as _};

/// 地面顶面(世界 y;Superflat classic 层厚 4:世界 −64..−61)。
const GROUND_TOP: f32 = -60.0;
/// 走的 chunk 数(x 向 −2..CHUNK_SPAN):3600 拍 × ~0.53 m/s ≈ 190 m ≈ 12
/// chunk,铺 14 个留余量;T1 的接缝 = chunk 边界,每 16 m 一条。
const CHUNK_SPAN: i32 = 14;
/// 行走带半宽(m):超出即提前结束(离开行走带)。
const LANE_HALF: f32 = 4.0;
/// 掉坑判定(y 低于地面顶面 3 m)。
const FALL_DROP: f32 = 3.0;
/// 行走带半宽(格;|z| ≤ 4 格 = 4 m):T2 挖坑避开这里。
const LANE_CELLS: i32 = 4;

/// 工况。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Workload {
    /// 大平面(同 gait 的地面 cuboid)。
    T0,
    /// chunk 接缝:平地跨 CHUNK_SPAN 个 chunk。
    T1,
    /// 密集接缝:行走带外挖坑打碎贪心合并。
    T2,
    /// T2 地形 + 45° 初始航向。
    T3,
    /// 1 格台阶下落:x ∈ [16, 32) 高台 +1 格,走下到平地。
    T4,
}

/// 体素世界:chunk x ∈ −2..CHUNK_SPAN、z ∈ −2..=2,Superflat classic,
/// T2/T3 挖坑、T4 高台。纯函数(无随机);同工况产出位级相同。
fn build_voxels(w: Workload) -> Vec<(ChunkPos, Chunk)> {
    let gen = Superflat::classic();
    let mut out = Vec::new();
    for cx in -2..CHUNK_SPAN {
        for cz in -2..=2 {
            let cp = ChunkPos::new(cx, cz);
            let mut chunk = gen.generate(WorldSeed(0), cp);
            if matches!(w, Workload::T2 | Workload::T3) {
                // 挖坑:1 格宽(x 向)空气槽,贯穿整列 z,深 3 格
                // (chunk 局部 y 1..3 = dirt 层,打穿到 bedrock 顶)。
                // 位置随 chunk 交错(偶数 chunk 每 4 列、奇数 chunk 错开
                // 2 列),让合并破洞的边界落进行走带,而不只是 chunk 界。
                // 行走带(|z| ≤ LANE_CELLS 格)不挖:坑在行走带外。
                let base_x = if cp.x.rem_euclid(2) == 0 { 1 } else { 3 };
                for lx in (base_x..CHUNK_X as i32).step_by(4) {
                    for lz in 0..CHUNK_Z {
                        let wz = cp.block_min().z + lz as i32;
                        if wz.abs() <= LANE_CELLS {
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
                // 高台顶面 = GROUND_TOP + 1;机器人从高台走下。
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

/// 与生产 rebuild_chunk_collider 相同的组装:greedy_colliders → fixed
/// compound,friction 0.6 / restitution 0。solid 判据用 SimpleRegistry
/// (内置表,AIR 非实心)。返回每 chunk 的 collider 数。
fn build_terrain(world: &mut PhysicsWorld, w: Workload) -> Vec<(ChunkPos, usize)> {
    let reg = SimpleRegistry::builtin();
    let voxels = build_voxels(w);
    let mut per_chunk = Vec::new();
    for (cp, chunk) in &voxels {
        let find = |x: i32, z: i32| {
            voxels
                .iter()
                .find(|(c, _)| c.x == x && c.z == z)
                .map(|(_, ch)| ch)
        };
        let neighbors = NeighborChunks {
            px: find(cp.x + 1, cp.z),
            nx: find(cp.x - 1, cp.z),
            pz: find(cp.x, cp.z + 1),
            nz: find(cp.x, cp.z - 1),
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
        let _ = world.insert(
            RigidBodyBuilder::fixed(),
            ColliderBuilder::compound(parts)
                .friction(quad::GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
        );
    }
    per_chunk
}

/// 收集一个工况的全部 collider 几何(接缝统计用;纯函数,不带 rapier)。
fn collider_geoms(w: Workload) -> Vec<BoxCollider> {
    let reg = SimpleRegistry::builtin();
    let voxels = build_voxels(w);
    let mut out = Vec::new();
    for (cp, chunk) in &voxels {
        let find = |x: i32, z: i32| {
            voxels
                .iter()
                .find(|(c, _)| c.x == x && c.z == z)
                .map(|(_, ch)| ch)
        };
        let neighbors = NeighborChunks {
            px: find(cp.x + 1, cp.z),
            nx: find(cp.x - 1, cp.z),
            pz: find(cp.x, cp.z + 1),
            nz: find(cp.x, cp.z - 1),
        };
        out.extend(greedy_colliders(chunk, *cp, &neighbors, |b| {
            reg.is_solid(b)
        }));
    }
    out
}

/// 统计地表平面(顶面 = GROUND_TOP)上、与行走带(|z| ≤ 4 m)相交的
/// 合并盒的 x 向竖直边界:接缝条数(0.5 m 网格去重)与线密度(条/m)。
/// 行走域 = x 1..(CHUNK_SPAN*16 − 1),扣除最外缘(边缘不是接缝)。
fn seam_stats(geoms: &[BoxCollider]) -> (usize, f32) {
    let mut xs: Vec<i32> = Vec::new();
    for b in geoms {
        let top = b.center.y + b.half.y;
        if (top - GROUND_TOP).abs() > 1e-3 {
            continue;
        }
        let z_lo = b.center.z - b.half.z;
        let z_hi = b.center.z + b.half.z;
        if z_lo > LANE_CELLS as f32 || z_hi < -(LANE_CELLS as f32) {
            continue;
        }
        for edge in [b.center.x - b.half.x, b.center.x + b.half.x] {
            let qx = (edge * 2.0).round() as i32;
            let wx = qx as f32 / 2.0;
            if wx <= 1.0 || wx >= (CHUNK_SPAN * 16 - 1) as f32 {
                continue;
            }
            xs.push(qx);
        }
    }
    xs.sort_unstable();
    xs.dedup();
    let span = (CHUNK_SPAN * 16 - 2) as f32;
    let density = xs.len() as f32 / span;
    (xs.len(), density)
}

// ── 单次运行 ────────────────────────────────────────────────────────────

struct Cfg {
    name: &'static str,
    w: Workload,
    /// 初始航向(弧度;0 = +x)。
    heading: f32,
    update_every: usize,
    total_ticks: usize,
    /// 8 只同世界(耗时测量,互不接触,z 间隔 2 m)。
    eight: bool,
}

struct Outcome {
    name: String,
    end_tick: usize,
    /// completed / left_lane / fell_pit / nan。
    end_reason: &'static str,
    up_y_min: f32,
    height_std: f32,
    fwd_speed_mean: f32,
    max_body_vy: f32,
    max_link_speed: f32,
    max_pen_ground: f32,
    /// 足端接触法线偏离竖直的流形数(|n.y| < 0.9),按每千拍计。
    normal_tilt_per_kt: f32,
    normal_tilt_count: u64,
    fell_tick: Option<usize>,
    nan: bool,
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

    if cfg.w == Workload::T0 {
        let _ = world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, GROUND_TOP - 0.5, 0.0)),
            ColliderBuilder::cuboid(400.0, 0.5, 400.0)
                .friction(quad::GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
        );
    } else {
        build_terrain(&mut world, cfg.w);
    }

    // 步态 = 默认组(kp100 kd3 f3 s0.15 l0.06,30 Hz 目标更新)。
    let gait = default_gait();
    let y0 = if cfg.w == Workload::T4 {
        // 出生在高台上(x = 2 < 16?不:T4 高台在 chunk cx=1 即 x∈[16,32),
        // 出生点要在台上 → spawn x = 20;走下台阶后沿 +x 继续走完行走域。
        GROUND_TOP + 1.0 + 2.0 * quad::L + quad::FOOT_R + 0.002
    } else {
        GROUND_TOP + 2.0 * quad::L + quad::FOOT_R + 0.002
    };
    let spawn_x = if cfg.w == Workload::T4 { 20.0 } else { 2.0 };
    // 8 只模式:同一行走带内沿 x 排开(间隔 2.5 m,互不接触;T2 挖坑只
    // 避让 |z| ≤ 4 m,z 向排开会让外侧几只出生在坑上)。带判定仍看第 0 只。
    let n_bots = if cfg.eight { 8 } else { 1 };
    let mut robots = Vec::new();
    for i in 0..n_bots {
        // 8 只沿 x 排开(间隔 2.5 m,互不接触;T2 挖坑只避让 |z| ≤ 4 m,
        // z 向排开会让外侧几只出生在坑上)。T4 若跑 8 只,出生在高台
        // (20, ·) 沿 x 纵排,高台 [16,32) 长 16 m 装得下。
        let off = Vector::new(i as f32 * 2.5, 0.0, 0.0);
        let spawn = Vector::new(spawn_x, y0, 0.0) + off;
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
    let fwd0 = Vector::new(cfg.heading.cos(), 0.0, cfg.heading.sin());

    let tau = quad::TAU_LIMIT;
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
    let mut tilt_count = 0u64;
    let mut fell_tick = None;
    let mut nan = false;
    let mut end_reason = "completed";
    let mut end_tick = cfg.total_ticks;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let colliders = &robot.colliders;
    let is_robot_collider = |h: rapier3d::geometry::ColliderHandle| colliders.contains(&h);
    let mut step_ms: Vec<f32> = Vec::new();

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

        // 横偏 = 相对"出生点 + 初始航向"的行走带轴线(出生点不在原点时,
        // 相对过原点的轴会把出生偏移算成横偏——T3 45° 出生 (2,0,0) 偏轴
        // 1.41 m 就是这么错的)。带轴线过第 0 只的出生点。
        let lat0 = if cfg.w == Workload::T4 { 20.0 } else { 2.0 };
        let lateral = (p.x - lat0) * (-fwd0.z) + p.z * fwd0.x;

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
            for sh in &robot.shanks {
                let rb = world.bodies.get(*sh).expect("alive");
                max_link_speed = max_link_speed.max(rb.linvel().length());
            }
            // 接触统计:连杆侧对地穿透 + 法线倾斜(|n.y| < 0.9,流形级)。
            for pair in world.narrow_phase.contact_graph().interactions() {
                if let PairContacts::Rigid(rigid) = &pair.contacts {
                    if !(is_robot_collider(pair.collider1) || is_robot_collider(pair.collider2)) {
                        continue;
                    }
                    for manifold in &rigid.manifolds {
                        if manifold.data.normal.y.abs() < 0.9 {
                            tilt_count += 1;
                        }
                        for pt in &manifold.points {
                            max_pen = max_pen.min(pt.dist);
                        }
                    }
                }
            }
            if lateral.abs() > LANE_HALF {
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

    let height_std = if h_n > 0 {
        let mean = h_sum / h_n as f64;
        ((h_sq / h_n as f64 - mean * mean).max(0.0)).sqrt() as f32
    } else {
        0.0
    };
    let fwd_speed_mean = (vfwd_sum / vel_n.max(1) as f64) as f32;

    step_ms.sort_by(|a, b| a.total_cmp(b));
    let p50 = percentile_of(&step_ms, 0.5);
    let p95 = percentile_of(&step_ms, 0.95);
    let mx = step_ms.last().copied().unwrap_or(0.0);

    Outcome {
        name: cfg.name.to_string(),
        end_tick,
        end_reason,
        up_y_min: up_min,
        height_std,
        fwd_speed_mean,
        max_body_vy: max_vy,
        max_link_speed,
        max_pen_ground: max_pen,
        normal_tilt_per_kt: tilt_count as f32 / (end_tick.max(1) as f32 / 1000.0),
        normal_tilt_count: tilt_count,
        fell_tick,
        nan,
        state_hash: hash,
        step_ms_p50: p50,
        step_ms_p95: p95,
        step_ms_max: mx,
    }
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
        "{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{},{},{:016x},{:.3},{:.3},{:.3}",
        o.name,
        o.end_tick,
        o.end_reason,
        o.up_y_min,
        o.height_std,
        o.fwd_speed_mean,
        o.max_body_vy,
        o.max_link_speed,
        o.max_pen_ground,
        o.normal_tilt_per_kt,
        o.normal_tilt_count,
        o.fell_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.nan,
        o.state_hash,
        o.step_ms_p50,
        o.step_ms_p95,
        o.step_ms_max,
    )
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("--list") {
        println!("T0_h0..4 T1_h0..4 T2_h0..4 T3_h0..4 T4_h0..4 (h2 = nominal heading)");
        println!("T2_h0 T2_h1 ... T2_h4 = heading offsets -2/-1/0/+1/+2 deg");
        return;
    }
    let det_mode = args.first().map(String::as_str) == Some("det");
    let only: Vec<&String> = if det_mode || args.is_empty() {
        Vec::new()
    } else {
        args.iter().collect()
    };

    // 接缝密度报告(T1/T2/T3;放行要求 ≥ 每 2 m 一条)。
    let (t2_seams, t2_density) = seam_stats(&collider_geoms(Workload::T2));
    let (t1_seams, t1_density) = seam_stats(&collider_geoms(Workload::T1));
    println!(
        "seam density: T1 {t1_seams} seams / {t1_density:.3} per m ; T2 {t2_seams} seams / {t2_density:.3} per m"
    );

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
            let heading = if tag == "T3" {
                std::f32::consts::FRAC_PI_4
            } else {
                0.0
            } + dy.to_radians();
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
    // 8 只同世界耗时组(T2 地形,互不接触;放行:同世界 8 只在 T2 上的
    // 每拍耗时 P50/P95/max)。
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
        "name,end_tick,end_reason,up_y_min,height_std,fwd_speed_mean,max_body_vy,\
         max_link_speed,max_pen_ground,normal_tilt_per_kt,normal_tilt_count,\
         fell_tick,nan,state_hash,step_ms_p50,step_ms_p95,step_ms_max\n",
    );
    for cfg in &cfgs {
        let selected = if det_mode {
            cfg.name.starts_with("T2_h0")
        } else if cfg.eight {
            only.iter().any(|a| a.as_str() == "eight")
        } else {
            only.is_empty() && !cfg.eight
                || (!only.is_empty()
                    && !only.iter().any(|a| a.as_str() == "eight")
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
        // det 模式只跑 T2 一组双跑(放行判据 5)。
        if det_mode && cfg.name.starts_with("T2_h0") {
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
}
