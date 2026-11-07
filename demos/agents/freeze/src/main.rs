//! REQ-012 A00-3 demo `freeze`(2026-11-07)。
//!
//! 问题(A00-3 放行原文):多连杆能不能冻住,唤醒后会不会弹飞?REQ-012 §3
//! 写明"rapier 对 multibody 的运动学切换行为未验证"。
//!
//! 先读 REQ-007 冻结语义与 `crates/physics-sim` 现有实现,再读 rapier 对
//! multibody 根/非根连杆 `body_type` 的处理:
//! - 生产语义(sim.rs freeze_entity):清速度 → `set_body_type(KinematicPositionBased)`
//!   (rapier 0.36 的 KinematicPositionBased 下 set_linvel/set_angvel 是 no-op,
//!   必须先清再切);唤醒 = `set_body_type(Dynamic)`。
//! - rapier 0.36.1 `Multibody::update_root_type`(multibody_joint/multibody.rs
//!   ~:1290):每拍 `forward_kinematics` 时若根刚体 `is_dynamic()` 与
//!   `root_is_dynamic` 不符,重建根关节(free ↔ fixed)。**只处理根连杆**;
//!   非根连杆切 kinematic/fixed 的行为本 demo 实证(预期:关节自由度仍在
//!   multibody 里,非根切型不影响求解——是否成立由数据回答)。
//!
//! 方案三种(都跑,放行):
//! - `kin`:全部连杆切 `KinematicPositionBased`,唤醒切回 `Dynamic`;
//! - `fixed`:全部连杆切 `Fixed`,唤醒切回;
//! - `snap`:移除整个 multibody(刚体 + 关节),碰撞体留在 fixed 刚体上当
//!   障碍物,唤醒按 migrate 的快照重建。
//!
//! 统一口径(A00-3 放行):60 Hz、生产求解参数(只覆写 dt)、默认步态
//! (kp 100、kd 3、3 Hz)、k = 1/100、阻尼按 k 缩放、30 Hz 更新目标、
//! 不开 extra。

use std::time::Instant;

use quad::{
    default_gait, ik, production_params, read_angle, restore_robot, set_target, spawn_robot,
    DampMode, GROUND_RESTITUTION, N_JOINTS, TAU_LIMIT,
};
use rapier3d::dynamics::{RigidBodyBuilder, RigidBodyHandle, RigidBodyType};
use rapier3d::geometry::{ColliderBuilder, ColliderHandle};
use rapier3d::math::{Pose, Vector};
use rapier3d::pipeline::PhysicsWorld;

/// 地面顶面(平面,同 migrate T0)。
const GROUND_TOP: f32 = 0.0;
/// 站立序列结束拍。
const STAND_TICKS: usize = 180;
/// 唤醒后跟踪窗口(判据 3)。
const POST_TICKS: usize = 120;
/// 单次冻结组的总拍数:站立 + 行走(冻结在行走中段)+ 唤醒后段。
const WALK_TICKS: usize = STAND_TICKS + 1200;
/// 长冻结组的静置拍数(放行:600 和 3600 两档)。
const FREEZE_LONG: usize = 3600;
const FREEZE_SHORT: usize = 600;
/// 质量比例 k(压缩档)。
const K: f32 = 1.0 / 100.0;

/// 冻结方案。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Scheme {
    /// 全部连杆切 KinematicPositionBased(放行原文口径;预期 panic,
    /// rapier multibody.rs:1457 断言非根连杆必须 Dynamic)。
    Kin,
    /// 全部连杆切 Fixed(同上,预期 panic)。
    Fixed,
    /// 移除 multibody,碰撞体留 fixed 障碍物;唤醒按快照重建。
    Snap,
    /// 只切根连杆 KinematicPositionBased(实际可行的运动学冻结)。
    KinRoot,
    /// 只切根连杆 Fixed。
    FixedRoot,
}

impl Scheme {
    fn tag(self) -> &'static str {
        match self {
            Scheme::Kin => "kin",
            Scheme::Fixed => "fixed",
            Scheme::Snap => "snap",
            Scheme::KinRoot => "kinroot",
            Scheme::FixedRoot => "fixedroot",
        }
    }
    /// 全连杆切型(rapier 断言会 panic 的方案)。
    fn all_links(self) -> bool {
        matches!(self, Scheme::Kin | Scheme::Fixed)
    }
}

/// 场景。
#[derive(Clone, Copy, PartialEq)]
enum Scene {
    /// 醒着站立中冻结(每拍 wake_up 的站立口径;冻结期间不再唤醒)。
    Stand,
    /// 行走中冻结:步态周期(f3 = 20 拍)内 12 个相位。
    WalkPhase { phase: usize },
    /// 站在 3×3 方块堆上冻结(堆在冻结期间不动;唤醒后主体在堆顶)。
    OnPile,
    /// 被单格方块压着冻结(方块轻放在背上,同 mass M3One 口径)。
    Pressed,
    /// 离地 0.5 m 下落途中冻结(瞬移悬空后第 10 拍冻结,落地前)。
    Falling,
    /// 冻结期间有方块砸中(冻结后 100 拍,1 kg 方块从 3 m 高落在背上)。
    HitWhileFrozen,
    /// 冻结期间脚下方块被移走(站 3×3 堆顶,冻结后 100 拍移除全堆)。
    SupportRemoved,
    /// 对照:同参数、不冻结的同一条轨迹。
    Control,
    /// 8 只一起冻结、一起唤醒(耗时)。
    Eight,
}

/// 唤醒速度口径:恢复冻结前的速度 / 唤醒时速度清零(两种都跑,放行)。
#[derive(Clone, Copy, PartialEq, Eq)]
enum WakeVel {
    /// 恢复冻结前的速度(快照里带)。
    Restore,
    /// 速度清零(生产 freeze_entity 语义:冻结时本来就清零)。
    Zero,
}

impl WakeVel {
    fn tag(self) -> &'static str {
        match self {
            WakeVel::Restore => "rst",
            WakeVel::Zero => "z",
        }
    }
}

struct Cfg {
    name: String,
    scene: Scene,
    scheme: Scheme,
    wake_vel: WakeVel,
    /// 冻结时长档(拍);0 = 场景默认。
    freeze_ticks: usize,
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

// ── 世界与机器人 ─────────────────────────────────────────────────────────

struct Side {
    world: PhysicsWorld,
    robot: Option<quad::Robot>,
    /// 地面 collider(穿透统计/冻结期位移判定用;当前统计走 bodies,保留字段)。
    #[allow(dead_code)]
    terrain: Vec<ColliderHandle>,
    blocks: Vec<RigidBodyHandle>,
}

fn make_side(with_robot: bool) -> Side {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = production_params(1.0 / 60.0);
    let (_, collider) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, GROUND_TOP - 0.5, 0.0)),
        ColliderBuilder::cuboid(400.0, 0.5, 400.0)
            .friction(quad::GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
    );
    let robot = if with_robot {
        let y0 = GROUND_TOP + 2.0 * quad::L + quad::FOOT_R + 0.002;
        Some(spawn_robot(
            &mut world,
            Vector::new(0.0, y0, 0.0),
            K,
            quad::GROUND_FRICTION,
            DampMode::Scaled,
        ))
    } else {
        None
    };
    Side {
        world,
        robot,
        terrain: vec![collider],
        blocks: Vec::new(),
    }
}

/// 生产一致的单格方块(同 mass:密度 1、摩擦 0.6、恢复 0.1)。
fn spawn_block(world: &mut PhysicsWorld, pos: Vector, vel: Vector) -> RigidBodyHandle {
    world
        .insert(
            RigidBodyBuilder::dynamic()
                .pose(Pose {
                    translation: pos,
                    ..Pose::IDENTITY
                })
                .linvel(vel),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5)
                .density(1.0)
                .friction(0.6)
                .restitution(0.1),
        )
        .0
}

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

// ── 冻结 / 唤醒三方案 ───────────────────────────────────────────────────

/// 冻结前的记录:全部连杆位姿 + 速度 + 关节坐标/速度(唤醒与判据 2 用)。
struct FrozenState {
    /// 13 个连杆位姿(links 序 + 根最后,同 quad::Robot.links 序)。
    link_poses: Vec<Pose>,
    link_linvels: Vec<Vector>,
    link_angvels: Vec<Vector>,
    joint_coords: [f32; N_JOINTS],
    joint_vels: [f32; N_JOINTS],
    targets: [f32; N_JOINTS],
}

fn capture(world: &PhysicsWorld, robot: &quad::Robot, targets: &[f32; N_JOINTS]) -> FrozenState {
    let mut poses = Vec::new();
    let mut lins = Vec::new();
    let mut angs = Vec::new();
    for lh in robot
        .links
        .iter()
        .copied()
        .chain(std::iter::once(robot.body))
    {
        let rb = world.bodies.get(lh).expect("alive");
        poses.push(*rb.position());
        lins.push(rb.linvel());
        angs.push(rb.angvel());
    }
    let mut coords = [0.0f32; N_JOINTS];
    let mut vels = [0.0f32; N_JOINTS];
    for dof in 0..N_JOINTS {
        coords[dof] = read_angle(&world.multibody_joints, robot, dof);
        vels[dof] = mb_joint_vel(world, robot, dof);
    }
    FrozenState {
        link_poses: poses,
        link_linvels: lins,
        link_angvels: angs,
        joint_coords: coords,
        joint_vels: vels,
        targets: *targets,
    }
}

/// 关节速度(solver 回写的 generalized velocity;快照口径同 quad)。
fn mb_joint_vel(world: &PhysicsWorld, robot: &quad::Robot, dof: usize) -> f32 {
    let (mb_index, link_id) = robot.joints[dof];
    let Some(mb) = world.multibody_joints.get_multibody(mb_index) else {
        return 0.0;
    };
    let aid = mb.link(link_id).expect("link alive").assembly_id();
    mb.generalized_velocity()[aid]
}

/// snap 方案的障碍物句柄:移除 multibody 时把连杆碰撞体保留在 fixed 刚体上。
struct SnapObstacles {
    /// 每连杆一个 fixed 刚体(位置 = 冻结时连杆位姿,collider 形状同原)。
    obstacles: Vec<RigidBodyHandle>,
}

/// 冻结(schema 分派)。返回 snap 方案的障碍物记录(其余返回空)。
///
/// `Kin`/`Fixed` = 全部连杆切型(放行原文的口径)。rapier 0.36.1 的
/// `Multibody::forward_kinematics`(multibody_joint/multibody.rs:1457)有
/// 硬断言"非根连杆必须 Dynamic"——全连杆切型在下一次 step 即 panic,
/// 这是本 demo 的实证结果(§3"运动学切换行为未验证"的答案),不是 demo bug。
/// `KinRoot`/`FixedRoot` = 只切根连杆(实际可行的运动学冻结;非根保持
/// Dynamic,关节由 multibody 求解冻结前的目标)。
fn do_freeze(side: &mut Side, scheme: Scheme, state: &FrozenState) -> SnapObstacles {
    let robot = side.robot.as_ref().expect("alive");
    match scheme {
        Scheme::Kin | Scheme::Fixed | Scheme::KinRoot | Scheme::FixedRoot => {
            let ty = match scheme {
                Scheme::Kin | Scheme::KinRoot => RigidBodyType::KinematicPositionBased,
                _ => RigidBodyType::Fixed,
            };
            // 生产语义:先清速度再切(rapier 0.36 kinematic 下 set_vel no-op)。
            let handles: Vec<RigidBodyHandle> =
                if matches!(scheme, Scheme::KinRoot | Scheme::FixedRoot) {
                    vec![robot.body]
                } else {
                    robot
                        .links
                        .iter()
                        .copied()
                        .chain(std::iter::once(robot.body))
                        .collect()
                };
            for h in handles {
                if let Some(rb) = side.world.bodies.get_mut(h) {
                    rb.set_linvel(Vector::ZERO, false);
                    rb.set_angvel(Vector::ZERO, false);
                    rb.set_body_type(ty, false);
                }
            }
            SnapObstacles {
                obstacles: Vec::new(),
            }
        }
        Scheme::Snap => {
            // 移除整个 multibody;每连杆碰撞体形状在 fixed 刚体上重建为
            // 障碍物(位置 = 冻结时连杆位姿)。
            let old = side.robot.take().expect("alive");
            let all: Vec<RigidBodyHandle> = old
                .links
                .iter()
                .copied()
                .chain(std::iter::once(old.body))
                .collect();
            // 障碍物:先建 fixed 刚体(位置同冻结连杆),再移除原刚体。
            // 形状近似:机身 cuboid;髋 ball;大腿/小腿 capsule_y;足 ball。
            // 用 quad 的常量重建(与 spawn_robot 同形状)。
            let mut obstacles = Vec::new();
            for (i, pose) in state.link_poses.iter().enumerate() {
                let shape = if i == old.links.len() {
                    // 根(机身)。
                    ColliderBuilder::cuboid(
                        quad::BODY_HALF[0],
                        quad::BODY_HALF[1],
                        quad::BODY_HALF[2],
                    )
                } else {
                    match i % 4 {
                        0 => ColliderBuilder::ball(quad::HIP_BALL_R),
                        1 => ColliderBuilder::capsule_y(quad::L / 2.0, quad::THIGH_RADIUS),
                        2 => ColliderBuilder::capsule_y(quad::L / 2.0, quad::SHANK_RADIUS),
                        _ => ColliderBuilder::ball(quad::FOOT_R).translation(Vector::new(
                            0.0,
                            -quad::L / 2.0,
                            0.0,
                        )),
                    }
                };
                let (h, _) = world_insert_fixed(&mut side.world, *pose, shape);
                obstacles.push(h);
            }
            for h in all {
                side.world.remove_body(h);
            }
            SnapObstacles { obstacles }
        }
    }
}

fn world_insert_fixed(
    world: &mut PhysicsWorld,
    pose: Pose,
    collider: ColliderBuilder,
) -> (RigidBodyHandle, ColliderHandle) {
    world.insert(
        RigidBodyBuilder::fixed().pose(Pose {
            translation: pose.translation,
            rotation: pose.rotation,
            ..Pose::IDENTITY
        }),
        collider
            .friction(quad::GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
    )
}

/// 唤醒(kin/fixed:切回 Dynamic + 按口径写速度;snap:按快照重建)。
/// 返回(新机器人(或原句柄组), 唤醒后步进前误差)。
fn do_unfreeze(
    side: &mut Side,
    scheme: Scheme,
    wake_vel: WakeVel,
    state: &FrozenState,
    obstacles: &SnapObstacles,
) -> (Option<quad::Robot>, (f32, f32, f32, f32)) {
    match scheme {
        Scheme::Kin | Scheme::Fixed | Scheme::KinRoot | Scheme::FixedRoot => {
            let robot = side.robot.as_ref().expect("alive");
            let handles: Vec<RigidBodyHandle> = if scheme.all_links() {
                robot
                    .links
                    .iter()
                    .copied()
                    .chain(std::iter::once(robot.body))
                    .collect()
            } else {
                vec![robot.body]
            };
            for h in &handles {
                if let Some(rb) = side.world.bodies.get_mut(*h) {
                    rb.set_body_type(RigidBodyType::Dynamic, false);
                }
            }
            // 速度口径:恢复(冻结前)/清零(生产语义本来就 0)。
            for (i, h) in handles.iter().enumerate() {
                if let Some(rb) = side.world.bodies.get_mut(*h) {
                    match wake_vel {
                        WakeVel::Restore => {
                            rb.set_linvel(state.link_linvels[i], false);
                            rb.set_angvel(state.link_angvels[i], false);
                        }
                        WakeVel::Zero => {}
                    }
                }
            }
            // multibody 根切型已发生:下一拍 forward_kinematics 的
            // update_root_type 会按新根类型重建根关节——这里先手动触发
            // (与生产唤醒后立刻 step 的路径一致)。全连杆切型的唤醒走不到
            // 这里(capture 时已 panic,矩阵记录 panicked 行)。
            let robot = side.robot.as_ref().expect("alive");
            let (mb_index, _) = robot.joints[0];
            if let Some(mb) = side.world.multibody_joints.get_multibody_mut(mb_index) {
                mb.forward_kinematics(&side.world.bodies, true);
                mb.update_rigid_bodies(&mut side.world.bodies, false);
            }
            wake_all(&mut side.world, robot);
            let err = measure_frozen_error(side, robot, state);
            (None, err)
        }
        Scheme::Snap => {
            // 移除障碍物;按 migrate 快照口径重建(restore_robot)。
            for h in &obstacles.obstacles {
                side.world.remove_body(*h);
            }
            // 构造 AgentSnapshot 兼容结构:根位姿 = links 最后一个。
            let n_links = state.link_poses.len() - 1;
            let snap = quad::AgentSnapshot {
                root_pose: state.link_poses[n_links],
                root_linvel: state.link_linvels[n_links],
                root_angvel: state.link_angvels[n_links],
                joint_coords: state.joint_coords,
                joint_vels: if wake_vel == WakeVel::Restore {
                    state.joint_vels
                } else {
                    [0.0; N_JOINTS]
                },
                link_poses: state.link_poses[..n_links].to_vec(),
                targets: state.targets,
            };
            let new_robot = restore_robot(
                &mut side.world,
                &snap,
                K,
                quad::GROUND_FRICTION,
                DampMode::Scaled,
            );
            let err = quad::measure_restore_error(&side.world, &new_robot, &snap);
            (Some(new_robot), err)
        }
    }
}

/// 唤醒后步进前误差(判据 2):kin/fixed 与 snap 同口径——连杆位姿/关节
/// 坐标 vs 冻结前(quad::measure_restore_error 的快照式)。
fn measure_frozen_error(
    side: &Side,
    robot: &quad::Robot,
    state: &FrozenState,
) -> (f32, f32, f32, f32) {
    let n_links = state.link_poses.len() - 1;
    let snap = quad::AgentSnapshot {
        root_pose: state.link_poses[n_links],
        root_linvel: state.link_linvels[n_links],
        root_angvel: state.link_angvels[n_links],
        joint_coords: state.joint_coords,
        joint_vels: state.joint_vels,
        link_poses: state.link_poses[..n_links].to_vec(),
        targets: state.targets,
    };
    quad::measure_restore_error(&side.world, robot, &snap)
}

/// 冻结期间连杆位移最大值(判据 1):对 kin/fixed 每拍读刚体平移 vs 冻结
/// 前;snap 对障碍物刚体(snap 期间原刚体已不存在,障碍物不动 = fixed)。
fn frozen_drift(
    side: &Side,
    scheme: Scheme,
    state: &FrozenState,
    obstacles: &SnapObstacles,
) -> f32 {
    match scheme {
        Scheme::Kin | Scheme::Fixed | Scheme::KinRoot | Scheme::FixedRoot => {
            let robot = side.robot.as_ref().expect("alive");
            let mut max_d = 0.0f32;
            for (i, lh) in robot
                .links
                .iter()
                .copied()
                .chain(std::iter::once(robot.body))
                .enumerate()
            {
                let rb = side.world.bodies.get(lh).expect("alive");
                let d = rb.translation() - state.link_poses[i].translation;
                max_d = max_d.max(d.length());
            }
            max_d
        }
        Scheme::Snap => {
            let mut max_d = 0.0f32;
            for (i, h) in obstacles.obstacles.iter().enumerate() {
                let rb = side.world.bodies.get(*h).expect("alive");
                let d = rb.translation() - state.link_poses[i].translation;
                max_d = max_d.max(d.length());
            }
            max_d
        }
    }
}

struct Outcome {
    name: String,
    /// 冻结期间连杆位移最大值(判据 1;snap = 障碍物位移)。
    frozen_drift: f32,
    /// 唤醒后步进前误差(判据 2)。
    err_coord: f32,
    err_pos: f32,
    err_rot: f32,
    err_vel: f32,
    /// 唤醒后 POST_TICKS:up·y 最小、最大 |vy|、连杆峰值、机身最大升高。
    post_upy_min: f32,
    post_max_vy: f32,
    post_link_max: f32,
    body_max_rise: f32,
    /// 全程 vfwd(行走拍;对照同口径)。
    vfwd: f32,
    fell_tick: Option<usize>,
    nan: bool,
    /// panic 隔离(全连杆切型方案 Kin/Fixed 预期 panic;catch_unwind 记行)。
    panicked: bool,
    panic_msg: String,
    /// 冻结/唤醒总耗时(µs;8 只组为 8 只平均)。
    freeze_us: f32,
    state_hash: u64,
}

fn outcome_csv(o: &Outcome) -> String {
    if o.panicked {
        // panic 行:整行占位 + 消息(列序一致;消息内逗号替换)。
        return format!(
            "{},-,-,-,-,-,-,-,-,-,-,-,false,true,\"{}\",0,{:016x}",
            o.name,
            o.panic_msg.replace(',', " "),
            o.state_hash,
        );
    }
    format!(
        "{},{:.3e},{:.3e},{:.3e},{:.3e},{:.4},{:.4},{:.4},{:.4},{:.4},{},{},{},false,{},{:.1},{:016x}",
        o.name,
        o.frozen_drift,
        o.err_coord,
        o.err_pos,
        o.err_rot,
        o.err_vel,
        o.post_upy_min,
        o.post_max_vy,
        o.post_link_max,
        o.body_max_rise,
        o.vfwd,
        o.fell_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.nan,
        o.panic_msg,
        o.freeze_us,
        o.state_hash,
    )
}

fn run_once(cfg: &Cfg) -> Result<Outcome, String> {
    match std::panic::catch_unwind(|| run(cfg)) {
        Ok(o) => Ok(o),
        Err(e) => {
            let msg = e
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_else(|| "non-string panic".into());
            Ok(Outcome {
                name: cfg.name.clone(),
                frozen_drift: 0.0,
                err_coord: 0.0,
                err_pos: 0.0,
                err_rot: 0.0,
                err_vel: 0.0,
                post_upy_min: 0.0,
                post_max_vy: 0.0,
                post_link_max: 0.0,
                body_max_rise: 0.0,
                vfwd: 0.0,
                fell_tick: None,
                nan: false,
                panicked: true,
                panic_msg: msg,
                freeze_us: 0.0,
                state_hash: 0,
            })
        }
    }
}

fn run(cfg: &Cfg) -> Outcome {
    let gait = default_gait();
    let period = (60.0 / gait.freq).round() as usize;
    let walk0 = STAND_TICKS;
    let walking = matches!(cfg.scene, Scene::WalkPhase { .. });
    // 冻结拍:WalkPhase 相位 → 行走开始后 phase·period/12;Stand → walk0;
    // 其余场景见循环内布景。冻结时长:cfg.freeze_ticks(0 = 场景默认:
    // 行走/站立组 = FREEZE_SHORT;pile/pressed/falling/hit/suprm = FREEZE_SHORT)。
    let freeze_len = if cfg.freeze_ticks > 0 {
        cfg.freeze_ticks
    } else {
        FREEZE_SHORT
    };
    // 冻结时刻。
    let freeze_tick: usize = match cfg.scene {
        Scene::WalkPhase { phase } => walk0 + 60 + phase * period / 12,
        Scene::Stand | Scene::OnPile | Scene::SupportRemoved => walk0,
        // Pressed / HitWhileFrozen / Falling 的冻结拍在 freeze_now 分支里
        // 单独判定(分别 210 / 180 / 70),不走 freeze_tick。
        _ => usize::MAX,
    };
    let total_ticks = match cfg.scene {
        Scene::WalkPhase { .. } | Scene::Stand | Scene::Control => WALK_TICKS,
        Scene::OnPile | Scene::Pressed | Scene::HitWhileFrozen | Scene::SupportRemoved => {
            STAND_TICKS + freeze_len + 600
        }
        Scene::Falling => STAND_TICKS + 300,
        Scene::Eight => STAND_TICKS + 600 + 600,
    };

    let mut side = make_side(true);
    // 布景方块(第 0 拍建)。
    match cfg.scene {
        Scene::OnPile | Scene::SupportRemoved => {
            for dx in -1i32..=1 {
                for dz in -1i32..=1 {
                    side.blocks.push(spawn_block(
                        &mut side.world,
                        Vector::new(dx as f32, GROUND_TOP + 0.5, dz as f32),
                        Vector::ZERO,
                    ));
                }
            }
            // 主体出生在堆顶(y0 = 1 m + 站姿)。瞬移根刚体 + **强唤醒整只**:
            // 不唤醒时非根连杆的位姿不同步(实测快照里非根仍在地面出生位,
            // 根在堆顶,差 1 m——rapier 只在 solver 路径里做 FK 传播,而
            // 睡眠/未唤醒的 island 不跑;land 场景的 wake_all 口径是对的)。
            if let Some(r) = side.robot.as_mut() {
                let y0 = GROUND_TOP + 1.0 + 2.0 * quad::L + quad::FOOT_R + 0.002;
                if let Some(rb) = side.world.bodies.get_mut(r.body) {
                    rb.set_translation(Vector::new(0.0, y0, 0.0), true);
                }
                wake_all(&mut side.world, r);
            }
        }
        _ => {}
    }

    let mut frozen: Option<FrozenState> = None;
    let mut obstacles = SnapObstacles {
        obstacles: Vec::new(),
    };
    let mut unfrozen = false;
    let mut new_robot: Option<quad::Robot> = None;
    let mut freeze_us = 0.0f32;
    let mut unfreeze_tick = 0usize;

    let mut err = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    let mut drift_max = 0.0f32;
    let mut upy_min = 1.0f32;
    let mut max_vy = 0.0f32;
    let mut link_max = 0.0f32;
    let mut body_max_rise = 0.0f32;
    let mut unfreeze_body_y = 0.0f32;
    let mut vfwd_sum = 0.0f64;
    let mut vfwd_n = 0u64;
    let mut fell_tick = None;
    let mut nan = false;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut cur_targets = [0.0f32; N_JOINTS];

    // falling 场景状态机:0 = 站稳中;1 = 悬空下落;2 = 冻结;3 = 唤醒后。
    let mut fall_stage = 0usize;

    for tick in 0..total_ticks {
        // 布景事件(hit/suprm 的方块在冻结后 100 拍触发)。
        if tick % 2 == 0 {
            cur_targets = targets_at(&gait, tick, walk0, walking);
        }

        // falling:站稳(60)→ 瞬移 0.5 m → 第 10 拍冻结(下落途中)。
        // snap 冻结期间机器人已移除,跳过(snap 冻结中的场景事件不再适用)。
        if cfg.scene == Scene::Falling {
            let Some(r) = side.robot.as_ref() else {
                // snap 冻结期间机器人已移除,布景事件不再适用。
                continue;
            };
            if fall_stage == 0 && tick == 60 {
                let y0 = GROUND_TOP + 0.5 + 2.0 * quad::L + quad::FOOT_R;
                if let Some(rb) = side.world.bodies.get_mut(r.body) {
                    let cur = *rb.rotation();
                    rb.set_position(
                        Pose {
                            translation: Vector::new(0.0, y0, 0.0),
                            rotation: cur,
                            ..Pose::IDENTITY
                        },
                        false,
                    );
                    rb.set_linvel(Vector::ZERO, false);
                    rb.set_angvel(Vector::ZERO, false);
                }
                wake_all(&mut side.world, r);
                fall_stage = 1;
            }
        }

        // 冻结触发。
        let freeze_now = if cfg.scene == Scene::Control {
            false
        } else if cfg.scene == Scene::Falling {
            fall_stage == 1 && tick == 60 + 10
        } else if cfg.scene == Scene::Pressed {
            // pressed:站稳后方块轻放背上(mass M3One 口径)+ 30 拍后冻结。
            tick == STAND_TICKS + 30
        } else if matches!(cfg.scene, Scene::HitWhileFrozen | Scene::SupportRemoved) {
            tick == STAND_TICKS
        } else {
            tick == freeze_tick
        };
        if freeze_now && !unfrozen && frozen.is_none() {
            // pressed:先放方块再冻结(冻结前 30 拍已放)。
            if cfg.scene == Scene::Pressed {
                let r = side.robot.as_ref().expect("alive");
                let bp = side.world.bodies.get(r.body).expect("alive").translation();
                let body_top = bp.y + quad::BODY_HALF[1];
                side.blocks.push(spawn_block(
                    &mut side.world,
                    Vector::new(bp.x, body_top + 0.5 + 0.002, bp.z),
                    Vector::ZERO,
                ));
            }
            let r = side.robot.as_ref().expect("alive");
            let st = capture(&side.world, r, &cur_targets);
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            obstacles = do_freeze(&mut side, cfg.scheme, &st);
            freeze_us += t0.elapsed().as_secs_f32() * 1e6;
            frozen = Some(st);
            // hit:冻结后 100 拍从 3 m 砸 1 kg 方块。
            if cfg.scene == Scene::HitWhileFrozen {
                let st = frozen.as_ref().unwrap();
                let body_pos = st.link_poses[st.link_poses.len() - 1].translation;
                side.blocks.push(spawn_block(
                    &mut side.world,
                    Vector::new(body_pos.x, body_pos.y + 0.5 + 3.0, body_pos.z),
                    Vector::ZERO,
                ));
            }
            if cfg.scene == Scene::SupportRemoved {
                // 冻结后 100 拍移除全堆(循环内触发)。
            }
            unfreeze_tick = tick + freeze_len;
        }

        // suprm:冻结后 100 拍移除脚下方块堆。
        if cfg.scene == Scene::SupportRemoved
            && frozen.is_some()
            && !unfrozen
            && tick == unfreeze_tick - freeze_len + 100
        {
            for h in side.blocks.drain(..) {
                side.world.remove_body(h);
            }
        }

        // 唤醒触发。
        if let (true, Some(st)) = (!unfrozen && tick == unfreeze_tick, frozen.as_ref()) {
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            let (nr, e) = do_unfreeze(&mut side, cfg.scheme, cfg.wake_vel, st, &obstacles);
            freeze_us += t0.elapsed().as_secs_f32() * 1e6;
            err = e;
            new_robot = nr;
            unfrozen = true;
            unfreeze_tick = tick;
            // 机身高度基点(判据 3:机身升高 ≤ 5 cm;snap 的新机器人优先——
            // snap 冻结时 side.robot 已 take)。
            unfreeze_body_y = new_robot
                .as_ref()
                .or(side.robot.as_ref())
                .map(|r| {
                    side.world
                        .bodies
                        .get(r.body)
                        .expect("alive")
                        .translation()
                        .y
                })
                .expect("robot after unfreeze");
        }

        // 目标写入(冻结期间写也没用——kin/fixed 的电机约束不求解;snap
        // 期间机器人不在;照写保持对照口径)。
        let active_robot: Option<&quad::Robot> = if unfrozen {
            new_robot.as_ref().or(side.robot.as_ref())
        } else {
            side.robot.as_ref()
        };
        if let (Some(r), false) = (active_robot, frozen.is_some() && !unfrozen) {
            if tick % 2 == 0 {
                write_targets(&mut side.world, r, &cur_targets);
            }
            // Stand 场景:醒着站立(冻结前每拍唤醒;冻结期间不唤醒)。
            if cfg.scene == Scene::Stand && !unfrozen && frozen.is_none() {
                wake_all(&mut side.world, r);
            }
            // 非冻结的对照 / 布景站立段保持醒着(pressed/pile 布景需要站立
            // 平衡;冻结期间不唤醒)。
            if matches!(
                cfg.scene,
                Scene::OnPile | Scene::Pressed | Scene::HitWhileFrozen | Scene::SupportRemoved
            ) && frozen.is_none()
            {
                wake_all(&mut side.world, r);
            }
        }

        side.world.step();

        // 统计。
        let frozen_now = frozen.is_some() && !unfrozen;
        let r_ref: Option<&quad::Robot> = if unfrozen {
            new_robot.as_ref().or(side.robot.as_ref())
        } else {
            side.robot.as_ref()
        };
        // 冻结期间位移(kin/fixed:连杆;snap:障碍物)。
        if frozen_now {
            let st = frozen.as_ref().unwrap();
            let d = frozen_drift(&side, cfg.scheme, st, &obstacles);
            drift_max = drift_max.max(d);
            // kin/fixed 期间机器人还在:哈希照记;snap 期间不在。
            if let Some(r) = r_ref {
                if cfg.scheme != Scheme::Snap {
                    let body = side.world.bodies.get(r.body).expect("alive");
                    let p = body.translation();
                    for v in [p.x, p.y, p.z] {
                        hash = hash_f32(hash, v);
                    }
                }
            }
            continue;
        }
        let Some(r) = r_ref else { continue };
        let body = side.world.bodies.get(r.body).expect("alive");
        let p = body.translation();
        if !p.x.is_finite() || !p.y.is_finite() {
            nan = true;
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
            hash = hash_f32(hash, read_angle(&side.world.multibody_joints, r, dof));
        }
        let upy = (rot * Vector::new(0.0, 1.0, 0.0)).y;
        let vy = body.linvel().y.abs();
        let mut tick_link_max = 0.0f32;
        for lh in r.links.iter().copied().chain(std::iter::once(r.body)) {
            let rb = side.world.bodies.get(lh).expect("alive");
            tick_link_max = tick_link_max.max(rb.linvel().length());
        }
        link_max = link_max.max(tick_link_max);
        if unfrozen && tick >= unfreeze_tick && tick < unfreeze_tick + POST_TICKS {
            upy_min = upy_min.min(upy);
            max_vy = max_vy.max(vy);
            body_max_rise = body_max_rise.max(p.y - unfreeze_body_y);
            if upy < 0.7
                && fell_tick.is_none()
                && !matches!(
                    cfg.scene,
                    Scene::Pressed | Scene::Falling | Scene::HitWhileFrozen | Scene::SupportRemoved
                )
            {
                fell_tick = Some(tick - unfreeze_tick);
            }
        }
        // falling:下落冻结组的唤醒发生在落地后(冻结 600 拍已落地),判据
        // 只报结果(判据 5 的"只报结果"类)。
        if walking && tick >= walk0 {
            let fwd = rot * Vector::new(1.0, 0.0, 0.0);
            let fh = Vector::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
            vfwd_sum += f64::from(body.linvel().dot(fh));
            vfwd_n += 1;
        }
    }

    let vfwd = (vfwd_sum / vfwd_n.max(1) as f64) as f32;
    Outcome {
        name: cfg.name.clone(),
        frozen_drift: drift_max,
        err_coord: err.0,
        err_pos: err.1,
        err_rot: err.2,
        err_vel: err.3,
        post_upy_min: upy_min,
        post_max_vy: max_vy,
        post_link_max: link_max,
        body_max_rise,
        vfwd,
        fell_tick,
        nan,
        panicked: false,
        panic_msg: String::new(),
        freeze_us,
        state_hash: hash,
    }
}

/// 8 只一起冻结、一起唤醒(kin 为主口径 + snap 对照;耗时)。
fn run_eight(cfg: &Cfg) -> Outcome {
    let _ = cfg;
    let gait = default_gait();
    let walk0 = STAND_TICKS;
    let freeze_tick = walk0 + 60;
    let freeze_len = FREEZE_SHORT;
    let total_ticks = STAND_TICKS + 600 + 600;

    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = production_params(1.0 / 60.0);
    let _ = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, GROUND_TOP - 0.5, 0.0)),
        ColliderBuilder::cuboid(400.0, 0.5, 400.0)
            .friction(quad::GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
    );
    let n = 8;
    let mut robots: Vec<quad::Robot> = Vec::new();
    for i in 0..n {
        let y0 = GROUND_TOP + 2.0 * quad::L + quad::FOOT_R + 0.002;
        robots.push(spawn_robot(
            &mut world,
            Vector::new(i as f32 * 2.5, y0, 0.0),
            K,
            quad::GROUND_FRICTION,
            DampMode::Scaled,
        ));
    }

    let mut frozen_states: Vec<Option<FrozenState>> = (0..n).map(|_| None).collect();
    let mut unfrozen = false;
    let mut freeze_us = 0.0f32;
    let mut upy_min = 1.0f32;
    let mut max_vy = 0.0f32;
    let mut link_max = 0.0f32;
    let mut fell_tick = None;
    let mut nan = false;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut cur_targets = [0.0f32; N_JOINTS];
    let unfreeze_tick = freeze_tick + freeze_len;

    for tick in 0..total_ticks {
        if tick % 2 == 0 {
            cur_targets = targets_at(&gait, tick, walk0, false);
        }
        if tick % 2 == 0 && frozen_states[0].is_none() {
            for r in &robots {
                write_targets(&mut world, r, &cur_targets);
            }
        }
        if tick == freeze_tick {
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            for (i, r) in robots.iter().enumerate() {
                let st = capture(&world, r, &cur_targets);
                frozen_states[i] = Some(st);
                // 只切根(KinRoot 语义):全连杆切型会踩 rapier
                // multibody.rs:1457 的非根 Dynamic 断言(见 do_freeze 注释)。
                if let Some(rb) = world.bodies.get_mut(r.body) {
                    rb.set_linvel(Vector::ZERO, false);
                    rb.set_angvel(Vector::ZERO, false);
                    rb.set_body_type(RigidBodyType::KinematicPositionBased, false);
                }
            }
            freeze_us = t0.elapsed().as_secs_f32() * 1e6 / n as f32;
        }
        if tick == unfreeze_tick {
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            for (i, r) in robots.iter().enumerate() {
                let handles: Vec<RigidBodyHandle> = r
                    .links
                    .iter()
                    .copied()
                    .chain(std::iter::once(r.body))
                    .collect();
                for h in handles {
                    if let Some(rb) = world.bodies.get_mut(h) {
                        rb.set_body_type(RigidBodyType::Dynamic, false);
                    }
                }
                let st = frozen_states[i].as_ref().unwrap();
                for (j, h) in r
                    .links
                    .iter()
                    .copied()
                    .chain(std::iter::once(r.body))
                    .enumerate()
                {
                    if let Some(rb) = world.bodies.get_mut(h) {
                        rb.set_linvel(st.link_linvels[j], false);
                        rb.set_angvel(st.link_angvels[j], false);
                    }
                }
                let (mb_index, _) = r.joints[0];
                if let Some(mb) = world.multibody_joints.get_multibody_mut(mb_index) {
                    mb.forward_kinematics(&world.bodies, true);
                    mb.update_rigid_bodies(&mut world.bodies, false);
                }
                wake_all(&mut world, r);
            }
            freeze_us += t0.elapsed().as_secs_f32() * 1e6 / n as f32;
            unfrozen = true;
        }
        if !unfrozen && frozen_states[0].is_none() && tick % 2 == 0 && tick < freeze_tick {
            // 站立唤醒(醒着站立口径)。
            for r in &robots {
                wake_all(&mut world, r);
            }
        }

        world.step();

        for (ri, r) in robots.iter().enumerate() {
            let body = world.bodies.get(r.body).expect("alive");
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
            }
            let rot = body.rotation();
            let upy = (rot * Vector::new(0.0, 1.0, 0.0)).y;
            let vy = body.linvel().y.abs();
            let mut tick_link_max = 0.0f32;
            for lh in r.links.iter().copied().chain(std::iter::once(r.body)) {
                let rb = world.bodies.get(lh).expect("alive");
                tick_link_max = tick_link_max.max(rb.linvel().length());
            }
            link_max = link_max.max(tick_link_max);
            if unfrozen && tick >= unfreeze_tick && tick < unfreeze_tick + POST_TICKS {
                upy_min = upy_min.min(upy);
                max_vy = max_vy.max(vy);
                if upy < 0.7 && fell_tick.is_none() {
                    fell_tick = Some(tick - unfreeze_tick);
                }
            }
        }
    }

    Outcome {
        name: cfg.name.clone(),
        frozen_drift: 0.0,
        err_coord: 0.0,
        err_pos: 0.0,
        err_rot: 0.0,
        err_vel: 0.0,
        post_upy_min: upy_min,
        post_max_vy: max_vy,
        post_link_max: link_max,
        body_max_rise: 0.0,
        vfwd: 0.0,
        fell_tick,
        nan,
        panicked: false,
        panic_msg: String::new(),
        freeze_us,
        state_hash: hash,
    }
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
        "name,frozen_drift_m,err_coord_rad,err_pos_m,err_rot_rad,err_vel,post_upy_min,\
         post_max_vy,post_link_max,body_max_rise,vfwd,fell_tick,nan,panicked,panic_msg,\
         freeze_us,state_hash\n",
    );
    let mut det_out = String::new();
    for cfg in &cfgs {
        let selected = if det_mode {
            cfg.name.contains("stand_kin_rst")
                || cfg.name.contains("walk_p06_snap_rst")
                || cfg.name.contains("walk_p06_kin_rst")
        } else if only.is_empty() {
            true
        } else {
            only.iter().any(|a| cfg.name.contains(a.as_str()))
        };
        if !selected {
            continue;
        }
        #[allow(clippy::disallowed_methods)]
        let t0 = Instant::now();
        let o = if cfg.eight {
            run_eight(cfg)
        } else {
            run_once(cfg).expect("run_once ok")
        };
        let line = outcome_csv(&o);
        println!("{line} | {:.1}s", t0.elapsed().as_secs_f32());
        out.push_str(&line);
        out.push('\n');
        if det_mode {
            let o2 = if cfg.eight {
                run_eight(cfg)
            } else {
                run_once(cfg).expect("run_once ok")
            };
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

fn build_matrix() -> Vec<Cfg> {
    let mut all: Vec<Cfg> = Vec::new();
    // 对照:不冻结(站立、行走 p06)。
    // 对照:不冻结(站立、行走 p06;场景 = Control,步行/站立由 walking 标志
    // 与冻结判定隔离——Control 永不冻结)。
    for (tag, walking_phase) in [("stand", None), ("walk_p06", Some(6))] {
        let _ = walking_phase;
        all.push(Cfg {
            name: format!("control_{tag}"),
            scene: Scene::Control,
            scheme: Scheme::Kin,
            wake_vel: WakeVel::Restore,
            freeze_ticks: 0,
            eight: false,
        });
    }
    // 醒着站立冻结:五方案 × 两唤醒速度(kin/fixed = 放行原文的全连杆口径,
    // 预期 panic——rapier multibody.rs:1457 断言非根连杆必须 Dynamic;
    // kinroot/fixedroot = 只切根的可行口径;snap = 移除重建)。
    for scheme in [
        Scheme::Kin,
        Scheme::Fixed,
        Scheme::KinRoot,
        Scheme::FixedRoot,
        Scheme::Snap,
    ] {
        for wv in [WakeVel::Restore, WakeVel::Zero] {
            all.push(Cfg {
                name: format!("stand_{}_{}", scheme.tag(), wv.tag()),
                scene: Scene::Stand,
                scheme,
                wake_vel: wv,
                freeze_ticks: 0,
                eight: false,
            });
        }
    }
    // 行走中冻结 12 相位:可行方案(kinroot/fixedroot/snap)= 12 × 4;
    // 全连杆口径(kin/fixed)只跑 p00 一相位作 panic 证据行。
    for phase in 0..12usize {
        for (scheme, wv) in [
            (Scheme::KinRoot, WakeVel::Restore),
            (Scheme::KinRoot, WakeVel::Zero),
            (Scheme::FixedRoot, WakeVel::Restore),
            (Scheme::Snap, WakeVel::Restore),
        ] {
            all.push(Cfg {
                name: format!("walk_p{phase:02}_{}_{}", scheme.tag(), wv.tag()),
                scene: Scene::WalkPhase { phase },
                scheme,
                wake_vel: wv,
                freeze_ticks: 0,
                eight: false,
            });
        }
    }
    // 全连杆口径的 panic 证据行(walk p00 一相位)。
    for (scheme, wv) in [
        (Scheme::Kin, WakeVel::Restore),
        (Scheme::Fixed, WakeVel::Restore),
    ] {
        all.push(Cfg {
            name: format!("walk_p00_{}_{}", scheme.tag(), wv.tag()),
            scene: Scene::WalkPhase { phase: 0 },
            scheme,
            wake_vel: wv,
            freeze_ticks: 0,
            eight: false,
        });
    }
    // 站在方块堆上:三可行方案 × 两唤醒速度。
    for scheme in [Scheme::KinRoot, Scheme::FixedRoot, Scheme::Snap] {
        for wv in [WakeVel::Restore, WakeVel::Zero] {
            all.push(Cfg {
                name: format!("pile_{}_{}", scheme.tag(), wv.tag()),
                scene: Scene::OnPile,
                scheme,
                wake_vel: wv,
                freeze_ticks: 0,
                eight: false,
            });
        }
    }
    // 被单格方块压着:三可行方案(Restore 口径;只报结果)。
    for scheme in [Scheme::KinRoot, Scheme::FixedRoot, Scheme::Snap] {
        all.push(Cfg {
            name: format!("pressed_{}_rst", scheme.tag()),
            scene: Scene::Pressed,
            scheme,
            wake_vel: WakeVel::Restore,
            freeze_ticks: 0,
            eight: false,
        });
    }
    // 离地 0.5 m 下落途中:三可行方案(只报结果)。
    for scheme in [Scheme::KinRoot, Scheme::FixedRoot, Scheme::Snap] {
        all.push(Cfg {
            name: format!("falling_{}_rst", scheme.tag()),
            scene: Scene::Falling,
            scheme,
            wake_vel: WakeVel::Restore,
            freeze_ticks: 0,
            eight: false,
        });
    }
    // 冻结期间方块砸中:三可行方案(只报结果)。
    for scheme in [Scheme::KinRoot, Scheme::FixedRoot, Scheme::Snap] {
        all.push(Cfg {
            name: format!("hit_{}_rst", scheme.tag()),
            scene: Scene::HitWhileFrozen,
            scheme,
            wake_vel: WakeVel::Restore,
            freeze_ticks: 0,
            eight: false,
        });
    }
    // 冻结期间脚下方块被移走:三可行方案(只报结果)。
    for scheme in [Scheme::KinRoot, Scheme::FixedRoot, Scheme::Snap] {
        all.push(Cfg {
            name: format!("suprm_{}_rst", scheme.tag()),
            scene: Scene::SupportRemoved,
            scheme,
            wake_vel: WakeVel::Restore,
            freeze_ticks: 0,
            eight: false,
        });
    }
    // 冻结时长两档:600(默认,上面全部)与 3600(stand/walk p06 × 三可行
    // 方案,Restore 口径)。
    for scheme in [Scheme::KinRoot, Scheme::FixedRoot, Scheme::Snap] {
        for (scene, tag) in [
            (Scene::Stand, "stand"),
            (Scene::WalkPhase { phase: 6 }, "walk_p06"),
        ] {
            all.push(Cfg {
                name: format!("{tag}_long_{}_rst", scheme.tag()),
                scene,
                scheme,
                wake_vel: WakeVel::Restore,
                freeze_ticks: FREEZE_LONG,
                eight: false,
            });
        }
    }
    // 8 只一起冻结、一起唤醒。
    all.push(Cfg {
        name: "eight_kin_rst".into(),
        scene: Scene::Eight,
        scheme: Scheme::Kin,
        wake_vel: WakeVel::Restore,
        freeze_ticks: 0,
        eight: true,
    });
    all
}
