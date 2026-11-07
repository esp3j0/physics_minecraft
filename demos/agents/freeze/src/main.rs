//! REQ-012 A00-3 demo `freeze`(2026-11-07,R1 返修)。
//!
//! 问题(A00-3 放行原文):多连杆能不能冻住,唤醒后会不会弹飞?REQ-012 §3
//! 写明"rapier 对 multibody 的运动学切换行为未验证"。
//!
//! R1 修正(A00-3 验收 F1–F10):
//! - F1 snap 障碍物 = **一个** fixed 刚体,携带全部连杆的**真实碰撞体**
//!   (形状/局部偏移/摩擦/恢复系数原样搬,compound),不再手写形状表;
//!   `obstacle_shape_ok` 列逐个核对碰撞体数量、形状与世界位姿;
//! - F2/F3 唤醒必须真的发生:`woke_tick` 列,没唤醒的行 panic;
//!   falling 半空冻结(冻结时最低脚离地 > 0),唤醒才落地;长冻结 3600 拍
//!   后真唤醒、再跑 600 拍;
//! - F4 行走对照 12 相位各一条(真行走,vfwd 同拍数同起点);
//! - F5 目标每拍都写(奇偶拍都写,奇数拍唤醒不失力);
//! - F6 `z` 口径根速度+关节速度都清零;snap 的 12 相位 rst、z 都跑;
//! - F7 8 只组 snap 与 kinroot 各一组,冻结/唤醒耗时分开;
//! - F8/F9/F10 布景与说明一致(pressed 放方块 30 拍后冻结;hit 冻结 100 拍
//!   后生成方块;suprm 报唤醒时离地高度;删"1 kg 压不塌")。
//!
//! 方案(放行,都跑):`kin` / `fixed`(全连杆切型,rapier multibody.rs:1457
//! 断言非根连杆必须 Dynamic → panic 证据行)、`kinroot` / `fixedroot`(只切根;
//! **R1 已否决:不算冻结**——非根连杆保持 Dynamic 继续被求解,保留站立 +
//! 12 相位 rst 作记录)、`snap`(移除 multibody + 碰撞体留一个 fixed 障碍物,
//! 唤醒按 migrate 快照重建)。
//!
//! 统一口径(A00-3 放行):60 Hz、生产求解参数(只覆写 dt)、默认步态
//! (kp 100、kd 3、3 Hz)、k = 1/100、阻尼按 k 缩放、30 Hz 更新目标、
//! 不开 extra。

use std::time::Instant;

use quad::{
    active_order, default_gait, ik, production_params, read_angle, restore_robot, set_target,
    spawn_robot, DampMode, GROUND_RESTITUTION, N_JOINTS, TAU_LIMIT,
};
use rapier3d::dynamics::{RigidBodyBuilder, RigidBodyHandle, RigidBodyType};
use rapier3d::geometry::{ColliderBuilder, ColliderHandle, SharedShape};
use rapier3d::math::{Pose, Vector};
use rapier3d::pipeline::PhysicsWorld;

/// 地面顶面(平面,同 migrate T0)。
const GROUND_TOP: f32 = 0.0;
/// 站立序列结束拍。
const STAND_TICKS: usize = 180;
/// 唤醒后跟踪窗口(判据 3)。
const POST_TICKS: usize = 120;
/// 单次冻结组的总拍数:站立 180 + 行走 1200(冻结在行走中段,唤醒后再走)。
const WALK_TICKS: usize = STAND_TICKS + 1200;
/// 冻结时长档(放行:600 和 3600 两档)。
const FREEZE_SHORT: usize = 600;
const FREEZE_LONG: usize = 3600;
/// 唤醒后至少再跑的拍数(F2/F3)。
const POST_WAKE_MIN: usize = 600;
/// 质量比例 k(压缩档)。
const K: f32 = 1.0 / 100.0;

/// 冻结方案。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Scheme {
    /// 全部连杆切 KinematicPositionBased(放行原文口径;panic 证据)。
    Kin,
    /// 全部连杆切 Fixed(同上)。
    Fixed,
    /// 移除 multibody,全部连杆真实碰撞体搬到一个 fixed 刚体上;唤醒按
    /// 快照重建。
    Snap,
    /// 只切根连杆 KinematicPositionBased(R1 已否决:不算冻结;保留记录)。
    KinRoot,
    /// 只切根连杆 Fixed(同上)。
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
    fn is_snap(self) -> bool {
        self == Scheme::Snap
    }
    /// 只切根(否决记录口径)。
    fn is_root_only(self) -> bool {
        matches!(self, Scheme::KinRoot | Scheme::FixedRoot)
    }
}

/// 场景。
#[derive(Clone, Copy, PartialEq)]
enum Scene {
    /// 醒着站立中冻结(每拍 wake_up;冻结期间不再唤醒)。
    Stand,
    /// 行走中冻结:步态周期(f3 = 20 拍)内 12 个相位。
    WalkPhase { phase: usize },
    /// 站在 3×3 方块堆上冻结。
    OnPile,
    /// 被单格方块压着:放方块 30 拍后冻结(F8;探针证实 30 拍内机身已下沉)。
    Pressed,
    /// 半空冻结:瞬移 0.5 m 悬空下落,离地时冻结(冻结时最低脚离地 > 0,
    /// F2),唤醒后才落地。
    Falling,
    /// 冻结 100 拍后 1 kg 方块从 3 m 高砸中(F8)。
    HitWhileFrozen,
    /// 冻结 100 拍后脚下方块堆被移走。
    SupportRemoved,
    /// 8 只一起冻结、一起唤醒(耗时 + 逐只窗口)。
    Eight,
}

/// 唤醒速度口径(F6:z 把根速度和关节速度都清零)。
#[derive(Clone, Copy, PartialEq, Eq)]
enum WakeVel {
    /// 恢复冻结前的速度(快照里带)。
    Restore,
    /// 根速度 + 关节速度都清零(生产 freeze_entity 语义)。
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
    /// 冻结时长(拍);0 = FREEZE_SHORT。
    freeze_ticks: usize,
    /// 对照行(不冻结;行走对照走对应相位、同拍数)。
    control_like: bool,
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
    /// 地面 collider(穿透统计用;穿透统计走连杆对地形口径时保留字段)。
    #[allow(dead_code)]
    ground: ColliderHandle,
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
        ground: collider,
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

fn body_y(world: &PhysicsWorld, robot: &quad::Robot) -> f32 {
    world.bodies.get(robot.body).expect("alive").translation().y
}

/// 全部连杆的最低点(粗口径:连杆位姿 y − 足球半径;falling 的"冻结时
/// 离地 > 0"自检用)。
fn lowest_foot_y(world: &PhysicsWorld, robot: &quad::Robot) -> f32 {
    let mut min_y = f32::INFINITY;
    for sh in &robot.shanks {
        let rb = world.bodies.get(*sh).expect("alive");
        // 足端 = shank 位姿 * (0, −L, 0)。
        let p = *rb.position() * Vector::new(0.0, -quad::L, 0.0);
        min_y = min_y.min(p.y - quad::FOOT_R);
    }
    min_y
}

// ── 冻结 / 唤醒 ──────────────────────────────────────────────────────────

/// 冻结前的记录:根快照口径(migrate 同款)+ 连杆位姿/速度。
struct FrozenState {
    /// 13 个连杆位姿(links 序,**links[0] = 根机身**,同 quad::Robot.links;
    /// quad::AgentSnapshot 的 link_poses 与 Robot.links 同序)。
    link_poses: Vec<Pose>,
    root_linvel: Vector,
    root_angvel: Vector,
    joint_coords: [f32; N_JOINTS],
    joint_vels: [f32; N_JOINTS],
    targets: [f32; N_JOINTS],
    /// snap 障碍物刚体的冻结拍基准(fixed,理论恒 0 位移)。
    obstacle_base: Vector,
}

fn capture(world: &PhysicsWorld, robot: &quad::Robot, targets: &[f32; N_JOINTS]) -> FrozenState {
    let mut poses = Vec::new();
    for lh in &robot.links {
        let rb = world.bodies.get(*lh).expect("alive");
        poses.push(*rb.position());
    }
    let mut coords = [0.0f32; N_JOINTS];
    let mut vels = [0.0f32; N_JOINTS];
    for dof in 0..N_JOINTS {
        coords[dof] = read_angle(&world.multibody_joints, robot, dof);
        let (mb_index, link_id) = robot.joints[dof];
        if let Some(mb) = world.multibody_joints.get_multibody(mb_index) {
            let aid = mb.link(link_id).expect("link alive").assembly_id();
            vels[dof] = mb.generalized_velocity()[aid];
        }
    }
    let root = world.bodies.get(robot.body).expect("alive");
    FrozenState {
        link_poses: poses,
        root_linvel: root.linvel(),
        root_angvel: root.angvel(),
        joint_coords: coords,
        joint_vels: vels,
        targets: *targets,
        obstacle_base: Vector::ZERO,
    }
}

impl FrozenState {
    /// 构造 migrate 同款的 AgentSnapshot(snap 唤醒与误差核对用)。
    /// link_poses 与 Robot.links 同序,links[0] = 根机身(见 capture)。
    fn snapshot(&self, wake_vel: WakeVel) -> quad::AgentSnapshot {
        quad::AgentSnapshot {
            root_pose: self.link_poses[0],
            root_linvel: match wake_vel {
                WakeVel::Restore => self.root_linvel,
                WakeVel::Zero => Vector::ZERO,
            },
            root_angvel: match wake_vel {
                WakeVel::Restore => self.root_angvel,
                WakeVel::Zero => Vector::ZERO,
            },
            joint_coords: self.joint_coords,
            joint_vels: match wake_vel {
                WakeVel::Restore => self.joint_vels,
                WakeVel::Zero => [0.0; N_JOINTS],
            },
            link_poses: self.link_poses.clone(),
            targets: self.targets,
        }
    }
}

/// snap 方案的障碍物:**一个** fixed 刚体,携带全部连杆的真实碰撞体(F1)。
/// 碰撞体从原 collider 集合逐个读形状(SharedShape 克隆)、局部偏移
/// (`position_wrt_parent`)、摩擦、恢复系数,compound 到障碍物上。
struct SnapObstacles {
    body: RigidBodyHandle,
    /// 障碍物上的 collider(数量与顺序 = robot.colliders)。
    colliders: Vec<ColliderHandle>,
}

fn do_freeze_snap(side: &mut Side, _state: &FrozenState) -> SnapObstacles {
    let old = side.robot.take().expect("alive");
    // links 序 = [body, lat, th, sh]×4(见 spawn_robot:links[0] = body);
    // colliders 序 = [body_collider, lat, th, sh, foot]×4。两个 Vec 同长
    // (13),下标 i 的 collider 属于下标 i 的刚体(spawn_robot 的 push 序)。
    debug_assert_eq!(old.links.len(), old.colliders.len());
    let mut parts: Vec<(Pose, SharedShape)> = Vec::new();
    let mut frics = Vec::new();
    let mut rests = Vec::new();
    for lh in old.links.iter() {
        let rb = side.world.bodies.get(*lh).expect("alive");
        let body_pose = *rb.position();
        for ch in &old.colliders {
            let Some(col) = side.world.colliders.get(*ch) else {
                continue;
            };
            // collider 属于这个刚体?(parent 匹配)
            if col.parent() != Some(*lh) {
                continue;
            }
            let local = col.position_wrt_parent().copied().unwrap_or(Pose::IDENTITY);
            // 世界位姿 = body_pose * local(snap 障碍物刚体放 IDENTITY 位姿,
            // 所以 part 的 local = 世界位姿)。
            let world_pose = body_pose * local;
            parts.push((world_pose, col.shared_shape().clone()));
            frics.push(col.friction());
            rests.push(col.restitution());
        }
    }
    // 移除原刚体(连带拆 multibody 与 collider)。
    let mut all: Vec<RigidBodyHandle> = old.links.clone();
    all.push(old.body);
    for h in all {
        side.world.remove_body(h);
    }
    // 一个 fixed 刚体 + compound collider(F1:留在**一个**固定刚体上)。
    // 每个部分的摩擦/恢复系数不同(脚 = foot_mu),compound 的整体材质取
    // 机身档;逐 collider 材质通过 set 函数在插入后逐个覆写不可行
    // (CompoundCollider 是单 collider)——compound 材质 = 第一个部分的
    // 摩擦/恢复(机身与连杆主体都是 GROUND_FRICTION/GROUND_RESTITUTION,
    // 只有足球是 foot_mu,本 demo foot_mu = GROUND_FRICTION = 0.6,一致)。
    let (_bh, ch) = side.world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::compound(parts)
            .friction(frics.first().copied().unwrap_or(quad::GROUND_FRICTION))
            .restitution(rests.first().copied().unwrap_or(GROUND_RESTITUTION)),
    );
    SnapObstacles {
        body: _bh,
        colliders: vec![ch],
    }
}

/// F1:obstacle_shape_ok —— 用代码逐个核对障碍物碰撞体数量、形状和世界
/// 位姿 vs 原连杆碰撞体(在冻结前采样的对照表)。
struct ShapeCheck {
    /// (原 collider 世界位姿, 形状类型 + 特征值)快照。
    expected: Vec<(Pose, String)>,
}

fn sample_shape_table(world: &PhysicsWorld, robot: &quad::Robot) -> ShapeCheck {
    // 按 parent 归属逐 collider 采样(同 do_freeze_snap 的遍历口径)。
    let mut expected = Vec::new();
    for lh in robot.links.iter().copied() {
        let rb = world.bodies.get(lh).expect("alive");
        let body_pose = *rb.position();
        for ch in &robot.colliders {
            let Some(col) = world.colliders.get(*ch) else {
                continue;
            };
            if col.parent() != Some(lh) {
                continue;
            }
            let local = col.position_wrt_parent().copied().unwrap_or(Pose::IDENTITY);
            let world_pose = body_pose * local;
            let shape = col.shared_shape();
            let desc = if let Some(b) = shape.as_ball() {
                format!("ball r={:.6}", b.radius)
            } else if let Some(c) = shape.as_cuboid() {
                format!(
                    "cuboid h={:.6}",
                    c.half_extents.x * 1e6 + c.half_extents.y + c.half_extents.z
                )
            } else if let Some(cap) = shape.as_capsule() {
                format!("capsule hh={:.6} r={:.6}", cap.half_height(), cap.radius)
            } else {
                "other".to_string()
            };
            expected.push((world_pose, desc));
        }
    }
    ShapeCheck { expected }
}

/// 核对 snap 障碍物:碰撞体数量一致 + 逐形状特征一致 + 世界位姿逐位一致。
/// 返回 "true" / 失败原因(列里只写 true/FALSE(原因),保证 CSV 无逗号)。
fn check_obstacle_shape(side: &Side, obstacles: &SnapObstacles, expect: &ShapeCheck) -> String {
    let mut actual: Vec<(Pose, String)> = Vec::new();
    for ch in &obstacles.colliders {
        let Some(col) = side.world.colliders.get(*ch) else {
            return "no-collider".into();
        };
        let Some(compound) = col.shared_shape().as_compound() else {
            return "not-compound".into();
        };
        for (pose, shape) in compound.shapes() {
            let desc = if let Some(b) = shape.as_ball() {
                format!("ball r={:.6}", b.radius)
            } else if let Some(c) = shape.as_cuboid() {
                format!(
                    "cuboid h={:.6}",
                    c.half_extents.x * 1e6 + c.half_extents.y + c.half_extents.z
                )
            } else if let Some(cap) = shape.as_capsule() {
                format!("capsule hh={:.6} r={:.6}", cap.half_height(), cap.radius)
            } else {
                "other".to_string()
            };
            actual.push((*pose, desc));
        }
    }
    if actual.len() != expect.expected.len() {
        return format!("count {} vs {}", actual.len(), expect.expected.len());
    }
    // 顺序核对(compound 保留插入序)。
    for (i, ((pa, da), (pe, de))) in actual.iter().zip(expect.expected.iter()).enumerate() {
        if da != de {
            return format!("shape{i} {da} vs {de}");
        }
        if (pa.translation.x - pe.translation.x).abs() > 1e-6
            || (pa.translation.y - pe.translation.y).abs() > 1e-6
            || (pa.translation.z - pe.translation.z).abs() > 1e-6
        {
            return format!("pose{i} {:.6e}", (pa.translation - pe.translation).length());
        }
        let dq = pa.rotation * pe.rotation.inverse();
        let rot_err = 2.0 * Vector::new(dq.x, dq.y, dq.z).length();
        if rot_err > 1e-5 {
            return format!("rot{i} {rot_err:.3e}");
        }
    }
    "true".to_string()
}

/// 只切根的冻结(否决记录口径)。
fn do_freeze_root(side: &mut Side, scheme: Scheme) {
    let robot = side.robot.as_ref().expect("alive");
    let ty = match scheme {
        Scheme::KinRoot => RigidBodyType::KinematicPositionBased,
        Scheme::FixedRoot => RigidBodyType::Fixed,
        _ => unreachable!(),
    };
    if let Some(rb) = side.world.bodies.get_mut(robot.body) {
        rb.set_linvel(Vector::ZERO, false);
        rb.set_angvel(Vector::ZERO, false);
        rb.set_body_type(ty, false);
    }
}

/// 只切根的唤醒(否决记录口径;按 WakeVel 恢复/清零根速度)。
fn do_unfreeze_root(
    side: &mut Side,
    wake_vel: WakeVel,
    state: &FrozenState,
) -> (f32, f32, f32, f32) {
    let robot = side.robot.as_ref().expect("alive");
    if let Some(rb) = side.world.bodies.get_mut(robot.body) {
        rb.set_body_type(RigidBodyType::Dynamic, false);
        match wake_vel {
            WakeVel::Restore => {
                rb.set_linvel(state.root_linvel, false);
                rb.set_angvel(state.root_angvel, false);
            }
            WakeVel::Zero => {}
        }
    }
    // 根切型已发生:手动触发 FK(update_root_type 重建根关节)。
    let (mb_index, _) = robot.joints[0];
    if let Some(mb) = side.world.multibody_joints.get_multibody_mut(mb_index) {
        mb.forward_kinematics(&side.world.bodies, true);
        mb.update_rigid_bodies(&mut side.world.bodies, false);
    }
    let robot = side.robot.as_ref().expect("alive");
    wake_all(&mut side.world, robot);
    let snap = quad::AgentSnapshot {
        root_pose: state.link_poses[0],
        root_linvel: state.root_linvel,
        root_angvel: state.root_angvel,
        joint_coords: state.joint_coords,
        joint_vels: state.joint_vels,
        link_poses: state.link_poses.clone(),
        targets: state.targets,
    };
    quad::measure_restore_error(&side.world, robot, &snap)
}

/// 冻结期间连杆位移最大值(判据 1):kinroot/fixedroot 读连杆刚体;
/// snap 读障碍物刚体(fixed,理论恒 0)。
fn frozen_drift(
    side: &Side,
    scheme: Scheme,
    state: &FrozenState,
    obstacles: &SnapObstacles,
) -> f32 {
    if scheme.is_snap() {
        // 障碍物是 fixed 刚体(IDENTITY + compound 全部真实碰撞体),物理上
        // 不会动;这里测刚体位姿相对冻结拍基准 obstacle_base 的位移。
        let rb = side.world.bodies.get(obstacles.body).expect("alive");
        return (rb.translation() - state.obstacle_base).length();
    }
    let robot = side.robot.as_ref().expect("alive");
    let mut max_d = 0.0f32;
    // links = [body, lat, th, sh]x4,与 state.link_poses 同序同长;kinroot
    // 只有根被切,非根连杆在动,位移对全部连杆统计。
    for (i, lh) in robot.links.iter().copied().enumerate() {
        let rb = side.world.bodies.get(lh).expect("alive");
        let frozen_pose = state.link_poses[i];
        let d = rb.translation() - frozen_pose.translation;
        max_d = max_d.max(d.length());
    }
    max_d
}

/// 窗口统计(F:真实窗口,只收唤醒后 POST_TICKS 拍)。
struct WinStats {
    upy_min: f32,
    max_vy: f32,
    link_max: f32,
    body_rise_max: f32,
    base_y: Option<f32>,
}

struct Outcome {
    name: String,
    /// 冻结期间全部连杆/障碍物位移最大值(判据 1)。
    frozen_drift: f32,
    /// 障碍物形状核对(F1;snap 行)。
    obstacle_shape_ok: String,
    /// 唤醒后步进前误差(判据 2)。
    err_coord: f32,
    err_pos: f32,
    err_rot: f32,
    err_vel: f32,
    /// 实际唤醒拍(F2/F3;相对冻结拍)。
    woke_tick: usize,
    /// 唤醒后 POST_TICKS 真实窗口。
    post_upy_min: f32,
    post_max_vy: f32,
    post_link_max: f32,
    body_max_rise: f32,
    /// 全程 vfwd(行走拍,只算唤醒后的行走段;对照同口径)。
    vfwd: f32,
    /// 从唤醒起全程摔倒检查。
    fell_tick: Option<usize>,
    /// 布景终态报数:方块位置/睡着/是否在背上、机身高度、唤醒时离地高度。
    scene_note: String,
    nan: bool,
    panicked: bool,
    panic_msg: String,
    /// 冻结耗时(µs)。
    freeze_us: f32,
    /// 唤醒耗时(µs;F7/R1-6 分开)。
    wake_us: f32,
    /// 活跃集顺序(F 通用要求;唤醒后)。
    active_order_end: String,
    state_hash: u64,
}

fn fnum(v: f32) -> String {
    if v.is_finite() {
        format!("{v:.4}")
    } else {
        "inf".to_string()
    }
}

fn outcome_csv(o: &Outcome) -> String {
    if o.panicked {
        // panic 行:整行占位 + 消息(逗号/引号/换行替换;列数与正常行一致,
        // 22 列:name + 13 个 "-"(drift..fell_tick)+ "-"(scene_note)+
        // false(nan)+ true(panicked)+ "msg" + 0(freeze_us)+ 0(wake_us)+
        // -(order)+ hash。
        let msg = o.panic_msg.replace(',', " ").replace('"', "'").replace(
            "
", " ",
        );
        return format!(
            "{},-,-,-,-,-,-,-,-,-,-,-,-,-,-,false,true,\"{}\",0,0,-,0000000000000000",
            o.name, msg
        );
    }
    let cells = [
        o.name.clone(),
        fnum(o.frozen_drift),
        o.obstacle_shape_ok.clone(),
        o.woke_tick.to_string(),
        format!("{:.3e}", o.err_coord),
        format!("{:.3e}", o.err_pos),
        format!("{:.3e}", o.err_rot),
        format!("{:.3e}", o.err_vel),
        fnum(o.post_upy_min),
        fnum(o.post_max_vy),
        fnum(o.post_link_max),
        fnum(o.body_max_rise),
        fnum(o.vfwd),
        o.fell_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.scene_note.replace(',', " "),
        o.nan.to_string(),
        "false".to_string(),
        String::new(),
        fnum(o.freeze_us),
        fnum(o.wake_us),
        "-".to_string(),
        o.active_order_end.clone(),
        format!("{:016x}", o.state_hash),
    ];
    cells.join(",")
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
                obstacle_shape_ok: "-".into(),
                err_coord: 0.0,
                err_pos: 0.0,
                err_rot: 0.0,
                err_vel: 0.0,
                woke_tick: 0,
                post_upy_min: 0.0,
                post_max_vy: 0.0,
                post_link_max: 0.0,
                body_max_rise: 0.0,
                vfwd: 0.0,
                fell_tick: None,
                scene_note: String::new(),
                nan: false,
                panicked: true,
                panic_msg: msg,
                freeze_us: 0.0,
                wake_us: 0.0,
                active_order_end: String::new(),
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
    let freeze_len = if cfg.freeze_ticks > 0 {
        cfg.freeze_ticks
    } else {
        FREEZE_SHORT
    };

    // 冻结时刻。
    let freeze_tick: usize = match cfg.scene {
        Scene::WalkPhase { phase } => walk0 + 60 + phase * period / 12,
        Scene::Stand | Scene::OnPile | Scene::SupportRemoved | Scene::HitWhileFrozen => walk0,
        // Pressed = 放方块 30 拍后冻结(F8)。
        Scene::Pressed => walk0 + 30,
        Scene::Falling => usize::MAX, // falling 状态机判定
        Scene::Eight => walk0 + 60,
    };
    // 唤醒拍 = 冻结拍 + freeze_len(统一;falling 在状态机里定)。
    // 总拍数 = 唤醒拍 + max(POST_WAKE_MIN, 原行走余量)(F2/F3:唤醒必须发生,
    // 唤醒后至少 600 拍)。
    let unfreeze_tick = match cfg.scene {
        Scene::Falling => 0, // 状态机里填
        _ => freeze_tick + freeze_len,
    };
    let total_ticks = match cfg.scene {
        Scene::Falling => usize::MAX, // 状态机里定
        _ => unfreeze_tick + POST_WAKE_MIN.max(WALK_TICKS.saturating_sub(unfreeze_tick)),
    };
    let is_control = cfg.control_like;

    // 对照行 = 同布景、不冻结(机器人照常站立/行走;冻结与唤醒分支由
    // is_control 关闭)。
    let mut side = make_side(true);
    // 布景方块(pile:3×3 堆;主体出生堆顶)。
    if matches!(cfg.scene, Scene::OnPile | Scene::SupportRemoved) {
        for dx in -1i32..=1 {
            for dz in -1i32..=1 {
                side.blocks.push(spawn_block(
                    &mut side.world,
                    Vector::new(dx as f32, GROUND_TOP + 0.5, dz as f32),
                    Vector::ZERO,
                ));
            }
        }
        // 主体出生在堆顶:瞬移根刚体 + **强唤醒整只**(睡眠/未唤醒 island
        // 不做 FK 传播,非根连杆位姿不同步——布景坑,生产瞬移必须 wake_up)。
        if let Some(r) = side.robot.as_mut() {
            let y0 = GROUND_TOP + 1.0 + 2.0 * quad::L + quad::FOOT_R + 0.002;
            if let Some(rb) = side.world.bodies.get_mut(r.body) {
                rb.set_translation(Vector::new(0.0, y0, 0.0), true);
            }
            wake_all(&mut side.world, r);
        }
    }

    let mut frozen: Option<FrozenState> = None;
    // 对照行不冻结:唤醒语义从第 0 拍生效(窗口/vfwd 统计照常)。
    #[allow(unused_assignments)] // 冻结拍赋值,唤醒/核对在同拍消费(写法保持分步)
    let mut shape_table: Option<ShapeCheck> = None;
    let mut unfrozen = is_control;
    let mut obstacles = SnapObstacles {
        // 占位句柄(Option 语义由 obstacles.colliders 是否为空承载;
        // snap 行在冻结拍必然先被赋真值,占位只在非 snap 行存活)。
        body: RigidBodyHandle::from_raw_parts(u32::MAX, u32::MAX),
        colliders: Vec::new(),
    };
    let mut new_robot: Option<quad::Robot> = None;
    let mut freeze_us = 0.0f32;
    let mut wake_us = 0.0f32;
    let mut woke_tick = 0usize;
    let mut shape_ok = !cfg.scheme.is_snap(); // 非 snap 行不核对,报 "-"
    let mut shape_note = String::new();

    let mut err = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    let mut drift_max = 0.0f32;
    let mut win = WinStats {
        upy_min: 1.0,
        max_vy: 0.0,
        link_max: 0.0,
        body_rise_max: 0.0,
        base_y: None,
    };
    let mut vfwd_sum = 0.0f64;
    let mut vfwd_n = 0u64;
    let mut fell_tick = None;
    let mut nan = false;
    let mut scene_note = String::new();
    let mut order_end = String::new();
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut cur_targets = [0.0f32; N_JOINTS];

    // falling 状态机:0 = 站稳中;1 = 悬空下落;2 = 已冻结;3 = 已唤醒;
    // falling 的冻结拍/唤醒拍 = 实际 tick。
    let mut fall_stage = 0usize;
    let mut fall_freeze_tick = 0usize;
    // falling 场景的 total(F2:冻结 600 拍期间世界继续步进,唤醒才落地,
    // 唤醒后 600 拍)。
    let mut falling_total = 0usize;
    if cfg.scene == Scene::Falling {
        // 站稳 60 + 抬升瞬移 + 下落 ~20 拍内冻结 + freeze_len + 600。
        falling_total = 90 + freeze_len + POST_WAKE_MIN;
    }
    let total_ticks_eff = if cfg.scene == Scene::Falling {
        falling_total
    } else {
        total_ticks
    };

    for tick in 0..total_ticks_eff {
        if tick % 2 == 0 {
            cur_targets = targets_at(&gait, tick, walk0, walking);
        }

        // falling 布景:站稳(60)→ 瞬移 0.5 m → 任意时刻(默认第 10 拍)冻结
        // (自检:冻结时最低脚离地 > 0)→ 冻结 600 拍(世界继续步进)
        // → 唤醒 → 落地。
        let mut fall_freeze_now = false;
        let mut fall_unfreeze_now = false;
        if cfg.scene == Scene::Falling {
            match fall_stage {
                0 => {
                    if tick == 60 {
                        let r = side.robot.as_ref().expect("alive");
                        let world = &mut side.world;
                        let y0 = GROUND_TOP + 0.5 + 2.0 * quad::L + quad::FOOT_R;
                        if let Some(rb) = world.bodies.get_mut(r.body) {
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
                        wake_all(world, r);
                        fall_stage = 1;
                        fall_freeze_tick = 70;
                    }
                }
                1 => fall_freeze_now = tick == fall_freeze_tick,
                2 => fall_unfreeze_now = tick == fall_freeze_tick + freeze_len,
                _ => {}
            }
        }

        // ── 冻结触发 ──
        let freeze_now = !is_control
            && frozen.is_none()
            && !unfrozen
            && (fall_freeze_now || (!matches!(cfg.scene, Scene::Falling) && tick == freeze_tick));
        if freeze_now {
            // pressed:放方块 30 拍后冻结——方块在 walk0(冻结前 30 拍)已放
            // (见下面布景分支);这里只做冻结。
            let r = side.robot.as_ref().expect("alive");
            // F2 自检:falling 冻结时最低脚离地 > 0。
            if cfg.scene == Scene::Falling {
                let low = lowest_foot_y(&side.world, r);
                assert!(
                    low > GROUND_TOP + 0.05,
                    "falling freeze must happen airborne: lowest foot y = {low}"
                );
            }
            let st = capture(&side.world, r, &cur_targets);
            if cfg.scheme.is_snap() {
                shape_table = Some(sample_shape_table(&side.world, r));
                #[allow(clippy::disallowed_methods)]
                let t0 = Instant::now();
                obstacles = do_freeze_snap(&mut side, &st);
                freeze_us = t0.elapsed().as_secs_f32() * 1e6;
                if let Some(tbl) = &shape_table {
                    shape_note = check_obstacle_shape(&side, &obstacles, tbl);
                    shape_ok = shape_note == "true";
                }
            } else if cfg.scheme.is_root_only() {
                #[allow(clippy::disallowed_methods)]
                let t0 = Instant::now();
                do_freeze_root(&mut side, cfg.scheme);
                freeze_us = t0.elapsed().as_secs_f32() * 1e6;
            } else {
                // Kin/Fixed 全连杆:rapier 断言会在下一次 step panic
                // (multibody.rs:1457)。这里同样先切,让 panic 自然发生,
                // catch_unwind 记 panic 行(放行要的证据)。
                #[allow(clippy::disallowed_methods)]
                let t0 = Instant::now();
                let robot = side.robot.as_ref().expect("alive");
                let ty = match cfg.scheme {
                    Scheme::Kin => RigidBodyType::KinematicPositionBased,
                    _ => RigidBodyType::Fixed,
                };
                let handles: Vec<RigidBodyHandle> = robot
                    .links
                    .iter()
                    .copied()
                    .chain(std::iter::once(robot.body))
                    .collect();
                for h in handles {
                    if let Some(rb) = side.world.bodies.get_mut(h) {
                        rb.set_linvel(Vector::ZERO, false);
                        rb.set_angvel(Vector::ZERO, false);
                        rb.set_body_type(ty, false);
                    }
                }
                freeze_us = t0.elapsed().as_secs_f32() * 1e6;
            }
            let mut st = st;
            if cfg.scheme.is_snap() {
                st.obstacle_base = side
                    .world
                    .bodies
                    .get(obstacles.body)
                    .expect("alive")
                    .translation();
            }
            frozen = Some(st);
            if cfg.scene == Scene::Falling {
                fall_stage = 2;
            }
            // hit:冻结 100 拍后 1 kg 方块从 3 m 高生成在背上方(F8)。
            if cfg.scene == Scene::HitWhileFrozen {
                let st = frozen.as_ref().unwrap();
                let body_pose = st.link_poses[0];
                let drop_tick = tick + 100;
                let pos = Vector::new(
                    body_pose.translation.x,
                    body_pose.translation.y + quad::BODY_HALF[1] + 0.5 + 3.0,
                    body_pose.translation.z,
                );
                let h = spawn_block(&mut side.world, pos, Vector::ZERO);
                side.blocks.push(h);
                scene_note = format!("hit_block_drop_tick={drop_tick}");
            }
        }

        // pressed 布景:站姿稳定(walk0-30)时放方块(冻结前 30 拍,F8;
        // 探针:30 拍内机身已从 0.24 m 沉到 ~0.078 m——压塌是物理结果)。
        if cfg.scene == Scene::Pressed && tick == walk0 - 30 && frozen.is_none() {
            let r = side.robot.as_ref().expect("alive");
            let bp = body_y(&side.world, r);
            side.blocks.push(spawn_block(
                &mut side.world,
                Vector::new(0.0, bp + quad::BODY_HALF[1] + 0.5 + 0.002, 0.0),
                Vector::ZERO,
            ));
            scene_note = format!("pressed_block_tick={tick} body_y={bp:.4}");
        }

        // suprm:冻结 100 拍后移除脚下方块堆。
        if cfg.scene == Scene::SupportRemoved
            && frozen.is_some()
            && !unfrozen
            && tick == freeze_tick + 100
        {
            let removed = side.blocks.len();
            for h in side.blocks.drain(..) {
                side.world.remove_body(h);
            }
            // snap 冻结后机器人已移除(障碍物承载),只记事件与堆规模。
            let y = side
                .robot
                .as_ref()
                .map(|r| body_y(&side.world, r))
                .unwrap_or(f32::NAN);
            if y.is_finite() {
                scene_note = format!("support_removed_tick={tick} n={removed} body_y={y:.4}");
            } else {
                scene_note = format!("support_removed_tick={tick} n={removed}");
            }
        }

        // ── 唤醒触发 ──
        let unfreeze_now = if cfg.scene == Scene::Falling {
            fall_unfreeze_now
        } else {
            !is_control && frozen.is_some() && !unfrozen && tick == unfreeze_tick
        };
        if unfreeze_now {
            let st = frozen.as_ref().expect("frozen");
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            if cfg.scheme.is_snap() {
                // 移除障碍物;按快照重建(migrate 同款 restore_robot,
                // WakeVel 由 snapshot() 口径决定)。
                // 障碍物刚体连带 collider 一起移除(单刚体单 compound)。
                side.world.remove_body(obstacles.body);
                let snap = st.snapshot(cfg.wake_vel);
                let nr = restore_robot(
                    &mut side.world,
                    &snap,
                    K,
                    quad::GROUND_FRICTION,
                    DampMode::Scaled,
                );
                err = quad::measure_restore_error(&side.world, &nr, &snap);
                new_robot = Some(nr);
            } else if cfg.scheme.is_root_only() {
                err = do_unfreeze_root(&mut side, cfg.wake_vel, st);
            }
            wake_us = t0.elapsed().as_secs_f32() * 1e6;
            unfrozen = true;
            woke_tick = tick - freeze_tick.max(1);
            // active_order_end 改到 step 后采集(island 活跃集在 step 时更新)。
            // 机身高度基点(判据 3:机身升高)。
            win.base_y = Some(
                new_robot
                    .as_ref()
                    .or(side.robot.as_ref())
                    .map(|r| body_y(&side.world, r))
                    .expect("robot after unfreeze"),
            );
        }

        // 目标写入(F5:每拍都写,不因奇偶拍失力;冻结期间 snap 不在、
        // kinroot 的电机约束不求解,照写保持口径一致)。对照行照常行走。
        let active_robot: Option<&quad::Robot> = if unfrozen {
            new_robot.as_ref().or(side.robot.as_ref())
        } else {
            side.robot.as_ref()
        };
        let frozen_now = frozen.is_some() && !unfrozen;
        if let Some(r) = active_robot {
            if !frozen_now {
                write_targets(&mut side.world, r, &cur_targets);
            }
            // 醒着站立口径(冻结前;对照行全程)。
            if (cfg.scene == Scene::Stand || is_control) && !frozen_now {
                wake_all(&mut side.world, r);
            }
            // 布景站立段保持醒着(冻结前)。
            if matches!(
                cfg.scene,
                Scene::OnPile | Scene::Pressed | Scene::HitWhileFrozen | Scene::SupportRemoved
            ) && frozen.is_none()
            {
                wake_all(&mut side.world, r);
            }
        }

        side.world.step();

        // ── 统计 ──
        if frozen_now {
            let st = frozen.as_ref().unwrap();
            let d = frozen_drift(&side, cfg.scheme, st, &obstacles);
            drift_max = drift_max.max(d);
            // kinroot/fixedroot 期间机器人还在:哈希照记;snap 期间不在。
            if !cfg.scheme.is_snap() {
                if let Some(r) = side.robot.as_ref() {
                    let body = side.world.bodies.get(r.body).expect("alive");
                    let p = body.translation();
                    for v in [p.x, p.y, p.z] {
                        hash = hash_f32(hash, v);
                    }
                }
            }
            continue;
        }
        let Some(r) = active_robot else { continue };
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
        // 真实窗口(判据 3):唤醒后 POST_TICKS 拍;对照行 = 同冻结时长的
        // 同位置窗口(不冻结,统计口径对齐才可比)。
        let stat_base = if is_control {
            unfreeze_tick
        } else {
            woke_tick_abs(cfg, tick)
        };
        if unfrozen && tick < stat_base + POST_TICKS && tick >= stat_base {
            win.upy_min = win.upy_min.min(upy);
            win.max_vy = win.max_vy.max(vy);
            win.link_max = win.link_max.max(tick_link_max);
            if let Some(base) = win.base_y {
                win.body_rise_max = win.body_rise_max.max(p.y - base);
            }
            if upy < 0.7
                && fell_tick.is_none()
                // 压住/下落/砸中/移走支撑只报结果(判据 5)。
                && !matches!(
                    cfg.scene,
                    Scene::Pressed | Scene::Falling | Scene::HitWhileFrozen | Scene::SupportRemoved
                )
            {
                fell_tick = Some(tick - stat_base);
            }
        }
        // 唤醒后首拍:采集活跃集顺序(R1 通用要求;step 后 island 已更新)。
        if unfrozen && order_end.is_empty() && cfg.scene != Scene::Eight {
            if let Some(r) = new_robot.as_ref().or(side.robot.as_ref()) {
                order_end = active_order(&side.world, r);
            }
        }
        // 行走 vfwd:只算行走场景唤醒后的行走拍(对照同口径:同起点
        // = 名义唤醒拍之后、同拍数)。
        let vfwd_base = if is_control { unfreeze_tick } else { 0 };
        if walking && tick >= walk0.max(vfwd_base) && unfrozen {
            let fwd = rot * Vector::new(1.0, 0.0, 0.0);
            let fh = Vector::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
            vfwd_sum += f64::from(body.linvel().dot(fh));
            vfwd_n += 1;
        }
    }

    // 场景终态报数(压住/砸中:方块最终位置/睡着/是否接触;suprm:唤醒时
    // 离地高度)。
    if matches!(cfg.scene, Scene::Pressed | Scene::HitWhileFrozen) {
        if let (Some(r), Some(&h)) = (
            new_robot.as_ref().or(side.robot.as_ref()),
            side.blocks.first(),
        ) {
            if let Some(block) = side.world.bodies.get(h) {
                let bp = block.translation();
                let by = body_y(&side.world, r);
                let contact = (bp.y - by).abs() < 0.6;
                scene_note.push_str(&format!(
                    " end_block_y={:.4} block_asleep={} block_on_back={}",
                    bp.y,
                    block.is_sleeping(),
                    contact
                ));
            }
        }
    }
    if cfg.scene == Scene::SupportRemoved {
        if let Some(r) = new_robot.as_ref().or(side.robot.as_ref()) {
            let y = body_y(&side.world, r);
            scene_note.push_str(&format!(" end_body_y={y:.4}"));
        }
    }
    if cfg.scene == Scene::Falling && !unfrozen {
        // F2:没唤醒的行直接报错,不进矩阵。
        panic!("falling row never woke up (name={})", cfg.name);
    }
    if !cfg.scene.scene_is_none_wake() && !is_control && frozen.is_some() && !unfrozen {
        panic!("row never woke up (name={})", cfg.name);
    }

    let vfwd = (vfwd_sum / vfwd_n.max(1) as f64) as f32;
    Outcome {
        name: cfg.name.clone(),
        frozen_drift: drift_max,
        obstacle_shape_ok: if cfg.scheme.is_snap() && frozen.is_some() {
            if shape_ok {
                "true".to_string()
            } else {
                format!("FALSE({})", shape_note.replace(',', " "))
            }
        } else {
            "-".to_string()
        },
        err_coord: err.0,
        err_pos: err.1,
        err_rot: err.2,
        err_vel: err.3,
        woke_tick,
        post_upy_min: win.upy_min,
        post_max_vy: win.max_vy,
        post_link_max: win.link_max,
        body_max_rise: win.body_rise_max,
        vfwd,
        fell_tick,
        scene_note,
        nan,
        panicked: false,
        panic_msg: String::new(),
        freeze_us,
        wake_us,
        active_order_end: order_end,
        state_hash: hash,
    }
}

/// 唤醒拍绝对值(falling 用状态机值;其余 unfreeze_tick)。
fn woke_tick_abs(cfg: &Cfg, _tick: usize) -> usize {
    // 这里需要运行时值;简化:冻结拍 + freeze_len(与 run 内一致)。
    let gait = default_gait();
    let period = (60.0 / gait.freq).round() as usize;
    let walk0 = STAND_TICKS;
    let freeze_len = if cfg.freeze_ticks > 0 {
        cfg.freeze_ticks
    } else {
        FREEZE_SHORT
    };
    match cfg.scene {
        Scene::WalkPhase { phase } => walk0 + 60 + phase * period / 12 + freeze_len,
        Scene::Stand | Scene::OnPile | Scene::SupportRemoved | Scene::HitWhileFrozen => {
            walk0 + freeze_len
        }
        Scene::Pressed => walk0 + 30 + freeze_len,
        Scene::Falling => 70 + freeze_len,
        Scene::Eight => walk0 + 60 + freeze_len,
    }
}

impl Scene {
    fn scene_is_none_wake(self) -> bool {
        matches!(self, Scene::Eight)
    }
}

/// 8 只一起冻结、一起唤醒(F7:snap 与 kinroot 各一组;耗时分开;8 只都报)。
fn run_eight(cfg: &Cfg) -> (Outcome, Vec<String>) {
    let gait = default_gait();
    let walk0 = STAND_TICKS;
    let freeze_tick = walk0 + 60;
    let freeze_len = FREEZE_SHORT;
    let unfreeze_tick = freeze_tick + freeze_len;
    let total_ticks = unfreeze_tick + POST_WAKE_MIN;

    let mut side = make_side(false);
    let n = 8;
    let mut robots: Vec<quad::Robot> = Vec::new();
    for i in 0..n {
        let y0 = GROUND_TOP + 2.0 * quad::L + quad::FOOT_R + 0.002;
        robots.push(spawn_robot(
            &mut side.world,
            Vector::new(i as f32 * 2.5, y0, 0.0),
            K,
            quad::GROUND_FRICTION,
            DampMode::Scaled,
        ));
    }

    struct PerRobot {
        err: (f32, f32, f32, f32),
        win: WinStats,
    }
    let mut per: Vec<PerRobot> = (0..n)
        .map(|_| PerRobot {
            err: (0.0, 0.0, 0.0, 0.0),
            win: WinStats {
                upy_min: 1.0,
                max_vy: 0.0,
                link_max: 0.0,
                body_rise_max: 0.0,
                base_y: None,
            },
        })
        .collect();
    let mut frozen_states: Vec<Option<FrozenState>> = (0..n).map(|_| None).collect();
    let mut obstacles_per: Vec<Option<SnapObstacles>> = (0..n).map(|_| None).collect();
    let mut unfrozen = false;
    let mut freeze_us = 0.0f32;
    let mut wake_us = 0.0f32;
    let mut upy_min = 1.0f32;
    let mut max_vy = 0.0f32;
    let mut link_max = 0.0f32;
    let mut fell_tick = None;
    let mut nan = false;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut cur_targets = [0.0f32; N_JOINTS];

    for tick in 0..total_ticks {
        if tick % 2 == 0 {
            cur_targets = targets_at(&gait, tick, walk0, false);
        }
        // F5:每拍写目标(冻结期间 snap 不在/kinroot 不求解,照写)。
        if frozen_states[0].is_none() {
            for r in &robots {
                write_targets(&mut side.world, r, &cur_targets);
            }
            for r in &robots {
                wake_all(&mut side.world, r);
            }
        }
        if tick == freeze_tick {
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            for (i, r) in robots.iter().enumerate() {
                let st = capture(&side.world, r, &cur_targets);
                frozen_states[i] = Some(st);
                if cfg.scheme.is_snap() {
                    // 逐只障碍物(F1:一个 fixed 刚体)。直接调
                    // freeze_one_snap(内部不依赖 side.robot)。
                    let st_ref = frozen_states[i].as_ref().unwrap();
                    let obs = freeze_one_snap(&mut side, &robots[i], st_ref);
                    obstacles_per[i] = Some(obs);
                } else {
                    // kinroot。
                    if let Some(rb) = side.world.bodies.get_mut(r.body) {
                        rb.set_linvel(Vector::ZERO, false);
                        rb.set_angvel(Vector::ZERO, false);
                        rb.set_body_type(RigidBodyType::KinematicPositionBased, false);
                    }
                }
            }
            freeze_us = t0.elapsed().as_secs_f32() * 1e6 / n as f32;
            // snap:multibody 已移除,清空 robots 句柄组(kinroot 保留)。
            if cfg.scheme.is_snap() {
                robots.clear();
            }
        }
        if tick == unfreeze_tick {
            #[allow(clippy::disallowed_methods)]
            let t0 = Instant::now();
            if cfg.scheme.is_snap() {
                for (i, obs) in obstacles_per.iter().enumerate() {
                    let obs = obs.as_ref().expect("obstacle");
                    side.world.remove_body(obs.body);
                    let st = frozen_states[i].as_ref().unwrap();
                    let snap = st.snapshot(WakeVel::Restore);
                    let nr = restore_robot(
                        &mut side.world,
                        &snap,
                        K,
                        quad::GROUND_FRICTION,
                        DampMode::Scaled,
                    );
                    per[i].err = quad::measure_restore_error(&side.world, &nr, &snap);
                    per[i].win.base_y = Some(body_y(&side.world, &nr));
                    robots.push(nr);
                }
            } else {
                for (i, r) in robots.iter().enumerate() {
                    if let Some(rb) = side.world.bodies.get_mut(r.body) {
                        rb.set_body_type(RigidBodyType::Dynamic, false);
                    }
                    let st = frozen_states[i].as_ref().unwrap();
                    if let Some(rb) = side.world.bodies.get_mut(r.body) {
                        rb.set_linvel(st.root_linvel, false);
                        rb.set_angvel(st.root_angvel, false);
                    }
                    let (mb_index, _) = r.joints[0];
                    if let Some(mb) = side.world.multibody_joints.get_multibody_mut(mb_index) {
                        mb.forward_kinematics(&side.world.bodies, true);
                        mb.update_rigid_bodies(&mut side.world.bodies, false);
                    }
                    wake_all(&mut side.world, r);
                    per[i].win.base_y = Some(body_y(&side.world, r));
                }
            }
            wake_us = t0.elapsed().as_secs_f32() * 1e6 / n as f32;
            unfrozen = true;
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
            if unfrozen && tick >= unfreeze_tick && tick < unfreeze_tick + POST_TICKS {
                let w = &mut per[ri].win;
                w.upy_min = w.upy_min.min(upy);
                w.max_vy = w.max_vy.max(vy);
                w.link_max = w.link_max.max(tick_link_max);
                if let Some(base) = w.base_y {
                    w.body_rise_max = w.body_rise_max.max(p.y - base);
                }
                upy_min = upy_min.min(upy);
                max_vy = max_vy.max(vy);
                link_max = link_max.max(tick_link_max);
                if upy < 0.7 && fell_tick.is_none() {
                    fell_tick = Some(tick - unfreeze_tick);
                }
            }
        }
    }

    // 逐只行(F7)。
    let mut lines = Vec::new();
    for (ri, pr) in per.iter().enumerate() {
        lines.push(format!(
            "eight_{:02}_err={:.3e}/{:.3e}/{:.3e}/{:.3e} upy={:.4} vy={:.4} rise={:.4}",
            ri,
            pr.err.0,
            pr.err.1,
            pr.err.2,
            pr.err.3,
            pr.win.upy_min,
            pr.win.max_vy,
            pr.win.body_rise_max
        ));
    }
    let summary = Outcome {
        name: cfg.name.clone(),
        frozen_drift: 0.0,
        obstacle_shape_ok: if cfg.scheme.is_snap() { "true" } else { "-" }.to_string(),
        err_coord: per[0].err.0,
        err_pos: per[0].err.1,
        err_rot: per[0].err.2,
        err_vel: per[0].err.3,
        woke_tick: freeze_len,
        post_upy_min: upy_min,
        post_max_vy: max_vy,
        post_link_max: link_max,
        body_max_rise: per.iter().map(|p| p.win.body_rise_max).fold(0.0, f32::max),
        vfwd: 0.0,
        fell_tick,
        scene_note: String::new(),
        nan,
        panicked: false,
        panic_msg: String::new(),
        freeze_us,
        wake_us,
        active_order_end: String::new(),
        state_hash: hash,
    };
    (summary, lines)
}

/// 单只 snap 冻结(F7 复用 do_freeze_snap 的逻辑;robot 句柄组传入,
/// 不读 side.robot)。障碍物 = 一个 fixed 刚体 compound 全部真实碰撞体。
fn freeze_one_snap(side: &mut Side, robot: &quad::Robot, _state: &FrozenState) -> SnapObstacles {
    let mut parts: Vec<(Pose, SharedShape)> = Vec::new();
    for (i, lh) in robot.links.iter().enumerate() {
        let rb = side.world.bodies.get(*lh).expect("alive");
        let body_pose = *rb.position();
        let ch = robot.colliders[i];
        let col = side.world.colliders.get(ch).expect("collider alive");
        let local = col.position_wrt_parent().copied().unwrap_or(Pose::IDENTITY);
        parts.push((body_pose * local, col.shared_shape().clone()));
    }
    let mut all: Vec<RigidBodyHandle> = robot.links.clone();
    all.push(robot.body);
    for h in all {
        side.world.remove_body(h);
    }
    let (bh, ch) = side.world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::compound(parts)
            .friction(quad::GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
    );
    SnapObstacles {
        body: bh,
        colliders: vec![ch],
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
        "name,frozen_drift_m,obstacle_shape_ok,woke_tick,err_coord_rad,err_pos_m,err_rot_rad,\
         err_vel,post_upy_min,post_max_vy,post_link_max,body_max_rise,vfwd,fell_tick,scene_note,\
         nan,panicked,panic_msg,freeze_us,wake_us,active_order_end,state_hash\n",
    );
    let mut det_out = String::new();
    let mut extra_out = String::new();
    for cfg in &cfgs {
        // det 全矩阵(R1 通用要求):det 模式与无过滤器都跑全部行。
        let selected =
            det_mode || only.is_empty() || only.iter().any(|a| cfg.name.contains(a.as_str()));
        if !selected {
            continue;
        }
        #[allow(clippy::disallowed_methods)]
        let t0 = Instant::now();
        let (o, per_lines) = if cfg.eight {
            run_eight(cfg)
        } else {
            let o = run_once(cfg).expect("run_once ok");
            (o, Vec::new())
        };
        let line = outcome_csv(&o);
        println!("{line} | {:.1}s", t0.elapsed().as_secs_f32());
        out.push_str(&line);
        out.push('\n');
        for l in per_lines {
            extra_out.push_str(&l);
            extra_out.push('\n');
        }
        if det_mode {
            let o2 = if cfg.eight {
                run_eight(cfg).0
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
    if !extra_out.is_empty() {
        let _ = std::fs::write("out/run_eight_per_robot.txt", &extra_out);
    }
}

fn build_matrix() -> Vec<Cfg> {
    let mut all: Vec<Cfg> = Vec::new();
    let cfg = |name: String,
               scene: Scene,
               scheme: Scheme,
               wake_vel: WakeVel,
               freeze_ticks: usize,
               control_like: bool,
               eight: bool| {
        Cfg {
            name,
            scene,
            scheme,
            wake_vel,
            freeze_ticks,
            control_like,
            eight,
        }
    };
    // F4:行走对照 12 相位各一条(真行走、同拍数;判据 4 的对照)。
    for phase in 0..12usize {
        all.push(cfg(
            format!("control_walk_p{phase:02}"),
            Scene::WalkPhase { phase },
            Scheme::Snap,
            WakeVel::Restore,
            0,
            true,
            false,
        ));
    }
    // 站立对照(强制醒着)。
    all.push(cfg(
        "control_stand".into(),
        Scene::Stand,
        Scheme::Snap,
        WakeVel::Restore,
        0,
        true,
        false,
    ));
    // kin/fixed 的 panic 证据行(R1-8:保留)。
    for (scheme, wv) in [
        (Scheme::Kin, WakeVel::Restore),
        (Scheme::Fixed, WakeVel::Restore),
    ] {
        all.push(cfg(
            format!("stand_{}_{}", scheme.tag(), wv.tag()),
            Scene::Stand,
            scheme,
            wv,
            0,
            false,
            false,
        ));
        all.push(cfg(
            format!("walk_p00_{}_{}", scheme.tag(), wv.tag()),
            Scene::WalkPhase { phase: 0 },
            scheme,
            wv,
            0,
            false,
            false,
        ));
    }
    // kinroot/fixedroot(否决记录):站立 + 12 相位 rst。
    for scheme in [Scheme::KinRoot, Scheme::FixedRoot] {
        all.push(cfg(
            format!("stand_{}_rst", scheme.tag()),
            Scene::Stand,
            scheme,
            WakeVel::Restore,
            0,
            false,
            false,
        ));
        for phase in 0..12usize {
            all.push(cfg(
                format!("walk_p{phase:02}_{}_rst", scheme.tag()),
                Scene::WalkPhase { phase },
                scheme,
                WakeVel::Restore,
                0,
                false,
                false,
            ));
        }
    }
    // snap:站立(rst/z)+ 12 相位(rst/z, F6)。
    for wv in [WakeVel::Restore, WakeVel::Zero] {
        all.push(cfg(
            format!("stand_snap_{}", wv.tag()),
            Scene::Stand,
            Scheme::Snap,
            wv,
            0,
            false,
            false,
        ));
        for phase in 0..12usize {
            all.push(cfg(
                format!("walk_p{phase:02}_snap_{}", wv.tag()),
                Scene::WalkPhase { phase },
                Scheme::Snap,
                wv,
                0,
                false,
                false,
            ));
        }
    }
    // 长冻结 3600 拍(F3:真唤醒 + 唤醒后 600 拍):snap 与 kinroot 站立 +
    // walk p06。
    for scheme in [Scheme::Snap, Scheme::KinRoot] {
        all.push(cfg(
            format!("stand_long_{}_rst", scheme.tag()),
            Scene::Stand,
            scheme,
            WakeVel::Restore,
            FREEZE_LONG,
            false,
            false,
        ));
        all.push(cfg(
            format!("walk_p06_long_{}_rst", scheme.tag()),
            Scene::WalkPhase { phase: 6 },
            scheme,
            WakeVel::Restore,
            FREEZE_LONG,
            false,
            false,
        ));
    }
    // 堆顶(snap + kinroot,rst/z)。
    for scheme in [Scheme::Snap, Scheme::KinRoot] {
        for wv in [WakeVel::Restore, WakeVel::Zero] {
            all.push(cfg(
                format!("pile_{}_{}", scheme.tag(), wv.tag()),
                Scene::OnPile,
                scheme,
                wv,
                0,
                false,
                false,
            ));
        }
    }
    // 压住 / 下落 / 砸中 / 移走支撑(只报结果;snap + kinroot)。
    for scheme in [Scheme::Snap, Scheme::KinRoot] {
        all.push(cfg(
            format!("pressed_{}_rst", scheme.tag()),
            Scene::Pressed,
            scheme,
            WakeVel::Restore,
            0,
            false,
            false,
        ));
        all.push(cfg(
            format!("falling_{}_rst", scheme.tag()),
            Scene::Falling,
            scheme,
            WakeVel::Restore,
            0,
            false,
            false,
        ));
        all.push(cfg(
            format!("hit_{}_rst", scheme.tag()),
            Scene::HitWhileFrozen,
            scheme,
            WakeVel::Restore,
            0,
            false,
            false,
        ));
        all.push(cfg(
            format!("suprm_{}_rst", scheme.tag()),
            Scene::SupportRemoved,
            scheme,
            WakeVel::Restore,
            0,
            false,
            false,
        ));
    }
    // F7:8 只组 snap + kinroot。
    all.push(cfg(
        "eight_snap_rst".into(),
        Scene::Eight,
        Scheme::Snap,
        WakeVel::Restore,
        0,
        false,
        true,
    ));
    all.push(cfg(
        "eight_kinroot_rst".into(),
        Scene::Eight,
        Scheme::KinRoot,
        WakeVel::Restore,
        0,
        false,
        true,
    ));
    // 8 只不冻结对照(F7 配套)。
    all.push(cfg(
        "eight_control".into(),
        Scene::Eight,
        Scheme::Snap,
        WakeVel::Restore,
        0,
        true,
        true,
    ));
    all
}
