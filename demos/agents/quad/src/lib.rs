//! REQ-012 A00 公共代码:Go2 尺度四足模型、逆解、开环步态、运行时电机目标、
//! 阻尼缩放。`gait` / `terrain` / `mass` 三个 demo 用路径依赖共用(A00-2 放行)。
//!
//! 生产参数口径与三个 rapier 坑(revolute 电机恒在 AngX 槽、目标变化要
//! `wake_up`、默认阻尼绝对常量 0.1)见 `demos/agents/gait/README.md` §13。
//!
//! 抽取纪律(放行原文):抽完后 `gait` 全矩阵的状态哈希必须与 9a7405a
//! 提交的逐行相同——所以本 crate 的 `spawn_robot` / `set_target` / `ik` /
//! `Gait` 的数值路径与 9a7405a 的 main.rs 逐行一致,只把类型与常量改为 pub。

use rapier3d::dynamics::{
    IntegrationParameters, JointAxis, MotorModel, Multibody, MultibodyIndex, MultibodyJointSet,
    MultibodyLink, RevoluteJointBuilder, RigidBodyBuilder, RigidBodyHandle,
};
use rapier3d::geometry::ColliderBuilder;
use rapier3d::math::{Pose, Vector};
use rapier3d::pipeline::PhysicsWorld;

// ── 模型常量(Go2 真实尺寸与质量,R1-1) ─────────────────────────────────

/// 大腿长 = 小腿长(Go2 URDF)。
pub const L: f32 = 0.213;
/// 足端球半径。
pub const FOOT_R: f32 = 0.022;
/// 髋锚点纵向偏移(机身 local x)。
pub const HIP_X: f32 = 0.1934;
/// 髋锚点横向偏移(机身 local z)。
pub const HIP_Z: f32 = 0.0465;
/// 大腿横向偏置(lat 关节 → hip pitch 关节,z 向)。
pub const THIGH_OFF: f32 = 0.0955;
/// 机身半尺寸(x 长 / y 厚 / z 宽)。
pub const BODY_HALF: [f32; 3] = [0.19, 0.057, 0.047];
/// 各连杆质量(kg):机身 / 髋 / 大腿 / 小腿 / 足端球。
pub const BODY_MASS: f32 = 6.921;
pub const HIP_MASS: f32 = 0.678;
pub const THIGH_MASS: f32 = 1.152;
pub const SHANK_MASS: f32 = 0.154;
pub const FOOT_MASS: f32 = 0.04;
/// 髋球半径(髋质量挂 ball,与总架构师探针一致)。
pub const HIP_BALL_R: f32 = 0.035;
/// 大腿 / 小腿胶囊半径。
pub const THIGH_RADIUS: f32 = 0.02;
pub const SHANK_RADIUS: f32 = 0.012;
/// 力矩上限(N·m,§9.1)。
pub const TAU_LIMIT: f32 = 23.7;
/// 地面摩擦 / 恢复(同 rebuild_chunk_collider;脚也用 0.6,R1-4)。
pub const GROUND_FRICTION: f32 = 0.6;
pub const GROUND_RESTITUTION: f32 = 0.0;
/// 关节总数 = 4 腿 × 3。
pub const N_JOINTS: usize = 12;
/// 站立段足端目标离髋高度(世界 m;R1-3:先降到 0.28 再渐入)。
pub const FOOT_HEIGHT: f32 = 0.28;
/// motors 数组下标:3D 布局 [LinX, LinY, LinZ, AngX, AngY, AngZ]。
pub const ANG_X: usize = 3;
/// 直腿(full stretch)足端离髋距离:2L − 1 mm,避免逆解在奇异点上。
pub const FULL_STRETCH_DY: f32 = -(2.0 * L - 0.001);
/// 默认步态组(A00-2 放行:kp 100、kd 3,f 3 Hz,步长 0.15,抬脚 0.06,
/// 脚摩擦 0.6,30 Hz 目标更新)。
pub const DEFAULT_KP: f32 = 100.0;
pub const DEFAULT_KD: f32 = 3.0;

// ── 世界 ────────────────────────────────────────────────────────────────

/// 生产参数的 IntegrationParameters(rapier 默认 + dt 覆写)。
pub fn production_params(dt: f32) -> IntegrationParameters {
    IntegrationParameters {
        dt,
        ..IntegrationParameters::default()
    }
}

/// 一只机器人:句柄 + 每关节的 (multibody, link) 写入地址 + 4 条小腿刚体。
pub struct Robot {
    pub body: RigidBodyHandle,
    /// 长度 12,顺序 = 腿序 [FL, FR, RL, RR] × [hip_lat, hip_pitch, knee]。
    pub joints: Vec<(MultibodyIndex, usize)>,
    /// 每腿小腿(shank)刚体句柄:足端世界坐标 = shank 位姿算(R1-6)。
    pub shanks: Vec<RigidBodyHandle>,
    /// 全部连杆刚体(含机身与髋):最大连杆速度 / 连杆穿透统计用。
    pub links: Vec<RigidBodyHandle>,
    /// 全部连杆的 collider 句柄(与 `links` 同序):接触对归属判定用
    /// (terrain/mass 的"对地穿透"只统计连杆侧)。
    pub colliders: Vec<rapier3d::geometry::ColliderHandle>,
    /// 每腿足端球 collider(4 个;terrain 窄口径"脚对地形"统计、
    /// mass M1Leg 首碰连杆判定用)。只记录,不参与数值路径。
    pub feet: Vec<rapier3d::geometry::ColliderHandle>,
}

/// 腿的锚点:机身 local 下的髋位置与横向符号。
pub fn leg_signs() -> [(f32, f32); 4] {
    [(1.0, -1.0), (1.0, 1.0), (-1.0, -1.0), (-1.0, 1.0)]
}

/// yaw 弧度 → 绕世界 Y 的旋转(机身 + 全部腿刚体出生时同向旋转;revolute
/// 轴是双方局部系,局部 anchor 不变,整只机器人即绕 Y 转过 yaw 的拷贝)。
pub fn yaw_quat(yaw: f32) -> rapier3d::math::Rotation {
    rapier3d::math::Rotation::from_rotation_y(yaw)
}

/// 建一只机器人:机身 + 4 腿 × 3 关节,连杆按"关节角全 0"的竖直链位姿。
/// 与总架构师探针同构(scratchpad r12a/probe-gait):碰撞体 `.mass()`、
/// 所有 joint `contacts_enabled(false)`、脚 = shank 末端的独立 ball。
/// `yaw` ≠ 0 时整只机器人绕世界 Y 旋转(关节局部系跟随刚体,不动)。
pub fn spawn_robot(
    world: &mut PhysicsWorld,
    origin: Vector,
    mass_scale: f32,
    foot_mu: f32,
    damp: DampMode,
) -> Robot {
    spawn_robot_yawed(world, origin, yaw_quat(0.0), mass_scale, foot_mu, damp)
}

/// 同 [`spawn_robot`],出生朝向可指定(terrain T3 斜穿 / 航向偏置)。
pub fn spawn_robot_yawed(
    world: &mut PhysicsWorld,
    origin: Vector,
    yaw: rapier3d::math::Rotation,
    mass_scale: f32,
    foot_mu: f32,
    damp: DampMode,
) -> Robot {
    let s = mass_scale;
    let body = world.insert_body(RigidBodyBuilder::dynamic().pose(Pose {
        rotation: yaw,
        translation: origin,
        ..Pose::IDENTITY
    }));
    let body_collider = world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(BODY_HALF[0], BODY_HALF[1], BODY_HALF[2])
            .mass(BODY_MASS * s)
            .friction(GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
        body,
        &mut world.bodies,
    );
    let mut colliders = vec![body_collider];
    let mut links = vec![body];

    let mut joints = Vec::new();
    let mut shanks = Vec::new();
    let mut feet = Vec::new();
    for (fx, fz) in leg_signs() {
        let anchor = Vector::new(fx * HIP_X, 0.0, fz * HIP_Z);
        // 髋/大腿/小腿出生位置按"关节角全 0"的竖直链计算(机身 local),
        // 再整体旋到 yaw 朝向:世界位姿 = origin + yaw·(local 偏移)。
        let hip_l = anchor;
        let lat_l = hip_l + Vector::new(0.0, 0.0, fz * THIGH_OFF / 2.0);
        let lat = world.insert_body(RigidBodyBuilder::dynamic().pose(Pose {
            rotation: yaw,
            translation: origin + yaw * lat_l,
            ..Pose::IDENTITY
        }));
        let lat_collider = world.colliders.insert_with_parent(
            ColliderBuilder::ball(HIP_BALL_R)
                .mass(HIP_MASS * s)
                .friction(GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
            lat,
            &mut world.bodies,
        );
        let lat_joint = RevoluteJointBuilder::new(Vector::new(1.0, 0.0, 0.0))
            .local_anchor1(anchor)
            .local_anchor2(Vector::new(0.0, 0.0, -fz * THIGH_OFF / 2.0))
            .contacts_enabled(false)
            .limits([-0.8, 0.8])
            .build();
        world
            .insert_multibody_joint(body, lat, lat_joint)
            .expect("lat joint");

        // 大腿:从髋外端竖直向下 L(capsule_y 半长)。
        let th_top_l = hip_l + Vector::new(0.0, 0.0, fz * THIGH_OFF);
        let th_l = th_top_l - Vector::new(0.0, L / 2.0, 0.0);
        let th = world.insert_body(RigidBodyBuilder::dynamic().pose(Pose {
            rotation: yaw,
            translation: origin + yaw * th_l,
            ..Pose::IDENTITY
        }));
        let th_collider = world.colliders.insert_with_parent(
            ColliderBuilder::capsule_y(L / 2.0, THIGH_RADIUS)
                .mass(THIGH_MASS * s)
                .friction(GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
            th,
            &mut world.bodies,
        );
        let hip_joint = RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0))
            .local_anchor1(Vector::new(0.0, 0.0, fz * THIGH_OFF / 2.0))
            .local_anchor2(Vector::new(0.0, L / 2.0, 0.0))
            .contacts_enabled(false)
            .limits([-1.6, 1.6])
            .build();
        world
            .insert_multibody_joint(lat, th, hip_joint)
            .expect("hip joint");

        // 小腿:从膝竖直向下 L;足端 = 末端独立 ball(摩擦 R1-4 默认同地面)。
        let sh_l = th_l - Vector::new(0.0, L, 0.0);
        let sh = world.insert_body(RigidBodyBuilder::dynamic().pose(Pose {
            rotation: yaw,
            translation: origin + yaw * sh_l,
            ..Pose::IDENTITY
        }));
        let sh_collider = world.colliders.insert_with_parent(
            ColliderBuilder::capsule_y(L / 2.0, SHANK_RADIUS)
                .mass(SHANK_MASS * s)
                .friction(GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
            sh,
            &mut world.bodies,
        );
        let foot_collider = world.colliders.insert_with_parent(
            ColliderBuilder::ball(FOOT_R)
                .translation(Vector::new(0.0, -L / 2.0, 0.0))
                .mass(FOOT_MASS * s)
                .friction(foot_mu)
                .restitution(GROUND_RESTITUTION),
            sh,
            &mut world.bodies,
        );
        let knee_joint = RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0))
            .local_anchor1(Vector::new(0.0, -L / 2.0, 0.0))
            .local_anchor2(Vector::new(0.0, L / 2.0, 0.0))
            .contacts_enabled(false)
            .limits([0.0, 2.6])
            .build();
        world
            .insert_multibody_joint(th, sh, knee_joint)
            .expect("knee joint");

        colliders.extend([lat_collider, th_collider, sh_collider, foot_collider]);
        links.extend([lat, th, sh]);
        feet.push(foot_collider);

        // 关节地址 = 第二个刚体的 link;三关节并进同一 multibody。
        for body_h in [lat, th, sh] {
            let lid = world
                .multibody_joints
                .rigid_body_link(body_h)
                .expect("link id");
            joints.push((lid.multibody, lid.id));
        }
        shanks.push(sh);
    }
    // 阻尼处理(R1-5):rapier 默认给根节点角自由度与每个关节设绝对常量
    // 0.1,不随质量缩放。三种口径:
    // - Scaled(默认):逐项 ×s,根节点平动的 0 天然保留;
    // - Unscaled:rapier 默认不动(×0.01 对照组,复现"D9 差异来自阻尼");
    // - Zero:全部清零。
    match damp {
        DampMode::Scaled => {
            let mi = joints[0].0;
            let mb = world.multibody_joints.get_multibody_mut(mi).unwrap();
            for d in mb.damping_mut().iter_mut() {
                *d *= s;
            }
        }
        DampMode::Zero => {
            let mi = joints[0].0;
            let mb = world.multibody_joints.get_multibody_mut(mi).unwrap();
            mb.damping_mut().fill(0.0);
        }
        DampMode::Unscaled => {}
    }
    Robot {
        body,
        joints,
        shanks,
        links,
        colliders,
        feet,
    }
}

/// 阻尼口径(每组参数;R1-5)。
#[derive(Clone, Copy, PartialEq)]
pub enum DampMode {
    /// 逐项 ×s(默认,×s 档 R1-5 的主口径)。
    Scaled,
    /// rapier 默认 0.1 不动(×0.01 不缩放阻尼对照组)。
    Unscaled,
    /// 全部清零。
    Zero,
}

/// 每关节的 motors 下标:一律 AngX。revolute 关节不管世界转轴是 x 还是
/// z,`RevoluteJoint::new(axis)` 都是把"主轴 = local X"用 complete_ang_frame
/// 转到给定世界轴,LOCKED_REVOLUTE_AXES 恒锁 AngY/AngZ、只放 AngX
/// (探针证据:motor_axes=100000、locked=110111 → AngZ 上的电机落在锁定位
/// 上,num_velocity_constraints=0,约束根本不建)。世界轴向由 joint frame 决定。
pub fn motor_index(_dof: usize) -> usize {
    ANG_X
}

/// 运行时写一个关节的目标(ForceBased PD + 力矩上限),不重建 multibody。
/// 新目标(与上次不同)时唤醒整个 multibody——生产的控制周期(D10)每 2 拍
/// 发一次目标,等价语义是"目标变化即唤醒",否则站立平衡后 island 睡着,
/// 电机约束不再被求解,起步目标永远不生效。
pub fn set_target(
    world: &mut PhysicsWorld,
    robot: &Robot,
    dof: usize,
    target: f32,
    kp: f32,
    kd: f32,
    tau_limit: f32,
) {
    let (mb_index, link_id) = robot.joints[dof];
    let mb: &mut Multibody = world
        .multibody_joints
        .get_multibody_mut(mb_index)
        .expect("multibody alive");
    let link: &mut MultibodyLink = mb.link_mut(link_id).expect("link alive");
    // motor_axes 位掩码必须含该轴,否则 solver 根本不建电机约束
    // (num_velocity_constraints 数 motor_bits);set_motor 负责置位。
    // revolute 的电机/限位恒在 AngX 槽(见 motor_index 注释)。
    let axis = JointAxis::AngX;
    let changed = (link.joint.data.motors[ANG_X].target_pos - target).abs() > 1e-6;
    link.joint
        .data
        .set_motor(axis, target, 0.0, kp, kd)
        .set_motor_model(axis, MotorModel::ForceBased);
    link.joint.data.motors[ANG_X].max_force = tau_limit;
    if changed {
        let handles: Vec<RigidBodyHandle> = mb
            .links()
            .map(|l| l.rigid_body_handle())
            .chain(std::iter::once(robot.body))
            .collect();
        for h in handles {
            if let Some(rb) = world.bodies.get_mut(h) {
                rb.wake_up(true);
            }
        }
    }
}

/// 按主体追加求解迭代(A00-2 R2 放行的 `extra` 变量;**R3 已判不采用**,
/// 见 mass README 隔离性一节):对主体全部连杆和机身调用
/// `RigidBody::set_additional_solver_iterations(n)`。R2 的隔离性实验
/// 证实:世界里只要有一个 multibody,rapier 0.36.1 的 solver 就把全部
/// 醒着的刚体合成一组,按 `base + max_extra` 个 solver substep 求解
/// (`dynamics/solver/staged_island_solver/init.rs:62` 的 `multi_group`
/// 条件)——extra 实际等同改全局迭代数,会改变同一分片内无关刚体的
/// 数值路径,不是按主体隔离的。函数保留只用于实验,生产不采用。
/// `extra = 0` 时不调用任何 API,数值路径与不开 `extra` 完全一致。
pub fn set_extra_solver_iterations(world: &mut PhysicsWorld, robot: &Robot, extra: usize) {
    if extra == 0 {
        return;
    }
    for h in robot
        .links
        .iter()
        .copied()
        .chain(std::iter::once(robot.body))
    {
        if let Some(rb) = world.bodies.get_mut(h) {
            rb.set_additional_solver_iterations(extra);
        }
    }
}

/// 读一个关节的当前角(rad)。revolute 单自由度,coords 的角分量里取
/// 对应轴;单自由度关节另一角分量恒 0。
pub fn read_angle(world: &MultibodyJointSet, robot: &Robot, dof: usize) -> f32 {
    let (mb_index, link_id) = robot.joints[dof];
    let mb = world.get_multibody(mb_index).expect("multibody alive");
    let link = mb.link(link_id).expect("link alive");
    link.joint.coords()[motor_index(dof)]
}

/// 读一个关节的当前角速度(rad/s;力矩饱和估算用)。
pub fn joint_velocity(world: &MultibodyJointSet, robot: &Robot, dof: usize) -> f32 {
    let (mb_index, link_id) = robot.joints[dof];
    let mb = world.get_multibody(mb_index).expect("multibody alive");
    let link = mb.link(link_id).expect("link alive");
    mb.joint_velocity(link)[0]
}

// ── 逆解与步态(R1-3) ──────────────────────────────────────────────────

/// 足端(相对髋,x 前 / y 下为负)→ (hip_pitch, knee)。膝朝后(Go2 口径)。
pub fn ik(dx: f32, dy: f32) -> (f32, f32) {
    let d2 = dx * dx + dy * dy;
    let c = ((d2 - 2.0 * L * L) / (2.0 * L * L)).clamp(-1.0, 1.0);
    let k = c.acos();
    let phi = dx.atan2(-dy);
    (phi - k / 2.0, k)
}

/// 开环步态参数(足端轨迹口径)。
#[derive(Clone, Copy)]
pub struct Gait {
    /// 步态频率(Hz)。
    pub freq: f32,
    /// 步长(单步前后位移,m)。
    pub stride: f32,
    /// 抬脚高度(m)。
    pub lift: f32,
    /// 支撑相占比(0..1),其余为摆动相。
    pub duty: f32,
    /// 足端目标离髋高度(站高,m)。
    pub h: f32,
    /// 足端目标前后中心(相对髋,m)。
    pub x0: f32,
}

impl Gait {
    /// 腿 leg 在行走时刻 t 的足端目标(相对髋)。腿序 [FL, FR, RL, RR],
    /// 对角小跑:FL/RR 相位 0,FR/RL 相位 0.5。
    pub fn foot_target(&self, leg: usize, t: f32) -> (f32, f32) {
        let ph0 = if leg == 0 || leg == 3 { 0.0 } else { 0.5 };
        let ramp = (t / 1.0).min(1.0); // 步幅/抬脚 1 s 渐入
        let p = (self.freq * t + ph0).rem_euclid(1.0);
        let stride = self.stride * ramp;
        let lift = self.lift * ramp;
        if p < self.duty {
            // 支撑相:匀速后扫。
            let u = p / self.duty;
            (self.x0 + stride / 2.0 - stride * u, -self.h)
        } else {
            // 摆动相:smoothstep 前摆 + 正弦抬脚。
            let u = (p - self.duty) / (1.0 - self.duty);
            let sm = u * u * (3.0 - 2.0 * u);
            (
                self.x0 - stride / 2.0 + stride * sm,
                -self.h + lift * (std::f32::consts::PI * u).sin(),
            )
        }
    }
    /// 站姿 / 摆动相位无关的初始目标:直腿降下来后取 (x0, −h)。
    pub fn stand_foot(&self) -> (f32, f32) {
        (self.x0, -self.h)
    }
}

/// 默认步态组(A00-2 放行)。
pub fn default_gait() -> Gait {
    Gait {
        freq: 3.0,
        stride: 0.15,
        lift: 0.06,
        duty: 0.5,
        h: FOOT_HEIGHT,
        x0: 0.0,
    }
}

/// 行走时刻 t(起步后秒数)、腿 leg 的 3 个关节目标 [lat, hip_pitch, knee]。
/// 纯函数,起步行列(站立→步态渐入)由调用方处理(`gait` 的 rise/settle
/// 序列保留在 demo 侧,terrain/mass 用 [`stand_targets`] 直接站立)。
pub fn joint_targets(gait: &Gait, leg: usize, t: f32) -> [f32; 3] {
    let (dx, dy) = gait.foot_target(leg, t);
    let (a, k) = ik(dx, dy);
    [0.0, a, k]
}

/// 站立姿态的 12 个关节目标。
pub fn stand_targets(gait: &Gait) -> [f32; N_JOINTS] {
    let (dx, dy) = gait.stand_foot();
    let (a, k) = ik(dx, dy);
    let mut out = [0.0f32; N_JOINTS];
    for leg in 0..4 {
        out[leg * 3 + 1] = a;
        out[leg * 3 + 2] = k;
    }
    out
}

// ── 快照 / 恢复(A00-3 放行追加;migrate / freeze 共用) ─────────────────
//
// 追加纪律(A00-3 放行原文):quad 只允许追加,追加后 `gait` / `terrain` /
// `mass` 的状态哈希必须与当前提交逐位相同。本节只新增类型与函数,不触碰
// 上面的常量、`spawn_robot(_yawed)`、`set_target`、`ik`、`Gait` 的任何
// 数值路径。
//
// rapier 0.36.1 依据(实测侦察,坐标恢复的唯一公开路径):
// - `MultibodyJoint::apply_displacement` → `integrate(1.0, disp)`:对 revolute
//   单自由度关节 = `coords[AngX] += disp` 并按新角重建 `joint_rot`(`coords`
//   是 `pub(crate)`,无 setter,这是唯一公开的坐标写入路径);
// - `Multibody::forward_kinematics(bodies, true)` 从根刚体位姿重算全部连杆
//   `local_to_world`(内部先 `update_root_type`:根刚体 `is_dynamic()` 与
//   `root_is_dynamic` 不符时重建 free/fixed 根关节——freeze demo 实证点,
//   rapier multibody.rs ~:1290);
// - `Multibody::update_rigid_bodies` 把连杆位姿写回各刚体 position/next;
// - `Multibody::generalized_velocity_mut` 写广义速度;下一拍 solver 的
//   `update_velocities` 从 `self.velocities` 重建各连杆刚体速度
//   (staged_island_solver/helpers.rs:108)。

/// 一只主体的迁移信封(REQ-012 §3 `MigrateAgent` 的 demo 口径):根状态、
/// 关节坐标、关节速度、最后的目标 + 连杆位姿(判据 1 的"连杆位置误差"
/// 核对基准;生产信封不带,连杆位姿由根状态 + 关节坐标唯一决定)。
///
/// `full = false` 时 `root_linvel/root_angvel/joint_vels` 清零——
/// "不带速度的快照"对照(证明速度是否必须带)。
#[derive(Clone)]
pub struct AgentSnapshot {
    /// 根刚体位姿(世界系)。
    pub root_pose: Pose,
    /// 根刚体线速度 / 角速度(full 快照才有值)。
    pub root_linvel: Vector,
    pub root_angvel: Vector,
    /// 12 个关节角(rad;`link.joint.coords()[ANG_X]`,与 `read_angle` 同源)。
    pub joint_coords: [f32; N_JOINTS],
    /// 12 个关节角速度(rad/s;`Multibody::joint_velocity(link)[0]`,
    /// 与 `joint_velocity()` 读数同源;full 快照才有值)。
    pub joint_vels: [f32; N_JOINTS],
    /// 全部 13 个连杆刚体的世界位姿(顺序同 `Robot.links` 再根放最后;
    /// 判据 1 的连杆位置误差核对基准)。
    pub link_poses: Vec<Pose>,
    /// 迁移时生效的 12 个电机目标(迁移后照写,不重算)。
    pub targets: [f32; N_JOINTS],
}

/// 从世界读一只主体的完整快照;`full = false` 时速度字段清零。
pub fn snapshot_robot(
    world: &PhysicsWorld,
    robot: &Robot,
    targets: &[f32; N_JOINTS],
    full: bool,
) -> AgentSnapshot {
    let body = world.bodies.get(robot.body).expect("root alive");
    let mut coords = [0.0f32; N_JOINTS];
    let mut vels = [0.0f32; N_JOINTS];
    for dof in 0..N_JOINTS {
        coords[dof] = read_angle(&world.multibody_joints, robot, dof);
        if full {
            vels[dof] = joint_velocity(&world.multibody_joints, robot, dof);
        }
    }
    let mut link_poses = Vec::with_capacity(robot.links.len());
    for lh in &robot.links {
        let rb = world.bodies.get(*lh).expect("link alive");
        link_poses.push(*rb.position());
    }
    AgentSnapshot {
        root_pose: *body.position(),
        root_linvel: if full { body.linvel() } else { Vector::ZERO },
        root_angvel: if full { body.angvel() } else { Vector::ZERO },
        joint_coords: coords,
        joint_vels: vels,
        link_poses,
        targets: *targets,
    }
}

/// 在 `world` 里按快照重建一只机器人(fresh `spawn_robot_yawed` + 写回),
/// 返回新句柄组。`same` / `cross` / `cross_hist` 三种重建、`freeze` 的
/// `snap` 方案共用;差异只在调用方(同世界/跨世界/目标世界历史)。
pub fn restore_robot(
    world: &mut PhysicsWorld,
    snap: &AgentSnapshot,
    mass_scale: f32,
    foot_mu: f32,
    damp_mode: DampMode,
) -> Robot {
    let robot = spawn_robot_yawed(
        world,
        snap.root_pose.translation,
        snap.root_pose.rotation,
        mass_scale,
        foot_mu,
        damp_mode,
    );
    let (mb_index, _) = robot.joints[0];
    {
        let mb = world
            .multibody_joints
            .get_multibody_mut(mb_index)
            .expect("multibody alive");
        // 广义坐标增量:fresh revolute 关节坐标恒 0,delta = 快照角。
        // disp 按 assembly_id 落位(根 free 6 维 + 每关节 1 维)。
        let mut disp = vec![0.0f32; mb.ndofs()];
        for dof in 0..N_JOINTS {
            let (_, link_id) = robot.joints[dof];
            let aid = mb.link(link_id).expect("link alive").assembly_id();
            disp[aid] = snap.joint_coords[dof];
        }
        mb.apply_displacements(&disp);
        if snap.joint_vels.iter().any(|v| *v != 0.0) {
            let aids: Vec<usize> = (0..N_JOINTS)
                .map(|dof| {
                    let (_, link_id) = robot.joints[dof];
                    mb.link(link_id).expect("link alive").assembly_id()
                })
                .collect();
            let mut gv = mb.generalized_velocity_mut();
            for dof in 0..N_JOINTS {
                gv[aids[dof]] = snap.joint_vels[dof];
            }
        }
    }
    // 根速度:广义速度(根 free 关节前 3 维线性 / 后 3 维角)与刚体各写一份
    // ——刚体速度供接触/读数,multibody velocities 供下一拍 solver 的
    // update_velocities 重建连杆速度。
    {
        let mb = world
            .multibody_joints
            .get_multibody_mut(mb_index)
            .expect("multibody alive");
        let mut gv = mb.generalized_velocity_mut();
        gv[0] = snap.root_linvel.x;
        gv[1] = snap.root_linvel.y;
        gv[2] = snap.root_linvel.z;
        gv[3] = snap.root_angvel.x;
        gv[4] = snap.root_angvel.y;
        gv[5] = snap.root_angvel.z;
    }
    if let Some(rb) = world.bodies.get_mut(robot.body) {
        rb.set_linvel(snap.root_linvel, false);
        rb.set_angvel(snap.root_angvel, false);
    }
    {
        let mb = world
            .multibody_joints
            .get_multibody_mut(mb_index)
            .expect("multibody alive");
        mb.forward_kinematics(&world.bodies, true);
        mb.update_rigid_bodies(&mut world.bodies, false);
    }
    // 状态注入 = "目标变化"级事件,按生产语义强唤醒整只(根 + 全部连杆)。
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
    robot
}

/// 重建误差核对(migrate 判据 1 / freeze 判据 2):重建后与快照逐项比较。
/// 返回(关节坐标最大绝对误差 rad,连杆位置最大误差 m,连杆转角最大误差
/// rad,速度误差最大 m/s 或 rad/s;速度误差 = 关节速度/根线/根角取大)。
///
/// - 连杆位置/转角误差用**刚体位姿**逐个比较(`link_poses` 与
///   `Robot.links` 同序;转角差 = 旋转差向量长度);
/// - 关节速度误差读 `Multibody::generalized_velocity()`(公开只读口;
///   `link.joint_velocity` 是 solver 每拍回写的字段,步进前恒 0,不适用)。
pub fn measure_restore_error(
    world: &PhysicsWorld,
    robot: &Robot,
    snap: &AgentSnapshot,
) -> (f32, f32, f32, f32) {
    let mut max_coord_err = 0.0f32;
    let mut max_vel_err = 0.0f32;
    for dof in 0..N_JOINTS {
        let q = read_angle(&world.multibody_joints, robot, dof);
        max_coord_err = max_coord_err.max((q - snap.joint_coords[dof]).abs());
    }
    let (mb_index, _) = robot.joints[0];
    if let Some(mb) = world.multibody_joints.get_multibody(mb_index) {
        let gv = mb.generalized_velocity();
        for dof in 0..N_JOINTS {
            let (_, link_id) = robot.joints[dof];
            let aid = mb.link(link_id).expect("link alive").assembly_id();
            max_vel_err = max_vel_err.max((gv[aid] - snap.joint_vels[dof]).abs());
        }
    }
    let mut max_pos_err = 0.0f32;
    let mut max_rot_err = 0.0f32;
    for (lh, pose) in robot.links.iter().zip(&snap.link_poses) {
        let rb = world.bodies.get(*lh).expect("link alive");
        let d = rb.translation() - pose.translation;
        max_pos_err = max_pos_err.max(d.length());
        // 旋转差:两个单位四元数夹角的 2·|sin(θ/2)| ≈ 相对转角(小角下)。
        let dq = rb.rotation() * pose.rotation.inverse();
        let rot_err = 2.0 * Vector::new(dq.x, dq.y, dq.z).length();
        max_rot_err = max_rot_err.max(rot_err);
    }
    let body = world.bodies.get(robot.body).expect("root alive");
    let root_lin_err = (body.linvel() - snap.root_linvel).length();
    let root_ang_err = (body.angvel() - snap.root_angvel).length();
    (
        max_coord_err,
        max_pos_err,
        max_rot_err,
        max_vel_err.max(root_lin_err).max(root_ang_err),
    )
}
