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
