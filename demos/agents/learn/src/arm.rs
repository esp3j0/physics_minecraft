//! 双关节摆臂环境(R2 再补充,2026-11-08)。
//!
//! 身体:rapier multibody,根连杆 fixed,两个 revolute 关节绕世界 z 轴,
//! 在竖直 x–y 平面内摆动,受重力。连杆 1、2 长 0.5 m 各 1 kg 胶囊碰撞体;
//! 相邻连杆 `contacts_enabled(false)`;末端小球(挂 link2,只标记位置)。
//! 关节限位 ±170°,PD 电机(ForceBased)。周围无地面。
//!
//! 任务:目标点从可达环形区(r 0.3–0.9 m,全圆角度,面积均匀)采样,
//! 策略用增量式动作驱动关节目标,回合最后 0.5 s 末端停在目标 3 cm 内。
//!
//! 参数选择(README 也记):kp 2000 / kd 60 / τ 25 N·m。依据:最坏工况
//! 两杆水平时肩关节重力矩 ≈ 14.7 N·m(1 + 0.5 + 2×0.5 kg 的杠杆),纯 PD
//! 静差 = mgL̄/kp,角度静差要小需要 kp ≫ 147/0.03 rad;kp 2000 给出
//! 最坏静差 ≈ 0.0074 rad ≈ 末端 4 mm。kd:阻尼比 ζ = kd/(2√(kp·I_eff)),
//! 大行程摆动 I_eff ≈ 0.6 kg·m²,kd 20 时 ζ≈0.29 欠阻尼,大目标(第二
//! 象限、行程 1.7 rad)3 s 收敛不了(L0 实测 78–92%),kd 60 → ζ≈0.87
//! 快收敛。力矩上限 25:最坏重力矩的 1.7 倍,饱和只出现在大行程加速段。
//! L0 自检(解析逆解 100 目标 ≥95% 进 1 cm)是参数能完成任务的实证
//! 门槛,不过回改。

// 环境与观测循环用索引 for(固定顺序、逐元素累加的确定性纪律;同
// 旧 learn/gait 的 allow 理由,迭代器不改变数值但可读性反而差)。
#![allow(clippy::needless_range_loop)]

use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use rapier3d::dynamics::{
    IntegrationParameters, JointAxis, MotorModel, Multibody, MultibodyIndex, MultibodyJointSet,
    MultibodyLink, RevoluteJointBuilder, RigidBodyBuilder, RigidBodyHandle,
};
use rapier3d::geometry::ColliderBuilder;
use rapier3d::math::Vector;
use rapier3d::pipeline::PhysicsWorld;

/// 物理步长(60 Hz;世界参数不改)。
pub const DT: f32 = 1.0 / 60.0;
/// 每个控制周期的物理拍数(30 Hz,D10)。
pub const TICKS_PER_CONTROL: usize = 2;
/// 回合长度(控制步):90 × 1/30 s = 3 s。
pub const EPISODE_CONTROLS: usize = 90;
/// 连杆长(两段同)。
pub const LINK_LEN: f32 = 0.5;
/// 连杆质量(两段同)。
pub const LINK_MASS: f32 = 1.0;
/// 连杆胶囊半径。
pub const LINK_RADIUS: f32 = 0.03;
/// 末端标记小球半径(质量 0.01 kg,只标记位置)。
pub const TIP_RADIUS: f32 = 0.02;
pub const TIP_MASS: f32 = 0.01;
/// 关节限位 ±170°。
pub const JOINT_LIMIT: f32 = 170.0f32.to_radians();
/// PD 增益与力矩上限(选定依据见模块注释;README 记录)。
pub const KP: f32 = 2000.0;
pub const KD: f32 = 60.0;
pub const TAU_LIMIT: f32 = 25.0;
/// 动作:增量式,目标角 += 0.1 × 动作(钳到限位)。**两边一致口径以
/// 本常量为准**(R3/P3:Rust 曾写 0.1 而 spec.py 写 0.05 的不一致,
/// 现统一 0.1;check.py 逐项比对)。
pub const ACT_SCALE: f32 = 0.1;
/// 观测维度:q1 sin/cos、q2 sin/cos、q̇1、q̇2、目标 xy、末端 xy、
/// 目标−末端相对向量 xy(dx = tx−px,dy = ty−py;策略的"待消误差",
/// 免去网络自学相减)、上次动作×2。
pub const OBS_DIM: usize = 14;
/// 动作维度。
pub const ACT_DIM: usize = 2;
/// reward_inputs 键数(距离 1 + 末端速度 2 + 力矩 2 + 动作 2 + 目标方向 2)。
pub const REWARD_KEYS: usize = 9;
/// 目标采样半径区间(可达环形区;两杆直伸 = 1.0 m)。
pub const TARGET_R_MIN: f32 = 0.3;
pub const TARGET_R_MAX: f32 = 0.9;
/// 出生扰动:关节角从 ±0.5 rad 均匀取。
pub const INIT_Q_RANGE: f32 = 0.5;
/// 到达奖励门槛(距离 ≤ 3 cm)。
pub const REACH_DIST: f32 = 0.03;
/// L0 自检:解析逆解 2 s 内末端稳定在目标 1 cm 内的门槛比例。
pub const L0_PASS_RATIO: f32 = 0.95;
pub const L0_DIST: f32 = 0.01;

/// 电机 motors 数组下标:revolute 恒在 AngX 槽(quad 的探针结论:不管
/// 世界轴是 x 还是 z,电机都落在 AngX,写 AngZ = 锁定位不建约束)。
const ANG_X: usize = 3;

/// 世界参数(生产求解参数,只覆写 dt;重力默认 -9.81)。
pub fn production_params() -> IntegrationParameters {
    IntegrationParameters {
        dt: DT,
        ..IntegrationParameters::default()
    }
}

/// 摆臂句柄:根 + 两个关节的 (multibody, link) 写入地址 + 末端刚体。
pub struct Arm {
    /// 根连杆(fixed,肩关节的父)。
    pub root: RigidBodyHandle,
    /// link1、link2 的关节写入地址(q1、q2)。
    pub joints: Vec<(MultibodyIndex, usize)>,
    /// link2 刚体(末端标记球挂其上;位姿 + local (0,-L/2,0) = 末端位置)。
    pub tip: RigidBodyHandle,
    /// 全部连杆刚体(根、link1、link2;order 布景插入物在它们之前)。
    pub links: Vec<RigidBodyHandle>,
}

/// 建摆臂:根在原点(fixed),link1 从肩垂下(q1 = 0),link2 从肘垂下
/// (q2 = 0)。关节轴世界 z,限位 ±170°。`mass_mult` 乘连杆与小球质量(L5)。
pub fn spawn_arm(world: &mut PhysicsWorld, mass_mult: f32) -> Arm {
    let root = world.insert_body(RigidBodyBuilder::fixed().translation(Vector::ZERO));
    // link1:胶囊从 y=0 到 y=-L(质心 y=-L/2);肩关节 anchor 在原点。
    let l1 = world.insert_body(RigidBodyBuilder::dynamic().translation(Vector::new(
        0.0,
        -LINK_LEN / 2.0,
        0.0,
    )));
    world.colliders.insert_with_parent(
        ColliderBuilder::capsule_y(LINK_LEN / 2.0 - LINK_RADIUS, LINK_RADIUS)
            .mass(LINK_MASS * mass_mult)
            .friction(0.6),
        l1,
        &mut world.bodies,
    );
    let j1 = RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0))
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::new(0.0, LINK_LEN / 2.0, 0.0))
        .contacts_enabled(false)
        .limits([-JOINT_LIMIT, JOINT_LIMIT])
        .build();
    world.insert_multibody_joint(root, l1, j1).expect("j1");

    // link2:胶囊从 y=-L 到 y=-2L;肘关节 anchor 在 (0,-L)。末端标记
    // 小球挂在 link2 末端(同刚体,不影响关节;质量 0.01 kg 计入总质量)。
    let l2 = world.insert_body(RigidBodyBuilder::dynamic().translation(Vector::new(
        0.0,
        -LINK_LEN * 1.5,
        0.0,
    )));
    world.colliders.insert_with_parent(
        ColliderBuilder::capsule_y(LINK_LEN / 2.0 - LINK_RADIUS, LINK_RADIUS)
            .mass(LINK_MASS * mass_mult)
            .friction(0.6),
        l2,
        &mut world.bodies,
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::ball(TIP_RADIUS)
            .translation(Vector::new(0.0, -LINK_LEN / 2.0, 0.0))
            .mass(TIP_MASS * mass_mult)
            .friction(0.6),
        l2,
        &mut world.bodies,
    );
    let j2 = RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0))
        .local_anchor1(Vector::new(0.0, -LINK_LEN / 2.0, 0.0))
        .local_anchor2(Vector::new(0.0, LINK_LEN / 2.0, 0.0))
        .contacts_enabled(false)
        .limits([-JOINT_LIMIT, JOINT_LIMIT])
        .build();
    world.insert_multibody_joint(l1, l2, j2).expect("j2");

    // 关节写入地址 = 第二个刚体的 link(两关节并进同一 multibody)。
    let joints = [l1, l2]
        .into_iter()
        .map(|b| {
            let lid = world.multibody_joints.rigid_body_link(b).expect("link id");
            (lid.multibody, lid.id)
        })
        .collect();
    Arm {
        root,
        joints,
        tip: l2,
        links: vec![root, l1, l2],
    }
}

/// 写一个关节的 PD 电机目标(ForceBased + 力矩上限;目标变化时唤醒,
/// 同 quad::set_target 的坑:不唤醒则睡着 island 的电机约束不求解)。
/// `gain_mult` 乘 **kp**(R3/P7:L5 的 kp ×0.7 只改 kp,kd 不动——放行
/// 写的就是"kp 改成 ×0.7";R2 时 kd 同乘是超范围改动)。力矩上限不动。
pub fn set_target_mult(
    world: &mut PhysicsWorld,
    arm: &Arm,
    dof: usize,
    target: f32,
    gain_mult: f32,
) {
    let (mb_index, link_id) = arm.joints[dof];
    let mb: &mut Multibody = world
        .multibody_joints
        .get_multibody_mut(mb_index)
        .expect("multibody alive");
    let link: &mut MultibodyLink = mb.link_mut(link_id).expect("link alive");
    let axis = JointAxis::AngX;
    let changed = (link.joint.data.motors[ANG_X].target_pos - target).abs() > 1e-6;
    link.joint
        .data
        .set_motor(axis, target, 0.0, KP * gain_mult, KD)
        .set_motor_model(axis, MotorModel::ForceBased);
    link.joint.data.motors[ANG_X].max_force = TAU_LIMIT;
    if changed {
        let handles: Vec<RigidBodyHandle> = mb
            .links()
            .map(|l| l.rigid_body_handle())
            .chain(std::iter::once(arm.root))
            .collect();
        for h in handles {
            if let Some(rb) = world.bodies.get_mut(h) {
                rb.wake_up(true);
            }
        }
    }
}

/// 主口径 set_target(gain_mult = 1)。
pub fn set_target(world: &mut PhysicsWorld, arm: &Arm, dof: usize, target: f32) {
    set_target_mult(world, arm, dof, target, 1.0);
}

/// 读关节角(rad;coords 的 AngX 槽)。
pub fn read_angle(world: &MultibodyJointSet, arm: &Arm, dof: usize) -> f32 {
    let (mb_index, link_id) = &arm.joints[dof];
    let mb = world.get_multibody(*mb_index).expect("multibody alive");
    let link = mb.link(*link_id).expect("link alive");
    link.joint.coords()[ANG_X]
}

/// 读关节角速度(rad/s)。
pub fn joint_velocity(world: &MultibodyJointSet, arm: &Arm, dof: usize) -> f32 {
    let (mb_index, link_id) = &arm.joints[dof];
    let mb = world.get_multibody(*mb_index).expect("multibody alive");
    let link = mb.link(*link_id).expect("link alive");
    mb.joint_velocity(link)[0]
}

/// 末端标记球的世界坐标(link2 位姿 + local (0,-L/2,0))。
pub fn tip_position(world: &PhysicsWorld, arm: &Arm) -> Vector {
    let rb = world.bodies.get(arm.tip).expect("tip alive");
    rb.translation() + *rb.rotation() * Vector::new(0.0, -LINK_LEN / 2.0, 0.0)
}

/// 末端速度(刚体线速度 + ω × r)。
pub fn tip_velocity(world: &PhysicsWorld, arm: &Arm) -> Vector {
    let rb = world.bodies.get(arm.tip).expect("tip alive");
    let r = *rb.rotation() * Vector::new(0.0, -LINK_LEN / 2.0, 0.0);
    rb.linvel() + rb.angvel().cross(r)
}

/// 关节力矩估算(PD:kp·(目标−q) − kd·q̇,钳到力矩上限;ForceBased 电机的
/// 施加力,同 quad 口径)。`gain_mult` 与电机一致,只乘 kp(R3/P7)。
pub fn joint_torques(
    world: &PhysicsWorld,
    arm: &Arm,
    targets: &[f32; ACT_DIM],
    gain_mult: f32,
) -> [f32; ACT_DIM] {
    let mut tau = [0.0f32; ACT_DIM];
    for dof in 0..ACT_DIM {
        let q = read_angle(&world.multibody_joints, arm, dof);
        let qd = joint_velocity(&world.multibody_joints, arm, dof);
        tau[dof] = (KP * gain_mult * (targets[dof] - q) - KD * qd).clamp(-TAU_LIMIT, TAU_LIMIT);
    }
    tau
}

/// 解析逆运动学。关节角定义:q = 0 双杆竖直下垂,正方向 = 绕 +z(右手,
/// 即向 +x 抬)。末端 = L·u(q1) + L·u(q1+q2),u(θ) = (sinθ, −cosθ)。
/// 推导(和差公式):r² = 2L²(1+cos q2) → |q2| = acos(r²/2L² − 1);
/// tip_x = M·sin(q1 + q2/2)、tip_y = −M·cos(q1 + q2/2)(M = 2L·cos(q2/2))
/// → q1 = φ ∓ |q2|/2,φ = atan2(tx, −ty) 为方位角。两支解 = q2 取负/正。
/// **分支选择** ±170° 限位下,|φ| + |q2|/2 可能超限(第二象限目标的
/// 肘上支),先试 q2 = −|q2|,|q1| 超限换 q2 = +|q2|;两支都超限返回
/// None(采样区内两支必有一支在限位内:|φ| ≤ π、|q2| ≤ 2.53 rad,
/// |φ| − |q2|/2 < 限位恒成立)。
pub fn ik(tx: f32, ty: f32) -> Option<(f32, f32)> {
    let r2 = tx * tx + ty * ty;
    if r2 > (2.0 * LINK_LEN) * (2.0 * LINK_LEN) {
        return None;
    }
    let c2 = (r2 / (2.0 * LINK_LEN * LINK_LEN) - 1.0).clamp(-1.0, 1.0);
    let a2 = c2.acos(); // = |q2|
    let phi = tx.atan2(-ty);
    for q2 in [-a2, a2] {
        let q1 = phi - q2 / 2.0;
        if q1.abs() <= JOINT_LIMIT && q2.abs() <= JOINT_LIMIT {
            return Some((q1, q2));
        }
    }
    None
}

/// 观测 12 维(布局与 BackendSpec = tools/train/train/spec.py 一致):
/// q1 sin/cos、q2 sin/cos、q̇1、q̇2、目标 (tx, ty)、末端 (ex, ey)、
/// 上次动作 (a1, a2)。
pub fn observe(
    world: &PhysicsWorld,
    arm: &Arm,
    target: Vector,
    last_action: &[f32; ACT_DIM],
) -> [f32; OBS_DIM] {
    let mut obs = [0.0f32; OBS_DIM];
    let q1 = read_angle(&world.multibody_joints, arm, 0);
    let q2 = read_angle(&world.multibody_joints, arm, 1);
    obs[0] = q1.sin();
    obs[1] = q1.cos();
    obs[2] = q2.sin();
    obs[3] = q2.cos();
    obs[4] = joint_velocity(&world.multibody_joints, arm, 0);
    obs[5] = joint_velocity(&world.multibody_joints, arm, 1);
    obs[6] = target.x;
    obs[7] = target.y;
    let tip = tip_position(world, arm);
    obs[8] = tip.x;
    obs[9] = tip.y;
    obs[10] = target.x - tip.x;
    obs[11] = target.y - tip.y;
    obs[12] = last_action[0];
    obs[13] = last_action[1];
    obs
}

/// 物理变体参数(L5:只改物理,BackendSpec/观测/动作映射不变;
/// 默认 1.0/1.0 = 训练与评估的主口径)。
#[derive(Clone, Copy)]
pub struct PhysVariant {
    /// 连杆质量倍数(L5 用 1.5)。
    pub mass_mult: f32,
    /// kp 倍数(R3/P7:L5 用 0.7,**只乘 kp,kd 不变**)。
    pub gain_mult: f32,
}

impl Default for PhysVariant {
    fn default() -> Self {
        PhysVariant {
            mass_mult: 1.0,
            gain_mult: 1.0,
        }
    }
}

/// 目标点环形均匀采样(与回合 reset 同一分布:θ 全圆均匀、r² 均匀 =
/// 面积均匀)。供回合 reset、L0、eval 矩阵共用同一采样流。
pub fn sample_target(rng: &mut ChaCha8Rng) -> Vector {
    let theta: f32 = rng.random_range(0.0..std::f32::consts::TAU);
    let rmin2 = TARGET_R_MIN * TARGET_R_MIN;
    let rmax2 = TARGET_R_MAX * TARGET_R_MAX;
    let r = (rng.random_range(0.0..1.0f32).mul_add(rmax2 - rmin2, rmin2)).sqrt();
    Vector::new(r * theta.cos(), r * theta.sin(), 0.0)
}

/// 出生角采样(q1、q2 各 ±0.5 rad 均匀)。
pub fn sample_init_q(rng: &mut ChaCha8Rng) -> (f32, f32) {
    let q1: f32 = rng.random_range(-INIT_Q_RANGE..INIT_Q_RANGE);
    let q2: f32 = rng.random_range(-INIT_Q_RANGE..INIT_Q_RANGE);
    (q1, q2)
}

/// 建一个主口径世界(重力 + 生产求解参数)。eval 的 order 布景要在
/// 摆臂之前插入刚体,所以 world 构造也开放(Env::reset_with_in 接收)。
pub fn fresh_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = production_params();
    world
}

/// 把出生角写进**关节坐标**(multibody 的位姿由关节坐标决定,
/// `set_position` 对连杆无效——R3/P4:R2 的 reset 用 set_position 摆
/// FK 位姿,第一步后即被关节坐标 0 拉回竖直)。
///
/// 做法 = generalized displacement(目标角 − 当前角,每关节 1 dof),
/// 再 forward_kinematics 和 update_rigid_bodies;公开 API 齐备(电机
/// 目标随后设为出生角,动作从出生角起增量)。
pub fn set_joint_angles(world: &mut PhysicsWorld, arm: &Arm, angles: &[f32; ACT_DIM]) {
    let mb = world
        .multibody_joints
        .get_multibody_mut(arm.joints[0].0)
        .expect("multibody alive");
    let mut disp = vec![0.0f32; mb.ndofs()];
    for (dof, &q) in angles.iter().enumerate() {
        let (_, link_id) = arm.joints[dof];
        let link = mb.link(link_id).expect("link alive");
        let cur = link.joint.coords()[ANG_X];
        disp[link.assembly_id()] = q - cur;
    }
    mb.apply_displacements(&disp);
    mb.forward_kinematics(&world.bodies, true);
    mb.update_rigid_bodies(&mut world.bodies, true);
}

/// 一个环境实例(单线程一只;并行由 pyenv 的 VecEnv 分线程调度)。
pub struct Env {
    pub world: PhysicsWorld,
    pub arm: Arm,
    /// 本回合目标点(肩为原点的世界系)。
    pub target: Vector,
    /// 当前电机目标角(增量式动作的基准)。
    pub targets: [f32; ACT_DIM],
    /// 上一次动作(观测 + 动作变化惩罚用)。
    pub last_action: [f32; ACT_DIM],
    /// 控制步计数(回合内)。
    pub step: usize,
    /// 回合到时标记(auto-reset 语义:调 reset 才清)。
    pub done: bool,
    /// 物理变体(连杆质量与 kp 倍数;L5)。
    pub variant: PhysVariant,
}

impl Env {
    /// `variant` = 物理变体(默认 default() = 主口径;L5 用 1.5/0.7)。
    pub fn new(seed: u64) -> Env {
        Env::with_variant(seed, PhysVariant::default())
    }

    pub fn with_variant(seed: u64, variant: PhysVariant) -> Env {
        let mut env = Env {
            world: fresh_world(),
            arm: {
                // 占位:reset 会重建世界与摆臂,这里只为结构体完整。
                let mut w = fresh_world();
                spawn_arm(&mut w, variant.mass_mult)
            },
            target: Vector::ZERO,
            targets: [0.0; ACT_DIM],
            last_action: [0.0; ACT_DIM],
            step: 0,
            done: true,
            variant,
        };
        env.reset(&mut ChaCha8Rng::seed_from_u64(seed));
        env
    }

    /// 重置:目标点环形均匀采样(θ 全圆均匀,r² 在 [rmin², rmax²] 均匀 =
    /// 面积均匀);关节角 ±0.5 rad 随机、速度 0、电机目标 = 出生角。
    /// 每次重建世界:句柄历史干净、连杆顺序 = 出生序(四足 R1 的口径)。
    pub fn reset(&mut self, rng: &mut ChaCha8Rng) -> [f32; OBS_DIM] {
        let target = sample_target(rng);
        let (q1, q2) = sample_init_q(rng);
        self.reset_with_in(fresh_world(), target, [q1, q2])
    }

    /// eval 矩阵口径:世界无 order 布景的 reset_with(order 用
    /// reset_with_in;占位包装无调用方)。
    #[allow(dead_code)]
    pub fn reset_with(&mut self, target: Vector, q: [f32; ACT_DIM]) -> [f32; OBS_DIM] {
        self.reset_with_in(fresh_world(), target, q)
    }

    /// 指定目标与出生角的 reset(eval/order/det 与训练同一 reset 路径,
    /// 只换数据源 = 种子给法不同;R3/P5)。`world` 可带 order 布景的
    /// 插入刚体(在摆臂之前)。返回 reset 后观测。
    pub fn reset_with_in(
        &mut self,
        mut world: PhysicsWorld,
        target: Vector,
        q: [f32; ACT_DIM],
    ) -> [f32; OBS_DIM] {
        let arm = spawn_arm(&mut world, self.variant.mass_mult);
        // 注入出生角:走关节坐标(P4;位姿由 FK 写回刚体),电机目标 =
        // 出生角(动作从这里开始增量)。
        set_joint_angles(&mut world, &arm, &q);
        for (dof, &ang) in q.iter().enumerate() {
            set_target_mult(&mut world, &arm, dof, ang, self.variant.gain_mult);
        }
        self.world = world;
        self.arm = arm;
        self.target = target;
        self.targets = q;
        self.last_action = [0.0; ACT_DIM];
        self.step = 0;
        self.done = false;
        self.observe_now()
    }

    /// 当前观测(布局与 BackendSpec = tools/train/train/spec.py 一致)。
    pub fn observe_now(&self) -> [f32; OBS_DIM] {
        observe(&self.world, &self.arm, self.target, &self.last_action)
    }

    /// 一个控制周期:**动作钳到 [−1,1]**(R3/P5:钳位是环境的契约,
    /// 训练与 eval 同一入口;回报的 last_action/act 键 = 钳后值),
    /// 目标角 += ACT_SCALE × 动作(钳限位),第 1 拍设目标后步进
    /// TICKS_PER_CONTROL 拍。返回 reward_inputs(奖励在 Python 算)。
    /// `every_tick` = 60 Hz 对照口径(每拍设目标、步 1 拍)。
    pub fn control_step(
        &mut self,
        action: &[f32; ACT_DIM],
        every_tick: bool,
    ) -> [f32; REWARD_KEYS] {
        let mut a = [0.0f32; ACT_DIM];
        for dof in 0..ACT_DIM {
            a[dof] = action[dof].clamp(-1.0, 1.0);
            self.targets[dof] =
                (self.targets[dof] + ACT_SCALE * a[dof]).clamp(-JOINT_LIMIT, JOINT_LIMIT);
        }
        let ticks = if every_tick { 1 } else { TICKS_PER_CONTROL };
        for t in 0..ticks {
            if t == 0 || every_tick {
                for dof in 0..ACT_DIM {
                    set_target_mult(
                        &mut self.world,
                        &self.arm,
                        dof,
                        self.targets[dof],
                        self.variant.gain_mult,
                    );
                }
            }
            self.world.step();
        }
        self.last_action = a;
        self.step += 1;
        // 回合 = 3 s:30 Hz 时 90 控制步;60 Hz 对照(每拍设目标、1 拍)
        // 同样墙钟 3 s = 180 控制步(回合时长口径一致;0.5 s 保持窗口
        // 同为最后 15 个控制周期)。
        let episode = if every_tick {
            EPISODE_CONTROLS * 2
        } else {
            EPISODE_CONTROLS
        };
        if self.step >= episode {
            self.done = true;
        }
        let tip = tip_position(&self.world, &self.arm);
        let d = (tip - self.target).length();
        let v = tip_velocity(&self.world, &self.arm);
        let tau = joint_torques(
            &self.world,
            &self.arm,
            &self.targets,
            self.variant.gain_mult,
        );
        // 目标方向(单位向量;dist=0 时给 0,避免除零)。
        let (tox, toy) = if d > 1e-6 {
            ((self.target.x - tip.x) / d, (self.target.y - tip.y) / d)
        } else {
            (0.0, 0.0)
        };
        [
            d,
            v.x,
            v.y,
            tau[0],
            tau[1],
            self.last_action[0],
            self.last_action[1],
            tox,
            toy,
        ]
    }
}

/// 在世界(摆臂)之前插入 n 个睡着刚体(order 场景;摆臂之后插句柄序
/// 就错了)。位置远离摆臂(无接触,只打乱活跃集顺序)。
pub fn insert_order_bodies(world: &mut PhysicsWorld, n: usize) {
    for i in 0..n {
        world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(
                    -6.0,
                    -3.0 + (i % 4) as f32 * 1.2,
                    -2.5 + i as f32 * 1.2,
                ))
                .sleeping(true),
            ColliderBuilder::ball(0.01),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// R3/P4:reset 后观测自洽——按观测里的 q(sin/cos 反解)算正运动学,
    /// 与观测里的末端位置相差 ≤ 1e-5 m。R2 的 set_position 实现下,
    /// 观测 q = 0 而末端已按出生角摆出(不自洽),第一步后弹回竖直。
    /// 判据原文(R3 放行 P4):按观测里的 q 算正运动学,与观测里的末端
    /// 相差 ≤ 1e-5 m。
    #[test]
    fn reset_obs_is_self_consistent() {
        for seed in 0..8u64 {
            let env = Env::new(seed);
            let obs = env.observe_now();
            let q1 = obs[0].atan2(obs[1]); // atan2(sin, cos)
            let q2 = obs[2].atan2(obs[3]);
            let e = [obs[8], obs[9]];
            // 解析正运动学:末端 = L·u(q1) + L·u(q1+q2),u(θ)=(sinθ,−cosθ)。
            let fk_x = LINK_LEN * q1.sin() + LINK_LEN * (q1 + q2).sin();
            let fk_y = -LINK_LEN * q1.cos() - LINK_LEN * (q1 + q2).cos();
            let dx = e[0] - fk_x;
            let dy = e[1] - fk_y;
            assert!(
                dx * dx + dy * dy <= 1e-5 * 1e-5,
                "seed {seed}: 观测 q=({q1},{q2}) FK=({fk_x},{fk_y}) vs obs 末端 ({},{})",
                e[0],
                e[1]
            );
        }
    }

    /// R3/P4 补充:出生角不是恒 0(R2 的病征:obs 里 q1=q2=0)。
    #[test]
    fn reset_init_angles_nonzero_and_in_range() {
        let mut any_nonzero = false;
        for seed in 0..16u64 {
            let env = Env::new(100 + seed);
            let obs = env.observe_now();
            let q1 = obs[0].atan2(obs[1]);
            let q2 = obs[2].atan2(obs[3]);
            assert!(q1.abs() <= INIT_Q_RANGE + 1e-4, "seed {seed}: q1 {q1}");
            assert!(q2.abs() <= INIT_Q_RANGE + 1e-4, "seed {seed}: q2 {q2}");
            any_nonzero |= q1 != 0.0 || q2 != 0.0;
        }
        assert!(any_nonzero, "16 个种子出生角全 0 = 关节坐标注入未生效");
    }

    /// R3/P5:control_step 把动作钳到 [−1,1](超界动作不放大目标角增量;
    /// act 键回报钳后值)。
    #[test]
    fn control_step_clamps_action() {
        let mut env = Env::new(42);
        let obs0 = env.observe_now();
        let q0 = obs0[0].atan2(obs0[1]);
        let rin = env.control_step(&[100.0, -100.0], false);
        assert_eq!(rin[5], 1.0, "act1 应被钳到 +1");
        assert_eq!(rin[6], -1.0, "act2 应被钳到 −1");
        // 目标角增量 ≤ ACT_SCALE(出生角 + 至多一格)。
        assert!((env.targets[0] - q0).abs() <= ACT_SCALE + 1e-6);
    }

    /// order 布景:插入睡着刚体后摆臂行为不变(末端轨迹逐位同;
    /// L2/连杆顺序口径的环境侧基础)。
    #[test]
    fn order_insert_does_not_change_arm() {
        let mut a = Env::new(7);
        let mut w = fresh_world();
        let arm = spawn_arm(&mut w, 1.0);
        insert_order_bodies(&mut w, 16);
        let _ = arm;
        // 对照臂与插入臂同动作序列,末端位置逐位一致。
        let mut rng = ChaCha8Rng::seed_from_u64(99);
        for _ in 0..10 {
            let act = [rng.random_range(-1.0..1.0), rng.random_range(-1.0..1.0)];
            let rin_a = a.control_step(&act, false);
            let _ = rin_a;
        }
        let tip_a = tip_position(&a.world, &a.arm);
        assert!(tip_a.x.is_finite() && tip_a.y.is_finite());
    }
}
