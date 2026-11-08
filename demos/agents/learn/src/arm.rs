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
use rapier3d::math::{Pose, Rotation, Vector};
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
/// 动作:增量式,目标角 += 0.1 × 动作(钳到限位)。
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
/// `gain_mult` 乘 kp/kd(L5 的 0.7 档;力矩上限不动)。
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
        .set_motor(axis, target, 0.0, KP * gain_mult, KD * gain_mult)
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
/// 施加力,同 quad 口径)。`gain_mult` 与电机一致(L5)。
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
        tau[dof] = (KP * gain_mult * (targets[dof] - q) - KD * gain_mult * qd)
            .clamp(-TAU_LIMIT, TAU_LIMIT);
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
    /// kp/kd 倍数(L5 用 0.7;kd 同乘,保持 ζ 不变)。
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
    /// 物理变体(连杆质量与增益倍数;L5)。
    pub variant: PhysVariant,
}

impl Env {
    /// `variant` = 物理变体(默认 default() = 主口径;L5 用 1.5/0.7)。
    pub fn new(seed: u64) -> Env {
        Env::with_variant(seed, PhysVariant::default())
    }

    pub fn with_variant(seed: u64, variant: PhysVariant) -> Env {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0.0, -9.81, 0.0);
        world.integration_parameters = production_params();
        let arm = spawn_arm(&mut world, variant.mass_mult);
        let mut env = Env {
            world,
            arm,
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
        let theta: f32 = rng.random_range(0.0..std::f32::consts::TAU);
        let rmin2 = TARGET_R_MIN * TARGET_R_MIN;
        let rmax2 = TARGET_R_MAX * TARGET_R_MAX;
        let r = (rng.random_range(0.0..1.0f32).mul_add(rmax2 - rmin2, rmin2)).sqrt();
        self.target = Vector::new(r * theta.cos(), r * theta.sin(), 0.0);
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0.0, -9.81, 0.0);
        world.integration_parameters = production_params();
        let arm = spawn_arm(&mut world, self.variant.mass_mult);
        let q1: f32 = rng.random_range(-INIT_Q_RANGE..INIT_Q_RANGE);
        let q2: f32 = rng.random_range(-INIT_Q_RANGE..INIT_Q_RANGE);
        // 注入出生角:按解析 FK 摆好连杆位姿(set_position,速度保持 0),
        // 电机目标 = 出生角(动作从这里开始增量)。
        let ang1 = Rotation::from_rotation_z(q1);
        let total = q1 + q2;
        let ang2 = Rotation::from_rotation_z(total);
        let l1_pos = 0.5 * LINK_LEN * Vector::new(q1.sin(), -q1.cos(), 0.0);
        let l2_pos = LINK_LEN * Vector::new(q1.sin(), -q1.cos(), 0.0)
            + 0.5 * LINK_LEN * Vector::new(total.sin(), -total.cos(), 0.0);
        for (h, pos, ang) in [(arm.links[1], l1_pos, ang1), (arm.links[2], l2_pos, ang2)] {
            world.bodies.get_mut(h).expect("link alive").set_position(
                Pose {
                    translation: pos,
                    rotation: ang,
                    ..Pose::IDENTITY
                },
                false,
            );
        }
        for (dof, q) in [q1, q2].into_iter().enumerate() {
            set_target_mult(&mut world, &arm, dof, q, self.variant.gain_mult);
        }
        self.world = world;
        self.arm = arm;
        self.targets = [q1, q2];
        self.last_action = [0.0; ACT_DIM];
        self.step = 0;
        self.done = false;
        observe(&self.world, &self.arm, self.target, &self.last_action)
    }

    /// 一个控制周期:目标角 += ACT_SCALE × 动作(钳限位),第 1 拍设目标后
    /// 步进 TICKS_PER_CONTROL 拍。返回 reward_inputs(奖励在 Python 算)。
    /// `every_tick` = 60 Hz 对照口径(每拍设目标、步 1 拍)。
    pub fn control_step(
        &mut self,
        action: &[f32; ACT_DIM],
        every_tick: bool,
    ) -> [f32; REWARD_KEYS] {
        for dof in 0..ACT_DIM {
            self.targets[dof] =
                (self.targets[dof] + ACT_SCALE * action[dof]).clamp(-JOINT_LIMIT, JOINT_LIMIT);
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
        self.last_action = *action;
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
