//! REQ-012 A00-1 demo `gait`(2026-11-06)。
//!
//! 问题:Go2 尺度四足(约 15 kg、12 自由度),用 rapier multibody + PD 电机,
//! 在全世界 60 Hz、不做子步、生产求解参数的条件下,靠运行时更新关节目标
//! 能不能开环对角小跑走起来、不发散。
//!
//! 生产参数口径(读 crates/physics-sim 确认,2026-11-06):
//! - `IntegrationParameters` 全部取 rapier 0.36.1 默认(`num_solver_iterations
//!   = 4`、warmstart 1.0、预测距离 0.02 等),只把 `dt` 覆写为
//!   `ShardSimConfig::new` 的 `1.0/60`(sim.rs:125、sim.rs:559);
//! - 重力 −9.81(sim.rs:124);
//! - 地面 = fixed cuboid,friction 0.6、restitution 0(`rebuild_chunk_collider`
//!   sim.rs:4205-4206)。
//!
//! 运行时目标的写法:rapier 0.36.1 的 multibody 电机每拍由 solver 从
//! `link.joint.data.motors[axis]` 重建约束(`unit_joint_motor_constraint`),
//! 所以改 `target_pos` 即可生效,不重建 multibody。PD 用 `MotorModel::
//! ForceBased`(force = kp·(q*−q) + kd·(q̇*−q̇)),这样 kp/kd/力矩上限是
//! 真实单位,×0.01 档可以同比缩放;默认的 AccelerationBased 是按质量归一
//! 的加速度增益,"力矩上限 23.7 N·m"对它没有意义。
//!
//! 几何:Go2 尺度手写,不导入 URDF。机身 0.38×0.12×0.10 m 10.5 kg;每腿
//! hip lateral(绕 z,横向短杆)→ hip pitch(绕 x,大腿)→ knee(绕 x,
//! 小腿)3 个转动关节;上腿 0.9 kg、横向杆 0.1 kg、下腿 0.6 kg →
//! 整体 ≈15 kg(§9.1)。
//!
//! 用法(见 README):
//! `cargo run --release -- --list` 列参数组;
//! `cargo run --release -- <name>...` 跑指定组(不带参数跑全部单只组);
//! `cargo run --release -- eight` 8 只耗时组;
//! `cargo run --release -- det <name>` 确定性双跑比对。

use std::f32::consts::FRAC_PI_2;
use std::time::Instant;

use rapier3d::dynamics::{
    IntegrationParameters, JointAxis, MotorModel, Multibody, MultibodyIndex, MultibodyJointSet,
    MultibodyLink, RevoluteJointBuilder, RigidBodyBuilder, RigidBodyHandle,
};
use rapier3d::geometry::{ColliderBuilder, ColliderHandle};
use rapier3d::math::Vector;
use rapier3d::pipeline::PhysicsWorld;

// ── 模型常量(Go2 尺度,§9.1) ───────────────────────────────────────────

/// 机身半尺寸(x 长 / y 厚 / z 宽)。
const BODY_HALF: [f32; 3] = [0.19, 0.06, 0.05];
/// 机身质量(kg)。
const BODY_MASS: f32 = 10.5;
/// 髋锚点纵向偏移。
const HIP_LONG: f32 = 0.14;
/// 髋锚点横向偏移(机身半宽之外)。
const HIP_LATERAL: f32 = 0.10;
/// 横向短杆半长(hip lateral → hip pitch 之间的连杆)。
const LAT_HALF: f32 = 0.03;
/// 横向短杆质量(kg)。
const LAT_MASS: f32 = 0.1;
/// 上腿长(hip pitch → knee)。
const THIGH_LEN: f32 = 0.11;
/// 下腿长(knee → 脚底)。
const SHANK_LEN: f32 = 0.12;
/// 上腿质量(kg)。
const THIGH_MASS: f32 = 0.9;
/// 下腿质量(kg,含脚)。
const SHANK_MASS: f32 = 0.6;
/// 上腿半径。
const THIGH_RADIUS: f32 = 0.025;
/// 下腿半径 = 脚半径。
const SHANK_RADIUS: f32 = 0.02;
/// shank(脚)材质摩擦:与地面 0.6 平均组合 ≈ 1.0(橡胶垫)。
const SHANK_FRICTION: f32 = 1.4;
/// 力矩上限(N·m,§9.1)。
const TAU_LIMIT: f32 = 23.7;
/// 地面摩擦 / 恢复(同 rebuild_chunk_collider)。
const GROUND_FRICTION: f32 = 0.6;
const GROUND_RESTITUTION: f32 = 0.0;
/// 关节总数 = 4 腿 × 3。
const N_JOINTS: usize = 12;
/// 期望站高(机身中心离地,§9.1 约 0.33 m)。
const STAND_HEIGHT: f32 = 0.33;
/// motors 数组下标:3D 布局 [LinX, LinY, LinZ, AngX, AngY, AngZ]。
const ANG_X: usize = 3;

// ── 世界与机器人 ────────────────────────────────────────────────────────

/// 生产参数的 IntegrationParameters(rapier 默认 + dt 覆写)。
fn production_params(dt: f32) -> IntegrationParameters {
    IntegrationParameters {
        dt,
        ..IntegrationParameters::default()
    }
}

/// 一只机器人:句柄 + 每关节的 (multibody, link) 写入地址。
struct Robot {
    body: RigidBodyHandle,
    /// 长度 12,顺序 = 腿序 [FL, FR, RL, RR] × [hip_lat, hip_pitch, knee]。
    joints: Vec<(MultibodyIndex, usize)>,
}

/// 腿的几何:锚点(机身 local)+ 关节限位。
struct LegGeom {
    hip_anchor: Vector,
    hip_lat_limits: [f32; 2],
    hip_pitch_limits: [f32; 2],
    knee_limits: [f32; 2],
    /// 横向朝外方向(±1)。
    out: f32,
}

fn leg_geoms() -> [LegGeom; 4] {
    let mk = |front: bool, side: f32| LegGeom {
        hip_anchor: Vector::new(
            if front { HIP_LONG } else { -HIP_LONG },
            -BODY_HALF[1] - 0.02,
            side * HIP_LATERAL,
        ),
        hip_lat_limits: if side > 0.0 { [0.0, 0.7] } else { [-0.7, 0.0] },
        hip_pitch_limits: [-1.0, 1.0],
        knee_limits: [-2.4, -0.2],
        out: if side > 0.0 { 1.0 } else { -1.0 },
    };
    [
        mk(true, -1.0),
        mk(true, 1.0),
        mk(false, -1.0),
        mk(false, 1.0),
    ]
}

/// 建一只机器人:机身 + 4 腿 × 3 关节,连杆按"关节角全 0"的竖直链位姿。
fn spawn_robot(world: &mut PhysicsWorld, origin: Vector, mass_scale: f32) -> Robot {
    let m = |v: f32| v * mass_scale;

    let body = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(origin)
            .additional_mass(m(BODY_MASS)),
    );
    let _ = world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(BODY_HALF[0], BODY_HALF[1], BODY_HALF[2])
            .density(0.0)
            .friction(GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
        body,
        &mut world.bodies,
    );

    let mut joints = Vec::new();
    for geom in leg_geoms() {
        let anchor_w = origin + geom.hip_anchor;
        // hip lateral 连杆:横向短杆(半径 0.02 的 y 向胶囊当杆),从锚点
        // 向外伸 LAT_HALF*2,外端是 hip_pitch 锚点。
        let lat_center = anchor_w + Vector::new(0.0, 0.0, geom.out * LAT_HALF);
        let lat_body = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(lat_center)
                .additional_mass(m(LAT_MASS)),
        );
        let _ = world.colliders.insert_with_parent(
            ColliderBuilder::capsule_y(LAT_HALF, 0.02)
                .density(0.0)
                .friction(GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
            lat_body,
            &mut world.bodies,
        );
        // hip lateral 关节(绕 x,y→z 的横向摆):机身 local 锚点 =
        // hip_anchor;lat 连杆 local 锚点 = 内端(-out*LAT_HALF)。
        let lat_joint = RevoluteJointBuilder::new(Vector::new(1.0, 0.0, 0.0))
            .local_anchor1(geom.hip_anchor)
            .local_anchor2(Vector::new(0.0, 0.0, -geom.out * LAT_HALF))
            .limits(geom.hip_lat_limits)
            .build();
        let lat_handle = world
            .insert_multibody_joint(body, lat_body, lat_joint)
            .expect("hip lateral joint");

        // thigh 连杆:从 lat 外端竖直向下 THIGH_LEN(capsule_y 半长)。
        let thigh_top = lat_center + Vector::new(0.0, 0.0, geom.out * LAT_HALF);
        let thigh_center = thigh_top - Vector::new(0.0, THIGH_LEN / 2.0, 0.0);
        let thigh_body = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(thigh_center)
                .additional_mass(m(THIGH_MASS)),
        );
        let _ = world.colliders.insert_with_parent(
            ColliderBuilder::capsule_y(THIGH_LEN / 2.0, THIGH_RADIUS)
                .density(0.0)
                .friction(GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
            thigh_body,
            &mut world.bodies,
        );
        let hip_pitch_joint = RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0))
            .local_anchor1(Vector::new(0.0, 0.0, geom.out * LAT_HALF))
            .local_anchor2(Vector::new(0.0, THIGH_LEN / 2.0, 0.0))
            .limits(geom.hip_pitch_limits)
            .build();
        let hip_handle = world
            .insert_multibody_joint(lat_body, thigh_body, hip_pitch_joint)
            .expect("hip pitch joint");

        // shank 连杆:从 thigh 下端竖直向下 SHANK_LEN;脚底 = capsule 下端
        // 半圆(SHANK_RADIUS 当脚半径)。脚垫变体(高摩擦 ball)实测更差:
        // 腿变长 1 cm、落地冲击变大,基线从 3600 拍不摔变 3889 拍摔,
        // 已回退(2026-11-06,记录进 README)。
        let knee_w = thigh_center - Vector::new(0.0, THIGH_LEN / 2.0, 0.0);
        let shank_center = knee_w - Vector::new(0.0, SHANK_LEN / 2.0, 0.0);
        let shank_body = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(shank_center)
                .additional_mass(m(SHANK_MASS)),
        );
        // 脚材质:shank capsule friction 1.4(地面 0.6,rapier 默认平均组合
        // → 接触≈1.0,像橡胶脚垫)。纯材质参数,不改几何;开环小跑的推进
        // 依赖脚底抓地,实测按这个口径报数。
        let _ = world.colliders.insert_with_parent(
            ColliderBuilder::capsule_y(SHANK_LEN / 2.0, SHANK_RADIUS)
                .density(0.0)
                .friction(SHANK_FRICTION)
                .restitution(GROUND_RESTITUTION),
            shank_body,
            &mut world.bodies,
        );
        let knee_joint = RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0))
            .local_anchor1(Vector::new(0.0, -THIGH_LEN / 2.0, 0.0))
            .local_anchor2(Vector::new(0.0, SHANK_LEN / 2.0, 0.0))
            .limits(geom.knee_limits)
            .build();
        let knee_handle = world
            .insert_multibody_joint(thigh_body, shank_body, knee_joint)
            .expect("knee joint");

        // 关节 handle = 第二个刚体的 handle;经 rigid_body_link 拿
        // (MultibodyIndex, link_id)——三关节已并进同一 multibody。
        for (h, body_h) in [
            (lat_handle, lat_body),
            (hip_handle, thigh_body),
            (knee_handle, shank_body),
        ] {
            let _ = h;
            let lid = world
                .multibody_joints
                .rigid_body_link(body_h)
                .expect("link id");
            joints.push((lid.multibody, lid.id));
        }
    }
    Robot { body, joints }
}

/// 每关节的 motors 下标:hip_lat → AngX,hip_pitch / knee → AngZ。
/// (前进 = +x;绕 z 的正角把竖直向下的腿摆向前方(+x),是 pitch;
/// 绕 x 的正角把腿摆向 -z,是 lateral。)
/// 每关节的 motors 下标:一律 AngX。revolute 关节不管世界转轴是 x 还是
/// z,`RevoluteJoint::new(axis)` 都是把"主轴 = local X"用 complete_ang_frame
/// 转到给定世界轴,LOCKED_REVOLUTE_AXES 恒锁 AngY/AngZ、只放 AngX
/// (探针证据:motor_axes=100000、locked=110111 → AngZ 上的电机落在锁定位
/// 上,num_velocity_constraints=0,约束根本不建)。世界轴向由 joint frame 决定。
fn motor_index(_dof: usize) -> usize {
    ANG_X
}

/// 运行时写一个关节的目标(ForceBased PD + 力矩上限),不重建 multibody。
/// 新目标(与上次不同)时唤醒整个 multibody——生产的控制周期(D10)每 2 拍
/// 发一次目标,等价语义是"目标变化即唤醒",否则站立平衡后 island 睡着,
/// 电机约束不再被求解,起步目标永远不生效。
fn set_target(
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
/// 对应轴;直腿初始构型下 coords[AngX/AngZ] 就是关节角( integrates
/// 累积;单自由度关节另一角分量恒 0)。
fn read_angle(world: &MultibodyJointSet, robot: &Robot, dof: usize) -> f32 {
    let (mb_index, link_id) = robot.joints[dof];
    let mb = world.get_multibody(mb_index).expect("multibody alive");
    let link = mb.link(link_id).expect("link alive");
    link.joint.coords()[motor_index(dof)]
}

/// 读一个关节的当前速度(rad/s)。单自由度关节 → joint_velocity[0]。
#[allow(dead_code)]
fn read_vel(world: &MultibodyJointSet, robot: &Robot, dof: usize) -> f32 {
    let (mb_index, link_id) = robot.joints[dof];
    let mb = world.get_multibody(mb_index).expect("multibody alive");
    let link = mb.link(link_id).expect("link alive");
    mb.joint_velocity(link)[0]
}

struct LimbProbe {
    /// 全连杆最低点(世界 y;AC-B2 判据用,经 min_penetration 主导,
    /// min_y 保留用于 README 观察)。
    #[allow(dead_code)]
    min_y: f32,
    max_speed: f32,
    nan: bool,
}

/// 4 只脚(shank 刚体)末端的世界 (x, y)。腿序同 robot.joints
/// (joints 里每腿第 3 项的 multibody link_id = 3、6、9、12 → %3 == 0 且非 0)。
fn robot_colliders_feet(world: &PhysicsWorld, robot: &Robot) -> Vec<(f32, f32)> {
    let mut feet = Vec::new();
    for (mb_index, link_id) in &robot.joints {
        if *link_id == 0 || link_id % 3 != 0 {
            continue; // 只取每腿第 3 个 link(shank)。
        }
        let mb = world
            .multibody_joints
            .get_multibody(*mb_index)
            .expect("alive");
        let link = mb.link(*link_id).expect("alive");
        let rb = world.bodies.get(link.rigid_body_handle()).expect("alive");
        let t = rb.translation();
        feet.push((t.x, t.y - SHANK_LEN / 2.0 - SHANK_RADIUS));
    }
    feet
}

/// 全连杆最低点(世界 y)、最大连杆速度、NaN 探测。
fn probe_limbs(world: &PhysicsWorld, robot: &Robot) -> LimbProbe {
    let body = world.bodies.get(robot.body).expect("body alive");
    let mut min_y = body.translation().y - BODY_HALF[1];
    let mut max_speed = body.linvel().length();
    let mut nan = !body.translation().x.is_finite();
    for (mb_index, link_id) in &robot.joints {
        let mb = world
            .multibody_joints
            .get_multibody(*mb_index)
            .expect("multibody alive");
        let link = mb.link(*link_id).expect("link alive");
        let rb = world.bodies.get(link.rigid_body_handle()).expect("alive");
        let t = rb.translation();
        min_y = min_y.min(t.y - SHANK_RADIUS);
        max_speed = max_speed.max(rb.linvel().length());
        nan |= !t.x.is_finite() || !t.y.is_finite() || !t.z.is_finite();
    }
    LimbProbe {
        min_y,
        max_speed,
        nan,
    }
}

/// 机器人全部 collider 句柄。
fn robot_colliders(world: &PhysicsWorld, robot: &Robot) -> Vec<ColliderHandle> {
    let mut v: Vec<ColliderHandle> = world
        .bodies
        .get(robot.body)
        .expect("alive")
        .colliders()
        .to_vec();
    for (mb_index, link_id) in &robot.joints {
        let mb = world
            .multibody_joints
            .get_multibody(*mb_index)
            .expect("alive");
        let link = mb.link(*link_id).expect("alive");
        let rb = world.bodies.get(link.rigid_body_handle()).expect("alive");
        v.extend_from_slice(rb.colliders());
    }
    v
}

/// 含机器人任一连杆的接触对里最负的 dist(穿透深度,正 = 无穿透)。
fn min_penetration(world: &PhysicsWorld, robot: &Robot) -> f32 {
    let all = robot_colliders(world, robot);
    let mut worst = 0.0f32;
    for pair in world.narrow_phase.contact_graph().interactions() {
        if !all.contains(&pair.collider1) && !all.contains(&pair.collider2) {
            continue;
        }
        if let rapier3d::geometry::PairContacts::Rigid(rigid) = &pair.contacts {
            for manifold in &rigid.manifolds {
                for pt in &manifold.points {
                    worst = worst.min(pt.dist);
                }
            }
        }
    }
    worst
}

// ── 步态 ────────────────────────────────────────────────────────────────

/// 步态种类:对角小跑(正弦反相,主口径)、4 拍慢步(相位表)、
/// 锯齿摆(支撑相匀速后扫;变体,实测起步摔)。
#[derive(Clone, Copy, PartialEq)]
enum GaitKind {
    Trot,
    Crawl,
    Sawtooth,
}

/// 开环步态:hip_pitch 锯齿摆动,knee 支撑相保持站姿折角、摆动相深折
/// 离地。hip_lateral 目标恒 0(只由限位和 PD 兜住)。
#[derive(Clone, Copy)]
struct Gait {
    /// 种类。
    kind: GaitKind,
    /// 频率(Hz)。
    freq: f32,
    /// hip_pitch 摆幅(rad)。
    hip_amp: f32,
    /// knee 摆幅(rad)。
    knee_amp: f32,
    /// knee 站姿目标(全程腿不伸直)。
    knee_stand: f32,
    /// hip_pitch 摆动中心相对站姿的偏移(rad):正 = 整体前倾摆,
    /// 负 = 摆动中心后移(触地点落后于髋,支撑相更长)。
    hip_bias: f32,
    /// 支撑相占比(0..1),其余为摆动相(锯齿摆用)。
    stance_frac: f32,
}

impl Gait {
    /// 站姿目标(站立段):knee 下折、hip_pitch 取 -knee/2 让脚回到
    /// 髋正下方(腿平面在 x–y 内,绕 z 角),脚不承受绕髋的力矩。
    fn stand_target(&self, dof: usize) -> f32 {
        match dof % 3 {
            0 => 0.0,
            1 => -self.knee_stand / 2.0,
            _ => self.knee_stand,
        }
    }
    /// 步态目标(主口径,正弦反相):hip 绕站姿正弦摆,knee 反相——
    /// hip 前摆时折腿、后摆时伸腿。这是 3600 拍全程不摔(up·y ≥ 0.997)
    /// 的稳定口径;锯齿摆(Sawtooth)与 4 拍慢步(Crawl)作为变体另报。
    fn walk_target(&self, t: f32, dof: usize) -> f32 {
        let leg = dof / 3;
        let phase = match self.kind {
            GaitKind::Trot => {
                if leg == 0 || leg == 3 {
                    0.0
                } else {
                    std::f32::consts::PI
                }
            }
            GaitKind::Crawl | GaitKind::Sawtooth => match leg {
                0 => 0.0,
                1 => 0.5,
                2 => 0.75,
                _ => 0.25,
            },
        };
        // 振幅 1 s 渐入:起步满幅目标会把对角支撑对瞬间换成摆动对,
        // 静态不稳的站姿直接前扑(实测起步 60 拍内摔)。
        let ramp = (t / 1.0).min(1.0);
        let wave = match self.kind {
            GaitKind::Sawtooth => {
                // 支撑相(stance_frac)匀速后扫,摆动相快速前回。
                let ph = (self.freq * t + phase).rem_euclid(1.0);
                let sf = self.stance_frac;
                if ph < sf {
                    1.0 - 2.0 * ph / sf
                } else {
                    -1.0 + 2.0 * (ph - sf) / (1.0 - sf)
                }
            }
            _ => (2.0 * std::f32::consts::PI * self.freq * t + phase).sin(),
        };
        let s = wave * ramp;
        let j = dof % 3;
        match j {
            0 => 0.0,
            1 => -self.knee_stand / 2.0 + self.hip_bias + self.hip_amp * s,
            _ => self.knee_stand - self.knee_amp * s,
        }
    }
}

/// 站高几何:hip_pitch 锚点高度 = origin.y − BODY_HALF.y − 0.02;站姿
/// (hip_pitch = −knee/2、knee = knee_stand)下脚底 y = anchor_y −
/// THIGH_LEN·cos(hp) − SHANK_LEN·cos(hp+knee) − SHANK_RADIUS。
/// knee_stand = −0.5 → hp = 0.25:腿下垂 0.11·cos0.25 + 0.12·cos(−0.25)
/// ≈ 0.2337;STAND_HEIGHT 0.33 → anchor_y ≈ 0.577,origin.y ≈ 0.657。
fn spawn_height(knee_stand: f32) -> f32 {
    let hp = -knee_stand / 2.0;
    let leg_drop = THIGH_LEN * hp.cos() + SHANK_LEN * (hp + knee_stand).cos() + SHANK_RADIUS;
    let anchor_y = STAND_HEIGHT + leg_drop;
    anchor_y + BODY_HALF[1] + 0.02
}

// ── 参数组 ──────────────────────────────────────────────────────────────

/// 一组扫描参数。
struct RunCfg {
    name: &'static str,
    dt: f32,
    num_solver_iterations: usize,
    kp: f32,
    kd: f32,
    /// 目标更新周期(拍)。
    update_every: usize,
    mass_scale: f32,
    gait: Gait,
    /// 站立段拍数。
    stand_ticks: usize,
    /// 行走段拍数。
    walk_ticks: usize,
    /// 8 只机器人(耗时测量)。
    eight: bool,
}

/// 一组结果(CSV 行 / README 表列)。
struct RunResult {
    name: String,
    up_y_min: f32,
    height_mean: f32,
    height_std: f32,
    rms_lat: f32,
    rms_pitch: f32,
    rms_knee: f32,
    lag_ticks: f32,
    fwd_speed_mean: f32,
    lateral_drift: f32,
    heading_drift_deg: f32,
    min_penetration: f32,
    nan: bool,
    max_link_speed: f32,
    step_ms_p50: f32,
    step_ms_p95: f32,
    step_ms_max: f32,
    fell_tick: Option<usize>,
    trace_hash: u64,
}

fn percentile(v: &mut [f32], p: f64) -> f32 {
    v.sort_by(|a, b| a.total_cmp(b));
    if v.is_empty() {
        return 0.0;
    }
    let n = v.len();
    let idx = ((n as f64 - 1.0) * p).round() as usize;
    v[idx.min(n - 1)]
}

/// FNV-1a 64 位逐字节。
fn hash_update(h: u64, bytes: &[u8]) -> u64 {
    let mut h = h;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

fn hash_f32(h: u64, v: f32) -> u64 {
    hash_update(h, &v.to_ne_bytes())
}

/// 跑一组。返回结果与逐拍 CSV(机身 y、up·y、前进速度;代表组用)。
#[allow(clippy::disallowed_methods)] // demo 自身要测每拍耗时;不在 workspace lints 下,allow 是纪律自束
fn run(cfg: &RunCfg, keep_trace: bool) -> (RunResult, String) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = production_params(cfg.dt);
    world.integration_parameters.num_solver_iterations = cfg.num_solver_iterations;
    let _ = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0)
            .friction(GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
    );

    let n_bots = if cfg.eight { 8 } else { 1 };
    let mut robots: Vec<Robot> = Vec::new();
    let y0 = spawn_height(cfg.gait.knee_stand);
    for i in 0..n_bots {
        let off = Vector::new((i % 4) as f32 * 2.0, 0.0, (i / 4) as f32 * 2.0);
        robots.push(spawn_robot(
            &mut world,
            Vector::new(0.0, y0, 0.0) + off,
            cfg.mass_scale,
        ));
    }

    let tau = cfg.mass_scale * TAU_LIMIT;
    let kp = cfg.mass_scale * cfg.kp;
    let kd = cfg.mass_scale * cfg.kd;

    let mut up_min = 1.0f32;
    let mut height_sum = 0.0f64;
    let mut height_sq_sum = 0.0f64;
    let mut height_n = 0u64;
    let mut err_lat = 0.0f64;
    let mut err_pitch = 0.0f64;
    let mut err_knee = 0.0f64;
    let mut err_n = 0u64;
    let mut tgt_hist: Vec<f32> = Vec::new();
    let mut act_hist: Vec<f32> = Vec::new();
    let mut vel_sum = 0.0f64;
    let mut vel_n = 0u64;
    let mut start_pos: Option<Vector> = None;
    let mut fell_tick = None;
    let mut step_ms: Vec<f32> = Vec::new();
    let mut max_link_speed = 0.0f32;
    let mut nan = false;
    let mut min_pen = 0.0f32;
    let mut trace_hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut trace_csv = String::new();
    if keep_trace {
        trace_csv.push_str("tick,y,up_y,vx,pen\n");
    }
    let total_ticks = cfg.stand_ticks + cfg.walk_ticks;

    for tick in 0..total_ticks {
        let t0 = Instant::now();

        // 1) 更新关节目标(每 update_every 拍;站立段 = 站姿)。
        if tick % cfg.update_every == 0 {
            let walking = tick >= cfg.stand_ticks;
            let walk_t = if walking {
                (tick - cfg.stand_ticks) as f32 * cfg.dt
            } else {
                0.0
            };
            for robot in &robots {
                for dof in 0..N_JOINTS {
                    let target = if walking {
                        cfg.gait.walk_target(walk_t, dof)
                    } else {
                        cfg.gait.stand_target(dof)
                    };
                    set_target(&mut world, robot, dof, target, kp, kd, tau);
                }
            }
        }

        // 2) 物理一步(60 Hz 不子步,生产求解参数)。
        world.step();

        // 3) 采样。
        let body = world.bodies.get(robots[0].body).expect("alive");
        let up = body.rotation() * Vector::new(0.0, 1.0, 0.0);
        up_min = up_min.min(up.y);
        if fell_tick.is_none() && up.y < 0.7 && tick >= cfg.stand_ticks {
            fell_tick = Some(tick);
        }
        let h = f64::from(body.translation().y);
        height_sum += h;
        height_sq_sum += h * h;
        height_n += 1;

        if tick >= cfg.stand_ticks {
            let pos = body.translation();
            if start_pos.is_none() {
                start_pos = Some(pos);
            }
            vel_sum += f64::from(body.linvel().x);
            vel_n += 1;
            let walk_t = (tick - cfg.stand_ticks) as f32 * cfg.dt;
            for robot in &robots {
                for dof in 0..N_JOINTS {
                    let target = cfg.gait.walk_target(walk_t, dof);
                    let q = read_angle(&world.multibody_joints, robot, dof);
                    let e = f64::from(target - q);
                    match dof % 3 {
                        0 => err_lat += e * e,
                        1 => err_pitch += e * e,
                        _ => err_knee += e * e,
                    }
                    err_n += 1;
                }
                if robots.len() == 1 {
                    tgt_hist.push(cfg.gait.walk_target(walk_t, 1));
                    act_hist.push(read_angle(&world.multibody_joints, robot, 1));
                }
            }
        }

        let probe = probe_limbs(&world, &robots[0]);
        if probe.nan {
            nan = true;
            max_link_speed = probe.max_speed;
            break;
        }
        if tick >= cfg.stand_ticks {
            min_pen = min_pen.min(min_penetration(&world, &robots[0]));
            max_link_speed = max_link_speed.max(probe.max_speed);
        }

        // 确定性 trace:机身位姿(平移 + 四元数)+ 12 关节角逐拍入 hash。
        {
            let rot = body.rotation();
            let tr = body.translation();
            trace_hash = hash_f32(trace_hash, tr.x);
            trace_hash = hash_f32(trace_hash, tr.y);
            trace_hash = hash_f32(trace_hash, tr.z);
            trace_hash = hash_f32(trace_hash, rot.w);
            trace_hash = hash_f32(trace_hash, rot.x);
            trace_hash = hash_f32(trace_hash, rot.y);
            trace_hash = hash_f32(trace_hash, rot.z);
            for dof in 0..N_JOINTS {
                let q = read_angle(&world.multibody_joints, &robots[0], dof);
                trace_hash = hash_f32(trace_hash, q);
            }
        }
        if keep_trace && tick % 60 == 0 {
            let pen = min_penetration(&world, &robots[0]);
            let angles: Vec<String> = (0..N_JOINTS)
                .map(|d| format!("{:.3}", read_angle(&world.multibody_joints, &robots[0], d)))
                .collect();
            let sleeping = world.bodies.get(robots[0].body).expect("a").is_sleeping();
            // 脚(4 只 shank 刚体)世界 x/y:判断支撑相脚是否真的抓地。
            let feet: Vec<String> = robot_colliders_feet(&world, &robots[0])
                .iter()
                .map(|(x, y)| format!("{x:.3}/{y:.3}"))
                .collect();
            trace_csv.push_str(&format!(
                "{},{:.6},{:.6},{:.6},{:.6},sleep={},joints=[{}],feet=[{}]\n",
                tick,
                body.translation().y,
                up.y,
                body.linvel().x,
                pen,
                sleeping,
                angles.join(" "),
                feet.join(" ")
            ));
        }

        step_ms.push(t0.elapsed().as_secs_f32() * 1000.0);
    }

    let height_mean = (height_sum / (height_n as f64)) as f32;
    let height_std = (height_sq_sum / (height_n as f64)
        - f64::from(height_mean) * f64::from(height_mean))
    .max(0.0)
    .sqrt() as f32;
    let den = err_n.max(1) as f64;
    let (rms_lat, rms_pitch, rms_knee) = (
        (err_lat / den).sqrt() as f32,
        (err_pitch / den).sqrt() as f32,
        (err_knee / den).sqrt() as f32,
    );
    // 相位滞后:互相关,滞后 0..2 周期(拍)内找峰。
    let lag_ticks = cross_corr_lag(&tgt_hist, &act_hist, cfg.dt);
    let fwd_speed_mean = (vel_sum / (vel_n.max(1) as f64)) as f32;
    let end = world
        .bodies
        .get(robots[0].body)
        .expect("alive")
        .translation();
    let lateral_drift = start_pos.map_or(0.0, |s| end.z - s.z);
    let fwd_drift = start_pos.map_or(0.0, |s| end.x - s.x);
    let heading = world.bodies.get(robots[0].body).expect("alive").rotation();
    let fwd_dir = heading * Vector::new(1.0, 0.0, 0.0);
    let heading_drift_deg = fwd_dir.z.atan2(fwd_dir.x).to_degrees();
    let _ = fwd_drift;

    let res = RunResult {
        name: cfg.name.to_string(),
        up_y_min: up_min,
        height_mean,
        height_std,
        rms_lat,
        rms_pitch,
        rms_knee,
        lag_ticks,
        fwd_speed_mean,
        lateral_drift,
        heading_drift_deg,
        min_penetration: min_pen,
        nan,
        max_link_speed,
        step_ms_p50: percentile(&mut step_ms, 0.5),
        step_ms_p95: percentile(&mut step_ms, 0.95),
        step_ms_max: step_ms.iter().copied().fold(0.0, f32::max),
        fell_tick,
        trace_hash,
    };
    (res, trace_csv)
}

/// 互相关求相位滞后(拍):把目标正弦和实际角在滞后 0..len/2 内扫,
/// 取相关最大处。数据不足返回 NaN → 0。
fn cross_corr_lag(tgt: &[f32], act: &[f32], dt: f32) -> f32 {
    let n = tgt.len().min(act.len());
    if n < 60 {
        return 0.0;
    }
    let mut best_lag = 0usize;
    let mut best = f32::NEG_INFINITY;
    let max_lag = n / 2;
    for lag in 0..max_lag {
        let mut dot = 0.0f32;
        for i in lag..n {
            dot += tgt[i] * act[i - lag];
        }
        if dot > best {
            best = dot;
            best_lag = lag;
        }
    }
    best_lag as f32 * dt * 60.0 // 拍数(60 Hz 基准下 1 拍 = 1/(60·dt) 拍…)
}

fn csv_escape(s: &str) -> String {
    if s.contains(',') {
        format!("\"{s}\"")
    } else {
        s.to_string()
    }
}

fn result_header() -> String {
    "name,up_y_min,height_mean,height_std,rms_hip_lat,rms_hip_pitch,rms_knee,\
     lag_ticks,fwd_speed_mean,lateral_drift,heading_drift_deg,min_penetration,\
     nan,max_link_speed,step_ms_p50,step_ms_p95,step_ms_max,fell_tick,trace_hash"
        .to_string()
}

fn result_csv(r: &RunResult) -> String {
    let hash = format!("{:016x}", r.trace_hash);
    format!(
        "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.1},{:.4},{:.4},{:.2},{:.4},{},{:.3},\
         {:.3},{:.3},{:.3},{},{}",
        csv_escape(&r.name),
        r.up_y_min,
        r.height_mean,
        r.height_std,
        r.rms_lat,
        r.rms_pitch,
        r.rms_knee,
        r.lag_ticks,
        r.fwd_speed_mean,
        r.lateral_drift,
        r.heading_drift_deg,
        r.min_penetration,
        r.nan,
        r.max_link_speed,
        r.step_ms_p50,
        r.step_ms_p95,
        r.step_ms_max,
        r.fell_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        hash,
    )
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();

    // 扫描矩阵(放行表:kp/kd 至少 §9.1 两组 + 2–3 组;更新 2 拍/1 拍;
    // 质量两档;参照 240 Hz、60 Hz 加迭代)。
    let mut all: Vec<RunCfg> = Vec::new();
    let mk = |name: &'static str, kp: f32, kd: f32, ue: usize, ms: f32| RunCfg {
        name,
        dt: 1.0 / 60.0,
        num_solver_iterations: 4,
        kp,
        kd,
        update_every: ue,
        mass_scale: ms,
        gait: Gait {
            kind: GaitKind::Trot,
            freq: 1.0,
            hip_amp: 0.18,
            knee_amp: 0.15,
            knee_stand: -0.5,
            hip_bias: 0.0,
            stance_frac: 0.55,
        },
        stand_ticks: 600,
        walk_ticks: 3600,
        eight: false,
    };
    for (kp, kd) in [
        (20.0f32, 0.5f32),
        (60.0, 2.0),
        (80.0, 3.0),
        (40.0, 1.0),
        (120.0, 4.0),
    ] {
        for ue in [2usize, 1] {
            for (ms, tag) in [(1.0f32, "real"), (0.01, "x001")] {
                let name = Box::leak(format!("kp{kp}_kd{kd}_u{ue}_{tag}").into_boxed_str());
                all.push(mk(name, kp, kd, ue, ms));
            }
        }
    }
    // 步态变体(步态设计是本次 demo 的自选维度;基线 1.5 Hz/0.25/0.2/bias0
    // 正弦反相 = 主口径):高频、后移摆心(bias −0.15)、高频+后移,
    // 各配 kp20 与 kp60(30 Hz 更新);锯齿摆与 4 拍慢步各一组。
    let gait_variants: [(&'static str, f32, f32, f32, f32); 4] = [
        ("f2", 2.0, 0.25, 0.2, 0.0),
        ("b15", 1.5, 0.25, 0.2, -0.15),
        ("f2b15", 2.0, 0.25, 0.2, -0.15),
        ("f2b15a35", 2.0, 0.35, 0.28, -0.15),
    ];
    for (tag, freq, hip, knee, bias) in gait_variants {
        for kp0 in [20.0f32, 60.0f32] {
            let kd0 = if kp0 == 20.0 { 0.5 } else { 2.0 };
            let name = Box::leak(format!("gait-{tag}_kp{kp0}_u2_real").into_boxed_str());
            let mut c = mk(name, kp0, kd0, 2, 1.0);
            c.gait.freq = freq;
            c.gait.hip_amp = hip;
            c.gait.knee_amp = knee;
            c.gait.hip_bias = bias;
            all.push(c);
        }
    }
    // 4 拍慢步(静态稳定)与锯齿摆:变体,如实报数(锯齿起步摔,
    // 见 README"试过什么")。
    for (tag, kind, freq, amp, sf) in [
        ("crawl1", GaitKind::Crawl, 1.0f32, 0.18f32, 0.75f32),
        ("crawl05", GaitKind::Crawl, 0.5, 0.18, 0.75),
        ("saw", GaitKind::Sawtooth, 1.0, 0.25, 0.7),
    ] {
        for kp0 in [20.0f32, 60.0f32] {
            let kd0 = if kp0 == 20.0 { 0.5 } else { 2.0 };
            let name = Box::leak(format!("gait-{tag}_kp{kp0}_u2_real").into_boxed_str());
            let mut c = mk(name, kp0, kd0, 2, 1.0);
            c.gait.kind = kind;
            c.gait.freq = freq;
            c.gait.hip_amp = amp;
            c.gait.stance_frac = sf;
            all.push(c);
        }
    }
    // 参照组:240 Hz;60 Hz 加大迭代(只对比,不算通过)。
    {
        let name = Box::leak("ref_240hz_kp20".to_string().into_boxed_str());
        let mut c = mk(name, 20.0, 0.5, 2, 1.0);
        c.dt = 1.0 / 240.0;
        all.push(c);
        let name = Box::leak("ref_iter16_kp20".to_string().into_boxed_str());
        let mut c = mk(name, 20.0, 0.5, 2, 1.0);
        c.num_solver_iterations = 16;
        all.push(c);
    }
    // 8 只耗时组(60 Hz 基准,真实尺度,kp20)。
    {
        let name = Box::leak("eight_kp20_u2_real".to_string().into_boxed_str());
        let mut c = mk(name, 20.0, 0.5, 2, 1.0);
        c.eight = true;
        all.push(c);
    }

    let only: Vec<&String> = if args.is_empty() {
        Vec::new()
    } else {
        args.iter().collect()
    };
    let det_mode = only.len() == 2 && only[0] == "det";

    let mut out = String::new();
    out.push_str(&result_header());
    out.push('\n');
    for cfg in &all {
        let selected = if det_mode {
            cfg.name == only[1].as_str()
        } else if only.is_empty() {
            !cfg.eight // 默认跑全部单只组(含参照)
        } else {
            only.iter().any(|a| a.as_str() == cfg.name)
        };
        if !selected {
            continue;
        }
        let keep = cfg.name.starts_with("kp20_kd0.5_u2_real")
            || cfg.name.starts_with("kp60_kd2_u2_real")
            || cfg.name.starts_with("gait-crawl1_kp20");
        let (res, trace) = run(cfg, keep);
        println!("{}", result_csv(&res));
        out.push_str(&result_csv(&res));
        out.push('\n');
        if !trace.is_empty() {
            let path = format!("out/trace_{}.csv", cfg.name);
            let _ = std::fs::write(&path, &trace);
        }
        // det 模式:同参数跑第二遍,逐位比较 hash。
        if det_mode {
            let (res2, _) = run(cfg, false);
            let same = res.trace_hash == res2.trace_hash;
            println!(
                "determinism: {} vs {} -> {}",
                res.trace_hash,
                res2.trace_hash,
                if same { "IDENTICAL" } else { "DIFFER" }
            );
        }
    }
    let _ = std::fs::create_dir_all("out");
    let _ = std::fs::write("out/summary.csv", &out);
    // 防未使用告警:FRAC_PI_2 在限位推导里被引用过;保留 import 稳定性。
    let _ = FRAC_PI_2;
}
