//! REQ-012 A00-2 demo `mass`(2026-11-06;A00-2 R1 重做布景时序、M1 网格、
//! M7 重物 + guard)。
//!
//! 问题(放行原文):方块质量固定为 1(不能改),主体质量比方块小很多时,
//! 方块砸在身上、压在身上或主体站在方块堆上,求解器会不会穿透或炸开?
//!
//! 方块与生产一致:sim.rs ~:4266 的单格实体路径——`cuboid(0.5, 0.5, 0.5)`,
//! density 1、friction 0.6、restitution 0.1(质量恰 1)。
//!
//! R1 修正(验收评论 10 条 + 放行 R1-1…R1-8):
//! - **方块在第 180 拍(站稳后)才生成**(R1-1),按当时机身实际位姿定位;
//!   旧版第 0 拍生成,主体还在直腿出生姿态,M3/M4 出生重叠 17 cm、M2 嵌地
//!   22 cm、M1 撞在下蹲段;
//! - **M1 改网格**(R1-2):ox、oz ∈ {−0.3,−0.15,0,0.15,0.3},5/10 m,
//!   每格 2 落点组合共 25 组/高度;**M1Leg z=−0.56**(方块边缘不盖机身),
//!   报首碰连杆与被推距离(R1-1);
//! - **比例扫描**(R1-3):s ∈ {1/1500, 1/500, 1/200, 1/100, 1/50, 1/20, 1}
//!   跑 M1Body 网格;
//! - **panic 隔离**(R1-4):每场景独立 catch_unwind + 每步 guard 检查,
//!   单次 panic 不中断矩阵;
//! - **全部连杆**计入速度统计(机身、髋、大腿、小腿;R1 通用要求);
//! - 穿透分"对地"与"对方块",动态/静置分开,静置窗口 = 末 120 拍,
//!   并报动态穿透是否 ≤ v·dt(一拍行程);
//! - **M5 双变体**(R1-5):自然睡眠(现状)+ 强制不睡眠(堆 9×9),
//!   修正 m5_settle_* 窗口上界(旧版把行走段也算进静置);
//! - **M7 重物**(R1-6):整块 cuboid(h,h,h),h ∈ {1,2,4},5 m,
//!   s ∈ {1/100, 1/20, 1/5, 1};**guard 防护验证**(R1-7):V_GUARD =
//!   1000 / 200 m/s,M7 与 real/1-500 档的 M1 网格。

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::time::Instant;

use quad::{
    default_gait, ik, production_params, read_angle, set_target, spawn_robot_yawed, DampMode,
    GROUND_RESTITUTION, N_JOINTS, TAU_LIMIT,
};
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::{ColliderBuilder, PairContacts};
use rapier3d::math::{Pose, Vector};
use rapier3d::pipeline::PhysicsWorld;

/// 地面顶面(平地口径,同 gait:cuboid 顶面 = 0)。
const GROUND_TOP: f32 = 0.0;
/// 方块尺寸与材质(生产:sim.rs ~:4266)。
const BLOCK_HALF: f32 = 0.5;
/// 站立序列结束拍(1 s 直腿降 + 2 s 稳)。
const STAND_TICKS: usize = 180;
/// guard 速度阈值(R1-7:1000 与 200 各跑一遍)。
const V_GUARDS: [f32; 2] = [1000.0, 200.0];

/// 质量档(§4 三档 + R1-3 扫描)。
#[derive(Clone, Copy, PartialEq)]
struct Scale {
    s: f32,
}

impl Scale {
    fn tag(self) -> String {
        if (self.s - 1.0 / 1500.0).abs() < 1e-9 {
            "real".into()
        } else if (self.s - 1.0 / 100.0).abs() < 1e-9 {
            "comp".into()
        } else if (self.s - 1.0).abs() < 1e-9 {
            "none".into()
        } else {
            format!("s{}", format_den(self.s))
        }
    }
}

/// s 的可读分母(1/s 为整数时)。
fn format_den(s: f32) -> String {
    let inv = 1.0 / s;
    if (inv - inv.round()).abs() < 1e-4 {
        format!("1-{}", inv.round() as i64)
    } else {
        format!("{:.4}", s)
    }
}

/// 场景。
#[derive(Clone, Copy, PartialEq)]
enum Scene {
    /// 砸机身(网格落点,5 m / 10 m)。
    M1Body5,
    M1Body10,
    /// 砸腿(z = −0.56,方块边缘不盖机身)。
    M1Leg5,
    M1Leg10,
    /// 侧撞 10 m/s(方块中心 y = 0.51,不嵌地)。
    M2,
    /// 1 个方块压背(底面贴机身顶面上方 2 mm),静置 600 拍。
    M3One,
    /// 4 个方块(2×2)压背,静置 600 拍。
    M3Four,
    /// M3One + 600 拍后移除方块,再看 600 拍。
    M4,
    /// 3×3 方块堆站上去:静置 3600 拍 + 走 1200 拍;自然睡眠。
    M5,
    /// M5 强制不睡眠变体:堆 9×9(否则 60 s 内滑出堆),目标不变。
    M5Awake,
    /// 走向 1 个静止方块,走 1200 拍。
    M6,
    /// 重物:整块 cuboid(h,h,h) 从 5 m 砸,h ∈ {1,2,4}。
    M7 {
        h: f32,
        ox: f32,
        oz: f32,
    },
}

impl Scene {
    fn name(self) -> String {
        match self {
            Scene::M1Body5 => "M1Body5".into(),
            Scene::M1Body10 => "M1Body10".into(),
            Scene::M1Leg5 => "M1Leg5".into(),
            Scene::M1Leg10 => "M1Leg10".into(),
            Scene::M2 => "M2".into(),
            Scene::M3One => "M3One".into(),
            Scene::M3Four => "M3Four".into(),
            Scene::M4 => "M4".into(),
            Scene::M5 => "M5".into(),
            Scene::M5Awake => "M5Awake".into(),
            Scene::M6 => "M6".into(),
            Scene::M7 { h, ox, oz } => format!("M7_h{}_ox{}_oz{}", h, ox, oz),
        }
    }

    fn total_ticks(self) -> usize {
        match self {
            Scene::M1Body5
            | Scene::M1Body10
            | Scene::M1Leg5
            | Scene::M1Leg10
            | Scene::M2
            | Scene::M7 { .. } => STAND_TICKS + 900,
            Scene::M3One | Scene::M3Four => STAND_TICKS + 600,
            Scene::M4 => STAND_TICKS + 600 + 600,
            Scene::M5 | Scene::M5Awake => STAND_TICKS + 3600 + 1200,
            Scene::M6 => STAND_TICKS + 1200,
        }
    }
}

/// M1 网格落点(R1-2:ox、oz ∈ {−0.3,−0.15,0,0.15,0.3})。
const M1_GRID: [f32; 5] = [-0.3, -0.15, 0.0, 0.15, 0.3];

struct Cfg {
    name: String,
    scene: Scene,
    scale: Scale,
    /// guard 阈值(None = 不防护)。
    v_guard: Option<f32>,
}

struct Outcome {
    name: String,
    nan: bool,
    panicked: bool,
    panic_msg: String,
    /// guard 移除次数(R1-7)。
    guard_removed: u32,
    /// 连杆单拍峰值(全部连杆 + 机身)。
    max_link_speed: f32,
    /// 连杆速度超限的拍数(判据 2 暂保留口径;V_LIMIT = max(20, 2×撞击))。
    link_over_ticks: u64,
    link_v_limit: f32,
    /// 机身最大速度。
    max_body_speed: f32,
    max_block_speed: f32,
    max_block_upthrow: f32,
    /// 撞后机身最高点(M1:撞后 300 拍内)。
    post_impact_body_max_y: f32,
    max_pen_body_block: f32,
    max_pen_body_ground: f32,
    /// 静置窗口(末 120 拍)穿透。
    static_pen_bb: f32,
    static_pen_bg: f32,
    /// 动态穿透是否 ≤ v·dt(一拍行程;只在有 body-block 接触的拍估)。
    dyn_pen_within_step: Option<bool>,
    torque_sat_ratio: f32,
    end_body_y: f32,
    end_up_y: f32,
    block_disp_max: f32,
    /// 方块撞后上抛(相对落定位置;落定 = 末 120 拍均值)。
    block_upthrow_after_settle: f32,
    /// 方块反弹速度(撞后首拍速度,M1 系列)。
    block_rebound_speed: f32,
    anomaly_tick: Option<usize>,
    m5_height_change: f32,
    /// M5:睡眠拍占比(自然睡眠变体)。
    m5_sleep_ratio: f32,
    /// M1Leg:方块先碰到的连杆(body / hip / thigh / shank / foot;无 = -)。
    first_hit_link: String,
    /// M1Leg:主体被推开的水平距离(方块生成时位置 → 结束位置,m)。
    pushed_dist: f32,
    /// M5Awake:滑移速度(静置段机身水平速度均值)。
    m5awake_slide: f32,
    state_hash: u64,
}

fn hash_f32(h: u64, v: f32) -> u64 {
    let mut h = h;
    for b in &v.to_ne_bytes() {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

/// 与生产一致的单格方块(density 1、friction 0.6、restitution 0.1)。
fn spawn_block(
    world: &mut PhysicsWorld,
    pos: Vector,
    vel: Vector,
    size: f32,
) -> rapier3d::dynamics::RigidBodyHandle {
    world
        .insert(
            RigidBodyBuilder::dynamic()
                .pose(Pose {
                    translation: pos,
                    ..Pose::IDENTITY
                })
                .linvel(vel),
            ColliderBuilder::cuboid(size * 0.5, size * 0.5, size * 0.5)
                .density(1.0)
                .friction(0.6)
                .restitution(0.1),
        )
        .0
}

fn run_once(cfg: &Cfg) -> Result<Outcome, String> {
    match catch_unwind(AssertUnwindSafe(|| run_inner(cfg))) {
        Ok(o) => Ok(o),
        Err(e) => {
            let msg = e
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_else(|| "non-string panic".into());
            Err(msg)
        }
    }
}

fn run_inner(cfg: &Cfg) -> Outcome {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = production_params(1.0 / 60.0);
    let _ = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, GROUND_TOP - 0.5, 0.0)),
        ColliderBuilder::cuboid(200.0, 0.5, 200.0)
            .friction(quad::GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
    );

    let s = cfg.scale.s;
    let gait = default_gait();
    let pile_top = if matches!(cfg.scene, Scene::M5 | Scene::M5Awake) {
        1.0f32
    } else {
        0.0
    };
    let y0 = GROUND_TOP + pile_top + 2.0 * quad::L + quad::FOOT_R + 0.002;
    let mut leg_spawn_pos = Vector::ZERO;
    let robot = spawn_robot_yawed(
        &mut world,
        Vector::new(0.0, y0, 0.0),
        quad::yaw_quat(0.0),
        s,
        quad::GROUND_FRICTION,
        DampMode::Scaled,
    );
    if matches!(cfg.scene, Scene::M1Leg5 | Scene::M1Leg10) {
        leg_spawn_pos = world.bodies.get(robot.body).expect("alive").translation();
    }

    let tau = s * TAU_LIMIT;
    let kp = s * quad::DEFAULT_KP;
    let kd = s * quad::DEFAULT_KD;
    let walk0 = match cfg.scene {
        Scene::M5 | Scene::M5Awake => STAND_TICKS + 3600,
        Scene::M6 => STAND_TICKS,
        Scene::M4 => STAND_TICKS + 600,
        _ => STAND_TICKS,
    };
    // M5Awake:静置段(在行走前)就强制醒;堆 9×9(R1-5)。
    let awake = cfg.scene == Scene::M5Awake;
    let pile_grid: i32 = if awake { 4 } else { 1 }; // ±4 格 → 9×9

    // 事件方块句柄(延迟生成;M5/M6 在第 0 拍生成——主体从堆顶出生是布景,
    // M6 的方块在地面上,都不需要"站稳后")。
    let mut blocks: Vec<rapier3d::dynamics::RigidBodyHandle> = Vec::new();
    let mut m5_pile: Vec<rapier3d::dynamics::RigidBodyHandle> = Vec::new();
    // 方块出生位置(位移参照;延迟生成的在生成拍记录)。
    let mut blocks_spawn_pos: Vec<Vector> = Vec::new();
    let mut pile_spawn_pos: Vec<Vector> = Vec::new();
    let spawn_tick = STAND_TICKS;
    match cfg.scene {
        Scene::M5 | Scene::M5Awake => {
            // 地面 NxN 静止方块堆(中心在出生点),主体出生在堆顶上方。
            for dx in -pile_grid..=pile_grid {
                for dz in -pile_grid..=pile_grid {
                    let pos = Vector::new(dx as f32, GROUND_TOP + BLOCK_HALF, dz as f32);
                    m5_pile.push(spawn_block(&mut world, pos, Vector::ZERO, 1.0));
                    pile_spawn_pos.push(pos);
                }
            }
        }
        Scene::M6 => {
            blocks.push(spawn_block(
                &mut world,
                Vector::new(2.0, GROUND_TOP + BLOCK_HALF, 0.0),
                Vector::ZERO,
                1.0,
            ));
            blocks_spawn_pos.push(Vector::new(2.0, 0.0, 0.0));
        }
        _ => {}
    }

    let mut targets = [0.0f32; N_JOINTS];
    let rise_ticks = 60usize;

    let mut nan = false;
    let mut guard_removed = 0u32;
    let mut robot_alive = true;
    let mut max_link_speed = 0.0f32;
    let mut link_over_ticks = 0u64;
    let mut max_body_speed = 0.0f32;
    let mut max_block_speed = 0.0f32;
    let mut max_upthrow = 0.0f32;
    let mut post_impact_body_max_y = 0.0f32;
    let mut max_pen_bb = 0.0f32;
    let mut max_pen_bg = 0.0f32;
    let mut sat_n = 0u64;
    let mut sat_total = 0u64;
    let mut anomaly = None;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut block_disp_max = 0.0f32;
    let m5_settle_start = STAND_TICKS;
    let m5_settle_end = STAND_TICKS + 3600;
    let mut m5_h_start: Option<f32> = None;
    let mut m5_h_end = 0.0f32;
    let mut m5_sleep_ticks = 0u64;
    let mut m5_settle_slide_sum = 0.0f64;
    let mut m5_settle_slide_n = 0u64;
    let static_window = 120usize;
    let mut static_pen_bb = 0.0f32;
    let mut static_pen_bg = 0.0f32;
    let mut dyn_pen_within_step: Option<bool> = None;
    // M1 系列:撞击后窗口(方块生成后 300 拍)机身最高点、方块反弹。
    let mut block_rebound_speed = 0.0f32;
    let mut block_rebound_caught = false;
    // M1Leg:首碰连杆与被推距离。
    let mut first_hit_link: Option<&'static str> = None;
    let mut pushed_dist = 0.0f32; // M1Leg:结尾按机身位移赋值
    let mut block_settle_y_sum = 0.0f64;
    let mut block_settle_y_n = 0u64;
    let mut block_spawn_y = 0.0f32;
    let total_ticks = cfg.scene.total_ticks();
    let mut v_limit = 20.0f32; // 撞击速度 2 倍口径;M1 落体末速 ×2
    if matches!(
        cfg.scene,
        Scene::M1Body5 | Scene::M1Leg5 | Scene::M7 { h: _, ox: _, oz: _ }
    ) {
        v_limit = (2.0 * (2.0f32 * 9.81f32 * 5.0f32).sqrt()).max(20.0f32); // ≈ 20.4
    }

    for tick in 0..total_ticks {
        // 方块延迟生成(R1-1):站稳后(第 180 拍)按当时机身实际位姿定位。
        if tick == spawn_tick && blocks.is_empty() && !m5_pile.is_empty() {
            // M5 系列无延迟方块;这里只对延迟场景生效(见下方 match)。
        }
        if tick == spawn_tick {
            let body = world.bodies.get(robot.body).expect("alive");
            let bp = body.translation();
            let body_top = bp.y + quad::BODY_HALF[1];
            match cfg.scene {
                Scene::M1Body5 | Scene::M1Body10 => {
                    let h = if cfg.scene == Scene::M1Body10 {
                        10.0
                    } else {
                        5.0
                    };
                    // M1 网格落点(R1-2): ox、oz 来自名字(M1Body5_ox-0.15_oz0.3)。
                    let (ox, oz) = m1_offsets(&cfg.name);
                    // 末速口径:上限按实际落距(机身顶面)。
                    let drop = body_top + BLOCK_HALF + h - (body_top + BLOCK_HALF);
                    v_limit = (2.0 * (2.0f32 * 9.81f32 * h).sqrt()).max(20.0f32);
                    let _ = drop;
                    blocks.push(spawn_block(
                        &mut world,
                        Vector::new(bp.x + ox, body_top + BLOCK_HALF + h, bp.z + oz),
                        Vector::ZERO,
                        1.0,
                    ));
                    blocks_spawn_pos.push(Vector::new(bp.x + ox, 0.0, bp.z + oz));
                }
                Scene::M1Leg5 | Scene::M1Leg10 => {
                    let h = if cfg.scene == Scene::M1Leg10 {
                        10.0
                    } else {
                        5.0
                    };
                    v_limit = (2.0 * (2.0f32 * 9.81f32 * h).sqrt()).max(20.0f32);
                    // R1-1:z = −0.56(方块边缘 0.06 不盖机身;z=−THIGH_OFF 时
                    // 1 m 方块盖住机身)。撞的是 FL 腿小腿段。
                    blocks.push(spawn_block(
                        &mut world,
                        Vector::new(bp.x, body_top + BLOCK_HALF + h, bp.z - 0.56),
                        Vector::ZERO,
                        1.0,
                    ));
                    blocks_spawn_pos.push(Vector::new(bp.x, 0.0, bp.z - 0.56));
                }
                Scene::M2 => {
                    // R1-1:中心 y = 0.51(底面 y=0.01,不嵌地);水平 10 m/s。
                    blocks.push(spawn_block(
                        &mut world,
                        Vector::new(-3.0, 0.51, 0.0),
                        Vector::new(10.0, 0.0, 0.0),
                        1.0,
                    ));
                    blocks_spawn_pos.push(Vector::new(-3.0, 0.0, 0.0));
                    v_limit = 20.0;
                }
                Scene::M3One | Scene::M4 => {
                    // R1-1:底面贴当时机身顶面上方 2 mm。
                    blocks.push(spawn_block(
                        &mut world,
                        Vector::new(bp.x, body_top + BLOCK_HALF + 0.002, bp.z),
                        Vector::ZERO,
                        1.0,
                    ));
                    blocks_spawn_pos.push(Vector::new(bp.x, 0.0, bp.z));
                }
                Scene::M3Four => {
                    // 2×2 平铺当时机身顶面。
                    for dx in [-0.5f32, 0.5] {
                        for dz in [-0.5f32, 0.5] {
                            blocks.push(spawn_block(
                                &mut world,
                                Vector::new(bp.x + dx, body_top + BLOCK_HALF + 0.002, bp.z + dz),
                                Vector::ZERO,
                                1.0,
                            ));
                            blocks_spawn_pos.push(Vector::new(bp.x + dx, 0.0, bp.z + dz));
                        }
                    }
                }
                Scene::M7 { h, ox, oz } => {
                    // R1-6:整块 cuboid(h,h,h) 从 5 m(底面距机身顶面 5 m)。
                    let _ = oz;
                    blocks.push(spawn_block(
                        &mut world,
                        Vector::new(bp.x + ox, body_top + h * 0.5 + 5.0, bp.z),
                        Vector::ZERO,
                        h,
                    ));
                    blocks_spawn_pos.push(Vector::new(bp.x + ox, 0.0, bp.z));
                    v_limit = (2.0 * (2.0f32 * 9.81f32 * 5.0f32).sqrt()).max(20.0f32);
                }
                Scene::M5 | Scene::M5Awake | Scene::M6 => {}
            }
            if !blocks.is_empty() {
                block_spawn_y = world.bodies.get(blocks[0]).expect("alive").translation().y;
            }
        }

        // M4:方块生成 600 拍后移除(R1-1 时序)。
        if cfg.scene == Scene::M4 && tick == spawn_tick + 600 {
            for h in blocks.drain(..) {
                world.remove_body(h);
            }
        }

        // 关节目标:站立序列;M5 系列在静置 3600 拍后行走,M6 站稳即走。
        // M5Awake:目标恒站姿(不走)。guard 移除主体后不再写目标
        // (multibody 已不在,R1-7:panic 必须归零)。
        if robot_alive && tick % 2 == 0 {
            for leg in 0..4 {
                let foot = if tick < rise_ticks {
                    let a = tick as f32 / rise_ticks as f32;
                    let sm = a * a * (3.0 - 2.0 * a);
                    let (x, _) = gait.stand_foot();
                    (
                        x,
                        quad::FULL_STRETCH_DY + (-gait.h - quad::FULL_STRETCH_DY) * sm,
                    )
                } else if tick < walk0 || !matches!(cfg.scene, Scene::M5 | Scene::M6) {
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
                set_target(&mut world, &robot, dof, *tgt, kp, kd, tau);
            }
        }
        // M5Awake:每拍强制唤醒(R1-5 变体二)。
        if awake && robot_alive && tick >= STAND_TICKS {
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

        // M5 判据 4/睡眠统计:静置段首拍睡眠占比。
        if cfg.scene == Scene::M5 && tick >= m5_settle_start {
            let body = world.bodies.get(robot.body).expect("alive");
            if body.is_sleeping() {
                m5_sleep_ticks += 1;
            }
        }

        world.step();

        // guard 检查(R1-7):非有限或速度超 V_GUARD → 整体移除,世界继续。
        // 先检查再采样:移除后本拍跳过主体统计。
        if let (true, Some(v_guard)) = (robot_alive, cfg.v_guard) {
            let mut kill = false;
            for lh in robot
                .links
                .iter()
                .copied()
                .chain(std::iter::once(robot.body))
            {
                if let Some(rb) = world.bodies.get(lh) {
                    let v = rb.linvel();
                    if !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite() {
                        nan = true;
                        kill = true;
                    } else if v.length() > v_guard {
                        kill = true;
                    }
                }
            }
            if kill {
                // 整体移除:multibody 全部刚体(quad 模型 = 根 + 12 链刚体,
                // 经 multibody_joints 的 rigid_body_link 全覆盖)。
                let mut all_handles: Vec<_> = robot.links.clone();
                all_handles.push(robot.body);
                for h in all_handles {
                    world.remove_body(h);
                }
                robot_alive = false;
                guard_removed += 1;
            }
        }
        if !robot_alive {
            // 世界继续跑(方块正常落定);主体统计冻结。
            continue;
        }

        let body = world.bodies.get(robot.body).expect("alive");
        let p = body.translation();
        if !p.x.is_finite() || !p.y.is_finite() {
            nan = true;
            if anomaly.is_none() {
                anomaly = Some(tick);
            }
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
            hash = hash_f32(hash, read_angle(&world.multibody_joints, &robot, dof));
        }

        // 速度统计:全部连杆 + 机身(R1 通用要求)。
        let mut tick_link_max = 0.0f32;
        for lh in robot
            .links
            .iter()
            .copied()
            .chain(std::iter::once(robot.body))
        {
            let rb = world.bodies.get(lh).expect("alive");
            let v = rb.linvel().length();
            tick_link_max = tick_link_max.max(v);
            max_link_speed = max_link_speed.max(v);
        }
        if tick_link_max > v_limit {
            link_over_ticks += 1;
        }
        max_body_speed = max_body_speed.max(body.linvel().length());

        // 方块速度 / 上抛 / 位移;M1 撞后窗口。
        let in_post_window = tick >= spawn_tick && tick < spawn_tick + 300;
        if in_post_window {
            post_impact_body_max_y = post_impact_body_max_y.max(p.y);
        }
        let mut cur_block_disp = 0.0f32;
        for (i, h) in blocks.iter().enumerate() {
            let rb = world.bodies.get(*h).expect("alive");
            let v = rb.linvel().length();
            max_block_speed = max_block_speed.max(v);
            max_upthrow = max_upthrow.max(rb.translation().y - block_spawn_y);
            let sp = blocks_spawn_pos[i];
            let d = rb.translation() - sp;
            cur_block_disp = cur_block_disp.max((d.x * d.x + d.z * d.z).sqrt());
            // M1:反弹速度 = 方块生成后首次上行速度。
            if !block_rebound_caught && rb.linvel().y > 0.05 && tick > spawn_tick {
                block_rebound_caught = true;
                block_rebound_speed = v;
            }
            // 方块落定位置(末 120 拍 y 均值)→ 撞后上抛 = max_y − 落定 y。
            if tick >= total_ticks.saturating_sub(static_window) {
                block_settle_y_sum += f64::from(rb.translation().y);
                block_settle_y_n += 1;
            }
            let _ = i;
        }
        for (i, h) in m5_pile.iter().enumerate() {
            let rb = world.bodies.get(*h).expect("alive");
            max_block_speed = max_block_speed.max(rb.linvel().length());
            let d = rb.translation() - pile_spawn_pos[i];
            cur_block_disp = cur_block_disp.max((d.x * d.x + d.y * d.y + d.z * d.z).sqrt());
        }
        block_disp_max = block_disp_max.max(cur_block_disp);

        // 穿透:主体侧(全部连杆+机身)对地、对方块;动态/静置分开。
        for pair in world.narrow_phase.contact_graph().interactions() {
            if let PairContacts::Rigid(rigid) = &pair.contacts {
                let c1 = pair.collider1;
                let c2 = pair.collider2;
                let is_robot = |c| robot.colliders.contains(&c);
                if !(is_robot(c1) || is_robot(c2)) {
                    continue;
                }
                let other = if is_robot(c1) { c2 } else { c1 };
                let other_rb = world
                    .colliders
                    .get(other)
                    .and_then(|co| co.parent())
                    .and_then(|ph| world.bodies.get(ph));
                let is_ground = other_rb.map(|rb| rb.is_fixed()).unwrap_or(true);
                // M1Leg 首碰连杆:方块生成后第一次 body-block 接触,另一侧
                // 是哪个连杆 collider。
                if !is_ground
                    && tick >= spawn_tick
                    && first_hit_link.is_none()
                    && matches!(cfg.scene, Scene::M1Leg5 | Scene::M1Leg10)
                {
                    let hit = if is_robot(c1) { c1 } else { c2 };
                    first_hit_link = Some(if hit == robot.colliders[0] {
                        "body"
                    } else {
                        // colliders 顺序:body,每腿 [lat, thigh, shank, foot]
                        let idx = robot.colliders.iter().position(|c| *c == hit).unwrap_or(0);
                        match idx % 4 {
                            1 => "hip",
                            2 => "thigh",
                            3 => "shank",
                            _ => "foot",
                        }
                    });
                }
                let in_static = tick >= total_ticks.saturating_sub(static_window);
                for manifold in &rigid.manifolds {
                    let deepest = manifold
                        .points
                        .iter()
                        .map(|pt| pt.dist)
                        .reduce(f32::max)
                        .unwrap_or(0.0);
                    if is_ground {
                        max_pen_bg = max_pen_bg.min(deepest);
                        if in_static {
                            static_pen_bg = static_pen_bg.min(deepest);
                        }
                    } else {
                        max_pen_bb = max_pen_bb.min(deepest);
                        if in_static {
                            static_pen_bb = static_pen_bb.min(deepest);
                        }
                        // 动态穿透 ≤ v·dt?以撞击拍刚体速度估一拍行程。
                        if deepest < 0.0 && dyn_pen_within_step.is_none() {
                            // 用当前主体连杆最大速度 × dt 比较。
                            dyn_pen_within_step =
                                Some(-deepest <= tick_link_max * world.integration_parameters.dt);
                        }
                    }
                }
            }
        }

        // M5 分阶段统计(静置段 = spawn 后 3600 拍,窗口上界修正 R1-5)。
        if matches!(cfg.scene, Scene::M5 | Scene::M5Awake) && tick >= m5_settle_start {
            if tick == m5_settle_start {
                m5_h_start = Some(p.y);
            }
            if tick == m5_settle_end {
                m5_h_end = p.y;
            }
            if tick < m5_settle_end && awake {
                let lv = body.linvel();
                m5_settle_slide_sum += f64::from((lv.x * lv.x + lv.z * lv.z).sqrt());
                m5_settle_slide_n += 1;
            }
        }

        // 力矩饱和(静置段之后全部拍;口径同 gait)。
        if tick >= STAND_TICKS && matches!(cfg.scene, Scene::M3One | Scene::M3Four | Scene::M4)
            || tick >= STAND_TICKS && matches!(cfg.scene, Scene::M1Body5 | Scene::M1Body10)
            || tick >= STAND_TICKS
        {
            for (dof, tgt) in targets.iter().enumerate() {
                let q = read_angle(&world.multibody_joints, &robot, dof);
                let e = tgt - q;
                let qd = {
                    let (mb_index, link_id) = robot.joints[dof];
                    let mb = world.multibody_joints.get_multibody(mb_index).expect("a");
                    mb.joint_velocity(mb.link(link_id).expect("a"))[0]
                };
                let tau_cmd = (kp * e + kd * (0.0 - qd)).abs();
                sat_total += 1;
                if tau_cmd >= tau {
                    sat_n += 1;
                }
            }
        }

        // 判据 3 监视:静置穿透 > 5 cm 首拍记异常(继续跑,报数)。
        if anomaly.is_none() && (static_pen_bb < -0.05 || static_pen_bg < -0.05) {
            anomaly = Some(tick);
        }
    }

    let end_body = if robot_alive {
        world.bodies.get(robot.body).expect("alive")
    } else {
        // guard 移除后:主体已不在世界(报 end y = NaN 语义不行,报移除拍)。
        // 用 0 向量占位,guard_removed 列说明一切。
        return Outcome {
            name: cfg.name.clone(),
            nan,
            panicked: false,
            panic_msg: String::new(),
            guard_removed,
            max_link_speed,
            link_over_ticks,
            link_v_limit: v_limit,
            max_body_speed,
            max_block_speed,
            max_block_upthrow: max_upthrow,
            post_impact_body_max_y,
            max_pen_body_block: max_pen_bb,
            max_pen_body_ground: max_pen_bg,
            static_pen_bb,
            static_pen_bg,
            dyn_pen_within_step,
            torque_sat_ratio: if sat_total > 0 {
                sat_n as f32 / sat_total as f32
            } else {
                0.0
            },
            end_body_y: f32::NAN,
            end_up_y: f32::NAN,
            block_disp_max,
            block_upthrow_after_settle: max_upthrow,
            block_rebound_speed,
            anomaly_tick: anomaly,
            m5_height_change: 0.0,
            m5_sleep_ratio: m5_sleep_ticks as f32 / 3600.0,
            m5awake_slide: (m5_settle_slide_sum / m5_settle_slide_n.max(1) as f64) as f32,
            first_hit_link: first_hit_link.unwrap_or("-").to_string(),
            pushed_dist,
            state_hash: hash,
        };
    };
    let end_up = end_body.rotation() * Vector::new(0.0, 1.0, 0.0);
    let settle_y_mean = (block_settle_y_sum / block_settle_y_n.max(1) as f64) as f32;
    let end_p = end_body.translation();
    pushed_dist =
        ((end_p.x - leg_spawn_pos.x).powi(2) + (end_p.z - leg_spawn_pos.z).powi(2)).sqrt();

    Outcome {
        name: cfg.name.clone(),
        nan,
        panicked: false,
        panic_msg: String::new(),
        guard_removed,
        max_link_speed,
        link_over_ticks,
        link_v_limit: v_limit,
        max_body_speed,
        max_block_speed,
        max_block_upthrow: max_upthrow,
        post_impact_body_max_y,
        max_pen_body_block: max_pen_bb,
        max_pen_body_ground: max_pen_bg,
        static_pen_bb,
        static_pen_bg,
        dyn_pen_within_step,
        torque_sat_ratio: if sat_total > 0 {
            sat_n as f32 / sat_total as f32
        } else {
            0.0
        },
        end_body_y: end_body.translation().y,
        end_up_y: end_up.y,
        block_disp_max,
        block_upthrow_after_settle: max_upthrow - (settle_y_mean - block_spawn_y),
        block_rebound_speed,
        anomaly_tick: anomaly,
        m5_height_change: m5_h_start.map_or(0.0, |h0| m5_h_end - h0),
        m5_sleep_ratio: m5_sleep_ticks as f32 / 3600.0,
        m5awake_slide: (m5_settle_slide_sum / m5_settle_slide_n.max(1) as f64) as f32,
        first_hit_link: first_hit_link.unwrap_or("-").to_string(),
        pushed_dist,
        state_hash: hash,
    }
}

/// 从组名解析 M1 网格偏移(`M1Body5_ox-0.15_oz0.3_comp` → (−0.15, 0.3))。
fn m1_offsets(name: &str) -> (f32, f32) {
    let mut ox = 0.0f32;
    let mut oz = 0.0f32;
    for part in name.split('_') {
        if let Some(v) = part.strip_prefix("ox") {
            ox = v.parse().unwrap_or(0.0);
        }
        if let Some(v) = part.strip_prefix("oz") {
            oz = v.parse().unwrap_or(0.0);
        }
    }
    (ox, oz)
}

fn outcome_csv(o: &Outcome) -> String {
    format!(
        "{},{},{},{},{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{:.4},{:.4},{},{},{:.4},{:.4},{:.4},{},{:.4},{:016x}",
        o.name,
        o.nan,
        o.panicked,
        o.guard_removed,
        o.panic_msg,
        o.max_link_speed,
        o.link_over_ticks,
        o.link_v_limit,
        o.max_body_speed,
        o.max_block_speed,
        o.max_block_upthrow,
        o.post_impact_body_max_y,
        o.max_pen_body_block,
        o.max_pen_body_ground,
        o.static_pen_bb,
        o.static_pen_bg,
        o.torque_sat_ratio,
        o.end_body_y,
        o.end_up_y,
        o.block_disp_max,
        o.block_upthrow_after_settle,
        o.block_rebound_speed,
        o.dyn_pen_within_step
            .map_or_else(|| "-".to_string(), |b| b.to_string()),
        o.anomaly_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.m5_height_change,
        o.m5_sleep_ratio,
        o.m5awake_slide,
        o.first_hit_link,
        o.pushed_dist,
        o.state_hash,
    )
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("--list") {
        let mut all = build_matrix(None);
        all.extend(build_guard_matrix());
        for c in all {
            println!("{}", c.name);
        }
        return;
    }
    let det_mode = args.iter().any(|a| a == "det");
    let only: Vec<&String> = args.iter().filter(|a| a.as_str() != "det").collect();

    // guard 矩阵(R1-7):V_GUARD 1000/200,M7 全部 + real/1-500 的 M1 网格。
    // 默认(无参数)运行 = 常规矩阵 + guard 矩阵(提交数据口径)。
    let guard_matrix = build_guard_matrix();
    let all = if let Some(g) = args.iter().find_map(|a| a.strip_prefix("guard")) {
        build_matrix(Some(g))
    } else if only.is_empty() {
        let mut all = build_matrix(None);
        all.extend(guard_matrix);
        all
    } else {
        // 选择模式:按子串过滤全矩阵 + guard 矩阵。
        let mut all = build_matrix(None);
        all.extend(guard_matrix);
        all.into_iter()
            .filter(|c| only.iter().any(|a| c.name.contains(a.as_str())))
            .collect()
    };

    let mut out = String::new();
    out.push_str(
        "name,nan,panicked,guard_removed,panic_msg,max_link_speed,link_over_ticks,\
         link_v_limit,max_body_speed,max_block_speed,max_block_upthrow,\
         post_impact_body_max_y,max_pen_body_block,max_pen_body_ground,\
         static_pen_bb,static_pen_bg,torque_sat_ratio,end_body_y,end_up_y,\
         block_disp_max,block_upthrow_after_settle,block_rebound_speed,\
         dyn_pen_le_step,anomaly_tick,m5_sleep_ratio,m5awake_slide,state_hash\n",
    );
    for cfg in &all {
        let selected = if det_mode {
            cfg.name.contains("comp")
                && (cfg.name.contains("M1Body5_ox-0.15_oz-0.15") || cfg.name.contains("M5_"))
        } else {
            true
        };
        if !selected {
            continue;
        }
        #[allow(clippy::disallowed_methods)]
        let t0 = Instant::now();
        let res = run_once(cfg);
        let line = match res {
            Ok(o) => outcome_csv(&o),
            Err(msg) => {
                // panic 隔离:单次 panic 记一行,矩阵继续(R1-4)。
                format!(
                    "{},false,true,0,{},0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-,-,0,0,PANIC",
                    cfg.name,
                    msg.replace(',', " ")
                )
            }
        };
        println!("{} | {:.1}s", line, t0.elapsed().as_secs_f32());
        out.push_str(&line);
        out.push('\n');
        if det_mode {
            if let (Ok(o1), Ok(o2)) = (run_once(cfg), run_once(cfg)) {
                println!(
                    "determinism: {:016x} vs {:016x} -> {}",
                    o1.state_hash,
                    o2.state_hash,
                    if o1.state_hash == o2.state_hash {
                        "IDENTICAL"
                    } else {
                        "DIFFER"
                    }
                );
            } else {
                println!("determinism: PANIC on re-run");
            }
        }
    }
    let _ = std::fs::create_dir_all("out");
    let _ = std::fs::write("out/run_summary.csv", &out);
    let _ = guard_matrix;
}

/// M1Body 网格矩阵(R1-2/3):7 档比例 × 25 落点 × 2 高度。
fn build_matrix(guard: Option<&str>) -> Vec<Cfg> {
    let mut all = Vec::new();
    let v_guard = guard.map(|g| g.parse::<f32>().unwrap());
    let scales = [
        1.0 / 1500.0f32,
        1.0 / 500.0,
        1.0 / 200.0,
        1.0 / 100.0,
        1.0 / 50.0,
        1.0 / 20.0,
        1.0,
    ];
    if let Some(vg) = v_guard {
        // guard 模式:M7 全部 + real / 1-500 的 M1 网格;名字带 _vg{n} 后缀。
        for s in [1.0 / 1500.0, 1.0 / 500.0] {
            for h5 in [5.0f32, 10.0] {
                for ox in M1_GRID {
                    for oz in M1_GRID {
                        let scene = if h5 == 5.0 {
                            Scene::M1Body5
                        } else {
                            Scene::M1Body10
                        };
                        all.push(Cfg {
                            name: format!(
                                "{}_ox{ox}_oz{oz}_{}_vg{}",
                                scene.name(),
                                Scale { s }.tag(),
                                vg as i32
                            ),
                            scene,
                            scale: Scale { s },
                            v_guard: Some(vg),
                        });
                    }
                }
            }
        }
        for h in [1.0f32, 2.0, 4.0] {
            for ox in [-0.3f32, 0.0, 0.3] {
                let scene = Scene::M7 { h, ox, oz: 0.0 };
                for s in [1.0 / 100.0, 1.0 / 20.0, 1.0 / 5.0, 1.0] {
                    all.push(Cfg {
                        name: format!("{}_{}_vg{}", scene.name(), Scale { s }.tag(), vg as i32),
                        scene,
                        scale: Scale { s },
                        v_guard: Some(vg),
                    });
                }
            }
        }
        return all;
    }
    // 常规矩阵:
    // - M1Body 网格 × 7 档(R1-3)
    // - M1Leg / M2 / M3 / M4 / M5 / M6 × §4 三档
    // - M5Awake × 三档(R1-5)
    // - M7 × 4 档(R1-6)
    for s in scales {
        for h5 in [5.0f32, 10.0] {
            for ox in M1_GRID {
                for oz in M1_GRID {
                    let scene = if h5 == 5.0 {
                        Scene::M1Body5
                    } else {
                        Scene::M1Body10
                    };
                    all.push(Cfg {
                        name: format!("{}_ox{ox}_oz{oz}_{}", scene.name(), Scale { s }.tag()),
                        scene,
                        scale: Scale { s },
                        v_guard: None,
                    });
                }
            }
        }
    }
    let three = [
        Scale { s: 1.0 / 1500.0 },
        Scale { s: 1.0 / 100.0 },
        Scale { s: 1.0 },
    ];
    for scale in three {
        let tag = scale.tag();
        for scene in [
            Scene::M1Leg5,
            Scene::M1Leg10,
            Scene::M2,
            Scene::M3One,
            Scene::M3Four,
            Scene::M4,
            Scene::M5,
            Scene::M5Awake,
            Scene::M6,
        ] {
            all.push(Cfg {
                name: format!("{}_{}", scene.name(), tag),
                scene,
                scale,
                v_guard: None,
            });
        }
    }
    for h in [1.0f32, 2.0, 4.0] {
        for ox in [-0.3f32, 0.0, 0.3] {
            let scene = Scene::M7 { h, ox, oz: 0.0 };
            for s in [1.0 / 100.0f32, 1.0 / 20.0, 1.0 / 5.0, 1.0] {
                all.push(Cfg {
                    name: format!("{}_{}", scene.name(), Scale { s }.tag()),
                    scene,
                    scale: Scale { s },
                    v_guard: None,
                });
            }
        }
    }
    all
}

/// guard 矩阵(R1-7):V_GUARD 1000/200 × (M7 全部 + real/1-500 M1 网格)。
fn build_guard_matrix() -> Vec<Cfg> {
    let mut all = Vec::new();
    for vg in V_GUARDS {
        all.extend(build_matrix(Some(&format!("{}", vg as i32))));
    }
    all
}
