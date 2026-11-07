//! REQ-012 A00-2 demo `mass`(2026-11-06):定系数 k。
//!
//! 问题(放行原文):方块质量固定为 1(不能改),主体质量比方块小很多时,
//! 方块砸在身上、压在身上或主体站在方块堆上,求解器会不会穿透或炸开?
//!
//! 方块与生产一致:sim.rs ~:4266 的单格实体路径——`cuboid(0.5, 0.5, 0.5)`,
//! density 1、friction 0.6、restitution 0.1(质量恰 1)。
//!
//! 主体按 §4 三档缩放,质量、kp、kd、力矩上限、默认阻尼都乘同一系数 s:
//! - 真实:  s = 1/1500(主体总质量 0.01 kg,1 格方块 ≈ 1500 kg)
//! - 压缩:  s = 1/100 (主体总质量 0.15 kg,1 格方块 ≈ 100 kg,D9 推荐)
//! - 不换算: s = 1     (主体总质量 15 kg)
//!
//! 场景(放行表):
//! - M1 砸中:站稳后 1 个方块从上方 5 m / 10 m 自由落下,分别砸机身与
//!   砸腿(偏置落点),各 2 × 3 档;
//! - M2 侧撞:方块以 10 m/s 水平撞机身;
//! - M3 压住:1 个、4 个(2×2)方块轻放背上,静置 600 拍;
//! - M4 脱困:M3 的 1 个方块 600 拍后移除,看主体能否回到站高(再 600 拍);
//! - M5 站方块堆:地面 3×3 × 1 层 9 个动态方块,主体站上去静置 3600 拍
//!   再走 1200 拍;
//! - M6 顶方块:主体沿直线走向地面上 1 个静止方块,走 1200 拍。
//!
//! 判据与报告口径见 README。所有结果写入 out/(默认 run_summary.csv,
//! 提交数据单独文件,默认运行不覆盖)。

use std::time::Instant;

use quad::{
    default_gait, ik, production_params, read_angle, set_target, spawn_robot_yawed, DampMode,
    GROUND_RESTITUTION, N_JOINTS,
};
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::{ColliderBuilder, PairContacts};
use rapier3d::math::{Pose, Vector};
use rapier3d::pipeline::PhysicsWorld;

/// 地面顶面(平地口径,同 gait:cuboid 顶面 = 0)。
const GROUND_TOP: f32 = 0.0;
/// 静置时机身原点高度:足端触地(2L + 足半径)− 站高(足端离髋 0.28)。
/// 机身原点 = 髋锚平面(机身中心)。足端 y = 0 → 髋 y = 0.28 → 机身原点 y
/// = 0.28(髋锚在机身 local y=0 平面上,见 quad::spawn_robot)。
const STAND_BODY_Y: f32 = quad::FOOT_HEIGHT;
/// 方块尺寸与材质(生产:sim.rs ~:4266)。
const BLOCK_HALF: f32 = 0.5;

/// 质量档。
#[derive(Clone, Copy, PartialEq)]
enum Scale {
    /// s = 1/1500,主体 0.01 kg。
    Real,
    /// s = 1/100,主体 0.15 kg(压缩,D9 推荐)。
    Compressed,
    /// s = 1,主体 15 kg。
    None,
}

impl Scale {
    fn s(self) -> f32 {
        match self {
            Scale::Real => 1.0 / 1500.0,
            Scale::Compressed => 1.0 / 100.0,
            Scale::None => 1.0,
        }
    }
    fn tag(self) -> &'static str {
        match self {
            Scale::Real => "real",
            Scale::Compressed => "comp",
            Scale::None => "none",
        }
    }
}

/// 场景。
#[derive(Clone, Copy, PartialEq)]
enum Scene {
    /// 砸机身,5 m。
    M1Body5,
    /// 砸机身,10 m。
    M1Body10,
    /// 砸腿(偏置落点),5 m。
    M1Leg5,
    /// 砸腿,10 m。
    M1Leg10,
    /// 侧撞 10 m/s。
    M2,
    /// 1 个方块压背,静置 600 拍。
    M3One,
    /// 4 个方块(2×2)压背,静置 600 拍。
    M3Four,
    /// M3One + 600 拍后移除方块,再看 600 拍。
    M4,
    /// 3×3 方块堆站上去,静置 3600 拍 + 走 1200 拍。
    M5,
    /// 走向 1 个静止方块,走 1200 拍。
    M6,
}

impl Scene {
    fn total_ticks(self) -> usize {
        match self {
            Scene::M1Body5 | Scene::M1Body10 | Scene::M1Leg5 | Scene::M1Leg10 | Scene::M2 => 900,
            Scene::M3One | Scene::M3Four => 180 + 600,
            Scene::M4 => 180 + 600 + 600,
            Scene::M5 => 180 + 3600 + 1200,
            Scene::M6 => 180 + 1200,
        }
    }
}

struct Cfg {
    name: &'static str,
    scene: Scene,
    scale: Scale,
}

struct Outcome {
    name: String,
    nan: bool,
    max_link_speed: f32,
    max_block_speed: f32,
    max_block_upthrow: f32,
    max_pen_body_block: f32,
    max_pen_body_ground: f32,
    torque_sat_ratio: f32,
    end_body_y: f32,
    end_up_y: f32,
    block_disp_max: f32,
    /// 发生异常的拍(NaN / 穿透超 5 cm 首拍;无 = -)。
    anomaly_tick: Option<usize>,
    /// M5 判据 4:静置段末 − 静置段首 的机身高度(其他场景 0)。
    m5_height_change: f32,
    m5_settle_pen_bb: f32,
    m5_settle_pen_bg: f32,
    m5_settle_max_link_v: f32,
    m5_settle_block_disp: f32,
    /// 最后 120 拍(静置窗口)的 body-block / body-ground 最小接触距离。
    static_pen_bb: f32,
    static_pen_bg: f32,
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

/// 与生产一致的单格方块(density 1、friction 0.6、restitution 0.1,
/// 质量恰 1 kg)。睡眠不关(放行:真实唤醒和落定)。
fn spawn_block(
    world: &mut PhysicsWorld,
    pos: Vector,
    vel: Vector,
) -> rapier3d::dynamics::RigidBodyHandle {
    world
        .insert(
            RigidBodyBuilder::dynamic()
                .pose(Pose {
                    translation: pos,
                    ..Pose::IDENTITY
                })
                .linvel(vel),
            ColliderBuilder::cuboid(BLOCK_HALF, BLOCK_HALF, BLOCK_HALF)
                .density(1.0)
                .friction(0.6)
                .restitution(0.1),
        )
        .0
}

fn run(cfg: &Cfg) -> Outcome {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = production_params(1.0 / 60.0);
    let _ = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, GROUND_TOP - 0.5, 0.0)),
        ColliderBuilder::cuboid(200.0, 0.5, 200.0)
            .friction(quad::GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
    );

    let s = cfg.scale.s();
    let gait = default_gait();
    // 出生高度:M5 出生在 3×3 方块堆顶(堆顶 y = 1.0)上方,其余在地面。
    // 出生在堆内会造成初始穿插,−0.5 m 穿透与 20+ m/s 连杆速度都来自这个
    // 布景错误,不是求解器行为。
    let pile_top = if cfg.scene == Scene::M5 { 1.0f32 } else { 0.0 };
    let y0 = GROUND_TOP + pile_top + 2.0 * quad::L + quad::FOOT_R + 0.002;
    let spawn = Vector::new(0.0, y0, 0.0);
    let robot = spawn_robot_yawed(
        &mut world,
        spawn,
        quad::yaw_quat(0.0),
        s,
        quad::GROUND_FRICTION,
        DampMode::Scaled,
    );

    let tau = s * quad::TAU_LIMIT;
    let kp = s * quad::DEFAULT_KP;
    let kd = s * quad::DEFAULT_KD;
    let rise_ticks = 60usize;
    let stand0 = rise_ticks + 2 * 60; // 3 s 站立序列结束
                                      // M5:静置 3600 拍之后才走(放行);M6:站立序列结束即走。
    let walk0 = match cfg.scene {
        Scene::M5 => stand0 + 3600,
        _ => stand0,
    };

    // 事件方块(每场景 0..4 个;记录句柄供逐拍测量/移除)。
    let mut blocks: Vec<rapier3d::dynamics::RigidBodyHandle> = Vec::new();
    let body_top = STAND_BODY_Y + quad::BODY_HALF[1]; // 机身顶面
    match cfg.scene {
        Scene::M1Body5 | Scene::M1Body10 => {
            let h = if cfg.scene == Scene::M1Body10 {
                10.0
            } else {
                5.0
            };
            // 方块底面砸在机身顶面正上方:中心 = body_top + half + h。
            blocks.push(spawn_block(
                &mut world,
                Vector::new(0.0, body_top + BLOCK_HALF + h, 0.0),
                Vector::ZERO,
            ));
        }
        Scene::M1Leg5 | Scene::M1Leg10 => {
            let h = if cfg.scene == Scene::M1Leg10 {
                10.0
            } else {
                5.0
            };
            // 偏置落点:砸 FL 腿小腿中段(x = HIP_X ≈ 0.19,z = −THIGH_OFF)。
            blocks.push(spawn_block(
                &mut world,
                Vector::new(quad::HIP_X, body_top + BLOCK_HALF + h, -quad::THIGH_OFF),
                Vector::ZERO,
            ));
        }
        Scene::M2 => {
            // 10 m/s 水平撞机身侧面:出生在 −x 侧 3 m 处(留 ~0.1 s 飞行)。
            blocks.push(spawn_block(
                &mut world,
                Vector::new(-3.0, STAND_BODY_Y, 0.0),
                Vector::new(10.0, 0.0, 0.0),
            ));
        }
        Scene::M3One | Scene::M4 => {
            // 轻放:底面贴机身顶面(初速 0,自然落下 ~1 mm 内接触)。
            blocks.push(spawn_block(
                &mut world,
                Vector::new(0.0, body_top + BLOCK_HALF + 0.002, 0.0),
                Vector::ZERO,
            ));
        }
        Scene::M3Four => {
            // 2×2 平铺机身顶面(机身 0.38×0.094,2 格 1.0×1.0 覆盖)。
            for dx in [-0.5f32, 0.5] {
                for dz in [-0.5, 0.5] {
                    blocks.push(spawn_block(
                        &mut world,
                        Vector::new(dx, body_top + BLOCK_HALF + 0.002, dz),
                        Vector::ZERO,
                    ));
                }
            }
        }
        Scene::M5 => {
            // 地面 3×3 静止方块(中心在出生点),主体落上去。
            for dx in [-1.0f32, 0.0, 1.0] {
                for dz in [-1.0, 0.0, 1.0] {
                    blocks.push(spawn_block(
                        &mut world,
                        Vector::new(dx, GROUND_TOP + BLOCK_HALF, dz),
                        Vector::ZERO,
                    ));
                }
            }
        }
        Scene::M6 => {
            // 前方 2 m 地面上 1 个静止方块。
            blocks.push(spawn_block(
                &mut world,
                Vector::new(2.0, GROUND_TOP + BLOCK_HALF, 0.0),
                Vector::ZERO,
            ));
        }
    }
    let block_spawn_pos: Vec<Vector> = blocks
        .iter()
        .map(|h| world.bodies.get(*h).expect("alive").translation())
        .collect();

    let mut targets = [0.0f32; N_JOINTS];
    let rise_len = rise_ticks;

    let mut nan = false;
    let mut max_link_speed = 0.0f32;
    let mut max_block_speed = 0.0f32;
    let mut max_upthrow = 0.0f32;
    let mut max_pen_bb = 0.0f32; // body-block
    let mut max_pen_bg = 0.0f32; // body-ground
    let mut sat_n = 0u64;
    let mut sat_total = 0u64;
    let mut anomaly = None;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut block_disp_max = 0.0f32;
    // M5 判据 4:静置段首拍(end of settle ramp)与末拍的机身高度。
    let m5_settle_start = stand0;
    let m5_settle_end = stand0 + 3600;
    let mut m5_h_start: Option<f32> = None;
    let mut m5_h_end = 0.0f32;
    // M5 的静置段穿透/速度单独统计(与下落瞬间的动态值分开)。
    let mut m5_settle_pen_bb = 0.0f32;
    let mut m5_settle_pen_bg = 0.0f32;
    let mut m5_settle_max_link_v = 0.0f32;
    let mut m5_settle_block_disp = 0.0f32;
    // 静置穿透口径(判据 3):全部场景取最后 120 拍(2 s,撞击动态已平息)
    // 的最小接触距离;动态穿透(撞击瞬间)单列 max_pen_*,如实报。
    let static_window = 120usize;
    let mut static_pen_bb = 0.0f32;
    let mut static_pen_bg = 0.0f32;

    for tick in 0..cfg.scene.total_ticks() {
        // 关节目标:站立序列(rise smoothstep + settle),M5/M6 之后行走。
        if tick % 2 == 0 {
            for leg in 0..4 {
                let foot = if tick < rise_len {
                    let a = tick as f32 / rise_len as f32;
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

        // M4:walk0 + 600 拍时移除背上方块。
        if cfg.scene == Scene::M4 && tick == walk0 + 600 {
            for h in blocks.drain(..) {
                world.remove_body(h);
            }
        }

        world.step();

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

        // 机身速度、连杆速度、方块速度/上抛。
        max_link_speed = max_link_speed.max(body.linvel().length());
        for sh in &robot.shanks {
            let rb = world.bodies.get(*sh).expect("alive");
            max_link_speed = max_link_speed.max(rb.linvel().length());
        }
        let mut cur_block_disp = 0.0f32;
        for (i, h) in blocks.iter().enumerate() {
            let rb = world.bodies.get(*h).expect("alive");
            max_block_speed = max_block_speed.max(rb.linvel().length());
            max_upthrow = max_upthrow.max(rb.translation().y - block_spawn_pos[i].y);
            let d = rb.translation() - block_spawn_pos[i];
            cur_block_disp = cur_block_disp.max((d.x * d.x + d.y * d.y + d.z * d.z).sqrt());
        }
        block_disp_max = block_disp_max.max(cur_block_disp);

        // 穿透:主体侧(连杆+机身)对地、对方块,按接触对归属分桶。
        for pair in world.narrow_phase.contact_graph().interactions() {
            if let PairContacts::Rigid(rigid) = &pair.contacts {
                let c1 = pair.collider1;
                let c2 = pair.collider2;
                let is_robot = |c| robot.colliders.contains(&c);
                let (rb_h, other_is_block) = if is_robot(c1) || is_robot(c2) {
                    // 主体参与的对:另一侧是不是方块?
                    let other = if is_robot(c1) { c2 } else { c1 };
                    // 方块 collider 句柄集合:每方块 1 collider,取 bodies 查父
                    // 不方便;直接用"另一侧刚体的质量"判?更稳:记录方块
                    // collider 句柄。spawn_block 返回 (body, collider)——这里
                    // 简化:对固定地面(fixed)判地,其余算方块。
                    let other_rb = world
                        .colliders
                        .get(other)
                        .and_then(|co| co.parent())
                        .and_then(|ph| world.bodies.get(ph));
                    let is_ground = other_rb.map(|rb| rb.is_fixed()).unwrap_or(true);
                    (true, !is_ground)
                } else {
                    (false, false)
                };
                if !rb_h {
                    continue;
                }
                for manifold in &rigid.manifolds {
                    for pt in &manifold.points {
                        let in_static =
                            tick >= cfg.scene.total_ticks().saturating_sub(static_window);
                        if other_is_block {
                            max_pen_bb = max_pen_bb.min(pt.dist);
                            if in_static {
                                static_pen_bb = static_pen_bb.min(pt.dist);
                            }
                            if cfg.scene == Scene::M5 && tick >= m5_settle_start {
                                m5_settle_pen_bb = m5_settle_pen_bb.min(pt.dist);
                            }
                        } else {
                            max_pen_bg = max_pen_bg.min(pt.dist);
                            if in_static {
                                static_pen_bg = static_pen_bg.min(pt.dist);
                            }
                            if cfg.scene == Scene::M5 && tick >= m5_settle_start {
                                m5_settle_pen_bg = m5_settle_pen_bg.min(pt.dist);
                            }
                        }
                    }
                }
            }
        }

        // M5 分阶段统计:静置段(3 s 序列后)的连杆速度、方块位移、机身高度。
        if cfg.scene == Scene::M5 && tick >= m5_settle_start {
            m5_settle_max_link_v = m5_settle_max_link_v.max(max_link_speed);
            m5_settle_block_disp = m5_settle_block_disp.max(block_disp_max);
            if tick == m5_settle_start {
                m5_h_start = Some(p.y);
            }
            if tick == m5_settle_end {
                m5_h_end = p.y;
            }
        }

        // 力矩饱和(站立段之后;口径同 gait)。
        if tick >= walk0 {
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
        if anomaly.is_none() && (max_pen_bb < -0.05 || max_pen_bg < -0.05) {
            anomaly = Some(tick);
        }
    }

    let end_body = world.bodies.get(robot.body).expect("alive");
    let end_up = end_body.rotation() * Vector::new(0.0, 1.0, 0.0);

    Outcome {
        name: cfg.name.to_string(),
        nan,
        max_link_speed,
        max_block_speed,
        max_block_upthrow: max_upthrow,
        max_pen_body_block: max_pen_bb,
        max_pen_body_ground: max_pen_bg,
        torque_sat_ratio: if sat_total > 0 {
            sat_n as f32 / sat_total as f32
        } else {
            0.0
        },
        end_body_y: end_body.translation().y,
        end_up_y: end_up.y,
        block_disp_max,
        anomaly_tick: anomaly,
        m5_height_change: m5_h_start.map_or(0.0, |h0| m5_h_end - h0),
        m5_settle_pen_bb,
        m5_settle_pen_bg,
        m5_settle_max_link_v,
        m5_settle_block_disp,
        static_pen_bb,
        static_pen_bg,
        state_hash: hash,
    }
}

fn outcome_csv(o: &Outcome) -> String {
    format!(
        "{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:016x}",
        o.name,
        o.nan,
        o.max_link_speed,
        o.max_block_speed,
        o.max_block_upthrow,
        o.max_pen_body_block,
        o.max_pen_body_ground,
        o.torque_sat_ratio,
        o.end_body_y,
        o.end_up_y,
        o.block_disp_max,
        o.anomaly_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        o.m5_height_change,
        o.m5_settle_pen_bb,
        o.m5_settle_pen_bg,
        o.m5_settle_max_link_v,
        o.m5_settle_block_disp,
        o.static_pen_bb,
        o.static_pen_bg,
        o.state_hash,
    )
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("--list") {
        for c in build_matrix() {
            println!("{}", c.name);
        }
        return;
    }
    let det_mode = args.first().map(String::as_str) == Some("det");
    let only: Vec<&String> = if det_mode || args.is_empty() {
        Vec::new()
    } else {
        args.iter().collect()
    };

    let all = build_matrix();
    let mut out = String::new();
    out.push_str(
        "name,nan,max_link_speed,max_block_speed,max_block_upthrow,\
         max_pen_body_block,max_pen_body_ground,torque_sat_ratio,end_body_y,\
         end_up_y,block_disp_max,anomaly_tick,m5_height_change,\
         m5_settle_pen_bb,m5_settle_pen_bg,m5_settle_max_link_v,\
         m5_settle_block_disp,static_pen_bb,static_pen_bg,state_hash\n",
    );
    for cfg in &all {
        let selected = if det_mode {
            cfg.name.contains("comp") && (cfg.name.contains("M1Body5") || cfg.name.contains("M5"))
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
        let o = run(cfg);
        println!("{} | {:.1}s", outcome_csv(&o), t0.elapsed().as_secs_f32());
        out.push_str(&outcome_csv(&o));
        out.push('\n');
        if det_mode {
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

fn build_matrix() -> Vec<Cfg> {
    let mut all = Vec::new();
    for scale in [Scale::Real, Scale::Compressed, Scale::None] {
        let tag = scale.tag();
        for (scene, tag_scene) in [
            (Scene::M1Body5, "M1Body5"),
            (Scene::M1Body10, "M1Body10"),
            (Scene::M1Leg5, "M1Leg5"),
            (Scene::M1Leg10, "M1Leg10"),
            (Scene::M2, "M2"),
            (Scene::M3One, "M3One"),
            (Scene::M3Four, "M3Four"),
            (Scene::M4, "M4"),
            (Scene::M5, "M5"),
            (Scene::M6, "M6"),
        ] {
            let name = Box::leak(format!("{tag_scene}_{tag}").into_boxed_str());
            all.push(Cfg { name, scene, scale });
        }
    }
    all
}
