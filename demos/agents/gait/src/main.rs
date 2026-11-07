//! REQ-012 A00-1 demo `gait`(2026-11-06,R1 重做;A00-2 抽公共代码到 quad)。
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
//!   sim.rs:4205-4206)。脚的摩擦也是 0.6,不加垫(R1-4)。
//!
//! 运行时目标的写法、三个 rapier 坑、几何与步态的完整说明见 `quad` crate
//! 文档与 README(R1-8)。本文件的数值路径与 9a7405a 提交版逐行一致
//! (A00-2 放行:抽取不改行为,全矩阵状态哈希必须逐行相同;相位滞后列
//! 按验收意见 1 改为 ±半周期取峰的有符号值)。
//!
//! 用法(见 README):
//! `cargo run --release -- --list` 列参数组;
//! `cargo run --release -- <name>...` 跑指定组(不带参数跑全部单只组);
//! `cargo run --release -- eight` 8 只耗时组;
//! `cargo run --release -- det <name>` 确定性双跑比对。
//! 结果写 `out/run_summary.csv`(gitignore;提交的数据是
//! `out/results_submitted.csv`,默认运行不覆盖)。

use std::time::Instant;

use quad::{ik, joint_velocity, production_params, read_angle, set_target, spawn_robot};
use quad::{
    DampMode, Gait, Robot, FOOT_R, FULL_STRETCH_DY, GROUND_FRICTION, GROUND_RESTITUTION, N_JOINTS,
    TAU_LIMIT,
};
use rapier3d::math::Vector;
use rapier3d::pipeline::PhysicsWorld;

// ── 参数组 ──────────────────────────────────────────────────────────────

/// 阻尼口径之外的额外变体开关。
struct RunCfg {
    name: &'static str,
    dt: f32,
    num_solver_iterations: usize,
    kp: f32,
    kd: f32,
    /// 目标更新周期(拍)。
    update_every: usize,
    mass_scale: f32,
    /// 脚摩擦(变体:1.4)。
    foot_mu: f32,
    /// 阻尼口径(R1-5,默认 Scaled)。
    damp: DampMode,
    gait: Gait,
    /// 站立段秒数不重要(由 dt 换算:1 s 降 + 2 s 稳);行走段拍数按
    /// 60 Hz 口径(240 Hz 参照组换算成同秒数)。
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
    /// 沿机身航向的平均速度(|v| 投影到航向,时间均值)。
    fwd_speed_mean: f32,
    /// 世界系 x 平均速度(对照口径)。
    vx_world_mean: f32,
    /// 净位移 / 行走时长。
    net_speed: f32,
    lateral_drift: f32,
    heading_drift_deg: f32,
    /// 主体与地面接触的最深穿透(自接触已关,R1-2)。
    min_penetration: f32,
    /// 行走段脚最高离地(世界 m,含小腿倾斜的真实足端,R1-6)。
    foot_clearance: f32,
    nan: bool,
    max_link_speed: f32,
    /// 力矩饱和占比:kp·Δq + kd·Δq̇ 达到力矩上限的拍数比例(R1-6)。
    torque_sat_ratio: f32,
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

/// 跑一组。返回结果与逐拍 CSV(代表组用)。
#[allow(clippy::disallowed_methods)] // demo 自身要测每拍耗时;不在 workspace lints 下,allow 是纪律自束
fn run(cfg: &RunCfg, keep_trace: bool) -> (RunResult, String) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = production_params(cfg.dt);
    world.integration_parameters.num_solver_iterations = cfg.num_solver_iterations;
    let _ = world.insert(
        rapier3d::dynamics::RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        rapier3d::geometry::ColliderBuilder::cuboid(200.0, 0.5, 200.0)
            .friction(GROUND_FRICTION)
            .restitution(GROUND_RESTITUTION),
    );

    let n_bots = if cfg.eight { 8 } else { 1 };
    let hz = (1.0 / cfg.dt).round();
    let rise_ticks = hz as usize; // 1 s 从直腿降到站姿
    let settle_ticks = 2 * hz as usize; // 2 s 站稳
    let walk0 = rise_ticks + settle_ticks;
    let total_ticks = walk0 + cfg.walk_ticks;
    // 直腿初始悬空高度:足端刚触地(2L + 足球半径 + 2 mm)。
    let y0 = 2.0 * quad::L + FOOT_R + 0.002;
    let mut robots: Vec<Robot> = Vec::new();
    for i in 0..n_bots {
        let off = Vector::new((i % 4) as f32 * 2.0, 0.0, (i / 4) as f32 * 2.0);
        robots.push(spawn_robot(
            &mut world,
            Vector::new(0.0, y0, 0.0) + off,
            cfg.mass_scale,
            cfg.foot_mu,
            cfg.damp,
        ));
    }

    let tau = cfg.mass_scale * TAU_LIMIT;
    let kp = cfg.mass_scale * cfg.kp;
    let kd = cfg.mass_scale * cfg.kd;

    let mut up_min = 1.0f32;
    let mut height_sum = 0.0f64;
    let mut height_sq_sum = 0.0f64;
    let mut height_n = 0u64;
    let mut err = [0.0f64; 3]; // lat / pitch / knee
    let mut tgt_hist: Vec<f32> = Vec::new();
    let mut act_hist: Vec<f32> = Vec::new();
    let mut vfwd_sum = 0.0f64;
    let mut vx_sum = 0.0f64;
    let mut vel_n = 0u64;
    let mut sat_n = 0u64;
    let mut sat_total = 0u64;
    let mut start_pos: Option<Vector> = None;
    let mut start_fwd = Vector::X;
    let mut last_pos = Vector::ZERO;
    let mut last_fwd = Vector::X;
    let mut fell_tick = None;
    let mut step_ms: Vec<f32> = Vec::new();
    let mut max_link_speed = 0.0f32;
    let mut nan = false;
    let mut min_pen = 0.0f32;
    let mut foot_clear = 0.0f32;
    let mut trace_hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut trace_csv = String::new();
    if keep_trace {
        trace_csv.push_str("tick,y,up_y,vx,pen,clear\n");
    }
    // 上一拍写入的目标(力矩饱和 = kp·Δq + kd·Δq̇ 达上限的估算用)。
    let mut targets = [0.0f32; N_JOINTS];

    for tick in 0..total_ticks {
        let t0 = Instant::now();

        // 1) 更新关节目标(每 update_every 拍;R1-3:先 1 s 直腿降到
        //    足端 0.28 站高,再站稳,起步后步态足端轨迹)。
        if tick % cfg.update_every == 0 {
            for robot in &robots {
                for leg in 0..4 {
                    let foot = if tick < rise_ticks {
                        // 直腿 → 站姿:smoothstep 插值足端 y(x 保持 x0)。
                        let a = tick as f32 / rise_ticks as f32;
                        let sm = a * a * (3.0 - 2.0 * a);
                        let (x, _) = cfg.gait.stand_foot();
                        (x, FULL_STRETCH_DY + (-cfg.gait.h - FULL_STRETCH_DY) * sm)
                    } else if tick < walk0 {
                        cfg.gait.stand_foot()
                    } else {
                        let walk_t = (tick - walk0) as f32 * cfg.dt;
                        cfg.gait.foot_target(leg, walk_t)
                    };
                    let (dx, dy) = foot;
                    let (a, k) = ik(dx, dy);
                    targets[leg * 3] = 0.0;
                    targets[leg * 3 + 1] = a;
                    targets[leg * 3 + 2] = k;
                    for j in 0..3 {
                        set_target(
                            &mut world,
                            robot,
                            leg * 3 + j,
                            targets[leg * 3 + j],
                            kp,
                            kd,
                            tau,
                        );
                    }
                }
            }
        }

        // 2) 物理一步(60 Hz 不子步,生产求解参数)。
        world.step();

        // 3) 采样。
        let body = world.bodies.get(robots[0].body).expect("alive");
        let up = body.rotation() * Vector::new(0.0, 1.0, 0.0);
        let fwd = body.rotation() * Vector::new(1.0, 0.0, 0.0);
        if !body.translation().x.is_finite() || !body.translation().y.is_finite() {
            nan = true;
            break;
        }
        for v in [
            body.translation().x,
            body.translation().y,
            body.translation().z,
        ] {
            trace_hash = hash_f32(trace_hash, v);
        }
        let rot = body.rotation();
        trace_hash = hash_f32(trace_hash, rot.w);
        trace_hash = hash_f32(trace_hash, rot.x);
        trace_hash = hash_f32(trace_hash, rot.y);
        trace_hash = hash_f32(trace_hash, rot.z);
        for dof in 0..N_JOINTS {
            let q = read_angle(&world.multibody_joints, &robots[0], dof);
            trace_hash = hash_f32(trace_hash, q);
        }

        if tick >= walk0 {
            if start_pos.is_none() {
                start_pos = Some(body.translation());
                start_fwd = fwd;
            }
            if up.y < 0.7 && fell_tick.is_none() {
                fell_tick = Some(tick - walk0);
            }
            up_min = up_min.min(up.y);
            let py = f64::from(body.translation().y);
            height_sum += py;
            height_sq_sum += py * py;
            height_n += 1;
            let lv = body.linvel();
            vx_sum += f64::from(lv.x);
            let fh = Vector::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
            vfwd_sum += f64::from(lv.dot(fh));
            vel_n += 1;
            last_pos = body.translation();
            last_fwd = fwd;

            // 关节跟踪 RMS(按 lat/hip/knee 分开)+ 力矩饱和占比(R1-6)。
            for dof in 0..N_JOINTS {
                let q = read_angle(&world.multibody_joints, &robots[0], dof);
                let e = f64::from(targets[dof] - q);
                err[dof % 3] += e * e;
                // 上一控制拍目标的 PD 输出(ForceBased 即力矩)。
                let qd = joint_velocity(&world.multibody_joints, &robots[0], dof);
                let tau_cmd = (kp * (e as f32) + kd * (0.0 - qd)).abs();
                sat_total += 1;
                if tau_cmd >= tau {
                    sat_n += 1;
                }
            }
            if robots.len() == 1 {
                tgt_hist.push(targets[1]);
                act_hist.push(read_angle(&world.multibody_joints, &robots[0], 1));
            }

            // 真实足端世界坐标(含小腿倾斜)+ 最大连杆速度(R1-6)。
            for sh in &robots[0].shanks {
                let rb = world.bodies.get(*sh).expect("alive");
                let f = rb.translation() + rb.rotation() * Vector::new(0.0, -quad::L / 2.0, 0.0);
                foot_clear = foot_clear.max(f.y - FOOT_R);
                max_link_speed = max_link_speed.max(rb.linvel().length());
            }

            // 穿透:自接触已关,交互表里就是主体连杆与地面(R1-2)。
            for pair in world.narrow_phase.contact_graph().interactions() {
                if let rapier3d::geometry::PairContacts::Rigid(rigid) = &pair.contacts {
                    for manifold in &rigid.manifolds {
                        for pt in &manifold.points {
                            min_pen = min_pen.min(pt.dist);
                        }
                    }
                }
            }
        }

        if keep_trace && tick % 60 == 0 {
            trace_csv.push_str(&format!(
                "{},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
                tick,
                body.translation().y,
                up.y,
                body.linvel().x,
                min_pen,
                foot_clear
            ));
        }

        step_ms.push(t0.elapsed().as_secs_f32() * 1000.0);
    }

    let height_mean = (height_sum / (height_n.max(1) as f64)) as f32;
    let height_std = (height_sq_sum / (height_n.max(1) as f64)
        - f64::from(height_mean) * f64::from(height_mean))
    .max(0.0)
    .sqrt() as f32;
    // RMS 分母 = 采样拍数 × 4 腿(每拍每类关节 4 项)。
    let n_ticks = vel_n.max(1) as f64;
    let rms = [
        (err[0] / (n_ticks * 4.0)).sqrt() as f32,
        (err[1] / (n_ticks * 4.0)).sqrt() as f32,
        (err[2] / (n_ticks * 4.0)).sqrt() as f32,
    ];
    // 步态周期(拍):f Hz → 60/f 拍;验收意见 1:在 ±半周期内取峰,
    // 有符号(正 = 实际落后目标),不再在 0..n/2 上取全局最大。
    let period_ticks = ((1.0 / cfg.gait.freq) / cfg.dt).round().max(2.0) as i32;
    let lag_ticks = cross_corr_lag(&tgt_hist, &act_hist, period_ticks);
    let fwd_speed_mean = (vfwd_sum / (vel_n.max(1) as f64)) as f32;
    let vx_world_mean = (vx_sum / (vel_n.max(1) as f64)) as f32;
    let net_speed = start_pos.map_or(0.0, |s| {
        let d = last_pos - s;
        (((d.x * d.x + d.z * d.z) as f64).sqrt() / (cfg.walk_ticks as f64 * f64::from(cfg.dt)))
            as f32
    });
    let lateral_drift = start_pos.map_or(0.0, |s| last_pos.z - s.z);
    let heading_drift_deg =
        (start_fwd.z.atan2(start_fwd.x) - last_fwd.z.atan2(last_fwd.x)).to_degrees();

    let res = RunResult {
        name: cfg.name.to_string(),
        up_y_min: up_min,
        height_mean,
        height_std,
        rms_lat: rms[0],
        rms_pitch: rms[1],
        rms_knee: rms[2],
        lag_ticks,
        fwd_speed_mean,
        vx_world_mean,
        net_speed,
        lateral_drift,
        heading_drift_deg,
        min_penetration: min_pen,
        foot_clearance: foot_clear,
        nan,
        max_link_speed,
        torque_sat_ratio: sat_n as f32 / sat_total.max(1) as f32,
        step_ms_p50: percentile(&mut step_ms, 0.5),
        step_ms_p95: percentile(&mut step_ms, 0.95),
        step_ms_max: step_ms.iter().copied().fold(0.0, f32::max),
        fell_tick,
        trace_hash,
    };
    (res, trace_csv)
}

/// 互相关求相位滞后(拍,有符号:正 = 实际落后目标)。验收意见 1:只在
/// ±半周期内取峰——0..n/2 的全局最大会把"相差一个周期"当成 0 滞后
/// (周期折返)。数据不足返回 0。
fn cross_corr_lag(tgt: &[f32], act: &[f32], period_ticks: i32) -> f32 {
    let n = tgt.len().min(act.len());
    if n < 60 {
        return 0.0;
    }
    let half = (period_ticks / 2).max(1) as usize;
    let mut best_lag = 0i32;
    let mut best = f64::NEG_INFINITY;
    // 滞后 lag>0:实际 = 目标向右平移 lag(实际落后);lag<0:实际超前。
    for lag in -(half as i32)..=(half as i32) {
        let mut dot = 0.0f64;
        let mut cnt = 0u64;
        for (i, tv) in tgt.iter().enumerate().take(n) {
            let j = i as i64 - lag as i64;
            if j < 0 || j >= n as i64 {
                continue;
            }
            dot += f64::from(*tv) * f64::from(act[j as usize]);
            cnt += 1;
        }
        if cnt == 0 {
            continue;
        }
        let dot = dot / cnt as f64; // 归一化:窗口大小不同不可比
        if dot > best {
            best = dot;
            best_lag = lag;
        }
    }
    best_lag as f32
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
     lag_ticks,fwd_speed_mean,vx_world_mean,net_speed,lateral_drift,heading_drift_deg,\
     min_penetration,foot_clearance,nan,max_link_speed,torque_sat_ratio,\
     step_ms_p50,step_ms_p95,step_ms_max,fell_tick,trace_hash"
        .to_string()
}

#[allow(clippy::too_many_arguments)]
fn result_csv(r: &RunResult) -> String {
    let hash = format!("{:016x}", r.trace_hash);
    format!(
        "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.1},{:.4},{:.4},{:.4},{:.4},{:.2},{:.4},{:.4},{},{:.3},{:.3},{:.3},{:.3},{:.3},{},{}",
        csv_escape(&r.name),
        r.up_y_min,
        r.height_mean,
        r.height_std,
        r.rms_lat,
        r.rms_pitch,
        r.rms_knee,
        r.lag_ticks,
        r.fwd_speed_mean,
        r.vx_world_mean,
        r.net_speed,
        r.lateral_drift,
        r.heading_drift_deg,
        r.min_penetration,
        r.foot_clearance,
        r.nan,
        r.max_link_speed,
        r.torque_sat_ratio,
        r.step_ms_p50,
        r.step_ms_p95,
        r.step_ms_max,
        r.fell_tick
            .map_or_else(|| "-".to_string(), |t| t.to_string()),
        hash,
    )
}

/// 基线参数组(与总架构师探针 base 一致)。
fn base_cfg(name: &'static str) -> RunCfg {
    RunCfg {
        name,
        dt: 1.0 / 60.0,
        num_solver_iterations: 4,
        kp: 40.0,
        kd: 1.0,
        update_every: 2,
        mass_scale: 1.0,
        foot_mu: GROUND_FRICTION,
        gait: Gait {
            freq: 2.0,
            stride: 0.10,
            lift: 0.05,
            duty: 0.5,
            h: quad::FOOT_HEIGHT,
            x0: 0.0,
        },
        damp: DampMode::Scaled,
        walk_ticks: 3600,
        eight: false,
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

    let all = build_matrix();
    let only: Vec<&String> = if args.is_empty() {
        Vec::new()
    } else {
        args.iter().collect()
    };
    let det_mode = only.len() == 2 && only[0] == "det";
    let eight_mode = only.len() == 1 && only[0] == "eight";

    let mut out = String::new();
    out.push_str(&result_header());
    out.push('\n');
    for cfg in &all {
        let selected = if det_mode {
            cfg.name == only[1]
        } else if eight_mode {
            cfg.name.starts_with("eight_")
        } else if only.is_empty() {
            !cfg.eight // 默认跑全部单只组(含参照与对照组)
        } else {
            only.iter().any(|a| a.as_str() == cfg.name)
        };
        if !selected {
            continue;
        }
        let keep = cfg.name.starts_with("kp100_kd3_u2_f3.0_s0.15_l0.06_real")
            || cfg.name.starts_with("kp20_kd0.5_u2_f3.0_s0.15_l0.06_real")
            || cfg.name.starts_with("x001_dampunscaled");
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
    // R1-9:默认写 gitignore 的 run_summary.csv;提交的数据文件
    // results_submitted.csv 只在显式指定 --submit 时写出。
    let _ = std::fs::write("out/run_summary.csv", &out);
    if args.iter().any(|a| a == "--submit") {
        let _ = std::fs::write("out/results_submitted.csv", &out);
    }
}

/// 扫描矩阵(R1-7):kp/kd ≥ (20,0.5)/(60,2)/(100,3)/(100,1) + 1–2 组;
/// 频率 ≥ 3 档、步长 ≥ 3 档、抬脚 ≥ 2 档;更新 2 拍与 1 拍;两种质量比例
/// (×s 档阻尼同比缩放);×0.01 不缩放阻尼对照 + 阻尼清零组;240 Hz 与
/// 16 迭代参照;脚摩擦 1.4 变体;8 只耗时组。
fn build_matrix() -> Vec<RunCfg> {
    let mut all: Vec<RunCfg> = Vec::new();
    let mk = |name: &'static str, kp: f32, kd: f32, ue: usize, ms: f32| RunCfg {
        kp,
        kd,
        update_every: ue,
        mass_scale: ms,
        ..base_cfg(name)
    };
    // 主扫:kp/kd × 频率 × 步长 × 抬脚,真实尺度,u2(与探针 scan 同构,
    // 5×4×3×2 = 120 组)。
    for (kp, kd) in [
        (20.0f32, 0.5f32),
        (40.0, 1.0),
        (60.0, 2.0),
        (100.0, 3.0),
        (100.0, 1.0),
    ] {
        for f in [1.5f32, 2.0, 2.5, 3.0] {
            for stride in [0.06f32, 0.10, 0.15] {
                for lift in [0.03f32, 0.06] {
                    let name = Box::leak(
                        format!("kp{kp}_kd{kd}_u2_f{f}_s{stride}_l{lift}_real").into_boxed_str(),
                    );
                    let mut c = mk(name, kp, kd, 2, 1.0);
                    c.gait.freq = f;
                    c.gait.stride = stride;
                    c.gait.lift = lift;
                    all.push(c);
                }
            }
        }
    }
    // 每拍更新(u1)× 两档 kp,真实 + ×0.01:更新周期维度(步态 = 探针 g1)。
    let g1 = |c: &mut RunCfg| {
        c.gait.freq = 3.0;
        c.gait.stride = 0.15;
        c.gait.lift = 0.06;
    };
    for (kp, kd) in [(20.0f32, 0.5f32), (100.0, 3.0)] {
        for (ms, tag) in [(1.0f32, "real"), (0.01, "x001")] {
            let name =
                Box::leak(format!("kp{kp}_kd{kd}_u1_f3.0_s0.15_l0.06_{tag}").into_boxed_str());
            let mut c = mk(name, kp, kd, 1, ms);
            g1(&mut c);
            all.push(c);
        }
    }
    // u2 × 质量:两档 kp × 两档 ms,×s 档阻尼同比缩放(主口径;步态 = g1)。
    for (kp, kd) in [(20.0f32, 0.5f32), (100.0, 3.0)] {
        for (ms, tag) in [(1.0f32, "real"), (0.01, "x001")] {
            let name =
                Box::leak(format!("kp{kp}_kd{kd}_u2_f3.0_s0.15_l0.06_{tag}").into_boxed_str());
            let mut c = mk(name, kp, kd, 2, ms);
            g1(&mut c);
            all.push(c);
        }
    }
    // R1-5 对照:×0.01 不缩放阻尼(rapier 默认 0.1 不动)与阻尼清零(×0.01
    // 和真实尺度各一组;步态 = g1)。
    {
        let name = "x001_dampunscaled_kp100_kd3_u2_f3.0_s0.15_l0.06";
        let mut c = mk(name, 100.0, 3.0, 2, 0.01);
        c.damp = DampMode::Unscaled;
        g1(&mut c);
        all.push(c);
        let name = "x001_dampzero_kp100_kd3_u2_f3.0_s0.15_l0.06";
        let mut c = mk(name, 100.0, 3.0, 2, 0.01);
        c.damp = DampMode::Zero;
        g1(&mut c);
        all.push(c);
        let name = "real_dampzero_kp100_kd3_u2_f3.0_s0.15_l0.06";
        let mut c = mk(name, 100.0, 3.0, 2, 1.0);
        c.damp = DampMode::Zero;
        g1(&mut c);
        all.push(c);
    }
    // 参照组(只对比,不算通过):240 Hz(秒数与 60 Hz 组相同:站立 3 s、
    // 行走 60 s;30 Hz 目标更新不变 → 每 8 拍)、60 Hz 加 16 迭代(步态 = g1)。
    {
        let name = "ref_240hz_kp100_kd3_u8_f3.0_s0.15_l0.06";
        let mut c = mk(name, 100.0, 3.0, 8, 1.0);
        c.dt = 1.0 / 240.0;
        c.update_every = 8; // 30 Hz 目标更新不变
        c.walk_ticks = 14400; // 同 60 s
        g1(&mut c);
        all.push(c);
        let name = "ref_iter16_kp100_kd3_u2_f3.0_s0.15_l0.06";
        let mut c = mk(name, 100.0, 3.0, 2, 1.0);
        c.num_solver_iterations = 16;
        g1(&mut c);
        all.push(c);
    }
    // 脚摩擦变体(R1-4):1.4。
    {
        let name = "mu14_kp100_kd3_u2_f3.0_s0.15_l0.06_real";
        let mut c = mk(name, 100.0, 3.0, 2, 1.0);
        c.foot_mu = 1.4;
        g1(&mut c);
        all.push(c);
    }
    // 8 只耗时组(60 Hz 基准,真实尺度,kp100)。
    {
        let name = "eight_kp100_kd3_u2_f3.0_s0.15_l0.06_real";
        let mut c = mk(name, 100.0, 3.0, 2, 1.0);
        g1(&mut c);
        c.eight = true;
        all.push(c);
    }
    all
}
