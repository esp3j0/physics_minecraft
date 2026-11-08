//! REQ-012 A00-5 demo `learn`(R2,2026-11-08):双关节摆臂 `arm`,
//! Python PyTorch PPO(tools/train,CleanRL 底本)+ tract 部署路径评估。
//!
//! 问题(R2 再补充):用最简单的任务把三步管线跑通——rapier 训练 →
//! 换 MuJoCo → 回 rapier 微调。本二进制负责 Rust 侧的判据与报数:
//! - `l0 <种子> [n]`:L0 环境自检,解析逆运动学直接给关节目标,
//!   100(默认)个随机目标,2 s 内末端稳定在目标 1 cm 内的比例 ≥ 95%。
//! - `eval <onnx> [--hz60] [--out <名>]`:加载 ONNX 策略(tract,部署
//!   路径;归一化参数从同名 policy toml 读),跑 L1 判据矩阵:200 个
//!   随机目标,报成功率、最终距离,写 out/eval_<名>.csv。
//! - `order <onnx> [--hz60] [--out <名>]`:连杆顺序打乱(主体之前插入
//!   1、4、16 个睡着刚体),各 200 目标,报成功率与最终距离。
//! - `det <onnx>`:双跑逐位,同一策略同一批目标跑两遍,逐拍末端位置
//!   哈希比对,写 out/det_<名>.txt。
//! - `throughput <n>...`:吞吐,每秒控制步数(物理单列;策略前向在
//!   Python 侧测,见 tools/train 的吞吐报告),写 out/throughput.txt。
//! - `torchtract <onnx> <obs.bin> <torch_out.bin>`:torch↔tract 最大差。
//!
//! 统一口径(R2 再补充):60 Hz 物理、生产求解参数、无地面、30 Hz 控制。
//! 训练预算:每种子 ≤ 1e7 控制步或 30 分钟墙钟(Python 侧)。

#![allow(clippy::needless_range_loop)]
#![allow(clippy::disallowed_methods)]
mod arm;

use std::time::Instant;

use arm::{Env, ACT_DIM, EPISODE_CONTROLS, OBS_DIM, TARGET_R_MAX, TARGET_R_MIN};
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::ColliderBuilder;
use rapier3d::math::{Pose, Vector};
use rapier3d::pipeline::PhysicsWorld;

/// 策略包的归一化参数(obs_mean | obs_inv_std,各 OBS_DIM f32;REQ-012
/// §5.2:推理 = (obs − mean) × inv_std,clip ±10 后喂 ONNX)。
#[derive(Clone)]
struct Norm {
    mean: Vec<f32>,
    inv_std: Vec<f32>,
}

impl Norm {
    /// 从 policy.toml 读 obs_mean / obs_inv_std 两个数组(格式 = export_onnx
    /// 写出的 `[v, v, ...]` 行)。eval/order/det 必须与训练同口径归一化。
    fn from_policy_toml(toml_path: &str) -> Norm {
        let text = std::fs::read_to_string(toml_path).expect("read policy.toml");
        let mut mean = None;
        let mut inv_std = None;
        for line in text.lines() {
            let line = line.trim();
            if let Some(v) = line.strip_prefix("obs_mean = [") {
                mean = Some(parse_f32_list(v));
            } else if let Some(v) = line.strip_prefix("obs_inv_std = [") {
                inv_std = Some(parse_f32_list(v));
            }
        }
        let mean = mean.expect("policy.toml 缺 obs_mean");
        let inv_std = inv_std.expect("policy.toml 缺 obs_inv_std");
        assert_eq!(mean.len(), OBS_DIM, "obs_mean 长度");
        assert_eq!(inv_std.len(), OBS_DIM, "obs_inv_std 长度");
        Norm { mean, inv_std }
    }

    fn apply(&self, obs: &[f32; OBS_DIM]) -> [f32; OBS_DIM] {
        let mut out = [0.0f32; OBS_DIM];
        for i in 0..OBS_DIM {
            out[i] = ((obs[i] - self.mean[i]) * self.inv_std[i]).clamp(-10.0, 10.0);
        }
        out
    }
}

/// "[v, v, ...]" → Vec<f32>(容忍结尾 ']')。
fn parse_f32_list(s: &str) -> Vec<f32> {
    let s = s.trim_end_matches(']');
    s.split(',')
        .map(|v| v.trim().parse::<f32>().expect("parse obs list"))
        .collect()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cmd = args.first().map(String::as_str).unwrap_or("");
    match cmd {
        "l0" => {
            let seed: u64 = args.get(1).map(|s| s.parse().unwrap()).unwrap_or(0);
            let n: usize = args.get(2).map(|s| s.parse().unwrap()).unwrap_or(100);
            l0(seed, n);
        }
        "eval" => {
            let onnx = args.get(1).expect("eval <policy.onnx>");
            let hz60 = args.iter().any(|a| a == "--hz60");
            let variant = parse_variant(&args);
            let out = flag_value(&args, "--out").unwrap_or_else(|| "submitted".into());
            eval(onnx, hz60, variant, &out);
        }
        "order" => {
            let onnx = args.get(1).expect("order <policy.onnx>");
            let hz60 = args.iter().any(|a| a == "--hz60");
            let variant = parse_variant(&args);
            let out = flag_value(&args, "--out").unwrap_or_else(|| "submitted".into());
            order(onnx, hz60, variant, &out);
        }
        "det" => {
            let onnx = args.get(1).expect("det <policy.onnx>");
            let variant = parse_variant(&args);
            let out = flag_value(&args, "--out").unwrap_or_else(|| "submitted".into());
            det(onnx, variant, &out);
        }
        "throughput" => {
            let threads: Vec<usize> = args[1..]
                .iter()
                .map(|s| s.parse().expect("threads"))
                .collect();
            throughput(&threads);
        }
        "torchtract" => {
            // torchtract <policy.onnx> <obs.bin> <torch_out.bin> [--out 名]
            // L3:同一批观测上 torch(Python 导出)与 tract(部署路径)
            // 的输出最大绝对差,写 out/torch_tract_<名>.txt。
            let onnx = args
                .get(1)
                .expect("torchtract <onnx> <obs.bin> <torch_out.bin>");
            let obs_bin = args.get(2).expect("obs.bin");
            let torch_bin = args.get(3).expect("torch_out.bin");
            let out = flag_value(&args, "--out").unwrap_or_else(|| "submitted".into());
            torch_tract(onnx, obs_bin, torch_bin, &out);
        }
        _ => {
            eprintln!(
                "用法: learn l0 <种子> [n] | eval <onnx> [--hz60] [--out 名] | order <onnx> [--out 名] | det <onnx> [--out 名] | throughput <n>... | torchtract <onnx> <obs.bin> <torch_out.bin> [--out 名]"
            );
            std::process::exit(2);
        }
    }
}

fn flag_value(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|a| a == flag)
        .and_then(|i| args.get(i + 1))
        .cloned()
}

/// --mass <f> / --gain <f>:L5 的物理变体(只改物理,spec 不变)。
fn parse_variant(args: &[String]) -> arm::PhysVariant {
    let getf = |key: &str| -> Option<f32> {
        args.iter()
            .position(|a| a == key)
            .and_then(|i| args.get(i + 1))
            .and_then(|v| v.parse().ok())
    };
    arm::PhysVariant {
        mass_mult: getf("--mass").unwrap_or(1.0),
        gain_mult: getf("--gain").unwrap_or(1.0),
    }
}

// ── L0 环境自检 ─────────────────────────────────────────────────────────

/// L0:解析逆运动学直接给关节目标。n 个随机目标(与训练同采样分布),
/// 每个目标:出生角 ±0.5 随机 → 每控制周期把电机目标直接设为逆解角
/// (不做增量式,绕过策略),跑 3 s(90 控制步),统计最后 0.5 s(15
/// 控制步)末端是否**稳定**在目标 L0_DIST 内。比例 ≥ 95% 过。
fn l0(seed: u64, n: usize) {
    std::fs::create_dir_all("out").expect("mkdir out");
    let mut rng = ChaCha8Rng::seed_from_u64(seed);
    let mut out = String::from("i,tx,ty,d_max_last15,d_mean_last15,d_end,pass\n");
    let mut passed = 0usize;
    let t0 = Instant::now();
    for i in 0..n {
        let theta: f32 = rng.random_range(0.0..std::f32::consts::TAU);
        let rmin2 = TARGET_R_MIN * TARGET_R_MIN;
        let rmax2 = TARGET_R_MAX * TARGET_R_MAX;
        let r = (rng.random_range(0.0..1.0f32).mul_add(rmax2 - rmin2, rmin2)).sqrt();
        let target = Vector::new(r * theta.cos(), r * theta.sin(), 0.0);
        let (q1t, q2t) = arm::ik(target.x, target.y).expect("可达域内");
        // 手动回合:出生与训练一致(Env::new 的 reset),电机目标直接设
        // 逆解角(不做增量式,绕过策略;这是"物理 + 电机参数能否完成
        // 任务"的自检,作用同四足的开环步态探针)。
        let run = {
            let mut env = Env::new(seed.wrapping_add(9000 + i as u64));
            env.targets = [q1t, q2t];
            let mut last15 = Vec::with_capacity(15);
            for step in 0..EPISODE_CONTROLS {
                for dof in 0..ACT_DIM {
                    arm::set_target(&mut env.world, &env.arm, dof, env.targets[dof]);
                }
                for _ in 0..arm::TICKS_PER_CONTROL {
                    env.world.step();
                }
                env.step += 1;
                if step >= EPISODE_CONTROLS - 15 {
                    let tip = arm::tip_position(&env.world, &env.arm);
                    last15.push((tip - target).length());
                }
            }
            last15
        };
        let d_max = run.iter().cloned().fold(0.0, f32::max);
        let d_mean = run.iter().sum::<f32>() / run.len() as f32;
        let d_end = run.last().copied().unwrap_or(f32::NAN);
        let ok = d_max <= arm::L0_DIST;
        if ok {
            passed += 1;
        }
        out.push_str(&format!(
            "{i},{:.6},{:.6},{:.6},{:.6},{:.6},{}\n",
            target.x,
            target.y,
            d_max,
            d_mean,
            d_end,
            if ok { "PASS" } else { "FAIL" },
        ));
    }
    let ratio = passed as f32 / n as f32;
    std::fs::write("out/l0.txt", &out).expect("write l0");
    println!(
        "L0: {passed}/{n} = {ratio:.3}(门槛 ≥ {} @ {} m)elapsed {:.1}s → out/l0.txt",
        arm::L0_PASS_RATIO,
        arm::L0_DIST,
        t0.elapsed().as_secs_f32()
    );
}

// ── tract 策略加载 ──────────────────────────────────────────────────────

/// tract 的 RunnableModel = SimplePlan<TypedFact, Box<dyn TypedOp>>(0.23 的
/// prelude 在 tract_core 下;infer demo 的同名别名同款)。`run` 在
/// `Arc<Self>` 上,策略句柄用 Arc 装。
type RunnableModel = std::sync::Arc<
    tract_onnx::tract_core::model::RunnableModel<
        tract_onnx::tract_core::prelude::TypedFact,
        Box<dyn tract_onnx::tract_core::prelude::TypedOp>,
    >,
>;

/// 加载 ONNX actor(部署路径;输入 [1,12] f32,输出 [1,2])。
pub fn load_policy(path: &str) -> RunnableModel {
    use tract_onnx::prelude::*;
    tract_onnx::onnx()
        .model_for_path(path)
        .expect("load onnx")
        .with_input_fact(0, f32::fact([1, OBS_DIM]).into())
        .expect("input fact")
        .into_optimized()
        .expect("optimize")
        .into_runnable()
        .expect("runnable")
}

/// 确定性动作(均值)。
fn policy_action(model: &RunnableModel, obs: &[f32; OBS_DIM]) -> [f32; ACT_DIM] {
    use tract_onnx::tract_core::prelude::*;
    let arr =
        tract_onnx::tract_core::ndarray::Array2::<f32>::from_shape_vec((1, OBS_DIM), obs.to_vec())
            .expect("shape");
    let tv: tract_onnx::prelude::TValue = arr.into_tvalue();
    let result = model.run(tract_onnx::prelude::tvec![tv]).expect("run");
    let out = result
        .into_iter()
        .next()
        .expect("one output")
        .into_tensor()
        .into_plain_array::<f32>()
        .expect("f32")
        .into_raw_vec_and_offset()
        .0;
    [out[0], out[1]]
}

// ── L1 评估矩阵 ─────────────────────────────────────────────────────────

/// 一次评估回合的结果。
pub struct EpResult {
    /// 回合最后 0.5 s 末端始终在目标 REACH_DIST 内。
    pub last_hold: bool,
    /// 最终距离。
    pub d_end: f32,
    pub nan: bool,
    /// 活跃集顺序(order 场景报)。
    pub active_order: String,
}

/// 跑一个评估回合:种子决定目标与出生;策略 = tract ONNX(确定性均值)。
/// 返回逐拍末端距离(det 双跑哈希用Vec<f32> 的位模式哈希)。
#[allow(clippy::too_many_arguments)]
fn run_episode(
    model: &RunnableModel,
    seed: u64,
    order_insert: Option<usize>,
    hz60: bool,
    collect: bool,
    variant: arm::PhysVariant,
    args_norm: &Norm,
) -> (EpResult, Vec<u64>) {
    let mut rng = ChaCha8Rng::seed_from_u64(seed);
    let theta: f32 = rng.random_range(0.0..std::f32::consts::TAU);
    let rmin2 = TARGET_R_MIN * TARGET_R_MIN;
    let rmax2 = TARGET_R_MAX * TARGET_R_MAX;
    let r = (rng.random_range(0.0..1.0f32).mul_add(rmax2 - rmin2, rmin2)).sqrt();
    let target = Vector::new(r * theta.cos(), r * theta.sin(), 0.0);
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters = arm::production_params();
    if let Some(n) = order_insert {
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
    let arm = arm::spawn_arm(&mut world, variant.mass_mult);
    let mut env = Env {
        world,
        arm,
        target,
        targets: [0.0; ACT_DIM],
        last_action: [0.0; ACT_DIM],
        step: 0,
        done: false,
        variant,
    };
    // 出生角:与 Env::reset 同流(前 3 个随机数 = θ、r、q1;第 4 = q2)。
    let _ = theta;
    let _ = r;
    let q1: f32 = rng.random_range(-arm::INIT_Q_RANGE..arm::INIT_Q_RANGE);
    let q2: f32 = rng.random_range(-arm::INIT_Q_RANGE..arm::INIT_Q_RANGE);
    // 按 FK 摆出生位姿(同 Env::reset)。
    use rapier3d::math::Rotation;
    let ang1 = Rotation::from_rotation_z(q1);
    let total = q1 + q2;
    let ang2 = Rotation::from_rotation_z(total);
    let l1_pos = 0.5 * arm::LINK_LEN * Vector::new(q1.sin(), -q1.cos(), 0.0);
    let l2_pos = arm::LINK_LEN * Vector::new(q1.sin(), -q1.cos(), 0.0)
        + 0.5 * arm::LINK_LEN * Vector::new(total.sin(), -total.cos(), 0.0);
    for (h, pos, ang) in [
        (env.arm.links[1], l1_pos, ang1),
        (env.arm.links[2], l2_pos, ang2),
    ] {
        env.world.bodies.get_mut(h).expect("link").set_position(
            Pose {
                translation: pos,
                rotation: ang,
                ..Pose::IDENTITY
            },
            false,
        );
    }
    for (dof, q) in [q1, q2].into_iter().enumerate() {
        arm::set_target(&mut env.world, &env.arm, dof, q);
    }
    env.targets = [q1, q2];

    let ticks_per = if hz60 { 1 } else { arm::TICKS_PER_CONTROL };
    let mut last15: Vec<f32> = Vec::with_capacity(15);
    let mut hashes: Vec<u64> = Vec::new();
    let mut nan = false;
    let controls = if hz60 {
        EPISODE_CONTROLS * 2
    } else {
        EPISODE_CONTROLS
    };
    for step in 0..controls {
        let obs = arm::observe(&env.world, &env.arm, env.target, &env.last_action);
        if !obs.iter().all(|v| v.is_finite()) {
            nan = true;
            break;
        }
        let nobs = args_norm.apply(&obs);
        let action = policy_action(model, &nobs);
        // 增量式目标(同训练口径)。
        for dof in 0..ACT_DIM {
            env.targets[dof] = (env.targets[dof] + arm::ACT_SCALE * action[dof])
                .clamp(-arm::JOINT_LIMIT, arm::JOINT_LIMIT);
        }
        for t in 0..ticks_per {
            if t == 0 || hz60 {
                for dof in 0..ACT_DIM {
                    arm::set_target(&mut env.world, &env.arm, dof, env.targets[dof]);
                }
            }
            env.world.step();
        }
        env.last_action = action;
        env.step += 1;
        let tip = arm::tip_position(&env.world, &env.arm);
        let d = (tip - target).length();
        if step >= controls - 15 {
            last15.push(d);
        }
        if collect {
            let mut th = 0xcbf2_9ce4_8422_2325u64;
            for v in [tip.x, tip.y, tip.z] {
                for b in &v.to_ne_bytes() {
                    th ^= u64::from(*b);
                    th = th.wrapping_mul(0x100_0000_01b3);
                }
            }
            hashes.push(th);
        }
        let _ = step;
    }
    let hold = last15.len() == 15 && last15.iter().all(|d| *d <= arm::REACH_DIST);
    let d_end = last15.last().copied().unwrap_or(f32::NAN);
    let active_order = {
        let mut s = String::new();
        for h in env.world.islands.active_bodies() {
            if let Some(pos) = env.arm.links.iter().position(|l| *l == h) {
                s.push_str(&pos.to_string());
                s.push('-');
            }
        }
        if s.ends_with('-') {
            s.pop();
        }
        if s.is_empty() {
            s.push_str("none");
        }
        s
    };
    (
        EpResult {
            last_hold: hold,
            d_end,
            nan,
            active_order,
        },
        hashes,
    )
}

/// L1 判据矩阵:200 个随机目标(种子 1..=200),报成功率与最终距离。
fn eval(onnx: &str, hz60: bool, variant: arm::PhysVariant, name: &str) {
    std::fs::create_dir_all("out").expect("mkdir out");
    let model = load_policy(onnx);
    let norm = Norm::from_policy_toml(&onnx.replace(".onnx", ".toml"));
    let n = 200usize;
    let mut out = String::from("target_i,seed,hold,d_end,nan,active_order\n");
    let mut pass = 0usize;
    let mut dists: Vec<f32> = Vec::new();
    let mut nan_count = 0usize;
    for i in 0..n {
        let (r, _) = run_episode(&model, 1 + i as u64, None, hz60, false, variant, &norm);
        if r.nan {
            nan_count += 1;
        }
        if r.last_hold && !r.nan {
            pass += 1;
        }
        dists.push(r.d_end);
        out.push_str(&format!(
            "{},{},{},{:.6},{},{}\n",
            i + 1,
            1 + i as u64,
            if r.last_hold { 1 } else { 0 },
            r.d_end,
            r.nan,
            r.active_order
        ));
    }
    dists.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let ratio = pass as f32 / n as f32;
    let median = dists[n / 2];
    std::fs::write(format!("out/eval_{name}.csv"), &out).expect("write eval");
    println!(
        "L1 [{name}]: {pass}/{n} = {ratio:.3}(门槛 ≥ 0.90)d_end 中位数 {median:.4} m(门槛 ≤ 0.02)nan {nan_count}"
    );
}

/// 连杆顺序打乱:主体之前插入 n ∈ {1,4,16} 个睡着刚体,各 200 目标。
fn order(onnx: &str, hz60: bool, variant: arm::PhysVariant, name: &str) {
    std::fs::create_dir_all("out").expect("mkdir out");
    let model = load_policy(onnx);
    let norm = Norm::from_policy_toml(&onnx.replace(".onnx", ".toml"));
    let n = 200usize;
    let mut out = String::from("setup,target_i,seed,hold,d_end,nan,active_order\n");
    for (tag, ins) in [("insert1", 1usize), ("insert4", 4), ("insert16", 16)] {
        let mut pass = 0usize;
        let mut dists: Vec<f32> = Vec::new();
        for i in 0..n {
            let (r, _) = run_episode(&model, 1 + i as u64, Some(ins), hz60, false, variant, &norm);
            if r.last_hold && !r.nan {
                pass += 1;
            }
            dists.push(r.d_end);
            out.push_str(&format!(
                "{tag},{},{},{:.6},{},{},{}\n",
                i + 1,
                1 + i as u64,
                if r.last_hold { 1 } else { 0 },
                r.d_end,
                r.nan,
                r.active_order
            ));
        }
        dists.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        println!(
            "order [{tag}]: {pass}/{n} = {:.3} d_end 中位数 {:.4}",
            pass as f32 / n as f32,
            dists[n / 2]
        );
    }
    std::fs::write(format!("out/eval_order_{name}.csv"), &out).expect("write order");
}

/// det:同一策略同一批目标(前 20 个种子)双跑,逐拍末端哈希比对。
fn det(onnx: &str, variant: arm::PhysVariant, name: &str) {
    std::fs::create_dir_all("out").expect("mkdir out");
    let model = load_policy(onnx);
    let norm = Norm::from_policy_toml(&onnx.replace(".onnx", ".toml"));
    let mut out = String::new();
    let mut all_same = true;
    for i in 0..20u64 {
        let seed = 1 + i;
        let (_, h1) = run_episode(&model, seed, None, false, true, variant, &norm);
        let (_, h2) = run_episode(&model, seed, None, false, true, variant, &norm);
        let same = h1 == h2;
        let diverge = h1
            .iter()
            .zip(&h2)
            .position(|(a, b)| a != b)
            .map(|p| p.to_string())
            .unwrap_or_else(|| "-".to_string());
        all_same &= same;
        out.push_str(&format!(
            "seed{seed},{},{},{}\n",
            h1.len(),
            if same { "IDENTICAL" } else { "DIFFER" },
            diverge
        ));
    }
    std::fs::write(format!("out/det_{name}.txt"), &out).expect("write det");
    println!("det: all_same {all_same} → out/det_{name}.txt");
}

/// torch vs tract:同一批观测(Python 侧 torch_export_obs 导出),tract
/// 逐条推理,与 torch 输出比最大绝对差。判据 L3:≤ 1e-5。
fn torch_tract(onnx: &str, obs_bin: &str, torch_bin: &str, name: &str) {
    use tract_onnx::tract_core::ndarray::Array2;
    use tract_onnx::tract_core::prelude::*;
    let model = load_policy(onnx);
    let obs_bytes = std::fs::read(obs_bin).expect("read obs.bin");
    let torch_bytes = std::fs::read(torch_bin).expect("read torch_out.bin");
    let n = obs_bytes.len() / 4 / OBS_DIM;
    assert_eq!(torch_bytes.len(), n * ACT_DIM * 4, "torch_out 长度不符");
    let mut max_diff = 0.0f32;
    let mut nan = false;
    for i in 0..n {
        let mut x = [0.0f32; OBS_DIM];
        for (k, c) in obs_bytes[i * OBS_DIM * 4..(i + 1) * OBS_DIM * 4]
            .as_chunks::<4>()
            .0
            .iter()
            .enumerate()
        {
            x[k] = f32::from_le_bytes([c[0], c[1], c[2], c[3]]);
        }
        let arr = Array2::<f32>::from_shape_vec((1, OBS_DIM), x.to_vec()).expect("shape");
        let tv: TValue = arr.into_tvalue();
        let result = model.run(tvec![tv]).expect("run");
        let out = result
            .into_iter()
            .next()
            .expect("one output")
            .into_tensor()
            .into_plain_array::<f32>()
            .expect("f32")
            .into_raw_vec_and_offset()
            .0;
        for j in 0..ACT_DIM {
            let c = &torch_bytes[(i * ACT_DIM + j) * 4..(i * ACT_DIM + j) * 4 + 4];
            let t = f32::from_le_bytes([c[0], c[1], c[2], c[3]]);
            if !out[j].is_finite() || !t.is_finite() {
                nan = true;
            }
            max_diff = max_diff.max((out[j] - t).abs());
        }
    }
    std::fs::create_dir_all("out").expect("mkdir out");
    let txt = format!("n,{n}\nmax_abs_diff,{max_diff:.3e}\nnan,{nan}\n");
    std::fs::write(format!("out/torch_tract_{name}.txt"), &txt).expect("write");
    println!("torch vs tract: n {n} max_abs_diff {max_diff:.3e}(门槛 ≤ 1e-5)nan {nan}");
}

/// 吞吐:N 线程各跑独立环境,纯物理步进(不含策略),报每秒控制步数
/// 与每控制周期物理耗时。策略前向/训练吞吐在 Python 侧(tools/train)
/// 测并写进 tools/train README,两边分开报。
fn throughput(threads: &[usize]) {
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;
    std::fs::create_dir_all("out").expect("mkdir");
    let mut out = String::from(
        "# 每线程 1 只环境 × 10 s 纯物理(观测 + 步进,不含策略前向);\n\
         # Python 侧网络与训练吞吐见 tools/train/README.md。\n",
    );
    out.push_str("threads,steps_per_s,physics_ms_per_control,ms_per_env_per_s\n");
    for &n in threads {
        let total = Arc::new(AtomicU64::new(0));
        let phys_us = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicU64::new(0));
        let mut handles = Vec::new();
        for tid in 0..n {
            let total = Arc::clone(&total);
            let phys_us = Arc::clone(&phys_us);
            let stop = Arc::clone(&stop);
            handles.push(std::thread::spawn(move || {
                let mut e = Env::new(7000 + tid as u64);
                let mut rng = ChaCha8Rng::seed_from_u64(9000 + tid as u64);
                let mut local = 0u64;
                let mut ptime = 0u64;
                while stop.load(Ordering::Relaxed) == 0 {
                    // 随机小动作(模拟策略输出;不为测确定性,只为不让
                    // 目标角恒定)。
                    let a = [rng.random_range(-1.0..1.0), rng.random_range(-1.0..1.0)];
                    let t0 = Instant::now();
                    let _ = e.control_step(&a, false);
                    ptime += t0.elapsed().as_micros() as u64;
                    if e.done {
                        e.reset(&mut rng);
                    }
                    local += 1;
                }
                total.fetch_add(local, Ordering::Relaxed);
                phys_us.fetch_add(ptime, Ordering::Relaxed);
            }));
        }
        std::thread::sleep(std::time::Duration::from_secs_f32(10.0));
        stop.store(1, Ordering::Relaxed);
        for h in handles {
            h.join().expect("join");
        }
        let steps = total.load(Ordering::Relaxed) as f64;
        let sps = steps / 10.0;
        let per = phys_us.load(Ordering::Relaxed) as f64 / steps / 1000.0;
        out.push_str(&format!("{n},{sps:.0},{per:.4},-\n"));
        println!("threads {n}: {sps:.0} controls/s physics {per:.3} ms/control");
    }
    std::fs::write("out/throughput.txt", &out).expect("write throughput");
}
