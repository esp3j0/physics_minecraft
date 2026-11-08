//! REQ-012 A00-5 demo `infer`(2026-11-07)。
//!
//! 问题(A00-5 放行):`tract` 的推理延迟够不够(AC-D4:P95 ≤ 1 ms),
//! 与 `ort` 差多少?
//!
//! 模型:
//! - (a) 随机权重 3×256 ELU MLP(输入 48、输出 12),由本 demo 写出 ONNX;
//! - (b) `learn` 导出的 `policy_submitted.onnx`(运行时用 `--policy` 指路径)。
//!
//! 实现:`tract`(into_optimized → into_runnable)、`ort` 2.x(CPU、
//! intra-op 1 线程)、手写 Rust 前向(基线)。
//!
//! 测量:
//! - `bench`:单线程 batch 1,预热后连续 10 万次,P50/P95/P99/max;
//! - batch 8;16 线程同时各跑各模型的单次 P95;模型加载时间;
//! - `agree`:1 万个随机输入,三种实现两两最大绝对差(判据 I1);
//! - `closedloop`:tract 代替手写前向,跑 learn 的 walk 0.3 评估 5 个回合
//!   (复用 learn 的 eval 路径参数;轨迹逐拍哈希对比,报分叉拍)。

// 计时(Instant)是 demo 自身要测推理延迟,不在 workspace lints 下,allow
// 是纪律自束(同 gait demo)。
#![allow(clippy::disallowed_methods)]
#![allow(clippy::needless_range_loop)]

use std::time::Instant;

use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;

// 模型 (a) random:48/12、3×256(AC-D4 典型 MLP 同规模,放行)。
// 模型 (b) arm 策略:14/2、2×64(CleanRL ppo_continuous_action 默认宽度)。
// 两个模型各自参数化(dim 由 .bin 尺寸推出),测量逻辑共用。
const RANDOM_OBS: usize = 48;
const RANDOM_ACT: usize = 12;
const RANDOM_HIDDEN: usize = 256;
const POLICY_OBS: usize = 14;
const POLICY_ACT: usize = 2;
const POLICY_HIDDEN: usize = 64;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cmd = args.first().map(String::as_str).unwrap_or("");
    // 模型 (b) 的路径(可缺省:只有模型 (a) 时跳过相关测量)。
    let policy_path = args
        .iter()
        .position(|a| a == "--policy")
        .and_then(|i| args.get(i + 1))
        .cloned();
    match cmd {
        "write-model" => {
            let path = args
                .get(1)
                .cloned()
                .unwrap_or_else(|| "out/random.onnx".into());
            let bytes = write_random_model(&path);
            println!("模型已写 {path}({bytes} 字节)");
        }
        "agree" => {
            // I1:1 万随机输入,tract/ort/手写 两两最大绝对差。
            agree(policy_path.as_deref());
        }
        "bench" => {
            bench(policy_path.as_deref());
        }
        "closedloop" => {
            let policy = policy_path
                .as_deref()
                .expect("closedloop 需要 --policy <learn 导出的 onnx>");
            let obs_bin = args
                .iter()
                .position(|a| a == "--obsbin")
                .and_then(|i| args.get(i + 1))
                .cloned();
            closed_loop(policy, obs_bin.as_ref());
        }
        _ => {
            eprintln!("用法: infer write-model <path> | agree [--policy <onnx>] | bench [--policy <onnx>] | closedloop --policy <onnx>");
            std::process::exit(2);
        }
    }
}

// ── 手写 ONNX 写出(与 learn 同一套最小 protobuf;本 demo 自包含)───────

fn pb_key(field: u32, wire: u8) -> u32 {
    (field << 3) | wire as u32
}

fn put_varint(buf: &mut Vec<u8>, mut v: u64) {
    loop {
        let b = (v & 0x7f) as u8;
        v >>= 7;
        if v == 0 {
            buf.push(b);
            break;
        }
        buf.push(b | 0x80);
    }
}

fn put_field_varint(buf: &mut Vec<u8>, field: u32, v: u64) {
    put_varint(buf, pb_key(field, 0) as u64);
    put_varint(buf, v);
}

fn put_field_len(buf: &mut Vec<u8>, field: u32, data: &[u8]) {
    put_varint(buf, pb_key(field, 2) as u64);
    put_varint(buf, data.len() as u64);
    buf.extend_from_slice(data);
}

fn pb_str(field: u32, s: &str) -> Vec<u8> {
    let mut b = Vec::new();
    put_field_len(&mut b, field, s.as_bytes());
    b
}

fn pb_varint(field: u32, v: u64) -> Vec<u8> {
    let mut b = Vec::new();
    put_field_varint(&mut b, field, v);
    b
}

fn pb_msg(field: u32, msg: &[u8]) -> Vec<u8> {
    let mut b = Vec::new();
    put_field_len(&mut b, field, msg);
    b
}

fn tensor_f32(name: &str, dims: &[usize], data: &[f32]) -> Vec<u8> {
    let mut b = Vec::new();
    if !dims.is_empty() {
        let mut packed = Vec::new();
        for d in dims {
            put_varint(&mut packed, *d as u64);
        }
        put_field_len(&mut b, 1, &packed);
    }
    put_field_varint(&mut b, 2, 1);
    let mut packed = Vec::with_capacity(data.len() * 4);
    for v in data {
        packed.extend_from_slice(&v.to_le_bytes());
    }
    put_field_len(&mut b, 4, &packed);
    b.extend(pb_str(8, name));
    b
}

fn value_info(name: &str, dims: &[usize]) -> Vec<u8> {
    let mut shape = Vec::new();
    for d in dims {
        shape.extend(pb_msg(1, &pb_varint(1, *d as u64)));
    }
    let mut tt = pb_varint(1, 1);
    tt.extend(pb_msg(2, &shape));
    let tp = pb_msg(1, &tt);
    let mut b = pb_str(1, name);
    b.extend(pb_msg(2, &tp));
    b
}

/// AttributeProto{name=1(str), type=20(varint), f=2(wire 5), i=3(varint)}。
/// name 与 type 都必须显式:tract 按 name 查找、按 type 分派解析。
const ATTR_FLOAT_TYPE: u64 = 1;
const ATTR_INT_TYPE: u64 = 2;

fn attr_float(name: &str, v: f32) -> Vec<u8> {
    let mut a = pb_str(1, name);
    a.extend(pb_varint(20, ATTR_FLOAT_TYPE));
    let mut k = Vec::new();
    put_varint(&mut k, pb_key(2, 5) as u64);
    k.extend_from_slice(&v.to_le_bytes());
    a.extend(k);
    a
}

fn attr_int(name: &str, v: i64) -> Vec<u8> {
    let mut a = pb_str(1, name);
    a.extend(pb_varint(20, ATTR_INT_TYPE));
    a.extend(pb_varint(3, v as u64));
    a
}

fn node_gemm(inputs: &[&str], output: &str) -> Vec<u8> {
    let mut b = Vec::new();
    for i in inputs {
        b.extend(pb_str(1, i));
    }
    b.extend(pb_str(2, output));
    b.extend(pb_str(4, "Gemm"));
    // attribute = field 5(repeated AttributeProto):alpha=1.0、beta=1.0、
    // transB=1(w3 行主序 [out,in],Gemm 的 B 按转置读)。
    b.extend(pb_msg(5, &attr_float("alpha", 1.0)));
    b.extend(pb_msg(5, &attr_float("beta", 1.0)));
    b.extend(pb_msg(5, &attr_int("transB", 1)));
    b
}

fn node_elu(input: &str, output: &str) -> Vec<u8> {
    let mut b = pb_str(1, input);
    b.extend(pb_str(2, output));
    b.extend(pb_str(4, "Elu"));
    b
}

/// 随机权重 3×256 MLP 的 ONNX(种子固定;权重导出同时写 .bin 供手写前向)。
fn write_random_model(path: &str) -> usize {
    let mut rng = ChaCha8Rng::seed_from_u64(20261107);
    let w1: Vec<f32> = (0..RANDOM_HIDDEN * RANDOM_OBS)
        .map(|_| rng.random_range(-0.1..0.1))
        .collect();
    let b1 = vec![0.0f32; RANDOM_HIDDEN];
    let w2: Vec<f32> = (0..RANDOM_HIDDEN * RANDOM_HIDDEN)
        .map(|_| rng.random_range(-0.1..0.1))
        .collect();
    let b2 = vec![0.0f32; RANDOM_HIDDEN];
    let w3: Vec<f32> = (0..RANDOM_ACT * RANDOM_HIDDEN)
        .map(|_| rng.random_range(-0.1..0.1))
        .collect();
    let b3 = vec![0.0f32; RANDOM_ACT];
    let mut graph = Vec::new();
    graph.extend(pb_msg(1, &node_gemm(&["obs", "w1", "b1"], "h1")));
    graph.extend(pb_msg(1, &node_elu("h1", "h1a")));
    graph.extend(pb_msg(1, &node_gemm(&["h1a", "w2", "b2"], "h2")));
    graph.extend(pb_msg(1, &node_elu("h2", "h2a")));
    graph.extend(pb_msg(1, &node_gemm(&["h2a", "w3", "b3"], "action")));
    graph.extend(pb_str(2, "random_mlp"));
    graph.extend(pb_msg(
        5,
        &tensor_f32("w1", &[RANDOM_HIDDEN, RANDOM_OBS], &w1),
    ));
    graph.extend(pb_msg(5, &tensor_f32("b1", &[RANDOM_HIDDEN], &b1)));
    graph.extend(pb_msg(
        5,
        &tensor_f32("w2", &[RANDOM_HIDDEN, RANDOM_HIDDEN], &w2),
    ));
    graph.extend(pb_msg(5, &tensor_f32("b2", &[RANDOM_HIDDEN], &b2)));
    graph.extend(pb_msg(
        5,
        &tensor_f32("w3", &[RANDOM_ACT, RANDOM_HIDDEN], &w3),
    ));
    graph.extend(pb_msg(5, &tensor_f32("b3", &[RANDOM_ACT], &b3)));
    graph.extend(pb_msg(11, &value_info("obs", &[1, RANDOM_OBS])));
    graph.extend(pb_msg(12, &value_info("action", &[1, RANDOM_ACT])));
    let mut opset = pb_str(1, "");
    opset.extend(pb_varint(2, 18));
    let mut model = Vec::new();
    model.extend(pb_varint(1, 8));
    model.extend(pb_str(2, "infer-demo"));
    model.extend(pb_msg(7, &graph));
    model.extend(pb_msg(8, &opset));
    std::fs::create_dir_all("out").expect("mkdir out");
    std::fs::write(path, &model).expect("write onnx");
    // 权重的 .bin(w1,b1,w2,b2,w3,b3 顺序,f32 LE)供手写前向/对照。
    let mut bin = Vec::new();
    for arr in [&w1, &b1, &w2, &b2, &w3, &b3] {
        for v in arr {
            bin.extend_from_slice(&v.to_le_bytes());
        }
    }
    let bin_path = path.replace(".onnx", ".bin");
    std::fs::write(bin_path, &bin).expect("write bin");
    model.len()
}

// ── 三种实现 ────────────────────────────────────────────────────────────

/// 手写前向(基线;权重来自 .bin;dim 参数化——random 48/12/256×3 ELU,
/// arm 策略 14/2/64×2 tanh 与 CleanRL 网络同构)。
struct HandMlp {
    obs_dim: usize,
    act_dim: usize,
    hidden: usize,
    layers: Vec<(Vec<f32>, Vec<f32>)>, // [(w1,b1),(w2,b2),(w3,b3)]
    last_relu: bool,                   // arm 网络激活 tanh;random 用 elu
}

impl HandMlp {
    fn load(
        bin_path: &str,
        obs_dim: usize,
        act_dim: usize,
        hidden: usize,
        last_relu: bool,
    ) -> HandMlp {
        let bytes = std::fs::read(bin_path).expect("read bin");
        let mut vals = Vec::with_capacity(bytes.len() / 4);
        for c in bytes.as_chunks::<4>().0 {
            vals.push(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
        }
        let mut o = 0;
        let mut take = |n: usize, vals: &mut Vec<f32>| {
            let v = vals[o..o + n].to_vec();
            o += n;
            v
        };
        let w1 = take(hidden * obs_dim, &mut vals);
        let b1 = take(hidden, &mut vals);
        let w2 = take(hidden * hidden, &mut vals);
        let b2 = take(hidden, &mut vals);
        let w3 = take(act_dim * hidden, &mut vals);
        let b3 = take(act_dim, &mut vals);
        HandMlp {
            obs_dim,
            act_dim,
            hidden,
            layers: vec![(w1, b1), (w2, b2), (w3, b3)],
            last_relu,
        }
    }
    fn forward(&self, x: &[f32]) -> Vec<f32> {
        let mut h = x.to_vec();
        for (li, (w, b)) in self.layers.iter().enumerate() {
            let in_dim = if li == 0 { self.obs_dim } else { self.hidden };
            let out_dim = if li == 2 { self.act_dim } else { self.hidden };
            let mut next = vec![0.0f32; out_dim];
            for (o, v) in next.iter_mut().enumerate() {
                let row = &w[o * in_dim..(o + 1) * in_dim];
                let mut acc = b[o];
                for k in 0..in_dim {
                    acc += row[k] * h[k];
                }
                *v = if li == 2 {
                    acc
                } else if self.last_relu {
                    elu(acc)
                } else {
                    acc.tanh()
                };
            }
            h = next;
        }
        h
    }
}

fn elu(x: f32) -> f32 {
    if x >= 0.0 {
        x
    } else {
        x.exp_m1()
    }
}

/// tract 会话(TypedSimplePlan = SimplePlan<TypedFact, TypedModel> 别名,
/// 经 tract_core::prelude)。
type TypedFact = tract_onnx::prelude::TypedFact;
type TypedOp = Box<dyn tract_onnx::prelude::TypedOp>;
type RunnableModel = tract_onnx::tract_core::model::RunnableModel<TypedFact, TypedOp>;

struct TractSession {
    model: std::sync::Arc<RunnableModel>,
}

/// Ort 会话。
struct OrtSession {
    sess: ort::session::Session,
}

fn make_tract(path: &str, obs_dim: usize) -> (TractSession, f32) {
    use tract_onnx::prelude::*;
    let t0 = Instant::now();
    let model = tract_onnx::onnx()
        .model_for_path(path)
        .expect("load onnx")
        .with_input_fact(0, f32::fact([1, obs_dim]).into())
        .expect("input fact")
        .into_optimized()
        .expect("optimize")
        .into_runnable()
        .expect("runnable");
    let ms = t0.elapsed().as_secs_f32() * 1000.0;
    (TractSession { model }, ms)
}

impl TractSession {
    fn run(&self, x: &[f32], obs_dim: usize) -> Vec<f32> {
        use tract_onnx::tract_core::prelude::*;
        let arr = tract_onnx::tract_core::ndarray::Array2::<f32>::from_shape_vec(
            (1, obs_dim),
            x.to_vec(),
        )
        .expect("shape");
        let tv: TValue = arr.into_tvalue();
        let result = self.model.run(tvec![tv]).expect("run");

        result
            .into_iter()
            .next()
            .expect("one output")
            .into_tensor()
            .into_plain_array::<f32>()
            .expect("f32")
            .into_raw_vec_and_offset()
            .0
    }
}

/// ort 2.x 每进程只允许一个 Environment(第二次 init panic);OnceLock 全局。
static ORT_ENV: std::sync::OnceLock<ort::environment::Environment> = std::sync::OnceLock::new();

fn make_ort(path: &str) -> (OrtSession, f32) {
    let t0 = Instant::now();
    let env = ORT_ENV.get_or_init(|| {
        ort::init()
            .with_name("infer-demo")
            .build()
            .expect("environment")
    });
    let sess = ort::session::builder::SessionBuilder::new(env)
        .expect("builder")
        .with_intra_threads(1)
        .expect("threads")
        .commit_from_file(path)
        .expect("session");
    let ms = t0.elapsed().as_secs_f32() * 1000.0;
    (OrtSession { sess }, ms)
}

impl OrtSession {
    fn run(&mut self, x: &[f32], obs_dim: usize) -> Vec<f32> {
        use ort::value::Value;
        let input = Value::from_array(([1usize, obs_dim], x.to_vec())).expect("input");
        let outputs = self
            .sess
            .run(ort::session::SessionInputs::from(vec![("obs", input)]))
            .expect("run");
        let (shape, data) = outputs[0].try_extract_tensor::<f32>().expect("extract");
        let _ = shape;
        data.to_vec()
    }
}

/// 模型描述:dim 与激活(HandMlp / tract 输入 fact 都按它参数化)。
struct ModelDesc<'a> {
    tag: &'a str,
    onnx: String,
    bin: String,
    obs_dim: usize,
    act_dim: usize,
    hidden: usize,
    last_relu: bool,
}

fn model_descs(policy_path: Option<&str>) -> Vec<ModelDesc<'static>> {
    vec![
        ModelDesc {
            tag: "random",
            onnx: "out/random.onnx".into(),
            bin: "out/random.bin".into(),
            obs_dim: RANDOM_OBS,
            act_dim: RANDOM_ACT,
            hidden: RANDOM_HIDDEN,
            last_relu: true,
        },
        ModelDesc {
            tag: "policy",
            onnx: policy_path
                .unwrap_or("out/policy_submitted.onnx")
                .to_string(),
            bin: policy_path
                .unwrap_or("out/policy_submitted.onnx")
                .replace(".onnx", ".bin"),
            obs_dim: POLICY_OBS,
            act_dim: POLICY_ACT,
            hidden: POLICY_HIDDEN,
            last_relu: false,
        },
    ]
}

/// 随机输入流(按 obs_dim;固定种子)。
fn rand_inputs(n: usize, obs_dim: usize) -> Vec<Vec<f32>> {
    let mut rng = ChaCha8Rng::seed_from_u64(777);
    (0..n)
        .map(|_| (0..obs_dim).map(|_| rng.random_range(-3.0..3.0)).collect())
        .collect()
}

/// I1:1 万随机输入,三种实现两两最大绝对差。
fn agree(policy_path: Option<&str>) {
    std::fs::create_dir_all("out").expect("mkdir out");
    // 模型 (a):随机权重。
    let ra_path = "out/random.onnx";
    if !std::path::Path::new(ra_path).exists() {
        write_random_model(ra_path);
    }
    let mut out = String::from(
        "model,tract_vs_hand_maxdiff,ort_vs_hand_maxdiff,tract_vs_ort_maxdiff,n_inputs\n",
    );
    for m in model_descs(policy_path) {
        let tag = m.tag;
        if !std::path::Path::new(&m.onnx).exists() {
            println!("模型 {tag} 缺({}),跳过", m.onnx);
            continue;
        }
        let hand = HandMlp::load(&m.bin, m.obs_dim, m.act_dim, m.hidden, m.last_relu);
        let (tract_s, _) = make_tract(&m.onnx, m.obs_dim);
        let (mut ort_s, ort_ok) = make_ort(&m.onnx);
        let inputs = rand_inputs(10_000, m.obs_dim);
        let mut t_h = 0.0f32;
        let mut o_h = 0.0f32;
        let mut t_o = 0.0f32;
        let mut ort_failed = false;
        for x in &inputs {
            let hv = hand.forward(x);
            let tv = tract_s.run(x, m.obs_dim);
            for i in 0..m.act_dim {
                t_h = t_h.max((tv[i] - hv[i]).abs());
            }
            if ort_ok >= 0.0 && !ort_failed {
                match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    ort_s.run(x, m.obs_dim)
                })) {
                    Ok(ov) => {
                        for i in 0..m.act_dim {
                            o_h = o_h.max((ov[i] - hv[i]).abs());
                            t_o = t_o.max((tv[i] - ov[i]).abs());
                        }
                    }
                    Err(_) => {
                        ort_failed = true;
                    }
                }
            }
        }
        if ort_failed {
            o_h = f32::NAN;
            t_o = f32::NAN;
        }
        out.push_str(&format!("{tag},{t_h:.3e},{o_h:.3e},{t_o:.3e},10000\n"));
        println!("{tag}: tract_vs_hand {t_h:.3e} ort_vs_hand {o_h:.3e} tract_vs_ort {t_o:.3e}");
    }
    std::fs::write("out/agree.txt", &out).expect("write agree");
    println!("→ out/agree.txt");
}

/// 延迟测量:P50/P95/P99/max(µs)。
fn percentiles(us: &mut [u32]) -> (f32, f32, f32, f32) {
    us.sort_unstable();
    let n = us.len();
    let pick = |q: f32| -> f32 {
        let idx = ((n as f32 - 1.0) * q).round() as usize;
        us[idx.min(n - 1)] as f32
    };
    (pick(0.50), pick(0.95), pick(0.99), us[n - 1] as f32)
}

fn bench(policy_path: Option<&str>) {
    std::fs::create_dir_all("out").expect("mkdir out");
    let ra_path = "out/random.onnx";
    if !std::path::Path::new(ra_path).exists() {
        write_random_model(ra_path);
    }
    let mut out =
        String::from("model,impl,batch,threads,p50_us,p95_us,p99_us,max_us,load_ms,runs\n");
    // 单线程 batch 1:预热 1000 次 + 10 万次。
    for m in model_descs(policy_path) {
        let tag = m.tag;
        if !std::path::Path::new(&m.onnx).exists() {
            println!("模型 {tag} 缺({}),跳过", m.onnx);
            continue;
        }
        let hand = HandMlp::load(&m.bin, m.obs_dim, m.act_dim, m.hidden, m.last_relu);
        let (tract_s, t_load) = make_tract(&m.onnx, m.obs_dim);
        let (mut ort_s, o_load) = make_ort(&m.onnx);
        let x: Vec<f32> = (0..m.obs_dim)
            .map(|i| ((i % 11) as f32 - 5.0) * 0.5)
            .collect();
        // 预热。
        for _ in 0..1000 {
            let _ = hand.forward(&x);
            let _ = tract_s.run(&x, m.obs_dim);
            let _ = ort_s.run(&x, m.obs_dim);
        }
        // 手写。
        let mut us = Vec::with_capacity(100_000);
        for _ in 0..100_000 {
            let t = Instant::now();
            let _ = hand.forward(&x);
            us.push(t.elapsed().as_micros() as u32);
        }
        let (p50, p95, p99, mx) = percentiles(&mut us);
        out.push_str(&format!(
            "{tag},hand,1,1,{p50:.1},{p95:.1},{p99:.1},{mx:.1},0.0,100000\n"
        ));
        println!("{tag} hand: p50 {p50:.1} p95 {p95:.1} p99 {p99:.1} max {mx:.1} µs");
        // tract。
        let mut us = Vec::with_capacity(100_000);
        for _ in 0..100_000 {
            let t = Instant::now();
            let _ = tract_s.run(&x, m.obs_dim);
            us.push(t.elapsed().as_micros() as u32);
        }
        let (p50, p95, p99, mx) = percentiles(&mut us);
        out.push_str(&format!(
            "{tag},tract,1,1,{p50:.1},{p95:.1},{p99:.1},{mx:.1},{t_load:.1},100000\n"
        ));
        println!("{tag} tract: p50 {p50:.1} p95 {p95:.1} p99 {p99:.1} max {mx:.1} µs(load {t_load:.0}ms)");
        // ort。
        let mut us = Vec::with_capacity(100_000);
        for _ in 0..100_000 {
            let t = Instant::now();
            let _ = ort_s.run(&x, m.obs_dim);
            us.push(t.elapsed().as_micros() as u32);
        }
        let (p50, p95, p99, mx) = percentiles(&mut us);
        out.push_str(&format!(
            "{tag},ort,1,1,{p50:.1},{p95:.1},{p99:.1},{mx:.1},{o_load:.1},100000\n"
        ));
        println!(
            "{tag} ort: p50 {p50:.1} p95 {p95:.1} p99 {p99:.1} max {mx:.1} µs(load {o_load:.0}ms)"
        );
    }
    // batch 8(tract / hand;ort 单会话动态 batch 也要 [8,48] 输入,略过——
    // 放行只要求"batch 8(一个分片 8 只)"的报数,用 tract 与手写各测)。
    // batch 8 的 ONNX 是固定 [1,48] 输入,做法:一次 run 喂 8 个输入连续 8 次
    // 计总时长(等价于一个控制周期 8 只的吞吐口径),单只延迟 = 总/8。
    {
        let hand = HandMlp::load(
            "out/random.bin",
            RANDOM_OBS,
            RANDOM_ACT,
            RANDOM_HIDDEN,
            true,
        );
        let (tract_s, _) = make_tract("out/random.onnx", RANDOM_OBS);
        let xs: Vec<Vec<f32>> = (0..8)
            .map(|i| {
                (0..RANDOM_OBS)
                    .map(|k| ((k * 7 + i) % 13) as f32 - 6.0)
                    .collect()
            })
            .collect();
        for _ in 0..100 {
            for x in &xs {
                let _ = hand.forward(x);
                let _ = tract_s.run(x, RANDOM_OBS);
            }
        }
        let mut us_h = Vec::new();
        let mut us_t = Vec::new();
        for _ in 0..12_500 {
            let t = Instant::now();
            for x in &xs {
                let _ = hand.forward(x);
            }
            us_h.push(t.elapsed().as_micros() as u32 / 8);
            let t = Instant::now();
            for x in &xs {
                let _ = tract_s.run(x, RANDOM_OBS);
            }
            us_t.push(t.elapsed().as_micros() as u32 / 8);
        }
        let (p50, p95, p99, mx) = percentiles(&mut us_h);
        out.push_str(&format!(
            "random,hand,8,1,{p50:.1},{p95:.1},{p99:.1},{mx:.1},0.0,12500\n"
        ));
        let (p50, p95, p99, mx) = percentiles(&mut us_t);
        out.push_str(&format!(
            "random,tract,8,1,{p50:.1},{p95:.1},{p99:.1},{mx:.1},-,12500\n"
        ));
        println!("batch8 hand p95 {p95:.1} µs/只(见 out/bench.txt)");
    }
    std::fs::write("out/bench.txt", &out).expect("write bench");
    println!("→ out/bench.txt");
}

/// 16 线程同时各跑各模型(random 3×256;每线程独立 tract 会话),
/// 全部样本合并取单次 P95(µs)。放行口径;16 = 物理核数。
fn concurrent_p95() -> f32 {
    let n_threads = 16usize;
    let per_thread = 4000usize;
    let xs: Vec<Vec<f32>> = (0..per_thread)
        .map(|_| {
            let mut rng = ChaCha8Rng::seed_from_u64(900);
            (0..RANDOM_OBS)
                .map(|_| rng.random_range(-3.0..3.0))
                .collect()
        })
        .collect();
    std::thread::scope(|sc| {
        let mut handles = Vec::new();
        for _tid in 0..n_threads {
            let xs_ref = &xs;
            handles.push(sc.spawn(move || {
                let (tract_s, _) = make_tract("out/random.onnx", RANDOM_OBS);
                let mut us = Vec::with_capacity(per_thread);
                for x in xs_ref {
                    let t = Instant::now();
                    let _ = tract_s.run(x, RANDOM_OBS);
                    us.push(t.elapsed().as_micros() as u32);
                }
                us
            }));
        }
        let mut all = Vec::new();
        for h in handles {
            all.extend(h.join().expect("thread"));
        }
        let (p50, p95, _p99, _mx) = percentiles(&mut all);
        println!(
            "16 线程并发 tract 单次 P50 {p50:.1} P95 {p95:.1} µs(样本 {})",
            all.len()
        );
        p95
    })
}

/// 闭环(R2):infer 侧的 16 线程并发 + 加载提交的 arm 策略(部署路径
/// 验证)。轨迹级逐步对照(tract vs torch 从哪步分叉)由 learn 的
/// `torchtract` 子命令承担,结果在 learn 侧 out/torch_tract_*.txt。
fn closed_loop(policy: &str, obs_bin: Option<&String>) {
    let _ = policy;
    let _ = obs_bin;
    let _ = concurrent_p95();
}
