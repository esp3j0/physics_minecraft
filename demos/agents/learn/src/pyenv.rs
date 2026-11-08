//! `learn_env` Python 模块(R2):`VecEnv(n_envs, threads, seed, hz60)`。
//!
//! 接口(§5.3;后端适配器 tools/train/train/backends/rapier.py 包装它):
//! - `reset(seed) -> obs`            float32 [n_envs, obs_dim]
//! - `step(actions) -> (obs, reward_inputs, terminated, truncated)`
//!                                   actions float32 [n_envs, act_dim] 取值
//!                                   [-1,1];reward_inputs: dict[str, float32
//!                                   [n_envs, k]];回合结束的实例**自动 reset**
//!                                   (truncated = 到时;terminated 恒 False,
//!                                   本任务不设提前终止)。
//!
//! reward_inputs 键(tools/train/train/spec.py 同名约定):
//! `dist`(末端到目标距离)、`tip_vx`、`tip_vy`(末端速度)、
//! `tau1`、`tau2`(关节力矩)、`act1`、`act2`(上一动作)。
//!
//! 确定性(L2):reset 的种子 = seed ⊕ env_id·φ;每实例独立世界,动作
//! 序列一致时观测与线程数无关;step 各环境独立、合并按 env 序,逐位确定。
//! GIL:step/reset 前 numpy → Rust 拷贝数据,Rust 计算段不持 GIL
//! (Python::allow_threads),多线程真并行。

use crate::arm::{self, Env, ACT_DIM, OBS_DIM, REWARD_KEYS};
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray2};
use pyo3::prelude::*;
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;

/// reward_inputs 的键序(与 spec.py 的 REWARD_KEYS 一致):
/// `dist`(末端到目标距离)、`tip_vx`、`tip_vy`(末端速度)、
/// `tau1`、`tau2`(关节力矩)、`act1`、`act2`(上一动作)。
/// 键名与含义由 BackendSpec(tools/train/train/spec.py)约定;
/// 本模块按固定下标写入,不重复定义键表(单一事实来源在 spec.py)。

/// 多线程 VecEnv:每环境一个 Mutex<Env>(独立世界),按 chunk 切线程。
#[pyclass]
pub struct VecEnv {
    envs: Vec<std::sync::Mutex<Env>>,
    n_envs: usize,
    threads: usize,
    /// 60 Hz 对照口径(每拍设目标、步 1 拍)。
    hz60: bool,
    /// 每环境的基础种子(reset(seed) 时 env_i 用 seed ⊕ i·φ 派生)。
    base_seed: u64,
}

#[pymethods]
impl VecEnv {
    #[new]
    #[pyo3(signature = (n_envs, threads, seed, hz60 = false, mass_mult = 1.0, gain_mult = 1.0))]
    fn new(
        n_envs: usize,
        threads: usize,
        seed: u64,
        hz60: bool,
        mass_mult: f32,
        gain_mult: f32,
    ) -> PyResult<VecEnv> {
        let threads = threads.clamp(1, n_envs.max(1));
        let variant = crate::arm::PhysVariant {
            mass_mult,
            gain_mult,
        };
        let envs = (0..n_envs)
            .map(|i| {
                let es = env_seed(seed, i);
                std::sync::Mutex::new(Env::with_variant(es, variant))
            })
            .collect();
        Ok(VecEnv {
            envs,
            n_envs,
            threads,
            hz60,
            base_seed: seed,
        })
    }

    /// 重置全部环境。每环境的种子 = seed ⊕ i·0x9E3779B97F4A7C15(φ 黄金比,
    /// 与线程数无关)。返回 obs [n_envs, obs_dim] float32。
    fn reset<'py>(&mut self, py: Python<'py>, seed: u64) -> Bound<'py, PyArray2<f32>> {
        self.base_seed = seed;
        let mut obs = vec![0.0f32; self.n_envs * OBS_DIM];
        py.allow_threads(|| {
            for (i, e) in self.envs.iter().enumerate() {
                let es = env_seed(seed, i);
                let mut e = e.lock().expect("env lock");
                let mut rng = ChaCha8Rng::seed_from_u64(es);
                let o = e.reset(&mut rng);
                obs[i * OBS_DIM..(i + 1) * OBS_DIM].copy_from_slice(&o);
            }
        });
        {
            let arr = ndarray::Array2::from_shape_vec((self.n_envs, OBS_DIM), obs).expect("shape");
            arr.into_pyarray(py)
        }
    }

    /// 一个控制周期。actions [-1,1] [n_envs, act_dim];返回
    /// (obs, reward_inputs, terminated, truncated)。回合到时的实例自动
    /// reset(obs/reward_inputs 是 reset 后新回合首步的值;truncated[i] =
    /// 该实例本步到时)。terminated 恒 false(本任务不设提前终止)。
    #[allow(clippy::type_complexity)] // PyO3 签名要求的多路返回;拆 type 反而绕
    fn step<'py>(
        &mut self,
        py: Python<'py>,
        actions: PyReadonlyArray2<f32>,
    ) -> (
        Bound<'py, PyArray2<f32>>,
        Bound<'py, PyArray2<f32>>,
        Vec<bool>,
        Vec<bool>,
    ) {
        let acts = actions.as_array();
        assert_eq!(acts.nrows(), self.n_envs, "actions 行数 = n_envs");
        assert_eq!(acts.ncols(), ACT_DIM, "actions 列数 = act_dim");
        let mut obs = vec![0.0f32; self.n_envs * OBS_DIM];
        let mut rew = vec![0.0f32; self.n_envs * REWARD_KEYS];
        let mut truncated = vec![false; self.n_envs];
        let hz60 = self.hz60;
        let base_seed = self.base_seed;
        let threads = self.threads;
        let chunk = self.n_envs.div_ceil(threads);
        py.allow_threads(|| {
            // 线程池:每线程独占一段连续 env 段(静态切分;每环境的数值
            // 路径固定,合并按 env 序 → 观测与线程数无关)。输出缓冲与
            // 环境列表同步 chunks_mut 切开,线程只写自己的段(借用检查
            // 能静态证明不相交)。auto-reset 在线程内做:env 全局编号 =
            // 线程序×chunk + 段内序,reset 种子由 base_seed ⊕ env_id 派生
            // (与线程数无关)。
            std::thread::scope(|sc| {
                let mut handles = Vec::new();
                let mut obs_rest: &mut [f32] = &mut obs;
                let mut rew_rest: &mut [f32] = &mut rew;
                let mut trunc_rest: &mut [bool] = &mut truncated;
                let mut env_rest: &mut [std::sync::Mutex<Env>] = &mut self.envs;
                let mut tid = 0usize;
                while !env_rest.is_empty() {
                    let take = chunk.min(env_rest.len());
                    let (env_block, env_next) = env_rest.split_at_mut(take);
                    let (obs_block, obs_next) = obs_rest.split_at_mut(take * OBS_DIM);
                    let (rew_block, rew_next) = rew_rest.split_at_mut(take * REWARD_KEYS);
                    let (trunc_block, trunc_next) = trunc_rest.split_at_mut(take);
                    env_rest = env_next;
                    obs_rest = obs_next;
                    rew_rest = rew_next;
                    trunc_rest = trunc_next;
                    let acts_ref = &acts;
                    let base = tid * chunk;
                    tid += 1;
                    handles.push(sc.spawn(move || {
                        for (k, slot) in env_block.iter_mut().enumerate() {
                            let env_id = base + k;
                            let mut a = [0.0f32; ACT_DIM];
                            for j in 0..ACT_DIM {
                                a[j] = acts_ref[[env_id, j]].clamp(-1.0, 1.0);
                            }
                            let mut e = slot.lock().expect("env lock");
                            let rin = e.control_step(&a, hz60);
                            let o;
                            if e.done {
                                // 到时:truncated 置位,同步 auto-reset
                                // (obs/reward_inputs = 新回合首步;与主
                                // 线程 reset 用同一派生种子 → 逐位确定)。
                                trunc_block[k] = true;
                                let es = env_seed(base_seed, env_id).wrapping_add(0xAB00);
                                let mut rng = ChaCha8Rng::seed_from_u64(es);
                                o = e.reset(&mut rng);
                                let tip = arm::tip_position(&e.world, &e.arm);
                                let d = (tip - e.target).length();
                                let mut r0 = [0.0f32; REWARD_KEYS];
                                r0[0] = d;
                                rew_block[k * REWARD_KEYS..(k + 1) * REWARD_KEYS]
                                    .copy_from_slice(&r0);
                            } else {
                                o = arm::observe(&e.world, &e.arm, e.target, &e.last_action);
                                rew_block[k * REWARD_KEYS..(k + 1) * REWARD_KEYS]
                                    .copy_from_slice(&rin);
                            }
                            obs_block[k * OBS_DIM..(k + 1) * OBS_DIM].copy_from_slice(&o);
                        }
                    }));
                }
                for h in handles {
                    h.join().expect("step thread");
                }
            });
        });
        let terminated = vec![false; self.n_envs];
        (
            {
                let arr =
                    ndarray::Array2::from_shape_vec((self.n_envs, OBS_DIM), obs).expect("shape");
                arr.into_pyarray(py)
            },
            {
                let arr = ndarray::Array2::from_shape_vec((self.n_envs, REWARD_KEYS), rew)
                    .expect("shape");
                arr.into_pyarray(py)
            },
            terminated,
            truncated,
        )
    }

    /// L2 复现探针:对环境 env_id 用固定动作序列跑 n 步,返回逐拍观测的
    /// FNV 哈希列表(调用方对 1/4/16 线程的 VecEnv 各调一次,逐位比对)。
    /// 探针路径不复用线程池:先 reset 该环境(种子固定),再顺序步进。
    fn probe_env<'py>(&mut self, py: Python<'py>, env_id: usize, n: usize, seed: u64) -> Vec<u64> {
        let hz60 = self.hz60;
        let mut out = Vec::with_capacity(n);
        py.allow_threads(|| {
            let mut e = self.envs[env_id].lock().expect("env lock");
            let mut rng = ChaCha8Rng::seed_from_u64(env_seed(seed, env_id));
            e.reset(&mut rng);
            // 固定动作序列:由种子派生(与线程数无关)。
            let mut arng = ChaCha8Rng::seed_from_u64(seed ^ 0x5EED);
            for _ in 0..n {
                let a = [arng.random_range(-1.0..1.0), arng.random_range(-1.0..1.0)];
                let rin = e.control_step(&a, hz60);
                let o = arm::observe(&e.world, &e.arm, e.target, &e.last_action);
                let mut th = 0xcbf2_9ce4_8422_2325u64;
                for v in o.iter().chain(rin.iter()) {
                    for b in &v.to_ne_bytes() {
                        th ^= u64::from(*b);
                        th = th.wrapping_mul(0x100_0000_01b3);
                    }
                }
                if e.done {
                    e.reset(&mut arng);
                }
                out.push(th);
            }
        });
        out
    }

    /// 环境数。
    fn n_envs(&self) -> usize {
        self.n_envs
    }
}

/// 每环境的 reset 种子(φ 黄金比散布;固定函数,与线程数无关)。
fn env_seed(base: u64, i: usize) -> u64 {
    base ^ ((i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15))
}

/// Python 模块注册(maturin cdylib 入口)。
#[pymodule]
fn learn_env(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<VecEnv>()?;
    Ok(())
}
