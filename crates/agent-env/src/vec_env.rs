//! REQ-012 E01b:并行多实例 [`VecEnv`](任务书 §1)。
//!
//! N 个 [`AgentEnv`] 各用独立线程:`std::thread` + `std::sync::mpsc`。环境 i 归
//! 线程 `i % n_threads`,每个 AgentEnv 在它所属的工作线程里构造。主线程每次
//! `reset`/`step` 给每个工作线程发一条命令(带它名下各环境的动作切片),等全部
//! 回复,**按环境下标**拼结果;工作线程里各环境按下标升序依次执行。
//!
//! 自动 reset(到时那一步):环境 i 的 step 返回 terminated/truncated 时,工作
//! 线程立刻 `reset(episode_seed(base_seed, i, 回合序号+1))`,回合序号加 1;这一
//! 步返回结束那一步的标志与 reward,obs 第 i 段换成新回合的第一帧观测。
//!
//! 种子只依赖 seed 与 env_id([`episode_seed`]),与线程数无关。

use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::JoinHandle;

use crate::{AgentEnv, EnvConfig, EnvError, RewardInputs, splitmix64_next};

/// 一步的并行输出(任务书 §1)。
pub struct VecStep {
    /// 长度 n_envs × obs_dim,按环境下标顺序拼接。
    pub obs: Vec<f32>,
    /// 长度 n_envs。
    pub reward: Vec<RewardInputs>,
    pub terminated: Vec<bool>,
    pub truncated: Vec<bool>,
}

/// 环境 env_id 第 episode 个回合(从 0 起)用的种子。公开,测试要用。
pub fn episode_seed(base_seed: u64, env_id: u64, episode: u64) -> u64 {
    let mut s = base_seed
        ^ (env_id + 1).wrapping_mul(0x9E3779B97F4A7C15)
        ^ (episode + 1).wrapping_mul(0xD1B54A32D192ED03);
    splitmix64_next(&mut s)
}

/// 单环境一步的结果(工作线程 → 主线程)。
struct EnvStepResult {
    obs: Vec<f32>,
    reward: RewardInputs,
    terminated: bool,
    truncated: bool,
}

enum Command {
    Reset { base_seed: u64 },
    Step { actions: Vec<(usize, Vec<f32>)> },
    Intent { env_id: usize },
    Shutdown,
}

enum Response {
    Init(Result<(usize, usize), EnvError>),
    Reset(Vec<(usize, Result<Vec<f32>, EnvError>)>),
    Step(Vec<(usize, Result<EnvStepResult, EnvError>)>),
    Intent(Result<[f32; 3], EnvError>),
}

struct Slot {
    id: usize,
    env: AgentEnv,
    episode: u64,
}

/// 并行多实例环境(任务书 §1)。
pub struct VecEnv {
    n_envs: usize,
    n_threads: usize,
    obs_dim: usize,
    act_dim: usize,
    reset_done: bool,
    cmd_txs: Vec<Sender<Command>>,
    resp_rxs: Vec<Receiver<Response>>,
    handles: Vec<JoinHandle<()>>,
}

fn worker_main(
    cfg: EnvConfig,
    ids: Vec<usize>,
    cmd_rx: Receiver<Command>,
    resp_tx: Sender<Response>,
) {
    let mut slots: Vec<Slot> = Vec::new();
    for &id in &ids {
        match AgentEnv::new(cfg.clone()) {
            Ok(env) => slots.push(Slot {
                id,
                env,
                episode: 0,
            }),
            Err(e) => {
                let _ = resp_tx.send(Response::Init(Err(e)));
                return;
            }
        }
    }
    let obs_dim = slots[0].env.obs_dim();
    let act_dim = slots[0].env.act_dim();
    if resp_tx
        .send(Response::Init(Ok((obs_dim, act_dim))))
        .is_err()
    {
        return;
    }
    let mut base_seed = 0u64;
    for cmd in cmd_rx {
        match cmd {
            Command::Shutdown => break,
            Command::Reset { base_seed: seed } => {
                base_seed = seed;
                let mut results = Vec::new();
                for slot in &mut slots {
                    slot.episode = 0;
                    let s = episode_seed(seed, slot.id as u64, slot.episode);
                    results.push((slot.id, slot.env.reset(s)));
                }
                if resp_tx.send(Response::Reset(results)).is_err() {
                    break;
                }
            }
            Command::Step { actions } => {
                let mut results = Vec::new();
                for (id, action) in actions {
                    let slot = slots
                        .iter_mut()
                        .find(|s| s.id == id)
                        .expect("环境属于本线程");
                    match slot.env.step(&action) {
                        Ok(out) => {
                            let ended = out.terminated || out.truncated;
                            let mut r = EnvStepResult {
                                obs: out.obs,
                                reward: out.reward,
                                terminated: out.terminated,
                                truncated: out.truncated,
                            };
                            if ended {
                                slot.episode += 1;
                                let s = episode_seed(base_seed, id as u64, slot.episode);
                                match slot.env.reset(s) {
                                    Ok(o) => r.obs = o,
                                    Err(e) => {
                                        results.push((id, Err(e)));
                                        continue;
                                    }
                                }
                            }
                            results.push((id, Ok(r)));
                        }
                        Err(e) => results.push((id, Err(e))),
                    }
                }
                if resp_tx.send(Response::Step(results)).is_err() {
                    break;
                }
            }
            Command::Intent { env_id } => {
                let slot = slots
                    .iter()
                    .find(|s| s.id == env_id)
                    .expect("环境属于本线程");
                if resp_tx
                    .send(Response::Intent(Ok(slot.env.intent())))
                    .is_err()
                {
                    break;
                }
            }
        }
    }
}

impl VecEnv {
    /// n_envs ≥ 1、n_threads ≥ 1(否则 Config);n_threads > n_envs 时按 n_envs 用。
    /// 环境 i 归线程 i % n_threads,每个 AgentEnv 在所属工作线程里构造。
    pub fn new(cfg: EnvConfig, n_envs: usize, n_threads: usize) -> Result<Self, EnvError> {
        if n_envs == 0 {
            return Err(EnvError::Config("n_envs 须 ≥ 1".to_owned()));
        }
        if n_threads == 0 {
            return Err(EnvError::Config("n_threads 须 ≥ 1".to_owned()));
        }
        let n_threads = n_threads.min(n_envs);
        let mut owned: Vec<Vec<usize>> = vec![Vec::new(); n_threads];
        for i in 0..n_envs {
            owned[i % n_threads].push(i);
        }
        let mut cmd_txs = Vec::new();
        let mut resp_rxs = Vec::new();
        let mut handles = Vec::new();
        for ids in &owned {
            let (cmd_tx, cmd_rx) = mpsc::channel();
            let (resp_tx, resp_rx) = mpsc::channel();
            let cfg = cfg.clone();
            let ids = ids.clone();
            let handle = std::thread::spawn(move || worker_main(cfg, ids, cmd_rx, resp_tx));
            cmd_txs.push(cmd_tx);
            resp_rxs.push(resp_rx);
            handles.push(handle);
        }
        let mut dims = None;
        let mut first_err = None;
        for rx in &resp_rxs {
            let resp = rx
                .recv()
                .unwrap_or_else(|_| panic!("工作线程构造 panic(fail-stop)"));
            match resp {
                Response::Init(Ok(d)) => dims = Some(d),
                Response::Init(Err(e)) => {
                    if first_err.is_none() {
                        first_err = Some(e);
                    }
                }
                _ => panic!("意外的初始化响应"),
            }
        }
        if let Some(e) = first_err {
            for tx in &cmd_txs {
                let _ = tx.send(Command::Shutdown);
            }
            for h in handles {
                let _ = h.join();
            }
            return Err(e);
        }
        let (obs_dim, act_dim) = dims.expect("至少一个环境");
        Ok(Self {
            n_envs,
            n_threads,
            obs_dim,
            act_dim,
            reset_done: false,
            cmd_txs,
            resp_rxs,
            handles,
        })
    }

    pub fn n_envs(&self) -> usize {
        self.n_envs
    }

    pub fn obs_dim(&self) -> usize {
        self.obs_dim
    }

    pub fn act_dim(&self) -> usize {
        self.act_dim
    }

    /// 全部环境回到回合 0,返回拼接的观测。
    pub fn reset(&mut self, seed: u64) -> Result<Vec<f32>, EnvError> {
        for tx in &self.cmd_txs {
            tx.send(Command::Reset { base_seed: seed })
                .unwrap_or_else(|_| panic!("工作线程已退出(fail-stop)"));
        }
        let mut obs_segs: Vec<Option<Vec<f32>>> = vec![None; self.n_envs];
        let mut first_err: Option<(usize, EnvError)> = None;
        for rx in &self.resp_rxs {
            let resp = rx
                .recv()
                .unwrap_or_else(|_| panic!("工作线程 panic(fail-stop)"));
            let Response::Reset(results) = resp else {
                panic!("意外的 reset 响应");
            };
            for (id, res) in results {
                match res {
                    Ok(obs) => obs_segs[id] = Some(obs),
                    Err(e) => note_err(&mut first_err, id, e),
                }
            }
        }
        if let Some((_, e)) = first_err {
            return Err(e);
        }
        let mut obs = Vec::with_capacity(self.n_envs * self.obs_dim);
        for seg in obs_segs {
            obs.extend(seg.expect("每个环境都有 obs"));
        }
        self.reset_done = true;
        Ok(obs)
    }

    /// actions 长度必须是 n_envs × act_dim(否则 ActionDim)。
    pub fn step(&mut self, actions: &[f32]) -> Result<VecStep, EnvError> {
        if !self.reset_done {
            return Err(EnvError::NeedsReset);
        }
        let want = self.n_envs * self.act_dim;
        if actions.len() != want {
            return Err(EnvError::ActionDim {
                got: actions.len(),
                want,
            });
        }
        for w in 0..self.n_threads {
            let mut wa = Vec::new();
            for i in (w..self.n_envs).step_by(self.n_threads) {
                wa.push((
                    i,
                    actions[i * self.act_dim..(i + 1) * self.act_dim].to_vec(),
                ));
            }
            self.cmd_txs[w]
                .send(Command::Step { actions: wa })
                .unwrap_or_else(|_| panic!("工作线程已退出(fail-stop)"));
        }
        let mut obs_segs: Vec<Option<Vec<f32>>> = vec![None; self.n_envs];
        let mut rewards: Vec<Option<RewardInputs>> = (0..self.n_envs).map(|_| None).collect();
        let mut terminated = vec![false; self.n_envs];
        let mut truncated = vec![false; self.n_envs];
        let mut first_err: Option<(usize, EnvError)> = None;
        for rx in &self.resp_rxs {
            let resp = rx
                .recv()
                .unwrap_or_else(|_| panic!("工作线程 panic(fail-stop)"));
            let Response::Step(results) = resp else {
                panic!("意外的 step 响应");
            };
            for (id, res) in results {
                match res {
                    Ok(r) => {
                        obs_segs[id] = Some(r.obs);
                        rewards[id] = Some(r.reward);
                        terminated[id] = r.terminated;
                        truncated[id] = r.truncated;
                    }
                    Err(e) => note_err(&mut first_err, id, e),
                }
            }
        }
        if let Some((_, e)) = first_err {
            return Err(e);
        }
        let mut obs = Vec::with_capacity(self.n_envs * self.obs_dim);
        for seg in obs_segs {
            obs.extend(seg.expect("每个环境都有 obs"));
        }
        let reward = rewards
            .into_iter()
            .map(|r| r.expect("每个环境都有 reward"))
            .collect();
        Ok(VecStep {
            obs,
            reward,
            terminated,
            truncated,
        })
    }

    /// 环境 env 当前的 intent(测试用)。
    pub fn intent(&mut self, env: usize) -> Result<[f32; 3], EnvError> {
        if env >= self.n_envs {
            return Err(EnvError::Config(format!("环境下标越界:{env}")));
        }
        let w = env % self.n_threads;
        self.cmd_txs[w]
            .send(Command::Intent { env_id: env })
            .unwrap_or_else(|_| panic!("工作线程已退出(fail-stop)"));
        let resp = self.resp_rxs[w]
            .recv()
            .unwrap_or_else(|_| panic!("工作线程 panic(fail-stop)"));
        let Response::Intent(r) = resp else {
            panic!("意外的 intent 响应");
        };
        r
    }
}

/// 记录下标最小的错误。
fn note_err(first: &mut Option<(usize, EnvError)>, id: usize, e: EnvError) {
    match first {
        Some((cur, _)) if *cur <= id => {}
        _ => *first = Some((id, e)),
    }
}

impl Drop for VecEnv {
    fn drop(&mut self) {
        for tx in &self.cmd_txs {
            let _ = tx.send(Command::Shutdown);
        }
        for h in self.handles.drain(..) {
            let _ = h.join();
        }
    }
}
