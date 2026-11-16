//! REQ-012 E01b 测试(任务书第 2 节)。manifest 与 E1 相同(45 维);
//! intent_range 全 [-1,1];动作:第 k 步、环境 e、分量 i 取
//! 0.5·sin(0.1k + i + 0.7e)。物理相关期望达不到就停下报实测值。

use std::path::{Path, PathBuf};

use agent_env::{AgentEnv, EnvConfig, EnvError, VecEnv, episode_seed};
use policy::PolicyManifest;
use types::BodyDef;

const DOF: usize = 12;
const OBS_DIM: usize = 45;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("仓库根")
}

fn quad_text() -> String {
    std::fs::read_to_string(repo_root().join("assets/bodies/quad.body.toml")).expect("读 quad")
}

fn quad_def() -> BodyDef {
    BodyDef::from_toml_str(&quad_text()).expect("quad 合法")
}

fn manifest() -> PolicyManifest {
    let hash = quad_def().hash().hex();
    let terms = [
        "base_ang_vel",
        "projected_gravity",
        "joint_pos",
        "joint_vel",
        "last_action",
        "intent",
    ];
    let mut s = format!(
        "name = \"e01b\"\nbody_hash = \"{hash}\"\ncontrol_period_ticks = 2\naction_scale = 0.25\naction_clip = 1.0\n"
    );
    for t in terms {
        s.push_str(&format!("[[obs]]\nterm = \"{t}\"\n"));
    }
    PolicyManifest::from_toml_str(&s).expect("manifest 合法")
}

fn cfg(episode_controls: u32) -> EnvConfig {
    EnvConfig {
        body_toml: quad_text(),
        manifest: manifest(),
        episode_controls,
        settle_ticks: 120,
        fall_up_y_min: 0.5,
        intent_range: [[-1.0, 1.0]; 3],
    }
}

fn act(k: usize, e: usize, i: usize) -> f32 {
    0.5 * ((0.1 * k as f32) + i as f32 + 0.7 * e as f32).sin()
}

/// 第 k 步全部环境的拼接动作(n_envs × act_dim)。
fn actions(k: usize, n_envs: usize) -> Vec<f32> {
    let mut v = Vec::with_capacity(n_envs * DOF);
    for e in 0..n_envs {
        for i in 0..DOF {
            v.push(act(k, e, i));
        }
    }
    v
}

/// 第 k 步环境 e 的动作切片。
fn env_action(k: usize, e: usize) -> Vec<f32> {
    (0..DOF).map(|i| act(k, e, i)).collect()
}

fn bits(v: &[f32]) -> Vec<u32> {
    v.iter().map(|x| x.to_bits()).collect()
}

fn clip1(a: &[f32]) -> Vec<f32> {
    a.iter().map(|x| x.clamp(-1.0, 1.0)).collect()
}

fn seg(obs: &[f32], i: usize) -> &[f32] {
    &obs[i * OBS_DIM..(i + 1) * OBS_DIM]
}

/// V1 与线程数无关:4 环境、25 控制步,1 线程与 4 线程 60 步逐项/逐位相同。
#[test]
fn v1_thread_count_invariant() {
    let mut v1 = VecEnv::new(cfg(25), 4, 1).expect("1 线程");
    let mut v4 = VecEnv::new(cfg(25), 4, 4).expect("4 线程");
    let o1 = v1.reset(11).expect("reset1");
    let o4 = v4.reset(11).expect("reset4");
    assert_eq!(bits(&o1), bits(&o4), "reset obs");
    for k in 0..60 {
        let a = actions(k, 4);
        let s1 = v1.step(&a).expect("step1");
        let s4 = v4.step(&a).expect("step4");
        assert_eq!(bits(&s1.obs), bits(&s4.obs), "第 {k} 步 obs");
        assert_eq!(s1.terminated, s4.terminated, "第 {k} 步 terminated");
        assert_eq!(s1.truncated, s4.truncated, "第 {k} 步 truncated");
        for i in 0..4 {
            assert_eq!(
                bits(&s1.reward[i].joint_pos),
                bits(&s4.reward[i].joint_pos),
                "第 {k} 步 env{i} joint_pos"
            );
        }
    }
}

/// V2 与单环境一致:3 环境 2 线程 200 控制步,每个 i 另建 AgentEnv 喂同一串
/// 动作 40 步,每步 obs 与 VecEnv 第 i 段逐位相同(摔倒则比到摔倒前)。
#[test]
fn v2_matches_single() {
    let mut vec = VecEnv::new(cfg(200), 3, 2).expect("vec");
    vec.reset(11).expect("reset");
    let mut singles: Vec<AgentEnv> = (0..3)
        .map(|i| {
            let mut e = AgentEnv::new(cfg(200)).expect("单环境");
            e.reset(episode_seed(11, i as u64, 0)).expect("reset");
            e
        })
        .collect();
    let mut fell = [false; 3];
    let mut fall_step = [usize::MAX; 3];
    for k in 0..40 {
        let out = vec.step(&actions(k, 3)).expect("vec step");
        for i in 0..3 {
            if fell[i] {
                continue;
            }
            let s = singles[i].step(&env_action(k, i)).expect("单步");
            if s.terminated {
                fell[i] = true;
                fall_step[i] = k;
                continue;
            }
            assert_eq!(
                bits(&s.obs),
                bits(seg(&out.obs, i)),
                "第 {k} 步 env{i} obs 与单环境"
            );
        }
    }
    for i in 0..3 {
        if fell[i] {
            println!("v2: env{i} 在第 {} 步摔倒,比到摔倒前", fall_step[i]);
        }
    }
}

/// V3 到时那一步:2 环境、5 控制步。
#[test]
fn v3_auto_reset() {
    let seed = 11u64;
    let mut vec = VecEnv::new(cfg(5), 2, 2).expect("vec");
    vec.reset(seed).expect("reset");
    // 第 1–4 步:truncated 全 false。
    for k in 0..4 {
        let out = vec.step(&actions(k, 2)).expect("step");
        assert_eq!(out.truncated, vec![false; 2], "第 {} 步 truncated", k + 1);
    }
    // 第 5 步:truncated 全 true、terminated 全 false。
    let out5 = vec.step(&actions(4, 2)).expect("第 5 步");
    assert_eq!(out5.truncated, vec![true; 2], "第 5 步 truncated");
    assert_eq!(out5.terminated, vec![false; 2], "第 5 步 terminated");
    for i in 0..2 {
        assert_eq!(
            bits(&out5.reward[i].action),
            bits(&clip1(&env_action(4, i))),
            "第 5 步 env{i} action"
        );
        assert_eq!(
            bits(&out5.reward[i].prev_action),
            bits(&clip1(&env_action(3, i))),
            "第 5 步 env{i} prev_action"
        );
        let mut fresh = AgentEnv::new(cfg(5)).expect("新环境");
        let want = fresh.reset(episode_seed(seed, i as u64, 1)).expect("reset");
        assert_eq!(
            bits(seg(&out5.obs, i)),
            bits(&want),
            "第 5 步 env{i} obs = 回合 1 首帧"
        );
    }
    // 第 6 步:不报 NeedsReset,标志全 false,prev_action 全 0。
    let out6 = vec.step(&actions(5, 2)).expect("第 6 步");
    assert_eq!(out6.terminated, vec![false; 2]);
    assert_eq!(out6.truncated, vec![false; 2]);
    for i in 0..2 {
        assert_eq!(
            bits(&out6.reward[i].prev_action),
            bits(&[0.0; DOF]),
            "第 6 步 env{i} prev_action 全 0"
        );
    }
    // 第 7–9 步推进到第 10 步。
    for k in 6..9 {
        vec.step(&actions(k, 2)).expect("step");
    }
    // 第 10 步:truncated 又全 true,obs = 回合 2 首帧。
    let out10 = vec.step(&actions(9, 2)).expect("第 10 步");
    assert_eq!(out10.truncated, vec![true; 2], "第 10 步 truncated");
    for i in 0..2 {
        let mut fresh = AgentEnv::new(cfg(5)).expect("新环境");
        let want = fresh.reset(episode_seed(seed, i as u64, 2)).expect("reset");
        assert_eq!(
            bits(seg(&out10.obs, i)),
            bits(&want),
            "第 10 步 env{i} obs = 回合 2 首帧"
        );
    }
}

/// V4 种子派生。
#[test]
fn v4_seed_derivation() {
    let vals = [
        episode_seed(0, 0, 0),
        episode_seed(0, 1, 0),
        episode_seed(0, 0, 1),
        episode_seed(1, 0, 0),
    ];
    for a in 0..vals.len() {
        for b in (a + 1)..vals.len() {
            assert_ne!(vals[a], vals[b], "episode_seed 两两不同({a},{b})");
        }
    }
    let mut vec = VecEnv::new(cfg(5), 2, 2).expect("vec");
    vec.reset(11).expect("reset");
    let i0 = vec.intent(0).expect("intent0");
    let i1 = vec.intent(1).expect("intent1");
    assert_ne!(i0, i1, "reset(11) 后 env0 与 env1 intent 不同");
    // 走 5 步触发自动 reset(回合 1),env0 intent 与回合 0 不同。
    for k in 0..5 {
        vec.step(&actions(k, 2)).expect("step");
    }
    let i0_ep1 = vec.intent(0).expect("intent0 回合1");
    assert_ne!(i0, i0_ep1, "第 5 步后 env0 intent 与回合 0 不同");
}

/// V5 错误与边界。
#[test]
fn v5_errors_bounds() {
    assert!(matches!(
        VecEnv::new(cfg(5), 0, 1),
        Err(EnvError::Config(_))
    ));
    assert!(matches!(
        VecEnv::new(cfg(5), 2, 0),
        Err(EnvError::Config(_))
    ));
    // n_threads > n_envs 按 n_envs 用,能正常 reset/step。
    let mut v = VecEnv::new(cfg(5), 2, 8).expect("2 环境 8 线程");
    v.reset(11).expect("reset");
    v.step(&actions(0, 2)).expect("step");

    // 没 reset 就 step。
    let mut fresh = VecEnv::new(cfg(5), 2, 2).expect("vec");
    assert!(matches!(
        fresh.step(&actions(0, 2)),
        Err(EnvError::NeedsReset)
    ));

    fresh.reset(11).expect("reset");
    // 长度 n_envs×act_dim−1。
    let short = vec![0.0; 2 * DOF - 1];
    assert!(matches!(
        fresh.step(&short),
        Err(EnvError::ActionDim { got, want }) if got == 2 * DOF - 1 && want == 2 * DOF
    ));
    // 含 NaN。
    let mut nan = actions(0, 2);
    nan[0] = f32::NAN;
    assert!(matches!(fresh.step(&nan), Err(EnvError::NonFiniteAction)));
}

/// V6 reset 重来:走 7 步后再 reset(11),obs 与第一次 reset(11) 逐位相同。
#[test]
fn v6_reset_again() {
    let mut vec = VecEnv::new(cfg(5), 2, 2).expect("vec");
    let first = vec.reset(11).expect("第一次 reset");
    for k in 0..7 {
        vec.step(&actions(k, 2)).expect("step");
    }
    let again = vec.reset(11).expect("再 reset");
    assert_eq!(
        bits(&first),
        bits(&again),
        "reset 重来 obs 逐位相同(回合序号清 0)"
    );
}

/// V7 速度(release,只报数):16 环境,n_threads 1/4/8/16,500 步。
#[test]
#[ignore]
#[allow(clippy::disallowed_methods)]
fn v7_speed() {
    use std::time::Instant;

    for n_threads in [1usize, 4, 8, 16] {
        let t_new = Instant::now();
        let mut vec = VecEnv::new(cfg(1000), 16, n_threads).expect("vec");
        let new_ms = t_new.elapsed().as_secs_f64() * 1000.0;
        vec.reset(11).expect("reset");
        let t0 = Instant::now();
        for k in 0..500 {
            vec.step(&actions(k, 16)).expect("step");
        }
        let elapsed = t0.elapsed().as_secs_f64();
        let total_steps = 16.0 * 500.0;
        println!(
            "v7: n_threads={n_threads} 总 {:.1} 控制步/秒, new {new_ms:.1} ms",
            total_steps / elapsed
        );
    }
}
