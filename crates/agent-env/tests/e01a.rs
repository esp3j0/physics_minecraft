//! REQ-012 E01a 测试(任务书第 3 节)。quad 身体;manifest:control_period_ticks=2、
//! action_scale=0.25、action_clip=1.0,obs = base_ang_vel、projected_gravity、
//! joint_pos、joint_vel、last_action、intent(scale 全 1,共 45 维)。
//! 物理稳态类的数(0.05/0.35/0.9/高度范围)照抄任务书,不改。

use std::path::{Path, PathBuf};

use agent_env::{AgentEnv, EnvConfig, EnvError};
use messaging::InputSource;
use policy::{PolicyCodec, PolicyManifest};
use shard::SchedulerConfig;
use sim_api::log::CommandLog;
use sim_api::replay::replay_world_with_bodies;
use sim_api::session::{Session, SessionConfig};
use types::{BodyDef, EntityState};
use voxel::SimpleRegistry;

const DOF: usize = 12;
const OBS_DIM: usize = 45;
// obs 段偏移:base_ang_vel[0..3] projected_gravity[3..6] joint_pos[6..18]
// joint_vel[18..30] last_action[30..42] intent[42..45]。
const GRAV_OFF: usize = 3;
const JPOS_OFF: usize = 6;
const LAST_OFF: usize = 30;
const INTENT_OFF: usize = 42;

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

fn defaults(def: &BodyDef) -> Vec<f32> {
    def.dof_joints()
        .map(|j| j.kind.params().default_angle.unwrap_or(0.0))
        .collect()
}

/// 任务书第 3 节的 manifest(scale 全 1,省略 scale 字段即默认 1)。
fn manifest_text(body_hash: &str, extra: &str) -> String {
    let terms = [
        "base_ang_vel",
        "projected_gravity",
        "joint_pos",
        "joint_vel",
        "last_action",
        "intent",
    ];
    let mut s = format!(
        "name = \"e01a\"\nbody_hash = \"{body_hash}\"\ncontrol_period_ticks = 2\naction_scale = 0.25\naction_clip = 1.0\n{extra}"
    );
    for t in terms {
        s.push_str(&format!("[[obs]]\nterm = \"{t}\"\n"));
    }
    s
}

fn manifest() -> PolicyManifest {
    let hash = quad_def().hash().hex();
    PolicyManifest::from_toml_str(&manifest_text(&hash, "")).expect("manifest 合法")
}

fn cfg(episode_controls: u32, fall_up_y_min: f32, intent_range: [[f32; 2]; 3]) -> EnvConfig {
    EnvConfig {
        body_toml: quad_text(),
        manifest: manifest(),
        episode_controls,
        settle_ticks: 120,
        fall_up_y_min,
        intent_range,
    }
}

fn zero_range() -> [[f32; 2]; 3] {
    [[0.0, 0.0]; 3]
}

fn make_env(episode_controls: u32) -> AgentEnv {
    AgentEnv::new(cfg(episode_controls, 0.5, zero_range())).expect("建环境")
}

/// 固定动作序列(输入,不是期望):第 k 步第 i 分量 0.5·sin(0.1k + i)。
fn action_seq(k: usize) -> Vec<f32> {
    (0..DOF)
        .map(|i| 0.5 * ((0.1 * k as f32) + i as f32).sin())
        .collect()
}

fn clip1(a: &[f32]) -> Vec<f32> {
    a.iter().map(|x| x.clamp(-1.0, 1.0)).collect()
}

fn bits(v: &[f32]) -> Vec<u32> {
    v.iter().map(|x| x.to_bits()).collect()
}

fn assert_obs_eq(got: &[f32], want: &[f32], tag: &str) {
    assert_eq!(got.len(), want.len(), "{tag} obs 长度");
    assert_eq!(bits(got), bits(want), "{tag} obs 逐位不同");
}

fn state_bits(s: &EntityState) -> Vec<u32> {
    let mut v = Vec::new();
    for a in [s.translation, s.linvel, s.angvel] {
        v.extend([a.x.to_bits(), a.y.to_bits(), a.z.to_bits()]);
    }
    v.extend([
        s.rotation.x.to_bits(),
        s.rotation.y.to_bits(),
        s.rotation.z.to_bits(),
        s.rotation.w.to_bits(),
    ]);
    v
}

fn assert_states_eq(got: &[EntityState], want: &[EntityState], tag: &str) {
    assert_eq!(got.len(), want.len(), "{tag} 连杆数");
    for (i, (g, w)) in got.iter().zip(want).enumerate() {
        assert_eq!(state_bits(g), state_bits(w), "{tag} 连杆 {i} 逐位不同");
    }
}

/// 与 run_quad / agent-env 同口径的会话配置(独立驱动用)。
fn session_cfg() -> SessionConfig {
    SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            router: shard::router::RouterConfig {
                inbox_capacity: 512,
                ..shard::router::RouterConfig::default()
            },
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    }
}

fn restore(
    initial: &sim_api::snapshot::WorldSnapshot,
    bodies: &physics_sim::BodyRegistry,
) -> Session {
    let bodies = bodies.clone();
    replay_world_with_bodies(
        initial,
        &CommandLog::new(),
        session_cfg(),
        || Box::new(SimpleRegistry::builtin()),
        || bodies.clone(),
    )
    .expect("恢复初态快照")
}

fn link_states(session: &Session, entity: types::EntityId) -> Vec<EntityState> {
    session
        .agent_states()
        .into_iter()
        .find(|(id, _, _)| *id == entity)
        .map(|(_, _, s)| s)
        .unwrap_or_default()
}

/// E1 AC-E1 同一物理:独立驱动逐步逐位相同 + 日志回放末态逐位相同。
#[test]
fn e1_same_physics() {
    let mut env = make_env(200);
    env.reset(7).expect("reset");
    let entity = env.entity();
    let start_root = env.link_states()[0].translation;

    // (b) 独立驱动:与环境同步,每步 decode → send → tick 两拍,连杆逐位相同。
    let def = quad_def();
    let codec = PolicyCodec::new(manifest(), &def).expect("独立 codec");
    let mut driven = restore(env.initial_snapshot(), env.bodies());
    for k in 0..150 {
        env.step(&action_seq(k)).expect("env step");
        let act = codec.decode(&action_seq(k)).expect("decode");
        driven
            .send_agent_joint_targets_to(InputSource::SCHEDULER, entity, act.targets)
            .expect("发目标");
        driven.tick();
        driven.tick();
        assert_states_eq(
            &link_states(&driven, entity),
            &env.link_states(),
            &format!("E1(b) 第 {k} 步"),
        );
    }
    driven.shutdown();

    let end_root = env.link_states()[0].translation;
    assert_ne!(start_root, end_root, "自检:动作应使根位置改变");

    // (a) 日志回放:初态快照 + 命令日志 → 末态连杆逐位相同。
    let bodies = env.bodies().clone();
    let mut replayed = replay_world_with_bodies(
        env.initial_snapshot(),
        env.command_log(),
        session_cfg(),
        || Box::new(SimpleRegistry::builtin()),
        || bodies.clone(),
    )
    .expect("回放");
    for _ in 0..(150 * 2) {
        replayed.tick();
    }
    assert_states_eq(
        &link_states(&replayed, entity),
        &env.link_states(),
        "E1(a) 回放",
    );
    replayed.shutdown();
}

/// E2 确定性:两环境 reset(7) 同序列 100 步 obs 逐位相同;reset(8) intent 不同。
#[test]
fn e2_deterministic() {
    let range = [[-1.0, 1.0]; 3];
    let mut a = AgentEnv::new(cfg(100, 0.5, range)).expect("环境 a");
    let mut b = AgentEnv::new(cfg(100, 0.5, range)).expect("环境 b");
    let oa = a.reset(7).expect("reset a");
    let ob = b.reset(7).expect("reset b");
    assert_obs_eq(&oa, &ob, "E2 reset");
    for k in 0..100 {
        let sa = a.step(&action_seq(k)).expect("step a");
        let sb = b.step(&action_seq(k)).expect("step b");
        assert_obs_eq(&sa.obs, &sb.obs, &format!("E2 第 {k} 步"));
    }
    let i7 = a.reset(7).expect("reset7");
    let _ = i7;
    let intent7 = a.intent();
    a.reset(8).expect("reset8");
    let intent8 = a.intent();
    assert_ne!(intent7, intent8, "reset(8) 与 reset(7) 的 intent 应不同");
}

/// E3 reset 回到初态:走 50 步后 reset(7) 与全新环境 reset(7) obs 逐位相同。
#[test]
fn e3_reset_restores() {
    let mut env = make_env(200);
    env.reset(7).expect("reset");
    for k in 0..50 {
        env.step(&action_seq(k)).expect("step");
    }
    let obs_reset = env.reset(7).expect("再 reset");
    let mut fresh = make_env(200);
    let obs_fresh = fresh.reset(7).expect("新环境 reset");
    assert_obs_eq(&obs_reset, &obs_fresh, "E3 reset obs");
    for k in 0..50 {
        let s1 = env.step(&action_seq(k)).expect("env step");
        let s2 = fresh.step(&action_seq(k)).expect("fresh step");
        assert_obs_eq(&s1.obs, &s2.obs, &format!("E3 第 {k} 步"));
    }
}

/// E4 观测的可手算部分(固定 intent)。
#[test]
fn e4_observation_handcalc() {
    let range = [[0.5, 0.5], [-0.25, -0.25], [1.0, 1.0]];
    let mut env = AgentEnv::new(cfg(200, 0.5, range)).expect("环境");
    let obs = env.reset(7).expect("reset");
    assert_eq!(obs.len(), OBS_DIM);
    for &v in &obs[LAST_OFF..LAST_OFF + DOF] {
        assert_eq!(v.to_bits(), 0.0f32.to_bits(), "last_action 段应全 0");
    }
    assert_eq!(
        bits(&obs[INTENT_OFF..INTENT_OFF + 3]),
        bits(&[0.5, -0.25, 1.0]),
        "intent 段"
    );
    let grav = [0.0f32, -1.0, 0.0];
    for (i, &g) in grav.iter().enumerate() {
        assert!(
            (obs[GRAV_OFF + i] - g).abs() < 0.05,
            "projected_gravity[{i}] = {} 期望 ≈ {}",
            obs[GRAV_OFF + i],
            g
        );
    }
    for i in 0..DOF {
        assert!(
            obs[JPOS_OFF + i].abs() < 0.35,
            "joint_pos[{i}] = {} 绝对值应 < 0.35",
            obs[JPOS_OFF + i]
        );
    }
    // 走一步动作 a:obs 的 last_action 段 == a 裁剪到 [-1,1]。
    let a = action_seq(0);
    let out = env.step(&a).expect("step");
    assert_eq!(
        bits(&out.obs[LAST_OFF..LAST_OFF + DOF]),
        bits(&clip1(&a)),
        "step 后 last_action 段"
    );
}

/// E5 结束条件。
#[test]
fn e5_termination() {
    // truncate:episode_controls=10,全 0 动作。
    let mut env = make_env(10);
    env.reset(7).expect("reset");
    let zero = vec![0.0; DOF];
    for k in 0..9 {
        let out = env.step(&zero).expect("step");
        assert!(!out.truncated, "第 {} 步不应 truncated", k + 1);
        assert!(!out.terminated, "第 {} 步不应 terminated", k + 1);
    }
    let out10 = env.step(&zero).expect("第 10 步");
    assert!(out10.truncated, "第 10 步应 truncated");
    assert!(!out10.terminated, "第 10 步不应 terminated");
    assert!(matches!(env.step(&zero), Err(EnvError::NeedsReset)));

    // 摔倒:fall_up_y_min=1.1(不可能满足)。
    let mut fall = AgentEnv::new(cfg(10, 1.1, zero_range())).expect("环境");
    fall.reset(7).expect("reset");
    let f1 = fall.step(&zero).expect("第一步");
    assert!(f1.terminated, "第一步应 terminated");
    assert!(!f1.truncated, "第一步不应 truncated");
    assert!(matches!(fall.step(&zero), Err(EnvError::NeedsReset)));

    // 默认 0.5、全 0 动作 100 步:从不 terminated,每步 up_y > 0.9。
    let mut stand = make_env(100);
    stand.reset(7).expect("reset");
    for k in 0..100 {
        let out = stand.step(&zero).expect("step");
        assert!(!out.terminated, "第 {k} 步不应 terminated");
        assert!(
            out.reward.up_y > 0.9,
            "第 {k} 步 up_y = {}",
            out.reward.up_y
        );
    }

    // 没 reset 就 step。
    let mut fresh = make_env(10);
    assert!(matches!(fresh.step(&zero), Err(EnvError::NeedsReset)));
}

/// E6 错误。
#[test]
fn e6_errors() {
    let mut env = make_env(10);
    env.reset(7).expect("reset");
    let short = vec![0.0; 11];
    assert!(matches!(
        env.step(&short),
        Err(EnvError::ActionDim { got: 11, want: 12 })
    ));
    let mut nan = vec![0.0; DOF];
    nan[0] = f32::NAN;
    assert!(matches!(env.step(&nan), Err(EnvError::NonFiniteAction)));

    // manifest 带 normalization → Config。
    let hash = quad_def().hash().hex();
    let norm_text = manifest_text(&hash, "[normalization]\nmean = [0.0]\nstd = [1.0]\n");
    let mut c = cfg(10, 0.5, zero_range());
    c.manifest = PolicyManifest::from_toml_str(&norm_text).expect("manifest 合法");
    assert!(matches!(AgentEnv::new(c), Err(EnvError::Config(_))));

    // body_hash 全 0 → Policy(..)。
    let zero_hash = "0".repeat(64);
    let mut c = cfg(10, 0.5, zero_range());
    c.manifest = PolicyManifest::from_toml_str(&manifest_text(&zero_hash, "")).expect("manifest");
    assert!(matches!(AgentEnv::new(c), Err(EnvError::Policy(_))));

    // episode_controls=0 → Config。
    assert!(matches!(
        AgentEnv::new(cfg(0, 0.5, zero_range())),
        Err(EnvError::Config(_))
    ));

    // intent_range lo > hi → Config。
    assert!(matches!(
        AgentEnv::new(cfg(10, 0.5, [[0.5, -0.5], [0.0, 0.0], [0.0, 0.0]])),
        Err(EnvError::Config(_))
    ));
}

/// E7 RewardInputs。
#[test]
fn e7_reward_inputs() {
    let def = quad_def();
    let def_pose = defaults(&def);
    let mut env = make_env(200);
    env.reset(7).expect("reset");
    let zero = vec![0.0; DOF];
    let s1 = env.step(&zero).expect("step1");
    assert_eq!(bits(&s1.reward.action), bits(&zero), "step1 action 全 0");
    assert_eq!(bits(&s1.reward.prev_action), bits(&zero), "step1 prev 全 0");

    let a = action_seq(0);
    let s2 = env.step(&a).expect("step2");
    assert_eq!(
        bits(&s2.reward.action),
        bits(&clip1(&a)),
        "step2 action=clip(a)"
    );
    assert_eq!(bits(&s2.reward.prev_action), bits(&zero), "step2 prev 全 0");

    let b = action_seq(1);
    let s3 = env.step(&b).expect("step3");
    assert_eq!(
        bits(&s3.reward.prev_action),
        bits(&clip1(&a)),
        "step3 prev=clip(a)"
    );
    assert_eq!(s3.reward.joint_pos.len(), DOF);
    for (i, (&jp, &dp)) in s3.reward.joint_pos.iter().zip(&def_pose).enumerate() {
        let lhs = jp - dp;
        assert_eq!(
            lhs.to_bits(),
            s3.obs[JPOS_OFF + i].to_bits(),
            "joint_pos[{i}] − 默认角 应逐位等于 obs joint_pos 段"
        );
    }
    assert!(
        s3.reward.root_height > 0.1 && s3.reward.root_height < 0.5,
        "root_height = {} 应在 (0.1, 0.5)",
        s3.reward.root_height
    );
}

/// E8 采样:seed=0 第一个输出参考值;intent_range 全 [0,0] 恒为 0。
#[test]
fn e8_sampling() {
    // 公开参考值:SplitMix64 seed=0 第一个输出 0xE220A8397B1DCDAF。
    // intent_range [0,1] 时 intent[0] = (first >> 40) as f32 / 16777216.0。
    let first: u64 = 0xE220A8397B1DCDAF;
    let u0 = (first >> 40) as f32 / 16777216.0;
    let mut env = AgentEnv::new(cfg(10, 0.5, [[0.0, 1.0]; 3])).expect("环境");
    env.reset(0).expect("reset0");
    assert_eq!(
        env.intent()[0].to_bits(),
        u0.to_bits(),
        "seed=0 intent[0] 应对应参考输出"
    );

    let mut z = AgentEnv::new(cfg(10, 0.5, zero_range())).expect("环境");
    z.reset(123).expect("reset");
    assert_eq!(z.intent(), [0.0, 0.0, 0.0], "全 [0,0] intent 恒为 0");
}

/// E9 速度(release 跑一次报数,不设阈值)。
#[test]
#[ignore]
#[allow(clippy::disallowed_methods)]
fn e9_speed() {
    use std::time::Instant;

    let mut env = make_env(2000);
    env.reset(7).expect("reset");
    let zero = vec![0.0; DOF];
    let t0 = Instant::now();
    for _ in 0..2000 {
        env.step(&zero).expect("step");
    }
    let elapsed = t0.elapsed().as_secs_f64();
    let steps_per_sec = 2000.0 / elapsed;
    let ms_per_step = elapsed * 1000.0 / 2000.0;
    println!("e9: {steps_per_sec:.1} 控制步/秒, {ms_per_step:.3} ms/步(2000 步,全 0 动作)");

    let mut reset_us = Vec::with_capacity(20);
    for _ in 0..20 {
        let t = Instant::now();
        env.reset(7).expect("reset");
        reset_us.push(t.elapsed().as_secs_f64() * 1e6);
    }
    let avg_reset_ms = reset_us.iter().sum::<f64>() / 20.0 / 1000.0;
    println!("e9: reset 平均 {avg_reset_ms:.3} ms(20 次)");
}
