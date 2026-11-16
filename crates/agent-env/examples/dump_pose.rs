//! REQ-012 E02a §3:姿态转储。
//!
//! `cargo run -j 10 -p agent-env --example dump_pose -- <body.toml> <out.json>`
//! 用 E1 的 manifest 与固定动作序列 `0.5·sin(0.1k + i)`,intent_range 全 0,
//! reset(0),走 60 步;第 0(reset 后)、20、40、60 步各转储一帧。另转储
//! 全 0 动作 300 步后的 stand_root_height 与 ground_top_y。
//! 四元数分量顺序为 glam 的 x y z w(JSON 顶层 quat_order 标明)。

use agent_env::{AgentEnv, EnvConfig};
use policy::PolicyManifest;
use types::BodyDef;

fn manifest(body_hash: &str) -> PolicyManifest {
    let terms = [
        "base_ang_vel",
        "projected_gravity",
        "joint_pos",
        "joint_vel",
        "last_action",
        "intent",
    ];
    let mut s = format!(
        "name = \"dump\"\nbody_hash = \"{body_hash}\"\ncontrol_period_ticks = 2\naction_scale = 0.25\naction_clip = 1.0\n"
    );
    for t in terms {
        s.push_str(&format!("[[obs]]\nterm = \"{t}\"\n"));
    }
    PolicyManifest::from_toml_str(&s).expect("manifest 合法")
}

fn cfg(body_toml: String, m: PolicyManifest) -> EnvConfig {
    EnvConfig {
        body_toml,
        manifest: m,
        episode_controls: 1000,
        settle_ticks: 120,
        // 转储要跑满 60 步:不因地面倾倒提前终止(姿态同构核对与是否摔倒无关)。
        fall_up_y_min: -1.0,
        intent_range: [[0.0, 0.0]; 3],
    }
}

fn action_seq(k: usize, dof: usize) -> Vec<f32> {
    (0..dof)
        .map(|i| 0.5 * ((0.1 * k as f32) + i as f32).sin())
        .collect()
}

fn farr(v: &[f32]) -> String {
    let parts: Vec<String> = v.iter().map(|x| format!("{x}")).collect();
    format!("[{}]", parts.join(", "))
}

fn arr3(v: [f32; 3]) -> String {
    format!("[{}, {}, {}]", v[0], v[1], v[2])
}

fn quat(q: types::glam::Quat) -> String {
    format!("[{}, {}, {}, {}]", q.x, q.y, q.z, q.w)
}

/// 一帧:根位姿 + 关节角 + 全部连杆世界位姿(身体定义顺序)。
fn frame(env: &AgentEnv, names: &[String], step: usize) -> String {
    let links = env.link_states();
    let root = links[0];
    let jp = env.joint_positions();
    let link_objs: Vec<String> = names
        .iter()
        .zip(&links)
        .map(|(name, s)| {
            format!(
                "{{\"name\": \"{name}\", \"pos\": {}, \"quat\": {}}}",
                arr3([s.translation.x, s.translation.y, s.translation.z]),
                quat(s.rotation)
            )
        })
        .collect();
    format!(
        "{{\"step\": {step}, \"root_pos\": {}, \"root_quat\": {}, \"joint_pos\": {}, \"links\": [{}]}}",
        arr3([root.translation.x, root.translation.y, root.translation.z]),
        quat(root.rotation),
        farr(&jp),
        link_objs.join(", ")
    )
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() != 2 {
        eprintln!("用法:dump_pose <body.toml> <out.json>");
        std::process::exit(2);
    }
    let body_toml = std::fs::read_to_string(&args[0]).expect("读身体");
    let def = BodyDef::from_toml_str(&body_toml).expect("身体合法");
    let names: Vec<String> = def.links.iter().map(|l| l.name.clone()).collect();
    let dof = def.dof_count();
    let m = manifest(&def.hash().hex());

    let mut env = AgentEnv::new(cfg(body_toml.clone(), m.clone())).expect("建环境");
    env.reset(0).expect("reset");
    let mut frames = vec![frame(&env, &names, 0)];
    for k in 0..60 {
        env.step(&action_seq(k, dof)).expect("step");
        let step = k + 1;
        if step == 20 || step == 40 || step == 60 {
            frames.push(frame(&env, &names, step));
        }
    }

    // 站立高度:全 0 动作(默认站姿)走 300 步。
    let mut stand = AgentEnv::new(cfg(body_toml, m)).expect("站立环境");
    stand.reset(0).expect("reset");
    let zero = vec![0.0; dof];
    let mut last_height = 0.0;
    for _ in 0..300 {
        let out = stand.step(&zero).expect("step");
        last_height = out.reward.root_height;
    }

    let json = format!(
        "{{\n  \"quat_order\": \"xyzw\",\n  \"ground_top_y\": {},\n  \"stand_root_height\": {last_height},\n  \"frames\": [\n    {}\n  ]\n}}\n",
        env.ground_top_y(),
        frames.join(",\n    ")
    );
    std::fs::write(&args[1], json).expect("写 out.json");
    println!("姿态转储:{}", args[1]);
}
