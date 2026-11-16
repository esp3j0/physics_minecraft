//! REQ-012 E02b-2 §4:`cargo xtask check-policy <策略目录> --body <body.toml>
//! [--probe <probe.json>]` —— 证明 Python 训练导出的策略包能在游戏推理路径上
//! 加载,并(probe)给出与 Python 侧逐位一致的动作。
//!
//! 1. `policy::Policy::load(dir, &body)`;失败打印原因、非零退出。
//! 2. 有 `--probe` 时,对每条:建 `AgentEnv`(manifest = 策略 manifest 去掉
//!    normalization 和 obs_clip;intent_range 全 0;episode_controls = 600),
//!    `reset(agent_env::episode_seed(seed, 0, 0))`:环境观测与 probe.obs 逐位相同;
//!    `env.snapshot()` → `policy.act()` 的 raw 与 probe.action 每分量差 < 1e-4。
//! 3. 全过打印 `OK` 和各条最大差。

use std::path::{Path, PathBuf};

use agent_env::{AgentEnv, EnvConfig, episode_seed};
use policy::{Policy, PolicyManifest};
use types::BodyDef;

/// 解析后的参数。
struct Args {
    dir: PathBuf,
    body: PathBuf,
    probe: Option<PathBuf>,
}

fn parse_args(args: &[String]) -> Result<Args, String> {
    let mut dir: Option<PathBuf> = None;
    let mut body: Option<PathBuf> = None;
    let mut probe: Option<PathBuf> = None;
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        let mut value = || -> Option<String> { iter.next().cloned() };
        match arg.as_str() {
            "--body" => {
                if body.is_some() {
                    return Err("--body 重复给出".to_owned());
                }
                body = Some(
                    value()
                        .map(PathBuf::from)
                        .ok_or_else(|| "--body 缺少参数值".to_owned())?,
                );
            }
            "--probe" => {
                if probe.is_some() {
                    return Err("--probe 重复给出".to_owned());
                }
                probe = Some(
                    value()
                        .map(PathBuf::from)
                        .ok_or_else(|| "--probe 缺少参数值".to_owned())?,
                );
            }
            other => {
                if other.starts_with("--") {
                    return Err(format!("未知参数 `{other}`"));
                }
                if dir.is_some() {
                    return Err("只接受一个策略目录位置参数".to_owned());
                }
                dir = Some(PathBuf::from(other));
            }
        }
    }
    let dir = dir.ok_or_else(|| {
        "用法:cargo xtask check-policy <策略目录> --body <body.toml> [--probe <probe.json>]"
            .to_owned()
    })?;
    let body = body.ok_or_else(|| "缺少 --body <body.toml>".to_owned())?;
    Ok(Args { dir, body, probe })
}

/// 加载策略包(测试用:成功 = 合法目录;失败消息含错误 Display)。
pub fn load_policy(dir: &Path, body: &BodyDef) -> Result<Policy, String> {
    Policy::load(dir, body).map_err(|e| e.to_string())
}

/// 去掉 normalization 与 obs_clip 的 manifest(训练环境输出未归一化观测)。
fn env_manifest(m: &PolicyManifest) -> PolicyManifest {
    let mut m = m.clone();
    m.normalization = None;
    m.obs_clip = None;
    m
}

/// 跑一条 probe:返回该条动作最大差;任何不符返回 Err(打印用)。
fn run_probe(
    policy: &mut Policy,
    body_toml: &str,
    entry: &serde_json::Value,
) -> Result<f32, String> {
    let seed = entry
        .get("seed")
        .and_then(serde_json::Value::as_u64)
        .ok_or_else(|| "probe 条缺 seed".to_owned())?;
    let probe_obs = entry
        .get("obs")
        .and_then(serde_json::Value::as_array)
        .ok_or_else(|| "probe 条缺 obs 数组".to_owned())?;
    let probe_act = entry
        .get("action")
        .and_then(serde_json::Value::as_array)
        .ok_or_else(|| "probe 条缺 action 数组".to_owned())?;

    let cfg = EnvConfig {
        body_toml: body_toml.to_owned(),
        manifest: env_manifest(policy.manifest()),
        episode_controls: 600,
        settle_ticks: 120,
        fall_up_y_min: 0.5,
        intent_range: [[0.0, 0.0]; 3],
    };
    let mut env = AgentEnv::new(cfg).map_err(|e| format!("建 AgentEnv 失败:{e}"))?;
    let obs = env
        .reset(episode_seed(seed, 0, 0))
        .map_err(|e| format!("reset 失败:{e}"))?;

    if obs.len() != probe_obs.len() {
        return Err(format!(
            "obs 长度 {} ≠ probe {}",
            obs.len(),
            probe_obs.len()
        ));
    }
    for (i, (got, want)) in obs.iter().zip(probe_obs).enumerate() {
        let want = want.as_f64().ok_or_else(|| "probe.obs 含非数".to_owned())? as f32;
        if got.to_bits() != want.to_bits() {
            return Err(format!(
                "seed {seed} obs 第 {i} 位不一致:环境 {got} ≠ probe {want}(取的不是同一状态)"
            ));
        }
    }

    let snapshot = env.snapshot().map_err(|e| format!("snapshot 失败:{e}"))?;
    let action = policy.act(&snapshot).map_err(|e| format!("act 失败:{e}"))?;
    if action.raw.len() != probe_act.len() {
        return Err(format!(
            "action 长度 {} ≠ probe {}",
            action.raw.len(),
            probe_act.len()
        ));
    }
    let mut max_diff = 0.0f32;
    for (i, (got, want)) in action.raw.iter().zip(probe_act).enumerate() {
        let want = want
            .as_f64()
            .ok_or_else(|| "probe.action 含非数".to_owned())?;
        let diff = (f64::from(*got) - want).abs();
        if diff >= 1e-4 {
            return Err(format!(
                "seed {seed} action 第 {i} 分量差 {diff} ≥ 1e-4:环境 {got} ≠ probe {want}"
            ));
        }
        max_diff = max_diff.max(diff as f32);
    }
    Ok(max_diff)
}

/// 子命令入口,返回进程退出码。
pub fn cmd_check_policy(args: &[String]) -> i32 {
    let parsed = match parse_args(args) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            return 2;
        }
    };
    let body_toml = match std::fs::read_to_string(&parsed.body) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("读 {} 失败:{e}", parsed.body.display());
            return 2;
        }
    };
    let body = match BodyDef::from_toml_str(&body_toml) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("身体定义错误:{e}");
            return 2;
        }
    };
    let mut policy = match load_policy(&parsed.dir, &body) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("策略加载失败:{e}");
            return 1;
        }
    };
    println!(
        "策略加载成功:dof {} obs_dim {}",
        policy.dof(),
        policy.obs_dim()
    );

    let Some(probe_path) = parsed.probe else {
        println!("OK(无 probe)");
        return 0;
    };
    let text = match std::fs::read_to_string(&probe_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("读 {} 失败:{e}", probe_path.display());
            return 2;
        }
    };
    let entries: serde_json::Value = match serde_json::from_str(&text) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("probe.json 解析失败:{e}");
            return 2;
        }
    };
    let Some(arr) = entries.as_array() else {
        eprintln!("probe.json 应是数组");
        return 2;
    };
    let mut overall_max = 0.0f32;
    for (idx, entry) in arr.iter().enumerate() {
        match run_probe(&mut policy, &body_toml, entry) {
            Ok(max_diff) => {
                overall_max = overall_max.max(max_diff);
                println!("probe[{idx}] seed {} 最大差 {max_diff:.3e}", entry["seed"]);
            }
            Err(e) => {
                eprintln!("probe[{idx}] 失败:{e}");
                return 1;
            }
        }
    }
    println!("OK({} 条 probe,总最大差 {overall_max:.3e})", arr.len());
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .canonicalize()
            .expect("仓库根")
    }

    fn quad() -> BodyDef {
        let text = std::fs::read_to_string(repo_root().join("assets/bodies/quad.body.toml"))
            .expect("读 quad");
        BodyDef::from_toml_str(&text).expect("quad 合法")
    }

    fn fixture(name: &str) -> Vec<u8> {
        std::fs::read(repo_root().join("assets/policies/test_fixtures").join(name)).expect("读夹具")
    }

    const TERMS45: [&str; 6] = [
        "base_lin_vel",
        "base_ang_vel",
        "projected_gravity",
        "joint_pos",
        "joint_vel",
        "last_action",
    ];

    fn manifest_text(body_hash: &str) -> String {
        let mut s = format!(
            "name = \"test\"\nbody_hash = \"{body_hash}\"\ncontrol_period_ticks = 2\naction_scale = 0.25\n"
        );
        for t in TERMS45 {
            s.push_str(&format!("[[obs]]\nterm = \"{t}\"\n"));
        }
        s
    }

    fn policy_dir(toml_text: &str, onnx: &[u8]) -> tempfile::TempDir {
        let dir = tempfile::TempDir::new().expect("临时目录");
        std::fs::write(dir.path().join("policy.toml"), toml_text).expect("写 toml");
        std::fs::write(dir.path().join("policy.onnx"), onnx).expect("写 onnx");
        dir
    }

    /// 合法策略目录(zero45x12 + quad + 45 维布局):不带 probe 加载成功、退出 0。
    #[test]
    fn legal_dir_loads_ok() {
        let q = quad();
        let dir = policy_dir(&manifest_text(&q.hash().hex()), &fixture("zero45x12.onnx"));
        assert!(load_policy(dir.path(), &q).is_ok(), "合法目录应加载成功");
        let code = cmd_check_policy(&[
            dir.path().display().to_string(),
            "--body".to_owned(),
            repo_root()
                .join("assets/bodies/quad.body.toml")
                .display()
                .to_string(),
        ]);
        assert_eq!(code, 0, "不带 probe 应返回 0");
    }

    /// 哈希写错:加载失败,消息含策略包哈希与身体哈希两个。
    #[test]
    fn wrong_hash_fails_with_both_hashes() {
        let q = quad();
        let bad = "0000000000000000";
        let dir = policy_dir(&manifest_text(bad), &fixture("zero45x12.onnx"));
        let err = load_policy(dir.path(), &q).expect_err("哈希不符应失败");
        let got = q.hash().hex();
        assert!(err.contains(bad), "消息应含策略包哈希 {bad}:{err}");
        assert!(err.contains(&got), "消息应含身体哈希 {got}:{err}");
        let code = cmd_check_policy(&[
            dir.path().display().to_string(),
            "--body".to_owned(),
            repo_root()
                .join("assets/bodies/quad.body.toml")
                .display()
                .to_string(),
        ]);
        assert_ne!(code, 0, "哈希错应非零退出");
    }
}
