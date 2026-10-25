//! smoke —— 图形冒烟编排的纯逻辑部分(REQ-008 §5;AC-S5)。
//!
//! 进程编排(起停服务器与 viewer、超时、日志收集)在 `crate::run`,本模块只含
//! 可单测的纯逻辑:配置生成、日志扫描、场景选择、结果汇总。

use std::path::{Path, PathBuf};

/// 生成场景用的服务器配置:以 `app::config::Config::default()` 为底,叠加场景 `[server]` 覆盖项
/// (键与 `Config` 的 toml 同名),再写入 `listen_addr` 与 `data_dir`。经 `Config` 的解析与校验:
/// 未知键、类型错误、违反校验规则(如 ② 半径大于 ① 半径)一律报错。
pub fn server_config(
    server: &toml::Table,
    listen_addr: &str,
    data_dir: &Path,
) -> Result<app::config::Config, String> {
    // 顶层未知键在 `Config` 的反序列化里会被静默忽略(结构体级 serde(default)
    // 无 deny_unknown_fields;只有 [activation] 表内拒绝),场景编排要求写错
    // 即报错:以默认配置序列化输出的顶层键集合为白名单(单一来源——字段
    // 集变更时白名单自动跟随;[activation] 表内的未知键由 from_toml_str 拒绝)。
    let known: std::collections::BTreeSet<String> = app::config::Config::default()
        .to_toml_string()
        .parse::<toml::Table>()
        .expect("默认配置的 toml 输出必为表")
        .keys()
        .cloned()
        .collect();
    for key in server.keys() {
        if !known.contains(key) {
            return Err(format!("未知的 [server] 覆盖键 `{key}`"));
        }
    }
    // 覆盖项序列化回 toml 文本,与编排器写入的 listen/data_dir 一起经
    // `Config::from_toml_str` 走生产解析路径(别名键、deny_unknown_fields、
    // 结构校验全部同源生效),不另写一份拼接逻辑。listen/data_dir 放最前:
    // 覆盖项可能以 `[表]` 结尾,追加的顶层键会掉进该表。
    let overrides = toml::to_string(server).map_err(|e| format!("覆盖项序列化失败:{e}"))?;
    let text = format!(
        "listen_addr = {listen_addr:?}\ndata_dir = {:?}\n{overrides}",
        data_dir.display().to_string()
    );
    app::config::Config::from_toml_str(&text).map_err(|e| e.to_string())
}

/// 服务器日志判定。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ServerLog {
    Clean,
    /// 含 Rust panic 消息(`panicked at`)的那一行。
    Panicked {
        line: String,
    },
}

/// 扫描服务器的标准输出与标准错误:任一处出现 panic 即 `Panicked`(取第一处)。
pub fn scan_server_log(stdout: &str, stderr: &str) -> ServerLog {
    let panic_line = |text: &str| -> Option<String> {
        text.lines()
            .find(|line| line.contains("panicked at"))
            .map(str::to_owned)
    };
    if let Some(line) = panic_line(stdout) {
        return ServerLog::Panicked { line };
    }
    match panic_line(stderr) {
        Some(line) => ServerLog::Panicked { line },
        None => ServerLog::Clean,
    }
}

/// 选出要跑的场景文件:
/// - `selectors` 为空 → `dir` 下全部 `*.toml`,按文件名升序;
/// - 每个选择项先当作已存在的文件路径,否则当作场景名(`dir/<名>.toml`);
/// - 找不到 → 报错并点名;`dir` 下没有任何场景 → 报错;重复项只保留第一次。
pub fn resolve_scenarios(dir: &Path, selectors: &[String]) -> Result<Vec<PathBuf>, String> {
    if selectors.is_empty() {
        let mut all: Vec<PathBuf> = std::fs::read_dir(dir)
            .map_err(|e| format!("读取场景目录 {} 失败:{e}", dir.display()))?
            .filter_map(|entry| {
                let path = entry.ok()?.path();
                (path.extension().and_then(|e| e.to_str()) == Some("toml")).then_some(path)
            })
            .collect();
        if all.is_empty() {
            return Err(format!("场景目录 {} 下没有任何 *.toml 场景", dir.display()));
        }
        all.sort();
        return Ok(all);
    }

    let mut picked: Vec<PathBuf> = Vec::new();
    for selector in selectors {
        let by_path = PathBuf::from(selector);
        let candidate = if by_path.is_file() {
            by_path
        } else {
            let by_name = dir.join(format!("{selector}.toml"));
            if by_name.is_file() {
                by_name
            } else {
                return Err(format!(
                    "找不到场景 `{selector}`(既不是文件,{} 下也没有 {selector}.toml)",
                    dir.display()
                ));
            }
        };
        if !picked.contains(&candidate) {
            picked.push(candidate);
        }
    }
    Ok(picked)
}

/// 单个场景的运行结果。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScenarioRun {
    pub name: String,
    /// viewer 进程退出码;被编排器超时杀掉时为 None。
    pub viewer_exit: Option<i32>,
    pub timed_out: bool,
    pub server_panicked: bool,
}

impl ScenarioRun {
    /// 通过 = viewer 退出码 0、没有超时、服务器没有 panic。
    pub fn passed(&self) -> bool {
        self.viewer_exit == Some(0) && !self.timed_out && !self.server_panicked
    }
}

/// 汇总退出码:全部通过为 0;任一未通过或一个场景都没跑为 1。
pub fn overall_exit_code(runs: &[ScenarioRun]) -> i32 {
    if runs.is_empty() || runs.iter().any(|run| !run.passed()) {
        1
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn table(text: &str) -> toml::Table {
        text.parse().expect("测试 toml 合法")
    }

    /// 覆盖项叠加在默认配置上;listen/data_dir 写入;往返无损;未知键 /
    /// 类型错误 / 校验失败经 `Config::from_toml_str` 报错。
    ///
    /// 会变红的实现:覆盖项被忽略(直接用默认);listen/data_dir 没写进去;
    /// 只拼字符串不经 Config 校验。
    #[test]
    fn server_config_overrides_and_validates() {
        let data = Path::new("C:/tmp/smoke-unit-data");
        let cfg = server_config(
            &table(
                "world_seed = 7\nmax_awake_dynamic = 300\n[activation]\ndata_load_radius = 40\n",
            ),
            "127.0.0.1:35999",
            data,
        )
        .expect("合法覆盖项");
        assert_eq!(cfg.world_seed, types::WorldSeed(7));
        assert_eq!(cfg.max_awake_dynamic, 300);
        assert_eq!(
            cfg.shard_sim_config(types::ShardId::new(
                types::WorldId(0),
                types::DimensionId(0),
                types::ShardPos::new(0, 0)
            ))
            .activation
            .data_load_radius,
            40.0
        );
        assert_eq!(cfg.listen_addr, "127.0.0.1:35999");
        assert_eq!(cfg.data_dir, PathBuf::from(data));
        let defaults = app::config::Config::default();
        assert_eq!(cfg.dt, defaults.dt, "未覆盖的键保持默认");
        assert_eq!(
            app::config::Config::from_toml_str(&cfg.to_toml_string()).expect("往返"),
            cfg,
            "写成配置文件后读回相等"
        );
        for bad in [
            "no_such_key = 1\n",
            "world_seed = \"seven\"\n",
            "[activation]\ncollider_load_radius = 40\n",
        ] {
            assert!(
                server_config(&table(bad), "127.0.0.1:1", data).is_err(),
                "{bad}:应报错"
            );
        }
    }

    /// 标准输出或标准错误任一处 panic 都判 Panicked 并带出那一行;普通日志不误判。
    ///
    /// 会变红的实现:只扫标准输出;把普通行判成 panic。
    #[test]
    fn scan_server_log_finds_panic_in_both_streams() {
        assert_eq!(scan_server_log("监听 127.0.0.1:1\n", ""), ServerLog::Clean);
        let line = "thread 'main' panicked at crates/app/src/main.rs:10:5:";
        assert_eq!(
            scan_server_log("", &format!("{line}\nboom\n")),
            ServerLog::Panicked {
                line: line.to_owned()
            }
        );
        assert!(matches!(
            scan_server_log(&format!("x\n{line}\n"), ""),
            ServerLog::Panicked { .. }
        ));
        assert_eq!(
            scan_server_log("", "普通错误信息,无 panic\n"),
            ServerLog::Clean
        );
    }

    /// 场景选择:全部按文件名升序、非 toml 排除;按名、按路径、混合、去重;
    /// 找不到点名;空目录报错。
    ///
    /// 会变红的实现:把 notes.txt 当场景;顺序不稳定;找不到被静默跳过。
    #[test]
    fn resolve_scenarios_selects_dedupes_and_reports_missing() {
        let dir = tempfile::tempdir().expect("临时目录");
        for f in ["b.toml", "a.toml", "notes.txt"] {
            std::fs::write(dir.path().join(f), "").expect("写文件");
        }
        let a = dir.path().join("a.toml");
        let b = dir.path().join("b.toml");
        assert_eq!(
            resolve_scenarios(dir.path(), &[]).expect("全部"),
            vec![a.clone(), b.clone()]
        );
        assert_eq!(
            resolve_scenarios(dir.path(), &["b".to_owned()]).expect("按名"),
            vec![b.clone()]
        );
        assert_eq!(
            resolve_scenarios(
                dir.path(),
                &["b".to_owned(), a.display().to_string(), "b".to_owned()]
            )
            .expect("混合"),
            vec![b.clone(), a.clone()],
            "保持选择顺序,重复只留一次"
        );
        let err = resolve_scenarios(dir.path(), &["nope".to_owned()]).expect_err("找不到应报错");
        assert!(err.contains("nope"), "错误信息点名:{err}");
        let empty = tempfile::tempdir().expect("临时目录");
        assert!(resolve_scenarios(empty.path(), &[]).is_err(), "空目录报错");
    }

    /// 通过判定与汇总退出码(锚定 ScenarioRun::passed / overall_exit_code)。
    ///
    /// 会变红的实现:超时或 panic 时判通过;全空列表返回 0。
    #[test]
    fn overall_exit_code_aggregates() {
        let run = |exit: Option<i32>, timed_out: bool, panicked: bool| ScenarioRun {
            name: "s".to_owned(),
            viewer_exit: exit,
            timed_out,
            server_panicked: panicked,
        };
        assert!(run(Some(0), false, false).passed());
        assert!(!run(Some(1), false, false).passed());
        assert!(!run(Some(2), false, false).passed());
        assert!(!run(None, true, false).passed());
        assert!(!run(Some(0), false, true).passed());
        assert_eq!(overall_exit_code(&[run(Some(0), false, false)]), 0);
        assert_eq!(
            overall_exit_code(&[run(Some(0), false, false), run(Some(1), false, false)]),
            1
        );
        assert_eq!(overall_exit_code(&[]), 1);
    }
}
