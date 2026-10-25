//! xtask —— 仓库内开发工具(REQ-008 D3)。子命令:`smoke`(图形冒烟编排)。
//!
//! 编排流程(REQ-008 §5):
//! 1. `cargo build --release -p app -p viewer`(目标目录尊重 `CARGO_TARGET_DIR`);
//! 2. 每个场景:系统临时目录建数据目录 → 探测空闲端口 → `smoke::server_config`
//!    生成配置 → 起服务器(等到"监听"行或端口可连,上限 30 s)→ 起 viewer
//!    `--script`,等待上限 = 场景 `timeout_secs` + 30 s,超时杀掉(`timed_out`)
//!    → 无论成败结束服务器 → `scan_server_log` 判 panic;
//! 3. 全部场景跑完打印汇总,以 `overall_exit_code` 退出。
//!
//! 墙钟与时间戳(放行评论风险 5):本 crate 是本机开发工具,不在确定性核内,
//! 计时/取时间用 `std::time`,逐处局部 `#[allow(clippy::disallowed_methods)]`。
//! 面向用户的进度/汇总输出用 `println!`/`eprintln!` 是编排器的正当输出。

/// REQ-008 单元 04:冒烟报告(接口桩)。
pub mod report;
/// REQ-008 单元 04:AI 审图与校准(接口桩)。
pub mod review;
pub mod smoke;

use std::path::{Path, PathBuf};
use std::process::Child;
use std::time::Duration;

use smoke::{ScenarioRun, ServerLog, scan_server_log, server_config};

/// 场景目录(仓库根相对;xtask 以仓库根为工作目录运行)。
const SCENARIO_DIR: &str = "smoke/scenarios";
/// 服务器就绪等待上限(REQ-008 §5:上限 30 s)。
const SERVER_READY_TIMEOUT: Duration = Duration::from_secs(30);
/// viewer 等待上限 = 场景 timeout_secs + 该余量(放行评论范围 3)。
const VIEWER_GRACE: Duration = Duration::from_secs(30);

/// 命令行入口(不含程序名),返回进程退出码。
pub fn run(args: Vec<String>) -> i32 {
    match parse_args(&args) {
        Ok(Command::Smoke { selectors, out }) => cmd_smoke(&selectors, out.as_deref()),
        Err(usage) => {
            eprintln!("{usage}");
            2
        }
    }
}

/// 解析出的子命令。
enum Command {
    /// `smoke [场景名或路径 ...] [--out <目录>] [--no-ai]`。
    Smoke {
        selectors: Vec<String>,
        out: Option<PathBuf>,
    },
}

/// 参数解析:未知子命令、未知参数、缺值、重复 `--out` 都报错;`--no-ai` 本单元
/// 接受、等同默认行为(AI 审图在单元 04 接入)。
fn parse_args(args: &[String]) -> Result<Command, String> {
    let mut selectors = Vec::new();
    let mut out: Option<PathBuf> = None;
    let mut iter = args.iter();
    let Some(sub) = iter.next() else {
        return Err(
            "用法:cargo xtask smoke [场景名或路径 ...] [--out <目录>] [--no-ai]".to_owned(),
        );
    };
    if sub != "smoke" {
        return Err(format!("未知子命令 `{sub}`(当前只有 smoke)"));
    }
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--out" => {
                if out.is_some() {
                    return Err("--out 重复给出".to_owned());
                }
                let value = iter.next().ok_or_else(|| "--out 缺少参数值".to_owned())?;
                out = Some(PathBuf::from(value));
            }
            // 单元 04 接入 AI 审图;本单元接受并等同默认行为。
            "--no-ai" => {}
            other => {
                if other.starts_with("--") {
                    return Err(format!("未知参数 `{other}`"));
                }
                selectors.push((*arg).to_owned());
            }
        }
    }
    Ok(Command::Smoke { selectors, out })
}

/// `cargo xtask smoke` 的编排主体。返回进程退出码。
fn cmd_smoke(selectors: &[String], out: Option<&Path>) -> i32 {
    let repo_root = repo_root();
    let scenario_dir = repo_root.join(SCENARIO_DIR);
    let scenarios = match smoke::resolve_scenarios(&scenario_dir, selectors) {
        Ok(paths) => paths,
        Err(e) => {
            eprintln!("场景选择失败:{e}");
            return 2;
        }
    };

    // 先编译(§5 步 1;目标目录尊重 CARGO_TARGET_DIR)。
    if let Err(e) = cargo_build_release(&repo_root) {
        eprintln!("编译失败:{e}");
        return 2;
    }
    let target_dir = target_dir(&repo_root);
    let server_bin = target_dir.join("release").join("app.exe");
    let viewer_bin = target_dir.join("release").join("viewer.exe");
    for bin in [&server_bin, &viewer_bin] {
        if !bin.is_file() {
            eprintln!("编译产物缺失:{}", bin.display());
            return 2;
        }
    }

    // 输出根目录:--out 或 <target>/smoke/<YYYYmmdd-HHMMSS>。
    let out_root = out.map(Path::to_path_buf).unwrap_or_else(|| {
        let stamp = timestamp_now();
        target_dir.join("smoke").join(stamp)
    });
    if let Err(e) = std::fs::create_dir_all(&out_root) {
        eprintln!("创建输出目录 {} 失败:{e}", out_root.display());
        return 2;
    }

    let mut runs: Vec<ScenarioRun> = Vec::new();
    for path in &scenarios {
        println!("=== 场景 {} ===", path.display());
        let run = run_scenario(path, &repo_root, &server_bin, &viewer_bin, &out_root);
        println!(
            "--- {}:{name}(viewer 退出码 {exit:?}{timeout};服务器 panic {panic})\n",
            if run.passed() { "通过" } else { "未通过" },
            name = run.name,
            exit = run.viewer_exit,
            timeout = if run.timed_out {
                ",编排器超时"
            } else {
                ""
            },
            panic = if run.server_panicked { "是" } else { "否" },
        );
        runs.push(run);
    }

    // 汇总(§5:每个场景是否通过、viewer 退出码、超时、服务器 panic、以
    // overall_exit_code 退出)。
    println!("==== 汇总(共 {} 个场景)====", runs.len());
    for run in &runs {
        println!(
            "[{}] {name}:viewer 退出码 {exit:?}{};服务器 panic {panic}",
            if run.passed() { "通过" } else { "未通过" },
            if run.timed_out {
                ";编排器超时"
            } else {
                ""
            },
            name = run.name,
            exit = run.viewer_exit,
            panic = if run.server_panicked { "是" } else { "否" },
        );
    }
    let code = smoke::overall_exit_code(&runs);
    println!("overall_exit_code = {code}");
    code
}

/// 跑单个场景(配置 → 服务器 → viewer → 收尾),产出 [`ScenarioRun`]。
/// 服务器与 viewer 的标准输出/错误、manifest、截图都落在 `<out_root>/<场景名>/`。
fn run_scenario(
    path: &Path,
    repo_root: &Path,
    server_bin: &Path,
    viewer_bin: &Path,
    out_root: &Path,
) -> ScenarioRun {
    let failed = |name: &str| ScenarioRun {
        name: name.to_owned(),
        viewer_exit: None,
        timed_out: false,
        server_panicked: false,
    };
    let name = match path.file_stem().and_then(|s| s.to_str()) {
        Some(name) => name.to_owned(),
        None => return failed(""),
    };
    let out_dir = out_root.join(&name);
    if let Err(e) = std::fs::create_dir_all(&out_dir) {
        eprintln!("创建场景输出目录 {} 失败:{e}", out_dir.display());
        return failed(&name);
    }

    // 场景解析(拿 [server] 覆盖项与 timeout_secs;解析失败 = 本场景失败)。
    let scenario = match std::fs::read_to_string(path)
        .map_err(|e| e.to_string())
        .and_then(|text| viewer::script::parse_scenario(&text).map_err(|e| e.to_string()))
    {
        Ok(scenario) => scenario,
        Err(e) => {
            eprintln!("场景 {} 解析失败:{e}", path.display());
            return failed(&name);
        }
    };

    // 临时数据目录(系统临时目录;场景结束后删除——服务器 checkpoint 每 600 拍
    // 落盘且从不清理,不能放输出目录,风险 4)。
    let data_dir = std::env::temp_dir().join(format!("xtask-smoke-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&data_dir);
    if let Err(e) = std::fs::create_dir_all(&data_dir) {
        eprintln!("创建临时数据目录 {} 失败:{e}", data_dir.display());
        return failed(&name);
    }

    // 配置 + 起服务器(起不来换端口重试一次,风险 3)。配置文件落在输出子目录
    // (server.toml,复现运行用)。
    let (_config_path, server) =
        match spawn_server(&scenario.server, &data_dir, &out_dir, repo_root, server_bin) {
            Ok(pair) => pair,
            Err(()) => return failed(&name),
        };

    // Drop 守卫:任何返回路径都结束服务器并删除临时数据目录(风险 2)。
    struct Guard<'a> {
        server: Child,
        data_dir: &'a Path,
    }
    impl Drop for Guard<'_> {
        fn drop(&mut self) {
            let _ = self.server.kill();
            let _ = self.server.wait();
            let _ = std::fs::remove_dir_all(self.data_dir);
        }
    }
    let mut guard = Guard {
        server,
        data_dir: &data_dir,
    };

    // 等服务器就绪:"监听"行或端口可连(上限 30 s,§5 步 2)。
    let stdout_log = out_dir.join("server.stdout.log");
    let listen = match wait_server_ready(&stdout_log, SERVER_READY_TIMEOUT) {
        Some(addr) => addr,
        None => {
            eprintln!(
                "服务器 {} s 内未就绪,放弃场景 {name}",
                SERVER_READY_TIMEOUT.as_secs()
            );
            return ScenarioRun {
                name,
                viewer_exit: None,
                timed_out: false,
                server_panicked: server_panicked(&out_dir),
            };
        }
    };

    // viewer:`--addr --script --out`(§5 步 3);等待上限 = timeout_secs + 30 s。
    let wait = Duration::from_secs(scenario.timeout_secs) + VIEWER_GRACE;
    let mut viewer = match std::process::Command::new(viewer_bin)
        .arg("--addr")
        .arg(&listen)
        .arg("--script")
        .arg(path)
        .arg("--out")
        .arg(&out_dir)
        .current_dir(repo_root)
        .spawn()
    {
        Ok(child) => child,
        Err(e) => {
            eprintln!("启动 viewer 失败:{e}");
            return ScenarioRun {
                name,
                viewer_exit: None,
                timed_out: false,
                server_panicked: server_panicked(&out_dir),
            };
        }
    };

    // viewer 等待的墙钟上限(编排器 = 本机工具,不属确定性核;风险 5)。
    #[allow(clippy::disallowed_methods)]
    let started = Instant::now();
    let mut timed_out = false;
    let viewer_exit = loop {
        match viewer.try_wait() {
            Ok(Some(status)) => break status.code(),
            Ok(None) => {}
            Err(e) => {
                eprintln!("等待 viewer 失败:{e}");
                break None;
            }
        }
        if started.elapsed() > wait {
            timed_out = true;
            let _ = viewer.kill();
            let _ = viewer.wait();
            break None;
        }
        std::thread::sleep(Duration::from_millis(50));
    };

    // 无论成败都结束服务器(§5 步 4;Guard 兜底,这里主动收口让日志完整落盘)。
    let _ = guard.server.kill();
    let _ = guard.server.wait();

    ScenarioRun {
        name,
        viewer_exit,
        timed_out,
        server_panicked: server_panicked(&out_dir),
    }
}

/// 生成配置并启动服务器(标准输出/标准错误落盘)。起不来(端口竞争)换端口
/// 重试一次;配置非法或两次都起不来返回 Err。
fn spawn_server(
    server: &toml::Table,
    data_dir: &Path,
    out_dir: &Path,
    repo_root: &Path,
    server_bin: &Path,
) -> Result<(PathBuf, Child), ()> {
    for attempt in 0..2 {
        let port = probe_free_port();
        let config = match server_config(server, &format!("127.0.0.1:{port}"), data_dir) {
            Ok(config) => config,
            Err(e) => {
                eprintln!("生成服务器配置失败:{e}");
                return Err(());
            }
        };
        let config_path = out_dir.join("server.toml");
        if let Err(e) = std::fs::write(&config_path, config.to_toml_string()) {
            eprintln!("写配置 {} 失败:{e}", config_path.display());
            return Err(());
        }
        let stdout =
            std::fs::File::create(out_dir.join("server.stdout.log")).expect("创建服务器日志文件");
        let stderr = std::fs::File::create(out_dir.join("server.stderr.log"))
            .expect("创建服务器错误日志文件");
        match std::process::Command::new(server_bin)
            .arg(&config_path)
            .current_dir(repo_root)
            .stdout(stdout)
            .stderr(stderr)
            .spawn()
        {
            Ok(child) => return Ok((config_path, child)),
            Err(e) => eprintln!("启动服务器失败(第 {} 次):{e}", attempt + 1),
        }
    }
    Err(())
}

/// `cargo build --release -p app -p viewer`(继承本进程输出)。
fn cargo_build_release(repo_root: &Path) -> Result<(), String> {
    let status = std::process::Command::new("cargo")
        .args(["build", "--release", "-p", "app", "-p", "viewer"])
        .current_dir(repo_root)
        .status()
        .map_err(|e| format!("cargo 启动失败:{e}"))?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("cargo build 失败:{status}"))
    }
}

/// 仓库根(xtask 源码位于 `<根>/crates/xtask`)。
fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .components()
        .collect()
}

/// 目标目录:尊重 `CARGO_TARGET_DIR`,缺省 `<仓库根>/target`。
fn target_dir(repo_root: &Path) -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
}

/// 探测一个空闲端口(TcpListener 绑 127.0.0.1:0,拿到端口即关)。
fn probe_free_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0")
        .expect("绑定 127.0.0.1:0 探测空闲端口")
        .local_addr()
        .expect("本地地址可读")
        .port()
}

/// 从日志文本抓"监听 <地址>"行(服务器 main 的就绪标志)。
fn parse_listen_line(text: &str) -> Option<String> {
    text.lines()
        .find(|line| line.starts_with("监听 "))
        .map(|line| line.trim_start_matches("监听 ").trim().to_owned())
}

/// 轮询等服务器就绪:先读标准输出日志抓"监听"行,抓到后验证端口可连;
/// 30 s 上限内不成返回 None。
fn wait_server_ready(stdout_log: &Path, limit: Duration) -> Option<String> {
    // 就绪等待的墙钟上限(§5:30 s;风险 5)。
    #[allow(clippy::disallowed_methods)]
    let deadline = Instant::now() + limit;
    loop {
        if let Some(addr) = std::fs::read_to_string(stdout_log)
            .ok()
            .and_then(|text| parse_listen_line(&text))
            && std::net::TcpStream::connect(&addr).is_ok()
        {
            return Some(addr);
        }
        #[allow(clippy::disallowed_methods)]
        if Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// 服务器日志 panic 扫描(标准输出 + 标准错误落盘文件;文件缺失当空串)。
fn server_panicked(out_dir: &Path) -> bool {
    let read = |name: &str| std::fs::read_to_string(out_dir.join(name)).unwrap_or_default();
    matches!(
        scan_server_log(&read("server.stdout.log"), &read("server.stderr.log")),
        ServerLog::Panicked { .. }
    )
}

/// `YYYYmmdd-HHMMSS` 时间戳(输出目录名;编排器 = 本机工具,风险 5)。
fn timestamp_now() -> String {
    #[allow(clippy::disallowed_methods)]
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("系统时间在 UNIX_EPOCH 之后");
    let days = now.as_secs() / 86_400;
    let secs_of_day = now.as_secs() % 86_400;
    let (y, m, d) = civil_from_days(days as i64);
    format!(
        "{y:04}{m:02}{d:02}-{h:02}{min:02}{s:02}",
        h = secs_of_day / 3600,
        min = (secs_of_day % 3600) / 60,
        s = secs_of_day % 60,
    )
}

/// 天数(自 1970-01-01)→ (年, 月, 日)(Howard Hinnant 的 civil_from_days)。
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

use std::time::{Instant, SystemTime, UNIX_EPOCH};

#[cfg(test)]
mod tests {
    use super::*;

    /// 参数解析:未知子命令 / 未知参数 / 缺值 / 重复 --out 报错;--no-ai 接受;
    /// 场景选择项与旗标顺序无关。
    ///
    /// 会变红的实现:未知参数被忽略(静默错跑);--no-ai 报错;缺值 panic。
    #[test]
    fn parse_args_smoke_selectors_and_flags() {
        let cmd = parse_args(&["smoke".to_owned()]).expect("无选择项 = 全部场景");
        assert!(
            matches!(cmd, Command::Smoke { ref selectors, out: None } if selectors.is_empty()),
            "无选择项 = 全部场景"
        );
        let cmd = parse_args(&[
            "smoke".to_owned(),
            "a".to_owned(),
            "--out".to_owned(),
            "o".to_owned(),
            "--no-ai".to_owned(),
            "b".to_owned(),
        ])
        .expect("选择项与旗标混合");
        let Command::Smoke { selectors, out } = &cmd;
        assert_eq!(selectors, &["a", "b"]);
        assert_eq!(*out, Some(PathBuf::from("o")));
        for bad in [
            vec!["run"],
            vec!["smoke", "--wat"],
            vec!["smoke", "--out"],
            vec!["smoke", "--out", "a", "--out", "b"],
        ] {
            let args: Vec<String> = bad.iter().map(|s| (*s).to_owned()).collect();
            assert!(parse_args(&args).is_err(), "应报错:{bad:?}");
        }
    }

    /// "监听"行解析:抓行去前缀;无行返回 None。
    ///
    /// 会变红的实现:前缀匹配不严(把别的行当地址)。
    #[test]
    fn parse_listen_line_extracts_addr() {
        assert_eq!(
            parse_listen_line("其他日志\n监听 127.0.0.1:35999\n驱动完成\n"),
            Some("127.0.0.1:35999".to_owned())
        );
        assert_eq!(parse_listen_line("没有监听行\n"), None);
    }

    /// 时间戳目录名:格式(8 位日期 - 6 位时间);civil 换算锚定已知日期。
    ///
    /// 会变红的实现:格式错(缺补零、分隔符);civil 换算错月日。
    #[test]
    fn timestamp_format_and_civil_conversion() {
        let stamp = timestamp_now();
        assert_eq!(stamp.len(), 15, "YYYYmmdd-HHMMSS:{stamp}");
        assert_eq!(stamp.as_bytes()[8], b'-');
        assert!(stamp[..8].chars().all(|c| c.is_ascii_digit()));
        assert!(stamp[9..].chars().all(|c| c.is_ascii_digit()));
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(19_723), (2024, 1, 1), "2024-01-01");
    }
}
