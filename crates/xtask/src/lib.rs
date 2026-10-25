//! xtask —— 仓库内开发工具(REQ-008 D3)。子命令:`smoke`(图形冒烟编排)。
//!
//! 编排流程(REQ-008 §5–§7):
//! 1. `cargo build --release -p app -p viewer`(目标目录尊重 `CARGO_TARGET_DIR`);
//! 2. 每个场景:系统临时目录建数据目录 → 探测空闲端口 → `smoke::server_config`
//!    生成配置 → 起服务器(等到"监听"行或端口可连,上限 30 s)→ 起 viewer
//!    `--script`,等待上限 = 场景 `timeout_secs` + 30 s,超时杀掉(`timed_out`)
//!    → 无论成败结束服务器 → `scan_server_log` 判 panic;
//! 3. AI 审图(单元 04,`--no-ai` 跳过):先读校准集并运行校准,再逐张审 manifest
//!    里的截图,按 `shot_flagged` 计算待复核;任何准备或校准失败 → 全部截图待复核;
//! 4. 输出根目录写 `report.md` 与 `report.json`,终端打印汇总与待复核清单;
//!    退出码只看机器检查(`overall_exit_code`),AI 结论不影响(D1);
//! 5. `--selftest`(单元 04,AC-S8):跑 `selftest-negative`(期望 viewer 退出码 1 且
//!    manifest 至少一步 check_failed)+ 校准(期望通过);预期失败全部发生才以 0 退出。
//!
//! 墙钟与时间戳(放行评论风险 5):本 crate 是本机开发工具,不在确定性核内,
//! 计时/取时间用 `std::time`,逐处局部 `#[allow(clippy::disallowed_methods)]`。
//! 面向用户的进度/汇总输出用 `println!`/`eprintln!` 是编排器的正当输出。

/// REQ-008 单元 04:冒烟报告。
pub mod report;
/// REQ-008 单元 04:AI 审图与校准。
pub mod review;
pub mod smoke;
/// REQ-008 单元 04:真实审图器(zcode CLI)。
pub mod zcode;

use std::path::{Path, PathBuf};
use std::process::Child;
use std::time::Duration;

use report::{RunReport, ScenarioEntry, ShotEntry, report_json, report_markdown};
use review::{
    CalibrationOutcome, ReviewResult, Reviewer, load_calibration, review_image, run_calibration,
    shot_flagged,
};
use smoke::{ScenarioRun, ServerLog, scan_server_log, server_config};
use viewer::script::manifest::Manifest;
use zcode::ZcodeReviewer;

/// 场景目录(仓库根相对;xtask 以仓库根为工作目录运行)。
const SCENARIO_DIR: &str = "smoke/scenarios";
/// 审图提示词模板(仓库根相对;总架构师财产,只读)。
const REVIEW_PROMPT: &str = "smoke/review-prompt.md";
/// 校准集目录(仓库根相对)。
const CALIBRATION_DIR: &str = "smoke/calibration";
/// 服务器就绪等待上限(REQ-008 §5:上限 30 s)。
const SERVER_READY_TIMEOUT: Duration = Duration::from_secs(30);
/// viewer 等待上限 = 场景 timeout_secs + 该余量(放行评论范围 3)。
const VIEWER_GRACE: Duration = Duration::from_secs(30);

/// 命令行入口(不含程序名),返回进程退出码。
pub fn run(args: Vec<String>) -> i32 {
    match parse_args(&args) {
        Ok(Command::Smoke {
            selectors,
            out,
            no_ai,
            selftest,
        }) => {
            if selftest {
                cmd_selftest(no_ai, out.as_deref())
            } else {
                cmd_smoke(&selectors, out.as_deref(), no_ai)
            }
        }
        Err(usage) => {
            eprintln!("{usage}");
            2
        }
    }
}

/// 解析出的子命令。
enum Command {
    /// `smoke [场景名或路径 ...] [--out <目录>] [--no-ai] [--selftest]`。
    Smoke {
        selectors: Vec<String>,
        out: Option<PathBuf>,
        /// 跳过 AI 审图与校准(REQ-008 §5)。
        no_ai: bool,
        /// 自检:跑 selftest-negative 与校准,预期失败全部发生才以 0 退出(AC-S8)。
        selftest: bool,
    },
}

/// 参数解析:未知子命令、未知参数、缺值、重复旗标、`--selftest` 带选择项都报错。
fn parse_args(args: &[String]) -> Result<Command, String> {
    let mut selectors = Vec::new();
    let mut out: Option<PathBuf> = None;
    let mut no_ai = false;
    let mut selftest = false;
    let mut iter = args.iter();
    let Some(sub) = iter.next() else {
        return Err(
            "用法:cargo xtask smoke [场景名或路径 ...] [--out <目录>] [--no-ai] [--selftest]"
                .to_owned(),
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
            "--no-ai" => {
                if no_ai {
                    return Err("--no-ai 重复给出".to_owned());
                }
                no_ai = true;
            }
            "--selftest" => {
                if selftest {
                    return Err("--selftest 重复给出".to_owned());
                }
                selftest = true;
            }
            other => {
                if other.starts_with("--") {
                    return Err(format!("未知参数 `{other}`"));
                }
                selectors.push((*arg).to_owned());
            }
        }
    }
    if selftest && !selectors.is_empty() {
        return Err("--selftest 只跑内置的 selftest-negative,不接受场景选择项".to_owned());
    }
    Ok(Command::Smoke {
        selectors,
        out,
        no_ai,
        selftest,
    })
}

/// `cargo xtask smoke` 的编排主体。返回进程退出码:只看机器检查(D1),
/// AI 结论与校准失败都不改变它。
fn cmd_smoke(selectors: &[String], out: Option<&Path>, no_ai: bool) -> i32 {
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

    let mut entries: Vec<ScenarioEntry> = Vec::new();
    for path in &scenarios {
        println!("=== 场景 {} ===", path.display());
        let (run, manifest) = run_scenario(path, &repo_root, &server_bin, &viewer_bin, &out_root);
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
        entries.push(ScenarioEntry { run, manifest });
    }

    // AI 审图(单元 04 范围 3):先读校准集并运行校准,再逐张审截图;--no-ai 跳过。
    // 准备失败(模板 / 校准集 / 审图器)→ 中止审图,全部截图标待复核;结论不影响
    // 退出码(D1)。
    let (calibration, shots) = if no_ai {
        println!("AI 审图:未运行(--no-ai),截图全部标记待复核");
        (None, collect_shots(&entries))
    } else {
        match review_all(&repo_root, &out_root, &entries) {
            Ok(pair) => pair,
            Err(e) => {
                eprintln!("AI 审图中止:{e}(全部截图标记待复核)");
                (None, collect_shots(&entries))
            }
        }
    };

    let report = RunReport {
        scenarios: entries,
        calibration,
        shots,
    };
    for (name, content) in [
        ("report.md", report_markdown(&report)),
        ("report.json", report_json(&report)),
    ] {
        let path = out_root.join(name);
        if let Err(e) = std::fs::write(&path, content) {
            eprintln!("写 {} 失败:{e}", path.display());
            return 2;
        }
        println!("报告:{}", path.display());
    }

    // 汇总(§5:每个场景是否通过、viewer 退出码、超时、服务器 panic,以
    // overall_exit_code 退出;单元 04 增:校准状态与待复核清单)。
    println!("==== 汇总(共 {} 个场景)====", report.scenarios.len());
    for entry in &report.scenarios {
        let run = &entry.run;
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
    match &report.calibration {
        Some(cal) if cal.ok => println!("校准:通过"),
        Some(cal) => {
            println!("校准:未通过(本次全部 AI 结论作废)");
            for m in &cal.mismatches {
                println!("  - {m}");
            }
        }
        None => println!("校准:AI 未运行"),
    }
    let flagged: Vec<&ShotEntry> = report.shots.iter().filter(|s| s.flagged).collect();
    println!("待复核截图 {} / {} 张:", flagged.len(), report.shots.len());
    for shot in &flagged {
        println!("  {}/{}", shot.scenario, shot.file);
    }
    let runs: Vec<ScenarioRun> = report.scenarios.iter().map(|e| e.run.clone()).collect();
    let code = smoke::overall_exit_code(&runs);
    println!("overall_exit_code = {code}");
    code
}

/// 跑单个场景(配置 → 服务器 → viewer → 收尾),产出 ([`ScenarioRun`], manifest)。
/// 服务器与 viewer 的标准输出/错误、manifest、截图都落在 `<out_root>/<场景名>/`;
/// manifest 缺失或解析失败时为 None(报告里如实标注)。
fn run_scenario(
    path: &Path,
    repo_root: &Path,
    server_bin: &Path,
    viewer_bin: &Path,
    out_root: &Path,
) -> (ScenarioRun, Option<Manifest>) {
    let failed = |name: &str| ScenarioRun {
        name: name.to_owned(),
        viewer_exit: None,
        timed_out: false,
        server_panicked: false,
    };
    let name = match path.file_stem().and_then(|s| s.to_str()) {
        Some(name) => name.to_owned(),
        None => return (failed(""), None),
    };
    let out_dir = out_root.join(&name);
    if let Err(e) = std::fs::create_dir_all(&out_dir) {
        eprintln!("创建场景输出目录 {} 失败:{e}", out_dir.display());
        return (failed(&name), None);
    }

    // 场景解析(拿 [server] 覆盖项与 timeout_secs;解析失败 = 本场景失败)。
    let scenario = match std::fs::read_to_string(path)
        .map_err(|e| e.to_string())
        .and_then(|text| viewer::script::parse_scenario(&text).map_err(|e| e.to_string()))
    {
        Ok(scenario) => scenario,
        Err(e) => {
            eprintln!("场景 {} 解析失败:{e}", path.display());
            return (failed(&name), None);
        }
    };

    // 临时数据目录(系统临时目录;场景结束后删除——服务器 checkpoint 每 600 拍
    // 落盘且从不清理,不能放输出目录,风险 4)。
    let data_dir = std::env::temp_dir().join(format!("xtask-smoke-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&data_dir);
    if let Err(e) = std::fs::create_dir_all(&data_dir) {
        eprintln!("创建临时数据目录 {} 失败:{e}", data_dir.display());
        return (failed(&name), None);
    }

    // 配置 + 起服务器(起不来换端口重试一次,风险 3)。配置文件落在输出子目录
    // (server.toml,复现运行用)。
    let (_config_path, server) =
        match spawn_server(&scenario.server, &data_dir, &out_dir, repo_root, server_bin) {
            Ok(pair) => pair,
            Err(()) => return (failed(&name), None),
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
            return (
                ScenarioRun {
                    name,
                    viewer_exit: None,
                    timed_out: false,
                    server_panicked: server_panicked(&out_dir),
                },
                None,
            );
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
            return (
                ScenarioRun {
                    name,
                    viewer_exit: None,
                    timed_out: false,
                    server_panicked: server_panicked(&out_dir),
                },
                None,
            );
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

    // manifest 由 viewer 逐步骤落盘;缺失或残缺按 None 如实进报告。
    let manifest = std::fs::read_to_string(out_dir.join("manifest.json"))
        .ok()
        .and_then(|text| Manifest::from_json(&text).ok());
    (
        ScenarioRun {
            name,
            viewer_exit,
            timed_out,
            server_panicked: server_panicked(&out_dir),
        },
        manifest,
    )
}

/// 从各场景的 manifest 收集截图条目(AI 未运行形态:review = None、一律待复核,
/// 同 `shot_flagged(None, _)`)。
fn collect_shots(entries: &[ScenarioEntry]) -> Vec<ShotEntry> {
    let mut shots = Vec::new();
    for entry in entries {
        let Some(manifest) = &entry.manifest else {
            continue;
        };
        for step in &manifest.steps {
            let Some(shot) = &step.screenshot else {
                continue;
            };
            shots.push(ShotEntry {
                scenario: entry.run.name.clone(),
                file: shot.file.clone(),
                expect: shot.expect.clone(),
                review: None,
                // AI 未运行 = 待复核。
                flagged: true,
            });
        }
    }
    shots
}

/// 读审图提示词模板(仓库根相对)。
fn load_template(repo_root: &Path) -> Result<String, String> {
    let path = repo_root.join(REVIEW_PROMPT);
    std::fs::read_to_string(&path).map_err(|e| format!("读取 {} 失败:{e}", path.display()))
}

/// 读校准集并运行校准(先于逐张审图,REQ-008 §6),打印结果。返回用例数与结果;
/// 校准集读不到或审图器初始化失败 → Err。
fn calibration_once(
    repo_root: &Path,
    template: &str,
    reviewer: &mut dyn Reviewer,
) -> Result<(usize, CalibrationOutcome), String> {
    let cases = load_calibration(&repo_root.join(CALIBRATION_DIR))?;
    println!("==== AI 校准(共 {} 条)====", cases.len());
    let outcome = run_calibration(reviewer, template, &cases);
    if outcome.ok {
        println!("校准:通过({} 条全部判对)", cases.len());
    } else {
        println!("校准:未通过——本次全部 AI 结论作废");
        for m in &outcome.mismatches {
            println!("  - {m}");
        }
    }
    Ok((cases.len(), outcome))
}

/// 校准 + 逐张审图(manifest 的截图步骤,期望取 manifest 的 expect)。返回
/// (校准结果, 截图条目)。准备失败 → Err,调用方按"AI 未运行"处理。
fn review_all(
    repo_root: &Path,
    out_root: &Path,
    entries: &[ScenarioEntry],
) -> Result<(Option<CalibrationOutcome>, Vec<ShotEntry>), String> {
    let template = load_template(repo_root)?;
    let mut reviewer = ZcodeReviewer::new()?.with_progress(true);
    let (_, calibration) = calibration_once(repo_root, &template, &mut reviewer)?;

    let total: usize = entries
        .iter()
        .filter_map(|entry| entry.manifest.as_ref())
        .map(|manifest| {
            manifest
                .steps
                .iter()
                .filter(|s| s.screenshot.is_some())
                .count()
        })
        .sum();
    let mut shots = Vec::new();
    let mut done = 0usize;
    for entry in entries {
        let Some(manifest) = &entry.manifest else {
            continue;
        };
        for step in &manifest.steps {
            let Some(shot) = &step.screenshot else {
                continue;
            };
            done += 1;
            println!(
                "==== AI 审图 {done}/{total}:{}/{} ====",
                entry.run.name, shot.file
            );
            let image = out_root.join(&entry.run.name).join(&shot.file);
            let review = review_image(&mut reviewer, &template, &image, &shot.expect);
            let flagged = shot_flagged(Some(&review), calibration.ok);
            let line = match &review {
                ReviewResult::Judged(v) => format!(
                    "pass = {},confidence = {v:.2}{}",
                    v.pass,
                    if flagged { "(待复核)" } else { "" },
                    v = v.confidence,
                ),
                ReviewResult::Unknown { reason } => format!("无法判定({reason})→ 待复核"),
            };
            println!("  {line}");
            shots.push(ShotEntry {
                scenario: entry.run.name.clone(),
                file: shot.file.clone(),
                expect: shot.expect.clone(),
                review: Some(review),
                flagged,
            });
        }
    }
    Ok((Some(calibration), shots))
}

/// `cargo xtask smoke --selftest`(单元 04 范围 4;AC-S8):运行 `selftest-negative`,
/// 要求 viewer 退出码 1 且 manifest 至少一步 `check_failed`;不带 `--no-ai` 时再运行
/// 校准,要求通过(应判不通过的用例确实被判不通过)。预期的失败全部发生才以 0 退出。
fn cmd_selftest(no_ai: bool, out: Option<&Path>) -> i32 {
    let repo_root = repo_root();
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

    let scenario_dir = repo_root.join(SCENARIO_DIR);
    let scenarios = match smoke::resolve_scenarios(&scenario_dir, &["selftest-negative".to_owned()])
    {
        Ok(paths) => paths,
        Err(e) => {
            eprintln!("场景选择失败:{e}");
            return 2;
        }
    };
    let Some(path) = scenarios.first() else {
        eprintln!("按名选择 selftest-negative 未命中任何场景文件");
        return 2;
    };

    let out_root = out.map(Path::to_path_buf).unwrap_or_else(|| {
        target_dir
            .join("smoke")
            .join(format!("selftest-{}", timestamp_now()))
    });
    if let Err(e) = std::fs::create_dir_all(&out_root) {
        eprintln!("创建输出目录 {} 失败:{e}", out_root.display());
        return 2;
    }

    println!("=== 自检:运行 {} ===", path.display());
    let (run, manifest) = run_scenario(path, &repo_root, &server_bin, &viewer_bin, &out_root);

    let mut items = selftest_machine_items(&run, manifest.as_ref());
    if !no_ai {
        items.push(selftest_calibration_item(&repo_root));
    }

    println!("==== 自检结果 ====");
    for (ok, line) in &items {
        println!("[{}] {line}", if *ok { "预期达成" } else { "预期落空" });
    }
    let code = selftest_exit_code(&items);
    println!("selftest_exit_code = {code}");
    code
}

/// --selftest 的机器核对项:viewer 退出码恰为 1(机器检查以非零退出码报失败)、
/// manifest 至少一步 `check_failed`、无编排器超时、服务器无 panic。校准项由调用方追加。
fn selftest_machine_items(run: &ScenarioRun, manifest: Option<&Manifest>) -> Vec<(bool, String)> {
    let mut items = Vec::new();
    items.push((
        run.viewer_exit == Some(1),
        format!(
            "viewer 退出码 = {:?}(期望 1:机器检查以非零退出码报失败)",
            run.viewer_exit
        ),
    ));
    let check_failed = manifest
        .map(|m| {
            m.steps
                .iter()
                .filter(|s| s.outcome == "check_failed")
                .count()
        })
        .unwrap_or(0);
    items.push((
        check_failed >= 1,
        if manifest.is_some() {
            format!("manifest 中 check_failed 步数 = {check_failed}(期望 ≥ 1)")
        } else {
            "manifest 缺失或解析失败(期望 ≥ 1 步 check_failed)".to_owned()
        },
    ));
    items.push((
        !run.timed_out,
        format!("编排器超时 = {}(期望 否)", run.timed_out),
    ));
    items.push((
        !run.server_panicked,
        format!(
            "服务器 panic = {}(期望 否)",
            if run.server_panicked { "是" } else { "否" }
        ),
    ));
    items
}

/// --selftest 的校准核对项:初始化失败 / 有判错都算"预期落空"。
fn selftest_calibration_item(repo_root: &Path) -> (bool, String) {
    let attempt = (|| {
        let template = load_template(repo_root)?;
        let mut reviewer = ZcodeReviewer::new()?.with_progress(true);
        calibration_once(repo_root, &template, &mut reviewer)
    })();
    match attempt {
        Ok((count, cal)) if cal.ok => (
            true,
            format!("校准:{count} 条全部判对(应判不通过的用例确实被判不通过)"),
        ),
        Ok((_, cal)) => (false, format!("校准未通过:{}", cal.mismatches.join(";"))),
        Err(e) => (false, format!("校准未能运行:{e}")),
    }
}

/// 自检退出码:核对项全部成立 → 0(预期的失败全部发生),否则 1。
fn selftest_exit_code(items: &[(bool, String)]) -> i32 {
    if items.iter().all(|(ok, _)| *ok) {
        0
    } else {
        1
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
    use std::path::Path;

    use viewer::script::manifest::{Manifest, ManifestCamera, ManifestShot, ManifestStep};

    use super::*;

    /// 参数解析:未知子命令 / 未知参数 / 缺值 / 重复旗标 / --selftest 带选择项报错;
    /// 场景选择项与旗标顺序无关;--no-ai 与 --selftest 都被接受。
    ///
    /// 会变红的实现:未知参数被忽略(静默错跑);旗标重复被静默吞掉;--selftest
    /// 带选择项仍运行;缺值 panic。
    #[test]
    fn parse_args_smoke_selectors_and_flags() {
        let cmd = parse_args(&["smoke".to_owned()]).expect("无选择项 = 全部场景");
        assert!(
            matches!(cmd, Command::Smoke { ref selectors, out: None, no_ai: false, selftest: false } if selectors.is_empty()),
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
        let Command::Smoke {
            selectors,
            out,
            no_ai,
            selftest,
        } = &cmd;
        assert_eq!(selectors, &["a", "b"]);
        assert_eq!(*out, Some(PathBuf::from("o")));
        assert!(*no_ai, "--no-ai 被接受");
        assert!(!*selftest);
        let cmd = parse_args(&["smoke".to_owned(), "--selftest".to_owned()]).expect("自检");
        assert!(
            matches!(
                cmd,
                Command::Smoke {
                    selftest: true,
                    no_ai: false,
                    ..
                }
            ),
            "--selftest 被接受"
        );
        for bad in [
            vec!["run"],
            vec!["smoke", "--wat"],
            vec!["smoke", "--out"],
            vec!["smoke", "--out", "a", "--out", "b"],
            vec!["smoke", "--no-ai", "--no-ai"],
            vec!["smoke", "--selftest", "--selftest"],
            vec!["smoke", "--selftest", "basic-world"],
        ] {
            let args: Vec<String> = bad.iter().map(|s| (*s).to_owned()).collect();
            assert!(parse_args(&args).is_err(), "应报错:{bad:?}");
        }
    }

    /// --selftest 的机器核对项与退出码:viewer 退出码 1 + manifest 至少一步
    /// check_failed + 无超时 + 无 panic → 全部成立、退出 0;退出码 0(自检场景
    /// 意外通过)、无 check_failed、manifest 缺失、超时、panic 各自落空 → 退出 1。
    ///
    /// 会变红的实现:期望退出码写成 0;check_failed 门槛写成 0;manifest 缺失当
    /// 通过;任一核对项落空仍返回 0。
    #[test]
    fn selftest_items_and_exit_code() {
        let run = |exit: Option<i32>, timed_out: bool, panicked: bool| ScenarioRun {
            name: "selftest-negative".to_owned(),
            viewer_exit: exit,
            timed_out,
            server_panicked: panicked,
        };
        let manifest = |check_failed_steps: usize| {
            let mut steps: Vec<ManifestStep> = Vec::new();
            for n in 0..check_failed_steps {
                steps.push(ManifestStep {
                    step: n + 1,
                    label: None,
                    action: "check".to_owned(),
                    start_tick: 0,
                    end_tick: 1,
                    camera: ManifestCamera {
                        pos: [0.0; 3],
                        yaw: 0.0,
                        pitch: 0.0,
                    },
                    outcome: "check_failed".to_owned(),
                    messages: vec!["活实体数 3 不满足 Eq(0)".to_owned()],
                    screenshot: None,
                });
            }
            Manifest {
                scenario: "selftest-negative".to_owned(),
                req: None,
                description: "d".to_owned(),
                steps,
                result: None,
            }
        };
        // 预期失败发生:退出码 1 + 一步 check_failed → 全部成立 → 0。
        let items = selftest_machine_items(&run(Some(1), false, false), Some(&manifest(1)));
        assert!(items.iter().all(|(ok, _)| *ok), "{items:?}");
        assert_eq!(selftest_exit_code(&items), 0);

        // 退出码 0(自检场景意外通过)→ 落空 → 1。
        let items = selftest_machine_items(&run(Some(0), false, false), Some(&manifest(1)));
        assert!(!items[0].0, "退出码 0 = 机器检查没报失败,预期落空");
        assert_eq!(selftest_exit_code(&items), 1);

        // manifest 没有 check_failed 步 → 落空。
        let items = selftest_machine_items(&run(Some(1), false, false), Some(&manifest(0)));
        assert!(!items[1].0, "没有 check_failed 步 = 机器检查没报失败");
        assert_eq!(selftest_exit_code(&items), 1);

        // manifest 缺失 → 落空,不能当通过。
        let items = selftest_machine_items(&run(Some(1), false, false), None);
        assert!(!items[1].0);
        assert_eq!(selftest_exit_code(&items), 1);

        // 编排器超时 / 服务器 panic → 各自落空 → 1。
        let items = selftest_machine_items(&run(Some(1), true, true), Some(&manifest(1)));
        assert!(!items[2].0 && !items[3].0);
        assert_eq!(selftest_exit_code(&items), 1);
    }

    /// AI 未运行时收集截图条目:每张 review = None、flagged = true;没有 manifest
    /// 的场景跳过;非截图步骤不产生条目。
    ///
    /// 会变红的实现:--no-ai 路径漏标待复核;expect 或文件名抄错;manifest 缺失
    /// panic。
    #[test]
    fn collect_shots_without_ai_marks_everything_for_review() {
        let step = |action: &str, shot: Option<ManifestShot>| ManifestStep {
            step: 1,
            label: None,
            action: action.to_owned(),
            start_tick: 0,
            end_tick: 1,
            camera: ManifestCamera {
                pos: [0.0; 3],
                yaw: 0.0,
                pitch: 0.0,
            },
            outcome: "ok".to_owned(),
            messages: vec![],
            screenshot: shot,
        };
        let shot = ManifestShot {
            file: "02-a.png".to_owned(),
            name: "a".to_owned(),
            expect: "期望文本".to_owned(),
            hud: false,
            live_entities: 0,
            chunks_loaded: 0,
        };
        let manifest = |steps| Manifest {
            scenario: "s".to_owned(),
            req: None,
            description: "d".to_owned(),
            steps,
            result: None,
        };
        let entries = vec![
            ScenarioEntry {
                run: ScenarioRun {
                    name: "alpha".to_owned(),
                    viewer_exit: Some(0),
                    timed_out: false,
                    server_panicked: false,
                },
                manifest: Some(manifest(vec![
                    step("camera", None),
                    step("screenshot", Some(shot)),
                ])),
            },
            ScenarioEntry {
                run: ScenarioRun {
                    name: "beta".to_owned(),
                    viewer_exit: None,
                    timed_out: true,
                    server_panicked: false,
                },
                manifest: None,
            },
        ];
        let shots = collect_shots(&entries);
        assert_eq!(shots.len(), 1, "没有 manifest 的场景没有截图可收集");
        let collected = &shots[0];
        assert_eq!(
            (
                collected.scenario.as_str(),
                collected.file.as_str(),
                collected.expect.as_str()
            ),
            ("alpha", "02-a.png", "期望文本"),
            "场景名、文件名、期望逐字转写"
        );
        assert!(collected.review.is_none(), "AI 未运行");
        assert!(collected.flagged, "AI 未运行 = 待复核");
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

    /// 回归锚(单元 03 遗留):smoke 模块可从根访问;报告入口存在。
    /// (防误删 pub 模块声明;不测行为。)
    #[test]
    fn modules_wired() {
        let _ = Path::new(".");
        assert_eq!(smoke::overall_exit_code(&[]), 1);
    }
}
