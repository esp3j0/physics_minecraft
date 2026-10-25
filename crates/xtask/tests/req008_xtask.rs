//! REQ-008 单元 03 验收用例(总架构师编写;issue #16):`cargo xtask smoke` 编排器的纯逻辑部分。
//!
//! 约定同 `crates/viewer/tests/req008_acceptance.rs`:实施方不得修改断言,完成后只去掉
//! `#[ignore]`。进程编排(起停服务器与 viewer、超时、日志收集)由总架构师实跑核对。

use std::path::{Path, PathBuf};

use xtask::smoke::{
    ScenarioRun, ServerLog, overall_exit_code, resolve_scenarios, scan_server_log, server_config,
};

fn table(text: &str) -> toml::Table {
    text.parse::<toml::Table>().expect("测试用 toml 合法")
}

/// 服务器配置:场景覆盖项叠加在默认配置上;`listen_addr`、`data_dir` 写入;结果经 toml 往返
/// 不变;未知键、类型错误、违反校验规则都报错。
///
/// 会变红的实现:覆盖项被忽略;只拼接字符串、不经 `Config` 校验(未知键或非法值漏到服务器
/// 启动时才失败);listen/data_dir 没写进去。
#[test]
#[ignore = "REQ-008 单元 03 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s5_server_config_applies_overrides_and_validates() {
    let data = Path::new("C:/tmp/smoke-data");
    let cfg = server_config(
        &table("world_seed = 7\nmax_awake_dynamic = 300\n[activation]\ndata_load_radius = 40\n"),
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
        40.0,
        "[activation] 表内覆盖项生效"
    );
    assert_eq!(cfg.listen_addr, "127.0.0.1:35999");
    assert_eq!(cfg.data_dir, PathBuf::from(data));
    let defaults = app::config::Config::default();
    assert_eq!(cfg.dt, defaults.dt, "未覆盖的键保持默认");
    assert_eq!(
        app::config::Config::from_toml_str(&cfg.to_toml_string()).expect("往返可解析"),
        cfg,
        "写成配置文件后读回相等"
    );

    let empty = server_config(&toml::Table::new(), "127.0.0.1:1", data).expect("空覆盖项");
    assert_eq!(empty.world_seed, defaults.world_seed);

    for (bad, what) in [
        ("no_such_key = 1\n", "未知键"),
        ("world_seed = \"seven\"\n", "类型错误"),
        (
            "[activation]\ncollider_load_radius = 40\n",
            "② 半径大于 ① 半径",
        ),
    ] {
        assert!(
            server_config(&table(bad), "127.0.0.1:1", data).is_err(),
            "{what}:应报错"
        );
    }
}

/// 服务器日志:标准输出或标准错误任一处出现 Rust panic 即判 panic,并带出那一行。
///
/// 会变红的实现:只扫标准输出(panic 消息在标准错误);把普通日志判成 panic。
#[test]
#[ignore = "REQ-008 单元 03 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s5_scan_server_log() {
    assert_eq!(
        scan_server_log("监听 127.0.0.1:35999\n", ""),
        ServerLog::Clean
    );
    let line = "thread 'main' panicked at crates/app/src/driver.rs:10:5:";
    assert_eq!(
        scan_server_log("监听 127.0.0.1:35999\n", &format!("{line}\nboom\n")),
        ServerLog::Panicked {
            line: line.to_owned()
        }
    );
    assert!(matches!(
        scan_server_log(&format!("x\n{line}\n"), ""),
        ServerLog::Panicked { .. }
    ));
}

/// 场景选择:无选择项 → 目录下全部 `*.toml` 按文件名升序;按名或按路径选;找不到报错;
/// 重复只保留一次;空目录报错。
///
/// 会变红的实现:把非 toml 文件当场景;顺序不稳定;找不到的名字被静默跳过。
#[test]
#[ignore = "REQ-008 单元 03 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s5_resolve_scenarios() {
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
        .expect("按名与路径混合"),
        vec![b.clone(), a.clone()],
        "保持选择顺序,重复只留一次"
    );
    match resolve_scenarios(dir.path(), &["nope".to_owned()]) {
        Ok(v) => panic!("找不到的场景应报错,实为 {v:?}"),
        Err(e) => assert!(e.contains("nope"), "错误信息点名:{e}"),
    }
    let empty = tempfile::tempdir().expect("临时目录");
    assert!(resolve_scenarios(empty.path(), &[]).is_err(), "空目录报错");
}

/// 通过判定与汇总退出码。
///
/// 会变红的实现:超时或服务器 panic 时仍判通过;检查失败(viewer 退出码 1)被当成通过;
/// 一个场景都没跑却返回 0。
#[test]
#[ignore = "REQ-008 单元 03 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_s5_exit_code_aggregation() {
    let run = |exit: Option<i32>, timed_out: bool, panicked: bool| ScenarioRun {
        name: "s".to_owned(),
        viewer_exit: exit,
        timed_out,
        server_panicked: panicked,
    };
    assert!(run(Some(0), false, false).passed());
    assert!(!run(Some(1), false, false).passed(), "检查失败");
    assert!(!run(Some(2), false, false).passed(), "场景失败");
    assert!(!run(None, true, false).passed(), "超时");
    assert!(!run(Some(0), false, true).passed(), "服务器 panic");
    assert_eq!(overall_exit_code(&[run(Some(0), false, false)]), 0);
    assert_eq!(
        overall_exit_code(&[run(Some(0), false, false), run(Some(1), false, false)]),
        1
    );
    assert_eq!(overall_exit_code(&[]), 1, "一个场景都没跑");
}
