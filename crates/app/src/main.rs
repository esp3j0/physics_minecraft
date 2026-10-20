//! 装配入口:命令行 → Config → bootstrap → 驱动 loop → 打印 outcome(蓝图 §3 main)。

use std::path::Path;
use std::time::Duration;

use app::config::Config;
use app::driver::{DriverArgs, DriverCommand, bootstrap, run};
use voxel::SimpleRegistry;

fn main() {
    // 命令行第一个参数 = 配置路径(无则 None → 全默认,Config::load 语义)。
    let config_path = std::env::args().nth(1);
    let config =
        Config::load(config_path.as_deref().map(Path::new)).expect("配置加载失败(fail-stop)");

    // 装配:开库 → 查最新检查点 → replay 恢复或全新(registry 用 SimpleRegistry::builtin)。
    let (session, log, store, resume_from, spawn_next_seq) =
        bootstrap(&config, || Box::new(SimpleRegistry::builtin())).expect("装配失败(fail-stop)");

    // 世界数据源 + 分片构造闭包(REQ-002 §4.3/§4.4):provider = worker 通道 +
    // 生成器(按 config.world_kind)+ config.world_seed;new_sim = shard_sim_config
    // 换算 + registry 工厂。
    let (provider, new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        });

    // tokio runtime(网络装配与驱动同 runtime;REQ-003 决议 2 并发模型)。
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("tokio runtime 构建失败");

    // 驱动:起 mpsc 通道 + spawn run + 等待完成(生产开 ctrl_c;命令发送端
    // 随 main 存续——外部命令缝本轮无写方,通道关闭 = 关机,故不能 drop)。
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(config.inbox_capacity);
    std::mem::forget(tx);

    // 网络装配(REQ-004A 任务 00):serve(spawn accept 循环)+ ObserveLink
    // 三缝真值;失败 = fail-stop(启动期错误)。async 装配经 block_on 在驱动
    // spawn 前完成,监听就绪后模拟才起跑;observe 三缝随 DriverArgs 移交。
    let network = runtime
        .block_on(app::net::assemble_network(&config))
        .unwrap_or_else(|e| panic!("网络监听失败(fail-stop):{e}"));
    println!("监听 {}", network.addr);

    let outcome = runtime.block_on(async move {
        let handle = tokio::spawn(run(DriverArgs {
            config,
            session,
            store,
            log,
            commands: rx,
            listen_ctrl_c: true,
            provider,
            new_sim,
            observe: Some(network.observe),
            spawn_next_seq,
        }));
        handle.await.expect("驱动任务 panic(fail-stop)")
    });
    // 驱动已退(ctrl_c/通道关闭):广播网络关机(accept 停止 + 存量连接收口)。
    network.shutdown.shutdown();
    runtime.shutdown_timeout(Duration::from_secs(1));

    // 一行摘要:续跑起点 + tick / 检查点 / 反压 / 告警 / 世界生长计数。
    match resume_from {
        Some(tick) => println!("续跑自拍 {}", tick.0),
        None => println!("全新会话"),
    }
    println!(
        "驱动完成:ticks={}, checkpoints={:?}, rejected={}, alert_at_exit={}, \
         shards_spawned={}, shards_despawned={}, chunks_loaded={}, chunks_unloaded={}",
        outcome.ticks_run,
        outcome.checkpoints_written,
        outcome.commands_rejected,
        outcome.alert_active_at_exit,
        outcome.shards_spawned,
        outcome.shards_despawned,
        outcome.chunks_loaded,
        outcome.chunks_unloaded
    );
}
