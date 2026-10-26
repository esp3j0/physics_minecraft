//! #17 验收用例(总架构师编写):checkpoint 保留最近 N 个 + v5 压缩。
//!
//! 约定同其他验收文件:实施方不得修改断言,完成后只去掉 `#[ignore]`;认为断言与需求不符时
//! 在 #17 提出。需求见 REQ-001 决议 11 修订(保留数)与 REQ-005 D8 版本记录(v5)。

use app::config::Config;
use app::driver::{
    AppError, CHECKPOINT_FORMAT_VERSION, CheckpointBundle, DriverArgs, DriverCommand,
    decode_checkpoint, encode_checkpoint, run,
};
use storage::mem::MemCheckpointStore;
use storage::redb::RedbStore;
use storage::traits::{CheckpointId, CheckpointStore};
use types::Tick;
use types::glam::Vec3;
use voxel::{BlockRegistry, SimpleRegistry};

// #17 D2:v5 起为"版本前缀 + lz4 压缩体";REQ-009 起为 v6,编码规则不变。
const _: () = assert!(CHECKPOINT_FORMAT_VERSION >= 5);

fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

/// 追加 `appends` 次(第 i 次的拍号为 10·i、内容为 [i]),再核对:ID 从 1 起连续;
/// `keep = 0` 全部保留,否则只剩最近 `keep` 个;`read_latest` 是最后一次追加。
fn check_retention(store: &mut dyn CheckpointStore, keep: usize, appends: u64) {
    for i in 1..=appends {
        let id = store.append(Tick(i * 10), &[i as u8]).expect("追加成功");
        assert_eq!(id, CheckpointId(i), "ID 从 1 起连续递增");
    }
    for i in 1..=appends {
        let present = store.read(CheckpointId(i)).expect("可读").is_some();
        let want = keep == 0 || i + keep as u64 > appends;
        assert_eq!(
            present,
            want,
            "keep = {keep}:第 {i} 个应{}",
            if want { "保留" } else { "删除" }
        );
    }
    assert_eq!(
        store.read_latest().expect("可读"),
        Some((Tick(appends * 10), vec![appends as u8])),
        "最新一个永远在"
    );
}

/// K1/K3:redb 与内存两种存储,保留最近 N 个;`keep = 0` 不清理。
///
/// 会变红的实现:只删不留(删掉最新的);删多一个或少一个;`keep = 0` 也清理;内存存储没跟上。
#[test]
fn acc_k1_k3_keep_last_n() {
    let dir = tempfile::tempdir().expect("临时目录");
    let mut redb3 = RedbStore::open_with_keep(&dir.path().join("k3.redb"), 3).expect("打开");
    check_retention(&mut redb3, 3, 8);
    let mut redb0 = RedbStore::open_with_keep(&dir.path().join("k0.redb"), 0).expect("打开");
    check_retention(&mut redb0, 0, 8);
    let mut redb1 = RedbStore::open_with_keep(&dir.path().join("k1.redb"), 1).expect("打开");
    check_retention(&mut redb1, 1, 5);
    check_retention(&mut MemCheckpointStore::with_keep(3), 3, 8);
    check_retention(&mut MemCheckpointStore::with_keep(0), 0, 8);
    check_retention(&mut MemCheckpointStore::new(), 0, 8);
}

/// K2 与启动清理:清理后重开库再追加,ID 继续单调、不复用;打开时若现存多于 keep 个,
/// 立即删到只剩最近 keep 个(覆盖调小 keep 后重启)。
///
/// 会变红的实现:按行数恢复 ID(删旧后行数变少,ID 被复用);只在追加时清理,打开时不清理。
#[test]
fn acc_k2_ids_monotonic_and_prune_on_open() {
    let dir = tempfile::tempdir().expect("临时目录");
    let path = dir.path().join("ck.redb");
    {
        let mut s = RedbStore::open_with_keep(&path, 3).expect("打开");
        for i in 1..=5 {
            s.append(Tick(i), &[1]).expect("追加");
        }
    }
    {
        let mut s = RedbStore::open_with_keep(&path, 3).expect("重开");
        let id = s.append(Tick(6), &[2]).expect("追加");
        assert_eq!(id, CheckpointId(6), "重开后 ID 继续单调,不复用");
        let present: Vec<u64> = (1..=6)
            .filter(|i| s.read(CheckpointId(*i)).expect("可读").is_some())
            .collect();
        assert_eq!(present, vec![4, 5, 6]);
    }
    let path2 = dir.path().join("grow.redb");
    {
        let mut s = RedbStore::open(&path2).expect("打开(不清理)");
        for i in 1..=8 {
            s.append(Tick(i), &[1]).expect("追加");
        }
    }
    let s = RedbStore::open_with_keep(&path2, 3).expect("以 keep = 3 重开");
    let present: Vec<u64> = (1..=8)
        .filter(|i| s.read(CheckpointId(*i)).expect("可读").is_some())
        .collect();
    assert_eq!(present, vec![6, 7, 8], "打开时即删到只剩最近 3 个");
}

/// K5:文件有界。1 MiB 的 blob 追加 150 次、keep = 4,文件不超过 32 MiB(issue 实测稳定在约 20 MiB)。
///
/// 会变红的实现:不删旧(文件随追加线性增长到 150 MiB 以上);删旧不在同一事务、空闲页无法复用。
#[test]
fn acc_k5_file_size_bounded() {
    let dir = tempfile::tempdir().expect("临时目录");
    let path = dir.path().join("bounded.redb");
    let blob = vec![0x5a_u8; 1 << 20];
    {
        let mut s = RedbStore::open_with_keep(&path, 4).expect("打开");
        for i in 1..=150 {
            s.append(Tick(i), &blob).expect("追加");
        }
    }
    let size = std::fs::metadata(&path).expect("文件存在").len();
    assert!(
        size <= 32 << 20,
        "keep = 4 时文件应有界(≤ 32 MiB),实为 {:.1} MiB",
        size as f64 / (1 << 20) as f64
    );
}

/// 用一个真实会话的调度器快照拼一个 bundle(场景无关,只为编解码)。
fn bundle(format_version: u32, log_bytes: Vec<u8>) -> CheckpointBundle {
    let session = sim_api::session::Session::new(Config::default().session_config());
    let world = session.world_snapshot();
    session.shutdown();
    CheckpointBundle {
        format_version,
        world,
        log_bytes,
        spawn_next_seq: 42,
    }
}

/// Z2 + 编解码:当前版本为 5;v5 往返无损;可压缩内容确实被压缩;v4 档(旧的未压缩
/// bincode)被明确拒绝并点名版本;乱码报错。
///
/// 会变红的实现:版本没升;没压缩;先解压再看版本(v4 档报"解压失败"而不是版本错误);
/// 乱码 panic。
#[test]
fn acc_z2_v5_codec_and_old_versions_rejected() {
    let original = bundle(CHECKPOINT_FORMAT_VERSION, vec![0u8; 1_000_000]);
    let bytes = encode_checkpoint(&original);
    assert!(
        bytes.len() < 50_000,
        "1 MB 全零日志应被压缩,实为 {} 字节",
        bytes.len()
    );
    let back = decode_checkpoint(&bytes).expect("自有编码可解码");
    assert_eq!(back.format_version, CHECKPOINT_FORMAT_VERSION);
    assert_eq!(back.world, original.world, "世界快照无损");
    assert_eq!(back.log_bytes, original.log_bytes, "日志无损");
    assert_eq!(back.spawn_next_seq, 42);

    let v4 = bincode::serde::encode_to_vec(bundle(4, vec![1, 2, 3]), bincode::config::standard())
        .expect("编码 v4 档");
    match decode_checkpoint(&v4) {
        Err(AppError::Decode(msg)) => assert!(
            msg.contains("unsupported checkpoint format version 4"),
            "v4 档应明确报版本错误:{msg}"
        ),
        Err(other) => panic!("v4 档应为 Decode 错误,实为 {other}"),
        Ok(_) => panic!("v4 档必须被拒绝"),
    }
    assert!(
        decode_checkpoint(&[5, 0xff, 0xfe, 0x00, 0x13]).is_err(),
        "版本对但内容是乱码:报错而非 panic"
    );
    assert!(decode_checkpoint(&[]).is_err(), "空字节报错");
}

/// C1:配置项 `checkpoint_keep` 默认 10,可由 toml 覆写,非法值报错。
///
/// 会变红的实现:默认值不是 10;键名不对(被当成未知键拒绝);负数被接受。
#[test]
fn acc_c1_checkpoint_keep_config() {
    assert_eq!(Config::default().checkpoint_keep, 10);
    assert_eq!(
        Config::from_toml_str("checkpoint_keep = 3\n")
            .expect("合法")
            .checkpoint_keep,
        3
    );
    assert_eq!(
        Config::from_toml_str("checkpoint_keep = 0\n")
            .expect("0 = 不清理,合法")
            .checkpoint_keep,
        0
    );
    assert!(Config::from_toml_str("checkpoint_keep = -1\n").is_err());
}

/// 暂停时间下推进 N 拍。
async fn advance_ticks(ticks: u32) {
    let dt = std::time::Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

/// K4 + Z1(生产路径):真 redb + 真驱动,每 60 拍一个 checkpoint、keep = 3,跑约 400 拍后
/// 停服:库中恰好剩最近 3 个(且确实发生过清理);最新的一个 ≤ 64 KB、是 v5;用同一目录重新
/// bootstrap 能从最新的 checkpoint 续跑。
///
/// 会变红的实现:bootstrap 没把 `checkpoint_keep` 传给存储(库里留下全部);压缩没走生产编码
/// 路径(最新一个约 2.7 MB);新格式写得出读不回(重启失败)。
#[tokio::test(start_paused = true)]
async fn acc_k4_z1_production_path_retention_and_compression() {
    let dir = tempfile::tempdir().expect("临时目录");
    let config = Config {
        data_dir: dir.path().to_path_buf(),
        worker_count: 2,
        checkpoint_interval_ticks: 60,
        checkpoint_keep: 3,
        ..Config::default()
    };
    let (session, log, store, resume_from, _seq) =
        app::driver::bootstrap(&config, new_registry()).expect("装配空目录");
    assert_eq!(resume_from, None);
    let (provider, new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(16);
    let driver = tokio::spawn(run(DriverArgs {
        config: config.clone(),
        session,
        store,
        log,
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: None,
        spawn_next_seq: 0,
    }));
    tx.send(DriverCommand::MoveObserver {
        position: Vec3::new(8.0, types::WORLD_Y_MIN as f32 + 6.0, 8.0),
    })
    .await
    .expect("驱动存活");
    advance_ticks(400).await;
    tx.send(DriverCommand::Shutdown).await.expect("驱动存活");
    driver.await.expect("驱动不 panic");

    let latest_tick = {
        let ck = RedbStore::open(&dir.path().join("checkpoints.redb")).expect("重开 checkpoint 库");
        let present: Vec<u64> = (1..=64)
            .filter(|i| ck.read(CheckpointId(*i)).expect("可读").is_some())
            .collect();
        assert_eq!(present.len(), 3, "库中恰好保留 3 个:{present:?}");
        assert!(
            present[0] >= 4 && present.windows(2).all(|w| w[1] == w[0] + 1),
            "保留的是最近且连续的 3 个,且之前至少写过 3 个更早的:{present:?}"
        );
        let (tick, blob) = ck.read_latest().expect("可读").expect("有最新");
        assert!(
            blob.len() <= 64 * 1024,
            "生产路径的 checkpoint 应已压缩(≤ 64 KB),实为 {} 字节",
            blob.len()
        );
        let b = decode_checkpoint(&blob).expect("最新的可解码");
        assert_eq!(b.format_version, CHECKPOINT_FORMAT_VERSION);
        tick
    };

    let (session, _log, store, resume_from, _seq) =
        app::driver::bootstrap(&config, new_registry()).expect("同一目录重新装配");
    assert_eq!(resume_from, Some(latest_tick), "从最新的 checkpoint 续跑");
    session.shutdown();
    let _ = store.shutdown();
}
