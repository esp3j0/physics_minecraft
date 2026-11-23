//! AC4 渲染与 sim 解耦验证测试(REQ-013 C05 §1.5/§1.7):
//! - T-A(`c05_ta_render_stall_dual_run_reconcile`):暂停时钟双跑对账(栈 B 在 10..=19 拍跳过 drain,
//!   两栈终态拍号、整段 S2C 编码字节流、落体根位置逐位相等);
//! - T-C(`c05_tc_real_thread_stall_sim_keeps_ticking`):真实 OS 线程阻塞 800 ms 不 drain,
//!   验证宿主自主推进(方块更新、落体下落、tick 推进)。

use std::path::{Path, PathBuf};
use std::time::Duration;

use app::config::Config;
use app::local::{LocalConfig, spawn_in_current_runtime};
use messaging::SimCommand;
use sim_api::proto::{C2S, S2C, encode_frame};
use types::glam::Vec3;
use types::{BlockPos, EntityDesc, EntityState};

use crate::client::s2c_to_event;
use crate::script::{
    CameraPose, Engine, EngineStatus, FrameInput, WorldQuery, engine_commands_to_c2s,
    parse_scenario,
};
use crate::world::WorldState;
use crate::{LocalTransport, SequencedTransport, Transport};

const TA_SCENARIO_TOML: &str = r#"
name = "c05_ta_stall"
req = "REQ-013"
timeout_secs = 60
[viewer]
view_distance = 2
[[step]]
wait_ticks = { n = 3 }
[[step]]
spawn = { at = [4.0, -55.0, 4.0] }
[[step]]
wait_ticks = { n = 3 }
[[step]]
set_block = { pos = [0, -60, 0], block = "stone" }
[[step]]
wait_ticks = { n = 39 }
"#;

struct DualRunResult {
    wire_bytes: Vec<u8>,
    world_tick: u64,
    chunk_data_count: usize,
    spawned_count: usize,
    saw_stone_update: bool,
    falling_y: f32,
}

fn repo_bodies_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies")
}

async fn run_ta_stack(stall_window: Option<(usize, usize)>) -> DualRunResult {
    let dir = tempfile::TempDir::new().expect("TempDir");
    let cfg = LocalConfig {
        config: Config {
            data_dir: dir.path().to_path_buf(),
            bodies_dir: repo_bodies_dir(),
            policies_dir: dir.path().join("policies"),
            ..Config::default()
        },
        view_distance: 2,
    };

    let (handle, driver) = spawn_in_current_runtime(cfg)
        .await
        .expect("spawn_in_current_runtime");
    // 暂停时钟对齐口径:先 yield_now() 让 driver 吃掉首枚立即 tick,
    // 此后每次 advance(17ms)+yield_now() 恰推进 1 拍,第 K 次推进后 tick_now()==K。
    tokio::task::yield_now().await;

    let mut transport = SequencedTransport::new(LocalTransport::from_handle(handle));
    let scenario = parse_scenario(TA_SCENARIO_TOML).expect("T-A 内联场景合法");
    let mut engine = Engine::new(
        scenario,
        CameraPose {
            pos: Vec3::ZERO,
            yaw: 0.0,
            pitch: 0.0,
        },
    );
    let mut world = WorldState::new();

    // 前置:经包装层发 MoveObserver(8,-60,8)(seq 1);不发 Subscribe(local 自动订阅)。
    transport.send(C2S::MoveObserver {
        position: Vec3::new(8.0, -60.0, 8.0),
    });

    let mut wire_bytes = Vec::new();
    let mut chunk_data_count = 0usize;
    let mut spawned_count = 0usize;
    let mut saw_stone_update = false;
    let target_cell = (BlockPos::new(0, -60, 0), voxel::builtin::STONE);

    let mut drain_and_apply = |transport: &mut SequencedTransport<LocalTransport>,
                               world: &mut WorldState| {
        for msg in transport.drain() {
            wire_bytes.extend_from_slice(&encode_frame(&msg));
            match &msg {
                S2C::ChunkData { .. } => chunk_data_count += 1,
                S2C::Spawned { .. } => spawned_count += 1,
                S2C::BlockUpdates { updates } if updates.contains(&target_cell) => {
                    saw_stone_update = true;
                }
                _ => {}
            }
            if let Some(ev) = s2c_to_event(msg) {
                world.apply(ev);
            }
        }
    };

    for step in 1..=48usize {
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;

        let stalled = stall_window.is_some_and(|(lo, hi)| (lo..=hi).contains(&step));
        if !stalled {
            drain_and_apply(&mut transport, &mut world);
        }

        let spawned = world.take_spawned();
        let server_tick = world.tick();
        let out = engine.frame(&FrameInput {
            dt: 1.0 / 60.0,
            server_tick,
            world: &world,
            spawned: &spawned,
        });
        for c2s in engine_commands_to_c2s(&out.commands) {
            transport.send(c2s);
        }
        if let EngineStatus::Failed { step: s, reason } = engine.status() {
            panic!("T-A 场景在第 {s} 步失败(推进 {step}):{reason}");
        }
    }

    assert_eq!(
        engine.status(),
        &EngineStatus::Done { failed_checks: 0 },
        "T-A 场景必须在 48 拍内无错完成"
    );

    // 循环结束后再推进 3 次(49..=51)并 drain(冲刷)。
    for _ in 49..=51usize {
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        drain_and_apply(&mut transport, &mut world);
    }

    let world_tick = world.tick();
    let entities = world.live_entities();
    assert_eq!(
        entities.len(),
        1,
        "T-A 终态必须恰好存在 1 个落体实体,实得 {}",
        entities.len()
    );
    let falling_y = entities[0].1.y;

    drop(transport);
    tokio::time::advance(Duration::from_millis(17)).await;
    tokio::task::yield_now().await;
    driver.await.expect("驱动正常退出");

    DualRunResult {
        wire_bytes,
        world_tick,
        chunk_data_count,
        spawned_count,
        saw_stone_update,
        falling_y,
    }
}

/// C05 T-A:headless 双跑对账 — 栈 A 每拍 drain,栈 B 第 10..=19 拍跳过 drain(模拟渲染阻塞)。
/// 两栈终态拍号、flattened S2C 字节流、关键事件计数与落体根位置 y 必须逐位相等。
#[tokio::test(start_paused = true)]
async fn c05_ta_render_stall_dual_run_reconcile() {
    let run_a = run_ta_stack(None).await;
    let run_b = run_ta_stack(Some((10, 19))).await;

    // 锚点 1:两栈 world.tick() == 51(渲染阻塞不改变 sim 拍计数)。
    assert_eq!(run_a.world_tick, 51, "栈 A 终态拍号必须为 51");
    assert_eq!(run_b.world_tick, 51, "栈 B 终态拍号必须为 51");

    // 锚点 2:两栈 flattened 字节向量逐位相等且非空。
    assert!(!run_a.wire_bytes.is_empty(), "栈 A 字节流非空");
    assert_eq!(
        run_a.wire_bytes,
        run_b.wire_bytes,
        "双跑 S2C 编码字节流必须逐位相等(A len={}, B len={})",
        run_a.wire_bytes.len(),
        run_b.wire_bytes.len()
    );

    // 锚点 3:ChunkData == 25、Spawned == 1、含 (0,-60,0)->STONE 的 BlockUpdates。
    for (label, run) in [("stack_a", &run_a), ("stack_b", &run_b)] {
        assert_eq!(
            run.chunk_data_count, 25,
            "{label}: vd=2 下 ChunkData 必须恰为 25"
        );
        assert_eq!(run.spawned_count, 1, "{label}: Spawned 必须恰为 1");
        assert!(
            run.saw_stone_update,
            "{label}: 必须收到含 (0,-60,0)->STONE 的 BlockUpdates"
        );
    }

    // 锚点 4:两栈终态落体根位置 y 逐位相等且 < -56.0。
    assert_eq!(
        run_a.falling_y.to_bits(),
        run_b.falling_y.to_bits(),
        "双跑终态落体 y 必须逐位相等(A={}, B={})",
        run_a.falling_y,
        run_b.falling_y
    );
    assert!(
        run_a.falling_y < -56.0,
        "51 拍后落体应已自 -55 下落至 < -56.0,实得 {}",
        run_a.falling_y
    );
}

/// C05 T-C:真实 OS 线程渲染阻塞 — 发 MoveObserver + Spawn + SetBlock,
/// settling 100 ms 后再 sleep 800 ms 全程不 drain,验证宿主自主推进。
#[test]
fn c05_tc_real_thread_stall_sim_keeps_ticking() {
    let tmp = tempfile::TempDir::new().expect("TempDir");
    let cfg = LocalConfig {
        config: Config {
            data_dir: tmp.path().to_path_buf(),
            bodies_dir: repo_bodies_dir(),
            policies_dir: tmp.path().join("policies"),
            ..Config::default()
        },
        view_distance: 2,
    };

    let mut transport = LocalTransport::new(cfg).expect("LocalTransport::new");
    let target = BlockPos::new(0, -60, 0);

    transport.send(C2S::MoveObserver {
        position: Vec3::new(8.0, -60.0, 8.0),
    });
    // 100 ms settling(等宿主完成 MoveObserver 触发的分片注册,期间不 drain)。
    std::thread::sleep(Duration::from_millis(100));

    transport.send(C2S::Spawn {
        state: EntityState {
            translation: Vec3::new(4.0, -55.0, 4.0),
            ..EntityState::default()
        },
        desc: EntityDesc::default(),
    });
    transport.send(C2S::Command(SimCommand::SetBlock {
        pos: target,
        block: voxel::builtin::STONE,
    }));

    // 800 ms 全程不 drain(模拟渲染阻塞)。
    std::thread::sleep(Duration::from_millis(800));

    let mut world = WorldState::new();
    let mut saw_stone_update = false;
    let mut falling_y: Option<f32> = None;

    for _ in 0..500 {
        for msg in transport.drain() {
            if let S2C::BlockUpdates { updates } = &msg
                && updates.contains(&(target, voxel::builtin::STONE))
            {
                saw_stone_update = true;
            }
            if let Some(ev) = s2c_to_event(msg) {
                world.apply(ev);
            }
        }
        if let Some((_, pos)) = world.live_entities().first() {
            falling_y = Some(pos.y);
        }
        if saw_stone_update && falling_y.is_some_and(|y| y <= -57.0) && world.tick() >= 40 {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    assert!(
        saw_stone_update,
        "5 s 内必须收到含 (0,-60,0)->STONE 的 BlockUpdates"
    );
    let y = falling_y.expect("5 s 内必须收到落体 EntityStates");
    assert!(y <= -57.0, "落体根位置 y 必须 <= -57.0,实得 {y}");
    assert!(
        world.tick() >= 40,
        "阻塞后 world.tick() 必须 >= 40,实得 {}",
        world.tick()
    );
}
