//! 调度器跨线程行为测试(任务书 05 测试清单下半):单调屏障、fail-stop 入口、
//! 命令经一拍前缀进入分片、关停无泄漏。
//!
//! 构造 sim 参照 `crates/physics-sim/tests/shard_sim.rs`:`ShardSimConfig::new`
//! + `SimpleRegistry`,凝固桥关闭(本文件验证调度器,不验证桥)。

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{ShardSim, ShardSimConfig};
use shard::{Scheduler, SchedulerConfig};
use types::glam::Vec3;
use types::{DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, Tick, WorldId};
use voxel::SimpleRegistry;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// 虚空分片,凝固桥关闭(几何/调度测试用)。
fn sim(x: i32, z: i32) -> ShardSim {
    let mut cfg = ShardSimConfig::new(shard_id(x, z));
    cfg.solidify.enabled = false;
    ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
}

fn sched() -> Scheduler {
    Scheduler::new(SchedulerConfig {
        worker_count: 2,
        ..SchedulerConfig::default()
    })
}

fn spawn_input(apply_at: Tick, seq: u64, entity: EntityId, pos: Vec3) -> StampedInput {
    StampedInput {
        apply_at,
        source: InputSource(7),
        sequence: seq,
        payload: InputPayload::Command(SimCommand::SpawnDynamic {
            entity,
            state: EntityState {
                translation: pos,
                ..EntityState::default()
            },
            desc: EntityDesc::default(),
        }),
    }
}

/// 连续 3 次 tick:拍号单调 +1,summary.tick 依次为 0、1、2(决策 7:第 k 次
/// 调用执行第 k-1 拍),tick_now 从 0 走到 3。
#[test]
fn tick_is_monotonic_and_barriered() {
    let mut sched = sched();
    sched.register(sim(0, 0));
    sched.register(sim(1, 0));
    assert_eq!(sched.tick_now(), Tick(0));
    for expected in [0u64, 1, 2] {
        let summary = sched.tick();
        assert_eq!(summary.tick, Tick(expected));
        assert_eq!(summary.stepped_shards, 2);
        assert_eq!(sched.tick_now(), Tick(expected + 1));
    }
    sched.shutdown();
}

#[test]
#[should_panic(expected = "重复注册")]
fn register_twice_panics() {
    let mut sched = sched();
    sched.register(sim(0, 0));
    sched.register(sim(0, 0)); // 同分片第二次 → fail-stop
}

#[test]
#[should_panic(expected = "未注册分片")]
fn submit_to_unknown_shard_panics() {
    let mut sched = sched();
    sched.register(sim(0, 0));
    let e = EntityId::from_parts(1, 1);
    let _ = sched.submit(shard_id(9, 9), spawn_input(Tick(0), 0, e, Vec3::ZERO));
}

#[test]
#[should_panic(expected = "迟到输入")]
fn submit_stale_input_panics() {
    let mut sched = sched();
    sched.register(sim(0, 0));
    sched.tick(); // 拍 0 已分发,当前拍 = 1
    let e = EntityId::from_parts(1, 1);
    // apply_at = 0 < 1:该拍已分发,调用方 bug → fail-stop
    let _ = sched.submit(shard_id(0, 0), spawn_input(Tick(0), 1, e, Vec3::ZERO));
}

/// 命令 → 收件箱 → 下一拍前缀进分片 → 虚空下落,位置随拍变化。
#[test]
fn entity_spawned_via_command_is_steppable() {
    let mut sched = sched();
    let a = shard_id(0, 0);
    sched.register(sim(0, 0));
    let e = EntityId::from_parts(1, 1);
    // 下一拍(拍 1)生效的出生命令。
    sched
        .submit(a, spawn_input(Tick(1), 0, e, Vec3::new(10.0, 60.0, 10.0)))
        .expect("收件箱未满");

    // 拍 0:命令未到期;拍 1:前缀应用,实体出现。
    sched.tick();
    sched.tick();
    let before = sched.entity_state(a, e).expect("拍 1 前缀已应用,实体应在");

    for _ in 0..30 {
        sched.tick();
    }
    let after = sched.entity_state(a, e).expect("虚空无阻碍,实体仍在");
    assert!(
        after.translation.y < before.translation.y - 0.5,
        "重力作用下落:before.y={} after.y={}",
        before.translation.y,
        after.translation.y
    );
    sched.shutdown();
}

/// tick 后正常 shutdown:函数返回即全部 worker 已 join,进程无挂起线程。
#[test]
fn shutdown_joins_all_workers() {
    let mut sched = sched();
    sched.register(sim(0, 0));
    sched.register(sim(1, 0));
    sched.register(sim(0, 1));
    for _ in 0..3 {
        sched.tick();
    }
    sched.shutdown(); // 返回 = 全部 join 完成
}
