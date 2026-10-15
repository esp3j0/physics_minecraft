//! 双分片迁移端到端集成测试(任务书 06):命令入箱 → 调度器分拍 → worker 执行 →
//! outbox 迁移意图 → Router 盖章入队 → 下一拍成为目的分片输入前缀 → 实体在邻片
//! 重建继续运动(REQ-001 AC2/AC5 调度器级前哨战);外加全链路确定性(AC1 多线程侧)。
//!
//! 固定场景:2 个 worker;分片 (0,0)(本体区 x∈[0,128))与 (1,0)(x∈[128,256)),
//! 各带地面 chunk;实体出生在 x=110 地面上方,linvel=(60,0,0)≈1 m/tick,
//! SpawnDynamic 命令 apply_at=Tick(1);`tick()` 循环 31 次(执行第 0..30 拍,
//! README 决策 7:第 k 次调用执行第 k-1 拍)。
//!
//! 构造与断言风格照 `crates/physics-sim/tests/shard_sim.rs` 的
//! `entity_migrates_across_boundary` adapt。

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{ShardSim, ShardSimConfig};
use shard::{Scheduler, SchedulerConfig, TickSummary};
use types::glam::Vec3;
use types::{
    BlockId, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, Tick,
    WorldId,
};
use voxel::{Chunk, SimpleRegistry};

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn entity(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

/// 顶层实心面在 world y = top_ly - 64 的地形 chunk(照 shard_sim.rs 帮助函数)。
fn ground_chunk(top_ly: usize) -> Chunk {
    let mut c = Chunk::air();
    for y in 0..top_ly {
        for x in 0..16 {
            for z in 0..16 {
                c.set(x, y, z, BlockId(1));
            }
        }
    }
    c
}

/// 带地面 chunk 的分片:直接构造 ShardSim 后 load_chunk(再由 Scheduler::register 接管)。
/// chunk 全高 [-64,319],top_ly=200 → 顶面 world y = 136。
fn ground_sim(shard_x: i32, chunk_xs: [i32; 2]) -> ShardSim {
    let mut s = ShardSim::new(
        ShardSimConfig::new(shard_id(shard_x, 0)),
        Box::new(SimpleRegistry::builtin()),
    );
    for cx in chunk_xs {
        s.load_chunk(ChunkPos::new(cx, 0), ground_chunk(200));
    }
    s
}

/// 出生命令:出生点 x=110(分片 (0,0) 本体区内)、地面顶面(136)上方 10 m,
/// 纯水平 60 m/s(dt=1/60 → ≈1 m/tick);飞行段不接触地形(31 拍仅下落 ~1.3 m)。
fn spawn_input(apply_at: Tick, seq: u64, id: EntityId) -> StampedInput {
    StampedInput {
        apply_at,
        source: InputSource(7),
        sequence: seq,
        payload: InputPayload::Command(SimCommand::SpawnDynamic {
            entity: id,
            state: EntityState {
                translation: Vec3::new(110.0, 146.0, 10.0),
                linvel: Vec3::new(60.0, 0.0, 0.0),
                angvel: Vec3::ZERO,
                ..EntityState::default()
            },
            desc: EntityDesc::default(),
        }),
    }
}

/// 场景固定拍数:执行第 0..30 拍。
const TICKS: usize = 31;

/// 一次完整场景的逐拍记录(实体从哪片消失/在哪片出现靠逐拍采样判定)。
struct RunResult {
    /// 每拍 TickSummary(summaries[k] = 第 k 拍)。
    summaries: Vec<TickSummary>,
    /// src_at[k] = 第 k 拍执行完后源片 (0,0) 里的实体状态。
    src_at: Vec<Option<EntityState>>,
    /// dst_at[k] = 第 k 拍执行完后目的片 (1,0) 里的实体状态。
    dst_at: Vec<Option<EntityState>>,
    /// 场景跑完后的下一拍号。
    tick_now: Tick,
}

/// 独立跑一遍固定场景。两个 Scheduler 之间不得有共享可变状态:每次调用
/// 从零构造 Scheduler 与两个 ShardSim,跑完 shutdown 归还线程。
fn run_scenario() -> RunResult {
    let (src, dst) = (shard_id(0, 0), shard_id(1, 0));
    let mut sched = Scheduler::new(SchedulerConfig {
        worker_count: 2,
        ..SchedulerConfig::default()
    });
    // 地面:分片 (0,0) 带 chunk x∈{6,7},分片 (1,0) 带 chunk x∈{8,9}
    // (覆盖飞行走廊 x∈[96,160),z=0 行);先 load_chunk 再 register。
    sched.register(ground_sim(0, [6, 7]));
    sched.register(ground_sim(1, [8, 9]));

    // 出生命令在第 0 拍入箱,apply_at = Tick(1)(第 1 拍前缀生效)。
    let e = entity(1);
    sched
        .submit(src, spawn_input(Tick(1), 0, e))
        .expect("收件箱未满");

    let mut summaries = Vec::with_capacity(TICKS);
    let mut src_at = Vec::with_capacity(TICKS);
    let mut dst_at = Vec::with_capacity(TICKS);
    for _ in 0..TICKS {
        summaries.push(sched.tick());
        src_at.push(sched.entity_state(src, e));
        dst_at.push(sched.entity_state(dst, e));
    }
    let tick_now = sched.tick_now();
    sched.shutdown();
    RunResult {
        summaries,
        src_at,
        dst_at,
        tick_now,
    }
}

/// 端到端主链路:命令 → 分拍 → worker → outbox → 盖章 → 前缀 → 邻片重建。
/// 结束时:源片已无实体;目的片实体 x ∈ (128,160) 且继续以 ≈60 m/s 运动;共 31 拍。
#[test]
fn entity_migrates_across_scheduler_boundary() {
    let r = run_scenario();
    assert!(r.src_at[TICKS - 1].is_none(), "实体已离开源片(0,0)");
    let dst = r.dst_at[TICKS - 1].expect("实体已在目的片(1,0)重建");
    assert!(
        dst.translation.x > 128.0 && dst.translation.x < 160.0,
        "越界后继续 +x 运动:x={}",
        dst.translation.x
    );
    assert!(
        (dst.linvel.x - 60.0).abs() < 2.0,
        "迁移动量近似保持:linvel.x={}",
        dst.linvel.x
    );
    assert_eq!(r.tick_now, Tick(31));
}

/// 信封前缀链路:migrations_admitted 恰在过界拍为 1、其余拍为 0;
/// 实体从源片消失的拍与目的片出现的拍相差恰 1(apply_at ≡ produced_at + 1)。
#[test]
fn migration_flows_through_envelope_prefix() {
    let r = run_scenario();
    let admitted: Vec<usize> = r.summaries.iter().map(|s| s.migrations_admitted).collect();
    let admitted_ticks: Vec<u64> = r
        .summaries
        .iter()
        .filter(|s| s.migrations_admitted > 0)
        .map(|s| s.tick.0)
        .collect();
    assert!(
        admitted.iter().all(|&n| n <= 1),
        "场景只有一个实体:migrations_admitted 只会是 0 或 1:{admitted:?}"
    );
    assert_eq!(admitted_ticks.len(), 1, "恰有一拍盖章入队:{admitted:?}");
    let produced = admitted_ticks[0];

    // 源片 Some→None 的转换拍 = 消失拍;目的片首次 Some 的拍 = 出现拍。
    let vanish = r
        .src_at
        .windows(2)
        .position(|w| w[0].is_some() && w[1].is_none())
        .expect("实体必须先从源片消失") as u64
        + 1;
    let appear = r
        .dst_at
        .iter()
        .position(|s| s.is_some())
        .expect("实体必须随后在目的片出现") as u64;

    assert_eq!(
        produced, vanish,
        "迁移意图在源片删除实体的当拍盖章入队:produced={produced} vanish={vanish}"
    );
    assert_eq!(
        appear,
        produced + 1,
        "信封在 produced+1 拍成为目的片前缀:appear={appear} produced={produced}"
    );
    assert!(
        r.dst_at[appear as usize..].iter().all(Option::is_some),
        "重建后留在目的片(场景内不再越界)"
    );
}

/// 全链路确定性(AC1 多线程侧):同场景两个独立 Scheduler、同输入同拍数,
/// 每拍 TickSummary 序列相等,终态 entity_state 位级相等(==,不许近似)。
#[test]
fn full_pipeline_is_deterministic() {
    let a = run_scenario();
    let b = run_scenario();
    assert_eq!(a.summaries, b.summaries, "每拍 TickSummary 序列位级相等");
    assert_eq!(a.src_at, b.src_at, "每拍源片状态序列位级相等");
    assert_eq!(a.dst_at, b.dst_at, "每拍目的片状态序列位级相等");
    assert_eq!(a.tick_now, b.tick_now);
    // 非退化保障:确定性比较的意义建立在场景确实发生迁移之上。
    assert!(a.dst_at[TICKS - 1].is_some(), "场景必须发生迁移");
}
