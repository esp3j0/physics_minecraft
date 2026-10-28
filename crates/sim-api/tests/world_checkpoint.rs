//! REQ-005 任务书 02:全局快照(checkpoint)与 replay_world 场景测试。
//!
//! 覆盖任务书测试清单中需要完整会话闭环(观察者/迁移信封/生命周期)的
//! 三条:pending 迁移信封存活、snapshot 拍命令回放、网络观察者命令
//! (Move/Remove)入日志重放。长程 600 拍对拍在 `long_replay.rs`。

use messaging::InputSource;
use messaging::SimCommand;
use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
use shard::{SchedulerConfig, TickSummary};
use sim_api::log::CommandLog;
use sim_api::replay::replay_world;
use sim_api::session::{Session, SessionConfig};
use sim_api::snapshot::WorldSnapshot;
use types::glam::Vec3;
use types::{DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, Tick, WorldId};
use voxel::{BlockRegistry, Chunk, SimpleRegistry};

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn entity(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

/// 凝固桥关闭的虚空分片(调度器/sim-api 测试同款)。
///
/// REQ-010 §12.1 第 3 条迁移:本文件用例按 REQ-009 及更早语义编写(悬空
/// SetBlock 保持原样),与默认开启的承载判定冲突 → 装配处显式关闭本开关,
/// 断言不变。两个构造器共用。
fn cohesion_off(mut cfg: ShardSimConfig) -> ShardSimConfig {
    cfg.cohesion.enabled = false;
    cfg
}

fn sim(x: i32, z: i32) -> ShardSim {
    let mut cfg = cohesion_off(ShardSimConfig::new(shard_id(x, z)));
    cfg.solidify = SolidifyConfig {
        enabled: false,
        ..SolidifyConfig::default()
    };
    ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
}

/// 带地面 chunk 的分片(顶面 world y = 136;long_replay.rs 同款口径)。
fn ground_sim(x: i32, chunk_xs: std::ops::Range<i32>) -> ShardSim {
    let mut cfg = cohesion_off(ShardSimConfig::new(shard_id(x, 0)));
    cfg.solidify = SolidifyConfig {
        enabled: false,
        ..SolidifyConfig::default()
    };
    let mut s = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
    for cx in chunk_xs {
        let mut c = Chunk::air();
        for y in 0..200 {
            for lx in 0..16 {
                for lz in 0..16 {
                    c.set(lx, y, lz, types::BlockId(1));
                }
            }
        }
        s.load_chunk(types::ChunkPos::new(cx, 0), c);
    }
    s
}

fn session_cfg() -> SessionConfig {
    SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    }
}

fn registry_factory() -> Box<dyn BlockRegistry> {
    Box::new(SimpleRegistry::builtin())
}

fn spawn_command(id: EntityId, translation: Vec3, linvel: Vec3) -> SimCommand {
    SimCommand::SpawnDynamic {
        entity: id,
        state: EntityState {
            translation,
            linvel,
            angvel: Vec3::ZERO,
            ..EntityState::default()
        },
        desc: EntityDesc::default(),
    }
}

/// 任务书测试 `checkpoint_pending_migration_entity_survives`:tick N 末已产生
/// 迁移信封(apply_at = N+1,实体已从源片删除、尚未到目的片)时 checkpoint;
/// replay_world 到 N+2 实体在目的片(信封经 Router 快照存活,实体不丢失——
/// AC-C5 的"待迁移信封"窗口)。
#[test]
fn checkpoint_pending_migration_entity_survives() {
    let (src, dst) = (shard_id(0, 0), shard_id(1, 0));
    let mut session = Session::new(session_cfg());
    session.register(ground_sim(0, 0..9));
    session.register(ground_sim(1, 7..16));

    // 拍 0:高速实体(linvel 240 m/s ≈ 4 m/拍,出生 130 → 拍 0 内越过 x=128)。
    let e = entity(1);
    let entry = session.send(
        src,
        spawn_command(e, Vec3::new(130.0, 137.5, 8.0), Vec3::new(240.0, 0.0, 0.0)),
    );
    let mut log = CommandLog::new();
    log.append(entry.expect("收件箱未满"));

    // 拍 0 执行:迁移检测触发,实体从源片删除、信封 produced_at=0 →
    // apply_at=1(尚未投递——正好是要测的窗口:tick_now 已是 1)。
    let summary: TickSummary = session.tick();
    assert_eq!(summary.tick, Tick(0));
    assert!(
        session.entity_state(src, e).is_none(),
        "前置:实体已从源片删除(拍 0 末)"
    );
    let world_at_n: WorldSnapshot = session.world_snapshot();
    let snapshot_tick = world_at_n.shards.tick;
    assert_eq!(
        snapshot_tick,
        Tick(1),
        "checkpoint 落在拍 0 后(下一待执行拍 1)"
    );
    assert_eq!(
        world_at_n.scheduler.router.envelopes.len(),
        1,
        "前置:待投递迁移信封在队(apply_at = 1 == snapshot.tick,要测的窗口)"
    );

    // 重放到 snapshot_tick+1:实体在目的片(信封经 Router 快照存活,
    // 在 == snapshot.tick 的拍投递——实体不丢)。
    let mut replayed =
        replay_world(&world_at_n, &log, session_cfg(), registry_factory).expect("自有快照必可恢复");
    assert_eq!(replayed.tick_now(), snapshot_tick, "自快照拍对齐");
    replayed.tick(); // 拍 1(== 快照拍):信封投递,实体在目的片重建
    assert!(
        replayed.entity_state(dst, e).is_some(),
        "拍 1:实体经存活的迁移信封在目的片重建"
    );
    assert_eq!(
        replayed.entity_owner(e).map(|o| o.shard),
        Some(dst),
        "REQ-006 任务 03:restore 后 ownership 重建,owner=目的片"
    );
    replayed.tick(); // 拍 2:实体继续存活(不再迁回)
    let at_next = replayed
        .entity_state(dst, e)
        .or_else(|| replayed.entity_state(shard_id(2, 0), e));
    assert!(
        at_next.is_some(),
        "拍 2:实体仍在世界(源片已删除的实体不因 checkpoint 丢失)"
    );
    replayed.shutdown();
    session.shutdown();
}

/// 任务书测试 `checkpoint_command_at_snapshot_tick_replayed`:`entry.tick ==
/// snapshot.tick` 的命令(checkpoint 后、下一拍前到达)在 replay_world 后
/// 必须被应用,不丢(细则 7:>= 语义)。
#[test]
fn checkpoint_command_at_snapshot_tick_replayed() {
    let shard = shard_id(0, 0);
    let mut session = Session::new(session_cfg());
    session.register(ground_sim(0, 0..4));
    for _ in 0..3 {
        session.tick();
    }
    // checkpoint 在拍 3(下一待执行拍);随后、tick 3 执行**之前**到达的命令
    // 盖章 == 拍 3(生产时序:write_checkpoint 与命令同在拍间窗口,命令可能
    // 晚于 checkpoint 写盘)。
    let world = session.world_snapshot();
    let pos = types::BlockPos::new(4, 137, 4);
    let entry = session
        .send(
            shard,
            SimCommand::SetBlock {
                pos,
                block: types::BlockId(2),
            },
        )
        .expect("收件箱未满");
    assert_eq!(entry.tick, world.shards.tick, "命令盖 == 快照拍");
    let mut log = CommandLog::new();
    log.append(entry);
    session.tick(); // 原线拍 3:命令生效
    assert_eq!(
        session
            .voxel_chunk(shard, types::ChunkPos::new(0, 0))
            .expect("已装载")
            .get(4, 201, 4),
        types::BlockId(2),
        "原线:命令在拍 3 生效"
    );
    session.shutdown();

    // 重放线:旧 `>` 过滤会把该条目当"已进快照"丢掉 → 块缺失(数据丢失);
    // `>=` 后必须同样生效。
    let mut replayed =
        replay_world(&world, &log, session_cfg(), registry_factory).expect("自有快照必可恢复");
    assert_eq!(replayed.tick_now(), Tick(3));
    replayed.tick(); // 拍 3:== 快照拍的命令应用
    assert_eq!(
        replayed
            .voxel_chunk(shard, types::ChunkPos::new(0, 0))
            .expect("已装载")
            .get(4, 201, 4),
        types::BlockId(2),
        "重放线:== snapshot.tick 的命令被应用,不丢(>= 语义)"
    );
    replayed.shutdown();
}

/// 任务书测试 `network_move_observer_logged_and_replayed`:玩家 source 的
/// `move_observer_from` 产生日志(source = 玩家);replay_world 后观察者表
/// 一致(source/sequence 原章再生)。
#[test]
fn network_move_observer_logged_and_replayed() {
    let mut session = Session::new(session_cfg());
    session.register(sim(0, 0));
    let (p1, p2) = (Vec3::new(8.0, 3.0, 8.0), Vec3::new(40.0, 3.0, 40.0));
    let player = InputSource(7);

    // 拍 0 前置:会话观察者入表(老玩家)。
    let e0 = session.move_observer(p1).expect("unbounded");
    session.tick();
    // 玩家移动两条(盖 == 拍 1):第一条生效,第二条覆盖(同 source)。
    let world = session.world_snapshot();
    let m1 = session.move_observer_from(player, p2).expect("unbounded");
    let m2 = session.move_observer_from(player, p1).expect("unbounded");
    assert_eq!(m1.source, player, "日志 source = 玩家");
    assert_eq!(m1.tick, world.shards.tick, "== 快照拍(窗口内到达)");
    session.tick(); // 拍 1:玩家命令应用,表 = (7, p1)
    assert_eq!(
        session.observers(),
        vec![(InputSource(1), p1), (player, p1)],
        "原线:玩家观察者入表"
    );
    session.shutdown();

    let mut log = CommandLog::new();
    log.append(e0);
    log.append(m1);
    log.append(m2);
    let mut replayed =
        replay_world(&world, &log, session_cfg(), registry_factory).expect("自有快照必可恢复");
    assert!(
        replayed.observers().contains(&(InputSource(1), p1)),
        "重放前置:快照兜底的会话观察者在表"
    );
    assert!(
        !replayed.observers().contains(&(player, p1)),
        "重放前置:玩家命令尚未 tick 应用"
    );
    replayed.tick(); // 拍 1:玩家命令应用
    assert_eq!(
        replayed.observers(),
        vec![(InputSource(1), p1), (player, p1)],
        "重放后观察者表与原线一致(source 原章直投)"
    );
    replayed.shutdown();
}

/// 任务书测试 `network_remove_observer_logged_and_replayed`:断线移除
/// (`remove_observer_from`)入日志;重放后无幽灵观察者;票撤
/// (despawn 候选在缩短窗口下如期出现)。
#[test]
fn network_remove_observer_logged_and_replayed() {
    let shard = shard_id(0, 0);
    let cfg = SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            despawn_after_ticks: 3,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    };
    let mut session = Session::new(cfg.clone());
    session.register(sim(0, 0));
    let player = InputSource(7);
    let pos = Vec3::new(8.0, 3.0, 8.0);

    // 玩家入表(拍 0)→ checkpoint(拍 1)→ 玩家断线移除(盖 == 拍 1)。
    session.move_observer_from(player, pos).expect("unbounded");
    session.tick();
    let world = session.world_snapshot();
    let rm = session.remove_observer_from(player).expect("unbounded");
    assert_eq!(
        rm.command,
        SimCommand::RemoveObserver,
        "日志携带 RemoveObserver 命令"
    );
    assert_eq!(rm.tick, world.shards.tick, "== 快照拍(窗口内到达)");
    session.tick(); // 拍 1:移除应用,表空(无幽灵)
    assert!(
        !session.observers().iter().any(|(s, _)| *s == player),
        "原线:玩家条目已删(无幽灵)"
    );
    // 票撤:空片无票,跑过缩短窗口(3 拍)→ despawn 候选。
    for _ in 0..4 {
        session.tick();
    }
    assert_eq!(
        session.take_despawn_requests(),
        vec![shard],
        "原线:票撤后 despawn 候选出现"
    );
    session.shutdown();

    // 重放:RemoveObserver 原章重投 → 无幽灵;票撤同拍再生。
    // 时序口径:快照(拍 1 前)兜底的玩家条目先入表;tick 拍 1 时
    // RemoveObserver 应用 → 条目消失(与原线同拍)。
    let mut log = CommandLog::new();
    log.append(rm);
    let mut replayed = replay_world(&world, &log, cfg, registry_factory).expect("自有快照必可恢复");
    assert!(
        replayed.observers().iter().any(|(s, _)| *s == player),
        "重放线 tick 前:快照兜底条目在表(移除命令待应用)"
    );
    replayed.tick(); // 拍 1:RemoveObserver 应用
    assert!(
        !replayed.observers().iter().any(|(s, _)| *s == player),
        "重放后无幽灵观察者(RemoveObserver 原章重投)"
    );
    for _ in 0..4 {
        replayed.tick();
    }
    assert_eq!(
        replayed.take_despawn_requests(),
        vec![shard],
        "重放线:票撤同拍再生(移除时序与原线一致)"
    );
    replayed.shutdown();
}
