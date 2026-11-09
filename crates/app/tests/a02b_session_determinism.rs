//! REQ-012 A02b(#25):判据 A02-9 的 Session 层半边——一条时间线包含行走、
//! 跳跃、被砸,**意图全部走 `AgentIntent` 输入日志**(`Session::agent_intent`
//! 本地受信通道正路,盖章入外部命令日志);`replay_world_with_bodies` 回放
//! 两次,每拍主体状态逐位相同。
//!
//! 内核检查点半边(行走中/空中/控制拍之间三检查点恢复)在
//! `physics_sim::agent::tests::a02_9_checkpoint_resume_bitwise_identical_at_three_positions`。

use messaging::SimCommand;
use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
use shard::SchedulerConfig;
use sim_api::log::CommandLog;
use sim_api::replay::replay_world_with_bodies;
use sim_api::session::{Session, SessionConfig};
use types::glam::Vec3;
use types::{
    BlockId, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, Tick,
    WorldId,
};
use voxel::{Chunk, SimpleRegistry};

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
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

fn capsule_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
}

/// 3×3 chunk 平板地形(层 world y = 0 全石)。
fn load_flat_ground(session: &mut Session, shard: ShardId) {
    let stone = BlockId(1);
    for cx in 0..3i32 {
        for cz in 0..3i32 {
            let mut chunk = Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    chunk.set(x, 64, z, stone);
                }
            }
            session.apply_chunk_load(shard, ChunkPos::new(cx, cz), chunk);
        }
    }
}

/// 带默认配置(凝固开启 = 生产)+ capsule 注册表的分片。
fn agent_sim() -> ShardSim {
    let cfg = ShardSimConfig::new(shard_id(0, 0));
    ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), capsule_registry())
}

/// A02-9 时间线(900 拍,主体在平板内往返,同内核半边的布景):
/// 拍 0 spawn;拍 1 意图 (5.6, 0)(行走);拍 121 = 行走中(与内核检查点 1
/// 同拍);拍 150 折返;拍 360 意图 (5.6, 0) + jump = true(移动中跳,空中
/// 段与内核检查点 2 同窗);拍 440 意图清零刹停;拍 500 方块砸下(位置读
/// 当拍实际状态,恢复线读到的逐位一致);拍 700 爆炸(爆心跟随主体)。
/// 全部意图走 [`Session::agent_intent`],LogEntry 逐条 append 进外部命令日志。
fn run_timeline(session: &mut Session, log: &mut CommandLog) -> Vec<EntityState> {
    let shard = shard_id(0, 0);
    let a = EntityId::from_parts(1, 1);
    let hash = capsule_registry().hash_of("capsule").expect("capsule 已装");
    let mut trace = Vec::with_capacity(900);
    for tick in 0..900u32 {
        match tick {
            0 => {
                let entry = session
                    .spawn_agent(
                        shard,
                        a,
                        hash,
                        EntityState {
                            translation: Vec3::new(24.0, 1.95, 24.0),
                            ..EntityState::default()
                        },
                    )
                    .expect("收件箱未满");
                log.append(entry);
            }
            1 => {
                let entry = session
                    .agent_intent(shard, a, (5.6, 0.0), 0.0, false)
                    .expect("收件箱未满");
                log.append(entry);
            }
            150 => {
                let entry = session
                    .agent_intent(shard, a, (-5.6, 0.0), 0.0, false)
                    .expect("收件箱未满");
                log.append(entry);
            }
            360 => {
                let entry = session
                    .agent_intent(shard, a, (5.6, 0.0), 0.0, true)
                    .expect("收件箱未满");
                log.append(entry);
            }
            440 => {
                let entry = session
                    .agent_intent(shard, a, (0.0, 0.0), 0.0, false)
                    .expect("收件箱未满");
                log.append(entry);
            }
            500 => {
                // 砸块:主体正上方(ac_a6 高度口径;位置读当拍实际状态——
                // 回放线在检查点前逐位复现,读到同一位置)。
                let st = session
                    .entity_state(shard, a)
                    .expect("拍 500 主体在(两拍之间查询,拍内命令已生效)");
                let entry = session
                    .send(
                        shard,
                        SimCommand::SpawnDynamic {
                            entity: EntityId::from_parts(1, 2),
                            state: EntityState {
                                translation: Vec3::new(st.translation.x, 6.3, st.translation.z),
                                ..EntityState::default()
                            },
                            desc: EntityDesc::default(),
                        },
                    )
                    .expect("收件箱未满");
                log.append(entry);
            }
            700 => {
                let st = session.entity_state(shard, a).expect("拍 700 主体在");
                let entries = session
                    .detonate(
                        Vec3::new(st.translation.x - 2.5, 1.0, st.translation.z),
                        12.0,
                        5.0,
                    )
                    .expect("爆炸入箱");
                entries.iter().for_each(|e| log.append(e.clone()));
            }
            _ => {}
        }
        session.tick();
        trace.push(
            session
                .entity_state(shard, a)
                .expect("主体全程在(布景约束:平板内往返)"),
        );
    }
    trace
}

/// A02-9(Session 层):时间线跑一遍 → 回放两次,每拍主体状态逐位相同;
/// 日志确实含 AgentIntent 条目(意图全走输入日志)。
#[test]
fn session_agent_intent_timeline_replay_is_bitwise_identical() {
    let shard = shard_id(0, 0);
    let a = EntityId::from_parts(1, 1);

    // ---- 原线 ----
    let mut session = Session::new(session_cfg());
    session.register(agent_sim());
    load_flat_ground(&mut session, shard);
    // 快照取在时间线开始前(拍 0):日志全部落在重放窗口内被原章重放。
    let snapshot = session.world_snapshot();
    let mut log = CommandLog::new();
    let trace = run_timeline(&mut session, &mut log);
    session.shutdown();

    // 日志自检:4 条 AgentIntent(拍 1/150/360/440),全部会话来源。
    let intent_entries: Vec<_> = log
        .entries()
        .iter()
        .filter(|e| matches!(e.command, SimCommand::AgentIntent { .. }))
        .collect();
    assert_eq!(
        intent_entries.len(),
        4,
        "日志恰有 4 条 AgentIntent(意图全部走输入日志)"
    );
    assert_eq!(trace.len(), 900);
    // 布景自检:行走段真实发生(拍 121 速度 ≈ 5.6,与内核检查点 1 同拍)。
    assert!(
        trace[121].linvel.x > 5.0,
        "拍 121 行走中(vx = {})",
        trace[121].linvel.x
    );
    // 布景自检:跳跃真实发生(360 拍起跳后窗口内 vy > 1.5)。
    assert!(
        (360..400).any(|t| trace[t as usize].linvel.y > 1.5),
        "360 拍起跳后出现空中拍"
    );

    // ---- 回放两次(replay_world_with_bodies;注册表每分片重新注入)----
    let new_registry = || -> Box<dyn voxel::BlockRegistry> { Box::new(SimpleRegistry::builtin()) };
    let new_bodies = capsule_registry;
    for replay_no in 1..=2u32 {
        let mut replayed =
            replay_world_with_bodies(&snapshot, &log, session_cfg(), new_registry, new_bodies)
                .expect("世界快照 + 日志恢复");
        let mut replay_trace = Vec::with_capacity(900);
        for _ in 0..900u32 {
            replayed.tick();
            replay_trace.push(replayed.entity_state(shard, a).expect("回放线主体全程在"));
        }
        for (t, (x, y)) in trace.iter().zip(replay_trace.iter()).enumerate() {
            assert_eq!(x, y, "回放 #{replay_no}:拍 {t} 主体状态逐位不同");
        }
        assert_eq!(replayed.tick_now(), Tick(900), "回放线与原线同拍数");
        replayed.shutdown();
    }
}
