//! REQ-012 A01 R1-P3(Session 层):`SpawnAgent` 走命令路径的正路测试——
//! `Session::spawn_agent`(本地受信通道,盖章入日志)生成胶囊,按 AC-A6
//! 时间线(方块砸中 + 爆炸)跑 3600 拍,然后 `replay_world_with_bodies`
//! 回放两次,每拍主体状态逐位相同;日志里确实有一条 `SpawnAgent`。
//!
//! 放行原文(A01 第 2 节):SpawnAgent "只走本地受信通道(driver commands /
//! Session)";R1 验收:P3 要求该路径有覆盖(此前 Session::spawn_agent 无
//! 调用方,validate_external_command 一律拒绝 = 生产上无法生成主体)。

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

/// 测试用分片身份(与既有测试同风格)。
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

/// capsule 定义(与 assets/bodies/capsule.body.toml 同源;include 保证一致)。
fn capsule_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
}

/// 3×3 chunk 平板地形(层 world y = 0 全石;与 physics-sim agent 判据同款)。
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

/// AC-A6 时间线(R1-P3c):拍 0 Session::spawn_agent 生成胶囊(正路盖章:
/// apply_at = tick_now、source = 会话 1、sequence 单调)、拍 500 方块实体
/// 砸下、拍 1500 爆炸。每条日志条目 append 进外部命令日志(驱动职责的
/// 测试内等价)。
fn run_timeline(session: &mut Session, log: &mut CommandLog) -> Vec<EntityState> {
    let shard = shard_id(0, 0);
    let a = EntityId::from_parts(1, 1);
    let hash = capsule_registry().hash_of("capsule").expect("capsule 已装");
    let mut trace = Vec::with_capacity(3600);
    for tick in 0..3600u32 {
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
            500 => {
                let entry = session
                    .send(
                        shard,
                        SimCommand::SpawnDynamic {
                            entity: EntityId::from_parts(1, 2),
                            state: EntityState {
                                translation: Vec3::new(24.0, 5.9, 24.0),
                                ..EntityState::default()
                            },
                            desc: EntityDesc::default(),
                        },
                    )
                    .expect("收件箱未满");
                log.append(entry);
            }
            1500 => {
                let entries = session
                    .detonate(Vec3::new(21.5, 1.0, 24.0), 12.0, 5.0)
                    .expect("爆炸入箱");
                entries.iter().for_each(|e| log.append(e.clone()));
            }
            _ => {}
        }
        session.tick();
        trace.push(
            session
                .entity_state(shard, a)
                .expect("主体全程在(布景约束)"),
        );
    }
    trace
}

/// R1-P3:Session::spawn_agent 正路 + 日志含 SpawnAgent + 回放两次每拍主体
/// 状态逐位相同。附带布景自检:主体生成成功、站立段不凝固( Session.tick
/// = 生产节奏:激活刷新 + step + lifecycle + outbox)。
#[test]
fn session_spawn_agent_replay_is_bitwise_identical() {
    let shard = shard_id(0, 0);
    let a = EntityId::from_parts(1, 1);

    // ---- 原线 ----
    let mut session = Session::new(session_cfg());
    session.register(agent_sim());
    load_flat_ground(&mut session, shard);
    // 快照取在时间线开始前(拍 0):重放窗口 >= 快照拍,日志全部落在窗口内
    // 被原章重放(拍 3600 的快照会让重放窗为空 = 空跑,不是回放)。
    let snapshot = session.world_snapshot();
    let mut log = CommandLog::new();
    let trace = run_timeline(&mut session, &mut log);
    session.shutdown();

    // 日志自检:确实有一条 SpawnAgent(拍 0、会话 source、sequence 0)。
    let agent_entries: Vec<_> = log
        .entries()
        .iter()
        .filter(|e| matches!(e.command, SimCommand::SpawnAgent { .. }))
        .collect();
    assert_eq!(agent_entries.len(), 1, "日志恰有一条 SpawnAgent(R1-P3)");
    assert_eq!(
        agent_entries[0].tick,
        Tick::ZERO,
        "盖章 apply_at = tick_now"
    );
    assert_eq!(
        agent_entries[0].source,
        messaging::InputSource(1),
        "会话 source"
    );
    assert!(trace.len() == 3600);
    // 站立段自检:前 60 拍主体位姿几乎不动(生成成功且未凝固移除——凝固会
    // 让 entity_state 变 None,expect 已挡)。
    let p0 = trace[0].translation;
    let p60 = trace[60].translation;
    assert!(
        (p0 - p60).length() < 0.1,
        "站立段主体近乎静止:拍 0 {p0:?} vs 拍 60 {p60:?}"
    );

    // ---- 回放两次(replay_world_with_bodies;注册表每分片重新注入)----
    let new_registry = || -> Box<dyn voxel::BlockRegistry> { Box::new(SimpleRegistry::builtin()) };
    let new_bodies = capsule_registry;
    for replay_no in 1..=2u32 {
        let mut replayed =
            replay_world_with_bodies(&snapshot, &log, session_cfg(), new_registry, new_bodies)
                .expect("世界快照 + 日志恢复");
        let mut replay_trace = Vec::with_capacity(3600);
        for _ in 0..3600u32 {
            replayed.tick();
            replay_trace.push(replayed.entity_state(shard, a).expect("回放线主体全程在"));
        }
        for (t, (x, y)) in trace.iter().zip(replay_trace.iter()).enumerate() {
            assert_eq!(x, y, "回放 #{replay_no}:拍 {t} 主体状态逐位不同");
        }
        replayed.shutdown();
    }
}
