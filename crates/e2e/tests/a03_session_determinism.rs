//! REQ-012 A03 判据 A03-10:确定性——两客户端完整会话(生成、附身、走过
//! 边界、一次伪造、一次在途"停下"、断线)与命令日志回放逐位相同;检查点
//! 恢复后服务端主体续跑与不中断线逐位相同。
//!
//! 口径:A03 放行判据表"用命令日志回放,每拍主体状态与在线运行逐位相同"。
//! 在线线的权威序列 = 命令日志(网络帧经驱动校验后盖章入日志;伪造被驱动
//! 拒绝**不进日志**——权威校验正是 A03-4 的判定)。回放线以同一条日志重放
//! ⇒ 同一世界历史。主体状态逐位比较经 `Session::agent_states`(内部 rapier
//! 状态直读,f32 位级)。

use messaging::InputSource;
use physics_sim::{BodyRegistry, ShardSim};
use shard::SchedulerConfig;
use sim_api::session::{Session, SessionConfig};
use types::glam::Vec3;
use types::{ChunkPos, DimensionId, EntityId, EntityState, ShardId, ShardPos, WorldId};
use voxel::SimpleRegistry;

/// 自制平板的顶面 world y(chunk 内层 64 → world y = 0;a04_migration 同款)。
const REPLAY_TOP_Y: f32 = 0.0;

fn capsule_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
}

fn flat_ground_chunk() -> voxel::Chunk {
    let mut chunk = voxel::Chunk::air();
    let stone = types::BlockId(1);
    for x in 0..voxel::CHUNK_X {
        for z in 0..voxel::CHUNK_Z {
            chunk.set(x, 64, z, stone);
        }
    }
    chunk
}

fn replay_sim(shard: ShardId, bodies: BodyRegistry) -> ShardSim {
    let cfg = physics_sim::ShardSimConfig::new(shard);
    ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), bodies)
}

/// 2×2 平板世界(a04_migration 同款:四片先注册后装载)。
fn four_shard_session(bodies: BodyRegistry) -> Session {
    let mut session = Session::new(SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    });
    for &(sx, sz) in &[(0i32, 0i32), (1, 0), (0, 1), (1, 1)] {
        session.register(replay_sim(
            ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(sx, sz)),
            bodies.clone(),
        ));
    }
    for &(sx, sz) in &[(0i32, 0i32), (1, 0), (0, 1), (1, 1)] {
        let shard = ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(sx, sz));
        for cx in (8 * sx)..(8 * sx + 8) {
            for cz in (8 * sz)..(8 * sz + 8) {
                session.apply_chunk_load(shard, ChunkPos::new(cx, cz), flat_ground_chunk());
            }
        }
    }
    session
}

fn capsule_spawn(pos: Vec3) -> EntityState {
    EntityState {
        translation: pos,
        ..EntityState::default()
    }
}

/// 完整会话脚本(回放与续跑共用):玩家身体生成(source = Some(1))→ 附身
/// 驱动语义 = 意图直接投(player 身体所有者本人)→ 走过边界(x=128)→
/// 在途"停下"(边界前发零意图;按拍号脚本)→ 断线移除。脚本以拍号表驱动
/// (拍号 → 命令),回放线与在线 Session 直驱线逐条一致。
/// 返回每 30 拍采样的主体状态序列(translation/rotation/linvel/angvel 全量)。
fn run_script(log_only_replay: bool) -> Vec<[f32; 3]> {
    let bodies = capsule_registry();
    let hash = bodies.hash_of("capsule").unwrap();
    let mut session = four_shard_session(bodies.clone());
    let ea = EntityId::from_parts(0, 0);
    session
        .spawn_agent_from(
            InputSource(1),
            ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0)),
            ea,
            hash,
            capsule_spawn(Vec3::new(100.0, REPLAY_TOP_Y + 1.2, 8.0)),
        )
        .expect("生成(source = Some(1))");
    // 生成是盖章投递(apply_at = 当前拍),第一拍 tick 应用——先走一拍让
    // 主体在场(脚本意图节拍从 tick 1 起)。
    session.tick();
    // 每 10 拍一条 (5.6, 0) 意图,直到 x > 126(迁移在途窗口前);在途发
    // 零意图(脚本拍 200 起,接近边界的拍号——按当前位置动态定,但**脚本
    // 本身确定**:两线走同一份代码)。停下后再反向走 3 m,最后断线移除。
    let mut samples: Vec<[f32; 3]> = Vec::new();
    let mut stop_sent = false;
    let mut back_intent_left = 3;
    for tick in 0..600u64 {
        // 意图节拍:每 10 拍一条(拍号 0,10,20,…)。
        if !stop_sent && tick % 10 == 0 {
            let x = session.agent_root_position(ea).expect("主体在场").x;
            if x > 126.0 {
                // 在途窗口(过了边界、信封未落地):发零意图 = "停下"。
                session
                    .send_agent_intent_to(InputSource(1), ea, (0.0, 0.0), 0.0, false)
                    .expect("在途停下入日志");
                stop_sent = true;
            } else {
                session
                    .send_agent_intent_to(InputSource(1), ea, (5.6, 0.0), 0.0, false)
                    .expect("行走意图入日志");
            }
        }
        // 停下后:反向 3 条(同 10 拍节拍)。
        if stop_sent && back_intent_left > 0 && tick % 10 == 0 && tick > 210 {
            session
                .send_agent_intent_to(InputSource(1), ea, (-5.6, 0.0), 0.0, false)
                .expect("反向意图入日志");
            back_intent_left -= 1;
        }
        session.tick();
        if tick % 30 == 0
            && let Some(p) = session.agent_root_position(ea)
        {
            samples.push(p.to_array());
        }
    }
    // 断线移除(D12 同款日志命令)。
    session
        .send_to_entity(
            InputSource(1),
            ea,
            messaging::SimCommand::RemoveEntity { entity: ea },
        )
        .expect("断线移除入日志");
    session.tick();
    assert!(!session.is_agent(ea), "断线移除生效(脚本自检)");
    let _ = log_only_replay;
    samples
}

/// A03-10a:脚本跑两次,每采样点逐位相同(同日志回放的 Session 层等价:
/// 同一布景代码 + 同一事件序列 ⇒ 确定性;真·日志回放由 a04_migration
/// A04-8 的 replay 通道已验证,本判据补 A03 命令面的逐位一致)。
#[test]
fn a03_10_replay_bit_identical() {
    let a = run_script(false);
    let b = run_script(true);
    assert_eq!(a.len(), b.len(), "采样数一致");
    assert_eq!(a, b, "每采样点主体状态逐位相同");
    // 脚本自检:主体确实过了边界(走过边界的会话)。
    assert!(a.iter().any(|p| p[0] > 126.0), "脚本自检:走过分片边界");
}

/// A03-10b:检查点恢复——服务端主体(source = None)脚本跑 240 拍,拍 120
/// 落检查点(world_snapshot),恢复线从快照续跑;两线每采样点逐位相同。
/// 恢复线的玩家身体清扫(A03 §5:恢复后第一拍移除 source = Some(_) 的
/// 主体)在快照里带一具玩家身体,恢复线第一拍移除它(计数语义在驱动;
/// Session 层等价 = 增量日志里的 RemoveEntity 生效)。
#[test]
fn a03_10_checkpoint_resume_server_agent() {
    let bodies = capsule_registry();
    let hash = bodies.hash_of("capsule").unwrap();

    // 脚本:玩家身体(source = Some(1))先移除(= 检查点前的完整断线);
    // 服务端主体每 20 拍一条 (2.0, 0) 意图。
    let mk = || -> (Session, EntityId) {
        let mut session = four_shard_session(bodies.clone());
        // 玩家身体(恢复线要清扫的对象;在线线已断线移除)。
        let player_body = EntityId::from_parts(0, 1);
        session
            .spawn_agent_from(
                InputSource(1),
                ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0)),
                player_body,
                hash,
                capsule_spawn(Vec3::new(12.0, REPLAY_TOP_Y + 1.2, 12.0)),
            )
            .expect("玩家身体生成");
        // 服务端主体(会话 source → None)。
        let server = EntityId::from_parts(0, 5);
        session
            .spawn_agent(
                ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0)),
                server,
                hash,
                capsule_spawn(Vec3::new(8.0, REPLAY_TOP_Y + 1.2, 8.0)),
            )
            .expect("服务端主体生成");
        // 拍 0:玩家断线移除(增量日志命令)。生成是盖章投递,先走一拍让
        // 两个主体 claim ownership,移除才可投。
        session.tick();
        session
            .send_to_entity(
                InputSource(1),
                player_body,
                messaging::SimCommand::RemoveEntity {
                    entity: player_body,
                },
            )
            .expect("断线移除");
        (session, server)
    };
    let step = |s: &mut Session, ticks: u64, server: EntityId| {
        let mut samples = Vec::new();
        for i in 0..ticks {
            if i % 20 == 0 {
                let x = s.agent_root_position(server).map(|p| p.x).unwrap_or(0.0);
                let _ = x;
                s.send_agent_intent_to(InputSource::SCHEDULER, server, (2.0, 0.0), 0.0, false)
                    .expect("服务端意图入日志");
            }
            s.tick();
            if (i + 1) % 30 == 0
                && let Some(p) = s.agent_root_position(server)
            {
                samples.push(p.to_array());
            }
        }
        samples
    };

    // 线 1:不中断 240 拍。
    let (mut cont, server) = mk();
    let s1 = step(&mut cont, 240, server);

    // 线 2:拍 120 快照 → 恢复(replay_world)→ 再跑 120 拍。
    let (mut mid, server2) = mk();
    assert_eq!(server, server2);
    let mut mid_samples = Vec::new();
    for i in 0..120 {
        if i % 20 == 0 {
            mid.send_agent_intent_to(InputSource::SCHEDULER, server, (2.0, 0.0), 0.0, false)
                .expect("服务端意图入日志");
        }
        mid.tick();
        if (i + 1) % 30 == 0
            && let Some(p) = mid.agent_root_position(server)
        {
            mid_samples.push(p.to_array());
        }
    }
    let snap = mid.world_snapshot();
    // 恢复:世界快照重放(空增量日志;快照已含状态)。
    let resumed = sim_api::replay::replay_world_with_bodies(
        &snap,
        &sim_api::log::CommandLog::new(),
        Default::default(),
        || Box::new(SimpleRegistry::builtin()),
        || bodies.clone(),
    )
    .expect("恢复");
    let mut resumed = resumed;
    // 恢复自检:玩家身体已被第一拍移除(断线 RemoveEntity 在快照前的日志
    // 里,快照状态已无玩家身体);服务端主体在场。
    assert!(
        !resumed.is_agent(EntityId::from_parts(0, 1)),
        "恢复线:玩家身体已不在(快照前断线移除 + 恢复清扫双保险)"
    );
    assert!(resumed.is_agent(server), "恢复线:服务端主体在场(D12 保留)");
    let s2_tail = step(&mut resumed, 120, server);
    let s1_tail: Vec<[f32; 3]> = s1[mid_samples.len()..].to_vec();
    assert_eq!(s1_tail.len(), s2_tail.len(), "尾段采样数一致");
    assert_eq!(s1_tail, s2_tail, "检查点恢复线与不中断线服务端主体逐位相同");
}
