//! REQ-006 任务 04:实体所有权/命令路由的验收测试(AC-E1~E8 的会话级面)。
//!
//! 真 Session/Scheduler,不 mock 被验收层;真 TCP/双客户端权限面由 app/e2e
//! 既有 `network_spawn_*`、`two_clients_spawn_*`、`entity_commands_route_*`
//! 测试承担,本文件聚焦 ownership/迁移/回放/权限校验的可重复断言。

use app::driver::validate_external_command;
use messaging::{InputSource, SimCommand};
use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
use sim_api::log::CommandLog;
use sim_api::replay::replay_world;
use sim_api::session::{Session, SessionConfig};
use types::glam::Vec3;
use types::{DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, WorldId};
use voxel::{BlockRegistry, SimpleRegistry};

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn sim(x: i32, z: i32) -> ShardSim {
    let mut cfg = ShardSimConfig::new(shard_id(x, z));
    cfg.solidify = SolidifyConfig {
        enabled: false,
        ..SolidifyConfig::default()
    };
    ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
}

fn session_cfg() -> SessionConfig {
    SessionConfig {
        scheduler: shard::SchedulerConfig {
            worker_count: 2,
            ..shard::SchedulerConfig::default()
        },
        ..SessionConfig::default()
    }
}

fn registry_factory() -> Box<dyn BlockRegistry> {
    Box::new(SimpleRegistry::builtin())
}

/// 两片会话:源片 (0,0) + 目的片 (1,0),实体从 120 迁到 136。
fn migrated_session() -> (Session, EntityId) {
    let mut session = Session::new(session_cfg());
    session.register(sim(0, 0));
    session.register(sim(1, 0));
    let e = EntityId::from_parts(0, 1);
    let entry = session
        .spawn_owned(
            shard_id(0, 0),
            e,
            EntityState {
                translation: Vec3::new(120.0, 10.0, 8.0),
                ..EntityState::default()
            },
            EntityDesc::default(),
            InputSource(7),
        )
        .expect("spawn");
    assert_eq!(entry.dest, shard_id(0, 0));
    session.tick();
    session
        .send_to_entity(
            InputSource(7),
            e,
            SimCommand::TeleportEntity {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(136.0, 10.0, 8.0),
                    ..EntityState::default()
                },
            },
        )
        .expect("teleport");
    session.tick(); // 源片 despawn,信封 apply_at = 下一拍
    assert!(session.entity_owner(e).is_none(), "迁移窗口 owner=None");
    session.tick(); // 目的片重建
    (session, e)
}

/// AC-E2:迁移窗口内实体命令必须被拒绝,不能投到旧片。
#[test]
fn ace2_commands_rejected_in_migration_window() {
    let mut session = Session::new(session_cfg());
    session.register(sim(0, 0));
    session.register(sim(1, 0));
    let e = EntityId::from_parts(0, 1);
    session
        .spawn_owned(
            shard_id(0, 0),
            e,
            EntityState {
                translation: Vec3::new(120.0, 10.0, 8.0),
                ..EntityState::default()
            },
            EntityDesc::default(),
            InputSource(7),
        )
        .expect("spawn");
    session.tick();
    session
        .send_to_entity(
            InputSource(7),
            e,
            SimCommand::TeleportEntity {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(136.0, 10.0, 8.0),
                    ..EntityState::default()
                },
            },
        )
        .expect("teleport");
    session.tick();
    assert!(session.entity_owner(e).is_none(), "迁移窗口 owner=None");
    let remove = SimCommand::RemoveEntity { entity: e };
    assert!(
        session
            .send_to_entity(InputSource(7), e, remove.clone())
            .is_err(),
        "迁移窗口 send_to_entity 必须拒绝"
    );
    assert!(
        validate_external_command(&session, Some(InputSource(7)), shard_id(0, 0), &remove).is_err(),
        "迁移窗口外部校验必须拒绝,不得投旧片"
    );
    session.tick(); // 目的片重建
    assert_eq!(
        session.entity_owner(e).map(|o| o.shard),
        Some(shard_id(1, 0))
    );
    session.shutdown();
}

/// AC-E1:归属唯一、迁移后 owner=dest、source 保留。
#[test]
fn ace1_ownership_unique_after_spawn_and_migrate() {
    let (session, e) = migrated_session();
    let owner = session.entity_owner(e).expect("目的片 owner");
    assert_eq!(owner.shard, shard_id(1, 0));
    assert_eq!(owner.source, Some(InputSource(7)), "source 随迁移保留");
    assert!(!owner.actor);
    session.shutdown();
}

/// AC-E2:迁移后 Remove 按 owner 路由,日志 dest=新 owner,实体消失。
#[test]
fn ace2_remove_after_migration_routes_to_owner() {
    let (mut session, e) = migrated_session();
    let entry = session
        .send_to_entity(InputSource(7), e, SimCommand::RemoveEntity { entity: e })
        .expect("按 owner 删除");
    assert_eq!(entry.dest, shard_id(1, 0), "日志 dest = 当前 owner");
    session.tick();
    assert!(session.entity_state(shard_id(1, 0), e).is_none());
    assert!(session.entity_owner(e).is_none());
    session.shutdown();
}

/// AC-E4/E2:迁移后 Teleport 按 owner 路由,actor/source 保留。
#[test]
fn ace4_teleport_after_migration_preserves_actor_source() {
    let (mut session, e) = migrated_session();
    // 服务端策略命令把实体置演员(commands 通道语义,source=None 不受限)。
    session
        .send_to_entity(
            InputSource::SCHEDULER,
            e,
            SimCommand::MarkActor {
                entity: e,
                actor: true,
            },
        )
        .expect("MarkActor 入 owner");
    session.tick();
    let entry = session
        .send_to_entity(
            InputSource(7),
            e,
            SimCommand::TeleportEntity {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(140.0, 10.0, 8.0),
                    ..EntityState::default()
                },
            },
        )
        .expect("迁移后 Teleport");
    assert_eq!(entry.dest, shard_id(1, 0));
    session.tick();
    let owner = session.entity_owner(e).expect("仍在目的片");
    assert_eq!(owner.shard, shard_id(1, 0));
    assert_eq!(owner.source, Some(InputSource(7)));
    assert!(owner.actor, "actor 经 Teleport 保留");
    session.shutdown();
}

/// AC-E6:player 8 不能操作 player 7 的实体;player 7 可以。
#[test]
fn ace6_player_cannot_command_other_entity() {
    let (mut session, e) = migrated_session();
    let remove = SimCommand::RemoveEntity { entity: e };
    assert!(
        validate_external_command(&session, Some(InputSource(8)), shard_id(9, 9), &remove,)
            .is_err(),
        "其他玩家越权必须拒绝(且 shard 参数无关)"
    );
    assert!(
        validate_external_command(&session, Some(InputSource(7)), shard_id(9, 9), &remove).is_ok(),
        "owner 玩家可操作;伪造 shard 被忽略"
    );
    // 服务端实体:外部拒绝,commands 通道(None)允许。
    let e2 = EntityId::from_parts(0, 2);
    session
        .spawn_owned(
            shard_id(1, 0),
            e2,
            EntityState {
                translation: Vec3::new(136.0, 10.0, 8.0),
                ..EntityState::default()
            },
            EntityDesc::default(),
            InputSource::SCHEDULER,
        )
        .expect("服务端 spawn");
    session.tick();
    let remove2 = SimCommand::RemoveEntity { entity: e2 };
    assert!(
        validate_external_command(&session, Some(InputSource(7)), shard_id(1, 0), &remove2)
            .is_err()
    );
    assert!(
        validate_external_command(&session, None, shard_id(1, 0), &remove2).is_ok(),
        "commands 通道可操作服务端实体"
    );
    session.shutdown();
}

/// AC-E5:checkpoint 恢复后 ownership 从实体表重建,owner=dest。
#[test]
fn ace5_checkpoint_restore_rebuilds_ownership() {
    let (session, e) = migrated_session();
    let world = session.world_snapshot();
    session.shutdown();

    let restored = replay_world(&world, &CommandLog::new(), session_cfg(), registry_factory)
        .expect("自有快照必可恢复");
    assert_eq!(
        restored.entity_owner(e).map(|o| o.shard),
        Some(shard_id(1, 0)),
        "restore 后 ownership 重建;owner=目的片"
    );
    restored.shutdown();
}

/// AC-E3:`spawn_owned` 重复 ID / 错片拒绝,服务端 ID 唯一由调用方分配。
#[test]
fn ace3_spawn_owned_rejects_duplicate_and_wrong_shard() {
    let mut session = Session::new(session_cfg());
    session.register(sim(0, 0));
    let e = EntityId::from_parts(0, 1);
    let state = EntityState {
        translation: Vec3::new(8.0, 10.0, 8.0),
        ..EntityState::default()
    };
    session
        .spawn_owned(
            shard_id(0, 0),
            e,
            state,
            EntityDesc::default(),
            InputSource(7),
        )
        .expect("首次");
    session.tick();
    assert!(matches!(
        session.spawn_owned(
            shard_id(0, 0),
            e,
            state,
            EntityDesc::default(),
            InputSource(7)
        ),
        Err(sim_api::session::SessionError::Rejected(_))
    ));
    assert!(matches!(
        session.spawn_owned(
            shard_id(0, 0),
            EntityId::from_parts(0, 2),
            EntityState {
                translation: Vec3::new(136.0, 10.0, 8.0),
                ..EntityState::default()
            },
            EntityDesc::default(),
            InputSource(7)
        ),
        Err(sim_api::session::SessionError::Rejected(_))
    ));
    session.shutdown();
}
