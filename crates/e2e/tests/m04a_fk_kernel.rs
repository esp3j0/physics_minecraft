//! REQ-014 M04a T9:客户端静姿 FK 与物理内核 PD 落定位姿一致性锚。
//!
//! 零重力无接触下生成 13 连杆 `quad` 主体,步进 600 拍(10s)待 PD 电机落定到
//! `default_angle`,将内核各连杆世界位姿换算为根相对位姿,与
//! `client_core::bodies::static_link_transforms` 逐连杆比对(防旋转符号或复合顺序错)。

use client_core::bodies::static_link_transforms;
use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
use shard::SchedulerConfig;
use sim_api::session::{Session, SessionConfig};
use types::glam::Vec3;
use types::{BodyDef, DimensionId, EntityId, EntityState, ShardId, ShardPos, WorldId};
use voxel::SimpleRegistry;

const QUAD_TOML: &str = include_str!("../../../assets/bodies/quad.body.toml");

#[test]
fn m04a_t9_fk_matches_kernel_settled() {
    let quad_def = BodyDef::from_toml_str(QUAD_TOML).expect("quad.body.toml 合法");
    let expected = static_link_transforms(&quad_def);
    assert_eq!(expected.len(), 13, "quad 静姿 FK 应产出 13 连杆");

    let mut bodies = BodyRegistry::new();
    bodies.insert_toml(QUAD_TOML).expect("quad 注册表插入成功");
    let hash = bodies.hash_of("quad").expect("quad 哈希可查");

    let shard = ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0));
    let mut sim_cfg = ShardSimConfig::new(shard);
    sim_cfg.gravity = Vec3::ZERO;
    sim_cfg.solidify.enabled = false;
    sim_cfg.cohesion.enabled = false;

    let mut session = Session::new(SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 1,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    });
    session.register(ShardSim::new_with_bodies(
        sim_cfg,
        Box::new(SimpleRegistry::builtin()),
        bodies,
    ));

    let entity = EntityId::from_parts(0, 1);
    session
        .spawn_agent(
            shard,
            entity,
            hash,
            EntityState {
                translation: Vec3::new(8.0, -58.0, 8.0),
                ..EntityState::default()
            },
        )
        .expect("spawn quad");

    for _ in 0..600 {
        session.tick();
    }

    let states = session.agent_states();
    let (_, got_hash, poses) = states
        .into_iter()
        .find(|(id, ..)| *id == entity)
        .expect("600 拍后 quad 主体必须仍在会话中");
    assert_eq!(got_hash, hash, "主体哈希一致");
    assert_eq!(poses.len(), 13, "内核应返回 13 连杆世界位姿");

    let root_pos = poses[0].translation;
    let root_rot = poses[0].rotation;
    let inv_root = root_rot.conjugate();

    for (i, (pose, &(exp_pos, exp_rot))) in poses.iter().zip(expected.iter()).enumerate() {
        let rel_pos = inv_root * (pose.translation - root_pos);
        let rel_rot = (inv_root * pose.rotation).normalize();

        let pos_diff = (rel_pos - exp_pos).abs();
        assert!(
            pos_diff.x <= 2e-3 && pos_diff.y <= 2e-3 && pos_diff.z <= 2e-3,
            "连杆 {i} ({}) 位置误差超 2e-3 m: rel={rel_pos:?}, fk={exp_pos:?}, diff={pos_diff:?}",
            quad_def.links[i].name
        );

        let rel_d = [
            rel_rot.x as f64,
            rel_rot.y as f64,
            rel_rot.z as f64,
            rel_rot.w as f64,
        ];
        let exp_d = [
            exp_rot.x as f64,
            exp_rot.y as f64,
            exp_rot.z as f64,
            exp_rot.w as f64,
        ];
        let nr =
            (rel_d[0] * rel_d[0] + rel_d[1] * rel_d[1] + rel_d[2] * rel_d[2] + rel_d[3] * rel_d[3])
                .sqrt();
        let ne =
            (exp_d[0] * exp_d[0] + exp_d[1] * exp_d[1] + exp_d[2] * exp_d[2] + exp_d[3] * exp_d[3])
                .sqrt();
        let dot =
            (rel_d[0] * exp_d[0] + rel_d[1] * exp_d[1] + rel_d[2] * exp_d[2] + rel_d[3] * exp_d[3])
                / (nr * ne);
        let ang_err = 2.0 * dot.abs().min(1.0).acos();
        assert!(
            ang_err <= 2e-3,
            "连杆 {i} ({}) 旋转角误差超 2e-3 rad: rel={rel_rot:?}, fk={exp_rot:?}, err={ang_err}",
            quad_def.links[i].name
        );
    }
}
