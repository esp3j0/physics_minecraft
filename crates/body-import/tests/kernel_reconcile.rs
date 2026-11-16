//! REQ-012 C01a K 组:导入结果与内核对账(用 physics-sim 公开接口搭布景)。
//! 期望值由架构师手算给出(issue #25),照抄,不改数字。
//!
//! 布景:全部走公开口——`ShardSim::new_with_bodies` + `BodyRegistry::insert_toml`
//! 注册身体(`body_to_toml` 输出与文件同构)、`load_chunk` 铺地面、
//! `apply_prefix`(SpawnAgent / AgentJointTargets 盖章命令)驱动、`step` 步进、
//! `agent_states` 读连杆世界位姿、`agent_joint_positions` 读关节角。

use body_import::{ImportOptions, UpAxis};
use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
use std::collections::BTreeMap;
use std::path::Path;
use types::glam::{Quat, Vec3};
use types::{ChunkPos, DimensionId, EntityId, ShardPos, Tick, WorldId};
use voxel::SimpleRegistry;

const TOL: f32 = 1e-4;
/// 出生点(y = 20,离地面顶面 19 m;120 拍自由落体约 15 m 不触地)。
const SPAWN: Vec3 = Vec3::new(24.0, 20.0, 24.0);
/// BodyDef.links 序(fixed 合并后):agent_states 的位姿条目按这个序。
const LINKS: [&str; 5] = ["base", "hip", "thigh", "slider", "wheel"];

fn a() -> EntityId {
    EntityId::from_parts(1, 1)
}

fn data_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
}

fn testbot_xml() -> String {
    std::fs::read_to_string(data_dir().join("testbot.urdf")).expect("read testbot.urdf")
}

fn opts(kp: f32) -> ImportOptions {
    ImportOptions {
        name: "testbot".to_string(),
        up_axis: UpAxis::Z,
        kp,
        kd: 3.0,
        friction: 0.6,
        default_angles: BTreeMap::new(),
        cylinder_segments: 16,
    }
}

/// 公开口布景:导入 → toml 文本 → 注册表;3×3 chunk 平板地面(chunk y=64,
/// 顶面世界 y = 1);主体出生在 (24, 20, 24)。
fn testbot_sim(kp: f32) -> (ShardSim, types::BodyHash) {
    let (def, _report) =
        body_import::import_urdf(&testbot_xml(), &data_dir(), &opts(kp)).expect("导入成功");
    let toml_text = body_import::body_to_toml(&def);
    let mut bodies = BodyRegistry::new();
    let hash = bodies.insert_toml(&toml_text).expect("注册表收下");
    let mut cfg = ShardSimConfig::new(types::ShardId::new(
        WorldId(0),
        DimensionId(0),
        ShardPos::new(0, 0),
    ));
    cfg.agent_mass_scale = 1.0;
    let mut sim = ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), bodies);
    let stone = types::BlockId(1);
    for cx in 0..3i32 {
        for cz in 0..3i32 {
            let mut chunk = voxel::Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    chunk.set(x, 64, z, stone);
                }
            }
            sim.load_chunk(ChunkPos::new(cx, cz), chunk);
        }
    }
    (sim, hash)
}

fn spawn(sim: &mut ShardSim, hash: types::BodyHash) {
    sim.apply_prefix(&[StampedInput {
        apply_at: Tick::ZERO,
        source: InputSource::SCHEDULER,
        sequence: 0,
        payload: InputPayload::Command(SimCommand::SpawnAgent {
            entity: a(),
            body: hash,
            state: types::EntityState {
                translation: SPAWN,
                rotation: Quat::IDENTITY,
                ..types::EntityState::default()
            },
        }),
    }]);
}

/// 生产节奏一拍:step(布景无观察者,不刷激活)。
fn tick(sim: &mut ShardSim) {
    sim.step();
    sim.run_transient_lifecycle();
    let _ = sim.take_outbox();
}

/// 按名字读连杆世界位姿(agent_states 条目序 = BodyDef.links 序)。
fn pose_of(sim: &ShardSim, name: &str) -> (Vec3, Quat) {
    let (.., poses) = sim
        .agent_states()
        .into_iter()
        .find(|(id, ..)| *id == a())
        .expect("主体已生成");
    assert_eq!(poses.len(), 5, "5 连杆");
    let idx = LINKS.iter().position(|n| *n == name).expect("已知连杆名");
    let s = &poses[idx];
    (s.translation, s.rotation)
}

/// 下发关节目标(全 dof 一条命令;生效拍 = 下一拍)。
fn set_targets(sim: &mut ShardSim, seq: u64, targets: Vec<f32>) {
    sim.apply_prefix(&[StampedInput {
        apply_at: Tick::ZERO,
        source: InputSource::SCHEDULER,
        sequence: seq,
        payload: InputPayload::Command(SimCommand::AgentJointTargets {
            entity: a(),
            targets,
        }),
    }]);
}

/// K1:关节角全 0、不步进,各连杆原点世界位置 − P 与手算一致;各连杆
/// 世界旋转 = 单位旋转。
#[test]
fn k1_link_positions() {
    let (mut sim, hash) = testbot_sim(100.0);
    spawn(&mut sim, hash);
    let expected: &[(&str, [f32; 3])] = &[
        ("base", [0.0, 0.0, 0.0]),
        ("hip", [0.4, 0.5, -0.2]),
        ("thigh", [0.4, 0.5, -1.2]),
        ("slider", [-0.4, 0.8, 0.0]),
        ("wheel", [0.0, 0.5, 0.4]),
    ];
    for (name, exp) in expected {
        let (t, r) = pose_of(&sim, name);
        for i in 0..3 {
            assert!(
                (t[i] - SPAWN[i] - exp[i]).abs() <= TOL,
                "{name} 世界位置 {t:?} − P:第 {i} 分量 = {} 期望 {exp:?}(差 {})",
                t[i] - SPAWN[i],
                t[i] - SPAWN[i] - exp[i]
            );
        }
        assert!(
            r.angle_between(Quat::IDENTITY) <= TOL,
            "{name} 世界旋转应为单位旋转,与单位四元数夹角 {}",
            r.angle_between(Quat::IDENTITY)
        );
    }
}

/// K2:thigh_joint(自由度下标 1)目标 +0.3 rad,其余 0;步进到关节角稳定
/// (上限 240 拍),取 hip/thigh 世界旋转算 v = R_hip⁻¹ · R_thigh · (0,−0.4,0),
/// 期望 (0, −0.382134, +0.118208),容差 0.03,且 v.z > 0.08(方向断言)。
#[test]
fn k2_joint_axis_sign() {
    let kp = 100.0;
    let (mut sim, hash) = testbot_sim(kp);
    spawn(&mut sim, hash);
    // dof 序:0 hip_joint、1 thigh_joint、2 slide_joint、3 spin_joint。
    set_targets(&mut sim, 1, vec![0.0, 0.3, 0.0, 0.0]);
    let mut settled = 0;
    for i in 0..240 {
        tick(&mut sim);
        if let Some(q) = sim.agent_joint_positions(a()) {
            if (q[1] - 0.3).abs() < 0.01 {
                settled += 1;
                if settled >= 30 {
                    println!("k2:第 {i} 拍关节角稳定(kp={kp}),q = {q:?}");
                    break;
                }
            } else {
                settled = 0;
            }
        }
    }
    let q = sim.agent_joint_positions(a()).expect("关节角可读");
    println!("k2:实测关节角 q = {q:?}(kp = {kp})");
    assert!(
        (q[1] - 0.3).abs() < 0.01,
        "thigh_joint 应到达 0.3 rad(实测 {})",
        q[1]
    );
    let r_hip = pose_of(&sim, "hip").1;
    let r_thigh = pose_of(&sim, "thigh").1;
    let foot_local = Vec3::new(0.0, -0.4, 0.0);
    let v = (r_hip.inverse() * r_thigh) * foot_local;
    println!("k2:实测 v = {v:?}");
    let expected = [0.0, -0.382134, 0.118208];
    for i in 0..3 {
        assert!(
            (v[i] - expected[i]).abs() <= 0.03,
            "v[{i}] = {} 期望 {}(差 {})",
            v[i],
            expected[i],
            v[i] - expected[i]
        );
    }
    assert!(v.z > 0.08, "方向断言:v.z = {} 应 > 0.08", v.z);
}

/// K3:生成后步进 120 拍,全部连杆位置和旋转都是有限数,不 panic。
#[test]
fn k3_spawn_and_step() {
    let (mut sim, hash) = testbot_sim(100.0);
    spawn(&mut sim, hash);
    for _ in 0..120 {
        tick(&mut sim);
    }
    for name in LINKS {
        let (t, r) = pose_of(&sim, name);
        assert!(t.is_finite(), "{name} 位置非有限:{t:?}");
        assert!(r.is_finite(), "{name} 旋转非有限:{r:?}");
    }
}
