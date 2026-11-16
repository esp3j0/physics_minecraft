//! REQ-012 C02 S 组:Go2 站立(AC-C3,判据与 AC-B1 同类)。
//! 布景照 tests/kernel_reconcile.rs,全部走 physics-sim 公开接口;
//! 每拍 apply_activation(观察点在主体上方 2 m,保证不被冻结),
//! 照 physics-sim/src/agent.rs 测试段 stand_probe 的节奏。

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
use std::path::{Path, PathBuf};
use types::glam::{Quat, Vec3};
use types::{ChunkPos, DimensionId, EntityId, ShardPos, Tick, WorldId};
use voxel::SimpleRegistry;

/// 地面顶面世界 y = 1(chunk y=64)。
const GROUND_TOP: f32 = 1.0;
/// 根生成高度:1 + 0.50(任务书 1.3)。
const SPAWN_Y: f32 = GROUND_TOP + 0.50;
const SPAWN_XZ: f32 = 24.0;

fn a() -> EntityId {
    EntityId::from_parts(1, 1)
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .canonicalize()
        .expect("仓库根")
}

fn go2_toml() -> String {
    std::fs::read_to_string(
        repo_root()
            .join("assets")
            .join("bodies")
            .join("go2.body.toml"),
    )
    .expect("读 go2.body.toml")
}

/// 布景:注册身体、3×3 平板地面、生成主体;自检全部连杆原点 y ≥ 1。
fn build_sim(toml_text: &str, spawn_y: f32) -> ShardSim {
    let mut bodies = BodyRegistry::new();
    let hash = bodies.insert_toml(toml_text).expect("注册表收下 Go2");
    let mut cfg = ShardSimConfig::new(types::ShardId::new(
        WorldId(0),
        DimensionId(0),
        ShardPos::new(0, 0),
    ));
    cfg.agent_mass_scale = 0.01;
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
    sim.apply_prefix(&[StampedInput {
        apply_at: Tick::ZERO,
        source: InputSource::SCHEDULER,
        sequence: 0,
        payload: InputPayload::Command(SimCommand::SpawnAgent {
            entity: a(),
            body: hash,
            state: types::EntityState {
                translation: Vec3::new(SPAWN_XZ, spawn_y, SPAWN_XZ),
                rotation: Quat::IDENTITY,
                ..types::EntityState::default()
            },
        }),
    }]);
    let (.., poses) = sim
        .agent_states()
        .into_iter()
        .find(|(id, ..)| *id == a())
        .expect("主体已生成");
    assert_eq!(poses.len(), 13, "Go2 13 连杆");
    for (i, p) in poses.iter().enumerate() {
        assert!(
            p.translation.y >= GROUND_TOP,
            "布景自检:连杆 {i} 原点 y = {} < {GROUND_TOP}",
            p.translation.y
        );
    }
    sim
}

fn root_pose(sim: &ShardSim) -> Option<types::EntityState> {
    sim.agent_states()
        .into_iter()
        .find(|(id, ..)| *id == a())
        .map(|(.., poses)| poses[0])
}

/// 一组站立统计(S1 判据 + 回报要打印的数字)。
struct StandStats {
    min_up: f32,
    min_up_after_120: f32,
    mean_h: f32,
    std_h: f32,
    max_rms: f32,
    max_rms_all: f32,
    h_max_120: f32,
    h_min_120: f32,
    horiz_disp: f32,
    removed: bool,
    non_finite: bool,
}

/// 步进 n 拍,目标保持默认角;逐拍记录并汇总。
fn run_stand(sim: &mut ShardSim, n: usize) -> StandStats {
    let observers = [Vec3::new(SPAWN_XZ, SPAWN_Y + 2.0, SPAWN_XZ)];
    let targets = sim.agent_joint_targets(a()).expect("默认目标表").to_vec();
    let start = root_pose(sim).expect("生成后根位姿").translation;
    let mut st = StandStats {
        min_up: f32::INFINITY,
        min_up_after_120: f32::INFINITY,
        mean_h: 0.0,
        std_h: 0.0,
        max_rms: 0.0,
        max_rms_all: 0.0,
        h_max_120: f32::NEG_INFINITY,
        h_min_120: f32::INFINITY,
        horiz_disp: 0.0,
        removed: false,
        non_finite: false,
    };
    let mut late_heights: Vec<f32> = Vec::new();
    let mut end = start;
    for tick in 1..=n {
        sim.apply_activation(&observers);
        sim.step();
        sim.run_transient_lifecycle();
        let _ = sim.take_outbox();
        let Some(root) = root_pose(sim) else {
            st.removed = true;
            println!("stand:第 {tick} 拍主体被移除");
            break;
        };
        let (.., poses) = sim
            .agent_states()
            .into_iter()
            .find(|(id, ..)| *id == a())
            .expect("主体在");
        for p in &poses {
            if !p.translation.is_finite() || !p.rotation.is_finite() {
                st.non_finite = true;
            }
        }
        let up_y = (root.rotation * Vec3::Y).y;
        let h = root.translation.y;
        let q = sim.agent_joint_positions(a()).expect("关节角可读");
        let rms = (q
            .iter()
            .zip(&targets)
            .map(|(x, t)| (x - t) * (x - t))
            .sum::<f32>()
            / q.len() as f32)
            .sqrt();
        st.min_up = st.min_up.min(up_y);
        st.max_rms_all = st.max_rms_all.max(rms);
        if tick <= 120 {
            st.h_max_120 = st.h_max_120.max(h);
            st.h_min_120 = st.h_min_120.min(h);
        }
        if tick > 120 {
            st.min_up_after_120 = st.min_up_after_120.min(up_y);
        }
        if tick > 600 {
            late_heights.push(h);
            st.max_rms = st.max_rms.max(rms);
        }
        end = root.translation;
    }
    if !late_heights.is_empty() {
        let m = late_heights.len() as f32;
        st.mean_h = late_heights.iter().sum::<f32>() / m;
        st.std_h = (late_heights
            .iter()
            .map(|h| (h - st.mean_h).powi(2))
            .sum::<f32>()
            / m)
            .sqrt();
    }
    let dx = end.x - start.x;
    let dz = end.z - start.z;
    st.horiz_disp = (dx * dx + dz * dz).sqrt();
    st
}

fn print_stats(tag: &str, st: &StandStats) {
    println!(
        "{tag}: min_up = {:.6}, min_up_after_120 = {:.6}, mean_h_after_600 = {:.6}, \
         std_h_after_600 = {:.6}, max_rms_after_600 = {:.6}, max_rms_all = {:.6}, \
         h_max_120 = {:.6}, h_min_120 = {:.6}, horiz_disp = {:.6}, removed = {}, non_finite = {}",
        st.min_up,
        st.min_up_after_120,
        st.mean_h,
        st.std_h,
        st.max_rms,
        st.max_rms_all,
        st.h_max_120,
        st.h_min_120,
        st.horiz_disp,
        st.removed,
        st.non_finite
    );
}

/// S1:3600 拍站立。判据照任务书(AC-B1 同类)。
#[test]
fn s1_go2_stands_3600_ticks() {
    let mut sim = build_sim(&go2_toml(), SPAWN_Y);
    let st = run_stand(&mut sim, 3600);
    print_stats("s1", &st);
    assert!(!st.removed, "主体被移除");
    assert!(!st.non_finite, "位姿出现非有限数");
    assert!(
        st.min_up_after_120 >= 0.95,
        "第 120 拍后最小 up·y = {}",
        st.min_up_after_120
    );
    assert!(st.std_h <= 0.01, "第 600 拍后高度标准差 = {}", st.std_h);
    assert!(
        st.max_rms <= 0.15,
        "第 600 拍后最大关节 RMS = {}",
        st.max_rms
    );
    let clearance = st.mean_h - GROUND_TOP;
    assert!(
        (0.25..=0.40).contains(&clearance),
        "平均高度减地面 = {clearance},应在 [0.25, 0.40]"
    );
}

/// S2:前 60 拍逐拍报数(根高度、小腿角 dof 2、根线速度大小),
/// 只查有限与未移除,不设其它门槛。
#[test]
fn s2_spawn_transient() {
    let mut sim = build_sim(&go2_toml(), SPAWN_Y);
    let observers = [Vec3::new(SPAWN_XZ, SPAWN_Y + 2.0, SPAWN_XZ)];
    println!("s2: tick, root_h, calf_q(dof2), |linvel|");
    for tick in 1..=60 {
        sim.apply_activation(&observers);
        sim.step();
        sim.run_transient_lifecycle();
        let _ = sim.take_outbox();
        let root = root_pose(&sim).expect("主体未被移除");
        assert!(
            root.translation.is_finite() && root.rotation.is_finite(),
            "第 {tick} 拍位姿非有限"
        );
        let q = sim.agent_joint_positions(a()).expect("关节角可读");
        println!(
            "s2: {tick}, {:.6}, {:.6}, {:.6}",
            root.translation.y,
            q[2],
            root.linvel.length()
        );
    }
}
