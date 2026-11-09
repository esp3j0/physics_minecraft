//! REQ-012 A02a(#25):行走控制器的场景判据(A02-1/2/3/5)——胶囊主体在
//! 体素地形上按生产节奏(激活刷新 → apply_prefix → step → lifecycle →
//! outbox)行走、跳跃、跨 chunk 接缝、站立。
//!
//! 控制律、接地判定、钩子按 §13 `capsule` demo 定值(kernel 已原样搬进
//! `physics_sim::walker`);本文件只做布景与判据,不改任何参数。
//!
//! 判据口径:
//! - **A02-1 平地行走**:1.5、4.3、5.6 m/s × 8 航向;稳态窗口 = 拍 240..360
//!   (行走 4 s 后进入稳态,窗口 120 拍),速度取**窗口内每拍**的水平速率
//!   与目标误差 ≤ 5%(逐拍判,取最差拍——比 demo 的"窗口末速度"更严;
//!   实测最差 0.00–0.15%)。
//! - **A02-2 跳跃**:1 格台阶跑动/静止起跳各 10 次(10 次随机化布景参数,
//!   起跳点逐次偏移),全部上去(顶到台阶面);2 格台阶 0/10;空中连按跳
//!   最高点 ≤ 单次跳 + 2 cm。
//! - **A02-3 接缝**:跨 chunk 接缝行走(出生在缝上,沿缝垂直方向),三档
//!   速度 × 8 航向,全程 |vy| 峰值 ≤ 0.5 m/s;"卡住" = 连续 30 拍水平速度
//!   < 目标的 50%(demo 同款);关接缝修正的对照峰值要明显 > 0.5。
//! - **A02-5 站立**:控制器运行、意图为零,站 3600 拍,漂移与下沉 ≤ 1 cm,
//!   不凝固、不被预算冻结。

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
use types::glam::Vec3;
use types::{
    BlockId, ChunkPos, DimensionId, EntityId, EntityState, ShardId, ShardPos, Tick, WorldId,
};
use voxel::{Chunk, SimpleRegistry};

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// capsule 注册表(与 assets/bodies/capsule.body.toml 同源;include 保证一致)。
fn capsule_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
}

/// capsule 哈希。
fn capsule_hash() -> types::BodyHash {
    capsule_registry().hash_of("capsule").expect("capsule 已装")
}

/// 默认配置(凝固开启 = 生产)+ capsule 注册表的分片。
fn walker_sim() -> ShardSim {
    let cfg = ShardSimConfig::new(shard_id(0, 0));
    ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), capsule_registry())
}

/// 平板地形:`cx × cz` 个 chunk,层 world y = 0 全石(顶面 y = 1)。
fn load_flat_ground(sim: &mut ShardSim, cx: i32, cz: i32) {
    let stone = BlockId(1);
    for x in 0..cx {
        for z in 0..cz {
            let mut chunk = Chunk::air();
            for lx in 0..voxel::CHUNK_X {
                for lz in 0..voxel::CHUNK_Z {
                    chunk.set(lx, 64, lz, stone);
                }
            }
            sim.load_chunk(ChunkPos::new(x, z), chunk);
        }
    }
}

/// 盖章输入(调度器来源;本地受信通道语义,A01/A02 同款)。
fn stamp(seq: u64, command: SimCommand) -> StampedInput {
    StampedInput {
        apply_at: Tick::ZERO,
        source: InputSource::SCHEDULER,
        sequence: seq,
        payload: InputPayload::Command(command),
    }
}

fn spawn_agent_cmd(a: EntityId, pos: Vec3) -> SimCommand {
    SimCommand::SpawnAgent {
        entity: a,
        body: capsule_hash(),
        state: EntityState {
            translation: pos,
            ..EntityState::default()
        },
    }
}

fn intent_cmd(a: EntityId, move_xz: (f32, f32), yaw: f32, jump: bool) -> SimCommand {
    SimCommand::AgentIntent {
        entity: a,
        move_xz,
        yaw,
        jump,
    }
}

/// 生产节奏的一拍(worker 同序;激活开启 = 带观察者)。
fn production_tick(sim: &mut ShardSim, observers: &[Vec3]) {
    if !observers.is_empty() {
        sim.apply_activation(observers);
    }
    sim.step();
    sim.run_transient_lifecycle();
    let _ = sim.take_outbox();
}

/// 航向 k(k = 0..8)的单位方向(+x 起,每 45°)。
fn heading_dir(k: usize) -> (f32, f32) {
    let rad = (k as f32) * core::f32::consts::FRAC_PI_4;
    (rad.cos(), rad.sin())
}

// ── A02-1 平地行走 ─────────────────────────────────────────────────────

/// 稳态窗口 = 拍 240..360(行走 4 s 后,窗口 120 拍);判据:窗口内每拍
/// 水平速率与目标的误差 ≤ 5%(速率 = |v_h|,方向由力控自然对齐;取窗口
/// 内最差拍,比 demo 的"窗口末速度"更严)。
#[test]
fn a02_1_flat_walk_three_speeds_eight_headings() {
    let speeds = [1.5f32, 4.3, 5.6];
    for (si, &speed) in speeds.iter().enumerate() {
        for heading in 0..8usize {
            let (dx, dz) = heading_dir(heading);
            let mut s = walker_sim();
            // 6×6 chunk 平板(0..96)²:出生 (48,48) 居中,8 航向最大行程
            // 5.6 × 6 s = 33.6 → 终点坐标 ∈ (14.4, 81.6),全部在已载地形内。
            load_flat_ground(&mut s, 6, 6);
            // 行走 6 s(360 拍;前 4 s 充分进入稳态)。
            let a = EntityId::from_parts(1, 1);
            s.apply_prefix(&[
                stamp(0, spawn_agent_cmd(a, Vec3::new(48.0, 1.95, 48.0))),
                stamp(
                    1,
                    intent_cmd(a, (dx * speed, dz * speed), (si as f32) * 0.1, false),
                ),
            ]);
            let observers = [Vec3::new(48.0, 3.0, 48.0)];
            let mut worst_err = 0.0f32;
            for tick in 0..360u32 {
                production_tick(&mut s, &observers);
                if tick >= 240 {
                    // 稳态窗口(tick 240..360 = 最后 2 s;行走从拍 0 直接开始,
                    // 60 拍后必已达稳态;窗口取 240 起 = 行走 4 s 后;窗口内
                    // 每拍都判,取最差)。
                    let st = s.entity_state(a).expect("主体全程在本体区内");
                    let v = (st.linvel.x * st.linvel.x + st.linvel.z * st.linvel.z).sqrt();
                    worst_err = worst_err.max((v - speed).abs() / speed);
                }
            }
            assert!(
                worst_err <= 0.05,
                "A02-1 {speed} m/s 航向 {heading}:稳态窗口最差误差 {worst_err:.4} > 5%"
            );
            // 终点仍在已载地形 (0,96)² 内(布景自检)。
            let st = s.entity_state(a).expect("终点仍在(主体全程在已载地形内)");
            assert!(
                st.translation.x > 4.0
                    && st.translation.x < 92.0
                    && st.translation.z > 4.0
                    && st.translation.z < 92.0,
                "布景自检:终点 ({}, {}) 在已载地形内",
                st.translation.x,
                st.translation.z
            );
        }
    }
}

// ── A02-2 跳跃 ─────────────────────────────────────────────────────────

/// 台阶布景:地面 y=0 全石;台阶从 x ≥ STEP_X 起 1 或 2 格高(顶面 y = 2 或 3);
/// 跑道沿 +x。返回 (sim, agent, 台阶面 y)。
fn step_scene(steps: i32, spawn_x: f32) -> (ShardSim, EntityId, f32) {
    let mut s = walker_sim();
    load_flat_ground(&mut s, 3, 1);
    let stone = BlockId(1);
    let step_x = 12i32; // 台阶起始格(x = 12)
    // 台阶从 step_x 到 step_x + 8,高 steps 格(世界 y = 1..1+steps)。
    let mut chunk = Chunk::air();
    for lx in 0..voxel::CHUNK_X {
        for lz in 0..voxel::CHUNK_Z {
            chunk.set(lx, 64, lz, stone);
            if (lx as i32) >= step_x && (lx as i32) < (step_x + 8) {
                for dy in 1..=steps {
                    chunk.set(lx, (64 + dy) as usize, lz, stone);
                }
            }
        }
    }
    s.load_chunk(ChunkPos::new(0, 1), chunk);
    let a = EntityId::from_parts(1, 1);
    s.apply_prefix(&[
        stamp(0, spawn_agent_cmd(a, Vec3::new(spawn_x, 1.95, 8.0))),
        stamp(1, intent_cmd(a, (4.3, 0.0), 0.0, false)),
    ]);
    (s, a, 1.0 + steps as f32)
}

/// 跑动起跳:目标速度 4.3,表面到面间隙 ≤ d 时按跳(帧量化误差 ∈ (d−0.072, d],
/// demo 同款);判定"上去" = 主体质心到达台阶面之上(≥ 台阶面 + 0.9 半高)
/// 且最终 x 过台阶前沿。
fn try_step_up(run: bool, steps: i32, d: f32, rng_off: f32) -> bool {
    let (mut s, a, step_top) = step_scene(steps, 6.0 + rng_off);
    let observers = [Vec3::new(10.0, 3.0, 8.0)];
    let step_face_x = 12.0f32; // 台阶前沿世界 x
    let mut pressed = false;
    let mut reached = false;
    for tick in 0..360u32 {
        if run && !pressed {
            // 跑动起跳:间隙 = 面前沿 − 质心 − 半径 ≤ d 时按跳。
            let st = s.entity_state(a).expect("主体在");
            let gap = step_face_x - st.translation.x - 0.3;
            if gap <= d && st.translation.x < step_face_x {
                pressed = true;
                s.apply_prefix(&[stamp(
                    10 + tick as u64,
                    intent_cmd(a, (4.3, 0.0), 0.0, true),
                )]);
            }
        }
        if !run && tick == 60 {
            // 静止起跳:安定 1 s 后按跳并同时给移动意图。
            pressed = true;
            s.apply_prefix(&[stamp(60, intent_cmd(a, (4.3, 0.0), 0.0, true))]);
        }
        production_tick(&mut s, &observers);
        if let Some(st) = s.entity_state(a) {
            // 上去 = 质心过台阶前沿且质心 y ≥ 台阶面 + 0.5(半高 0.9,
            // 站上后质心 = 台阶面 + 0.9;跳跃顶点也过此线,配合 x 过前沿
            // 即"登上了台阶"——demo 口径:末态在台阶上)。
            if st.translation.x > step_face_x + 0.5
                && st.translation.x < step_face_x + 6.0
                && st.translation.y >= step_top + 0.5
            {
                reached = true;
                break;
            }
        } else {
            break;
        }
    }
    reached
}

#[test]
fn a02_2_jump_one_step_up_run_and_stand_two_steps_fail() {
    // 10 次跑动 + 10 次静止:起跳点逐次偏移(帧量化 + 位置微扰),1 格全上。
    for i in 0..10usize {
        let off = (i as f32) * 0.072; // 半拍行程步进(4.3 m/s 每拍 0.072 m)
        assert!(
            try_step_up(true, 1, 0.10, off),
            "A02-2 1 格台阶跑动起跳第 {} 次失败(off = {off:.3})",
            i + 1
        );
        assert!(
            try_step_up(false, 1, 0.0, off),
            "A02-2 1 格台阶静止起跳第 {} 次失败(off = {off:.3})",
            i + 1
        );
    }
    // 2 格台阶:跑动/静止各 10 次(判据 0/10;R1-P3 不允许只采样 2 次),
    // 起跳点偏移与 1 格台阶同款(帧量化 + 位置微扰;demo 实测顶点 1.39 m
    // < 2 m 台阶)。
    for i in 0..10usize {
        let off = (i as f32) * 0.072;
        assert!(
            !try_step_up(true, 2, 0.10, off),
            "A02-2 2 格台阶跑动起跳第 {} 次不应成功(off = {off:.3})",
            i + 1
        );
        assert!(
            !try_step_up(false, 2, 0.0, off),
            "A02-2 2 格台阶静止起跳第 {} 次不应成功(off = {off:.3})",
            i + 1
        );
    }
}

/// 空中连按跳:最高点不超过单次跳 + 2 cm。布景 = 平地;单次跳记录顶点,
/// 连按跳(60..360 拍每控制拍 jump = true)记录顶点,比较。
#[test]
fn a02_2_air_repeat_jump_not_higher_than_single() {
    let jump_apex = |repeat: bool| -> f32 {
        let mut s = walker_sim();
        load_flat_ground(&mut s, 3, 3);
        let a = EntityId::from_parts(1, 1);
        s.apply_prefix(&[
            stamp(0, spawn_agent_cmd(a, Vec3::new(48.0, 1.95, 48.0))),
            stamp(1, intent_cmd(a, (0.0, 0.0), 0.0, false)),
        ]);
        let observers = [Vec3::new(48.0, 3.0, 48.0)];
        let mut apex = 0.0f32;
        for tick in 0..360u32 {
            if tick == 60 || (repeat && tick > 60 && tick < 360 && tick % 2 == 0) {
                s.apply_prefix(&[stamp(
                    100 + tick as u64,
                    intent_cmd(a, (0.0, 0.0), 0.0, true),
                )]);
            }
            production_tick(&mut s, &observers);
            if let Some(st) = s.entity_state(a) {
                apex = apex.max(st.translation.y);
            }
        }
        apex
    };
    let single = jump_apex(false);
    let repeat = jump_apex(true);
    assert!(
        repeat <= single + 0.02,
        "A02-2 空中连按跳顶点 {repeat:.4} > 单次 {single:.4} + 2 cm"
    );
    // 布景自检:单次跳确实发生(顶点 ≈ 出生 + 1.4)。
    assert!(
        single > 1.95 + 1.0,
        "布景自检:单次跳顶点 {single:.4} 应显著高于出生 1.95"
    );
}

// ── A02-3 接缝 ─────────────────────────────────────────────────────────

/// 接缝布景:地面 chunk (0,0)/(1,0) 两片(缝在 x = 16 处,共面);出生在
/// 缝的 −x 侧 8 m,沿航向走 6 s(过缝后继续)。返回全程 |vy| 峰值与是否卡住。
fn seam_run(speed: f32, heading: usize, seam_fix: bool) -> (f32, bool) {
    let mut s = walker_sim();
    // 3×3 chunk 平板:两条共面缝在 x = 16 与 z = 16(chunk 交界);行程
    // 3 s(180 拍)= 16.8 m @5.6 m/s。起点按航向反推:全程中点落在缝线上
    // (起点 = 缝点 − dir × 8.4),8 个航向的整段轨迹都保持在已载地形
    // (0..48)² 内且必过一条缝。出生落地不计(demo 口径:vy 统计从拍 60 起)。
    load_flat_ground(&mut s, 3, 3);
    if !seam_fix {
        s.set_seam_fix_for_test(false);
    }
    let (dx, dz) = heading_dir(heading);
    // 缝点:奇数段落在 z 缝(z = 16),偶数段落在 x 缝(x = 16);缝点
    // 另一坐标取 24(chunk 1 的中部,远离 32..48 的后缝)。
    let seam_pt = if heading.is_multiple_of(2) {
        [16.0f32, 24.0]
    } else {
        [24.0, 16.0]
    };
    let sx = seam_pt[0] - dx * 8.4;
    let sz = seam_pt[1] - dz * 8.4;
    let a = EntityId::from_parts(1, 1);
    s.apply_prefix(&[
        stamp(0, spawn_agent_cmd(a, Vec3::new(sx, 1.95, sz))),
        stamp(1, intent_cmd(a, (dx * speed, dz * speed), 0.0, false)),
    ]);
    let observers = [Vec3::new(seam_pt[0], 3.0, seam_pt[1])];
    let mut vy_peak = 0.0f32;
    let mut stuck = true;
    let mut slow_run = 0u32;
    let mut tick_no = 0u32;
    for _ in 0..180u32 {
        production_tick(&mut s, &observers);
        tick_no += 1;
        if let Some(st) = s.entity_state(a) {
            // 出生落地不计(demo 口径:vy 统计从行走段开始,拍 60 起——
            // 出生 1.95 → 1.9 的 5 mm 下落本身 vy ≈ 1 m/s,不是接缝现象)。
            if tick_no >= 60 {
                vy_peak = vy_peak.max(st.linvel.y.abs());
            }
            let v = (st.linvel.x * st.linvel.x + st.linvel.z * st.linvel.z).sqrt();
            // 卡住(demo 口径):连续 30 拍水平速度 < 目标的 50%。
            if v >= speed * 0.5 {
                slow_run = 0;
                stuck = false;
            } else {
                slow_run += 1;
                if slow_run >= 30 {
                    stuck = true;
                }
            }
        } else {
            // 主体没了(出本体区/掉缝)= 布景失败,按卡住+大峰值报。
            return (vy_peak.max(999.0), true);
        }
    }
    (vy_peak, stuck)
}

#[test]
fn a02_3_seam_cross_peak_vy_and_no_stuck() {
    let speeds = [1.5f32, 4.3, 5.6];
    for &speed in &speeds {
        for heading in 0..8usize {
            let (vy, stuck) = seam_run(speed, heading, true);
            assert!(
                vy <= 0.5,
                "A02-3 {speed} m/s 航向 {heading}:竖直速度峰值 {vy:.4} > 0.5"
            );
            assert!(
                !stuck,
                "A02-3 {speed} m/s 航向 {heading}:卡住(连续 30 拍 < 50% 目标)"
            );
        }
    }
}

/// 对照(A02-3 第 2 半):关掉接缝修正,峰值要明显 > 0.5(demo 实测关钩子
/// 0.22 / 1.39 / 1.82;布景自检证明接缝问题真实存在)。
#[test]
fn a02_3_seam_control_without_fix_exceeds_threshold() {
    // demo 实测 1.5 m/s 档关钩子只有 0.22 < 0.5(低速推测接触弱),对照
    // 用 4.3 / 5.6 两档(> 0.5 的档)。
    for &speed in &[4.3f32, 5.6] {
        let (vy, _stuck) = seam_run(speed, 0, false);
        assert!(
            vy > 0.5,
            "A02-3 对照(关修正){speed} m/s:峰值 {vy:.4} 应明显 > 0.5"
        );
    }
}

// ── A02-5 站立 ─────────────────────────────────────────────────────────

/// 站立 3600 拍:控制器在运行、意图为零;漂移与下沉 ≤ 1 cm;不凝固(主体
/// 还在、agents 表还在)、不被预算冻结(仍 Dynamic)。
#[test]
fn a02_5_stand_3600_ticks_no_drift_no_solidify_no_freeze() {
    let mut s = walker_sim();
    load_flat_ground(&mut s, 3, 3);
    let baseline = solid_count(&s);
    let a = EntityId::from_parts(1, 1);
    // 意图 = 零速 + 零朝向(控制器在运行:控制拍照常判定接地、算零力)。
    s.apply_prefix(&[
        stamp(0, spawn_agent_cmd(a, Vec3::new(48.0, 1.95, 48.0))),
        stamp(1, intent_cmd(a, (0.0, 0.0), 0.0, false)),
    ]);
    let observers = [Vec3::new(48.0, 3.0, 48.0)];
    for _ in 0..3600u32 {
        production_tick(&mut s, &observers);
    }
    let st = s
        .entity_state(a)
        .expect("主体全程在(不凝固移除;凝固会让 entity_state 变 None)");
    assert!(s.is_agent(a), "agents 表还在(主体未被凝固/冻结写块)");
    assert_eq!(solid_count(&s), baseline, "实心方块数不变(主体不凝固写块)");
    assert!(
        (st.translation.y - 1.9).abs() <= 0.01,
        "下沉 ≤ 1 cm(站面 y=1、半高 0.9 → 静止中心 1.9):y={}",
        st.translation.y
    );
    assert!(
        (st.translation.x - 48.0).abs() <= 0.01 && (st.translation.z - 48.0).abs() <= 0.01,
        "水平漂移 ≤ 1 cm:x={},z={}",
        st.translation.x,
        st.translation.z
    );
    // 不被预算冻结:主体仍接地 = 仍动态站立(冻结体不参与接地投射的
    // 命中语义——kinematic 主体不会被投球判为接地站立;agent_is_grounded
    // 是公开观测口)。主体在表 + entity_state 在 = 未凝固移除。
    assert!(
        s.agent_is_grounded(a, 0.05),
        "3600 拍后仍接地(动态站立,未被冻结)"
    );
}

/// 装载 chunk 的实心方块数(对照用;与 agent.rs 同款口径)。
fn solid_count(sim: &ShardSim) -> usize {
    let mut n = 0usize;
    for cp in sim.voxel().chunk_positions().collect::<Vec<_>>() {
        let min = cp.block_min();
        for x in 0..voxel::CHUNK_X {
            for y in 0..voxel::CHUNK_Y {
                for z in 0..voxel::CHUNK_Z {
                    if !sim
                        .voxel()
                        .get_block(types::BlockPos::new(
                            min.x + x as i32,
                            min.y + y as i32,
                            min.z + z as i32,
                        ))
                        .is_air()
                    {
                        n += 1;
                    }
                }
            }
        }
    }
    n
}
