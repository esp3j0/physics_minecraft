//! REQ-012 A04(#25):主体迁移(单连杆)的场景判据——胶囊主体经 Session /
//! 调度器层驱动跨分片边界行走、过四片交汇点、空中过界、相位对齐、迁移拍
//! 施力、迁移后仍是主体、被推过界、确定性、MARGIN 校验。
//!
//! 判据口径(A04 放行判据表;R1 返修按总架构师验收 P1–P10 调整):
//! - **A04-1 跨边界行走**:4.3、5.6 m/s × x/z 向边界 × 两个方向;全世界
//!   恰好一个主体、相邻拍位移 ≤ 1.5 × max_speed × dt(采样间隔平常 1,
//!   仅跨迁移在途窗口的样本为 2)、高度 ± 2 cm、过界前后各 30 拍水平速度
//!   误差 ≤ 5%。
//! - **A04-2 四片交汇点**:斜穿交汇点(两个对角方向);零意图站交汇点
//!   600 拍漂移与下沉 ≤ 1 cm;迁移次数写进回报(assert! 后打印)。
//! - **A04-3 空中过界**:起跳后空中过界,最高点与同一布景不过界跳跃差
//!   ≤ 2 cm,落地后继续按意图行走。
//! - **A04-4 相位**:偶数全局拍创建分片 A、奇数全局拍创建分片 B,主体
//!   A→B→A;保持力的变化拍恒落在全局偶数拍(walker_hold_of)。
//! - **A04-5 迁移拍不丢力**:迁移分别落在控制拍/非控制拍,迁移那一拍目的
//!   分片施加了保持的力(口径:该拍速度增量,对照同布景零保持力布景)。
//! - **A04-6 到目的分片仍是主体**:过界后站 3600 拍,不凝固、不被预算
//!   冻结、is_agent、actor、EntityStates 不含、entity_owner = 目的分片。
//! - **A04-7 被推过界**:爆炸把站着的主体高速推过边界,无 panic、无 NaN、
//!   相邻拍位移 ≤ 该拍实际速度 × dt × 1.5。
//! - **A04-8 确定性**:时间线(行走 + 空中 + 交汇点)Session 回放两次逐位
//!   相同;检查点落迁移拍(信封在途)与之后一拍,续跑与不中断线逐位相同。
//! - **A04-9 MARGIN 校验**:包络超限的身体被拒(类型化错误)。
//! - **A04-10 不影响旧行为**:MigrateEntity 路径行为不变(既有 migration.rs
//!   测试守住)、golden 不变(既有套件);门禁与 smoke 在 CI。
//!
//! 布景:2×2 分片(片 (0,0)/(1,0)/(0,1)/(1,1)),各装载本体 8×8 chunk 平板
//! (层 0 全石,顶面 y = 1)+ 扩张带重叠(边界两侧都有 collider,REQ-001
//! §4.2 几何保证)。凝固打开(默认参数),生产节奏 = Session::tick(激活
//! 刷新 + apply_prefix + step + lifecycle + outbox)。

use messaging::SimCommand;
use physics_sim::{BodyRegistry, ShardSim};
use shard::SchedulerConfig;
use sim_api::session::{Session, SessionConfig};
use types::glam::Vec3;
use types::{
    BlockId, ChunkPos, DimensionId, EntityId, EntityState, ShardId, ShardPos, Tick, WorldId,
};
use voxel::{Chunk, SimpleRegistry};

/// capsule max_speed(A02a 定值;位移上界判据的系数)。
const MAX_SPEED: f32 = 5.6;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn capsule_registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(include_str!("../../../assets/bodies/capsule.body.toml"))
        .expect("capsule.body.toml 合法");
    bodies
}

fn capsule_hash() -> types::BodyHash {
    capsule_registry().hash_of("capsule").expect("capsule 已装")
}

/// 带默认配置(凝固开启 = 生产)+ capsule 注册表的分片。
fn agent_sim(shard: ShardId) -> ShardSim {
    let cfg = physics_sim::ShardSimConfig::new(shard);
    ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), capsule_registry())
}

/// 平板地形:`cx × cz` 个 chunk,层 world y = 0 全石(顶面 y = 1)。
fn flat_ground_chunk() -> Chunk {
    let mut chunk = Chunk::air();
    let stone = BlockId(1);
    for x in 0..voxel::CHUNK_X {
        for z in 0..voxel::CHUNK_Z {
            chunk.set(x, 64, z, stone);
        }
    }
    chunk
}

/// 2×2 分片的 Session:四片先全部注册(装配期先 owner 后 holder;反向会在
/// apply_chunk_load 触发"owner 未注册先装载"fail-stop),再各装本体 8×8
/// chunk 平板(层 0 全石,顶面 y = 1)+ 各自向 +x/+z 方向多载一列扩张副本
/// (边界两侧都有 collider;REQ-001 §4.2 几何保证:扩张区重叠、本体互斥)。
/// tick() 未跑过,全部注册在全局拍 0(A04-4 的奇偶相位布景自行控制注册拍)。
fn four_shard_session() -> Session {
    let mut session = Session::new(SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    });
    for &(sx, sz) in &[(0i32, 0i32), (1, 0), (0, 1), (1, 1)] {
        session.register(agent_sim(shard_id(sx, sz)));
    }
    for &(sx, sz) in &[(0i32, 0i32), (1, 0), (0, 1), (1, 1)] {
        let shard = shard_id(sx, sz);
        // 本体 8×8 chunk(chunk 坐标 [8sx, 8sx+8) × [8sz, 8sz+8));扩张副本
        // 只向**四片覆盖内**的邻片方向扩一列(边界两侧都有 collider = 扩张区
        // 重叠;超出四片覆盖的方向不扩——owner 未注册,装载 fail-stop)。
        let xs: Vec<i32> = ((8 * sx)..(8 * sx + 8))
            .chain(std::iter::once(8 * sx - 1).filter(|_| sx > 0))
            .chain(std::iter::once(8 * sx + 8).filter(|_| sx < 1))
            .collect();
        let zs: Vec<i32> = ((8 * sz)..(8 * sz + 8))
            .chain(std::iter::once(8 * sz - 1).filter(|_| sz > 0))
            .chain(std::iter::once(8 * sz + 8).filter(|_| sz < 1))
            .collect();
        for cx in &xs {
            for cz in &zs {
                session.apply_chunk_load(shard, ChunkPos::new(*cx, *cz), flat_ground_chunk());
            }
        }
    }
    session
}

/// 主体出生:Session 正路(Session::spawn_agent,盖章入日志)。
/// 返回 LogEntry(确定性回放需把生成命令落日志)。
fn spawn_agent(
    session: &mut Session,
    shard: ShardId,
    a: EntityId,
    pos: Vec3,
) -> sim_api::log::LogEntry {
    session
        .spawn_agent(
            shard,
            a,
            capsule_hash(),
            EntityState {
                translation: pos,
                ..EntityState::default()
            },
        )
        .expect("收件箱未满")
}

/// 意图命令(Session 正路)。
fn send_intent(session: &mut Session, shard: ShardId, a: EntityId, mv: (f32, f32), yaw: f32) {
    session
        .send(
            shard,
            SimCommand::AgentIntent {
                entity: a,
                move_xz: mv,
                yaw,
                jump: false,
            },
        )
        .expect("收件箱未满");
}

/// 全世界恰好一个该主体的分片(Session 层扫 4 片;返回 Some(所在片)或
/// None = 迁移窗口)。恰好一片持有 = 判据 A04-1 的"不重复、不丢"。
fn agent_shard_of(session: &Session, a: EntityId) -> Option<ShardId> {
    let mut found = None;
    for shard in session.shard_ids() {
        if session.entity_state(shard, a).is_some() {
            if found.is_some() {
                panic!("主体 {a:?} 同时出现在多个分片(不重复判据,fail)");
            }
            found = Some(shard);
        }
    }
    found
}

/// tick 一拍并返回主体所在分片(迁移在途窗口恰一拍 None;下一拍必已重建)。
fn tick_and_shard(session: &mut Session, a: EntityId) -> ShardId {
    session.tick();
    match agent_shard_of(session, a) {
        Some(s) => s,
        None => {
            session.tick();
            agent_shard_of(session, a).expect("迁移在途窗口恰一拍,下一拍必已重建(不丢)")
        }
    }
}

// ---- 判据 A04-1:跨边界行走(1.5 / 5.6 m/s,x 向 + z 向,各两个方向)----

/// 一条跨边界行走(主体已出生并安定):沿 mv 方向走 700 拍。逐拍断言:
/// 唯一性、位移上界(平常拍采样间隔 1;仅跨迁移在途窗口的样本间隔 2
/// ——R1-P1:间隔不再恒按 2 放宽)、高度稳定;过界前后各 30 拍的水平
/// 速度误差 ≤ 5%。
fn assert_cross_boundary(session: &mut Session, a: EntityId, mv: (f32, f32), label: &str) {
    let yaw = yaw_of(mv.0, mv.1);
    let dt = 1.0 / 60.0f32;
    let mut cross_tick: Option<usize> = None;
    let mut prev: Option<(usize, Vec3, Vec3)> = None; // (样本序, pos, vel)
    let mut horiz_speeds: Vec<(usize, f32)> = Vec::new();
    // 走 700 拍(1.5 m/s 约 20 s;5.6 m/s 不足 8 s,足以跨两片)。
    let ticks = 700;
    let birth = agent_shard_of(session, a).expect("出生片(安定后主体在场)");
    send_intent(session, birth, a, mv, yaw);
    let mut samples = 0usize;
    for t in 0..ticks {
        session.tick();
        // 迁移在途窗口(tick N 末产出信封,tick N+1 前缀重建):恰一拍 None。
        // 原子性:移除拍里源片不得残留保持态(发出方清 walker_holds)。
        let mut gap = 1usize; // 本迭代消耗的拍数:平常 1
        let shard = match agent_shard_of(session, a) {
            Some(s) => s,
            None => {
                // 原子性:移除拍里任何分片都不得残留主体保持态
                // (发出方清 walker_holds;快照注册路径同源不误伤)。
                for shard in session.shard_ids() {
                    assert!(
                        session.walker_hold_of(a).is_none(),
                        "迁移在途窗口:分片 {shard:?} 仍残留主体保持态                         (发出方未清 walker_holds)"
                    );
                }
                session.tick();
                gap = 2; // 跨在途窗口的样本:间隔 2(R1-P1 唯一放宽点)
                agent_shard_of(session, a).expect("迁移在途窗口恰一拍,下一拍必已重建(不丢判据)")
            }
        };
        let st = session.entity_state(shard, a).expect("主体在场");
        let pos = st.translation;
        let v = st.linvel;
        // 唯一性(agent_shard_of 内已 panic 多片)。
        // 位移上界:相邻拍位移 ≤ 1.5 × max_speed × dt × 采样间隔拍数
        // (R1-P1:平常拍间隔 = 1;只有跨过迁移在途窗口的那个样本间隔 = 2,
        // 不再恒按 2 放宽)。
        if let Some((_, pp, _)) = prev {
            let step = (pos - pp).length();
            assert!(
                step <= 1.5 * MAX_SPEED * dt * gap as f32 + 1e-4,
                "{label}:拍 {t} 相邻拍位移 {step:.4} 超上界(1.5 × {MAX_SPEED} × dt × {gap})"
            );
        }
        // 高度稳定:站立高度 1.9 ± 2 cm(顶面 y = 1、半高 0.9)。
        assert!(
            (pos.y - 1.9).abs() <= 0.02,
            "{label}:拍 {t} 高度 {} 掉出站立高度 ± 2 cm(不掉穿地形)",
            pos.y
        );
        // NaN 守护(A04-7 口径同源)。
        assert!(
            pos.x.is_finite() && pos.y.is_finite() && pos.z.is_finite(),
            "{label}:拍 {t} 位置非有限"
        );
        horiz_speeds.push((t, (v.x * v.x + v.z * v.z).sqrt()));
        // 过界检测:x/z 向边界 = chunk 坐标 128(片 (0,0) 本体 [0,128))。
        samples += 1;
        let crossed = if mv.0.abs() > mv.1.abs() {
            (pos.x - 128.0).signum()
                != prev
                    .map(|(_, p, _)| (p.x - 128.0).signum())
                    .unwrap_or(pos.x.signum())
        } else {
            (pos.z - 128.0).signum()
                != prev
                    .map(|(_, p, _)| (p.z - 128.0).signum())
                    .unwrap_or(pos.z.signum())
        } && prev.is_some();
        if crossed && cross_tick.is_none() {
            cross_tick = Some(samples);
        }
        prev = Some((samples, pos, v));
    }
    let ct = cross_tick.expect("{label}:布景自检失败——全程未跨过边界(布景约束:行程与速度须跨 128)");
    let _ = ct;
    // 控制不中断:过界前后各 30 拍的水平速度误差 ≤ 5%(取两侧窗口内每拍
    // 样本,与目标 |mv| 比;A02-1 同口径逐拍判)。
    let target = (mv.0 * mv.0 + mv.1 * mv.1).sqrt();
    let window = |center: usize| -> Vec<f32> {
        (center.saturating_sub(30)..(center + 30).min(horiz_speeds.len()))
            .filter_map(|i| horiz_speeds.iter().find(|(t, _)| *t == i).map(|(_, s)| *s))
            .collect()
    };
    for (side, window_speeds) in [
        ("前", window(ct.saturating_sub(30))),
        ("后", window(ct + 30)),
    ] {
        let worst = window_speeds
            .iter()
            .map(|s| (s - target).abs() / target)
            .fold(0.0f32, f32::max);
        assert!(
            worst <= 0.05,
            "{label}:过界{side} 30 拍窗口内最差水平速度误差 {worst:.4} > 5%(控制不中断)"
        );
    }
}

#[test]
fn a04_1_cross_boundary_four_directions_two_speeds() {
    for (speed, mv) in [
        (4.3f32, (4.3f32, 0.0f32)),
        (5.6, (5.6, 0.0)),
        (4.3, (-4.3, 0.0)),
        (5.6, (-5.6, 0.0)),
        (4.3, (0.0, 4.3)),
        (5.6, (0.0, 5.6)),
        (4.3, (0.0, -4.3)),
        (5.6, (0.0, -5.6)),
    ] {
        // 两档速度都须跨过 128:正方向出生在片 (0,0) 内侧 (118,118 附近;
        // 距边界 10 m < 4.3 m/s × 700 拍 = 50 m),负方向出生在片 (1,1)
        // 内侧对称位 (138,138)。
        let (birth, start) = if mv.0 > 0.0 || mv.1 > 0.0 {
            (shard_id(0, 0), Vec3::new(118.0, 1.95, 118.0))
        } else {
            (shard_id(1, 1), Vec3::new(138.0, 1.95, 138.0))
        };
        let mut session = four_shard_session();
        let a = EntityId::from_parts(1, 1);
        spawn_agent(&mut session, birth, a, start);
        for _ in 0..60 {
            session.tick();
        }
        let label = format!("speed={speed} mv=({},{})", mv.0, mv.1);
        assert_cross_boundary(&mut session, a, mv, &label);
        session.shutdown();
    }
}

/// 8 航向的 yaw(A02a 同款:+x = 0,+z = π/2,取行走方向)。
fn yaw_of(mv_x: f32, mv_z: f32) -> f32 {
    mv_z.atan2(mv_x)
}

/// 出生并安定(站 60 拍),返回安定后的位置(供检查过界计数)。
fn spawn_and_settle(session: &mut Session, birth: ShardId, a: EntityId, pos: Vec3) {
    spawn_agent(session, birth, a, pos);
    for _ in 0..60 {
        session.tick();
    }
}

// ---- 判据 A04-2:四片交汇点 ----

/// 斜穿交汇点(128,128):出生在片 (0,0) 靠近交汇点内侧 (120, 120),沿
/// (+,+) 对角走(进 (1,1));再从 (1,1) 内侧沿 (−,−) 走回(反向对角)。
/// 断言同 A04-1(唯一、位移、高度)+ 迁移次数写进回报。
#[test]
fn a04_2_diagonal_through_corner() {
    let dt = 1.0 / 60.0f32;
    for (label, birth, start, mv) in [
        (
            "对角 (+,+)",
            shard_id(0, 0),
            Vec3::new(120.0, 1.95, 120.0),
            (1.5f32, 1.5f32),
        ),
        (
            "对角 (−,−)",
            shard_id(1, 1),
            Vec3::new(136.0, 1.95, 136.0),
            (-1.5f32, -1.5f32),
        ),
    ] {
        let mut session = four_shard_session();
        let a = EntityId::from_parts(1, 1);
        spawn_and_settle(&mut session, birth, a, start);
        send_intent(&mut session, birth, a, mv, yaw_of(mv.0, mv.1));
        let mut migrations = 0usize;
        let mut last_shard = agent_shard_of(&session, a);
        let mut prev_pos = start;
        let mut crossed = false;
        for t in 0..400usize {
            let shard = tick_and_shard(&mut session, a);
            if shard != last_shard.expect("布景约束:主体在场") {
                migrations += 1;
            }
            last_shard = Some(shard);
            let st = session.entity_state(shard, a).expect("主体在场");
            let pos = st.translation;
            let step = (pos - prev_pos).length();
            assert!(
                step <= 1.5 * MAX_SPEED * dt + 1e-4,
                "{label}:拍 {t} 相邻拍位移 {step:.4} 超上界"
            );
            assert!(
                (pos.y - 1.9).abs() <= 0.02,
                "{label}:拍 {t} 高度 {} 超出 ± 2 cm",
                pos.y
            );
            // 过交汇点检测:x、z 都跨过 128。
            if !crossed
                && ((pos.x - 128.0) * (start.x - 128.0) < 0.0)
                && ((pos.z - 128.0) * (start.z - 128.0) < 0.0)
            {
                crossed = true;
            }
            prev_pos = pos;
        }
        assert!(crossed, "{label}:布景自检失败——未跨过交汇点(128,128)");
        println!("{label}:迁移 {migrations} 次(斜穿交汇点 400 拍)");
        session.shutdown();
    }
}

/// 零意图站交汇点(R1-P2 两组,各 600 拍):漂移与下沉 ≤ 1 cm、相邻拍
/// 不跳变(和**上一拍**位置比,≤ 1.5 × max_speed × dt)、迁移次数如实写
/// 进回报(0 次也照写):
/// - 组 1:主体中心恰放在交汇点 (128.0, 128.0)(floor 口径归属片 (1,1));
/// - 组 2:主体沿对角走到交汇点后意图清零,刹车停在交汇点上。
#[test]
fn a04_2_stand_on_corner_600_ticks() {
    let dt = 1.0 / 60.0f32;
    // 组 2 的走线:从 (120,120) 沿对角 (+,+) 1.5 m/s 走到 (128,128) 需
    // ~38 拍(加速度段另计;600 拍窗口内先走后站,前 120 拍内必已到)。
    let cases: [(&str, bool); 2] = [("中心恰在交汇点", false), ("对角走到交汇点刹车", true)];
    for (label, walk_in) in cases {
        let mut session = four_shard_session();
        let a = EntityId::from_parts(1, 1);
        let start = Vec3::new(128.0, 1.95, 128.0);
        if walk_in {
            // 沿对角走到交汇点:出生 (120,120),意图 (+,+),到 128 后清意图
            // (每分量 8 m @ 1.5 m/s ≈ 320 拍,含加速段 480 拍窗口;走段与
            // 站段分离,600 拍只约束站段)。
            spawn_and_settle(
                &mut session,
                shard_id(0, 0),
                a,
                Vec3::new(120.0, 1.95, 120.0),
            );
            send_intent(
                &mut session,
                shard_id(0, 0),
                a,
                (1.5, 1.5),
                yaw_of(1.5, 1.5),
            );
            let mut arrived = false;
            for _ in 0..480usize {
                let shard = tick_and_shard(&mut session, a);
                let st = session.entity_state(shard, a).expect("主体在场");
                if st.translation.x >= 128.0 && st.translation.z >= 128.0 {
                    arrived = true;
                    send_intent(&mut session, shard, a, (0.0, 0.0), yaw_of(1.5, 1.5));
                    break;
                }
            }
            assert!(arrived, "{label}:布景自检——480 拍内未走到交汇点(128,128)");
            // 刹车停稳(零意图 60 拍,τ = 0.2 s ≈ 12 拍足够)。
            for _ in 0..60usize {
                session.tick();
            }
        } else {
            // 中心恰放交汇点(片 (1,1) 内侧紧贴角落;floor(128) = 128 →
            // 归属片 (1,1) 本体)。
            spawn_and_settle(&mut session, shard_id(1, 1), a, start);
            send_intent(&mut session, shard_id(1, 1), a, (0.0, 0.0), 0.0);
        }
        let anchor = session
            .entity_state(agent_shard_of(&session, a).expect("主体在场"), a)
            .expect("主体在场")
            .translation;
        let mut migrations = 0usize;
        let mut last_shard = agent_shard_of(&session, a);
        let mut max_drift = 0.0f32;
        let mut max_sink = 0.0f32;
        let mut prev_pos = anchor;
        for _ in 0..600usize {
            let shard = tick_and_shard(&mut session, a);
            if shard != last_shard.expect("主体在场") {
                migrations += 1;
            }
            last_shard = Some(shard);
            let st = session.entity_state(shard, a).expect("主体在场");
            let pos = st.translation;
            // 相邻拍不跳变:和上一拍位置比(R1-P2,不再和起点比)。
            let step = (pos - prev_pos).length();
            assert!(
                step <= 1.5 * MAX_SPEED * dt + 1e-4,
                "{label}:静止主体相邻拍位移 {step:.4} 超上界 = 跳变"
            );
            prev_pos = pos;
            max_drift = max_drift
                .max((pos.x - anchor.x).abs())
                .max((pos.z - anchor.z).abs());
            max_sink = max_sink.max((1.9 - pos.y).abs());
        }
        assert!(
            max_drift <= 0.01 && max_sink <= 0.01,
            "{label}:站交汇点 600 拍漂移 {max_drift:.4} / 下沉 {max_sink:.4} 超 1 cm"
        );
        println!(
            "站交汇点({label})600 拍:漂移 {max_drift:.4} m、下沉 {max_sink:.4} m、迁移 {migrations} 次"
        );
        session.shutdown();
    }
}

// ---- 判据 A04-3:空中过界 ----

/// 布景:出生在片 (0,0) 内侧距边界 3 m(x = 125),带水平意图 5.6 起跳。
/// 对照:同布景但出生在 x = 118(同样跑动起跳,5.6 m/s × 空中时间不足以
/// 过界)。最高点对比差 ≤ 2 cm;落地后(R1-P3)30 拍内水平速度与 5.6 的
/// 误差 ≤ 5%,且主体已过界、在目的分片(不再比落地 x——跨线组起点本就
/// 更远,该断言不可能失败)。
///
/// 布景细节:跑动起跳(x 向 5.6 + jump),起跳点 x = 125(距边界 3 m):
/// 跳跃滞空约 0.53 s(跳高 1.4 m),5.6 × 0.53 ≈ 3.0 m——恰好过界;对照
/// 起跳点 x = 118(不过界)。两者最高点应一致(跳跃动力学不因迁移而变)。
#[test]
fn a04_3_airborne_cross_boundary() {
    let run = |start_x: f32| -> (f32, Option<ShardId>, Vec<f32>) {
        let mut session = four_shard_session();
        let a = EntityId::from_parts(1, 1);
        let start = Vec3::new(start_x, 1.95, 24.0);
        spawn_and_settle(&mut session, shard_id(0, 0), a, start);
        // 跑起来(1 s 稳速)。
        send_intent(&mut session, shard_id(0, 0), a, (5.6, 0.0), 0.0);
        for _ in 0..60 {
            session.tick();
        }
        // 起跳(jump = true 一次性)。
        session
            .send(
                shard_id(0, 0),
                SimCommand::AgentIntent {
                    entity: a,
                    move_xz: (5.6, 0.0),
                    yaw: 0.0,
                    jump: true,
                },
            )
            .expect("收件箱未满");
        let mut apex = 0.0f32;
        let mut landed = false;
        for _ in 0..120usize {
            let shard = tick_and_shard(&mut session, a);
            let st = session.entity_state(shard, a).expect("主体在场");
            if st.translation.y > apex {
                apex = st.translation.y;
            }
            if st.linvel.y <= 0.0 && st.translation.y < 2.0 {
                landed = true;
                break;
            }
        }
        assert!(landed, "布景约束:120 拍内应落地");
        // R1-P3:落地后 30 拍,逐拍采水平速度;并记录主体所在分片。
        let mut post_land: Vec<f32> = Vec::new();
        let mut shard_after = None;
        for _ in 0..30usize {
            let shard = tick_and_shard(&mut session, a);
            let st = session.entity_state(shard, a).expect("主体在场");
            post_land.push((st.linvel.x * st.linvel.x + st.linvel.z * st.linvel.z).sqrt());
            shard_after = Some(shard);
        }
        session.shutdown();
        (apex, shard_after, post_land)
    };
    let (apex_cross, shard_cross, post_cross) = run(121.0);
    let (apex_ctrl, _, _) = run(114.0);
    assert!(
        shard_cross == Some(shard_id(1, 0)),
        "布景自检:跨线组落地后应在目的分片 (1,0)(实为 {shard_cross:?})"
    );
    assert!(
        (apex_cross - apex_ctrl).abs() <= 0.02,
        "空中过界最高点差 {:.4} m > 2 cm(过界 {apex_cross} vs 对照 {apex_ctrl})",
        (apex_cross - apex_ctrl).abs()
    );
    // 落地后继续按意图行走(R1-P3):30 拍内每拍水平速度与 5.6 误差 ≤ 5%。
    let target = 5.6f32;
    let worst = post_cross
        .iter()
        .map(|s| (s - target).abs() / target)
        .fold(0.0f32, f32::max);
    assert!(
        worst <= 0.05,
        "落地后 30 拍最差水平速度误差 {worst:.4} > 5%(落地后继续按意图行走)"
    );
    println!(
        "A04-3:过界组落地后 30 拍最差水平速度误差 {worst:.4}(目标 5.6 m/s)、最高点 {apex_cross} vs 对照 {apex_ctrl}"
    );
}

// ---- 判据 A04-4:相位 ----

/// 一个分片在偶数全局拍创建、相邻分片在奇数全局拍创建;主体从前者走进
/// 后者再走回。断言保持力变化拍恒落在全局偶数拍(walker_hold_of 的变化)。
///
/// 布景:空 Session(0 片)。拍 0(偶):先 tick 一次推进到拍 0…… 注册时
/// 机 = coordinator 当前拍。Session 新建时 tick_now() = 0(偶)。流程:
/// 1)注册片 A(0,0)(tick_now = 0,偶)→ 装地形 → 出生安定 → 记住 hold 变化序列;
/// 2)跑 7 拍(tick_now = 7,奇)→ 注册片 B(1,0)(奇拍创建)+ 装地形;
/// 3)主体沿 +x 从 A 走进 B 再走回(意图反向)。
/// 全程逐拍采样 hold 变化,断言变化拍全为偶数。
#[test]
fn a04_4_control_tick_always_even_across_phases() {
    let mut session = Session::new(SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    });
    let a = EntityId::from_parts(1, 1);
    // 拍 0(偶):注册片 A。
    assert_eq!(session.tick_now(), Tick(0), "布景前提:全局拍 0(偶)");
    session.register(agent_sim(shard_id(0, 0)));
    // 本体 8×8(此时 B 未注册,chunk 8 的 owner 不在——不能先装扩张列)。
    for cx in 0..8i32 {
        for cz in 0..8i32 {
            session.apply_chunk_load(shard_id(0, 0), ChunkPos::new(cx, cz), flat_ground_chunk());
        }
    }
    spawn_and_settle(
        &mut session,
        shard_id(0, 0),
        a,
        Vec3::new(120.0, 1.95, 24.0),
    );
    // 拍 60 起:让主体向 +x 走(目标:跨 128;从 120 到 128 需 ~20 拍,
    // 片 B 在拍 67 已注册,行程内重叠列均已就位)。
    send_intent(&mut session, shard_id(0, 0), a, (5.6, 0.0), 0.0);
    // 跑 7 拍到全局拍 67(tick_now = 67,奇)。60+7:安定 60 拍后意图生效
    // 起第 7 拍。逐拍采样 hold 变化。
    let mut changes: Vec<u64> = Vec::new();
    {
        let sample = |session: &mut Session, n: usize, changes: &mut Vec<u64>| {
            let mut prev = session.walker_hold_of(a);
            for _ in 0..n {
                session.tick();
                let done = session.tick_now().0 - 1;
                let cur = session.walker_hold_of(a);
                if cur.is_some() && prev.is_some() && cur != prev {
                    changes.push(done);
                }
                prev = cur;
            }
        };
        sample(&mut session, 7, &mut changes);
    }
    // 全局拍 67(奇):注册片 B(相邻分片在奇数全局拍创建)。
    assert_eq!(
        session.tick_now(),
        Tick(67),
        "布景前提:片 B 创建于全局拍 67(奇)"
    );
    session.register(agent_sim(shard_id(1, 0)));
    // 本体 8×8(chunk x 8..16)+ A 侧重叠列(x = 8;B 未注册时 A 不能装)。
    // +x 方向不扩(chunk 16 的 owner (2,0) 不在布景内;主体过界后随即反向,
    // 不靠近 B 本体右缘 256)。
    session.apply_chunk_load(shard_id(0, 0), ChunkPos::new(8, 0), flat_ground_chunk());
    for cx in 8..16i32 {
        for cz in 0..8i32 {
            session.apply_chunk_load(shard_id(1, 0), ChunkPos::new(cx, cz), flat_ground_chunk());
        }
    }
    // 主体继续 +x 走进 B(5.6 m/s × ~90 拍 ≈ 8.4 m,从 120 到 128+);
    // 之后反向走回 A。逐拍采样。R1-P4:直接断言分片序列 A → B → A
    // (不再"由 A04-1 的同型布景保证")。
    let mut shard_seq: Vec<ShardId> = vec![shard_id(0, 0)];
    let mut prev = session.walker_hold_of(a);
    for _ in 0..100usize {
        session.tick();
        let done = session.tick_now().0 - 1;
        let cur = session.walker_hold_of(a);
        if cur.is_some() && prev.is_some() && cur != prev {
            changes.push(done);
        }
        prev = cur;
        // 过界后(B 片)反向:一次性翻转意图(保持状态语义,发一次即可;
        // 迁移后信封写回的就是翻转后的)。
        let t = session.tick_now().0;
        if let Some(b) = agent_shard_of(&session, a)
            && *shard_seq.last().expect("非空") != b
        {
            shard_seq.push(b);
        }
        if t > 90
            && let Some(b) = agent_shard_of(&session, a)
            && b == shard_id(1, 0)
        {
            send_intent(&mut session, b, a, (-5.6, 0.0), 0.0);
            break;
        }
    }
    // 走回段:再采 120 拍(应已跨回 A)。
    let mut prev = session.walker_hold_of(a);
    for _ in 0..120usize {
        session.tick();
        let done = session.tick_now().0 - 1;
        let cur = session.walker_hold_of(a);
        if cur.is_some() && prev.is_some() && cur != prev {
            changes.push(done);
        }
        prev = cur;
        if let Some(b) = agent_shard_of(&session, a)
            && *shard_seq.last().expect("非空") != b
        {
            shard_seq.push(b);
        }
    }
    session.shutdown();
    // 布景自检(R1-P4):主体真的进了 B 再回到 A——分片去重序列恰为
    // A → B → A。
    assert_eq!(
        shard_seq,
        vec![shard_id(0, 0), shard_id(1, 0), shard_id(0, 0)],
        "布景自检:分片序列必须 A → B → A(主体先进 B 再走回)"
    );
    // 布景自检:确实发生了跨片(主体曾出现在 B)——用 hold 变化数与迁移
    // 事实(奇拍注册片 B 后主体走进去再走回来)的自证:changes 非空说明
    // 控制器在跑。
    assert!(
        !changes.is_empty(),
        "布景自检:全程保持力变化 > 0 次(控制器在跑)"
    );
    let odd: Vec<u64> = changes.iter().filter(|t| *t % 2 != 0).copied().collect();
    assert!(
        odd.is_empty(),
        "控制拍必须恒落在全局偶数拍:奇数变化拍 {odd:?}(共 {} 变化)",
        changes.len()
    );
}

// ---- 判据 A04-5:迁移那一拍不丢力 ----

/// run() 的采样结果(精确口径,R1-P5):
/// (重建拍全局拍号, 两拍合并增量 dv_mig, 期望增量 dv_expect, 重建拍末保持态
///   (Fx, Fz, last_yaw), 重建拍偶 = 控制拍重建标记)。
type Mig5Sample = (u64, f32, f32, (f32, f32, f32), bool);

/// 口径(R1-P5,精确):主体以 5.6 m/s 意图从紧贴边界的加速区起跑,迁移
/// 落在加速阶段。消失拍 N(step 内原子移除,**移除前该拍控制器已施力**)
/// 与重建拍 N+1(apply_at = N+1 前缀重建 + 本拍 step 施力)两拍合并的速度
/// 增量 `dv_mig` = v(N+1 末) − v(N−1 末)(N 拍不可见,无采样)必须等于
/// 两拍施力之和 `(F_N + F_{N+1})·dt/m`,误差 ≤ 2%:
/// - 非控制拍重建(重建拍奇偶与消失拍相反的相位组):F_N = F_{N+1} =
///   重建拍末 `walker_hold_of`(保持力两拍不变——控制律只在控制拍重算);
/// - 控制拍重建:F_N = 消失前最后可见拍末的保持力,F_{N+1} = 重建拍末
///   保持力(控制拍重算)。
///
/// 质量 m 从身体定义读(`capsule` link.mass × `agent_mass_scale`),不写死。
/// 保持力减半的变异使 `dv_mig` 与期望差 ~50% > 2%,必红。
#[test]
fn a04_5_migration_tick_applies_held_force() {
    let dt = 1.0 / 60.0f32;
    // 质量:身体定义 link.mass × 全局质量系数(默认 1/100)。
    let cfg_scale = physics_sim::ShardSimConfig::new(shard_id(0, 0)).agent_mass_scale;
    let link_mass = capsule_registry()
        .get(&capsule_hash())
        .expect("capsule 已装")
        .links[0]
        .mass;
    let mass = link_mass * cfg_scale;
    let run = |x0: f32| -> Option<Mig5Sample> {
        let mut session = four_shard_session();
        let a = EntityId::from_parts(1, 1);
        spawn_and_settle(&mut session, shard_id(0, 0), a, Vec3::new(x0, 1.95, 24.0));
        send_intent(&mut session, shard_id(0, 0), a, (5.6, 0.0), 0.0);
        let mut migrate_tick: Option<u64> = None;
        let mut dv_mig = 0.0f32;
        let mut dv_expect = 0.0f32;
        let mut hold_after = None;
        let mut prev_vx = 0.0f32;
        // 消失前最后可见拍末的保持力(控制拍重建相位的 F_N)。
        let mut hold_last_visible: Option<(f32, f32, f32)> = None;
        for _ in 0..400usize {
            session.tick();
            let now = session.tick_now().0 - 1;
            match agent_shard_of(&session, a) {
                // 消失拍(tick N 末原子移除,拍内施力已施加):主体不可见,
                // 不采样;消失前的最后可见拍末保持力已记 hold_last_visible。
                None => {}
                Some(s) => {
                    let st = session.entity_state(s, a).expect("在场");
                    if s == shard_id(1, 0) && migrate_tick.is_none() {
                        // 重建拍(apply_at = N+1 前缀重建 + 本拍 step):
                        // dv_mig = v(N+1 末) − v(N−1 末)(两拍合并增量)。
                        dv_mig = st.linvel.x - prev_vx;
                        migrate_tick = Some(now);
                        hold_after = session.walker_hold_of(a);
                        let hold_n1 = hold_after.expect("重建拍末保持态在场");
                        // 相位归类:重建拍 = 全局偶数拍 → 控制拍重算过
                        // (F_N 用消失前最后可见拍的保持力);奇数 → 非控制
                        // 拍(保持力沿用,F_N = F_{N+1})。
                        let f_n = if now.is_multiple_of(2) {
                            hold_last_visible.expect("消失前最后可见拍末保持态在场").0
                        } else {
                            hold_n1.0
                        };
                        dv_expect = (f_n + hold_n1.0) * dt / mass;
                    } else if migrate_tick.is_none() {
                        // 源片可见拍:记速度与保持力(消失拍施的力 =
                        // 本拍末保持的力)。
                        prev_vx = st.linvel.x;
                        if let Some(h) = session.walker_hold_of(a) {
                            hold_last_visible = Some(h);
                        }
                    }
                }
            }
            if migrate_tick.is_some() {
                break;
            }
        }
        session.shutdown();
        migrate_tick.map(|t| {
            (
                t,
                dv_mig,
                dv_expect,
                hold_after.expect("重建拍末保持态在场"),
                t % 2 == 0,
            )
        })
    };
    let mut even: Option<Mig5Sample> = None;
    let mut odd: Option<Mig5Sample> = None;
    // 加速区窗口:5.6 m/s 意图从静止到满速 τ=0.2 s(~12 拍);1.2~3.3 m
    // 行程 ~15~30 拍,迁移落在加速末段(保持力仍 > 死区)。步长 0.3 m ≈
    // 2~3 拍,保证 8 档覆盖迁移重建拍的奇偶两相位。R1-P5:两相位都必须
    // 采到(采不到 = 布景不合格,panic,不许打印跳过)。
    for k in 0..8usize {
        let x0 = 126.8 - 0.3 * k as f32;
        let Some((tick, dv, expect, hold, is_even)) = run(x0) else {
            continue;
        };
        if is_even && even.is_none() {
            even = Some((tick, dv, expect, hold, is_even));
        }
        if !is_even && odd.is_none() {
            odd = Some((tick, dv, expect, hold, is_even));
        }
    }
    for (label, hit) in [
        ("迁移在控制拍(重建拍偶)", even),
        ("迁移在非控制拍(重建拍奇)", odd),
    ] {
        let Some((tick, dv, expect, hold, _)) = hit else {
            panic!("A04-5 {label}:扫 8 个起跑点未采到该相位(布景不合格,R1-P5:两相位都必须采到)");
        };
        assert!(
            hold.0 != 0.0 || hold.1 != 0.0,
            "A04-5 {label}:迁移拍(全局拍 {tick})重建后目的分片保持力为零              (WalkerHold 未随信封写回)"
        );
        let err = (dv - expect).abs() / expect.abs().max(1e-9);
        assert!(
            err <= 0.02,
            "A04-5 {label}:迁移拍(全局拍 {tick})两拍合并增量 {dv:.6} vs 期望 \
             (F_N + F_N1)·dt/m = {expect:.6},相对误差 {err:.4} > 2%(丢力或双倍力)"
        );
        println!(
            "A04-5 {label}:重建拍 {tick} dv {dv:.6} vs (F_N+F_N1)·dt/m {expect:.6} \
             误差 {err:.4} hold=({:.3},{:.3}) m={mass} ✓",
            hold.0, hold.1
        );
    }
}

// ---- 判据 A04-6:到了目的分片仍是主体 ----

/// 过界后站 3600 拍(R1-P6):观察者移到唤醒范围外(`awake_unload_radius`
/// 默认 32 m——主体在 x≈128+ 刹停,观察者放 (60,3,24) 距主体 >68 m,出
/// 唤醒范围)证明"不被预算冻结"——可冻结的主体同样不下沉,只查下沉是假
/// 判据;站满 3600 拍后发一条行走意图,断言 60 拍内水平速度达到意图的
/// 95% 以上(仍是动态刚体、控制器还在跑);`entity_owner(a).actor` 为真、
/// is_agent、EntityStates 不含、entity_owner 指向目的分片。"实心方块数
/// 不变"恒返回 0 是假断言,R1-P6 删掉(计数断言由内核 A01 测试承担)。
#[test]
fn a04_6_agent_persists_at_destination_3600_ticks() {
    let mut session = four_shard_session();
    let a = EntityId::from_parts(1, 1);
    // 出生距边界 4 m(5.6 m/s 加速后 ~20 拍过界,80 拍窗口绰绰有余)。
    spawn_and_settle(
        &mut session,
        shard_id(0, 0),
        a,
        Vec3::new(124.0, 1.95, 24.0),
    );
    // 观察者放在行程全程(出发片 → 目的片),激活开启 = 生产口径。
    session
        .move_observer(Vec3::new(124.0, 3.0, 24.0))
        .expect("观察者入箱");
    send_intent(&mut session, shard_id(0, 0), a, (5.6, 0.0), 0.0);
    let mut dest = None;
    for _ in 0..80usize {
        if tick_and_shard(&mut session, a) == shard_id(1, 0) {
            dest = Some(shard_id(1, 0));
            break;
        }
    }
    let dest = dest.expect("布景约束:80 拍内过界");
    send_intent(&mut session, dest, a, (0.0, 0.0), 0.0);
    // 观察者移到唤醒范围外(距刹停点 > 68 m > awake_unload_radius 32 m):
    // 主体在无观察者的区域站 3600 拍。
    session
        .move_observer(Vec3::new(60.0, 3.0, 24.0))
        .expect("观察者入箱");
    for _ in 0..3600usize {
        session.tick();
    }
    let st = session
        .entity_state(dest, a)
        .expect("过界 3600 拍后主体仍在(不凝固)");
    assert!(session.is_agent(a), "is_agent 为真");
    assert!(
        (st.translation.y - 1.9).abs() <= 0.01,
        "站立下沉 ≤ 1 cm(y={})",
        st.translation.y
    );
    assert!(
        st.translation.x >= 128.0 && st.translation.x <= 134.0,
        "刹车站稳在过界后附近(x={},>128 已过界;零意图刹车行程 < 6 m)",
        st.translation.x
    );
    let listed = session.entity_states(dest).expect("目的分片已注册");
    assert!(
        !listed.iter().any(|(id, _)| *id == a),
        "EntityStates 不含主体"
    );
    let owner = session.entity_owner(a).expect("owner 在表");
    assert_eq!(owner.shard, dest, "entity_owner 指向目的分片");
    assert!(owner.actor, "entity_owner(a).actor 为真(演员政策标记保留)");
    // R1-P6:不被预算冻结的本体判据——发一条行走意图,60 拍内水平速度
    // 达到意图的 95% 以上(仍是动态刚体、控制器还在跑)。意图发往主体
    // 所在分片。
    send_intent(&mut session, dest, a, (4.3, 0.0), 0.0);
    let mut reached = None;
    for _ in 0..60usize {
        session.tick();
        let st = session.entity_state(dest, a).expect("主体在场");
        let speed = (st.linvel.x * st.linvel.x + st.linvel.z * st.linvel.z).sqrt();
        if speed >= 4.3 * 0.95 {
            reached = Some(speed);
            break;
        }
    }
    let reached =
        reached.expect("A04-6:60 拍内水平速度未达到意图 4.3 × 95%(主体被预算冻结或控制器不在跑)");
    println!("A04-6:唤醒范围外站 3600 拍后行走意图 60 拍内水平速度达 {reached:.3} m/s(≥ 95%)");
    session.shutdown();
}

// ---- 判据 A04-7:被推过界 ----

/// 爆炸把站着的主体高速推过边界:爆炸中心在主体正西 2 m(冲量沿 +x),
/// power 20 / radius 6。断言:无 panic、无 NaN、相邻拍位移 ≤ 该拍实际速度
/// × dt × 1.5、确实过界。
#[test]
fn a04_7_pushed_across_boundary_by_explosion() {
    let dt = 1.0 / 60.0f32;
    let mut session = four_shard_session();
    let a = EntityId::from_parts(1, 1);
    spawn_and_settle(
        &mut session,
        shard_id(0, 0),
        a,
        Vec3::new(124.0, 1.95, 24.0),
    );
    session
        .move_observer(Vec3::new(124.0, 3.0, 24.0))
        .expect("观察者入箱");
    session
        .detonate(Vec3::new(122.0, 1.0, 24.0), 20.0, 6.0)
        .expect("爆炸入箱");
    let mut prev_pos = Vec3::new(124.0, 1.95, 24.0);
    let mut crossed = false;
    let mut prev_shard = agent_shard_of(&session, a);
    let mut gap; // 采样间隔拍数(上一拍在途窗口 = 2,否则 1)。
    for t in 0..300usize {
        // 迁移在途窗口:恰一拍 None(信封 N+1 达)。
        session.tick();
        gap = 1;
        let shard = match agent_shard_of(&session, a) {
            Some(s) => s,
            None => {
                session.tick();
                gap = 2;
                agent_shard_of(&session, a).expect("迁移窗口恰一拍(不丢)")
            }
        };
        if prev_shard.is_some() && Some(shard) != prev_shard {
            // 迁移发生:本拍位置跨片连续(无跳变由位移断言承担)。
        }
        prev_shard = Some(shard);
        let st = session.entity_state(shard, a).expect("主体在场");
        let pos = st.translation;
        let v = st.linvel;
        assert!(
            pos.x.is_finite()
                && pos.y.is_finite()
                && pos.z.is_finite()
                && v.x.is_finite()
                && v.y.is_finite()
                && v.z.is_finite(),
            "拍 {t}:位置或速度非有限(NaN/inf)"
        );
        let step = (pos - prev_pos).length();
        let vmax_now = v.length();
        assert!(
            step <= vmax_now * dt * 1.5 * gap as f32 + 1e-4,
            "拍 {t}:相邻拍位移 {step:.4} > 实际速度 {vmax_now:.4} × dt × 1.5 × {gap}\
             (间隔拍数)"
        );
        if pos.x >= 128.0 {
            crossed = true;
        }
        prev_pos = pos;
    }
    assert!(crossed, "布景自检:主体应被推过边界(x ≥ 128)");
    session.shutdown();
}

// ---- 判据 A04-8:确定性 ----

/// 时间线(R1-P7 调整,真正斜穿交汇点 (128,128)):拍 0 出生 (110,110);
/// 拍 60 对角意图 (+,+) 5.6(沿对角线走过 (128,128) 进片 (1,1));拍 200
/// 空中跳(方向保持);拍 500 反向对角 (−5.6,+5.6)(跨 x = 128 进片
/// (0,1))。分片序列覆盖 (0,0) → (1,1) → (0,1) ≥ 3 个(断言)。命令发
/// 往主体**当前所在分片**(迁走后发原片会被驱动侧丢弃,意图不生效)。
/// Session 层回放两次(世界快照 + 命令日志),每拍主体状态逐位相同。
#[test]
fn a04_8_determinism_replay_and_checkpoint() {
    let a = EntityId::from_parts(1, 1);
    fn feed(
        session: &mut Session,
        log: &mut sim_api::log::CommandLog,
        t: u32,
        a: EntityId,
        at: ShardId,
    ) {
        let cmd = match t {
            60 => Some(SimCommand::AgentIntent {
                entity: a,
                move_xz: (5.6, 5.6),
                yaw: yaw_of(5.6, 5.6),
                jump: false,
            }),
            200 => Some(SimCommand::AgentIntent {
                entity: a,
                move_xz: (5.6, 5.6),
                yaw: yaw_of(5.6, 5.6),
                jump: true,
            }),
            500 => Some(SimCommand::AgentIntent {
                entity: a,
                move_xz: (-5.6, 5.6),
                yaw: yaw_of(-5.6, 5.6),
                jump: false,
            }),
            _ => None,
        };
        if let Some(c) = cmd {
            let e = session.send(at, c).expect("入箱");
            log.append(e);
        }
    }
    fn run(
        session: &mut Session,
        log: &mut sim_api::log::CommandLog,
        a: EntityId,
    ) -> (Vec<(Vec3, Vec3)>, Vec<ShardId>) {
        // 生成与观察者都走正路盖章:LogEntry 落命令日志(回放线由此再生)。
        let e = spawn_agent(session, shard_id(0, 0), a, Vec3::new(110.0, 1.95, 110.0));
        log.append(e);
        let eo = session
            .move_observer(Vec3::new(128.0, 3.0, 128.0))
            .expect("入箱");
        log.append(eo);
        let mut trace = Vec::new();
        // 逐拍主体所在分片(在途窗口占位上一拍;R1-P7 分片覆盖断言用)。
        let mut shard_timeline: Vec<ShardId> = Vec::new();
        let mut cur = shard_id(0, 0);
        for t in 0..900u32 {
            feed(session, log, t, a, cur);
            session.tick();
            let found = session
                .shard_ids()
                .iter()
                .find_map(|s| session.entity_state(*s, a).map(|st| (*s, st)));
            match found {
                Some((s, st)) => {
                    cur = s;
                    trace.push((st.translation, st.linvel));
                }
                // 迁移在途窗口恰一拍:占位上一拍(两线同窗口同占位)。
                None => trace.push(*trace.last().unwrap_or(&(Vec3::ZERO, Vec3::ZERO))),
            }
            shard_timeline.push(cur);
        }
        (trace, shard_timeline)
    }
    // 原线(快照取在拍 0 的时间线开始前)。
    let mut golden = four_shard_session();
    let golden_snap = golden.world_snapshot();
    let mut golden_log = sim_api::log::CommandLog::new();
    let (golden_trace, golden_shards) = run(&mut golden, &mut golden_log, a);
    golden.shutdown();
    assert_eq!(golden_trace.len(), 900);
    // 布景自检(R1-P7):时间线确实发生迁移(存在占位窗口)、行程过界、
    // 主体先后出现在 ≥ 3 个分片(含片 (1,1) = 斜穿交汇点的证据)。
    assert!(
        golden_trace.iter().any(|(p, _)| p.x > 128.0 || p.z > 128.0),
        "布景自检:时间线应跨过边界"
    );
    let mut seen: std::collections::BTreeSet<ShardId> = std::collections::BTreeSet::new();
    seen.extend(golden_shards.iter().copied());
    assert!(
        seen.len() >= 3,
        "布景自检:主体应先后出现在至少 3 个分片(实为 {seen:?})"
    );
    assert!(
        seen.contains(&shard_id(1, 1)),
        "布景自检:主体应斜穿交汇点进入片 (1,1)(实为 {seen:?})"
    );

    // 回放线 1:replay_world_with_bodies(生产恢复路径)。
    let mut replay1 = sim_api::replay::replay_world_with_bodies(
        &golden_snap,
        &golden_log,
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        },
        || Box::new(SimpleRegistry::builtin()),
        capsule_registry,
    )
    .expect("重放装配成功");
    // 回放线照常跑完 900 拍(不 feed:命令已在日志里,由 replay 投递)。
    let mut trace1 = Vec::new();
    let _ = &mut replay1;
    {
        let a = EntityId::from_parts(1, 1);
        for _ in 0..900u32 {
            replay1.tick();
            let st = replay1
                .shard_ids()
                .iter()
                .find_map(|s| replay1.entity_state(*s, a));
            trace1.push(match st {
                Some(s) => (s.translation, s.linvel),
                None => *trace1.last().unwrap_or(&(Vec3::ZERO, Vec3::ZERO)),
            });
        }
    }
    replay1.shutdown();
    for (t, (x, y)) in golden_trace.iter().zip(trace1.iter()).enumerate() {
        assert_eq!(x, y, "回放线:拍 {t} 主体状态逐位不同");
    }
}

// ---- 判据 A04-8 后半:检查点(迁移拍/之后一拍)恢复续跑逐位 ----

/// 检查点落在消失拍 N 末(信封在途)与之后一拍 N+1(R1-P7:原实现取在
/// 主体首次出现在新分片的那一拍 = 重建之后,信封已不在途;消失拍 N 末
/// 主体在任何分片都查不到、信封在 Router 里——断言这两个事实)。恢复
/// (new Session + 调度器快照 restore_state + 分片快照
/// register_snapshot_with_bodies)后续跑时间线,与不中断的 golden 线逐位
/// 相同。走 Session/调度器层。时间线与 a04_8 前半同款(斜穿 (128,128),
/// ≥ 3 分片;命令发往主体当前所在分片)。
#[test]
fn a04_8_checkpoint_resume_bitwise() {
    // 时间线生成器:头段到 cp_tick 为止用 log_b 的输入,之后用 resume_log
    // 继续写(恢复线 = 快照 + 后续日志)。`at` = 主体当前所在分片。
    fn feed(
        session: &mut Session,
        log: &mut sim_api::log::CommandLog,
        t: u32,
        a: EntityId,
        at: ShardId,
    ) {
        // 时间线命令拍 60/200/500;其它拍无命令。feed 由调用方保证 t 不早于
        // 恢复点(续跑线重发已生效的命令 = 迟到输入 fail-stop)。
        let cmd = match t {
            60 => Some((SimCommand::AgentIntent {
                entity: a,
                move_xz: (5.6, 5.6),
                yaw: yaw_of(5.6, 5.6),
                jump: false,
            },)),
            200 => Some((SimCommand::AgentIntent {
                entity: a,
                move_xz: (5.6, 5.6),
                yaw: yaw_of(5.6, 5.6),
                jump: true,
            },)),
            500 => Some((SimCommand::AgentIntent {
                entity: a,
                move_xz: (-5.6, 5.6),
                yaw: yaw_of(-5.6, 5.6),
                jump: false,
            },)),
            _ => None,
        };
        if let Some((c,)) = cmd {
            let e = session.send(at, c).expect("入箱");
            log.append(e);
        }
    }
    let a = EntityId::from_parts(1, 1);
    // golden 线 + 迁移拍探测。
    let mut golden = four_shard_session();
    let mut golden_log = sim_api::log::CommandLog::new();
    let eo = golden
        .move_observer(Vec3::new(128.0, 3.0, 128.0))
        .expect("入箱");
    golden_log.append(eo);
    let es = spawn_agent(
        &mut golden,
        shard_id(0, 0),
        a,
        Vec3::new(110.0, 1.95, 110.0),
    );
    golden_log.append(es);
    let mut migrated_at: Option<u32> = None;
    let mut cur = shard_id(0, 0);
    let mut golden_trace: Vec<(Vec3, Vec3)> = Vec::new();
    let mut cp_scheds: Vec<shard::SchedulerSnapshot> = Vec::new();
    let mut cp_shards: Vec<sim_api::snapshot::GlobalSnapshot> = Vec::new();
    for t in 0..900u32 {
        feed(&mut golden, &mut golden_log, t, a, cur);
        golden.tick();
        let found = golden
            .shard_ids()
            .iter()
            .find_map(|sh| golden.entity_state(*sh, a).map(|st| (*sh, st)));
        match found {
            Some((c, st)) => {
                cur = c;
                golden_trace.push((st.translation, st.linvel));
            }
            None => {
                // 消失拍 N(tick N 末原子移除):R1-P7 检查点 1 取在此拍末
                // ——主体在任何分片都查不到、信封在 Router 里(在途)。
                if migrated_at.is_none() {
                    migrated_at = Some(t);
                    let mut still_visible = false;
                    for sh in golden.shard_ids() {
                        if golden.entity_state(sh, a).is_some() {
                            still_visible = true;
                        }
                    }
                    assert!(
                        !still_visible,
                        "检查点 1 布景自检:消失拍 N 末主体必须任何分片都查不到"
                    );
                    let snap = golden.world_snapshot();
                    let in_transit: Vec<_> = snap
                        .scheduler
                        .router
                        .envelopes
                        .iter()
                        .filter(|env| {
                            matches!(env.payload, messaging::EnvelopePayload::MigrateAgent { .. })
                        })
                        .collect();
                    assert!(
                        !in_transit.is_empty(),
                        "检查点 1 布景自检:消失拍 N 末信封必须在 Router 里(MigrateAgent 在途)"
                    );
                    cp_scheds.push(snap.scheduler.clone());
                    cp_shards.push(snap.shards.clone());
                }
                // 迁移在途窗口:占位(与续跑线同窗口同占位)。
                let last = *golden_trace.last().unwrap_or(&(Vec3::ZERO, Vec3::ZERO));
                golden_trace.push(last);
            }
        }
        // 检查点 2:重建拍 N+1(t = migrated_at + 1 完成)。
        if Some(t) == migrated_at.map(|m| m + 1) {
            let snap = golden.world_snapshot();
            cp_scheds.push(snap.scheduler.clone());
            cp_shards.push(snap.shards.clone());
        }
    }
    let m = migrated_at.expect("布景约束:900 拍内应发生迁移");
    golden.shutdown();
    assert!(golden_trace.len() == 900);
    // 布景自检(R1-P7):时间线与 a04_8 前半同款(斜穿 (128,128)、覆盖
    // ≥ 3 分片,前半已断言);此处断言主体末段行程已跨过 z = 128(反向
    // 对角段把主体带进片 (0,1)),证明时间线真实走完 500 拍之后的行程。
    let last = golden_trace.last().expect("非空");
    assert!(
        last.0.z >= 128.0,
        "布景自检:时间线末段应已跨过 z = 128(消失拍 {m},末位 {},{})",
        last.0.x,
        last.0.z
    );

    // 两个检查点分别恢复续跑:快照恢复(coordinator 对齐 + 全部 4 片按
    // 分片快照注册——调度器快照键集须一致)+ 之后的时间线输入逐拍续跑,
    // 与 golden 逐位比。
    for (k, sched_snap) in cp_scheds.iter().enumerate() {
        let resume_from = sched_snap.tick;
        // 恢复:coordinator 起点对齐快照拍(用 SchedulerConfig.start_tick),
        // 分片经 register_snapshot_with_bodies 注册(快照里的 step_tick 非
        // 零 → 沿用,restore_state 断言一致)。
        let cfg = SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                start_tick: resume_from,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        };
        let mut s = Session::new(cfg);
        for bytes in cp_shards[k].per_shard.values() {
            s.register_snapshot_with_bodies(
                bytes,
                Box::new(SimpleRegistry::builtin()),
                capsule_registry(),
            )
            .expect("分片快照恢复");
        }
        s.restore_scheduler_state(sched_snap.clone());
        // 恢复线从快照拍继续:时间线命令拍(60/200/500)中 < resume_from 的
        // 已烙进快照,重发会触发"迟到输入" fail-stop;只续发 ≥ resume_from
        // 的。`cur` 沿 golden 线的所在片:恢复点的主体所在片 = 快照时刻
        // Router 信封载荷的目的片(检查点 1 在消失拍 N 末,主体"将在" N+1
        // 出现在目的片;检查点 2 在 N+1 末,主体已在目的片)。两线逐位一致
        // 的前提是续跑命令与 golden 在同一拍发往同一片——检查点 1 恢复线
        // 的 cur 从信封载荷读,检查点 2 从主体实际所在片读。
        let mut resume_cur = if k == 0 {
            // 消失拍 N 末:信封在途,主体目的片 = Router 里 MigrateAgent
            // 信封的目的分片(与 golden 线 N+1 拍所在片一致)。
            cp_scheds[k]
                .router
                .envelopes
                .iter()
                .find_map(|env| match &env.payload {
                    messaging::EnvelopePayload::MigrateAgent { entity, .. } if *entity == a => {
                        Some(env.destination)
                    }
                    _ => None,
                })
                .expect("检查点 1:Router 必有该主体的在途信封")
        } else {
            // 重建拍 N+1 末:重新注册的快照里查主体所在片。
            s.shard_ids()
                .iter()
                .find_map(|sh| s.entity_state(*sh, a).map(|_| *sh))
                .expect("检查点 2:重建拍末主体已在目的分片")
        };
        let mut trace: Vec<(Vec3, Vec3)> = Vec::new();
        // 快照之前的 trace 段:从 golden_trace 复制(两线逐位一致的前提段)。
        let skip = resume_from.0 as usize;
        trace.extend_from_slice(&golden_trace[..skip]);
        for t in (resume_from.0 as u32)..900u32 {
            feed(
                &mut s,
                &mut sim_api::log::CommandLog::new(),
                t,
                a,
                resume_cur,
            );
            s.tick();
            let found = s
                .shard_ids()
                .iter()
                .find_map(|sh| s.entity_state(*sh, a).map(|st| (*sh, st)));
            match found {
                Some((c, st)) => {
                    resume_cur = c;
                    trace.push((st.translation, st.linvel));
                }
                None => trace.push(*trace.last().unwrap_or(&(Vec3::ZERO, Vec3::ZERO))),
            }
        }
        s.shutdown();
        for (t, (x, y)) in golden_trace.iter().zip(trace.iter()).enumerate() {
            assert_eq!(
                x,
                y,
                "检查点 {}({} ,恢复自拍 {}):拍 {t} 逐位不同",
                resume_from.0,
                if k == 0 {
                    "消失拍 N 末(信封在 Router)"
                } else {
                    "重建拍 N+1"
                },
                resume_from.0
            );
        }
    }
}

// ---- 判据 A04-9:MARGIN 校验 ----

/// 包络超过 MARGIN 推导上限的身体被拒(类型化 `AppError::Margin`,拒绝
/// 启动):身体 = 半径 15.5 的球(margin 16 ≥ 15.5 + ε 勉强够)→ 换
/// 半径 16.1 的球必超 16 m margin(16.1 + 5.6·dt + 0.05 > 16)。
/// 布景:临时目录写 body.toml,load_body_registry(经 Config)返回 Err。
/// 正对照:默认 capsule(包络 0.9)启动成功(Err 为 None)。
#[test]
fn a04_9_margin_rejects_oversized_body() {
    use app::config::Config;
    use app::driver::{AppError, load_body_registry};
    // 超限身体:球半径 16.1(包络 = 半径 16.1;16 < 16.1 + 5.6/60 + 0.05)。
    let oversized = r#"
name = "big"
[[links]]
name = "core"
mass = 80.0
[[links.shapes]]
kind = "sphere"
radius = 16.1
"#;
    let dir = tempfile::tempdir().expect("临时目录");
    let bodies_dir = dir.path().join("bodies");
    std::fs::create_dir_all(&bodies_dir).expect("建目录");
    std::fs::write(bodies_dir.join("big.body.toml"), oversized).expect("写 body.toml");
    let config = Config {
        bodies_dir: bodies_dir.clone(),
        ..Config::default()
    };
    let err = load_body_registry(&config).expect_err("超限身体必须被拒");
    match err {
        AppError::Margin {
            margin_m,
            required_m,
            envelope_m,
            ..
        } => {
            assert_eq!(margin_m, 16.0, "margin = 默认 16");
            assert!(envelope_m > 16.0, "包络 = 球半径 {envelope_m}(超限布景)");
            assert!(
                required_m > margin_m,
                "必需值 {required_m} > margin {margin_m}(类型化错误携带推导)"
            );
        }
        other => panic!("必须是 AppError::Margin,实为 {other:?}"),
    }
    // 正对照:默认 capsule(包络 0.9)启动成功。
    let mut config_ok = Config {
        bodies_dir: std::path::PathBuf::from("assets/bodies"),
        ..Config::default()
    };
    // e2e 测试 cwd = workspace 根(dev-dependency;与既有 include_str 同基准)。
    if !config_ok.bodies_dir.exists() {
        // 相对路径不可用时按仓库根定位(workspace 布局)。
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("e2e 在 crates 下")
            .parent()
            .expect("crates 在根下")
            .to_path_buf();
        config_ok.bodies_dir = root.join("assets/bodies");
    }
    let bodies = load_body_registry(&config_ok).expect("默认 capsule 包络 0.9 远小于 16");
    assert!(!bodies.is_empty(), "capsule 已加载");
}
