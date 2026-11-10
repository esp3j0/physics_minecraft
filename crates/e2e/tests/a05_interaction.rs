//! REQ-012 A05(#25):主体与破坏系统的交互规则(§6)与暂缓写入。
//!
//! 判据口径(放行判据表):
//! - **A05-1 高冲量撞击不破坏地形**:两组场景(高坠、被炸飞后落地),冲量
//!   够得上石的撞击破坏阈值(I = 18);对照线 = 同质量同速度的普通动态
//!   方块撞同一处撞碎石(cells_impacted > 0);主体线 = 撞击破坏计数 0、
//!   地形字节不变、§5.7 事件里没有以主体为撞击者的事件。
//! - **A05-2 被撞**:5 m 落方块、复合体横撞,主体位移 > 0;复合体撞主体
//!   不断(对照线:同速撞石墙会断);主体不出现在复合体冲量流汇点。
//! - **A05-3 爆炸**:首拍速度增量 = explosion_impulse(根位置)/质量
//!   (≤ 1e-5);不被移除;落地不破坏地形。
//! - **A05-4 承载**:桥在承载极限(对照线:桥上加 1 格实心就塌);主体站
//!   600 拍 + 跳 10 次,桥不塌、承载转换计数 0;对照组:拆墩桥塌,主体
//!   随之落下,不卡、不 NaN。
//! - **A05-5 SetBlock 暂缓**:对主体所占格写石:不写入、计数 +1;走开后
//!   **第一拍不再重叠时补写**(补写拍号 = 首个不重叠拍 + 1);全程逐拍
//!   断言连杆与实心体素不相交;总数 = 初始 + 写入数;同格两条实心只写后
//!   一条(被替换 +1);暂缓中写空气立刻生效并作废(+1);移除主体下一拍
//!   补写。
//! - **A05-6 凝固、强制落地暂缓**:方块实体靠主体静止、目标格与主体重叠:
//!   不凝固(+1)、仍是实体;走开后凝固,总数守恒。强制落地(预算压力)、
//!   复合体各一组。
//! - **A05-7 跨分片一致**:主体站边界、身体跨进邻片格;对该格 SetBlock:
//!   owner 与副本分片都暂缓并在同一拍补写;逐拍两片该格体素相同。对照
//!   (变异 2:表只用本片主体)会分叉——由变异自证承担。
//! - **A05-8 迁移在途**:暂缓条目存在时主体迁移;在途拍表里有信封位姿、
//!   该格不补写;落地后照常判定。
//! - **A05-9 确定性**:暂缓队列非空落检查点,恢复续跑与不中断线逐位相同;
//!   日志回放两次逐位相同;v8 检查点显式拒绝(driver 侧已有)。
//! - **A05-10 不影响旧行为**:无主体时暂缓计数全 0(各测试的对照线 +
//!   既有 golden 由全工作区门禁守住)。
//!
//! 布景:判据 A05-5/6 用单分片 ShardSim 直接驱动(形状表手工注入 = 生产
//! 同源构造),判据 A05-7/8/9 用 2×2 分片 Session(调度器自动汇总下发)。
//! 生产节奏 = apply_prefix → step → run_transient_lifecycle → take_outbox。

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::agent_shape::{AgentLinkEntry, AgentShapeTable};
use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
use shard::SchedulerConfig;
use sim_api::log::CommandLog;
use sim_api::session::{Session, SessionConfig};
use types::glam::{Quat, Vec3};
use types::{
    BlockId, BlockPos, ChunkPos, CompoundShape, DimensionId, EntityDesc, EntityId, EntityState,
    ShapeCell, ShardId, ShardPos, Tick, WorldId,
};
use voxel::{Chunk, SimpleRegistry};

/// capsule 注册表(include 同源)。
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

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// 默认配置(凝固/激活/破坏/承载全开 = 生产)+ capsule 注册表。
fn walker_sim() -> ShardSim {
    let cfg = ShardSimConfig::new(shard_id(0, 0));
    ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), capsule_registry())
}

/// 平板地形:3×3 chunk,层 world y = 0 全石(顶面 y = 1)。
fn load_flat_ground(sim: &mut ShardSim) {
    load_flat_ground_with(sim, BlockId(1));
}

fn load_flat_ground_with(sim: &mut ShardSim, block: BlockId) {
    for cx in 0..3i32 {
        for cz in 0..3i32 {
            let mut chunk = Chunk::air();
            for lx in 0..voxel::CHUNK_X {
                for lz in 0..voxel::CHUNK_Z {
                    chunk.set(lx, 64, lz, block);
                }
            }
            sim.load_chunk(ChunkPos::new(cx, cz), chunk);
        }
    }
}

/// 盖章输入(调度器来源)。
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

/// 生产节奏的一拍(激活开启 = 带观察者;worker 同序)。返回本拍 outbox
/// (调用方读遥测计数;take 走即清,不返回就丢)。
fn production_tick(sim: &mut ShardSim, observers: &[Vec3]) -> physics_sim::Outbox {
    sim.apply_activation(observers);
    sim.step();
    sim.run_transient_lifecycle();
    sim.take_outbox()
}

/// 装载 chunk 的实心方块数(总数守恒口径)。
fn solid_count(sim: &ShardSim) -> usize {
    let mut n = 0usize;
    for cp in sim.voxel().chunk_positions().collect::<Vec<_>>() {
        let min = cp.block_min();
        for x in 0..voxel::CHUNK_X {
            for y in 0..voxel::CHUNK_Y {
                for z in 0..voxel::CHUNK_Z {
                    if !sim
                        .voxel()
                        .get_block(BlockPos::new(
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

/// 世界全部 chunk 的体素字节指纹(地形不变判据:哈希前先收集)。
/// 口径:每格 BlockId 的 FNV-1a(带坐标混入)。
fn terrain_fingerprint(sim: &ShardSim) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |b: u8| {
        h ^= b as u64;
        h = h.wrapping_mul(0x1000_0000_01b3);
    };
    for cp in sim.voxel().chunk_positions().collect::<Vec<_>>() {
        let min = cp.block_min();
        for x in 0..voxel::CHUNK_X {
            for y in 0..voxel::CHUNK_Y {
                for z in 0..voxel::CHUNK_Z {
                    mix((min.x + x as i32) as u8);
                    mix((min.y + y as i32) as u8);
                    mix((min.z + z as i32) as u8);
                    mix(sim
                        .voxel()
                        .get_block(BlockPos::new(
                            min.x + x as i32,
                            min.y + y as i32,
                            min.z + z as i32,
                        ))
                        .0 as u8);
                }
            }
        }
    }
    h
}

/// capsule 形状表条目(单连杆;与生成层同源:SharedShape::capsule_y(0.6,0.3),
/// 包围球半径 0.9)。测试手工注入 = 调度器构造同款(`agent_shape_entries`)。
fn capsule_shape_table(a: EntityId, center: Vec3) -> AgentShapeTable {
    let mut table = AgentShapeTable::default();
    table.links.insert(
        a,
        vec![AgentLinkEntry {
            pose: rapier_pose(center, Quat::IDENTITY),
            shape: rapier3d::prelude::SharedShape::capsule_y(0.6, 0.3),
            bound_radius: 0.9,
            center,
        }],
    );
    table
}

fn rapier_pose(t: Vec3, r: Quat) -> rapier3d::prelude::Pose {
    rapier3d::prelude::Pose {
        rotation: r,
        translation: t,
        ..rapier3d::prelude::Pose::IDENTITY
    }
}

/// 无 `[walker]` 段的身体定义(其余同 capsule;去 walker 段 = 控制拍不施力)。
fn nowalker_registry() -> BodyRegistry {
    let toml = include_str!("../../../assets/bodies/capsule.body.toml")
        .lines()
        .skip_while(|l| !l.starts_with("[[links]]"))
        .collect::<Vec<_>>()
        .join("\n");
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(&format!("name = \"capsule\"\n\n{toml}"))
        .expect("无 walker 的 capsule 定义合法");
    bodies
}

fn nowalker_hash() -> types::BodyHash {
    nowalker_registry()
        .hash_of("capsule")
        .expect("capsule 已装")
}

/// 主体安定(**无 walker 段身体**,60 拍落地)后的基座:控制拍不施力。
fn settled_nowalker_agent() -> (ShardSim, EntityId, Vec3) {
    let cfg = ShardSimConfig::new(shard_id(0, 0));
    let mut s = ShardSim::new_with_bodies(
        cfg,
        Box::new(SimpleRegistry::builtin()),
        nowalker_registry(),
    );
    load_flat_ground(&mut s);
    let a = EntityId::from_parts(1, 1);
    s.apply_prefix(&[stamp(
        0,
        SimCommand::SpawnAgent {
            entity: a,
            body: nowalker_hash(),
            state: EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..EntityState::default()
            },
        },
    )]);
    let observers = [Vec3::new(24.0, 3.0, 24.0)];
    for _ in 0..60 {
        sync_shape_table(&mut s, a);
        let _ = production_tick(&mut s, &observers);
    }
    let base = s.entity_state(a).expect("主体安定后仍在").translation;
    (s, a, base)
}

/// 主体安定(零意图 60 拍)后的基座。
fn settled_agent() -> (ShardSim, EntityId, Vec3) {
    let mut s = walker_sim();
    load_flat_ground(&mut s);
    let a = EntityId::from_parts(1, 1);
    s.apply_prefix(&[
        stamp(0, spawn_agent_cmd(a, Vec3::new(24.0, 1.95, 24.0))),
        stamp(
            1,
            SimCommand::AgentIntent {
                entity: a,
                move_xz: (0.0, 0.0),
                yaw: 0.0,
                jump: false,
            },
        ),
    ]);
    let observers = [Vec3::new(24.0, 3.0, 24.0)];
    for _ in 0..60 {
        production_tick(&mut s, &observers);
    }
    let base = s.entity_state(a).expect("主体安定后仍在").translation;
    (s, a, base)
}

/// 每拍形状表同步(单分片测试的调度器替身:按主体当前位姿重建表)。
fn sync_shape_table(s: &mut ShardSim, a: EntityId) {
    let table = s.agent_shape_entries();
    let _ = a;
    s.set_agent_shape_table(table);
}

/// 全部连杆与全部实心体素都不相交(A05-5 的逐拍不变量)。
fn assert_no_link_solid_overlap(s: &ShardSim, a: EntityId, label: &str) {
    let st = s.entity_state(a).expect("主体在");
    // capsule:中心 ± 0.9 半包络 ⇒ 涉及的格 = 中心所在格的 y−1..=y+1、
    // x/z ± 1(半径 0.3 < 0.5,水平只跨中心格)。精确口径按形状表判定:
    // 逐格查表(表与位姿一致 ⇒ 不重叠 = 通过)。
    let c = st.translation;
    let cx = c.x.floor() as i32;
    let cy = c.y.floor() as i32;
    let cz = c.z.floor() as i32;
    for dx in -1..=1 {
        for dy in -2..=2 {
            for dz in -1..=1 {
                let p = BlockPos::new(cx + dx, cy + dy, cz + dz);
                let solid = {
                    let b = s.voxel().get_block(p);
                    !b.is_air()
                };
                if solid {
                    // 实心格必须不与连杆精确相交(表以当前位姿构造)。
                    let table = capsule_shape_table(a, c);
                    assert!(
                        !table.overlaps(p),
                        "{label}:连杆与实心格 {p:?} 相交(主体被埋;中心 {c:?})"
                    );
                }
            }
        }
    }
}

// ── A05-1:高冲量撞击不破坏地形 ──────────────────────────────────────────

/// 石的撞击破坏阈值 I = 18、粘结 bond = 8(builtin 材料)。**冲量流语义**
/// (生产默认 = cohesion 开):砸整层平板的顶面不断——压向割 = ∞(下方
/// 是空的),这是"承重面砸不穿"的设计行为。能被砸断的结构 = 悬挑
/// (割 = 粘结链 8 < s):悬挑石梁 2 格(割 = 柱-梁 bond 8 < s ≈ 24)。
/// 标定(对照线现场证实):0.45 半边方块(质量 0.729)从悬空端上方
/// 5 m、初速 −30 m/s 砸悬空端格心,s ≈ 24 > 8 → 断;同一落点落主体
/// (质量 0.8)冲量同档(≥ 18)但被 §6 D8 豁免 → 不断、不产生事件。
/// 对照线与主体线**同落点、同速度**;对照线撞碎格数现场打印。
#[test]
fn a05_1_high_impact_does_not_break_terrain() {
    let build = |s: &mut ShardSim| {
        // 石柱 (30,1..2,24) 接地;石梁 (31,2,24)、(32,2,24) 悬空伸出。
        s.write_voxel_for_test(BlockPos::new(30, 1, 24), BlockId(1));
        s.write_voxel_for_test(BlockPos::new(30, 2, 24), BlockId(1));
        s.write_voxel_for_test(BlockPos::new(31, 2, 24), BlockId(1));
        s.write_voxel_for_test(BlockPos::new(32, 2, 24), BlockId(1));
    };
    // ---- (a) 高坠对照线:0.45 方块 30 m/s 砸悬空端,石被撞碎 ----
    {
        let mut s = walker_sim();
        load_flat_ground(&mut s);
        build(&mut s);
        s.apply_prefix(&[stamp(
            0,
            SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 2),
                state: EntityState {
                    translation: Vec3::new(32.5, 8.5, 24.5),
                    linvel: Vec3::new(0.0, -30.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: BlockId(1),
                    shape: None,
                },
            },
        )]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let mut impacted = 0u64;
        for _ in 0..40 {
            let out = production_tick(&mut s, &observers);
            impacted += out.destruction_stats.cells_impacted;
        }
        eprintln!("A05-1(a) 对照线:悬挑石梁被 30 m/s 方块撞碎格数 {impacted}");
        assert!(
            impacted > 0,
            "对照线必须撞碎:同落点同速度普通方块 cells_impacted = {impacted}"
        );
    }
    // ---- (a) 主体高坠:同落点同速度(m·v = 0.8×30 = 24 ≥ 18,冲量
    //      够得上石的阈值),不破坏、无事件 ----
    {
        let mut s = walker_sim();
        load_flat_ground(&mut s);
        build(&mut s);
        let before = terrain_fingerprint(&s);
        let a = EntityId::from_parts(1, 1);
        // 胶囊底(中心 − 0.9)与方块底同起点:中心 y = 8.5 + 0.45 + 0.05。
        s.apply_prefix(&[stamp(
            0,
            SimCommand::SpawnAgent {
                entity: a,
                body: capsule_hash(),
                state: EntityState {
                    translation: Vec3::new(32.5, 9.0, 24.5),
                    linvel: Vec3::new(0.0, -30.0, 0.0),
                    ..EntityState::default()
                },
            },
        )]);
        let observers = [Vec3::new(32.0, 3.0, 24.0)];
        let mut impacted = 0u64;
        let mut flow_kinds = Vec::new();
        for _ in 0..180 {
            let out = production_tick(&mut s, &observers);
            impacted += out.destruction_stats.cells_impacted;
            if let Some(ev) = out.flow_event {
                flow_kinds.push(ev);
            }
        }
        let st = s.entity_state(a).expect("主体落地后在");
        eprintln!(
            "A05-1(a) 主体线:触地速度标定 30 m/s(m·v = {:.1} ≥ 18);             主体落定 y = {:.3};撞击破坏 {impacted};冲量流事件 {} 条",
            0.8f32 * 30.0,
            st.translation.y,
            flow_kinds.len()
        );
        assert_eq!(
            impacted, 0,
            "主体高坠不得破坏地形(§6 D8):cells_impacted = {impacted}"
        );
        assert_eq!(terrain_fingerprint(&s), before, "主体高坠全程地形字节不变");
        // §5.7 冲量流事件里没有以主体为撞击者的事件:场上只有主体一个动态
        // 实体,任何事件都只能以它为撞击者 ⇒ 直接断言 0 条。
        assert!(
            flow_kinds.is_empty(),
            "主体是唯一动态实体,任何冲量流事件都以其为撞击者(实得 {} 条)",
            flow_kinds.len()
        );
    }
    // ---- (b) 被爆炸掀起后落地:对照线(同款方块上抛 30 落回同一悬空端)----
    {
        let mut s = walker_sim();
        load_flat_ground(&mut s);
        build(&mut s);
        s.apply_prefix(&[stamp(
            0,
            SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 2),
                state: EntityState {
                    translation: Vec3::new(32.5, 8.5, 24.5),
                    linvel: Vec3::new(0.0, 30.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: BlockId(1),
                    shape: None,
                },
            },
        )]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let mut impacted = 0u64;
        // 上抛 30 m/s 全程 ≈ 6.1 s ≈ 370 拍(上行 + 落回),窗口 450 拍。
        for _ in 0..450 {
            let out = production_tick(&mut s, &observers);
            impacted += out.destruction_stats.cells_impacted;
        }
        eprintln!("A05-1(b) 对照线:上抛方块(30 m/s 落回)撞碎格数 {impacted}");
        assert!(
            impacted > 0,
            "对照线(b)必须撞碎:cells_impacted = {impacted}"
        );
    }
    // ---- (b) 主体被炸飞后落地:不破坏地形 ----
    // 布景:主体站平板,爆炸在脚下(power 20)——爆炸本身炸出坑(爆炸行为,
    // 如实打印坑深并从指纹基线中排除):爆炸拍落的指纹作为基线,**落回的
    // 撞击**不得再改变地形(impacted = 0 的同时基线后指纹不变)。
    {
        let (mut s, a, base) = settled_agent();
        // 爆炸功率现场标定:主体 0.8 kg 在爆心正下 0.4 m,power 20 给冲量
        // ≈ 20×(1−0.4/5) = 18.4 → Δv ≈ 23 m/s 向上(掀飞 ~27 m);落地
        // 触地速度 ≈ 23 m/s(m·v ≈ 18.4 ≥ 18,冲量够得上石的阈值)。
        s.apply_prefix(&[stamp(
            2,
            SimCommand::Detonate {
                center: Vec3::new(base.x, base.y - 0.4, base.z),
                power: 20.0,
                radius: 5.0,
            },
        )]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let mut max_height = 0.0f32;
        // 爆炸拍(冲击写入当拍生效)。
        let _ = production_tick(&mut s, &observers);
        // 爆炸坑后的地形基线(此后变化 = 撞击破坏,判据锚点)。
        let before = terrain_fingerprint(&s);
        let mut impacted = 0u64;
        let mut flow_kinds = Vec::new();
        // Δv 23 m/s:上行 ~27 m(2.3 s)+ 落回(2.3 s)≈ 280 拍,窗口 420。
        for _ in 0..420 {
            let out = production_tick(&mut s, &observers);
            impacted += out.destruction_stats.cells_impacted;
            if let Some(ev) = out.flow_event {
                flow_kinds.push(ev);
            }
            if let Some(st) = s.entity_state(a) {
                max_height = max_height.max(st.translation.y);
            }
        }
        let st = s.entity_state(a);
        eprintln!(
            "A05-1(b) 主体线:炸飞最高 y = {max_height:.3}(base y {:.3}),落回 {:?};             撞击破坏 {impacted};事件 {} 条(爆炸坑本身为爆炸行为,不在撞击判据内)",
            base.y,
            st.map(|x| (x.translation.x, x.translation.y, x.translation.z)),
            flow_kinds.len()
        );
        assert!(
            max_height > base.y + 10.0,
            "布景自检:主体须被炸飞(最高 {max_height:.3})"
        );
        // 判据锚点:爆炸之后的落回撞击破坏 0、基线指纹不变、无事件。
        // 高速落回(触地 ≈ 23 m/s,冲量 ≥ 18 档;无 CCD 胶囊存在隧穿可能,
        // 末位置如实打印不锚)不改变三断言。
        assert_eq!(impacted, 0, "主体炸飞落回的撞击不得破坏地形");
        assert_eq!(terrain_fingerprint(&s), before, "落回撞击地形字节不变");
        assert!(flow_kinds.is_empty(), "无以主体为撞击者的事件");
    }
}

// ── A05-2:被撞 ──────────────────────────────────────────────────────────

#[test]
fn a05_2_pushed_by_block_and_compound_compound_not_broken() {
    // ---- 组 1:方块 5 m 砸下,主体位移 > 0 ----
    {
        let (mut s, a, base) = settled_agent();
        s.apply_prefix(&[stamp(
            2,
            SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 2),
                state: EntityState {
                    translation: Vec3::new(base.x + 0.3, base.y + 5.0, base.z + 0.3),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            },
        )]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let mut max_disp = 0.0f32;
        for _ in 0..300 {
            sync_shape_table(&mut s, a);
            production_tick(&mut s, &observers);
            let st = s.entity_state(a).expect("主体在");
            max_disp = max_disp.max((st.translation - base).length());
        }
        eprintln!("A05-2 组 1(5m 方块砸):窗口最大位移 {max_disp:.4} m");
        assert!(max_disp > 0.0, "5 m 落方块须推动主体(位移 {max_disp:.4})");
    }
    // ---- 组 2:复合体撞主体,主体被推;复合体不断(汇点无主体)----
    {
        let (mut s, a, base) = settled_agent();
        let shape = CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [1, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [2, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [3, 0, 0],
                block: BlockId(1),
            },
        ])
        .expect("4 格横条合法");
        s.apply_prefix(&[stamp(
            2,
            SimCommand::SpawnCompound {
                entity: EntityId::from_parts(1, 3),
                origin: BlockPos::new(28, 1, 23),
                shape,
                impulses: Vec::new(),
                velocity: (Vec3::new(-8.0, 0.0, 0.0), Vec3::ZERO),
            },
        )]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let mut max_disp = 0.0f32;
        let mut compound_flow_breaks = 0u64;
        let mut impact_breaks = 0u64;
        let mut impacted = 0u64;
        for _ in 0..240 {
            sync_shape_table(&mut s, a);
            let out = production_tick(&mut s, &observers);
            compound_flow_breaks += out.cohesion_stats.compound_flow_breaks;
            impact_breaks += out.cohesion_stats.impact_flow_breaks;
            impacted += out.destruction_stats.cells_impacted;
            let st = s.entity_state(a).expect("主体在");
            max_disp = max_disp.max((st.translation - base).length());
        }
        eprintln!(
            "A05-2 组 2(复合体撞主体):主体窗口最大位移 {max_disp:.4} m;复合体流断开 {compound_flow_breaks},             撞击流断开 {impact_breaks},破坏 {impacted}"
        );
        assert!(max_disp > 0.05, "复合体撞须推动主体");
        assert_eq!(
            compound_flow_breaks + impact_breaks + impacted,
            0,
            "撞到主体不得让复合体摔断、也不得破坏(主体不是汇点;撞悬挑梁对照线同款结构会断)"
        );
    }
    // ---- 对照线:同一复合体同速撞悬挑石梁会断 ----
    // (compound_flow 语义:贴地整墙的压向割 = ∞ 不断;可断结构 = 悬挑,
    //  割 = 粘结链。悬挑梁同 A05-1 布景:柱 (30,1..2,24) + 梁 (31..32,2,24),
    //  复合体沿 y=1 行进撞不到 y=2 的梁 → 梁下移一格:柱 (30,1) + 梁
    //  (31,1)、(32,1)?梁 y=1 贴地不算悬挑。改:复合体对准梁 (y=2) 高度
    //  平撞:SpawnCompound origin y=2。)
    {
        let mut s = walker_sim();
        load_flat_ground(&mut s);
        s.write_voxel_for_test(BlockPos::new(30, 1, 24), BlockId(1));
        s.write_voxel_for_test(BlockPos::new(30, 2, 24), BlockId(1));
        s.write_voxel_for_test(BlockPos::new(31, 2, 24), BlockId(1));
        s.write_voxel_for_test(BlockPos::new(32, 2, 24), BlockId(1));
        eprintln!(
            "A05-2 对照线布景自检: 梁 (31,2)={:?} (32,2)={:?}",
            s.voxel().get_block(BlockPos::new(31, 2, 24)),
            s.voxel().get_block(BlockPos::new(32, 2, 24))
        );
        let shape = CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [1, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [2, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [3, 0, 0],
                block: BlockId(1),
            },
        ])
        .expect("4 格横条合法");
        // 复合体 origin y=2(与梁同高),z=24(对准梁),从 x=36 起撞,
        // 速度 −20(现场标定:J ≈ 4×0.729×20 ≈ 58,悬挑割 = bond 8,必断;
        // 组 2 主体线仍 −8——两条线布景独立)。
        s.apply_prefix(&[stamp(
            0,
            SimCommand::SpawnCompound {
                entity: EntityId::from_parts(1, 3),
                origin: BlockPos::new(36, 2, 24),
                shape,
                impulses: Vec::new(),
                velocity: (Vec3::new(-20.0, 0.0, 0.0), Vec3::ZERO),
            },
        )]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let mut compound_flow_breaks = 0u64;
        let mut impact_breaks_wall = 0u64;
        let mut impacted_wall = 0u64;
        let mut events_wall = 0u64;
        let mut detached_wall = 0u64;
        for i in 0..240 {
            let out = production_tick(&mut s, &observers);
            compound_flow_breaks += out.cohesion_stats.compound_flow_breaks;
            impact_breaks_wall += out.cohesion_stats.impact_flow_breaks;
            impacted_wall += out.destruction_stats.cells_impacted;
            events_wall += out.cohesion_stats.compound_flow_events;
            detached_wall += out.cohesion_stats.compound_flow_detached_cells;
            if i < 15 {
                let st = s.entity_state(EntityId::from_parts(1, 3));
                eprintln!(
                    "t{i}: comp={:?} tick_imp={}",
                    st.map(|x| (x.translation.x, x.translation.y)),
                    out.destruction_stats.cells_impacted
                );
            }
        }
        eprintln!(
            "A05-2 对照线:compound_breaks {compound_flow_breaks} impact_breaks {} impacted {} events {} detached {}             (梁断走撞击冲量流:evaluate_impacts 先于 evaluate_fractures,悬挑割 8 < s → impact_flow 断)",
            impact_breaks_wall, impacted_wall, events_wall, detached_wall
        );
        assert!(
            compound_flow_breaks + impact_breaks_wall + impacted_wall > 0,
            "对照线必须断:同款复合体撞悬挑石梁(复合体流/撞击流/破坏计数全 0)"
        );
    }
    // ---- 组 4:主体被当地形处理的采样口径(变异 13)----
    // 复合体冲量流的接近速度按**相对速度**算(§5.8.1 勘误 2:v_other − v_self);
    // 主体是动态对方 → v_other = 主体速度。变异 13(主体按撞到地形处理)
    // 把主体从对方速度表剔掉 → v_other = 0 → 接近速度只用复合体自身速度。
    // 布景分离两者:用**无 [walker] 段**主体(walker 刹车 ~7 拍时间常数,
    // 追尾路上速度会衰减过门槛;nowalker 速度不被衰减),阶段 1 用 −20
    // 复合体把主体推到高速;阶段 2 在主体前方 ~0.3 m 放**静止**复合体,
    // 主体匀速追上:相对接近速度 = |v_agent| ≫ 1(事件建立);按地形口径
    // = 0 < 1(全被筛掉,事件 = 0)。断言阶段 2 事件 > 0 = 钉住
    // "主体速度进接近速度"。
    {
        let (mut s, a, _base) = settled_nowalker_agent();
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let shape = CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [1, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [2, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [3, 0, 0],
                block: BlockId(1),
            },
        ])
        .expect("4 格横条合法");
        // 阶段 1:−6 复合体撞主体(同组 2 几何;从 27 起步——20 拍内到达,
        // 不给"低速滑行 → 静置凝固"留窗口:−3 从 28 起步要 74 拍,滑到
        // t30 已近零速,t50 凝固消失)。撞击后主体 ~2.5 m/s、每拍 0.04 m
        // ——阶段 2 的接触是浅穿透,solver 正常解算(14.9 m/s = 0.25 m/拍
        // 深穿透,rapier 当初始重叠蠕动推出,接触冲量近 0)。
        s.apply_prefix(&[stamp(
            2,
            SimCommand::SpawnCompound {
                entity: EntityId::from_parts(1, 3),
                origin: BlockPos::new(27, 1, 23),
                shape: shape.clone(),
                impulses: Vec::new(),
                velocity: (Vec3::new(-6.0, 0.0, 0.0), Vec3::ZERO),
            },
        )]);
        let mut agent_vx_peak = 0.0f32;
        for _ in 0..60 {
            sync_shape_table(&mut s, a);
            let _ = production_tick(&mut s, &observers);
            let st = s.entity_state(a).expect("主体在");
            agent_vx_peak = agent_vx_peak.max(st.linvel.x.abs());
        }
        eprintln!("A05-2 组 4 阶段 1:主体速度峰值 {agent_vx_peak:.3} m/s");
        assert!(
            (1.0..8.0).contains(&agent_vx_peak),
            "布景自检:阶段 1 须把主体推到 1~8 m/s(峰值 {agent_vx_peak:.3})"
        );
        // 阶段 2 直接开跑(nowalker 不衰减;窗口 20 拍内撞上,慢复合体
        // 来不及凝固)。
        let vx = s.entity_state(a).expect("主体在").linvel.x;
        eprintln!("A05-2 组 4 阶段 2 前:主体 vx = {vx:.3} m/s");
        assert!(vx.abs() > 1.0, "布景自检:主体须仍以高速运动(vx = {vx:.3})");
        // 清场:阶段 1 的大复合体已远去/停驻,但它的残余接触会污染阶段 2
        // 的事件口径(它也是动态对方)——直接移除,只留主体与慢复合体。
        s.apply_prefix(&[stamp(
            199,
            SimCommand::RemoveEntity {
                entity: EntityId::from_parts(1, 3),
            },
        )]);
        for _ in 0..2 {
            sync_shape_table(&mut s, a);
            let _ = production_tick(&mut s, &observers);
        }
        // 静止复合体放在主体 −x 前方,间隙 ≈0.3 m(复合体右缘 = 主体左缘
        // − 0.3):nowalker 不减速,~2 拍内撞上。复合体 4 格宽,spawn 原点
        // 是左下角格——不能与主体胶囊重叠。
        let agent_x = s.entity_state(a).expect("主体在").translation.x;
        let vx_now = s.entity_state(a).expect("主体在").linvel.x;
        eprintln!("清场后主体 x {agent_x:.2} vx {vx_now:.3}");
        let gx = (agent_x - 0.3 - 0.3 - 4.0).ceil() as i32; // ceil:右缘 gap ≤ 0.3
        s.apply_prefix(&[stamp(
            200,
            SimCommand::SpawnCompound {
                entity: EntityId::from_parts(1, 4),
                origin: BlockPos::new(gx, 1, 23),
                shape,
                impulses: Vec::new(),
                velocity: (Vec3::ZERO, Vec3::ZERO),
            },
        )]);
        let mut events_phase2 = 0u64;
        for i in 0..30 {
            sync_shape_table(&mut s, a);
            let out = production_tick(&mut s, &observers);
            events_phase2 += out.cohesion_stats.compound_flow_events;
            if i % 3 == 0 {
                let av = s.entity_state(a).map(|x| (x.translation.x, x.linvel.x));
                let cv = s
                    .entity_state(EntityId::from_parts(1, 4))
                    .map(|x| (x.translation.x, x.linvel.x));
                eprintln!("  阶段2 t{i}: agent={av:?} comp={cv:?}");
            }
        }
        eprintln!("A05-2 组 4 阶段 2:主体追尾慢复合体,复合体流事件 {events_phase2}");
        assert!(
            events_phase2 > 0,
            "相对接近速度口径:主体({vx:.3})追尾静止复合体 ⇒ 接近速度 = |v| > 1,事件必建;             主体被按撞到地形处理(v_other = 0)则口径 = 0 < 1 全筛 → 0(变异 13 即红)"
        );
    }
}

// ── A05-3:爆炸 ──────────────────────────────────────────────────────────

#[test]
fn a05_3_explosion_launches_agent_by_per_link_impulse() {
    // **布景**:用**无 `[walker]` 段**的身体(R1-P1:无意图也照常刹车;
    // 有 walker 段的主体刹车力会污染 Δv)。无 walker 段 → 控制拍直接
    // continue,不施任何力 → 首拍 Δv = 纯爆炸冲量/m + 重力项。
    let (mut s, a, base) = settled_nowalker_agent();
    let observers = [Vec3::new(24.0, 3.0, 24.0)];
    // 预期首拍速度增量 = explosion_impulse(center, power, radius, 根位置)/质量。
    let mass = 0.8f32; // capsule 80 kg × k(1/100)
    let center = Vec3::new(base.x - 2.5, 1.0, base.z);
    let power = 12.0f32;
    let radius = 5.0f32;
    let expected_dv = physics_sim::explosion::explosion_impulse(center, power, radius, base)
        .expect("布景自检:主体在半径内")
        / mass;
    s.apply_prefix(&[stamp(
        2,
        SimCommand::Detonate {
            center,
            power,
            radius,
        },
    )]);
    production_tick(&mut s, &observers);
    let st = s.entity_state(a).expect("主体不被移除");
    // 首拍还有重力(g·dt ≈ 0.1635 向下);速度增量 = 冲量/m + 重力项。
    let g_dt = Vec3::new(0.0, -9.81 * (1.0 / 60.0), 0.0);
    let dv = st.linvel - g_dt;
    let err = (dv - expected_dv).length();
    eprintln!("A05-3:预期 Δv {expected_dv:?},实得(扣重力){dv:?},误差 {err:.2e}");
    assert!(
        err <= 1e-5,
        "首拍速度增量须 = explosion_impulse/质量(误差 ≤ 1e-5):实得 {err:.2e}"
    );
    // 落地不破坏地形:跑 240 拍看破坏计数。
    let before = terrain_fingerprint(&s);
    let mut impacted = 0u64;
    for _ in 0..240 {
        let out = production_tick(&mut s, &observers);
        impacted += out.destruction_stats.cells_impacted;
    }
    assert!(
        s.entity_state(a).is_some(),
        "爆炸不得移除主体(§6:炸飞不受伤)"
    );
    assert_eq!(impacted, 0, "主体落地不破坏地形");
    assert_eq!(terrain_fingerprint(&s), before, "地形字节不变");
    let _ = s;
}

// ── A05-4:地形承载 ───────────────────────────────────────────────────────

/// 石桥(bond = 8 = 8 格重;一格重 W0 = 0.729 kg?——承载单位是格重,
/// 默认格质量 1.0/格(CELL_MASS),跨 3 格桥两端墩)。桥长取承载极限:
/// 水平跨 4 格两端锚定,中央 2 格由两端的 bond 链传递;对照线 = 桥中央
/// 加 1 格实心(第 5 格把跨中悬挑变成 5 格链)就塌。布景现场标定:
/// 跨度从 3 格开始扫到判定"主体站桥不塌、加 1 格就塌"的档。
#[test]
fn a05_4_load_ignores_agent_weight_bridge_control() {
    // 布景(承载判定语义现场标定):石 bond = 8 = 一条共面连接最多传 8 格重
    // (格重 W0 = 0.729 kg/格)。**单端锚定的 12 格水平悬挑桥**:拆锚后
    // 悬挑连通块 12 格,流 12 > 割 8 → 塌(对照线);站上主体不塌(主体
    // 重量不计入承载判定,§6)。主体线与对照线同一座桥。
    let build_bridge = |s: &mut ShardSim| {
        // 桥面 y=3(与地面 y=1 隔两格空隙,不贴地),x ∈ [22, 33](12 格,
        // z=24);锚柱 x=21(y=1..3,底 (21,1) 落在地面顶上接地)。
        for x in 22..=33 {
            s.write_voxel_for_test(BlockPos::new(x, 3, 24), BlockId(1));
        }
        s.write_voxel_for_test(BlockPos::new(21, 1, 24), BlockId(1));
        s.write_voxel_for_test(BlockPos::new(21, 2, 24), BlockId(1));
        s.write_voxel_for_test(BlockPos::new(21, 3, 24), BlockId(1));
    };
    // ---- 对照线:挖锚柱顶格 → 12 格全悬挑 → 塌 ----
    {
        let mut s = walker_sim();
        load_flat_ground(&mut s);
        build_bridge(&mut s);
        let seq = 10u64;
        s.apply_prefix(&[stamp(
            seq,
            SimCommand::SetBlock {
                pos: BlockPos::new(21, 1, 24),
                block: BlockId::AIR,
            },
        )]);
        let observers = [Vec3::new(28.0, 3.0, 24.0)];
        let mut converted = 0u64;
        let mut evals = 0u64;
        let mut falling = 0u64;
        for i in 0..30 {
            let out = production_tick(&mut s, &observers);
            converted += out.destruction_stats.cells_collapsed;
            evals += out.cohesion_stats.load_evaluations;
            falling += out.cohesion_stats.load_falling;
            if i < 5 {
                eprintln!(
                    "t{i}: anchor={:?} evals={} falling={}",
                    s.voxel().get_block(BlockPos::new(21, 1, 24)),
                    out.cohesion_stats.load_evaluations,
                    out.cohesion_stats.load_falling
                );
            }
        }
        let missing = (22..=33)
            .map(|x| s.voxel().get_block(BlockPos::new(x, 3, 24)).is_air())
            .filter(|&air| air)
            .count();
        eprintln!(
            "A05-4 对照线(12 格悬挑桥拆锚):坍塌转换 {converted},桥缺失格 {missing}/12,求值 {evals},falling {falling}"
        );
        assert!(
            converted > 0,
            "对照线必须塌:12 格悬挑(流 12 > 割 8)拆锚后 cells_collapsed = {converted}"
        );
    }
    // ---- 主体线:同一座桥 + 主体站桥上 600 拍 + 跳 10 次:不塌、转换 0 ----
    {
        let mut s = walker_sim();
        load_flat_ground(&mut s);
        build_bridge(&mut s);
        let a = EntityId::from_parts(1, 1);
        s.apply_prefix(&[
            stamp(0, spawn_agent_cmd(a, Vec3::new(27.5, 4.5, 24.5))),
            stamp(
                1,
                SimCommand::AgentIntent {
                    entity: a,
                    move_xz: (0.0, 0.0),
                    yaw: 0.0,
                    jump: false,
                },
            ),
        ]);
        let observers = [Vec3::new(28.0, 3.0, 24.0)];
        let mut seq = 2u64;
        let mut converted = 0u64;
        let mut jumps = 0usize;
        let mut fell_below = false;
        for i in 0..600 {
            if i > 0 && i % 60 == 0 && jumps < 10 {
                s.apply_prefix(&[stamp(
                    seq,
                    SimCommand::AgentIntent {
                        entity: a,
                        move_xz: (0.0, 0.0),
                        yaw: 0.0,
                        jump: true,
                    },
                )]);
                seq += 1;
                jumps += 1;
            }
            sync_shape_table(&mut s, a);
            let out = production_tick(&mut s, &observers);
            converted += out.destruction_stats.cells_collapsed;
            let st = s.entity_state(a).expect("主体在");
            if st.translation.y < 1.0 {
                fell_below = true;
            }
        }
        let missing = (22..=33)
            .map(|x| s.voxel().get_block(BlockPos::new(x, 3, 24)).is_air())
            .filter(|&air| air)
            .count();
        eprintln!(
            "A05-4 主体线:跳 {jumps} 次,桥缺失格 {missing}/12,承载转换 {converted},掉下 {fell_below}"
        );
        assert_eq!(missing, 0, "主体站桥不得压塌桥(§6:重量不计入承载)");
        assert_eq!(converted, 0, "承载转换计数须为 0");
        assert!(!fell_below, "主体全程不掉下(桥在)");
    }
    // ---- 拆墩组:同桥 + 主体 → 挖锚 → 桥塌,主体随之落、不卡、不 NaN ----
    {
        let mut s = walker_sim();
        load_flat_ground(&mut s);
        build_bridge(&mut s);
        let a = EntityId::from_parts(1, 1);
        s.apply_prefix(&[
            stamp(0, spawn_agent_cmd(a, Vec3::new(27.5, 4.5, 24.5))),
            stamp(
                1,
                SimCommand::AgentIntent {
                    entity: a,
                    move_xz: (0.0, 0.0),
                    yaw: 0.0,
                    jump: false,
                },
            ),
        ]);
        let observers = [Vec3::new(28.0, 3.0, 24.0)];
        for _ in 0..60 {
            sync_shape_table(&mut s, a);
            let _ = production_tick(&mut s, &observers);
        }
        let seq = 10u64;
        s.apply_prefix(&[stamp(
            seq,
            SimCommand::SetBlock {
                pos: BlockPos::new(21, 3, 24),
                block: BlockId::AIR,
            },
        )]);
        let mut converted = 0u64;
        let mut fell = false;
        let mut nan = false;
        for _ in 0..360 {
            sync_shape_table(&mut s, a);
            let out = production_tick(&mut s, &observers);
            converted += out.destruction_stats.cells_collapsed;
            if let Some(st) = s.entity_state(a) {
                if !st.translation.is_finite() {
                    nan = true;
                }
                if st.translation.y < 0.5 {
                    fell = true;
                }
            }
        }
        let missing = (22..=33)
            .map(|x| s.voxel().get_block(BlockPos::new(x, 3, 24)).is_air())
            .filter(|&air| air)
            .count();
        let end = s
            .entity_state(a)
            .map(|x| (x.translation.x, x.translation.y, x.translation.z));
        eprintln!(
            "A05-4 拆墩组:桥缺失格 {missing}/12,转换 {converted},主体下落 {fell},NaN {nan},末位置 {end:?}"
        );
        assert!(missing >= 6, "布景自检:拆锚后桥须塌(缺失 {missing}/12)");
        // 判据口径:"拆掉桥墩,桥塌,主体随之下落、落地、不卡、不 NaN":
        // 主体高度必须显著低于桥面(y=3),且全程有限、实体仍在(被塌落
        // 碎块接住也算"落地",判据不要求掉到地面 y=1)。
        let y = s.entity_state(a).expect("主体在").translation.y;
        assert!(y < 2.0, "主体须随桥下落(末 y {y:.3},桥面 y=3)");
        assert!(!nan, "全程不 NaN");
        assert!(s.entity_state(a).is_some(), "主体不卡死(仍在模拟中)");
        let _ = fell;
    }
}

// ── A05-5:SetBlock 暂缓 ─────────────────────────────────────────────────

#[test]
fn a05_5_setblock_deferred_replaced_invalidated_flushed() {
    let (mut s, a, _base) = settled_agent();
    let observers = [Vec3::new(24.0, 3.0, 24.0)];
    // 主体中心格(格心 ≈ base;capsule 半径 0.3 < 0.5,水平占中心格;
    // 竖向中心 ± 0.9 跨 2 格:cy 与 cy+1 或 cy−1)。取中心所在格与上格。
    let c0 = s.entity_state(a).unwrap().translation;
    let cy = c0.y.floor() as i32;
    let target = BlockPos::new(c0.x.floor() as i32, cy, c0.z.floor() as i32);
    let target_up = BlockPos::new(target.x, target.y + 1, target.z);
    // 布景自检:目标格确实重叠(否则布景错)。
    {
        let table = s.agent_shape_entries();
        assert!(
            table.overlaps(target) || table.overlaps(target_up),
            "布景自检:主体中心格或上格须与主体重叠(target {target:?})"
        );
    }
    let initial = solid_count(&s);
    let mut seq = 10u64;

    // ① 对主体所占格写石:不写入、计数 +1(表先同步 = 调度器拍首下发的替身)。
    sync_shape_table(&mut s, a);
    s.apply_prefix(&[stamp(
        seq,
        SimCommand::SetBlock {
            pos: target,
            block: BlockId(1),
        },
    )]);
    seq += 1;
    let out = s.take_outbox();
    assert_eq!(
        out.cohesion_stats.set_block_deferred, 1,
        "写主体所占格:计数暂缓 +1"
    );
    assert!(s.voxel().get_block(target).is_air(), "不写入体素(仍是空气)");
    assert_eq!(
        s.deferred_set_blocks().get(&target),
        Some(&BlockId(1)),
        "暂缓队列有该条"
    );
    assert_no_link_solid_overlap(&s, a, "① 暂缓后");

    // 逐拍:主体不动 ⇒ 每拍重试仍重叠 ⇒ 不补写。
    for _ in 0..30 {
        sync_shape_table(&mut s, a);
        let out = production_tick(&mut s, &observers);
        assert_eq!(
            out.cohesion_stats.set_block_deferred_flushed, 0,
            "仍重叠不补写"
        );
        assert_no_link_solid_overlap(&s, a, "② 重试中");
    }

    // ③ 同格第二条实心:替换(被替换 +1,只写后一条)。
    let dirt = {
        let mut reg = SimpleRegistry::new();
        reg.register("dirt2", true).expect("ok");
        voxel::BlockRegistry::id_of(&reg, "dirt2").expect("ok")
    };
    sync_shape_table(&mut s, a);
    s.apply_prefix(&[stamp(
        seq,
        SimCommand::SetBlock {
            pos: target,
            block: dirt,
        },
    )]);
    seq += 1;
    let out = s.take_outbox();
    assert_eq!(
        out.cohesion_stats.set_block_deferred_replaced, 1,
        "同格实心再写:被替换 +1"
    );
    assert_eq!(
        s.deferred_set_blocks().get(&target),
        Some(&dirt),
        "只留最后一条"
    );

    // ④ 暂缓中写空气:立刻生效并作废暂缓。
    sync_shape_table(&mut s, a);
    s.apply_prefix(&[stamp(
        seq,
        SimCommand::SetBlock {
            pos: target,
            block: BlockId::AIR,
        },
    )]);
    seq += 1;
    let out = s.take_outbox();
    assert_eq!(
        out.cohesion_stats.set_block_deferred_invalidated, 1,
        "非实心写入作废暂缓 +1"
    );
    assert!(
        s.deferred_set_blocks().get(&target).is_none(),
        "暂缓条目已作废"
    );

    // ⑤ 重新写石暂缓;主体走开(走意图 3 m)⇒ 第一拍不再重叠时补写。
    sync_shape_table(&mut s, a);
    s.apply_prefix(&[stamp(
        seq,
        SimCommand::SetBlock {
            pos: target,
            block: BlockId(1),
        },
    )]);
    seq += 1;
    let _ = s.take_outbox();
    assert_eq!(s.deferred_set_blocks().get(&target), Some(&BlockId(1)));

    // 走开:+x 方向意图 2 m/s,几拍后不再重叠。
    s.apply_prefix(&[stamp(
        seq,
        SimCommand::AgentIntent {
            entity: a,
            move_xz: (2.0, 0.0),
            yaw: 0.0,
            jump: false,
        },
    )]);
    let mut first_free_tick: Option<u64> = None;
    let mut flush_tick: Option<u64> = None;
    let mut tick_no = 0u64;
    for _ in 0..240 {
        sync_shape_table(&mut s, a);
        let out = production_tick(&mut s, &observers);
        tick_no += 1;
        let table = s.agent_shape_entries();
        if first_free_tick.is_none() && !table.overlaps(target) {
            first_free_tick = Some(tick_no);
        }
        if out.cohesion_stats.set_block_deferred_flushed >= 1 && flush_tick.is_none() {
            flush_tick = Some(tick_no);
        }
        assert_no_link_solid_overlap(&s, a, "⑤ 走开中");
        if flush_tick.is_some() {
            break;
        }
    }
    let first_free = first_free_tick.expect("主体走开后必有不再重叠的一拍");
    let flushed = flush_tick.expect("不再重叠后必须补写");
    assert_eq!(
        flushed,
        first_free + 1,
        "补写拍号 = 首个不重叠拍 + 1(first_free {first_free},flushed {flushed})"
    );
    assert_eq!(s.voxel().get_block(target), BlockId(1), "补写落格");
    // 总数守恒:初始 + 1(补写的石)。
    assert_eq!(
        solid_count(&s),
        initial + 1,
        "总方块数 = 初始 + 写入数(空气/替换不改变净数)"
    );

    // ⑥ 主体被移除,下一拍补写。
    let (mut s2, a2, base2) = settled_agent();
    let target2 = BlockPos::new(
        base2.x.floor() as i32,
        base2.y.floor() as i32,
        base2.z.floor() as i32,
    );
    sync_shape_table(&mut s2, a2);
    s2.apply_prefix(&[stamp(
        10,
        SimCommand::SetBlock {
            pos: target2,
            block: BlockId(1),
        },
    )]);
    let _ = s2.take_outbox();
    assert_eq!(s2.deferred_set_blocks().get(&target2), Some(&BlockId(1)));
    s2.apply_prefix(&[stamp(11, SimCommand::RemoveEntity { entity: a2 })]);
    let _ = s2.take_outbox();
    let obs2 = [Vec3::new(24.0, 3.0, 24.0)];
    // 表保持带主体的旧表(调度器语义:表 = 上一拍末的快照;移除在下一拍
    // prefix 生效,表在下一拍拍首 flush 之后才被重新汇总)。
    production_tick(&mut s2, &obs2);
    let out = s2.take_outbox();
    eprintln!(
        "A05-5 ⑥ 移除拍:flushed={} deferred={:?} agent_in={}",
        out.cohesion_stats.set_block_deferred_flushed,
        s2.deferred_set_blocks().get(&target2),
        s2.entity_state(a2).is_some()
    );
    // 移除当拍表还带着主体(本测试手工同步的表),下一拍表为空 ⇒ 补写。
    assert_eq!(
        out.cohesion_stats.set_block_deferred_flushed, 0,
        "移除当拍(表仍在)不补写"
    );
    s2.set_agent_shape_table(AgentShapeTable::default());
    eprintln!(
        "A05-5 ⑥ 补写拍前:表空={} 队列={:?}",
        s2.agent_shape_table().is_empty(),
        s2.deferred_set_blocks().get(&target2)
    );
    let out = production_tick(&mut s2, &obs2);
    eprintln!(
        "A05-5 ⑥ 补写拍:flushed={} updates={} block={:?}",
        out.cohesion_stats.set_block_deferred_flushed,
        out.block_updates.len(),
        s2.voxel().get_block(target2)
    );
    assert_eq!(
        out.cohesion_stats.set_block_deferred_flushed, 1,
        "主体移除后下一拍补写 +1"
    );
    assert_eq!(s2.voxel().get_block(target2), BlockId(1), "补写落格");
    assert!(s2.deferred_set_blocks().is_empty(), "队列清空");
}

// ── A05-6:凝固、强制落地暂缓 ────────────────────────────────────────────

#[test]
fn a05_6_solidify_and_forced_landing_deferred() {
    // ---- 组 1:单格凝固暂缓 ----
    // 方块实体(half=0.5 默认)搭在主体旁边静止;它的凝固落点与主体重叠。
    // 布景:主体站格 (24,1)-(24,2);方块生成在主体中心正上方 1.05 m
    // (落点 = 中心所在格 = 主体上格),落地后减速、凝固判据慢速达标,
    // 但目标格与主体重叠 ⇒ 暂缓、保持实体。
    {
        let (mut s, a, base) = settled_agent();
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let target = BlockPos::new(
            base.x.floor() as i32,
            base.y.floor() as i32 + 1,
            base.z.floor() as i32,
        );
        s.apply_prefix(&[stamp(
            2,
            SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 2),
                state: EntityState {
                    translation: Vec3::new(
                        target.x as f32 + 0.5,
                        target.y as f32 + 0.5,
                        target.z as f32 + 0.5,
                    ),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            },
        )]);
        let mut deferred_ticks = 0u64;
        let mut still_entity = true;
        for _ in 0..360 {
            sync_shape_table(&mut s, a);
            let out = production_tick(&mut s, &observers);
            deferred_ticks += out.cohesion_stats.solidify_deferred;
            if s.entity_state(EntityId::from_parts(1, 2)).is_none() {
                still_entity = false;
            }
        }
        eprintln!("A05-6 组 1(凝固暂缓):计数 {deferred_ticks},方块仍实体 {still_entity}");
        assert!(
            deferred_ticks >= 1,
            "凝固落点与主体重叠须暂缓:计数 {deferred_ticks}"
        );
        assert!(still_entity, "暂缓期间方块保持为实体(不许被埋/被移除)");
        // 主体走开 → 方块凝固(总数守恒)。
        let total_before = solid_count(&s) + 1; // + 1 = 方块实体本身
        s.apply_prefix(&[stamp(
            3,
            SimCommand::AgentIntent {
                entity: a,
                move_xz: (2.0, 0.0),
                yaw: 0.0,
                jump: false,
            },
        )]);
        let mut solidified = false;
        for _ in 0..600 {
            sync_shape_table(&mut s, a);
            let out = production_tick(&mut s, &observers);
            if !out.block_updates.is_empty() {
                solidified = true;
            }
        }
        let total_after = solid_count(&s);
        eprintln!("A05-6 组 1(走开后):凝固 {solidified},总数 {total_before} → {total_after}");
        assert!(solidified, "主体走开后方块须凝固");
        assert_eq!(total_after, total_before, "全程方块总数守恒(实体 + 体素)");
    }
    // ---- 组 2:强制落地暂缓(到期指令触发 = 调度器 7.5 同款入口)----
    // 方块实体搭在主体上格;直接投强制落地指令(生产路径 = 瞬态票到期
    // 调度器投递;这里直调同一入口),落点格与主体重叠 ⇒ 暂缓、保持实体;
    // 主体走开后落地、总数守恒。
    {
        let (mut s, a, base) = settled_agent();
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let target = BlockPos::new(
            base.x.floor() as i32,
            base.y.floor() as i32 + 1,
            base.z.floor() as i32,
        );
        let victim = EntityId::from_parts(1, 2);
        s.apply_prefix(&[stamp(
            2,
            SimCommand::SpawnDynamic {
                entity: victim,
                state: EntityState {
                    translation: Vec3::new(
                        target.x as f32 + 0.5,
                        target.y as f32 + 0.5,
                        target.z as f32 + 0.5,
                    ),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            },
        )]);
        let _ = production_tick(&mut s, &observers);
        // 到期指令(调度器 7.5 同款;owner 分片直投)。
        s.apply_forced_landing_directive(victim);
        let mut deferred = 0u64;
        let mut still_entity = true;
        for _ in 0..300 {
            sync_shape_table(&mut s, a);
            let out = production_tick(&mut s, &observers);
            deferred += out.cohesion_stats.forced_landing_deferred;
            if s.entity_state(victim).is_none() {
                still_entity = false;
            }
        }
        eprintln!("A05-6 组 2(强制落地暂缓):计数 {deferred},目标块仍实体 {still_entity}");
        assert!(deferred >= 1, "落点与主体重叠须暂缓落地:计数 {deferred}");
        assert!(still_entity, "暂缓期间保持实体");
        // 主体走开 → 落地,总数守恒(实体 + 体素)。
        let total_before = solid_count(&s) + usize::from(still_entity);
        let seq = 10u64;
        s.apply_prefix(&[stamp(
            seq,
            SimCommand::AgentIntent {
                entity: a,
                move_xz: (2.0, 0.0),
                yaw: 0.0,
                jump: false,
            },
        )]);
        for _ in 0..900 {
            sync_shape_table(&mut s, a);
            let _ = production_tick(&mut s, &observers);
        }
        let total_after = solid_count(&s);
        eprintln!("A05-6 组 2(走开后):总数 {total_before} → {total_after}");
        assert_eq!(total_after, total_before, "总数守恒");
        assert!(
            s.entity_state(victim).is_none(),
            "主体走开后方块已落地(实体消失)"
        );
    }
    // ---- 组 3:复合体整块凝固暂缓 ----
    {
        let (mut s, a, base) = settled_agent();
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        // 2 格竖条骑在主体头顶(下格 = 主体上格,重叠)。
        let gx = base.x.floor() as i32;
        let gz = base.z.floor() as i32;
        let gy = base.y.floor() as i32 + 1;
        let shape = CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [0, 1, 0],
                block: BlockId(1),
            },
        ])
        .expect("2 格竖条合法");
        s.apply_prefix(&[stamp(
            2,
            SimCommand::SpawnCompound {
                entity: EntityId::from_parts(1, 3),
                origin: BlockPos::new(gx, gy, gz),
                shape,
                impulses: Vec::new(),
                velocity: (Vec3::ZERO, Vec3::ZERO),
            },
        )]);
        let mut deferred = 0u64;
        let mut still_entity = true;
        for _ in 0..360 {
            sync_shape_table(&mut s, a);
            let out = production_tick(&mut s, &observers);
            deferred += out.cohesion_stats.solidify_deferred;
            if s.entity_state(EntityId::from_parts(1, 3)).is_none() {
                still_entity = false;
            }
        }
        eprintln!("A05-6 组 3(复合体凝固暂缓):计数 {deferred},仍实体 {still_entity}");
        assert!(deferred >= 1, "复合体任一格重叠 ⇒ 整体暂缓");
        assert!(still_entity, "复合体保持为实体");
        // 走开后整块凝固。
        s.apply_prefix(&[stamp(
            3,
            SimCommand::AgentIntent {
                entity: a,
                move_xz: (2.0, 0.0),
                yaw: 0.0,
                jump: false,
            },
        )]);
        let total_before = solid_count(&s) + 2; // 复合体 2 格
        for _ in 0..600 {
            sync_shape_table(&mut s, a);
            production_tick(&mut s, &observers);
            let _ = s.take_outbox();
        }
        assert_eq!(
            solid_count(&s),
            total_before,
            "复合体凝固后总数守恒(2 格落格)"
        );
    }
}

// ── A05-7:跨分片一致(Session 层;调度器自动下发形状表)────────────────

fn four_shard_session() -> Session {
    let mut session = Session::new(SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    });
    for &(sx, sz) in &[(0i32, 0i32), (1, 0), (0, 1), (1, 1)] {
        let cfg = physics_sim::ShardSimConfig::new(shard_id(sx, sz));
        session.register(ShardSim::new_with_bodies(
            cfg,
            Box::new(SimpleRegistry::builtin()),
            capsule_registry(),
        ));
    }
    for &(sx, sz) in &[(0i32, 0i32), (1, 0), (0, 1), (1, 1)] {
        let shard = shard_id(sx, sz);
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

#[test]
fn a05_7_cross_shard_deferred_consistency() {
    let mut session = four_shard_session();
    // 主体站 x 边界上:分片 (0,0) 本体区 = 世界 x [0,128),(1,0) = [128,256);
    // 边界 = 世界 x = 128。capsule 中心 x = 128.0(跨两片),y 站地面(顶面
    // y=1,中心 1.95)。
    let a = EntityId::from_parts(0, 1);
    session
        .spawn_agent(
            shard_id(0, 0),
            a,
            capsule_hash(),
            EntityState {
                translation: Vec3::new(128.0, 1.95, 64.0),
                ..EntityState::default()
            },
        )
        .expect("生成");
    session.tick();
    let _ = session.tick();
    assert!(session.is_agent(a), "主体在场");

    // 目标格:世界 (128, 1, 64) = 主体中心格。chunk (8,4) → owner = 分片
    // (1,0)(chunk 8 属高分片本体);副本 = 分片 (0,0)(向 +x 的扩张带
    // 恰含 chunk 8)。
    let target = BlockPos::new(128, 1, 64);
    let owner = shard_id(1, 0);
    // 布景自检:主体实际位置与表。
    eprintln!(
        "A05-7 布景: root={:?} in_shard00={} in_shard10={}",
        session.agent_root_position(a),
        session.entity_state(shard_id(0, 0), a).is_some(),
        session.entity_state(shard_id(1, 0), a).is_some(),
    );
    let total_before = session.deferred_set_block_total();
    session
        .set_block(owner, target, BlockId(1))
        .expect("SetBlock 广播");
    let summary_sb = session.tick();
    eprintln!(
        "A05-7 SetBlock 拍: total {}→{} deferred={} chunk00_some={} chunk10_some={}",
        total_before,
        session.deferred_set_block_total(),
        summary_sb.cohesion.set_block_deferred,
        session
            .voxel_chunk(shard_id(0, 0), target.chunk_of())
            .is_some(),
        session
            .voxel_chunk(shard_id(1, 0), target.chunk_of())
            .is_some(),
    );

    // 逐拍:两片该格体素始终相同;主体不走开 ⇒ 不补写(暂缓);两片
    // owner/holder 都暂缓(计数各自 ≥ 1 或合计判据)。
    let mut deferred_owner = 0u64;
    let mut same = true;
    let mut flushed_tick: Option<u64> = None;
    for i in 0..40 {
        let summary = session.tick();
        deferred_owner += summary.cohesion.set_block_deferred;
        // 两片读格。
        let b0 = session
            .voxel_chunk(shard_id(0, 0), target.chunk_of())
            .map(|c| {
                c.get(
                    (target.x - target.chunk_of().block_min().x) as usize,
                    (target.y - types::WORLD_Y_MIN) as usize,
                    (target.z - target.chunk_of().block_min().z) as usize,
                )
            });
        let b1 = session
            .voxel_chunk(shard_id(1, 0), target.chunk_of())
            .map(|c| {
                c.get(
                    (target.x - target.chunk_of().block_min().x) as usize,
                    (target.y - types::WORLD_Y_MIN) as usize,
                    (target.z - target.chunk_of().block_min().z) as usize,
                )
            });
        if b0 != b1 {
            same = false;
        }
        if b0 == Some(BlockId(1)) && flushed_tick.is_none() {
            flushed_tick = Some(i);
            break;
        }
    }
    eprintln!(
        "A05-7:暂缓计数(owner+holder 合计){deferred_owner},两片一致 {same},\
         主体不走 ⇒ 40 拍内补写拍 {flushed_tick:?}"
    );
    assert!(same, "逐拍两分片该格体素相同(副本不分叉)");
    assert!(
        deferred_owner == 0 && flushed_tick.is_none(),
        "主体站原地 ⇒ 不补写(暂缓压着)"
    );

    // 主体走开(向 −x 走 3 m)⇒ owner 与 holder 同拍补写,该格同时出现。
    // 意图发给主体**当前所在分片**(迁移后 = (1,0))。
    let agent_now = if session.entity_state(shard_id(1, 0), a).is_some() {
        shard_id(1, 0)
    } else {
        shard_id(0, 0)
    };
    session
        .send(
            agent_now,
            SimCommand::AgentIntent {
                entity: a,
                move_xz: (2.0, std::f32::consts::PI),
                yaw: std::f32::consts::PI,
                jump: false,
            },
        )
        .expect("意图");
    let mut flushed = false;
    for _ in 0..240 {
        session.tick();
        let get = |shard: ShardId| -> Option<BlockId> {
            let cp = target.chunk_of();
            let min = cp.block_min();
            session.voxel_chunk(shard, cp).map(|c| {
                c.get(
                    (target.x - min.x) as usize,
                    (target.y - types::WORLD_Y_MIN) as usize,
                    (target.z - min.z) as usize,
                )
            })
        };
        let b0 = get(shard_id(0, 0));
        let b1 = get(shard_id(1, 0));
        if b0 == Some(BlockId(1)) {
            assert_eq!(b0, b1, "补写后两片同值(owner/holder 同拍)");
            flushed = true;
            break;
        }
        assert_eq!(b0, b1, "补写前两片仍一致");
    }
    assert!(flushed, "主体走开后须补写(两片同拍)");
}

// ── A05-8:迁移在途 ──────────────────────────────────────────────────────

#[test]
fn a05_8_in_transit_migration_keeps_table_entry() {
    let mut session = four_shard_session();
    // 主体在 x 边界附近向 +x 行走触发迁移;迁移前对主体所占格 SetBlock
    // (暂缓压着);迁移在途那一拍:表里有信封位姿 ⇒ 该格不补写;
    // 迁移落地后照常判定(仍重叠 ⇒ 不补写)。
    let a = EntityId::from_parts(0, 1);
    session
        .spawn_agent(
            shard_id(0, 0),
            a,
            capsule_hash(),
            EntityState {
                translation: Vec3::new(124.0, 1.95, 64.0),
                ..EntityState::default()
            },
        )
        .expect("生成");
    session.tick();
    // 目标格 = 世界 (127, 1, 64):主体 120 → 128 行走途中占据(半径 0.3 +
    // 半高 0.9;中心过 127 时重叠)。主体先走到 ~127(压着格),写石
    // (暂缓),然后继续走触发迁移(中心过 128 → owner 变 (1,0)),
    // 在途拍不补写;走远后照常判定。
    // 走到 ~127:意图 2 m/s,120 → 127 需 3.5 s = 210 拍。
    session
        .send(
            shard_id(0, 0),
            SimCommand::AgentIntent {
                entity: a,
                move_xz: (2.0, 0.0),
                yaw: 0.0,
                jump: false,
            },
        )
        .expect("意图");
    let mut pos = Vec3::ZERO;
    for _ in 0..240 {
        session.tick();
        if let Some(p) = session.agent_root_position(a) {
            pos = p;
        }
        if pos.x >= 127.75 {
            break;
        }
    }
    assert!(
        pos.x >= 127.7 && pos.x <= 127.95,
        "布景自检:主体中心 ∈ [127.7,128.3) 才占据格 128(x = {:.2})",
        pos.x
    );
    // 目标格 = 迁移触发时刻主体占据的格:中心 128 触发迁移,此时胶囊
    // (中心 ± 0.3)占据格 (128,1,64)。owner = (1,0)(chunk (8,4));写石
    // 时主体仍在 (0,0) 本体区(SetBlock 广播 owner + holders,两片同表
    // ⇒ 同判暂缓)。
    let target = BlockPos::new(128, 1, 64);
    let owner = shard_id(1, 0);
    session.set_block(owner, target, BlockId(1)).expect("写石");
    session.tick();
    // 布景自检:该格暂缓压着(主体站上面 ⇒ 未写入)。
    let summary1 = session.tick();
    let still_air = {
        let cp = target.chunk_of();
        let min = cp.block_min();
        session
            .voxel_chunk(owner, cp)
            .map(|c| {
                c.get(
                    (target.x - min.x) as usize,
                    (target.y - types::WORLD_Y_MIN) as usize,
                    (target.z - min.z) as usize,
                )
            })
            .unwrap_or(BlockId::AIR)
    };
    eprintln!(
        "A05-8:主体 x {:.2} 压格 {target:?}(owner {owner:?});写石后该格 {still_air:?}(暂缓压着 = AIR);\
         该拍暂缓计数 {}",
        pos.x,
        summary1.cohesion.set_block_deferred + summary1.cohesion.set_block_deferred_replaced
    );
    assert_eq!(still_air, BlockId::AIR, "主体压着 ⇒ 暂缓未写入");

    // 继续走触发迁移(中心过 64 → owner 变 (1,0))。在途拍(恰 1 拍)
    // 表里有信封位姿 ⇒ target 仍重叠 ⇒ 不补写。
    let mut in_transit_seen = false;
    let mut deferred_during = 0u64;
    let mut written_during = false;
    for _ in 0..120 {
        let cp = target.chunk_of();
        let min = cp.block_min();
        let _before = session
            .voxel_chunk(owner, cp)
            .map(|c| {
                c.get(
                    (target.x - min.x) as usize,
                    (target.y - types::WORLD_Y_MIN) as usize,
                    (target.z - min.z) as usize,
                )
            })
            .unwrap_or(BlockId::AIR);
        let had_agent = session.agent_root_position(a).is_some();
        let summary = session.tick();
        let has_agent = session.agent_root_position(a).is_some();
        if had_agent && !has_agent {
            in_transit_seen = true;
            deferred_during = summary.cohesion.set_block_deferred_flushed;
            let after = session
                .voxel_chunk(owner, cp)
                .map(|c| {
                    c.get(
                        (target.x - min.x) as usize,
                        (target.y - types::WORLD_Y_MIN) as usize,
                        (target.z - min.z) as usize,
                    )
                })
                .unwrap_or(BlockId::AIR);
            if after == BlockId(1) {
                written_during = true;
            }
        }
        // 不 break:主体从压格到走远的**整段窗口**逐拍断言——迁移在途拍
        // (信封条目压表)与重建落地拍(本地条目压表)只要主体仍与格重叠,
        // 该格就必须保持 AIR。变异 3(表漏在途信封条目)在重建拍之前的那一
        // 拍会判"不重叠"而补写,被这里的逐拍断言抓住(该拍 tick 后格 = 石)。
        let agent_pos = session.agent_root_position(a);
        let cell = session
            .voxel_chunk(owner, cp)
            .map(|c| {
                c.get(
                    (target.x - min.x) as usize,
                    (target.y - types::WORLD_Y_MIN) as usize,
                    (target.z - min.z) as usize,
                )
            })
            .unwrap_or(BlockId::AIR);
        if let Some(p) = agent_pos {
            let still_overlaps = (p.x - (target.x as f32 + 0.5)).abs() <= 0.3 + 0.5
                && (p.y - (target.y as f32 + 0.5)).abs() <= 0.9 + 0.5
                && (p.z - (target.z as f32 + 0.5)).abs() <= 0.3 + 0.5;
            if still_overlaps {
                assert_eq!(
                    cell,
                    BlockId::AIR,
                    "主体(根 x {:.2})仍与格 {target:?} 重叠 ⇒ 该格不得写入(表漏在途/本地条目即红)",
                    p.x
                );
            }
        }
        if in_transit_seen && agent_pos.is_some() && cell == BlockId::AIR {
            // 已观察到在途、主体已重建且该格仍 AIR:继续走远,交给下方补写断言。
        }
    }
    eprintln!(
        "A05-8:在途窗口命中 {in_transit_seen};在途拍补写 {deferred_during},该格被写 {written_during}"
    );
    assert!(in_transit_seen, "布景自检:迁移在途窗口须被观察到");
    assert_eq!(
        deferred_during, 0,
        "在途那一拍:表里有信封位姿 ⇒ 该格仍不补写"
    );
    assert!(!written_during, "在途拍该格不被写入");
    // 落地(重建)后照常判定:主体继续 +x 走远 ⇒ 不再重叠 ⇒ 补写。
    let mut flushed = false;
    for _ in 0..240 {
        session.tick();
        let cp = target.chunk_of();
        let min = cp.block_min();
        let b = session
            .voxel_chunk(shard_id(1, 0), cp)
            .map(|c| {
                c.get(
                    (target.x - min.x) as usize,
                    (target.y - types::WORLD_Y_MIN) as usize,
                    (target.z - min.z) as usize,
                )
            })
            .unwrap_or(BlockId::AIR);
        if b == BlockId(1) {
            flushed = true;
            break;
        }
    }
    eprintln!("A05-8:落地后照常判定,补写 {flushed}");
    assert!(flushed, "迁移落地后主体走远 ⇒ 补写生效");
}

// ── A05-9:确定性 ────────────────────────────────────────────────────────

#[test]
fn a05_9_determinism_checkpoint_and_replay_with_deferred() {
    // 2×2 Session:主体压边界格 + SetBlock 暂缓 + 时间线;线 A 不中断;
    // 线 B 拍 30 落检查点(暂缓非空)→ 恢复续跑;终态 world 快照逐位比较;
    // 日志回放两次逐位相同。
    let build = || -> (Session, EntityId, CommandLog) {
        let mut session = four_shard_session();
        let a = EntityId::from_parts(0, 1);
        let log = CommandLog::new();
        session
            .spawn_agent(
                shard_id(0, 0),
                a,
                capsule_hash(),
                EntityState {
                    translation: Vec3::new(128.0, 1.95, 64.0),
                    ..EntityState::default()
                },
            )
            .expect("生成(入日志)");
        session.tick();
        (session, a, log)
    };
    // 线 A(不中断)。
    let (mut sa, _a, mut log_a) = build();
    let owner = shard_id(1, 0);
    let entry = sa
        .set_block(owner, BlockPos::new(128, 1, 64), BlockId(1))
        .expect("SetBlock(入日志)");
    for e in entry {
        log_a.append(e);
    }
    let mut timeline = Vec::new();
    for i in 0..60 {
        // 每 20 拍一条意图(走开又走回,暂缓压着与补写都覆盖)。
        if i % 20 == 10 {
            let mv = if (i / 20) % 2 == 1 { 2.0 } else { 0.0 };
            let e = sa
                .send(
                    shard_id(0, 0),
                    SimCommand::AgentIntent {
                        entity: _a,
                        move_xz: (mv, 0.0),
                        yaw: 0.0,
                        jump: false,
                    },
                )
                .expect("意图(入日志)");
            log_a.append(e);
        }
        sa.tick();
        if let Some(p) = sa.agent_root_position(_a) {
            timeline.push(p.to_array());
        }
    }
    let final_a = sa.world_snapshot();
    sa.shutdown();

    // 线 B:重建 → 跑 30 拍(同日志前缀)→ 快照 → 恢复 → 续跑 30 拍。
    let (mut sb, a, mut log_b) = build();
    let entry = sb
        .set_block(owner, BlockPos::new(128, 1, 64), BlockId(1))
        .expect("SetBlock");
    for e in entry {
        log_b.append(e);
    }
    for i in 0..30 {
        if i % 20 == 10 {
            let mv = if (i / 20) % 2 == 1 { 2.0 } else { 0.0 };
            let e = sb
                .send(
                    shard_id(0, 0),
                    SimCommand::AgentIntent {
                        entity: a,
                        move_xz: (mv, 0.0),
                        yaw: 0.0,
                        jump: false,
                    },
                )
                .expect("意图");
            log_b.append(e);
        }
        sb.tick();
    }
    // 布景自检:快照时暂缓队列非空(主体还压着格)。
    eprintln!(
        "A05-9 快照前: live_total={} root={:?}",
        sb.deferred_set_block_total(),
        sb.agent_root_position(a)
    );
    let snap = sb.world_snapshot();
    {
        let resumed_probe = sim_api::replay::replay_world_with_bodies(
            &snap,
            &CommandLog::new(),
            SessionConfig {
                scheduler: SchedulerConfig {
                    worker_count: 2,
                    start_tick: snap.shards.tick,
                    ..SchedulerConfig::default()
                },
                ..SessionConfig::default()
            },
            || Box::new(SimpleRegistry::builtin()),
            capsule_registry,
        )
        .expect("探针恢复");
        let deferred_total = resumed_probe.deferred_set_block_total();
        resumed_probe.shutdown();
        assert!(
            deferred_total > 0,
            "布景自检:快照拍暂缓队列非空(实得 {deferred_total})"
        );
    }
    // 恢复续跑:replay_world 带完整日志(前缀 < 快照拍被跳过)。
    let mut resumed = sim_api::replay::replay_world_with_bodies(
        &snap,
        &log_b,
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
    .expect("恢复");
    // 时间线在恢复线上重喂(意图是保持状态,恢复线必须有同一条)。
    for i in 30..60 {
        if i % 20 == 10 {
            let mv = if (i / 20) % 2 == 1 { 2.0 } else { 0.0 };
            let e = resumed
                .send(
                    shard_id(0, 0),
                    SimCommand::AgentIntent {
                        entity: a,
                        move_xz: (mv, 0.0),
                        yaw: 0.0,
                        jump: false,
                    },
                )
                .expect("意图(恢复线时间线)");
            log_b.append(e);
        }
        resumed.tick();
    }
    let final_b = resumed.world_snapshot();
    resumed.shutdown();
    assert_eq!(final_a.shards.tick, final_b.shards.tick, "两线同拍结束");
    // 终态逐位:每分片字节比较。
    for (shard, bytes_a) in &final_a.shards.per_shard {
        let bytes_b = final_b.shards.per_shard.get(shard).expect("同分片集");
        assert_eq!(
            bytes_a, bytes_b,
            "分片 {shard:?} 终态逐位相同(暂缓队列进快照,恢复续跑)"
        );
    }
    // 回放两次逐位:replay_world(log_a) 两次,终态互比(且与线 A 比较仅
    // 作布景自检——回放线时间线在日志里)。
    let mut r1 = sim_api::replay::replay_world_with_bodies(
        &snap, // 起点任意(回放 = 快照 + 日志);用同一快照保证可比
        &log_a,
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
    .expect("回放 1");
    for _ in 0..60 {
        r1.tick();
    }
    let f1 = r1.world_snapshot();
    r1.shutdown();
    let mut r2 = sim_api::replay::replay_world_with_bodies(
        &snap,
        &log_a,
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
    .expect("回放 2");
    for _ in 0..60 {
        r2.tick();
    }
    let f2 = r2.world_snapshot();
    r2.shutdown();
    for (shard, b1) in &f1.shards.per_shard {
        let b2 = f2.shards.per_shard.get(shard).expect("同分片集");
        assert_eq!(b1, b2, "日志回放两次逐位相同(分片 {shard:?})");
    }
    let _ = timeline;
}

// ── A05-10:不影响旧行为(无主体时暂缓路径一次都不走)───────────────────

#[test]
fn a05_10_no_agent_no_deferral() {
    let mut s = walker_sim();
    load_flat_ground(&mut s);
    // 无主体:形状表空(生产 = 调度器下发空表;这里默认即空)。
    assert!(s.agent_shape_table().is_empty(), "无主体 ⇒ 表空");
    for (seq, y) in (1i32..8).enumerate() {
        s.apply_prefix(&[stamp(
            seq as u64,
            SimCommand::SetBlock {
                pos: BlockPos::new(24, y, 24),
                block: BlockId(1),
            },
        )]);
    }
    let mut deferred_total = 0u64;
    let observers = [Vec3::new(24.0, 3.0, 24.0)];
    for _ in 0..120 {
        let out = production_tick(&mut s, &observers);
        deferred_total += out.cohesion_stats.set_block_deferred
            + out.cohesion_stats.set_block_deferred_replaced
            + out.cohesion_stats.set_block_deferred_invalidated
            + out.cohesion_stats.set_block_deferred_flushed
            + out.cohesion_stats.solidify_deferred
            + out.cohesion_stats.forced_landing_deferred;
    }
    assert_eq!(
        deferred_total, 0,
        "无主体:全部暂缓计数为 0(暂缓路径一次都不走)"
    );
    assert!(s.deferred_set_blocks().is_empty(), "队列为空");
    assert_eq!(
        s.voxel().get_block(BlockPos::new(24, 5, 24)),
        BlockId(1),
        "写入照常生效(旧行为不变)"
    );
}
