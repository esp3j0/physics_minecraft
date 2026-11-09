//! REQ-012 A02b(#25):被物理影响(A02-6)、重物(A02-7)、不破坏地形(A02-8)
//! 的场景判据——胶囊主体在体素地形上按生产节奏(激活刷新 → apply_prefix →
//! step → lifecycle → outbox)被方块砸、被炸、被复合体撞;重物按 capsule demo
//! 的 `Scene::Heavy` / `HEAVY_GRID` / `D_GRID` 口径扫描;破坏系统打开时主体
//! 站立与跳台阶不破坏地形。
//!
//! 判据口径:
//! - **A02-6 被物理影响**:三种情况(方块砸中、爆炸、被复合体撞)主体都
//!   产生位移,每组写明位移量;主体意图为零(控制器在刹车),证明被推开的
//!   是物理不是控制器。
//! - **A02-7 重物**:half ∈ {1,2,4}(质量 8/64/512 = 8·h³)的整块从 5 m
//!   高砸下,落点扫 HEAVY_GRID × HEAVY_GRID(3×3 含擦边),每落点加
//!   D_GRID(5 档)高度微扰,共 45 组/尺寸;无 panic、无 NaN,胶囊水平
//!   位移 ≤ 10 m;每组列表写最大水平位移与最大速度。
//! - **A02-8 不破坏地形**:破坏系统打开(默认配置),主体站立 3600 拍、
//!   从 1 格台阶跳下各一组:实心方块数不变,破坏遥测(destruction_stats
//!   全字段)为 0。

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
use types::glam::Vec3;
use types::{
    BlockId, BlockPos, ChunkPos, CompoundShape, DimensionId, EntityDesc, EntityId, EntityState,
    ShapeCell, ShardId, ShardPos, Tick, WorldId,
};
use voxel::{Chunk, SimpleRegistry};

/// 重物落点(9 点,同 capsule demo 的 HEAVY_GRID / mass M7 口径)。
const HEAVY_GRID: [f32; 3] = [-0.3, 0.0, 0.3];
/// 高度微扰(capsule demo 的 D_GRID:撞击类必须扫落点并加微扰)。
const D_GRID: [f32; 5] = [0.0, 0.017, 0.034, 0.051, 0.068];

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

fn capsule_hash() -> types::BodyHash {
    capsule_registry().hash_of("capsule").expect("capsule 已装")
}

/// 默认配置(凝固 + 破坏都开 = 生产)+ capsule 注册表的分片。
fn walker_sim() -> ShardSim {
    let cfg = ShardSimConfig::new(shard_id(0, 0));
    ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), capsule_registry())
}

/// 平板地形:3×3 chunk,层 world y = 0 全石(顶面 y = 1)。
fn load_flat_ground(sim: &mut ShardSim) {
    let stone = BlockId(1);
    for cx in 0..3i32 {
        for cz in 0..3i32 {
            let mut chunk = Chunk::air();
            for lx in 0..voxel::CHUNK_X {
                for lz in 0..voxel::CHUNK_Z {
                    chunk.set(lx, 64, lz, stone);
                }
            }
            sim.load_chunk(ChunkPos::new(cx, cz), chunk);
        }
    }
}

/// 盖章输入(调度器来源;本地受信通道语义)。
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

/// 生产节奏的一拍(worker 同序;激活开启 = 带观察者)。
fn production_tick(sim: &mut ShardSim, observers: &[Vec3]) {
    if !observers.is_empty() {
        sim.apply_activation(observers);
    }
    sim.step();
    sim.run_transient_lifecycle();
    let _ = sim.take_outbox();
}

/// 装载 chunk 的实心方块数(A02-8 的地形不变口径)。
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

// ── A02-6 被物理影响 ─────────────────────────────────────────────────────

/// 主体安定(零意图,60 拍落地)后的布景基座:返回 (sim, 主体, 安定后质心)。
fn settled_agent() -> (ShardSim, EntityId, Vec3) {
    let mut s = walker_sim();
    load_flat_ground(&mut s);
    let a = EntityId::from_parts(1, 1);
    s.apply_prefix(&[
        stamp(0, spawn_agent_cmd(a, Vec3::new(24.0, 1.95, 24.0))),
        // 意图显式为零(控制器在刹车)——被推开的只能是物理,不是控制器。
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

/// 三种被物理影响的情况,主体意图全为零(控制器在刹车),位移全由物理给出:
/// 1. 方块砸中:落点扫 3×3 网格(±0.3,含擦边)× 高度微扰 5 档 = 45 组
///    (中心正砸只垂直压缩、水平位移近 0——A01 R1-P4 同款结论;水平位移
///    判据看**窗口最大值**,擦边组给出水平推力);
/// 2. 爆炸:主体旁 2.5 m Detonate(power 12, radius 5),跑 120 拍;
/// 3. 被复合体撞:主体 +x 侧 3.5 m 生成 2 格横条复合体(SpawnCompound,
///    初速 −8 m/s 直冲主体),跑 240 拍。
///
/// 每组断言窗口最大位移 > 阈值;每组位移量 eprintln 出来(回报逐组列明)。
#[test]
fn a02_6_agent_displaced_by_block_blast_and_compound() {
    // ---- 组 1:方块砸中(45 组扫描)----
    {
        let mut worst = (0.0f32, (0.0f32, 0.0f32, 0.0f32));
        let mut nonzero = 0usize;
        for &ox in &HEAVY_GRID {
            for &oz in &HEAVY_GRID {
                for &d in &D_GRID {
                    let (mut s, a, base) = settled_agent();
                    let observers = [Vec3::new(24.0, 3.0, 24.0)];
                    // 方块生成在胶囊顶(top = base.y + 0.9)+ 0.5 + 3.0 + d
                    // (capsule demo 的 Hit 布景高度口径)。
                    let top = base.y + 0.9;
                    s.apply_prefix(&[stamp(
                        2,
                        SimCommand::SpawnDynamic {
                            entity: EntityId::from_parts(1, 2),
                            state: EntityState {
                                translation: Vec3::new(base.x + ox, top + 3.5 + d, base.z + oz),
                                ..EntityState::default()
                            },
                            desc: EntityDesc::default(),
                        },
                    )]);
                    let mut max_disp = 0.0f32;
                    for _ in 0..300 {
                        production_tick(&mut s, &observers);
                        let st = s.entity_state(a).expect("组 1 主体全程在");
                        let disp = (st.translation - base).length();
                        max_disp = max_disp.max(disp);
                    }
                    if max_disp > 0.0 {
                        nonzero += 1;
                    }
                    if max_disp > worst.0 {
                        worst = (max_disp, (ox, oz, d));
                    }
                }
            }
        }
        eprintln!(
            "A02-6 组 1(方块砸中,45 组):位移全 > 0 的组数 {nonzero}/45;最大 {:.4} m(落点 ({},{}), 微扰 {})",
            worst.0, worst.1.0, worst.1.1, worst.1.2
        );
        assert!(
            worst.0 > 0.05,
            "方块砸中须推动主体:45 组窗口最大位移只有 {:.4} m",
            worst.0
        );
    }
    // ---- 组 2:爆炸 ----
    {
        let (mut s, a, base) = settled_agent();
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        s.apply_prefix(&[stamp(
            2,
            SimCommand::Detonate {
                center: Vec3::new(base.x - 2.5, 1.0, base.z),
                power: 12.0,
                radius: 5.0,
            },
        )]);
        let mut max_disp = 0.0f32;
        for _ in 0..120 {
            production_tick(&mut s, &observers);
            let st = s.entity_state(a).expect("组 2 主体全程在");
            let disp = (st.translation - base).length();
            max_disp = max_disp.max(disp);
        }
        let st = s.entity_state(a).expect("组 2 主体全程在");
        let disp = (st.translation - base).length();
        eprintln!("A02-6 组 2(爆炸):末态位移 {disp:.4} m,窗口最大 {max_disp:.4} m");
        assert!(disp > 0.01, "爆炸须炸动主体:末态位移 {disp:.4} m");
    }
    // ---- 组 3:被复合体撞(SpawnCompound)----
    {
        let (mut s, a, base) = settled_agent();
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        // 2 格横条(世界格 y = 1,即地面上一格;中心世界 y = 1.5),origin
        // 在主体 +x 侧 3 m 外(格 (28,1,23) → 世界中心 (28.5,1.5,23.5) 与
        // (29.5,1.5,23.5)),初速 (−8,0,0) 直冲主体。
        let shape = CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [1, 0, 0],
                block: BlockId(1),
            },
        ])
        .expect("2 格横条合法");
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
        let mut max_disp = 0.0f32;
        for _ in 0..240 {
            production_tick(&mut s, &observers);
            let st = s.entity_state(a).expect("组 3 主体全程在");
            let disp = (st.translation - base).length();
            max_disp = max_disp.max(disp);
        }
        let st = s.entity_state(a).expect("组 3 主体全程在");
        let disp = (st.translation - base).length();
        eprintln!("A02-6 组 3(复合体撞):末态位移 {disp:.4} m,窗口最大 {max_disp:.4} m");
        assert!(disp > 0.05, "复合体撞击须推动主体:末态位移 {disp:.4} m");
    }
}

// ── A02-7 重物 ──────────────────────────────────────────────────────────

/// 重物一组(h = 半边长,质量 8·h³;ox/oz 落点偏移;d 高度微扰):主体安定
/// 60 拍后按当拍质心定位生成重物(top + h + 5 + d),再跑 300 拍。返回
/// (窗口最大水平位移, 窗口最大速度)。主体全程必须有限、在场(capsule demo
/// 的 Heavy 口径:速度/位置逐拍 is_finite 检查;内核的失控防护在 B01,
/// demo 实测 V_GUARD 200 下 0 次触发)。
fn heavy_one(h: f32, ox: f32, oz: f32, d: f32) -> (f32, f32) {
    let (mut s, a, base) = settled_agent();
    let observers = [Vec3::new(24.0, 3.0, 24.0)];
    let top = base.y + 0.9;
    // 整块 = half_size h 的单格动态实体(密度 1 → 质量 (2h)³ = 8·h³)。
    s.apply_prefix(&[stamp(
        2,
        SimCommand::SpawnDynamic {
            entity: EntityId::from_parts(1, 2),
            state: EntityState {
                translation: Vec3::new(base.x + ox, top + h + 5.0 + d, base.z + oz),
                ..EntityState::default()
            },
            desc: EntityDesc {
                half_size: h,
                block: BlockId(1),
                shape: None,
            },
        },
    )]);
    // 质量自检(capsule demo 的 assert 同款):m = 8·h³。
    let m = s
        .entity_mass(EntityId::from_parts(1, 2))
        .expect("重物已生成");
    let want = 8.0 * h * h * h;
    assert!(
        (m - want).abs() < want * 1e-3,
        "重物质量 {m} != 8·h³ = {want}"
    );
    let mut max_disp_h = 0.0f32;
    let mut max_speed = 0.0f32;
    for _ in 0..300 {
        production_tick(&mut s, &observers);
        let st = s.entity_state(a).expect("重物组主体全程在");
        assert!(
            st.translation.x.is_finite()
                && st.translation.y.is_finite()
                && st.translation.z.is_finite()
                && st.linvel.x.is_finite()
                && st.linvel.y.is_finite()
                && st.linvel.z.is_finite(),
            "重物 m{} 落点 ({ox},{oz}) 微扰 {d}:主体出现非有限状态",
            want as i32
        );
        let dx = st.translation.x - base.x;
        let dz = st.translation.z - base.z;
        max_disp_h = max_disp_h.max((dx * dx + dz * dz).sqrt());
        max_speed = max_speed.max(st.linvel.length());
    }
    (max_disp_h, max_speed)
}

/// 重物扫描(一个尺寸一个测试,45 组/尺寸,组内串行):无 panic、无 NaN、
/// 每组水平位移 ≤ 10 m;每组打印最大水平位移与最大速度(回报列表)。
macro_rules! heavy_scan {
    ($fn_name:ident, $h:expr, $mass:expr) => {
        #[test]
        fn $fn_name() {
            let h: f32 = $h;
            let mut worst_disp = 0.0f32;
            let mut worst_speed = 0.0f32;
            let mut worst_at = (0.0f32, 0.0f32, 0.0f32);
            for &ox in &HEAVY_GRID {
                for &oz in &HEAVY_GRID {
                    for &d in &D_GRID {
                        let (disp, speed) = heavy_one(h, ox, oz, d);
                        assert!(
                            disp <= 10.0,
                            "重物 {} 落点 ({ox},{oz}) 微扰 {d}:水平位移 {disp:.3} m > 10 m",
                            $mass
                        );
                        if disp > worst_disp {
                            worst_disp = disp;
                            worst_at = (ox, oz, d);
                        }
                        worst_speed = worst_speed.max(speed);
                    }
                }
            }
            eprintln!(
                "A02-7 重物 m{}(h = {}):45 组全过;最大水平位移 {:.4} m(落点 ({},{}), 微扰 {}),最大速度 {:.4} m/s",
                $mass, h, worst_disp, worst_at.0, worst_at.1, worst_at.2, worst_speed
            );
        }
    };
}

heavy_scan!(a02_7_heavy_m8_scan_45, 1.0, 8);
heavy_scan!(a02_7_heavy_m64_scan_45, 2.0, 64);
heavy_scan!(a02_7_heavy_m512_scan_45, 4.0, 512);

// ── A02-8 不破坏地形 ────────────────────────────────────────────────────

/// 一组的破坏遥测累计(每拍 take_outbox 的 destruction_stats 逐字段求和)。
#[derive(Default, Debug)]
struct DestruSum {
    cells_blasted: u64,
    cells_impacted: u64,
    cells_collapsed: u64,
    detached: u64,
    pulverized: u64,
    pulverized_over_cap: u64,
}

impl DestruSum {
    fn add(&mut self, s: &physics_sim::destruction::DestructionStats) {
        self.cells_blasted += s.cells_blasted;
        self.cells_impacted += s.cells_impacted;
        self.cells_collapsed += s.cells_collapsed;
        self.detached += s.detached;
        self.pulverized += s.pulverized;
        self.pulverized_over_cap += s.pulverized_over_cap;
    }
    fn total(&self) -> u64 {
        self.cells_blasted
            + self.cells_impacted
            + self.cells_collapsed
            + self.detached
            + self.pulverized
            + self.pulverized_over_cap
    }
}

/// A02-8 第一组:破坏系统打开(默认配置),主体站立 3600 拍(A02-5 布景
/// 同款:意图为零):实心方块数不变、破坏遥测全 0。
#[test]
fn a02_8_stand_3600_ticks_terrain_unchanged_zero_destruction() {
    let mut s = walker_sim();
    load_flat_ground(&mut s);
    let baseline = solid_count(&s);
    let a = EntityId::from_parts(1, 1);
    s.apply_prefix(&[
        stamp(0, spawn_agent_cmd(a, Vec3::new(48.0, 1.95, 48.0))),
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
    let observers = [Vec3::new(48.0, 3.0, 48.0)];
    let mut destru = DestruSum::default();
    for _ in 0..3600 {
        if !observers.is_empty() {
            s.apply_activation(&observers);
        }
        s.step();
        s.run_transient_lifecycle();
        destru.add(&s.take_outbox().destruction_stats);
    }
    assert_eq!(
        solid_count(&s),
        baseline,
        "站立 3600 拍地形方块数不变(主体站着不破坏任何地形)"
    );
    assert_eq!(
        destru.total(),
        0,
        "站立 3600 拍破坏遥测全 0(实测 {destru:?})"
    );
    assert!(s.entity_state(a).is_some(), "主体全程在(未凝固写块/未移除)");
}

/// A02-8 第二组:从 1 格台阶跳下。布景:平板 + 台阶(世界 x 8..16 抬高
/// 1 格,顶面 y = 2),主体出生在台阶顶 (12, 2.95, 8),意图 (−4.3, 0) 跑
/// 向台阶前沿(x = 8)越过边缘跳下落回地面(顶面 y = 1)。480 拍内:主体
/// 回到地面(x < 8 区域)站定,实心方块数不变、破坏遥测全 0。
#[test]
fn a02_8_step_down_terrain_unchanged_zero_destruction() {
    let mut s = walker_sim();
    let stone = BlockId(1);
    // 平板 3×1(x 0..48,z 0..16)+ 台阶在 chunk (0,0)(世界 x 8..16 抬高 1 格)。
    for cx in 0..3i32 {
        let mut chunk = Chunk::air();
        for lx in 0..voxel::CHUNK_X {
            for lz in 0..voxel::CHUNK_Z {
                chunk.set(lx, 64, lz, stone);
                if cx == 0 && (8..16).contains(&lx) {
                    chunk.set(lx, 65, lz, stone);
                }
            }
        }
        s.load_chunk(ChunkPos::new(cx, 0), chunk);
    }
    let baseline = solid_count(&s);
    let a = EntityId::from_parts(1, 1);
    // 出生在台阶顶面上(台阶顶 y = 2,站立中心 = 2 + 0.9 = 2.9)。
    s.apply_prefix(&[
        stamp(0, spawn_agent_cmd(a, Vec3::new(12.0, 2.95, 8.0))),
        stamp(
            1,
            SimCommand::AgentIntent {
                entity: a,
                move_xz: (-4.3, 0.0),
                yaw: 0.0,
                jump: false,
            },
        ),
    ]);
    let observers = [Vec3::new(12.0, 4.0, 8.0)];
    let mut destru = DestruSum::default();
    let mut reached_ground = false;
    let mut end_state = None;
    let mut seq = 2u64;
    for _ in 0..480 {
        if !observers.is_empty() {
            s.apply_activation(&observers);
        }
        s.step();
        s.run_transient_lifecycle();
        destru.add(&s.take_outbox().destruction_stats);
        if let Some(st) = s.entity_state(a) {
            // 跳下后回到平地:质心 y ≈ 1.9(站面 y=1 + 半高 0.9)且已离开
            // 台阶区(x < 8 前沿)。首次到达即清零意图(控制器刹车),主体
            // 停在平地上——布景纪律:主体远离分片边界,不能一路走出本体区。
            if !reached_ground && (st.translation.y - 1.9).abs() < 0.3 && st.translation.x < 7.0 {
                reached_ground = true;
                s.apply_prefix(&[stamp(
                    seq,
                    SimCommand::AgentIntent {
                        entity: a,
                        move_xz: (0.0, 0.0),
                        yaw: 0.0,
                        jump: false,
                    },
                )]);
                seq += 1;
            }
            end_state = Some(st);
        }
    }
    assert!(
        reached_ground,
        "布景自检:480 拍内主体从 1 格台阶跳下回到地面"
    );
    // 末态自检:落地后水平速度已衰减(刹车生效,不是匀速滑行穿过判据窗)。
    let st = end_state.expect("reached_ground 为真必有末态");
    assert!(
        st.linvel.length() < 4.3,
        "布景自检:末态主体已落地趋停(速度 {:.2} m/s)",
        st.linvel.length()
    );
    assert_eq!(
        solid_count(&s),
        baseline,
        "跳台阶全程地形方块数不变(主体走路/摔落不破坏地形)"
    );
    assert_eq!(destru.total(), 0, "跳台阶全程破坏遥测全 0(实测 {destru:?})");
}
