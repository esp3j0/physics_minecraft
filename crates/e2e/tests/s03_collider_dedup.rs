//! S03:碰撞体重建去重 + 足迹口径修正(总架构师放行评论,#24)。
//!
//! 四条测试各自钉住一项行为,并已在**修复前实现**上验证变红(回评附验证方式):
//! 1. 去重计数:同一拍多条写入(apply_prefix 信封路径 / 凝固路径)指向同一
//!    chunk 时,该拍重建次数 = 去重后的 chunk 集大小,不再逐条重建;
//! 2. 同拍生效:`apply_prefix` 的写入仍在同拍 `phys.step()` 之前生效(放/拆
//!    方块当拍托住/释放上方实体)——"去重实现忘了 flush"在这里变红;
//! 3. 确定性:多写入场景两次运行逐位相同;中途 checkpoint 恢复续跑与不中断
//!    逐位相同(重建次数变化会改刚体句柄分配序,任何非确定性在这里暴露);
//! 4. 足迹:`entity_motion_of` 两个分支对复合体返回包络口径(÷√3 等效
//!    half_size)、单格返回 half_size 原样(旧实现:在途分支 `desc.half_size`、
//!    分片内分支写死 0.5,复合体足迹过窄)。

use std::collections::BTreeSet;

use app::driver::{
    CHECKPOINT_FORMAT_VERSION, CheckpointBundle, decode_checkpoint, encode_checkpoint,
};
use messaging::InputSource;
use physics_sim::destruction::DEBRIS_HALF_SIZE;
use shard::SchedulerConfig;
use sim_api::log::CommandLog;
use sim_api::replay;
use sim_api::session::{Session, SessionConfig};
use types::glam::{Quat, Vec3};
use types::{
    BlockId, BlockPos, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos,
    WorldId, WorldSeed,
};
use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

fn shard() -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0))
}

fn cfg() -> SessionConfig {
    SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    }
}

fn registry() -> Box<dyn BlockRegistry> {
    Box::new(SimpleRegistry::builtin())
}

/// 单分片会话:chunk (2..=6, 2..=6) 平地已装载(地表顶面 = 世界 y −60)。
/// `load_enabled` = destruction ∧ cohesion 默认全开(承载判定生效:空中
/// 放置的悬空格会被当拍转换回 air,布景必须贴地)。
fn session() -> Session {
    let sim_cfg = physics_sim::ShardSimConfig::new(shard());
    let mut sim = physics_sim::ShardSim::new(sim_cfg, registry());
    for cx in 2..=6 {
        for cz in 2..=6 {
            let cp = ChunkPos::new(cx, cz);
            sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
        }
    }
    let mut s = Session::new(cfg());
    s.register(sim);
    s
}

/// 纯物理会话:破坏/承载判定关闭(SetBlock 不经承载转换,放置不被塌掉)、
/// 凝固关闭(实体保持动态,拆块测试不被凝固吞掉)。
fn session_pure_physics() -> Session {
    let mut sim_cfg = physics_sim::ShardSimConfig::new(shard());
    sim_cfg.destruction.enabled = false;
    sim_cfg.cohesion.enabled = false;
    sim_cfg.solidify.enabled = false;
    let mut sim = physics_sim::ShardSim::new(sim_cfg, registry());
    for cx in 2..=6 {
        for cz in 2..=6 {
            let cp = ChunkPos::new(cx, cz);
            sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
        }
    }
    let mut s = Session::new(cfg());
    s.register(sim);
    s
}

fn set_block_cmd(s: &mut Session, pos: BlockPos, block: BlockId) {
    s.set_block(shard(), pos, block).expect("SetBlock 入队");
}

/// 观察者就位 + 激活装载收敛(② 集连续 3 拍不变,上限 60 拍):装载路径的
/// 立即重建只发生在收敛前,之后的写拍计数才只剩去重后的重建。
fn warm_until_settled(s: &mut Session, center: Vec3) {
    s.move_observer(center).expect("观察者");
    let mut stable = 0u32;
    let mut last = usize::MAX;
    for _ in 0..60 {
        let t = s.tick().telemetry.chunks_collider;
        let _ = s.take_solidified();
        let _ = s.take_block_updates();
        if t == last {
            stable += 1;
        } else {
            stable = 0;
        }
        last = t;
        if stable >= 3 {
            break;
        }
    }
}

// ================================================================= 1. 去重计数

/// 同一拍 `apply_prefix` 多条写入指向同一 chunk(自身 + 4 已装载邻居),去重后
/// 该拍重建次数恰 = expand_boundary 集大小;旧实现(逐条立即重建)是条数倍。
///
/// 变红验证(修复前实现):同拍 3 条 SetBlock(同一 chunk)→ 重建 15 次
/// (3 × 5 chunk),新实现 5 次;凝固路径同拍 2 实体凝固到同一 chunk 同理。
#[test]
fn dedup_same_tick_writes_rebuild_each_chunk_once() {
    let mut s = session();
    warm_until_settled(&mut s, Vec3::new(64.5, -55.5, 64.5));
    // 收敛拍计数归零(collider_rebuilds = 每拍当拍值,取走即清)。
    let _ = s.tick();
    // 同拍 3 条 SetBlock,替换地表顶层格(贴地实心→实心,承载判定不转换;
    // 全部落在 chunk (4,4),x 64..66 不贴 chunk 边界)。
    let cells = [
        BlockPos::new(64, -60, 64),
        BlockPos::new(65, -60, 64),
        BlockPos::new(66, -60, 64),
    ];
    for pos in cells {
        set_block_cmd(&mut s, pos, builtin::STONE);
    }
    let summary = s.tick();
    let delta = summary.telemetry.collider_rebuilds;
    // 去重后:chunk (4,4) 及其 4 个已装载邻居各重建一次 = 5。承载求值无
    // falling(贴地替换)、凝固桥无实体;装配/生命周期路径本拍无动作。
    assert_eq!(
        delta, 5,
        "同拍 3 条写入同一 chunk 应只重建 5 个 chunk 各一次,实得 {delta}(旧实现 = 15)"
    );
    // 写入已生效(替换成 stone)。
    assert_eq!(
        s.voxel_chunk(shard(), ChunkPos::new(4, 4))
            .map(|c| c.get(0, 4, 0)),
        Some(builtin::STONE)
    );
    s.shutdown();
}

/// 凝固路径的去重:两个实体同拍凝固到同一 chunk,该拍重建次数仍 = 边界集
/// 大小(旧实现逐实体重建 ×2)。
#[test]
fn dedup_solidify_same_tick_same_chunk() {
    // 凝固参数放宽:低速即凝、位移门关、阈值 1 拍。
    let mut sim_cfg = physics_sim::ShardSimConfig::new(shard());
    sim_cfg.solidify = physics_sim::SolidifyConfig {
        linear_threshold: 10.0,
        angular_threshold: 10.0,
        consecutive_ticks: 1,
        displacement_gate_m: 0.0,
        ..physics_sim::SolidifyConfig::default()
    };
    let mut sim = physics_sim::ShardSim::new(sim_cfg, registry());
    for cx in 2..=6 {
        for cz in 2..=6 {
            let cp = ChunkPos::new(cx, cz);
            sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
        }
    }
    let mut s = Session::new(cfg());
    s.register(sim);
    warm_until_settled(&mut s, Vec3::new(64.5, -55.5, 64.5));
    let _ = s.tick();
    // 两个单格实体贴地 spawn(体心 y = −59.5,snap 格 = 地表顶层);放宽
    // 参数下**spawn 拍 step 即凝固**(slow_ticks 1 ≥ 阈值 1):两实体同拍写
    // 同一 chunk (4,4)(凝固落点 (64/65,−60,64) 替换地表格)。
    for i in 0..2u32 {
        let id = EntityId::from_parts(1, 10 + i);
        s.spawn_owned(
            shard(),
            id,
            EntityState {
                translation: Vec3::new(64.5 + i as f32, -59.5, 64.5),
                rotation: Quat::IDENTITY,
                linvel: Vec3::ZERO,
                angvel: Vec3::ZERO,
            },
            EntityDesc::default(),
            InputSource(7),
        )
        .expect("spawn");
    }
    // spawn 拍 = 凝固拍:两实体同拍凝固到同一 chunk,去重后 = chunk (4,4)
    // + 4 已装载邻居 = 5(旧实现逐实体立即重建 = 10)。
    let summary = s.tick();
    let delta = summary.telemetry.collider_rebuilds;
    assert_eq!(
        delta, 5,
        "两实体同拍凝固到同一 chunk 应只重建 5 个 chunk 各一次,实得 {delta}(旧实现 = 10)"
    );
    assert_eq!(summary.telemetry.entities_awake, 0, "两实体都已凝固");
    // 下一拍无新写入(凝固格替换、承载判定贴地格无 falling):零重建。
    let after = s.tick().telemetry.collider_rebuilds;
    assert_eq!(after, 0, "凝固后无新写入,不应有重建,实得 {after}");
    s.shutdown();
}

// ================================================================= 2. 同拍生效

/// `apply_prefix` 的写入必须在同拍 `phys.step()` 之前生效(放行原文两向)。
/// 纯物理布景(承载/凝固关:SetBlock 不被承载转换吞掉、实体不被凝固收走)。
/// (a) 本拍放出的方块托住本拍正在下落的实体——放块拍结束时接触解算已把
///     入射速度(≈ −3 m/s)基本止住(|vy| < 1.5);
/// (b) 本拍拆掉的方块,本拍 step 后实体已开始下落(拆块拍 vy ≤ −0.1)。
///
/// 变红验证(去重实现漏 flush,写入延迟到下一拍):(a) 放块拍实体仍自由
/// 落体(vy ≈ −3.3 > 1.5 下限)→ 红;(b) 拆块拍实体仍被旧 collider 托住
/// (vy ≈ 0)→ 红。
#[test]
fn prefix_write_takes_effect_same_tick_step() {
    let mut s = session_pure_physics();
    warm_until_settled(&mut s, Vec3::new(64.5, -55.5, 64.5));
    // (a) 实体 spawn 在地表上方 ~3.5 m,自由落体 15 拍(速度 ≈ −2.5 m/s)。
    let id = EntityId::from_parts(1, 1);
    s.spawn_owned(
        shard(),
        id,
        EntityState {
            translation: Vec3::new(64.5, -56.5, 64.5),
            rotation: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
        EntityDesc::default(),
        InputSource(7),
    )
    .expect("spawn");
    for _ in 0..15 {
        let _ = s.tick();
    }
    let st = s.entity_state(shard(), id).expect("实体在场");
    let y_before = st.translation.y;
    let v_before = st.linvel.y;
    assert!(v_before < -2.0, "前置:实体应在下落中,实得 vy = {v_before}");
    // 同一拍:在实体底面正下方 0.05 m 放方块(顶面 = floor(底面 − 0.05) + 1,
    // 布景不嵌格;承载判定已关,空中放置不会被塌掉)。写入同拍生效时本拍
    // step 接触解算把 vy 止住;延迟一拍时本拍仍自由落体(vy 更低)。
    let block_y = ((y_before - 0.45 - 0.05).floor() as i32) + 1;
    set_block_cmd(&mut s, BlockPos::new(64, block_y, 64), builtin::DIRT);
    let _ = s.tick();
    let st = s.entity_state(shard(), id).expect("实体在场");
    assert!(
        st.linvel.y.abs() < 1.5,
        "方块同拍生效:放块拍接触后 vy 应已基本止住,实得 vy = {}(入射 {v_before})",
        st.linvel.y
    );
    // (b) 实体已停在方块顶:再跑 10 拍收敛(速度归零)。
    for _ in 0..10 {
        let _ = s.tick();
    }
    let st = s.entity_state(shard(), id).expect("实体在场");
    assert!(
        st.linvel.y.abs() < 0.05,
        "实体应停在方块顶(速度归零),实得 vy = {}",
        st.linvel.y
    );
    // 同一拍拆掉方块:同拍生效时 step 后实体已开始下落(vy ≤ −0.1);
    // 延迟实现当拍仍被旧 collider 托住(vy ≈ 0)。
    set_block_cmd(&mut s, BlockPos::new(64, block_y, 64), builtin::AIR);
    let _ = s.tick();
    let st = s.entity_state(shard(), id).expect("实体在场");
    assert!(
        st.linvel.y <= -0.1,
        "拆方块同拍生效:本拍 step 后实体应已开始下落,实得 vy = {}",
        st.linvel.y
    );
    s.shutdown();
}

// ================================================================= 3. 确定性

/// 多写入场景(爆炸 563 格写 + 同拍凝固/碎块生成)跑两次逐位相同;在写入
/// 最密的拍取 checkpoint 恢复续跑,与不中断线逐位相同。
///
/// 变红验证:重建次数/顺序随执行路径变化(如 flush 序不确定、BTreeSet 换
/// Vec)→ 刚体句柄分配序变化,`entity_states`/体素快照出现位级差异。
#[test]
fn determinism_repeat_and_checkpoint_resume() {
    let center = Vec3::new(64.5, -59.0, 64.5);
    // --- 线 A:不中断,跑 40 拍。
    let mut a = session();
    a.move_observer(center).expect("观察者");
    a.tick();
    let mut log = CommandLog::new();
    for entry in a
        .detonate_from(InputSource(9), center, 40.0, 8.0)
        .expect("爆炸")
    {
        log.append(entry);
    }
    let mut frames_a: Vec<Vec<(EntityId, EntityState)>> = Vec::new();
    let mut snap_a: Vec<Vec<u8>> = Vec::new();
    for _ in 0..40 {
        a.tick();
        let _ = a.take_solidified();
        let _ = a.take_block_updates();
        // 每拍全量分片快照字节(含 PhysicsWorld:collider 刚体句柄分配序、
        // 接触对状态)——重建序的位级观测面,比实体/体素表细一层。
        snap_a.push(encode_checkpoint(&CheckpointBundle {
            format_version: CHECKPOINT_FORMAT_VERSION,
            world: a.world_snapshot(),
            log_bytes: Vec::new(),
            spawn_next_seq: 0,
            controllers: Vec::new(),
        }));
        frames_a.push(a.entity_states(shard()).unwrap_or_default());
    }

    // --- 线 B:同输入跑第二遍。
    let mut b = session();
    b.move_observer(center).expect("观察者");
    b.tick();
    let _ = b
        .detonate_from(InputSource(9), center, 40.0, 8.0)
        .expect("爆炸");
    for i in 0..40 {
        b.tick();
        let _ = b.take_solidified();
        let _ = b.take_block_updates();
        let snap_b = encode_checkpoint(&CheckpointBundle {
            format_version: CHECKPOINT_FORMAT_VERSION,
            world: b.world_snapshot(),
            log_bytes: Vec::new(),
            spawn_next_seq: 0,
            controllers: Vec::new(),
        });
        assert_eq!(
            snap_b, snap_a[i],
            "第 {i} 拍两线分片快照字节分叉(重建序不确定?)"
        );
        assert_eq!(
            b.entity_states(shard()),
            Some(frames_a[i].clone()),
            "第 {i} 拍两线实体分叉(重建序不确定?)"
        );
    }

    // --- 线 C:第 3 拍(写入最密、碎块刚生成)取 checkpoint,恢复续跑 37 拍。
    let mut c = session();
    c.move_observer(center).expect("观察者");
    c.tick();
    let mut log_c = CommandLog::new();
    for entry in c
        .detonate_from(InputSource(9), center, 40.0, 8.0)
        .expect("爆炸")
    {
        log_c.append(entry);
    }
    for _ in 0..3 {
        c.tick();
        let _ = c.take_solidified();
        let _ = c.take_block_updates();
    }
    let bundle = CheckpointBundle {
        format_version: CHECKPOINT_FORMAT_VERSION,
        world: c.world_snapshot(),
        log_bytes: log_c.to_bytes(),
        spawn_next_seq: 0,
        controllers: Vec::new(),
    };
    let back = decode_checkpoint(&encode_checkpoint(&bundle)).expect("往返");
    let replay_log = CommandLog::from_bytes(&back.log_bytes).expect("日志");
    let mut r = replay::replay_world(&back.world, &replay_log, cfg(), || {
        Box::new(SimpleRegistry::builtin())
    })
    .expect("恢复");
    for i in 3..40 {
        r.tick();
        let _ = r.take_solidified();
        let _ = r.take_block_updates();
        let snap_r = encode_checkpoint(&CheckpointBundle {
            format_version: CHECKPOINT_FORMAT_VERSION,
            world: r.world_snapshot(),
            log_bytes: Vec::new(),
            spawn_next_seq: 0,
            controllers: Vec::new(),
        });
        assert_eq!(
            snap_r, snap_a[i],
            "checkpoint 恢复线第 {i} 拍分片快照字节与不中断线分叉"
        );
        assert_eq!(
            r.entity_states(shard()),
            Some(frames_a[i].clone()),
            "checkpoint 恢复线第 {i} 拍与不中断线分叉"
        );
    }
    a.shutdown();
    b.shutdown();
    c.shutdown();
    r.shutdown();
}

// ================================================================= 4. 足迹

/// `entity_motion_of` 两个分支(经 Scheduler 私有函数的公开缝 = 票视图解析,
/// 这里直接对复合体/单格断言:足迹覆盖的 chunk 数按包络口径变宽)。
/// 直测面:瞬态运动票(足迹装载义务)覆盖的 chunk 数——复合体的足迹必须
/// 覆盖包络 AABB(÷√3 等效 half_size),单格保持 half_size 原样。
///
/// 变红验证(旧实现):复合体在途/分片内足迹按 0.5 算 → 覆盖 1 个 chunk;
/// 新实现(ρ/√3)覆盖 ≥ 2 个。单格旧新一致(回归保护)。
#[test]
fn footprint_uses_envelope_for_compound_and_half_for_single() {
    // 纯函数口径直测:复合体 (6,1,1) 格、half 0.45 → ρ = 3 + 0.45√3 ≈ 3.78
    // (最远偏移 |(3,0,0)| = 3,与同函数后文一致),等效 half = ρ/√3 ≈ 2.18;
    // 单格 half 0.45 原样。
    let cells: Vec<(BlockPos, BlockId)> = (0..6)
        .map(|i| (BlockPos::new(i, 0, 0), builtin::STONE))
        .collect();
    let origin = BlockPos::new(2, 0, 0);
    let shape = types::CompoundShape::from_world(origin, &cells).expect("6 格");
    let desc = EntityDesc::compound(DEBRIS_HALF_SIZE, shape);
    let h = physics_sim::settle::envelope_half(&desc);
    let rho = desc.envelope_radius();
    assert!(
        (h * 3f32.sqrt() - rho).abs() < 1e-4,
        "复合体等效 half × √3 = {h}·√3 应还原包络半径 {rho}"
    );
    // 6 格条(half 0.45)的旧口径 0.5 → 0.5√3 ≈ 0.87;新口径 ρ ≈ 3.78:
    // 中心放 chunk 内部(x = 66.0,chunk 4 = x∈[64,80)),旧 reach 覆盖
    // 1 chunk、新 reach 3.78+0 跨到 chunk 3(x 62.2),覆盖 4 chunk。
    let pos = Vec3::new(66.0, -55.0, 66.0);
    let guard = 0.0;
    let new = physics_sim::settle::current_footprint(pos, h, guard);
    let old = physics_sim::settle::current_footprint(pos, DEBRIS_HALF_SIZE, guard);
    assert!(
        new.len() > old.len(),
        "复合体足迹应按包络口径变宽:新 {} chunk vs 旧口径 {} chunk",
        new.len(),
        old.len()
    );
    // 单格:数值逐位不变。
    let single = EntityDesc::default();
    assert_eq!(
        physics_sim::settle::envelope_half(&single),
        single.half_size,
        "单格等效 half = half_size 原样"
    );
    // 调度器侧分片内分支的端到端:复合体瞬态票的 Footprint → Chunks 解析
    // (entity_motion_of → preload_footprint)按包络口径覆盖邻 chunk。
    // 观测面 = plan 实际产出的装载请求(decisions.chunks_to_load):观察者
    // 圆 (锚 x=48.5,r=32 → chunks 1..=5)与已有装载 (1..=5)×(2..=6) 完全
    // 重合 → 观察者票无新请求;复合体中心 x = 93.5(chunk 5,距 5/6 边界
    // 2.5 m):旧口径 reach = 0.5·√3 + guard 1.0 ≈ 1.87 不跨边界(足迹
    // ⊆ 已装载,无请求);新口径 reach = 3.78 + 1.0 = 4.78 跨到 chunk 6
    // → 装载请求必含 (6,3)/(6,4)。
    let mut sim_cfg = physics_sim::ShardSimConfig::new(shard());
    sim_cfg.solidify.enabled = false;
    let mut sim = physics_sim::ShardSim::new(sim_cfg, registry());
    for cx in 1..=5 {
        for cz in 2..=6 {
            let cp = ChunkPos::new(cx, cz);
            sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
        }
    }
    let mut s = Session::new(cfg());
    s.register(sim);
    s.move_observer(Vec3::new(48.5, -55.5, 65.5))
        .expect("观察者");
    warm_until_settled(&mut s, Vec3::new(48.5, -55.5, 65.5));
    let _ = s.tick();
    // 6 格石条:世界格 x 91..97、y −55、z 65;from_world 原点 = 首格 + 2
    // (偏移 = 世界格 − 原点,cells 必须用世界坐标)。
    let cells: Vec<(BlockPos, BlockId)> = (91..97)
        .map(|x| (BlockPos::new(x, -55, 65), builtin::STONE))
        .collect();
    let shape = types::CompoundShape::from_world(BlockPos::new(93, -55, 65), &cells).expect("6 格");
    let desc = EntityDesc::compound(DEBRIS_HALF_SIZE, shape);
    let id = EntityId::from_parts(1, 5);
    s.spawn_owned(
        shard(),
        id,
        EntityState {
            // 原点格心 = (93+0.5, −54.5, 65.5)。
            translation: Vec3::new(93.5, -54.5, 65.5),
            rotation: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
        desc,
        InputSource(7),
    )
    .expect("spawn 复合体");
    // spawn 拍:瞬态运动票签发在拍末 lifecycle(ownership 同步),plan 在
    // 拍首——票义务装载请求自**下一拍**起产出;跑 2 拍累积请求。
    let mut requested: BTreeSet<ChunkPos> = BTreeSet::new();
    for _ in 0..2 {
        let _ = s.tick();
        for (_, decisions) in s.take_activation_requests() {
            requested.extend(decisions.chunks_to_load.iter().copied());
        }
    }
    assert!(
        requested.contains(&ChunkPos::new(6, 3)) && requested.contains(&ChunkPos::new(6, 4)),
        "复合体瞬态票的装载义务应按包络口径覆盖 chunk 6(跨 5/6 边界),\
         实得请求 {requested:?}(旧实现 0.5 口径 reach 1.87 < 2.5 无此请求)"
    );
    s.shutdown();
}

/// 足迹端到端(放行第 4 条后半):大复合体的包络伸出已装载 chunk、盖住未
/// 装载 chunk 时,瞬态运动票的装载义务(footprint 口径)必须把它装齐——
/// 否则待地形恢复(update_awaiting_terrain 用包络口径)永远缺 ②,实体
/// kinematic 挂起直到 1200 拍截止被强制落地(bridge 1024 三块 84 格冻在
/// y=−54.5 的生产现象)。
///
/// 布景:观察者装载圆覆盖 chunks 2..=5;复合体中心 x = 93.5(chunk 5 内,
/// 距 5/6 边界 2.5 m),包络 reach = 3.78 + guard 1.0 = 4.78 → 足迹含
/// chunk 6(观察者圆外)。新实现:票义务装载 chunk 6(① 预算 2/拍)→
/// ② 齐 → 待地形 1–2 拍恢复;旧实现(0.5 口径 reach 1.87):票义务不含
/// chunk 6 → 永缺 → 挂死。
///
/// 变红验证(旧实现):实体在待地形滞留到 1200 拍截止(本测试 100 拍窗口
/// 内 streak 破 30)。
#[test]
fn large_compound_near_load_edge_settles_before_deadline() {
    let mut sim_cfg = physics_sim::ShardSimConfig::new(shard());
    sim_cfg.solidify.enabled = false;
    let mut sim = physics_sim::ShardSim::new(sim_cfg, registry());
    // 已有 ② = (1..=5)×(2..=6):覆盖观察者圆(锚 x=48.5,r=32 → chunks
    // 1..=5 × 2..=6);chunk (6,3)/(6,4) 留给票义务装载(预装载会掩盖 bug)。
    for cx in 1..=5 {
        for cz in 2..=6 {
            let cp = ChunkPos::new(cx, cz);
            sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
        }
    }
    let mut s = Session::new(cfg());
    s.register(sim);
    // 观察者锚 (48.5, −55.5, 65.5):装载圆 32 m → chunks 1..=5(不含 6),
    // 与已有装载重合 → 观察者不产生新请求;包络伸出的 chunk 6 只能来自
    // 实体票义务(足迹口径)。
    s.move_observer(Vec3::new(48.5, -55.5, 65.5))
        .expect("观察者");
    warm_until_settled(&mut s, Vec3::new(48.5, -55.5, 65.5));
    let _ = s.tick();
    // 6 格石条:世界格 x 91..97、y −55、z 65(cells 用世界坐标,from_world
    // 原点 = 首格 + 2)。体心 x = 93.5(chunk 5,距 5/6 边界 2.5 m)。
    let cells: Vec<(BlockPos, BlockId)> = (91..97)
        .map(|x| (BlockPos::new(x, -55, 65), builtin::STONE))
        .collect();
    let shape = types::CompoundShape::from_world(BlockPos::new(93, -55, 65), &cells).expect("6 格");
    let desc = EntityDesc::compound(DEBRIS_HALF_SIZE, shape);
    let id = EntityId::from_parts(1, 7);
    s.spawn_owned(
        shard(),
        id,
        EntityState {
            // 悬空(凝固关闭,恢复醒着后落到地表睡着,不影响待地形断言)。
            translation: Vec3::new(93.5, -54.5, 65.5),
            rotation: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
        desc,
        InputSource(7),
    )
    .expect("spawn 复合体");
    // 100 拍内:待地形滞留 ≤ 30 拍(新实现 ≈ 2–3 拍恢复;旧实现挂满 1200
    // 拍截止,streak 破 30 即红)。每拍后执行生产编排(e2e 内复刻 driver
    // orchestrate 的装载段):取激活决策 → 按申请装载 ①+②(生产
    // `apply_chunk_load` 路径)——票义务装载由这段执行。
    let mut awaiting_streak = 0usize;
    let mut resumed = false;
    for tick in 0..100 {
        let _ = s.tick();
        for (sh, decisions) in s.take_activation_requests() {
            for cp in &decisions.chunks_to_load {
                let chunk = Superflat::classic().generate(WorldSeed(7), *cp);
                s.apply_chunk_load(sh, *cp, chunk);
            }
        }
        let _ = s.take_solidified();
        let _ = s.take_block_updates();
        if s.is_awaiting_terrain(shard(), id) {
            awaiting_streak += 1;
        } else {
            if awaiting_streak > 0 {
                resumed = true;
            }
            awaiting_streak = 0;
        }
        assert!(
            awaiting_streak <= 30,
            "第 {tick} 拍:复合体已连续 {awaiting_streak} 拍待地形\
             (足迹 bug:装载义务未覆盖包络伸出的 chunk)"
        );
    }
    assert!(resumed, "复合体应从待地形恢复(装载义务已把包络 chunk 装齐)");
    s.shutdown();
}
