//! ShardSim 集成测试——REQ-001 验收标准的单分片侧:
//! AC1 确定性(位级)、体素地形支撑、SetBlock 重建、迁移检测(demo A)、
//! 爆炸事件(demo D)、凝固桥(demo C)。

use messaging::{
    CrossEvent, Envelope, EnvelopePayload, InputPayload, InputSource, SimCommand, StampedInput,
};
use physics_sim::{ShardSim, ShardSimConfig};
use types::glam::Vec3;
use types::{
    BlockId, BlockPos, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos,
    Tick, WorldId,
};
use voxel::{Chunk, SimpleRegistry};

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn sim(x: i32, z: i32) -> ShardSim {
    ShardSim::new(
        ShardSimConfig::new(shard_id(x, z)),
        Box::new(SimpleRegistry::builtin()),
    )
}

/// 关闭凝固桥的分片(几何测试用:验证 collider,而非桥行为)。
fn sim_no_solidify(x: i32, z: i32) -> ShardSim {
    let mut cfg = ShardSimConfig::new(shard_id(x, z));
    cfg.solidify.enabled = false;
    // REQ-010 §12.1 第 3 条迁移:悬空 SetBlock 的旧语义用例(悬空放方块后测
    // 实体停在其上),与默认开启的承载判定冲突 → 装配处显式关闭,断言不变。
    cfg.cohesion.enabled = false;
    ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
}

/// 顶层实心面在 world y = top_ly - 64 的地形 chunk。
fn ground_chunk(top_ly: usize) -> Chunk {
    let mut c = Chunk::air();
    for y in 0..top_ly {
        for x in 0..16 {
            for z in 0..16 {
                c.set(x, y, z, BlockId(1));
            }
        }
    }
    c
}

fn entity(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

fn spawn_input(seq: u64, id: EntityId, pos: Vec3, linvel: Vec3, angvel: Vec3) -> StampedInput {
    StampedInput {
        apply_at: Tick(0),
        source: InputSource::SCHEDULER,
        sequence: seq,
        payload: InputPayload::Command(SimCommand::SpawnDynamic {
            entity: id,
            state: EntityState {
                translation: pos,
                linvel,
                angvel,
                ..EntityState::default()
            },
            desc: EntityDesc::default(),
        }),
    }
}

fn input(seq: u64, payload: InputPayload) -> StampedInput {
    StampedInput {
        apply_at: Tick(0),
        source: InputSource::SCHEDULER,
        sequence: seq,
        payload,
    }
}

/// AC1(单分片侧):两个同构实例、相同输入序列 → 位级一致(含地形与爆炸)。
#[test]
fn ac1_determinism_two_instances_bitwise() {
    // 简单场景:虚空下落
    let make = |v: f32| {
        let mut s = sim(0, 0);
        s.apply_prefix(&[spawn_input(
            0,
            entity(1),
            Vec3::new(10.0, 200.0, 10.0),
            Vec3::new(v, 0.0, 0.0),
            Vec3::ZERO,
        )]);
        s
    };
    let (mut a, mut b) = (make(3.5), make(3.5));
    for _ in 0..120 {
        a.step();
        b.step();
    }
    assert_eq!(
        a.entity_state(entity(1)),
        b.entity_state(entity(1)),
        "虚空场景位级一致"
    );

    // 复合场景:地形 + 下落翻滚 + 爆炸事件
    let mut a = sim(0, 0);
    let mut b = sim(0, 0);
    let inputs = vec![
        spawn_input(
            0,
            entity(1),
            Vec3::new(10.0, 150.0, 10.0),
            Vec3::new(0.5, 0.0, 0.0),
            Vec3::new(2.0, 1.0, 0.5),
        ),
        input(
            1,
            InputPayload::Event(CrossEvent::Detonate {
                center: Vec3::new(10.0, 140.0, 10.0),
                power: 4.0,
                radius: 8.0,
            }),
        ),
    ];
    for s in [&mut a, &mut b] {
        s.load_chunk(ChunkPos::new(0, 0), ground_chunk(200));
        s.apply_prefix(&inputs);
    }
    for _ in 0..200 {
        a.step();
        b.step();
    }
    assert_eq!(
        a.entity_state(entity(1)),
        b.entity_state(entity(1)),
        "复合场景位级一致"
    );
    assert_eq!(
        a.take_outbox(),
        b.take_outbox(),
        "产出(凝固/迁移/脏 chunk)也位级一致"
    );
}

/// 体素地形支撑:cube 落在地面顶层 collider 上(rest 高度正确)。
#[test]
fn cube_rests_on_voxel_terrain() {
    let mut s = sim_no_solidify(0, 0);
    s.load_chunk(ChunkPos::new(0, 0), ground_chunk(200)); // 顶面 y = 136
    s.apply_prefix(&[spawn_input(
        0,
        entity(1),
        Vec3::new(10.0, 150.0, 10.0),
        Vec3::ZERO,
        Vec3::ZERO,
    )]);
    for _ in 0..600 {
        s.step();
    }
    let y = s.entity_state(entity(1)).expect("实体仍在").translation.y;
    // 顶面 136 + 半格 0.5(D15 内缩:薄片外表面贴在格面上,不再 +0.075 垫高)
    assert!((y - 136.5).abs() < 0.05, "静止在顶面薄片上:y={y}");
}

/// SetBlock → 该 chunk collider 重建:空中新方块挡住下落路径。
#[test]
fn set_block_rebuilds_chunk_collider() {
    let mut s = sim_no_solidify(0, 0);
    s.load_chunk(ChunkPos::new(0, 0), ground_chunk(200)); // 顶面 136
    s.apply_prefix(&[input(
        0,
        InputPayload::Command(SimCommand::SetBlock {
            pos: BlockPos::new(10, 140, 10),
            block: BlockId(1),
        }),
    )]);
    s.apply_prefix(&[spawn_input(
        1,
        entity(1),
        Vec3::new(10.5, 150.0, 10.5),
        Vec3::ZERO,
        Vec3::ZERO,
    )]);
    for _ in 0..600 {
        s.step();
    }
    let y = s.entity_state(entity(1)).expect("实体仍在").translation.y;
    // 新方块顶面 141 + 半格 0.5 = 141.5(D15 内缩);未重建会落到 136.5
    assert!(
        (y - 141.5).abs() < 0.05,
        "停在新放的方块上:y={y}(未重建则 =136.5)"
    );
}

/// demo A:实体中心离开本体区 → tick 末提取状态产出迁移意图;目的分片可无缝重建。
#[test]
fn entity_migrates_across_boundary() {
    let mut a = sim(0, 0); // core x ∈ [0, 128)
    a.apply_prefix(&[spawn_input(
        0,
        entity(1),
        Vec3::new(100.0, 50.0, 10.0),
        Vec3::new(60.0, 0.0, 0.0),
        Vec3::ZERO,
    )]);
    let mut intent = None;
    for _ in 0..120 {
        a.step();
        let out = a.take_outbox();
        if let Some(i) = out.migrations.into_iter().next() {
            intent = Some(i);
            break;
        }
    }
    let intent = intent.expect("60 m/s 从 x=100 出发,~28 tick 越过 128");
    assert_eq!(
        intent.destination,
        shard_id(1, 0),
        "目的地 = 越界位置所在分片"
    );
    assert!(a.entity_state(entity(1)).is_none(), "源分片已删除");

    let EnvelopePayload::MigrateEntity { entity: id, .. } = intent.payload else {
        panic!("迁移意图载荷必为 MigrateEntity(变体穷举连带,任务 03)");
    };
    let mut b = sim(1, 0);
    b.apply_prefix(&[input(
        1,
        InputPayload::Envelope(Envelope::stamp(a.shard(), intent, Tick(0), 1)),
    )]);
    assert!(b.entity_state(id).is_some(), "目的分片重建成功");
    for _ in 0..10 {
        b.step();
    }
    let x = b.entity_state(id).expect("仍在").translation.x;
    assert!(x > 128.0, "继续沿 +x 运动:x={x}");
}

/// demo D:同一爆炸事件,各实体按本地位置独立求值(对称冲量、线性衰减)。
#[test]
fn detonate_event_applies_symmetric_impulses() {
    let mut s = sim(0, 0);
    s.apply_prefix(&[
        spawn_input(
            0,
            entity(1),
            Vec3::new(10.0, 100.0, 9.0),
            Vec3::ZERO,
            Vec3::ZERO,
        ),
        spawn_input(
            1,
            entity(2),
            Vec3::new(10.0, 100.0, 11.0),
            Vec3::ZERO,
            Vec3::ZERO,
        ),
    ]);
    s.apply_prefix(&[input(
        2,
        InputPayload::Event(CrossEvent::Detonate {
            center: Vec3::new(10.0, 100.0, 10.0),
            power: 6.0,
            radius: 5.0,
        }),
    )]);
    let v1 = s.entity_state(entity(1)).unwrap().linvel.z;
    let v2 = s.entity_state(entity(2)).unwrap().linvel.z;
    // dist=1, radius=5 → 6*(1-0.2)=4.8;质量 1(density 1 × 体积 1)
    assert!((v1 + 4.8).abs() < 1e-4, "v1={v1}(远离爆心方向)");
    assert!((v2 - 4.8).abs() < 1e-4, "v2={v2}");
    assert!((v1 + v2).abs() < 1e-4, "对称");
}

/// demo C:凝固桥——翻滚落地的 cube 在低速触发后 snap 进体素数组,
/// collider 重建,后续实体能停在新凝固的方块上。
#[test]
fn solidify_bridge_freezes_cube_into_voxels() {
    let mut s = sim(0, 0);
    s.load_chunk(ChunkPos::new(0, 0), ground_chunk(200)); // 顶面 136
    s.apply_prefix(&[spawn_input(
        0,
        entity(1),
        Vec3::new(10.0, 145.0, 10.0),
        Vec3::new(0.5, 0.0, 0.0),
        Vec3::new(2.0, 1.0, 0.5),
    )]);
    let mut cell = None;
    for _ in 0..1500 {
        s.step();
        let out = s.take_outbox();
        // 任务 03 适配:凝固写入迁 `block_updates` 的 `Emergent`(字段变,意图不变)。
        if let Some(w) = out
            .block_updates
            .iter()
            .find(|w| w.kind == physics_sim::BlockWriteKind::Emergent)
        {
            cell = Some(w.pos);
            break;
        }
    }
    let cell = cell.expect("1500 tick 内凝固(P2 + 睡眠兜底)");
    assert_eq!(s.voxel().get_block(cell), BlockId(1), "体素数组已写入");
    assert!(s.entity_state(entity(1)).is_none(), "刚体已删除");
    assert!(
        (136..=140).contains(&cell.y),
        "落在地面顶层上方一两格:cell={cell:?}"
    );

    // 凝固块有真实碰撞:小方块落到它上面
    s.apply_prefix(&[input(
        1,
        InputPayload::Command(SimCommand::SpawnDynamic {
            entity: entity(2),
            state: EntityState {
                translation: Vec3::new(
                    cell.x as f32 + 0.5,
                    cell.y as f32 + 6.0,
                    cell.z as f32 + 0.5,
                ),
                ..EntityState::default()
            },
            desc: EntityDesc {
                half_size: 0.3,
                block: BlockId(1),
                shape: None,
            },
        }),
    )]);
    let mut last_y = f32::NAN;
    let mut settled = false;
    for _ in 0..300 {
        s.step();
        let out = s.take_outbox();
        // 任务 03 适配:凝固检测读 `block_updates` 的 `Emergent`(同上)。
        if out
            .block_updates
            .iter()
            .any(|w| w.kind == physics_sim::BlockWriteKind::Emergent)
        {
            settled = true; // 小方块自己也凝固了,用凝固前最后位置
            break;
        }
        if let Some(st) = s.entity_state(entity(2)) {
            last_y = st.translation.y;
        }
    }
    assert!(
        settled || (last_y - (cell.y as f32 + 1.3)).abs() < 0.3,
        "小方块停在凝固块上方:y={last_y},cell.y={}",
        cell.y
    );
}

// ---- REQ-010 单元 03a:复合实体形状与刚体(§5.4)----

use types::CompoundShape;

/// 2×1×1 双格石梁:原点在两格中间的格上,偏移 [0,0,0] 与 [1,0,0]。
fn two_cell_beam_desc() -> (BlockPos, EntityDesc) {
    let origin = BlockPos::new(8, 190, 8);
    let cells = vec![
        (BlockPos::new(8, 190, 8), BlockId(1)),
        (BlockPos::new(9, 190, 8), BlockId(1)),
    ];
    let shape = CompoundShape::from_world(origin, &cells).expect("2 格");
    (origin, EntityDesc::compound(0.45, shape))
}

fn spawn_desc(seq: u64, id: EntityId, state: EntityState, desc: EntityDesc) -> StampedInput {
    StampedInput {
        apply_at: Tick(0),
        source: InputSource::SCHEDULER,
        sequence: seq,
        payload: InputPayload::Command(SimCommand::SpawnDynamic {
            entity: id,
            state,
            desc,
        }),
    }
}

/// §5.4 第 3 条:复合体一个动态刚体、每格一个 cuboid;质量 = 格数 × 0.729
/// (0.9³ × 密度 1.0);单格 = 0.729。刚体平移 = 原点(刚体平移语义与单格同)。
///
/// 变红:复合体只建一个 collider 且密度按体积放大(质量 ≠ 格数 × 0.729);
/// 或忘了给复合体建刚体(spawn panic)。
#[test]
fn compound_mass_is_cells_times_unit_mass() {
    let mut s = sim_no_solidify(0, 0);
    s.load_chunk(ChunkPos::new(0, 0), ground_chunk(200));
    let id = entity(30);
    s.apply_prefix(&[spawn_desc(
        0,
        id,
        EntityState {
            translation: Vec3::new(8.5, 191.5, 8.5),
            ..EntityState::default()
        },
        two_cell_beam_desc().1,
    )]);
    // 单格参照(默认描述,half = 0.5):质量 = 1.0(1 m³ × 密度 1)。
    let mut ref_sim = sim_no_solidify(0, 0);
    ref_sim.load_chunk(ChunkPos::new(0, 0), ground_chunk(200));
    ref_sim.apply_prefix(&[spawn_input(
        0,
        entity(31),
        Vec3::new(8.5, 191.5, 8.5),
        Vec3::ZERO,
        Vec3::ZERO,
    )]);
    let single = ref_sim.entity_mass(entity(31)).expect("单格在场");
    assert!(
        (single - 1.0).abs() < 1e-6,
        "半格立方单格质量 1.0,实为 {single}"
    );
    let compound = s.entity_mass(id).expect("复合体在场");
    // 复合体每格 0.45 cuboid → 0.9³ × 密度 1 = 0.729/格(§5.4 第 3 条)。
    let want = 2.0 * physics_sim::destruction::DEBRIS_MASS;
    assert!(
        (compound - want).abs() < 1e-5,
        "复合体质量 = 格数 × 0.729,实为 {compound}(期望 {want})"
    );
    // spawn 后刚体平移 = 原点(传入的 translation;§5.4 第 3 条)。
    assert_eq!(
        s.entity_state(id).expect("在场").translation,
        Vec3::new(8.5, 191.5, 8.5)
    );
}

/// §5.4 第 6 条:迁移信封携带完整描述(含形状);目的分片重建为同一复合体
/// (质量不变 = 形状没丢),源分片不再持有。
///
/// 变红:迁移信封丢形状(目的分片重建成单格,质量变 0.729)。
#[test]
fn compound_migration_carries_shape() {
    let mut s = sim_no_solidify(0, 0);
    s.load_chunk(ChunkPos::new(0, 0), ground_chunk(200));
    let id = entity(32);
    s.apply_prefix(&[spawn_desc(
        0,
        id,
        EntityState {
            translation: Vec3::new(8.5, 191.5, 8.5),
            ..EntityState::default()
        },
        two_cell_beam_desc().1,
    )]);
    // 传送到本体区外(x = 136 > 128)触发迁移。
    s.apply_prefix(&[input(
        1,
        InputPayload::Command(SimCommand::TeleportEntity {
            entity: id,
            state: EntityState {
                translation: Vec3::new(136.5, 191.5, 8.5),
                ..EntityState::default()
            },
        }),
    )]);
    s.step();
    let out = s.take_outbox();
    assert_eq!(out.migrations.len(), 1, "迁移信封已产出");
    let desc = match &out.migrations[0].payload {
        EnvelopePayload::MigrateEntity { desc, .. } => desc.clone(),
        other => panic!("期望 MigrateEntity,实得 {other:?}"),
    };
    assert!(
        desc.shape().is_some(),
        "迁移信封的描述必须带形状(§5.4 第 6 条)"
    );
    // 形状逐格都在(canonical 2 格):包络半径 = 最远偏移 1 + half·√3。
    let want_rho = 1.0f32 + 0.45f32 * 3f32.sqrt();
    assert!((desc.envelope_radius() - want_rho).abs() < 1e-5);
    assert_eq!(desc.shape().expect("形状").cells().len(), 2, "两格都在");
}

/// §5.4 第 4 条分叉点:复合体冻结时的地形变更唤醒按 ρ 包络(等效 half =
/// ρ/√3)判定,而不是 half_size。写一个"离原点 1.9 m"的格:单格包络
/// (0.45·√3 ≈ 0.78 m + skin)够不到,复合体包络(ρ = 1 + 0.45√3 ≈ 1.78 m)
/// 必须够到——冻结的复合体被唤醒(切回 Dynamic)。
///
/// 变红:分叉点漏改(复合体按 half_size 判定,包络外的写入点永远不唤醒它)。
#[test]
fn compound_envelope_half_covers_shape_extent() {
    let mut cfg = ShardSimConfig::new(shard_id(0, 0));
    cfg.activation.data_load_radius = 96.0;
    cfg.activation.collider_load_radius = 96.0;
    let mut s = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
    s.load_chunk(ChunkPos::new(0, 0), ground_chunk(255));
    let id = entity(35);
    // 复合体:2 格梁,原点在 (8,190,8),中心 = (8.5,190.5,8.5)。
    s.apply_prefix(&[spawn_desc(
        0,
        id,
        EntityState {
            translation: Vec3::new(8.5, 191.5, 8.5),
            ..EntityState::default()
        },
        two_cell_beam_desc().1,
    )]);
    // 观察者放远(96 m 之外):排除 plan 的观察者唤醒,只测地形变更唤醒。
    let far = Vec3::new(127.5, 190.5, 8.5);
    s.apply_activation(&[far]);
    s.freeze_entity_for_test(id);
    assert!(!s.entity_awake(id), "前置:实体已冻结");
    // 写入格 (10,190,8):其中心 (10.5,·,8.5) 距原点中心 (8.5,·,8.5) dx = 2.0。
    // 复合体最右格覆盖到 x = 9.95;包络 AABB 沿 x 半高 = ρ ≈ 1.778,
    // 原点中心 8.5 + 1.778 + skin(0.075) = 10.353 > 10.0 → 相交。
    // 单格包络:8.5 + 0.779 + 0.075 = 9.354 < 10.0 → 不相交(分叉失效时)。
    s.write_voxel_for_test(BlockPos::new(10, 190, 8), BlockId(1))
        .expect("写入");
    s.apply_activation(&[far]);
    assert!(
        s.entity_awake(id),
        "复合体按 ρ 包络被地形写入唤醒(分叉点生效)"
    );
}
