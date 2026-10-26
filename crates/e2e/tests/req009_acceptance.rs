//! REQ-009 验收用例(总架构师编写;issue #18)。
//!
//! 约定同 REQ-007/REQ-008:实施方**不得修改断言**,实现完成后只去掉对应单元用例上的
//! `#[ignore]`;认为断言与需求不符时在 #18 提出,由总架构师裁定。每个用例注释写明
//! "在什么错误实现下会变红"(纪律 12)。viewer 脚本相关用例在
//! `crates/viewer/tests/req009_acceptance.rs`。

mod unit01 {
    use app::config::{Config, WorldKind};
    use types::{BlockId, ChunkPos};
    use voxel::{BlockRegistry, Material, SimpleRegistry, SimpleRegistryError, Support, builtin};

    fn mat(resistance: f32, impact_threshold: f32, support: Support) -> Material {
        Material {
            resistance,
            impact_threshold,
            support,
            pulverize_factor: 2.0,
        }
    }

    /// AC-D1:内置方块表追加 bedrock(3)、sand(4);五种方块的名字、实心与材料数值与 §3.2 一致;
    /// 空气、未知 ID 没有材料。
    ///
    /// 会变红的实现:ID 顺序不对(破坏存档兼容);数值抄错或 k 不是 2;基岩不是 `Anchor`;
    /// 沙不是 `NeedsBelow`;空气返回了材料。
    #[test]
    fn acc_d1_builtin_materials() {
        let reg = SimpleRegistry::builtin();
        assert_eq!(reg.block_count(), 5);
        let table = [
            (builtin::AIR, "air", false, None),
            (
                builtin::STONE,
                "stone",
                true,
                Some(mat(6.0, 18.0, Support::Connected)),
            ),
            (
                builtin::DIRT,
                "dirt",
                true,
                Some(mat(2.0, 7.0, Support::Connected)),
            ),
            (
                builtin::BEDROCK,
                "bedrock",
                true,
                Some(mat(f32::INFINITY, f32::INFINITY, Support::Anchor)),
            ),
            (
                builtin::SAND,
                "sand",
                true,
                Some(mat(2.0, 4.0, Support::NeedsBelow)),
            ),
        ];
        for (id, name, solid, material) in table {
            assert_eq!(reg.id_of(name), Some(id), "{name} 的 ID");
            assert_eq!(reg.name_of(id), Some(name));
            assert_eq!(reg.is_solid(id), solid, "{name} 是否实心");
            assert_eq!(reg.material(id), material, "{name} 的材料");
        }
        assert_eq!(builtin::BEDROCK, BlockId(3));
        assert_eq!(builtin::SAND, BlockId(4));
        assert_eq!(reg.material(BlockId(99)), None, "未知 ID 没有材料");
    }

    /// AC-D1:`register` 注册的实心方块默认不可破坏(测试用的自定义方块表行为不变);非实心没有材料;
    /// `register_with_material` 注册的方块是实心、材料原样返回,重名同样报错且不占 ID。
    ///
    /// 会变红的实现:`register` 的实心方块给了可破坏的材料(既有用例会突然开始破坏地形);
    /// `register_with_material` 注册成非实心;重名时占用了 ID。
    #[test]
    fn acc_d1_register_defaults_and_custom_material() {
        let mut reg = SimpleRegistry::new();
        let x = reg.register("x", true).expect("注册");
        let y = reg.register("y", false).expect("注册");
        assert_eq!(reg.material(x), Some(Material::INDESTRUCTIBLE));
        assert_eq!(
            Material::INDESTRUCTIBLE,
            mat(f32::INFINITY, f32::INFINITY, Support::Connected)
        );
        assert_eq!(reg.material(y), None, "非实心没有材料");

        let glass = mat(1.0, 3.0, Support::Connected);
        let z = reg.register_with_material("glass", glass).expect("注册");
        assert_eq!(z, BlockId(3));
        assert!(reg.is_solid(z), "带材料注册的方块是实心");
        assert_eq!(reg.material(z), Some(glass));
        assert_eq!(
            reg.register_with_material("glass", glass),
            Err(SimpleRegistryError::DuplicateName("glass".into()))
        );
        assert_eq!(reg.register("w", true), Ok(BlockId(4)), "重名失败不占 ID");
    }

    /// AC-D1 + D6(生产路径):两种世界类型经 `wire_provider_and_new_sim` 生成的 chunk,最底层
    /// (局部 y = 0,世界 y = −64)全部是 bedrock,其上三层 dirt,再上是空气;地表高度不变。
    ///
    /// 会变红的实现:只改了 `Superflat::classic` 没改散布世界的基底;把 bedrock 叠在原有
    /// 四层之上(地表抬高一格,既有轨迹基线全部变化);只改了生成器单测用的表,生产装配没换。
    #[test]
    fn acc_d1_world_bottom_is_bedrock() {
        for kind in [WorldKind::Superflat, WorldKind::SeededScatter] {
            let dir = tempfile::tempdir().expect("临时目录");
            let config = Config {
                data_dir: dir.path().to_path_buf(),
                world_kind: kind,
                ..Config::default()
            };
            let (session, _log, store, _resume, _seq) = app::driver::bootstrap(&config, || {
                Box::new(SimpleRegistry::builtin()) as Box<dyn BlockRegistry>
            })
            .expect("装配");
            let (provider, _new_sim) =
                app::driver::wire_provider_and_new_sim(config.clone(), &store, || {
                    Box::new(SimpleRegistry::builtin()) as Box<dyn BlockRegistry>
                });
            for pos in [ChunkPos::new(0, 0), ChunkPos::new(-3, 7)] {
                let chunk = provider.generate(pos);
                for x in 0..16 {
                    for z in 0..16 {
                        assert_eq!(
                            chunk.get(x, 0, z),
                            builtin::BEDROCK,
                            "{kind:?} {pos:?} ({x},{z}) 最底层"
                        );
                        for y in 1..=3 {
                            assert_eq!(chunk.get(x, y, z), builtin::DIRT, "{kind:?} 第 {y} 层");
                        }
                        if kind == WorldKind::Superflat {
                            assert_eq!(chunk.get(x, 4, z), builtin::AIR, "地表之上是空气");
                        }
                    }
                }
            }
            drop(provider);
            session.shutdown();
            let _ = store.shutdown();
        }
    }
}

/// 单元 02:爆炸求值与粉碎(AC-D2、AC-D3、checkpoint v6)。
///
/// 标准答案来自总架构师按 REQ-009 §4.2–§4.3 写的参照实现(与 demo `crater.py` 逐数一致:
/// 平地破坏 60 = 粉碎 36 + 脱落 24;1 格墙、3 格墙都只掉朝爆心的 8 格,墙后 0 格)。
/// 坐标:生产超平坦,y = −64 bedrock、−63…−61 dirt,地表 y = −60;爆心 (64, −59.5, 64)。
mod unit02 {
    use messaging::{CrossEvent, InputPayload, InputSource, SimCommand, StampedInput};
    use physics_sim::destruction::{
        BlastCell, CellFate, DestructionConfig, MAX_TERRAIN_BLAST_RADIUS, debris_direction,
        evaluate_blast, segment_cells,
    };
    use physics_sim::{BlockWriteKind, ShardSim, ShardSimConfig};
    use std::collections::BTreeSet;
    use types::glam::Vec3;
    use types::{
        BlockId, BlockPos, ChunkPos, DimensionId, ShardId, ShardPos, Tick, WorldId, WorldSeed,
    };
    use voxel::{SimpleRegistry, Superflat, WorldGenerator, builtin};

    const CENTER: Vec3 = Vec3::new(64.0, -59.5, 64.0);

    fn flat(pos: BlockPos) -> Option<BlockId> {
        Some(match pos.y {
            -64 => builtin::BEDROCK,
            -63..=-61 => builtin::DIRT,
            _ => builtin::AIR,
        })
    }

    /// 平地 + 一堵石墙:x ∈ [67, 67+t),z ∈ [57, 71],y ∈ [−60, −55]。
    fn with_wall(t: i32) -> impl Fn(BlockPos) -> Option<BlockId> {
        move |p: BlockPos| {
            if (67..67 + t).contains(&p.x) && (57..=71).contains(&p.z) && (-60..=-55).contains(&p.y)
            {
                Some(builtin::STONE)
            } else {
                flat(p)
            }
        }
    }

    fn blast(
        world: &dyn Fn(BlockPos) -> Option<BlockId>,
        center: Vec3,
        power: f32,
        radius: f32,
        max_debris: usize,
    ) -> Vec<BlastCell> {
        evaluate_blast(
            world,
            &SimpleRegistry::builtin(),
            &|_| true,
            center,
            power,
            radius,
            1.0,
            max_debris,
        )
    }

    /// (破坏数, 粉碎数, 脱落数)。
    fn counts(cells: &[BlastCell]) -> (usize, usize, usize) {
        let pul = cells
            .iter()
            .filter(|c| c.fate == CellFate::Pulverize)
            .count();
        (cells.len(), pul, cells.len() - pul)
    }

    fn find(cells: &[BlastCell], x: i32, y: i32, z: i32) -> BlastCell {
        *cells
            .iter()
            .find(|c| c.pos == BlockPos::new(x, y, z))
            .unwrap_or_else(|| panic!("({x},{y},{z}) 应被破坏"))
    }

    fn assert_vec(got: Vec3, want: [f32; 3], what: &str) {
        assert!(
            (got - Vec3::from(want)).length() < 1e-4,
            "{what}:得到 {got:?},应为 {want:?}"
        );
    }

    /// 体素遍历规则(§4.2):含两端所在格;同时跨多个面时按 x、y、z 顺序逐轴步进;
    /// 负方向同理;某轴分量为 0 不步进。
    ///
    /// 会变红的实现:漏掉起点或终点;对角穿角时跳过中间格或按 z、y、x 顺序步进;
    /// 负方向边界取错(差一格)。
    #[test]
    fn acc_d2_segment_cells_rule() {
        let b = BlockPos::new;
        assert_eq!(
            segment_cells([0.5, 0.5, 0.5], [2.5, 0.5, 0.5]),
            [b(0, 0, 0), b(1, 0, 0), b(2, 0, 0)]
        );
        assert_eq!(
            segment_cells([0.5, 0.5, 0.5], [1.5, 1.5, 0.5]),
            [b(0, 0, 0), b(1, 0, 0), b(1, 1, 0)],
            "穿角:先 x 后 y"
        );
        assert_eq!(
            segment_cells([0.5, 0.5, 0.5], [-0.5, -0.5, -0.5]),
            [b(0, 0, 0), b(-1, 0, 0), b(-1, -1, 0), b(-1, -1, -1)],
            "负方向穿角:x、y、z 依次"
        );
        assert_eq!(
            segment_cells([3.2, 4.7, 5.1], [3.9, 4.1, 5.8]),
            [b(3, 4, 5)]
        );
    }

    /// AC-D2 平地弹坑:破坏 60 = 粉碎 36 + 脱落 24;基岩不坏;只到 y −61…−63;按 BlockPos 升序;
    /// 抽样格的压力、去向与冲量逐数吻合;碎块方向以向上为主。
    ///
    /// 会变红的实现:用球形模型(破坏 224);k 用 3;距离量到格角而不是格中心;射线把目标格自身算进
    /// 衰减;冲量用 `P − R` 或用爆心方向;返回顺序不是升序。
    #[test]
    fn acc_d2_flat_crater_matches_reference() {
        let cells = blast(&flat, CENTER, 12.0, 8.0, 48);
        assert_eq!(counts(&cells), (60, 36, 24));
        assert!(
            cells.windows(2).all(|w| w[0].pos < w[1].pos),
            "BlockPos 升序"
        );
        let ys: BTreeSet<i32> = cells.iter().map(|c| c.pos.y).collect();
        assert_eq!(ys, BTreeSet::from([-63, -62, -61]), "基岩(−64)不坏");
        assert!(cells.iter().all(|c| c.block == builtin::DIRT));

        let top = find(&cells, 64, -61, 64);
        assert_eq!(top.fate, CellFate::Pulverize);
        assert!((top.pressure - 10.162883).abs() < 1e-5, "{}", top.pressure);
        let deep = find(&cells, 64, -63, 64);
        assert!((deep.pressure - 3.376689).abs() < 1e-5, "{}", deep.pressure);
        assert!(matches!(deep.fate, CellFate::Detach { .. }));
        match find(&cells, 61, -61, 62).fate {
            CellFate::Detach { impulse } => assert_vec(
                impulse,
                [-1.024652, 3.158188, -0.614791],
                "(61,−61,62) 冲量",
            ),
            other => panic!("(61,−61,62) 应脱落,实为 {other:?}"),
        }
        let weakest = cells
            .iter()
            .filter(|c| c.fate != CellFate::Pulverize)
            .map(|c| c.pressure)
            .fold(f64::INFINITY, f64::min);
        assert!((weakest - 2.488648).abs() < 1e-5, "{weakest}");
        let ties: Vec<BlockPos> = cells
            .iter()
            .filter(|c| c.fate != CellFate::Pulverize && c.pressure == weakest)
            .map(|c| c.pos)
            .collect();
        let b = BlockPos::new;
        assert_eq!(
            ties,
            [
                b(60, -61, 63),
                b(60, -61, 64),
                b(61, -61, 61),
                b(61, -61, 66),
                b(63, -61, 60),
                b(63, -61, 67),
                b(64, -61, 60),
                b(64, -61, 67),
                b(66, -61, 61),
                b(66, -61, 66),
                b(67, -61, 63),
                b(67, -61, 64),
            ],
            "压力最低的 12 格逐位相等(坑缘对称位置),按 BlockPos 升序"
        );

        assert_vec(
            debris_direction(Vec3::new(0.5, 5.0, 0.5), BlockPos::new(0, 0, 0)),
            [0.0, 1.0, 0.0],
            "正下方的格:竖直向上",
        );
    }

    /// AC-D2 墙的遮挡:1 格墙与 3 格墙都只掉朝爆心一面的 8 格,墙后 0 格;总数 68 = 粉碎 36 + 脱落 32。
    ///
    /// 会变红的实现:不做射线衰减(墙后被炸);衰减把整面墙挡死(墙面也不掉);石头强度不是 6。
    #[test]
    fn acc_d2_walls_shield() {
        for t in [1, 3] {
            let world = with_wall(t);
            let cells = blast(&world, CENTER, 12.0, 8.0, 48);
            let wall: Vec<&BlastCell> = cells
                .iter()
                .filter(|c| {
                    (67..67 + t).contains(&c.pos.x)
                        && (57..=71).contains(&c.pos.z)
                        && (-60..=-55).contains(&c.pos.y)
                })
                .collect();
            assert_eq!(wall.len(), 8, "厚 {t}:墙上破坏 8 格");
            assert!(
                wall.iter()
                    .all(|c| c.pos.x == 67 && c.block == builtin::STONE),
                "只掉朝爆心一面"
            );
            assert_eq!(
                cells.iter().filter(|c| c.pos.x >= 67 + t).count(),
                0,
                "厚 {t}:墙后 0 格"
            );
            assert_eq!(counts(&cells), (68, 36, 32), "厚 {t} 总数");
        }
    }

    /// AC-D2 脱落上限:上限 10 时保留压力最大的 10 个(同压力按 BlockPos 升序),其余改为粉碎。
    ///
    /// 会变红的实现:截断按 BlockPos 而不是压力;多出来的直接丢掉而不是粉碎(坑变小);同压力取后者。
    #[test]
    fn acc_d2_debris_cap() {
        let full = blast(&flat, CENTER, 12.0, 8.0, 48);
        let capped = blast(&flat, CENTER, 12.0, 8.0, 10);
        assert_eq!(counts(&capped), (60, 50, 10));
        let mut ranked: Vec<(f64, BlockPos)> = full
            .iter()
            .filter(|c| c.fate != CellFate::Pulverize)
            .map(|c| (c.pressure, c.pos))
            .collect();
        ranked.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        let want: BTreeSet<BlockPos> = ranked[..10].iter().map(|r| r.1).collect();
        let got: BTreeSet<BlockPos> = capped
            .iter()
            .filter(|c| c.fate != CellFate::Pulverize)
            .map(|c| c.pos)
            .collect();
        assert_eq!(got, want);
    }

    /// AC-D2 求值范围、未装载与半径上限:按 x = 128 切开,两边各 30 格,并集等于整体、无重叠;
    /// 未装载的墙既不求值也不衰减(结果与无墙相同);半径 40 按 16 计算(与半径 16 完全相同)。
    ///
    /// 会变红的实现:`in_scope` 之外的格也返回(跨片重复);把未装载当成实心(墙后被挡);半径不封顶。
    #[test]
    fn acc_d2_scope_unloaded_and_radius_cap() {
        let reg = SimpleRegistry::builtin();
        let edge = Vec3::new(128.0, -59.5, 64.0);
        let run = |scope: &dyn Fn(BlockPos) -> bool| {
            evaluate_blast(&flat, &reg, scope, edge, 12.0, 8.0, 1.0, 48)
        };
        let all = run(&|_| true);
        let left = run(&|p: BlockPos| p.x < 128);
        let right = run(&|p: BlockPos| p.x >= 128);
        assert_eq!(counts(&all), (60, 36, 24));
        assert_eq!((left.len(), right.len()), (30, 30));
        let l: BTreeSet<BlockPos> = left.iter().map(|c| c.pos).collect();
        let r: BTreeSet<BlockPos> = right.iter().map(|c| c.pos).collect();
        assert!(l.is_disjoint(&r));
        assert_eq!(
            l.union(&r).copied().collect::<BTreeSet<_>>(),
            all.iter().map(|c| c.pos).collect::<BTreeSet<_>>()
        );

        let unloaded_wall = |p: BlockPos| {
            if p.x == 67 && (57..=71).contains(&p.z) && (-60..=-55).contains(&p.y) {
                None
            } else {
                flat(p)
            }
        };
        assert_eq!(
            blast(&unloaded_wall, CENTER, 12.0, 8.0, 48),
            blast(&flat, CENTER, 12.0, 8.0, 48)
        );

        let big = blast(&flat, CENTER, 200.0, 40.0, 48);
        assert_eq!(
            big,
            blast(&flat, CENTER, 200.0, MAX_TERRAIN_BLAST_RADIUS, 48)
        );
        assert_eq!(counts(&big), (1752, 1704, 48));
    }

    fn shard_id(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    fn sim(x: i32, z: i32, enabled: bool, chunks: &[(i32, i32)]) -> ShardSim {
        let mut cfg = ShardSimConfig::new(shard_id(x, z));
        cfg.destruction = DestructionConfig {
            enabled,
            ..DestructionConfig::default()
        };
        let mut sim = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
        for &(cx, cz) in chunks {
            let cp = ChunkPos::new(cx, cz);
            sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
        }
        sim
    }

    fn apply(sim: &mut ShardSim, payload: InputPayload) {
        sim.apply_prefix(&[StampedInput {
            apply_at: Tick(0),
            source: InputSource::SCHEDULER,
            sequence: 0,
            payload,
        }]);
    }

    fn square(c0: i32, c1: i32, z0: i32, z1: i32) -> Vec<(i32, i32)> {
        (c0..=c1)
            .flat_map(|x| (z0..=z1).map(move |z| (x, z)))
            .collect()
    }

    fn detonate_command(center: Vec3) -> InputPayload {
        InputPayload::Command(SimCommand::Detonate {
            center,
            power: 12.0,
            radius: 8.0,
        })
    }

    /// AC-D3(生产路径):开启破坏的 ShardSim 收到 `Detonate` 命令后,与纯函数结果完全一致——
    /// 体素写为 air、每格一条 `Emergent` 写入、脱落请求(格、方块、冲量)按 BlockPos 升序、所在 chunk
    /// 重建 collider;关闭时地形不变、不产生任何写入。
    ///
    /// 会变红的实现:没接进 `detonate`;写入标成 `External`(副本不同步);脱落格没写 air;脱落请求的
    /// 冲量与纯函数不同;开关不起作用。
    #[test]
    fn acc_d3_shard_sim_applies_blast() {
        let want = blast(&flat, CENTER, 12.0, 8.0, 48);
        let mut s = sim(0, 0, true, &square(2, 6, 2, 6));
        apply(&mut s, detonate_command(CENTER));
        let out = s.take_outbox();
        for c in &want {
            assert_eq!(
                s.voxel().get_block(c.pos),
                builtin::AIR,
                "{:?} 写为 air",
                c.pos
            );
        }
        assert_eq!(out.block_updates.len(), 60, "每格一条写入");
        for w in &out.block_updates {
            assert_eq!((w.block, w.kind), (builtin::AIR, BlockWriteKind::Emergent));
        }
        let writes: BTreeSet<BlockPos> = out.block_updates.iter().map(|w| w.pos).collect();
        assert_eq!(writes, want.iter().map(|c| c.pos).collect::<BTreeSet<_>>());
        let detach: Vec<(BlockPos, BlockId, Vec3)> = want
            .iter()
            .filter_map(|c| match c.fate {
                CellFate::Detach { impulse } => Some((c.pos, c.block, impulse)),
                CellFate::Pulverize => None,
            })
            .collect();
        assert_eq!(out.detach_requests.len(), detach.len());
        for (req, (pos, block, impulse)) in out.detach_requests.iter().zip(&detach) {
            assert_eq!((req.cell, req.block), (*pos, *block));
            assert!((req.impulse - *impulse).length() < 1e-5);
        }
        assert!(
            out.dirty_chunks.contains(&ChunkPos::new(4, 4)),
            "爆心所在 chunk 重建 collider"
        );
        assert_eq!(
            s.voxel().get_block(BlockPos::new(64, -64, 64)),
            builtin::BEDROCK
        );

        let mut off = sim(0, 0, false, &square(2, 6, 2, 6));
        apply(&mut off, detonate_command(CENTER));
        let out = off.take_outbox();
        assert!(
            out.block_updates.is_empty() && out.detach_requests.is_empty(),
            "关闭时不破坏"
        );
        assert_eq!(
            off.voxel().get_block(BlockPos::new(64, -61, 64)),
            builtin::DIRT
        );
    }

    /// AC-D3 跨分片:爆心压在 x = 128 边界,两个分片收到同一个 `Detonate` 事件,各自只写本体区内的格
    /// (各 30),并集等于纯函数的整体结果。
    ///
    /// 会变红的实现:分片把扩张区副本格也写了(两边各 60、重复写);把副本格当成未装载
    /// (射线不衰减,结果与整体不同)。
    #[test]
    fn acc_d3_two_shards_each_write_own_core() {
        let edge = Vec3::new(128.0, -59.5, 64.0);
        let event = InputPayload::Event(CrossEvent::Detonate {
            center: edge,
            power: 12.0,
            radius: 8.0,
        });
        let chunks = square(6, 9, 2, 5);
        let mut a = sim(0, 0, true, &chunks);
        let mut b = sim(1, 0, true, &chunks);
        apply(&mut a, event.clone());
        apply(&mut b, event);
        let wa: BTreeSet<BlockPos> = a
            .take_outbox()
            .block_updates
            .iter()
            .map(|w| w.pos)
            .collect();
        let wb: BTreeSet<BlockPos> = b
            .take_outbox()
            .block_updates
            .iter()
            .map(|w| w.pos)
            .collect();
        assert!(wa.iter().all(|p| p.x < 128) && wb.iter().all(|p| p.x >= 128));
        assert_eq!((wa.len(), wb.len()), (30, 30));
        let all: BTreeSet<BlockPos> = blast(&flat, edge, 12.0, 8.0, 48)
            .iter()
            .map(|c| c.pos)
            .collect();
        assert_eq!(wa.union(&wb).copied().collect::<BTreeSet<_>>(), all);
    }

    /// checkpoint 格式升到 v6(ShardSimConfig 追加 `destruction`,快照布局变化;REQ-005 D8 版本记录)。
    ///
    /// 会变红的实现:布局变了却没升版本(旧档按 v5 解码会读出错位数据或报非版本错误)。
    #[test]
    fn acc_d10_checkpoint_version_is_v6() {
        assert_eq!(app::driver::CHECKPOINT_FORMAT_VERSION, 6);
    }
}

/// 单元 03:脱落实体的生成——调度器分配涌现 ID、N+1 生成、每拍上限(AC-D4、AC-D7)。
///
/// 经 `Session`(生产会话接口)驱动;坐标与单元 02 相同(平地标准爆炸:脱落 24 格)。
mod unit03 {
    use physics_sim::destruction::{
        CellFate, DEBRIS_HALF_SIZE, DEBRIS_MASS, DestructionConfig, DetachRequest, detach_spawn,
        evaluate_blast,
    };
    use physics_sim::{ShardSim, ShardSimConfig};
    use shard::SchedulerConfig;
    use sim_api::log::CommandLog;
    use sim_api::session::{Session, SessionConfig};
    use types::glam::{Quat, Vec3};
    use types::{
        BlockId, BlockPos, ChunkPos, DimensionId, EMERGENT_ORIGIN, EntityDesc, EntityId, ShardId,
        ShardPos, WORLD_Y_MIN, WorldId, WorldSeed,
    };
    use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

    const CENTER: Vec3 = Vec3::new(64.0, -59.5, 64.0);
    const FAR: Vec3 = Vec3::new(64.0, -59.5, 96.0);

    fn flat(pos: BlockPos) -> Option<BlockId> {
        Some(match pos.y {
            -64 => builtin::BEDROCK,
            -63..=-61 => builtin::DIRT,
            _ => builtin::AIR,
        })
    }

    /// 纯函数口径下某次爆炸的脱落格(BlockPos 升序)。
    fn detached_cells(center: Vec3, max_debris: usize) -> Vec<BlockPos> {
        evaluate_blast(
            &flat,
            &SimpleRegistry::builtin(),
            &|_| true,
            center,
            12.0,
            8.0,
            1.0,
            max_debris,
        )
        .into_iter()
        .filter(|c| c.fate != CellFate::Pulverize)
        .map(|c| c.pos)
        .collect()
    }

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

    /// 单分片会话:平地 chunk (2..=6, 2..=6) 已装载,破坏开启,每拍生成上限 `cap`。
    fn session(cap: usize) -> Session {
        let mut sim_cfg = ShardSimConfig::new(shard());
        sim_cfg.destruction = DestructionConfig {
            enabled: true,
            max_detach_per_tick: cap,
            ..DestructionConfig::default()
        };
        let mut sim = ShardSim::new(sim_cfg, registry());
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

    fn detonate(s: &mut Session, log: &mut CommandLog, center: Vec3) {
        for entry in s.detonate(center, 12.0, 8.0).expect("爆炸命令") {
            log.append(entry);
        }
    }

    /// 本分片的涌现实体(EntityId 升序)。
    fn emergent(s: &Session) -> Vec<(EntityId, types::EntityState)> {
        s.entity_states(shard())
            .expect("分片已注册")
            .into_iter()
            .filter(|(id, _)| id.origin_shard() == EMERGENT_ORIGIN)
            .collect()
    }

    fn block(s: &Session, p: BlockPos) -> BlockId {
        let cp = p.chunk_of();
        let chunk = s.voxel_chunk(shard(), cp).expect("chunk 已装载");
        chunk.get(
            (p.x - cp.x * 16) as usize,
            (p.y - WORLD_Y_MIN) as usize,
            (p.z - cp.z * 16) as usize,
        )
    }

    fn center_of(p: BlockPos) -> Vec3 {
        Vec3::new(p.x as f32 + 0.5, p.y as f32 + 0.5, p.z as f32 + 0.5)
    }

    /// 第 k 个涌现实体(seq = k)在生成后一拍内仍在第 k 个脱落格附近。
    fn assert_spawned_at(got: &[(EntityId, types::EntityState)], cells: &[BlockPos]) {
        assert_eq!(got.len(), cells.len(), "涌现实体数");
        for (k, ((id, state), cell)) in got.iter().zip(cells).enumerate() {
            assert_eq!(*id, EntityId::from_parts(EMERGENT_ORIGIN, k as u32 + 1));
            assert!(
                (state.translation - center_of(*cell)).length() < 0.35,
                "{id:?} 应在 {cell:?} 附近,实为 {:?}",
                state.translation
            );
        }
    }

    /// AC-D4 生成状态(纯函数):格中心、单位旋转、`linvel = impulse / DEBRIS_MASS`、角速度 0;
    /// 半边长 0.45、方块为原方块;质量 0.729 kg。
    ///
    /// 会变红的实现:位置用格角;速度没除质量(或除了 1 kg);描述沿用默认的 stone / 0.5。
    #[test]
    fn acc_d4_detach_spawn_state() {
        assert!((DEBRIS_MASS - 0.729).abs() < 1e-6);
        let impulse = Vec3::new(-1.024652, 3.158188, -0.614791);
        let (state, desc) = detach_spawn(&DetachRequest {
            cell: BlockPos::new(61, -61, 62),
            block: builtin::DIRT,
            impulse,
        });
        assert!((state.translation - Vec3::new(61.5, -60.5, 62.5)).length() < 1e-6);
        assert!(state.rotation.abs_diff_eq(Quat::IDENTITY, 1e-6));
        assert!((state.linvel - impulse / DEBRIS_MASS).length() < 1e-4);
        assert_eq!(state.angvel, Vec3::ZERO);
        assert_eq!(
            desc,
            EntityDesc {
                half_size: DEBRIS_HALF_SIZE,
                block: builtin::DIRT,
            }
        );
    }

    /// AC-D4 生产路径:爆炸那一拍(N)格已写 air、还没有涌现实体;N+1 出现 24 个,ID 为
    /// `(EMERGENT_ORIGIN, 1..=24)`,第 k 个对应第 k 个脱落格(BlockPos 升序);描述正确;归属本分片。
    ///
    /// 会变红的实现:当拍就生成(与 N+1 语义不符);ID 用服务端 origin 0 或按别的顺序编号;
    /// 生成到别的分片;描述沿用默认值。
    #[test]
    fn acc_d4_spawned_next_tick_with_emergent_ids() {
        let mut s = session(64);
        let mut log = CommandLog::new();
        detonate(&mut s, &mut log, CENTER);
        s.tick();
        assert_eq!(
            block(&s, BlockPos::new(64, -61, 64)),
            builtin::AIR,
            "N 拍已写 air"
        );
        assert!(emergent(&s).is_empty(), "N 拍还没有涌现实体");
        s.tick();
        let got = emergent(&s);
        assert_spawned_at(&got, &detached_cells(CENTER, 48));
        for (id, _) in &got {
            assert_eq!(
                s.entity_desc(shard(), *id),
                Some(EntityDesc {
                    half_size: DEBRIS_HALF_SIZE,
                    block: builtin::DIRT,
                })
            );
            assert!(s.entities_of(shard()).contains(id), "归属本分片");
        }
        s.shutdown();
    }

    /// AC-D7 每拍上限:上限 10 时只生成压力最大的 10 个(其余粉碎,60 格全部变 air);同一拍两次爆炸
    /// 共用上限 30——先到的爆炸脱落 24,后到的只剩 6 个名额,取它压力最大的 6 个。
    ///
    /// 会变红的实现:上限只按单次爆炸计;超限的格没写 air(坑变小);超限时按 BlockPos 截断而不是按压力。
    #[test]
    fn acc_d7_per_tick_cap() {
        let mut s = session(10);
        let mut log = CommandLog::new();
        detonate(&mut s, &mut log, CENTER);
        s.tick();
        s.tick();
        assert_spawned_at(&emergent(&s), &detached_cells(CENTER, 10));
        for cell in detached_cells(CENTER, 48) {
            assert_eq!(block(&s, cell), builtin::AIR, "{cell:?} 超限也要写 air");
        }
        s.shutdown();

        let mut s = session(30);
        let mut log = CommandLog::new();
        detonate(&mut s, &mut log, CENTER);
        detonate(&mut s, &mut log, FAR);
        s.tick();
        s.tick();
        let mut cells = detached_cells(CENTER, 48);
        cells.extend(detached_cells(FAR, 6));
        cells.sort();
        assert_spawned_at(&emergent(&s), &cells);
        s.shutdown();
    }

    /// AC-D4 + 回放(生产恢复路径):在 N 拍(爆炸)与 N+1 拍(生成)之间取世界快照,用 `replay_world`
    /// 恢复;两边继续跑,生成的实体 ID 与状态逐位相同;再各炸一次,新 ID 从 25 接着编,仍逐位相同。
    ///
    /// 会变红的实现:待生成的脱落请求不在快照里(恢复线少 24 个);涌现计数器不进快照(恢复线
    /// 第二次爆炸从 1 重新编号);生成的盖章输入写进了外部命令日志(回放时重复生成)。
    #[test]
    fn acc_d4_pending_spawns_survive_snapshot_and_replay() {
        let mut a = session(64);
        let mut log = CommandLog::new();
        detonate(&mut a, &mut log, CENTER);
        a.tick();
        let world = a.world_snapshot();
        let mut b = sim_api::replay::replay_world(&world, &log, cfg(), registry).expect("回放");
        a.tick();
        b.tick();
        let ea = emergent(&a);
        assert_eq!(ea.len(), 24);
        assert_eq!(ea, emergent(&b), "恢复线生成相同的实体");

        detonate(&mut a, &mut log, FAR);
        detonate(&mut b, &mut CommandLog::new(), FAR);
        for _ in 0..2 {
            a.tick();
            b.tick();
        }
        let ea = emergent(&a);
        assert_eq!(ea.len(), 48);
        assert_eq!(
            ea[24].0,
            EntityId::from_parts(EMERGENT_ORIGIN, 25),
            "接着编号"
        );
        assert_eq!(ea, emergent(&b), "第二次爆炸后两线仍逐位相同");
        a.shutdown();
        b.shutdown();
    }

    /// AC-D4 生命周期:碎块落定后凝固回 dirt——900 拍内涌现实体全部消失;弹坑周围的 dirt 比刚炸完时多
    /// (碎块凝固回去),且没有任何 stone 出现(描述里的方块是原方块,不是默认的 stone)。
    ///
    /// 会变红的实现:碎块永远不凝固(比如被当成常驻体);凝固成 stone;碎块穿地掉进虚空。
    #[test]
    fn acc_d4_debris_settles_and_solidifies() {
        let mut s = session(64);
        let mut log = CommandLog::new();
        detonate(&mut s, &mut log, CENTER);
        let count = |s: &Session, b: BlockId| {
            let mut n = 0;
            for x in 52..=76 {
                for z in 52..=76 {
                    for y in -63..=-55 {
                        if block(s, BlockPos::new(x, y, z)) == b {
                            n += 1;
                        }
                    }
                }
            }
            n
        };
        assert_eq!(count(&s, builtin::DIRT), 25 * 25 * 3);
        s.tick();
        assert_eq!(count(&s, builtin::DIRT), 25 * 25 * 3 - 60, "刚炸完");
        let mut ticks = 1;
        while ticks < 900 {
            s.tick();
            ticks += 1;
            if ticks > 2 && emergent(&s).is_empty() {
                break;
            }
        }
        assert!(emergent(&s).is_empty(), "900 拍内碎块全部凝固");
        let dirt = count(&s, builtin::DIRT);
        assert!(
            dirt > 25 * 25 * 3 - 60 + 16 && dirt <= 25 * 25 * 3 - 60 + 24,
            "碎块凝固回 dirt:{dirt}"
        );
        assert_eq!(count(&s, builtin::STONE), 0, "没有凝固成 stone");
        s.shutdown();
    }
}

/// 单元 04a:坍塌求值纯函数——沙子规则与向下优先的连通性搜索(AC-D5 的算法部分)。
///
/// 标准答案来自总架构师按 REQ-009 §6 写的参照实现。局部坐标:y = −4 bedrock、−3…−1 dirt
/// (`thick` 层 dirt 时 bedrock 在 y = −thick−1),其上为结构;`removed` 中的格在世界里已是空气。
mod unit04a {
    use physics_sim::destruction::{CollapseOutcome, find_collapse};
    use std::collections::{BTreeMap, BTreeSet};
    use types::{BlockId, BlockPos};
    use voxel::{SimpleRegistry, builtin};

    const S: BlockId = builtin::STONE;

    fn p(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    struct Case {
        cells: BTreeMap<BlockPos, BlockId>,
        thick: i32,
        unloaded: BTreeSet<BlockPos>,
    }

    impl Case {
        fn new(cells: impl IntoIterator<Item = (BlockPos, BlockId)>) -> Self {
            Self {
                cells: cells.into_iter().collect(),
                thick: 3,
                unloaded: BTreeSet::new(),
            }
        }

        /// 移除 `removed` 后求值;同时核对把 `removed` 倒序传入结果相同(与顺序无关)。
        fn run(
            &self,
            removed: &[BlockPos],
            max_cells: usize,
            max_cluster: usize,
        ) -> CollapseOutcome {
            let gone: BTreeSet<BlockPos> = removed.iter().copied().collect();
            let world = |q: BlockPos| -> Option<BlockId> {
                if self.unloaded.contains(&q) {
                    return None;
                }
                if gone.contains(&q) {
                    return Some(builtin::AIR);
                }
                if let Some(b) = self.cells.get(&q) {
                    return Some(*b);
                }
                Some(if q.y == -self.thick - 1 {
                    builtin::BEDROCK
                } else if (-self.thick..=-1).contains(&q.y) {
                    builtin::DIRT
                } else {
                    builtin::AIR
                })
            };
            let reg = SimpleRegistry::builtin();
            let out = find_collapse(&world, &reg, removed, max_cells, max_cluster);
            let mut rev = removed.to_vec();
            rev.reverse();
            let again = find_collapse(&world, &reg, &rev, max_cells, max_cluster);
            assert_eq!(
                (&out.falling, out.oversized_clusters, out.budget_anchored),
                (
                    &again.falling,
                    again.oversized_clusters,
                    again.budget_anchored
                ),
                "结果与 removed 的顺序无关"
            );
            assert!(
                out.falling.windows(2).all(|w| w[0] < w[1]),
                "falling 按 BlockPos 升序且无重复"
            );
            out
        }

        fn run_default(&self, removed: &[BlockPos]) -> CollapseOutcome {
            self.run(removed, 2048, 512)
        }
    }

    fn line(xs: std::ops::RangeInclusive<i32>, y: i32, z: i32) -> Vec<BlockPos> {
        xs.map(|x| p(x, y, z)).collect()
    }

    fn column(x: i32, ys: std::ops::RangeInclusive<i32>, z: i32) -> Vec<BlockPos> {
        ys.map(|y| p(x, y, z)).collect()
    }

    /// AC-D5 石桥:两根 6 高石柱(x = 0、11)托着 12 格桥面(y = 6)。只去掉一根柱子,桥面经另一根柱子
    /// 连着地基,不塌;两根都去掉,桥面 12 格塌落(柱子已全部移除,不在结果里)。
    ///
    /// 会变红的实现:把"去掉一根柱子"判成悬空(没有走到另一根柱子);把 `removed` 的格也算进结果;
    /// 结果没排序。
    #[test]
    fn acc_d5_bridge_needs_both_pillars() {
        let pa = column(0, 0..=5, 0);
        let pb = column(11, 0..=5, 0);
        let deck = line(0..=11, 6, 0);
        let case = Case::new(pa.iter().chain(&pb).chain(&deck).map(|&c| (c, S)));
        let one = case.run_default(&pa);
        assert!(one.falling.is_empty(), "一根柱子还在,不塌:{one:?}");
        let mut both = pa.clone();
        both.extend(&pb);
        assert_eq!(case.run_default(&both).falling, deck);
    }

    /// AC-D5 悬臂与塔:墙上伸出的 8 格横臂切断根部 → 其余 7 格塌落;1×1×20 的塔切断底部 → 上面 19 格塌落。
    ///
    /// 会变红的实现:墙与横臂的连接没算(横臂根部那一格被当成锚);塔只塌最底下一格。
    #[test]
    fn acc_d5_cantilever_and_tower() {
        let mut cells: Vec<(BlockPos, BlockId)> = Vec::new();
        for y in 0..=7 {
            for z in -3..=3 {
                cells.push((p(0, y, z), S));
            }
        }
        cells.extend(line(1..=8, 5, 0).into_iter().map(|c| (c, S)));
        assert_eq!(
            Case::new(cells).run_default(&[p(1, 5, 0)]).falling,
            line(2..=8, 5, 0)
        );
        let tower = Case::new(column(0, 0..=19, 0).into_iter().map(|c| (c, S)));
        assert_eq!(
            tower.run_default(&[p(0, 0, 0)]).falling,
            column(0, 1..=19, 0)
        );
    }

    /// AC-D5 平台与两个上限:4 根柱子托着 20×20 的平台,柱子全部去掉 → 400 格塌落;`max_cluster` = 300 时
    /// 整块留在原地(计 1 个超大块);`max_cells` = 256 时搜索预算用完、判为锚定(计 1 次)。
    ///
    /// 会变红的实现:超大块塌了一部分;预算用完被当成悬空(大平台被误塌);两个计数混用。
    #[test]
    fn acc_d5_platform_and_limits() {
        let mut pillars = Vec::new();
        for (px, pz) in [(0, 0), (19, 0), (0, 19), (19, 19)] {
            pillars.extend(column(px, 0..=4, pz));
        }
        let mut deck = Vec::new();
        for x in 0..=19 {
            for z in 0..=19 {
                deck.push(p(x, 5, z));
            }
        }
        let case = Case::new(pillars.iter().chain(&deck).map(|&c| (c, S)));
        let all = case.run_default(&pillars);
        assert_eq!(all.falling.len(), 400);
        assert_eq!(all.falling, deck);
        assert_eq!((all.oversized_clusters, all.budget_anchored), (0, 0));

        let big = case.run(&pillars, 2048, 300);
        assert!(big.falling.is_empty());
        assert_eq!((big.oversized_clusters, big.budget_anchored), (1, 0));

        let poor = case.run(&pillars, 256, 512);
        assert!(poor.falling.is_empty());
        assert_eq!((poor.oversized_clusters, poor.budget_anchored), (0, 1));
    }

    /// AC-D5 开阔地面上的代价:平地移除一格,向下优先很快碰到基岩(访问 ≤ 64);24 层厚的地基同样
    /// 很快(访问 ≤ 500;按 BlockPos 顺序的广度优先要访问上千格)。
    ///
    /// 会变红的实现:用广度优先或按 BlockPos 出队(厚地基访问数上千);碰到基岩不停。
    #[test]
    fn acc_d5_open_ground_is_cheap() {
        let flat = Case::new([]).run_default(&[p(5, -1, 5)]);
        assert!(flat.falling.is_empty());
        assert!(flat.visited <= 64, "平地访问 {}", flat.visited);
        let mut thick = Case::new([]);
        thick.thick = 24;
        let deep = thick.run_default(&[p(5, -1, 5)]);
        assert!(deep.falling.is_empty());
        assert!(deep.visited <= 500, "厚地基访问 {}", deep.visited);
    }

    /// AC-D5 未装载即锚定:横臂唯一的支撑点所在 chunk 未装载 → 不塌;同样的支撑点是空气 → 横臂 8 格塌落。
    ///
    /// 会变红的实现:把未装载当成空气(看不见的地方被当成悬空,远处的结构被误塌)。
    #[test]
    fn acc_d5_unloaded_counts_as_anchor() {
        let arm = || Case::new(line(1..=9, 5, 0).into_iter().map(|c| (c, S)));
        let mut hidden = arm();
        hidden.unloaded.insert(p(0, 5, 0));
        assert!(hidden.run_default(&[p(9, 5, 0)]).falling.is_empty());
        assert_eq!(arm().run_default(&[p(9, 5, 0)]).falling, line(1..=8, 5, 0));
    }

    /// AC-D5 沙子:石头上的 5 格沙柱,石头被移走 → 5 格沙同一轮全部塌落;贴在墙侧的沙子,下面那格被移走 →
    /// 它塌落(尽管与墙相连);放在地面上的沙子,旁边的格被移走 → 不动。
    ///
    /// 会变红的实现:沙柱一拍只落最下面一格;沙子也按连通性判定(贴墙的沙子不落);旁边的移除也让沙子掉落。
    #[test]
    fn acc_d5_sand_rules() {
        let mut col: Vec<(BlockPos, BlockId)> = vec![(p(0, 0, 0), S)];
        col.extend(column(0, 1..=5, 0).into_iter().map(|c| (c, builtin::SAND)));
        assert_eq!(
            Case::new(col).run_default(&[p(0, 0, 0)]).falling,
            column(0, 1..=5, 0)
        );

        let mut side: Vec<(BlockPos, BlockId)> =
            column(0, 0..=5, 0).into_iter().map(|c| (c, S)).collect();
        side.push((p(1, 2, 0), S));
        side.push((p(1, 3, 0), builtin::SAND));
        assert_eq!(
            Case::new(side).run_default(&[p(1, 2, 0)]).falling,
            [p(1, 3, 0)]
        );

        let grounded = Case::new([(p(2, 0, 0), builtin::SAND), (p(3, 0, 0), S)]);
        assert!(grounded.run_default(&[p(3, 0, 0)]).falling.is_empty());
    }
}

/// 单元 04b:坍塌接入分片——触发与时机、本体区转换、每拍名额与推迟、跨分片(AC-D5 生产路径、AC-D6)。
///
/// 经 `Session` 驱动;用 `set_block` 搭结构、移除支撑(外部写入同样触发坍塌)。生产坐标:地表 y = −60。
mod unit04b {
    use physics_sim::destruction::{DEBRIS_HALF_SIZE, DestructionConfig};
    use physics_sim::{ShardSim, ShardSimConfig};
    use shard::SchedulerConfig;
    use sim_api::log::CommandLog;
    use sim_api::session::{Session, SessionConfig};
    use types::glam::Vec3;
    use types::{
        BlockId, BlockPos, ChunkPos, DimensionId, EMERGENT_ORIGIN, EntityDesc, EntityId,
        EntityState, ShardId, ShardPos, WORLD_Y_MIN, WorldId, WorldSeed,
    };
    use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

    fn shard(x: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, 0))
    }

    fn owner(p: BlockPos) -> ShardId {
        shard(if p.x < 128 { 0 } else { 1 })
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

    /// 会话:`shards` 中每个分片都装载 chunk x ∈ `cx`、z ∈ 2..=5 的平地(两分片时互为扩张区副本)。
    fn session(
        shards: &[i32],
        cx: std::ops::RangeInclusive<i32>,
        enabled: bool,
        cap: usize,
    ) -> Session {
        let mut s = Session::new(cfg());
        for &x in shards {
            let mut sim_cfg = ShardSimConfig::new(shard(x));
            sim_cfg.destruction = DestructionConfig {
                enabled,
                max_detach_per_tick: cap,
                ..DestructionConfig::default()
            };
            let mut sim = ShardSim::new(sim_cfg, registry());
            for x in cx.clone() {
                for z in 2..=5 {
                    let cp = ChunkPos::new(x, z);
                    sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
                }
            }
            s.register(sim);
        }
        s
    }

    fn set(s: &mut Session, log: &mut CommandLog, cells: &[BlockPos], block: BlockId) {
        for &c in cells {
            for entry in s.set_block(owner(c), c, block).expect("set_block") {
                log.append(entry);
            }
        }
    }

    fn block(s: &Session, sh: ShardId, p: BlockPos) -> BlockId {
        let cp = p.chunk_of();
        s.voxel_chunk(sh, cp).expect("chunk 已装载").get(
            (p.x - cp.x * 16) as usize,
            (p.y - WORLD_Y_MIN) as usize,
            (p.z - cp.z * 16) as usize,
        )
    }

    fn emergent(s: &Session, sh: ShardId) -> Vec<(EntityId, EntityState)> {
        s.entity_states(sh)
            .expect("分片已注册")
            .into_iter()
            .filter(|(id, _)| id.origin_shard() == EMERGENT_ORIGIN)
            .collect()
    }

    fn center_of(p: BlockPos) -> Vec3 {
        Vec3::new(p.x as f32 + 0.5, p.y as f32 + 0.5, p.z as f32 + 0.5)
    }

    fn column(x: i32, ys: std::ops::RangeInclusive<i32>, z: i32) -> Vec<BlockPos> {
        ys.map(|y| BlockPos::new(x, y, z)).collect()
    }

    fn row(xs: std::ops::RangeInclusive<i32>, y: i32, z: i32) -> Vec<BlockPos> {
        xs.map(|x| BlockPos::new(x, y, z)).collect()
    }

    /// 单分片石桥:柱 x = 60、71(y −60…−55,z = 64),桥面 y = −54、x 60…71。
    fn bridge(xa: i32, xb: i32) -> (Vec<BlockPos>, Vec<BlockPos>, Vec<BlockPos>) {
        (
            column(xa, -60..=-55, 64),
            column(xb, -60..=-55, 64),
            row(xa..=xb, -54, 64),
        )
    }

    fn count(
        s: &Session,
        sh: ShardId,
        b: BlockId,
        xs: std::ops::RangeInclusive<i32>,
        ys: std::ops::RangeInclusive<i32>,
        zs: std::ops::RangeInclusive<i32>,
    ) -> usize {
        let mut n = 0;
        for x in xs.clone() {
            for y in ys.clone() {
                for z in zs.clone() {
                    if block(s, sh, BlockPos::new(x, y, z)) == b {
                        n += 1;
                    }
                }
            }
        }
        n
    }

    /// 跑到本分片没有涌现实体为止(上限 900 拍),返回是否全部消失。
    fn settle(s: &mut Session, sh: ShardId) -> bool {
        for _ in 0..900 {
            s.tick();
            if emergent(s, sh).is_empty() {
                return true;
            }
        }
        false
    }

    /// AC-D5 石桥(生产路径):去掉一根柱子桥不塌;两根都去掉,当拍(N)桥面 12 格变 air,N+1 生成 12 个
    /// 涌现实体(描述 = 0.45 + stone,第 k 个对应 BlockPos 升序第 k 格);落地后全部凝固回 stone,
    /// 都落在地表那一层(y = −60),空中不留东西。
    ///
    /// 会变红的实现:外部 SetBlock 不触发坍塌;坍塌推迟到下一拍才写 air;生成的实体描述沿用默认;
    /// 坍塌块带了爆炸冲量四散。
    #[test]
    fn acc_d5_bridge_collapse_end_to_end() {
        let sh = shard(0);
        let mut s = session(&[0], 2..=6, true, 64);
        let mut log = CommandLog::new();
        let (pa, pb, deck) = bridge(60, 71);
        set(
            &mut s,
            &mut log,
            &[pa.clone(), pb.clone(), deck.clone()].concat(),
            builtin::STONE,
        );
        s.tick();
        set(&mut s, &mut log, &pa, builtin::AIR);
        s.tick();
        s.tick();
        assert_eq!(
            block(&s, sh, BlockPos::new(65, -54, 64)),
            builtin::STONE,
            "一根柱子还在,不塌"
        );
        assert!(emergent(&s, sh).is_empty());

        set(&mut s, &mut log, &pb, builtin::AIR);
        s.tick();
        for c in &deck {
            assert_eq!(block(&s, sh, *c), builtin::AIR, "N 拍桥面 {c:?} 已写 air");
        }
        assert!(emergent(&s, sh).is_empty(), "N 拍还没有实体");
        s.tick();
        let got = emergent(&s, sh);
        assert_eq!(got.len(), 12);
        for ((id, state), cell) in got.iter().zip(&deck) {
            assert!(
                (state.translation - center_of(*cell)).length() < 0.35,
                "{id:?} 在 {cell:?} 附近"
            );
            assert_eq!(
                s.entity_desc(sh, *id),
                Some(EntityDesc {
                    half_size: DEBRIS_HALF_SIZE,
                    block: builtin::STONE
                })
            );
        }
        assert!(settle(&mut s, sh), "900 拍内全部凝固");
        assert_eq!(
            count(&s, sh, builtin::STONE, 56..=75, -60..=-50, 60..=68),
            12,
            "凝固回 12 格 stone"
        );
        assert_eq!(
            count(&s, sh, builtin::STONE, 56..=75, -59..=-50, 60..=68),
            0,
            "都落在地表那一层"
        );
        s.shutdown();
    }

    /// AC-D5 沙柱(生产路径):石块上 5 格沙,移走石块 → 当拍 5 格沙变 air,N+1 生成 5 个实体
    /// (方块 = sand),落地后叠成 5 格沙柱(最高 y = −56)。
    ///
    /// 会变红的实现:沙柱一拍只落一格;沙子凝固成 stone;沙子按连通性判定(沙柱经旁边的地面锚定)。
    #[test]
    fn acc_d5_sand_column_end_to_end() {
        let sh = shard(0);
        let mut s = session(&[0], 2..=6, true, 64);
        let mut log = CommandLog::new();
        let base = BlockPos::new(66, -60, 70);
        let sand = column(66, -59..=-55, 70);
        set(&mut s, &mut log, &[base], builtin::STONE);
        set(&mut s, &mut log, &sand, builtin::SAND);
        s.tick();
        set(&mut s, &mut log, &[base], builtin::AIR);
        s.tick();
        for c in &sand {
            assert_eq!(block(&s, sh, *c), builtin::AIR, "{c:?}");
        }
        s.tick();
        let got = emergent(&s, sh);
        assert_eq!(got.len(), 5);
        for (id, _) in &got {
            assert_eq!(s.entity_desc(sh, *id).map(|d| d.block), Some(builtin::SAND));
        }
        assert!(settle(&mut s, sh));
        assert_eq!(count(&s, sh, builtin::SAND, 63..=69, -60..=-50, 67..=73), 5);
        assert_eq!(
            count(&s, sh, builtin::SAND, 63..=69, -55..=-50, 67..=73),
            0,
            "叠成 5 格高"
        );
        s.shutdown();
    }

    /// AC-D7 坍塌推迟 + 快照:每拍名额 5,两根柱子同拍去掉 → N 拍按 BlockPos 升序转换前 5 格,其余 7 格
    /// 推迟;在 N 与 N+1 之间取世界快照并回放,两线继续,N+3 后都正好生成 12 个实体,逐位相同、ID 不重复。
    ///
    /// 会变红的实现:超出名额的格丢失(只塌 5 格);推迟队列不进快照(回放线少塌 7 格);名额用完后
    /// 当拍没有转换就再也不触发搜索。
    #[test]
    fn acc_d7_collapse_deferred_and_snapshot() {
        let sh = shard(0);
        let mut a = session(&[0], 2..=6, true, 5);
        let mut log = CommandLog::new();
        let (pa, pb, deck) = bridge(60, 71);
        set(
            &mut a,
            &mut log,
            &[pa.clone(), pb.clone(), deck.clone()].concat(),
            builtin::STONE,
        );
        a.tick();
        set(&mut a, &mut log, &[pa, pb].concat(), builtin::AIR);
        a.tick();
        let air: Vec<BlockPos> = deck
            .iter()
            .copied()
            .filter(|c| block(&a, sh, *c) == builtin::AIR)
            .collect();
        assert_eq!(air, deck[..5], "N 拍按 BlockPos 升序转换前 5 格");

        let world = a.world_snapshot();
        let mut b = sim_api::replay::replay_world(&world, &log, cfg(), registry).expect("回放");
        for _ in 0..3 {
            a.tick();
            b.tick();
        }
        let ea = emergent(&a, sh);
        assert_eq!(ea.len(), 12, "三拍内全部生成");
        let ids: std::collections::BTreeSet<EntityId> = ea.iter().map(|e| e.0).collect();
        assert_eq!(ids.len(), 12, "ID 不重复");
        assert_eq!(ea, emergent(&b, sh), "回放线逐位相同");
        for c in &deck {
            assert_eq!(block(&a, sh, *c), builtin::AIR);
        }
        a.shutdown();
        b.shutdown();
    }

    /// AC-D6 跨分片石桥:柱 x = 122(分片 (0,0))与 x = 133(分片 (1,0)),桥面 x 122…133 横跨 x = 128。
    /// 两根柱子同拍去掉 → 两个分片各转换本体区内的 6 格;N+1 分片 (0,0) 得到 ID 1…6(x 122…127),
    /// 分片 (1,0) 得到 ID 7…12(x 128…133);再过一拍,两边的体素视图里整座桥面都是 air。
    ///
    /// 会变红的实现:分片把副本格也转换了(两边各 12 个实体);只有一边塌;ID 分配不按 ShardId 升序。
    #[test]
    fn acc_d6_cross_shard_bridge() {
        let mut s = session(&[0, 1], 6..=9, true, 64);
        let mut log = CommandLog::new();
        let (pa, pb, deck) = bridge(122, 133);
        set(
            &mut s,
            &mut log,
            &[pa.clone(), pb.clone(), deck.clone()].concat(),
            builtin::STONE,
        );
        s.tick();
        set(&mut s, &mut log, &[pa, pb].concat(), builtin::AIR);
        s.tick();
        s.tick();
        for (sh, xs, first) in [(shard(0), 122..=127, 1u32), (shard(1), 128..=133, 7u32)] {
            let got = emergent(&s, sh);
            let cells = row(xs, -54, 64);
            assert_eq!(got.len(), 6, "{sh:?} 生成 6 个");
            for (k, ((id, state), cell)) in got.iter().zip(&cells).enumerate() {
                assert_eq!(*id, EntityId::from_parts(EMERGENT_ORIGIN, first + k as u32));
                assert!((state.translation - center_of(*cell)).length() < 0.35);
            }
        }
        s.tick();
        for sh in [shard(0), shard(1)] {
            for c in &deck {
                assert_eq!(
                    block(&s, sh, *c),
                    builtin::AIR,
                    "{sh:?} 视图里 {c:?} 已是 air"
                );
            }
        }
        s.shutdown();
    }

    /// AC-D5 世界顶部:贴着世界最高格(y = 319)的 20 格石塔,切断底部 → 上面 19 格照样塌落。世界高度之外的
    /// 格是空气,不能当成"未装载"(否则碰到世界顶的结构会被误判为有地基)。
    ///
    /// 会变红的实现:`block_at` 对 y > WORLD_Y_MAX 返回 None(被当成未装载 → 锚定,塔不塌)。
    #[test]
    fn acc_d5_world_top_is_not_anchor() {
        let sh = shard(0);
        let mut s = session(&[0], 2..=6, true, 64);
        let mut log = CommandLog::new();
        let tower = column(66, 300..=319, 60);
        set(&mut s, &mut log, &tower, builtin::STONE);
        s.tick();
        set(&mut s, &mut log, &[tower[0]], builtin::AIR);
        s.tick();
        for c in &tower[1..] {
            assert_eq!(block(&s, sh, *c), builtin::AIR, "{c:?} 塌落");
        }
        s.tick();
        assert_eq!(emergent(&s, sh).len(), 19);
        s.shutdown();
    }

    /// 关闭破坏时行为不变:两根柱子都去掉,桥面仍悬在原处,不产生任何实体。
    ///
    /// 会变红的实现:坍塌不受总开关控制。
    #[test]
    fn acc_d5_disabled_no_collapse() {
        let sh = shard(0);
        let mut s = session(&[0], 2..=6, false, 64);
        let mut log = CommandLog::new();
        let (pa, pb, deck) = bridge(60, 71);
        set(
            &mut s,
            &mut log,
            &[pa.clone(), pb.clone(), deck.clone()].concat(),
            builtin::STONE,
        );
        s.tick();
        set(&mut s, &mut log, &[pa, pb].concat(), builtin::AIR);
        for _ in 0..5 {
            s.tick();
        }
        for c in &deck {
            assert_eq!(block(&s, sh, *c), builtin::STONE);
        }
        assert!(emergent(&s, sh).is_empty());
        s.shutdown();
    }
}

/// 单元 05:配置表 `[destruction]`、地形破坏遥测、checkpoint v6 往返回放(AC-D10、AC-D11)。
mod unit05 {
    use app::config::{Config, DestructionConfig};
    use app::driver::{
        CHECKPOINT_FORMAT_VERSION, CheckpointBundle, decode_checkpoint, encode_checkpoint,
    };
    use physics_sim::destruction::DestructionStats;
    use physics_sim::{ShardSim, ShardSimConfig};
    use shard::SchedulerConfig;
    use sim_api::log::CommandLog;
    use sim_api::session::{Session, SessionConfig};
    use types::glam::Vec3;
    use types::{
        BlockId, BlockPos, ChunkPos, DimensionId, ShardId, ShardPos, WORLD_Y_MIN, WorldId,
        WorldSeed,
    };
    use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

    const CENTER: Vec3 = Vec3::new(64.0, -59.5, 64.0);
    const FAR: Vec3 = Vec3::new(64.0, -59.5, 96.0);

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

    /// 单分片会话(平地 chunk 2..=6 × 2..=6,破坏开启),破坏参数由 `tweak` 调整。
    fn session(tweak: impl Fn(&mut physics_sim::destruction::DestructionConfig)) -> Session {
        let mut sim_cfg = ShardSimConfig::new(shard());
        sim_cfg.destruction.enabled = true;
        tweak(&mut sim_cfg.destruction);
        let mut sim = ShardSim::new(sim_cfg, registry());
        for x in 2..=6 {
            for z in 2..=6 {
                let cp = ChunkPos::new(x, z);
                sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
            }
        }
        let mut s = Session::new(cfg());
        s.register(sim);
        s
    }

    fn set(s: &mut Session, log: &mut CommandLog, cells: &[BlockPos], block: BlockId) {
        for &c in cells {
            for entry in s.set_block(shard(), c, block).expect("set_block") {
                log.append(entry);
            }
        }
    }

    fn detonate(s: &mut Session, log: &mut CommandLog, center: Vec3) {
        for entry in s.detonate(center, 12.0, 8.0).expect("爆炸") {
            log.append(entry);
        }
    }

    fn block(s: &Session, p: BlockPos) -> BlockId {
        let cp = p.chunk_of();
        s.voxel_chunk(shard(), cp).expect("chunk 已装载").get(
            (p.x - cp.x * 16) as usize,
            (p.y - WORLD_Y_MIN) as usize,
            (p.z - cp.z * 16) as usize,
        )
    }

    /// 单分片石桥:柱 x = 60、71(y −60…−55,z = 64),桥面 y = −54、x 60…71。
    fn bridge() -> (Vec<BlockPos>, Vec<BlockPos>) {
        let mut pillars = Vec::new();
        for x in [60, 71] {
            for y in -60..=-55 {
                pillars.push(BlockPos::new(x, y, 64));
            }
        }
        let deck = (60..=71).map(|x| BlockPos::new(x, -54, 64)).collect();
        (pillars, deck)
    }

    /// AC-D11 配置表:默认值;`[destruction]` 各键可覆写并映射到每个分片的配置;未知键、非正或非有限的
    /// `attenuation`、为 0 的上限、`collapse_max_cells < collapse_max_cluster` 都报错;toml 往返不丢字段。
    /// (`enabled` 的默认值在单元 07 定为 true,这里不断言。)
    ///
    /// 会变红的实现:表没接进 `shard_sim_config`(分片仍用核内默认);漏掉某条校验;键名拼错被当成未知键。
    #[test]
    fn acc_d11_destruction_config_table() {
        let d = Config::default().destruction;
        assert_eq!(
            (
                d.attenuation,
                d.max_debris_per_explosion,
                d.max_detach_per_tick,
                d.max_impact_breaks_per_tick,
                d.collapse_max_cells,
                d.collapse_max_cluster
            ),
            (1.0, 48, 64, 16, 2048, 512)
        );
        let text = "[destruction]\nenabled = true\nattenuation = 0.5\nmax_debris_per_explosion = 10\n\
                    max_detach_per_tick = 20\nmax_impact_breaks_per_tick = 3\ncollapse_max_cells = 900\n\
                    collapse_max_cluster = 300\n";
        let c = Config::from_toml_str(text).expect("合法");
        let want = DestructionConfig {
            enabled: true,
            attenuation: 0.5,
            max_debris_per_explosion: 10,
            max_detach_per_tick: 20,
            max_impact_breaks_per_tick: 3,
            collapse_max_cells: 900,
            collapse_max_cluster: 300,
        };
        assert_eq!(c.destruction, want);
        assert_eq!(
            c.shard_sim_config(shard()).destruction,
            physics_sim::destruction::DestructionConfig {
                enabled: true,
                attenuation: 0.5,
                max_debris_per_explosion: 10,
                max_detach_per_tick: 20,
                max_impact_breaks_per_tick: 3,
                collapse_max_cells: 900,
                collapse_max_cluster: 300,
            }
        );
        assert_eq!(Config::from_toml_str(&c.to_toml_string()).expect("往返"), c);
        for bad in [
            "[destruction]\nfoo = 1\n",
            "[destruction]\nattenuation = 0.0\n",
            "[destruction]\nattenuation = -1.0\n",
            "[destruction]\nattenuation = inf\n",
            "[destruction]\nmax_debris_per_explosion = 0\n",
            "[destruction]\nmax_detach_per_tick = 0\n",
            "[destruction]\nmax_impact_breaks_per_tick = 0\n",
            "[destruction]\ncollapse_max_cluster = 0\n",
            "[destruction]\ncollapse_max_cells = 100\ncollapse_max_cluster = 512\n",
        ] {
            assert!(Config::from_toml_str(bad).is_err(), "应拒绝:{bad}");
        }
    }

    /// AC-D11 爆炸遥测:平地标准爆炸那一拍 `cells_blasted` 60 = 粉碎 36 + 脱落 24,爆炸写入随后触发一次坍塌
    /// 求值(访问数 > 0、没有转换);下一拍(生成)全部为 0。名额 10 时脱落 10、粉碎 50,其中 14 个是因名额改粉碎。
    ///
    /// 会变红的实现:计数没接线;粉碎数漏掉因上限改粉碎的;调度器只取了某一个分片的计数;计数跨拍累计。
    #[test]
    fn acc_d11_blast_telemetry() {
        let mut s = session(|_| {});
        let mut log = CommandLog::new();
        detonate(&mut s, &mut log, CENTER);
        let t = s.tick().destruction;
        assert_eq!(
            (
                t.cells_blasted,
                t.pulverized,
                t.detached,
                t.pulverized_over_cap,
                t.cells_collapsed
            ),
            (60, 36, 24, 0, 0)
        );
        assert_eq!(t.collapse_evaluations, 1);
        assert!(t.collapse_visited > 0);
        assert_eq!(
            s.tick().destruction,
            DestructionStats::default(),
            "下一拍归零"
        );
        s.shutdown();

        let mut s = session(|d| d.max_detach_per_tick = 10);
        let mut log = CommandLog::new();
        detonate(&mut s, &mut log, CENTER);
        let t = s.tick().destruction;
        assert_eq!(
            (
                t.cells_blasted,
                t.pulverized,
                t.detached,
                t.pulverized_over_cap
            ),
            (60, 50, 10, 14)
        );
        s.shutdown();
    }

    /// AC-D11 坍塌遥测:名额 5 时桥面分三拍塌完(转换 5/5/2、推迟 7/2/0);搜索预算 8 时判为锚定
    /// (`collapse_budget_anchored` ≥ 1,不塌);悬空块上限 8 时计 1 个超大块(不塌)。
    ///
    /// 会变红的实现:推迟数没算或算成触发集大小;预算锚定与超大块混计;坍塌转换没计入 `detached`。
    #[test]
    fn acc_d11_collapse_telemetry() {
        let (pillars, deck) = bridge();
        let mut s = session(|d| d.max_detach_per_tick = 5);
        let mut log = CommandLog::new();
        set(
            &mut s,
            &mut log,
            &[pillars.clone(), deck.clone()].concat(),
            builtin::STONE,
        );
        s.tick();
        set(&mut s, &mut log, &pillars, builtin::AIR);
        let mut seen = Vec::new();
        for _ in 0..3 {
            let t = s.tick().destruction;
            seen.push((t.cells_collapsed, t.collapse_deferred, t.detached));
        }
        assert_eq!(seen, [(5, 7, 5), (5, 2, 5), (2, 0, 2)]);
        s.shutdown();

        for (cells, cluster, budget, oversized) in [(8, 8, true, false), (2048, 8, false, true)] {
            let mut s = session(|d| {
                d.collapse_max_cells = cells;
                d.collapse_max_cluster = cluster;
            });
            let mut log = CommandLog::new();
            set(
                &mut s,
                &mut log,
                &[pillars.clone(), deck.clone()].concat(),
                builtin::STONE,
            );
            s.tick();
            set(&mut s, &mut log, &pillars, builtin::AIR);
            let t = s.tick().destruction;
            assert_eq!(t.cells_collapsed, 0);
            assert_eq!(t.collapse_budget_anchored >= 1, budget, "预算锚定:{t:?}");
            assert_eq!(t.collapse_oversized, u64::from(oversized), "超大块:{t:?}");
            assert_eq!(
                block(&s, BlockPos::new(65, -54, 64)),
                builtin::STONE,
                "桥面留在原地"
            );
            s.shutdown();
        }
    }

    /// AC-D10 checkpoint v6 往返 + 回放:在"爆炸碎块待生成、桥面坍塌被名额推迟、涌现计数器 > 1"的时刻,
    /// 把世界快照与命令日志编码成 checkpoint 字节、解码、`replay_world` 恢复;两线各跑 300 拍,
    /// 全部 chunk 与全部实体状态逐位相同。
    ///
    /// 会变红的实现:任何一块新状态(待生成输入、推迟队列、涌现计数器、破坏配置)没进 v6 布局,
    /// 或解码后丢失——恢复线会少生成实体、少塌格或 ID 不同。
    #[test]
    fn acc_d10_checkpoint_roundtrip_replay() {
        let (pillars, deck) = bridge();
        let mut a = session(|d| d.max_detach_per_tick = 5);
        let mut log = CommandLog::new();
        set(
            &mut a,
            &mut log,
            &[pillars.clone(), deck].concat(),
            builtin::STONE,
        );
        a.tick();
        detonate(&mut a, &mut log, CENTER);
        a.tick();
        a.tick();
        set(&mut a, &mut log, &pillars, builtin::AIR);
        detonate(&mut a, &mut log, FAR);
        a.tick();

        let bundle = CheckpointBundle {
            format_version: CHECKPOINT_FORMAT_VERSION,
            world: a.world_snapshot(),
            log_bytes: log.to_bytes(),
            spawn_next_seq: 0,
        };
        let back = decode_checkpoint(&encode_checkpoint(&bundle)).expect("v6 往返");
        let replay_log = CommandLog::from_bytes(&back.log_bytes).expect("日志");
        let mut b =
            sim_api::replay::replay_world(&back.world, &replay_log, cfg(), registry).expect("回放");
        for _ in 0..300 {
            a.tick();
            b.tick();
        }
        assert_eq!(
            a.entity_states(shard()),
            b.entity_states(shard()),
            "实体逐位相同"
        );
        for x in 2..=6 {
            for z in 2..=6 {
                let cp = ChunkPos::new(x, z);
                assert_eq!(
                    a.voxel_chunk(shard(), cp),
                    b.voxel_chunk(shard(), cp),
                    "{cp:?}"
                );
            }
        }
        a.shutdown();
        b.shutdown();
    }
}
