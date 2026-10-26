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
    #[ignore = "REQ-009 单元 02 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
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
    #[ignore = "REQ-009 单元 02 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
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
            .min_by(|a, b| a.pressure.total_cmp(&b.pressure))
            .expect("有脱落格");
        assert_eq!(weakest.pos, BlockPos::new(67, -61, 64));
        assert!((weakest.pressure - 2.488648).abs() < 1e-5);

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
    #[ignore = "REQ-009 单元 02 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
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
    #[ignore = "REQ-009 单元 02 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
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
    #[ignore = "REQ-009 单元 02 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
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
    #[ignore = "REQ-009 单元 02 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
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
    #[ignore = "REQ-009 单元 02 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
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
    #[ignore = "REQ-009 单元 02 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
    fn acc_d10_checkpoint_version_is_v6() {
        assert_eq!(app::driver::CHECKPOINT_FORMAT_VERSION, 6);
    }
}
