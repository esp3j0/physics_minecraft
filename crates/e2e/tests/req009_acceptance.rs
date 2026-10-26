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
