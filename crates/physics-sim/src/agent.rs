//! 主体(agent)内核记录与生成(REQ-012 §3,A01):`BodyRegistry`(按内容
//! 哈希索引的身体注册表)、`AgentRecord`(随快照走)、类型化生成/移除。
//!
//! 范围(放行 A01 第 2/3 节):
//! - 一个主体占一个 `EntityId`;`EntityRecord.handle` 指向根连杆;`agents`
//!   表(ShardSim 字段)存身体哈希与连杆句柄表;主体 `actor` 恒为 true;
//! - 本单元内核只生成**单连杆**(玩家胶囊);多连杆定义可解析可校验(types),
//!   生成返回类型化拒绝并计入遥测,多连杆在 B01 实现;
//! - 生成口径与 `SpawnDynamic` 一致:重复 `EntityId` = fail-stop assert;
//!   生成位置与实心体素重叠 = 不检查(物理接触自然解决);
//! - 未知身体哈希 = 类型化拒绝 + 遥测(`Outbox.agent_spawn_rejected`);
//! - 主体迁移在 A04 实现:主体走到迁移检测路径 fail-stop;
//! - 主体不进任何把实体写成方块的路径(凝固/强制落地/兜底),不在
//!   `EntityStates` 下行(协议 S2C::AgentStates 在 A03)。
//!
//! rapier 类型不泄漏出 crate(防腐层,蓝图 §0)。

use std::collections::BTreeMap;

use types::{BodyDef, BodyHash, EntityId};

use crate::sim::ShardSim;

/// 按内容哈希索引的身体注册表(REQ-012 A01 第 1 节):启动时加载,注入
/// `ShardSim`,做法与 `BlockRegistry` 一样——不进快照,restore 时重新注入。
#[derive(Debug, Clone, Default)]
pub struct BodyRegistry {
    /// 哈希 → 定义。
    by_hash: BTreeMap<BodyHash, BodyDef>,
    /// 名字 → 哈希(客户端按名字引用;服务端解析成哈希)。
    by_name: BTreeMap<String, BodyHash>,
}

/// 注册表构造失败原因(类型化)。
#[derive(Debug, thiserror::Error)]
pub enum BodyRegistryError {
    #[error("身体名 {name} 对应哈希冲突(内容不同的定义同名):{name}")]
    HashConflict { name: String },
    #[error("body.toml 加载失败:{0}")]
    Parse(String),
}

impl BodyRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// 追加一个身体(解析 + 校验 + 哈希索引)。
    pub fn insert_toml(&mut self, text: &str) -> Result<BodyHash, BodyRegistryError> {
        let def =
            BodyDef::from_toml_str(text).map_err(|e| BodyRegistryError::Parse(e.to_string()))?;
        self.insert_checked(def)
    }

    /// 追加一个已构造的身体:哈希入索引,同名同哈希幂等,同名不同哈希冲突
    /// (类型化 `BodyRegistryError::HashConflict`,不覆盖旧条目——"名字"
    /// 与"哈希"两种引用不得分叉)。
    pub fn insert_checked(&mut self, def: BodyDef) -> Result<BodyHash, BodyRegistryError> {
        let hash = def.hash();
        let name = def.name.clone();
        match self.by_name.get(&name) {
            Some(prev) if *prev == hash => {}
            Some(_) => {
                return Err(BodyRegistryError::HashConflict { name });
            }
            None => {
                self.by_name.insert(name, hash);
            }
        }
        self.by_hash.insert(hash, def);
        Ok(hash)
    }

    /// 按哈希查定义。
    pub fn get(&self, hash: &BodyHash) -> Option<&BodyDef> {
        self.by_hash.get(hash)
    }

    /// 按名字查哈希(REQ-012 §3:客户端只能按名字引用)。
    pub fn hash_of(&self, name: &str) -> Option<BodyHash> {
        self.by_name.get(name).copied()
    }

    pub fn len(&self) -> usize {
        self.by_hash.len()
    }

    pub fn is_empty(&self) -> bool {
        self.by_hash.is_empty()
    }

    /// 全部哈希(升序 = BTreeMap 键序)。
    pub fn hashes(&self) -> Vec<BodyHash> {
        self.by_hash.keys().copied().collect()
    }
}

/// 主体的分片侧附加记录(REQ-012 §3):`EntityRecord` 之外,连杆句柄表与
/// 身体哈希。**随快照走**(A01 第 4 节)。
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AgentRecord {
    /// 身体内容哈希(restore 时校验注册表可解析,缺失 = 类型化错误)。
    pub body_hash: BodyHash,
    /// 连杆名 → 刚体句柄(单连杆 = 1 条,键 = 根连杆名)。
    pub links: BTreeMap<String, rapier3d::prelude::RigidBodyHandle>,
    /// 连杆碰撞体句柄(与 links 同序;登记进撞击归因表,A05 用)。
    pub colliders: Vec<rapier3d::prelude::ColliderHandle>,
}

/// 主体生成失败原因(类型化拒绝;不 panic)。
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum AgentSpawnError {
    /// 哈希不在注册表(REQ-012 A01 第 3 节:类型化拒绝 + 遥测)。
    #[error("未知身体哈希:{0}")]
    UnknownBody(BodyHash),
    /// 多连杆身体:内核 B01 才支持(A01 只生成单连杆)。
    #[error("多连杆身体({0} 连杆)在 B01 实现;A01 只生成单连杆")]
    MultiLink(usize),
    /// 定义校验失败(注册表加载时已校验;防御)。
    #[error("身体定义非法:{0}")]
    InvalidBody(String),
}

impl ShardSim {
    /// 主体是否存在。
    pub fn is_agent(&self, id: EntityId) -> bool {
        self.agents.contains_key(&id)
    }

    /// 主体的身体哈希;非主体/不存在 → None。
    pub fn agent_body_hash(&self, id: EntityId) -> Option<BodyHash> {
        self.agents.get(&id).map(|a| a.body_hash)
    }

    /// 主体数(测试观测口)。
    pub fn agent_count(&self) -> usize {
        self.agents.len()
    }

    /// 生成主体(REQ-012 §3;A01 第 3 节):
    /// - 未知哈希 → `Err(UnknownBody)` + 遥测(`agent_spawn_rejected`);
    /// - 多连杆 → `Err(MultiLink)` + 遥测(B01 实现);
    /// - 重复 `EntityId` → fail-stop(与 `SpawnDynamic` 同口径 assert);
    /// - 生成位置与实心体素重叠 → 不检查(与 `SpawnDynamic` 同口径:物理
    ///   接触下一步自然解决);
    /// - 质量/惯量乘全局系数 k(`cfg.agent_mass_scale`,§4 默认 1/100);
    /// - `actor` 恒为 true;`outbox.spawned` 照常(调度器 claim ownership)。
    pub fn spawn_agent(
        &mut self,
        entity: EntityId,
        body: BodyHash,
        state: types::EntityState,
    ) -> Result<(), AgentSpawnError> {
        let def = match self.bodies.get(&body) {
            Some(def) => def.clone(),
            None => {
                self.outbox.agent_spawn_rejected += 1;
                return Err(AgentSpawnError::UnknownBody(body));
            }
        };
        if def.links.len() != 1 {
            self.outbox.agent_spawn_rejected += 1;
            return Err(AgentSpawnError::MultiLink(def.links.len()));
        }
        if let Err(e) = def.validate() {
            self.outbox.agent_spawn_rejected += 1;
            return Err(AgentSpawnError::InvalidBody(e.to_string()));
        }
        // 重复 EntityId:与 SpawnDynamic 同口径 fail-stop(spawn_entity 的
        // assert 同款;类型化拒绝只留给"可选重试"的输入错误)。
        assert!(
            !self.entities.contains_key(&entity) && !self.agents.contains_key(&entity),
            "主体重复 spawn(fail-stop):{entity:?}"
        );
        let link = def.links.first().expect("单连杆(上面已判)");
        let k = self.cfg.agent_mass_scale;
        let mass = link.mass * k;
        // 单连杆生成:动态刚体 + 形状 collider(质量显式给定 = 真实单位 × k,
        // 不用密度)。锁定转动按定义(capsule demo)。
        let mut body_builder = rapier3d::prelude::RigidBodyBuilder::dynamic()
            .pose(crate::conv::pose_to_rapier(
                state.translation,
                state.rotation,
            ))
            .linvel(crate::conv::vec3_to_rapier(state.linvel))
            .angvel(crate::conv::vec3_to_rapier(state.angvel));
        if def.lock_rotations {
            body_builder = body_builder.lock_rotations();
        }
        let mut collider_builder = agent_collider_builder(&link.shapes, mass);
        collider_builder = collider_builder
            .friction(link.friction)
            .restitution(link.restitution);
        let (handle, collider) = self.phys.insert(body_builder, collider_builder);

        // 主体描述:凝固路径按 actor 豁免永不使用 `desc.block`;`half_size`
        // 取形状包络半高(唤醒/预算几何的足迹外扩用,保守取胶囊总半高)。
        let desc = types::EntityDesc {
            half_size: agent_half_size(&def),
            block: types::BlockId(1),
            shape: None,
        };
        self.entities.insert(
            entity,
            crate::sim::EntityRecord {
                handle,
                collider,
                desc,
                slow_ticks: 0,
                actor: true,
                source: None,
                window: std::collections::VecDeque::new(),
                carried: false,
                at_rest: false,
                actor_motion_ticks: None,
            },
        );
        let mut links = std::collections::BTreeMap::new();
        links.insert(link.name.clone(), handle);
        self.agents.insert(
            entity,
            AgentRecord {
                body_hash: body,
                links,
                colliders: vec![collider],
            },
        );
        // 调度器据此 claim 全局 ownership(REQ-006 任务 00;恰好一次)。
        self.outbox.spawned.push(entity);
        Ok(())
    }

    /// 移除主体(REQ-012 A01 第 3 节:刚体、碰撞体、`collider_to_entity`
    /// (拍内重建表,无需清理)、`agents` 全部回到生成前状态)。
    /// 主体不存在 → panic(fail-stop;调用方先判 `is_agent`)。
    pub fn despawn_agent(&mut self, id: EntityId) {
        let record = self.agents.remove(&id).expect("主体存在(fail-stop)");
        debug_assert!(
            record.links.len() == 1,
            "A01 只支持单连杆主体(多连杆在 B01)"
        );
        let rec = self.entities.remove(&id).expect("实体记录存在(fail-stop)");
        self.phys.remove_body(rec.handle);
        // 调度器据此 release 全局 ownership(REQ-006 任务 00)。
        self.outbox.despawned.push(id);
    }
}

/// 按形状定义构造 collider(REQ-012 §2.1:盒、球、胶囊、凸包;多形状 =
/// compound)。质量显式给定(真实单位 × k)。
pub(crate) fn agent_collider_builder(
    shapes: &[types::ShapeDef],
    mass: f32,
) -> rapier3d::prelude::ColliderBuilder {
    use rapier3d::prelude::*;
    if shapes.len() == 1 {
        return shape_collider(&shapes[0], mass);
    }
    let parts: Vec<(Pose, SharedShape)> = shapes
        .iter()
        .map(|s| {
            let shape = match s {
                types::ShapeDef::Box { half } => SharedShape::cuboid(half[0], half[1], half[2]),
                types::ShapeDef::Sphere { radius } => SharedShape::ball(*radius),
                types::ShapeDef::CapsuleY {
                    half_height,
                    radius,
                } => SharedShape::capsule_y(*half_height, *radius),
                types::ShapeDef::ConvexHull { points } => {
                    let pts: Vec<rapier3d::prelude::Vector> =
                        points.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
                    SharedShape::convex_hull(&pts).expect("凸包顶点合法(校验保证非退化)")
                }
            };
            (Pose::IDENTITY, shape)
        })
        .collect();
    ColliderBuilder::compound(parts).mass(mass)
}

fn shape_collider(s: &types::ShapeDef, mass: f32) -> rapier3d::prelude::ColliderBuilder {
    use rapier3d::prelude::*;
    let builder = match s {
        types::ShapeDef::Box { half } => ColliderBuilder::cuboid(half[0], half[1], half[2]),
        types::ShapeDef::Sphere { radius } => ColliderBuilder::ball(*radius),
        types::ShapeDef::CapsuleY {
            half_height,
            radius,
        } => ColliderBuilder::capsule_y(*half_height, *radius),
        types::ShapeDef::ConvexHull { points } => {
            let pts: Vec<rapier3d::prelude::Vector> =
                points.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
            ColliderBuilder::convex_hull(&pts).expect("凸包顶点合法(校验保证非退化)")
        }
    };
    builder.mass(mass)
}

/// 主体描述的 half_size:形状最大竖向半包络(足迹/唤醒几何的保守包络;
/// `envelope_radius = half·√3` ≥ 半高即可)。胶囊 = half_height + radius;
/// 盒/球/凸包取最大轴或半径。
pub(crate) fn agent_half_size(def: &BodyDef) -> f32 {
    def.links
        .iter()
        .flat_map(|l| l.shapes.iter())
        .map(|s| match s {
            types::ShapeDef::Box { half } => (half[0].abs().max(half[1].abs())).max(half[2].abs()),
            types::ShapeDef::Sphere { radius } => radius.abs(),
            types::ShapeDef::CapsuleY {
                half_height,
                radius,
            } => half_height.abs() + radius.abs(),
            types::ShapeDef::ConvexHull { points } => points
                .iter()
                .map(|p| p.iter().fold(0.0f32, |m, v| m.max(v.abs())))
                .fold(0.5f32, f32::max),
        })
        .fold(0.5f32, f32::max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::glam::Vec3;
    use types::{DimensionId, ShardPos, WorldId};
    use voxel::SimpleRegistry;

    /// 测试用分片身份(同 crate 既有测试风格)。
    pub(crate) fn shard_id(x: i32, z: i32) -> types::ShardId {
        types::ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    /// capsule 定义(与 assets/bodies/capsule.body.toml 同内容)。
    pub(crate) fn capsule_toml() -> &'static str {
        include_str!("../../../assets/bodies/capsule.body.toml")
    }

    /// 只装 capsule 的注册表(快照 restore 用:同哈希即可恢复)。
    pub(crate) fn fresh_capsule_registry() -> BodyRegistry {
        let mut bodies = BodyRegistry::new();
        bodies
            .insert_toml(capsule_toml())
            .expect("capsule.body.toml 合法");
        bodies
    }

    /// 凝固关闭的测试配置(与既有测试同风格)+ capsule 注册表。
    pub(crate) fn test_sim() -> (crate::sim::ShardSim, BodyHash) {
        let mut cfg = crate::sim::ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        let mut bodies = BodyRegistry::new();
        let hash = bodies
            .insert_toml(capsule_toml())
            .expect("capsule.body.toml 合法");
        let sim =
            crate::sim::ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), bodies);
        (sim, hash)
    }

    /// 载入 3×3 chunk 平板地形(层 0 全石,世界 y = 64-64 = 0)。
    pub(crate) fn load_flat_ground(sim: &mut crate::sim::ShardSim) {
        let stone = types::BlockId(1);
        for cx in 0..3i32 {
            for cz in 0..3i32 {
                let mut chunk = voxel::Chunk::air();
                for x in 0..voxel::CHUNK_X {
                    for z in 0..voxel::CHUNK_Z {
                        chunk.set(x, 64, z, stone);
                    }
                }
                sim.load_chunk(types::ChunkPos::new(cx, cz), chunk);
            }
        }
    }

    #[test]
    fn registry_indexes_by_hash_and_name() {
        let mut bodies = BodyRegistry::new();
        let hash = bodies.insert_toml(capsule_toml()).expect("合法");
        assert_eq!(bodies.len(), 1);
        assert_eq!(bodies.get(&hash).expect("哈希可查").name, "capsule");
        assert_eq!(bodies.hash_of("capsule"), Some(hash));
        assert_eq!(bodies.hash_of("不存在"), None);
        // 同名同内容幂等(名字仍指该哈希)。
        let again = bodies.insert_toml(capsule_toml()).expect("合法");
        assert_eq!(again, hash);
        assert_eq!(bodies.len(), 1, "同内容重复插入不增条目");
    }

    #[test]
    fn registry_rejects_invalid_toml() {
        let mut bodies = BodyRegistry::new();
        let err = bodies
            .insert_toml("name = \"x\"\n[[links]]\nname=\"a\"\nmass = 0\n")
            .unwrap_err();
        assert!(
            err.to_string().contains("质量") || err.to_string().contains("解析"),
            "类型化错误:{err}"
        );
    }

    /// 同名不同内容:类型化 `HashConflict`(名字与哈希两种引用不得分叉)。
    #[test]
    fn registry_rejects_name_hash_conflict() {
        let mut bodies = BodyRegistry::new();
        let hash = bodies.insert_toml(capsule_toml()).expect("合法");
        let conflicting = capsule_toml().replace("mass = 80.0", "mass = 79.0");
        match bodies.insert_toml(&conflicting) {
            Err(BodyRegistryError::HashConflict { name }) => {
                assert_eq!(name, "capsule");
            }
            other => panic!("同名不同内容必须 HashConflict,实为 {other:?}"),
        }
        // 旧条目未被覆盖:名字仍指原哈希。
        assert_eq!(bodies.hash_of("capsule"), Some(hash));
        assert_eq!(bodies.len(), 1);
    }

    // ---- 判据 A01-1(AC-A1):体素平地上生成胶囊,静止 3600 拍 ----

    /// 生成后单连杆主体进世界:质量 = 80 kg × 0.01 = 0.8(§4,rapier 刚体
    /// 质量断言);`actor = true`;`outbox.spawned` 照常。
    #[test]
    fn spawn_agent_mass_scaled_and_actor() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..types::EntityState::default()
            },
        )
        .expect("capsule 可生成");
        assert!(s.is_agent(a));
        assert_eq!(s.agent_body_hash(a), Some(hash));
        assert!(s.agent_count() == 1);
        let rec = s.entities.get(&a).expect("主体有实体记录");
        assert!(rec.actor, "主体 actor 恒为 true");
        let body = s.phys.bodies.get(rec.handle).expect("handle 有效");
        assert!(
            (body.mass() - 0.8).abs() < 1e-4,
            "质量 = 80 kg × 0.01 = 0.8,实为 {}",
            body.mass()
        );
        // 锁定转动:capsule demo 定值(三轴全锁)。
        assert_eq!(
            body.is_rotation_locked(),
            [true, true, true],
            "lock_rotations = true"
        );
        // 连杆碰撞体登记进 collider_to_entity(A05 归因结构就位)。
        let table = s.collider_to_entity_of();
        assert_eq!(
            table.get(&collider_key(rec.collider)),
            Some(&a),
            "主体连杆碰撞体进归因表"
        );
        assert!(
            s.outbox.spawned.contains(&a),
            "spawned 照常(claim ownership)"
        );
    }

    /// collider 句柄 → (index, generation) 键(collider_to_entity 同款)。
    fn collider_key(h: rapier3d::prelude::ColliderHandle) -> (u32, u32) {
        let (i, g) = h.into_raw_parts();
        (i, g)
    }

    /// 判据 A01-1:静止 3600 拍——主体还在、体素计数不变、下沉 ≤ 1 cm、
    /// 水平漂移 ≤ 1 cm;期间不加任何控制力。全程 `run_transient_lifecycle`
    /// 照常跑(凝固/强制落地/兜底冻结各路径都过一遍,主体都不受影响)。
    #[test]
    fn agent_stands_3600_ticks_on_flat_ground() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..types::EntityState::default()
            },
        )
        .expect("capsule 可生成");
        let chunks_before = s.voxel().chunk_count();
        let mut solidified = 0usize;
        for _ in 0..3600 {
            s.step();
            let out = s.take_outbox();
            solidified += out
                .block_updates
                .iter()
                .filter(|w| w.kind == crate::sim::BlockWriteKind::Emergent)
                .count();
        }
        s.run_transient_lifecycle();
        let st = s.entity_state(a).expect("主体还在(不移除、不凝固)");
        assert_eq!(
            s.voxel().chunk_count(),
            chunks_before,
            "体素计数不变(不凝固)"
        );
        assert_eq!(solidified, 0, "无任何 Emergent 写入(主体永不写方块)");
        assert!(
            (st.translation.y - 1.9).abs() <= 0.01,
            "下沉 ≤ 1 cm(站面 y=1、半高 0.9 → 静止中心 1.9):y={}",
            st.translation.y
        );
        assert!(
            (st.translation.x - 24.0).abs() <= 0.01 && (st.translation.z - 24.0).abs() <= 0.01,
            "水平漂移 ≤ 1 cm:x={},z={}",
            st.translation.x,
            st.translation.z
        );
    }

    /// 判据 A01-1(预算豁免):会让普通实体被预算冻结的配置下,主体不被
    /// 预算冻结(用现有预算测试的布景:budget_cfg(1),观察者 + 远距实体)。
    /// 常规实体在同布景下被淘汰(前置自证),主体同位不被冻、不被强制落地。
    #[test]
    fn agent_exempt_from_budget_freeze_and_forced_landing() {
        let mut cfg = crate::sim::ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.gravity = Vec3::ZERO; // 同 budget_cfg:零重力静止 = 可冻结
        cfg.activation.max_awake_dynamic = 1;
        let mut bodies = BodyRegistry::new();
        let hash = bodies
            .insert_toml(capsule_toml())
            .expect("capsule.body.toml 合法");
        let mut s =
            crate::sim::ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), bodies);
        let a = types::EntityId::from_parts(1, 1);
        let plain = types::EntityId::from_parts(1, 2);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(20.0, 3.0, 8.0),
                ..types::EntityState::default()
            },
        )
        .expect("capsule 可生成");
        s.spawn_entity_for_test(plain, Vec3::new(100.0, 3.0, 100.0));
        s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]); // 预算 1,两个醒者
        for _ in 0..21 {
            s.step();
        }
        s.run_transient_lifecycle();
        // 前置自证:普通常驻体已被预算冻结(速度清零 + kinematic)。
        {
            let rec = s.entities.get(&plain).expect("普通实体还在");
            let b = s.phys.bodies.get(rec.handle).expect("handle 有效");
            assert!(
                b.is_kinematic(),
                "前置:预算 1 下普通常驻体被淘汰冻结(同布景对照)"
            );
        }
        // 主体同布景:仍醒着(Dynamic)、速度未清零。
        {
            let rec = s.entities.get(&a).expect("主体还在");
            assert!(rec.actor);
            let b = s.phys.bodies.get(rec.handle).expect("handle 有效");
            assert!(b.is_dynamic(), "主体豁免预算冻结(不被 kinematic 化)");
        }
        assert!(!s.forced_pending.contains(&a), "主体不进强制落地队列");
    }

    /// 判据 A01-1(移除无残留):移除后刚体、碰撞体、归因表、agents、
    /// 实体表、spawned/despawned 簿记全部回到生成前状态(逐项断言数量
    /// 与键集合)。
    #[test]
    fn despawn_agent_leaves_no_residue() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..types::EntityState::default()
            },
        )
        .expect("capsule 可生成");
        s.step();
        let _ = s.take_outbox();
        let (bodies_before, colliders_before) = (s.phys.bodies.len(), s.phys.colliders.len());
        let collider = s.entities.get(&a).expect("存在").collider;
        s.despawn_agent(a);
        assert_eq!(s.agent_count(), 0, "agents 表清空");
        assert!(!s.entities.contains_key(&a), "实体记录清空");
        assert!(s.entity_state(a).is_none());
        let (bodies_after, colliders_after) = (s.phys.bodies.len(), s.phys.colliders.len());
        assert_eq!(bodies_after, bodies_before - 1, "刚体回到生成前");
        assert_eq!(colliders_after, colliders_before - 1, "碰撞体回到生成前");
        let table = s.collider_to_entity_of();
        assert!(
            !table.contains_key(&collider_key(collider)),
            "collider_to_entity 不再有主体条目"
        );
        let out = s.take_outbox();
        assert!(
            out.despawned.contains(&a),
            "despawned 照常(release ownership)"
        );
    }

    /// 重复 EntityId:与 SpawnDynamic 同口径 fail-stop(类型化拒绝只留给
    /// 可选重试的输入错误)。
    #[test]
    #[should_panic(expected = "主体重复 spawn")]
    fn spawn_agent_duplicate_entity_fails_stop() {
        let (mut s, hash) = test_sim();
        let a = types::EntityId::from_parts(1, 1);
        let st = types::EntityState {
            translation: Vec3::new(60.0, 3.0, 60.0),
            ..types::EntityState::default()
        };
        s.spawn_agent(a, hash, st).expect("首次成功");
        let _ = s.spawn_agent(a, hash, st);
    }

    /// 未知身体哈希:类型化拒绝 + 遥测(Outbox.agent_spawn_rejected)。
    #[test]
    fn spawn_agent_unknown_hash_rejected_with_telemetry() {
        let mut s = test_sim().0;
        let unknown = BodyHash([0xAB; 8]);
        let err = s
            .spawn_agent(
                types::EntityId::from_parts(1, 1),
                unknown,
                types::EntityState::default(),
            )
            .unwrap_err();
        assert_eq!(err, AgentSpawnError::UnknownBody(unknown), "类型化拒绝");
        assert_eq!(s.outbox.agent_spawn_rejected, 1, "遥测计数");
        assert_eq!(s.agent_count(), 0);
    }

    /// 多连杆身体:A01 类型化拒绝 + 遥测(B01 实现)。
    #[test]
    fn spawn_agent_multi_link_rejected_with_telemetry() {
        let two_link = r#"
name = "two"
[[links]]
name = "base"
mass = 10.0
[[links.shapes]]
kind = "sphere"
radius = 0.1
[[links]]
name = "arm"
mass = 1.0
[[links.shapes]]
kind = "sphere"
radius = 0.1
[[joints]]
parent = "base"
child = "arm"
type = "revolute"
axis = [1.0, 0.0, 0.0]
anchor = [0.0, 0.0, 0.0]
"#;
        let mut bodies = BodyRegistry::new();
        let h = bodies.insert_toml(two_link).expect("两连杆定义合法");
        // 同一注册表实例注入(s.bodies 不可变借走,重建 sim)。
        let mut cfg = crate::sim::ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        let mut s2 =
            crate::sim::ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), bodies);
        let err = s2
            .spawn_agent(
                types::EntityId::from_parts(1, 1),
                h,
                types::EntityState::default(),
            )
            .unwrap_err();
        assert!(matches!(err, AgentSpawnError::MultiLink(2)), "类型化拒绝");
        assert_eq!(s2.outbox.agent_spawn_rejected, 1, "遥测计数");
    }

    // ---- 判据 A01-4:检查点 v8 快照往返 + UnknownBody ----

    /// 含主体的快照:往返后 agents 表逐位还原(哈希、连杆句柄表、碰撞体
    /// 句柄表),空注册表 restore = 类型化 UnknownBody(不 panic)。
    #[test]
    fn snapshot_roundtrip_with_agent_and_unknown_body() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..types::EntityState::default()
            },
        )
        .expect("capsule 可生成");
        s.step();
        let bytes = s.snapshot();

        // 空注册表 restore:类型化拒绝(不 panic)。
        let err = match crate::sim::ShardSim::restore_with_bodies(
            &bytes,
            Box::new(SimpleRegistry::builtin()),
            BodyRegistry::new(),
        ) {
            Err(e) => e,
            Ok(_) => panic!("空注册表 restore 含主体快照必须类型化拒绝"),
        };
        match err {
            crate::snapshot::SimSnapshotError::UnknownBody { entity, hash: h } => {
                assert_eq!(entity, a);
                assert_eq!(h, hash);
            }
            other => panic!("必须是 UnknownBody,实为 {other:?}"),
        }

        // 同哈希注册表 restore:agents 表逐位还原。
        let restored = crate::sim::ShardSim::restore_with_bodies(
            &bytes,
            Box::new(SimpleRegistry::builtin()),
            fresh_capsule_registry(),
        )
        .expect("同注册表恢复成功");
        assert_eq!(restored.agent_count(), 1);
        assert_eq!(restored.agent_body_hash(a), Some(hash));
        let rec_before = s.entities.get(&a).expect("存在");
        let rec_after = restored.entities.get(&a).expect("存在");
        assert_eq!(rec_before.handle, rec_after.handle, "连杆句柄还原");
        assert_eq!(rec_before.collider, rec_after.collider, "碰撞体句柄还原");
        assert!(rec_after.actor, "actor 恒 true");
    }

    // ---- 判据 A01-2(AC-A6):撞击 + 爆炸 3600 拍,回放/检查点逐位 ----

    /// AC-A6 布景时间线(3600 拍):站着的胶囊 + 第 500 拍生成一个方块实体
    /// 在其正上方(重力下落砸中)、第 1500 拍附近一次爆炸。同一输入序列在
    /// 新实例上重放,每拍主体状态逐位相同。
    ///
    /// 布景参数:砸块从头顶 4 m 落下(~26 拍触顶),质量 0.729 vs 胶囊 0.8,
    /// 砸中后弹落在地面滚走凝固;爆炸 power 12 / radius 5,中心 (21.5, 1, 24)
    /// (距主体 2.5 m,斜向冲量把主体沿 +x 推开、阻尼停住;地面炸坑但 3×3
    /// 平板足够大,主体不出本体区)。
    fn ac_a6_run(s: &mut crate::sim::ShardSim, _hash: BodyHash) -> Vec<types::EntityState> {
        let a = types::EntityId::from_parts(1, 1);
        let mut trace = Vec::with_capacity(3600);
        for tick in 0..3600u32 {
            s.apply_prefix(&ac_a6_inputs(tick));
            s.step();
            let _ = s.take_outbox();
            trace.push(s.entity_state(a).expect("主体全程在(布景约束:不出本体区)"));
        }
        trace
    }

    fn ac_a6_sim() -> (crate::sim::ShardSim, BodyHash) {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..types::EntityState::default()
            },
        )
        .expect("capsule 可生成");
        (s, hash)
    }

    /// 判据 A01-2 前半:同一输入日志回放两次,每拍主体状态逐位相同。
    /// 布景自检:撞击与爆炸都实际发生(爆炸后主体被推动)。
    #[test]
    fn ac_a6_replay_is_bitwise_identical() {
        let (mut s1, hash) = ac_a6_sim();
        let trace1 = ac_a6_run(&mut s1, hash);
        let (mut s2, _) = ac_a6_sim();
        let trace2 = ac_a6_run(&mut s2, hash);
        assert_eq!(trace1.len(), 3600);
        for (t, (x, y)) in trace1.iter().zip(trace2.iter()).enumerate() {
            assert_eq!(x, y, "拍 {t} 主体状态逐位不同");
        }
        // 布景自检:爆炸后主体确实被推动(否则布景没有扰动,测试空转)。
        let (before, after) = (trace1[1499].translation, trace1[1520].translation);
        assert!(
            (before - after).length() > 0.01,
            "爆炸须实际推动主体:前 {before:?} 后 {after:?}"
        );
    }

    /// 判据 A01-2 后半:第 1000 拍检查点、恢复后续跑,与不中断的那一线
    /// 逐位相同;另做一组检查点落在方块撞胶囊的那几拍之间(砸块 ~26 拍
    /// 触顶,取拍 525 = 接触前一拍恢复)。
    #[test]
    fn ac_a6_checkpoint_resume_is_bitwise_identical() {
        let (mut golden, hash) = ac_a6_sim();
        let golden_trace = ac_a6_run(&mut golden, hash);
        for (cp_tick, label) in [(1000u32, "拍 1000"), (525u32, "拍 525(撞前)")] {
            let (mut s, _) = ac_a6_sim();
            let mut resume = Vec::new();
            let a = types::EntityId::from_parts(1, 1);
            for tick in 0..3600u32 {
                if tick == cp_tick {
                    let bytes = s.snapshot();
                    s = crate::sim::ShardSim::restore_with_bodies(
                        &bytes,
                        Box::new(SimpleRegistry::builtin()),
                        fresh_capsule_registry(),
                    )
                    .expect("检查点恢复");
                }
                // 每拍重演输入(与 ac_a6_run 同一序列)。
                s.apply_prefix(&ac_a6_inputs(tick));
                s.step();
                let _ = s.take_outbox();
                resume.push(s.entity_state(a).expect("主体全程在"));
            }
            for (t, (x, y)) in golden_trace.iter().zip(resume.iter()).enumerate() {
                assert_eq!(x, y, "{label} 检查点续跑:拍 {t} 主体状态逐位不同");
            }
        }
    }

    /// AC-A6 时间线的单拍输入(回放/检查点续跑共用;与 ac_a6_run 一致)。
    fn ac_a6_inputs(tick: u32) -> Vec<messaging::StampedInput> {
        use messaging::{InputPayload, SimCommand, StampedInput};
        use types::Tick;
        let mut inputs = Vec::new();
        if tick == 500 {
            inputs.push(StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: types::EntityId::from_parts(1, 2),
                    state: types::EntityState {
                        translation: Vec3::new(24.0, 5.9, 24.0),
                        ..types::EntityState::default()
                    },
                    desc: types::EntityDesc::default(),
                }),
            });
        }
        if tick == 1500 {
            inputs.push(StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 2,
                payload: InputPayload::Command(SimCommand::Detonate {
                    center: Vec3::new(21.5, 1.0, 24.0),
                    power: 12.0,
                    radius: 5.0,
                }),
            });
        }
        inputs
    }
}
