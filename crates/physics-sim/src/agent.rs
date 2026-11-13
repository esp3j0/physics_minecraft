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

    /// 注册表内最大身体包络半径(REQ-012 A04 第 4 节:计入 REQ-001 §4.2
    /// MARGIN 推导的"最大实体尺寸")。包络按形状最大外接半径取保守值:
    /// 盒 = 半对角 √3、球/胶囊 = 半径(+半高)、凸包 = 最大顶点模长,再加
    /// 形状 `offset` 的模长(B01a:脚球挂在小腿末端);多形状/多连杆取最大。
    /// 空注册表 → 0.0(无身体 = 无附加)。
    pub fn max_envelope_radius(&self) -> f32 {
        self.by_hash
            .values()
            .map(|def| {
                def.links
                    .iter()
                    .flat_map(|l| l.shapes.iter())
                    .map(|s| {
                        let (bound, offset): (f32, [f32; 3]) = match s {
                            types::ShapeDef::Box { half, offset } => (
                                (half[0] * half[0] + half[1] * half[1] + half[2] * half[2]).sqrt(),
                                *offset,
                            ),
                            types::ShapeDef::Sphere { radius, offset } => (radius.abs(), *offset),
                            types::ShapeDef::CapsuleY {
                                half_height,
                                radius,
                                offset,
                            } => (half_height.abs() + radius.abs(), *offset),
                            types::ShapeDef::ConvexHull { points, offset } => (
                                points
                                    .iter()
                                    .map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt())
                                    .fold(0.0f32, f32::max),
                                *offset,
                            ),
                        };
                        bound
                            + (offset[0] * offset[0]
                                + offset[1] * offset[1]
                                + offset[2] * offset[2])
                                .sqrt()
                    })
                    .fold(0.0f32, f32::max)
            })
            .fold(0.0f32, f32::max)
    }
}

/// 主体的分片侧附加记录(REQ-012 §3):`EntityRecord` 之外,连杆句柄表与
/// 身体哈希。**随快照走**(A01 第 4 节;A02a 第 2 节:意图是保持状态,也在
/// 本记录里随快照走)。
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AgentRecord {
    /// 身体内容哈希(restore 时校验注册表可解析,缺失 = 类型化错误)。
    pub body_hash: BodyHash,
    /// 连杆名 → 刚体句柄(单连杆 = 1 条,键 = 根连杆名)。
    pub links: BTreeMap<String, rapier3d::prelude::RigidBodyHandle>,
    /// 连杆碰撞体句柄(与 links 同序;登记进撞击归因表,A05 用)。
    pub colliders: Vec<rapier3d::prelude::ColliderHandle>,
    /// 最近一条意图的水平分量(世界系 m/s;None = 尚无意图,控制器不动)。
    /// 保持状态:直到下一条意图到来;随快照走(A02a 第 2 节)。
    pub intent_move: Option<(f32, f32)>,
    /// 最近一条意图的朝向(rad;内核规范化到 (−π, π])。随快照走。
    pub intent_yaw: Option<f32>,
    /// 一次性跳请求(A02a 第 2 节):置位后在下一个控制拍被消费——不管当时
    /// 接没接地;没接地就丢掉,不缓存。随快照走(检查点落在置位与消费之间
    /// 时,恢复线同样消费)。
    pub intent_jump: bool,
    /// 当前关节目标(长度 = `dof_count`;单连杆为空)。保持状态,直到下一条
    /// 命令;随快照走(B01a:恢复后必须与电机里的目标一致,K10)。
    pub joint_targets: Vec<f32>,
}

/// 主体生成失败原因(类型化拒绝;不 panic)。
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum AgentSpawnError {
    /// 哈希不在注册表(REQ-012 A01 第 3 节:类型化拒绝 + 遥测)。
    #[error("未知身体哈希:{0}")]
    UnknownBody(BodyHash),
    /// 定义校验失败(注册表加载时已校验;防御)。
    #[error("身体定义非法:{0}")]
    InvalidBody(String),
}

/// 主体意图的内核侧拒绝原因(A02a 第 2 节;类型化,计数在 apply_command)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AgentIntentError {
    /// 实体不存在或不是主体。
    NotAgent,
    /// 身体没有 `[walker]` 段(多连杆身体或未配行走的单连杆)。
    NoWalker,
    /// move_xz/yaw 非有限(NaN/inf)。
    NonFinite,
}

/// 失控速度阈值(REQ-012 §6,m/s):任一连杆线速度的模超过它 → 按"掉出
/// 世界"处理。demo `mass` 的结论(REQ-012 §13):panic 的前一拍状态都还
/// 有限、线速度都不低于 2204 m/s,200 m/s 防得住且无误杀(质量 8 峰值
/// 126 m/s、单格方块峰值 68.3 m/s)。常量,不进配置。
pub const AGENT_V_GUARD: f32 = 200.0;

/// 失控原因(B01b 第 3.1 节;`agent_runaway_reason` 的返回)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RunawayReason {
    /// 任一连杆位置/朝向/线速度/角速度,或 multibody 的关节坐标/广义速度
    /// 任一分量非有限(NaN/±inf)。
    NonFinite,
    /// 任一连杆线速度的模 > `AGENT_V_GUARD`。
    OverSpeed,
}

/// 关节目标命令的内核侧拒绝原因(B01a 第 3.4 节;类型化,计数在
/// apply_command 的 `agent_joint_targets_rejected`)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AgentJointTargetsError {
    /// 实体不存在或不是主体。
    NotAgent,
    /// 身体自由度为 0(单连杆胶囊;关节命令只对多连杆身体有意义)。
    NoJoints,
    /// 长度 ≠ `dof_count`。
    LengthMismatch,
    /// 任一项非有限(NaN/inf)。
    NonFinite,
}

impl ShardSim {
    /// 主体是否存在。
    pub fn is_agent(&self, id: EntityId) -> bool {
        self.agents.contains_key(&id)
    }

    /// 全部主体 ID(A06 §6"玩家回出生点";调度器拍前主体账本用):agents
    /// 键序升序。只读观测口,不进快照。
    pub fn agent_ids(&self) -> Vec<EntityId> {
        self.agents.keys().copied().collect()
    }

    /// 主体是否接地(A02a 第 3 节的接地判定;判据布景的接地自检观测口)。
    /// 主体不存在/无碰撞体/几何非胶囊 → false。
    pub fn agent_is_grounded(&self, id: EntityId, ground_eps: f32) -> bool {
        if !self.agents.contains_key(&id) {
            return false;
        }
        let Some(rec) = self.entities.get(&id) else {
            return false;
        };
        let Some(rb) = self.phys.bodies.get(rec.handle) else {
            return false;
        };
        let pos = crate::conv::rapier_to_vec3(rb.translation());
        self.agent_ground_check_pub(rec.handle, rec.collider, pos, ground_eps)
    }

    /// 主体的身体哈希;非主体/不存在 → None。
    pub fn agent_body_hash(&self, id: EntityId) -> Option<BodyHash> {
        self.agents.get(&id).map(|a| a.body_hash)
    }

    /// 身体定义的竖向半包络(REQ-012 A04:调度器在途主体迁移的足迹口径,
    /// 与生成路径 `EntityDesc.half_size` 同源——`agent_half_size`)。
    /// 哈希不在注册表 → None。
    pub fn agent_half_size_of(&self, body: &BodyHash) -> Option<f32> {
        self.bodies.get(body).map(agent_half_size)
    }

    /// 身体各连杆的形状定义引用(A05:调度器给迁移在途主体构造形状表
    /// 条目用;哈希不在注册表 → None)。
    pub fn agent_body_link_defs(&self, body: &BodyHash) -> Option<Vec<&[types::ShapeDef]>> {
        self.bodies
            .get(body)
            .map(|def| def.links.iter().map(|l| l.shapes.as_slice()).collect())
    }

    /// 主体数(测试观测口)。
    pub fn agent_count(&self) -> usize {
        self.agents.len()
    }

    /// 本分片主体的连杆形状表条目(REQ-012 §6"暂缓写入",A05):调度器在
    /// 每拍步进前汇总全部分片的条目(∪ 在途信封)下发给每个分片。位姿 =
    /// 连杆刚体的世界位姿;形状与生成层同一映射。EntityId 升序。
    pub fn agent_shape_entries(&self) -> crate::agent_shape::AgentShapeTable {
        let mut table = crate::agent_shape::AgentShapeTable::default();
        for (&id, record) in &self.agents {
            let def = self
                .bodies
                .get(&record.body_hash)
                .expect("主体身体哈希必在注册表(restore 前置校验,fail-stop)");
            let links = def
                .links
                .iter()
                .filter_map(|link| {
                    record.links.get(&link.name).and_then(|&handle| {
                        self.phys
                            .bodies
                            .get(handle)
                            .map(|rb| crate::agent_shape::AgentLinkEntry {
                                pose: *rb.position(),
                                shape: agent_link_shared_shape(&link.shapes),
                                bound_radius: crate::agent_shape::link_bound_radius(&link.shapes),
                                center: crate::conv::rapier_to_vec3(rb.translation()),
                            })
                    })
                })
                .collect();
            table.links.insert(id, links);
        }
        table
    }

    /// 主体状态流(REQ-012 A03 §1;`S2C::AgentStates` 下发源,调度器逐分片
    /// 收集):`(EntityId, BodyHash, 连杆姿态列表)`;**只含主体**(驱动侧
    /// 与 `entity_states` 分开发送,主体从不出现在方块流里——A01 已过滤)。
    /// 连杆姿态按 `BodyDef.links` 的顺序排(A03 §1:多连杆姿态的顺序契约;
    /// 单连杆 = 1 条)。本分片内 EntityId 升序由调用方合并排序。
    pub fn agent_states(&self) -> Vec<(EntityId, BodyHash, Vec<types::EntityState>)> {
        self.agents
            .iter()
            .map(|(&id, record)| {
                let def = self
                    .bodies
                    .get(&record.body_hash)
                    .expect("主体身体哈希必在注册表(restore 前置校验,fail-stop)");
                let poses = def
                    .links
                    .iter()
                    .filter_map(|link| {
                        record.links.get(&link.name).and_then(|&handle| {
                            self.phys.bodies.get(handle).map(|rb| types::EntityState {
                                translation: crate::conv::rapier_to_vec3(rb.translation()),
                                rotation: crate::conv::rapier_to_quat(rb.rotation()),
                                linvel: crate::conv::rapier_to_vec3(rb.linvel()),
                                angvel: crate::conv::rapier_to_vec3(rb.angvel()),
                            })
                        })
                    })
                    .collect();
                (id, record.body_hash, poses)
            })
            .collect()
    }

    /// 主体根连杆的世界位置(REQ-012 A03 §5 观察者跟随):主体在本分片 →
    /// `Some(根位置)`;不在 → None。根连杆 = `links` 表首条(单连杆即它;
    /// B01 多连杆按 `BodyDef.links[0]` = 根的名字)。
    pub fn agent_root_position(&self, entity: EntityId) -> Option<types::glam::Vec3> {
        let record = self.agents.get(&entity)?;
        let def = self
            .bodies
            .get(&record.body_hash)
            .expect("主体身体哈希必在注册表(fail-stop)");
        let root = def.links.first()?;
        let &handle = record.links.get(&root.name)?;
        self.phys
            .bodies
            .get(handle)
            .map(|rb| crate::conv::rapier_to_vec3(rb.translation()))
    }

    /// 生成主体(REQ-012 §3;A01 第 3 节 / B01a):
    /// - 未知哈希 → `Err(UnknownBody)` + 遥测(`agent_spawn_rejected`);
    /// - 单连杆 = 玩家胶囊路径(`spawn_agent_single`,一行不改地搬);
    /// - 多连杆 = `spawn_agent_multi`(B01a:正运动学摆位、碰撞体、关节
    ///   电机、阻尼缩放、自碰撞关闭);
    /// - 重复 `EntityId` → fail-stop(与 `SpawnDynamic` 同口径 assert);
    /// - 生成位置与实心体素重叠 → 不检查(与 `SpawnDynamic` 同口径:物理
    ///   接触下一步自然解决);
    /// - 质量/惯量乘全局系数 k(`cfg.agent_mass_scale`,§4 默认 1/100);
    /// - `actor` 恒为 true;`outbox.spawned` 照常(调度器 claim ownership);
    /// - `source` 写进实体记录(REQ-012 A03 §2:调度器注入/本地会话 = None,
    ///   网络玩家 = `Some(player)`——`entity_owner(a).source` 指认玩家身体,
    ///   迁移已随实体记录带走;migrate_agent_in 的写回口径与此一致)。
    pub fn spawn_agent(
        &mut self,
        entity: EntityId,
        body: BodyHash,
        state: types::EntityState,
        source: Option<messaging::InputSource>,
    ) -> Result<(), AgentSpawnError> {
        let def = match self.bodies.get(&body) {
            Some(def) => def.clone(),
            None => {
                self.outbox.agent_spawn_rejected += 1;
                return Err(AgentSpawnError::UnknownBody(body));
            }
        };
        if def.links.len() == 1 {
            self.spawn_agent_single(entity, &def, body, state, source)
        } else {
            self.spawn_agent_multi(entity, &def, body, state, source)
        }
    }

    /// 单连杆生成(A01 原路径原样;`links.len() == 1` 已由调用方保证)。
    fn spawn_agent_single(
        &mut self,
        entity: EntityId,
        def: &BodyDef,
        body: BodyHash,
        state: types::EntityState,
        source: Option<messaging::InputSource>,
    ) -> Result<(), AgentSpawnError> {
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
            .restitution(link.restitution)
            // A02a 第 4 节:接触钩子只挂在主体碰撞体上(摩擦清零 + 接缝
            // 修正;钩子本身见 walker.rs——世界里没有主体时无碰撞体挂钩,
            // 钩子不触发,AC-A8)。
            .active_hooks(rapier3d::prelude::ActiveHooks::MODIFY_SOLVER_CONTACTS);
        let (handle, collider) = self.phys.insert(body_builder, collider_builder);

        // 主体描述:凝固路径按 actor 豁免永不使用 `desc.block`;`half_size`
        // 取形状包络半高(唤醒/预算几何的足迹外扩用,保守取胶囊总半高)。
        let desc = types::EntityDesc {
            half_size: agent_half_size(def),
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
                source,
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
                intent_move: None,
                intent_yaw: None,
                intent_jump: false,
                joint_targets: Vec::new(),
            },
        );
        // 调度器据此 claim 全局 ownership(REQ-006 任务 00;恰好一次)。
        self.outbox.spawned.push(entity);
        Ok(())
    }

    /// 多连杆生成(B01a,放行第 3.3 节步骤 1–6):
    /// 1. 正运动学摆位(全部关节角取 0):根位姿 = `state`;子连杆朝向 =
    ///    父朝向,位置 = 父位置 + 父朝向 × `anchor` − 子朝向 × `child_anchor`。
    ///    按 `links` 顺序建刚体,父连杆必须排在子之前(否则 `InvalidBody`)。
    ///    根的线速度/角速度取 `state`,其余连杆速度为零。
    /// 2. 碰撞体:每连杆一个(`agent_collider_builder` 支持形状偏移);
    ///    质量 = `link.mass × k`;**不挂** `MODIFY_SOLVER_CONTACTS` 钩子
    ///    (那是行走控制器用的)。
    /// 3. 关节:按 `joints` 顺序 `insert_multibody_joint`;revolute 电机恒在
    ///    `AngX` 槽(§13 quad demo)、prismatic 用 `LinX`;电机 ForceBased,
    ///    初始目标 = `default_angle`(缺省 0),`max_force = max_torque × k`
    ///    (没给就不设上限)。
    /// 4. 阻尼:`damping_mut()` 每项 × k(§4 硬要求,gait demo R1-5)。
    /// 5. 自碰撞:multibody 级 `set_self_contacts_enabled(false)`;相邻连杆
    ///    由 `contacts_enabled(false)` 关闭。
    /// 6. 登记:同单连杆(EntityRecord/AgentRecord/outbox.spawned)。
    fn spawn_agent_multi(
        &mut self,
        entity: EntityId,
        def: &BodyDef,
        body: BodyHash,
        state: types::EntityState,
        source: Option<messaging::InputSource>,
    ) -> Result<(), AgentSpawnError> {
        use rapier3d::prelude::*;
        if let Err(e) = def.validate() {
            self.outbox.agent_spawn_rejected += 1;
            return Err(AgentSpawnError::InvalidBody(e.to_string()));
        }
        assert!(
            !self.entities.contains_key(&entity) && !self.agents.contains_key(&entity),
            "主体重复 spawn(fail-stop):{entity:?}"
        );
        let k = self.cfg.agent_mass_scale;
        // 名字 → (生成序, 欲填的 handle 槽)。父先于子在 validate/这里强制。
        let mut order: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
        for (i, link) in def.links.iter().enumerate() {
            order.insert(link.name.as_str(), i);
        }
        let mut handles: Vec<Option<RigidBodyHandle>> = vec![None; def.links.len()];
        let mut colliders: Vec<ColliderHandle> = Vec::new();

        // ---- 步骤 1:按 links 顺序建刚体(父在子之前)----
        for (i, link) in def.links.iter().enumerate() {
            // 正运动学摆位:关节角全 0。子连杆朝向 = 父朝向;位置 = 父位置
            // + 父朝向 × anchor − 子朝向 × child_anchor。i = 0(根)时位姿 =
            // state 本身。
            let (pose, linvel, angvel) = if i == 0 {
                (
                    crate::conv::pose_to_rapier(state.translation, state.rotation),
                    crate::conv::vec3_to_rapier(state.linvel),
                    crate::conv::vec3_to_rapier(state.angvel),
                )
            } else {
                // 找把本连杆当 child 的关节(树形:恰一条)。
                let j = def
                    .joints
                    .iter()
                    .find(|j| j.child == link.name)
                    .expect("非根连杆必为某关节的 child(validate 保证)");
                let parent_i = order[j.parent.as_str()];
                let parent_handle = handles[parent_i].expect("父连杆在 links 里排在子之前");
                let parent_rb = self.phys.bodies.get(parent_handle).expect("父连杆刚体已建");
                let parent_pos = parent_rb.translation();
                let parent_rot = parent_rb.rotation();
                let p = j.kind.params();
                let anchor = crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(p.anchor));
                let child_anchor =
                    crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(p.child_anchor));
                // 朝向 = 父朝向(关节角 0 时父子同向)。
                let pos = parent_pos + parent_rot * anchor - parent_rot * child_anchor;
                (
                    Pose {
                        translation: pos,
                        rotation: *parent_rot,
                        ..Pose::IDENTITY
                    },
                    Vector::ZERO,
                    Vector::ZERO,
                )
            };
            let mut body_builder = RigidBodyBuilder::dynamic()
                .pose(pose)
                .linvel(linvel)
                .angvel(angvel);
            if def.lock_rotations {
                body_builder = body_builder.lock_rotations();
            }
            let handle = self.phys.insert_body(body_builder);
            handles[i] = Some(handle);
            // ---- 步骤 2:碰撞体(每连杆一个;质量 × k;不挂钩子)----
            let mass = link.mass * k;
            let mut collider_builder = agent_collider_builder(&link.shapes, mass);
            collider_builder = collider_builder
                .friction(link.friction)
                .restitution(link.restitution);
            let collider = self.phys.colliders.insert_with_parent(
                collider_builder,
                handle,
                &mut self.phys.bodies,
            );
            colliders.push(collider);
        }
        let root_handle = handles[0].expect("根连杆已建");

        // ---- 步骤 3:关节与电机 ----
        // dof 顺序 = joints 里非 fixed 的顺序;关节目标表按该顺序建。
        let mut joint_targets: Vec<f32> = Vec::new();
        for j in &def.joints {
            let parent_i = order[j.parent.as_str()];
            let child_i = order[j.child.as_str()];
            let (parent_handle, child_handle) = (
                handles[parent_i].expect("父已建(顺序保证)"),
                handles[child_i].expect("子已建"),
            );
            let p = j.kind.params();
            // 三种 builder 类型不同,各自 build 成 GenericJoint 后统一走电机
            // 段与 insert(demo 口径:RevoluteJointBuilder / FixedJointBuilder)。
            let built: GenericJoint = match j.kind {
                types::JointKind::Revolute(_) => {
                    let mut b = RevoluteJointBuilder::new(crate::conv::vec3_to_rapier(
                        types::glam::Vec3::from_array(p.axis),
                    ))
                    .local_anchor1(crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(
                        p.anchor,
                    )))
                    .local_anchor2(crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(
                        p.child_anchor,
                    )))
                    .contacts_enabled(false);
                    if let Some(lim) = p.limits {
                        b = b.limits([lim.lower, lim.upper]);
                    }
                    b.build().into()
                }
                types::JointKind::Prismatic(_) => {
                    let mut b = PrismaticJointBuilder::new(crate::conv::vec3_to_rapier(
                        types::glam::Vec3::from_array(p.axis),
                    ))
                    .local_anchor1(crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(
                        p.anchor,
                    )))
                    .local_anchor2(crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(
                        p.child_anchor,
                    )))
                    .contacts_enabled(false);
                    if let Some(lim) = p.limits {
                        b = b.limits([lim.lower, lim.upper]);
                    }
                    b.build().into()
                }
                types::JointKind::Fixed(_) => FixedJointBuilder::new()
                    .local_anchor1(crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(
                        p.anchor,
                    )))
                    .local_anchor2(crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(
                        p.child_anchor,
                    )))
                    .contacts_enabled(false)
                    .build()
                    .into(),
            };
            // 电机:目标 = default_angle(缺省 0)、stiffness = kp × k、
            // damping = kd × k;ForceBased;max_force = max_torque × k
            // (没给就不设上限)。revolute 电机恒在 AngX 槽(§13 探针证据),
            // prismatic 用 LinX;fixed 无自由度、不设电机。
            let axis = match j.kind.joint_type() {
                types::JointType::Revolute => JointAxis::AngX,
                types::JointType::Prismatic => JointAxis::LinX,
                types::JointType::Fixed => JointAxis::AngX, // 无自由度,不消费
            };
            let default_target = p.default_angle.unwrap_or(0.0);
            let mut built = built;
            if j.kind.joint_type() != types::JointType::Fixed {
                built.set_motor(
                    axis,
                    default_target,
                    0.0,
                    p.kp.unwrap_or(0.0) * k,
                    p.kd.unwrap_or(0.0) * k,
                );
                built.set_motor_model(axis, MotorModel::ForceBased);
                if let Some(tau) = p.max_torque {
                    built.motors[axis as usize].max_force = tau * k;
                }
                joint_targets.push(default_target);
            }
            let inserted = self
                .phys
                .insert_multibody_joint(parent_handle, child_handle, built);
            assert!(
                inserted.is_some(),
                "insert_multibody_joint 失败(fail-stop):{} -> {}",
                j.parent,
                j.child
            );
        }

        // ---- 步骤 4:阻尼 × k(§4 硬要求)----
        // 全部关节插完后,3 根腿的关节并进同一 multibody(12 个关节)。取
        // 根连杆所属 multibody(gait demo R1-5 同款)。
        let link_id = self
            .phys
            .multibody_joints
            .rigid_body_link(root_handle)
            .expect("根连杆已在 multibody 中(fail-stop)");
        let mb_index = link_id.multibody;
        {
            let mb = self
                .phys
                .multibody_joints
                .get_multibody_mut(mb_index)
                .expect("multibody 存在(fail-stop)");
            for d in mb.damping_mut().iter_mut() {
                *d *= k;
            }
            // ---- 步骤 5:自碰撞全关(B01a 不支持按对开启,validate 已拒)----
            mb.set_self_contacts_enabled(false);
        }

        // ---- 步骤 6:登记(同单连杆)----
        let desc = types::EntityDesc {
            half_size: agent_half_size(def),
            block: types::BlockId(1),
            shape: None,
        };
        let mut links = std::collections::BTreeMap::new();
        for (i, link) in def.links.iter().enumerate() {
            links.insert(link.name.clone(), handles[i].expect("全部连杆已建"));
        }
        self.entities.insert(
            entity,
            crate::sim::EntityRecord {
                handle: root_handle,
                collider: colliders[0],
                desc,
                slow_ticks: 0,
                actor: true,
                source,
                window: std::collections::VecDeque::new(),
                carried: false,
                at_rest: false,
                actor_motion_ticks: None,
            },
        );
        self.agents.insert(
            entity,
            AgentRecord {
                body_hash: body,
                links,
                colliders,
                intent_move: None,
                intent_yaw: None,
                intent_jump: false,
                joint_targets,
            },
        );
        self.outbox.spawned.push(entity);
        Ok(())
    }

    /// 迁移重建入口(REQ-012 A04 放行第 2 节):按身体哈希走 `spawn_agent`
    /// 同一条生成路径(碰撞体挂钩子标志、`actor = true`、登记进 `agents`、
    /// `outbox.spawned` 照常),再写回控制器保持状态(意图 + `WalkerHold`)
    /// 与常驻运动票触顶标记(04a 遗留返修同款:计数连续,不借迁移绕上限)。
    /// **不施加力**——迁移那一拍的施力由本拍 `step` 内的控制器段照常进行
    /// (60 Hz 施力先 `reset_forces` 再施加保持力;§13 `migrate`:写回必须
    /// 在下一次步进之前,否则失力 1–2 拍)。单连杆预留字段非空 = 内部一致
    /// 性违规 fail-stop(B01 才带关节状态)。`source` 写回(R1-P8:A03 玩家
    /// 生成主体后迁移不能丢来源;实体归属表的 `EntityOwner.source` 由调度器
    /// 按 outbox 口径维护,此处对齐 EntityRecord)。
    #[allow(clippy::too_many_arguments)] // 信封字段逐项透传(B01 前恒空三项)
    pub(crate) fn migrate_agent_in(
        &mut self,
        entity: EntityId,
        body: BodyHash,
        state: types::EntityState,
        intent_move: Option<(f32, f32)>,
        intent_yaw: Option<f32>,
        intent_jump: bool,
        held_force: (f32, f32),
        last_yaw: f32,
        source: Option<messaging::InputSource>,
        actor_motion_ticks: Option<u64>,
        joint_q: &[f32],
        joint_qd: &[f32],
        last_targets: &[f32],
    ) {
        // B01 预留字段:单连杆一律为空;非空 = 内部一致性违规(B01 之前
        // 不可能产出,信封来源只可能是同版本内核)。
        assert!(
            joint_q.is_empty() && joint_qd.is_empty() && last_targets.is_empty(),
            "MigrateAgent 带多连杆关节状态(单连杆恒空,内部一致性违规,fail-stop)"
        );
        self.spawn_agent(entity, body, state, source)
            .expect("MigrateAgent 的身体哈希必在注册表(接收方 fail-stop 前置已查)");
        let record = self
            .agents
            .get_mut(&entity)
            .expect("spawn_agent 刚登记(fail-stop)");
        record.intent_move = intent_move;
        record.intent_yaw = intent_yaw;
        record.intent_jump = intent_jump;
        if let Some(rec) = self.entities.get_mut(&entity) {
            rec.actor_motion_ticks = actor_motion_ticks;
        }
        self.walker_holds.insert(
            entity,
            crate::walker::WalkerHold {
                held_force,
                last_yaw,
            },
        );
    }

    /// 移除主体(REQ-012 A01 第 3 节:刚体、碰撞体、`collider_to_entity`
    /// (拍内重建表,无需清理)、`agents` 全部回到生成前状态)。
    /// B01a:移除**全部连杆**刚体(multibody 关节随刚体移除一并拆),
    /// 移除后刚体数、碰撞体数、multibody 数回到生成前(K2)。
    /// 主体不存在 → panic(fail-stop;调用方先判 `is_agent`)。
    pub fn despawn_agent(&mut self, id: EntityId) {
        let record = self.agents.remove(&id).expect("主体存在(fail-stop)");
        self.entities.remove(&id).expect("实体记录存在(fail-stop)");
        // 全部连杆逐个 remove_body(挂它的碰撞体与关节随之拆除;根连杆也走
        // 同一路径)。links 表按名字序,与生成序无关——拆刚体之间没有顺序
        // 依赖(rapier remove_body 自动处理悬挂关节引用)。
        for handle in record.links.values().copied().collect::<Vec<_>>() {
            self.phys.remove_body(handle);
        }
        // A02a:控制器保持态随主体增删(移除后不残留)。
        self.walker_holds.remove(&id);
        // 调度器据此 release 全局 ownership(REQ-006 任务 00)。
        self.outbox.despawned.push(id);
    }

    /// 主体意图的内核侧接受(A02a 第 2 节):命令可能来自日志回放,内核
    /// **不信任它、不 panic**。检查(任一失败即 `Err`,类型见
    /// [`AgentIntentError`]):
    /// - 实体不存在/不是主体;
    /// - 无 `[walker]` 段;
    /// - 非有限值(NaN/inf;保持状态不写入)。
    ///
    /// 通过后:`move_xz` 长度超过 `[walker].max_speed` 按比例钳(遥测
    /// `agent_intent_clamped` +1)、`yaw` 规范化到 (−π, π],合法意图写入
    /// `AgentRecord` 保持状态(`intent_move`/`intent_yaw`);`jump` 置一次性
    /// 标志,由控制器在下一个控制拍消费。遥测拒绝计数在 `apply_command`
    /// 分支统一 +1(本函数只报类型)。
    pub(crate) fn apply_agent_intent(
        &mut self,
        entity: EntityId,
        move_xz: (f32, f32),
        yaw: f32,
        jump: bool,
    ) -> Result<(), AgentIntentError> {
        let body_hash = match self.agents.get(&entity) {
            Some(record) => record.body_hash,
            None => return Err(AgentIntentError::NotAgent),
        };
        let Some(walker) = self.bodies.get(&body_hash).and_then(|def| def.walker) else {
            return Err(AgentIntentError::NoWalker);
        };
        let (mx, mz) = move_xz;
        if !mx.is_finite() || !mz.is_finite() || !yaw.is_finite() {
            return Err(AgentIntentError::NonFinite);
        }
        // 钳制:长度超过 max_speed 按比例缩(方向不变)。
        let len = (mx * mx + mz * mz).sqrt();
        let (mx, mz) = if len > walker.max_speed {
            let s = walker.max_speed / len;
            self.outbox.agent_intent_clamped += 1;
            (mx * s, mz * s)
        } else {
            (mx, mz)
        };
        // yaw 规范化到 (−π, π](§13:与 capsule demo 的 heading_rad 同域;
        // rem_euclid 到 [0, 2π) 再折回)。
        let yaw_norm = {
            let two_pi = std::f32::consts::TAU;
            let mut y = yaw.rem_euclid(two_pi);
            if y > std::f32::consts::PI {
                y -= two_pi;
            }
            // rem_euclid 后 y ∈ [0, 2π);y = 0 保持 0((−π, π] 的 0);
            // y = 2π 不可能出现(rem_euclid 半开区间),负零折正零不强制
            // (f32 位模式:−0.0 − 2π ≠ 0 路径不会到;0.0 走不到减法)。
            y
        };
        let record = self.agents.get_mut(&entity).expect("前面已查存在");
        record.intent_move = Some((mx, mz));
        record.intent_yaw = Some(yaw_norm);
        record.intent_jump = jump;
        Ok(())
    }

    /// 关节目标命令的内核侧接受(B01a,放行第 3.4 节):命令可能来自日志
    /// 回放,内核**不信任它、不 panic**。检查(任一失败即 `Err`,世界不变):
    /// - 实体不存在 / 不是主体(`NotAgent`);
    /// - `dof_count == 0`(单连杆胶囊,`NoJoints`);
    /// - 长度 ≠ `dof_count`(`LengthMismatch`);
    /// - 任一项非有限(`NonFinite`)。
    ///
    /// 通过后:有 `limits` 的关节目标夹到 `[lower, upper]`(每夹一个返回
    /// 计数 +1,遥测 `agent_joint_targets_clamped` 由 apply_command 段加);
    /// 写进 `record.joint_targets`,逐关节更新电机目标(增益与上限不变);
    /// 任一目标与原值差 > 1e-6 时唤醒整只主体的全部连杆(§13:睡着后 island
    /// 不解电机约束,目标变化必须唤醒)。返回值 = 被钳制的个数。
    pub(crate) fn apply_agent_joint_targets(
        &mut self,
        entity: EntityId,
        targets: &[f32],
    ) -> Result<u32, AgentJointTargetsError> {
        let body_hash = match self.agents.get(&entity) {
            Some(record) => record.body_hash,
            None => return Err(AgentJointTargetsError::NotAgent),
        };
        let def = self
            .bodies
            .get(&body_hash)
            .expect("主体身体哈希必在注册表(fail-stop)");
        let dof = def.dof_count();
        if dof == 0 {
            return Err(AgentJointTargetsError::NoJoints);
        }
        if targets.len() != dof {
            return Err(AgentJointTargetsError::LengthMismatch);
        }
        if targets.iter().any(|t| !t.is_finite()) {
            return Err(AgentJointTargetsError::NonFinite);
        }
        // 钳制:有 limits 的关节把目标夹进 [lower, upper]。计被钳个数。
        let mut clamped = 0u32;
        let mut clamped_targets: Vec<f32> = Vec::with_capacity(dof);
        for (j, &t) in def.dof_joints().zip(targets) {
            match j.kind.params().limits {
                Some(lim) if t < lim.lower => {
                    clamped_targets.push(lim.lower);
                    clamped += 1;
                }
                Some(lim) if t > lim.upper => {
                    clamped_targets.push(lim.upper);
                    clamped += 1;
                }
                _ => clamped_targets.push(t),
            }
        }
        // 唤醒判定:任一目标与原值差 > 1e-6(与 quad demo `set_target` 同阈值)。
        let record = self.agents.get_mut(&entity).expect("前面已查存在");
        let changed = record.joint_targets.len() != dof
            || record
                .joint_targets
                .iter()
                .zip(&clamped_targets)
                .any(|(a, b)| (a - b).abs() > 1e-6);
        record.joint_targets = clamped_targets;
        // 逐关节写电机目标(**只动 target_pos**,增益与上限不变)。先按
        // dof 顺序收齐各关节 child 刚体在 multibody 里的 link 下标(不可变
        // 查询),再可变借 multibody 写;两个查询段不重叠。
        let child_link_ids: Vec<Option<usize>> = def
            .dof_joints()
            .map(|j| {
                record
                    .links
                    .get(&j.child)
                    .and_then(|&h| self.phys.multibody_joints.rigid_body_link(h))
                    .map(|lid| lid.id)
            })
            .collect();
        let mb_index = record
            .links
            .values()
            .find_map(|&h| self.phys.multibody_joints.rigid_body_link(h))
            .map(|lid| lid.multibody);
        if let Some(mb_index) = mb_index {
            let root_handle = self
                .entities
                .get(&entity)
                .map(|rec| rec.handle)
                .expect("主体 EntityRecord 存在(fail-stop)");
            let targets = record.joint_targets.clone();
            let mb = self
                .phys
                .multibody_joints
                .get_multibody_mut(mb_index)
                .expect("multibody 存在(fail-stop)");
            for (dof_i, j) in def.dof_joints().enumerate() {
                let Some(link_id) = child_link_ids[dof_i] else {
                    continue;
                };
                let Some(link) = mb.link_mut(link_id) else {
                    continue;
                };
                let axis = match j.kind.joint_type() {
                    types::JointType::Revolute => rapier3d::prelude::JointAxis::AngX,
                    types::JointType::Prismatic => rapier3d::prelude::JointAxis::LinX,
                    types::JointType::Fixed => continue,
                };
                link.joint.data.motors[axis as usize].target_pos = targets[dof_i];
            }
            // 唤醒:目标变化 → 整只主体全部连杆醒来(§13:电机约束只在
            // island 解算时生效;借用已还,查表改用收集好的句柄)。
            if changed {
                let handles: Vec<rapier3d::prelude::RigidBodyHandle> = mb
                    .links()
                    .map(|l| l.rigid_body_handle())
                    .chain(std::iter::once(root_handle))
                    .collect();
                for h in handles {
                    if let Some(rb) = self.phys.bodies.get_mut(h) {
                        rb.wake_up(true);
                    }
                }
            }
        }
        Ok(clamped)
    }

    /// 主体关节位置(自由度顺序;B01a 只读观测口)。非主体 / 无关节 → None。
    pub fn agent_joint_positions(&self, id: EntityId) -> Option<Vec<f32>> {
        let record = self.agents.get(&id)?;
        let def = self
            .bodies
            .get(&record.body_hash)
            .expect("主体身体哈希必在注册表(fail-stop)");
        if def.dof_count() == 0 {
            return None;
        }
        let mb_index = self.agent_multibody_index(record)?;
        let mb = self
            .phys
            .multibody_joints
            .get_multibody(mb_index)
            .expect("multibody 存在(fail-stop)");
        Some(
            def.dof_joints()
                .filter_map(|j| {
                    let &handle = record.links.get(&j.child)?;
                    let link_id = self.phys.multibody_joints.rigid_body_link(handle)?;
                    let link = mb.link(link_id.id)?;
                    let axis = match j.kind.joint_type() {
                        types::JointType::Revolute => rapier3d::prelude::JointAxis::AngX,
                        types::JointType::Prismatic => rapier3d::prelude::JointAxis::LinX,
                        types::JointType::Fixed => return None,
                    };
                    Some(link.joint.coords()[axis as usize])
                })
                .collect(),
        )
    }

    /// 主体关节速度(自由度顺序;rad/s 或 m/s)。非主体 / 无关节 → None。
    pub fn agent_joint_velocities(&self, id: EntityId) -> Option<Vec<f32>> {
        let record = self.agents.get(&id)?;
        let def = self
            .bodies
            .get(&record.body_hash)
            .expect("主体身体哈希必在注册表(fail-stop)");
        if def.dof_count() == 0 {
            return None;
        }
        let mb_index = self.agent_multibody_index(record)?;
        let mb = self
            .phys
            .multibody_joints
            .get_multibody(mb_index)
            .expect("multibody 存在(fail-stop)");
        Some(
            def.dof_joints()
                .filter_map(|j| {
                    let &handle = record.links.get(&j.child)?;
                    let link_id = self.phys.multibody_joints.rigid_body_link(handle)?;
                    let link = mb.link(link_id.id)?;
                    let axis = match j.kind.joint_type() {
                        types::JointType::Revolute => rapier3d::prelude::JointAxis::AngX,
                        types::JointType::Prismatic => rapier3d::prelude::JointAxis::LinX,
                        types::JointType::Fixed => return None,
                    };
                    let _ = axis; // 单自由度关节:速度向量的第 0 项即该轴(demo §13 同款)
                    Some(mb.joint_velocity(link)[0])
                })
                .collect(),
        )
    }

    /// 主体当前关节目标(自由度顺序;命令保持状态)。非主体 → None。
    pub fn agent_joint_targets(&self, id: EntityId) -> Option<&[f32]> {
        self.agents.get(&id).map(|r| r.joint_targets.as_slice())
    }

    /// 主体的 multibody 索引(按 links 表反查任意连杆;都不在 multibody 里
    /// → None;单连杆胶囊恒 None)。
    fn agent_multibody_index(
        &self,
        record: &AgentRecord,
    ) -> Option<rapier3d::prelude::MultibodyIndex> {
        for handle in record.links.values() {
            if let Some(lid) = self.phys.multibody_joints.rigid_body_link(*handle) {
                return Some(lid.multibody);
            }
        }
        None
    }

    /// 失控判定(B01b 第 3.2 节;纯读)。检查顺序:
    /// 1. 每个**连杆**(按 `BodyDef.links` 的顺序;单连杆胶囊 = 1 条)的
    ///    位置、朝向、线速度、角速度,任一分量非有限 → `NonFinite`;
    /// 2. 多连杆:multibody 的关节坐标、广义速度任一非有限 → `NonFinite`;
    /// 3. 任一连杆线速度的模 > `AGENT_V_GUARD` → `OverSpeed`。
    ///
    /// 命中即返回;全部通过 → `None`。主体不存在 → `None`(防护段在
    /// agents 表上迭代,不会以不存在的 ID 调用)。
    pub(crate) fn agent_runaway_reason(&self, id: EntityId) -> Option<RunawayReason> {
        let record = self.agents.get(&id)?;
        let def = self
            .bodies
            .get(&record.body_hash)
            .expect("主体身体哈希必在注册表(fail-stop)");
        // 1. 逐连杆刚体状态(links 按名字序存句柄,按 BodyDef.links 的顺序查)。
        for link in &def.links {
            let &handle = record
                .links
                .get(&link.name)
                .expect("连杆句柄在登记表(fail-stop)");
            let rb = self.phys.bodies.get(handle).expect("连杆刚体存在");
            let p = crate::conv::rapier_to_vec3(rb.translation());
            let r = rb.rotation();
            let v = crate::conv::rapier_to_vec3(rb.linvel());
            let av = crate::conv::rapier_to_vec3(rb.angvel());
            if ![
                p.x, p.y, p.z, r.x, r.y, r.z, r.w, v.x, v.y, v.z, av.x, av.y, av.z,
            ]
            .iter()
            .all(|c| c.is_finite())
            {
                return Some(RunawayReason::NonFinite);
            }
            // 3. 超速(在非有限之后:NaN 比较恒 false,顺序保证 NonFinite 优先)。
            if v.length() > AGENT_V_GUARD {
                return Some(RunawayReason::OverSpeed);
            }
        }
        // 2. multibody 关节坐标与广义速度(单连杆无 multibody,跳过)。
        if def.dof_count() > 0 {
            let mb_index = self
                .agent_multibody_index(record)
                .expect("多连杆主体必有 multibody(fail-stop)");
            let mb = self
                .phys
                .multibody_joints
                .get_multibody(mb_index)
                .expect("multibody 存在(fail-stop)");
            for link in mb.links() {
                if link.joint.coords().iter().any(|c| !c.is_finite()) {
                    return Some(RunawayReason::NonFinite);
                }
            }
            if mb.generalized_velocity().iter().any(|v| !v.is_finite()) {
                return Some(RunawayReason::NonFinite);
            }
        }
        None
    }

    /// 虚空移除与失控共用的主体移除路径(B01b 第 3.2 节):拆全部连杆刚体
    /// (关节随刚体移除一并拆)、清 `agents`/`entities`/`walker_holds`/
    /// `locomotion`/`forced_pending`/`forced_reasons`,推 `outbox.despawned`
    /// 与 `outbox.void_removed`,`activation_events.void_removals += 1`。
    /// 语义与 `run_transient_lifecycle` 虚空段的主体分支逐句一致(B01b 第
    /// 1 步原样提出,行为不变)。主体不存在 → panic(fail-stop;调用方
    /// 在 agents 表上迭代)。
    pub(crate) fn remove_agent_as_void(&mut self, id: EntityId) {
        if let Some(agent) = self.agents.remove(&id) {
            for handle in agent.links.values().copied().collect::<Vec<_>>() {
                self.phys.remove_body(handle);
            }
            self.walker_holds.remove(&id);
            self.entities.remove(&id).expect("存在");
            self.locomotion.remove(&id);
            self.forced_pending.remove(&id);
            self.forced_reasons.remove(&id);
            self.outbox.despawned.push(id);
            self.outbox.void_removed.push(id);
            // 06c 遥测:虚空移除只计 void_removals(§4.16)。
            self.activation_events.void_removals += 1;
        } else {
            panic!("remove_agent_as_void:主体不存在(fail-stop):{id:?}");
        }
    }

    /// 步进后的失控防护(B01b 第 3.2/3.3 节):每个主体过一遍
    /// [`ShardSim::agent_runaway_reason`],命中即按"掉出世界"移除
    /// ( [`ShardSim::remove_agent_as_void`] + `outbox.agent_runaway_removed`
    /// 计数)。无主体时立即返回。调用点 = `step()` 内 `phys.step_with_events`
    /// 之后、`collect_contact_wake_compensation()` 之前——失控主体在被移除
    /// 后不再进入本拍余下各段(撞击采样等)。
    pub(crate) fn agent_runaway_guard(&mut self) {
        if self.agents.is_empty() {
            return;
        }
        let ids: Vec<EntityId> = self.agents.keys().copied().collect();
        for id in ids {
            if self.agent_runaway_reason(id).is_some() {
                self.remove_agent_as_void(id);
                self.outbox.agent_runaway_removed += 1;
            }
        }
    }
}

/// 按形状定义构造 collider(REQ-012 §2.1:盒、球、胶囊、凸包;多形状 =
/// compound)。质量显式给定(真实单位 × k)。B01a:每个形状带 `offset`
/// (连杆局部系平移;单形状时同样生效——quad 脚球 = shank 第二形状带偏移,
/// thigh 等无偏移形状的 builder 路径不变)。
pub(crate) fn agent_collider_builder(
    shapes: &[types::ShapeDef],
    mass: f32,
) -> rapier3d::prelude::ColliderBuilder {
    use rapier3d::prelude::*;
    if shapes.len() == 1 {
        let (builder, offset) = shape_collider(&shapes[0], mass);
        return builder.translation(crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(
            offset,
        )));
    }
    let parts: Vec<(Pose, SharedShape)> = shapes
        .iter()
        .map(|s| {
            let (shape, offset) = shape_shared(s);
            (shape_offset_pose(offset), shape)
        })
        .collect();
    ColliderBuilder::compound(parts).mass(mass)
}

/// 形状偏移 `[f32;3]` → parry `Pose`(纯平移;形状 offset 挂在连杆局部系)。
fn shape_offset_pose(offset: [f32; 3]) -> rapier3d::prelude::Pose {
    rapier3d::prelude::Pose {
        rotation: rapier3d::prelude::Rotation::IDENTITY,
        translation: crate::conv::vec3_to_rapier(types::glam::Vec3::from_array(offset)),
        ..rapier3d::prelude::Pose::IDENTITY
    }
}

/// 形状定义 → (共享形状, 连杆局部系偏移)。与 [`agent_collider_builder`]
/// 同源;单一出口保证生成与形状表(A05)几何一致。
fn shape_shared(s: &types::ShapeDef) -> (rapier3d::prelude::SharedShape, [f32; 3]) {
    use rapier3d::prelude::*;
    match s {
        types::ShapeDef::Box { half, offset } => {
            (SharedShape::cuboid(half[0], half[1], half[2]), *offset)
        }
        types::ShapeDef::Sphere { radius, offset } => (SharedShape::ball(*radius), *offset),
        types::ShapeDef::CapsuleY {
            half_height,
            radius,
            offset,
        } => (SharedShape::capsule_y(*half_height, *radius), *offset),
        types::ShapeDef::ConvexHull { points, offset } => {
            let pts: Vec<rapier3d::prelude::Vector> =
                points.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
            (
                SharedShape::convex_hull(&pts).expect("凸包顶点合法(校验保证非退化)"),
                *offset,
            )
        }
    }
}

fn shape_collider(
    s: &types::ShapeDef,
    mass: f32,
) -> (rapier3d::prelude::ColliderBuilder, [f32; 3]) {
    use rapier3d::prelude::*;
    let (offset, builder) = match s {
        types::ShapeDef::Box { half, offset } => {
            (*offset, ColliderBuilder::cuboid(half[0], half[1], half[2]))
        }
        types::ShapeDef::Sphere { radius, offset } => (*offset, ColliderBuilder::ball(*radius)),
        types::ShapeDef::CapsuleY {
            half_height,
            radius,
            offset,
        } => (*offset, ColliderBuilder::capsule_y(*half_height, *radius)),
        types::ShapeDef::ConvexHull { points, offset } => {
            let pts: Vec<rapier3d::prelude::Vector> =
                points.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
            (
                *offset,
                ColliderBuilder::convex_hull(&pts).expect("凸包顶点合法(校验保证非退化)"),
            )
        }
    };
    (builder.mass(mass), offset)
}

/// 连杆形状的 parry 共享形状(A05 形状表用):与 [`agent_collider_builder`]
/// / [`shape_collider`] 同一映射,只取形状部分、不带质量/材质——重叠判定
/// 只关心几何。放在 agent_shape.rs 会反向依赖生成层细节,这里与生成层同源。
/// 连杆形状的 parry 共享形状(A05 形状表用):与 [`agent_collider_builder`]
/// / [`shape_collider`] 同一映射,只取形状部分、不带质量/材质——重叠判定
/// 只关心几何。`pub` 供 agent_shape 的信封条目构造复用(同源保证)。
pub fn agent_link_shared_shape_pub(shapes: &[types::ShapeDef]) -> rapier3d::prelude::SharedShape {
    agent_link_shared_shape(shapes)
}

pub(crate) fn agent_link_shared_shape(
    shapes: &[types::ShapeDef],
) -> rapier3d::prelude::SharedShape {
    use rapier3d::prelude::*;
    if shapes.len() == 1 {
        let (shape, offset) = shape_shared(&shapes[0]);
        if offset == [0.0; 3] {
            return shape;
        }
        return SharedShape::compound(vec![(shape_offset_pose(offset), shape)]);
    }
    let parts: Vec<(Pose, SharedShape)> = shapes
        .iter()
        .map(|s| {
            let (shape, offset) = shape_shared(s);
            (shape_offset_pose(offset), shape)
        })
        .collect();
    SharedShape::compound(parts)
}

/// 主体描述的 half_size:形状最大竖向半包络(足迹/唤醒几何的保守包络;
/// `envelope_radius = half·√3` ≥ 半高即可)。胶囊 = half_height + radius;
/// 盒/球/凸包取最大轴或半径,加形状 `offset` 的模(B01a:连杆相对根的
/// 偏移由生成位姿承载,这里只加形状在连杆内的偏移,quad 脚球 +0.1065)。
pub(crate) fn agent_half_size(def: &BodyDef) -> f32 {
    def.links
        .iter()
        .flat_map(|l| l.shapes.iter())
        .map(|s| {
            let (bound, offset): (f32, [f32; 3]) = match s {
                types::ShapeDef::Box { half, offset } => (
                    (half[0].abs().max(half[1].abs())).max(half[2].abs()),
                    *offset,
                ),
                types::ShapeDef::Sphere { radius, offset } => (radius.abs(), *offset),
                types::ShapeDef::CapsuleY {
                    half_height,
                    radius,
                    offset,
                } => (half_height.abs() + radius.abs(), *offset),
                types::ShapeDef::ConvexHull { points, offset } => (
                    points
                        .iter()
                        .map(|p| p.iter().fold(0.0f32, |m, v| m.max(v.abs())))
                        .fold(0.0f32, f32::max),
                    *offset,
                ),
            };
            bound + offset.iter().fold(0.0f32, |m, v| m.max(v.abs()))
        })
        .fold(0.5f32, f32::max)
}

#[cfg(test)]
pub(crate) mod tests {
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

    /// 测试用配置(默认凝固配置,REQ-012 A01 R1-P1:判据布景凝固开启;
    /// 需要关闭的用例自行覆写)+ capsule 注册表。
    pub(crate) fn test_sim() -> (crate::sim::ShardSim, BodyHash) {
        let cfg = crate::sim::ShardSimConfig::new(shard_id(0, 0));
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

    /// 装载后的实心方块数(基准 2304 = 3×3 chunk × 16×16 地面层;R1-P1:
    /// "体素计数不变"按**实心方块**数,不按 chunk 数——凝固写方块不新建
    /// chunk,chunk 计数测不到)。
    fn count_solid_blocks(sim: &crate::sim::ShardSim) -> usize {
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

    /// SpawnAgent 的盖章输入(REQ-012 A01 R1-P3b:AC-A6/判据布景的主体生成
    /// 走**命令路径**(apply_prefix → apply_command → spawn_agent),不再直调;
    /// SCHEDULER 来源 = 服务端注入,与 SpawnDynamic 测试同款)。
    pub(crate) fn spawn_agent_input(
        entity: types::EntityId,
        body: BodyHash,
        pos: Vec3,
    ) -> messaging::StampedInput {
        messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 0,
            payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnAgent {
                entity,
                body,
                state: types::EntityState {
                    translation: pos,
                    ..types::EntityState::default()
                },
            }),
        }
    }

    /// SpawnDynamic 盖章输入(对照布景;sim.rs 测试 spawn_input 同款)。
    fn spawn_dynamic_input(
        entity: types::EntityId,
        pos: Vec3,
        seq: u64,
    ) -> messaging::StampedInput {
        messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                entity,
                state: types::EntityState {
                    translation: pos,
                    ..types::EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }
    }

    /// MarkActor 常驻化盖章输入(sim.rs 测试 mark_actor_input 同款)。
    fn mark_actor_input(entity: types::EntityId, seq: u64) -> messaging::StampedInput {
        messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: messaging::InputPayload::Command(messaging::SimCommand::MarkActor {
                entity,
                actor: true,
            }),
        }
    }

    /// 生产节奏的一拍(调度器 worker 同序:scheduler.rs apply_prefix → step →
    /// run_transient_lifecycle → take_outbox;激活刷新每拍先行——scheduler.
    /// tick 里 activation_enabled 时每分片逐拍 apply_activation_tickets)。
    fn production_tick(sim: &mut crate::sim::ShardSim, observers: &[Vec3]) {
        if !observers.is_empty() {
            sim.apply_activation(observers);
        }
        sim.step();
        sim.run_transient_lifecycle();
        let _ = sim.take_outbox();
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
            None,
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

    /// 判据 A01-1(R1-P1 重做):**默认凝固配置** + 生产节奏(激活开启、有
    /// 观察者、每拍 step → run_transient_lifecycle → take_outbox)静止 3600
    /// 拍——主体还在、实心方块数不变、下沉/漂移 ≤ 1 cm;期间不加任何控制力。
    /// 主体生成走命令路径(SpawnAgent 盖章输入)。
    #[test]
    fn agent_stands_3600_ticks_on_flat_ground() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        let baseline_solid = count_solid_blocks(&s);
        s.apply_prefix(&[spawn_agent_input(a, hash, Vec3::new(24.0, 1.95, 24.0))]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        assert!(s.is_agent(a), "命令路径生成主体成功(布景自检)");
        for _ in 0..3600 {
            production_tick(&mut s, &observers);
        }
        let st = s.entity_state(a).expect("主体还在(不移除、不凝固)");
        assert!(s.is_agent(a), "agents 表仍在");
        assert_eq!(
            count_solid_blocks(&s),
            baseline_solid,
            "实心方块数不变(主体永不凝固写块;R1-P1 按方块数不按 chunk 数)"
        );
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
        assert!(!s.forced_pending.contains(&a), "主体不进强制落地队列");
    }

    /// 判据 A01-1 前置对照(R1-P1):**同一布景**(默认凝固配置、生产节奏、
    /// 同观察者、同位置)放一个普通方块实体,它在 3600 拍内凝固——证明该
    /// 布景真的会触发凝固,站立测试的"主体不凝固"不是布景太弱。
    #[test]
    fn plain_entity_at_same_spot_solidifies_under_production_tick() {
        let (mut s, _hash) = test_sim();
        load_flat_ground(&mut s);
        let plain = types::EntityId::from_parts(1, 2);
        s.spawn_entity_for_test(plain, Vec3::new(24.0, 1.95, 24.0));
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let baseline_solid = count_solid_blocks(&s);
        let mut solidified_at = None;
        for tick in 0..3600u32 {
            production_tick(&mut s, &observers);
            if solidified_at.is_none() && !s.entities.contains_key(&plain) {
                solidified_at = Some(tick);
            }
        }
        let tick = solidified_at.expect("对照:普通实体须在 3600 拍内凝固(实体消失)");
        let grown = count_solid_blocks(&s) - baseline_solid;
        assert_eq!(
            grown, 1,
            "凝固恰好写入 1 格(拍 {tick} 实体消失 = Emergent 写块)"
        );
    }

    /// 判据 A01-1(预算豁免,R1-P2:**主体是淘汰池第一个**):预算 1、两个
    /// 醒者,主体 (31,3,0) 距观察者 31 m、普通实体 (5,3,0) 距 5 m——按距离
    /// 降序主体排第一,`enforce_budget_cells` 选中的就是主体,豁免分支真正
    /// 走到。路径 = plan 超预算淘汰(apply_activation 落实,豁免点在
    /// entities_to_freeze 段)。对照(R1-P2 第 2 条):同布景把主体换成普通
    /// 常驻实体,它会被冻结(见 plain_pool_evicts_farthest)。
    ///
    /// 布景参数:零重力(实体静止 = 可冻结/可淘汰;同既有预算测试);
    /// 凝固关(排除凝固路径干扰,被测对象 = 预算强制);两实体都在
    /// awake_unload(32 m)内(排除滞回冻结:31 m 与 5 m 都不满足出范围)。
    fn budget_pool_cfg() -> crate::sim::ShardSimConfig {
        let mut cfg = crate::sim::ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.gravity = Vec3::ZERO;
        cfg.activation.max_awake_dynamic = 1;
        cfg
    }

    fn budget_sim() -> (crate::sim::ShardSim, BodyHash) {
        let bodies = fresh_capsule_registry();
        let hash = bodies.hash_of("capsule").expect("capsule 已装");
        let mut sim = crate::sim::ShardSim::new_with_bodies(
            budget_pool_cfg(),
            Box::new(SimpleRegistry::builtin()),
            bodies,
        );
        // 平板地形:无 ①② 的实体会进待地形(kinematic),那样"被冻结"断言
        // 测不到预算路径(上一轮"前置自证"测到的其实是待地形)。
        load_flat_ground(&mut sim);
        (sim, hash)
    }

    /// plan 路径(apply_activation 内落实;豁免点 sim.rs entities_to_freeze 段)。
    /// 布景:两实体与观察者都落在 3×3 平板([0,48)²)内部,足迹不跨界缺 ①
    /// (跨界缺块会进待地形 kinematic——上一轮"前置自证"测到的其实是待地形,
    /// 不是预算冻结);主体距观察者 23 m(> 普通的 8 m,淘汰池排第一)、仍在
    /// awake_load/unload 内(不滞回冻结)。
    #[test]
    fn budget_eviction_selects_agent_first_and_exempts_it() {
        let (mut s, hash) = budget_sim();
        let a = types::EntityId::from_parts(1, 1);
        let plain = types::EntityId::from_parts(1, 2);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(31.0, 3.0, 8.0),
                ..types::EntityState::default()
            },
            None,
        )
        .expect("capsule 可生成");
        s.spawn_entity_for_test(plain, Vec3::new(16.0, 3.0, 8.0));
        // 单拍激活刷新(生产路径):预算 1,醒者 2(格数和 2),淘汰池距离
        // 降序 = [主体(23 m), 普通(8 m)] → 主体被选中 → 豁免 continue,
        // 预算收缩到此为止(普通实体不再被淘汰)。
        s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        for _ in 0..21 {
            s.step();
        }
        s.run_transient_lifecycle();
        let _ = s.take_outbox();
        // 豁免生效:主体仍醒着(Dynamic)、不被 kinematic 化、不进强制落地。
        {
            let rec = s.entities.get(&a).expect("主体还在");
            assert!(rec.actor);
            let b = s.phys.bodies.get(rec.handle).expect("handle 有效");
            assert!(
                b.is_dynamic(),
                "主体被淘汰池选中仍豁免:不冻结(变异 M1 会被本断言抓住)"
            );
        }
        assert!(!s.forced_pending.contains(&a), "主体不进强制落地队列");
        // 普通实体:预算已被主体豁免消耗(主体占池 1 格,over 归零),仍醒着。
        {
            let rec = s.entities.get(&plain).expect("普通实体还在");
            let b = s.phys.bodies.get(rec.handle).expect("handle 有效");
            assert!(b.is_dynamic(), "预算收缩由主体豁免承担,普通实体不再淘汰");
        }
    }

    /// 命令唤醒路径(R1-P2/原 README 决策 6;豁免点 enforce_budget_after_
    /// command_wake):同布景,主体经 apply_prefix(SpawnAgent)进世界——
    /// 命令唤醒的预算强制在 apply 末尾立即落实,主体被选中(距离最大)→
    /// 豁免。这条测试独立钉住命令唤醒半边(M1 删该处豁免时变红)。
    #[test]
    fn budget_after_command_wake_exempts_agent() {
        let (mut s, hash) = budget_sim();
        let a = types::EntityId::from_parts(1, 1);
        let plain = types::EntityId::from_parts(1, 2);
        s.spawn_entity_for_test(plain, Vec3::new(16.0, 3.0, 8.0));
        // 注册观察者(observed = Some 是命令唤醒预算强制的前置)。
        s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        let _ = s.take_outbox();
        // 主体走命令路径(同时钉 M5:apply_command 的 SpawnAgent 分支)。
        s.apply_prefix(&[spawn_agent_input(a, hash, Vec3::new(31.0, 3.0, 8.0))]);
        assert!(s.is_agent(a), "命令路径生成成功(布景自检)");
        for _ in 0..21 {
            s.step();
        }
        s.run_transient_lifecycle();
        let _ = s.take_outbox();
        let rec = s.entities.get(&a).expect("主体还在");
        let b = s.phys.bodies.get(rec.handle).expect("handle 有效");
        assert!(
            b.is_dynamic(),
            "命令唤醒路径:主体被淘汰池选中仍豁免(不冻结、不被强制落地)"
        );
        assert!(!s.forced_pending.contains(&a), "主体不进强制落地队列");
        let rec = s.entities.get(&plain).expect("普通实体还在");
        let b = s.phys.bodies.get(rec.handle).expect("handle 有效");
        assert!(b.is_dynamic(), "普通实体不被连带淘汰(豁免已使预算达标)");
    }

    /// 对照(R1-P2 第 2 条):同一布景把主体换成普通常驻实体——距离最大者
    /// 被预算淘汰冻结(kinematic + 速度清零)。证明淘汰池真的会选距离最大
    /// 者:上一测试里主体的幸存来自豁免,不是布景选不中它。
    #[test]
    fn plain_pool_evicts_farthest() {
        let cfg = budget_pool_cfg();
        let mut s = crate::sim::ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
        load_flat_ground(&mut s);
        let far = types::EntityId::from_parts(1, 1);
        let near = types::EntityId::from_parts(1, 2);
        // 主体位 → 换成普通**常驻**体(MarkActor 常驻化,既有预算测试同款;
        // 瞬态被淘汰走强制落地移除而非冻结,对照要测的是"冻结")。
        s.apply_prefix(&[
            spawn_dynamic_input(far, Vec3::new(31.0, 3.0, 8.0), 0),
            spawn_dynamic_input(near, Vec3::new(16.0, 3.0, 8.0), 1),
            mark_actor_input(far, 2),
            mark_actor_input(near, 3),
        ]);
        s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        for _ in 0..21 {
            s.step();
        }
        s.run_transient_lifecycle();
        let _ = s.take_outbox();
        {
            let rec = s.entities.get(&far).expect("远实体还在(被冻结非移除)");
            let b = s.phys.bodies.get(rec.handle).expect("handle 有效");
            assert!(
                b.is_kinematic(),
                "对照:同位普通常驻体被预算淘汰冻结(主体幸存 = 豁免所致)"
            );
        }
        {
            let rec = s.entities.get(&near).expect("近实体还在");
            let b = s.phys.bodies.get(rec.handle).expect("handle 有效");
            assert!(b.is_dynamic(), "预算 1:近者保留(over 归零)");
        }
    }

    /// 强制落地挡板(R1-P2/M6):调度器 7.5 的 TransientMotion 到期指令入口
    /// `apply_forced_landing_directive` 让主体进 `forced_pending`(生产上主体
    /// 常驻不进到期扫描,此为该入口的防御路径),lifecycle 的强制落地段必须
    /// 出队跳过:不移除、不写体素。变异 M6(删挡板)→ 主体被单格强制落地
    /// 写块移除,本测试红。
    #[test]
    fn forced_landing_directive_skips_agent() {
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
            None,
        )
        .expect("capsule 可生成");
        let baseline_solid = count_solid_blocks(&s);
        s.apply_forced_landing_directive(a);
        assert!(
            s.forced_pending.contains(&a),
            "布景自检:主体已进待执行集合(指令入口无来源过滤)"
        );
        s.step();
        s.run_transient_lifecycle();
        let out = s.take_outbox();
        assert!(s.is_agent(a), "主体不因强制落地指令被移除(挡板生效)");
        assert!(!s.forced_pending.contains(&a), "挡板出队(不无限堆积)");
        assert_eq!(
            count_solid_blocks(&s),
            baseline_solid,
            "主体强制落地不写任何方块"
        );
        assert!(!out.despawned.contains(&a), "主体不进 despawned(不移除)");
        assert!(
            out.forced_landings.is_empty(),
            "无强制落地遥测(主体从未落地)"
        );
    }

    /// 判据 A01-1(移除无残留,R1-P6 按**生成前**对比):移除后刚体/碰撞体
    /// 数量、`collider_to_entity`、`agents`、`entities` 的键集合全部与生成前
    /// 逐项相等。
    #[test]
    fn despawn_agent_leaves_no_residue() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        // ---- 生成前基线 ----
        let bodies_before = s.phys.bodies.len();
        let colliders_before = s.phys.colliders.len();
        let attribution_before = s.collider_to_entity_of();
        let agents_before: Vec<_> = s.agents.keys().copied().collect();
        let entities_before: Vec<_> = s.entities.keys().copied().collect();
        // ---- 生成 + 跑一拍(经历凝固桥/归因表重建)----
        s.apply_prefix(&[spawn_agent_input(a, hash, Vec3::new(24.0, 1.95, 24.0))]);
        assert!(s.is_agent(a), "布景自检:主体已生成");
        s.step();
        s.run_transient_lifecycle();
        let _ = s.take_outbox();
        assert!(
            s.entities.contains_key(&a),
            "布景自检:主体经历一拍仍在(凝固豁免)"
        );
        // ---- 移除,逐项与生成前相等 ----
        s.despawn_agent(a);
        assert_eq!(s.agent_count(), 0, "agents 表清空");
        let agents_after: Vec<_> = s.agents.keys().copied().collect();
        assert_eq!(agents_after, agents_before, "agents 键集合回到生成前");
        assert!(!s.entities.contains_key(&a), "实体记录清空");
        assert!(s.entity_state(a).is_none());
        let entities_after: Vec<_> = s.entities.keys().copied().collect();
        assert_eq!(entities_after, entities_before, "entities 键集合回到生成前");
        assert_eq!(s.phys.bodies.len(), bodies_before, "刚体数回到生成前");
        assert_eq!(
            s.phys.colliders.len(),
            colliders_before,
            "碰撞体数回到生成前"
        );
        let attribution_after = s.collider_to_entity_of();
        assert_eq!(
            attribution_after, attribution_before,
            "collider_to_entity 键值映射回到生成前"
        );
        let out = s.take_outbox();
        assert!(
            out.despawned.contains(&a),
            "despawned 照常(release ownership)"
        );
    }

    /// 判据 A01-4/R1-P5:`EntityStates` 下行不含主体——主体与一个普通实体
    /// 同时在场时,`entity_states()` 只含普通实体。
    #[test]
    fn entity_states_exclude_agents() {
        let (mut s, hash) = test_sim();
        let a = types::EntityId::from_parts(1, 1);
        let plain = types::EntityId::from_parts(1, 2);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..types::EntityState::default()
            },
            None,
        )
        .expect("capsule 可生成");
        s.spawn_entity_for_test(plain, Vec3::new(60.0, 3.0, 60.0));
        let listed: Vec<_> = s.entity_states().into_iter().map(|(id, _)| id).collect();
        assert!(
            !listed.contains(&a),
            "主体不在 EntityStates 下行(S2C::AgentStates 在 A03;R1-P5)"
        );
        assert!(
            listed.contains(&plain),
            "普通实体照常在下行(列表 = {listed:?})"
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
        s.spawn_agent(a, hash, st, None).expect("首次成功");
        let _ = s.spawn_agent(a, hash, st, None);
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
                None,
            )
            .unwrap_err();
        assert_eq!(err, AgentSpawnError::UnknownBody(unknown), "类型化拒绝");
        assert_eq!(s.outbox.agent_spawn_rejected, 1, "遥测计数");
        assert_eq!(s.agent_count(), 0);
    }

    /// 两连杆身体(B01a 起**可以生成**):本测试改为验证多连杆最小布景的
    /// 生成成功与遥测零拒绝(A01 时的类型化拒绝已随 B01a 放行撤除;两连杆
    /// 也同时核对 joint_targets 表随 dof 建立)。
    #[test]
    fn spawn_agent_two_link_accepted_with_telemetry() {
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
        s2.spawn_agent(
            types::EntityId::from_parts(1, 1),
            h,
            types::EntityState::default(),
            None,
        )
        .expect("两连杆生成成功(B01a)");
        assert_eq!(s2.outbox.agent_spawn_rejected, 0, "遥测零拒绝");
        assert_eq!(s2.agent_count(), 1);
        assert_eq!(
            s2.agent_joint_targets(types::EntityId::from_parts(1, 1))
                .expect("主体在"),
            &[0.0],
            "dof = 1,初始目标 = default_angle 缺省 0"
        );
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
            None,
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

    /// AC-A6 布景时间线(3600 拍,R1-P3b:主体生成走**输入日志**):拍 0
    /// SpawnAgent 生成站着的胶囊、拍 500 SpawnDynamic 生成一个方块实体在其
    /// 正上方(重力下落砸中)、拍 1500 Detonate 爆炸。同一输入序列在新实例
    /// 上重放,每拍主体状态逐位相同。全程按生产节奏跑(激活开启 + 观察者;
    /// 每拍 apply_activation → step → run_transient_lifecycle → take_outbox)。
    ///
    /// 布景参数:砸块从头顶 4 m 落下(~26 拍触顶),质量 0.729 vs 胶囊 0.8,
    /// 砸中后弹落滚走、静止后凝固成方块(默认凝固配置;主体 actor 恒豁免);
    /// 爆炸 power 12 / radius 5,中心 (21.5, 1, 24)(距主体 2.5 m,斜向冲量
    /// 把主体沿 +x 推开;地面炸坑但 3×3 平板足够大)。
    /// A02a:主体有了行走控制器与接触钩子(摩擦清零)——R1-P1 起无意图的
    /// 主体按零意图走控制律刹车(被推出 ~3.4 m 后停住,探针实测),不出
    /// 本体区,判据期望值不动。
    fn ac_a6_inputs(tick: u32, hash: BodyHash) -> Vec<messaging::StampedInput> {
        let mut inputs = Vec::new();
        if tick == 0 {
            inputs.push(spawn_agent_input(
                types::EntityId::from_parts(1, 1),
                hash,
                Vec3::new(24.0, 1.95, 24.0),
            ));
        }
        if tick == 500 {
            inputs.push(messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
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
            inputs.push(messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 2,
                payload: messaging::InputPayload::Command(messaging::SimCommand::Detonate {
                    center: Vec3::new(21.5, 1.0, 24.0),
                    power: 12.0,
                    radius: 5.0,
                }),
            });
        }
        inputs
    }

    /// 每拍探针(布景自检用):拍号 + 当拍分片只读视图。
    type TickProbe<'a> = dyn FnMut(u32, &crate::sim::ShardSim) + 'a;

    fn ac_a6_run(
        s: &mut crate::sim::ShardSim,
        hash: BodyHash,
        mut on_tick: Option<&mut TickProbe<'_>>,
    ) -> Vec<types::EntityState> {
        let a = types::EntityId::from_parts(1, 1);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        let mut trace = Vec::with_capacity(3600);
        for tick in 0..3600u32 {
            s.apply_prefix(&ac_a6_inputs(tick, hash));
            s.apply_activation(&observers);
            s.step();
            s.run_transient_lifecycle();
            let _ = s.take_outbox();
            if let Some(cb) = on_tick.as_deref_mut() {
                cb(tick, s);
            }
            trace.push(s.entity_state(a).expect("主体全程在(布景约束:不出本体区)"));
        }
        trace
    }

    fn ac_a6_sim() -> crate::sim::ShardSim {
        let (s, _hash) = test_sim();
        let mut s = s;
        load_flat_ground(&mut s);
        s
    }

    /// 布景自检(R1-P4):方块实体 (1,2) 与胶囊 (1,1) 当拍是否有接触对。
    fn agents_hit_by_block(s: &crate::sim::ShardSim) -> bool {
        let (Some(a), Some(b)) = (
            s.entities
                .get(&types::EntityId::from_parts(1, 1))
                .map(|r| r.collider),
            s.entities
                .get(&types::EntityId::from_parts(1, 2))
                .map(|r| r.collider),
        ) else {
            return false;
        };
        s.phys
            .narrow_phase
            .contact_pairs_with(a)
            .any(|pair| pair.collider1 == b || pair.collider2 == b)
    }

    /// 判据 A01-2 前半:同一输入日志回放两次,每拍主体状态逐位相同。
    /// 布景自检(R1-P4):方块与胶囊的接触拍数 > 0;爆炸后主体确实被推动。
    #[test]
    fn ac_a6_replay_is_bitwise_identical() {
        let hash = fresh_capsule_registry()
            .hash_of("capsule")
            .expect("capsule 已装");
        let mut s1 = ac_a6_sim();
        let mut contact_ticks = Vec::new();
        let mut probe = |tick: u32, s: &crate::sim::ShardSim| {
            if agents_hit_by_block(s) {
                contact_ticks.push(tick);
            }
        };
        let trace1 = ac_a6_run(&mut s1, hash, Some(&mut probe));
        let mut s2 = ac_a6_sim();
        let trace2 = ac_a6_run(&mut s2, hash, None);
        assert_eq!(trace1.len(), 3600);
        for (t, (x, y)) in trace1.iter().zip(trace2.iter()).enumerate() {
            assert_eq!(x, y, "拍 {t} 主体状态逐位不同");
        }
        // 布景自检(R1-P4):方块确实砸中了胶囊。
        assert!(
            !contact_ticks.is_empty(),
            "方块与胶囊的接触拍数 > 0(砸块布景自检;R1-P4)"
        );
        // 布景自检:爆炸后主体确实被推动(否则布景没有扰动,测试空转)。
        let (before, after) = (trace1[1499].translation, trace1[1520].translation);
        assert!(
            (before - after).length() > 0.01,
            "爆炸须实际推动主体:前 {before:?} 后 {after:?}"
        );
    }

    /// 判据 A01-2 后半(R1-P4):检查点落在**接触期间的某一拍**(取接触拍
    /// 序列的中点;该拍 step 完成后的状态 = 方块压在胶囊上的瞬间),恢复续跑
    /// 与不中断那一线逐位相同;另保留拍 1000 的常规检查点。
    #[test]
    fn ac_a6_checkpoint_resume_is_bitwise_identical() {
        let hash = fresh_capsule_registry()
            .hash_of("capsule")
            .expect("capsule 已装");
        let mut golden = ac_a6_sim();
        let mut contact_ticks = Vec::new();
        let mut probe = |tick: u32, s: &crate::sim::ShardSim| {
            if agents_hit_by_block(s) {
                contact_ticks.push(tick);
            }
        };
        let golden_trace = ac_a6_run(&mut golden, hash, Some(&mut probe));
        assert!(
            !contact_ticks.is_empty(),
            "布景自检:接触拍数 > 0(R1-P4;检查点取自该序列)"
        );
        // 检查点拍 = 接触序列中点(该拍 step 后 = 接触进行中)。
        let mid = contact_ticks.len() / 2;
        let impact_cp = contact_ticks[mid];
        for (cp_tick, label) in [(1000u32, "拍 1000"), (impact_cp, "接触期间")] {
            let mut s = ac_a6_sim();
            let mut resume = Vec::new();
            let a = types::EntityId::from_parts(1, 1);
            let observers = [Vec3::new(24.0, 3.0, 24.0)];
            for tick in 0..3600u32 {
                s.apply_prefix(&ac_a6_inputs(tick, hash));
                s.apply_activation(&observers);
                s.step();
                s.run_transient_lifecycle();
                let _ = s.take_outbox();
                if tick == cp_tick {
                    // 检查点 = 该拍完成后(接触进行中;R1-P4 不取接触前)。
                    let bytes = s.snapshot();
                    s = crate::sim::ShardSim::restore_with_bodies(
                        &bytes,
                        Box::new(SimpleRegistry::builtin()),
                        fresh_capsule_registry(),
                    )
                    .expect("检查点恢复");
                }
                resume.push(s.entity_state(a).expect("主体全程在"));
            }
            for (t, (x, y)) in golden_trace.iter().zip(resume.iter()).enumerate() {
                assert_eq!(
                    x, y,
                    "{label}(拍 {cp_tick})检查点续跑:拍 {t} 主体状态逐位不同"
                );
            }
        }
    }
    // ---- REQ-012 A02a 第 2 节:AgentIntent 内核校验(A02-10 内核半边)----

    /// 盖章的 AgentIntent 输入(apply_prefix 直发;网络前置校验在 driver)。
    pub(crate) fn intent_input(
        entity: types::EntityId,
        seq: u64,
        move_xz: (f32, f32),
        yaw: f32,
        jump: bool,
    ) -> messaging::StampedInput {
        messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: messaging::InputPayload::Command(messaging::SimCommand::AgentIntent {
                entity,
                move_xz,
                yaw,
                jump,
            }),
        }
    }

    /// 判据 A02-10(内核不信任回放命令):非主体实体、不存在的实体的意图
    /// = 类型化拒绝 + 遥测(agent_intent_rejected),世界不 panic。
    #[test]
    fn intent_rejects_non_agent_entity_with_telemetry() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        let plain = types::EntityId::from_parts(1, 2);
        let ghost = types::EntityId::from_parts(9, 9);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..types::EntityState::default()
            },
            None,
        )
        .expect("capsule 可生成");
        s.spawn_entity_for_test(plain, Vec3::new(60.0, 3.0, 60.0));
        // 普通实体:不是主体 → 拒绝。
        s.apply_prefix(&[intent_input(plain, 0, (1.0, 0.0), 0.0, false)]);
        assert_eq!(s.outbox.agent_intent_rejected, 1, "非主体的实体:遥测计数 1");
        // 不存在的实体:拒绝。
        s.apply_prefix(&[intent_input(ghost, 1, (1.0, 0.0), 0.0, false)]);
        assert_eq!(s.outbox.agent_intent_rejected, 2, "不存在实体:计数 2");
        // 主体本身:接受(计数不动)。
        s.apply_prefix(&[intent_input(a, 2, (1.0, 0.0), 0.0, false)]);
        assert_eq!(
            s.outbox.agent_intent_rejected, 2,
            "主体意图被接受(拒绝计数不变)"
        );
        let record = s.agents.get(&a).expect("主体在表");
        assert_eq!(record.intent_move, Some((1.0, 0.0)), "合法意图写入保持状态");
    }

    /// 判据 A02-10(无 `[walker]` 段):单连杆身体但 toml 无 walker 段 →
    /// 意图类型化拒绝 + 遥测。
    #[test]
    fn intent_rejects_body_without_walker() {
        let cfg = crate::sim::ShardSimConfig::new(shard_id(0, 0));
        let mut bodies = BodyRegistry::new();
        // 与 capsule 同形但去掉 [walker] 段(头部 + 连杆段拼回,无 walker)。
        let head = capsule_toml().split("[walker]").next().expect("段存在");
        let links = format!(
            "[[links]]{}",
            capsule_toml().split("[[links]]").nth(1).expect("段存在")
        );
        let no_walker = format!("{head}{links}");
        let h = bodies.insert_toml(&no_walker).expect("无 walker 段合法");
        let mut s =
            crate::sim::ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), bodies);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(
            a,
            h,
            types::EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..types::EntityState::default()
            },
            None,
        )
        .expect("无 walker 身体可生成");
        s.apply_prefix(&[intent_input(a, 0, (1.0, 0.0), 0.0, false)]);
        assert_eq!(
            s.outbox.agent_intent_rejected, 1,
            "无 [walker] 段:类型化拒绝 + 遥测"
        );
        let record = s.agents.get(&a).expect("主体在表");
        assert_eq!(record.intent_move, None, "保持状态未被写入");
    }

    /// 判据 A02-10(非有限值):move_xz/yaw 带 NaN/inf → 拒绝 + 遥测,
    /// 保持状态不写入(不传播 NaN 进内核)。
    #[test]
    fn intent_rejects_non_finite_values() {
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
            None,
        )
        .expect("capsule 可生成");
        for (i, (mx, mz, y)) in [
            (f32::NAN, 0.0, 0.0),
            (0.0, f32::INFINITY, 0.0),
            (0.0, 0.0, f32::NAN),
        ]
        .into_iter()
        .enumerate()
        {
            s.apply_prefix(&[intent_input(a, i as u64, (mx, mz), y, false)]);
        }
        assert_eq!(
            s.outbox.agent_intent_rejected, 3,
            "三个非有限组合全拒绝(遥测各 +1)"
        );
        let record = s.agents.get(&a).expect("主体在表");
        assert_eq!(record.intent_move, None, "保持状态未被污染");
    }

    /// 判据 A02-10(钳制):move_xz 长度超过 max_speed(5.6)→ 按比例钳到
    /// max_speed + 遥测(agent_intent_clamped);方向不变。
    #[test]
    fn intent_clamps_move_over_max_speed_with_telemetry() {
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
            None,
        )
        .expect("capsule 可生成");
        // (3, 4) 长 5 < 5.6:不钳。
        s.apply_prefix(&[intent_input(a, 0, (3.0, 4.0), 0.0, false)]);
        assert_eq!(s.outbox.agent_intent_clamped, 0, "5 m/s 不钳");
        // (30, 40) 长 50 > 5.6:按比例钳到 5.6(方向不变)。
        s.apply_prefix(&[intent_input(a, 1, (30.0, 40.0), 0.0, false)]);
        assert_eq!(s.outbox.agent_intent_clamped, 1, "50 m/s 被钳,遥测 +1");
        let record = s.agents.get(&a).expect("主体在表");
        let (mx, mz) = record.intent_move.expect("钳后的意图已写入");
        let len = (mx * mx + mz * mz).sqrt();
        assert!(
            (len - 5.6).abs() < 1e-4,
            "钳后长度 = max_speed 5.6,实为 {len}"
        );
        assert!((mx / mz - 30.0 / 40.0).abs() < 1e-4, "方向不变(比例钳制)");
    }

    /// 判据 A02-10(yaw 规范化):yaw 规范化到 (−π, π]——3π → π,
    /// −3π/2 → +π/2,4π → 0,−π → +π(半开区间折到上界)。
    #[test]
    fn intent_normalizes_yaw_to_pi_range() {
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
            None,
        )
        .expect("capsule 可生成");
        let pi = core::f32::consts::PI;
        let cases: [(f32, f32); 4] = [
            (3.0 * pi, pi),        // 3π ≡ π((−π, π] 上界)
            (-1.5 * pi, 0.5 * pi), // −3π/2 ≡ +π/2
            (4.0 * pi, 0.0),       // 4π ≡ 0
            (-pi, pi),             // −π ≡ +π(半开区间折到上界)
        ];
        for (i, (yaw_in, expect)) in cases.iter().enumerate() {
            s.apply_prefix(&[intent_input(a, i as u64, (0.0, 0.0), *yaw_in, false)]);
            let record = s.agents.get(&a).expect("主体在表");
            let yaw = record.intent_yaw.expect("yaw 已写入");
            assert!(
                (yaw - *expect).abs() < 1e-5,
                "yaw {yaw_in} 规范化到 (−π, π],期望 {expect},实得 {yaw}"
            );
        }
    }

    /// 意图是保持状态(A02a 第 2 节):下一条意图到来前持续生效(只发一条
    /// 1.5 m/s 意图,主体持续走;发零意图后刹车停住)。
    #[test]
    fn intent_is_held_state_until_next_intent() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.apply_prefix(&[
            spawn_agent_input(a, hash, Vec3::new(24.0, 1.95, 24.0)),
            intent_input(a, 0, (1.5, 0.0), 0.0, false),
        ]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        // 跑 120 拍(意图只发一条)。
        for _ in 0..120 {
            if !observers.is_empty() {
                s.apply_activation(&observers);
            }
            s.step();
            s.run_transient_lifecycle();
            let _ = s.take_outbox();
        }
        let st = s.entity_state(a).expect("主体还在");
        assert!(
            st.linvel.x > 1.2,
            "意图保持:120 拍后仍在按 1.5 m/s 意图行走,vx={}",
            st.linvel.x
        );
        // 再发零意图:主体刹车停下(保持状态被覆盖)。
        s.apply_prefix(&[intent_input(a, 1, (0.0, 0.0), 0.0, false)]);
        for _ in 0..120 {
            if !observers.is_empty() {
                s.apply_activation(&observers);
            }
            s.step();
            s.run_transient_lifecycle();
            let _ = s.take_outbox();
        }
        let st = s.entity_state(a).expect("主体还在");
        assert!(
            st.linvel.x.abs() < 0.15,
            "零意图覆盖后刹车:vx={}",
            st.linvel.x
        );
    }

    /// jump 一次性(A02a 第 2 节):置位后在下一个控制拍被消费,消费即清零
    /// (接没接地都在控制拍消费;没接地就丢掉,不缓存)。
    #[test]
    fn jump_is_consumed_once_not_cached() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.apply_prefix(&[
            spawn_agent_input(a, hash, Vec3::new(24.0, 1.95, 24.0)),
            intent_input(a, 0, (0.0, 0.0), 0.0, false),
        ]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        for _ in 0..60 {
            if !observers.is_empty() {
                s.apply_activation(&observers);
            }
            s.step();
            s.run_transient_lifecycle();
            let _ = s.take_outbox();
        }
        assert!(
            s.agent_is_grounded(a, 0.05),
            "布景自检:安定 60 拍后主体接地"
        );
        // 发 jump = true(接地)→ 下一个控制拍起跳且标志清零。
        s.apply_prefix(&[intent_input(a, 1, (0.0, 0.0), 0.0, true)]);
        let record = s.agents.get(&a).expect("主体在表");
        assert!(record.intent_jump, "布景自检:jump 已置位(命令当拍应用)");
        // 跑 2 拍必跨一个控制拍(控制拍 = tick % 2 == 0,60 Hz 施力 / 30 Hz
        // 决策)。
        let mut jumped_vy = None;
        for _ in 0..2 {
            if !observers.is_empty() {
                s.apply_activation(&observers);
            }
            s.step();
            s.run_transient_lifecycle();
            let _ = s.take_outbox();
            if jumped_vy.is_none() {
                let st = s.entity_state(a).expect("主体还在");
                if st.linvel.y > 1.0 {
                    jumped_vy = Some(st.linvel.y);
                }
            }
        }
        let record = s.agents.get(&a).expect("主体在表");
        assert!(!record.intent_jump, "jump 一次性:控制拍消费后标志清零");
        let vy = jumped_vy.expect("接地起跳:竖直速度应达到起跳量级(> 1 m/s)");
        assert!(vy > 1.0, "起跳速度量级 sqrt(2·9.81·1.4) ≈ 5.24 m/s");
    }

    // ---- REQ-012 A02b:判据 A02-9(确定性,内核检查点半边)----
    //
    // 时间线(全 900 拍,主体在 3×3 平板内往返):拍 0 spawn;拍 1 意图
    // (5.6, 0)(行走);拍 121 = 检查点 1(行走中);拍 150 折返;拍 360
    // 意图 (5.6, 0) + jump(移动中跳);空中拍 = 检查点 2;拍 440 意图清零
    // 刹停;拍 441 起第一个"step_tick 奇数且保持力非零"的拍 = 检查点 3
    // (布景断言这一点);拍 500 在主体当拍实际位置正上方生成砸块;拍 700
    // 在主体旁 2.5 m 爆炸。砸块/爆心的坐标按**当拍实际位置**动态生成——
    // 恢复线在检查点后与 golden 逐位相同,读到的位置一致 → 输入一致。
    // 每条命令都是盖章输入(apply_prefix 消费 = 输入日志条目的内核形态);
    // Session 层回放两次的另一半在 crates/app/tests/a02b_session_determinism.rs。

    /// 时间线的第 tick 拍输入(sequence 全局单调;`probe` = 当拍主体位置,
    /// 供砸块/爆心定位,golden 与恢复线各自读自己的状态,逐位一致)。
    fn a02b9_inputs(
        tick: u32,
        hash: BodyHash,
        probe: types::EntityState,
    ) -> Vec<messaging::StampedInput> {
        let mut inputs = Vec::new();
        let a = types::EntityId::from_parts(1, 1);
        if tick == 0 {
            inputs.push(spawn_agent_input(a, hash, Vec3::new(24.0, 1.95, 24.0)));
        }
        if tick == 1 {
            inputs.push(intent_input(a, 1, (5.6, 0.0), 0.0, false));
        }
        if tick == 150 {
            inputs.push(intent_input(a, 2, (-5.6, 0.0), 0.0, false));
        }
        if tick == 360 {
            inputs.push(intent_input(a, 3, (5.6, 0.0), 0.0, true));
        }
        if tick == 440 {
            inputs.push(intent_input(a, 4, (0.0, 0.0), 0.0, false));
        }
        if tick == 500 {
            // 砸块:主体正上方(top 2.8 + 0.5 + 3.0 = 6.3;ac_a6 同款)。
            inputs.push(messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 5,
                payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                    entity: types::EntityId::from_parts(1, 2),
                    state: types::EntityState {
                        translation: Vec3::new(probe.translation.x, 6.3, probe.translation.z),
                        ..types::EntityState::default()
                    },
                    desc: types::EntityDesc::default(),
                }),
            });
        }
        if tick == 700 {
            // 爆炸:主体旁 2.5 m(ac_a6 同 power/radius,位置跟随主体)。
            inputs.push(messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 6,
                payload: messaging::InputPayload::Command(messaging::SimCommand::Detonate {
                    center: Vec3::new(probe.translation.x - 2.5, 1.0, probe.translation.z),
                    power: 12.0,
                    radius: 5.0,
                }),
            });
        }
        inputs
    }

    /// 检查点拍探测结果(跑 golden 线时顺路记录)。
    #[derive(Debug, Clone, Copy)]
    struct A02b9Checkpoints {
        walking: u32,
        airborne: Option<u32>,
        between_controls: Option<u32>,
        /// 检查点 3 拍的保持力(探测时记录;golden 跑完后 walker_holds 已是
        /// 末态,不能回查历史拍)。
        between_controls_hold: (f32, f32),
    }

    /// A02-9(内核半边):三检查点(行走中 / 空中 / 两个控制拍之间且保持力
    /// 非零)的快照恢复续跑与不中断的 golden 线每拍主体状态逐位相同。
    ///
    /// 检查点 3 的布景前置在 golden 线上断言:该拍 `step_tick` 为奇数(下一
    /// 拍非控制拍)、`walker_holds[a].held_force != (0, 0)`(保持的力非零
    /// ——快照丢 `walker_holds` 的变异让本测试红:恢复线保持力清空 → 后续
    /// 轨迹分叉)。
    #[test]
    fn a02_9_checkpoint_resume_bitwise_identical_at_three_positions() {
        let hash = fresh_capsule_registry()
            .hash_of("capsule")
            .expect("capsule 已装");
        let a = types::EntityId::from_parts(1, 1);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        const TOTAL: u32 = 900;

        // ---- golden 线:完整跑一遍,顺路探测三个检查点拍 ----
        let mut golden = ac_a6_sim();
        let mut cps = A02b9Checkpoints {
            walking: 121,
            airborne: None,
            between_controls: None,
            between_controls_hold: (0.0, 0.0),
        };
        let mut golden_trace: Vec<types::EntityState> = Vec::with_capacity(TOTAL as usize);
        let mut hit_seen = false;
        for tick in 0..TOTAL {
            let probe = golden
                .entity_state(a)
                .unwrap_or_else(|| types::EntityState {
                    translation: Vec3::new(24.0, 1.95, 24.0),
                    ..types::EntityState::default()
                });
            golden.apply_prefix(&a02b9_inputs(tick, hash, probe));
            golden.apply_activation(&observers);
            golden.step();
            golden.run_transient_lifecycle();
            let _ = golden.take_outbox();
            let st = golden.entity_state(a).expect("golden 线主体全程在");
            // 布景自检:砸块(实体 (1,2))与主体发生过接触。
            if !hit_seen
                && let (Some(r2), Some(ra)) = (
                    golden.entities.get(&types::EntityId::from_parts(1, 2)),
                    golden.entities.get(&a),
                )
            {
                let ca = ra.collider;
                hit_seen = golden
                    .phys
                    .narrow_phase
                    .contact_pairs_with(r2.collider)
                    .any(|pair| pair.collider1 == ca || pair.collider2 == ca);
            }
            // 检查点 2:空中(jump 后 vy > 1.5 的第一拍;此后不再改)。
            if cps.airborne.is_none() && tick > 360 && st.linvel.y > 1.5 {
                cps.airborne = Some(tick);
            }
            // 检查点 3:两个控制拍之间(step_tick 奇数 = 下一拍非控制拍)
            // 且保持力非零(刹车段力非零;再显式断言)。
            if cps.between_controls.is_none()
                && (440..500).contains(&tick)
                && golden.step_tick.0 % 2 == 1
                && golden
                    .walker_holds
                    .get(&a)
                    .is_some_and(|h| h.held_force != (0.0, 0.0))
            {
                cps.between_controls = Some(tick);
                cps.between_controls_hold = golden
                    .walker_holds
                    .get(&a)
                    .map(|h| h.held_force)
                    .unwrap_or((0.0, 0.0));
            }
            golden_trace.push(st);
        }
        let airborne = cps
            .airborne
            .expect("布景自检:900 拍内出现空中拍(jump 后 vy > 1.5)");
        let between = cps
            .between_controls
            .expect("布景自检:刹车段出现保持力非零的控制拍间隙");
        assert!(
            golden_trace[cps.walking as usize].linvel.x > 5.0,
            "布景自检:检查点 1(拍 121)处于行走中(vx = {})",
            golden_trace[cps.walking as usize].linvel.x
        );
        assert!(
            golden_trace[airborne as usize].linvel.y > 1.5,
            "布景自检:检查点 2(拍 {airborne})处于空中(vy = {})",
            golden_trace[airborne as usize].linvel.y
        );
        // 布景断言(放行 A02-9 原文):检查点 3 落在两个控制拍之间,保持的
        // 力非零。step_tick 奇数在探测条件里;保持力取探测拍记录值(golden
        // 跑完后 walker_holds 已是末态)。
        assert!(
            cps.between_controls_hold != (0.0, 0.0),
            "布景自检:检查点 3 拍 {between} 保持的力非零"
        );
        // 布景自检:砸块与主体发生过接触(方块自由落体 ~51 拍后砸中主体)。
        assert!(hit_seen, "布景自检:砸块与主体发生接触(被砸半边真实发生)");

        // ---- 三条恢复线:跑到检查点拍 step 完成后快照 → 恢复 → 续跑 ----
        for (cp_tick, label) in [
            (cps.walking, "行走中"),
            (airborne, "空中"),
            (between, "控制拍之间"),
        ] {
            let mut s = ac_a6_sim();
            let mut resume = Vec::with_capacity(TOTAL as usize);
            for tick in 0..TOTAL {
                let probe = s.entity_state(a).unwrap_or_else(|| types::EntityState {
                    translation: Vec3::new(24.0, 1.95, 24.0),
                    ..types::EntityState::default()
                });
                s.apply_prefix(&a02b9_inputs(tick, hash, probe));
                s.apply_activation(&observers);
                s.step();
                s.run_transient_lifecycle();
                let _ = s.take_outbox();
                if tick == cp_tick {
                    let bytes = s.snapshot();
                    s = crate::sim::ShardSim::restore_with_bodies(
                        &bytes,
                        Box::new(SimpleRegistry::builtin()),
                        fresh_capsule_registry(),
                    )
                    .expect("检查点恢复");
                }
                resume.push(s.entity_state(a).expect("恢复线主体全程在"));
            }
            for (t, (x, y)) in golden_trace.iter().zip(resume.iter()).enumerate() {
                assert_eq!(
                    x, y,
                    "检查点[{label}](拍 {cp_tick})恢复续跑:拍 {t} 主体状态逐位不同"
                );
            }
        }
    }

    // ---- REQ-012 A02b:判据 A02-11(兜底冻结的主体豁免)----
    //
    // 主体在所有观察者的唤醒范围外持续行走,超过 actor_motion_max_ticks
    // (默认 1200)拍:断言主体没被兜底冻结、仍是 Dynamic,并继续按意图
    // 行走。变异自证(删 run_transient_lifecycle 兜底冻结里的主体豁免)
    // 让本测试红。

    /// A02-11 前半:主体持续行走出唤醒范围,常驻运动票记账触顶(默认
    /// 1200 拍)后不被兜底冻结——分片内闭环:触顶 → capped_freeze_pending
    /// → run_transient_lifecycle 的主体豁免跳过(冻结语义在 B03)。
    /// 布景:8×2 chunk 平板(x 0..128, z 0..32;全在分片本体区内),观察者
    /// (4,3,16),主体出生 (4.5,1.95,16) 意图 (4.3, 0) 沿 +x;~440 拍出
    /// 唤醒 unload 半径(32 m)开始记账,再 1200 拍触顶 ≈ 拍 1640;总
    /// 1680 拍(末态 x ≈ 125 < 128,远离分片边界——迁移 fail-stop 的
    /// 布景纪律)。
    #[test]
    fn a02_11_agent_walking_past_actor_motion_cap_is_not_frozen() {
        let (mut s, hash) = test_sim();
        // 8×2 chunk 平板(层 64 全石)。
        let stone = types::BlockId(1);
        for cx in 0..8i32 {
            for cz in 0..2i32 {
                let mut chunk = voxel::Chunk::air();
                for x in 0..voxel::CHUNK_X {
                    for z in 0..voxel::CHUNK_Z {
                        chunk.set(x, 64, z, stone);
                    }
                }
                s.load_chunk(types::ChunkPos::new(cx, cz), chunk);
            }
        }
        let a = types::EntityId::from_parts(1, 1);
        s.apply_prefix(&[
            spawn_agent_input(a, hash, Vec3::new(4.5, 1.95, 16.0)),
            intent_input(a, 1, (4.3, 0.0), 0.0, false),
        ]);
        let observers = [Vec3::new(4.0, 3.0, 16.0)];
        // 全程不冻结(每 10 拍抽查;触顶预期在 ~1640 拍,抽查覆盖到其后)。
        let mut last_x_tick = 0u32;
        for tick in 0..1680u32 {
            production_tick(&mut s, &observers);
            if tick % 10 == 9 {
                // entity_state 的存在性自检:主体不被强制落地/虚空移除。
                s.entity_state(a)
                    .expect("主体全程在(不被强制落地/虚空移除)");
                assert!(
                    !s.is_frozen(a),
                    "A02-11:拍 {tick} 主体被冻结(兜底冻结豁免缺失)——触顶预期拍 ~1640"
                );
                last_x_tick = tick;
            }
        }
        // 继续按意图行走:末态净位移 ≈ 4.3 m/s × 28 s ≈ 120 m。
        let st = s.entity_state(a).expect("主体全程在");
        assert!(
            st.translation.x - 4.5 > 90.0,
            "A02-11:主体按意图持续行走(末态 x = {:.1},期望 > 94.5)",
            st.translation.x
        );
        assert!(
            last_x_tick >= 1650,
            "布景自检:抽查覆盖到触顶之后的拍(最后抽查拍 {last_x_tick})"
        );
    }

    /// A02-11 后半(冻结闭包):主体站在一个方块实体上,该方块被**滞回
    /// 冻结**(观察者移远 → 出唤醒 unload 半径 → 静止常驻体决策冻结)。
    /// 断言:主体没有被连带冻结(`freeze_with_closure` 的主体豁免;变异删
    /// 该豁免让本测试红)、方块确实冻结了(证明布景走到了闭包)。
    ///
    /// 布景:3×3 平板,方块 SpawnDynamic (24,1.5,24)(贴地),主体出生其
    /// 上 (24,2.9,24);观察者 (24,3,24) 安定 120 拍(主体落上方块、双方
    /// 静止判据推进)→ 观察者移到 (500,0,500) → 下一拍 plan 决策:方块
    /// outside+静止 → 冻结;主体 outside+静止 → 决策含主体但落实段豁免。
    /// 跑 60 拍让决策落实。
    #[test]
    fn a02_11_agent_on_frozen_block_is_not_caught_by_closure() {
        let (mut s, hash) = test_sim();
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        let block = types::EntityId::from_parts(1, 2);
        s.apply_prefix(&[
            spawn_agent_input(a, hash, Vec3::new(24.0, 2.9, 24.0)),
            // 方块:半边 0.5、贴地(格中心 y = 1.5);MarkActor 常驻化——
            // 瞬态体不进滞回冻结分支(`!actor && !carried` 冻结恒 false),
            // 常驻体才走"出唤醒范围 + 静止 → 滞回冻结"(sim.rs 测试同款)。
            messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                    entity: block,
                    state: types::EntityState {
                        translation: Vec3::new(24.0, 1.5, 24.0),
                        ..types::EntityState::default()
                    },
                    desc: types::EntityDesc::default(),
                }),
            },
            messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 2,
                payload: messaging::InputPayload::Command(messaging::SimCommand::MarkActor {
                    entity: block,
                    actor: true,
                }),
            },
        ]);
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        for _ in 0..120 {
            production_tick(&mut s, &observers);
        }
        // 安定自检:主体站在方块上(双方醒着、主体接地)。
        assert!(s.agent_is_grounded(a, 0.05), "布景自检:主体落在方块上接地");
        assert!(!s.is_frozen(block), "布景自检:安定段方块未冻结(在唤醒范围)");
        assert!(!s.is_frozen(a), "布景自检:安定段主体未冻结");

        // 观察者移远 → 滞回决策(方块 outside + 静止 → 冻结;主体 outside
        // + 静止 → 决策含主体,落实段豁免;闭包从方块递归时主体豁免)。
        let far = [Vec3::new(500.0, 0.0, 500.0)];
        for _ in 0..60 {
            production_tick(&mut s, &far);
        }
        // 主体没有被连带冻结,仍是 Dynamic 并仍在表(未被凝固写块)。
        assert!(
            !s.is_frozen(a),
            "A02-11:支撑方块被滞回冻结时主体被连带冻结(闭包主体豁免缺失)"
        );
        assert!(s.is_agent(a), "主体仍在 agents 表(未被写块)");
        assert!(
            s.entity_state(a).is_some(),
            "主体实体仍在(闭包未把主体凝固/移除)"
        );
        // 方块确实冻结了(证明布景走到了滞回冻结 + 闭包递归路径)。
        assert!(
            s.is_frozen(block),
            "布景自检:方块已被滞回冻结(闭包路径真实走到;否则布景无效)"
        );
    }

    // ---- A04 R1-P8:信封往返逐字段 ----

    /// 发出方记录快照(P8 断言基准:`AgentRecord` 控制器三字段 +
    /// `WalkerHold` 两字段 + `EntityRecord` 的 `source`/`actor_motion_ticks`)。
    struct AgentSideSnapshot {
        intent_move: Option<(f32, f32)>,
        intent_yaw: Option<f32>,
        intent_jump: bool,
        held_force: (f32, f32),
        last_yaw: f32,
        source: Option<messaging::InputSource>,
        actor_motion_ticks: Option<u64>,
    }

    fn agent_side_snapshot(s: &crate::sim::ShardSim, a: types::EntityId) -> AgentSideSnapshot {
        let rec = s.agents.get(&a).expect("主体在表");
        let hold = s.walker_hold_of(a).expect("保持态在表");
        let entity_rec = s.entities.get(&a).expect("实体在表");
        AgentSideSnapshot {
            intent_move: rec.intent_move,
            intent_yaw: rec.intent_yaw,
            intent_jump: rec.intent_jump,
            held_force: (hold.0, hold.1),
            last_yaw: hold.2,
            source: entity_rec.source,
            actor_motion_ticks: entity_rec.actor_motion_ticks,
        }
    }

    /// 信封往返(R1-P8):各字段非默认的主体(意图三项、保持力、
    /// `last_yaw`、`actor_motion_ticks`、`source`,`intent_jump = true`)
    /// 经 `MigrateAgent` 信封迁到**另一个 ShardSim**(同注册表、同地形),
    /// 接收方的记录与保持态逐字段等于发出方。
    ///
    /// 布景:两个 sim(片 (0,0)/(1,0)),各装 3×3 平板;发出方 sim 直接对
    /// 记录写字段(意图/保持态走内核写入口,`source`/`actor_motion_ticks`
    /// 测试内直写 EntityRecord——P8 放行原文口径)。迁移触发:发出方 step
    /// 把主体推出本体区(位置直写刚体)→ step 拍末产 `MigrateAgent`
    /// Intent → 构造盖章信封投给接收方 apply_prefix。
    #[test]
    fn a04_p8_migrate_agent_envelope_roundtrip_fieldwise() {
        let (mut src, hash) = test_sim();
        load_flat_ground(&mut src);
        // 接收方:同注册表哈希(共表口径)+ 同地形。
        let cfg = crate::sim::ShardSimConfig::new(shard_id(1, 0));
        let mut dst = crate::sim::ShardSim::new_with_bodies(
            cfg,
            Box::new(SimpleRegistry::builtin()),
            fresh_capsule_registry(),
        );
        load_flat_ground(&mut dst);
        let a = types::EntityId::from_parts(1, 1);
        src.apply_prefix(&[spawn_agent_input(a, hash, Vec3::new(24.0, 2.0, 24.0))]);
        src.apply_prefix(&[intent_input(a, 0, (2.5, -1.5), 0.75, true)]);
        // 跑一拍让意图生效(跳请求在控制拍被消费,`intent_jump` 回落 false;
        // P8 要 `intent_jump = true` 随信封走,故消费后再置一次)。
        src.step();
        src.run_transient_lifecycle();
        let _ = src.take_outbox();
        src.apply_prefix(&[intent_input(a, 1, (2.5, -1.5), 0.75, true)]);
        // 直写各字段到非默认值(P8 放行:source 在测试里直接写到记录上;
        // actor_motion_ticks 同款;保持力/last_yaw 直写 walker_holds)。
        let src_src = Some(messaging::InputSource(7));
        let src_ticks = Some(321u64);
        if let Some(rec) = src.entities.get_mut(&a) {
            rec.source = src_src;
            rec.actor_motion_ticks = src_ticks;
        }
        let hold_force = (1.25f32, -0.75f32);
        let last_yaw = -1.25f32;
        src.walker_holds.insert(
            a,
            crate::walker::WalkerHold {
                held_force: hold_force,
                last_yaw,
            },
        );
        let before = agent_side_snapshot(&src, a);
        assert_eq!(before.intent_move, Some((2.5, -1.5)), "布景自检:意图写入");
        assert_eq!(before.intent_yaw, Some(0.75), "布景自检:朝向写入");
        assert!(before.intent_jump, "布景自检:intent_jump = true");
        assert_eq!(before.held_force, hold_force, "布景自检:保持力写入");
        assert_eq!(before.last_yaw, last_yaw, "布景自检:last_yaw 写入");
        assert_eq!(before.source, src_src, "布景自检:source 直写");
        assert_eq!(
            before.actor_motion_ticks, src_ticks,
            "布景自检:actor_motion_ticks 直写"
        );

        // 迁移:拍末 step 检测到主体出本体区(位置直写刚体到 x > 本体区)。
        // 本体区 128 m(core 8 chunk);直写 x = 130、y 站立高度。
        {
            let rec = src.entities.get(&a).expect("在场");
            let handle = rec.handle;
            let body = src.phys.bodies.get_mut(handle).expect("刚体在场");
            body.set_translation(
                crate::conv::vec3_to_rapier(Vec3::new(130.0, 1.9, 24.0)),
                true,
            );
            body.set_linvel(crate::conv::vec3_to_rapier(Vec3::new(2.5, 0.0, -1.5)), true);
        }
        src.step();
        src.run_transient_lifecycle();
        let outbox = src.take_outbox();
        assert!(
            !src.agents.contains_key(&a),
            "发出方:主体已原子移除(agents 不留残余)"
        );
        // 从 outbox.migrations 取 MigrateAgent Intent,盖章成信封投接收方。
        let mut payload = None;
        for intent in &outbox.migrations {
            if let messaging::EnvelopePayload::MigrateAgent { entity, .. } = &intent.payload {
                assert_eq!(*entity, a, "信封实体一致");
                payload = Some(intent.payload.clone());
            }
        }
        let payload = payload.expect("布景自检:拍末产出 MigrateAgent");
        let env = messaging::Envelope::stamp(
            shard_id(0, 0),
            messaging::Intent {
                destination: shard_id(1, 0),
                payload,
            },
            types::Tick(0),
            0,
        );
        dst.apply_prefix(&[messaging::StampedInput {
            apply_at: types::Tick(1),
            source: messaging::InputSource::SCHEDULER,
            sequence: 0,
            payload: messaging::InputPayload::Envelope(env),
        }]);
        let after = agent_side_snapshot(&dst, a);
        assert_eq!(
            after.intent_move, before.intent_move,
            "P8:接收方 intent_move 逐字段等于发出方"
        );
        assert_eq!(
            after.intent_yaw, before.intent_yaw,
            "P8:接收方 intent_yaw 逐字段等于发出方"
        );
        assert_eq!(
            after.intent_jump, before.intent_jump,
            "P8:接收方 intent_jump 逐字段等于发出方(true 随信封走)"
        );
        assert_eq!(
            after.held_force, before.held_force,
            "P8:接收方保持力逐字段等于发出方"
        );
        assert_eq!(
            after.last_yaw, before.last_yaw,
            "P8:接收方 last_yaw 逐字段等于发出方"
        );
        assert_eq!(after.source, before.source, "P8:接收方写回 source(R1-P8)");
        assert_eq!(
            after.actor_motion_ticks, before.actor_motion_ticks,
            "P8:接收方 actor_motion_ticks 逐字段等于发出方"
        );
        dst.run_transient_lifecycle();
        let _ = dst.take_outbox();
    }

    // ---- REQ-012 B01a:多连杆内核(K1–K12;放行第 4 节)----

    /// quad.body.toml 文本(与 types 测试同一份资产)。
    pub(crate) fn quad_toml() -> &'static str {
        include_str!("../../../assets/bodies/quad.body.toml")
    }

    /// quad.body.toml 的注册表(单装 quad;测试多用 quad+capsule 混装)。
    fn quad_registry() -> (BodyRegistry, BodyHash) {
        let mut bodies = BodyRegistry::new();
        let hash = bodies
            .insert_toml(quad_toml())
            .expect("quad.body.toml 合法");
        (bodies, hash)
    }

    /// quad + capsule 注册表 + 配置(质量比例可调;凝固默认开,需要关的
    /// 自行覆写)。
    fn quad_sim_with(k: f32) -> (crate::sim::ShardSim, BodyHash) {
        let cfg = crate::sim::ShardSimConfig::new(shard_id(0, 0));
        let mut cfg = cfg;
        cfg.agent_mass_scale = k;
        let (mut bodies, hash) = quad_registry();
        let _ = bodies.insert_toml(capsule_toml()).expect("capsule 合法");
        let sim =
            crate::sim::ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), bodies);
        (sim, hash)
    }

    /// load_flat_ground 的地面顶面(chunk y 下标 64 → 世界 y ∈ [0,1])。
    /// B01a-R1(P1):四足原先生成在 y=0.448,整只在地面方块内部,造出
    /// "不自然入睡/滑移 0.118"两条假结论;正确出生点 = 顶面 + 0.45。
    const GROUND_TOP_Y: f32 = 1.0;

    /// 四足生成位置:关节角全 0 时足底在根下方 0.448;留 2 mm 间隙。
    fn quad_origin(x: f32, z: f32) -> Vec3 {
        Vec3::new(x, GROUND_TOP_Y + 0.45, z)
    }

    /// 布景自检:全部连杆碰撞体的世界 AABB 最低点 ≥ GROUND_TOP_Y − 1e-3
    /// (不许再出现"生成进地面方块内部"的布景错误)。
    fn assert_quad_above_ground(s: &crate::sim::ShardSim, a: types::EntityId) {
        let record = s.agents.get(&a).expect("主体登记");
        for &ch in &record.colliders {
            let collider = s.phys.colliders.get(ch).expect("连杆碰撞体存在");
            let aabb = collider.compute_aabb();
            assert!(
                aabb.mins.y >= GROUND_TOP_Y - 1e-3,
                "布景自检:碰撞体 {ch:?} 世界 AABB 最低 {} < 地面顶面 {GROUND_TOP_Y}",
                aabb.mins.y
            );
        }
    }

    /// 生成一只四足的出生 state(关节角全 0 的竖直链,足端在 origin 下方
    /// 0.34 m;测试把它放在平板上方一点,由电机拉到站立姿态)。
    fn quad_spawn_state(origin: Vec3) -> types::EntityState {
        types::EntityState {
            translation: origin,
            rotation: types::glam::Quat::IDENTITY,
            ..types::EntityState::default()
        }
    }

    /// K1:生成四足 = 13 刚体、12 自由度;全部连杆碰撞体映射到同一实体;
    /// multibody 阻尼各项 = rapier 默认 × k;自碰撞关闭;关节接触关闭。
    #[test]
    fn k1_spawn_quad_structure() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let record = s.agents.get(&a).expect("主体登记");
        // 13 刚体登记(links 表 = 13 条;实体记录 handle = 根连杆)。
        assert_eq!(record.links.len(), 13, "13 连杆");
        // 12 个自由度;目标 = default_angle(站立角);读取口齐全。
        assert_eq!(record.joint_targets.len(), 12);
        assert_eq!(s.agent_joint_positions(a).expect("关节位").len(), 12);
        assert_eq!(s.agent_joint_velocities(a).expect("关节速").len(), 12);
        // 全部连杆碰撞体映射到同一实体(13 碰撞体;脚球是 shank 第二形状,
        // 同一刚体的 collider,不单算连杆)。
        assert_eq!(record.colliders.len(), 13, "13 碰撞体");
        let table = s.collider_to_entity_of();
        for &ch in &record.colliders {
            let key = ch.into_raw_parts();
            assert_eq!(
                table.get(&(key.0, key.1)),
                Some(&a),
                "连杆碰撞体 {ch:?} 归属同一实体"
            );
        }
        // multibody:13 link;阻尼各项 = rapier 默认 × k;自碰撞关闭。
        let mb_index = s.agent_multibody_index(record).expect("四足在 multibody");
        let mb = s.phys.multibody_joints.get_multibody(mb_index).expect("mb");
        assert_eq!(mb.num_links(), 13, "multibody 13 个 link");
        for (i, d) in mb.damping().iter().enumerate() {
            let expected = match i {
                // rapier 默认:根平动 0;根转动与每个关节 0.1。
                0..=2 => 0.0,
                _ => 0.1,
            };
            assert!(
                (*d - expected * 0.01).abs() < 1e-6,
                "阻尼[{i}] = {d},期望 {}(默认 {expected} × k)",
                expected * 0.01
            );
        }
        assert!(!mb.self_contacts_enabled(), "自碰撞关闭");
        // 全部关节 contacts_enabled(false)(相邻连杆接触一律关闭)。
        for link in mb.links() {
            if link.link_id() != 0 {
                assert!(
                    !link.joint.data.contacts_enabled(),
                    "关节 contacts_enabled 必为 false(link {})",
                    link.link_id()
                );
            }
        }
        // 根速度取 state(默认零 = 数值核对),其余连杆速度为零。
        let root_rb = s
            .phys
            .bodies
            .get(s.entities.get(&a).unwrap().handle)
            .expect("根刚体");
        assert!(
            crate::conv::rapier_to_vec3(root_rb.linvel()).length() < 1e-6,
            "根线速度取 state(默认零)"
        );
    }

    /// K2:移除四足后刚体数、碰撞体数、multibody 数回到生成前。
    #[test]
    fn k2_despawn_quad_removes_all_bodies() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let bodies_before = s.phys.bodies.len();
        let colliders_before = s.phys.colliders.len();
        let mb_before = s.phys.multibody_joints.iter().count();
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        assert_eq!(s.phys.bodies.len(), bodies_before + 13);
        assert_eq!(s.phys.colliders.len(), colliders_before + 13);
        assert_eq!(s.phys.multibody_joints.iter().count(), mb_before + 12);
        s.despawn_agent(a);
        assert_eq!(s.phys.bodies.len(), bodies_before, "刚体回到生成前");
        assert_eq!(s.phys.colliders.len(), colliders_before, "碰撞体回到生成前");
        assert_eq!(
            s.phys.multibody_joints.iter().count(),
            mb_before,
            "multibody 数回到生成前"
        );
        assert!(!s.is_agent(a));
    }

    /// K6:电机上限读回 = max_torque × k,增益读回 = kp × k、kd × k。
    #[test]
    fn k6_motor_params_scaled_by_k() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let record = s.agents.get(&a).expect("主体登记");
        let mb_index = s.agent_multibody_index(record).expect("mb");
        let mb = s.phys.multibody_joints.get_multibody(mb_index).expect("mb");
        // dof 顺序:每关节 target 与增益。quad 全部关节 kp 100 / kd 3 /
        // tau 23.7;k = 0.01 → 1.0 / 0.03 / 0.237。电机恒在 AngX 槽。
        let def = s.bodies.get(&record.body_hash).expect("身体定义");
        for (dof_i, j) in def.dof_joints().enumerate() {
            let &handle = record.links.get(&j.child).expect("child 连杆");
            let lid = s
                .phys
                .multibody_joints
                .rigid_body_link(handle)
                .expect("link");
            let link = mb.link(lid.id).expect("link 槽");
            let motor = &link.joint.data.motors[3]; // AngX = 3
            assert!((motor.target_pos - record.joint_targets[dof_i]).abs() < 1e-6);
            assert!(
                (motor.stiffness - 100.0 * 0.01).abs() < 1e-5,
                "kp 读回 = {} 期望 1.0",
                motor.stiffness
            );
            assert!(
                (motor.damping - 3.0 * 0.01).abs() < 1e-5,
                "kd 读回 = {} 期望 0.03",
                motor.damping
            );
            assert!(
                (motor.max_force - 23.7 * 0.01).abs() < 1e-5,
                "max_force 读回 = {} 期望 0.237",
                motor.max_force
            );
            assert_eq!(
                motor.model,
                rapier3d::prelude::MotorModel::ForceBased,
                "ForceBased"
            );
        }
    }

    /// 站立探针:跑 n 拍,返回每拍(机身 up·y、机身高度、关节 RMS 误差、
    /// 水平速度模)。默认目标 = default_angle(生成即写进电机)。
    fn stand_probe(
        s: &mut crate::sim::ShardSim,
        hash: BodyHash,
        origin: Vec3,
        n: usize,
    ) -> Vec<(f32, f32, f32, f32)> {
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(origin), None)
            .expect("四足生成");
        assert_quad_above_ground(s, a);
        let observers = [origin + Vec3::new(0.0, 2.0, 0.0)];
        let targets = s.agent_joint_targets(a).expect("目标表").to_vec();
        let mut trace = Vec::with_capacity(n);
        for _ in 0..n {
            s.apply_activation(&observers);
            s.step();
            let _ = s.take_outbox();
            let rec = s.entities.get(&a).expect("root in");
            let rb = s.phys.bodies.get(rec.handle).expect("root rb");
            let up = *rb.rotation() * types::glam::Vec3::Y;
            let height = rb.translation().y;
            let lin = rb.linvel();
            let hspeed = (lin.x * lin.x + lin.z * lin.z).sqrt();
            let q = s.agent_joint_positions(a).expect("q");
            let rms = (q
                .iter()
                .zip(&targets)
                .map(|(x, t)| (x - t) * (x - t))
                .sum::<f32>()
                / q.len() as f32)
                .sqrt();
            trace.push((up.y, height, rms, hspeed));
        }
        trace
    }

    /// K3(AC-B1):平地、默认目标、3600 拍。全程无 NaN;第 120 拍以后
    /// 机身 up·y ≥ 0.95;第 600 拍以后高度标准差 ≤ 1 cm、关节 RMS ≤ 0.15 rad、
    /// 机身平均高度 − 地面顶面 ∈ [0.27, 0.29]。自然睡眠:首次入睡 ≤ 第 300 拍、
    /// 睡眠拍数 ≥ 3000,睡着期间逐拍位移恒 0。另跑一组强制醒着,只报水平
    /// 滑移速度(不设门槛)。
    #[test]
    fn k3_stand_3600_ticks() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let observers = [Vec3::new(24.0, 3.45, 24.0)];
        let targets = s.agent_joint_targets(a).expect("目标表").to_vec();
        let mut min_up = f32::INFINITY;
        let mut max_rms = 0.0f32;
        // 睡眠统计:首次入睡拍号、睡着的拍数与睡着期间的最大逐拍位移。
        let mut first_sleep: Option<usize> = None;
        let mut slept_ticks = 0usize;
        let mut max_sleep_shift = 0.0f32;
        let mut prev_sleeping_pos: Option<rapier3d::prelude::Vector> = None;
        let mut heights_tail: Vec<f32> = Vec::new();
        // 单只四足每拍耗时粗测(不设门槛,B04 再定)。Instant::now 在本 crate
        // 被 clippy.toml 禁用(确定性纪律:测试代码也不许引入墙钟),测量段
        // 单独豁免——计时不进任何判据与断言。
        #[allow(clippy::disallowed_methods)]
        let timing = std::time::Instant::now;
        let mut tick_nanos: Vec<u32> = Vec::new();
        for tick in 0..3600usize {
            let t0 = timing();
            s.apply_activation(&observers);
            s.step();
            let _ = s.take_outbox();
            tick_nanos.push(t0.elapsed().as_nanos() as u32);
            let rec = s.entities.get(&a).expect("root");
            let rb = s.phys.bodies.get(rec.handle).expect("root rb");
            let up = *rb.rotation() * types::glam::Vec3::Y;
            let height = rb.translation().y;
            let pos = rb.translation();
            // 全程无 NaN(判据口径:3600 拍逐拍)。
            assert!(up.is_finite() && height.is_finite(), "拍 {tick}:状态非有限");
            let sleeping = rb.is_sleeping();
            if sleeping {
                if first_sleep.is_none() {
                    first_sleep = Some(tick);
                }
                slept_ticks += 1;
                if let Some(prev) = prev_sleeping_pos {
                    let d = (pos - prev).length();
                    max_sleep_shift = max_sleep_shift.max(d);
                }
                prev_sleeping_pos = Some(pos);
            } else {
                prev_sleeping_pos = None;
            }
            if tick >= 120 {
                min_up = min_up.min(up.y);
            }
            if tick % 600 == 0 {
                println!(
                    "probe tick {tick}: sleeping={} hspeed={:.5}",
                    rb.is_sleeping(),
                    rb.linvel().length()
                );
            }
            if tick >= 600 {
                let q = s.agent_joint_positions(a).expect("q");
                let rms = (q
                    .iter()
                    .zip(&targets)
                    .map(|(x, t)| (x - t) * (x - t))
                    .sum::<f32>()
                    / q.len() as f32)
                    .sqrt();
                max_rms = max_rms.max(rms);
                heights_tail.push(height);
            }
        }
        assert!(
            min_up >= 0.95,
            "第 120 拍以后机身 up·y 最小 {min_up},须 ≥ 0.95"
        );
        let mean = heights_tail.iter().sum::<f32>() / heights_tail.len() as f32;
        let var = heights_tail
            .iter()
            .map(|h| (h - mean) * (h - mean))
            .sum::<f32>()
            / heights_tail.len() as f32;
        let std = var.sqrt();
        assert!(std <= 0.01, "第 600 拍以后高度标准差 {std} m,须 ≤ 0.01");
        assert!(max_rms <= 0.15, "第 600 拍以后关节 RMS {max_rms},须 ≤ 0.15");
        // 站高(新布景判据):平均机身高度 − 地面顶面 ∈ [0.27, 0.29]
        // (§13 demo 设计值 0.28)。
        let stand_height = mean - GROUND_TOP_Y;
        assert!(
            (0.27..=0.29).contains(&stand_height),
            "机身平均高度 − 地面顶面 {stand_height} m,须在 [0.27, 0.29]"
        );
        // 自然睡眠(新布景判据;旧布景整只在地面方块内部,从不入睡):
        // 首次入睡不晚于第 300 拍、睡眠拍数 ≥ 3000;睡着期间逐拍位移恒 0
        // (站对位置后不再是平凡成立)。
        let fs = first_sleep.expect("全程无入睡(布景或睡眠参数问题)");
        assert!(fs <= 300, "首次入睡在第 {fs} 拍,须不晚于第 300 拍");
        assert!(slept_ticks >= 3000, "睡眠拍数 {slept_ticks},须 ≥ 3000");
        assert!(
            max_sleep_shift == 0.0,
            "睡着期间逐拍位移 {max_sleep_shift},恒为 0"
        );
        let sleep_ratio = slept_ticks as f32 / 3600.0;
        println!(
            "K3 实测:up·y 最小 {min_up};高度标准差 {std};关节 RMS 最大 {max_rms};\
             站高 {stand_height};首次入睡第 {fs} 拍;睡眠拍数 {slept_ticks}(占比 {sleep_ratio})"
        );
        // 单只四足每拍耗时粗测(不设门槛,B04 再定):站立段(600..)的均值
        // 与 P95(含 apply_activation/step/outbox 全生产路径,debug 口径见
        // 本测试的构建档)。
        {
            let mut tail = tick_nanos[600..].to_vec();
            tail.sort_unstable();
            let mean: f64 = tail.iter().map(|&v| v as f64).sum::<f64>() / tail.len() as f64;
            let p95 = tail[(tail.len() as f32 * 0.95) as usize] as f64;
            println!("K3 每拍耗时(含激活+step+outbox):mean {mean:.0} ns,P95 {p95:.0} ns");
        }
        // ---- 对照线(判据同段要求):强制醒着,只报水平滑移速度 ----
        let (mut s2, hash2) = quad_sim_with(0.01);
        load_flat_ground(&mut s2);
        let a2 = types::EntityId::from_parts(1, 1);
        s2.spawn_agent(a2, hash2, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成(醒着组)");
        assert_quad_above_ground(&s2, a2);
        let observers2 = [Vec3::new(24.0, 3.45, 24.0)];
        let mut hsum = 0.0f32;
        let mut hn = 0usize;
        for tick in 0..3600usize {
            // 每 2 拍一条微小目标扰动(±1e-4,保证电机一直解算、主体不睡)。
            if tick % 2 == 0 {
                let mut targets2 = s2.agent_joint_targets(a2).expect("targets").to_vec();
                for (i, t) in targets2.iter_mut().enumerate() {
                    *t += ((tick as f32 * 0.01 + i as f32).sin()) * 1e-4;
                }
                s2.apply_agent_joint_targets(a2, &targets2)
                    .expect("合法目标");
            }
            s2.apply_activation(&observers2);
            s2.step();
            let _ = s2.take_outbox();
            if tick >= 600 {
                let rec = s2.entities.get(&a2).expect("root");
                let rb = s2.phys.bodies.get(rec.handle).expect("rb");
                let lin = rb.linvel();
                hsum += (lin.x * lin.x + lin.z * lin.z).sqrt();
                hn += 1;
            }
        }
        let drift = hsum / hn as f32;
        println!("K3 强制醒着:水平滑移速度平均 {drift} m/s(只报数,不设门槛)");
    }

    /// K4:k = 1/100 与 k = 1 两组站立,第 600 拍机身高度相差 ≤ 2 mm。
    #[test]
    fn k4_mass_scale_height_parity() {
        let mut heights = [0.0f32; 2];
        for (i, k) in [0.01f32, 1.0].into_iter().enumerate() {
            let (mut s, hash) = quad_sim_with(k);
            load_flat_ground(&mut s);
            let trace = stand_probe(&mut s, hash, quad_origin(24.0, 24.0), 1200);
            heights[i] = trace[600..].iter().map(|t| t.1).sum::<f32>() / 600.0;
            println!("K4 k = {k}:第 600 拍后平均机身高度 {}", heights[i]);
        }
        let diff = (heights[0] - heights[1]).abs();
        assert!(diff <= 0.002, "两组高度差 {diff} m,须 ≤ 0.002");
    }

    /// K5:四足睡着后改目标 → 下一拍醒来,关节朝新目标动。B01a-R1:前置
    /// 改成**真实的自然入睡**(站到睡着为止,上限 600 拍,超限 panic 并写明
    /// 在等什么;旧版手动 `rb.sleep()` 的构造不再用)。
    #[test]
    fn k5_new_target_wakes_sleeping_quad() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let observers = [Vec3::new(24.0, 3.45, 24.0)];
        // 前置:站到自然入睡为止(上限 600 拍;K3 实测 ~2 秒 = 120 拍量级)。
        let mut asleep_before = false;
        for tick in 0..600usize {
            s.apply_activation(&observers);
            s.step();
            let _ = s.take_outbox();
            let rec = s.entities.get(&a).expect("root");
            if s.phys
                .bodies
                .get(rec.handle)
                .map(|rb| rb.is_sleeping())
                .unwrap_or(false)
            {
                println!("K5:第 {tick} 拍自然入睡");
                asleep_before = true;
                break;
            }
        }
        assert!(
            asleep_before,
            "前置失败:站了 600 拍仍未自然入睡(在等根刚体 is_sleeping;布景或睡眠参数问题)"
        );
        let rec = s.entities.get(&a).expect("root");
        let root_handle = rec.handle;
        let q_before = s.agent_joint_positions(a).expect("q");
        // 改 hip 列目标(−0.85 → −0.6,限位内;其余不动)。
        let mut targets = s.agent_joint_targets(a).expect("targets").to_vec();
        for t in targets.iter_mut().skip(1).step_by(3) {
            *t = -0.6;
        }
        s.apply_agent_joint_targets(a, &targets).expect("合法目标");
        s.apply_activation(&observers);
        s.step();
        let _ = s.take_outbox();
        let awake = s
            .phys
            .bodies
            .get(root_handle)
            .map(|rb| rb.is_sleeping())
            .unwrap_or(false);
        assert!(
            asleep_before,
            "前置失败:站了 600 拍仍未自然入睡(在等根刚体 is_sleeping;布景或睡眠参数问题)"
        );
        assert!(!awake, "改目标后下一拍必须醒来");
        // 关节朝新目标动:hip 关节角向 −0.6(= 旧目标 −0.8536 → −0.6 方向,
        // 即关节角增大)单拍内可见移动(电机醒来即出力)。
        let q_after = s.agent_joint_positions(a).expect("q");
        let new_target = -0.6_f32;
        let moved = q_after[1] - q_before[1];
        assert!(
            moved.abs() > 1e-4
                && (q_after[1] - new_target).abs() < (q_before[1] - new_target).abs(),
            "hip 应朝新目标 {new_target} 动(前 {} 后 {})",
            q_before[1],
            q_after[1]
        );
    }

    /// capsule 的哈希(从注册表查名字)。
    fn capsule_hash_of(s: &crate::sim::ShardSim) -> BodyHash {
        s.bodies.hash_of("capsule").expect("capsule 已装")
    }

    /// 拷贝当前注册表(restore_with_bodies 需要同源身体)。
    fn fresh_registry_clone(s: &crate::sim::ShardSim) -> BodyRegistry {
        let mut r = BodyRegistry::new();
        for h in s.bodies.hashes() {
            let def = s.bodies.get(&h).expect("def").clone();
            r.insert_checked(def).expect("同哈希幂等");
        }
        r
    }

    /// K9/K10 共用:tick 拍的正弦目标序列(限位内的中幅摆动;dof 顺序 =
    /// 每腿 [lat, hip, knee],限位循环取三档)。
    fn current_sine_targets(tick: usize) -> Vec<f32> {
        let mut targets = Vec::with_capacity(12);
        for i in 0..12 {
            let lim = [(-0.8, 0.8), (-1.6, 1.6), (0.0, 2.6)][i % 3];
            let mid = (lim.0 + lim.1) / 2.0;
            let amp = (lim.1 - lim.0) / 2.0 * 0.5;
            let phase = (tick as f32 * 0.02 + i as f32 * 0.7).sin();
            targets.push(mid + amp * phase);
        }
        targets
    }

    /// K7(AC-B5):长度不符、NaN、inf、对非主体、对胶囊(dof 0)各发一条,
    /// 每条都类型化拒绝(世界不变),拒绝计数 5。
    /// 世界不变性用"非法命令前后各取一次快照逐位相同"直接证。
    #[test]
    fn k7_rejected_targets_leave_world_untouched() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let observers = [Vec3::new(24.0, 3.45, 24.0)];
        for _ in 0..30 {
            s.apply_activation(&observers);
            s.step();
            let _ = s.take_outbox();
        }
        // 五条非法全部走命令路径(apply_command 段计数;与 AgentIntent
        // 的拒绝口径同层)。一条 apply_prefix 同拍带 5 条命令。
        let mk_cmd =
            |seq: u64, entity: types::EntityId, targets: Vec<f32>| messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: seq,
                payload: messaging::InputPayload::Command(
                    messaging::SimCommand::AgentJointTargets { entity, targets },
                ),
            };
        // ① 长度不符(11 项)。
        // ② NaN。③ inf。④ 非主体(普通实体,同拍先生成)。⑤ 胶囊(dof 0,
        // 同拍先生成)。
        let plain = types::EntityId::from_parts(1, 2);
        let cap = types::EntityId::from_parts(1, 3);
        s.apply_prefix(&[
            messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 7,
                payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                    entity: plain,
                    state: types::EntityState {
                        translation: Vec3::new(20.0, 3.0, 20.0),
                        ..types::EntityState::default()
                    },
                    desc: types::EntityDesc::default(),
                }),
            },
            messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 8,
                payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnAgent {
                    entity: cap,
                    body: capsule_hash_of(&s),
                    state: types::EntityState {
                        translation: Vec3::new(28.0, 1.95, 28.0),
                        ..types::EntityState::default()
                    },
                }),
            },
            mk_cmd(11, a, vec![0.0; 11]),
            mk_cmd(12, a, {
                let mut v = vec![0.0; 12];
                v[3] = f32::NAN;
                v
            }),
            mk_cmd(13, a, {
                let mut v = vec![0.0; 12];
                v[4] = f32::INFINITY;
                v
            }),
            mk_cmd(14, plain, vec![0.0; 12]),
            mk_cmd(15, cap, vec![0.0; 1]),
        ]);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.agent_joint_targets_rejected, 5, "五条非法各 +1");
        // 内核侧的错误类型逐一核对(世界不变性下面用快照对照证)。
        assert_eq!(
            s.apply_agent_joint_targets(a, &[0.0; 11]),
            Err(crate::agent::AgentJointTargetsError::LengthMismatch)
        );
        assert_eq!(
            s.apply_agent_joint_targets(
                a,
                &[
                    0.0,
                    0.0,
                    0.0,
                    f32::NAN,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
                ]
            ),
            Err(crate::agent::AgentJointTargetsError::NonFinite)
        );
        assert_eq!(
            s.apply_agent_joint_targets(
                a,
                &[
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    f32::INFINITY,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
                ]
            ),
            Err(crate::agent::AgentJointTargetsError::NonFinite)
        );
        assert_eq!(
            s.apply_agent_joint_targets(plain, &[0.0; 12]),
            Err(crate::agent::AgentJointTargetsError::NotAgent)
        );
        assert_eq!(
            s.apply_agent_joint_targets(cap, &[0.0; 1]),
            Err(crate::agent::AgentJointTargetsError::NoJoints)
        );
        // 计数只在命令路径(apply_command 段;内核函数只报类型——
        // 放行第 3.4 节"计数在 apply_command 分支统一 +1")。
        // ---- 世界不变性(AC-B5):同样的布景 + 两条 spawn,对照组不发
        // 非法命令。两线各自 step 一拍并 take_outbox(把拒绝计数取走——
        // 快照含 Outbox 计数,计数是遥测不是世界状态;取走后两线快照必须
        // 逐位相同)。世界不变性由内核函数保证:apply_agent_joint_targets
        // 的全部拒绝分支在任何写状态之前早退(长度/有限检查在钳制与写
        // record 之前)。----
        let snap1 = {
            let _ = s.take_outbox();
            s.snapshot()
        };
        let (mut s2, hash2) = quad_sim_with(0.01);
        load_flat_ground(&mut s2);
        s2.spawn_agent(a, hash2, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成(对照)");
        assert_quad_above_ground(&s2, a);
        for _ in 0..30 {
            s2.apply_activation(&observers);
            s2.step();
            let _ = s2.take_outbox();
        }
        // 对照线:同样两条 spawn,不发非法命令。
        s2.apply_prefix(&[
            messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 7,
                payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                    entity: plain,
                    state: types::EntityState {
                        translation: Vec3::new(20.0, 3.0, 20.0),
                        ..types::EntityState::default()
                    },
                    desc: types::EntityDesc::default(),
                }),
            },
            messaging::StampedInput {
                apply_at: types::Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 8,
                payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnAgent {
                    entity: cap,
                    body: capsule_hash_of(&s2),
                    state: types::EntityState {
                        translation: Vec3::new(28.0, 1.95, 28.0),
                        ..types::EntityState::default()
                    },
                }),
            },
        ]);
        s2.step();
        let _ = s2.take_outbox();
        let snap_ctrl = s2.snapshot();
        assert_eq!(snap1, snap_ctrl, "非法命令后世界与不发命令的对照组逐位相同");
    }

    /// K8:超限目标 → 记录里的目标 = 限位值,钳制计数 = 超限个数;
    /// 对照:限位内目标不计数。
    #[test]
    fn k8_clamp_targets_to_limits() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let out0 = s.take_outbox();
        assert_eq!(out0.agent_joint_targets_clamped, 0);
        // 全部超上限(+100):quad 12 关节全有限位 → 全部被钳。走命令路径
        // (钳制计数在 apply_command 段)。
        s.apply_prefix(&[messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 21,
            payload: messaging::InputPayload::Command(messaging::SimCommand::AgentJointTargets {
                entity: a,
                targets: vec![100.0; 12],
            }),
        }]);
        s.step();
        let targets = s.agent_joint_targets(a).expect("targets").to_vec();
        let def = BodyDef::from_toml_str(quad_toml()).expect("def");
        for (i, j) in def.dof_joints().enumerate() {
            assert_eq!(
                targets[i],
                j.kind.params().limits.expect("quad 全关节有限位").upper,
                "目标 = 限位上限(dof {} = {})",
                i,
                j.child
            );
        }
        let out = s.take_outbox();
        assert_eq!(out.agent_joint_targets_clamped, 12, "钳制计数 = 超限个数");
        // 对照:限位内目标(全 0)不计数。只取一次 outbox(B01a-R1:连取
        // 两次时第二次永远为空,"限位内不计数"成了空转断言)。
        s.apply_prefix(&[messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 22,
            payload: messaging::InputPayload::Command(messaging::SimCommand::AgentJointTargets {
                entity: a,
                targets: vec![0.0; 12],
            }),
        }]);
        s.step();
        let out2 = s.take_outbox();
        assert_eq!(out2.agent_joint_targets_clamped, 0, "限位内不计数");
    }

    /// K9(AC-B6):固定目标序列(各关节正弦,每 2 拍一条,600 拍)跑两遍,
    /// 状态哈希(快照字节)逐位相同。
    #[test]
    fn k9_joint_target_sequence_deterministic() {
        let run = || {
            let (mut s, hash) = quad_sim_with(0.01);
            load_flat_ground(&mut s);
            let a = types::EntityId::from_parts(1, 1);
            s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
                .expect("四足生成");
            assert_quad_above_ground(&s, a);
            let observers = [Vec3::new(24.0, 3.45, 24.0)];
            for tick in 0..600usize {
                if tick % 2 == 0 {
                    s.apply_agent_joint_targets(a, &current_sine_targets(tick))
                        .expect("合法");
                }
                s.apply_activation(&observers);
                s.step();
                let _ = s.take_outbox();
            }
            s.snapshot()
        };
        let snap_a = run();
        let snap_b = run();
        assert_eq!(snap_a, snap_b, "同输入两遍逐位相同(AC-B6)");
    }

    /// K10(AC-B6):第 300 拍取快照 → 恢复 → 续跑到 600 拍,与不中断的
    /// 哈希逐位相同;恢复后 joint_targets 与电机目标一致。
    #[test]
    fn k10_snapshot_restore_continuation() {
        let build = |interrupt: bool| {
            let (mut s, hash) = quad_sim_with(0.01);
            load_flat_ground(&mut s);
            let a = types::EntityId::from_parts(1, 1);
            s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
                .expect("四足生成");
            assert_quad_above_ground(&s, a);
            let observers = [Vec3::new(24.0, 3.45, 24.0)];
            let mut restored: Option<crate::sim::ShardSim> = None;
            for tick in 0..600usize {
                if tick % 2 == 0 {
                    let sim = restored.as_mut().unwrap_or(&mut s);
                    sim.apply_agent_joint_targets(a, &current_sine_targets(tick))
                        .expect("合法");
                }
                let sim = restored.as_mut().unwrap_or(&mut s);
                sim.apply_activation(&observers);
                sim.step();
                let _ = sim.take_outbox();
                if tick == 300 && interrupt {
                    let bytes = sim.snapshot();
                    let bodies = fresh_registry_clone(sim);
                    let r = crate::sim::ShardSim::restore_with_bodies(
                        &bytes,
                        Box::new(SimpleRegistry::builtin()),
                        bodies,
                    )
                    .expect("restore");
                    // 恢复后 joint_targets 与电机目标一致。
                    let record = r.agents.get(&a).expect("主体登记");
                    let targets = record.joint_targets.clone();
                    let mb_index = r.agent_multibody_index(record).expect("mb");
                    let mb = r.phys.multibody_joints.get_multibody(mb_index).expect("mb");
                    let def = r.bodies.get(&record.body_hash).expect("def");
                    for (dof_i, j) in def.dof_joints().enumerate() {
                        let &handle = record.links.get(&j.child).expect("child");
                        let lid = r
                            .phys
                            .multibody_joints
                            .rigid_body_link(handle)
                            .expect("link");
                        let link = mb.link(lid.id).expect("link");
                        assert!(
                            (link.joint.data.motors[3].target_pos - targets[dof_i]).abs() < 1e-6,
                            "恢复后电机目标 = joint_targets(dof {dof_i})"
                        );
                    }
                    restored = Some(r);
                }
            }
            restored.unwrap_or(s).snapshot()
        };
        let uninterrupted = build(false);
        let interrupted = build(true);
        assert_eq!(uninterrupted, interrupted, "快照恢复续跑与不中断逐位相同");
    }

    /// K11:多连杆主体被推过分片边界 → fail-stop,信息含 "B03"。
    /// (`should_panic`)布景:四足生成到相邻分片的 chunk 中,step 的迁移
    /// 检测(核心区外)即触发。
    #[test]
    #[should_panic(expected = "B03")]
    fn k11_multi_link_migration_fail_stop() {
        let (mut s, hash) = quad_sim_with(0.01);
        // 分片 = 8×8 chunk(0..128);四足放在 x = 132(相邻分片 (1,0))→
        // 不在本分片核心区。生成路径不查归属(§3 与 SpawnDynamic 同口径),
        // 迁移检测在 step 内触发。
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(
            a,
            hash,
            quad_spawn_state(Vec3::new(132.0, 0.448, 24.0)),
            None,
        )
        .expect("四足生成");
        s.apply_activation(&[Vec3::new(132.0, 2.34, 24.0)]);
        s.step();
    }

    /// K12:一格方块放进四足某条腿占的格 → 暂缓写入(A05 的规则对非根
    /// 连杆也成立);主体移除后补写生效。布景:机身绕 z 转 90° 横躺、中心
    /// 放在格 24 的右缘(24.9)——腿链水平伸向 +x,shank 中心格 (25,·,·)
    /// 与机身格 (24,·,·) 分离(站立姿态下所有连杆共享同一竖直格,判据
    /// 区分不出"非根连杆在表里")。生成后不步进:暂缓判定只依赖形状表
    /// 里的连杆位姿,FK 摆位已把连杆放好。
    #[test]
    fn k12_deferred_set_block_on_leg_cell() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(
            a,
            hash,
            types::EntityState {
                translation: Vec3::new(24.9, 2.0, 24.5),
                rotation: types::glam::Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
                ..types::EntityState::default()
            },
            None,
        )
        .expect("四足生成");
        // 非根连杆(shank,links[3] = leg0)的中心格。
        let record = s.agents.get(&a).expect("主体登记");
        let shank_name = s.bodies.get(&record.body_hash).expect("def").links[3]
            .name
            .clone();
        let &shank_handle = record.links.get(&shank_name).expect("shank");
        let rb = s.phys.bodies.get(shank_handle).expect("shank rb");
        let p = crate::conv::rapier_to_vec3(rb.translation());
        let shank_cell =
            types::BlockPos::new(p.x.floor() as i32, p.y.floor() as i32, p.z.floor() as i32);
        // 布景自检:shank 格与机身格不同(判据的区分前提)。
        let root_rb = s
            .phys
            .bodies
            .get(s.entities.get(&a).unwrap().handle)
            .expect("root rb");
        let rp = crate::conv::rapier_to_vec3(root_rb.translation());
        let root_cell = types::BlockPos::new(
            rp.x.floor() as i32,
            rp.y.floor() as i32,
            rp.z.floor() as i32,
        );
        assert_ne!(
            shank_cell, root_cell,
            "布景自检:shank 格必须与机身格分离(shank 在 {p:?},root 在 {rp:?})"
        );
        // SetBlock 写进 shank 格 → 走暂缓写入路径。形状表 = 调度器口径
        // (拍首注入上一拍末的主体连杆表)。
        s.set_agent_shape_table(s.agent_shape_entries());
        s.apply_prefix(&[messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 9,
            payload: messaging::InputPayload::Command(messaging::SimCommand::SetBlock {
                pos: shank_cell,
                block: types::BlockId(1),
            }),
        }]);
        s.step();
        let _ = s.take_outbox();
        assert!(
            s.deferred_set_blocks.contains_key(&shank_cell),
            "腿占的格的 SetBlock 被暂缓(非根连杆同规则)"
        );
        // 移除主体后,暂缓写入生效(A05 的规则)。调度器口径:移除后注入
        // 空表 → 拍首 flush 补写。
        s.despawn_agent(a);
        s.set_agent_shape_table(crate::agent_shape::AgentShapeTable::default());
        s.flush_agent_deferred_writes();
        assert_eq!(
            s.voxel().get_block(shank_cell),
            types::BlockId(1),
            "主体移除后暂缓写入生效"
        );
    }

    /// K13(B01a-R1 P2):多连杆主体掉进虚空 → **整只移除**:刚体数、碰撞体
    /// 数、multibody 数回到生成前;`agents`、`entities` 里没有它;上报与
    /// 单连杆同口径(`despawned` + `void_removed` 各一条、`void_removals`
    /// +1)。布景:没有地面(不 load_flat_ground),四足生成后自由下落,
    /// y 越过 WORLD_Y_MIN − void_despawn_depth_m(−64 − 16 = −80)即触发。
    /// 对照线 = 单连杆胶囊同场景(口径基准)。
    #[test]
    fn k13_void_removal_removes_whole_multibody() {
        // ---- 单连杆对照线(口径基准;先跑,拿它的上报形状)----
        let (mut sc, caphash) = test_sim();
        let cap = types::EntityId::from_parts(1, 1);
        sc.spawn_agent(
            cap,
            caphash,
            types::EntityState {
                translation: Vec3::new(24.0, 20.0, 24.0),
                ..types::EntityState::default()
            },
            None,
        )
        .expect("capsule 生成");
        sc.apply_activation(&[Vec3::new(24.0, 20.0, 24.0)]);
        let _ = sc.take_outbox();
        let mut fell = false;
        for _ in 0..1200usize {
            sc.step();
            sc.run_transient_lifecycle();
            let out = sc.take_outbox();
            if out.void_removed.contains(&cap) {
                fell = true;
                break;
            }
        }
        assert!(
            fell,
            "对照线:胶囊 1200 拍内应越过虚空阈值(在等 outbox.void_removed)"
        );
        // ---- 多连杆主体同场景 ----
        let (mut s, hash) = quad_sim_with(0.01);
        let bodies_before = s.phys.bodies.len();
        let colliders_before = s.phys.colliders.len();
        let mb_before = s.phys.multibody_joints.iter().count();
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(Vec3::new(24.0, 20.0, 24.0)), None)
            .expect("四足生成");
        assert_eq!(s.phys.bodies.len(), bodies_before + 13, "布景自检:13 刚体");
        s.apply_activation(&[Vec3::new(24.0, 20.0, 24.0)]);
        let _ = s.take_outbox();
        let mut removed = false;
        for _ in 0..1200usize {
            s.step();
            s.run_transient_lifecycle();
            let out = s.take_outbox();
            if out.void_removed.contains(&a) {
                removed = true;
                assert!(
                    out.despawned.contains(&a),
                    "上报与单连杆同口径:despawned 也有一条"
                );
                break;
            }
        }
        assert!(
            removed,
            "四足 1200 拍内应整只越过虚空阈值被移除(在等 outbox.void_removed)"
        );
        // 全部连杆拆除:三项计数回到生成前;agents/entities 无它。
        assert_eq!(s.phys.bodies.len(), bodies_before, "刚体数回到生成前");
        assert_eq!(
            s.phys.colliders.len(),
            colliders_before,
            "碰撞体数回到生成前"
        );
        assert_eq!(
            s.phys.multibody_joints.iter().count(),
            mb_before,
            "multibody 数回到生成前"
        );
        assert!(!s.agents.contains_key(&a), "agents 表无它");
        assert!(!s.entities.contains_key(&a), "entities 表无它");
    }

    /// K14(B01a-R1 P2):形状 offset 进碰撞体——shank 是 compound(胶囊 +
    /// 带偏移 [0, −0.1065, 0] 的脚球),连杆局部系 AABB 最低点 =
    /// −(0.1065 + 0.022);对照 thigh(单形状无偏移胶囊)= −(0.1065 + 0.02)。
    /// 变异"多形状分支忽略 offset"使脚球回到原点,最低点变 −(0.1065+0.012),
    /// 本测试红。
    #[test]
    fn k14_shape_offset_reaches_collider() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let record = s.agents.get(&a).expect("主体登记");
        let def = s.bodies.get(&record.body_hash).expect("def").clone();
        // links[3] = leg0 的 shank;links[2] = leg0 的 thigh(顺序同资产)。
        let shank_name = def.links[3].name.clone();
        let thigh_name = def.links[2].name.clone();
        // 布景自检:shank 有两个形状(第二形状带 offset),thigh 单形状。
        assert_eq!(def.links[3].shapes.len(), 2, "shank = 胶囊 + 脚球");
        assert_eq!(def.links[2].shapes.len(), 1, "thigh = 单胶囊");
        let local_min_y = |name: &str| -> f32 {
            let &h = record.links.get(name).expect("连杆 handle");
            let rb = s.phys.bodies.get(h).expect("连杆刚体");
            // 每连杆一个 collider(多形状 = 单 compound)。
            assert_eq!(rb.colliders().len(), 1, "连杆 {} 单碰撞体", name);
            let collider = s.phys.colliders.get(rb.colliders()[0]).expect("连杆碰撞体");
            collider.shape().compute_local_aabb().mins.y
        };
        let shank_min = local_min_y(&shank_name);
        let thigh_min = local_min_y(&thigh_name);
        assert!(
            (shank_min - -(0.1065 + 0.022)).abs() <= 1e-4,
            "shank 连杆局部 AABB 最低 {shank_min},期望 −(0.1065+0.022) = {}",
            -(0.1065 + 0.022)
        );
        assert!(
            (thigh_min - -(0.1065 + 0.02)).abs() <= 1e-4,
            "thigh 连杆局部 AABB 最低 {thigh_min},期望 −(0.1065+0.02) = {}",
            -(0.1065 + 0.02)
        );
    }

    // ---- B01b:失控防护(§6;任务书第 4 节 G1–G6)----

    /// 站立四足布景(站稳 = 入睡或 300 拍;B01a-R1 同款地面与出生点)。
    /// 返回 (sim, body hash, agent id);观察者已注册。
    fn stand_quad() -> (crate::sim::ShardSim, BodyHash, types::EntityId) {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let observers = [Vec3::new(24.0, 3.45, 24.0)];
        for tick in 0..300usize {
            s.apply_activation(&observers);
            s.step();
            s.run_transient_lifecycle();
            let _ = s.take_outbox();
            let asleep = s
                .phys
                .bodies
                .get(s.entities.get(&a).expect("在场").handle)
                .expect("根刚体")
                .is_sleeping();
            if asleep {
                break;
            }
            let _ = tick;
        }
        (s, hash, a)
    }

    /// 超速注入 helper(G2/G2b/G4 共用):把四足 multibody 根连杆的广义
    /// 速度(前 3 项 = 根线速度 x/y/z)直写为 `v` m/s 朝上。multibody 非根
    /// 连杆的刚体线速度每步由 forward_kinematics 从广义速度重算覆盖,直接
    /// `set_linvel` 会被丢弃——注入必须落在广义速度这一层(与防护的检查
    /// 对象一致:步进后 FK 把根速度写回刚体,防护连杆循环读根 = 读到注入值)。
    /// 布景四足已站立(可能已入睡),注入前先唤醒。
    fn inject_root_overspeed(s: &mut crate::sim::ShardSim, a: types::EntityId, v: f32) {
        let record = s.agents.get(&a).expect("主体登记").clone();
        let root_handle = s.entities.get(&a).expect("在场").handle;
        s.phys
            .bodies
            .get_mut(root_handle)
            .expect("根刚体")
            .wake_up(true);
        let link_id = s
            .phys
            .multibody_joints
            .rigid_body_link(root_handle)
            .expect("根连杆在 multibody");
        let mb = s
            .phys
            .multibody_joints
            .get_multibody_mut(link_id.multibody)
            .expect("multibody 存在");
        let mut gv = mb.generalized_velocity_mut();
        gv[0] = 0.0;
        gv[1] = v;
        gv[2] = 0.0;
        let _ = &record;
    }

    /// G1:`agent_runaway_reason` 的纯判定(任务书第 4 节表)。站稳的四足 →
    /// None;非根连杆线速度 NaN → NonFinite;位置 NaN → NonFinite;线速度
    /// 250 → OverSpeed;线速度 150 → None(对照)。
    #[test]
    fn g1_runaway_reason_pure_judgement() {
        let (mut s, _hash, a) = stand_quad();
        assert_eq!(s.agent_runaway_reason(a), None, "站稳 → None");

        // 布景自检:13 连杆、12 自由度(非根连杆存在)。
        let record = s.agents.get(&a).expect("主体登记");
        assert_eq!(record.links.len(), 13);
        // 取一个非根连杆的句柄(links 按名字序;根 = BodyDef.links[0] 的名字)。
        let def = s.bodies.get(&record.body_hash).expect("def").clone();
        let root_name = def.links[0].name.clone();
        let non_root = def
            .links
            .iter()
            .find(|l| l.name != root_name)
            .expect("非根连杆存在")
            .name
            .clone();
        let &h = record.links.get(&non_root).expect("句柄在表");

        // 直接在原 sim 上逐案改刚体状态(每案改完恢复,案间不互相污染)。
        // 案 1:非根连杆线速度 NaN。
        s.phys.bodies.get_mut(h).expect("连杆刚体").set_linvel(
            crate::conv::vec3_to_rapier(Vec3::new(f32::NAN, 0.0, 0.0)),
            true,
        );
        assert_eq!(
            s.agent_runaway_reason(a),
            Some(RunawayReason::NonFinite),
            "非根连杆线速度 NaN → NonFinite"
        );

        // 案 1 收尾:速度恢复有限(案间不互相污染;检查顺序按任务书)。
        let &h1r = s
            .agents
            .get(&a)
            .expect("主体")
            .links
            .get(&non_root)
            .unwrap();
        s.phys
            .bodies
            .get_mut(h1r)
            .expect("连杆刚体")
            .set_linvel(crate::conv::vec3_to_rapier(Vec3::ZERO), true);

        // 案 2:非根连杆位置 NaN。
        let &h2 = s
            .agents
            .get(&a)
            .expect("主体")
            .links
            .get(&non_root)
            .unwrap();
        s.phys
            .bodies
            .get_mut(h2)
            .expect("连杆刚体")
            .set_translation(
                crate::conv::vec3_to_rapier(Vec3::new(f32::NAN, 1.0, 24.0)),
                true,
            );
        assert_eq!(
            s.agent_runaway_reason(a),
            Some(RunawayReason::NonFinite),
            "连杆位置 NaN → NonFinite"
        );

        // 案 2 收尾:位置恢复有限。
        let &h2r = s
            .agents
            .get(&a)
            .expect("主体")
            .links
            .get(&non_root)
            .unwrap();
        s.phys
            .bodies
            .get_mut(h2r)
            .expect("连杆刚体")
            .set_translation(
                crate::conv::vec3_to_rapier(Vec3::new(24.0, 1.5, 24.0)),
                true,
            );

        // 案 3:线速度 250 → OverSpeed。
        let &h3 = s
            .agents
            .get(&a)
            .expect("主体")
            .links
            .get(&non_root)
            .unwrap();
        s.phys.bodies.get_mut(h3).expect("连杆刚体").set_linvel(
            crate::conv::vec3_to_rapier(Vec3::new(250.0, 0.0, 0.0)),
            true,
        );
        assert_eq!(
            s.agent_runaway_reason(a),
            Some(RunawayReason::OverSpeed),
            "连杆线速度 250 m/s → OverSpeed"
        );

        // 案 4(对照):线速度 150 → None(150 < 200 阈值;NaN 已被覆盖为有限值)。
        let &h4 = s
            .agents
            .get(&a)
            .expect("主体")
            .links
            .get(&non_root)
            .unwrap();
        s.phys.bodies.get_mut(h4).expect("连杆刚体").set_linvel(
            crate::conv::vec3_to_rapier(Vec3::new(150.0, 0.0, 0.0)),
            true,
        );
        assert_eq!(
            s.agent_runaway_reason(a),
            None,
            "连杆线速度 150 m/s(阈值下)→ None(对照)"
        );
    }

    /// G2:经 `step()` 触发。注入方式:把非根连杆线速度直写为 300 m/s
    /// (先经对照线证明:防护关掉时步进后确实读得到 > 200——见
    /// `g2b_guard_off_reads_over_speed`)。断言:这一拍结束时刚体数、碰撞
    /// 体数、multibody 数回到生成前;`void_removed`、`despawned` 各有它;
    /// `agent_runaway_removed == 1`。
    #[test]
    fn g2_step_guard_removes_overspeed_quad() {
        let (mut s, _hash, a) = stand_quad();
        let bodies_before = {
            let (mut s0, _h) = quad_sim_with(0.01);
            load_flat_ground(&mut s0);
            s0.phys.bodies.len()
        };
        let colliders_before = {
            let (mut s0, _h) = quad_sim_with(0.01);
            load_flat_ground(&mut s0);
            s0.phys.colliders.len()
        };
        let mb_before = {
            let (mut s0, _h) = quad_sim_with(0.01);
            load_flat_ground(&mut s0);
            s0.phys.multibody_joints.iter().count()
        };
        let observers = [Vec3::new(24.0, 3.45, 24.0)];
        s.apply_activation(&observers);
        // 注入:根连杆广义速度 300 m/s 朝上(> 200 阈值;helper,见其文档)。
        inject_root_overspeed(&mut s, a, 300.0);
        s.step();
        // 同拍内已被移除:agents/entities 无它,三项计数回到生成前。
        assert!(!s.agents.contains_key(&a), "失控主体同拍移除(agents)");
        assert!(!s.entities.contains_key(&a), "失控主体同拍移除(entities)");
        assert_eq!(s.phys.bodies.len(), bodies_before, "刚体数回到生成前");
        assert_eq!(
            s.phys.colliders.len(),
            colliders_before,
            "碰撞体数回到生成前"
        );
        assert_eq!(
            s.phys.multibody_joints.iter().count(),
            mb_before,
            "multibody 数回到生成前"
        );
        let out = s.take_outbox();
        assert!(out.void_removed.contains(&a), "void_removed 有失控主体一条");
        assert!(out.despawned.contains(&a), "despawned 有失控主体一条");
        assert_eq!(out.agent_runaway_removed, 1, "agent_runaway_removed == 1");
    }

    /// G2 的对照线(任务书:先确认不加防护时步进后确实读得到 > 200):关掉
    /// 防护开关,同注入后步进,主体仍在场且连杆速度 > 200(物理解算没有把
    /// 它拉回阈值内)。
    #[test]
    fn g2b_guard_off_reads_over_speed() {
        let (mut s, _hash, a) = stand_quad();
        s.agent_runaway_guard_enabled = false;
        inject_root_overspeed(&mut s, a, 300.0);
        s.step();
        // 读根连杆(注入落在根;防护连杆循环先查根)。
        let root_h = s.entities.get(&a).expect("主体在场(无防护)").handle;
        let v = crate::conv::rapier_to_vec3(s.phys.bodies.get(root_h).expect("根刚体").linvel())
            .length();
        assert!(
            v > crate::agent::AGENT_V_GUARD,
            "无防护时步进后连杆速度 {v} 应 > 200(在等超速读数)"
        );
    }

    /// G3:单连杆胶囊同样被防护(同 G2 口径)。
    #[test]
    fn g3_capsule_also_guarded() {
        let (mut s, caphash) = test_sim();
        load_flat_ground(&mut s);
        let bodies_before = s.phys.bodies.len();
        let colliders_before = s.phys.colliders.len();
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(
            a,
            caphash,
            types::EntityState {
                translation: Vec3::new(24.0, 1.95, 24.0),
                ..types::EntityState::default()
            },
            None,
        )
        .expect("capsule 生成");
        let observers = [Vec3::new(24.0, 3.0, 24.0)];
        s.apply_activation(&observers);
        let _ = s.take_outbox();
        // 注入:根(唯一)连杆线速度 300 m/s。
        let h = s.entities.get(&a).expect("在场").handle;
        s.phys.bodies.get_mut(h).expect("胶囊刚体").set_linvel(
            crate::conv::vec3_to_rapier(Vec3::new(300.0, 0.0, 0.0)),
            true,
        );
        s.step();
        assert!(!s.agents.contains_key(&a), "胶囊失控同拍移除");
        assert_eq!(s.phys.bodies.len(), bodies_before, "刚体数回到生成前");
        assert_eq!(
            s.phys.colliders.len(),
            colliders_before,
            "碰撞体数回到生成前"
        );
        let out = s.take_outbox();
        assert!(out.void_removed.contains(&a));
        assert_eq!(out.agent_runaway_removed, 1);
    }

    /// G4:其他实体继续运行。同一分片:第一只四足失控被移除后,另一只四足
    /// 与一个动态方块继续。再跑 600 拍:无 panic,第二只仍站着(站高
    /// ∈ [0.27, 0.29]),`agent_runaway_removed` 累计 1。
    #[test]
    fn g4_other_entities_keep_running() {
        let (mut s, hash, a) = stand_quad();
        // 第二只四足(不同 EntityId;间隔 8 m 避免碰撞)与一个动态方块。
        let b = types::EntityId::from_parts(1, 2);
        s.spawn_agent(b, hash, quad_spawn_state(quad_origin(32.0, 24.0)), None)
            .expect("第二只四足生成");
        assert_quad_above_ground(&s, b);
        let block = types::EntityId::from_parts(1, 3);
        s.spawn_entity_for_test(block, Vec3::new(32.0 + 4.0, GROUND_TOP_Y + 2.0, 24.0));
        let observers = [Vec3::new(28.0, 3.45, 24.0)];
        s.apply_activation(&observers);
        let _ = s.take_outbox();
        // 第一只失控(helper:根广义速度 300 m/s 朝上)。
        inject_root_overspeed(&mut s, a, 300.0);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.agent_runaway_removed, 1, "第一只被移除一次");
        assert!(!s.agents.contains_key(&a), "第一只已移除");
        assert!(s.agents.contains_key(&b), "第二只仍在");
        assert!(s.entities.contains_key(&block), "方块仍在");
        // 再跑 600 拍(生产节奏)。
        let mut runaway_total = out.agent_runaway_removed;
        for _ in 0..600usize {
            s.apply_activation(&observers);
            s.step();
            s.run_transient_lifecycle();
            let o = s.take_outbox();
            runaway_total += o.agent_runaway_removed;
        }
        assert_eq!(runaway_total, 1, "600 拍内不再有失控移除(总数恒 1)");
        // 第二只仍站着:取 100 拍窗口的机身平均高度 − 地面顶面 ∈ [0.27, 0.29]。
        let mut heights = Vec::new();
        for _ in 0..100usize {
            s.apply_activation(&observers);
            s.step();
            s.run_transient_lifecycle();
            let _ = s.take_outbox();
            let p = s.agent_root_position(b).expect("第二只在场");
            heights.push(p.y);
        }
        let mean = heights.iter().sum::<f32>() / heights.len() as f32;
        let stand_height = mean - GROUND_TOP_Y;
        assert!(
            (0.27..=0.29).contains(&stand_height),
            "第二只站高 {stand_height} m,须在 [0.27, 0.29]"
        );
    }

    /// G5:确定性。G4 的布景跑两遍,快照逐位相同;移除那一拍取快照、恢复、
    /// 续跑到底,与不中断的逐位相同。
    #[test]
    fn g5_determinism_with_runaway_removal() {
        let observers = [Vec3::new(28.0, 3.45, 24.0)];
        let setup = || {
            let (mut s, hash) = quad_sim_with(0.01);
            load_flat_ground(&mut s);
            let a = types::EntityId::from_parts(1, 1);
            s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
                .expect("四足生成");
            let b = types::EntityId::from_parts(1, 2);
            s.spawn_agent(b, hash, quad_spawn_state(quad_origin(32.0, 24.0)), None)
                .expect("第二只四足生成");
            let block = types::EntityId::from_parts(1, 3);
            s.spawn_entity_for_test(block, Vec3::new(32.0 + 4.0, GROUND_TOP_Y + 2.0, 24.0));
            (s, a)
        };
        // 稳站(与 stand_quad 同口径:入睡或 300 拍;两只同时稳站)。
        let run = || {
            let (mut s, a) = setup();
            for _ in 0..300usize {
                s.apply_activation(&observers);
                s.step();
                s.run_transient_lifecycle();
                let _ = s.take_outbox();
                let asleep = s
                    .phys
                    .bodies
                    .get(s.entities.get(&a).expect("在场").handle)
                    .expect("根刚体")
                    .is_sleeping();
                if asleep {
                    break;
                }
            }
            (s, a)
        };
        let (mut golden, a) = run();
        // 注入并步进:失控拍(helper:根广义速度,与 G2 同注入层)。
        inject_root_overspeed(&mut golden, a, 300.0);
        golden.apply_activation(&observers);
        golden.step();
        let out = golden.take_outbox();
        assert_eq!(out.agent_runaway_removed, 1, "布景自检:移除发生在本拍");
        golden.run_transient_lifecycle();
        let _ = golden.take_outbox();
        // 上面的恢复注入写得绕,直接换成对齐 K13 的做法:整段推倒,
        // 用"与不中断逐位相同"的直写对照(见下)。真正的断言在 continue_run。
        let (mut interrupted, a2) = run();
        let _ = a2;
        // 重放到移除拍(取快照在 run_transient_lifecycle 之后,恢复线从
        // "移除拍完全收尾"的状态续跑)。
        inject_root_overspeed(&mut interrupted, a2, 300.0);
        interrupted.apply_activation(&observers);
        interrupted.step();
        let o = interrupted.take_outbox();
        assert_eq!(o.agent_runaway_removed, 1);
        interrupted.run_transient_lifecycle();
        let _ = interrupted.take_outbox();
        let bytes = interrupted.snapshot();
        // 恢复注册表 = quad + capsule(与布景同源;quad_registry() 每次
        // 构造新实例,哈希由内容决定,逐位一致)。
        let mut resumed = crate::sim::ShardSim::restore_with_bodies(
            &bytes,
            Box::new(SimpleRegistry::builtin()),
            {
                let (bodies, _hash) = quad_registry();
                let mut bodies = bodies;
                let _ = bodies.insert_toml(capsule_toml()).expect("capsule 合法");
                bodies
            },
        )
        .expect("恢复成功(注册表含 quad 与 capsule)");
        // 续跑 200 拍,与不中断线逐拍快照对比。
        let mut interrupted_snapshots = Vec::new();
        for _ in 0..200usize {
            resumed.apply_activation(&observers);
            resumed.step();
            resumed.run_transient_lifecycle();
            let _ = resumed.take_outbox();
            interrupted_snapshots.push(resumed.snapshot());
        }
        let mut golden_snapshots = Vec::new();
        for _ in 0..200usize {
            golden.apply_activation(&observers);
            golden.step();
            golden.run_transient_lifecycle();
            let _ = golden.take_outbox();
            golden_snapshots.push(golden.snapshot());
        }
        for (i, (a_snap, g_snap)) in interrupted_snapshots
            .iter()
            .zip(golden_snapshots.iter())
            .enumerate()
        {
            assert_eq!(a_snap, g_snap, "续跑第 {i} 拍快照与不中断线逐位相同");
        }
    }

    /// G6:低于阈值不误杀。四足从 3 m 自由落下着地,全程
    /// `agent_runaway_removed == 0`,打印全程连杆速度峰值。
    #[test]
    fn g6_below_threshold_no_false_kill() {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        // 3 m 自由落下:出生在地面顶面上方 3 m(足底 ~2.55 m 间隙)。
        let origin = Vec3::new(24.0, GROUND_TOP_Y + 3.0 + 0.45, 24.0);
        s.spawn_agent(a, hash, quad_spawn_state(origin), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let observers = [Vec3::new(24.0, 3.45, 24.0)];
        let mut runaway_total = 0u64;
        let mut peak = 0.0f32;
        for _ in 0..600usize {
            s.apply_activation(&observers);
            s.step();
            s.run_transient_lifecycle();
            let o = s.take_outbox();
            runaway_total += o.agent_runaway_removed;
            // 全程连杆速度峰值(每拍扫一遍;主体被移除即退出循环)。
            if let Some(record) = s.agents.get(&a) {
                for &h in record.links.values() {
                    if let Some(rb) = s.phys.bodies.get(h) {
                        let v = crate::conv::rapier_to_vec3(rb.linvel()).length();
                        peak = peak.max(v);
                    }
                }
            } else {
                break;
            }
        }
        assert_eq!(runaway_total, 0, "3 m 落体全程不触发防护(0 移除)");
        println!(
            "G6 全程连杆速度峰值 {peak:.1} m/s(阈值 {})",
            crate::agent::AGENT_V_GUARD
        );
        assert!(s.agents.contains_key(&a), "四足全程在场(不被误杀)");
    }

    // ---- B01b:极端接触(AC-B8;任务书第 4 节 X1/X2/X3)----

    /// X 系列单组判据统计。
    #[derive(Debug, Default)]
    struct XStats {
        /// 连杆速度全程峰值。
        peak_link_v: f32,
        /// 机身(根连杆)速度全程峰值。
        peak_body_v: f32,
        /// 任一连杆速度超 max(20, 2v) 的最长连续拍数。
        max_overrun_ticks: usize,
        /// 全程状态是否一直有限(无 NaN/inf)。
        all_finite: bool,
        /// 触发失控移除的次数(0/1)。
        runaway_removed: u64,
        /// 末 60 拍机身速度最大值(末态不再发散的判据)。
        tail_body_v_max: f32,
    }

    /// 末态分类(任务书 X1 打印项):站回 / 倒地 / 蹲低。口径(demo mass
    /// §13 同源):机身 up·y ≥ 0.85 → 站回;≤ 0.35 → 蹲低;其余 → 倒地。
    fn end_state_class(s: &crate::sim::ShardSim, a: types::EntityId) -> Option<&'static str> {
        let record = s.agents.get(&a)?;
        let root = record.links.values().next().copied()?;
        let rb = s.phys.bodies.get(root)?;
        let up = rb.rotation() * crate::conv::vec3_to_rapier(Vec3::Y);
        Some(if up.y >= 0.85 {
            "站回"
        } else if up.y <= 0.35 {
            "蹲低"
        } else {
            "倒地"
        })
    }

    /// X 系列落块描述(生产默认密度 1.0 → 质量 = 8·h³)。
    struct XDrop {
        /// 落点(世界系 x/z;四足在 (24, 24))。
        dx: f32,
        dz: f32,
        /// 起始高度(落点底面离地;X1 = 5/10,X2 = 5)。
        height: f32,
        /// 半边长(单格 0.5 → 质量 1;X2 = 1/2/4 → 质量 8/64/512)。
        half_size: f32,
    }

    /// 单组跑法(X1/X2/X3 共用):四足先站到入睡或 300 拍 → SpawnDynamic
    /// 生产路径(apply_prefix)生成落块(初速零,自由落 h)→ 600 拍
    /// (+ extra_tail_ticks 尾段,每拍全生产节奏)。逐拍统计判据窗口;
    /// 主体被移除后世界继续跑完剩余拍(G4 语义),统计停止。
    fn run_x_group(
        drop: &XDrop,
        jitter: f32,
        guard_on: bool,
        extra_tail_ticks: usize,
    ) -> (crate::sim::ShardSim, types::EntityId, XStats) {
        let (mut s, hash) = quad_sim_with(0.01);
        load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.spawn_agent(a, hash, quad_spawn_state(quad_origin(24.0, 24.0)), None)
            .expect("四足生成");
        assert_quad_above_ground(&s, a);
        let observers = [Vec3::new(24.0, 3.45, 24.0)];
        for _ in 0..300usize {
            s.apply_activation(&observers);
            s.step();
            s.run_transient_lifecycle();
            let _ = s.take_outbox();
            let asleep = s
                .phys
                .bodies
                .get(s.entities.get(&a).expect("在场").handle)
                .expect("根刚体")
                .is_sleeping();
            if asleep {
                break;
            }
        }
        // 生成落块:SpawnDynamic 生产路径(apply_prefix → apply_command →
        // spawn_entity,生产默认密度 1.0 → 质量 = 8·h³)。落块底面从
        // `height + jitter` 落到机身顶面(~0.28 + half_size)。
        let start_y = GROUND_TOP_Y + drop.height + jitter + drop.half_size;
        let id = types::EntityId::from_parts(2, 1);
        let state = types::EntityState {
            translation: Vec3::new(drop.dx, start_y, drop.dz),
            ..types::EntityState::default()
        };
        let desc = types::EntityDesc {
            half_size: drop.half_size,
            ..types::EntityDesc::default()
        };
        s.apply_prefix(&[messaging::StampedInput {
            apply_at: types::Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 0,
            payload: messaging::InputPayload::Command(messaging::SimCommand::SpawnDynamic {
                entity: id,
                state,
                desc,
            }),
        }]);
        if !guard_on {
            s.agent_runaway_guard_enabled = false;
        }
        // 读回质量(生产默认密度校验;任务书 X1/X2"读回质量并打印")。
        let mass = s.entity_mass(id).expect("落块在场");
        let expected = 8.0 * drop.half_size * drop.half_size * drop.half_size;
        assert!(
            (mass - expected).abs() < 1e-3,
            "落块质量读回 {mass},期望密度 1.0 × 体积 = {expected}"
        );
        // 撞击速度 = √(2g·h_eff):底面从 height 落到机身顶面 0.28。
        let impact_v = (2.0f32 * 9.81 * drop.height.max(0.0)).sqrt();
        let cap = (20.0f32).max(2.0 * impact_v);
        let mut st = XStats {
            all_finite: true,
            ..XStats::default()
        };
        let total = 600usize + extra_tail_ticks;
        let mut overrun_run = 0usize;
        for tick in 0..total {
            s.apply_activation(&observers);
            s.step();
            s.run_transient_lifecycle();
            let out = s.take_outbox();
            st.runaway_removed += out.agent_runaway_removed;
            let Some(record) = s.agents.get(&a) else {
                // 主体已移除:世界继续跑完剩余拍(不早退),统计终止。
                for _ in tick + 1..total {
                    s.apply_activation(&observers);
                    s.step();
                    s.run_transient_lifecycle();
                    let _ = s.take_outbox();
                }
                break;
            };
            let mut link_peak = 0.0f32;
            for &h in record.links.values() {
                let rb = s.phys.bodies.get(h).expect("连杆在场");
                let v3 = crate::conv::rapier_to_vec3(rb.linvel());
                let p3 = crate::conv::rapier_to_vec3(rb.translation());
                let r = rb.rotation();
                let av = crate::conv::rapier_to_vec3(rb.angvel());
                if ![
                    v3.x, v3.y, v3.z, av.x, av.y, av.z, p3.x, p3.y, p3.z, r.w, r.x, r.y, r.z,
                ]
                .iter()
                .all(|c| c.is_finite())
                {
                    st.all_finite = false;
                }
                link_peak = link_peak.max(v3.length());
            }
            st.peak_link_v = st.peak_link_v.max(link_peak);
            let body_v = crate::conv::rapier_to_vec3(
                s.phys
                    .bodies
                    .get(s.entities.get(&a).expect("在场").handle)
                    .expect("根刚体")
                    .linvel(),
            )
            .length();
            st.peak_body_v = st.peak_body_v.max(body_v);
            if link_peak > cap {
                overrun_run += 1;
                st.max_overrun_ticks = st.max_overrun_ticks.max(overrun_run);
            } else {
                overrun_run = 0;
            }
            if tick + 60 >= total {
                st.tail_body_v_max = st.tail_body_v_max.max(body_v);
            }
        }
        (s, a, st)
    }

    /// X1 机身 + 四条腿的 5×5 落点网格(demo mass 同口径;覆盖 ±0.5 m,
    /// 步长 0.25 m:机身 ±0.15 与四腿 ±0.35 都在网格内)。
    fn x1_grid() -> Vec<(f32, f32)> {
        let mut g = Vec::new();
        for i in -2i32..=2 {
            for j in -2i32..=2 {
                g.push((0.25 * i as f32, 0.25 * j as f32));
            }
        }
        g
    }

    /// X1 切片 Runner:高度 h × 落点行 row(0..4,即 dx = 0.25·(row−2))
    /// × 微扰档 j,共 5 组。三个(高度 × 行 × 微扰)= 250 组全矩阵由 50 个
    /// #[test] 各跑一片;单片 ≈ 5 × 21 s ≈ 105 s(debug 档,任务书 5 分钟限)。
    fn run_x1_slice(h: f32, row: usize, j: f32) {
        let grid = x1_grid();
        let cap = (20.0f32).max(2.0 * (2.0 * 9.81 * h).sqrt());
        let mut groups = 0usize;
        let mut panic_groups = 0usize;
        let mut peak_link = 0.0f32;
        let mut worst_overrun = 0usize;
        let mut worst_body_v = 0.0f32;
        let mut worst_tail_v = 0.0f32;
        let mut finite_fail = Vec::new();
        let mut overrun_fail = Vec::new();
        let mut body_v_fail = Vec::new();
        let mut tail_fail = Vec::new();
        let mut runaway_fail = Vec::new();
        let mut classes = std::collections::BTreeMap::<&'static str, usize>::new();
        // 该行 5 个落点(row 0..4 → i = row−2;j 全扫)。
        for &(dx, dz) in grid.iter().skip(row * 5).take(5) {
            if (dx.abs() - 0.5 * (row as f32 - 2.0).abs()).abs() > 1e-6 {
                // 只取该行(x 偏移 = 0.25·(row−2));x1_grid 是 5×5 按行排。
            }
            for j_this in [j] {
                let drop = XDrop {
                    dx: 24.0 + dx,
                    dz: 24.0 + dz,
                    height: h,
                    half_size: 0.5,
                };
                let tag = format!("d({dx:+.2},{dz:+.2}) j{j_this:.3}");
                groups += 1;
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    run_x_group(&drop, j_this, true, 0)
                }));
                let Ok((s, a, st)) = result else {
                    panic_groups += 1;
                    continue;
                };
                peak_link = peak_link.max(st.peak_link_v);
                worst_overrun = worst_overrun.max(st.max_overrun_ticks);
                worst_body_v = worst_body_v.max(st.peak_body_v);
                worst_tail_v = worst_tail_v.max(st.tail_body_v_max);
                if !st.all_finite {
                    finite_fail.push(tag.clone());
                }
                if st.max_overrun_ticks > 5 {
                    overrun_fail.push(tag.clone());
                }
                if st.peak_body_v > cap {
                    body_v_fail.push(tag.clone());
                }
                if st.tail_body_v_max >= 1.0 {
                    tail_fail.push(tag.clone());
                }
                if st.runaway_removed != 0 {
                    runaway_fail.push(tag.clone());
                }
                if let Some(c) = end_state_class(&s, a) {
                    *classes.entry(c).or_default() += 1;
                }
            }
        }
        println!(
            "X1 h={h} r{row} j{j:+.3}: 组数 {groups} panic {panic_groups};连杆峰值 {peak_link:.1};             最长超限 {worst_overrun};机身峰值 {worst_body_v:.1};末60拍 {worst_tail_v:.3};末态 {classes:?}"
        );
        assert_eq!(panic_groups, 0, "X1 h={h} r{row} j{j:.3} 无 panic");
        assert!(
            finite_fail.is_empty(),
            "X1 h={h} r{row} j{j:.3} 非有限组 {finite_fail:?}"
        );
        assert!(
            overrun_fail.is_empty(),
            "X1 h={h} r{row} j{j:.3} 超限>5 组 {overrun_fail:?}"
        );
        assert!(
            body_v_fail.is_empty(),
            "X1 h={h} r{row} j{j:.3} 机身>{cap:.0} 组 {body_v_fail:?}"
        );
        assert!(
            tail_fail.is_empty(),
            "X1 h={h} r{row} j{j:.3} 末态发散组 {tail_fail:?}"
        );
        assert!(
            runaway_fail.is_empty(),
            "X1 h={h} r{row} j{j:.3} 误移除组 {runaway_fail:?}"
        );
    }

    /// X2 切片 Runner:质量档(half, mass)× 落点行 row(0..2,dx = 0.5·(row−1))
    /// × 微扰档 j,共 5 组(每组 600 + 300 拍)。全矩阵 3 质量 × 3 行 × 5 微扰
    /// = 45 组/质量,由 15 个 #[test] 各跑一片(debug 档限时)。
    fn run_x2_slice(half: f32, mass: f32, row: usize, j: f32) {
        let mut grid = Vec::new();
        for i in -1i32..=1 {
            for jz in -1i32..=1 {
                grid.push((0.5 * i as f32, 0.5 * jz as f32));
            }
        }
        let mut groups = 0usize;
        let mut panic_groups = 0usize;
        let mut removed_groups = 0usize;
        let mut peak_link_alive = 0.0f32;
        let mut nonfinite_alive = Vec::new();
        for &(dx, dz) in grid.iter().skip(row * 3).take(3) {
            let drop = XDrop {
                dx: 24.0 + dx,
                dz: 24.0 + dz,
                height: 5.0,
                half_size: half,
            };
            let tag = format!("d({dx:+.2},{dz:+.2}) j{j:.3}");
            groups += 1;
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                run_x_group(&drop, j, true, 300)
            }));
            let Ok((_s, _a, st)) = result else {
                panic_groups += 1;
                continue;
            };
            if st.runaway_removed > 0 {
                removed_groups += 1;
            }
            if st.peak_link_v.is_finite() {
                peak_link_alive = peak_link_alive.max(st.peak_link_v);
            }
            if !st.all_finite && st.runaway_removed == 0 {
                nonfinite_alive.push(tag);
            }
        }
        println!(
            "X2 m{mass} r{row} j{j:+.3}: 组数 {groups} panic {panic_groups} 移除 {removed_groups};             未移除组连杆峰值 {peak_link_alive:.1}"
        );
        assert_eq!(panic_groups, 0, "X2 m{mass} r{row} j{j:.3} 无 panic");
        assert!(
            nonfinite_alive.is_empty(),
            "X2 m{mass} r{row} j{j:.3} 未移除但非有限 {nonfinite_alive:?}"
        );
        if mass == 8.0 {
            assert_eq!(
                removed_groups, 0,
                "X2 m8 r{row} j{j:.3} 误移除组存在(demo §13 峰值 126 < 200)"
            );
        }
    }

    /// X3 切片 Runner:质量档 half × 微扰档 j,跑全部 9 落点(防护关闭 +
    /// catch_unwind),共 9 组。只报数:panic 组数、未 panic 组连杆速度峰值。
    fn run_x3_slice(half: f32, j: f32) -> (usize, usize, f32) {
        let mut grid = Vec::new();
        for i in -1i32..=1 {
            for jz in -1i32..=1 {
                grid.push((0.5 * i as f32, 0.5 * jz as f32));
            }
        }
        let mut total = 0usize;
        let mut panics = 0usize;
        let mut max_peak = 0.0f32;
        for &(dx, dz) in &grid {
            let drop = XDrop {
                dx: 24.0 + dx,
                dz: 24.0 + dz,
                height: 5.0,
                half_size: half,
            };
            total += 1;
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                run_x_group(&drop, j, false, 0)
            }));
            match result {
                Err(_) => panics += 1,
                Ok((_s, _a, st)) => {
                    if st.peak_link_v.is_finite() {
                        max_peak = max_peak.max(st.peak_link_v);
                    }
                }
            }
        }
        (total, panics, max_peak)
    }

    // ---- 参数化测试阵列(代码生成;每片 debug 档 ≤ 5 分钟)----

    /// X1 h=5.0 落点行 0(dx = -0.50)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h5m_r0_j0() {
        run_x1_slice(5.0, 0, 0.0);
    }

    /// X1 h=5.0 落点行 0(dx = -0.50)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h5m_r0_j1() {
        run_x1_slice(5.0, 0, 0.017);
    }

    /// X1 h=5.0 落点行 0(dx = -0.50)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h5m_r0_j2() {
        run_x1_slice(5.0, 0, 0.034);
    }

    /// X1 h=5.0 落点行 0(dx = -0.50)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h5m_r0_j3() {
        run_x1_slice(5.0, 0, 0.051);
    }

    /// X1 h=5.0 落点行 0(dx = -0.50)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h5m_r0_j4() {
        run_x1_slice(5.0, 0, 0.068);
    }

    /// X1 h=5.0 落点行 1(dx = -0.25)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h5m_r1_j0() {
        run_x1_slice(5.0, 1, 0.0);
    }

    /// X1 h=5.0 落点行 1(dx = -0.25)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h5m_r1_j1() {
        run_x1_slice(5.0, 1, 0.017);
    }

    /// X1 h=5.0 落点行 1(dx = -0.25)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h5m_r1_j2() {
        run_x1_slice(5.0, 1, 0.034);
    }

    /// X1 h=5.0 落点行 1(dx = -0.25)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h5m_r1_j3() {
        run_x1_slice(5.0, 1, 0.051);
    }

    /// X1 h=5.0 落点行 1(dx = -0.25)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h5m_r1_j4() {
        run_x1_slice(5.0, 1, 0.068);
    }

    /// X1 h=5.0 落点行 2(dx = +0.00)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h5m_r2_j0() {
        run_x1_slice(5.0, 2, 0.0);
    }

    /// X1 h=5.0 落点行 2(dx = +0.00)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h5m_r2_j1() {
        run_x1_slice(5.0, 2, 0.017);
    }

    /// X1 h=5.0 落点行 2(dx = +0.00)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h5m_r2_j2() {
        run_x1_slice(5.0, 2, 0.034);
    }

    /// X1 h=5.0 落点行 2(dx = +0.00)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h5m_r2_j3() {
        run_x1_slice(5.0, 2, 0.051);
    }

    /// X1 h=5.0 落点行 2(dx = +0.00)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h5m_r2_j4() {
        run_x1_slice(5.0, 2, 0.068);
    }

    /// X1 h=5.0 落点行 3(dx = +0.25)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h5m_r3_j0() {
        run_x1_slice(5.0, 3, 0.0);
    }

    /// X1 h=5.0 落点行 3(dx = +0.25)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h5m_r3_j1() {
        run_x1_slice(5.0, 3, 0.017);
    }

    /// X1 h=5.0 落点行 3(dx = +0.25)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h5m_r3_j2() {
        run_x1_slice(5.0, 3, 0.034);
    }

    /// X1 h=5.0 落点行 3(dx = +0.25)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h5m_r3_j3() {
        run_x1_slice(5.0, 3, 0.051);
    }

    /// X1 h=5.0 落点行 3(dx = +0.25)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h5m_r3_j4() {
        run_x1_slice(5.0, 3, 0.068);
    }

    /// X1 h=5.0 落点行 4(dx = +0.50)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h5m_r4_j0() {
        run_x1_slice(5.0, 4, 0.0);
    }

    /// X1 h=5.0 落点行 4(dx = +0.50)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h5m_r4_j1() {
        run_x1_slice(5.0, 4, 0.017);
    }

    /// X1 h=5.0 落点行 4(dx = +0.50)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h5m_r4_j2() {
        run_x1_slice(5.0, 4, 0.034);
    }

    /// X1 h=5.0 落点行 4(dx = +0.50)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h5m_r4_j3() {
        run_x1_slice(5.0, 4, 0.051);
    }

    /// X1 h=5.0 落点行 4(dx = +0.50)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h5m_r4_j4() {
        run_x1_slice(5.0, 4, 0.068);
    }

    /// X1 h=10.0 落点行 0(dx = -0.50)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h10m_r0_j0() {
        run_x1_slice(10.0, 0, 0.0);
    }

    /// X1 h=10.0 落点行 0(dx = -0.50)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h10m_r0_j1() {
        run_x1_slice(10.0, 0, 0.017);
    }

    /// X1 h=10.0 落点行 0(dx = -0.50)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h10m_r0_j2() {
        run_x1_slice(10.0, 0, 0.034);
    }

    /// X1 h=10.0 落点行 0(dx = -0.50)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h10m_r0_j3() {
        run_x1_slice(10.0, 0, 0.051);
    }

    /// X1 h=10.0 落点行 0(dx = -0.50)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h10m_r0_j4() {
        run_x1_slice(10.0, 0, 0.068);
    }

    /// X1 h=10.0 落点行 1(dx = -0.25)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h10m_r1_j0() {
        run_x1_slice(10.0, 1, 0.0);
    }

    /// X1 h=10.0 落点行 1(dx = -0.25)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h10m_r1_j1() {
        run_x1_slice(10.0, 1, 0.017);
    }

    /// X1 h=10.0 落点行 1(dx = -0.25)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h10m_r1_j2() {
        run_x1_slice(10.0, 1, 0.034);
    }

    /// X1 h=10.0 落点行 1(dx = -0.25)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h10m_r1_j3() {
        run_x1_slice(10.0, 1, 0.051);
    }

    /// X1 h=10.0 落点行 1(dx = -0.25)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h10m_r1_j4() {
        run_x1_slice(10.0, 1, 0.068);
    }

    /// X1 h=10.0 落点行 2(dx = +0.00)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h10m_r2_j0() {
        run_x1_slice(10.0, 2, 0.0);
    }

    /// X1 h=10.0 落点行 2(dx = +0.00)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h10m_r2_j1() {
        run_x1_slice(10.0, 2, 0.017);
    }

    /// X1 h=10.0 落点行 2(dx = +0.00)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h10m_r2_j2() {
        run_x1_slice(10.0, 2, 0.034);
    }

    /// X1 h=10.0 落点行 2(dx = +0.00)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h10m_r2_j3() {
        run_x1_slice(10.0, 2, 0.051);
    }

    /// X1 h=10.0 落点行 2(dx = +0.00)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h10m_r2_j4() {
        run_x1_slice(10.0, 2, 0.068);
    }

    /// X1 h=10.0 落点行 3(dx = +0.25)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h10m_r3_j0() {
        run_x1_slice(10.0, 3, 0.0);
    }

    /// X1 h=10.0 落点行 3(dx = +0.25)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h10m_r3_j1() {
        run_x1_slice(10.0, 3, 0.017);
    }

    /// X1 h=10.0 落点行 3(dx = +0.25)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h10m_r3_j2() {
        run_x1_slice(10.0, 3, 0.034);
    }

    /// X1 h=10.0 落点行 3(dx = +0.25)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h10m_r3_j3() {
        run_x1_slice(10.0, 3, 0.051);
    }

    /// X1 h=10.0 落点行 3(dx = +0.25)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h10m_r3_j4() {
        run_x1_slice(10.0, 3, 0.068);
    }

    /// X1 h=10.0 落点行 4(dx = +0.50)× 微扰 0.0 m:5 组。
    #[test]
    fn x1_h10m_r4_j0() {
        run_x1_slice(10.0, 4, 0.0);
    }

    /// X1 h=10.0 落点行 4(dx = +0.50)× 微扰 0.017 m:5 组。
    #[test]
    fn x1_h10m_r4_j1() {
        run_x1_slice(10.0, 4, 0.017);
    }

    /// X1 h=10.0 落点行 4(dx = +0.50)× 微扰 0.034 m:5 组。
    #[test]
    fn x1_h10m_r4_j2() {
        run_x1_slice(10.0, 4, 0.034);
    }

    /// X1 h=10.0 落点行 4(dx = +0.50)× 微扰 0.051 m:5 组。
    #[test]
    fn x1_h10m_r4_j3() {
        run_x1_slice(10.0, 4, 0.051);
    }

    /// X1 h=10.0 落点行 4(dx = +0.50)× 微扰 0.068 m:5 组。
    #[test]
    fn x1_h10m_r4_j4() {
        run_x1_slice(10.0, 4, 0.068);
    }

    /// X2 质量 8.0 落点行 0 × 微扰 0.0 m:5 组。
    #[test]
    fn x2_m8_r0_j0() {
        run_x2_slice(1.0, 8.0, 0, 0.0);
    }

    /// X2 质量 8.0 落点行 0 × 微扰 0.017 m:5 组。
    #[test]
    fn x2_m8_r0_j1() {
        run_x2_slice(1.0, 8.0, 0, 0.017);
    }

    /// X2 质量 8.0 落点行 0 × 微扰 0.034 m:5 组。
    #[test]
    fn x2_m8_r0_j2() {
        run_x2_slice(1.0, 8.0, 0, 0.034);
    }

    /// X2 质量 8.0 落点行 0 × 微扰 0.051 m:5 组。
    #[test]
    fn x2_m8_r0_j3() {
        run_x2_slice(1.0, 8.0, 0, 0.051);
    }

    /// X2 质量 8.0 落点行 0 × 微扰 0.068 m:5 组。
    #[test]
    fn x2_m8_r0_j4() {
        run_x2_slice(1.0, 8.0, 0, 0.068);
    }

    /// X2 质量 8.0 落点行 1 × 微扰 0.0 m:5 组。
    #[test]
    fn x2_m8_r1_j0() {
        run_x2_slice(1.0, 8.0, 1, 0.0);
    }

    /// X2 质量 8.0 落点行 1 × 微扰 0.017 m:5 组。
    #[test]
    fn x2_m8_r1_j1() {
        run_x2_slice(1.0, 8.0, 1, 0.017);
    }

    /// X2 质量 8.0 落点行 1 × 微扰 0.034 m:5 组。
    #[test]
    fn x2_m8_r1_j2() {
        run_x2_slice(1.0, 8.0, 1, 0.034);
    }

    /// X2 质量 8.0 落点行 1 × 微扰 0.051 m:5 组。
    #[test]
    fn x2_m8_r1_j3() {
        run_x2_slice(1.0, 8.0, 1, 0.051);
    }

    /// X2 质量 8.0 落点行 1 × 微扰 0.068 m:5 组。
    #[test]
    fn x2_m8_r1_j4() {
        run_x2_slice(1.0, 8.0, 1, 0.068);
    }

    /// X2 质量 8.0 落点行 2 × 微扰 0.0 m:5 组。
    #[test]
    fn x2_m8_r2_j0() {
        run_x2_slice(1.0, 8.0, 2, 0.0);
    }

    /// X2 质量 8.0 落点行 2 × 微扰 0.017 m:5 组。
    #[test]
    fn x2_m8_r2_j1() {
        run_x2_slice(1.0, 8.0, 2, 0.017);
    }

    /// X2 质量 8.0 落点行 2 × 微扰 0.034 m:5 组。
    #[test]
    fn x2_m8_r2_j2() {
        run_x2_slice(1.0, 8.0, 2, 0.034);
    }

    /// X2 质量 8.0 落点行 2 × 微扰 0.051 m:5 组。
    #[test]
    fn x2_m8_r2_j3() {
        run_x2_slice(1.0, 8.0, 2, 0.051);
    }

    /// X2 质量 8.0 落点行 2 × 微扰 0.068 m:5 组。
    #[test]
    fn x2_m8_r2_j4() {
        run_x2_slice(1.0, 8.0, 2, 0.068);
    }

    /// X2 质量 64.0 落点行 0 × 微扰 0.0 m:5 组。
    #[test]
    fn x2_m64_r0_j0() {
        run_x2_slice(2.0, 64.0, 0, 0.0);
    }

    /// X2 质量 64.0 落点行 0 × 微扰 0.017 m:5 组。
    #[test]
    fn x2_m64_r0_j1() {
        run_x2_slice(2.0, 64.0, 0, 0.017);
    }

    /// X2 质量 64.0 落点行 0 × 微扰 0.034 m:5 组。
    #[test]
    fn x2_m64_r0_j2() {
        run_x2_slice(2.0, 64.0, 0, 0.034);
    }

    /// X2 质量 64.0 落点行 0 × 微扰 0.051 m:5 组。
    #[test]
    fn x2_m64_r0_j3() {
        run_x2_slice(2.0, 64.0, 0, 0.051);
    }

    /// X2 质量 64.0 落点行 0 × 微扰 0.068 m:5 组。
    #[test]
    fn x2_m64_r0_j4() {
        run_x2_slice(2.0, 64.0, 0, 0.068);
    }

    /// X2 质量 64.0 落点行 1 × 微扰 0.0 m:5 组。
    #[test]
    fn x2_m64_r1_j0() {
        run_x2_slice(2.0, 64.0, 1, 0.0);
    }

    /// X2 质量 64.0 落点行 1 × 微扰 0.017 m:5 组。
    #[test]
    fn x2_m64_r1_j1() {
        run_x2_slice(2.0, 64.0, 1, 0.017);
    }

    /// X2 质量 64.0 落点行 1 × 微扰 0.034 m:5 组。
    #[test]
    fn x2_m64_r1_j2() {
        run_x2_slice(2.0, 64.0, 1, 0.034);
    }

    /// X2 质量 64.0 落点行 1 × 微扰 0.051 m:5 组。
    #[test]
    fn x2_m64_r1_j3() {
        run_x2_slice(2.0, 64.0, 1, 0.051);
    }

    /// X2 质量 64.0 落点行 1 × 微扰 0.068 m:5 组。
    #[test]
    fn x2_m64_r1_j4() {
        run_x2_slice(2.0, 64.0, 1, 0.068);
    }

    /// X2 质量 64.0 落点行 2 × 微扰 0.0 m:5 组。
    #[test]
    fn x2_m64_r2_j0() {
        run_x2_slice(2.0, 64.0, 2, 0.0);
    }

    /// X2 质量 64.0 落点行 2 × 微扰 0.017 m:5 组。
    #[test]
    fn x2_m64_r2_j1() {
        run_x2_slice(2.0, 64.0, 2, 0.017);
    }

    /// X2 质量 64.0 落点行 2 × 微扰 0.034 m:5 组。
    #[test]
    fn x2_m64_r2_j2() {
        run_x2_slice(2.0, 64.0, 2, 0.034);
    }

    /// X2 质量 64.0 落点行 2 × 微扰 0.051 m:5 组。
    #[test]
    fn x2_m64_r2_j3() {
        run_x2_slice(2.0, 64.0, 2, 0.051);
    }

    /// X2 质量 64.0 落点行 2 × 微扰 0.068 m:5 组。
    #[test]
    fn x2_m64_r2_j4() {
        run_x2_slice(2.0, 64.0, 2, 0.068);
    }

    /// X2 质量 512.0 落点行 0 × 微扰 0.0 m:5 组。
    #[test]
    fn x2_m512_r0_j0() {
        run_x2_slice(4.0, 512.0, 0, 0.0);
    }

    /// X2 质量 512.0 落点行 0 × 微扰 0.017 m:5 组。
    #[test]
    fn x2_m512_r0_j1() {
        run_x2_slice(4.0, 512.0, 0, 0.017);
    }

    /// X2 质量 512.0 落点行 0 × 微扰 0.034 m:5 组。
    #[test]
    fn x2_m512_r0_j2() {
        run_x2_slice(4.0, 512.0, 0, 0.034);
    }

    /// X2 质量 512.0 落点行 0 × 微扰 0.051 m:5 组。
    #[test]
    fn x2_m512_r0_j3() {
        run_x2_slice(4.0, 512.0, 0, 0.051);
    }

    /// X2 质量 512.0 落点行 0 × 微扰 0.068 m:5 组。
    #[test]
    fn x2_m512_r0_j4() {
        run_x2_slice(4.0, 512.0, 0, 0.068);
    }

    /// X2 质量 512.0 落点行 1 × 微扰 0.0 m:5 组。
    #[test]
    fn x2_m512_r1_j0() {
        run_x2_slice(4.0, 512.0, 1, 0.0);
    }

    /// X2 质量 512.0 落点行 1 × 微扰 0.017 m:5 组。
    #[test]
    fn x2_m512_r1_j1() {
        run_x2_slice(4.0, 512.0, 1, 0.017);
    }

    /// X2 质量 512.0 落点行 1 × 微扰 0.034 m:5 组。
    #[test]
    fn x2_m512_r1_j2() {
        run_x2_slice(4.0, 512.0, 1, 0.034);
    }

    /// X2 质量 512.0 落点行 1 × 微扰 0.051 m:5 组。
    #[test]
    fn x2_m512_r1_j3() {
        run_x2_slice(4.0, 512.0, 1, 0.051);
    }

    /// X2 质量 512.0 落点行 1 × 微扰 0.068 m:5 组。
    #[test]
    fn x2_m512_r1_j4() {
        run_x2_slice(4.0, 512.0, 1, 0.068);
    }

    /// X2 质量 512.0 落点行 2 × 微扰 0.0 m:5 组。
    #[test]
    fn x2_m512_r2_j0() {
        run_x2_slice(4.0, 512.0, 2, 0.0);
    }

    /// X2 质量 512.0 落点行 2 × 微扰 0.017 m:5 组。
    #[test]
    fn x2_m512_r2_j1() {
        run_x2_slice(4.0, 512.0, 2, 0.017);
    }

    /// X2 质量 512.0 落点行 2 × 微扰 0.034 m:5 组。
    #[test]
    fn x2_m512_r2_j2() {
        run_x2_slice(4.0, 512.0, 2, 0.034);
    }

    /// X2 质量 512.0 落点行 2 × 微扰 0.051 m:5 组。
    #[test]
    fn x2_m512_r2_j3() {
        run_x2_slice(4.0, 512.0, 2, 0.051);
    }

    /// X2 质量 512.0 落点行 2 × 微扰 0.068 m:5 组。
    #[test]
    fn x2_m512_r2_j4() {
        run_x2_slice(4.0, 512.0, 2, 0.068);
    }

    /// X3 质量 64(防护关)× 微扰 0.0 m:9 落点组,只报数。
    #[test]
    fn x3_m64_j0() {
        let (total, panics, peak) = run_x3_slice(2.0, 0.0);
        println!("X3 m64 j+0.0: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }

    /// X3 质量 64(防护关)× 微扰 0.017 m:9 落点组,只报数。
    #[test]
    fn x3_m64_j1() {
        let (total, panics, peak) = run_x3_slice(2.0, 0.017);
        println!("X3 m64 j+0.017: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }

    /// X3 质量 64(防护关)× 微扰 0.034 m:9 落点组,只报数。
    #[test]
    fn x3_m64_j2() {
        let (total, panics, peak) = run_x3_slice(2.0, 0.034);
        println!("X3 m64 j+0.034: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }

    /// X3 质量 64(防护关)× 微扰 0.051 m:9 落点组,只报数。
    #[test]
    fn x3_m64_j3() {
        let (total, panics, peak) = run_x3_slice(2.0, 0.051);
        println!("X3 m64 j+0.051: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }

    /// X3 质量 64(防护关)× 微扰 0.068 m:9 落点组,只报数。
    #[test]
    fn x3_m64_j4() {
        let (total, panics, peak) = run_x3_slice(2.0, 0.068);
        println!("X3 m64 j+0.068: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }

    /// X3 质量 512(防护关)× 微扰 0.0 m:9 落点组,只报数。
    #[test]
    fn x3_m512_j0() {
        let (total, panics, peak) = run_x3_slice(4.0, 0.0);
        println!("X3 m512 j+0.0: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }

    /// X3 质量 512(防护关)× 微扰 0.017 m:9 落点组,只报数。
    #[test]
    fn x3_m512_j1() {
        let (total, panics, peak) = run_x3_slice(4.0, 0.017);
        println!("X3 m512 j+0.017: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }

    /// X3 质量 512(防护关)× 微扰 0.034 m:9 落点组,只报数。
    #[test]
    fn x3_m512_j2() {
        let (total, panics, peak) = run_x3_slice(4.0, 0.034);
        println!("X3 m512 j+0.034: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }

    /// X3 质量 512(防护关)× 微扰 0.051 m:9 落点组,只报数。
    #[test]
    fn x3_m512_j3() {
        let (total, panics, peak) = run_x3_slice(4.0, 0.051);
        println!("X3 m512 j+0.051: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }

    /// X3 质量 512(防护关)× 微扰 0.068 m:9 落点组,只报数。
    #[test]
    fn x3_m512_j4() {
        let (total, panics, peak) = run_x3_slice(4.0, 0.068);
        println!("X3 m512 j+0.068: 组数 {total} panic {panics};未 panic 组连杆峰值 {peak:.3e} m/s");
    }
}
