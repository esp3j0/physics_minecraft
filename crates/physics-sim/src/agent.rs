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
    /// 盒 = 半对角 √3、球/胶囊 = 半径(+半高)、凸包 = 最大顶点模长;多形状
    /// /多连杆取最大。空注册表 → 0.0(无身体 = 无附加)。
    pub fn max_envelope_radius(&self) -> f32 {
        self.by_hash
            .values()
            .map(|def| {
                def.links
                    .iter()
                    .flat_map(|l| l.shapes.iter())
                    .map(|s| match s {
                        types::ShapeDef::Box { half } => {
                            (half[0] * half[0] + half[1] * half[1] + half[2] * half[2]).sqrt()
                        }
                        types::ShapeDef::Sphere { radius } => radius.abs(),
                        types::ShapeDef::CapsuleY {
                            half_height,
                            radius,
                        } => half_height.abs() + radius.abs(),
                        types::ShapeDef::ConvexHull { points } => points
                            .iter()
                            .map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt())
                            .fold(0.0f32, f32::max),
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

impl ShardSim {
    /// 主体是否存在。
    pub fn is_agent(&self, id: EntityId) -> bool {
        self.agents.contains_key(&id)
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

    /// 主体数(测试观测口)。
    pub fn agent_count(&self) -> usize {
        self.agents.len()
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

    /// 生成主体(REQ-012 §3;A01 第 3 节):
    /// - 未知哈希 → `Err(UnknownBody)` + 遥测(`agent_spawn_rejected`);
    /// - 多连杆 → `Err(MultiLink)` + 遥测(B01 实现);
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
            .restitution(link.restitution)
            // A02a 第 4 节:接触钩子只挂在主体碰撞体上(摩擦清零 + 接缝
            // 修正;钩子本身见 walker.rs——世界里没有主体时无碰撞体挂钩,
            // 钩子不触发,AC-A8)。
            .active_hooks(rapier3d::prelude::ActiveHooks::MODIFY_SOLVER_CONTACTS);
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
            },
        );
        // 调度器据此 claim 全局 ownership(REQ-006 任务 00;恰好一次)。
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
    /// 主体不存在 → panic(fail-stop;调用方先判 `is_agent`)。
    pub fn despawn_agent(&mut self, id: EntityId) {
        let record = self.agents.remove(&id).expect("主体存在(fail-stop)");
        debug_assert!(
            record.links.len() == 1,
            "A01 只支持单连杆主体(多连杆在 B01)"
        );
        let rec = self.entities.remove(&id).expect("实体记录存在(fail-stop)");
        self.phys.remove_body(rec.handle);
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
                None,
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
}
