//! `ShardSim`:单分片确定性物理核(rapier 封装,REQ-001 §4.1/§4.3)。
//!
//! 单个分片 = 一个 rapier `PhysicsWorld` + 自己的体素 chunk(`VoxelWorld`)+ 实体表。
//! 一切状态变更只经 `apply_prefix`(盖章输入);`step` 推进一拍后在 tick 末做
//! 凝固桥检测与迁移检测(demo A/C 协议);产出经 `take_outbox` 交给调度器。
//! rapier 类型不出本模块的公开接口(防腐层,蓝图 §0)。

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{
    BlockId, BlockPos, CHUNK_SIZE, ChunkPos, EntityDesc, EntityId, EntityState, SHARD_CHUNKS,
    ShardId, Tick,
};
use voxel::{BlockRegistry, Chunk, VoxelEdit, VoxelWorld};

use crate::activation::{ActivationConfig, ActivationDecisions};
use crate::collider_build::{BoxCollider, NeighborChunks, greedy_colliders};
use crate::conv;
use crate::solidify;
use messaging::{
    CrossEvent, EnvelopePayload, InputPayload, InputSource, Intent, SimCommand, StampedInput,
};

/// 凝固桥触发参数(P2 + 睡眠兜底,REQ-001 §9.5 / demo C)。可序列化(快照)。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolidifyConfig {
    /// 总开关。几何测试等场景可关闭;生产恒开(共识 6)。
    pub enabled: bool,
    /// 线速度阈值(m/s;REQ-002 决议 4c 单位澄清:0.1 m/s ≈ 每 tick 1.67mm)。
    pub linear_threshold: f32,
    /// 角速度阈值(rad/s;决议 4c 单位澄清)。
    pub angular_threshold: f32,
    /// 连续低于阈值的 tick 数。
    pub consecutive_ticks: u32,
    /// snap 位移超过此值(格)→ 升级为等睡眠。
    pub max_snap_blocks: f32,
    /// snap 角度误差超过此值 → 升级为等睡眠。
    pub max_snap_angle_deg: f32,
    /// 自动触发位移门(REQ-001 §9.5 修订,REQ-002 决议 4a/demos/solidify-slopes):
    /// 最近 20 tick 位移 ≤ 此值(m)才允许凝固;0 = 关闭门(直通,兼容开关)。
    /// 默认 0.003(3mm;实验:合法落定 d20 ≤1.4mm,受驱爬行 ≥6.3mm)。
    /// 同时约束 P2 计数路径与睡眠兜底路径(决议:d)兜底不得视为误凝固保险)。
    pub displacement_gate_m: f32,
}

impl Default for SolidifyConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            linear_threshold: 0.1,
            angular_threshold: 0.05,
            consecutive_ticks: 20,
            max_snap_blocks: 0.5,
            max_snap_angle_deg: 45.0,
            displacement_gate_m: 0.003,
        }
    }
}

/// 实体票状态操作(§4.3.1;**不含 Issue**——S5:签发单点在调度器 claim/补签,
/// 分片只产出状态依赖操作)。只追加;幂等由 TicketTable 保证。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TicketOp {
    End,
    Pause,
    Unpause,
    Suspend,
    Resume,
}

/// 路由载体:(实体, 操作)——调度器按持有者分片盖章投递。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TicketOpRecord {
    pub entity: EntityId,
    pub op: TicketOp,
}

/// 实体运动状态(REQ-007 §4.6;任务 03):醒着 或 待地形(保存速度)。
/// sim 内部记录 + 快照;不进 `EntityState`(任务书授权:types 不动)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum LocomotionState {
    Awake,
    AwaitingTerrain {
        saved_linvel: Vec3,
        saved_angvel: Vec3,
    },
}

/// 位移门滚动窗口的样本数:最近 20 tick 位移 = 21 个 translation
/// (每 tick step 后采一个点,首尾相距 20 tick;任务书 01 接口设计)。
pub(crate) const DISPLACEMENT_WINDOW_LEN: usize = 21;

/// 单分片配置。可序列化(快照,任务书 01 细则 2:gravity 为 glam Vec3,
/// types 的 glam 已开 serde)。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShardSimConfig {
    pub shard: ShardId,
    pub gravity: Vec3,
    pub dt: f32,
    pub solidify: SolidifyConfig,
    /// 激活参数(§4.3;Default 默认,任务书 02)。
    pub activation: crate::activation::ActivationConfig,
}

impl ShardSimConfig {
    pub fn new(shard: ShardId) -> Self {
        Self {
            shard,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            dt: 1.0 / 60.0,
            solidify: SolidifyConfig::default(),
            activation: crate::activation::ActivationConfig::default(),
        }
    }
}

/// 方块写入来源(REQ-005 D11):外部命令(同拍广播,本任务)或涌现
/// (凝固桥/未来破坏,下一拍信封,任务 03 接入)。追加枚举:声明序即序号。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BlockWriteKind {
    /// 外部 SetBlock(同拍广播到 owner + holders)。
    External,
    /// 涌现写入(凝固桥等;任务 03 起产出版本)。
    Emergent,
}

/// 一次方块写入(REQ-005 D11:网络/持久化统一从 `Outbox.block_updates` 取,
/// 不再靠驱动镜像外部命令)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BlockWrite {
    pub pos: BlockPos,
    pub block: BlockId,
    pub kind: BlockWriteKind,
}

/// 本 tick 产出(调度器消费):迁移意图、collider 重建过的 chunk、凝固记录。
/// 可序列化 + 可克隆(快照携带未消费的产出,任务书 01 细则 1)。
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct Outbox {
    /// 迁移意图(调度器盖章成信封,REQ-001 §4.2 (b))。
    pub migrations: Vec<Intent>,
    /// 本 tick collider 重建过的 chunk(可见性/换装统计用)。
    pub dirty_chunks: BTreeSet<ChunkPos>,
    /// 凝固完成记录:(实体, 写入的格)。**任务 03 起恒空**(兼容字段,serde
    /// 布局不变):凝固写入已迁 `block_updates` 的 `Emergent`,网络/广播面统一
    /// 走该通道;`solidified` 仅保留字段形状供既有快照格式/兼容测试。
    pub solidified: Vec<(EntityId, BlockPos)>,
    /// 本拍物理/命令产生的方块写入(REQ-005 D11):`External` = 外部 SetBlock,
    /// `Emergent` = 凝固桥(任务 03 迁入)与未来的破坏语义。快照格式随本字段
    /// 变更:bincode 为非自描述定长格式,旧检查点(三字段 Outbox)用新二进制
    /// 解码必报 UnexpectedEnd(bootstrap 显式 fail-stop,不静默错);
    /// `#[serde(default)]` 在此格式下对缺失尾字段不生效,不可用它声称旧快照
    /// 兼容。格式版本化由任务 02 的 format_version 处理。
    pub block_updates: Vec<BlockWrite>,
    /// 涌现事件生产口(REQ-005 任务 03):分片内部语义(撞击触发等)未来把
    /// `CrossEvent` 推入此处,调度器 tick 末经 `broadcast_event` 广播(目标
    /// 分片 apply_at = N+1 本地求值)。本任务模拟核不主动产事件,字段恒空,
    /// 测试可直接调 `Scheduler::broadcast_event`。
    pub events: Vec<CrossEvent>,
    /// 本拍新出现的实体(REQ-006 任务 00):调度器据此 claim 全局 ownership。
    /// 迁移目的重建 / Teleport 重建 / 普通 spawn 都记录;恰好一次。
    pub spawned: Vec<EntityId>,
    /// 本拍删除的实体(REQ-006 任务 00):调度器据此 release ownership。
    /// 迁移源删除、RemoveEntity、凝固都记录;与 `spawned` 同拍时先 release。
    pub despawned: Vec<EntityId>,
    /// 实体票状态操作(REQ-007 任务 03;S5:分片只产出状态依赖操作,签发
    /// 单点在调度器 claim/补签)。调度器按 TicketKey 序应用,N 拍产生 N+1 生效
    /// (§14-6);幂等由 TicketTable 保证。
    pub ticket_ops: Vec<crate::sim::TicketOpRecord>,
    /// 强制落地完成(REQ-007 §4.8;任务 03):(实体, 落点格)——调用方已写块
    /// (Emergent BlockWrite)+ despawned;调度器据此 End 票(幂等)。
    pub forced_landings: Vec<(EntityId, BlockPos)>,
    /// 虚空移除(§4.8):y < WORLD_Y_MIN − 深度 的实体(无 BlockWrite)。
    pub void_removed: Vec<EntityId>,
}

/// 实体的分片侧记录(crate 内可见:snapshot.rs 的快照结构复用)。
pub(crate) struct EntityRecord {
    pub(crate) handle: RigidBodyHandle,
    /// 实体唯一 collider(insert 1:1 创建;支撑判定的接触对查询键,任务 03)。
    pub(crate) collider: ColliderHandle,
    pub(crate) desc: EntityDesc,
    /// 连续"低速"tick 计数(P2 触发用)。
    pub(crate) slow_ticks: u32,
    /// 演员标记(REQ-002 决议 4b 政策层豁免):置位后完全豁免自动凝固
    /// (P2 计数与睡眠兜底两条路径),仅显式命令将来触发。默认 false。
    pub(crate) actor: bool,
    /// 实体来源(REQ-006):外部 spawn = 玩家;服务端/内部 = None。
    /// 迁移/传送必须保留;快照随记录走。
    pub(crate) source: Option<InputSource>,
    /// 位移门滚动窗口:最近 21 个 translation(首尾距离 = 窗口位移)。
    /// 实体存在期间持续滚动;凝固/迁移重建后随实体重置为单点(细则 1)。
    pub(crate) window: VecDeque<Vec3>,
}

/// 单分片模拟器(字段 crate 内可见:snapshot.rs 读写快照结构)。
pub struct ShardSim {
    pub(crate) cfg: ShardSimConfig,
    pub(crate) registry: Box<dyn BlockRegistry>,
    pub(crate) voxels: VoxelWorld,
    pub(crate) phys: PhysicsWorld,
    pub(crate) entities: BTreeMap<EntityId, EntityRecord>,
    pub(crate) chunk_bodies: BTreeMap<ChunkPos, RigidBodyHandle>,
    /// ② 已建集合(修订 5 D18:**独立状态**,与 chunk 是否生成了碰撞盒无关)。
    /// 全空气 chunk 的碰撞体为空(chunk_bodies 无条目),但同样算 ② 已建——
    /// 否则每过一个冷却周期被重新提为构建候选(白耗 ② 预算),03 的 I2 判定
    /// 会把全空气 chunk 永远当"缺地形"。I1 断言、plan 的 ② 输入、快照均以
    /// 本集合为准。
    pub(crate) built_colliders: BTreeSet<ChunkPos>,
    /// R1(D18 情况 1)"装载时附带建 ②"的暂存集合:plan 在 N 拍决定
    /// `colliders_pending_build`,`apply_activation` 把它存到这里;`apply_chunk_load`
    /// 送 ① 到场时同点构建并移除条目;未兑现条目(驱动未执行该装载)在
    /// 下一次 `apply_activation` 开头作废,由该拍重新决策。随快照走(恢复后
    /// 驱动继续送 ①,集合在场即兑现——不进快照会有"有①无②"一拍窗口,
    /// 违反 D18 默认配置逐拍相等)。
    pub(crate) pending_collider_builds: BTreeSet<ChunkPos>,
    /// 待地形实体(REQ-007 §4.6;任务 03/提交 B):LocomotionState 保存的
    /// 速度随档;恢复时原样写回(位级)。未启用模式(D17)恒空。
    pub(crate) locomotion: BTreeMap<EntityId, LocomotionState>,
    /// 待执行强制落地(REQ-007 §4.8):到期内部指令/超预算淘汰进入;中心柱
    /// ① 缺 → 推迟(阻塞请求),柱就绪的拍执行。随快照(R10)。
    pub(crate) forced_pending: BTreeSet<EntityId>,
    pub(crate) outbox: Outbox,
    /// 激活策略机(滞回/冷却/预算簿记;任务书 02)。
    pub(crate) activation: crate::activation::ActivationController,
    /// 激活内部拍:`apply_activation` 已执行次数(随快照走,恢复后冷却
    /// 相对差不变;`apply_activation` 签名无 tick 参数,由本计数供拍)。
    pub(crate) activation_now: Tick,
    /// 观察者注册状态(README 决策 6 的命令唤醒淘汰前提;随快照走,语义同
    /// 任务书 02 细则 5 的 Option 开关:`None` = 从未注册 → 激活完全不生效,
    /// 行为与历史一致;`Some` = 最近一次 `apply_activation` 的观察者表,
    /// 命令唤醒路径的预算强制以其为距离中心)。
    pub(crate) observed: Option<Vec<Vec3>>,
}

impl ShardSim {
    pub fn new(cfg: ShardSimConfig, registry: Box<dyn BlockRegistry>) -> Self {
        let mut phys = PhysicsWorld::new();
        phys.gravity = conv::vec3_to_rapier(cfg.gravity);
        phys.integration_parameters.dt = cfg.dt;
        let activation = crate::activation::ActivationController::new(cfg.activation);
        Self {
            cfg,
            registry,
            voxels: VoxelWorld::new(),
            phys,
            entities: BTreeMap::new(),
            chunk_bodies: BTreeMap::new(),
            built_colliders: BTreeSet::new(),
            pending_collider_builds: BTreeSet::new(),
            locomotion: BTreeMap::new(),
            forced_pending: BTreeSet::new(),
            outbox: Outbox::default(),
            activation,
            activation_now: Tick::ZERO,
            observed: None,
        }
    }

    pub fn shard(&self) -> ShardId {
        self.cfg.shard
    }

    /// 体素世界只读视图(测试/导出用)。
    pub fn voxel(&self) -> &VoxelWorld {
        &self.voxels
    }

    pub fn entity_count(&self) -> usize {
        self.entities.len()
    }

    /// 实体当前状态;不存在(已迁移/已凝固)返回 None。
    pub fn entity_state(&self, id: EntityId) -> Option<EntityState> {
        let rec = self.entities.get(&id)?;
        Some(self.read_state(rec))
    }

    /// 全实体表(观察输出用,REQ-003 §4.4):`(EntityId, EntityState)` 列表,
    /// 按 EntityId 升序(底层 BTreeMap 键序,确定性)。空表 → 空列表。
    pub fn entity_states(&self) -> Vec<(EntityId, EntityState)> {
        self.entities
            .iter()
            .map(|(id, rec)| (*id, self.read_state(rec)))
            .collect()
    }

    /// 全实体归属元数据(REQ-006 任务 00:调度器恢复 ownership 用):
    /// `(EntityId, source, actor)` 按 EntityId 升序;不暴露 rapier handle。
    pub fn entity_owners(&self) -> Vec<(EntityId, Option<InputSource>, bool)> {
        self.entities
            .iter()
            .map(|(id, rec)| (*id, rec.source, rec.actor))
            .collect()
    }

    /// 加载一个体素 chunk 并(与已加载邻居一起)重建边界 collider。
    ///
    /// REQ-007 任务 01(①② 拆分/D17):本方法 = **合一装载**(① + ② 同建),
    /// 供激活未启用模式(§4.0:行为与 `8702670` 一致)与既有测试。启用模式
    /// 的驱动路径 = [`Self::load_data_only`](只装 ①),② 由激活决策构建。
    pub fn load_chunk(&mut self, cp: ChunkPos, chunk: Chunk) {
        self.load_data_only(cp, chunk);
        self.built_colliders.insert(cp);
        for ncp in expand_boundary([cp], &self.voxels) {
            self.rebuild_chunk_collider(ncp);
        }
    }

    /// 只装 ①(体素),不建本 chunk ②;已有 ② 的邻居照常重建(REQ-007
    /// 任务 01 细则 5)。启用模式的驱动装载路径。R1(D18 情况 1):若该 chunk
    /// 在 pending 集合(N 拍"随 ① 同点构建"决策),① 到场同点构建并出集合。
    pub fn load_data_only(&mut self, cp: ChunkPos, chunk: Chunk) {
        self.voxels.insert_chunk(cp, chunk);
        if self.pending_collider_builds.remove(&cp) {
            self.build_chunk_collider(cp);
        }
        for ncp in expand_boundary([cp], &self.voxels) {
            // 只重建**已有 ②** 的邻居(无 ② 的邻居不因邻居 ① 到达而新建)。
            if self.built_colliders.contains(&ncp) {
                self.rebuild_chunk_collider(ncp);
            }
        }
    }

    /// 装载的 D17 开关臂(调度器 `apply_chunk_load` 调用;REQ-007 §4.0):
    /// enabled → 只 ①(pending 的随 ① 同点建);未启用 → 合一(① + ② 同建,
    /// 行为与 8702670 一致)。
    pub fn apply_chunk_load_tiered(&mut self, cp: ChunkPos, chunk: Chunk, enabled: bool) {
        if enabled {
            self.load_data_only(cp, chunk);
        } else {
            self.load_chunk(cp, chunk);
        }
    }

    /// 卸载一个 chunk(返回它供持久化屏障),重建邻居边界 collider。
    ///
    /// REQ-007 任务 01(I1):卸 ① 必同步拆 ②(同一执行点;debug 断言
    /// "有 ② 必有 ①" 的对称操作)。
    pub fn unload_chunk(&mut self, cp: ChunkPos) -> Option<Chunk> {
        let removed = self.voxels.remove_chunk(cp);
        if let Some(handle) = self.chunk_bodies.remove(&cp) {
            self.phys.remove_body(handle);
        }
        self.built_colliders.remove(&cp);
        self.pending_collider_builds.remove(&cp);
        // 边界重建同样只动已建 ②:邻居 ① 卸载不给无 ② 的邻居新建 ②(细则 4;
        // 未启用模式所有 ① 皆有 ②,守卫恒真,行为不变)。
        for ncp in expand_boundary([cp], &self.voxels) {
            if self.built_colliders.contains(&ncp) {
                self.rebuild_chunk_collider(ncp);
            }
        }
        removed
    }

    /// ② 是否已建(测试/I1 断言/观测用;D18:独立状态,与碰撞盒是否存在
    /// 无关——全空气 chunk 同样算已建)。
    pub fn has_collider(&self, cp: ChunkPos) -> bool {
        self.built_colliders.contains(&cp)
    }

    /// ② 已建集合(快照重建/观测用;BTreeSet 键序 = ChunkPos 升序)。
    pub fn collider_loaded_chunks(&self) -> BTreeSet<ChunkPos> {
        self.built_colliders.clone()
    }

    /// 只建 ②(体素在手;plan 的 colliders_to_build / pending 兑现的执行点,
    /// B3)。无 ① → panic(② 构建前提:① 已载;plan 候选已过滤,违反 = bug)。
    /// 登记 **built_colliders**(D18 独立状态;碰撞盒为空的全空气 chunk 同样
    /// 登记为已建)。
    fn build_chunk_collider(&mut self, cp: ChunkPos) {
        assert!(
            self.voxels.get_chunk(cp).is_some(),
            "构建 ② 时 ① 未装(I1 违反,fail-stop):{cp:?}"
        );
        debug_assert!(
            !self.built_colliders.contains(&cp),
            "重复构建 ②(plan 决策应过滤已建;fail-stop):{cp:?}"
        );
        self.rebuild_chunk_collider(cp);
        self.built_colliders.insert(cp);
    }

    /// 只拆 ②(① 保留;plan 的 colliders_to_drop 执行点,B3/R2)。
    /// 注销 built_colliders + 移除物理体(若有);幂等(防御)。
    fn drop_chunk_collider(&mut self, cp: ChunkPos) {
        self.built_colliders.remove(&cp);
        if let Some(handle) = self.chunk_bodies.remove(&cp) {
            self.phys.remove_body(handle);
        }
    }

    /// 应用一个 tick 的输入前缀(命令/事件/信封,按 canonical 序)。
    ///
    /// README 决策 6(绑定):命令唤醒路径(Spawn/Teleport 重建即醒,§9.12)
    /// 超预算时**同一拍立即**按距离淘汰(`enforce_budget` 语义;hook 在 apply
    /// 末尾、step 之前,淘汰与 step 同拍生效)。淘汰 = 冻结落实(清速 +
    /// kinematic)+ 按决策即簿记记入冷却表;追加产出并入 outbox 无额外通道。
    /// 预算内 → 零额外动作(唤醒当拍行为与历史完全一致)。
    pub fn apply_prefix(&mut self, inputs: &[StampedInput]) {
        let mut sorted: Vec<&StampedInput> = inputs.iter().collect();
        sorted.sort_by_key(|i| i.canonical_key());
        for input in sorted {
            match input.payload {
                InputPayload::Command(cmd) => self.apply_command(cmd, input.source),
                InputPayload::Event(CrossEvent::Detonate {
                    center,
                    power,
                    radius,
                }) => self.detonate(center, power, radius),
                InputPayload::Envelope(e) => match e.payload {
                    EnvelopePayload::MigrateEntity {
                        entity,
                        state,
                        desc,
                        actor,
                        source,
                    } => self.spawn_entity(entity, state, desc, source, actor),
                    // 涌现事件(REQ-005 任务 03):共享参数本地求值,复用同一
                    // 纯函数;冻结体免疫语义不变(§9.12)。CrossEvent 未来变体
                    // 追加时本 match 穷举强制处理。
                    EnvelopePayload::Event(CrossEvent::Detonate {
                        center,
                        power,
                        radius,
                    }) => self.detonate(center, power, radius),
                    // 涌现 block write(REQ-005 任务 03):目标 chunk 必已装载
                    // (owner/holder 才会被盖章为收件人;未装载 = 内部一致性
                    // 违规 fail-stop),写体素 + 重建边界 collider;**不推
                    // `outbox.block_updates`**(防广播回环)。
                    EnvelopePayload::BlockWrite { pos, block } => {
                        assert!(
                            self.voxels.get_chunk(pos.chunk_of()).is_some(),
                            "BlockWrite 信封落到未装载 chunk {pos:?}(owner/holder 必然\
                             装载着该 chunk,内部一致性违规,fail-stop)"
                        );
                        if self.voxels.set_block(pos, block).is_ok() {
                            // 细则 4/5:只重建已建 ②(同 SetBlock 守卫)。
                            for cp in expand_boundary([pos.chunk_of()], &self.voxels) {
                                if self.built_colliders.contains(&cp) {
                                    self.rebuild_chunk_collider(cp);
                                }
                            }
                        }
                    }
                },
            }
        }
        self.enforce_budget_after_command_wake();
    }

    /// 命令唤醒路径的预算强制(README 决策 6):观察者已注册过(激活生效中)
    /// 且醒数超限时,按到最近观察者距离降序给出应冻结者并**立即落实**。
    /// 淘汰条目照 plan 的超预算路径记入冷却簿记(唤醒须等冷却/显式命令)。
    /// 从未注册观察者 → 恒空,行为与历史一致(向后兼容开关,任务书 02 细则 5)。
    fn enforce_budget_after_command_wake(&mut self) {
        let Some(observers) = self.observed.clone() else {
            return;
        };
        let awake: Vec<(EntityId, Vec3)> = self
            .entities
            .iter()
            .map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                (*id, conv::rapier_to_vec3(b.translation()))
            })
            .collect();
        for id in self.activation.enforce_budget(&observers, &awake) {
            let now = self.activation_now;
            // §9-2/任务书细则 10(瞬态半边):被淘汰的**瞬态体 → 强制落地**
            // (产出同 §4.8;非冻结);常驻体 → 冻结(04 起改 End ActorMotion)。
            let is_transient = self.entities.get(&id).is_some_and(|rec| !rec.actor);
            if is_transient {
                self.forced_pending.insert(id);
            } else {
                self.freeze_entity(&id);
                self.activation.note_flip(&id, now);
            }
        }
    }

    /// 冻结落实(tier③;apply_activation 与命令唤醒淘汰共用):
    /// rapier 0.36 语义:KinematicPositionBased 下 set_linvel/set_angvel 是
    /// no-op → 必须先清速度再切类型。
    fn freeze_entity(&mut self, id: &EntityId) {
        let rec = self
            .entities
            .get(id)
            .expect("决策实体必在实体表(输入即来源)");
        let body = self.phys.bodies.get_mut(rec.handle).expect("handle 有效");
        body.set_linvel(conv::vec3_to_rapier(Vec3::ZERO), true);
        body.set_angvel(conv::vec3_to_rapier(Vec3::ZERO), true);
        body.set_body_type(RigidBodyType::KinematicPositionBased, true);
    }

    /// 推进一拍:物理步进 → 凝固桥检测 → 迁移检测(demo A/C 时序)。
    pub fn step(&mut self) {
        self.phys.step();

        let ids: Vec<EntityId> = self.entities.keys().copied().collect();
        // 支撑判定(§4.4.1;一次计算,凝固前提 + 承载产出共用)。
        let entity_supported = self.evaluate_supports();
        let orients = solidify::axis_aligned_orientations();
        let cfg = self.cfg.solidify.clone();
        for id in ids {
            let Some(rec) = self.entities.get(&id) else {
                continue;
            };
            let (handle, desc, slow_ticks_base, source) =
                (rec.handle, rec.desc, rec.slow_ticks, rec.source);
            let state = self.read_state(rec);
            let is_sleeping = self
                .phys
                .bodies
                .get(handle)
                .expect("handle 有效")
                .is_sleeping();
            // 冻结体(kinematic)跳过凝固桥与迁移检测(§9.12/任务书 02):
            // 不凝固(冻结清零速度会使 slow 判据恒真)、不迁移(防御性;
            // kinematic 零速度本就静止,两跳为不变量的显式保证)。
            let is_frozen = self
                .phys
                .bodies
                .get(handle)
                .expect("handle 有效")
                .is_kinematic();
            if is_frozen {
                continue;
            }

            // ---- 位移门滚动窗口(P2′,REQ-001 §9.5 修订)----
            // 每拍采当前 translation;窗口在实体存在期间持续滚动,首尾距离
            // 即"最近 20 tick 位移"。演员豁免只影响凝固,窗口照常滚动。
            let gate_passes = {
                let rec = self.entities.get_mut(&id).expect("存在");
                rec.window.push_back(state.translation);
                while rec.window.len() > DISPLACEMENT_WINDOW_LEN {
                    rec.window.pop_front();
                }
                cfg.displacement_gate_m <= 0.0
                    || rec.window.front().is_none_or(|oldest| {
                        (*oldest - state.translation).length() <= cfg.displacement_gate_m
                    })
            };

            // ---- 凝固桥(P2′ + 睡眠兜底,§9.5 修订)----
            // 演员实体(政策层,REQ-002 决议 4b)完全豁免自动凝固(两条
            // 自动路径),仅显式命令将来触发;位移门同时约束 P2 计数路径
            // 与睡眠兜底路径(决议 d:兜底不得视为误凝固保险)。
            let slow = state.linvel.length() < cfg.linear_threshold
                && state.angvel.length() < cfg.angular_threshold;
            let slow_ticks = if slow { slow_ticks_base + 1 } else { 0 };
            let actor = {
                let rec = self.entities.get_mut(&id).expect("存在");
                rec.slow_ticks = slow_ticks;
                rec.actor
            };
            // 凝固前提修订(§4.7/任务 03):本拍无实体支撑(被承载瞬态不凝固,
            // §4.4.2)。零重力不判定支撑 → 前提恒真(既有零重力用例不变,§9-6)。
            let no_entity_support = !entity_supported.get(&id).copied().unwrap_or(false);
            if cfg.enabled
                && !actor
                && no_entity_support
                && gate_passes
                && (slow_ticks >= cfg.consecutive_ticks || is_sleeping)
            {
                let snapped = solidify::nearest_orientation(state.rotation, &orients);
                let (cell, _center, disp) = solidify::snap_position(state.translation);
                let ang_err = solidify::snap_angle_error(state.rotation, snapped);
                let too_far =
                    disp > cfg.max_snap_blocks || ang_err > cfg.max_snap_angle_deg.to_radians();
                if !too_far || is_sleeping {
                    // 睡眠兜底:睡着即无条件凝固;未睡但误差过大 → 继续等(升级路径)
                    if self.voxels.set_block(cell, desc.block).is_ok() {
                        // 细则 4/5:只重建已建 ②(同 SetBlock 守卫)。
                        for cp in expand_boundary([cell.chunk_of()], &self.voxels) {
                            if self.built_colliders.contains(&cp) {
                                self.rebuild_chunk_collider(cp);
                            }
                        }
                        self.entities.remove(&id).expect("存在");
                        self.phys.remove_body(handle);
                        self.outbox.despawned.push(id);
                        // 涌现写入(REQ-005 任务 03):凝固桥经 block_updates 的
                        // `Emergent` 出片——调度器 tick 末发布产生者 + 对
                        // owner/holders 盖 `EnvelopePayload::BlockWrite`
                        // (apply_at = N+1);`solidified` 兼容字段自本任务恒空。
                        self.outbox.block_updates.push(BlockWrite {
                            pos: cell,
                            block: desc.block,
                            kind: BlockWriteKind::Emergent,
                        });
                        continue;
                    }
                }
            }

            // ---- 迁移检测(实体中心离开本体区,demo A)----
            if !self.core_contains(state.translation) {
                let dest = ShardId::new(
                    self.cfg.shard.world,
                    self.cfg.shard.dimension,
                    pos_chunk(state.translation).shard_of(),
                );
                self.entities.remove(&id).expect("存在");
                self.phys.remove_body(handle);
                self.outbox.despawned.push(id);
                self.outbox.migrations.push(Intent {
                    destination: dest,
                    payload: EnvelopePayload::MigrateEntity {
                        entity: id,
                        state,
                        desc,
                        actor,
                        source,
                    },
                });
            }
        }
    }

    /// 取走并清空本 tick 产出。
    pub fn take_outbox(&mut self) -> Outbox {
        std::mem::take(&mut self.outbox)
    }

    /// 激活刷新(README 决策 5):tier③ 落实(冻结 = set_body_type
    /// KinematicPositionBased + 清速度;唤醒 = Dynamic + wake_up),
    /// tier①② 以决策返回(chunk 数据源在 REQ-002,本轮调用方可忽略)。
    /// 冻结体跳过:detonate(§9.12 事件免疫)、凝固桥、迁移检测(防御性)。
    ///
    /// 供拍的激活时钟 = 本方法已执行次数(随快照走,恢复后冷却相对差不变);
    /// 每 tick 恰好调用一次时即全局拍号。同输入两次调用第二次输出空
    /// (决策即簿记,见 `ActivationController::plan`)。首次调用即注册
    /// 观察者(`observed = Some`,覆盖式更新)→ 命令唤醒路径的预算强制
    /// 随之生效(README 决策 6)。
    /// 本分片允许加载的静态范围 = core ∪ margin_ring(REQ-005 D2/D4;
    /// `ChunkRegion::core(shard).expanded(region_margin_chunks)`)。
    /// holder 副本只允许出现在这个范围(调度器装载校验用)。
    pub fn activation_region(&self) -> types::ChunkRegion {
        types::ChunkRegion::core(self.cfg.shard).expanded(self.cfg.activation.region_margin_chunks)
    }

    pub fn apply_activation(&mut self, observers: &[Vec3]) -> ActivationDecisions {
        // 观察者 → 一张 Observer 票(REQ-007 任务 02:观察者票等价改造;
        // 半径取本 sim 激活配置,order 0/priority 0 与 01 逐位一致)。
        let cfg = self.cfg.activation;
        let tickets: Vec<crate::tickets::TicketView> = observers
            .iter()
            .map(|&pos| crate::tickets::TicketView {
                order: 0,
                priority: 0,
                data: crate::tickets::RegionSpec::Radius {
                    load_m: cfg.data_load_radius,
                    unload_m: cfg.data_unload_radius,
                },
                collider: crate::tickets::RegionSpec::Radius {
                    load_m: cfg.collider_load_radius,
                    unload_m: cfg.collider_unload_radius,
                },
                wake: crate::tickets::WakeSpec::Radius {
                    load_m: cfg.awake_load_radius,
                    unload_m: cfg.awake_unload_radius,
                },
                state: crate::tickets::TicketState::Active,
                anchor: pos,
            })
            .collect();
        self.apply_activation_tickets(&tickets, observers)
    }

    /// 票据视图入口(REQ-007 任务 02:调度器的生产路径)。`observed_centers`
    /// = 命令唤醒预算强制的距离中心(唤醒中心锚点;调度器传全部占区域票锚点
    /// ——与 01 的观察者表同语义)。
    pub fn apply_activation_tickets(
        &mut self,
        tickets: &[crate::tickets::TicketView],
        observed_centers: &[Vec3],
    ) -> ActivationDecisions {
        let now = self.activation_now;
        self.activation_now = now.next();
        self.observed = Some(observed_centers.to_vec());
        // R1(D18 情况 1):上一拍 pending 未兑现的条目(驱动未执行该装载,
        // 例如观察者已移走致请求作废)就地作废——本拍 plan 重新决策。兑现过
        // 的条目已被 load_data_only 移除。
        self.pending_collider_builds.clear();
        let region = self.activation_region();
        let loaded: Vec<ChunkPos> = self.voxels.chunk_positions().collect();
        // ② 已建集合(REQ-007 任务 01;修订 5 D18:独立状态,非 chunk_bodies 键
        // ——全空气 chunk 无碰撞盒但 ② 已建)。
        let collider_loaded: Vec<ChunkPos> = self.built_colliders.iter().copied().collect();
        // H2:carried = 有实体支撑(§4.4.1)——被承载瞬态按常驻规则参与滞回。
        let supports = self.evaluate_supports();
        let entities: Vec<(EntityId, Vec3, bool, bool, bool)> = self
            .entities
            .iter()
            .map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                (
                    *id,
                    conv::rapier_to_vec3(b.translation()),
                    b.is_kinematic(),
                    rec.actor,
                    supports.get(id).copied().unwrap_or(false),
                )
            })
            .collect();
        // 阻塞请求(D15;判定只依实体状态,§14-8):待地形实体的足迹缺 ② +
        // 缺 ①(§4.12 三来源的前两个;强制落地柱缺 ① 在后续提交接)。
        let blocked = self.awaiting_terrain_blocked();
        let decisions = self.activation.plan(
            tickets,
            &loaded,
            &collider_loaded,
            &entities,
            now,
            region,
            &blocked,
        );

        // tier② 落实(REQ-007 任务 01/B3:N3——决策在 plan,执行在此,与
        // ③ 冻结/唤醒同方式;驱动层不执行 ②):
        // - 单独 build(D18 情况 2):同 load_chunk 的 collider 语义(体素在手,
        //   greedy + compound);step 之前生效;
        // - pending(D18 情况 1):存入暂存集合,`apply_chunk_load` 送 ① 到场
        //   时同点构建(load_data_only);plan 已过滤"本拍就装载"的重复
        //   (pending ∩ chunks_to_load = ∅,装载候选必未载);
        // - drop(R2):只拆"① 仍被保留"(≤ data_unload)而 ② 超 collider_unload
        //   的;① 卸载/排队待卸的 chunk,② 随 ① 在卸载点一起拆(I1)。
        for &cp in &decisions.colliders_to_build {
            self.build_chunk_collider(cp);
        }
        debug_assert!(
            decisions
                .colliders_pending_build
                .iter()
                .all(|cp| !self.built_colliders.contains(cp)),
            "pending ② 构建条目不得已建(plan 应过滤;fail-stop)"
        );
        self.pending_collider_builds
            .extend(decisions.colliders_pending_build.iter().copied());
        for &cp in &decisions.colliders_to_drop {
            self.drop_chunk_collider(cp);
        }

        // tier③ 落实:施体类型切换。H2:被承载瞬态的冻结条目加**静止前置**
        // (§4.4.2:静止 ∧ 被承载才冻结;运动中的被承载瞬态不冻结);唤醒
        // 条目加**当前足迹 ② 前置**(缺 → 不唤醒,交由待地形状态机处理
        // ——kinematic 待地形体在 ② 齐时由 apply_awaiting_terrain 恢复,
        // 速度原样写回,语义等价"推迟唤醒"而非丢弃)。
        for id in &decisions.entities_to_freeze {
            let (is_transient_carried, static_now) = {
                let Some(rec) = self.entities.get(id) else {
                    continue;
                };
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                let carried = supports.get(id).copied().unwrap_or(false);
                let static_now =
                    rec.slow_ticks >= self.cfg.solidify.consecutive_ticks || b.is_sleeping();
                (!rec.actor && carried, static_now)
            };
            if is_transient_carried && !static_now {
                continue; // 被承载但仍在运动:不冻结(§4.4.2 静止前置)
            }
            self.freeze_entity(id);
        }
        for id in &decisions.entities_to_wake {
            let Some(rec) = self.entities.get(id) else {
                continue;
            };
            // 唤醒前置:当前足迹 ∩ 区域 ② 齐(§4.9 共同规则 1;瞬态体)。
            if !rec.actor {
                let pos = {
                    let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                    conv::rapier_to_vec3(b.translation())
                };
                let region = self.activation_region();
                let ready = crate::settle::current_footprint(
                    pos,
                    rec.desc.half_size,
                    self.cfg.activation.terrain_guard_m,
                )
                .iter()
                .all(|cp| !region.contains(*cp) || self.built_colliders.contains(cp));
                if !ready {
                    continue; // 推迟唤醒(待地形状态机接管)
                }
            }
            let body = self.phys.bodies.get_mut(rec.handle).expect("handle 有效");
            body.set_body_type(RigidBodyType::Dynamic, true);
        }

        // ---- 待地形(REQ-007 §4.6;任务 03/提交 B;S4 I2 完整执行)----
        // 判定点 = apply_activation(step 之前);判定只对**瞬态体**(actor
        // == false;常驻体 04)。I2:当前足迹 ∩ 本分片区域内每 chunk 都有 ②;
        // 缺 → Kinematic + 保存速度(位级);齐 → 恢复(原样写回 + 重置静止
        // 判据)。期间不计醒数(plan 实体档的 frozen 输入 = kinematic 已含)、
        // 不凝固不迁移(step 的 is_frozen 跳过)、事件免疫(step 前缀 detonate
        // 对 kinematic 无效=既有免疫)。
        self.update_awaiting_terrain();
        decisions
    }

    /// 待地形进入/恢复(§4.6;瞬态体;调用点 = apply_activation 末与
    /// spawn_entity)。返回本拍阻塞请求(足迹 ∩ 区域内缺 ② 的 chunk + 待地形
    /// 实体的当前足迹缺 ①;D15:免冷却、排序最前、仍占预算)。
    /// 待地形阻塞请求(纯读;§4.6/§4.12):待地形实体足迹 ∩ 区域内缺 ②/缺 ①。
    fn awaiting_terrain_blocked(&self) -> BTreeSet<ChunkPos> {
        let mut blocked = BTreeSet::new();
        let region = self.activation_region();
        for (id, st) in &self.locomotion {
            if !matches!(st, LocomotionState::AwaitingTerrain { .. }) {
                continue;
            }
            let Some(rec) = self.entities.get(id) else {
                continue;
            };
            let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
            let pos = conv::rapier_to_vec3(body.translation());
            for cp in crate::settle::current_footprint(
                pos,
                rec.desc.half_size,
                self.cfg.activation.terrain_guard_m,
            ) {
                if !region.contains(cp) {
                    continue;
                }
                if !self.built_colliders.contains(&cp) || !self.voxels.get_chunk(cp).is_some() {
                    blocked.insert(cp);
                }
            }
        }
        blocked
    }

    fn update_awaiting_terrain(&mut self) -> BTreeSet<ChunkPos> {
        let mut blocked: BTreeSet<ChunkPos> = BTreeSet::new();
        let region = self.activation_region();
        let ids: Vec<EntityId> = self.entities.keys().copied().collect();
        for id in ids {
            let Some(rec) = self.entities.get(&id) else {
                continue;
            };
            if rec.actor {
                continue; // 常驻体 04
            }
            let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
            let pos = conv::rapier_to_vec3(body.translation());
            let is_kinematic = body.is_kinematic();
            let footprint = crate::settle::current_footprint(
                pos,
                rec.desc.half_size,
                self.cfg.activation.terrain_guard_m,
            );
            let missing_collider: BTreeSet<ChunkPos> = footprint
                .iter()
                .filter(|cp| region.contains(**cp) && !self.built_colliders.contains(*cp))
                .copied()
                .collect();
            let missing_data: BTreeSet<ChunkPos> = footprint
                .iter()
                .filter(|cp| region.contains(**cp) && !self.voxels.get_chunk(**cp).is_some())
                .copied()
                .collect();
            let awaiting = matches!(
                self.locomotion.get(&id),
                Some(LocomotionState::AwaitingTerrain { .. })
            );
            if missing_collider.is_empty() && missing_data.is_empty() {
                // 足迹齐:若在待地形 → 恢复(切回 Dynamic + 原样写回速度;
                // 重置静止判据 = 恢复属唤醒语义)。
                if let Some(LocomotionState::AwaitingTerrain {
                    saved_linvel,
                    saved_angvel,
                }) = self.locomotion.remove(&id)
                {
                    let rec = self.entities.get_mut(&id).expect("存在");
                    rec.slow_ticks = 0;
                    rec.window.clear();
                    rec.window.push_back(pos);
                    let body = self.phys.bodies.get_mut(rec.handle).expect("handle 有效");
                    body.set_linvel(conv::vec3_to_rapier(saved_linvel), true);
                    body.set_angvel(conv::vec3_to_rapier(saved_angvel), true);
                    body.set_body_type(RigidBodyType::Dynamic, true);
                } else if !awaiting {
                    self.locomotion.entry(id).or_insert(LocomotionState::Awake);
                }
            } else {
                // 缺 ②(或缺 ①):进入待地形(醒着 Dynamic 才切;已在待地形
                // 保持)。速度保存位级;已 Kinematic(被滞回冻结)的瞬态不再
                // 存在(03 实体档移除瞬态冻结;防御:不覆盖)。
                blocked.extend(missing_collider.iter().copied());
                blocked.extend(missing_data.iter().copied());
                if !is_kinematic {
                    let body = self.phys.bodies.get_mut(rec.handle).expect("handle 有效");
                    let lv = conv::rapier_to_vec3(body.linvel());
                    let av = conv::rapier_to_vec3(body.angvel());
                    body.set_linvel(conv::vec3_to_rapier(Vec3::ZERO), true);
                    body.set_angvel(conv::vec3_to_rapier(Vec3::ZERO), true);
                    body.set_body_type(RigidBodyType::KinematicPositionBased, true);
                    self.locomotion.insert(
                        id,
                        LocomotionState::AwaitingTerrain {
                            saved_linvel: lv,
                            saved_angvel: av,
                        },
                    );
                } else if !awaiting {
                    // kinematic 但非待地形(异常:防御性按醒着待处理?不——
                    // 瞬态无滞回冻结路径;此分支 = 未知来源 kinematic,保持)。
                    self.locomotion.entry(id).or_insert(LocomotionState::Awake);
                }
            }
        }
        blocked
    }

    /// 到期内部指令应用(§4.3.2;任务 03):实体进入待执行强制落地集。
    /// 调度器 SCHEDULER 来源(不入外部日志);实体在本分片才收(在途时由
    /// ownership 切换后的目的分片收)。
    pub fn apply_forced_landing_directive(&mut self, entity: EntityId) {
        self.forced_pending.insert(entity);
    }

    /// 实体票 ops 应用入口(分片产出 → 调度器盖章 → 目的分片 N+1 应用;
    /// 幂等语义在调度器 TicketTable,本入口仅为对称的执行侧占位——实际状态
    /// 变化(Pause 等)落在票上,分片无票表,无本地动作)。
    pub fn apply_ticket_ops(&mut self, _ops: &[TicketOpRecord]) {
        // 分片侧无票据表:Pause/Suspend 等状态由调度器表持有;分片的职责是
        // **判定与产出**(step 内按实体支撑/承载状态产出 ops)。执行侧 no-op。
    }

    /// 支撑判定(§4.4.1;R5):对每个**醒着**(Dynamic,非待地形)实体,检查
    /// 其接触对:法向(从对方指向 E)· û ≥ support_normal_min → 对方为支撑;
    /// 对方 = chunk 静态体 → 地形支撑,实体 → 实体支撑。零重力不判定(全 None
    /// → 凝固前提恒真,§9-6)。rapier 接触流形法向从 collider1 指向 collider2,
    /// 按配对顺序换算方向。
    pub fn evaluate_supports(&self) -> std::collections::BTreeMap<EntityId, bool> {
        let mut out = std::collections::BTreeMap::new();
        let gravity_len = self.cfg.gravity.length();
        if gravity_len <= 0.0 {
            return out; // 零重力:不判定(前提恒真)
        }
        let up = -self.cfg.gravity / gravity_len; // û = 重力反向单位向量
        // 实体刚体 → 其 collider 句柄(insert 时 1:1 创建)。
        for (id, rec) in &self.entities {
            let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
            if body.is_kinematic() {
                continue; // 只有醒着(含待地形外的 Dynamic)需要判定
            }
            let self_collider = rec.collider;
            let has_entity_support = self
                .phys
                .narrow_phase
                .contact_pairs_with(self_collider)
                .any(|pair| {
                    // 接触对无全局序:按"我方 collider"归一。
                    let (other_collider, e_is_first) = if pair.collider1 == self_collider {
                        (pair.collider2, true)
                    } else if pair.collider2 == self_collider {
                        (pair.collider1, false)
                    } else {
                        return false;
                    };
                    // 对方 collider 所属刚体:实体(非固定)→ 潜在实体支撑。
                    let Some(other_body) = self
                        .phys
                        .colliders
                        .get(other_collider)
                        .and_then(|c| c.parent())
                    else {
                        return false;
                    };
                    if self
                        .phys
                        .bodies
                        .get(other_body)
                        .is_none_or(|b| b.is_fixed())
                    {
                        return false; // 地形支撑(非实体支撑)
                    }
                    // 法向(G2 返修):`manifold.data.normal` 是**世界系**法向
                    // (从 collider1 指向 collider2;rapier 0.36
                    // `ContactManifoldData::normal`);归一到"从对方指向 E":
                    // E 为 collider1 → n = -normal,E 为 collider2 → n =
                    // +normal。不用 `local_n1`(shape 局部系:方块侧翻 90° 时
                    // 局部"上"对应世界"侧",cos 阈值盖不住)。
                    pair.manifolds().iter().any(|m| {
                        let n = if e_is_first {
                            -m.data.normal
                        } else {
                            m.data.normal
                        };
                        n.dot(conv::vec3_to_rapier(up)) >= self.cfg.activation.support_normal_min
                    })
                });
            out.insert(*id, has_entity_support);
        }
        out
    }

    /// 承载判定与 ticket_ops 产出(§4.3.4/§4.4.2;G1 返修):对每个**瞬态体**
    /// 按"实体支撑 × 静止 × 唤醒范围"产出状态操作(调度器拍末按键序应用,
    /// N 拍产生 N+1 生效;幂等由 TicketTable 保证,重复产出无副作用):
    /// - 被承载(有实体支撑)→ Pause;
    /// - 被承载 ∧ 静止 ∧ 不在任何唤醒范围 → 冻结 + Suspend(票不占区域,
    ///   §4.4.2"被承载的瞬态体按常驻规则冻结");
    /// - 被承载的冻结体进入唤醒范围 → Resume + 唤醒;
    /// - 不被承载且醒着 → Unpause(幂等;承载结束)。
    fn emit_carry_ticket_ops(&mut self) {
        let supports = self.evaluate_supports();
        let wake_centers: Vec<Vec3> = self.observed.clone().unwrap_or_default();
        let ids: Vec<EntityId> = self.entities.keys().copied().collect();
        for id in ids {
            let Some(rec) = self.entities.get(&id) else {
                continue;
            };
            if rec.actor {
                continue; // 常驻票 04
            }
            let Some(body) = self.phys.bodies.get(rec.handle) else {
                continue;
            };
            let carried = carried_of(&supports, &id);
            let pos = conv::rapier_to_vec3(body.translation());
            let in_any_wake = wake_centers
                .iter()
                .any(|c| (*c - pos).length() <= self.cfg.activation.awake_load_radius);
            let kinematic = body.is_kinematic();
            let static_now =
                rec.slow_ticks >= self.cfg.solidify.consecutive_ticks || body.is_sleeping();
            if carried && !kinematic {
                // 被承载:暂停计时;静止 ∧ 不在唤醒范围 → 冻结 + Suspend。
                self.outbox.ticket_ops.push(TicketOpRecord {
                    entity: id,
                    op: TicketOp::Pause,
                });
                if static_now && !in_any_wake {
                    self.freeze_entity(&id);
                    self.outbox.ticket_ops.push(TicketOpRecord {
                        entity: id,
                        op: TicketOp::Suspend,
                    });
                }
            } else if carried && kinematic {
                // 被承载的冻结体:唤醒范围到 → Resume + 唤醒(整组原子在 05)。
                if in_any_wake {
                    self.outbox.ticket_ops.push(TicketOpRecord {
                        entity: id,
                        op: TicketOp::Resume,
                    });
                    let body = self.phys.bodies.get_mut(rec.handle).expect("handle 有效");
                    body.set_body_type(RigidBodyType::Dynamic, true);
                }
            } else if !carried && !kinematic {
                // 不被承载且醒着:Unpause(承载结束;已 Active 时幂等无操作)。
                self.outbox.ticket_ops.push(TicketOpRecord {
                    entity: id,
                    op: TicketOp::Unpause,
                });
            }
        }
    }

    /// step 末的瞬态体生命周期段(任务 03/提交 C):虚空移除 + 强制落地执行。
    /// 产出:despawned(+ forced 的 Emergent BlockWrite)/ void_removed。
    pub fn run_transient_lifecycle(&mut self) {
        // G1:承载判定与 ticket_ops 产出(§4.4.2;调度器拍末键序应用)。
        self.emit_carry_ticket_ops();
        // ---- 虚空移除(§4.8;所有实体;启用模式 = 已激活刷新过)----
        if self.observed.is_some() {
            let threshold = types::WORLD_Y_MIN as f32 - self.cfg.activation.void_despawn_depth_m;
            let ids: Vec<EntityId> = self.entities.keys().copied().collect();
            for id in ids {
                let (handle, y) = {
                    let rec = self.entities.get(&id).expect("keys 有效");
                    let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
                    (rec.handle, conv::rapier_to_vec3(body.translation()).y)
                };
                if y < threshold {
                    self.entities.remove(&id).expect("存在");
                    self.phys.remove_body(handle);
                    self.locomotion.remove(&id);
                    self.forced_pending.remove(&id);
                    self.outbox.despawned.push(id);
                    self.outbox.void_removed.push(id);
                }
            }
        }
        // ---- 强制落地执行(§4.8 五步;中心柱 ① 缺 → 推迟 + 阻塞)----
        let pending: Vec<EntityId> = self.forced_pending.iter().copied().collect();
        for id in pending {
            let Some(rec) = self.entities.get(&id) else {
                // 实体已不在(迁移在途/凝固/移除):出队(调度器终结判定兜底)。
                self.forced_pending.remove(&id);
                continue;
            };
            let (handle, block_desc) = (rec.handle, rec.desc);
            let body = self.phys.bodies.get(handle).expect("handle 有效");
            let pos = conv::rapier_to_vec3(body.translation());
            let cell = crate::settle::cell_of(pos.x, pos.y, pos.z);
            let cp = pos_chunk(pos);
            let Some(_chunk) = self.voxels.get_chunk(cp) else {
                continue; // 柱 ① 缺 → 推迟(阻塞请求由 awaiting_terrain_blocked
                // 同型逻辑在 plan 输入侧补充——见 blocked_forced)
            };
            // 借用收口:landing 计算的闭包借用 self.entities/VoxelWorld,块内
            // 结束后才能 &mut 移除实体。
            let landing = {
                let overlaps = |b: BlockPos| -> bool {
                    self.entities.iter().any(|(other, orec)| {
                        if *other == id {
                            return false;
                        }
                        let ob = self.phys.bodies.get(orec.handle).expect("handle 有效");
                        let op = conv::rapier_to_vec3(ob.translation());
                        let oc = crate::settle::cell_of(op.x, op.y, op.z);
                        oc == b
                    })
                };
                crate::settle::forced_landing_cell(
                    &|_x, by| Some(self.voxels.get_block(BlockPos::new(cell.x, by, cell.z))),
                    cell.x,
                    cell.y,
                    cell.z,
                    types::WORLD_Y_MIN,
                    types::WORLD_Y_MAX,
                    &overlaps,
                )
            };
            let handle = rec.handle;
            let Some(landing) = landing else {
                // 虚空移除(无支撑柱;无 BlockWrite)。
                self.entities.remove(&id).expect("存在");
                self.phys.remove_body(handle);
                self.locomotion.remove(&id);
                self.outbox.despawned.push(id);
                self.outbox.void_removed.push(id);
                self.forced_pending.remove(&id);
                continue;
            };
            // 落点写块(Emergent)+ despawn(与凝固桥同产出;复用 REQ-005
            // 广播持久化)。
            let _ = self.voxels.set_block(landing, block_desc.block);
            for ncp in expand_boundary([landing.chunk_of()], &self.voxels) {
                if self.built_colliders.contains(&ncp) {
                    self.rebuild_chunk_collider(ncp);
                }
            }
            self.outbox.block_updates.push(BlockWrite {
                pos: landing,
                block: block_desc.block,
                kind: BlockWriteKind::Emergent,
            });
            self.entities.remove(&id).expect("存在");
            self.phys.remove_body(handle);
            self.locomotion.remove(&id);
            self.outbox.despawned.push(id);
            self.outbox.forced_landings.push((id, landing));
            self.forced_pending.remove(&id);
        }
    }

    /// I2 断言(§14-7;debug):醒着的瞬态体当前足迹 ∩ 区域内缺 ② → panic。
    /// 钩子:测试以"醒着 + 缺 ②"状态直接调用。
    #[cfg(debug_assertions)]
    pub fn i2_assert_awake_footprint_ready(&self) {
        let region = self.activation_region();
        for (id, rec) in &self.entities {
            let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
            if body.is_kinematic() {
                continue; // 待地形/冻结体不算"醒着"
            }
            let pos = conv::rapier_to_vec3(body.translation());
            for cp in crate::settle::current_footprint(
                pos,
                rec.desc.half_size,
                self.cfg.activation.terrain_guard_m,
            ) {
                if region.contains(cp) {
                    assert!(
                        self.built_colliders.contains(&cp),
                        "I2 违反:醒着瞬态体 {id:?} 的足迹 chunk {cp:?} 缺 ②(待地形应已进入,fail-stop)"
                    );
                }
            }
        }
    }

    /// 激活配置(测试/注入用)。
    pub fn activation_config(&self) -> &ActivationConfig {
        &self.cfg.activation
    }

    // ---- 内部 ----

    fn apply_command(&mut self, cmd: SimCommand, source: InputSource) {
        match cmd {
            SimCommand::SetBlock { pos, block } => {
                let edit = VoxelEdit::new().set(pos, block);
                match edit.apply(&mut self.voxels) {
                    Ok(dirty) => {
                        // 细则 4/5:只重建已建 ②;有①无② 的 chunk 写块不新建
                        // ②(未启用模式 ②⊆① 恒真,行为不变,§14-9)。
                        for cp in expand_boundary(dirty, &self.voxels) {
                            if self.built_colliders.contains(&cp) {
                                self.rebuild_chunk_collider(cp);
                            }
                        }
                        // 方块写入统一出口(REQ-005 D11):外部命令成功写体素
                        // 后记 `External`;调度器 `take_block_updates` 只发布
                        // owner 的条目(非 owner 副本的条目由调度器丢弃)。
                        self.outbox.block_updates.push(BlockWrite {
                            pos,
                            block,
                            kind: BlockWriteKind::External,
                        });
                    }
                    Err(e) => panic!("SetBlock 越界(fail-stop):{e}"),
                }
            }
            SimCommand::SpawnDynamic {
                entity,
                state,
                desc,
            } => {
                // 调度器自身注入的命令 = 服务端实体(source=None);玩家来源
                // = 外部 spawn(owner 权限用)。
                let owner_source = if source == InputSource::SCHEDULER {
                    None
                } else {
                    Some(source)
                };
                self.spawn_entity(entity, state, desc, owner_source, false);
            }
            SimCommand::RemoveEntity { entity } => {
                // 幂等(REQ-007 任务 03 生产接入后):实体可能已在本拍前的
                // 生命周期段合法终结(凝固/强制落地/虚空)——不存在 = 无操作,
                // 不再 fail-stop(回评请求追认;网络路径本就经
                // validate_external_command 前置挡未知实体)。
                if self.entities.contains_key(&entity) {
                    self.despawn_entity(entity);
                }
            }
            SimCommand::TeleportEntity { entity, state } => {
                // 一切位姿变更 = 删除 + 重建(REQ-001 §5 行 10);actor/source
                // 是政策状态,必须跨重建保留(REQ-006 任务 00)。
                let (_, desc, owner_source, actor) = self.despawn_entity(entity);
                self.spawn_entity(entity, state, desc, owner_source, actor);
            }
            SimCommand::Detonate {
                center,
                power,
                radius,
            } => self.detonate(center, power, radius),
            SimCommand::MarkActor { entity, actor } => {
                let rec = self
                    .entities
                    .get_mut(&entity)
                    .expect("MarkActor 触及不存在实体(fail-stop)");
                rec.actor = actor;
            }
            // SetObserverPosition 永不走本路径(REQ-002 §4.2:调度器级专用
            // 收件箱消费,分片输入前缀里不出现)→ 出现即一致性违规 fail-stop。
            SimCommand::SetObserverPosition { .. } => {
                panic!("SetObserverPosition 不得进入分片输入前缀(调度器专用收件箱,fail-stop)");
            }
            // RemoveObserver 同款(REQ-005 任务书 02:观察者命令永不进分片前缀)
            // → 出现即一致性违规 fail-stop(match 穷举强制)。
            SimCommand::RemoveObserver => {
                panic!("RemoveObserver 不得进入分片输入前缀(观察者收件箱专用,fail-stop)");
            }
            // 区域票命令永不走本路径(REQ-007 任务 02:调度器级票据收件箱,
            // 与观察者命令同语义);出现 = 一致性违规 fail-stop。
            SimCommand::SetAreaTicket { .. } => {
                panic!("SetAreaTicket 不得进入分片输入前缀(票据收件箱专用,fail-stop)");
            }
            SimCommand::RemoveAreaTicket { .. } => {
                panic!("RemoveAreaTicket 不得进入分片输入前缀(票据收件箱专用,fail-stop)");
            }
        }
    }

    fn rebuild_chunk_collider(&mut self, cp: ChunkPos) {
        if let Some(handle) = self.chunk_bodies.remove(&cp) {
            self.phys.remove_body(handle);
        }
        let Some(chunk) = self.voxels.get_chunk(cp) else {
            return;
        };
        let boxes: Vec<BoxCollider> =
            greedy_colliders(chunk, cp, &neighbors_of(&self.voxels, cp), |b| {
                self.registry.is_solid(b)
            });
        if boxes.is_empty() {
            return;
        }
        let parts: Vec<(Pose, SharedShape)> = boxes
            .iter()
            .map(|b| {
                (
                    conv::pose_to_rapier(b.center, types::glam::Quat::IDENTITY),
                    SharedShape::cuboid(b.half.x, b.half.y, b.half.z),
                )
            })
            .collect();
        let (handle, _) = self.phys.insert(
            RigidBodyBuilder::fixed(),
            ColliderBuilder::compound(parts)
                .friction(0.6)
                .restitution(0.0),
        );
        self.chunk_bodies.insert(cp, handle);
        self.outbox.dirty_chunks.insert(cp);
    }

    fn spawn_entity(
        &mut self,
        id: EntityId,
        state: EntityState,
        desc: EntityDesc,
        source: Option<InputSource>,
        actor: bool,
    ) {
        assert!(
            !self.entities.contains_key(&id),
            "实体重复 spawn(fail-stop):{id:?}"
        );
        let h = desc.half_size;
        let (handle, collider) = self.phys.insert(
            RigidBodyBuilder::dynamic()
                .pose(conv::pose_to_rapier(state.translation, state.rotation))
                .linvel(conv::vec3_to_rapier(state.linvel))
                .angvel(conv::vec3_to_rapier(state.angvel)),
            ColliderBuilder::cuboid(h, h, h)
                .density(1.0)
                .friction(0.6)
                .restitution(0.1),
        );
        self.entities.insert(
            id,
            EntityRecord {
                handle,
                collider,
                desc,
                slow_ticks: 0,
                actor,
                source,
                window: VecDeque::new(),
            },
        );
        // 调度器据此 claim 全局 ownership(REQ-006 任务 00;恰好一次)。
        self.outbox.spawned.push(id);
    }

    /// 删除实体(迁移/传送/凝固),返回最后状态、描述与政策状态。
    fn despawn_entity(
        &mut self,
        id: EntityId,
    ) -> (EntityState, EntityDesc, Option<InputSource>, bool) {
        let rec = self.entities.remove(&id).expect("实体存在(fail-stop)");
        let state = self.read_state(&rec);
        self.phys.remove_body(rec.handle);
        // 调度器据此 release 全局 ownership(REQ-006 任务 00)。
        self.outbox.despawned.push(id);
        (state, rec.desc, rec.source, rec.actor)
    }

    fn read_state(&self, rec: &EntityRecord) -> EntityState {
        let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
        EntityState {
            translation: conv::rapier_to_vec3(b.translation()),
            rotation: conv::rapier_to_quat(b.rotation()),
            linvel: conv::rapier_to_vec3(b.linvel()),
            angvel: conv::rapier_to_vec3(b.angvel()),
        }
    }

    fn detonate(&mut self, center: Vec3, power: f32, radius: f32) {
        let ids: Vec<EntityId> = self.entities.keys().copied().collect();
        for id in ids {
            let rec = self.entities.get(&id).expect("存在");
            let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
            // §9.12 事件免疫:冻结体不响应事件(爆炸对其无效)。
            if body.is_kinematic() {
                continue;
            }
            let pos = conv::rapier_to_vec3(body.translation());
            let Some(impulse) = crate::explosion::explosion_impulse(center, power, radius, pos)
            else {
                continue;
            };
            self.phys
                .bodies
                .get_mut(rec.handle)
                .expect("handle 有效")
                .apply_impulse(conv::vec3_to_rapier(impulse), true);
        }
    }

    /// 实体中心是否在本体区内(8×8 chunk,REQ-001 §4.1)。
    fn core_contains(&self, pos: Vec3) -> bool {
        let min = self.cfg.shard.pos.min_chunk().block_min();
        let max_x = min.x + SHARD_CHUNKS * CHUNK_SIZE;
        let max_z = min.z + SHARD_CHUNKS * CHUNK_SIZE;
        pos.x >= min.x as f32
            && pos.x < max_x as f32
            && pos.z >= min.z as f32
            && pos.z < max_z as f32
    }
}

/// 位置 → 所在 chunk(floor 语义,负方向向下取整)。
/// 支撑表查询(承载判定用;G1)。
fn carried_of(supports: &std::collections::BTreeMap<EntityId, bool>, id: &EntityId) -> bool {
    supports.get(id).copied().unwrap_or(false)
}

fn pos_chunk(pos: Vec3) -> ChunkPos {
    ChunkPos::new((pos.x / 16.0).floor() as i32, (pos.z / 16.0).floor() as i32)
}

/// 从 VoxelWorld 取四邻居(collider 重建用)。
fn neighbors_of(world: &VoxelWorld, cp: ChunkPos) -> NeighborChunks<'_> {
    NeighborChunks {
        px: world.get_chunk(ChunkPos::new(cp.x + 1, cp.z)),
        nx: world.get_chunk(ChunkPos::new(cp.x - 1, cp.z)),
        pz: world.get_chunk(ChunkPos::new(cp.x, cp.z + 1)),
        nz: world.get_chunk(ChunkPos::new(cp.x, cp.z - 1)),
    }
}

/// collider 需要重建的 chunk 集合:自身 + 已加载的边界邻居。
fn expand_boundary(
    dirty: impl IntoIterator<Item = ChunkPos>,
    world: &VoxelWorld,
) -> BTreeSet<ChunkPos> {
    let mut out = BTreeSet::new();
    for cp in dirty {
        out.insert(cp);
        for (dx, dz) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let ncp = ChunkPos::new(cp.x + dx, cp.z + dz);
            if world.get_chunk(ncp).is_some() {
                out.insert(ncp);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::{DimensionId, ShardPos, WorldId};

    fn shard_id(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    fn sim(x: i32, z: i32) -> ShardSim {
        ShardSim::new(
            ShardSimConfig::new(shard_id(x, z)),
            Box::new(voxel::SimpleRegistry::builtin()),
        )
    }

    fn entity(n: u32) -> EntityId {
        EntityId::from_parts(1, n)
    }

    fn spawn_input(seq: u64, id: EntityId, pos: Vec3) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: pos,
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            }),
        }
    }

    fn detonate_input(seq: u64, center: Vec3, power: f32, radius: f32) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::Detonate {
                center,
                power,
                radius,
            }),
        }
    }

    fn teleport_input(seq: u64, id: EntityId, pos: Vec3) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::TeleportEntity {
                entity: id,
                state: EntityState {
                    translation: pos,
                    ..EntityState::default()
                },
            }),
        }
    }

    /// 测试清单(sim)1:apply_activation 冻结远处实体后 Detonate:
    /// 冻结体状态位级不变(§9.12 事件免疫),近处实体受冲量。
    #[test]
    fn frozen_body_ignores_detonate() {
        let mut s = sim(0, 0);
        let (near, far) = (entity(1), entity(2)); // 近 (8,3,8)、远 (100,3,100)——爆炸中心取两者正中,几何对称。
        s.apply_prefix(&[
            spawn_input(0, near, Vec3::new(8.0, 3.0, 8.0)),
            spawn_input(1, far, Vec3::new(100.0, 3.0, 100.0)),
            // §9-1(瞬态半边)迁移:被测实体改 actor = true(REQ-007 任务 03;
            // 瞬态不再滞回冻结且足迹缺 ② 进待地形,会吞掉爆炸冲量断言)。
            mark_actor_input(2, near, true),
            mark_actor_input(3, far, true),
        ]);
        let decisions = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert!(
            decisions.entities_to_freeze.contains(&far),
            "远处实体(距 ≈130 > 32)必须被冻结:{decisions:?}"
        );
        assert!(!decisions.entities_to_freeze.contains(&near));
        let far_before = s.entity_state(far).expect("冻结体仍在实体表");

        // 对称爆炸:两者到爆心等距(≈65 < 70)。
        s.apply_prefix(&[detonate_input(2, Vec3::new(54.0, 3.0, 54.0), 30.0, 70.0)]);

        let far_after = s.entity_state(far).expect("冻结体未被移除");
        assert_eq!(far_after, far_before, "冻结体对爆炸免疫:状态位级不变");
        let near_after = s.entity_state(near).expect("近处实体仍在");
        assert!(
            near_after.linvel.length() > 2.0 && near_after.linvel.x < 0.0,
            "近处实体受冲量且方向背离爆心:{:?}",
            near_after.linvel
        );
    }

    /// 测试清单(sim)2:冻结体不凝固、不产生迁移意图(200 拍冒烟)。
    /// 实体放本体区外 (130,·,100)(若不冻结第 1 拍即迁移),速度冻结后清零
    /// (若不跳过凝固,slow 判据恒真 → ≤20 拍即凝固移除);另设无激活对照
    /// 证明场景非退化。
    #[test]
    fn frozen_body_excluded_from_solidify_and_migration() {
        let e = entity(1);
        // 对照:不做激活 → 第 1 拍即迁出(场景确实会触发两条路径)。
        let mut control = sim(0, 0);
        control.apply_prefix(&[spawn_input(0, e, Vec3::new(130.0, 3.0, 100.0))]);
        control.step();
        assert!(
            control.entity_state(e).is_none(),
            "非退化前提:动态体在本体区外第 1 拍即迁移"
        );
        assert_eq!(control.take_outbox().migrations.len(), 1);

        // 冻结路径:apply_activation → 200 拍仍原位原态,outbox 全空。
        // §9-1 迁移:常驻化(瞬态不滞回冻结,任务 03)。
        let mut s = sim(0, 0);
        s.apply_prefix(&[
            spawn_input(0, e, Vec3::new(130.0, 3.0, 100.0)),
            mark_actor_input(1, e, true),
        ]);
        let decisions = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(decisions.entities_to_freeze, vec![e], "远处实体被冻结");
        let far_state = s.entity_state(e).expect("冻结后仍在表");
        for _ in 0..200 {
            s.step();
        }
        assert_eq!(s.entity_state(e), Some(far_state), "冻结体 200 拍位级不变");
        let outbox = s.take_outbox();
        assert!(outbox.migrations.is_empty(), "冻结体不迁移(防御性跳过)");
        assert!(
            outbox
                .block_updates
                .iter()
                .all(|w| w.kind != BlockWriteKind::Emergent),
            "冻结体不凝固(凝固写入零条 Emergent)"
        );
        assert_eq!(s.voxel().chunk_count(), 0, "凝固桥未写入任何体素");
    }

    /// activation_config 返回注入配置(非默认值可辨)。
    #[test]
    fn activation_config_reflects_injection() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.activation.max_awake_dynamic = 7;
        let s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        assert_eq!(s.activation_config().max_awake_dynamic, 7);
        let expected = ActivationConfig {
            max_awake_dynamic: 7,
            ..ActivationConfig::default()
        };
        assert_eq!(s.activation_config(), &expected);
    }

    /// sim 级配置:关闭凝固桥、压低醒预算(其余默认)。
    fn budget_cfg(max_awake: usize) -> ShardSimConfig {
        let mut c = ShardSimConfig::new(shard_id(0, 0));
        c.solidify.enabled = false;
        c.activation.max_awake_dynamic = max_awake;
        c
    }

    /// 测试清单(sim)3(README 决策 6,绑定):命令唤醒(Teleport 重建即醒)
    /// 超预算 → **同一拍立即**按距离淘汰(enforce_budget 语义),不得把超限
    /// 窗口拖到下一拍 plan。场景:预算 1,观察者 (8,3,8);拍 0 生成 e1(距 5)、
    /// e3(距 ≈130,当拍冻结);拍 2 前缀 Teleport(e3 → 距 2)→ 重建即醒,
    /// 醒数 {e1@5, e3@2} = 2 > 1 → 同拍淘汰最远者 e1:step 前 e1 已清速 +
    /// kinematic → 此后逐拍位级悬停;命令唤醒者 e3 照常下落。
    #[test]
    fn command_wake_culls_immediately_same_tick() {
        let mut s = ShardSim::new(budget_cfg(1), Box::new(voxel::SimpleRegistry::builtin()));
        let (e1, e3) = (entity(1), entity(3));
        s.apply_prefix(&[
            spawn_input(0, e1, Vec3::new(13.0, 3.0, 8.0)),
            spawn_input(1, e3, Vec3::new(100.0, 3.0, 100.0)),
            // §9-1/2 迁移:常驻化(瞬态超预算淘汰 → 强制落地、足迹缺 ② →
            // 待地形,均会吞掉"冻结悬停/当拍下落"断言;任务 03)。
            mark_actor_input(2, e1, true),
            mark_actor_input(3, e3, true),
        ]);
        // 拍 0(观察者已设):e3 距 ≈130 > 32 → 冻结;同时注册观察者供
        // 后续命令唤醒路径复用。醒数 {e1} = 1 ≤ 预算 1,不淘汰。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.entities_to_freeze, vec![e3], "非退化:e3 先被滞回冻结");
        s.step(); // 拍 1:无输入,e1 继续下落。

        // 拍 2:命令唤醒 e3(Teleport 重建,距 2 ≤ 24)→ 醒数 2 > 1 →
        // 同拍淘汰最远醒者 e1(距 5)。
        s.apply_prefix(&[teleport_input(2, e3, Vec3::new(10.0, 3.0, 8.0))]);
        s.step();
        let e3_after = s.entity_state(e3).expect("e3 命令唤醒存活");
        assert_eq!(e3_after.translation.x, 10.0, "传送落点位级生效");
        assert!(
            e3_after.translation.y < 3.0,
            "命令唤醒者当拍已受重力下落:y={}",
            e3_after.translation.y
        );
        let frozen1 = s.entity_state(e1).expect("e1 被淘汰,仍在实体表");
        assert_eq!(frozen1.linvel, Vec3::ZERO, "e1 淘汰:速度清零");
        for k in 3..8 {
            s.step();
            assert_eq!(
                s.entity_state(e1),
                Some(frozen1),
                "拍 {k}:e1 在唤醒拍 2 已被淘汰,位级悬停"
            );
        }
        // 后续拍观察者刷新(plan 路径)无残余决策:醒数已回预算内,
        // 淘汰条目在冷却窗内不得被唤醒。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert!(
            d.entities_to_freeze.is_empty() && d.entities_to_wake.is_empty(),
            "同拍淘汰后 plan 无残余决策:{d:?}"
        );
    }

    /// 测试清单(sim)4:命令唤醒不超预算 → 零额外决策(淘汰只发生在超限拍)。
    #[test]
    fn command_wake_within_budget_no_cull() {
        let mut s = ShardSim::new(budget_cfg(2), Box::new(voxel::SimpleRegistry::builtin()));
        let (e1, e2) = (entity(1), entity(2));
        s.apply_prefix(&[
            spawn_input(0, e1, Vec3::new(13.0, 3.0, 8.0)),
            spawn_input(1, e2, Vec3::new(100.0, 3.0, 100.0)),
            // §9-1 迁移(同上):常驻化(瞬态会进待地形,吞掉"正常下落"断言)。
            mark_actor_input(2, e1, true),
            mark_actor_input(3, e2, true),
        ]);
        // 注册观察者(e2 远 → 冻结);醒数 {e1} = 1 ≤ 预算 2。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.entities_to_freeze, vec![e2]);
        s.step();
        // 命令唤醒 e2(重建至近处):醒数 {e1@5, e2@2} = 2 ≤ 预算 2 → 不淘汰。
        s.apply_prefix(&[teleport_input(2, e2, Vec3::new(10.0, 3.0, 8.0))]);
        s.step();
        let s2 = s.entity_state(e2).expect("e2 唤醒存活");
        assert!(
            s2.translation.y < 3.0,
            "e2 命令唤醒后正常下落:y={}",
            s2.translation.y
        );
        assert!(
            s.entity_state(e1).expect("e1 存活").translation.y < 3.0,
            "e1 未被淘汰,正常下落"
        );
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert!(
            d.entities_to_freeze.is_empty(),
            "预算内命令唤醒不得触发淘汰:{d:?}"
        );
    }

    /// 测试清单(sim)5:命令唤醒路径的淘汰条目按「决策即簿记」记冷却 →
    /// 窗口内 plan 不得把它唤醒(§9.4 kinematic 化语义:解冻只走显式命令
    /// 或冷却满两条路)。
    #[test]
    fn command_wake_cull_respects_cooldown() {
        let mut s = ShardSim::new(budget_cfg(1), Box::new(voxel::SimpleRegistry::builtin()));
        let (e1, e2) = (entity(1), entity(2));
        s.apply_prefix(&[
            spawn_input(0, e1, Vec3::new(13.0, 3.0, 8.0)),
            spawn_input(1, e2, Vec3::new(100.0, 3.0, 100.0)),
            // §9-1 迁移(同上):常驻化(瞬态会进待地形,吞掉"正常下落"断言)。
            mark_actor_input(2, e1, true),
            mark_actor_input(3, e2, true),
        ]);
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.entities_to_freeze, vec![e2], "e2 远 → 冻结(观察者已注册)");
        s.step();
        // 拍 2:命令唤醒 e2 → 醒数 {e1, e2} = 2 > 1 → 同拍淘汰 e1(距 5 最远)。
        s.apply_prefix(&[teleport_input(2, e2, Vec3::new(10.0, 3.0, 8.0))]);
        s.step();
        assert_eq!(
            s.entity_state(e1).expect("e1 已淘汰").linvel,
            Vec3::ZERO,
            "e1 淘汰落实"
        );
        // 紧接的观察者刷新:e1 距 5 ≤ 24(唤醒半径)但淘汰簿记刚记 →
        // 冷却窗内 plan 不得唤醒。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert!(
            d.entities_to_wake.is_empty(),
            "淘汰条目入冷却簿记:窗口内 plan 不得唤醒:{d:?}"
        );
    }

    // ---- 任务书 01:P2′ 位移门 + 演员豁免(测试清单) ----

    /// 零重力配置:虚空运动学场景(悬停/匀速漂移不受重力干扰——重力会使
    /// 1 tick 后 |linvel| ≈ 0.16 > 0.1,slow 判据恒假,与被测语义无关)。
    fn zero_g_cfg() -> ShardSimConfig {
        let mut c = ShardSimConfig::new(shard_id(0, 0));
        c.gravity = Vec3::ZERO;
        c
    }

    fn zero_g_sim() -> ShardSim {
        ShardSim::new(zero_g_cfg(), Box::new(voxel::SimpleRegistry::builtin()))
    }

    /// 虚空匀速漂移实体:零重力无碰撞,rapier 默认阻尼 0 → linvel 恒定;
    /// 0.08 m/s < 0.1(P2 slow 判据恒真)。
    fn drift_input(seq: u64, id: EntityId, v: f32) -> StampedInput {
        let mut inp = spawn_input(seq, id, Vec3::new(60.0, 3.0, 60.0));
        if let InputPayload::Command(SimCommand::SpawnDynamic { state, .. }) = &mut inp.payload {
            state.linvel = Vec3::new(v, 0.0, 0.0);
        }
        inp
    }

    fn mark_actor_input(seq: u64, id: EntityId, actor: bool) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::MarkActor { entity: id, actor }),
        }
    }

    /// 禁用实体的引擎睡眠(场景构造;rapier 阈值 <0 = 永不睡眠)。匀速慢漂体
    /// 在 rapier 默认阈值下 30 tick 即被引擎睡眠(判据 `drift·0.5 < 0.05·dt`
    /// ⟺ v < 0.1 m/s,与 P2 线阈值同界),会把"P2 计数 + 位移门"场景混入
    /// 睡眠兜底——此辅助隔离出纯计数路径(demo H:受驱实体永不睡眠)。
    fn disable_engine_sleep(s: &mut ShardSim, id: EntityId) {
        let rec = s.entities.get(&id).expect("实体存在");
        let body = s.phys.bodies.get_mut(rec.handle).expect("handle 有效");
        let act = body.activation_mut();
        act.normalized_linear_threshold = -1.0;
        act.angular_threshold = -1.0;
    }

    /// 测试清单 1 `gate_blocks_sustained_slow_drift`:虚空实体 linvel=0.08 m/s
    /// 漂 600 tick:窗口 20 tick 位移 ≈26.7mm ≫ 3mm,P2 计数 @tick 20 达标但
    /// 位移门挡住 → 永不凝固;实体仍在(禁引擎睡眠以隔离计数路径,见上)。
    #[test]
    fn gate_blocks_sustained_slow_drift() {
        let mut s = zero_g_sim();
        let e = entity(1);
        s.apply_prefix(&[drift_input(0, e, 0.08)]);
        disable_engine_sleep(&mut s, e);
        let mut solidified = 0;
        for _ in 0..600 {
            s.step();
            solidified += s
                .take_outbox()
                .block_updates
                .iter()
                .filter(|w| w.kind == BlockWriteKind::Emergent)
                .count();
        }
        assert_eq!(solidified, 0, "位移门挡住慢漂:600 tick 零凝固");
        assert!(
            s.entity_state(e).is_some(),
            "实体仍在(600×0.08/60=0.8m,本体区内未迁移)"
        );
    }

    /// 测试清单 2 `gate_passes_true_rest`:真静止 20 tick 照常凝固
    /// (窗口位移 ≈0 ≤ 门);snap 正常(体素写入 = outbox 记录格)。
    /// 任务 03 适配:凝固写入迁 `block_updates` 的 `Emergent`(断言字段变,
    /// 测试意图不变——凝固仍被记录)。
    #[test]
    fn gate_passes_true_rest() {
        let mut s = zero_g_sim();
        let e = entity(1);
        s.apply_prefix(&[spawn_input(0, e, Vec3::new(60.5, 3.5, 60.5))]);
        let mut cell = None;
        for _ in 0..100 {
            s.step();
            let out = s.take_outbox();
            if let Some(w) = out
                .block_updates
                .iter()
                .find(|w| w.kind == BlockWriteKind::Emergent)
            {
                assert_eq!(w.pos, BlockPos::new(60, 3, 60), " Emergent 写入格");
                cell = Some(w.pos);
                break;
            }
        }
        let cell = cell.expect("真静止 20 tick 内凝固(门不阻合法)");
        assert_eq!(s.voxel().get_block(cell), types::BlockId(1), "snap 正常");
        assert!(s.entity_state(e).is_none(), "实体已移除");
    }

    /// 测试清单 3 `sleeping_body_freezes_only_after_stop`:linvel=0.03 m/s
    /// 滑行体(demo 场景 C 语义):滑行中不截断——P2 计数 @20 被位移门挡
    /// (20 tick 位移 10mm > 3mm);引擎睡眠(30 tick 累积,@31)后速度冻结,
    /// 窗口随滚动衰减(首端逐步排空带位移样本)→ 凝固只发生在睡眠之后
    /// (t > 31),即兜底不再抢在睡眠/衰减前凝固。任务 03 适配:凝固检测改
    /// 读 `block_updates` 的 `Emergent`(字段迁移,意图不变)。
    #[test]
    fn sleeping_body_freezes_only_after_stop() {
        let mut s = zero_g_sim();
        let e = entity(1);
        s.apply_prefix(&[drift_input(0, e, 0.03)]);
        let mut freeze_tick = None;
        for tick in 1..=600u32 {
            s.step();
            if s.take_outbox()
                .block_updates
                .iter()
                .any(|w| w.kind == BlockWriteKind::Emergent)
            {
                freeze_tick = Some(tick);
                break;
            }
        }
        let t = freeze_tick.expect("睡眠后窗口位移衰减,600 tick 内应凝固");
        // 引擎睡眠最早 @tick 31(time_until_sleep=0.5s=30 tick 累积);
        // 睡着后 translation 冻结,窗口再滚 ~18 tick 首尾距离才衰减到
        // ≤3mm(t=48 实测)→ 凝固必须晚于睡眠(细则 1/决策 1:窗口约束
        // 睡眠兜底路径,衰减后方放行)。
        assert!(
            t > 31,
            "凝固必须发生在引擎睡眠(≥31)之后(窗口先衰减再放行):t={t}"
        );
    }

    /// 测试清单 4 `mark_actor_exempts_all_auto_paths`:MarkActor{true} 后真静止
    /// 600 tick 不凝固(P2 与睡眠兜底两路全豁免);MarkActor{false} 后恢复凝固。
    #[test]
    fn mark_actor_exempts_all_auto_paths() {
        let mut s = zero_g_sim();
        let e = entity(1);
        s.apply_prefix(&[
            spawn_input(0, e, Vec3::new(60.5, 3.5, 60.5)),
            mark_actor_input(1, e, true),
        ]);
        for _ in 0..600 {
            s.step();
        }
        assert_eq!(
            s.take_outbox()
                .block_updates
                .iter()
                .filter(|w| w.kind == BlockWriteKind::Emergent)
                .count(),
            0,
            "演员实体 600 tick 真静止不凝固(完全豁免)"
        );
        assert!(s.entity_state(e).is_some(), "演员实体仍在实体表");

        s.apply_prefix(&[mark_actor_input(2, e, false)]);
        let mut restored = false;
        for _ in 0..100 {
            s.step();
            if s.take_outbox()
                .block_updates
                .iter()
                .any(|w| w.kind == BlockWriteKind::Emergent)
            {
                restored = true;
                break;
            }
        }
        assert!(restored, "取消演员后恢复自动凝固");
    }

    /// 测试清单 5 `mark_actor_unknown_entity_panics`:触及不存在实体 →
    /// panic(fail-stop)。
    #[test]
    #[should_panic]
    fn mark_actor_unknown_entity_panics() {
        let mut s = zero_g_sim();
        s.apply_prefix(&[mark_actor_input(0, entity(9), true)]);
    }

    /// 测试清单 6 `snapshot_keeps_actor_and_window`:窗口满的慢漂实体快照/恢复
    /// 后继续被门挡住(窗口空会使首尾距离=0 → 直通,即失败);演员标记随快照
    /// 走,恢复后 600 tick 真静止仍不凝固;字节幂等(窗口序列化行为一致)。
    #[test]
    fn snapshot_keeps_actor_and_window() {
        let reg = || Box::new(voxel::SimpleRegistry::builtin()) as Box<dyn BlockRegistry>;
        // 窗口:漂 100 tick(窗口满)→ 快照 → 恢复实例继续漂仍被挡。
        // (禁引擎睡眠:窗口语义不与睡眠兜底衰减耦合。)
        let mut a = zero_g_sim();
        let e = entity(1);
        a.apply_prefix(&[drift_input(0, e, 0.08)]);
        disable_engine_sleep(&mut a, e);
        for _ in 0..100 {
            a.step();
        }
        a.take_outbox();
        let bytes = a.snapshot();
        let mut b = ShardSim::restore(&bytes, reg()).expect("自有快照必可解码");
        for _ in 0..100 {
            b.step();
        }
        assert!(
            b.entity_state(e).is_some(),
            "恢复实例的窗口延续:慢漂仍被位移门挡住"
        );

        // 演员标记:演员 → 快照 → 恢复 → 600 tick 真静止仍不凝固。
        let mut c = zero_g_sim();
        let e2 = entity(2);
        c.apply_prefix(&[
            spawn_input(0, e2, Vec3::new(60.5, 3.5, 60.5)),
            mark_actor_input(1, e2, true),
        ]);
        c.step();
        let s1 = c.snapshot();
        let mut d = ShardSim::restore(&s1, reg()).expect("自有快照必可解码");
        for _ in 0..600 {
            d.step();
        }
        assert!(
            d.entity_state(e2).is_some(),
            "演员标记随快照走:恢复后 600 tick 真静止仍不凝固"
        );
        // 字节幂等(既有 idempotence 测试同款,窗口参与序列化)。
        let d2 = ShardSim::restore(&s1, reg()).expect("自有快照必可解码");
        assert_eq!(s1, d2.snapshot(), "restore→snapshot 字节一致(含窗口)");
    }

    /// 测试清单 7 `gate_disabled_when_zero`:displacement_gate_m=0 → 门关闭
    /// (直通):同样的慢漂实体照旧凝固(兼容开关)。任务 03 适配:凝固计数
    /// 读 `block_updates` 的 `Emergent`(字段迁移,意图不变)。
    #[test]
    fn gate_disabled_when_zero() {
        let mut s = zero_g_sim();
        s.cfg.solidify.displacement_gate_m = 0.0;
        let e = entity(1);
        s.apply_prefix(&[drift_input(0, e, 0.08)]);
        let mut solidified = 0;
        for _ in 0..600 {
            s.step();
            solidified += s
                .take_outbox()
                .block_updates
                .iter()
                .filter(|w| w.kind == BlockWriteKind::Emergent)
                .count();
        }
        assert_eq!(solidified, 1, "门关闭:慢漂实体照旧凝固一次(直通)");
        assert!(s.entity_state(e).is_none(), "实体已凝固移除");
    }

    // ---- REQ-006 任务 00:实体生命周期/来源/actor ----

    fn spawn_input_from(source: InputSource, seq: u64, id: EntityId, pos: Vec3) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: pos,
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            }),
        }
    }

    /// 任务 00:spawn/despawn 生命周期记录 + entity_owners 元数据。
    #[test]
    fn lifecycle_outbox_and_entity_owners() {
        let mut s = {
            let mut cfg = zero_g_cfg();
            cfg.solidify.enabled = false;
            ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
        };
        let e = entity(1);
        s.apply_prefix(&[spawn_input_from(
            InputSource(7),
            1,
            e,
            Vec3::new(8.0, 10.0, 8.0),
        )]);
        let out = s.take_outbox();
        assert_eq!(out.spawned, vec![e], "spawn 记 spawned");
        assert!(out.despawned.is_empty());
        assert_eq!(
            s.entity_owners(),
            vec![(e, Some(InputSource(7)), false)],
            "来源/actor 记录"
        );
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource(7),
            sequence: 2,
            payload: InputPayload::Command(SimCommand::RemoveEntity { entity: e }),
        }]);
        let out = s.take_outbox();
        assert_eq!(out.despawned, vec![e], "Remove 记 despawned");
        assert!(out.spawned.is_empty());
        assert!(s.entity_owners().is_empty());
    }

    /// 任务 00:Teleport 保留 actor/source。
    #[test]
    fn teleport_preserves_source_and_actor() {
        let mut s = {
            let mut cfg = zero_g_cfg();
            cfg.solidify.enabled = false;
            ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
        };
        let e = entity(1);
        s.apply_prefix(&[spawn_input_from(
            InputSource(7),
            1,
            e,
            Vec3::new(8.0, 10.0, 8.0),
        )]);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 2,
            payload: InputPayload::Command(SimCommand::MarkActor {
                entity: e,
                actor: true,
            }),
        }]);
        s.apply_prefix(&[teleport_input(3, e, Vec3::new(20.0, 10.0, 8.0))]);
        assert_eq!(
            s.entity_owners(),
            vec![(e, Some(InputSource(7)), true)],
            "Teleport 后 actor/source 保持"
        );
    }

    /// 任务 00:快照保留实体 source/actor,恢复后 ownership 元数据一致。
    #[test]
    fn snapshot_keeps_source_and_actor() {
        let reg = || Box::new(voxel::SimpleRegistry::builtin()) as Box<dyn BlockRegistry>;
        let mut s = {
            let mut cfg = zero_g_cfg();
            cfg.solidify.enabled = false;
            ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
        };
        let e = entity(1);
        s.apply_prefix(&[
            spawn_input_from(InputSource(7), 0, e, Vec3::new(8.0, 10.0, 8.0)),
            mark_actor_input(1, e, true),
        ]);
        let bytes = s.snapshot();
        let restored = ShardSim::restore(&bytes, reg()).expect("自有快照必可解码");
        assert_eq!(
            restored.entity_owners(),
            vec![(e, Some(InputSource(7)), true)],
            "source/actor 随快照往返"
        );
    }

    /// 任务 00:迁移 payload 携带 actor/source,源片记录 despawned。
    #[test]
    fn migration_payload_preserves_actor_source() {
        let mut s = {
            let mut cfg = zero_g_cfg();
            cfg.solidify.enabled = false;
            ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
        };
        let e = entity(1);
        s.apply_prefix(&[spawn_input_from(
            InputSource(7),
            1,
            e,
            Vec3::new(120.0, 10.0, 8.0),
        )]);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 2,
            payload: InputPayload::Command(SimCommand::MarkActor {
                entity: e,
                actor: true,
            }),
        }]);
        // 传送到 core 外;本拍 step 触发迁移。
        s.apply_prefix(&[teleport_input(3, e, Vec3::new(136.0, 10.0, 8.0))]);
        s.step();
        let out = s.take_outbox();
        // Teleport 先 despawn+spawn,step 再迁移同一 ID:本拍有两次
        // despawn 生命周期事件(旧身/迁移身);最终表已无实体。
        assert_eq!(out.despawned, vec![e, e], "Teleport + 迁移各记一次 despawn");
        assert_eq!(out.migrations.len(), 1);
        match out.migrations[0].payload {
            EnvelopePayload::MigrateEntity {
                entity,
                actor,
                source,
                ..
            } => {
                assert_eq!(entity, e);
                assert!(actor, "actor 随迁移 payload");
                assert_eq!(source, Some(InputSource(7)), "source 随迁移 payload");
            }
            other => panic!("期望 MigrateEntity,实得 {other:?}"),
        }
    }

    // ---- 任务书 00(REQ-003):观察输出 API(测试清单) ----

    /// 测试清单 4 `entity_states_lists_all_in_id_order`:3 实体分片 → 全表
    /// 升序、逐条状态与单查一致;空分片 → 空表。
    #[test]
    fn entity_states_lists_all_in_id_order() {
        // 空分片:空表。
        let empty = sim(0, 0);
        assert!(empty.entity_states().is_empty(), "空分片 → 空表");

        // 3 实体(乱序注入);关凝固桥(不关心桥行为)。
        let mut s = {
            let mut cfg = zero_g_cfg();
            cfg.solidify.enabled = false;
            ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
        };
        let ids = [entity(3), entity(1), entity(2)];
        for (seq, id) in ids.iter().enumerate() {
            s.apply_prefix(&[spawn_input(
                seq as u64,
                *id,
                Vec3::new(60.0, 3.0 + seq as f32, 60.0),
            )]);
        }
        let table = s.entity_states();
        let got_ids: Vec<EntityId> = table.iter().map(|(id, _)| *id).collect();
        assert_eq!(
            got_ids,
            vec![entity(1), entity(2), entity(3)],
            "全表 = EntityId 升序(注入乱序无关)"
        );
        for (id, state) in &table {
            assert_eq!(
                Some(state),
                s.entity_state(*id).as_ref(),
                "逐条状态与单查一致:{id:?}"
            );
        }
    }

    // ---- 任务书 req005-consistency/01:block write outbox(REQ-005 D11) ----

    /// 任务书测试(block write outbox):SetBlock 写体素成功 → `take_outbox()`
    /// 的 `block_updates` 含恰一条 `External` 写入(pos/block 逐字段一致);
    /// 凝固桥路径不产生本字段条目(任务 03 迁 Emergent)。
    #[test]
    fn setblock_records_external_block_write_in_outbox() {
        let mut s = sim(0, 0);
        // 装载一块地面(写体素前 chunk 必须在;VoxelEdit 会自动建 chunk,
        // 但本测试要证明的是"已加载格写入也记录",与自动建路径无涉)。
        s.load_chunk(types::ChunkPos::new(0, 0), voxel::Chunk::air());
        let pos = BlockPos::new(3, 5, 4);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource(1),
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SetBlock {
                pos,
                block: types::BlockId(2),
            }),
        }]);
        let outbox = s.take_outbox();
        assert_eq!(
            outbox.block_updates,
            vec![BlockWrite {
                pos,
                block: types::BlockId(2),
                kind: BlockWriteKind::External,
            }],
            "SetBlock 成功 → outbox 恰一条 External 写入"
        );
        assert!(outbox.solidified.is_empty(), "凝固桥路径不在本字段");
        // 体素确已写入。
        assert_eq!(s.voxel().get_block(pos), types::BlockId(2));
    }

    /// 快照携带(REQ-005 D11):含未消费 block_updates 的新格式快照往返一致
    /// (仅自有新格式;旧格式检查点与本字段不兼容,见 Outbox::block_updates
    /// 注释——bincode 非自描述,不存在"旧字节补默认值"路径)。
    #[test]
    fn outbox_block_updates_survive_snapshot() {
        let mut s = sim(0, 0);
        s.load_chunk(types::ChunkPos::new(0, 0), voxel::Chunk::air());
        let pos = BlockPos::new(1, 2, 3);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource(1),
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SetBlock {
                pos,
                block: types::BlockId(1),
            }),
        }]);
        // 不 take_outbox:未消费产出随快照走(任务书 01 细则 1)。
        let bytes = s.snapshot();
        let mut restored = ShardSim::restore(&bytes, Box::new(voxel::SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        assert_eq!(
            restored.take_outbox().block_updates,
            s.take_outbox().block_updates,
            "未消费的 block_updates 随快照往返"
        );
    }

    // ---- 任务书 req005-consistency/03:涌现 block write / 跨片事件信封 ----

    /// 盖章信封输入(Envelope 载荷;apply_at/序列可控)。
    fn envelope_input(seq: u64, source: ShardId, payload: EnvelopePayload) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Envelope(messaging::Envelope::stamp(
                source,
                Intent {
                    destination: shard_id(0, 0),
                    payload,
                },
                Tick(0),
                seq,
            )),
        }
    }

    /// 任务书测试 `blockwrite_envelope_reaches_owner_and_holders`(sim 级,
    /// 副本一致面):BlockWrite 信封进输入前缀 → 体素写入 + collider 重建
    /// (重建 = 实体能停上去,不可直接观测 colliders,用驻留证明);
    /// owner 持久化面由调度器 owner 发布/卸载写库承载(shard/app 级覆盖)。
    #[test]
    fn blockwrite_envelope_writes_voxel_and_rebuilds_collider() {
        let mut s = zero_g_sim();
        // 预装一个顶面在局部 y=64(world y=0)的地形 chunk。
        let mut ground = voxel::Chunk::air();
        for y in 0..64 {
            for x in 0..16 {
                for z in 0..16 {
                    ground.set(x, y, z, types::BlockId(1));
                }
            }
        }
        s.load_chunk(types::ChunkPos::new(0, 0), ground);
        let pos = BlockPos::new(3, 0, 3); // 地面上方一格
        s.apply_prefix(&[envelope_input(
            0,
            shard_id(9, 9),
            EnvelopePayload::BlockWrite {
                pos,
                block: types::BlockId(2),
            },
        )]);
        // 体素已写入(信封直接应用)。
        assert_eq!(s.voxel().get_block(pos), types::BlockId(2), "信封写体素");
        // 防回环:信封应用不产 block_updates、不产事件(任务书细则 3)。
        let outbox = s.take_outbox();
        assert!(
            outbox.block_updates.is_empty(),
            "BlockWrite 信封不推 outbox(防广播回环)"
        );
        assert!(outbox.events.is_empty(), "信封应用不产事件");
        // collider 已重建:新方块真实承托(有重力探针从上落下停在 (3,1,3)
        // 顶面 ≈y1.5;探针也可能 P2′ 凝固落格——凝固同样证明"有真实碰撞
        // 停住",两态皆证据)。
        let probe = entity(1);
        s.apply_prefix(&[spawn_input(1, probe, Vec3::new(3.5, 3.0, 3.5))]);
        let mut y = None;
        let mut solidified = false;
        for _ in 0..200 {
            s.step();
            match s.entity_state(probe) {
                Some(st) => y = Some(st.translation.y),
                None => {
                    solidified = true; // 凝固落格(承托成立)
                    break;
                }
            }
        }
        assert!(
            solidified || y.is_some_and(|y| (y - 1.5).abs() < 0.6),
            "实体停在新写入的方块上(y≈1.5 或凝固,信封写入的 collider 生效):\
             y={y:?} solidified={solidified}"
        );
    }

    /// 任务书测试 `blockwrite_does_not_rebroadcast`(sim 级):凝固定格上
    /// 重复投同一格 BlockWrite(空气→方块→同值再写)——每次应用都零产出
    /// (不产 External/Emergent 写入,不产信封),广播环在应用侧结构性关闭。
    #[test]
    fn blockwrite_apply_never_produces_outbox_writes() {
        let mut s = zero_g_sim();
        s.load_chunk(types::ChunkPos::new(0, 0), voxel::Chunk::air());
        let pos = BlockPos::new(2, 5, 2);
        for seq in 0..3u64 {
            s.apply_prefix(&[envelope_input(
                seq,
                shard_id(9, 9),
                EnvelopePayload::BlockWrite {
                    pos,
                    block: types::BlockId(7),
                },
            )]);
            let outbox = s.take_outbox();
            assert!(
                outbox.block_updates.is_empty() && outbox.events.is_empty(),
                "第 {seq} 次应用:零 block_updates 零 events(无回环)"
            );
        }
        assert_eq!(s.voxel().get_block(pos), types::BlockId(7));
    }

    /// 任务书测试 `emergent_event_envelope_next_tick` 的 sim 级前置:
    /// `EnvelopePayload::Event(Detonate)` 进输入前缀 → 本地 detonate(复用
    /// 同一函数):范围内实体受冲量,冻结体免疫(§9.12)。
    #[test]
    fn event_envelope_detonates_locally_frozen_immune() {
        let mut s = zero_g_sim();
        let (near, far) = (entity(1), entity(2));
        s.apply_prefix(&[
            spawn_input(0, near, Vec3::new(8.0, 3.0, 8.0)),
            spawn_input(1, far, Vec3::new(100.0, 3.0, 100.0)),
            // §9-1 迁移(同上):常驻化,保留冻结/免疫语义锚。
            mark_actor_input(2, near, true),
            mark_actor_input(3, far, true),
        ]);
        let decisions = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(decisions.entities_to_freeze, vec![far], "非退化:far 冻结");
        let far_before = s.entity_state(far).expect("冻结体仍在表");

        s.apply_prefix(&[envelope_input(
            2,
            shard_id(1, 0),
            EnvelopePayload::Event(CrossEvent::Detonate {
                center: Vec3::new(54.0, 3.0, 54.0),
                power: 30.0,
                radius: 70.0,
            }),
        )]);
        let far_after = s.entity_state(far).expect("冻结体未被移除");
        assert_eq!(far_after, far_before, "冻结体对涌现事件免疫(位级不变)");
        let near_after = s.entity_state(near).expect("近处实体仍在");
        assert!(
            near_after.linvel.length() > 2.0,
            "近处实体受冲量(事件信封本地求值):{:?}",
            near_after.linvel
        );
        // 事件信封应用不产任何 outbox 条目(事件是终端语义,非产出)。
        let outbox = s.take_outbox();
        assert!(outbox.block_updates.is_empty() && outbox.events.is_empty());
    }

    /// `Outbox.events` 生产口(任务 03 只留字段):sim 不主动推事件;字段落
    /// outbox/take_outbox 往返(serde 布局连带)。语义面(撞击触发)非本任务。
    #[test]
    fn outbox_events_field_present_and_default_empty() {
        let mut s = sim(0, 0);
        s.outbox.events.push(CrossEvent::Detonate {
            center: Vec3::ZERO,
            power: 1.0,
            radius: 1.0,
        });
        let out = s.take_outbox();
        assert_eq!(out.events.len(), 1, "events 字段随 outbox 出片");
        assert!(s.take_outbox().events.is_empty(), "取走即清");
        // 快照往返(events 非空时随快照走,serde 布局不变量)。
        s.outbox.events.push(CrossEvent::Detonate {
            center: Vec3::ONE,
            power: 2.0,
            radius: 3.0,
        });
        let bytes = s.snapshot();
        let mut restored = ShardSim::restore(&bytes, Box::new(voxel::SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        assert_eq!(
            restored.take_outbox().events,
            s.outbox.events,
            "未消费 events 随快照往返"
        );
    }

    /// 任务书细则 3(fail-stop 面):BlockWrite 信封落到未装载 chunk =
    /// 内部一致性违规 → panic(owner/holder 必然装载着该 chunk)。
    #[test]
    #[should_panic(expected = "未装载 chunk")]
    fn blockwrite_to_unloaded_chunk_panics() {
        let mut s = zero_g_sim();
        s.apply_prefix(&[envelope_input(
            0,
            shard_id(9, 9),
            EnvelopePayload::BlockWrite {
                pos: BlockPos::new(200, 5, 200),
                block: types::BlockId(1),
            },
        )]);
    }
    // ==== REQ-007 任务 01(任务书测试清单 sim 侧)====

    /// 实心平板 chunk(层 64 全石;与既有测试同风格)。
    fn solid_chunk() -> Chunk {
        let mut ch = Chunk::air();
        for x in 0..voxel::CHUNK_X {
            for z in 0..voxel::CHUNK_Z {
                ch.set(x, 64, z, types::BlockId(1));
            }
        }
        ch
    }

    /// 任务书测试 `sim::apply_activation_executes_collider_decisions`(B3:
    /// 决策进、collider 出,当场生效,单点执行无二次派生)。
    /// cfg:② 卸 32 < ① 卸 48。合一载 (0,0)、只装 ① (1,0);锚 (48,·,8):
    /// (0,0) 距 40 → 拆 ②、① 留;(1,0) 距 24 → 建 ②。返回的 decisions 与
    /// sim 状态一致,且拆/建都是**单点**执行(没有"凡 ① 皆 ②"的二次派生:
    /// (0,0) 拆后不会因 ① 仍在被自动重建)。
    #[test]
    fn apply_activation_executes_collider_decisions() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        cfg.activation.collider_unload_radius = 32.0;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        s.load_data_only(ChunkPos::new(1, 0), solid_chunk());
        assert!(s.has_collider(ChunkPos::new(0, 0)));
        assert!(!s.has_collider(ChunkPos::new(1, 0)));

        // D18 预算语义:pending(装载候选的 ②)与单独构建共享 ② 预算 2。
        // 本拍装载候选 (0,±1)(距 16)先占满 pending → (1,0)(距 24)的
        // 单独构建被饿(下拍续取);R2 拆除:(0,0) 距 40 ∈ (32,48] →
        // ② 拆、① 留(单独拆合法路径)。
        let d = s.apply_activation(&[Vec3::new(48.0, 0.0, 8.0)]);
        assert_eq!(d.colliders_to_drop, vec![ChunkPos::new(0, 0)]);
        assert!(
            d.colliders_to_build.is_empty(),
            "② 预算被 pending 占满 → 单独构建饿(下拍续取)"
        );
        assert_eq!(
            d.chunks_to_load,
            vec![ChunkPos::new(2, 0), ChunkPos::new(3, 0)],
            "① 装载候选(距 8 近于 (0,±1);预算 2 截断)"
        );
        assert_eq!(
            d.colliders_pending_build,
            vec![ChunkPos::new(2, 0), ChunkPos::new(3, 0)],
            "装载候选的 ② 同拍决定(距离升序)"
        );
        assert!(!s.has_collider(ChunkPos::new(0, 0)), "拆除当场执行(① 保留)");
        assert!(!s.has_collider(ChunkPos::new(1, 0)), "未单独构建(预算饿)");
        assert!(s.voxel().get_chunk(ChunkPos::new(0, 0)).is_some(), "① 未动");
        // pending 兑现:驱动执行 ① 装载 → apply_chunk_load 同点建 ②。
        s.apply_chunk_load_tiered(ChunkPos::new(2, 0), solid_chunk(), true);
        s.apply_chunk_load_tiered(ChunkPos::new(3, 0), solid_chunk(), true);
        assert!(
            s.has_collider(ChunkPos::new(2, 0)) && s.has_collider(ChunkPos::new(3, 0)),
            "① 到场同点建 ②(D18 情况 1 兑现)"
        );
        // 单点执行:同拍再刷(决策即簿记)→ pending 空(已兑现);
        // (1,0) 单独构建续取(预算恢复);无二次派生重建 (0,0)(R2:>48 才卸)。
        // (① 继续装载 (0,±1) 是合法行为——距离内未载,① 簿记未记;不在此锚。)
        let d2 = s.apply_activation(&[Vec3::new(48.0, 0.0, 8.0)]);
        assert_eq!(
            d2.colliders_pending_build, d2.chunks_to_load,
            "新装载候选照常 pending;已兑现条目不在"
        );
        assert!(!d2.colliders_pending_build.contains(&ChunkPos::new(2, 0)));
        assert!(!d2.colliders_pending_build.contains(&ChunkPos::new(3, 0)));
        // (1,0) 单独构建要等 pending 不占预算的拍(此处 ① 候选继续占预算);
        // 锚定执行链:pending 兑现 → ② 与 ① 同点。
        assert!(!s.has_collider(ChunkPos::new(0, 0)), "无 '凡①皆②' 二次派生");
    }

    /// 测试清单(sim,任务书 02):plan 输入票视图;观察者票 == 旧 observers
    /// 路径(对拍:同坐标票同输出——apply_activation 内部即此转换,此处显式
    /// 构造票走 apply_activation_tickets 对拍)。
    #[test]
    fn apply_activation_reads_ticket_views() {
        let obs = [Vec3::new(8.0, 0.0, 8.0)];
        let mut a = sim(0, 0);
        let d_obs = a.apply_activation(&obs);
        let mut b = sim(0, 0);
        let cfg = b.activation_config();
        let tickets = vec![crate::tickets::TicketView {
            order: 0,
            priority: 0,
            data: crate::tickets::RegionSpec::Radius {
                load_m: cfg.data_load_radius,
                unload_m: cfg.data_unload_radius,
            },
            collider: crate::tickets::RegionSpec::Radius {
                load_m: cfg.collider_load_radius,
                unload_m: cfg.collider_unload_radius,
            },
            wake: crate::tickets::WakeSpec::Radius {
                load_m: cfg.awake_load_radius,
                unload_m: cfg.awake_unload_radius,
            },
            state: crate::tickets::TicketState::Active,
            anchor: obs[0],
        }];
        let d_tix = b.apply_activation_tickets(&tickets, &obs);
        assert_eq!(
            d_obs, d_tix,
            "同坐标观察者票 == 旧 observers 路径(逐字段相等)"
        );
    }

    /// 测试清单(sim,S1 对拍/§14-1):Observer 与 Area 两票同 RegionSpec/
    /// WakeSpec/priority/anchor → 决策逐字段相等(决策层无票种分支的结构
    /// 证据:plan 只见视图)。
    #[test]
    fn observer_and_area_same_spec_same_decision() {
        let anchor = Vec3::new(64.0, 0.0, 64.0);
        let mk = |order: u32| crate::tickets::TicketView {
            order,
            priority: 1,
            data: crate::tickets::RegionSpec::Radius {
                load_m: 32.0,
                unload_m: 48.0,
            },
            collider: crate::tickets::RegionSpec::Radius {
                load_m: 32.0,
                unload_m: 48.0,
            },
            wake: crate::tickets::WakeSpec::Radius {
                load_m: 24.0,
                unload_m: 32.0,
            },
            state: crate::tickets::TicketState::Active,
            anchor,
        };
        let as_observer = [mk(0)];
        let as_area = [mk(0)];
        let mut a = sim(2, 2); // 锚点在分片 (0,0) 区域外?分片 (2,2) core 含 64m 处
        let d1 = a.apply_activation_tickets(
            &as_observer,
            &as_observer.iter().map(|t| t.anchor).collect::<Vec<_>>(),
        );
        let mut b = sim(2, 2);
        let d2 = b.apply_activation_tickets(
            &as_area,
            &as_area.iter().map(|t| t.anchor).collect::<Vec<_>>(),
        );
        assert_eq!(d1, d2, "同规格不同票种 → 决策逐字段相等(S1)");
    }

    /// 测试清单(sim,结构检查):TicketView 无 holder/kind 字段——编译期
    /// 保证;测试断言字段集(反射式字段清单锚定:新增字段必须显式更新本测)。
    #[test]
    fn ticket_view_has_no_kind() {
        // 构造成功即无 holder/kind 字段(若存在同名公共字段,此初始化仍编译;
        // 用字段名清单比对防静默加字段)。
        let v = crate::tickets::TicketView {
            order: 0,
            priority: 0,
            data: crate::tickets::RegionSpec::None,
            collider: crate::tickets::RegionSpec::None,
            wake: crate::tickets::WakeSpec::None,
            state: crate::tickets::TicketState::Active,
            anchor: Vec3::ZERO,
        };
        // 结构性自检:仅允许的字段集(通过解构编译保证)。
        let crate::tickets::TicketView {
            order: _,
            priority: _,
            data: _,
            collider: _,
            wake: _,
            state: _,
            anchor: _,
        } = v;
    }

    /// D18:pending 未兑现(驱动未执行装载)→ 下一次 apply_activation 开头
    /// 作废,同拍重新决策(不残留僵尸条目)。
    #[test]
    fn pending_builds_invalidated_when_driver_skips_load() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        cfg.activation.data_load_radius = 0.0; // 关其他装载候选,聚焦作废语义
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        let d = s.apply_activation(&[Vec3::new(8.0, 0.0, 8.0)]);
        assert_eq!(
            d.chunks_to_load,
            vec![ChunkPos::new(0, 0)],
            "仅锚点所在 chunk"
        );
        assert_eq!(d.colliders_pending_build, vec![ChunkPos::new(0, 0)]);
        // 驱动不执行装载 → 下拍 pending 作废并重新决策(冷却 @0 未过 → 空)。
        let d2 = s.apply_activation(&[Vec3::new(8.0, 0.0, 8.0)]);
        assert!(
            d2.colliders_pending_build.is_empty() && d2.chunks_to_load.is_empty(),
            "未兑现条目作废;① 冷却 @0 挡下重复装载"
        );
    }

    /// R3(D18"② 已建是独立状态"):全空气 chunk 碰撞盒为空(chunk_bodies
    /// 无条目)但 ② 已建(built_colliders 登记)——不再被 plan 反复提为构建
    /// 候选(白耗 ② 预算),I2(03)不会把它当"缺地形",快照 ② 集合含它。
    /// 场景:激活刷新建 (0,0)(全空气)→ has_collider true;同输入第二次
    /// plan 不再产出该 chunk;快照往返保持。
    #[test]
    fn air_chunk_counts_as_built_collider() {
        let mut s = sim(0, 0);
        // 先合一装载全空气 chunk(未启用模式)→ ② 已建(独立集合)。
        s.load_chunk(ChunkPos::new(0, 0), Chunk::air());
        assert!(
            s.voxel().get_chunk(ChunkPos::new(0, 0)).is_some(),
            "① 已装(空气)"
        );
        assert!(
            s.has_collider(ChunkPos::new(0, 0)),
            "全空气 chunk ② 已建(独立状态,与碰撞盒无关)"
        );
        // 启用激活(设观察者),观察者驻留 → 无装卸/拆除决策;该 chunk 不再
        // 成为构建候选(已建集合过滤)。
        let d = s.apply_activation(&[Vec3::new(8.0, 0.0, 8.0)]);
        assert!(
            !d.colliders_to_build.contains(&ChunkPos::new(0, 0))
                && !d.colliders_pending_build.contains(&ChunkPos::new(0, 0)),
            "已建(空气)不再进构建候选"
        );
        // 快照往返:② 集合含全空气 chunk。
        let bytes = s.snapshot();
        let r = crate::ShardSim::restore(&bytes, Box::new(voxel::SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        assert!(
            r.has_collider(ChunkPos::new(0, 0)),
            "快照 ② 集合包含全空气 chunk(独立状态随档)"
        );
    }

    /// 任务书测试 `sim::load_chunk_legacy_mode_unified`(§9-7 锚):未启用
    /// 模式(从未激活刷新)下 `load_chunk` 合一装 ①②,与旧行为一致;
    /// `apply_chunk_load_tiered(enabled=false)` 同款。
    #[test]
    fn load_chunk_legacy_mode_unified() {
        let mut s = sim(0, 0);
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        assert!(s.voxel().get_chunk(ChunkPos::new(0, 0)).is_some(), "① 已装");
        assert!(s.has_collider(ChunkPos::new(0, 0)), "② 同建(合一)");
        // 卸载同步拆 ②(I1;未启用模式的对称行为)。
        s.unload_chunk(ChunkPos::new(0, 0));
        assert!(
            s.voxel().get_chunk(ChunkPos::new(0, 0)).is_none()
                && !s.has_collider(ChunkPos::new(0, 0)),
            "① 卸 ② 同拆"
        );
        // 开关臂:enabled=false 走合一,enabled=true 走只 ①。
        let mut s = sim(0, 0);
        s.apply_chunk_load_tiered(ChunkPos::new(0, 0), solid_chunk(), false);
        assert!(s.has_collider(ChunkPos::new(0, 0)), "false → 合一");
        s.apply_chunk_load_tiered(ChunkPos::new(1, 0), solid_chunk(), true);
        assert!(
            s.voxel().get_chunk(ChunkPos::new(1, 0)).is_some()
                && !s.has_collider(ChunkPos::new(1, 0)),
            "true → 只 ①"
        );
    }
    // ==== REQ-007 任务 03(任务书测试清单,提交 C)====

    /// 任务书测试 `sim::transient_keeps_ticket_while_moving`(T3/S6)与
    /// `transient_not_frozen_when_far`(§9-1 迁移):抛出瞬态体(落点 > 60 m,
    /// 场景以远处区域票启用激活):全程醒着不被滞回冻结。
    #[test]
    fn transient_not_frozen_when_far() {
        let mut s = sim(0, 0);
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let e = entity(9);
        s.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 3.0, 8.0))]);
        // 远处区域票启用(等价:观察者激活生效即本路径;区域票由调度器派生,
        // sim 侧入口 = apply_activation,票视图等价测试已锚)。
        let d = s.apply_activation(&[Vec3::new(300.0, 0.0, 300.0)]);
        assert!(
            !d.entities_to_freeze.contains(&e),
            "瞬态体不被滞回冻结(§9-1):{d:?}"
        );
        s.step();
        let st = s.entity_state(e).expect("存活");
        assert!(st.linvel.y < 0.0, "醒着下落(未冻结)");
    }

    /// 任务书测试 `sim::void_removal_enabled_mode_only`(T7/T24):启用模式
    /// y < 阈值删除;未启用模式(从未激活刷新)不删(§9-7 对照)。
    #[test]
    fn void_removal_enabled_mode_only() {
        // 启用:激活刷新过(observed = Some)。
        let mut s = sim(0, 0);
        let e = entity(1);
        s.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 0.0, 8.0))]);
        let _ = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        s.run_transient_lifecycle();
        assert!(s.entity_state(e).is_some(), "未越线:不移除");
        // 未越线 + 移到深虚空:
        let deep = Vec3::new(8.0, (types::WORLD_Y_MIN - 20) as f32, 8.0);
        s.apply_prefix(&[teleport_input(2, e, deep)]);
        s.step();
        s.run_transient_lifecycle();
        assert!(s.entity_state(e).is_none(), "y < WORLD_Y_MIN − 16 → 移除");
        // 未启用:全新 sim,同场景不删(§9-7;never observed)。
        let mut s2 = sim(0, 0);
        let e2 = entity(1);
        s2.apply_prefix(&[spawn_input(1, e2, deep)]);
        s2.step();
        s2.run_transient_lifecycle();
        assert!(
            s2.entity_state(e2).is_some(),
            "未启用模式(从未激活刷新)无虚空移除(§4.0/§9-7)"
        );
    }

    /// 任务书测试 `sim::over_budget_transient_forced_landing`(T8/§9-2):
    /// 预算超限瞬态 → 强制落地(非冻结)。
    #[test]
    fn over_budget_transient_forced_landing() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        cfg.activation.max_awake_dynamic = 1;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let (a, b) = (entity(1), entity(2));
        s.apply_prefix(&[
            spawn_input(1, a, Vec3::new(8.0, 0.5, 8.0)),
            spawn_input(2, b, Vec3::new(10.0, 0.5, 8.0)),
        ]);
        // 命令唤醒超预算:a 近者保,b 远者强制落地(瞬态)。
        let _ = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        s.apply_prefix(&[teleport_input(3, b, Vec3::new(9.0, 0.5, 8.0))]);
        s.run_transient_lifecycle();
        assert!(s.entity_state(b).is_none(), "超预算瞬态 → 强制落地(消失)");
        assert!(
            s.outbox
                .block_updates
                .iter()
                .any(|w| w.kind == BlockWriteKind::Emergent),
            "落点写块(Emergent)"
        );
        assert!(s.entity_state(a).is_some(), "近者保留");
    }
    // ==== REQ-007 任务 03(G3 返修补齐测试)====

    /// 任务书测试 `sim::awaiting_terrain_enters_and_recovers`(T5):② 预算 0
    /// → 待地形(Kinematic、位置不变、事件免疫);就绪 → 醒、速度位级写回。
    #[test]
    fn awaiting_terrain_enters_and_recovers() {
        // ② 预算 0:chunk 有 ① 也永远缺 ② → 待地形进得去、恢复不了。
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        cfg.activation.collider_budget_per_tick = 0;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_data_only(ChunkPos::new(0, 0), solid_chunk());
        let e = entity(9);
        s.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 3.0, 8.0))]);
        let _ = s.apply_activation(&[Vec3::new(300.0, 0.0, 300.0)]);
        assert!(
            s.locomotion
                .get(&e)
                .is_some_and(|l| matches!(l, LocomotionState::AwaitingTerrain { .. })),
            "足迹缺 ② → 进入待地形"
        );
        let before = s.entity_state(e).expect("在表").translation;
        s.step();
        assert_eq!(
            s.entity_state(e).expect("在表").translation,
            before,
            "待地形:位置不变(Kinematic)"
        );
        s.apply_prefix(&[detonate_input(2, Vec3::new(8.0, 3.0, 8.0), 30.0, 20.0)]);
        s.step();
        assert_eq!(
            s.entity_state(e).expect("在表").linvel,
            Vec3::ZERO,
            "待地形期间事件免疫(爆炸无效)"
        );
        // 恢复路径:待地形(速度已保存)→ ② 齐 → 醒 + 速度位级写回。
        let mut s2 = ShardSim::new(
            ShardSimConfig {
                solidify: SolidifyConfig {
                    enabled: false,
                    ..SolidifyConfig::default()
                },
                ..ShardSimConfig::new(shard_id(0, 0))
            },
            Box::new(voxel::SimpleRegistry::builtin()),
        );
        s2.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        s2.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 3.0, 8.0))]);
        let saved = Vec3::new(3.0, -1.0, 2.0);
        s2.locomotion.insert(
            e,
            LocomotionState::AwaitingTerrain {
                saved_linvel: saved,
                saved_angvel: Vec3::ZERO,
            },
        );
        s2.apply_activation(&[Vec3::new(300.0, 0.0, 300.0)]);
        let st2 = s2.entity_state(e).expect("恢复后仍在表");
        assert_eq!(st2.linvel, saved, "恢复:保存的速度位级写回(§4.6)");
        assert!(
            !s2.locomotion
                .get(&e)
                .is_some_and(|l| matches!(l, LocomotionState::AwaitingTerrain { .. })),
            "恢复后离开待地形"
        );
    }

    /// 任务书测试 `sim::i2_footprint_assertion`(I2/§14-7):钩子制造违反
    /// (醒着 + 缺 ②)→ panic。
    #[test]
    #[should_panic(expected = "I2 违反")]
    fn i2_footprint_assertion() {
        let mut s = sim(0, 0); // 无地形:足迹 (0,0) 缺 ①②
        let e = entity(1);
        s.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 3.0, 8.0))]);
        s.i2_assert_awake_footprint_ready();
    }

    /// 任务书测试 `sim::support_detection_normal_and_entity`(T9):地形/实体
    /// 支撑区分(表只记实体支撑);零重力不判定。
    #[test]
    fn support_detection_normal_and_entity() {
        let mut s = sim(0, 0);
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let e = entity(1);
        s.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 64.5, 8.0))]);
        for _ in 0..3 {
            s.step();
        }
        let supports = s.evaluate_supports();
        assert_eq!(
            supports.get(&e).copied(),
            Some(false),
            "落在地形上:无实体支撑(地形支撑不入表)"
        );
        let mut z = ShardSim::new(
            ShardSimConfig {
                gravity: Vec3::ZERO,
                ..ShardSimConfig::new(shard_id(0, 0))
            },
            Box::new(voxel::SimpleRegistry::builtin()),
        );
        z.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 3.0, 8.0))]);
        assert!(z.evaluate_supports().is_empty(), "零重力不判定支撑(§9-6)");
    }

    /// G2 返修测试:绕 x 轴旋转 90° 的方块压在常驻体上 → 被承载
    /// (世界系法向 `manifold.data.normal` 在旋转下仍指对方向)。
    #[test]
    fn support_normal_survives_rotated_block() {
        let mut s = sim(0, 0);
        let carrier = entity(1);
        s.apply_prefix(&[
            spawn_input(1, carrier, Vec3::new(8.0, 0.5, 8.0)),
            mark_actor_input(2, carrier, true),
        ]);
        for _ in 0..2 {
            s.step();
        }
        let rotated = EntityState {
            translation: Vec3::new(8.0, 1.5, 8.0),
            rotation: types::glam::Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        };
        s.apply_prefix(&[
            spawn_input(3, entity(2), Vec3::new(8.0, 1.5, 8.0)),
            StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 4,
                payload: InputPayload::Command(SimCommand::TeleportEntity {
                    entity: entity(2),
                    state: rotated,
                }),
            },
        ]);
        for _ in 0..4 {
            s.step();
        }
        let supports = s.evaluate_supports();
        assert_eq!(
            supports.get(&entity(2)).copied(),
            Some(true),
            "旋转 90° 压在常驻体上 → 被承载(世界系法向)"
        );
    }

    /// 任务书测试 `sim::solidify_requires_no_entity_support`(T9/§4.7):
    /// 落地瞬态凝固;压在常驻体上的瞬态不凝固。
    #[test]
    fn solidify_requires_no_entity_support() {
        let mut s = sim(0, 0);
        // 关位移门(贴地接触的毫米级抖动不干扰;只看 slow/sleeping 前提)。
        s.cfg.solidify.displacement_gate_m = 0.0;
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let carrier = entity(1);
        let box_e = entity(2);
        s.apply_prefix(&[
            spawn_input(1, carrier, Vec3::new(8.0, 0.5, 8.0)),
            mark_actor_input(2, carrier, true),
            spawn_input(3, box_e, Vec3::new(9.5, 0.5001, 8.0)),
        ]);
        for _ in 0..200 {
            s.step();
            if s.entity_state(box_e).is_none() {
                break;
            }
        }
        assert!(s.entity_state(box_e).is_none(), "落地瞬态正常凝固(≤200 拍)");
        let box2 = entity(3);
        s.apply_prefix(&[spawn_input(4, box2, Vec3::new(8.0, 1.5, 8.0))]);
        for _ in 0..10 {
            s.step();
        }
        // 锁支撑语义:box2 压在常驻体上 → 实体支撑为真(凝固前提不过;
        // 长程不凝固由该前提 + §4.4.2 承载规则共同保证)。
        let supports = s.evaluate_supports();
        assert_eq!(
            supports.get(&box2).copied(),
            Some(true),
            "压在常驻体上 → 实体支撑(§4.7 凝固前提的输入)"
        );
        assert!(
            s.entity_state(box2).is_some(),
            "压在常驻体上的瞬态不凝固(§4.7)"
        );
    }

    /// 任务书测试 `sim::carried_ticket_pause_suspend_resume`(T9,G1):
    /// 承载产出 Pause → 静止冻结 Suspend。
    #[test]
    fn carried_ticket_pause_suspend_resume() {
        let mut s = sim(0, 0);
        s.cfg.solidify.displacement_gate_m = 0.0;
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let carrier = entity(1);
        let cargo = entity(2);
        s.apply_prefix(&[
            spawn_input(1, carrier, Vec3::new(8.0, 0.5, 8.0)),
            mark_actor_input(2, carrier, true),
            spawn_input(3, cargo, Vec3::new(8.0, 1.5, 8.0)),
        ]);
        for _ in 0..3 {
            s.step();
        }
        s.run_transient_lifecycle();
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == cargo && o.op == TicketOp::Pause),
            "被承载 → Pause 产出(G1)"
        );
        // 静止(slow_ticks 达标)∧ 不在唤醒范围 → Suspend + 冻结。
        for _ in 0..25 {
            s.step();
        }
        s.run_transient_lifecycle();
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == cargo && o.op == TicketOp::Suspend),
            "被承载 ∧ 静止 ∧ 不在唤醒范围 → Suspend(G1)"
        );
        assert!(
            s.entity_state(cargo)
                .is_some_and(|st| st.linvel == Vec3::ZERO),
            "Suspend:被承载瞬态按常驻规则冻结"
        );
    }

    /// 任务书测试 `sim::carried_clock_paused_no_forced_landing`(T9):
    /// 被承载瞬态不因本分片路径进入强制落地(时钟 Paused 在调度器侧停走)。
    #[test]
    fn carried_not_forced_landed_by_deadline() {
        let mut s = sim(0, 0);
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let carrier = entity(1);
        let cargo = entity(2);
        s.apply_prefix(&[
            spawn_input(1, carrier, Vec3::new(8.0, 0.5, 8.0)),
            mark_actor_input(2, carrier, true),
            spawn_input(3, cargo, Vec3::new(8.0, 1.5, 8.0)),
        ]);
        for _ in 0..3 {
            s.step();
        }
        s.run_transient_lifecycle();
        assert!(
            !s.forced_pending.contains(&cargo),
            "被承载瞬态不被分片自身路径强制落地(到期指令由调度器按 Paused 时钟停走不投)"
        );
    }

    /// 任务书测试 `sim::ticket_deadline_forced_landing`(T7):到期指令 →
    /// 强制落地(关凝固零重力漂移场景);指令直投不进日志(日志在调度器侧,
    /// 由 shard 测试覆盖)。
    #[test]
    fn ticket_deadline_forced_landing() {
        let mut s = ShardSim::new(
            ShardSimConfig {
                gravity: Vec3::ZERO,
                solidify: SolidifyConfig {
                    enabled: false,
                    ..SolidifyConfig::default()
                },
                ..ShardSimConfig::new(shard_id(0, 0))
            },
            Box::new(voxel::SimpleRegistry::builtin()),
        );
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let e = entity(1);
        s.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 70.0, 8.0))]);
        s.apply_forced_landing_directive(e);
        s.run_transient_lifecycle();
        assert!(s.entity_state(e).is_none(), "到期 → 强制落地");
        assert!(
            s.outbox
                .block_updates
                .iter()
                .any(|w| w.kind == BlockWriteKind::Emergent),
            "落点写块(Emergent)"
        );
        assert!(s.outbox.despawned.contains(&e), "despawned(终结 End 兜底)");
    }

    /// 任务书测试 `sim::awaiting_terrain_blocked`(D15):待地形足迹缺 ② →
    /// blocked 请求非空(plan 排序/免冷却/卸载排除消费该输入)。
    #[test]
    fn blocked_requests_from_awaiting_terrain() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        cfg.activation.collider_budget_per_tick = 0;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_data_only(ChunkPos::new(0, 0), solid_chunk());
        let e = entity(1);
        s.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 3.0, 8.0))]);
        let _ = s.apply_activation(&[Vec3::new(300.0, 0.0, 300.0)]);
        assert!(
            !s.awaiting_terrain_blocked().is_empty(),
            "待地形足迹缺 ② → blocked 请求(D15)"
        );
    }

    /// 任务书测试 `sim::transient_lands_solidifies_ticket_ends`(T3 前半):
    /// 落地凝固产出(Emergent BlockWrite + despawned;票 End 由调度器终结
    /// 判定兜底)。
    #[test]
    fn transient_lands_solidifies() {
        let mut s = sim(0, 0);
        s.cfg.solidify.displacement_gate_m = 0.0;
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let e = entity(1);
        // 贴地出生(位移门 3mm/20 拍能过;大落差会持续滑动不过门)。
        s.apply_prefix(&[spawn_input(1, e, Vec3::new(8.0, 0.5001, 8.0))]);
        for _ in 0..200 {
            s.step();
            if s.entity_state(e).is_none() {
                break;
            }
        }
        assert!(s.entity_state(e).is_none(), "落地凝固(实体消失;≤200 拍)");
        assert!(
            s.outbox
                .block_updates
                .iter()
                .any(|w| w.kind == BlockWriteKind::Emergent),
            "Emergent BlockWrite 产出"
        );
        assert!(s.outbox.despawned.contains(&e));
    }
}
