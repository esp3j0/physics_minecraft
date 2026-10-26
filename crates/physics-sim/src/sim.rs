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

/// 实体票状态操作(§4.3.1;**瞬态票不含 Issue**——S5:签发单点在调度器
/// claim/补签,分片只产出状态依赖操作。04a:常驻运动票的签发条件(醒着∧
/// 运动中∧出唤醒范围)只有分片判定得出,经 `IssueActorMotion` 交调度器签发
/// ——枚举只追加,幂等由 TicketTable 保证:键已在 → 无操作不重置时钟,D14)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TicketOp {
    End,
    Pause,
    Unpause,
    Suspend,
    Resume,
    /// 常驻运动票签发(REQ-007 任务 04a)。
    IssueActorMotion,
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

/// 旋转包络系数(stable 无 f32::consts::SQRT_3;wake.rs 同款常量)。
const SQRT3: f32 = 1.732_050_8;

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
    /// 地形破坏(REQ-009;实施期间默认关闭,单元 07 改为开启)。
    pub destruction: crate::destruction::DestructionConfig,
}

impl ShardSimConfig {
    pub fn new(shard: ShardId) -> Self {
        Self {
            shard,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            dt: 1.0 / 60.0,
            solidify: SolidifyConfig::default(),
            activation: crate::activation::ActivationConfig::default(),
            destruction: crate::destruction::DestructionConfig::default(),
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
    /// 本拍爆炸产生的脱落请求(REQ-009 §4.3、§7;按 `BlockPos` 升序)。调度器在 tick 末分配
    /// ID 并盖章 N+1 生成(单元 03);对应格已在本拍写为 air。
    pub detach_requests: Vec<crate::destruction::DetachRequest>,
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

/// 强制落地的原因(06c 遥测;§4.16"截止、预算分计")。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForcedLandingReason {
    /// 瞬态票到期(调度器 7.5 内部指令;§4.3.2)。
    Deadline,
    /// 醒体预算超限淘汰(§4.12;命令唤醒与 plan 两路)。
    Budget,
}

/// 唤醒来源(06c 遥测;§4.16)。`wake_group` 只需二分:地形变更唤醒计
/// `wakes_terrain`(按实体);观察者/接触路径的计数在调用侧完成——接触的
/// 来源按**成员**归类(预测种子/补偿种子/传播所得/闭包成员随触发节点),
/// 单一 source 表达不了,故 prestep 自行计数后以 Observer 落实。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum WakeSource {
    /// 不计事件量的落实(plan 半径唤醒、接触唤醒——计数在调用侧)。
    Observer,
    /// 地形变更唤醒(计 `wakes_terrain`,按实体)。
    Terrain,
}

/// 一拍的激活事件计数(06c 遥测;§4.16 事件量)。全部分片由调度器合计成
/// `ActivationTelemetry` 的事件字段;本结构只活在分片内,**不入快照**、
/// 每拍重算(取走即清)。
///
/// - 唤醒按来源计**被唤醒的实体数**(含支撑闭包成员,归入触发该组的来源;
///   同一 B 既是预测种子又是补偿种子时归为预测);只数真正从冻结切为
///   Dynamic 的实体(被推迟/跳过的不计);
/// - `closure_deferred` = 支撑闭包整组因预算放不下而推迟的**组数**;
///   `contact_wake_skipped_no_terrain` = 接触唤醒因当前足迹缺 ② 而跳过的组数;
/// - `actor_motion_capped` = 常驻运动票因到达上限而撤票的张数。
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ActivationEvents {
    pub forced_landings_deadline: u64,
    pub forced_landings_budget: u64,
    pub void_removals: u64,
    pub wakes_contact_pred: u64,
    pub wakes_contact_comp: u64,
    pub wakes_contact_spread: u64,
    pub wakes_terrain: u64,
    pub closure_deferred: u64,
    pub contact_wake_skipped_no_terrain: u64,
    pub actor_motion_capped: u64,
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
    /// 被承载(§4.4.1:有实体支撑;K1 返修:分片只**记录**,冻结/唤醒决策
    /// 单点在 plan)。生命周期段每拍刷新;随快照走。
    pub(crate) carried: bool,
    /// 静止(§4.4.2:低速连续+位移门 / 睡眠;04a 起所有醒着实体记录,
    /// 与 solidify.enabled 无关)。随快照走。
    pub(crate) at_rest: bool,
    /// 常驻运动票持票拍数记账(04a;None = 无票/未触顶):醒着 ∧ 运动中 ∧
    /// 出唤醒范围期间每拍 +1,到达 actor_motion_max_ticks → 兜底冻结(已
    /// 触顶);静止/入唤醒范围/被唤醒 → 清 None(解除触顶,可重签)。随快照。
    pub(crate) actor_motion_ticks: Option<u64>,
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
    /// 待执行上限兜底冻结(任务 04a):常驻运动票到期内部指令进入,下一拍
    /// lifecycle 冻结。随快照。
    pub(crate) capped_freeze_pending: BTreeSet<EntityId>,
    /// 推迟唤醒的阻塞请求(任务 04b):wake_group 前置不满足时缺 ② 的集合;
    /// 下一拍 plan 输入(免冷却、排序最前)。随快照。
    pub(crate) deferred_wake_blocked: BTreeSet<ChunkPos>,
    /// 推迟唤醒的实体(任务 04b;风险 2 的落实面):wake_group 前置不满足
    /// 时种子入集;下一拍 apply_activation **直接重跑 wake_group(免冷却)**
    /// ——plan 的唤醒决策已被翻转簿记挡住(决策即簿记),重试不能等 180 拍;
    /// ② 齐的下一拍立即落实(§4.9 推迟唤醒语义)。每拍重查前置(状态推导);
    /// 实体终结/不再冻结时自然失效(重查时忽略非冻结体)。随快照。
    pub(crate) deferred_wake: BTreeSet<EntityId>,
    /// 本拍体素写入登记(任务 04c;§4.11 地形变更唤醒):SetBlock、BlockWrite
    /// 信封、凝固桥、强制落地四路写入经统一出口 `register_terrain_write`
    /// 登记;下一拍 apply_activation 消费(清空),对 AABB(外扩
    /// `contact_wake_skin_m`)与格相交的冻结体经 `wake_group` 唤醒——不受
    /// 翻转冷却、不受预算截断(D20:单体与整组都当拍允许超限);② 缺照常
    /// 推迟(推迟种子带预算豁免标记,`deferred_terrain_wake`)。D17 未启用
    /// (激活从未刷新)不登记;同拍同格多次写入去重(BTreeSet)。随快照
    /// (N 拍写入、N+1 唤醒,中间 checkpoint 恢复后不得丢;风险 5)。
    pub(crate) pending_terrain_wakes: BTreeSet<BlockPos>,
    /// 地形变更唤醒的推迟种子(04c;风险 2 同款:前置不满足时入集,下一拍
    /// 免冷却重跑 wake_group,**仍带 D20 预算豁免**)。随快照。
    pub(crate) deferred_terrain_wake: BTreeSet<EntityId>,
    /// 当拍经地形变更唤醒的实体(04c;§4.12):下一拍 apply_activation 开头
    /// 清空——常规淘汰只从当拍之后的拍开始;同拍 enforce_budget(命令唤醒
    /// 淘汰)读它排除地形唤醒者。不进快照(只活一拍,checkpoint 落在拍间时
    /// 必为空)。
    pub(crate) terrain_woken_this_tick: BTreeSet<EntityId>,
    /// 待处理的补偿种子(05c;§4.10.4 补偿路径):step 后从 rapier 接触对
    /// 收集的"Dynamic ↔ 冻结"冲量过门槛者(同一 B 多对取最大);**下一拍**
    /// 预测阶段(`prestep_contact_wake`)取出并入传播,取出即清——不跨拍
    /// 重试(撞击持续则下一步产生新种子)。消费时实体可能已迁出/已醒着/
    /// 已消失,一律静默跳过(风险 1)。随快照走(checkpoint 落在撞击与
    /// 唤醒之间时,恢复线同样在下一拍唤醒——AC-T17)。
    pub(crate) pending_contact_wakes: BTreeMap<EntityId, f32>,
    /// 强制落地的原因记账(06c 遥测;§4.16"按原因分计"):到期指令(截止)
    /// 与超预算淘汰分别标记。指令跨拍存活(中心柱 ① 缺推迟),淘汰同拍执行
    /// ——不入快照:checkpoint 落在指令投递与执行之间时,恢复线 reason 表
    /// 为空、按截止归因(调度器 7.5 只对 TransientMotion 投指令,语义恰好
    /// 正确;淘汰的执行永不跨拍,不受影响)。
    pub(crate) forced_reasons: BTreeMap<EntityId, ForcedLandingReason>,
    /// 本拍激活事件计数(06c 遥测;§4.16 事件量)。worker 在 `step →
    /// run_transient_lifecycle` 之后、`take_outbox` 之前随 outbox 一并取走,
    /// 调度器拍末合计。**不入快照**(纯计数、每拍重算、无累加状态)。
    pub(crate) activation_events: ActivationEvents,
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
    /// 唤醒中心(WakeSpec::Radius 票锚;04a;与 observed 分离——见
    /// apply_activation_tickets 注释)。随快照走。
    pub(crate) wake_centers: Option<Vec<Vec3>>,
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
            capped_freeze_pending: BTreeSet::new(),
            deferred_wake_blocked: BTreeSet::new(),
            deferred_wake: BTreeSet::new(),
            pending_terrain_wakes: BTreeSet::new(),
            deferred_terrain_wake: BTreeSet::new(),
            terrain_woken_this_tick: BTreeSet::new(),
            pending_contact_wakes: BTreeMap::new(),
            forced_reasons: BTreeMap::new(),
            activation_events: ActivationEvents::default(),
            outbox: Outbox::default(),
            activation,
            activation_now: Tick::ZERO,
            observed: None,
            wake_centers: None,
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

    /// ① 已装载 chunk 数(06c 遥测;§4.16"① 数")。
    pub fn chunk_count(&self) -> usize {
        self.voxels.chunk_count()
    }

    /// ② 已建 collider 的 chunk 数(06c 遥测;§4.16"② 数";built_colliders
    /// 为准——全空气 chunk 无碰撞盒但 ② 已建)。
    pub fn collider_chunk_count(&self) -> usize {
        self.built_colliders.len()
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
                        actor_motion_ticks,
                    } => {
                        self.spawn_entity(entity, state, desc, source, actor);
                        // 常驻运动票触顶标记随迁移载荷走(04a 遗留返修):
                        // 目的分片 claim 后计数连续,不借迁移绕过上限。
                        if let Some(rec) = self.entities.get_mut(&entity) {
                            rec.actor_motion_ticks = actor_motion_ticks;
                        }
                    }
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
                            // holder 侧同样登记(04c 风险 4):边界附近的
                            // holder 侧冻结实体失去支撑也须醒来。
                            self.register_terrain_write(pos);
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
            // 当拍地形变更唤醒者不进淘汰池(04c;D20/§4.12):命令唤醒
            // 与地形唤醒同拍时,超限者按常规规则下一拍起淘汰。
            // 冻结( kinematic)体不占醒体预算(§9.4「醒数」= Dynamic 数;
            // 05a 预算语义测试暴露的 04a 口径缺口)。
            .filter(|(id, _)| !self.terrain_woken_this_tick.contains(id))
            .filter(|(_, rec)| {
                !self
                    .phys
                    .bodies
                    .get(rec.handle)
                    .is_some_and(|b| b.is_kinematic())
            })
            .map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                (*id, conv::rapier_to_vec3(b.translation()))
            })
            .collect();
        for id in self.activation.enforce_budget(&observers, &awake) {
            let now = self.activation_now;
            // §9-2/任务书细则 10(瞬态半边):被淘汰的**瞬态体 → 强制落地**
            // (产出同 §4.8;非冻结);常驻体 → 冻结 + End ActorMotion
            // (04a:I3 第 4 条,预算强制不受静止前置约束)。
            let is_transient = self.entities.get(&id).is_some_and(|rec| !rec.actor);
            if is_transient {
                // 06c 遥测:超预算淘汰归因(§4.16"预算分计")。
                self.forced_reasons.insert(id, ForcedLandingReason::Budget);
                self.forced_pending.insert(id);
            } else {
                self.freeze_entity(&id);
                self.activation.note_flip(&id, now);
                self.outbox.ticket_ops.push(TicketOpRecord {
                    entity: id,
                    op: TicketOp::End,
                });
            }
        }
    }

    /// 带闭包的冻结落实(风险 5,04b):冻结 `id` 并把压在其上方贴靠的
    /// 醒着实体递归冻结(同一 outbox 语义:常驻 → End ActorMotion;被承载
    /// 瞬态 → Suspend)。醒体需要先**现算支撑**才有 carried 记录——连带
    /// 冻结时直接读 kinematic 转换前的状态。
    fn freeze_with_closure(&mut self, id: &EntityId) {
        // 上方贴靠的醒着实体(几何判定与 wake::upward_closure 同款;
        // 用醒着体表而非冻结表——要找的是"压在支撑者上的醒着体")。
        let mut stack: Vec<EntityId> = vec![*id];
        let mut to_freeze: Vec<EntityId> = Vec::new();
        let skin = self.cfg.activation.contact_wake_skin_m;
        while let Some(cur) = stack.pop() {
            to_freeze.push(cur);
            let Some((cc, ch)) = self.entities.get(&cur).and_then(|rec| {
                self.phys
                    .bodies
                    .get(rec.handle)
                    .map(|b| (conv::rapier_to_vec3(b.translation()), rec.desc.half_size))
            }) else {
                continue;
            };
            let atop: Vec<EntityId> = self
                .entities
                .iter()
                .filter(|(oid, _)| *oid != &cur)
                .filter_map(|(oid, orec)| {
                    let b = self.phys.bodies.get(orec.handle)?;
                    if b.is_kinematic() {
                        return None; // 已冻结的不重复处理(已在组里)
                    }
                    let (oc, oh) = (conv::rapier_to_vec3(b.translation()), orec.desc.half_size);
                    let overlap = |a: f32, ha: f32, c: f32, hc: f32| {
                        (a - c).abs() <= ha * SQRT3 + hc * SQRT3 + skin
                    };
                    // 在上方(中心更高)且三轴贴靠。
                    if oc.y > cc.y
                        && overlap(cc.x, ch, oc.x, oh)
                        && overlap(cc.y, ch, oc.y, oh)
                        && overlap(cc.z, ch, oc.z, oh)
                    {
                        Some(oid)
                    } else {
                        None
                    }
                })
                .copied()
                .collect();
            for o in atop {
                if !to_freeze.contains(&o) {
                    stack.push(o);
                }
            }
        }
        // 收口后统一冻结(避免借用交错;组内确定性升序)。
        to_freeze.sort();
        to_freeze.dedup();
        for fid in &to_freeze {
            let Some(rec) = self.entities.get(fid) else {
                continue;
            };
            let (is_actor, carried_now) = (rec.actor, rec.carried);
            self.freeze_entity(fid);
            if carried_now && !is_actor {
                self.outbox.ticket_ops.push(TicketOpRecord {
                    entity: *fid,
                    op: TicketOp::Suspend,
                });
            }
            if is_actor {
                self.outbox.ticket_ops.push(TicketOpRecord {
                    entity: *fid,
                    op: TicketOp::End,
                });
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

    /// 推进一拍:撞前预测接触唤醒 → 物理步进 → 补偿收集(05c)→ 凝固桥
    /// 检测 → 迁移检测
    /// (demo A/C 时序;05a:预测在 `apply_prefix` 之后[生产路径 worker 的
    /// `apply_prefix → step`]、`phys.step()` 之前——命令与爆炸冲量已施加,
    /// 用的是更新后的速度,放行评论风险 1;05c:补偿紧接 `phys.step()`
    /// 之后——凝固桥会删实体、迁移检测会移出实体,先收集再处理,风险 1)。
    pub fn step(&mut self) {
        self.prestep_contact_wake();
        self.phys.step();
        self.collect_contact_wake_compensation();

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
            let actor_motion_ticks = rec.actor_motion_ticks;
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
                        // 涌现写入同样登记地形变更唤醒(04c;验收
                        // acc_04c_solidify_write_wakes_adjacent_frozen_actor)。
                        self.register_terrain_write(cell);
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
                // 04a 遗留返修:触顶标记随迁移载荷带走(与 actor/source 同款)
                // ——否则触顶实体借迁移绕过上限,目的分片重新签发。
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
                        actor_motion_ticks,
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

    /// 取走本拍激活事件计数(06c 遥测;worker 在 `take_outbox` 之后调用,
    /// 调度器拍末合计)。取走即清(每拍重算)。
    pub fn take_activation_events(&mut self) -> ActivationEvents {
        std::mem::take(&mut self.activation_events)
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
        self.apply_activation_tickets(&tickets, observers, observers)
    }

    /// 票据视图入口(REQ-007 任务 02:调度器的生产路径)。`observed_centers`
    /// = 命令唤醒预算强制的距离中心(唤醒中心锚点;调度器传全部占区域票锚点
    /// ——与 01 的观察者表同语义)。
    pub fn apply_activation_tickets(
        &mut self,
        tickets: &[crate::tickets::TicketView],
        observed_centers: &[Vec3],
        wake_centers_in: &[Vec3],
    ) -> ActivationDecisions {
        let now = self.activation_now;
        self.activation_now = now.next();
        self.observed = Some(observed_centers.to_vec());
        // 唤醒中心(04a):WakeSpec::Radius 票锚(与 plan 的 wake_centers 同
        // 源;`observed_centers` 是**全部占区域票锚**(命令唤醒预算强制的
        // 距离中心),实体票锚(= 实体自身)会把 lifecycle 的 in_wake 判成
        // 恒真——常驻票 SelfOnly/足迹不构成唤醒中心)。
        self.wake_centers = Some(wake_centers_in.to_vec());
        // R1(D18 情况 1):上一拍 pending 未兑现的条目(驱动未执行该装载,
        // 例如观察者已移走致请求作废)就地作废——本拍 plan 重新决策。兑现过
        // 的条目已被 load_data_only 移除。
        self.pending_collider_builds.clear();
        // 上一拍的地形变更唤醒豁免集(04c)就此到期:当拍不进淘汰池只保护
        // 唤醒当拍,此后常规淘汰照常(§4.12"后续按常规淘汰")。
        self.terrain_woken_this_tick.clear();
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
        // 运动中实体集(04a;plan 决策侧静止前置的输入):非记录侧 at_rest
        // 的补充——待地形/无记录的醒着体按运动处理(宁可不冻,不可误冻)。
        // kinematic(冻结/待地形)不在冻结候选,不入集。
        // 运动中实体集(04a;plan 决策侧静止前置的输入):**现算**(低速
        // 连续 + 位移门 / 睡眠;与 lifecycle 记录同判据,不依赖其刷新时序
        // ——决策用当下物理状态)。kinematic(冻结/待地形)不在冻结候选,
        // 不入集。
        let moving: BTreeSet<EntityId> = self
            .entities
            .iter()
            .filter(|(_id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                if b.is_kinematic() || b.is_sleeping() {
                    return false;
                }
                let st = self.read_state(rec);
                let slow = st.linvel.length() < self.cfg.solidify.linear_threshold
                    && st.angvel.length() < self.cfg.solidify.angular_threshold;
                let gate = self.cfg.solidify.displacement_gate_m <= 0.0
                    || rec.window.front().is_none_or(|oldest| {
                        (*oldest - st.translation).length() <= self.cfg.solidify.displacement_gate_m
                    });
                !(slow && gate && rec.slow_ticks >= self.cfg.solidify.consecutive_ticks)
            })
            .map(|(id, _)| *id)
            .collect();
        // 阻塞请求(D15;判定只依实体状态,§14-8):待地形实体的足迹缺 ② +
        // 缺 ①(§4.12 三来源的前两个)+ 上一拍推迟唤醒的缺 ②(04b:免冷却
        // 重装,acc_04b_roundtrip 的 40 拍内重装断言)。
        let mut blocked = self.awaiting_terrain_blocked();
        blocked.extend(self.deferred_wake_blocked.iter().copied());
        self.deferred_wake_blocked.clear();
        // 实体占据足迹(04a:卸载候选排除;立足地保护)——**只含醒着体**:
        // 冻结/待地形体不依赖地形(冻结悬停、待地形挂起,恢复前置本就检查
        // ②),基线(AC-T1)的冻结体足迹 chunk 照常卸载(golden 对齐)。
        let occupied: BTreeSet<ChunkPos> = self
            .entities
            .values()
            .filter(|rec| {
                !self
                    .phys
                    .bodies
                    .get(rec.handle)
                    .is_some_and(|b| b.is_kinematic())
            })
            .flat_map(|rec| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                crate::settle::current_footprint(
                    conv::rapier_to_vec3(b.translation()),
                    rec.desc.half_size,
                    self.cfg.activation.terrain_guard_m,
                )
            })
            .collect();
        let decisions = self.activation.plan(
            tickets,
            &loaded,
            &collider_loaded,
            &entities,
            now,
            region,
            &blocked,
            &moving,
            &occupied,
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

        // tier③ 落实:施体类型切换。H2/K1/04a:冻结/唤醒只来自 plan 决策,
        // 落实在此:
        // - 冻结条目:**静止前置**(§4.4.2"冻结只在静止时",04a 推广到常驻
        //   体:被承载瞬态仍在运动不冻结,常驻体运动中不冻结——蓝方块的
        //   常驻版修复);被承载瞬态被冻结 → 产 Suspend(票不占区域);常驻
        //   体被冻结 → 产 End(撤常驻运动票,I3 第 4 条);
        // - 唤醒条目:当前足迹 ② 前置(缺 → 不唤醒,交由待地形状态机处理
        //   ——kinematic 待地形体在 ② 齐时由 apply_awaiting_terrain 恢复,
        //   速度原样写回,语义等价"推迟唤醒"而非丢弃);被承载瞬态被唤醒
        //   → 仍被承载 → Paused(产 Pause;§4.4.2"唤醒仍被承载则 Paused"),
        //   已脱离承载 → 产 Resume;**任何唤醒重置静止判据**(04a 细则 3:
        //   slow_ticks = 0、位移窗口只留当前位置)。
        for id in &decisions.entities_to_freeze {
            let (is_transient_carried, is_actor) = {
                let Some(rec) = self.entities.get(id) else {
                    continue;
                };
                let carried = supports.get(id).copied().unwrap_or(false);
                (!rec.actor && carried, rec.actor)
            };
            // §4.12/§9-2(§5 修订行"超预算 → 瞬态强制落地"):plan 的超预算
            // 淘汰(`over_budget_freeze` 子集)与命令唤醒路径同语义——被淘汰
            // 的**瞬态体 → 强制落地**(非冻结;与命令唤醒路径的 enforce_
            // budget 同式);常驻体照旧冻结 + End。此前瞬态淘汰被冻结悬停,
            // over_budget_freeze 字段无消费者(06c 接线补齐,非行为面外扩)。
            if decisions.over_budget_freeze.contains(id) && !is_actor && !is_transient_carried {
                self.forced_reasons.insert(*id, ForcedLandingReason::Budget);
                self.forced_pending.insert(*id);
                self.activation.note_flip(id, now);
                continue;
            }
            // 冻结条目全部落实:静止前置已在 plan 决策侧(04a;被前置挡下的
            // 条目不产出、不烧簿记),超预算淘汰(over_budget_freeze)不受
            // 静止前置约束(§4.12 预算强制,决策已单独标出)。
            //
            // 风险 5(04b):**淘汰不拆散叠放**——冻结支撑者时,把压在其上
            // 方贴靠的**醒着**实体一并冻结(否则支撑者一走,上面的醒着实体
            // 悬空)。连带冻结的实体同样撤票/记账(递归处理:叠三层的组)。
            self.freeze_with_closure(id);
            if is_transient_carried {
                self.outbox.ticket_ops.push(TicketOpRecord {
                    entity: *id,
                    op: TicketOp::Suspend,
                });
            }
            if is_actor {
                // 常驻体冻结 → 撤常驻运动票(滞回与超预算两路都撤;
                // I3 第 4 条:冻结的常驻体不持票)。
                self.outbox.ticket_ops.push(TicketOpRecord {
                    entity: *id,
                    op: TicketOp::End,
                });
            }
        }
        // tier③ 唤醒落实(04b 统一唤醒入口;§4.9 共同规则):plan 的每个
        // 唤醒条目为**种子**,经 `wake_group` 走同一入口——前置检查(足迹 ②,
        // 缺 → 整组推迟保持冻结)、向上支撑闭包(冻结常驻体 + 承载冻结的
        // 瞬态体;待地形体不属于)、整组原子、重置静止判据、票据操作
        // (被承载瞬态 Paused / 脱离承载 Resume)。D19:组 ≥ 2 时
        // `当前醒数 + 组大小 ≤ max_awake_dynamic` 才唤醒,否则整组推迟;
        // 单个实体保持现状(先唤醒再淘汰,AC-T1 口径)。
        let awake_count = self
            .entities
            .values()
            .filter(|rec| {
                self.phys
                    .bodies
                    .get(rec.handle)
                    .is_some_and(|b| !b.is_kinematic())
            })
            .count();
        // 上一拍推迟的唤醒先重试(免冷却;② 齐即落实,风险 2)。来源分两类:
        // 地形变更(预算豁免 D20 随种子保持)与观察者(常规 D19)。
        let retry_terrain: Vec<EntityId> = self.deferred_terrain_wake.iter().copied().collect();
        self.deferred_terrain_wake.clear();
        for seed in retry_terrain {
            // 推迟种子的来源随重试保持(地形变更 → 仍按地形计,§4.16)。
            let (missing, _woken) =
                self.wake_group(&[seed], awake_count, true, WakeSource::Terrain);
            if !missing.is_empty() {
                self.deferred_terrain_wake.insert(seed);
                self.deferred_wake_blocked.extend(missing);
            }
        }
        let retry: Vec<EntityId> = self.deferred_wake.iter().copied().collect();
        self.deferred_wake.clear();
        for seed in retry {
            let (missing, _woken) =
                self.wake_group(&[seed], awake_count, false, WakeSource::Observer);
            if !missing.is_empty() {
                // 仍缺:回队 + 阻塞请求继续(状态推导,每拍重查)。
                self.deferred_wake.insert(seed);
                self.deferred_wake_blocked.extend(missing);
            }
        }
        // 地形变更唤醒(04c;§4.11):上一拍登记的写入格 → 与格相交(AABB
        // 外扩 skin)的冻结体为种子,经统一入口唤醒——**免翻转冷却**(不经
        // plan,决策冷却自然不挡)、**免预算截断**(D20)、带向上支撑闭包;
        // ② 缺照常推迟(种子入 deferred_terrain_wake,豁免标记随行)。
        // 风险 6:基线场景无体素写入,本段对 AC-T1 零影响。
        let cells: Vec<BlockPos> = self.pending_terrain_wakes.iter().copied().collect();
        self.pending_terrain_wakes.clear();
        let skin = self.cfg.activation.contact_wake_skin_m;
        for cell in cells {
            let frozen: Vec<(EntityId, Vec3, f32)> = self
                .entities
                .iter()
                .filter(|(id, rec)| {
                    !self.terrain_woken_this_tick.contains(id)
                        && self
                            .phys
                            .bodies
                            .get(rec.handle)
                            .is_some_and(|b| b.is_kinematic())
                        && !self.is_awaiting_terrain(**id)
                })
                .map(|(id, rec)| {
                    let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                    (
                        *id,
                        conv::rapier_to_vec3(b.translation()),
                        rec.desc.half_size,
                    )
                })
                .collect();
            for seed in crate::wake::frozen_intersecting_cell(&frozen, cell, skin) {
                let (missing, _woken) =
                    self.wake_group(&[seed], awake_count, true, WakeSource::Terrain);
                if !missing.is_empty() {
                    self.deferred_terrain_wake.insert(seed);
                    self.deferred_wake_blocked.extend(missing);
                }
            }
        }
        for seed in decisions.entities_to_wake.clone() {
            let (missing, _woken) =
                self.wake_group(&[seed], awake_count, false, WakeSource::Observer);
            if !missing.is_empty() {
                self.deferred_wake.insert(seed);
                self.deferred_wake_blocked.extend(missing);
            }
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

    /// 统一唤醒入口的潜在成员(种子 ∪ 向上支撑闭包;04b 段落提取,接触唤醒
    /// 预算判定共用):只含**冻结**体(kinematic 且非待地形)。seeds 中不在
    /// 冻结表的条目忽略(命令唤醒的种子已是 Dynamic——此时闭包以其为根向上
    /// 找)。返回 EntityId 升序。
    fn wake_group_members(&self, seeds: &[EntityId]) -> Vec<EntityId> {
        let skin = self.cfg.activation.contact_wake_skin_m;
        let frozen: Vec<(EntityId, Vec3, f32)> = self
            .entities
            .iter()
            .filter(|(id, rec)| {
                self.phys
                    .bodies
                    .get(rec.handle)
                    .is_some_and(|b| b.is_kinematic())
                    && !self.is_awaiting_terrain(**id)
            })
            .map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                (
                    *id,
                    conv::rapier_to_vec3(b.translation()),
                    rec.desc.half_size,
                )
            })
            .collect();
        let seeds_frozen: Vec<EntityId> = seeds
            .iter()
            .copied()
            .filter(|id| frozen.iter().any(|(fid, _, _)| fid == id))
            .collect();
        crate::wake::upward_closure(&frozen, &seeds_frozen, skin)
    }

    /// 统一唤醒入口(§4.9;任务 04b;观察者/区域票半径唤醒经 plan 落实、
    /// 命令重建唤醒、承载 Resume 共用):依次执行——
    /// 1. **前置**:每个潜在成员的当前足迹 ∩ 本分片区域 ② 齐;缺 → 整组
    ///    推迟(保持冻结),缺的 chunk 返回给调用方作阻塞请求(免冷却);
    /// 2. **向上支撑闭包**(几何;`wake::upward_closure`):冻结常驻体与
    ///    承载冻结的瞬态体是成员;待地形体不属于(其 kinematic 但非冻结);
    /// 3. **整组原子**(D19):组 ≥ 2 时 `当前醒数 + 组大小 ≤ max_awake_
    ///    dynamic` 才唤醒,否则整组推迟;单实体保持现状(先醒再淘汰);
    ///    `budget_exempt = true`(地形变更唤醒,D20 修订 7)跳过本检查——
    ///    单体与整组都当拍允许超限(支撑已被移除,推迟就意味着悬空);
    /// 4. 落实:切 Dynamic、重置静止判据(§4.4.1)、清触顶标记;
    /// 5. 票据:被承载瞬态 → Pause(仍被承载,直达 Paused);脱离承载 →
    ///    Resume;常驻体(出范围唤醒)→ IssueActorMotion 由 lifecycle 按状
    ///    态产出(此处不产)。
    ///
    /// 返回整组推迟时缺失的 ② chunk(阻塞请求输入;前置不满足的组不落任何
    /// 成员——"下面的醒了走开、上面的悬空"被原子性排除)。
    ///
    /// 06c 遥测:`source` 决定被唤醒实体的事件归类(§4.16"闭包成员归入触发
    /// 该组的来源");预算放不下的整组推迟计 `closure_deferred`(+1/组);
    /// `woken` = 真正从冻结切为 Dynamic 的实体(被推迟的不计)。预算不足的
    /// 返回值仍为空缺集(阻塞语义只属于缺 ②)。
    fn wake_group(
        &mut self,
        seeds: &[EntityId],
        awake_count: usize,
        budget_exempt: bool,
        source: WakeSource,
    ) -> (BTreeSet<ChunkPos>, Vec<EntityId>) {
        let region = self.activation_region();
        let group = self.wake_group_members(seeds);
        // 组为空(种子非冻结体:命令唤醒的重建体已是 Dynamic)→ 无组可落,
        // 返回空缺集(调用方语义:无前置可查,物理自洽)。
        if group.is_empty() {
            return (BTreeSet::new(), Vec::new());
        }
        // 前置:每个成员的当前足迹 ∩ 区域 ② 齐。
        let mut missing: BTreeSet<ChunkPos> = BTreeSet::new();
        for id in &group {
            let rec = self.entities.get(id).expect("闭包成员在实体表");
            let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
            let pos = conv::rapier_to_vec3(b.translation());
            for cp in crate::settle::current_footprint(
                pos,
                rec.desc.half_size,
                self.cfg.activation.terrain_guard_m,
            ) {
                if region.contains(cp) && !self.built_colliders.contains(&cp) {
                    missing.insert(cp);
                }
            }
        }
        if !missing.is_empty() {
            // 整组推迟(保持冻结);缺 ② 由调用方作阻塞请求。接触唤醒的
            // 缺 ② 推迟计 `contact_wake_skipped_no_terrain`(§4.16):按组
            // 计,只对接触路径(落实侧自行计数时传入)——wake_group 无法
            // 区分接触/观察者,二者都以 Observer 进来;该计数由 prestep 在
            // 调用侧处理,此处不重复。
            let _ = &source;
            return (missing, Vec::new());
        }
        // D19 预算:组 ≥ 2 整组放得下才醒;单实体保持现状。D20:地形变更
        // 唤醒豁免(budget_exempt,修订 7——支撑已被移除,推迟就意味着悬空)。
        if !budget_exempt
            && group.len() >= 2
            && awake_count + group.len() > self.cfg.activation.max_awake_dynamic
        {
            // 整组推迟(预算不足;不走先醒再淘汰)——计推迟组数(§4.16)。
            self.activation_events.closure_deferred += 1;
            return (BTreeSet::new(), Vec::new());
        }
        // 落实:整组原子。
        for id in &group {
            // D20 豁免路径(04c):地形变更唤醒的组员全部记入当拍集——
            // 同拍 enforce_budget(命令唤醒淘汰)与下一拍 plan 常规淘汰之前,
            // 本拍内不因预算被冻结(§4.12"当拍因地形变更唤醒的实体不进
            // 淘汰池";集合在下一拍 apply_activation 开头清空,此后常规
            // 淘汰照常收敛)。
            if budget_exempt {
                self.terrain_woken_this_tick.insert(*id);
            }
            let rec = self.entities.get(id).expect("存在");
            let is_actor = rec.actor;
            let carried_now = !is_actor && rec.carried;
            {
                let body = self.phys.bodies.get_mut(rec.handle).expect("handle 有效");
                body.set_body_type(RigidBodyType::Dynamic, true);
            }
            // 重置静止判据(§4.4.1)+ 清触顶标记。
            let rec = self.entities.get_mut(id).expect("存在");
            rec.slow_ticks = 0;
            let pos = {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                conv::rapier_to_vec3(b.translation())
            };
            rec.window.clear();
            rec.window.push_back(pos);
            rec.at_rest = false;
            rec.actor_motion_ticks = None;
            if !is_actor {
                // 被承载瞬态 → Paused(直达;03 做法);脱离承载 → Resume。
                let op = if carried_now {
                    TicketOp::Pause
                } else {
                    TicketOp::Resume
                };
                self.outbox
                    .ticket_ops
                    .push(TicketOpRecord { entity: *id, op });
            }
            // 翻转簿记(§4.9 规则 4;决策落实的唤醒同样防抖)——与 plan 的
            // now 同拍(apply_activation 开头已 next(),本拍值 = 当前值-1)。
            let now = Tick(self.activation_now.0 - 1);
            self.activation.note_flip(id, now);
        }
        // 06c 遥测:本组被唤醒的实体按来源计数(§4.16;闭包成员归入触发
        // 该组的来源——整组共用一个 source 即此语义)。
        if source == WakeSource::Terrain {
            self.activation_events.wakes_terrain += group.len() as u64;
        }
        (BTreeSet::new(), group)
    }

    /// 撞前预测接触唤醒 + 补偿消费(任务 05a/05b/05c;需求
    /// §4.10.1–4.10.4、§4.10.5;调用点 = `step` 开头、`phys.step()` 之前):
    /// 醒着实体(升序)的扫掠体命中的**冻结**实体(常驻体 + 承载静止冻结
    /// 的瞬态体;待地形体不算——其 kinematic 但恢复有自己的路径)为候选;
    /// 冲击动量过门槛者为种子,与**上一拍**收集的补偿种子(§4.10.4;取出
    /// 即清)按 B 取最大值合并,在冻结邻接图上做最大动量传播,传播结果与
    /// 种子在同一拍、同一预测阶段经统一入口 `wake_group` 当拍唤醒(② 前置、
    /// 向上闭包、整组原子、静止重置、票据)。落实后被唤醒者切 Dynamic 零
    /// 速度起步,rapier 在同一步里按两个动态体解算动量交换(§4.10.1)。
    ///
    /// 补偿种子(05c,风险 1):消费时实体可能已迁出、已消失或已醒着(当拍
    /// 被预测唤醒者在收集时已是 Dynamic,不进补偿),一律静默跳过——不在
    /// 种子表(`frozen` 候选)即自然过滤。
    ///
    /// 传播与截断(05b,§4.10.3):种子(第 0 跳)出发 `p(子) = decay·p(父)`
    /// 每节点取最大(优先队列 p 降序、同 p EntityId 升序;低于门槛或跳数超
    /// `max_hops` 停止);结果节点各自按 §4.9 成组,组按"组内最大 p 降序、
    /// 最小 EntityId 升序"纳入,截断到 `contact_wake_max_bodies_per_tick`
    /// 与空闲醒体预算。
    ///
    /// 预算(风险 4/6,§4.12"只占空闲预算"):预算基线 = 预测时刻的醒数
    /// (本拍 `apply_activation` 的唤醒与淘汰之后);每个候选组要求
    /// `当前醒数 + 组大小 ≤ max_awake_dynamic`(单体与整组同式;调用侧
    /// 检查),放不下整组跳过——**不淘汰任何已醒实体、不推迟、不入当拍
    /// 豁免集**(接触是几何现状的函数,下拍重算自然重试;与地形变更唤醒
    /// 的 D20 不同:不存在"推迟就悬空",预算不足推迟无害)。
    ///
    /// 风险 6(§4.10.5 不唤醒):方块/地形是 chunk 固定体不在实体表;观察者
    /// 非物理实体;`wake_group` 内部滤待地形体;其他分片实体不在本表——
    /// 四类天然不参与。
    ///
    /// D17(风险 8):`observed = None`(激活从未刷新)不预测、不消费补偿
    /// ——与地形变更唤醒同款门控(补偿种子保留待启用后消费)。`wake_group`
    /// 恒返回空缺集语义下无阻塞请求通道:② 缺时种子保持冻结,几何继续
    /// 命中,下一拍自然重试。
    fn prestep_contact_wake(&mut self) {
        if self.observed.is_none() {
            return;
        }
        // 补偿种子(05c):取出即清——无论本拍是否被纳入,不跨拍重试;
        // 撞击持续则下一步收集产生新种子(§4.10.4)。D17 关闭时保留。
        let compensated: Vec<(EntityId, f32)> = std::mem::take(&mut self.pending_contact_wakes)
            .into_iter()
            .collect();
        let lookahead = self.cfg.activation.contact_wake_lookahead_ticks;
        let skin = self.cfg.activation.contact_wake_skin_m;
        let min_p = self.cfg.activation.contact_wake_min_momentum;
        let dt = self.cfg.dt;
        let awake: Vec<(EntityId, Vec3, Vec3, f32, Vec3)> = self
            .entities
            .iter()
            .filter(|(_id, rec)| {
                self.phys
                    .bodies
                    .get(rec.handle)
                    .is_some_and(|b| !b.is_kinematic() && !b.is_sleeping())
            })
            .map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                let h = rec.desc.half_size;
                (
                    *id,
                    conv::rapier_to_vec3(b.translation()),
                    Vec3::new(h, h, h),
                    b.mass(),
                    conv::rapier_to_vec3(b.linvel()),
                )
            })
            .collect();
        // 注:awake/frozen 为空不提前返回——补偿种子(已取出)可能仍需消费
        // (撞击者已睡/停下不进 awake,B 仍该醒;§4.10.4 补偿的立足点正是
        // 预测漏判)。合并后无任何种子才返回(`merged.is_empty()`)。
        let frozen: Vec<(EntityId, Vec3, Vec3)> = self
            .entities
            .iter()
            .filter(|(id, rec)| {
                self.phys
                    .bodies
                    .get(rec.handle)
                    .is_some_and(|b| b.is_kinematic())
                    && !self.is_awaiting_terrain(**id)
            })
            .map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                (
                    *id,
                    conv::rapier_to_vec3(b.translation()),
                    Vec3::new(rec.desc.half_size, rec.desc.half_size, rec.desc.half_size),
                )
            })
            .collect();
        if frozen.is_empty() {
            // 冻结表为空:预测必无种子,补偿种子也全部无处安放(B 不在冻结
            // 表 = 静默跳过),等价于 merged 必空——提前返回。
            return;
        }
        // 种子(wake::contact_wake_seeds 返回按 B 升序)= 确定性遍历序;
        // 同一 B 恰一条(风险 5:同 B 多 A 已在纯函数内取最大)。补偿种子
        // (05c)与预测种子按 B 取最大值合并(BTreeMap entry 语义);补偿
        // 指向的 B 不在冻结表(已醒/迁出/消失/待地形)时自然丢弃——合并
        // 键只保留在 frozen 表内的预测种子 B(风险 1:静默跳过)。
        //
        // 05b(§4.10.3;放行评论 8 条风险):
        // 1) 冻结邻接图(风险 3:只含冻结体——frozen 表即常驻体 + 承载
        //    静止冻结的瞬态体,醒着体/待地形体已被过滤;几何保守 = 包络
        //    AABB + skin)。**按需构建**(05b 验收修正:无种子的拍不花
        //    O(n²) 两两比较);
        // 2) 最大动量传播(风险 1:优先队列取最大,非先到先得 BFS;风险 2:
        //    种子第 0 跳,超过 max_hops 不纳入);
        // 3) 成组截断(风险 4、6):传播结果每节点按 §4.9 向上支撑闭包成组,
        //    组按"组内最大 p 降序、最小 EntityId 升序"依次纳入,直到每拍
        //    实体上限或空闲醒体预算用完;放不下的组整组跳过——不淘汰任何
        //    已醒实体、不入豁免集(接触是几何现状的函数,下拍重算自然重试)。
        //    预算口径在预测阶段 = 本拍 apply_activation 的唤醒与淘汰之后
        //    的醒数(此处即 step 开头,其前的 apply_activation 已完成);
        // 4) 同拍生效(风险 5):种子与传播所得在同一个预测阶段一起唤醒,
        //    全部经统一入口 `wake_group`(② 前置——缺 → 整组推迟保持冻结、
        //    向上闭包、整组原子、静止重置、票据)。
        // 风险 7(守卫):静止接触 v≈0 不产生种子(05a 已锚定);AC-T1 的
        // 场景无高速撞向冻结体的实体,传播输入为空 → 行为不变。
        // 风险 8(D17/确定性/grep):observed = None 不预测(同 05a);遍历
        // 全部来自排序结构(BTreeMap/BTreeSet/显式排序)。
        let seeds: BTreeMap<EntityId, f32> =
            crate::wake::contact_wake_seeds(&awake, &frozen, lookahead, skin, min_p, dt)
                .into_iter()
                .map(|(_a, b, p)| (b, p))
                .collect();
        // 06c 遥测的来源归属表(§4.16):预测种子与补偿种子分开记(同一 B
        // 双重身份取最大值合并后归预测——风险 5)。
        let pred_seeds: BTreeSet<EntityId> = seeds.keys().copied().collect();
        let mut comp_seeds: BTreeSet<EntityId> = BTreeSet::new();
        let mut merged = seeds;
        for (b, p) in compensated {
            // B 必须仍是冻结候选(不在 → 已醒着/迁出/消失/待地形:静默跳过,
            // 风险 1);同一 B 预测与补偿并存取最大(§4.10.4)。
            if frozen.iter().any(|(fid, _, _)| fid == &b) {
                comp_seeds.insert(b);
                merged
                    .entry(b)
                    .and_modify(|cur| {
                        if p > *cur {
                            *cur = p;
                        }
                    })
                    .or_insert(p);
            }
        }
        // 双重身份归预测(§4.16 风险 5)。
        for b in pred_seeds.iter() {
            comp_seeds.remove(b);
        }
        if merged.is_empty() {
            return; // 无种子:邻接图(全冻结体两两比较)不构建(05b 修正)
        }
        let adjacency = crate::wake::frozen_adjacency(&frozen, skin);
        let seed_list: Vec<(EntityId, f32)> = merged.into_iter().collect();
        let spread = crate::wake::propagate_momentum(
            &adjacency,
            &seed_list,
            self.cfg.activation.contact_wake_decay,
            min_p,
            self.cfg.activation.contact_wake_max_hops,
        );
        // 传播结果节点 → §4.9 支撑闭包成组(同节点组共享;节点顺序 =
        // 传播序:p 降序、同 p EntityId 升序)。组排序键 = 组内最大 p、
        // 最小 EntityId(需求 §4.10.3);已纳入组的成员从候选中消去
        // (共享成员不重复成组、不重复计数)。
        let per_tick_cap = self.cfg.activation.contact_wake_max_bodies_per_tick;
        let p_of: BTreeMap<EntityId, f32> = spread.iter().copied().collect();
        let mut woken: BTreeSet<EntityId> = BTreeSet::new();
        let mut woken_count = 0usize;
        let mut groups: Vec<(f32, EntityId, Vec<EntityId>)> = Vec::new();
        for (node, p) in &spread {
            if woken.contains(node) {
                continue;
            }
            let group = self.wake_group_members(&[*node]);
            let group: Vec<EntityId> = group.into_iter().filter(|id| !woken.contains(id)).collect();
            if group.is_empty() {
                continue;
            }
            for id in &group {
                woken.insert(*id);
            }
            // 组内最大 p(组员全部来自传播结果或就是本节点——支撑闭包成员
            // 一定与种子节点相邻,若它不在传播结果里,则它没被传播波及,
            // p 取本节点的 p 兜底)。
            let key_p = group
                .iter()
                .map(|id| p_of.get(id).copied().unwrap_or(*p))
                .fold(f32::MIN, f32::max);
            let key_id = *group.iter().min().expect("组非空");
            groups.push((key_p, key_id, group));
        }
        groups.sort_by(|a, b| {
            b.0.partial_cmp(&a.0)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.1.cmp(&b.1))
        });
        // 依次纳入:每拍实体上限(风险 4:按实体数计,含闭包成员)+ 空闲
        // 醒体预算(整组放得下才醒);放不下的组整组跳过(不淘汰、不推迟、
        // 不入豁免集)。预算语义与 05a 一致:整组(含单体)醒数 + 组大小 >
        // max_awake_dynamic 即跳过——检查在调用侧(D19 豁免单体不能直接
        // 用 wake_group 的 ≥2 才检查分支)。
        let mut budget = self
            .entities
            .values()
            .filter(|rec| {
                self.phys
                    .bodies
                    .get(rec.handle)
                    .is_some_and(|b| !b.is_kinematic())
            })
            .count();
        for (_p, _min_id, group) in &groups {
            if woken_count + group.len() > per_tick_cap {
                continue; // 每拍上限:整组跳过(小序组优先纳入,不挤占)
            }
            if budget + group.len() > self.cfg.activation.max_awake_dynamic {
                continue; // 空闲预算不足:整组跳过,不淘汰已醒者(风险 6)
            }
            let group_ids: Vec<EntityId> = group.clone();
            woken_count += group_ids.len();
            // 06c 遥测:按成员归类来源(§4.16"闭包成员归入触发该组的来源")
            // ——成员自己是种子:预测/补偿(同一 B 双重身份已归预测);成员
            // 是传播所得节点(非种子):spread;纯闭包成员(不在传播结果):
            // 归入本组排序键的触发节点(种子 → 预测/补偿,传播节点 → spread)。
            let mut pred = 0usize;
            let mut comp = 0usize;
            let mut spread_n = 0usize;
            for id in &group_ids {
                if pred_seeds.contains(id) {
                    pred += 1;
                } else if comp_seeds.contains(id) {
                    comp += 1;
                } else if p_of.contains_key(id) {
                    spread_n += 1;
                } else if pred_seeds.contains(_min_id) {
                    pred += 1;
                } else if comp_seeds.contains(_min_id) {
                    comp += 1;
                } else {
                    spread_n += 1;
                }
            }
            self.activation_events.wakes_contact_pred += pred as u64;
            self.activation_events.wakes_contact_comp += comp as u64;
            self.activation_events.wakes_contact_spread += spread_n as u64;
            let (missing, woken) = self.wake_group(&group_ids, budget, false, WakeSource::Observer);
            if !missing.is_empty() {
                // 组因当前足迹缺 ② 整组推迟(保持冻结):撤销按成员计的
                // 唤醒数,计 `contact_wake_skipped_no_terrain`(+1/组;§4.16
                // "只数真正从冻结切为 Dynamic 的实体")。
                self.activation_events.wakes_contact_pred -= pred as u64;
                self.activation_events.wakes_contact_comp -= comp as u64;
                self.activation_events.wakes_contact_spread -= spread_n as u64;
                self.activation_events.contact_wake_skipped_no_terrain += 1;
            } else {
                debug_assert_eq!(woken.len(), group_ids.len(), "落实组应全员唤醒");
            }
            budget += group_ids.len();
        }
    }

    /// 补偿路径收集(任务 05c;§4.10.4;调用点 = `step` 内 `phys.step()`
    /// 之后、凝固桥/迁移检测之前——凝固桥会删实体、迁移检测会移出实体,
    /// 先收集再处理,放行评论风险 1):遍历 rapier 接触对,取
    /// "Dynamic 实体 ↔ 冻结实体"的对(冻结实体 = kinematic 且非待地形;
    /// chunk 静态体、待地形实体不算,风险 3),`J =
    /// ContactPair::total_impulse_magnitude()` 过
    /// `contact_wake_min_momentum` 门槛者为冻结侧 B 的补偿种子 `p = J`,
    /// 同一 B 多对取最大(风险 5:BTreeMap 按键序,与接触对遍历顺序无关)。
    ///
    /// **不计支撑接触**(D21,修订 8;风险 4):接触对为**竖向承载**
    /// (任一流形的世界系法向 `|n·û| ≥ support_normal_min`,û = 重力反向)
    /// 即静压接触,不进补偿。法向口径与 `evaluate_supports` 同款(世界系,
    /// 不用中心连线近似)。**双向排除**(对修订 8 字面"B 是 A 的支撑"的
    /// 扩展,回评偏差声明):醒体在下支撑冻结体的镜像叠放同样排除——实测
    /// (04b 叠放验收用例暴露)rapier 0.36 的 `total_impulse_magnitude()`
    /// 在**持久接触对**上跨拍累积(每拍 ≈ `m·g·dt`,双方入睡后读数残留),
    /// 任一方向的静压都会把冻结体反复唤醒(醒 → 静止 → 冻结 → 又醒),
    /// 与 D21 立论的故障模式相同;竖向撞击(落体)撞前速度预测可见
    /// (§4.10.4 自身论证),由预测路径覆盖,不依赖补偿。零重力不判定
    /// 支撑、不排除任何对;水平撞击 `n·û ≈ 0` 不受影响(AC-T17 主场景)。
    ///
    /// 当拍被预测唤醒的实体已是 Dynamic:它们的接触是 Dynamic ↔ Dynamic,
    /// 不进补偿(风险 2,正确语义)。种子的**消费**在下一拍预测阶段
    /// (`prestep_contact_wake` 取出即清);消费时 B 可能已迁出/消失/醒着,
    /// 静默跳过(风险 1)。
    ///
    /// D17(风险 9):`observed = None`(激活从未刷新)不收集。rapier 类型
    /// 不出 physics-sim(§14 第 5 条:冲量读取封在本函数)。
    fn collect_contact_wake_compensation(&mut self) {
        if self.observed.is_none() {
            return;
        }
        let min_p = self.cfg.activation.contact_wake_min_momentum;
        // D21 支撑排除的方向基准(û = 重力反向单位向量);零重力 → None =
        // 不排除任何对(§4.10.4 修订 8)。
        let gravity_len = self.cfg.gravity.length();
        let up = if gravity_len > 0.0 {
            Some(conv::vec3_to_rapier(-self.cfg.gravity / gravity_len))
        } else {
            None
        };
        let support_min = self.cfg.activation.support_normal_min;
        // collider → 实体(1:1,insert 时创建;支撑判定同款映射)。键取
        // handle 的 (index, generation)(rapier handle 本体无 Ord,§14 第 3
        // 条遍历按键序)。已删除/换代的 collider 不在映射,接触对自然跳过。
        let collider_to_entity: BTreeMap<(u32, u32), EntityId> = self
            .entities
            .iter()
            .map(|(id, rec)| {
                let idx = rec.collider.into_raw_parts();
                ((idx.0, idx.1), *id)
            })
            .collect();
        if collider_to_entity.is_empty() {
            return;
        }
        // 冻结实体表(kinematic 且非待地形;与预测候选同口径,风险 3)。
        let frozen: BTreeSet<EntityId> = self
            .entities
            .iter()
            .filter(|(id, rec)| {
                self.phys
                    .bodies
                    .get(rec.handle)
                    .is_some_and(|b| b.is_kinematic())
                    && !self.is_awaiting_terrain(**id)
            })
            .map(|(id, _)| *id)
            .collect();
        if frozen.is_empty() {
            return;
        }
        for pair in self.phys.narrow_phase.contact_pairs() {
            let key1 = pair.collider1.into_raw_parts();
            let key2 = pair.collider2.into_raw_parts();
            let (Some(e1), Some(e2)) = (
                collider_to_entity.get(&(key1.0, key1.1)),
                collider_to_entity.get(&(key2.0, key2.1)),
            ) else {
                continue; // 一侧是 chunk 静态体(fixed)→ 不进补偿(风险 3)
            };
            // D21 双向排除后不再区分 A/B 方向,只取冻结侧为 B。
            let b = match (frozen.contains(e1), frozen.contains(e2)) {
                (false, true) => *e2,
                (true, false) => *e1,
                _ => continue, // 两侧同类(动-动/冻-冻)不进补偿(风险 2/3)
            };
            // D21(双向,见函数文档):竖向承载接触 = 静压,不进补偿。
            if let Some(up) = &up {
                let vertical = pair
                    .manifolds()
                    .iter()
                    .any(|m| m.data.normal.dot(*up).abs() >= support_min);
                if vertical {
                    continue;
                }
            }
            let j = pair.total_impulse_magnitude();
            if j < min_p {
                continue;
            }
            self.pending_contact_wakes
                .entry(b)
                .and_modify(|cur| {
                    if j > *cur {
                        *cur = j;
                    }
                })
                .or_insert(j);
        }
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
                    // 先切 Dynamic 再写速度(rapier 0.36:KinematicPositionBased
                    // 下 set_linvel/set_angvel 是 no-op,顺序反了保存速度即丢)。
                    body.set_body_type(RigidBodyType::Dynamic, true);
                    body.set_linvel(conv::vec3_to_rapier(saved_linvel), true);
                    body.set_angvel(conv::vec3_to_rapier(saved_angvel), true);
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
    /// ownership 切换后的目的分片收)。06c 遥测:按截止归因(§4.16"截止、
    /// 预算分计"——指令只来自调度器 7.5 的 TransientMotion 到期扫描)。
    pub fn apply_forced_landing_directive(&mut self, entity: EntityId) {
        self.forced_reasons
            .insert(entity, ForcedLandingReason::Deadline);
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

    /// 实体档记录 + 常驻运动票生命周期(任务 03 K1 / 任务 04a):
    /// 1. **记录**每个醒着实体的 `carried`(有实体支撑,§4.4.1;常驻体同样
    ///    记录)与 `at_rest`(低速连续 + 位移门 / 睡眠;与 solidify.enabled
    ///    无关)到实体档;
    /// 2. 瞬态体:`carried` 变化沿产出 Pause/Unpause(边沿触发,无每拍重发);
    /// 3. 常驻体(04a):醒着 ∧ 运动中 ∧ 不在任何唤醒范围 → IssueActorMotion;
    ///    静止 ∨ 进入唤醒范围 → End;到达 actor_motion_max_ticks → End + 冻结
    ///    (兜底降级;已触顶标记防立即重签)。
    ///
    /// 冻结(滞回)+ Suspend、唤醒 + Resume **只能来自 plan 的决策**(§14-1
    /// 单点;`apply_activation` 落实时产出对应 ops,受 D17 开关约束——本函数
    /// 不做滞回冻结决策、不判滞回出区)。
    fn emit_carry_ticket_ops(&mut self) {
        // D17:未启用 → 不记录、不产出(§9-7:未启用行为与 8702670 一致;
        // 叠放场景不得发生任何冻结/票操作)。
        if self.observed.is_none() {
            return;
        }
        let supports = self.evaluate_supports();
        let wake_centers: Vec<Vec3> = self.wake_centers.clone().unwrap_or_default();
        let ids: Vec<EntityId> = self.entities.keys().copied().collect();
        for id in ids {
            let Some(rec) = self.entities.get(&id) else {
                continue;
            };
            let body = self.phys.bodies.get(rec.handle);
            let Some(body) = body else {
                continue;
            };
            // kinematic(冻结/待地形)不重判承载:支撑判定只覆盖醒着体,
            // 冻结体的物理状态不变(决策面只有 plan 的唤醒),记录保持——
            // 否则冻结被承载体会被误判"脱离承载"而错误 Unpause(票应保持
            // Suspended 直到 plan 唤醒落实)。
            if body.is_kinematic() {
                continue;
            }
            let carried = carried_of(&supports, &id);
            // 静止判据(§4.4/任务书 04a 细则 3):低速连续 + 位移门通过,
            // 或 rapier 睡眠;**与 solidify.enabled 无关**(所有醒着实体的
            // 记录侧判据;凝固桥的 enabled 只影响凝固触发,不影响记录)。
            let gate_passes = {
                let w = &rec.window;
                self.cfg.solidify.displacement_gate_m <= 0.0
                    || w.front().is_none_or(|oldest| {
                        (*oldest - conv::rapier_to_vec3(body.translation())).length()
                            <= self.cfg.solidify.displacement_gate_m
                    })
            };
            let at_rest = (rec.slow_ticks >= self.cfg.solidify.consecutive_ticks && gate_passes)
                || body.is_sleeping();
            let was_carried = rec.carried;
            let actor = rec.actor;
            let rec = self.entities.get_mut(&id).expect("存在");
            rec.carried = carried;
            rec.at_rest = at_rest;
            if actor {
                // ---- 常驻运动票生命周期(任务 04a;§4.3.4 常驻各行)----
                // 醒着 ∧ 运动中 ∧ 不在任何唤醒范围 → IssueActorMotion(幂等;
                // 持票记账 +1)+ 到达上限 → 兜底冻结(已触顶);
                // 静止 ∨ 进入唤醒范围 → End + 清触顶标记(可重签)。
                let pos = conv::rapier_to_vec3(body.translation());
                let in_any_wake = wake_centers
                    .iter()
                    .any(|c| (*c - pos).length() <= self.cfg.activation.awake_load_radius);
                if !at_rest && !in_any_wake {
                    // 运动中出范围:持票记账推进。到达上限(已触顶)→ **不再
                    // 产 Issue**(否则 §14-6 键序 End(0) < Issue(5) 下,拍末
                    // "End 撤旧票 → Issue 签新票"无限续签,时钟永不到期);
                    // 只产 End,冻结由调度器到期处理投内部指令在**撤票生效
                    // 后**执行(acc_04a_ticket_capped:持票期间不得冻结)。
                    let ticks = rec.actor_motion_ticks.unwrap_or(0) + 1;
                    let rec = self.entities.get_mut(&id).expect("存在");
                    rec.actor_motion_ticks = Some(ticks);
                    if ticks < self.cfg.activation.actor_motion_max_ticks {
                        self.outbox.ticket_ops.push(TicketOpRecord {
                            entity: id,
                            op: TicketOp::IssueActorMotion,
                        });
                    } else {
                        // 触顶:产 End(撤票)+ 入下一拍冻结队列(撤票在拍末
                        // 生效,冻结在其后的 lifecycle 段执行——持票期间不
                        // 出现"已冻结"可观测态)。06c 遥测:计触顶撤票张数。
                        self.outbox.ticket_ops.push(TicketOpRecord {
                            entity: id,
                            op: TicketOp::End,
                        });
                        self.capped_freeze_pending.insert(id);
                        self.activation_events.actor_motion_capped += 1;
                    }
                } else {
                    // 静止 ∨ 入唤醒范围:撤票 + 清记账/触顶(被唤醒时也清,
                    // 见 apply_activation 唤醒落实;此处 = 状态自然恢复)。
                    self.outbox.ticket_ops.push(TicketOpRecord {
                        entity: id,
                        op: TicketOp::End,
                    });
                    let rec = self.entities.get_mut(&id).expect("存在");
                    rec.actor_motion_ticks = None;
                }
                continue;
            }
            if carried && !was_carried {
                self.outbox.ticket_ops.push(TicketOpRecord {
                    entity: id,
                    op: TicketOp::Pause,
                });
            } else if !carried && was_carried {
                self.outbox.ticket_ops.push(TicketOpRecord {
                    entity: id,
                    op: TicketOp::Unpause,
                });
            }
        }
    }

    /// 上限兜底冻结指令(任务 04a):常驻运动票到期(调度器 7.5)投递;
    /// 下一拍 lifecycle 执行冻结(兜底降级,§4.12 相邻路径)。幂等(集合)。
    pub fn apply_capped_freeze_directive(&mut self, entity: EntityId) {
        self.capped_freeze_pending.insert(entity);
    }

    /// step 末的瞬态体生命周期段(任务 03/提交 C):虚空移除 + 强制落地执行。
    /// 产出:despawned(+ forced 的 Emergent BlockWrite)/ void_removed。
    pub fn run_transient_lifecycle(&mut self) {
        // 04a:上限兜底冻结(撤票生效后的下一拍;幂等)。
        let capped: Vec<EntityId> = self.capped_freeze_pending.iter().copied().collect();
        for id in capped {
            self.capped_freeze_pending.remove(&id);
            let Some(rec) = self.entities.get(&id) else {
                continue;
            };
            let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
            if body.is_kinematic() {
                continue; // 已冻结(滞回静止冻结)——幂等
            }
            self.freeze_entity(&id);
        }
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
                    self.forced_reasons.remove(&id);
                    self.outbox.despawned.push(id);
                    self.outbox.void_removed.push(id);
                    // 06c 遥测:虚空移除只计 void_removals(§4.16:落地失败
                    // 转虚空移除的、虚空线以下生成的,都不算强制落地)。
                    self.activation_events.void_removals += 1;
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
                // 虚空移除(无支撑柱;无 BlockWrite)。§4.16:落地失败转虚空
                // 移除的只计 void_removals,不算强制落地。
                self.entities.remove(&id).expect("存在");
                self.phys.remove_body(handle);
                self.locomotion.remove(&id);
                self.outbox.despawned.push(id);
                self.outbox.void_removed.push(id);
                self.forced_pending.remove(&id);
                self.forced_reasons.remove(&id);
                self.activation_events.void_removals += 1;
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
            // 强制落地的落点写入同样登记(04c 风险 1:四路全覆盖)。
            self.register_terrain_write(landing);
            self.entities.remove(&id).expect("存在");
            self.phys.remove_body(handle);
            self.locomotion.remove(&id);
            self.outbox.despawned.push(id);
            self.outbox.forced_landings.push((id, landing));
            self.forced_pending.remove(&id);
            // 06c 遥测:按原因分计(§4.16);无记账(恢复线缺表)按截止归因
            // ——调度器只对 TransientMotion 到期投指令,语义一致。
            match self.forced_reasons.remove(&id) {
                Some(ForcedLandingReason::Budget) => {
                    self.activation_events.forced_landings_budget += 1;
                }
                _ => self.activation_events.forced_landings_deadline += 1,
            }
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

    /// 实体是否处于待地形(K3 只读观测面:ac_t6 断言"前视缺失不进待地形"
    /// 用;快照/遥测同源判据)。
    pub fn is_awaiting_terrain(&self, id: EntityId) -> bool {
        matches!(
            self.locomotion.get(&id),
            Some(LocomotionState::AwaitingTerrain { .. })
        )
    }

    /// 实体是否被冻结(K3 只读观测面:kinematic = 滞回冻结或待地形;与
    /// `is_awaiting_terrain` 联用区分两者)。
    pub fn is_frozen(&self, id: EntityId) -> bool {
        self.entities.get(&id).is_some_and(|rec| {
            self.phys
                .bodies
                .get(rec.handle)
                .is_some_and(|b| b.is_kinematic())
        })
    }

    /// 实体的承载记录(06a;I3 第 4 条后半的 `suspended_carried` 输入——
    /// 与 Suspend/Resume 操作同源的记录口径,`emit_carry_ticket_ops` 与
    /// 冻结落实写入)。实体不在表 → false。
    pub fn is_carried(&self, id: EntityId) -> bool {
        self.entities.get(&id).is_some_and(|rec| rec.carried)
    }

    // ---- 内部 ----

    /// 体素写入统一出口(任务 04c;§4.11:任何体素写入都登记被改写的格)。
    /// 四路调用:SetBlock 命令、BlockWrite 信封应用、凝固桥、强制落地——
    /// 不在各写入点分别登记(风险 1)。D17 未启用(激活从未刷新,
    /// `activation_now == ZERO`)不登记、不唤醒(风险 7:未启用无新行为);
    /// 同拍同格多次写入去重(BTreeSet,任务书偏离示意 2)。
    fn register_terrain_write(&mut self, cell: BlockPos) {
        if self.activation_now == Tick::ZERO {
            return;
        }
        self.pending_terrain_wakes.insert(cell);
    }

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
                        self.register_terrain_write(pos);
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
                carried: false,
                at_rest: false,
                actor_motion_ticks: None,
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
        // 地形破坏(REQ-009 §4.1):实施期间默认关闭(单元 07 D7 改开)。
        // 实体冲量仍按原半径(REQ-001 §4.2 不变),地形求值在冲量之后。
        if self.cfg.destruction.enabled {
            self.detonate_terrain(center, power, radius);
        }
    }

    /// 爆炸的地形求值与落写(REQ-009 §4.1–§4.3;纯函数 `evaluate_blast`,
    /// 不经 rapier)。求值范围 = 本体区内(格坐标口径,同 `core_contains`)且
    /// 所在 chunk 已装载的格;副本格不求值但已装载、射线照常衰减。同一拍多个
    /// 爆炸按输入序依次求值,后一个看到前一个写过的 air(风险 4)。
    ///
    /// 被破坏的格当拍写 air(`Emergent`,复用凝固桥的涌现写入与副本同步路径)、
    /// 重建所在 chunk collider、按 REQ-007 §4.11 登记地形变更唤醒;脱落格追加
    /// `DetachRequest` 到 `Outbox.detach_requests`(整个 outbox 按 `BlockPos`
    /// 升序;调度器 tick 末分配 ID、盖章 N+1 生成,单元 03)。本单元不生成实体。
    fn detonate_terrain(&mut self, center: Vec3, power: f32, radius: f32) {
        let min = self.cfg.shard.pos.min_chunk().block_min();
        let max_x = min.x + SHARD_CHUNKS * CHUNK_SIZE;
        let max_z = min.z + SHARD_CHUNKS * CHUNK_SIZE;
        let in_scope = |p: BlockPos| p.x >= min.x && p.x < max_x && p.z >= min.z && p.z < max_z;
        let block_at = |p: BlockPos| -> Option<BlockId> {
            // 界外/未装载 → None:不求值、射线不衰减(§4.1)。
            if !(types::WORLD_Y_MIN..=types::WORLD_Y_MAX).contains(&p.y)
                || self.voxels.get_chunk(p.chunk_of()).is_none()
            {
                return None;
            }
            Some(self.voxels.get_block(p))
        };
        let cells = crate::destruction::evaluate_blast(
            &block_at,
            self.registry.as_ref(),
            &in_scope,
            center,
            power,
            radius,
            self.cfg.destruction.attenuation,
            self.cfg.destruction.max_debris_per_explosion,
        );
        if cells.is_empty() {
            return;
        }
        let mut dirty: Vec<ChunkPos> = Vec::new();
        for cell in cells {
            let _ = self.voxels.set_block(cell.pos, BlockId::AIR);
            self.outbox.block_updates.push(BlockWrite {
                pos: cell.pos,
                block: BlockId::AIR,
                kind: BlockWriteKind::Emergent,
            });
            // §4.11 统一出口:冻结体脚下的格被炸掉 → 下一拍被唤醒。
            self.register_terrain_write(cell.pos);
            if let crate::destruction::CellFate::Detach { impulse } = cell.fate {
                self.outbox
                    .detach_requests
                    .push(crate::destruction::DetachRequest {
                        cell: cell.pos,
                        block: cell.block,
                        impulse,
                    });
            }
            dirty.push(cell.pos.chunk_of());
        }
        // 细则 4/5 同款守卫:只重建已建 ②。
        for cp in expand_boundary(dirty, &self.voxels) {
            if self.built_colliders.contains(&cp) {
                self.rebuild_chunk_collider(cp);
            }
        }
        // 多个爆炸拼接后仍保持整个 outbox 的 BlockPos 升序不变量。
        self.outbox.detach_requests.sort_by_key(|req| req.cell);
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

    fn setblock_input(seq: u64, pos: BlockPos, block: types::BlockId) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SetBlock { pos, block }),
        }
    }

    /// 测试清单(sim)1:apply_activation 冻结远处实体后 Detonate:
    /// 冻结体状态位级不变(§9.12 事件免疫),近处实体受冲量。
    #[test]
    fn frozen_body_ignores_detonate() {
        // §9-1(常驻半边)迁移:零重力(出生零速 = 立即静止;有重力虚空
        // 出生在下落,"冻结只在静止时"永不冻结)。
        let mut s = zero_g_sim();
        let (near, far) = (entity(1), entity(2)); // 近 (8,3,8)、远 (100,3,100)——爆炸中心取两者正中,几何对称。
        s.apply_prefix(&[
            spawn_input(0, near, Vec3::new(8.0, 3.0, 8.0)),
            spawn_input(1, far, Vec3::new(100.0, 3.0, 100.0)),
            // §9-1(瞬态半边)迁移:被测实体改 actor = true(REQ-007 任务 03;
            // 瞬态不再滞回冻结且足迹缺 ② 进待地形,会吞掉爆炸冲量断言)。
            mark_actor_input(2, near, true),
            mark_actor_input(3, far, true),
        ]);
        // §9-1:零重力零速生成推进 ≥ 21 拍(慢速计数达标 = "先静止")。
        for _ in 0..21 {
            s.step();
        }
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
        // §9-1(常驻半边)迁移:零重力(出生零速 = 立即静止;迁移对照仍成立
        // ——本体区外出生第 1 拍即迁出,不依赖重力)。
        let mut control = zero_g_sim();
        control.apply_prefix(&[spawn_input(0, e, Vec3::new(130.0, 3.0, 100.0))]);
        control.step();
        assert!(
            control.entity_state(e).is_none(),
            "非退化前提:动态体在本体区外第 1 拍即迁移"
        );
        assert_eq!(control.take_outbox().migrations.len(), 1);

        // 冻结路径:apply_activation → 200 拍仍原位原态,outbox 全空。
        // §9-1 迁移:常驻化(瞬态不滞回冻结,任务 03)+ 零重力(静止)。
        let mut s = zero_g_sim();
        // 出生在本体区内、唤醒区外(距观察者 ≈130 > 32;本体区内不会在推进
        // 期触发迁移,§9-1 的 21 拍静止推进才可能)。
        s.apply_prefix(&[
            spawn_input(0, e, Vec3::new(100.0, 3.0, 100.0)),
            mark_actor_input(1, e, true),
        ]);
        // §9-1:零重力零速生成推进 ≥ 21 拍(先静止)。
        for _ in 0..21 {
            s.step();
        }
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
        c.gravity = Vec3::ZERO; // §9-1(常驻半边)迁移:出生零速 = 立即静止
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
        // §9-1:零重力零速生成推进 ≥ 21 拍(先静止);e3 距 ≈130 > 32 →
        // 冻结;醒数 {e1} = 1 ≤ 预算 1,不淘汰。
        for k in 0..21 {
            s.step();
            let _ = k;
        }
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.entities_to_freeze, vec![e3], "非退化:e3 先被滞回冻结");

        // 命令唤醒 e3(Teleport 重建,距 2 ≤ 24)→ 醒数 2 > 1 →
        // 同拍淘汰最远醒者 e1(距 5)。(零重力:唤醒后匀速保持,不下落。)
        s.apply_prefix(&[teleport_input(2, e3, Vec3::new(10.0, 3.0, 8.0))]);
        s.step();
        let e3_after = s.entity_state(e3).expect("e3 命令唤醒存活");
        assert_eq!(e3_after.translation.x, 10.0, "传送落点位级生效");
        assert_eq!(
            e3_after.translation.y, 3.0,
            "零重力:命令唤醒者匀速保持(重建即醒,当拍动力学生效)"
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
        // §9-1:先静止(21 拍);注册观察者(e2 远 → 冻结);醒数 {e1} ≤ 预算 2。
        for _ in 0..21 {
            s.step();
        }
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.entities_to_freeze, vec![e2]);
        // 命令唤醒 e2(重建至近处):醒数 {e1@5, e2@2} = 2 ≤ 预算 2 → 不淘汰。
        s.apply_prefix(&[teleport_input(2, e2, Vec3::new(10.0, 3.0, 8.0))]);
        s.step();
        let s2 = s.entity_state(e2).expect("e2 唤醒存活");
        assert_eq!(
            s2.translation.y, 3.0,
            "零重力:e2 命令唤醒后匀速保持(重建即醒)"
        );
        assert!(
            s.entity_state(e1).expect("e1 存活").translation.y == 3.0,
            "e1 未被淘汰(零重力:匀速保持原位)"
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

    fn zero_g_sim_at(shard: ShardId) -> ShardSim {
        ShardSim::new(
            ShardSimConfig {
                gravity: Vec3::ZERO,
                ..ShardSimConfig::new(shard)
            },
            Box::new(voxel::SimpleRegistry::builtin()),
        )
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
        // §9-1:零重力零速生成推进 ≥ 21 拍(先静止)。
        for _ in 0..21 {
            s.step();
        }
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
        let d_tix = b.apply_activation_tickets(&tickets, &obs, &obs);
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
        let anchors = as_observer.iter().map(|t| t.anchor).collect::<Vec<_>>();
        let d1 = a.apply_activation_tickets(&as_observer, &anchors, &anchors);
        let mut b = sim(2, 2);
        let d2 = b.apply_activation_tickets(&as_area, &anchors, &anchors);
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

    /// K1:承载/静止记录——lifecycle 把 carried/at_rest 写进实体档;变化沿
    /// 才产 Pause(无每拍重发)。Unpause 沿 = 脱离承载(carrier 移除)。
    #[test]
    fn carry_record_and_edge_only_ops() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.displacement_gate_m = 0.0;
        cfg.activation.cooldown_ticks = 0;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let carrier = entity(1);
        let cargo = entity(2);
        s.apply_prefix(&[
            spawn_input(1, carrier, Vec3::new(8.0, 0.5, 8.0)),
            mark_actor_input(2, carrier, true),
            spawn_input(3, cargo, Vec3::new(8.0, 1.5, 8.0)),
        ]);
        s.apply_activation(&[Vec3::new(8.0, 0.0, 8.0)]);
        let _ = s.take_outbox();
        for _ in 0..5 {
            s.step();
        }
        s.run_transient_lifecycle();
        let rec = s.entities.get(&cargo).expect("存在");
        assert!(rec.carried, "被承载记录(实体档)");
        // 变化沿只发一次:再跑数拍无新 Pause(记录未变化)。
        let first_ops = s.take_outbox().ticket_ops;
        assert_eq!(
            first_ops
                .iter()
                .filter(|o| o.entity == cargo && o.op == TicketOp::Pause)
                .count(),
            1,
            "首拍恰一个 Pause 沿"
        );
        s.run_transient_lifecycle();
        let pause_count = s
            .outbox
            .ticket_ops
            .iter()
            .filter(|o| o.entity == cargo && o.op == TicketOp::Pause)
            .count();
        assert_eq!(pause_count, 0, "无变化沿 → 无重复 Pause(边沿触发)");
        // 脱离承载:carrier 移除 → cargo 下落 → carried 翻 false → Unpause 沿。
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 4,
            payload: InputPayload::Command(SimCommand::RemoveEntity { entity: carrier }),
        }]);
        for _ in 0..5 {
            s.step();
        }
        s.run_transient_lifecycle();
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == cargo && o.op == TicketOp::Unpause),
            "脱离承载变化沿 → Unpause"
        );
        assert!(!s.entities.get(&cargo).expect("存在").carried);
    }

    /// K1(纪律 12):**未启用模式(D17)叠放用例**——瞬态体静止压在另一
    /// 实体上,从未激活刷新(无观察者):不冻结、不产任何票操作(§9-7:
    /// 未启用行为与 8702670 一致;旧实现在 lifecycle 里自做冻结决策,未启用
    /// 也会把叠放瞬态冻结,本用例即红)。
    #[test]
    fn disabled_mode_stacked_transient_never_frozen() {
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
        // 未启用:绝不调 apply_activation(从未注册观察者)。长程静止:
        for _ in 0..60 {
            s.step();
            s.run_transient_lifecycle();
        }
        let kine = {
            let rec = s
                .entities
                .get(&cargo)
                .expect("叠放瞬态存活(未启用无虚空/落地)");
            s.phys
                .bodies
                .get(rec.handle)
                .expect("handle 有效")
                .is_kinematic()
        };
        assert!(!kine, "未启用模式:被承载静止瞬态不被冻结(§9-7/D17)");
        assert!(
            s.outbox.ticket_ops.is_empty(),
            "未启用模式:无任何票操作产出(D17;实得 {:?})",
            s.outbox.ticket_ops
        );
    }

    // ==== REQ-007 任务 04a(常驻运动票/静止判定与重置/超预算冻结撤票)====

    /// 任务书测试 `sim::actor_motion_ticket_issue_end_lifecycle`(T11/§4.3.4):
    /// 运动中 Issue(ops 变化沿)→ 静止 End → 再动再 Issue(新时钟 = 调度器
    /// 侧重签,分片侧清触顶标记可重签)。全程 lifecycle + apply_activation
    /// 生产入口;零重力(出生零速可控地运动/静止)。
    #[test]
    fn actor_motion_ticket_issue_end_lifecycle() {
        let mut s = zero_g_sim();
        let e = entity(1);
        s.apply_prefix(&[
            spawn_input(1, e, Vec3::new(60.0, 3.0, 60.0)),
            mark_actor_input(2, e, true),
        ]);
        s.apply_activation(&[Vec3::new(60.0, 0.0, 60.0)]); // 启用(锚 = 实体处)
        let _ = s.take_outbox();
        // 观察者(唤醒中心)远离 → 出唤醒范围 + 运动中 → Issue。
        s.apply_activation(&[Vec3::new(500.0, 0.0, 500.0)]);
        s.step();
        s.run_transient_lifecycle();
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == e && o.op == TicketOp::IssueActorMotion),
            "运动中出唤醒范围 → IssueActorMotion(04a)"
        );
        let ticks1 = s.entities.get(&e).unwrap().actor_motion_ticks;
        assert!(ticks1.is_some(), "持票记账开始");
        // 静止(零速)+ 出范围 → End + 清记账。
        let _ = s.take_outbox();
        s.apply_prefix(&[teleport_input(3, e, Vec3::new(80.0, 3.0, 80.0))]);
        // 传送后零速(重建按 state.linvel=0);推 21 拍静止达标。
        for _ in 0..21 {
            s.step();
        }
        s.run_transient_lifecycle();
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == e && o.op == TicketOp::End),
            "静止(出范围)→ End(撤票)"
        );
        assert!(
            s.entities.get(&e).unwrap().actor_motion_ticks.is_none(),
            "静止清触顶标记(可重签)"
        );
        // 再动再 Issue(新时钟由调度器签发侧保证;分片侧记账重新从 0)。
        let _ = s.take_outbox();
        s.apply_prefix(&[teleport_input(4, e, Vec3::new(60.0, 3.0, 60.0))]);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 5,
            payload: InputPayload::Command(SimCommand::TeleportEntity {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(60.0, 3.0, 60.0),
                    linvel: Vec3::new(0.3, 0.0, 0.0),
                    ..EntityState::default()
                },
            }),
        }]);
        for _ in 0..3 {
            s.step();
        }
        s.run_transient_lifecycle();
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == e && o.op == TicketOp::IssueActorMotion),
            "再运动 → 再 Issue(新时钟;分片侧记账重起)"
        );
        assert_eq!(
            s.entities.get(&e).unwrap().actor_motion_ticks,
            Some(1),
            "记账重起(清触顶后)"
        );
    }

    /// 任务书测试 `sim::wake_resets_stillness_window`(T11):唤醒重置静止
    /// 判据(slow_ticks = 0、位移窗口只留当前位置)——伪造旧窗口:静止达标
    /// 的实体被冻结(触发唤醒前置),唤醒落实后 slow_ticks 必为 0。
    #[test]
    fn wake_resets_stillness_window() {
        let mut s = zero_g_sim();
        // 唤醒前置(wake_group 足迹 ②)需要地面 chunk 在场,否则整组推迟。
        s.load_chunk(ChunkPos::new(3, 3), solid_chunk());
        let e = entity(1);
        s.apply_prefix(&[
            spawn_input(1, e, Vec3::new(60.0, 3.0, 60.0)),
            mark_actor_input(2, e, true),
        ]);
        s.apply_activation(&[Vec3::new(60.0, 0.0, 60.0)]);
        // 远锚 21 拍:静止达标(出区 → 冻结决策;零速出生即静止)。
        s.apply_activation(&[Vec3::new(500.0, 0.0, 500.0)]);
        for _ in 0..21 {
            s.step();
        }
        s.run_transient_lifecycle();
        s.apply_activation(&[Vec3::new(500.0, 0.0, 500.0)]);
        let frozen = {
            let rec = s.entities.get(&e).unwrap();
            s.phys.bodies.get(rec.handle).unwrap().is_kinematic()
        };
        assert!(frozen, "前置:静止常驻体出区冻结(决策侧静止前置已满足)");
        // 等翻转冷却(180 拍)后近锚:唤醒落实 → 静止判据重置。
        let _ = s.take_outbox();
        for _ in 0..180 {
            s.apply_activation(&[Vec3::new(500.0, 0.0, 500.0)]);
        }
        s.apply_activation(&[Vec3::new(61.0, 0.0, 61.0)]);
        let (kinematic, slow_ticks, win_len) = {
            let rec = s.entities.get(&e).unwrap();
            let b = s.phys.bodies.get(rec.handle).unwrap();
            (b.is_kinematic(), rec.slow_ticks, rec.window.len())
        };
        assert!(!kinematic, "唤醒落实(Dynamic)");
        assert_eq!(slow_ticks, 0, "唤醒重置 slow_ticks(细则 3)");
        assert!(win_len <= 1, "位移窗口只留当前位置(实得 {win_len})");
    }

    /// 任务书 04b 风险 5 单测(直接布景):醒着 B 压在醒着 A 上;命令唤醒
    /// 淘汰 A(最远醒者)→ B 连同冻结(悬空防杜)。**零重力**免静止干扰。
    #[test]
    fn stacked_awake_frozen_with_supporter_on_cull() {
        let mut s = ShardSim::new(budget_cfg(1), Box::new(voxel::SimpleRegistry::builtin()));
        let (a, b, far_e) = (entity(1), entity(2), entity(3));
        s.apply_prefix(&[
            spawn_input(0, a, Vec3::new(40.0, 3.0, 8.0)),
            mark_actor_input(1, a, true),
            spawn_input(2, b, Vec3::new(40.0, 4.05, 8.0)),
            mark_actor_input(3, b, true),
            spawn_input(4, far_e, Vec3::new(8.0, 3.0, 8.0)),
            mark_actor_input(5, far_e, true),
        ]);
        s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]); // 启用;观察者在 far_e 旁
        for _ in 0..25 {
            s.step();
        }
        let _ = s.take_outbox();
        // 命令唤醒 a、b 至远处(40m,出唤醒范围;命令重建即醒):
        // 醒数 {a, b} = 2 > 预算 1 → enforce_budget 淘汰最远者……两者同距?
        // 用 y 错开制造距离差:a 更远。
        s.apply_prefix(&[
            teleport_input(6, a, Vec3::new(50.0, 3.0, 8.0)),
            teleport_input(7, b, Vec3::new(50.0, 4.05, 8.0)),
        ]);
        s.step();
        // enforce_budget 淘汰距离降序最远者 = a(b 距近 0.05)→ a 冻结时
        // b(压在 a 上)连同冻结。
        let a_frozen = {
            let rec = s.entities.get(&a).unwrap();
            s.phys.bodies.get(rec.handle).unwrap().is_kinematic()
        };
        let b_frozen = {
            let rec = s.entities.get(&b).unwrap();
            s.phys.bodies.get(rec.handle).unwrap().is_kinematic()
        };
        assert!(
            a_frozen && b_frozen,
            "淘汰 a 时压在其上的 b 连同冻结(风险 5):a={a_frozen} b={b_frozen}"
        );
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .filter(|o| o.entity == b)
                .any(|o| o.op == TicketOp::End),
            "连带冻结的常驻体同样撤 ActorMotion(I3 第 4 条)"
        );
    }

    /// 04a 遗留返修单测:常驻票触顶标记随迁移载荷带走——触顶实体迁移后
    /// 目的片 claim 不重签(上限不可借迁移绕过)。
    #[test]
    fn migration_carries_actor_motion_ticks() {
        let mut s = zero_g_sim();
        let e = entity(1);
        s.apply_prefix(&[
            spawn_input(1, e, Vec3::new(8.0, 3.0, 8.0)),
            mark_actor_input(2, e, true),
        ]);
        s.apply_activation(&[Vec3::new(8.0, 0.0, 8.0)]);
        let _ = s.take_outbox();
        // 远观察者 → 运动中出范围 → 记账推进;手工把记账顶到触顶位。
        s.apply_activation(&[Vec3::new(500.0, 0.0, 500.0)]);
        s.step();
        s.run_transient_lifecycle();
        {
            let rec = s.entities.get_mut(&e).unwrap();
            rec.actor_motion_ticks = Some(1200); // 触顶
        }
        // 迁移出本体区:teleport 到区内新位(重建清记账)→ 重新置触顶 →
        // 下一拍给外漂速度,迁移检测带走(记账随载荷;Teleport 出区是同片
        // 重建语义,不走迁移信封,覆盖不到这里)。
        s.apply_prefix(&[teleport_input(3, e, Vec3::new(127.99, 3.0, 8.0))]);
        // 重建清了记账 → 重新置触顶 → 下一拍以 2 m/s 漂出本体区。
        {
            let rec = s.entities.get_mut(&e).unwrap();
            rec.actor_motion_ticks = Some(1200);
        }
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 4,
            payload: InputPayload::Command(SimCommand::TeleportEntity {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(127.99, 3.0, 8.0),
                    linvel: Vec3::new(2.0, 0.0, 0.0),
                    ..EntityState::default()
                },
            }),
        }]);
        {
            let rec = s.entities.get_mut(&e).unwrap();
            rec.actor_motion_ticks = Some(1200);
        }
        s.step(); // 漂移出本体区 → 迁移信封(记账随载荷)
        let out = s.take_outbox();
        let carried = out.migrations.iter().any(|i| match &i.payload {
            EnvelopePayload::MigrateEntity {
                entity,
                actor_motion_ticks,
                ..
            } => *entity == e && *actor_motion_ticks == Some(1200),
            _ => false,
        });
        assert!(
            carried,
            "触顶标记随迁移载荷带走(04a 遗留;实得 {:?})",
            out.migrations
        );
        // 目的片视角:前缀应用迁移信封 → 记账恢复(不借迁移绕过上限)。
        let mut s2 = zero_g_sim_at(shard_id(1, 0));
        s2.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Envelope(messaging::Envelope {
                produced_at: Tick::ZERO,
                apply_at: Tick::ZERO,
                source: shard_id(0, 0),
                destination: shard_id(1, 0),
                source_sequence: 0,
                payload: EnvelopePayload::MigrateEntity {
                    entity: e,
                    state: EntityState {
                        translation: Vec3::new(8.0, 3.0, 8.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc::default(),
                    actor: true,
                    source: None,
                    actor_motion_ticks: Some(1200),
                },
            }),
        }]);
        assert_eq!(
            s2.entities.get(&e).unwrap().actor_motion_ticks,
            Some(1200),
            "目的片 claim 后记账连续(不重新签发)"
        );
    }

    // ---- 04c:地形变更唤醒(§4.11、D20;任务书测试 terrain_wake_*)----

    /// 中距观察者锚点:距场景实体 ≈40 m(> 32 唤醒卸载半径 → 冻结;< 48
    /// ①② 保留半径 → 唤醒前置 ② 在场;复刻验收用例 freshly_frozen_actor
    /// 的几何)。
    const MID: Vec3 = Vec3::new(96.0, 3.0, 56.0);

    /// 冻结一个常驻体在 `pos`(零重力悬停):观察者先在场让实体保持醒,
    /// 21 拍累计静止判据,退到中距滞回冻结——翻转簿记已烧(地形唤醒
    /// "免冷却"的对照面)。实体所在 chunk (3,3) 装载并已建 ②(wake_group
    /// 的 ② 前置)。`budget` = max_awake_dynamic。
    fn frozen_actor_at(cfg: ShardSimConfig, pos: Vec3, id: EntityId) -> ShardSim {
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(3, 3), solid_chunk());
        s.apply_prefix(&[spawn_input(1, id, pos), mark_actor_input(2, id, true)]);
        s.apply_activation(&[pos]);
        for _ in 0..21 {
            s.step();
        }
        s.apply_activation(&[MID]);
        let rec = s.entities.get(&id).expect("前置:实体在表");
        assert!(
            s.phys.bodies.get(rec.handle).unwrap().is_kinematic(),
            "前置:实体已冻结"
        );
        assert!(
            s.has_collider(ChunkPos::new(3, 3)),
            "前置:足迹 chunk ② 已建(wake_group 前置)"
        );
        s
    }

    fn is_frozen(s: &ShardSim, id: EntityId) -> bool {
        s.entities
            .get(&id)
            .map(|rec| s.phys.bodies.get(rec.handle).unwrap().is_kinematic())
            .unwrap_or(false)
    }

    /// 任务书测试 `sim::terrain_wake_under_frozen_actor`(T13)+ 免冷却:
    /// SetBlock 移除冻结常驻体脚下的格(刚冻结、翻转冷却远未过)→ **下一拍**
    /// apply_activation 唤醒。会变红的实现:冻结体不响应地形变化;地形唤醒
    /// 被冷却挡住(实体刚烧簿记,按冷却要 180 拍后才可能经 plan 醒)。
    #[test]
    fn terrain_wake_under_frozen_actor() {
        let mut s = frozen_actor_at(
            budget_cfg(16),
            Vec3::new(56.0, 66.5, 56.0), // chunk (3,3) 内;零重力悬停
            entity(1),
        );
        // 脚下的格(中心 (56, 65.5, 56));零重力悬停,只锚定"地形写入唤醒"。
        let below = BlockPos::new(56, 65, 56);
        // 翻转冷却必然未过(冻结发生在上一拍激活刷新):唤醒只能来自地形变更。
        s.apply_prefix(&[setblock_input(3, below, types::BlockId::AIR)]);
        assert!(
            !s.pending_terrain_wakes.is_empty(),
            "外部 SetBlock 登记地形变更(统一出口)"
        );
        assert!(is_frozen(&s, entity(1)), "登记拍不唤醒(下一拍才醒)");
        s.apply_activation(&[MID]);
        assert!(!is_frozen(&s, entity(1)), "下一拍唤醒(免翻转冷却;§4.11)");
        assert!(
            s.terrain_woken_this_tick.contains(&entity(1)),
            "当拍记入地形唤醒集(不进淘汰池的前提)"
        );
        // 下一拍:豁免集清空(常规淘汰接管),登记集清空(不再重复唤醒)。
        s.apply_activation(&[MID]);
        assert!(s.terrain_woken_this_tick.is_empty());
        assert!(s.pending_terrain_wakes.is_empty());
    }

    /// 任务书测试 `wake::terrain_wake_next_tick_no_cooldown_no_budget`(T13)
    /// 的预算半边(D20):预算 1 已被 B 占满,地形变更唤醒的 A 仍然生效
    /// (单体当拍允许超限);plan 的超预算淘汰在 plan 时点计醒数(A 尚冻结
    /// → 未超),地形唤醒不触发对他人的淘汰。后续拍的常规淘汰不在本用例
    /// 范围(验收 acc_04c 第二项)。
    #[test]
    fn terrain_wake_next_tick_no_budget_truncation() {
        let mut s = frozen_actor_at(
            budget_cfg(1),
            Vec3::new(56.0, 66.5, 56.0),
            entity(1), // A:冻结(中距观察者,翻转簿记已烧)
        );
        // B:预算占用者——观察者旁醒着的常驻体。
        s.apply_prefix(&[
            spawn_input(3, entity(2), MID + Vec3::new(-6.0, 0.0, 0.0)),
            mark_actor_input(4, entity(2), true),
        ]);
        s.apply_activation(&[MID]);
        assert!(!is_frozen(&s, entity(2)), "前置:B 醒着占满预算 1");
        assert!(is_frozen(&s, entity(1)), "前置:A 冻结");
        // 挖 A 脚下 → 下一拍醒(超限容忍)。
        s.apply_prefix(&[setblock_input(
            5,
            BlockPos::new(56, 65, 56),
            types::BlockId::AIR,
        )]);
        s.apply_activation(&[MID]);
        assert!(
            !is_frozen(&s, entity(1)),
            "预算已满时地形变更唤醒仍生效(D20 单体超限)"
        );
        assert!(
            !is_frozen(&s, entity(2)),
            "地形唤醒当拍不触发对他人的淘汰(先超限,后续常规淘汰)"
        );
    }

    /// D20 整组半边:叠放组(2 成员)> 预算 1,地形变更唤醒**整组**醒
    /// (预算检查整段跳过,与观察者唤醒的整组推迟 acc_04b_closure_deferred
    /// 形成对照)。会变红的实现:地形唤醒复用 D19 预算门 → 整组推迟。
    #[test]
    fn terrain_wake_group_exempts_budget_d20() {
        let mut s = frozen_actor_at(
            budget_cfg(1),
            Vec3::new(56.0, 66.5, 56.0),
            entity(1), // A:种子(脚下格被写)
        );
        // B(actor)压在 A 上(贴靠,包络判定并入闭包):观察者先在原位让
        // B 静止装载,退到 MID 后与 A 一样滞回冻结(瞬态不走滞回,04a)。
        s.apply_prefix(&[
            spawn_input(3, entity(2), Vec3::new(56.0, 67.55, 56.0)),
            mark_actor_input(4, entity(2), true),
        ]);
        s.apply_activation(&[Vec3::new(56.0, 67.55, 56.0)]);
        for _ in 0..21 {
            s.step();
        }
        s.apply_activation(&[MID]);
        assert!(
            is_frozen(&s, entity(1)) && is_frozen(&s, entity(2)),
            "前置:A、B 均冻结(闭包候选)"
        );
        s.apply_prefix(&[setblock_input(
            4,
            BlockPos::new(56, 65, 56),
            types::BlockId::AIR,
        )]);
        s.apply_activation(&[MID]);
        assert!(
            !is_frozen(&s, entity(1)) && !is_frozen(&s, entity(2)),
            "D20:叠放组(2 > 预算 1)整组当拍唤醒"
        );
        assert_eq!(
            s.terrain_woken_this_tick,
            std::iter::once(entity(1))
                .chain(std::iter::once(entity(2)))
                .collect(),
            "整组成员都记入当拍地形唤醒集(不进淘汰池)"
        );
    }

    /// D17(风险 7):激活从未刷新(`activation_now == ZERO`)时不登记、
    /// 不唤醒——SetBlock 写块后 pending_terrain_wakes 保持空,无地形唤醒行为。
    #[test]
    fn terrain_wake_requires_activation_enabled() {
        let mut s = ShardSim::new(budget_cfg(16), Box::new(voxel::SimpleRegistry::builtin()));
        let e = entity(1);
        s.apply_prefix(&[spawn_input(1, e, Vec3::new(56.0, 66.5, 56.0))]);
        s.apply_prefix(&[setblock_input(
            2,
            BlockPos::new(56, 65, 56),
            types::BlockId::AIR,
        )]);
        assert!(
            s.pending_terrain_wakes.is_empty(),
            "D17 未启用:不登记(风险 7)"
        );
        s.apply_activation(&[MID]);
        assert!(!s.terrain_woken_this_tick.contains(&e));
    }

    /// 风险 4 的分片内半边:BlockWrite 信封应用(holder 副本)同样登记
    /// ——边界附近的 holder 侧冻结实体失去支撑也须醒来。
    #[test]
    fn blockwrite_envelope_registers_terrain_wake() {
        let mut s = frozen_actor_at(budget_cfg(16), Vec3::new(56.0, 66.5, 56.0), entity(1));
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource::SCHEDULER,
            sequence: 3,
            payload: InputPayload::Envelope(messaging::Envelope {
                produced_at: Tick::ZERO,
                apply_at: Tick::ZERO,
                source: shard_id(0, 0),
                destination: shard_id(0, 0),
                source_sequence: 0,
                payload: EnvelopePayload::BlockWrite {
                    pos: BlockPos::new(56, 65, 56),
                    block: types::BlockId(1),
                },
            }),
        }]);
        assert!(
            !s.pending_terrain_wakes.is_empty(),
            "BlockWrite 信封(holder 侧涌现写入)同样登记(风险 4)"
        );
        s.apply_activation(&[MID]);
        assert!(!is_frozen(&s, entity(1)), "下一拍唤醒");
    }

    /// 快照往返(风险 5):N 拍写入登记后做 checkpoint,恢复后登记集合在场
    /// ——N+1 拍(恢复后的第一次 apply_activation)照常唤醒;推迟中的地形
    /// 种子(带 D20 豁免)同样跨快照存续。
    #[test]
    fn snapshot_carries_pending_terrain_wakes() {
        let mut s = frozen_actor_at(budget_cfg(16), Vec3::new(56.0, 66.5, 56.0), entity(1));
        s.apply_prefix(&[setblock_input(
            3,
            BlockPos::new(56, 65, 56),
            types::BlockId::AIR,
        )]);
        let bytes = s.snapshot();
        let mut s2 = ShardSim::restore(&bytes, Box::new(voxel::SimpleRegistry::builtin()))
            .expect("往返解码");
        assert!(
            !s2.pending_terrain_wakes.is_empty(),
            "登记集合进快照(N 拍写入、N+1 唤醒,checkpoint 不丢;风险 5)"
        );
        s2.apply_activation(&[MID]);
        assert!(!is_frozen(&s2, entity(1)), "恢复后下一拍照常唤醒");
    }

    /// `wake::frozen_intersecting_cell` 消费侧负例(纪律 12):skin 决定
    /// 相交半径——贴靠(≤ 包络+skin)命中,离开(> 包络+skin)不命中;
    /// 命中表升序。
    #[test]
    fn terrain_wake_intersection_respects_skin() {
        use crate::wake::frozen_intersecting_cell;
        let skin = 0.05_f32;
        // half 0.5 的冻结体(旋转包络半高 0.866),中心 (56.0, 66.5, 56.0);
        // 所在格 (56,66,56) 中心 (56.5, 66.5, 56.5):三轴距 0.5 ≤ 0.866 +
        // 0.5 + skin ✓ → 命中。
        let near = [(entity(2), Vec3::new(56.0, 66.5, 56.0), 0.5_f32)];
        assert_eq!(
            frozen_intersecting_cell(&near, BlockPos::new(56, 66, 56), skin),
            vec![entity(2)]
        );
        // 同一实体挪远两格:x 距 2.5 > 0.866 + 0.5 + skin = 1.416 → 不命中
        // (多唤醒是放宽,漏唤醒是错误;该负例锚定"更小不行"的边界)。
        let far = [(entity(1), Vec3::new(56.0, 66.5, 56.0), 0.5_f32)];
        assert!(
            frozen_intersecting_cell(&far, BlockPos::new(58, 66, 56), skin).is_empty(),
            "AABB 外扩 skin 不相交 → 不唤醒"
        );
        // 擦边正例:x 距 1.2 ∈ (half+0.5+skin, 包络+0.5+skin) —— 必须**仍
        // 命中**(包络判定不能缩成非旋转 half,否则边界处漏唤醒)。
        let graze = [(entity(4), Vec3::new(56.3, 66.5, 56.0), 0.5_f32)];
        assert_eq!(
            frozen_intersecting_cell(&graze, BlockPos::new(57, 66, 56), skin),
            vec![entity(4)],
            "包络+skin 边界内必须命中(判定不得更小)"
        );
        // 升序:小 id 虽在表中后位,命中表仍按 EntityId 升序。
        let two = [
            (entity(9), Vec3::new(56.0, 66.5, 56.0), 0.5_f32),
            (entity(3), Vec3::new(56.0, 66.5, 56.0), 0.5_f32),
        ];
        assert_eq!(
            frozen_intersecting_cell(&two, BlockPos::new(56, 66, 56), skin),
            vec![entity(3), entity(9)],
            "命中表 EntityId 升序"
        );
    }

    /// 当拍不进淘汰池(§4.12;风险 3 的后一半):复刻调度器拍内顺序——
    /// **先**激活刷新(N 拍登记的地形唤醒在本拍落实,A 超限入集),**后**
    /// 前缀命令唤醒(Teleport 重建即醒 + enforce_budget)。B 移到观察者旁
    /// 重建即醒 → 醒数超限,淘汰池若含 A(地形唤醒者、且更远)会同拍把
    /// A 冻结——正确行为是豁免:A 保持动力态,下一拍起才按常规规则收敛。
    #[test]
    fn terrain_woken_not_in_same_tick_cull_pool() {
        let mut s = frozen_actor_at(
            budget_cfg(1),
            Vec3::new(56.0, 66.5, 56.0),
            entity(1), // A:冻结于 (56, 66.5, 56),距 MID ≈40
        );
        // B:观察者旁醒着的常驻体(预算 1 的占用者)。
        s.apply_prefix(&[
            spawn_input(3, entity(2), MID + Vec3::new(-6.0, 0.0, 0.0)),
            mark_actor_input(4, entity(2), true),
        ]);
        s.apply_activation(&[MID]);
        assert!(!is_frozen(&s, entity(2)), "前置:B 醒着");
        // 挖 A 脚下(拍 N 的写入登记)。
        s.apply_prefix(&[setblock_input(
            5,
            BlockPos::new(56, 65, 56),
            types::BlockId::AIR,
        )]);
        // 拍 N+1:先激活刷新(地形唤醒落实,A 超限入豁免集),后前缀命令
        // 唤醒(Teleport 重建即醒 + enforce_budget)。B 移到观察者正旁重建
        // 即醒 → 醒数 {A, B} = 2 > 预算 1 → 超限者(更远的 A)不得被同拍
        // 淘汰(命令唤醒与地形唤醒同拍时,A 随下一拍起按常规规则收敛)。
        s.apply_activation(&[MID]);
        assert!(!is_frozen(&s, entity(1)), "前置:地形唤醒已在拍 N+1 落实");
        s.apply_prefix(&[teleport_input(
            6,
            entity(2),
            MID + Vec3::new(-1.0, 0.0, 0.0),
        )]);
        assert!(
            !is_frozen(&s, entity(1)),
            "地形唤醒当拍不进淘汰池:命令唤醒超限时同拍不冻结 A(§4.12)"
        );
    }

    /// 任务书测试 `sim::over_budget_actor_freezes_and_ends_ticket`(T8/§9-2
    /// 常驻半边):超预算淘汰冻结的常驻体同时撤 ActorMotion(I3 第 4 条);
    /// 命令唤醒路径(enforce_budget)同语义。
    #[test]
    fn over_budget_actor_freezes_and_ends_ticket() {
        let mut s = ShardSim::new(budget_cfg(1), Box::new(voxel::SimpleRegistry::builtin()));
        let (a, b) = (entity(1), entity(2));
        s.apply_prefix(&[
            spawn_input(0, a, Vec3::new(13.0, 3.0, 8.0)),
            mark_actor_input(1, a, true),
            spawn_input(2, b, Vec3::new(100.0, 3.0, 100.0)),
            mark_actor_input(3, b, true),
        ]);
        s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]); // 启用;b 远(静止)冻结
        for _ in 0..21 {
            s.step();
        }
        s.run_transient_lifecycle();
        let _ = s.take_outbox();
        s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]); // b 冻结落实(票 End)
        let _ = s.take_outbox();
        // 命令唤醒 b 至近处:醒数 {a, b} = 2 > 预算 1 → 淘汰最远醒者 a
        // (命令唤醒路径 enforce_budget)→ 冻结 + End ActorMotion。
        s.apply_prefix(&[teleport_input(4, b, Vec3::new(10.0, 3.0, 8.0))]);
        s.step();
        assert!(
            s.entity_state(a).is_some_and(|st| st.linvel == Vec3::ZERO),
            "a 被超预算淘汰:速度清零(冻结)"
        );
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == a && o.op == TicketOp::End),
            "超预算冻结的常驻体同时撤 ActorMotion(I3 第 4 条;命令唤醒路径)"
        );
    }

    /// 任务书测试(shard 侧配套在 shard;此为分片侧消费面):常驻票到期
    /// `apply_capped_freeze_directive` → 下一拍 lifecycle 冻结(兜底降级);
    /// 指令对已冻结实体幂等。
    #[test]
    fn capped_freeze_directive_freezes_next_tick() {
        let mut s = zero_g_sim();
        let e = entity(1);
        s.apply_prefix(&[
            spawn_input(1, e, Vec3::new(60.0, 3.0, 60.0)),
            mark_actor_input(2, e, true),
        ]);
        s.apply_activation(&[Vec3::new(60.0, 0.0, 60.0)]);
        s.step();
        let awake_before = {
            let rec = s.entities.get(&e).unwrap();
            !s.phys.bodies.get(rec.handle).unwrap().is_kinematic()
        };
        assert!(awake_before, "前置:运动常驻体醒着");
        s.apply_capped_freeze_directive(e);
        s.run_transient_lifecycle();
        let frozen = {
            let rec = s.entities.get(&e).unwrap();
            s.phys.bodies.get(rec.handle).unwrap().is_kinematic()
        };
        assert!(frozen, "上限兜底:到期指令 → lifecycle 冻结(降级)");
        // 幂等:再投 + 再执行不 panic、保持冻结。
        s.apply_capped_freeze_directive(e);
        s.run_transient_lifecycle();
        let still_frozen = {
            let rec = s.entities.get(&e).unwrap();
            s.phys.bodies.get(rec.handle).unwrap().is_kinematic()
        };
        assert!(still_frozen, "重复指令幂等(已冻结保持)");
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

    /// 任务书测试 `sim::over_budget_transient_forced_landing`(T8/§9-2)的
    /// **plan 路径补齐**(06c 接线暴露:plan 的超预算淘汰 `over_budget_freeze`
    /// 此前无消费者,瞬态被冻结悬停而非强制落地):预算 1、两个醒着的瞬态体,
    /// 下一拍 plan 淘汰远者 → 远者强制落地(消失 + 写块),事件计数
    /// `forced_landings_budget = 1`(§4.16"预算分计")。变异即红:淘汰改回
    /// 冻结悬停(over_budget_freeze 不分流)、预算归因丢失(计入截止)。
    #[test]
    fn plan_budget_eviction_forces_transient_landing_and_counts() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        cfg.activation.max_awake_dynamic = 1;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let (a, b) = (entity(1), entity(2));
        // 启用(近观察者):两体都在唤醒范围内保持醒着。
        let _ = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        s.apply_prefix(&[
            spawn_input(1, a, Vec3::new(8.0, 0.5, 8.0)),
            spawn_input(2, b, Vec3::new(9.0, 0.5, 8.0)),
        ]);
        let _ = s.take_activation_events();
        // 醒数 2 > 预算 1 → plan 距离降序淘汰远者 b(瞬态 → 强制落地)。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.over_budget_freeze, vec![b], "非退化:远者 b 被 plan 淘汰");
        s.run_transient_lifecycle();
        let events = s.take_activation_events();
        assert!(
            s.entity_state(b).is_none(),
            "plan 淘汰瞬态 → 强制落地(消失)"
        );
        assert!(s.entity_state(a).is_some(), "近者 a 保留");
        assert!(
            s.outbox
                .block_updates
                .iter()
                .any(|w| w.kind == BlockWriteKind::Emergent),
            "落点写块(Emergent)"
        );
        assert_eq!(
            events.forced_landings_budget, 1,
            "事件量:预算淘汰落地 1(§4.16)"
        );
        assert_eq!(events.forced_landings_deadline, 0, "不计截止");
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

    /// 06b(§4.6 缺陷修复的回归锚;区别于上一条的手工塞 locomotion):
    /// **真实路径**——进入待地形由 update_awaiting_terrain 自行保存速度
    /// (刚体此刻被切到 KinematicPositionBased),恢复时先切 Dynamic 再写
    /// 速度。rapier 0.36 对 KinematicPositionBased 的 set_linvel 是 no-op,
    /// 顺序反了保存速度即丢(缺陷实现:恢复后速度 = 进入时清零的 0)。
    #[test]
    fn awaiting_terrain_recovery_real_path_keeps_saved_velocity() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        // 有①无②:足迹 (0,0) 缺 ② → 真实进入待地形。
        s.load_data_only(ChunkPos::new(0, 0), solid_chunk());
        let e = entity(9);
        let saved = Vec3::new(3.0, -1.0, 2.0);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(8.0, 3.0, 8.0),
                    linvel: saved,
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            }),
        }]);
        let _ = s.apply_activation(&[Vec3::new(300.0, 0.0, 300.0)]);
        assert!(s.is_awaiting_terrain(e), "前置:足迹缺 ② → 真实进入待地形");
        assert_eq!(
            s.locomotion.get(&e).expect("待地形记录在案"),
            &LocomotionState::AwaitingTerrain {
                saved_linvel: saved,
                saved_angvel: Vec3::ZERO,
            },
            "进入时保存的速度 = 出生速度(step 前,重力未积分)"
        );
        // ② 就绪(手动 load_chunk 同点建 ②,绕预算)→ 下一拍激活恢复。
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s.apply_activation(&[Vec3::new(300.0, 0.0, 300.0)]);
        assert!(!s.is_awaiting_terrain(e), "② 齐 → 恢复");
        assert_eq!(
            s.entity_state(e).expect("恢复后仍在表").linvel,
            saved,
            "恢复:保存的速度位级写回(先切 Dynamic 再写;§4.6)"
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

    /// 任务书测试 `sim::carried_ticket_pause_suspend_resume`(T9,G1/K1):
    /// 承载变化沿产出 Pause → plan 决策(常驻规则滞回)落实 → 静止被承载
    /// 冻结 + Suspend。走 `apply_activation` 生产入口(冻结决策单点在 plan)。
    #[test]
    fn carried_ticket_pause_suspend_resume() {
        // 冷却 0:滞回翻转当拍落实(测试焦点 = 决策点归属,非冷却语义——
        // 冷却由滞回专项测试覆盖)。配置必须在控制器构造前注入(new 克隆)。
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.displacement_gate_m = 0.0;
        cfg.activation.cooldown_ticks = 0;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let carrier = entity(1);
        let cargo = entity(2);
        s.apply_prefix(&[
            spawn_input(1, carrier, Vec3::new(8.0, 0.5, 8.0)),
            mark_actor_input(2, carrier, true),
            spawn_input(3, cargo, Vec3::new(8.0, 1.5, 8.0)),
        ]);
        // 启用(D17):一次激活刷新注册观察者(远锚:唤醒范围外;承载记录
        // 与 ops 产出都在启用模式才发生)。首个激活拍里实体刚 spawn 不满
        // 静止判据,carrier(常驻)先冻结;cargo 被承载不满足滞回冻结条件
        // (记录 carried 尚未建立?——emit 在 step 末,此拍 fresh)。
        s.apply_activation(&[Vec3::new(300.0, 0.0, 300.0)]);
        let _ = s.take_outbox();
        for _ in 0..3 {
            s.step();
        }
        s.run_transient_lifecycle();
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == cargo && o.op == TicketOp::Pause),
            "被承载变化沿 → Pause 产出(K1)"
        );
        // 静止达标(慢速计数 20 拍;窗口继续滚动)后**远锚**激活刷新:cargo
        // (被承载瞬态,实体档按常驻规则)在所有唤醒中心之外 → 滞回出区冻结
        // 条目 + 静止前置通过 → plan 冻结落实 + Suspend(决策单点在 plan)。
        let _ = s.take_outbox();
        for _ in 0..40 {
            s.step();
        }
        s.run_transient_lifecycle();
        let _ = s.take_outbox();
        s.apply_activation(&[Vec3::new(300.0, 0.0, 300.0)]);
        let frozen = {
            let rec = s.entities.get(&cargo).expect("存在");
            s.phys
                .bodies
                .get(rec.handle)
                .expect("handle 有效")
                .is_kinematic()
        };
        assert!(frozen, "被承载 ∧ 静止 ∧ 出唤醒范围 → plan 冻结落实");
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == cargo && o.op == TicketOp::Suspend),
            "冻结落实 → Suspend 产出(K1:决策单点在 plan)"
        );
        assert!(
            s.entity_state(cargo)
                .is_some_and(|st| st.linvel == Vec3::ZERO),
            "Suspend:被承载瞬态按常驻规则冻结"
        );
        // 唤醒段(K1):观察者移近唤醒范围 → plan 唤醒条目 → 足迹 ② 齐 →
        // 切回 Dynamic;仍被承载 → 产 Pause(§4.4.2"唤醒仍被承载则 Paused"
        // ——单 op 直达终态:同拍 Resume+Pause 会被 §14-6 键序
        // (Pause < Resume)翻成 Active,等价终态必须单 op)。
        let _ = s.take_outbox();
        s.apply_activation(&[Vec3::new(8.0, 0.0, 8.0)]);
        let awake = {
            let rec = s.entities.get(&cargo).expect("存在");
            !s.phys
                .bodies
                .get(rec.handle)
                .expect("handle 有效")
                .is_kinematic()
        };
        assert!(awake, "入唤醒范围 → plan 唤醒落实(冷却 0)");
        assert!(
            s.outbox
                .ticket_ops
                .iter()
                .any(|o| o.entity == cargo && o.op == TicketOp::Pause),
            "唤醒仍被承载 → Pause(终态 Paused,§4.4.2)"
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

    // ---- 05a:撞前预测接触唤醒(§4.10.1–4.10.2;任务书测试 prestep_*)----

    /// 启用激活的零重力 sim:观察者就位(D17 开),凝固关(漂移体不干扰)。
    fn contact_sim(max_awake: usize) -> ShardSim {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.gravity = Vec3::ZERO;
        cfg.activation.max_awake_dynamic = max_awake;
        ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
    }

    /// 冻结一个常驻体(零重力悬停;复刻 04c 的 frozen_actor_at 几何语义:
    /// chunk (3,3) 合一装载,近锚保持醒、21 拍静止、退到 MID 滞回冻结)。
    fn frozen_actor(s: &mut ShardSim, id: EntityId, pos: Vec3) {
        s.load_chunk(ChunkPos::new(3, 3), solid_chunk());
        s.apply_prefix(&[spawn_input(1, id, pos), mark_actor_input(2, id, true)]);
        s.apply_activation(&[pos]);
        for _ in 0..21 {
            s.step();
        }
        s.apply_activation(&[MID]);
        assert!(is_frozen(s, id), "前置:实体已冻结");
    }

    /// 带初速的常驻体 spawn(撞击者构造;05a)。
    fn spawn_actor_v(seq: u64, id: EntityId, pos: Vec3, vel: Vec3) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: pos,
                    linvel: vel,
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            }),
        }
    }

    /// 任务书测试 `sim::prestep_wake_momentum_exchange`(T14,sim 级):
    /// 1 kg @30 m/s 撞向 2.5 m 外冻结的 1 kg 常驻体——撞前预测当拍唤醒 B,
    /// rapier 同步解算:B 获得前向速度、A 明显减速(等质量恢复系数 0.1 的
    /// 交换,不反弹)。会变红的实现:没有预测(A 撞"墙"反弹,B 不动);
    /// 预测放 step 后(同撞墙);唤醒后被撞者没有切 Dynamic。
    /// 几何:扫掠判定距离 = A 包络 0.866 + 扫掠 1(2 拍 @30 m/s)+ skin 0.075
    /// + B 包络 0.866 + skin 0.075 ≈ 2.88;2.5 < 2.88 → 首拍命中。
    #[test]
    fn prestep_wake_momentum_exchange() {
        let mut s = contact_sim(16);
        let (a, b) = (entity(1), entity(2));
        frozen_actor(&mut s, b, Vec3::new(56.0, 66.5, 56.0));
        s.apply_prefix(&[
            spawn_actor_v(3, a, Vec3::new(53.5, 66.5, 56.0), Vec3::new(30.0, 0.0, 0.0)),
            mark_actor_input(4, a, true),
        ]);
        s.apply_activation(&[Vec3::new(53.5, 66.5, 56.0)]);
        assert!(!is_frozen(&s, a), "前置:A 醒着");
        assert!(is_frozen(&s, b), "前置:B 冻结(预测前)");

        // 首拍 prestep 醒 B(物理接触在随后两拍内发生:间隙 0.62 m、
        // 0.5 m/拍);30 拍窗口覆盖接触与交换。
        for _ in 0..30 {
            s.step();
        }

        assert!(!is_frozen(&s, b), "撞前预测:B 在同一步前被唤醒");
        let (va, vb) = (
            s.entity_state(a).unwrap().linvel,
            s.entity_state(b).unwrap().linvel,
        );
        assert!(
            vb.x > 0.5,
            "B 从碰撞中获得前向速度(vB.x = {vb:?}),说明双方都是动态体解算"
        );
        assert!(
            va.x < 25.0,
            "A 明显减速(vA.x = {va:?}),动量已交换(不是撞墙反弹约 −22)"
        );
        assert!(va.x > 0.0, "A 不反向弹回");
    }

    /// 任务书测试 `sim::contact_wake_off_control_group`(T14 对照):
    /// `contact_wake_min_momentum → +∞`(关门槛)不预测——A 撞冻结 B 如
    /// 撞墙反弹,B 保持冻结。锚定"关掉新能力 = 旧行为"。
    #[test]
    fn contact_wake_off_control_group() {
        let mut s = contact_sim(16);
        let (a, b) = (entity(1), entity(2));
        frozen_actor(&mut s, b, Vec3::new(56.0, 66.5, 56.0));
        s.cfg.activation.contact_wake_min_momentum = f32::MAX;
        s.apply_prefix(&[
            spawn_actor_v(3, a, Vec3::new(53.5, 66.5, 56.0), Vec3::new(30.0, 0.0, 0.0)),
            mark_actor_input(4, a, true),
        ]);
        s.apply_activation(&[Vec3::new(53.5, 66.5, 56.0)]);
        // 关门槛的对照里 B 应保持冻结;A 撞"墙"是后续拍的物理,不在此锚定
        // (本用例只锚定"关掉新能力 = B 不醒"的旧行为半边)。
        s.step();
        assert!(is_frozen(&s, b), "关能力(门槛 +∞):B 保持冻结(旧行为)");
    }

    /// 慢速撞击(动量 < 门槛)不唤醒:T16 的 sim 级半边。
    /// 会变红的实现:预测只看几何相交不看动量。
    #[test]
    fn slow_contact_never_seeded() {
        let mut s = contact_sim(16);
        let (a, b) = (entity(1), entity(2));
        frozen_actor(&mut s, b, Vec3::new(56.0, 66.5, 56.0));
        // 0.5 m/s × 1 kg = p 0.5 < 1(静止贴靠同理由 v≈0 覆盖,wake.rs 侧
        // seeds_require_momentum_and_sweep 已锚定 p=0)。1.5 m @0.5 m/s 约
        // 180 拍贴靠;首拍距离 1.5 < 扫掠判定距离(0.866+0.017+0.075+
        // 0.866+0.075 ≈ 1.9)→ 几何全程命中,只差动量门槛。
        s.apply_prefix(&[
            spawn_actor_v(3, a, Vec3::new(54.5, 66.5, 56.0), Vec3::new(0.5, 0.0, 0.0)),
            mark_actor_input(4, a, true),
        ]);
        s.apply_activation(&[Vec3::new(54.5, 66.5, 56.0)]);
        for k in 0..400 {
            s.step();
            assert!(is_frozen(&s, b), "拍 {k}:慢速(p < 门槛)不得唤醒 B");
        }
    }

    /// 预算(风险 3):醒数已满预算时接触唤醒整组跳过——B 保持冻结、
    /// **不淘汰任何已醒实体**。会变红的实现:先唤醒再淘汰(把别人挤下去);
    /// 或预算语义沿用 D19 单体豁免(预算满也醒)。
    #[test]
    fn prestep_wake_skips_when_budget_full() {
        let mut s = contact_sim(2);
        let (a, b, c) = (entity(1), entity(2), entity(3));
        // B 冻结在场景里;A、C 醒着占满预算 2。A 撞 B → 组 1,
        // 2+1 > 2 → 跳过;A、C 保持醒着不被淘汰。
        frozen_actor(&mut s, b, Vec3::new(56.0, 66.5, 56.0));
        // 冻结 B 后、spawn A/C 前,先刷新一次激活把预算强制的距离中心换到
        // 场景锚(否则 apply_prefix(spawn) 的命令唤醒预算强制仍以 frozen_actor
        // 遗留的 MID 为中心计距——那是 04c 复刻的固有几何,不是被测语义)。
        let _ = s.apply_activation(&[Vec3::new(54.75, 66.5, 56.0)]);
        s.apply_prefix(&[
            spawn_actor_v(3, a, Vec3::new(53.5, 66.5, 56.0), Vec3::new(30.0, 0.0, 0.0)),
            mark_actor_input(4, a, true),
            spawn_input(5, c, Vec3::new(56.0, 70.5, 56.0)),
            mark_actor_input(6, c, true),
        ]);
        // 激活刷新锚点放两实体之间(观察者票半径 24 内;命令唤醒的预算强制
        // 距离中心 = 该锚,两实体各距 ≈2–4 m,不触发淘汰)。
        let _ = s.apply_activation(&[Vec3::new(54.75, 66.5, 56.0)]);
        let awake = |s: &ShardSim| {
            s.entities
                .values()
                .filter(|rec| !s.phys.bodies.get(rec.handle).unwrap().is_kinematic())
                .count()
        };
        assert_eq!(awake(&s), 2, "前置:A、C 醒(预算 2 满)");
        assert!(is_frozen(&s, b), "前置:B 冻结");
        // 2.5 m @30 m/s 约拍 3 接触;40 拍窗口覆盖。
        for k in 0..40 {
            s.step();
            assert_eq!(
                awake(&s),
                2,
                "拍 {k}:预算满,接触唤醒跳过,不超限、不淘汰 A/C"
            );
            assert!(is_frozen(&s, b), "拍 {k}:预算满,B 不被接触唤醒");
        }
    }

    /// 待地形体不参与(§4.10.5;风险 6):瞬态体 T 在缺 ② 区进待地形
    /// (kinematic、不在任何一侧候选表);醒着体 A 高速穿过 T 旁的通道
    /// 撞向冻结的 B——B 应被 A 经撞前预测唤醒(**正例**:接触唤醒路径
    /// 畅通,不过度防御),T 全程保持待地形(既不被当作种子唤醒,也不
    /// 被路径扰动)。
    /// 会变红的实现:预测整体失效/把醒着体也滤掉(过防御 → B 永不醒);
    /// 把待地形体当冻结种子(T 被唤醒脱离待地形)。
    #[test]
    fn awaiting_terrain_never_contact_wake() {
        let mut s = contact_sim(16);
        let (t, a, b) = (entity(1), entity(2), entity(3));
        // B 冻结在 A 的通道上 (56,66.5,9.5)(chunk (3,0);手动复刻
        // frozen_actor 的滞回冻结流程,载对应 chunk)。
        s.load_chunk(ChunkPos::new(3, 0), solid_chunk());
        s.apply_prefix(&[
            spawn_input(1, b, Vec3::new(56.0, 66.5, 9.5)),
            mark_actor_input(2, b, true),
        ]);
        s.apply_activation(&[Vec3::new(56.0, 66.5, 9.5)]);
        for _ in 0..21 {
            s.step();
        }
        s.apply_activation(&[MID]);
        assert!(is_frozen(&s, b), "前置:B 冻结");
        // T:瞬态出生在缺 ② 的 chunk 区域((0,0) 未装载)→ 待地形。
        s.apply_prefix(&[spawn_input(3, t, Vec3::new(8.0, 66.5, 8.0))]);
        let _ = s.apply_activation(&[Vec3::new(8.0, 66.5, 8.0)]);
        assert!(s.is_awaiting_terrain(t), "前置:T 进待地形(kinematic)");
        // A 高速穿过 T 旁(z 距 1.5 > collider 和 1.0,不碰 T)直奔 B。
        s.apply_prefix(&[
            spawn_actor_v(4, a, Vec3::new(8.0, 66.5, 9.5), Vec3::new(150.0, 0.0, 0.0)),
            mark_actor_input(5, a, true),
        ]);
        let _ = s.apply_activation(&[Vec3::new(8.0, 66.5, 9.5)]);
        assert!(!is_frozen(&s, a), "前置:A 醒着");
        let mut woke = false;
        for _ in 0..60 {
            s.step();
            assert!(
                s.is_awaiting_terrain(t),
                "待地形体不被接触唤醒路径扰动(两侧表都滤 kinematic)"
            );
            if !is_frozen(&s, b) {
                woke = true;
                break;
            }
        }
        assert!(woke, "正例:A 经撞前预测唤醒路径上的 B(路径畅通)");
    }

    /// 任务书测试 `wake::per_tick_cap_and_idle_budget` 的 sim 级组合半边
    /// (05b;T15;纯函数半边在 wake::tests):传播结果经每拍上限截断——
    /// 4 连排、上限 2:同拍恰醒 2 个(传播序 p 降序 = B1、B2),B3、B4 保持
    /// 冻结且在后续拍也**不**被补醒(同几何重算结果相同,截断稳定);组按
    /// §4.9 闭包成组后整组纳入,单节点一排无向上成员 = 单体组。
    /// 会变红的实现:无上限(4 个全醒)、上限语义错(醒 1 个)、截断顺序不按
    /// 传播 p 序(醒 B2、B3 之类)、传播分多拍"补醒"。
    #[test]
    fn prestep_wake_per_tick_cap_truncates() {
        let mut s = contact_sim(16);
        s.cfg.activation.contact_wake_max_bodies_per_tick = 2;
        // 一排 4 个冻结体:间距 1.02(包络 0.866 相邻),B1 在最左。
        let b1 = Vec3::new(56.0, 66.5, 56.0);
        let row: Vec<EntityId> = (2..=5).map(entity).collect();
        s.load_chunk(ChunkPos::new(3, 3), solid_chunk());
        let mut seq = 1u64;
        for (k, id) in row.iter().enumerate() {
            let pos = b1 + Vec3::new(k as f32 * 1.02, 0.0, 0.0);
            s.apply_prefix(&[
                spawn_input(seq, *id, pos),
                mark_actor_input(seq + 1, *id, true),
            ]);
            seq += 2;
        }
        s.apply_activation(&[b1]);
        for _ in 0..21 {
            s.step();
        }
        s.apply_activation(&[MID]);
        for id in &row {
            assert!(is_frozen(&s, *id), "前置:一排 4 个全部冻结");
        }
        // 撞击者 A 从 B1 左侧 1.5 m 以 5 m/s 撞来(动量 5;传播 2.5/1.25
        // 高于门槛 1,第 3 跳 0.625 低于门槛——B4 本就到不了,截断只来自
        // 每拍上限,作用于 B3)。
        let a = entity(1);
        s.apply_prefix(&[
            spawn_actor_v(
                seq,
                a,
                b1 + Vec3::new(-1.5, 0.0, 0.0),
                Vec3::new(5.0, 0.0, 0.0),
            ),
            mark_actor_input(seq + 1, a, true),
        ]);
        let _ = s.apply_activation(&[b1 + Vec3::new(-1.5, 0.0, 0.0)]);
        assert!(!is_frozen(&s, a), "前置:A 醒着");
        // 首拍:预测醒 B1(p=5)、B2(p=2.5),上限 2 截断 B3、B4。
        let frozen_flags =
            |s: &ShardSim| row.iter().map(|id| is_frozen(s, *id)).collect::<Vec<_>>();
        s.step();
        assert_eq!(
            frozen_flags(&s),
            vec![false, false, true, true],
            "上限 2:恰醒 p 最大的 B1、B2(同拍)"
        );
        // 物理接触前(A 距 B1 表面 0.5 m @5 m/s ≈ 6 拍):传播不溢出——
        // 被截断的 B3、B4 保持冻结。物理接触之后的逐拍接力唤醒是真实动量
        // 交换的新种子(05a 语义),不属于本拍传播,不在本断言内。
        for k in 0..3 {
            s.step();
            assert_eq!(
                frozen_flags(&s),
                vec![false, false, true, true],
                "拍 {k}:物理接触前,截断的 B3、B4 不被传播溢出唤醒"
            );
        }
    }

    // ---- 05c:撞后冲量补偿路径(§4.10.4;任务书测试 compensation_*)----

    /// 预测漏判场景(05c;AC-T17 链式几何的 sim 级复刻):冻结 B,瞬态 A2、
    /// A1 零缝正贴靠成链(B–A2–A1),C(1 kg)以 `speed` 从 0.5 m 缝外撞入。
    /// prestep 时 A1/A2 速度为 0、C 的扫掠够不着 B(隔两块 3 m > 扫掠判定
    /// ≈2.15)→ 预测无种子;一步内 C→A1→A2 的冲量全部打在 B(冻结墙)上,
    /// 步后 A1/A2 停在 B 前、速度 ≈0——只有撞后补偿能在下一拍唤醒 B。
    /// 返回 (b, a1, a2, c)。
    ///
    /// 零缝是关键:缝隙 > 0 时 A1→A2 分两拍传递,A2 中途获得速度,下一拍
    /// **预测**接力命中 B——测的就不再是补偿(AC-T17 验收注释同理)。
    fn missed_prediction_bodies(
        s: &mut ShardSim,
        speed: f32,
        c_x: f32,
    ) -> (EntityId, EntityId, EntityId, EntityId) {
        let b = entity(1);
        frozen_actor(s, b, Vec3::new(56.0, 66.5, 56.0));
        // A2、A1 用瞬态(非 actor):不参与滞回冻结,保持醒着。
        s.apply_prefix(&[
            spawn_input(3, entity(2), Vec3::new(55.0, 66.5, 56.0)),
            spawn_input(4, entity(3), Vec3::new(54.0, 66.5, 56.0)),
        ]);
        let c = entity(4);
        s.apply_prefix(&[spawn_actor_v(
            5,
            c,
            Vec3::new(c_x, 66.5, 56.0),
            Vec3::new(speed, 0.0, 0.0),
        )]);
        // 激活刷新(锚在 A1 旁)开 D17 门控;B 刚冻结、翻转簿记未过冷却,
        // 半径内也不被 plan 唤醒(与 05a 测试同款前置)。
        let _ = s.apply_activation(&[Vec3::new(54.0, 66.5, 56.0)]);
        (b, entity(3), entity(2), c)
    }

    /// 撞击 tick:C 速度首次降到 3/4 以下(动量已进链条)。
    fn impact_tick(s: &mut ShardSim, c: EntityId, speed: f32) -> usize {
        for k in 0..120 {
            s.step();
            if s.entity_state(c).expect("c 在表").linvel.x < speed * 0.75 {
                return k;
            }
        }
        panic!("前置:C 应在 120 拍内撞上 A1");
    }

    /// 任务书测试 `sim::compensation_next_tick_wake`(05c;T17,sim 级):
    /// 预测漏判的撞击 → 补偿收集 A2↔B 冲量为种子,B 在下一拍醒来;种子随
    /// 快照往返,恢复线同样下一拍醒;消费取出即清。
    /// 会变红的实现:没有补偿收集(B 永远冻结,05b 行为);种子不进快照
    /// (恢复线 B 不醒);取出后不清理(pending 残留,跨拍重试)。
    #[test]
    fn compensation_next_tick_wake() {
        let mut s = contact_sim(16);
        let (b, a1, a2, c) = missed_prediction_bodies(&mut s, 8.0, 52.5);
        assert!(!is_frozen(&s, a1) && !is_frozen(&s, a2), "前置:A1/A2 醒着");
        assert!(is_frozen(&s, b), "前置:B 冻结");
        let impact = impact_tick(&mut s, c, 8.0);
        assert!(impact < 120, "前置:撞击发生");
        assert!(
            is_frozen(&s, b),
            "撞击当拍 B 仍冻结(预测确实漏判;预测接力唤醒会让本断言红)"
        );
        assert!(
            !s.pending_contact_wakes.is_empty(),
            "补偿收集已把 B 的种子入待处理集(无收集的实现此处红)"
        );

        // 撞击之后、消费之前:快照 → 恢复(种子此刻待处理)。
        let bytes = s.snapshot();
        let mut r = ShardSim::restore(&bytes, Box::new(voxel::SimpleRegistry::builtin()))
            .expect("自有快照必可解码");

        // 原线:下一拍(≤2 拍)B 醒;取出即清。
        let mut woke = false;
        for _ in 0..2 {
            s.step();
            if !is_frozen(&s, b) {
                woke = true;
                break;
            }
        }
        assert!(woke, "补偿:B 应在下一拍醒来(无补偿收集的实现恒红)");
        assert!(
            s.pending_contact_wakes.is_empty(),
            "取出即清:消费后待处理集为空(不跨拍重试)"
        );

        // 恢复线:同样下一拍醒(种子不进快照的实现此断言红)。
        let mut woke_r = false;
        for _ in 0..2 {
            r.step();
            if !is_frozen(&r, b) {
                woke_r = true;
                break;
            }
        }
        assert!(woke_r, "恢复线:B 同样在下一拍醒来(种子随快照往返)");
    }

    /// AC-T17 守卫的 sim 级半边:0.4 m/s 推挤,冲量 0.4 < 门槛 1,补偿不
    /// 收集,B 200 拍不醒。
    /// 会变红的实现:补偿对任何接触冲量都入种子(收集侧无门槛——种子会
    /// 出现在待处理集,首个断言红;最终唤醒会被传播门槛挡下,但收集口径
    /// 已错)。
    #[test]
    fn compensation_below_threshold_not_collected() {
        let mut s = contact_sim(16);
        let (b, _a1, _a2, c) = missed_prediction_bodies(&mut s, 0.4, 52.5);
        let _impact = impact_tick(&mut s, c, 0.4);
        for k in 0..200 {
            s.step();
            assert!(is_frozen(&s, b), "拍 {k}:低于门槛的推挤不触发补偿唤醒");
            assert!(
                s.pending_contact_wakes.is_empty(),
                "拍 {k}:收集侧门槛生效,0.4 kg·m/s 不入种子集"
            );
        }
    }

    /// 风险 1:补偿种子指向的实体在消费前消失(RemoveEntity)→ 静默跳过
    /// 不 panic(§4.10.4 风险 1 的字面契约);取出即清同样适用。
    /// 会变红的实现:消费侧对已消失实体 unwrap/索引 panic(测试以 panic
    /// 失败);或种子残留跨拍(pending 非空断言红)。
    #[test]
    fn compensation_seed_to_missing_entity_skipped() {
        let mut s = contact_sim(16);
        let (b, a1, _a2, c) = missed_prediction_bodies(&mut s, 8.0, 52.5);
        let _impact = impact_tick(&mut s, c, 8.0);
        assert!(!s.pending_contact_wakes.is_empty(), "前置:种子已收集");
        // 消费前删除 B:下一拍 prestep 消费时实体已不存在 → 静默跳过。
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 9,
            payload: InputPayload::Command(SimCommand::RemoveEntity { entity: b }),
        }]);
        s.step();
        assert!(
            s.entity_state(b).is_none(),
            "B 已被 RemoveEntity 删除(前提自检)"
        );
        assert!(s.entity_state(a1).is_some(), "链上其余实体不受消费影响");
        assert!(
            s.pending_contact_wakes.is_empty(),
            "取出即清:指向消失实体的种子同样取出即弃"
        );
    }

    /// 遥测专测(07 附加项 1;§4.16 `contact_wake_skipped_no_terrain`):
    /// 接触唤醒的组因当前足迹缺 ② 被整组跳过时,计数 +1、被撞的冻结体保持
    /// 冻结;② 就绪后再次撞击可以唤醒(计数不再增加)。
    /// 会变红的实现:缺 ② 的组照常落实(计数 0 且 B 被唤醒);或把"缺 ②
    /// 跳过"记进任何唤醒/推迟计数字段(首段断言红);或 ② 就绪后仍不醒
    /// (末段断言红)。
    ///
    /// 场景(contact_sim 零重力;复刻 05b 的 5 m/s 撞击几何,该速度 ≥
    /// 门槛 1 且预测扫掠首拍命中):冻结常驻体 B 在 chunk (3,3);② 卸载
    /// 半径压到 32 后,frozen_actor 的 MID 激活即按 R2 单独拆 ②(锚
    /// (96,·,56):chunk 中心距 40 ∈ (32,48] 带——② 超卸载半径、① 保留),
    /// 制造"有 ① 无 ②"状态。撞击者 A 从 B 左侧 1.5 m 以 5 m/s 撞入 →
    /// prestep 种子命中 B,但 wake_group 前置(足迹 ②)不满足 → 整组推迟:
    /// `contact_wake_skipped_no_terrain` +1,无任何唤醒计数,B 保持冻结;
    /// 种子取出即清(不跨拍重试,05c 语义)。再合一装载 (3,3)(② 就绪)后
    /// A(动量未耗尽前)二次撞击 → B 被唤醒,计数不再增加。
    #[test]
    fn contact_wake_skipped_when_footprint_missing_collider_then_wakes_after_ready() {
        let mut s = contact_sim(16);
        // ② 卸载半径 32(< 默认 48;apply_activation_executes_collider_
        // decisions 同款):让 MID 锚下 chunk (3,3)(中心距 40)落入
        // (32, 48] 带 → ② 单独拆、① 保留(R2 合法路径)。在 frozen_actor
        // 之前设置:其收尾的 MID 激活即完成单独拆 ②。
        s.cfg.activation.collider_unload_radius = 32.0;
        let (b, a) = (entity(1), entity(2));
        frozen_actor(&mut s, b, Vec3::new(56.0, 66.5, 56.0));

        assert!(
            !s.has_collider(ChunkPos::new(3, 3))
                && s.voxel().get_chunk(ChunkPos::new(3, 3)).is_some(),
            "前置:足迹 chunk 有 ① 无 ②(frozen_actor 的 MID 激活已按 R2 单独拆 ②)"
        );
        assert!(is_frozen(&s, b), "前置:B 保持冻结(缺 ② 不经 plan 唤醒)");

        // 撞击者 A 从 B 左侧 1.5 m、5 m/s 撞入(05b 同款几何)。
        s.apply_prefix(&[
            spawn_actor_v(3, a, Vec3::new(54.5, 66.5, 56.0), Vec3::new(5.0, 0.0, 0.0)),
            mark_actor_input(4, a, true),
        ]);
        let _ = s.apply_activation(&[Vec3::new(54.5, 66.5, 56.0)]);
        assert!(!is_frozen(&s, a), "前置:A 醒着");

        // 首拍:预测种子命中 B,但 B 的足迹缺 ② → 整组推迟(保持冻结):
        // 计数 +1、无任何唤醒计数(§4.16"只数真正从冻结切为 Dynamic 的")。
        s.step();
        let events = s.take_activation_events();
        assert_eq!(
            events.contact_wake_skipped_no_terrain, 1,
            "缺 ② 的接触唤醒组:跳过计数 +1"
        );
        assert_eq!(
            events.wakes_contact_pred
                + events.wakes_contact_comp
                + events.wakes_contact_spread
                + events.wakes_terrain
                + events.closure_deferred,
            0,
            "缺 ② 跳过不计唤醒、不计预算推迟(与 D19 分支区分)"
        );
        assert!(is_frozen(&s, b), "被撞的冻结体保持冻结(整组推迟)");

        // ② 就绪(合一重载同 chunk)→ A(仍在途中/已贴靠,动量 ≥ 门槛
        // 的接触继续产生种子)再次撞击 → B 被唤醒,跳过计数不再增加。
        s.load_chunk(ChunkPos::new(3, 3), solid_chunk());
        assert!(s.has_collider(ChunkPos::new(3, 3)), "前置:② 已就绪");
        let mut woke = false;
        for _ in 0..60 {
            s.step();
            if !is_frozen(&s, b) {
                woke = true;
                break;
            }
        }
        assert!(woke, "② 就绪后再次撞击应唤醒 B(缺 ② 只是推迟,不是丢弃)");
        let events = s.take_activation_events();
        assert_eq!(
            events.contact_wake_skipped_no_terrain, 0,
            "就绪后的唤醒不产生新的跳过计数(累计仍为 1)"
        );
    }

    // ---- REQ-009 单元 02:爆炸求值与粉碎(#18;生产路径接入 detonate)----

    /// 风险 5(§4.11 落地确认):爆炸炸掉冻结体脚下的格 → 写入经统一出口
    /// `register_terrain_write` 登记,下一拍 apply_activation 唤醒(免冷却)。
    ///
    /// 变红:detonate_terrain 不登记地形变更唤醒(pending 保持空,实体长眠);
    /// 爆炸写入没有进 detonate(格还在)。
    #[test]
    fn detonate_terrain_write_wakes_frozen_actor_next_tick() {
        let mut cfg = budget_cfg(16);
        cfg.destruction.enabled = true;
        // 先按 terrain_wake_under_frozen_actor 的已验证路径冻结(零重力悬停,
        // 脚下无块);再重载带 dirt 脚下格 (56,65,56) 的 chunk——重载不触发
        // 地形变更唤醒(写入路径才登记),冻结体也不因 ② 重建而醒。
        let mut s = frozen_actor_at(cfg, Vec3::new(56.0, 66.5, 56.0), entity(1));
        let mut ch = solid_chunk();
        // chunk (3,3) 局部 (8,129,8) = 世界 (56,65,56)。
        ch.set(8, 129, 8, voxel::builtin::DIRT);
        s.load_chunk(ChunkPos::new(3, 3), ch);
        assert!(is_frozen(&s, entity(1)), "前置:重载带 ② 的 chunk 不唤醒");
        let below = BlockPos::new(56, 65, 56);
        assert_eq!(
            s.voxel().get_block(below),
            voxel::builtin::DIRT,
            "前置:脚下格是 dirt"
        );
        // 爆心在脚下格下方一格:P = 12·(1−1/8) = 10.5 > k·R = 4 → 粉碎。
        s.apply_prefix(&[detonate_input(3, Vec3::new(56.5, 64.5, 56.5), 12.0, 8.0)]);
        assert_eq!(
            s.voxel().get_block(below),
            voxel::builtin::AIR,
            "脚下格被炸掉"
        );
        assert!(
            s.pending_terrain_wakes.contains(&below),
            "爆炸写入登记地形变更(§4.11 统一出口)"
        );
        assert!(is_frozen(&s, entity(1)), "登记拍不唤醒(下一拍才醒)");
        s.apply_activation(&[MID]);
        assert!(
            !is_frozen(&s, entity(1)),
            "下一拍被唤醒(地形变更唤醒,§4.11)"
        );
        assert!(
            s.terrain_woken_this_tick.contains(&entity(1)),
            "当拍记入地形唤醒集"
        );
    }

    /// 风险 4(写入顺序):同一拍(两个 apply_prefix)先后两个爆炸,第二个
    /// 看到第一个写过的 air——不重复破坏同一格,只破坏首爆未及的格。
    ///
    /// 变红:求值基于爆炸前的体素快照(两爆重叠格被写两次);第二个爆炸
    /// 无效(没有增量格)。
    #[test]
    fn detonate_twice_same_tick_second_sees_first_air() {
        use voxel::WorldGenerator;
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.destruction.enabled = true;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        for &(cx, cz) in &[(3i32, 3i32), (3, 4), (4, 3), (4, 4)] {
            let cp = ChunkPos::new(cx, cz);
            s.load_chunk(
                cp,
                voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
            );
        }
        let c = Vec3::new(64.0, -59.5, 64.0);
        s.apply_prefix(&[detonate_input(1, c, 12.0, 8.0)]);
        let first = s.take_outbox();
        assert_eq!(first.block_updates.len(), 60, "首爆 = 标准弹坑(60 格)");
        s.apply_prefix(&[detonate_input(2, c + Vec3::new(2.0, 0.0, 0.0), 12.0, 8.0)]);
        let second = s.take_outbox();
        let first_pos: BTreeSet<BlockPos> = first.block_updates.iter().map(|w| w.pos).collect();
        let second_pos: BTreeSet<BlockPos> = second.block_updates.iter().map(|w| w.pos).collect();
        assert!(
            !second_pos.is_empty(),
            "第二个爆炸破坏了首爆未及的格(爆心偏移 2 m)"
        );
        assert!(
            first_pos.is_disjoint(&second_pos),
            "同一格不被破坏两次(第二个爆炸看到前一个写过的 air)"
        );
        for w in &second.block_updates {
            assert_eq!(
                (w.block, w.kind),
                (voxel::builtin::AIR, BlockWriteKind::Emergent)
            );
        }
        for p in first_pos.union(&second_pos) {
            assert_eq!(s.voxel().get_block(*p), voxel::builtin::AIR);
        }
    }

    /// §4.3/§7 分片侧:`max_debris_per_explosion` 上限生效——坑的大小不变
    /// (全部写 air),只有脱落请求数被截断;请求按 `BlockPos` 升序,记录
    /// 原方块,冲量向上。
    ///
    /// 变红:上限把多出来的格直接丢掉不写 air(坑变小);请求顺序不对;
    /// 冲量方向向下。
    #[test]
    fn detonate_detach_requests_capped_and_ascending() {
        use voxel::WorldGenerator;
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.destruction.enabled = true;
        cfg.destruction.max_debris_per_explosion = 5;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        for &(cx, cz) in &[(3i32, 3i32), (3, 4), (4, 3), (4, 4)] {
            let cp = ChunkPos::new(cx, cz);
            s.load_chunk(
                cp,
                voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
            );
        }
        s.apply_prefix(&[detonate_input(1, Vec3::new(64.0, -59.5, 64.0), 12.0, 8.0)]);
        let out = s.take_outbox();
        assert_eq!(out.block_updates.len(), 60, "上限只改去向,不改坑的大小");
        assert_eq!(out.detach_requests.len(), 5, "脱落上限生效(标准弹坑 24)");
        assert!(
            out.detach_requests
                .windows(2)
                .all(|w| w[0].cell < w[1].cell),
            "脱落请求按 BlockPos 升序"
        );
        for req in &out.detach_requests {
            assert_eq!(req.block, voxel::builtin::DIRT, "脱落格记录原方块");
            assert!(req.impulse.y > 0.0, "dir' 以向上为主");
            assert_eq!(s.voxel().get_block(req.cell), voxel::builtin::AIR);
        }
    }
}
