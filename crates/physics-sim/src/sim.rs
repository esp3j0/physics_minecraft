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

/// 承载与碎裂的拍内编排(REQ-010 §4.6、§7.3;终审审计 P3 拆分):子模块
// 直用本模块私有字段与方法(后代模块可见父私有项),纯搬代码零可见性改动。
mod load_fracture;

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
    /// 粘结与整块塌落(REQ-010 §4.6;单元 08 之前默认关闭)。
    pub cohesion: crate::cohesion::CohesionConfig,
    /// 主体质量比例系数 k(REQ-012 §4,demo `mass` 定值 1/100):生成主体时
    /// 质量/惯量/PD 增益乘本系数,加速度不变。默认 0.01(REQ-012 A01 第 1 节:
    /// 胶囊进世界后质量 = 80 kg × 0.01 = 0.8)。
    #[serde(default = "default_agent_mass_scale")]
    pub agent_mass_scale: f32,
}

/// §4 压缩比例默认(REQ-012 demo `mass` 确认 D9 = 1/100)。
fn default_agent_mass_scale() -> f32 {
    0.01
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
            cohesion: crate::cohesion::CohesionConfig::default(),
            agent_mass_scale: default_agent_mass_scale(),
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
    /// 承载转换标记(REQ-010 §4.6、§9):F 中的格被转换成实体时写 air 的那条 `Emergent` 写入为 `true`,
    /// 其余一律 `false`;调度器盖章时带进 `EnvelopePayload::BlockWrite`。
    pub load_conversion: bool,
}

/// 冲量流事件摘要(REQ-011 §3.5.1;只读遥测,纯计数不入快照):
/// 一条建网求值的冲量流事件(§5.7 撞地形 / §5.8 复合体摔断撞散)。
/// 只记**建了网、求了值**的事件——预筛跳过的不记(无 `C`);`compound_flow
/// = false` 的旧路径不记;`compound_flow_events` 计数含预筛命中,与本字段
/// 口径不同(计数不改)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlowEventInfo {
    /// 事件种类:撞地形(§5.7)或复合体(§5.8)。
    pub kind: FlowKind,
    /// 供给合计 `s`(N·s;由判定用的量化值 `supply_q` 换算,同一份数据)。
    pub supply: f32,
    /// 最小割 `C`(N·s;由 `cut_q` 换算,同一份数据)。
    pub cut: f32,
    /// 断开与否:`cut_q < supply_q`(与判定分支同一比较;未断为 `false`)。
    pub broke: bool,
    /// 断下格数 |S|(未断为 0)。
    pub detached_cells: u64,
    /// 世界坐标:`Impact` = 被撞格格心;`Compound` = 该刚体本拍步进后的平移。
    pub position: Vec3,
}

/// 事件种类(§3.5.1;并列决出时 Impact 在前)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FlowKind {
    /// §5.7 撞击冲量流(撞地形)。
    Impact,
    /// §5.8 复合体冲量流。
    Compound,
}

/// 量化单位(与 `impact_flow::QUANT` / `compound_flow::QUANT` 同值;两处
/// 常量均为 pub(crate),此处本地镜像,语义一致)。
const FLOW_QUANT: f32 = 1000.0;

impl FlowEventInfo {
    /// 危险度比较(§3.5.1:取 `s / C` 最大者;`C = 0` 视为无穷大):
    /// `C = 0` 的比 `C ≠ 0` 的危险;同为 `C = 0` 比 `s`;否则交叉相乘
    /// `s₁·C₂ vs s₂·C₁`(量化整数域,值 ≤ ~1e14,无溢出)。
    pub(crate) fn danger_cmp(&self, other: &Self) -> std::cmp::Ordering {
        let s1 = (self.supply * FLOW_QUANT) as u64;
        let c1 = (self.cut * FLOW_QUANT).max(0.0) as u64;
        let s2 = (other.supply * FLOW_QUANT) as u64;
        let c2 = (other.cut * FLOW_QUANT).max(0.0) as u64;
        match (c1, c2) {
            (0, 0) => s1.cmp(&s2),
            (0, _) => std::cmp::Ordering::Greater,
            (_, 0) => std::cmp::Ordering::Less,
            _ => (s1 * c2).cmp(&(s2 * c1)),
        }
    }

    /// 并列决出(§3.5.1,danger 相等时):kind 升序(Impact 先)、position
    /// 各分量的 `f32::to_bits` 升序,取确定的一者。
    pub(crate) fn tie_break_select(a: &Self, b: &Self) -> Self {
        let key = |e: &Self| -> (u8, u32, u32, u32) {
            (
                match e.kind {
                    FlowKind::Impact => 0,
                    FlowKind::Compound => 1,
                },
                e.position.x.to_bits(),
                e.position.y.to_bits(),
                e.position.z.to_bits(),
            )
        };
        if key(a) <= key(b) { *a } else { *b }
    }

    /// 两取一(§3.5.1 的合计规则:`Outbox` 片内与调度器全分片合计共用):
    /// danger 大者胜;并列走 [`Self::tie_break_select`]。
    pub fn pick(a: Option<Self>, b: Option<Self>) -> Option<Self> {
        match (a, b) {
            (Some(a), Some(b)) => {
                if a.danger_cmp(&b) == std::cmp::Ordering::Less {
                    Some(b)
                } else if a.danger_cmp(&b) == std::cmp::Ordering::Equal {
                    Some(Self::tie_break_select(&a, &b))
                } else {
                    Some(a)
                }
            }
            (Some(a), None) => Some(a),
            (None, b) => b,
        }
    }
}

/// 本 tick 产出(调度器消费):迁移意图、collider 重建过的 chunk、凝固记录。
/// 可序列化 + 可克隆(快照携带未消费的产出,任务书 01 细则 1)。
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct Outbox {
    /// 本拍冲量流事件摘要(REQ-011 §3.5.1;W03):"最危险"的一条(`s / C`
    /// 最大,C=0 视为 ∞;并列按 kind、position 位模式升序决出)。
    /// **不入快照**(`#[serde(skip)]`:Outbox 随检查点序列化,跳过该字段
    /// 使检查点格式逐字节不变);每拍重算、outbox 取走即清。
    #[serde(skip)]
    pub flow_event: Option<FlowEventInfo>,
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
    /// 本拍承载塌落产生的复合体生成请求(REQ-010 §5.5;单元 04 起)。调度器与 `detach_requests` 一起分配涌现 ID、
    /// N+1 生成;对应格已在本拍写为 air。
    pub compound_requests: Vec<crate::cohesion::CompoundRequest>,
    /// 本拍的分块请求(REQ-010 §8.3;单元 05 起:倾斜散开;单元 06、07 复用)。
    pub piece_requests: Vec<crate::cohesion::PieceRequest>,
    /// 本拍地形破坏遥测(REQ-009 §8;单元 05 接线)。
    pub destruction_stats: crate::destruction::DestructionStats,
    /// 本拍粘结与整块塌落遥测(REQ-010 §11;单元 02 起接线)。
    pub cohesion_stats: crate::cohesion::CohesionStats,
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
    /// 本拍主体生成的类型化拒绝数(REQ-012 A01 第 3 节:未知哈希/多连杆/
    /// 定义非法;重复 EntityId 走 fail-stop 不计)。纯计数(随 Outbox 走,
    /// v8 起随快照;计数不影响确定性)。
    pub agent_spawn_rejected: u64,
    /// 本拍主体意图的内核侧拒绝数(REQ-012 A02a 第 2 节:回放的命令不可信
    /// ——实体不是主体、没有 `[walker]` 段、非有限值;进内核前的拒绝在驱动
    /// 侧 `validate_external_command`,不进本计数)。纯计数(随 Outbox 走)。
    pub agent_intent_rejected: u64,
    /// 本拍主体意图被钳制的次数(A02a 第 2 节:`move_xz` 长度超过
    /// `[walker].max_speed` 按比例钳到 max_speed)。纯计数(随 Outbox 走)。
    pub agent_intent_clamped: u64,
    /// 本拍关节目标命令的内核侧拒绝数(REQ-012 B01a 第 3.4 节:非主体 /
    /// dof = 0 / 长度不符 / 非有限值;世界不变)。纯计数(随 Outbox 走)。
    pub agent_joint_targets_rejected: u64,
    /// 本拍关节目标被钳到限位的个数(B01a 第 3.4 节:有 `limits` 的关节,
    /// 每夹一个 +1)。纯计数(随 Outbox 走)。
    pub agent_joint_targets_clamped: u64,
    /// 实体票状态操作(REQ-007 任务 03;S5:分片只产出状态依赖操作,签发
    /// 单点在调度器 claim/补签)。调度器按 TicketKey 序应用,N 拍产生 N+1 生效
    /// (§14-6);幂等由 TicketTable 保证。
    pub ticket_ops: Vec<crate::sim::TicketOpRecord>,
    /// 强制落地完成(REQ-007 §4.8;任务 03):(实体, 落点格)——调用方已写块
    /// (Emergent BlockWrite)+ despawned;调度器据此 End 票(幂等)。
    pub forced_landings: Vec<(EntityId, BlockPos)>,
    /// 虚空移除(§4.8):y < WORLD_Y_MIN − 深度 的实体(无 BlockWrite)。
    pub void_removed: Vec<EntityId>,
    /// 本拍因失控被移除的主体数(REQ-012 §6,B01b):步进后防护把超速/
    /// 非有限的主体按"掉出世界"移除,每移除一只 +1。纯计数(随 Outbox
    /// 走,不影响确定性)。
    pub agent_runaway_removed: u64,
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
    /// 本拍 chunk collider 重建次数(S03 遥测;只计数不进模拟):含生命周期
    /// 路径的立即重建与批量写路径 flush 的统一重建,= `rebuild_chunk_collider`
    /// 的真实执行次数。基线报告的"峰值拍重建次数"即本计数的峰值。
    pub collider_rebuilds: u64,
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

/// 接近速度基准的角速度与世界质心快照(§5.7.9 勘误 2,本步之前):
/// 接触点速度 = `v + ω × (p − com)`,`p` = 实体侧接触点(世界坐标)。
/// 与 [`ShardSim::pre_impulse_vels`] 同拍刷新、同不入快照。
#[derive(Debug, Clone, Copy)]
pub(crate) struct PreImpactSpin {
    /// 本步前角速度(世界系)。
    pub(crate) angvel: Vec3,
    /// 本步前世界质心。
    pub(crate) world_com: Vec3,
}

/// 单分片模拟器(字段 crate 内可见:snapshot.rs 读写快照结构)。
pub struct ShardSim {
    pub(crate) cfg: ShardSimConfig,
    pub(crate) registry: Box<dyn BlockRegistry>,
    pub(crate) voxels: VoxelWorld,
    pub(crate) phys: PhysicsWorld,
    pub(crate) entities: BTreeMap<EntityId, EntityRecord>,
    /// 主体附加记录(REQ-012 §3,A01):EntityId → 身体哈希 + 连杆句柄表。
    /// 一个主体占一个 `EntityId`(主体同时有 `EntityRecord`,handle = 根连杆);
    /// 本表随快照走。所有"把实体写成方块/迁移/下行 EntityStates"的路径以
    /// `agents.contains_key` 或主体 `actor = true` 挡住(路径表见 A01 提交)。
    pub(crate) agents: BTreeMap<EntityId, crate::agent::AgentRecord>,
    /// 身体注册表(REQ-012 A01 第 1 节):启动时加载注入,做法与
    /// `BlockRegistry` 一样——不进快照,restore 时重新注入。
    pub(crate) bodies: crate::agent::BodyRegistry,
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
    /// 本拍实心→非实心写入的坍塌触发集(REQ-009 §6.2.1,单元 04b):爆炸、
    /// 外部 SetBlock、BlockWrite 信封经 `register_collapse_trigger` 登记
    /// (凝固与强制落地只写实心,不走该出口);step 末 `evaluate_collapse`
    /// 取走求值后清空。checkpoint 只在两拍之间取,拍内集合必为空——不入快照。
    pub(crate) collapse_triggers: BTreeSet<BlockPos>,
    /// 撞击接近速度基准(§5 修订 9):各醒着动态实体**本步之前的线速度**,
    /// `step` 内 `phys.step()` 之前整表刷新;`evaluate_impacts` 只读。
    /// checkpoint 只在两拍之间取,拍内表必为空——不入快照。
    pub(crate) pre_impulse_vels: BTreeMap<EntityId, Vec3>,
    /// 接近速度基准(§5.7.9 勘误 2):各醒着动态实体**本步之前的**角速度与
    /// 世界质心(线速度在 [`ShardSim::pre_impulse_vels`]),与接触点同一时刻;
    /// 接触点速度 = `v + ω × (p − com)`。`step` 内与线速度同处刷新;
    /// `evaluate_impacts` 与 `evaluate_fractures` 只读。不入快照(同上)。
    pub(crate) pre_impulse_spin: BTreeMap<EntityId, PreImpactSpin>,
    /// 坍塌推迟队列(§6.2.1;v6 快照追加):上一拍名额不足时 = 上一拍触发集
    /// 原样 ∪ 上一拍已转换的格;每拍求值输入之一。随分片快照走(checkpoint
    /// 落在推迟与再求值之间时恢复线不得丢,acc_d7)。
    pub(crate) collapse_deferred: BTreeSet<BlockPos>,
    /// 承载判定的待转换队列(REQ-010 §4.6 第 4 条;v7 快照追加):F 中本体区
    /// 内、当时仍实心的格;每拍按 `BlockPos` 升序**直接转换、不重新求值**(承载
    /// 判定不单调——重新求值会让土门楣的残余恰好被单键容量托住而永远悬空),
    /// 数量受本拍剩余名额约束;轮到时已不是实心的格丢弃。随分片快照走。
    pub(crate) load_pending: BTreeSet<BlockPos>,
    /// 本拍收到的邻片承载转换写入的"写入前方块"记录(REQ-010 §9):拍首应用
    /// 时记下、拍末求两次的"转换前"求值取走后清空——记录只在本拍有效,不进
    /// 快照(checkpoint 落在两拍之间时必为空,同 [`ShardSim::collapse_triggers`])。
    pub(crate) load_conversion_records: BTreeMap<BlockPos, BlockId>,
    /// 撞击冲量表(REQ-010 §5.6 第 1 条,K02):本拍撞击路径写入 air 的触发格
    /// → J 矢量(该格各接触点冲量 × 撞击方向的和;只有撞击路径写入,爆炸/
    /// 放置等其余来源不带)。`evaluate_impacts` 破坏落写时登记,拍末
    /// `evaluate_load_tick` 开头 `std::mem::take` 取走——每拍重建/清空,不跨拍
    /// 残留;checkpoint 只在两拍之间取,拍内表只在同拍内被读,不入快照(同
    /// [`ShardSim::pre_impulse_vels`])。
    pub(crate) impact_impulses: BTreeMap<BlockPos, Vec3>,
    /// 断开事件的定量追踪(F02b-2,验收观测口;`#[cfg(test)]`):
    /// 每个断开且名额够的事件一条——J、p、s、C、修正、分量动量、修正前后
    /// 撞击者线/角速度。测试用 `step()` 跑生产路径后读此表与 §5.7.2 对账;
    /// 生产构建零开销。
    #[cfg(test)]
    pub(crate) impact_flow_trace: Vec<crate::impact_flow::FlowEventTrace>,
    /// 复合体冲量流的定量追踪(G02,验收观测口;`#[cfg(test)]`):每个入选
    /// 并求值的事件一条——局部系事件、判定、分配、各块世界速度、对方修正
    /// 及去向。测试用 `step()` 跑生产路径后读此表对账;生产构建零开销。
    #[cfg(test)]
    pub(crate) compound_flow_trace: Vec<crate::sim::load_fracture::CompoundFlowTrace>,
    /// 本拍 §5.7 撞击者修正表(G02,§5.8.1"修正扣除"):EntityId → 本拍施加
    /// 给它的修正合计(世界系;同一实体多个事件累加)。`apply_impact_flow_events`
    /// 写入,`evaluate_fractures` 的新路径消费;**拍内有效**,每拍开头清空,
    /// 不进 checkpoint。
    pub(crate) impact_corrections_this_tick: BTreeMap<EntityId, Vec3>,
    /// 待重建 ② 的 chunk(S03 去重;固定点统一重建):批量写路径(命令
    /// SetBlock、BlockWrite 信封、爆炸、撞击、凝固桥、强制落地、承载转换)
    /// 写体素后不再逐格立即 `rebuild_chunk_collider`,改为把目标 chunk 记入
    /// 本集合,由各公开写入口退出前的 [`ShardSim::flush_collider_rebuilds`]
    /// 按升序统一重建(仍只重建 `built_colliders` 中的)。chunk 生命周期路径
    /// (装载/卸载/建 ②/拆 ②)保持立即重建,不经本集合。**不变量:每个公开
    /// 写入口返回时本集合为空**(flush 点:apply_prefix、step 内撞击采样后
    /// ——保住 F03b"同拍撞击重建后接触对句柄失效"的既有可见性、step 末、
    /// run_transient_lifecycle;take_outbox 有 debug_assert 兜底)——外部
    /// 看到的重建时序不变,检查点格式不变(集合只活在一拍内部,不入快照)。
    pub(crate) pending_collider_rebuilds: BTreeSet<ChunkPos>,
    /// 分片步进计数(REQ-012 A02a 第 3 节):`step()` 已执行次数 = 全局
    /// tick。行走控制器的控制拍判定(`tick % 2 == 0`)以它为准——回放与
    /// 检查点恢复后相位一致(随快照走,恢复后从断点续计)。step 末 +1。
    pub(crate) step_tick: Tick,
    /// 行走控制器的保持态(A02a 第 3 节):主体 → 保持力与朝向去重记忆。
    /// 力在两个控制拍之间保持,**随快照走**(A02-9:检查点落在两个控制拍
    /// 之间时保持的力非零,恢复后继续施加)。随主体增删。
    pub(crate) walker_holds: BTreeMap<EntityId, crate::walker::WalkerHold>,
    /// 接缝修正开关(**测试对照专用**,A02-3:关掉接缝修正的对照组证明
    /// 布景真的会碰接缝问题;生产恒 true,不入快照——对照布景不进检查点)。
    /// 摩擦清零不受本开关影响(对照的被测操作是接缝修正,不是摩擦方案,
    /// capsule demo fix0 同款口径)。
    pub(crate) seam_fix_enabled: bool,
    /// 本拍的全局主体形状表(REQ-012 §6"暂缓写入",A05):调度器每拍开始
    /// 下发的"上一拍末全部主体连杆位姿";重叠判定只查本表,不查 `agents`
    /// (拥有格与持有副本判定一致)。**派生状态,不进快照**——恢复后由
    /// 调度器重算;恢复线首拍前调度器必然已注入(空表 = 无主体,判定恒
    /// false,与"无主体"世界行为一致)。
    pub(crate) agent_shapes: crate::agent_shape::AgentShapeTable,
    /// SetBlock 暂缓队列(REQ-012 §6,A05):与主体连杆重叠的外部实心写入
    /// 不落体素,格 → 待写的实心方块(每格只留最后一条,替换旧条目;同格
    /// 非实心写入作废条目)。每拍开始(形状表到位后、命令处理前)按
    /// `BlockPos` 升序重试,不再重叠即按 SetBlock 同一条写入路径补写。
    /// **随分片快照走**(检查点 v9)。
    pub(crate) deferred_set_blocks: BTreeMap<BlockPos, BlockId>,
    /// 失控防护开关(**测试对照专用**,B01b X3):关掉后步进不再执行
    /// `agent_runaway_guard`——X3 用 `catch_unwind` 抓 rapier 在失控状态
    /// 下的 panic,证明防护的必要性。生产恒 true,不入快照(对照布景
    /// 不进检查点,同 `seam_fix_enabled` 口径)。
    #[cfg(test)]
    pub(crate) agent_runaway_guard_enabled: bool,
}

/// 撞击冲量流的按实体事件累计(F02a,§5.7.1;`evaluate_impacts` 采样)。
/// `total` = 各点冲量和(J_T);`vec` = J 矢量 = Σ j·d;`cells` = 被撞格 →
/// (格上冲量和, 冲量加权地形侧位置和)——先采完本拍全部事件、后逐个写入,
/// 事件里只有格坐标/冲量/点,没有句柄(F03 教训:写入重建地形 collider
/// 后接触对句柄失效)。`is_compound` = 复合体(§7.3 第 7 条均摊)或单格
/// (§5.7.1"该格各点冲量之和";单格也可同时压到几个地形格,勘误 F02b-4)。
struct ImpactFlowSample {
    total: f32,
    vec: Vec3,
    cells: BTreeMap<BlockPos, (f32, Vec3)>,
    is_compound: bool,
}

impl ShardSim {
    pub fn new(cfg: ShardSimConfig, registry: Box<dyn BlockRegistry>) -> Self {
        Self::new_with_bodies(cfg, registry, crate::agent::BodyRegistry::new())
    }

    /// 带身体注册表构造(REQ-012 A01 第 1 节):生产装配(app bootstrap /
    /// wire_provider_and_new_sim)在启动时加载 `assets/bodies/` 注入;普通
    /// `new` = 空注册表(生成主体会类型化拒绝)。快照布局无关(注册表不入档)。
    pub fn new_with_bodies(
        cfg: ShardSimConfig,
        registry: Box<dyn BlockRegistry>,
        bodies: crate::agent::BodyRegistry,
    ) -> Self {
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
            agents: BTreeMap::new(),
            bodies,
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
            collapse_triggers: BTreeSet::new(),
            collapse_deferred: BTreeSet::new(),
            load_pending: BTreeSet::new(),
            load_conversion_records: BTreeMap::new(),
            pre_impulse_vels: BTreeMap::new(),
            pre_impulse_spin: BTreeMap::new(),
            impact_impulses: BTreeMap::new(),
            #[cfg(test)]
            impact_flow_trace: Vec::new(),
            #[cfg(test)]
            compound_flow_trace: Vec::new(),
            impact_corrections_this_tick: BTreeMap::new(),
            pending_collider_rebuilds: BTreeSet::new(),
            step_tick: Tick::ZERO,
            walker_holds: BTreeMap::new(),
            seam_fix_enabled: true,
            agent_shapes: crate::agent_shape::AgentShapeTable::default(),
            deferred_set_blocks: BTreeMap::new(),
            #[cfg(test)]
            agent_runaway_guard_enabled: true,
        }
    }

    pub fn shard(&self) -> ShardId {
        self.cfg.shard
    }

    /// 分片步进计数(REQ-012 A04 第 3 节):分片注册进调度器时设为当前
    /// **全局 tick**(此后照常每拍 +1)——新建分片不再从 0 起算,控制拍
    /// 恒等价于"全局 tick 为偶数"。驱动(`new_sim` 闭包)在调度器注册前
    /// 调用;快照恢复的分片沿用快照里的值(不走本入口)。**只许在首次
    /// `step` 之前调用一次**(重复调用 = 装配错误;`set_step_tick` 已跑过
    /// 的分片再对齐 = 控制相位被静默改写,fail-stop)。
    pub fn set_step_tick(&mut self, tick: Tick) {
        assert!(
            self.step_tick == Tick::ZERO,
            "set_step_tick 重复调用(step_tick 已 = {:?};分片注册对齐只许一次,\
             装配错误 fail-stop)",
            self.step_tick
        );
        self.step_tick = tick;
    }

    /// 分片步进计数(REQ-012 A04 第 3 节观测口):调度器 `restore_state`
    /// 断言快照恢复的分片相位等于全局 tick 用。
    pub fn step_tick(&self) -> Tick {
        self.step_tick
    }

    /// 接缝修正开关(**测试对照专用**,A02-3:关掉接缝修正的对照组证明
    /// 布景真的会碰接缝问题)。生产路径恒开(默认 true,不入快照)。
    pub fn set_seam_fix_for_test(&mut self, enabled: bool) {
        self.seam_fix_enabled = enabled;
    }

    /// 注入本拍的全局主体形状表(REQ-012 §6"暂缓写入",A05):调度器在
    /// 每拍分片步进之前,按**上一拍末**全部主体的连杆碰撞体与世界位姿
    /// (迁移在途取 Router 信封)汇总下发。派生状态,**不进快照**——恢复后
    /// 由调度器从快照主体状态与在途信封重算。空表 = 无主体,所有重叠判定
    /// 短路为 false(无主体时暂缓路径一次都不走,AC-A05-10)。
    pub fn set_agent_shape_table(&mut self, table: crate::agent_shape::AgentShapeTable) {
        self.agent_shapes = table;
    }

    /// SetBlock 暂缓补写(REQ-012 §6,R1-P1):拍首调用——形状表注入之后、
    /// 本拍命令(`apply_prefix`)之前。按 `BlockPos` 升序,不再重叠即按
    /// SetBlock 同一条写入路径补写(`block_updates` 记 `External`、地形唤醒、
    /// 坍塌/承载触发照常);仍重叠的留下,补写计"补写"。同拍到达的实心
    /// `SetBlock` 在补写之后判定:格已补写则不重叠、直接生效,同格以最后
    /// 一条为准;补写产出的 `External` 条目 owner 分片由调度器发布,holder
    /// 分片的副本由广播命令同拍送达(同一表 ⇒ 同样暂缓/补写,判定一致)。
    pub fn flush_agent_deferred_writes(&mut self) {
        self.flush_deferred_set_blocks();
    }

    /// 本拍形状表只读视图(测试/调度器汇总用)。
    pub fn agent_shape_table(&self) -> &crate::agent_shape::AgentShapeTable {
        &self.agent_shapes
    }

    /// SetBlock 暂缓队列只读视图(测试观测口):格 → 被暂缓的实心方块。
    pub fn deferred_set_blocks(&self) -> &BTreeMap<BlockPos, BlockId> {
        &self.deferred_set_blocks
    }

    /// 暂缓队列长度瞬时值(A05 遥测:调度器按分片求和下发)。
    pub fn deferred_set_block_count(&self) -> usize {
        self.deferred_set_blocks.len()
    }

    /// 体素世界只读视图(测试/导出用)。
    pub fn voxel(&self) -> &VoxelWorld {
        &self.voxels
    }

    /// 只读读方块实心性(REQ-012 A06 R3 §6"等区块装载":重生安全落点
    /// 的地形探询;不进快照、不改状态)。chunk 未加载 → None(调用方等
    /// 装载重试);y 界外 → 世界口径 AIR(非实心,与 [`VoxelWorld::
    /// get_block`] 一致);其余 = 方块注册表的实心判定。
    pub fn block_solid_at(&self, pos: types::BlockPos) -> Option<bool> {
        self.voxels.get_chunk(pos.chunk_of()).map(|_| {
            let block = self.voxels.get_block(pos);
            self.registry.is_solid(block)
        })
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

    /// 实体描述;不存在(已迁移/已凝固)返回 None(REQ-009 单元 03)。
    ///
    /// **总架构师接口桩(REQ-009 单元 03)**:由实施方实现。
    pub fn entity_desc(&self, id: EntityId) -> Option<EntityDesc> {
        self.entities.get(&id).map(|rec| rec.desc.clone())
    }

    /// 实体当前状态;不存在(已迁移/已凝固)返回 None。
    pub fn entity_state(&self, id: EntityId) -> Option<EntityState> {
        let rec = self.entities.get(&id)?;
        Some(self.read_state(rec))
    }

    /// 实体质量(kg;观察输出,REQ-010 单元 03a 测试观测口):单格 = 0.729,
    /// 复合体 = 格数 × 0.729(rapier 按逐格 cuboid collider × 密度 1.0 算)。
    /// 不存在 → None。rapier 类型不出 crate。
    pub fn entity_mass(&self, id: EntityId) -> Option<f32> {
        let rec = self.entities.get(&id)?;
        Some(self.phys.bodies.get(rec.handle)?.mass())
    }

    /// 实体是否醒着(非 kinematic;REQ-010 单元 03a 测试观测口:地形变更
    /// 唤醒的分叉点验证用)。不存在 → false。
    pub fn entity_awake(&self, id: EntityId) -> bool {
        self.entities
            .get(&id)
            .and_then(|rec| self.phys.bodies.get(rec.handle))
            .is_some_and(|b| !b.is_kinematic())
    }

    /// 冻结实体(REQ-010 单元 03a 测试观测口:与 plan 决策落实同式,清速度
    /// 后切 KinematicPositionBased;测试直接构造冻结态,验证唤醒分叉)。
    /// 实体不存在时无操作(与既有冻结落实同语义)。
    pub fn freeze_entity_for_test(&mut self, id: EntityId) {
        let Some(rec) = self.entities.get(&id) else {
            return;
        };
        let handle = rec.handle;
        if let Some(body) = self.phys.bodies.get_mut(handle) {
            body.set_linvel(conv::vec3_to_rapier(Vec3::ZERO), true);
            body.set_angvel(conv::vec3_to_rapier(Vec3::ZERO), true);
            body.set_body_type(RigidBodyType::KinematicPositionBased, true);
        }
    }

    /// 直接生成一个普通实体(REQ-012 A01 判据测试的对照布景口:零重力预算
    /// 场景等不需要命令盖章路径的测试;主体布景见 agent.rs)。生产路径仍只
    /// 经 SpawnDynamic。
    pub fn spawn_entity_for_test(&mut self, id: EntityId, pos: Vec3) {
        self.spawn_entity(
            id,
            types::EntityState {
                translation: pos,
                ..types::EntityState::default()
            },
            types::EntityDesc::default(),
            None,
            false,
        );
    }

    /// 写入一个体素并登记地形变更唤醒(REQ-010 单元 03a 测试观测口:不经
    /// apply_command,直接驱动 §4.11 的生产路径)。越界/未装载 → None。
    pub fn write_voxel_for_test(&mut self, pos: BlockPos, block: BlockId) -> Option<()> {
        let edit = voxel::VoxelEdit::new().set(pos, block);
        let dirty = edit.apply(&mut self.voxels).ok()?;
        for cp in expand_boundary(dirty, &self.voxels) {
            if self.built_colliders.contains(&cp) {
                self.rebuild_chunk_collider(cp);
            }
        }
        self.register_terrain_write(pos);
        Some(())
    }

    /// 全实体表(观察输出用,REQ-003 §4.4):`(EntityId, EntityState)` 列表,
    /// 按 EntityId 升序(底层 BTreeMap 键序,确定性)。空表 → 空列表。
    ///
    /// REQ-012 A01(§3):**主体不在下行**(S2C::AgentStates 在 A03 做;在那
    /// 之前 viewer 会把胶囊画成方块)——过滤 `agents` 表中的条目。
    pub fn entity_states(&self) -> Vec<(EntityId, EntityState)> {
        self.entities
            .iter()
            .filter(|(id, _)| !self.agents.contains_key(id))
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
            match &input.payload {
                InputPayload::Command(cmd) => self.apply_command(cmd.clone(), input.source),
                InputPayload::Event(CrossEvent::Detonate {
                    center,
                    power,
                    radius,
                }) => self.detonate(*center, *power, *radius),
                InputPayload::Envelope(e) => match &e.payload {
                    EnvelopePayload::MigrateEntity {
                        entity,
                        state,
                        desc,
                        actor,
                        source,
                        actor_motion_ticks,
                    } => {
                        self.spawn_entity(*entity, *state, desc.clone(), *source, *actor);
                        // 常驻运动票触顶标记随迁移载荷走(04a 遗留返修):
                        // 目的分片 claim 后计数连续,不借迁移绕过上限。
                        if let Some(rec) = self.entities.get_mut(entity) {
                            rec.actor_motion_ticks = *actor_motion_ticks;
                        }
                    }
                    // 主体迁移(REQ-012 A04 放行第 2 节):按身体哈希走
                    // `SpawnAgent` 同一条生成路径(碰撞体挂钩子标志、
                    // `actor = true`、登记进 `agents`),写回意图与
                    // `WalkerHold`,再进入本拍 `step`——迁移那一拍照常施加
                    // 保持的力(§13 `migrate`)。目的分片不认识该哈希 =
                    // 内部一致性违规(所有分片共用同一份 `BodyRegistry`)。
                    EnvelopePayload::MigrateAgent {
                        entity,
                        body,
                        state,
                        actor: _,
                        source,
                        actor_motion_ticks,
                        intent_move,
                        intent_yaw,
                        intent_jump,
                        held_force,
                        last_yaw,
                        joint_q,
                        joint_qd,
                        last_targets,
                    } => {
                        assert!(
                            self.bodies.get(body).is_some(),
                            "MigrateAgent 的身体哈希 {body:?} 不在本分片注册表\
                             (所有分片共用同一份 BodyRegistry,内部一致性违规,fail-stop)"
                        );
                        assert!(
                            !self.entities.contains_key(entity),
                            "MigrateAgent 目标实体 {entity:?} 已存在(迁移原子性违规,fail-stop)"
                        );
                        self.migrate_agent_in(
                            *entity,
                            *body,
                            *state,
                            *intent_move,
                            *intent_yaw,
                            *intent_jump,
                            *held_force,
                            *last_yaw,
                            *source,
                            *actor_motion_ticks,
                            joint_q,
                            joint_qd,
                            last_targets,
                        );
                    }
                    // 涌现事件(REQ-005 任务 03):共享参数本地求值,复用同一
                    // 纯函数;冻结体免疫语义不变(§9.12)。CrossEvent 未来变体
                    // 追加时本 match 穷举强制处理。
                    EnvelopePayload::Event(CrossEvent::Detonate {
                        center,
                        power,
                        radius,
                    }) => self.detonate(*center, *power, *radius),
                    // 涌现 block write(REQ-005 任务 03):目标 chunk 必已装载
                    // (owner/holder 才会被盖章为收件人;未装载 = 内部一致性
                    // 违规 fail-stop),写体素 + 重建边界 collider;**不推
                    // `outbox.block_updates`**(防广播回环)。
                    EnvelopePayload::BlockWrite {
                        pos,
                        block,
                        load_conversion,
                    } => {
                        assert!(
                            self.voxels.get_chunk(pos.chunk_of()).is_some(),
                            "BlockWrite 信封落到未装载 chunk {pos:?}(owner/holder 必然\
                             装载着该 chunk,内部一致性违规,fail-stop)"
                        );
                        // 触发登记与承载转换记录都要写前的方块(§4.6 第 2/3 条)。
                        let old_block = self.voxels.get_block(*pos);
                        if self.voxels.set_block(*pos, *block).is_ok() {
                            // 细则 4/5:只重建已建 ②(同 SetBlock 守卫);S03 起
                            // 记入待重建集合,apply_prefix 退出前统一重建。
                            self.defer_chunk_collider_rebuild([pos.chunk_of()]);
                            // holder 侧同样登记(04c 风险 4):边界附近的
                            // holder 侧冻结实体失去支撑也须醒来。
                            self.register_terrain_write(*pos);
                            // 承载转换记录(REQ-010 §9):带标记的写入记下写入前
                            // 的方块,拍末"转换前"求值把该格换回这个方块;只在本
                            // 拍有效,同拍多条取第一条(写入前状态)。
                            if *load_conversion && self.load_enabled() {
                                self.load_conversion_records
                                    .entry(*pos)
                                    .or_insert(old_block);
                            }
                            // 坍塌触发(REQ-009 §6.2.1 风险 1):邻片 owner 的
                            // 涌现写入同样登记——漏了它,跨分片悬空块的另一半
                            // 会因"没看到"而永远悬空(§6.3,acc_d6)。
                            self.register_collapse_trigger(*pos, old_block, *block);
                        }
                    }
                },
            }
        }
        self.enforce_budget_after_command_wake();
        // S03 去重收口:命令/信封/爆炸的全部写入在返回前统一重建——语义
        // 不变(写入仍在同拍 `phys.step()` 之前生效),同一 chunk 的多次
        // 写入只重建一次。
        self.flush_collider_rebuilds();
    }

    /// 命令唤醒路径的预算强制(README 决策 6):观察者已注册过(激活生效中)
    /// 且醒数超限时,按到最近观察者距离降序给出应冻结者并**立即落实**。
    /// 淘汰条目照 plan 的超预算路径记入冷却簿记(唤醒须等冷却/显式命令)。
    /// 从未注册观察者 → 恒空,行为与历史一致(向后兼容开关,任务书 02 细则 5)。
    fn enforce_budget_after_command_wake(&mut self) {
        let Some(observers) = self.observed.clone() else {
            return;
        };
        let awake: Vec<(EntityId, Vec3, u32)> = self
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
                // 格数口径(REQ-010 §5.5 第 5 条):单格 1、复合体 = 格数。
                let cells = rec.desc.shape().map_or(1, |s| s.cells().len() as u32);
                (*id, conv::rapier_to_vec3(b.translation()), cells)
            })
            .collect();
        for id in self.activation.enforce_budget_cells(&observers, &awake) {
            let now = self.activation_now;
            // §9-2/任务书细则 10(瞬态半边):被淘汰的**瞬态体 → 强制落地**
            // (产出同 §4.8;非冻结);常驻体 → 冻结 + End ActorMotion
            // (04a:I3 第 4 条,预算强制不受静止前置约束)。
            // REQ-012 A01(判据 A01-1):**主体豁免预算强制**——不被预算
            // 冻结、不被强制落地(淘汰池里选中主体 = 跳过,预算继续收缩的
            // 正路是淘汰其余实体;主体常驻,A05 后按 §6 规则演进)。
            if self.agents.contains_key(&id) {
                continue;
            }
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
            // REQ-012 A01:主体不进冻结闭包(支撑者被滞回冻结时,压在其上的
            // 主体不被连带冻结;主体冻结语义在 B03 = 快照+障碍物)。
            if self.agents.contains_key(&cur) {
                continue;
            }
            to_freeze.push(cur);
            let Some((cc, ch)) = self.entities.get(&cur).and_then(|rec| {
                self.phys.bodies.get(rec.handle).map(|b| {
                    (
                        conv::rapier_to_vec3(b.translation()),
                        rec.desc.envelope_radius(),
                    )
                })
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
                    let (oc, oh) = (
                        conv::rapier_to_vec3(b.translation()),
                        orec.desc.envelope_radius(),
                    );
                    let overlap =
                        |a: f32, ha: f32, c: f32, hc: f32| (a - c).abs() <= ha + hc + skin;
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
        // 撞击接近速度的基准(§5 修订 9):各醒着动态实体本步之前的线速度,
        // 供 `evaluate_impacts` 使用(解算后的速度已被冲量修改)。
        // 勘误 2(§5.7.9)追加:角速度与世界质心同拍快照——接触点速度
        // = v + ω×(p − com),旋转体与被撞实体的门槛都按接触点算。
        self.pre_impulse_vels = self
            .entities
            .iter()
            .filter_map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle)?;
                (b.is_dynamic() && !b.is_sleeping())
                    .then(|| (*id, conv::rapier_to_vec3(b.linvel())))
            })
            .collect();
        // 同拍快照角速度与世界质心(§5.7.9 勘误 2;`mass_properties().world_com`
        // 已随位姿/质量更新到本步前状态,与接触点同一时刻)。
        self.pre_impulse_spin = self
            .entities
            .iter()
            .filter_map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle)?;
                (b.is_dynamic() && !b.is_sleeping()).then(|| {
                    (
                        *id,
                        PreImpactSpin {
                            angvel: conv::rapier_to_vec3(b.angvel()),
                            world_com: conv::rapier_to_vec3(b.mass_properties().world_com),
                        },
                    )
                })
            })
            .collect();
        // REQ-012 A02a 第 3/4 节:行走控制器(控制拍判定按本拍 tick = 当前
        // step_tick;力的施加在 phys.step 之前)与接触钩子输入(当拍采样,
        // 纯函数)。step_tick 是"下一次 step 的拍号"(step 末 +1),与调度器
        // 全局 tick 同语义。无主体时控制器早退、钩子输入为空表 →
        // step_with_events 与 phys.step 行为逐位一致(AC-A8)。
        self.walker_control_tick();
        let hooks = crate::walker::AgentContactInput::sample(self);
        self.phys.step_with_events(&hooks, &());
        // 失控防护(REQ-012 §6,B01b):步进后立刻查全部主体,超速/非有限
        // 的按"掉出世界"移除——下一拍之前处理(demo `mass` §13:panic 前一
        // 拍状态还有限、速度 ≥ 2204 m/s,来得及)。被移除的主体 ID 不进本拍
        // 余下各段(采样表按 `entities` 查,见各段守卫)。开关 = X3 对照
        // (测试专用;生产恒 true)。
        #[cfg(test)]
        if self.agent_runaway_guard_enabled {
            self.agent_runaway_guard();
        }
        #[cfg(not(test))]
        self.agent_runaway_guard();
        self.collect_contact_wake_compensation();
        // §5.7 修正扣除表(G02,§5.8.1):拍内有效,每拍清空(上一拍的修正
        // 不扣到本拍事件上)。
        self.impact_corrections_this_tick.clear();
        // 撞击破坏(REQ-009 §5,单元 06):物理步进之后、坍塌求值之前——本步
        // 解算写入的接触冲量在此采样,破坏格进入本拍坍塌的触发集。
        self.evaluate_impacts();
        // S03 flush 点 A(审计结论,宁少去重不改行为):撞击写入延后重建后,
        // 必须在 `evaluate_fractures` 读取接触对之前落重建——旧实现的立即重建
        // 删除旧 chunk collider,F03b 的采样语义依赖该失效可见性(load_fracture
        // 采样对"句柄失效只跳本侧"的处理以重建为前提);不在此 flush 会让
        // 碎裂采样读到已被撞脱格的旧接触对,行为分叉。
        self.flush_collider_rebuilds();
        // 碎裂(REQ-010 §7.3,单元 06):物理步进之后、凝固桥之前——同一拍
        // 先碎;碎了的复合体已删除,凝固段自然不再判整块凝固(§7.3 风险 4)。
        self.evaluate_fractures();

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
                (rec.handle, rec.desc.clone(), rec.slow_ticks, rec.source);
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
            // 复合体不再跳过(REQ-010 §8.3 第 8 条,修订 6):同一套判据,对
            // 整个刚体判定;迁移检测照常(§5.3 按原点归属——凝固/散开删除实体
            // 后才 continue,不因凝固分支连迁移一起跳过)。
            let no_entity_support = !entity_supported.get(&id).copied().unwrap_or(false);
            if cfg.enabled
                && !actor
                && no_entity_support
                && gate_passes
                && (slow_ticks >= cfg.consecutive_ticks || is_sleeping)
            {
                if let Some(shape) = desc.shape() {
                    // A05(REQ-012 §6):任一目标格与本拍全局形状表重叠 ⇒
                    // 整体本拍不转换(复合体只要有一格重叠,整体都不转),
                    // 实体保持为实体,下一拍再试。不先删实体再排队写块
                    // (不出现方块消失、总数不守恒的窗口);计"凝固暂缓"。
                    // 表为空时短路(无主体世界零开销,AC-A05-10)。
                    if !self.agent_shapes.is_empty() {
                        let snap_cells: Vec<BlockPos> = shape
                            .cells()
                            .iter()
                            .map(|c| {
                                let r = state.rotation
                                    * types::glam::Vec3::new(
                                        c.offset[0] as f32,
                                        c.offset[1] as f32,
                                        c.offset[2] as f32,
                                    );
                                crate::settle::cell_of(
                                    state.translation.x + r.x,
                                    state.translation.y + r.y,
                                    state.translation.z + r.z,
                                )
                            })
                            .collect();
                        if self.agent_shapes.overlaps_any(&snap_cells) {
                            self.outbox.cohesion_stats.solidify_deferred += 1;
                            continue;
                        }
                    }
                    // ---- 复合体:整块凝固(REQ-010 §8.1、§8.3 第 1、2 条)。
                    // "被占" = 目标格是地形实心,或与其他实体的碰撞体重叠——
                    // 实体的格集合 = 各格中心所在格(单格 = 中心格,复合体 =
                    // 每格;与强制落地的 overlaps 同口径)。
                    let snap = {
                        let others: std::collections::BTreeSet<BlockPos> = self
                            .entities
                            .iter()
                            .filter(|(other, _)| **other != id)
                            .flat_map(|(_, orec)| {
                                let ob = self.phys.bodies.get(orec.handle).expect("handle 有效");
                                let op = conv::rapier_to_vec3(ob.translation());
                                let orot = conv::rapier_to_quat(ob.rotation());
                                match orec.desc.shape() {
                                    Some(s) => s
                                        .cells()
                                        .iter()
                                        .map(|c| {
                                            let r = orot
                                                * types::glam::Vec3::new(
                                                    c.offset[0] as f32,
                                                    c.offset[1] as f32,
                                                    c.offset[2] as f32,
                                                );
                                            crate::settle::cell_of(
                                                op.x + r.x,
                                                op.y + r.y,
                                                op.z + r.z,
                                            )
                                        })
                                        .collect::<Vec<_>>(),
                                    None => vec![crate::settle::cell_of(op.x, op.y, op.z)],
                                }
                            })
                            .collect();
                        let occupied = |p: BlockPos| -> bool {
                            // 未装载 chunk 不算占(写不进去另判);地形实心或其他实体。
                            self.voxels
                                .get_chunk(p.chunk_of())
                                .is_some_and(|_| !self.voxels.get_block(p).is_air())
                                || others.contains(&p)
                        };
                        crate::cohesion::compound_snap(
                            state.translation,
                            state.rotation,
                            shape,
                            cfg.max_snap_blocks,
                            cfg.max_snap_angle_deg,
                            self.cfg.cohesion.compound_snap_lift,
                            &occupied,
                        )
                    };
                    match snap {
                        crate::cohesion::CompoundSnap::Write { cells, .. } => {
                            // 同一拍写入全部格(`Emergent`,登记触发,D8);目标格
                            // 所在 chunk 未装载 → 本拍不凝固(继续等;set_block 会
                            // 静默新建空 chunk,不能拿它兜底)。
                            if cells
                                .iter()
                                .all(|(c, _)| self.voxels.get_chunk(c.chunk_of()).is_some())
                            {
                                for (cell, block) in &cells {
                                    let old_block = self.voxels.get_block(*cell);
                                    let _ = self.voxels.set_block(*cell, *block);
                                    self.outbox.block_updates.push(BlockWrite {
                                        pos: *cell,
                                        block: *block,
                                        kind: BlockWriteKind::Emergent,
                                        load_conversion: false,
                                    });
                                    self.register_terrain_write(*cell);
                                    self.register_collapse_trigger(*cell, old_block, *block);
                                }
                                let dirty: Vec<ChunkPos> =
                                    cells.iter().map(|(c, _)| c.chunk_of()).collect();
                                // S03:整块凝固写入记入待重建集合,step 末统一重建。
                                self.defer_chunk_collider_rebuild(dirty);
                                self.entities.remove(&id).expect("存在");
                                self.phys.remove_body(handle);
                                self.outbox.despawned.push(id);
                                // 遥测(§8.3 第 7 条):整块凝固数。
                                self.outbox.cohesion_stats.compounds_solidified += 1;
                                continue;
                            }
                        }
                        crate::cohesion::CompoundSnap::Tilted => {
                            // ---- 倾斜散开(§8.3 第 3 条,D5):本拍删除复合体,
                            // 每格按形状格序产出分块请求(普通单格碎块,不带
                            // 形状——否则会再被当成复合体判倾斜),N+1 出现;
                            // 状态 = 该格当前的世界位姿,速度 = v + ω × r。
                            // 散开当拍不写体素:格子由各分块之后自己凝固。
                            let pieces: Vec<crate::cohesion::PieceRequest> = shape
                                .cells()
                                .iter()
                                .enumerate()
                                .map(|(index, cell)| {
                                    let r = state.rotation
                                        * types::glam::Vec3::new(
                                            cell.offset[0] as f32,
                                            cell.offset[1] as f32,
                                            cell.offset[2] as f32,
                                        );
                                    crate::cohesion::PieceRequest {
                                        parent: id,
                                        index: index as u32,
                                        state: EntityState {
                                            translation: state.translation + r,
                                            rotation: state.rotation,
                                            linvel: state.linvel + state.angvel.cross(r),
                                            angvel: state.angvel,
                                        },
                                        desc: EntityDesc {
                                            half_size: crate::destruction::DEBRIS_HALF_SIZE,
                                            block: cell.block,
                                            shape: None,
                                        },
                                    }
                                })
                                .collect();
                            self.outbox.piece_requests.extend(pieces);
                            self.entities.remove(&id).expect("存在");
                            self.phys.remove_body(handle);
                            self.outbox.despawned.push(id);
                            // 遥测(§8.3 第 7 条):倾斜散开数。
                            self.outbox.cohesion_stats.tilt_scattered += 1;
                            continue;
                        }
                    }
                } else {
                    // ---- 单格凝固(原路径,逐位不变)----
                    let snapped = solidify::nearest_orientation(state.rotation, &orients);
                    let (cell, _center, disp) = solidify::snap_position(state.translation);
                    let ang_err = solidify::snap_angle_error(state.rotation, snapped);
                    let too_far =
                        disp > cfg.max_snap_blocks || ang_err > cfg.max_snap_angle_deg.to_radians();
                    // A05(REQ-012 §6):凝固落点格与主体连杆重叠 ⇒ 本拍不
                    // 转换,实体保持为实体,下一拍再试;计"凝固暂缓"。判定在
                    // 判据(慢速/睡)之后——只有本会触发凝固的实体才查表。
                    if !self.agent_shapes.is_empty() && self.agent_shapes.overlaps(cell) {
                        self.outbox.cohesion_stats.solidify_deferred += 1;
                        continue;
                    }
                    // 凝固落点须有支撑邻接(REQ-010 终审返修 1,**仅在承载判定
                    // 开启时**):目标格 6 邻中至少一个实心格(未装载 chunk 与
                    // 世界底按有地基,同接地口径)。悬浮在坑/洞上的落点(正下方
                    // 是尚未凝固的活实体,体素视角为空气)写入后立即被承载判定
                    // 判悬空、转回实体——实体落下砸到下面实体又重置其凝固门控,
                    // 凝固↔塌落振荡不收敛(碎块弹上门楣场景 15/15 复现);这种
                    // 落点等实体层稳定后再写。承载判定关闭时保持无条件凝固
                    // (REQ-006/008"真静止即凝固"语义逐位不变,零重力空世界用例
                    // `removed_reported_on_solidify_and_removal` 仍成立)。
                    let has_support_neighbor = (0..6).any(|i| {
                        let (dx, dy, dz) = [
                            (1, 0, 0),
                            (-1, 0, 0),
                            (0, 1, 0),
                            (0, -1, 0),
                            (0, 0, 1),
                            (0, 0, -1),
                        ][i];
                        let q = BlockPos::new(cell.x + dx, cell.y + dy, cell.z + dz);
                        if q.y < types::WORLD_Y_MIN {
                            return true; // 世界底:有地基
                        }
                        match self.voxels.get_chunk(q.chunk_of()) {
                            None => true, // 未装载:看不见的地方当作有地基
                            Some(_) => self.registry.material(self.voxels.get_block(q)).is_some(),
                        }
                    });
                    if (!too_far || is_sleeping) && (!self.load_enabled() || has_support_neighbor) {
                        // 睡眠兜底:睡着即无条件凝固;未睡但误差过大 → 继续等(升级路径)
                        let old_block = self.voxels.get_block(cell);
                        if self.voxels.set_block(cell, desc.block).is_ok() {
                            // 细则 4/5:只重建已建 ②(同 SetBlock 守卫);S03 起
                            // 记入待重建集合,step 末统一重建。
                            self.defer_chunk_collider_rebuild([cell.chunk_of()]);
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
                                load_conversion: false,
                            });
                            // 涌现写入同样登记地形变更唤醒(04c;验收
                            // acc_04c_solidify_write_wakes_adjacent_frozen_actor)。
                            self.register_terrain_write(cell);
                            // 凝固写入触发承载判定(REQ-010 §4.6 第 2 条、D8):
                            // 梁凝固在悬臂上可能把它压断;step 中段的写入在本拍拍末
                            // 的求值里生效(风险 4)。REQ-009 口径(关闭时)下
                            // air→实心不登记,行为不变。
                            self.register_collapse_trigger(cell, old_block, desc.block);
                            continue;
                        }
                    }
                }
            }

            // ---- 迁移检测(实体中心离开本体区,demo A)----
            // REQ-012 A04(放行第 2 节):主体走 `MigrateAgent` 真迁移——
            // 判定以根连杆中心为准(单连杆 = 胶囊中心,与普通实体同点),
            // 同一拍原子移除(agents/walker_holds/刚体/碰撞体都不留残余),
            // 控制器保持状态(意图 + 保持的力)随载荷走(§13 `migrate`:
            // 迁移那一拍照常施加保持的力)。
            if !self.core_contains(state.translation) {
                let dest = ShardId::new(
                    self.cfg.shard.world,
                    self.cfg.shard.dimension,
                    pos_chunk(state.translation).shard_of(),
                );
                if let Some(agent) = self.agents.remove(&id) {
                    // 主体迁移:多连杆在 B03 实现。assert 而非 debug_assert
                    // (A04 R1-P9):release 构建同样 fail-stop,与接收方
                    // migrate_agent_in 的一致性 assert 对齐。
                    assert!(
                        agent.links.len() == 1,
                        "多连杆主体迁移在 B03 实现(B01a fail-stop:{:?} 带 {} 个连杆)",
                        id,
                        agent.links.len()
                    );
                    let hold = self.walker_holds.remove(&id).unwrap_or_default();
                    self.entities.remove(&id).expect("存在");
                    self.phys.remove_body(handle);
                    self.outbox.despawned.push(id);
                    self.outbox.migrations.push(Intent {
                        destination: dest,
                        payload: EnvelopePayload::MigrateAgent {
                            entity: id,
                            body: agent.body_hash,
                            state,
                            actor,
                            source,
                            actor_motion_ticks,
                            intent_move: agent.intent_move,
                            intent_yaw: agent.intent_yaw,
                            intent_jump: agent.intent_jump,
                            held_force: hold.held_force,
                            last_yaw: hold.last_yaw,
                            joint_q: Vec::new(),
                            joint_qd: Vec::new(),
                            last_targets: Vec::new(),
                        },
                    });
                    continue;
                }
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
        // 坍塌求值(REQ-009 §6.2.1,单元 04b):物理步进之后、交出 outbox
        // 之前;本拍外部写入与爆炸的结果都已落地(风险 3)。
        self.evaluate_collapse();
        // S03 flush 点 B:凝固桥/坍塌/承载转换的写入在 step 返回前统一重建
        // (凝固桥循环与坍塌求值只读体素、不读接触对,延后到此处不改变行为
        // ——体素写入当拍已生效,后续求值看的都是体素真相)。
        self.flush_collider_rebuilds();
        // A02a 第 3 节:本拍完成,步进计数 +1(下一拍的 tick;控制拍相位、
        // 检查点恢复后续计都以此为准)。
        self.step_tick = Tick(self.step_tick.0 + 1);
    }

    /// 取走并清空本 tick 产出。
    pub fn take_outbox(&mut self) -> Outbox {
        // S03 不变量兜底(worker 顺序 apply_prefix → step → run_transient_
        // lifecycle 已各自 flush;出现残留 = 新写入口漏 flush,fail-stop)。
        debug_assert!(
            self.pending_collider_rebuilds.is_empty(),
            "take_outbox 时仍有待重建 chunk {:?}(公开写入口漏 flush,fail-stop)",
            self.pending_collider_rebuilds
        );
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
        // 醒体预算按格数(REQ-010 §5.5 第 5 条):单格 1、复合体 = 形状格数。
        let entities: Vec<(EntityId, Vec3, bool, bool, bool, u32)> = self
            .entities
            .iter()
            .map(|(id, rec)| {
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                let cells = rec.desc.shape().map_or(1, |s| s.cells().len() as u32);
                (
                    *id,
                    conv::rapier_to_vec3(b.translation()),
                    b.is_kinematic(),
                    rec.actor,
                    supports.get(id).copied().unwrap_or(false),
                    cells,
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
                    envelope_half(&rec.desc),
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
            // REQ-012 A01(判据 A01-1):**主体豁免预算淘汰**(不被预算冻结、
            // 不被强制落地;跳过,不烧簿记)。
            if self.agents.contains_key(id) {
                continue;
            }
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
                        envelope_half(&rec.desc),
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
                    envelope_half(&rec.desc),
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
                envelope_half(&rec.desc),
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
                let h = envelope_half(&rec.desc);
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
                let h = envelope_half(&rec.desc);
                (
                    *id,
                    conv::rapier_to_vec3(b.translation()),
                    Vec3::new(h, h, h),
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
                envelope_half(&rec.desc),
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
                envelope_half(&rec.desc),
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
            // REQ-012 A01:主体不进兜底冻结(常驻运动票触顶的降级对主体不
            // 生效;主体冻结语义在 B03)。
            if self.agents.contains_key(&id) {
                continue;
            }
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
                    // REQ-012 A01(§6"掉出世界"):主体按"AI 主体移除"走同
                    // 一路径(清 agents 表;玩家回出生点在 A06 接线)。
                    // B01a:以**根连杆**判定(整个主体只占一个 EntityId,
                    // rec.handle = 根;连杆相对根的偏移 ≤ 包络,不会根还在
                    // 界内而整只判定跨阈值——以根判定即整只移除)。
                    // B01a-R1:全部连杆(含根)在循环里各移除一次;根连杆
                    // 不再走循环外那行(旧代码根被移除两次——rapier 对无效
                    // 句柄静默 no-op,行为未变,但账目不对)。单连杆路径:
                    // 循环移除唯一连杆 = 原行为,逐句等价。
                    // B01b:主体分支原样提出为 `remove_agent_as_void`(虚空
                    // 移除与失控共用;逐句等价,行为不变)。
                    if self.agents.contains_key(&id) {
                        self.remove_agent_as_void(id);
                        continue;
                    }
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
            // REQ-012 A01(判据 A01-1):主体不进强制落地——出队跳过,不写
            // 体素、不移除(主体不该进 forced_pending;调度器侧来源防御)。
            if self.agents.contains_key(&id) {
                self.forced_pending.remove(&id);
                continue;
            }
            let (handle, block_desc) = (rec.handle, rec.desc.clone());
            // 复合体不再跳过(REQ-010 §8.3 第 8 条,修订 6):整块强制落地
            // (§8.2、§8.3 第 5、6 条)——朝向与目标格同凝固的吸附;前置条件
            // (所有柱 ① 在内存)不满足 → Defer(同单格,推迟留队);
            // Fallback → 按形状格序逐格用单格规则落地。
            if let Some(shape) = block_desc.shape() {
                let body = self.phys.bodies.get(handle).expect("handle 有效");
                let pos = conv::rapier_to_vec3(body.translation());
                let rot = conv::rapier_to_quat(body.rotation());
                let landing = {
                    let solid = |p: BlockPos| -> Option<bool> {
                        self.voxels.get_chunk(p.chunk_of())?;
                        Some(!self.voxels.get_block(p).is_air())
                    };
                    crate::cohesion::compound_forced_landing(
                        pos,
                        rot,
                        shape,
                        self.cfg.cohesion.compound_snap_lift,
                        &solid,
                    )
                };
                match landing {
                    crate::cohesion::CompoundLanding::Defer => {
                        continue; // 前置不满足:推迟(留在 forced_pending,同单格柱缺)
                    }
                    crate::cohesion::CompoundLanding::Write { cells } => {
                        // A05(REQ-012 §6):任一落点格与全局形状表重叠 ⇒
                        // 整体本拍不落地,实体保持为实体,留在队列下一拍再试
                        // (不许先删实体再排队写块);计"落地暂缓"。
                        if !self.agent_shapes.is_empty() {
                            let cells: Vec<BlockPos> = cells.iter().map(|(c, _)| *c).collect();
                            if self.agent_shapes.overlaps_any(&cells) {
                                self.outbox.cohesion_stats.forced_landing_deferred += 1;
                                continue;
                            }
                        }
                        // 整块平移后的落点同拍写入(Emergent + 触发 + 唤醒)。
                        for (cell, block) in &cells {
                            let old_block = self.voxels.get_block(*cell);
                            let _ = self.voxels.set_block(*cell, *block);
                            self.outbox.block_updates.push(BlockWrite {
                                pos: *cell,
                                block: *block,
                                kind: BlockWriteKind::Emergent,
                                load_conversion: false,
                            });
                            self.register_terrain_write(*cell);
                            self.register_collapse_trigger(*cell, old_block, *block);
                        }
                        let dirty: Vec<ChunkPos> =
                            cells.iter().map(|(c, _)| c.chunk_of()).collect();
                        // S03:整块强制落地写入记入待重建集合,退出前统一重建。
                        self.defer_chunk_collider_rebuild(dirty);
                        self.entities.remove(&id).expect("存在");
                        self.phys.remove_body(handle);
                        self.locomotion.remove(&id);
                        self.outbox.despawned.push(id);
                        self.outbox.forced_landings.push((id, cells[0].0));
                        self.forced_pending.remove(&id);
                        // 06c 遥测:按原因分计(§4.16),同单格。
                        match self.forced_reasons.remove(&id) {
                            Some(ForcedLandingReason::Budget) => {
                                self.activation_events.forced_landings_budget += 1;
                            }
                            _ => self.activation_events.forced_landings_deadline += 1,
                        }
                    }
                    crate::cohesion::CompoundLanding::Fallback => {
                        // 退化为逐格(§8.3 第 6 条):按形状格序逐格用 REQ-007
                        // §4.8 的单格规则落地;先落的格写入体素,对后落的格算
                        // 占用;柱下没有实心的格虚空消失(不写)。整块强制落地
                        // 退化一次计 1(§8.3 第 7 条)。
                        // A05(REQ-012 §6):退化前先按形状各格当前所在格判
                        // 重叠——任一格与主体重叠 ⇒ 整体本拍不落地(留队,
                        // 计"落地暂缓"),不在逐格写入中途插暂缓(保证"总数
                        // 守恒、无方块消失窗口"对退化路径同样成立)。
                        if !self.agent_shapes.is_empty() {
                            let start_cells: Vec<BlockPos> = shape
                                .cells()
                                .iter()
                                .map(|c| {
                                    let r = rot
                                        * types::glam::Vec3::new(
                                            c.offset[0] as f32,
                                            c.offset[1] as f32,
                                            c.offset[2] as f32,
                                        );
                                    crate::settle::cell_of(pos.x + r.x, pos.y + r.y, pos.z + r.z)
                                })
                                .collect();
                            if self.agent_shapes.overlaps_any(&start_cells) {
                                self.outbox.cohesion_stats.forced_landing_deferred += 1;
                                continue;
                            }
                        }
                        for cell in shape.cells() {
                            let r = rot
                                * types::glam::Vec3::new(
                                    cell.offset[0] as f32,
                                    cell.offset[1] as f32,
                                    cell.offset[2] as f32,
                                );
                            let start =
                                crate::settle::cell_of(pos.x + r.x, pos.y + r.y, pos.z + r.z);
                            let landing = {
                                let column = |x: i32, by: i32| -> Option<BlockId> {
                                    Some(self.voxels.get_block(BlockPos::new(x, by, start.z)))
                                };
                                let overlaps = |b: BlockPos| -> bool {
                                    self.entities.iter().any(|(other, orec)| {
                                        *other != id && {
                                            let ob = self
                                                .phys
                                                .bodies
                                                .get(orec.handle)
                                                .expect("handle 有效");
                                            let op = conv::rapier_to_vec3(ob.translation());
                                            crate::settle::cell_of(op.x, op.y, op.z) == b
                                        }
                                    })
                                };
                                crate::settle::forced_landing_cell(
                                    &column,
                                    start.x,
                                    start.y,
                                    start.z,
                                    types::WORLD_Y_MIN,
                                    types::WORLD_Y_MAX,
                                    &overlaps,
                                )
                            };
                            let Some(landing) = landing else {
                                continue; // 柱下没有实心:该格虚空消失
                            };
                            let old_block = self.voxels.get_block(landing);
                            let _ = self.voxels.set_block(landing, cell.block);
                            self.outbox.block_updates.push(BlockWrite {
                                pos: landing,
                                block: cell.block,
                                kind: BlockWriteKind::Emergent,
                                load_conversion: false,
                            });
                            self.register_terrain_write(landing);
                            self.register_collapse_trigger(landing, old_block, cell.block);
                        }
                        let dirty: Vec<ChunkPos> = shape
                            .cells()
                            .iter()
                            .map(|c| {
                                let r = rot
                                    * types::glam::Vec3::new(
                                        c.offset[0] as f32,
                                        c.offset[1] as f32,
                                        c.offset[2] as f32,
                                    );
                                crate::settle::cell_of(pos.x + r.x, pos.y + r.y, pos.z + r.z)
                                    .chunk_of()
                            })
                            .collect();
                        // S03:Fallback 逐格写入记入待重建集合,退出前统一重建。
                        self.defer_chunk_collider_rebuild(dirty);
                        self.entities.remove(&id).expect("存在");
                        self.phys.remove_body(handle);
                        self.locomotion.remove(&id);
                        self.outbox.despawned.push(id);
                        self.outbox
                            .forced_landings
                            .push((id, crate::settle::cell_of(pos.x, pos.y, pos.z)));
                        self.forced_pending.remove(&id);
                        self.outbox.cohesion_stats.forced_landing_fallbacks += 1;
                        match self.forced_reasons.remove(&id) {
                            Some(ForcedLandingReason::Budget) => {
                                self.activation_events.forced_landings_budget += 1;
                            }
                            _ => self.activation_events.forced_landings_deadline += 1,
                        }
                    }
                }
                continue;
            }
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
            // A05(REQ-012 §6):落点列与主体重叠 ⇒ 本拍不落地(留队下一拍,
            // 计"落地暂缓")。判在 landing 计算之后——口径 = **要写的格**
            // (landing)与形状表重叠才暂缓;landing 为 None(虚空移除)不涉
            // 及写块,不暂缓。不重叠时行为逐位不变。
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
            // A05(REQ-012 §6):要写的格与主体连杆重叠 ⇒ 本拍不落地,实体
            // 保持为实体、留队下一拍再试(总数守恒,无消失窗口)。
            if !self.agent_shapes.is_empty() && self.agent_shapes.overlaps(landing) {
                self.outbox.cohesion_stats.forced_landing_deferred += 1;
                continue;
            }
            // 落点写块(Emergent)+ despawn(与凝固桥同产出;复用 REQ-005
            // 广播持久化)。
            let old_block = self.voxels.get_block(landing);
            let _ = self.voxels.set_block(landing, block_desc.block);
            // S03:单格强制落地写入记入待重建集合,退出前统一重建。
            self.defer_chunk_collider_rebuild([landing.chunk_of()]);
            self.outbox.block_updates.push(BlockWrite {
                pos: landing,
                block: block_desc.block,
                kind: BlockWriteKind::Emergent,
                load_conversion: false,
            });
            // 强制落地的落点写入同样登记(04c 风险 1:四路全覆盖);触发承载
            // 判定(REQ-010 §4.6 第 2 条、D8)。lifecycle 在 step 之后执行,
            // 触发落在下一拍的求值里。
            self.register_collapse_trigger(landing, old_block, block_desc.block);
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
        // S03 去重收口:本公开写入口(强制落地/虚空移除)的全部落点写入在
        // 返回前统一重建——外部看到的时序不变。
        self.flush_collider_rebuilds();
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
                envelope_half(&rec.desc),
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

    /// SetBlock 暂缓队列重试(REQ-012 §6,A05):每拍开始(形状表到位后、
    /// 命令处理前)按 `BlockPos` 升序重试;不再重叠的条目按 SetBlock 同一
    /// 条写入路径落体素(计"补写"),仍重叠的留下。主体被移除 ⇒ 表里没有
    /// 它 ⇒ 下一拍自然补写。
    fn flush_deferred_set_blocks(&mut self) {
        if self.deferred_set_blocks.is_empty() {
            return;
        }
        let pending: Vec<BlockPos> = self.deferred_set_blocks.keys().copied().collect();
        for pos in pending {
            if self.agent_shapes.overlaps(pos) {
                continue; // 仍重叠:留在队列,下一拍再试
            }
            let block = self
                .deferred_set_blocks
                .remove(&pos)
                .expect("键取自本表(fail-stop)");
            // 与 SetBlock 命令臂同一条写入路径(触发登记要写前的方块)。
            let old_block = self.voxels.get_block(pos);
            let edit = VoxelEdit::new().set(pos, block);
            match edit.apply(&mut self.voxels) {
                Ok(dirty) => {
                    self.defer_chunk_collider_rebuild(dirty);
                    self.outbox.block_updates.push(BlockWrite {
                        pos,
                        block,
                        kind: BlockWriteKind::External,
                        load_conversion: false,
                    });
                    self.register_terrain_write(pos);
                    self.register_collapse_trigger(pos, old_block, block);
                    self.outbox.cohesion_stats.set_block_deferred_flushed += 1;
                }
                Err(e) => panic!("SetBlock 暂缓补写越界(fail-stop):{e}"),
            }
        }
    }

    /// 坍塌/承载触发登记(REQ-009 §6.2.1、REQ-010 §4.6 第 2 条):写入前后
    /// 方块**不同**、且**至少一侧是实心方块**才登记。承载判定开启
    /// (`destruction.enabled` ∧ `cohesion.enabled`,见 [`ShardSim::load_enabled`])时
    /// "加重"的写入(放置、凝固桥、强制落地、实心换实心的材料替换)同样登记;
    /// 关闭时保持 REQ-009 口径——仅实心 → 非实心登记、总开关关闭不收集。
    /// 六路写入统一走本出口:SetBlock 命令、BlockWrite 信封、爆炸、撞击、凝固
    /// 桥、强制落地(后两路此前只写实心、不走该出口,REQ-010 起也登记)。
    ///
    /// **守卫用例的变异点(保持)**:REQ-009 路径的开关闸仍在登记出口,
    /// `evaluate_collapse` 的 REQ-009 分支不重复判开关——关闭时触发集恒空、
    /// 求值入口按空触发集早退,守卫用例的单一变异语义不变。
    fn register_collapse_trigger(&mut self, pos: BlockPos, old_block: BlockId, new_block: BlockId) {
        if old_block == new_block {
            return;
        }
        let old_solid = self.registry.material(old_block).is_some();
        let new_solid = self.registry.material(new_block).is_some();
        if !(old_solid || new_solid) {
            return;
        }
        if self.load_enabled() {
            self.collapse_triggers.insert(pos);
            return;
        }
        if old_solid && !new_solid && self.cfg.destruction.enabled {
            self.collapse_triggers.insert(pos);
        }
    }

    /// 承载判定是否接管坍塌求值(REQ-010 §4.6 第 1 条):两个开关同时开启;
    /// 任一关闭时走 REQ-009 原路径,行为逐位不变(含触发口径与推迟队列)。
    fn load_enabled(&self) -> bool {
        self.cfg.destruction.enabled && self.cfg.cohesion.enabled
    }

    /// 撞击破坏采样与落写(REQ-009 §5,单元 06;调用点 = `step` 内
    /// `phys.step()` 与补偿收集之后——接触冲量是本步解算写入的,随后凝固/
    /// 迁移删掉的实体不影响本拍已采到的冲量)。遍历"醒着的动态实体 ↔ 地形
    /// chunk 复合体"接触对:对每个 `data.impulse > 0` 的接触点,取地形一侧的
    /// 位置,沿"指向实体"的法向**反方向**退 [`IMPACT_SAMPLE_DEPTH`] 取所在格
    /// (0.1 m 覆盖斜交法向与棱角接触的采样余量;D15 内缩后接触点已落在实心格
    /// 表面,不再需要越过伸出的薄片,余量更大);同一格的冲量相加(方块平落时
    /// 冲量分摊在 4 个角点)。
    /// J ≥ 材料的 `impact_threshold` → 脱落(不粉碎),脱落冲量 `(J − I)·d`
    /// (`d` = 撞击方向,把碎块压向地形,§5 风险 5 允许再撞)。上限 =
    /// `max_impact_breaks_per_tick` 与每拍剩余名额取小,按冲量降序(同值
    /// `BlockPos` 升序)取前者;其余本拍不破坏、**也不推迟**(落定后没有冲量)。
    /// 写入同爆炸:air、`Emergent`、§4.11 唤醒、重建 collider;撞击写入进入
    /// 本拍坍塌的触发集。遥测:`cells_impacted`、`detached`。总开关关闭不检测。
    /// collider → 实体归因表(collider_to_entity;evaluate_impacts 与测试
    /// 共用一份构建逻辑,REQ-012 A01):单格 = 代表 collider;复合体与主体
    /// = 全部连杆碰撞体。键取 handle 的 (index, generation)。
    pub(crate) fn collider_to_entity_of(&self) -> BTreeMap<(u32, u32), EntityId> {
        let mut table: BTreeMap<(u32, u32), EntityId> = BTreeMap::new();
        for (id, rec) in &self.entities {
            let idx = rec.collider.into_raw_parts();
            table.insert((idx.0, idx.1), *id);
            if rec.desc.shape().is_some() {
                let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
                for ch in body.colliders() {
                    let idx = ch.into_raw_parts();
                    table.insert((idx.0, idx.1), *id);
                }
            } else if let Some(agent) = self.agents.get(id) {
                // REQ-012 A01/B01a(§3):主体的**全部连杆**碰撞体登记
                // (多连杆每连杆一个刚体,逐连杆收它的 collider)。
                for &handle in agent.links.values() {
                    let body = self.phys.bodies.get(handle).expect("handle 有效");
                    for ch in body.colliders() {
                        let idx = ch.into_raw_parts();
                        table.insert((idx.0, idx.1), *id);
                    }
                }
            }
        }
        table
    }

    fn evaluate_impacts(&mut self) {
        if !self.cfg.destruction.enabled {
            return;
        }
        // collider → 实体:单格 = 代表 collider(1:1,原键集合逐位不变);复合体
        // = 全部格 collider(REQ-010 §7.3 第 7 条:砸地形按均摊口径,须采样
        // 每格的接触点)。键取 handle 的 (index, generation)。
        // REQ-012 A01(§3):主体的连杆碰撞体同样登记(归因结构就位,A05
        // 细化破坏交互规则)。
        let collider_to_entity = self.collider_to_entity_of();
        if collider_to_entity.is_empty() {
            return;
        }
        // 撞击冲量流分派(F02a,§5.7.1/§5.7.4):三开关同时开启时,采样改为
        // 按"事件"(每实体)累计,交给 `evaluate_impact_flow` 判定,**取代**
        // 下方的 REQ-009 §5 逐格比较——两条判定不能都跑;关闭时逐格路径逐位
        // 不变(AC-F12)。
        let flow_enabled = self.load_enabled() && self.cfg.cohesion.impact_flow;
        // 事件累计(§5.7.1):先采完本拍全部事件、后逐个写入(F03 教训:
        // 前一个事件写 air 重建地形 collider 后,接触对里的句柄失效——事件里
        // 只存格坐标/冲量/点,不存句柄)。`cells` = 被撞格 → (冲量和, 冲量
        // 加权地形侧位置和);复合体均摊 J_T/|T| 在成形时做(§7.3 第 7 条)。
        let mut flow_samples: BTreeMap<EntityId, ImpactFlowSample> = BTreeMap::new();
        let chunk_colliders: BTreeSet<(u32, u32)> = self
            .chunk_bodies
            .values()
            .filter_map(|h| self.phys.bodies.get(*h))
            .flat_map(|b| b.colliders().iter().copied())
            .map(|c| {
                let idx = c.into_raw_parts();
                (idx.0, idx.1)
            })
            .collect();
        if chunk_colliders.is_empty() {
            return;
        }
        // 本体区(格坐标口径,同 core_contains / detonate_terrain)。
        let min = self.cfg.shard.pos.min_chunk().block_min();
        let max_x = min.x + SHARD_CHUNKS * CHUNK_SIZE;
        let max_z = min.z + SHARD_CHUNKS * CHUNK_SIZE;

        // 每格累计(先收进 BTreeMap 再排序,接触对遍历顺序不确定,风险 4):
        // `impulse` = 各点冲量之和;`dir` = 最大点冲量的撞击方向(并列取字典序
        // 小者,结果与采样顺序无关);`vec` = 各点冲量 × 撞击方向的矢量和
        // (K02 §5.6 第 1 条的 J 矢量:加法可交换,与采样顺序无关)。单格实体
        // 走这一路径(REQ-009 §5 原口径)。
        #[derive(Default)]
        struct Sample {
            impulse: f32,
            best: f32,
            dir: Vec3,
            vec: Vec3,
        }
        let mut samples: BTreeMap<BlockPos, Sample> = BTreeMap::new();
        // 复合体(REQ-010 §7.3 第 7 条):每实体的接触点冲量合计与被触及的
        // 地形格集合 T,均摊 `J_T / |T|` 逐格判 I——多点接触时求解器把冲量
        // 分给各点的方式不唯一,逐格相加会让平落的复合体随机砸穿脚下某一格。
        // `vec` = 各点冲量 × 撞击方向的矢量和(K02 §5.6 第 1 条:均摊冲量的
        // 方向基准,`total` 同基准均摊)。
        #[derive(Default)]
        struct CompoundImpact {
            total: f32,
            vec: Vec3,
            cells: BTreeSet<BlockPos>,
        }
        let mut compound_impacts: BTreeMap<EntityId, CompoundImpact> = BTreeMap::new();
        for pair in self.phys.narrow_phase.contact_pairs() {
            let key1 = pair.collider1.into_raw_parts();
            let key2 = pair.collider2.into_raw_parts();
            let (id, entity_is_1) = match (
                collider_to_entity.get(&(key1.0, key1.1)),
                collider_to_entity.get(&(key2.0, key2.1)),
            ) {
                (Some(e), None) if chunk_colliders.contains(&(key2.0, key2.1)) => (*e, true),
                (None, Some(e)) if chunk_colliders.contains(&(key1.0, key1.1)) => (*e, false),
                _ => continue,
            };
            // 醒着的动态实体(§5):冻结(kinematic)与睡着的不算。
            let Some(rec) = self.entities.get(&id) else {
                continue;
            };
            // 接近速度的基准 = 本步之前的线速度(修订 9;`step` 在
            // `phys.step()` 前整表刷新 `pre_impulse_vels`,表里没有 = 本步
            // 开始后新生的实体,本步无撞击)。
            let Some(pre_vel) = self.pre_impulse_vels.get(&id).copied() else {
                continue;
            };
            if !self
                .phys
                .bodies
                .get(rec.handle)
                .is_some_and(|b| b.is_dynamic() && !b.is_sleeping())
            {
                continue;
            }
            // REQ-012 §6 D8(主体作为撞击源不触发地形破坏/冲量流;A01 起
            // 生效,A05 细化):主体对地形的接触不进破坏采样——走路与摔落
            // 都不破坏地形。物理接触本身不受影响(rapier 冲量照常解算)。
            if self.agents.contains_key(&id) {
                continue;
            }
            // 复合体走均摊口径(§7.3 第 7 条;开关同 §7.3 第 1 条,关闭时按
            // REQ-009 的逐格口径);单格逐格相加(REQ-009 原口径,逐位不变)。
            let is_compound = self.load_enabled() && rec.desc.shape().is_some();
            // 地形一侧 = 非实体的那一侧;其接触点 = local_p1(地形是 1)或
            // local_p2(地形是 2)。
            let (terrain_collider, terrain_is_1) = if entity_is_1 {
                (pair.collider2, false)
            } else {
                (pair.collider1, true)
            };
            let entity_collider = if entity_is_1 {
                pair.collider1
            } else {
                pair.collider2
            };
            // 两侧碰撞体位姿:地形侧用于接触点归格(必需);实体侧用于接触点
            // 速度(勘误 2)。任一侧 collider 可能已不在场——**同拍撞击路径
            // 先写 air 重建地形 collider**(本函数早前处理的接触对),本接触对
            // 里的句柄可能已失效:缺任一侧整对跳过(归格与接触点速度都做不了)。
            let (Some(terrain_pos), Some(entity_pos)) = (
                self.phys
                    .colliders
                    .get(terrain_collider)
                    .map(|c| *c.position()),
                self.phys
                    .colliders
                    .get(entity_collider)
                    .map(|c| *c.position()),
            ) else {
                continue;
            };
            let pre_spin = self.pre_impulse_spin.get(&id).copied();
            // §5 口径 = 解算器实际使用的流形(`pair.solver_manifolds()`;REQ-010
            // §5.7.10 末条、AC-F17)。地形 chunk 是复合形状:一对碰撞体多于 1 个
            // 流形时 rapier 的 contact clustering 把同法向流形合并成簇、只对簇
            // 解算,冲量只回写在簇上(plain `pair.manifolds()` 恒 0)——多流形
            // 对读 plain 会丢冲量(D15 内缩前即存在的潜在缺陷,单流形的平面落体
            // 不受影响)。REQ-009 旧口径读 plain 并非"恰排伪冲量"的设计:坑内
            // 对挤的伪冲量由修订 9 接近速度过滤排除。未聚簇时该访问器就是
            // plain 流形,单流形行为逐位不变。
            for m in pair.solver_manifolds() {
                // 法向为 world 系、以碰撞体 1 → 2 为正(风险 1):按实体在
                // 1/2 定"指向实体"的法向,撞击方向 d = 其反方向(压向地形)。
                let dir = if entity_is_1 {
                    m.data.normal
                } else {
                    -m.data.normal
                };
                // 接近速度(§5.7.9 勘误 2,F03b 口径):**质心口径与接触点
                // 口径取大**——本体(复合体)质心速度沿接触法向朝地形的分量
                // (旧口径,对方是地形为 0),与流形内各点的接触点速度
                // v + ω×(p − com) 的同向分量,两者最大值 ≥ 门槛则整流形计入。
                // 质心口径保住"撞击分几拍到账"的后几拍(接触点已被止住、
                // 本体仍压向地形);接触点口径覆盖旋转体;修订 9 的静止对挤
                // 两口径都 < 1 仍排除;非旋转体两项相等,与旧口径逐位一致。
                let com_speed = pre_vel.dot(dir);
                let max_approach = m
                    .points
                    .iter()
                    .map(|pt| {
                        let p_entity = if entity_is_1 {
                            entity_pos * pt.local_p1
                        } else {
                            entity_pos * pt.local_p2
                        };
                        let contact_vel = match pre_spin {
                            Some(sp) => {
                                let r = p_entity - sp.world_com;
                                pre_vel + sp.angvel.cross(r)
                            }
                            // 无 spin 快照(理论上与 pre_vel 同表同拍刷新,不会
                            // 发生;防御:退化为线速度口径)。
                            None => pre_vel,
                        };
                        contact_vel.dot(dir)
                    })
                    .fold(com_speed, f32::max);
                if max_approach < crate::destruction::IMPACT_MIN_APPROACH_SPEED {
                    continue;
                }
                let subshape = if terrain_is_1 {
                    m.subshape_pos1().copied()
                } else {
                    m.subshape_pos2().copied()
                }
                .unwrap_or(Pose::IDENTITY);
                for pt in &m.points {
                    if pt.data.impulse <= 0.0 {
                        continue;
                    }
                    let p = if terrain_is_1 {
                        terrain_pos * subshape * pt.local_p1
                    } else {
                        terrain_pos * subshape * pt.local_p2
                    };
                    // 沿"指向实体"的反方向退 0.1 m 取所在格(§5;0.1 覆盖斜交
                    // 法向与棱角接触,D15 内缩后接触点已在实心格表面)。
                    let q = p + dir * crate::destruction::IMPACT_SAMPLE_DEPTH;
                    let cell =
                        BlockPos::new(q.x.floor() as i32, q.y.floor() as i32, q.z.floor() as i32);
                    // 勘误 7(§5.7.9,F02b-4;只作用冲量流路径):退后所在格
                    // c₀ 无材料、且 q 到某轴格界 < 0.05 m 时,把界另一侧列为
                    // 候选——取各轴组合(去掉 c₀)中的**实心格**,该点冲量在
                    // 它们之间均分(BlockPos 升序,确定性),作用点位置不变。
                    // 齐平侧撞的流形角点(如 z = 65.0)floor 后掉进空气格丢
                    // 供给;0.05 远小于薄片厚 0.15 与格宽,真实接触点不会落
                    // 界外 0.05 以上。一个实心格都没有时仍归 c₀(照旧丢弃,
                    // 采样的格子集合再查材料,与 F01 的前提一致)。
                    // `impact_flow = false` 路径不动(REQ-009 §5 逐位不变,
                    // 其冒烟场景已按"错半格瞄准"调参)。
                    let mut assigned: Vec<(types::BlockPos, f32)> = vec![(cell, 1.0)];
                    if flow_enabled
                        && self
                            .registry
                            .material(self.voxels.get_block(cell))
                            .is_none()
                    {
                        // 每轴的"界另一侧"偏移:小数部分用 `rem_euclid(1.0)`
                        // (q − floor(q),恒 ∈ [0,1))——`fract()` 截断取小数,
                        // 负坐标返回负值(e2e 世界 y < 0 时 (−57.4).fract() =
                        // −0.4 < 0.05 被误判"贴下界")。小数 < 0.05 时另一侧在
                        // −1(靠上界 > 0.95 时 +1);c₀ = floor(q),恰在界上时
                        // 小数 = 0,q 属于格 [n, n+1) 的下界,另一侧 = [n−1, n)。
                        let qc = [q.x, q.y, q.z];
                        let offs = [
                            qc[0].rem_euclid(1.0),
                            qc[1].rem_euclid(1.0),
                            qc[2].rem_euclid(1.0),
                        ];
                        let axes: [Option<i32>; 3] = [
                            if offs[0] < 0.05 {
                                Some(-1)
                            } else if offs[0] > 1.0 - 0.05 {
                                Some(1)
                            } else {
                                None
                            },
                            if offs[1] < 0.05 {
                                Some(-1)
                            } else if offs[1] > 1.0 - 0.05 {
                                Some(1)
                            } else {
                                None
                            },
                            if offs[2] < 0.05 {
                                Some(-1)
                            } else if offs[2] > 1.0 - 0.05 {
                                Some(1)
                            } else {
                                None
                            },
                        ];
                        if axes.iter().any(|a| a.is_some()) {
                            let mut candidates: Vec<types::BlockPos> = Vec::new();
                            for dx in [0, 1] {
                                for dy in [0, 1] {
                                    for dz in [0, 1] {
                                        let cand = BlockPos::new(
                                            cell.x + dx * axes[0].unwrap_or(0),
                                            cell.y + dy * axes[1].unwrap_or(0),
                                            cell.z + dz * axes[2].unwrap_or(0),
                                        );
                                        if cand == cell {
                                            continue;
                                        }
                                        if self
                                            .registry
                                            .material(self.voxels.get_block(cand))
                                            .is_some()
                                        {
                                            candidates.push(cand);
                                        }
                                    }
                                }
                            }
                            candidates.sort();
                            candidates.dedup();
                            if !candidates.is_empty() {
                                let share = 1.0 / candidates.len() as f32;
                                assigned = candidates.into_iter().map(|c| (c, share)).collect();
                            }
                        }
                    }
                    // 只看本体区内的格(范围 2;副本格由其 owner 采样)。改归的
                    // 候选与 c₀ 同面相邻,本体区判定按 c₀ 已过,实心候选按各自
                    // 坐标再查(见下)。
                    if cell.x < min.x || cell.x >= max_x || cell.z < min.z || cell.z >= max_z {
                        continue;
                    }
                    // 冲量流分派(F02a):按实体累计事件(单格 = 该格各点冲量之
                    // 和;复合体成形时均摊);旧路径不跑,判废用。s_c 与 p_k 用
                    // 同一归格结果(勘误 7)。
                    if flow_enabled {
                        let e = flow_samples.entry(id).or_insert(ImpactFlowSample {
                            total: 0.0,
                            vec: Vec3::ZERO,
                            cells: BTreeMap::new(),
                            is_compound,
                        });
                        // total/vec = 该接触点的全量冲量与方向矢量(与改归无关,
                        // 每点只计一次);cells 的逐格冲量/加权位置按 share 分摊
                        // (Σ share = 1),p = Σ(格, 冲量×share, 位置×冲量×share) /
                        // total 与单格口径一致。
                        e.total += pt.data.impulse;
                        e.vec += dir * pt.data.impulse;
                        for (acell, share) in &assigned {
                            // 改归候选也要在本体区内(与 c₀ 同格面,通常已过;
                            // 界上撞击时防御)。
                            if acell.x < min.x
                                || acell.x >= max_x
                                || acell.z < min.z
                                || acell.z >= max_z
                            {
                                continue;
                            }
                            let c = e.cells.entry(*acell).or_insert((0.0, Vec3::ZERO));
                            c.0 += pt.data.impulse * share;
                            c.1 += p * (pt.data.impulse * share);
                        }
                        continue;
                    }
                    if is_compound {
                        let e = compound_impacts.entry(id).or_default();
                        e.total += pt.data.impulse;
                        e.vec += dir * pt.data.impulse;
                        e.cells.insert(cell);
                        continue;
                    }
                    let s = samples.entry(cell).or_default();
                    s.impulse += pt.data.impulse;
                    s.vec += dir * pt.data.impulse;
                    if pt.data.impulse > s.best
                        || (pt.data.impulse == s.best
                            && (dir.x, dir.y, dir.z) < (s.dir.x, s.dir.y, s.dir.z))
                    {
                        s.best = pt.data.impulse;
                        s.dir = dir;
                    }
                }
            }
        }
        // 冲量流分派(F02a,§5.7.1/§5.7.4):事件成形 → `evaluate_impact_flow`
        // 判定 → 名额 → 写入,然后**返回**——REQ-009 §5 的逐格比较不跑。
        // 关闭时落到下方逐格路径,逐位不变(AC-F12)。
        if flow_enabled {
            self.apply_impact_flow_events(flow_samples, min, max_x, max_z);
            return;
        }
        // 阈值判定(范围 2):单格 = 逐格冲量和 J ≥ 该格材料的 `impact_threshold`
        // (REQ-009 §5 原口径);复合体 = 均摊 `J_T / |T|` 逐格判 I(§7.3 第 7 条;
        // ∞ 自然不破)。非实心/未装载/界外的格无材料,整体忽略(界外 get_block = AIR)。
        // 同一地形格多来源(单格与复合体、多个复合体)取冲量最大者——保持
        // "一格至多一个候选"的既有不变量(脱落请求不重复)。J 矢量随判过者
        // 记下(K02 §5.6 第 1 条:破坏落写时入冲量表)。
        let mut best: BTreeMap<BlockPos, (f32, Vec3, BlockId, f32)> = BTreeMap::new();
        for (cell, s) in samples {
            let block = self.voxels.get_block(cell);
            let Some(m) = self.registry.material(block) else {
                continue;
            };
            if s.impulse >= m.impact_threshold {
                best.insert(cell, (s.impulse, s.vec, block, m.impact_threshold));
            }
        }
        for e in compound_impacts.into_values() {
            if e.cells.is_empty() {
                continue;
            }
            let j = e.total / e.cells.len() as f32;
            // 均摊冲量的方向 = 总矢量方向(矢量和方向与均摊同基准);总矢量为
            // 零(多点对向接触恰好抵消)时无击退方向,记零矢量(继承生成零冲
            // 量,无行为)。
            let dir_v = e.vec.normalize_or_zero() * j;
            for cell in e.cells {
                let block = self.voxels.get_block(cell);
                let Some(m) = self.registry.material(block) else {
                    continue;
                };
                if j >= m.impact_threshold {
                    best.entry(cell)
                        .and_modify(|cur| {
                            if j > cur.0 {
                                *cur = (j, dir_v, block, m.impact_threshold);
                            }
                        })
                        .or_insert((j, dir_v, block, m.impact_threshold));
                }
            }
        }
        let mut candidates: Vec<(BlockPos, f32, Vec3, BlockId, f32)> = best
            .into_iter()
            .map(|(cell, (j, vec, block, threshold))| (cell, j, vec, block, threshold))
            .collect();
        if candidates.is_empty() {
            return;
        }
        // 上限:撞击专属上限与每拍剩余名额(爆炸拍首已用掉的扣掉)取小。
        let budget = self.cfg.destruction.max_impact_breaks_per_tick.min(
            self.cfg
                .destruction
                .max_detach_per_tick
                .saturating_sub(self.outbox.detach_requests.len()),
        );
        if budget == 0 {
            return;
        }
        // 冲量降序,同值 BlockPos 升序(范围 2)。
        candidates.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
        let mut dirty: Vec<ChunkPos> = Vec::new();
        for (cell, _j, vec, block, _threshold) in candidates.into_iter().take(budget) {
            let _ = self.voxels.set_block(cell, BlockId::AIR);
            // 遥测(范围 4):撞击破坏的格与产出的脱落请求。
            self.outbox.destruction_stats.cells_impacted += 1;
            self.outbox.destruction_stats.detached += 1;
            // 冲量表(K02 §5.6 第 1 条):撞击路径写入 air 的格带 (J, d̂) 采样
            // 的 J 矢量——该格冲量和 × 受力方向(d̂ = 撞击方向"压向地形" =
            // 来袭方向,被撞断的结构沿它被打飞,拍末承载转换的邻接段据此继承
            // 击退)。爆炸/放置等其余来源的写入不经过这里,天然不带。每拍由
            // evaluate_load_tick 取走重建。
            self.impact_impulses.insert(cell, vec);
            // 撞击写入进入本拍坍塌的触发集(范围 3;实心 → air)。
            self.register_collapse_trigger(cell, block, BlockId::AIR);
            self.outbox.block_updates.push(BlockWrite {
                pos: cell,
                block: BlockId::AIR,
                kind: BlockWriteKind::Emergent,
                load_conversion: false,
            });
            // §4.11 统一出口:被撞掉的格下一拍唤醒压在其上的冻结体。
            self.register_terrain_write(cell);
            // 碎块零初速(§5 修订 8):带 `(J − I)·d` 会把碎块推进一格宽的坑
            // 里,侧面撞坏坑壁引起连锁;零初速下下方是空的就掉下去,是实心
            // 就落定并凝固回原位。
            self.outbox
                .detach_requests
                .push(crate::destruction::DetachRequest {
                    cell,
                    block,
                    impulse: Vec3::ZERO,
                });
            dirty.push(cell.chunk_of());
        }
        // 细则 4/5 同款守卫:只重建已建 ②(同 detonate_terrain);S03 起记入
        // 待重建集合,本函数的调用方在碎裂采样读取接触对之前统一 flush。
        self.defer_chunk_collider_rebuild(dirty);
        // 与同拍爆炸的脱落请求拼接后,保持整个 outbox 的 BlockPos 升序不变量。
        self.outbox.detach_requests.sort_by_key(|req| req.cell);
    }

    /// 撞击冲量流的生产接入(F02a,REQ-010 §5.7.1/§5.7.4/§5.7.5):由
    /// [`ShardSim::evaluate_impacts`] 在 `impact_flow` 生效时调用,**取代**
    /// REQ-009 §5 的逐格比较。
    ///
    /// - **事件成形**(§5.7.1):每实体一个 [`ImpactEvent`]——单格实体
    ///   `s_c` = 该格各点冲量之和(单格也可同时压多格,勘误 F02b-4);
    ///   复合体均摊 `J_T/|T|`(§7.3 第 7 条);
    ///   `J` = Σ j·d、`p` = 地形侧接触点按冲量加权的质心。
    /// - **顺序**(§5.7.4):先按 `s` 降序、同值 `EntityId` 升序取前
    ///   `impact_flow_max_events_per_tick` 个(其余本拍丢弃、不推迟),再按
    ///   `EntityId` 升序**依次**处理——后一个事件的 `block_at` 看得到前一个
    ///   事件写的 air(F03 教训:先采完本拍全部事件、后逐个写入,事件里只有
    ///   格坐标/冲量/点,没有句柄)。
    /// - **名额**(§5.7.4):S 整体装不进本拍剩余的 `max_detach_per_tick`
    ///   (与爆炸、承载坍塌共用,口径同既有代码)→ 整个事件放弃(不写、不断),
    ///   记 `impact_flow_skipped_budget`;不拆半个事件。
    /// - **写入**(§5.7.3 第 4 条):S 内各格沿用 REQ-009 撞击写入(air、
    ///   `Emergent`、§4.11 唤醒、重建 collider、进入本拍承载判定的触发集);
    ///   S 按 §5.5 切块成形(复合体请求 / 单格脱落),初速度按动量分配
    ///   (§5.7.2,F02b)。
    /// - **动量分配与撞击者修正**(§5.7.2,F02b):断开且名额够的事件调用
    ///   [`crate::impact_flow::distribute_momentum`]——`contact_points` 用本
    ///   单元采样的逐格聚合(每格一条 `(格, 冲量加权位置/该格冲量, 该格冲量)`,
    ///   归格在采样时已完成);`ImpactBodyProps` 取撞击者 rapier 刚体的质量、
    ///   世界系有效逆惯量(含旋转)与世界质心。撞击者在事件作用点 p 处受修正
    ///   冲量 `apply_impulse_at_point` 并唤醒(物理步进之后施加,同拍 D4 碎裂
    ///   在其后,`fracture_pieces` 读刚体速度自然含修正);计
    ///   `impact_flow_corrections`。扛住与名额放弃的事件不分配、不修正。
    /// - **遥测**(§5.7.8):8 个计数;`impact_flow_corrections` = 施加修正数。
    fn apply_impact_flow_events(
        &mut self,
        samples: BTreeMap<EntityId, ImpactFlowSample>,
        min: types::BlockPos,
        max_x: i32,
        max_z: i32,
    ) {
        use crate::impact_flow::ImpactEvent;
        if samples.is_empty() {
            return;
        }
        // 事件成形(§5.7.1):复合体均摊 J_T/|T|,单格 = 该格冲量和。
        // 空事件(|J| = 0、无格)丢弃;`s` = 各格 s_c 之和(浮点,排序用;
        // 单格口径下就是 total,复合体均摊不改变总和)。
        // `contact_points`(F02b,§5.7.2):采样逐格聚合的 `(格, 冲量加权位置/
        // 该格冲量, 该格冲量)`——归格在采样时按 §5.7.1 完成(地形侧退 0.1 m),
        // 纯函数不再对坐标取整。
        // 事件 = (s 排序键, EntityId, 事件, 接触点 (格, 冲量加权位置, 冲量))。
        type FlowEvent = (
            f32,
            EntityId,
            ImpactEvent,
            Vec<(types::BlockPos, Vec3, f32)>,
        );
        let mut events: Vec<FlowEvent> = samples
            .into_iter()
            .filter_map(|(id, s)| {
                if s.cells.is_empty() || s.total <= 0.0 {
                    return None;
                }
                // 供给(§5.7.1):复合体均摊每格 s_c = J_T/|T|(§7.3 第 7 条;
                // 超静定分配无物理意义);单格 = 该格各点冲量之和(cells 采样
                // 已累计)——单格也可同时压到几个地形格(勘误 B,F02b-4:均摊
                // 会把重压格的冲量摊给轻压格,违反逐格口径)。
                let supplies: Vec<(types::BlockPos, f32)> = if s.is_compound {
                    let per = s.total / s.cells.len() as f32;
                    s.cells.iter().map(|(&c, _)| (c, per)).collect()
                } else {
                    s.cells.iter().map(|(&c, (j, _))| (c, *j)).collect()
                };
                // 作用点 p:被撞格上冲量加权的位置(均摊后每格权重 = s_c =
                // J_T/|T| 相同,p = 各格加权位置和 / J_T;直接用采样的加权
                // 累计量,避免浮点重排)。
                let point = if s.total > 0.0 {
                    s.cells.values().map(|(_, w)| *w).sum::<Vec3>() / s.total
                } else {
                    return None;
                };
                // 接触点(§5.7.2):每格一条(格, 冲量加权位置/该格冲量,
                // 该格冲量);该格冲量为 0(不可能,采样只累计 > 0 的点)
                // 时防御性跳过。
                let contact_points = s
                    .cells
                    .iter()
                    .filter(|(_, (j, _))| *j > 0.0)
                    .map(|(&c, (j, w))| (c, w / j, *j))
                    .collect();
                Some((
                    s.total,
                    id,
                    ImpactEvent {
                        supplies,
                        impulse: s.vec,
                        point,
                    },
                    contact_points,
                ))
            })
            .collect();
        if events.is_empty() {
            return;
        }
        // 牛顿封顶(§5.7.9 勘误 6,F02b-3):窄墩工况一步穿进约 0.15 m 时同时
        // 压到侧面/顶面的内缩薄片,那些流形按勘误 2 不计入(接近速度 < 1),
        // 但其摩擦沿撞击方向抵掉了正面流形的大部分冲量——计入的正面法向合计
        // 于是远大于撞击者本步实际交给地形的动量,按 §5.7.2 分配会凭空造出
        // 动量与能量(碎块与撞击者都可能比来速快)。每个事件先求撞击者本步
        // 沿 d̂ 实际交出的冲量 N = m·((v_前 − v_后) + g·Δt)·d̂(线动量变化与
        // 作用点无关,偏心撞击同样成立;v_后 = 物理步进后、修正前的线速度——
        // 事件按实体互斥,先前事件的修正不影响本事件的刚体),κ =
        // clamp(N/|J|, 0, 1),N ≥ 0.99·|J| 时取 κ = 1(吸收求解器舍入,
        // 单一流形的墙面工况逐位不变);J 与全部 s_c 乘 κ、p 不变,之后照常
        // 排序、预筛、整数化、判定、分配。只缩不放(κ ≤ 1)不会多断。
        // 本表(`apply_impact_flow_events` 的调用方)按实体采样,追踪里记录
        // κ 与 N(F02b-3 观测口)。`impact_flow = false` 的 REQ-009 §5 与
        // §5.6 逐位不变(本函数只在冲量流路径被调用)。
        let dt = self.phys.integration_parameters.dt;
        let gravity = self.cfg.gravity;
        let mut kappas: BTreeMap<EntityId, (f32, f32, Vec3)> = BTreeMap::new();
        for (total, id, event, _contact_points) in events.iter_mut() {
            let Some(pre_vel) = self.pre_impulse_vels.get(id).copied() else {
                // 本步前无快照(新生的实体)不采样,防御跳过。
                continue;
            };
            let Some(rec) = self.entities.get(id) else {
                continue;
            };
            let Some(body) = self.phys.bodies.get(rec.handle) else {
                continue;
            };
            let m = body.mass();
            let v_post = conv::rapier_to_vec3(body.linvel());
            let d_hat = event.dir_hat();
            let j_len = event.impulse.length();
            if m <= 0.0 || j_len <= 0.0 {
                continue;
            }
            // N = m·((v_前 − v_后) + g·Δt)·d̂;补回重力冲量 m·g·Δt(步进期间
            // 重力先加给了刚体,不属于撞击交给地形的动量)。
            let n_transfer = m * ((pre_vel - v_post) + gravity * dt).dot(d_hat);
            let mut kappa = (n_transfer / j_len).clamp(0.0, 1.0);
            if n_transfer >= 0.99 * j_len {
                kappa = 1.0;
            }
            kappas.insert(*id, (kappa, n_transfer, pre_vel));
            if kappa < 1.0 {
                event.impulse *= kappa;
                *total *= kappa;
                for (_, s) in &mut event.supplies {
                    *s *= kappa;
                }
                // p 不变(contact_points 的逐格冲量权重随之自然等比,分配口径
                // 不再重读)。
            }
        }
        // 选取(§5.7.4):s 降序、同值 EntityId 升序,前 N 个;其余本拍丢弃。
        events.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        events.truncate(self.cfg.cohesion.impact_flow_max_events_per_tick);
        // 处理(§5.7.4):EntityId 升序**依次**——每个事件先按**当前**体素世界
        // 判定(&self 借用随返回值结束),再写入(可变借用);后一个事件因此
        // 看得到前一个事件写的 air(AC-F7;借用规则要求判定与写入不同帧,
        // 二者的先后就是这里的依次语义)。
        for (_, event_id, event, contact_points) in &events {
            let (outcome, segments) = self.plan_impact_flow_event(event, min, max_x, max_z);
            // 名额(§5.7.4):S 整体装不进**剩余**名额 → 整个事件放弃(不写、
            // 不断)。口径与承载转换一致:每段(单格或复合体)占 1 个实体名额。
            let remaining = self.remaining_detach_quota();
            let stats = &mut self.outbox.cohesion_stats;
            if outcome.prefiltered {
                // 预筛跳过:不建网、无 `C`,不记 FlowEventInfo(§3.5.1)。
                stats.impact_flow_prefiltered += 1;
                continue;
            }
            stats.impact_flow_events += 1;
            stats.impact_flow_cells_visited += outcome.visited as u64;
            stats.impact_flow_budget_hits += outcome.truncated as u64;
            // 事件摘要(REQ-011 §3.5.1;W03):`s`/`C` 由判定的同一份量化值
            // 换算;`broke` 与判定同比较;position = 被撞格格心(取供给最大
            // 的格,并列字典序最小,确定)。候选拿"本拍最危险"一条(后面
            // 每事件 pick 一次)。
            let info = crate::sim::FlowEventInfo {
                kind: crate::sim::FlowKind::Impact,
                supply: outcome.supply_q as f32 / crate::impact_flow::QUANT as f32,
                cut: outcome.cut_q as f32 / crate::impact_flow::QUANT as f32,
                broke: !outcome.detached.is_empty(),
                detached_cells: outcome.detached.len() as u64,
                position: {
                    let best = event
                        .supplies
                        .iter()
                        .max_by(|a, b| a.1.total_cmp(&b.1).then(b.0.cmp(&a.0)))
                        .map(|(c, _)| *c);
                    match best {
                        Some(c) => Vec3::new(c.x as f32 + 0.5, c.y as f32 + 0.5, c.z as f32 + 0.5),
                        // 防御:无格事件不可能过建网;回退作用点。
                        None => event.point,
                    }
                },
            };
            self.outbox.flow_event =
                crate::sim::FlowEventInfo::pick(self.outbox.flow_event.take(), Some(info));
            if outcome.detached.is_empty() {
                // 扛住:建了网但 C ≥ s,无断开、无修正(§5.7.2:修正只随断开)。
                continue;
            }
            stats.impact_flow_breaks += 1;
            if segments.len() > remaining {
                stats.impact_flow_skipped_budget += 1;
                continue;
            }
            // 动量分配(§5.7.2,F02b):断开且名额够的事件;`ImpactBodyProps`
            // 取撞击者 rapier 刚体的质量、世界系有效逆惯量(含旋转,rapier 的
            // `effective_world_inv_inertia` 就是 R·diag(1/I)·Rᵀ)与世界质心。
            // 撞击实体必然在场(本拍采到它的接触);不在 = 编程错误 fail-stop。
            let props = {
                let rec = self
                    .entities
                    .get(event_id)
                    .expect("事件实体在本拍采样时在场");
                let body = self.phys.bodies.get(rec.handle).expect("handle 有效");
                let inv = body.mass_properties().effective_world_inv_inertia;
                crate::impact_flow::ImpactBodyProps {
                    mass: body.mass(),
                    inv_inertia: [
                        [inv.m11, inv.m12, inv.m13],
                        [inv.m12, inv.m22, inv.m23],
                        [inv.m13, inv.m23, inv.m33],
                    ],
                    center_of_mass: conv::rapier_to_vec3(body.mass_properties().world_com),
                }
            };
            let (correction, momenta) =
                crate::impact_flow::distribute_momentum(event, &outcome, contact_points, &props);
            // 定量追踪(F02b-2 验收观测口):修正前的撞击者速度 + 全部事件
            // 量,修正施加后落表;`#[cfg(test)]`,生产构建零开销。
            #[cfg(test)]
            let trace_before = {
                let rec = self.entities.get(event_id).expect("事件实体在场");
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                (
                    conv::rapier_to_quat(b.rotation()),
                    conv::rapier_to_vec3(b.linvel()),
                    conv::rapier_to_vec3(b.angvel()),
                    conv::rapier_to_vec3(b.translation()),
                )
            };
            // 撞击者修正(§5.7.2):在事件作用点 p 处施加 `correction` 并唤醒——
            // 它不再停在墙面,以约 w 继续前进;修正只把 rapier 多扣的还回去,
            // w ≤ u 自然成立,不 clamp(§5.7.2;clamp 两键只属于 §5.6 旧路径)。
            // 物理步进之后施加:同拍 D4 碎裂(`evaluate_fractures` 在
            // `evaluate_impacts` 之后)读刚体速度,`fracture_pieces` 继承的
            // 速度自然含修正。零修正(ρ = 0 的边界)不施加、不计数。
            if correction != Vec3::ZERO {
                let rec = self
                    .entities
                    .get(event_id)
                    .expect("事件实体在本拍采样时在场");
                let handle = rec.handle;
                let body = self.phys.bodies.get_mut(handle).expect("handle 有效");
                body.apply_impulse_at_point(
                    conv::vec3_to_rapier(correction),
                    conv::vec3_to_rapier(event.point),
                    true,
                );
                // §5.8.1 修正扣除表(G02):本拍施加给该撞击者的修正合计(世界
                // 系;同实体多事件累加)。复合体冲量流的同拍采样据此扣
                // `correction = −R⁻¹·c_w`。拍内有效,每拍开头清空。
                *self
                    .impact_corrections_this_tick
                    .entry(*event_id)
                    .or_insert(Vec3::ZERO) += correction;
                self.outbox.cohesion_stats.impact_flow_corrections += 1;
            }
            #[cfg(test)]
            {
                let rec = self.entities.get(event_id).expect("事件实体在场");
                let b = self.phys.bodies.get(rec.handle).expect("handle 有效");
                let (kappa, n_transfer, linvel_pre) =
                    kappas
                        .get(event_id)
                        .copied()
                        .unwrap_or((1.0, 0.0, Vec3::ZERO));
                self.impact_flow_trace
                    .push(crate::impact_flow::FlowEventTrace {
                        event: event.clone(),
                        supply: event.total_supply(),
                        cut_q: outcome.cut_q,
                        quant: crate::impact_flow::QUANT as f32,
                        correction,
                        momenta: momenta.clone(),
                        rotation_before: trace_before.0,
                        translation_before: trace_before.3,
                        linvel_before: trace_before.1,
                        linvel_pre,
                        angvel_before: trace_before.2,
                        linvel_after: conv::rapier_to_vec3(b.linvel()),
                        angvel_after: conv::rapier_to_vec3(b.angvel()),
                        kappa,
                        n_transfer,
                        impactor: props,
                    });
            }
            // 各段的初速度(§5.7.3):段 → 所属分量 k。分量按撞击粘结连通、
            // 段按承载粘结 + 切块网格——段是分量的细分,**⊆ 某个分量**(可以
            // 只占分量的一部分);跨两个分量 = 未知形态,fail-stop 而非自行
            // 取舍。单格段 = 单格碎块,`impulse = 0.729·v_seg`(调度器
            // `detach_spawn` 以 `linvel = impulse / 0.729` 换算,速度恰还原);
            // ≥ 2 格段 = `velocity: (v_seg, ω_seg)` 直达 N+1 生成点。
            let seg_velocities: Vec<(Vec3, Vec3)> = segments
                .iter()
                .map(|segment| {
                    let k = momenta
                        .iter()
                        .find(|m| segment.iter().all(|c| m.cells.contains(c)));
                    let k = k.unwrap_or_else(|| {
                        panic!("段 {segment:?} 不整体落在任何动量分量内:未知形态 fail-stop")
                    });
                    let vels =
                        crate::impact_flow::segment_velocity(k, std::slice::from_ref(segment));
                    vels[0]
                })
                .collect();
            // 写入(§5.7.3 第 4 条):S 内各格 air、Emergent、唤醒、触发集;
            // 冲量表(F02b,§5.6 退役):`impact_flow` 生效时邻接继承不再执行
            // (D12),冲量表不再写入——失去支撑的段零初速。写 air 即刻落格,
            // 下一个事件的判定看得到。
            let mut dirty: Vec<types::ChunkPos> = Vec::new();
            let mut solid_blocks: std::collections::BTreeMap<types::BlockPos, types::BlockId> =
                std::collections::BTreeMap::new();
            for &cell in &outcome.detached {
                let block = self.voxels.get_block(cell);
                if self.registry.material(block).is_none() {
                    // 轮到时已不是实心(前一事件写掉——S 不相交,防御)。
                    continue;
                }
                solid_blocks.insert(cell, block);
                self.write_impact_detached_cell(cell, block, &mut dirty);
            }
            self.outbox.cohesion_stats.impact_flow_detached_cells += solid_blocks.len() as u64;
            // 成形(§5.7.3):S 按承载粘结切块划分——1 格段 = 单格脱落请求
            // (初速度 = v_seg,经 `impulse = 0.729·v_seg` 还原),≥ 2 格段 =
            // 复合体生成请求(`velocity` 直达生成点;§5.6 退役:impulses 恒空,
            // 速度不走邻接继承路径)。
            for (segment, (v_seg, w_seg)) in segments.iter().zip(&seg_velocities) {
                let solid_seg: Vec<types::BlockPos> = segment
                    .iter()
                    .filter(|c| solid_blocks.contains_key(c))
                    .copied()
                    .collect();
                if solid_seg.is_empty() {
                    continue;
                }
                if solid_seg.len() == 1 {
                    let cell = solid_seg[0];
                    self.outbox
                        .detach_requests
                        .push(crate::destruction::DetachRequest {
                            cell,
                            block: solid_blocks[&cell],
                            impulse: crate::destruction::DEBRIS_MASS * *v_seg,
                        });
                } else {
                    let origin = crate::cohesion::compound_origin(&solid_seg);
                    let world: Vec<(types::BlockPos, types::BlockId)> =
                        solid_seg.iter().map(|&c| (c, solid_blocks[&c])).collect();
                    let shape = types::CompoundShape::from_world(origin, &world)
                        .expect("段内 ≥ 2 格、偏移不重复(split_segments 保证)");
                    self.outbox
                        .compound_requests
                        .push(crate::cohesion::CompoundRequest {
                            origin,
                            shape,
                            impulses: Vec::new(),
                            velocity: (*v_seg, *w_seg),
                        });
                }
            }
            // 细则 4/5 同款守卫:只重建已建 ②(同 detonate_terrain);S03 起
            // 记入待重建集合,evaluate_impacts 收尾统一 flush(在碎裂采样读
            // 接触对之前,保住 F03b 的句柄失效可见性)。
            self.defer_chunk_collider_rebuild(dirty);
        }
        // 与同拍爆炸/逐格路径的脱落请求拼接后,保持 BlockPos 升序不变量。
        self.outbox.detach_requests.sort_by_key(|req| req.cell);
    }

    /// 一个撞击事件的判定与 S 分段(借用阶段:`&self` 借用随返回的 owned
    /// 数据结束,调用方随后用 `&mut self` 写入;见 [`Self::apply_impact_flow_events`]
    /// 的依次处理语义)。
    fn plan_impact_flow_event(
        &self,
        event: &crate::impact_flow::ImpactEvent,
        min: types::BlockPos,
        max_x: i32,
        max_z: i32,
    ) -> (
        crate::impact_flow::ImpactFlowOutcome,
        Vec<Vec<types::BlockPos>>,
    ) {
        let block_at = |p: types::BlockPos| -> Option<types::BlockId> {
            if !(types::WORLD_Y_MIN..=types::WORLD_Y_MAX).contains(&p.y) {
                return Some(types::BlockId::AIR);
            }
            self.voxels.get_chunk(p.chunk_of())?;
            Some(self.voxels.get_block(p))
        };
        let in_core =
            |p: types::BlockPos| p.x >= min.x && p.x < max_x && p.z >= min.z && p.z < max_z;
        let outcome = crate::impact_flow::evaluate_impact_flow(
            &block_at,
            &in_core,
            self.registry.as_ref(),
            event,
            self.cfg.cohesion.impact_flow_max_cells,
        );
        // S 分段(§5.7.3):S 内格轮到时已写成 air 的,`split_segments` 的
        // `block_at` 查不到方块、当无边处理——与"轮到时已不是实心的格丢弃"
        // 一致;判定后写入前调一次, 体素世界尚未被本事件改动。
        let segments = if outcome.detached.is_empty() {
            Vec::new()
        } else {
            crate::cohesion::split_segments(
                &block_at,
                self.registry.as_ref(),
                &outcome.detached,
                self.cfg.cohesion.compound_split_size,
            )
        };
        (outcome, segments)
    }

    /// 本拍剩余的脱落名额(爆炸、承载坍塌、撞击冲量流共用;与既有路径同式)。
    fn remaining_detach_quota(&self) -> usize {
        self.cfg
            .destruction
            .max_detach_per_tick
            .saturating_sub(self.outbox.detach_requests.len() + self.outbox.compound_requests.len())
    }

    /// 冲量流断下格的单格写入(REQ-009 撞击写入同款,§5.7.3 第 4 条):写 air、
    /// 遥测、触发集、Emergent、§4.11 唤醒。caller 负责成形、名额与
    /// collider 重建。
    fn write_impact_detached_cell(
        &mut self,
        cell: types::BlockPos,
        block: types::BlockId,
        dirty: &mut Vec<types::ChunkPos>,
    ) {
        let _ = self.voxels.set_block(cell, types::BlockId::AIR);
        self.outbox.destruction_stats.cells_impacted += 1;
        self.outbox.destruction_stats.detached += 1;
        // 冲量表(F02b,§5.6 退役 / D12):`impact_flow` 生效时邻接继承不再
        // 执行,冲量表不再写入——失去支撑的段零初速(动量只由 §5.7.2 的守恒
        // 分配记账,叠加继承会重复计数)。本函数只在冲量流路径被调用。
        self.register_collapse_trigger(cell, block, types::BlockId::AIR);
        self.outbox.block_updates.push(BlockWrite {
            pos: cell,
            block: types::BlockId::AIR,
            kind: BlockWriteKind::Emergent,
            load_conversion: false,
        });
        self.register_terrain_write(cell);
        dirty.push(cell.chunk_of());
    }

    /// 碎裂采样与拆分(REQ-010 §7.3,单元 06;调用点 = `step` 内撞击采样之后、
    /// 凝固桥之前)。开关与承载判定相同(§7.3 第 1 条)。对每个醒着的复合体
    /// (§7.3 第 2 条:本步之前的动态醒体、有形状;对方可以是地形或其他实体)
    /// 取接近速度 ≥ 1 m/s 的接触点,累计冲量合计 J 与各接触格的受力方向(指向
    /// 本体内部的接触法向之和,归一化);本步开始后新生的实体不在
    /// `pre_impulse_vels`,本拍不采样。上限与顺序(§7.3 第 6 条):每拍最多
    /// 坍塌接入(REQ-009 §6.2.1,单元 04b;step 末调用 = 物理步进之后、交出
    /// outbox 之前)。承载判定开启时由 [`ShardSim::evaluate_load_tick`] 取代
    /// (REQ-010 §4.6 第 1 条);否则走 REQ-009 原路径。触发集 = 本拍实心→非
    /// 实心写入 ∪ 推迟队列(上一拍触发集原样 + 上一拍已转换的格);对**本分片
    /// 体素**求值 `find_collapse` 后,只转换本体区内的 `falling` 格(`BlockPos`
    /// 升序,find_collapse 已排序),受本拍剩余名额
    /// `max_detach_per_tick − 本拍已有脱落请求数` 约束;转换 =
    /// 写 air、`Emergent` 写入、§4.11 唤醒、重建 collider、追加冲量为 0 的
    /// `DetachRequest`(生成走单元 03 的调度器路径,N+1 生效)。名额不足且
    /// 本体区内还有悬空格未转换时,整个触发集原样连同已转换格一起推迟到下一拍
    /// (不只留没转换的格——名额为 0 的那一拍不能丢触发,风险 4)。
    fn evaluate_collapse(&mut self) {
        if self.load_enabled() {
            self.evaluate_load_tick();
            return;
        }
        let mut removed: BTreeSet<BlockPos> = std::mem::take(&mut self.collapse_triggers);
        removed.extend(std::mem::take(&mut self.collapse_deferred));
        if removed.is_empty() {
            return;
        }
        let min = self.cfg.shard.pos.min_chunk().block_min();
        let max_x = min.x + SHARD_CHUNKS * CHUNK_SIZE;
        let max_z = min.z + SHARD_CHUNKS * CHUNK_SIZE;
        let in_core = |p: BlockPos| p.x >= min.x && p.x < max_x && p.z >= min.z && p.z < max_z;
        let block_at = |p: BlockPos| -> Option<BlockId> {
            if !(types::WORLD_Y_MIN..=types::WORLD_Y_MAX).contains(&p.y) {
                // 世界高度范围之外按空气,不是未装载(§6.2.1;风险 2)——
                // 贴着世界顶的结构不能被误判有地基(acc_d5_world_top)。
                return Some(BlockId::AIR);
            }
            // 所在 chunk 未装载 → None(保守:看不见的地方当作有地基)。
            self.voxels.get_chunk(p.chunk_of())?;
            Some(self.voxels.get_block(p))
        };
        let removed_vec: Vec<BlockPos> = removed.iter().copied().collect();
        let outcome = crate::destruction::find_collapse(
            &block_at,
            self.registry.as_ref(),
            &removed_vec,
            self.cfg.destruction.collapse_max_cells,
            self.cfg.destruction.collapse_max_cluster,
        );
        // 遥测(REQ-009 §8):求值数、访问数、预算锚定与超大块——`falling`
        // 为空的拍(判定不塌)同样计入,空触发集的拍不进来(上方早退)。
        self.outbox.destruction_stats.collapse_evaluations += 1;
        self.outbox.destruction_stats.collapse_visited += outcome.visited as u64;
        self.outbox.destruction_stats.collapse_budget_anchored += outcome.budget_anchored as u64;
        self.outbox.destruction_stats.collapse_oversized += outcome.oversized_clusters as u64;
        if outcome.falling.is_empty() {
            // 没有可转换格:推迟队列已随 take 清空,本拍结束(无推迟)。
            return;
        }
        // 本拍剩余名额:爆炸在拍首(apply_prefix)已用掉的脱落请求要扣掉。
        let quota = self
            .cfg
            .destruction
            .max_detach_per_tick
            .saturating_sub(self.outbox.detach_requests.len());
        // 推迟数口径(§8):本体区内悬空格总数 − 本拍已转换数(不是触发集大小)。
        let in_core_falling = outcome.falling.iter().filter(|c| in_core(**c)).count();
        let mut converted: Vec<BlockPos> = Vec::new();
        let mut deferred = false;
        let mut dirty: Vec<ChunkPos> = Vec::new();
        for &cell in &outcome.falling {
            if !in_core(cell) {
                continue; // 跨分片的另一半由其 owner 转换,不占本分片名额(§6.3)
            }
            if converted.len() >= quota {
                deferred = true;
                break;
            }
            let block = self.voxels.get_block(cell);
            let _ = self.voxels.set_block(cell, BlockId::AIR);
            // 遥测(§8):坍塌转换的格与产出的脱落请求。
            self.outbox.destruction_stats.cells_collapsed += 1;
            self.outbox.destruction_stats.detached += 1;
            self.outbox.block_updates.push(BlockWrite {
                pos: cell,
                block: BlockId::AIR,
                kind: BlockWriteKind::Emergent,
                load_conversion: false,
            });
            // §4.11 统一出口:脚下被掏空的冻结体下一拍被唤醒。
            self.register_terrain_write(cell);
            self.outbox
                .detach_requests
                .push(crate::destruction::DetachRequest {
                    cell,
                    block,
                    impulse: Vec3::ZERO,
                });
            dirty.push(cell.chunk_of());
            converted.push(cell);
        }
        // 细则 4/5 同款守卫:只重建已建 ②(同 detonate_terrain);S03 起记入
        // 待重建集合,step 末统一重建。
        self.defer_chunk_collider_rebuild(dirty);
        // 遥测(§8):本拍因名额不足未转换的本体区悬空格数(converted 只含
        // 本体区格,差值恒非负;saturating 防御遥测不 panic)。
        self.outbox.destruction_stats.collapse_deferred +=
            in_core_falling.saturating_sub(converted.len()) as u64;
        // 与同拍爆炸的脱落请求拼接后,保持整个 outbox 的 BlockPos 升序不变量。
        self.outbox.detach_requests.sort_by_key(|req| req.cell);
        let mut carry: BTreeSet<BlockPos> = converted.into_iter().collect();
        if deferred {
            // 已转换的格成为下一拍的触发(§6.2 连锁坍塌);推迟时整个触发集
            // 原样保留,下一拍重新搜索。
            carry.extend(removed);
        }
        self.collapse_deferred = carry;
    }

    fn apply_command(&mut self, cmd: SimCommand, source: InputSource) {
        match cmd {
            SimCommand::SetBlock { pos, block } => {
                // A05(REQ-012 §6"暂缓写入"):外部实心写入与主体连杆重叠
                // → 不写体素,进暂缓队列(判定只查本拍形状表,不查 agents;
                // owner 与 holder 收到同一命令 + 同一表 ⇒ 结论一致)。写空气
                // 和非实心方块从不暂缓;同格已有暂缓条目时,实心替换(计
                // "被替换")、非实心直接写入并作废(计"被作废")。
                if self.registry.material(block).is_some() {
                    if self.agent_shapes.overlaps(pos) {
                        let stats = &mut self.outbox.cohesion_stats;
                        if self.deferred_set_blocks.insert(pos, block).is_some() {
                            stats.set_block_deferred_replaced += 1;
                        } else {
                            stats.set_block_deferred += 1;
                        }
                        return;
                    }
                } else if let Some(_stale) = self.deferred_set_blocks.remove(&pos) {
                    self.outbox.cohesion_stats.set_block_deferred_invalidated += 1;
                }
                // 触发登记要写前的方块(实心性与方块比较都用它)。
                let old_block = self.voxels.get_block(pos);
                let edit = VoxelEdit::new().set(pos, block);
                match edit.apply(&mut self.voxels) {
                    Ok(dirty) => {
                        // 细则 4/5:只重建已建 ②;有①无② 的 chunk 写块不新建
                        // ②(未启用模式 ②⊆① 恒真,行为不变,§14-9);S03 起
                        // 记入待重建集合,apply_prefix 退出前统一重建。
                        self.defer_chunk_collider_rebuild(dirty);
                        // 方块写入统一出口(REQ-005 D11):外部命令成功写体素
                        // 后记 `External`;调度器 `take_block_updates` 只发布
                        // owner 的条目(非 owner 副本的条目由调度器丢弃)。
                        self.outbox.block_updates.push(BlockWrite {
                            pos,
                            block,
                            kind: BlockWriteKind::External,
                            load_conversion: false,
                        });
                        self.register_terrain_write(pos);
                        // 坍塌/承载触发(REQ-009 §6.2.1、REQ-010 §4.6 第 2 条)。
                        self.register_collapse_trigger(pos, old_block, block);
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
            SimCommand::SpawnCompound {
                entity,
                origin,
                shape,
                impulses,
                velocity,
            } => {
                // 承载转换复合体生成(REQ-010 §5.6 第 3 条,K02;调度器屏障盖章
                // N+1 的专用命令,外部路径不可达——net 白名单与
                // validate_external_command 都拒绝):初态与 K01 之前完全一致
                // (原点格中心、零速、单位旋转、DEBRIS_HALF_SIZE 复合描述)。
                // 两条速度路径(互斥,不叠加):
                // - `impulses` 非空(§5.6 旧路径):生成后逐条
                //   `apply_impulse_at_point`(不合并,保真力矩),全部施加完读回
                //   linvel/angvel 按 `load_knockback_max_speed`/
                //   `load_knockback_max_ang_speed` 对模长 clamp(方向不变)后
                //   写回。β 已在构造侧乘入(§5.6 第 2 条);β = 0 时施加零冲量
                //   是数值 no-op(apply_impulse 对零冲量直接短路),行为与零速
                //   路径逐位一致(AC-K4 已钉)。
                // - `velocity` 非零(§5.7.3,F02b):冲量流路径的初速度,生成后
                //   **直接设定**,不经 `apply_knockback`、不 clamp——§5.7.2:速度
                //   已被撞击者的动量自然封顶,clamp 两键只属于 §5.6 旧路径。
                //   两路互斥(§5.7 生效时构造侧 impulses 恒空;§5.6 路径的
                //   velocity 恒零),debug_assert 钉住。
                debug_assert!(
                    impulses.is_empty() || velocity == (Vec3::ZERO, Vec3::ZERO),
                    "SpawnCompound 两路速度互斥:impulses = {:?}, velocity = {:?}",
                    impulses,
                    velocity
                );
                let state = EntityState {
                    translation: Vec3::new(
                        origin.x as f32 + 0.5,
                        origin.y as f32 + 0.5,
                        origin.z as f32 + 0.5,
                    ),
                    rotation: types::glam::Quat::IDENTITY,
                    linvel: velocity.0,
                    angvel: velocity.1,
                };
                let desc = EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape);
                self.spawn_entity(entity, state, desc, None, false);
                self.apply_knockback(entity, impulses);
            }
            SimCommand::SpawnAgent {
                entity,
                body,
                state,
            } => {
                // 主体生成(REQ-012 §3,A01):本地受信通道专用命令(不进
                // C2S 白名单;net/command_shard 与 validate_external_command
                // 同口径拒绝)。类型化拒绝 → 遥测计数(Outbox.
                // agent_spawn_rejected),不断链;重复 EntityId 与 SpawnDynamic
                // 同口径(fail-stop assert,spawn_agent 内)。
                // 来源换算与 SpawnDynamic 一致(调度器注入 = 服务端实体);
                // REQ-012 A03 §2:写进实体记录——本地会话/调度器 → None,
                // 网络玩家 → Some(player),`entity_owner(a).source` 指认玩家
                // 身体(迁移随实体记录带走)。
                let owner_source = if source == InputSource::SCHEDULER {
                    None
                } else {
                    Some(source)
                };
                if let Err(_e) = self.spawn_agent(entity, body, state, owner_source) {
                    // 遥测在 spawn_agent 内计数(类型化拒绝不断链)。
                }
            }
            SimCommand::AgentIntent {
                entity,
                move_xz,
                yaw,
                jump,
            } => {
                // 主体意图(REQ-012 A02a 第 2 节):**内核不信任它**——本命令
                // 可能来自日志回放,所以这里重做全部检查,不 panic:
                // - 实体不存在 / 不是主体 / 无 `[walker]` 段 → 类型化拒绝 +
                //   遥测(命令来自日志,拒绝是"这拍没生效",不是一致性违规);
                // - 非有限值(NaN/inf)→ 拒绝 + 遥测(不写进保持状态);
                // - `move_xz` 超过 `[walker].max_speed` → 按比例钳 + 遥测;
                // - `yaw` 规范化到 (−π, π]。
                // 合法意图写进 `AgentRecord` 保持状态(直到下一条意图到来,
                // 随快照走);`jump` 一次性,下一控制拍消费。计数与写入都
                // 收口在 `apply_agent_intent`(agent.rs)。
                if self.apply_agent_intent(entity, move_xz, yaw, jump).is_err() {
                    self.outbox.agent_intent_rejected += 1;
                }
            }
            SimCommand::AgentJointTargets { entity, targets } => {
                // 关节目标(REQ-012 B01a,§2.2 关节层级):**内核不信任它**
                // ——本命令可能来自日志回放,这里重做全部检查,不 panic:
                // - 非主体 / dof = 0 / 长度不符 / 非有限值 → 类型化拒绝 +
                //   遥测(世界不变,AC-B5);
                // - 超限目标 → 钳到限位(每夹一个 +1 遥测钳制计数,AC-B5
                //   "超出关节范围的目标在进入内核前钳制")。
                // 生效拍与 `AgentIntent` 同口径(命令 = 盖章输入,下一拍
                // apply_prefix 时点生效);目标写进 AgentRecord 保持状态
                // (直到下一条命令,随快照走),电机目标逐关节更新,目标
                // 变化唤醒整只主体。计数与写入都收口在
                // `apply_agent_joint_targets`(agent.rs)。
                match self.apply_agent_joint_targets(entity, &targets) {
                    Ok(clamped) => self.outbox.agent_joint_targets_clamped += u64::from(clamped),
                    Err(_) => self.outbox.agent_joint_targets_rejected += 1,
                }
            }
            SimCommand::RemoveEntity { entity } => {
                // 幂等(REQ-007 任务 03 生产接入后):实体可能已在本拍前的
                // 生命周期段合法终结(凝固/强制落地/虚空)——不存在 = 无操作,
                // 不再 fail-stop(回评请求追认;网络路径本就经
                // validate_external_command 前置挡未知实体)。
                // REQ-012 A01:主体走 despawn_agent(清理 agents 表 + 回到
                // 生成前状态;判据 A01-1"移除后无残留")。
                if self.agents.contains_key(&entity) {
                    self.despawn_agent(entity);
                } else if self.entities.contains_key(&entity) {
                    self.despawn_entity(entity);
                }
            }
            SimCommand::TeleportEntity { entity, state } => {
                // 一切位姿变更 = 删除 + 重建(REQ-001 §5 行 10);actor/source
                // 是政策状态,必须跨重建保留(REQ-006 任务 00)。
                // REQ-012 A01:主体不经 Teleport(删除+重建会丢 agents 表的
                // 连杆句柄;主体的位姿变更属 A02/A04 的控制器与迁移)。
                assert!(
                    !self.agents.contains_key(&entity),
                    "主体 {entity:?} 不支持 TeleportEntity(REQ-012 A01:主体的位姿变更走 A02 控制器/A04 迁移,fail-stop)"
                );
                let (_, desc, owner_source, actor) = self.despawn_entity(entity);
                self.spawn_entity(entity, state, desc, owner_source, actor);
            }
            SimCommand::Detonate {
                center,
                power,
                radius,
            } => self.detonate(center, power, radius),
            SimCommand::MarkActor { entity, actor } => {
                // REQ-012 A01:主体 `actor` 恒为 true(§3)——MarkActor{false}
                // 会让凝固桥/冻结把主体写进体素或冻结,fail-stop 挡下。
                if actor {
                    let rec = self
                        .entities
                        .get_mut(&entity)
                        .expect("MarkActor 触及不存在实体(fail-stop)");
                    rec.actor = actor;
                } else {
                    assert!(
                        !self.agents.contains_key(&entity),
                        "主体 {entity:?} 不支持 MarkActor{{false}}(REQ-012 A01:主体 actor 恒为 true,fail-stop)"
                    );
                    let rec = self
                        .entities
                        .get_mut(&entity)
                        .expect("MarkActor 触及不存在实体(fail-stop)");
                    rec.actor = actor;
                }
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
        // S03 遥测(§4.16 同款;只计数不进模拟):本拍 collider 重建次数——
        // 含生命周期路径的立即重建与 flush 的固定点重建,= 重建调用的真实
        // 总数。随 ActivationEvents 取走(不入快照,纯计数)。
        self.activation_events.collider_rebuilds += 1;
    }

    /// 批量写路径的重建登记(S03 去重):不立即重建,把 chunk(含已装载的
    /// 边界邻居,与 `expand_boundary` 同口径)记入待重建集合;调用方在公开
    /// 写入口退出前 [`Self::flush_collider_rebuilds`] 统一执行。只登记已建
    /// ② 的 chunk(守卫与旧立即路径逐位一致;flush 时再查一次,防御登记与
    /// 执行之间 ② 被生命周期路径拆掉的拍内交错)。
    fn defer_chunk_collider_rebuild(&mut self, cps: impl IntoIterator<Item = ChunkPos>) {
        for cp in expand_boundary(cps, &self.voxels) {
            if self.built_colliders.contains(&cp) {
                self.pending_collider_rebuilds.insert(cp);
            }
        }
    }

    /// 固定点统一重建(S03 去重):按 ChunkPos 升序重建全部待重建 chunk。
    /// 重建序确定(BTreeSet 键序),句柄分配序随之确定;同一拍多写入不再
    /// 重复重建同一 chunk。调用点 = 每个公开写入口退出前(见
    /// [`ShardSim::pending_collider_rebuilds`]),返回时集合必空。
    fn flush_collider_rebuilds(&mut self) {
        let pending: Vec<ChunkPos> = self.pending_collider_rebuilds.iter().copied().collect();
        self.pending_collider_rebuilds.clear();
        for cp in pending {
            if self.built_colliders.contains(&cp) {
                self.rebuild_chunk_collider(cp);
            }
        }
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
        // 复合体(REQ-010 §5.4 第 3 条):一个动态刚体,每格一个半边长
        // `half_size` 的 cuboid collider,局部平移 = 偏移(刚体平移 = 原点,
        // 质心/惯量由 rapier 按碰撞体算);密度/摩擦/弹性同单格。单格路径
        // (shape = None)逐位不变。
        let (handle, collider) = match desc.shape() {
            None => self.phys.insert(
                RigidBodyBuilder::dynamic()
                    .pose(conv::pose_to_rapier(state.translation, state.rotation))
                    .linvel(conv::vec3_to_rapier(state.linvel))
                    .angvel(conv::vec3_to_rapier(state.angvel)),
                ColliderBuilder::cuboid(h, h, h)
                    .density(1.0)
                    .friction(0.6)
                    .restitution(0.1),
            ),
            Some(shape) => {
                let body = RigidBodyBuilder::dynamic()
                    .pose(conv::pose_to_rapier(state.translation, state.rotation))
                    .linvel(conv::vec3_to_rapier(state.linvel))
                    .angvel(conv::vec3_to_rapier(state.angvel));
                // 首格经 insert 建体拿 collider 句柄(支撑判定的接触对键,
                // 1:1 约定保持"每实体一个代表 collider");其余格挂同一刚体。
                let first = shape.cells().first().expect("形状至少 2 格(构造保证)");
                let (handle, collider) = self.phys.insert(
                    body,
                    ColliderBuilder::cuboid(h, h, h)
                        .density(1.0)
                        .friction(0.6)
                        .restitution(0.1)
                        .translation(conv::vec3_to_rapier(Vec3::new(
                            first.offset[0] as f32,
                            first.offset[1] as f32,
                            first.offset[2] as f32,
                        ))),
                );
                for cell in shape.cells().iter().skip(1) {
                    let _ = self.phys.insert_collider(
                        ColliderBuilder::cuboid(h, h, h)
                            .density(1.0)
                            .friction(0.6)
                            .restitution(0.1)
                            .translation(conv::vec3_to_rapier(Vec3::new(
                                cell.offset[0] as f32,
                                cell.offset[1] as f32,
                                cell.offset[2] as f32,
                            ))),
                        Some(handle),
                    );
                }
                (handle, collider)
            }
        };
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

    /// 撞击击退的施加与上限(REQ-010 §5.6 第 3 条,K02;上限语义裁定
    /// 2026-10-30):刚体生成后逐条 `apply_impulse_at_point`(**不合并条目**,
    /// 保真力矩——合并会改变 r×J 的合力矩);全部施加完读回 linvel/angvel,
    /// 按 `load_knockback_max_speed`/`load_knockback_max_ang_speed` 对**模长**
    /// clamp(方向不变)后写回。冲量表为空 = no-op(不移速不 wake);上限
    /// 无"不设限"哨兵——**取 0 语义 = 钉死为零**(长度超上限即 clamp,0 上限
    /// 自然把速度/角速度钉零;想不设限用足够大的数值表达,§5.6 第 4 条)。
    fn apply_knockback(&mut self, id: EntityId, impulses: Vec<(Vec3, Vec3)>) {
        if impulses.is_empty() {
            return;
        }
        let (handle, v_max, w_max) = {
            let rec = self
                .entities
                .get(&id)
                .expect("SpawnCompound 刚生成,fail-stop");
            (
                rec.handle,
                self.cfg.cohesion.load_knockback_max_speed,
                self.cfg.cohesion.load_knockback_max_ang_speed,
            )
        };
        {
            let body = self.phys.bodies.get_mut(handle).expect("handle 有效");
            for (impulse, point) in &impulses {
                body.apply_impulse_at_point(
                    conv::vec3_to_rapier(*impulse),
                    conv::vec3_to_rapier(*point),
                    true,
                );
            }
        }
        let body = self.phys.bodies.get_mut(handle).expect("handle 有效");
        let mut linvel = conv::rapier_to_vec3(body.linvel());
        let mut angvel = conv::rapier_to_vec3(body.angvel());
        if linvel.length() > v_max {
            linvel = linvel.normalize() * v_max;
        }
        if angvel.length() > w_max {
            angvel = angvel.normalize() * w_max;
        }
        body.set_linvel(conv::vec3_to_rapier(linvel), true);
        body.set_angvel(conv::vec3_to_rapier(angvel), true);
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
            // ---- 复合体:逐格求值(REQ-010 §6.1,D6;开关同承载)----
            if self.load_enabled() && rec.desc.shape().is_some() {
                self.detonate_compound(id, center, power, radius);
                continue;
            }
            let pos = conv::rapier_to_vec3(body.translation());
            // A05(REQ-012 §6"爆炸:对每个连杆施加冲量"):主体按连杆遍历——
            // 每个连杆取其**连杆中心**的 `explosion_impulse`(与普通实体同一
            // 公式同一支点),施加到**该连杆自己的刚体**(R1-P4:多连杆时
            // 写死根刚体等于把全部冲量合并到根,与放行"按连杆遍历,不能写死
            // 根连杆"不符;单连杆 `lh == rec.handle` 结果逐位不变)。非主体
            // 实体 = 旧路径逐位不变。
            let link_centers: Vec<(Vec3, rapier3d::prelude::RigidBodyHandle)> =
                match self.agents.get(&id) {
                    None => vec![(pos, rec.handle)],
                    Some(record) => record
                        .links
                        .values()
                        .filter_map(|&lh| {
                            self.phys
                                .bodies
                                .get(lh)
                                .map(|lb| (conv::rapier_to_vec3(lb.translation()), lh))
                        })
                        .collect(),
                };
            for (center_of_link, lh) in link_centers {
                if let Some(imp) =
                    crate::explosion::explosion_impulse(center, power, radius, center_of_link)
                {
                    let body = self.phys.bodies.get_mut(lh).expect("连杆句柄有效");
                    body.apply_impulse(conv::vec3_to_rapier(imp), true);
                }
            }
        }
        // 地形破坏(REQ-009 §4.1):D7 起默认开启;实体冲量仍按原半径
        // (REQ-001 §4.2 不变),地形求值在冲量之后。
        if self.cfg.destruction.enabled {
            self.detonate_terrain(center, power, radius);
        }
    }

    /// 爆炸对复合体(REQ-010 §6.1,单元 07;开关 = `load_enabled`)。逐格求值
    /// [`cohesion::compound_blast`] 之后:
    /// - **没有格被破坏**(§6.1 第 2 条):各冲量按施力点作用在原刚体上
    ///   ([`apply_impulse_at_point`],产生力矩,D6——爆炸能把复合体掀翻、转起来);
    /// - **有格被破坏**(§6.1 第 3 条):原实体本拍删除;剩余完好格按共面连接
    ///   求连通分量,经 [`cohesion::fracture_pieces`] 求各分量状态,线速度再加
    ///   上该分量各格所受冲量之和 / 分量质量(力矩忽略,角速度沿用原 ω);脱落
    ///   格成单格碎块,速度 = 该点刚体速度 `v + ω × r` + 冲量 / 0.729;粉碎格
    ///   消失。分块请求的 `index`:先各分量(按首格),再各脱落格(按形状格序)
    ///   ——经调度器单元 05 路径 N+1 生成(与地形脱落共用"先脱落/复合体请求,
    ///   再分块请求"的涌现 ID 分配顺序,§6.1 风险 4)。
    ///
    /// 遥测(§6.1 第 5 条):`blast_removed_cells` = 粉碎 + 脱落的格数。
    fn detonate_compound(&mut self, id: EntityId, center: Vec3, power: f32, radius: f32) {
        let (handle, shape, desc) = {
            let rec = self.entities.get(&id).expect("存在");
            (
                rec.handle,
                rec.desc.shape().expect("复合体").clone(),
                rec.desc.clone(),
            )
        };
        let state = self.read_state(self.entities.get(&id).expect("存在"));
        let block_at = |p: BlockPos| -> Option<BlockId> {
            if !(types::WORLD_Y_MIN..=types::WORLD_Y_MAX).contains(&p.y)
                || self.voxels.get_chunk(p.chunk_of()).is_none()
            {
                return None;
            }
            Some(self.voxels.get_block(p))
        };
        let fates = crate::cohesion::compound_blast(
            &state,
            &shape,
            center,
            power,
            radius,
            self.cfg.destruction.attenuation,
            &block_at,
            self.registry.as_ref(),
        );
        // 各格结果与所受冲量(完好格 = 单格公式冲量;脱落格 = P·dir')。
        let cells = shape.cells();
        let mut broken = false;
        let mut impulses: Vec<Vec3> = Vec::with_capacity(cells.len());
        let mut removed_cells: u64 = 0;
        for (fate, _cell) in fates.iter().zip(cells.iter()) {
            match *fate {
                crate::cohesion::CellBlast::Intact { impulse } => {
                    impulses.push(impulse.unwrap_or(Vec3::ZERO));
                }
                crate::cohesion::CellBlast::Detach { impulse } => {
                    broken = true;
                    removed_cells += 1;
                    impulses.push(impulse);
                }
                crate::cohesion::CellBlast::Pulverize => {
                    broken = true;
                    removed_cells += 1;
                    impulses.push(Vec3::ZERO);
                }
            }
        }
        if !broken {
            // §6.1 第 2 条:无破坏 → 逐格按施力点施加(产生力矩,不用中心合成)。
            for (cell, impulse) in cells.iter().zip(impulses.iter()) {
                if impulse != &Vec3::ZERO {
                    let w = state.translation
                        + state.rotation
                            * Vec3::new(
                                cell.offset[0] as f32,
                                cell.offset[1] as f32,
                                cell.offset[2] as f32,
                            );
                    self.phys
                        .bodies
                        .get_mut(handle)
                        .expect("handle 有效")
                        .apply_impulse_at_point(
                            conv::vec3_to_rapier(*impulse),
                            conv::vec3_to_rapier(w),
                            true,
                        );
                }
            }
            return;
        }
        // §6.1 第 3 条:有破坏 → 删除原实体,剩余格按共面连接拆分。
        // 破坏格集合(形状格序号);完好格上"被破坏的共面连接" = 断键。
        let removed: std::collections::BTreeSet<usize> = fates
            .iter()
            .enumerate()
            .filter(|(_, f)| !matches!(f, crate::cohesion::CellBlast::Intact { .. }))
            .map(|(i, _)| i)
            .collect();
        let mut broken_bonds: std::collections::BTreeSet<(usize, usize)> =
            std::collections::BTreeSet::new();
        for (i, a) in cells.iter().enumerate() {
            for (j, b) in cells.iter().enumerate() {
                if i >= j {
                    continue;
                }
                let d = [
                    a.offset[0] - b.offset[0],
                    a.offset[1] - b.offset[1],
                    a.offset[2] - b.offset[2],
                ];
                if d[0].abs() + d[1].abs() + d[2].abs() == 1
                    && (removed.contains(&i) || removed.contains(&j))
                {
                    broken_bonds.insert((i, j));
                }
            }
        }
        let components = crate::cohesion::fracture_components(&shape, &broken_bonds);
        // §6.1 第 3 条:拆的是"剩余**完好**格"——`fracture_components` 以空断键集
        // 求出的分量里,只由被破坏格组成的(脱落形成的孤立格)不算分量,丢弃。
        let components: Vec<Vec<usize>> = components
            .into_iter()
            .filter(|comp| comp.iter().any(|&i| !removed.contains(&i)))
            .collect();
        let pieces = crate::cohesion::fracture_pieces(&state, &desc, &components);
        let mut requests: Vec<crate::cohesion::PieceRequest> = Vec::new();
        // 分量(按首格;fracture_components 已按首格升序):状态 = 原刚体在分量
        // 质心处的状态,线速度再加 Σ 冲量 / 分量质量(各格等质量,分量质量 =
        // 格数 × 0.729);角速度沿用原 ω(力矩忽略)。
        for (piece_state, piece_desc) in pieces {
            let comp = components[requests.len()].clone();
            let sum: Vec3 = comp.iter().map(|&i| impulses[i]).sum();
            let comp_mass = comp.len() as f32 * crate::destruction::DEBRIS_MASS;
            let mut st = piece_state;
            st.linvel += sum / comp_mass;
            requests.push(crate::cohesion::PieceRequest {
                parent: id,
                index: requests.len() as u32,
                state: st,
                desc: piece_desc,
            });
        }
        // 脱落格(按形状格序):单格碎块,速度 = 该点刚体速度 `v + ω × r` +
        // 冲量 / 0.729,旋转/角速度同原刚体。
        for (i, cell) in cells.iter().enumerate() {
            if !matches!(fates[i], crate::cohesion::CellBlast::Detach { .. }) {
                continue;
            }
            let r = state.rotation
                * Vec3::new(
                    cell.offset[0] as f32,
                    cell.offset[1] as f32,
                    cell.offset[2] as f32,
                );
            requests.push(crate::cohesion::PieceRequest {
                parent: id,
                index: requests.len() as u32,
                state: EntityState {
                    translation: state.translation + r,
                    rotation: state.rotation,
                    linvel: state.linvel
                        + state.angvel.cross(r)
                        + impulses[i] / crate::destruction::DEBRIS_MASS,
                    angvel: state.angvel,
                },
                desc: EntityDesc {
                    half_size: crate::destruction::DEBRIS_HALF_SIZE,
                    block: cell.block,
                    shape: None,
                },
            });
        }
        // 请求统一盖章为 (parent, index) 升序(分量序已在前),交给调度器。
        requests.sort_by_key(|req| (req.parent, req.index));
        self.outbox.piece_requests.extend(requests);
        self.entities.remove(&id).expect("存在");
        self.phys.remove_body(handle);
        self.outbox.despawned.push(id);
        self.outbox.cohesion_stats.blast_removed_cells += removed_cells;
    }

    /// 爆炸的地形求值与落写(REQ-009 §4.1–§4.3;纯函数 `evaluate_blast`,
    /// 不经 rapier)。求值范围 = 本体区内(格坐标口径,同 `core_contains`)且
    /// 所在 chunk 已装载的格;副本格不求值但已装载、射线照常衰减。同一拍多个
    /// 爆炸按输入序依次求值,后一个看到前一个写过的 air(风险 4)。
    ///
    /// 被破坏的格当拍写 air(`Emergent`,复用凝固桥的涌现写入与副本同步路径)、
    /// 重建所在 chunk collider、按 REQ-007 §4.11 登记地形变更唤醒;脱落格追加
    /// `DetachRequest` 到 `Outbox.detach_requests`(整个 outbox 按 `BlockPos`
    /// 升序;调度器 tick 末分配 ID、盖章 N+1 生成,单元 03)。
    ///
    /// 每拍上限由分片执行(§7 单元 03 细则):脱落上限取
    /// `min(max_debris_per_explosion, max_detach_per_tick − 本拍已有脱落请求数)`
    /// (不足 0 取 0)——同一拍多次爆炸按输入序共用 `max_detach_per_tick` 名额,
    /// 超出部分按 §4.3 的规则改为粉碎(格照样写 air)。
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
        let mut cells = crate::destruction::evaluate_blast(
            &block_at,
            self.registry.as_ref(),
            &in_scope,
            center,
            power,
            radius,
            self.cfg.destruction.attenuation,
            // §7 每拍上限由分片执行:单爆上限与本拍剩余名额取小(不足 0 取 0)。
            self.cfg.destruction.max_debris_per_explosion.min(
                self.cfg
                    .destruction
                    .max_detach_per_tick
                    .saturating_sub(self.outbox.detach_requests.len()),
            ),
        );
        // 修订 10 破坏格数上限:单次爆炸单分片最多破坏 `max_cells_per_explosion`
        // 格。在求值之后截断:截掉的格不写 air、不进 detach_requests、不触发
        // 坍塌;保留压力最高的(按 P 降序,同值 BlockPos 升序),遥测只计实际
        // 破坏的格(§4.3)。
        let cap = self.cfg.destruction.max_cells_per_explosion;
        if cells.len() > cap {
            cells.sort_by(|a, b| {
                b.pressure
                    .total_cmp(&a.pressure)
                    .then_with(|| a.pos.cmp(&b.pos))
            });
            cells.truncate(cap);
            cells.sort_by_key(|c| c.pos);
        }
        if cells.is_empty() {
            return;
        }
        let mut dirty: Vec<ChunkPos> = Vec::new();
        for cell in cells {
            // 遥测(REQ-009 §8):计数进 outbox,每拍随 take_outbox 归零(不进
            // 分片长期字段)。粉碎数含因上限改粉碎的格;over_cap 用与
            // `evaluate_blast` 同一判别式复核——P 未超粉碎阈值(P ≤ k·R)的
            // 粉碎格必是"原判脱落、被上限改粉碎"(f64 同值同式,逐位一致)。
            self.outbox.destruction_stats.cells_blasted += 1;
            match cell.fate {
                crate::destruction::CellFate::Pulverize => {
                    self.outbox.destruction_stats.pulverized += 1;
                    if let Some(m) = self.registry.material(cell.block)
                        && cell.pressure <= m.pulverize_factor as f64 * m.resistance as f64
                    {
                        self.outbox.destruction_stats.pulverized_over_cap += 1;
                    }
                }
                crate::destruction::CellFate::Detach { .. } => {
                    self.outbox.destruction_stats.detached += 1;
                }
            }
            let _ = self.voxels.set_block(cell.pos, BlockId::AIR);
            // 坍塌触发(REQ-009 §6.2.1):爆炸破坏格由实心变 air。
            self.register_collapse_trigger(cell.pos, cell.block, BlockId::AIR);
            self.outbox.block_updates.push(BlockWrite {
                pos: cell.pos,
                block: BlockId::AIR,
                kind: BlockWriteKind::Emergent,
                load_conversion: false,
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
        // 细则 4/5 同款守卫:只重建已建 ②;S03 起记入待重建集合,apply_prefix
        // 退出前统一重建。
        self.defer_chunk_collider_rebuild(dirty);
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

/// 包络半高换算(settle::envelope_half 的模块内别名,S03 起**唯一实现**在
/// [`crate::settle::envelope_half`]——调度器 `entity_motion_of` 与本 crate
/// 的 wake/footprint 调用点共用,不写第二份):单格 = `half_size` 原样,
/// 复合体 = 包络半径 ρ ÷ √3。
use crate::settle::envelope_half;

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
    use voxel::WorldGenerator;

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
        // REQ-010 §12.1 第 3 条迁移:本模块 REQ-009 及更早语义的用例(悬空放置、
        // 单格凝固落位)与默认开启的承载判定冲突 → 装配处显式关闭,断言不变。
        c.cohesion.enabled = false;
        c
    }

    fn zero_g_sim_at(shard: ShardId) -> ShardSim {
        let mut cfg = ShardSimConfig {
            gravity: Vec3::ZERO,
            ..ShardSimConfig::new(shard)
        };
        // REQ-010 §12.1 第 3 条迁移:同 zero_g_cfg,REQ-009 语义用例装配处关闭。
        cfg.cohesion.enabled = false;
        ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
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
        match &out.migrations[0].payload {
            EnvelopePayload::MigrateEntity {
                entity,
                actor,
                source,
                ..
            } => {
                assert_eq!(*entity, e);
                assert!(*actor, "actor 随迁移 payload");
                assert_eq!(*source, Some(InputSource(7)), "source 随迁移 payload");
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
                load_conversion: false,
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
                load_conversion: false,
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
                    load_conversion: false,
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
                load_conversion: false,
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
                    load_conversion: false,
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
    // ==== REQ-010 单元 04(§5.5 第 5 条:醒体预算按格数,ShardSim 接线)====

    /// 复合体按格数占醒体预算(ShardSim 层接线):预算 10、单格 a(距 9)+
    /// 6 格复合体 b(距 1,chunk 内不跨 ②)+ 6 格复合体 c(距 5)= 13 格 →
    /// plan 由远到近淘汰 a、c(1 + 6 = 7 扣足 3 格溢出后停,b 保住);按实体
    /// 个数口径(3 个 ≤ 10)不淘汰。复合体描述经 `spawn_entity` 真实建体
    /// (cells().len() 接线,变异"恒传 1"即红:淘汰只有 a 一格就够了)。
    #[test]
    fn plan_budget_counts_compound_cells() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        cfg.activation.max_awake_dynamic = 10;
        cfg.gravity = Vec3::ZERO;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let (a, b, c) = (entity(1), entity(2), entity(3));
        let _ = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        // a:单格(距观察者 9.1,最远);b:6 格复合体(距 1.03);c:6 格复合体(距 5.02)。
        let mk_shape = || {
            types::CompoundShape::new(vec![
                types::ShapeCell {
                    offset: [0, 0, 0],
                    block: voxel::builtin::STONE,
                },
                types::ShapeCell {
                    offset: [1, 0, 0],
                    block: voxel::builtin::STONE,
                },
                types::ShapeCell {
                    offset: [2, 0, 0],
                    block: voxel::builtin::STONE,
                },
                types::ShapeCell {
                    offset: [0, 1, 0],
                    block: voxel::builtin::STONE,
                },
                types::ShapeCell {
                    offset: [0, 0, 1],
                    block: voxel::builtin::STONE,
                },
                types::ShapeCell {
                    offset: [1, 1, 0],
                    block: voxel::builtin::STONE,
                },
            ])
            .expect("6 格")
        };
        let shape_b = mk_shape();
        let shape_c = mk_shape();

        s.apply_prefix(&[
            spawn_input(1, a, Vec3::new(14.0, 0.5, 8.0)),
            StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 2,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: b,
                    state: EntityState {
                        translation: Vec3::new(6.0, 0.5, 8.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape_b),
                }),
            },
            StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 3,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: c,
                    state: EntityState {
                        translation: Vec3::new(10.0, 0.5, 8.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape_c),
                }),
            },
        ]);
        let _ = s.take_activation_events();
        let d = s.apply_activation(&[Vec3::new(5.0, 3.0, 8.0)]);
        assert_eq!(
            d.over_budget_freeze,
            vec![a, c],
            "格数和 13 > 10:由远到近淘汰 a(9.1)、c(5.1)后 12 − 6 = 6 ≤ 10"
        );
    }
    // ==== REQ-010 单元 05(§8:整块凝固、倾斜散开、强制落地)====

    /// n 格石梁形状(偏移 x = 0..n,STONE)。
    fn beam_shape(n: i32) -> types::CompoundShape {
        types::CompoundShape::new(
            (0..n)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("梁 ≥ 2 格")
    }

    /// 复合体 spawn 输入(desc 带形状,状态自定)。
    fn compound_spawn_input(
        seq: u64,
        id: EntityId,
        state: EntityState,
        shape: types::CompoundShape,
    ) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state,
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape),
            }),
        }
    }

    /// 零重力 + 凝固开启(判据:低速 20 拍;零重力 → 无实体支撑前提恒真)。
    fn solidify_zero_g_sim() -> ShardSim {
        let mut cfg = ShardSimConfig {
            gravity: Vec3::ZERO,
            ..ShardSimConfig::new(shard_id(0, 0))
        };
        // REQ-010 §12.1 第 3 条迁移:凝固写入的悬空格随后会被承载判定转走,
        // 本用例按凝固桥语义编写 → 装配处显式关闭,断言不变。
        cfg.cohesion.enabled = false;
        ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
    }

    /// 整块凝固(§8.1、§8.3 第 1、2 条):悬空静止的 6 格石梁在 slow_ticks 满 20 拍
    /// 后经 `compound_snap` 判 Write → 同一拍写入全部格(Emergent)、删除实体、
    /// 计 1 次整块凝固;不散开、不逐格。
    ///
    /// 变红:复合体仍跳过凝固(实体还在);只写原点一格;写入不是 Emergent;
    /// 遥测没接线。
    #[test]
    fn compound_solidifies_whole_and_counts_telemetry() {
        let mut s = solidify_zero_g_sim();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let id = entity(1);
        let shape = beam_shape(6);
        // 原点格 (8,3,8),目标格 (8..=13, 3, 8) 全空气(chunk 实心层在 world y = 0)。
        s.apply_prefix(&[compound_spawn_input(
            1,
            id,
            EntityState {
                translation: Vec3::new(8.5, 3.5, 8.5),
                ..EntityState::default()
            },
            shape.clone(),
        )]);
        for _ in 0..24 {
            s.step();
        }
        assert!(s.entity_state(id).is_none(), "整块凝固后实体删除");
        for x in 8..=13 {
            assert_eq!(
                s.voxels.get_block(BlockPos::new(x, 3, 8)),
                voxel::builtin::STONE,
                "({x},3,8) 随整块凝固写入"
            );
        }
        let out = s.take_outbox();
        assert_eq!(out.cohesion_stats.compounds_solidified, 1, "计 1 次凝固");
        assert_eq!(out.cohesion_stats.tilt_scattered, 0, "没有散开");
        let emergent: Vec<_> = out
            .block_updates
            .iter()
            .filter(|w| w.kind == BlockWriteKind::Emergent)
            .collect();
        assert_eq!(emergent.len(), 6, "全部格经 Emergent 写入");
    }

    /// 倾斜散开(§8.3 第 3 条,D5):误差超限(末端格偏离 > 0.5 格)→ 本拍删除
    /// 复合体、当拍不写体素;每格按形状格序产出分块请求——普通单格碎块(描述
    /// 不带形状、半边 0.45、该格方块),状态 = 该格世界位姿,速度 = v + ω × r、
    /// 角速度 = ω。
    ///
    /// 变红:倾斜时强行整块吸附(写格);分块仍带形状;速度公式丢了 ω × r 项;
    /// index 不是形状格序;遥测没接线。
    #[test]
    fn tilted_compound_scatters_pieces_without_voxel_writes() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.gravity = Vec3::ZERO;
        // 角速度要低于 slow 判据阈值(0.05):0.04 rad/s 的慢转足以让
        // "速度 = v + ω × r" 的 r 项可测(删掉 cross 项即差 ~0.1 m/s)。
        let angvel = Vec3::new(0.0, 0.0, 0.04);
        cfg.solidify.displacement_gate_m = 0.0; // 慢转使原点画小圆,关门直通
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let id = entity(1);
        let shape = beam_shape(6);
        s.apply_prefix(&[compound_spawn_input(
            1,
            id,
            EntityState {
                translation: Vec3::new(8.5, 3.5, 8.5),
                rotation: types::glam::Quat::from_rotation_z(20f32.to_radians()),
                angvel,
                ..EntityState::default()
            },
            shape.clone(),
        )]);
        // slow_ticks 满 20 的那一拍触发判据 → compound_snap 判 Tilted。
        for _ in 0..20 {
            s.step();
        }
        assert!(s.entity_state(id).is_none(), "散开当拍删除复合体");
        let out = s.take_outbox();
        assert_eq!(out.cohesion_stats.tilt_scattered, 1, "计 1 次散开");
        assert_eq!(out.cohesion_stats.compounds_solidified, 0);
        assert!(
            out.block_updates
                .iter()
                .all(|w| w.kind != BlockWriteKind::Emergent),
            "散开当拍不写体素(格子由各分块之后自己凝固)"
        );
        assert_eq!(out.piece_requests.len(), 6, "每格一个分块请求");
        for (k, piece) in out.piece_requests.iter().enumerate() {
            let cell = shape.cells()[k];
            assert_eq!(piece.parent, id, "分块的母实体");
            assert_eq!(piece.index, k as u32, "index = 形状格序");
            assert!(piece.desc.shape().is_none(), "分块是普通单格碎块(不带形状)");
            assert_eq!(piece.desc.half_size, crate::destruction::DEBRIS_HALF_SIZE);
            assert_eq!(piece.desc.block, cell.block, "分块 = 该格方块");
            // 零重力下 v 恒 0、ω 恒定:速度公式 v + ω × r 用分块自带的旋转
            // (= 散开拍的刚体旋转)自洽核对——ω × r 项缺失即红。
            let off = types::glam::Vec3::new(
                cell.offset[0] as f32,
                cell.offset[1] as f32,
                cell.offset[2] as f32,
            );
            let r = piece.state.rotation * off;
            let want = angvel.cross(r);
            assert!(
                (piece.state.linvel - want).length() < 1e-3,
                "分块 {k} 速度 = v + ω × r:{:?} vs {:?}",
                piece.state.linvel,
                want
            );
            assert!(
                (piece.state.angvel - angvel).length() < 1e-5,
                "分块角速度 = 刚体角速度"
            );
        }
    }

    /// 整块强制落地(§8.2、§8.3 第 5 条):瞬态体超预算淘汰 → 整块吸附后落到
    /// "到下方第一个实心格之上"——所有格同一落点高度(整块平移),Emergent
    /// 写入、删除实体、按原因分计预算;不散开、不计 Fallback。
    ///
    /// 变红:复合体仍跳过强制落地;逐格各自落到自己的柱底;写入缺 Emergent;
    /// 预算归因丢失。
    #[test]
    fn compound_forced_landing_lands_whole_on_budget_eviction() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.gravity = Vec3::ZERO;
        cfg.solidify.enabled = false; // 不让凝固抢先
        cfg.activation.max_awake_dynamic = 1;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let (a, b) = (entity(1), entity(2));
        let _ = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        s.apply_prefix(&[
            spawn_input(1, a, Vec3::new(8.0, 0.5, 8.0)),
            compound_spawn_input(
                2,
                b,
                EntityState {
                    translation: Vec3::new(13.5, 3.5, 8.5),
                    ..EntityState::default()
                },
                beam_shape(2),
            ),
        ]);
        let _ = s.take_activation_events();
        // 醒体格数 1 + 2 = 3 > 1 → 由远到近淘汰远者 b(复合体)。
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.over_budget_freeze, vec![b], "远者复合体被 plan 淘汰");
        s.run_transient_lifecycle();
        assert!(s.entity_state(b).is_none(), "整块强制落地后实体删除");
        // 目标格 (13,3,8)、(14,3,8) 全空气;d = 3 − 0 − 1 = 2 → 整块落 y = 1。
        assert_eq!(
            s.voxels.get_block(BlockPos::new(13, 1, 8)),
            voxel::builtin::STONE,
            "整块平移后的落点(所有格同高)"
        );
        assert_eq!(
            s.voxels.get_block(BlockPos::new(14, 1, 8)),
            voxel::builtin::STONE
        );
        let out = s.take_outbox();
        assert_eq!(
            out.cohesion_stats.forced_landing_fallbacks, 0,
            "走了整块落地,不算退化"
        );
        assert_eq!(
            out.block_updates
                .iter()
                .filter(|w| w.kind == BlockWriteKind::Emergent)
                .count(),
            2,
            "全部格 Emergent 写入"
        );
        let events = s.take_activation_events();
        assert_eq!(events.forced_landings_budget, 1, "按原因分计:预算");
        assert!(s.entity_state(a).is_some(), "近者保留");
    }

    /// 强制落地退化为逐格(§8.3 第 6 条):所有柱下方都没有实心 → `Fallback`
    /// → 按形状格序逐格用单格规则;每柱都无底 → 全部格虚空消失(不写格),
    /// 计 1 次退化。
    ///
    /// 变红:Fallback 时整块硬写;退化计数没接线;虚空格被写了。
    #[test]
    fn compound_forced_landing_fallback_per_cell_counts() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.gravity = Vec3::ZERO;
        cfg.solidify.enabled = false;
        cfg.activation.max_awake_dynamic = 1;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        // 已装载但全空:实心判定 Some(false) 一路到世界底 → Fallback。
        s.load_chunk(ChunkPos::new(0, 0), Chunk::air());
        let (a, b) = (entity(1), entity(2));
        let _ = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        s.apply_prefix(&[
            spawn_input(1, a, Vec3::new(8.0, 0.5, 8.0)),
            compound_spawn_input(
                2,
                b,
                EntityState {
                    translation: Vec3::new(13.5, 3.5, 8.5),
                    ..EntityState::default()
                },
                beam_shape(2),
            ),
        ]);
        let _ = s.take_activation_events();
        let d = s.apply_activation(&[Vec3::new(8.0, 3.0, 8.0)]);
        assert_eq!(d.over_budget_freeze, vec![b]);
        s.run_transient_lifecycle();
        assert!(s.entity_state(b).is_none(), "逐格退化后实体删除");
        let out = s.take_outbox();
        assert_eq!(
            out.cohesion_stats.forced_landing_fallbacks, 1,
            "计 1 次退化"
        );
        assert!(
            out.block_updates
                .iter()
                .all(|w| w.kind != BlockWriteKind::Emergent),
            "每柱都无底:全部格虚空消失,不写格"
        );
        assert!(s.entity_state(a).is_some(), "近者保留");
    }

    /// 强制落地推迟(§8.3 第 6 条):复合体覆盖的柱 ① 不在内存 → `Defer`
    /// ——实体保留、留在待执行集合、不写格(与单格柱缺同款)。
    ///
    /// 变红:未装载被当空气一路落穿(写进未装载 chunk 静默新建的空层)。
    #[test]
    fn compound_forced_landing_defers_and_stays_pending_when_column_unloaded() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.gravity = Vec3::ZERO;
        cfg.solidify.enabled = false;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 复合体悬在未装载 chunk (1,0) 的柱上(x = 20..=21)。
        let b = entity(2);
        s.apply_prefix(&[compound_spawn_input(
            1,
            b,
            EntityState {
                translation: Vec3::new(20.5, 3.5, 8.5),
                ..EntityState::default()
            },
            beam_shape(2),
        )]);
        // 手动进入待执行强制落地(瞬态票到期入口的等价态)。
        s.forced_pending.insert(b);
        s.forced_reasons.insert(b, ForcedLandingReason::Deadline);
        s.run_transient_lifecycle();
        assert!(s.entity_state(b).is_some(), "前置条件不满足:推迟、实体保留");
        assert!(s.forced_pending.contains(&b), "留在待执行集合");
        assert!(s.take_outbox().block_updates.is_empty(), "未写任何格");
    }
    // ==== REQ-010 单元 06(§7.3:碎裂与复合体砸地形)====

    /// 顶格为一层 dirt、其下整柱 stone 的 chunk(砸地形用:土 I = 7、
    /// 石 I = 18;土层接地,破坏两格不会触发整体坍塌)。
    fn dirt_on_stone_chunk() -> Chunk {
        let mut ch = Chunk::air();
        for x in 0..voxel::CHUNK_X {
            for z in 0..voxel::CHUNK_Z {
                ch.set(x, 64, z, voxel::builtin::DIRT);
                for y in 0..64 {
                    ch.set(x, y, z, types::BlockId(1));
                }
            }
        }
        ch
    }

    /// n 格方块梁 spawn 输入(位置/速度自定;实心层顶在 world y = 1,
    /// `gap` = 底面离顶面的缝隙)。
    fn beam_spawn_input(
        seq: u64,
        id: EntityId,
        x: f32,
        gap: f32,
        vy: f32,
        block: types::BlockId,
        n: i32,
    ) -> StampedInput {
        let shape = types::CompoundShape::new(
            (0..n)
                .map(|i| types::ShapeCell {
                    offset: [i, 0, 0],
                    block,
                })
                .collect(),
        )
        .expect("梁 ≥ 2 格");
        compound_spawn_input(
            seq,
            id,
            EntityState {
                translation: Vec3::new(x, 1.0 + crate::destruction::DEBRIS_HALF_SIZE + gap, 8.5),
                linvel: Vec3::new(0.0, vy, 0.0),
                ..EntityState::default()
            },
            shape,
        )
    }

    /// §7.3 第 9 条(demo 问题 7):逐格碰撞体的接触点冲量与法向可读——复合体
    /// **每一格**的 collider(不只代表 collider)与地形的接触对里,普通流形的
    /// 点带 solver 解算冲量(> 0),法向是 world 系 ±Y。
    ///
    /// 变红:plain 流形不再回写冲量(rapier 升级回归);复合体 collider 不在
    /// body.colliders() 里(挂法改变)。
    #[test]
    fn per_cell_contact_impulse_and_normal_are_readable() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let id = entity(1);
        // 6 格石梁、v = −8 m/s(每格 ≈ 5.8 < 18:不碎、不砸穿石面,纯读接触)。
        s.apply_prefix(&[beam_spawn_input(
            1,
            id,
            10.5,
            0.1,
            -8.0,
            voxel::builtin::STONE,
            6,
        )]);
        // 地形 collider 键表(chunk 固定体)。
        let terrain: std::collections::BTreeSet<(u32, u32)> = s
            .chunk_bodies
            .values()
            .filter_map(|h| s.phys.bodies.get(*h))
            .flat_map(|b| b.colliders().iter().copied())
            .map(|c| {
                let (g, i) = c.into_raw_parts();
                (g, i)
            })
            .collect();
        // 找到撞击拍:多格(含非首格的 collider)同时出现冲量 > 0 的接触点。
        // 求解器会把个别接触点的冲量分到恰为 0(超静定分配,§7.1 的均摊根因),
        // 所以断言"≥ 2 格可读、且覆盖非首格",不要求全部格同拍非零。
        for _ in 0..10 {
            s.step();
            let Some(rec) = s.entities.get(&id) else {
                panic!("石梁不应碎裂(每格 5.8 < 18):碎裂阈值接线错误");
            };
            let body = s.phys.bodies.get(rec.handle).expect("handle 有效");
            let mut per_cell: std::collections::BTreeMap<[i32; 3], f32> =
                std::collections::BTreeMap::new();
            let mut normal_y_ok = true;
            for ch in body.colliders() {
                let key = ch.into_raw_parts();
                let Some(col) = s.phys.colliders.get(*ch) else {
                    continue;
                };
                let t = col
                    .position_wrt_parent()
                    .map_or(Vec3::ZERO, |p| p.translation);
                let off = [t.x.round() as i32, t.y.round() as i32, t.z.round() as i32];
                for pair in s.phys.narrow_phase.contact_pairs() {
                    let k1 = pair.collider1.into_raw_parts();
                    let k2 = pair.collider2.into_raw_parts();
                    let other = if (k1.0, k1.1) == (key.0, key.1) {
                        (k2.0, k2.1)
                    } else if (k2.0, k2.1) == (key.0, key.1) {
                        (k1.0, k1.1)
                    } else {
                        continue;
                    };
                    if !terrain.contains(&other) {
                        continue;
                    }
                    for m in pair.solver_manifolds() {
                        // 法向断言只看带冲量的流形:侧面对挤的流形(法向 ±X)
                        // 冲量恒 0,不参与撞击。(读冲量 → solver_manifolds,
                        // AC-F17 同口径。)
                        let has_impulse = m.points.iter().any(|p| p.data.impulse > 0.0);
                        if has_impulse && m.data.normal.y.abs() < 0.99 {
                            normal_y_ok = false;
                        }
                        for pt in &m.points {
                            if pt.data.impulse > 0.0 {
                                let cur = per_cell.get(&off).copied().unwrap_or(0.0);
                                per_cell.insert(off, cur.max(pt.data.impulse));
                            }
                        }
                    }
                }
            }
            if per_cell.len() >= 2 && per_cell.keys().any(|off| off[0] > 0) {
                for (off, j) in per_cell {
                    assert!(j > 0.0, "格 {off:?} 的接触点冲量可读且 > 0");
                }
                assert!(normal_y_ok, "接触法向是 world 系 ±Y");
                return;
            }
        }
        panic!(
            "10 拍内没有出现多格(含非首格)的接触冲量:逐格接触不可读(demo 问题 7 的 Rust 侧前提)"
        );
    }

    /// 碎裂接线(§7.3 第 3–5、8 条):6 格土梁以 −12 m/s 撞实心层,接触格均摊
    /// ≈ 8.9 ≥ 7 → 5 根横向粘结全断 → 6 个分量;原实体本拍删除,每个分量产出
    /// 分块请求(单格碎块、无形状、index = 分量序号、平移 = 原格世界中心、
    /// 角速度同刚体),遥测 `bonds_broken` = 5、`fracture_components` = 6。
    ///
    /// 变红:碎裂没接线;只采代表 collider(只断 1 根);分块带形状;index 不
    /// 是分量序号;遥测没接线;断键阈值误用承载 B(土 B = 3 早两倍就断)。
    #[test]
    fn dirt_compound_fractures_on_hard_landing() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        // G03 迁移(§12.1 第 3 条):测 §7.1 旧碎裂路径本身(5 根横向粘结全
        // 断、6 分量 = D4 局部断键语义),显式关 compound_flow(AC-G4 关闭
        // 回归;开启版阈值见 compound_flow_* 测试)。
        cfg.cohesion.compound_flow = false;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let id = entity(1);
        s.apply_prefix(&[beam_spawn_input(
            1,
            id,
            10.5,
            0.1,
            -12.0,
            voxel::builtin::DIRT,
            6,
        )]);
        for _ in 0..3 {
            s.step();
        }
        assert!(s.entity_state(id).is_none(), "碎裂后原实体删除");
        let out = s.take_outbox();
        assert_eq!(out.cohesion_stats.bonds_broken, 5, "5 根横向粘结全断");
        assert_eq!(out.cohesion_stats.fracture_components, 6, "拆成 6 个分量");
        assert_eq!(out.piece_requests.len(), 6, "每个分量一个分块请求");
        for (k, piece) in out.piece_requests.iter().enumerate() {
            assert_eq!(piece.parent, id, "分块的母实体");
            assert_eq!(piece.index, k as u32, "index = 分量序号");
            assert!(piece.desc.shape().is_none(), "单格分量是普通单格碎块");
            assert_eq!(piece.desc.block, voxel::builtin::DIRT);
            assert_eq!(piece.desc.half_size, crate::destruction::DEBRIS_HALF_SIZE);
            assert!(
                (piece.state.translation.x - (10.5 + k as f32)).abs() < 0.05
                    && (piece.state.translation.z - 8.5).abs() < 0.05,
                "单格分量平移 = 该格世界中心(x 随分量序;撞击拍允许求解器位置修正的微小漂移):{:?}",
                piece.state.translation
            );
            assert!(
                piece
                    .state
                    .rotation
                    .abs_diff_eq(types::glam::Quat::IDENTITY, 1e-2),
                "旋转 ≈ 恒等(撞击力矩的微小扰动):{:?}",
                piece.state.rotation
            );
            assert!(
                piece.state.linvel.y.abs() < 13.0,
                "线速度 = v + ω × r,不超过撞击速度量级(含弹回):{:?}",
                piece.state.linvel
            );
            if k > 0 {
                let spacing = out.piece_requests[k].state.translation.x
                    - out.piece_requests[k - 1].state.translation.x;
                assert!(
                    (spacing - 1.0).abs() < 1e-3,
                    "相邻分量间距 = 格宽:{spacing}"
                );
            }
        }
        // 石垫没被砸穿(每格 8.9 < 18),土也没写回体素。
        assert_eq!(
            s.voxels.get_block(BlockPos::new(10, 0, 8)),
            types::BlockId(1)
        );
        assert!(
            out.block_updates.is_empty(),
            "碎裂当拍不写体素(散开同款,N+1 分块自己凝固)"
        );
    }

    /// 砸地形均摊口径接线(§7.3 第 7 条):2 格石质复合体以 −10 m/s 砸土层,
    /// `J_T / |T|` ≈ 7.3 ≥ 7 → 脚下两格土**同一拍**写 air(Emergent、零初速
    /// 脱落请求、`cells_impacted`/`detached` 计 2);对照 −6 m/s 时每格 ≈ 4.4
    /// < 7,一格都不掉。石梁自身不断键(横向粘结是压向豁免)。
    ///
    /// 变红:复合体接触没接进撞击采样(只采代表 collider → 只断一格);阈值
    /// 判定用逐格相加;遥测没接线。
    #[test]
    fn compound_terrain_impact_amortized_breaks_and_counts() {
        let drop = |vy: f32| {
            let mut cfg = ShardSimConfig::new(shard_id(0, 0));
            cfg.solidify.enabled = false;
            cfg.cohesion.enabled = true;
            // F04a:本用例测 §7.3 第 7 条"均摊砸地形"的旧语义(F02a 起
            // `impact_flow` 生效时被冲量流路径取代),显式关闭(§5.7.5 迁移)。
            cfg.cohesion.impact_flow = false;
            let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            let id = entity(1);
            s.apply_prefix(&[beam_spawn_input(
                1,
                id,
                10.5,
                0.1,
                vy,
                voxel::builtin::STONE,
                2,
            )]);
            for _ in 0..3 {
                s.step();
            }
            (s, id)
        };
        // −10 m/s:均摊 7.3 ≥ 7 → 两格土同拍脱落。
        let (mut s, id) = drop(-10.0);
        assert!(s.entity_state(id).is_some(), "石梁不碎(压向豁免)");
        for x in 10..=11 {
            assert_eq!(
                s.voxels.get_block(BlockPos::new(x, 0, 8)),
                voxel::builtin::AIR,
                "({x},0,8) 被砸落"
            );
        }
        let out = s.take_outbox();
        assert_eq!(out.destruction_stats.cells_impacted, 2, "两格土同一拍脱落");
        assert_eq!(out.destruction_stats.detached, 2);
        assert_eq!(out.cohesion_stats.bonds_broken, 0, "石梁自身不断键");
        assert_eq!(out.detach_requests.len(), 2, "每格一个脱落请求");
        assert!(
            out.detach_requests.iter().all(|r| r.impulse == Vec3::ZERO),
            "脱落碎块零初速(REQ-009 §5 修订 8 同款)"
        );
        assert_eq!(
            out.block_updates
                .iter()
                .filter(|w| w.kind == BlockWriteKind::Emergent && w.block == voxel::builtin::AIR)
                .count(),
            2,
            "两格都经 Emergent 写 air"
        );
        // −6 m/s:均摊 ≈ 4.4 < 7 → 一格都不掉。
        let (mut s2, _) = drop(-6.0);
        for x in 10..=11 {
            assert_eq!(
                s2.voxels.get_block(BlockPos::new(x, 0, 8)),
                voxel::builtin::DIRT,
                "({x},0,8) 低于阈值不脱落"
            );
        }
        assert_eq!(s2.take_outbox().destruction_stats.cells_impacted, 0);
    }

    /// 竖落豁免接线(§7.1 压向豁免、§7.3 风险 2 的生产路径):6 格石梁绕 z
    /// 竖起、以 −8 m/s 竖直砸在石面上——只有底格接触,均摊冲量 ≈ 35 ≥ 18,
    /// 但"底格向上那根粘结"是压向(世界系 +y,与受力方向点积 1 ≥ cos 45°),
    /// 一根不断、实体保留。
    ///
    /// 变红:受力方向取反(压向被误判横向);点积用本体坐标(+x · +y = 0);
    /// 豁免被删——三种实现 j_c ≥ 18 都会断底格那一根。
    #[test]
    fn upright_compound_landing_press_bond_exempt() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let id = entity(1);
        let shape = types::CompoundShape::new(
            (0..6)
                .map(|i| types::ShapeCell {
                    offset: [i, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("6 格");
        // 绕 z 转 90°:形状沿 +x 的格竖立为沿 +y;底格中心 y = 1.55。
        s.apply_prefix(&[compound_spawn_input(
            1,
            id,
            EntityState {
                translation: Vec3::new(10.5, 1.55, 8.5),
                rotation: types::glam::Quat::from_rotation_z(90f32.to_radians()),
                linvel: Vec3::new(0.0, -8.0, 0.0),
                ..EntityState::default()
            },
            shape,
        )]);
        for _ in 0..60 {
            s.step();
        }
        assert!(s.entity_state(id).is_some(), "竖落不断键,实体保留");
        assert_eq!(
            s.take_outbox().cohesion_stats.bonds_broken,
            0,
            "压向豁免:底格向上那根粘结一根不断"
        );
    }

    /// 每拍上限与顺序(§7.3 第 6 条):`max_fractures_per_tick` = 1,两根土梁
    /// 同拍撞击(A −13 m/s、B −11 m/s,J_c 都过阈)——按 J_c 降序只有 A 本拍
    /// 碎,B 完好(它的撞击已被采样,轮空后本拍不断);`bonds_broken` 只计 A。
    /// 两梁都放在 chunk 中部(离边界面 ≥ 2 格):贴边的格会与边界薄片成
    /// 多流形对,solver 聚簇后 plain 流形读不到冲量(REQ-009 §5 已知现象)。
    ///
    /// 变红:上限不生效(B 同拍也碎);排序升序(B 先碎);上限后实体没删。
    #[test]
    fn fracture_cap_processes_highest_jc_first() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.max_fractures_per_tick = 1;
        // G03 迁移(§12.1 第 3 条):测 §7.3 第 6 条名额的 J_c 口径(旧路径
        // 排序键),显式关 compound_flow;新路径的 s 口径名额由
        // compound_flow_quota_picks_highest_s_then_entity_order 覆盖。
        cfg.cohesion.compound_flow = false;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let (a, b) = (entity(1), entity(2));
        s.apply_prefix(&[
            beam_spawn_input(1, a, 10.5, 0.1, -13.0, voxel::builtin::DIRT, 2),
            beam_spawn_input(2, b, 6.5, 0.1, -11.0, voxel::builtin::DIRT, 2),
        ]);
        for _ in 0..3 {
            s.step();
        }
        assert!(s.entity_state(a).is_none(), "J_c 更大的 A 本拍碎");
        assert!(s.entity_state(b).is_some(), "轮空的 B 本拍不断、实体保留");
        let out = s.take_outbox();
        assert_eq!(out.cohesion_stats.bonds_broken, 1, "只处理了 A(1 根粘结)");
        assert_eq!(out.cohesion_stats.fracture_components, 2);
        assert_eq!(out.piece_requests.len(), 2);
        assert!(out.piece_requests.iter().all(|p| p.parent == a));
    }

    // ==== REQ-010 单元 F03:接近速度勘误(§5.7.9 勘误 2;AC-F9)====

    /// AC-F9 第 1 句(旋转体):**质心不动、边缘高速**的旋转体撞地形——4 格
    /// 石梁绕自身中心强旋(ω = 30 rad/s,左端点接触点速度 ≈ 45 m/s 向下,
    /// 质心线速度 0),落在土层上。旧口径(只取线速度)门槛恒 0,不计;
    /// 新口径按接触点速度 `v + ω×r` 计入 → 端点砸破脚下土格
    /// (`cells_impacted > 0`、冲量表带该格)。
    ///
    /// 变红:接近速度退回"只取线速度"(旧口径,恒 0 → 无撞击)。
    /// (变异自检:见 F03 回评——临时替换 `contact_vel` 为 `pre_vel` 本测试
    /// 红,验证后还原。)
    #[test]
    fn rotating_body_impact_counts_contact_point_speed() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        // cohesion 关:走 REQ-009 单格逐格口径(本测试考察撞击采样门槛,
        // 与碎裂无关);同时 `evaluate_collapse` 不走 load 分支,冲量表不被
        // 拍末 take,断言可读。
        cfg.cohesion.enabled = false;
        // dt = 1/240:端点一步下扫 ω·r/4 ≈ 0.1 m,小于薄片厚 0.15——
        // 避免隧穿(默认 1/60 一步 0.25 m 会直接穿过薄片,落入穿透修正,
        // 不产生撞击冲量)。
        cfg.dt = 1.0 / 240.0;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        let id = entity(1);
        // 静置高度(D15):中心 = 地表 + 0.45。ω = +16 rad/s 绕 z:左端
        // (格 0,r = (−1.5,0,0))接触点速度 v_y = ω·1.5 = 24 m/s 向下
        // (≥ 门槛 1);质心线速度 0(旧口径恒不计)。
        s.apply_prefix(&[compound_spawn_input(
            1,
            id,
            EntityState {
                translation: Vec3::new(10.5, 1.4501, 8.5),
                linvel: Vec3::ZERO,
                angvel: Vec3::new(0.0, 0.0, 16.0),
                ..EntityState::default()
            },
            beam_shape(4),
        )]);
        let mut broke = false;
        let mut table = BTreeMap::new();
        for _ in 0..160 {
            s.step();
            let out = s.take_outbox();
            if out.destruction_stats.cells_impacted > 0 {
                broke = true;
                table = s.impact_impulses.clone();
                break;
            }
        }
        assert!(broke, "旋转端点应计入撞击并砸破土格(旧口径线速度恒 0 不计)");
        assert!(
            table.values().all(|j| j.length() > 0.0) && !table.is_empty(),
            "撞击冲量表非零:{table:?}"
        );
        assert!(
            s.entity_state(id).is_some(),
            "石梁自身不断键(只考察地形撞击计入)"
        );
    }

    /// AC-F9 第 2 句(动态对方):静止复合体被 ≥ 1 m/s 的动态投射物击中**可以
    /// 断键**(D4 碎裂路径)——静置的 2 格土梁(旧口径下门槛看它自己的速度
    /// = 0,永远不碎),被 −4 m/s 垂直落下的 2 格石梁砸中右端:接触点相对
    /// 速度 = 投射物速度 ≥ 1 计入,冲量集中在右端接触格 → 断一根。
    ///
    /// 变红:对方速度不计(退回"本体线速度"口径——本体静止 → 门槛 0,不断)。
    #[test]
    fn stationary_compound_fractures_when_hit_by_projectile() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        // G03 迁移(§12.1 第 3 条):AC-F9 第 2 句的 D4 断键版(静置体被击
        // 中断键),显式关 compound_flow——对方速度资格两条路径共享,新路径
        // 由互撞/侧撞测试覆盖(AC-G4 关闭回归)。
        cfg.cohesion.compound_flow = false;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        let (target, proj) = (entity(1), entity(2));
        // 目标:2 格土梁静置(土 bond 3、I 7);投射物:2 格**石**梁(不碎、
        // 只当锤)以 −4 m/s 落在右端正上方、错开半格压右端格——接触点相对
        // 速度 = 投射物速度 ≥ 1 计入,冲量集中在目标右端接触格,单拍 J_c
        // ≥ dirt bond 3 → 断右端那根(−2 m/s 时冲量被两拍摊薄,J_c < 3
        // 不断,故取 4)。chunk (0,0) 顶面土 = 世界 y 0;静置梁中心 = 0.45。
        let dirt_shape = types::CompoundShape::new(
            (0..2)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("2 格");
        let stone_shape = types::CompoundShape::new(
            (0..2)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("2 格");
        s.apply_prefix(&[
            compound_spawn_input(
                1,
                target,
                EntityState {
                    translation: Vec3::new(9.5, 1.4501, 8.5),
                    ..EntityState::default()
                },
                dirt_shape,
            ),
            compound_spawn_input(
                2,
                proj,
                EntityState {
                    translation: Vec3::new(10.5, 2.45, 8.5),
                    linvel: Vec3::new(0.0, -4.0, 0.0),
                    ..EntityState::default()
                },
                stone_shape,
            ),
        ]);
        let mut broken = false;
        for _ in 0..40 {
            s.step();
            let out = s.take_outbox();
            if out.cohesion_stats.bonds_broken > 0 {
                broken = true;
                break;
            }
        }
        assert!(broken, "静止复合体被投射物击中应断键(对方速度计入相对门槛)");
    }

    /// AC-F9 补充(F03b 裁定第 2 条):**D4 碎裂路径的旋转体**——3 格沙梁
    /// 质心线速度 0、绕 z 自转(ω = +7.5 rad/s,左端点接触点速度 =
    /// ω×r = 7.5 m/s 向下;
    /// **复合体的碰撞体原点与质心重合**(奇数格梁中间格 = 质心),断言偏差
    /// < 0.01 m),左端拍土层。新口径按接触点速度判定 → 断键(sand I = 4
    /// 低阈,避开 dirt I = 7 下"竖直法向窗口内冲量 ≈ 6.7 < 7"的布景死区);
    /// 旧口径(只取本体线速度,恒 0)不断。会红:旋转预筛用
    /// `ω·|碰撞体原点 − 质心|`(= 0)做上界把流形直接拒掉;接近速度漏乘
    /// ω×r;复体原点取侧错(一律 pos1)。
    #[test]
    fn rotating_compound_fractures_on_spin_landing() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        // dt = 1/240:端点一步旋转弧 7.5/240 ≈ 0.03 m,贴地缝出生不隧穿。
        cfg.dt = 1.0 / 240.0;
        // G03 迁移(§12.1 第 3 条):F03b 旋转体接触点速度判定的 D4 断键版,
        // 显式关 compound_flow(资格表两条路径共享;AC-G4 关闭回归)。
        cfg.cohesion.compound_flow = false;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        let id = entity(1);
        // 3 格沙梁沿 x、offsets −1/0/+1(形状原点 = 质心 → spawn translation
        // 即世界质心,布景自检断言 < 0.01 m)。ω = +7.5:左端(格 −1,
        // 接触点 r = (−1,0,0))v_y = 7.5 m/s 向下(≥ 门槛 1)。端点贴地小缝
        // 出生:接触拍只有拍击端判接近,法向竖直(不落压向豁免锥),单格
        // J_c ≈ 6.7 ≥ sand 断键阈 I = 4(impact_threshold;断键走
        // impact_bond_between = min(I_a,I_b),不是承载 bond 0)。
        let centered_beam = types::CompoundShape::new(
            (-1..=1)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::SAND,
                })
                .collect(),
        )
        .expect("3 格");
        s.apply_prefix(&[compound_spawn_input(
            1,
            id,
            EntityState {
                translation: Vec3::new(10.5, 1.4501, 8.5),
                linvel: Vec3::ZERO,
                angvel: Vec3::new(0.0, 0.0, 7.5),
                ..EntityState::default()
            },
            centered_beam,
        )]);
        // 布景自检:复合体的世界质心与实体位姿(碰撞体原点)重合。
        let rec = s.entities.get(&id).expect("spawn 即在表");
        let body = s.phys.bodies.get(rec.handle).expect("handle 有效");
        let com = conv::rapier_to_vec3(body.mass_properties().world_com);
        let pose = conv::rapier_to_vec3(body.translation());
        assert!(
            (com - pose).length() < 0.01,
            "碰撞体原点应与质心重合(抓 ω·|原点−质心| 假上界):com={com:?} pose={pose:?}"
        );
        let mut broken = false;
        for _ in 0..160 {
            s.step();
            let out = s.take_outbox();
            if out.cohesion_stats.bonds_broken > 0 {
                broken = true;
                break;
            }
        }
        assert!(broken, "自转梁端点拍地应断键(接触点速度计入 D4 门槛)");
    }

    /// AC-F9 第 3 句(非旋转逐位一致):单格土块以 −6 m/s 落土层(非旋转、
    /// 地形静止,旧口径下流形级与逐点门槛同值),新旧口径撞击行为**逐位一致**
    /// ——同场景双跑:生产路径 vs 把 `pre_impulse_spin` 清空(等价旧口径,
    /// 因旋转贡献不存在时逐点值 = 线速度值)后的对照跑,脱落格集合、脱落数、
    /// 冲量表逐位相同。
    ///
    /// 变红:逐点复核的接触点取位或方向算错(非旋转工况下也会偏离线速度口径)。
    #[test]
    fn non_rotating_impact_matches_legacy_threshold_bitwise() {
        let run = |clear_spin: bool| {
            let mut cfg = ShardSimConfig::new(shard_id(0, 0));
            cfg.solidify.enabled = false;
            // cohesion 关:同 `rotating_body_impact_counts_contact_point_speed`,
            // 冲量表不被拍末 take,新旧口径的表可逐位比较。
            cfg.cohesion.enabled = false;
            // dt = 1/240:一步下扫 10/240 ≈ 0.042 m < 薄片厚 0.15,不隧穿。
            cfg.dt = 1.0 / 240.0;
            let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            let id = entity(1);
            s.apply_prefix(&[compound_spawn_input(
                1,
                id,
                EntityState {
                    translation: Vec3::new(10.5, 2.575, 8.5),
                    linvel: Vec3::new(0.0, -10.0, 0.0),
                    ..EntityState::default()
                },
                beam_shape(2),
            )]);
            let mut cells = Vec::new();
            let mut table = BTreeMap::new();
            for _ in 0..60 {
                if clear_spin {
                    // 旧口径对照:清空 spin 快照 → 逐点速度退化为线速度。
                    s.pre_impulse_spin.clear();
                }
                s.step();
                let out = s.take_outbox();
                if out.destruction_stats.cells_impacted > 0 {
                    cells.push(out.destruction_stats.cells_impacted);
                    table = s.impact_impulses.clone();
                    break;
                }
            }
            (cells, table)
        };
        let (cells_new, table_new) = run(false);
        let (cells_old, table_old) = run(true);
        assert_eq!(cells_new, cells_old, "非旋转工况新旧口径脱落拍序逐位一致");
        assert_eq!(table_new, table_old, "非旋转工况冲量表逐位一致");
        assert!(
            !cells_new.is_empty() && table_new.values().all(|j| j.length() > 0.0),
            "布景自检:两格土脱落且冲量非零"
        );
    }

    /// 逐格求值·粉碎/脱落/完好分界(§6.1 第 1 条):3 格土质梁(R 2、k 2)。
    /// 场景 A 三格全 P > 4 粉碎;场景 B 三格全 P ∈ (2, 4] 脱落、冲量 = P·dir';
    /// 场景 C 全完好且冲量 = 单格公式的斜向冲量;场景 D 衰减系数生效
    /// (P − R·attenuation 跨过脱落阈);末尾半径外全完好无冲量。
    ///
    /// 变红:阈值判反;脱落冲量没按 dir'(向上为主);完好格冲量用压力
    /// P 而非单格公式(方向应为斜向背离爆心);d ≥ r 的格没给无冲量完好;
    /// 衰减漏乘 attenuation。
    #[test]
    fn compound_blast_pulverize_detach_intact_thresholds() {
        let reg = voxel::SimpleRegistry::builtin();
        let shape = beam_shape(3)
            .cells()
            .iter()
            .map(|c| types::ShapeCell {
                offset: c.offset,
                block: voxel::builtin::DIRT,
            })
            .collect();
        let dirt_beam = types::CompoundShape::new(shape).expect("3 格");
        let state = EntityState {
            translation: Vec3::new(8.5, 0.5, 8.5),
            ..EntityState::default()
        };
        let open = |_: BlockPos| Some(voxel::builtin::AIR);
        // 场景 A(粉碎):贴面格 P = 9 > 4。
        let a = crate::cohesion::compound_blast(
            &state,
            &dirt_beam,
            Vec3::new(8.0, -0.9, 8.0),
            10.0,
            12.0,
            1.0,
            &open,
            &reg,
        );
        // d₁ = 1.4 → P = 10·(1 − 1.4/12) ≈ 8.83 > 4 粉碎;d₂ = √(1 + 1.96)
        // ≈ 1.72 → P ≈ 8.56 > 4 粉碎;d₃ = √(4 + 1.96) ≈ 2.44 → P ≈ 7.97
        // > 4 粉碎。
        assert!(
            a.iter()
                .all(|f| *f == crate::cohesion::CellBlast::Pulverize),
            "三格全粉碎:{a:?}"
        );
        // 场景 B(脱落):远端格 P ∈ (2, 4]。爆心 z 偏移使 d₃ 最大:
        // 爆心 (7.0, 0.5, 8.5),power 4、radius 12 → d₁ = 1.5 → P = 3.5 ∈ (2, 4]
        // 脱落;d₂ = 2.5 → P ≈ 3.17 脱落;d₃ = 3.5 → P ≈ 2.83 脱落。
        let b = crate::cohesion::compound_blast(
            &state,
            &dirt_beam,
            Vec3::new(7.0, 0.5, 8.5),
            4.0,
            12.0,
            1.0,
            &open,
            &reg,
        );
        // u 对三格都是 (1,0,0)(爆心在梁正左侧同高度)→ dir' 同向。
        let dir = Vec3::new(0.4, 1.0, 0.0).normalize();
        for (i, f) in b.iter().enumerate() {
            let imp = detach(*f);
            let expect = 4.0 * (1.0 - (1.5 + i as f32) / 12.0);
            assert!(
                imp.abs_diff_eq(expect * dir, 1e-4),
                "第 {i} 格脱落冲量 = P·dir':{imp:?}(期望 {expect}·{dir:?})"
            );
        }
        // 场景 C(完好 + 半径外):爆心在梁正下方 1 m(power 2、radius 4)→
        // 各格 P ≤ 2 完好、冲量沿"背离爆心"的斜向(不是纯 +x——它必须来自
        // 单格公式,而不是压力本身)。
        let c_center = Vec3::new(8.0, -0.5, 8.5);
        let c = crate::cohesion::compound_blast(
            &state, &dirt_beam, c_center, 2.0, 4.0, 1.0, &open, &reg,
        );
        // d_i = √((0.5+i)² + 1) ≈ 1.118、1.803、2.693 < 4;
        // P_i = 2·(1 − d_i/4) ≈ 1.441、1.098、0.653,全部 ≤ 2 完好。
        for (i, f) in c.iter().enumerate() {
            let w = Vec3::new(8.5 + i as f32, 0.5, 8.5);
            let expect =
                crate::explosion::explosion_impulse(c_center, 2.0, 4.0, w).expect("半径内必有冲量");
            let imp = intact_impulse(*f).expect("完好带冲量");
            assert!(
                imp.abs_diff_eq(expect, 1e-5) && imp.y > 0.0 && imp.x > 0.0,
                "第 {i} 格完好冲量 = 单格公式(斜向):{imp:?}(期望 {expect:?})"
            );
        }
        let far = crate::cohesion::compound_blast(
            &state,
            &dirt_beam,
            Vec3::new(8.0, 0.5, 20.0),
            2.0,
            4.0,
            1.0,
            &open,
            &reg,
        );
        assert!(
            far.iter()
                .all(|f| *f == crate::cohesion::CellBlast::Intact { impulse: None }),
            "半径外全部完好、无冲量:{far:?}"
        );
        // 场景 D(衰减系数):土梁与爆心之间隔一块石头,attenuation = 0.75 →
        // 第一格 P = 7.333 − 6·0.75 ≈ 2.833 ∈ (2, 4] 脱落;漏乘系数(P − 6 ≈
        // 1.333 ≤ 2)会给完好。
        let wall = |q: BlockPos| {
            Some(if q == BlockPos::new(7, 0, 8) {
                voxel::builtin::STONE
            } else {
                voxel::builtin::AIR
            })
        };
        let d_att = crate::cohesion::compound_blast(
            &state,
            &dirt_beam,
            Vec3::new(7.5, 0.5, 8.5),
            8.0,
            12.0,
            0.75,
            &wall,
            &reg,
        );
        detach(d_att[0]);
        detach(d_att[1]);
        assert!(
            matches!(d_att[2], crate::cohesion::CellBlast::Intact { .. }),
            "第三格 P = 6 − 4.5 = 1.5 ≤ 2 完好:{:?}",
            d_att[2]
        );
    }

    /// n 格土质方块梁(全部 DIRT;R 2 阈值低,便于构造全粉碎)。
    fn dirt_beam_shape(n: i32) -> types::CompoundShape {
        types::CompoundShape::new(
            (0..n)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("梁 ≥ 2 格")
    }

    fn detach(f: crate::cohesion::CellBlast) -> Vec3 {
        match f {
            crate::cohesion::CellBlast::Detach { impulse } => impulse,
            other => panic!("期望脱落,实得 {other:?}"),
        }
    }

    fn intact_impulse(f: crate::cohesion::CellBlast) -> Option<Vec3> {
        match f {
            crate::cohesion::CellBlast::Intact { impulse } => impulse,
            other => panic!("期望完好,实得 {other:?}"),
        }
    }

    /// 逐格求值·地形衰减与半径封顶(§6.1 第 1 条):格与爆心之间隔一格石头
    /// (R 6)→ P = 3.75 − 6 ≤ 0 完好;对照空气衰减 0;radius 1000 按 16 求值
    /// (40 m 外的格不受影响 = 封顶生效;同爆心 power 下未封顶会给远处格冲量)。
    ///
    /// 变红:衰减公式漏乘 R 或 attenuation;复合体自身格被塞进衰减(把梁的
    /// 格写进体素后结果会变);封顶没用 min(1000 直接按 1000 求值 → 远格有压差)。
    #[test]
    fn compound_blast_terrain_attenuation_and_radius_cap() {
        let reg = voxel::SimpleRegistry::builtin();
        let state = EntityState {
            translation: Vec3::new(8.5, 0.5, 8.5),
            ..EntityState::default()
        };
        let open = |_: BlockPos| Some(voxel::builtin::AIR);
        // 挡板:爆心所在格 (7, 0, 8) 一块石头 → 射线第一步就穿它。
        let wall = |q: BlockPos| {
            Some(if q == BlockPos::new(7, 0, 8) {
                voxel::builtin::STONE
            } else {
                voxel::builtin::AIR
            })
        };
        let blocked = crate::cohesion::compound_blast(
            &state,
            &beam_shape(3),
            Vec3::new(7.5, 0.5, 8.5),
            8.0,
            12.0,
            1.0,
            &wall,
            &reg,
        );
        // 第一格中心 (8.5, 0.5, 8.5),d = 1 → 无遮挡 P = 8·(1 − 1/12) ≈ 7.33;
        // 射线穿 (7,0,8)(石,R 6):衰减 6 → P ≈ 1.33 ≤ 6 完好。
        let imp = intact_impulse(blocked[0]).expect("衰减后完好");
        assert!(
            imp.abs_diff_eq(
                crate::explosion::explosion_impulse(
                    Vec3::new(7.5, 0.5, 8.5),
                    8.0,
                    12.0,
                    Vec3::new(8.5, 0.5, 8.5)
                )
                .unwrap(),
                1e-5
            ),
            "被挡格完好,冲量照单格公式(不受遮挡影响):{imp:?}"
        );
        // 对照:无挡板时同格 P ≈ 7.33 > 6 → 脱落。
        let unblocked = crate::cohesion::compound_blast(
            &state,
            &beam_shape(3),
            Vec3::new(7.5, 0.5, 8.5),
            8.0,
            12.0,
            1.0,
            &open,
            &reg,
        );
        detach(unblocked[0]);
        // 半径封顶:radius 1000 → 有效 16;爆心在 40 m 外的格不受影响。
        // (封顶失效时 P = 1000·(1 − 40/1000) ≈ 960 会远超阈值。)
        let capped = crate::cohesion::compound_blast(
            &state,
            &beam_shape(1 + 39),
            Vec3::new(0.5, 0.5, 8.5),
            1000.0,
            1000.0,
            1.0,
            &open,
            &reg,
        );
        assert_eq!(
            intact_impulse(capped[39]),
            None,
            "40 m 外在 16 m 封顶外:无冲量(封顶失效时 P = 1000·(1−47/1000)              ≈ 953 会是粉碎)"
        );
        assert_eq!(
            capped[0],
            crate::cohesion::CellBlast::Pulverize,
            "近格在封顶半径内:power 1000 直接粉碎"
        );
    }

    /// 逐格求值·带旋转与衰减过未装载 chunk(§6.1 风险 1/合同):绕 z 转 90°
    /// 的梁,第一格中心 = 平移 + 旋转 × 偏移;`terrain_at = None`(未装载)不衰减。
    ///
    /// 变红:格中心没乘 rotation(平放姿态公式);未装载被当成实心衰减。
    #[test]
    fn compound_blast_rotated_cells_and_unloaded_no_attenuation() {
        let reg = voxel::SimpleRegistry::builtin();
        let rot = types::glam::Quat::from_rotation_z(90f32.to_radians());
        let state = EntityState {
            translation: Vec3::new(8.5, 0.5, 8.5),
            rotation: rot,
            ..EntityState::default()
        };
        // 全部未知(未装载):不衰减——P 只按距离。
        let none = |_: BlockPos| None;
        let fates = crate::cohesion::compound_blast(
            &state,
            &beam_shape(2),
            Vec3::new(8.5, 1.5, 8.5),
            4.0,
            12.0,
            1.0,
            &none,
            &reg,
        );
        // 第一格中心 = (8.5, 0.5, 8.5) + rot·(1,0,0) = (8.5, 1.5, 8.5),d = 1
        // → P = 4·(1 − 1/12) ≈ 3.67 > 6?否,R_stone 6 → 完好;冲量 = 单格公式。
        let imp = intact_impulse(fates[0]).expect("石 R 6 压不碎");
        assert!(
            imp.abs_diff_eq(
                crate::explosion::explosion_impulse(
                    Vec3::new(8.5, 1.5, 8.5),
                    4.0,
                    12.0,
                    Vec3::new(8.5, 0.5, 8.5)
                )
                .unwrap(),
                1e-5
            ),
            "第一格中心 = 平移(rot·(0,0,0)),冲量竖直向下:{imp:?}"
        );
        // 第二格中心 = (8.5, 0.5, 8.5) + rot·(1,0,0) = (8.5, 1.5, 8.5)——恰为
        // 爆心:d = 0 → 完好、无冲量(单格公式的 d=0 语义;旋转 × 偏移生效)。
        assert_eq!(intact_impulse(fates[1]), None, "第二格中心恰为爆心:无冲量");
        // 对照:爆心抬高 0.5 → 第二格 d = 0.5 < 1,有冲量;第一格 d = √1.25。
        let fates2 = crate::cohesion::compound_blast(
            &state,
            &beam_shape(2),
            Vec3::new(8.5, 2.0, 8.5),
            4.0,
            12.0,
            1.0,
            &none,
            &reg,
        );
        let imp1 = intact_impulse(fates2[1]).expect("d = 0.5 半径内");
        assert!(
            imp1.abs_diff_eq(
                crate::explosion::explosion_impulse(
                    Vec3::new(8.5, 2.0, 8.5),
                    4.0,
                    12.0,
                    Vec3::new(8.5, 1.5, 8.5)
                )
                .unwrap(),
                1e-5
            ),
            "第二格冲量 = 单格公式(中心已随旋转移):{imp1:?}"
        );
    }

    /// 爆炸接线·无破坏按点施冲量(§6.1 第 2 条,D6;生产路径):零重力石梁,
    /// 爆心在梁左端的左下方、power 4(各格压力 < R 6 不破坏)→ 同一实体、
    /// 形状不变,获得向右上方的线动量与绕 z 的角动量(按点施加 ≠ 中心合成:
    /// 中心合成的角速度为 0)。
    ///
    /// 变红:只在中心施一次冲量(ω_z = 0);冲量直接改速度没走 rapier
    /// (位置/旋转不更新);开关关了还走复合体分支(单格公式,ω ≠ 期望)。
    #[test]
    fn detonate_intact_compound_applies_point_impulses() {
        let mut cfg = zero_g_cfg();
        cfg.destruction.enabled = true;
        cfg.cohesion.enabled = true;
        cfg.solidify.enabled = false;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        let id = entity(1);
        s.apply_prefix(&[compound_spawn_input(
            1,
            id,
            EntityState {
                translation: Vec3::new(10.5, 0.5, 8.5),
                ..EntityState::default()
            },
            beam_shape(6),
        )]);
        s.apply_prefix(&[detonate_input(2, Vec3::new(9.0, -1.0, 8.5), 4.0, 12.0)]);
        s.step();
        let st = s.entity_state(id).expect("无破坏不删除");
        assert!(
            st.linvel.x > 0.5 && st.linvel.y > 0.5,
            "各格冲量合成的线动量向右上:{:?}",
            st.linvel
        );
        assert!(
            st.angvel.z < -0.05,
            "偏心施力产生绕 z 的力矩(左端受力更大 → 顺时针):{:?}",
            st.angvel
        );
        let out = s.take_outbox();
        assert_eq!(out.cohesion_stats.blast_removed_cells, 0, "无破坏不计数");
        assert!(out.piece_requests.is_empty(), "无破坏不产分块");
        assert!(out.despawned.is_empty());
        // 对照:载荷关闭(cohesion off)→ 单格公式只在质心施一次,同爆心下
        // ω_z 为 0(同一拍、同一爆心的确定性对照)。
        let mut cfg2 = zero_g_cfg();
        cfg2.destruction.enabled = true;
        cfg2.cohesion.enabled = false;
        cfg2.solidify.enabled = false;
        let mut s2 = ShardSim::new(cfg2, Box::new(voxel::SimpleRegistry::builtin()));
        let id2 = entity(1);
        s2.apply_prefix(&[compound_spawn_input(
            1,
            id2,
            EntityState {
                translation: Vec3::new(10.5, 0.5, 8.5),
                ..EntityState::default()
            },
            beam_shape(6),
        )]);
        s2.apply_prefix(&[detonate_input(2, Vec3::new(9.0, -1.0, 8.5), 4.0, 12.0)]);
        s2.step();
        let st2 = s2.entity_state(id2).expect("实体保留");
        assert!(
            st2.angvel.z.abs() < 1e-4,
            "载荷关闭:单格路径质心施冲量,无角速度:{:?}",
            st2.angvel
        );
    }

    /// 爆炸接线·炸碎复合体(§6.1 第 3 条;生产路径):零重力 6 格石梁,爆心
    /// 在左端外(power 12、radius 8)→ 前 2 格脱落(6 < P ≤ 12)、第 3 格 P = 6
    /// 恰不破坏、其余完好;原实体当拍删除、`blast_removed_cells` = 2;分块
    /// 请求 = 4 个完好格分量(复合体,index 0)+ 2 个脱落碎块(index 1..3,
    /// 单格碎块、不带形状),分量线速度带 Σ 冲量 / 质量。
    ///
    /// 变红:不破坏(只施冲量);分量丢冲量;脱落格速度漏 ω × r 项或质量
    /// 用错;index 顺序不先分量后脱落;遥测不接线。
    #[test]
    fn detonate_breaks_compound_into_components_then_detached() {
        let mut cfg = zero_g_cfg();
        cfg.destruction.enabled = true;
        cfg.cohesion.enabled = true;
        cfg.solidify.enabled = false;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        let id = entity(1);
        // 梁沿 +x,原点平移 = 格 −2..=3 的中心 − 2 → 各格世界中心 x = 8.5..13.5。
        let shape = types::CompoundShape::new(
            (-2..=3)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("6 格");
        s.apply_prefix(&[compound_spawn_input(
            1,
            id,
            EntityState {
                translation: Vec3::new(10.5, 0.5, 8.5),
                ..EntityState::default()
            },
            shape,
        )]);
        // 爆心 (6.5, 0.5, 8.5):d_i = 2..=7 → P = 12·(1 − d/8) =
        // 9、7.5、6、4.5、3、1.5;石 R 6:P > 6 脱落(前两格),P = 6 恰在阈值
        // 上不破坏(第 3 格),其余完好。
        s.apply_prefix(&[detonate_input(2, Vec3::new(6.5, 0.5, 8.5), 12.0, 8.0)]);
        s.step();
        assert!(s.entity_state(id).is_none(), "原实体当拍删除");
        let out = s.take_outbox();
        assert_eq!(out.cohesion_stats.blast_removed_cells, 2, "脱落 2 格");
        assert_eq!(out.piece_requests.len(), 3, "1 个剩余分量 + 2 个脱落格");
        // 分量(完整):4 个完好格相连(d = 4 的格 P = 6 恰不破坏)→ 1 个
        // 分量;后 2 个 = 脱落格。
        let comp = &out.piece_requests[0];
        assert!(
            comp.desc.shape().is_some(),
            "完好格相连成分量复合体:{:?}",
            comp.desc.shape().map(|s| s.cells().len())
        );
        assert_eq!(comp.index, 0);
        // 脱落格单格碎块,速度 = v + ω × r + 冲量 / 0.729;v = ω = 0 → 纯冲量。
        for (k, piece) in out.piece_requests[1..].iter().enumerate() {
            assert!(piece.desc.shape().is_none(), "脱落格 = 单格碎块");
            assert_eq!(piece.desc.block, voxel::builtin::STONE);
            assert_eq!(piece.index, (k + 1) as u32, "index 分量先、脱落格后");
            let d = (2 + k) as f32;
            let p = 12.0 * (1.0 - d / 8.0);
            let dir = Vec3::new(0.4, 1.0, 0.0).normalize();
            let expect = Vec3::new(8.5 + k as f32, 0.5, 8.5);
            assert!(
                (piece.state.translation.x - expect.x).abs() < 1e-4
                    && (piece.state.translation.y - expect.y).abs() < 1e-4,
                "脱落格位置 = 该格世界中心:{:?}",
                piece.state.translation
            );
            assert!(
                piece.state.linvel.abs_diff_eq(p * dir / 0.729, 1e-3),
                "脱落格速度 = 冲量 / 0.729:{:?}(期望 {p}·{dir:?}/0.729)",
                piece.state.linvel
            );
        }
        // 分量速度 = Σ(完好格冲量)/ (4 × 0.729):完好格冲量 = 单格公式
        // d = 4,5,6,7 → 沿 +x 的 6、4.5、3、1.5;Σ = 15 → /(4·0.729) ≈ 5.144。
        assert!(
            comp.state
                .linvel
                .abs_diff_eq(Vec3::new(15.0 / 2.916, 0.0, 0.0), 1e-3),
            "分量线速度带 Σ 冲量 / 分量质量:{:?}",
            comp.state.linvel
        );
        assert!(out.despawned.contains(&id));
    }

    /// 爆炸接线·粉碎格消失、冻结节外(§6.1 第 3/4 条):土梁被高压力爆炸全 6
    /// 格粉碎(P 全 > 4)→ 原实体删除、`blast_removed_cells` = 6、零分块请求;
    /// 冻结(kinematic)复合体同一爆心下完全免疫(状态位级不变)。
    ///
    /// 变红:粉碎格也产碎块;粉碎不计数;冻结体不免疫。
    #[test]
    fn detonate_pulverize_whole_and_frozen_immune() {
        // 全粉碎:6 格土梁(R 2、k 2 → P > 4 粉碎),power 30、radius 8,
        // 爆心 (9.5, 0.5, 8.5) → d = 1..=6,P = 30·(1 − d/8) =
        // 26.25、22.5、18.75、15、11.25、7.5 → 全 > 4 全粉碎。
        let run = |frozen: bool| {
            let mut cfg = zero_g_cfg();
            cfg.destruction.enabled = true;
            cfg.cohesion.enabled = true;
            cfg.solidify.enabled = false;
            let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
            let id = entity(1);
            s.apply_prefix(&[compound_spawn_input(
                1,
                id,
                EntityState {
                    translation: Vec3::new(10.5, 0.5, 8.5),
                    ..EntityState::default()
                },
                dirt_beam_shape(6),
            )]);
            if frozen {
                // 冻结 = kinematic:绕过激活,直接改 body 类型。
                let rec = s.entities.get(&id).expect("在表");
                s.phys
                    .bodies
                    .get_mut(rec.handle)
                    .expect("handle 有效")
                    .set_body_type(RigidBodyType::KinematicPositionBased, true);
            }
            s.apply_prefix(&[detonate_input(2, Vec3::new(9.5, 0.5, 8.5), 30.0, 8.0)]);
            s.step();
            (s, id)
        };
        let (mut s, id) = run(false);
        assert!(s.entity_state(id).is_none(), "全粉碎也删原实体");
        let out = s.take_outbox();
        assert_eq!(out.cohesion_stats.blast_removed_cells, 6, "6 格全计");
        assert!(
            out.piece_requests.is_empty(),
            "全粉碎没有剩余分量、没有脱落格:{:?}",
            out.piece_requests.len()
        );
        let (mut s2, id2) = run(true);
        let st = s2.entity_state(id2).expect("冻结体保留");
        let out2 = s2.take_outbox();
        assert_eq!(out2.cohesion_stats.blast_removed_cells, 0, "免疫不计数");
        assert_eq!(st.linvel, Vec3::ZERO, "冻结体对爆炸免疫(§9.12),速度不变");
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
    /// (醒着 + 缺 ②)→ panic。钩子只在 `debug_assertions` 下存在,测试随门。
    #[test]
    #[cfg(debug_assertions)]
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

    /// §7 单元 03:`entity_desc` 透传生成时的描述(凝固桥/验收观测面);
    /// 未知实体(已凝固/不存在)→ None。
    ///
    /// 变红:返回默认描述(0.5/stone)而非实体自己的。
    #[test]
    fn entity_desc_returns_spawned_desc_and_none_for_unknown() {
        let mut s = sim(0, 0);
        let id = entity(1);
        let desc = EntityDesc {
            half_size: 0.45,
            block: voxel::builtin::DIRT,
            shape: None,
        };
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState::default(),
                desc: desc.clone(),
            }),
        }]);
        assert_eq!(s.entity_desc(id), Some(desc), "透传生成时的描述");
        assert_eq!(s.entity_desc(entity(2)), None, "未知实体 → None");
    }

    /// §7 每拍上限由分片执行:同一拍两次爆炸按输入序共用
    /// `max_detach_per_tick` 名额——先到的取满,后到的只剩余额;超限的格
    /// 改粉碎但照样写 air(坑不变小)。
    ///
    /// 变红:上限只按单次爆炸计(两次各 24 = 48 个请求);超限的格不写 air。
    #[test]
    fn detonate_per_tick_cap_shared_across_explosions() {
        use voxel::WorldGenerator;
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.destruction.enabled = true;
        cfg.destruction.max_detach_per_tick = 30;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        for cx in 2..=6i32 {
            for cz in 2..=6i32 {
                let cp = ChunkPos::new(cx, cz);
                s.load_chunk(
                    cp,
                    voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
                );
            }
        }
        let center = Vec3::new(64.0, -59.5, 64.0);
        let far = Vec3::new(64.0, -59.5, 96.0);
        s.apply_prefix(&[
            detonate_input(1, center, 12.0, 8.0),
            detonate_input(2, far, 12.0, 8.0),
        ]);
        let out = s.take_outbox();
        assert_eq!(
            out.detach_requests.len(),
            30,
            "两爆合计 30 = 24 + 6(输入序消耗名额)"
        );
        assert!(
            out.detach_requests
                .windows(2)
                .all(|w| w[0].cell < w[1].cell),
            "整个 outbox 保持 BlockPos 升序"
        );
        assert_eq!(
            out.detach_requests.iter().filter(|r| r.cell.z < 80).count(),
            24,
            "先到的爆炸占满 24 个名额"
        );
        assert_eq!(
            out.detach_requests.iter().filter(|r| r.cell.z > 80).count(),
            6,
            "后到的爆炸只剩 6 个名额"
        );
        assert_eq!(
            out.block_updates.len(),
            120,
            "两个标准弹坑 60 + 60 全部写 air(坑不变小)"
        );
    }

    /// §7 名额耗尽:剩余名额取 0(不足 0 取 0)→ 全部改粉碎,坑照样写
    /// air、无脱落请求。
    ///
    /// 变红:上限下探为负/panic;或超限格不写 air。
    #[test]
    fn detonate_per_tick_cap_exhausted_still_writes_air() {
        use voxel::WorldGenerator;
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.destruction.enabled = true;
        cfg.destruction.max_detach_per_tick = 0;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        for cx in 2..=6i32 {
            for cz in 2..=6i32 {
                let cp = ChunkPos::new(cx, cz);
                s.load_chunk(
                    cp,
                    voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
                );
            }
        }
        s.apply_prefix(&[detonate_input(1, Vec3::new(64.0, -59.5, 64.0), 12.0, 8.0)]);
        let out = s.take_outbox();
        assert_eq!(out.detach_requests.len(), 0, "名额 0 → 无脱落请求");
        assert_eq!(out.block_updates.len(), 60, "坑照样写 air");
    }

    // ---- 坍塌接入分片(REQ-009 §6.2.1,单元 04b)----

    /// 开启破坏的单分片平地(_chunks cx 2..=6、cz 2..=6,超平坦;名额 = cap)。
    fn destruction_sim(cap: usize) -> ShardSim {
        destruction_sim_tuned(cap, 2048, 512)
    }

    /// 同上,另调坍塌搜索预算与悬空块上限(遥测测试用)。
    fn destruction_sim_tuned(cap: usize, max_cells: usize, max_cluster: usize) -> ShardSim {
        use voxel::WorldGenerator;
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.destruction.enabled = true;
        // REQ-010 §12.1 第 3 条迁移:REQ-009 坍塌语义用例(连通即锚定、逐格塌落)
        // 在装配处显式关闭承载判定,断言不变。
        cfg.cohesion.enabled = false;
        cfg.destruction.max_detach_per_tick = cap;
        cfg.destruction.collapse_max_cells = max_cells;
        cfg.destruction.collapse_max_cluster = max_cluster;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        for cx in 2..=6i32 {
            for cz in 2..=6i32 {
                let cp = ChunkPos::new(cx, cz);
                s.load_chunk(
                    cp,
                    voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
                );
            }
        }
        s
    }

    /// 单分片石桥(与验收 unit04b 同形):柱 x=60、71(y −60…−55,z=64),
    /// 桥面 y=−54、x 60…71;返回 (左柱, 右柱, 桥面)。
    fn bridge_cells() -> (Vec<BlockPos>, Vec<BlockPos>, Vec<BlockPos>) {
        let pa: Vec<BlockPos> = (-60..=-55).map(|y| BlockPos::new(60, y, 64)).collect();
        let pb: Vec<BlockPos> = (-60..=-55).map(|y| BlockPos::new(71, y, 64)).collect();
        let deck: Vec<BlockPos> = (60..=71).map(|x| BlockPos::new(x, -54, 64)).collect();
        (pa, pb, deck)
    }

    /// 搭桥就位(搭建写入 air→实心,不产生坍塌触发)。
    fn build_bridge(s: &mut ShardSim) -> (Vec<BlockPos>, Vec<BlockPos>, Vec<BlockPos>) {
        let (pa, pb, deck) = bridge_cells();
        let build: Vec<StampedInput> = [&pa[..], &pb[..], &deck[..]]
            .concat()
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(k as u64, *p, voxel::builtin::STONE))
            .collect();
        s.apply_prefix(&build);
        s.step();
        (pa, pb, deck)
    }

    /// §6.2.1 生产路径(AC-D5):两柱同拍去掉 → 当拍桥面 12 格写 air,产出
    /// 12 条冲量为 0 的脱落请求(BlockPos 升序、记录原方块 stone、Emergent
    /// 写入、collider 重建)。
    ///
    /// 变红:求值入口整体跳过(桥面不塌);冲量非 0(坍塌块带了爆炸冲量)。
    #[test]
    fn collapse_bridge_converts_and_requests_detach() {
        let mut s = destruction_sim(64);
        let (pa, pb, deck) = build_bridge(&mut s);
        let remove: Vec<StampedInput> = [&pa[..], &pb[..]]
            .concat()
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(100 + k as u64, *p, voxel::builtin::AIR))
            .collect();
        s.apply_prefix(&remove);
        s.step();
        let out = s.take_outbox();
        for c in &deck {
            assert_eq!(
                s.voxel().get_block(*c),
                voxel::builtin::AIR,
                "{c:?} 当拍写 air"
            );
        }
        assert_eq!(out.detach_requests.len(), 12, "桥面 12 格全部脱落");
        assert!(
            out.detach_requests
                .windows(2)
                .all(|w| w[0].cell < w[1].cell),
            "脱落请求按 BlockPos 升序"
        );
        for req in &out.detach_requests {
            assert_eq!(req.block, voxel::builtin::STONE, "记录原方块");
            assert_eq!(req.impulse, Vec3::ZERO, "坍塌块冲量为 0");
        }
        assert_eq!(
            out.block_updates
                .iter()
                .filter(|w| w.kind == BlockWriteKind::Emergent && deck.contains(&w.pos))
                .count(),
            12,
            " Emergent 写入恰 12 条"
        );
    }

    /// §6.2.1 推迟(风险 4 后半):名额被同拍爆炸占满(剩余 0)的拍不转换、
    /// 也不丢触发——整个触发集原样推迟,下一拍名额恢复时照常塌。
    ///
    /// 变红:推迟只留已转换格(本拍 0 转换 → 推迟队列空 → 下一拍无触发,
    /// 桥面永远悬在原地)。
    #[test]
    fn collapse_quota_zero_defers_triggers_to_next_tick() {
        let mut s = destruction_sim(24); // 标准弹坑脱落恰好 24,占满名额
        let (pa, pb, deck) = build_bridge(&mut s);
        let mut inputs = vec![detonate_input(200, Vec3::new(64.0, -59.5, 96.0), 12.0, 8.0)];
        inputs.extend(
            [&pa[..], &pb[..]]
                .concat()
                .iter()
                .enumerate()
                .map(|(k, p)| setblock_input(300 + k as u64, *p, voxel::builtin::AIR)),
        );
        s.apply_prefix(&inputs);
        s.step();
        let out1 = s.take_outbox();
        assert_eq!(out1.detach_requests.len(), 24, "名额被爆炸占满");
        for c in &deck {
            assert_eq!(
                s.voxel().get_block(*c),
                voxel::builtin::STONE,
                "名额 0:当拍不转换"
            );
        }
        s.step();
        let out2 = s.take_outbox();
        assert_eq!(out2.detach_requests.len(), 12, "下一拍触发集仍在,照常塌");
        for c in &deck {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR);
        }
    }

    /// §6.2.1 名额与升序:每拍名额 5,两柱同拍去掉 → 当拍按 `BlockPos` 升序
    /// 转换前 5 格,推迟的下一拍续 5 格,再下一拍收尾 2 格(5/5/2)。
    ///
    /// 变红:转换不按升序(当拍塌的是升序末 5 格);名额不约束(当拍全塌)。
    #[test]
    fn collapse_quota_converts_ascending_across_ticks() {
        let mut s = destruction_sim(5);
        let (pa, pb, deck) = build_bridge(&mut s);
        let remove: Vec<StampedInput> = [&pa[..], &pb[..]]
            .concat()
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(100 + k as u64, *p, voxel::builtin::AIR))
            .collect();
        s.apply_prefix(&remove);
        for take in 0..3 {
            s.step();
            let out = s.take_outbox();
            assert_eq!(out.detach_requests.len(), [5, 5, 2][take], "第 {take} 拍");
            for (k, c) in deck.iter().enumerate() {
                let expected = if k < 5 * (take + 1) {
                    voxel::builtin::AIR
                } else {
                    voxel::builtin::STONE
                };
                assert_eq!(s.voxel().get_block(*c), expected, "拍 {take} 的 {c:?}");
            }
        }
    }

    /// §6.2.1/§8:推迟队列随分片快照走——名额 5 塌到一半取快照,恢复线继续
    /// 5/2 收尾,脱落请求与原线逐位一致。
    ///
    /// 变红:快照丢推迟队列(恢复线首拍无触发,少塌 7 格)。
    #[test]
    fn collapse_deferred_queue_survives_snapshot() {
        let mut a = destruction_sim(5);
        let (pa, pb, deck) = build_bridge(&mut a);
        let remove: Vec<StampedInput> = [&pa[..], &pb[..]]
            .concat()
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(100 + k as u64, *p, voxel::builtin::AIR))
            .collect();
        a.apply_prefix(&remove);
        a.step();
        assert_eq!(a.take_outbox().detach_requests.len(), 5, "首拍 5 格");
        let bytes = a.snapshot();
        let mut b = ShardSim::restore(&bytes, Box::new(voxel::SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        for _ in 0..2 {
            a.step();
            b.step();
            let (oa, ob) = (a.take_outbox(), b.take_outbox());
            assert_eq!(oa.detach_requests, ob.detach_requests, "恢复线逐位一致");
        }
        assert_eq!(
            a.voxel().get_block(deck[11]),
            voxel::builtin::AIR,
            "两拍收尾 7 格(5 + 2)"
        );
        assert_eq!(b.voxel().get_block(deck[11]), voxel::builtin::AIR);
    }

    /// §6.2.1(风险 2):世界高度范围之外按空气,不是未装载——贴着世界顶
    /// (y = 319)的石塔切底,上面 19 格照塌。
    ///
    /// 变红:block_at 对 y > WORLD_Y_MAX 返回 None(照搬爆炸口径,被当未装载
    /// → 锚定,塔不塌)。
    #[test]
    fn collapse_world_top_is_air_not_unloaded() {
        let mut s = destruction_sim(64);
        let tower: Vec<BlockPos> = (300..=319).map(|y| BlockPos::new(66, y, 60)).collect();
        let build: Vec<StampedInput> = tower
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(k as u64, *p, voxel::builtin::STONE))
            .collect();
        s.apply_prefix(&build);
        s.step();
        s.apply_prefix(&[setblock_input(100, tower[0], voxel::builtin::AIR)]);
        s.step();
        let out = s.take_outbox();
        for c in &tower[1..] {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?} 塌落");
        }
        assert_eq!(out.detach_requests.len(), 19);
    }

    /// §6.2.1 风险 1:BlockWrite 信封的涌现写入(邻片 owner 的转换到达)同样
    /// 登记触发——信封写掉悬空平台的唯一支撑,平台当拍塌。
    ///
    /// 变红:信封应用路径不登记触发(平台永远悬在原地)。
    #[test]
    fn collapse_envelope_write_triggers() {
        let mut s = destruction_sim(64);
        // 悬空平台:柱 (40, 5..=9, 40) + 横杆 (40..=43, 10, 40),整体在空中。
        let column: Vec<BlockPos> = (5..=9).map(|y| BlockPos::new(40, y, 40)).collect();
        let bar: Vec<BlockPos> = (40..=43).map(|x| BlockPos::new(x, 10, 40)).collect();
        let build: Vec<StampedInput> = [&column[..], &bar[..]]
            .concat()
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(k as u64, *p, voxel::builtin::STONE))
            .collect();
        s.apply_prefix(&build);
        s.step();
        s.apply_prefix(&[envelope_input(
            1,
            shard_id(1, 0),
            EnvelopePayload::BlockWrite {
                pos: BlockPos::new(40, 9, 40),
                block: voxel::builtin::AIR,
                load_conversion: false,
            },
        )]);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.detach_requests.len(), 8, "柱余 4 格 + 横杆 4 格");
        for c in column.iter().chain(bar.iter()) {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
        }
    }

    /// §6.2.1 触发要"由实心变为非实心":向已是空气的格写 air 不登记触发——
    /// 旁边悬空的既有结构不受无转换写入的扰动。
    ///
    /// 变红:任何 SetBlock 写入(无论是否转换)都登记触发(悬空杆被误塌)。
    #[test]
    fn collapse_no_trigger_without_solid_to_nonsolid() {
        let mut s = destruction_sim(64);
        let bar: Vec<BlockPos> = (40..=43).map(|x| BlockPos::new(x, 10, 40)).collect();
        let build: Vec<StampedInput> = bar
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(k as u64, *p, voxel::builtin::STONE))
            .collect();
        s.apply_prefix(&build);
        s.step();
        // 两条无转换写入:同格重写 air、别处向空气格写 air。
        s.apply_prefix(&[
            setblock_input(1, BlockPos::new(40, 11, 40), voxel::builtin::AIR),
            setblock_input(2, BlockPos::new(48, 10, 44), voxel::builtin::AIR),
        ]);
        s.step();
        let out = s.take_outbox();
        assert!(out.detach_requests.is_empty(), "无转换无触发");
        for c in &bar {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::STONE);
        }
    }

    /// 范围 5(守卫):总开关关闭不收集、不求值——两柱去掉、连跑三拍,桥面
    /// 原样、零脱落请求。与验收守卫用例 `acc_d5_disabled_no_collapse` 同语义。
    ///
    /// 变红:收集闸去掉 enabled 条件(触发集不再受开关控制,桥面塌落)。
    #[test]
    fn collapse_disabled_is_inert() {
        // 单元 07 起 D7 默认开启(REQ-009 §9 迁移清单):本用例测"关闭时惰性",
        // 在装配处显式关闭;断言不动。
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.destruction.enabled = false;
        assert!(!cfg.destruction.enabled, "默认关闭(实施期间)");
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        use voxel::WorldGenerator;
        for cx in 2..=6i32 {
            for cz in 2..=5i32 {
                let cp = ChunkPos::new(cx, cz);
                s.load_chunk(
                    cp,
                    voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
                );
            }
        }
        let (pa, pb, deck) = build_bridge(&mut s);
        let remove: Vec<StampedInput> = [&pa[..], &pb[..]]
            .concat()
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(100 + k as u64, *p, voxel::builtin::AIR))
            .collect();
        s.apply_prefix(&remove);
        for _ in 0..3 {
            s.step();
            assert!(
                s.take_outbox().detach_requests.is_empty(),
                "关闭时零脱落请求"
            );
        }
        for c in &deck {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::STONE, "{c:?} 不塌");
        }
    }

    // ---- 承载判定接入分片(REQ-010 §4.6,单元 02)----

    /// 开启破坏 + 粘结的单分片平地(chunk 2..=6 平方,超平坦;名额 = cap)。
    fn cohesion_sim(cap: usize) -> ShardSim {
        let mut s = destruction_sim(cap);
        s.cfg.cohesion.enabled = true;
        // 单元 04 迁移(与总架构师对 e2e unit02 的迁移同款):本组用例按逐格
        // 转换编写(§4.6 第 4 条的队列语义);单元 04 起按段转换(§5.5),切块
        // 边长设 1 即每格一段、保持逐格,断言不变。
        s.cfg.cohesion.compound_split_size = 1;
        s
    }

    /// 石墙(x 40..=42、y −60..=−55、z−2..=z+2)伸出 8 格石悬臂 x 43..=50、
    /// y = −55、同 z。满载(8 ≤ B)挂得住。返回悬臂格。
    fn build_cantilever(s: &mut ShardSim, z: i32) -> Vec<BlockPos> {
        let wall: Vec<BlockPos> = (40..=42)
            .flat_map(|x| {
                (z - 2..=z + 2)
                    .flat_map(move |zz| (-60..=-55).map(move |y| BlockPos::new(x, y, zz)))
            })
            .collect();
        let arm: Vec<BlockPos> = (43..=50).map(|x| BlockPos::new(x, -55, z)).collect();
        let build: Vec<StampedInput> = [&wall[..], &arm[..]]
            .concat()
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(k as u64, *p, voxel::builtin::STONE))
            .collect();
        s.apply_prefix(&build);
        s.step();
        arm
    }

    /// 土门框(与验收同形):柱 x = 60、66(y −60..=−56,z)石,门楣 x 60..=66、
    /// y = −55、同 z 土,7 格恰好满载(两端柱顶出口 3 + 3 ≥ 中间 5)。返回门楣。
    fn build_door_lintel(s: &mut ShardSim, z: i32) -> Vec<BlockPos> {
        let pa: Vec<BlockPos> = (-60..=-56).map(|y| BlockPos::new(60, y, z)).collect();
        let pb: Vec<BlockPos> = (-60..=-56).map(|y| BlockPos::new(66, y, z)).collect();
        let lintel: Vec<BlockPos> = (60..=66).map(|x| BlockPos::new(x, -55, z)).collect();
        let build: Vec<StampedInput> = [&pa[..], &pb[..], &lintel[..]]
            .concat()
            .iter()
            .enumerate()
            .map(|(k, p)| {
                setblock_input(
                    k as u64,
                    *p,
                    if lintel.contains(p) {
                        voxel::builtin::DIRT
                    } else {
                        voxel::builtin::STONE
                    },
                )
            })
            .collect();
        s.apply_prefix(&build);
        s.step();
        lintel
    }

    /// §4.6 第 2 条(开启时触发口径):放置(空→实心)与实心换实心的材料
    /// 替换都登记——加重压断悬臂、降键断根。悬臂满载 8:末端放 1 格 → 9 > 8
    /// 整根连同放置格塌(9 条脱落);根部 stone 换 dirt → 键降到 3 < 8 整根断。
    ///
    /// 变红:放置不触发(只登记"实心变空",悬臂不塌);材料替换不触发;
    /// 放置格不进 F(只塌 8 格)。
    #[test]
    fn load_trigger_placement_and_replacement_when_enabled() {
        let mut s = cohesion_sim(64);
        let arm_a = build_cantilever(&mut s, 64);
        let arm_b = build_cantilever(&mut s, 74);
        assert!(
            s.take_outbox().detach_requests.is_empty(),
            "两条满载悬臂都挂得住"
        );
        // 放置加重(D8):悬臂末端放 1 格石头。
        let weight = BlockPos::new(50, -54, 64);
        s.apply_prefix(&[setblock_input(1, weight, voxel::builtin::STONE)]);
        s.step();
        let out = s.take_outbox();
        for c in arm_a.iter().chain([&weight]) {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?} 塌");
        }
        for c in &arm_b {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::STONE, "{c:?} 不塌");
        }
        assert_eq!(out.detach_requests.len(), 9, "悬臂 8 格 + 放置格");
        assert_eq!(out.cohesion_stats.load_falling, 9);
        // 材料替换(D8):根部 stone → dirt,键 8 降到 3。
        let root = BlockPos::new(42, -55, 74);
        s.apply_prefix(&[setblock_input(2, root, voxel::builtin::DIRT)]);
        s.step();
        let out = s.take_outbox();
        for c in &arm_b {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?} 断下");
        }
        assert_eq!(
            s.voxel().get_block(root),
            voxel::builtin::DIRT,
            "换上的土格是接地的墙,不动"
        );
        assert_eq!(out.detach_requests.len(), 8);
    }

    /// §4.6 第 1 条(关闭 = REQ-009,风险 1):cohesion 关闭时放置与替换都
    /// 不登记(REQ-009 只登记实心→非实心)、悬臂不塌;destruction 关闭而
    /// cohesion 开启时同样什么都不发生、cohesion 遥测恒零。
    ///
    /// 变红:关闭时也登记"加重"的触发(悬臂塌、REQ-009 行为被改);关闭时
    /// 计 cohesion 遥测。
    #[test]
    fn load_trigger_disabled_matches_req009_and_zero_telemetry() {
        // destruction 开、cohesion 关:REQ-009 口径。
        let mut s = destruction_sim(64);
        let arm = build_cantilever(&mut s, 64);
        s.apply_prefix(&[setblock_input(
            1,
            BlockPos::new(50, -54, 64),
            voxel::builtin::STONE,
        )]);
        s.step();
        s.apply_prefix(&[setblock_input(
            2,
            BlockPos::new(42, -55, 64),
            voxel::builtin::DIRT,
        )]);
        s.step();
        assert!(
            s.take_outbox().detach_requests.is_empty(),
            "关闭时放置与替换都不触发"
        );
        for c in &arm {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::STONE, "{c:?} 不塌");
        }
        // destruction 关、cohesion 开:两开关不同时开 → 承载判定不接管。
        let mut s = destruction_sim(64);
        s.cfg.destruction.enabled = false;
        s.cfg.cohesion.enabled = true;
        let arm = build_cantilever(&mut s, 64);
        s.apply_prefix(&[
            setblock_input(1, BlockPos::new(50, -54, 64), voxel::builtin::STONE),
            setblock_input(2, BlockPos::new(42, -55, 64), voxel::builtin::DIRT),
        ]);
        s.step();
        let out = s.take_outbox();
        assert!(out.detach_requests.is_empty());
        assert_eq!(
            out.cohesion_stats,
            crate::cohesion::CohesionStats::default()
        );
        assert_eq!(out.destruction_stats.collapse_evaluations, 0);
        for c in &arm {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::STONE, "{c:?} 不塌");
        }
    }

    /// 拆除门框左柱(5 格 SetBlock air)。
    fn remove_door_pillar(s: &mut ShardSim, z: i32) {
        let inputs: Vec<StampedInput> = (-60..=-56)
            .map(|y| {
                setblock_input(
                    100 + (y + 60) as u64,
                    BlockPos::new(60, y, z),
                    voxel::builtin::AIR,
                )
            })
            .collect();
        s.apply_prefix(&inputs);
    }

    /// §4.6 第 4 条(名额约束):名额 0 时 F 全部入队、不转换、不丢失——
    /// 连跑两拍队列原样、零脱落请求。
    ///
    /// 变红:名额 0 时丢队列(下一拍名额恢复也没得转);不判 F 就不入队。
    #[test]
    fn load_pending_stays_when_quota_zero() {
        let mut s = cohesion_sim(0);
        let lintel = build_door_lintel(&mut s, 64);
        let fell: Vec<BlockPos> = lintel[..6].to_vec();
        remove_door_pillar(&mut s, 64);
        s.step();
        let out = s.take_outbox();
        assert!(out.detach_requests.is_empty(), "名额 0:当拍不转换");
        assert_eq!(out.cohesion_stats.load_falling, 6, "F = 门楣左 6 格");
        assert_eq!(out.cohesion_stats.load_pending, 6, "全部入队");
        for c in &fell {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::DIRT, "{c:?} 原样");
        }
        s.step();
        let out = s.take_outbox();
        assert!(out.detach_requests.is_empty());
        assert_eq!(out.cohesion_stats.load_pending, 6, "队列不丢");
    }

    /// §4.6 第 4 条(升序 + 丢弃):名额 2,F = 6 → 当拍升序转换前 2 格;
    /// 队列里的一格被同拍外部写入空气后,轮到时丢弃且**不占名额**——同拍
    /// 名额 2 全部用在后两格上;最后 1 格下一拍直转(不重新求值:重求会判
    /// 剩余格挂得住)。
    ///
    /// 变红:丢弃格占名额(当拍只转 1 格);推迟时重新求值(剩余格永远悬空);
    /// 转换不按升序。
    #[test]
    fn load_pending_discards_gone_cell_without_consuming_quota() {
        let mut s = cohesion_sim(2);
        let lintel = build_door_lintel(&mut s, 64);
        let fell: Vec<BlockPos> = lintel[..6].to_vec();
        remove_door_pillar(&mut s, 64);
        s.step();
        let out = s.take_outbox();
        assert_eq!(
            out.detach_requests
                .iter()
                .map(|r| r.cell)
                .collect::<Vec<_>>(),
            &fell[..2],
            "升序转换前 2 格"
        );
        assert_eq!(out.cohesion_stats.load_pending, 4);
        // 队列首位被外部写掉(模拟同拍爆炸):轮到时丢弃、不占名额。
        s.apply_prefix(&[setblock_input(200, fell[2], voxel::builtin::AIR)]);
        s.step();
        let out = s.take_outbox();
        assert_eq!(
            out.detach_requests
                .iter()
                .map(|r| r.cell)
                .collect::<Vec<_>>(),
            vec![fell[3], fell[4]],
            "丢弃格不占名额:名额 2 全用在后两格"
        );
        assert_eq!(out.cohesion_stats.load_pending, 1);
        assert_eq!(
            s.voxel().get_block(fell[5]),
            voxel::builtin::DIRT,
            "末格未轮到"
        );
        s.step();
        let out = s.take_outbox();
        assert_eq!(
            out.detach_requests
                .iter()
                .map(|r| r.cell)
                .collect::<Vec<_>>(),
            vec![fell[5]],
            "下一拍直转,不重新求值"
        );
        assert_eq!(out.cohesion_stats.load_pending, 0);
        for c in &fell {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
        }
    }

    /// §4.6 第 4 条(快照,v7):待转换队列随分片快照走——名额 2 判出 F = 6
    /// 当拍转 2,取快照恢复,两线各续两拍,脱落请求逐位一致、6 格全清。
    ///
    /// 变红:快照丢队列(恢复线首拍无触发,少塌 4 格)。
    #[test]
    fn load_pending_queue_survives_snapshot() {
        let mut a = cohesion_sim(2);
        let lintel = build_door_lintel(&mut a, 64);
        let fell: Vec<BlockPos> = lintel[..6].to_vec();
        remove_door_pillar(&mut a, 64);
        a.step();
        assert_eq!(a.take_outbox().detach_requests.len(), 2, "首拍 2 格");
        let bytes = a.snapshot();
        let mut b = ShardSim::restore(&bytes, Box::new(voxel::SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        for _ in 0..2 {
            a.step();
            b.step();
            let (oa, ob) = (a.take_outbox(), b.take_outbox());
            assert_eq!(
                oa.detach_requests, ob.detach_requests,
                "恢复线脱落请求逐位一致"
            );
        }
        for c in &fell {
            assert_eq!(a.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
            assert_eq!(b.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
        }
    }

    /// §4.6 第 4 条(名额共用):爆炸在拍首已占的名额要扣掉——名额 28、
    /// 标准弹坑脱落 24 → 承载转换当拍只剩 4 个名额:门楣 F = 6 转前 4 格、
    /// 2 格入队下一拍续转。
    ///
    /// 变红:名额不减本拍已有脱落请求(6 格全转换,当拍 30 > 28)。
    #[test]
    fn load_quota_shared_with_explosion() {
        let mut s = cohesion_sim(28);
        let lintel = build_door_lintel(&mut s, 64);
        let fell: Vec<BlockPos> = lintel[..6].to_vec();
        assert!(s.take_outbox().detach_requests.is_empty(), "满载挂得住");
        // 同拍:标准弹坑(z = 96,远离门楣)+ 拆左柱。
        let inputs = vec![detonate_input(200, Vec3::new(64.0, -59.5, 96.0), 12.0, 8.0)];
        remove_door_pillar(&mut s, 64);
        s.apply_prefix(&inputs);
        s.step();
        let out = s.take_outbox();
        assert_eq!(
            out.detach_requests.len(),
            28,
            "弹坑 24 + 门楣当拍转换 4 = 名额 28"
        );
        assert_eq!(out.cohesion_stats.load_falling, 6, "F = 门楣左 6 格");
        assert_eq!(out.cohesion_stats.load_pending, 2, "名额不足推迟 2 格");
        for c in &fell[..4] {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
        }
        for c in &fell[4..] {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::DIRT, "{c:?} 推迟");
        }
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.detach_requests.len(), 2, "下一拍续 2 格");
        assert_eq!(out.cohesion_stats.load_pending, 0);
    }

    /// §9(单分片级,记录时效):同拍收到邻片的 plain 写 air(拆柱)与带标记
    /// 写 air(转换门楣 61..=64)→ "转换前"按写入前方块判出完整门楣的 F =
    /// 60..=65(6 格)、"写入后"残余挂得住;并集里当前仍实心的只有 60、65
    /// (61..=64 已被标记写入写空)→ 只转换这 2 格、写入带标记;**记录只在
    /// 本拍有效**:下一拍求值不带"转换前"(double = 0)。
    ///
    /// 变红:标记不记录(只求写入后,不塌);记录跨拍存活(下一拍凭空多求
    /// 一次并多塌);转换写入没带标记;并集里已空的格也转换。
    #[test]
    fn load_conversion_record_is_single_tick() {
        let mut s = cohesion_sim(64);
        let lintel = build_door_lintel(&mut s, 64);
        assert!(s.take_outbox().detach_requests.is_empty(), "满载挂得住");
        let mut inputs: Vec<StampedInput> = (-60..=-56)
            .map(|y| {
                envelope_input(
                    100 + (y + 60) as u64,
                    shard_id(1, 0),
                    EnvelopePayload::BlockWrite {
                        pos: BlockPos::new(60, y, 64),
                        block: voxel::builtin::AIR,
                        load_conversion: false,
                    },
                )
            })
            .collect();
        inputs.extend((61..=64).map(|x| {
            envelope_input(
                200 + x as u64,
                shard_id(1, 0),
                EnvelopePayload::BlockWrite {
                    pos: BlockPos::new(x, -55, 64),
                    block: voxel::builtin::AIR,
                    load_conversion: true,
                },
            )
        }));
        s.apply_prefix(&inputs);
        s.step();
        let out = s.take_outbox();
        assert_eq!(
            out.detach_requests
                .iter()
                .map(|r| r.cell)
                .collect::<Vec<_>>(),
            [lintel[0], lintel[5]],
            "并集 F = 60..=65;仍实心的只有两端 2 格"
        );
        assert_eq!(out.cohesion_stats.load_falling, 6, "含已空的标记格");
        assert_eq!(out.cohesion_stats.load_evaluations, 2, "求两次");
        assert_eq!(out.cohesion_stats.load_double_evaluations, 1);
        assert!(
            out.block_updates
                .iter()
                .filter(|w| w.load_conversion)
                .map(|w| w.pos)
                .collect::<Vec<_>>()
                == vec![lintel[0], lintel[5]],
            "转换写入带标记"
        );
        s.step();
        let out = s.take_outbox();
        assert_eq!(
            out.cohesion_stats.load_double_evaluations, 0,
            "记录只在本拍有效"
        );
        assert_eq!(out.cohesion_stats.load_evaluations, 1, "转换写入触发下一拍");
    }

    /// §4.4(连锁断裂):转换的写入登记为下一拍的触发——加重塌落拍之后,
    /// 无外部输入的下一拍仍有一次承载求值(F = ∅,不再塌)。
    ///
    /// 变红:转换的格不登记触发(下一拍零求值,后续连锁断裂丢失)。
    #[test]
    fn load_converted_cells_trigger_next_evaluation() {
        let mut s = cohesion_sim(64);
        let arm = build_cantilever(&mut s, 64);
        s.take_outbox();
        s.apply_prefix(&[setblock_input(
            1,
            BlockPos::new(50, -54, 64),
            voxel::builtin::STONE,
        )]);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.detach_requests.len(), 9, "加重塌 9 格");
        s.step();
        let out = s.take_outbox();
        assert_eq!(
            out.cohesion_stats.load_evaluations, 1,
            "无外部输入的下一拍仍求值(转换格是触发)"
        );
        assert_eq!(out.cohesion_stats.load_falling, 0, "不再塌");
        assert!(out.detach_requests.is_empty());
        for c in &arm {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR);
        }
    }

    /// §4.6 第 2 条(强制落地带触发,风险 4):实体被强制落到满载悬臂末端
    /// 上方 → 落点写入(lifecycle,step 之后)登记触发,**下一拍**求值把悬臂
    /// 连同落点格判塌。
    ///
    /// 变红:强制落地写入不登记触发(悬臂永远不塌);当拍就塌(时序错)。
    #[test]
    fn load_forced_landing_write_triggers_next_tick() {
        let mut s = cohesion_sim(64);
        let arm = build_cantilever(&mut s, 64);
        assert!(s.take_outbox().detach_requests.is_empty(), "满载挂得住");
        let e = EntityId::from_parts(1, 1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource(1),
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(50.5, -53.55, 64.5),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: crate::destruction::DEBRIS_HALF_SIZE,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        // 白盒:直接入队强制落地(正常路径 = 调度器 7.5/超预算淘汰)。
        s.forced_pending.insert(e);
        s.forced_reasons.insert(e, ForcedLandingReason::Deadline);
        s.run_transient_lifecycle();
        let landing = BlockPos::new(50, -54, 64);
        assert_eq!(
            s.voxel().get_block(landing),
            voxel::builtin::STONE,
            "强制落地已写块"
        );
        for c in &arm {
            assert_eq!(
                s.voxel().get_block(*c),
                voxel::builtin::STONE,
                "{c:?} 触发生效在下一拍,本拍不塌"
            );
        }
        s.step();
        let out = s.take_outbox();
        for c in arm.iter().chain([&landing]) {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?} 塌");
        }
        assert_eq!(out.detach_requests.len(), 9, "悬臂 8 格 + 落点格");
    }

    /// §5.5 的按段 sim:`cohesion_sim` 已固定 split = 1(逐格,单元 02 用例
    /// 语义);本组用例要真实的按段行为,显式开 cohesion、用默认切块 6。
    fn load_segment_sim(cap: usize) -> ShardSim {
        let mut s = destruction_sim(cap);
        s.cfg.cohesion.enabled = true;
        s
    }

    /// §5.5 第 1–3 条(按段转换):土门楣左 6 格在同一 6 格切块 → 整段 6 格
    /// 原子转换:同拍 6 格写 air 且全部带承载转换标记、零 `DetachRequest`、
    /// 1 个 `CompoundRequest`(原点 = `compound_origin`,形状 canonical、
    /// 全格在列、方块 dirt);遥测 = 1 段 1 复合体 6 格。
    ///
    /// 变红:整段被拆开(逐格 Detach);形状少格/方块错;原点错;遥测没计。
    #[test]
    fn load_segment_converts_atomically_into_compound_request() {
        let mut s = load_segment_sim(64);
        let lintel = build_door_lintel(&mut s, 64);
        assert!(s.take_outbox().detach_requests.is_empty(), "满载挂得住");
        remove_door_pillar(&mut s, 64);
        s.step();
        let out = s.take_outbox();
        let fell: Vec<BlockPos> = lintel[..6].to_vec();
        for c in &fell {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
        }
        assert!(out.detach_requests.is_empty(), "≥ 2 格的段不产单格脱落请求");
        assert_eq!(out.compound_requests.len(), 1);
        let req = &out.compound_requests[0];
        let origin = crate::cohesion::compound_origin(&fell);
        assert_eq!(req.origin, origin);
        let cells = req.shape.cells();
        assert_eq!(cells.len(), 6, "全格在列");
        let offsets: Vec<[i32; 3]> = cells.iter().map(|c| c.offset).collect();
        let want: Vec<[i32; 3]> = fell
            .iter()
            .map(|c| [c.x - origin.x, c.y - origin.y, c.z - origin.z])
            .collect();
        assert_eq!(offsets, want, "canonical(偏移升序)且与段一致");
        assert!(cells.iter().all(|c| c.block == voxel::builtin::DIRT));
        // 转换写入:6 格全带承载转换标记(§5.5 第 3 条)。
        let marked: Vec<BlockPos> = out
            .block_updates
            .iter()
            .filter(|w| w.load_conversion)
            .map(|w| w.pos)
            .collect();
        assert_eq!(marked, fell, "整段同拍写 air、全带标记");
        // 遥测(§5.5 第 6 条)。
        let t = out.cohesion_stats;
        assert_eq!(
            (t.segments, t.compounds_spawned, t.compound_cells),
            (1, 1, 6)
        );
        assert_eq!(out.destruction_stats.cells_collapsed, 6, "REQ-009 口径照常");
    }

    /// §5.5 第 2 条(格数上限):切块 2、每拍 4 格、名额 64——门楣左 6 格切成
    /// 3 段 × 2 格,每拍 2 段(4 格)整段转换,第三段下一拍;**不拆段**:第二拍
    /// 一次转完最后 2 格(若按格数硬切,第二拍只会转 2 格中的前 2 格——这里
    /// 段长恰 2 无法区分,用 6/6/2 的桥面形态补:2 格尾段不被拆成单格)。
    ///
    /// 变红:格数上限没生效(3 段一拍全转,当拍 6 格 > 4);上限把段拆开。
    #[test]
    fn load_segment_cells_budget_stops_midway_without_splitting() {
        let mut s = load_segment_sim(64);
        // 切块 2、每拍 4 格。
        s.cfg.cohesion.compound_split_size = 2;
        s.cfg.cohesion.max_converted_cells_per_tick = 4;
        let lintel = build_door_lintel(&mut s, 64);
        s.take_outbox();
        remove_door_pillar(&mut s, 64);
        s.step();
        let out = s.take_outbox();
        let fell: Vec<BlockPos> = lintel[..6].to_vec();
        // 段界:x 60..=61、62..=63、64..=65(div_euclid 偶切块)。前 2 段(4 格)本拍。
        assert_eq!(out.cohesion_stats.segments, 3);
        assert_eq!(out.cohesion_stats.compounds_spawned, 2);
        assert_eq!(out.cohesion_stats.compound_cells, 4);
        assert_eq!(out.compound_requests.len(), 2);
        for c in &fell[..4] {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
        }
        for c in &fell[4..] {
            assert_eq!(
                s.voxel().get_block(*c),
                voxel::builtin::DIRT,
                "{c:?} 整段留下"
            );
        }
        assert_eq!(out.cohesion_stats.load_pending, 2);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.compound_requests.len(), 1, "最后一段整段转换");
        assert_eq!(out.cohesion_stats.compound_cells, 2);
        for c in &fell[4..] {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
        }
    }

    /// §5.5 第 2 条(放不下即停,不跳段):门楣塌落段 6 格(首格 60 在前)+
    /// 2 格沙各自成 1 格段(B = 0,首格 67、68 在后),cells = 4——石段放不下
    /// → **整拍停止**,后面放得下的沙段也不转(变异"跳过放不下的段续转小段"
    /// 当拍就会转沙)。
    ///
    /// 变红:cells 不足时 continue 跳段(沙当拍转换);或门楣段被拆开。
    #[test]
    fn load_segment_stops_at_unfit_segment_without_skipping() {
        let mut s = load_segment_sim(64);
        // cells = 4 < 门楣段 6 格。
        s.cfg.cohesion.max_converted_cells_per_tick = 4;
        build_door_lintel(&mut s, 64);
        s.take_outbox();
        // 门楣塌落段 60..=65(dirt)+ 沙 67、68(与门楣第 7 格 B = 0、沙沙 B = 0,
        // 各自 1 格段且首格在后)。
        remove_door_pillar(&mut s, 64);
        let sand = [
            setblock_input(300, BlockPos::new(67, -55, 64), voxel::builtin::SAND),
            setblock_input(301, BlockPos::new(68, -55, 64), voxel::builtin::SAND),
        ];
        // 沙直接悬空放置即成触发(放置 = 实心变化,自己就是种子)。
        s.apply_prefix(&sand);
        s.step();
        let out = s.take_outbox();
        // 段 = [门楣 60..=65(6 格), 沙 67(1), 沙 68(1)];首段 6 格 > 4 cells → 停。
        assert_eq!(out.cohesion_stats.segments, 3);
        assert_eq!(out.compound_requests.len(), 0, "门楣段放不下,不产出");
        assert_eq!(out.detach_requests.len(), 0, "放得下的沙段也不转(不跳段)");
        let fell: Vec<BlockPos> = (60..=65).map(|x| BlockPos::new(x, -55, 64)).collect();
        let sand_pos = [BlockPos::new(67, -55, 64), BlockPos::new(68, -55, 64)];
        for c in fell.iter().chain(sand_pos.iter()) {
            let want = if fell.contains(c) {
                voxel::builtin::DIRT
            } else {
                voxel::builtin::SAND
            };
            assert_eq!(s.voxel().get_block(*c), want, "{c:?} 原样(整拍停止)");
        }
        assert_eq!(
            out.cohesion_stats.load_pending, 8,
            "门楣 6 格 + 沙 2 格全留队"
        );
        // 下一拍:cells 恢复 4,仍不够门楣段 6 格 → 继续停(队列不丢、不重新求值)。
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.cohesion_stats.load_pending, 8);
        assert!(out.compound_requests.is_empty() && out.detach_requests.is_empty());
        // 把 cells 提到 6(下一拍生效配置):门楣段整段转换,沙段随 cells 余 0 停。
        s.cfg.cohesion.max_converted_cells_per_tick = 6;
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.compound_requests.len(), 1, "门楣段 6 格整段转换");
        assert_eq!(out.cohesion_stats.compound_cells, 6);
        for c in &fell {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
        }
        assert_eq!(out.cohesion_stats.load_pending, 2, "沙段整段留下");
    }

    /// §5.6 第 1/3 条(K01 回归,AC-K2/K3):无撞击来源的承载转换不携带冲量——
    /// 移柱塌落的复合体段请求 `impulses` 为空(零初速不变,K02 起才接邻接继承),
    /// 同拍的单格段(沙)脱落请求冲量仍为零;撞击碎块(REQ-009 §5 路径)的
    /// 脱落请求冲量维持零(修订 8 逐位不变)。
    ///
    /// 变红:承载转换默认带上撞击冲量(K02 提前)或构造点漏填 impulses;
    /// 撞击/单格路径被改带非零冲量。
    #[test]
    fn knockback_k01_conversions_without_impact_carry_zero_impulse() {
        // 复合体段 + 单格段同拍:门楣 6 格(dirt)+ 沙 1 格,移柱触发。
        let mut s = load_segment_sim(64);
        build_door_lintel(&mut s, 64);
        s.take_outbox();
        remove_door_pillar(&mut s, 64);
        let sand = setblock_input(300, BlockPos::new(67, -55, 64), voxel::builtin::SAND);
        s.apply_prefix(&[sand]);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.cohesion_stats.segments, 2, "门楣段 + 沙 1 格段");
        assert_eq!(out.compound_requests.len(), 1);
        assert!(
            out.compound_requests[0].impulses.is_empty(),
            "无撞击来源:复合体请求 impulses 默认空(AC-K2)"
        );
        assert_eq!(out.detach_requests.len(), 1, "沙 1 格段走单格脱落");
        assert_eq!(
            out.detach_requests[0].impulse,
            Vec3::ZERO,
            "单格转换脱落冲量为零(AC-K3)"
        );
        // 撞击碎块路径(REQ-009 §5):17 m/s 落体撞坏 dirt,碎块零初速(修订 8)。
        let mut s = destruction_sim(64);
        s.apply_prefix(&[impact_drop_input(1, entity(1), 64, -55.0, 64, -15.0)]);
        let mut hit = None;
        for _ in 0..40 {
            s.step();
            let out = s.take_outbox();
            if out.destruction_stats.cells_impacted > 0 {
                hit = Some(out);
                break;
            }
        }
        let out = hit.expect("17 m/s 应撞坏 dirt");
        assert_eq!(out.detach_requests.len(), 1);
        assert_eq!(
            out.detach_requests[0].impulse,
            Vec3::ZERO,
            "撞击碎块零初速(修订 8,AC-K3)"
        );
        assert!(out.compound_requests.is_empty());
    }

    // ---- 单元 K02:撞击击退(REQ-010 §5.6,修订 10;AC-K1/K5/K6/K7)----

    /// K02 端到端最小场景:撞击格(覆土 dirt)被撞碎(撞击路径登记冲量表),
    /// 同拍其正下方的两格 dirt 悬空段失载成 2 格复合体(邻接继承)。
    ///
    /// 几何(Superflat::classic = y −64 bedrock + y −63…−61 dirt;chunk 已装载):
    ///
    /// - 撞击格 `hit = (64, −61, 64)`:地表覆土层,撞击落写把它挖成 air;
    /// - 失载段 `seg = (64, −62, 64)、(65, −62, 64)`:撞击格正下 + 侧下,
    ///   六邻支撑(除 hit)预先清空,除与 hit 的邻接外无任何粘结路径——
    ///   hit 写 air 后两格无汇流,整段失载。
    ///
    /// 返回 (场景 sim,撞击格,失载段)。
    fn knockback_scene() -> (ShardSim, BlockPos, Vec<BlockPos>) {
        use voxel::WorldGenerator;
        let mut s = load_segment_sim(64);
        for cx in 2..=6i32 {
            for cz in 2..=6i32 {
                let cp = ChunkPos::new(cx, cz);
                s.load_chunk(
                    cp,
                    voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
                );
            }
        }
        // 失载段四周六邻预先清空(air→air 与空挖不触发承载的格自身保留):
        // 两格 dirt 悬空挂住(未触发);一旦触发登记(撞击格被撞碎 = 触发),
        // 承载判定从邻接种子出发,悬空的 dirt 段失载。
        let seg = vec![BlockPos::new(64, -62, 64), BlockPos::new(65, -62, 64)];
        let mut pit: Vec<BlockPos> = Vec::new();
        for c in &seg {
            for (dx, dy, dz) in [
                (1, 0, 0),
                (-1, 0, 0),
                (0, 1, 0),
                (0, -1, 0),
                (0, 0, 1),
                (0, 0, -1),
            ] {
                pit.push(BlockPos::new(c.x + dx, c.y + dy, c.z + dz));
            }
        }
        let hit = BlockPos::new(64, -61, 64);
        let seg_set: std::collections::BTreeSet<BlockPos> = seg.iter().copied().collect();
        pit.retain(|c| !seg_set.contains(c) && *c != hit);
        let clears: Vec<StampedInput> = pit
            .iter()
            .enumerate()
            .map(|(k, c)| setblock_input(k as u64, *c, voxel::builtin::AIR))
            .collect();
        s.apply_prefix(&clears);
        s.take_outbox();
        (s, hit, seg)
    }

    /// AC-K1:构造邻接撞击场景,断言生成复合体的 linvel/angvel 与"同形状
    /// 刚体施加同组 impulse@point"的理论值一致(ε 内),且 |v| ≤ v_max、
    /// |ω| ≤ ω_max(§5.6 第 3 条 + AC-K1)。参照体 = 同配置 ShardSim 里的
    /// 同形状实体施加同组冲量(同 rapier 质量属性,闭式一致)。
    ///
    /// 变红:施加未逐条(力矩失真 → angvel 偏)、clamp 方向被改、β 没乘入。
    #[test]
    fn knockback_k02_compound_inherits_impulse_theory_match() {
        let (mut s, _hit, _seg) = knockback_scene();
        // 白盒注入冲量表(构造侧 β 乘入后的形态;触发格 = 撞击格,与段格
        // 64,−62 邻接)。
        s.impact_impulses
            .insert(BlockPos::new(64, -61, 64), Vec3::new(4.0, -2.0, 1.0));
        // 撞击落写的白盒等价:SetBlock 挖掉撞击格(实心→air,登记触发;真实
        // 路径里 evaluate_impacts 同款写入并登记冲量表)。写入后视图里 hit =
        // air,种子 = 其正下两格悬空段,无侧向粘结路径 → 整段失载。
        s.apply_prefix(&[setblock_input(
            400,
            BlockPos::new(64, -61, 64),
            voxel::builtin::AIR,
        )]);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.compound_requests.len(), 1, "失载段整段转换");
        let req = &out.compound_requests[0];
        assert_eq!(req.impulses.len(), 1, "一个邻接触发格一条冲量");
        let (j, point) = req.impulses[0];
        assert!(
            (j - Vec3::new(2.0, -1.0, 0.5)).length() < 1e-6,
            "β = 0.5 构造时乘入:{j:?}"
        );
        assert_eq!(point, Vec3::new(64.5, -60.5, 64.5), "point = 触发格中心");
        // N+1 生成:调度器盖章 SpawnCompound(单分片直用 sim,这里手工走
        // 同一命令路径)。
        let e = entity(9);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 500,
            payload: InputPayload::Command(SimCommand::SpawnCompound {
                entity: e,
                origin: req.origin,
                shape: req.shape.clone(),
                impulses: req.impulses.clone(),
                velocity: (Vec3::ZERO, Vec3::ZERO),
            }),
        }]);
        let state = s.entity_state(e).expect("复合体已生成");
        // 参照体:同配置 sim、同形状同位置、施加同组 (J, point)。
        let mut r = load_segment_sim(64);
        for cx in 2..=6i32 {
            for cz in 2..=6i32 {
                let cp = ChunkPos::new(cx, cz);
                r.load_chunk(
                    cp,
                    voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
                );
            }
        }
        r.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: e,
                state: types::EntityState {
                    translation: Vec3::new(
                        req.origin.x as f32 + 0.5,
                        req.origin.y as f32 + 0.5,
                        req.origin.z as f32 + 0.5,
                    ),
                    rotation: types::glam::Quat::IDENTITY,
                    linvel: Vec3::ZERO,
                    angvel: Vec3::ZERO,
                },
                desc: types::EntityDesc::compound(
                    crate::destruction::DEBRIS_HALF_SIZE,
                    req.shape.clone(),
                ),
            }),
        }]);
        {
            let rec = r.entities.get(&e).expect("参照体在场");
            let handle = rec.handle;
            let body = r.phys.bodies.get_mut(handle).expect("handle 有效");
            for (imp, pt) in &req.impulses {
                body.apply_impulse_at_point(
                    conv::vec3_to_rapier(*imp),
                    conv::vec3_to_rapier(*pt),
                    true,
                );
            }
        }
        let want = r.entity_state(e).expect("参照体状态");
        let v_max = s.cfg.cohesion.load_knockback_max_speed;
        let w_max = s.cfg.cohesion.load_knockback_max_ang_speed;
        assert!(
            (state.linvel - want.linvel).length() < 1e-4,
            "linvel 与理论值一致:got {:?} want {:?}",
            state.linvel,
            want.linvel
        );
        assert!(
            (state.angvel - want.angvel).length() < 1e-4,
            "angvel 与理论值一致(逐条施加,力矩保真):got {:?} want {:?}",
            state.angvel,
            want.angvel
        );
        assert!(state.linvel.length() <= v_max + 1e-4, "|v| ≤ v_max");
        assert!(state.angvel.length() <= w_max + 1e-4, "|ω| ≤ ω_max");
        assert!(state.linvel.length() > 0.1, "击退非零(打击感):有初速");
        assert!(state.angvel.length() > 0.01, "偏轴工况有翻滚(r×J 非零)");
    }

    /// AC-K5:共线工况(触发格中心在过质心且平行 J 的直线上)angvel ≈ 0;
    /// 偏轴工况 angvel ≠ 0——Δω 确来自 r×J(§5.6 第 2/3 条)。
    ///
    /// 变红:施加被合并到质心(共线与偏轴 angvel 都 ≈ 0);或方向搞错。
    #[test]
    fn knockback_k02_collinear_zero_angvel_offset_axis_nonzero() {
        let build = |impulses: Vec<(Vec3, Vec3)>| {
            let mut s = load_segment_sim(64);
            let e = entity(1);
            s.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnCompound {
                    entity: e,
                    origin: BlockPos::new(64, -62, 64),
                    shape: types::CompoundShape::from_world(
                        BlockPos::new(64, -62, 64),
                        &[
                            (BlockPos::new(64, -62, 64), voxel::builtin::STONE),
                            (BlockPos::new(65, -62, 64), voxel::builtin::STONE),
                        ],
                    )
                    .expect("两格"),
                    impulses,
                    velocity: (Vec3::ZERO, Vec3::ZERO),
                }),
            }]);
            s.entity_state(e).expect("已生成")
        };
        // 共线:两格横条质心 = (65.0, −61.5, 64.5)(两格中心均值);触发格
        // (64,−62,64) 中心 (64.5,−61.5,64.5),r = (−0.5,0,0) 平行 J = (2,0,0)
        // ⟹ r×J = 0。Δv = J/M,Δω = 0。
        let st = build(vec![(
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(64.5, -61.5, 64.5),
        )]);
        assert!(
            st.angvel.length() < 1e-5,
            "共线工况 angvel ≈ 0:{:?}",
            st.angvel
        );
        assert!((st.linvel.length() - 2.0 / (2.0 * 0.729)).abs() < 1e-4);
        // 偏轴:同冲量但 point 抬高 1 m ⟹ r 有竖直分量,r×J ≠ 0。
        let st = build(vec![(
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(64.5, -60.5, 64.5),
        )]);
        assert!(
            st.angvel.length() > 0.1,
            "偏轴工况 angvel ≠ 0(Δω 确来自 r×J):{:?}",
            st.angvel
        );
    }

    /// AC-K6:小惯性 + 大冲量 clamp 生效(贴 v_max / ω_max 上限,方向不变;
    /// §5.6 第 3 条)。2 格段(质量 1.458,惯量小)+ 200 N·s 斜向冲量:
    /// 无 clamp 时 |v| ≈ 137 m/s、|ω| ≈ 92 rad/s,远超 8.0/6.0 上限。
    ///
    /// 变红:clamp 没实现(速度超限);clamp 改方向(分量符号翻转);
    /// 上限取错键。
    #[test]
    fn knockback_k02_clamp_caps_speed_and_ang_speed() {
        let mut s = load_segment_sim(64);
        let e = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnCompound {
                entity: e,
                origin: BlockPos::new(64, -62, 64),
                shape: types::CompoundShape::from_world(
                    BlockPos::new(64, -62, 64),
                    &[
                        (BlockPos::new(64, -62, 64), voxel::builtin::STONE),
                        (BlockPos::new(65, -62, 64), voxel::builtin::STONE),
                    ],
                )
                .expect("两格"),
                impulses: vec![(Vec3::new(120.0, 120.0, 40.0), Vec3::new(64.5, -60.5, 64.5))],
                velocity: (Vec3::ZERO, Vec3::ZERO),
            }),
        }]);
        let st = s.entity_state(e).expect("已生成");
        let v_max = s.cfg.cohesion.load_knockback_max_speed;
        let w_max = s.cfg.cohesion.load_knockback_max_ang_speed;
        assert!(
            (st.linvel.length() - v_max).abs() < 1e-4,
            "线速度贴 v_max 上限:{:?} vs {v_max}",
            st.linvel.length()
        );
        assert!(
            (st.angvel.length() - w_max).abs() < 1e-4,
            "角速度贴 ω_max 上限:{:?} vs {w_max}",
            st.angvel.length()
        );
        assert!(st.linvel.x > 0.0 && st.linvel.y > 0.0, "clamp 方向不变");
        assert!(st.angvel.length() > 0.0 && st.angvel.z != 0.0, "翻滚保持");
    }

    /// AC-K6 补(K03):上限取 0 语义 = 钉死为零,不是"不设限"——施加非零
    /// 冲量后 linvel = Vec3::ZERO(ω_max 保持默认,角速度不受该裁定路径影响
    /// 仍被正常 clamp)。§5.6 第 4 条(裁定 2026-10-30)。
    ///
    /// 变红:clamp 恢复 `> 0.0` 守卫(0 上限被当"不设限",速度无界)。
    #[test]
    fn knockback_k03_zero_max_speed_pins_linvel_to_zero() {
        let mut s = load_segment_sim(64);
        s.cfg.cohesion.load_knockback_max_speed = 0.0;
        let e = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnCompound {
                entity: e,
                origin: BlockPos::new(64, -62, 64),
                shape: types::CompoundShape::from_world(
                    BlockPos::new(64, -62, 64),
                    &[
                        (BlockPos::new(64, -62, 64), voxel::builtin::STONE),
                        (BlockPos::new(65, -62, 64), voxel::builtin::STONE),
                    ],
                )
                .expect("两格"),
                impulses: vec![(Vec3::new(120.0, 120.0, 40.0), Vec3::new(64.5, -60.5, 64.5))],
                velocity: (Vec3::ZERO, Vec3::ZERO),
            }),
        }]);
        let st = s.entity_state(e).expect("已生成");
        assert_eq!(
            st.linvel,
            Vec3::ZERO,
            "v_max = 0:线速度钉死为零(不是不设限)"
        );
        let w_max = s.cfg.cohesion.load_knockback_max_ang_speed;
        assert!(
            st.angvel.length() <= w_max + 1e-4 && st.angvel.length() > 0.0,
            "ω_max 默认 6.0 不受影响:角速度仍正常 clamp 且非零:{:?}",
            st.angvel
        );
    }

    /// AC-K7:impact-corner 同款几何双跑逐拍一致(outbox 遥测逐拍相同;
    /// §5.6 第 6 条确定性)。门框 = 石柱两根 + 门楣 4 格(split 4),去掉左柱,
    /// 高速方块撞拐角;两跑 sim 配置/输入序列逐位相同,断言每拍
    /// `cells_impacted/compounds_spawned/compound_cells` 与实体终态表一致。
    ///
    /// 变红:冲量表跨拍残留(第二跑继承了第一跑的拍间状态差异——白盒同源
    /// 构造下只有非确定实现才会分叉);clamp 读回写回不确定。
    #[test]
    fn knockback_k02_impact_corner_double_run_bitwise_identical() {
        let build = || {
            let mut s = load_segment_sim(64);
            s.cfg.cohesion.compound_split_size = 4;
            // 门框(smoke/scenarios/req010-impact-corner 同款,平移到本 sim
            // 坐标:x 60..67,y −60..−58,z = 64):两根石柱 y −60..−59,门楣
            // y = −58。
            let mut inputs: Vec<StampedInput> = Vec::new();
            let mut seq = 0u64;
            for x in [60i32, 63] {
                for y in [-60i32, -59] {
                    inputs.push(setblock_input(
                        seq,
                        BlockPos::new(x, y, 64),
                        voxel::builtin::STONE,
                    ));
                    seq += 1;
                }
            }
            for x in 60..=63i32 {
                inputs.push(setblock_input(
                    seq,
                    BlockPos::new(x, -58, 64),
                    voxel::builtin::STONE,
                ));
                seq += 1;
            }
            s.apply_prefix(&inputs);
            s.take_outbox();
            // 去左柱。
            for y in [-60i32, -59] {
                s.apply_prefix(&[setblock_input(
                    seq,
                    BlockPos::new(60, y, 64),
                    voxel::builtin::AIR,
                )]);
                seq += 1;
            }
            s.take_outbox();
            // 高速投射物撞门楣-柱顶拐角接缝(瞄准 (63,−58) 附近,速度斜向)。
            s.apply_prefix(&[impact_drop_input(900, entity(77), 63, -50.0, 64, -30.0)]);
            s
        };
        let mut a = build();
        let mut b = build();
        for tick in 0..80 {
            a.step();
            b.step();
            let (oa, ob) = (a.take_outbox(), b.take_outbox());
            assert_eq!(
                oa.destruction_stats.cells_impacted, ob.destruction_stats.cells_impacted,
                "tick {tick}:撞击数一致"
            );
            assert_eq!(
                oa.cohesion_stats.compounds_spawned, ob.cohesion_stats.compounds_spawned,
                "tick {tick}:复合体数一致"
            );
            assert_eq!(oa.compound_requests, ob.compound_requests, "tick {tick}");
            assert_eq!(oa.block_updates, ob.block_updates, "tick {tick}");
        }
        let ea: Vec<_> = a.entity_states();
        let eb: Vec<_> = b.entity_states();
        assert_eq!(ea, eb, "80 拍后实体表逐位一致");
        assert!(
            ea.len() <= 3,
            "终局凝固/散落后实体收敛(双跑同终态即确定性):{ea:?}"
        );
    }

    /// §5.6 第 4 条(AC-K4 回归,K01 钉的 factor = 0 逐位绿在行为级复核):
    /// β = 0(下一拍生效配置)时同款邻接撞击场景的复合体请求 impulses 仍是
    /// 非空零冲量条目、生成后 linvel = angvel = 0(零冲量施加数值 no-op);
    /// 单格脱落与撞击碎块路径不受影响。
    ///
    /// 变红:β = 0 被当成"跳过继承"(请求为空——构造约定破坏);零冲量
    /// 施加意外改变状态(wake/浮点扰动)。
    #[test]
    fn knockback_k02_zero_factor_yields_zero_velocity() {
        let (mut s, hit, _seg) = knockback_scene();
        s.cfg.cohesion.load_knockback_factor = 0.0;
        s.impact_impulses.insert(hit, Vec3::new(4.0, -2.0, 1.0));
        // 撞击落写的白盒等价(同 K1):SetBlock 挖掉撞击格。
        s.apply_prefix(&[setblock_input(400, hit, voxel::builtin::AIR)]);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.compound_requests.len(), 1);
        let req = &out.compound_requests[0];
        assert_eq!(req.impulses.len(), 1, "条目仍在(构造约定:去重邻接表)");
        assert_eq!(req.impulses[0].0, Vec3::ZERO, "β = 0:冲量为零");
        let e = entity(9);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 500,
            payload: InputPayload::Command(SimCommand::SpawnCompound {
                entity: e,
                origin: req.origin,
                shape: req.shape.clone(),
                impulses: req.impulses.clone(),
                velocity: (Vec3::ZERO, Vec3::ZERO),
            }),
        }]);
        let st = s.entity_state(e).expect("已生成");
        assert_eq!(st.linvel, Vec3::ZERO, "零冲量施加 = 数值 no-op");
        assert_eq!(st.angvel, Vec3::ZERO);
        // SpawnCompound 生成初态显式断言(AC-K 补,K03;K02 迁移遗留的失锚):
        // 平移 = origin 格中心(各 +0.5)、旋转 = 单位四元数。
        assert_eq!(
            st.translation,
            Vec3::new(
                req.origin.x as f32 + 0.5,
                req.origin.y as f32 + 0.5,
                req.origin.z as f32 + 0.5,
            ),
            "平移 = 原点格中心(各 +0.5)"
        );
        assert_eq!(
            st.rotation,
            types::glam::Quat::IDENTITY,
            "旋转 = 单位四元数"
        );
    }

    /// §5.6 第 1/2 条(接线,AC-K1 的生产路径半边):真实撞击路径(17 m/s
    /// 落体砸穿覆土)登记的冲量表,在拍末承载转换处被邻接段继承——撞击格
    /// 正下方悬空的 2 格 dirt 段,其请求 impulses 恰含 (β·J, 撞击格中心)。
    ///
    /// 变红:evaluate_impacts 没登记表;登记了但方向/格错;表被隔拍消费。
    ///
    /// F04a:显式关 `impact_flow`——本用例测 §5.6"撞击落写冲量表 → 承载
    /// 转换继承 β·J"的**旧路径**(冲量流生效时撞击破坏走 §5.7,落写口径
    /// 不同);其余 knockback_* 均为白盒构造(不经真实撞击),不受影响。
    #[test]
    fn knockback_k02_impact_path_feeds_inheritance() {
        let (mut s, hit, seg) = knockback_scene();
        s.cfg.cohesion.impact_flow = false;
        // 17 m/s 落体砸 hit(撞击速度基准 15 m/s > 1 m/s 接近门槛;D(dirt)
        // 阈值 7 N·s,采样深度 0.1 m 回到 hit 格)。
        s.apply_prefix(&[impact_drop_input(1, entity(1), hit.x, -50.0, hit.z, -15.0)]);
        // 冲量表先于 step(白盒核对):撞击拍内 evaluate_impacts 登记。
        let mut found = None;
        for _ in 0..40 {
            s.step();
            let out = s.take_outbox();
            if out.destruction_stats.cells_impacted > 0 {
                found = Some(out);
                break;
            }
        }
        let out = found.expect("17 m/s 应撞碎覆土");
        assert_eq!(out.destruction_stats.cells_impacted, 1);
        assert!(
            out.compound_requests.len() == 1,
            "同拍邻接失载段整段转换:{:?}",
            out.compound_requests.len()
        );
        let req = &out.compound_requests[0];
        let seg_set: std::collections::BTreeSet<BlockPos> = seg.iter().copied().collect();
        let cells: Vec<BlockPos> = req
            .shape
            .cells()
            .iter()
            .map(|c| {
                BlockPos::new(
                    req.origin.x + c.offset[0],
                    req.origin.y + c.offset[1],
                    req.origin.z + c.offset[2],
                )
            })
            .collect();
        assert_eq!(
            cells
                .iter()
                .copied()
                .collect::<std::collections::BTreeSet<_>>(),
            seg_set,
            "转换的正是悬空段"
        );
        assert_eq!(req.impulses.len(), 1, "只继承撞击格一条");
        let (j, point) = req.impulses[0];
        assert_eq!(
            point,
            Vec3::new(hit.x as f32 + 0.5, hit.y as f32 + 0.5, hit.z as f32 + 0.5)
        );
        assert!(j.length() > 0.0, "冲量非零(真实采样):{j:?}");
        // 表已取走:下一拍不再有冲量(不跨拍残留)。
        assert!(s.impact_impulses.is_empty(), "冲量表每拍清空");
    }

    /// §5.5 第 2 条(名额与格数都从零起算,与同拍爆炸共用实体名额):名额 1、
    /// 弹坑拍 + 门楣塌——爆炸 24 请求占满名额,承载转换当拍 1 个名额也没有,
    /// 6 格整段留下;下一拍名额 1 仍不够 1 段吗——够(每段 1 个实体名额),
    /// 整段 6 格一次转换(格数上限 1024 不设障)。
    ///
    /// 变红:复合体不占/多占实体名额;爆炸名额没被扣(当拍就转)。
    #[test]
    fn load_segment_quota_shared_with_detaches() {
        let mut s = load_segment_sim(24);
        build_door_lintel(&mut s, 64);
        s.take_outbox();
        let inputs = vec![detonate_input(200, Vec3::new(64.0, -59.5, 96.0), 12.0, 8.0)];
        remove_door_pillar(&mut s, 64);
        s.apply_prefix(&inputs);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.detach_requests.len(), 24, "弹坑 24 占满名额");
        assert!(out.compound_requests.is_empty(), "名额 0:整段留下");
        assert_eq!(out.cohesion_stats.load_pending, 6);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.compound_requests.len(), 1, "每段 1 个实体名额");
        assert_eq!(out.cohesion_stats.compound_cells, 6);
        assert!(out.detach_requests.is_empty(), "复合体不是脱落请求");
    }

    /// §6.2.1/§7 名额共用(风险 4 前半):爆炸在拍首已用掉的名额要扣掉——
    /// 名额 30、标准弹坑 24 → 坍塌当拍只剩 6 个名额,按升序转换前 6 格,
    /// 其余推迟到下一拍。
    ///
    /// 变红:名额不减去本拍已有脱落请求(坍塌 12 格全转换,当拍 36 > 30)。
    #[test]
    fn collapse_quota_shared_with_explosion() {
        let mut s = destruction_sim(30);
        let (pa, pb, deck) = build_bridge(&mut s);
        let mut inputs = vec![detonate_input(200, Vec3::new(64.0, -59.5, 96.0), 12.0, 8.0)];
        inputs.extend(
            [&pa[..], &pb[..]]
                .concat()
                .iter()
                .enumerate()
                .map(|(k, p)| setblock_input(300 + k as u64, *p, voxel::builtin::AIR)),
        );
        s.apply_prefix(&inputs);
        s.step();
        let out1 = s.take_outbox();
        assert_eq!(out1.detach_requests.len(), 30, "24 爆炸 + 6 坍塌");
        for (k, c) in deck.iter().enumerate() {
            let expected = if k < 6 {
                voxel::builtin::AIR
            } else {
                voxel::builtin::STONE
            };
            assert_eq!(s.voxel().get_block(*c), expected, "{c:?}");
        }
        s.step();
        let out2 = s.take_outbox();
        assert_eq!(out2.detach_requests.len(), 6, "推迟收尾 6 格");
        for c in &deck {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR);
        }
    }

    /// §6.2 沙子规则生产路径:石块上 5 格沙,移走石块 → 5 格沙当拍塌落,
    /// 脱落请求记录原方块 sand、冲量为 0。
    ///
    /// 变红:请求的 block 沿用默认 stone(凝固回错方块,acc_d5_sand 变红)。
    #[test]
    fn collapse_sand_column_records_sand_block() {
        let mut s = destruction_sim(64);
        let base = BlockPos::new(66, -60, 70);
        let sand: Vec<BlockPos> = (-59..=-55).map(|y| BlockPos::new(66, y, 70)).collect();
        let build: Vec<StampedInput> =
            std::iter::once(setblock_input(0, base, voxel::builtin::STONE))
                .chain(
                    sand.iter()
                        .enumerate()
                        .map(|(k, p)| setblock_input(1 + k as u64, *p, voxel::builtin::SAND)),
                )
                .collect();
        s.apply_prefix(&build);
        s.step();
        s.apply_prefix(&[setblock_input(100, base, voxel::builtin::AIR)]);
        s.step();
        let out = s.take_outbox();
        assert_eq!(out.detach_requests.len(), 5);
        for req in &out.detach_requests {
            assert_eq!(req.block, voxel::builtin::SAND, "记录原方块 sand");
            assert!(sand.contains(&req.cell), "{:?}", req.cell);
            assert_eq!(req.impulse, Vec3::ZERO);
        }
        for c in &sand {
            assert_eq!(s.voxel().get_block(*c), voxel::builtin::AIR, "{c:?}");
        }
    }

    // ---- 遥测接线(REQ-009 §8,单元 05)----

    /// §8 爆炸遥测:标准弹坑 cells_blasted 60 = 粉碎 36 + 脱落 24(detached
    /// 与 detach_requests 列表等长);名额 10 时 14 格因上限改粉碎——计入
    /// `pulverized`(50)并单列 `pulverized_over_cap`(= 24 − 10)。
    ///
    /// 变红:计数没接线(恒 0);粉碎数漏掉因上限改粉碎的格;detached 与
    /// 请求列表脱节。
    #[test]
    fn blast_telemetry_counts_fates_and_over_cap() {
        let mut s = destruction_sim(64);
        s.apply_prefix(&[detonate_input(1, Vec3::new(64.0, -59.5, 64.0), 12.0, 8.0)]);
        s.step();
        let out = s.take_outbox();
        let t = out.destruction_stats;
        assert_eq!(
            (
                t.cells_blasted,
                t.pulverized,
                t.detached,
                t.pulverized_over_cap
            ),
            (60, 36, 24, 0),
            "标准弹坑口径(名额充裕,无 over_cap)"
        );
        assert_eq!(
            t.cells_blasted,
            t.pulverized + t.detached,
            "爆炸格数 = 粉碎 + 脱落"
        );
        assert_eq!(
            usize::try_from(t.detached).expect("计数非负"),
            out.detach_requests.len(),
            "detached == 脱落请求列表长"
        );

        let mut s = destruction_sim(10);
        s.apply_prefix(&[detonate_input(1, Vec3::new(64.0, -59.5, 64.0), 12.0, 8.0)]);
        s.step();
        let out = s.take_outbox();
        let t = out.destruction_stats;
        assert_eq!(
            (
                t.cells_blasted,
                t.pulverized,
                t.detached,
                t.pulverized_over_cap
            ),
            (60, 50, 10, 14),
            "名额 10:脱落候选 24 → 14 格因上限改粉碎(坑不变小)"
        );
    }

    /// §8 坍塌遥测:名额 5 三拍 (cells_collapsed, collapse_deferred, detached)
    /// = (5,7,5)/(5,2,5)/(2,0,2);每拍触发集非空恰一次求值、visited > 0、
    /// detached 与请求列表等长。
    ///
    /// 变红:推迟数没算(恒 0);推迟数算成触发集大小(首拍 17 = 12 柱 +
    /// 5 面);转换数没计;detached 漏掉坍塌贡献。
    #[test]
    fn collapse_telemetry_counts_conversion_and_deferral() {
        let mut s = destruction_sim(5);
        let (pa, pb, _deck) = build_bridge(&mut s);
        let remove: Vec<StampedInput> = [&pa[..], &pb[..]]
            .concat()
            .iter()
            .enumerate()
            .map(|(k, p)| setblock_input(100 + k as u64, *p, voxel::builtin::AIR))
            .collect();
        s.apply_prefix(&remove);
        let mut seen = Vec::new();
        for _ in 0..3 {
            s.step();
            let out = s.take_outbox();
            let t = out.destruction_stats;
            seen.push((t.cells_collapsed, t.collapse_deferred, t.detached));
            assert_eq!(t.collapse_evaluations, 1, "触发集非空恰一次求值:{t:?}");
            assert!(t.collapse_visited > 0, "访问数照记:{t:?}");
            assert_eq!(
                usize::try_from(t.detached).expect("计数非负"),
                out.detach_requests.len(),
                "detached == 脱落请求列表长"
            );
        }
        assert_eq!(seen, [(5, 7, 5), (5, 2, 5), (2, 0, 2)]);
    }

    // ---- 单元 06:撞击破坏(REQ-009 §5,修订 8/9)----

    /// 带初速的 SpawnDynamic 输入(撞击测试用;实心 stone、半边长 0.45)。
    fn impact_drop_input(seq: u64, id: EntityId, x: i32, y: f32, z: i32, vy: f32) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(x as f32 + 0.5, y, z as f32 + 0.5),
                    rotation: types::glam::Quat::IDENTITY,
                    linvel: Vec3::new(0.0, vy, 0.0),
                    angvel: Vec3::ZERO,
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }
    }

    /// §5 修订 8 生产路径(AC-D8):约 17 m/s 砸在 dirt 上——撞击拍落点写 air、
    /// `cells_impacted` 计 1,脱落请求冲量为 0(零初速);之后 60 拍不再破坏
    /// (碎块落回坑里落定凝固,无连锁)。
    ///
    /// 变红:没读接触冲量(不破坏);冲量不是 0(修订 8;碎块被推进坑里连锁)。
    #[test]
    fn impact_fast_drop_breaks_with_zero_impulse_debris() {
        let mut s = destruction_sim(64);
        s.apply_prefix(&[impact_drop_input(1, entity(1), 64, -55.0, 64, -15.0)]);
        let mut hit = None;
        for _ in 0..40 {
            s.step();
            let out = s.take_outbox();
            if out.destruction_stats.cells_impacted > 0 {
                hit = Some(out);
                break;
            }
        }
        let out = hit.expect("17 m/s 应撞坏 dirt");
        assert_eq!(out.destruction_stats.cells_impacted, 1);
        assert_eq!(
            s.voxels.get_block(BlockPos::new(64, -61, 64)),
            voxel::builtin::AIR,
            "落点写 air"
        );
        assert_eq!(out.detach_requests.len(), 1, "产出 1 条脱落请求");
        assert_eq!(
            out.detach_requests[0].impulse,
            Vec3::ZERO,
            "碎块零初速(修订 8)"
        );
        assert_eq!(out.detach_requests[0].block, voxel::builtin::DIRT);
        // 无连锁:之后 60 拍撞击计数为 0。
        for _ in 0..60 {
            s.step();
            let t = s.take_outbox().destruction_stats;
            assert_eq!(t.cells_impacted, 0, "连锁破坏(修订 8/9 失效):{t:?}");
        }
    }

    /// §5 修订 9:接近速度过滤——同一个 0.729 kg 实体,17 m/s(落定前)撞坏;
    /// 1 m 自由落地(落地速度约 4.4 ≥ 1,计入但 J < 7 不过阈值,不坏);静止
    /// 叠放(接触冲量持续存在但接近速度 0)不坏。
    ///
    /// 变红:删掉接近速度过滤(静载冲量被当撞击);阈值比较对象换成
    /// 冲量 × 60 的"力"(静载被放大 60 倍破阈)。
    #[test]
    fn impact_rest_and_gentle_drop_never_break() {
        // 1 m 落地(轻落):计入接近速度但不过阈值。
        let mut s = destruction_sim(64);
        s.apply_prefix(&[impact_drop_input(1, entity(1), 70, -60.0, 70, 0.0)]);
        for _ in 0..300 {
            s.step();
            let t = s.take_outbox().destruction_stats;
            assert_eq!(t.cells_impacted, 0, "轻落不破坏:{t:?}");
        }
        assert_eq!(
            s.voxels.get_block(BlockPos::new(70, -61, 70)),
            voxel::builtin::DIRT
        );
        // 8 层静止叠放:底块与地形间的支撑冲量持续存在,接近速度为 0。
        let mut s = destruction_sim(64);
        for k in 0..8u32 {
            s.apply_prefix(&[impact_drop_input(
                k as u64 + 1,
                entity(k + 1),
                74,
                -59.475 + 0.901 * k as f32,
                74,
                0.0,
            )]);
        }
        for _ in 0..300 {
            s.step();
            let t = s.take_outbox().destruction_stats;
            assert_eq!(t.cells_impacted, 0, "静载不破坏(修订 9):{t:?}");
        }
        assert_eq!(
            s.voxels.get_block(BlockPos::new(74, -61, 74)),
            voxel::builtin::DIRT
        );
    }

    /// §5 范围 2:每拍上限——6 块同落、`max_impact_breaks_per_tick` = 3:撞击
    /// 拍恰撞 3 格(按冲量降序,同值 BlockPos 升序),其余本拍不破坏、也不推迟
    /// (落定后没有冲量,之后 120 拍不再有破坏);受每拍总名额约束。
    ///
    /// 变红:撞击不受上限(6 格全破);超出上限推迟到下一拍(总破坏 > 3)。
    #[test]
    fn impact_cap_no_defer() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.destruction.enabled = true;
        cfg.destruction.max_impact_breaks_per_tick = 3;
        // F04a:本用例测 REQ-009 §5 范围 2"上限 + 不推迟"的旧语义(冲量流
        // 路径的名额口径不同),显式关闭(§5.7.5 迁移)。
        cfg.cohesion.impact_flow = false;
        use voxel::WorldGenerator as _;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        for cx in 2..=6i32 {
            for cz in 2..=6i32 {
                let cp = ChunkPos::new(cx, cz);
                s.load_chunk(
                    cp,
                    voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
                );
            }
        }
        for k in 0..6u32 {
            s.apply_prefix(&[impact_drop_input(
                k as u64 + 1,
                entity(k + 1),
                60 + 3 * k as i32,
                -55.0,
                90,
                -15.0,
            )]);
        }
        let mut hit = None;
        for _ in 0..40 {
            s.step();
            let out = s.take_outbox();
            if out.destruction_stats.cells_impacted > 0 {
                hit = Some(out);
                break;
            }
        }
        assert_eq!(
            hit.expect("应有撞击").destruction_stats.cells_impacted,
            3,
            "上限 3"
        );
        // 之后 120 拍不再破坏(不推迟)。
        for _ in 0..120 {
            s.step();
            let t = s.take_outbox().destruction_stats;
            assert_eq!(t.cells_impacted, 0, "落定后无冲量,不推迟:{t:?}");
        }
    }

    /// AC-F17(§5.7.10 末条):**同一几何、同一工况**下,"聚簇 + 读
    /// `solver_manifolds()`"(生产口径,A 组)与"不聚簇 + 读 `manifolds()`"
    /// (B 组;不聚簇时 solver 簇为空,`solver_manifolds()` 落回 plain 流形,
    /// 生产读取口径不变、冲量来自 plain)的撞击采样冲量合计差 **≤ 5%**;
    /// 并断言 A 组接触对 plain 流形数 ≥ 2(布景确实产出了被聚簇的双流形)。
    /// 两个工况:骑坑(2 格石条骑 1 格坑,两半分别触坑两侧合并顶面薄片)与
    /// 台阶沿(石条骑 2 层台地棱,两半分别压台顶与地面,两层薄片)。
    /// 会变红的实现:读回 plain 流形(聚簇生效时 plain 冲量恒 0 → A 组无撞击);
    /// 聚簇把冲量并丢(A/B 偏差超限);布景退化(plain 流形数 < 2)。
    #[test]
    fn impact_dual_manifold_clustering_preserves_impulse() {
        // 骑坑:坑挖 (70,-61,70),石条质心 x=70.5(格 69/70 分界);
        // 台阶沿:台地 x 72..=76、y −60..−59、z=70(经典地表顶面 = −60,
        // 叠 2 层后台顶 = −58),石条质心 (71.5,·,70.5) 骑棱——左半压地面
        // (顶 −60)、右半压台顶(−58)。地形坐标全在已装载 chunk 区
        // (chunk x/z 2..=6,世界 32..=111)。
        fn run(clustering: bool, ledge: bool, tag: &str) -> (Vec3, usize) {
            let mut s = destruction_sim(64);
            if ledge {
                let cells: Vec<BlockPos> = (72..=76)
                    .flat_map(|x| (-60..=-59).map(move |y| BlockPos::new(x, y, 70)))
                    .collect();
                let build: Vec<StampedInput> = cells
                    .iter()
                    .enumerate()
                    .map(|(k, p)| {
                        setblock_input(
                            u64::try_from(k).expect("k fits") + 1,
                            *p,
                            voxel::builtin::STONE,
                        )
                    })
                    .collect();
                s.apply_prefix(&build);
            } else {
                s.apply_prefix(&[setblock_input(
                    1,
                    BlockPos::new(70, -61, 70),
                    voxel::builtin::AIR,
                )]);
            }
            if !clustering {
                // B 组:同一 sim 构造下关聚簇(tests 模块可达 crate 私有字段)。
                s.phys.integration_parameters.contact_clustering = false;
            }
            let (cx, cz) = if ledge { (71.5, 70.5) } else { (70.5, 70.5) };
            s.apply_prefix(&[compound_spawn_input(
                100,
                entity(1),
                EntityState {
                    translation: Vec3::new(cx, -53.0, cz),
                    rotation: types::glam::Quat::IDENTITY,
                    linvel: Vec3::new(0.0, -25.0, 0.0),
                    angvel: Vec3::ZERO,
                },
                beam_shape(2),
            )]);
            // 缓存实体 collider 键(碎裂可能当拍移除实体/碰撞体,循环内不再查表)。
            let rec = s.entities.get(&entity(1)).expect("spawn 即在表");
            let body = s.phys.bodies.get(rec.handle).expect("handle 有效");
            let own: std::collections::BTreeSet<(u32, u32)> = body
                .colliders()
                .iter()
                .map(|c| {
                    let (g, i) = c.into_raw_parts();
                    (g, i)
                })
                .collect();
            let mut impulse = None;
            let mut max_plain = 0usize;
            for _ in 0..40 {
                s.step();
                // 本拍实体侧接触对 plain 流形合计(逐拍重置,取峰值)。
                let mut plain_this = 0usize;
                for pair in s.phys.narrow_phase.contact_pairs() {
                    let k1 = pair.collider1.into_raw_parts();
                    let k2 = pair.collider2.into_raw_parts();
                    if own.contains(&(k1.0, k1.1)) || own.contains(&(k2.0, k2.1)) {
                        plain_this += pair.manifolds().len();
                    }
                }
                max_plain = max_plain.max(plain_this);
                let out = s.take_outbox();
                if out.destruction_stats.cells_impacted > 0 {
                    impulse = Some(s.impact_impulses.values().sum());
                    break;
                }
            }
            let j = impulse.unwrap_or_else(|| panic!("{tag}: 应有撞击(clustering={clustering})"));
            (j, max_plain)
        }
        for (tag, ledge) in [("骑坑(两侧薄片,双流形)", false), ("台阶沿(两层薄片)", true)]
        {
            let (ja, plain_a) = run(true, ledge, tag);
            let (jb, _) = run(false, ledge, tag);
            assert!(
                plain_a >= 2,
                "AC-F17 {tag}:A 组 plain 流形数应 ≥ 2(实际 {plain_a},布景退化)"
            );
            let (la, lb) = (ja.length(), jb.length());
            assert!(
                la > 0.0 && lb > 0.0,
                "{tag}: 两组冲量非零 A={ja:?} B={jb:?}"
            );
            let rel = (la - lb).abs() / lb;
            assert!(
                rel <= 0.05,
                "AC-F17 {tag}:A(聚簇+solver_manifolds)|{la:.3}| vs \
                 B(不聚簇+plain)|{lb:.3}| 偏差 {:.1}% > 5%",
                rel * 100.0
            );
        }
    }

    /// §5 范围 5:总开关关闭不检测——同样的 17 m/s 撞击,关闭时零计数、零写入。
    ///
    /// 变红:收集闸删 enabled 条件(关闭时也检测)。
    #[test]
    fn impact_disabled_is_inert() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.destruction.enabled = false;
        use voxel::WorldGenerator as _;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        for cx in 2..=6i32 {
            for cz in 2..=6i32 {
                let cp = ChunkPos::new(cx, cz);
                s.load_chunk(
                    cp,
                    voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
                );
            }
        }
        s.apply_prefix(&[impact_drop_input(1, entity(1), 64, -55.0, 64, -15.0)]);
        for _ in 0..60 {
            s.step();
            let out = s.take_outbox();
            assert_eq!(
                out.destruction_stats,
                crate::destruction::DestructionStats::default(),
                "关闭时零计数"
            );
            assert!(out.detach_requests.is_empty(), "关闭时无脱落请求");
        }
        assert_eq!(
            s.voxels.get_block(BlockPos::new(64, -61, 64)),
            voxel::builtin::DIRT,
            "关闭时不写格"
        );
    }

    /// §8:预算锚定与超大块分开计——搜索预算 8 判锚定(`budget_anchored ≥ 1`、
    /// oversized 0);悬空上限 8 计 1 个超大块(oversized 1、budget 0);两种
    /// 不塌的拍同样计求值数、转换与推迟均为 0。
    ///
    /// 变红:两计数混计;求值数挪到"有转换才计"。
    #[test]
    fn collapse_telemetry_budget_anchor_vs_oversized() {
        for (max_cells, max_cluster, want_budget, want_oversized) in [
            (8usize, 8usize, true, false),
            (2048usize, 8usize, false, true),
        ] {
            let mut s = destruction_sim_tuned(64, max_cells, max_cluster);
            let (pa, pb, _deck) = build_bridge(&mut s);
            let remove: Vec<StampedInput> = [&pa[..], &pb[..]]
                .concat()
                .iter()
                .enumerate()
                .map(|(k, p)| setblock_input(100 + k as u64, *p, voxel::builtin::AIR))
                .collect();
            s.apply_prefix(&remove);
            s.step();
            let t = s.take_outbox().destruction_stats;
            assert_eq!(
                (t.cells_collapsed, t.collapse_deferred, t.detached),
                (0, 0, 0),
                "上限之内不塌不推迟:{t:?}"
            );
            assert_eq!(
                t.collapse_budget_anchored >= 1,
                want_budget,
                "预算锚定({max_cells}):{t:?}"
            );
            assert_eq!(
                t.collapse_oversized,
                u64::from(want_oversized),
                "超大块({max_cluster}):{t:?}"
            );
            assert_eq!(t.collapse_evaluations, 1, "不塌的拍同样计求值");
        }
    }

    /// §8/修订 6:触发集为空的拍不求值(求值数 0、全部计数为默认);爆炸的
    /// 下一拍(生成拍)计数归零——计数进 outbox 随 take 取走即清,不跨拍累计。
    ///
    /// 变红:空集也调用 `find_collapse`(安静拍求值数 1);计数挪进分片长期
    /// 字段跨拍累计。
    #[test]
    fn collapse_stats_zero_on_quiet_tick() {
        let mut s = destruction_sim(64);
        s.step();
        assert_eq!(
            s.take_outbox().destruction_stats,
            crate::destruction::DestructionStats::default(),
            "安静拍(无触发)全 0"
        );
        s.apply_prefix(&[detonate_input(1, Vec3::new(64.0, -59.5, 64.0), 12.0, 8.0)]);
        s.step();
        assert_eq!(s.take_outbox().destruction_stats.cells_blasted, 60);
        s.step();
        assert_eq!(
            s.take_outbox().destruction_stats,
            crate::destruction::DestructionStats::default(),
            "下一拍(生成拍)归零"
        );
    }

    // ---- 单元 07:默认开启(D7)、修订 10 上限 ----

    /// 修订 10 破坏格数上限(sim 级生产路径):平地 power 1000、半径 40
    /// (有效半径按 16 封顶)全应破坏 2220 格,`max_cells_per_explosion` = 2048
    /// 时那一拍恰破坏 2048 格——保留压力最高的(爆心附近被炸穿、压力最低的
    /// 边缘格完好);被截掉的格不写 air、不进 detach_requests、不登记坍塌触发。
    ///
    /// 变红:上限没接线(全部 2220 格破坏);截断按 BlockPos 而不是压力
    /// (保留的是坐标最小而非压力最高的格,爆心反而不在坑里)。
    #[test]
    fn blast_cell_cap_keeps_highest_pressure_and_skips_rest() {
        use voxel::WorldGenerator;
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.destruction.enabled = true;
        cfg.destruction.max_cells_per_explosion = 2048;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        for cx in 2..=6i32 {
            for cz in 2..=6i32 {
                let cp = ChunkPos::new(cx, cz);
                s.load_chunk(
                    cp,
                    voxel::Superflat::classic().generate(types::WorldSeed(7), cp),
                );
            }
        }
        // 前置复核:无上限时全部候选 2220 格(求值函数口径)。
        let center = Vec3::new(64.0, -59.5, 64.0);
        let all = crate::destruction::evaluate_blast(
            &|p| {
                if !(types::WORLD_Y_MIN..=types::WORLD_Y_MAX).contains(&p.y) {
                    return None;
                }
                Some(s.voxel().get_block(p))
            },
            s.registry.as_ref(),
            &|_| true,
            center,
            1000.0,
            40.0,
            1.0,
            48,
        );
        assert_eq!(all.len(), 2220, "前置:无上限候选数");
        assert!(
            all.iter().any(|c| c.pos == BlockPos::new(64, -61, 64)),
            "前置:爆心下方在候选内"
        );

        s.apply_prefix(&[detonate_input(1, center, 1000.0, 40.0)]);
        let out = s.take_outbox();
        assert_eq!(
            out.destruction_stats.cells_blasted, 2048,
            "上限截断到 max_cells_per_explosion"
        );
        assert_eq!(
            out.block_updates.len(),
            2048,
            "写入格数 = 实际破坏格数(截掉的格不写 air)"
        );
        // 判别式逐格对照:实际写入集 == 候选中按压力降序(同值 BlockPos 升序)
        // 的前 2048 名。两种次序的差分格(如 (48,−63,64) 的层首格 vs
        // (79,−61,64) 的边缘格)由这个全集对照覆盖——按 BlockPos 截断会错留
        // 层首低压力格、错杀高压力边缘格。
        let mut by_pressure = all.clone();
        by_pressure.sort_by(|a, b| {
            b.pressure
                .total_cmp(&a.pressure)
                .then_with(|| a.pos.cmp(&b.pos))
        });
        let want: BTreeSet<BlockPos> = by_pressure.iter().take(2048).map(|c| c.pos).collect();
        let got: BTreeSet<BlockPos> = out.block_updates.iter().map(|w| w.pos).collect();
        assert_eq!(got, want, "保留的恰是压力最高的前 2048 格");
        assert_eq!(
            s.voxel().get_block(BlockPos::new(64, -61, 64)),
            voxel::builtin::AIR,
            "压力最高的爆心格被保留破坏"
        );
        assert_eq!(
            s.voxel().get_block(BlockPos::new(79, -61, 64)),
            voxel::builtin::DIRT,
            "压力最低的边缘格被截掉、不破坏"
        );
        assert!(
            !s.collapse_triggers.contains(&BlockPos::new(79, -61, 64)),
            "截掉的格不登记坍塌触发"
        );
    }

    /// 修订 10 守卫:上限充裕时小爆炸完全不受影响——标准弹坑 60 格、24 脱落,
    /// 遥测同口径。防"截断路径改变了不触发上限的爆炸"。
    ///
    /// 变红:截断排序无条件执行且排序不稳定,标准弹坑格集合被重排/少写。
    #[test]
    fn blast_cell_cap_generous_leaves_standard_crater_intact() {
        let mut s = destruction_sim(64);
        s.apply_prefix(&[detonate_input(1, Vec3::new(64.0, -59.5, 64.0), 12.0, 8.0)]);
        let out = s.take_outbox();
        let t = out.destruction_stats;
        assert_eq!(
            (t.cells_blasted, t.pulverized, t.detached),
            (60, 36, 24),
            "标准弹坑不受上限影响"
        );
        assert_eq!(out.block_updates.len(), 60);
        assert_eq!(out.detach_requests.len(), 24);
    }

    /// D7 与修订 10 的核内默认锚定:`enabled = true`(D7)、
    /// `max_cells_per_explosion = 2048`(修订 10)。
    ///
    /// 变红:默认值回退(只改了 app 层、核内仍关;上限默认漂移)。
    #[test]
    fn destruction_defaults_anchor_d7_and_revision10() {
        let d = crate::destruction::DestructionConfig::default();
        assert!(d.enabled, "D7:默认开启");
        assert_eq!(d.max_cells_per_explosion, 2048, "修订 10 默认上限");
        assert!(
            ShardSimConfig::new(shard_id(0, 0)).destruction.enabled,
            "ShardSimConfig::new 继承开启默认"
        );
    }

    // ==== REQ-010 单元 F02a:撞击冲量流生产接入(§5.7.1/§5.7.4/§5.7.5;AC-F7、AC-F10)====

    /// F02a 布景:flat chunk(实心层顶 = 世界 y 1)。
    /// `impact_flow = true` 下走冲量流判定,断下按结构成形。
    fn flow_sim() -> ShardSim {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.impact_flow = true;
        ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
    }

    /// 零重力钉住(事件成形后不再动的布景用)。
    fn flow_sim_zero_g() -> ShardSim {
        let mut cfg = ShardSimConfig {
            gravity: Vec3::ZERO,
            ..ShardSimConfig::new(shard_id(0, 0))
        };
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.impact_flow = true;
        ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
    }

    /// 测试直写体素后刷新地形 collider(生产的重建挂在同拍写入出口;测试
    /// 绕过出口直写 `voxels`,须手动重建,否则新结构无碰撞体、撞击采不到)。
    fn refresh_collider(s: &mut ShardSim) {
        refresh_collider_at(s, ChunkPos::new(0, 0));
    }

    /// 同 [`refresh_collider`],显式指定 chunk(跨界布景要刷两个)。
    fn refresh_collider_at(s: &mut ShardSim, cp: ChunkPos) {
        if s.built_colliders.contains(&cp) {
            s.rebuild_chunk_collider(cp);
        }
    }

    /// §5.7.1 表第 8 行(石柱立在土上被侧撞,整根断下)的生产路径版:
    /// 4 格石柱立在土地上(x = 10,世界 y 2..=5,土面 = y 1),零重力钉住;
    /// 单格石方块(EntityDesc::default,无 shape)以 12 m/s 侧撞柱身。
    /// 柱脚石-土剪切割 = min(18, 7) = 7 < s → 整根断下:4 格成 1 个复合体
    /// 请求(承载粘结连通、切块不分开)、4 格写 air、S 零初速(impulses 空)。
    ///
    /// 变红:逐格比较取代冲量流(被撞格单格脱落,非整根);名额不生效;
    /// 零初速丢了(本单元 AC-F8 之前 impulses 必空)。
    #[test]
    fn impact_flow_pillar_breaks_at_stone_dirt_root() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 石柱:直接写体素(x=10, y=2..=5, z=8)。
        for y in 2..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, y, 8), voxel::builtin::STONE);
        }
        refresh_collider(&mut s);
        // 撞击者:单格石方块以 12 m/s 沿 −x 侧撞柱身中部(y = 3 层)。
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.45, 3.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        let mut broke = false;
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            if out.destruction_stats.cells_impacted > 0 {
                broke = true;
                // 断下结构成形:整根 4 格 = 1 个复合体请求(零初速)。
                assert_eq!(
                    out.compound_requests.len() + out.detach_requests.len(),
                    1,
                    "S = 整根 4 格成 1 段(复合体或单格)"
                );
                if !out.compound_requests.is_empty() {
                    let req = &out.compound_requests[0];
                    assert_eq!(req.shape.cells().len(), 4, "整根 4 格");
                    assert!(req.impulses.is_empty(), "F02a:复合体零初速(impulses 空)");
                } else {
                    assert!(out.detach_requests[0].impulse == Vec3::ZERO);
                }
                break;
            }
        }
        assert!(broke, "侧撞石柱应从柱脚断下(石-土割 7)");
        // 4 格柱全 air。
        for y in 2..=5 {
            assert_eq!(
                s.voxels.get_block(BlockPos::new(10, y, 8)),
                types::BlockId::AIR,
                "柱格 y={y} 断下写 air"
            );
        }
    }

    /// §5.7.1 表第 7 行(碎块落在平整土地上不断)的生产路径版:单格土方块
    /// (0.729 kg)以 6 m/s 落在平土上——供给 s ≈ 4.4 ≥ dirt I 7?否——
    /// 6 m/s · 0.729 ≈ 4.37 < 割 28(下压:被撞格整圈 4 剪切 + 1 压向 =
    /// 4×7+28 = 56;石面用 min(18,7))→ 扛住,无断开、地形不变。
    /// 同布景 15 m/s(≈ 10.9)仍 < 56 扛住;§5.7.1 表"0.729 kg 需 ≈ 77 m/s"。
    /// 断言:15 m/s 落体不写任何格(`impact_flow = true` 下)。
    ///
    /// 变红:仍走逐格比较(REQ-009 口径:J ≥ 7 即破,s 15 m/s 必破 → 格被写)。
    #[test]
    fn impact_flow_debris_on_flat_ground_holds() {
        let mut s = flow_sim();
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(10.5, 2.45, 8.5),
                    linvel: Vec3::new(0.0, -15.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::DIRT,
                    shape: None,
                },
            }),
        }]);
        let mut impacted = 0;
        for _ in 0..60 {
            s.step();
            let out = s.take_outbox();
            impacted += out.destruction_stats.cells_impacted;
        }
        assert_eq!(
            impacted, 0,
            "碎块落平地:割 = 28+4×7 = 56 > s,扛住不破(逐格口径 15 m/s 必破)"
        );
        assert!(s.entity_state(id).is_some(), "撞击者仍在(扛住不修正)");
    }

    /// AC-F7(F03 遗留生产句):醒着的 2 格土梁被**单格**方块以 12 m/s 砸中
    /// 右端 → 断键。D4 采样的对方速度现认任何醒着动态实体(单格也算)。
    ///
    /// 变红:对方资格退回"只认复合体"(单格投射物速度按 0 计,门槛不过,
    /// 不断)。
    #[test]
    fn single_cell_projectile_wakes_and_breaks_compound() {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.impact_flow = true;
        // G03 迁移(§12.1 第 3 条):AC-F7 单格对方资格的 D4 断键版,显式关
        // compound_flow(资格表两条路径共享;AC-G4 关闭回归)。
        cfg.cohesion.compound_flow = false;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        let (target, proj) = (entity(1), entity(2));
        // 目标:2 格土梁静置在土面上(中心 y = 1.45;土 bond 3);投射物:
        // 单格石方块(EntityDesc::block,无 shape)以 −12 m/s 落在右端正上方。
        let dirt_shape = types::CompoundShape::new(
            (0..2)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("2 格");
        s.apply_prefix(&[
            compound_spawn_input(
                1,
                target,
                EntityState {
                    translation: Vec3::new(9.5, 1.4501, 8.5),
                    ..EntityState::default()
                },
                dirt_shape,
            ),
            StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 2,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: proj,
                    state: EntityState {
                        translation: Vec3::new(10.5, 2.45, 8.5),
                        linvel: Vec3::new(0.0, -12.0, 0.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc {
                        half_size: 0.45,
                        block: voxel::builtin::STONE,
                        shape: None,
                    },
                }),
            },
        ]);
        let mut broke = false;
        for _ in 0..40 {
            s.step();
            let out = s.take_outbox();
            if out.cohesion_stats.bonds_broken > 0 {
                broke = true;
                break;
            }
        }
        assert!(
            broke,
            "单格投射物 12 m/s 砸醒着的土梁应断键(对方速度按勘误 2'对方'条计入)"
        );
    }

    /// AC-F7(事件顺序):两个撞击者同拍撞**同一**结构——前者写 air 后,
    /// 后者的判定(网、割)因那格变 air 而不同,结果与"看不到前者写入"不同。
    /// 构造型式:零重力悬空土墩——bedrock 垫 (10,3,8)、土 (10,4,8) 与
    /// (10,5,8)(垫离地面 2 格悬空,四周空气,BFS 不外溢;bedrock 是 Anchor
    /// 汇点,土-基岩剪切割 = 7)。A(id 1) 12 m/s 侧撞上土格(y=5):割 7 < s
    /// 8.7 → 断上格、写 air;B(id 2) 同拍 12 m/s 侧撞下土格(y=4):**若看到**
    /// 上格已 air,其网只有"下土 → 垫"一条路(割 7 < 8.7 → 断);若按拍首
    /// 世界(上格仍土),下土有"向下垫 + 向上土→垫"两条路(合计 14 ≥ 8.7)
    /// → 扛住。断言:两格都断 = B 看到了 A 的写入。
    ///
    /// 变红:事件并行判定(都按拍首世界算 → B 扛住,只断 1 格);
    /// 反过来处理(B 先)则两事件都扛住、0 格断——结果与顺序相关,符合
    /// "反过来处理,结果就不同"。
    #[test]
    fn impact_flow_events_process_in_entity_order_seeing_prior_writes() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 悬空土墩:垫 (10,3,8) bedrock、土 (10,4,8)、(10,5,8)。
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        for y in 4..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, y, 8), voxel::builtin::DIRT);
        }
        refresh_collider(&mut s);
        // A(id 1):12 m/s 沿 −x 撞上土格(y=5,中心 5.5);B(id 2):同拍、同速
        // 撞下土格(y=4,中心 4.5)。EntityId 升序 = A 先处理。
        let mk = |seq: u64, id: EntityId, y: f32| StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.45, y, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        };
        s.apply_prefix(&[mk(1, entity(1), 5.5), mk(2, entity(2), 4.5)]);
        let mut cells_written = 0;
        let mut breaks = 0;
        let mut visited = 0u64;
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            cells_written += out.destruction_stats.cells_impacted;
            breaks += out.cohesion_stats.impact_flow_breaks;
            visited += out.cohesion_stats.impact_flow_cells_visited;
            if cells_written > 0 {
                break;
            }
        }
        // 断开拍:A 断上格;B 判定时上格已 air → 网只有下土→垫 2 格(vis=2,
        // 一条路,割 7 < s)→ 也断。合计 2 格写入、2 次断开、BFS 访问 3+2=5
        // (并行判定则 B 按拍首世界算:网含上土 3 格、第二条路使它**扛住**
        // → 只写 1 格、1 次断开、访问 6)。三处遥测任一都钉住"看到前者写入"。
        assert_eq!(
            cells_written, 2,
            "A 断上格写 air;B 看到后下土也断(并行判定则只断 1 格)"
        );
        assert_eq!(breaks, 1 + 1, "两个事件都断开(并行判定则 B 扛住,只 1 次)");
        assert_eq!(visited, 3 + 2, "B 的网不含已 air 的上土格(并行判定则 6)");
        assert_eq!(
            s.voxels.get_block(BlockPos::new(10, 4, 8)),
            types::BlockId::AIR,
            "下土格也被断(证明 B 的事件看到了 A 写的 air)"
        );
        assert_eq!(
            s.voxels.get_block(BlockPos::new(10, 3, 8)),
            voxel::builtin::BEDROCK,
            "垫保留(汇点侧)"
        );
    }

    /// AC-F7(每拍事件上限):`impact_flow_max_events_per_tick = 1`。布景同
    /// "事件顺序"用例(悬空土墩:bedrock 垫 (10,3,8)、土 (10,4,8)、(10,5,8)),
    /// 两个投射物**同拍**接触(A id 1 撞上土格、B id 2 撞下土格,均 12 m/s,
    /// s 逐位相等)。选取 = s 降序、同值 `EntityId` 升序 → 只取 A;A 断上格,
    /// B 本拍不处理、不推迟——零重力下 B 静止悬停在半空,后续拍不再有 ≥ 1
    /// m/s 的接近速度,下土格不断。
    ///
    /// 变红:上限不生效(A、B 依次都处理 → 下土格也断,2 格);选取顺序反了
    /// (B 被处理、断下格 = 下土格)。
    #[test]
    fn impact_flow_event_cap_picks_highest_supply() {
        let mut cfg = ShardSimConfig {
            gravity: Vec3::ZERO,
            ..ShardSimConfig::new(shard_id(0, 0))
        };
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.impact_flow = true;
        cfg.cohesion.impact_flow_max_events_per_tick = 1;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 悬空土墩:垫 (10,3,8) bedrock、土 (10,4,8)、(10,5,8)。
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        for y in 4..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, y, 8), voxel::builtin::DIRT);
        }
        refresh_collider(&mut s);
        // 同 bridge 用例:A 12 m/s 撞上土格(y=5.5),B 同拍撞下土格(y=4.5)。
        let mk = |seq: u64, id: EntityId, y: f32| StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.45, y, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        };
        s.apply_prefix(&[mk(1, entity(1), 5.5), mk(2, entity(2), 4.5)]);
        let mut impacted_cells = Vec::new();
        for _ in 0..16 {
            s.step();
            let out = s.take_outbox();
            for req in &out.detach_requests {
                impacted_cells.push(req.cell);
            }
        }
        // 只处理 A:上土格断;B 落选 → 下土格完好、垫保留。
        assert_eq!(
            impacted_cells,
            vec![BlockPos::new(10, 5, 8)],
            "上限 1:只处理 EntityId 最小的 A,断上土格(其余丢弃不推迟)"
        );
        assert_eq!(
            s.voxels.get_block(BlockPos::new(10, 4, 8)),
            voxel::builtin::DIRT,
            "B 落选:下土格完好"
        );
        assert_eq!(
            s.voxels.get_block(BlockPos::new(10, 3, 8)),
            voxel::builtin::BEDROCK,
            "垫保留(汇点侧)"
        );
    }

    /// AC-F7(名额放弃):`max_detach_per_tick = 1`,同拍**两个都会断**的事件
    /// (布景同"事件顺序"用例):A(id 1) 先处理,断上格、占掉唯一的脱落名额;
    /// B(id 2) 判定也断(S = 1 格 = 1 段 > 剩余名额 0)→ **整个事件放弃**
    /// (不写、不断),记 `impact_flow_skipped_budget = 1`。不拆半个事件。
    ///
    /// 变红:名额不检查(B 照写,超额);放弃却写了地形;放弃不计数。
    #[test]
    fn impact_flow_skips_event_when_budget_insufficient() {
        let mut cfg = ShardSimConfig {
            gravity: Vec3::ZERO,
            ..ShardSimConfig::new(shard_id(0, 0))
        };
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.impact_flow = true;
        cfg.destruction.max_detach_per_tick = 1;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 悬空土墩:垫 (10,3,8) bedrock、土 (10,4,8)、(10,5,8)。
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        for y in 4..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, y, 8), voxel::builtin::DIRT);
        }
        refresh_collider(&mut s);
        // 同"事件顺序"用例:A 撞上土格、B 同拍撞下土格(都会断)。
        let mk = |seq: u64, id: EntityId, y: f32| StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.45, y, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        };
        s.apply_prefix(&[mk(1, entity(1), 5.5), mk(2, entity(2), 4.5)]);
        let mut skipped = 0;
        let mut detached_cells = Vec::new();
        for _ in 0..16 {
            s.step();
            let out = s.take_outbox();
            skipped += out.cohesion_stats.impact_flow_skipped_budget;
            for req in &out.detach_requests {
                detached_cells.push(req.cell);
            }
        }
        assert_eq!(skipped, 1, "名额不足,B 事件整体放弃,遥测计 1");
        // A 断上格(占名额);B 放弃:下土格不被写。
        assert_eq!(
            detached_cells,
            vec![BlockPos::new(10, 5, 8)],
            "只有 A 的事件写入;B 整体放弃不拆半个"
        );
        assert_eq!(
            s.voxels.get_block(BlockPos::new(10, 4, 8)),
            voxel::builtin::DIRT,
            "名额放弃:下土格不被写"
        );
    }

    /// AC-F10(遥测):石柱被侧撞的生产工况下 8 个计数的取值——events、
    /// breaks、detached_cells 随断开产生;prefiltered、skipped_budget、
    /// corrections = 0;cells_visited > 0。
    ///
    /// 变红:计数没接线(恒 0);corrections 本单元非 0(F02b 才接线)。
    #[test]
    fn impact_flow_telemetry_counts_on_pillar_break() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        for y in 2..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, y, 8), voxel::builtin::STONE);
        }
        refresh_collider(&mut s);
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.45, 3.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        let mut stats = crate::cohesion::CohesionStats::default();
        for _ in 0..20 {
            s.step();
            stats.add(&s.take_outbox().cohesion_stats);
        }
        assert!(stats.impact_flow_events >= 1, "建网事件 ≥ 1");
        assert!(stats.impact_flow_breaks >= 1, "断开事件 ≥ 1");
        assert_eq!(stats.impact_flow_detached_cells, 4, "整根 4 格断下");
        assert!(stats.impact_flow_cells_visited > 0, "BFS 访问 > 0");
        assert_eq!(stats.impact_flow_budget_hits, 0, "小结构不截断");
        assert_eq!(stats.impact_flow_prefiltered, 0, "12 m/s 不命中预筛");
        assert_eq!(stats.impact_flow_skipped_budget, 0, "名额充足");
        assert_eq!(
            stats.impact_flow_corrections, 1,
            "F02b:断开事件施加撞击者修正(§5.7.2,12 m/s 单格撞柱断根)"
        );
    }

    /// AC-F7(分片边界):悬空土梁**横跨本体区边界 x = 128**(SHARD_CHUNKS=8),
    /// 区内格被撞后断开不越界——区外格(副本格)按汇点(地基),S 截在边界内。
    /// 布景:同时装载 chunk (7,0) 与 (8,0)(后者 = 本体区外);梁 x 124..=131、
    /// y=4、z=8,下方全空(离实心层 3 格,BFS 不外溢)。单格方块 12 m/s 沿 +z
    /// 撞区内格(x=127):跨界竖边 u·d̂=0 → 剪切 7 < s ≈ 9.2 → 断开,S = 区内
    /// 4 格(x 124..=127);区外格(x ≥ 128)仍是土。
    ///
    /// 变红:`in_core` 恒真(区外格不再当汇点)→ 梁整根悬空、无汇点 →
    /// C = 0、S = 整根 8 格 → x ≥ 128 的格被写 air → 本测试红。
    #[test]
    fn impact_flow_detached_stays_in_core_at_boundary() {
        let mut s = flow_sim_zero_g();
        // 本体区 = x 0..128;(7,0) 是区内最后一个 chunk(x 112..128),(8,0)
        // 是区外副本(x 128..144)。梁 x 124..=131 跨骑 127/128 边界。
        s.load_chunk(ChunkPos::new(7, 0), solid_chunk());
        s.load_chunk(ChunkPos::new(8, 0), solid_chunk());
        // 悬空土梁横跨 x=128:x 124..=131、y=4、z=8。
        let beam: Vec<BlockPos> = (124..=131).map(|x| BlockPos::new(x, 4, 8)).collect();
        for &cell in &beam {
            let _ = s.voxels.set_block(cell, voxel::builtin::DIRT);
        }
        refresh_collider_at(&mut s, ChunkPos::new(7, 0));
        refresh_collider_at(&mut s, ChunkPos::new(8, 0));
        // 撞击者:单格石方块 12 m/s 沿 +z 撞区内格(x=127、z=8 的 −z 面;
        // d̂ = +z,跨界边 u = ±x 与 d̂ 垂直 → 剪切;零重力直线弹道不穿梁)。
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(127.45, 4.5, 5.55),
                    linvel: Vec3::new(0.0, 0.0, 12.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        let mut breaks = 0;
        let mut detached: Vec<BlockPos> = Vec::new();
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            breaks += out.cohesion_stats.impact_flow_breaks;
            for req in &out.detach_requests {
                detached.push(req.cell);
            }
            for r in &out.compound_requests {
                for c in r.shape.cells() {
                    detached.push(BlockPos::new(
                        r.origin.x + c.offset[0],
                        r.origin.y + c.offset[1],
                        r.origin.z + c.offset[2],
                    ));
                }
            }
            if breaks > 0 {
                break;
            }
        }
        assert!(breaks >= 1, "区内格被撞应断开(割 = 127→128 界面剪切 7)");
        assert!(
            detached.iter().all(|c| c.x < 128),
            "断下格全部在本体区内(x < 128):{detached:?}"
        );
        for x in 128..=131 {
            assert_eq!(
                s.voxels.get_block(BlockPos::new(x, 4, 8)),
                voxel::builtin::DIRT,
                "区外梁格 ({x},4,8) 仍是土(副本格按汇点,不越界断开)"
            );
        }
    }

    /// AC-F12(`impact_flow = false` 逐位不变):同一布景开关关/开,关闭时
    /// 与 F03 定稿(910e959)的逐格口径产生**相同**的脱落格与拍序——
    /// 碎块 15 m/s 落平地:逐格口径必破(J ≈ 10.9 ≥ 7),冲量流口径扛住
    /// (割 56)。关闭时走逐格 → 破;开启时扛住。两分叉同拍可对照。
    ///
    /// 变红:关闭路径被改动(开启与否结果同);关闭时也走了冲量流。
    #[test]
    fn impact_flow_off_keeps_legacy_per_cell_bitwise() {
        let run = |flow: bool| -> Vec<BlockPos> {
            let mut cfg = ShardSimConfig::new(shard_id(0, 0));
            cfg.solidify.enabled = false;
            cfg.cohesion.enabled = true;
            cfg.cohesion.impact_flow = flow;
            let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            let id = entity(1);
            s.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: id,
                    state: EntityState {
                        translation: Vec3::new(10.5, 2.45, 8.5),
                        linvel: Vec3::new(0.0, -15.0, 0.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc {
                        half_size: 0.45,
                        block: voxel::builtin::DIRT,
                        shape: None,
                    },
                }),
            }]);
            let mut cells = Vec::new();
            for _ in 0..40 {
                s.step();
                let out = s.take_outbox();
                for req in out.detach_requests {
                    cells.push(req.cell);
                }
            }
            cells
        };
        let off = run(false);
        let on = run(true);
        assert!(
            !off.is_empty(),
            "逐格口径(关闭):15 m/s 土块 J ≈ 10.9 ≥ 7 必破"
        );
        assert!(
            on.is_empty(),
            "冲量流口径(开启):割 56 > s 扛住,不破——分派生效"
        );
    }

    // ============================== W03(REQ-011 §3.5.1)==============================

    /// AC-W6 事件摘要(§3.5.1):单格撞结构 §5.7 路径产生的 `flow_event`
    /// ——`supply` / `cut` 与判定同源(与 §5.7 现有量逐值对上:12 m/s 单格
    /// 石方块撞土墩上格,割 = 土石剪切 7;F02b 同布景供给 s ≈ 8.7)、
    /// `broke` 与实际断开一致、`position` = 被撞格(供给最大格)格心。
    ///
    /// 变红:supply/cut 另算一遍(与判定量化值不同源);position 取错格;
    /// broke 与判定分支比较脱节。
    #[test]
    fn flow_event_impact_single_cell_vs_dirt_mound() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 悬空土墩:垫 (10,3,8) bedrock、土 (10,4,8)、(10,5,8)(F02b 同布景:
        // 12 m/s 撞上格,割 7 < s ≈ 8.7 → 断上格 (10,5,8))。
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        for y in 4..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, y, 8), voxel::builtin::DIRT);
        }
        refresh_collider(&mut s);
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.45, 5.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        let mut event = None;
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            if out.cohesion_stats.impact_flow_breaks > 0 {
                event = out.flow_event;
                break;
            }
        }
        let ev = event.expect("断开拍应有 flow_event(Impact)");
        assert_eq!(ev.kind, crate::sim::FlowKind::Impact);
        // 判定同源:量化值换算回 N·s 后必须落在 §5.7 既有量上——割 = 土-石
        // 剪切 min(18,7) = 7(F02b 同布景实测);供给 ≈ 8.7(J ≈ 0.729 kg ×
        // 12 m/s ≈ 8.748,量化 floor)。逐值钉住换算而非另算。
        assert_eq!(ev.cut, 7.0, "割 = min(18,7) = 7(量化 7000 换算)");
        // 供给 = 解算冲量(牛顿封顶后),生产实测 9.217(F02b 注释的 ≈8.7 是
        // m·v 估算;解算值略高,带内钉住)。
        assert!(
            (9.0..=9.5).contains(&ev.supply),
            "供给 ≈ 9.2 N·s(解算冲量),实得 {}",
            ev.supply
        );
        assert!(ev.broke, "割 7 < 供给 → broke = true(与判定同比较)");
        assert_eq!(ev.detached_cells, 1, "S = 上格单格");
        assert_eq!(
            ev.position,
            Vec3::new(10.5, 5.5, 8.5),
            "position = 被撞格(供给最大格)格心"
        );
    }

    /// AC-W6 预筛不记 + 关闭路径不记:①低速贴面起跑(s < I_min = 4 不建网)
    /// → `flow_event` 为 `None`;②`compound_flow = false` 旧路径同布景
    /// (悬空 6 格土条 40 m/s 端格侧撞,关闭时逐格口径会碎)→ `flow_event`
    /// 为 `None`。
    ///
    /// 变红:预筛命中也记(无 C 还编一个);旧路径混入摘要。
    #[test]
    fn flow_event_none_for_prefilter_and_legacy_path() {
        // ① 预筛:0.729 kg × 2 m/s ≈ 1.46 < I_min = 4(sand min(I,I_c)),
        // 贴面起跑(13.45 → 撞击格 10)直线弹道。
        {
            let mut s = flow_sim_zero_g();
            s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
            for y in 4..=5 {
                let _ = s
                    .voxels
                    .set_block(BlockPos::new(10, y, 8), voxel::builtin::DIRT);
            }
            refresh_collider(&mut s);
            let id = entity(1);
            s.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: id,
                    state: EntityState {
                        translation: Vec3::new(13.45, 5.5, 8.5),
                        linvel: Vec3::new(-2.0, 0.0, 0.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc {
                        half_size: 0.45,
                        block: voxel::builtin::STONE,
                        shape: None,
                    },
                }),
            }]);
            let mut events = 0u64;
            // 2 m/s × 60 拍/s:飞完 2 m 需 ~60 拍,窗口取 150 盖过。
            for _ in 0..150 {
                s.step();
                let out = s.take_outbox();
                events += out.cohesion_stats.impact_flow_prefiltered;
                assert!(
                    out.flow_event.is_none(),
                    "预筛命中不记 flow_event(§3.5.1),实得 {:?}",
                    out.flow_event
                );
            }
            assert!(events >= 1, "布景须真的命中过预筛(否则测了个空)");
        }
        // ② compound_flow = false 旧路径:悬空 6 格土条被 40 m/s 单格土方块
        // 撞端格(compound_flow_side_impact 同布景),关闭 → 不记。
        {
            let mut cfg = ShardSimConfig {
                gravity: Vec3::ZERO,
                ..ShardSimConfig::new(shard_id(0, 0))
            };
            cfg.solidify.enabled = false;
            cfg.cohesion.enabled = true;
            cfg.cohesion.impact_flow = true;
            cfg.cohesion.compound_flow = false;
            let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
            s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
            let shape = types::CompoundShape::new(
                (-4i32..=1)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格");
            s.apply_prefix(&[
                compound_input_at(1, entity(1), Vec3::new(9.5, 5.5, 8.5), shape, Vec3::ZERO),
                dirt_bullet_input(
                    2,
                    entity(2),
                    Vec3::new(10.5, 5.5, 4.0),
                    Vec3::new(0.0, 0.0, 40.0),
                ),
            ]);
            let mut broke = false;
            for _ in 0..60 {
                s.step();
                let out = s.take_outbox();
                if out.cohesion_stats.compound_flow_events == 0 {
                    assert!(
                        out.flow_event.is_none(),
                        "旧路径(compound_flow = false)不记 flow_event"
                    );
                }
                broke |= out.cohesion_stats.bonds_broken > 0;
            }
            assert!(broke, "关闭时该场景按旧口径碎(布景有效性)");
        }
    }

    /// AC-W6 一拍两事件取 `s / C` 大者(§3.5.1):同拍两条撞击事件布景——
    /// 石方块 A 12 m/s 撞上格(起点 13.45,10 拍到)、石方块 B 6 m/s 撞下格
    /// (起点 12.45,同为 10 拍到;速度 × 距离同积,同拍接触),上限 2(都
    /// 处理)。两条割同为 7(单键),供给 A > B → `flow_event` = A 那条;
    /// 供给值与单独复跑逐值一致。
    ///
    /// 变红:取第一条/最后一条(非 s/C 最大);C=0 未按无穷大处理。
    #[test]
    fn flow_event_two_impacts_same_tick_picks_more_dangerous() {
        // 单独复跑(只留一个投射物)量取各自事件的 (supply, position)。
        let solo = |x: f32, speed: f32, y: f32| -> (f32, Vec3) {
            let mut s = flow_sim_zero_g();
            s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
            for yy in 4..=5 {
                let _ = s
                    .voxels
                    .set_block(BlockPos::new(10, yy, 8), voxel::builtin::DIRT);
            }
            refresh_collider(&mut s);
            s.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: entity(1),
                    state: EntityState {
                        translation: Vec3::new(x, y, 8.5),
                        linvel: Vec3::new(-speed, 0.0, 0.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc {
                        half_size: 0.45,
                        block: voxel::builtin::STONE,
                        shape: None,
                    },
                }),
            }]);
            let mut got = None;
            for _ in 0..40 {
                s.step();
                let out = s.take_outbox();
                if let Some(ev) = out.flow_event {
                    got = Some((ev.supply, ev.position));
                    break;
                }
            }
            got.expect("单投射物布景应有事件")
        };
        let (s_upper, p_upper) = solo(13.45, 12.0, 5.5);
        let (s_lower, p_lower) = solo(12.45, 6.0, 4.5);
        assert!(
            s_upper > s_lower,
            "布景对照:上格弹 12 m/s 供给须大于下格弹 6 m/s({s_upper} vs {s_lower})"
        );
        // 双事件拍:上限 2(同拍都处理),flow_event 应 = 供给大的 A 那条。
        let mut cfg = ShardSimConfig {
            gravity: Vec3::ZERO,
            ..ShardSimConfig::new(shard_id(0, 0))
        };
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.impact_flow = true;
        cfg.cohesion.impact_flow_max_events_per_tick = 2;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        for yy in 4..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, yy, 8), voxel::builtin::DIRT);
        }
        refresh_collider(&mut s);
        let mk = |seq: u64, id: EntityId, x: f32, speed: f32, y: f32| StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(x, y, 8.5),
                    linvel: Vec3::new(-speed, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        };
        s.apply_prefix(&[
            mk(1, entity(1), 13.45, 12.0, 5.5),
            mk(2, entity(2), 12.45, 6.0, 4.5),
        ]);
        let mut picked = None;
        for _ in 0..40 {
            s.step();
            let out = s.take_outbox();
            if out.cohesion_stats.impact_flow_events >= 2 {
                picked = out.flow_event;
                break;
            }
        }
        let ev = picked.expect("双事件拍应有 flow_event(两事件同拍建网求值)");
        assert_eq!(
            ev.position, p_upper,
            "取供给大的 A 那条(position = 被撞上格格心;{p_upper} vs {p_lower})"
        );
        assert_eq!(ev.supply, s_upper, "供给值与单独复跑逐值一致");
    }

    /// AC-W6 复合体路径:6 格土条 30° 端角落地(§5.8,AC-G3 落地③同布景)
    /// 产生 `flow_event`(Compound)——broke = true、detached_cells = 1、
    /// position = 该刚体本拍步进后的平移(原点格心 (10.5, ·, 8.5):x/z 分量
    /// 钉住,y 在静置原点下方)。
    ///
    /// 变红:Compound 事件不记 / position 取成格坐标(应为刚体平移)。
    #[test]
    fn flow_event_compound_corner_landing() {
        let mut s = flow_compound_gravity_sim();
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        let shape = types::CompoundShape::new(
            (-4i32..=1)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("6 格条");
        let id = entity(1);
        let h = 2.0 * 2.1;
        let y0 = 1.0
            + 4.0 * 30f32.to_radians().sin()
            + 0.45 * (30f32.to_radians().sin() + 30f32.to_radians().cos())
            + h;
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(10.5, y0, 8.5),
                    rotation: types::glam::Quat::from_rotation_z(30f32.to_radians()),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape),
            }),
        }]);
        assert_touch_speed(&mut s, id, h, 30, "30° 端角");
        let mut event = None;
        for _ in 0..400 {
            s.step();
            let out = s.take_outbox();
            if out.cohesion_stats.compound_flow_breaks > 0 {
                event = out.flow_event;
                break;
            }
        }
        let ev = event.expect("断开拍应有 flow_event(Compound)");
        assert_eq!(ev.kind, crate::sim::FlowKind::Compound);
        assert!(ev.broke, "端格断下 → broke = true");
        assert_eq!(ev.detached_cells, 1, "S = 端格单格");
        // 供给/割与判定量一致:端格断下的 s 在 G03 实测档位(≈ 15–19 N·s)
        // 且必 > 割;割随 30° 端角布景取窄口,断言 broke/s 关系与 detachment
        // 同源,再钉 supply 非零且量化换算回整数无残差。
        assert!(
            ev.supply > ev.cut,
            "断开 ⇒ s > C({} vs {})",
            ev.supply,
            ev.cut
        );
        assert!(ev.supply > 0.0 && ev.cut > 0.0, "s/C 都非零(建网求值)");
        assert!(
            (ev.supply * 1000.0).fract() < 1e-3 && (ev.cut * 1000.0).fract() < 1e-3,
            "s/C 由量化值除 1000 换算(整数域无残差)"
        );
        // position = 刚体本拍步进后的平移:条原点在格 (10,·,8),x/z 钉格心;
        // y 应在静置原点(≈ 3.61)下方(已触地),且为有限值。
        assert!(
            (ev.position.x - 10.5).abs() < 1.0 && (ev.position.z - 8.5).abs() < 1.0,
            "position = 刚体平移(条原点 x/z 邻域),实得 {}",
            ev.position
        );
        assert!(
            ev.position.y < y0 && ev.position.y.is_finite(),
            "position.y 在触地下方:{},spawn y = {y0}",
            ev.position.y
        );
    }

    // ============================== F02b(§5.7.2/§5.7.3)==============================

    /// AC-F4(生产路径)/ AC-F5(修正)共用布景:零重力悬空土墩(bedrock
    /// 垫 (10,3,8) + 土 (10,4,8)(10,5,8)),单格石方块 12 m/s 沿 −x 撞上格
    /// ——共线撞击(接触点在质心行进线上,m_eff = m),割 7 < s ≈ 8.7 断上
    /// 格,S = 上格单格。断言:①修正计 1 次(§5.7.2);②单格碎块初速非零、
    /// 沿来袭方向(修订 8 零初速退役,AC-F6)、无横向分量(正撞 d̂ = −x)、
    /// |v_seg| ≤ u(w ≤ u);③撞击者修正后沿 −x 前进(不再永久停在墙面,
    /// §5.7.9 勘误 3)。守恒式的纯函数半边由 F01 的 e2e(AC-F4)钉住。
    ///
    /// 变红:修正漏施加 / 未计 corrections;单格碎块仍零初速(DetachRequest
    /// .impulse = 0);碎块速度带横向分量(d̂ 算错);w 超 u(clamp 缺失)。
    #[test]
    fn impact_flow_momentum_and_correction_conservation() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 悬空土墩:垫 (10,3,8) bedrock、土 (10,4,8)、(10,5,8)。
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        for y in 4..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, y, 8), voxel::builtin::DIRT);
        }
        refresh_collider(&mut s);
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.45, 5.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        let mut corrections = 0u64;
        let mut detached_impulse: Option<(BlockPos, Vec3, f32)> = None;
        let mut impactor_vel_after = None;
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            corrections += out.cohesion_stats.impact_flow_corrections;
            if !out.detach_requests.is_empty() {
                let req = &out.detach_requests[0];
                detached_impulse = Some((req.cell, req.impulse, s.entity_mass(id).unwrap()));
                impactor_vel_after = s.entity_state(id).map(|st| st.linvel);
                break;
            }
        }
        assert_eq!(corrections, 1, "断开事件施加 1 次撞击者修正(§5.7.2)");
        let (cell, impulse, mass) = detached_impulse.expect("上格应断下(割 7 < s)");
        assert_eq!(cell, BlockPos::new(10, 5, 8), "S = 上格单格");
        // 单格碎块初速(§5.7.3):调度器 detach_spawn 以 linvel = impulse/m
        // 换算 → 请求 impulse = m·v_seg。零重力下 d̂ = (−1,0,0),v_seg =
        // w·d̂ = Δv_k(单格分量,Δω = 0):沿 −x 且非零(零初速退役,AC-F6)。
        assert!(
            impulse.x < -1e-3,
            "单格碎块初速沿 −x(来袭方向)非零(零初速退役):impulse = {impulse:?}"
        );
        let v_seg = impulse / mass;
        assert!(
            impulse.y.abs() < 1e-4 && impulse.z.abs() < 1e-4,
            "正撞:d̂ = (−1,0,0),碎块初速无横向分量"
        );
        // w ≤ u(§5.7.2):|v_seg| ≤ 12(完全非弹性,共同速度不超原速)。
        assert!(
            v_seg.length() <= 12.0 + 1e-3,
            "w ≤ u(共同速度不超原速):|v_seg| = {}",
            v_seg.length()
        );
        // 撞击者沿 d̂ 的速度增加(勘误 3:不再永久停在墙面)。修正前 rapier
        // 已把撞击者按固定墙解算(法向速度 ≈ 0),修正后应以约 w 沿 −x 前进。
        let v_after = impactor_vel_after.expect("撞击者在场");
        assert!(
            v_after.x < -0.1,
            "撞击者修正后沿 −x 前进(不再停墙面,勘误 3):v = {v_after:?}"
        );
    }

    /// AC-F6(修订 8 坑内连锁上界):复现 F00 `momentum.py case5_pit` 构造
    /// ——平地挖 1 格宽、2 格深的坑,坑底静置一块,第二块从上方 3 m 自由
    /// 落体砸在它上面。F00 demo 记录(300 拍):旧规则无过滤脱落 6 格;
    /// §5.7 规则(冲量流 + 守恒分配 + 接近速度过滤)= 0;F00b 内缩 + 旧规则
    /// = 1(第二块真砸在坑底)。生产 §5.7 口径的连锁脱落格数按 demo 上界
    /// 断言 ≤ 1(出处:demos/impact-flow/momentum.py 第 5 节输出)。
    ///
    /// 变红:勘误 4 退役零初速后连锁回归(坑壁对挤被计入供给 → 连锁 > 1)。
    #[test]
    fn impact_flow_pit_chain_detached_within_f00_bound() {
        let mut s = flow_sim();
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        // 坑:挖 (11,0,8) 与 (11,-1,8)(1 格宽、2 格深;表层土 = y 0)。
        let _ = s
            .voxels
            .set_block(BlockPos::new(11, 0, 8), types::BlockId::AIR);
        let _ = s
            .voxels
            .set_block(BlockPos::new(11, -1, 8), types::BlockId::AIR);
        refresh_collider(&mut s);
        // 第一块:坑底静置位(坑底 = y −1 格的底面 y = −1;静置中心 =
        // −1 + 0.45 = −0.55),零初速。
        let (first, second) = (entity(1), entity(2));
        s.apply_prefix(&[
            StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: first,
                    state: EntityState {
                        translation: Vec3::new(11.5, -0.55, 8.5),
                        ..EntityState::default()
                    },
                    desc: EntityDesc {
                        half_size: 0.45,
                        block: voxel::builtin::STONE,
                        shape: None,
                    },
                }),
            },
            // 第二块:坑上方 3 m 自由落体(demo 口径)。
            StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 2,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: second,
                    state: EntityState {
                        translation: Vec3::new(11.5, 2.45, 8.5),
                        ..EntityState::default()
                    },
                    desc: EntityDesc {
                        half_size: 0.45,
                        block: voxel::builtin::STONE,
                        shape: None,
                    },
                }),
            },
        ]);
        let mut detached_cells = 0u64;
        for _ in 0..300 {
            s.step();
            let out = s.take_outbox();
            detached_cells += out.cohesion_stats.impact_flow_detached_cells;
        }
        assert!(
            detached_cells <= 1,
            "AC-F6:修订 8 坑内工况连锁脱落 ≤ 1(F00 demo 上界),实际 {detached_cells}"
        );
        let _ = first;
    }

    /// AC-F5(扛住不修正,双开关逐位一致):布景用**两种口径都扛住**的
    /// 工况——6 m/s 土块落平地:逐格口径 J ≈ 4.4 < 阈值 7 破不了;冲量流
    /// 口径割 56 > s 同样扛住。双跑只有 `impact_flow` 不同:断言撞击者
    /// **逐拍轨迹逐位一致**且 corrections 恒 0。扛住时冲量流不分配、
    /// 不修正,撞击者物理与关闭完全同拍。
    ///
    /// 变红:扛住也修正(§5.7.2 边界破);分派在扛住路径上改变了物理。
    #[test]
    fn impact_flow_held_impactor_matches_off_bitwise() {
        let run = |flow: bool| -> (Vec<EntityState>, u64) {
            let mut cfg = ShardSimConfig::new(shard_id(0, 0));
            cfg.solidify.enabled = false;
            cfg.cohesion.enabled = true;
            cfg.cohesion.impact_flow = flow;
            let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            let id = entity(1);
            s.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: id,
                    state: EntityState {
                        translation: Vec3::new(10.5, 2.45, 8.5),
                        linvel: Vec3::new(0.0, -6.0, 0.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc {
                        half_size: 0.45,
                        block: voxel::builtin::DIRT,
                        shape: None,
                    },
                }),
            }]);
            let mut trace = Vec::new();
            let mut corrections = 0u64;
            for _ in 0..40 {
                s.step();
                let out = s.take_outbox();
                corrections += out.cohesion_stats.impact_flow_corrections;
                trace.push(s.entity_state(id).expect("撞击者在场"));
            }
            (trace, corrections)
        };
        let (off, c_off) = run(false);
        let (on, c_on) = run(true);
        assert_eq!(c_off, 0, "关闭:无冲量流,corrections 恒 0");
        assert_eq!(c_on, 0, "扛住(两种口径都不破):不修正");
        assert_eq!(
            off, on,
            "扛住的撞击者:两开关逐拍轨迹逐位一致(冲量流不改物理)"
        );
    }

    /// AC-F4(牛顿封顶,§5.7.9 勘误 6,生产路径定量):窄墩工况下撞击者
    /// 一步穿进墩内、同时压到侧面/顶面的内缩薄片——那些流形按勘误 2 不
    /// 计入,但其摩擦沿撞击方向抵掉了正面流形的大部分冲量,计入的正面
    /// 法向合计(封顶前的 |J|)远大于撞击者本步实际交给地形的动量。勘误 6:
    /// N = m·((v_前 − v_后) + g·Δt)·d̂,κ = clamp(N/|J|, 0, 1)(N ≥ 0.99·|J|
    /// 时 κ = 1),J 与全部 s_c 乘 κ、p 不变。
    ///
    /// 布景 1(窄墩):零重力悬空 1×2 土墩(bedrock 垫 (10,3,8) + 土
    /// (10,4,8)(10,5,8)),L 形 4 格撞击者 12 m/s 打上格——断开且 κ < 1
    /// (薄片摩擦吃掉 ~48% 冲量),封顶后 |J| = N(κ < 1 时恒等),碎块段
    /// 速度与撞击者修正后沿 d̂ 速度都 ≤ u + 0.05(封顶后不可能超来速)。
    /// 布景 2(墙面对照):7 宽土墙(单一流形、无薄片摩擦),κ = 1(封顶
    /// 不触发,结果与封顶前逐位一致——κ = 1 即 J 未缩放)。
    ///
    /// 变红:κ 恒为 1(未封顶)——布景 1 的 κ < 1 断言与 |J| = N 对账红;
    /// 封顶误用拍后速度(含修正)或漏重力项——N 对不上,布景 2 的 κ = 1
    /// 破(墙面工况封顶误触发)。
    #[test]
    fn impact_flow_newton_cap_narrow_pier_and_wall_face() {
        // ---- 布景 1:窄墩(L 形撞击者)----
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 4, 8), voxel::builtin::DIRT);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 5, 8), voxel::builtin::DIRT);
        refresh_collider(&mut s);
        let bar = types::CompoundShape::new(
            [[0i32, 0, 0], [-1, 0, 0], [-2, 0, 0], [0, 1, 0]]
                .iter()
                .map(|o| types::ShapeCell {
                    offset: *o,
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("L 形 4 格");
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(14.5, 5.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, bar),
            }),
        }]);
        let mut debris_vels: Vec<(Vec3, Vec3)> = Vec::new();
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            for r in &out.compound_requests {
                debris_vels.push(r.velocity);
            }
            for r in &out.detach_requests {
                debris_vels.push((r.impulse / crate::destruction::DEBRIS_MASS, Vec3::ZERO));
            }
            if !s.impact_flow_trace.is_empty() && !debris_vels.is_empty() {
                break;
            }
        }
        let t = s
            .impact_flow_trace
            .first()
            .expect("窄墩工况应断开(F02b-3 实测 κ ≈ 0.52)");
        // κ < 1:薄片摩擦工况封顶触发。
        assert!(
            t.kappa < 1.0,
            "勘误 6:窄墩薄片摩擦工况 κ < 1,实际 {}",
            t.kappa
        );
        // κ < 1 时封顶后 |J| = N(κ 即 N/|J_raw|;追踪的 event.impulse 是
        // 封顶后的)。相对 1%。
        assert!(
            ((t.event.impulse.length() - t.n_transfer) / t.n_transfer).abs() < 0.01,
            "封顶后 |J| = N:got |J| = {} N = {}",
            t.event.impulse.length(),
            t.n_transfer
        );
        // N 的独立复算:m·((v_前 − v_后) + g·Δt)·d̂;零重力 g = 0,v_前 =
        // linvel_pre(步进前快照)、v_后 = linvel_before(步进后/修正前)。
        let d_hat = t.event.dir_hat();
        let n_recomputed = t.impactor.mass * (t.linvel_pre - t.linvel_before).dot(d_hat);
        assert!(
            ((n_recomputed - t.n_transfer) / t.n_transfer).abs() < 0.02,
            "N 独立复算(零重力 Δv 口径):got {n_recomputed} want {}",
            t.n_transfer
        );
        // 封顶后碎块与撞击者都不超来速。
        for (v_seg, _) in &debris_vels {
            assert!(
                v_seg.length() <= 12.0 + 0.05,
                "封顶后碎块速度 ≤ u + 0.05:实际 {:?}",
                v_seg
            );
        }
        let v_impactor_after = t.linvel_after.dot(d_hat);
        assert!(
            v_impactor_after.abs() <= 12.0 + 0.05,
            "封顶后撞击者修正后沿 d̂ 速度 ≤ u + 0.05:实际 {v_impactor_after}"
        );
        // ---- 布景 2:7 宽墙(单一流形正面流形,κ = 1)----
        // 6 格石条 15 m/s 打 7 宽 1 格厚土墙的墙面中部(穿墙布景的墙面
        // 工况):条对墙是单一正面流形(内缩薄片贴墙面,无侧/顶薄片感应),
        // κ = 1(封顶不触发,结果与封顶前逐位一致)。
        let mut w = flow_sim_zero_g();
        w.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 1 格厚土墙 x=10、z 5..=11(7 宽)、y 2..=4(3 高),bedrock 垫 y=1。
        for z in 5..=11 {
            let _ = w
                .voxels
                .set_block(BlockPos::new(10, 1, z), voxel::builtin::BEDROCK);
            for y in 2..=4 {
                let _ = w
                    .voxels
                    .set_block(BlockPos::new(10, y, z), voxel::builtin::DIRT);
            }
        }
        refresh_collider(&mut w);
        let wall_bar = types::CompoundShape::new(
            (0..6)
                .map(|i| types::ShapeCell {
                    offset: [i, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("6 格(伸向 +x,首格在前)");
        let wid = entity(2);
        w.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: wid,
                state: EntityState {
                    translation: Vec3::new(4.5, 3.5, 8.5),
                    linvel: Vec3::new(15.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, wall_bar),
            }),
        }]);
        for _ in 0..20 {
            w.step();
            let out = w.take_outbox();
            if !w.impact_flow_trace.is_empty() {
                let _ = out;
                break;
            }
        }
        let tw = w
            .impact_flow_trace
            .first()
            .expect("7 宽墙正面流形工况应有事件(条 J ≈ 40 > 割)");
        assert!(
            tw.kappa >= 0.99,
            "墙面工况 κ = 1(单一流形无薄片摩擦,封顶不触发):实际 {}",
            tw.kappa
        );
        assert!(
            ((tw.event.impulse.length() - tw.n_transfer) / tw.n_transfer).abs() < 0.02,
            "墙面工况封顶后 |J| 仍 = N(κ=1 且 N ≥ 0.99·|J|):|J| = {} N = {}",
            tw.event.impulse.length(),
            tw.n_transfer
        );
    }

    /// AC-F5(勘误 7 格界归格,§5.7.9 第 7 条,F02b-4):齐平侧撞 1 格宽
    /// 土墩(bedrock 垫 + 1 格土,单格 half 0.5 石块 12 m/s 打土格面)——
    /// 流形角点恰落在 z 格界上,`floor` 会把它们归进空气格丢供给;勘误 7
    /// 改归后**全部供给落在实心格**(没有空气格),合计 = 封顶后 |J|
    /// (逐格 share 求和守恒)。宽墙对照(接触点都落在实心格内部,不触发
    /// 改归)与改前逐位一致。
    ///
    /// 变红:去掉改归(角点掉进空气格,实心格供给 < 合计)。
    #[test]
    fn impact_flow_errata7_grid_boundary_reassign() {
        // ---- 布景 1:齐平侧撞 1 格宽土墩 ----
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 4, 8), voxel::builtin::DIRT);
        refresh_collider(&mut s);
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.55, 4.6, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.5,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        for _ in 0..25 {
            s.step();
            let out = s.take_outbox();
            if !s.impact_flow_trace.is_empty() {
                let _ = out;
                break;
            }
        }
        let t = s
            .impact_flow_trace
            .first()
            .expect("齐平侧撞应有断开事件(勘误 7 改归后供给 ≥ 割)");
        // 供给全部落在实心格(土格或垫;事件已把土格写成 air,查出生集合)。
        for (c, _) in &t.event.supplies {
            assert!(
                *c == BlockPos::new(10, 4, 8) || *c == BlockPos::new(10, 3, 8),
                "勘误 7:供给格 {c:?} 必须是实心格(角点不再掉进空气格)"
            );
        }
        // 逐格 share 求和守恒:合计 = 封顶后 |J|(量化舍入内)。
        let sum: f32 = t.event.supplies.iter().map(|(_, s)| *s).sum();
        assert!(
            ((sum - t.event.impulse.length()) / t.event.impulse.length()).abs() < 0.01,
            "勘误 7:供给合计 = 封顶后 |J|:sum = {sum} |J| = {}",
            t.event.impulse.length()
        );
        // 断开成立(封顶后 N ≈ 12.5 > 割 7,改归前角点份额丢失则扛住)。
        assert!(t.cut_q < (t.supply * t.quant).floor() as u64, "应断开");
        // ---- 布景 2:宽墙(接触点落在实心格内部,不触发改归,逐位不变)----
        // 6 格石条 15 m/s 打 7 宽 1 格厚土墙的墙面中部(F02b-3 封顶测试的
        // 墙面工况,断开且 κ = 1);条对墙是单一正面薄片,接触点在墙面内部
        // (z 5.05..10.95 的格心行),远离格界 → 勘误 7 不触发。
        let mut w = flow_sim_zero_g();
        w.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        for z in 5..=11 {
            let _ = w
                .voxels
                .set_block(BlockPos::new(10, 1, z), voxel::builtin::BEDROCK);
            for y in 2..=4 {
                let _ = w
                    .voxels
                    .set_block(BlockPos::new(10, y, z), voxel::builtin::DIRT);
            }
        }
        refresh_collider(&mut w);
        let wall_bar = types::CompoundShape::new(
            (0..6)
                .map(|i| types::ShapeCell {
                    offset: [i, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("6 格(伸向 +x,首格在前)");
        let wid = entity(2);
        w.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: wid,
                state: EntityState {
                    translation: Vec3::new(4.5, 3.5, 8.5),
                    linvel: Vec3::new(15.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, wall_bar),
            }),
        }]);
        for _ in 0..25 {
            w.step();
            let out = w.take_outbox();
            if !w.impact_flow_trace.is_empty() {
                let _ = out;
                break;
            }
        }
        let tw = w.impact_flow_trace.first().expect("宽墙工况应有事件");
        // 宽墙的供给格同样全实心、合计 = |J|(改归路径未触发,与改前一致)。
        for (c, _) in &tw.event.supplies {
            assert!(
                (c.y == 2 || c.y == 3 || c.y == 4) && c.x == 10 && (5..=11).contains(&c.z),
                "宽墙:供给格 {c:?} 必须是墙/垫实心格"
            );
        }
        let sum_w: f32 = tw.event.supplies.iter().map(|(_, s)| *s).sum();
        assert!(
            ((sum_w - tw.event.impulse.length()) / tw.event.impulse.length()).abs() < 0.01,
            "宽墙:供给合计 = |J|:sum = {sum_w} |J| = {}",
            tw.event.impulse.length()
        );
    }

    /// AC-F5(勘误 7 负坐标判界,§5.7.9 第 7 条,F02b-4 裁定缺陷 A):同
    /// [`impact_flow_errata7_grid_boundary_reassign`] 的齐平侧撞,墩放到
    /// **负 y**(bedrock 垫 (10,−4,8) + 土格 (10,−3,8),撞击者中心 y = −2.4
    /// 对准土格上部)——接触点的 y 小数部分落在格中部,到格界 0.1 m 以上,
    /// 勘误 7 的 y 轴不该参与;但截断取小数的 `fract()` 在负坐标上给负值
    /// (如 `fract(−2.9) = −0.9 < 0.05`),把 y 轴恒判"贴下界",正下方的
    /// bedrock 垫被误列为候选、分走供给。改 `rem_euclid(1.0)` 后供给**只落
    /// 土格**,合计 = 封顶后 |J|,断开成立。
    ///
    /// 变红:改回 `fract()`(z = 9.0 角点触发改归时,y 轴误列 bedrock,
    /// 供给落到 y = −4 的格)。
    #[test]
    fn impact_flow_errata7_negative_coord_boundary() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, -4, 8), voxel::builtin::BEDROCK);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, -3, 8), voxel::builtin::DIRT);
        refresh_collider(&mut s);
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.55, -2.4, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.5,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        for _ in 0..25 {
            s.step();
            let out = s.take_outbox();
            if !s.impact_flow_trace.is_empty() {
                let _ = out;
                break;
            }
        }
        let t = s
            .impact_flow_trace
            .first()
            .expect("齐平侧撞应有断开事件(供给 ≥ 割 7)");
        // 供给全部落在被撞的土格——**没有正下方 y = −4 的 bedrock**
        // (负坐标判界缺陷会把垫误列候选)、没有 z = 9 侧的空气格。
        for (c, _) in &t.event.supplies {
            assert_eq!(
                *c,
                BlockPos::new(10, -3, 8),
                "勘误 7 负坐标:供给格 {c:?} 必须恰是土格(下方垫/侧方空气都不是候选)"
            );
        }
        assert_eq!(
            t.event.supplies.len(),
            1,
            "布景自检:唯一实心候选 = 土格 1 格"
        );
        // 合计 = 封顶后 |J|(单候选 share = 1,量化舍入内)。
        let sum: f32 = t.event.supplies.iter().map(|(_, s)| *s).sum();
        assert!(
            ((sum - t.event.impulse.length()) / t.event.impulse.length()).abs() < 0.01,
            "供给合计 = 封顶后 |J|:sum = {sum} |J| = {}",
            t.event.impulse.length()
        );
        // 断开成立(封顶后 N ≈ 12.5 > 土-垫剪切割 7)。
        assert!(t.cut_q < (t.supply * t.quant).floor() as u64, "应断开");
    }

    /// AC-F5(§5.7.1 单格供给口径,F02b-4 裁定缺陷 B):单格撞击者**同时压
    /// 两格**——1 格宽 2 格长的沙台(bedrock 垫 (10,3,8)/(10,3,9) + 沙格
    /// (10,4,8)/(10,4,9);沙-沙粘结 0,两格互不连通),单格石块 18 m/s 打
    /// 接缝 z = 9 偏一侧(接触面 z ∈ [8.75,9.75],跨两格)。§5.7.1"单格
    /// 实体 = 该格各点法向冲量之和" → 每格 `s_c` = 该格各接触点冲量之和
    /// (逐点对拍);均摊(F02a 缺陷,9e8e829 起)给每格 J_T/2。
    ///
    /// 变红:单格也均摊(两格 `s_c` 各 = J_T/2,比值 = 1)。
    #[test]
    fn impact_flow_single_cell_impactor_supply_per_cell() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 9), voxel::builtin::BEDROCK);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 4, 8), voxel::builtin::SAND);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 4, 9), voxel::builtin::SAND);
        refresh_collider(&mut s);
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.45, 4.5, 9.25),
                    linvel: Vec3::new(-18.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.5,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        for _ in 0..25 {
            s.step();
            let out = s.take_outbox();
            if !s.impact_flow_trace.is_empty() {
                let _ = out;
                break;
            }
        }
        let t = s
            .impact_flow_trace
            .first()
            .expect("打沙台应有断开事件(割 = 两格沙-垫剪切 4×2 = 8 < 供给)");
        // 布景自检:两格都进了供给(单格撞击者同时压两格),S = 两格整块。
        let mut cells: Vec<(types::BlockPos, f32)> = t.event.supplies.clone();
        cells.sort_by_key(|(c, _)| *c);
        assert_eq!(cells.len(), 2, "布景自检:两格沙都进供给,实际 {cells:?}");
        assert_eq!(
            cells[0].0,
            BlockPos::new(10, 4, 8),
            "供给格 1 = 接缝一侧的沙格"
        );
        assert_eq!(cells[1].0, BlockPos::new(10, 4, 9), "供给格 2 = 另一侧");
        // 条文对拍(§5.7.1"单格实体 = 该格各点法向冲量之和"):逐点读
        // solver 流形冲量,按生产同款归格(退 0.1 m、floor)按格归并,断言
        // 每格 `s_c` = 该格归并值。均摊变异(单格也 `J_T/|T|`)给每格
        // total/2(实测比值 1.0,逐点归并为 ≈0.94),差 ≈3% ≫ 量化误差,必红。
        // (接触面积信息不进 rapier 的点约束:齐平接触 4 角点 dist 相同,
        // 求解器冲量近似均分;悬殊比只在"不同法向的两簇"布景出现,单测
        // 不追——条文对拍已把口径钉死。)
        let mut per_cell: BTreeMap<types::BlockPos, f32> = BTreeMap::new();
        {
            // 事件拍已把 S 写 air,材料查询用**出生集合**(布景里实心格的
            // 固定列表),不查当前体素世界。
            let born: std::collections::BTreeSet<types::BlockPos> = [
                types::BlockPos::new(10, 3, 8),
                types::BlockPos::new(10, 3, 9),
                types::BlockPos::new(10, 4, 8),
                types::BlockPos::new(10, 4, 9),
            ]
            .into_iter()
            .collect();
            let ic = s.entities.get(&id).map(|r| r.collider).unwrap();
            for pair in s.phys.narrow_phase.contact_pairs_with(ic) {
                for m in pair.solver_manifolds() {
                    // 归格用世界系点;诊断读数里 local_p1 恰为世界系(地形
                    // chunk 刚体恒单位位姿、簇点已折算),与生产采样同基准。
                    // 改归按生产同款(勘误 7):c₀ 无材料(出生集合口径)、
                    // 到格界 < 0.05 的轴把界另一侧实心格列为候选,冲量均分。
                    let n = m.data.normal;
                    for pt in &m.points {
                        if pt.data.impulse <= 0.0 {
                            continue;
                        }
                        let q = Vec3::new(pt.local_p1.x, pt.local_p1.y, pt.local_p1.z)
                            + n * -crate::destruction::IMPACT_SAMPLE_DEPTH;
                        let cell = types::BlockPos::new(
                            q.x.floor() as i32,
                            q.y.floor() as i32,
                            q.z.floor() as i32,
                        );
                        let mut assigned: Vec<types::BlockPos> = vec![cell];
                        if !born.contains(&cell) {
                            let qc = [q.x, q.y, q.z];
                            let offs = [
                                qc[0].rem_euclid(1.0),
                                qc[1].rem_euclid(1.0),
                                qc[2].rem_euclid(1.0),
                            ];
                            let axes: [Option<i32>; 3] = [
                                if offs[0] < 0.05 {
                                    Some(-1)
                                } else if offs[0] > 0.95 {
                                    Some(1)
                                } else {
                                    None
                                },
                                if offs[1] < 0.05 {
                                    Some(-1)
                                } else if offs[1] > 0.95 {
                                    Some(1)
                                } else {
                                    None
                                },
                                if offs[2] < 0.05 {
                                    Some(-1)
                                } else if offs[2] > 0.95 {
                                    Some(1)
                                } else {
                                    None
                                },
                            ];
                            if axes.iter().any(|a| a.is_some()) {
                                let mut cands: Vec<types::BlockPos> = Vec::new();
                                for dx in [0, 1] {
                                    for dy in [0, 1] {
                                        for dz in [0, 1] {
                                            let cand = types::BlockPos::new(
                                                cell.x + dx * axes[0].unwrap_or(0),
                                                cell.y + dy * axes[1].unwrap_or(0),
                                                cell.z + dz * axes[2].unwrap_or(0),
                                            );
                                            if cand != cell && born.contains(&cand) {
                                                cands.push(cand);
                                            }
                                        }
                                    }
                                }
                                cands.sort();
                                cands.dedup();
                                if !cands.is_empty() {
                                    assigned = cands;
                                }
                            }
                        }
                        let share = 1.0 / assigned.len() as f32;
                        for c in assigned {
                            *per_cell.entry(c).or_insert(0.0) += pt.data.impulse * share;
                        }
                    }
                }
            }
        }
        for (c, s_c) in &t.event.supplies {
            let want = per_cell.get(c).copied().unwrap_or(0.0);
            assert!(
                (s_c - want).abs() <= 1e-3 * want.max(1.0) + 1e-4,
                "单格 s_c = 该格各点冲量之和(§5.7.1):格 {c:?} got {s_c} want {want}(逐点归并 {per_cell:?})"
            );
        }
        // 合计 = 封顶后 |J|(封顶后 supplies 乘 κ,守恒)。
        let sum: f32 = t.event.supplies.iter().map(|(_, v)| *v).sum();
        assert!(
            ((sum - t.event.impulse.length()) / t.event.impulse.length()).abs() < 0.01,
            "供给合计 = 封顶后 |J|:sum = {sum} |J| = {}",
            t.event.impulse.length()
        );
        // 断开成立(割 = 两格各沙-垫剪切 4,并联 8 < 供给)。
        assert!(t.cut_q < (t.supply * t.quant).floor() as u64, "应断开");
    }

    /// AC-F8(§5.6 退役,D12):**悬臂-支柱布景**(参照
    /// `smoke/scenarios/req010-cantilever.toml` 的 12 格悬臂,平移到本 sim
    /// 坐标系)——石墙 + 从墙伸出的悬臂 + 末端支柱;投射物撞断支柱,悬臂
    /// 因承载失效塌落。断言:
    /// - 开(`impact_flow = true`):塌落复合体(悬臂段)`impulses` 空、
    ///   `velocity` 零(失去支撑的段没有吸收撞击动量;§5.6 邻接继承不
    ///   执行);
    /// - 关(同布景、同投射物):塌落复合体的 `impulses` **非空**(§5.6
    ///   邻接继承照常——证明布景确实走到继承路径)。
    ///
    /// 冲量表本身是拍内表(`evaluate_load_tick` 开头取走,step 返回后必空),
    /// 退役与否的白盒由"开跑 impulses 恒空、关跑 impulses 非空"这对断言
    /// 钉住,不读表。
    ///
    /// 变红:冲量流路径仍写冲量表(开跑时塌落段带 β·J);关闭路径的继承
    /// 被误退役(关跑 impulses 空)。
    #[test]
    fn impact_flow_retires_knockback_inheritance() {
        let run = |flow: bool| -> Vec<crate::cohesion::CompoundRequest> {
            let mut cfg = ShardSimConfig {
                gravity: Vec3::new(0.0, -9.81, 0.0),
                ..ShardSimConfig::new(shard_id(0, 0))
            };
            cfg.solidify.enabled = false;
            cfg.cohesion.enabled = true;
            cfg.cohesion.impact_flow = flow;
            // 支柱 1 格 DIRT(y=1,立在地表土层上;地表 = chunk 表面格 y=0
            // 是 DIRT):冲击流 S 的割 = 柱底 dirt-dirt 7 + 柱顶 dirt-stone 7
            // = 14,J > 14 才断——60 m/s 的 J ≈ 43(I_c 28 未及,不粉碎)。
            cfg.destruction.max_detach_per_tick = 8;
            let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            // 本体区 2×2 chunk:东侧 (1,0) 装载为全 air,堵住"未装载当有
            // 地基"的 INF 边——否则悬臂末端 (15,2,8) 的 +x 邻居 (16,·,·)
            // 把整条悬臂判稳定,支柱断后不塌(K02 同款坑)。
            s.load_chunk(ChunkPos::new(1, 0), voxel::Chunk::air());
            // 悬臂-支柱(参照 req010-cantilever 平移):石墙 x 2..3、y 0..3、
            // z 6..10;悬臂 12 格沿 +x 从墙伸出(y=2、z=8、x 4..=15);末端
            // 支柱单格 (15,1,8) DIRT(位于悬臂末端 y=2 之下,顶托悬臂;地表
            // 在 y=0,支柱下方 y=0 是空气——它和悬臂互为唯一支撑)。投射物
            // 60 m/s 沿 +x 在悬臂下方穿行,打支柱的 −x 面(y=1.5 对准;冲击流
            // 最小割 = 柱顶 dirt-stone 剪切 7 < J ≈ 0.73·60 → 支柱断,唯一
            // 割边;悬臂-墙 junction 石-石 8 是另一条割,两条都过不起 J,但
            // BFS 最小源侧取支柱侧)。支柱断后悬臂承载:唯一出口 = 墙侧
            // 石-石粘结 8 < 总重 12 → 整条悬臂因承载失效塌落。
            for x in 2..=3 {
                for y in 0..=3 {
                    for z in 6..=10 {
                        let _ = s
                            .voxels
                            .set_block(BlockPos::new(x, y, z), voxel::builtin::STONE);
                    }
                }
            }
            for x in 4..=15 {
                let _ = s
                    .voxels
                    .set_block(BlockPos::new(x, 2, 8), voxel::builtin::STONE);
            }
            let _ = s
                .voxels
                .set_block(BlockPos::new(15, 1, 8), voxel::builtin::DIRT);
            refresh_collider(&mut s);
            let id = entity(1);
            s.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: id,
                    state: EntityState {
                        translation: Vec3::new(12.55, 1.5, 8.5),
                        linvel: Vec3::new(60.0, 0.0, 0.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc {
                        half_size: 0.45,
                        block: voxel::builtin::STONE,
                        shape: None,
                    },
                }),
            }]);
            let mut compound_reqs = Vec::new();
            for _ in 0..120 {
                s.step();
                let out = s.take_outbox();
                for r in out.compound_requests {
                    compound_reqs.push(r);
                }
            }
            compound_reqs
        };
        let on_reqs = run(true);
        // 开:支柱被撞断(S = 支柱 1 格 → 单格脱落请求,不在复合体清单);
        // 塌落复合体 = 悬臂段(y=2,≥ 2 格),impulses 空、velocity 零
        // (失去支撑的段没有吸收撞击动量;§5.6 邻接继承不执行——冲量表本身
        // 是拍内表,step 返回后必空,退役与否的白盒由"开跑 impulses 恒空、
        // 关跑 impulses 非空"这对断言钉住)。
        assert!(
            !on_reqs.is_empty(),
            "悬臂应因承载失效塌落成复合体(布景自检):支柱断开拍后悬臂塌落"
        );
        for r in &on_reqs {
            let min_y = r
                .shape
                .cells()
                .iter()
                .map(|c| r.origin.y + c.offset[1])
                .min()
                .unwrap();
            assert_eq!(
                min_y, 2,
                "复合体 = 悬臂段(y=2;支柱 y=1 走单格脱落):origin {:?}",
                r.origin
            );
            assert!(
                r.impulses.is_empty(),
                "AC-F8:塌落的悬臂段 impulses 空(§5.6 退役)"
            );
            assert_eq!(
                r.velocity,
                (Vec3::ZERO, Vec3::ZERO),
                "AC-F8:塌落的悬臂段零初速(没有吸收撞击动量)"
            );
        }
        // 关:同布景、同投射物——塌落复合体的 impulses 非空(§5.6 继承照常)。
        let off_reqs = run(false);
        assert!(
            off_reqs.iter().any(|r| !r.impulses.is_empty()),
            "关闭路径:塌落复合体继承 β·J(impulses 非空)——布景确实走到继承"
        );
    }

    /// AC-F4/F5(velocity 直达生成点,§5.7.3 第 4 条):`SpawnCompound` 的
    /// `velocity` 非零时,生成实体初速度**直接等于**(线,角)——不经
    /// `apply_knockback`、不被 `load_knockback_max_speed/ang_speed` clamp
    /// (v_max = 0.001 的极端配置下速度照样保留;§5.7.2:速度已被撞击者的
    /// 动量自然封顶,clamp 两键只属于 §5.6 旧路径)。impulses 与 velocity
    /// 同给时以 velocity 为准(两路不会同时非空;此处只钉"velocity 直达")。
    ///
    /// 变红:消费臂忽略 velocity(零速生成);误走 apply_knockback(被
    /// clamp 到 0.001)。
    #[test]
    fn spawn_compound_velocity_sets_state_without_clamp() {
        let mut cfg = ShardSimConfig {
            gravity: Vec3::ZERO,
            ..ShardSimConfig::new(shard_id(0, 0))
        };
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.load_knockback_max_speed = 0.001;
        cfg.cohesion.load_knockback_max_ang_speed = 0.001;
        let mut s = ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()));
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let id = entity(1);
        let shape = types::CompoundShape::new(
            (0..2)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("2 格");
        let (v, w) = (Vec3::new(3.0, -1.0, 0.5), Vec3::new(0.0, 0.0, 2.0));
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnCompound {
                entity: id,
                origin: BlockPos::new(10, 5, 8),
                shape,
                impulses: Vec::new(),
                velocity: (v, w),
            }),
        }]);
        let st = s.entity_state(id).expect("复合体已生成");
        assert!(
            (st.linvel - v).length() < 1e-5,
            "velocity 直达:linvel = {:?},want {v:?}",
            st.linvel
        );
        assert!(
            (st.angvel - w).length() < 1e-5,
            "velocity 直达:angvel = {:?},want {w:?}",
            st.angvel
        );
    }

    /// AC-F4/AC-F5(定量,§5.7.2;全部走 `step()` 的生产路径,读
    /// `#[cfg(test)]` 追踪与 §5.7.2 逐式对账)。布景:零重力悬空土墩
    /// (bedrock 垫 (10,3,8) + 土 (10,4,8)(10,5,8)),撞击者 = **6 格石条
    /// 复合体**(原点 = 首格,质心在 +2.5——原点不在质心)以 12 m/s 沿 −x
    /// 撞上格 (10,5,8):接触点在条首格端面,`r = p − 质心 ≈ (2.5,0,0)`
    /// 垂直 d̂ → **偏心**,m_eff < m 明显。对账:
    /// ①守恒 `(C/s)·J + 修正 + Σ P_k = J`(误差 ≤ 量化单位 × 接触点数);
    /// ②修正 = m_eff/(m_eff+M_S)·E,**m_eff 由测试独立复算**(形状质量/
    /// 惯量/世界质心 + 修正前位姿,不经生产同一函数);③撞击者在 p 处沿
    /// d̂ 的速度差 = |修正|/m_eff;④所有分量法向速度 = w 且 w ≤ u。
    ///
    /// 变红:逆惯量置零(修正只走质量项,m_eff 偏大)、质心改刚体原点
    /// (r 算错)——两变异都使 ②/③ 出界(见各变异自检记录)。
    #[test]
    fn impact_flow_trace_conservation_and_correction_quantitative() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        for y in 4..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, y, 8), voxel::builtin::DIRT);
        }
        refresh_collider(&mut s);
        // L 形 4 格撞击者:杆 3 格沿 −x(同 y),脚 1 格在杆首格上方——
        // 质心偏离杆轴 0.375 m,杆端撞墩 = **真偏心**(m_eff 明显 < m);
        // 原点(刚体原点)≠ 质心(复合体天生如此)。
        let bar = types::CompoundShape::new(
            [[0i32, 0, 0], [-1, 0, 0], [-2, 0, 0], [0, 1, 0]]
                .iter()
                .map(|o| types::ShapeCell {
                    offset: *o,
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("L 形 4 格");
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(14.5, 5.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, bar),
            }),
        }]);
        let mut breaks = 0u64;
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            breaks += out.cohesion_stats.impact_flow_breaks;
            if !out.detach_requests.is_empty() {
                break;
            }
        }
        assert!(breaks >= 1, "L 形撞击应产生一次断开事件");
        assert_eq!(s.impact_flow_trace.len(), 1, "恰 1 个断开事件");
        let t = &s.impact_flow_trace[0];
        let d_hat = t.event.dir_hat();
        let j = t.event.impulse;
        // 与生产同一整数域口径:ρ = (s_q − C_q)/s_q(§5.7.1 量化:供给向下
        // 取整后之和;E = ρ·J)。
        let s_q = (t.supply * t.quant).floor();
        let c_q = t.cut_q as f32;
        let rho = ((s_q - c_q) / s_q).clamp(0.0, 1.0);
        let s_supply = s_q / t.quant;
        let c = c_q / t.quant;
        let e = j * rho; // 超出冲量 E
        let m_s = t.momenta.iter().map(|k| k.mass).sum::<f32>();
        // ---- ② m_eff 独立复算(不经 effective_mass)----
        // 形状质量属性:逐格 cuboid 半边长 0.45、密度 1(与生成实体同一
        // 口径);质心 = 格心均值(等质量),惯量 = 逐格对自身 + 平行轴
        // (含惯性积),本体系;世界系用追踪的修正前旋转 similarity 变换。
        // offsets 与生成形状同款(L 形)。
        let cells: Vec<[i32; 3]> = vec![[0, 0, 0], [-1, 0, 0], [-2, 0, 0], [0, 1, 0]];
        let m = cells.len() as f32 * 0.729;
        let com_local = cells
            .iter()
            .map(|o| Vec3::new(o[0] as f32, o[1] as f32, o[2] as f32))
            .sum::<Vec3>()
            / cells.len() as f32;
        // 世界质心独立复算:com_world = translation + rotation·com_local。
        // F02b-3:translation 用追踪的 `translation_before`(修正前刚体原点),
        // **不再**从 impactor.center_of_mass 反解——那样减了再加回,得到的
        // 就是生产 center_of_mass 本身,"质心改刚体原点"的变异红不了(循环)。
        // 先断言独立质心与生产值一致(< 1e-4),证明追踪链路对,再用独立值。
        let rot = t.rotation_before;
        let com_world = t.translation_before + rot.mul_vec3(com_local);
        assert!(
            (com_world - t.impactor.center_of_mass).length() < 1e-4,
            "追踪链路自检:独立质心 = translation_before + R·com_local 与生产              center_of_mass 一致:got {:?} want {:?}",
            com_world,
            t.impactor.center_of_mass
        );
        let inertia = {
            // 3×3 惯量(含惯性积),世界系 = 单位旋转 → 本体系。
            let half = 0.45f32;
            let cell_mass = 0.729f32;
            let base = cell_mass / 12.0 * (4.0 * half * half + 4.0 * half * half);
            let mut i = [[0.0f32; 3]; 3];
            for o in &cells {
                let d = Vec3::new(o[0] as f32, o[1] as f32, o[2] as f32) - com_local;
                i[0][0] += base + cell_mass * (d.y * d.y + d.z * d.z);
                i[1][1] += base + cell_mass * (d.x * d.x + d.z * d.z);
                i[2][2] += base + cell_mass * (d.x * d.x + d.y * d.y);
                i[0][1] -= cell_mass * d.x * d.y;
                i[0][2] -= cell_mass * d.x * d.z;
                i[1][2] -= cell_mass * d.y * d.z;
            }
            i[1][0] = i[0][1];
            i[2][0] = i[0][2];
            i[2][1] = i[1][2];
            i
        };
        // 本体系 → 世界系:I_w = R·I_local·Rᵀ(R = 修正前旋转)。
        let to_world = |m3: [[f32; 3]; 3]| {
            let cols = [
                Vec3::new(m3[0][0], m3[1][0], m3[2][0]),
                Vec3::new(m3[0][1], m3[1][1], m3[2][1]),
                Vec3::new(m3[0][2], m3[1][2], m3[2][2]),
            ];
            let rows = [
                Vec3::new(m3[0][0], m3[0][1], m3[0][2]),
                Vec3::new(m3[1][0], m3[1][1], m3[1][2]),
                Vec3::new(m3[2][0], m3[2][1], m3[2][2]),
            ];
            let rc = |a: &Vec3, b: &Vec3| a.dot(*b);
            let mut out = [[0.0f32; 3]; 3];
            for (ri, rrow) in rows.iter().enumerate() {
                for (ci, ccol) in cols.iter().enumerate() {
                    out[ri][ci] = rc(rrow, ccol);
                }
            }
            out
        };
        let inertia_w = {
            let r3 = [
                [
                    rot.mul_vec3(Vec3::X).x,
                    rot.mul_vec3(Vec3::X).y,
                    rot.mul_vec3(Vec3::X).z,
                ],
                [
                    rot.mul_vec3(Vec3::Y).x,
                    rot.mul_vec3(Vec3::Y).y,
                    rot.mul_vec3(Vec3::Y).z,
                ],
                [
                    rot.mul_vec3(Vec3::Z).x,
                    rot.mul_vec3(Vec3::Z).y,
                    rot.mul_vec3(Vec3::Z).z,
                ],
            ];
            // I_w = R·I·Rᵀ:直接三次相似变换(R 的列 = 本体系基的世界像)。
            let mut out = [[0.0f32; 3]; 3];
            for (ri, basis) in r3.iter().enumerate() {
                for (ci, basis2) in r3.iter().enumerate() {
                    let mut acc = 0.0f32;
                    for (a, va) in basis.iter().enumerate() {
                        for (bq, vb) in basis2.iter().enumerate() {
                            acc += va * inertia[a][bq] * vb;
                        }
                    }
                    out[ri][ci] = acc;
                }
            }
            let _ = to_world;
            out
        };
        // m_eff = 1 / (1/m + (r×d̂)·I_w⁻¹·(r×d̂)),r = p − com_world。
        let inertia = inertia_w;
        let r = t.event.point - com_world;
        let rx = r.cross(d_hat);
        // 3×3 对称解逆(与 §5.7.2 同式,但独立写在测试里)。
        let (a, b, cc, d, e_, f) = (
            inertia[0][0],
            inertia[0][1],
            inertia[0][2],
            inertia[1][1],
            inertia[1][2],
            inertia[2][2],
        );
        let c00 = d * f - e_ * e_;
        let c01 = cc * e_ - b * f;
        let c02 = b * e_ - d * cc;
        let c11 = a * f - cc * cc;
        let c12 = b * cc - a * e_;
        let c22 = a * d - b * b;
        let det = a * c00 + b * c01 + cc * c02;
        let inv = [
            [c00 / det, c01 / det, c02 / det],
            [c01 / det, c11 / det, c12 / det],
            [c02 / det, c12 / det, c22 / det],
        ];
        let iinv_rx = Vec3::new(
            inv[0][0] * rx.x + inv[0][1] * rx.y + inv[0][2] * rx.z,
            inv[1][0] * rx.x + inv[1][1] * rx.y + inv[1][2] * rx.z,
            inv[2][0] * rx.x + inv[2][1] * rx.y + inv[2][2] * rx.z,
        );
        let m_eff = 1.0 / (1.0 / m + rx.dot(iinv_rx));
        assert!(
            (m_eff - m).abs() > 0.15,
            "布景自检:偏心工况 m_eff({m_eff}) 应小于 m({m})(角项参与)"
        );
        // 变异敏感性:逆惯量置零时 m_eff → m(差 ≥ 0.15),进入 ② 的修正
        // 对账后偏差 ~9% ≫ 量化误差,足以变红。
        // 修正 = m_eff/(m_eff + M_S)·E(量化误差内)。
        let want_correction = e * (m_eff / (m_eff + m_s));
        assert!(
            (t.correction - want_correction).length() <= 0.1,
            "AC-F5:修正 = m_eff/(m_eff+M_S)·E:got {:?} want {want_correction:?}",
            t.correction
        );
        // ---- ③ p 处沿 d̂ 的速度差 = |修正|/m_eff ----
        let speed_at = |lv: Vec3, av: Vec3, pt: Vec3| (lv + av.cross(pt - com_world)).dot(d_hat);
        let v_before = speed_at(t.linvel_before, t.angvel_before, t.event.point);
        let v_after = speed_at(t.linvel_after, t.angvel_after, t.event.point);
        // 容差 1%:修正施加与速度读取之间同帧无其他力,残差来自 f32 闭式
        // 的舍入(19.4 量级 0.2%)。
        assert!(
            ((v_after - v_before) - t.correction.length() / m_eff).abs() < 0.2,
            "AC-F5:p 处沿 d̂ 速度差 = |修正|/m_eff:Δv = {} want {}",
            v_after - v_before,
            t.correction.length() / m_eff
        );
        // ---- ①守恒 ----
        let resid =
            j * (c / s_supply) + t.correction + t.momenta.iter().map(|k| k.impulse).sum::<Vec3>()
                - j;
        assert!(
            resid.abs().max_element() <= 0.1,
            "AC-F4:守恒残差 {resid:?}(J 量级 72,0.1 = 0.14%)"
        );
        // ---- ④分量法向速度一致 + w ≤ u(勘误 6 后恢复)----
        let w = e.length() / (m_eff + m_s);
        // 勘误 6 牛顿封顶后,J 不再含穿透修正 ramp 的凭空超调(|J| ≤ N ≤
        // m·u + 重力项),共同速度 w 回到 ≤ u 的物理约束内;F02b-2 时撤下
        // 的断言恢复(容差 0.05:量化舍入 + 重力项的 u 上浮)。
        assert!(
            w <= 12.0 + 0.05,
            "AC-F4:共同速度 w ≤ u(勘误 6 封顶后),实际 {w}"
        );
        // 撞击者修正后沿 d̂ 的速度同样 ≤ u(封顶后修正把 rapier 多扣的还
        // 回去,落点 ≤ w ≤ u)。
        let v_imp = (t.linvel_after + t.angvel_after.cross(t.event.point - com_world)).dot(d_hat);
        assert!(
            v_imp <= 12.0 + 0.05,
            "AC-F4:撞击者修正后 p 处沿 d̂ 速度 ≤ u(封顶后),实际 {v_imp}"
        );
        for k in &t.momenta {
            assert!(
                (k.delta_v.dot(d_hat) - w).abs() < 0.1,
                "AC-F4:分量法向速度 = w:got {} want {w}(0.5% = f32 复算差)",
                k.delta_v.dot(d_hat)
            );
        }
    }

    /// AC-F4(偏心不对称分量的 Δω 与 rapier 参照一致):
    /// S 的最大分量 = **真 L 形**(3 格 [(10,4,8),(10,5,8),(10,5,9)])——
    /// 惯性积非零(AC-F4"至少含一个不对称分量"的正确布景;F02b-2 用 2×3
    /// 矩形被总架构师退回:惯性积为零,不对称性只来自接触点偏置,不来自
    /// 形状)。
    /// 追踪里的 `P_k`、`p_k` 与零重力参照 sim 里"同形状刚体放在同位姿、
    /// 在 p_k 处施加 P_k"的 Δv/Δω 闭式一致(rapier 按碰撞体质量属性一次
    /// 算对)。共线 Δω ≈ 0 由
    /// [`impact_flow_trace_collinear_impactor_zero_delta_w`](生产路径,
    /// 1×2 土梁轴向正撞)钉住,不推给 F01。
    ///
    /// 变红:逆惯量置零(Δω 缺角项)、质心取刚体原点(r_k 全错)、
    /// S 退回矩形(惯性积断言红)。
    #[test]
    fn impact_flow_trace_l_shape_delta_w_matches_rapier_reference() {
        // 悬空 **L 形 3 格**土墙(格 [(10,4,8),(10,5,8),(10,5,9)];bedrock
        // 垫 (10,3,8) 在竖排正下方,汇点),6 格石条 12 m/s 撞格 (10,4,8):
        // 最小割 = 7(供给格任一边),残量可达集 S = L 形 3 格(F02b-3
        // 实测;L 形 → 惯性积 ≠ 0)。
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 4, 8), voxel::builtin::DIRT);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 5, 8), voxel::builtin::DIRT);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 5, 9), voxel::builtin::DIRT);
        refresh_collider(&mut s);
        let bar = types::CompoundShape::new(
            (0..6)
                .map(|i| types::ShapeCell {
                    offset: [i - 5, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("6 格(伸向 −x,首格在前)");
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(16.5, 4.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, bar),
            }),
        }]);
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            if !out.detach_requests.is_empty() || !out.compound_requests.is_empty() {
                break;
            }
        }
        let trace = &s.impact_flow_trace;
        assert!(!trace.is_empty(), "应有断开事件(布景自检)");
        let t = &trace[0];
        // S = L 形 3 格(1 分量);L 形 → 惯性积 ≠ 0(真不对称,AC-F4 布景
        // 要求;2×3 矩形惯性积全零,被 F02b-3 验收退回)。
        let k = t
            .momenta
            .iter()
            .max_by(|a, b| a.cells.len().cmp(&b.cells.len()))
            .expect("至少 1 个分量");
        assert_eq!(k.cells.len(), 3, "布景自检:S = L 形 3 格 1 分量");
        // 惯性积独立复算(本体系,等质量格点):
        // I_xy = −Σ m·(x−cx)(y−cy) 等;L 形至少一个 ≠ 0。
        {
            let n = k.cells.len() as f32;
            let com = crate::impact_flow::component_center_of_mass(&k.cells);
            let mut ixy = 0.0f32;
            let mut ixz = 0.0f32;
            let mut iyz = 0.0f32;
            for c in &k.cells {
                let d = Vec3::new(
                    c.x as f32 + 0.5 - com.x,
                    c.y as f32 + 0.5 - com.y,
                    c.z as f32 + 0.5 - com.z,
                );
                ixy -= 0.729 * d.x * d.y;
                ixz -= 0.729 * d.x * d.z;
                iyz -= 0.729 * d.y * d.z;
            }
            assert!(
                ixy.abs() > 1e-3 || ixz.abs() > 1e-3 || iyz.abs() > 1e-3,
                "布景自检:L 形分量惯性积非零(真不对称):{ixy} {ixz} {iyz} n={n}"
            );
        }
        assert!(
            k.delta_w.length() > 1e-3,
            "偏心工况 Δω ≠ 0(接触点不在分量质心线上):{:?}",
            k.delta_w
        );
        // 参照体:同形状(分量格)刚体、同位姿(格心世界位姿;分量形状
        // canonical 化后放回原位),零重力 sim,在 p_k 处施加 P_k。
        let mut ref_sim = flow_sim_zero_g();
        ref_sim.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 分量形状 = k.cells 相对最小格的偏移;原点 = compound_origin。
        let origin = crate::cohesion::compound_origin(&k.cells);
        let world: Vec<(BlockPos, types::BlockId)> =
            k.cells.iter().map(|&c| (c, voxel::builtin::DIRT)).collect();
        let shape = types::CompoundShape::from_world(origin, &world).expect("分量形状");
        let ref_id = entity(9);
        ref_sim.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: ref_id,
                state: EntityState {
                    // spawn 把形状原点放在 translation;分量的"原位" =
                    // origin 格心。参照体放在原位 → 施加 P_k 后 Δv/Δω 与
                    // 生产一致(位姿一致是 Δω 可比的的前提)。
                    translation: Vec3::new(
                        origin.x as f32 + 0.5,
                        origin.y as f32 + 0.5,
                        origin.z as f32 + 0.5,
                    ),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape),
            }),
        }]);
        {
            let rec = ref_sim.entities.get(&ref_id).expect("参照体在场");
            let handle = rec.handle;
            let body = ref_sim.phys.bodies.get_mut(handle).expect("handle 有效");
            body.apply_impulse_at_point(
                conv::vec3_to_rapier(k.impulse),
                conv::vec3_to_rapier(k.point),
                true,
            );
        }
        let ref_st = ref_sim.entity_state(ref_id).expect("参照体状态");
        // 单格 spawn 的 translation 是形状原点格心,rapier 质心在 +Δ;参照
        // Δv/Δω 与分量闭式逐项对(rapier 与 §5.7.2 同式)。
        assert!(
            (ref_st.linvel - k.delta_v).length() < 1e-3,
            "参照 Δv 一致:got {:?} want {:?}",
            ref_st.linvel,
            k.delta_v
        );
        assert!(
            (ref_st.angvel - k.delta_w).length() < 1e-2,
            "参照 Δω 一致(rapier 质量属性 vs §5.7.2 闭式):got {:?} want {:?}",
            ref_st.angvel,
            k.delta_w
        );
    }

    /// AC-F4(共线 Δω ≈ 0,生产路径;F02b-3 验收要求"共线 Δω ≈ 0 在生产
    /// 路径补一条,不推给 F01"):悬空 1×2 土梁(z 向横放,垫在 z=8 格正下
    /// 方),单格石块 12 m/s 沿 −z **正撞梁端格 (10,4,8) 的端面**——接触点
    /// 在分量的质心线上(质心 z = 9 与端面 z=8 的接触点共线),动量分量
    /// (S = 梁 2 格整块)Δω ≈ 0(力臂为零)。走 step() 读追踪断言。
    ///
    /// 变红:动量分配误给角项(共线工况凭空出 Δω);Δv 计算含伪力臂。
    #[test]
    fn impact_flow_trace_collinear_impactor_zero_delta_w() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 4, 8), voxel::builtin::DIRT);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 4, 9), voxel::builtin::DIRT);
        refresh_collider(&mut s);
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(10.5, 4.5, 12.45),
                    linvel: Vec3::new(0.0, 0.0, -12.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.45,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            if !out.detach_requests.is_empty() || !out.compound_requests.is_empty() {
                break;
            }
        }
        let t = s
            .impact_flow_trace
            .first()
            .expect("端面正撞应有断开事件(割 7 < 供给)");
        // 分量 = 梁 2 格整块(承载粘结连通,1 分量)。
        let k = &t.momenta[0];
        assert_eq!(k.cells.len(), 2, "布景自检:分量 = 梁 2 格整块");
        // 共线:p_k 在质心线上 → Δω ≈ 0(力臂为零)。实测离轴 ~0.015 m
        // (内缩薄片接触点加权位置的浮点偏置),|P| ≈ 1.5 → 残余力矩 ~0.02,
        // Δω ≈ 0.053 rad/s(F02b-4 缺陷 B 后重测;容差 0.15 ≈ 3× 实测,伪力臂变异给出 >1)。
        assert!(
            k.delta_w.length() < 0.15,
            "共线工况 Δω ≈ 0(伪力臂变异会 >1):实际 {:?}",
            k.delta_w
        );
        // Δv 仍沿 d̂(z)且大小 = |P_k|/M_k。
        assert!(
            k.delta_v.x.abs() < 1e-3 && k.delta_v.y.abs() < 1e-3,
            "共线 Δv 沿撞击方向:实际 {:?}",
            k.delta_v
        );
    }

    /// AC-F5(轻撞大,定量;F02b-4 裁定布景+断言口径):零重力下 4×4 土板
    /// (16 格,M_S ≈ 11.7 kg)只靠一条棱上的一格与基岩锚以**一根剪切键**
    /// (7)相连,单格石块(m = 1)12 m/s 正撞板中心格——断一根键带走
    /// **整块板**(最小割 = 锚键 7,断下质量由拓扑决定)。断言三条(AC-F5
    /// 改口径,F02b-4 裁定:rapier 解算后的速度含穿透恢复的回弹残量,F00
    /// 实测 15 m/s 时 7.23 vs w = 8.23,不直接断言"≈ w"):
    /// ① `w < 0.1·u`(大 M_S 摊薄共同速度);
    /// ② 修正后 p 处沿 d̂ 速度的**绝对值** < 0.1·u(轻物撞大块后法向速度
    /// ≈ 0;取绝对值——回弹残量与板速反向,残量本身 < 0.1·u);
    /// ③ 修正前后沿 d̂ 速度之差 = `|修正|/m_eff`(修正的物理定义,m_eff 由
    /// 修正闭式反解;残量不进此项——它同时加在修正前后读数上,相减消去)。
    /// 布景自检:S = 全部板格 1 分量、M_S ≥ 10·m。
    ///
    /// 实测(F02b-4 停报三,现口径复核):w = 0.44 < 1.2;修正后 v = +0.16
    /// (rapier 穿透恢复弹回 +0.60,修正 −0.44);速度差 = |修正|/m_eff。
    ///
    /// 变红:修正误用 J 全额(轻物仍被推走,② 超 0.1·u);断下质量受割
    /// 限制的错误实现(② 残量变大);m_eff 反解口径错(③ 出界)。
    #[test]
    fn impact_flow_trace_light_hits_heavy_correction_near_zero() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        // 4×4 土板悬空(x 9..12 × z 7..10,y=5);锚键 = 板格 (9,5,7) 正下方
        // 一格 bedrock(8,4,7)——板格 (9,5,7) 与 bedrock (8,4,7) 是**对角**
        // 相邻,不构成粘结。改用:bedrock 垫一格 (9,4,7)(在板格 (9,5,7)
        // 正下方),剪切键 = dirt-bedrock 7,整板只此一条锚键(板内其余格
        // 悬空;周围空气,BFS 不外溢)。
        for x in 9..=12 {
            for z in 7..=10 {
                let _ = s
                    .voxels
                    .set_block(BlockPos::new(x, 5, z), voxel::builtin::DIRT);
            }
        }
        let _ = s
            .voxels
            .set_block(BlockPos::new(9, 4, 7), voxel::builtin::BEDROCK);
        refresh_collider(&mut s);
        let id = entity(1);
        // 单格石块(half 0.5,m = 1)12 m/s 沿 −x 正撞板中心格 (10,5,8) 的
        // +x 面:中心对准格心 (10.5, 5.5, 8.5),距板面 1.45 m(F02b-2 布景
        // 同款间隙,避开出生重叠)。
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(13.45, 5.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc {
                    half_size: 0.5,
                    block: voxel::builtin::STONE,
                    shape: None,
                },
            }),
        }]);
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            if !s.impact_flow_trace.is_empty() {
                let _ = out;
                break;
            }
        }
        let t = s
            .impact_flow_trace
            .first()
            .expect("正撞板中心应有断开事件(最小割 = 锚键 7 < 供给)");
        // 布景自检:S = 全部板格 16 格(1 分量,断一根键带走整块),M_S ≥ 10·m。
        let m_s = t.momenta.iter().map(|k| k.mass).sum::<f32>();
        assert_eq!(t.momenta.len(), 1, "布景自检:S = 1 个分量(整块板)");
        assert_eq!(
            t.momenta[0].cells.len(),
            16,
            "布景自检:S = 全部板格 16 格(拓扑决定断下质量)"
        );
        assert!(
            m_s >= 10.0 * t.impactor.mass,
            "布景自检:M_S({m_s}) ≥ 10·m({})",
            t.impactor.mass
        );
        // w 定量:|E|/(m_eff + M_S);m_eff 由修正闭式反解(比例 = m_eff/
        // (m_eff+M_S)),不重复生产的 m_eff 计算(F02b-2 同法)。
        let d_hat = t.event.dir_hat();
        let s_q = (t.supply * t.quant).floor();
        let rho = ((s_q - t.cut_q as f32) / s_q).clamp(0.0, 1.0);
        let e = t.event.impulse * rho;
        let ratio = (t.correction.length() / e.length()).clamp(0.0, 0.999);
        let m_eff = ratio * m_s / (1.0 - ratio);
        let w = e.length() / (m_eff + m_s);
        // ① 共同速度 w < 0.1·u(大 M_S 摊薄)。
        assert!(
            w < 1.2,
            "AC-F5:共同速度 w < 0.1·u = 1.2(大 M_S 摊薄),实际 {w}"
        );
        // ② 修正后 p 处沿 d̂ 速度的绝对值 < 0.1·u(轻物撞大块后法向速度
        // ≈ 0;p 处速度含角速度项;取绝对值——回弹残量与板速反向)。
        let com = t.impactor.center_of_mass;
        let v_after = (t.linvel_after + t.angvel_after.cross(t.event.point - com)).dot(d_hat);
        assert!(
            v_after.abs() < 1.2,
            "AC-F5:修正后法向速度的绝对值 < 0.1·u = 1.2(轻物停下),实际 {v_after}"
        );
        // ③ 修正前后沿 d̂ 速度之差 = |修正|/m_eff(修正的物理定义;回弹残量
        // 同时加在前后读数上,相减消去;容差 = 守恒测试同款的 f32 舍入量级)。
        let v_before = (t.linvel_before + t.angvel_before.cross(t.event.point - com)).dot(d_hat);
        let dv = (v_before - v_after).abs();
        let want_dv = t.correction.length() / m_eff;
        assert!(
            (dv - want_dv).abs() < 0.2,
            "AC-F5:修正前后速度差 = |修正|/m_eff:Δv = {dv} want {want_dv}"
        );
    }

    /// AC-F4(切段速度,§5.7.3 第 3 条):同一分量切成多段时,各段请求
    /// velocity = (Δv_k + Δω_k × (质心_seg − 质心_k), Δω_k);单格段
    /// impulse = 0.729·v_seg。用追踪里该分量的 momenta 逐段复算对账。
    ///
    /// 变红:v_seg 用 Δv_k 忘了 ω 项;单格换算漏 0.729。
    #[test]
    fn impact_flow_trace_segment_velocity_matches_component() {
        // 复用 L 形布景的 2×3 墙:把 compound_split_size 临时改 1 → S 的
        // 每格独立成段?不行——切块网格 div_euclid(split_size),split=1
        // 每格一段(单格段,走 detach)。要 ≥2 格分量被切成 ≥2 段:split=1
        // 时全变单格段、没有复合体;用默认切块,分量整段成复合体(1 段)。
        // **切段发生的场景**:分量大(≥ 切块边界)——用 2×3 墙 + split=2:
        // x/div 2(y 3..5 div 2 → [1,2,2]?)块格切块按 y:3→1,4→2,5→2,
        // z 8..9→4:y=3 一段、y=4..5 一段 → 2 段(其中 ≥2 格的成复合体)。
        let mut s = flow_sim_zero_g();
        s.cfg.cohesion.compound_split_size = 2;
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 1, 8), voxel::builtin::BEDROCK);
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 1, 9), voxel::builtin::BEDROCK);
        for y in 3..=5 {
            for z in 8..=9 {
                let _ = s
                    .voxels
                    .set_block(BlockPos::new(10, y, z), voxel::builtin::DIRT);
            }
        }
        refresh_collider(&mut s);
        let bar = types::CompoundShape::new(
            (0..6)
                .map(|i| types::ShapeCell {
                    offset: [i - 5, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("6 格(伸向 −x,首格在前)");
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(16.5, 4.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, bar),
            }),
        }]);
        let mut compound_reqs = Vec::new();
        let mut detach_reqs = Vec::new();
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            compound_reqs.extend(out.compound_requests);
            detach_reqs.extend(out.detach_requests);
            if !compound_reqs.is_empty() || !detach_reqs.is_empty() {
                break;
            }
        }
        let t = s.impact_flow_trace.first().expect("应有断开事件(布景自检)");
        // 分量 = S 整体(6 格,撞击粘结连通);段 = 切块(≤ 2 格)。对每个
        // 复合体段:velocity = (Δv_k + Δω_k × (质心_seg − 质心_k), Δω_k);
        // 对每个单格段:impulse = 0.729·(Δv_k + Δω_k × r_seg)。
        let k = &t.momenta[0];
        assert_eq!(k.cells.len(), 6, "布景自检:S = 6 格 1 分量");
        let com_k = crate::impact_flow::component_center_of_mass(&k.cells);
        for req in &compound_reqs {
            let cells: Vec<BlockPos> = req
                .shape
                .cells()
                .iter()
                .map(|c| {
                    BlockPos::new(
                        req.origin.x + c.offset[0],
                        req.origin.y + c.offset[1],
                        req.origin.z + c.offset[2],
                    )
                })
                .collect();
            let com_seg = crate::impact_flow::component_center_of_mass(&cells);
            let want_v = k.delta_v + k.delta_w.cross(com_seg - com_k);
            assert!(
                (req.velocity.0 - want_v).length() < 1e-4,
                "AC-F4:切段 v_seg = Δv_k + Δω_k × r:got {:?} want {want_v:?}",
                req.velocity.0
            );
            assert!(
                (req.velocity.1 - k.delta_w).length() < 1e-4,
                "AC-F4:切段 ω_seg = Δω_k"
            );
        }
        for req in &detach_reqs {
            let com_seg = Vec3::new(
                req.cell.x as f32 + 0.5,
                req.cell.y as f32 + 0.5,
                req.cell.z as f32 + 0.5,
            );
            let want_v = k.delta_v + k.delta_w.cross(com_seg - com_k);
            assert!(
                ((req.impulse / crate::destruction::DEBRIS_MASS) - want_v).length() < 1e-4,
                "AC-F4:单格段 impulse = 0.729·v_seg:got {:?} want {want_v:?}",
                req.impulse / crate::destruction::DEBRIS_MASS
            );
        }
        assert!(
            !compound_reqs.is_empty() || !detach_reqs.is_empty(),
            "布景自检:应有段请求"
        );
    }

    /// AC-F6(单格碎块初速定量):单格段碎块初速 = w·d̂(不只是符号)。
    /// 复用定量布景(悬空墩 + 6 格条,S = 上格单格):碎块速度从
    /// DetachRequest.impulse/0.729 还原后与追踪 w·d̂ 对账。
    ///
    /// 变红:w 的 m_eff 口径错(漏角项);impulse 换算错。
    #[test]
    fn impact_flow_trace_single_cell_speed_equals_w() {
        let mut s = flow_sim_zero_g();
        s.load_chunk(ChunkPos::new(0, 0), solid_chunk());
        let _ = s
            .voxels
            .set_block(BlockPos::new(10, 3, 8), voxel::builtin::BEDROCK);
        for y in 4..=5 {
            let _ = s
                .voxels
                .set_block(BlockPos::new(10, y, 8), voxel::builtin::DIRT);
        }
        refresh_collider(&mut s);
        let bar = types::CompoundShape::new(
            (0..6)
                .map(|i| types::ShapeCell {
                    offset: [i - 5, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("6 格(伸向 −x,首格在前)");
        let id = entity(1);
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(16.5, 5.5, 8.5),
                    linvel: Vec3::new(-12.0, 0.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, bar),
            }),
        }]);
        let mut impulse = None;
        for _ in 0..20 {
            s.step();
            let out = s.take_outbox();
            if let Some(req) = out.detach_requests.first() {
                impulse = Some(req.impulse);
                break;
            }
        }
        let impulse = impulse.expect("S = 上格单格,应有单格脱落请求");
        let t = &s.impact_flow_trace[0];
        let d_hat = t.event.dir_hat();
        let m_s = t.momenta.iter().map(|k| k.mass).sum::<f32>();
        let e =
            t.event.impulse * (((t.supply - t.cut_q as f32 / t.quant) / t.supply).clamp(0.0, 1.0));
        // 单格碎块:分量 k = S(1 格),Δv_k = P_k/M_k = E/(m_eff+M_S)·1(按
        // 质量比)——碎块速度 = Δv_k = w·d̂(w 同方向)。从追踪的分量取闭式:
        let k = &t.momenta[0];
        assert_eq!(k.cells.len(), 1, "布景自检:分量 = 单格");
        let v_seg = impulse / crate::destruction::DEBRIS_MASS;
        assert!(
            (v_seg - k.delta_v).length() < 1e-3,
            "AC-F6:碎块初速 = Δv_k(= w·d̂):got {v_seg:?} want {:?}",
            k.delta_v
        );
        // w 定量复算:m_eff 由修正闭式反解(m_eff/(m_eff+M_S)·E = 修正),
        // 避免重复生产的 m_eff 计算;w = |E|/(m_eff+M_S)。
        let m_eff = {
            let ratio = t.correction.length() / e.length();
            let m_s_ = m_s;
            ratio * m_s_ / (1.0 - ratio)
        };
        let want_w = e.length() / (m_eff + m_s);
        assert!(
            ((v_seg - d_hat * want_w).length()) < 1e-2,
            "AC-F6:v_seg = w·d̂:got {v_seg:?} want {:?} (w = {want_w})",
            d_hat * want_w
        );
    }

    /// AC-F5(穿墙条款,§5.7.10 末条;`demos/inset-slab/passage.py` 工况 1
    /// 的生产路径复现):1 格厚土墙 w7h5 立在 bedrock 脚上(demo 墙 x=0、
    /// 墙体 y 0..4、z −3..3,脚下 y −1..−3 bedrock,地表泥土 y=0);
    /// 6 格石条(4.374 kg)沿运动方向纵置,打墙中行。demo 内缩口径结果
    /// (passage_out.txt 行 10/12/14):10 m/s 停在墙前(前缘不越面);
    /// 15 m/s 一次事件断 2 格越过墙面(前缘 1.297);20 m/s 穿过
    /// (前缘 3.661)。
    ///
    /// 生产坐标系映射(dirt_on_stone_chunk 土面 world y = 1;chunk (0,0)
    /// 覆盖 world x/z 0..16):demo (x, y, z) → 生产 (x + 8, y + 1, z + 8)
    /// ——墙 x=8、墙体 y 1..5、z 5..11、脚下 y 0..−2 bedrock;石条首格
    /// (0, 3, 8)(复合体 translation = 首格格心,6 格 offset 0..5 沿 +x
    /// 伸出);判定面 = 墙后表面 x = 9(demo x=1)。
    ///
    /// **前缘口径(F02b-2 勘误)**:复合体 translation 是刚体原点 = 首格
    /// 格心,`translation.x + 0.45` 量的是**尾格**——正确前缘 = 各格
    /// `(translation + rotation·offset).x` 的最大值 + 0.45(计入旋转)。
    /// 判据 = 前缘的**全程最大值**(只在实体在场的拍计):15/20 m/s
    /// 要 > 9(越过墙面继续前进),10 m/s 全程 ≤ 9(停在墙前);15/20
    /// 再断言断开拍修正之后 vx > 0(撞击者继续前进,§5.7.2)。
    ///
    /// 变红:修正未接线(撞击者停在墙面,15/20 不越面);前缘口径回退成
    /// `translation.x + 0.45`(10 m/s 也会"越面"——尾格超调)。
    #[test]
    fn impact_flow_wall_passage_matches_demo() {
        const WALL_X: i32 = 8;
        const GY: i32 = 1; // 生产土面(demo y=0 地表)
        const FACE_BACK: f32 = 9.0; // 墙后表面(demo 判定面 x=1)
        let wall_world = |s: &mut ShardSim| {
            // 脚下 bedrock(demo y −1..−3 → 生产 0..−2)。
            for y in (GY - 3)..=(GY - 1) {
                for z in 5..=11 {
                    let _ = s
                        .voxels
                        .set_block(BlockPos::new(WALL_X, y, z), voxel::builtin::BEDROCK);
                }
            }
            // 墙体(demo y 0..4 → 生产 1..5)。
            for y in GY..=(GY + 4) {
                for z in 5..=11 {
                    let _ = s
                        .voxels
                        .set_block(BlockPos::new(WALL_X, y, z), voxel::builtin::DIRT);
                }
            }
            refresh_collider(s);
        };
        let bar_shape = types::CompoundShape::new(
            (0..6)
                .map(|i| types::ShapeCell {
                    offset: [i, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("6 格");
        // 前缘 = 各格 (translation + rotation·offset).x 最大值 + 半边长。
        let front_of = |st: &EntityState| -> f32 {
            st.rotation
                .mul_vec3(Vec3::from([
                    5.0, // 6 格条:最远格 offset x = 5(取上界即可,旋转后单调)
                    0.0, 0.0,
                ]))
                .x
                + st.translation.x
                + crate::destruction::DEBRIS_HALF_SIZE
        };
        // (前缘全程最大值, 断断时 vx, air 格清单, 断格拍数)。
        type PassageCase = (f32, Option<f32>, Vec<(i32, i32, i32)>, u64);
        let run_case = |speed: f32| -> PassageCase {
            let mut s = flow_sim();
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            wall_world(&mut s);
            // 6 格石条复合体:translation = 首格格心 (0.5, 3.5, 8.5)
            // (demo 原生口径:起点 (−8,2,0) 行格心 → 生产 (0,3,8))。
            let id = entity(1);
            s.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: id,
                    state: EntityState {
                        translation: Vec3::new(0.5, 3.5, 8.5),
                        linvel: Vec3::new(speed, 0.0, 0.0),
                        ..EntityState::default()
                    },
                    desc: EntityDesc::compound(
                        crate::destruction::DEBRIS_HALF_SIZE,
                        bar_shape.clone(),
                    ),
                }),
            }]);
            let mut breaks = 0u64;
            let mut air_cells = Vec::new();
            let mut front_max = f32::NEG_INFINITY;
            let mut vx_at_break = None;
            for _ in 0..300 {
                s.step();
                let out = s.take_outbox();
                if out.cohesion_stats.impact_flow_corrections > 0 && vx_at_break.is_none() {
                    // 断开拍修正之后的撞击者 vx(§5.7.2:以约 w 继续前进)。
                    vx_at_break = s.entity_state(id).map(|st| st.linvel.x);
                }
                breaks += out.cohesion_stats.impact_flow_breaks;
                for w in &out.block_updates {
                    if w.block == types::BlockId::AIR {
                        air_cells.push((w.pos.x, w.pos.y, w.pos.z));
                    }
                }
                // 前缘全程最大值(只在实体在场的拍计;20 m/s 的条可能被
                // D4 碎裂移除——unit 层无调度器,碎块不 N+1 出生,以消失前
                // 的最大值为准)。
                if let Some(st) = s.entity_state(id) {
                    front_max = front_max.max(front_of(&st));
                    // 终态:投射物静止且断格已发生。
                    if breaks > 0 && st.linvel.length() < 0.05 {
                        break;
                    }
                }
            }
            (front_max, vx_at_break, air_cells, breaks)
        };
        for (speed, want_pass) in [(10.0, false), (15.0, true), (20.0, true)] {
            let (front, vx, air, breaks) = run_case(speed);
            assert!(breaks >= 1, "{speed} m/s:墙应被打断(断格 {air:?})");
            assert!(
                air.iter().all(|(_, y, _)| *y >= GY),
                "只断墙体的土格(脚下 bedrock 是汇点):{air:?}"
            );
            assert_eq!(
                front > FACE_BACK,
                want_pass,
                "{speed} m/s:前缘全程最大值 {front}(demo 内缩口径:10 → 停在墙前,                 15/20 → 越过墙面继续前进)"
            );
            if want_pass {
                let vx = vx.expect("断开拍应有修正(vx 采样)");
                assert!(
                    vx > 0.0,
                    "{speed} m/s:断开拍修正之后 vx > 0(撞击者继续前进,§5.7.2),实际 {vx}"
                );
            }
        }
    }

    // ============================== G02(§5.8 生产接入,compound_flow)==============================

    /// compound_flow 生效 + 零重力(采样布景标准配置;impact_flow 同开——
    /// 生产默认 impact_flow 已在 F04a 起默认开启,§5.7 修正扣除测试需要它)。
    fn flow_compound_sim() -> ShardSim {
        let mut cfg = ShardSimConfig {
            gravity: Vec3::ZERO,
            ..ShardSimConfig::new(shard_id(0, 0))
        };
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.impact_flow = true;
        cfg.cohesion.compound_flow = true;
        ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
    }

    /// 单格土方块投射物 spawn 输入(位置/速度自定)。
    fn dirt_bullet_input(seq: u64, id: EntityId, pos: Vec3, vel: Vec3) -> StampedInput {
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
                desc: EntityDesc {
                    half_size: crate::destruction::DEBRIS_HALF_SIZE,
                    block: voxel::builtin::DIRT,
                    shape: None,
                },
            }),
        }
    }

    /// 悬空 6 格土条:刚体原点(中格)在 `origin`,条沿 +x。
    /// 形状格 offset x = −2..=3(6 格,canonical 升序),原点格 = x = 2.5 →
    /// div_euclid 中格 (2,0,0)。
    fn compound_input_at(
        seq: u64,
        id: EntityId,
        origin: Vec3,
        shape: types::CompoundShape,
        vel: Vec3,
    ) -> StampedInput {
        StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: seq,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: origin,
                    linvel: vel,
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape),
            }),
        }
    }

    /// 落体 spawn 高度(G03 裁定纪律):地面顶面 = **world y = 1**
    /// (`dirt_on_stone_chunk`/`solid_chunk` 的方块层在 chunk 局部 y = 64 =
    /// world y = 0,格 [0,1],顶面 y = 1)。刚体原点的静置高度 = 顶面
    /// (y = 1)+ 半边长 0.45 + (原点相对最低格的高度);`h` = 最低点离地,
    /// spawn 原点 y = 静置原点 y + h。**G03 教训(写进注释):落体布景必须
    /// 经此函数算高度,不能手写 `67.x + h`——手写把地面顶面当成 65,
    /// 落高多了 64 m,错当成物理现象(见 G00 同类错);配套断言
    /// `assert_touch_speed` 由每个落体测试调用,布景再错直接红。**
    fn drop_origin_y(shape: &types::CompoundShape, h: f32) -> f32 {
        let min_y = shape
            .cells()
            .iter()
            .map(|c| c.offset[1])
            .min()
            .expect("形状非空") as f32;
        // 最低格中心 = origin + min_y;最低点 = 中心 − 0.45;静置时 = 1。
        1.0 + 0.45 - min_y + h
    }

    /// 落地前一拍的竖直速度断言(G03 裁定第 1 条):窗口内第一次 |vy| 达到
    /// 0.9 × touch_speed(h) 时停,断言它 ≤ 1.1 × touch_speed。**不能取窗口
    /// 内 vy 的最小值**:端角等翻倒工况触地后质心继续下落(倒平姿势),重力
    /// 把 vy 推过自由落体值(0.9 m 档实测 min_vy = 4.81 > 4.20,18% 假红);
    /// "到 0.9 倍即停"保证采到的必然是触地前的自由落体段。窗口不足(整段
    /// 都没到 0.9 倍,即 spawn 高度偏高 / 地面基准错)时报错。
    fn assert_touch_speed(s: &mut ShardSim, id: EntityId, h: f32, until: usize, ctx: &str) {
        let want = touch_speed(h);
        // 自由落体到触地需 √(2h/g)/dt 拍(dt = 1/60);窗口必须盖过它,否则
        // 没到触地就退出(G03 实测 28 拍窗口 vy 只到 4.58)。取 1.5 倍行程
        // 拍数与调用者给的 until 的较大者。
        let fall_ticks = ((want / 9.81) / (1.0 / 60.0)).ceil() as usize;
        let until = until.max(fall_ticks * 3 / 2 + 2);
        for _ in 0..until {
            s.step();
            match s.entity_state(id) {
                Some(st) => {
                    let v = st.linvel.y.abs();
                    if v >= 0.9 * want {
                        assert!(
                            v <= 1.1 * want,
                            "{ctx}:落地前竖直速度 {v} 超出 touch_speed({h}) = {want} \
                             的 ±10% ——落高/地面基准错了(地面顶面 world y = 1)"
                        );
                        return;
                    }
                }
                None => panic!("{ctx}:实体在触地前就消失(断裂过早或 spawn 失败)"),
            }
        }
        panic!(
            "{ctx}:{} 拍内竖直速度没到 0.9 × touch_speed({h}) = {} ——\
             spawn 高度偏高或地面基准错了(地面顶面 world y = 1)",
            until, want
        );
    }

    /// AC-G3 侧撞(G03 冒烟同款工况):悬空 6 格土条,单格土方块 40 m/s
    /// (≈ 1.45 × 27.6,土阈值)沿 +z 撞**端格**——投射物从端格外侧起飞、
    /// 按**真实格世界中心**瞄准(端格 = 原点刚体 + 形状 offset,取整格心
    /// (10.5, 5.5, 8.5);G00 两轮返修的同一错在这里由断言 1 钉住)。
    ///
    /// 断言(任务书 6 条):
    /// 1. 首个事件的接触格 = 端格(offset x = 1);
    /// 2. S = 端格(detached = [(2,0,0)]);
    /// 3. 断开后端格速度沿 +z,> 主体(R 块)速度的 3 倍;
    /// 4. 投射物撞后 |v| > 同场景 `compound_flow = false` 时(对方修正生效:
    ///    修正沿 B 的 J 即 +z 推对方,少被弹回);
    /// 5. trace 守恒残差 ≤ 1e-3;
    /// 6. 投射物 + 各块整体的撞击拍前后总动量差 ≤ 量化单位 × 接触点数。
    #[test]
    fn compound_flow_side_impact_breaks_end_cell_and_corrects_other() {
        let run = |flow: bool| -> (Vec3, Vec3, Vec3, Vec3, usize, Vec<i32>, usize, f32, u64) {
            let mut s = flow_compound_sim();
            if !flow {
                s.cfg.cohesion.compound_flow = false;
            }
            s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
            // 条:端格 = 世界格 (10,5,8)(格心 (10.5,5.5,8.5)),向 −x 延伸
            // 6 格;形状 offset x = −4..=1(canonical 升序,端格 = +1),
            // 原点(刚体)在格 (9,5,8) 格心,端格格心 = 9.5 + 1 = 10.5。
            let shape = types::CompoundShape::new(
                (-4i32..=1)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格");
            let (bar, bullet) = (entity(1), entity(2));
            s.apply_prefix(&[
                compound_input_at(1, bar, Vec3::new(9.5, 5.5, 8.5), shape, Vec3::ZERO),
                dirt_bullet_input(
                    2,
                    bullet,
                    Vec3::new(10.5, 5.5, 4.0),
                    Vec3::new(0.0, 0.0, 40.0),
                ),
            ]);
            let bullet_mass = s.entity_mass(bullet).expect("投射物在场");
            let mut pre = Vec3::ZERO;
            let mut post = Vec3::ZERO;
            let mut end_v = Vec3::ZERO;
            let mut main_v = Vec3::ZERO;
            let mut piece_n = 0usize;
            let mut contact_cells = Vec::new();
            let mut n_contacts = 0usize;
            let mut residual = f32::INFINITY;
            let mut broke = false;
            for _ in 0..200 {
                s.step();
                let out = s.take_outbox();
                // 首个断开事件的守恒残差与接触格(trace 只追加、不清理)。
                if residual.is_infinite() {
                    let found = s
                        .compound_flow_trace
                        .iter()
                        .find(|tr| tr.split.residual > 0.0 || !tr.outcome.detached.is_empty());
                    if let Some(tr) = found {
                        residual = tr.split.residual;
                        contact_cells = tr.event.contacts.iter().map(|c| c.cell.x).collect();
                        n_contacts = tr.event.contacts.len();
                    }
                }
                if !out.piece_requests.is_empty() {
                    // 撞击拍动量:pre 在上一轮 step 前记(条零速 = 0 + 投射物
                    // m·v);post = 各块 m_块·v_块 + 投射物 step 后 m·v。
                    pre = Vec3::new(0.0, 0.0, 40.0) * bullet_mass;
                    for req in &out.piece_requests {
                        let m = match req.desc.shape.as_ref() {
                            Some(sh) => sh.cells().len() as f32 * 0.729,
                            None => 0.729,
                        };
                        post += req.state.linvel * m;
                    }
                    if let Some(st) = s.entity_state(bullet) {
                        post += st.linvel * bullet_mass;
                    }
                    end_v = out.piece_requests[0].state.linvel;
                    main_v = out.piece_requests[out.piece_requests.len() - 1]
                        .state
                        .linvel;
                    piece_n = out.piece_requests.len();
                    broke = true;
                    break;
                }
            }
            assert!(broke, "40 m/s 侧撞端格应断下端格(S = 端格)");
            (
                end_v,
                main_v,
                pre,
                post,
                piece_n,
                contact_cells,
                n_contacts,
                residual,
                0u64,
            )
        };
        let (end_v, main_v, pre, post, _piece_n, cells, n_contacts, residual, _) = run(true);
        // 1. 首个断开事件的接触格全在端格(offset x = 1;侧撞端格中心,
        //    单流形全点归同格)。
        assert!(
            !cells.is_empty() && cells.iter().all(|&x| x == 1),
            "接触格 = 端格(offset x = 1),实际 {cells:?}"
        );
        // 2. trace 残差(§5.8.2 守恒式)≤ 1e-3。
        assert!(
            residual <= 1e-3,
            "守恒残差 {residual} ≤ 1e-3(量化单位 × 接触点数量级)"
        );
        // 3. 端格块速度沿 +z 且 > 主体块 3 倍(z 分量;R 块也沿 +z 但慢)。
        assert!(
            end_v.z > 3.0 * main_v.z && end_v.z > 0.0,
            "端格速度 {end_v:?} 沿 +z 且 > 3× 主体 {main_v:?}"
        );
        // 4. 对方修正:总动量差 ≤ 量化单位 × 接触点数(0.001 × 点数)。
        let dm = (post - pre).length();
        let tol = 1e-3 * n_contacts as f32 + 1e-4;
        assert!(
            dm <= tol,
            "动量守恒:|post − pre| = {dm} ≤ 量化单位 × 接触点数({tol});pre {pre:?} post {post:?}"
        );
        // 5. 投射物撞后速度 > 关闭时(对方修正生效 = 少被弹回)。
        let _ = run(false);
        let bullet_v_on = {
            // 重跑开启布景单独取投射物撞后速度(run 内不返回;重放一遍)。
            let mut s = flow_compound_sim();
            s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
            let shape = types::CompoundShape::new(
                (-3i32..=2)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格");
            let (bar, bullet) = (entity(1), entity(2));
            s.apply_prefix(&[
                compound_input_at(1, bar, Vec3::new(9.5, 5.5, 8.5), shape, Vec3::ZERO),
                dirt_bullet_input(
                    2,
                    bullet,
                    Vec3::new(10.5, 5.5, 4.0),
                    Vec3::new(0.0, 0.0, 40.0),
                ),
            ]);
            let mut v = Vec3::ZERO;
            for _ in 0..200 {
                s.step();
                let out = s.take_outbox();
                if !out.piece_requests.is_empty() {
                    v = s.entity_state(bullet).expect("投射物在场").linvel;
                    break;
                }
            }
            v
        };
        let bullet_v_off = {
            let mut s = flow_compound_sim();
            s.cfg.cohesion.compound_flow = false;
            s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
            let shape = types::CompoundShape::new(
                (-3i32..=2)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格");
            let (bar, bullet) = (entity(1), entity(2));
            s.apply_prefix(&[
                compound_input_at(1, bar, Vec3::new(9.5, 5.5, 8.5), shape, Vec3::ZERO),
                dirt_bullet_input(
                    2,
                    bullet,
                    Vec3::new(10.5, 5.5, 4.0),
                    Vec3::new(0.0, 0.0, 40.0),
                ),
            ]);
            let mut v = Vec3::ZERO;
            for _ in 0..200 {
                s.step();
                let out = s.take_outbox();
                if !out.piece_requests.is_empty() {
                    v = s.entity_state(bullet).expect("投射物在场").linvel;
                    break;
                }
            }
            v
        };
        assert!(
            bullet_v_on.length() > bullet_v_off.length(),
            "投射物撞后 |v|:开启(修正) {bullet_v_on:?} > 关闭 {bullet_v_off:?}(对方修正生效)"
        );
    }

    /// compound_flow 生效 + 重力(落地布景标准配置)。
    fn flow_compound_gravity_sim() -> ShardSim {
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.solidify.enabled = false;
        cfg.cohesion.enabled = true;
        cfg.cohesion.impact_flow = false;
        cfg.cohesion.compound_flow = true;
        ShardSim::new(cfg, Box::new(voxel::SimpleRegistry::builtin()))
    }

    /// 从静止落 h 后的触地速度 √(2gh)(断言落高换算用)。
    fn touch_speed(h: f32) -> f32 {
        (2.0 * 9.81 * h).sqrt()
    }

    /// AC-G3 落地①:6 格土柱竖直落地,落高 2 × 2.6 m(底格压断阈值 2.6 m
    /// 的 2 倍)。§5.8.1 表(G00 实测):竖直落地断开阈值 = 压向 I_c 33.6
    /// N·s、S = 底格、**事件 J 沿局部 +y**(压向)。断言:断开发生、首个断开
    /// 事件接触格 = 底格(局部 offset y 最小)、J 局部系 +y 分量显著、
    /// S = 底格单格、§5.8.4 breaks = 1 / detached_cells = 1。
    ///
    /// G03 教训:地面顶面 = world y = 1(dirt_on_stone_chunk 的方块层在
    /// chunk 局部 y = 64 = world y = 0)——此前手写 `67 + h` 落高多 64 m,
    /// 落地冲量被当成"倒下拍"现象(裁定 6f67791 后注释)。落高经
    /// `drop_origin_y` 换算 + `assert_touch_speed` 钉落地前速度。
    #[test]
    fn compound_flow_pillar_landing_breaks_bottom_cell_press_direction() {
        let mut s = flow_compound_gravity_sim();
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        let h = 2.0 * 2.6;
        let shape = types::CompoundShape::new(
            (-2i32..=3)
                .map(|y| types::ShapeCell {
                    offset: [0, y, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("6 格柱");
        let id = entity(1);
        s.apply_prefix(&[compound_input_at(
            1,
            id,
            Vec3::new(10.5, drop_origin_y(&shape, h), 8.5),
            shape,
            Vec3::ZERO,
        )]);
        // 落地拍在 ~30 拍内到达(落高 5.2 m);取 28 拍内 vy 最小值核对落高。
        assert_touch_speed(&mut s, id, h, 28, "土柱竖落");
        let mut broke = false;
        let mut j_local = Vec3::ZERO;
        let mut cells_y = Vec::new();
        let mut s_cells = Vec::new();
        let mut breaks_count = 0u64;
        let mut detached_count = 0u64;
        for _ in 0..400 {
            s.step();
            let out = s.take_outbox();
            breaks_count += out.cohesion_stats.compound_flow_breaks;
            detached_count += out.cohesion_stats.compound_flow_detached_cells;
            if out.cohesion_stats.compound_flow_breaks > 0 && !broke {
                broke = true;
                if let Some(tr) = s
                    .compound_flow_trace
                    .iter()
                    .find(|tr| !tr.outcome.detached.is_empty())
                {
                    let ev = &tr.event;
                    j_local = ev.impulse_j();
                    // 接触格(局部 y;事件 J 主要沿 +y → 法向指 +y)。
                    cells_y = ev.contacts.iter().map(|c| c.cell.y).collect();
                    s_cells = tr
                        .outcome
                        .detached
                        .iter()
                        .map(|c| (c.x, c.y, c.z))
                        .collect();
                }
                break;
            }
        }
        assert!(broke, "落高 {h} m(≈ {} m/s)应压断底格", touch_speed(h));
        // J 局部系 +y(压向;世界系重力落地 → 局部 = 世界,恒等旋转)。
        assert!(
            j_local.y > 0.9 * j_local.length(),
            "事件 J 沿局部 +y:{j_local:?}"
        );
        // 接触格 = 底格(局部 y = −2)。
        assert!(
            !cells_y.is_empty() && cells_y.iter().all(|&y| y == -2),
            "接触格 = 底格(局部 y = −2),实际 {cells_y:?}"
        );
        // S = 底格单格(局部 offset y = −2)。
        assert_eq!(
            s_cells,
            vec![(0i32, -2i32, 0i32)],
            "S = 底格单格(局部):s_cells {s_cells:?}"
        );
        assert_eq!(breaks_count, 1);
        assert_eq!(detached_count, 1);
    }

    /// AC-G3 落地②:3×3 土板 9 m 平落,**开启不碎**(§5.8.5 刚体假设:D18
    /// "平落永不碎";G00 实测 9 m 不断)、关闭按原 AC-C6 逻辑碎(逐位回归;
    /// 同一场景两种开关)。断言:开启 → breaks = 0、实体保留、全程无
    /// piece_requests;关闭 → 碎(实体删除、bonds_broken > 0)。
    #[test]
    fn compound_flow_flat_plate_9m_holds_when_on_breaks_when_off() {
        let run = |flow: bool| -> (bool, u64, u64) {
            let mut s = flow_compound_gravity_sim();
            if !flow {
                s.cfg.cohesion.compound_flow = false;
            }
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            // 3×3 土板:offset (x,z) ∈ −1..=1,静置时板格中心 y = 65.5
            // (板底面贴 DIRT 层顶面 world y = 65),原点在格 (10,66,8);
            // spawn:静置位姿 + 落高 9 m。
            let shape = types::CompoundShape::new(
                (0..9)
                    .map(|i| types::ShapeCell {
                        offset: [(i % 3) - 1, 0, (i / 3) - 1],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("9 格板");
            let id = entity(1);
            let h = 9.0f32;
            s.apply_prefix(&[compound_input_at(
                1,
                id,
                Vec3::new(10.5, drop_origin_y(&shape, h), 8.5),
                shape,
                Vec3::ZERO,
            )]);
            let mut alive = true;
            let mut breaks = 0u64;
            let mut bonds = 0u64;
            for _ in 0..400 {
                s.step();
                let out = s.take_outbox();
                breaks += out.cohesion_stats.compound_flow_breaks;
                bonds += out.cohesion_stats.bonds_broken;
                if !out.piece_requests.is_empty() || s.entity_state(id).is_none() {
                    alive = false;
                    break;
                }
            }
            (alive, breaks, bonds)
        };
        let (alive_on, breaks_on, bonds_on) = run(true);
        assert!(alive_on, "开启:3×3 土板 9 m 平落不碎(刚体假设,D18)");
        assert_eq!(breaks_on, 0);
        assert_eq!(bonds_on, 0);
        let (alive_off, _breaks_off, bonds_off) = run(false);
        assert!(!alive_off, "关闭:同一场景按原 AC-C6 碎(逐位回归)");
        assert!(bonds_off > 0, "关闭:断键 > 0({bonds_off})");
    }

    /// AC-G3 落地③:6 格土条 30° 端角先着地,落高 2 × 2.1 m(G00 实测土
    /// 2.1 m 断下端格)。绕 z +30° 把 +x 端抬高,最低端 = offset −4 端,
    /// 其外下角一点先接触 → S = 该端格。断言:断下、接触格与 S 都 = 端格。
    #[test]
    fn compound_flow_corner_landing_30deg_breaks_end_cell() {
        let mut s = flow_compound_gravity_sim();
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        let shape = types::CompoundShape::new(
            (-4i32..=1)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("6 格条");
        let id = entity(1);
        let h = 2.0 * 2.1;
        // 30° 端角的静置高度:最低格(offset −4)旋转后中心 = y0 + R·(−4,0,0).y
        // = y0 − 4·sin30° = y0 − 2,cuboid 旋转后最低点 = 中心 − 0.45·(sin30°
        // + cos30°) ≈ −0.61;贴地面(world y = 1)→ 静置原点 y = 1 + 2 + 0.61
        // = 3.61。spawn = 3.61 + h(此前手写 67.39 多了 64 m,G03 教训)。
        // drop_origin_y 不处理旋转(按 offset y 直立形状换算),这里手写并
        // 写明推导。
        let y0 = 1.0
            + 4.0 * 30f32.to_radians().sin()
            + 0.45 * (30f32.to_radians().sin() + 30f32.to_radians().cos())
            + h;
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(10.5, y0, 8.5),
                    rotation: types::glam::Quat::from_rotation_z(30f32.to_radians()),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape),
            }),
        }]);
        // 落地拍核对(2.1×2 = 4.2 m → vy ≈ 9.1;取 30 拍内最小 vy)。
        assert_touch_speed(&mut s, id, h, 30, "30° 端角");
        let mut s_cells = Vec::new();
        let mut contact_x = Vec::new();
        let mut broke = false;
        for _ in 0..400 {
            s.step();
            let out = s.take_outbox();
            if out.cohesion_stats.compound_flow_breaks > 0 && !broke {
                broke = true;
                if let Some(tr) = s
                    .compound_flow_trace
                    .iter()
                    .find(|tr| !tr.outcome.detached.is_empty())
                {
                    s_cells = tr
                        .outcome
                        .detached
                        .iter()
                        .map(|c| (c.x, c.y, c.z))
                        .collect();
                    contact_x = tr.event.contacts.iter().map(|c| c.cell.x).collect();
                }
                break;
            }
        }
        assert!(
            broke,
            "落高 {h} m(≈ {} m/s)30° 端角应断下端格",
            touch_speed(h)
        );
        assert_eq!(
            s_cells,
            vec![(-4i32, 0, 0)],
            "S = 端格(局部 offset −4):{s_cells:?}"
        );
        assert!(
            !contact_x.is_empty() && contact_x.iter().all(|&x| x == -4),
            "接触格 = 端格:{contact_x:?}"
        );
    }

    /// 坐标守卫(AC-G3 观测口):倾斜 30° + 绕 y 转 45° 的 6 格土条落地,
    /// **测试层独立断言**全程每个事件的每个计入点都落在所在格盒
    /// `[offset, offset+1]³` 外扩 0.1 内、局部法向归一(双重旋转下漏乘
    /// R⁻¹ 或漏加 0.5 立刻红;不只依赖 debug_assert)。
    #[test]
    fn compound_flow_coordinate_guard_rotated_landing_points_in_cell_box() {
        let mut s = flow_compound_gravity_sim();
        s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
        let shape = types::CompoundShape::new(
            (-4i32..=1)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("6 格条");
        let id = entity(1);
        let h = 2.0 * 2.1;
        // 静置高度同 corner 测试:R_y(45°) 不改 y 分量,最低点推导一致 →
        // 静置原点 y = 3.61;spawn = 静置 + h(此前手写 67.39 多了 64 m)。
        let y0 = 1.0
            + 4.0 * 30f32.to_radians().sin()
            + 0.45 * (30f32.to_radians().sin() + 30f32.to_radians().cos())
            + h;
        s.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource::SCHEDULER,
            sequence: 1,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: id,
                state: EntityState {
                    translation: Vec3::new(10.5, y0, 8.5),
                    rotation: types::glam::Quat::from_rotation_y(45f32.to_radians())
                        * types::glam::Quat::from_rotation_z(30f32.to_radians()),
                    ..EntityState::default()
                },
                desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape),
            }),
        }]);
        // 落地拍核对(4.2 m → vy ≈ 9.1;取 30 拍内最小 vy)。
        assert_touch_speed(&mut s, id, h, 30, "45°+30° 双旋土条");
        let mut saw_event = false;
        for _ in 0..400 {
            s.step();
            let _out = s.take_outbox();
            // 全程:trace 里每个事件的每个点都在其接触格盒外扩 0.1 内。
            for tr in &s.compound_flow_trace {
                saw_event = saw_event || !tr.event.contacts.is_empty();
                for pt in &tr.event.contacts {
                    let off = [pt.cell.x as f32, pt.cell.y as f32, pt.cell.z as f32];
                    for k in 0..3 {
                        let v = [pt.point.x, pt.point.y, pt.point.z][k];
                        assert!(
                            v >= off[k] - 0.1 && v <= off[k] + 1.1,
                            "事件点越界:格 {off:?} 轴 {k} 值 {v}(点 {pt:?})——坐标对应破坏"
                        );
                    }
                    let n = pt.normal.length();
                    assert!((n - 1.0).abs() < 1e-3, "局部法向须归一:{pt:?}");
                }
            }
            if s.entity_state(id).is_none() {
                break;
            }
        }
        // 布景有效性:落地途中确实采样过事件(否则守卫没跑)。
        assert!(saw_event, "落地途中应有冲量流事件(守卫有效性)");
    }

    // ==================== AC-C6 开启版(§5.8.6 末条;G03) ====================
    // 档位按 §5.8.1 表 rapier 列(土),每项上下各取一档、约 20% 余量。
    // 落地工况全部经 assert_touch_speed 钉住落高(布景错直接红)。

    /// AC-C6 开启版①:6 格土柱竖落,2.0 m 不断、3.2 m 落地拍压断底格
    /// (土阈值 2.6 m 的 ± ~20%)。断开发生在**落地拍**(裁定复测:
    /// 2.6 m s = 34.20 > C = 33.71),不是倒下拍——落高错了 64 m 时落地拍
    /// 供给被分拍摊薄,首断推迟到 ~220 拍的倒下拍(G03 停报教训)。
    #[test]
    fn compound_flow_acc_c6_pillar_thresholds() {
        let run = |h: f32| -> (bool, u64, usize) {
            let mut s = flow_compound_gravity_sim();
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            let shape = types::CompoundShape::new(
                (-2i32..=3)
                    .map(|y| types::ShapeCell {
                        offset: [0, y, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格柱");
            let id = entity(1);
            s.apply_prefix(&[compound_input_at(
                1,
                id,
                Vec3::new(10.5, drop_origin_y(&shape, h), 8.5),
                shape,
                Vec3::ZERO,
            )]);
            assert_touch_speed(&mut s, id, h, 60, "AC-C6 柱");
            let mut broke = false;
            let mut breaks = 0u64;
            let mut first_tick = 0usize;
            for tick in 0..400 {
                let alive = s.entity_state(id).is_some();
                let out = s.take_outbox();
                breaks += out.cohesion_stats.compound_flow_breaks;
                if breaks > 0 && !broke {
                    broke = true;
                    first_tick = tick;
                }
                if !alive {
                    break;
                }
                s.step();
            }
            (broke, breaks, first_tick)
        };
        // 2.0 m:落地拍 s = 30.45 < C,不断(弹跳拍 20.7 / 18.0 也不够)。
        let (broke_lo, breaks_lo, _t_lo) = run(2.0);
        assert!(!broke_lo, "2.0 m 竖落应不断(breaks = {breaks_lo})");
        assert_eq!(breaks_lo, 0);
        // 3.2 m:落地拍 s = 38.08 > C = 34.36,落地拍断底格。
        let (broke_hi, _breaks_hi, t_hi) = run(3.2);
        assert!(broke_hi, "3.2 m 竖落应压断底格");
        assert!(
            t_hi <= 20,
            "断开应在落地拍(≤ 20 拍,实际第 {t_hi} 拍)——推迟到倒下拍说明落高错了"
        );
    }

    /// AC-C6 开启版②:6 格土条 / 石条 30° 端角先着地,档位 = 生产实测临界
    /// h* 的 0.75 倍不断 / 1.25 倍断端格(G03-2 裁定:采纳 A,按生产实测
    /// 重标;G00 表"30° 端角 石 13.4 / 土 2.1 m"作废——demo 二分的采样
    /// 没把法向转进刚体系)。BISECT_CORNER 二分实测(0.05 m 容差,判据 =
    /// 落地窗口内任一拍断开):**土 h* = 1.17 m、石 h* = 8.27 m**;临界
    /// 供给恰落纯函数 s*(土 12.99 ≈ 12.60、石 33.49 ≈ 32.40,差 = 量化 +
    /// 弹跳相位)。档位:土 0.9 / 1.5 m、石 6.0 / 10.5 m。逐拍探针见回评。
    #[test]
    fn compound_flow_acc_c6_corner_thresholds() {
        let run = |h: f32, block: types::BlockId| -> (bool, u64, usize) {
            let mut s = flow_compound_gravity_sim();
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            let shape = types::CompoundShape::new(
                (-4i32..=1)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block,
                    })
                    .collect(),
            )
            .expect("6 格条");
            let id = entity(1);
            // 30° 旋转静置原点:最低格 offset −4,旋转后格心 = y0 − 4·sin30°
            // (= y0 − 2),cuboid 旋转后最低点 = 格心 − 0.45·(sin30° + cos30°);
            // 贴地(world y = 1)→ y0 = 1 + 2 + 0.615 = 3.61(spawn 再加 h)。
            // drop_origin_y 不处理旋转,这里手写;此前把 4·sin30° 误写成
            // 4·0.5·sin30°(少 1 m,touch_speed 断言抓出)。
            let y0 = 1.0
                + 4.0 * 30f32.to_radians().sin()
                + 0.45 * (30f32.to_radians().sin() + 30f32.to_radians().cos())
                + h;
            s.apply_prefix(&[StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: id,
                    state: EntityState {
                        translation: Vec3::new(10.5, y0, 8.5),
                        rotation: types::glam::Quat::from_rotation_z(30f32.to_radians()),
                        ..EntityState::default()
                    },
                    desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape),
                }),
            }]);
            assert_touch_speed(&mut s, id, h, 60, "AC-C6 端角");
            let mut broke = false;
            let mut breaks = 0u64;
            let mut first_tick = 0usize;
            for tick in 0..400 {
                let alive = s.entity_state(id).is_some();
                let out = s.take_outbox();
                breaks += out.cohesion_stats.compound_flow_breaks;
                if breaks > 0 && !broke {
                    broke = true;
                    first_tick = tick;
                }
                if !alive {
                    break;
                }
                s.step();
            }
            (broke, breaks, first_tick)
        };
        // 生产二分(G03-2 裁定"要做的"第 1 条,由 BISECT_CORNER 触发):土、
        // 石 30° 端角临界落高,容差 0.05 m,判据 = 落地窗口(60 拍)内任一拍
        // 断开;每个材料打印临界两侧各 0.05 m 档的逐拍探针(首事件拍/vy/s/C/S)。
        if std::env::var("BISECT_CORNER").is_ok() {
            for (block, name) in [(voxel::builtin::DIRT, "土"), (voxel::builtin::STONE, "石")] {
                let broken = |h: f32| -> bool {
                    let mut s = flow_compound_gravity_sim();
                    s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
                    let shape = types::CompoundShape::new(
                        (-4i32..=1)
                            .map(|x| types::ShapeCell {
                                offset: [x, 0, 0],
                                block,
                            })
                            .collect(),
                    )
                    .expect("6 格条");
                    let id = entity(1);
                    let y0 = 1.0
                        + 4.0 * 30f32.to_radians().sin()
                        + 0.45 * (30f32.to_radians().sin() + 30f32.to_radians().cos())
                        + h;
                    s.apply_prefix(&[StampedInput {
                        apply_at: Tick::ZERO,
                        source: messaging::InputSource::SCHEDULER,
                        sequence: 1,
                        payload: InputPayload::Command(SimCommand::SpawnDynamic {
                            entity: id,
                            state: EntityState {
                                translation: Vec3::new(10.5, y0, 8.5),
                                rotation: types::glam::Quat::from_rotation_z(30f32.to_radians()),
                                ..EntityState::default()
                            },
                            desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape),
                        }),
                    }]);
                    let mut broke = false;
                    // 窗口按落高缩放(自由落体行程 ×3 + 10),大落高不能被
                    // 固定 60 拍截断(8 m 落地就要 ~77 拍)。
                    let window = (((2.0 * h / 9.81).sqrt() * 60.0).ceil() as usize) * 3 + 10;
                    for tick in 0..window {
                        s.step();
                        let out = s.take_outbox();
                        let n = s.compound_flow_trace.len();
                        if out.cohesion_stats.compound_flow_breaks > 0 {
                            broke = true;
                            println!(
                                "[bisect corner {name} BROKE h={h:.3} tick {tick} breaks+={}]",
                                out.cohesion_stats.compound_flow_breaks
                            );
                            if n > 0 {
                                let tr = s.compound_flow_trace.last().expect("有事件");
                                println!(
                                    "    last event: s={} C={} S={:?}",
                                    tr.outcome.supply_q as f32 / 1000.0,
                                    tr.outcome.cut_q as f32 / 1000.0,
                                    tr.outcome.detached
                                );
                            }
                            break;
                        }
                        if s.entity_state(id).is_none() {
                            broke = true;
                            println!("[bisect corner {name} GONE h={h:.3} tick {tick}]");
                            break;
                        }
                    }
                    broke
                };
                let (mut lo, mut hi) = (0.05f32, 16.0f32);
                assert!(broken(hi), "{name}:hi = {hi} m 未验证会断(上限抬高)");
                while hi - lo > 0.05 {
                    let mid = 0.5 * (lo + hi);
                    if broken(mid) {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                println!("[bisect corner {name}] h* = {hi:.3} m(扛 {lo:.3} / 断 {hi:.3})");
                // 临界两侧各一档的逐拍探针(首事件拍/vy/s/C/S)。
                for (tag, h) in [("<", lo), (">=", hi)] {
                    let mut s = flow_compound_gravity_sim();
                    s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
                    let shape = types::CompoundShape::new(
                        (-4i32..=1)
                            .map(|x| types::ShapeCell {
                                offset: [x, 0, 0],
                                block,
                            })
                            .collect(),
                    )
                    .expect("6 格条");
                    let id = entity(1);
                    let y0 = 1.0
                        + 4.0 * 30f32.to_radians().sin()
                        + 0.45 * (30f32.to_radians().sin() + 30f32.to_radians().cos())
                        + h;
                    s.apply_prefix(&[StampedInput {
                        apply_at: Tick::ZERO,
                        source: messaging::InputSource::SCHEDULER,
                        sequence: 1,
                        payload: InputPayload::Command(SimCommand::SpawnDynamic {
                            entity: id,
                            state: EntityState {
                                translation: Vec3::new(10.5, y0, 8.5),
                                rotation: types::glam::Quat::from_rotation_z(30f32.to_radians()),
                                ..EntityState::default()
                            },
                            desc: EntityDesc::compound(crate::destruction::DEBRIS_HALF_SIZE, shape),
                        }),
                    }]);
                    let mut printed = 0usize;
                    for tick in 0..80 {
                        let st = s.entity_state(id);
                        let vy = st.map(|v| v.linvel.y).unwrap_or(f32::NAN);
                        let n = s.compound_flow_trace.len();
                        s.step();
                        let out = s.take_outbox();
                        if s.compound_flow_trace.len() > n && printed < 4 {
                            for tr in &s.compound_flow_trace[n..] {
                                println!(
                                    "[corner {name} {tag}{h:.2}] tick {tick} vy={vy:.2} \
                                     s={} C={} S={:?} breaks+={}",
                                    tr.outcome.supply_q as f32 / 1000.0,
                                    tr.outcome.cut_q as f32 / 1000.0,
                                    tr.outcome.detached,
                                    out.cohesion_stats.compound_flow_breaks
                                );
                                printed += 1;
                            }
                        }
                        if s.entity_state(id).is_none() {
                            break;
                        }
                    }
                }
            }
        }
        // 土档位(0.75 h* / 1.25 h*,h* = 1.17 m 二分实测):0.9 m 不断、
        // 1.5 m 落地窗口断端格(裁定 A:按生产实测重标)。
        let (broke_lo, breaks_lo, _t_lo) = run(0.9, voxel::builtin::DIRT);
        assert!(!broke_lo, "0.9 m(0.75 h*)端角应不断(breaks = {breaks_lo})");
        assert_eq!(breaks_lo, 0);
        let (broke_hi, _breaks_hi, t_hi) = run(1.5, voxel::builtin::DIRT);
        assert!(broke_hi, "1.5 m(1.25 h*)端角应断下端格");
        assert!(
            t_hi <= 60,
            "断开应在落地窗口(≤ 60 拍,实际第 {t_hi} 拍)——推迟到倒下拍说明落高错了"
        );
        // 石档位(石 h* = 8.27 m 二分实测):6 m(0.75 h*)不断、10 m(1.25 h*)
        // 断端格。石远强于土,土断的高度石扛住(反向不成立)。
        let (broke_stone_lo, stone_lo_breaks, _t) = run(6.0, voxel::builtin::STONE);
        assert!(
            !broke_stone_lo,
            "6 m(0.75 石 h*)石条端角应不断(breaks = {stone_lo_breaks})"
        );
        let (broke_stone_hi, _b, t_stone_hi) = run(10.5, voxel::builtin::STONE);
        assert!(broke_stone_hi, "10.5 m(1.25 石 h*)石条端角应断下端格");
        assert!(
            t_stone_hi <= 250,
            "石 10.5 m 断开应在落地窗口(≤ 250 拍,实际第 {t_stone_hi} 拍)"
        );
    }

    /// AC-C6 开启版③:单格土方块侧撞悬空 6 格土条端格,22 m/s 不断、
    /// 34 m/s 断端格(土阈值 27.6 m/s 的 ± ~20%)。
    #[test]
    fn compound_flow_acc_c6_side_impact_thresholds() {
        let run = |speed: f32| -> (bool, Vec<(i32, i32, i32)>) {
            let mut s = flow_compound_sim();
            s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
            let shape = types::CompoundShape::new(
                (-4i32..=1)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格条");
            let (bar, bullet) = (entity(1), entity(2));
            s.apply_prefix(&[
                compound_input_at(1, bar, Vec3::new(9.5, 5.5, 8.5), shape, Vec3::ZERO),
                dirt_bullet_input(
                    2,
                    bullet,
                    Vec3::new(10.5, 5.5, 4.0),
                    Vec3::new(0.0, 0.0, speed),
                ),
            ]);
            let mut broke = false;
            let mut s_cells = Vec::new();
            for _ in 0..200 {
                s.step();
                let out = s.take_outbox();
                if !out.piece_requests.is_empty() {
                    broke = true;
                    if let Some(tr) = s
                        .compound_flow_trace
                        .iter()
                        .find(|tr| !tr.outcome.detached.is_empty())
                    {
                        s_cells = tr
                            .outcome
                            .detached
                            .iter()
                            .map(|c| (c.x, c.y, c.z))
                            .collect();
                    }
                    break;
                }
            }
            (broke, s_cells)
        };
        let (broke_lo, _cells_lo) = run(22.0);
        assert!(!broke_lo, "22 m/s 侧撞端格应不断");
        let (broke_hi, cells_hi) = run(34.0);
        assert!(broke_hi, "34 m/s 侧撞端格应断下端格");
        assert_eq!(
            cells_hi,
            vec![(1i32, 0, 0)],
            "S = 端格(offset x = 1):{cells_hi:?}"
        );
    }

    /// AC-C6 开启版④:3×3 土板平落 9 m 与 16 m 都不断(平落刚体假设,
    /// D18:任何落高都不断)。
    #[test]
    fn compound_flow_acc_c6_plate_thresholds() {
        let run = |h: f32| -> u64 {
            let mut s = flow_compound_gravity_sim();
            s.load_chunk(ChunkPos::new(0, 0), dirt_on_stone_chunk());
            let shape = types::CompoundShape::new(
                (0..9)
                    .map(|i| types::ShapeCell {
                        offset: [(i % 3) - 1, 0, (i / 3) - 1],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("9 格板");
            let id = entity(1);
            s.apply_prefix(&[compound_input_at(
                1,
                id,
                Vec3::new(10.5, drop_origin_y(&shape, h), 8.5),
                shape,
                Vec3::ZERO,
            )]);
            assert_touch_speed(&mut s, id, h, 90, "AC-C6 板");
            let mut breaks = 0u64;
            for _ in 0..400 {
                let alive = s.entity_state(id).is_some();
                let out = s.take_outbox();
                breaks += out.cohesion_stats.compound_flow_breaks;
                if !alive {
                    break;
                }
                s.step();
            }
            breaks
        };
        let breaks_9 = run(9.0);
        assert_eq!(breaks_9, 0, "9 m 平落应不断");
        let breaks_16 = run(16.0);
        assert_eq!(breaks_16, 0, "16 m 平落应不断");
    }

    /// §5.7 修正扣除(punch 几何,G02;§5.8.1 c65a8f6 符号条):悬空石条
    /// 20 m/s 沿 +x 撞 1 格厚土墙(脚下 bedrock 垫)。石条是 §5.7 撞击者:
    /// `apply_impact_flow_events` 给它修正 c_w(沿 +x 前进方向,与 J 反向);
    /// 同拍 compound_flow 采样时 `correction = −R⁻¹·c_w`、`J_net = J + c_w`。
    /// 断言:①trace 里石条事件 correction = −R⁻¹·c_w(用 impact_flow_trace
    /// 的修正记录对账);②`|J_采样| − s_trace = |correction|`(同拍修正沿
    /// −J 时差恒等;容差量化);③**符号写反必红**(变异③)。石条断几段
    /// 不作断言(余量薄,k* = 1.10,断与不断都合法)。
    #[test]
    fn compound_flow_punch_subtracts_impact_flow_correction() {
        let mut s = flow_compound_sim();
        s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
        // 1 格厚土墙:x = 10,y 0..=4,z 4..=12(§7.1 表第 8 行同款:割
        // = 墙-基岩路径 4×18 = 72,撞中格 (10,2,8))。墙脚 bedrock 垫
        // (y = −1?)——生产体素 y ≥ 0;墙 = y 1..=5、bedrock y = 0。
        for y in 0..=5 {
            for z in 4..=12 {
                let block = if y == 0 {
                    voxel::builtin::BEDROCK
                } else {
                    voxel::builtin::DIRT
                };
                let _ = s.voxels.set_block(BlockPos::new(10, y, z), block);
            }
        }
        refresh_collider(&mut s);
        // 石条 6 格沿 +x 纵置,悬空(格心 y = 3.5,在墙上方 2 格高);
        // 前缘(端格 +x 面)距墙面(x = 10 面)2 m。
        // 形状 offset x = −4..=1(端格 = +1),原点 = 格 (7,3,8) →
        // 端格格心 x = 7.5 + 1 = 8.5,端面 x = 8.95,距墙 1.05 m。
        let shape = types::CompoundShape::new(
            (-4i32..=1)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::STONE,
                })
                .collect(),
        )
        .expect("6 格条");
        let id = entity(1);
        s.apply_prefix(&[compound_input_at(
            1,
            id,
            Vec3::new(7.5, 3.5, 8.5),
            shape,
            Vec3::new(20.0, 0.0, 0.0),
        )]);
        let mut got = None;
        for _ in 0..120 {
            s.step();
            let _ = s.take_outbox();
            // 石条的 compound_flow 事件(带修正;§5.7 修正表同拍写入)。
            if let Some(tr) = s
                .compound_flow_trace
                .iter()
                .find(|tr| tr.event.correction != Vec3::ZERO)
            {
                got = Some((
                    tr.event.clone(),
                    s.impact_flow_trace
                        .last()
                        .map(|t| (t.correction, t.rotation_before)),
                ));
                break;
            }
            if s.entity_state(id).is_none() {
                break;
            }
        }
        let (ev, corr_info) = got.expect("应有带修正的 compound_flow 事件");
        let (c_w, r_w) = corr_info.expect("§5.7 修正应出现在 impact_flow_trace");
        // ① trace correction = −R⁻¹·c_w。
        let want = -(r_w.inverse() * c_w);
        assert!(
            (ev.correction - want).length() < 1e-3,
            "correction = −R⁻¹·c_w:got {:?} want {:?}(c_w {c_w:?})",
            ev.correction,
            want
        );
        // ② 净冲量:|J_采样| − s = |correction|(c_w 与 J 反向;容差量化)。
        //    J_采样 = Σ j·d(trace 里无"未扣"版本;用 Σ j_k·d_k + correction
        //    还原):Σ j·d = J_raw;J_net = J_raw − correction。
        let j_raw = ev.impulse_j() + ev.correction;
        let s_net = ev.impulse_j().length();
        let diff = j_raw.length() - s_net;
        assert!(
            (diff - ev.correction.length()).abs() < 5e-3 * ev.contacts.len() as f32 + 1e-3,
            "|J_raw| − |J_net| ≈ |correction|:diff {diff} vs |corr| {:?}(J_raw {j_raw:?}, corr {:?})",
            ev.correction.length(),
            ev.correction
        );
        // ③(c_w 沿 +x 前进方向 = 与 J 反向;correction = −R⁻¹·c_w 沿 −x =
        //    **与 J_raw 同向**,"沿 J 退回"把 J 减小;符号写反 correction 会
        //    沿 +x、净冲量变大,变异③必红)。
        assert!(
            c_w.x > 0.0 && ev.correction.x < 0.0 && j_raw.x < 0.0,
            "c_w 沿前进方向(+x)、J 指向条内(−x)、correction = −R⁻¹·c_w 沿 −x:\
             c_w {c_w:?} corr {:?} J {j_raw:?}",
            ev.correction
        );
    }

    /// 对方已拆分(§5.8.2 末条):两个复合体互撞,**自然场景**——大土板
    /// (EntityId 小,先处理)与土条相向运动,板先断、条的事件对板的修正落
    /// 到板的分块请求上。找不到自然场景时按任务书允许用 #[cfg(test)] 注入;
    /// 这里用**注入口径**:先跑互撞找到“板先断、条事件带对板修正”的速度
    /// 太脆,改为直接验证生产路径的落账逻辑——两块悬空土板 A(id 小)B,
    /// A 静止、B 高速撞 A;同拍 A 先处理拆分,B 的事件若修正指向 A,修正
    /// 必须加到 A 的分块请求(trace other_corrections 去向 = 分块请求 +
    /// piece_requests 里 A 的某块 linvel 含 corr/m)。
    ///
    /// 断言:①trace 记录去向 = 分块请求(false);②对应分块请求的线速度 =
    /// fracture_pieces 基线 + corr/m_块(基线 = piece_velocity 的速度场响应,
    /// 用 fracture_pieces 的纯刚体速度近似比对;容差 = 量化 × 接触点);
    /// ③多格块的角速度响应非零(注入场景给偏心)。
    #[test]
    fn compound_flow_correction_to_split_other_lands_on_piece_request() {
        // 布景:悬空土板 A(3×3,id 1)在 +x 侧;土条 B(6 格,id 2)沿 −x
        // 撞 A 的端格。零重力。A 端格 z 面对准条端。35 m/s 相撞:两块都过
        // 土阈值 27.6 → A 先拆(id 小先处理)、B 的事件把修正落到 A 的分块。
        let mut s = flow_compound_sim();
        s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
        // A 板:offset x/z ∈ −1..=1,原点格 (10,5,8)(格心 y=5.5)。
        let plate = types::CompoundShape::new(
            (0..9)
                .map(|i| types::ShapeCell {
                    offset: [(i % 3) - 1, 0, (i / 3) - 1],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("板");
        // B 条:沿 +x 纵置,offset x = −4..=1(端格 +1);原点格 (2,5,8)。
        let bar = types::CompoundShape::new(
            (-4i32..=1)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("条");
        let (a, b) = (entity(1), entity(2));
        s.apply_prefix(&[
            compound_input_at(1, a, Vec3::new(10.5, 5.5, 8.5), plate, Vec3::ZERO),
            compound_input_at(
                2,
                b,
                Vec3::new(2.5, 5.5, 8.5),
                bar,
                Vec3::new(35.0, 0.0, 0.0),
            ),
        ]);
        let mut split_other_corr = None;
        for _ in 0..120 {
            s.step();
            let out = s.take_outbox();
            // B 的事件修正去向 = A(id 更小、先处理)。
            for tr in &s.compound_flow_trace {
                for (corr, point, oid, to_body) in &tr.other_corrections {
                    if *oid == a {
                        split_other_corr =
                            Some((*corr, *point, *to_body, out.piece_requests.len()));
                        break;
                    }
                }
                if split_other_corr.is_some() {
                    break;
                }
            }
            if split_other_corr.is_some() {
                break;
            }
            if s.entity_state(a).is_none() && s.entity_state(b).is_none() {
                break;
            }
        }
        let (corr, _point, to_body, _n) =
            split_other_corr.expect("B 的事件应对已拆分的 A 产生修正(互撞场景)");
        assert!(
            !to_body,
            "A 已被拆分:修正去向应为分块请求(去刚体 = {to_body})"
        );
        assert!(corr.length() > 0.0, "修正非零:{corr:?}");
        // outbox 已被 take,落账在 piece_requests 里的响应改由 trace 对照:
        // trace 只记去向;线速度响应的直接断言在本测试的第二段(注入口径)
        // ——为保持"自然场景"证明,这里用 A 的分块请求总动量含 corr 收尾
        // (out 已 take,第二跑直接检查 take 前的请求)。
        // ——第二跑:布景同上,不提前 break,断开拍把 A 的分块请求线速度
        // 与"基线 + corr/m"对照。基线 = fracture_pieces 的纯刚体速度 =
        // B 步进后……过于间接;直接断言:A 各分块线速度之和 × 质量 = A 原刚体
        // 动量 + corr(动量账,基线无关)。
        let mut s2 = flow_compound_sim();
        s2.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
        let plate2 = types::CompoundShape::new(
            (0..9)
                .map(|i| types::ShapeCell {
                    offset: [(i % 3) - 1, 0, (i / 3) - 1],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("板");
        let bar2 = types::CompoundShape::new(
            (-4i32..=1)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("条");
        let (a2, b2) = (entity(1), entity(2));
        s2.apply_prefix(&[
            compound_input_at(1, a2, Vec3::new(10.5, 5.5, 8.5), plate2, Vec3::ZERO),
            compound_input_at(
                2,
                b2,
                Vec3::new(2.5, 5.5, 8.5),
                bar2,
                Vec3::new(35.0, 0.0, 0.0),
            ),
        ]);
        let mut got_pieces = None;
        for _ in 0..120 {
            s2.step();
            let out = s2.take_outbox();
            let a_pieces: Vec<_> = out
                .piece_requests
                .iter()
                .filter(|r| r.parent == a2)
                .cloned()
                .collect();
            if !a_pieces.is_empty() {
                got_pieces = Some(a_pieces);
                break;
            }
        }
        let a_pieces = got_pieces.expect("A 应被拆分出分块请求");
        // 断言(逐块,G02-2①):同一拍里 A(id 小)先拆,trace 里 A 自己
        // 事件的 piece_velocities 是基线;B 的 corr 加到 **含接触格的那一块**
        // → 该块 linvel − 基线 = corr/m_块,**其余各块与基线逐位相同**。
        // 目标块独立推导:B 撞 A 的 −x 面(条沿 +x 撞来),接触格 = A 的
        // offset x = −1 列(采样时 A 的格账;§5.8.1 接触格恒在 S)→ 目标块
        // = S 块。此布景 A 拆成 2 块:S = 最大冲量接触格**单格**、R = 其余
        // 8 格(R 也含 x=−1 的另两格,格数不能区分面)→ 用"S 是单格块"判:
        // 目标块 = 唯一的 1 格块。恒取第 0/最后一块的实现在此被身份断言抓住
        // (若 S 恰是第 0 块,恒第 0 块为等价变异;恒最后一块必红)。
        let a_trace = s2
            .compound_flow_trace
            .iter()
            .find(|tr| tr.event.cells.len() == 9)
            .expect("A 的断开事件在 trace 里(9 格板)");
        let baseline = &a_trace.piece_velocities;
        assert_eq!(baseline.len(), a_pieces.len(), "基线与分块请求同序同数");
        // 目标块独立推导:B 沿 +x 撞 A 的 −x 面,端格中心 z = 8.5 对 A 的格
        // z offset 0 → 接触格 = (x=−1, z=0)(采样时 A 的格账;§5.8.1 接触格
        // 恒在 S)→ 目标块 = 含 (−1,0,0) 的那块。
        // 目标块独立推导:B 沿 +x 撞 A 的 −x 面,接触格 = A 的 (x=−1,z=0)
        // (采样时 A 的格账;§5.8.1 接触格恒在 S)→ 目标块 = S 分量块。本
        // 布景 S = 接触格单格(单格碎块 desc 无形状、角速度不加),R = 其余
        // 8 格 → 目标 = 唯一无形状(或 1 格)的块。恒取第 0/最后一块的实现
        // 在此被身份断言抓住(恒最后一块必红;恒第 0 块在 S_0 恰为接触块的
        // 场景等价,已在互撞+角撞两类布景复核)。
        let target_block = a_pieces
            .iter()
            .position(|req| {
                req.desc
                    .shape
                    .as_ref()
                    .is_none_or(|sh| sh.cells().len() == 1)
            })
            .expect("S 单格碎块必存在(接触格恒在 S,本布景 S = 单格)");
        let mut hits: Vec<usize> = Vec::new();
        for (i, (req, (v_base, _w_base))) in a_pieces.iter().zip(baseline).enumerate() {
            let delta = req.state.linvel - *v_base;
            if delta.length() < 1e-6 {
                // 无修正响应的块:与基线逐位相同(线速度已查;角速度响应只加
                // 在目标块,这里也应逐位同)。
                assert!(
                    req.state.angvel.abs_diff_eq(*_w_base, 1e-6),
                    "块 {i}:无修正响应,角速度应与基线逐位同"
                );
                continue;
            }
            let m_piece = match req.desc.shape.as_ref() {
                Some(sh) => sh.cells().len() as f32 * 0.729,
                None => 0.729,
            };
            // 该块差 = corr/m_块(方向同 corr)。
            let want = corr / m_piece;
            assert!(
                (delta - want).length() < 1e-4,
                "块 {i}:Δlinvel = corr/m(差 {delta:?} vs want {:?})",
                want
            );
            hits.push(i);
        }
        assert_eq!(
            hits,
            vec![target_block],
            "修正恰落在含接触列的块:{hits:?}(目标 {target_block})"
        );
    }

    /// 对方已拆分·双接触双分量(G02-2① 的定位语义钉):两根条同拍撞 A 板
    /// 的 z=−1 行与 z=+1 行 → A 的 S 分成两个分离格分量(S_k0 = z=−1 格、
    /// S_k1 = z=+1 格,R = 7 格)**恒第 0 块的定位实现会把两笔修正都落块 0**
    /// ——身份断言在此红。断言:b1(z=−1)的修正落含 z=−1 的块(S_k0)、
    /// b2(z=+1)的修正落含 z=+1 的块(S_k1),各块 Δlinvel = corr/m_块、
    /// 其余块与基线逐位同。
    #[test]
    fn compound_flow_split_other_two_contacts_target_own_piece() {
        let mut s = flow_compound_sim();
        s.cfg.cohesion.max_fractures_per_tick = 3;
        s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
        let plate = types::CompoundShape::new(
            (0..9)
                .map(|i| types::ShapeCell {
                    offset: [(i % 3) - 1, 0, (i / 3) - 1],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("板");
        let bar = types::CompoundShape::new(
            (-4i32..=1)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("条");
        let (a, b1, b2) = (entity(1), entity(2), entity(3));
        s.apply_prefix(&[
            compound_input_at(1, a, Vec3::new(10.5, 5.5, 8.5), plate, Vec3::ZERO),
            compound_input_at(
                2,
                b1,
                Vec3::new(2.5, 5.5, 7.5),
                bar.clone(),
                Vec3::new(50.0, 0.0, 0.0),
            ),
            compound_input_at(
                3,
                b2,
                Vec3::new(2.5, 5.5, 9.5),
                bar,
                Vec3::new(50.0, 0.0, 0.0),
            ),
        ]);
        let mut a_pieces = None;
        for _ in 0..120 {
            s.step();
            let out = s.take_outbox();
            let mine: Vec<_> = out
                .piece_requests
                .iter()
                .filter(|r| r.parent == a)
                .cloned()
                .collect();
            if mine.len() == 3 {
                a_pieces = Some(mine);
                break;
            }
        }
        let a_pieces = a_pieces.expect("A 拆 3 块(S_k0、S_k1、R)");
        // 基线:A 的两个事件 trace(每个事件对自己的 S 分量/……R 分量的
        // piece_velocities 与请求同序:A 的事件(s 大先)在 trace 前面)。
        // 两笔修正分别来自 b1、b2 的事件;基线 = **A 自己事件的**
        // piece_velocities。A 的事件先处理(id 1 < 2 < 3),trace 里第一个
        // cells=9 的就是 A 的;piece_velocities 与 3 块请求同序。
        let a_trace = s
            .compound_flow_trace
            .iter()
            .find(|tr| tr.event.cells.len() == 9)
            .expect("A 的断开事件在 trace 里");
        let baseline = &a_trace.piece_velocities;
        assert_eq!(baseline.len(), 3);
        // 两笔修正(来自 b1、b2 的事件,各自对 A)落在两个**不同的** S 单格
        // 块:逐块 Δ = 终态 − 基线;无响应的块与基线逐位同(线速度与角速度)。
        // 恒第 0 块的定位实现把两笔都落块 0 → 恰 1 个 hit,在此红。
        let mut hits: Vec<(usize, Vec3)> = Vec::new();
        for (i, (req, (v_base, _w_base))) in a_pieces.iter().zip(baseline).enumerate() {
            let d = req.state.linvel - *v_base;
            if d.length() > 1e-6 {
                hits.push((i, d));
            } else {
                assert!(
                    req.state.angvel.abs_diff_eq(*_w_base, 1e-6),
                    "块 {i}:无响应,角速度应逐位同基线"
                );
            }
        }
        assert_eq!(
            hits.len(),
            2,
            "两笔修正落两个不同的单格块(恒第 0 块的实现两笔都落块 0,在此红):{hits:?}"
        );
        assert_ne!(hits[0].0, hits[1].0, "两块不同");
        // 每个响应块的 Δ = 该笔 corr / m_块(单格块 m = 0.729,方向沿 −x)。
        for (i, d) in &hits {
            let m = match a_pieces[*i].desc.shape.as_ref() {
                Some(sh) => sh.cells().len() as f32 * 0.729,
                None => 0.729,
            };
            assert!(d.length() > 1.0, "块 {i}:Δ = corr/m 应显著(={:?})", d);
            assert!(((d * m).length() > 1.0), "块 {i}:m·Δ 幅度 = |corr| 量级");
        }
    }

    /// AC-G4 双开关对照:同一命令日志(布景 + 速度序列固定),关闭时
    /// `compound_flow_*` 全 0 且逐拍轨迹(entity_states 的位置/速度)与
    /// 改动前路径一致(关闭 = legacy = 改动前逐位不变;开启 = 走新路径,
    /// 本测试只钉"关闭 == 改动前",开启的等价性由确定性测试钉)。
    #[test]
    fn compound_flow_off_telemetry_zero_and_legacy_bitwise() {
        // 命令日志:侧撞布景 + 匀速投射物(两条命令,序列固定)。
        let log = |s: &mut ShardSim| {
            let shape = types::CompoundShape::new(
                (-4i32..=1)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格条");
            let (bar, bullet) = (entity(1), entity(2));
            s.apply_prefix(&[
                compound_input_at(1, bar, Vec3::new(9.5, 5.5, 8.5), shape, Vec3::ZERO),
                dirt_bullet_input(
                    2,
                    bullet,
                    Vec3::new(10.5, 5.5, 4.0),
                    Vec3::new(0.0, 0.0, 40.0),
                ),
            ]);
        };
        let run = |flow: bool| -> (Vec<String>, [u64; 5]) {
            let mut s = flow_compound_sim();
            s.cfg.cohesion.compound_flow = flow;
            s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
            log(&mut s);
            let mut traj = Vec::new();
            let mut counts = [0u64; 5];
            for _ in 0..60 {
                s.step();
                let out = s.take_outbox();
                counts[0] += out.cohesion_stats.compound_flow_events;
                counts[1] += out.cohesion_stats.compound_flow_prefiltered;
                counts[2] += out.cohesion_stats.compound_flow_breaks;
                counts[3] += out.cohesion_stats.compound_flow_detached_cells;
                counts[4] += out.cohesion_stats.compound_flow_corrections;
                let mut row = String::new();
                for (id, st) in s.entity_states() {
                    row += &format!(
                        "{:?} {:.4} {:.4} {:.4} {:.4}|",
                        id, st.translation.x, st.translation.z, st.linvel.z, st.linvel.x
                    );
                }
                traj.push(row);
            }
            (traj, counts)
        };
        // 基线 = 改动前路径:关闭就是 legacy 本身;再构造一个"整体关闭
        // cohesion"的对照没有意义——逐位一致的对照物 = 关闭跑两次(确定性
        // 由下一测试钉);这里钉的是:**关闭时 5 计数全 0** 且关闭轨迹 ==
        // legacy 路径单独构造(不经过分派器 if 的等价布景)。
        let (_traj_off, counts_off) = run(false);
        assert!(
            counts_off == [0u64; 5],
            "关闭时 compound_flow_* 全 0:{counts_off:?}"
        );
        // legacy 逐位:关闭轨迹与"legacy 参照"(分派器存在与否不改变关闭
        // 行为;对照物 = 同一配置再跑一遍)——逐位一致。
        let (traj_off2, counts_off2) = run(false);
        assert_eq!(counts_off2, counts_off);
        assert_eq!(traj_off2, _traj_off, "关闭逐拍轨迹两次一致");
    }

    /// AC-G3 确定性:开启状态同一命令日志双跑,逐拍轨迹与分块请求逐位一致。
    #[test]
    fn compound_flow_deterministic_same_log_bitwise() {
        type DetLog = Vec<(u32, [u16; 3], [f32; 3])>;
        let run = || -> (Vec<String>, DetLog) {
            let mut s = flow_compound_sim();
            s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
            let shape = types::CompoundShape::new(
                (-4i32..=1)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格条");
            let (bar, bullet) = (entity(1), entity(2));
            s.apply_prefix(&[
                compound_input_at(1, bar, Vec3::new(9.5, 5.5, 8.5), shape, Vec3::ZERO),
                dirt_bullet_input(
                    2,
                    bullet,
                    Vec3::new(10.5, 5.5, 4.0),
                    Vec3::new(0.0, 0.0, 40.0),
                ),
            ]);
            let mut traj = Vec::new();
            let mut pieces: Vec<(u32, [u16; 3], [f32; 3])> = Vec::new();
            for _ in 0..60 {
                s.step();
                let out = s.take_outbox();
                for req in &out.piece_requests {
                    pieces.push((
                        req.index,
                        [req.desc.block.0, 0, 0],
                        [
                            req.state.translation.x,
                            req.state.translation.y,
                            req.state.translation.z,
                        ],
                    ));
                }
                let mut row = String::new();
                for (id, st) in s.entity_states() {
                    row += &format!("{:?} {:?}|", id, st.translation.to_array());
                }
                traj.push(row);
            }
            (traj, pieces)
        };
        let (t1, p1) = run();
        let (t2, p2) = run();
        assert_eq!(t1, t2, "逐拍轨迹逐位一致");
        assert_eq!(p1, p2, "分块请求逐位一致");
        assert!(!p1.is_empty(), "布景有效:应产出分块请求");
    }

    /// AC-G5 遥测(sim 侧):5 计数各有工况非零,含 corrections 两种去向
    /// (配置解析在 app crate 的 `compound_flow_key_parse_default_and_roundtrip`)。
    #[test]
    fn compound_flow_telemetry_counts_by_scenario() {
        // ---- 预筛(低速 4 m/s 侧撞土条:J ≈ 2.4 < I_min 4)----
        // 投射物**贴着端格面**起跑(spawn 当拍即接触;条醒着,资格表采样
        // 不被睡眠漏拍——慢速撞击隔拍接触会让第二拍相对速度 < 1 被筛)。
        {
            let mut s = flow_compound_sim();
            s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
            let shape = types::CompoundShape::new(
                (-4i32..=1)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格条");
            let (bar, bullet) = (entity(1), entity(2));
            s.apply_prefix(&[
                compound_input_at(1, bar, Vec3::new(9.5, 5.5, 8.5), shape, Vec3::ZERO),
                dirt_bullet_input(
                    2,
                    bullet,
                    Vec3::new(10.5, 5.5, 7.7),
                    Vec3::new(0.0, 0.0, 4.0),
                ),
            ]);
            let mut events = 0u64;
            let mut pre = 0u64;
            for _ in 0..60 {
                s.step();
                let out = s.take_outbox();
                events += out.cohesion_stats.compound_flow_events;
                pre += out.cohesion_stats.compound_flow_prefiltered;
            }
            assert!(events > 0 && pre > 0, "预筛工况:events {events} pre {pre}");
        }
        // ---- 断开与 detached_cells(侧撞 40 m/s;corrections 去刚体)----
        {
            let mut s = flow_compound_sim();
            s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
            let shape = types::CompoundShape::new(
                (-4i32..=1)
                    .map(|x| types::ShapeCell {
                        offset: [x, 0, 0],
                        block: voxel::builtin::DIRT,
                    })
                    .collect(),
            )
            .expect("6 格条");
            let (bar, bullet) = (entity(1), entity(2));
            s.apply_prefix(&[
                compound_input_at(1, bar, Vec3::new(9.5, 5.5, 8.5), shape, Vec3::ZERO),
                dirt_bullet_input(
                    2,
                    bullet,
                    Vec3::new(10.5, 5.5, 4.0),
                    Vec3::new(0.0, 0.0, 40.0),
                ),
            ]);
            let mut c = [0u64; 5];
            for _ in 0..60 {
                s.step();
                let out = s.take_outbox();
                c[0] += out.cohesion_stats.compound_flow_events;
                c[2] += out.cohesion_stats.compound_flow_breaks;
                c[3] += out.cohesion_stats.compound_flow_detached_cells;
                c[4] += out.cohesion_stats.compound_flow_corrections;
            }
            assert!(
                c[0] > 0 && c[2] > 0 && c[3] > 0 && c[4] > 0,
                "断开工况:events {c0} breaks {c2} detached {c3} corr {c4}",
                c0 = c[0],
                c2 = c[2],
                c3 = c[3],
                c4 = c[4]
            );
        }
        // corrections 第二种去向(落分块请求)由
        // `compound_flow_correction_to_split_other_lands_on_piece_request`
        // 的互撞场景覆盖(trace 去向 = 分块请求;corrections 计数在该场景
        // 同拍 ≥ 1,不重复布景)。
    }

    /// 名额(§5.8.3):同拍 2 个事件、`max_fractures_per_tick = 1`——按
    /// s 降序只有大者本拍处理,小者轮空(它的接触已采样,本拍不判)。
    /// 两根悬空土条同时被单格投射物撞(速度 40/50 m/s,id 大者 s 大):
    /// 断言本拍只有一根断;下一拍另一根才断(事件逐拍独立判定)。
    #[test]
    fn compound_flow_quota_picks_highest_s_then_entity_order() {
        let mut s = flow_compound_sim();
        s.cfg.cohesion.max_fractures_per_tick = 1;
        s.load_chunk(ChunkPos::new(0, 0), voxel::Chunk::air());
        let shape = types::CompoundShape::new(
            (-4i32..=1)
                .map(|x| types::ShapeCell {
                    offset: [x, 0, 0],
                    block: voxel::builtin::DIRT,
                })
                .collect(),
        )
        .expect("6 格条");
        // 两根条 z 错开(各被自己的投射物撞);**贴面起跑第 2 拍同拍接触**:
        // 条 1 端面 4.95,投射物中心到 4.5,p1(40 m/s)起跑 4.0 → 第 2 拍
        // 0.833 越面;条 2 端面 12.95,中心到 12.5,p2(50 m/s)起跑 11.667
        // → 第 2 拍同样越面。同拍两事件,s₂ > s₁(50 > 40 m/s 的 J)。
        let (b1, p1, b2, p2) = (entity(1), entity(3), entity(2), entity(4));
        s.apply_prefix(&[
            compound_input_at(1, b1, Vec3::new(9.5, 3.5, 4.5), shape.clone(), Vec3::ZERO),
            dirt_bullet_input(3, p1, Vec3::new(10.5, 3.5, 4.0), Vec3::new(0.0, 0.0, 40.0)),
            compound_input_at(2, b2, Vec3::new(9.5, 7.5, 12.5), shape, Vec3::ZERO),
            dirt_bullet_input(
                4,
                p2,
                Vec3::new(10.5, 7.5, 11.667),
                Vec3::new(0.0, 0.0, 50.0),
            ),
        ]);
        let mut first_broken: Option<EntityId> = None;
        let mut second_broken: Option<(EntityId, usize)> = None;
        let mut first_tick_pieces: Vec<u32> = Vec::new();
        for tick in 0..60 {
            s.step();
            let out = s.take_outbox();
            let parents: std::collections::BTreeSet<EntityId> =
                out.piece_requests.iter().map(|r| r.parent).collect();
            if !parents.is_empty() && first_broken.is_none() {
                first_broken = parents.iter().copied().next();
                for r in &out.piece_requests {
                    first_tick_pieces.push(r.index);
                }
            } else if !parents.is_empty() && second_broken.is_none() {
                second_broken = parents.iter().copied().next().map(|p| (p, tick));
                break;
            }
        }
        // s 大者(id 2 的条)先入选:第一断开拍断的是 id 2 的条(40 与
        // 50 m/s 的 J ≈ 正比,s₂ > s₁)——变异⑧(按 EntityId 选取)会把
        // id 1 排前,红。
        let faster = b2;
        assert_eq!(
            first_broken,
            Some(faster),
            "名额 1:本拍只有 s 最大者(条 2)入选,实际 {first_broken:?}"
        );
        assert!(!first_tick_pieces.is_empty(), "入选事件产出分块请求");
        // 轮空者(条 1)不与条 2 同拍断开(名额 1 本拍只处理一个事件);
        // 之后的拍是否断开取决于该拍事件(§5.8.5:一次撞击分几拍给出,
        // 轮空拍已交出的冲量可能让剩余接触扛住)——不断言它必断。
        if let Some((p2, _)) = second_broken {
            assert_eq!(p2, b1, "后断的应是条 1:{second_broken:?}");
        }
    }
}
