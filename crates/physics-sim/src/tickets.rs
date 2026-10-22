//! 票据视图类型(REQ-007 §4.2;任务书 req007-tickets/02)。
//!
//! 本模块只定义**决策层可见的形状**:[`TicketView`] 与其引用的规格/状态类型。
//! 票据表、操作语义(幂等签发/转递)、收件箱在 `shard::tickets`(调度器级
//! 全局唯一一张表);决策层(`ActivationController::plan`)只见扁平视图切片。
//!
//! §4.2.2 扩展性硬性要求(S1/§14-1):视图**不含 holder/kind**——票种语义在
//! 结构上进不了决策层;`order` 是票据表键序中的名次(0 起连续),仅用于
//! 确定性排序与平局决胜。全部枚举**只追加**(serde 序列化随 format_version
//! 管理,§14-2)。

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use types::{ChunkPos, EntityId, Tick, glam::Vec3};

use messaging::InputSource;

/// 区域票 id(任务书 02;`SetAreaTicket.id`)。
pub type AreaId = u64;

/// 票据持有者(§4.2.1;只追加)。全局键 = `(TicketHolder, TicketKind)`。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum TicketHolder {
    Observer(InputSource),
    /// 实体运动票(瞬态/常驻;03/04 接生产者,本任务只落表语义与测试)。
    Entity(EntityId),
    Area(AreaId),
}

/// 票据种类(§4.2.1;只追加,声明序即序号)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum TicketKind {
    Observer,
    Area,
    TransientMotion,
    ActorMotion,
}

/// ①/② 覆盖规格(§4.2.1;只追加)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RegionSpec {
    None,
    /// 以锚点为圆心、水平半径、带滞回(`load_m` 进 / `unload_m` 出)。
    Radius {
        load_m: f32,
        unload_m: f32,
    },
    /// 随实体足迹(03 语义;本任务不产生产者)。
    Footprint {
        lookahead_ticks: u64,
        max_lookahead_chunks: usize,
    },
    /// 固定 chunk 集(未来事件票等)。
    Chunks(BTreeSet<ChunkPos>),
}

/// ③ 唤醒规格(§4.2.1;只追加)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum WakeSpec {
    None,
    /// 只保证持有者自身清醒(03 语义)。
    SelfOnly,
    /// 唤醒范围内的他人(3D 半径,带滞回)。
    Radius {
        load_m: f32,
        unload_m: f32,
    },
}

/// 票据时钟(§4.3.2:调度器每拍末对 Active 票 `active_ticks += 1`;
/// `limit` 以 active_ticks 计的上限,`None` = 无上限)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TicketClock {
    pub issued_at: Tick,
    pub active_ticks: u64,
    pub limit: Option<u64>,
    /// 落点预约票预留(本 REQ 恒 `None`)。
    pub starts_at: Option<Tick>,
}

/// 票据状态(§4.2.1):Active 占区域时钟走;Paused 占区域时钟停;
/// Suspended 不占区域时钟停。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TicketState {
    Active,
    Paused,
    Suspended,
}

/// 决策层唯一可见形状(S1/§14-1)。**不含 holder/kind**——票种语义在结构上
/// 进不了决策层;`order` 是票据表键序中的名次(0 起连续),仅用于确定性
/// 排序与平局决胜。`anchor` 每拍由调用方按持有者派生(观察者坐标 / 实体
/// 位置 / 区域票中心)。
#[derive(Debug, Clone)]
pub struct TicketView {
    /// 票据表键序名次(0 起,连续;排序/平局决胜用,不透明)。
    pub order: u32,
    /// 预算排序,小者优先(§4.12)。
    pub priority: u8,
    /// ① 数据覆盖。
    pub data: RegionSpec,
    /// ② collider 覆盖(须 ⊆ data;校验在签发/配置层)。
    pub collider: RegionSpec,
    /// ③ 唤醒覆盖。
    pub wake: WakeSpec,
    pub state: TicketState,
    /// 锚点(每拍派生:观察者坐标 / 实体位置 / 区域票中心)。
    pub anchor: Vec3,
}

impl TicketView {
    /// 该票是否参与决策/保活/ensure(§4.2.1:Suspended 不占用区域)。
    pub fn occupies(&self) -> bool {
        !matches!(self.state, TicketState::Suspended)
    }
}
