//! 外部命令与统一盖章输入。

use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{BlockId, BlockPos, EntityDesc, EntityId, EntityState, Tick};

/// 输入来源:外部命令 = 连接/会话序号(app 层分配);内部 = 分片序号(shard 注册表)。
/// 0 保留给调度器注入的输入;`u32` 高位段自 `u32::MAX` 向左为调度器
/// `set_observers` 直通多观察者落表保留(见 `Scheduler::set_observers`),
/// 分配给连接/会话的正向序号(1,2,…)永不触及该段。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct InputSource(pub u32);

impl InputSource {
    /// 调度器自身(注入类输入的来源)。
    pub const SCHEDULER: Self = Self(0);
}

/// 外部命令:模拟状态只能因它改变(REQ-001 共识 1)。
/// 变体只允许**追加**(序号即 canonical 排序的类型序号,§9.6)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SimCommand {
    SetBlock {
        pos: BlockPos,
        block: BlockId,
    },
    SpawnDynamic {
        entity: EntityId,
        state: EntityState,
        desc: EntityDesc,
    },
    RemoveEntity {
        entity: EntityId,
    },
    TeleportEntity {
        entity: EntityId,
        state: EntityState,
    },
    Detonate {
        center: Vec3,
        power: f32,
        radius: f32,
    },
    /// 演员标记(REQ-002 决议 4b):置位/清除实体的政策层豁免标记;
    /// 演员实体完全豁免自动凝固(P2 计数与睡眠兜底),仅显式命令将来触发。
    /// 触及不存在实体 → panic(fail-stop)。
    MarkActor {
        entity: EntityId,
        actor: bool,
    },
    /// 观察者移动(REQ-002 §4.2 命令化):设置来源玩家/agent 的观察者位置。
    /// 不进分片收件箱——由调度器级专用收件箱消费(`Scheduler::submit_observer`),
    /// 分片输入前缀里永远不会出现本变体(回放确定性的硬要求)。
    SetObserverPosition {
        position: Vec3,
    },
    /// 观察者移除(REQ-005 任务书 02/D9):删除 `source`(由
    /// `StampedInput.source` 隐含,无字段)的观察者条目;不存在 = 幂等。
    /// **不走分片前缀**——与 [`SimCommand::SetObserverPosition`] 同走调度器
    /// 级观察者收件箱(观察者收件箱专用);分片输入前缀出现本变体 = 一致性
    /// 违规 fail-stop。客户端白名单拒绝(C2S translate 无上行映射)。
    RemoveObserver,
    /// 区域票签发(REQ-007 §4.2.3/D10,任务 02):同 id 覆盖 = 续期(S3,
    /// 时钟重置);超上限由 Session 类型化拒绝(`AreaTicketError`,不 panic)。
    /// **不走分片前缀**——与观察者命令同走调度器级票据收件箱(当拍生效);
    /// **不进 C2S 白名单**(仅本地受信通道:driver commands / Session);
    /// 照常盖章入外部命令日志(回放再生)。
    SetAreaTicket {
        id: u64,
        center: Vec3,
        radius_m: f32,
        /// 层级:① = 0b001,①② = 0b011,①②③ = 0b111(位集;其余 = 拒绝)。
        tiers: u8,
        ttl_ticks: Option<u64>,
    },
    /// 区域票移除(REQ-007 §4.2.3/D10):不存在 = 幂等;同走票据收件箱,
    /// 不进 C2S。
    RemoveAreaTicket {
        id: u64,
    },
}

impl SimCommand {
    /// 枚举声明序(canonical 类型序号的低位)。
    pub const fn ordinal(self) -> u16 {
        match self {
            Self::SetBlock { .. } => 0,
            Self::SpawnDynamic { .. } => 1,
            Self::RemoveEntity { .. } => 2,
            Self::TeleportEntity { .. } => 3,
            Self::Detonate { .. } => 4,
            Self::MarkActor { .. } => 5,
            Self::SetObserverPosition { .. } => 6,
            Self::RemoveObserver => 7,
            Self::SetAreaTicket { .. } => 8,
            Self::RemoveAreaTicket { .. } => 9,
        }
    }
}

/// 盖章后的分片输入——统一输入模型(REQ-001 §4.2):一切分片输入(命令/信封/事件广播)
/// 都由调度器盖章 apply_at;分片只消费盖章输入。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StampedInput {
    pub apply_at: Tick,
    pub source: InputSource,
    pub sequence: u64,
    pub payload: InputPayload,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum InputPayload {
    Command(SimCommand),
    Event(crate::event::CrossEvent),
    Envelope(crate::envelope::Envelope),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd_set() -> SimCommand {
        SimCommand::SetBlock {
            pos: BlockPos::new(1, 2, 3),
            block: BlockId(1),
        }
    }

    fn cmd_detonate() -> SimCommand {
        SimCommand::Detonate {
            center: Vec3::ZERO,
            power: 6.0,
            radius: 5.0,
        }
    }

    #[test]
    fn command_ordinals_follow_declared_order() {
        let all = [
            SimCommand::SetBlock {
                pos: BlockPos::ZERO,
                block: BlockId::AIR,
            },
            SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 1),
                state: EntityState::default(),
                desc: EntityDesc::default(),
            },
            SimCommand::RemoveEntity {
                entity: EntityId::from_parts(1, 1),
            },
            SimCommand::TeleportEntity {
                entity: EntityId::from_parts(1, 1),
                state: EntityState::default(),
            },
            cmd_detonate(),
            // 任务书 01:MarkActor 追加(声明序最后,ordinal = 5)。
            SimCommand::MarkActor {
                entity: EntityId::from_parts(1, 1),
                actor: true,
            },
            // 任务书 02:SetObserverPosition 追加(声明序最后,ordinal = 6)。
            SimCommand::SetObserverPosition {
                position: Vec3::ZERO,
            },
            // REQ-005 任务书 02:RemoveObserver 追加(声明序最后,ordinal = 7)。
            SimCommand::RemoveObserver,
        ];
        for (i, c) in all.iter().enumerate() {
            assert_eq!(c.ordinal() as usize, i, "声明序 = 类型序号");
        }
        assert!(cmd_set().ordinal() < cmd_detonate().ordinal());
        // 追加变体:ordinal 续在最后(枚举只许追加,REQ-001 §9.6)。
        assert_eq!(
            SimCommand::MarkActor {
                entity: EntityId::from_parts(1, 1),
                actor: false
            }
            .ordinal(),
            5,
            "MarkActor = 追加序 5"
        );
        assert_eq!(
            SimCommand::SetObserverPosition {
                position: Vec3::ZERO
            }
            .ordinal(),
            6,
            "SetObserverPosition = 追加序 6(任务书 02)"
        );
        // REQ-005 任务书 02:RemoveObserver = 追加序 7(枚举只许追加)。
        assert_eq!(
            SimCommand::RemoveObserver.ordinal(),
            7,
            "RemoveObserver = 追加序 7(REQ-005 任务书 02)"
        );
    }

    #[test]
    fn scheduler_source_is_zero() {
        assert_eq!(InputSource::SCHEDULER, InputSource(0));
        assert!(InputSource::SCHEDULER < InputSource(1), "调度器来源排最前");
    }
    /// 测试清单(messaging,任务书 02):区域票两变体 ordinal = 8/9;旧 0–7
    /// 不变;旧编码可解(只追加枚举:新变体追加在尾,反序列化旧字节不受影响)。
    #[test]
    fn area_ticket_ordinals_appended() {
        assert_eq!(SimCommand::RemoveObserver.ordinal(), 7);
        assert_eq!(
            SimCommand::SetAreaTicket {
                id: 1,
                center: Vec3::ZERO,
                radius_m: 32.0,
                tiers: 0b111,
                ttl_ticks: None,
            }
            .ordinal(),
            8
        );
        assert_eq!(SimCommand::RemoveAreaTicket { id: 1 }.ordinal(), 9);
        assert_eq!(cmd_set().ordinal(), 0, "旧 0 不变");
        assert_eq!(cmd_detonate().ordinal(), 4, "旧 4 不变");
        // 旧编码可解:bincode 依赖不在本 crate(日志层 sim-api 覆盖);此处
        // 锚"只追加"的结构前提——新变体只追加在尾、ordinal 单调,旧字节解码
        // 不受影响(旧 0–7 断言已在上方)。
    }
}
