//! 外部命令与统一盖章输入。

use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{BlockId, BlockPos, EntityId, EntityState, Tick};

/// 输入来源:外部命令 = 连接/会话序号(app 层分配);内部 = 分片序号(shard 注册表)。
/// 0 保留给调度器注入的输入。
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
            },
            SimCommand::RemoveEntity {
                entity: EntityId::from_parts(1, 1),
            },
            SimCommand::TeleportEntity {
                entity: EntityId::from_parts(1, 1),
                state: EntityState::default(),
            },
            cmd_detonate(),
        ];
        for (i, c) in all.iter().enumerate() {
            assert_eq!(c.ordinal() as usize, i, "声明序 = 类型序号");
        }
        assert!(cmd_set().ordinal() < cmd_detonate().ordinal());
    }

    #[test]
    fn scheduler_source_is_zero() {
        assert_eq!(InputSource::SCHEDULER, InputSource(0));
        assert!(InputSource::SCHEDULER < InputSource(1), "调度器来源排最前");
    }
}
