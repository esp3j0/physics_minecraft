//! canonical 全序(§9.6):五元组 `(apply_at, destination, 类型序号, source, sequence)`
//! 字典序;类型序号 = 枚举声明序(编译期固定)。destination 作为分桶键(信封队列的
//! bucket / 分片收件箱),本结构承载其余四元——在同一分片内与五元组全序等价。

use serde::{Deserialize, Serialize};
use types::Tick;

use crate::command::{InputPayload, InputSource, StampedInput};

/// 类型序号高位:输入种类(InputPayload 声明序)。低位:各载荷枚举自己的声明序。
const KIND_COMMAND: u16 = 0;
const KIND_EVENT: u16 = 1;
const KIND_ENVELOPE: u16 = 2;

/// 同一分片内输入的 canonical 排序键。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct CanonicalKey {
    pub apply_at: Tick,
    pub type_ordinal: u16,
    pub source: InputSource,
    pub sequence: u64,
}

impl StampedInput {
    /// 计算 canonical 键。同 tick 同分片的输入按此排序后应用,保证确定性。
    pub fn canonical_key(&self) -> CanonicalKey {
        let (kind, leaf) = match &self.payload {
            InputPayload::Command(c) => (KIND_COMMAND, c.ordinal()),
            InputPayload::Event(e) => (KIND_EVENT, e.ordinal()),
            InputPayload::Envelope(e) => (KIND_ENVELOPE, e.payload.ordinal()),
        };
        CanonicalKey {
            apply_at: self.apply_at,
            type_ordinal: (kind << 8) | leaf,
            source: self.source,
            sequence: self.sequence,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::SimCommand;
    use crate::envelope::{Envelope, EnvelopePayload, Intent};
    use crate::event::CrossEvent;
    use types::glam::Vec3;
    use types::{
        BlockId, BlockPos, DimensionId, EntityId, EntityState, ShardId, ShardPos, WorldId,
    };

    fn input(seq: u64, source: u32, payload: InputPayload) -> StampedInput {
        StampedInput {
            apply_at: Tick(42),
            source: InputSource(source),
            sequence: seq,
            payload,
        }
    }

    fn set_block() -> InputPayload {
        InputPayload::Command(SimCommand::SetBlock {
            pos: BlockPos::ZERO,
            block: BlockId::AIR,
        })
    }

    fn detonate() -> InputPayload {
        InputPayload::Event(CrossEvent::Detonate {
            center: Vec3::ZERO,
            power: 1.0,
            radius: 1.0,
        })
    }

    fn migrate() -> InputPayload {
        let source = ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0));
        let dest = ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(1, 0));
        InputPayload::Envelope(Envelope::stamp(
            source,
            Intent {
                destination: dest,
                payload: EnvelopePayload::MigrateEntity {
                    entity: EntityId::from_parts(1, 1),
                    state: EntityState::default(),
                    desc: types::EntityDesc::default(),
                    actor: false,
                    source: None,
                    actor_motion_ticks: None,
                },
            },
            Tick(41),
            1,
        ))
    }

    #[test]
    fn kind_ordinal_separates_payload_kinds() {
        let k_cmd = input(1, 1, set_block()).canonical_key();
        let k_evt = input(1, 1, detonate()).canonical_key();
        let k_env = input(1, 1, migrate()).canonical_key();
        assert!(k_cmd.type_ordinal < k_evt.type_ordinal);
        assert!(k_evt.type_ordinal < k_env.type_ordinal);
    }

    #[test]
    fn key_orders_by_source_then_sequence() {
        let a = input(5, 2, set_block()).canonical_key();
        let b = input(1, 3, set_block()).canonical_key();
        assert!(a < b, "source 优先于 sequence");
        let c = input(2, 2, set_block()).canonical_key();
        let d = input(9, 2, set_block()).canonical_key();
        assert!(c < d, "同 source 按 sequence");
    }

    #[test]
    fn apply_at_dominates() {
        let mut early = input(1, 9, set_block());
        early.apply_at = Tick(1);
        let late = input(1, 1, set_block()); // apply_at 42
        assert!(early.canonical_key() < late.canonical_key());
    }

    #[test]
    fn sorting_a_batch_is_deterministic() {
        let mut batch = [
            input(1, 1, detonate()),
            input(2, 1, set_block()),
            input(1, 1, set_block()),
        ];
        batch.sort_by_key(|i| i.canonical_key());
        // §9.6:类型序号先于 source/sequence → 全部 Command 排在任何 Event 之前,
        // Command 内部再按 sequence
        assert!(matches!(batch[0].payload, InputPayload::Command(_)));
        assert_eq!(batch[0].sequence, 1);
        assert!(matches!(batch[1].payload, InputPayload::Command(_)));
        assert_eq!(batch[1].sequence, 2);
        assert!(matches!(batch[2].payload, InputPayload::Event(_)));
        // 类型主导:sequence 更小的 Event 仍排在 sequence 更大的 Command 之后
        assert!(input(0, 1, detonate()).canonical_key() > input(9, 1, set_block()).canonical_key());
    }
}
