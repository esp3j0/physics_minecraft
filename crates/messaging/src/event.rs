//! 跨片事件:广播共享参数,各分片用本地实体状态独立求值(demo D)。

use serde::{Deserialize, Serialize};
use types::glam::Vec3;

/// 跨片事件载荷。命令调度的事件当拍生效;涌现事件由分片在 tick N 末产出,
/// 调度器盖章 N+1 生效(REQ-001 §4.2)。变体只允许**追加**。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CrossEvent {
    /// 爆炸:冲量 = dir * power * (1 - dist/radius),线性衰减,不除以质量(demo D 公式)。
    Detonate {
        center: Vec3,
        power: f32,
        radius: f32,
    },
}

impl CrossEvent {
    /// 枚举声明序(canonical 类型序号的低位)。
    pub const fn ordinal(self) -> u16 {
        match self {
            Self::Detonate { .. } => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_ordinals_follow_declared_order() {
        let e = CrossEvent::Detonate {
            center: Vec3::ZERO,
            power: 1.0,
            radius: 1.0,
        };
        assert_eq!(e.ordinal(), 0);
    }
}
