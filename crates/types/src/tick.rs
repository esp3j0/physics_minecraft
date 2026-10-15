//! 逻辑时钟。

use serde::{Deserialize, Serialize};

/// 全局逻辑 tick(REQ-001 共识 4):单调 +1,不查墙钟、无追赶(lag 只上报)。
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize,
)]
pub struct Tick(pub u64);

impl Tick {
    pub const ZERO: Self = Self(0);

    /// 推进一拍。u64 溢出视为一致性违规 → panic(fail-stop,蓝图 §4)。
    pub fn next(self) -> Self {
        Self(self.0.checked_add(1).expect("tick 溢出(u64 上限)"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tick_advances_monotonically() {
        let t = Tick::ZERO;
        assert_eq!(t.next(), Tick(1));
        assert!(t < t.next());
        assert_eq!(Tick(41).next().next(), Tick(43));
    }
}
