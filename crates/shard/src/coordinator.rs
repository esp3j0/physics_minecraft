//! TickCoordinator:逻辑 tick 单调 +1、不看墙钟、lag 只上报(蓝图 §3)。

use types::Tick;

/// 逻辑 tick 协调器(REQ-001 §4.4):单调 +1,不看墙钟,无追赶。
#[derive(Debug, Clone)]
pub struct TickCoordinator {
    current: Tick,
}

impl TickCoordinator {
    /// 从给定拍起动(通常 `Tick::ZERO`)。
    pub fn new(start: Tick) -> Self {
        Self { current: start }
    }

    /// 当前拍(下一个将被执行的拍)。
    pub fn tick(&self) -> Tick {
        self.current
    }

    /// 推进恰好一拍,返回新拍号。溢出由 `Tick::next` panic(fail-stop)。
    pub fn advance(&mut self) -> Tick {
        self.current = self.current.next();
        self.current
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_starts_at_given_tick() {
        let coord = TickCoordinator::new(Tick(41));
        assert_eq!(coord.tick(), Tick(41));
    }

    #[test]
    fn advance_increments_exactly_one_per_call() {
        let mut coord = TickCoordinator::new(Tick(41));
        assert_eq!(coord.advance(), Tick(42));
        assert_eq!(coord.advance(), Tick(43));
        assert_eq!(coord.advance(), Tick(44));
    }

    #[test]
    fn advance_returns_new_tick_value() {
        let mut coord = TickCoordinator::new(Tick::ZERO);
        let returned = coord.advance();
        assert_eq!(returned, coord.tick());
    }

    #[test]
    fn read_does_not_advance() {
        let mut coord = TickCoordinator::new(Tick(7));
        for _ in 0..5 {
            assert_eq!(coord.tick(), Tick(7));
        }
        // 读操作不得有副作用:后续 advance 仍是恰好 +1。
        assert_eq!(coord.advance(), Tick(8));
    }
}
