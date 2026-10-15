# 任务 01:TickCoordinator(tick 协调器)

## 目标

REQ-001 §4.4:逻辑时钟单调 +1、不看墙钟、无追赶(lag 由 app 层对比墙钟后仅上报,
不进核)。本模块是全 crate 最小的一块,也是对仓库风格的熟悉。

## 必读

- `crates/types/src/tick.rs`(`Tick::next()` 已实现,溢出即 panic——直接复用)
- `crates/shard/src/coordinator.rs`(骨架)
- `docs/design/codebase-layout.md` §3 coordinator 行

## 交付物

`crates/shard/src/coordinator.rs`:类型 + 单元测试(`#[cfg(test)] mod tests` 内嵌)。

## 接口设计(签名固定)

```rust
/// 逻辑 tick 协调器(REQ-001 §4.4):单调 +1,不看墙钟,无追赶。
#[derive(Debug, Clone)]
pub struct TickCoordinator {
    current: Tick,
}

impl TickCoordinator {
    /// 从给定拍起动(通常 `Tick::ZERO`)。
    pub fn new(start: Tick) -> Self;

    /// 当前拍(下一个将被执行的拍)。
    pub fn tick(&self) -> Tick;

    /// 推进恰好一拍,返回新拍号。溢出由 `Tick::next` panic(fail-stop)。
    pub fn advance(&mut self) -> Tick;
}
```

字段 `current` 私有;不引入任何时间/随机/IO 概念。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `new_starts_at_given_tick` | `new(Tick(41)).tick() == Tick(41)` |
| `advance_increments_exactly_one_per_call` | 连续 3 次 advance:Tick(41)→42→43→44 |
| `advance_returns_new_tick_value` | 返回值 == advance 之后的 `tick()` |
| `read_does_not_advance` | 多次调用 `tick()` 不改变状态 |

## 完成定义

见 [README 公共完成定义](README.md)。模块内 `cargo test` 全绿。
