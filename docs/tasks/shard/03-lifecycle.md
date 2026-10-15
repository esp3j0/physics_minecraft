# 任务 03:分片生命周期状态机

## 目标

蓝图 §3:"Created→Active→Draining→Stopped;懒创建;空片+无票 30s despawn(§9.3)"。
核心 crate 不看墙钟,30 秒换算为逻辑拍:`30s × 60Hz = 1800` 拍(§9.3 决议)。
"空片+无票"的判据(实体数、票务)由调度器掌握,本模块只跟踪阶段与空闲计时,
提供判定原语(见 README 决策 4:本轮只接线不自动化)。

## 必读

- `crates/types/src/tick.rs`
- `crates/shard/src/lifecycle.rs`(骨架)
- `docs/design/codebase-layout.md` §3 lifecycle 行、§6(`despawn_after_secs`=30)

## 交付物

`crates/shard/src/lifecycle.rs`:类型 + 内嵌单元测试。

## 接口设计(签名固定)

```rust
/// 生命周期阶段。Created 阶段由"记录不存在"表达(懒创建:首次活动即 Active)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stage {
    Active,
    Draining,
    Stopped,
}

#[derive(Debug, Clone, Copy)]
pub struct LifecycleConfig {
    /// 空闲多少拍后可 despawn(默认 1800 = 30s × 60Hz,§9.3)。
    pub despawn_after_ticks: u64,
}

impl Default for LifecycleConfig {
    fn default() -> Self {
        Self { despawn_after_ticks: 1800 }
    }
}

/// 单分片生命周期记录。非法阶段转移 = 一致性违规 → panic(fail-stop)。
#[derive(Debug)]
pub struct ShardLifecycle {
    stage: Stage,
    last_activity: Tick,
    drained: bool,
    cfg: LifecycleConfig,
}

impl ShardLifecycle {
    /// 懒创建:首次活动即 Active。
    pub fn activate(at: Tick, cfg: LifecycleConfig) -> Self;

    /// 记录一次活动(任何输入/实体存在/票),重置空闲计时。
    /// Stopped 之后仍有活动 → panic。
    pub fn note_activity(&mut self, at: Tick);

    pub fn stage(&self) -> Stage;

    /// 进入 Draining(仅 Active 合法)。
    pub fn begin_draining(&mut self);

    /// 标记排空完成(仅 Draining 合法;outbox/信封清空后由调度器调用)。
    pub fn note_drained(&mut self);

    /// 停止(仅 Draining 且已排空合法;Active 不得直接 Stopped)。
    pub fn stop(&mut self);

    /// 空闲判定:Active 且 `now - last_activity >= despawn_after_ticks`。
    /// 非 Active 恒 false。调用方须自行确认"空片+无票"后才可据此 despawn。
    pub fn idle_expired(&self, now: Tick) -> bool;

    pub fn is_stopped(&self) -> bool;
}
```

## 语义细则

- `idle_expired` 的拍运算用 `saturating_sub`,不 panic;
- `note_activity` 在 Draining 阶段合法(排空期间迁移输入仍可能到达),只重置计时
  不改阶段。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `activate_starts_active` | `activate(Tick(0), cfg)` → `stage() == Active` |
| `idle_expires_after_despawn_window` | despawn=1800:activate@0,`idle_expired(1799)==false`,`idle_expired(1800)==true` |
| `activity_resets_idle_clock` | activate@0,note@1000 → `idle_expired(2799)==false`,`idle_expired(2800)==true` |
| `drain_then_drained_then_stop_is_legal` | 完整链路后 `is_stopped()` |
| `active_cannot_stop_directly` | `#[should_panic]` Active 直接 stop |
| `stop_requires_drained` | `#[should_panic]` begin_draining 后未 note_drained 就 stop |
| `stopped_rejects_activity` | `#[should_panic]` stop 后 note_activity |
| `draining_rejects_second_drain` | `#[should_panic]` 重复 begin_draining |
| `non_active_never_idle_expired` | Draining 阶段 `idle_expired` 恒 false |

## 完成定义

见 [README 公共完成定义](README.md)。
