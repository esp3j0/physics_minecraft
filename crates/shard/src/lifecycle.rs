//! 生命周期:Active→Draining→Stopped、懒创建、空闲 despawn 判定(蓝图 §3)。
//!
//! 蓝图 §3:"Created→Active→Draining→Stopped;懒创建;空片+无票 30s despawn(§9.3)"。
//! 核心 crate 不看墙钟,30 秒换算为逻辑拍:`30s × 60Hz = 1800` 拍(§9.3 决议)。
//! "空片+无票"的判据(实体数、票务)由调度器掌握,本模块只跟踪阶段与空闲计时,
//! 提供判定原语(README 决策 4:本轮只接线不自动化)。

use serde::{Deserialize, Serialize};
use types::Tick;

/// 生命周期快照(REQ-005 任务书 02,签名固定):阶段 + 空闲计时 + 排空标志。
/// cfg 不入快照(restore 用当前配置);despawn 计时不被重置靠 `last_activity`
/// 原样恢复(AC-C5)。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifecycleSnapshot {
    pub stage: Stage,
    pub last_activity: Tick,
    pub drained: bool,
}

/// 生命周期阶段。Created 阶段由"记录不存在"表达(懒创建:首次活动即 Active)。
/// serde(REQ-005 D7):阶段入 [`LifecycleSnapshot`] 随快照序列化。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Stage {
    Active,
    Draining,
    Stopped,
}

/// 生命周期配置。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LifecycleConfig {
    /// 空闲多少拍后可 despawn(默认 1800 = 30s × 60Hz,§9.3)。
    pub despawn_after_ticks: u64,
}

impl Default for LifecycleConfig {
    fn default() -> Self {
        Self {
            despawn_after_ticks: 1800,
        }
    }
}

/// 单分片生命周期记录。非法阶段转移 = 一致性违规 → panic(fail-stop,蓝图 §4)。
#[derive(Debug)]
pub struct ShardLifecycle {
    stage: Stage,
    last_activity: Tick,
    drained: bool,
    cfg: LifecycleConfig,
}

impl ShardLifecycle {
    /// 懒创建:首次活动即 Active。
    pub fn activate(at: Tick, cfg: LifecycleConfig) -> Self {
        Self {
            stage: Stage::Active,
            last_activity: at,
            drained: false,
            cfg,
        }
    }

    /// 快照(REQ-005 D7):阶段 + 空闲计时 + 排空标志;cfg 不入(恢复用当前
    /// 配置,计时基准 `last_activity` 是本表唯一必须跨 checkpoint 保留的状态)。
    pub fn snapshot(&self) -> LifecycleSnapshot {
        LifecycleSnapshot {
            stage: self.stage,
            last_activity: self.last_activity,
            drained: self.drained,
        }
    }

    /// 从快照恢复(REQ-005 D7):cfg 用调用方当前配置(不入快照)。
    pub fn restore(snap: LifecycleSnapshot, cfg: LifecycleConfig) -> Self {
        Self {
            stage: snap.stage,
            last_activity: snap.last_activity,
            drained: snap.drained,
            cfg,
        }
    }

    /// 记录一次活动(任何输入/实体存在/票),重置空闲计时。
    /// Draining 阶段合法(排空期间迁移输入仍可能到达),只重置计时不改阶段;
    /// Stopped 之后仍有活动 → panic(fail-stop)。
    pub fn note_activity(&mut self, at: Tick) {
        match self.stage {
            Stage::Active | Stage::Draining => self.last_activity = at,
            Stage::Stopped => panic!("非法阶段转移:Stopped 之后仍有活动(拍 {}),调用方 bug", at.0),
        }
    }

    /// 当前阶段。
    pub fn stage(&self) -> Stage {
        self.stage
    }

    /// 进入 Draining(仅 Active 合法)。
    pub fn begin_draining(&mut self) {
        if self.stage != Stage::Active {
            panic!(
                "非法阶段转移:begin_draining 只允许从 Active 进入,当前 {:?}",
                self.stage
            );
        }
        self.stage = Stage::Draining;
    }

    /// 标记排空完成(仅 Draining 合法;outbox/信封清空后由调度器调用)。
    pub fn note_drained(&mut self) {
        if self.stage != Stage::Draining {
            panic!(
                "非法阶段转移:note_drained 只允许在 Draining 阶段,当前 {:?}",
                self.stage
            );
        }
        self.drained = true;
    }

    /// 停止(仅 Draining 且已排空合法;Active 不得直接 Stopped)。
    pub fn stop(&mut self) {
        if self.stage != Stage::Draining || !self.drained {
            panic!(
                "非法阶段转移:stop 只允许在 Draining 且已排空,当前 {:?}(drained={})",
                self.stage, self.drained
            );
        }
        self.stage = Stage::Stopped;
    }

    /// 空闲判定:Active 且 `now - last_activity >= despawn_after_ticks`。
    /// 非 Active 恒 false。调用方须自行确认"空片+无票"后才可据此 despawn。
    pub fn idle_expired(&self, now: Tick) -> bool {
        if self.stage != Stage::Active {
            return false;
        }
        // saturating_sub:now 早于 last_activity(时钟回拨属调用方问题)时不 panic。
        now.0.saturating_sub(self.last_activity.0) >= self.cfg.despawn_after_ticks
    }

    /// 是否已停止。
    pub fn is_stopped(&self) -> bool {
        self.stage == Stage::Stopped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn activate_starts_active() {
        let lc = ShardLifecycle::activate(Tick(0), LifecycleConfig::default());
        assert_eq!(lc.stage(), Stage::Active);
    }

    #[test]
    fn idle_expires_after_despawn_window() {
        let lc = ShardLifecycle::activate(Tick(0), LifecycleConfig::default());
        assert!(!lc.idle_expired(Tick(1799)));
        assert!(lc.idle_expired(Tick(1800)));
    }

    #[test]
    fn activity_resets_idle_clock() {
        let mut lc = ShardLifecycle::activate(Tick(0), LifecycleConfig::default());
        lc.note_activity(Tick(1000));
        assert!(!lc.idle_expired(Tick(2799)));
        assert!(lc.idle_expired(Tick(2800)));
    }

    #[test]
    fn drain_then_drained_then_stop_is_legal() {
        let mut lc = ShardLifecycle::activate(Tick(0), LifecycleConfig::default());
        lc.begin_draining();
        assert_eq!(lc.stage(), Stage::Draining);
        lc.note_drained();
        lc.stop();
        assert!(lc.is_stopped());
        assert_eq!(lc.stage(), Stage::Stopped);
    }

    #[test]
    #[should_panic(expected = "非法阶段转移")]
    fn active_cannot_stop_directly() {
        let mut lc = ShardLifecycle::activate(Tick(0), LifecycleConfig::default());
        lc.stop(); // Active 不得直接 Stopped → fail-stop
    }

    #[test]
    #[should_panic(expected = "非法阶段转移")]
    fn stop_requires_drained() {
        let mut lc = ShardLifecycle::activate(Tick(0), LifecycleConfig::default());
        lc.begin_draining();
        lc.stop(); // 未 note_drained → fail-stop
    }

    #[test]
    #[should_panic(expected = "非法阶段转移")]
    fn stopped_rejects_activity() {
        let mut lc = ShardLifecycle::activate(Tick(0), LifecycleConfig::default());
        lc.begin_draining();
        lc.note_drained();
        lc.stop();
        lc.note_activity(Tick(9999)); // Stopped 之后仍有活动 → fail-stop
    }

    #[test]
    #[should_panic(expected = "非法阶段转移")]
    fn draining_rejects_second_drain() {
        let mut lc = ShardLifecycle::activate(Tick(0), LifecycleConfig::default());
        lc.begin_draining();
        lc.begin_draining(); // 重复 begin_draining → fail-stop
    }

    #[test]
    fn non_active_never_idle_expired() {
        let mut lc = ShardLifecycle::activate(Tick(0), LifecycleConfig::default());
        lc.begin_draining();
        // Draining 阶段无论空闲多久都不判空闲。
        assert!(!lc.idle_expired(Tick(1799)));
        assert!(!lc.idle_expired(Tick(1800)));
        assert!(!lc.idle_expired(Tick(100_000)));
    }
}
