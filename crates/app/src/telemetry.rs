//! tick 耗时度量:P95 滑动窗口告警(REQ-001 §9.2)——**只度量不决策**,告警仅供
//! 记录/上报,驱动不因此改变行为。
//!
//! 度量对象 = **整拍(含屏障)非单分片**:app 从调度器屏障外只能测
//! `session.tick()` 全程耗时,即为度量口径。
//!
//! 计时归属:std `Instant` 属 app::telemetry(clippy.toml 顶部注释授权),但本类型
//! 自身**完全不碰时钟**——接口本意是外部计时:调用方(驱动)测 `t0.elapsed()` 喂给
//! [`TickTelemetry::record`],record 只收 [`Duration`](std::time::Duration),
//! 因此无需任何 `#[allow(clippy::disallowed_methods)]`。

use std::collections::VecDeque;

/// tick 耗时度量:P95 滑动窗口告警(§9.2,只度量不决策)。
///
/// 计时用 std Instant——确定性核的墙钟禁令只针对核 crate
/// (clippy.toml 顶部注释授权"计时只属于 app::telemetry"),故驱动侧计时处
/// 局部 allow 并注明;本类型只接收现成 Duration,自身无时钟。
#[derive(Debug, Clone)]
pub struct TickTelemetry {
    /// 滑动窗口:整拍耗时(毫秒),容量 = `window_ticks`,FIFO 淘汰最旧样本。
    window: VecDeque<f32>,
    /// 窗口长度(拍),默认 60(§9.2"持续 60 tick")。
    window_ticks: usize,
    /// P95 告警阈值(毫秒),默认 5(§9.2);严格大于才告警。
    threshold_ms: f32,
}

impl TickTelemetry {
    /// 构造:`threshold_ms` = 告警阈值,`window_ticks` = 滑窗长度(拍)。
    ///
    /// # Panics
    ///
    /// `window_ticks == 0` → panic(空窗无法定义 P95,一致性违规 = fail-stop)。
    pub fn new(threshold_ms: f32, window_ticks: usize) -> Self {
        assert!(
            window_ticks > 0,
            "window_ticks 必须大于 0(空窗无法定义 P95,fail-stop)"
        );
        Self {
            window: VecDeque::with_capacity(window_ticks),
            window_ticks,
            threshold_ms,
        }
    }

    /// 记录一拍耗时(测试喂合成 Duration;不依赖时钟)。
    pub fn record(&mut self, elapsed: std::time::Duration) {
        let ms = elapsed.as_secs_f32() * 1000.0;
        // FIFO 淘汰:满窗先弹最旧再入新,滑窗 = 最新 window_ticks 个样本。
        if self.window.len() == self.window_ticks {
            self.window.pop_front();
        }
        self.window.push_back(ms);
    }

    /// 窗口已满且 P95 > 阈值 → true(告警持续期间恒 true 直到恢复)。
    ///
    /// 纯查询:不改变任何状态;恢复(新样本把高值挤出窗口)后自然回 false。
    pub fn alert(&self) -> bool {
        self.window.len() == self.window_ticks && self.p95_ms() > self.threshold_ms
    }

    /// 窗口内样本数(测试/上报)。
    pub fn samples(&self) -> usize {
        self.window.len()
    }

    /// 当前窗口 P95(毫秒;空窗口 → 0.0)。
    ///
    /// P95 定义:窗口样本升序,取第 `ceil(0.95 × n)` 个(1-based)。
    pub fn p95_ms(&self) -> f32 {
        if self.window.is_empty() {
            return 0.0;
        }
        let n = self.window.len();
        let mut sorted: Vec<f32> = self.window.iter().copied().collect();
        // total_cmp:全序(对 NaN 也确定),排序不受输入顺序影响。
        sorted.sort_by(f32::total_cmp);
        // 1-based 秩:0.95n ≤ n 恒成立 → ceil 秩 ∈ [1, n],rank-1 必为合法下标。
        let rank = (0.95 * n as f64).ceil() as usize;
        sorted[rank - 1]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    /// 毫秒 → Duration(测试喂合成耗时,不碰时钟)。
    fn ms(v: u64) -> Duration {
        Duration::from_millis(v)
    }

    /// f32 存 ms 有 ~1e-5 量级舍入误差(as_secs_f32 往返),断言用 1e-3 ms 容差。
    fn assert_ms(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 1e-3,
            "期望 {expected} ms,实际 {actual} ms"
        );
    }

    /// 测试清单 1:60 个 1..=60ms → P95 = 57(ceil(0.95×60)=57,升序第 57 个)。
    #[test]
    fn p95_of_known_samples() {
        let mut t = TickTelemetry::new(5.0, 60);
        for i in 1..=60 {
            t.record(ms(i));
        }
        assert_eq!(t.samples(), 60, "窗口满 60 样本");
        assert_ms(t.p95_ms(), 57.0);
    }

    /// 测试清单 2:59 个 100ms 样本 → alert == false(窗口未满不告警)。
    #[test]
    fn no_alert_before_window_full() {
        let mut t = TickTelemetry::new(5.0, 60);
        for _ in 0..59 {
            t.record(ms(100));
        }
        assert_eq!(t.samples(), 59, "未满 60");
        assert!(!t.alert(), "窗口未满不告警(即使 P95=100 > 5)");
    }

    /// 测试清单 3:60 个 10ms(阈值 5)→ alert == true。
    #[test]
    fn alert_after_sustained_slowdown() {
        let mut t = TickTelemetry::new(5.0, 60);
        for _ in 0..60 {
            t.record(ms(10));
        }
        assert!(t.alert(), "满窗 P95=10 > 5 → 告警");
    }

    /// 测试清单 4:60×10ms 告警后 → 60×1ms → alert == false(纯查询自然恢复)。
    #[test]
    fn alert_clears_after_recovery() {
        let mut t = TickTelemetry::new(5.0, 60);
        for _ in 0..60 {
            t.record(ms(10));
        }
        assert!(t.alert(), "前置:慢窗口告警中");
        for _ in 0..60 {
            t.record(ms(1));
        }
        assert!(!t.alert(), "高值被新样本挤出窗口后恢复");
        assert_ms(t.p95_ms(), 1.0);
    }

    /// 测试清单 5:新建 → samples()==0、p95_ms()==0.0、alert()==false。
    #[test]
    fn empty_window_p95_is_zero() {
        let t = TickTelemetry::new(5.0, 60);
        assert_eq!(t.samples(), 0);
        assert_eq!(t.p95_ms(), 0.0, "空窗口 P95 = 0.0");
        assert!(!t.alert(), "空窗口不告警");
    }

    /// 测试清单 6:灌 70 个样本 → samples()==60,p95 只反映最新 60 个
    /// (最旧被 FIFO 淘汰:60×1ms 后再灌 10×100ms,若保留最旧则 P95 应为 1.0,
    /// 实际按最新 60 个(50×1ms+10×100ms)算 P95 = 100)。
    #[test]
    fn window_evicts_oldest() {
        let mut t = TickTelemetry::new(5.0, 60);
        for _ in 0..60 {
            t.record(ms(1));
        }
        for _ in 0..10 {
            t.record(ms(100));
        }
        assert_eq!(t.samples(), 60, "容量封顶,70 灌入只留 60");
        assert_ms(t.p95_ms(), 100.0);
        assert!(t.alert(), "最新 10 拍 100ms 把 P95 顶到 100 > 5");
    }
}
