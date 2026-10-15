# 任务 02:tick 耗时告警(telemetry.rs)

## 目标

REQ-001 §9.2:分片 step 耗时 P95 > 5ms 持续 60 tick → 告警;**只度量不决策**
(驱动不因此改变行为)。app 从调度器屏障外只能测整拍耗时
(`session.tick()` 全程),即为度量对象——doc 注明"整拍(含屏障)非单分片"。

## 必读

- `docs/tasks/app/README.md` 决策 8/10
- 根目录 `clippy.toml`(顶部注释:边缘 crate 局部 allow 的授权条款)

## 交付物

`crates/app/src/telemetry.rs`:类型 + 内嵌测试。

## 接口设计(签名固定)

```rust
/// tick 耗时度量:P95 滑动窗口告警(§9.2,只度量不决策)。
///
/// 计时用 std Instant——确定性核的墙钟禁令只针对核 crate
/// (clippy.toml 顶部注释授权"计时只属于 app::telemetry"),
/// 故此处局部 allow 并注明。
pub struct TickTelemetry {
    window: VecDeque<f32>,   // 毫秒,容量 = window_ticks
    window_ticks: usize,     // 默认 60
    threshold_ms: f32,       // 默认 5(§9.2)
    /* 私有统计字段自定 */
}

impl TickTelemetry {
    pub fn new(threshold_ms: f32, window_ticks: usize) -> Self;

    /// 记录一拍耗时(测试喂合成 Duration;不依赖时钟)。
    pub fn record(&mut self, elapsed: std::time::Duration);

    /// 窗口已满且 P95 > 阈值 → true(告警持续期间恒 true 直到恢复)。
    pub fn alert(&self) -> bool;

    /// 窗口内样本数(测试/上报)。
    pub fn samples(&self) -> usize;

    /// 当前窗口 P95(毫秒;空窗口 → 0.0)。
    pub fn p95_ms(&self) -> f32;
}
```

## 语义细则

1. P95 定义:窗口样本升序,取第 `ceil(0.95 × n)` 个(1-based);窗口未满不告警;
2. 滑窗 = 最新 `window_ticks` 个样本(FIFO 淘汰);
3. `alert()` 是纯查询:恢复(新样本把高值挤出窗口)后自然回 false;
4. `record` 里如需 `Instant::now`,局部
   `#[allow(clippy::disallowed_methods)]` + 理由注释(见接口 doc);
   但接口本意是**外部计时、record 只收 Duration**——驱动传
   `t0.elapsed()`,telemetry 自身不碰时钟更干净(推荐:telemetry 完全无时钟)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `p95_of_known_samples` | 60 个 1..=60ms → P95 = 57(ceil(0.95×60)=57,升序第 57 个) |
| `no_alert_before_window_full` | 59 个 100ms 样本 → alert == false |
| `alert_after_sustained_slowdown` | 60 个 10ms(阈值 5)→ alert == true |
| `alert_clears_after_recovery` | 60×10ms 告警后 → 60×1ms → alert == false |
| `empty_window_p95_is_zero` | 新建 → samples()==0、p95_ms()==0.0、alert()==false |
| `window_evicts_oldest` | 灌 70 个样本 → samples()==60,p95 只反映最新 60 个 |

## 完成定义

见 [README 公共完成定义](README.md)。
