//! S02 压力基线(总架构师放行评论,#24):
//! - 百分位计算的单元测试(线性插值、排序不变量);
//! - `--quick` 冒烟进 CI:每个场景最小一档各 1 次,遥测断言成立
//!   (场景真的发生了:实体数、破坏方块、撞击/塌落/凝固计数)。
//!
//! 共享实现经 `#[path]` 引入 example(单一来源;example 的 `main` 只在
//! bin target 生效,不冲突)。

#[path = "../examples/stress_baseline.rs"]
mod stress;

use stress::percentile;

/// 奇数长度:q=0.5 取中位;边界 q=0/1 取 min/max。
#[test]
fn percentile_odd_median_and_bounds() {
    let v = vec![3.0, 1.0, 2.0];
    assert_eq!(percentile(&v, 0.5), 2.0);
    assert_eq!(percentile(&v, 0.0), 1.0);
    assert_eq!(percentile(&v, 1.0), 3.0);
}

/// 偶数长度:q=0.5 线性插值取两中位均值;q=0.25 插值。
#[test]
fn percentile_even_interpolates() {
    let v = vec![1.0, 2.0, 3.0, 4.0];
    assert_eq!(percentile(&v, 0.5), 2.5);
    assert_eq!(percentile(&v, 0.25), 1.75);
    assert_eq!(percentile(&v, 0.0), 1.0);
    assert_eq!(percentile(&v, 1.0), 4.0);
}

/// 单样本恒等;20 样本 q=0.95:pos = 0.95×19 = 18.05,插值 18 + 0.05×1
/// (会变红的实现:取上/下整数下标 18 或 19)。
#[test]
fn percentile_single_and_large() {
    assert_eq!(percentile(&[7.5], 0.95), 7.5);
    let v: Vec<f64> = (0..20).map(|i| i as f64).collect();
    let expect = 18.0 + 0.05;
    assert!((percentile(&v, 0.95) - expect).abs() < 1e-9);
}

/// 会变红的实现:按未排序样本直接取下标。
#[test]
fn percentile_sorts_input_order() {
    let v = vec![10.0, 0.0, 5.0];
    assert_eq!(percentile(&v, 0.0), 0.0);
    assert_eq!(percentile(&v, 1.0), 10.0);
}

/// `--quick` 全场景冒烟(CI):run_once 内的遥测断言 = 场景真的发生了
/// (loose/rain 实体数与凝固、bridge 塌落复合体、blast 破坏方块、
/// wall-impact 撞击破坏、cross-shard 两档);不 panic 即全过。
#[test]
fn quick_scenarios_actually_happen() {
    let tiers = stress::run_all(true, None);
    // quick = 每场景最小档全跑(loose/bridge/blast/wall/rain 首档 +
    // cross-shard 两档),至少 7 档。
    assert!(tiers.len() >= 7, "quick 应覆盖 ≥ 7 档,实得 {}", tiers.len());
    let mut scenarios: Vec<&str> = tiers.iter().map(|t| t.scenario.as_str()).collect();
    scenarios.sort();
    scenarios.dedup();
    assert_eq!(
        scenarios,
        vec![
            "blast",
            "bridge",
            "compound-rain",
            "cross-shard",
            "loose",
            "wall-impact"
        ],
        "quick 必须覆盖全部 6 场景"
    );
}
