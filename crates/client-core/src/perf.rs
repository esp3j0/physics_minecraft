//! 客户端性能统计纯函数(REQ-013 C01b-3 §1.3/§1.4):零墙钟,入参-only。

/// 帧耗时列表(毫秒)汇总为 `(mean_fps, min_fps)`。
///
/// - `mean_fps = 1000.0 / mean_dt_ms`
/// - `min_fps = 1000.0 / max_dt_ms`
/// - 空切片或耗时非正时返回 `(0.0, 0.0)`。
pub fn fps_summary(dts_ms: &[f64]) -> (f64, f64) {
    if dts_ms.is_empty() {
        return (0.0, 0.0);
    }
    let mut sum = 0.0_f64;
    let mut max_dt = 0.0_f64;
    for &dt in dts_ms {
        sum += dt;
        if dt > max_dt {
            max_dt = dt;
        }
    }
    let mean_fps = if sum > 0.0 {
        1000.0 * (dts_ms.len() as f64) / sum
    } else {
        0.0
    };
    let min_fps = if max_dt > 0.0 { 1000.0 / max_dt } else { 0.0 };
    (mean_fps, min_fps)
}

/// 异步网格构建耗时累计遥测(纯数据结构,不碰时钟)。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MeshPerf {
    pub count: u64,
    pub total_us: u64,
    pub max_us: u64,
}

/// 纯累加器:将单段网格构建耗时(微秒)并入 [`MeshPerf`]。
pub fn acc_mesh_perf(p: &mut MeshPerf, us: u64) {
    p.count += 1;
    p.total_us += us;
    p.max_us = p.max_us.max(us);
}

/// 编辑风暴 FPS 统计窗口的闭区间下标 `(start, end)`(对应 `&history[start..=end]`)。
///
/// 窗口为 `[first_dig, min(last_edit_upload + 10, len - 1)]`;
/// 无编辑、历史为空、起点越界或区间倒置时返回 `None`。
pub fn storm_window(
    first_dig: Option<u64>,
    last_edit_upload: Option<u64>,
    len: u64,
) -> Option<(usize, usize)> {
    let start = first_dig?;
    let last = last_edit_upload?;
    if len == 0 || start >= len {
        return None;
    }
    let end = last.saturating_add(10).min(len - 1);
    if end < start {
        return None;
    }
    Some((start as usize, end as usize))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// MS8:手算 `[16.0, 16.0, 32.0]` → `(46.875, 31.25)`;空切片 `(0.0, 0.0)`。
    #[test]
    fn ms8_fps_summary() {
        let (mean_fps, min_fps) = fps_summary(&[16.0, 16.0, 32.0]);
        assert_eq!(mean_fps, 46.875, "3000 / 64 = 46.875");
        assert_eq!(min_fps, 31.25, "1000 / 32 = 31.25");
        assert_eq!(fps_summary(&[]), (0.0, 0.0), "空切片返回 (0.0, 0.0)");
    }

    /// MS9:`acc_mesh_perf` 依次喂入 1500 / 2500 / 0 → count=3, total_us=4000, max_us=2500。
    #[test]
    fn ms9_acc_mesh_perf() {
        let mut p = MeshPerf::default();
        acc_mesh_perf(&mut p, 1500);
        assert_eq!(
            p,
            MeshPerf {
                count: 1,
                total_us: 1500,
                max_us: 1500,
            }
        );
        acc_mesh_perf(&mut p, 2500);
        assert_eq!(
            p,
            MeshPerf {
                count: 2,
                total_us: 4000,
                max_us: 2500,
            }
        );
        acc_mesh_perf(&mut p, 0);
        assert_eq!(
            p,
            MeshPerf {
                count: 3,
                total_us: 4000,
                max_us: 2500,
            }
        );
    }

    /// MS10:`storm_window` 空输入 / 正常窗口 / `+10` 截断到 `len - 1`。
    #[test]
    fn ms10_storm_window() {
        assert_eq!(storm_window(None, None, 100), None);
        assert_eq!(storm_window(Some(5), Some(9), 100), Some((5, 19)));
        assert_eq!(storm_window(Some(5), Some(95), 100), Some((5, 99)));
        assert_eq!(storm_window(Some(5), None, 100), None);
        assert_eq!(storm_window(Some(0), Some(0), 0), None);
    }
}
