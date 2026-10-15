//! 凝固桥的几何部分(demo C):24 轴对齐朝向、最近朝向匹配、位置 snap。
//! 触发策略(P2 + 睡眠兜底 + 误差升级)在 sim.rs。

use types::BlockPos;
use types::glam::{Mat3, Quat, Vec3};

/// 24 个轴对齐朝向(立方体旋转群:符号置换矩阵,det = +1)。
/// demo C 结论:任何触发策略下 snap 去旋转都不能省略——睡眠角阈值太松。
pub(crate) fn axis_aligned_orientations() -> Vec<Quat> {
    let perms: [[usize; 3]; 6] = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let mut out = Vec::with_capacity(24);
    for sx in [-1.0f32, 1.0] {
        for sy in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                for p in perms {
                    let mut m = [[0.0f32; 3]; 3];
                    for (row, &axis) in p.iter().enumerate() {
                        m[row][axis] = [sx, sy, sz][row];
                    }
                    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
                    if det > 0.0 {
                        out.push(Quat::from_mat3(&Mat3::from_cols_array_2d(&m)));
                    }
                }
            }
        }
    }
    out
}

/// 与当前姿态最接近的轴对齐朝向(|四元数点积| 最大;q 与 -q 同旋转)。
pub(crate) fn nearest_orientation(r: Quat, orients: &[Quat]) -> Quat {
    orients
        .iter()
        .copied()
        .max_by(|a, b| {
            a.dot(r)
                .abs()
                .partial_cmp(&b.dot(r).abs())
                .expect("朝向四元数为有限值")
        })
        .expect("朝向表非空")
}

/// 位置 snap:吸附到最近体素中心。返回 (格坐标, 吸附后中心, 位移距离)。
pub(crate) fn snap_position(pos: Vec3) -> (BlockPos, Vec3, f32) {
    let (fx, fy, fz) = (pos.x.floor(), pos.y.floor(), pos.z.floor());
    let cell = BlockPos::new(fx as i32, fy as i32, fz as i32);
    let center = Vec3::new(fx + 0.5, fy + 0.5, fz + 0.5);
    let dist = (center - pos).length();
    (cell, center, dist)
}

/// 旋转 snap 的角度误差(弧度)。|dot| = cos(θ/2),用半角公式
/// `θ = 4·asin(√((1-d)/2))`——acos 在 d→1(小角度)时误差放大,f32 下不可用。
pub(crate) fn snap_angle_error(r: Quat, snapped: Quat) -> f32 {
    let d = r.dot(snapped).abs().clamp(0.0, 1.0);
    4.0 * (0.5 * (1.0 - d)).max(0.0).sqrt().asin()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn twenty_four_unique_orientations() {
        let orients = axis_aligned_orientations();
        assert_eq!(orients.len(), 24);
        for (i, a) in orients.iter().enumerate() {
            for b in orients.iter().skip(i + 1) {
                assert!(a.dot(*b).abs() < 0.999_999, "朝向必须两两不同(旋转意义下)");
            }
        }
    }

    #[test]
    fn identity_snaps_to_identity() {
        let orients = axis_aligned_orientations();
        let snapped = nearest_orientation(Quat::IDENTITY, &orients);
        assert!(snapped.dot(Quat::IDENTITY).abs() > 0.999_999);
    }

    #[test]
    fn yaw_fifty_snaps_to_ninety() {
        let orients = axis_aligned_orientations();
        let yaw50 = Quat::from_axis_angle(Vec3::Y, 50.0f32.to_radians());
        let snapped = nearest_orientation(yaw50, &orients);
        let expect = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);
        assert!(snapped.dot(expect).abs() > 0.999_999, "50° → 90° 而非 0°");
    }

    #[test]
    fn snap_position_includes_negative_cells() {
        let (cell, center, dist) = snap_position(Vec3::new(-0.2, 1.7, 3.0));
        assert_eq!(cell, BlockPos::new(-1, 1, 3));
        assert_eq!(center, Vec3::new(-0.5, 1.5, 3.5));
        assert!((dist - (center - Vec3::new(-0.2, 1.7, 3.0)).length()).abs() < 1e-6);
        // 恰在中心 → 位移 0
        let (_, _, d0) = snap_position(Vec3::new(0.5, 0.5, 0.5));
        assert_eq!(d0, 0.0);
    }

    #[test]
    fn angle_error_zero_for_exact_orientation() {
        let q = Quat::from_axis_angle(Vec3::X, FRAC_PI_2);
        // f32 单位四元数自身点积只能达到 ~1-6e-8,误差下限 ~7e-4 rad,不可能为 0
        assert!(snap_angle_error(q, q) < 2e-3, "相同朝向误差接近 0");
        // 45° 误差
        let q45 = Quat::from_axis_angle(Vec3::Y, 0.25 * std::f32::consts::PI);
        let err = snap_angle_error(q45, Quat::IDENTITY);
        assert!((err - 0.25 * std::f32::consts::PI).abs() < 1e-5);
    }
}
