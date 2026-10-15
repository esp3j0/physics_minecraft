//! glam ↔ rapier 转换——防腐层的唯一出口(蓝图 §0:rapier 类型不得泄漏出 crate)。
//!
//! rapier 0.36 的数学层是 glamx(直接 re-export glam 0.33):`Vector` 即 glam `Vec3`,
//! `Rotation` 即 glam `Quat`。本层与 glam 0.33 对齐后全部为零成本恒等转换;
//! **保留本层的意义**是隔离上游:若未来 rapier 更换数学库,只改这里。

use rapier3d::math::{Pose, Vector};
use types::glam::{Quat, Vec3};

/// glam `Vec3` → rapier 向量(当前同类型,恒等)。
#[inline]
pub(crate) fn vec3_to_rapier(v: Vec3) -> Vector {
    v
}

/// rapier 向量 → glam `Vec3`(当前同类型,恒等)。
#[inline]
pub(crate) fn rapier_to_vec3(v: Vector) -> Vec3 {
    v
}

/// rapier 旋转类型别名(= glam `Quat`)。
pub(crate) type Rotation = Quat;

/// rapier 旋转 → glam `Quat`(当前同类型,恒等)。
#[inline]
pub(crate) fn rapier_to_quat(r: &Rotation) -> Quat {
    *r
}

/// 位姿 → rapier `Pose`。
#[inline]
pub(crate) fn pose_to_rapier(translation: Vec3, rotation: Quat) -> Pose {
    Pose {
        rotation,
        translation,
        ..Pose::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn vec_roundtrip_exact() {
        let v = Vec3::new(1.25, -3.5, 0.0625);
        assert_eq!(rapier_to_vec3(vec3_to_rapier(v)), v, "逐位一致");
    }

    #[test]
    fn quat_roundtrip_via_pose() {
        let q = Quat::from_axis_angle(Vec3::new(1.0, 2.0, -1.0).normalize(), 1.234);
        let pose = pose_to_rapier(Vec3::ZERO, q);
        assert_eq!(rapier_to_quat(&pose.rotation), q, "分量顺序不搞反");
    }

    #[test]
    fn pose_translation_and_rotation() {
        let iso = pose_to_rapier(
            Vec3::new(5.0, -6.0, 7.5),
            Quat::from_axis_angle(Vec3::Y, PI / 2.0),
        );
        assert_eq!(iso.translation, Vec3::new(5.0, -6.0, 7.5));
        assert!((iso.rotation.w - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6);
    }
}
