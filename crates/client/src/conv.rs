//! 世界坐标(`types::glam`)与渲染坐标(`bevy::math`)之间的薄转换层(REQ-013 §5)。
//!
//! 统一收口两套 `glam` 版本之间的字段转换,禁止在渲染系统里散落手工拆字段。

// C01b-1 仅使用 `from_bevy_vec3` 上报观察者坐标;其余方向与四元数转换供 C02/C03 实体与主体姿态复用。
#![allow(dead_code)]

use bevy::math::{Quat as BQuat, Vec3 as BVec3};
use types::glam::{Quat as TQuat, Vec3 as TVec3};

/// `types::glam::Vec3` → `bevy::math::Vec3`。
#[inline]
pub fn to_bevy_vec3(v: TVec3) -> BVec3 {
    BVec3::new(v.x, v.y, v.z)
}

/// `bevy::math::Vec3` → `types::glam::Vec3`。
#[inline]
pub fn from_bevy_vec3(v: BVec3) -> TVec3 {
    TVec3::new(v.x, v.y, v.z)
}

/// `types::glam::Quat` → `bevy::math::Quat`。
#[inline]
pub fn to_bevy_quat(q: TQuat) -> BQuat {
    BQuat::from_xyzw(q.x, q.y, q.z, q.w)
}

/// `bevy::math::Quat` → `types::glam::Quat`。
#[inline]
pub fn from_bevy_quat(q: BQuat) -> TQuat {
    TQuat::from_xyzw(q.x, q.y, q.z, q.w)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec3_and_quat_roundtrip() {
        let v = TVec3::new(1.5, -60.25, 24.0);
        assert_eq!(from_bevy_vec3(to_bevy_vec3(v)), v);

        let q = TQuat::from_xyzw(0.1, -0.2, 0.3, 0.9).normalize();
        assert_eq!(from_bevy_quat(to_bevy_quat(q)), q);
    }
}
