//! 跨片事件本地求值:爆炸冲量纯函数(demo D 移植)。
//!
//! 公式(REQ-001 §4.2 / demo D):对每个实体
//! `dist = |pos - center|;dist >= radius → 0;否则 impulse = dir * power * (1 - dist/radius)`
//! 线性衰减,不除以质量(重球速度变化小)。所需输入 = 共享 {center, power, radius} +
//! 本地 {实体 pos}——分片各自求值与单世界位级一致(demo D 实测误差 0.000e+00)。

use types::glam::Vec3;

/// 计算单个实体受到的爆炸冲量;`None` = 不受影响(超出半径或恰在中心)。
pub fn explosion_impulse(center: Vec3, power: f32, radius: f32, entity_pos: Vec3) -> Option<Vec3> {
    let offset = entity_pos - center;
    let dist = offset.length();
    if dist >= radius || dist == 0.0 {
        return None;
    }
    // dir * power * (1 - dist/radius) = offset/dist * ... 一步算完
    Some(offset * (power * (1.0 - dist / radius) / dist))
}

#[cfg(test)]
mod tests {
    use super::*;

    const CENTER: Vec3 = Vec3::new(0.0, 0.0, 0.0);

    #[test]
    fn outside_radius_is_none() {
        assert_eq!(
            explosion_impulse(CENTER, 6.0, 5.0, Vec3::new(5.1, 0.0, 0.0)),
            None
        );
        assert_eq!(
            explosion_impulse(CENTER, 6.0, 5.0, Vec3::new(3.0, 4.0, 0.0)),
            None
        ); // dist=5 恰在边界
    }

    #[test]
    fn direction_points_away_from_center() {
        let imp = explosion_impulse(CENTER, 6.0, 5.0, Vec3::new(0.0, 3.0, 0.0)).unwrap();
        assert!(
            imp.y > 0.0 && imp.x == 0.0 && imp.z == 0.0,
            "正上方 → 冲量朝上"
        );
    }

    #[test]
    fn linear_falloff() {
        // dist = radius/2 → 大小 = power * (1 - 0.5) = power/2
        let imp = explosion_impulse(CENTER, 6.0, 4.0, Vec3::new(2.0, 0.0, 0.0)).unwrap();
        assert!((imp.length() - 3.0).abs() < 1e-6);
        // 贴近中心 → 接近满功率
        let imp = explosion_impulse(CENTER, 6.0, 4.0, Vec3::new(0.001, 0.0, 0.0)).unwrap();
        assert!(imp.length() > 5.9);
    }

    #[test]
    fn exact_center_is_none() {
        // dist = 0 无方向定义;demo D 中不会出现(实体与爆心重合属罕见,显式 None)
        assert_eq!(explosion_impulse(CENTER, 6.0, 5.0, CENTER), None);
    }

    #[test]
    fn offset_center_diagonal_direction() {
        let c = Vec3::new(1.0, 0.0, 1.0);
        let imp = explosion_impulse(c, 2.0, 2.0, Vec3::new(2.0, 0.0, 2.0)).unwrap();
        // 方向 = (1,0,1)/√2,大小 = 2*(1 - √2/2)
        let expected_len = 2.0 * (1.0 - std::f32::consts::SQRT_2 / 2.0);
        assert!((imp.length() - expected_len).abs() < 1e-6);
        assert!((imp.x - imp.z).abs() < 1e-6, "对角对称");
    }
}
