//! 实体迁移载荷(demo A 协议)。

use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

/// 迁移所需的完整刚体状态:四向量(REQ-001 §4.2 (b))。
/// rapier 类型不进本结构(防腐层,蓝图 §0);glam 经 types 统一转发。
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct EntityState {
    pub translation: Vec3,
    pub rotation: Quat,
    pub linvel: Vec3,
    pub angvel: Vec3,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_rest_pose() {
        let s = EntityState::default();
        assert_eq!(s.translation, Vec3::ZERO);
        assert_eq!(s.rotation, Quat::IDENTITY);
        assert_eq!(s.linvel, Vec3::ZERO);
        assert_eq!(s.angvel, Vec3::ZERO);
    }
}
