//! 实体描述与迁移载荷(demo A 协议)。

use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

use crate::ids::BlockId;

/// 迁移所需的完整刚体状态:四向量(REQ-001 §4.2 (b))。
/// rapier 类型不进本结构(防腐层,蓝图 §0);glam 经 types 统一转发。
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct EntityState {
    pub translation: Vec3,
    pub rotation: Quat,
    pub linvel: Vec3,
    pub angvel: Vec3,
}

/// 实体静态描述:形状与凝固桥所需的方块类型。迁移载荷必须携带它,
/// 目的分片才能重建 collider / 执行凝固(REQ-001 §4.2 (b))。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EntityDesc {
    /// 立方体半边长(当前阶段所有实体都是立方体)。
    pub half_size: f32,
    /// 凝固时写入体素数组的方块。
    pub block: BlockId,
}

impl Default for EntityDesc {
    /// 半格立方体 + 1 号方块(约定 stone)。
    fn default() -> Self {
        Self {
            half_size: 0.5,
            block: BlockId(1),
        }
    }
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

    #[test]
    fn default_desc_is_half_cube_stone() {
        let d = EntityDesc::default();
        assert_eq!(d.half_size, 0.5);
        assert_eq!(d.block, BlockId(1));
    }
}
