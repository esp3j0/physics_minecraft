//! 实体描述与迁移载荷(demo A 协议)。

use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

use crate::coords::BlockPos;
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

/// 复合实体的一格(REQ-010 §5.4):相对原点格的整数偏移 + 方块。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ShapeCell {
    pub offset: [i32; 3],
    pub block: BlockId,
}

/// 复合实体形状构造失败的原因(REQ-010 §5.4)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeError {
    /// 少于 2 格(1 格的段是单格碎块,不是复合体)。
    TooFewCells,
    /// 两格偏移相同。
    DuplicateOffset([i32; 3]),
}

/// 复合实体形状(REQ-010 §5.1、§5.4):格按偏移升序(canonical)、偏移不重复、至少 2 格。
/// 只能经构造函数得到。
///
/// **总架构师接口桩(REQ-010 单元 03a)**:签名固定,函数体由实施方实现。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CompoundShape {
    cells: Vec<ShapeCell>,
}

impl CompoundShape {
    /// 校验并规范化:按偏移升序排列;偏移重复 → `DuplicateOffset`;少于 2 格 → `TooFewCells`。
    pub fn new(cells: Vec<ShapeCell>) -> Result<Self, ShapeError> {
        let _ = cells;
        Err(ShapeError::TooFewCells)
    }

    /// 由世界格与原点构造:偏移 = 格 − 原点;其余同 [`CompoundShape::new`]。
    pub fn from_world(origin: BlockPos, cells: &[(BlockPos, BlockId)]) -> Result<Self, ShapeError> {
        let _ = (origin, cells);
        Err(ShapeError::TooFewCells)
    }

    /// 各格(偏移升序)。
    pub fn cells(&self) -> &[ShapeCell] {
        &self.cells
    }

    /// 包络半径 ρ = 最远偏移的长度 + `half_size·√3`。
    pub fn envelope_radius(&self, half_size: f32) -> f32 {
        let _ = half_size;
        0.0
    }
}

impl EntityDesc {
    /// 复合体描述(REQ-010 §5.4):每格半边长 `half_size`;`block` 取第一格的方块(仅作兼容)。
    /// **接口桩**:实施方给 `EntityDesc` 追加 `shape: Option<CompoundShape>` 字段(随之不再 `Copy`)后实现。
    pub fn compound(half_size: f32, shape: CompoundShape) -> Self {
        Self {
            half_size,
            block: shape.cells().first().map_or(BlockId(1), |c| c.block),
        }
    }

    /// 复合体的形状;单格实体为 `None`。**接口桩**。
    pub fn shape(&self) -> Option<&CompoundShape> {
        None
    }

    /// 包络半径:单格 = `half_size·√3`(与现状相同),复合体 = 形状的包络半径。**接口桩**(桩阶段一律按单格)。
    pub fn envelope_radius(&self) -> f32 {
        self.half_size * 3f32.sqrt()
    }
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
