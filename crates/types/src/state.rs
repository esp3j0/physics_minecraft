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
/// REQ-010 §5.4:复合体追加 `shape`(可变长)后不再 `Copy`。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityDesc {
    /// 立方体半边长(当前阶段所有实体都是立方体;复合体 = 每格 cuboid 半边长)。
    pub half_size: f32,
    /// 凝固时写入体素数组的方块(复合体取第一格的方块,仅作兼容)。
    pub block: BlockId,
    /// 复合体形状;单格实体为 `None`(REQ-010 §5.4)。
    pub shape: Option<CompoundShape>,
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
    pub fn new(mut cells: Vec<ShapeCell>) -> Result<Self, ShapeError> {
        cells.sort_by_key(|c| c.offset);
        for pair in cells.windows(2) {
            if pair[0].offset == pair[1].offset {
                return Err(ShapeError::DuplicateOffset(pair[1].offset));
            }
        }
        if cells.len() < 2 {
            return Err(ShapeError::TooFewCells);
        }
        Ok(Self { cells })
    }

    /// 由世界格与原点构造:偏移 = 格 − 原点;其余同 [`CompoundShape::new`]。
    pub fn from_world(origin: BlockPos, cells: &[(BlockPos, BlockId)]) -> Result<Self, ShapeError> {
        Self::new(
            cells
                .iter()
                .map(|(p, b)| ShapeCell {
                    offset: [p.x - origin.x, p.y - origin.y, p.z - origin.z],
                    block: *b,
                })
                .collect(),
        )
    }

    /// 各格(偏移升序)。
    pub fn cells(&self) -> &[ShapeCell] {
        &self.cells
    }

    /// 包络半径 ρ = 最远偏移的长度 + `half_size·√3`。
    pub fn envelope_radius(&self, half_size: f32) -> f32 {
        let far = self
            .cells
            .iter()
            .map(|c| {
                let [x, y, z] = c.offset;
                let (x, y, z) = (x as f32, y as f32, z as f32);
                (x * x + y * y + z * z).sqrt()
            })
            .fold(0.0f32, f32::max);
        far + half_size * 3f32.sqrt()
    }
}

impl EntityDesc {
    /// 复合体描述(REQ-010 §5.4):每格半边长 `half_size`;`block` 取第一格的方块(仅作兼容)。
    pub fn compound(half_size: f32, shape: CompoundShape) -> Self {
        Self {
            half_size,
            block: shape.cells().first().map_or(BlockId(1), |c| c.block),
            shape: Some(shape),
        }
    }

    /// 复合体的形状;单格实体为 `None`。
    pub fn shape(&self) -> Option<&CompoundShape> {
        self.shape.as_ref()
    }

    /// 包络半径:单格 = `half_size·√3`(与现状相同),复合体 = 形状的包络半径。
    pub fn envelope_radius(&self) -> f32 {
        match &self.shape {
            Some(s) => s.envelope_radius(self.half_size),
            None => self.half_size * 3f32.sqrt(),
        }
    }
}

impl Default for EntityDesc {
    /// 半格立方体 + 1 号方块(约定 stone)。
    fn default() -> Self {
        Self {
            half_size: 0.5,
            block: BlockId(1),
            shape: None,
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
        assert!(d.shape().is_none());
    }

    fn cell(offset: [i32; 3], block: BlockId) -> ShapeCell {
        ShapeCell { offset, block }
    }

    /// REQ-010 §5.4:构造即规范化(偏移升序 canonical);重复偏移、少于 2 格报错。
    #[test]
    fn shape_new_sorts_and_rejects_duplicates_and_single_cell() {
        let shape = CompoundShape::new(vec![
            cell([2, 0, 0], BlockId(3)),
            cell([0, 0, 0], BlockId(1)),
            cell([1, 5, -7], BlockId(2)),
        ])
        .expect("3 格");
        let offsets: Vec<[i32; 3]> = shape.cells().iter().map(|c| c.offset).collect();
        assert_eq!(offsets, vec![[0, 0, 0], [1, 5, -7], [2, 0, 0]]);
        assert_eq!(
            CompoundShape::new(vec![
                cell([4, 4, 4], BlockId(1)),
                cell([4, 4, 4], BlockId(2))
            ]),
            Err(ShapeError::DuplicateOffset([4, 4, 4]))
        );
        assert_eq!(
            CompoundShape::new(vec![cell([0, 0, 0], BlockId(1))]),
            Err(ShapeError::TooFewCells)
        );
        assert_eq!(CompoundShape::new(Vec::new()), Err(ShapeError::TooFewCells));
    }

    /// REQ-010 §5.4:from_world 的偏移 = 格 − 原点,含负坐标;与 new 等价。
    #[test]
    fn shape_from_world_subtracts_origin() {
        let origin = BlockPos::new(-3, 2, 9);
        let cells = vec![
            (BlockPos::new(-3, 2, 9), BlockId(1)),
            (BlockPos::new(-2, 2, 9), BlockId(2)),
            (BlockPos::new(-1, 3, 8), BlockId(3)),
        ];
        let shape = CompoundShape::from_world(origin, &cells).expect("3 格");
        let by_offset: std::collections::BTreeMap<[i32; 3], BlockId> =
            shape.cells().iter().map(|c| (c.offset, c.block)).collect();
        assert_eq!(by_offset[&[0, 0, 0]], BlockId(1));
        assert_eq!(by_offset[&[1, 0, 0]], BlockId(2));
        assert_eq!(by_offset[&[2, 1, -1]], BlockId(3));
    }

    /// REQ-010 §5.1/§5.4:包络半径 = 最远偏移长度 + half·√3;最远按格中心
    /// 距离(非切比雪夫);空形不存在(构造挡住),单格偏移 [0,0,0] 不可能。
    #[test]
    fn shape_envelope_radius_is_farthest_offset_plus_half_sqrt3() {
        let shape = CompoundShape::new(vec![
            cell([2, 0, 0], BlockId(1)),
            cell([0, 1, 1], BlockId(1)),
            cell([1, 0, 0], BlockId(1)),
        ])
        .expect("3 格");
        // 最远 = [2,0,0] → 2;[0,1,1] → √2 ≈ 1.414。
        let want = 2.0f32 + 0.45 * 3f32.sqrt();
        assert!((shape.envelope_radius(0.45) - want).abs() < 1e-6);
        // 对角格最远:√3 > 2 → 最远 = √3。
        let diag = CompoundShape::new(vec![
            cell([1, 1, 1], BlockId(1)),
            cell([0, 0, 0], BlockId(1)),
        ])
        .expect("2 格");
        let want_diag = 3f32.sqrt() + 0.5 * 3f32.sqrt();
        assert!((diag.envelope_radius(0.5) - want_diag).abs() < 1e-6);
    }

    /// REQ-010 §5.4 第 2/7 条:单格描述的包络半径 = half·√3 逐位不变(与
    /// sim/wake 的 `SQRT3 = 1.732_050_8` 常量同源;REQ-007 轨迹基线盯着)。
    #[test]
    fn single_cell_envelope_radius_bitwise_stable() {
        let d = EntityDesc::default();
        assert_eq!(d.envelope_radius(), 0.5f32 * 1.732_050_8f32);
        let half = 0.45f32;
        let single = EntityDesc {
            half_size: half,
            block: BlockId(2),
            shape: None,
        };
        assert_eq!(single.envelope_radius(), half * 1.732_050_8f32);
    }

    /// REQ-010 §5.4:复合体描述带形状;block 取第一格(canonical 序);包络
    /// 半径取形状的;shape() 返回构造时传入的形状。
    #[test]
    fn compound_desc_carries_shape_and_first_cell_block() {
        let shape = CompoundShape::new(vec![
            cell([0, 0, 0], BlockId(4)),
            cell([1, 0, 0], BlockId(2)),
        ])
        .expect("2 格");
        let desc = EntityDesc::compound(0.45, shape.clone());
        assert_eq!(desc.block, BlockId(4), "block = 第一格(canonical 序)");
        assert_eq!(desc.half_size, 0.45);
        assert_eq!(desc.shape(), Some(&shape));
        assert_eq!(desc.envelope_radius(), shape.envelope_radius(0.45));
    }
}
