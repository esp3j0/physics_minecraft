//! 体素 → 碰撞几何的纯函数(demo B 移植)。
//!
//! **纯函数是硬要求**(demo A 结论):同一份体素数据在任何分片必须生成完全相同的
//! collider,否则分片接缝会产生确定性但可感知的轨迹分叉(0.26m 实测)。
//!
//! 体素约定(demo C/B):格 (x,y,z) 覆盖 [x,x+1)³,体素中心 (x+0.5, y+0.5, z+0.5)。
//! 暴露面 = 6 邻居判据;面 cuboid 为薄片(half 厚 `FACE_HALF_THICKNESS`),**内缩在实心格内,
//! 外表面落在格面上**(D15,REQ-010 §5.7.10)——碰撞几何与体素一致,1 格洞净宽恢复 1.0。

use types::glam::Vec3;
use types::{BlockId, ChunkPos, WORLD_Y_MIN};
use voxel::{CHUNK_X, CHUNK_Y, CHUNK_Z, Chunk};

/// 薄片半厚(demo B:0.075)。
pub const FACE_HALF_THICKNESS: f32 = 0.075;

/// 轴对齐盒 collider(世界坐标)。rapier 侧由 sim.rs 负责组装,本类型不依赖 rapier。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxCollider {
    /// 盒中心(世界坐标)。
    pub center: Vec3,
    /// 半边长。
    pub half: Vec3,
}

/// chunk 的四个水平邻居。**生产路径必须提供**(分片扩张区保证邻 chunk 已加载),
/// `None` 按"边界外是空气"处理(会生成保守的边界面)——仅测试/孤立 chunk 用。
#[derive(Debug, Clone, Copy, Default)]
pub struct NeighborChunks<'a> {
    pub px: Option<&'a Chunk>,
    pub nx: Option<&'a Chunk>,
    pub pz: Option<&'a Chunk>,
    pub nz: Option<&'a Chunk>,
}

impl NeighborChunks<'_> {
    /// 全空(边界外空气)。
    pub fn none() -> Self {
        Self::default()
    }
}

/// 单个暴露面:cell 局部坐标 + 法向(axis: 0=x,1=y,2=z)。
#[derive(Debug, Clone, Copy)]
struct Face {
    axis: usize,
    positive: bool,
    lx: usize,
    ly: usize,
    lz: usize,
}

impl Face {
    /// 面平面在 chunk 局部方块坐标上的位置(格 (l,·) 的 +x 面在 l+1,-x 面在 l)。
    fn plane_local(self) -> i32 {
        let l = [self.lx, self.ly, self.lz][self.axis];
        if self.positive {
            l as i32 + 1
        } else {
            l as i32
        }
    }
}

/// 提取全部暴露面(6 邻居判据;边界外 None 邻居 = 空气,y 出世界 = 空气)。
fn extract_faces(
    chunk: &Chunk,
    neighbors: &NeighborChunks<'_>,
    solid: &impl Fn(BlockId) -> bool,
) -> Vec<Face> {
    let solid_at = |x: i32, y: i32, z: i32| -> bool {
        if y < 0 || y >= CHUNK_Y as i32 {
            return false; // 世界上下界外 = 空气
        }
        let (uy, uz) = (y as usize, z as usize);
        if x < 0 {
            return match neighbors.nx {
                Some(n) => solid(n.get(CHUNK_X - 1, uy, uz.clamp(0, CHUNK_Z - 1))),
                None => false,
            };
        }
        if x >= CHUNK_X as i32 {
            return match neighbors.px {
                Some(n) => solid(n.get(0, uy, uz.clamp(0, CHUNK_Z - 1))),
                None => false,
            };
        }
        let ux = x as usize;
        if z < 0 {
            return match neighbors.nz {
                Some(n) => solid(n.get(ux, uy, CHUNK_Z - 1)),
                None => false,
            };
        }
        if z >= CHUNK_Z as i32 {
            return match neighbors.pz {
                Some(n) => solid(n.get(ux, uy, 0)),
                None => false,
            };
        }
        solid(chunk.get(ux, uy, uz))
    };

    let mut faces = Vec::new();
    for lx in 0..CHUNK_X {
        for ly in 0..CHUNK_Y {
            for lz in 0..CHUNK_Z {
                if !solid(chunk.get(lx, ly, lz)) {
                    continue;
                }
                let (ix, iy, iz) = (lx as i32, ly as i32, lz as i32);
                let mut push = |axis: usize, positive: bool| {
                    faces.push(Face {
                        axis,
                        positive,
                        lx,
                        ly,
                        lz,
                    });
                };
                if !solid_at(ix + 1, iy, iz) {
                    push(0, true);
                }
                if !solid_at(ix - 1, iy, iz) {
                    push(0, false);
                }
                if !solid_at(ix, iy + 1, iz) {
                    push(1, true);
                }
                if !solid_at(ix, iy - 1, iz) {
                    push(1, false);
                }
                if !solid_at(ix, iy, iz + 1) {
                    push(2, true);
                }
                if !solid_at(ix, iy, iz - 1) {
                    push(2, false);
                }
            }
        }
    }
    faces
}

/// 面(或矩形合并后的面)→ 世界坐标 cuboid。
///
/// `positive`:法向朝轴正方向。薄片中心从面平面 `plane_world` 向实心格一侧移
/// `FACE_HALF_THICKNESS`(正法向面减、负法向面加,REQ-010 §5.7.10),
/// 外表面恰好落在格面上。
#[allow(clippy::too_many_arguments)]
fn face_box(
    axis: usize,
    positive: bool,
    plane_world: f32,
    r1: usize,
    r2: usize,
    c1: usize,
    c2: usize,
    cp: ChunkPos,
) -> BoxCollider {
    let min = cp.block_min();
    let (a1, a2) = match axis {
        0 => (1, 2), // y, z
        1 => (0, 2), // x, z
        _ => (0, 1), // x, y
    };
    let (t1_min, t2_min) = match axis {
        0 => (WORLD_Y_MIN as f32, min.z as f32),
        1 => (min.x as f32, min.z as f32),
        _ => (min.x as f32, WORLD_Y_MIN as f32),
    };
    let e1 = (r2 - r1 + 1) as f32;
    let e2 = (c2 - c1 + 1) as f32;
    let mut center = [0.0f32; 3];
    let mut half = [0.0f32; 3];
    center[axis] = if positive {
        plane_world - FACE_HALF_THICKNESS
    } else {
        plane_world + FACE_HALF_THICKNESS
    };
    half[axis] = FACE_HALF_THICKNESS;
    center[a1] = t1_min + r1 as f32 + e1 / 2.0;
    half[a1] = e1 / 2.0;
    center[a2] = t2_min + c1 as f32 + e2 / 2.0;
    half[a2] = e2 / 2.0;
    BoxCollider {
        center: Vec3::new(center[0], center[1], center[2]),
        half: Vec3::new(half[0], half[1], half[2]),
    }
}

/// 逐面策略:每个暴露面一个薄片 cuboid(demo B 策略 1)。
pub fn per_face_colliders(
    chunk: &Chunk,
    cp: ChunkPos,
    neighbors: &NeighborChunks<'_>,
    solid: impl Fn(BlockId) -> bool,
) -> Vec<BoxCollider> {
    let min = cp.block_min();
    extract_faces(chunk, neighbors, &solid)
        .into_iter()
        .map(|f| {
            let plane_world = match f.axis {
                0 => min.x as f32 + f.plane_local() as f32,
                1 => WORLD_Y_MIN as f32 + f.plane_local() as f32,
                _ => min.z as f32 + f.plane_local() as f32,
            };
            let (r, c) = match f.axis {
                0 => (f.ly, f.lz),
                1 => (f.lx, f.lz),
                _ => (f.lx, f.ly),
            };
            face_box(f.axis, f.positive, plane_world, r, r, c, c, cp)
        })
        .collect()
}

/// 贪心合并策略:同一(法向, 平面)上的面按行/列合并成大矩形(demo B 策略 2,
/// 实测压缩 ~4.8×,收益 ~7%,可选优化)。合并严格限制在 chunk 界内。
pub fn greedy_colliders(
    chunk: &Chunk,
    cp: ChunkPos,
    neighbors: &NeighborChunks<'_>,
    solid: impl Fn(BlockId) -> bool,
) -> Vec<BoxCollider> {
    let min = cp.block_min();
    let faces = extract_faces(chunk, neighbors, &solid);

    // 分组:(axis, positive, plane_local) → 切向 mask
    use std::collections::BTreeMap;
    let mut groups: BTreeMap<(usize, bool, i32), FaceGroup> = BTreeMap::new();
    for f in faces {
        let (d1, d2) = match f.axis {
            0 => (CHUNK_Y, CHUNK_Z),
            1 => (CHUNK_X, CHUNK_Z),
            _ => (CHUNK_X, CHUNK_Y),
        };
        let (r, c) = match f.axis {
            0 => (f.ly, f.lz),
            1 => (f.lx, f.lz),
            _ => (f.lx, f.ly),
        };
        let entry = groups
            .entry((f.axis, f.positive, f.plane_local()))
            .or_insert_with(|| FaceGroup {
                mask: vec![vec![false; d2]; d1],
                h: d1,
                w: d2,
            });
        entry.mask[r][c] = true;
    }

    let mut out = Vec::new();
    for ((axis, positive, plane), group) in groups {
        for (r1, c1, r2, c2) in merge_rects(&group.mask, group.h, group.w) {
            let plane_world = match axis {
                0 => min.x as f32 + plane as f32,
                1 => WORLD_Y_MIN as f32 + plane as f32,
                _ => min.z as f32 + plane as f32,
            };
            out.push(face_box(axis, positive, plane_world, r1, r2, c1, c2, cp));
        }
    }
    out
}

/// 贪心分组:同一(法向, 平面)上暴露面的切向布尔 mask 及其尺寸。
struct FaceGroup {
    mask: Vec<Vec<bool>>,
    h: usize,
    w: usize,
}

/// 行内扩 run、再向下扩等宽 run 的贪心矩形合并(demo B 移植,返回含端点矩形)。
fn merge_rects(mask: &[Vec<bool>], h: usize, w: usize) -> Vec<(usize, usize, usize, usize)> {
    let mut used = vec![vec![false; w]; h];
    let mut rects = Vec::new();
    for i in 0..h {
        let mut j = 0;
        while j < w {
            if mask[i][j] && !used[i][j] {
                let mut j2 = j;
                while j2 + 1 < w && mask[i][j2 + 1] && !used[i][j2 + 1] {
                    j2 += 1;
                }
                let mut i2 = i;
                'grow: while i2 + 1 < h {
                    for c in j..=j2 {
                        if !mask[i2 + 1][c] || used[i2 + 1][c] {
                            break 'grow;
                        }
                    }
                    i2 += 1;
                }
                for row in used.iter_mut().take(i2 + 1).skip(i) {
                    for cell in row.iter_mut().take(j2 + 1).skip(j) {
                        *cell = true;
                    }
                }
                rects.push((i, j, i2, j2));
                j = j2 + 1;
            } else {
                j += 1;
            }
        }
    }
    rects
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::ShardPos;

    fn solid_of(b: BlockId) -> bool {
        !b.is_air() // 测试 chunk 用 BlockId(1) 表示实心方块
    }

    /// 只有一个实心方块的 chunk。
    fn single_block_chunk(lx: usize, ly: usize, lz: usize) -> Chunk {
        let mut c = Chunk::air();
        c.set(lx, ly, lz, BlockId(1));
        c
    }

    #[test]
    fn isolated_block_has_six_faces() {
        let chunk = single_block_chunk(8, 100, 8);
        let boxes = per_face_colliders(
            &chunk,
            ChunkPos::new(0, 0),
            &NeighborChunks::none(),
            solid_of,
        );
        assert_eq!(boxes.len(), 6);
        // 朝上的面:+y 面薄片内缩在格内(中心 = 格顶 − 0.075),薄片沿 y
        let top = boxes
            .iter()
            .find(|b| b.half.y == FACE_HALF_THICKNESS)
            .expect("存在 +y/-y 面之一");
        assert_eq!(top.half, Vec3::new(0.5, FACE_HALF_THICKNESS, 0.5));
    }

    #[test]
    fn face_center_world_coordinates() {
        // 格 (0,0,0) 属于 chunk(0,0):世界格 (0, WORLD_Y_MIN, 0),中心 (0.5, -63.5, 0.5)
        let chunk = single_block_chunk(0, 0, 0);
        let boxes = per_face_colliders(
            &chunk,
            ChunkPos::new(0, 0),
            &NeighborChunks::none(),
            solid_of,
        );
        // 六个面的薄片都内缩在格内:外表面在格面上,中心 = 格面 ∓ 0.075
        // 格 (0,0,0) 世界覆盖 x∈[0,1], y∈[WORLD_Y_MIN, WORLD_Y_MIN+1], z∈[0,1]
        let t = FACE_HALF_THICKNESS;
        let by_half = |axis: usize| -> Vec<Vec3> {
            let mut v: Vec<Vec3> = boxes
                .iter()
                .filter(|b| {
                    let h = [b.half.x, b.half.y, b.half.z][axis];
                    h == FACE_HALF_THICKNESS
                })
                .map(|b| b.center)
                .collect();
            v.sort_by(|a, b| {
                (a.x, a.y, a.z)
                    .partial_cmp(&(b.x, b.y, b.z))
                    .expect("有限坐标")
            });
            v
        };
        // x 面:平面 x=0 与 x=1 → 中心 x = 0 + t 与 1 − t
        assert_eq!(
            by_half(0),
            vec![
                Vec3::new(t, (WORLD_Y_MIN + 1) as f32 - 0.5, 0.5),
                Vec3::new(1.0 - t, (WORLD_Y_MIN + 1) as f32 - 0.5, 0.5),
            ]
        );
        // y 面:平面 WORLD_Y_MIN 与 WORLD_Y_MIN+1 → 中心 y = WORLD_Y_MIN + t 与 WORLD_Y_MIN+1 − t
        assert_eq!(
            by_half(1),
            vec![
                Vec3::new(0.5, WORLD_Y_MIN as f32 + t, 0.5),
                Vec3::new(0.5, (WORLD_Y_MIN + 1) as f32 - t, 0.5),
            ]
        );
        // z 面:平面 z=0 与 z=1 → 中心 z = t 与 1 − t
        assert_eq!(
            by_half(2),
            vec![
                Vec3::new(0.5, (WORLD_Y_MIN + 1) as f32 - 0.5, t),
                Vec3::new(0.5, (WORLD_Y_MIN + 1) as f32 - 0.5, 1.0 - t),
            ]
        );
    }

    #[test]
    fn buried_block_has_no_faces() {
        // 3×3×3 实心块的中心格:六个邻居全部实心 → 无暴露面
        let mut chunk = Chunk::air();
        for x in 7..=9 {
            for y in 99..=101 {
                for z in 7..=9 {
                    chunk.set(x, y, z, BlockId(1));
                }
            }
        }
        chunk.set(8, 100, 8, BlockId(1)); // 中心格再次确保
        let boxes = per_face_colliders(
            &chunk,
            ChunkPos::new(0, 0),
            &NeighborChunks::none(),
            solid_of,
        );
        // 只有 3×3×3 的外壳暴露面:6 面 × 9 = 54
        assert_eq!(boxes.len(), 54);
    }

    #[test]
    fn solid_neighbor_suppresses_boundary_face() {
        // chunk 全实心;+x 侧放一个全实心邻居 → +x 边界面消失
        let chunk = Chunk::filled(BlockId(1));
        let boxes_alone = per_face_colliders(
            &chunk,
            ChunkPos::new(0, 0),
            &NeighborChunks::none(),
            solid_of,
        );
        let boxes_with_px = per_face_colliders(
            &chunk,
            ChunkPos::new(0, 0),
            &NeighborChunks {
                px: Some(&chunk),
                ..NeighborChunks::none()
            },
            solid_of,
        );
        let alone_px = boxes_alone
            .iter()
            .filter(|b| {
                b.half.x == FACE_HALF_THICKNESS
                    && b.center.x == CHUNK_X as f32 - FACE_HALF_THICKNESS
            })
            .count();
        let with_px = boxes_with_px
            .iter()
            .filter(|b| {
                b.half.x == FACE_HALF_THICKNESS
                    && b.center.x == CHUNK_X as f32 - FACE_HALF_THICKNESS
            })
            .count();
        assert!(alone_px > 0, "无邻居时边界生成保守面");
        assert_eq!(with_px, 0, "实心邻居抑制边界面(接缝两侧一致的关键)");
    }

    #[test]
    fn greedy_merges_flat_slab() {
        // 顶层 2×2 实心平台:顶面 4 面合并为 1;逐面为 4
        let mut chunk = Chunk::air();
        for x in 0..2 {
            for z in 0..2 {
                chunk.set(x, CHUNK_Y - 1, z, BlockId(1));
            }
        }
        let per = per_face_colliders(
            &chunk,
            ChunkPos::new(0, 0),
            &NeighborChunks::none(),
            solid_of,
        );
        let greedy = greedy_colliders(
            &chunk,
            ChunkPos::new(0, 0),
            &NeighborChunks::none(),
            solid_of,
        );
        // 2×2 柱体:顶面 4 + 底面 4 + 侧面 (4 面各 2) = 16 逐面
        assert_eq!(per.len(), 16);
        // 顶/底各合并为 1;+x/-x/+z/-z 各 1(高 1 宽 2)→ 共 6
        assert_eq!(greedy.len(), 6);
        // 合并后的顶面盖住整个 2×2(+y 面:取 center.y 最大的 y 向薄片)
        let top = greedy
            .iter()
            .filter(|b| b.half.y == FACE_HALF_THICKNESS)
            .max_by(|a, b| a.center.y.partial_cmp(&b.center.y).expect("有限坐标"))
            .expect("顶面");
        assert_eq!(top.half, Vec3::new(1.0, FACE_HALF_THICKNESS, 1.0));
        assert_eq!(top.center.x, 1.0);
        assert_eq!(top.center.z, 1.0);
        // 内缩:顶面薄片外表面在 chunk 顶格面(WORLD_Y_MIN + CHUNK_Y)上
        assert_eq!(
            top.center.y,
            (WORLD_Y_MIN + CHUNK_Y as i32) as f32 - FACE_HALF_THICKNESS
        );
        // 底面薄片同规则:外表面在 chunk 底格所在平面(WORLD_Y_MIN + 383)上,中心 + t
        let bottom = greedy
            .iter()
            .filter(|b| b.half.y == FACE_HALF_THICKNESS)
            .min_by(|a, b| a.center.y.partial_cmp(&b.center.y).expect("有限坐标"))
            .expect("底面");
        assert_eq!(
            bottom.center.y,
            (WORLD_Y_MIN + CHUNK_Y as i32) as f32 - 1.0 + FACE_HALF_THICKNESS
        );
    }

    #[test]
    fn pure_function_is_deterministic() {
        let mut chunk = Chunk::air();
        for i in 0..10 {
            chunk.set(i, 2 * i, i, BlockId(1));
        }
        let a = greedy_colliders(
            &chunk,
            ChunkPos::new(3, -2),
            &NeighborChunks::none(),
            solid_of,
        );
        let b = greedy_colliders(
            &chunk,
            ChunkPos::new(3, -2),
            &NeighborChunks::none(),
            solid_of,
        );
        assert_eq!(a, b, "同输入必同输出(demo A:纯函数是接缝一致性的根基)");
        let _ = ShardPos::new(0, 0); // 确认 types 依赖可用
    }
}
