//! subchunk 网格生成 + 顶点 AO(REQ-013 C01b-1 §2.2)。
//!
//! 纯函数、零渲染依赖、零墙钟。输出世界坐标顶点/法向/烘焙顶点色与 `u32` 三角索引。

use types::{BlockId, BlockPos, CHUNK_SIZE, WORLD_Y_MIN};
use voxel::{CHUNK_X, CHUNK_Z};

use crate::mirror::{ChunkMirror, SECTION_HEIGHT, SECTIONS_PER_CHUNK, SubchunkPos};

/// 单个 subchunk 的网格几何数据(世界坐标)。
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SubchunkMesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub colors: Vec<[f32; 4]>,
    pub indices: Vec<u32>,
}

/// 单个暴露面的固定规格:法向轴/符号、法向向量、4 角点偏移、方向亮度。
struct FaceSpec {
    /// 法向轴索引:0 = X, 1 = Y, 2 = Z。
    axis: usize,
    /// 法向符号:+1 或 -1。
    sign: i32,
    normal: [f32; 3],
    corners: [[i32; 3]; 4],
    brightness: f32,
}

/// 固定方向序:+X, -X, +Y, -Y, +Z, -Z(§2.2 第 2–3 步)。
const FACES: [FaceSpec; 6] = [
    // +X
    FaceSpec {
        axis: 0,
        sign: 1,
        normal: [1.0, 0.0, 0.0],
        corners: [[1, 0, 0], [1, 1, 0], [1, 1, 1], [1, 0, 1]],
        brightness: 0.6,
    },
    // -X
    FaceSpec {
        axis: 0,
        sign: -1,
        normal: [-1.0, 0.0, 0.0],
        corners: [[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 1, 0]],
        brightness: 0.6,
    },
    // +Y
    FaceSpec {
        axis: 1,
        sign: 1,
        normal: [0.0, 1.0, 0.0],
        corners: [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
        brightness: 1.0,
    },
    // -Y
    FaceSpec {
        axis: 1,
        sign: -1,
        normal: [0.0, -1.0, 0.0],
        corners: [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]],
        brightness: 0.4,
    },
    // +Z
    FaceSpec {
        axis: 2,
        sign: 1,
        normal: [0.0, 0.0, 1.0],
        corners: [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
        brightness: 0.8,
    },
    // -Z
    FaceSpec {
        axis: 2,
        sign: -1,
        normal: [0.0, 0.0, -1.0],
        corners: [[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]],
        brightness: 0.8,
    },
];

/// 标准体素顶点 AO 等级(0..=3):两侧同时遮挡强制 0,否则 `3 - (side1 + side2 + corner)`。
pub fn vertex_ao(side1: bool, side2: bool, corner: bool) -> u8 {
    if side1 && side2 {
        0
    } else {
        3 - (u8::from(side1) + u8::from(side2) + u8::from(corner))
    }
}

/// 方块 RGBA 颜色表(沿用 `viewer::mesh::block_color` 口径)。
pub fn block_color(id: BlockId) -> [f32; 4] {
    match id {
        BlockId::AIR => [0.0, 0.0, 0.0, 0.0],
        BlockId(1) => [0.55, 0.55, 0.58, 1.0],
        BlockId(2) => [0.45, 0.33, 0.22, 1.0],
        BlockId(3) => [0.30, 0.30, 0.32, 1.0],
        BlockId(4) => [0.87, 0.80, 0.55, 1.0],
        _ => [1.0, 0.0, 1.0, 1.0],
    }
}

/// 生成单个 subchunk 的网格(纯函数):跨 chunk 查询邻格与顶点 AO 采样,烘焙方向亮度与 AO。
/// 未装载 chunk、非法段号或全空气段返回四个全空 `Vec`。
pub fn mesh_subchunk(mirror: &ChunkMirror, sc: SubchunkPos) -> SubchunkMesh {
    let mut mesh = SubchunkMesh::default();
    if sc.section >= SECTIONS_PER_CHUNK {
        return mesh;
    }
    let Some(chunk) = mirror.chunk(sc.chunk) else {
        return mesh;
    };

    let origin_x = sc.chunk.x * CHUNK_SIZE;
    let origin_z = sc.chunk.z * CHUNK_SIZE;
    let y_start = usize::from(sc.section) * SECTION_HEIGHT;
    let y_end = y_start + SECTION_HEIGHT;

    for ly in y_start..y_end {
        let wy = WORLD_Y_MIN + ly as i32;
        for lz in 0..CHUNK_Z {
            let wz = origin_z + lz as i32;
            for lx in 0..CHUNK_X {
                let block = chunk.get(lx, ly, lz);
                if block.is_air() {
                    continue;
                }
                let wx = origin_x + lx as i32;
                let base_color = block_color(block);
                let cell = [wx, wy, wz];

                for face in &FACES {
                    let mut outer = cell;
                    outer[face.axis] += face.sign;
                    let neighbor_pos = BlockPos::new(outer[0], outer[1], outer[2]);
                    if is_solid(mirror, neighbor_pos) {
                        continue;
                    }

                    // 其余两轴按 (x, y, z) 顺序记为 (u, v)。
                    let (u_axis, v_axis) = match face.axis {
                        0 => (1, 2),
                        1 => (0, 2),
                        _ => (0, 1),
                    };

                    let base_vertex = mesh.positions.len() as u32;
                    for corner in &face.corners {
                        let du = if corner[u_axis] == 1 { 1 } else { -1 };
                        let dv = if corner[v_axis] == 1 { 1 } else { -1 };

                        let mut s1 = outer;
                        s1[u_axis] += du;
                        let mut s2 = outer;
                        s2[v_axis] += dv;
                        let mut c = outer;
                        c[u_axis] += du;
                        c[v_axis] += dv;

                        let ao_level = vertex_ao(
                            is_solid(mirror, BlockPos::new(s1[0], s1[1], s1[2])),
                            is_solid(mirror, BlockPos::new(s2[0], s2[1], s2[2])),
                            is_solid(mirror, BlockPos::new(c[0], c[1], c[2])),
                        );
                        let ao_factor = 0.4 + 0.2 * f32::from(ao_level);
                        let shade = face.brightness * ao_factor;

                        mesh.positions.push([
                            (wx + corner[0]) as f32,
                            (wy + corner[1]) as f32,
                            (wz + corner[2]) as f32,
                        ]);
                        mesh.normals.push(face.normal);
                        mesh.colors.push([
                            base_color[0] * shade,
                            base_color[1] * shade,
                            base_color[2] * shade,
                            1.0,
                        ]);
                    }

                    mesh.indices.extend_from_slice(&[
                        base_vertex,
                        base_vertex + 1,
                        base_vertex + 2,
                        base_vertex,
                        base_vertex + 2,
                        base_vertex + 3,
                    ]);
                }
            }
        }
    }

    mesh
}

#[inline]
fn is_solid(mirror: &ChunkMirror, pos: BlockPos) -> bool {
    mirror.get_block(pos).is_some_and(|id| !id.is_air())
}

#[cfg(test)]
mod tests {
    use types::ChunkPos;
    use voxel::Chunk;

    use super::*;
    use crate::mirror::DirtyKind;

    const STONE: BlockId = BlockId(1);
    const EPS: f32 = 1e-6;

    fn assert_color_eq(actual: [f32; 4], expected: [f32; 4], label: &str) {
        for (c, (a, e)) in actual.iter().zip(expected.iter()).enumerate() {
            assert!(
                (a - e).abs() <= EPS,
                "{label} 通道 {c} 不匹配: 实测 {actual:?}, 期望 {expected:?}"
            );
        }
    }

    /// MA1:单块 STONE@(0,-60,0) → 段 0 共 6 面/24 顶点/36 索引,全 AO=3,色按面准确;段 1 空。
    #[test]
    fn ma1_single_stone_faces_and_colors() {
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        chunk.set(0, 4, 0, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        let m0 = mesh_subchunk(
            &mirror,
            SubchunkPos {
                chunk: ChunkPos::new(0, 0),
                section: 0,
            },
        );
        assert_eq!(m0.positions.len(), 24);
        assert_eq!(m0.normals.len(), 24);
        assert_eq!(m0.colors.len(), 24);
        assert_eq!(m0.indices.len(), 36);

        // 方向序 +X, -X, +Y, -Y, +Z, -Z,每面 4 顶点。
        let expected_per_face: [([f32; 3], [f32; 4]); 6] = [
            ([1.0, 0.0, 0.0], [0.33, 0.33, 0.348, 1.0]),
            ([-1.0, 0.0, 0.0], [0.33, 0.33, 0.348, 1.0]),
            ([0.0, 1.0, 0.0], [0.55, 0.55, 0.58, 1.0]),
            ([0.0, -1.0, 0.0], [0.22, 0.22, 0.232, 1.0]),
            ([0.0, 0.0, 1.0], [0.44, 0.44, 0.464, 1.0]),
            ([0.0, 0.0, -1.0], [0.44, 0.44, 0.464, 1.0]),
        ];
        for (face_idx, (exp_n, exp_c)) in expected_per_face.iter().enumerate() {
            for v in 0..4 {
                let i = face_idx * 4 + v;
                assert_eq!(m0.normals[i], *exp_n, "面 {face_idx} 顶点 {v} 法向");
                assert_color_eq(m0.colors[i], *exp_c, &format!("面 {face_idx} 顶点 {v}"));
            }
        }

        let m1 = mesh_subchunk(
            &mirror,
            SubchunkPos {
                chunk: ChunkPos::new(0, 0),
                section: 1,
            },
        );
        assert!(m1.positions.is_empty());
        assert!(m1.normals.is_empty());
        assert!(m1.colors.is_empty());
        assert!(m1.indices.is_empty());
    }

    /// MA2:vertex_ao 完整 8 组真值表。
    #[test]
    fn ma2_vertex_ao_truth_table() {
        assert_eq!(vertex_ao(false, false, false), 3);
        assert_eq!(vertex_ao(true, false, false), 2);
        assert_eq!(vertex_ao(false, true, false), 2);
        assert_eq!(vertex_ao(false, false, true), 2);
        assert_eq!(vertex_ao(true, true, false), 0);
        assert_eq!(vertex_ao(true, false, true), 1);
        assert_eq!(vertex_ao(false, true, true), 1);
        assert_eq!(vertex_ao(true, true, true), 0);
    }

    /// MA3(裁定 §1 L 形):STONE@(0,-60,0) B1、(1,-60,0) B2、(1,-59,0) B3 →
    /// 14 面/56 顶点/84 索引,按 (位置, 法向) 精确检验凹角 AO=2 与顶角 AO=3。
    #[test]
    fn ma3_l_shape_concave_ao() {
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        chunk.set(0, 4, 0, STONE); // B1: (0, -60, 0)
        chunk.set(1, 4, 0, STONE); // B2: (1, -60, 0)
        chunk.set(1, 5, 0, STONE); // B3: (1, -59, 0)
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        let m0 = mesh_subchunk(
            &mirror,
            SubchunkPos {
                chunk: ChunkPos::new(0, 0),
                section: 0,
            },
        );
        assert_eq!(m0.positions.len(), 56, "14 面 × 4 顶点 = 56");
        assert_eq!(m0.normals.len(), 56);
        assert_eq!(m0.colors.len(), 56);
        assert_eq!(m0.indices.len(), 84, "14 面 × 6 索引 = 84");

        let find_colors = |pos: [f32; 3], normal: [f32; 3]| -> Vec<[f32; 4]> {
            m0.positions
                .iter()
                .zip(m0.normals.iter())
                .zip(m0.colors.iter())
                .filter(|((p, n), _)| {
                    (p[0] - pos[0]).abs() <= EPS
                        && (p[1] - pos[1]).abs() <= EPS
                        && (p[2] - pos[2]).abs() <= EPS
                        && (n[0] - normal[0]).abs() <= EPS
                        && (n[1] - normal[1]).abs() <= EPS
                        && (n[2] - normal[2]).abs() <= EPS
                })
                .map(|((_, _), c)| *c)
                .collect()
        };

        // 2. 法向 (0,1,0) 且位于 (1,-59,0) 与 (1,-59,1) 的顶点(恰各 1 个,合计 2 个):AO=2 → [0.44, 0.44, 0.464, 1]
        let mut top_concave = Vec::new();
        for pos in [[1.0, -59.0, 0.0], [1.0, -59.0, 1.0]] {
            let cs = find_colors(pos, [0.0, 1.0, 0.0]);
            assert_eq!(cs.len(), 1, "法向 +Y 位于 {pos:?} 应恰 1 顶点");
            assert_color_eq(cs[0], [0.44, 0.44, 0.464, 1.0], &format!("+Y@{pos:?}"));
            top_concave.extend(cs);
        }
        assert_eq!(top_concave.len(), 2);

        // 3. 法向 (-1,0,0) 且位于 (1,-59,0) 与 (1,-59,1) 的顶点(恰各 1 个,合计 2 个):AO=2 → [0.264, 0.264, 0.2784, 1]
        let mut west_bottom = Vec::new();
        for pos in [[1.0, -59.0, 0.0], [1.0, -59.0, 1.0]] {
            let cs = find_colors(pos, [-1.0, 0.0, 0.0]);
            assert_eq!(cs.len(), 1, "法向 -X 位于 {pos:?} 应恰 1 顶点");
            assert_color_eq(cs[0], [0.264, 0.264, 0.2784, 1.0], &format!("-X@{pos:?}"));
            west_bottom.extend(cs);
        }
        assert_eq!(west_bottom.len(), 2);

        // 4. 法向 (-1,0,0) 且位于 (1,-58,0) 与 (1,-58,1) 的顶点(B3 -X 顶角):AO=3 → [0.33, 0.33, 0.348, 1]
        for pos in [[1.0, -58.0, 0.0], [1.0, -58.0, 1.0]] {
            let cs = find_colors(pos, [-1.0, 0.0, 0.0]);
            assert_eq!(cs.len(), 1, "法向 -X 位于 {pos:?} 应恰 1 顶点");
            assert_color_eq(cs[0], [0.33, 0.33, 0.348, 1.0], &format!("-X@{pos:?}"));
        }
    }

    /// MA4:跨 chunk 剔除 — chunk(0,0)@(15,-60,0) 与 chunk(1,0)@(16,-60,0) 相邻时 5 面,
    /// 移除 chunk(1,0) 后恢复 6 面。
    #[test]
    fn ma4_cross_chunk_face_culling() {
        let mut mirror = ChunkMirror::new();
        let mut c0 = Chunk::air();
        c0.set(15, 4, 0, STONE);
        let mut c1 = Chunk::air();
        c1.set(0, 4, 0, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), c0);
        mirror.insert_chunk(ChunkPos::new(1, 0), c1);

        let sc0 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let m_both = mesh_subchunk(&mirror, sc0);
        assert_eq!(
            m_both.positions.len(),
            20,
            "跨 chunk +X 被剔 → 5 面 = 20 顶点"
        );
        assert_eq!(m_both.indices.len(), 30);

        mirror.remove_chunk(ChunkPos::new(1, 0));
        let m_alone = mesh_subchunk(&mirror, sc0);
        assert_eq!(
            m_alone.positions.len(),
            24,
            "移除邻 chunk 后恢复 6 面 = 24 顶点"
        );
        assert_eq!(m_alone.indices.len(), 36);
    }

    /// MA5:段边界块 STONE@(0,-49,0)(局部 y=15) → 段 0 共 6 面、段 1 共 0 面;
    /// 挖空后段 0 与段 1 均标 Edit 脏。
    #[test]
    fn ma5_section_boundary_mesh_and_edit_dirty() {
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        chunk.set(0, 15, 0, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        let s0 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let s1 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 1,
        };
        assert_eq!(
            mesh_subchunk(&mirror, s0).positions.len(),
            24,
            "段 0 = 6 面"
        );
        assert_eq!(mesh_subchunk(&mirror, s1).positions.len(), 0, "段 1 = 0 面");

        while mirror.pop_dirty().is_some() {}
        mirror.apply_updates(&[(BlockPos::new(0, -49, 0), BlockId::AIR)]);

        assert_eq!(mirror.dirty_len(), 2);
        assert_eq!(mirror.pop_dirty(), Some((s0, DirtyKind::Edit)));
        assert_eq!(mirror.pop_dirty(), Some((s1, DirtyKind::Edit)));
        assert_eq!(mirror.pop_dirty(), None);
    }
}
