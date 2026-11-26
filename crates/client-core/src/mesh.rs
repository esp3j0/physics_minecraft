//! subchunk 网格生成 + 顶点 AO(REQ-013 C01b-1 §2.2)。
//!
//! 纯函数、零渲染依赖、零墙钟。输出世界坐标顶点/法向/烘焙顶点色与 `u32` 三角索引。

use std::sync::Arc;

use types::{BlockId, BlockPos, CHUNK_SIZE, ChunkPos, WORLD_Y_MAX, WORLD_Y_MIN};
use voxel::{CHUNK_X, CHUNK_Z, Chunk};

use crate::mirror::{ChunkMirror, SECTION_HEIGHT, SECTIONS_PER_CHUNK, SubchunkPos};

/// 面方向(6 邻居;声明序 = 提取遍历序,故带 Ord 供去重/比对;C02-1 自 `viewer::mesh` 迁入)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FaceDir {
    /// -x(西)。
    West,
    /// +x(东)。
    East,
    /// -y(下)。
    Bottom,
    /// +y(上)。
    Top,
    /// -z(北)。
    North,
    /// +z(南)。
    South,
}

impl FaceDir {
    /// 单位法线(轴对齐;x 东 / y 上 / z 南)。
    pub fn normal(self) -> [f32; 3] {
        match self {
            Self::West => [-1.0, 0.0, 0.0],
            Self::East => [1.0, 0.0, 0.0],
            Self::Bottom => [0.0, -1.0, 0.0],
            Self::Top => [0.0, 1.0, 0.0],
            Self::North => [0.0, 0.0, -1.0],
            Self::South => [0.0, 0.0, 1.0],
        }
    }

    /// 方向亮度(简单方向光照:法线 dot 光向定 0..=1 系数;顶面最亮、底面最暗)。
    pub fn brightness(self) -> f32 {
        match self {
            Self::Top => 1.0,
            Self::South | Self::North => 0.8,
            Self::East | Self::West => 0.6,
            Self::Bottom => 0.4,
        }
    }
}

/// 异步网格任务持有的只读体素快照:中心 chunk 与四水平邻 chunk。
///
/// `neighbors` 固定顺序为 `(-1, 0), (1, 0), (0, -1), (0, 1)`(与 `mirror::HORIZONTAL_NEIGHBORS` 一致)。
#[derive(Clone)]
pub struct ChunkSnapshot {
    pub center: Arc<Chunk>,
    pub neighbors: [Option<Arc<Chunk>>; 4],
}

/// 从镜像截取单段所需的只读快照(仅 clone `Arc`,零拷贝);中心 chunk 未装载返回 `None`。
pub fn snapshot(mirror: &ChunkMirror, sc: SubchunkPos) -> Option<ChunkSnapshot> {
    let center = mirror.chunk_arc(sc.chunk)?;
    let neighbors = [
        mirror.chunk_arc(ChunkPos::new(sc.chunk.x - 1, sc.chunk.z)),
        mirror.chunk_arc(ChunkPos::new(sc.chunk.x + 1, sc.chunk.z)),
        mirror.chunk_arc(ChunkPos::new(sc.chunk.x, sc.chunk.z - 1)),
        mirror.chunk_arc(ChunkPos::new(sc.chunk.x, sc.chunk.z + 1)),
    ];
    Some(ChunkSnapshot { center, neighbors })
}

/// 单个 subchunk 的网格几何数据(世界坐标)。
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SubchunkMesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub colors: Vec<[f32; 4]>,
    pub uvs: Vec<[f32; 2]>,
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

/// 生成单个 subchunk 的网格(纯函数):从镜像截取快照后委托 [`mesh_subchunk_snapshot`]。
/// 未装载 chunk、非法段号或全空气段返回四个全空 `Vec`。
pub fn mesh_subchunk(mirror: &ChunkMirror, sc: SubchunkPos) -> SubchunkMesh {
    if sc.section >= SECTIONS_PER_CHUNK {
        return SubchunkMesh::default();
    }
    let Some(snap) = snapshot(mirror, sc) else {
        return SubchunkMesh::default();
    };
    mesh_subchunk_snapshot(&snap, sc)
}

/// 基于只读快照 [`ChunkSnapshot`] 生成单个 subchunk 的网格(纯函数,供异步任务池调用)。
///
/// 算法与输出(面序/绕序/AO/方向亮度/方块色)与 [`mesh_subchunk`] 完全一致;
/// 邻格与 AO 采样走 `center` + 四水平邻 `(-1,0),(1,0),(0,-1),(0,1)`,越界或 `None` 视为空气。
pub fn mesh_subchunk_snapshot(snap: &ChunkSnapshot, sc: SubchunkPos) -> SubchunkMesh {
    let mut mesh = SubchunkMesh::default();
    if sc.section >= SECTIONS_PER_CHUNK {
        return mesh;
    }
    let chunk = snap.center.as_ref();

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
                    if is_solid_in_snapshot(snap, sc.chunk, neighbor_pos) {
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
                            is_solid_in_snapshot(
                                snap,
                                sc.chunk,
                                BlockPos::new(s1[0], s1[1], s1[2]),
                            ),
                            is_solid_in_snapshot(
                                snap,
                                sc.chunk,
                                BlockPos::new(s2[0], s2[1], s2[2]),
                            ),
                            is_solid_in_snapshot(snap, sc.chunk, BlockPos::new(c[0], c[1], c[2])),
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
                        mesh.uvs.push(crate::texture::face_uv(
                            block,
                            corner[u_axis] as u32,
                            corner[v_axis] as u32,
                        ));
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

/// 描边线段网格(LineList 端点对,世界坐标;C07)。
#[derive(Debug, Clone, Default, PartialEq)]
pub struct OutlineMesh {
    pub positions: Vec<[f32; 3]>,
    pub colors: Vec<[f32; 4]>,
}

/// 描边角点外扩偏移(米;防与面片深度冲突)。
pub const OUTLINE_OFFSET: f32 = 0.002;
/// 方块棱线颜色(淡黑)。
pub const OUTLINE_BLOCK_COLOR: [f32; 4] = [0.0, 0.0, 0.0, 0.25];
/// 区块/段分界棱线颜色(琥珀)。
pub const OUTLINE_BOUNDARY_COLOR: [f32; 4] = [1.0, 0.75, 0.2, 0.5];

/// 生成单个 subchunk 的描边线段网格(纯函数):从镜像截取快照后委托 [`outline_subchunk_snapshot`]。
/// 未装载 chunk、非法段号或全空气段返回空 [`OutlineMesh::default`]。
pub fn outline_subchunk(mirror: &ChunkMirror, sc: SubchunkPos) -> OutlineMesh {
    if sc.section >= SECTIONS_PER_CHUNK {
        return OutlineMesh::default();
    }
    let Some(snap) = snapshot(mirror, sc) else {
        return OutlineMesh::default();
    };
    outline_subchunk_snapshot(&snap, sc)
}

/// 基于只读快照 [`ChunkSnapshot`] 生成单个 subchunk 的描边线段网格(纯函数;C07)。
///
/// 遍历序与 [`mesh_subchunk_snapshot`] 完全相同;暴露面四棱经无序整数端点对去重(first-wins),
/// 逐端点三轴角点外扩 [`OUTLINE_OFFSET`],按偏移前整数坐标分档着色。
pub fn outline_subchunk_snapshot(snap: &ChunkSnapshot, sc: SubchunkPos) -> OutlineMesh {
    let mut mesh = OutlineMesh::default();
    if sc.section >= SECTIONS_PER_CHUNK {
        return mesh;
    }
    let chunk = snap.center.as_ref();

    let origin_x = sc.chunk.x * CHUNK_SIZE;
    let origin_z = sc.chunk.z * CHUNK_SIZE;
    let y_start = usize::from(sc.section) * SECTION_HEIGHT;
    let y_end = y_start + SECTION_HEIGHT;
    let section_height = SECTION_HEIGHT as i32;

    let mut seen: std::collections::BTreeSet<([i32; 3], [i32; 3])> =
        std::collections::BTreeSet::new();

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
                let cell = [wx, wy, wz];

                for face in &FACES {
                    let mut outer = cell;
                    outer[face.axis] += face.sign;
                    let neighbor_pos = BlockPos::new(outer[0], outer[1], outer[2]);
                    if is_solid_in_snapshot(snap, sc.chunk, neighbor_pos) {
                        continue;
                    }

                    for (i, &c0) in face.corners.iter().enumerate() {
                        let c1 = face.corners[(i + 1) % 4];
                        let p0 = [cell[0] + c0[0], cell[1] + c0[1], cell[2] + c0[2]];
                        let p1 = [cell[0] + c1[0], cell[1] + c1[1], cell[2] + c1[2]];
                        let key = if p0 <= p1 { (p0, p1) } else { (p1, p0) };
                        if !seen.insert(key) {
                            continue;
                        }

                        let is_boundary = (p0[0].rem_euclid(CHUNK_SIZE) == 0
                            && p1[0].rem_euclid(CHUNK_SIZE) == 0)
                            || (p0[2].rem_euclid(CHUNK_SIZE) == 0
                                && p1[2].rem_euclid(CHUNK_SIZE) == 0)
                            || ((p0[1] - WORLD_Y_MIN).rem_euclid(section_height) == 0
                                && (p1[1] - WORLD_Y_MIN).rem_euclid(section_height) == 0);
                        let color = if is_boundary {
                            OUTLINE_BOUNDARY_COLOR
                        } else {
                            OUTLINE_BLOCK_COLOR
                        };

                        let offset_pt = |p: [i32; 3], c: [i32; 3]| -> [f32; 3] {
                            [
                                p[0] as f32
                                    + if c[0] == 0 {
                                        -OUTLINE_OFFSET
                                    } else {
                                        OUTLINE_OFFSET
                                    },
                                p[1] as f32
                                    + if c[1] == 0 {
                                        -OUTLINE_OFFSET
                                    } else {
                                        OUTLINE_OFFSET
                                    },
                                p[2] as f32
                                    + if c[2] == 0 {
                                        -OUTLINE_OFFSET
                                    } else {
                                        OUTLINE_OFFSET
                                    },
                            ]
                        };

                        mesh.positions.push(offset_pt(p0, c0));
                        mesh.positions.push(offset_pt(p1, c1));
                        mesh.colors.push(color);
                        mesh.colors.push(color);
                    }
                }
            }
        }
    }

    mesh
}

#[inline]
fn is_solid_in_snapshot(snap: &ChunkSnapshot, center_pos: ChunkPos, world: BlockPos) -> bool {
    if !(WORLD_Y_MIN..=WORLD_Y_MAX).contains(&world.y) {
        return false;
    }
    let cpos = world.chunk_of();
    let dx = cpos.x - center_pos.x;
    let dz = cpos.z - center_pos.z;
    let chunk: &Chunk = match (dx, dz) {
        (0, 0) => snap.center.as_ref(),
        (-1, 0) => match snap.neighbors[0].as_deref() {
            Some(c) => c,
            None => return false,
        },
        (1, 0) => match snap.neighbors[1].as_deref() {
            Some(c) => c,
            None => return false,
        },
        (0, -1) => match snap.neighbors[2].as_deref() {
            Some(c) => c,
            None => return false,
        },
        (0, 1) => match snap.neighbors[3].as_deref() {
            Some(c) => c,
            None => return false,
        },
        _ => return false,
    };
    let lx = world.x.rem_euclid(CHUNK_SIZE) as usize;
    let ly = (world.y - WORLD_Y_MIN) as usize;
    let lz = world.z.rem_euclid(CHUNK_SIZE) as usize;
    !chunk.get(lx, ly, lz).is_air()
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

    /// MS1(C01b-2):L 形快照网格与 `mesh_subchunk` 四 Vec 逐元素相等(56 顶点/84 索引);
    /// 跨 chunk(MA4 双 chunk)快照版 positions.len()==20。
    #[test]
    fn ms1_snapshot_mesh_matches_direct_and_cross_chunk() {
        // L 形(同 MA3):B1(0,-60,0), B2(1,-60,0), B3(1,-59,0)
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        chunk.set(0, 4, 0, STONE);
        chunk.set(1, 4, 0, STONE);
        chunk.set(1, 5, 0, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        let s0 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let snap = snapshot(&mirror, s0).expect("chunk(0,0) 已装载");
        let m_snap = mesh_subchunk_snapshot(&snap, s0);
        let m_direct = mesh_subchunk(&mirror, s0);

        assert_eq!(m_snap.positions, m_direct.positions);
        assert_eq!(m_snap.normals, m_direct.normals);
        assert_eq!(m_snap.colors, m_direct.colors);
        assert_eq!(m_snap.indices, m_direct.indices);
        assert_eq!(m_snap.positions.len(), 56);
        assert_eq!(m_snap.indices.len(), 84);

        // 跨 chunk(同 MA4):chunk(0,0)@(15,-60,0) 与 chunk(1,0)@(16,-60,0)
        let mut cross = ChunkMirror::new();
        let mut c0 = Chunk::air();
        c0.set(15, 4, 0, STONE);
        let mut c1 = Chunk::air();
        c1.set(0, 4, 0, STONE);
        cross.insert_chunk(ChunkPos::new(0, 0), c0);
        cross.insert_chunk(ChunkPos::new(1, 0), c1);

        let snap_cross = snapshot(&cross, s0).expect("chunk(0,0) 已装载");
        let m_cross = mesh_subchunk_snapshot(&snap_cross, s0);
        assert_eq!(m_cross.positions.len(), 20);
    }

    type EdgeSeg = ([f32; 3], [f32; 3]);

    fn pt_eq(a: [f32; 3], b: [f32; 3]) -> bool {
        (a[0] - b[0]).abs() <= EPS && (a[1] - b[1]).abs() <= EPS && (a[2] - b[2]).abs() <= EPS
    }

    fn pt_lt(a: [f32; 3], b: [f32; 3]) -> bool {
        for axis in 0..3 {
            if (a[axis] - b[axis]).abs() > EPS {
                return a[axis] < b[axis];
            }
        }
        false
    }

    fn norm_seg(a: [f32; 3], b: [f32; 3]) -> EdgeSeg {
        if pt_lt(b, a) { (b, a) } else { (a, b) }
    }

    fn seg_eq(lhs: EdgeSeg, rhs: EdgeSeg) -> bool {
        let l = norm_seg(lhs.0, lhs.1);
        let r = norm_seg(rhs.0, rhs.1);
        pt_eq(l.0, r.0) && pt_eq(l.1, r.1)
    }

    fn color_eq(a: [f32; 4], b: [f32; 4]) -> bool {
        a.iter().zip(b.iter()).all(|(x, y)| (x - y).abs() <= EPS)
    }

    fn extract_segments(o: &OutlineMesh) -> Vec<(EdgeSeg, [f32; 4])> {
        assert_eq!(o.positions.len(), o.colors.len());
        assert_eq!(o.positions.len() % 2, 0);
        let mut out = Vec::with_capacity(o.positions.len() / 2);
        for pair in 0..(o.positions.len() / 2) {
            let i = pair * 2;
            assert!(
                color_eq(o.colors[i], o.colors[i + 1]),
                "线段 {pair} 两端点颜色必须一致"
            );
            out.push((norm_seg(o.positions[i], o.positions[i + 1]), o.colors[i]));
        }
        out
    }

    fn assert_segment_set_eq(actual: &[EdgeSeg], expected: &[EdgeSeg], label: &str) {
        assert_eq!(
            actual.len(),
            expected.len(),
            "{label}: 线段数不符, 实测 {}, 期望 {}",
            actual.len(),
            expected.len()
        );
        let mut matched = vec![false; actual.len()];
        for exp in expected {
            let found = actual
                .iter()
                .enumerate()
                .find(|(idx, act)| !matched[*idx] && seg_eq(**act, *exp));
            let Some((idx, _)) = found else {
                panic!("{label}: 缺失期望线段 {exp:?}, 实测={actual:?}");
            };
            matched[idx] = true;
        }
    }

    /// MO1:单块 12 棱 — chunk(0,0) 仅 (1,-63,1) STONE → 24 顶点、12 线段、全淡黑档、三轴角点外扩。
    #[test]
    fn mo1_single_stone_12_edges() {
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        chunk.set(1, 1, 1, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        let sc = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let o = outline_subchunk(&mirror, sc);
        assert_eq!(o.positions.len(), 24);
        assert_eq!(o.colors.len(), 24);

        let segs = extract_segments(&o);
        assert_eq!(segs.len(), 12);
        for (i, (_, c)) in segs.iter().enumerate() {
            assert_color_eq(*c, OUTLINE_BLOCK_COLOR, &format!("MO1 线段 {i}"));
        }

        let expected: [EdgeSeg; 12] = [
            // 沿 x 4 条
            ([0.998, -63.002, 0.998], [2.002, -63.002, 0.998]),
            ([0.998, -63.002, 2.002], [2.002, -63.002, 2.002]),
            ([0.998, -61.998, 0.998], [2.002, -61.998, 0.998]),
            ([0.998, -61.998, 2.002], [2.002, -61.998, 2.002]),
            // 沿 y 4 条
            ([0.998, -63.002, 0.998], [0.998, -61.998, 0.998]),
            ([0.998, -63.002, 2.002], [0.998, -61.998, 2.002]),
            ([2.002, -63.002, 0.998], [2.002, -61.998, 0.998]),
            ([2.002, -63.002, 2.002], [2.002, -61.998, 2.002]),
            // 沿 z 4 条
            ([0.998, -63.002, 0.998], [0.998, -63.002, 2.002]),
            ([0.998, -61.998, 0.998], [0.998, -61.998, 2.002]),
            ([2.002, -63.002, 0.998], [2.002, -63.002, 2.002]),
            ([2.002, -61.998, 0.998], [2.002, -61.998, 2.002]),
        ];
        let actual: Vec<EdgeSeg> = segs.iter().map(|(s, _)| *s).collect();
        assert_segment_set_eq(&actual, &expected, "MO1");
    }

    /// MO2:相邻两块 (1,-63,1) 与 (2,-63,1) → 20 线段(40 顶点),接缝棱保留且全淡黑档。
    #[test]
    fn mo2_adjacent_blocks_keep_seam_edge() {
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        chunk.set(1, 1, 1, STONE);
        chunk.set(2, 1, 1, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        let sc = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let o = outline_subchunk(&mirror, sc);
        assert_eq!(o.positions.len(), 40);
        assert_eq!(o.colors.len(), 40);

        let segs = extract_segments(&o);
        assert_eq!(segs.len(), 20);
        for (i, (_, c)) in segs.iter().enumerate() {
            assert_color_eq(*c, OUTLINE_BLOCK_COLOR, &format!("MO2 线段 {i}"));
        }

        let seam = ([2.002, -61.998, 0.998], [2.002, -61.998, 2.002]);
        assert!(
            segs.iter().any(|(s, _)| seg_eq(*s, seam)),
            "MO2: x=2 接缝棱 {seam:?} 必须存在"
        );
    }

    /// MO3:角块 (0,-64,0) 分档 9/3 — 边界棱 9 条琥珀、方块棱 3 条淡黑。
    #[test]
    fn mo3_corner_block_9_boundary_3_inner() {
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        chunk.set(0, 0, 0, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        let sc = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let o = outline_subchunk(&mirror, sc);
        assert_eq!(o.positions.len(), 24);
        assert_eq!(o.colors.len(), 24);

        let segs = extract_segments(&o);
        assert_eq!(segs.len(), 12);

        let mut boundary_segs = Vec::new();
        let mut block_segs = Vec::new();
        for (seg, color) in segs {
            if color_eq(color, OUTLINE_BOUNDARY_COLOR) {
                boundary_segs.push(seg);
            } else if color_eq(color, OUTLINE_BLOCK_COLOR) {
                block_segs.push(seg);
            } else {
                panic!("MO3: 未知描边色 {color:?}");
            }
        }
        assert_eq!(boundary_segs.len(), 9, "MO3: 边界棱应恰 9 条");
        assert_eq!(block_segs.len(), 3, "MO3: 方块棱应恰 3 条");

        let expected_inner: [EdgeSeg; 3] = [
            ([1.002, -62.998, -0.002], [1.002, -62.998, 1.002]),
            ([1.002, -64.002, 1.002], [1.002, -62.998, 1.002]),
            ([-0.002, -62.998, 1.002], [1.002, -62.998, 1.002]),
        ];
        assert_segment_set_eq(&block_segs, &expected_inner, "MO3 方块棱");
    }

    /// MO4:跨 chunk 邻居剔除 + 边界档(单 subchunk 语义):
    /// chunk(0,0)@(15,-63,1) 与 chunk(1,0)@(16,-63,1) 均装载,对 (0,0) 段 0 输出 12 棱(4 琥珀/8 淡黑),
    /// 且所有端点 x <= 16.002(邻 chunk B 未被遍历)。
    #[test]
    fn mo4_cross_chunk_culling_and_boundary_tier() {
        let mut mirror = ChunkMirror::new();
        let mut c0 = Chunk::air();
        c0.set(15, 1, 1, STONE);
        let mut c1 = Chunk::air();
        c1.set(0, 1, 1, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), c0);
        mirror.insert_chunk(ChunkPos::new(1, 0), c1);

        let sc = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let o = outline_subchunk(&mirror, sc);
        assert_eq!(o.positions.len(), 24);
        assert_eq!(o.colors.len(), 24);

        for p in &o.positions {
            assert!(
                p[0] <= 16.002 + EPS,
                "MO4: 端点 x={} 不得超过 16.002(邻 chunk 不得被遍历)",
                p[0]
            );
        }

        let segs = extract_segments(&o);
        assert_eq!(segs.len(), 12);

        let mut boundary_segs = Vec::new();
        let mut block_segs = Vec::new();
        for (seg, color) in segs {
            if color_eq(color, OUTLINE_BOUNDARY_COLOR) {
                boundary_segs.push(seg);
            } else if color_eq(color, OUTLINE_BLOCK_COLOR) {
                block_segs.push(seg);
            } else {
                panic!("MO4: 未知描边色 {color:?}");
            }
        }
        assert_eq!(boundary_segs.len(), 4, "MO4: 边界棱应恰 4 条");
        assert_eq!(block_segs.len(), 8, "MO4: 方块棱应恰 8 条");

        let expected_boundary: [EdgeSeg; 4] = [
            // 沿 y 2 条
            ([16.002, -63.002, 0.998], [16.002, -61.998, 0.998]),
            ([16.002, -63.002, 2.002], [16.002, -61.998, 2.002]),
            // 沿 z 2 条
            ([16.002, -63.002, 0.998], [16.002, -63.002, 2.002]),
            ([16.002, -61.998, 0.998], [16.002, -61.998, 2.002]),
        ];
        assert_segment_set_eq(&boundary_segs, &expected_boundary, "MO4 边界棱");
    }

    /// MO5:守卫与便捷函数 — 全空气段/非法段/未装载 chunk 返回 default;装载情形两入口结果一致。
    #[test]
    fn mo5_guards_and_convenience_equivalence() {
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        chunk.set(2, 2, 2, STONE);
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        // 全空气段(section 1)
        let sc_air = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 1,
        };
        assert_eq!(outline_subchunk(&mirror, sc_air), OutlineMesh::default());

        // 非法段号
        let sc_invalid = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: SECTIONS_PER_CHUNK,
        };
        assert_eq!(
            outline_subchunk(&mirror, sc_invalid),
            OutlineMesh::default()
        );
        let snap0 = snapshot(
            &mirror,
            SubchunkPos {
                chunk: ChunkPos::new(0, 0),
                section: 0,
            },
        )
        .expect("chunk(0,0) 已装载");
        assert_eq!(
            outline_subchunk_snapshot(&snap0, sc_invalid),
            OutlineMesh::default()
        );

        // 未装载 chunk
        let sc_unloaded = SubchunkPos {
            chunk: ChunkPos::new(9, 9),
            section: 0,
        };
        assert_eq!(
            outline_subchunk(&mirror, sc_unloaded),
            OutlineMesh::default()
        );

        // 装载情形两入口等价
        let sc0 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        assert_eq!(
            outline_subchunk(&mirror, sc0),
            outline_subchunk_snapshot(&snapshot(&mirror, sc0).unwrap(), sc0)
        );
    }

    /// MA6(REQ-013 C11):单块 `BlockId(2)` 与未知方块 `BlockId(9)` 的图集 UV 锚点及全空气段守卫。
    #[test]
    fn ma6_uv_atlas_anchors() {
        let mut mirror = ChunkMirror::new();
        let mut chunk = Chunk::air();
        chunk.set(0, 4, 0, BlockId(2));
        mirror.insert_chunk(ChunkPos::new(0, 0), chunk);

        let sc0 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let m0 = mesh_subchunk(&mirror, sc0);
        assert_eq!(m0.uvs.len(), m0.positions.len());
        for uv in &m0.uvs {
            assert!((0.0..=1.0).contains(&uv[0]), "s 越界: {:?}", uv);
            assert!((0.0..=1.0).contains(&uv[1]), "v 越界: {:?}", uv);
        }

        let uvs_for_normal = |m: &SubchunkMesh, normal: [f32; 3]| -> Vec<[f32; 2]> {
            m.normals
                .iter()
                .zip(m.uvs.iter())
                .filter(|(n, _)| **n == normal)
                .map(|(_, uv)| *uv)
                .collect()
        };

        assert_eq!(
            uvs_for_normal(&m0, [0.0, 1.0, 0.0]),
            vec![
                [0.25390625, 0.96875],
                [0.25390625, 0.03125],
                [0.37109375, 0.03125],
                [0.37109375, 0.96875],
            ]
        );
        assert_eq!(
            uvs_for_normal(&m0, [1.0, 0.0, 0.0]),
            vec![
                [0.25390625, 0.96875],
                [0.37109375, 0.96875],
                [0.37109375, 0.03125],
                [0.25390625, 0.03125],
            ]
        );

        // 未知方块 BlockId(9) 走 tile 7 fallback。
        let mut unknown_mirror = ChunkMirror::new();
        let mut unknown_chunk = Chunk::air();
        unknown_chunk.set(0, 4, 0, BlockId(9));
        unknown_mirror.insert_chunk(ChunkPos::new(0, 0), unknown_chunk);
        let mu = mesh_subchunk(&unknown_mirror, sc0);
        let top_u = uvs_for_normal(&mu, [0.0, 1.0, 0.0]);
        assert_eq!(top_u[0], [0.87890625, 0.96875]);
        assert_eq!(top_u[1], [0.87890625, 0.03125]);

        // 全空气段:uvs 为空。
        let m1 = mesh_subchunk(
            &mirror,
            SubchunkPos {
                chunk: ChunkPos::new(0, 0),
                section: 1,
            },
        );
        assert!(m1.uvs.is_empty());
    }
}
