//! mesh —— 暴露面提取 / 颜色映射 / 顶点生成纯函数(REQ-004A 任务 01 接口设计;
//! ACV3 单测主战场)。渲染侧 Mesh 组装的唯一算力来源,render 只做窗口编排。
//!
//! 方向约定(与 types::coords 一致):x 东 / y 上 / z 南;格 (x,y,z) 覆盖
//! [x,x+1)³,体素中心在 (x+0.5, y+0.5, z+0.5)。
//!
//! 暴露面判定(README 决策 4):6 邻居 air(或界外)即暴露——chunk 内自实现,
//! 不引 physics-sim(collider_build 是碰撞薄片,语义与渲染面不同)。

use macroquad::models::{Mesh, Vertex};
use types::BlockId;
use voxel::Chunk;

/// 面方向(6 邻居;声明序 = 提取遍历序,故带 Ord 供去重/比对)。
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

/// 6 邻居暴露面提取:6 邻居为 air(**或界外**)即暴露。
/// 返回 [(局部格, 面方向)](枚举序遍历,序确定)。
pub fn exposed_faces(chunk: &Chunk) -> Vec<((u32, u32, u32), FaceDir)> {
    let mut faces = Vec::new();
    for y in 0..voxel::CHUNK_Y {
        for z in 0..voxel::CHUNK_Z {
            for x in 0..voxel::CHUNK_X {
                if chunk.get(x, y, z).is_air() {
                    continue; // 空气不产出面
                }
                // (邻格 air 或邻格在界外)→ 该方向暴露。侧向判界用 usize
                // 比较防回绕;y=0 底邻 y=-1 界外 = 暴露(chunk 全高,底即世界底)。
                let dirs = [
                    (x == 0 || chunk.get(x - 1, y, z).is_air(), FaceDir::West),
                    (
                        x + 1 >= voxel::CHUNK_X || chunk.get(x + 1, y, z).is_air(),
                        FaceDir::East,
                    ),
                    (y == 0 || chunk.get(x, y - 1, z).is_air(), FaceDir::Bottom),
                    (
                        y + 1 >= voxel::CHUNK_Y || chunk.get(x, y + 1, z).is_air(),
                        FaceDir::Top,
                    ),
                    (z == 0 || chunk.get(x, y, z - 1).is_air(), FaceDir::North),
                    (
                        z + 1 >= voxel::CHUNK_Z || chunk.get(x, y, z + 1).is_air(),
                        FaceDir::South,
                    ),
                ];
                for (exposed, dir) in dirs {
                    if exposed {
                        faces.push(((x as u32, y as u32, z as u32), dir));
                    }
                }
            }
        }
    }
    faces
}

/// BlockId → RGBA(表:air 透明 / stone 灰 (0.55,0.55,0.58) / dirt 棕
/// (0.45,0.33,0.22);未知 → 品红警示色)。
pub fn block_color(id: BlockId) -> [f32; 4] {
    match id {
        BlockId::AIR => [0.0, 0.0, 0.0, 0.0],
        BlockId(1) => [0.55, 0.55, 0.58, 1.0],
        BlockId(2) => [0.45, 0.33, 0.22, 1.0],
        _ => [1.0, 0.0, 1.0, 1.0], // 品红警示色(未知方块不静默)
    }
}

/// chunk + 世界原点 → macroquad Mesh(每暴露面 4 顶点 2 三角,面朝向正确,
/// 简单方向光照:法线 dot 定亮度;顶点色 = 方块色 × 亮度,按 macroquad
/// Vertex 的 RGBA8 量化写入)。索引 = 每面 [0,1,2, 0,2,3] 增量;
/// 顶点数 = 暴露面 × 4。
///
/// 宏观超界防御:macroquad 单 drawcall 上限(vertex/index)由渲染端分块
/// 调用规避(render 每 chunk 一 Mesh;16×16×384 chunk 单 Mesh 顶点量级
/// ≤ 数万,在上限内),本函数不截断。
pub fn chunk_mesh(chunk: &Chunk, origin: (f32, f32, f32)) -> Mesh {
    let (ox, oy, oz) = origin;
    let faces = exposed_faces(chunk);
    let mut vertices = Vec::with_capacity(faces.len() * 4);
    let mut indices = Vec::with_capacity(faces.len() * 6);
    for ((x, y, z), dir) in faces {
        // 格世界最小角(格覆盖 [x,x+1)³);四角按面上逆时针 CCW(从面外看)排列。
        let (bx, by, bz) = (ox + x as f32, oy + y as f32, oz + z as f32);
        let corners: [[f32; 3]; 4] = match dir {
            FaceDir::West => [
                [bx, by, bz],
                [bx, by, bz + 1.0],
                [bx, by + 1.0, bz + 1.0],
                [bx, by + 1.0, bz],
            ],
            FaceDir::East => [
                [bx + 1.0, by, bz + 1.0],
                [bx + 1.0, by, bz],
                [bx + 1.0, by + 1.0, bz],
                [bx + 1.0, by + 1.0, bz + 1.0],
            ],
            FaceDir::Bottom => [
                [bx, by, bz + 1.0],
                [bx, by, bz],
                [bx + 1.0, by, bz],
                [bx + 1.0, by, bz + 1.0],
            ],
            FaceDir::Top => [
                [bx, by + 1.0, bz],
                [bx, by + 1.0, bz + 1.0],
                [bx + 1.0, by + 1.0, bz + 1.0],
                [bx + 1.0, by + 1.0, bz],
            ],
            FaceDir::North => [
                [bx + 1.0, by, bz],
                [bx, by, bz],
                [bx, by + 1.0, bz],
                [bx + 1.0, by + 1.0, bz],
            ],
            FaceDir::South => [
                [bx, by, bz + 1.0],
                [bx + 1.0, by, bz + 1.0],
                [bx + 1.0, by + 1.0, bz + 1.0],
                [bx, by + 1.0, bz + 1.0],
            ],
        };
        // 顶点色 = 方块色 × 方向亮度;写入前按 macroquad From<Color> 同式
        // 量化 RGBA8(管线顶点布局固定 [u8;4])。
        let color = block_color(chunk.get(x as usize, y as usize, z as usize));
        let b = dir.brightness();
        let rgba: [u8; 4] = [
            ((color[0] * b) * 255.0) as u8,
            ((color[1] * b) * 255.0) as u8,
            ((color[2] * b) * 255.0) as u8,
            (color[3] * 255.0) as u8,
        ];
        let n = dir.normal();
        let base = vertices.len() as u16;
        for c in corners {
            vertices.push(Vertex {
                position: macroquad::prelude::vec3(c[0], c[1], c[2]),
                uv: macroquad::prelude::vec2(0.0, 0.0),
                color: rgba,
                normal: macroquad::prelude::vec4(n[0], n[1], n[2], 0.0),
            });
        }
        // 每面 2 三角:CCW 四角 (0,1,2) + (0,2,3)。
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    Mesh {
        vertices,
        indices,
        texture: None,
    }
}

// ---- 测试清单(任务书 01;纯函数;窗口行为走 ACV4 手动清单) ----

#[cfg(test)]
mod tests {
    use super::*;
    use voxel::{CHUNK_X, CHUNK_Y, CHUNK_Z, WorldGenerator};

    const STONE: BlockId = BlockId(1);
    const DIRT: BlockId = BlockId(2);
    const UNKNOWN: BlockId = BlockId(42);

    /// 超平坦经典层表(stone×1 + dirt×3,顶面局部 y=3)。
    fn superflat() -> Chunk {
        use types::{ChunkPos, WorldSeed};
        voxel::Superflat::classic().generate(WorldSeed(0), ChunkPos::new(0, 0))
    }

    /// 数暴露面(块, 方向)出现次数。
    fn count(faces: &[((u32, u32, u32), FaceDir)], dir: FaceDir) -> usize {
        faces.iter().filter(|(_, d)| *d == dir).count()
    }

    /// 测试清单 `exposed_faces_count_superflat_surface`:超平坦 chunk——顶面
    /// 256(层表顶 y=3 的 16×16)+ 侧面界外暴露数 = 4 层 × 4 边 × 16 = 256
    /// (x 两侧 y∈[0,4) 各 64,z 两侧同),埋藏块零面;底面(y=0,邻 y=-1
    /// 界外)亦暴露 = 256。
    #[test]
    fn exposed_faces_count_superflat_surface() {
        let chunk = superflat();
        let faces = exposed_faces(&chunk);

        // 顶面:y=3 一整层 256。
        assert_eq!(count(&faces, FaceDir::Top), 256, "顶面 = 16×16 = 256");
        // 埋藏面:层间相邻实心块不暴露;东西/南北向内部块互不暴露。
        assert_eq!(count(&faces, FaceDir::Bottom), 256, "底面 y=0 邻界外 = 256");
        // 侧向:4 层 × 每边 16 = 每方向 64。
        assert_eq!(count(&faces, FaceDir::West), 64, "西侧面(x=0,4 层)");
        assert_eq!(count(&faces, FaceDir::East), 64, "东侧面(x=15,4 层)");
        assert_eq!(count(&faces, FaceDir::North), 64, "北侧面(z=0,4 层)");
        assert_eq!(count(&faces, FaceDir::South), 64, "南侧面(z=15,4 层)");
        // 总数闭合:256×2 + 64×4 = 768。
        assert_eq!(faces.len(), 768, "总数 = 顶256 + 底256 + 侧256");

        // 无重复(同格同方向只出现一次)。
        let mut uniq = faces.clone();
        uniq.sort();
        let before = uniq.len();
        uniq.dedup();
        assert_eq!(uniq.len(), before, "无重复 (格, 方向) 对");

        // 抽查语义:顶面条目恰是 y=3 的格;x>0 的格不得出现 West 面。
        assert!(
            faces
                .iter()
                .all(|((_, y, _), d)| *d != FaceDir::Top || *y == 3),
            "顶面条目必在局部 y=3"
        );
        assert!(
            faces
                .iter()
                .all(|((x, _, _), d)| *d != FaceDir::West || *x == 0),
            "West 面条目必在 x=0(邻块为 air/界外才暴露)"
        );
    }

    /// 测试清单 `exposed_faces_respects_neighbors`:空气 chunk 中的中央凸块
    /// (单个 stone):5 面暴露(底面亦暴露——邻 air);随后把它埋进超平坦:
    /// 底面贴地(邻 dirt)不暴露,顶面 + 四侧 = 5 面。
    #[test]
    fn exposed_faces_respects_neighbors() {
        // 悬空单块:6 邻全 air → 6 面全暴露。
        let mut floating = Chunk::air();
        floating.set(8, 100, 8, STONE);
        let faces = exposed_faces(&floating);
        assert_eq!(faces.len(), 6, "悬空单块 6 面全暴露");
        for dir in [
            FaceDir::West,
            FaceDir::East,
            FaceDir::Bottom,
            FaceDir::Top,
            FaceDir::North,
            FaceDir::South,
        ] {
            assert_eq!(count(&faces, dir), 1, "{dir:?} 恰 1 面");
        }

        // 贴地凸块:埋在超平坦顶面上(替换 y=4 的一个 air 格)→ 底面贴地
        // (邻 y=3 的 dirt)不暴露;顶 + 四侧 = 5 面。注意:凸块同时**盖住**
        // 下方 (8,3,8) 的原顶面 —— 全 chunk 顶面计数不变(255 面顶 + 凸块
        // 顶 = 256),总数 = 768 + 5 - 1 = 772。
        let mut bump = superflat();
        bump.set(8, 4, 8, STONE);
        let faces = exposed_faces(&bump);
        assert_eq!(
            count(&faces, FaceDir::Top),
            256,
            "顶面 255 + 凸块顶 1(下方原顶面被盖住)"
        );
        assert_eq!(
            count(&faces, FaceDir::Bottom),
            256,
            "底面仍 256(凸块底贴地)"
        );
        assert_eq!(
            count(&faces, FaceDir::West) + count(&faces, FaceDir::East),
            128 + 2,
            "东西侧面 + 凸块两肩"
        );
        assert_eq!(faces.len(), 772, "总数 = 768 + 凸块 5 面 − 被盖住的 1 顶面");

        // 凸块本格恰 5 面暴露,底面不在其中(贴地不暴露——任务书断言意图)。
        let bump_faces: Vec<FaceDir> = faces
            .iter()
            .filter(|((x, y, z), _)| (*x, *y, *z) == (8, 4, 8))
            .map(|(_, d)| *d)
            .collect();
        assert_eq!(bump_faces.len(), 5, "凸块 5 面暴露(实 {bump_faces:?})");
        assert!(!bump_faces.contains(&FaceDir::Bottom), "凸块底面贴地不暴露");

        // 空气格本身零面(air 不产出面)。
        let empty = Chunk::air();
        assert!(exposed_faces(&empty).is_empty(), "全 air chunk 零面");
    }

    /// 测试清单 `block_color_table`:三内置色(air 透明/stone 灰/dirt 棕)
    /// + 未知品红警示色。
    #[test]
    fn block_color_table() {
        let air = block_color(BlockId::AIR);
        assert_eq!(air[3], 0.0, "air alpha = 0(透明)");
        assert_eq!(air[0], 0.0, "air r = 0(全透明色值无关紧要,取黑)");

        let stone = block_color(STONE);
        assert_eq!(stone[0], 0.55, "stone r");
        assert_eq!(stone[1], 0.55, "stone g");
        assert_eq!(stone[2], 0.58, "stone b");
        assert_eq!(stone[3], 1.0, "stone alpha = 1");

        let dirt = block_color(DIRT);
        assert_eq!(dirt[0], 0.45, "dirt r");
        assert_eq!(dirt[1], 0.33, "dirt g");
        assert_eq!(dirt[2], 0.22, "dirt b");
        assert_eq!(dirt[3], 1.0, "dirt alpha = 1");

        // 未知(未进表的实心方块)→ 品红 (1,0,1) 警示色,不透明。
        let magenta = block_color(UNKNOWN);
        assert_eq!(magenta[0], 1.0, "品红 r");
        assert_eq!(magenta[1], 0.0, "品红 g");
        assert_eq!(magenta[2], 1.0, "品红 b");
        assert_eq!(magenta[3], 1.0, "品红 alpha = 1");
        // 极大未知 ID 同样品红(表只按已知 ID 匹配)。
        assert_eq!(block_color(BlockId(u16::MAX)), magenta, "任意未知 → 品红");
    }

    /// 测试清单 `chunk_mesh_vertex_count_matches_faces`:顶点数 = 暴露面 × 4,
    /// 索引数 = 暴露面 × 6;每面法线非零且与方向一致;亮度 ∈ (0,1] 非零;
    /// 顶点位置 = 原点 + 局部格(世界坐标正确)。
    #[test]
    fn chunk_mesh_vertex_count_matches_faces() {
        // 单块场景,几何可手算:原点 (100, -64, -32),格 (2, 100, 3) 一块 stone。
        let mut chunk = Chunk::air();
        chunk.set(2, 100, 3, STONE);
        let origin = (100.0_f32, -64.0_f32, -32.0_f32);
        let mesh = chunk_mesh(&chunk, origin);

        let faces = exposed_faces(&chunk);
        assert_eq!(faces.len(), 6);
        assert_eq!(mesh.vertices.len(), faces.len() * 4, "顶点 = 面数 × 4");
        assert_eq!(mesh.indices.len(), faces.len() * 6, "索引 = 面数 × 6");
        assert!(mesh.texture.is_none(), "无纹理(纯色顶点)");

        // 索引模式:每面 [base, base+1, base+2, base, base+2, base+3]。
        for (f, quad) in mesh.indices.as_chunks::<6>().0.iter().enumerate() {
            let base = (f * 4) as u16;
            assert_eq!(
                quad,
                &[base, base + 1, base + 2, base, base + 2, base + 3],
                "第 {f} 面的三角索引模式"
            );
        }

        // 每面:4 顶点共面,法线 = 面方向单位向量(非零),色 = 方块色 × 亮度
        // (macroquad Vertex.color = RGBA8,按 From<Color> 同式量化:×255 截断)。
        let stone = block_color(STONE);
        for (face, quad) in mesh.vertices.as_chunks::<4>().0.iter().enumerate() {
            let (dir, normal) = match face {
                0 => (FaceDir::West, [-1.0, 0.0, 0.0]),
                1 => (FaceDir::East, [1.0, 0.0, 0.0]),
                2 => (FaceDir::Bottom, [0.0, -1.0, 0.0]),
                3 => (FaceDir::Top, [0.0, 1.0, 0.0]),
                4 => (FaceDir::North, [0.0, 0.0, -1.0]),
                _ => (FaceDir::South, [0.0, 0.0, 1.0]),
            };
            let b = dir.brightness();
            assert!(b > 0.0 && b <= 1.0, "{dir:?} 亮度在 (0,1](实 {b})");
            for (i, v) in quad.iter().enumerate() {
                assert!(
                    v.normal.x * normal[0] + v.normal.y * normal[1] + v.normal.z * normal[2] > 0.9,
                    "第 {face} 面第 {i} 顶点法线 = {dir:?} 方向(非零且对向)"
                );
                let dot = (stone[0] * 0.2126 + stone[1] * 0.7152 + stone[2] * 0.0722) * 0.0;
                let _ = dot; // 亮度断言在下方 alpha 不变量 + 灰度单调(实现定式后逐通道量化比对)
                let (er, eg, eb, ea) = (stone[0] * b, stone[1] * b, stone[2] * b, stone[3]);
                let quant = |x: f32| (x * 255.0) as u8;
                let got = v.color;
                assert_eq!(
                    (got[0], got[1], got[2], got[3]),
                    (quant(er), quant(eg), quant(eb), quant(ea)),
                    "第 {face} 面第 {i} 顶点色 = 方块色 × 亮度(RGBA8 量化)"
                );
            }
        }

        // 世界坐标:块世界包围盒 = 原点 + (2..3, 100..101, 3..4)。
        let (ox, oy, oz) = origin;
        let min = [ox + 2.0, oy + 100.0, oz + 3.0];
        let max = [min[0] + 1.0, min[1] + 1.0, min[2] + 1.0];
        for v in &mesh.vertices {
            assert!(
                v.position.x >= min[0] - 1e-4
                    && v.position.x <= max[0] + 1e-4
                    && v.position.y >= min[1] - 1e-4
                    && v.position.y <= max[1] + 1e-4
                    && v.position.z >= min[2] - 1e-4
                    && v.position.z <= max[2] + 1e-4,
                "顶点位置 {:#?} 越出块包围盒",
                v.position
            );
        }
        // 顶点数随暴露面数变化(超平坦 = 768 面 → 3072 顶点,宏观口径)。
        let flat = superflat();
        let mesh_flat = chunk_mesh(&flat, (0.0, 0.0, 0.0));
        assert_eq!(
            mesh_flat.vertices.len(),
            exposed_faces(&flat).len() * 4,
            "超平坦顶点数 = 暴露面 × 4"
        );
        let _ = (CHUNK_X, CHUNK_Y, CHUNK_Z); // 尺寸常数在场(chunk.rs §坐标)
    }
}
