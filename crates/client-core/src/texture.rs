//! 程序化方块细节纹理图集、天空色与距离雾参数(REQ-013 C11)。
//!
//! 纯函数、零渲染依赖、零外部随机源:8 个 16×16 灰度细节 tile 横排为 128×16 RGBA8 图集,
//! 与既有顶点色(`block_color × 方向亮度 × AO`)相乘得到最终像素颜色。

use types::BlockId;

/// 浅蓝天空清屏色与雾色(sRGB)。
pub const SKY: [f32; 3] = [0.47, 0.65, 1.0];

const ATLAS_WIDTH: u32 = 128;
const ATLAS_HEIGHT: u32 = 16;
const TILE_SIZE: u32 = 16;
const TILE_COUNT: u32 = 8;

const NOISE: [i32; 16] = [-4, 2, -1, 3, 0, -3, 1, -2, 4, -1, 2, -4, 3, 0, -2, 1];

/// 程序化生成的 RGBA8 图集原始像素数据。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AtlasImage {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}

fn tile_amp(tile: u32) -> f32 {
    match tile {
        1 => 1.0,
        2 => 1.5,
        3 => 2.5,
        4 => 0.75,
        _ => 0.0,
    }
}

/// 生成 128×16 RGBA8 灰度细节图集(8 个 16×16 tile 横排)。
pub fn atlas_image() -> AtlasImage {
    let mut rgba = vec![0u8; (ATLAS_WIDTH * ATLAS_HEIGHT * 4) as usize];
    for t in 0..TILE_COUNT {
        let amp = tile_amp(t);
        for y in 0..TILE_SIZE {
            for x in 0..TILE_SIZE {
                let idx = ((x * 5 + y * 11 + t * 3) % 16) as usize;
                let g = 1.0 + NOISE[idx] as f32 * amp / 64.0;
                let byte = (g.clamp(0.0, 1.0) * 255.0).round() as u8;
                let offset = ((y * ATLAS_WIDTH + (t * TILE_SIZE + x)) * 4) as usize;
                rgba[offset] = byte;
                rgba[offset + 1] = byte;
                rgba[offset + 2] = byte;
                rgba[offset + 3] = 255;
            }
        }
    }
    AtlasImage {
        width: ATLAS_WIDTH,
        height: ATLAS_HEIGHT,
        rgba,
    }
}

/// 方块 ID 对应的 tile 索引:`id.0 < 8` 时取 `id.0`,否则回退到 tile 7(纯白)。
pub fn tile_of(id: BlockId) -> u32 {
    let raw = u32::from(id.0);
    if raw < TILE_COUNT { raw } else { 7 }
}

/// 指定方块 tile 左上角在图集中的起始 UV。
pub fn tile_uv(id: BlockId) -> [f32; 2] {
    [(TILE_SIZE * tile_of(id)) as f32 / ATLAS_WIDTH as f32, 0.0]
}

/// 面角点 `(cu, cv) ∈ {0, 1}²` 在图集中的 UV(半纹素内缩防渗色,v 轴翻转使方块顶对应 tile 顶)。
pub fn face_uv(id: BlockId, cu: u32, cv: u32) -> [f32; 2] {
    let t = tile_of(id);
    let s = ((TILE_SIZE * t) as f32 + 0.5 + 15.0 * cu as f32) / ATLAS_WIDTH as f32;
    let v = (0.5 + 15.0 * (1.0 - cv as f32)) / ATLAS_HEIGHT as f32;
    [s, v]
}

/// 视距(chunk 数)对应的线性距离雾起止距离 `(start, end)`(米)。
pub fn fog_range(view_distance: u32) -> (f32, f32) {
    let r = (view_distance * 16) as f32;
    (0.6 * r, 0.95 * r)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn t1_atlas_dimensions_and_determinism() {
        let a = atlas_image();
        let b = atlas_image();
        assert_eq!(a.width, 128);
        assert_eq!(a.height, 16);
        assert_eq!(a.rgba.len(), 8192);
        assert_eq!(a.rgba, b.rgba);
    }

    #[test]
    fn t2_atlas_exact_pixel_anchors() {
        let atlas = atlas_image();
        let cases: [(u32, u32, u32, u8); 10] = [
            (2, 0, 0, 255),
            (2, 1, 0, 231),
            (2, 0, 1, 255),
            (1, 3, 2, 255),
            (1, 2, 3, 247),
            (3, 1, 1, 245),
            (3, 1, 0, 235),
            (4, 0, 1, 249),
            (7, 5, 5, 255),
            (0, 0, 0, 255),
        ];
        for (t, x, y, expected) in cases {
            let off = ((y * 128) + (t * 16 + x)) as usize * 4;
            assert_eq!(
                &atlas.rgba[off..off + 4],
                &[expected, expected, expected, 255],
                "tile={t} ({x},{y}) 像素不匹配"
            );
        }
    }

    #[test]
    fn t3_tile_of_mapping_and_fallback() {
        assert_eq!(tile_of(BlockId(2)), 2);
        assert_eq!(tile_of(BlockId(7)), 7);
        assert_eq!(tile_of(BlockId(8)), 7);
        assert_eq!(tile_of(BlockId(99)), 7);
        assert_eq!(tile_of(BlockId::AIR), 0);
    }

    #[test]
    fn t4_fog_range_and_sky_anchors() {
        let (s3, e3) = fog_range(3);
        assert!((s3 - 28.8).abs() <= 1e-4, "s3={s3}");
        assert!((e3 - 45.6).abs() <= 1e-4, "e3={e3}");

        assert_eq!(fog_range(0), (0.0, 0.0));

        let (s8, e8) = fog_range(8);
        assert!((s8 - 76.8).abs() <= 1e-4, "s8={s8}");
        assert!((e8 - 121.6).abs() <= 1e-4, "e8={e8}");

        assert_eq!(SKY[0], 0.47);
        assert_eq!(SKY[1], 0.65);
        assert_eq!(SKY[2], 1.0);
    }
}
