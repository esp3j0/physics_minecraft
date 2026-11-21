//! prefab —— 局部轴取整与预制结构展开(REQ-011 §3.2 / §3.4)。
//!
//! 纯函数,零 macroquad。约定:
//! - 局部轴:u = 右、v = 上、w = 前;结构表用局部坐标,锚点 = (0,0,0);
//! - **yaw 约定**(沿 viewer 相机,`CameraPose::forward()` 同源):
//!   `forward = (cos yaw · cos pitch, sin pitch, sin yaw · cos pitch)`,
//!   即 yaw = 0° 朝 +x、yaw 增大逆时针(俯视 +x → +z);水平前向
//!   `(cos yaw, 0, sin yaw)`;
//! - **取整**(AC-W2):`w` = 相机水平朝向取整到最近的 ±x / ±z;`u` = `w`
//!   绕 y **右转** 90°(从上往下看顺时针:w = +x → u = +z;w = +z →
//!   u = −x;w = −x → u = −z;w = −z → u = +x);`v` 恒 = +y。
//!   平局(45° 边界)归到逆时针一侧:|cos yaw| ≥ |sin yaw| 时取 ±x 档
//!   (与 `pick` 平局先 x 同风格;测试用具体数值钉死)。
//!   正上 / 正下看(pitch = ±90°)水平朝向仍按 yaw 取,取整稳定。

use types::{BlockId, BlockPos};

/// 朝向:u / w 的世界方向(整数单位向量;v 恒 +y 不在场)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Orientation {
    /// 右方向(u)。
    pub u: (i32, i32),
    /// 前方向(w)。
    pub w: (i32, i32),
}

/// yaw(度)→ u / w 取整(AC-W2;见模块文档的约定与平局规则)。
#[must_use]
pub fn orientation_from_yaw(yaw_deg: f32) -> Orientation {
    // 水平前向(俯仰不参与);|cos| ≥ |sin| → ±x 档,否则 ±z 档。
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let w = if cy.abs() >= sy.abs() {
        (cy.signum() as i32, 0)
    } else {
        (0, sy.signum() as i32)
    };
    // u = w 右转 90°(俯视顺时针):(wx, wz) → (−wz, wx)。
    let u = (-w.1, w.0);
    Orientation { u, w }
}

/// 一个预制结构(REQ-011 §3.4 表;局部坐标 u 右 / v 上 / w 前,锚点 = 原点)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Prefab {
    /// 土墙:土 u=−3..3、v=0..4、w=3;基岩 u=−3..3、v=−1、w=3(墙脚)。
    DirtWall,
    /// 石柱:石 u=0、v=0..3、w=3。
    StonePillar,
    /// 石桥:两柱 u=±3、v=0..2 + 桥面 u=−4..4、v=3(w=3,全石)。
    StoneBridge,
    /// 土门楣门框:石柱 u=±3、v=0..2 + 土门楣 u=−3..3、v=3(w=3)。
    DirtLintel,
    /// 石 7 字悬臂:石 u=0、v=0..3 + 石 u=1..4、v=3(w=3)。
    StoneCantilever,
}

impl Prefab {
    /// `T` 循环顺序(REQ-011 §3.1:单方块 → 土墙 → 石柱 → 石桥 → 土门楣门框
    /// → 石 7 字 → 单方块;循环在放置物层做,这里给"下一个预制结构")。
    pub fn next(self) -> Self {
        match self {
            Self::DirtWall => Self::StonePillar,
            Self::StonePillar => Self::StoneBridge,
            Self::StoneBridge => Self::DirtLintel,
            Self::DirtLintel => Self::StoneCantilever,
            Self::StoneCantilever => Self::DirtWall,
        }
    }

    /// 英文名(HUD / 帮助 / 拒绝提示;macroquad 位图字体不含 CJK)。
    pub fn name(self) -> &'static str {
        match self {
            Self::DirtWall => "dirt wall",
            Self::StonePillar => "stone pillar",
            Self::StoneBridge => "stone bridge",
            Self::DirtLintel => "dirt lintel",
            Self::StoneCantilever => "stone 7-shape",
        }
    }
}

/// 展开失败原因(REQ-011 §3.4:任何一格越出世界 y 范围 → 整个结构不发)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExpandError {
    /// 最低格低于世界底(`y_min` = 越界的格 y)。
    BelowWorld { y_min: i32 },
    /// 最高格高于世界顶(`y_max` = 越界的格 y)。
    AboveWorld { y_max: i32 },
}

/// 预制结构展开:锚点 + 朝向 → (格, 方块) 列表,按 `BlockPos` 升序
/// (REQ-011 §3.4;AC-W5)。轮廓预览与实际放置调用**同一个函数**。
pub fn expand(
    prefab: Prefab,
    anchor: BlockPos,
    orientation: Orientation,
    y_min: i32,
    y_max: i32,
) -> Result<Vec<(BlockPos, BlockId)>, ExpandError> {
    let (du, dv) = orientation.u;
    let (dw_x, dw_z) = orientation.w;
    // 局部 (u, v, w) → 世界:pos = anchor + u·û + v·ŷ + w·ŵ。
    let mut cells: Vec<(BlockPos, BlockId)> = Vec::new();
    let mut y_min_seen = i32::MAX;
    let mut y_max_seen = i32::MIN;
    let mut push = |u: i32, v: i32, w: i32, block: BlockId| {
        let pos = BlockPos::new(
            anchor.x + u * du + w * dw_x,
            anchor.y + v,
            anchor.z + u * dv + w * dw_z,
        );
        y_min_seen = y_min_seen.min(pos.y);
        y_max_seen = y_max_seen.max(pos.y);
        cells.push((pos, block));
    };

    let dirt = voxel::builtin::DIRT;
    let stone = voxel::builtin::STONE;
    let bedrock = voxel::builtin::BEDROCK;
    let w3 = 3; // 结构离锚点 3 格(§3.4 表)
    match prefab {
        Prefab::DirtWall => {
            for u in -3..=3 {
                for v in 0..=4 {
                    push(u, v, w3, dirt);
                }
                push(u, -1, w3, bedrock); // 墙脚(替换地面一行)
            }
        }
        Prefab::StonePillar => {
            for v in 0..=3 {
                push(0, v, w3, stone);
            }
        }
        Prefab::StoneBridge => {
            for u in [-3, 3] {
                for v in 0..=2 {
                    push(u, v, w3, stone);
                }
            }
            for u in -4..=4 {
                push(u, 3, w3, stone);
            }
        }
        Prefab::DirtLintel => {
            for u in [-3, 3] {
                for v in 0..=2 {
                    push(u, v, w3, stone);
                }
            }
            for u in -3..=3 {
                push(u, 3, w3, dirt);
            }
        }
        Prefab::StoneCantilever => {
            for v in 0..=3 {
                push(0, v, w3, stone);
            }
            for u in 1..=4 {
                push(u, 3, w3, stone);
            }
        }
    }

    // 越界检查:任何一格在世界 y 范围外 → 整构拒绝。
    if y_min_seen < y_min {
        return Err(ExpandError::BelowWorld { y_min: y_min_seen });
    }
    if y_max_seen > y_max {
        return Err(ExpandError::AboveWorld { y_max: y_max_seen });
    }
    cells.sort_by_key(|(pos, _)| *pos);
    Ok(cells)
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::WORLD_Y_MAX;
    use voxel::builtin;

    const Y_MIN: i32 = types::WORLD_Y_MIN;
    const Y_MAX: i32 = WORLD_Y_MAX;

    /// AC-W2:八个象限边界两侧各一点钉死取整(数值锚定,注释给手算)。
    #[test]
    fn orientation_rounds_eight_sectors() {
        // 边界两侧点:±0°(取 ±x 边界,cos 赢)、±45°(平局归 ±x)、±46°、
        // ±90°(±z 边界)、±134°、±135°(平局)、±136°、±180°。
        // yaw=0° 朝 +x;yaw=90° 朝 +z(yaw 增大逆时针,俯视 +x→+z)。
        let cases: [(f32, (i32, i32)); 12] = [
            (0.0, (1, 0)),    // 朝 +x
            (44.0, (1, 0)),   // cos > sin → +x
            (45.0, (1, 0)),   // 平局归 ±x(cos 档)
            (46.0, (0, 1)),   // sin > cos → +z
            (90.0, (0, 1)),   // 朝 +z
            (134.0, (0, 1)),  // |sin| > |cos| → +z
            (135.0, (-1, 0)), // 平局归 ±x 档;cos 为负 → −x
            (136.0, (-1, 0)), // −x
            (180.0, (-1, 0)), // 朝 −x
            (-44.0, (1, 0)),  // 朝 −z 半平面之前仍是 +x
            (-45.0, (1, 0)),  // 平局 → +x
            (-46.0, (0, -1)), // 朝 −z
        ];
        for (yaw, want_w) in cases {
            let o = orientation_from_yaw(yaw);
            assert_eq!(o.w, want_w, "yaw = {yaw}° 的 w 取整");
            // u = w 右转 90°(俯视顺时针):+x→+z、+z→−x、−x→−z、−z→+x。
            let want_u = (-want_w.1, want_w.0);
            assert_eq!(o.u, want_u, "yaw = {yaw}° 的 u = w 右转 90°");
        }
    }

    /// AC-W2 正上 / 正下看:水平朝向只按 yaw 取,俯仰不影响(pitch 不进参
    /// 数表,函数天然稳定;此处钉 90° 俯仰下 yaw ±45° 平局仍归 ±x)。
    #[test]
    fn orientation_independent_of_pitch() {
        // orientation_from_yaw 只吃 yaw;正上/正下时同一 yaw 仍得同一取整。
        assert_eq!(orientation_from_yaw(45.0).w, (1, 0));
        assert_eq!(orientation_from_yaw(-45.0).w, (1, 0));
        assert_eq!(orientation_from_yaw(45.0).w, orientation_from_yaw(405.0).w);
    }

    /// AC-W5:五个结构在 yaw = 0°(u=+z, w=+x)的展开格表,与 §3.4 表逐格
    /// 一致(含土墙基岩墙脚);回评附此表。
    #[test]
    fn expand_matches_table_yaw0() {
        let anchor = BlockPos::new(100, 0, 200);
        let o = orientation_from_yaw(0.0);
        assert_eq!(o.w, (1, 0), "w = +x");
        assert_eq!(o.u, (0, 1), "u = +z");

        // 土墙:42 格;土 (100..106? 不——u=−3..3 沿 +z,w=3 沿 +x)。
        let wall = expand(Prefab::DirtWall, anchor, o, Y_MIN, Y_MAX).expect("土墙在界内");
        assert_eq!(wall.len(), 42, "土墙 35 土 + 7 基岩");
        let dirt = builtin::DIRT;
        let bedrock = builtin::BEDROCK;
        let mut want = Vec::new();
        for u in -3..=3 {
            for v in 0..=4 {
                want.push((BlockPos::new(103, v, 200 + u), dirt));
            }
            want.push((BlockPos::new(103, -1, 200 + u), bedrock));
        }
        want.sort_by_key(|(p, _)| *p);
        assert_eq!(
            wall, want,
            "土墙逐格(yaw=0°:w=+x 列在 x=103,u=+z 沿 z 展开)"
        );

        // 石柱:4 格。
        let pillar = expand(Prefab::StonePillar, anchor, o, Y_MIN, Y_MAX).expect("石柱在界内");
        assert_eq!(pillar.len(), 4);
        assert_eq!(
            pillar,
            vec![
                (BlockPos::new(103, 0, 200), builtin::STONE),
                (BlockPos::new(103, 1, 200), builtin::STONE),
                (BlockPos::new(103, 2, 200), builtin::STONE),
                (BlockPos::new(103, 3, 200), builtin::STONE),
            ],
            "石柱:x=103(w=3),z=200(u=0),v=0..3"
        );

        // 石桥:15 格 = 2 柱 ×3 + 桥面 9。
        let bridge = expand(Prefab::StoneBridge, anchor, o, Y_MIN, Y_MAX).expect("石桥在界内");
        assert_eq!(bridge.len(), 15);
        for u in [-3, 3] {
            for v in 0..=2 {
                assert!(
                    bridge.contains(&(BlockPos::new(103, v, 200 + u), builtin::STONE)),
                    "柱格 u={u} v={v}"
                );
            }
        }
        for u in -4..=4 {
            assert!(
                bridge.contains(&(BlockPos::new(103, 3, 200 + u), builtin::STONE)),
                "桥面格 u={u}"
            );
        }

        // 土门楣门框:13 格 = 2 石柱 ×3 + 土门楣 7。
        let lintel = expand(Prefab::DirtLintel, anchor, o, Y_MIN, Y_MAX).expect("门框在界内");
        assert_eq!(lintel.len(), 13);
        let stone_count = lintel.iter().filter(|(_, b)| *b == builtin::STONE).count();
        let dirt_count = lintel.iter().filter(|(_, b)| *b == dirt).count();
        assert_eq!(stone_count, 6, "两柱石 6 格");
        assert_eq!(dirt_count, 7, "门楣土 7 格(u=−3..3,v=3)");
        for u in -3..=3 {
            assert!(
                lintel.contains(&(BlockPos::new(103, 3, 200 + u), dirt)),
                "门楣格 u={u} 是土"
            );
        }

        // 石 7 字:8 格 = 立柱 4 + 悬臂 4。
        let cant = expand(Prefab::StoneCantilever, anchor, o, Y_MIN, Y_MAX).expect("7 字在界内");
        assert_eq!(cant.len(), 8);
        assert!(
            cant.contains(&(BlockPos::new(103, 3, 200), builtin::STONE)),
            "拐角格"
        );
        for u in 1..=4 {
            assert!(
                cant.contains(&(BlockPos::new(103, 3, 200 + u), builtin::STONE)),
                "悬臂格 u={u}"
            );
        }
    }

    /// AC-W5:四个朝向 × 一个结构——同一局部格映射到正确的世界格(以石桥
    /// 面向桥面中点抽查两格;四朝向 = yaw 0/90/180/270°)。
    #[test]
    fn expand_respects_orientation() {
        let anchor = BlockPos::new(0, 0, 0);
        // yaw = 90°:w = +z,u = −x。桥面 u=−4..4 → x = 4..−4(反转),w=3 → z=3。
        let o = orientation_from_yaw(90.0);
        assert_eq!(o.w, (0, 1));
        assert_eq!(o.u, (-1, 0));
        let bridge = expand(Prefab::StoneBridge, anchor, o, Y_MIN, Y_MAX).expect("界内");
        assert!(
            bridge.contains(&(BlockPos::new(4, 3, 3), builtin::STONE)),
            "u=−4 → x=+4(yaw 90° 的 u=−x)"
        );
        assert!(
            bridge.contains(&(BlockPos::new(-3, 0, 3), builtin::STONE)),
            "柱格 u=+3 → x=−3"
        );

        // yaw = 180°:w = −x,u = −z。
        let o = orientation_from_yaw(180.0);
        let wall = expand(Prefab::DirtWall, anchor, o, Y_MIN, Y_MAX).expect("界内");
        assert!(
            wall.contains(&(BlockPos::new(-3, 0, 3), builtin::DIRT)),
            "w=3 → x=−3;u=0 → z=0"
        );
        assert!(
            wall.contains(&(BlockPos::new(-3, -1, -3), builtin::BEDROCK)),
            "墙脚 u=−3 → z=−3"
        );

        // yaw = -90°(= 270°):w = −z,u = +x。
        let o = orientation_from_yaw(-90.0);
        assert_eq!(o.w, (0, -1));
        assert_eq!(o.u, (1, 0));
        let pillar = expand(Prefab::StonePillar, anchor, o, Y_MIN, Y_MAX).expect("界内");
        assert_eq!(
            pillar,
            vec![
                (BlockPos::new(0, 0, -3), builtin::STONE),
                (BlockPos::new(0, 1, -3), builtin::STONE),
                (BlockPos::new(0, 2, -3), builtin::STONE),
                (BlockPos::new(0, 3, -3), builtin::STONE),
            ],
            "w=3 → z=−3;u=0 → x=0"
        );
    }

    /// AC-W5:五结构格数 ≤ 64(§3.4 表:42/4/15/13/8)。
    #[test]
    fn expand_cell_counts_within_fill_batch() {
        let anchor = BlockPos::new(0, 0, 0);
        let o = orientation_from_yaw(0.0);
        for (prefab, want) in [
            (Prefab::DirtWall, 42),
            (Prefab::StonePillar, 4),
            (Prefab::StoneBridge, 15),
            (Prefab::DirtLintel, 13),
            (Prefab::StoneCantilever, 8),
        ] {
            let cells = expand(prefab, anchor, o, Y_MIN, Y_MAX).expect("界内");
            assert_eq!(cells.len(), want, "{prefab:?} 格数");
            assert!(cells.len() <= crate::script::FILL_BATCH_PER_FRAME, "≤ 64");
        }
    }

    /// AC-W5:结果按 BlockPos 升序、无重复格。
    #[test]
    fn expand_sorted_and_unique() {
        let anchor = BlockPos::new(-7, 10, 33);
        let o = orientation_from_yaw(46.0); // 任意非平凡朝向
        for prefab in [
            Prefab::DirtWall,
            Prefab::StonePillar,
            Prefab::StoneBridge,
            Prefab::DirtLintel,
            Prefab::StoneCantilever,
        ] {
            let cells = expand(prefab, anchor, o, Y_MIN, Y_MAX).expect("界内");
            let mut sorted = cells.clone();
            sorted.sort_by_key(|(p, _)| *p);
            assert_eq!(cells, sorted, "{prefab:?} 升序");
            let uniq: std::collections::BTreeSet<BlockPos> =
                cells.iter().map(|(p, _)| *p).collect();
            assert_eq!(uniq.len(), cells.len(), "{prefab:?} 无重复格");
        }
    }

    /// AC-W5 越界:任何一格在世界 y 范围外 → 整构拒绝(枚 Below/Above)。
    #[test]
    fn expand_rejects_out_of_world() {
        let anchor = BlockPos::new(0, 0, 0);
        let o = orientation_from_yaw(0.0);
        // 土墙含 v=−1 的墙脚:y_min = 0 时越界(整构拒绝,不出半截)。
        assert_eq!(
            expand(Prefab::DirtWall, anchor, o, 0, Y_MAX),
            Err(ExpandError::BelowWorld { y_min: -1 }),
            "墙脚低于世界底 → 整构拒绝"
        );
        // 石柱 v=0..3:锚点 y = Y_MAX−2 时顶格 4 格越界。
        assert_eq!(
            expand(
                Prefab::StonePillar,
                BlockPos::new(0, Y_MAX - 2, 0),
                o,
                Y_MIN,
                Y_MAX
            ),
            Err(ExpandError::AboveWorld { y_max: Y_MAX + 1 }),
            "柱顶高于世界顶 → 整构拒绝"
        );
        // 恰好贴界:通过。
        assert!(
            expand(
                Prefab::StonePillar,
                BlockPos::new(0, Y_MAX - 3, 0),
                o,
                Y_MIN,
                Y_MAX
            )
            .is_ok()
        );
        assert!(
            expand(
                Prefab::DirtWall,
                BlockPos::new(0, Y_MIN + 1, 0),
                o,
                Y_MIN,
                Y_MAX
            )
            .is_ok()
        );
    }
}
