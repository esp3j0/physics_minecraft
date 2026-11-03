//! tools 层测试(REQ-011 W01;AC-W11 前身——拒绝提示计时、帮助、HUD 行、
//! 预览与展开同源)。拾取 / 预制 / 动作的用例在各子模块内。

use types::BlockPos;
use types::glam::Vec3;

use super::*;
use crate::mesh::FaceDir;
use crate::script::WorldQuery;

/// HUD 行用最小假世界(一个 chunk,顶层一格 dirt)。
struct OneChunkWorld {
    block: Option<(BlockPos, BlockId)>,
}

impl OneChunkWorld {
    fn empty() -> Self {
        Self { block: None }
    }
}

impl WorldQuery for OneChunkWorld {
    fn live_entities(&self) -> Vec<(types::EntityId, Vec3)> {
        vec![]
    }

    fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
        match self.block {
            Some((p, b)) if p == pos => Some(b),
            _ => Some(BlockId::AIR), // 其余格当空气(不参与拾取测试)
        }
    }

    fn chunks_loaded(&self) -> usize {
        1
    }
}

/// 拒绝提示:tick 递减、3 秒过期、覆盖重置(AC-W11 的 3 秒过期前身)。
/// 整秒递减(浮点精确)钉过期点;0.1 帧序列验证"逐帧累加"路径。
#[test]
fn notice_expires_after_three_seconds() {
    let mut state = ToolState::default();
    assert!(state.notice.is_none(), "默认无提示");

    state.reject("结构越界");
    assert!(state.notice.is_some(), "拒绝即显示");
    state.tick(1.0);
    state.tick(1.0);
    assert!(state.notice.is_some(), "2 s 仍在");
    state.tick(1.0);
    assert!(state.notice.is_none(), "3 s 到点过期");

    // 逐帧 0.1 s:2.9 s(29 帧)仍在;累计误差下过期点允许 ±1 帧,再
    // 5 帧内必过期。
    state.reject("无目标");
    for _ in 0..29 {
        state.tick(0.1);
    }
    assert!(state.notice.is_some(), "≈2.9 s 仍在");
    for _ in 0..5 {
        state.tick(0.1);
    }
    assert!(state.notice.is_none(), "3 s ± 误差内必过期");

    // 覆盖:旧提示未到期,新提示顶掉旧的并重置时长。
    state.reject("无目标");
    state.tick(2.0);
    state.reject("基岩不可发射");
    assert_eq!(
        state.notice.as_ref().map(|(m, _)| m.as_str()),
        Some("基岩不可发射")
    );
    state.tick(2.0);
    assert!(state.notice.is_some(), "重置后又可亮 2 s");
    state.tick(1.0);
    assert!(state.notice.is_none());
}

/// H:帮助切换来回。
#[test]
fn toggle_help_flips() {
    let mut state = ToolState::default();
    assert!(!state.help_shown, "默认隐藏");
    toggle_help(&mut state);
    assert!(state.help_shown);
    toggle_help(&mut state);
    assert!(!state.help_shown);
}

/// 帮助表列全 §3.1 按键(ASCII 英文;W02 起已接键无 todo,未接键标
/// (todo) + 单元)。
#[test]
fn help_lines_cover_all_keys() {
    let joined = help_lines().join("\n");
    for key in [
        "1-4", "LMB", "RMB", "T ", "F ", "E ", "wheel", "F3", "C ", "R ", "K ", "H ", "B ", "X ",
    ] {
        assert!(joined.contains(key), "帮助表缺按键 {key}");
    }
    // W02 后 F/E/wheel 已接,不得再带 todo;仍未接的 4 键(F3/C/R/K)
    // 带 (todo(3 处标记覆盖 4 键,R/K 同行)。
    let todo_count = joined.matches("(todo").count();
    assert_eq!(todo_count, 3, "W02 后仍带 (todo 的标记数");
    for (key, line) in [("F ", 2), ("E ", 2), ("wheel", 2)] {
        let l = help_lines()[line];
        assert!(
            l.contains(key) && !l.contains("(todo"),
            "已接键 {key} 不带 todo:{l}"
        );
    }
    for (key, line) in [("F3", 3), ("C ", 3), ("R ", 4), ("K ", 4)] {
        let l = help_lines()[line];
        assert!(
            l.contains(key) && l.contains("(todo"),
            "未接键 {key} 须带 todo:{l}"
        );
    }
}

/// HUD 工具行:材料 / 放置物 / 目标(坐标 + 方块名),无目标 "-"(ASCII)。
#[test]
fn hud_tool_line_shows_material_placeable_target() {
    let world = OneChunkWorld::empty();
    let mut state = ToolState::default();

    // 无目标。
    let line = hud_tool_line(&state, None, &world);
    assert!(line.contains("material dirt"), "{line}");
    assert!(line.contains("placeable single block"), "{line}");
    assert!(line.contains("target -"), "{line}");

    // 有目标:命中格 (5,−61,7) dirt(世界返回 AIR 只在非命中格——让
    // block_at 对命中格返回 dirt)。
    let mut world = OneChunkWorld::empty();
    world.block = Some((BlockPos::new(5, -61, 7), voxel::builtin::DIRT));
    let hit = Hit {
        block_pos: BlockPos::new(5, -61, 7),
        block: voxel::builtin::DIRT,
        face: FaceDir::Top,
    };
    let line = hud_tool_line(&state, Some(&hit), &world);
    assert!(line.contains("(5,-61,7) dirt"), "{line}");

    // 换材料与预制结构后行文案跟随。
    state.material = Material::Stone;
    state.placeable = Placeable::Prefab(Prefab::StoneBridge);
    let line = hud_tool_line(&state, None, &world);
    assert!(line.contains("material stone"), "{line}");
    assert!(line.contains("placeable stone bridge"), "{line}");
}

/// 轮廓预览与实际展开同一函数:格集合逐格相同(AC-W5"断言二者格集合相同")。
#[test]
fn preview_outline_matches_expand() {
    let anchor = BlockPos::new(40, -50, -12);
    for prefab in [
        Prefab::DirtWall,
        Prefab::StonePillar,
        Prefab::StoneBridge,
        Prefab::DirtLintel,
        Prefab::StoneCantilever,
    ] {
        let o = orientation_from_yaw(30.0);
        let (outline, out_of_world) = preview_outline(prefab, anchor, o);
        assert!(!out_of_world, "界内锚点不应警示");
        let direct = expand(prefab, anchor, o, types::WORLD_Y_MIN, types::WORLD_Y_MAX)
            .expect("同参展开必成");
        assert_eq!(outline, direct, "{prefab:?} 预览与展开同源");
    }
}

/// 轮廓预览越界:警示标记 + 夹回后仍有完整轮廓(警示色由 render 用)。
#[test]
fn preview_outline_flags_out_of_world() {
    let anchor = BlockPos::new(0, types::WORLD_Y_MAX - 2, 0); // 石柱顶 2 格越界
    let (outline, out_of_world) =
        preview_outline(Prefab::StonePillar, anchor, orientation_from_yaw(0.0));
    assert!(out_of_world, "越界要置警示");
    assert_eq!(outline.len(), 4, "夹回后轮廓仍完整(画在夹回位置)");

    // 土墙墙脚越界(锚点 y = WORLD_Y_MIN):同样警示。
    let anchor = BlockPos::new(0, types::WORLD_Y_MIN, 0);
    let (_, out_of_world) = preview_outline(Prefab::DirtWall, anchor, orientation_from_yaw(0.0));
    assert!(out_of_world, "墙脚低于世界底 → 警示");
}

/// 默认工具状态(§3.1:材料土、放置物单方块;W02 字段占位:单格 / 20 m/s)。
#[test]
fn default_tool_state_matches_spec() {
    let state = ToolState::default();
    assert_eq!(state.material, Material::Dirt);
    assert_eq!(state.placeable, Placeable::SingleBlock);
    assert_eq!(state.launch_shape, LaunchShape::Single);
    assert_eq!(state.preset_speed, 20.0);
    assert!(!state.help_shown);
    assert!(state.notice.is_none());
}
