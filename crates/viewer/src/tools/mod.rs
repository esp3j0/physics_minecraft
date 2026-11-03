//! tools —— 交互模式调试工具(REQ-011 §3):工具状态、放置/拆除动作、
//! 拒绝提示计时、帮助表。全部纯逻辑,零 macroquad;render.rs 只做按键到
//! 纯函数的接线。
//!
//! 单元范围(REQ-011 §6 W01):材料 1–4、左键拆除、右键放置(单方块 /
//! 预制结构一批)、T 循环放置物、H 帮助;拾取([`pick`])、线框高亮、
//! 轮廓预览。发射参数与蓄力(W02)、调试面板(W04)、布景(W05)只在
//! [`ToolState`] 留字段。

mod actions;
mod pick;
mod prefab;

pub use actions::{PLACEABLE_CYCLE, ToolAction};
pub use actions::{dig, place, select_material};
pub use pick::{Hit, pick};
pub use prefab::{ExpandError, Orientation, Prefab, expand, orientation_from_yaw};

use types::{BlockId, BlockPos};
use voxel::builtin;

/// 工具状态(REQ-011 §3.1;交互模式独有)。
#[derive(Debug, Clone, PartialEq)]
pub struct ToolState {
    /// 当前材料(1–4 切;单方块放置用)。
    pub material: Material,
    /// 当前放置物(单方块或预制结构;T 循环)。
    pub placeable: Placeable,
    /// 当前发射形状(W02 消费;本单元只存默认值)。
    pub launch_shape: LaunchShape,
    /// 预设发射速度 m/s(滚轮 ±5,夹 5–55;W02 消费)。
    pub preset_speed: f32,
    /// 按键帮助显示(H 切换)。
    pub help_shown: bool,
    /// 拒绝提示(消息 + 剩余秒数;逐帧 dt 累减,3 秒过期)。
    pub notice: Option<(String, f32)>,
}

/// 当前材料(REQ-011 §3.1:土 / 石 / 沙 / 基岩)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Material {
    Dirt,
    Stone,
    Sand,
    Bedrock,
}

impl Material {
    /// `1`–`4` 的序(0 起)。
    #[must_use]
    pub fn from_index(i: usize) -> Self {
        [Self::Dirt, Self::Stone, Self::Sand, Self::Bedrock][i % 4]
    }

    /// 对应 BlockId(voxel::builtin;ID 只追加,与服务器同表)。
    #[must_use]
    pub fn block(self) -> BlockId {
        match self {
            Self::Dirt => builtin::DIRT,
            Self::Stone => builtin::STONE,
            Self::Sand => builtin::SAND,
            Self::Bedrock => builtin::BEDROCK,
        }
    }

    /// 中文名(HUD / 提示)。
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Dirt => "土",
            Self::Stone => "石",
            Self::Sand => "沙",
            Self::Bedrock => "基岩",
        }
    }
}

/// 当前放置物(REQ-011 §3.1 T 循环:单方块 → 土墙 → 石柱 → 石桥 →
/// 土门楣门框 → 石 7 字 → 单方块;序见 [`actions::PLACEABLE_CYCLE`])。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Placeable {
    SingleBlock,
    Prefab(Prefab),
}

/// 发射形状(W02 消费;REQ-011 §3.2 表,E 循环。本单元只存字段)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaunchShape {
    Single,
    BeamLongitudinal,
    BeamCross,
    Plate2x2,
    Block3x3x3,
}

/// 拒绝提示显示时长(秒;REQ-011 §3.5.3 第 5 条)。
pub const NOTICE_SECS: f32 = 3.0;

impl Default for ToolState {
    fn default() -> Self {
        Self {
            material: Material::Dirt,
            placeable: Placeable::SingleBlock,
            launch_shape: LaunchShape::Single,
            preset_speed: 20.0,
            help_shown: false,
            notice: None,
        }
    }
}

impl ToolState {
    /// 推进拒绝提示计时(每渲染帧调用;dt 秒,墙钟不进模拟)。
    pub fn tick(&mut self, dt: f32) {
        if let Some((_, remain)) = &mut self.notice {
            *remain -= dt;
            if *remain <= 0.0 {
                self.notice = None;
            }
        }
    }

    /// 显示拒绝提示(覆盖旧提示,重置 3 秒)。
    pub fn reject(&mut self, message: impl Into<String>) {
        self.notice = Some((message.into(), NOTICE_SECS));
    }
}

/// `T`:切换放置物(沿 [`PLACEABLE_CYCLE`] 表循环,§3.1)。
pub fn cycle_placeable(state: &mut ToolState) {
    let cur = state.placeable;
    let idx = PLACEABLE_CYCLE.iter().position(|p| *p == cur).unwrap_or(0); // 不在表内(不可能)回单方块,自守
    let next = (idx + 1) % PLACEABLE_CYCLE.len();
    state.placeable = PLACEABLE_CYCLE[next];
}

/// `H`:切换帮助。
pub fn toggle_help(state: &mut ToolState) {
    state.help_shown = !state.help_shown;
}

/// 帮助表(REQ-011 §3.1 全按键;H 显示。本单元未接的键注明所属单元)。
#[must_use]
pub fn help_lines() -> Vec<&'static str> {
    vec![
        "WASD+鼠标 飞行  Shift/Ctrl 升降  Space 相机耦合  M 释放/锁定鼠标",
        "1-4 材料(土/石/沙/基岩)  左键 拆除  右键 放置(单方块/预制结构)",
        "T 切换放置物  H 帮助  F 发射/蓄力(W02)  E 发射形状(W02)",
        "滚轮 发射速度(W02)  F3 调试面板(W04)  C 计数清零(W04)",
        "R 重建布景 / K 播放动作(W05)  B 生成立方体  X 爆炸",
    ]
}

/// HUD 工具行(§3.5.3 第 1 条的临时版,W04 并入调试面板):材料、放置物、
/// 目标格坐标与方块名(无目标 "—")。
#[must_use]
pub fn hud_tool_line(
    state: &ToolState,
    target: Option<&Hit>,
    world: &dyn crate::script::WorldQuery,
) -> String {
    let placeable = match state.placeable {
        Placeable::SingleBlock => "单方块".to_owned(),
        Placeable::Prefab(p) => p.name().to_owned(),
    };
    let target = match target {
        Some(hit) => {
            let name = world
                .block_at(hit.block_pos)
                .and_then(block_name)
                .unwrap_or_else(|| "?".to_owned());
            format!(
                "({},{},{}) {}",
                hit.block_pos.x, hit.block_pos.y, hit.block_pos.z, name
            )
        }
        None => "—".to_owned(),
    };
    format!(
        "工具:材料 {} | 放置物 {} | 目标 {}",
        state.material.name(),
        placeable,
        target
    )
}

/// 方块名(交互显示用;与 voxel::builtin 注册表同名)。
fn block_name(b: BlockId) -> Option<String> {
    match b {
        builtin::AIR => Some("air".to_owned()),
        builtin::STONE => Some("stone".to_owned()),
        builtin::DIRT => Some("dirt".to_owned()),
        builtin::BEDROCK => Some("bedrock".to_owned()),
        builtin::SAND => Some("sand".to_owned()),
        _ => None,
    }
}

/// 预制结构轮廓格(轮廓预览与实际放置**同一展开函数** [`expand`];§3.4)。
/// 返回 (格列表, 是否越界):越界时整构拒绝放置,但预览仍画出警示轮廓——
/// 约定把锚点 y 夹回世界范围内展开(画在哪总比没有强),警示色由 render 定。
#[must_use]
pub fn preview_outline(
    prefab: Prefab,
    anchor: BlockPos,
    orientation: Orientation,
) -> (Vec<(BlockPos, BlockId)>, bool) {
    match expand(
        prefab,
        anchor,
        orientation,
        types::WORLD_Y_MIN,
        types::WORLD_Y_MAX,
    ) {
        Ok(cells) => (cells, false),
        Err(_) => {
            let clamped = anchor
                .y
                .clamp(types::WORLD_Y_MIN + 1, types::WORLD_Y_MAX - 4);
            let cells = expand(
                prefab,
                BlockPos::new(anchor.x, clamped, anchor.z),
                orientation,
                types::WORLD_Y_MIN,
                types::WORLD_Y_MAX,
            )
            .unwrap_or_default();
            (cells, true)
        }
    }
}

/// 拒绝提示文案(§3.5.3 第 5 条的原因;集中在此)。
pub mod notices {
    pub const NO_TARGET: &str = "无目标";
    pub const OUT_OF_WORLD: &str = "结构越界,未放置";
    pub const BEDROCK_LAUNCH: &str = "基岩不可发射(W02)";
}

#[cfg(test)]
#[path = "tools_tests.rs"]
mod tests;
