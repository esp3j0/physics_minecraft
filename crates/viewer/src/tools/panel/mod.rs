//! panel —— 调试面板文本(REQ-011 §3.5.3;W04)。**零 macroquad**:输入
//! 工具状态、目标格、世界查询、本地实体表、最近一次 [`DebugStats`] 与基线,
//! 输出带颜色标记的文本行;render.rs 只做按键接线与绘制。全部 ASCII 英文
//! (macroquad 位图字体不含 CJK,W01-2 纪律)。
//!
//! 布局(AC-W11 测试覆盖):
//! - 工具段(取代 W01 的临时 `hud_tool_line`,面板显示时不再重复画);
//! - 实体段:本地活实体数,其中复合体数(有形状的实体);
//! - 计数段:13 项相对基线的增量(`saturating_sub` 防服务器重启负数),
//!   每行 4 项;未收到任何 DebugStats 时只显示 `waiting for server`;
//! - 最近事件段:最新 5 条、新的在上,`C/s < 1` 即断(`BREAK`,红),
//!   `C = 0` 显示 `0.00` 判断开,`s = 0` 显示 `inf`(防御,也判断);
//! - 提示行:沿用 [`ToolState::notice`] 的 3 秒过期(由调用方传入当前值)。

use sim_api::proto::{DebugCounters, FlowEventWire};
use types::Tick;

use super::{Hit, Placeable, ToolState, block_name};

/// 面板一行:(文本, 颜色标记)。颜色标记由 render.rs 映射成 macroquad 颜色;
/// 纯函数层不 import macroquad。
pub type PanelLine = (String, PanelColor);

/// 面板行颜色(§3.5.3 第 4 条:断开行醒目色;其余默认白)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PanelColor {
    /// 默认(白)。
    Normal,
    /// 断开事件行(红)。
    Break,
    /// 分段标题(黄)。
    Header,
    /// 提示行(橙)。
    Notice,
}

/// 面板状态(W04):最近一次收到的 DebugStats + 计数基线。首次收到时自动
/// 设基线;`C` 键把当前值设为基线(增量归零重新累计)。
#[derive(Debug, Default, Clone, PartialEq)]
pub struct PanelState {
    /// 最近一次收到的包(None = 尚未收到,计数段显示 waiting for server)。
    pub latest: Option<DebugPacket>,
    /// 计数基线(首次收到 = 该包;`C` 键 = 当前值)。
    pub baseline: Option<DebugPacket>,
}

/// 一次 DebugStats 的内容(tick + 13 项 + recent;从 ViewEvent::DebugStats 拆出)。
#[derive(Debug, Clone, PartialEq)]
pub struct DebugPacket {
    /// 包拍号。
    pub tick: Tick,
    /// 13 项累计。
    pub counters: DebugCounters,
    /// 最近 8 条事件(旧 → 新;服务器口径)。
    pub recent: Vec<(Tick, FlowEventWire)>,
}

impl DebugPacket {
    fn from_parts(tick: Tick, counters: DebugCounters, recent: Vec<(Tick, FlowEventWire)>) -> Self {
        Self {
            tick,
            counters,
            recent,
        }
    }

    /// 13 项的数组视图(顺序与 DebugCounters 字段声明序一致)。
    fn counters_array(&self) -> [u64; 13] {
        let c = &self.counters;
        [
            c.impact_flow_events,
            c.impact_flow_breaks,
            c.impact_flow_detached_cells,
            c.compound_flow_events,
            c.compound_flow_breaks,
            c.compound_flow_detached_cells,
            c.compound_flow_corrections,
            c.bonds_broken,
            c.compounds_spawned,
            c.compounds_solidified,
            c.cells_blasted,
            c.cells_impacted,
            c.cells_collapsed,
        ]
    }
}

impl PanelState {
    /// 收到一包:覆盖 latest;基线未设(首次)则同时设为基线。
    pub fn receive(
        &mut self,
        tick: Tick,
        counters: DebugCounters,
        recent: Vec<(Tick, FlowEventWire)>,
    ) {
        let packet = DebugPacket::from_parts(tick, counters, recent);
        if self.baseline.is_none() {
            self.baseline = Some(packet.clone());
        }
        self.latest = Some(packet);
    }

    /// `C` 键:把当前值设为基线(增量从此归零)。
    pub fn reset_baseline(&mut self) {
        if let Some(packet) = self.latest.clone() {
            self.baseline = Some(packet);
        }
    }
}

/// 13 项计数缩写(与 DebugCounters 字段声明序一一对应;§3.5.3 第 3 条:
/// 名字缩写自定,但要与 13 个字段一一对应)。
const COUNTER_LABELS: [&str; 13] = [
    "i.ev", "i.brk", "i.det", "c.ev", "c.brk", "c.det", "c.corr", "bonds", "spawn", "solid",
    "blast", "impact", "collap",
];

/// 面板文本(§3.5.3;AC-W11 纯函数主战场)。
///
/// 入参:工具状态、拾取目标、世界查询(目标格方块名)、本地实体表(活实体
/// 与有形状者)、面板状态(latest + 基线)、提示行(已有 3 秒过期机制的
/// `ToolState::notice` 当前值)。
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn panel_lines(
    tools: &ToolState,
    target: Option<&Hit>,
    world: &dyn crate::script::WorldQuery,
    panel: &PanelState,
) -> Vec<PanelLine> {
    let mut lines = Vec::new();

    // —— 1. 工具段(取代 hud_tool_line;含上一发)——
    lines.push(("-- tools --".to_owned(), PanelColor::Header));
    lines.push((tools_line(tools, target, world), PanelColor::Normal));

    // —— 2. 实体段 ——
    let live = world.live_entities();
    let compounds = live
        .iter()
        .filter(|(id, _)| world.entity_shape(*id).is_some())
        .count();
    lines.push((
        format!("entities: {} live, {} compound", live.len(), compounds),
        PanelColor::Normal,
    ));

    // —— 3. 计数段 ——
    lines.push(("-- counters --".to_owned(), PanelColor::Header));
    match (&panel.latest, &panel.baseline) {
        (Some(latest), Some(baseline)) => {
            let now = latest.counters_array();
            let base = baseline.counters_array();
            let deltas: [u64; 13] = std::array::from_fn(|i| now[i].saturating_sub(base[i]));
            // 每行 4 项;块起始下标按块序推(13 = 4+4+4+1,chunks 不带块序)。
            for (row, chunk) in deltas.chunks(4).enumerate() {
                let start = row * 4;
                let text: Vec<String> = chunk
                    .iter()
                    .enumerate()
                    .map(|(k, v)| format!("{}={}", COUNTER_LABELS[start + k], v))
                    .collect();
                lines.push((text.join(" "), PanelColor::Normal));
            }
        }
        _ => {
            lines.push(("waiting for server".to_owned(), PanelColor::Normal));
        }
    }

    // —— 4. 最近事件段(最新 5 条,新的在上)——
    lines.push(("-- events --".to_owned(), PanelColor::Header));
    if let Some(latest) = &panel.latest {
        for (tick, ev) in latest.recent.iter().rev().take(5) {
            let color = if is_break(ev) {
                PanelColor::Break
            } else {
                PanelColor::Normal
            };
            lines.push((event_line(*tick, ev), color));
        }
    }

    // —— 5. 提示行(3 秒过期由 ToolState::tick 维护,这里只透传)——
    if let Some((message, _)) = &tools.notice {
        lines.push((message.clone(), PanelColor::Notice));
    }
    lines
}

/// 工具段一行(材料、放置物、形状、预设速度、上一发、目标格)。
fn tools_line(
    tools: &ToolState,
    target: Option<&Hit>,
    world: &dyn crate::script::WorldQuery,
) -> String {
    let placeable = match tools.placeable {
        Placeable::SingleBlock => "single".to_owned(),
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
        None => "-".to_owned(),
    };
    let last = match &tools.last_shot {
        Some((shape, mat, speed)) => format!(" | last {} {} {}", shape.name(), mat, *speed as i32),
        None => String::new(),
    };
    format!(
        "mat {} | place {} | shape {} | spd {} | target {}{}",
        tools.material.name(),
        placeable,
        tools.launch_shape.name(),
        tools.preset_speed as i32,
        target,
        last
    )
}

/// 断开判定(§3.5.3 第 4 条):`C/s < 1` 即断;`C = 0` 时 `C/s` 显示 `0.00`,
/// 算断开(0 无穷大危险度);`s = 0` 时显示 `inf`(防御;0/0 也判断)。
#[must_use]
pub fn is_break(ev: &FlowEventWire) -> bool {
    ev.cut < ev.supply
}

/// `C/s` 两位小数(s = 0 → `inf`,防御)。
fn ratio(ev: &FlowEventWire) -> String {
    if ev.supply == 0.0 {
        "inf".to_owned()
    } else {
        format!("{:.2}", ev.cut / ev.supply)
    }
}

/// 一条事件行(§3.5.3 第 4 条格式;数值 1 位小数,坐标 floor 取整;ASCII)。
fn event_line(tick: Tick, ev: &FlowEventWire) -> String {
    let kind = if ev.impact { "impact" } else { "compound" };
    let verdict = if is_break(ev) { "BREAK" } else { "hold" };
    format!(
        "t={} {} s={:.1} C={:.1} C/s={} {} |S|={} @({},{},{})",
        tick.0,
        kind,
        ev.supply,
        ev.cut,
        ratio(ev),
        verdict,
        ev.detached_cells,
        ev.position.x.floor() as i32,
        ev.position.y.floor() as i32,
        ev.position.z.floor() as i32,
    )
}

/// 断连原因英文化后的 HUD 文本(交互模式;W01 遗留)。脚本模式不经此。
#[must_use]
pub fn disconnect_text(reason: &str) -> String {
    format!(
        "disconnected: {} (window will close)",
        crate::client::localize_disconnect_reason(reason)
    )
}

#[cfg(test)]
mod tests;
