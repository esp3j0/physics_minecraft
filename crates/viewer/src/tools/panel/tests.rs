//! panel 层测试(REQ-011 W04;AC-W11):基线差、首包自动基线、C 清零、
//! 负数防御、未收到提示、最近 5 条倒序、C/s 边界与着色、无目标 `-`、
//! 复合体计数、提示 3 秒过期、断连原因映射、全文本 ASCII。

use sim_api::proto::{DebugCounters, FlowEventWire};
use types::glam::Vec3;
use types::{BlockId, BlockPos};

use super::*;
use crate::mesh::FaceDir;
use crate::script::WorldQuery;
use crate::tools::{LaunchShape, Material, Prefab};

/// 最小假世界(一个 dirt 格 + 可调实体表)。
struct PanelWorld {
    block: Option<(BlockPos, BlockId)>,
    entities: Vec<(types::EntityId, Vec3)>,
    shapes: Vec<types::EntityId>,
}

impl PanelWorld {
    fn empty() -> Self {
        Self {
            block: None,
            entities: Vec::new(),
            shapes: Vec::new(),
        }
    }
}

impl WorldQuery for PanelWorld {
    fn live_entities(&self) -> Vec<(types::EntityId, Vec3)> {
        self.entities.clone()
    }

    fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
        match &self.block {
            Some((p, b)) if *p == pos => Some(*b),
            _ => Some(BlockId::AIR),
        }
    }

    fn chunks_loaded(&self) -> usize {
        1
    }

    fn entity_shape(&self, id: types::EntityId) -> Option<&types::CompoundShape> {
        let _ = id;
        if self.shapes.contains(&id) {
            // 测试桩:有形状的实体给一个占位形状(计数只看 Option 有无)。
            static SHAPE: std::sync::OnceLock<types::CompoundShape> = std::sync::OnceLock::new();
            Some(SHAPE.get_or_init(|| {
                types::CompoundShape::new(vec![
                    types::ShapeCell {
                        offset: [0, 0, 0],
                        block: BlockId(2),
                    },
                    types::ShapeCell {
                        offset: [1, 0, 0],
                        block: BlockId(2),
                    },
                ])
                .expect("两格梁形状")
            }))
        } else {
            None
        }
    }
}

/// 计数包构造(impact_flow_events / impact_flow_breaks / cells_blasted 自定,
/// 其余 0;recent 自定)。
fn packet(
    tick: u64,
    events: u64,
    breaks: u64,
    blasted: u64,
    recent: Vec<(Tick, FlowEventWire)>,
) -> (Tick, DebugCounters, Vec<(Tick, FlowEventWire)>) {
    (
        Tick(tick),
        DebugCounters {
            impact_flow_events: events,
            impact_flow_breaks: breaks,
            cells_blasted: blasted,
            ..DebugCounters::default()
        },
        recent,
    )
}

/// 事件构造(impact/compound、s、C、|S|、坐标)。
fn event(
    impact: bool,
    tick: u64,
    s: f32,
    c: f32,
    detached: u64,
    pos: Vec3,
) -> (Tick, FlowEventWire) {
    (
        Tick(tick),
        FlowEventWire {
            impact,
            supply: s,
            cut: c,
            broke: c < s,
            detached_cells: detached,
            position: pos,
        },
    )
}

/// 基线差:两包之间 13 项的增量正确;首包自动设基线(增量全 0)。
#[test]
fn counters_delta_against_baseline() {
    let mut panel = PanelState::default();
    panel.receive(
        packet(10, 3, 1, 5, vec![]).0,
        packet(10, 3, 1, 5, vec![]).1,
        vec![],
    );
    // 首包即基线:增量全 0。
    let lines: Vec<String> = panel_lines(&ToolState::default(), None, &PanelWorld::empty(), &panel)
        .into_iter()
        .map(|(t, _)| t)
        .collect();
    let joined = lines.join("\n");
    assert!(joined.contains("i.ev=0"), "首包增量全 0:{joined}");
    assert!(joined.contains("blast=0"), "{joined}");

    // 第二包:增量 = 当前 − 基线。
    panel.receive(
        packet(20, 7, 2, 9, vec![]).0,
        packet(20, 7, 2, 9, vec![]).1,
        vec![],
    );
    let lines: Vec<String> = panel_lines(&ToolState::default(), None, &PanelWorld::empty(), &panel)
        .into_iter()
        .map(|(t, _)| t)
        .collect();
    let joined = lines.join("\n");
    assert!(joined.contains("i.ev=4"), "7−3 = 4:{joined}");
    assert!(joined.contains("i.brk=1"), "2−1 = 1:{joined}");
    assert!(joined.contains("blast=4"), "9−5 = 4:{joined}");
}

/// `C` 清零:reset_baseline 后增量从 0 重新累计。
#[test]
fn reset_baseline_zeroes_deltas() {
    let mut panel = PanelState::default();
    panel.receive(
        packet(10, 3, 1, 5, vec![]).0,
        packet(10, 3, 1, 5, vec![]).1,
        vec![],
    );
    panel.receive(
        packet(20, 7, 2, 9, vec![]).0,
        packet(20, 7, 2, 9, vec![]).1,
        vec![],
    );
    panel.reset_baseline();
    let lines: Vec<String> = panel_lines(&ToolState::default(), None, &PanelWorld::empty(), &panel)
        .into_iter()
        .map(|(t, _)| t)
        .collect();
    let joined = lines.join("\n");
    assert!(joined.contains("i.ev=0"), "C 后增量归零:{joined}");
    assert!(joined.contains("blast=0"), "{joined}");
}

/// 计数比基线小(服务器重启归零):saturating_sub 取 0,不出负数。
#[test]
fn counters_below_baseline_clamp_to_zero() {
    let mut panel = PanelState::default();
    panel.receive(
        packet(10, 9, 5, 20, vec![]).0,
        packet(10, 9, 5, 20, vec![]).1,
        vec![],
    );
    // 服务器重启:计数回到更小值。
    panel.receive(
        packet(3, 2, 1, 4, vec![]).0,
        packet(3, 2, 1, 4, vec![]).1,
        vec![],
    );
    let lines: Vec<String> = panel_lines(&ToolState::default(), None, &PanelWorld::empty(), &panel)
        .into_iter()
        .map(|(t, _)| t)
        .collect();
    let joined = lines.join("\n");
    assert!(joined.contains("i.ev=0"), "2−9 → 0(无负数):{joined}");
    assert!(joined.contains("blast=0"), "{joined}");
    assert!(!joined.contains("i.ev=-"), "不出负号:{joined}");
}

/// 未收到任何 DebugStats:计数段只显示 `waiting for server`。
#[test]
fn waiting_for_server_before_first_packet() {
    let panel = PanelState::default();
    let lines: Vec<String> = panel_lines(&ToolState::default(), None, &PanelWorld::empty(), &panel)
        .into_iter()
        .map(|(t, _)| t)
        .collect();
    let joined = lines.join("\n");
    assert!(joined.contains("waiting for server"), "{joined}");
    assert!(!joined.contains("i.ev"), "未收到时不显示计数项:{joined}");
    assert!(
        !joined.contains("-- events --") || !joined.contains("t="),
        "无事件段内容:{joined}"
    );
}

/// 最近事件:最新 5 条、新的在上;第 6 条以前的不显示。
#[test]
fn recent_events_latest_five_newest_first() {
    let mut panel = PanelState::default();
    let recent: Vec<(Tick, FlowEventWire)> = (1..=7)
        .map(|i| {
            event(
                true,
                i,
                10.0 + i as f32,
                7.0,
                1,
                Vec3::new(i as f32, 0.0, 0.0),
            )
        })
        .collect();
    panel.receive(Tick(100), DebugCounters::default(), recent);
    let lines: Vec<String> = panel_lines(&ToolState::default(), None, &PanelWorld::empty(), &panel)
        .into_iter()
        .map(|(t, _)| t)
        .collect();
    let event_lines: Vec<&String> = lines.iter().filter(|l| l.starts_with("t=")).collect();
    assert_eq!(event_lines.len(), 5, "恰 5 条");
    assert!(
        event_lines[0].contains("t=7"),
        "最新在最上:{}",
        event_lines[0]
    );
    assert!(
        event_lines[4].contains("t=3"),
        "第 5 条 = 倒数第 5:{}",
        event_lines[4]
    );
    assert!(
        !event_lines
            .iter()
            .any(|l| l.contains("t=2") || l.contains("t=1")),
        "第 6 条以前不显示"
    );
}

/// `C/s` 两位小数、BREAK/hold 判定与着色;`C = 0`(显示 0.00,判断开)、
/// `s = 0`(显示 inf,防御)两边界。
#[test]
fn event_ratio_verdict_and_colors() {
    let mut panel = PanelState::default();
    let recent = vec![
        event(true, 1, 9.0, 7.0, 1, Vec3::ZERO), // C/s = 0.78 < 1 → BREAK
        event(false, 2, 9.0, 9.0, 0, Vec3::ZERO), // C/s = 1.00 → hold
        event(true, 3, 12.3456, 0.0, 1, Vec3::ZERO), // C = 0 → 0.00,BREAK
        event(false, 4, 0.0, 5.0, 0, Vec3::ZERO), // s = 0 → inf(防御);C/s=inf ≥ 1 → hold
    ];
    panel.receive(Tick(10), DebugCounters::default(), recent);
    let out = panel_lines(&ToolState::default(), None, &PanelWorld::empty(), &panel);
    let events: Vec<&(String, PanelColor)> =
        out.iter().filter(|(l, _)| l.starts_with("t=")).collect();
    assert_eq!(events.len(), 4);
    // 新的在上:t=4(inf)→ t=3(0.00)→ t=2(hold)→ t=1(0.78)。
    // s = 0 → C/s 显示 inf(防御);按字面口径 inf 不 < 1 → hold、不着断开色。
    assert!(
        events[0].0.contains("C/s=inf") && events[0].0.contains("hold"),
        "{:?}",
        events[0]
    );
    assert_eq!(events[0].1, PanelColor::Normal);
    assert!(
        events[1].0.contains("C/s=0.00") && events[1].0.contains("BREAK"),
        "{:?}",
        events[1]
    );
    assert!(
        events[2].0.contains("C/s=1.00") && events[2].0.contains("hold"),
        "{:?}",
        events[2]
    );
    assert_eq!(events[2].1, PanelColor::Normal, "hold 行不着断开色");
    assert!(
        events[3].0.contains("C/s=0.78") && events[3].0.contains("BREAK"),
        "{:?}",
        events[3]
    );
    assert_eq!(events[3].1, PanelColor::Break);
    // s/C 一位小数。
    assert!(
        events[3].0.contains("s=9.0") && events[3].0.contains("C=7.0"),
        "{:?}",
        events[3]
    );
    // 12.3456 → 12.3(一位小数)。
    assert!(events[1].0.contains("s=12.3"), "{:?}", events[1]);
}

/// 事件行 kind 与坐标 floor 取整。
#[test]
fn event_line_kind_and_floored_position() {
    let mut panel = PanelState::default();
    let recent = vec![
        event(true, 1, 9.0, 7.0, 2, Vec3::new(10.9, -61.2, 8.5)),
        event(false, 2, 9.0, 9.0, 0, Vec3::new(-0.5, 3.0, -2.5)),
    ];
    panel.receive(Tick(10), DebugCounters::default(), recent);
    let out = panel_lines(&ToolState::default(), None, &PanelWorld::empty(), &panel);
    let lines: Vec<String> = out.into_iter().map(|(t, _)| t).collect();
    let first = lines.iter().find(|l| l.contains("t=1")).expect("impact 行");
    assert!(first.contains("impact"), "{first}");
    assert!(
        first.contains("@(10,-62,8)"),
        "floor 取整(-61.2 → -62):{first}"
    );
    assert!(first.contains("|S|=2"), "{first}");
    let second = lines
        .iter()
        .find(|l| l.contains("t=2"))
        .expect("compound 行");
    assert!(second.contains("compound"), "{second}");
    assert!(second.contains("@(-1,3,-3)"), "负坐标 floor:{second}");
}

/// 无目标:工具段 target 显示 `-`。
#[test]
fn tools_line_without_target_shows_dash() {
    let tools = ToolState::default();
    let panel = PanelState::default();
    let out = panel_lines(&tools, None, &PanelWorld::empty(), &panel);
    let tool = out
        .iter()
        .find(|(l, _)| l.contains("target"))
        .expect("工具行")
        .0
        .clone();
    assert!(tool.contains("target -"), "{tool}");
    // 有目标:坐标与方块名。
    let mut world = PanelWorld::empty();
    world.block = Some((BlockPos::new(5, -61, 7), voxel::builtin::DIRT));
    let hit = Hit {
        block_pos: BlockPos::new(5, -61, 7),
        block: voxel::builtin::DIRT,
        face: FaceDir::Top,
    };
    let out = panel_lines(&tools, Some(&hit), &world, &panel);
    let tool = out
        .iter()
        .find(|(l, _)| l.contains("target"))
        .expect("工具行")
        .0
        .clone();
    assert!(tool.contains("(5,-61,7) dirt"), "{tool}");
    // 材料与上一发。
    let tools = ToolState {
        material: Material::Stone,
        last_shot: Some((LaunchShape::BarLongitudinal, "dirt", 30.0)),
        ..ToolState::default()
    };
    let out = panel_lines(&tools, Some(&hit), &world, &panel);
    let tool = out
        .iter()
        .find(|(l, _)| l.contains("mat stone"))
        .expect("工具行")
        .0
        .clone();
    assert!(tool.contains("last bar-long dirt 30"), "{tool}");
}

/// 复合体计数只计有形状的实体(本地桩:3 活实体、1 有形状 → 1 compound)。
#[test]
fn entity_counts_only_shaped_compounds() {
    let mut world = PanelWorld::empty();
    world.entities = vec![
        (types::EntityId::from_parts(1, 1), Vec3::ZERO),
        (types::EntityId::from_parts(1, 2), Vec3::ZERO),
        (types::EntityId::from_parts(1, 3), Vec3::ZERO),
    ];
    world.shapes = vec![types::EntityId::from_parts(1, 2)];
    let out = panel_lines(&ToolState::default(), None, &world, &PanelState::default());
    let line = out
        .iter()
        .find(|(l, _)| l.contains("entities:"))
        .expect("实体行")
        .0
        .clone();
    assert!(line.contains("3 live, 1 compound"), "{line}");
}

/// 提示行:3 秒过期沿用 ToolState::tick(面板文本透传当前值)。
#[test]
fn notice_line_expires_after_three_seconds() {
    let mut tools = ToolState::default();
    let panel = PanelState::default();
    let world = PanelWorld::empty();
    let has_notice = |tools: &ToolState| {
        panel_lines(tools, None, &world, &panel)
            .iter()
            .any(|(l, c)| *c == PanelColor::Notice && l.contains("rejected"))
    };
    tools.reject("rejected: no target");
    assert!(has_notice(&tools), "提示在");
    // 整秒递减(浮点精确;0.1×30 的 f32 累计 = 2.999… 不触发,tools_tests 同教训)。
    tools.tick(1.0);
    tools.tick(1.0);
    assert!(has_notice(&tools), "2 s 仍在");
    tools.tick(1.0);
    assert!(!has_notice(&tools), "3 秒后过期(面板不再透传)");
}

/// 断连原因映射:三个已知原因 + 非 ASCII 替换 + 英文透传。
#[test]
fn disconnect_reason_english_mapping() {
    use crate::client::localize_disconnect_reason as loc;
    assert_eq!(loc("连接关闭"), "connection closed");
    assert_eq!(loc("帧解码失败"), "frame decode failed");
    assert_eq!(
        loc("连接失败:拒绝连接 (os error 10061)"),
        "connect failed: ???? (os error 10061)"
    );
    // 服务器 Bye 原因(英文)透传。
    assert_eq!(loc("server shutdown"), "server shutdown");
    assert_eq!(loc("goodbye"), "goodbye");
    // 非 ASCII 一律 `?`。
    assert_eq!(loc("系统错误"), "????");
    // disconnect_text 包装。
    let text = disconnect_text("连接关闭");
    assert_eq!(text, "disconnected: connection closed (window will close)");
}

/// 面板全部文本只含 ASCII(一组典型输入)。
#[test]
fn panel_text_is_ascii_only() {
    let mut world = PanelWorld::empty();
    world.block = Some((BlockPos::new(-3, -62, 9), voxel::builtin::STONE));
    world.entities = vec![(types::EntityId::from_parts(1, 1), Vec3::ZERO)];
    world.shapes = vec![types::EntityId::from_parts(1, 1)];
    let mut tools = ToolState {
        material: Material::Sand,
        placeable: Placeable::Prefab(Prefab::DirtLintel),
        ..ToolState::default()
    };
    tools.reject("rejected: bedrock cannot be launched");
    let mut panel = PanelState::default();
    let recent = vec![
        event(true, 1, 9.217, 7.0, 1, Vec3::new(10.5, -57.5, 20.5)),
        event(false, 2, 23.87, 17.898, 1, Vec3::new(138.5, 3.4, 8.5)),
    ];
    panel.receive(Tick(120), DebugCounters::default(), recent);
    let out = panel_lines(&tools, None, &world, &panel);
    assert!(out.len() >= 8, "典型输入行数合理:{}", out.len());
    for (line, _) in &out {
        assert!(
            line.is_ascii(),
            "面板文本须全 ASCII(不相干 CJK 会画成方框):{line}"
        );
    }
    // 断连文本同口径。
    assert!(disconnect_text("连接关闭").is_ascii());
    assert!(disconnect_text("帧解码失败").is_ascii());
    assert!(disconnect_text("连接失败:os error 10061").is_ascii());
    // 非 ASCII 原因(如系统语言的 io::Error)也被替换成 ASCII。
    assert!(disconnect_text("连接失败:拒绝访问").is_ascii());
}
