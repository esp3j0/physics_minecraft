//! stage 层测试(REQ-011 W05;AC-W13):切分点对六个真实场景文件、布景执行
//! 对打桩世界发出的命令序列、R 的清场顺序、K 的播放范围与二次按无效、
//! wait_until 超时提示、ASCII 文本。

use std::path::Path;
use std::sync::OnceLock;

use types::glam::Vec3;
use types::{BlockId, BlockPos, CompoundShape, EntityId};

use super::{
    StageDriver, StageOut, StagePhase, fill_cells, has_server_table, is_action_step, split,
};
use crate::script::{CameraPose, EngineCommand, Scenario, WorldQuery, parse_scenario};

/// 读仓库里的真实场景文件(AC-W13 用现有场景核对切分)。
fn load_scenario(name: &str) -> Scenario {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../smoke/scenarios")
        .join(format!("{name}.toml"));
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("读 {} 失败:{e}", path.display()));
    parse_scenario(&text).unwrap_or_else(|e| panic!("{name} 解析失败:{e}"))
}

/// 六个场景的切分点(§3.6 核对结论;本测试断言失效 = 切分规则跑偏)。
/// punch/pillar 的布景是结构本身(动作从 spawn 起);door-lintel/cantilever
/// 布景含临时脚手架(动作从拆脚手架的 fill air 起);beam-whole 动作从移走
/// 左柱起;impact-corner 布景是完整门框(动作从去掉左柱起);shatter 布景
/// 只有相机和等待(动作从 spawn 起);materials 全部是布景。
#[test]
fn split_points_match_spec_on_real_scenarios() {
    let expected: [(&str, usize, usize); 6] = [
        ("req010-impact-punch", 9, 24),
        ("req010-door-lintel", 14, 19),
        ("req010-beam-whole", 10, 14),
        ("req010-impact-corner", 8, 35),
        ("req010-shatter", 3, 18),
        ("req009-materials", 12, 0),
    ];
    for (name, staging, action) in expected {
        let scenario = load_scenario(name);
        let s = split(&scenario);
        assert_eq!(
            (s.staging.len(), s.action.len()),
            (staging, action),
            "{name} 切分点不符(实得 {}/{},期望 {staging}/{action})",
            s.staging.len(),
            s.action.len()
        );
        assert_eq!(s.staging.len() + s.action.len(), scenario.steps.len());
        // 布景段每步都不是动作步骤;动作段第一步(若存在)是动作步骤——段内
        // 其余允许 wait/check/screenshot 等非动作步骤(它们跟随动作段执行口径)。
        for &ix in &s.staging {
            assert!(
                !is_action_step(&scenario.steps[ix].action),
                "{name} 步 {ix}"
            );
        }
        if let Some(&first) = s.action.first() {
            assert!(
                is_action_step(&scenario.steps[first].action),
                "{name} 步 {first}"
            );
        }
    }
}

/// 动作步骤判定的单元口径:spawn/detonate 恒真;写空气的 set_block/fill 真;
/// 写实心方块的不算(搭结构是布景)。
#[test]
fn action_step_predicate_by_air_writes() {
    let block = |b: &str| {
        let text = format!(
            "name=\"t\"\ndescription=\"d\"\ntimeout_secs=60\n[[step]]\nset_block={{pos=[0,0,0],block=\"{b}\"}}"
        );
        parse_scenario(&text).unwrap()
    };
    for name in ["dirt", "stone", "bedrock", "sand"] {
        let s = block(name);
        assert!(
            !is_action_step(&s.steps[0].action),
            "写实心 {name} 不是动作步骤"
        );
    }
    let air = block("air");
    assert!(is_action_step(&air.steps[0].action), "写空气是动作步骤");

    let text = "name=\"t\"\ndescription=\"d\"\ntimeout_secs=60\n[[step]]\nfill={from=[0,0,0],to=[1,1,1],block=\"air\"}\n[[step]]\nfill={from=[0,0,0],to=[1,1,1],block=\"dirt\"}";
    let s = parse_scenario(text).unwrap();
    assert!(is_action_step(&s.steps[0].action));
    assert!(!is_action_step(&s.steps[1].action));
}

/// 打桩世界:可注入活实体;方块恒返回空气(检查即时成立),chunk 数固定 9。
struct StubWorld {
    entities: Vec<(EntityId, Vec3)>,
}

impl StubWorld {
    fn empty() -> Self {
        Self {
            entities: Vec::new(),
        }
    }
    fn with_entities(ids: Vec<EntityId>) -> Self {
        let mut ids = ids;
        ids.sort_unstable();
        Self {
            entities: ids.into_iter().map(|id| (id, Vec3::ZERO)).collect(),
        }
    }
}

impl WorldQuery for StubWorld {
    fn live_entities(&self) -> Vec<(EntityId, Vec3)> {
        self.entities.clone()
    }
    fn block_at(&self, _pos: BlockPos) -> Option<BlockId> {
        Some(BlockId::AIR)
    }
    fn chunks_loaded(&self) -> usize {
        9
    }
}

/// 最小场景文本构造(顶层键 + [[step]];与解析器口径一致)。
fn scenario_text(steps: &[&str]) -> String {
    let mut text = String::from("name = \"t\"\ndescription = \"d\"\ntimeout_secs = 60\n");
    for s in steps {
        text.push_str("[[step]]\n");
        text.push_str(s);
        text.push('\n');
    }
    text
}

fn parse(text: &str) -> Scenario {
    parse_scenario(text).unwrap()
}

/// 相机位姿(测试用固定值)。
fn pose() -> CameraPose {
    CameraPose {
        pos: Vec3::new(0.0, 0.0, 0.0),
        yaw: 0.0,
        pitch: 0.0,
    }
}

/// 布景执行:对打桩世界执行布景段,命令序列 = 布景段里 set_block/fill 的展开,
/// fill 分批同脚本引擎(>64 格拆多批;AC-W13)。相机/look_at 生效,
/// check/screenshot/fly/couple 跳过。
#[test]
fn staging_executes_expanded_set_and_fill_in_batches() {
    // 布景段:camera、look_at、1 条 set_block、126 格 fill(64+62 拆两批)、
    // wait_ticks、check(跳过)、screenshot(跳过)、fly(跳过)、couple(跳过);
    // 动作段从写空气起(不在本测试断言范围)。
    let text = scenario_text(&[
        "camera = { pos = [1.0, 2.0, 3.0], yaw = 45.0, pitch = -10.0 }",
        "look_at = { target = [1.0, 1.0, 4.0] }",
        "set_block = { pos = [5, -60, 5], block = \"stone\" }",
        "fill = { from = [0, -60, 0], to = [6, -58, 5], block = \"dirt\" }", // 7*3*6=126
        "wait_ticks = { n = 10 }",
        "check = { chunks_loaded = { ge = 1 } }",
        "screenshot = { name = \"x\", expect = \"e\" }",
        "fly = { to = [9.0, 9.0, 9.0], speed = 10.0 }",
        "couple = { on = false }",
        "set_block = { pos = [5, -60, 5], block = \"air\" }",
    ]);
    let scenario = parse(&text);
    let mut driver = StageDriver::new(scenario, pose());
    let world = StubWorld::empty();

    // 帧 1(tick 0):瞬时步骤全发,fill 第一批 64 条截断(1 set_block + 64)。
    let out = driver.frame(0.016, 0, &world);
    let mut set_count = 0usize;
    for entry in &out {
        match entry {
            StageOut::Command(EngineCommand::SetBlock { .. }) => set_count += 1,
            StageOut::Command(EngineCommand::Spawn { .. }) => panic!("布景段不发 spawn"),
            StageOut::Command(EngineCommand::Detonate { .. }) => panic!("布景段不发 detonate"),
            StageOut::Remove(_) => panic!("布景段不应发 RemoveEntity"),
        }
    }
    assert_eq!(set_count, 65, "帧 1 = set_block 1 + fill 第一批 64 = 65 条");
    assert_eq!(
        driver.phase(),
        StagePhase::Building,
        "fill 未发完,布景仍在执行"
    );

    // 帧 2(同拍):fill 尾批 62 条;wait_ticks(起点 = 进入帧拍 0,n=10)未到。
    let out = driver.frame(0.016, 0, &world);
    set_count = 0;
    for entry in &out {
        match entry {
            StageOut::Command(EngineCommand::SetBlock { .. }) => set_count += 1,
            _ => panic!("帧 2 只有 fill 尾批"),
        }
    }
    assert_eq!(set_count, 62, "fill 尾批 = 126 - 64 = 62 条");
    assert_eq!(driver.phase(), StagePhase::Building, "wait_ticks 10 拍未到");

    // fill 展开条数与脚本引擎同一实现一致。
    assert_eq!(
        fill_cells(BlockPos::new(0, -60, 0), BlockPos::new(6, -58, 5)).len(),
        126
    );

    // tick 推进到 10:wait_ticks 结束 → Ready。
    let out = driver.frame(0.016, 9, &world);
    assert!(out.is_empty(), "等待帧无命令");
    assert_eq!(driver.phase(), StagePhase::Building, "tick 9 < 10 仍等");
    let out = driver.frame(0.016, 10, &world);
    assert!(out.is_empty(), "收尾帧无命令");
    assert_eq!(driver.phase(), StagePhase::Ready, "布景段完成");
    assert_eq!(driver.status_line(), "stage: ready (R rebuild, K play)");
    // 相机停在布景位姿(camera + look_at 的合成,值由解析器语义定;只验被改写)。
    assert_ne!(driver.camera().pos, Vec3::ZERO, "布景相机生效");
}

/// 布景执行跳过列表的正向验证:动作段的相机步骤在 K 播放时被跳过
/// (K 后相机保持布景位姿)。
#[test]
fn camera_follows_staging_and_k_skips_action_cameras() {
    let text = scenario_text(&[
        "camera = { pos = [1.0, 2.0, 3.0], yaw = 90.0, pitch = 0.0 }",
        "look_at = { target = [1.0, 1.0, 4.0] }",
        "spawn = { at = [5.0, -59.0, 5.0], count = 1, velocity = [1.0, 0.0, 0.0] }",
        "camera = { pos = [9.0, 9.0, 9.0], yaw = 0.0, pitch = 0.0 }",
    ]);
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();
    let _ = driver.frame(0.016, 0, &world);
    assert_eq!(driver.phase(), StagePhase::Ready);
    let cam = driver.camera();
    assert_eq!(cam.pos, Vec3::new(1.0, 2.0, 3.0), "布景相机生效");
    // look_at:从 (1,2,3) 看向 (1,1,4) → 水平距离 1、dy = −1 → pitch −45°(yaw 不变,方向沿 +z)。
    assert!((cam.pitch - (-45.0)).abs() < 1e-4, "look_at pitch = −45°");

    driver.play();
    let out = driver.frame(0.016, 0, &world);
    assert_eq!(out.len(), 1, "K 只发动作段的 spawn");
    assert!(matches!(
        &out[0],
        StageOut::Command(EngineCommand::Spawn { state, .. })
            if state.translation == Vec3::new(5.0, -59.0, 5.0)
    ));
    assert_eq!(
        driver.phase(),
        StagePhase::Played,
        "动作段一步播完(第二步相机被跳过)"
    );
    assert_eq!(
        driver.camera().pos,
        Vec3::new(1.0, 2.0, 3.0),
        "K 跳过相机步骤:相机保持布景位姿"
    );
    assert!(driver.status_line().contains("action playback done"));
}

/// 相机覆写纯函数(W05-2):只在 `Building`(搭布景,含 R 重放)时返回
/// `Some(布景位姿)`;`Playing` 返回 `None`(§3.6:K 播放相机留给用户——
/// punch 动作段带等待,锁死相机用户看不到撞击);`Ready` / `Played` 同样
/// `None`;`rebuild()` 后回 `Building` 重新返回 `Some`。
#[test]
fn camera_override_only_while_building() {
    let text = scenario_text(&[
        "camera = { pos = [1.0, 2.0, 3.0], yaw = 0.0, pitch = 0.0 }",
        "wait_ticks = { n = 5 }",
        "spawn = { at = [0.0, -59.0, 0.0], count = 1, velocity = [0.0, 0.0, 0.0] }",
        "wait_ticks = { n = 5 }",
    ]);
    // wait_ticks 不是动作步骤 → 布景 = [0, 1],动作 = [2, 3]。
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();

    // Building(刚创建,布景相机步骤尚未跑):Some(初始位姿)。
    assert_eq!(
        driver.camera_override(),
        Some(pose()),
        "Building 时覆写相机"
    );

    // 布景段等待中(camera 步骤已跑 → 位姿已更新,仍 Some)。
    let _ = driver.frame(0.016, 0, &world);
    assert_eq!(
        driver.camera_override().map(|c| c.pos),
        Some(Vec3::new(1.0, 2.0, 3.0)),
        "Building 等待中仍覆写,且位姿是布景相机"
    );
    let _ = driver.frame(0.016, 5, &world);
    assert_eq!(driver.phase(), StagePhase::Ready);
    assert_eq!(driver.camera_override(), None, "Ready 不覆写");

    // Playing:None(动作段的等待步骤播放中,用户可自由转视角)。
    driver.play();
    assert!(
        matches!(driver.phase(), StagePhase::Playing { .. }),
        "动作段有等待,进入 Playing"
    );
    assert_eq!(driver.camera_override(), None, "Playing 不覆写");

    // Played:None(动作段 = spawn + wait_ticks 5:tick 10 到点播完)。
    let _ = driver.frame(0.016, 5, &world); // spawn 当帧发出,wait_ticks 起点 = 5
    let _ = driver.frame(0.016, 10, &world); // 10 ≥ 5 + 5 → 播完
    assert_eq!(driver.phase(), StagePhase::Played);
    assert_eq!(driver.camera_override(), None, "Played 不覆写");

    // R → 重新 Building:重新 Some(布景位姿)。
    driver.rebuild(&world, &mut Vec::new());
    assert_eq!(driver.phase(), StagePhase::Building);
    assert_eq!(
        driver.camera_override().map(|c| c.pos),
        Some(Vec3::new(1.0, 2.0, 3.0)),
        "R 后重放布景,相机覆写恢复(回布景位姿)"
    );
}

/// Building 时按 `K`(W05-2):无效——phase 与 cursor 不变、不产生命令,
/// 并给出提示 `stage still building, wait`(ASCII);布景搭完进 Ready 后
/// `play()` 正常进入 Playing。
#[test]
fn play_during_building_is_rejected_with_notice() {
    let text = scenario_text(&[
        "wait_ticks = { n = 5 }",
        "spawn = { at = [0.0, -59.0, 0.0], count = 1, velocity = [1.0, 0.0, 0.0] }",
    ]);
    // 布景 = [0](wait_ticks 等待中),动作 = [1]。
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();
    let _ = driver.frame(0.016, 0, &world);
    assert_eq!(
        driver.phase(),
        StagePhase::Building,
        "wait_ticks 5 拍等待中"
    );

    // Building 中按 K:拒绝 + 提示,状态不变。
    driver.play();
    assert_eq!(
        driver.phase(),
        StagePhase::Building,
        "Building 按 K 不切播放"
    );
    let out = driver.frame(0.016, 0, &world);
    assert!(out.is_empty(), "Building 按 K 不产生命令");
    let notice = driver.take_reject_notice().expect("Building 按 K 要给提示");
    assert_eq!(notice, "stage still building, wait");
    assert!(notice.is_ascii());
    assert!(driver.take_reject_notice().is_none(), "提示取走一次性");

    // cursor 不变:布景段等待仍在同一步(camera() 仍为初始位姿,佐证没有
    // 前进过任何 camera 步骤)。
    assert_eq!(
        driver.camera(),
        pose(),
        "cursor 未前进(无 camera 步骤被跑过)"
    );

    // 布景搭完 → Ready:play() 正常生效。
    let _ = driver.frame(0.016, 5, &world);
    assert_eq!(driver.phase(), StagePhase::Ready);
    driver.play();
    assert!(
        matches!(driver.phase(), StagePhase::Playing { .. }),
        "Ready 按 K 正常播放"
    );
    let out = driver.frame(0.016, 5, &world);
    assert_eq!(out.len(), 1, "Ready 后 K 发出动作段 spawn");
    assert_eq!(driver.phase(), StagePhase::Played);
}

/// `R`:打桩世界 3 个活实体,按 R 后先发 3 条 RemoveEntity(按 EntityId 升序),
/// 再重放布景(相机回到布景位姿由布景段 camera 步骤恢复;AC-W13)。
#[test]
fn rebuild_removes_entities_ascending_then_replays_staging() {
    let text = scenario_text(&[
        "camera = { pos = [4.0, 5.0, 6.0], yaw = 0.0, pitch = 0.0 }",
        "set_block = { pos = [1, -60, 1], block = \"stone\" }",
        "spawn = { at = [2.0, -59.0, 2.0], count = 1, velocity = [0.0, 0.0, 0.0] }",
    ]);
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();
    let _ = driver.frame(0.016, 0, &world);
    assert_eq!(driver.phase(), StagePhase::Ready);

    // 布景后世界里有 3 个实体(乱序注入)。
    let world = StubWorld::with_entities(vec![
        EntityId::from_parts(0, 7),
        EntityId::from_parts(0, 3),
        EntityId::from_parts(0, 5),
    ]);
    let mut out = Vec::new();
    driver.rebuild(&world, &mut out);
    assert_eq!(
        out,
        vec![
            StageOut::Remove(EntityId::from_parts(0, 3)),
            StageOut::Remove(EntityId::from_parts(0, 5)),
            StageOut::Remove(EntityId::from_parts(0, 7)),
        ],
        "RemoveEntity 按 EntityId 升序"
    );
    assert_eq!(driver.phase(), StagePhase::Building, "R 后重放布景");

    // 重放:tick 0 一帧发完瞬时步骤(相机 → (4,5,6)、set_block 1 条,动作段
    // 的 spawn 不发),完成 → Ready;相机已回到布景位姿。
    let replay = driver.frame(0.016, 0, &world);
    let sets = replay
        .iter()
        .filter(|e| matches!(e, StageOut::Command(EngineCommand::SetBlock { .. })))
        .count();
    assert_eq!(sets, 1, "重放只发布景段命令,不发动作段 spawn");
    assert!(!replay.iter().any(|e| matches!(e, StageOut::Remove(_))));
    assert_eq!(driver.phase(), StagePhase::Ready);
    assert_eq!(
        driver.camera().pos,
        Vec3::new(4.0, 5.0, 6.0),
        "相机回布景位姿"
    );
}

/// `K` 播放:count=2 → 两条 spawn 当帧发出,一帧即播完 → Played。
#[test]
fn play_emits_all_spawns_then_finishes() {
    let text = scenario_text(&[
        "wait_ticks = { n = 5 }",
        "spawn = { at = [0.0, -59.0, 0.0], count = 2, velocity = [1.0, 0.0, 0.0] }",
    ]);
    // wait_ticks 不是动作步骤 → 布景 = [0],动作 = [1]。
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();
    let _ = driver.frame(0.016, 0, &world); // 布景 wait_ticks 等待中
    let _ = driver.frame(0.016, 5, &world); // 完成 → Ready
    assert_eq!(driver.phase(), StagePhase::Ready);

    driver.play();
    let first = driver.frame(0.016, 5, &world);
    assert_eq!(first.len(), 2, "count=2 → 两条 spawn 当帧发出");
    assert_eq!(
        driver.phase(),
        StagePhase::Played,
        "spawn 不等回执,一帧播完"
    );
    assert!(driver.status_line().contains("action playback done"));
}

/// 播放中(wait_until 未满足)再按 K:不产生命令、状态不变(AC-W13)。
#[test]
fn play_k_double_press_while_waiting() {
    let text = scenario_text(&[
        "wait_ticks = { n = 1 }",
        // 动作段:先 spawn(瞬时),再 wait_until(等 chunks_loaded ge 999——
        // 永不成立,走超时路径),再 spawn。
        "spawn = { at = [0.0, -59.0, 0.0], count = 1, velocity = [0.0, 0.0, 0.0] }",
        "wait_until = { chunks_loaded = { ge = 999 }, timeout_ticks = 100 }",
        "spawn = { at = [1.0, -59.0, 0.0], count = 1, velocity = [0.0, 0.0, 0.0] }",
    ]);
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();
    let _ = driver.frame(0.016, 0, &world);
    let _ = driver.frame(0.016, 1, &world); // wait_ticks n=1 到点 → Ready
    assert_eq!(driver.phase(), StagePhase::Ready);

    driver.play();
    let out = driver.frame(0.016, 1, &world);
    assert_eq!(out.len(), 1, "第一个 spawn 当帧发出");
    assert!(
        matches!(driver.phase(), StagePhase::Playing { .. }),
        "wait_until 未满足,播放中"
    );

    // 播放中再按 K:无效(不产生命令、状态不变)。
    let before = driver.phase();
    driver.play();
    let out = driver.frame(0.016, 1, &world);
    assert!(out.is_empty(), "播放中二次 K 不产生命令");
    assert_eq!(driver.phase(), before, "状态不变");

    // 等到超时(tick 1 + 100 = 101):提示、不退出、继续下一步 → 第二个 spawn。
    let out = driver.frame(0.016, 100, &world);
    assert!(out.is_empty(), "tick 100 仍未超时(≥ 101 才超)");
    let out = driver.frame(0.016, 101, &world);
    assert_eq!(out.len(), 1, "超时后继续:第二个 spawn 发出");
    let notice = driver.take_timeout_notice().expect("超时要给提示");
    assert!(notice.starts_with("stage wait timeout: "), "{notice}");
    assert!(notice.is_ascii(), "提示 ASCII:{notice}");
    assert!(notice.contains("chunks_loaded"), "提示含条件:{notice}");
    assert_eq!(driver.phase(), StagePhase::Played, "动作段播完");
    assert!(driver.status_line().contains("action playback done"));
    assert!(driver.take_timeout_notice().is_none(), "提示取走一次性");
}

/// `[server]` 表提示:含表 → 创建即有,取走一次性;无表 → 无提示。
#[test]
fn server_table_notice_once() {
    let text = scenario_text(&["wait_ticks = { n = 1 }"]).replace(
        "timeout_secs = 60",
        "timeout_secs = 60\n\n[server]\nworld_seed = 7",
    );
    let scenario = parse(&text);
    assert!(has_server_table(&scenario), "含 [server] 表");
    let mut driver = StageDriver::new(scenario, pose());
    let notice = driver.take_server_notice().expect("含 [server] 表应提示");
    assert_eq!(
        notice,
        "[server] settings not applied (server already running); use cargo xtask smoke if needed"
    );
    assert!(notice.is_ascii());
    assert!(driver.take_server_notice().is_none(), "取走一次性");

    let mut driver = StageDriver::new(parse(&scenario_text(&["wait_ticks = { n = 1 }"])), pose());
    assert!(
        driver.take_server_notice().is_none(),
        "无 [server] 表不提示"
    );
}

/// 空动作段(materials 类)按 K:一步即完,提示播完;R 在无实体时只重放布景。
#[test]
fn play_empty_action_segment_and_rebuild_without_entities() {
    let text = scenario_text(&["camera = { pos = [3.0, 3.0, 3.0], yaw = 0.0, pitch = 0.0 }"]);
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();
    let _ = driver.frame(0.016, 0, &world);
    assert_eq!(driver.phase(), StagePhase::Ready);

    driver.play();
    assert_eq!(driver.phase(), StagePhase::Played, "空动作段立即播完");
    let out = driver.frame(0.016, 0, &world);
    assert!(out.is_empty());
    assert!(driver.status_line().contains("action playback done"));

    // 无实体 R:没有 RemoveEntity,只重放布景。
    let mut out = Vec::new();
    driver.rebuild(&world, &mut out);
    assert!(out.is_empty(), "无实体不发 RemoveEntity");
    assert_eq!(driver.phase(), StagePhase::Building);
    let _ = driver.frame(0.016, 0, &world);
    assert_eq!(driver.phase(), StagePhase::Ready);
}

/// 布景执行器全状态行 ASCII(W05 屏幕文字纪律)。
#[test]
fn status_lines_are_ascii() {
    let text = scenario_text(&[
        "wait_ticks = { n = 1 }",
        "spawn = { at = [0.0, -59.0, 0.0], count = 1, velocity = [0.0, 0.0, 0.0] }",
        "wait_until = { chunks_loaded = { ge = 999 }, timeout_ticks = 50 }",
    ]);
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();
    assert!(driver.status_line().is_ascii(), "building 行");
    let _ = driver.frame(0.016, 0, &world);
    let _ = driver.frame(0.016, 1, &world);
    driver.play();
    assert!(driver.status_line().is_ascii(), "playing 行");
    // play 后首帧(tick 1):spawn 当帧发出,wait_until 起点 = 1、timeout 50
    // → 超时点 tick 51。
    let out = driver.frame(0.016, 1, &world);
    assert_eq!(out.len(), 1, "spawn 当帧发出");
    let _ = driver.frame(0.016, 50, &world);
    assert!(
        matches!(driver.phase(), StagePhase::Playing { .. }),
        "tick 50 未超时"
    );
    let _ = driver.frame(0.016, 51, &world);
    assert_eq!(driver.phase(), StagePhase::Played, "超时后继续 → 播完");
    assert!(driver.status_line().is_ascii(), "done 行");
    let notice = driver.take_timeout_notice().expect("超时提示应存在");
    assert!(notice.is_ascii());
}

/// fill 分批与脚本引擎逐位一致:布景执行器逐帧发出的 SetBlock 序列 = 脚本
/// 引擎 fill_cells 展开按 FILL_BATCH_PER_FRAME 切片拼接(含尾批)。
#[test]
fn fill_batching_matches_script_engine_expansion() {
    // 72 格 fill(64 + 8,验证尾批):x 8 × y 3 × z 3。
    let text = scenario_text(&[
        "wait_ticks = { n = 1 }",
        "fill = { from = [0, -60, 0], to = [7, -58, 2], block = \"sand\" }", // 8*3*3=72
    ]);
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();
    let _ = driver.frame(0.016, 0, &world); // wait_ticks 等待
    let mut actual = driver.frame(0.016, 1, &world);
    actual.extend(driver.frame(0.016, 1, &world));

    let cells = fill_cells(BlockPos::new(0, -60, 0), BlockPos::new(7, -58, 2));
    assert_eq!(cells.len(), 72);
    let expected: Vec<StageOut> = cells
        .iter()
        .map(|p| {
            StageOut::Command(EngineCommand::SetBlock {
                pos: *p,
                block: voxel::builtin::SAND,
            })
        })
        .collect();
    // 两帧发完:第一批 64(fill 是动作段唯一步骤,第一帧 = 64 条),尾批 8。
    assert_eq!(actual.len(), 72, "两帧合计 = 72 条(64 + 8)");
    assert_eq!(actual, expected, "逐条与脚本引擎展开一致(顺序含排序)");
}

/// 复合形状 spawn 的 desc 带形状(K 播放;与脚本引擎同型)。
#[test]
fn play_spawn_with_shape_carries_compound_desc() {
    static SHAPE: OnceLock<CompoundShape> = OnceLock::new();
    let shape = SHAPE
        .get_or_init(|| {
            CompoundShape::new(vec![
                types::ShapeCell {
                    offset: [0, 0, 0],
                    block: voxel::builtin::STONE,
                },
                types::ShapeCell {
                    offset: [1, 0, 0],
                    block: voxel::builtin::STONE,
                },
            ])
            .unwrap()
        })
        .clone();
    let text = scenario_text(&[
        "wait_ticks = { n = 1 }",
        "spawn = { at = [2.5, -57.5, 20.5], count = 1, velocity = [20.0, 0.0, 0.0], shape = { cells = [[0,0,0],[1,0,0]], block = \"stone\" } }",
    ]);
    let mut driver = StageDriver::new(parse(&text), pose());
    let world = StubWorld::empty();
    let _ = driver.frame(0.016, 0, &world);
    let _ = driver.frame(0.016, 1, &world);
    driver.play();
    let out = driver.frame(0.016, 1, &world);
    assert_eq!(out.len(), 1);
    match &out[0] {
        StageOut::Command(EngineCommand::Spawn { state, desc }) => {
            assert_eq!(state.translation, Vec3::new(2.5, -57.5, 20.5));
            assert_eq!(state.linvel, Vec3::new(20.0, 0.0, 0.0));
            let cells = desc.shape().expect("复合 desc 带形状").cells();
            assert_eq!(cells.len(), 2, "解析出的 shape 与手搭的等价(逐格)");
            assert_eq!(cells, shape.cells(), "形状逐格一致");
        }
        other => panic!("应为 Spawn,实得 {other:?}"),
    }
}
