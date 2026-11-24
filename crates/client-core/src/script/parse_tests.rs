//! 解析单元测试(纪律 11、12):每条锚定一个可变异的实现点,功能失效即红。

use crate::script::{
    Action, Check, Cmp, DetonateAt, FILL_MAX_CELLS, ScriptError, SpawnAt, parse_scenario,
};
use types::glam::Vec3;
use types::{BlockId, BlockPos};

use crate::script::tests::HEADER;

/// 单步场景文本(带 `[[step]]`)。
fn one_step(action: &str) -> String {
    format!("{HEADER}[[step]]\n{action}\n")
}

fn err(text: &str) -> ScriptError {
    parse_scenario(text).expect_err("应报错")
}

/// 坐标数组:整数可读作浮点;恰好 3 个数(细则 12)。
#[test]
fn vec3_accepts_integers_and_rejects_wrong_length() {
    let s = parse_scenario(&one_step("fly = { to = [1, 2, 3] }")).expect("整数坐标应可解析");
    assert_eq!(
        s.steps[0].action,
        Action::Fly {
            to: Vec3::new(1.0, 2.0, 3.0),
            speed: 16.0
        }
    );
    for bad in ["[1.0, 2.0]", "[1.0, 2.0, 3.0, 4.0]"] {
        let e = err(&one_step(&format!("fly = {{ to = {bad} }}")));
        assert_eq!(e.step, Some(1), "{bad}:应报步骤 1");
    }
    let e = err(&one_step("fly = { to = [1.0, \"x\", 3.0] }"));
    assert_eq!(e.step, Some(1), "非数项应报步骤 1");
}

/// `[server]` 原样透传:不校验键名,连未知键也保留(细则 12;键由 app::Config 校验)。
#[test]
fn server_table_is_passed_through_untouched() {
    let text = format!(
        "{HEADER}[server]\nworld_seed = 7\nsome_future_key = \"x\"\n\n[[step]]\nwait_ticks = {{ n = 1 }}\n"
    );
    let s = parse_scenario(&text).expect("[server] 不校验");
    assert_eq!(s.server.len(), 2);
    assert_eq!(
        s.server.get("some_future_key").and_then(|v| v.as_str()),
        Some("x")
    );
}

/// 顶层与 `[viewer]` 拒绝未知键;`req`/`timeout_secs` 可选,`description` 缺省为空。
#[test]
fn unknown_top_and_viewer_keys_rejected() {
    let e = err(&format!("{HEADER}oops = 1\n"));
    assert_eq!(e.step, None, "顶层错误无步骤序号");
    let e = err(&format!("{HEADER}[viewer]\nfov = 90\n"));
    assert_eq!(e.step, None);
    let minimal = "name = \"m\"\n[[step]]\nwait_ticks = { n = 1 }\n";
    let s = parse_scenario(minimal).expect("最小场景应可解析");
    assert_eq!(s.timeout_secs, 180);
    assert_eq!(s.req, None);
    assert_eq!(s.description, "");
}

/// spawn 的 ahead 必须搭配 lift;count/spacing/velocity/label 缺省(细则 6、12)。
#[test]
fn spawn_ahead_requires_lift_and_defaults() {
    let e = err(&one_step("spawn = { ahead = 8.0 }"));
    assert_eq!(e.step, Some(1), "ahead 无 lift 应报错");
    let s = parse_scenario(&one_step("spawn = { ahead = 8.0, lift = 6.0 }"))
        .expect("ahead + lift 应可解析");
    assert_eq!(
        s.steps[0].action,
        Action::Spawn {
            at: SpawnAt::Ahead {
                ahead: 8.0,
                lift: 6.0
            },
            count: 1,
            spacing: 1.6,
            velocity: Vec3::ZERO,
            label: None,
            shape: None,
        }
    );
}

/// detonate 的 nearest = false 无意义,报错;at 与 nearest 互斥。
#[test]
fn detonate_nearest_false_rejected() {
    let e = err(&one_step(
        "detonate = { nearest = false, power = 1.0, radius = 1.0 }",
    ));
    assert_eq!(e.step, Some(1));
    let e = err(&one_step(
        "detonate = { at = [0.0, 0.0, 0.0], nearest = true, power = 1.0, radius = 1.0 }",
    ));
    assert_eq!(e.step, Some(1));
    let s = parse_scenario(&one_step(
        "detonate = { nearest = true, power = 1.0, radius = 1.0 }",
    ))
    .expect("nearest = true 应可解析");
    assert_eq!(
        s.steps[0].action,
        Action::Detonate {
            at: DetonateAt::Nearest,
            power: 1.0,
            radius: 1.0
        }
    );
}

/// 检查项比较:恰好一个 eq/le/ge;未知检查项报错;labeled_alive 内联形态。
#[test]
fn check_forms() {
    let e = err(&one_step("check = { live_entities = {} }"));
    assert_eq!(e.step, Some(1), "无比较项应报错");
    let e = err(&one_step("check = { no_such_check = { eq = 1 } }"));
    assert_eq!(e.step, Some(1), "未知检查项应报错");
    let s = parse_scenario(&one_step(
        r#"check = { labeled_alive = { label = "b", ge = 1 }, block = { pos = [0, 0, 0], is = "stone" }, chunks_loaded = { le = 9 } }"#,
    ))
    .expect("合法检查项应可解析");
    match &s.steps[0].action {
        Action::Check { checks } => {
            // toml 表遍历序不保证与书写顺序一致(实现按表序产出),按包含断言。
            assert_eq!(checks.len(), 3, "三个检查项都在");
            assert!(checks.contains(&Check::LabeledAlive {
                label: "b".to_owned(),
                cmp: Cmp::Ge(1)
            }));
            assert!(checks.contains(&Check::Block {
                pos: BlockPos::new(0, 0, 0),
                is: BlockId(1)
            }));
            assert!(checks.contains(&Check::ChunksLoaded(Cmp::Le(9))));
        }
        other => panic!("应为 check:{other:?}"),
    }
}

/// count 区域计数检查(F04b-2):正例(from/to 无序规整、区域格数 ≤ 上限)
/// 与反例(缺键、多个比较键、超限、未知方块)。
#[test]
fn count_check_parses_and_rejects_malformed() {
    // 正例:区域 20×2×10 = 400 ≤ 4096;from/to 各分量无序 → 规整为最小/最大。
    let s = parse_scenario(&one_step(
        r#"check = { count = { from = [10, -58, 20], to = [-9, -61, 29], is = "stone", ge = 2 } }"#,
    ))
    .expect("合法 count 应可解析");
    match &s.steps[0].action {
        Action::Check { checks } => {
            assert_eq!(checks.len(), 1);
            assert_eq!(
                checks[0],
                Check::Count {
                    from: BlockPos::new(-9, -61, 20),
                    to: BlockPos::new(10, -58, 29),
                    is: BlockId(1),
                    cmp: Cmp::Ge(2),
                },
                "from/to 应规整为各轴最小/最大(含两端)"
            );
        }
        other => panic!("应为 check:{other:?}"),
    }

    // 反例(全部带步骤序号):
    // ① 缺 from;② 缺 to;③ 缺 is;④ 缺比较项;⑤ 多个比较键;
    // ⑥ 区域格数超 FILL_MAX_CELLS;⑦ 未知方块。
    for (i, bad) in [
        r#"check = { count = { to = [0, 0, 0], is = "stone", ge = 1 } }"#,
        r#"check = { count = { from = [0, 0, 0], is = "stone", ge = 1 } }"#,
        r#"check = { count = { from = [0, 0, 0], to = [1, 1, 1], ge = 1 } }"#,
        r#"check = { count = { from = [0, 0, 0], to = [1, 1, 1], is = "stone" } }"#,
        r#"check = { count = { from = [0, 0, 0], to = [1, 1, 1], is = "stone", ge = 1, le = 2 } }"#,
        r#"check = { count = { from = [0, 0, 0], to = [100, 100, 100], is = "stone", ge = 1 } }"#,
        r#"check = { count = { from = [0, 0, 0], to = [1, 1, 1], is = "lava", ge = 1 } }"#,
    ]
    .into_iter()
    .enumerate()
    {
        let e = err(&one_step(bad));
        assert_eq!(e.step, Some(1), "反例 ①..⑦ 第 {i} 条应报步骤 1:{bad}");
    }
    // ⑥ 单独校验消息含上限数字(101³ = 1 030 301 > 4096)。
    let e = err(&one_step(
        r#"check = { count = { from = [0, 0, 0], to = [100, 100, 100], is = "stone", ge = 1 } }"#,
    ));
    assert!(e.to_string().contains("4096"), "超限消息应含上限:{e}");
}

/// 方块名经内置注册表解析:air/stone/dirt → 0/1/2;未知名字报错不落成空气。
#[test]
fn block_names_resolve_via_builtin_registry() {
    for (name, id) in [("air", 0), ("stone", 1), ("dirt", 2)] {
        let s = parse_scenario(&one_step(&format!(
            "set_block = {{ pos = [0, 0, 0], block = \"{name}\" }}"
        )))
        .expect("内置方块名应可解析");
        assert_eq!(
            s.steps[0].action,
            Action::SetBlock {
                pos: BlockPos::new(0, 0, 0),
                block: BlockId(id)
            }
        );
    }
    let e = err(&one_step(
        "set_block = { pos = [0, 0, 0], block = \"lava\" }",
    ));
    assert_eq!(e.step, Some(1), "未知方块名应报错");
}

/// 步骤序号:错误落在第几个 `[[step]]` 就报几(从 1 起);顶层错误为 None。
#[test]
fn step_index_is_one_based_for_later_steps() {
    let ok = "[[step]]\nwait_ticks = { n = 1 }\n";
    let text = format!("{HEADER}{ok}{ok}[[step]]\nfly = {{ speed = 1.0 }}\n");
    let e = err(&text);
    assert_eq!(e.step, Some(3), "第 3 步缺 to");
    assert!(e.to_string().contains("第 3 步"), "Display 带序号:{e}");
}

/// wait_until 至少要能不带检查项(只等超时前的条件为空 = 立即成立)不被滥用:
/// 空检查的 wait_until 立即成立;但 check 空表报错。
#[test]
fn empty_check_rejected_but_wait_until_allows_empty() {
    let e = err(&one_step("check = {}"));
    assert_eq!(e.step, Some(1), "check 空表应报错");
    let s = parse_scenario(&one_step("wait_until = { timeout_ticks = 5 }"))
        .expect("wait_until 无检查项 = 立即成立,应可解析");
    match &s.steps[0].action {
        Action::WaitUntil {
            checks,
            timeout_ticks,
            hold_ticks,
        } => {
            assert!(checks.is_empty());
            assert_eq!(*timeout_ticks, 5);
            assert_eq!(*hold_ticks, 0, "hold_ticks 缺省 0(旧语义)");
        }
        other => panic!("应为 wait_until:{other:?}"),
    }
}

/// wait_until 的 hold_ticks(S01 冒烟降噪):可选、非负整数;缺省 0 = 旧语义;
/// 写了按写值解析;负数/非整数报错并带步骤序号;check 动作不接受 hold_ticks。
#[test]
fn wait_until_hold_ticks_parses_and_rejects_malformed() {
    // 正例:hold_ticks = 30。
    let s = parse_scenario(&one_step(
        "wait_until = { live_entities = { eq = 0 }, timeout_ticks = 900, hold_ticks = 30 }",
    ))
    .expect("hold_ticks 正例应可解析");
    match &s.steps[0].action {
        Action::WaitUntil {
            checks,
            timeout_ticks,
            hold_ticks,
        } => {
            assert_eq!(checks, &vec![Check::LiveEntities(Cmp::Eq(0))]);
            assert_eq!(*timeout_ticks, 900);
            assert_eq!(*hold_ticks, 30);
        }
        other => panic!("应为 wait_until:{other:?}"),
    }

    // hold_ticks = 0 显式写出与缺省等价。
    let s = parse_scenario(&one_step(
        "wait_until = { live_entities = { eq = 0 }, timeout_ticks = 900, hold_ticks = 0 }",
    ))
    .expect("hold_ticks = 0 应可解析");
    match &s.steps[0].action {
        Action::WaitUntil { hold_ticks, .. } => assert_eq!(*hold_ticks, 0),
        other => panic!("应为 wait_until:{other:?}"),
    }

    // 反例:负数、浮点小数、字符串;check 带 hold_ticks 报未知键。
    for bad in [
        "wait_until = { timeout_ticks = 5, hold_ticks = -1 }",
        "wait_until = { timeout_ticks = 5, hold_ticks = 1.5 }",
        "wait_until = { timeout_ticks = 5, hold_ticks = \"x\" }",
        "check = { live_entities = { eq = 0 }, hold_ticks = 5 }",
    ] {
        let e = err(&one_step(bad));
        assert_eq!(e.step, Some(1), "反例应报步骤 1:{bad} → {e}");
    }
}

/// fill(REQ-009 §10):两个对角顺序任意,规整为 from = 各轴最小、to = 各轴最大;
/// 新方块名 bedrock/sand 可用;1×1×4096 细长盒恰好合法、再长一格报错
/// (与验收用例的 16³ 形状互补,锚定"按体积而非单轴判限"与上限差一)。
#[test]
fn fill_normalizes_corners_and_cell_limit_boundary() {
    let s = parse_scenario(&one_step(
        "fill = { from = [3, -60, 5], to = [1, -58, 4], block = \"sand\" }",
    ))
    .expect("对角逆序也应可解析");
    assert_eq!(
        s.steps[0].action,
        Action::Fill {
            from: BlockPos::new(1, -60, 4),
            to: BlockPos::new(3, -58, 5),
            block: BlockId(4),
        }
    );
    let bedrock = parse_scenario(&one_step(
        "fill = { from = [0, 0, 0], to = [0, 0, 0], block = \"bedrock\" }",
    ))
    .expect("单格 fill(from = to)应可解析");
    assert_eq!(
        bedrock.steps[0].action,
        Action::Fill {
            from: BlockPos::ZERO,
            to: BlockPos::ZERO,
            block: BlockId(3),
        }
    );

    let line = |len: i32| {
        one_step(&format!(
            "fill = {{ from = [0, -64, 0], to = [0, {}, 0], block = \"stone\" }}",
            -64 + len - 1
        ))
    };
    let n = FILL_MAX_CELLS as i32;
    parse_scenario(&line(n)).expect("恰好上限格数应合法(1×1×4096)");
    let e = err(&line(n + 1));
    assert_eq!(e.step, Some(1), "超限要带步骤序号");
    assert!(e.message.contains("4096"), "错误要点名上限:{e}");
}

/// fill 错误路径:缺 to、缺 block、坐标非整数、未知方块、未知键,全部带步骤序号。
#[test]
fn fill_rejects_missing_and_malformed_params() {
    let cases = [
        "fill = { from = [0, -60, 0], block = \"stone\" }",
        "fill = { from = [0, -60, 0], to = [1, -60, 1] }",
        "fill = { from = [0, -60.5, 0], to = [1, -60, 1], block = \"stone\" }",
        "fill = { from = [0, -60, 0], to = [1, -60, 1], block = \"lava\" }",
        "fill = { from = [0, -60, 0], to = [1, -60, 1], block = \"stone\", speed = 1 }",
    ];
    for case in cases {
        let e = err(&one_step(case));
        assert_eq!(e.step, Some(1), "错误要带步骤序号:{case} → {e}");
    }
}

/// F04a `spawn.shape` 解析正例:2×1 石条,`at` = 原点格 [0,0,0] 格心(引擎侧),
/// 形状 canonical(偏移升序)、全格同块。
#[test]
fn spawn_shape_parses_and_canonicalizes() {
    let s = parse_scenario(&one_step(
        "spawn = { at = [10.0, -50.0, 20.0], shape = { cells = [[1,0,0],[0,0,0]], block = \"stone\" } }",
    ))
    .expect("shape 正例应可解析");
    let Action::Spawn { shape, .. } = &s.steps[0].action else {
        panic!("应为 spawn 动作");
    };
    let shape = shape.as_ref().expect("shape 应为 Some");
    assert_eq!(
        shape.cells().iter().map(|c| c.offset).collect::<Vec<_>>(),
        vec![[0, 0, 0], [1, 0, 0]],
        "canonical:偏移升序"
    );
    assert!(
        shape.cells().iter().all(|c| c.block == BlockId(1)),
        "全格 stone"
    );
}

/// F04a `spawn.shape` 反例:空 cells、重复偏移、不连通、未知方块、
/// 非实心方块(air)、单格、cells 非三元组、缺 block、未知键——
/// 全部带步骤序号,错误指明是哪一步校验。
#[test]
fn spawn_shape_rejects_malformed() {
    let cases = [
        (
            "空 cells",
            "spawn = { at = [0.0, -50.0, 0.0], shape = { cells = [], block = \"stone\" } }",
        ),
        (
            "重复偏移",
            "spawn = { at = [0.0, -50.0, 0.0], shape = { cells = [[0,0,0],[0,0,0],[1,0,0]], block = \"stone\" } }",
        ),
        (
            "不连通",
            "spawn = { at = [0.0, -50.0, 0.0], shape = { cells = [[0,0,0],[2,0,0]], block = \"stone\" } }",
        ),
        (
            "未知方块",
            "spawn = { at = [0.0, -50.0, 0.0], shape = { cells = [[0,0,0],[1,0,0]], block = \"lava\" } }",
        ),
        (
            "单格",
            "spawn = { at = [0.0, -50.0, 0.0], shape = { cells = [[0,0,0]], block = \"stone\" } }",
        ),
        (
            "cells 非三元组",
            "spawn = { at = [0.0, -50.0, 0.0], shape = { cells = [[0,0],[1,0,0]], block = \"stone\" } }",
        ),
        (
            "缺 block",
            "spawn = { at = [0.0, -50.0, 0.0], shape = { cells = [[0,0,0],[1,0,0]] } }",
        ),
        (
            "缺 cells",
            "spawn = { at = [0.0, -50.0, 0.0], shape = { block = \"stone\" } }",
        ),
        (
            "未知键",
            "spawn = { at = [0.0, -50.0, 0.0], shape = { cells = [[0,0,0],[1,0,0]], block = \"stone\", half = 1 } }",
        ),
    ];
    for (why, case) in cases {
        let e = err(&one_step(case));
        assert_eq!(e.step, Some(1), "{why}:错误要带步骤序号");
    }
}

// ---- A06 §4:主体动作与检查(解析 + 跨步骤校验)----

/// 新动作解析形状(spawn_agent/possess/unpossess/intent/mark_agent)。
#[test]
fn agent_actions_parse() {
    let s = parse_scenario(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [8.0, -57.0, 24.0], yaw = 90.0, label = \"p\" }}\n[[step]]\npossess = {{ label = \"p\" }}\n[[step]]\nintent = {{ move = [4.3, 0.0], yaw = 0.0, jump = false }}\n[[step]]\nmark_agent = {{ label = \"p\" }}\n[[step]]\nunpossess = {{}}\n"
    ))
    .expect("主体动作场景可解析");
    assert_eq!(s.steps.len(), 5);
    assert_eq!(
        s.steps[0].action,
        Action::SpawnAgent {
            at: Vec3::new(8.0, -57.0, 24.0),
            yaw: 90.0,
            label: "p".to_owned()
        }
    );
    assert_eq!(
        s.steps[1].action,
        Action::Possess {
            label: "p".to_owned(),
            follow: false
        },
        "follow 缺省 false"
    );
    assert_eq!(
        s.steps[2].action,
        Action::Intent {
            mv: (4.3, 0.0),
            yaw: 0.0,
            jump: false
        }
    );
    assert_eq!(s.steps[4].action, Action::Unpossess);
}

/// 错误路径(放行 §4 校验):① possess 前没有 spawn_agent;② agent_moved
/// 前没有 mark_agent;③ 未定义标签;④ 一场景两条 spawn_agent;⑤ intent/
/// unpossess/mark_agent 前没有 possess;⑥ 引用了别的标签。
#[test]
fn agent_actions_validation_errors() {
    // ① possess 前没有 spawn_agent。
    let e = err(&one_step("possess = { label = \"p\" }"));
    assert!(
        e.message.contains("spawn_agent"),
        "possess 无 spawn_agent:{}",
        e.message
    );
    // ② agent_moved 前没有 mark_agent(spawn_agent 有,mark 没有)。
    let e = err(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0.0, 0.0, 0.0], label = \"p\" }}\n[[step]]\ncheck = {{ agent_moved = {{ label = \"p\", ge = 1.0 }} }}\n"
    ));
    assert!(
        e.message.contains("mark_agent"),
        "agent_moved 无 mark:{}",
        e.message
    );
    // ③ 未定义标签(agent_in 引用没登记过的)。
    let e = err(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0.0, 0.0, 0.0], label = \"p\" }}\n[[step]]\ncheck = {{ agent_in = {{ label = \"q\", from = [0,0,0], to = [1,1,1] }} }}\n"
    ));
    assert!(e.message.contains("q"), "未定义标签:{}", e.message);
    // ④ 一场景两条 spawn_agent。
    let e = err(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0.0, 0.0, 0.0], label = \"p\" }}\n[[step]]\nspawn_agent = {{ at = [1.0, 0.0, 0.0], label = \"r\" }}\n"
    ));
    assert!(
        e.message.contains("最多一个身体"),
        "两条 spawn_agent:{}",
        e.message
    );
    // ⑤ intent 前没有 possess。
    let e = err(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0.0, 0.0, 0.0], label = \"p\" }}\n[[step]]\nintent = {{ move = [1.0, 0.0] }}\n"
    ));
    assert!(
        e.message.contains("possess"),
        "intent 无 possess:{}",
        e.message
    );
    // ⑥ mark_agent 引用别的标签。
    let e = err(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0.0, 0.0, 0.0], label = \"p\" }}\n[[step]]\npossess = {{ label = \"p\" }}\n[[step]]\nmark_agent = {{ label = \"q\" }}\n"
    ));
    assert!(e.message.contains("q"), "mark_agent 错标签:{}", e.message);
}

/// 主体检查的参数错误:agent_moved 的 ge/le 都不给、给两个、负值、非数。
#[test]
fn agent_moved_cmp_errors() {
    let cases = [
        "check = { agent_moved = { label = \"p\" } }",
        "check = { agent_moved = { label = \"p\", ge = 1.0, le = 2.0 } }",
        "check = { agent_moved = { label = \"p\", ge = -1.0 } }",
        "check = { agent_moved = { label = \"p\", ge = \"x\" } }",
    ];
    for case in cases {
        let e = err(&format!(
            "{HEADER}[[step]]\nspawn_agent = {{ at = [0.0, 0.0, 0.0], label = \"p\" }}\n[[step]]\nmark_agent = {{ label = \"p\" }}\n[[step]]\n{case}\n"
        ));
        assert_eq!(e.step, Some(3), "{case}:应报步骤 3");
    }
}

/// 合法路径:agent_moved le、agent_alive、wait_until 里用 agent_in。
#[test]
fn agent_checks_ok_forms() {
    let s = parse_scenario(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0.0, 0.0, 0.0], label = \"p\" }}\n[[step]]\npossess = {{ label = \"p\" }}\n[[step]]\nmark_agent = {{ label = \"p\" }}\n[[step]]\nwait_until = {{ agent_alive = {{ label = \"p\" }}, agent_moved = {{ label = \"p\", le = 2.5 }}, timeout_ticks = 100 }}\n"
    ))
    .expect("合法主体检查场景");
    match &s.steps[3].action {
        Action::WaitUntil { checks, .. } => {
            assert_eq!(checks.len(), 2);
            assert!(matches!(checks[0], Check::AgentAlive { .. }));
            match &checks[1] {
                Check::AgentMoved { ge, le, .. } => {
                    assert_eq!(*ge, None);
                    assert_eq!(*le, Some(2.5));
                }
                other => panic!("应为 AgentMoved,实为 {other:?}"),
            }
        }
        other => panic!("应为 WaitUntil,实为 {other:?}"),
    }
    // wait_until 里用 agent_in 也合法(标签已定义)。
    let s = parse_scenario(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0.0, 0.0, 0.0], label = \"p\" }}\n[[step]]\nwait_until = {{ agent_in = {{ label = \"p\", from = [0,0,0], to = [1,1,1] }}, timeout_ticks = 10 }}\n"
    ))
    .expect("agent_in 在 wait_until 里合法");
    assert_eq!(s.steps.len(), 2);
}

// ---- B02c §2:P1/P2(adopt_agent、mark_agent at 的解析)----

/// P1 解析:adopt_agent 四个参数、默认超时 600、未知键拒绝、标签与
/// spawn_agent 重复拒绝、两条 adopt_agent 同标签拒绝、within = 0 拒绝。
#[test]
fn adopt_agent_parse_full_default_and_rejections() {
    // 四个参数齐全。
    let s = parse_scenario(&one_step(
        "adopt_agent = { label = \"dog\", near = [24.5, -59.5, 24.5], within = 2.0, timeout_ticks = 1200 }",
    ))
    .expect("四参数应可解析");
    assert_eq!(
        s.steps[0].action,
        Action::AdoptAgent {
            label: "dog".into(),
            near: Vec3::new(24.5, -59.5, 24.5),
            within: 2.0,
            timeout_ticks: 1200,
        }
    );
    // timeout_ticks 缺省 = 600。
    let s = parse_scenario(&one_step(
        "adopt_agent = { label = \"d\", near = [0, 0, 0], within = 1.0 }",
    ))
    .expect("缺省超时应可解析");
    match &s.steps[0].action {
        Action::AdoptAgent { timeout_ticks, .. } => assert_eq!(*timeout_ticks, 600, "默认 600"),
        other => panic!("应为 AdoptAgent,实为 {other:?}"),
    }
    // 未知键拒绝。
    let e = err(&one_step(
        "adopt_agent = { label = \"d\", near = [0, 0, 0], within = 1.0, bogus = 1 }",
    ));
    assert!(e.message.contains("未知键"), "{}", e.message);
    // 标签与 spawn_agent 重复拒绝。
    let e = err(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0, 0, 0], label = \"x\" }}\n[[step]]\nadopt_agent = {{ label = \"x\", near = [0, 0, 0], within = 1.0 }}\n"
    ));
    assert!(e.message.contains("已在第 1 步登记"), "{}", e.message);
    // 两条 adopt_agent 同标签拒绝。
    let e = err(&format!(
        "{HEADER}[[step]]\nadopt_agent = {{ label = \"x\", near = [0, 0, 0], within = 1.0 }}\n[[step]]\nadopt_agent = {{ label = \"x\", near = [5, 0, 0], within = 1.0 }}\n"
    ));
    assert!(e.message.contains("已在第 1 步登记"), "{}", e.message);
    // within = 0 拒绝。
    let e = err(&one_step(
        "adopt_agent = { label = \"d\", near = [0, 0, 0], within = 0.0 }",
    ));
    assert!(e.message.contains("within"), "{}", e.message);
    // within 非有限(NaN 在 toml 里写不出;用字符串探类型)拒绝。
    let e = err(&one_step(
        "adopt_agent = { label = \"d\", near = [0, 0, 0], within = \"x\" }",
    ));
    assert!(e.message.contains("within"), "{}", e.message);
    // timeout_ticks = 0 拒绝(须 ≥ 1)。
    let e = err(&one_step(
        "adopt_agent = { label = \"d\", near = [0, 0, 0], within = 1.0, timeout_ticks = 0 }",
    ));
    assert!(e.message.contains("timeout_ticks"), "{}", e.message);
}

/// P2 解析:mark_agent 带 at、不带 at。
#[test]
fn mark_agent_parse_with_and_without_at() {
    // 带 at(附身场景,行为不变的老路径)。
    let s = parse_scenario(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0, 0, 0], label = \"p\" }}\n[[step]]\npossess = {{ label = \"p\" }}\n[[step]]\nmark_agent = {{ label = \"p\", at = [24.5, -59.72, 34.5] }}\n"
    ))
    .expect("mark_agent 带 at 应可解析");
    match &s.steps[2].action {
        Action::MarkAgent { label, at } => {
            assert_eq!(label, "p");
            assert_eq!(*at, Some(Vec3::new(24.5, -59.72, 34.5)));
        }
        other => panic!("应为 MarkAgent,实为 {other:?}"),
    }
    // 不带 at(缺省 None;老语义)。
    let s = parse_scenario(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0, 0, 0], label = \"p\" }}\n[[step]]\npossess = {{ label = \"p\" }}\n[[step]]\nmark_agent = {{ label = \"p\" }}\n"
    ))
    .expect("mark_agent 不带 at 应可解析");
    match &s.steps[2].action {
        Action::MarkAgent { at, .. } => assert_eq!(*at, None),
        other => panic!("应为 MarkAgent,实为 {other:?}"),
    }
    // 未知键拒绝。
    let e = err(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0, 0, 0], label = \"p\" }}\n[[step]]\npossess = {{ label = \"p\" }}\n[[step]]\nmark_agent = {{ label = \"p\", bogus = 1 }}\n"
    ));
    assert!(e.message.contains("未知键"), "{}", e.message);
    // 认领标签 + mark_agent at(无 possess)合法。
    let s = parse_scenario(&format!(
        "{HEADER}[[step]]\nadopt_agent = {{ label = \"dog\", near = [0, 0, 0], within = 1.0 }}\n[[step]]\nmark_agent = {{ label = \"dog\", at = [1, 2, 3] }}\n[[step]]\nwait_until = {{ agent_moved = {{ label = \"dog\", le = 5.0 }}, timeout_ticks = 10 }}\n"
    ))
    .expect("认领标签可 mark_agent(at)");
    assert_eq!(s.steps.len(), 3);
}

// ---- E03 R1-a:possess / 无 at mark_agent 标签校验放宽(spawn 或 adopt)----

/// R1-a①:adopt_agent 登记的标签可 possess,亦可无 at mark_agent(旧口径作废)。
#[test]
fn possess_accepts_adopted_label() {
    let s = parse_scenario(&format!(
        "{HEADER}[[step]]\nadopt_agent = {{ label = \"dog\", near = [0, 0, 0], within = 1.0 }}\n[[step]]\npossess = {{ label = \"dog\" }}\n[[step]]\nmark_agent = {{ label = \"dog\" }}\n"
    ))
    .expect("认领标签可附身(E03 R1-a)");
    assert_eq!(s.steps.len(), 3);
    match &s.steps[1].action {
        Action::Possess { label, .. } => assert_eq!(label, "dog"),
        other => panic!("应为 Possess,实为 {other:?}"),
    }
}

/// R1-a②:possess 引用未登记标签仍报错(点名该标签)。
#[test]
fn possess_rejects_unregistered_label() {
    let e = err(&format!(
        "{HEADER}[[step]]\nadopt_agent = {{ label = \"dog\", near = [0, 0, 0], within = 1.0 }}\n[[step]]\npossess = {{ label = \"cat\" }}\n"
    ));
    assert_eq!(e.step, Some(2));
    assert!(
        e.message.contains("possess 引用标签 \"cat\""),
        "{}",
        e.message
    );
    assert!(
        e.message.contains("spawn_agent/adopt_agent 登记"),
        "{}",
        e.message
    );
    // 无任何登记时 possess 也报错。
    let e = err(&one_step("possess = { label = \"dog\" }"));
    assert_eq!(e.step, Some(1));
    assert!(
        e.message.contains("此前没有 spawn_agent/adopt_agent 登记"),
        "{}",
        e.message
    );
}

/// R1-a③:spawn_agent→possess 不回归;一场景一身体限制不变。
#[test]
fn possess_spawn_agent_no_regression() {
    let s = parse_scenario(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0, 0, 0], label = \"p\" }}\n[[step]]\npossess = {{ label = \"p\" }}\n[[step]]\nmark_agent = {{ label = \"p\" }}\n"
    ))
    .expect("spawn_agent→possess 不回归");
    assert_eq!(s.steps.len(), 3);
    // 第二次 spawn_agent 仍拒(一场景一身体不变)。
    let e = err(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0, 0, 0], label = \"p\" }}\n[[step]]\nspawn_agent = {{ at = [1, 0, 0], label = \"q\" }}\n"
    ));
    assert!(e.message.contains("一场景最多一个身体"), "{}", e.message);
    // possess 引用与 spawn 不同的未登记标签仍拒。
    let e = err(&format!(
        "{HEADER}[[step]]\nspawn_agent = {{ at = [0, 0, 0], label = \"p\" }}\n[[step]]\npossess = {{ label = \"q\" }}\n"
    ));
    assert!(
        e.message.contains("possess 引用标签 \"q\""),
        "{}",
        e.message
    );
}

/// R2-b①:adopt 登记后不 possess 直接无 at mark_agent → 解析通过;
/// 从未登记的标签 → 错误点名该标签。
#[test]
fn mark_agent_no_at_without_possess() {
    let s = parse_scenario(&format!(
        "{HEADER}[[step]]\nadopt_agent = {{ label = \"dog\", near = [0, 0, 0], within = 1.0 }}\n[[step]]\nmark_agent = {{ label = \"dog\" }}\n[[step]]\nwait_until = {{ agent_moved = {{ label = \"dog\", ge = 2.0 }}, timeout_ticks = 10 }}\n"
    ))
    .expect("认领标签不附身可直接无 at mark_agent(E03 R2-b)");
    assert_eq!(s.steps.len(), 3);
    match &s.steps[1].action {
        Action::MarkAgent { label, at } => {
            assert_eq!(label, "dog");
            assert_eq!(*at, None);
        }
        other => panic!("应为 MarkAgent,实为 {other:?}"),
    }
    // 从未登记的标签 → 错误点名该标签。
    let e = err(&one_step("mark_agent = { label = \"ghost\" }"));
    assert_eq!(e.step, Some(1));
    assert!(
        e.message.contains("mark_agent 引用标签 \"ghost\""),
        "{}",
        e.message
    );
    assert!(
        e.message.contains("spawn_agent/adopt_agent 登记"),
        "{}",
        e.message
    );
}

/// REQ-013 C01b-3 R1 MS11:`perf_reset = {}` 合法空表解析为 `Action::PerfReset`;
/// 带多余键报错。
#[test]
fn ms11_parse_perf_reset() {
    let s = parse_scenario(&one_step("perf_reset = {}")).expect("空表 perf_reset 应合法解析");
    assert_eq!(s.steps.len(), 1);
    assert_eq!(s.steps[0].action, Action::PerfReset);

    let e = err(&one_step("perf_reset = { extra = 1 }"));
    assert_eq!(e.step, Some(1));
    assert!(
        e.message.contains("extra"),
        "带多余键必须点名报错:{}",
        e.message
    );
}

/// REQ-013 C02-1 §4.1 MS21:`dig`/`place`/`select_material` 白名单与参数校验。
#[test]
fn ms21_parse_dig_place_select_material() {
    let s = parse_scenario(&one_step("dig = {}")).expect("空表 dig 应合法解析");
    assert_eq!(s.steps.len(), 1);
    assert_eq!(s.steps[0].action, Action::Dig);

    let e = err(&one_step("dig = { extra = 1 }"));
    assert_eq!(e.step, Some(1));
    assert!(
        e.message.contains("extra"),
        "dig 多余键必须点名 extra:{}",
        e.message
    );

    let s = parse_scenario(&one_step("place = { material = 1 }")).expect("place 带 material 合法");
    assert_eq!(s.steps[0].action, Action::Place { material: Some(1) });

    let s_default = parse_scenario(&one_step("place = {}")).expect("place 省去 material 合法");
    assert_eq!(s_default.steps[0].action, Action::Place { material: None });

    let e = err(&one_step("place = { material = 1, extra = 2 }"));
    assert_eq!(e.step, Some(1));
    assert!(
        e.message.contains("extra"),
        "place 多余键必须点名 extra:{}",
        e.message
    );

    let e = err(&one_step("select_material = {}"));
    assert_eq!(e.step, Some(1));
    assert!(
        e.message.contains("index"),
        "select_material 缺 index 必须点名报错:{}",
        e.message
    );

    let s_sel = parse_scenario(&one_step("select_material = { index = 2 }"))
        .expect("select_material 带 index 合法");
    assert_eq!(s_sel.steps[0].action, Action::SelectMaterial { index: 2 });
}

/// REQ-013 C05 T-B(1):`stall_render` 参数校验(ms 必填、整数且 1..=5000)。
#[test]
fn c05_tb_parse_stall_render() {
    let s = parse_scenario(&one_step("stall_render = { ms = 800 }"))
        .expect("stall_render = { ms = 800 } 应合法解析");
    assert_eq!(s.steps.len(), 1);
    assert_eq!(s.steps[0].action, Action::StallRender { ms: 800 });

    let s_min = parse_scenario(&one_step("stall_render = { ms = 1 }")).expect("ms = 1 下边界合法");
    assert_eq!(s_min.steps[0].action, Action::StallRender { ms: 1 });

    let s_max =
        parse_scenario(&one_step("stall_render = { ms = 5000 }")).expect("ms = 5000 上边界合法");
    assert_eq!(s_max.steps[0].action, Action::StallRender { ms: 5000 });

    let e_missing = err(&one_step("stall_render = {}"));
    assert_eq!(e_missing.step, Some(1));
    assert!(
        e_missing.message.contains("ms"),
        "缺 ms 必须点名报错:{}",
        e_missing.message
    );

    let e_zero = err(&one_step("stall_render = { ms = 0 }"));
    assert_eq!(e_zero.step, Some(1));
    assert!(
        e_zero.message.contains("1..=5000"),
        "ms=0 越界必须报错:{}",
        e_zero.message
    );

    let e_over = err(&one_step("stall_render = { ms = 5001 }"));
    assert_eq!(e_over.step, Some(1));
    assert!(
        e_over.message.contains("1..=5000"),
        "ms=5001 越界必须报错:{}",
        e_over.message
    );

    let e_str = err(&one_step("stall_render = { ms = \"x\" }"));
    assert_eq!(e_str.step, Some(1));
    assert!(
        e_str.message.contains("整数"),
        "ms=\"x\" 非整数必须报错:{}",
        e_str.message
    );

    let e_extra = err(&one_step("stall_render = { ms = 800, extra = 1 }"));
    assert_eq!(e_extra.step, Some(1));
    assert!(
        e_extra.message.contains("extra"),
        "多余键必须点名报错:{}",
        e_extra.message
    );
}

/// REQ-013 C07 §1.3:`outline` 参数校验(on 必填、布尔值、拒绝未知键)。
#[test]
fn c07_parse_outline() {
    let s = parse_scenario(&one_step("outline = { on = true }"))
        .expect("outline = { on = true } 应合法解析");
    assert_eq!(s.steps.len(), 1);
    assert_eq!(s.steps[0].action, Action::Outline { on: true });

    let s_off = parse_scenario(&one_step("outline = { on = false }"))
        .expect("outline = { on = false } 应合法解析");
    assert_eq!(s_off.steps[0].action, Action::Outline { on: false });

    let e_missing = err(&one_step("outline = {}"));
    assert_eq!(e_missing.step, Some(1));
    assert!(
        e_missing.message.contains("outline"),
        "缺 on 错误消息应含 outline:{}",
        e_missing.message
    );

    let e_type = err(&one_step("outline = { on = 3 }"));
    assert_eq!(e_type.step, Some(1));

    let e_extra = err(&one_step("outline = { on = true, extra = 1 }"));
    assert_eq!(e_extra.step, Some(1));
    assert!(
        e_extra.message.contains("extra"),
        "多余键必须点名报错:{}",
        e_extra.message
    );
}
