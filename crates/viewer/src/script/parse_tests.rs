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
        } => {
            assert!(checks.is_empty());
            assert_eq!(*timeout_ticks, 5);
        }
        other => panic!("应为 wait_until:{other:?}"),
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
