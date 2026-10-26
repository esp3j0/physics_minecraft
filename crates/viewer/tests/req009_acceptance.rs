//! REQ-009 验收用例(viewer 部分;总架构师编写;issue #18)。
//!
//! 约定同 `crates/e2e/tests/req009_acceptance.rs`:实施方只去掉 `#[ignore]`,不得修改断言。

use types::glam::Vec3;
use types::{BlockId, BlockPos, EntityId};
use viewer::mesh::block_color;
use viewer::script::{
    Action, CameraPose, Engine, EngineCommand, FILL_BATCH_PER_FRAME, FILL_MAX_CELLS, FrameInput,
    FrameOutput, StepOutcome, WorldQuery, parse_scenario,
};
use voxel::builtin;

/// 假世界:全部已装载、全部空气(本文件只测命令展开,不测检查项)。
struct World;

impl WorldQuery for World {
    fn live_entities(&self) -> Vec<(EntityId, Vec3)> {
        Vec::new()
    }
    fn block_at(&self, _pos: BlockPos) -> Option<BlockId> {
        Some(builtin::AIR)
    }
    fn chunks_loaded(&self) -> usize {
        9
    }
}

const HEADER: &str = "name = \"t\"\ndescription = \"d\"\n";

fn engine(steps: &str) -> Engine {
    let scenario = parse_scenario(&format!("{HEADER}{steps}")).expect("场景应可解析");
    Engine::new(
        scenario,
        CameraPose {
            pos: Vec3::new(0.0, -50.0, 0.0),
            yaw: 0.0,
            pitch: 0.0,
        },
    )
}

fn frame(e: &mut Engine, tick: u64) -> FrameOutput {
    e.frame(&FrameInput {
        dt: 0.016,
        server_tick: tick,
        world: &World,
        spawned: &[],
    })
}

fn set_blocks(out: &FrameOutput) -> Vec<(BlockPos, BlockId)> {
    out.commands
        .iter()
        .map(|c| match c {
            EngineCommand::SetBlock { pos, block } => (*pos, *block),
            other => panic!("只应有 SetBlock,实为 {other:?}"),
        })
        .collect()
}

/// 盒内全部格,按 `BlockPos` 升序。
fn box_cells(from: (i32, i32, i32), to: (i32, i32, i32)) -> Vec<BlockPos> {
    let mut v = Vec::new();
    for x in from.0..=to.0 {
        for y in from.1..=to.1 {
            for z in from.2..=to.2 {
                v.push(BlockPos::new(x, y, z));
            }
        }
    }
    v.sort();
    v
}

/// AC-D1 `fill` 解析:两个对角顺序任意,规整为 from = 最小角、to = 最大角;新方块名可用;
/// 恰好 4096 格合法、4097 格及以上报错;缺参、未知方块、未知键、坐标不是整数都报带步骤序号的错误。
///
/// 会变红的实现:不规整对角(引擎展开为空集或负数个格);格数上限差一;`bedrock`/`sand` 不认;
/// 错误不带步骤序号;把未知方块当成空气。
#[test]
#[ignore = "REQ-009 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_d1_fill_parse() {
    assert_eq!((FILL_MAX_CELLS, FILL_BATCH_PER_FRAME), (4096, 64));
    let sc = parse_scenario(&format!(
        "{HEADER}[[step]]\nfill = {{ from = [3, -60, 5], to = [1, -58, 4], block = \"sand\" }}\n\
         [[step]]\nset_block = {{ pos = [0, -61, 0], block = \"bedrock\" }}\n"
    ))
    .expect("可解析");
    assert_eq!(
        sc.steps[0].action,
        Action::Fill {
            from: BlockPos::new(1, -60, 4),
            to: BlockPos::new(3, -58, 5),
            block: builtin::SAND,
        }
    );
    assert_eq!(
        sc.steps[1].action,
        Action::SetBlock {
            pos: BlockPos::new(0, -61, 0),
            block: builtin::BEDROCK,
        }
    );

    // 16×16×16 = 4096 恰好合法;17×16×16 超限。
    let ok = format!(
        "{HEADER}[[step]]\nfill = {{ from = [0, -60, 0], to = [15, -45, 15], block = \"stone\" }}\n"
    );
    assert!(parse_scenario(&ok).is_ok(), "4096 格合法");

    let bad_cases = [
        "fill = { from = [0, -60, 0], to = [16, -45, 15], block = \"stone\" }",
        "fill = { from = [0, -60, 0], block = \"stone\" }",
        "fill = { from = [0, -60, 0], to = [1, -60, 1], block = \"lava\" }",
        "fill = { from = [0, -60, 0], to = [1, -60, 1], block = \"stone\", speed = 1 }",
        "fill = { from = [0.5, -60, 0], to = [1, -60, 1], block = \"stone\" }",
    ];
    for case in bad_cases {
        let text = format!(
            "{HEADER}[[step]]\ncamera = {{ pos = [0.0, 0.0, 0.0], yaw = 0.0, pitch = 0.0 }}\n\
             [[step]]\n{case}\n"
        );
        let err = parse_scenario(&text).expect_err(case);
        assert_eq!(err.step, Some(2), "错误要带步骤序号:{case} → {err}");
    }
}

/// AC-D1 `fill` 展开:按 `BlockPos` 升序、每格恰好一次、每帧最多 64 条;最后一批发出的那一帧该步
/// 结束(结果 Ok,动作名 "fill"),后续瞬时步骤同一帧继续;不超过 64 格的填充一帧完成。
///
/// 会变红的实现:一帧发完 90 条(服务器收件箱会拒);顺序不是升序;漏格或重格;最后一批之后
/// 还要空等一帧;动作名不是 "fill"。
#[test]
#[ignore = "REQ-009 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_d1_fill_engine_batches() {
    let mut e = engine(
        "[[step]]\nfill = { from = [2, -51, 2], to = [0, -60, 0], block = \"stone\" }\n\
         [[step]]\nset_block = { pos = [9, -61, 9], block = \"dirt\" }\n",
    );
    let want: Vec<(BlockPos, BlockId)> = box_cells((0, -60, 0), (2, -51, 2))
        .into_iter()
        .map(|p| (p, builtin::STONE))
        .collect();
    assert_eq!(want.len(), 90);

    let first = frame(&mut e, 10);
    assert_eq!(set_blocks(&first), want[..64], "第一帧发前 64 格");
    assert!(first.finished.is_empty(), "还有 26 格没发,fill 未结束");

    let second = frame(&mut e, 11);
    let mut tail = want[64..].to_vec();
    tail.push((BlockPos::new(9, -61, 9), builtin::DIRT));
    assert_eq!(
        set_blocks(&second),
        tail,
        "第二帧发剩下 26 格,set_block 同帧继续"
    );
    let names: Vec<&str> = second.finished.iter().map(|r| r.action.as_str()).collect();
    assert_eq!(names, ["fill", "set_block"]);
    assert_eq!(second.finished[0].outcome, StepOutcome::Ok);
    assert_eq!(e.exit_code(), Some(0), "全部步骤完成");

    let mut small = engine(
        "[[step]]\nfill = { from = [0, -60, 0], to = [1, -60, 0], block = \"sand\" }\n\
         [[step]]\nset_block = { pos = [5, -60, 5], block = \"stone\" }\n",
    );
    let out = frame(&mut small, 10);
    assert_eq!(
        set_blocks(&out),
        [
            (BlockPos::new(0, -60, 0), builtin::SAND),
            (BlockPos::new(1, -60, 0), builtin::SAND),
            (BlockPos::new(5, -60, 5), builtin::STONE),
        ],
        "2 格填充当帧完成,后续步骤同帧"
    );
}

/// AC-D1 viewer 颜色:bedrock 与 sand 有各自的颜色(不是品红警示色、不透明、彼此及与 stone/dirt
/// 不同);bedrock 比 stone 暗;sand 偏黄;既有三色不变。
///
/// 会变红的实现:新方块落到品红警示色;bedrock 用了和 stone 一样的灰;sand 用了 dirt 的棕。
#[test]
#[ignore = "REQ-009 单元 01 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_d1_block_colors() {
    assert_eq!(block_color(builtin::AIR), [0.0, 0.0, 0.0, 0.0]);
    assert_eq!(block_color(builtin::STONE), [0.55, 0.55, 0.58, 1.0]);
    assert_eq!(block_color(builtin::DIRT), [0.45, 0.33, 0.22, 1.0]);
    let bedrock = block_color(builtin::BEDROCK);
    let sand = block_color(builtin::SAND);
    let magenta = [1.0, 0.0, 1.0, 1.0];
    for (name, c) in [("bedrock", bedrock), ("sand", sand)] {
        assert_ne!(c, magenta, "{name} 不能是警示色");
        assert_eq!(c[3], 1.0, "{name} 不透明");
        assert_ne!(c, block_color(builtin::STONE), "{name} 与 stone 区分");
        assert_ne!(c, block_color(builtin::DIRT), "{name} 与 dirt 区分");
    }
    let lum = |c: [f32; 4]| c[0] + c[1] + c[2];
    assert!(
        lum(bedrock) < lum(block_color(builtin::STONE)),
        "bedrock 比 stone 暗"
    );
    assert!(
        sand[0] > sand[2] + 0.2 && sand[1] > sand[2] + 0.15,
        "sand 偏黄:{sand:?}"
    );
}
