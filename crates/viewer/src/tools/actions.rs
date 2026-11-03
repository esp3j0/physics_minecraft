//! actions —— 工具动作 → 命令(REQ-011 §3.3 / §3.4;AC-W5 批语义)。
//!
//! 纯函数层:输入(工具状态、拾取命中、世界查询)→ [`ToolAction`](要发的
//! 命令 + 供提示的原因)。render.rs 只负责把 Action 经 ControlHandle 发出。
//! 批语义在这里成形:预制结构 = 一条批(同拍落地),单方块 = 单条。

use messaging::SimCommand;
use types::{BlockId, BlockPos};

use crate::script::WorldQuery;

use super::pick::Hit;
use super::prefab::{ExpandError, Prefab, expand, orientation_from_yaw};
use super::{Material, Placeable, ToolState};

/// 一次工具动作的产物(render 层原样发出)。
#[derive(Debug, Clone, PartialEq)]
pub enum ToolAction {
    /// 单条命令(左键拆除、右键单方块)。
    Single(SimCommand),
    /// 同帧命令批(右键预制结构;REQ-010 终审返修 1 的原子编辑语义——
    /// 一批 = 一次原子编辑,服务器同一拍落地,承载判定不见半个结构)。
    Batch(Vec<SimCommand>),
    /// 不发命令(动作被拒绝;`reason` 进 HUD 提示行,3 秒)。
    Rejected(String),
}

/// `T` 循环的放置物全序(§3.1:单方块 → 土墙 → 石柱 → 石桥 → 土门楣门框
/// → 石 7 字 → 单方块;表驱动,循环实现与测试共用一份序)。
pub const PLACEABLE_CYCLE: [Placeable; 6] = [
    Placeable::SingleBlock,
    Placeable::Prefab(Prefab::DirtWall),
    Placeable::Prefab(Prefab::StonePillar),
    Placeable::Prefab(Prefab::StoneBridge),
    Placeable::Prefab(Prefab::DirtLintel),
    Placeable::Prefab(Prefab::StoneCantilever),
];

/// `1`–`4`:选当前材料(键序 0–3;REQ-011 §3.1)。
pub fn select_material(state: &mut ToolState, index: usize) {
    state.material = Material::from_index(index);
}

/// 左键:拆除目标格(§3.3)。无目标 → 拒绝提示。
#[must_use]
pub fn dig(target: Option<&Hit>) -> ToolAction {
    match target {
        None => ToolAction::Rejected(super::notices::NO_TARGET.to_owned()),
        Some(hit) => ToolAction::Single(SimCommand::SetBlock {
            pos: hit.block_pos,
            block: BlockId::AIR,
        }),
    }
}

/// 右键(§3.3 / §3.4):
/// - 无目标 → 拒绝(无目标);
/// - 单方块:命中面外侧格发 `SetBlock` 当前材料;该格在世界 y 范围外不发;
///   已装载且非空气 → 拒绝(§3.3"不会发生,防御");未装载放行(服务器
///   现有规则处理);
/// - 预制结构:锚点 = 命中面外侧格、朝向 = 相机水平朝向取整(同 §3.2 的
///   u / w),整个结构作为**一条批**发出;任何一格越界 → 整构拒绝。
#[must_use]
pub fn place(
    state: &ToolState,
    target: Option<&Hit>,
    yaw_deg: f32,
    world: &dyn WorldQuery,
) -> ToolAction {
    let Some(hit) = target else {
        return ToolAction::Rejected(super::notices::NO_TARGET.to_owned());
    };
    let pos = hit.place_pos();
    match state.placeable {
        Placeable::SingleBlock => {
            if pos.y < types::WORLD_Y_MIN || pos.y > types::WORLD_Y_MAX {
                return ToolAction::Rejected(format!(
                    "放置格 ({},{},{}) 在世界 y 范围外,未放置",
                    pos.x, pos.y, pos.z
                ));
            }
            if let Some(existing) = world.block_at(pos)
                && !existing.is_air()
            {
                return ToolAction::Rejected(format!(
                    "放置格 ({},{},{}) 已有方块,未放置",
                    pos.x, pos.y, pos.z
                ));
            }
            ToolAction::Single(SimCommand::SetBlock {
                pos,
                block: state.material.block(),
            })
        }
        Placeable::Prefab(prefab) => {
            let orientation = orientation_from_yaw(yaw_deg);
            match expand(
                prefab,
                pos,
                orientation,
                types::WORLD_Y_MIN,
                types::WORLD_Y_MAX,
            ) {
                Ok(cells) => ToolAction::Batch(
                    cells
                        .into_iter()
                        .map(|(pos, block)| SimCommand::SetBlock { pos, block })
                        .collect(),
                ),
                Err(err) => ToolAction::Rejected(out_of_world_message(prefab, pos, err)),
            }
        }
    }
}

/// 越界拒绝文案(§3.5.3 第 5 条"结构越界")。
fn out_of_world_message(prefab: Prefab, anchor: BlockPos, err: ExpandError) -> String {
    let detail = match err {
        ExpandError::BelowWorld { y_min } => format!("最低格 y={y_min} 低于世界底"),
        ExpandError::AboveWorld { y_max } => format!("最高格 y={y_max} 高于世界顶"),
    };
    format!(
        "{}越界(锚点 ({}, {}, {})):{}",
        prefab.name(),
        anchor.x,
        anchor.y,
        anchor.z,
        detail
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::FaceDir;
    use std::collections::BTreeMap;
    use types::{CHUNK_SIZE, ChunkPos, WorldSeed};
    use voxel::{Chunk, Superflat, WorldGenerator};

    const DIRT: BlockId = voxel::builtin::DIRT;
    const STONE: BlockId = voxel::builtin::STONE;
    /// 超平坦最高实心格 y。
    const TOP: i32 = types::WORLD_Y_MIN + 4 - 1;

    /// 最小世界:九宫格超平坦 + 补写格(place 的防御检查用)。
    struct FakeWorld {
        chunks: BTreeMap<ChunkPos, Chunk>,
    }

    impl FakeWorld {
        fn superflat(blocks: &[(i32, i32, i32, BlockId)]) -> Self {
            let mut chunks = BTreeMap::new();
            for cx in -1..=1 {
                for cz in -1..=1 {
                    chunks.insert(
                        ChunkPos::new(cx, cz),
                        Superflat::classic().generate(WorldSeed(0), ChunkPos::new(cx, cz)),
                    );
                }
            }
            for &(x, y, z, b) in blocks {
                let pos = BlockPos::new(x, y, z).chunk_of();
                let chunk = chunks.entry(pos).or_insert_with(Chunk::air);
                chunk.set(
                    x.rem_euclid(CHUNK_SIZE) as usize,
                    (y - types::WORLD_Y_MIN) as usize,
                    z.rem_euclid(CHUNK_SIZE) as usize,
                    b,
                );
            }
            Self { chunks }
        }
    }

    impl WorldQuery for FakeWorld {
        fn live_entities(&self) -> Vec<(types::EntityId, types::glam::Vec3)> {
            vec![]
        }

        fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
            if pos.y < types::WORLD_Y_MIN || pos.y > types::WORLD_Y_MAX {
                return None;
            }
            self.chunks.get(&pos.chunk_of()).map(|c| {
                c.get(
                    pos.x.rem_euclid(CHUNK_SIZE) as usize,
                    (pos.y - types::WORLD_Y_MIN) as usize,
                    pos.z.rem_euclid(CHUNK_SIZE) as usize,
                )
            })
        }

        fn chunks_loaded(&self) -> usize {
            self.chunks.len()
        }
    }

    /// 构造一个命中(目标格 + 命中面)。
    fn hit_at(x: i32, y: i32, z: i32, block: BlockId, face: FaceDir) -> Hit {
        Hit {
            block_pos: BlockPos::new(x, y, z),
            block,
            face,
        }
    }

    /// 左键:对目标格发 SetBlock AIR;无目标 → 拒绝。
    #[test]
    fn dig_sends_air_and_rejects_without_target() {
        let hit = hit_at(4, TOP, 2, DIRT, FaceDir::Top);
        assert_eq!(
            dig(Some(&hit)),
            ToolAction::Single(SimCommand::SetBlock {
                pos: BlockPos::new(4, TOP, 2),
                block: BlockId::AIR,
            }),
            "左键 = 目标格置空气"
        );
        assert_eq!(
            dig(None),
            ToolAction::Rejected("无目标".to_owned()),
            "无目标拒绝"
        );
    }

    /// 右键单方块:命中面外侧格 = 当前材料;防御(已有方块 / 越界)不发。
    #[test]
    fn place_single_block_targets_place_pos() {
        let world = FakeWorld::superflat(&[]);
        let state = ToolState {
            material: Material::Stone,
            ..ToolState::default()
        };

        let hit = hit_at(0, TOP, 0, DIRT, FaceDir::Top);
        assert_eq!(
            place(&state, Some(&hit), 0.0, &world),
            ToolAction::Single(SimCommand::SetBlock {
                pos: BlockPos::new(0, TOP + 1, 0),
                block: STONE,
            }),
            "单方块 = 命中面外侧格置当前材料"
        );

        // 防御:放置格已被占(顶层已有石块)。
        let world_full = FakeWorld::superflat(&[(0, TOP + 1, 0, STONE)]);
        assert!(
            matches!(
                place(&state, Some(&hit), 0.0, &world_full),
                ToolAction::Rejected(_)
            ),
            "放置格已有方块 → 拒绝(防御)"
        );

        // 越界:目标格贴世界顶、面 Top → 放置格 y = Y_MAX+1,拒绝。
        let hit_top = hit_at(0, types::WORLD_Y_MAX, 0, STONE, FaceDir::Top);
        assert!(
            matches!(
                place(&state, Some(&hit_top), 0.0, &world),
                ToolAction::Rejected(_)
            ),
            "放置格在世界顶外 → 拒绝"
        );

        // 无目标。
        assert!(matches!(
            place(&state, None, 0.0, &world),
            ToolAction::Rejected(_)
        ));
    }

    /// AC-W5:右键预制结构 = **一条批**(格数条 SetBlock);yaw 取整进格
    /// 坐标;越界整构拒绝。
    #[test]
    fn place_prefab_is_single_batch() {
        let world = FakeWorld::superflat(&[]);
        let pillar_state = ToolState {
            placeable: Placeable::Prefab(crate::tools::Prefab::StonePillar),
            ..ToolState::default()
        };

        // 地面顶格 (0,TOP,0),面 Top → 锚点 (0,TOP+1,0);yaw = 0° → w=+x、u=+z。
        let hit = hit_at(0, TOP, 0, DIRT, FaceDir::Top);
        let action = place(&pillar_state, Some(&hit), 0.0, &world);
        let ToolAction::Batch(commands) = action else {
            panic!("预制结构必须是一条批,实为 {action:?}");
        };
        assert_eq!(commands.len(), 4, "石柱 4 格");
        let want: Vec<SimCommand> = (0..=3)
            .map(|v| SimCommand::SetBlock {
                pos: BlockPos::new(3, TOP + 1 + v, 0),
                block: STONE,
            })
            .collect();
        assert_eq!(commands, want, "柱在 x=3(w=3·+x),z=0(u=0),v=0..3");

        // 越界:目标格贴世界顶、面 Top → 土墙锚点(命中面外侧格)的顶格
        // (v=4)越出世界顶,整构拒绝。
        let wall_state = ToolState {
            placeable: Placeable::Prefab(crate::tools::Prefab::DirtWall),
            ..ToolState::default()
        };
        let hit_top = hit_at(0, types::WORLD_Y_MAX - 3, 0, DIRT, FaceDir::Top);
        let rejected = place(&wall_state, Some(&hit_top), 0.0, &world);
        assert!(matches!(rejected, ToolAction::Rejected(_)), "整构拒绝");
        if let ToolAction::Rejected(msg) = rejected {
            assert!(msg.contains("土墙"), "拒绝文案含结构名:{msg}");
        }
    }

    /// AC-W5:T 循环顺序与 §3.1 一致(单方块 → 土墙 → 石柱 → 石桥 → 门框
    /// → 石 7 字 → 单方块),`cycle_placeable` 沿 PLACEABLE_CYCLE 推进。
    #[test]
    fn placeable_cycle_order() {
        let mut state = ToolState::default();
        assert_eq!(state.placeable, Placeable::SingleBlock, "默认单方块");
        for want in PLACEABLE_CYCLE {
            assert_eq!(state.placeable, want, "循环到位");
            crate::tools::cycle_placeable(&mut state);
        }
        assert_eq!(state.placeable, Placeable::SingleBlock, "循环回单方块");
    }

    /// 材料 1–4 与 BlockId 对齐(voxel::builtin 注册序)。
    #[test]
    fn material_index_maps_builtin_ids() {
        assert_eq!(Material::from_index(0).block(), voxel::builtin::DIRT);
        assert_eq!(Material::from_index(1).block(), voxel::builtin::STONE);
        assert_eq!(Material::from_index(2).block(), voxel::builtin::SAND);
        assert_eq!(Material::from_index(3).block(), voxel::builtin::BEDROCK);
        // 越界序号回绕(防御;接线处不产生 >3,纯函数自守)。
        assert_eq!(Material::from_index(4), Material::Dirt);
    }
}
