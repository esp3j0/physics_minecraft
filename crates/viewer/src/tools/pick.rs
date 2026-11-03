//! pick —— 准星拾取:相机位置沿视线做体素 DDA,命中第一个非空气格(REQ-011 §3.3)。
//!
//! 纯函数,零 macroquad;世界读取走 [`WorldQuery`]。坐标约定(与 types::coords
//! 一致):格 (x,y,z) 覆盖 [x,x+1)³,体素中心在 (x+0.5, y+0.5, z+0.5)。
//!
//! DDA 语义(AC-W1):
//! - 从 `origin` 沿 `dir`(内部单位化;零向量 = 无拾取)逐格推进;
//! - 命中第一个实心格(非空气)为**目标格**;进入该格时穿过的面为**命中面**,
//!   其外侧相邻格 = 目标格 + 面法向 = **放置格**;
//! - `block_at` 返回 `None`(chunk 未装载、世界 y 范围外)当空气,继续前进;
//! - 最远 `max_dist` 米(§3.3 定 48)无命中 = 无目标;
//! - 两轴同时跨界的平局:先 x、再 z、最后 y(轴序固定,结果确定);
//! - 起点就在实心格内:该格即目标格,**命中面约定** = 视线反方向的主轴面
//!   (放置格相应落在视线来路一侧)。这条是 §3.3 未覆盖情形的自行约定,
//!   锚定于此(有测试)。

use types::glam::Vec3;
use types::{BlockId, BlockPos};

use crate::mesh::FaceDir;
use crate::script::WorldQuery;

/// 一个拾取命中。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hit {
    /// 目标格(命中格)。
    pub block_pos: BlockPos,
    /// 目标格的方块(线框变色、面板显示用)。
    pub block: BlockId,
    /// 命中面(进入目标格时穿过的面;外法向见 [`FaceDir::normal`])。
    pub face: FaceDir,
}

impl Hit {
    /// 命中面外侧相邻格(放置格 = 目标格 + 面法向;REQ-011 §3.3)。
    pub fn place_pos(&self) -> BlockPos {
        let [nx, ny, nz] = self.face.normal();
        BlockPos::new(
            self.block_pos.x + nx as i32,
            self.block_pos.y + ny as i32,
            self.block_pos.z + nz as i32,
        )
    }
}

/// 体素 DDA 拾取(§3.3;AC-W1)。
pub fn pick(world: &dyn WorldQuery, origin: Vec3, dir: Vec3, max_dist: f32) -> Option<Hit> {
    if dir == Vec3::ZERO {
        return None;
    }
    let dir = dir.normalize();

    // 当前所在格(负方向 floor;格点在整数坐标)。
    let mut cell = BlockPos::new(
        origin.x.floor() as i32,
        origin.y.floor() as i32,
        origin.z.floor() as i32,
    );

    // 起点在实心格内:目标即此格,命中面 = 视线反方向主轴面(模块文档约定)。
    if let Some(block) = world.block_at(cell)
        && !block.is_air()
    {
        let face = main_axis_face(-dir.x, -dir.y, -dir.z);
        return Some(Hit {
            block_pos: cell,
            block,
            face,
        });
    }

    // 各轴:步进方向与到下一格边界的 t(射线 p = origin + t·dir,t ∈ [0, max_dist])。
    let step = [
        dir.x.signum() as i32,
        dir.y.signum() as i32,
        dir.z.signum() as i32,
    ];
    let mut t_max = [
        axis_t_max(origin.x, dir.x),
        axis_t_max(origin.y, dir.y),
        axis_t_max(origin.z, dir.z),
    ];
    let t_delta = [
        if dir.x != 0.0 {
            1.0 / dir.x.abs()
        } else {
            f32::INFINITY
        },
        if dir.y != 0.0 {
            1.0 / dir.y.abs()
        } else {
            f32::INFINITY
        },
        if dir.z != 0.0 {
            1.0 / dir.z.abs()
        } else {
            f32::INFINITY
        },
    ];

    loop {
        // 走 t 最小的那根轴跨入邻格;平局固定 x → z → y(确定序,模块文档)。
        let axis = if t_max[0] <= t_max[1] && t_max[0] <= t_max[2] {
            0
        } else if t_max[2] <= t_max[1] {
            2
        } else {
            1
        };
        if t_max[axis] > max_dist {
            return None; // 越过最远距离仍无命中
        }
        let (axis, s) = (axis, step[axis]);
        match axis {
            0 => cell.x += s,
            1 => cell.y += s,
            _ => cell.z += s,
        }
        t_max[axis] += t_delta[axis];

        // 未装载(None)当空气继续;命中第一个实心格。
        if let Some(block) = world.block_at(cell)
            && !block.is_air()
        {
            // 跨入方向的反向 = 命中面外法向:+x 跨入 → 穿过目标格的西面。
            let face = match (axis, s) {
                (0, s) if s > 0 => FaceDir::West,
                (0, _) => FaceDir::East,
                (1, s) if s > 0 => FaceDir::Bottom,
                (1, _) => FaceDir::Top,
                (_, s) if s > 0 => FaceDir::North,
                _ => FaceDir::South,
            };
            return Some(Hit {
                block_pos: cell,
                block,
                face,
            });
        }
    }
}

/// 主轴面:|(a,b,c)| 最大的分量的符号定面(平局 x > y > z;分量全零不定,调用方保证非零)。
fn main_axis_face(a: f32, b: f32, c: f32) -> FaceDir {
    let (aa, ab, ac) = (a.abs(), b.abs(), c.abs());
    if aa >= ab && aa >= ac {
        if a > 0.0 {
            FaceDir::East
        } else {
            FaceDir::West
        }
    } else if ab >= ac {
        if b > 0.0 {
            FaceDir::Top
        } else {
            FaceDir::Bottom
        }
    } else if c > 0.0 {
        FaceDir::North
    } else {
        FaceDir::South
    }
}

/// 沿某轴到下一格边界的 t(t ≥ 0;零分量 = 无穷远)。
fn axis_t_max(o: f32, d: f32) -> f32 {
    if d == 0.0 {
        f32::INFINITY
    } else if d > 0.0 {
        (o.floor() + 1.0 - o) / d
    } else {
        (o - o.floor()) / -d
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    use types::{CHUNK_SIZE, ChunkPos, WORLD_Y_MAX, WORLD_Y_MIN, WorldSeed};
    use voxel::{Chunk, Superflat, WorldGenerator};

    const DIRT: BlockId = voxel::builtin::DIRT;
    const STONE: BlockId = voxel::builtin::STONE;
    const BEDROCK: BlockId = voxel::builtin::BEDROCK;
    /// 超平坦最高实心格 y(层表 bedrock×1 + dirt×3,顶面 world y = −61)。
    const TOP: i32 = WORLD_Y_MIN + 4 - 1; // -61

    /// 测试世界:列出的 chunk 生成超平坦(可再用 blocks 覆写单格;覆写所在
    /// chunk 不在列表里时造一个空气 chunk 承接)。未列出的 chunk = 未装载。
    struct FakeWorld {
        chunks: BTreeMap<ChunkPos, Chunk>,
    }

    impl FakeWorld {
        fn new(loaded: &[ChunkPos], blocks: &[(i32, i32, i32, BlockId)]) -> Self {
            let mut w = Self {
                chunks: BTreeMap::new(),
            };
            for &pos in loaded {
                w.chunks
                    .insert(pos, Superflat::classic().generate(WorldSeed(0), pos));
            }
            for &(x, y, z, b) in blocks {
                let pos = BlockPos::new(x, y, z).chunk_of();
                let chunk = w.chunks.entry(pos).or_insert_with(Chunk::air);
                chunk.set(
                    x.rem_euclid(CHUNK_SIZE) as usize,
                    (y - WORLD_Y_MIN) as usize,
                    z.rem_euclid(CHUNK_SIZE) as usize,
                    b,
                );
            }
            w
        }

        /// 九宫格超平坦(chunk (−1,−1)..(1,1) 全装载)。
        fn superflat(blocks: &[(i32, i32, i32, BlockId)]) -> Self {
            let mut loaded = Vec::new();
            for cx in -1..=1 {
                for cz in -1..=1 {
                    loaded.push(ChunkPos::new(cx, cz));
                }
            }
            Self::new(&loaded, blocks)
        }
    }

    impl WorldQuery for FakeWorld {
        fn live_entities(&self) -> Vec<(types::EntityId, Vec3)> {
            vec![]
        }

        fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
            if pos.y < WORLD_Y_MIN || pos.y > WORLD_Y_MAX {
                return None;
            }
            self.chunks.get(&pos.chunk_of()).map(|c| {
                c.get(
                    pos.x.rem_euclid(CHUNK_SIZE) as usize,
                    (pos.y - WORLD_Y_MIN) as usize,
                    pos.z.rem_euclid(CHUNK_SIZE) as usize,
                )
            })
        }

        fn chunks_loaded(&self) -> usize {
            self.chunks.len()
        }
    }

    /// AC-W1 垂直下视:命中地面顶格 (0,−61,0),命中面 Top,放置格 = 上方一格。
    #[test]
    fn pick_hits_ground_straight_down() {
        let w = FakeWorld::superflat(&[]);
        let hit = pick(
            &w,
            Vec3::new(0.3, -50.0, 0.7),
            Vec3::new(0.0, -1.0, 0.0),
            48.0,
        )
        .expect("垂直下视必命中地面");
        assert_eq!(hit.block_pos, BlockPos::new(0, TOP, 0));
        assert_eq!(hit.block, DIRT);
        assert_eq!(hit.face, FaceDir::Top);
        assert_eq!(
            hit.place_pos(),
            BlockPos::new(0, TOP + 1, 0),
            "命中面外侧格 = 目标格 + 面法向"
        );
    }

    /// AC-W1 斜向:视线自斜上方打向凸出地面一格的石块,命中其顶面(手算
    /// DDA:y 逐格下降三次落在 (5,TOP+1,5);跨入方向 −y → 面 Top)。
    #[test]
    fn pick_diagonal_hits_top_face() {
        let w = FakeWorld::superflat(&[(5, TOP + 1, 5, STONE)]);
        // 起点 (4.2,−56.6,4.2),方向 (0.4,−1,0.4) 归一:dy 主导,每 ~1.15 m
        // 降一格、每 ~2.87 m 进一格 x/z——第 5 次跨入落在石块格。
        let hit = pick(
            &w,
            Vec3::new(4.2, TOP as f32 + 4.4, 4.2),
            Vec3::new(0.4, -1.0, 0.4),
            48.0,
        )
        .expect("斜向命中凸块");
        assert_eq!(hit.block_pos, BlockPos::new(5, TOP + 1, 5), "目标格 = 凸块");
        assert_eq!(hit.block, STONE);
        assert_eq!(hit.face, FaceDir::Top);
        assert_eq!(hit.place_pos(), BlockPos::new(5, TOP + 2, 5));
    }

    /// AC-W1 侧面命中:平视打 2 高石柱,命中柱身西面。
    #[test]
    fn pick_hits_side_face() {
        let w = FakeWorld::superflat(&[(10, TOP + 1, 0, STONE), (10, TOP + 2, 0, STONE)]);
        let hit = pick(
            &w,
            Vec3::new(6.5, TOP as f32 + 1.5, 0.5),
            Vec3::new(1.0, 0.0, 0.0),
            48.0,
        )
        .expect("平视命中柱身");
        assert_eq!(hit.block_pos, BlockPos::new(10, TOP + 1, 0));
        assert_eq!(hit.block, STONE);
        assert_eq!(hit.face, FaceDir::West, "+x 跨入 → 命中柱的西面");
        assert_eq!(hit.place_pos(), BlockPos::new(9, TOP + 1, 0));
    }

    /// AC-W1 穿过 chunk 边界:斜向缓降,命中 chunk (1,1) 的地面顶格
    /// (16,−61,16)(手算:x/z 交替各 16 步后 y 降一格落在顶面)。
    #[test]
    fn pick_crosses_chunk_boundary() {
        let w = FakeWorld::superflat(&[]);
        let hit = pick(
            &w,
            Vec3::new(0.5, TOP as f32 + 2.0, 0.5),
            Vec3::new(16.0, -1.0, 16.0),
            48.0,
        )
        .expect("跨界命中");
        assert_eq!(
            hit.block_pos,
            BlockPos::new(16, TOP, 16),
            "目标格在 chunk (1,1)"
        );
        assert_eq!(hit.block, DIRT);
        assert_eq!(hit.place_pos(), BlockPos::new(16, TOP + 1, 16));
    }

    /// AC-W1 贴棱:起点恰在格界 x=0 上、斜向下看——首个 t_max 为 0(立即
    /// 跨步),平局先 x:第 1 步进 (−1,TOP+1,0),第 2 步 y 降一格命中
    /// (−1,−61,0)。钉死平局策略(先 x)与立即跨步不挂死。
    #[test]
    fn pick_ray_on_grid_boundary_is_deterministic() {
        let w = FakeWorld::superflat(&[]);
        let hit = pick(
            &w,
            Vec3::new(0.0, TOP as f32 + 1.5, 0.5),
            Vec3::new(-1.0, -1.0, 0.0),
            48.0,
        )
        .expect("贴棱射线正常命中");
        assert_eq!(
            hit.block_pos,
            BlockPos::new(-1, TOP, 0),
            "先 x 后 y 的跨入序"
        );
        assert_eq!(hit.block, DIRT);
        assert_eq!(hit.face, FaceDir::Top);
    }

    /// AC-W1 48 m 外无命中:抬头、平视 10 m 截断、零方向,均 None。
    #[test]
    fn pick_returns_none_beyond_max_dist() {
        let w = FakeWorld::superflat(&[]);
        assert!(
            pick(
                &w,
                Vec3::new(0.0, TOP as f32 + 2.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                48.0
            )
            .is_none(),
            "抬头永无命中"
        );
        // y=TOP+1 整行全空;缓降 10 m 内到不了地面。
        assert!(
            pick(
                &w,
                Vec3::new(0.5, TOP as f32 + 1.5, 0.5),
                Vec3::new(1.0, -0.05, 0.0),
                10.0
            )
            .is_none(),
            "10 m 内无命中"
        );
        assert!(
            pick(&w, Vec3::new(0.0, 0.0, 0.0), Vec3::ZERO, 48.0).is_none(),
            "零方向 = 无拾取"
        );
    }

    /// AC-W1 未装载区域当空气:① 整行穿过未装载 chunk(buggy 实现把 None
    /// 当实心会在 x=±16 提前命中)→ None;② 视线穿过未装载的 chunk (1,0)
    /// 继续前进,在装载的 chunk (2,0) 里落地(手算:缓降斜率 1/64,x 走 32
    /// 格时 y 降一格,恰在 (32,−61,0) 命中)。
    #[test]
    fn pick_treats_unloaded_as_air() {
        let only_center = FakeWorld::new(&[ChunkPos::new(0, 0)], &[]);
        for dir in [Vec3::new(1.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)] {
            assert!(
                pick(
                    &only_center,
                    Vec3::new(0.5, TOP as f32 + 1.5, 0.5),
                    dir,
                    48.0
                )
                .is_none(),
                "方向 {dir}:未装载区当空气,平视整行无命中"
            );
        }

        // chunk (1,0) 缺席,(0,0) 与 (2,0) 装载:视线越过空洞在另一侧落地。
        let gap = FakeWorld::new(&[ChunkPos::new(0, 0), ChunkPos::new(2, 0)], &[]);
        let hit = pick(
            &gap,
            Vec3::new(0.5, TOP as f32 + 1.5, 0.5),
            Vec3::new(64.0, -1.0, 0.0),
            48.0,
        )
        .expect("穿过未装载区后命中");
        assert_eq!(hit.block_pos, BlockPos::new(32, TOP, 0), "落地在空洞对侧");
        assert_eq!(hit.block, DIRT);
        assert_eq!(hit.face, FaceDir::Top);
    }

    /// AC-W1 起点在实心格内:返回该格,命中面 = 视线反方向主轴面,放置格在
    /// 来路侧(自行约定,模块文档锚定)。
    #[test]
    fn pick_start_inside_solid_returns_that_cell() {
        let w = FakeWorld::superflat(&[]);
        // 钻进地里向 +x 看:目标 = 所在格,面 = West(反方向主轴)。
        let hit = pick(
            &w,
            Vec3::new(3.4, TOP as f32 + 0.5, 8.9),
            Vec3::new(1.0, 0.0, 0.0),
            48.0,
        )
        .expect("实心格内起点返回该格");
        assert_eq!(hit.block_pos, BlockPos::new(3, TOP, 8));
        assert_eq!(hit.block, DIRT);
        assert_eq!(hit.face, FaceDir::West, "视线 +x → 反方向主轴面 = 西面");
        assert_eq!(hit.place_pos(), BlockPos::new(2, TOP, 8));

        // 斜向主轴:俯视为主 → 反方向面 = Top。
        let hit = pick(
            &w,
            Vec3::new(6.2, TOP as f32 + 0.2, 6.8),
            Vec3::new(0.2, -1.0, 0.2),
            48.0,
        )
        .expect("斜向实心起点");
        assert_eq!(hit.block_pos, BlockPos::new(6, TOP, 6));
        assert_eq!(hit.face, FaceDir::Top, "视线主轴向下 → 来路面 = 顶面");
    }

    /// AC-W1 命中基岩:目标格与方块值正确(线框变色按 block 判定)。
    #[test]
    fn pick_reports_bedrock_hit() {
        let w = FakeWorld::superflat(&[(7, TOP, 7, BEDROCK)]);
        let hit = pick(
            &w,
            Vec3::new(7.5, TOP as f32 + 3.0, 7.5),
            Vec3::new(0.0, -1.0, 0.0),
            48.0,
        )
        .expect("命中替换的基岩格");
        assert_eq!(hit.block_pos, BlockPos::new(7, TOP, 7));
        assert_eq!(hit.block, BEDROCK);
    }
}
