//! 世界生成器(REQ-002 §4.1):纯函数,同 `(seed, pos)` 位级确定(AC-G1)。
//!
//! 纪律(蓝图 §4):无 IO/随机/墙钟;哈希用自实现 splitmix64,不引 rand 依赖。
//! 内置方块表只有 air(0)/stone(1)/dirt(2)(`voxel::SimpleRegistry::builtin`),
//! 层表只允许取其中的方块。

use types::{BlockId, ChunkPos, WorldSeed};

use crate::chunk::{CHUNK_X, CHUNK_Y, CHUNK_Z, Chunk};

const STONE: BlockId = BlockId(1);
const DIRT: BlockId = BlockId(2);

/// 世界生成器:纯函数,无 IO/随机/墙钟(蓝图 §4 纪律)。
/// 同 `(seed, pos)` 必须产出位级相同的 Chunk(AC-G1)。
pub trait WorldGenerator: Send {
    fn generate(&self, seed: WorldSeed, pos: ChunkPos) -> Chunk;
}

/// 超平坦:自下而上层表(thickness, BlockId);内置方块表内取值。
#[derive(Debug, Clone)]
pub struct Superflat {
    pub layers: Vec<(usize, BlockId)>,
}

impl Superflat {
    /// 经典:v1 内置方块 = stone×1 + dirt×3(方块表扩充后自然可加 grass)。
    pub fn classic() -> Self {
        Self {
            layers: vec![
                (1, BlockId(1)), // stone × 1
                (3, BlockId(2)), // dirt  × 3
            ],
        }
    }
}

/// 种子化散布:超平坦基底 + 坐标哈希散布的方块(测试/RL 简单多样性)。
#[derive(Debug, Clone)]
pub struct SeededScatter {
    pub base: Superflat,
    /// 散布概率密度,0.0-1.0。
    pub density: f32,
}

impl SeededScatter {
    pub fn new(density: f32) -> Self {
        Self {
            base: Superflat::classic(),
            density,
        }
    }
}

impl WorldGenerator for Superflat {
    fn generate(&self, _seed: WorldSeed, _pos: ChunkPos) -> Chunk {
        let total: usize = self.layers.iter().map(|(t, _)| *t).sum();
        assert!(
            total <= CHUNK_Y,
            "层表总厚 {total} 超过 chunk 高度 {CHUNK_Y}(一致性违规,fail-stop)"
        );
        let mut chunk = Chunk::air();
        // 局部 y=0 对应世界最低层(WORLD_Y_MIN);层表自下而上依次铺放。
        let mut y = 0;
        for &(thickness, block) in &self.layers {
            for _ in 0..thickness {
                for z in 0..CHUNK_Z {
                    for x in 0..CHUNK_X {
                        chunk.set(x, y, z, block);
                    }
                }
                y += 1;
            }
        }
        chunk
    }
}

impl WorldGenerator for SeededScatter {
    fn generate(&self, seed: WorldSeed, pos: ChunkPos) -> Chunk {
        let mut chunk = self.base.generate(seed, pos);
        // 散布层仅顶面上一格(层表总厚处);density 出界时比较自然退化为
        // 恒散布(>1)或不散布(<0),无需特判。
        let top = self.base.layers.iter().map(|(t, _)| *t).sum::<usize>();
        if top >= CHUNK_Y {
            return chunk; // 基底已满高,无顶面上一格可散布
        }
        let y = top;
        for z in 0..CHUNK_Z {
            for x in 0..CHUNK_X {
                let h = block_hash(seed, pos, x, y, z);
                // f64 域比较:f32 density 精确提升,比值取高 53 位无精度损失。
                if scatter_ratio(h) < f64::from(self.density) {
                    // 顶层取一比特选散布方块(内置表:stone/dirt)。
                    let block = if h >> 63 == 0 { STONE } else { DIRT };
                    chunk.set(x, y, z, block);
                }
            }
        }
        chunk
    }
}

/// splitmix64 单步混合(自实现,确定性,不引 rand 依赖,蓝图 §4)。
fn splitmix_mix(v: u64) -> u64 {
    let mut z = v.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// (seed, chunk 坐标, 局部坐标) → 64 位确定性哈希。
/// 有符号坐标按补码扩展混入,负 chunk 坐标同样确定。
fn block_hash(seed: WorldSeed, pos: ChunkPos, x: usize, y: usize, z: usize) -> u64 {
    let mut s = seed.0;
    s = splitmix_mix(s ^ (i64::from(pos.x) as u64));
    s = splitmix_mix(s ^ (i64::from(pos.z) as u64));
    s = splitmix_mix(s ^ (x as u64));
    s = splitmix_mix(s ^ (y as u64));
    s = splitmix_mix(s ^ (z as u64));
    s
}

/// 哈希 → [0,1) 比值:f64 精确表示,取高 53 位。
fn scatter_ratio(h: u64) -> f64 {
    (h >> 11) as f64 / (1u64 << 53) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_chunk_bitwise() {
        let generator = SeededScatter::new(0.5);
        let seed = WorldSeed(0x0123_4567_89ab_cdef);
        let pos = ChunkPos::new(-3, 7);
        let a = generator.generate(seed, pos);
        let b = generator.generate(seed, pos);
        for y in 0..CHUNK_Y {
            for z in 0..CHUNK_Z {
                for x in 0..CHUNK_X {
                    assert_eq!(a.get(x, y, z), b.get(x, y, z), "({x},{y},{z}) 不相等");
                }
            }
        }
    }

    #[test]
    fn different_seed_differs() {
        let generator = SeededScatter::new(0.5);
        let pos = ChunkPos::new(0, 0);
        let a = generator.generate(WorldSeed(1), pos);
        let b = generator.generate(WorldSeed(2), pos);
        let diff = (0..CHUNK_Y)
            .flat_map(|y| (0..CHUNK_Z).map(move |z| (y, z)))
            .flat_map(|(y, z)| (0..CHUNK_X).map(move |x| (y, z, x)))
            .filter(|&(y, z, x)| a.get(x, y, z) != b.get(x, y, z))
            .count();
        assert!(diff > 0, "不同 seed 至少应有一格不同");
    }

    #[test]
    fn superflat_layers_match_table() {
        let generator = Superflat::classic();
        let chunk = generator.generate(WorldSeed(0), ChunkPos::new(0, 0));
        // 局部 y=0 对应世界 y = WORLD_Y_MIN(-64):stone 底,dirt×3 叠上。
        assert_eq!(chunk.get(0, 0, 0), STONE);
        assert_eq!(chunk.get(CHUNK_X - 1, 0, CHUNK_Z - 1), STONE);
        for ly in 1..=3 {
            for z in 0..CHUNK_Z {
                for x in 0..CHUNK_X {
                    assert_eq!(chunk.get(x, ly, z), DIRT, "第 {ly} 层应为 dirt");
                }
            }
        }
        for z in 0..CHUNK_Z {
            for x in 0..CHUNK_X {
                assert_eq!(chunk.get(x, 4, z), BlockId::AIR, "层表之上应为空气");
            }
        }
    }

    #[test]
    fn superflat_seed_independent() {
        let generator = Superflat::classic();
        let pos = ChunkPos::new(-5, 9);
        assert_eq!(
            generator.generate(WorldSeed(0xdead_beef), pos),
            generator.generate(WorldSeed(42), pos),
        );
    }

    #[test]
    fn scatter_within_base_shape() {
        let generator = SeededScatter::new(0.5);
        let seed = WorldSeed(7);
        let chunk = generator.generate(seed, ChunkPos::new(2, -2));
        // 顶面上一格的局部 y = 层厚和(1+3) = 4。
        for y in 0..CHUNK_Y {
            for z in 0..CHUNK_Z {
                for x in 0..CHUNK_X {
                    let block = chunk.get(x, y, z);
                    if y < 4 {
                        // 基地层内必须与层表一致,不被散布破坏。
                        assert_eq!(
                            block,
                            generator
                                .base
                                .generate(seed, ChunkPos::new(2, -2))
                                .get(x, y, z),
                            "层表在 ({x},{y},{z}) 被破坏"
                        );
                    } else if y == 4 {
                        // 顶面上一格:air 或散布方块(内置表内)。
                        assert!(
                            block == BlockId::AIR || block == STONE || block == DIRT,
                            "散布格出现未知方块 {block:?}"
                        );
                    } else {
                        assert_eq!(block, BlockId::AIR, "顶面上两格不应有方块");
                    }
                }
            }
        }
    }

    #[test]
    fn generator_is_send() {
        fn assert_send<T: Send>(_: &T) {}
        let superflat = Superflat::classic();
        let scatter = SeededScatter::new(0.3);
        assert_send(&superflat);
        assert_send(&scatter);
        let boxed: Box<dyn WorldGenerator> = Box::new(Superflat::classic());
        assert_send(&boxed);
        let boxed_scatter: Box<dyn WorldGenerator> = Box::new(SeededScatter::new(1.0));
        assert_send(&boxed_scatter);
    }
}
