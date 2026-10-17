# 任务 00:世界生成器(voxel)

## 目标

REQ-002 §4.1:`WorldGenerator` 纯函数 trait + Superflat + SeededScatter,
同 `(seed, pos)` 位级确定(AC-G1 地基)。内置注册表只有 air/stone/dirt
(voxel::SimpleRegistry),层表只用现有方块。

## 必读

- `crates/voxel/src/chunk.rs`(Chunk::air/set/get;CHUNK_X/Y/Z)
- `crates/types/src/ids.rs`(新类型风格)
- `docs/tasks/req002-world/README.md`

## 交付物

1. `crates/types/src/ids.rs`(或同 crate 合适文件):`WorldSeed(pub u64)`
   (Debug/Clone/Copy/PartialEq/Eq/Hash/Ord/serde derive,照 WorldId 风格),
   lib.rs re-export;
2. `crates/voxel/src/generator.rs` + lib.rs `pub mod generator;`:
   trait + 两个实现 + 内嵌测试。

## 接口设计(签名固定)

```rust
/// 世界生成器:纯函数,无 IO/随机/墙钟(蓝图 §4 纪律)。
/// 同 `(seed, pos)` 必须产出位级相同的 Chunk(AC-G1)。
pub trait WorldGenerator: Send {
    fn generate(&self, seed: WorldSeed, pos: ChunkPos) -> Chunk;
}

/// 超平坦:自下而上层表(thickness, BlockId);内置方块表内取值。
#[derive(Debug, Clone)]
pub struct Superflat { pub layers: Vec<(usize, BlockId)> }
impl Superflat {
    /// 经典:v1 内置方块 = stone×1 + dirt×3(方块表扩充后自然可加 grass)。
    pub fn classic() -> Self;
}

/// 种子化散布:超平坦基底 + 坐标哈希散布的方块(测试/RL 简单多样性)。
#[derive(Debug, Clone)]
pub struct SeededScatter { pub base: Superflat, pub density: f32 /* 0.0-1.0 */ }
impl SeededScatter { pub fn new(density: f32) -> Self }
```

哈希用 splitmix64 或 FNV-1a 自实现(确定性,不引 rand 依赖);散布判定 =
`hash(seed, pos, x, y, z) 的比值 < density`,散布层仅顶面上一格。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `same_seed_same_chunk_bitwise` | 同 (seed,pos) 两次 generate 逐坐标相等 |
| `different_seed_differs` | SeededScatter 下不同 seed 至少有一格不同 |
| `superflat_layers_match_table` | classic() 层表逐层断言(stone 底、dirt×3 上) |
| `superflat_seed_independent` | Superflat 忽略 seed(两次不同 seed 相等) |
| `scatter_within_base_shape` | 散布只出现在基底顶面上一格,不破坏层表 |
| `generator_is_send` | 编译期断言两个实现 + Box<dyn WorldGenerator> |

## 完成定义

见 [README 公共完成定义](README.md)。
