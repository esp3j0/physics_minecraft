//! destruction —— 地形破坏(REQ-009):爆炸求值、碎块方向、配置。
//!
//! **总架构师接口桩(REQ-009 单元 02)**:公开类型、常量与签名固定,函数体由实施方实现。
//! 算法规格见 REQ-009 §4(修订 2);标准答案由总架构师的参照实现给出(验收用例)。

use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{BlockId, BlockPos};
use voxel::BlockRegistry;

/// 脱落块的半边长(REQ-009 §4.3;demo:0.45 静止最快)。
pub const DEBRIS_HALF_SIZE: f32 = 0.45;
/// 碎块冲量方向里水平分量的比例(REQ-009 §4.3:`dir' = normalize(0.4·u_x, 1, 0.4·u_z)`)。
pub const DEBRIS_SPREAD: f32 = 0.4;
/// 地形求值的半径上限 = REQ-001 的 `INTERACTION_MARGIN` 默认值(16 m):保证从爆心到本体区
/// 任一受影响格的线段落在"本体区 ∪ 扩张区"之内(REQ-009 §4.1)。
pub const MAX_TERRAIN_BLAST_RADIUS: f32 = 16.0;

/// 地形破坏配置(REQ-009 §8)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DestructionConfig {
    /// 总开关。**REQ-009 实施期间默认 `false`**,单元 07 迁移既有用例后改为 `true`(D7)。
    pub enabled: bool,
    /// 射线衰减系数(1.0)。
    pub attenuation: f32,
    /// 单次爆炸在单个分片内的脱落上限(48)。
    pub max_debris_per_explosion: usize,
    /// 每分片每拍新生成实体上限(64;单元 03 使用)。
    pub max_detach_per_tick: usize,
    /// 每分片每拍撞击破坏上限(16;单元 06 使用)。
    pub max_impact_breaks_per_tick: usize,
    /// 单个种子的坍塌搜索访问上限(2048;单元 04 使用)。
    pub collapse_max_cells: usize,
    /// 会塌落的悬空块大小上限(512;单元 04 使用)。
    pub collapse_max_cluster: usize,
}

impl Default for DestructionConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            attenuation: 1.0,
            max_debris_per_explosion: 48,
            max_detach_per_tick: 64,
            max_impact_breaks_per_tick: 16,
            collapse_max_cells: 2048,
            collapse_max_cluster: 512,
        }
    }
}

/// 被破坏的格的去向(REQ-009 §4.3)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CellFate {
    /// 粉碎:写 air,不产生实体。
    Pulverize,
    /// 脱落:写 air,并请求在 N+1 生成实体;`impulse` = `P · dir'`。
    Detach { impulse: Vec3 },
}

/// 一个被破坏的格。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BlastCell {
    pub pos: BlockPos,
    /// 破坏前的方块。
    pub block: BlockId,
    /// 有效压力 `P(c)`(f64,扣除射线衰减之后)。
    pub pressure: f64,
    pub fate: CellFate,
}

/// 脱落请求(分片在 tick N 产出,调度器在 N 末分配 ID、盖章 N+1 生成;REQ-009 §7)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DetachRequest {
    pub cell: BlockPos,
    pub block: BlockId,
    pub impulse: Vec3,
}

/// 线段 `from → to` 依次穿过的格,**含两端所在格**,按穿过顺序(REQ-009 §4.2):
/// Amanatides–Woo,全程 f64;同时跨越多个面(参数 t 相等)时按 x、y、z 的顺序逐轴步进,
/// 中间格都计入;某轴方向分量为 0 时该轴永不步进。
pub fn segment_cells(from: [f64; 3], to: [f64; 3]) -> Vec<BlockPos> {
    let _ = (from, to);
    unimplemented!("REQ-009 单元 02:由实施方实现")
}

/// 碎块冲量方向 `dir' = normalize(DEBRIS_SPREAD·u_x, 1, DEBRIS_SPREAD·u_z)`,`u` 为爆心指向
/// 格中心的单位向量(REQ-009 §4.3)。
pub fn debris_direction(center: Vec3, cell: BlockPos) -> Vec3 {
    let _ = (center, cell);
    unimplemented!("REQ-009 单元 02:由实施方实现")
}

/// 爆炸的地形求值(纯函数,REQ-009 §4.2–§4.3):
///
/// - `block_at`:`None` = 该格所在 chunk 未装载——不求值、射线穿过时不衰减;
/// - `in_scope`:是否求值该格(分片传"本体区内");
/// - 有效半径 `r = min(radius, MAX_TERRAIN_BLAST_RADIUS)`;只看实心格(`registry.material` 为
///   `Some`);`d` = 爆心到格中心的距离(f64),`d ≥ r` 不受影响;
/// - `P = power·(1 − d/r) − Σ R(穿过的实心格)·attenuation`,穿过的格 = `segment_cells(爆心, 格中心)`
///   去掉最后一个(目标格自身);`P > R` 破坏;`P > k·R` 粉碎,否则脱落;
/// - 脱落数超过 `max_debris` 时,按 `P` 降序(同值 `BlockPos` 升序)保留前 `max_debris` 个,其余改为粉碎;
/// - 返回按 `BlockPos` 升序。
#[allow(clippy::too_many_arguments)]
pub fn evaluate_blast(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    registry: &dyn BlockRegistry,
    in_scope: &dyn Fn(BlockPos) -> bool,
    center: Vec3,
    power: f32,
    radius: f32,
    attenuation: f32,
    max_debris: usize,
) -> Vec<BlastCell> {
    let _ = (
        block_at,
        registry,
        in_scope,
        center,
        power,
        radius,
        attenuation,
        max_debris,
    );
    unimplemented!("REQ-009 单元 02:由实施方实现")
}
