//! 足迹与强制落地(REQ-007 §4.5.2/§4.8;任务书 req007-tickets/03)。
//!
//! 纯函数:无墙钟、无随机、无 IO、rapier 类型不出(§14-3/5)。阻塞请求判定
//! 只依实体状态(§14-8:本模块无票种参数)。

use std::collections::BTreeSet;

use types::{BlockId, BlockPos, CHUNK_SIZE, ChunkPos};
#[cfg(test)]
use types::{WORLD_Y_MAX, WORLD_Y_MIN};

/// √3(half_size 旋转包络系数;stable 工具链无 `f32::consts::SQRT_3`)。
const SQRT_3: f32 = 1.732_050_8;

/// 实体所在格(与凝固桥同口径:中心取 floor)。
pub fn cell_of(x: f32, y: f32, z: f32) -> BlockPos {
    BlockPos::new(x.floor() as i32, y.floor() as i32, z.floor() as i32)
}

/// 当前足迹:实体 AABB(旋转包络 = `half_size·√3`)外扩 `guard_m` 的水平
/// chunk 集(§4.5.2;全高柱投影)。I2/待地形/强制落地口径。
pub fn current_footprint(
    pos: types::glam::Vec3,
    half_size: f32,
    guard_m: f32,
) -> BTreeSet<ChunkPos> {
    let reach = half_size * SQRT_3 + guard_m;
    footprint_chunks(pos, reach)
}

/// 预载足迹 = 当前足迹 ∪ 前视:沿水平速度扫掠 `lookahead_ticks` 拍覆盖的
/// chunk,按到实体距离升序截断到 `max_chunks`(§4.5.2)。**只用于装载请求**
/// (调用方拼装),不参与 I2。
pub fn preload_footprint(
    pos: types::glam::Vec3,
    half_size: f32,
    guard_m: f32,
    linvel: types::glam::Vec3,
    lookahead_ticks: u64,
    max_chunks: usize,
) -> BTreeSet<ChunkPos> {
    let mut out = current_footprint(pos, half_size, guard_m);
    // 前视:扫掠终点 = 位置 + 水平速度 × lookahead × dt(1/60 由调用方折算——
    // 纯函数不取配置,扫掠长度直接给米)。这里以 ticks 参数 × 每拍位移矢量的
    // 形式收口:调用方传"每拍位移"。为保持签名任务书形(linvel + ticks),
    // 内部按 60Hz 常量折算(dt = 1/60,与 ShardSimConfig::new 同源)。
    const DT: f32 = 1.0 / 60.0;
    let sweep = types::glam::Vec3::new(
        linvel.x * lookahead_ticks as f32 * DT,
        0.0,
        linvel.z * lookahead_ticks as f32 * DT,
    );
    let end = pos + sweep;
    // 沿扫掠线按步长 CHUNK_SIZE/2 采样端点足迹(保守覆盖路径;端点足远近都到)。
    let steps = ((end - pos).length() / (CHUNK_SIZE as f32 * 0.5))
        .ceil()
        .max(1.0) as usize;
    let mut cand = BTreeSet::new();
    for k in 0..=steps {
        let t = k as f32 / steps as f32;
        let p = pos + (end - pos) * t;
        cand.extend(footprint_chunks(p, half_size * SQRT_3 + guard_m));
    }
    // 按到实体中心距离升序截断到 max_chunks(去掉已在当前足迹的)。
    let mut extra: Vec<(f32, ChunkPos)> = cand
        .into_iter()
        .filter(|cp| !out.contains(cp))
        .map(|cp| (chunk_center_distance(cp, pos), cp))
        .collect();
    extra.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
    for (_, cp) in extra.into_iter().take(max_chunks) {
        out.insert(cp);
    }
    out
}

/// 覆盖以 `pos` 为中心、水平半宽 `reach` 的 AABB 的 chunk 集(全高柱)。
fn footprint_chunks(pos: types::glam::Vec3, reach: f32) -> BTreeSet<ChunkPos> {
    let min_x = pos.x - reach;
    let max_x = pos.x + reach;
    let min_z = pos.z - reach;
    let max_z = pos.z + reach;
    let cx0 = (min_x / CHUNK_SIZE as f32).floor() as i32;
    let cx1 = (max_x / CHUNK_SIZE as f32).floor() as i32;
    let cz0 = (min_z / CHUNK_SIZE as f32).floor() as i32;
    let cz1 = (max_z / CHUNK_SIZE as f32).floor() as i32;
    let mut out = BTreeSet::new();
    for cx in cx0..=cx1 {
        for cz in cz0..=cz1 {
            out.insert(ChunkPos::new(cx, cz));
        }
    }
    out
}

fn chunk_center_distance(cp: ChunkPos, pos: types::glam::Vec3) -> f32 {
    let half = CHUNK_SIZE as f32 * 0.5;
    let center = types::glam::Vec3::new(
        (cp.x * CHUNK_SIZE) as f32 + half,
        0.0,
        (cp.z * CHUNK_SIZE) as f32 + half,
    );
    let (dx, dz) = (center.x - pos.x, center.z - pos.z);
    (dx * dx + dz * dz).sqrt()
}

/// 强制落地(§4.8 五步;纯体素,只读该柱):
/// - `column(x, z)` = 该柱某 y 的方块(None = ① 不在内存——调用方先判推迟,
///   本函数假定柱可用);
/// - `overlaps(pos)` = 候选格与本分片任一其他实体 AABB 重叠;
/// - 实心向上找第一个非实心;悬空向下找第一处支撑;扫到 `WORLD_Y_MIN` 仍无
///   支撑 → `None`(虚空移除);重叠让位沿柱向上。
pub fn forced_landing_cell(
    column: &dyn Fn(i32, i32) -> Option<BlockId>,
    x: i32,
    y: i32,
    z: i32,
    world_y_min: i32,
    world_y_max: i32,
    overlaps: &dyn Fn(BlockPos) -> bool,
) -> Option<BlockPos> {
    let solid_at = |by: i32| -> bool { column(x, by).is_some_and(|b| !b.is_air()) };
    let start = y.clamp(world_y_min, world_y_max);
    let candidate: BlockPos = if solid_at(start) {
        // 实心:向上找第一个非实心格(≤ WORLD_Y_MAX;到顶无空格 → None)。
        let mut by = start;
        while by <= world_y_max && solid_at(by) {
            by += 1;
        }
        if by > world_y_max {
            return None;
        }
        BlockPos::new(x, by, z)
    } else {
        // 非实心:向下找第一处实心,其上一格为候选;扫到 Y_MIN 仍无 → 虚空。
        let mut by = start;
        while by > world_y_min && !solid_at(by - 1) {
            by -= 1;
        }
        if by <= world_y_min && !solid_at(by - 1) {
            // 到 Y_MIN:若 Y_MIN 本格之下无支撑 → 虚空移除(调用方语义)。
            return None;
        }
        BlockPos::new(x, by, z)
    };
    // 重叠让位:沿柱向上找第一个既非实心也不重叠的格。
    let mut pos = candidate;
    while overlaps(pos) || solid_at(pos.y) {
        if pos.y >= world_y_max {
            return None;
        }
        pos = BlockPos::new(x, pos.y + 1, z);
    }
    Some(pos)
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::glam::Vec3;

    /// 测试清单(settle,任务书 03):足迹包络/guard/前视;前视截断按距离;
    /// 前视不进 I2(I2 只取 current_footprint——口径由调用方保证,此处锚
    /// current 与 preload 的差集即"前视部分")。
    #[test]
    fn footprint_current_and_preload() {
        // 静止实体:current = preload(half 0.5 → 包络 0.87 + guard 1.0 ≈ 1.87m
        // → 单 chunk)。
        let pos = Vec3::new(8.0, 0.0, 8.0);
        let cur = current_footprint(pos, 0.5, 1.0);
        assert_eq!(cur, BTreeSet::from([ChunkPos::new(0, 0)]), "静止:单 chunk");
        let pre = preload_footprint(pos, 0.5, 1.0, Vec3::ZERO, 30, 4);
        assert_eq!(pre, cur, "零速:预载 == 当前(无前视)");
        // 高速东移:30 m/s × 30 拍 × (1/60 s) = 15 m → 端点跨入 chunk (1,0)
        // (前视按速度扫掠折算;非线性 12 m/拍)。
        let fast = preload_footprint(pos, 0.5, 1.0, Vec3::new(30.0, 0.0, 0.0), 30, 4);
        assert!(fast.len() > cur.len(), "前视扩大装载集");
        assert!(fast.contains(&ChunkPos::new(1, 0)), "东向最近 chunk 在前视");
        // 截断:extra 恰 ≤ 4。
        assert!(fast.len() <= cur.len() + 4, "前视上限 4(按距离升序截断)");
        // 跨 chunk 边界实体(中心在 chunk (1,0) 边缘):包络跨两 chunk。
        let edge = current_footprint(Vec3::new(16.0, 0.0, 8.0), 0.5, 0.0);
        assert!(
            edge.contains(&ChunkPos::new(0, 0)) && edge.contains(&ChunkPos::new(1, 0)),
            "边界实体跨 chunk(零 guard 时由包络半径跨)"
        );
    }

    /// 测试清单(settle):实心向上/悬空向下/到 Y_MIN 虚空/重叠让位。
    #[test]
    fn forced_landing_column_scan() {
        let x = 3;
        let z = 4;
        // 实心柱:y=5 处实心 → 向上找第一个非实心 = 6。
        let solid = |_: i32, by: i32| {
            Some(if (0..=5).contains(&by) {
                BlockId(1)
            } else {
                BlockId::AIR
            })
        };
        let cell = forced_landing_cell(&solid, x, 5, z, WORLD_Y_MIN, WORLD_Y_MAX, &|_| false);
        assert_eq!(cell, Some(BlockPos::new(x, 6, z)), "实心向上找第一个非实心");
        // 悬空:y=20 空气、0..=5 实心 → 向下扫到下一格实心(5),候选 = 6
        // (§4.8 第 3 步:"直到下一格为实心,该格为候选")。
        let cell = forced_landing_cell(&solid, x, 20, z, WORLD_Y_MIN, WORLD_Y_MAX, &|_| false);
        assert_eq!(cell, Some(BlockPos::new(x, 6, z)), "悬空向下找支撑");
        // 虚空:全空柱 → None。
        let empty = |_: i32, _: i32| Some(BlockId::AIR);
        assert_eq!(
            forced_landing_cell(&empty, x, 10, z, WORLD_Y_MIN, WORLD_Y_MAX, &|_| false),
            None,
            "扫到 Y_MIN 无支撑 → 虚空移除"
        );
        // 重叠让位:候选格被占 → 向上找既非实心也不重叠的格。
        let cell = forced_landing_cell(&solid, x, 5, z, WORLD_Y_MIN, WORLD_Y_MAX, &|p| {
            p == BlockPos::new(x, 6, z)
        });
        assert_eq!(cell, Some(BlockPos::new(x, 7, z)), "重叠让位沿柱向上");
    }

    /// 测试清单(settle):柱 ① 缺(None)→ "推迟"路径(调用方产阻塞请求);
    /// 纯函数口径:column 返回 None 的格视为非实心(查询永不失败——但柱整体
    /// 缺数据由调用方在前置判定,此处锚 None 不 panic)。
    #[test]
    fn forced_landing_requires_column_data() {
        // column 恒 None(模拟柱 ① 缺——实际由调用方先判;此处锚健壮性)。
        let missing = |_: i32, _: i32| -> Option<BlockId> { None };
        // None 视为非实心 → 向下扫到 Y_MIN → None(虚空语义;真实推迟判定在
        // 调用方:柱 ① 缺时不调用本函数,改为产阻塞请求)。
        assert_eq!(
            forced_landing_cell(&missing, 0, 10, 0, WORLD_Y_MIN, WORLD_Y_MAX, &|_| false),
            None
        );
    }
}
