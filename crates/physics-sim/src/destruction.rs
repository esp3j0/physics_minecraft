//! destruction —— 地形破坏(REQ-009):爆炸求值、碎块方向、配置。
//!
//! **总架构师接口桩(REQ-009 单元 02)**:公开类型、常量与签名固定,函数体由实施方实现。
//! 算法规格见 REQ-009 §4(修订 2);标准答案由总架构师的参照实现给出(验收用例)。

use serde::{Deserialize, Serialize};
use types::glam::{Quat, Vec3};
use types::{BlockId, BlockPos};
use voxel::BlockRegistry;

/// 脱落块的半边长(REQ-009 §4.3;demo:0.45 静止最快)。
pub const DEBRIS_HALF_SIZE: f32 = 0.45;
/// 脱落块质量:碰撞体密度 1.0 × 体积 `(2·DEBRIS_HALF_SIZE)³`(与 sim 生成实体时 rapier 算出的质量一致)。
pub const DEBRIS_MASS: f32 =
    (2.0 * DEBRIS_HALF_SIZE) * (2.0 * DEBRIS_HALF_SIZE) * (2.0 * DEBRIS_HALF_SIZE);
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

/// 脱落请求 → 生成实体的初始状态与描述(REQ-009 §4.3、§7):位置 = 格中心、旋转为单位四元数、
/// `linvel = impulse / DEBRIS_MASS`、`angvel = 0`;描述 = 半边长 `DEBRIS_HALF_SIZE`、方块 = 原方块
/// (落地后凝固回同种材料)。坍塌块的 `impulse` 为 0。
///
/// **总架构师接口桩(REQ-009 单元 03)**:由实施方实现。
pub fn detach_spawn(req: &DetachRequest) -> (types::EntityState, types::EntityDesc) {
    let state = types::EntityState {
        translation: Vec3::new(
            req.cell.x as f32 + 0.5,
            req.cell.y as f32 + 0.5,
            req.cell.z as f32 + 0.5,
        ),
        rotation: Quat::IDENTITY,
        linvel: req.impulse / DEBRIS_MASS,
        angvel: Vec3::ZERO,
    };
    let desc = types::EntityDesc {
        half_size: DEBRIS_HALF_SIZE,
        block: req.block,
    };
    (state, desc)
}

/// 坍塌求值结果(REQ-009 §6)。
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CollapseOutcome {
    /// 应塌落的格(沙子规则 + 悬空且不超过 `max_cluster` 的连通块),`BlockPos` 升序、无重复。
    pub falling: Vec<BlockPos>,
    /// 悬空但超过 `max_cluster`、留在原地的连通块数。
    pub oversized_clusters: usize,
    /// 因单次搜索访问数超过 `max_cells` 而判为锚定的搜索次数。
    pub budget_anchored: usize,
    /// 全部搜索出队的格数之和(只用于遥测与代价验收,不参与判定)。
    pub visited: usize,
}

/// 坍塌求值(纯函数,REQ-009 §6.1–§6.2):
///
/// - `block_at` 给出**移除之后**的世界(`removed` 中的格已是非实心);`None` = 所在 chunk 未装载;
/// - **沙子**:对 `removed` 中每个格(`BlockPos` 升序),从其正上方起向上,连续的
///   `Support::NeedsBelow` 实心格全部塌落;
/// - **连通性**:记 `gone = removed ∪ 沙子塌落格`。种子 = `gone` 中各格的 6 邻居里的实心格(去重、
///   `BlockPos` 升序、不含 `gone`)。对每个尚未判定的种子做 6 连通的**向下优先**搜索(优先队列,键
///   `(y, x, z)` 升序;`gone` 视为非实心),出现以下任一即判定该连通块**已锚定**:出队的格是
///   `Support::Anchor` 或 `y == WORLD_Y_MIN`;出队的格已被前面的搜索判为锚定;要入队的邻居所在
///   chunk 未装载;本次搜索出队数超过 `max_cells`(计入 `budget_anchored`)。锚定时本次见过的格全部
///   标记为锚定;否则整块为悬空:格数 ≤ `max_cluster` 的并入 `falling`,否则计入 `oversized_clusters`;
///   悬空块同样标记,不重复搜索;
/// - 结果与 `removed` 的顺序无关。
///
/// **总架构师接口桩(REQ-009 单元 04a)**:由实施方实现。
pub fn find_collapse(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    registry: &dyn BlockRegistry,
    removed: &[BlockPos],
    max_cells: usize,
    max_cluster: usize,
) -> CollapseOutcome {
    let _ = (block_at, registry, removed, max_cells, max_cluster);
    unimplemented!("REQ-009 单元 04a:由实施方实现")
}

/// 线段 `from → to` 依次穿过的格,**含两端所在格**,按穿过顺序(REQ-009 §4.2):
/// Amanatides–Woo,全程 f64;同时跨越多个面(参数 t 相等)时按 x、y、z 的顺序逐轴步进,
/// 中间格都计入;某轴方向分量为 0 时该轴永不步进。
pub fn segment_cells(from: [f64; 3], to: [f64; 3]) -> Vec<BlockPos> {
    let mut cell = [
        from[0].floor() as i32,
        from[1].floor() as i32,
        from[2].floor() as i32,
    ];
    let end = [
        to[0].floor() as i32,
        to[1].floor() as i32,
        to[2].floor() as i32,
    ];
    let mut cells = vec![BlockPos::new(cell[0], cell[1], cell[2])];
    if cell == end {
        return cells;
    }
    let delta = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
    let mut step = [0i32; 3];
    // t_max:当前参数 t(整段为 1)下到下一格界的时间;0 分量轴恒 ∞(永不步进)。
    let mut t_max = [f64::INFINITY; 3];
    let mut t_delta = [f64::INFINITY; 3];
    for axis in 0..3 {
        if delta[axis] > 0.0 {
            step[axis] = 1;
            t_max[axis] = ((cell[axis] + 1) as f64 - from[axis]) / delta[axis];
            t_delta[axis] = 1.0 / delta[axis];
        } else if delta[axis] < 0.0 {
            step[axis] = -1;
            // 起点恰在格界上:boundary = 起点 → t = 0,立即步进(负方向语义)。
            t_max[axis] = (cell[axis] as f64 - from[axis]) / delta[axis];
            t_delta[axis] = -1.0 / delta[axis];
        }
    }
    // 每轴至多跨 ⌈|Δ|⌉ 次格界;留 3 步余量,超限 = 内部错误 fail-stop。
    let max_steps = delta.iter().map(|d| d.abs().ceil() as usize).sum::<usize>() + 3;
    for _ in 0..max_steps {
        // 取 t_max 最小的轴;并列时严格 < 使 x、y、z 顺序优先(§4.2 步进序)。
        let mut axis = 0;
        for a in 1..3 {
            if t_max[a] < t_max[axis] {
                axis = a;
            }
        }
        cell[axis] += step[axis];
        t_max[axis] += t_delta[axis];
        cells.push(BlockPos::new(cell[0], cell[1], cell[2]));
        if cell == end {
            return cells;
        }
    }
    unreachable!("segment_cells 未在步数上限内到达终点:from {from:?} → to {to:?}")
}

/// 碎块冲量方向 `dir' = normalize(DEBRIS_SPREAD·u_x, 1, DEBRIS_SPREAD·u_z)`,`u` 为爆心指向
/// 格中心的单位向量(REQ-009 §4.3)。
pub fn debris_direction(center: Vec3, cell: BlockPos) -> Vec3 {
    let cell_center = Vec3::new(
        cell.x as f32 + 0.5,
        cell.y as f32 + 0.5,
        cell.z as f32 + 0.5,
    );
    let offset = cell_center - center;
    let dist = offset.length();
    // 爆心恰在格中心:d 无定义,u 取零向量 → dir' 竖直向上(与 d=0 时冲量为
    // 零的语义一致,方向无观测面)。
    let u = if dist > 0.0 {
        offset / dist
    } else {
        Vec3::ZERO
    };
    Vec3::new(DEBRIS_SPREAD * u.x, 1.0, DEBRIS_SPREAD * u.z).normalize()
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
    // 有效半径封顶(§4.1):f64 全程。
    let r = radius.min(MAX_TERRAIN_BLAST_RADIUS) as f64;
    let c = [center.x as f64, center.y as f64, center.z as f64];
    // 候选包围盒:格中心(ix+0.5)与爆心距离可能 < r 的范围(逐格按 d 复核,
    // 这里取超集)。
    let lo = |v: f64| (v - r - 0.5).floor() as i32;
    let hi = |v: f64| (v + r - 0.5).floor() as i32;
    let mut cells: Vec<BlastCell> = Vec::new();
    for ix in lo(c[0])..=hi(c[0]) {
        for iy in lo(c[1])..=hi(c[1]) {
            for iz in lo(c[2])..=hi(c[2]) {
                let pos = BlockPos::new(ix, iy, iz);
                if !in_scope(pos) {
                    continue;
                }
                // 未装载(含界外):不求值(射线穿过它的衰减规则见下)。
                let Some(block) = block_at(pos) else {
                    continue;
                };
                // 只看实心格:空气、非实心与未知 ID 无材料。
                let Some(material) = registry.material(block) else {
                    continue;
                };
                let cell_center = [ix as f64 + 0.5, iy as f64 + 0.5, iz as f64 + 0.5];
                let dx = cell_center[0] - c[0];
                let dy = cell_center[1] - c[1];
                let dz = cell_center[2] - c[2];
                let d = (dx * dx + dy * dy + dz * dz).sqrt();
                if d >= r {
                    continue;
                }
                let mut pressure = power as f64 * (1.0 - d / r);
                // 射线衰减:穿过的格 = 全段去掉最后一个(目标格自身);
                // 未装载格不衰减,实心格按其材料 R 扣减(§4.2)。
                let through = segment_cells(c, cell_center);
                for &cell in &through[..through.len() - 1] {
                    if let Some(m) = block_at(cell).and_then(|b| registry.material(b)) {
                        pressure -= m.resistance as f64 * attenuation as f64;
                    }
                }
                if pressure <= material.resistance as f64 {
                    continue; // P > R 才破坏
                }
                let fate =
                    if pressure > material.pulverize_factor as f64 * material.resistance as f64 {
                        CellFate::Pulverize
                    } else {
                        CellFate::Detach {
                            impulse: pressure as f32 * debris_direction(center, pos),
                        }
                    };
                cells.push(BlastCell {
                    pos,
                    block,
                    pressure,
                    fate,
                });
            }
        }
    }
    // 脱落上限(§4.3):按 P 降序(同值 BlockPos 升序)保留前 max_debris,
    // 其余改粉碎(坑不变小)。
    let mut detach: Vec<usize> = cells
        .iter()
        .enumerate()
        .filter(|(_, c)| matches!(c.fate, CellFate::Detach { .. }))
        .map(|(i, _)| i)
        .collect();
    if detach.len() > max_debris {
        detach.sort_by(|&a, &b| {
            cells[b]
                .pressure
                .total_cmp(&cells[a].pressure)
                .then(cells[a].pos.cmp(&cells[b].pos))
        });
        for &i in &detach[max_debris..] {
            cells[i].fate = CellFate::Pulverize;
        }
    }
    cells.sort_by_key(|c| c.pos);
    cells
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::glam::Vec3;
    use voxel::{SimpleRegistry, builtin};

    fn reg() -> SimpleRegistry {
        SimpleRegistry::builtin()
    }

    fn pos(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    /// §4.2 体素遍历:某轴方向分量为 0 时该轴永不步进;同时跨面按 x、y、z 序。
    ///
    /// 变红:y 分量为 0 却步进(多出中间格);跨面并列按 z 先于 x(顺序交换)。
    #[test]
    fn segment_cells_zero_component_axis_never_steps() {
        // x 正向 2 格、z 负向 2 格、y 不动;t=0.25 与 t=0.75 各有一次 x/z 并列。
        assert_eq!(
            segment_cells([0.5, 0.5, 0.5], [2.5, 0.5, -1.5]),
            [
                pos(0, 0, 0),
                pos(1, 0, 0),
                pos(1, 0, -1),
                pos(2, 0, -1),
                pos(2, 0, -2)
            ]
        );
    }

    /// §4.2 负方向边界:起点恰在格界上时 t = 0 立即步进(不漏、不差一格)。
    ///
    /// 变红:把负方向的下一格界算成起点再退一格(漏掉立即步进,少算一格)。
    #[test]
    fn segment_cells_steps_immediately_from_negative_boundary() {
        assert_eq!(
            segment_cells([2.0, 0.5, 0.5], [0.5, 0.5, 0.5]),
            [pos(2, 0, 0), pos(1, 0, 0), pos(0, 0, 0)]
        );
    }

    /// §4.3 dir' = normalize(0.4·u_x, 1, 0.4·u_z):u 的 y 分量不进公式(恒以
    /// 向上为主);水平格的水平分量按 0.4 缩。
    ///
    /// 变红:dir' 直接用 u(向下/水平方向发射,回到修订 1 前的"碎块不飞散");
    /// 忘记 0.4 缩放(水平分量过大)。
    #[test]
    fn debris_direction_up_dominant_ignores_u_y() {
        // 正下方:u = (0,-1,0) → dir' = (0,1,0)。
        assert_vec(
            debris_direction(Vec3::new(0.5, 5.0, 0.5), pos(0, 0, 0)),
            [0.0, 1.0, 0.0],
            "正下方",
        );
        // 正上方(u 朝上):公式不看 u_y,仍竖直向上。
        assert_vec(
            debris_direction(Vec3::new(0.5, -5.0, 0.5), pos(0, 0, 0)),
            [0.0, 1.0, 0.0],
            "正上方",
        );
        // 水平相邻:u = (1,0,0) → dir' = normalize(0.4, 1, 0),y 分量占优。
        assert_vec(
            debris_direction(Vec3::new(64.5, -59.5, 64.5), pos(65, -60, 64)),
            [0.4 / 1.16_f32.sqrt(), 1.0 / 1.16_f32.sqrt(), 0.0],
            "水平相邻",
        );
    }

    fn assert_vec(got: Vec3, want: [f32; 3], what: &str) {
        assert!(
            (got - Vec3::from(want)).length() < 1e-6,
            "{what}:得到 {got:?},应为 {want:?}"
        );
    }

    /// §4.2 射线口径:起点格计入衰减、目标格自身不计;爆心在实心格内时
    /// (64,−1,64) 自身不衰减(P0=10),(64,−2,64) 只扣起点一格 R。
    ///
    /// 变红:衰减段去掉第一个(漏起点,压力偏高 2.0);把目标格算进衰减
    /// (压力偏低 2.0)。
    #[test]
    fn evaluate_blast_ray_includes_start_cell_and_excludes_target() {
        // y ∈ [-3,-1] 全 dirt,其余空气;爆心埋在 (64,-1,64) 内。
        let world = |p: BlockPos| {
            Some(match p.y {
                -3..=-1 => builtin::DIRT,
                _ => builtin::AIR,
            })
        };
        let cells = evaluate_blast(
            &world,
            &reg(),
            &|_| true,
            Vec3::new(64.5, -0.5, 64.5),
            10.0,
            4.0,
            1.0,
            48,
        );
        let find = |x: i32, y: i32, z: i32| {
            cells
                .iter()
                .find(|c| c.pos == pos(x, y, z))
                .unwrap_or_else(|| panic!("({x},{y},{z}) 应被破坏"))
        };
        let self_cell = find(64, -1, 64);
        assert!(
            (self_cell.pressure - 10.0).abs() < 1e-9,
            "爆心所在格:无中间格,P0 原值,实为 {}",
            self_cell.pressure
        );
        assert_eq!(self_cell.fate, CellFate::Pulverize);
        let below = find(64, -2, 64);
        assert!(
            (below.pressure - 5.5).abs() < 1e-9,
            "正下一格:P0=7.5 扣起点一格 R=2,实为 {}",
            below.pressure
        );
        assert_eq!(below.fate, CellFate::Pulverize, "5.5 > k·R=4");
        // 再下一格 P = 5 − 2·2 = 1 ≤ R:不破坏(不在结果里)。
        assert!(cells.iter().all(|c| c.pos != pos(64, -3, 64)));

        // 目标格自身不计:起点格 stone(R=6)、目标格 dirt(R=2),材质不同才
        // 能把"衰减到目标为止"与"衰减穿进目标"区分开。P = 7.5 − 6 = 1.5 ≤ 2
        // → (64,−2,64) 不破坏;把目标格算进衰减(或漏掉起点)都会变红。
        let stone_start = |p: BlockPos| {
            if p == pos(64, -1, 64) {
                Some(builtin::STONE)
            } else if p == pos(64, -2, 64) {
                Some(builtin::DIRT)
            } else {
                Some(builtin::AIR)
            }
        };
        let cells = evaluate_blast(
            &stone_start,
            &reg(),
            &|_| true,
            Vec3::new(64.5, -0.5, 64.5),
            10.0,
            4.0,
            1.0,
            48,
        );
        assert!(
            cells.iter().all(|c| c.pos != pos(64, -2, 64)),
            "衰减含起点 stone 不含目标 dirt:P=1.5 ≤ 2,不该破坏"
        );
    }

    /// §4.1 未装载格(None):不求值、射线穿过不衰减—— dirt 在"未装载"墙后
    /// 仍按无遮挡压力破坏。
    ///
    /// 变红:把 None 当实心衰减(压力偏低,破坏不了);把 None 也求值(多出格)。
    #[test]
    fn evaluate_blast_unloaded_cells_pass_through_and_skip() {
        // 只有 (66,0,0) 是 dirt;(65,0,0) 未装载;其余(含起点)空气。
        let world = |p: BlockPos| {
            if p == pos(66, 0, 0) {
                Some(builtin::DIRT)
            } else if p == pos(65, 0, 0) {
                None
            } else {
                Some(builtin::AIR)
            }
        };
        let cells = evaluate_blast(
            &world,
            &reg(),
            &|_| true,
            Vec3::new(64.5, 0.5, 0.5),
            8.0,
            4.0,
            1.0,
            48,
        );
        assert_eq!(cells.len(), 1, "只有 (66,0,0) 被求值破坏");
        assert_eq!(cells[0].pos, pos(66, 0, 0));
        assert!(
            (cells[0].pressure - 4.0).abs() < 1e-9,
            "P0 = 8·(1−2/4) = 4,未装载墙不衰减,实为 {}",
            cells[0].pressure
        );
        assert!(
            matches!(cells[0].fate, CellFate::Detach { .. }),
            "4 ∈ (2,4]"
        );
    }

    /// §4.3 上限:按 P 降序保留,不是按 BlockPos——压力较低但坐标较小的格
    /// 必须被改粉碎。
    ///
    /// 变红:截断按 BlockPos 升序(留下压力低的 (10,0,0))。
    #[test]
    fn evaluate_blast_cap_keeps_higher_pressure_not_lower_pos() {
        // 两块 dirt 关于爆心不对称:(12,0,0) 近(P≈2.834)、(10,0,0) 远(P≈2.280)。
        let world = |p: BlockPos| {
            if p == pos(12, 0, 0) || p == pos(10, 0, 0) {
                Some(builtin::DIRT)
            } else {
                Some(builtin::AIR)
            }
        };
        let cells = evaluate_blast(
            &world,
            &reg(),
            &|_| true,
            Vec3::new(11.9, 1.5, 0.5),
            4.0,
            4.0,
            1.0,
            1,
        );
        assert_eq!(cells.len(), 2, "两格都被破坏");
        let detach: Vec<BlockPos> = cells
            .iter()
            .filter(|c| matches!(c.fate, CellFate::Detach { .. }))
            .map(|c| c.pos)
            .collect();
        assert_eq!(detach, [pos(12, 0, 0)], "只留压力最高的 (12,0,0)");
    }

    /// §4.3 上限同压力并列:按 BlockPos 升序取前者。
    ///
    /// 变红:同值取后者(留下 (10,0,2));排序比较器写成全降序。
    #[test]
    fn evaluate_blast_cap_tie_breaks_by_block_pos() {
        // 两块 dirt 关于爆心 z 对称,压力逐位相等。
        let world = |p: BlockPos| {
            if p == pos(10, 0, 0) || p == pos(10, 0, 2) {
                Some(builtin::DIRT)
            } else {
                Some(builtin::AIR)
            }
        };
        let cells = evaluate_blast(
            &world,
            &reg(),
            &|_| true,
            Vec3::new(10.5, 1.5, 1.5),
            4.0,
            4.0,
            1.0,
            1,
        );
        let detach: Vec<BlockPos> = cells
            .iter()
            .filter(|c| matches!(c.fate, CellFate::Detach { .. }))
            .map(|c| c.pos)
            .collect();
        assert_eq!(detach, [pos(10, 0, 0)], "同压力按 BlockPos 升序取前者");
    }

    /// §4.1 半径上限:r = min(radius, 16)。半径 1000 与 16 的结果逐位相同。
    ///
    /// 变红:去掉封顶(半径 1000 把整层 dirt 炸光)。
    #[test]
    fn evaluate_blast_radius_capped_at_limit() {
        let world = |p: BlockPos| {
            Some(if p.y == -1 {
                builtin::DIRT
            } else {
                builtin::AIR
            })
        };
        let center = Vec3::new(64.5, 0.5, 64.5);
        let capped = evaluate_blast(&world, &reg(), &|_| true, center, 100.0, 1000.0, 1.0, 48);
        let sixteen = evaluate_blast(&world, &reg(), &|_| true, center, 100.0, 16.0, 1.0, 48);
        assert_eq!(capped, sixteen, "半径 1000 按 16 求值");
        assert!(
            !sixteen.is_empty() && sixteen.len() < 40 * 40,
            "16 m 半径只破坏周围一圈,而非整层:{}",
            sixteen.len()
        );
    }

    /// §7 生成状态(纯函数):位置 = 格中心、旋转 = 单位四元数、
    /// `linvel = impulse / DEBRIS_MASS`(0.729 kg)、角速度 0;描述 =
    /// 半边长 `DEBRIS_HALF_SIZE`、原方块(凝固回原方块,不出现默认 stone)。
    ///
    /// 变红:位置用格角(漏 +0.5);速度没除质量;描述沿用默认 0.5/stone。
    #[test]
    fn detach_spawn_matches_spec() {
        let impulse = Vec3::new(-1.024652, 3.158188, -0.614791);
        let (state, desc) = detach_spawn(&DetachRequest {
            cell: pos(61, -61, 62),
            block: builtin::DIRT,
            impulse,
        });
        assert!(
            (state.translation - Vec3::new(61.5, -60.5, 62.5)).length() < 1e-6,
            "位置 = 格中心,实为 {:?}",
            state.translation
        );
        assert!(
            state.rotation.abs_diff_eq(Quat::IDENTITY, 1e-6),
            "旋转 = 单位四元数"
        );
        assert!(
            (state.linvel - impulse / DEBRIS_MASS).length() < 1e-6,
            "linvel = impulse / DEBRIS_MASS,实为 {:?}",
            state.linvel
        );
        assert_eq!(state.angvel, Vec3::ZERO, "角速度 0");
        assert_eq!(
            desc,
            types::EntityDesc {
                half_size: DEBRIS_HALF_SIZE,
                block: builtin::DIRT,
            },
            "描述 = 半边长 0.45 + 原方块"
        );
    }
}
