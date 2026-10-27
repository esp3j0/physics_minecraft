//! cohesion —— 方块粘结与承载判定(REQ-010)。
//!
//! **总架构师接口桩(REQ-010 单元 01)**:公开类型、常量与签名固定,函数体由实施方实现。
//! 规格见 REQ-010 §3–§5.2(修订 1);标准答案由验收用例中的参照实现给出。

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use serde::{Deserialize, Serialize};
use types::{BlockId, BlockPos, WORLD_Y_MIN};
use voxel::{BOND_INFINITE, BlockRegistry, Support};

use crate::destruction::DEBRIS_HALF_SIZE;

/// 6 邻居的偏移(种子收集、连通块 BFS 与建网的固定顺序:+x、−x、+y、−y、+z、−z)。
const NEIGHBOR_DELTAS: [(i32, i32, i32); 6] = [
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
];

/// 正下方(压,不限量)。
const DOWN: (i32, i32, i32) = (0, -1, 0);

/// "正下方、未装载 chunk、[`BOND_INFINITE`]" 共用的不限量容量:远大于任何合法格数,
/// 参与加减不会溢出(不拿 `u32::MAX` 直接参与运算)。
const INF: u64 = u64::MAX / 4;

/// 粘结与整块塌落配置(REQ-010 §11;`ShardSimConfig::cohesion`)。
///
/// **总架构师接口桩(REQ-010 单元 02)**:键与默认值固定。`enabled` 在单元 08 之前默认 `false`(D9 由单元 08 打开)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct CohesionConfig {
    /// 总开关(D9);与 `destruction.enabled` 同时为真才生效(§4.6),关闭时行为与 REQ-009 完全一致。
    pub enabled: bool,
    /// 单个悬挑连通块的格数上限(2048,§4.2)。
    pub load_max_cells: usize,
    /// 切块网格边长 S(6,§5.2;单元 04 使用)。
    pub compound_split_size: i32,
    /// 每分片每拍转换成实体的格数上限(1024,§5.2;单元 04 使用)。
    pub max_converted_cells_per_tick: usize,
    /// 每分片每拍处理碎裂的复合体数(16,§7.1;单元 06 使用)。
    pub max_fractures_per_tick: usize,
    /// 整块凝固 / 强制落地时最多上移的格数(2,§8;单元 05 使用)。
    pub compound_snap_lift: i32,
}

impl Default for CohesionConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            load_max_cells: 2048,
            compound_split_size: 6,
            max_converted_cells_per_tick: 1024,
            max_fractures_per_tick: 16,
            compound_snap_lift: 2,
        }
    }
}

/// 粘结与整块塌落遥测(REQ-010 §11):分片每拍计数(经 `Outbox::cohesion_stats` 交出),调度器全部分片合计进
/// `TickSummary::cohesion`。纯计数、每拍重算、不入快照。
///
/// **总架构师接口桩(REQ-010 单元 02)**:字段固定;单元 02 接线 `load_*` 六项与 `add`,其余由后续单元接线(此前恒为 0)。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CohesionStats {
    /// 承载判定次数(一次 `evaluate_load` 调用计 1;"转换前"那次另计)。
    pub load_evaluations: u64,
    /// 承载判定访问的悬挑格数(`LoadOutcome::visited` 之和)。
    pub load_visited: u64,
    /// 因超过 `load_max_cells` 判为稳定的连通块数(`LoadOutcome::budget_stable` 之和)。
    pub load_budget_stable: u64,
    /// 收到邻片承载转换而多求一次"转换前"F 的次数(§9)。
    pub load_double_evaluations: u64,
    /// 本拍判定出的 F 的格数(两次求值取并集后,含本体区外的格)。
    pub load_falling: u64,
    /// 本拍末仍留在待转换集合里的格数(§4.6)。
    pub load_pending: u64,
    /// 生成的复合体数(单元 04)。
    pub compounds_spawned: u64,
    /// 生成的复合体的格数之和(单元 04)。
    pub compound_cells: u64,
    /// 切块产生的段数(单元 04)。
    pub segments: u64,
    /// 碎裂断开的粘结数(单元 06)。
    pub bonds_broken: u64,
    /// 碎裂生成的分量数(单元 06)。
    pub fracture_components: u64,
    /// 整块凝固数(单元 05)。
    pub compounds_solidified: u64,
    /// 倾斜搁置散成单格的复合体数(单元 05)。
    pub tilt_scattered: u64,
    /// 强制落地退化为逐格的复合体数(单元 05)。
    pub forced_landing_fallbacks: u64,
    /// 爆炸从复合体上移除的格数(单元 07)。
    pub blast_removed_cells: u64,
}

impl CohesionStats {
    /// 逐字段相加(调度器合计用)。
    pub fn add(&mut self, other: &Self) {
        self.load_evaluations += other.load_evaluations;
        self.load_visited += other.load_visited;
        self.load_budget_stable += other.load_budget_stable;
        self.load_double_evaluations += other.load_double_evaluations;
        self.load_falling += other.load_falling;
        self.load_pending += other.load_pending;
        self.compounds_spawned += other.compounds_spawned;
        self.compound_cells += other.compound_cells;
        self.segments += other.segments;
        self.bonds_broken += other.bonds_broken;
        self.fracture_components += other.fracture_components;
        self.compounds_solidified += other.compounds_solidified;
        self.tilt_scattered += other.tilt_scattered;
        self.forced_landing_fallbacks += other.forced_landing_fallbacks;
        self.blast_removed_cells += other.blast_removed_cells;
    }
}

/// 复合体生成请求(REQ-010 §5.5):承载塌落的一段(≥ 2 格)转换成一个复合实体。
/// 原点 = `compound_origin(段)`,形状 = `CompoundShape::from_world(origin, 段)`。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompoundRequest {
    pub origin: BlockPos,
    pub shape: types::CompoundShape,
}

/// 分块请求(REQ-010 §8.3 第 3 条;单元 06、07 复用):一个实体在本拍被删除,其一部分在 N+1 作为新实体出现。
/// 调度器在同一分片的脱落与复合体请求之后,按 (`parent`, `index`) 升序分配涌现 ID。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PieceRequest {
    /// 被删除的原实体。
    pub parent: types::EntityId,
    /// 该分块在原实体里的序号(散开时 = 形状格序)。
    pub index: u32,
    pub state: types::EntityState,
    pub desc: types::EntityDesc,
}

/// 整块凝固的吸附判定结果(REQ-010 §8.3 第 1 条)。
#[derive(Debug, Clone, PartialEq)]
pub enum CompoundSnap {
    /// 写入这些格(世界格, 方块;`BlockPos` 升序);`lifted` = 上移的格数。
    Write {
        cells: Vec<(BlockPos, BlockId)>,
        lifted: i32,
    },
    /// 误差超限,或上移到上限仍被占 → 倾斜处理(散成单格)。
    Tilted,
}

/// 目标姿态(§8.3 第 1 条,凝固与强制落地共用):目标朝向 = 24 个轴对齐朝向中
/// 最近的一个;原点格 = 平移所在的格(格中心取整,`floor`);每格的目标格 =
/// 原点格 + 目标朝向 × 偏移(轴对齐朝向下乘积是符号置换,`round` 去浮点噪声)。
fn snap_targets(
    translation: types::glam::Vec3,
    rotation: types::glam::Quat,
    shape: &types::CompoundShape,
) -> (types::glam::Quat, Vec<(BlockPos, BlockId)>) {
    let orients = crate::solidify::axis_aligned_orientations();
    let snapped = crate::solidify::nearest_orientation(rotation, &orients);
    let origin = crate::settle::cell_of(translation.x, translation.y, translation.z);
    let targets = shape
        .cells()
        .iter()
        .map(|cell| {
            let off = types::glam::Vec3::new(
                cell.offset[0] as f32,
                cell.offset[1] as f32,
                cell.offset[2] as f32,
            );
            let spun = snapped * off;
            (
                BlockPos::new(
                    origin.x + spun.x.round() as i32,
                    origin.y + spun.y.round() as i32,
                    origin.z + spun.z.round() as i32,
                ),
                cell.block,
            )
        })
        .collect();
    (snapped, targets)
}

/// 整块凝固的吸附判定(纯函数,REQ-010 §8.3 第 1 条)。`occupied(p)` = 格 p 被地形实心或其他实体占用。
///
/// 误差 = 各格"实际中心(`translation + rotation × 偏移`)→ 目标格中心"距离的
/// 最大值,以及旋转与目标朝向的夹角——**都在上移之前算**(上移只解决被占,
/// 不放宽误差);超过 `max_snap_blocks` 或 `max_snap_angle_deg` → `Tilted`。
pub fn compound_snap(
    translation: types::glam::Vec3,
    rotation: types::glam::Quat,
    shape: &types::CompoundShape,
    max_snap_blocks: f32,
    max_snap_angle_deg: f32,
    max_lift: i32,
    occupied: &dyn Fn(BlockPos) -> bool,
) -> CompoundSnap {
    let (snapped, targets) = snap_targets(translation, rotation, shape);
    let mut max_disp = 0.0_f32;
    for cell in shape.cells() {
        let off = types::glam::Vec3::new(
            cell.offset[0] as f32,
            cell.offset[1] as f32,
            cell.offset[2] as f32,
        );
        let actual = translation + rotation * off;
        // 目标格中心:floor(平移) + round(旋转后偏移) + 0.5,与 snap_targets
        // 的取整逐位一致。
        let spun = snapped * off;
        let center = types::glam::Vec3::new(
            translation.x.floor() + spun.x.round() + 0.5,
            translation.y.floor() + spun.y.round() + 0.5,
            translation.z.floor() + spun.z.round() + 0.5,
        );
        max_disp = max_disp.max((actual - center).length());
    }
    let angle = crate::solidify::snap_angle_error(rotation, snapped);
    if max_disp > max_snap_blocks || angle > max_snap_angle_deg.to_radians() {
        return CompoundSnap::Tilted;
    }
    // 从上移 0 格起到 `max_lift`:取第一个所有目标格都没被占的上移量;都被占
    // → Tilted。格按 `BlockPos` 升序。
    for lift in 0..=max_lift {
        if targets
            .iter()
            .all(|(t, _)| !occupied(BlockPos::new(t.x, t.y + lift, t.z)))
        {
            let mut cells: Vec<(BlockPos, BlockId)> = targets
                .iter()
                .map(|(t, b)| (BlockPos::new(t.x, t.y + lift, t.z), *b))
                .collect();
            cells.sort();
            return CompoundSnap::Write {
                cells,
                lifted: lift,
            };
        }
    }
    CompoundSnap::Tilted
}

/// 整块强制落地的结果(REQ-010 §8.3 第 5 条)。
#[derive(Debug, Clone, PartialEq)]
pub enum CompoundLanding {
    /// 写入这些格(`BlockPos` 升序)。
    Write { cells: Vec<(BlockPos, BlockId)> },
    /// 涉及的柱有未装载的格 → 推迟。
    Defer,
    /// 上移到上限仍与实心重叠,或所有柱下方都没有实心 → 退化为逐格。
    Fallback,
}

/// 整块强制落地(纯函数,REQ-010 §8.3 第 5 条)。`solid(p)`:`Some(true)` 实心、`Some(false)` 非实心、`None` 未装载;
/// 向下扫描到 `WORLD_Y_MIN` 为止。
///
/// 目标姿态与 [`compound_snap`] 同构;先找最小上移量 `k ∈ 0..=max_lift` 使目标格
/// 都不是实心(期间查到未装载 → `Defer`;找不到 → `Fallback`),再整块向下平移
/// `d` 格——d = 各格所在柱"到下方第一个实心格之上"的距离的最小值(整块一个
/// 平移量,不是各格各落各的;扫描期间查到未装载 → `Defer`;所有柱下方都没有
/// 实心 → `Fallback`)。`Write` 的格按 `BlockPos` 升序。
pub fn compound_forced_landing(
    translation: types::glam::Vec3,
    rotation: types::glam::Quat,
    shape: &types::CompoundShape,
    max_lift: i32,
    solid: &dyn Fn(BlockPos) -> Option<bool>,
) -> CompoundLanding {
    let (_, targets) = snap_targets(translation, rotation, shape);
    let mut lift = None;
    for k in 0..=max_lift {
        let mut deferred = false;
        let free = targets
            .iter()
            .all(|(t, _)| match solid(BlockPos::new(t.x, t.y + k, t.z)) {
                Some(s) => !s,
                None => {
                    deferred = true;
                    false
                }
            });
        if deferred {
            return CompoundLanding::Defer;
        }
        if free {
            lift = Some(k);
            break;
        }
    }
    let Some(k) = lift else {
        // 上移到上限仍与实心重叠 → 退化为逐格。
        return CompoundLanding::Fallback;
    };
    let mut drop: Option<i32> = None;
    for (t, _) in &targets {
        // 该柱上严格低于目标格的最高实心格 s:落点 = s.y + 1,d = 落点与目标格的距离。
        let mut y = t.y + k - 1;
        let mut found = None;
        while y >= WORLD_Y_MIN {
            match solid(BlockPos::new(t.x, y, t.z)) {
                None => return CompoundLanding::Defer,
                Some(true) => {
                    found = Some(t.y + k - y - 1);
                    break;
                }
                Some(false) => y -= 1,
            }
        }
        drop = match (drop, found) {
            (Some(b), Some(f)) => Some(b.min(f)),
            (None, f) => f,
            (d, None) => d,
        };
    }
    let Some(d) = drop else {
        // 所有柱下方都没有实心 → 退化为逐格(逐格时空柱虚空的格自行消失)。
        return CompoundLanding::Fallback;
    };
    let mut cells: Vec<(BlockPos, BlockId)> = targets
        .iter()
        .map(|(t, b)| (BlockPos::new(t.x, t.y + k - d, t.z), *b))
        .collect();
    cells.sort();
    CompoundLanding::Write { cells }
}

/// 两个共面实心格之间的承载粘结 `B(a,b) = min(B_a, B_b)`(REQ-010 §3.1);
/// 任一侧不是实心方块(空气、非实心、未知 ID)为 0。
pub fn bond_between(registry: &dyn BlockRegistry, a: BlockId, b: BlockId) -> u32 {
    match (registry.material(a), registry.material(b)) {
        (Some(x), Some(y)) => x.bond.min(y.bond),
        _ => 0,
    }
}

/// 两个共面实心格之间的撞击粘结 `I(a,b) = min(I_a, I_b)`(REQ-010 §3.1,复用 REQ-009 的
/// `impact_threshold`);任一侧不是实心方块为 0.0。
pub fn impact_bond_between(registry: &dyn BlockRegistry, a: BlockId, b: BlockId) -> f32 {
    match (registry.material(a), registry.material(b)) {
        (Some(x), Some(y)) => x.impact_threshold.min(y.impact_threshold),
        _ => 0.0,
    }
}

/// 接地判定的参考实现(REQ-010 §4.2,逐格向下扫描;生产路径由单元 02 换成增量维护的柱缓存,
/// 结果必须与本函数一致):
///
/// 从 `pos` 起向下逐格检查当前格 `q`——
/// - `block_at(q) == None`(所在 chunk 未装载)→ `true`(看不见的地方当作有地基);
/// - `q` 不是实心方块 → `false`;
/// - `q` 的材料是 `Support::Anchor`,或 `q.y == WORLD_Y_MIN` → `true`;
/// - 否则 `q.y − 1` 继续。
pub fn is_grounded_by_scan(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    registry: &dyn BlockRegistry,
    pos: BlockPos,
) -> bool {
    let mut q = pos;
    loop {
        let Some(b) = block_at(q) else { return true };
        let Some(m) = registry.material(b) else {
            return false;
        };
        if m.support == Support::Anchor || q.y == WORLD_Y_MIN {
            return true;
        }
        q.y -= 1;
    }
}

/// 接地判定(生产路径,REQ-010 §4.2 修订 2):一次判定之内**按柱记忆化**——
/// 从查询格沿柱向下扫描,得出结论前途经的连续实心格(含得出结论的那格)与
/// 结论同判并缓存;后续查询命中缓存即返回,每段被查询到的连续实心格只向下
/// 扫描一次。判定语义与 [`is_grounded_by_scan`] 逐格一致(未装载 → true、
/// 非实心 → false、`Support::Anchor` 或 `WORLD_Y_MIN` → true);缓存只活在本
/// 次判定之内(返回的闭包持有,不跨拍)。
pub fn grounded_by_column_memo<'a>(
    block_at: &'a dyn Fn(BlockPos) -> Option<BlockId>,
    registry: &'a dyn BlockRegistry,
) -> impl Fn(BlockPos) -> bool + 'a {
    let cache: std::cell::RefCell<BTreeMap<BlockPos, bool>> =
        std::cell::RefCell::new(BTreeMap::new());
    move |start: BlockPos| -> bool {
        let mut cache = cache.borrow_mut();
        let mut chain: Vec<BlockPos> = Vec::new();
        let mut q = start;
        let grounded = loop {
            // 命中缓存:链上各格都在 q 正上方且连续实心,与 q 同判。
            if let Some(&known) = cache.get(&q) {
                break known;
            }
            chain.push(q);
            let Some(b) = block_at(q) else {
                break true; // 未装载的 chunk:看不见的地方当作有地基
            };
            let Some(m) = registry.material(b) else {
                break false; // 途经非实心格:上方悬空
            };
            if m.support == Support::Anchor || q.y == WORLD_Y_MIN {
                break true;
            }
            q = BlockPos::new(q.x, q.y - 1, q.z);
        };
        cache.extend(chain.iter().map(|&c| (c, grounded)));
        grounded
    }
}

/// 承载判定结果(REQ-010 §4)。
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct LoadOutcome {
    /// F:应塌落的格,`BlockPos` 升序、无重复。
    pub falling: Vec<BlockPos>,
    /// 求过最大流的悬挑连通块的格数之和(遥测;因预算判稳定的连通块不计入)。
    pub visited: usize,
    /// 因超过 `max_cells` 判为稳定的连通块数(遥测)。
    pub budget_stable: usize,
    /// 被切断的粘结数:一端在 F、另一端是 F 之外的实心格、`bond_between > 0` 的共面连接数(遥测)。
    pub cut_bonds: usize,
}

/// 承载判定(纯函数,REQ-010 §4.2–§4.3):
///
/// - `block_at` 给出**改动之后**的世界;`None` = 所在 chunk 未装载;`grounded(p)` 回答实心格 `p` 是否
///   接地,结果须与 [`is_grounded_by_scan`] 一致;
/// - `changed`:本拍实心性发生变化的格(实心变空、空变实心);
/// - **种子** = `changed` 各格及其 6 邻格中"实心且不接地"的格,去重后按 `BlockPos` 升序处理;
/// - **悬挑连通块**:从尚未处理的种子出发做 6 连通 BFS(先进先出;邻格顺序 +x、−x、+y、−y、+z、−z),
///   只进入"实心且不接地"的格,`block_at == None` 的格不进入。出队数超过 `max_cells`,或扩展时碰到
///   已被判为"预算稳定"的格 → 本连通块按**稳定**处理:本次入队过的格全部标记为预算稳定,不再搜索;
///   因出队数超限而判稳定时 `budget_stable += 1`;
/// - **容量网络**(每个完整的连通块 C 单独建网,全部整数):源点 → C 中每格容量 1;C 中格 a 的 6 邻格 b——
///   `block_at(b) == None` → a 到汇点容量 ∞;b 非实心 → 无边;否则容量 = b 在正下方时 ∞,
///   b 在水平方向或正上方时 `bond_between(a, b)`(为 0 即无边);b 接地 → 这条边连到汇点,否则连到 b;
/// - 求最大流(算法不限;推荐 Dinic);**残量网络中从源点可达的 C 中格**并入 F;
/// - 结果与 `changed` 的顺序无关。
///
/// **总架构师接口桩(REQ-010 单元 01)**:由实施方实现。
pub fn evaluate_load(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    grounded: &dyn Fn(BlockPos) -> bool,
    registry: &dyn BlockRegistry,
    changed: &[BlockPos],
    max_cells: usize,
) -> LoadOutcome {
    let solid = |q: BlockPos| block_at(q).is_some_and(|b| registry.material(b).is_some());
    let overhang = |q: BlockPos| solid(q) && !grounded(q);

    // 种子:changed 各格及其 6 邻格中"实心且不接地"的格;去重、升序 → 与 changed 的顺序无关。
    let mut seeds: BTreeSet<BlockPos> = BTreeSet::new();
    for &c in changed {
        let neighbours =
            NEIGHBOR_DELTAS.map(|(dx, dy, dz)| BlockPos::new(c.x + dx, c.y + dy, c.z + dz));
        for q in std::iter::once(c).chain(neighbours) {
            if overhang(q) {
                seeds.insert(q);
            }
        }
    }

    let mut done: BTreeSet<BlockPos> = BTreeSet::new();
    let mut budget_marked: BTreeSet<BlockPos> = BTreeSet::new();
    let mut falling: BTreeSet<BlockPos> = BTreeSet::new();
    let (mut visited, mut budget_stable) = (0usize, 0usize);

    for &seed in &seeds {
        if done.contains(&seed) || budget_marked.contains(&seed) {
            continue;
        }
        let mut queue = VecDeque::from([seed]);
        let mut enqueued: BTreeSet<BlockPos> = BTreeSet::from([seed]);
        let mut order: Vec<BlockPos> = Vec::new();
        let (mut stable, mut over_budget) = (false, false);
        'bfs: while let Some(c) = queue.pop_front() {
            order.push(c);
            if order.len() > max_cells {
                // 出队数超限:整个区域按稳定(保守,与 REQ-009 搜索预算用完视为锚定同理)。
                stable = true;
                over_budget = true;
                break;
            }
            for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
                let n = BlockPos::new(c.x + dx, c.y + dy, c.z + dz);
                if budget_marked.contains(&n) {
                    // 碰到本拍已按预算判稳定的格:本块同样按稳定,不再搜索(不加计数)。
                    stable = true;
                    break 'bfs;
                }
                if !enqueued.contains(&n) && overhang(n) {
                    enqueued.insert(n);
                    queue.push_back(n);
                }
            }
        }
        if stable {
            if over_budget {
                budget_stable += 1;
            }
            budget_marked.extend(enqueued);
            continue;
        }
        visited += order.len();
        falling.extend(falling_component(block_at, registry, &order));
        done.extend(order);
    }

    // 被切断的粘结:F 中的格与 F 之外带粘结的实心格之间的共面连接。
    let mut cut_bonds = 0usize;
    for &c in &falling {
        let bc = block_at(c).expect("F 中的格已装载");
        for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
            let n = BlockPos::new(c.x + dx, c.y + dy, c.z + dz);
            if falling.contains(&n) {
                continue;
            }
            if let Some(bn) = block_at(n)
                && registry.material(bn).is_some()
                && bond_between(registry, bc, bn) > 0
            {
                cut_bonds += 1;
            }
        }
    }

    LoadOutcome {
        falling: falling.into_iter().collect(),
        visited,
        budget_stable,
        cut_bonds,
    }
}

/// 对一个完整的悬挑连通块建容量网络、求最大流,返回残量网络中从源点可达的格(F;
/// 最小割的最小源侧,与算法、遍历顺序无关)。
fn falling_component(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    registry: &dyn BlockRegistry,
    order: &[BlockPos],
) -> BTreeSet<BlockPos> {
    // 节点 0 = 源点、1 = 汇点,悬挑格从 2 起按 BFS 序编号。
    let idx: BTreeMap<BlockPos, usize> =
        order.iter().enumerate().map(|(i, c)| (*c, i + 2)).collect();
    let mut net = Dinic::new(order.len() + 2);
    for a in order {
        let u = idx[a];
        // 一格的重量。
        net.edge(0, u, 1);
        let ba = block_at(*a).expect("悬挑连通块内的格已装载");
        for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
            let b = BlockPos::new(a.x + dx, a.y + dy, a.z + dz);
            let capacity = match block_at(b) {
                // 未装载的 chunk:当作有地基。
                None => INF,
                Some(bb) => match registry.material(bb) {
                    // 非实心:无边。
                    None => continue,
                    // 正下方(压)不限量;水平与正上方(剪、拉)受 B(a,b) 限制。
                    Some(_) if (dx, dy, dz) == DOWN => INF,
                    Some(_) => match bond_between(registry, ba, bb) {
                        0 => continue,
                        BOND_INFINITE => INF,
                        bond => u64::from(bond),
                    },
                },
            };
            match idx.get(&b) {
                // 块内悬挑格;不在块内的实心格必是接地格 → 汇点(容量照常)。
                Some(&v) => net.edge(u, v, capacity),
                None => net.edge(u, 1, capacity),
            }
        }
    }
    net.max_flow(0, 1);
    let reach = net.reachable(0);
    order.iter().filter(|c| reach[idx[*c]]).copied().collect()
}

/// Dinic 最大流(邻接表残量网络,全整数容量;`INF` 远大于格数,累加不溢出)。
struct Dinic {
    adj: Vec<Vec<usize>>,
    to: Vec<usize>,
    cap: Vec<u64>,
    level: Vec<Option<u32>>,
}

impl Dinic {
    fn new(n: usize) -> Self {
        Self {
            adj: vec![Vec::new(); n],
            to: Vec::new(),
            cap: Vec::new(),
            level: Vec::new(),
        }
    }

    /// 加一条容量 `c` 的边(同时建容量 0 的反向边)。
    fn edge(&mut self, u: usize, v: usize, c: u64) {
        self.adj[u].push(self.to.len());
        self.to.push(v);
        self.cap.push(c);
        self.adj[v].push(self.to.len());
        self.to.push(u);
        self.cap.push(0);
    }

    fn max_flow(&mut self, s: usize, t: usize) {
        while self.leveling(s, t) {
            let mut next = vec![0usize; self.adj.len()];
            while self.send(s, t, INF, &mut next) > 0 {}
        }
    }

    /// BFS 分层;汇点不可达时结束。
    fn leveling(&mut self, s: usize, t: usize) -> bool {
        self.level = vec![None; self.adj.len()];
        self.level[s] = Some(0);
        let mut q = VecDeque::from([s]);
        while let Some(v) = q.pop_front() {
            let lv = self.level[v].expect("已出队的节点有层号");
            for &e in &self.adj[v] {
                let u = self.to[e];
                if self.cap[e] > 0 && self.level[u].is_none() {
                    self.level[u] = Some(lv + 1);
                    q.push_back(u);
                }
            }
        }
        self.level[t].is_some()
    }

    /// 在分层图中沿一条增广路推送;返回推送量(0 = 当前分层图已无增广路)。
    /// 显式边栈迭代(单元 01 遗留):分层深度可达格数(2048 格一字长链约
    /// 2050 层),shard worker 线程是默认栈,递归版有爆栈风险;推进/回退
    /// 与递归版逐位等价(成功弧不推 `next`,死端连同其弧一起跳过)。
    fn send(&mut self, s: usize, t: usize, f: u64, next: &mut [usize]) -> u64 {
        let mut path: Vec<usize> = Vec::new();
        let mut v = s;
        'advance: while v != t {
            while next[v] < self.adj[v].len() {
                let e = self.adj[v][next[v]];
                let u = self.to[e];
                if self.cap[e] > 0 && self.level[u] == self.level[v].map(|lv| lv + 1) {
                    path.push(e);
                    v = u;
                    continue 'advance;
                }
                next[v] += 1;
            }
            // v 已无可用弧:回退到本条边的起点并跳过它(递归版返回 0 的展开)。
            match path.pop() {
                Some(e) => {
                    v = self.to[e ^ 1];
                    next[v] += 1;
                }
                None => return 0,
            }
        }
        let pushed = f.min(path.iter().map(|&e| self.cap[e]).min().unwrap_or(f));
        for &e in &path {
            self.cap[e] -= pushed;
            self.cap[e ^ 1] += pushed;
        }
        pushed
    }

    /// 残量网络中从 `s` 出发沿剩余容量可达的节点。
    fn reachable(&self, s: usize) -> Vec<bool> {
        let mut seen = vec![false; self.adj.len()];
        seen[s] = true;
        let mut q = VecDeque::from([s]);
        while let Some(v) = q.pop_front() {
            for &e in &self.adj[v] {
                let u = self.to[e];
                if !seen[u] && self.cap[e] > 0 {
                    seen[u] = true;
                    q.push_back(u);
                }
            }
        }
        seen
    }
}

/// 分组与切块(纯函数,REQ-010 §5.2):两格**同在一个切块立方内**且 `bond_between > 0` 的共面相连才算连通
/// (切块立方:格 `(x, y, z)` 属于 `(x.div_euclid(S), y.div_euclid(S), z.div_euclid(S))`,S = `split_size`),
/// 按此把 `cells` 划分为连通分量。等价于"先按粘结连通分组、再用网格切开、每块内再按粘结连通划分"。
///
/// 返回各段:段内格按 `BlockPos` 升序,各段按首格升序排列;格的总数与输入相同(输入去重)。
/// `block_at` 给出这些格**转换之前**的方块(用来求 `bond_between`)。
pub fn split_segments(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    registry: &dyn BlockRegistry,
    cells: &[BlockPos],
    split_size: i32,
) -> Vec<Vec<BlockPos>> {
    let unique: BTreeSet<BlockPos> = cells.iter().copied().collect();
    // 世界对齐的切块:div_euclid 让负坐标也按网格边界切开(截断除法会把 −1 与 0 切进同一块)。
    let cube = |c: BlockPos| {
        (
            c.x.div_euclid(split_size),
            c.y.div_euclid(split_size),
            c.z.div_euclid(split_size),
        )
    };
    let mut segments: Vec<Vec<BlockPos>> = Vec::new();
    let mut seen: BTreeSet<BlockPos> = BTreeSet::new();
    for &start in &unique {
        if !seen.insert(start) {
            continue;
        }
        let mut segment: BTreeSet<BlockPos> = BTreeSet::from([start]);
        let mut queue = VecDeque::from([start]);
        while let Some(c) = queue.pop_front() {
            for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
                let n = BlockPos::new(c.x + dx, c.y + dy, c.z + dz);
                if !unique.contains(&n)
                    || segment.contains(&n)
                    || cube(n) != cube(c)
                    || match (block_at(c), block_at(n)) {
                        (Some(a), Some(b)) => bond_between(registry, a, b) == 0,
                        _ => true,
                    }
                {
                    continue;
                }
                segment.insert(n);
                queue.push_back(n);
            }
        }
        seen.extend(segment.iter().copied());
        segments.push(segment.into_iter().collect());
    }
    segments
}

/// 复合体原点(REQ-010 §5.1):各格包围盒中心取整到最近的格中心,同距取坐标较小者——
/// 每个轴上 `(min + max).div_euclid(2)`。`cells` 非空。
pub fn compound_origin(cells: &[BlockPos]) -> BlockPos {
    let (mut min, mut max) = (cells[0], cells[0]);
    for c in cells {
        min.x = min.x.min(c.x);
        min.y = min.y.min(c.y);
        min.z = min.z.min(c.z);
        max.x = max.x.max(c.x);
        max.y = max.y.max(c.y);
        max.z = max.z.max(c.z);
    }
    BlockPos::new(
        (min.x + max.x).div_euclid(2),
        (min.y + max.y).div_euclid(2),
        (min.z + max.z).div_euclid(2),
    )
}

/// 包络半径 ρ(REQ-010 §5.1):各格中心到原点格中心的最大距离 + `DEBRIS_HALF_SIZE·√3`。
pub fn envelope_radius(cells: &[BlockPos], origin: BlockPos) -> f32 {
    let farthest = cells
        .iter()
        .map(|c| {
            let (dx, dy, dz) = (
                (c.x - origin.x) as f32,
                (c.y - origin.y) as f32,
                (c.z - origin.z) as f32,
            );
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .fold(0.0_f32, f32::max);
    farthest + DEBRIS_HALF_SIZE * 3.0_f32.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use voxel::{SimpleRegistry, builtin};

    fn p(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    /// 指定格为给定方块、其余空气的世界。
    fn block_world(
        cells: impl IntoIterator<Item = (BlockPos, BlockId)>,
    ) -> impl Fn(BlockPos) -> Option<BlockId> {
        let map: BTreeMap<BlockPos, BlockId> = cells.into_iter().collect();
        move |q| Some(map.get(&q).copied().unwrap_or(builtin::AIR))
    }

    fn stone_line(xs: std::ops::Range<i32>, y: i32, z: i32) -> Vec<(BlockPos, BlockId)> {
        xs.map(|x| (p(x, y, z), builtin::STONE)).collect()
    }

    // ------------------------------------------------------------------ bond

    /// B/I 两格取较小值;任一侧不是实心方块(空气、未知 ID)为 0。
    #[test]
    fn pairwise_bond_takes_min_and_zero_for_non_solid() {
        let reg = SimpleRegistry::builtin();
        assert_eq!(bond_between(&reg, builtin::STONE, builtin::STONE), 8);
        assert_eq!(bond_between(&reg, builtin::STONE, builtin::DIRT), 3);
        assert_eq!(bond_between(&reg, builtin::DIRT, builtin::STONE), 3);
        assert_eq!(bond_between(&reg, builtin::STONE, builtin::SAND), 0);
        assert_eq!(
            bond_between(&reg, builtin::BEDROCK, builtin::BEDROCK),
            BOND_INFINITE
        );
        assert_eq!(bond_between(&reg, builtin::STONE, builtin::AIR), 0);
        assert_eq!(bond_between(&reg, builtin::STONE, BlockId(99)), 0);
        assert_eq!(
            impact_bond_between(&reg, builtin::STONE, builtin::DIRT),
            7.0
        );
        assert_eq!(impact_bond_between(&reg, builtin::SAND, builtin::DIRT), 4.0);
        assert_eq!(
            impact_bond_between(&reg, builtin::STONE, builtin::BEDROCK),
            18.0
        );
        assert_eq!(impact_bond_between(&reg, builtin::STONE, builtin::AIR), 0.0);
    }

    // ------------------------------------------------------------------ grounded

    /// 接地扫描:悬空基岩是地基、整列实心到世界底、未装载按有地基、起点空气为 false。
    #[test]
    fn grounded_scan_rules() {
        let reg = SimpleRegistry::builtin();
        let unloaded = p(4, 0, 0);
        let base = block_world([
            (p(-1, -1, 0), builtin::BEDROCK),
            (p(0, -1, 0), builtin::BEDROCK),
            (p(0, 0, 0), builtin::STONE),
            (p(1, 2, 0), builtin::STONE),
            (p(2, 10, 0), builtin::BEDROCK),
            (p(2, 11, 0), builtin::STONE),
            (p(4, 1, 0), builtin::STONE),
            (p(7, WORLD_Y_MIN, 0), builtin::STONE),
            (p(7, WORLD_Y_MIN + 1, 0), builtin::STONE),
        ]);
        let world = move |q: BlockPos| -> Option<BlockId> {
            if q == unloaded {
                return None;
            }
            base(q)
        };
        let g = |q: BlockPos| is_grounded_by_scan(&world, &reg, q);
        assert!(g(p(0, 0, 0)), "压在基岩上的石头");
        assert!(!g(p(1, 2, 0)), "下方是空气");
        assert!(g(p(2, 10, 0)), "悬空的基岩是地基");
        assert!(g(p(2, 11, 0)), "压在悬空基岩上的格子");
        assert!(g(p(4, 1, 0)), "正下方所在 chunk 未装载按有地基");
        assert!(g(p(4, 0, 0)), "起点本身未装载按有地基");
        assert!(g(p(7, WORLD_Y_MIN, 0)), "世界底");
        assert!(g(p(7, WORLD_Y_MIN + 1, 0)), "整列实心到世界底");
        assert!(!g(p(0, 5, 0)), "起点是空气");
    }

    /// `CohesionStats::add`:15 个字段逐一相加(调度器合计用)。
    ///
    /// 变红:漏加任一字段(合计比两分片之和小)。
    #[test]
    fn cohesion_stats_add_sums_every_field() {
        // 第 i 个字段 = i·k(1..=15):mk(1) + mk(100) = mk(101) 当且仅当
        // 每个字段都被加过一次。
        let mk = |k: u64| CohesionStats {
            load_evaluations: k,
            load_visited: 2 * k,
            load_budget_stable: 3 * k,
            load_double_evaluations: 4 * k,
            load_falling: 5 * k,
            load_pending: 6 * k,
            compounds_spawned: 7 * k,
            compound_cells: 8 * k,
            segments: 9 * k,
            bonds_broken: 10 * k,
            fracture_components: 11 * k,
            compounds_solidified: 12 * k,
            tilt_scattered: 13 * k,
            forced_landing_fallbacks: 14 * k,
            blast_removed_cells: 15 * k,
        };
        let (a, b) = (mk(1), mk(100));
        let mut sum = a;
        sum.add(&b);
        assert_eq!(sum, mk(101), "逐字段相加,一个不漏");
    }

    /// 按柱记忆化的接地判定与逐格扫描逐格一致:浮空 dirt、悬空基岩、未装载
    /// 格(含柱中段)、世界底、非实心格全部覆盖;判定过一遍后重查零次体素读
    /// (每段被查询到的连续实心格只向下扫描一次)。
    ///
    /// 变红:记忆化串格(不同柱共享结论/链上格未随结论标记);缓存命中乱返;
    /// 途经非实心格误判接地。
    #[test]
    fn grounded_memo_matches_scan_and_scans_each_segment_once() {
        let reg = SimpleRegistry::builtin();
        // x=0:dirt(-61..=-59)浮空(正下方是洞)、stone(-56)、悬空基岩(-55);
        // x=2:整列实心到世界底;x=4:stone(-57..=-56)下方一格未装载;
        // x=5:下半实心、(-60..=-50)未装载。
        let base: BTreeMap<BlockPos, BlockId> = [
            (p(0, -61, 0), builtin::DIRT),
            (p(0, -60, 0), builtin::DIRT),
            (p(0, -59, 0), builtin::DIRT),
            (p(0, -56, 0), builtin::STONE),
            (p(0, -55, 0), builtin::BEDROCK),
            (p(2, WORLD_Y_MIN, 0), builtin::BEDROCK),
            (p(2, -63, 0), builtin::DIRT),
            (p(2, -62, 0), builtin::DIRT),
            (p(2, -61, 0), builtin::DIRT),
            (p(4, -57, 0), builtin::STONE),
            (p(4, -56, 0), builtin::STONE),
            (p(5, WORLD_Y_MIN, 0), builtin::BEDROCK),
            (p(5, -63, 0), builtin::DIRT),
            (p(5, -62, 0), builtin::DIRT),
            (p(5, -61, 0), builtin::DIRT),
        ]
        .into_iter()
        .collect();
        let unloaded: BTreeSet<BlockPos> = (-60..=-50)
            .map(|y| p(5, y, 0))
            .chain((-60..=-58).map(|y| p(4, y, 0)))
            .collect();
        let lookup = |q: BlockPos| -> Option<BlockId> {
            if unloaded.contains(&q) {
                return None;
            }
            Some(base.get(&q).copied().unwrap_or(builtin::AIR))
        };
        let reads = std::cell::Cell::new(0u64);
        let counted = |q: BlockPos| -> Option<BlockId> {
            reads.set(reads.get() + 1);
            lookup(q)
        };
        let memo = grounded_by_column_memo(&counted, &reg);
        let scan = |q: BlockPos| is_grounded_by_scan(&lookup, &reg, q);
        for x in -1..=6 {
            for y in -70..=-50 {
                let q = p(x, y, 0);
                assert_eq!(memo(q), scan(q), "({x},{y}) 记忆化与扫描不一致");
            }
        }
        // 缓存命中:同片重查一遍,体素读数为 0(首查已把查询到的段全部缓存)。
        let after_first = reads.get();
        for x in -1..=6 {
            for y in -70..=-50 {
                let q = p(x, y, 0);
                assert_eq!(memo(q), scan(q), "({x},{y}) 重查不一致");
            }
        }
        assert_eq!(reads.get(), after_first, "命中缓存的查询不再读体素");
    }

    // ------------------------------------------------------------------ evaluate_load
    /// 预算边界:悬挑块恰好 `max_cells` 时照常求值(全塌),超一格按稳定且计 1 次。
    #[test]
    fn load_budget_boundary_exactly_max_cells() {
        let reg = SimpleRegistry::builtin();
        let load = |n: i32, max_cells: usize| {
            let cells: Vec<BlockPos> = (0..n).map(|x| p(x, 0, 0)).collect();
            let world = block_world(stone_line(0..n, 0, 0));
            let grounded = |_: BlockPos| false;
            let out = evaluate_load(&world, &grounded, &reg, &cells, max_cells);
            (cells, out)
        };
        let (cells, out) = load(4, 4);
        assert_eq!(out.falling, cells, "恰好 max_cells:照常求值,全塌");
        assert_eq!(out.visited, 4);
        assert_eq!(out.budget_stable, 0);
        let out = load(5, 4).1;
        assert!(out.falling.is_empty(), "超一格:按稳定");
        assert_eq!(out.budget_stable, 1);
        assert_eq!(out.visited, 0, "按稳定的块不计 visited");
    }

    /// 超预算块把"没入队的残余种子"也判稳定(不加计数),独立的悬空块照常判塌——
    /// 预算按连通块算,不按全部种子合计。
    #[test]
    fn load_budget_stable_blocks_residual_seed_and_independent_block() {
        let reg = SimpleRegistry::builtin();
        // 长线 6 格 > max_cells 4:首个种子的 BFS 只入队前 5 格,x = 5 的种子碰到标记格。
        let big: Vec<BlockPos> = (0..6).map(|x| p(x, 0, 0)).collect();
        // 独立悬空块:x 距离 ≥ 2,不与 big 连通。
        let indep: Vec<BlockPos> = (-13..-10).map(|x| p(x, 8, 0)).collect();
        let mut changed = big.clone();
        changed.extend(indep.iter().copied());
        let mut cells = stone_line(0..6, 0, 0);
        cells.extend(stone_line(-13..-10, 8, 0));
        let world = block_world(cells);
        let grounded = |_: BlockPos| false;
        let out = evaluate_load(&world, &grounded, &reg, &changed, 4);
        assert_eq!(out.budget_stable, 1, "只有长线超预算");
        assert_eq!(out.falling, indep, "独立小块照常判塌");
        assert_eq!(out.visited, indep.len(), "预算稳定的块不计 visited");
    }

    /// 向上的粘结边(剪/拉,容量 B):下层的重量只能经向上边传给上层,再由上层水平剪给
    /// 两端接地格。漏掉向上边时下层 4 格会误判塌。
    #[test]
    fn load_upward_bond_carries_weight_to_upper_floor() {
        let reg = SimpleRegistry::builtin();
        let lower: Vec<BlockPos> = (0..4).map(|x| p(x, 0, 0)).collect();
        let upper: Vec<BlockPos> = (0..4).map(|x| p(x, 1, 0)).collect();
        let anchors: BTreeSet<BlockPos> = BTreeSet::from([p(-1, 1, 0), p(4, 1, 0)]);
        let mut cells = Vec::new();
        for c in lower.iter().chain(upper.iter()).chain(anchors.iter()) {
            cells.push((*c, builtin::STONE));
        }
        let world = block_world(cells);
        let grounded = move |q: BlockPos| anchors.contains(&q);
        let changed: Vec<BlockPos> = lower.iter().chain(upper.iter()).copied().collect();
        let out = evaluate_load(&world, &grounded, &reg, &changed, 64);
        assert!(
            out.falling.is_empty(),
            "8 格走廊经向上边与两端出口挂住:{out:?}"
        );
        assert_eq!(out.visited, 8);
        assert_eq!(out.cut_bonds, 0);
    }

    /// 向下的压不限量:竖排 9 格加底部 1 格共重 10,两个出口容量 2×8 = 16,挂住;
    /// 若把向下边也按 B 限量,竖直链瓶颈 8 < 10,会误判塌。
    #[test]
    fn load_downward_pressure_is_unlimited() {
        let reg = SimpleRegistry::builtin();
        let chain: Vec<BlockPos> = (0..=9).map(|y| p(1, y, 0)).collect();
        let anchors: BTreeSet<BlockPos> = BTreeSet::from([p(0, 0, 0), p(2, 0, 0)]);
        let mut cells: Vec<(BlockPos, BlockId)> =
            chain.iter().map(|c| (*c, builtin::STONE)).collect();
        cells.extend(anchors.iter().map(|c| (*c, builtin::STONE)));
        let world = block_world(cells);
        let grounded = move |q: BlockPos| anchors.contains(&q);
        let out = evaluate_load(&world, &grounded, &reg, &chain, 64);
        assert!(out.falling.is_empty(), "向下的压不限量:10 格挂住");
        assert_eq!(out.visited, 10);
    }

    /// 未装载的 chunk 当作有地基:贴边的悬空条稳定(求过流,计 visited),
    /// 旁边无未装载邻居的悬空条照常塌。
    #[test]
    fn load_unloaded_chunk_counts_as_ground() {
        let reg = SimpleRegistry::builtin();
        let near: Vec<BlockPos> = (0..3).map(|x| p(x, 5, 0)).collect();
        let far: Vec<BlockPos> = (-3..0).map(|x| p(x, 5, 2)).collect();
        let mut changed = near.clone();
        changed.extend(far.iter().copied());
        let mut cells = stone_line(0..3, 5, 0);
        cells.extend(stone_line(-3..0, 5, 2));
        let world = move |q: BlockPos| -> Option<BlockId> {
            if q.x >= 3 {
                return None;
            }
            let map: BTreeMap<BlockPos, BlockId> = cells.iter().copied().collect();
            Some(map.get(&q).copied().unwrap_or(builtin::AIR))
        };
        let grounded = |_: BlockPos| false;
        let out = evaluate_load(&world, &grounded, &reg, &changed, 64);
        assert_eq!(out.falling, far, "贴未装载的石条按有地基,独立石条塌");
        assert_eq!(out.visited, 6);
    }

    /// changed 自身也是种子:悬空放置 1 格石头,当拍判塌。
    #[test]
    fn load_changed_cell_itself_is_a_seed() {
        let reg = SimpleRegistry::builtin();
        let world = block_world([(p(0, 3, 0), builtin::STONE)]);
        let grounded = |_: BlockPos| false;
        let out = evaluate_load(&world, &grounded, &reg, &[p(0, 3, 0)], 64);
        assert_eq!(out.falling, vec![p(0, 3, 0)]);
        assert_eq!(out.visited, 1);
        assert_eq!(out.cut_bonds, 0);
    }

    /// cut_bonds 只数 F 与 F 之外 `bond > 0` 实心格之间的连接:土门楣塌左 4 格断 1 根
    /// (dirt–dirt);换成沙门楣逐格塌、一根不断(B = 0)。
    #[test]
    fn load_cut_bonds_counts_only_bonded_solid_outside_falling() {
        let reg = SimpleRegistry::builtin();
        // 右柱 x = 3、y 0..=3 接地;门楣 x = −1..=3、y = 3;左柱已被炸掉(= removed)。
        let removed = [p(-1, 0, 0), p(-1, 1, 0), p(-1, 2, 0)];
        let falling_want: Vec<BlockPos> = (-1..3).map(|x| p(x, 3, 0)).collect();
        for (lintel, cut_want) in [(builtin::DIRT, 1), (builtin::SAND, 0)] {
            let mut cells: Vec<(BlockPos, BlockId)> =
                (0..3).map(|y| (p(3, y, 0), builtin::STONE)).collect();
            cells.extend((-1..=3).map(|x| (p(x, 3, 0), lintel)));
            let anchors: BTreeSet<BlockPos> = (0..3).map(|y| p(3, y, 0)).collect();
            let world = block_world(cells);
            let grounded = move |q: BlockPos| anchors.contains(&q);
            let out = evaluate_load(&world, &grounded, &reg, &removed, 64);
            assert_eq!(out.falling, falling_want, "门楣塌左 4 格");
            assert_eq!(out.cut_bonds, cut_want, "断键数");
            assert_eq!(out.visited, 5, "门楣 5 格都求过流");
        }
    }

    // ------------------------------------------------------------------ split / origin / radius

    /// S = 1 逐格一段;S = 6 时 x = −1 与 x = 0 切进不同块(div_euclid)。
    #[test]
    fn split_size_one_and_negative_coords_use_euclid() {
        let reg = SimpleRegistry::builtin();
        let cells: Vec<BlockPos> = (-3..=3).map(|x| p(x, 0, 0)).collect();
        let world = block_world(stone_line(-3..4, 0, 0));
        let segs = split_segments(&world, &reg, &cells, 1);
        assert_eq!(segs.len(), 7, "S = 1:每格一段");
        let segs = split_segments(&world, &reg, &cells, 6);
        assert_eq!(
            segs,
            vec![
                vec![p(-3, 0, 0), p(-2, 0, 0), p(-1, 0, 0)],
                vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0), p(3, 0, 0)],
            ],
            "块界在 x = 0:负半段与正半段分开"
        );
    }

    /// 同一切块内不相连的格分两段;相邻但 B = 0(沙–石)也分两段。
    #[test]
    fn split_splits_disconnected_and_zero_bond_within_cube() {
        let reg = SimpleRegistry::builtin();
        let all_stone = |_: BlockPos| Some(builtin::STONE);
        let segs = split_segments(&all_stone, &reg, &[p(0, 5, 0), p(2, 5, 0)], 6);
        assert_eq!(
            segs,
            vec![vec![p(0, 5, 0)], vec![p(2, 5, 0)]],
            "不相邻不连通"
        );
        let cells = [(p(0, 5, 0), builtin::STONE), (p(1, 5, 0), builtin::SAND)];
        let world = block_world(cells);
        let segs = split_segments(&world, &reg, &[p(1, 5, 0), p(0, 5, 0)], 6);
        assert_eq!(
            segs,
            vec![vec![p(0, 5, 0)], vec![p(1, 5, 0)]],
            "沙–石 B = 0"
        );
    }

    /// 原点:包围盒中心 `div_euclid(2)`,同距取坐标较小者(截断除法在负数会取错);
    /// 包络半径 = 最远格中心距离 + `DEBRIS_HALF_SIZE·√3`。
    #[test]
    fn origin_takes_smaller_on_tie_and_radius_adds_half_diagonal() {
        assert_eq!(compound_origin(&[p(0, 5, 0), p(1, 5, 0)]), p(0, 5, 0));
        assert_eq!(
            compound_origin(&[p(-2, 5, 0), p(-1, 5, 0)]),
            p(-2, 5, 0),
            "同距取小:截断除法会得 −1"
        );
        assert_eq!(compound_origin(&[p(3, -1, 4)]), p(3, -1, 4));
        let half = DEBRIS_HALF_SIZE * 3.0_f32.sqrt();
        let r = envelope_radius(&[p(0, 5, 0), p(5, 5, 0)], p(2, 5, 0));
        assert!((r - (3.0 + half)).abs() < 1e-4, "6 格梁 ρ = 3 + 0.45√3:{r}");
        let r = envelope_radius(&[p(1, 1, 1)], p(0, 0, 0));
        assert!(
            (r - (3.0_f32.sqrt() + half)).abs() < 1e-4,
            "对角格 ρ = √3 + 0.45√3:{r}"
        );
    }

    // ------------------------------------------------------------------ compound_snap / forced_landing(§8.3,单元 05)

    use types::glam::Vec3;

    fn pair() -> types::CompoundShape {
        types::CompoundShape::new(vec![
            types::ShapeCell {
                offset: [0, 0, 0],
                block: builtin::STONE,
            },
            types::ShapeCell {
                offset: [1, 0, 0],
                block: builtin::STONE,
            },
        ])
        .expect("2 格")
    }

    /// `compound_snap`:lift = 0 时目标格被占 → 直接 `Tilted`(上移从 0 起、
    /// 上限 0 即无路可走)。变异:上移循环从 1 起(跳过 lift 0)会给出上移 1
    /// 的 Write。
    #[test]
    fn compound_snap_lift_zero_occupied_is_tilted() {
        let shape = pair();
        let occupied = |q: BlockPos| q == p(3, 5, 0);
        let out = compound_snap(
            Vec3::new(3.5, 5.5, 0.5),
            types::glam::Quat::IDENTITY,
            &shape,
            0.5,
            45.0,
            0,
            &occupied,
        );
        assert_eq!(out, CompoundSnap::Tilted, "上限 0 且被占:无路可走");
    }

    /// `compound_snap`:误差超限优先于上移——倾斜 20°(末端格偏离 > 0.5 格)
    /// 即使全部目标格空闲也 `Tilted`,不是"先上移成功就当能落"。变异:把误差
    /// 检查挪到上移成功之后,本例会错误地返回 lift 0 的 Write。
    #[test]
    fn compound_snap_error_precedes_lift() {
        // 两格间距 3:旋转 20° 后末端格偏离 3·sin20° ≈ 1.03 > 0.5。
        let shape = types::CompoundShape::new(vec![
            types::ShapeCell {
                offset: [0, 0, 0],
                block: builtin::STONE,
            },
            types::ShapeCell {
                offset: [3, 0, 0],
                block: builtin::STONE,
            },
        ])
        .expect("2 格");
        let free = |_: BlockPos| false;
        let out = compound_snap(
            Vec3::new(3.5, 5.5, 0.5),
            types::glam::Quat::from_rotation_z(20f32.to_radians()),
            &shape,
            0.5,
            45.0,
            2,
            &free,
        );
        assert_eq!(out, CompoundSnap::Tilted, "误差超限:不上移、不写");
    }

    /// `compound_forced_landing`:部分柱下方全空、部分柱有实心时仍 `Write`
    /// ——d 取有底柱的最小下降量,整块停在那一柱的高度上("所有柱**都**没有
    /// 实心才 Fallback")。变异:任一柱无实心就 Fallback 即红。
    #[test]
    fn forced_landing_partial_void_columns_still_write() {
        let shape = pair();
        // 柱 x = 3 有底(world 底),柱 x = 4 全空。
        let solid = |q: BlockPos| -> Option<bool> { Some(q.y <= -64 && q.x == 3) };
        let out = compound_forced_landing(
            Vec3::new(3.5, 5.5, 0.5),
            types::glam::Quat::IDENTITY,
            &shape,
            2,
            &solid,
        );
        assert_eq!(
            out,
            CompoundLanding::Write {
                cells: vec![
                    (p(3, -63, 0), builtin::STONE),
                    (p(4, -63, 0), builtin::STONE)
                ]
            },
            "整块停在有底柱的那层(x=4 的格悬空挂着)"
        );
    }

    /// `compound_forced_landing`:目标格所在柱未装载 → `Defer`(不是当空气
    /// 一路落穿)。变异:把 None 当 false 继续,会给出穿到世界底的 Write。
    #[test]
    fn forced_landing_defers_on_unloaded_target_column() {
        let shape = pair();
        let solid = |q: BlockPos| -> Option<bool> {
            if q.x == 4 {
                None // x = 4 柱未装载
            } else {
                Some(q.y <= -64)
            }
        };
        let out = compound_forced_landing(
            Vec3::new(3.5, 5.5, 0.5),
            types::glam::Quat::IDENTITY,
            &shape,
            2,
            &solid,
        );
        assert_eq!(out, CompoundLanding::Defer, "前置条件不满足 → 推迟");
    }

    /// `compound_forced_landing`:向下扫描到 `WORLD_Y_MIN` 为止——底格只在
    /// 世界底那一层的柱,落点 = `WORLD_Y_MIN + 1`。变异:扫描漏掉世界底层
    /// (少扫一层)会把这柱判成无底。
    #[test]
    fn forced_landing_scans_down_to_world_y_min() {
        let shape = pair();
        let solid = |q: BlockPos| -> Option<bool> { Some(q.y == types::WORLD_Y_MIN) };
        let out = compound_forced_landing(
            Vec3::new(3.5, 5.5, 0.5),
            types::glam::Quat::IDENTITY,
            &shape,
            2,
            &solid,
        );
        assert_eq!(
            out,
            CompoundLanding::Write {
                cells: vec![
                    (p(3, types::WORLD_Y_MIN + 1, 0), builtin::STONE),
                    (p(4, types::WORLD_Y_MIN + 1, 0), builtin::STONE)
                ]
            },
            "落点 = 世界底实心格之上"
        );
    }
}
