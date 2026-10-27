//! cohesion —— 方块粘结与承载判定(REQ-010)。
//!
//! **总架构师接口桩(REQ-010 单元 01)**:公开类型、常量与签名固定,函数体由实施方实现。
//! 规格见 REQ-010 §3–§5.2(修订 1);标准答案由验收用例中的参照实现给出。

use std::collections::{BTreeMap, BTreeSet, VecDeque};

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
    fn send(&mut self, v: usize, t: usize, f: u64, next: &mut [usize]) -> u64 {
        if v == t {
            return f;
        }
        while next[v] < self.adj[v].len() {
            let e = self.adj[v][next[v]];
            let u = self.to[e];
            if self.cap[e] > 0 && self.level[u] == self.level[v].map(|lv| lv + 1) {
                let pushed = self.send(u, t, f.min(self.cap[e]), next);
                if pushed > 0 {
                    self.cap[e] -= pushed;
                    self.cap[e ^ 1] += pushed;
                    return pushed;
                }
            }
            next[v] += 1;
        }
        0
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
}
