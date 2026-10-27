//! REQ-010 方块粘结与整块塌落——验收用例(总架构师编写;实施方只许删 `#[ignore]`)。
//!
//! 按单元分模块;每条用例注明对应的 AC 与"会变红的实现"。

/// 单元 01:粘结属性与承载求值纯函数(AC-C1、AC-C2、AC-C3 的纯函数部分)。
mod unit01 {
    use physics_sim::cohesion::{
        LoadOutcome, bond_between, compound_origin, envelope_radius, evaluate_load,
        impact_bond_between, is_grounded_by_scan, split_segments,
    };
    use physics_sim::destruction::find_collapse;
    use std::collections::{BTreeMap, BTreeSet, VecDeque};
    use types::{BlockId, BlockPos, WORLD_Y_MIN};
    use voxel::{BOND_INFINITE, BlockRegistry, Material, SimpleRegistry, Support, builtin};

    const STONE: BlockId = builtin::STONE;
    const DIRT: BlockId = builtin::DIRT;
    const SAND: BlockId = builtin::SAND;
    const BEDROCK: BlockId = builtin::BEDROCK;
    const AIR: BlockId = builtin::AIR;
    const MAX_CELLS: usize = 2048;

    const D6: [(i32, i32, i32); 6] = [
        (1, 0, 0),
        (-1, 0, 0),
        (0, 1, 0),
        (0, -1, 0),
        (0, 0, 1),
        (0, 0, -1),
    ];

    fn p(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    fn add(a: BlockPos, d: (i32, i32, i32)) -> BlockPos {
        p(a.x + d.0, a.y + d.1, a.z + d.2)
    }

    fn sorted(cells: impl IntoIterator<Item = BlockPos>) -> Vec<BlockPos> {
        let set: BTreeSet<BlockPos> = cells.into_iter().collect();
        set.into_iter().collect()
    }

    fn line_x(xs: std::ops::Range<i32>, y: i32, z: i32) -> Vec<BlockPos> {
        xs.map(|x| p(x, y, z)).collect()
    }

    fn column(x: i32, ys: std::ops::Range<i32>, z: i32) -> Vec<BlockPos> {
        ys.map(|y| p(x, y, z)).collect()
    }

    // ------------------------------------------------------------------ 世界

    /// 超平坦(与 REQ-009 demo 同):y = −4 基岩、−3..=−1 土、其上与其下为空气;`cells` 覆盖;
    /// `unloaded` 中的格与 x ≥ `unloaded_from_x` 的格所在 chunk 视为未装载。
    struct W {
        cells: BTreeMap<BlockPos, BlockId>,
        unloaded: BTreeSet<BlockPos>,
        unloaded_from_x: Option<i32>,
    }

    impl W {
        fn new() -> Self {
            Self {
                cells: BTreeMap::new(),
                unloaded: BTreeSet::new(),
                unloaded_from_x: None,
            }
        }

        fn fill(&mut self, cells: impl IntoIterator<Item = BlockPos>, block: BlockId) {
            for c in cells {
                self.cells.insert(c, block);
            }
        }

        fn clear(&mut self, cells: impl IntoIterator<Item = BlockPos>) {
            self.fill(cells, AIR);
        }

        fn at(&self, q: BlockPos) -> Option<BlockId> {
            if self.unloaded.contains(&q) || self.unloaded_from_x.is_some_and(|x| q.x >= x) {
                return None;
            }
            if let Some(b) = self.cells.get(&q) {
                return Some(*b);
            }
            Some(match q.y {
                -4 => BEDROCK,
                -3..=-1 => DIRT,
                _ => AIR,
            })
        }
    }

    // ------------------------------------------------------------------ 参照实现

    fn ref_bond(reg: &dyn BlockRegistry, a: BlockId, b: BlockId) -> u32 {
        match (reg.material(a), reg.material(b)) {
            (Some(x), Some(y)) => x.bond.min(y.bond),
            _ => 0,
        }
    }

    fn ref_grounded(
        world: &dyn Fn(BlockPos) -> Option<BlockId>,
        reg: &dyn BlockRegistry,
        pos: BlockPos,
    ) -> bool {
        let mut q = pos;
        loop {
            let Some(b) = world(q) else { return true };
            let Some(m) = reg.material(b) else {
                return false;
            };
            if m.support == Support::Anchor || q.y == WORLD_Y_MIN {
                return true;
            }
            q.y -= 1;
        }
    }

    const INF: u64 = u64::MAX / 4;

    /// 邻接表残量网络 + Edmonds–Karp(与实现的算法无关:F 唯一)。
    struct Net {
        adj: Vec<Vec<usize>>,
        to: Vec<usize>,
        cap: Vec<u64>,
    }

    impl Net {
        fn new(n: usize) -> Self {
            Self {
                adj: vec![Vec::new(); n],
                to: Vec::new(),
                cap: Vec::new(),
            }
        }

        fn edge(&mut self, u: usize, v: usize, c: u64) {
            self.adj[u].push(self.to.len());
            self.to.push(v);
            self.cap.push(c);
            self.adj[v].push(self.to.len());
            self.to.push(u);
            self.cap.push(0);
        }

        fn max_flow(&mut self, s: usize, t: usize) {
            loop {
                let mut prev: Vec<Option<usize>> = vec![None; self.adj.len()];
                let mut seen = vec![false; self.adj.len()];
                seen[s] = true;
                let mut q = VecDeque::from([s]);
                while let Some(u) = q.pop_front() {
                    for &e in &self.adj[u] {
                        let v = self.to[e];
                        if !seen[v] && self.cap[e] > 0 {
                            seen[v] = true;
                            prev[v] = Some(e);
                            q.push_back(v);
                        }
                    }
                }
                if !seen[t] {
                    return;
                }
                let mut bottleneck = INF;
                let mut v = t;
                while let Some(e) = prev[v] {
                    bottleneck = bottleneck.min(self.cap[e]);
                    v = self.to[e ^ 1];
                }
                let mut v = t;
                while let Some(e) = prev[v] {
                    self.cap[e] -= bottleneck;
                    self.cap[e ^ 1] += bottleneck;
                    v = self.to[e ^ 1];
                }
            }
        }

        fn reachable(&self, s: usize) -> Vec<bool> {
            let mut seen = vec![false; self.adj.len()];
            seen[s] = true;
            let mut q = VecDeque::from([s]);
            while let Some(u) = q.pop_front() {
                for &e in &self.adj[u] {
                    let v = self.to[e];
                    if !seen[v] && self.cap[e] > 0 {
                        seen[v] = true;
                        q.push_back(v);
                    }
                }
            }
            seen
        }
    }

    /// REQ-010 §4 的逐字参照实现(规格见 `evaluate_load` 的文档注释)。
    fn reference(
        world: &dyn Fn(BlockPos) -> Option<BlockId>,
        reg: &dyn BlockRegistry,
        changed: &[BlockPos],
        max_cells: usize,
    ) -> LoadOutcome {
        let solid = |q: BlockPos| world(q).is_some_and(|b| reg.material(b).is_some());
        let overhang = |q: BlockPos| solid(q) && !ref_grounded(world, reg, q);
        let mut seeds = BTreeSet::new();
        for &c in changed {
            for q in std::iter::once(c).chain(D6.iter().map(|&d| add(c, d))) {
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
            let mut enqueued = BTreeSet::from([seed]);
            let mut order: Vec<BlockPos> = Vec::new();
            let (mut stable, mut over) = (false, false);
            'bfs: while let Some(c) = queue.pop_front() {
                order.push(c);
                if order.len() > max_cells {
                    stable = true;
                    over = true;
                    break;
                }
                for d in D6 {
                    let n = add(c, d);
                    if budget_marked.contains(&n) {
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
                if over {
                    budget_stable += 1;
                }
                budget_marked.extend(enqueued);
                continue;
            }
            visited += order.len();
            let idx: BTreeMap<BlockPos, usize> =
                order.iter().enumerate().map(|(i, c)| (*c, i + 2)).collect();
            let (src, sink) = (0, 1);
            let mut net = Net::new(order.len() + 2);
            for &a in &order {
                let u = idx[&a];
                net.edge(src, u, 1);
                let ba = world(a).expect("连通块内的格已装载");
                for d in D6 {
                    let b = add(a, d);
                    let Some(bb) = world(b) else {
                        net.edge(u, sink, INF);
                        continue;
                    };
                    if reg.material(bb).is_none() {
                        continue;
                    }
                    let cap = if d == (0, -1, 0) {
                        INF
                    } else {
                        match ref_bond(reg, ba, bb) {
                            0 => continue,
                            BOND_INFINITE => INF,
                            x => u64::from(x),
                        }
                    };
                    match idx.get(&b) {
                        Some(&v) => net.edge(u, v, cap),
                        None => net.edge(u, sink, cap),
                    }
                }
            }
            net.max_flow(src, sink);
            let reach = net.reachable(src);
            for &c in &order {
                if reach[idx[&c]] {
                    falling.insert(c);
                }
            }
            done.extend(order);
        }
        let mut cut_bonds = 0;
        for &c in &falling {
            let bc = world(c).expect("F 中的格已装载");
            for d in D6 {
                let n = add(c, d);
                if falling.contains(&n) {
                    continue;
                }
                if let Some(bn) = world(n)
                    && reg.material(bn).is_some()
                    && ref_bond(reg, bc, bn) > 0
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

    /// 求值并核对:与参照实现逐字段一致;把 `changed` 倒序、轮转后传入结果相同(与顺序无关)。
    fn eval_with(
        w: &W,
        reg: &SimpleRegistry,
        changed: &[BlockPos],
        max_cells: usize,
    ) -> LoadOutcome {
        let world = |q: BlockPos| w.at(q);
        let grounded = |q: BlockPos| is_grounded_by_scan(&world, reg, q);
        let out = evaluate_load(&world, &grounded, reg, changed, max_cells);
        let want = reference(&world, reg, changed, max_cells);
        assert_eq!(
            out, want,
            "与参照实现一致(F、visited、budget_stable、cut_bonds)"
        );
        let mut rev = changed.to_vec();
        rev.reverse();
        assert_eq!(
            evaluate_load(&world, &grounded, reg, &rev, max_cells),
            out,
            "与 changed 的顺序无关(倒序)"
        );
        if changed.len() > 2 {
            let mut rot = changed.to_vec();
            rot.rotate_left(changed.len() / 3);
            assert_eq!(
                evaluate_load(&world, &grounded, reg, &rot, max_cells),
                out,
                "与 changed 的顺序无关(轮转)"
            );
        }
        out
    }

    fn eval(w: &W, changed: &[BlockPos]) -> LoadOutcome {
        eval_with(w, &SimpleRegistry::builtin(), changed, MAX_CELLS)
    }

    // ------------------------------------------------------------------ 结构

    /// 门框:柱 x = −3 / 3 高 5,门楣 x = −3..=3、y = 5;移走左柱。返回 (世界, 移走的格)。
    fn door(lintel: BlockId) -> (W, Vec<BlockPos>) {
        let mut w = W::new();
        w.fill(column(-3, 0..5, 0), STONE);
        w.fill(column(3, 0..5, 0), STONE);
        w.fill(line_x(-3..4, 5, 0), lintel);
        let removed = column(-3, 0..5, 0);
        w.clear(removed.clone());
        (w, removed)
    }

    /// 石墙(x −3..=−1、z −1..=1)伸出的悬臂,y = 5 起,x = 0..L,厚 `thick`;返回 (世界, 悬臂格)。
    fn cantilever(len: i32, thick: i32, block: BlockId) -> (W, Vec<BlockPos>) {
        let mut w = W::new();
        for x in -3..0 {
            for z in -1..=1 {
                w.fill(column(x, 0..5 + thick, z), STONE);
            }
        }
        let arm: Vec<BlockPos> = (0..len)
            .flat_map(|x| (0..thick).map(move |j| p(x, 5 + j, 0)))
            .collect();
        w.fill(arm.clone(), block);
        (w, arm)
    }

    /// 20×20 石平台(y = 6,x、z ∈ −10..10),4 根柱在 (−9,−9)、(−9,8)、(8,−9)、(8,8)。
    fn platform() -> (W, Vec<BlockPos>) {
        let mut w = W::new();
        let plat: Vec<BlockPos> = (-10..10)
            .flat_map(|x| (-10..10).map(move |z| p(x, 6, z)))
            .collect();
        for (x, z) in [(-9, -9), (-9, 8), (8, -9), (8, 8)] {
            w.fill(column(x, 0..6, z), STONE);
        }
        w.fill(plat.clone(), STONE);
        (w, plat)
    }

    /// 1 格厚石板 y = 6,(2h+1)² 见方,四边下方有墙(y 0..6)。返回 (世界, 板格)。
    fn edge_slab(h: i32) -> (W, Vec<BlockPos>) {
        let mut w = W::new();
        for i in -h..=h {
            for (x, z) in [(i, -h), (i, h), (-h, i), (h, i)] {
                w.fill(column(x, 0..6, z), STONE);
            }
        }
        let slab: Vec<BlockPos> = (-h..=h)
            .flat_map(|x| (-h..=h).map(move |z| p(x, 6, z)))
            .collect();
        w.fill(slab.clone(), STONE);
        (w, slab)
    }

    /// 实心土层 y 0..=5 覆盖 x ∈ −20..=20、z ∈ zs;挖出 x 0..wid、z ∈ hole_z、y 0..=1 的洞。
    fn dug(
        wid: i32,
        zs: std::ops::RangeInclusive<i32>,
        hole_z: std::ops::Range<i32>,
    ) -> (W, Vec<BlockPos>) {
        let mut w = W::new();
        for x in -20..=20 {
            for z in zs.clone() {
                w.fill(column(x, 0..6, z), DIRT);
            }
        }
        let hole: Vec<BlockPos> = (0..wid)
            .flat_map(|x| {
                hole_z
                    .clone()
                    .flat_map(move |z| (0..2).map(move |y| p(x, y, z)))
            })
            .collect();
        w.clear(hole.clone());
        (w, hole)
    }

    // ================================================================== AC-C1

    /// AC-C1:内置方块的承载粘结 B 与撞击粘结 I(REQ-010 §3.2);`register` 注册的实心方块 B = ∞;
    /// 方块 ID 不变。
    ///
    /// 会变红的实现:数值抄错;`register` 的默认材料 B 不是 ∞(既有用例的自定义方块会突然开始断);
    /// 改了内置方块的 ID 顺序。
    #[test]
    fn acc_c1_builtin_bonds() {
        let reg = SimpleRegistry::builtin();
        let table = [
            (STONE, "stone", 8, 18.0),
            (DIRT, "dirt", 3, 7.0),
            (BEDROCK, "bedrock", BOND_INFINITE, f32::INFINITY),
            (SAND, "sand", 0, 4.0),
        ];
        for (id, name, bond, impact) in table {
            assert_eq!(reg.id_of(name), Some(id), "{name} 的 ID 不变");
            let m = reg.material(id).expect("实心方块有材料");
            assert_eq!(m.bond, bond, "{name} 的 B");
            assert_eq!(m.impact_threshold, impact, "{name} 的 I");
        }
        assert_eq!(
            (STONE, DIRT, BEDROCK, SAND),
            (BlockId(1), BlockId(2), BlockId(3), BlockId(4))
        );
        let mut custom = SimpleRegistry::new();
        let x = custom.register("x", true).expect("注册");
        assert_eq!(
            custom.material(x).map(|m| m.bond),
            Some(BOND_INFINITE),
            "未指定材料 B = ∞"
        );
        assert_eq!(Material::INDESTRUCTIBLE.bond, BOND_INFINITE);
    }

    /// AC-C1:两格之间取较小值;任一侧不是实心方块为 0。
    ///
    /// 会变红的实现:取了较大值或和;空气、未知 ID 没有返回 0。
    #[test]
    fn acc_c1_pairwise_min() {
        let reg = SimpleRegistry::builtin();
        assert_eq!(bond_between(&reg, STONE, STONE), 8);
        assert_eq!(bond_between(&reg, STONE, DIRT), 3);
        assert_eq!(bond_between(&reg, DIRT, STONE), 3);
        assert_eq!(bond_between(&reg, STONE, SAND), 0);
        assert_eq!(bond_between(&reg, STONE, BEDROCK), 8);
        assert_eq!(bond_between(&reg, BEDROCK, BEDROCK), BOND_INFINITE);
        assert_eq!(bond_between(&reg, STONE, AIR), 0);
        assert_eq!(bond_between(&reg, STONE, BlockId(99)), 0);
        assert_eq!(impact_bond_between(&reg, STONE, STONE), 18.0);
        assert_eq!(impact_bond_between(&reg, STONE, DIRT), 7.0);
        assert_eq!(impact_bond_between(&reg, SAND, DIRT), 4.0);
        assert_eq!(impact_bond_between(&reg, STONE, BEDROCK), 18.0);
        assert_eq!(impact_bond_between(&reg, STONE, AIR), 0.0);
    }

    // ================================================================== AC-C2

    /// AC-C2:接地判定的参考实现(§4.2):整列实心直到 Anchor 或世界底;遇到未装载按有地基。
    ///
    /// 会变红的实现:只认世界底、不认悬空的基岩(与 REQ-009 的锚定口径不一致);未装载按无地基;
    /// 起点本身是空气却返回 true。
    #[test]
    fn acc_c2_grounded_scan() {
        let reg = SimpleRegistry::builtin();
        let mut w = W::new();
        w.fill([p(0, 0, 0)], STONE);
        w.fill([p(1, 2, 0)], STONE);
        w.fill([p(2, 10, 0)], BEDROCK);
        w.fill([p(2, 11, 0), p(2, 12, 0)], STONE);
        w.fill([p(5, 3, 0)], STONE);
        w.unloaded.insert(p(5, 2, 0));
        w.fill([p(7, WORLD_Y_MIN, 0), p(7, WORLD_Y_MIN + 1, 0)], STONE);
        w.unloaded_from_x = Some(30);
        let world = |q: BlockPos| w.at(q);
        let g = |q: BlockPos| is_grounded_by_scan(&world, &reg, q);
        assert!(g(p(0, -1, 0)), "地层土");
        assert!(g(p(0, -4, 0)), "基岩本身");
        assert!(g(p(0, 0, 0)), "压在地面上的石头");
        assert!(!g(p(0, 1, 0)), "空气");
        assert!(!g(p(1, 2, 0)), "下方是空气");
        assert!(g(p(2, 10, 0)), "悬空的基岩是地基");
        assert!(g(p(2, 12, 0)), "压在悬空基岩上的一列");
        assert!(g(p(5, 3, 0)), "下方未装载按有地基");
        assert!(g(p(7, WORLD_Y_MIN + 1, 0)), "整列实心到世界底");
        assert!(g(p(31, 50, 0)), "起点所在 chunk 未装载按有地基");
    }

    /// AC-C2 门框(§4.3 例):土门楣移走左柱,左边 6 格整块断下(断 1 根粘结);石门楣挂得住。
    ///
    /// 会变红的实现:只掉超载的那几格;粘结容量用了较大值;F 漏掉被吊着的格。
    #[test]
    fn acc_c2_door_lintel() {
        let (w, removed) = door(DIRT);
        let out = eval(&w, &removed);
        assert_eq!(out.falling, line_x(-3..3, 5, 0), "土门楣左 6 格");
        assert_eq!(out.cut_bonds, 1);
        assert_eq!(out.visited, 6);
        let (w, removed) = door(STONE);
        assert!(eval(&w, &removed).falling.is_empty(), "石门楣挂得住");
    }

    /// AC-C2 悬臂(§4.5):1 格厚的悬臂上限恰好等于 B,加厚不能加长;超载时整根从根部断下。
    /// 放置即触发(D8):触发集为悬臂自身。
    ///
    /// 会变红的实现:上限不等于 B(容量或网络方向写错);厚 2 能挂更长(把正下方也限了量或漏了
    /// 正上方的拉);超载只掉末端几格;放置不触发。
    #[test]
    fn acc_c2_cantilever_limit_equals_bond() {
        for (block, bond, name) in [(STONE, 8, "石"), (DIRT, 3, "土")] {
            for thick in [1, 2] {
                for len in 1..=bond + 3 {
                    let (w, arm) = cantilever(len, thick, block);
                    let out = eval(&w, &arm);
                    if len <= bond {
                        assert!(
                            out.falling.is_empty(),
                            "{name}悬臂 厚{thick} 长{len} 挂得住"
                        );
                    } else {
                        assert_eq!(
                            out.falling,
                            sorted(arm),
                            "{name}悬臂 厚{thick} 长{len} 整根断下"
                        );
                    }
                }
            }
        }
    }

    /// AC-C2 T 形横梁与两端支撑的梁(§4.5):每侧 ≤ B、净跨 ≤ 2B。
    ///
    /// 会变红的实现:两侧共用一个容量;梁只计一端的容量;净跨超限时只掉中间几格。
    #[test]
    fn acc_c2_tbeam_and_two_end_beam() {
        for k in 3..=10 {
            let mut w = W::new();
            w.fill(column(0, 0..5, 0), STONE);
            let beam = line_x(-k..k + 1, 5, 0);
            w.fill(beam.clone(), STONE);
            let out = eval(&w, &beam);
            if k <= 8 {
                assert!(out.falling.is_empty(), "T 形每侧 {k} 挂得住");
            } else {
                let want: Vec<BlockPos> = beam.into_iter().filter(|c| c.x != 0).collect();
                assert_eq!(out.falling, sorted(want), "T 形每侧 {k} 两侧都断");
                assert_eq!(out.cut_bonds, 2);
            }
        }
        for (deck_block, max_span, name) in [(STONE, 16, "石"), (DIRT, 6, "土")] {
            for span in [max_span - 1, max_span, max_span + 1] {
                let mut w = W::new();
                w.fill(column(-1, 0..5, 0), STONE);
                w.fill(column(span, 0..5, 0), STONE);
                let deck = line_x(-1..span + 1, 5, 0);
                w.fill(deck.clone(), deck_block);
                let out = eval(&w, &deck);
                if span <= max_span {
                    assert!(out.falling.is_empty(), "{name}梁净跨 {span} 挂得住");
                } else {
                    assert_eq!(
                        out.falling,
                        line_x(0..span, 5, 0),
                        "{name}梁净跨 {span} 整根落下"
                    );
                }
            }
        }
    }

    /// AC-C2 拱:完好、移走拱顶都稳定;移走一条拱腿,剩下的悬挑部分整段断下(10 格)。
    ///
    /// 会变红的实现:向上的边当成了不限量(拱腿移走后仍挂住);向下的边受 B 限制。
    #[test]
    fn acc_c2_arch() {
        let cells: Vec<BlockPos> = (-4..=4)
            .flat_map(|x| (0..=4).map(move |y| p(x, y, 0)))
            .filter(|c| (8..=16).contains(&(c.x * c.x + c.y * c.y)))
            .collect();
        let mut w = W::new();
        w.fill(cells.clone(), STONE);
        assert!(eval(&w, &cells).falling.is_empty(), "完好的拱");
        let mut no_key = W::new();
        no_key.fill(cells.clone(), STONE);
        no_key.clear([p(0, 4, 0)]);
        assert!(eval(&no_key, &[p(0, 4, 0)]).falling.is_empty(), "移走拱顶");
        let mut no_leg = W::new();
        no_leg.fill(cells, STONE);
        let leg = [p(-4, 0, 0), p(-3, 0, 0)];
        no_leg.clear(leg);
        let out = eval(&no_leg, &leg);
        assert_eq!(out.falling.len(), 10, "移走左腿:悬挑的 10 格断下");
        assert!(out.falling.iter().all(|c| c.y >= 1), "地面上的右腿不动");
    }

    /// AC-C2 加重(D8)与塔:满载的 8 格石悬臂末端再放 1/2/4 格石头或 1 格沙,整根连同放上去的格落下;
    /// 1×1×20 的塔切掉底格,上面 19 格落下。
    ///
    /// 会变红的实现:加重的写入不触发;放上去的格不进 F;沙子压不断(沙也有重量);塔判稳定。
    #[test]
    fn acc_c2_weight_and_tower() {
        for (extra, block) in [(1, STONE), (2, STONE), (4, STONE), (1, SAND)] {
            let (mut w, arm) = cantilever(8, 1, STONE);
            assert!(eval(&w, &arm).falling.is_empty(), "8 格石悬臂恰好挂住");
            let load = column(7, 6..6 + extra, 0);
            w.fill(load.clone(), block);
            let out = eval(&w, &load);
            assert_eq!(
                out.falling,
                sorted(arm.into_iter().chain(load)),
                "加重 {extra} 格"
            );
            assert_eq!(out.cut_bonds, 1, "从根部断开");
        }
        let mut w = W::new();
        w.fill(column(0, 0..20, 0), STONE);
        w.clear([p(0, 0, 0)]);
        let out = eval(&w, &[p(0, 0, 0)]);
        assert_eq!(out.falling, column(0, 1..20, 0));
        assert_eq!(out.cut_bonds, 0);
    }

    /// AC-C2 沙子(B = 0):贴墙放的沙下方一空就落;沙柱底下的石头移走,整列沙落下(与 REQ-009 终态相同)。
    ///
    /// 会变红的实现:沙与墙之间有粘结;沙柱只掉最底下一格。
    #[test]
    fn acc_c2_sand() {
        let mut w = W::new();
        w.fill(column(0, 0..3, 0), STONE);
        w.fill([p(1, 1, 0)], SAND);
        let out = eval(&w, &[p(1, 1, 0)]);
        assert_eq!(out.falling, vec![p(1, 1, 0)], "贴墙的沙");
        assert_eq!(out.cut_bonds, 0);
        let mut w = W::new();
        w.fill([p(0, 0, 0)], STONE);
        let sand = column(0, 1..4, 0);
        w.fill(sand.clone(), SAND);
        assert!(eval(&w, &sand).falling.is_empty(), "沙柱压在石头上");
        w.clear([p(0, 0, 0)]);
        let out = eval(&w, &[p(0, 0, 0)]);
        assert_eq!(out.falling, sand, "整列沙同拍落下");
    }

    /// AC-C2 点支撑平台(§4.5):4 根 1×1 柱只能托 128 格,20×20 石平台满员即判塌——柱顶 4 格之外全部落下。
    ///
    /// 会变红的实现:接地格之间也建了边(容量算多);柱顶格算作悬挑。
    #[test]
    fn acc_c2_point_supported_platform_collapses() {
        let (w, plat) = platform();
        let out = eval(&w, &plat);
        let legs = [p(-9, 6, -9), p(-9, 6, 8), p(8, 6, -9), p(8, 6, 8)];
        let want: Vec<BlockPos> = plat.into_iter().filter(|c| !legs.contains(c)).collect();
        assert_eq!(out.falling, sorted(want), "柱顶之外 396 格");
        assert_eq!(out.cut_bonds, 16);
        assert_eq!(out.visited, 396);
    }

    /// AC-C2 地下洞室(§4.5):土房间 12×12 挂得住、13×13 房顶到地表整片落下;很长的土隧道宽 6 挂得住、7 塌。
    ///
    /// 会变红的实现:上层的重量只能经下层传出(层与层之间向上的边漏了);接地判定把房顶当作接地。
    #[test]
    fn acc_c2_rooms_and_tunnels() {
        let (w, hole) = dug(12, -20..=20, 0..12);
        assert!(eval(&w, &hole).falling.is_empty(), "土房间 12×12");
        let (w, hole) = dug(13, -20..=20, 0..13);
        let out = eval(&w, &hole);
        assert_eq!(
            out.falling.len(),
            13 * 13 * 4,
            "土房间 13×13:房顶 4 层整片落下"
        );
        assert!(out.falling.iter().all(|c| (2..=5).contains(&c.y)));
        let (w, hole) = dug(6, -35..=35, -30..31);
        assert!(eval(&w, &hole).falling.is_empty(), "土隧道宽 6");
        let (w, hole) = dug(7, -35..=35, -30..31);
        assert_eq!(eval(&w, &hole).falling.len(), 7 * 61 * 4, "土隧道宽 7");
    }

    /// AC-C2 预算与未装载:31×31 四边支撑的石板挂得住;49×49(悬挑 2209 > 2048)按稳定处理且计 1 次,
    /// 同一拍另一个独立的悬空块照常判塌;紧贴未装载 chunk 的悬空石条按有地基。弹坑地面不进最大流。
    ///
    /// 会变红的实现:预算按全部种子合计(大块把小块也判稳定了);超预算的块被判塌;未装载按空气;
    /// 弹坑周围的接地格被当作悬挑(visited 不为 0)。
    #[test]
    fn acc_c2_budget_unloaded_and_crater() {
        let (w, slab) = edge_slab(15);
        let out = eval(&w, &slab);
        assert!(out.falling.is_empty(), "31×31 四边支撑");
        assert_eq!(out.visited, 29 * 29);
        let (mut w, slab) = edge_slab(24);
        let bar = line_x(40..43, 10, 0);
        w.fill(bar.clone(), STONE);
        let changed: Vec<BlockPos> = slab.into_iter().chain(bar.clone()).collect();
        let out = eval(&w, &changed);
        assert_eq!(out.budget_stable, 1, "49×49 超预算");
        assert_eq!(out.falling, bar, "独立的小块照常判塌");

        let mut w = W::new();
        w.unloaded_from_x = Some(30);
        let near = line_x(25..30, 10, 0);
        let far = line_x(20..25, 10, 2);
        w.fill(near.clone(), STONE);
        w.fill(far.clone(), STONE);
        let changed: Vec<BlockPos> = near.into_iter().chain(far.clone()).collect();
        assert_eq!(eval(&w, &changed).falling, far, "紧贴未装载的石条按有地基");

        let mut w = W::new();
        let bowl: Vec<BlockPos> = (-3..=3)
            .flat_map(|x| (-3..=-1).flat_map(move |y| (-3..=3).map(move |z| p(x, y, z))))
            .filter(|c| c.x * c.x + c.y * c.y + c.z * c.z <= 9)
            .collect();
        w.clear(bowl.clone());
        let out = eval(&w, &bowl);
        assert!(out.falling.is_empty());
        assert_eq!(out.visited, 0, "弹坑地面没有悬挑格");
    }

    /// AC-C2 退化:所有 B 为 ∞ 时,F 与 REQ-009 `find_collapse` 的塌落集完全一致(无沙、不触预算)。
    ///
    /// 会变红的实现:接地判定与 REQ-009 的锚定口径不一致;未装载或世界底的处理不同。
    #[test]
    fn acc_c2_infinite_bonds_match_find_collapse() {
        let mut reg = SimpleRegistry::new();
        let inf = |support| Material {
            resistance: f32::INFINITY,
            impact_threshold: f32::INFINITY,
            support,
            pulverize_factor: 2.0,
            bond: BOND_INFINITE,
        };
        assert_eq!(
            reg.register_with_material("stone", inf(Support::Connected)),
            Ok(STONE)
        );
        assert_eq!(
            reg.register_with_material("dirt", inf(Support::Connected)),
            Ok(DIRT)
        );
        assert_eq!(
            reg.register_with_material("bedrock", inf(Support::Anchor)),
            Ok(BEDROCK)
        );

        let mut cases: Vec<(&str, W, Vec<BlockPos>)> = Vec::new();
        let bridge = |pillars: &[i32]| {
            let mut w = W::new();
            w.fill(column(0, 0..5, 0), STONE);
            w.fill(column(11, 0..5, 0), STONE);
            w.fill(line_x(0..12, 5, 0), STONE);
            let removed: Vec<BlockPos> = pillars.iter().flat_map(|&x| column(x, 0..5, 0)).collect();
            w.clear(removed.clone());
            (w, removed)
        };
        let (w, r) = bridge(&[0]);
        cases.push(("桥炸一根柱", w, r));
        let (w, r) = bridge(&[0, 11]);
        cases.push(("桥炸两根柱", w, r));
        let (mut w, _) = cantilever(7, 1, STONE);
        let root = column(-1, 0..6, 0);
        w.clear(root.clone());
        cases.push(("悬臂切根", w, root));
        let mut w = W::new();
        w.fill(column(0, 0..20, 0), STONE);
        w.clear([p(0, 0, 0)]);
        cases.push(("塔切底", w, vec![p(0, 0, 0)]));
        let (mut w, _) = platform();
        let legs: Vec<BlockPos> = [(-9, -9), (-9, 8), (8, -9), (8, 8)]
            .into_iter()
            .flat_map(|(x, z)| column(x, 0..6, z))
            .collect();
        w.clear(legs.clone());
        cases.push(("平台 4 柱全断", w, legs));
        let mut w = W::new();
        w.fill([p(0, 0, 0)], STONE);
        w.clear([p(0, -1, 0)]);
        cases.push(("地层掏空一格", w, vec![p(0, -1, 0)]));

        for (name, w, removed) in cases {
            let world = |q: BlockPos| w.at(q);
            let collapse = find_collapse(&world, &reg, &removed, 100_000, 100_000).falling;
            let out = eval_with(&w, &reg, &removed, 100_000);
            assert_eq!(
                out.falling, collapse,
                "{name}:B = ∞ 时与 find_collapse 一致"
            );
        }
    }

    /// AC-C2 随机结构:300 个固定种子生成的混合材料结构(含放置与移除),与参照实现逐字段一致、
    /// 与触发顺序无关。
    ///
    /// 会变红的实现:任何与 §4.3 网络规格不一致的细节(边的方向、容量、接地判定、种子范围)。
    #[test]
    fn acc_c2_random_structures_match_reference() {
        let mut state: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let mut nonempty = 0;
        for _ in 0..300 {
            let mut w = W::new();
            for x in 0..8 {
                for y in 0..6 {
                    for z in 0..3 {
                        let r = next() % 100;
                        let block = match r {
                            0..=32 => STONE,
                            33..=49 => DIRT,
                            50..=55 => SAND,
                            _ => continue,
                        };
                        w.fill([p(x, y, z)], block);
                    }
                }
            }
            let mut changed = Vec::new();
            for _ in 0..4 {
                let c = p(
                    (next() % 8) as i32,
                    (next() % 6) as i32,
                    (next() % 3) as i32,
                );
                let now = w.at(c).expect("已装载");
                w.fill([c], if now == AIR { STONE } else { AIR });
                changed.push(c);
            }
            if !eval(&w, &changed).falling.is_empty() {
                nonempty += 1;
            }
        }
        assert!(
            nonempty >= 30,
            "随机样本里要有足够多的塌落情形(实际 {nonempty})"
        );
    }

    // ================================================================== AC-C3

    fn split(cells: &[BlockPos], blocks: &W, s: i32) -> Vec<Vec<BlockPos>> {
        let reg = SimpleRegistry::builtin();
        let world = |q: BlockPos| blocks.at(q);
        let segs = split_segments(&world, &reg, cells, s);
        let total: usize = segs.iter().map(Vec::len).sum();
        assert_eq!(total, sorted(cells.iter().copied()).len(), "格数守恒");
        for seg in &segs {
            assert!(seg.windows(2).all(|w| w[0] < w[1]), "段内升序");
            assert!(seg.len() <= (s * s * s) as usize, "段的格数 ≤ S³");
            let o = compound_origin(seg);
            let bound = (s as f32 / 2.0) * 3f32.sqrt() + 1.65;
            assert!(
                envelope_radius(seg, o) <= bound,
                "包络半径 ≤ (S/2)·√3 + 1.65"
            );
        }
        assert!(segs.windows(2).all(|w| w[0][0] < w[1][0]), "各段按首格升序");
        segs
    }

    /// AC-C3:F 按粘结连通 + 世界对齐的切块网格分段——门楣 6 格跨 x = 0 分成 3 + 3;20 格桥面按 S = 6/4/7
    /// 分成 6/6/6/2、4×5、7/7/6;沙子逐格;不同材料之间按粘结判断;同一切块内靠块外连接的两部分分开。
    ///
    /// 会变红的实现:切块网格不是世界对齐(按 F 自身的包围盒切);负坐标用了截断除法;沙子与石头成块;
    /// 只按网格切、没有在块内再按粘结连通划分。
    #[test]
    fn acc_c3_split_segments() {
        let mut w = W::new();
        let lintel = line_x(-3..3, 5, 0);
        w.fill(lintel.clone(), DIRT);
        assert_eq!(
            split(&lintel, &w, 6),
            vec![line_x(-3..0, 5, 0), line_x(0..3, 5, 0)],
            "门楣跨 x = 0"
        );
        let mut w = W::new();
        let deck = line_x(0..20, 5, 0);
        w.fill(deck.clone(), STONE);
        for (s, sizes) in [(6, vec![6, 6, 6, 2]), (4, vec![4; 5]), (7, vec![7, 7, 6])] {
            let got: Vec<usize> = split(&deck, &w, s).iter().map(Vec::len).collect();
            assert_eq!(got, sizes, "20 格桥面 S = {s}");
        }
        let mut w = W::new();
        let sand = column(0, 1..4, 0);
        w.fill(sand.clone(), SAND);
        assert_eq!(split(&sand, &w, 6).len(), 3, "沙子逐格");
        let mut w = W::new();
        w.fill([p(0, 5, 0), p(1, 5, 0)], STONE);
        w.fill([p(2, 5, 0)], SAND);
        assert_eq!(
            split(&[p(2, 5, 0), p(0, 5, 0), p(1, 5, 0)], &w, 6),
            vec![vec![p(0, 5, 0), p(1, 5, 0)], vec![p(2, 5, 0)]],
            "石与沙之间没有粘结"
        );
        let mut w = W::new();
        let u: Vec<BlockPos> = [p(0, 4, 0), p(0, 5, 0), p(5, 4, 0), p(5, 5, 0)]
            .into_iter()
            .chain(line_x(0..6, 6, 0))
            .collect();
        w.fill(u.clone(), STONE);
        assert_eq!(
            split(&u, &w, 6),
            vec![
                vec![p(0, 4, 0), p(0, 5, 0)],
                line_x(0..6, 6, 0),
                vec![p(5, 4, 0), p(5, 5, 0)],
            ],
            "U 形:两条腿在 y < 6 的切块内只靠块外的横梁相连"
        );
    }

    /// AC-C3:平台的 F(396 格)按 S = 6 切段,每段格数与包络半径都在上界之内、段内粘结连通。
    ///
    /// 会变红的实现:段内不连通;段过大或包络半径超界。
    #[test]
    fn acc_c3_platform_segments_bounded() {
        let (w, plat) = platform();
        let falling = eval(&w, &plat).falling;
        let segs = split(&falling, &w, 6);
        assert_eq!(segs.len(), 16, "20×20 按 6 格网格切成 16 段");
        for seg in &segs {
            let set: BTreeSet<BlockPos> = seg.iter().copied().collect();
            let mut seen = BTreeSet::from([seg[0]]);
            let mut q = VecDeque::from([seg[0]]);
            while let Some(c) = q.pop_front() {
                for d in D6 {
                    let n = add(c, d);
                    if set.contains(&n) && seen.insert(n) {
                        q.push_back(n);
                    }
                }
            }
            assert_eq!(seen.len(), seg.len(), "段内粘结连通");
        }
    }

    /// AC-C3:原点 = 包围盒中心取整到格中心、同距取小;包络半径 = 最远格中心距离 + 0.45·√3。
    ///
    /// 会变红的实现:同距取了大者;负坐标取整方向错;半径漏了 0.45·√3。
    #[test]
    fn acc_c3_origin_and_radius() {
        assert_eq!(compound_origin(&line_x(0..6, 5, 0)), p(2, 5, 0));
        assert_eq!(compound_origin(&line_x(-3..3, 5, 0)), p(-1, 5, 0));
        assert_eq!(compound_origin(&line_x(-3..0, 5, 0)), p(-2, 5, 0));
        let cube = |lo: i32| -> Vec<BlockPos> {
            (lo..lo + 2)
                .flat_map(|x| (lo..lo + 2).flat_map(move |y| (lo..lo + 2).map(move |z| p(x, y, z))))
                .collect()
        };
        assert_eq!(compound_origin(&cube(0)), p(0, 0, 0));
        assert_eq!(compound_origin(&cube(-2)), p(-2, -2, -2));
        assert_eq!(compound_origin(&[p(7, -3, 4)]), p(7, -3, 4));
        let half = 0.45 * 3f32.sqrt();
        let r = envelope_radius(&line_x(0..6, 5, 0), p(2, 5, 0));
        assert!(
            (r - (3.0 + half)).abs() < 1e-4,
            "6 格梁 ρ = 3 + 0.45√3,实际 {r}"
        );
        let r = envelope_radius(&cube(0), p(0, 0, 0));
        assert!(
            (r - (3f32.sqrt() + half)).abs() < 1e-4,
            "2×2×2 ρ = √3 + 0.45√3,实际 {r}"
        );
    }
}

/// 单元 02:承载判定接入分片(AC-C2 生产路径、AC-C10 的转换部分;REQ-010 §4.6、§9,修订 2)。
mod unit02 {
    use messaging::{
        Envelope, EnvelopePayload, InputPayload, InputSource, Intent, SimCommand, StampedInput,
    };
    use physics_sim::cohesion::{CohesionConfig, CohesionStats};
    use physics_sim::destruction::{DEBRIS_HALF_SIZE, DestructionConfig};
    use physics_sim::{BlockWriteKind, ShardSim, ShardSimConfig};
    use shard::SchedulerConfig;
    use sim_api::log::CommandLog;
    use sim_api::session::{Session, SessionConfig};
    use std::collections::BTreeSet;
    use types::glam::Vec3;
    use types::{
        BlockId, BlockPos, ChunkPos, DimensionId, EMERGENT_ORIGIN, EntityDesc, EntityId,
        EntityState, ShardId, ShardPos, Tick, WORLD_Y_MIN, WorldId, WorldSeed,
    };
    use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

    const STONE: BlockId = builtin::STONE;
    const DIRT: BlockId = builtin::DIRT;
    const AIR: BlockId = builtin::AIR;

    fn shard(x: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, 0))
    }

    fn owner(q: BlockPos) -> ShardId {
        shard(if q.x < 128 { 0 } else { 1 })
    }

    fn p(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    fn column(x: i32, ys: std::ops::RangeInclusive<i32>, z: i32) -> Vec<BlockPos> {
        ys.map(|y| p(x, y, z)).collect()
    }

    fn row(xs: std::ops::RangeInclusive<i32>, y: i32, z: i32) -> Vec<BlockPos> {
        xs.map(|x| p(x, y, z)).collect()
    }

    fn cfg() -> SessionConfig {
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        }
    }

    fn registry() -> Box<dyn BlockRegistry> {
        Box::new(SimpleRegistry::builtin())
    }

    fn sim_cfg(x: i32, destruction: bool, cohesion: bool, cap: usize) -> ShardSimConfig {
        let mut c = ShardSimConfig::new(shard(x));
        c.destruction = DestructionConfig {
            enabled: destruction,
            max_detach_per_tick: cap,
            ..DestructionConfig::default()
        };
        // 单元 04 迁移:本模块的用例按逐格转换编写;单元 04 起按段转换(§5.5),切块边长设为 1 即每格一段、保持逐格。
        c.cohesion = CohesionConfig {
            enabled: cohesion,
            compound_split_size: 1,
            ..CohesionConfig::default()
        };
        c
    }

    fn load(
        sim: &mut ShardSim,
        cx: std::ops::RangeInclusive<i32>,
        cz: std::ops::RangeInclusive<i32>,
    ) {
        for x in cx {
            for z in cz.clone() {
                let cp = ChunkPos::new(x, z);
                sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
            }
        }
    }

    /// 会话:`shards` 中每个分片都装载 chunk x ∈ `cx`、z ∈ 2..=5 的平地(地表 y = −61,其上为空气)。
    fn session(
        shards: &[i32],
        cx: std::ops::RangeInclusive<i32>,
        destruction: bool,
        cohesion: bool,
        cap: usize,
    ) -> Session {
        let mut s = Session::new(cfg());
        for &x in shards {
            let mut sim = ShardSim::new(sim_cfg(x, destruction, cohesion, cap), registry());
            load(&mut sim, cx.clone(), 2..=5);
            s.register(sim);
        }
        s
    }

    fn set(s: &mut Session, log: &mut CommandLog, cells: &[BlockPos], block: BlockId) {
        for &c in cells {
            for entry in s.set_block(owner(c), c, block).expect("set_block") {
                log.append(entry);
            }
        }
    }

    fn block(s: &Session, sh: ShardId, q: BlockPos) -> BlockId {
        let cp = q.chunk_of();
        s.voxel_chunk(sh, cp).expect("chunk 已装载").get(
            (q.x - cp.x * 16) as usize,
            (q.y - WORLD_Y_MIN) as usize,
            (q.z - cp.z * 16) as usize,
        )
    }

    fn emergent(s: &Session, sh: ShardId) -> Vec<(EntityId, EntityState)> {
        s.entity_states(sh)
            .expect("分片已注册")
            .into_iter()
            .filter(|(id, _)| id.origin_shard() == EMERGENT_ORIGIN)
            .collect()
    }

    fn center_of(q: BlockPos) -> Vec3 {
        Vec3::new(q.x as f32 + 0.5, q.y as f32 + 0.5, q.z as f32 + 0.5)
    }

    /// 跑到给定分片都没有涌现实体为止(上限 900 拍)。
    fn settle(s: &mut Session, shards: &[ShardId]) -> bool {
        for _ in 0..900 {
            s.tick();
            if shards.iter().all(|sh| emergent(s, *sh).is_empty()) {
                return true;
            }
        }
        false
    }

    /// 门框:柱 x = `xa`、`xb`(y −60..=−56),门楣 x = xa..=xb、y = −55,同在 z。返回 (左柱, 右柱, 门楣)。
    fn door(xa: i32, xb: i32, z: i32) -> (Vec<BlockPos>, Vec<BlockPos>, Vec<BlockPos>) {
        (
            column(xa, -60..=-56, z),
            column(xb, -60..=-56, z),
            row(xa..=xb, -55, z),
        )
    }

    fn build_door(
        s: &mut Session,
        log: &mut CommandLog,
        xa: i32,
        xb: i32,
        z: i32,
        lintel: BlockId,
    ) {
        let (pa, pb, top) = door(xa, xb, z);
        set(s, log, &[pa, pb].concat(), STONE);
        set(s, log, &top, lintel);
    }

    /// 石墙(x 40..=42、y −60..=−55、z−2..=z+2)伸出 8 格石悬臂 x 43..=50、y = −55。返回悬臂格。
    fn build_cantilever(s: &mut Session, log: &mut CommandLog, z: i32) -> Vec<BlockPos> {
        let wall: Vec<BlockPos> = (40..=42)
            .flat_map(|x| (z - 2..=z + 2).flat_map(move |zz| column(x, -60..=-55, zz)))
            .collect();
        set(s, log, &wall, STONE);
        let arm = row(43..=50, -55, z);
        set(s, log, &arm, STONE);
        arm
    }

    // ================================================================== 单分片

    /// AC-C2 门框(生产路径):并排两个门框,土门楣与石门楣;同一拍移走两个左柱 → N 拍土门楣左 6 格写 air
    /// (石门楣不动),N+1 生成 6 个单格碎块(方块 = dirt、BlockPos 升序);落定后石门楣完好,土块落回地面。
    /// 承载判定只计入 `TickSummary::cohesion`:1 次求值、访问 12 格、F = 6、无待转换;不计 REQ-009 的坍塌求值,
    /// 转换照常计入 `cells_collapsed`。
    ///
    /// 会变红的实现:开启后仍走 `find_collapse`(土门楣经右柱锚定,不塌);承载判定推迟到下一拍;
    /// 遥测没接线或合计漏了分片;承载判定也计进了 `collapse_evaluations`。
    #[test]
    fn acc_c2_door_lintel_end_to_end() {
        let sh = shard(0);
        let mut s = session(&[0], 2..=6, true, true, 64);
        let mut log = CommandLog::new();
        build_door(&mut s, &mut log, 60, 66, 64, DIRT);
        build_door(&mut s, &mut log, 60, 66, 70, STONE);
        s.tick();
        assert!(emergent(&s, sh).is_empty(), "搭好时两个门框都稳定");
        set(&mut s, &mut log, &column(60, -60..=-56, 64), AIR);
        set(&mut s, &mut log, &column(60, -60..=-56, 70), AIR);
        let n = s.tick();
        let fell = row(60..=65, -55, 64);
        for c in &fell {
            assert_eq!(block(&s, sh, *c), AIR, "N 拍土门楣 {c:?} 已写 air");
        }
        assert_eq!(block(&s, sh, p(66, -55, 64)), DIRT, "右柱顶上那格不动");
        for c in row(60..=66, -55, 70) {
            assert_eq!(block(&s, sh, c), STONE, "石门楣挂得住 {c:?}");
        }
        assert_eq!(n.cohesion.load_evaluations, 1);
        assert_eq!(n.cohesion.load_visited, 12);
        assert_eq!(n.cohesion.load_falling, 6);
        assert_eq!(n.cohesion.load_pending, 0);
        assert_eq!(n.cohesion.load_double_evaluations, 0);
        assert_eq!(
            n.destruction.collapse_evaluations, 0,
            "承载判定不计 REQ-009 的坍塌求值"
        );
        assert_eq!(
            n.destruction.cells_collapsed, 6,
            "转换照常计入 cells_collapsed"
        );
        assert!(emergent(&s, sh).is_empty(), "N 拍还没有实体");
        s.tick();
        let got = emergent(&s, sh);
        assert_eq!(got.len(), 6);
        for ((id, state), cell) in got.iter().zip(&fell) {
            assert!(
                (state.translation - center_of(*cell)).length() < 0.35,
                "{id:?} 在 {cell:?} 附近"
            );
            assert_eq!(
                s.entity_desc(sh, *id),
                Some(EntityDesc {
                    half_size: DEBRIS_HALF_SIZE,
                    block: DIRT,
                    shape: None
                })
            );
        }
        assert!(settle(&mut s, &[sh]), "900 拍内全部凝固");
        for c in row(60..=66, -55, 70) {
            assert_eq!(block(&s, sh, c), STONE);
        }
        let mut dirt_on_ground = 0;
        for x in 54..=72 {
            for y in -60..=-56 {
                for z in 58..=68 {
                    if block(&s, sh, p(x, y, z)) == DIRT {
                        dirt_on_ground += 1;
                    }
                }
            }
        }
        assert_eq!(dirt_on_ground, 6, "6 格土落回地面");
        s.shutdown();
    }

    /// AC-C2 加重与换材料(D8,§4.6 第 2 条):8 格石悬臂末端放 1 格石头 → N 拍悬臂连同放上去的格(9 格)写 air,
    /// N+1 生成 9 个碎块;把另一根悬臂根部的墙格从 stone 换成 dirt(实心 → 实心)→ 粘结降到 3,整根 8 格断下。
    ///
    /// 会变红的实现:放置不触发;只登记"实心变空";材料替换不触发;放上去的格不进 F。
    #[test]
    fn acc_c2_placement_and_replacement_trigger() {
        let sh = shard(0);
        let mut s = session(&[0], 2..=6, true, true, 64);
        let mut log = CommandLog::new();
        let arm_a = build_cantilever(&mut s, &mut log, 64);
        let arm_b = build_cantilever(&mut s, &mut log, 74);
        s.tick();
        assert!(emergent(&s, sh).is_empty(), "8 格石悬臂挂得住");
        let weight = p(50, -54, 64);
        set(&mut s, &mut log, &[weight], STONE);
        let n = s.tick();
        for c in arm_a.iter().chain([&weight]) {
            assert_eq!(block(&s, sh, *c), AIR, "{c:?}");
        }
        assert_eq!(n.cohesion.load_falling, 9);
        s.tick();
        assert_eq!(emergent(&s, sh).len(), 9, "N+1 生成 9 个碎块");

        set(&mut s, &mut log, &[p(42, -55, 74)], DIRT);
        s.tick();
        for c in &arm_b {
            assert_eq!(block(&s, sh, *c), AIR, "根部换成土后整根断下 {c:?}");
        }
        assert_eq!(
            block(&s, sh, p(42, -55, 74)),
            DIRT,
            "换上的土格是接地的墙,不动"
        );
        s.shutdown();
    }

    /// AC-C2 凝固触发(D8):一个方块实体落在满载的 8 格石悬臂末端并凝固 → 凝固写入触发承载判定,悬臂连同凝固的格断下。
    ///
    /// 会变红的实现:凝固写入不登记触发。
    #[test]
    fn acc_c2_solidify_triggers_load_check() {
        let sh = shard(0);
        let mut s = session(&[0], 2..=6, true, true, 64);
        let mut log = CommandLog::new();
        let arm = build_cantilever(&mut s, &mut log, 84);
        s.tick();
        s.spawn_owned(
            sh,
            EntityId::from_parts(0, 1),
            EntityState {
                translation: Vec3::new(50.5, -52.5, 84.5),
                ..EntityState::default()
            },
            EntityDesc {
                half_size: DEBRIS_HALF_SIZE,
                block: STONE,
                shape: None,
            },
            InputSource(7),
        )
        .expect("spawn");
        let mut broke = false;
        for _ in 0..600 {
            s.tick();
            if arm.iter().all(|c| block(&s, sh, *c) == AIR) {
                broke = true;
                break;
            }
        }
        assert!(broke, "实体凝固在悬臂上后,600 拍内整根悬臂断下");
        s.shutdown();
    }

    /// AC-C2 待转换队列(§4.6 第 4 条):名额 3,土门楣 F = 6 → N 拍按 BlockPos 升序转换前 3 格、3 格留在队列
    /// (`load_pending` = 3);之后直接转换剩下 3 格,不重新求值(重新求值会判剩下 3 格挂得住)。在 N 与 N+1 之间
    /// 取世界快照并回放,两线继续,生成的 6 个实体逐位相同、ID 不重复。
    ///
    /// 会变红的实现:推迟时重新求值(剩下 3 格永远悬空);待转换集合不进快照(回放线少塌 3 格);超出名额的格丢失。
    #[test]
    fn acc_c2_pending_conversions_survive_without_reevaluation() {
        let sh = shard(0);
        let mut a = session(&[0], 2..=6, true, true, 3);
        let mut log = CommandLog::new();
        build_door(&mut a, &mut log, 60, 66, 64, DIRT);
        a.tick();
        set(&mut a, &mut log, &column(60, -60..=-56, 64), AIR);
        let n = a.tick();
        let fell = row(60..=65, -55, 64);
        let air: Vec<BlockPos> = fell
            .iter()
            .copied()
            .filter(|c| block(&a, sh, *c) == AIR)
            .collect();
        assert_eq!(air, fell[..3], "N 拍按 BlockPos 升序转换前 3 格");
        assert_eq!(n.cohesion.load_falling, 6);
        assert_eq!(n.cohesion.load_pending, 3);

        let world = a.world_snapshot();
        let mut b = sim_api::replay::replay_world(&world, &log, cfg(), registry).expect("回放");
        for _ in 0..3 {
            a.tick();
            b.tick();
        }
        for c in &fell {
            assert_eq!(block(&a, sh, *c), AIR, "剩下的格也转换了 {c:?}");
        }
        let ea = emergent(&a, sh);
        assert_eq!(ea.len(), 6);
        let ids: BTreeSet<EntityId> = ea.iter().map(|e| e.0).collect();
        assert_eq!(ids.len(), 6, "ID 不重复");
        assert_eq!(ea, emergent(&b, sh), "回放线逐位相同");
        a.shutdown();
        b.shutdown();
    }

    /// AC-C16 开关(§4.6 第 1 条):本 REQ 关闭时与 REQ-009 一致——土门楣经右柱锚定不塌、加重不触发,坍塌走
    /// `find_collapse` 并计 REQ-009 遥测;破坏关闭而本 REQ 开启时同样什么都不塌。两种情况 `cohesion` 遥测全为 0。
    ///
    /// 会变红的实现:承载判定不看 `destruction.enabled`;关闭时仍登记"加重"的触发;遥测在关闭时计数。
    #[test]
    fn acc_c16_switches() {
        for (destruction, cohesion) in [(true, false), (false, true)] {
            let sh = shard(0);
            let mut s = session(&[0], 2..=6, destruction, cohesion, 64);
            let mut log = CommandLog::new();
            build_door(&mut s, &mut log, 60, 66, 64, DIRT);
            let arm = build_cantilever(&mut s, &mut log, 80);
            s.tick();
            set(&mut s, &mut log, &column(60, -60..=-56, 64), AIR);
            set(&mut s, &mut log, &[p(50, -54, 80)], STONE);
            let n = s.tick();
            s.tick();
            for c in row(61..=66, -55, 64).iter().chain(&arm) {
                assert_ne!(
                    block(&s, sh, *c),
                    AIR,
                    "destruction={destruction} cohesion={cohesion}:{c:?} 不塌"
                );
            }
            assert_eq!(n.cohesion, CohesionStats::default());
            assert_eq!(
                n.destruction.collapse_evaluations,
                u64::from(destruction),
                "只有破坏开启时走 REQ-009 的坍塌求值"
            );
            assert!(emergent(&s, sh).is_empty());
            s.shutdown();
        }
    }

    // ================================================================== 跨分片

    fn shard_sim(x: i32) -> ShardSim {
        let mut sim = ShardSim::new(sim_cfg(x, true, true, 64), registry());
        load(&mut sim, 7..=9, 3..=4);
        sim
    }

    fn command(sim: &mut ShardSim, cells: &[BlockPos], block: BlockId) {
        let inputs: Vec<StampedInput> = cells
            .iter()
            .enumerate()
            .map(|(k, &pos)| StampedInput {
                apply_at: Tick(0),
                source: InputSource::SCHEDULER,
                sequence: k as u64,
                payload: InputPayload::Command(SimCommand::SetBlock { pos, block }),
            })
            .collect();
        sim.apply_prefix(&inputs);
    }

    /// 邻片 (0,0) 发来的写 air 信封;`marked` 中的格带承载转换标记。
    fn envelopes(sim: &mut ShardSim, plain: &[BlockPos], marked: &[BlockPos]) {
        let inputs: Vec<StampedInput> = plain
            .iter()
            .map(|c| (*c, false))
            .chain(marked.iter().map(|c| (*c, true)))
            .enumerate()
            .map(|(k, (pos, load_conversion))| StampedInput {
                apply_at: Tick(0),
                source: InputSource::SCHEDULER,
                sequence: k as u64,
                payload: InputPayload::Envelope(Envelope::stamp(
                    shard(0),
                    Intent {
                        destination: shard(1),
                        payload: EnvelopePayload::BlockWrite {
                            pos,
                            block: AIR,
                            load_conversion,
                        },
                    },
                    Tick(0),
                    k as u64,
                )),
            })
            .collect();
        sim.apply_prefix(&inputs);
    }

    /// AC-C10 求两次取并集(§9,分片级):门框柱 x = 124(邻片 (0,0) 的本体区)与 131,土门楣 124..=131。
    /// 分片 (1,0) 同一拍收到邻片的写入:左柱写 air(不带标记,模拟邻片的涌现破坏)+ 门楣 124..=127 写 air(带承载
    /// 转换标记)。"转换前"那次看到完整门楣 → F = 124..=130 共 7 格 → 本片转换 128..=130,写入带标记,计 2 次求值、
    /// 1 次两次求值;同样的写入若不带标记,只按写入后的视图求值,剩下 3 格恰好挂住、不转换。
    ///
    /// 会变红的实现:不看标记(永远只求写入后的一次,门楣残余永远悬空);把邻片本体区的格也转换了;
    /// 本片的转换写入没带标记(下游分片又会漏判)。
    #[test]
    fn acc_c10_neighbor_conversion_evaluated_before_and_after() {
        for marked in [true, false] {
            let mut sim = shard_sim(1);
            let (pa, pb, top) = door(124, 131, 64);
            command(&mut sim, &[pa.clone(), pb].concat(), STONE);
            command(&mut sim, &top, DIRT);
            sim.step();
            assert!(sim.take_outbox().detach_requests.is_empty(), "搭好时稳定");
            let left = row(124..=127, -55, 64);
            if marked {
                envelopes(&mut sim, &pa, &left);
            } else {
                envelopes(&mut sim, &[pa.clone(), left].concat(), &[]);
            }
            sim.step();
            let out = sim.take_outbox();
            let detached: Vec<BlockPos> = out.detach_requests.iter().map(|r| r.cell).collect();
            if marked {
                assert_eq!(
                    detached,
                    row(128..=130, -55, 64),
                    "本片转换自己本体区内的 3 格"
                );
                let flagged: Vec<BlockPos> = out
                    .block_updates
                    .iter()
                    .filter(|w| w.load_conversion)
                    .map(|w| w.pos)
                    .collect();
                assert_eq!(flagged, row(128..=130, -55, 64), "本片的转换写入带标记");
                assert!(
                    out.block_updates
                        .iter()
                        .filter(|w| w.load_conversion)
                        .all(|w| w.kind == BlockWriteKind::Emergent && w.block == AIR)
                );
                assert_eq!(out.cohesion_stats.load_evaluations, 2);
                assert_eq!(out.cohesion_stats.load_double_evaluations, 1);
                assert_eq!(out.cohesion_stats.load_falling, 7);
            } else {
                assert!(detached.is_empty(), "不带标记:剩下 3 格被单键容量 3 托住");
                assert_eq!(out.cohesion_stats.load_evaluations, 1);
                assert_eq!(out.cohesion_stats.load_double_evaluations, 0);
            }
        }
    }

    /// AC-C10 "写入后"那次不能省(§9):桥面 112..=133 用 `register` 注册的方块(未指定材料,B = ∞),贴着分片 (1,0)
    /// 视图的边缘——x = 111 未装载,只有 x = 112 那格连汇点 ∞,但 ∞ 粘结能把整条桥面的重量都传过去,搭好时稳定。
    /// 只收到带标记的 112..=127 写 air:"转换前"那次把视图外当地基、判稳定;"写入后"那次剩下 128..=133 无支撑 → 转换 6 格。
    ///
    /// 会变红的实现:有标记时只求"转换前"那一次。
    #[test]
    fn acc_c10_after_write_evaluation_still_needed() {
        let mut reg = SimpleRegistry::builtin();
        let beam = reg.register("beam", true).expect("注册");
        let mut sim = ShardSim::new(sim_cfg(1, true, true, 64), Box::new(reg));
        load(&mut sim, 7..=9, 3..=4);
        let deck = row(112..=133, -55, 64);
        command(&mut sim, &deck, beam);
        sim.step();
        assert!(
            sim.take_outbox().detach_requests.is_empty(),
            "∞ 粘结的桥面贴着未装载 chunk:挂得住"
        );
        envelopes(&mut sim, &[], &row(112..=127, -55, 64));
        sim.step();
        let out = sim.take_outbox();
        let detached: Vec<BlockPos> = out.detach_requests.iter().map(|r| r.cell).collect();
        assert_eq!(detached, row(128..=133, -55, 64));
        assert_eq!(out.cohesion_stats.load_double_evaluations, 1);
    }

    /// AC-C10 端到端(调度器带标记):门框柱 x = 124(分片 (0,0))与 131(分片 (1,0)),土门楣 124..=131 恰好满载
    /// (中间 6 格、两端容量 3 + 3)。一个土块实体落在门楣 x = 126 上并凝固(分片 (0,0) 的涌现写入,邻片晚一拍才看到)
    /// → 分片 (0,0) 判 F = 门楣中间 6 格 + 凝固的那格,转换自己的 4 格;分片 (1,0) 晚一拍收到凝固写入与带标记的转换写入,
    /// 按"转换前"判出同一个 F,转换 128..=130。落定后两边视图里门楣中间 6 格都是 air,两端各剩 1 格土;
    /// 期间某一拍的 `TickSummary::cohesion.load_double_evaluations` ≥ 1。
    ///
    /// 会变红的实现:调度器没把标记带进信封;接收方不求"转换前"那次(128..=130 永远悬空);凝固不触发。
    #[test]
    fn acc_c10_cross_shard_lintel_end_to_end() {
        let (s0, s1) = (shard(0), shard(1));
        let mut s = session(&[0, 1], 6..=9, true, true, 64);
        let mut log = CommandLog::new();
        build_door(&mut s, &mut log, 124, 131, 64, DIRT);
        s.tick();
        s.tick();
        assert!(
            emergent(&s, s0).is_empty() && emergent(&s, s1).is_empty(),
            "满载的门楣挂得住"
        );
        s.spawn_owned(
            s0,
            EntityId::from_parts(0, 1),
            EntityState {
                translation: Vec3::new(126.5, -53.4, 64.5),
                ..EntityState::default()
            },
            EntityDesc {
                half_size: DEBRIS_HALF_SIZE,
                block: DIRT,
                shape: None,
            },
            InputSource(7),
        )
        .expect("spawn");
        let mut doubled = false;
        let mut broke = false;
        for _ in 0..600 {
            let summary = s.tick();
            doubled |= summary.cohesion.load_double_evaluations >= 1;
            if block(&s, s1, p(129, -55, 64)) == AIR {
                broke = true;
                break;
            }
        }
        assert!(broke, "分片 (1,0) 视图里门楣的右半段在 600 拍内塌下");
        assert!(settle(&mut s, &[s0, s1]), "900 拍内全部凝固");
        for sh in [s0, s1] {
            for c in row(125..=130, -55, 64) {
                assert_eq!(block(&s, sh, c), AIR, "{sh:?} 视图里 {c:?}");
            }
            assert_eq!(block(&s, sh, p(124, -55, 64)), DIRT, "{sh:?} 左柱顶上那格");
            assert_eq!(block(&s, sh, p(131, -55, 64)), DIRT, "{sh:?} 右柱顶上那格");
            assert_eq!(
                block(&s, sh, p(126, -54, 64)),
                AIR,
                "{sh:?} 凝固的那格随门楣落下"
            );
        }
        assert!(doubled, "TickSummary 合计到了邻片的两次求值");
        s.shutdown();
    }

    /// AC-C12:checkpoint 自本单元起为 v7(§4.6 第 8 条)。
    ///
    /// 会变红的实现:信封、出站写入与待转换集合的格式变了却没升版本。
    #[test]
    fn acc_c12_checkpoint_version_is_v7() {
        assert_eq!(app::driver::CHECKPOINT_FORMAT_VERSION, 7);
    }
}

/// 单元 03a:复合实体形状与刚体(AC-C4 形状部分、AC-C5 运动部分、AC-C10 迁移、AC-C12;REQ-010 §5.4,修订 3)。
// 桩阶段 `EntityDesc` 仍是 `Copy`;单元 03a 实现后不再 `Copy`,这里的 `.clone()` 才是必需的。
#[allow(clippy::clone_on_copy)]
mod unit03a {
    use messaging::InputSource;
    use physics_sim::destruction::{DEBRIS_HALF_SIZE, DestructionConfig};
    use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
    use shard::SchedulerConfig;
    use sim_api::log::CommandLog;
    use sim_api::session::{Session, SessionConfig};
    use types::glam::{Quat, Vec3};
    use types::{
        BlockId, BlockPos, ChunkPos, CompoundShape, DimensionId, EntityDesc, EntityId, EntityState,
        ShapeCell, ShapeError, ShardId, ShardPos, WorldId, WorldSeed,
    };
    use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

    const STONE: BlockId = builtin::STONE;
    const DIRT: BlockId = builtin::DIRT;

    fn shard(x: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, 0))
    }

    fn p(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    fn cell(offset: [i32; 3], block: BlockId) -> ShapeCell {
        ShapeCell { offset, block }
    }

    fn cfg() -> SessionConfig {
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        }
    }

    fn registry() -> Box<dyn BlockRegistry> {
        Box::new(SimpleRegistry::builtin())
    }

    /// 关闭破坏与凝固的会话(复合体的凝固由单元 05 接入;这里只看刚体运动)。
    fn session(shards: &[i32], cx: std::ops::RangeInclusive<i32>) -> Session {
        let mut s = Session::new(cfg());
        for &x in shards {
            let mut c = ShardSimConfig::new(shard(x));
            c.destruction = DestructionConfig {
                enabled: false,
                ..DestructionConfig::default()
            };
            c.solidify = SolidifyConfig {
                enabled: false,
                ..SolidifyConfig::default()
            };
            let mut sim = ShardSim::new(c, registry());
            for cxx in cx.clone() {
                for z in 2..=5 {
                    let cp = ChunkPos::new(cxx, z);
                    sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
                }
            }
            s.register(sim);
        }
        s
    }

    /// 6 格石梁:世界格 x = x0..x0+6、y、z,原点按 §5.1 = (x0 + 2, y, z)。返回 (原点格, 描述)。
    fn beam(x0: i32, y: i32, z: i32) -> (BlockPos, EntityDesc) {
        let cells: Vec<(BlockPos, BlockId)> = (x0..x0 + 6).map(|x| (p(x, y, z), STONE)).collect();
        let origin = p(x0 + 2, y, z);
        let shape = CompoundShape::from_world(origin, &cells).expect("6 格石梁");
        (origin, EntityDesc::compound(DEBRIS_HALF_SIZE, shape))
    }

    fn spawn(
        s: &mut Session,
        log: &mut CommandLog,
        sh: ShardId,
        id: EntityId,
        origin: BlockPos,
        desc: &EntityDesc,
        (linvel, angvel): (Vec3, Vec3),
    ) {
        let entry = s
            .spawn_owned(
                sh,
                id,
                EntityState {
                    translation: Vec3::new(
                        origin.x as f32 + 0.5,
                        origin.y as f32 + 0.5,
                        origin.z as f32 + 0.5,
                    ),
                    rotation: Quat::IDENTITY,
                    linvel,
                    angvel,
                },
                desc.clone(),
                InputSource(7),
            )
            .expect("spawn");
        log.append(entry);
    }

    /// AC-C4 形状(§5.4 第 1–2 条):按偏移升序规范化;偏移重复、少于 2 格报错;`from_world` 由原点算偏移;
    /// 包络半径 = 最远偏移长度 + half·√3;复合体描述带形状、包络半径取形状的;单格描述没有形状、包络半径不变。
    ///
    /// 会变红的实现:不排序(同一形状两种写法不相等,快照与回放对不上);重复偏移被静默合并;
    /// 包络半径漏了 half·√3;单格描述的包络半径变了(既有足迹与唤醒全部偏移)。
    #[test]
    fn acc_c4_shape_canonical_and_radius() {
        let shape = CompoundShape::new(vec![
            cell([3, 0, 0], STONE),
            cell([0, 0, 0], DIRT),
            cell([1, 0, 0], STONE),
        ])
        .expect("3 格");
        assert_eq!(
            shape.cells(),
            &[
                cell([0, 0, 0], DIRT),
                cell([1, 0, 0], STONE),
                cell([3, 0, 0], STONE)
            ]
        );
        let again = CompoundShape::new(vec![
            cell([1, 0, 0], STONE),
            cell([3, 0, 0], STONE),
            cell([0, 0, 0], DIRT),
        ])
        .expect("3 格");
        assert_eq!(shape, again, "与输入顺序无关");
        assert_eq!(
            CompoundShape::new(vec![cell([1, 0, 0], STONE), cell([1, 0, 0], DIRT)]),
            Err(ShapeError::DuplicateOffset([1, 0, 0]))
        );
        assert_eq!(
            CompoundShape::new(vec![cell([0, 0, 0], STONE)]),
            Err(ShapeError::TooFewCells)
        );
        assert_eq!(CompoundShape::new(Vec::new()), Err(ShapeError::TooFewCells));

        let (origin, desc) = beam(0, 5, 0);
        assert_eq!(origin, p(2, 5, 0));
        let offsets: Vec<[i32; 3]> = desc
            .shape()
            .expect("复合体描述带形状")
            .cells()
            .iter()
            .map(|c| c.offset)
            .collect();
        assert_eq!(
            offsets,
            vec![
                [-2, 0, 0],
                [-1, 0, 0],
                [0, 0, 0],
                [1, 0, 0],
                [2, 0, 0],
                [3, 0, 0]
            ]
        );
        let want = 3.0 + DEBRIS_HALF_SIZE * 3f32.sqrt();
        let shape = desc.shape().expect("形状");
        assert!((shape.envelope_radius(DEBRIS_HALF_SIZE) - want).abs() < 1e-5);
        assert!(
            (desc.envelope_radius() - want).abs() < 1e-5,
            "复合体描述的包络半径取形状的"
        );
        assert_eq!(desc.half_size, DEBRIS_HALF_SIZE);
        assert_eq!(desc.clone(), desc);

        let single = EntityDesc::default();
        assert!(single.shape().is_none(), "单格实体没有形状");
        assert!(
            (single.envelope_radius() - 0.5 * 3f32.sqrt()).abs() < 1e-6,
            "单格包络半径不变"
        );
    }

    /// AC-C5 逐格碰撞体(§5.4 第 3 条):地面挖一条 4 格宽、1 格深的沟(x 61..=64),6 格石梁(x 60..=65)从 2 m 高
    /// 平落——两端的格搭在沟沿上,整根梁停在地面高度(格中心 y ≈ −59.475:地表薄片跨在 y = −60 的面上,顶面 −59.925)、不转、不掉进沟里;描述(含形状)保持不变。
    ///
    /// 会变红的实现:复合体只建了原点那一个碰撞体(原点在沟正上方,整根梁掉进沟里,y ≈ −60.475);
    /// 偏移没随刚体旋转;迁移/重建时丢了形状。
    #[test]
    fn acc_c5_compound_bridges_trench() {
        let sh = shard(0);
        let mut s = session(&[0], 2..=6);
        let mut log = CommandLog::new();
        for x in 61..=64 {
            for z in 60..=68 {
                for entry in s.set_block(sh, p(x, -61, z), builtin::AIR).expect("挖沟") {
                    log.append(entry);
                }
            }
        }
        s.tick();
        let id = EntityId::from_parts(0, 1);
        let (origin, desc) = beam(60, -58, 64);
        spawn(
            &mut s,
            &mut log,
            sh,
            id,
            origin,
            &desc,
            (Vec3::ZERO, Vec3::ZERO),
        );
        for _ in 0..150 {
            s.tick();
        }
        let state = s.entity_state(sh, id).expect("梁还是实体(本单元不凝固)");
        assert!(
            (state.translation.y - (-59.475)).abs() < 0.06,
            "搭在沟沿上:原点 y ≈ −59.475,实际 {}",
            state.translation.y
        );
        assert!(
            (state.translation.x - 62.5).abs() < 0.1,
            "没有横移:{}",
            state.translation.x
        );
        assert!(
            state.rotation.angle_between(Quat::IDENTITY) < 2f32.to_radians(),
            "没有转动:{:?}",
            state.rotation
        );
        assert_eq!(s.entity_desc(sh, id), Some(desc), "描述(含形状)不变");
        s.shutdown();
    }

    /// AC-C10 迁移(§5.4 第 6 条):6 格石梁从 x = 118.5 以 8 m/s 横飞、边飞边落,原点越过 x = 128 后归分片 (1,0);
    /// 目的分片里的描述(含形状)与出生时相同,源分片不再持有它。
    ///
    /// 会变红的实现:迁移信封丢了形状(目的分片重建成单格);按单格包络判定迁移余量导致提前或滞后迁移出错。
    #[test]
    fn acc_c10_compound_migrates_with_shape() {
        let mut s = session(&[0, 1], 6..=9);
        let mut log = CommandLog::new();
        let id = EntityId::from_parts(0, 1);
        let (origin, desc) = beam(116, -40, 64);
        spawn(
            &mut s,
            &mut log,
            shard(0),
            id,
            origin,
            &desc,
            (Vec3::new(8.0, 0.0, 0.0), Vec3::ZERO),
        );
        for _ in 0..100 {
            s.tick();
        }
        let state = s.entity_state(shard(1), id).expect("已迁到分片 (1,0)");
        assert!(
            state.translation.x > 128.0,
            "原点越过边界:{}",
            state.translation.x
        );
        assert_eq!(
            s.entity_desc(shard(1), id),
            Some(desc),
            "目的分片的描述带形状"
        );
        assert!(s.entity_state(shard(0), id).is_none(), "源分片不再持有");
        s.shutdown();
    }

    /// AC-C12 快照与回放(§5.4 第 6 条):带角速度下落中的复合体,在第 10 拍取世界快照并回放,两线再跑 60 拍,
    /// 状态与描述逐位相同。
    ///
    /// 会变红的实现:实体快照不带形状(回放线重建成单格,轨迹分叉);形状的规范顺序在快照往返后变了。
    #[test]
    fn acc_c12_compound_snapshot_replay() {
        let sh = shard(0);
        let mut a = session(&[0], 2..=6);
        let mut log = CommandLog::new();
        let id = EntityId::from_parts(0, 1);
        let (origin, desc) = beam(60, -50, 64);
        spawn(
            &mut a,
            &mut log,
            sh,
            id,
            origin,
            &desc,
            (Vec3::new(0.5, 0.0, 0.3), Vec3::new(0.4, 1.1, -0.7)),
        );
        for _ in 0..10 {
            a.tick();
        }
        let world = a.world_snapshot();
        let mut b = sim_api::replay::replay_world(&world, &log, cfg(), registry).expect("回放");
        for _ in 0..60 {
            a.tick();
            b.tick();
        }
        assert_eq!(
            a.entity_state(sh, id),
            b.entity_state(sh, id),
            "回放线状态逐位相同"
        );
        assert_eq!(b.entity_desc(sh, id), Some(desc), "回放线的描述带形状");
        a.shutdown();
        b.shutdown();
    }
}

/// 单元 04:塌落按段生成复合体(AC-C3、AC-C4、AC-C11;REQ-010 §5.5,修订 5)。
// 桩阶段 `EntityDesc` 的比较依赖 `Clone`;与 unit03a 同理。
#[allow(clippy::clone_on_copy)]
mod unit04 {
    use physics_sim::activation::{ActivationConfig, ActivationController};
    use physics_sim::cohesion::CohesionConfig;
    use physics_sim::destruction::{DEBRIS_HALF_SIZE, DestructionConfig};
    use physics_sim::{ShardSim, ShardSimConfig};
    use shard::SchedulerConfig;
    use sim_api::log::CommandLog;
    use sim_api::session::{Session, SessionConfig};
    use types::glam::Vec3;
    use types::{
        BlockId, BlockPos, ChunkPos, CompoundShape, DimensionId, EMERGENT_ORIGIN, EntityDesc,
        EntityId, EntityState, ShardId, ShardPos, WORLD_Y_MIN, WorldId, WorldSeed,
    };
    use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

    const STONE: BlockId = builtin::STONE;
    const DIRT: BlockId = builtin::DIRT;
    const SAND: BlockId = builtin::SAND;
    const AIR: BlockId = builtin::AIR;

    fn shard() -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0))
    }

    fn p(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    fn column(x: i32, ys: std::ops::RangeInclusive<i32>, z: i32) -> Vec<BlockPos> {
        ys.map(|y| p(x, y, z)).collect()
    }

    fn row(xs: std::ops::RangeInclusive<i32>, y: i32, z: i32) -> Vec<BlockPos> {
        xs.map(|x| p(x, y, z)).collect()
    }

    fn eid(k: u32) -> EntityId {
        EntityId::from_parts(EMERGENT_ORIGIN, k)
    }

    fn cfg() -> SessionConfig {
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        }
    }

    fn registry() -> Box<dyn BlockRegistry> {
        Box::new(SimpleRegistry::builtin())
    }

    /// 单分片会话(chunk x 2..=6、z 2..=5);破坏与本 REQ 开启,`slots` = 每拍实体名额,`split` = 切块边长,
    /// `cells` = 每拍转换格数上限。
    fn session(slots: usize, split: i32, cells: usize) -> Session {
        let mut c = ShardSimConfig::new(shard());
        c.destruction = DestructionConfig {
            enabled: true,
            max_detach_per_tick: slots,
            ..DestructionConfig::default()
        };
        c.cohesion = CohesionConfig {
            enabled: true,
            compound_split_size: split,
            max_converted_cells_per_tick: cells,
            ..CohesionConfig::default()
        };
        let mut sim = ShardSim::new(c, registry());
        for x in 2..=6 {
            for z in 2..=5 {
                let cp = ChunkPos::new(x, z);
                sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
            }
        }
        let mut s = Session::new(cfg());
        s.register(sim);
        s
    }

    fn set(s: &mut Session, log: &mut CommandLog, cells: &[BlockPos], block: BlockId) {
        for &c in cells {
            for entry in s.set_block(shard(), c, block).expect("set_block") {
                log.append(entry);
            }
        }
    }

    fn block(s: &Session, q: BlockPos) -> BlockId {
        let cp = q.chunk_of();
        s.voxel_chunk(shard(), cp).expect("chunk 已装载").get(
            (q.x - cp.x * 16) as usize,
            (q.y - WORLD_Y_MIN) as usize,
            (q.z - cp.z * 16) as usize,
        )
    }

    fn emergent(s: &Session) -> Vec<(EntityId, EntityState)> {
        s.entity_states(shard())
            .expect("分片已注册")
            .into_iter()
            .filter(|(id, _)| id.origin_shard() == EMERGENT_ORIGIN)
            .collect()
    }

    fn center_of(q: BlockPos) -> Vec3 {
        Vec3::new(q.x as f32 + 0.5, q.y as f32 + 0.5, q.z as f32 + 0.5)
    }

    /// 期望的复合体描述:由世界格与 §5.1 的原点规则得出。
    fn compound(cells: &[BlockPos], block: BlockId, origin: BlockPos) -> EntityDesc {
        let world: Vec<(BlockPos, BlockId)> = cells.iter().map(|c| (*c, block)).collect();
        EntityDesc::compound(
            DEBRIS_HALF_SIZE,
            CompoundShape::from_world(origin, &world).expect("≥ 2 格"),
        )
    }

    /// 20 格石桥面 x 60..=79、y = −55、z = 72,石柱在 x = 60、70、79(y −60..=−56;两段净跨 9 与 8,都挂得住)。
    /// 返回 (柱格, 桥面)。
    fn bridge(s: &mut Session, log: &mut CommandLog) -> (Vec<BlockPos>, Vec<BlockPos>) {
        let pillars: Vec<BlockPos> = [60, 70, 79]
            .into_iter()
            .flat_map(|x| column(x, -60..=-56, 72))
            .collect();
        let deck = row(60..=79, -55, 72);
        set(s, log, &pillars, STONE);
        set(s, log, &deck, STONE);
        (pillars, deck)
    }

    /// AC-C4 门楣整块落下:土门楣左 6 格(x 60..=65,同一个 6 格切块)→ N 拍 6 格写 air、计 1 段 1 个复合体 6 格;
    /// N+1 只生成 1 个实体——复合体,原点 (62, −55, 64)、偏移 −2..=3、方块 dirt,出生在原点格中心附近。
    ///
    /// 会变红的实现:仍逐格生成 6 个碎块;原点不按包围盒中心取整;复合体的描述没带形状;遥测没接线。
    #[test]
    fn acc_c4_door_lintel_falls_as_one_compound() {
        let mut s = session(64, 6, 1024);
        let mut log = CommandLog::new();
        set(
            &mut s,
            &mut log,
            &[column(60, -60..=-56, 64), column(66, -60..=-56, 64)].concat(),
            STONE,
        );
        set(&mut s, &mut log, &row(60..=66, -55, 64), DIRT);
        s.tick();
        set(&mut s, &mut log, &column(60, -60..=-56, 64), AIR);
        let n = s.tick();
        let fell = row(60..=65, -55, 64);
        for c in &fell {
            assert_eq!(block(&s, *c), AIR, "N 拍 {c:?} 写 air");
        }
        assert_eq!(n.cohesion.segments, 1);
        assert_eq!(n.cohesion.compounds_spawned, 1);
        assert_eq!(n.cohesion.compound_cells, 6);
        assert_eq!(n.destruction.cells_collapsed, 6);
        s.tick();
        let got = emergent(&s);
        assert_eq!(got.len(), 1, "只生成一个实体");
        let (id, state) = got[0];
        assert_eq!(id, eid(1));
        let origin = p(62, -55, 64);
        assert!((state.translation - center_of(origin)).length() < 0.35);
        assert_eq!(
            s.entity_desc(shard(), id),
            Some(compound(&fell, DIRT, origin))
        );
    }

    /// AC-C3/C4 切块:20 格石桥面三根柱子同拍移走 → F = 20 格,按世界对齐的 6 格网格落成 4 段(6/6/6/2),
    /// N+1 生成 4 个复合体,ID 按各段最小格升序依次为 1..=4,原点分别是 x = 62、68、74、78。
    ///
    /// 会变红的实现:切块网格按 F 自身的包围盒对齐;2 格的段被当成单格碎块;ID 不按最小格排序。
    #[test]
    fn acc_c4_bridge_splits_into_grid_segments() {
        let mut s = session(64, 6, 1024);
        let mut log = CommandLog::new();
        let (pillars, deck) = bridge(&mut s, &mut log);
        s.tick();
        assert!(emergent(&s).is_empty(), "三根柱子撑着挂得住");
        set(&mut s, &mut log, &pillars, AIR);
        let n = s.tick();
        assert_eq!(n.cohesion.segments, 4);
        assert_eq!(n.cohesion.compounds_spawned, 4);
        assert_eq!(n.cohesion.compound_cells, 20);
        s.tick();
        let got = emergent(&s);
        let segs: [(std::ops::RangeInclusive<i32>, i32); 4] =
            [(60..=65, 62), (66..=71, 68), (72..=77, 74), (78..=79, 78)];
        assert_eq!(got.len(), 4);
        for (k, (xs, ox)) in segs.into_iter().enumerate() {
            let (id, state) = got[k];
            assert_eq!(id, eid(k as u32 + 1), "第 {k} 段的 ID");
            let origin = p(ox, -55, 72);
            assert!((state.translation - center_of(origin)).length() < 0.35);
            let cells: Vec<BlockPos> = deck.iter().copied().filter(|c| xs.contains(&c.x)).collect();
            assert_eq!(
                s.entity_desc(shard(), id),
                Some(compound(&cells, STONE, origin))
            );
        }
    }

    /// AC-C3/C4 材料分段与 ID 交错:石柱顶上 2 格石头(x 90..=91)、右格上压 1 格沙;移走石柱 → 2 格石头成一个复合体、
    /// 沙子单独成单格碎块(沙没有粘结);两者的 ID 按最小格升序:复合体 (90, −58) 在前,沙 (91, −57) 在后。
    ///
    /// 会变红的实现:沙子并进了复合体;单格与复合体分开编号(先发完一种再发另一种)。
    #[test]
    fn acc_c4_sand_stays_single_and_ids_interleave() {
        let mut s = session(64, 6, 1024);
        let mut log = CommandLog::new();
        let pillar = column(90, -60..=-59, 80);
        set(&mut s, &mut log, &pillar, STONE);
        set(&mut s, &mut log, &[p(90, -58, 80), p(91, -58, 80)], STONE);
        set(&mut s, &mut log, &[p(91, -57, 80)], SAND);
        s.tick();
        set(&mut s, &mut log, &pillar, AIR);
        let n = s.tick();
        assert_eq!(n.cohesion.segments, 2);
        assert_eq!(n.cohesion.compounds_spawned, 1);
        s.tick();
        let got = emergent(&s);
        assert_eq!(got.len(), 2);
        let pair = [p(90, -58, 80), p(91, -58, 80)];
        assert_eq!(
            s.entity_desc(shard(), eid(1)),
            Some(compound(&pair, STONE, p(90, -58, 80)))
        );
        let sand = s.entity_desc(shard(), eid(2)).expect("沙子碎块");
        assert!(sand.shape().is_none(), "沙子是单格碎块");
        assert_eq!(sand.block, SAND);
    }

    /// AC-C4 名额与原子性(§5.5 第 2 条):
    /// - 每拍 2 个实体名额:桥面 4 段 → N 拍转换前 2 段(x 60..=71),后 2 段整段留下(x 72..=79 仍是石头,待转换 8 格);
    ///   N+1 转换其余;最终 4 个复合体;
    /// - 切块边长 2、每拍 8 格:10 段 × 2 格 → 每拍 4 段,N、N+1、N+2 依次转换 x 60..=67、68..=75、76..=79。
    ///
    /// 会变红的实现:段被拆开转换;名额不足时跳过大段先转后面的小段;格数上限没生效。
    #[test]
    fn acc_c4_segment_quota_is_atomic() {
        let air_xs = |s: &Session| -> Vec<i32> {
            (60..=79)
                .filter(|x| block(s, p(*x, -55, 72)) == AIR)
                .collect()
        };
        let mut s = session(2, 6, 1024);
        let mut log = CommandLog::new();
        let (pillars, _) = bridge(&mut s, &mut log);
        s.tick();
        set(&mut s, &mut log, &pillars, AIR);
        let n = s.tick();
        assert_eq!(
            air_xs(&s),
            (60..=71).collect::<Vec<_>>(),
            "N 拍整段转换前 2 段"
        );
        assert_eq!(n.cohesion.load_pending, 8);
        s.tick();
        assert_eq!(air_xs(&s), (60..=79).collect::<Vec<_>>(), "N+1 拍转换其余");
        s.tick();
        assert_eq!(emergent(&s).len(), 4);

        let mut s = session(64, 2, 8);
        let mut log = CommandLog::new();
        let (pillars, _) = bridge(&mut s, &mut log);
        s.tick();
        set(&mut s, &mut log, &pillars, AIR);
        for want in [60..=67, 60..=75, 60..=79] {
            s.tick();
            assert_eq!(air_xs(&s), want.collect::<Vec<_>>(), "每拍至多 8 格");
        }
    }

    /// AC-C12 生成请求进快照:门楣塌落的 N 拍与 N+1 之间取世界快照并回放,两线各跑 2 拍,生成的复合体
    /// (ID、状态、描述)逐位相同。
    ///
    /// 会变红的实现:复合体生成请求不进调度器快照(回放线没有这个实体);回放时形状丢失。
    #[test]
    fn acc_c12_compound_request_survives_snapshot() {
        let mut a = session(64, 6, 1024);
        let mut log = CommandLog::new();
        set(
            &mut a,
            &mut log,
            &[column(60, -60..=-56, 64), column(66, -60..=-56, 64)].concat(),
            STONE,
        );
        set(&mut a, &mut log, &row(60..=66, -55, 64), DIRT);
        a.tick();
        set(&mut a, &mut log, &column(60, -60..=-56, 64), AIR);
        a.tick();
        let world = a.world_snapshot();
        let mut b = sim_api::replay::replay_world(&world, &log, cfg(), registry).expect("回放");
        for _ in 0..2 {
            a.tick();
            b.tick();
        }
        let (ea, eb) = (emergent(&a), emergent(&b));
        assert_eq!(ea.len(), 1);
        assert_eq!(ea, eb, "回放线逐位相同");
        assert_eq!(
            a.entity_desc(shard(), ea[0].0),
            b.entity_desc(shard(), eb[0].0)
        );
        assert!(
            b.entity_desc(shard(), eb[0].0)
                .and_then(|d| d.shape().cloned())
                .is_some()
        );
        a.shutdown();
        b.shutdown();
    }

    /// AC-C11 醒体预算按格数计(§5.5 第 5 条):上限 10 格。近处 6 格复合体 + 近处单格 = 7 格,不淘汰;
    /// 再加远处 6 格复合体 = 13 格 → 只淘汰最远的那个;三个 6 格复合体(18 格)→ 由远到近淘汰两个。
    ///
    /// 会变红的实现:仍按实体个数计(3 个 < 10,不淘汰);淘汰顺序不是由远到近;多淘汰了。
    #[test]
    fn acc_c11_awake_budget_counts_cells() {
        let ctl = ActivationController::new(ActivationConfig {
            max_awake_dynamic: 10,
            ..ActivationConfig::default()
        });
        let obs = [Vec3::ZERO];
        let (a, b, c) = (eid(1), eid(2), eid(3));
        let near6 = (a, Vec3::new(5.0, 0.0, 0.0), 6);
        let far6 = (b, Vec3::new(50.0, 0.0, 0.0), 6);
        let near1 = (c, Vec3::new(3.0, 0.0, 0.0), 1);
        assert!(
            ctl.enforce_budget_cells(&obs, &[near6, near1]).is_empty(),
            "7 格不超"
        );
        assert_eq!(
            ctl.enforce_budget_cells(&obs, &[near6, far6, near1]),
            vec![b]
        );
        let mid6 = (c, Vec3::new(20.0, 0.0, 0.0), 6);
        assert_eq!(
            ctl.enforce_budget_cells(&obs, &[near6, far6, mid6]),
            vec![b, c],
            "由远到近淘汰到 ≤ 10 格"
        );
    }
}

/// 单元 05:整块凝固、倾斜散开与强制落地(AC-C7、AC-C8;REQ-010 §8.3,修订 6)。
#[allow(clippy::clone_on_copy)]
mod unit05 {
    use messaging::InputSource;
    use physics_sim::cohesion::{
        CohesionConfig, CompoundLanding, CompoundSnap, compound_forced_landing, compound_snap,
    };
    use physics_sim::destruction::{DEBRIS_HALF_SIZE, DestructionConfig};
    use physics_sim::{ShardSim, ShardSimConfig};
    use shard::SchedulerConfig;
    use sim_api::log::CommandLog;
    use sim_api::session::{Session, SessionConfig};
    use types::glam::{Quat, Vec3};
    use types::{
        BlockId, BlockPos, ChunkPos, CompoundShape, DimensionId, EntityDesc, EntityId, EntityState,
        ShapeCell, ShardId, ShardPos, WORLD_Y_MIN, WorldId, WorldSeed,
    };
    use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

    const STONE: BlockId = builtin::STONE;
    const AIR: BlockId = builtin::AIR;

    fn p(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    fn row(xs: std::ops::RangeInclusive<i32>, y: i32, z: i32) -> Vec<BlockPos> {
        xs.map(|x| p(x, y, z)).collect()
    }

    fn stones(cells: &[BlockPos]) -> Vec<(BlockPos, BlockId)> {
        let mut v: Vec<(BlockPos, BlockId)> = cells.iter().map(|c| (*c, STONE)).collect();
        v.sort();
        v
    }

    /// 6 格石梁的形状:偏移 x = −2..=3。
    fn beam() -> CompoundShape {
        CompoundShape::new(
            (-2..=3)
                .map(|x| ShapeCell {
                    offset: [x, 0, 0],
                    block: STONE,
                })
                .collect(),
        )
        .expect("6 格梁")
    }

    fn rot_y(deg: f32) -> Quat {
        Quat::from_rotation_y(deg.to_radians())
    }

    // ------------------------------------------------------------------ 纯函数

    /// AC-C7 吸附判定(§8.3 第 1 条):平放 → 写在原点格那一行;绕 y 转 90° → 偏移随朝向旋转;5° 小偏差 → 吸附到
    /// 轴对齐朝向;倾斜 20° → 末端格偏离超过 0.5 格,判倾斜;目标格被占 → 上移 1 格;上移到上限仍被占 → 倾斜。
    ///
    /// 会变红的实现:原点格按四舍五入而不是格中心取整;偏移没随目标朝向旋转;误差只看原点不看各格;
    /// 上移判定用了上移后的误差;被占时直接判倾斜而不上移。
    #[test]
    fn acc_c7_compound_snap_rules() {
        let shape = beam();
        let free = |_: BlockPos| false;
        let rest = Vec3::new(62.5, -59.475, 64.5);
        let snap = |t: Vec3, r: Quat, lift: i32, occ: &dyn Fn(BlockPos) -> bool| {
            compound_snap(t, r, &shape, 0.5, 45.0, lift, occ)
        };
        let flat = CompoundSnap::Write {
            cells: stones(&row(60..=65, -60, 64)),
            lifted: 0,
        };
        assert_eq!(snap(rest, Quat::IDENTITY, 2, &free), flat, "平放");
        assert_eq!(
            snap(rest, rot_y(5.0), 2, &free),
            flat,
            "5° 小偏差吸附到轴对齐"
        );
        let turned: Vec<BlockPos> = (61..=66).map(|z| p(62, -60, z)).collect();
        assert_eq!(
            snap(rest, rot_y(90.0), 2, &free),
            CompoundSnap::Write {
                cells: stones(&turned),
                lifted: 0
            },
            "绕 y 转 90°:+x 偏移转到 −z"
        );
        assert_eq!(
            snap(rest, Quat::from_rotation_z(20f32.to_radians()), 2, &free),
            CompoundSnap::Tilted,
            "倾斜 20°:末端格偏离 > 0.5 格"
        );
        let one = |q: BlockPos| q == p(63, -60, 64);
        assert_eq!(
            snap(rest, Quat::IDENTITY, 2, &one),
            CompoundSnap::Write {
                cells: stones(&row(60..=65, -59, 64)),
                lifted: 1
            },
            "目标格被占:上移 1 格"
        );
        let two = |q: BlockPos| q == p(63, -60, 64) || q == p(63, -59, 64);
        assert_eq!(
            snap(rest, Quat::IDENTITY, 1, &two),
            CompoundSnap::Tilted,
            "上移到上限仍被占 → 倾斜"
        );
    }

    /// AC-C8 强制落地(§8.3 第 5 条):空中的梁整块落到地面;一根柱子上多一格石头 → 整块停在它上面(所有格同高);
    /// 起点与实心重叠 → 先上移再落;涉及的柱有未装载 → 推迟;下方全空 → 退化为逐格;上移到上限仍重叠 → 退化为逐格。
    ///
    /// 会变红的实现:逐格各自落到自己的柱底(不再是一整块);只看原点所在柱;未装载当成空气一路落穿。
    #[test]
    fn acc_c8_compound_forced_landing_rules() {
        let shape = beam();
        let ground = |q: BlockPos| -> Option<bool> { Some(q.y <= -61) };
        let land = |t: Vec3, lift: i32, solid: &dyn Fn(BlockPos) -> Option<bool>| {
            compound_forced_landing(t, Quat::IDENTITY, &shape, lift, solid)
        };
        let high = Vec3::new(62.5, -50.5, 64.5);
        assert_eq!(
            land(high, 2, &ground),
            CompoundLanding::Write {
                cells: stones(&row(60..=65, -60, 64))
            },
            "落到地面"
        );
        let bump = |q: BlockPos| -> Option<bool> { Some(q.y <= -61 || q == p(65, -60, 64)) };
        assert_eq!(
            land(high, 2, &bump),
            CompoundLanding::Write {
                cells: stones(&row(60..=65, -59, 64))
            },
            "整块停在最高的那一柱上"
        );
        let low = Vec3::new(62.5, -59.5, 64.5);
        let inside = |q: BlockPos| -> Option<bool> { Some(q.y <= -61 || q == p(63, -60, 64)) };
        assert_eq!(
            land(low, 2, &inside),
            CompoundLanding::Write {
                cells: stones(&row(60..=65, -59, 64))
            },
            "起点重叠:上移 1 格"
        );
        let unloaded = |q: BlockPos| -> Option<bool> {
            if q.x == 60 && q.y < -55 {
                None
            } else {
                Some(q.y <= -61)
            }
        };
        assert_eq!(
            land(high, 2, &unloaded),
            CompoundLanding::Defer,
            "未装载 → 推迟"
        );
        let void = |_: BlockPos| -> Option<bool> { Some(false) };
        assert_eq!(
            land(high, 2, &void),
            CompoundLanding::Fallback,
            "下方全空 → 逐格"
        );
        let wall = |q: BlockPos| -> Option<bool> { Some(q.y <= -61 || (q.x == 63 && q.y <= -58)) };
        assert_eq!(
            land(low, 1, &wall),
            CompoundLanding::Fallback,
            "上移到上限仍重叠 → 逐格"
        );
    }

    // ------------------------------------------------------------------ 端到端

    fn shard() -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0))
    }

    fn cfg() -> SessionConfig {
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        }
    }

    fn registry() -> Box<dyn BlockRegistry> {
        Box::new(SimpleRegistry::builtin())
    }

    /// 单分片会话(chunk x 2..=6、z 2..=5;凝固开启)。`load` = 破坏与本 REQ 是否开启。
    fn session(load: bool) -> Session {
        let mut c = ShardSimConfig::new(shard());
        c.destruction = DestructionConfig {
            enabled: load,
            ..DestructionConfig::default()
        };
        c.cohesion = CohesionConfig {
            enabled: load,
            ..CohesionConfig::default()
        };
        let mut sim = ShardSim::new(c, registry());
        for x in 2..=6 {
            for z in 2..=5 {
                let cp = ChunkPos::new(x, z);
                sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
            }
        }
        let mut s = Session::new(cfg());
        s.register(sim);
        s
    }

    fn set(s: &mut Session, log: &mut CommandLog, cells: &[BlockPos], block: BlockId) {
        for &c in cells {
            for entry in s.set_block(shard(), c, block).expect("set_block") {
                log.append(entry);
            }
        }
    }

    fn block(s: &Session, q: BlockPos) -> BlockId {
        let cp = q.chunk_of();
        s.voxel_chunk(shard(), cp).expect("chunk 已装载").get(
            (q.x - cp.x * 16) as usize,
            (q.y - WORLD_Y_MIN) as usize,
            (q.z - cp.z * 16) as usize,
        )
    }

    fn spawn_beam(
        s: &mut Session,
        log: &mut CommandLog,
        k: u32,
        translation: Vec3,
        rotation: Quat,
    ) {
        let entry = s
            .spawn_owned(
                shard(),
                EntityId::from_parts(0, k),
                EntityState {
                    translation,
                    rotation,
                    ..EntityState::default()
                },
                EntityDesc::compound(DEBRIS_HALF_SIZE, beam()),
                InputSource(7),
            )
            .expect("spawn");
        log.append(entry);
    }

    /// 跑到分片里连续 2 拍没有任何实体为止(上限 900 拍),返回期间 `TickSummary::cohesion` 的累计值。
    /// 连续 2 拍:倾斜散开时复合体在 N 拍删除、碎块 N+1 才出现,中间有 1 拍实体表为空(§8.3 第 3 条)。
    fn settle(s: &mut Session) -> (bool, physics_sim::cohesion::CohesionStats) {
        let mut total = physics_sim::cohesion::CohesionStats::default();
        let mut empty = 0;
        for _ in 0..900 {
            let n = s.tick();
            total.add(&n.cohesion);
            empty = if s.entity_states(shard()).expect("分片").is_empty() {
                empty + 1
            } else {
                0
            };
            if empty >= 2 {
                return (true, total);
            }
        }
        (false, total)
    }

    fn count_stone(
        s: &Session,
        xs: std::ops::RangeInclusive<i32>,
        zs: std::ops::RangeInclusive<i32>,
    ) -> usize {
        let mut n = 0;
        for x in xs {
            for y in -60..=-50 {
                for z in zs.clone() {
                    if block(s, p(x, y, z)) == STONE {
                        n += 1;
                    }
                }
            }
        }
        n
    }

    /// AC-C7 整块凝固(§8.3 第 2 条):一根平放、一根绕 y 转 90° 的 6 格石梁从 2 m 高落下,停稳后同一拍整块写回地形,
    /// 实体消失——前者占 x 60..=65(z 64),后者占 z 61..=66(x 80);期间计 2 次整块凝固。
    ///
    /// 会变红的实现:复合体仍跳过凝固;只写原点一格;旋转后的偏移没随朝向转;遥测没接线。
    #[test]
    fn acc_c7_compounds_solidify_whole() {
        let mut s = session(false);
        let mut log = CommandLog::new();
        spawn_beam(
            &mut s,
            &mut log,
            1,
            Vec3::new(62.5, -57.5, 64.5),
            Quat::IDENTITY,
        );
        spawn_beam(
            &mut s,
            &mut log,
            2,
            Vec3::new(80.5, -57.5, 64.5),
            rot_y(90.0),
        );
        let (done, total) = settle(&mut s);
        assert!(done, "900 拍内两根梁都凝固");
        for c in row(60..=65, -60, 64) {
            assert_eq!(block(&s, c), STONE, "{c:?}");
        }
        for z in 61..=66 {
            assert_eq!(block(&s, p(80, -60, z)), STONE, "转过的梁 z = {z}");
        }
        assert_eq!(total.compounds_solidified, 2);
        assert_eq!(total.tilt_scattered, 0);
    }

    /// AC-C7 倾斜散开(§8.3 第 3 条,D5):6 格石梁的一端落在 2 格高的石堆上,停稳时倾斜约 23°(末端格偏离约 1.2 格,
    /// 已用 rapier demo 核过)→ 整块散成 6 个单格碎块,各自落定凝固;最终区域里正好多出 6 格石头,计 1 次散开。
    ///
    /// 会变红的实现:倾斜时强行整块吸附(石头穿进石堆或悬空);散开时丢格;散开后的碎块仍带形状。
    #[test]
    fn acc_c7_tilted_compound_scatters() {
        let mut s = session(false);
        let mut log = CommandLog::new();
        set(&mut s, &mut log, &[p(65, -60, 72), p(65, -59, 72)], STONE);
        s.tick();
        spawn_beam(
            &mut s,
            &mut log,
            1,
            Vec3::new(62.5, -57.275, 72.5),
            Quat::IDENTITY,
        );
        let (done, total) = settle(&mut s);
        assert!(done, "散开的碎块 900 拍内都凝固");
        assert_eq!(total.tilt_scattered, 1);
        assert_eq!(total.compounds_solidified, 0);
        assert_eq!(
            count_stone(&s, 54..=72, 66..=78),
            2 + 6,
            "石堆 2 格 + 碎块 6 格"
        );
    }

    /// AC-C7 凝固触发承载判定(§8.3 第 2 条、D8):6 格石梁轻轻落在满载的 8 格石悬臂上(下落 0.075 m,撞击远低于 I)
    /// 并整块凝固 → 写入登记触发 → 悬臂超载,从根部整根断下。
    ///
    /// 会变红的实现:复合体的凝固写入不登记触发。
    #[test]
    fn acc_c7_compound_solidify_triggers_load_check() {
        let mut s = session(true);
        let mut log = CommandLog::new();
        let wall: Vec<BlockPos> = (40..=42)
            .flat_map(|x| (82..=86).flat_map(move |z| (-60..=-55).map(move |y| p(x, y, z))))
            .collect();
        set(&mut s, &mut log, &wall, STONE);
        let arm = row(43..=50, -55, 84);
        set(&mut s, &mut log, &arm, STONE);
        s.tick();
        assert_eq!(block(&s, p(50, -55, 84)), STONE, "8 格石悬臂挂得住");
        spawn_beam(
            &mut s,
            &mut log,
            1,
            Vec3::new(47.5, -53.4, 84.5),
            Quat::IDENTITY,
        );
        let mut broke = false;
        for _ in 0..600 {
            s.tick();
            if arm.iter().all(|c| block(&s, *c) == AIR) {
                broke = true;
                break;
            }
        }
        assert!(broke, "梁凝固在悬臂上后,悬臂整根断下");
    }
}

/// 单元 06:碎裂与复合体砸地形(AC-C5 撞地形、AC-C6;REQ-010 §7.3,修订 7)。
#[allow(clippy::clone_on_copy)]
mod unit06 {
    use messaging::InputSource;
    use physics_sim::cohesion::{
        CohesionConfig, CohesionStats, fracture_bonds, fracture_components, fracture_pieces,
    };
    use physics_sim::destruction::{DEBRIS_HALF_SIZE, DestructionConfig};
    use physics_sim::{ShardSim, ShardSimConfig};
    use shard::SchedulerConfig;
    use sim_api::log::CommandLog;
    use sim_api::session::{Session, SessionConfig};
    use std::collections::BTreeSet;
    use types::glam::{Quat, Vec3};
    use types::{
        BlockId, BlockPos, ChunkPos, CompoundShape, DimensionId, EntityDesc, EntityId, EntityState,
        ShapeCell, ShardId, ShardPos, WORLD_Y_MIN, WorldId, WorldSeed,
    };
    use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

    const STONE: BlockId = builtin::STONE;
    const DIRT: BlockId = builtin::DIRT;
    const AIR: BlockId = builtin::AIR;
    const UP: Vec3 = Vec3::Y;

    fn p(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    /// 6 格梁:偏移 x = −2..=3,方块 `block`(序号 0..=5 依次对应偏移 −2..=3)。
    fn beam(block: BlockId) -> CompoundShape {
        CompoundShape::new(
            (-2..=3)
                .map(|x| ShapeCell {
                    offset: [x, 0, 0],
                    block,
                })
                .collect(),
        )
        .expect("6 格梁")
    }

    fn bonds(pairs: &[(usize, usize)]) -> BTreeSet<(usize, usize)> {
        pairs.iter().copied().collect()
    }

    fn near(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-4
    }

    // ------------------------------------------------------------------ 纯函数

    /// AC-C6 断键(§7.3 第 3 条):平落时冲量在接触格间均摊——石梁合计 46.8(每格 7.8 < 18)不断,合计 108(每格 18)
    /// 5 根横向粘结全断;土梁合计 46.8(每格 7.8 ≥ 7)全断;竖立的梁只有底格着地,向上那根粘结是压向,冲量再大也不断;
    /// 倾斜 30° 角先着地,粘结方向与受力方向夹角 60°(点积 0.5 < cos 45°),冲量 20 ≥ 18 → 断开底格那一根。
    ///
    /// 会变红的实现:不均摊(每格都按合计算,石梁平落 46.8 就全断);不豁免压向(竖落必断底格);压向判定用了本体坐标。
    #[test]
    fn acc_c6_fracture_bond_rules() {
        let reg = SimpleRegistry::builtin();
        let all: Vec<(usize, Vec3)> = (0..6).map(|i| (i, UP)).collect();
        let chain = bonds(&[(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)]);
        let stone = beam(STONE);
        assert!(
            fracture_bonds(&stone, Quat::IDENTITY, 46.8, &all, &reg).is_empty(),
            "石梁平落:每格 7.8 < 18"
        );
        assert_eq!(
            fracture_bonds(&stone, Quat::IDENTITY, 108.0, &all, &reg),
            chain,
            "每格 18 ≥ 18:横向粘结全断"
        );
        assert_eq!(
            fracture_bonds(&beam(DIRT), Quat::IDENTITY, 46.8, &all, &reg),
            chain,
            "土梁平落:每格 7.8 ≥ 7"
        );
        let upright = Quat::from_rotation_z(90f32.to_radians());
        assert!(
            fracture_bonds(&stone, upright, 1000.0, &[(0, UP)], &reg).is_empty(),
            "竖落:底格向上那根粘结是压向"
        );
        let tilted = Quat::from_rotation_z(30f32.to_radians());
        assert_eq!(
            fracture_bonds(&stone, tilted, 20.0, &[(0, UP)], &reg),
            bonds(&[(0, 1)]),
            "角先着地:断开底格那一根"
        );
        let mixed = CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: STONE,
            },
            ShapeCell {
                offset: [1, 0, 0],
                block: DIRT,
            },
        ])
        .expect("2 格");
        assert_eq!(
            fracture_bonds(&mixed, Quat::IDENTITY, 8.0, &[(0, UP)], &reg),
            bonds(&[(0, 1)]),
            "石–土粘结取较小的 I = 7"
        );
    }

    /// AC-C6 拆分(§7.3 第 4 条):梁断一根 → 2 个分量;断两根 → 3 个;2×2×2 立方底层四根横向粘结全断,竖向与顶层仍连着 → 1 个。
    ///
    /// 会变红的实现:分量顺序不按首格;把断开粘结两端的格都单独拆出来。
    #[test]
    fn acc_c6_fracture_components_rules() {
        let stone = beam(STONE);
        assert_eq!(
            fracture_components(&stone, &bonds(&[(0, 1)])),
            vec![vec![0], vec![1, 2, 3, 4, 5]]
        );
        assert_eq!(
            fracture_components(&stone, &bonds(&[(1, 2), (3, 4)])),
            vec![vec![0, 1], vec![2, 3], vec![4, 5]]
        );
        let cube = CompoundShape::new(
            (0..2)
                .flat_map(|x| (0..2).flat_map(move |y| (0..2).map(move |z| [x, y, z])))
                .map(|offset| ShapeCell {
                    offset,
                    block: STONE,
                })
                .collect(),
        )
        .expect("立方");
        // 偏移升序:0 (0,0,0) 1 (0,0,1) 2 (0,1,0) 3 (0,1,1) 4 (1,0,0) 5 (1,0,1) 6 (1,1,0) 7 (1,1,1);底层 = 0、1、4、5。
        assert_eq!(
            fracture_components(&cube, &bonds(&[(0, 1), (0, 4), (1, 5), (4, 5)])),
            vec![(0..8).collect::<Vec<usize>>()],
            "底层横向全断,仍经竖向与顶层连成一块"
        );
    }

    /// AC-C6 分块(§7.3 第 5 条):梁以 v = (0, −5, 0)、ω = (0, 0, 2) 运动时拆成 [0,1,2] 与 [3,4,5]——两段的新原点在偏移
    /// −1 与 2,形状都是 −1..=1;速度 = v + ω × (分量质心 − 原质心) = (0, −8, 0) 与 (0, −2, 0)。拆成 [0] 与 [1..=5] 时,
    /// 单格那段是普通单格碎块(平移 = 该格中心,速度 (0, −10, 0)),另一段原点偏移 1、形状 −2..=2、速度 (0, −4, 0)。
    ///
    /// 会变红的实现:分量直接沿用原平移(各段叠在一起);速度只给 v 不加 ω × r;单格分量仍带形状。
    #[test]
    fn acc_c6_fracture_pieces_rules() {
        let state = EntityState {
            translation: Vec3::new(10.5, 5.5, 3.5),
            rotation: Quat::IDENTITY,
            linvel: Vec3::new(0.0, -5.0, 0.0),
            angvel: Vec3::new(0.0, 0.0, 2.0),
        };
        let desc = EntityDesc::compound(DEBRIS_HALF_SIZE, beam(STONE));
        let short = |lo: i32, hi: i32| {
            EntityDesc::compound(
                DEBRIS_HALF_SIZE,
                CompoundShape::new(
                    (lo..=hi)
                        .map(|x| ShapeCell {
                            offset: [x, 0, 0],
                            block: STONE,
                        })
                        .collect(),
                )
                .expect("段"),
            )
        };
        let halves = fracture_pieces(&state, &desc, &[vec![0, 1, 2], vec![3, 4, 5]]);
        assert_eq!(halves.len(), 2);
        let want = [
            (Vec3::new(9.5, 5.5, 3.5), Vec3::new(0.0, -8.0, 0.0)),
            (Vec3::new(12.5, 5.5, 3.5), Vec3::new(0.0, -2.0, 0.0)),
        ];
        for ((st, d), (t, v)) in halves.iter().zip(want) {
            assert!(near(st.translation, t), "平移 {:?} ≠ {t:?}", st.translation);
            assert!(near(st.linvel, v), "速度 {:?} ≠ {v:?}", st.linvel);
            assert!(near(st.angvel, state.angvel));
            assert_eq!(st.rotation, state.rotation);
            assert_eq!(d, &short(-1, 1));
        }
        let split = fracture_pieces(&state, &desc, &[vec![0], vec![1, 2, 3, 4, 5]]);
        let (single, single_desc) = &split[0];
        assert!(near(single.translation, Vec3::new(8.5, 5.5, 3.5)));
        assert!(near(single.linvel, Vec3::new(0.0, -10.0, 0.0)));
        assert!(single_desc.shape().is_none(), "单格分量是普通单格碎块");
        assert_eq!(
            (single_desc.half_size, single_desc.block),
            (DEBRIS_HALF_SIZE, STONE)
        );
        let (rest, rest_desc) = &split[1];
        assert!(near(rest.translation, Vec3::new(11.5, 5.5, 3.5)));
        assert!(near(rest.linvel, Vec3::new(0.0, -4.0, 0.0)));
        assert_eq!(rest_desc, &short(-2, 2));
    }

    // ------------------------------------------------------------------ 端到端

    fn shard() -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0))
    }

    fn cfg() -> SessionConfig {
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        }
    }

    fn registry() -> Box<dyn BlockRegistry> {
        Box::new(SimpleRegistry::builtin())
    }

    /// 单分片会话(chunk x 2..=6、z 2..=5),破坏与本 REQ 开启;`pad` 时把 x 56..=69、z 60..=80 的地表一层(y = −61)换成石头
    /// (I = 18,只看复合体自身是否碎裂,不让地面被砸穿)。
    fn session(pad: bool) -> (Session, CommandLog) {
        let mut c = ShardSimConfig::new(shard());
        c.destruction = DestructionConfig {
            enabled: true,
            ..DestructionConfig::default()
        };
        c.cohesion = CohesionConfig {
            enabled: true,
            ..CohesionConfig::default()
        };
        let mut sim = ShardSim::new(c, registry());
        for x in 2..=6 {
            for z in 2..=5 {
                let cp = ChunkPos::new(x, z);
                sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
            }
        }
        let mut s = Session::new(cfg());
        s.register(sim);
        let mut log = CommandLog::new();
        if pad {
            // 每拍命令收件箱上限 256:逐行写、每行一拍。
            for x in 56..=69 {
                for z in 60..=80 {
                    for entry in s.set_block(shard(), p(x, -61, z), STONE).expect("石垫") {
                        log.append(entry);
                    }
                }
                s.tick();
            }
        }
        (s, log)
    }

    fn block(s: &Session, q: BlockPos) -> BlockId {
        let cp = q.chunk_of();
        s.voxel_chunk(shard(), cp).expect("chunk 已装载").get(
            (q.x - cp.x * 16) as usize,
            (q.y - WORLD_Y_MIN) as usize,
            (q.z - cp.z * 16) as usize,
        )
    }

    /// 生成一根 6 格梁:原点格中心 x = 62.5、给定 z,最低点离地 `h` 米(平放时原点 y = −59.475 + h)。
    fn drop_beam(
        s: &mut Session,
        log: &mut CommandLog,
        k: u32,
        z: f32,
        h: f32,
        block: BlockId,
        rotation: Quat,
    ) {
        let entry = s
            .spawn_owned(
                shard(),
                EntityId::from_parts(0, k),
                EntityState {
                    translation: Vec3::new(62.5, -59.475 + h, z),
                    rotation,
                    ..EntityState::default()
                },
                EntityDesc::compound(DEBRIS_HALF_SIZE, beam(block)),
                InputSource(7),
            )
            .expect("spawn");
        log.append(entry);
    }

    /// 跑到连续 2 拍没有实体(上限 900 拍),返回累计遥测。
    fn settle(s: &mut Session) -> (bool, CohesionStats) {
        let mut total = CohesionStats::default();
        let mut empty = 0;
        for _ in 0..900 {
            let n = s.tick();
            total.add(&n.cohesion);
            empty = if s.entity_states(shard()).expect("分片").is_empty() {
                empty + 1
            } else {
                0
            };
            if empty >= 2 {
                return (true, total);
            }
        }
        (false, total)
    }

    /// AC-C6 石梁平落 5 m 不碎(设计意图):落在石垫上,整块凝固回一排 6 格石头,期间不断任何粘结。
    ///
    /// 会变红的实现:逐格相加而不均摊(求解器把冲量集中在一格,5 m 就断)。
    #[test]
    fn acc_c6_stone_beam_flat_5m_stays_whole() {
        let (mut s, mut log) = session(true);
        drop_beam(&mut s, &mut log, 1, 64.5, 5.0, STONE, Quat::IDENTITY);
        let (done, total) = settle(&mut s);
        assert!(done);
        assert_eq!(total.bonds_broken, 0);
        assert_eq!(total.fracture_components, 0);
        assert_eq!(total.compounds_solidified, 1);
        for x in 60..=65 {
            assert_eq!(block(&s, p(x, -60, 64)), STONE, "x = {x}");
        }
    }

    /// AC-C6 土梁平落 8 m 碎开(设计意图:土桥面同高平落碎成几段):每个接触格均摊约 9.8 ≥ 7,5 根横向粘结全断,
    /// 拆成 6 个单格碎块,最终石垫上多出 6 格土。
    ///
    /// 会变红的实现:碎裂没接线;拆分后丢格。
    #[test]
    fn acc_c6_dirt_beam_flat_8m_shatters() {
        let (mut s, mut log) = session(true);
        drop_beam(&mut s, &mut log, 1, 64.5, 8.0, DIRT, Quat::IDENTITY);
        let (done, total) = settle(&mut s);
        assert!(done);
        assert_eq!(total.bonds_broken, 5);
        assert_eq!(total.fracture_components, 6);
        let mut dirt = 0;
        for x in 54..=72 {
            for y in -60..=-55 {
                for z in 58..=72 {
                    if block(&s, p(x, y, z)) == DIRT {
                        dirt += 1;
                    }
                }
            }
        }
        assert_eq!(dirt, 6, "6 格土落在石垫上");
    }

    /// AC-C6 角先着地:石梁倾斜 30° 从 8 m 高落下,最低的那一格先着地(demo:单格冲量约 30 ≥ 18)→ 断开、拆成至少 2 段。
    ///
    /// 会变红的实现:压向豁免写反(所有粘结都被当成压向)。
    #[test]
    fn acc_c6_corner_first_stone_beam_breaks() {
        let (mut s, mut log) = session(true);
        drop_beam(
            &mut s,
            &mut log,
            1,
            72.5,
            8.0 + 1.0,
            STONE,
            Quat::from_rotation_z(30f32.to_radians()),
        );
        let (done, total) = settle(&mut s);
        assert!(done);
        assert!(total.bonds_broken >= 1, "至少断一根:{total:?}");
        assert!(total.fracture_components >= 2, "至少拆成 2 段:{total:?}");
    }

    /// AC-C5 复合体砸地形按均摊口径(§7.3 第 7 条):两根石梁平落在土地上——2 m 那根每格 4.9 < 7,脚下的土一格都不脱落;
    /// 8 m 那根每格 9.8 ≥ 7,脚下 6 格土在同一拍一起脱落(写 air)。
    ///
    /// 会变红的实现:复合体仍按单格口径逐地形格相加(桩上实测:8 m 那根脚下的 6 格不会同拍一起脱落);阈值或均摊分母算错。
    #[test]
    fn acc_c5_compound_terrain_impact_averaged() {
        let (mut s, mut log) = session(false);
        drop_beam(&mut s, &mut log, 1, 64.5, 2.0, STONE, Quat::IDENTITY);
        drop_beam(&mut s, &mut log, 2, 72.5, 8.0, STONE, Quat::IDENTITY);
        let low: Vec<BlockPos> = (60..=65).map(|x| p(x, -61, 64)).collect();
        let high: Vec<BlockPos> = (60..=65).map(|x| p(x, -61, 72)).collect();
        let mut crushed_together = false;
        for _ in 0..300 {
            s.tick();
            for c in &low {
                assert_eq!(block(&s, *c), DIRT, "2 m 那根脚下的土 {c:?} 不脱落");
            }
            if high.iter().all(|c| block(&s, *c) == AIR) {
                crushed_together = true;
            }
        }
        assert!(crushed_together, "8 m 那根脚下 6 格土同一拍一起脱落");
    }
}

/// 单元 07:爆炸对复合体(AC-C9;REQ-010 §6.1,修订 8)。
#[allow(clippy::clone_on_copy)]
mod unit07 {
    use messaging::InputSource;
    use physics_sim::cohesion::{CellBlast, CohesionConfig, compound_blast};
    use physics_sim::destruction::{DEBRIS_HALF_SIZE, DestructionConfig};
    use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
    use shard::SchedulerConfig;
    use sim_api::session::{Session, SessionConfig};
    use types::glam::{Quat, Vec3};
    use types::{
        BlockId, BlockPos, ChunkPos, CompoundShape, DimensionId, EMERGENT_ORIGIN, EntityDesc,
        EntityId, EntityState, ShapeCell, ShardId, ShardPos, WorldId, WorldSeed,
    };
    use voxel::{BlockRegistry, SimpleRegistry, Superflat, WorldGenerator, builtin};

    const STONE: BlockId = builtin::STONE;
    const DIRT: BlockId = builtin::DIRT;
    const AIR: BlockId = builtin::AIR;

    fn beam(block: BlockId) -> CompoundShape {
        CompoundShape::new(
            (-2..=3)
                .map(|x| ShapeCell {
                    offset: [x, 0, 0],
                    block,
                })
                .collect(),
        )
        .expect("6 格梁")
    }

    fn near(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-3
    }

    fn intact(fate: CellBlast) -> Option<Vec3> {
        match fate {
            CellBlast::Intact { impulse } => impulse,
            other => panic!("期望完好,实得 {other:?}"),
        }
    }

    fn detach(fate: CellBlast) -> Vec3 {
        match fate {
            CellBlast::Detach { impulse } => impulse,
            other => panic!("期望脱落,实得 {other:?}"),
        }
    }

    /// AC-C9 逐格求值(§6.1 第 1 条):6 格梁各格中心 x = 60.5..=65.5,爆心在 x = 59.0(同一高度),power 12、radius 8 →
    /// 各格 P = 12·(1 − d/8) = 9.75、8.25、6.75、5.25、3.75、2.25:
    /// - 石梁(R 6、k 2):前 3 格脱落(冲量 P·dir',dir' = normalize(0.4, 1, 0)),后 3 格完好、受 (P, 0, 0) 的冲量——第 2 格的 P
    ///   没有被前面的格衰减(复合体自身的格不挡);
    /// - 土梁(R 2):前 4 格粉碎,后 2 格脱落;
    /// - 爆心与梁之间隔一格石头地形(x = 58):最近那格 P = 6.75 − 6 = 0.75,完好;冲量照单格公式不受遮挡;
    /// - 爆心在 20 m 外:全部完好、没有冲量。
    ///
    /// 会变红的实现:复合体自身的格也参与衰减;脱落冲量没按"向上为主"的方向;冲量用了压力而不是单格公式。
    #[test]
    fn acc_c9_compound_blast_rules() {
        let reg = SimpleRegistry::builtin();
        let state = EntityState {
            translation: Vec3::new(62.5, 10.5, 64.5),
            ..EntityState::default()
        };
        let open = |_: BlockPos| Some(AIR);
        let center = Vec3::new(59.0, 10.5, 64.5);
        let pressures = [9.75f32, 8.25, 6.75, 5.25, 3.75, 2.25];
        let up = Vec3::new(0.4, 1.0, 0.0).normalize();

        let stone = compound_blast(&state, &beam(STONE), center, 12.0, 8.0, 1.0, &open, &reg);
        assert_eq!(stone.len(), 6);
        for (i, fate) in stone.iter().enumerate() {
            if i < 3 {
                assert!(near(detach(*fate), up * pressures[i]), "石梁第 {i} 格脱落");
            } else {
                let imp = intact(*fate).expect("半径内有冲量");
                assert!(
                    near(imp, Vec3::new(pressures[i], 0.0, 0.0)),
                    "石梁第 {i} 格冲量 {imp:?}"
                );
            }
        }

        let dirt = compound_blast(&state, &beam(DIRT), center, 12.0, 8.0, 1.0, &open, &reg);
        for (i, fate) in dirt.iter().enumerate() {
            if i < 4 {
                assert_eq!(*fate, CellBlast::Pulverize, "土梁第 {i} 格粉碎");
            } else {
                assert!(near(detach(*fate), up * pressures[i]), "土梁第 {i} 格脱落");
            }
        }

        let wall = |q: BlockPos| {
            Some(if q == BlockPos::new(58, 10, 64) {
                STONE
            } else {
                AIR
            })
        };
        let behind = Vec3::new(57.0, 10.5, 64.5);
        let shielded = compound_blast(&state, &beam(STONE), behind, 12.0, 8.0, 1.0, &wall, &reg);
        let imp = intact(shielded[0]).expect("半径内有冲量");
        assert!(
            near(imp, Vec3::new(6.75, 0.0, 0.0)),
            "被地形挡住的格完好,冲量照单格公式:{imp:?}"
        );

        let far = compound_blast(
            &state,
            &beam(STONE),
            Vec3::new(40.0, 10.5, 64.5),
            12.0,
            8.0,
            1.0,
            &open,
            &reg,
        );
        assert!(
            far.iter()
                .all(|f| *f == CellBlast::Intact { impulse: None }),
            "半径外"
        );
    }

    fn shard() -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0))
    }

    /// 零重力会话(破坏与本 REQ 开启、凝固关闭),梁悬在 y = −40 附近,离地面 > 20 m(爆炸不碰地形)。
    fn session() -> Session {
        let mut c = ShardSimConfig::new(shard());
        c.gravity = Vec3::ZERO;
        c.destruction = DestructionConfig {
            enabled: true,
            ..DestructionConfig::default()
        };
        c.cohesion = CohesionConfig {
            enabled: true,
            ..CohesionConfig::default()
        };
        c.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        let mut sim = ShardSim::new(
            c,
            Box::new(SimpleRegistry::builtin()) as Box<dyn BlockRegistry>,
        );
        for x in 2..=6 {
            for z in 2..=5 {
                let cp = ChunkPos::new(x, z);
                sim.load_chunk(cp, Superflat::classic().generate(WorldSeed(7), cp));
            }
        }
        let mut s = Session::new(SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        });
        s.register(sim);
        s
    }

    fn spawn_beam(s: &mut Session) -> EntityId {
        let id = EntityId::from_parts(0, 1);
        s.spawn_owned(
            shard(),
            id,
            EntityState {
                translation: Vec3::new(62.5, -39.5, 64.5),
                ..EntityState::default()
            },
            EntityDesc::compound(DEBRIS_HALF_SIZE, beam(STONE)),
            InputSource(7),
        )
        .expect("spawn");
        s.tick();
        s.tick();
        id
    }

    /// AC-C9 炸碎复合体(§6.1 第 3 条):零重力里悬着的石梁,爆心在左端外 1.5 m(power 12、radius 8)→ 原实体当拍删除、
    /// 计移除 3 格;下一拍出现 4 个新实体:先是剩下 3 格组成的复合体(形状 −1..=1、原点在 x ≈ 64.5,带着冲量向 +x 飞,
    /// 速度 ≈ 11.25 / 2.187 ≈ 5.1 m/s),再是 3 个脱落的单格碎块(不带形状、向上飞)。
    ///
    /// 会变红的实现:爆炸对复合体只在中心施一次冲量、不破坏;破坏后没有拆分;剩余部分丢了冲量;编号顺序不对。
    #[test]
    fn acc_c9_blast_breaks_compound_into_pieces() {
        let mut s = session();
        let id = spawn_beam(&mut s);
        s.detonate(Vec3::new(59.0, -39.5, 64.5), 12.0, 8.0)
            .expect("引爆");
        let n = s.tick();
        assert_eq!(n.cohesion.blast_removed_cells, 3);
        assert!(s.entity_state(shard(), id).is_none(), "原实体当拍删除");
        s.tick();
        let mut got: Vec<(EntityId, EntityState)> = s
            .entity_states(shard())
            .expect("分片")
            .into_iter()
            .filter(|(e, _)| e.origin_shard() == EMERGENT_ORIGIN)
            .collect();
        got.sort_by_key(|(e, _)| *e);
        assert_eq!(got.len(), 4, "1 个剩余复合体 + 3 个脱落碎块");
        let (first, rest) = (got[0], &got[1..]);
        let desc = s.entity_desc(shard(), first.0).expect("描述");
        let offsets: Vec<[i32; 3]> = desc
            .shape()
            .expect("剩余部分是复合体")
            .cells()
            .iter()
            .map(|c| c.offset)
            .collect();
        assert_eq!(offsets, vec![[-1, 0, 0], [0, 0, 0], [1, 0, 0]]);
        assert!(
            (first.1.translation.x - 64.5).abs() < 0.3,
            "{:?}",
            first.1.translation
        );
        assert!(
            first.1.linvel.x > 4.0,
            "剩余部分带着冲量:{:?}",
            first.1.linvel
        );
        for (e, st) in rest {
            let d = s.entity_desc(shard(), *e).expect("描述");
            assert!(d.shape().is_none(), "脱落格是单格碎块");
            assert_eq!(d.block, STONE);
            assert!(st.linvel.y > 0.0, "脱落碎块向上飞:{:?}", st.linvel);
        }
    }

    /// AC-C9 冲量产生力矩(§6.1 第 2 条,D6):爆心在梁左端的左下方,power 4(各格压力 ≤ 2.75 < R 6,不破坏)→ 梁还是同一个
    /// 实体,形状不变,并绕 z 轴转起来(估算 ω_z ≈ −0.52 rad/s)。
    ///
    /// 会变红的实现:只在中心施一次冲量(没有力矩,角速度为 0)。
    #[test]
    fn acc_c9_blast_spins_intact_compound() {
        let mut s = session();
        let id = spawn_beam(&mut s);
        s.detonate(Vec3::new(59.0, -41.5, 64.5), 4.0, 8.0)
            .expect("引爆");
        s.tick();
        s.tick();
        let st = s
            .entity_state(shard(), id)
            .expect("没有被破坏,还是同一个实体");
        assert_eq!(
            s.entity_desc(shard(), id),
            Some(EntityDesc::compound(DEBRIS_HALF_SIZE, beam(STONE)))
        );
        assert!(st.angvel.z < -0.2, "绕 z 轴转起来:{:?}", st.angvel);
        assert!(st.rotation != Quat::IDENTITY || st.angvel.length() > 0.2);
    }
}

/// 单元 08:默认开启、配置表、迁移与图形冒烟(AC-C13、AC-C15、AC-C16;REQ-010 §11、§12.1,修订 9)。
/// 图形冒烟(AC-C15)由 `smoke/scenarios-pending/req010-*.toml` 四个场景承担,机器检查通过后移入 `smoke/scenarios/`。
mod unit08 {
    use app::config::Config;
    use physics_sim::ShardSimConfig;
    use physics_sim::cohesion::CohesionConfig;
    use types::{DimensionId, ShardId, ShardPos, WorldId};

    fn shard() -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0))
    }

    /// AC-C16 默认开启(D9):核内默认、分片默认、app 配置默认、装配出的分片配置都开启。
    ///
    /// 会变红的实现:只改了某一层的默认值。
    #[test]
    #[ignore = "REQ-010 单元 08 待实施方解除"]
    fn acc_c16_default_on() {
        assert!(CohesionConfig::default().enabled);
        assert!(ShardSimConfig::new(shard()).cohesion.enabled);
        assert!(Config::default().cohesion.enabled);
        assert!(Config::default().shard_sim_config(shard()).cohesion.enabled);
    }

    /// AC-C13 配置表 `[cohesion]`(§12.1 第 2 条):默认值;全键覆写 1:1 直传到分片配置;toml 往返;只写部分键时其余取默认;
    /// 校验拒绝未知键、`load_max_cells = 0`、`compound_split_size` 为 0 或超出余量(默认 16 m 下 S = 8 → 8.58 > 8;
    /// margin 8 m 下 S = 6 → 6.85 > 4)、`max_converted_cells_per_tick < S³`、`max_fractures_per_tick = 0`、负的上移格数;
    /// S = 7 与默认余量相容(7.71 ≤ 8,1024 ≥ 343)。
    ///
    /// 会变红的实现:映射漏字段或写成默认值;校验缺项;余量没按 `margin_meters` 算。
    #[test]
    #[ignore = "REQ-010 单元 08 待实施方解除"]
    fn acc_c13_cohesion_config_table() {
        let d = Config::default().cohesion;
        assert_eq!(
            (
                d.load_max_cells,
                d.compound_split_size,
                d.max_converted_cells_per_tick,
                d.max_fractures_per_tick,
                d.compound_snap_lift
            ),
            (2048, 6, 1024, 16, 2)
        );
        let text = "[cohesion]\nenabled = false\nload_max_cells = 900\ncompound_split_size = 4\n\
                    max_converted_cells_per_tick = 64\nmax_fractures_per_tick = 3\ncompound_snap_lift = 1\n";
        let c = Config::from_toml_str(text).expect("合法");
        assert_eq!(
            c.shard_sim_config(shard()).cohesion,
            CohesionConfig {
                enabled: false,
                load_max_cells: 900,
                compound_split_size: 4,
                max_converted_cells_per_tick: 64,
                max_fractures_per_tick: 3,
                compound_snap_lift: 1,
            }
        );
        assert_eq!(Config::from_toml_str(&c.to_toml_string()).expect("往返"), c);
        let partial = Config::from_toml_str("[cohesion]\nload_max_cells = 100\n").expect("部分键");
        assert_eq!(partial.cohesion.load_max_cells, 100);
        assert_eq!(partial.cohesion.compound_split_size, 6, "未写的键取默认");
        assert!(
            Config::from_toml_str("[cohesion]\ncompound_split_size = 7\n").is_ok(),
            "S = 7 与默认余量相容"
        );
        for bad in [
            "[cohesion]\nfoo = 1\n",
            "[cohesion]\nload_max_cells = 0\n",
            "[cohesion]\ncompound_split_size = 0\n",
            "[cohesion]\ncompound_split_size = 8\n",
            "margin_meters = 8\n[cohesion]\ncompound_split_size = 6\n",
            "[cohesion]\ncompound_split_size = 6\nmax_converted_cells_per_tick = 215\n",
            "[cohesion]\nmax_fractures_per_tick = 0\n",
            "[cohesion]\ncompound_snap_lift = -1\n",
        ] {
            assert!(Config::from_toml_str(bad).is_err(), "应拒绝:{bad}");
        }
    }
}
