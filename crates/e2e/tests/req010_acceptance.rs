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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
    #[ignore = "REQ-010 单元 01 待实施方解除"]
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
