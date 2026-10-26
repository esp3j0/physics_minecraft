"""collapse.py — REQ-009 任务 00 脚本 2:结构坍塌搜索(纯体素)。

对比 6 连通 BFS(按 BlockPos 序 FIFO)与向下优先(优先队列,键 (y,x,z) 升序)两种
坍塌搜索:各结构的悬空块大小、访问格数、耗时;锚定规则(bedrock / y 最底层 /
进入未装载 chunk / 超预算)与锚定标记缓存;以全量连通分量标注为真值核对;
扫描 collapse_max_cells 预算的正确性边界;跨分片"晚一拍"逐拍模拟
(各拍转换并集 = 单世界结果,无重复转换)。

结构:① 石桥炸一根柱;①b 石桥炸两根柱;② 悬臂切根;③ 1×1×20 塔切底;
④ 4 柱 20×20 平台全断;⑤ 带空洞厚板全断;⑥⑦ 平地/20 层地基移除单格(必锚定)。

运行:.venv/Scripts/python.exe demos/terrain-destruction/collapse.py
"""
import heapq
import os
import sys
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import AIR, BEDROCK, DIRT, STONE

DIRS6 = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
INF_BUDGET = 1 << 60


def dkey(c):
    """向下优先队列的键:(y,x,z) 升序。"""
    return (c[1], c[0], c[2])


class VWorld:
    """有限体素世界;边界外视为未装载 chunk(flood 返回 None)。"""

    def __init__(self, vox, x0, y0, z0):
        self.vox = vox
        self.x0, self.y0, self.z0 = x0, y0, z0

    def get(self, c):
        ix, iy, iz = c[0] - self.x0, c[1] - self.y0, c[2] - self.z0
        if ix < 0 or iy < 0 or iz < 0 or ix >= self.vox.shape[0] \
                or iy >= self.vox.shape[1] or iz >= self.vox.shape[2]:
            return None
        return int(self.vox[ix, iy, iz])

    def set_air(self, c):
        self.vox[c[0] - self.x0, c[1] - self.y0, c[2] - self.z0] = AIR

    def cells(self):
        for ix, iy, iz in zip(*np.nonzero(self.vox != AIR)):
            yield (ix + self.x0, iy + self.y0, iz + self.z0)


def ground(deep=False, nx=41, nz=41):
    """超平坦(x/z -20..20):bedrock + 3 层 dirt;deep=True:bedrock y=-21 + 20 层 stone。"""
    if deep:
        vox = np.zeros((nx, 41, nz), np.int8)
        vox[:, 0, :] = BEDROCK
        vox[:, 1:21, :] = STONE
        return VWorld(vox, -20, -21, -20)
    vox = np.zeros((nx, 24, nz), np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:4, :] = DIRT
    return VWorld(vox, -20, -4, -20)


def fill(w, cells):
    for (x, y, z) in cells:
        w.vox[x - w.x0, y - w.y0, z - w.z0] = STONE


def remove(w, cells):
    for (x, y, z) in cells:
        w.vox[x - w.x0, y - w.y0, z - w.z0] = AIR


def seeds_of(removed):
    """被改格的 6 邻居(调用方过滤实心)。"""
    out = set()
    for (x, y, z) in removed:
        for d in DIRS6:
            out.add((x + d[0], y + d[1], z + d[2]))
    out -= set(removed)
    return out


def flood(w, seed_set, budget, order, marks):
    """单种子洪泛。返回 (anchored, visited_set, budget_hit)。

    锚定:命中 bedrock / y=y_min / 未装载 chunk / 锚定标记 / 访问数超预算。
    """
    seen = set()
    if order == "bfs":
        q = deque(sorted(seed_set))
        pop, push = q.popleft, q.append
    else:
        h = [(dkey(c), c) for c in seed_set]
        heapq.heapify(h)

        def pop():
            return heapq.heappop(h)[1]

        def push(c):
            heapq.heappush(h, (dkey(c), c))
    while True:
        try:
            c = pop()
        except IndexError:
            break
        if c in seen:
            continue
        seen.add(c)
        if len(seen) > budget:
            return True, seen, True
        if c in marks:
            return True, seen, False
        b = w.get(c)
        if b is None or b == BEDROCK:
            return True, seen, False
        x, y, z = c
        for dx, dy, dz in DIRS6:
            n = (x + dx, y + dy, z + dz)
            if n not in seen and w.get(n):
                push(n)
    return False, seen, False


def collapse_search(w, removed, budget, order, use_marks=True):
    """多种子驱动:返回 (floating_cells, anchored_components, visited_total, budget_hits)。

    已被先前种子覆盖的格跳过;锚定(非预算耗尽)组件的访问格写入标记缓存。
    """
    seeds = sorted(c for c in seeds_of(removed) if w.get(c))
    marks, floating, anchored_n = set(), set(), 0
    visited_total, budget_hits = 0, 0
    for s in seeds:
        if s in marks or s in floating:
            continue
        anchored, seen, bh = flood(w, {s}, budget, order, marks)
        visited_total += len(seen)
        if anchored:
            anchored_n += 1
            if bh:
                budget_hits += 1
            elif use_marks:
                marks |= seen
        else:
            floating |= seen
    return floating, anchored_n, visited_total, budget_hits


def truth(w, removed):
    """全量连通分量标注:返回 (anchored_total, floating_components(list[set]))。"""
    removed = set(removed)
    todo = {c for c in w.cells() if c not in removed}
    floating, anchored_total = [], 0
    while todo:
        s = min(todo)
        comp = {s}
        q = deque([s])
        todo.discard(s)
        while q:
            x, y, z = q.popleft()
            for dx, dy, dz in DIRS6:
                n = (x + dx, y + dy, z + dz)
                if n in todo:
                    todo.discard(n)
                    comp.add(n)
                    q.append(n)
        anchored = False
        for (x, y, z) in comp:
            if w.get((x, y, z)) == BEDROCK or y == w.y0 \
                    or x in (w.x0, w.x0 + w.vox.shape[0] - 1) \
                    or z in (w.z0, w.z0 + w.vox.shape[2] - 1):
                anchored = True
                break
        if anchored:
            anchored_total += 1
        else:
            floating.append(comp)
    return anchored_total, floating


def build_structures():
    """返回 {name: (world, removed, note)};removed 格在搭建后真实移除。"""
    out = {}

    def reg(name, w, removed, note):
        remove(w, removed)
        out[name] = (w, removed, note)

    pillar1 = [(0, y, 0) for y in range(6)]
    pillar2 = [(10, y, 0) for y in range(6)]
    deck = [(x, 6, 0) for x in range(-1, 11)]

    w = ground()
    fill(w, pillar1 + pillar2 + deck)
    reg("1-桥·炸一根柱", w, pillar1, "桥面仍连另一柱 → 预期锚定(连通性判据的已知限制)")

    w = ground()
    fill(w, pillar1 + pillar2 + deck)
    reg("1b-桥·炸两根柱", w, pillar1 + pillar2, "桥面 12 格整体悬空")

    w = ground()
    wall = [(x, y, z) for x in (-3, -2, -1) for y in range(8) for z in range(-3, 4)]
    arm = [(x, 5, 0) for x in range(0, 8)]
    fill(w, wall + arm)
    reg("2-悬臂·切根", w, [(0, 5, 0)], "横臂剩余 7 格悬空")

    w = ground()
    tower = [(0, y, 0) for y in range(20)]
    fill(w, tower)
    reg("3-塔·切底", w, [(0, 0, 0)], "塔身剩余 19 格悬空")

    w = ground()
    plat = [(x, 6, z) for x in range(-10, 10) for z in range(-10, 10)]
    legs = [(x, y, z) for (x, z) in ((-9, -9), (-9, 8), (8, -9), (8, 8)) for y in range(6)]
    fill(w, plat + legs)
    reg("4-平台·4柱全断", w, legs, "平台 400 格整体悬空")

    w = ground()
    slab = [(x, y, z) for x in range(-5, 5) for z in range(-5, 5) for y in (6, 7)]
    slab = [c for c in slab if not (c[1] == 6 and -3 <= c[0] <= 2 and -3 <= c[2] <= 2)]
    legs5 = [(x, y, z) for (x, z) in ((-5, -5), (-5, 4), (4, -5), (4, 4)) for y in range(6)]
    fill(w, slab + legs5)
    reg("5-空洞厚板·全断", w, legs5, "厚板 164 格悬空(内部空洞不进洪泛)")

    w = ground()
    reg("6-平地·移除单格", w, [(0, -1, 0)], "必须判定锚定")

    w = ground(deep=True)
    reg("7-20层地基·移除单格", w, [(0, -1, 0)], "必须判定锚定;锚定洪泛最贵")

    return out


def match_truth(fl, float_t):
    return set(fl) == set().union(*float_t) if float_t else not fl


def main():
    structs = build_structures()
    print("===== 各结构:真值 / BFS vs 向下优先(预算 ∞,含标记缓存) =====")
    print(f"{'结构':<16} {'搜索':<5} {'真值':<12} {'判定':<10} {'访问格':>7} {'预算命中':>6} 核对")
    for name, (w, removed, note) in structs.items():
        anch_t, float_t = truth(w, removed)
        tsize = sum(len(c) for c in float_t)
        tv = "锚定" if not float_t else f"悬空{tsize}格"
        for order, label in (("bfs", "BFS"), ("down", "向下")):
            fl, anch_n, vis, bh = collapse_search(w, removed, INF_BUDGET, order)
            verdict = "锚定" if not fl else f"悬空{len(fl)}格"
            ok = "✓" if match_truth(fl, float_t) else "✗"
            print(f"{name:<16} {label:<5} {tv:<12} {verdict:<10} {vis:>7} {bh:>6}   {ok}")
        print(f"{'':<16} 说明:{note}")

    print("\n===== 耗时(预算 ∞、全新标记;Python 集合/堆实现,Rust 预期快 1–2 个量级) =====")
    for name, (w, removed, note) in structs.items():
        row = []
        for order in ("bfs", "down"):
            t0 = time.perf_counter()
            collapse_search(w, removed, INF_BUDGET, order)
            row.append(f"{order}={(time.perf_counter()-t0)*1000:.2f}ms")
        print(f"{name:<16} " + "  ".join(row))

    print("\n===== 锚定标记缓存的价值(结构7,5 个种子逐个洪泛,BFS) =====")
    w, removed, _ = structs["7-20层地基·移除单格"]
    seeds = sorted(c for c in seeds_of(removed) if w.get(c))
    for use_marks in (False, True):
        vis_all, marks = [], set()
        for s in seeds:
            anchored, seen, bh = flood(w, {s}, INF_BUDGET, "bfs", marks if use_marks else set())
            vis_all.append(len(seen))
            if anchored and not bh and use_marks:
                marks |= seen
        print(f"标记={use_marks!s:<5} 各种子访问 {vis_all} 合计 {sum(vis_all)}")

    print("\n===== 预算扫描(collapse_max_cells,向下优先;预算耗尽按锚定处理) =====")
    budgets = [64, 256, 512, 1024, 4096]
    head = f"{'预算':>8} | " + " | ".join(f"{n.split('-')[0]:>2}" for n in structs) + " | 结论"
    print(head)
    for b in budgets:
        verdicts, all_ok = [], True
        for name, (w, removed, note) in structs.items():
            anch_t, float_t = truth(w, removed)
            fl, _, _, _ = collapse_search(w, removed, b, "down")
            ok = match_truth(fl, float_t)
            all_ok &= ok
            verdicts.append("✓" if ok else "✗")
        print(f"{b:>8} | " + " | ".join(f"{v:>2}" for v in verdicts)
              + f" | {'全部与真值一致' if all_ok else '悬空块被预算误判为锚定'}")

    print("\n===== 跨分片:石桥横跨 x=0,炸两根柱,晚一拍模拟 =====")
    w = ground()
    pA = [(-5, y, 0) for y in range(6)]
    pB = [(4, y, 0) for y in range(6)]
    deck2 = [(x, 6, 0) for x in range(-6, 6)]
    fill(w, pA + pB + deck2)
    w_single = VWorld(w.vox.copy(), w.x0, w.y0, w.z0)
    remove(w_single, pA + pB)
    single_fl, _, _, _ = collapse_search(w_single, pA + pB, INF_BUDGET, "down")
    single = set(single_fl)
    print(f"单世界(两柱即刻消失):悬空 {len(single)} 格(桥面 x=-6..5,y=6,z=0)")

    own_rem = {"A": set(pA), "B": set(pB)}
    region = {"A": lambda c: c[0] < 0, "B": lambda c: c[0] >= 0}
    views, conv_log, hidden = {}, {}, {}
    for s in ("A", "B"):
        views[s] = VWorld(w.vox.copy(), w.x0, w.y0, w.z0)
        conv_log[s] = {}
        hidden[s] = set()
    all_converted = {}
    for tick in range(4):
        for s in ("A", "B"):
            new_hidden = set()
            if tick == 0:
                new_hidden |= own_rem[s]                       # 自己的爆炸写入立即可见
                for c in own_rem[s]:
                    views[s].set_air(c)
            else:
                for other in ("A", "B"):
                    if other == s:
                        continue
                    new_hidden |= conv_log[other].get(tick - 1, set())  # BlockWrite N+1
                    if tick == 1:
                        new_hidden |= own_rem[other]
                for c in new_hidden:
                    views[s].set_air(c)
            hidden[s] = new_hidden
        print(f"-- tick {tick} --")
        for s in ("A", "B"):
            if not hidden[s]:
                print(f"  分片 {s}: 无新写入,不触发搜索")
                continue
            seeds = sorted(c for c in seeds_of(hidden[s]) if views[s].get(c))
            conv, marks = set(), set()
            for cs in seeds:
                if cs in marks or cs in conv:
                    continue
                anchored, seen, bh = flood(views[s], {cs}, INF_BUDGET, "down", marks)
                if anchored:
                    marks |= seen
                else:
                    conv |= {c for c in seen if region[s](c)}
            for c in conv:
                views[s].set_air(c)
            conv_log[s][tick] = conv
            for c in conv:
                assert c not in all_converted, f"重复转换:{c}"
                all_converted[c] = s
            print(f"  分片 {s}: 触发种子 {len(seeds)},转换本体区 {len(conv)} 格")
    union = set(all_converted)
    print(f"各拍转换并集 = {len(union)} 格;与单世界结果一致:{union == single};"
          f"无重复转换:断言通过")
    print("推荐:collapse_max_cells 见预算扫描;collapse_max_cluster ≥ 最大需塌落悬空块"
          "(平台 400 → 取 512);搜索顺序 = 向下优先")


if __name__ == "__main__":
    main()
