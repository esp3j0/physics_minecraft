"""load.py — REQ-010 任务 00 脚本 1:承载判定(纯体素,最大流)。

按 REQ-010 §4 实现承载判定(接地格按柱缓存、悬挑区、整数容量网络、最大流),
Edmonds–Karp 与 Dinic 各算一遍核对 F 唯一。13 组结构 + B 扫描 + 切块(D3)+
跨分片晚一拍(D7)。

运行:.venv/Scripts/python.exe demos/cohesion/load.py
"""
import importlib.util
import os
import random
import sys
import time
from collections import deque

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _here)
from common import (AIR, BEDROCK, BLOCK_CHAR, DIRT, INF_CAP, LOAD_MAX_CELLS, SAND,
                    STONE, LoadEval, VWorld, cut_segments, flat_ground,
                    origin_and_rho, trigger_of)

# demos/terrain-destruction/common.py 与本目录 common.py 同名,按路径加载
_here = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "td_common", os.path.join(_here, "..", "terrain-destruction", "common.py"))
td = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(td)
_spec2 = importlib.util.spec_from_file_location(
    "td_collapse", os.path.join(_here, "..", "terrain-destruction", "collapse.py"))
tdc = importlib.util.module_from_spec(_spec2)
_spec2.loader.exec_module(tdc)

B_REC = {AIR: 0, STONE: 8, DIRT: 3, SAND: 0, BEDROCK: INF_CAP}   # §3.2 占位


def ev(w, B=None):
    return LoadEval(w, B or B_REC)


def fmt(r):
    if r["truncated"]:
        return f"截断→稳定 访问{r['visited']}"
    if not r["f"]:
        return "稳定"
    return f"F={len(r['f'])} 断键{r['cut_bonds']} 流{r['flow']}"


def profile(w, f, z=0, top=8, bottom=-4, x_range=None):
    """x–y 剖面(z 固定);F 标 F,其余按材料。"""
    fs = set(f)
    rows = []
    xs = x_range or range(w.x0, w.x0 + w.nx)
    for y in range(top, bottom - 1, -1):
        row = []
        for x in xs:
            c = (x, y, z)
            if c in fs:
                row.append("F")
            else:
                b = w.get(c)
                row.append(BLOCK_CHAR[b] if b is not None else "?")
        rows.append(f"{y:>3} " + "".join(row))
    rows.append("    " + "".join(str(x % 10) for x in xs))
    return "\n".join(rows)


def door(B_lintel, lintel_block):
    """门框:柱 x=-3/x=3 高 5,门楣 x=-3..3 y=5;返回 (w, removed)。"""
    w = flat_ground()
    w.fill([(-3, y, 0) for y in range(5)] + [(3, y, 0) for y in range(5)])
    w.fill([(x, 5, 0) for x in range(-3, 4)], lintel_block)
    removed = [(-3, y, 0) for y in range(5)]
    w.remove(removed)
    return w, removed


def single_pillar_bridge(L, pillar_at="end"):
    """单柱桥:桥面 y=5 x=0..L-1,柱在端/中;返回 (w, removed柱)。"""
    w = flat_ground()
    px = 0 if pillar_at == "end" else L // 2
    w.fill([(px, y, 0) for y in range(5)])
    w.fill([(x, 5, 0) for x in range(L)])
    removed = [(px, y, 0) for y in range(5)]
    w.remove(removed)
    return w, removed


def cantilever(L, thick, block):
    """石墙(x=-3..-1,z=-1..1)伸出的悬臂,y=5 起 x=0..L-1;不移除任何格,
    触发格 = 悬臂自身(按"放置即触发",D8 同口径)。"""
    w = flat_ground()
    wall = [(x, y, z) for x in (-3, -2, -1) for y in range(5 + thick) for z in (-1, 0, 1)]
    w.fill(wall)
    arm = [(x, 5 + j, 0) for x in range(L) for j in range(thick)]
    w.fill(arm, block)
    return w, arm


def arch():
    """跨度 9 的半圆石拱(带 8≤x²+y²≤16, y≥0);返回 (w, keystone, leg)。"""
    w = flat_ground()
    cells = [(x, y, 0) for x in range(-4, 5) for y in range(0, 5)
             if 8 <= x * x + y * y <= 16]
    w.fill(cells)
    return w, (0, 4, 0), [(-4, 0, 0), (-3, 0, 0)]


def tbeam(k):
    """石柱 x=0 高 5 + 对称横梁 x=-k..k,y=5;触发 = 横梁(放置口径)。"""
    w = flat_ground()
    w.fill([(0, y, 0) for y in range(5)])
    beam = [(x, 5, 0) for x in range(-k, k + 1)]
    w.fill(beam)
    return w, beam


def tower():
    w = flat_ground()
    w.fill([(0, y, 0) for y in range(20)])
    removed = [(0, 0, 0)]
    w.remove(removed)
    return w, removed


def platform():
    """20×20 平台 y=6,4 柱;返回 (w, legs)。"""
    w = flat_ground()
    plat = [(x, 6, z) for x in range(-10, 10) for z in range(-10, 10)]
    legs = [(x, y, z) for (x, z) in ((-9, -9), (-9, 8), (8, -9), (8, 8))
            for y in range(6)]
    w.fill(plat)
    w.fill(legs)
    return w, legs


def edge_slab(half=15):
    """half*2+1 见方 1 格厚石板 y=6,四边有支撑墙;返回 (w, triggers=全板)。"""
    w = flat_ground(61, 61)
    wall = []
    for i in range(-half, half + 1):
        wall += [(i, y, -half) for y in range(6)] + [(i, y, half) for y in range(6)]
        wall += [(-half, y, i) for y in range(6)] + [(half, y, i) for y in range(6)]
    w.fill(wall)
    slab = [(x, 6, z) for x in range(-half, half + 1) for z in range(-half, half + 1)]
    w.fill(slab)
    return w, slab


def main():
    print("=" * 72)
    print("1–13 结构(B 按占位:stone 8 / dirt 3;EK 与 Dinic 各算一遍)")
    print("=" * 72)
    hdr = (f"{'结构':<24}{'结果':<18}{'访问':>5}{'区域ms':>8}{'EK ms':>8}{'Dinic ms':>9} {'一致':>4}")
    print(hdr)
    saved_F = {}

    def run(name, w, triggers, note="", prof=None):
        r = ev(w).evaluate(triggers)
        verdict = fmt(r)
        ok = "✓" if r["agree"] else "✗"
        tf = r["t_flows"]
        print(f"{name:<24}{verdict:<18}{r['visited']:>5}{r['t_region']*1e3:>8.2f}"
              f"{tf.get('ek', 0)*1e3:>8.2f}{tf.get('dinic', 0)*1e3:>9.2f} {ok:>4}")
        if note:
            print(f"{'':<24}{note}")
        if prof:
            print(profile(w, r["f"] if r["f"] else set(), *prof))
        saved_F[name] = r["f"]
        return r

    # 1 门框
    w_door, _ = door(3, DIRT)
    run("1a 门楣=土(B3)", w_door, trigger_of([(-3, y, 0) for y in range(5)]),
        note="土门楣悬空 6 格整断(设计意图 ✓)", prof=(0, 6))
    run("1b 门楣=石(B8)", *door(3, STONE), note="石门楣挂得住(设计意图 ✓)")
    assert saved_F["1b 门楣=石(B8)"] == set()

    # 2 单柱桥(柱被移走)
    for pos in ("end", "center"):
        for L in (4, 8, 12, 16, 20):
            w, removed = single_pillar_bridge(L, pos)
            r = run(f"2 单柱桥 L={L} 柱={pos[:3]}", w, trigger_of(removed))
            assert len(r["f"]) == L, (L, pos, len(r["f"]))

    # 3 悬臂扫描(放置口径)
    print("--- 悬臂:长 1..20,厚 1/2,土/石(放置即触发)---")
    for block, bn in ((STONE, "石"), (DIRT, "土")):
        for thick in (1, 2):
            holds = []
            for L in range(1, 21):
                w, trig = cantilever(L, thick, block)
                r = ev(w).evaluate(trig)
                (holds if not r["f"] else []).append(L)
            print(f"  {bn}悬臂 厚{thick}: 挂住 1..{max(holds) if holds else 0}"
                  f"(B={'石8' if block == STONE else '土3'})")

    # 4 悬臂加重(D8)
    print("--- 4 悬臂加重(D8):8 格石悬臂末端上方放 1/2/4 格 ---")
    for n in (0, 1, 2, 4):
        w, arm = cantilever(8, 1, STONE)
        weight = [(5 + i, 6, 0) for i in range(n)]
        w.fill(weight, STONE)
        trig = sorted(set(trigger_of(weight)) | set(arm))
        r = ev(w).evaluate(trig)
        print(f"  加重 {n} 格: {fmt(r)}")

    # 5 拱
    w, key, leg = arch()
    full = [(x, y, 0) for x in range(-4, 5) for y in range(0, 5) if 8 <= x * x + y * y <= 16]
    r = run("5a 拱·完好", w, full, prof=(0, 5))
    w, key, leg = arch()
    w.remove([key])
    run("5b 拱·移走拱顶", w, trigger_of([key]), prof=(0, 4))
    w, key, leg = arch()
    w.remove(leg)
    run("5c 拱·移走左腿", w, trigger_of(leg), prof=(0, 4))

    # 6 T 形
    for k in range(3, 11):
        w, beam = tbeam(k)
        r = run(f"6 T形 侧长{k}", w, beam)

    # 7 塔
    w, removed = tower()
    run("7 塔·切底", w, trigger_of(removed), prof=(0, 7), note="19 格悬空,无力矩模型下笔直下坠")

    # 8 平台逐根移柱
    w, legs = platform()
    for i in range(5):
        w2_flat = flat_ground()
        plat = [(x, 6, z) for x in range(-10, 10) for z in range(-10, 10)]
        legset = [(x, y, z) for (x, z) in ((-9, -9), (-9, 8), (8, -9), (8, 8)) for y in range(6)]
        w2 = w2_flat
        w2.fill(plat)
        w2.fill(legset)
        removed = legset[:i * 6]
        w2.remove(removed)
        run(f"8 平台 移走{i}根柱", w2, trigger_of(removed) if removed else sorted(plat))

    # 9 沙子(与 REQ-009 NeedsBelow 逐格对照)
    print("--- 9 沙子 vs REQ-009 NeedsBelow ---")

    def needsbelow_final(w, removed):
        """REQ-009 口径:反复移除"正下方为空"的沙,直到不动点;返回剩余实心格。"""
        w = VWorld(w.vox.copy(), w.x0, w.y0, w.z0)
        w.remove(removed)
        while True:
            fall = [c for c in w.solid_cells() if w.get(c) == SAND
                    and w.get((c[0], c[1] - 1, c[2])) in (AIR,)]
            if not fall:
                return w.solid_cells()
            w.remove(fall)

    # 9a 墙侧沙
    w = flat_ground()
    w.fill([(0, y, 0) for y in range(3)])
    w.fill([(1, 1, 0)], SAND)
    r = ev(w).evaluate([(1, 1, 0)])
    remain = needsbelow_final(w, [])
    same = (r["f"] == {(1, 1, 0)}) and ((1, 1, 0) not in remain)
    print(f"  9a 墙侧沙(下方空): F={sorted(r['f'])} NeedsBelow=掉 一致={same}")
    # 9b 沙柱、底块被移走
    w = flat_ground()
    w.fill([(0, 0, 0)], STONE)
    w.fill([(0, y, 0) for y in (1, 2, 3)], SAND)
    removed = [(0, 0, 0)]
    w.remove(removed)
    r = ev(w).evaluate(trigger_of(removed))
    remain = needsbelow_final(w, removed)
    load_falls = r["f"]
    req_falls = {(0, y, 0) for y in (1, 2, 3)} - remain
    print(f"  9b 沙柱底块移走: 本拍 F={len(load_falls)} 格(全部同拍),"
          f" NeedsBelow 逐拍掉完终态相同={req_falls == load_falls}")
    # 9c 石悬臂上放沙(D8)
    w, arm = cantilever(8, 1, STONE)
    w.fill([(7, 6, 0)], SAND)
    r = ev(w).evaluate(sorted(set(trigger_of([(7, 6, 0)])) | set(arm)))
    remain = needsbelow_final(w, [])
    print(f"  9c 悬臂末端放沙(B8): {fmt(r)}(沙与整根悬臂同塌;"
          f" NeedsBelow 口径沙不掉={len(remain & {(7, 6, 0)}) == 1}→D8 差异,见 README)")

    # 10 弹坑地面
    print("--- 10 弹坑地面(REQ-009 推荐配置:power12 r8 dirtR2 stoneR6 k3 att1.0)---")
    for label, blasts in (("单坑", [(0, 0.5, 0)]), ("双坑(相距 9)", [(-5, 0.5, 0), (4, 0.5, 0)])):
        w = flat_ground(61, 61)
        changed = []
        resd = {AIR: 0, STONE: 6.0, DIRT: 2.0, BEDROCK: 1e12, SAND: 2.0}
        for b in blasts:
            field = td.BlastField(w.vox, w.x0, w.y0, w.z0, b, rmax=12.0, surface_y=-1)
            out = field.evaluate(resd, 12, 8, 3.0, 1.0, "ray")
            for (ix, iy, iz) in out["cells"]:
                changed.append((ix + w.x0, iy + w.y0, iz + w.z0))
        w.remove(changed)
        t0 = time.perf_counter()
        r = ev(w).evaluate(trigger_of(changed))
        print(f"  {label}: 破坏{len(changed)} 格;{fmt(r)};"
              f"求值耗时 {(time.perf_counter()-t0)*1e3:.2f} ms(含区域)")

    # 11 大薄板(四边支撑)+ 超预算
    w_slab, slab = edge_slab(15)
    run("11a 31×31 板四边支撑", w_slab, slab, note="内部 29×29=841 格悬挑:最坏耗时样本")
    r11 = saved_F["11a 31×31 板四边支撑"]
    w, slab = edge_slab(24)   # 内部 47×47=2209 > 2048
    r = ev(w).evaluate(slab)
    print(f"{'11b 49×49 板(2209>2048)':<24}{fmt(r):<18}{'':>5}  ← 超预算按稳定处理")

    # 12 退化对照:B=∞ 与 REQ-009 连通性真值
    print("--- 12 退化对照(B_石=B_土=∞,对照 REQ-009 全量连通分量真值)---")
    BINFP = {AIR: 0, STONE: INF_CAP, DIRT: INF_CAP, SAND: 0, BEDROCK: INF_CAP}
    for name, (tdw, removed, note) in tdc.build_structures().items():
        if not name[0].isdigit():
            continue
        w = VWorld(tdw.vox, tdw.x0, tdw.y0, tdw.z0)
        r = ev(w, BINFP).evaluate(trigger_of(removed))
        anch_t, float_t = tdc.truth(tdw, removed)
        truth = set().union(*float_t) if float_t else set()
        ok = r["f"] == truth and r["agree"]
        print(f"  {name:<22} F={len(r['f']):>3} 真值={len(truth):>3} 一致={'✓' if ok else '✗'}")

    # 13 触发顺序无关
    print("--- 13 触发顺序无关(门楣=土,打乱 20 次)---")
    w, removed = door(3, DIRT)
    trig = trigger_of(removed)
    rng = random.Random(42)
    base = None
    all_same = True
    for i in range(20):
        t2 = trig[:]
        rng.shuffle(t2)
        r = ev(w).evaluate(t2)
        fs = frozenset(r["f"])
        if base is None:
            base = fs
        elif fs != base:
            all_same = False
    print(f"  20 次打乱(F 大小 {len(base)}):全部相同 = {all_same}")

    # ---- B 扫描 ----
    print()
    print("=" * 72)
    print("B 扫描(设计意图:石悬臂 6 挂 12 断;土悬臂 1–2 挂;石门楣挂土门楣断;D8 加重)")
    print("=" * 72)
    print(f"{'B(stone)':>8} {'石悬臂max':>9} {'石门楣7':>8} {'T侧长max':>9} "
          f"{'拱顶/拱腿':>10} {'8+加重1/2/4':>12}")
    for bs in (4, 6, 8, 12, 16):
        B = {**B_REC, STONE: bs}
        hmax = 0
        for L in range(1, 21):
            w, trig = cantilever(L, 1, STONE)
            if not ev(w, B).evaluate(trig)["f"]:
                hmax = L
        w, removed = door(3, STONE)
        lint = "挂" if not ev(w, B).evaluate(trigger_of(removed))["f"] else "断"
        kmax = 0
        for k in range(3, 11):
            w, beam = tbeam(k)
            if not ev(w, B).evaluate(beam)["f"]:
                kmax = k
        w, key, leg = arch()
        w.remove([key])
        v_key = "挂" if not ev(w, B).evaluate(trigger_of([key]))["f"] else "断"
        w, key, leg = arch()
        w.remove(leg)
        v_leg = "挂" if not ev(w, B).evaluate(trigger_of(leg))["f"] else "断"
        w8 = []
        for n in (1, 2, 4):
            w, arm = cantilever(8, 1, STONE)
            weight = [(5 + i, 6, 0) for i in range(n)]
            w.fill(weight, STONE)
            w8.append("挂" if not ev(w, B).evaluate(
                sorted(set(trigger_of(weight)) | set(arm)))["f"] else "断")
        print(f"{bs:>8} {hmax:>9} {lint:>8} {kmax:>9} {v_key+'/'+v_leg:>10} {'/'.join(w8):>12}")
    print(f"{'B(dirt)':>8} {'土悬臂max':>9} {'土门楣7':>8}")
    for bd in (1, 2, 3, 4):
        B = {**B_REC, DIRT: bd}
        hmax = 0
        for L in range(1, 21):
            w, trig = cantilever(L, 1, DIRT)
            if not ev(w, B).evaluate(trig)["f"]:
                hmax = L
        w, removed = door(3, DIRT)
        lint = "挂" if not ev(w, B).evaluate(trigger_of(removed))["f"] else "断"
        print(f"{bd:>8} {hmax:>9} {lint:>8}")

    # ---- 切块(D3) ----
    print()
    print("=" * 72)
    print("切块(D3):对非空 F 先按粘结连通分组,再按世界对齐 S 格立方网格切")
    print("=" * 72)
    wbridge, remb = single_pillar_bridge(20, "end")
    Bev_bridge = ev(wbridge)
    f_bridge = Bev_bridge.evaluate(trigger_of(remb))["f"]
    w_plat = flat_ground()
    plat = [(x, 6, z) for x in range(-10, 10) for z in range(-10, 10)]
    legset = [(x, y, z) for (x, z) in ((-9, -9), (-9, 8), (8, -9), (8, 8)) for y in range(6)]
    w_plat.fill(plat)
    w_plat.fill(legset)
    Bev_plat = ev(w_plat)
    f_plat = Bev_plat.evaluate(sorted(plat))["f"]
    w_door2, _ = door(3, DIRT)
    Bev_door = ev(w_door2)
    for name, f, Bev in (("门楣F(6格土)", saved_F["1a 门楣=土(B3)"], Bev_door),
                         ("20桥面F", f_bridge, Bev_bridge),
                         ("平台F", f_plat, Bev_plat),
                         ("大薄板F", r11, ev(w_slab))):
        if not f:
            continue
        print(f"  {name}({len(f)} 格):")
        for S in (4, 6, 7):
            segs = cut_segments(f, Bev, S)
            sizes = sorted((len(s) for s in segs), reverse=True)
            rhos = [origin_and_rho(s)[1] for s in segs]
            bound = S / 2 * (3 ** 0.5) + 1.65
            ok = all(len(s) <= S ** 3 and rr <= bound for s, rr in zip(segs, rhos))
            print(f"    S={S}: {len(segs)} 段,格数 max/min {sizes[0]}/{sizes[-1]},"
                  f" ρmax={max(rhos):.2f}(上界 {bound:.2f}) {'✓' if ok else '✗'}")

    # ---- 跨分片(D7) ----
    print()
    print("=" * 72)
    print("跨分片(D7):x=0 切两片,本体区+16m 扩张,写入晚一拍")
    print("=" * 72)
    cross_shard()


def cross_shard():
    B = B_REC
    MARGIN = 16
    own_of = lambda c: "A" if c[0] < 0 else "B"

    def simulate(name, build, removals_of):
        """build() -> (w, 说明);removals_of: {shard: [(x,y,z),...]} tick0 各自移除。"""
        w = build()[0]
        # 单世界真值
        w_single = VWorld(w.vox.copy(), w.x0, w.y0, w.z0)
        rem_all = [c for s in "AB" for c in removals_of[s]]
        w_single.remove(rem_all)
        single = ev(w_single, B).evaluate(trigger_of(rem_all))["f"]
        views = {s: VWorld(w.vox.copy(), w.x0, w.y0, w.z0) for s in "AB"}
        conv_log = {"A": {}, "B": {}}
        all_conv = {}
        pending = {s: set() for s in "AB"}
        for tick in range(8):
            any_act = False
            for s in "AB":
                o = "B" if s == "A" else "A"
                new = set()
                if tick == 0:
                    new |= set(removals_of[s])
                else:
                    new |= set(removals_of[o]) if tick == 1 else set()
                    new |= conv_log[o].get(tick - 1, set())
                new -= pending[s]  # 已应用过
                if not new:
                    continue
                any_act = True
                for c in new:
                    views[s].set_air(c)
                pending[s] |= new
                seeds = trigger_of(sorted(new))
                r = ev(views[s], B).evaluate(seeds)
                conv = {c for c in r["f"] if own_of(c) == s}
                for c in conv:
                    views[s].set_air(c)
                conv_log[s][tick] = conv
                for c in conv:
                    assert c not in all_conv, f"重复转换 {c}"
                    all_conv[c] = s
                print(f"  tick{tick} 分片{s}: 新见写入{len(new)},F={len(r['f'])},"
                      f"转换本体 {len(conv)} 格")
            if not any_act and tick > 0:
                break
        union = set(all_conv)
        under = single - union
        over = union - single
        print(f"  [{name}] 单世界 F={len(single)},各片转换并集={len(union)},"
              f"少塌={len(under)} 多塌={len(over)}")
        if under:
            ex = sorted(under)[:8]
            print(f"    少塌例(永悬空):{ex}")
        if over:
            ex = sorted(over)[:8]
            print(f"    多塌例:{ex}")
        return under, over

    print("门框(柱x=-3∈A/x=3∈B,土门楣 x=-3..3):")
    def build_door():
        w, removed = door(3, DIRT)
        return w, removed
    simulate("门框土门楣", build_door, {"A": [(-3, y, 0) for y in range(5)], "B": []})

    print("单柱桥(桥面 x=-9..10,柱 x=0∈B):")
    def build_bridge():
        w = flat_ground()
        w.fill([(0, y, 0) for y in range(5)])
        w.fill([(x, 5, 0) for x in range(-9, 11)])
        return w, None
    simulate("单柱桥", build_bridge, {"A": [], "B": [(0, y, 0) for y in range(5)]})

    print("平台 20×20(4 柱各 2 根属 A/B,各自同拍移走):")
    def build_plat():
        w = flat_ground()
        plat = [(x, 6, z) for x in range(-10, 10) for z in range(-10, 10)]
        legs = [(x, y, z) for (x, z) in ((-9, -9), (-9, 8), (8, -9), (8, 8)) for y in range(6)]
        w.fill(plat)
        w.fill(legs)
        return w, legs
    legsA = [(x, y, z) for (x, z) in ((-9, -9), (-9, 8)) for y in range(6)]
    legsB = [(x, y, z) for (x, z) in ((8, -9), (8, 8)) for y in range(6)]
    simulate("平台", build_plat, {"A": legsA, "B": legsB})


if __name__ == "__main__":
    main()
