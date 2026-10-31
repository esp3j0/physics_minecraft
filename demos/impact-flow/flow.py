"""flow.py — REQ-010 单元 F00 脚本 1:撞击冲量流(§5.7.1,纯体素)。

按 §5.7.1 实现:事件(供给 s_c、冲量矢量 J、d̂)、BFS 区域(层序、层内 BlockPos
升序、上限)、有向容量网络(压向判定 u·d̂ ≥ cos45° 取 I_c = min(I_c,a,I_c,b),
否则剪切粘结 I = min(I_a,I_b))、五类汇点(Anchor、WORLD_Y_MIN 向下、未装载、
本体区外、BFS 截断)、0.001 N·s 整数化(供给向下、容量向上)、最大流 C、
残量网络源可达集 S(最小源侧)、预筛 s < I_min = 4。

Edmonds–Karp 与 Dinic 各求一遍,逐工况核对 C 与 S 完全相同;每个工况把供给
随机拆分到命中格+邻格 20 次(模拟接触点顺序打乱),断开与否与算法一致性不变。

供给口径:工况 6 用 m·|v| = 0.729 × √(40²+7²) = 29.77 N·s(req010-impact-corner
的投射物),其余工况打印"断开阈值 s*"(恰断开所需的最小供给)与最小割容量 C,
换算 0.729 kg 单格速度 = s*/0.729、4.374 kg 六格复合体速度 = s*/4.374
(REQ-009 spike 实测 rapier 单步吸收峰值 ≈ m·v,momentum.py 第 1 条复核)。

运行:.venv/Scripts/python.exe demos/impact-flow/flow.py   (~1 分钟)
"""
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (AIR, BEDROCK, CELL_MASS, DIRT, DIRS6, I_MIN, I_OF, IC_OF,
                    ImpulseFlow, INF_CAP, SAND, STONE, bond_crush, bond_impact,
                    flat_ground, profile_ascii, q_supply)


# ---------------------------------------------------------------- 工况构造

def case1_corner_block():
    """工况 1:墙角伸出的石格(只剩 1 根粘结),横撞 d = (-1,0,0)。
    石墙 x=-2..-1、y=0..2、z=-2..2 立在 bedrock 上(墙脚下 dirt 换 bedrock,
    隔离出"那 1 根粘结"的问题),伸出格 (0,1,0)。命中格 = 伸出格本身。"""
    w = flat_ground()
    w.fill([(x, -y, z) for x in (-2, -1) for y in (1, 2, 3)
            for z in (-2, -1, 0, 1, 2)], BEDROCK)
    w.fill([(x, y, z) for x in (-2, -1) for y in (0, 1, 2)
            for z in (-2, -1, 0, 1, 2)], STONE)
    w.fill([(0, 1, 0)], STONE)
    return w, (0, 1, 0), (-1.0, 0.0, 0.0)


def case2_wall(width=7, height=5, base=BEDROCK):
    """工况 2:1 格厚石墙 x=-1、y=0..height-1、z=-(width//2)..width//2,
    正面中部横撞 d = (-1,0,0),命中格 = 墙面格 (-1,2,0)。
    base=BEDROCK:墙脚下(y=-1..-3、墙投影内)换 bedrock(立在基岩上);
    base=DIRT:保持超平坦地面(墙直接砌在土上,墙脚是石-土界面)。"""
    w = flat_ground()
    if base == BEDROCK:
        w.fill([(-1, -y, z) for y in (1, 2, 3)
                for z in range(-(width // 2), width // 2 + 1)], BEDROCK)
    for y in range(height):
        w.fill([(-1, y, z) for z in range(-(width // 2), width // 2 + 1)], STONE)
    return w, (-1, 2, 0), (-1.0, 0.0, 0.0)


def case3_thick_wall():
    """工况 3:3 格厚石墙 x=-3..-1、y=0..6、z=-4..4,立在 bedrock,正面中部横撞。
    需求期望 S 是非单格的"冲剪塞"——实测核对。"""
    w = flat_ground()
    w.fill([(x, -y, z) for x in (-3, -2, -1) for y in (1, 2, 3)
            for z in range(-4, 5)], BEDROCK)
    w.fill([(x, y, z) for x in (-3, -2, -1) for y in range(7)
            for z in range(-4, 5)], STONE)
    return w, (-1, 3, 0), (-1.0, 0.0, 0.0)


def case4_bridge(width=1):
    """工况 4:架空土桥:桥面 y=3、x=0..6、z=-(width//2)..width//2(dirt),
    两端石柱 x=-1、7、y=0..3(柱脚下换 bedrock);桥面下方是空。
    中部格 (3,3,0) 被从上砸 d = (0,-1,0)。"""
    w = flat_ground()
    for x in (-1, 7):
        w.fill([(x, -y, 0) for y in (1, 2, 3)], BEDROCK)
        w.fill([(x, y, 0) for y in range(4)], STONE)
    for x in range(7):
        w.fill([(x, 3, dz) for dz in range(-(width // 2), width // 2 + 1)], DIRT)
    return w, (3, 3, 0), (0.0, -1.0, 0.0)


def case5_flat_ground():
    """工况 5:平整土地被从上砸:命中地表 dirt (0,-1,0),d = (0,-1,0)。"""
    w = flat_ground()
    return w, (0, -1, 0), (0.0, -1.0, 0.0)


def case6_impact_corner():
    """工况 6:req010-impact-corner 几何(toml 平移 Δy=+59、Δz=-20):
    石柱 (8,y,0) y=-1..0,门楣 (5..8,1,0)(左端悬空),地面 dirt y=-1..-3、
    bedrock y=-4。投射物 velocity=[-40,-7,0]、m=0.729 →
    d̂ = normalize(-40,-7,0) = (-0.9851,-0.1724,0),s = m·|v| = 29.77 N·s。
    供给简化:全部记在接缝上格 (8,1,0)(momentum.py 第 4 条用 rapier 实测分配)。"""
    w = flat_ground()
    w.fill([(8, -1, 0), (8, 0, 0)], STONE)
    w.fill([(x, 1, 0) for x in (5, 6, 7, 8)], STONE)
    v = math.hypot(40.0, 7.0)
    dhat = (-40.0 / v, -7.0 / v, 0.0)
    return w, (8, 1, 0), dhat


CASE6_SUPPLY = CELL_MASS * math.hypot(40.0, 7.0)   # = 29.77 N·s


def case7_pillar(mode):
    """工况 7:石柱 1×1×4 (0,0..3,0) 立在土上,侧撞 d=(-1,0,0)。"""
    w = flat_ground()
    w.fill([(0, y, 0) for y in range(4)], STONE)
    hit = (0, 1, 0) if mode == "mid" else (0, 3, 0)
    return w, hit, (-1.0, 0.0, 0.0)


def case8_sandpile():
    """工况 8:沙堆 y=0 一层 3×5 + y=-1 内芯 2×3,d=(-1,0,0),命中 (2,0,0)。"""
    w = flat_ground()
    w.fill([(x, 0, z) for x in (0, 1, 2) for z in (-2, -1, 0, 1, 2)], SAND)
    w.fill([(x, -1, z) for x in (0, 1) for z in (-1, 0, 1)], SAND)
    return w, (2, 0, 0), (-1.0, 0.0, 0.0)


CASES = [
    ("1 corner-block 石伸出格 横撞", case1_corner_block, 1.0),
    ("2a wall1-stone-on-bedrock w7h5 横撞", lambda: case2_wall(base=BEDROCK), 1.0),
    ("2b wall1-stone-on-dirt w7h5 横撞", lambda: case2_wall(base=DIRT), 1.0),
    ("2c wall1-stone-on-bedrock w15 横撞", lambda: case2_wall(width=15, base=BEDROCK), 1.0),
    ("3 wall3-stone-on-bedrock w9h7 横撞", case3_thick_wall, 1.0),
    ("4a bridge1-dirt span7 上砸", lambda: case4_bridge(1), 1.0),
    ("4b bridge3-dirt span7 上砸", lambda: case4_bridge(3), 1.0),
    ("5 flat-ground-dirt 上砸", case5_flat_ground, 1.0),
    ("6 impact-corner 斜撞 s=29.77", case6_impact_corner, CASE6_SUPPLY),
    ("7a pillar1x1x4-on-dirt 中部侧撞", lambda: case7_pillar("mid"), 1.0),
    ("7b pillar1x1x4-on-dirt 柱顶侧撞", lambda: case7_pillar("top"), 1.0),
    ("8 sandpile 横撞", case8_sandpile, 1.0),
]

CASE_BY_PREFIX = {name.split()[0]: (name, b, s) for name, b, s in CASES}


# ---------------------------------------------------------------- 求值与打印

def evaluate_print(name, builder, s_units, max_cells=1024):
    w, hit, J = builder()
    fl = ImpulseFlow(w, max_cells=max_cells)
    res = fl.evaluate({hit: s_units}, J)
    s_q = q_supply(s_units)
    print(f"\n=== {name} ===")
    print(f"hit={hit} d̂={tuple(round(x, 4) for x in res['dhat'])} "
          f"s={s_units:.3f}(量化 {s_q}) visited={res['visited']} "
          f"truncated={res['truncated']}")
    if res["prefiltered"]:
        print(f"预筛:s < I_min={I_MIN} → 不建网,无断开")
        return res
    verdict = "断开" if res["C"] < s_q else "扛住"
    print(f"C={res['C']}({'%.3f N·s' % (res['C'] / 1000)}) "
          f"{'<' if res['C'] < s_q else '≥'} s → {verdict}   "
          f"S格数={len(res['S'])} cut_bonds={res['cut_bonds']} "
          f"ek/dinic同={res['algos_agree']} "
          f"t_ek={res['t_flows']['ek'] * 1000:.2f}ms "
          f"t_dinic={res['t_flows']['dinic'] * 1000:.2f}ms")
    if res["S"]:
        comps = res["components"]
        print(f"S 连通分量 {len(comps)} 个,尺寸 {[len(c) for c in comps]}")
        if all(c[2] == 0 for c in res["S"]):
            print("S 剖面(x–y @z=0):")
            print(profile_ascii(res["S"], plane="xy", z=0))
        else:
            print("S 剖面(x–z):")
            print(profile_ascii(res["S"], plane="xz"))
    return res


def broke_at(fl, hit, J, s_units):
    """s_units 供给下是否断开。预筛(s < I_min)= 任何割都 > s = 扛住。"""
    res = fl.evaluate({hit: s_units}, J)
    if res["prefiltered"]:
        return False, res
    return (res["C"] < q_supply(s_units)), res


def break_threshold(builder, max_cells=1024):
    """二分恰断开供给。返回 (s*, C, v_single, v_comp6):
    s* = 最小断开供给样本(N·s,精度 0.001);C = s* 下的最小割(量化单位)。"""
    w, hit, J = builder()
    fl = ImpulseFlow(w, max_cells=max_cells)
    lo, hi = 0.0005, 4.0
    b_hi, _ = broke_at(fl, hit, J, hi)
    while not b_hi:
        hi *= 2.0
        if hi > 1e7:
            return float("inf"), 0, float("inf"), float("inf")
        b_hi, _ = broke_at(fl, hit, J, hi)
    for _ in range(40):
        mid = (lo + hi) / 2.0
        b, _ = broke_at(fl, hit, J, mid)
        if b:
            hi = mid
        else:
            lo = mid
    _, res = broke_at(fl, hit, J, hi)
    return hi, res["C"], hi / CELL_MASS, hi / 4.374


# ---------------------------------------------------------------- 打乱验证

def shuffle_check(builder, s_units, trials=20, seed=7):
    """接触点顺序打乱:把同一总供给随机拆到命中格+实心邻格之间(顺序即
    归格的随机化),验证断开与否不变、ek/dinic 一致。"""
    w, hit, J = builder()
    fl = ImpulseFlow(w, max_cells=1024)
    rng = random.Random(seed)
    cand = [c for c in [tuple(hit[i] + d[i] for i in range(3)) for d in DIRS6]
            if w.get(c) not in (AIR, None)]
    base_broke, _ = broke_at(fl, hit, J, s_units)
    ok_break, ok_agree, n_eff = 0, 0, 0
    for _ in range(trials):
        k = rng.randint(1, len(cand))
        cells = rng.sample(cand, k)
        wts = [rng.random() + 0.1 for _ in range(k)]
        tot = sum(wts)
        sup = {c: s_units * x / tot for c, x in zip(cells, wts)}
        res = fl.evaluate(sup, J)
        brk = res["prefiltered"] or (res["C"] is not None
                                     and res["C"] < q_supply(s_units))
        if brk == base_broke:
            ok_break += 1
        if res["algos_agree"]:
            ok_agree += 1
        n_eff += 1
    return ok_break, ok_agree, n_eff


# ---------------------------------------------------------------- 扫描

def ic_scan(prefixes, multipliers=(2.0, 4.0, 8.0, None)):
    """I_c 扫描:I_c = m·I(m ∈ {2,4,8},None = ∞),对指定工况求 s* 与 C。"""
    saved = dict(IC_OF)
    rows = {}
    for m in multipliers:
        for name, builder, _ in CASES:
            if name.split()[0] not in prefixes:
                continue
            mats = (STONE, DIRT, SAND)
            if m is None:
                for mat in mats:
                    IC_OF[mat] = INF_CAP
            else:
                for mat in mats:
                    IC_OF[mat] = m * I_OF[mat]
            s_star, C, v1, v6 = break_threshold(builder)
            rows.setdefault(name.split()[0], []).append((m, s_star, C, v1, v6))
    IC_OF.clear()
    IC_OF.update(saved)
    return rows


def region_scan(prefixes, s_units, max_cells_values=(256, 512, 1024, 2048)):
    """区域上限扫描:C、S、访问格数、truncated、耗时是否随上限变化。"""
    rows = {}
    for name, builder, s0 in CASES:
        if name.split()[0] not in prefixes:
            continue
        per = []
        for mc in max_cells_values:
            w, hit, J = builder()
            fl = ImpulseFlow(w, max_cells=mc)
            t0 = time.perf_counter()
            res = fl.evaluate({hit: s_units}, J)
            dt = time.perf_counter() - t0
            per.append((mc, res["C"], len(res["S"]), res["visited"],
                        res["truncated"], res["prefiltered"], dt))
        rows[name.split()[0]] = per
    return rows


def prefilter_stats():
    """常见轻碰撞的 s = m·v 分布 vs I_min = 4。"""
    g = 9.81
    rows = []
    for h in (1.0, 2.0, 5.0):
        v = math.sqrt(2 * g * h)
        rows.append((f"碎块落地 {h:.0f} m", v, CELL_MASS * v))
    for v in (1.0, 2.0, 5.0):
        rows.append((f"横撞 {v:.0f} m/s", v, CELL_MASS * v))
    return rows


# ---------------------------------------------------------------- 主流程

def main():
    print("=" * 78)
    print("flow.py — §5.7.1 冲量最大流纯体素验证"
          "(stone I=18/Ic=72, dirt 7/28, sand 4/16;量化 0.001 N·s)")
    print("=" * 78)

    print("\n--- 0. 预筛统计(s = m·v,I_min = 4 = sand 的 min(I,I_c);"
          "s < 4 不建网直接跳过)---")
    print("场景,速度 m/s,s (N·s),建网?")
    for scen, v, s in prefilter_stats():
        print(f"{scen},{v:.2f},{s:.3f},{'是' if s >= I_MIN else '否(跳过)'}")

    print("\n--- 1–8. 逐工况结构与最小割 S(结构打印用供给 60 N·s,足够断开"
          "全部工况;工况 6 用实测供给 29.77)---")
    for name, builder, s0 in CASES:
        evaluate_print(name, builder, max(s0, 60.0 if s0 == 1.0 else s0))

    print("\n" + "=" * 78)
    print("9. I_c 扫描:I_c = {2,4,8}×I 与 ∞,断开阈值 s*(恰断开最小供给)与割 C")
    print("工况,I_c,s* (N·s),C (N·s),0.729kg 速度,4.374kg 复合体速度")
    rows = ic_scan({"1", "2a", "3", "4a", "5", "7a"})
    for pre, per in rows.items():
        for m, s_star, C, v1, v6 in per:
            ms = "∞" if m is None else f"{m:g}·I"
            print(f"{pre},I_c={ms},{s_star:.3f},{C / 1000:.3f},{v1:.1f},{v6:.1f}")

    print("\n" + "=" * 78)
    print("10. 区域上限扫描 s=60(C、|S|、visited、truncated、单次求值耗时;"
          "60 N·s 让 2b/5/7a 全部断开)")
    rscan = region_scan({"2b", "5", "7a"}, 60.0)
    for pre, per in rscan.items():
        print(pre)
        for mc, C, nS, vis, trunc, pre_f, dt in per:
            print(f"  max_cells={mc:5d}: C={C} |S|={nS} visited={vis} "
                  f"truncated={trunc} prefiltered={pre_f} t={dt * 1000:.2f}ms")

    print("\n" + "=" * 78)
    print("接触点顺序打乱 20 次(供给 = 各工况断开阈值的 2 倍,必然断开;"
          "随机拆到命中格+邻格;断开不变 / ek=dinic)")
    for name, builder, s0 in CASES:
        s_star = break_threshold(builder)[0]
        s_shuffle = 2.0 * s_star if s_star != float("inf") else 60.0
        okb, oka, n = shuffle_check(builder, s_shuffle)
        print(f"{name}: break 不变 {okb}/{n},algos_agree {oka}/{n}")

    print("\n" + "=" * 78)
    print("断开阈值表(默认 I_c=4·I;二分精度 0.001 N·s)")
    print("工况,s* (N·s),最小割 C (N·s),0.729kg 速度,4.374kg 六格复合体速度")
    for name, builder, s0 in CASES:
        s_star, C, v1, v6 = break_threshold(builder)
        print(f"{name},{s_star:.3f},{C / 1000:.3f},{v1:.1f},{v6:.1f}")

    print("\n" + "=" * 78)
    print("耗时上界:1600 格石板(40×40,BFS 1024 截断处当汇点),供给 50 N·s 上砸")
    big = flat_ground(81, 81)
    big.fill([(x, 5, z) for x in range(40) for z in range(40)], STONE)
    flb = ImpulseFlow(big, max_cells=1024)
    res = flb.evaluate({(20, 5, 20): 50.0}, (0.0, -1.0, 0.0))
    print(f"visited={res['visited']} truncated={res['truncated']} C={res['C']} "
          f"|S|={len(res['S'])} "
          f"t_ek={res['t_flows']['ek'] * 1000:.2f}ms "
          f"t_dinic={res['t_flows']['dinic'] * 1000:.2f}ms")
    print("附:1024 格恰好封闭的悬空板(32×32,BFS 恰好吃满、无任何汇边)"
          "是极端案例:C=0 → 整板断下,语义自洽。")


if __name__ == "__main__":
    main()
