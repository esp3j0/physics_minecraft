"""flow.py — G00 脚本 1:§5.8.1 惯性汇点冲量最大流(纯函数)。

按 §5.8.1 逐条实现的事件求值在 common.py(compound_flow / rigid_response /
d4_rule)。本脚本覆盖任务书 8 类工况:
- 每个工况打印:断开阈值 s*(二分,量化精度 0.001 N·s)、S 的格数与剖面 ASCII、
  被切断粘结数,以及同工况 D4 现行规则(§7.1)的断键阈值对照;
- 两种最大流算法(EK / Dinic)逐工况核对 C 与 S 完全相同;
- 每个工况把接触点顺序随机打乱 20 次,supplies 的合计不变(浮点求和顺序敏感,
  这里对供给做排序后求和以保持确定性——§5.8.1 整数化前 events 供给按格排序),
  S 不变;
- 工况 8:对全部工况随机扫 s,断言 S ≠ 全体(R 非空)。

运行:.venv/Scripts/python.exe demos/compound-flow/flow.py
"""
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (BEDROCK, CELL_MASS, COS45, DIRT, I_MIN, STONE, SAND,
                    beam_cells, cell_center, compound_flow, cube_cells,
                    d4_rule, lshape_cells, pillar_cells, plate_cells,
                    profile_ascii, seven_cells, tbeam_cells)

Q = 1000  # 0.001 N·s


def eval_case(name, cells, mats, supply_fn, J_fn, p_fn, verbose=True):
    """一个工况:二分断开阈值 s*(量化域上),打印结果。

    supply_fn(s) -> {cell: s_c};J_fn(s) -> 矢量(默认 = (0,-s,0));
    p_fn(s) -> 作用点。断开判据:res['S'] 非空(= C < s,§5.8.1 判定)。
    返回 (s_star, info_at_s_star, d4_star)。
    """
    lo, hi = 0.001, 100000.0

    def broken(s):
        supply = supply_fn(s)
        J = np.asarray(J_fn(s), float)
        p = np.asarray(p_fn(s), float)
        res = compound_flow(cells, mats, supply, J, p)
        return bool(res["S"]), res, supply, J, p

    if not broken(hi)[0]:
        print(f"{name}: s ≤ {hi:.0f} N·s 都不断")
        return None
    while hi - lo > 1.0 / Q:
        mid = 0.5 * (lo + hi)
        if broken(mid)[0]:
            hi = mid
        else:
            lo = mid
    s_star = hi
    br, res, supply, J, p = broken(s_star)
    # D4 对照:同一 s* 下 §7.1 的断键(阈值 = 断键的最小 s,同样二分)
    def d4_broken(s):
        pts = d4_points(s)
        return bool(d4_rule(cells, mats, supply_fn(s), J_fn(s), pts))

    def d4_points(s):
        # D4 的接触点集与 §5.8 相同(接触格、作用点、法向):从 J 与 p 反推——
        # 法向 = -d̂(指向本体),接触格 = supply 的键
        supply = supply_fn(s)
        J = np.asarray(J_fn(s), float)
        p = np.asarray(p_fn(s), float)
        n = len(supply)
        Jm = float(np.linalg.norm(J))
        nrm = tuple((-J / Jm)) if Jm > 1e-12 else (0.0, 1.0, 0.0)
        return [(c, s / n, tuple(p), nrm) for c in supply]

    d4_lo, d4_hi = 0.001, 100000.0
    d4_any = d4_broken(d4_hi)
    d4_star = None
    if d4_any:
        while d4_hi - d4_lo > 1.0 / Q:
            mid = 0.5 * (d4_lo + d4_hi)
            if d4_broken(mid):
                d4_hi = mid
            else:
                d4_lo = mid
        d4_star = d4_hi
    if verbose:
        print(f"\n== {name} ==")
        print(f"  s* = {s_star:.3f} N·s | S = {len(res['S'])} 格 | "
              f"被切断粘结 {res['cut_bonds']} 根")
        print(profile_ascii(res["S"], cells))
        print(f"  D4 现行(§7.1)对照阈值: "
              + (f"{d4_star:.3f} N·s" if d4_star else f"≤ {d4_hi:.0f} N·s 不断"))
    return s_star, res, d4_star


def shuffle_check(name, cells, mats, supply_fn, J_fn, p_fn, s, trials=20, seed=7):
    """接触点顺序打乱 trials 次:供给按打乱序累加(浮点),S 应不变。"""
    rng = random.Random(seed)
    base = compound_flow(cells, mats, supply_fn(s), np.asarray(J_fn(s), float),
                         np.asarray(p_fn(s), float))["S"]
    flips = 0
    for t in range(trials):
        items = list(supply_fn(s).items())
        rng.shuffle(items)
        acc = {}
        for c, v in items:
            acc[c] = acc.get(c, 0.0) + v
        S = compound_flow(cells, mats, acc, np.asarray(J_fn(s), float),
                          np.asarray(p_fn(s), float))["S"]
        if S != base:
            flips += 1
    print(f"  打乱 20 次 [{name} @ s={s:.1f}]: S 翻转 {flips} 次")
    return flips


def main():
    print("=" * 72)
    print("§5.8.1 惯性汇点冲量最大流 — 8 类工况(纯函数)")
    print("=" * 72)
    results = {}   # 工况名 -> (cells, mats, supply_fn, J_fn, p_fn, s_star)

    def reg(name, cells, mats, supply_fn, J_fn=None, p_fn=None):
        J_fn = J_fn or (lambda s: (0.0, -s, 0.0))
        p_fn = p_fn or (lambda s: tuple(np.mean([cell_center(c) for c in cells], axis=0)))
        out = eval_case(name, cells, mats, supply_fn, J_fn, p_fn)
        results[name] = (cells, mats, supply_fn, J_fn, p_fn, out)
        return out

    stone = lambda cs: {c: STONE for c in cs}
    dirt = lambda cs: {c: DIRT for c in cs}

    # ---- 工况 1:6 格条平落(全 6 格接触均摊)——验证任何 s 都不断
    def supply_even(s):
        return {c: s / 6 for c in beam_cells()}
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"1 六格条平落({mn})", beam_cells(), mat(beam_cells()), supply_even)

    # ---- 工况 2:6 格条端角先着地(冲量竖直向上、作用于端格外下角)
    def p_end_corner(s):
        return tuple(cell_center(beam_cells()[0]) + np.array((-0.5, -0.5, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"2 六格条端角先着地({mn})", beam_cells(), mat(beam_cells()),
            lambda s: {beam_cells()[0]: s}, None, p_end_corner)

    # ---- 工况 3:6 格柱竖直落地(作用于底格底面中心)
    def p_pillar_bottom(s):
        return tuple(cell_center(pillar_cells()[0]) + np.array((0.0, -0.5, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"3 六格柱竖直落地({mn})", pillar_cells(), mat(pillar_cells()),
            lambda s: {pillar_cells()[0]: s}, None, p_pillar_bottom)

    # ---- 工况 4:单格方块横撞静止 6 格条(供给 = 约化质量 × u)
    # 约化质量:接触点在端格中心时,整条按刚体响应的 m_eff(转动含偏心);
    # 供给 s = m_eff · u → 反过来"断开阈值速度" = s* / m_eff。这里直接以 s 为
    # 自变量,速度换算在打印时给(m_eff 数值见工况内)。
    cells_b = beam_cells()
    m_b = len(cells_b) * CELL_MASS
    for hit_i, tag in ((0, "端格"), (1, "第2格"), (2, "第3格")):
        hit = cells_b[hit_i]
        def p_side(s, hit=hit):
            return tuple(cell_center(hit) + np.array((-0.5, 0.0, 0.0)))
        # m_eff 沿 x:1/(1/M + (r×d̂)I⁻¹(r×d̂)),r = p − 质心
        M, com, dhat, dv, _ = __import__("common").rigid_response(
            cells_b, stone(cells_b), p_side(1.0), np.array((1.0, 0, 0)))
        r_ = np.array(p_side(1.0)) - com
        invI = None  # 用 Δv 直接反推:m_eff = 1/Δv_com·d̂?更直接:Δv_i 已含
        # m_eff·u = s(供给定义)→ 速度阈值 = s*/m_eff,m_eff 用解析式算:
        # 完整惯量(含惯性积;本工况轴对齐条,惯性积为 0,用数值验证过)
        I = np.zeros((3, 3))
        for c in cells_b:
            d = cell_center(c) - com
            sself = CELL_MASS * 0.81 / 6.0
            I += np.diag([sself] * 3) + CELL_MASS * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        rx = np.cross(r_, np.array((1.0, 0, 0)))
        m_eff = 1.0 / (1.0 / M + float(np.dot(rx, np.linalg.solve(I, rx))))
        for mat, mn in ((stone, "石"), (dirt, "土")):
            out = reg(f"4 单格横撞{tag}({mn})", cells_b, mat(cells_b),
                      lambda s: {hit: s}, lambda s: (s, 0.0, 0.0), p_side)
            if out and out[0]:
                # rapier 实测(momentum 工况 3 诊断):单拍 s = μ·u,μ = mM/(m+M)
                # = 0.625(投射物 1 格);全额口径 s*=m_eff·u 仅作参照。
                mu = CELL_MASS * m_b / (CELL_MASS + m_b)
                print(f"    供给换算:s*={out[0]:.3f} N·s;m_eff={m_eff:.3f} kg → "
                      f"全额口径 ≈ {out[0]/m_eff:.1f} m/s;rapier 口径(s=μ·u,"
                      f"μ={mu:.3f})≈ {out[0]/mu:.1f} m/s(单拍)")

    # ---- 工况 5:3×3×1 板:平落;一角先着地;单格横撞边格中点与角格
    pc = plate_cells()
    def supply_plate_even(s):
        return {c: s / 9 for c in pc}
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"5 板平落({mn})", pc, mat(pc), supply_plate_even)
    def p_plate_corner(s):
        return tuple(cell_center(pc[0]) + np.array((-0.5, -0.5, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"5 板一角先着地({mn})", pc, mat(pc),
            lambda s: {pc[0]: s}, None, p_plate_corner)
    for hit, tag in ((pc[1], "边格中点"), (pc[0], "角格")):
        def p_hit(s, hit=hit):
            return tuple(cell_center(hit) + np.array((-0.5, 0.0, 0.0)))
        for mat, mn in ((stone, "石"), (dirt, "土")):
            reg(f"5 板被横撞{tag}({mn})", pc, mat(pc),
                lambda s: {hit: s}, lambda s: (s, 0.0, 0.0), p_hit)

    # ---- 工况 6:L 形(4 格)与 7 字(6 格)被打在拐角格
    lc = lshape_cells()
    def p_ljoint(s):
        return tuple(cell_center(lc[1]) + np.array((0.5, 0.0, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"6 L形拐角被撞({mn})", lc, mat(lc),
            lambda s: {lc[1]: s}, lambda s: (s, 0.0, 0.0), p_ljoint)
    sc = seven_cells()
    def p_seven_joint(s):
        return tuple(cell_center(sc[3]) + np.array((0.5, 0.0, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"6 七字拐角被撞({mn})", sc, mat(sc),
            lambda s: {sc[3]: s}, lambda s: (s, 0.0, 0.0), p_seven_joint)

    # ---- 工况 7:2×2×2 立方:平落、一棱先着地
    cc = cube_cells()
    def supply_cube_even(s):
        return {c: s / 8 for c in cc}
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"7 立方平落({mn})", cc, mat(cc), supply_cube_even)
    def p_cube_edge(s):
        return tuple(cell_center(cc[0]) + np.array((-0.5, -0.5, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"7 立方一棱先着地({mn})", cc, mat(cc),
            lambda s: {cc[0]: s / 2, cc[1]: s / 2} if False else {cc[0]: s},
            None, p_cube_edge)

    # ---- 算法一致 + 打乱
    print("\n" + "=" * 72)
    print("两种最大流算法一致性与接触点顺序打乱(20 次)")
    print("=" * 72)
    bad = 0
    for name, (cells, mats, supply_fn, J_fn, p_fn, out) in results.items():
        if out is None:
            continue
        _, res, _ = out
        if not res["algos_agree"]:
            print(f"  [不一致] {name}")
            bad += 1
        s_probe = out[0] * 1.5
        rng = __import__("random").Random(7)
        base = compound_flow(cells, mats, supply_fn(s_probe),
                             np.asarray(J_fn(s_probe), float),
                             np.asarray(p_fn(s_probe), float))["S"]
        flips = 0
        for _ in range(20):
            items = list(supply_fn(s_probe).items())
            rng.shuffle(items)
            acc = {}
            for c, v in items:
                acc[c] = acc.get(c, 0.0) + v
            S = compound_flow(cells, mats, acc, np.asarray(J_fn(s_probe), float),
                              np.asarray(p_fn(s_probe), float))["S"]
            if S != base:
                flips += 1
        if flips:
            print(f"  [打乱翻转 {flips}] {name}")
            bad += 1
    print(f"  算法不一致工况数 = {bad  if bad else 0};打乱翻转工况数 = {bad}")

    # ---- 工况 8:S ≠ 全体(R 非空)
    print("\n" + "=" * 72)
    print("8 随机扫 s:S 永不为全体")
    print("=" * 72)
    total, viol = 0, 0
    rng = __import__("random").Random(11)
    for name, (cells, mats, supply_fn, J_fn, p_fn, out) in results.items():
        for _ in range(40):
            s = 10 ** rng.uniform(0.3, 4.2)
            res = compound_flow(cells, mats, supply_fn(s),
                                np.asarray(J_fn(s), float),
                                np.asarray(p_fn(s), float))
            if res["prefiltered"] or res["S"] is None or not res["S"]:
                continue
            total += 1
            if res["S"] == set(cells):
                viol += 1
                print(f"  [违反] {name} @ s={s:.2f}: S = 全体")
    print(f"  有效样本 {total},S = 全体的次数 = {viol}")


if __name__ == "__main__":
    main()
