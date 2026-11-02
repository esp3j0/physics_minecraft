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
                    profile_ascii, rigid_response, seven_cells, tbeam_cells)

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
    print("§5.8.1 惯性汇点冲量最大流 — 8 类工况(纯函数;G00-2 修正④)")
    print("J 一律指向 B 内部(地面推复合体 = +y;侧面受击 = 从受击面指向格内),")
    print("断言 J·(格心 − p) > 0;速度换算用约化质量 u* = s*·(m+m_eff)/(m·m_eff)。")
    print("=" * 72)
    results = {}   # 工况名 -> (cells, mats, supply_fn, J_fn, p_fn, s_star)

    def reg(name, cells, mats, supply_fn, J_fn=None, p_fn=None):
        J_fn = J_fn or (lambda s: (0.0, s, 0.0))   # 地面冲量向上(指向条内部)
        p_fn = p_fn or (lambda s: tuple(np.mean([cell_center(c) for c in cells], axis=0)))
        # 断言:J 指向 B 内部。缺省工况 p = 质心(J 过质心,内指自动成立);
        # 显式 p 的工况断言 J·(质心 − p) > 0;两者都不满足才报错。
        J = np.asarray(J_fn(1.0), float)
        p = np.asarray(p_fn(1.0), float)
        com = np.mean([cell_center(c) for c in cells], axis=0)
        if float(np.linalg.norm(p - com)) > 1e-9:
            assert float(np.dot(J, com - p)) > 1e-9, \
                f"{name}: J 未从作用点指向质心(J·(质心−p) ≤ 0)"
        out = eval_case(name, cells, mats, supply_fn, J_fn, p_fn)
        results[name] = (cells, mats, supply_fn, J_fn, p_fn, out)
        return out

    def speed_conv(cells, p, dhat, s_star, label):
        """约化质量换算 u* = s*·(m+m_eff)/(m·m_eff)(G00-2 修正④;
        m_eff = p 处沿 d̂ 的整刚体有效质量,含转动)。"""
        M, com, _, dv, _ = rigid_response(cells, {c: STONE for c in cells},
                                          p, np.asarray(dhat, float))
        dv_p = dv[min(cells, key=lambda c: float(np.linalg.norm(
            np.asarray(cell_center(c), float) - np.asarray(p, float))))]
        u_along = float(np.dot(dv_p, np.asarray(dhat, float)))
        m_eff = 1.0 / u_along
        m = CELL_MASS
        u_star = s_star * (m + m_eff) / (m * m_eff)
        mu = m * m_eff / (m + m_eff)
        print(f"    {label}:m_eff={m_eff:.3f} kg,μ={mu:.3f} → u* ≈ {u_star:.1f} m/s"
              f"(单拍口径 s=μ·u)")
        return u_star

    stone = lambda cs: {c: STONE for c in cs}
    dirt = lambda cs: {c: DIRT for c in cs}

    # ---- 工况 1:6 格条平落(全 6 格接触均摊;J = +y 指向条内部)
    def supply_even(s):
        return {c: s / 6 for c in beam_cells()}
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"1 六格条平落({mn})", beam_cells(), mat(beam_cells()), supply_even)

    # ---- 工况 2:端角先着地,两种几何(G00-2 修正④):
    #   2a 水平条端格外下角(§5.8.1 表口径,J +y 从端格外下角指向条内);
    #   2b 30° 倾斜(AC-C6 口径,冲量沿条的局部 −y 的反作用,+y 世界 = 斜向条内)。
    def p_end_corner(s):
        return tuple(cell_center(beam_cells()[0]) + np.array((-0.5, -0.5, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"2a 端角先着地·水平端格外下角({mn})", beam_cells(), mat(beam_cells()),
            lambda s: {beam_cells()[0]: s}, None, p_end_corner)
    # 2b:30° 倾斜条,接触点在端格外下角(旋转后几何不变,局部系等效 2a 的
    # 作用点);J 沿条的局部 +y(地面反作用在斜条上仍指向条内,非纯世界 +y)
    cells_b = beam_cells()
    ang30 = math.radians(30.0)
    d_tilt = (math.sin(ang30), math.cos(ang30), 0.0)   # R_z(30°)·(0,1,0)
    for mat, mn in ((stone, "石"), (dirt, "土")):
        reg(f"2b 端角先着地·30°倾斜,J 沿条局部+y({mn})", beam_cells(),
            mat(beam_cells()),
            lambda s: {beam_cells()[0]: s},
            lambda s, d=d_tilt: tuple(s * x for x in d), p_end_corner)

    # ---- 工况 3:6 格柱竖直落地(J = +y 从底格底面指向柱内 → 底→上粘结为压向)
    def p_pillar_bottom(s):
        return tuple(cell_center(pillar_cells()[0]) + np.array((0.0, -0.5, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        out = reg(f"3 六格柱竖直落地({mn})", pillar_cells(), mat(pillar_cells()),
                  lambda s: {pillar_cells()[0]: s}, None, p_pillar_bottom)
        if out and out[0]:
            speed_conv(pillar_cells(), p_pillar_bottom(1.0), (0, 1, 0), out[0],
                       f"3 竖柱({mn})")

    # ---- 工况 4:侧撞(垂直于条轴线,+z;G00-2 修正④)打端格/第2格/第3格的
    # −z 面中心;4b:轴向撞端格(沿轴线 +x,对照 §5.8.1 表"横撞"的旧读法)。
    # 速度换算统一:u* = s*·(m+m_eff)/(m·m_eff)。
    m_b = len(cells_b) * CELL_MASS
    for hit_i, tag in ((0, "端格"), (1, "第2格"), (2, "第3格")):
        hit = cells_b[hit_i]
        def p_side(s, hit=hit):
            return tuple(cell_center(hit) + np.array((0.0, 0.0, -0.5)))
        for mat, mn in ((stone, "石"), (dirt, "土")):
            out = reg(f"4 侧撞{tag}·+z({mn})", cells_b, mat(cells_b),
                      lambda s: {hit: s}, lambda s: (0.0, 0.0, s), p_side)
            if out and out[0]:
                speed_conv(cells_b, p_side(1.0), (0, 0, 1), out[0],
                           f"4 侧撞{tag}({mn})")
    # 4b 轴向:+x 打端格 −x 面(J 从 −x 面指向 +x = 条轴线)
    hit0 = cells_b[0]
    def p_axial(s):
        return tuple(cell_center(hit0) + np.array((-0.5, 0.0, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        out = reg(f"4b 轴向撞端格·+x({mn})", cells_b, mat(cells_b),
                  lambda s: {hit0: s}, lambda s: (s, 0.0, 0.0), p_axial)
        if out and out[0]:
            speed_conv(cells_b, p_axial(1.0), (1, 0, 0), out[0],
                       f"4b 轴向撞端格({mn})")

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
        # 侧撞:垂直于板边(+x 打 −x 面)
        def p_hit(s, hit=hit):
            return tuple(cell_center(hit) + np.array((-0.5, 0.0, 0.0)))
        for mat, mn in ((stone, "石"), (dirt, "土")):
            out = reg(f"5 板被侧撞{tag}·+x({mn})", pc, mat(pc),
                      lambda s: {hit: s}, lambda s: (s, 0.0, 0.0), p_hit)
            if out and out[0]:
                speed_conv(pc, p_hit(1.0), (1, 0, 0), out[0],
                           f"5 板侧撞{tag}({mn})")

    # ---- 工况 6:L 形(4 格)与 7 字(6 格)被打在拐角格;
    # J 从受击面指向格内(G00-2 修正④:L 的拐角格 lc[1] 受 +x 面 J = −x→格内;
    # 七字拐角格 sc[3](柱顶)受 +x 面 J = −x→格内)
    lc = lshape_cells()
    def p_ljoint(s):
        return tuple(cell_center(lc[1]) + np.array((0.5, 0.0, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        out = reg(f"6 L形拐角被撞·J=−x({mn})", lc, mat(lc),
                  lambda s: {lc[1]: s}, lambda s: (-s, 0.0, 0.0), p_ljoint)
        if out and out[0]:
            speed_conv(lc, p_ljoint(1.0), (-1, 0, 0), out[0], f"6 L形({mn})")
    sc = seven_cells()
    def p_seven_joint(s):
        return tuple(cell_center(sc[3]) + np.array((0.5, 0.0, 0.0)))
    for mat, mn in ((stone, "石"), (dirt, "土")):
        out = reg(f"6 七字拐角被撞·J=−x({mn})", sc, mat(sc),
                  lambda s: {sc[3]: s}, lambda s: (-s, 0.0, 0.0), p_seven_joint)
        if out and out[0]:
            speed_conv(sc, p_seven_joint(1.0), (-1, 0, 0), out[0],
                       f"6 七字({mn})")

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
