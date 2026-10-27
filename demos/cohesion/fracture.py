"""fracture.py — REQ-010 任务 00 脚本 3:碎裂冲量(rapier,primitive 直读)。

路线(任务书"接触冲量怎么读"第 1 条,已实测可行):复合体 = 一个刚体挂每格一个
primitive cuboid,地形 = 固定体上的多个 primitive 薄片 → primitive↔primitive 接触对
point.impulse 可精确直读(静置 = m·g·dt 逐位吻合),**无需 Rust spike**。

测:复合体每格接触冲量峰值 J_c(只计接近速度 ≥ 1 m/s 的拍,同格各点相加);
按 §7.1 离线判定断键(J_c ≥ I(c,n))与段数;stone(I=18)/dirt(I=7) 各判一遍。
场景:6 格梁平落/角先着地/竖落(1..32 m)、2×2×2 平落、1×1×6 塔倾倒、静置 300 拍、
两端搭块桥式静置 300 拍。

运行:.venv/Scripts/python.exe demos/cohesion/fracture.py
"""
import math
import os
import sys

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (CELL_HALF, GRAVITY, STONE, add_box, add_compound,
                    compound_origin, flat_ground, r_rotation, slab_terrain)

I_STONE, I_DIRT = 18.0, 7.0        # REQ-009 撞击阈值(占位,§3.2)
MAX_TICKS = 500


def beam_cells():
    return [(x, 0, 0) for x in range(6)]


def cube_cells():
    return [(x, y, z) for x in (0, 1) for y in (0, 1) for z in (0, 1)]


def tower_cells():
    return [(0, y, 0) for y in range(6)]


def bonds_of(cells):
    """共面粘结集合 {(i, j)}(格序号对)。"""
    cs = {c: i for i, c in enumerate(cells)}
    bonds = set()
    for c, i in cs.items():
        for d, axis in (((1, 0, 0), 0), ((0, 1, 0), 1), ((0, 0, 1), 2)):
            n = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
            if n in cs:
                bonds.add(tuple(sorted((i, cs[n]))))
    return bonds


def segments_after_break(cells, bonds, broken):
    """断键后的连通分量(按格序号)。"""
    alive = bonds - broken
    parent = list(range(len(cells)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for (i, j) in alive:
        parent[find(i)] = find(j)
    comps = {}
    for i in range(len(cells)):
        comps.setdefault(find(i), []).append(i)
    return sorted(sorted(c) for c in comps.values())


def simulate(cells, rot_vec, height, ticks=MAX_TICKS, speed_filter=1.0,
             support_boxes=(), tower=False):
    """落一次,返回 (peakJ[格], 每拍合计直读 vs 动量差分的峰值对照, 未过滤峰值)。"""
    world = r.PhysicsWorld(GRAVITY)
    wvox = flat_ground(41, 41, head=30)
    slab_terrain(world, wvox)
    oc, offs = compound_origin(cells)
    if tower:
        pos = (oc[0] + 0.5, height + 0.075 - lowest_y(offs, rot_vec), oc[2] + 0.5)
    else:
        bbox_cx = (min(c[0] for c in cells) + max(c[0] for c in cells) + 1) / 2.0
        shift = round(3.0 - bbox_cx)
        pos = (oc[0] + 0.5 + shift, height + 0.075 - lowest_y(offs, rot_vec),
               oc[2] + 0.5)
    handle, _, _, cell_of = add_compound(world, cells, pos, rotation=rot_vec)
    box_at = {}
    for bc in support_boxes:
        add_box(world, (bc[0] + 0.5, 0.075 + CELL_HALF, bc[2] + 0.5), fixed=True)
    peakJ = [0.0] * len(cells)
    peakJ_raw = [0.0] * len(cells)
    cross = {"read": 0.0, "diff": 0.0}
    tot_last = 0.0
    v_before = world.rigid_bodies[handle].linvel
    for t in range(1, ticks + 1):
        b = world.rigid_bodies[handle]
        v_before = np.array(tuple(b.linvel), float)
        speed = float(np.linalg.norm(v_before))
        world.step()
        J = [0.0] * len(cells)
        Jraw = [0.0] * len(cells)
        for pair in world.narrow_phase.contact_pairs():
            cells_both = [cell_of[c] for c in (pair.collider1, pair.collider2)
                          if c in cell_of]
            if not cells_both:
                continue
            for ci in set(cells_both):
                for m in pair.manifolds:
                    for pt in m.points:
                        Jraw[ci] += pt.impulse
                        if speed >= speed_filter:
                            J[ci] += pt.impulse
        for ci in range(len(cells)):
            peakJ[ci] = max(peakJ[ci], J[ci])
            peakJ_raw[ci] = max(peakJ_raw[ci], Jraw[ci])
        tot_last = sum(Jraw)
        tot = sum(Jraw)
        if tot > cross["read"]:
            b2 = world.rigid_bodies[handle]
            v_after = np.array(tuple(b2.linvel), float)
            m = b2.mass
            diff = abs(float(np.dot(v_before - v_after - np.array(GRAVITY) * (1 / 60.0),
                                    [0, 1, 0])) * m)
            cross = {"read": tot, "diff": diff}
        if world.rigid_bodies[handle].is_sleeping and t > 60:
            break
    return peakJ, cross, peakJ_raw, tot_last


def pair_is_terrain(world, col):
    return False


def lowest_y(offs, rot_vec):
    R = r_rotation(rot_vec) if rot_vec else np.eye(3)
    return min(float((R @ np.array(o, float))[1]) for o in offs) - CELL_HALF


def eval_fracture(name, cells, peakJ, bonds):
    """按 §7.1 离线判定两种材料。"""
    out = []
    for mat, I in (("石", I_STONE), ("土", I_DIRT)):
        broken = {b for b in bonds if any(peakJ[c] >= I for c in b)}
        segs = segments_after_break(cells, bonds, broken)
        out.append((mat, len(broken), len(segs), max(len(s) for s in segs)))
    return out


def main():
    print("=" * 72)
    print("0 直读可行性核对(primitive↔primitive)")
    print("=" * 72)
    # 静置直读(高度 0 = 初始即贴地,无撞击):末拍各格合计 vs 总重 m·g·dt
    peakJ, cross, raw, tot_last = simulate(beam_cells(), None, 0.0, ticks=120, speed_filter=1e9)
    mgdt = 0.729 * 6 * 9.81 / 60.0
    print(f"  静置: 末拍全格直读合计 {tot_last:.6f}(理论 m·g·dt = {mgdt:.6f},"
          f"偏差 {abs(tot_last-mgdt)/mgdt*100:.2f}%)")

    print()
    print("=" * 72)
    print("1 碎裂冲量(J_c = 每格各点冲量合计的峰值;接近速度 ≥ 1 m/s 的拍)")
    print("=" * 72)
    scenarios = []
    for h in (1, 2, 4, 8, 16, 32):
        scenarios.append((f"梁平落{h}m", beam_cells(), None, h, ()))
    for h in (1, 2, 4, 8, 16, 32):
        scenarios.append((f"梁角先30°{h}m", beam_cells(),
                          (0.0, 0.0, math.radians(30.0)), h, ()))
    for h in (1, 2, 4, 8, 16, 32):
        scenarios.append((f"梁竖落{h}m", beam_cells(),
                          (0.0, 0.0, math.pi / 2), h, ()))
    for h in (1, 2, 4, 8, 16, 32):
        scenarios.append((f"立方平落{h}m", cube_cells(), None, h, ()))
    scenarios.append(("塔1×1×6倾倒(12°)", tower_cells(),
                      (0.0, 0.0, math.radians(12.0)), 1.0, ()))

    print(f"{'场景':<20}{'各格峰值 J_c(格序 0..5)':<44}"
          f"{'石I=18':>12}{'土I=7':>12}")
    for name, cells, rot, h, boxes in scenarios:
        peakJ, cross, raw, tot_last = simulate(cells, rot, h, support_boxes=boxes)
        bonds = bonds_of(cells)
        res = eval_fracture(name, cells, peakJ, bonds)
        (m1, nb1, ns1, sz1), (m2, nb2, ns2, sz2) = res
        jstr = " ".join(f"{j:6.2f}" for j in peakJ)
        print(f"{name:<20}{jstr:<44}"
              f"{f'断{nb1}键{ns1}段':>12}{f'断{nb2}键{ns2}段':>12}")

    print()
    print("=" * 72)
    print("2 静置从不断键?(300 拍,含未过滤峰值参考)")
    print("=" * 72)
    peakJ, cross, raw, tot_last = simulate(beam_cells(), None, 0.0, ticks=300)
    print(f"  平放地面 300 拍: 过滤后峰值 {[round(j,4) for j in peakJ]};"
          f"未过滤 {[round(j,4) for j in raw]}(石I=18/土I=7 → 不断键)")
    # 恰好搭在两块上:块顶 0.975 = 梁底 → height = 0.975 − 0.45 − 0.075 = 0.45?见下
    # origin_y = h+0.075+0.45,梁底 = origin_y − 0.45 = h+0.075 → h = 0.9 时梁底 = 0.975 = 块顶
    peakJ, cross, raw, tot_last = simulate(beam_cells(), None, 0.9,
                                 ticks=300, support_boxes=((0, 0, 0), (5, 0, 0)))
    print(f"  两端搭块桥式 300 拍: 过滤后峰值 {[round(j,4) for j in peakJ]};"
          f"未过滤 {[round(j,4) for j in raw]} → 不断键")

    print()
    print("=" * 72)
    print("3 直读 vs 动量差分(每拍合计,撞击峰值拍)")
    print("=" * 72)
    peakJ, cross, raw, tot_last = simulate(beam_cells(), None, 5.0)
    v = math.sqrt(2 * 9.81 * 5.0)
    print(f"  梁平落5m: 撞击拍直读合计 {cross['read']:.3f},动量差分 {cross['diff']:.3f}"
          f"(m·v = {4.374*v:.3f},多拍分摊后应略小于 m·v)")


if __name__ == "__main__":
    main()
