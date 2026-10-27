"""bridge.py — REQ-010 任务 00 脚本 4:20 格石桥整块塌落(rapier)。

同一 20 格石桥(1×1×20,离地 6 m,同拍失去全部支撑、零初速)三种做法:
(a) REQ-009 逐格:20 个单格实体;(b) D3 切块 S=6:4 段复合体(6/6/6/2);
(c) 不切:1 个 20 格复合体(仅参照)。
对比:全部静止拍数、醒着刚体数与碰撞体数、落地分布(ASCII 俯视)、
各段能否整块凝固、落定后是否连成一排。

运行:.venv/Scripts/python.exe demos/cohesion/bridge.py
"""
import math
import os
import sys

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (CELL_HALF, GRAVITY, add_compound, compound_origin,
                    flat_ground, slab_terrain, snap_error_max, snap_pose,
                    SettleTracker)

HEIGHT = 6.0
MAX_TICKS = 900
BRIDGE = [(x, 6, 0) for x in range(20)]


def build_world():
    world = r.PhysicsWorld(GRAVITY)
    wvox = flat_ground(61, 61, head=12)
    slab_terrain(world, wvox)
    return world


def spawn_bridges(mode):
    """返回 (world, handles, segs)。segs: [(名称, cells, handle, origin_cell, offsets)]。"""
    world = build_world()
    segs = []
    if mode == "per-cell":
        for i, c in enumerate(BRIDGE):
            h = world.add_body(
                r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
                .translation((c[0] + 0.5, c[1] + 0.5, c[2] + 0.5)),
                colliders=[r.Collider.cuboid(CELL_HALF, CELL_HALF, CELL_HALF)
                           .density(1.0).friction(0.6).restitution(0.1)])
            segs.append((f"c{i}", [c], h, c, [(0, 0, 0)]))
    else:
        if mode == "segments":
            groups = {}
            for c in BRIDGE:
                groups.setdefault((c[0] // 6, c[1] // 6, c[2] // 6), []).append(c)
            parts_list = [groups[k] for k in sorted(groups)]
        else:
            parts_list = [list(BRIDGE)]
        for gi, cells in enumerate(parts_list):
            oc, offs = compound_origin(cells)
            pos = (oc[0] + 0.5, oc[1] + 0.5, oc[2] + 0.5)
            h, _, _, _ = add_compound(world, cells, pos)
            segs.append((f"s{gi}", cells, h, oc, offs))
    return world, segs


def run_mode(mode):
    world, segs = simulate_ready = spawn_bridges(mode)
    tracker = SettleTracker(world, [s[2] for s in segs])
    all_rest = None
    max_speed = 0.0
    max_horiz = 0.0
    for t in range(1, MAX_TICKS + 1):
        world.step()
        for s in segs:
            b = world.rigid_bodies[s[2]]
            v = np.array(tuple(b.linvel), float)
            max_speed = max(max_speed, float(np.linalg.norm(v)))
            max_horiz = max(max_horiz, math.hypot(v[0], v[2]))
        if tracker.step(t) == len(segs):
            all_rest = t
            break
    # 落点格与 snap 判定
    rows = []
    landed = {}
    snap_ok = 0
    for name, cells, h, oc, offs in segs:
        b = world.rigid_bodies[h]
        Rk, ocell, ang = snap_pose(b.translation, b.rotation)
        err = snap_error_max(world, h, ocell, offs, Rk)
        ok = err <= 0.5 and math.degrees(ang) <= 45.0
        snap_ok += ok
        for off in offs:
            tgt = np.array(ocell, float) + np.array(Rk, float) @ np.array(off, float)
            landed[tuple(int(round(v)) for v in tgt)] = name
        rows.append((name, len(cells), err, math.degrees(ang), ok))
    # 连续性:落定行 y=0 的 x 覆盖
    xs = sorted(x for (x, y, z) in landed if y <= 1)
    gaps = [] if not xs else [x for x in range(xs[0], xs[-1] + 1)
                              if x not in set(xs)]
    # ASCII 俯视(z=-2..2 × x=-2..24)
    lines = []
    for z in range(2, -3, -1):
        row = []
        for x in range(-2, 25):
            row.append(landed.get((x, 0, z), ".")[-1] if (x, 0, z) in landed else ".")
        lines.append(f"z={z:+d} " + "".join(row))
    awake_bodies = sum(1 for s in segs if not world.rigid_bodies[s[2]].is_sleeping)
    return {
        "mode": mode, "n_bodies": len(segs),
        "n_colliders": sum(len(s[1]) for s in segs),
        "rest": all_rest, "max_speed": max_speed, "max_horiz": max_horiz,
        "rows": rows, "snap_ok": snap_ok, "gaps": gaps,
        "ascii": "\n".join(lines), "landed": landed,
        "xs": (xs[0], xs[-1]) if xs else None,
    }


def main():
    print("=" * 72)
    print("20 格石桥(1×1×20,离地 6 m,同拍失支撑,零初速)三种塌落对比")
    print("=" * 72)
    results = []
    for mode in ("per-cell", "segments", "whole"):
        res = run_mode(mode)
        results.append(res)
        label = {"per-cell": "a) REQ-009 逐格(20 体)",
                 "segments": "b) D3 切块 S=6(4 段)",
                 "whole": "c) 不切(1×20 格,参照)"}[mode]
        print(f"\n{label}:")
        print(f"  醒着刚体(初始)={res['n_bodies']},碰撞体={res['n_colliders']},"
              f"全部静止={res['rest'] or f'>{MAX_TICKS}'} 拍,最大速度 "
              f"{res['max_speed']:.2f} m/s,最大水平速度 {res['max_horiz']:.3f} m/s")
        for (name, n, err, ang, ok) in res["rows"]:
            print(f"    {name:>4}({n:>2}格): snap位移 {err:.3f},角度 {ang:.1f}°,"
                  f"整块凝固 {'✓' if ok else '✗'}")
        cover = (f"x {res['xs'][0]}..{res['xs'][1]}" if res["xs"] else "—")
        print(f"  落定行连续: {cover},缺口 {len(res['gaps'])} 个"
              f"{(' ' + str(res['gaps'][:10])) if res['gaps'] else ''}")
        print("  俯视(y=0 行,'.'=空,a-d/o=段落格):")
        print(res["ascii"])

    print()
    print("=" * 72)
    print("形态评价")
    print("=" * 72)
    for res in results:
        n_move = len({name for name, _, _, _, ok in res["rows"]})
        print(f"  {res['mode']}: 整块凝固 {res['snap_ok']}/{n_move},"
              f"落定覆盖 {res['xs']},与原桥位(x 0..19)一致="
              f"{res['xs'] == (0, 19)}")


if __name__ == "__main__":
    main()
