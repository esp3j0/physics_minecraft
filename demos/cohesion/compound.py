"""compound.py — REQ-010 任务 00 脚本 2:复合体落定(rapier)。

复合体 = 一个动态刚体 + 每格一个 primitive cuboid(半边长 0.45,密度 1,摩擦 0.6,
弹性 0.1);地形 = 固定体上的多个 primitive 薄片(生产口径半厚 0.075)。
测:凝固判据拍数(线速度<0.1 且角速度<0.05 连续 20 拍 + 最近 20 拍位移 ≤ 3 mm)、
§8.1 吸附 snap 误差(上限 0.5 格 / 45°)、目标格重叠;断面旁生成初始重叠 0.025
是否弹开/被摩擦卡住;每格 cuboid vs 合并长方体对比。

运行:.venv/Scripts/python.exe demos/cohesion/compound.py
"""
import math
import os
import sys
import time

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (CELL_HALF, GRAVITY, add_box, add_compound, compound_origin,
                    flat_ground, r_rotation, slab_terrain, snap_error_max,
                    snap_pose, SettleTracker)

MAX_TICKS = 900


def shapes():
    return {
        "beam6": [(x, 0, 0) for x in range(6)],
        "L47": [(0, y, 0) for y in range(4)] + [(x, 0, 0) for x in range(1, 4)],
        "cube2": [(x, y, z) for x in (0, 1) for y in (0, 1) for z in (0, 1)],
        "plate3": [(x, 0, z) for x in range(3) for z in range(3)],
    }


def vertical_rot(shape_name):
    """竖落:长轴竖直。beam 绕 z 转 90°(x→y);L 本就竖直;板绕 x 转 90°(立棱)。"""
    return {"beam6": (0.0, 0.0, math.pi / 2), "L47": None,
            "cube2": None, "plate3": (math.pi / 2, 0.0, 0.0)}[shape_name]


def drop_modes(shape_name):
    """[(标签, 旋转, 落高, 垫块), ...];落高 = 最低点在薄片顶(0.075)之上 h 米。"""
    modes = [(f"平落{h:g}m", None, h, False) for h in (1.0, 2.0, 5.0, 10.0)]
    modes += [
        ("竖落", vertical_rot(shape_name), 2.0, False),
        ("倾斜20°", (0.0, 0.0, math.radians(20.0)), 2.0, False),
        ("倾斜45°", (0.0, 0.0, math.radians(45.0)), 2.0, False),
        ("垫3块", None, 2.0, True),
    ]
    return modes


def lowest_local_y(offsets, rot_vec):
    R = r_rotation(rot_vec) if rot_vec else np.eye(3)
    return min(float((R @ np.array(o, float))[1]) for o in offsets) - CELL_HALF


def spawn_pos(cells, offs, rot_vec, height):
    """合法网格生成位姿:原点在格中心上,形体中心平移到 x≈3、z≈0.5。"""
    oc = offs and compound_origin(cells)[0]
    bbox_cx = (min(c[0] for c in cells) + max(c[0] for c in cells) + 1) / 2.0
    bbox_cz = (min(c[2] for c in cells) + max(c[2] for c in cells) + 1) / 2.0
    return (oc[0] + 0.5 + round(3.0 - bbox_cx),
            height + 0.075 - lowest_local_y(offs, rot_vec),
            oc[2] + 0.5 + round(0.5 - bbox_cz))


def run_drop(shape_name, cells, label, rot_vec, height, blocks=False, merged=False):
    world = r.PhysicsWorld(GRAVITY)
    wvox = flat_ground(41, 41, head=30)
    terrain, solid = slab_terrain(world, wvox)
    block_cells = set()
    if blocks:
        for bc in [(1, 0, 0), (3, 0, 1), (2, 0, -1)]:
            add_box(world, (bc[0] + 0.5, 0.075 + CELL_HALF, bc[2] + 0.5), fixed=True)
            block_cells.add(bc)
    oc, offs = compound_origin(cells)
    origin_pos = spawn_pos(cells, offs, rot_vec, height)
    handle, oc, offs, _ = add_compound(world, cells, origin_pos, rotation=rot_vec,
                                       merged=merged)
    tracker = SettleTracker(world, [handle])
    settle = None
    for t in range(1, MAX_TICKS + 1):
        world.step()
        if tracker.step(t) == 1:
            settle = t
            break
    b = world.rigid_bodies[handle]
    Rk, ocell, ang = snap_pose(b.translation, b.rotation)
    err = snap_error_max(world, handle, ocell, offs, Rk)
    targets = set()
    for o in offs:
        t = np.array(ocell, float) + np.array(Rk, float) @ np.array(o, float)
        targets.add(tuple(int(round(v)) for v in t))
    overlap = bool((targets & solid) or (targets & block_cells))
    ok = err <= 0.5 and math.degrees(ang) <= 45.0 and not overlap
    return {"shape": shape_name, "mode": label, "settle": settle, "err": err,
            "ang": math.degrees(ang), "overlap": overlap, "ok": ok,
            "mass": b.mass}


def spawn_overlap_test():
    """断面旁零速度生成 6 格梁,端面与墙面薄片重叠 0.025;对照无墙自由落体。"""
    def build(with_wall):
        world = r.PhysicsWorld(GRAVITY)
        wvox = flat_ground(41, 41, head=30)
        slab_terrain(world, wvox)
        if with_wall:
            # 生产口径:墙格 +x 面在 x=6.0,薄片跨在面上(中心 6.0,半厚 0.075);
            # 梁 0.9 立方的端面在 5.95 → 与薄片重叠 0.025
            wall = r.RigidBodyBuilder(r.RigidBodyType.FIXED)
            world.add_body(wall, colliders=[
                r.Collider.cuboid(0.075, 0.5, 3.0).translation((6.0, 0.5 + k, 0.5))
                for k in range(6)])
        cells = [(x, 0, 0) for x in range(6)]
        handle, oc, offs, _ = add_compound(world, cells, (2.5, 0.45 + 1.0, 0.5))
        return world, handle
    world, h = build(True)
    vmax10, vy10, vx10 = 0.0, None, 0.0
    for t in range(1, 11):
        world.step()
        v = world.rigid_bodies[h].linvel
        vmax10 = max(vmax10, float(np.linalg.norm(tuple(v))))
        vy10, vx10 = v[1], max(vx10, abs(v[0]))
    world2, h2 = build(False)
    vy_free = None
    for t in range(1, 11):
        world2.step()
        vy_free = world2.rigid_bodies[h2].linvel[1]
    return {"vmax10": vmax10, "vy10": vy10, "vx10": vx10, "vy_free": vy_free,
            "free_expect": 9.81 * (10.0 / 60.0)}


def collider_compare():
    rows = []
    for name, cells in shapes().items():
        row = {"shape": name}
        for merged in (False, True):
            res = run_drop(name, cells, "平落2m", None, 2.0, merged=merged)
            t0 = time.perf_counter()
            world = r.PhysicsWorld(GRAVITY)
            wvox = flat_ground(41, 41, head=30)
            slab_terrain(world, wvox)
            oc, offs = compound_origin(cells)
            origin_pos = spawn_pos(cells, offs, None, 2.0)
            handle, _, _, _ = add_compound(world, cells, origin_pos, merged=merged)
            n = 300
            for _ in range(n):
                world.step()
            per_step = (time.perf_counter() - t0) / n * 1e3
            tag = "merged" if merged else "percell"
            row[f"{tag}_settle"] = res["settle"]
            row[f"{tag}_err"] = round(res["err"], 4)
            row[f"{tag}_mass"] = round(res["mass"], 4)
            row[f"{tag}_ms"] = per_step
        rows.append(row)
    return rows


def main():
    print("=" * 72)
    print("复合体落定(4 形状 × 8 落法;snap 上限 0.5 格 / 45°;目标格不得重叠)")
    print("=" * 72)
    print(f"{'形状':<8}{'落法':<10}{'静止拍':>7}{'snap位移':>10}{'角度°':>8}{'重叠':>6}{'整块凝固':>9}{'质量':>7}")
    results = []
    for name, cells in shapes().items():
        for label, rot_vec, height, blocks in drop_modes(name):
            res = run_drop(name, cells, label, rot_vec, height, blocks)
            results.append(res)
            st = res["settle"] if res["settle"] else f">{MAX_TICKS}"
            print(f"{res['shape']:<8}{res['mode']:<10}{st:>7}"
                  f"{res['err']:>10.3f}{res['ang']:>8.1f}{str(res['overlap']):>6}"
                  f"{'✓' if res['ok'] else '✗':>9}{res['mass']:>7.3f}")
    n_ok = sum(1 for r in results if r["ok"])
    print(f"\n整块凝固 {n_ok}/{len(results)}")
    causes = {}
    for r in results:
        if r["ok"]:
            continue
        c = ("角度>45°" if r["ang"] > 45 else
             ("snap位移>0.5" if r["err"] > 0.5 else "目标格被占"))
        causes.setdefault(c, []).append(f"{r['shape']}/{r['mode']}")
    for c, lst in causes.items():
        print(f"  失败原因 {c}: {len(lst)} 例({', '.join(lst)})")

    print("\n" + "=" * 72)
    print("生成时初始重叠(断面旁 6 格梁,端面与墙薄片重叠 0.025,零速度)")
    print("=" * 72)
    ov = spawn_overlap_test()
    print(f"  前 10 拍最大速度 {ov['vmax10']:.3f} m/s,最大水平速度 {ov['vx10']:.3f} m/s"
          f"(自由落体 10 拍末理论 {ov['free_expect']:.3f});"
          f"有墙 vy(10)={ov['vy10']:.3f} vs 无墙 {ov['vy_free']:.3f} m/s")

    print("\n" + "=" * 72)
    print("碰撞体做法对比(平落 2 m;每格 cuboid vs 合并长方体;300 步计时含建世界)")
    print("=" * 72)
    print(f"{'形状':<8} {'每格: 静止拍/snap/质量/ms每步':<34}"
          f"{'合并: 静止拍/snap/质量/ms每步':<34}一致")
    for row in collider_compare():
        same = (abs(row["percell_mass"] - row["merged_mass"]) < 1e-6
                and row["percell_err"] <= 0.5 and row["merged_err"] <= 0.5)
        pc = f"{row['percell_settle']}/{row['percell_err']:.3f}/{row['percell_mass']:.3f}/{row['percell_ms']:.2f}"
        mg = f"{row['merged_settle']}/{row['merged_err']:.3f}/{row['merged_mass']:.3f}/{row['merged_ms']:.2f}"
        print(f"{row['shape']:<8} {pc:<34} {mg:<34} {'等价' if same else '✗'}")


if __name__ == "__main__":
    main()
