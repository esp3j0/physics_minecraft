"""bridge.py — REQ-009 任务 00 脚本 5:30 格石桥塌落(rapier)。

1 宽 × 1 高 × 30 长的石桥(格 y=6,离地 6 m,x=-15..14,z=0),每格同时变为零速度
立方体(半边长取 debris.py 推荐值 0.45,并对比 0.5),落地过程:
全部静止所需拍数、最大速度、落地后碎块分布(ASCII 俯视图)、是否有立方体被弹飞
超过 10 m 或穿地(重心沉到地表平面以下,即半块以上没入)。
评价"逐格独立实体"的塌落形态是否可接受。

运行:.venv/Scripts/python.exe demos/terrain-destruction/bridge.py
"""
import math
import os
import sys

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import AIR, BEDROCK, DIRT, GRAVITY
from common import RestTracker, terrain_bodies

HALVES = [0.45, 0.5]
MAX_TICKS = 1800
NX, NZ, X0, Z0 = 80, 32, -40, -16


def top_solid_y(vox, x0, y0, z0, wx, wz):
    ix, iz = int(math.floor(wx)) - x0, int(math.floor(wz)) - z0
    if not (0 <= ix < vox.shape[0] and 0 <= iz < vox.shape[2]):
        return None
    nz_ = np.nonzero(vox[ix, :, iz] != AIR)[0]
    return int(nz_[-1]) + y0 if len(nz_) else None


def spawn_cube(world, center, half):
    dens = 1.0 / (8.0 * half ** 3)  # 恒 1 kg
    h = world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation(center).can_sleep(True),
        colliders=[r.Collider.cuboid(half, half, half).density(dens)
                   .friction(0.5).restitution(0.0)],
    )
    act = world.rigid_bodies[h].activation
    act.linear_threshold = 0.09
    act.angular_threshold = 0.049
    return h


def run(half):
    vox = np.zeros((NX, 8, NZ), np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:4, :] = DIRT
    world = r.PhysicsWorld(GRAVITY)
    terrain_bodies(world, vox, X0, -4, Z0)
    cells = [(x, 6, 0) for x in range(-15, 15)]
    handles, spawn = [], {}
    for (x, y, z) in cells:
        c = (x + 0.5, y + 0.5, z + 0.5)
        handles.append(spawn_cube(world, c, half))
        spawn[handles[-1]] = c
    tracker = RestTracker(world, handles)
    max_speed = 0.0
    max_disp = 0.0
    flung = set()
    sunk = set()
    min_y = 1e9
    peak_awake = 0
    for tick in range(1, MAX_TICKS + 1):
        world.step()
        awake = 0
        for h in handles:
            b = world.rigid_bodies[h]
            if not b.is_sleeping:
                awake += 1
            v = b.linvel
            sp = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            max_speed = max(max_speed, sp)
            p = b.translation
            d = math.hypot(p[0] - spawn[h][0], p[2] - spawn[h][2])
            max_disp = max(max_disp, d)
            if d > 10.0:
                flung.add(h)
            ty = top_solid_y(vox, X0, -4, Z0, p[0], p[2])
            if ty is not None and p[1] < ty + 1 - 0.075:
                sunk.add(h)  # 重心低于地表平面 → 半块以上没入
            min_y = min(min_y, p[1])
        peak_awake = max(peak_awake, awake)
        tracker.step(tick)
        if tracker.settled_tick is not None:
            break
    # 终态俯视图
    grid = {}
    for h in handles:
        p = world.rigid_bodies[h].translation
        key = (int(math.floor(p[0])), int(math.floor(p[2])))
        grid[key] = grid.get(key, 0) + 1
    xs = [k[0] for k in grid]
    rows = []
    for z in range(4, -5, -1):
        line = f"z={z:>2} "
        for x in range(min(xs) - 2, max(xs) + 3):
            n = grid.get((x, z), 0)
            line += " " if n == 0 else ("." if n == 1 else str(min(n, 9)))
        rows.append(line)
    head = "     " + "".join(str(abs(x) % 10) for x in range(min(xs) - 2, max(xs) + 3))
    print("\n".join(rows + [head]))
    return {
        "half": half, "settled": tracker.settled_tick, "peak_awake": peak_awake,
        "max_speed": max_speed, "max_disp": max_disp, "flung": len(flung),
        "sunk": len(sunk), "min_y": min_y, "spread": (min(xs), max(xs)),
    }


def main():
    for half in HALVES:
        res = run(half)
        tag = "推荐(0.45)" if half == 0.45 else "对比(0.5)"
        print(f"===== half={half}({tag}):30 格石桥 y=6 同时变零速度立方体 =====")
        print(f"全部静止拍数 {res['settled']}(上限 {MAX_TICKS}),醒体峰值 {res['peak_awake']},"
              f"全程最大速度 {res['max_speed']:.2f} m/s")
        print(f"最大水平位移 {res['max_disp']:.2f} m;弹飞 >10 m:{res['flung']} 块;"
              f"穿地(重心低于地表平面):{res['sunk']} 块;最低重心 y={res['min_y']:.3f}")
        print(f"落点 x 范围 {res['spread'][0]}..{res['spread'][1]}"
              f"(原桥 x=-15..14)")


if __name__ == "__main__":
    main()
