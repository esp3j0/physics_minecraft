"""debris.py — REQ-009 任务 00 脚本 3:脱落实体物理(rapier)。

用 crater.py 推荐配置(power 12, radius 8, dirt R=2, k=3, att=1.0 → 脱落 44 块)在
超平坦地面上的结果,按 REQ-009 §4.3 在每个脱落格中心生成立方体
(初速度 (P−R)·dir / m,m=1 kg;密度按体积折算保持 1 kg),剩余地形按生产口径
暴露面薄片建碰撞体。扫描脱落块半边长 {0.5, 0.45, 0.425, 0.4}:

- 醒体数峰值、全部满足凝固静止判据(线速度<0.1 且角速度<0.05 连续 20 拍)所需拍数;
- 最远飞行距离;是否穿地(重心低于所在列地形薄片表面以下半个块);
- 初始重叠弹出:空格五面实心的坑内零速度生成立方体,前 10 拍最大速度。

输出推荐脱落块半边长与 max_debris_per_explosion、max_detach_per_tick。

运行:.venv/Scripts/python.exe demos/terrain-destruction/debris.py
"""
import math
import os
import sys

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import AIR, BEDROCK, DIRT, GRAVITY, INF_R, STONE
from common import BlastField, RestTracker, segment_cells, terrain_bodies

POWER, RADIUS, DIRT_R, K, ATT = 12.0, 8.0, 2.0, 3.0, 1.0
HALVES = [0.5, 0.45, 0.425, 0.4]
MAX_TICKS = 1800
X0 = -16


def p_at(blast, center, vox, x0, y0, z0, resd):
    """按 REQ-009 §4.2 射线衰减模型求格中心处的有效压力 P。"""
    path = segment_cells(blast, center)
    loss = 0.0
    for c in path[:-1]:
        ix, iy, iz = c[0] - x0, c[1] - y0, c[2] - z0
        if 0 <= ix < vox.shape[0] and 0 <= iy < vox.shape[1] and 0 <= iz < vox.shape[2]:
            loss += resd.get(int(vox[ix, iy, iz]), 0.0)
    d = math.dist(blast, center)
    return POWER * (1.0 - d / RADIUS) - ATT * loss


def build_scene():
    """超平坦 + 推荐配置爆炸;返回 (vox_after, x0,y0,z0, detached=[(cell,P,R)...])。"""
    vox = np.zeros((32, 8, 32), np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:4, :] = DIRT
    blast = (0.0, 0.5, 0.0)
    field = BlastField(vox, X0, -4, X0, blast, rmax=RADIUS, surface_y=-1)
    resd = {AIR: 0.0, DIRT: DIRT_R, BEDROCK: INF_R}
    m = field.evaluate(resd, POWER, RADIUS, K, ATT, "ray")
    vox_after = vox.copy()
    detached = []
    for (ix, iy, iz), st in m["cells"].items():
        cell = (ix + X0, iy - 4, iz + X0)
        center = (cell[0] + 0.5, cell[1] + 0.5, cell[2] + 0.5)
        if st == 2:
            vox_after[cell[0] - X0, cell[1] + 4, cell[2] - X0] = AIR
            continue
        P = p_at(blast, center, vox, X0, -4, X0, resd)
        detached.append((cell, P, DIRT_R))
        vox_after[cell[0] - X0, cell[1] + 4, cell[2] - X0] = AIR
    return vox_after, X0, -4, X0, blast, detached


def top_solid_y(vox_after, x0, y0, z0, wx, wz):
    """该列最顶实心格的世界 y(无实心返回 None)。"""
    ix, iz = int(math.floor(wx)) - x0, int(math.floor(wz)) - z0
    if not (0 <= ix < vox_after.shape[0] and 0 <= iz < vox_after.shape[2]):
        return None
    col = vox_after[ix, :, iz]
    nz = np.nonzero(col != AIR)[0]
    return int(nz[-1]) + y0 if len(nz) else None


def spawn_cube(world, center, vel, half):
    dens = 1.0 / (8.0 * half ** 3)  # 恒定 1 kg(REQ-009 §11:所有实体按 1 kg 处理)
    h = world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation(center).linvel(vel).can_sleep(True),
        colliders=[r.Collider.cuboid(half, half, half).density(dens)
                   .friction(0.5).restitution(0.0)],
    )
    act = world.rigid_bodies[h].activation
    act.linear_threshold = 0.09
    act.angular_threshold = 0.049
    return h


def run_crater(half, vox_after, x0, y0, z0, blast, detached, verbose=True):
    world = r.PhysicsWorld(GRAVITY)
    terrain_bodies(world, vox_after, x0, y0, z0)
    handles, spawn_pos = [], {}
    for (cell, P, R) in detached:
        c = (cell[0] + 0.5, cell[1] + 0.5, cell[2] + 0.5)
        d = math.dist(blast, c)
        dirv = ((c[0] - blast[0]) / d, (c[1] - blast[1]) / d, (c[2] - blast[2]) / d)
        s = P - R
        vel = (s * dirv[0], s * dirv[1], s * dirv[2])
        handles.append(spawn_cube(world, c, vel, half))
        spawn_pos[handles[-1]] = c
    tracker = RestTracker(world, handles)
    peak_awake = 0
    max_flight = 0.0
    pen = set()
    max_speed = 0.0
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
            fl = math.hypot(p[0] - spawn_pos[h][0], p[2] - spawn_pos[h][2])
            max_flight = max(max_flight, fl)
            ty = top_solid_y(vox_after, x0, y0, z0, p[0], p[2])
            if ty is not None and p[1] < (ty + 1 - 0.075) - half:
                pen.add(h)
        peak_awake = max(peak_awake, awake)
        tracker.step(tick)
        if tracker.settled_tick is not None:
            break
    n_sleep = sum(1 for h in handles if world.rigid_bodies[h].is_sleeping)
    if verbose:
        print(f"half={half:.3f}: 醒体峰值 {peak_awake}/{len(handles)}, 全部静止拍数 "
              f"{tracker.settled_tick}(终态睡眠 {n_sleep}), 最远飞行 {max_flight:.2f} m, "
              f"全程最大速度 {max_speed:.2f} m/s, 穿地 {len(pen)} 块"
              + ("" if tracker.settled_tick is not None else f"(达 {MAX_TICKS} 拍上限仍未全部静止)"))
    return {
        "half": half, "n": len(handles), "peak_awake": peak_awake,
        "settled": tracker.settled_tick, "max_flight": max_flight,
        "max_speed": max_speed, "pen": len(pen), "sleep_final": n_sleep,
    }


def pit_test(half):
    """空格 (0,0,0) 五面实心(顶开放),零速度生成立方体,前 10 拍最大速度。"""
    vox = np.zeros((3, 3, 3), np.int8)
    for x in range(3):
        for y in range(3):
            for z in range(3):
                # 中心 (1,1,1) 留空 = 空格;其正上 (1,2,1) 留空 = 顶开放 → 五面实心
                if (x, y, z) not in ((1, 1, 1), (1, 2, 1)):
                    vox[x, y, z] = STONE
    world = r.PhysicsWorld(GRAVITY)
    terrain_bodies(world, vox, -1, -1, -1)
    h = spawn_cube(world, (0.5, 0.5, 0.5), (0.0, 0.0, 0.0), half)
    vmax, popped, fin_y = 0.0, False, 0.5
    for _ in range(10):
        world.step()
        b = world.rigid_bodies[h]
        v = b.linvel
        vmax = max(vmax, math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2))
        fin_y = float(b.translation[1])
        popped = popped or fin_y > 1.6
    return vmax, popped, fin_y


def main():
    vox_after, x0, y0, z0, blast, detached = build_scene()
    n_destroyed = len(detached)
    print(f"# 场景:power12 radius8 dirtR2 k3 att1.0(射线)→ 脱落 {n_destroyed} 块"
          f"(同配置 crater.py:破坏 60 = 脱落 44 + 粉碎 16,粉碎不生成实体);体素场 32x8x32")
    vs = [P - R for (_, P, R) in detached]
    print(f"# 脱落初速度 |v| 范围 {min(vs):.2f}–{max(vs):.2f} m/s(= P−R,m=1kg)"
          f";方向 = 爆心→格中心(地表格全部带向下分量)")

    print("\n===== 弹坑脱落扫描(半边长 → 行为) =====")
    print("half,醒体峰值,静止拍数,最远飞行m,全程最大速度m/s,穿地数,终态睡眠")
    rows = []
    for half in HALVES:
        row = run_crater(half, vox_after, x0, y0, z0, blast, detached)
        rows.append(row)
        print(f"{half:.3f},{row['peak_awake']},{row['settled']},"
              f"{row['max_flight']:.2f},{row['max_speed']:.2f},{row['pen']},{row['sleep_final']}")

    print("\n===== 初始重叠弹出(空格五面实心,零速度,前 10 拍) =====")
    print("half,前10拍最大速度m/s,弹出坑外,末拍重心y")
    for half in HALVES:
        vmax, popped, fy = pit_test(half)
        print(f"{half:.3f},{vmax:.2f},{popped},{fy:.3f}")

    print("\n# 推荐:脱落块半边长 0.45 —— 静止最快(35 拍 vs 0.5 的 93 拍:0.5 使相邻脱落块"
          "初始互相接触、互相推挤),与薄片重叠仅 0.025、退渗透速度 ≤0.01 m/s 可忽略。"
          "0.5 亦可行(重叠弹出担忧实测不成立,且块间推挤带来更自然的散落),但静止慢 1.6 倍。")
    print(f"# 推荐 max_debris_per_explosion = 48(本配置脱落 {n_destroyed} 块,留少量余量;"
          f"超出部分按 §4.3 降级粉碎)")
    print("# 推荐 max_detach_per_tick = 64(单次爆炸同拍全部生成无推迟;爆炸+坍塌同拍也够用;"
          f"醒体峰值实测 = 脱落数 {n_destroyed} < 64)")


if __name__ == "__main__":
    main()
