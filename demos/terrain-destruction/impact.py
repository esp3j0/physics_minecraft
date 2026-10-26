"""impact.py — REQ-009 任务 00 脚本 4:撞击冲量(rapier)。

先在运行时探查绑定接触冲量读取链路(结论追加 _API_NOTES.md)。实测发现:
复合地形碰撞体(生产口径)的接触点 impulse 回读恒为 0(impact step 亦然),
动态↔动态 primitive 之间正常 → 本脚本全部改用 REQ-009 §5 认可的动量差分法
(被测立方体每步 I = m·Δv − m·g⃗·dt;单接触场景与接触冲量等价):

- 静载 1–8 层 1 kg 叠放:最底下立方体的地形冲量 = m·Δvy + m·g·dt + 上方载荷,
  其中上方载荷由"底块↔第二块"接触对直读(动态↔dynamic,实测可靠);
- 竖直下落 1/2/5/10 m 与水平撞墙 5/10/20 m/s 的每步冲量峰值;
- 每个场景同时打印接触对直读值作对照(直观暴露绑定缺陷)。

输出两类冲量分布,推荐 dirt/stone/sand 的 impact_threshold(8 层静载峰值
< 阈值/3),并给出多大速度能撞坏 dirt/stone。

运行:.venv/Scripts/python.exe demos/terrain-destruction/impact.py
"""
import math
import os
import sys

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import AIR, BEDROCK, DIRT, GRAVITY, INF_R, STONE  # noqa: F401
from common import DT, terrain_bodies

G = 9.81
HALF = 0.5
MASS = 1.0


def pair_impulse_with(world, body):
    """body 与其它动态体接触对的点冲量之和(动态↔primitive,实测可靠)。"""
    total = 0.0
    for pair in world.narrow_phase.contact_pairs():
        b1 = world.colliders.get(pair.collider1).parent
        b2 = world.colliders.get(pair.collider2).parent
        if (b1 == body or b2 == body) and b1 != b2:
            other = b2 if b1 == body else b1
            if world.rigid_bodies[other].is_dynamic:
                for mf in pair.manifolds:
                    for p in mf.points:
                        total += p.impulse
    return total


def pair_impulse_terrain(world, body, terrain):
    """body 与地形接触对的点冲量之和(对照列:复合体下恒 0,展示绑定缺陷)。"""
    total = 0.0
    for pair in world.narrow_phase.contact_pairs():
        b1 = world.colliders.get(pair.collider1).parent
        b2 = world.colliders.get(pair.collider2).parent
        if (b1 == body and b2 in terrain) or (b2 == body and b1 in terrain):
            for mf in pair.manifolds:
                for p in mf.points:
                    total += p.impulse
    return total


def make_ground(nx=24, nz=24):
    vox = np.zeros((nx, 8, nz), np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:4, :] = DIRT
    return vox, -12, -4, -12


def make_wall_scene():
    vox = np.zeros((28, 13, 21), np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:4, :] = DIRT
    vox[3 + 14:3 + 3 + 14, 4:12, 4:17] = STONE  # 墙 x=3..5, z=-4..4, y=0..7
    return vox, -14, -4, -10


def add_cube(world, center, vel=(0.0, 0.0, 0.0)):
    return world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation(center).linvel(vel).can_sleep(False),
        colliders=[r.Collider.cuboid(HALF, HALF, HALF).density(1.0)
                   .friction(0.5).restitution(0.0)],
    )


def probe_api(world):
    """运行时探查接触冲量读取链路,打印结论(供 _API_NOTES.md 引用)。"""
    ps = world.narrow_phase.contact_pairs()
    print("[API] world.narrow_phase.contact_pairs() -> ContactPair 列表;"
          f"本例 {len(ps)} 对(立方体已落地)")
    p = ps[0]
    c1 = world.colliders.get(p.collider1)
    print("[API] pair.collider1/2 = ColliderHandle;"
          "world.colliders.get(h).parent -> RigidBodyHandle =", c1.parent)
    print("[API] pair.manifolds[i].points[j].impulse(法向冲量标量)/ dist(刺穿量)/"
          "tangent_impulse;pair.total_impulse() 与 total_impulse_magnitude() 是方法")
    print("[API] narrow_phase 无需手动 update,world.step() 后即有效")
    print("[API] ⚠ 缺陷:复合 collider(生产地形口径)参与的对,point.impulse 恒读 0"
          "(含撞击步);primitive↔primitive(动态↔动态)正常 → 冲量测量用动量差分")


def probe_defect():
    """直读缺陷的证据行:静置 1kg 立方体,动量差分 vs 复合地形直读。"""
    vox, x0, y0, z0 = make_ground()
    world = r.PhysicsWorld(GRAVITY)
    terrain = set(terrain_bodies(world, vox, x0, y0, z0).values())
    c = add_cube(world, (0.5, 0.575, 0.5))
    prev = 0.0
    for _ in range(300):
        world.step()
        prev = world.rigid_bodies[c].linvel[1]
    mdiff = MASS * (prev + G * DT)
    direct = pair_impulse_terrain(world, c, terrain)
    print(f"[缺陷证据] 静置 1 kg:动量差分 = {mdiff:.4f}(理论 mg·dt=0.1635),"
          f"复合地形直读 = {direct:.4f}")


def static_stack(n, settle=240, window=120):
    """n 层叠放;稳定后窗口内每步:地形对底块的冲量(动量差分+上方载荷)。"""
    vox, x0, y0, z0 = make_ground()
    world = r.PhysicsWorld(GRAVITY)
    terrain = set(terrain_bodies(world, vox, x0, y0, z0).values())
    handles = [add_cube(world, (0.5, 0.5 + i, 0.5)) for i in range(n)]
    bottom = handles[0]
    best = direct_best = above_best = 0.0
    prev = 0.0
    for tick in range(1, settle + window + 1):
        world.step()
        vy = world.rigid_bodies[bottom].linvel[1]
        if tick > settle:
            i_above = pair_impulse_with(world, bottom)
            i_self = MASS * (vy - prev + G * DT)
            best = max(best, i_self + i_above)
            above_best = max(above_best, i_above)
            direct_best = max(direct_best, pair_impulse_terrain(world, bottom, terrain))
        prev = vy
    return best, direct_best, above_best


def drop(h):
    """从高 h m 竖直下落(重心起始 0.5+h);返回 (v_impact, 峰值动量差分, 峰值直读)。"""
    vox, x0, y0, z0 = make_ground()
    world = r.PhysicsWorld(GRAVITY)
    terrain = set(terrain_bodies(world, vox, x0, y0, z0).values())
    cube = add_cube(world, (0.5, HALF + h, 0.5))
    v_imp = math.sqrt(2 * G * (h - 0.075))
    best = direct_best = 0.0
    prev = (0.0, 0.0, 0.0)
    for _ in range(180):
        world.step()
        v = world.rigid_bodies[cube].linvel
        i_vert = MASS * (v[1] - prev[1] + G * DT)
        i_horiz = math.sqrt((v[0] - prev[0]) ** 2 + (v[2] - prev[2]) ** 2) * MASS
        best = max(best, math.hypot(i_vert, i_horiz))
        direct_best = max(direct_best, pair_impulse_terrain(world, cube, terrain))
        prev = v
    return v_imp, best, direct_best


def wall_hit(speed):
    """水平撞墙(墙面 x=3,命中高度 y≈6.4);45 拍窗口内峰值。"""
    vox, x0, y0, z0 = make_wall_scene()
    world = r.PhysicsWorld(GRAVITY)
    terrain = set(terrain_bodies(world, vox, x0, y0, z0).values())
    cube = add_cube(world, (2.0, 6.5, 0.5), (speed, 0.0, 0.0))
    best = direct_best = 0.0
    prev = (0.0, 0.0, 0.0)
    for _ in range(45):
        world.step()
        v = world.rigid_bodies[cube].linvel
        i_vec = (MASS * (v[0] - prev[0]), MASS * (v[1] - prev[1] + G * DT),
                 MASS * (v[2] - prev[2]))
        best = max(best, math.sqrt(sum(c * c for c in i_vec)))
        direct_best = max(direct_best, pair_impulse_terrain(world, cube, terrain))
        prev = v
    return best, direct_best


def main():
    world = r.PhysicsWorld(GRAVITY)
    vox, x0, y0, z0 = make_ground()
    terrain_bodies(world, vox, x0, y0, z0)
    add_cube(world, (0.5, 0.575, 0.5))
    for _ in range(60):
        world.step()
    probe_api(world)
    probe_defect()

    print("\n===== 静载:1–8 层 1 kg 叠放,底块受到的地形冲量(稳定后 120 拍窗口峰值) =====")
    print("层数,理论n·mg·dt,动量差分+上方载荷,其中上方载荷(接触对直读),地形直读(缺陷对照)")
    static_pts = {}
    for n in range(1, 9):
        best, direct, above = static_stack(n)
        static_pts[n] = best
        print(f"{n},{n * MASS * G * DT:.4f},{best:.4f},{above:.4f},{direct:.4f}")

    print("\n===== 撞击:竖直下落(峰值 = 全程每步最大) =====")
    print("高度m,触地速度m/s,峰值冲量(动量差分),直读(缺陷对照)")
    drop_pts = {}
    for h in (1, 2, 5, 10):
        v, best, direct = drop(h)
        drop_pts[h] = (v, best)
        print(f"{h},{v:.2f},{best:.4f},{direct:.4f}")

    print("\n===== 撞击:水平撞墙(命中高度 y≈6.4,45 拍窗口) =====")
    print("速度m/s,峰值冲量(动量差分),直读(缺陷对照)")
    wall_pts = {}
    for s in (5, 10, 20):
        best, direct = wall_hit(s)
        wall_pts[s] = best
        print(f"{s},{best:.4f},{direct:.4f}")

    s8 = static_pts[8]
    d1 = drop_pts[1][1]
    d10 = drop_pts[10][1]
    floor = 3 * s8  # REQ:8 层静载峰值 < 阈值/3
    print("\n===== 阈值推导(动量差分口径,kg·m/s;实测触地冲量 ≈ m·v,单步吸收) =====")
    print(f"8 层静载峰值 {s8:.3f} → 阈值下界(×3)= {floor:.2f}")
    print(f"1 m 落地峰值 {d1:.3f}(行走/跳跃落地的量级,不应破坏);"
          f"10 m 落地峰值 {d10:.3f}")
    th_dirt = math.ceil(max(floor, 1.5 * d1))
    th_sand = math.ceil(max(floor, 0.8 * d1))
    th_stone = math.ceil(max(2.5 * th_dirt, d10))
    print(f"推荐 impact_threshold:dirt = {th_dirt},stone = {th_stone},sand = {th_sand}(kg·m/s)")
    print(f"校验:静载 8 层 {s8:.3f} < 阈值/3(dirt {th_dirt/3:.2f} ✓,"
          f"stone {th_stone/3:.2f} ✓,sand {th_sand/3:.2f} ✓)")
    print(f"能撞坏 dirt({th_dirt})的速度 ≈ {th_dirt - MASS * G * DT:.1f} m/s"
          f"(约 1 kg 撞击者,单步吸收;对应下落高度 ≈ {(th_dirt - MASS * G * DT) ** 2 / (2 * G):.1f} m)")
    print(f"能撞坏 stone({th_stone})的速度 ≈ {th_stone - MASS * G * DT:.1f} m/s"
          f"(对应下落高度 ≈ {(th_stone - MASS * G * DT) ** 2 / (2 * G):.1f} m)")
    print("# 单位备注:rapier 冲量单位为 kg·m/s(N·s);REQ-009 §3.1 写的 'J' 与量纲不符,见问题清单")


if __name__ == "__main__":
    main()
