"""rest.py — REQ-010 单元 F00b 脚本 2:静置、滑行与接触归格(薄片内缩的副作用,D15)。

每个工况两种地形薄片口径并排跑(straddle=跨面现状 / inset=内缩 D15):

1. 静置高度:单格、6 格条、2×2×2 复合体静置在平地、台阶边缘(一半悬空)上的
   质心高度与静止拍数——内缩让一切"站在顶面薄片上"的高度降 0.075;
2. 滑行:单格与 6 格条以 2/5 m/s 沿 x 与沿 45° 对角在平地上滑 120 拍——速度损失、
   竖直最大跳动(检查内缩后相邻薄片接缝会不会产生"幽灵碰撞");再在 1 格高台阶的
   上沿与下沿各滑一次;
3. 贴墙与边角:单格贴着墙面下落、落在凸边(两暴露面交线)上、落进内角——
   是否卡住或弹飞;
4. 接触归格:生产按"接触点沿法向退 0.1 m 取格"(sim.rs:2961、
   destruction.rs IMPACT_SAMPLE_DEPTH=0.1)。对齐撞(竖落)与齐平侧撞在两种口径下
   统计"归到被触薄片自己的格"的比例(0.1 是否够越过内缩后的薄片),并扫描
   退距 0.05..0.3 的正确率;
5. impact-corner 预演(正确几何):smoke/scenarios/req010-impact-corner.toml
   平移 Δx=0、Δy=+60(世界地表空气层 y=-60 ↔ 局部 y=0)、Δz=-20:
   石柱 (8, 0..1, 0)、门楣 (5..8, 2, 0);投射物 (14, 2.6, 0.2)、
   velocity=[-40,-7,0]。先用 flow 纯函数核对 C 与 S,再用完整管线跑两种口径的
   轨迹与落点——取代 F00 作废的预演,供 F04 改写场景期望。

运行:.venv/Scripts/python.exe demos/inset-slab/rest.py   (~5 分钟)
GBK 控制台:先 set PYTHONIOENCODING=utf-8。
"""
import math
import os
import sys

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (AIR, CELL_HALF, CELL_MASS, DT, G, MODES, MODE_CN,
                    WatchPipeline, add_projectile, cell_center, cells_of_body,
                    flat_ground, rest_tick_of, run_plain, sample_points_raw,
                    side_ascii, slab_terrain_mode, spawn_cells)


def f3(x):
    return round(float(x), 3)


def cm_of(world, h):
    return tuple(f3(v) for v in np.array(tuple(world.rigid_bodies[h].translation),
                                         float))


# ---------------------------------------------------------------- 1. 静置

def step_world():
    """平地 + 1 格高台阶(x=3..5、y=0,长 z=-3..3)。"""
    w = flat_ground(41, 41)
    w.fill([(x, 0, z) for x in (3, 4, 5) for z in range(-3, 4)], __import__("common").STONE)
    return w


def rest_case(cells, mode, surface="flat", drop_h=0.5, ticks=240):
    """把 cells 复合体在表面上方 drop_h 处零初速生成,静置。
    surface=flat:平地(x=0 柱);edge:台阶边缘(一半悬空,质心 x=3)。"""
    w = step_world() if surface == "edge" else flat_ground(41, 41)
    world = r.PhysicsWorld((0.0, -G, 0.0))
    th, cof = slab_terrain_mode(world, w, mode)
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    zs = [c[2] for c in cells]
    ox = (min(xs) + max(xs)) / 2.0 if surface == "edge" else float(np.mean(xs))
    oz = float(np.mean(zs))
    base_y = 1.45 if surface == "edge" else 0.55   # 台阶顶面 1.075 / 平地顶面 0.075
    # spawn_cells 的原点 = 各格质心均值取整到格心 → y 不可调;直接重建刚体:
    import common
    offsets = [(c[0] - xs[0], c[1] - min(ys), c[2] - zs[0]) for c in cells]
    pos = (ox + 0.5 + (np.mean(xs) - xs[0] - (np.mean(xs) - xs[0])), 0, 0)
    # 用 F00 spawn_cells 生成后整体平移不支持;改为:原点放在格心,
    # y 通过"生成高度 = 目标面高 + drop_h"用平移量实现——spawn_cells 只能
    # 格心原点,故这里用 add_compound(自由平移):
    h = common.add_compound(world, cells, (0.0, 0.0, 0.0),
                            face_x0=max(xs) + 1.0,
                            y_cm=float(np.mean(ys)) + 0.5 + drop_h
                            + (1.0 if surface == "edge" else 0.0),
                            z_cm=oz + 0.5)
    tr = run_plain(world, [h], cof, ticks, track=h)
    fy = sum(t[3] for t in tr[-20:]) / 20.0
    return {"cm": cm_of(world, h), "y": round(fy, 4),
            "rest": rest_tick_of(tr)}


def sec1_rest():
    import common
    print("=" * 78)
    print("1. 静置:单格 / 6格条(-3..2,0,0)/ 2×2×2,平地与台阶边缘(一半悬空)")
    print("   静止判定 = 其后 30 拍速率(线+角×0.45)< 0.02;高度 = 末 20 拍质心 y 均值")
    shapes = (("单格", [(0, 0, 0)]),
              ("6格条", [(-3 + i, 0, 0) for i in range(6)]),
              ("2×2×2", [(0 + i, 0 + j, 0 + k) for i in (0, 1)
                         for j in (0, 1) for k in (0, 1)]))
    for surface in ("flat", "edge"):
        print(f"\n--- {'平地' if surface == 'flat' else '台阶边缘(一半悬空)'} ---")
        print("口径,形状,终态质心,最终 y,静止拍")
        for name, cells in shapes:
            for mode in MODES:
                r_ = rest_case(cells, mode, surface=surface)
                print(f"{MODE_CN[mode]},{name},{r_['cm']},{r_['y']:.4f},"
                      f"{r_['rest']}")


# ---------------------------------------------------------------- 2. 滑行

def slide_case(cells, vel, mode, start=(0.0, 1.4, 0.5), ticks=120):
    """给复合体初速 vel,在平地/台阶上滑行;返回 (末速率, 竖直最大跳动, 静止拍)。
    start = 质心起点(台阶上沿滑行用 y=2.4、x=2.5 起向 +x;下沿 y=1.4)。"""
    w = step_world()
    world = r.PhysicsWorld((0.0, -G, 0.0))
    th, cof = slab_terrain_mode(world, w, mode)
    import common
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    zs = [c[2] for c in cells]
    h = common.add_compound(world, cells, vel, face_x0=max(xs) + 1.0,
                            y_cm=start[1], z_cm=start[2])
    tr = run_plain(world, [h], cof, ticks, track=h)
    v_end = float(np.linalg.norm(np.array(tuple(world.rigid_bodies[h].linvel),
                                        float)))
    cm_y = [t[3] for t in tr if not math.isnan(t[3])]
    bounce = max(cm_y) - min(cm_y) if cm_y else float("nan")
    return {"v_end": round(v_end, 3), "bounce": round(bounce, 4),
            "rest": rest_tick_of(tr), "cm_end": cm_of(world, h)}


def sec2_slide():
    import common
    print()
    print("=" * 78)
    print("2. 滑行 120 拍(摩擦 0.6):速度损失、竖直最大跳动(幽灵碰撞检查)")
    print("   平地起点 x=0;台阶上沿:质心 (2.5, 2.4) 向 +x;台阶下沿(贴台阶根):x 向 +x")
    bar = [(-3 + i, 0, 0) for i in range(6)]
    cases = [("单格 沿x 2", [(0, 0, 0)], (2.0, 0.0, 0.0), (0.0, 1.4, 0.5)),
             ("单格 沿x 5", [(0, 0, 0)], (5.0, 0.0, 0.0), (0.0, 1.4, 0.5)),
             ("6格条 沿x 2", bar, (2.0, 0.0, 0.0), (0.0, 1.4, 0.5)),
             ("6格条 沿x 5", bar, (5.0, 0.0, 0.0), (0.0, 1.4, 0.5)),
             ("单格 对角 2", [(0, 0, 0)], (2 ** 0.5, 0.0, 2 ** 0.5), (0.0, 1.4, 0.5)),
             ("单格 对角 5", [(0, 0, 0)], (5 ** 0.5, 0.0, 5 ** 0.5), (0.0, 1.4, 0.5)),
             ("6格条 台阶上沿 5", bar, (5.0, 0.0, 0.0), (2.5, 2.4, 0.5)),
             ("6格条 台阶下沿 5", bar, (5.0, 0.0, 0.0), (-0.5, 1.4, 0.5))]
    print("口径,工况,末速率,竖直最大跳动,静止拍,终态质心")
    for mode in MODES:
        for name, cells, vel, start in cases:
            r_ = slide_case(cells, vel, mode, start=start)
            print(f"{MODE_CN[mode]},{name},{r_['v_end']},{r_['bounce']},"
                  f"{r_['rest']},{r_['cm_end']}")


# ---------------------------------------------------------------- 3. 贴墙与边角

def corner_world():
    """凸边与内角测试场:一面墙 x=0(y=0..2、z=-2..2)+ 地面。凸边 = 墙顶棱;
    内角 = 墙脚地面。"""
    import common
    w = flat_ground(41, 41)
    w.fill([(0, y, z) for y in range(3) for z in range(-2, 3)], common.STONE)
    return w


def drop_case(world_w, pos, mode, ticks=180, vel=(0.0, 0.0, 0.0)):
    """单格在 pos(质心世界坐标)以 vel 生成,自由运动;返回轨迹摘要。"""
    w = world_w
    world = r.PhysicsWorld((0.0, -G, 0.0))
    th, cof = slab_terrain_mode(world, w, mode)
    import common
    b = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
         .translation(tuple(float(x) for x in pos))
         .linvel(tuple(float(x) for x in vel)))
    h = world.add_body(b, colliders=[r.Collider.cuboid(CELL_HALF, CELL_HALF,
                                                       CELL_HALF)
                                     .density(1.0).friction(0.6).restitution(0.0)])
    tr = run_plain(world, [h], cof, ticks, track=h)
    return {"cm": cm_of(world, h),
            "y": round(sum(t[3] for t in tr[-20:]) / 20.0, 4),
            "rest": rest_tick_of(tr),
            "max_v": round(max(t[1] for t in tr), 2)}


def sec3_corner():
    print()
    print("=" * 78)
    print("3. 贴墙与边角(单格,墙 x=0、y=0..2、z=-2..2)")
    print("   贴墙 = 质心 (0.975, 4, 0.5) 紧贴墙面下落;凸边 = (0.975, 3.4, 0.5) 落墙顶棱;"
          "内角 = (0.975, 2, 0.5) 落墙脚地面;判卡住 = 静止时未落到地面/未贴墙就停")
    w = corner_world()
    print("口径,工况,终态质心,最终 y,静止拍,过程最大速率")
    for mode in MODES:
        for name, pos in (("贴墙下落", (0.975, 4.0, 0.5)),
                          ("落凸边", (0.975, 3.4, 0.5)),
                          ("落内角", (0.975, 2.0, 0.5))):
            r_ = drop_case(w, pos, mode)
            print(f"{MODE_CN[mode]},{name},{r_['cm']},{r_['y']:.4f},"
                  f"{r_['rest']},{r_['max_v']}")


# ---------------------------------------------------------------- 4. 接触归格

def retract_stats(mode, drop_v=5.0, ticks=30):
    """对齐撞(竖落 5 m/s 撞地)+ 齐平侧撞(5 m/s 撞墙面),统计退 0.1 归格:
    被触薄片自己的格(cell_of)vs 生产口径退 RETRACT 得到的格。
    返回 (对齐撞: n_ok/n_pts, 侧撞: n_ok/n_pts, 样本)。"""
    import common
    w = corner_world()
    world = r.PhysicsWorld((0.0, -G, 0.0))
    th, cof = slab_terrain_mode(world, w, mode)
    handles = []
    b = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
         .translation((0.5, 4.0, 0.5)).linvel((0.0, -drop_v, 0.0)))
    h1 = world.add_body(b, colliders=[r.Collider.cuboid(CELL_HALF, CELL_HALF,
                                                        CELL_HALF)
                                      .density(1.0).friction(0.6).restitution(0.0)])
    handles.append(h1)
    b2 = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
          .translation((-2.0, 1.5, 0.5)).linvel((drop_v, 0.0, 0.0)))
    h2 = world.add_body(b2, colliders=[r.Collider.cuboid(CELL_HALF, CELL_HALF,
                                                         CELL_HALF)
                                       .density(1.0).friction(0.6).restitution(0.0)])
    handles.append(h2)
    stats = {}
    sample = {}
    for t in range(1, ticks + 1):
        world.step()
        pts = sample_points_raw(world, cof, set(handles))
        for truth, cell_r, imp, pw in pts:
            key = "drop" if truth[0] == 0 or True else "?"
            # 区分工况:竖落块的接触点 y≈0(地面)或 x≈0(墙面);
            # 侧撞块接触点在墙 +x 面(x≈0.075/0)。
            kind = "side" if abs(pw[1] - 0.5) < 0.6 and truth[1] >= 0 \
                else "drop"
            if truth[1] >= 0 and abs(pw[0]) < 0.3 and abs(pw[1] - 0.5) < 0.6:
                kind = "side"
            elif truth[1] < 0 or pw[1] < 0.2:
                kind = "drop"
            ok = cell_r == truth
            n, k = stats.get(kind, (0, 0))
            stats[kind] = (n + (1 if ok else 0), k + 1)
            if kind not in sample:
                sample[kind] = (truth, cell_r, round(imp, 3),
                                tuple(round(v, 3) for v in pw))
    return stats, sample


def retract_scan(mode):
    """退距扫描:对侧撞工况,退距 0.05..0.30 的归格正确率。"""
    import common
    w = corner_world()
    out = []
    for retract in (0.05, 0.1, 0.15, 0.2, 0.25, 0.3):
        world = r.PhysicsWorld((0.0, -G, 0.0))
        th, cof = slab_terrain_mode(world, w, mode)
        b2 = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
              .translation((-2.0, 1.5, 0.5)).linvel((5.0, 0.0, 0.0)))
        h2 = world.add_body(b2, colliders=[r.Collider.cuboid(CELL_HALF, CELL_HALF,
                                                             CELL_HALF)
                                           .density(1.0).friction(0.6)
                                           .restitution(0.0)])
        n_ok = n = 0
        for t in range(1, 30):
            world.step()
            for pair in world.narrow_phase.contact_pairs():
                c1, c2 = pair.collider1, pair.collider2
                p1 = world.colliders.get(c1).parent
                p2 = world.colliders.get(c2).parent
                if c1 in cof and p2 == h2:
                    terrain_c, n_f, p_f = c1, "local_n1", "local_p1"
                elif c2 in cof and p1 == h2:
                    terrain_c, n_f, p_f = c2, "local_n2", "local_p2"
                else:
                    continue
                tp0 = np.array(tuple(world.colliders.get(terrain_c).translation),
                               float)
                for mf in pair.manifolds:
                    n_ = np.array(tuple(getattr(mf, n_f)), float)
                    for pt in mf.points:
                        if pt.impulse <= 0:
                            continue
                        pw = tp0 + np.array(tuple(getattr(pt, p_f)), float)
                        q = pw - n_ * retract
                        cell_r = (int(math.floor(q[0])), int(math.floor(q[1])),
                                  int(math.floor(q[2])))
                        n += 1
                        n_ok += int(cell_r == cof[terrain_c])
        out.append((retract, n_ok, n))
    return out


def sec4_retract():
    print()
    print("=" * 78)
    print("4. 接触归格:生产口径 = 接触点沿法向退 0.1 m 取格(sim.rs:2961)")
    print("   正确 = 归到被触薄片自己的格;工况:竖落撞地面(对齐撞)与横撞墙面(齐平侧撞)")
    for mode in MODES:
        stats, sample = retract_stats(mode)
        row = ", ".join(f"{k}:{v[0]}/{v[1]}" for k, v in sorted(stats.items()))
        print(f"{MODE_CN[mode]}: 正确/总点 {row};样本 {sample}")
    print("\n   退距扫描(侧撞工况,正确率随退距):")
    for mode in MODES:
        rows = retract_scan(mode)
        print(f"{MODE_CN[mode]}: " + ", ".join(f"{r[0]}:{r[1]}/{r[2]}"
                                               for r in rows))


# ---------------------------------------------------------------- 5. impact-corner

def ic_flow_check():
    """flow 纯函数核对:投射物供给记在拐角上格 (8,2,0),d̂ = normalize(-40,-7,0)。"""
    import common
    from common import if_common as IF
    w = flat_ground(41, 41)
    w.fill([(8, 0, 0), (8, 1, 0)], common.STONE)
    w.fill([(x, 2, 0) for x in (5, 6, 7, 8)], common.STONE)
    v = math.hypot(40.0, 7.0)
    dhat = (-40.0 / v, -7.0 / v, 0.0)
    s = CELL_MASS * v
    fl = IF.ImpulseFlow(w, max_cells=1024)
    rows = []
    for hit in [(8, 2, 0), (8, 1, 0)]:
        res = fl.evaluate({hit: s}, dhat)
        rows.append((hit, round(s, 2), res["C"], len(res["S"]),
                     sorted(res["S"]), res["algos_agree"]))
    return rows


def ic_pipeline(mode, ticks=700):
    """完整管线:场景口径投射物 (14,2.6,0.2)、vel=[-40,-7,0](世界 -57.4 ↔ 局部 2.6)。"""
    import common
    w = flat_ground(41, 41)
    w.fill([(8, 0, 0), (8, 1, 0)], common.STONE)
    w.fill([(x, 2, 0) for x in (5, 6, 7, 8)], common.STONE)
    pl = WatchPipeline(w, mode=mode)
    h = pl.add_projectile([(14.0, 2.6, 0.2)], (-40.0, -7.0, 0.0))
    pl.watch = h
    frames = []
    for t in range(1, ticks + 1):
        pl.process_tick(t)
        if t in (7, 8, 9, 10, 12, 15, 20, 30, 60, 120):
            tagged = {"P": [(c[0], c[1]) for c in cells_of_body(pl.world, h)],
                      "S": [(c[0], c[1]) for hh in
                            sorted(pl.dyn_handles,
                                   key=lambda x: (x.index, x.generation))[1:]
                            for c in cells_of_body(pl.world, hh)]}
            frames.append((t, tagged))
    hnd = sorted(pl.dyn_handles, key=lambda x: (x.index, x.generation))
    info = []
    for hh in hnd:
        b = pl.world.rigid_bodies[hh]
        info.append({"pos": cm_of(pl.world, hh),
                     "sleep": bool(b.is_sleeping), "mass": round(b.mass, 3)})
    evs = [(ev["tick"], round(ev["s"], 2), round(ev["C"], 2),
            sorted(ev["S_cells"]), round(ev["w"], 2))
           for ev in pl.events
           if not ev.get("held") and not ev.get("prefiltered")]
    return info, evs, frames


def sec5_impact_corner():
    import common
    print()
    print("=" * 78)
    print("5. impact-corner 预演(正确几何:Δy=+60、Δz=-20)")
    print("   平移核对:世界柱 (8,-60..-59,20) → 局部 (8,0..1,0);"
          "门楣 (5..8,-58,20) → 局部 (5..8,2,0);"
          "投射物 spawn 世界 (14,-57.4,20.2) → 局部 (14.0,2.6,0.2)")
    print("\n   5a. flow 纯函数(s = 0.729×√(40²+7²),供给记在拐角格):")
    print("   命中格,供给,最小割 C,|S|,S,ek=dinic")
    for hit, s, C, nS, S, agree in ic_flow_check():
        print(f"   {hit},{s},{C},{nS},{S},{agree}")
    print("\n   5b. 完整管线(700 拍):两种口径的事件、终态与轨迹")
    for mode in MODES:
        info, evs, frames = ic_pipeline(mode)
        print(f"   --- {MODE_CN[mode]} ---")
        for e in evs:
            print(f"   事件 t={e[0]} s={e[1]} C={e[2]} |S|={len(e[3])} "
                  f"S={e[3]} w={e[4]}")
        for i, d in enumerate(info):
            print(f"   终态 body{i}: pos={d['pos']} sleeping={d['sleep']} "
                  f"mass={d['mass']}")
        print("   轨迹(P=投射物 S=7字,x–y 剖面 @z≈0):")
        for t, tagged in frames:
            print(side_ascii(tagged, t, xr=(2, 16), yr=(-1, 4)))


# ---------------------------------------------------------------- 主流程

def main():
    print("rest.py — F00b 静置、滑行与接触归格"
          "(薄片 straddle=跨面现状 / inset=内缩 D15)")
    sec1_rest()
    sec2_slide()
    sec3_corner()
    sec4_retract()
    sec5_impact_corner()


if __name__ == "__main__":
    main()
