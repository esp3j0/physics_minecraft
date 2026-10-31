"""passage.py — REQ-010 单元 F00b 脚本 1:穿洞与落坑(薄片内缩的收益,D15)。

每个工况在两种地形薄片口径下并排跑、并排打印:
- straddle = 跨面(现状,生产口径:薄片中心在面平面上、向空气侧伸出 0.075);
- inset    = 内缩(D15:薄片整体缩进实心格,外表面恰在格面上)。

1. 穿墙:6 格石条(4.374 kg,沿运动方向纵置)与 2×2 截面复合体(8 格,5.832 kg)
   以 10/15/20 m/s 撞 1 格厚土墙(w7h5,立在 bedrock),F00 momentum.py 工况 1 的
   原生口径(起点 (-8,2,0) 行、无 y 对准;撞击瞬间的质心 y 打印出来核对仍跨
   (·,2,·) 行)——是否穿过(前缘越过墙后表面 x=1)、断格数、事件序列。
   另加"正对一格"的对照:起点反推到撞击瞬间质心恰在格心高度(打印核对),
   15 m/s 6 格条。
2. 落坑:单格碎块(纯物理,无撞击判定)落进 1 格宽、1/2 格深的坑,水平偏心
   0/0.02/0.05/0.1,初速 0 与竖直 5 m/s——最终 y、静止拍数、静止前接触冲量
   合计的峰值、是否落到坑底。
3. 修订 8 坑内工况(F00 momentum.case5_pit 构造,300 拍):旧规则(无接近速度
   过滤、碎块初速 (J−I)·d̂)的脱落格数——"不靠过滤,只靠几何能不能消除连锁",
   并打印逐事件(拍、格、冲量、I)。

运行:.venv/Scripts/python.exe demos/inset-slab/passage.py   (~6 分钟)
GBK 控制台:先 set PYTHONIOENCODING=utf-8。
"""
import math
import os
import sys

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (AIR, BEDROCK, CELL_MASS, DIRT, DT, G, I_OF, MODES, MODE_CN,
                    WatchPipeline, add_projectile, flat_ground, rest_tick_of,
                    run_plain, sample_contacts_unfiltered, slab_terrain_mode,
                    spawn_cells)

# import 顺序:passage 依赖 common;common 里 G 常量在下方定义(避免循环)
try:
    G  # noqa: F821
except NameError:
    G = 9.81


# ---------------------------------------------------------------- 1. 穿墙

def wall_world(width=7, height=5):
    """F00 momentum.wall_world 同款:1 格厚墙 x=0、y=0..h-1、z=-(w//2)..(w//2),
    脚下 y=-1..-3 换 bedrock;地表 y=0。"""
    w = flat_ground(41, 41)
    w.fill([(0, -y, z) for y in (1, 2, 3)
            for z in range(-(width // 2), width // 2 + 1)], BEDROCK)
    w.fill([(0, y, z) for y in range(height)
            for z in range(-(width // 2), width // 2 + 1)], DIRT)
    return w


def bar_cells():
    """6 格石条,沿 x 纵置 (-8..-3,2,0)。"""
    return [(-8 + i, 2, 0) for i in range(6)]


def q22_cells():
    """2×2 截面复合体(8 格深 1):x=-9..-8、y=2..3、z=0..1。"""
    return [(-9 + i, 2 + j, k) for i in range(2) for j in range(2)
            for k in range(2)]


def f3(x):
    """np.float64 → 原生 float,保留 3 位(打印用)。"""
    return round(float(x), 3)


def wall_case(cells, speed, mode, aimed=False, ticks=200):
    """横撞 1 格厚土墙;返回终态摘要。
    aimed=False:F00 momentum 工况 1 原生口径(起点 (-8,2,0) 行格心)。
    aimed=True:起点反推——撞击瞬间质心 y 回到 2.5(重力补偿 ½g·t²)、
    前缘在第 t 拍接触且穿深 0.05(打印撞击瞬间质心核对)。"""
    w = wall_world()
    pl = WatchPipeline(w, mode=mode)
    ycomp = 0.0
    if aimed:
        gap = 0.925 if mode == "straddle" else 1.0
        t_c = max(1, math.ceil(gap / (speed * DT) - 1e-9))
        ycomp = 0.5 * G * (t_c * DT) ** 2
        face_x0 = (-0.075 if mode == "straddle" else 0.0) \
            - (t_c * speed * DT - 0.05)
        import common
        h = common.add_compound(pl.world, cells, (speed, 0.0, 0.0),
                                face_x0=face_x0, y_cm=2.5 + ycomp, z_cm=0.5)
    else:
        h = add_projectile(pl.world, None, (speed, 0.0, 0.0), cells=cells)[0]
    pl.dyn_handles.add(h)
    pl.watch = h
    pl.run(ticks)
    b = pl.world.rigid_bodies[h]
    cm = np.array(tuple(b.translation), float)
    vx = float(np.array(tuple(b.linvel), float)[0])
    lead = max(c[0] for c in cells) - np.mean([c[0] for c in cells])
    front_end = float(cm[0] + lead + 0.45)       # 终态前缘(+x 面)
    # 撞击瞬间:前缘首次越过墙前表面 +0.1
    face_now = -0.075 if mode == "straddle" else 0.0
    impact_cm = None
    for t, p in pl.trace:
        if float(p[0]) + lead + 0.45 > face_now + 0.1:
            impact_cm = (t, tuple(f3(v) for v in p))
            break
    if impact_cm is None and pl.trace:
        impact_cm = (pl.trace[0][0], tuple(f3(v) for v in pl.trace[0][1]))
    seq = [(t, round(d["s"], 1), round(d["C"], 1), len(d["S_cells"]),
            sorted(d["S_cells"]))
           for t, d in ((ev["tick"], ev) for ev in pl.events)
           if not d.get("held") and not d.get("prefiltered")]
    held_seq = [ev["tick"] for ev in pl.events
                if ev.get("held") and not ev.get("degenerate_cut")]
    return {
        "mode": mode,
        "impact_cm": impact_cm,
        "end_cm": tuple(f3(v) for v in cm),
        "front_end": round(front_end, 3),
        "vx_end": round(vx, 3),
        "penetrated": bool(front_end > 1.0),
        "n_events": len(pl.events),
        "held_ticks": held_seq,
        "seq": seq,
        "detached": pl.detach_total,
    }


def sec1_wall():
    print("=" * 78)
    print("1. 穿墙:撞 1 格厚土墙 w7h5(立在 bedrock;两种薄片口径)")
    print("   穿墙判定 = 终态前缘(最前格 +x 面)> 1.0(墙后表面);")
    print("   事件列 = (拍, s, C, |S|, S格);原生口径 = F00 momentum 工况 1 起点")
    for label, cells in (("6格石条 4.374kg", bar_cells()),
                         ("2×2复合体 5.832kg", q22_cells())):
        print(f"\n--- {label} ---")
        print("口径,速度,撞击瞬间(拍,质心),终态质心,终态前缘,终态vx,"
              "前缘越面?,事件数,断格数,事件列,扛住拍")
        for speed in (10.0, 15.0, 20.0):
            for mode in MODES:
                r_ = wall_case(cells, speed, mode)
                print(f"{MODE_CN[mode]},{speed:4.0f},{r_['impact_cm']},"
                      f"{r_['end_cm']},{r_['front_end']},{r_['vx_end']:+.2f},"
                      f"{'是' if r_['penetrated'] else '否'},"
                      f"{r_['n_events']},{r_['detached']},{r_['seq']},"
                      f"{r_['held_ticks']}")
    print("\n--- 对照:6格石条 15 m/s 正对一格(撞击瞬间质心 y=2.5)---")
    print("口径,撞击瞬间(拍,质心),终态质心,终态前缘,前缘越面?,事件数,断格数,事件列")
    for mode in MODES:
        r_ = wall_case(bar_cells(), 15.0, mode, aimed=True)
        print(f"{MODE_CN[mode]},{r_['impact_cm']},{r_['end_cm']},"
              f"{r_['front_end']},{'是' if r_['penetrated'] else '否'},"
              f"{r_['n_events']},{r_['detached']},{r_['seq']}")


# ---------------------------------------------------------------- 2. 落坑

def pit_world(depth):
    """平地挖 1 格宽的坑:x=1、y=-1..-depth、z=0 为 air(修订 8 工况同款)。"""
    w = flat_ground(41, 41)
    for y in range(-1, -depth - 1, -1):
        w.set_air((1, y, 0))
    return w


def pit_case(depth, off, v0, mode, ticks=240):
    """单格碎块落坑(纯物理,无撞击判定):格心 x=1+off、y=3、z=0,
    初速 (0,-v0,0)。返回最终 y(末 20 拍均值)、静止拍、静止前接触冲量峰值、
    是否落到坑底(坑底静止质心 y = 坑底格顶 0.45)。"""
    w = pit_world(depth)
    world = r.PhysicsWorld((0.0, -G, 0.0))
    th, cof = slab_terrain_mode(world, w, mode)
    h = spawn_cells(world, [(1, 3, 0)], (0.0, -float(v0), 0.0))
    tr = run_plain(world, [h], cof, ticks, track=h)
    cm = tuple(f3(v) for v in np.array(
        tuple(world.rigid_bodies[h].translation), float))
    rest = rest_tick_of(tr)
    span = tr[:rest] if rest else tr
    peak = max(t[2] for t in span)
    fy = sum(t[3] for t in tr[-20:]) / 20.0
    bottom = -(depth - 1) + 0.45
    return {"final_cm": cm, "y": round(fy, 4),
            "rest": rest, "peak_imp": round(peak, 3),
            "reached": bool(fy < bottom + 0.05)}


def sec2_pit():
    print()
    print("=" * 78)
    print("2. 落坑:单格碎块(0.729 kg)落进 1 格宽的坑(纯物理,无撞击判定)")
    print("   坑 x=1、z=0;碎块起点 (1,3,0) 格心+偏心;坑底静止质心 y:深1=-0.55、深2=-1.55")
    for depth in (1, 2):
        print(f"\n--- 坑深 {depth} 格 ---")
        print("口径,偏心,初速 v_y,终态质心,最终 y,静止拍,静止前冲量峰值,到达坑底?")
        for off in (0.0, 0.02, 0.05, 0.1):
            for v0 in (0.0, 5.0):
                for mode in MODES:
                    r_ = pit_case(depth, off, v0, mode)
                    print(f"{MODE_CN[mode]},{off:.2f},{v0:.0f},"
                          f"{r_['final_cm']},{r_['y']:.4f},{r_['rest']},"
                          f"{r_['peak_imp']:.3f},"
                          f"{'是' if r_['reached'] else '否'}")


# ---------------------------------------------------------------- 3. 修订 8 坑内

def rev8_case(mode, ticks=300):
    """F00 momentum.case5_pit 构造 + 旧规则(无接近速度过滤、
    碎块初速 (J−I)·d̂ 竖直向下):返回 (总脱落格数, 逐事件)。"""
    w = flat_ground(41, 41)
    w.set_air((1, -1, 0))          # 一格宽、两格深的坑
    w.set_air((1, -2, 0))
    pl = WatchPipeline(w, mode=mode)
    h1 = spawn_cells(pl.world, [(1, -1, 0)], (0.0, 0.0, 0.0))
    pl.dyn_handles.add(h1)         # 静止块:不参与撞击判定,但要被看到
    pl.add_projectile([(1, 4, 0)], (0.0, 0.0, 0.0))
    detached = 0
    log = []
    for t in range(1, ticks + 1):
        tb = {hh: (pl.world.rigid_bodies[hh].linvel,
                   pl.world.rigid_bodies[hh].angvel) for hh in pl.dyn_handles}
        pl.world.step()
        imp, J, pts = sample_contacts_unfiltered(pl.world, pl.cell_of,
                                                 pl.dyn_handles, tb, pl.w)
        if not imp:
            continue
        for cell, s_c in sorted(imp.items()):
            mat = pl.w.get(cell)
            if mat in (AIR, None):
                continue
            if s_c >= I_OF[mat]:
                log.append((t, cell, round(s_c, 2), I_OF[mat]))
                pl.w.set_air(cell)
                detached += 1
                v = max(s_c - I_OF[mat], 0.0)
                spawn_cells(pl.world, [cell], (0.0, -min(v, 8.0), 0.0))
                pl.world.remove_body(pl.terrain_h)
                pl.terrain_h, pl.cell_of = slab_terrain_mode(pl.world, pl.w,
                                                             mode)
    return detached, log


def sec3_rev8(ticks=300):
    print()
    print("=" * 78)
    print(f"3. 修订 8 坑内工况({ticks} 拍;F00 momentum.case5_pit 构造):")
    print("   旧规则 = 无接近速度过滤、碎块初速 (J−I)·d̂(修订 8 之前的连锁来源)")
    for mode in MODES:
        detached, log = rev8_case(mode, ticks)
        print(f"\n{MODE_CN[mode]}:总脱落格数 = {detached}(事件 {len(log)} 次)")
        for t, cell, s_c, mat_i in log:
            print(f"   t={t} 格={cell} 冲量={s_c} ≥ I={mat_i}")


# ---------------------------------------------------------------- 主流程

def main():
    print("passage.py — F00b 穿洞与落坑"
          "(薄片 straddle=跨面现状 / inset=内缩 D15)")
    sec1_wall()
    sec2_pit()
    sec3_rev8()


if __name__ == "__main__":
    main()
