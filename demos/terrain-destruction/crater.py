"""crater.py — REQ-009 任务 00 脚本 1:爆炸弹坑(纯体素)。

对比球形(P = power·(1−d/r))与射线衰减(每穿过一个实心格减 R×attenuation,
Amanatides–Woo 遍历、不含目标格自身)两种压力模型;判定 P>R 破坏、P>k·R 粉碎;
扫描 power/radius/dirt R/stone R/k/attenuation 全组合(648 组 × 2 模型 × 3 场景)。

场景:① 超平坦;② 石墙(长 15、高 6,厚 1 与厚 3,距爆心 3 m);③ 20 层石头地基。
爆心 (0, 0.5, 0)。输出每组破坏/脱落/粉碎数、坑深/直径、墙后破坏数、求值访问格数,
以及代表性配置的 x–y 剖面 ASCII 图与设计意图核对。

运行:.venv/Scripts/python.exe demos/terrain-destruction/crater.py
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import AIR, BEDROCK, DIRT, INF_R, STONE
from common import BlastField, render_slice

POWERS = [6, 12, 24, 48]
RADII = [4, 8, 12]
DIRT_RS = [2.0, 4.0, 6.0]
STONE_RS = [6.0, 10.0, 15.0]
KS = [2.0, 3.0, 4.0]
ATTS = [0.5, 1.0]
BLAST = (0.0, 0.5, 0.0)
REP = dict(power=12, radius=8, dR=4.0, sR=10.0, k=3.0, att=1.0)  # 需求 §3.2 占位值


def build_flat():
    """① 超平坦:y=-4 bedrock,y=-3..-1 dirt。"""
    vox = np.zeros((32, 8, 32), np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:4, :] = DIRT
    return vox, -16, -4, -16


def build_wall(t):
    """② 石墙:x=3..3+t-1(厚 t),z=-7..7(长 15),y=0..5(高 6)。"""
    vox = np.zeros((31, 12, 19), np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:4, :] = DIRT
    vox[3 + 13:3 + t + 13, 4:10, 2:17] = STONE
    return vox, -13, -4, -9


def build_deep():
    """③ 20 层石头地基:y=-21 bedrock,y=-20..-1 stone。"""
    vox = np.zeros((32, 25, 32), np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:, :] = STONE
    return vox, -16, -21, -16


def wall_stats(field, res, t):
    """墙面(第 1 层 x=3)、墙身总破坏(x∈[3,3+t) 且在墙的 y/z 范围内)、墙后破坏(x ≥ 3+t)。

    墙体范围:y=0..5,z=-7..7;x<3+t 之外的破坏(含墙下地基)不计入墙身。
    """
    face = wall_total = behind = 0
    for (ix, iy, iz), _ in res["cells"].items():
        wx, wy, wz = ix + field.x0, iy + field.y0, iz + field.z0
        if 3 <= wx < 3 + t:
            if 0 <= wy <= 5 and -7 <= wz <= 7:
                wall_total += 1
                if wx == 3:
                    face += 1
        elif wx >= 3 + t:
            behind += 1
    return face, wall_total, behind


def main():
    scen = {}
    scen["A-flat"] = (*build_flat(), -1)
    scen["B-wall1"] = (*build_wall(1), -1)
    scen["B-wall3"] = (*build_wall(3), -1)
    scen["C-deep"] = (*build_deep(), -1)
    fields = {}
    for name, (vox, x0, y0, z0, surf) in scen.items():
        t0 = time.perf_counter()
        fields[name] = BlastField(vox, x0, y0, z0, BLAST, rmax=max(RADII), surface_y=surf)
        f = fields[name]
        print(f"# 预计算 {name}: 目标格 {len(f.dists)}, 访问格数 r=4/8/12 = "
              f"{f.visited_by_r[4]}/{f.visited_by_r[8]}/{f.visited_by_r[12]}, "
              f"耗时 {time.perf_counter()-t0:.2f}s")

    all_rows = {}
    hdr = ("model,power,radius,dirtR,stoneR,k,att,destroyed,detached,pulverized,"
           "depth,diameter,behind,wall_destroyed,visited,targets,ray_steps")
    for name, (vox, x0, y0, z0, surf) in scen.items():
        field = fields[name]
        print(f"\n===== 场景 {name} =====")
        print(hdr)
        t0 = time.perf_counter()
        for model in ("sphere", "ray"):
            for power in POWERS:
                for radius in RADII:
                    for dR in DIRT_RS:
                        for sR in STONE_RS:
                            for k in KS:
                                for att in ATTS:
                                    resd = {AIR: 0.0, STONE: sR, DIRT: dR, BEDROCK: INF_R}
                                    m = field.evaluate(resd, power, radius, k, att, model)
                                    if name.startswith("B-wall"):
                                        t_ = int(name[-1])
                                        face, wtot, behind = wall_stats(field, m, t_)
                                    else:
                                        face = wtot = behind = 0
                                    print(f"{model},{power},{radius},{dR:g},{sR:g},{k:g},{att:g},"
                                          f"{m['destroyed']},{m['detached']},{m['pulverized']},"
                                          f"{m['depth']},{m['diameter']},{behind},{wtot},"
                                          f"{m['visited']},{m['targets']},{m['ray_steps']}")
                                    all_rows[(model, power, radius, dR, sR, k, att, name)] = \
                                        (m, face, wtot, behind)
        print(f"# 场景 {name} 扫描耗时 {time.perf_counter()-t0:.1f}s")

    def get(model, power, radius, dR, sR, k, att, name):
        return all_rows[(model, power, radius, dR, sR, k, att, name)]

    print("\n===== 需求 §3.2 占位值基线(power 12, radius 8, dirt R=4, stone R=10, k=3, att=1.0) =====")
    for model in ("sphere", "ray"):
        for name in ("A-flat", "B-wall1", "B-wall3", "C-deep"):
            m, face, wtot, behind = get(model, 12, 8, 4.0, 10.0, 3.0, 1.0, name)
            print(f"{model:6s} {name}: 破坏 {m['destroyed']}(脱落 {m['detached']} / 粉碎 "
                  f"{m['pulverized']}),坑深 {m['depth']},直径 {m['diameter']},"
                  f"墙面/墙身/墙后 = {face}/{wtot}/{behind},"
                  f"访问 {m['visited']} 格 + 射线 {m['ray_steps']} 步")

    print("\n===== 设计意图核对(射线模型,power12 radius8;A=破坏≥15且10≤脱落≤60,"
          "B=厚1墙墙后≤2,C=厚3墙只炸朝爆心一面) =====")
    print("dirtR   k  att | A 破坏/脱落  | B(按 stoneR)墙后    | C(按 stoneR)面/墙/后")
    for dR in DIRT_RS:
        for k in KS:
            for att in ATTS:
                mA, _, _, _ = get("ray", 12, 8, dR, 10.0, k, att, "A-flat")
                a_ok = mA["destroyed"] >= 15 and 10 <= mA["detached"] <= 60
                b_cells, c_cells = [], []
                for sR in STONE_RS:
                    _, _, _, b1 = get("ray", 12, 8, dR, sR, k, att, "B-wall1")
                    _, f3, w3, b3 = get("ray", 12, 8, dR, sR, k, att, "B-wall3")
                    okc = b3 == 0 and f3 >= 1 and w3 == f3
                    b_cells.append(f"R{sR:g}:{b1}")
                    c_cells.append(f"R{sR:g}:{f3}/{w3}/{b3}{'✓' if okc else '✗'}")
                print(f"{dR:>5g} {k:>3g} {att:>4g} | {mA['destroyed']:>5}/{mA['detached']:<5}"
                      f"{'✓' if a_ok else '✗'} | {' '.join(b_cells):>18} | {' '.join(c_cells)}")

    ray_keys = [k_ for k_ in all_rows if k_[0] == "ray" and k_[7] == "A-flat"]
    full = []
    for key in ray_keys:
        _, power, radius, dR, sR, k, att, _ = key
        _, _, _, b1 = get("ray", power, radius, dR, sR, k, att, "B-wall1")
        _, f3, w3, b3 = get("ray", power, radius, dR, sR, k, att, "B-wall3")
        mA, _, _, _ = get("ray", power, radius, dR, sR, k, att, "A-flat")
        okA = mA["destroyed"] >= 15 and 10 <= mA["detached"] <= 60
        okB = b1 <= 2
        okC = b3 == 0 and f3 >= 1 and w3 == f3
        if okA and okB and okC:
            full.append((power, radius, dR, sR, k, att))
    print(f"\n三意图同时满足的组合数:{len(full)} / {len(ray_keys)}(射线模型全扫描)")
    for row in full:
        print("  ray power=%g radius=%g dirtR=%g stoneR=%g k=%g att=%g" % row)

    print("\n===== 代表性配置 x–y 剖面(过 z=0;o=脱落 x=粉碎 ==dirt #stone B=bedrock) =====")
    for model in ("ray", "sphere"):
        for name, (vox, x0, y0, z0, surf) in scen.items():
            if name not in ("A-flat", "B-wall1"):
                continue
            m = get(model, REP["power"], REP["radius"], REP["dR"], REP["sR"],
                    REP["k"], REP["att"], name)[0]
            top = y0 + vox.shape[1] - 1
            print(f"\n-- {model} / {name} (power12 radius8 dirtR4 stoneR10 k3 att1.0) --")
            print(render_slice(vox, x0, y0, z0, m["cells"], 0, top, -5))


if __name__ == "__main__":
    main()
