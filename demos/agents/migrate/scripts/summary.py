#!/usr/bin/env python3
"""从 out/matrix_submitted.txt(提交数据)计算 README 的总结数字。

A00-3 R2 数据要求(MR1):按完整名字段筛行,不用子串;名字拆成
场景 / 相位 / 家族 / 快照 / 地形 后比较。判据 3 按家族逐个报 same、cross,
cross_hist 与拆分家族单列。
用法:python scripts/summary.py  (在 demos/agents/migrate/ 下运行)
输出:可直接粘贴进 README 的数字清单。
"""
from __future__ import annotations

import csv
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MATRIX = os.path.join(HERE, "out", "matrix_submitted.txt")
DET = os.path.join(HERE, "out", "det_submitted.txt")

# 名字格式:scene[_phase]_family_snap_terrain(scene 可能带后缀段)。
# 完整段比较,不用子串(MR1)。
FAMILIES = ("same", "cross", "cross_hist", "histonly", "pilesleep", "pileawake")
SNAPS = ("full", "novec")
TERRAINS = ("T0", "T1")


def parse_name(name: str) -> dict:
    """把行名拆成字段(完整段匹配,MR1)。家族段可能带下划线
    (cross_hist),从已知家族表匹配;快照段从已知快照表匹配。"""
    out: dict = {"raw": name}
    parts = name.split("_")
    if name.startswith("walk_p") and len(parts) >= 5:
        # walk_pXX_<family 可能两段>_<snap>_<terrain>:家族用已知集合钉住。
        if len(parts) > 3 and parts[2] + "_" + parts[3] in FAMILIES:
            fam = parts[2] + "_" + parts[3]
            rest = parts[4:]
        else:
            fam = parts[2]
            rest = parts[3:]
        out.update(scene="walk", phase=int(parts[1][1:]), family=fam,
                   snap=rest[0], terrain=rest[1])
    elif name.startswith("control_walk_p") and len(parts) >= 4:
        out.update(scene="control_walk", phase=int(parts[2][1:]),
                   terrain=parts[3])
    elif name.startswith(("stand_awake_", "stand_asleep_")) and len(parts) >= 5:
        if len(parts) > 3 and parts[2] + "_" + parts[3] in FAMILIES:
            fam = parts[2] + "_" + parts[3]
            rest = parts[4:]
        else:
            fam = parts[2]
            rest = parts[3:]
        out.update(scene=parts[0] + "_" + parts[1], family=fam,
                   snap=rest[0], terrain=rest[1])
    elif name.startswith("land_") and len(parts) >= 4:
        if len(parts) > 2 and parts[1] + "_" + parts[2] in FAMILIES:
            fam = parts[1] + "_" + parts[2]
            rest = parts[3:]
        else:
            fam = parts[1]
            rest = parts[2:]
        out.update(scene="land", family=fam, snap=rest[0], terrain=rest[1])
    elif name.startswith(("control_stand_awake_", "control_stand_sleep_")):
        out.update(scene="control_" + parts[1] + "_" + parts[2], terrain=parts[-1])
    elif name.startswith("control_land_"):
        out.update(scene="control_land", terrain=parts[-1])
    elif name.startswith("control_pingpong_"):
        out.update(scene="control_pingpong", terrain=parts[-1])
    elif name.startswith("pingpong_g"):
        gap = int(parts[1][1:])
        if len(parts) > 3 and parts[2] + "_" + parts[3] in FAMILIES:
            fam = parts[2] + "_" + parts[3]
            rest = parts[4:]
        else:
            fam = parts[2]
            rest = parts[3:]
        out.update(scene="pingpong", gap=gap, family=fam, snap=rest[0],
                   terrain=rest[1])
    elif name.startswith("order_"):
        out.update(scene="order")
    elif name.startswith("seq_"):
        out.update(scene="seq")
    elif name.startswith("eight_"):
        out.update(scene="eight")
    return out


def load_rows():
    with open(MATRIX, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    # MR8/FR8 同类要求:列头与数据列数一致(防错位)。
    with open(MATRIX, encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
    for r in rows:
        assert len(r) == len(header), f"列数不一致: {r.get('name')} {len(r)} vs {len(header)}"
    return rows, header


def f(v: str) -> float:
    if v in ("-", "", "inf"):
        return float("nan") if v == "inf" else 0.0
    return float(v)


def main() -> int:
    rows, _ = load_rows()
    parsed = [parse_name(r["name"]) for r in rows]
    by = {}
    for r, p in zip(rows, parsed):
        by[p["raw"]] = (r, p)

    def sel(pred):
        return [(r, p) for r, p in zip(rows, parsed) if pred(p)]

    scenes = {}
    for p in parsed:
        scenes[p.get("scene", "?")] = scenes.get(p.get("scene", "?"), 0) + 1
    print("== 行数 ==", "total=" + str(len(rows)),
          " ".join(f"{k}={v}" for k, v in sorted(scenes.items())))

    # 判据 1:full 行重建误差(迁移行;不含 control)
    full_mig = sel(lambda p: p.get("snap") == "full" and not p.get("raw", "").startswith("control"))
    f_err_c = [f(r["err_coord_rad"]) for r, _ in full_mig]
    f_err_p = [f(r["err_pos_m"]) for r, _ in full_mig]
    f_err_r = [f(r["err_rot_rad"]) for r, _ in full_mig]
    print(f"== 判据1 (full 迁移 {len(full_mig)} 行) == max err_coord={max(f_err_c):.3e} "
          f"max err_pos={max(f_err_p):.3e} max err_rot={max(f_err_r):.3e} "
          f"零误差行={sum(1 for c, p in zip(f_err_c, f_err_p) if c == 0.0 and p == 0.0)}")

    # 判据 2:full 迁移行窗口指标(行走 + 站立;land 单独)
    j2 = sel(lambda p: p.get("snap") == "full" and p.get("scene") in
             ("walk", "stand_awake", "stand_asleep"))
    worst_upy = min(f(r["post_upy_min"]) for r, _ in j2)
    worst_vy = max(f(r["post_max_vy"]) for r, _ in j2)
    worst_link = max(f(r["post_link_max"]) for r, _ in j2)
    worst_step = max(f(r["post_body_step_max"]) for r, _ in j2)
    j2c = sel(lambda p: p.get("scene") == "control_walk")
    ctrl_link = max(f(r["post_link_max"]) for r, _ in j2c)
    ctrl_step = max(f(r["post_body_step_max"]) for r, _ in j2c)
    print(f"== 判据2 (full 迁移 {len(j2)} 行) == upy_min={worst_upy:.4f} "
          f"vy_max={worst_vy:.4f} link_max={worst_link:.4f}(对照 {ctrl_link:.4f}) "
          f"body_step_max={worst_step:.4f}(对照 {ctrl_step:.4f})")
    # land 组逐项 vs 对照
    land_mig = sel(lambda p: p.get("scene") == "land" and p.get("snap") == "full")
    land_ctrl = sel(lambda p: p.get("scene") == "control_land")
    if land_mig and land_ctrl:
        lc = land_ctrl[0][0]
        for key in ("post_upy_min", "post_max_vy", "post_link_max", "post_body_step_max"):
            vals = [f(r[key]) for r, _ in land_mig]
            cv = f(lc[key])
            print(f"== land {key} == 迁移 {min(vals):.4f}–{max(vals):.4f} vs 对照 {cv:.4f}")

    # 判据 3:vfwd 相对同相位对照的百分比;家族逐个(MR1:完整字段匹配)
    ctrl_by = {}
    for r, p in sel(lambda p: p.get("scene") == "control_walk"):
        ctrl_by[(p["phase"], p["terrain"])] = f(r["vfwd"])
    print("== 判据3 (单次迁移 vfwd,相对同相位对照;家族逐个) ==")
    for terrain in TERRAINS:
        for family in FAMILIES:
            if family in ("same", "cross"):
                snaps = SNAPS
            else:
                snaps = ("full",)
            for snap in snaps:
                grp = sel(lambda p: p.get("scene") == "walk" and p.get("family") == family
                          and p.get("snap") == snap and p.get("terrain") == terrain)
                if not grp:
                    continue
                pcts, vs = [], []
                for r, p in grp:
                    base = ctrl_by.get((p["phase"], terrain))
                    if base:
                        v = f(r["vfwd"])
                        vs.append(v)
                        pcts.append(v / base * 100)
                if pcts:
                    print(f"  {family}_{snap}_{terrain}: n={len(pcts)} "
                          f"vfwd {min(vs):.4f}–{max(vs):.4f} "
                          f"{min(pcts):.1f}%–{max(pcts):.1f}%")
    # 摔倒 / 穿透
    fell = [r["name"] for r, _ in sel(lambda p: r["fell_tick"] not in ("-", ""))]
    walk_all = sel(lambda p: p.get("scene") == "walk")
    pens = [f(r["post_pen_max"]) for r, _ in walk_all]
    print(f"== 摔倒行 == {fell if fell else '无'}; 行走窗口对地穿透最深 "
          f"{min(pens):.4f} m(全迁移行走行)")
    land_pen = min(f(r["post_pen_max"]) for r, _ in sel(lambda p: p.get("scene") == "land"))
    print(f"== land 穿透 == {land_pen:.4f} m")

    # 判据 4:pingpong
    cpp = by.get("control_pingpong_T0")
    pbase = f(cpp[0]["vfwd"]) if cpp else float("nan")
    for gap in (2, 5, 30, 120):
        key = f"pingpong_g{gap}_same_full_T0"
        if key in by:
            v = f(by[key][0]["vfwd"])
            upy = f(by[key][0]["post_upy_min"])
            print(f"== 判据4 g{gap}: vfwd={v:.4f} ({v / pbase * 100:.1f}% of 对照 "
                  f"{pbase:.4f}) upy_min={upy:.4f}")
    for family in ("cross", "cross_hist"):
        key = f"pingpong_g30_{family}_full_T0"
        if key in by:
            print(f"== 判据4 g30 {family}: vfwd={f(by[key][0]['vfwd']):.4f}")

    # novec 对照(same 家族 only)
    for terrain in TERRAINS:
        bf = [f(r["vfwd"]) for r, p in sel(lambda p: p.get("scene") == "walk"
                                           and p.get("family") == "same"
                                           and p.get("snap") == "full"
                                           and p.get("terrain") == terrain)]
        bn = [f(r["vfwd"]) for r, p in sel(lambda p: p.get("scene") == "walk"
                                           and p.get("family") == "same"
                                           and p.get("snap") == "novec"
                                           and p.get("terrain") == terrain)]
        if bf and bn:
            print(f"== novec {terrain}: full 均值 {sum(bf)/len(bf):.4f} novec 均值 "
                  f"{sum(bn)/len(bn):.4f} = {sum(bn)/sum(bf)*100:.1f}%")

    # diverge(cross 家族 vs same_full 同快照;按完整字段)
    div = []
    for r, p in sel(lambda p: p.get("scene") == "walk"):
        if r["diverge_tick"] not in ("-", ""):
            div.append((p["family"], p["snap"], p["terrain"], int(r["diverge_tick"])))
    for family in FAMILIES:
        d = [t for fa, sn, te, t in div if fa == family and sn == "full"]
        n_fam = len(sel(lambda p: p.get("scene") == "walk" and p.get("family") == family
                        and p.get("snap") == "full"))
        if d or n_fam:
            print(f"== diverge {family}_full == 分流行 {len(d)}/{n_fam}, "
                  f"分歧拍 {min(d) if d else '-'}–{max(d) if d else '-'}")

    # 耗时
    mus = [f(r["migrate_us"]) for r, _ in sel(lambda p: f(r["migrate_us"]) > 0
                                              and p.get("scene") not in ("eight",))]
    print(f"== migrate_us == 单次 {min(mus):.1f}–{max(mus):.1f} µs")

    # MR6:order_* 表(含 yaw_change_deg / path_len_m / slept)
    for r, p in sel(lambda p: p.get("scene") == "order"):
        print(f"== order == {r['name']}: start={r.get('active_order_start', '-')} "
              f"end={r.get('active_order_end', '-')} vfwd={f(r['vfwd']):.4f} "
              f"yaw={f(r['yaw_change_deg']):.1f}° path={f(r['path_len_m']):.2f} m "
              f"slept={r.get('slept', '-')}")
    for r, p in sel(lambda p: p.get("scene") == "seq"):
        print(f"== seq == {r['name']}: pre={r.get('seq_pre_order', '-')} "
              f"post={r.get('seq_post_order', '-')} "
              f"pre_vfwd={f(r['seq_pre_vfwd']):.4f} "
              f"post_vfwd={f(r['seq_post_vfwd']):.4f} "
              f"yaw_rate {f(r['seq_pre_yaw_rate']):.2f}->{f(r['seq_post_yaw_rate']):.2f} °/s "
              f"yaw={f(r['yaw_change_deg']):.1f}° path={f(r['path_len_m']):.2f} m "
              f"slept={r.get('slept', '-')}")

    # MR2/MR3:active_order_post / dst_awake / dst_rewake(跨世界家族逐个)
    for family in ("cross", "cross_hist", "histonly", "pilesleep", "pileawake"):
        grp = sel(lambda p: p.get("scene") == "walk" and p.get("family") == family
                  and p.get("snap") == "full" and p.get("terrain") == "T0")
        if not grp:
            continue
        awakes = {r["dst_awake_at_migrate"] for r, _ in grp}
        rewakes = [r["dst_rewake_tick"] for r, _ in grp if r["dst_rewake_tick"] not in ("-",)]
        posts = {r["active_order_post"] for r, _ in grp}
        print(f"== MR2/MR3 {family}_walk_T0 == dst_awake 集合 {sorted(awakes)} "
              f"rewake {sorted(set(rewakes)) if rewakes else '无'} "
              f"post_order 种类 {len(posts)}(示例 {sorted(posts)[:2]})")

    # eight
    for r, p in sel(lambda p: p.get("scene") == "eight"):
        print(f"== eight == {r['name']}: upy={f(r['post_upy_min']):.4f} "
              f"vy={f(r['post_max_vy']):.4f} link={f(r['post_link_max']):.4f} "
              f"step={f(r['post_body_step_max']):.4f} vfwd={f(r['vfwd']):.4f} "
              f"us={f(r['migrate_us']):.1f}")

    # det
    if os.path.exists(DET):
        lines = [l.strip() for l in open(DET, encoding="utf-8") if l.strip()]
        n_ident = sum(1 for l in lines if l.endswith("true") and "determinism" not in l)
        n_pair = sum(1 for l in lines if " vs " in l)
        print(f"== det == {n_pair} 对,IDENTICAL {n_ident}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
