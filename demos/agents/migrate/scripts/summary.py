#!/usr/bin/env python3
"""从 out/matrix_submitted.txt(提交数据)计算 README 的总结数字。

A00-3 R1 数据要求:README 的每个总结数字用脚本从提交文件算出来再写。
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


def load_rows():
    with open(MATRIX, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def f(v: str) -> float:
    if v in ("-", "", "inf"):
        return float("nan") if v == "inf" else 0.0
    return float(v)


def main() -> int:
    rows = load_rows()
    full = [r for r in rows if r["name"].endswith("_full") or "_full_" in r["name"]]
    walk = [r for r in rows if r["name"].startswith("walk_p")]
    stand = [r for r in rows if r["name"].startswith("stand_")]
    land = [r for r in rows if r["name"].startswith("land_")]
    ctrl = [r for r in rows if r["name"].startswith("control_")]
    pp = [r for r in rows if r["name"].startswith("pingpong_")]
    order = [r for r in rows if r["name"].startswith("order_")]
    seq = [r for r in rows if r["name"].startswith("seq_")]
    eight = [r for r in rows if r["name"].startswith("eight_")]

    print(f"== 行数 ==  total={len(rows)} walk={len(walk)} stand={len(stand)} "
          f"land={len(land)} control={len(ctrl)} pingpong={len(pp)} "
          f"order={len(order)} seq={len(seq)} eight={len(eight)}")

    # 判据 1:full 行重建误差
    f_err_c = [f(r["err_coord_rad"]) for r in full]
    f_err_p = [f(r["err_pos_m"]) for r in full]
    f_err_r = [f(r["err_rot_rad"]) for r in full]
    print(f"== 判据1 (full {len(full)} 行) == max err_coord={max(f_err_c):.3e} "
          f"max err_pos={max(f_err_p):.3e} max err_rot={max(f_err_r):.3e} "
          f"零误差行={sum(1 for c, p in zip(f_err_c, f_err_p) if c == 0.0 and p == 0.0)}")

    # 判据 2:full 迁移行窗口指标(行走 + 站立;land 单独)
    mig_full = [r for r in full if not r["name"].startswith(("land_", "control_"))]
    ctrl_walk = [r for r in ctrl if r["name"].startswith("control_walk")]
    worst_upy = min(f(r["post_upy_min"]) for r in mig_full)
    worst_vy = max(f(r["post_max_vy"]) for r in mig_full)
    worst_link = max(f(r["post_link_max"]) for r in mig_full)
    worst_step = max(f(r["post_body_step_max"]) for r in mig_full)
    ctrl_link = max(f(r["post_link_max"]) for r in ctrl_walk)
    ctrl_step = max(f(r["post_body_step_max"]) for r in ctrl_walk)
    land_mig = [r for r in full if r["name"].startswith("land_")]
    land_ctrl = [r for r in ctrl if r["name"].startswith("control_land")]
    print(f"== 判据2 (full 迁移 {len(mig_full)} 行) == upy_min={worst_upy:.4f} "
          f"vy_max={worst_vy:.4f} link_max={worst_link:.4f}(对照 {ctrl_link:.4f}) "
          f"body_step_max={worst_step:.4f}(对照 {ctrl_step:.4f})")
    print(f"== land 组 == 迁移 upy {[f(r['post_upy_min']) for r in land_mig][:3]}.. "
          f"vy {[f(r['post_max_vy']) for r in land_mig][:3]}.. link "
          f"{[f(r['post_link_max']) for r in land_mig][:3]}.. vs 对照 "
          f"upy {[f(r['post_upy_min']) for r in land_ctrl]} "
          f"vy {[f(r['post_max_vy']) for r in land_ctrl]} "
          f"link {[f(r['post_link_max']) for r in land_ctrl]}")

    # 判据 3:vfwd 相对对照的百分比(单次迁移,same/cross)
    for t in ("T0", "T1"):
        cw = [r for r in ctrl if r["name"] == f"control_walk_p06_{t}"]
        if not cw:
            continue
        base = f(cw[0]["vfwd"])
        for rb in ("same", "cross", "cross_hist", "histonly", "pilesleep", "pileawake"):
            grp = [r for r in walk if r["name"].startswith(f"walk_p") and rb in r["name"]
                   and r["name"].endswith(f"_{t}")]
            for sk in ("full", "novec"):
                g = [r for r in grp if f"_{sk}_" in r["name"]]
                if not g:
                    continue
                vs = [f(r["vfwd"]) for r in g]
                pct = [v / base * 100 for v in vs]
                print(f"== 判据3 {t} {rb}_{sk}: n={len(g)} vfwd min={min(vs):.4f} "
                      f"max={max(vs):.4f} 相对对照({base:.4f}) "
                      f"{min(pct):.1f}%–{max(pct):.1f}%")
    # 摔倒 / 穿透
    fell = [r["name"] for r in rows if r["fell_tick"] not in ("-", "")]
    pens = [f(r["post_pen_max"]) for r in walk]
    print(f"== 摔倒行 == {fell if fell else '无'}; 行走窗口对地穿透最深 "
          f"{min(pens):.4f} m(全迁移行走行)")
    land_pen = min(f(r["post_pen_max"]) for r in land)
    print(f"== land 穿透 == {land_pen:.4f} m")

    # 判据 4:pingpong
    cpp = [r for r in ctrl if r["name"] == "control_pingpong_T0"]
    pbase = f(cpp[0]["vfwd"]) if cpp else float("nan")
    for gap in (2, 5, 30, 120):
        g = [r for r in pp if r["name"] == f"pingpong_g{gap}_same_full_T0"]
        if g:
            v = f(g[0]["vfwd"])
            upy = f(g[0]["post_upy_min"])
            print(f"== 判据4 g{gap}: vfwd={v:.4f} ({v / pbase * 100:.1f}% of 对照 "
                  f"{pbase:.4f}) upy_min={upy:.4f}")
    for rb in ("cross", "cross_hist"):
        g = [r for r in pp if r["name"] == f"pingpong_g30_{rb}_full_T0"]
        if g:
            print(f"== 判据4 g30 {rb}: vfwd={f(g[0]['vfwd']):.4f}")

    # novec 对照
    for t in ("T0", "T1"):
        bf = [f(r["vfwd"]) for r in walk if f"same_full_{t}" in r["name"]]
        bn = [f(r["vfwd"]) for r in walk if f"same_novec_{t}" in r["name"]]
        if bf and bn:
            print(f"== novec {t}: full 均值 {sum(bf)/len(bf):.4f} novec 均值 "
                  f"{sum(bn)/len(bn):.4f} = {sum(bn)/sum(bf)*100:.1f}%")

    # diverge(cross 家族 vs same_full 同快照;M9)
    div = [(r["name"], r["diverge_tick"]) for r in rows
           if r["diverge_tick"] not in ("-", "") and r["name"].startswith("walk_p")]
    hist_div = [int(t) for n, t in div if any(k in n for k in
                ("cross_hist", "histonly", "pile"))]
    cross_div = [int(t) for n, t in div if "cross_full" in n]
    cross_same = [int(t) for n, t in div if "cross_full" in n]
    n_cross = len([r for r in walk if "cross_full" in r["name"]])
    print(f"== diverge == cross_full 分流行 {len(cross_div)}/{n_cross}, "
          f"分歧拍范围 {min(cross_div) if cross_div else '-'}–"
          f"{max(cross_div) if cross_div else '-'}; "
          f"cross_hist/histonly/pile 家族分流 {len(hist_div)} 行, "
          f"分歧拍 {min(hist_div) if hist_div else '-'}–{max(hist_div) if hist_div else '-'}")

    # 耗时
    mus = [f(r["migrate_us"]) for r in rows if f(r["migrate_us"]) > 0
           and not r["name"].startswith("eight")]
    print(f"== migrate_us == 单次 {min(mus):.1f}–{max(mus):.1f} µs")

    # order_* 表
    for r in order:
        print(f"== order == {r['name']}: start={r.get('active_order_start', '-')} "
              f"end={r.get('active_order_end', '-')} vfwd={f(r['vfwd']):.4f}")
    for r in seq:
        print(f"== seq == {r['name']}: pre={r.get('seq_pre_order', '-')} "
              f"post={r.get('seq_post_order', '-')} "
              f"pre_vfwd={f(r['seq_pre_vfwd']):.4f} "
              f"post_vfwd={f(r['seq_post_vfwd']):.4f} "
              f"yaw_rate {f(r['seq_pre_yaw_rate']):.2f}->{f(r['seq_post_yaw_rate']):.2f} °/s")

    # eight
    for r in eight:
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


if __name__ == "__main__":
    sys.exit(main())
