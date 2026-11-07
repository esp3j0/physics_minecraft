#!/usr/bin/env python3
"""从 out/matrix_submitted.txt(提交数据)计算 README 的总结数字。

A00-3 R1 数据要求:README 的每个总结数字用脚本从提交文件算出来再写。
用法:python scripts/summary.py  (在 demos/agents/freeze/ 下运行)
"""
from __future__ import annotations

import csv
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MATRIX = os.path.join(HERE, "out", "matrix_submitted.txt")
DET = os.path.join(HERE, "out", "det_submitted.txt")


def f(v: str) -> float:
    if v in ("-", "", "inf"):
        return float("nan") if v == "inf" else 0.0
    return float(v)


def main() -> int:
    with open(MATRIX, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    ok = lambda r: r["panicked"] == "false"

    snap = [r for r in rows if "_snap_" in r["name"] and ok(r) and not r["name"].startswith("control")]
    kinroot = [r for r in rows if "_kinroot_" in r["name"] and ok(r)]
    fixedroot = [r for r in rows if "_fixedroot_" in r["name"] and ok(r)]
    panics = [r for r in rows if r["panicked"] == "true"]
    ctrl = [r for r in rows if r["name"].startswith("control_")]
    ctrl_walk = [r for r in ctrl if "_walk_" in r["name"]]
    eight = [r for r in rows if r["name"].startswith("eight")]

    print(f"== 行数 == total={len(rows)} snap={len(snap)} kinroot={len(kinroot)} "
          f"fixedroot={len(fixedroot)} panic证据行={len(panics)} control={len(ctrl)} "
          f"eight={len(eight)}")
    print("panic 行:", [r["name"] for r in panics])

    # 判据 1:snap 冻结期位移 + 形状核对
    drifts = [f(r["frozen_drift_m"]) for r in snap]
    bad_shape = [r["name"] for r in snap if r["obstacle_shape_ok"] != "true"]
    snap_with_freeze = [r for r in snap if r["obstacle_shape_ok"] == "true"]
    print(f"== 判据1 == snap 冻结 drift max={max(drifts) if drifts else 0:.3e} m; "
          f"obstacle_shape_ok=true 行 {len(snap_with_freeze)};非 true:{bad_shape}")

    # 判据 2:唤醒后步进前误差
    ec = max(f(r["err_coord_rad"]) for r in snap)
    ep = max(f(r["err_pos_m"]) for r in snap)
    er = max(f(r["err_rot_rad"]) for r in snap)
    print(f"== 判据2 == snap max err_coord={ec:.3e} err_pos={ep:.3e} err_rot={er:.3e}")

    # 判据 3:窗口指标(站立 + 行走)
    for tag, grp in (("stand", [r for r in snap if r["name"].startswith("stand_")
                                and "long" not in r["name"]]),
                     ("walk", [r for r in snap if r["name"].startswith("walk_p")])):
        if not grp:
            continue
        upy = min(f(r["post_upy_min"]) for r in grp)
        vy = max(f(r["post_max_vy"]) for r in grp)
        link = max(f(r["post_link_max"]) for r in grp)
        rise = max(f(r["body_max_rise"]) for r in grp)
        clink = max(f(r["post_link_max"]) for r in ctrl_walk) if ctrl_walk else 0
        print(f"== 判据3 {tag}(n={len(grp)})== upy_min={upy:.4f} vy_max={vy:.4f} "
              f"link_max={link:.4f}(对照 {clink:.4f}) rise_max={rise:.4f}")

    # 判据 4:摔倒与 vfwd
    fell = [r["name"] for r in snap if r["fell_tick"] not in ("-", "")]
    ctrl_v = [f(r["vfwd"]) for r in ctrl_walk if f(r["vfwd"]) > 0]
    walk_snap = [r for r in snap if r["name"].startswith("walk_p")]
    snap_v = [f(r["vfwd"]) for r in walk_snap if f(r["vfwd"]) > 0]
    # 逐相位比(snap 行 vs 同相位对照):对照均值口径会错(对照 12 行均值
    # 对 24 个 snap 行( rst/z 两种)分母不同)。
    pct = []
    for r in walk_snap:
        ph = r["name"].split("_")[1]  # walk_pXX_...
        c = [f(x["vfwd"]) for x in ctrl_walk if x["name"].endswith(ph)]
        if c and f(r["vfwd"]) > 0:
            pct.append(f(r["vfwd"]) / c[0] * 100)
    print(f"== 判据4 == snap 摔倒行: {fell if fell else '无'}; 对照 vfwd 均值 "
          f"{sum(ctrl_v)/len(ctrl_v):.4f}, snap 行走均值 "
          f"{sum(snap_v)/len(snap_v):.4f};逐相位比 {min(pct):.1f}%–{max(pct):.1f}%")
    fr_fell = [r["name"] for r in fixedroot if r["fell_tick"] not in ("-", "")]
    kr_fell = [r["name"] for r in kinroot if r["fell_tick"] not in ("-", "")]
    print(f"== 否决记录 == fixedroot 摔倒 {len(fr_fell)} 行: {fr_fell[:3]}..; "
          f"kinroot 摔倒 {len(kr_fell)} 行")
    kr_drift = max(f(r["frozen_drift_m"]) for r in kinroot)
    fr_drift = max(f(r["frozen_drift_m"]) for r in fixedroot)
    print(f"== 否决记录 drift == kinroot max={kr_drift:.4f} m, fixedroot max={fr_drift:.4f} m")

    # 判据 5:NaN
    nan_rows = [r["name"] for r in rows
                if r["nan"] == "true" and r["panicked"] != "true"]
    print(f"== 判据5 == NaN 行(panic 证据行除外): "
          f"{nan_rows if nan_rows else '无'}")

    # 场景报数(snap 压住/砸中/下落/移走支撑)
    for key in ("pressed_snap", "hit_snap", "falling_snap", "suprm_snap",
                "pile_snap", "stand_long_snap", "walk_p06_long_snap"):
        g = [r for r in snap if r["name"].startswith(key)]
        for r in g:
            print(f"== 场景 == {r['name']}: woke={r['woke_tick']} "
                  f"upy={f(r['post_upy_min']):.4f} vy={f(r['post_max_vy']):.4f} "
                  f"rise={f(r['body_max_rise']):.4f} note={r['scene_note']}")

    # 耗时
    fus = [f(r["freeze_us"]) for r in snap if f(r["freeze_us"]) > 0]
    wus = [f(r["wake_us"]) for r in snap if f(r["wake_us"]) > 0]
    print(f"== 耗时 == snap freeze {min(fus):.1f}–{max(fus):.1f} µs, "
          f"wake {min(wus):.1f}–{max(wus):.1f} µs")
    for r in eight:
        print(f"== eight == {r['name']}: freeze_us={f(r['freeze_us']):.2f} "
              f"wake_us={f(r['wake_us']):.2f} upy={f(r['post_upy_min']):.4f} "
              f"hash={r['state_hash']}")

    # det
    if os.path.exists(DET):
        lines = [l.strip() for l in open(DET, encoding="utf-8") if l.strip()]
        pairs = [l for l in lines if " vs " in l]
        ident = [l for l in pairs if l.endswith("true")]
        print(f"== det == {len(pairs)} 对, IDENTICAL {len(ident)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
