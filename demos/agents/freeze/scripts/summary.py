#!/usr/bin/env python3
"""从 out/matrix_submitted.txt(提交数据)计算 README 的总结数字。

A00-3 R2 数据要求:按完整名字段筛行(拆 scene/scheme/wake_vel),不用子串;
判据表每个数字由脚本输出;对照行(eight_control)的 freeze_us/wake_us 必须
为 "-"(FR1);每行列数 == 列头数(FR8)。
用法:python scripts/summary.py  (在 demos/agents/freeze/ 下运行)
"""
from __future__ import annotations

import csv
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MATRIX = os.path.join(HERE, "out", "matrix_submitted.txt")
PER = os.path.join(HERE, "out", "eight_per_robot_submitted.txt")
DET = os.path.join(HERE, "out", "det_submitted.txt")


def parse_name(name: str) -> dict:
    """行名拆字段(完整段比较)。例:walk_p06_snap_rst / stand_long_snap_rst /
    pile_snap_z / pressed_kinroot_rst / control_walk_p03 / eight_snap_rst。"""
    out: dict = {"raw": name}
    parts = name.split("_")
    if name.startswith("control_walk_p"):
        out.update(scene="control_walk", phase=int(parts[2][1:]))
    elif name.startswith("control_stand"):
        out.update(scene="control_stand")
    elif name.startswith("walk_p") and len(parts) >= 4:
        out.update(scene="walk", phase=int(parts[1][1:]), scheme=parts[2],
                   vel=parts[3])
    elif name.startswith("walk_p") or name.startswith("stand_"):
        # walk_p06_long_snap_rst(带 long 段)
        if name.startswith("stand_long_") or "_long_" in name:
            scheme = parts[-2]
            out.update(scene="long", scheme=scheme)
            if name.startswith("walk_p"):
                out["scene"] = "walk_long"
        elif name.startswith("stand_"):
            out.update(scene="stand", scheme=parts[1], vel=parts[2])
    elif name.startswith("pile_"):
        out.update(scene="pile", scheme=parts[1], vel=parts[2])
    elif name.startswith(("pressed_", "falling_", "hit_", "suprm_")):
        out.update(scene=parts[0], scheme=parts[1], vel=parts[2])
    elif name.startswith("eight"):
        out.update(scene="eight")
    elif name.startswith(("stand_kin_", "stand_fixed_", "walk_p00_kin_", "walk_p00_fixed_")):
        out.update(scene="panic", scheme=parts[1])
    return out


def f(v: str) -> float:
    if v in ("-", "", "inf"):
        return float("nan") if v == "inf" else 0.0
    return float(v)


def main() -> int:
    with open(MATRIX, encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
    with open(MATRIX, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    # FR8:每行列数 == 列头数。
    for r in rows:
        assert len(r) == len(header), (
            f"列数不一致: {r.get('name')} {len(r)} vs {len(header)}")
    # FR1:对照行(eight_control)耗列必须是 "-"。
    for r in rows:
        if r["name"].startswith("eight_control"):
            assert r["freeze_us"] == "-" and r["wake_us"] == "-", (
                f"FR1: 对照行耗列必须为 -: {r['name']} freeze_us={r['freeze_us']} "
                f"wake_us={r['wake_us']}")

    parsed = [parse_name(r["name"]) for r in rows]
    sel = lambda pred: [(r, p) for r, p in zip(rows, parsed) if pred(p)]
    scenes: dict = {}
    for p in parsed:
        scenes[p.get("scene", "?")] = scenes.get(p.get("scene", "?"), 0) + 1
    print("== 行数 ==", "total=" + str(len(rows)),
          " ".join(f"{k}={v}" for k, v in sorted(scenes.items())))
    panics = sel(lambda p: r["panicked"] == "true")
    print("panic 行:", [r["name"] for r, _ in panics])

    snap = sel(lambda p: p.get("scheme") == "snap" and p.get("scene") not in
               ("control_walk", "control_stand", "eight", "panic", "?"))
    snap_walk = sel(lambda p: p.get("scene") == "walk" and p.get("scheme") == "snap")
    snap_stand = sel(lambda p: p.get("scene") == "stand" and p.get("scheme") == "snap")
    ctrl_walk = sel(lambda p: p.get("scene") == "control_walk")
    kinroot = sel(lambda p: p.get("scheme") == "kinroot")
    fixedroot = sel(lambda p: p.get("scheme") == "fixedroot")
    eight = sel(lambda p: p.get("scene") == "eight")

    # 判据 1:snap 冻结 drift + 形状/材质核对
    drifts = [f(r["frozen_drift_m"]) for r, _ in snap]
    bad_shape = [r["name"] for r, _ in snap if r["obstacle_shape_ok"] != "true"]
    print(f"== 判据1 == snap 冻结 drift max={max(drifts) if drifts else 0:.3e} m "
          f"(n={len(snap)});obstacle_shape_ok=true 行 "
          f"{len(snap) - len(bad_shape)}/{len(snap)};非 true: {bad_shape}")

    # 判据 2:唤醒误差
    print(f"== 判据2 == snap max err_coord="
          f"{max(f(r['err_coord_rad']) for r, _ in snap):.3e} err_pos="
          f"{max(f(r['err_pos_m']) for r, _ in snap):.3e} err_rot="
          f"{max(f(r['err_rot_rad']) for r, _ in snap):.3e}")

    # 判据 3:窗口指标(站立 rst + 行走 snap;对照同相位)
    clink = max(f(r["post_link_max"]) for r, _ in ctrl_walk)
    for tag, grp in (("stand(rst)", snap_stand), ("walk", snap_walk)):
        if not grp:
            continue
        print(f"== 判据3 {tag}(n={len(grp)})== upy_min="
              f"{min(f(r['post_upy_min']) for r, _ in grp):.4f} vy_max="
              f"{max(f(r['post_max_vy']) for r, _ in grp):.4f} link_max="
              f"{max(f(r['post_link_max']) for r, _ in grp):.4f}(对照 {clink:.4f}) "
              f"rise_max={max(f(r['body_max_rise']) for r, _ in grp):.4f}")

    # 判据 4:摔倒与 vfwd(逐相位、同行走拍数对照)
    fell = [r["name"] for r, _ in snap if r["fell_tick"] not in ("-", "")]
    ctrl_by = {p["phase"]: f(r["vfwd"]) for r, p in ctrl_walk if f(r["vfwd"]) > 0}
    pct = []
    for r, p in snap_walk:
        c = ctrl_by.get(p["phase"])
        if c and f(r["vfwd"]) > 0:
            pct.append(f(r["vfwd"]) / c * 100)
    ctrl_v = [f(r["vfwd"]) for r, _ in ctrl_walk if f(r["vfwd"]) > 0]
    snap_v = [f(r["vfwd"]) for r, _ in snap_walk if f(r["vfwd"]) > 0]
    print(f"== 判据4 == snap 摔倒行: {fell if fell else '无'}; 对照 vfwd 均值 "
          f"{sum(ctrl_v)/len(ctrl_v):.4f}, snap 行走均值 "
          f"{sum(snap_v)/len(snap_v):.4f};逐相位比 {min(pct):.1f}%–{max(pct):.1f}%")
    fr_fell = [r["name"] for r, _ in fixedroot if r["fell_tick"] not in ("-", "")]
    kr_fell = [r["name"] for r, _ in kinroot if r["fell_tick"] not in ("-", "")]
    print(f"== 否决记录 == fixedroot 摔倒 {len(fr_fell)} 行: {fr_fell[:3]}..; "
          f"kinroot 摔倒 {len(kr_fell)} 行")
    print(f"== 否决记录 drift == kinroot max="
          f"{max(f(r['frozen_drift_m']) for r, _ in kinroot):.4f} m, fixedroot max="
          f"{max(f(r['frozen_drift_m']) for r, _ in fixedroot):.4f} m")

    # 判据 5:NaN(panic 证据行除外)
    nan_rows = [r["name"] for r, _ in sel(lambda p: r["nan"] == "true")]
    print(f"== 判据5 == NaN 行(panic 证据行除外): {nan_rows if nan_rows else '无'}")

    # 场景报数(pressed/hit/falling/suprm/pile/long)
    for key in ("pressed_snap", "hit_snap", "falling_snap", "suprm_snap",
                "pile_snap", "stand_long_snap", "walk_p06_long_snap"):
        for r, _ in sel(lambda p: r["name"].startswith(key)):
            print(f"== 场景 == {r['name']}: woke={r['woke_tick']} "
                  f"upy={f(r['post_upy_min']):.4f} vy={f(r['post_max_vy']):.4f} "
                  f"rise={f(r['body_max_rise']):.4f} note={r['scene_note']}")

    # 耗时
    fus = [f(r["freeze_us"]) for r, _ in snap if f(r["freeze_us"]) > 0]
    wus = [f(r["wake_us"]) for r, _ in snap if f(r["wake_us"]) > 0]
    print(f"== 耗时 == snap freeze {min(fus):.1f}–{max(fus):.1f} µs, "
          f"wake {min(wus):.1f}–{max(wus):.1f} µs")
    for r, _ in eight:
        print(f"== eight == {r['name']}: freeze_us={r['freeze_us']} "
              f"wake_us={r['wake_us']} upy={f(r['post_upy_min']):.4f} "
              f"hash={r['state_hash']}")

    # 8 只逐只行(FR3;列数断言)
    if os.path.exists(PER):
        with open(PER, encoding="utf-8") as fh:
            per_header = fh.readline().strip().split(",")
        per_rows = list(csv.DictReader(open(PER, encoding="utf-8")))
        for r in per_rows:
            assert len(r) == len(per_header), (
                f"逐只行列数不一致: {r.get('name')}")
        print(f"== 逐只行 == {len(per_rows)} 行(列数 {len(per_header)} 全一致);"
              f"示例 {per_rows[0]['name']} drift={per_rows[0]['frozen_drift_m']} "
              f"err_coord={per_rows[0]['err_coord_rad']}")

    # det
    if os.path.exists(DET):
        lines = [l.strip() for l in open(DET, encoding="utf-8") if l.strip()]
        pairs = [l for l in lines if " vs " in l]
        ident = [l for l in pairs if l.endswith("true")]
        print(f"== det == {len(pairs)} 对, IDENTICAL {len(ident)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
