#!/usr/bin/env python3
"""从 out/ 提交数据计算 README 的全部总结数字。

A00-4 数据要求(放行):按完整名字段筛行,不用子串;CSV 列头与数据列数
一致(脚本断言);对照/基线行不做被测的操作;判据没过门槛不打 ✅。
用法:python scripts/summary.py  (在 demos/agents/capsule/ 下运行)
"""
from __future__ import annotations

import csv
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SWEEP = os.path.join(HERE, "out", "sweep_submitted.txt")
MATRIX = os.path.join(HERE, "out", "matrix_submitted.txt")
DET = os.path.join(HERE, "out", "det_submitted.txt")
CANDS = os.path.join(HERE, "out", "cands.txt")


def f(v: str) -> float:
    if v in ("-", "", "PANIC"):
        return float("nan")
    return float(v)


def load(path: str) -> tuple[list[str], list[dict]]:
    with open(path, encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
    with open(path, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        assert len(r) == len(header), (
            f"列数不一致: {r.get('name')} {len(r)} vs {len(header)}")
    return header, rows


def sweep_tuple(name: str) -> str:
    """swA_s1p5_<tuple> / swB_hj1p4_<tuple>_t<i> → <tuple>(A 行格式)。"""
    body = name[4:]
    if body.startswith("hj"):
        return body.split("_", 1)[1].rsplit("_t", 1)[0]
    return body.split("_", 1)[1]


def main() -> int:
    # ── sweep ──
    _, sw = load(SWEEP)
    a_rows = [r for r in sw if r["stage"] == "A"]
    b_rows = [r for r in sw if r["stage"] == "B"]
    tuples: dict[str, list[dict]] = {}
    for r in a_rows:
        tuples.setdefault(sweep_tuple(r["name"]), []).append(r)
    strict = {k for k, rs in tuples.items()
              if all(r["pass"] == "A" for r in rs)}
    hi = {k for k, rs in tuples.items()
          if all(float(r["speed_err_pct"]) <= 5.0
                 for r in rs if r["speed_target"] != "1.5")
          and all(float(r["vy_peak"]) <= 0.5 for r in rs)}
    b_tuples: dict[str, list[dict]] = {}
    for r in b_rows:
        # B 行名 swB_hj<hj>_<tuple>_t<i>:键含 hj 档(同一 tuple 3 个 hj)。
        body = r["name"][4:]
        hj = body.split("_", 1)[0]
        b_tuples.setdefault(hj + "|" + sweep_tuple(r["name"]), []).append(r)
    pass_b = {k for k, rs in b_tuples.items()
              if len(rs) == 3 and all(r["pass"] == "B" for r in rs)}
    print("== sweep == A 行", len(a_rows), "(元组", len(tuples), ")",
          "strict(passA)", len(strict), ";B 行", len(b_rows),
          "(hj 元组", len(b_tuples), ") passB", len(pass_b))
    for sp in ("1.5", "4.3", "5.6"):
        es = [float(r["speed_err_pct"]) for r in a_rows
              if r["speed_target"] == sp]
        ok = sum(1 for e in es if e <= 5.0)
        print(f"== sweepA s{sp} == err min={min(es):.2f}% "
              f"max(排除 no-move)={max([e for e in es if e < 99]):.2f}% "
              f"(no-move {sum(1 for e in es if e >= 99)} 行);≤5% {ok}/{len(es)}")

    # ── 候选 ──
    cands: dict[str, dict[str, str]] = {}
    for line in open(CANDS, encoding="utf-8"):
        if line.startswith("#") or not line.strip():
            continue
        parts = line.split()
        cands[parts[0]] = dict(p.split("=", 1) for p in parts[1:])
    print("== cands ==")
    for t, c in cands.items():
        print(f"  {t}: tau={c['tau']} fmax={c['fmax']} mu={c['mu']} "
              f"air={c['air']} hj={c['hj']} ge={c['ge']} mode={c['mode']}")

    # ── matrix ──
    _, rows = load(MATRIX)
    print("== matrix == 行数 total=", len(rows))
    panic_rows = [r["name"] for r in rows if r["panicked"] == "true"]
    print("panic 行:", panic_rows if panic_rows else "无")
    nan_rows = [r["name"] for r in rows if r["nan"] == "true"]
    print("NaN 行:", nan_rows if nan_rows else "无")

    def sel(pred) -> list[dict]:
        return [r for r in rows if pred(r["name"])]

    for tag in cands:
        sfx = "_" + tag
        # 判据 1:平地(T0/T1 × 3 速度 × 8 航向)稳态 err ≤5%
        w = sel(lambda n: n.startswith("walk_") and n.endswith(sfx))
        bad = [r["name"] for r in w
               if not f(r["speed_err_pct"]) <= 5.0]
        emax = max(f(r["speed_err_pct"]) for r in w)
        print(f"== 判据1 [{tag}] == 行 {len(w)} err max={emax:.2f}% "
              f">5% {len(bad)} 行")
        for sp, lbl in (("s1p5", "1.5"), ("s4p3", "4.3"), ("s5p6", "5.6")):
            sub = [r for r in w if f"_{sp}_h" in r["name"]]
            es = [f(r["speed_err_pct"]) for r in sub]
            okn = sum(1 for e in es if e <= 5.0)
            print(f"== 判据1 [{tag}] s{lbl} == err "
                  f"{min(es):.2f}–{max(es):.2f}% ≤5% {okn}/{len(es)}")
        # 判据 2:台阶
        for lbl, pre in (("1run", "step_1run_"), ("1stand", "step_1stand_"),
                         ("2run", "step_2run_"), ("2stand", "step_2stand_")):
            grp = sel(lambda n, p=pre: n.startswith(p) and n.endswith(sfx))
            per: dict[str, list[dict]] = {}
            for r in grp:
                d = r["name"].split("_")[2]
                per.setdefault(d, []).append(r)
            cells = [f"d{d} {sum(1 for r in rs if r['step_success'] == 'true')}"
                     f"/{len(rs)}" for d, rs in sorted(per.items())]
            print(f"== 判据2 [{tag}] {lbl} ==" + " ".join(cells))
        # 判据 3:空中连按跳 vs 单次
        aj1 = sel(lambda n: n == f"airjump_single_{tag}")
        ajr = sel(lambda n: n == f"airjump_repeat_{tag}")
        if aj1 and ajr:
            h1 = f(aj1[0]["jump_max_h"])
            hr = f(ajr[0]["jump_max_h"])
            print(f"== 判据3 [{tag}] == single={h1:.4f} m repeat={hr:.4f} m "
                  f"diff={hr - h1:+.4f} m(门槛 ≤+0.02)")
        # 判据 4:站立 3600 拍
        for r in sel(lambda n: n.startswith("stand_") and n.endswith(sfx)):
            print(f"== 判据4 [{tag}] == {r['name']}: drift={r['drift_m']} m "
                  f"sink={r['sink_m']} m sleep_ratio={r['sleep_ratio']} "
                  f"slept={r['slept']}")
        # 判据 5:接缝(8 航向)
        sm = sel(lambda n: n.startswith("seam_h") and n.endswith(sfx))
        stuck_rows = [r["name"] for r in sm if r["stuck"] == "true"]
        vy_max = max(f(r["vy_peak"]) for r in sm)
        dips = [f(r["dip_pct"]) for r in sm if r["dip_pct"] != "-"]
        fell = [r["name"] for r in sm if r["end_reason"] != "completed"]
        print(f"== 判据5 [{tag}] == 行 {len(sm)} 卡住: "
              f"{stuck_rows if stuck_rows else '无'} vy_peak max={vy_max:.4f} "
              f"dip max={max(dips):.2f}% 提前结束: "
              f"{fell if fell else '无'}")
        # 场景 6:方块堆
        for r in sel(lambda n: n.startswith("pile_") and n.endswith(sfx)):
            print(f"== 场景6 [{tag}] == {r['name']}: disp={r['block_disp_max']} "
                  f"woken={r['blocks_woken']} vy_peak={r['vy_peak']}")
        # 场景 6b:推方块
        for r in sel(lambda n: n == f"push_one_{tag}"):
            print(f"== 场景6b [{tag}] == disp={r['block_disp_max']}")
        # 场景 7:被砸(125 行)
        hit = sel(lambda n: n.startswith("hit_") and n.endswith(sfx))
        hd = [f(r["hit_disp_h"]) for r in hit]
        nz = [r["name"] for r in hit if f(r["hit_disp_h"]) > 0.0]
        hs = [f(r["speed_max"]) for r in hit]
        hp = min(f(r["pen_obj_min"]) for r in hit)
        hg = min(f(r["pen_min"]) for r in hit)
        print(f"== 场景7 [{tag}] == 行 {len(hit)} 位移>0 {len(nz)} 行 "
              f"(disp max {max(hd):.4f});speed max={max(hs):.4f};"
              f"对方块穿透 min={hp:.4f} 对地 min={hg:.4f}")
        # 场景 8:重物
        hv_all = sel(lambda n: n.startswith("heavy_m") and n.endswith(sfx))
        hv = [r for r in hv_all if "_vg200_" not in r["name"]]
        hv200 = [r for r in hv_all if "_vg200_" in r["name"]]
        for lbl, grp in (("无防护", hv), ("vg200", hv200)):
            for m in ("m8", "m64", "m512"):
                sub = [r for r in grp if f"_{m}_" in r["name"]]
                if not sub:
                    continue
                panics = sum(1 for r in sub if r["panicked"] == "true")
                nans = sum(1 for r in sub if r["nan"] == "true")
                guards = sum(int(r["guard_removed"]) for r in sub)
                sp_max = max(f(r["speed_max"]) for r in sub)
                print(f"== 场景8 [{tag}] {lbl} {m} == 行 {len(sub)} panic "
                      f"{panics} NaN {nans} guard {guards} speed max="
                      f"{sp_max:.4f}")
        # 场景 9:连杆顺序对照(与 walk_T0_s4p3 同航向 v_mean 比)
        base = {n.split("_h")[1].split("_")[0]: f(r["v_mean"])
                for r in rows
                if r["name"].startswith("walk_T0_s4p3_h")
                and r["name"].endswith(sfx)
                for n in [r["name"]]}
        for pre in ("order16_", "order16ball_"):
            od = sel(lambda n: n.startswith(pre) and n.endswith(sfx))
            diffs = []
            for r in od:
                h = r["name"].split("_h")[1].split("_")[0]
                if h in base:
                    diffs.append(f(r["v_mean"]) - base[h])
            dv = max((abs(d) for d in diffs), default=0.0)
            print(f"== 场景9 [{tag}] {pre}== 行 {len(od)} |Δv| max="
                  f"{dv:.4f} m/s(基线 walk_T0_s4p3 同航向)")

    # det
    if os.path.exists(DET):
        lines = [l.strip() for l in open(DET, encoding="utf-8") if l.strip()]
        pairs = [l for l in lines if " vs " in l]
        ident = [l for l in pairs if l.endswith("IDENTICAL")]
        differs = [l for l in pairs if l.endswith("DIFFER")]
        print("== det ==", len(pairs), "对 IDENTICAL", len(ident),
              "DIFFER", len(differs))
        if differs:
            for d in differs[:8]:
                print("  DIFFER:", d)
    return 0


if __name__ == "__main__":
    sys.exit(main())
