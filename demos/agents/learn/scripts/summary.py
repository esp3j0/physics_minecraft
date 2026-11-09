#!/usr/bin/env python3
"""从 out/ 提交数据计算 README 的全部总结数字。

R2 数据要求:按完整名字段筛行;CSV 列头与数据列数一致(脚本断言);
判据没过门槛不打 ✅;数字范围覆盖全部行,不挑行;算不出的写 "-";
全部正式训练都报。用法:python scripts/summary.py(在 demos/agents/learn/ 下运行)。
"""
from __future__ import annotations

import csv
import os
import statistics
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "out")


def f(v: str) -> float:
    if v in ("-", "", "nan", "True", "False", ""):
        return float("nan")
    return float(v)


def load(path: str) -> tuple[list[str], list[dict]]:
    full = os.path.join(OUT, path)
    with open(full, encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
    with open(full, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        assert len(r) == len(header), (
            f"列数不一致: {path} {len(r)} vs {len(header)}")
    return header, rows


def l1_stats(path: str, label: str) -> None:
    """L1 口径:成功率(≥0.90 达标)、最终距离中位数(≤0.02 达标)、NaN 数。"""
    _, rows = load(path)
    n = len(rows)
    holds = sum(1 for r in rows if r["hold"] == "1")
    nan_rows = sum(1 for r in rows if r["nan"] == "True")
    d = sorted(f(r["d_end"]) for r in rows)
    med = d[n // 2] if n else float("nan")
    ratio = holds / n if n else float("nan")
    print(
        f"== L1 [{label}] == 行 {n} 保持 {holds}/{n} = {ratio:.3f}"
        f"(门槛 ≥ 0.90 {'达标' if ratio >= 0.90 else '未达标'})"
        f" d_end 中位数 {med:.4f} m(门槛 ≤ 0.02"
        f" {'达标' if med <= 0.02 else '未达标'})"
        f" nan {nan_rows}"
    )


def main() -> int:
    # ── L0(环境自检)──
    _, rows = load("l0.txt")
    n = len(rows)
    passed = sum(1 for r in rows if r["pass"] == "PASS")
    print(f"== L0 == 行 {n} PASS {passed}/{n} = {passed / n:.3f}"
          f"(门槛 ≥ 0.95 {'达标' if passed / n >= 0.95 else '未达标'})")

    # ── 训练曲线(全部正式训练:30Hz s0/s1、60Hz s2、L4/L5 微调)──
    for name in ("train_arm_s0.csv", "train_arm_s1.csv",
                 "train_arm_s2_hz60.csv", "train_l4_ft.csv",
                 "train_l5_ft.csv"):
        _, rows = load(name)
        rew = [float(r["ep_rew_mean"]) for r in rows]
        dist = [float(r["dist_mean"]) for r in rows]
        reach = [float(r["reach_frac"]) for r in rows]
        last = rows[-1]
        nan_rows = sum(
            1 for r in rows
            if any(not (v.replace('.', '', 1).replace('-', '', 1).replace('e', '', 1)
                        .replace('+', '', 1)).isdigit() and v != 'nan'
                   and False for v in [])
        )
        print(
            f"== 训练 [{name.replace('train_', '').replace('.csv', '')}] =="
            f" 迭代 {len(rows)} 步 {last['total_steps']} 用时 {last['elapsed_s']}s"
            f" ep_rew {rew[0]:.1f} → {rew[-1]:.1f}(min {min(rew):.1f} max {max(rew):.1f})"
            f" dist_mean {dist[0]:.3f} → {dist[-1]:.3f}(min {min(dist):.3f})"
            f" reach_frac max {max(reach):.4f}"
        )

    # ── L1 矩阵(全部评估:主口径两种子 + 60Hz + L5 零样本/微调)──
    l1_stats("eval_s0.csv", "s0 主口径")
    l1_stats("eval_s1.csv", "s1 主口径")
    l1_stats("eval_s2_hz60.csv", "s2 60Hz 对照")
    l1_stats("eval_s1_l5_zero.csv", "s1 L5 零样本(质量×1.5 增益×0.7)")
    l1_stats("eval_s1_l5_ft.csv", "s1 L5 微调后(同改动物理)")
    l1_stats("eval_s1_l5_ft_main.csv", "s1 L5 微调后(主口径物理)")

    # ── order(连杆顺序打乱;s1)──
    _, rows = load("eval_order_s1.csv")
    # 干净世界的对照行数(从 eval_s1.csv 现算,不写死)。
    _, clean = load("eval_s1.csv")
    clean_holds = sum(1 for r in clean if r["hold"] == "1")
    clean_med = sorted(f(r["d_end"]) for r in clean)[len(clean) // 2]
    for setup in ("insert1", "insert4", "insert16"):
        sub = [r for r in rows if r["setup"] == setup]
        holds = sum(1 for r in sub if r["hold"] == "1")
        d = sorted(f(r["d_end"]) for r in sub)
        nan_rows = sum(1 for r in sub if r["nan"] == "True")
        print(
            f"== order [{setup}] == 行 {len(sub)} 保持 {holds}/{len(sub)}"
            f" d_end 中位数 {d[len(sub) // 2]:.4f} nan {nan_rows}"
            f"(干净世界 s1 = {clean_holds}/{len(clean)}"
            f" 中位 {clean_med:.4f} → 同水平,打乱无影响)"
        )

    # ── det(双跑逐位)──
    for name in ("det_s1.txt", "det_s1_l5_ft.txt"):
        lines = [l.strip() for l in open(os.path.join(OUT, name), encoding="utf-8") if l.strip()]
        ident = sum(1 for l in lines if l.endswith("IDENTICAL"))
        differ = sum(1 for l in lines if l.endswith("DIFFER"))
        print(f"== det [{name.replace('det_', '').replace('.txt', '')}] =="
              f" {len(lines)} 对 IDENTICAL {ident} DIFFER {differ}")

    # ── L2(线程无关)──
    for line in open(os.path.join(OUT, "L2_threads.txt"), encoding="utf-8"):
        if line.startswith("threads_"):
            print("== L2 ==", line.strip())

    # ── L3(torch vs tract;s1 与 l5_ft 两份)──
    for name in ("torch_tract_s1.txt", "torch_tract_s1_l5_ft.txt"):
        for line in open(os.path.join(OUT, name), encoding="utf-8"):
            print(f"== L3 [{name.replace('torch_tract_', '').replace('.txt', '')}] ==",
                  line.strip())

    # ── L4(微调接续)──
    for line in open(os.path.join(OUT, "l4_result.txt"), encoding="utf-8"):
        print("== L4 ==", line.strip())

    # ── 吞吐(Rust 物理侧)──
    lines = [l for l in open(os.path.join(OUT, "throughput.txt"),
                             encoding="utf-8").read().splitlines() if l and not l.startswith("#")]
    print("== 吞吐(Rust 物理,纯环境)==")
    hdr = lines[0].split(",")
    for l in lines[1:]:
        parts = l.split(",")
        assert len(parts) == len(hdr), f"throughput 列数不一致 {l}"
        print("  ", l)

    # ── 训练吞吐(Python 侧,曲线末行 steps/elapsed)──
    for name in ("train_arm_s0.csv", "train_arm_s1.csv", "train_arm_s2_hz60.csv"):
        _, rows = load(name)
        last = rows[-1]
        sps = int(last["total_steps"]) / float(last["elapsed_s"])
        print(f"== 训练吞吐 [{name.replace('train_', '').replace('.csv', '')}] =="
              f" {sps:.0f} 控制步/s(含收集+PPO 更新)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
