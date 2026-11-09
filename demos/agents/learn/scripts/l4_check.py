#!/usr/bin/env python3
"""L4 判据数据:--init-from 微调的"接得上"检查。

从提交的训练曲线 CSV 算:
- ckpt_last20_avg = 基线训练曲线最后 20 迭代 ep_rew_mean 的均值;
- ft_first_iter = 微调曲线第 1 迭代的 ep_rew_mean;
- diff_pct = 两者相对差(R3 判据:|diff| ≤ 20% PASS);
- nan_in_ft_run = 微调日志里 NaN 出现次数(应为 0)。

用法:python scripts/l4_check.py <baseline_train.csv> <ft_train.csv> <ft.log> <out.txt>
"""
from __future__ import annotations

import csv
import sys


def main() -> int:
    base_csv, ft_csv, ft_log, out_path = sys.argv[1:5]
    base = list(csv.DictReader(open(base_csv, encoding="utf-8")))
    ft = list(csv.DictReader(open(ft_csv, encoding="utf-8")))
    last20 = [float(r["ep_rew_mean"]) for r in base[-20:]]
    ckpt_avg = sum(last20) / len(last20)
    ft_first = float(ft[0]["ep_rew_mean"])
    diff_pct = abs(ft_first - ckpt_avg) / abs(ckpt_avg) * 100.0
    nan_count = sum(
        1 for line in open(ft_log, encoding="utf-8") if "nan" in line.lower()
    )
    ok = diff_pct <= 20.0
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"ckpt_last20_avg,{ckpt_avg:.4f}\n")
        f.write(f"ft_first_iter,{ft_first:.4f}\n")
        f.write(f"diff_pct,{diff_pct:.2f}\n")
        f.write(f"pass_20pct,{'PASS' if ok else 'FAIL'}\n")
        f.write(f"nan_in_ft_run,{nan_count}\n")
    print(
        f"L4: ckpt last20 {ckpt_avg:.4f} vs ft first {ft_first:.4f}"
        f" diff {diff_pct:.2f}%({'PASS' if ok else 'FAIL'}) nan {nan_count} → {out_path}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
