"""四足奖励手算自检(E02b-2 §1;`python -m quad.selftest`,有不过的非零退出)。

期望值照抄任务书(架构师手算),不用被测函数反推、不另写一套算法。
"""

from __future__ import annotations

import sys

import numpy as np

from quad import task


def _ri(v, w, g, h, a, ap, intent) -> dict[str, np.ndarray]:
    return {
        "base_linvel_body": np.asarray([v], dtype=np.float32),
        "base_angvel_body": np.asarray([w], dtype=np.float32),
        "projected_gravity": np.asarray([g], dtype=np.float32),
        "root_height": np.asarray([[h]], dtype=np.float32),
        "action": np.asarray([a], dtype=np.float32),
        "prev_action": np.asarray([ap], dtype=np.float32),
        "intent": np.asarray([intent], dtype=np.float32),
    }


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    failed = []
    z12 = [0.0] * 12

    # R-A:intent=[1,0,0],v=(0.6,0,0),ω=0,g=(0,-1,0),h=0.2775,a=a' → r=1.5。
    ri = _ri([0.6, 0, 0], [0, 0, 0], [0, -1, 0], 0.2775, z12, z12, [1, 0, 0])
    r = float(task.reward(ri)[0])
    ok = abs(r - 1.5) < 1e-6
    print(f"[{'PASS' if ok else 'FAIL'}] R-A r={r:.8f} 期望 1.5(误差<1e-6)")
    if not ok:
        failed.append("R-A")

    # R-B:v=(0.1,0.1,0),ω=(1,0.5,0),g=(0.1,-0.99,0),h=0.3775,a 与 a' 一分量差 1。
    a = list(z12)
    a[0] = 1.0
    ri = _ri([0.1, 0.1, 0], [1, 0.5, 0], [0.1, -0.99, 0], 0.3775, a, z12, [1, 0, 0])
    r = float(task.reward(ri)[0])
    ok = abs(r - 0.16182) < 1e-4
    print(f"[{'PASS' if ok else 'FAIL'}] R-B r={r:.8f} 期望 0.16182(误差<1e-4)")
    if not ok:
        failed.append("R-B")

    # R-C:command([[-1,0.5,1]]) == (-0.6, 0.15, 1.0)。
    cx, cz, cyaw = task.command(np.asarray([[-1.0, 0.5, 1.0]], dtype=np.float32))
    got = (float(cx[0]), float(cz[0]), float(cyaw[0]))
    ok = all(abs(g - e) < 1e-6 for g, e in zip(got, (-0.6, 0.15, 1.0)))
    print(f"[{'PASS' if ok else 'FAIL'}] R-C command={got} 期望 (-0.6, 0.15, 1.0)")
    if not ok:
        failed.append("R-C")

    if failed:
        print(f"==== 结论: 有 FAIL({failed})====")
        return 1
    print("==== 结论: R-A/R-B/R-C 全部 PASS ====")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())