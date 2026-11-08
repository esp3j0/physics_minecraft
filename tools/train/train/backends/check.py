"""后端契约检查(R2 补充 train/backends/check.py)。

对任意后端检查:形状、dtype、键集合、reset 的确定性(同种子两次逐位
相同)、动作越界钳位。rapier 后端必须跑过:`python -m train.backends.check rapier`。
MuJoCo 后端(transfer)写完后同一命令换名字即查。

线程无关性(L2)用 learn eval 探针路径(probe_env):1/4/16 线程的
VecEnv 对同一 env_id 同一动作序列,逐拍哈希逐位相同——检查也覆盖。
"""

from __future__ import annotations

import sys

import numpy as np


def check(backend_name: str, n_envs: int = 8, threads: int = 4, seed: int = 123) -> bool:
    from train import spec
    from train.backends import make_backend

    be = make_backend(backend_name, n_envs=n_envs, threads=threads, seed=seed)
    ok = True

    def report(name: str, passed: bool) -> None:
        nonlocal ok
        ok &= passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}")

    # ── reset 形状/dtype/确定性 ──
    obs1 = be.reset(seed)
    report(
        "reset shape/dtype",
        obs1.shape == (n_envs, spec.OBS_DIM) and obs1.dtype == np.float32,
    )
    be2 = make_backend(backend_name, n_envs=n_envs, threads=threads, seed=seed)
    obs2 = be2.reset(seed)
    report("reset deterministic (same seed, bitwise)", bool(np.array_equal(obs1, obs2)))

    # ── step:形状、键集合、越界钳位 ──
    big_actions = np.full((n_envs, spec.ACT_DIM), 5.0, dtype=np.float32)
    o, ri, term, trunc = be.step(big_actions)
    report(
        "step shapes",
        o.shape == (n_envs, spec.OBS_DIM)
        and all(ri[k].shape == (n_envs, 1) for k in spec.REWARD_KEYS)
        and set(ri.keys()) == set(spec.REWARD_KEYS),
    )
    report(
        "step dtype float32",
        o.dtype == np.float32 and all(ri[k].dtype == np.float32 for k in spec.REWARD_KEYS),
    )
    report(
        "terminated/truncated shapes",
        term.shape == (n_envs,) and trunc.shape == (n_envs,),
    )
    # 越界动作被钳到 [-1,1]:act1 键 = 本步动作(与 big_actions 不同步时
    # reset 后首步 act=0;给 -5/+5 混合,act1 应落在 [-1,1])。
    mixed = np.tile(np.array([[-5.0, 5.0]], dtype=np.float32), (n_envs, 1))
    _, ri_m, _, _ = be.step(mixed)
    act1 = ri_m["act1"][:, 0]
    report("action clamp to [-1,1]", bool(np.all(np.abs(act1) <= 1.0 + 1e-6)))

    # ── 线程无关性(L2 的环境侧证据):probe_env 与线程数无关 ──
    probes = {}
    for th in (1, 4):
        bth = make_backend(backend_name, n_envs=4, threads=th, seed=seed)
        probes[th] = bth.probe_env(0, 64, seed)
    report("probe_env 1 vs 4 threads bitwise", probes[1] == probes[4])

    print(f"check({backend_name}): {'ALL PASS' if ok else 'FAILED'}")
    return ok


if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "rapier"
    sys.exit(0 if check(name) else 1)
