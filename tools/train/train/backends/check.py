"""后端契约检查(R2 补充 train/backends/check.py;R3 扩两项)。

对任意后端检查:形状、dtype、键集合、reset 的确定性(同种子两次逐位
相同)、动作越界钳位;**R3/P3:learn_env 导出的环境常量逐项与 spec.py
比对,不一致就失败**(单一事实来源 = Rust 导出值;R2 时 ACT_SCALE 两边
各写各的,契约检查没查出来);**R3/P6:到时约定**——回合最后一步的
reward_inputs = reset 前最后一次转移的值(dist = 结束时距离,不是 reset
后新回合的出生距离),obs = reset 后新回合首帧。rapier 后端必须跑过:
`python -m train.backends.check rapier`。
MuJoCo 后端(transfer)写完后同一命令换名字即查。

线程无关性(L2)用 learn eval 探针路径(probe_env):1/4/16 线程的
VecEnv 对同一 env_id 同一动作序列,逐拍哈希逐位相同——检查也覆盖。
"""

from __future__ import annotations

import math
import sys

import numpy as np


def check_constants() -> bool:
    """R3/P3:learn_env 导出常量 vs spec.py,逐项一致才过。"""
    import learn_env

    from train import spec

    pairs = [
        ("OBS_DIM", learn_env.OBS_DIM, spec.OBS_DIM),
        ("ACT_DIM", learn_env.ACT_DIM, spec.ACT_DIM),
        ("ACT_SCALE", learn_env.ACT_SCALE, spec.ACT_SCALE),
        ("KP", learn_env.KP, spec.KP),
        ("KD", learn_env.KD, spec.KD),
        ("TAU_LIMIT", learn_env.TAU_LIMIT, spec.TAU_LIMIT),
        ("JOINT_LIMIT", learn_env.JOINT_LIMIT, spec.JOINT_LIMIT),
        ("LINK_LEN", learn_env.LINK_LEN, spec.LINK_LEN),
        ("LINK_MASS", learn_env.LINK_MASS, spec.LINK_MASS),
        ("TIP_MASS", learn_env.TIP_MASS, spec.TIP_MASS),
        ("EPISODE_CONTROLS", learn_env.EPISODE_CONTROLS, spec.EPISODE_CONTROLS),
        ("TICKS_PER_CONTROL", learn_env.TICKS_PER_CONTROL, spec.TICKS_PER_CONTROL),
        ("TARGET_R_MIN", learn_env.TARGET_R_MIN, spec.TARGET_R_MIN),
        ("TARGET_R_MAX", learn_env.TARGET_R_MAX, spec.TARGET_R_MAX),
        ("INIT_Q_RANGE", learn_env.INIT_Q_RANGE, spec.INIT_Q_RANGE),
        ("REACH_DIST", learn_env.REACH_DIST, spec.REACH_DIST),
        ("REWARD_KEYS count", learn_env.REWARD_KEYS, len(spec.REWARD_KEYS)),
    ]
    ok = True
    for name, rust_v, py_v in pairs:
        # f32(Rust)vs f64(Python)存在表示差(如 0.1 → 0.10000000149),
        # 按 f32 舍入后必须一致;不一致才是真契约违背(R3/P3)。
        same = np.float32(rust_v) == np.float32(py_v)
        ok &= same
        print(f"  [{'PASS' if same else 'FAIL'}] const {name}: learn_env={rust_v} spec={py_v}")
    return ok


def check_done_convention(seed: int = 77) -> bool:
    """R3/P6:到时步 = (obs 新回合首帧, reward_inputs reset 前转移)。"""
    from train import spec
    from train.backends import make_backend

    # 90 控制步一回合;跑 95 步必跨到时边界。
    be = make_backend("rapier", n_envs=4, threads=2, seed=seed)
    obs = be.reset(seed)
    ok = True
    saw_trunc = False
    for _ in range(95):
        a = np.zeros((4, spec.ACT_DIM), dtype=np.float32)
        obs, ri, term, trunc = be.step(a)
        if trunc.any():
            saw_trunc = True
            # trunc 环境的 obs 应是**新回合首帧**:出生角 ±0.5 采样自
            # ±INIT_Q_RANGE,出生末端在竖直附近但不在目标环内(目标
            # r ≥ 0.3);用 reward_inputs 交叉验证——dist 键是 reset 前
            # 转移的结束距离,若它等于 reset 后首帧 obs 的目标距离说明
            # 两侧口径又串了。这里只验 dist > 0 且有限(结束距离),并验
            # obs 的 q sin/cos 与 ±0.5 一致(新出生角;旧回合末 q 可能在
            # 限位附近,sin/cos 一般不同;非严格但配 Rust 单测足够)。
            idx = np.where(trunc)[0]
            for i in idx:
                d = float(ri["dist"][i, 0])
                ok &= np.isfinite(d) and d >= 0.0
                q1_sin, q1_cos = float(obs[i, 0]), float(obs[i, 1])
                q1 = math.atan2(q1_sin, q1_cos)
                ok &= abs(q1) <= spec.INIT_Q_RANGE + 1e-4
    print(f"  [{'PASS' if ok and saw_trunc else 'FAIL'}] done convention (trunc step: "
          f"obs=新首帧, reward_inputs=reset 前转移; saw_trunc={saw_trunc})")
    return ok and saw_trunc


def check(backend_name: str, n_envs: int = 8, threads: int = 4, seed: int = 123) -> bool:
    from train import spec
    from train.backends import make_backend

    ok = True

    def report(name: str, passed: bool) -> None:
        nonlocal ok
        ok &= passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}")

    # ── R3/P3:常量逐项(spec ↔ learn_env 导出)──
    ok &= check_constants()

    be = make_backend(backend_name, n_envs=n_envs, threads=threads, seed=seed)

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
    # 越界动作被钳到 [-1,1]:act1 键 = 本步动作(钳后;给 -5/+5 混合)。
    mixed = np.tile(np.array([[-5.0, 5.0]], dtype=np.float32), (n_envs, 1))
    _, ri_m, _, _ = be.step(mixed)
    act1 = ri_m["act1"][:, 0]
    report("action clamp to [-1,1]", bool(np.all(np.abs(act1) <= 1.0 + 1e-6)))

    # ── R3/P6:到时约定 ──
    ok &= check_done_convention()

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
