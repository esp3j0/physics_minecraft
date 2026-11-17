"""四足评估(E02b-2 §5)。

python -m quad.eval --ckpt <pt> --backend rapier --body … --manifest … \
    --intent a b c --episodes N --seconds S [--seed]
N 个环境各跑一个 S 秒回合(动作取 actor 均值,不采样),打印并写
<ckpt 同目录>/eval_<标签>.json:摔倒回合数/N、存活时间均值、|v.x−cx|、
|v.z−cz|、|ω.y−cyaw| 的时间平均(只统计存活的步)、平均机身高度、平均 up_y。
"""

from __future__ import annotations

import argparse
import json
import os

import numpy as np
import torch

from quad import task
from quad.ppo import Agent

OBS_CLIP = 10.0


def main() -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument("--ckpt", required=True)
    cli.add_argument("--backend", default="rapier")
    cli.add_argument("--body", required=True)
    cli.add_argument("--manifest", required=True)
    cli.add_argument("--mjcf", default=None, help="mujoco 后端必填")
    cli.add_argument("--intent", type=float, nargs=3, required=True, metavar=("a", "b", "c"))
    cli.add_argument("--episodes", type=int, required=True)
    cli.add_argument("--seconds", type=float, required=True)
    cli.add_argument("--seed", type=int, default=0)
    cli.add_argument("--threads", type=int, default=8)
    args = cli.parse_args()

    if args.backend not in ("rapier", "mujoco"):
        raise SystemExit(f"未知后端 `{args.backend}`")
    if args.backend == "mujoco" and args.mjcf is None:
        raise SystemExit("--backend mujoco 需要 --mjcf")

    device = torch.device("cpu")
    ckpt = torch.load(args.ckpt, map_location=device, weights_only=False)
    spec_dict = ckpt["spec"]
    obs_dim = spec_dict["obs_dim"]
    act_dim = spec_dict["act_dim"]
    action_clip = spec_dict.get("action_clip")
    control_hz = spec_dict["tick_hz"] / spec_dict["control_period_ticks"]
    mean = np.asarray(ckpt["obs_mean"], dtype=np.float64)
    std = np.sqrt(np.asarray(ckpt["obs_var"], dtype=np.float64) + 1e-8)

    agent = Agent(obs_dim, act_dim)
    agent.load_state_dict(ckpt["agent_state"])
    agent.eval()

    n = args.episodes
    steps = int(round(args.seconds * control_hz))
    a, b, c = args.intent
    intent_range = [[a, a], [b, b], [c, c]]
    if args.backend == "mujoco":
        from quad.backends.mujoco import MujocoBackend

        backend = MujocoBackend(
            args.body, args.manifest, n_envs=n, threads=args.threads,
            episode_controls=steps, intent_range=intent_range, mjcf_path=args.mjcf,
        )
    else:
        from quad.backends.rapier import RapierBackend

        backend = RapierBackend(
            args.body, args.manifest, n_envs=n, threads=args.threads,
            episode_controls=steps, intent_range=intent_range,
        )
    cx, cz, cyaw = a * task.V_FWD_MAX, b * task.V_LAT_MAX, c * task.YAW_MAX

    obs = backend.reset(args.seed)
    alive = np.ones(n, dtype=bool)
    survival = np.zeros(n, dtype=np.int64)
    fall_count = 0
    sum_dx = sum_dz = sum_dyaw = sum_h = sum_up = 0.0
    alive_steps = 0
    for _ in range(steps):
        norm = np.clip((obs - mean) / std, -OBS_CLIP, OBS_CLIP).astype(np.float32)
        with torch.no_grad():
            act = agent.actor_mean(torch.from_numpy(norm)).numpy()
        if action_clip is not None:
            act = np.clip(act, -action_clip, action_clip)
        obs, ri, terminated, truncated = backend.step(act.astype(np.float32))
        surviving = alive & ~terminated
        v = ri["base_linvel_body"]
        w = ri["base_angvel_body"]
        m = surviving
        cnt = int(m.sum())
        if cnt:
            sum_dx += float(np.abs(v[m, 0] - cx).sum())
            sum_dz += float(np.abs(v[m, 2] - cz).sum())
            sum_dyaw += float(np.abs(w[m, 1] - cyaw).sum())
            sum_h += float(ri["root_height"][m, 0].sum())
            sum_up += float(ri["up_y"][m, 0].sum())
            alive_steps += cnt
            survival[m] += 1
        newly = terminated & alive
        fall_count += int(newly.sum())
        alive = alive & ~terminated
        if not alive.any():
            break

    denom = max(1, alive_steps)
    result = {
        "episodes": n,
        "seconds": args.seconds,
        "intent": [a, b, c],
        "seed": args.seed,
        "control_hz": control_hz,
        "fall_episodes": fall_count,
        "fall_frac": fall_count / n,
        "survival_time_mean": float(survival.mean()) / control_hz,
        "abs_vx_err_mean": sum_dx / denom,
        "abs_vz_err_mean": sum_dz / denom,
        "abs_yaw_err_mean": sum_dyaw / denom,
        "height_mean": sum_h / denom,
        "up_y_mean": sum_up / denom,
    }
    label = "_".join(f"{x:g}" for x in (a, b, c))
    out_path = os.path.join(os.path.dirname(args.ckpt) or ".", f"eval_{label}.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(
        f"eval: 摔倒 {fall_count}/{n},存活均值 {result['survival_time_mean']:.3f}s,"
        f"|vx_err| {result['abs_vx_err_mean']:.4f},|vz_err| {result['abs_vz_err_mean']:.4f},"
        f"|yaw_err| {result['abs_yaw_err_mean']:.4f},高度 {result['height_mean']:.4f},"
        f"up_y {result['up_y_mean']:.4f}",
        flush=True,
    )
    print(f"eval_{label}.json → {out_path}")


if __name__ == "__main__":
    main()