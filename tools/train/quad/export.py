"""四足策略导出(E02b-2 §3)。

python -m quad.export --ckpt <pt> --manifest <训练 manifest> --name <策略名> --out <目录>
写出:
- policy.onnx:actor 均值网络(不含归一化),obs float32 [1,obs_dim] → action [1,act_dim];
- policy.toml:训练 manifest 原样,name 换成 --name,加 [normalization]
  (mean = 检查点均值,std = sqrt(var+1e-8)),obs_clip = PPO 归一化后观测裁剪值
  (底本 RunningMeanStd.normalize 默认 10.0);格式可被 PolicyManifest::from_toml_str 解析;
- probe.json:RapierBackend(1 env,intent 全 0,episode_controls 600)reset(seed) 首帧观测,
  seed=0/1/2 各一条 {seed, obs, action};action = 归一化(并裁剪)后过 actor 均值、
  再裁剪到 action_clip。
"""

from __future__ import annotations

import argparse
import json
import os
import tomllib

import numpy as np
import torch

from quad.ppo import Agent

# 底本 RunningMeanStd.normalize 的默认裁剪值(ppo 用默认调用)。
OBS_CLIP = 10.0


def _fmt_arr(vals) -> str:
    return "[" + ", ".join(f"{float(v):.9g}" for v in vals) + "]"


def write_policy_toml(path, manifest: dict, name: str, mean, std) -> None:
    """训练 manifest 原样 + name 替换 + [normalization] + obs_clip。"""
    lines = [
        f'name = "{name}"',
        f'body_hash = "{manifest["body_hash"]}"',
        f'control_period_ticks = {manifest["control_period_ticks"]}',
        f'action_scale = {float(manifest["action_scale"]):.9g}',
    ]
    if manifest.get("action_clip") is not None:
        lines.append(f'action_clip = {float(manifest["action_clip"]):.9g}')
    lines.append(f"obs_clip = {OBS_CLIP:.9g}")
    if manifest.get("default_pose") is not None:
        lines.append(f"default_pose = {_fmt_arr(manifest['default_pose'])}")
    lines.append("")
    lines.append("[normalization]")
    lines.append(f"mean = {_fmt_arr(mean)}")
    lines.append(f"std = {_fmt_arr(std)}")
    for e in manifest["obs"]:
        lines.append("")
        lines.append("[[obs]]")
        lines.append(f'term = "{e["term"]}"')
        lines.append(f'scale = {float(e.get("scale", 1.0)):.9g}')
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def main() -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument("--ckpt", required=True)
    cli.add_argument("--manifest", required=True)
    cli.add_argument("--name", required=True)
    cli.add_argument("--out", required=True)
    cli.add_argument("--body", default="../../assets/bodies/quad.body.toml")
    args = cli.parse_args()

    device = torch.device("cpu")
    ckpt = torch.load(args.ckpt, map_location=device, weights_only=False)
    spec_dict = ckpt["spec"]
    obs_dim = spec_dict["obs_dim"]
    act_dim = spec_dict["act_dim"]
    action_clip = spec_dict.get("action_clip")

    agent = Agent(obs_dim, act_dim)
    agent.load_state_dict(ckpt["agent_state"])
    agent.eval()

    os.makedirs(args.out, exist_ok=True)
    onnx_path = os.path.join(args.out, "policy.onnx")
    dummy = torch.zeros(1, obs_dim)
    torch.onnx.export(
        agent.actor_mean,
        dummy,
        onnx_path,
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes=None,
        opset_version=17,
        external_data=None,
    )
    print(f"policy.onnx → {onnx_path}")

    with open(args.manifest, "rb") as f:
        manifest = tomllib.load(f)
    mean = np.asarray(ckpt["obs_mean"], dtype=np.float64)
    var = np.asarray(ckpt["obs_var"], dtype=np.float64)
    std = np.sqrt(var + 1e-8)
    toml_path = os.path.join(args.out, "policy.toml")
    write_policy_toml(toml_path, manifest, args.name, mean, std)
    print(f"policy.toml → {toml_path}")

    # probe.json:1 env、intent 全 0、episode_controls 600,seed 0/1/2 首帧。
    from quad.backends.rapier import RapierBackend

    backend = RapierBackend(
        args.body,
        args.manifest,
        n_envs=1,
        threads=1,
        episode_controls=600,
        intent_range=[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
    )
    probes = []
    for seed in (0, 1, 2):
        obs = backend.reset(seed)[0]
        norm = np.clip((obs - mean) / std, -OBS_CLIP, OBS_CLIP).astype(np.float32)
        with torch.no_grad():
            act = agent.actor_mean(torch.from_numpy(norm).reshape(1, obs_dim)).numpy()[0]
        if action_clip is not None:
            act = np.clip(act, -action_clip, action_clip)
        probes.append(
            {"seed": seed, "obs": obs.astype(np.float64).tolist(), "action": act.astype(np.float64).tolist()}
        )
    probe_path = os.path.join(args.out, "probe.json")
    with open(probe_path, "w", encoding="utf-8") as f:
        json.dump(probes, f, indent=2)
    print(f"probe.json → {probe_path}({len(probes)} 条)")


if __name__ == "__main__":
    main()