"""torch 与 tract 的输出一致性(R2 判据 L3 的"torch vs tract ≤ 1e-5"部分)。

同一批 1000 个观测(固定种子的真实环境观测,不是随机数——分布对齐
评估时的输入),torch actor_mean 与 Rust tract(经 learn eval 的同款
ONNX 加载路径,这里用 onnxruntime 不可用,改与 Rust 侧同一 ONNX 由
Python onnx.reference 参照实现复核;真正的 tract 对照在 learn eval 侧
输出 `out/torch_tract.txt`——Rust `learn torchtract <onnx>` 子命令)。

Python 侧职责:生成观测 + 保存 torch 输出;Rust 侧职责:tract 输出 +
逐位对比(写 out/torch_tract.txt)。
用法:python -m train.torch_export_obs --ckpt <ckpt> --out <目录>
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import torch

from train import spec


def main() -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument("--ckpt", required=True)
    cli.add_argument("--out", required=True, help="输出目录(写 obs.bin + torch_out.bin)")
    cli.add_argument("--n", type=int, default=1000)
    cli.add_argument("--seed", type=int, default=2026)
    args = cli.parse_args()

    device = torch.device("cpu")
    ckpt = torch.load(args.ckpt, map_location=device, weights_only=False)
    from train.ppo import Agent

    agent = Agent(spec.OBS_DIM, spec.ACT_DIM)
    agent.load_state_dict(ckpt["agent_state"])
    agent.eval()

    # 1000 个真实分布观测:从环境 reset + 随机策略采(CleanRL 训练早期
    # 分布;归一化用检查点统计——与评估口径一致)。
    from train.backends import make_backend

    be = make_backend("rapier", n_envs=64, threads=8, seed=args.seed)
    obs = be.reset(args.seed)
    mean = ckpt["obs_mean"]
    std = np.sqrt(ckpt["obs_var"] + 1e-8)
    collected = []
    rng = np.random.default_rng(args.seed ^ 0xA11CE)
    while len(collected) * 64 < args.n:
        with torch.no_grad():
            o_norm = np.clip((obs - mean) / std, -10, 10).astype(np.float32)
            a = agent.actor_mean(torch.from_numpy(o_norm)).numpy()
        obs, _, _, _ = be.step(a.astype(np.float32))
        collected.append(obs.copy())
    all_obs = np.concatenate(collected, axis=0)[: args.n].astype(np.float32)

    # torch 输出(归一化后喂 actor)。
    o_norm = np.clip((all_obs - mean) / std, -10, 10).astype(np.float32)
    with torch.no_grad():
        out = agent.actor_mean(torch.from_numpy(o_norm)).numpy().astype(np.float32)

    os.makedirs(args.out, exist_ok=True)
    # torch_obs.bin = **归一化后的观测**(= ONNX 的输入口径;tract 侧
    # learn torchtract 直接喂本文件,不再二次归一化)。
    o_norm.tofile(os.path.join(args.out, "torch_obs.bin"))
    out.tofile(os.path.join(args.out, "torch_out.bin"))
    print(f"norm obs {o_norm.shape} → {args.out}/torch_obs.bin;torch 输出 → torch_out.bin")


if __name__ == "__main__":
    main()
