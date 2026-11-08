"""ONNX 导出(R2):torch.onnx.export 导出 actor 的均值网络。

输入 = 归一化前的原始观测([1, obs_dim]);归一化参数写进 policy.toml
(REQ-012 §5.2),`learn eval`(Rust tract)先做归一化再喂网络。
确定性动作 = 均值(actor_mean),log σ 不导出(探索只在训练用)。

用法:python -m train.export_onnx --ckpt out/arm_s0/ckpt_final.pt --out out/arm_s0
"""

from __future__ import annotations

import argparse
import csv
import os

import numpy as np
import torch

from train import spec


def main() -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument("--ckpt", required=True)
    cli.add_argument("--out", required=True, help="输出目录(写 policy.onnx + policy.toml)")
    args = cli.parse_args()

    device = torch.device("cpu")
    ckpt = torch.load(args.ckpt, map_location=device, weights_only=False)
    ckpt_spec = ckpt["spec"]
    if not spec.spec_matches(ckpt_spec):
        cur = spec.to_dict()
        diffs = [k for k in set(cur) | set(ckpt_spec) if cur.get(k) != ckpt_spec.get(k)]
        raise SystemExit(f"导出拒绝:spec 不一致,差异字段 {sorted(diffs)}")

    from train.ppo import Agent

    agent = Agent(spec.OBS_DIM, spec.ACT_DIM)
    agent.load_state_dict(ckpt["agent_state"])
    agent.eval()

    os.makedirs(args.out, exist_ok=True)
    onnx_path = os.path.join(args.out, "policy.onnx")
    dummy = torch.zeros(1, spec.OBS_DIM)
    torch.onnx.export(
        agent.actor_mean,
        dummy,
        onnx_path,
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes=None,
        opset_version=17,
        # torch 2.15 新导出器默认把大初始值放外置 .data 文件;本模型
        # 只有 ~20 KB,内联进 .onnx(Rust 侧 tract 单文件加载,评估
        # 必须能从提交文件运行)。
        external_data=None,
    )
    print(f"policy.onnx → {onnx_path}")

    # policy.toml(REQ-012 §5.2:身体哈希、控制周期、观测布局、动作缩放、
    # 默认姿态、归一化参数;本任务无默认姿态锚,动作是增量式)。
    mean = ckpt["obs_mean"]
    var = ckpt["obs_var"]
    inv_std = (1.0 / np.sqrt(var + 1e-8)).astype(np.float64)
    def fmt(vals):
        return ", ".join(f"{v:.8g}" for v in vals)
    toml = (
        "# REQ-012 §5.2 策略包(ARM 任务;demo learn R2 导出)\n"
        f'body_hash = "{spec.BODY}"\n'
        f"control_hz = {spec.CONTROL_HZ}\n"
        "obs_layout = [\n"
        + "".join(f'  "{k}",\n' for k in spec.OBS_LAYOUT)
        + "]\n"
        f"act_dim = {spec.ACT_DIM}\n"
        f"act_scale = {spec.ACT_SCALE}\n"
        "act_mode = \"incremental\"\n"
        f"joint_limit_rad = {spec.JOINT_LIMIT:.6f}\n"
        f"norm = \"obs_mean[{spec.OBS_DIM}] | obs_inv_std[{spec.OBS_DIM}](f32)\"\n"
        f"obs_mean = [{fmt(mean)}]\n"
        f"obs_inv_std = [{fmt(inv_std)}]\n"
        f"hidden_act = \"tanh\"\n"
        "onnx_inputs = \"obs[1,12](归一化后)\"\n"
        "onnx_outputs = \"action[1,2]\"\n"
    )
    toml_path = os.path.join(args.out, "policy.toml")
    with open(toml_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(toml)
    print(f"policy.toml → {toml_path}")


if __name__ == "__main__":
    main()
