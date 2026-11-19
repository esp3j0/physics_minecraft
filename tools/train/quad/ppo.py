"""四足 PPO 唯一入口(E02b-2 §2;E02h D2 全路由张量循环)。

来源:CleanRL ppo_continuous_action.py(MIT);底本的 GAE、done 处理、
归一化冻结逻辑保持原样(那份调试过)。E02h 起 legacy numpy 训练循环退役:
--backend {rapier|mujoco|mujoco_warp} --device {cpu|cuda:0|cuda:1} 全部路由到
ppo_gpu.train_tensor(唯一张量循环)。rapier/mujoco 经 NumpyToTensorAdapter
零拷贝接入(mujoco 物理仍走 mujoco.rollout,nthread);mujoco_warp 原生张量。
奖励统一 quad.reward.reward_gpu(对 task.reward parity ≤1e-5)。

本模块保留:Agent(3 层 256 ELU,可学习 log_std 初值 -0.5)、检查点读写
(结构不变:rms numpy float64、spec 不带 torch 张量)、--init-from 的
spec_matches 比对(忽略 episode_controls、fall_up_y_min)。
curve.csv 每迭代追加:迭代号、累计步数、平均回合回报、平均回合长度、
摔倒占结束回合比例、每秒步数。

用法:
  python -m quad.ppo --backend mujoco --body ../../assets/bodies/quad.body.toml \
      --manifest quad/manifests/quad_train.toml --mjcf <quad.xml> \
      --n-envs 64 --threads 8 --device cpu --max-minutes 3 --out out/quad_smoke
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optimizer

from quad.spec import spec_matches

SOURCE = "Adapted from CleanRL ppo_continuous_action.py (MIT License)"
# --init-from 比对 spec 时忽略的两个键(任务书 §2)。
_SPEC_IGNORE = ("episode_controls", "fall_up_y_min")


def layer_init(layer: nn.Linear, std: float = np.sqrt(2).item(), bias_const: float = 0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer


class Agent(nn.Module):
    """actor/critic 各 3 层 256 ELU;动作 = 状态无关可学习 log σ(初值 -0.5)。"""

    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()
        self.critic = nn.Sequential(
            layer_init(nn.Linear(obs_dim, 256)),
            nn.ELU(),
            layer_init(nn.Linear(256, 256)),
            nn.ELU(),
            layer_init(nn.Linear(256, 256)),
            nn.ELU(),
            layer_init(nn.Linear(256, 1), std=1.0),
        )
        self.actor_mean = nn.Sequential(
            layer_init(nn.Linear(obs_dim, 256)),
            nn.ELU(),
            layer_init(nn.Linear(256, 256)),
            nn.ELU(),
            layer_init(nn.Linear(256, 256)),
            nn.ELU(),
            layer_init(nn.Linear(256, act_dim), std=0.01),
        )
        self.actor_logstd = nn.Parameter(torch.full((1, act_dim), -0.5))

    def get_value(self, x):
        return self.critic(x)

    def get_action_and_value(self, x, action=None):
        action_mean = self.actor_mean(x)
        action_logstd = self.actor_logstd.expand_as(action_mean)
        action_std = torch.exp(action_logstd)
        probs = torch.distributions.Normal(action_mean, action_std)
        if action is None:
            action = probs.sample()
        return (
            action,
            probs.log_prob(action).sum(1),
            probs.entropy().sum(1),
            self.critic(x),
        )


class RunningMeanStd:
    """观测归一化统计(Welford;CleanRL 同款;导出进 policy.toml)。"""

    def __init__(self, dim: int):
        self.mean = np.zeros(dim, dtype=np.float64)
        self.var = np.ones(dim, dtype=np.float64)
        self.count = 1e-4

    def update(self, x: np.ndarray) -> None:
        batch_mean = x.mean(axis=0)
        batch_var = x.var(axis=0)
        batch_count = x.shape[0]
        delta = batch_mean - self.mean
        tot = self.count + batch_count
        new_mean = self.mean + delta * batch_count / tot
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + delta**2 * self.count * batch_count / tot
        self.mean = new_mean
        self.var = m2 / tot
        self.count = tot

    def normalize(self, x: np.ndarray, clip: float = 10.0) -> np.ndarray:
        return np.clip((x - self.mean) / np.sqrt(self.var + 1e-8), -clip, clip).astype(
            np.float32
        )


def save_checkpoint(path, agent, opt, rms, args, spec_dict, total_steps) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    torch.save(
        {
            "agent_state": agent.state_dict(),
            "optimizer_state": opt.state_dict(),
            "obs_mean": rms.mean,
            "obs_var": rms.var,
            "obs_count": rms.count,
            "spec": spec_dict,
            "args": vars(args),
            "total_steps": total_steps,
        },
        path,
    )


def _spec_diffs(cur: dict, ckpt_spec: dict) -> list[str]:
    a = {k: v for k, v in cur.items() if k not in _SPEC_IGNORE}
    b = {k: v for k, v in ckpt_spec.items() if k not in _SPEC_IGNORE}
    return spec_matches(a, b)


def load_checkpoint(path, device, spec_dict):
    ckpt = torch.load(path, map_location=device, weights_only=False)
    ckpt_spec = ckpt["spec"]
    diffs = _spec_diffs(spec_dict, ckpt_spec)
    if diffs:
        raise SystemExit(f"--init-from 拒绝:spec 不一致,差异字段 {sorted(diffs)}")
    agent = Agent(spec_dict["obs_dim"], spec_dict["act_dim"]).to(device)
    agent.load_state_dict(ckpt["agent_state"])
    opt = optimizer.Adam(agent.parameters(), lr=3e-4, eps=1e-5)
    opt.load_state_dict(ckpt["optimizer_state"])
    rms = RunningMeanStd(spec_dict["obs_dim"])
    rms.mean = ckpt["obs_mean"]
    rms.var = ckpt["obs_var"]
    rms.count = ckpt["obs_count"]
    return agent, opt, rms, ckpt["args"], ckpt_spec, ckpt["total_steps"]


def main() -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument("--backend", default="rapier", help="rapier|mujoco|mujoco_warp")
    cli.add_argument("--device", default="cpu", help="cpu|cuda:0|cuda:1(rapier/mujoco 仅 cpu)")
    cli.add_argument("--body", required=True)
    cli.add_argument("--manifest", required=True)
    cli.add_argument("--mjcf", default=None, help="mujoco/mujoco_warp 后端必填")
    cli.add_argument("--seed", type=int, default=0)
    cli.add_argument("--out", default="out/quad")
    cli.add_argument("--n-envs", type=int, default=64)
    cli.add_argument("--threads", type=int, default=8)
    cli.add_argument("--torch-threads", type=int, default=4)
    cli.add_argument("--episode-controls", type=int, default=600, help="回合长 = 20 秒")
    cli.add_argument("--intent-range", type=float, nargs=6, default=[0.0] * 6,
                     metavar=("lo0", "hi0", "lo1", "hi1", "lo2", "hi2"))
    cli.add_argument("--num-steps", type=int, default=48)
    cli.add_argument("--total-timesteps", type=int, default=10_000_000)
    cli.add_argument("--max-minutes", type=float, default=30.0)
    cli.add_argument("--num-minibatches", type=int, default=32)
    cli.add_argument("--update-epochs", type=int, default=10)
    cli.add_argument("--lr", type=float, default=3e-4)
    cli.add_argument("--gamma", type=float, default=0.99)
    cli.add_argument("--gae-lambda", type=float, default=0.95)
    cli.add_argument("--clip-coef", type=float, default=0.2)
    cli.add_argument("--ent-coef", type=float, default=0.0)
    cli.add_argument("--vf-coef", type=float, default=0.5)
    cli.add_argument("--max-grad-norm", type=float, default=0.5)
    cli.add_argument("--norm-adv", type=lambda s: s == "1", default=True)
    cli.add_argument("--target-kl", type=float, default=None)
    cli.add_argument("--anneal-lr", type=lambda s: s == "1", default=True)
    cli.add_argument("--init-from", default=None)
    cli.add_argument("--freeze-obs-norm", action="store_true")
    cli.add_argument("--freeze-norm-after", type=int, default=10)
    cli.add_argument("--reward-scale", type=float, default=0.1)
    cli.add_argument("--report-every", type=int, default=10)
    args = cli.parse_args()

    from quad.ppo_gpu import train_tensor

    train_tensor(args)


if __name__ == "__main__":
    main()