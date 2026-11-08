"""PPO(CleanRL 底本改写)——REQ-012 A00-5 R2。

来源:CleanRL `ppo_continuous_action.py`(MIT 许可,https://github.com/vwxyzjn/cleanrl),
文件头注明来源(R2 要求)。算法主体(网络、GAE、clip 目标、优化循环)
保持 CleanRL 原样;改动只有两处(放行允许):
1. 环境接口:直接用后端无关的 VecBackend(tools/train/train/backends/base.py),
   不走 gymnasium 的向量包装;
2. 奖励:train/rewards.py 按 reward_inputs 计算(legged_gym 常用组合的
   摆臂版:−距离 − 小权重×动作变化 + 到达奖励)。

检查点(R2 补充):actor、critic、log σ、观测归一化统计、优化器状态、
spec、超参,一包存尽;`--init-from` 可以跨后端(spec 不一致就拒绝,
见 main 的 load_checkpoint)。

用法:
  python -m train.ppo --backend rapier --seed 0 --out out/arm_s0
  python -m train.ppo --backend rapier --seed 1 --hz60 --out out/arm_s1_hz60
  python -m train.ppo --init-from out/arm_s0/ckpt_final.pt --out out/arm_ft
预算:1e7 控制步或 30 分钟墙钟,先到为准(R2 再补充)。
"""

from __future__ import annotations

import argparse
import os
import time

import numpy as np

import torch
import torch.nn as nn
import torch.optim as optimizer

from train import spec
from train.rewards import reward

# 来源声明(R2:以 CleanRL ppo_continuous_action.py 为底本,MIT)。
SOURCE = "Adapted from CleanRL ppo_continuous_action.py (MIT License)"


def layer_init(layer: nn.Linear, std: float = np.sqrt(2).item(), bias_const: float = 0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer


class Agent(nn.Module):
    """CleanRL 原样:两个 64 隐层 tanh 网络;动作 = 状态无关可学习 log σ。"""

    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()
        self.critic = nn.Sequential(
            layer_init(nn.Linear(obs_dim, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 1), std=1.0),
        )
        self.actor_mean = nn.Sequential(
            layer_init(nn.Linear(obs_dim, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, act_dim), std=0.01),
        )
        # CleanRL 默认 zeros(σ=1);本任务动作空间 [-1,1],σ=1 时过半
        # 采样饱和在边界、梯度 dead,初始化 -0.5(σ≈0.6)(README 记录)。
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
    """观测归一化统计(Welford;CleanRL 用 numpy 同款;导出进 policy.toml)。"""

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


def save_checkpoint(path: str, agent: Agent, opt: optimizer.Optimizer, rms: RunningMeanStd, args, spec_dict: dict, total_steps: int) -> None:
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


def load_checkpoint(path: str, device) -> tuple[Agent, optimizer.Optimizer, RunningMeanStd, dict, dict, int]:
    ckpt = torch.load(path, map_location=device, weights_only=False)
    ckpt_spec = ckpt["spec"]
    if not spec.spec_matches(ckpt_spec):
        # 字段级差异报告(spec 不一致就拒绝,R2 补充)。
        cur = spec.to_dict()
        diffs = [k for k in set(cur) | set(ckpt_spec) if cur.get(k) != ckpt_spec.get(k)]
        raise SystemExit(f"--init-from 拒绝:spec 不一致,差异字段 {sorted(diffs)}")
    agent = Agent(spec.OBS_DIM, spec.ACT_DIM).to(device)
    agent.load_state_dict(ckpt["agent_state"])
    opt = optimizer.Adam(agent.parameters(), lr=3e-4, eps=1e-5)
    opt.load_state_dict(ckpt["optimizer_state"])
    rms = RunningMeanStd(spec.OBS_DIM)
    rms.mean = ckpt["obs_mean"]
    rms.var = ckpt["obs_var"]
    rms.count = ckpt["obs_count"]
    return agent, opt, rms, ckpt["args"], ckpt_spec, ckpt["total_steps"]


def main() -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument("--backend", default="rapier")
    cli.add_argument("--seed", type=int, default=0)
    cli.add_argument("--hz60", action="store_true", help="60 Hz 对照(每拍设目标)")
    cli.add_argument("--out", default="out/arm")
    cli.add_argument("--n-envs", type=int, default=128, help="并行环境(R2 再补充 64–256)")
    cli.add_argument("--threads", type=int, default=8, help="环境线程数")
    cli.add_argument("--torch-threads", type=int, default=4, help="torch.set_num_threads")
    cli.add_argument("--total-timesteps", type=int, default=10_000_000, help="预算:1e7(R2 再补充)")
    cli.add_argument("--max-minutes", type=float, default=30.0, help="预算:30 分钟墙钟,先到为准")
    cli.add_argument("--num-steps", type=int, default=90, help="每次迭代每环境收集的控制步 = 一个回合长")
    cli.add_argument("--num-minibatches", type=int, default=4)
    cli.add_argument("--update-epochs", type=int, default=4)
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
    cli.add_argument("--init-from", default=None, help="从检查点继续训练(可换后端,spec 不一致拒绝)")
    cli.add_argument("--freeze-obs-norm", action="store_true", help="微调时冻结归一化统计")
    cli.add_argument("--freeze-norm-after", type=int, default=10, help="预热 N 迭代后冻结归一化统计(V 目标不漂移)")
    cli.add_argument("--reward-scale", type=float, default=0.01,
                     help="奖励缩放(进 buffer 前;回合 return 尺度 -100 级 → critic 可拟合;legged_gym 惯例)")
    cli.add_argument("--mass-mult", type=float, default=1.0, help="L5:连杆质量倍数(只改物理,spec 不变)")
    cli.add_argument("--gain-mult", type=float, default=1.0, help="L5:增益倍数 kp/kd 同乘(ζ 不变;spec 不变)")
    cli.add_argument("--report-every", type=int, default=10, help="每 N 迭代写曲线/检查点")
    args = cli.parse_args()

    torch.set_num_threads(args.torch_threads)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = torch.device("cpu")

    from train.backends import make_backend

    backend = make_backend(
        args.backend,
        n_envs=args.n_envs,
        threads=args.threads,
        seed=args.seed,
        hz60=args.hz60,
        mass_mult=args.mass_mult,
        gain_mult=args.gain_mult,
    )

    agent = Agent(spec.OBS_DIM, spec.ACT_DIM).to(device)
    opt = optimizer.Adam(agent.parameters(), lr=args.lr, eps=1e-5)
    rms = RunningMeanStd(spec.OBS_DIM)
    start_steps = 0
    init_iter = 0
    resume_args = {}
    if args.init_from:
        agent, opt, rms, resume_args, _, start_steps = load_checkpoint(args.init_from, device)
        init_iter = start_steps // (args.n_envs * args.num_steps)
        print(f"--init-from {args.init_from}:从总步 {start_steps}(≈迭代 {init_iter})继续")
        if args.freeze_obs_norm:
            print("观测归一化已冻结(--freeze-obs-norm)")
    batch_size = args.n_envs * args.num_steps
    minibatch_size = batch_size // args.num_minibatches

    os.makedirs(args.out, exist_ok=True)
    curve_path = os.path.join(args.out, "train.csv")
    curve_header = (
        "iter,reward_mean,dist_mean,reach_frac,ep_rew_mean,total_steps,elapsed_s,"
        "clip_frac,vf_mse,entropy,kl"
    )
    with open(curve_path, "w", encoding="utf-8") as f:
        f.write(curve_header + "\n")

    obs = backend.reset(args.seed)
    last_action = np.zeros((args.n_envs, spec.ACT_DIM), dtype=np.float32)
    # 存每环境本回合的 (obs, action, logp, value, reward, done)(CleanRL 的
    # rollout buffer,布局 [steps, n_envs, ...])。
    obs_buf = np.zeros((args.num_steps, args.n_envs, spec.OBS_DIM), dtype=np.float32)
    act_buf = np.zeros((args.num_steps, args.n_envs, spec.ACT_DIM), dtype=np.float32)
    logp_buf = np.zeros((args.num_steps, args.n_envs), dtype=np.float32)
    val_buf = np.zeros((args.num_steps, args.n_envs), dtype=np.float32)
    rew_buf = np.zeros((args.num_steps, args.n_envs), dtype=np.float32)
    done_buf = np.zeros((args.num_steps, args.n_envs), dtype=np.float32)
    ep_rew = np.zeros(args.n_envs, dtype=np.float64)
    dist_sum = 0.0
    reach_count = 0
    step_count = 0
    ep_rew_mean = float("nan")  # 平滑回合奖励(报告用)

    global_step = start_steps
    t0 = time.time()
    iterno = init_iter
    stop = False
    next_done = np.zeros(args.n_envs, dtype=np.float32)
    # 自动迭代:先写 rollout 再更新(CleanRL 结构;退出条件在循环头检查)。
    while not stop:
        if args.anneal_lr and not args.init_from:
            frac = 1.0 - (global_step - start_steps) / max(1, args.total_timesteps - start_steps)
            lrnow = frac * args.lr
            opt.param_groups[0]["lr"] = lrnow

        iterno += 1
        # ── 收集一个 rollout(= num_steps × n_envs 控制步)──
        for step in range(args.num_steps):
            global_step += args.n_envs
            o_norm = rms.normalize(obs)
            obs_buf[step] = o_norm  # 网络输入口径(CleanRL:b_obs 即网络输入)
            with torch.no_grad():
                t_o = torch.from_numpy(o_norm).to(device)
                a, logp, _, v = agent.get_action_and_value(t_o)
            act_buf[step] = a.numpy()
            logp_buf[step] = logp.numpy()
            val_buf[step] = v.numpy().reshape(-1)
            obs, ri, terminated, truncated = backend.step(act_buf[step])
            r = reward(ri, act_buf[step], last_action, step)
            # truncated 步(自动 reset)的奖励不可靠(reset 后首步的
            # reward_inputs):置 done 并用 0 奖励 —— CleanRL 的 done 语义
            # 本来就跨回合。
            r = np.where(truncated, 0.0, r)
            rew_buf[step] = r * args.reward_scale
            done_buf[step] = np.maximum(terminated.astype(np.float32), truncated.astype(np.float32))
            last_action = act_buf[step].copy()
            dist_sum += float(ri["dist"][:, 0].sum())
            reach_count += int((ri["dist"][:, 0] <= spec.REACH_DIST).sum())
            step_count += args.n_envs
            ep_rew += r
            newly = done_buf[step].astype(bool)
            if newly.any():
                m = ep_rew[newly].mean()
                ep_rew_mean = m if np.isnan(ep_rew_mean) else 0.95 * ep_rew_mean + 0.05 * m
                ep_rew[newly] = 0.0
            if not args.freeze_obs_norm and iterno <= args.freeze_norm_after:
                rms.update(obs)

        # ── GAE(CleanRL 原样)──
        with torch.no_grad():
            o_norm = rms.normalize(obs)
            next_value = agent.get_value(torch.from_numpy(o_norm).to(device)).reshape(1, -1)
            rew_t = torch.from_numpy(rew_buf).to(device)
            done_t = torch.from_numpy(done_buf).to(device)
            val_t = torch.from_numpy(val_buf).to(device)
            next_done_t = torch.from_numpy(next_done).to(device)
            advantages = torch.zeros_like(rew_t)
            lastgaelam = 0
            for step_i in reversed(range(args.num_steps)):
                if step_i == args.num_steps - 1:
                    nextnonterminal = 1.0 - next_done_t
                    nextvalues = next_value
                else:
                    nextnonterminal = 1.0 - done_t[step_i + 1]
                    nextvalues = val_t[step_i + 1]
                delta = rew_t[step_i] + args.gamma * nextvalues * nextnonterminal - val_t[step_i]
                advantages[step_i] = lastgaelam = delta + args.gamma * args.gae_lambda * nextnonterminal * lastgaelam
            returns = advantages + val_t

        # ── PPO 更新(CleanRL 原样:扁平化 + minibatch 循环)──
        b_obs = torch.from_numpy(obs_buf.reshape((-1,) + (spec.OBS_DIM,))).to(device)
        b_actions = torch.from_numpy(act_buf.reshape((-1,) + (spec.ACT_DIM,))).to(device)
        b_logprobs = torch.from_numpy(logp_buf.reshape(-1)).to(device)
        b_advantages = advantages.reshape(-1)
        b_returns = returns.reshape(-1)
        b_values = val_t.reshape(-1)

        b_index = np.arange(batch_size)
        np.random.shuffle(b_index)
        clipfracs = []
        vf_mses = []
        entropies = []
        kls = []
        for epoch in range(args.update_epochs):
            for start in range(0, batch_size, minibatch_size):
                mb_idx = torch.from_numpy(b_index[start: start + minibatch_size]).to(device)
                _, newlogprob, entropy, newvalue = agent.get_action_and_value(
                    b_obs[mb_idx], b_actions[mb_idx]
                )
                logratio = newlogprob - b_logprobs[mb_idx]
                ratio = logratio.exp()
                with torch.no_grad():
                    approx_kl = ((ratio - 1) - logratio).mean()
                    kls.append(float(approx_kl))
                if args.norm_adv:
                    adv = b_advantages[mb_idx]
                    adv = (adv - adv.mean()) / (adv.std() + 1e-8)
                else:
                    adv = b_advantages[mb_idx]
                pg_losses = -adv * ratio
                pg_loss2 = -adv * torch.clamp(ratio, 1 - args.clip_coef, 1 + args.clip_coef)
                pg_loss = torch.max(pg_losses, pg_loss2).mean()
                v_loss = 0.5 * ((newvalue.reshape(-1) - b_returns[mb_idx]) ** 2)
                with torch.no_grad():
                    v_clipped = b_values[mb_idx] + torch.clamp(
                        newvalue.reshape(-1) - b_values[mb_idx],
                        -args.clip_coef,
                        args.clip_coef,
                    )
                    v_loss_max = torch.max(
                        v_loss, 0.5 * ((v_clipped - b_returns[mb_idx]) ** 2)
                    )
                v_loss = v_loss_max.mean()
                entropy_loss = entropy.mean()
                loss = pg_loss - args.ent_coef * entropy_loss + args.vf_coef * v_loss
                with torch.no_grad():
                    clipfracs.append(float(((ratio - 1).abs() > args.clip_coef).float().mean()))
                    vf_mses.append(float(v_loss))
                    entropies.append(float(entropy_loss))
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), args.max_grad_norm)
                opt.step()
            if args.target_kl is not None and float(np.mean(kls[-args.num_minibatches:])) > args.target_kl:
                break

        # ── 记录(R2:全部正式训练都报;曲线文件每次迭代追加一行)──
        d_mean = dist_sum / max(1, step_count)
        reach_frac = reach_count / max(1, step_count)
        dist_sum = 0.0
        reach_count = 0
        step_count = 0
        elapsed = time.time() - t0
        line = (
            f"{iterno},{ep_rew_mean:.4f},{d_mean:.4f},{reach_frac:.4f},{ep_rew_mean:.4f},"
            f"{global_step},{elapsed:.1f},{np.mean(clipfracs):.4f},{np.mean(vf_mses):.6f},"
            f"{np.mean(entropies):.3f},{np.mean(kls):.5f}"
        )
        with open(curve_path, "a", encoding="utf-8") as f:
            f.write(line + "\n")
        if iterno % args.report_every == 0 or iterno == init_iter + 1:
            print(
                f"iter {iterno} rew {ep_rew_mean:.3f} clip {np.mean(clipfracs):.3f} "
                f"kl {np.mean(kls):.4f} vf {np.mean(vf_mses):.4f} steps {global_step} {elapsed:.0f}s",
                flush=True,
            )
            save_checkpoint(
                os.path.join(args.out, "ckpt_last.pt"),
                agent, opt, rms, args, spec.to_dict(), global_step,
            )

        stop = (
            global_step - start_steps >= args.total_timesteps
            or elapsed >= args.max_minutes * 60.0
        )

    save_checkpoint(
        os.path.join(args.out, "ckpt_final.pt"), agent, opt, rms, args, spec.to_dict(), global_step
    )
    print(
        f"train done: {args.out} iters {iterno} steps {global_step} "
        f"elapsed {time.time() - t0:.0f}s(预算 {args.total_timesteps} 步 / {args.max_minutes} 分钟)",
        flush=True,
    )


if __name__ == "__main__":
    main()
