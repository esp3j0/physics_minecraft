"""四足 PPO(E02b-2 §2)——以 train/ppo.py 为底本复制后改。

来源:CleanRL ppo_continuous_action.py(MIT);底本的 GAE、done 处理、
归一化冻结逻辑保持原样(那份调试过)。改动只有:
1. 维度/后端/spec 全部来自后端 spec_dict(不再 import train.spec);奖励用
   quad.task.reward(只读 reward_inputs);
2. 网络:actor/critic 各 3 层 256 单元 ELU(AC-D4 典型规模);actor 输出
   act_dim 维均值,可学习 log_std(初值 -0.5);
3. 检查点带 spec_dict;--init-from 用 quad.spec.spec_matches 比(忽略
   episode_controls、fall_up_y_min),不一致拒绝并列出不一致键;
4. curve.csv 每 --report-every 迭代追加:迭代号、累计步数、平均回合回报、
   平均回合长度、摔倒占结束回合比例、每秒步数。

用法:
  python -m quad.ppo --backend rapier --body ../../assets/bodies/quad.body.toml \
      --manifest quad/manifests/quad_train.toml --n-envs 64 --threads 8 \
      --max-minutes 3 --out out/quad_smoke
"""

from __future__ import annotations

import argparse
import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optimizer

from quad import task
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


def make_backend(args):
    if args.backend == "mujoco":
        raise SystemExit("mujoco 后端未实现(下一单)")
    if args.backend != "rapier":
        raise SystemExit(f"未知后端 `{args.backend}`(只支持 rapier|mujoco)")
    from quad.backends.rapier import RapierBackend

    lo = args.intent_range
    intent_range = [[lo[0], lo[1]], [lo[2], lo[3]], [lo[4], lo[5]]]
    return RapierBackend(
        args.body,
        args.manifest,
        n_envs=args.n_envs,
        threads=args.threads,
        episode_controls=args.episode_controls,
        intent_range=intent_range,
    )


def main() -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument("--backend", default="rapier", help="rapier|mujoco(mujoco 未实现)")
    cli.add_argument("--body", required=True)
    cli.add_argument("--manifest", required=True)
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

    torch.set_num_threads(args.torch_threads)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = torch.device("cpu")

    backend = make_backend(args)
    spec_dict = backend.spec_dict
    obs_dim = spec_dict["obs_dim"]
    act_dim = spec_dict["act_dim"]

    agent = Agent(obs_dim, act_dim).to(device)
    opt = optimizer.Adam(agent.parameters(), lr=args.lr, eps=1e-5)
    rms = RunningMeanStd(obs_dim)
    start_steps = 0
    init_iter = 0
    if args.init_from:
        agent, opt, rms, _, _, start_steps = load_checkpoint(args.init_from, device, spec_dict)
        init_iter = start_steps // (args.n_envs * args.num_steps)
        print(f"--init-from {args.init_from}:从总步 {start_steps}(≈迭代 {init_iter})继续")
        if args.freeze_obs_norm:
            print("观测归一化已冻结(--freeze-obs-norm)")
    batch_size = args.n_envs * args.num_steps
    minibatch_size = batch_size // args.num_minibatches

    os.makedirs(args.out, exist_ok=True)
    curve_path = os.path.join(args.out, "curve.csv")
    with open(curve_path, "w", encoding="utf-8") as f:
        f.write("iter,total_steps,ep_rew_mean,ep_len_mean,fall_frac,steps_per_sec\n")

    obs = backend.reset(args.seed)
    obs_buf = np.zeros((args.num_steps, args.n_envs, obs_dim), dtype=np.float32)
    act_buf = np.zeros((args.num_steps, args.n_envs, act_dim), dtype=np.float32)
    logp_buf = np.zeros((args.num_steps, args.n_envs), dtype=np.float32)
    val_buf = np.zeros((args.num_steps, args.n_envs), dtype=np.float32)
    rew_buf = np.zeros((args.num_steps, args.n_envs), dtype=np.float32)
    done_buf = np.zeros((args.num_steps, args.n_envs), dtype=np.float32)
    ep_rew = np.zeros(args.n_envs, dtype=np.float64)
    ep_len = np.zeros(args.n_envs, dtype=np.int64)

    global_step = start_steps
    t0 = time.time()
    iterno = init_iter
    stop = False
    next_done = np.ones(args.n_envs, dtype=np.float32)
    while not stop:
        if args.anneal_lr and not args.init_from:
            frac = 1.0 - (global_step - start_steps) / max(1, args.total_timesteps - start_steps)
            opt.param_groups[0]["lr"] = frac * args.lr

        iterno += 1
        iter_t0 = time.time()
        fin_rew: list[float] = []
        fin_len: list[int] = []
        fall_count = 0
        end_count = 0
        for step in range(args.num_steps):
            global_step += args.n_envs
            o_norm = rms.normalize(obs)
            obs_buf[step] = o_norm
            done_buf[step] = next_done
            with torch.no_grad():
                t_o = torch.from_numpy(o_norm).to(device)
                a, logp, _, v = agent.get_action_and_value(t_o)
            act_buf[step] = a.numpy()
            logp_buf[step] = logp.numpy()
            val_buf[step] = v.numpy().reshape(-1)
            obs, ri, terminated, truncated = backend.step(act_buf[step])
            r = task.reward(ri)
            rew_buf[step] = r * args.reward_scale
            next_done = np.maximum(
                terminated.astype(np.float32), truncated.astype(np.float32)
            )
            ep_rew += r
            ep_len += 1
            newly = next_done.astype(bool)
            if newly.any():
                for i in np.nonzero(newly)[0]:
                    fin_rew.append(float(ep_rew[i]))
                    fin_len.append(int(ep_len[i]))
                    end_count += 1
                    if terminated[i]:
                        fall_count += 1
                ep_rew[newly] = 0.0
                ep_len[newly] = 0
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

        # ── PPO 更新(CleanRL 原样)──
        b_obs = torch.from_numpy(obs_buf.reshape((-1, obs_dim))).to(device)
        b_actions = torch.from_numpy(act_buf.reshape((-1, act_dim))).to(device)
        b_logprobs = torch.from_numpy(logp_buf.reshape(-1)).to(device)
        b_advantages = advantages.reshape(-1)
        b_returns = returns.reshape(-1)
        b_values = val_t.reshape(-1)

        b_index = np.arange(batch_size)
        np.random.shuffle(b_index)
        clipfracs = []
        vf_mses = []
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
                    v_loss_max = torch.max(v_loss, 0.5 * ((v_clipped - b_returns[mb_idx]) ** 2))
                v_loss = v_loss_max.mean()
                entropy_loss = entropy.mean()
                loss = pg_loss - args.ent_coef * entropy_loss + args.vf_coef * v_loss
                with torch.no_grad():
                    clipfracs.append(float(((ratio - 1).abs() > args.clip_coef).float().mean()))
                    vf_mses.append(float(v_loss))
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), args.max_grad_norm)
                opt.step()
            if args.target_kl is not None and float(np.mean(kls[-args.num_minibatches:])) > args.target_kl:
                break

        # ── 记录(curve.csv 每迭代追加一行)──
        elapsed = time.time() - t0
        iter_steps = args.n_envs * args.num_steps
        sps = iter_steps / max(1e-9, time.time() - iter_t0)
        rew_mean = float(np.mean(fin_rew)) if fin_rew else float("nan")
        len_mean = float(np.mean(fin_len)) if fin_len else float("nan")
        fall_frac = fall_count / end_count if end_count else float("nan")
        with open(curve_path, "a", encoding="utf-8") as f:
            f.write(
                f"{iterno},{global_step},{rew_mean:.4f},{len_mean:.2f},"
                f"{fall_frac:.4f},{sps:.1f}\n"
            )
        if iterno % args.report_every == 0 or iterno == init_iter + 1:
            print(
                f"iter {iterno} rew {rew_mean:.3f} len {len_mean:.1f} fall {fall_frac:.3f} "
                f"clip {np.mean(clipfracs):.3f} kl {np.mean(kls):.4f} vf {np.mean(vf_mses):.4f} "
                f"steps {global_step} {elapsed:.0f}s {sps:.0f}步/s",
                flush=True,
            )
            save_checkpoint(
                os.path.join(args.out, "ckpt_last.pt"),
                agent, opt, rms, args, spec_dict, global_step,
            )

        stop = (
            global_step - start_steps >= args.total_timesteps
            or elapsed >= args.max_minutes * 60.0
        )

    save_checkpoint(
        os.path.join(args.out, "ckpt_final.pt"), agent, opt, rms, args, spec_dict, global_step
    )
    print(
        f"train done: {args.out} iters {iterno} steps {global_step} "
        f"elapsed {time.time() - t0:.0f}s(预算 {args.total_timesteps} 步 / {args.max_minutes} 分钟)",
        flush=True,
    )


if __name__ == "__main__":
    main()