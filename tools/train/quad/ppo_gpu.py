"""四足唯一张量训练循环(E02f D4;E02h D2 统一所有后端)。

--backend {rapier|mujoco|mujoco_warp} --device {cpu|cuda:0|cuda:1} 全路由到此:
rapier/mujoco 经 NumpyToTensorAdapter(零拷贝,CPU,rollout 仍 mujoco.rollout),
mujoco_warp 原生张量。rollout 缓冲、奖励、GAE、minibatch、归一化统计全部是
torch 张量(设备无关);reward 统一 reward_gpu。检查点结构与 legacy 一致
(obs_mean/var 存 numpy float64,spec 不带 torch 张量),export.py 原样可用。
"""

from __future__ import annotations

import os
import time
from types import SimpleNamespace

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optimizer

from quad.ppo import Agent, load_checkpoint, save_checkpoint
from quad.backends.tensor import make_tensor_backend
from quad.reward import reward_gpu


class TorchRunningMeanStd:
    """Welford 同 ppo.RunningMeanStd,统计量留 GPU(float64)。"""

    def __init__(self, dim: int, device):
        self.mean = torch.zeros(dim, dtype=torch.float64, device=device)
        self.var = torch.ones(dim, dtype=torch.float64, device=device)
        self.count = 1e-4

    def update(self, x: torch.Tensor) -> None:
        xd = x.double()
        batch_mean = xd.mean(dim=0)
        batch_var = xd.var(dim=0, unbiased=False)
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

    def normalize(self, x: torch.Tensor, clip: float = 10.0) -> torch.Tensor:
        return torch.clamp(
            (x - self.mean) / torch.sqrt(self.var + 1e-8), -clip, clip
        ).float()

    def to_numpy_rms(self):
        return SimpleNamespace(
            mean=self.mean.detach().cpu().numpy(),
            var=self.var.detach().cpu().numpy(),
            count=float(self.count),
        )


def _plain_spec(spec: dict) -> dict:
    return {k: v for k, v in spec.items() if not k.startswith("_")}


def train_tensor(args) -> None:
    torch.set_num_threads(args.torch_threads)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    np.random.seed(args.seed)
    device = torch.device(args.device)

    backend = make_tensor_backend(args)
    spec_dict = _plain_spec(backend.spec_dict)
    obs_dim = spec_dict["obs_dim"]
    act_dim = spec_dict["act_dim"]

    agent = Agent(obs_dim, act_dim).to(device)
    opt = optimizer.Adam(agent.parameters(), lr=args.lr, eps=1e-5)
    rms = TorchRunningMeanStd(obs_dim, device)
    start_steps = 0
    init_iter = 0
    if args.init_from:
        agent, opt, rms_np, _, _, start_steps = load_checkpoint(
            args.init_from, device, spec_dict
        )
        rms.mean = torch.as_tensor(rms_np.mean, dtype=torch.float64, device=device)
        rms.var = torch.as_tensor(rms_np.var, dtype=torch.float64, device=device)
        rms.count = float(rms_np.count)
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
    obs_buf = torch.zeros((args.num_steps, args.n_envs, obs_dim), device=device)
    act_buf = torch.zeros((args.num_steps, args.n_envs, act_dim), device=device)
    logp_buf = torch.zeros((args.num_steps, args.n_envs), device=device)
    val_buf = torch.zeros((args.num_steps, args.n_envs), device=device)
    rew_buf = torch.zeros((args.num_steps, args.n_envs), device=device)
    done_buf = torch.zeros((args.num_steps, args.n_envs), device=device)
    ep_rew = torch.zeros(args.n_envs, dtype=torch.float64, device=device)
    ep_len = torch.zeros(args.n_envs, dtype=torch.int64, device=device)

    global_step = start_steps
    t0 = time.time()
    iterno = init_iter
    stop = False
    next_done = torch.ones(args.n_envs, device=device)
    while not stop:
        if args.anneal_lr and not args.init_from:
            frac = 1.0 - (global_step - start_steps) / max(1, args.total_timesteps - start_steps)
            opt.param_groups[0]["lr"] = frac * args.lr

        iterno += 1
        iter_t0 = time.time()
        fin_rew_sum = torch.zeros((), dtype=torch.float64, device=device)
        fin_len_sum = torch.zeros((), dtype=torch.int64, device=device)
        fall_count = torch.zeros((), dtype=torch.int64, device=device)
        end_count = torch.zeros((), dtype=torch.int64, device=device)
        for step in range(args.num_steps):
            global_step += args.n_envs
            o_norm = rms.normalize(obs)
            obs_buf[step] = o_norm
            done_buf[step] = next_done
            with torch.no_grad():
                a, logp, _, v = agent.get_action_and_value(o_norm)
            act_buf[step] = a
            logp_buf[step] = logp
            val_buf[step] = v.reshape(-1)
            obs, ri, terminated, truncated = backend.step(a)
            r = reward_gpu(ri)
            rew_buf[step] = r * args.reward_scale
            next_done = (terminated | truncated).float()
            ep_rew += r.double()
            ep_len += 1
            newly = next_done.bool()
            if bool(newly.any()):
                fin_rew_sum += ep_rew[newly].sum()
                fin_len_sum += ep_len[newly].sum()
                end_count += newly.sum()
                fall_count += (newly & terminated).sum()
                ep_rew[newly] = 0.0
                ep_len[newly] = 0
            if not args.freeze_obs_norm and iterno <= args.freeze_norm_after:
                rms.update(obs)

        # ── GAE(同 CPU 路径)──
        with torch.no_grad():
            next_value = agent.get_value(rms.normalize(obs)).reshape(1, -1)
            advantages = torch.zeros_like(rew_buf)
            lastgaelam = 0
            for step_i in reversed(range(args.num_steps)):
                if step_i == args.num_steps - 1:
                    nextnonterminal = 1.0 - next_done
                    nextvalues = next_value
                else:
                    nextnonterminal = 1.0 - done_buf[step_i + 1]
                    nextvalues = val_buf[step_i + 1]
                delta = rew_buf[step_i] + args.gamma * nextvalues * nextnonterminal - val_buf[step_i]
                advantages[step_i] = lastgaelam = delta + args.gamma * args.gae_lambda * nextnonterminal * lastgaelam
            returns = advantages + val_buf

        # ── PPO 更新(同 CPU 路径)──
        b_obs = obs_buf.reshape((-1, obs_dim))
        b_actions = act_buf.reshape((-1, act_dim))
        b_logprobs = logp_buf.reshape(-1)
        b_advantages = advantages.reshape(-1)
        b_returns = returns.reshape(-1)
        b_values = val_buf.reshape(-1)

        b_index = torch.randperm(batch_size, device=device)
        clip_sum = torch.zeros((), device=device)
        vf_sum = torch.zeros((), device=device)
        n_mb = 0
        kls: list[torch.Tensor] = []
        for epoch in range(args.update_epochs):
            for start in range(0, batch_size, minibatch_size):
                mb_idx = b_index[start: start + minibatch_size]
                _, newlogprob, entropy, newvalue = agent.get_action_and_value(
                    b_obs[mb_idx], b_actions[mb_idx]
                )
                logratio = newlogprob - b_logprobs[mb_idx]
                ratio = logratio.exp()
                with torch.no_grad():
                    approx_kl = ((ratio - 1) - logratio).mean()
                    kls.append(approx_kl)
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
                    clip_sum += ((ratio - 1).abs() > args.clip_coef).float().mean()
                    vf_sum += v_loss
                n_mb += 1
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), args.max_grad_norm)
                opt.step()
            if args.target_kl is not None and float(torch.stack(kls[-args.num_minibatches:]).mean()) > args.target_kl:
                break

        # ── 记录(curve.csv 每迭代追加一行)──
        elapsed = time.time() - t0
        iter_steps = args.n_envs * args.num_steps
        sps = iter_steps / max(1e-9, time.time() - iter_t0)
        n_end = int(end_count)
        rew_mean = float(fin_rew_sum / end_count) if n_end else float("nan")
        len_mean = float(fin_len_sum.double() / end_count) if n_end else float("nan")
        fall_frac = float(fall_count.double() / end_count) if n_end else float("nan")
        with open(curve_path, "a", encoding="utf-8") as f:
            f.write(
                f"{iterno},{global_step},{rew_mean:.4f},{len_mean:.2f},"
                f"{fall_frac:.4f},{sps:.1f}\n"
            )
        if iterno % args.report_every == 0 or iterno == init_iter + 1:
            print(
                f"iter {iterno} rew {rew_mean:.3f} len {len_mean:.1f} fall {fall_frac:.3f} "
                f"clip {float(clip_sum / n_mb):.3f} kl {float(torch.stack(kls).mean()):.4f} "
                f"vf {float(vf_sum / n_mb):.4f} "
                f"steps {global_step} {elapsed:.0f}s {sps:.0f}步/s",
                flush=True,
            )
            save_checkpoint(
                os.path.join(args.out, "ckpt_last.pt"),
                agent, opt, rms.to_numpy_rms(), args, spec_dict, global_step,
            )

        stop = (
            global_step - start_steps >= args.total_timesteps
            or elapsed >= args.max_minutes * 60.0
        )

    save_checkpoint(
        os.path.join(args.out, "ckpt_final.pt"),
        agent, opt, rms.to_numpy_rms(), args, spec_dict, global_step,
    )
    print(
        f"train done: {args.out} iters {iterno} steps {global_step} "
        f"elapsed {time.time() - t0:.0f}s(预算 {args.total_timesteps} 步 / {args.max_minutes} 分钟)",
        flush=True,
    )