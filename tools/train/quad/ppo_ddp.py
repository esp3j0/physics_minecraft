"""四足 PPO 双卡数据并行(E02g D3)。

torchrun --nproc_per_node=2 -m quad.ppo_ddp …(--n-envs 为总 N)。
探针坑:单进程交替步进两个 warp 设备会 CUDA error 700,故每 rank 进程
启动时把 CUDA_VISIBLE_DEVICES 重设为仅其本卡并 re-exec(进程内只见一张卡,
backend 用 cuda:0)。PPO 数学与 ppo_gpu.train_tensor 逐字一致;policy 用
DistributedDataParallel 包裹,backward 自动 allreduce 梯度;obs-norm 每步
all_gather 各 rank 批统计后按 rank 序 Chan 合并,各 rank 逐位一致。
有效 batch = world_size × local。rank0 写 curve.csv/ckpt(聚合口径),
每 rank 写 rank_stats_<r>.json(权重 sha256/rms/分片)供一致性测试。
"""

from __future__ import annotations

import argparse
import os
import sys


def main() -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument("--backend", default="mujoco_warp")
    cli.add_argument("--device", default="cuda:0")
    cli.add_argument("--body", required=True)
    cli.add_argument("--manifest", required=True)
    cli.add_argument("--mjcf", default=None)
    cli.add_argument("--seed", type=int, default=0)
    cli.add_argument("--out", default="out/quad_ddp")
    cli.add_argument("--n-envs", type=int, default=4096, help="总 N(各 rank 均分)")
    cli.add_argument("--threads", type=int, default=8)
    cli.add_argument("--torch-threads", type=int, default=4)
    cli.add_argument("--episode-controls", type=int, default=600)
    cli.add_argument("--intent-range", type=float, nargs=6, default=[0.0] * 6,
                     metavar=("lo0", "hi0", "lo1", "hi1", "lo2", "hi2"))
    cli.add_argument("--num-steps", type=int, default=48)
    cli.add_argument("--total-timesteps", type=int, default=10_000_000, help="聚合预算")
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

    # 隔离须在任何 CUDA/warp 初始化之前:re-exec 后进程只见本卡。
    if os.environ.get("E02G_ISOLATED") != "1":
        local_rank = int(os.environ.get("LOCAL_RANK", "0"))
        env = dict(os.environ)
        env["CUDA_VISIBLE_DEVICES"] = str(local_rank)
        env["E02G_ISOLATED"] = "1"
        os.execve(sys.executable, [sys.executable, "-u", *sys.argv], env)

    _run(args)


def _run(args) -> None:
    import hashlib
    import json
    import time

    import numpy as np
    import torch
    import torch.distributed as dist
    import torch.nn as nn
    import torch.optim as optimizer
    from torch.nn.parallel import DistributedDataParallel

    from types import SimpleNamespace

    from quad.ppo import Agent, load_checkpoint, save_checkpoint
    from quad.ppo_gpu import TorchRunningMeanStd, _plain_spec
    from quad.backends.tensor import make_tensor_backend
    from quad.reward import reward_gpu

    rank = int(os.environ.get("RANK", "0"))
    ws = int(os.environ.get("WORLD_SIZE", "1"))
    os.environ.setdefault("MASTER_ADDR", "127.0.0.1")
    os.environ.setdefault("MASTER_PORT", "29500")
    if args.n_envs % ws != 0:
        raise SystemExit(f"总 N={args.n_envs} 不能被 world_size={ws} 整除")
    local_n = args.n_envs // ws

    torch.cuda.set_device(0)
    device = torch.device("cuda:0")
    dist.init_process_group("nccl", rank=rank, world_size=ws)
    torch.set_num_threads(args.torch_threads)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    # 统一后端工厂(E02h D3);DDP 是 GPU 路径,每 rank 隔离后只见 cuda:0。
    local_args = SimpleNamespace(**vars(args))
    local_args.n_envs = local_n
    local_args.device = "cuda:0"
    backend = make_tensor_backend(local_args)
    spec_dict = _plain_spec(backend.spec_dict)
    obs_dim = spec_dict["obs_dim"]
    act_dim = spec_dict["act_dim"]

    class DDPActor(Agent):
        def forward(self, x, action=None):
            return self.get_action_and_value(x, action)

    agent = DDPActor(obs_dim, act_dim).to(device)
    rms = TorchRunningMeanStd(obs_dim, device)
    start_steps = 0
    init_iter = 0
    if args.init_from:
        base, opt0, rms_np, _, _, start_steps = load_checkpoint(
            args.init_from, device, spec_dict
        )
        agent.load_state_dict(base.state_dict())
        del base, opt0
        rms.mean = torch.as_tensor(rms_np.mean, dtype=torch.float64, device=device)
        rms.var = torch.as_tensor(rms_np.var, dtype=torch.float64, device=device)
        rms.count = float(rms_np.count)
        init_iter = start_steps // (args.n_envs * args.num_steps)
    # PPO clip 全触发时 actor 梯度整体为零,默认 reducer 会误报未结束;
    # find_unused_parameters 把零梯度 rank 按 0 参与 allreduce(均值损失真梯度语义)。
    policy = DistributedDataParallel(agent, device_ids=[0], find_unused_parameters=True)
    opt = optimizer.Adam(agent.parameters(), lr=args.lr, eps=1e-5)

    # 模型初始化各 rank 相同(DDP 构造时广播 rank0);rollout 采样各 rank 不同。
    torch.manual_seed(args.seed + 1000 * rank + 1)
    np.random.seed(args.seed + 1000 * rank + 1)

    batch_size = local_n * args.num_steps
    minibatch_size = batch_size // args.num_minibatches

    if rank == 0:
        os.makedirs(args.out, exist_ok=True)
        with open(os.path.join(args.out, "curve.csv"), "w", encoding="utf-8") as f:
            f.write("iter,total_steps,ep_rew_mean,ep_len_mean,fall_frac,steps_per_sec\n")
    dist.barrier()

    obs = backend.reset(args.seed + rank)
    obs_buf = torch.zeros((args.num_steps, local_n, obs_dim), device=device)
    act_buf = torch.zeros((args.num_steps, local_n, act_dim), device=device)
    logp_buf = torch.zeros((args.num_steps, local_n), device=device)
    val_buf = torch.zeros((args.num_steps, local_n), device=device)
    rew_buf = torch.zeros((args.num_steps, local_n), device=device)
    done_buf = torch.zeros((args.num_steps, local_n), device=device)
    ep_rew = torch.zeros(local_n, dtype=torch.float64, device=device)
    ep_len = torch.zeros(local_n, dtype=torch.int64, device=device)

    def rms_update_sync(x: torch.Tensor) -> None:
        xd = x.double()
        bm = xd.mean(dim=0)
        bv = xd.var(dim=0, unbiased=False)
        bc = torch.full((obs_dim,), float(x.shape[0]), dtype=torch.float64, device=device)
        stats = torch.stack([bm, bv, bc])
        gathered = [torch.empty_like(stats) for _ in range(ws)]
        dist.all_gather(gathered, stats)
        for g in gathered:
            b_mean, b_var, b_count = g[0], g[1], float(g[2, 0])
            delta = b_mean - rms.mean
            tot = rms.count + b_count
            rms.mean = rms.mean + delta * b_count / tot
            m2 = rms.var * rms.count + b_var * b_count + delta**2 * rms.count * b_count / tot
            rms.var = m2 / tot
            rms.count = tot

    global_step = start_steps
    t0 = time.time()
    iterno = init_iter
    stop = False
    next_done = torch.ones(local_n, device=device)
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
                rms_update_sync(obs)

        # ── GAE(同 ppo_gpu)──
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

        # ── PPO 更新(同 ppo_gpu;forward 走 DDP,backward allreduce)──
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
                _, newlogprob, entropy, newvalue = policy(b_obs[mb_idx], b_actions[mb_idx])
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
            if args.target_kl is not None:
                local_break = float(torch.stack(kls[-args.num_minibatches:]).mean()) > args.target_kl
                flag = torch.tensor([float(local_break)], device=device)
                dist.all_reduce(flag, op=dist.ReduceOp.MAX)
                if bool(flag.item()):
                    break

        # ── 记录(rank0 聚合口径)──
        for t in (fin_rew_sum, fin_len_sum, fall_count, end_count):
            dist.all_reduce(t, op=dist.ReduceOp.SUM)
        elapsed = time.time() - t0
        iter_dt = max(1e-9, time.time() - iter_t0)
        sps = ws * local_n * args.num_steps / iter_dt
        n_end = int(end_count)
        rew_mean = float(fin_rew_sum / end_count) if n_end else float("nan")
        len_mean = float(fin_len_sum.double() / end_count) if n_end else float("nan")
        fall_frac = float(fall_count.double() / end_count) if n_end else float("nan")
        if rank == 0:
            with open(os.path.join(args.out, "curve.csv"), "a", encoding="utf-8") as f:
                f.write(
                    f"{iterno},{global_step},{rew_mean:.4f},{len_mean:.2f},"
                    f"{fall_frac:.4f},{sps:.1f}\n"
                )
            if iterno % args.report_every == 0 or iterno == init_iter + 1:
                print(
                    f"[ddp] iter {iterno} rew {rew_mean:.3f} len {len_mean:.1f} "
                    f"fall {fall_frac:.3f} clip {float(clip_sum / n_mb):.3f} "
                    f"kl {float(torch.stack(kls).mean()):.4f} vf {float(vf_sum / n_mb):.4f} "
                    f"steps {global_step} {elapsed:.0f}s {sps:.0f}步/s(聚合)",
                    flush=True,
                )
                save_checkpoint(
                    os.path.join(args.out, "ckpt_last.pt"),
                    agent, opt, rms.to_numpy_rms(), args, spec_dict, global_step,
                )

        stop_local = (
            global_step - start_steps >= args.total_timesteps
            or elapsed >= args.max_minutes * 60.0
        )
        flag = torch.tensor([float(stop_local)], device=device)
        dist.all_reduce(flag, op=dist.ReduceOp.MAX)
        stop = bool(flag.item())

    if rank == 0:
        save_checkpoint(
            os.path.join(args.out, "ckpt_final.pt"),
            agent, opt, rms.to_numpy_rms(), args, spec_dict, global_step,
        )
    sha = hashlib.sha256()
    for p in agent.parameters():
        sha.update(p.detach().cpu().numpy().tobytes())
    stats = {
        "rank": rank,
        "world_size": ws,
        "n_envs": local_n,
        "cuda_device_count": torch.cuda.device_count(),
        "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
        "param_sha256": sha.hexdigest(),
        "obs_mean": rms.mean.detach().cpu().tolist(),
        "obs_var": rms.var.detach().cpu().tolist(),
        "obs_count": rms.count,
        "aggregate_steps": global_step,
    }
    with open(os.path.join(args.out, f"rank_stats_{rank}.json"), "w", encoding="utf-8") as f:
        json.dump(stats, f, indent=2)
    dist.barrier()
    if rank == 0:
        print(
            f"[ddp] train done: {args.out} iters {iterno} steps {global_step} "
            f"elapsed {time.time() - t0:.0f}s(预算 {args.total_timesteps} 步 / {args.max_minutes} 分钟)",
            flush=True,
        )
    dist.destroy_process_group()


if __name__ == "__main__":
    main()