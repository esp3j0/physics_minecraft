"""E02f D5 双后端对照:CPU mujoco(threads=10)vs GPU mujoco-warp。
E02g D2 扩展:--ns 支持到 65536,bp/bt 报峰值显存(torch 峰值 + nvidia-smi
轮询峰值),bp 按 --bench-sim-steps 自适应控制步数,大 N bt 限迭代预算。

阶段(--phases,逗号分隔):
  a   行为 parity:同策略(新初始化 Agent 的均值动作)同种子前 K 控制步,
      obs/ri 逐键 max abs diff,只报不判。
  bp  纯物理吞吐 sim-steps/s(GPU 直接 mw.step 循环;CPU 每线程 mj_step)。
  bt  端到端训练 control-steps/s:子进程跑 ppo.py,读 curve.csv 稳态均值。
  c   迁移:bt 的 1024 检查点 export + eval.py(rapier),报速度/存活差。
  report  汇总各阶段 json → report.md。

用法(vm-ai,repo 根):
  uv run --offline --with onnx --with onnxscript python -m tools.train.quad.compare_backends \
      --phases a,bp --out target/gpu-compare
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import mujoco
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[3]


def _p(s: str) -> str:
    p = Path(s)
    return str(p if p.is_absolute() else ROOT / p)


def _save(out: str, name: str, obj) -> None:
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, name), "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)


def _load(out: str, name: str):
    with open(os.path.join(out, name), encoding="utf-8") as f:
        return json.load(f)


def _smi_used_mib(idx: int):
    try:
        p = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits",
             "-i", str(idx)],
            capture_output=True, text=True, timeout=10,
        )
        return float(p.stdout.strip().splitlines()[0])
    except Exception:
        return None


class _SmiPeak:
    """后台轮询 nvidia-smi 显存占用峰值(warp 分配不在 torch  allocator 内)。"""

    def __init__(self, idx: int, interval: float = 0.2):
        self.idx = idx
        self.interval = interval
        self.peak = None
        self._stop = False
        self._thread = None

    def __enter__(self):
        import threading

        def loop():
            while not self._stop:
                v = _smi_used_mib(self.idx)
                if v is not None:
                    self.peak = v if self.peak is None else max(self.peak, v)
                time.sleep(self.interval)

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop = True
        if self._thread is not None:
            self._thread.join(timeout=5)
        return False


def _dev_idx(device: str) -> int:
    return int(device.split(":")[-1]) if ":" in device else 0


def _bench_controls(args, n: int, nstep: int) -> int:
    target = args.bench_sim_steps
    return max(5, min(args.bench_controls, round(target / (n * nstep))))


def phase_a(args) -> None:
    from quad.ppo import Agent
    from quad.backends.mujoco import MujocoBackend
    from quad.backends.mujoco_warp import MujocoWarpBackend

    n = args.ns[0]
    ir = [[args.intent_range[0], args.intent_range[1]],
          [args.intent_range[2], args.intent_range[3]],
          [args.intent_range[4], args.intent_range[5]]]
    cpu = MujocoBackend(args.body, args.manifest, n_envs=n, threads=args.threads,
                        episode_controls=args.episode_controls, intent_range=ir,
                        mjcf_path=args.mjcf)
    gpu = MujocoWarpBackend(args.body, args.manifest, n_envs=n,
                            episode_controls=args.episode_controls, intent_range=ir,
                            mjcf_path=args.mjcf, device=args.device)
    torch.manual_seed(args.seed)
    agent = Agent(cpu.obs_dim, cpu.act_dim)
    agent.eval()

    def policy(obs_np):
        with torch.no_grad():
            return agent.actor_mean(torch.from_numpy(obs_np)).numpy()

    obs_c = cpu.reset(args.seed)
    obs_g = gpu.reset(args.seed).cpu().numpy()
    keys = None
    max_diff: dict[str, float] = {}
    first_diff: dict[str, float] = {}
    last_diff: dict[str, float] = {}
    for k in range(args.k):
        a_c = policy(obs_c)
        a_g = policy(obs_g)
        obs_c, ri_c, _, _ = cpu.step(a_c)
        obs_g_t, ri_g, _, _ = gpu.step(torch.from_numpy(a_g).to(args.device))
        obs_g = obs_g_t.cpu().numpy()
        d = {"obs": float(np.abs(obs_c - obs_g).max())}
        if keys is None:
            keys = sorted(ri_c.keys())
        for key in keys:
            d[key] = float(np.abs(ri_c[key] - ri_g[key].cpu().numpy()).max())
        for key, v in d.items():
            max_diff[key] = max(max_diff.get(key, 0.0), v)
            last_diff[key] = v
            if k == 0:
                first_diff[key] = v
    res = {"n_envs": n, "k": args.k, "seed": args.seed,
           "first_step": first_diff, "last_step": last_diff, "max_over_k": max_diff}
    _save(args.out, "results_a.json", res)
    print(f"(a) n={n} K={args.k} obs max diff {max_diff['obs']:.6g}(首步 {first_diff['obs']:.6g})")


def _gpu_pure_physics(args, n: int, reps: int = 3) -> dict:
    import warp as wp
    from quad.backends.mujoco_warp import MujocoWarpBackend
    import mujoco_warp as mw

    ir = [[args.intent_range[0], args.intent_range[1]],
          [args.intent_range[2], args.intent_range[3]],
          [args.intent_range[4], args.intent_range[5]]]
    be = MujocoWarpBackend(args.body, args.manifest, n_envs=n,
                           episode_controls=args.episode_controls, intent_range=ir,
                           mjcf_path=args.mjcf, device=args.device)
    be.reset(args.seed)
    nstep = be.control_period_ticks * be.substeps
    bc = _bench_controls(args, n, nstep)
    total = n * bc * nstep
    rates = []
    torch.cuda.reset_peak_memory_stats()
    with _SmiPeak(_dev_idx(args.device)) as smi:
        for _ in range(reps):
            for _ in range(5 * nstep):
                mw.step(be._m, be._d)
            wp.synchronize()
            t0 = time.perf_counter()
            for _ in range(bc * nstep):
                mw.step(be._m, be._d)
            wp.synchronize()
            rates.append(total / (time.perf_counter() - t0))
    res = {
        "rate": float(np.median(rates)),
        "bench_controls": bc,
        "torch_peak_alloc_mib": torch.cuda.max_memory_allocated() / 2**20,
        "torch_peak_reserved_mib": torch.cuda.max_memory_reserved() / 2**20,
        "smi_peak_mib": smi.peak,
    }
    del be
    torch.cuda.empty_cache()
    return res


def _cpu_pure_physics(args, n: int, reps: int = 3) -> float:
    from quad.backends.mujoco import MujocoBackend

    ir = [[args.intent_range[0], args.intent_range[1]],
          [args.intent_range[2], args.intent_range[3]],
          [args.intent_range[4], args.intent_range[5]]]
    be = MujocoBackend(args.body, args.manifest, n_envs=n, threads=args.threads,
                       episode_controls=args.episode_controls, intent_range=ir,
                       mjcf_path=args.mjcf)
    m = be._model
    nstep = be.control_period_ticks * be.substeps
    bc = _bench_controls(args, n, nstep)
    chunk = math.ceil(n / args.threads)
    total = args.threads * chunk * bc * nstep
    loops = chunk * bc * nstep

    def worker(_):
        d = mujoco.MjData(m)
        mujoco.mj_setState(m, d, be._initial, mujoco.mjtState.mjSTATE_FULLPHYSICS.value)
        d.ctrl[:] = m.key_ctrl[0]
        for _ in range(loops):
            mujoco.mj_step(m, d)

    rates = []
    for _ in range(reps):
        t0 = time.perf_counter()
        with ThreadPoolExecutor(max_workers=args.threads) as ex:
            list(ex.map(worker, range(args.threads)))
        rates.append(total / (time.perf_counter() - t0))
    return float(np.median(rates))


def phase_bp(args) -> None:
    res = {}
    if os.path.exists(os.path.join(args.out, "results_bp.json")):
        res = _load(args.out, "results_bp.json")
    for n in args.ns:
        g = _gpu_pure_physics(args, n)
        c = _cpu_pure_physics(args, n)
        res[str(n)] = {"gpu_sim_steps_per_sec": g["rate"], "cpu_sim_steps_per_sec": c,
                       "ratio": g["rate"] / c, "threads": args.threads,
                       "bench_controls": g["bench_controls"],
                       "torch_peak_alloc_mib": g["torch_peak_alloc_mib"],
                       "torch_peak_reserved_mib": g["torch_peak_reserved_mib"],
                       "smi_peak_mib": g["smi_peak_mib"]}
        print(f"(bp) N={n} GPU {g['rate']:.3e} CPU({args.threads}t) {c:.3e} sim-steps/s "
              f"ratio {g['rate'] / c:.2f} VRAM(smi峰值) {g['smi_peak_mib']}MiB "
              f"bc={g['bench_controls']}")
    _save(args.out, "results_bp.json", res)


def _train_cmd(args, backend: str, n: int, total: int, outdir: str) -> list[str]:
    cmd = [sys.executable, "-m", "tools.train.quad.ppo",
           "--backend", backend, "--body", args.body, "--manifest", args.manifest,
           "--mjcf", args.mjcf, "--n-envs", str(n), "--seed", str(args.seed),
           "--episode-controls", str(args.episode_controls),
           "--num-steps", str(args.num_steps),
           "--total-timesteps", str(total), "--max-minutes", str(args.train_minutes),
           "--intent-range", *[str(x) for x in args.intent_range], "--out", outdir]
    if backend in ("mujoco", "rapier"):
        cmd += ["--threads", str(args.threads), "--device", "cpu"]
    else:
        cmd += ["--device", args.device]
    return cmd


def _run_train(args, backend: str, n: int, total: int) -> dict:
    outdir = os.path.join(args.out, f"train_{backend}_{n}")
    os.makedirs(outdir, exist_ok=True)
    # warp-cpu × CUDA 可见会堆损坏(E02g 上游坑):CPU warp 跑一律 CVD=""。
    cvd = "" if backend == "mujoco_warp" and args.device == "cpu" else "0"
    env = dict(os.environ, PYTHONIOENCODING="utf-8", CUDA_VISIBLE_DEVICES=cvd,
               PYTHONPATH=str(ROOT / "tools" / "train"))
    t0 = time.perf_counter()
    peak = None
    with open(os.path.join(outdir, "train.log"), "w", encoding="utf-8") as log:
        if backend == "mujoco_warp":
            with _SmiPeak(0) as smi:
                p = subprocess.run(_train_cmd(args, backend, n, total, outdir),
                                   cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   timeout=900)
            peak = smi.peak
        else:
            p = subprocess.run(_train_cmd(args, backend, n, total, outdir),
                               cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                               timeout=900)
    elapsed = time.perf_counter() - t0
    if p.returncode != 0:
        raise RuntimeError(f"{backend} N={n} 训练退出码 {p.returncode},见 {outdir}/train.log")
    rows = []
    with open(os.path.join(outdir, "curve.csv"), encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    sps_all = [float(r["steps_per_sec"]) for r in rows]
    steady = [float(r["steps_per_sec"]) for r in rows if int(r["iter"]) >= 2]
    return {"backend": backend, "n_envs": n, "total_timesteps": total,
            "iters": len(rows), "elapsed_sec": elapsed,
            "sps_iter1": sps_all[0] if sps_all else None,
            "sps_steady_mean": float(np.mean(steady)) if steady else (sps_all[0] if sps_all else None),
            "sps_all": sps_all, "peak_vram_mib": peak, "outdir": outdir}


def phase_bt(args) -> None:
    res = {}
    for n in args.ns:
        p = os.path.join(args.out, f"results_bt_{n}.json")
        if os.path.exists(p):
            res.update(_load(args.out, f"results_bt_{n}.json"))
        # 大 N 控制迭代预算:GPU 2 迭代(iter2 为稳态),CPU 1 迭代(防超 10 分钟)。
        for backend in args.bt_backends.split(","):
            backend = backend.strip()
            if n <= 4096:
                total_n = max(200_000, 4 * n * args.num_steps)
            elif backend == "mujoco_warp":
                total_n = 2 * n * args.num_steps
            else:
                total_n = n * args.num_steps
            r = _run_train(args, backend, n, total_n)
            res[f"{backend}_{n}"] = r
            print(f"(bt) {backend} N={n} steady {r['sps_steady_mean']:.0f} control-steps/s "
                  f"({r['iters']} iters, {r['elapsed_sec']:.0f}s) VRAM峰值 {r['peak_vram_mib']}MiB")
            _save(args.out, f"results_bt_{n}.json", res)
    if all(f"mujoco_{n}" in res and f"mujoco_warp_{n}" in res for n in args.ns):
        n = 4096 if 4096 in args.ns else args.ns[-1]
        g = res[f"mujoco_warp_{n}"]["sps_steady_mean"]
        c = res[f"mujoco_{n}"]["sps_steady_mean"]
        print(f"(bt) 判据 N={n}:GPU {g:.0f} vs 2×CPU {2 * c:.0f} → "
              f"{'达标' if g >= 2 * c else '未达标'}")


def phase_c(args) -> None:
    res = {}
    env = dict(os.environ, PYTHONIOENCODING="utf-8", CUDA_VISIBLE_DEVICES="0",
               PYTHONPATH=str(ROOT / "tools" / "train"))
    for tag, backend in (("gpu", "mujoco_warp"), ("cpu", "mujoco")):
        outdir = os.path.join(args.out, f"train_{backend}_1024")
        ckpt = os.path.join(outdir, "ckpt_final.pt")
        if not os.path.exists(ckpt):
            _run_train(args, backend, 1024, 200_000)
        exp_out = os.path.join(args.out, f"export_{tag}")
        p = subprocess.run(
            [sys.executable, "-m", "tools.train.quad.export", "--ckpt", ckpt,
             "--manifest", args.manifest, "--name", f"{tag}_smoke",
             "--out", exp_out, "--body", args.body],
            cwd=ROOT, env=env, capture_output=True, text=True)
        if p.returncode != 0:
            raise RuntimeError(f"export {tag} 失败:{p.stderr[-800:]}")
        p = subprocess.run(
            [sys.executable, "-m", "tools.train.quad.eval", "--ckpt", ckpt,
             "--backend", "rapier", "--body", args.body, "--manifest", args.manifest,
             "--intent", "1", "0", "0", "--episodes", "8", "--seconds", "10",
             "--seed", "0", "--threads", "8"],
            cwd=ROOT, env=env, capture_output=True, text=True)
        if p.returncode != 0:
            raise RuntimeError(f"eval {tag} 失败:{p.stderr[-800:]}")
        ev = _load(outdir, "eval_1_0_0.json")
        res[tag] = {"ckpt": ckpt, "export_out": exp_out, "eval": ev}
        print(f"(c) {tag}: 摔倒 {ev['fall_frac']:.3f} 存活 {ev['survival_time_mean']:.3f}s "
              f"|vx_err| {ev['abs_vx_err_mean']:.4f} 高度 {ev['height_mean']:.4f}")
    _save(args.out, "results_c.json", res)


def phase_report(args) -> None:
    lines = ["# E02f D5 双后端对照(CPU mujoco threads=%d vs GPU mujoco-warp)" % args.threads, ""]
    lines.append(f"seed={args.seed} episode_controls={args.episode_controls} "
                 f"intent_range={args.intent_range} body={args.body}")
    lines.append("")
    if os.path.exists(os.path.join(args.out, "results_a.json")):
        a = _load(args.out, "results_a.json")
        lines.append(f"## (a) 行为 parity(n_envs={a['n_envs']},K={a['k']},只报不判)")
        lines.append("")
        lines.append("| 量 | 首步 max abs | 末步 max abs | K 步最大 |")
        lines.append("|---|---|---|---|")
        for key in sorted(a["max_over_k"]):
            lines.append(f"| {key} | {a['first_step'][key]:.6g} | "
                         f"{a['last_step'][key]:.6g} | {a['max_over_k'][key]:.6g} |")
        lines.append("")
    if os.path.exists(os.path.join(args.out, "results_bp.json")):
        bp = _load(args.out, "results_bp.json")
        lines.append("## (b-1) 纯物理吞吐(sim-steps/s,3 次中位)+ 峰值显存")
        lines.append("")
        lines.append("| N | GPU | CPU(%dt) | GPU/CPU | bench控制步 | torch峰值分配MiB | smi峰值MiB |" % args.threads)
        lines.append("|---|---|---|---|---|---|---|")
        for n, r in bp.items():
            smi = r.get("smi_peak_mib")
            ta = r.get("torch_peak_alloc_mib")
            ta_s = f"{ta:.0f}" if ta is not None else "-"
            smi_s = f"{smi:.0f}" if smi is not None else "-"
            lines.append(f"| {n} | {r['gpu_sim_steps_per_sec']:.3e} | "
                         f"{r['cpu_sim_steps_per_sec']:.3e} | {r['ratio']:.2f} | "
                         f"{r.get('bench_controls', '-')} | {ta_s} | {smi_s} |")
        lines.append("")
        gn = max(bp, key=lambda k: bp[k]["gpu_sim_steps_per_sec"])
        lines.append(f"**纯物理饱和点**:N={gn} 峰值 {bp[gn]['gpu_sim_steps_per_sec']:.3e} sim-steps/s"
                     f"(只报实测,不设硬阈值)")
        lines.append("")
    bt = {}
    for n in args.ns:
        p = os.path.join(args.out, f"results_bt_{n}.json")
        if os.path.exists(p):
            bt.update(_load(args.out, f"results_bt_{n}.json"))
    if bt:
        lines.append("## (b-2) 端到端训练吞吐(control-steps/s,含 obs/reward/PPO)")
        lines.append("")
        lines.append("| N | 后端 | iters | iter1 | 稳态均值(iter≥2) | 耗时 s | VRAM峰值MiB |")
        lines.append("|---|---|---|---|---|---|---|")
        for key, r in bt.items():
            pv = r.get("peak_vram_mib")
            pv_s = f"{pv:.0f}" if pv is not None else "-"
            lines.append(f"| {r['n_envs']} | {r['backend']} | {r['iters']} | "
                         f"{r['sps_iter1']:.0f} | {r['sps_steady_mean']:.0f} | "
                         f"{r['elapsed_sec']:.0f} | {pv_s} |")
        lines.append("")
        gpu_rows = {r["n_envs"]: r for r in bt.values() if r["backend"] == "mujoco_warp"}
        if gpu_rows:
            gn = max(gpu_rows, key=lambda k: gpu_rows[k]["sps_steady_mean"])
            lines.append(f"**端到端饱和点(GPU)**:N={gn} 稳态 {gpu_rows[gn]['sps_steady_mean']:.0f} "
                         f"control-steps/s(只报实测,不设硬阈值)")
            lines.append("")
        if "mujoco_4096" in bt and "mujoco_warp_4096" in bt:
            g = bt["mujoco_warp_4096"]["sps_steady_mean"]
            c = bt["mujoco_4096"]["sps_steady_mean"]
            ok = g >= 2 * c
            lines.append(f"**吞吐判据**:GPU@4096 {g:.0f} ≥ 2×CPU-10t@4096 {2 * c:.0f} → "
                         f"{'达标' if ok else '未达标(报实测,不放宽)'}")
            lines.append("")
    if os.path.exists(os.path.join(args.out, "results_c.json")):
        c = _load(args.out, "results_c.json")
        lines.append("## (c) 迁移(rapier eval,intent=(1,0,0),8 回合×10s,不求效果)")
        lines.append("")
        lines.append("| 训练后端 | 摔倒占比 | 存活均值 s | |vx_err| | 高度均值 | up_y |")
        lines.append("|---|---|---|---|---|---|")
        for tag in ("gpu", "cpu"):
            e = c[tag]["eval"]
            lines.append(f"| {tag} | {e['fall_frac']:.3f} | {e['survival_time_mean']:.3f} | "
                         f"{e['abs_vx_err_mean']:.4f} | {e['height_mean']:.4f} | "
                         f"{e['up_y_mean']:.4f} |")
        lines.append("")
    with open(os.path.join(args.out, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"report.md → {os.path.join(args.out, 'report.md')}")


def main() -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument("--body", default="assets/bodies/quad.body.toml")
    cli.add_argument("--manifest", default="tools/train/quad/manifests/quad_train.toml")
    cli.add_argument("--mjcf", default="target/e02c/quad.xml")
    cli.add_argument("--out", default="target/gpu-compare")
    cli.add_argument("--seed", type=int, default=0)
    cli.add_argument("--episode-controls", type=int, default=600)
    cli.add_argument("--intent-range", type=float, nargs=6,
                     default=[-1.0, 1.0, -1.0, 1.0, -1.0, 1.0])
    cli.add_argument("--threads", type=int, default=10)
    cli.add_argument("--device", default="cuda")
    cli.add_argument("--ns", default="1024,4096")
    cli.add_argument("--k", type=int, default=200)
    cli.add_argument("--bench-controls", type=int, default=200)
    cli.add_argument("--bench-sim-steps", type=float, default=10_000_000,
                     help="bp 每 rep 目标 sim-step 总量(大 N 自动减控制步,防 CPU 超 10 分钟)")
    cli.add_argument("--num-steps", type=int, default=48)
    cli.add_argument("--train-minutes", type=float, default=5.0)
    cli.add_argument("--phases", default="a,bp,bt,c,report")
    cli.add_argument("--bt-backends", default="mujoco,mujoco_warp",
                     help="bt 阶段跑哪些后端(逗号分隔,大 N 可分次跑)")
    args = cli.parse_args()
    args.body = _p(args.body)
    args.manifest = _p(args.manifest)
    args.mjcf = _p(args.mjcf)
    args.out = _p(args.out)
    args.ns = [int(x) for x in args.ns.split(",")]
    os.makedirs(args.out, exist_ok=True)
    phases = {"a": phase_a, "bp": phase_bp, "bt": phase_bt, "c": phase_c,
              "report": phase_report}
    for name in args.phases.split(","):
        phases[name.strip()](args)


if __name__ == "__main__":
    main()