"""E02g D3:DDP 一致性 + 单进程双卡防御测试。

一致性:torchrun 2 rank 小冒烟后,两 rank 权重 sha256 逐位相同、obs-norm
统计逐位相同、分片均分且和等于总 N、每进程只见 1 张卡。
防御:单进程交替步进 cuda:0/cuda:1 应崩(探针 CUDA error 700 坑),
验证每 rank 隔离单卡的必要性。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    not Path("/dev/nvidia1").exists(), reason="需要双卡(仅 vm-ai 跑)"
)

_TRAIN = Path(__file__).resolve().parents[2]


def _env() -> dict:
    env = dict(os.environ)
    env["PYTHONPATH"] = str(_TRAIN)
    env["CUDA_VISIBLE_DEVICES"] = "0,1"
    env["PYTHONIOENCODING"] = "utf-8"
    return env


def test_ddp_weight_rms_consistency(repo_root, body_path, manifest_path, mjcf_path, tmp_path):
    out = tmp_path / "ddp"
    total_n = 256
    cmd = [
        sys.executable, "-m", "torch.distributed.run",
        "--nproc_per_node=2", "--master_port=29517",
        "-m", "quad.ppo_ddp",
        "--body", str(body_path), "--manifest", str(manifest_path),
        "--mjcf", str(mjcf_path),
        "--n-envs", str(total_n), "--total-timesteps", "24576",
        "--max-minutes", "3", "--num-steps", "48",
        "--out", str(out), "--seed", "0",
    ]
    proc = subprocess.run(
        cmd, cwd=repo_root, env=_env(), capture_output=True, text=True, timeout=480
    )
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
    stats = []
    for r in (0, 1):
        with open(out / f"rank_stats_{r}.json", encoding="utf-8") as f:
            stats.append(json.load(f))
    assert stats[0]["param_sha256"] == stats[1]["param_sha256"], "两 rank 权重不一致(allreduce 未生效?)"
    assert stats[0]["obs_mean"] == stats[1]["obs_mean"], "obs-norm mean 跨 rank 不一致"
    assert stats[0]["obs_var"] == stats[1]["obs_var"], "obs-norm var 跨 rank 不一致"
    assert stats[0]["obs_count"] == stats[1]["obs_count"]
    assert stats[0]["n_envs"] == stats[1]["n_envs"] == total_n // 2
    assert stats[0]["n_envs"] + stats[1]["n_envs"] == total_n
    for s in stats:
        assert s["cuda_device_count"] == 1, "rank 进程未隔离单卡"
    assert (out / "ckpt_final.pt").exists()
    assert (out / "curve.csv").exists()


_CRASH_CODE = """
import torch
from quad.backends.mujoco_warp import MujocoWarpBackend
import sys
kw = dict(body_toml_path=sys.argv[1], manifest_toml_path=sys.argv[2],
          n_envs=8, episode_controls=600, mjcf_path=sys.argv[3])
be0 = MujocoWarpBackend(device="cuda:0", **kw)
be1 = MujocoWarpBackend(device="cuda:1", **kw)
o0 = be0.reset(0)
o1 = be1.reset(0)
a0 = torch.zeros(8, be0.act_dim, device="cuda:0")
a1 = torch.zeros(8, be1.act_dim, device="cuda:1")
for i in range(30):
    be0.step(a0)
    be1.step(a1)
    if i % 5 == 0:
        print("alt step", i, flush=True)
print("NO_CRASH", flush=True)
"""


def test_single_process_dual_device_crashes(repo_root, body_path, manifest_path, mjcf_path):
    proc = subprocess.run(
        [sys.executable, "-u", "-c", _CRASH_CODE,
         str(body_path), str(manifest_path), str(mjcf_path)],
        cwd=repo_root, env=_env(), capture_output=True, text=True, timeout=300,
    )
    assert proc.returncode != 0, (
        "单进程交替步进双卡未崩溃(探针坑未复现):\n" + proc.stdout[-1000:]
    )