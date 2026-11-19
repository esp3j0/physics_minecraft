"""E02h D2:统一入口路由冒烟(vm-ai)——一份 ppo.py 插所有后端/设备。"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest
import torch

_TRAIN = Path(__file__).resolve().parents[2]
_ROOT = _TRAIN.parents[1]


def _env(cvd="0"):
    return dict(os.environ, PYTHONPATH=str(_TRAIN), PYTHONIOENCODING="utf-8",
                CUDA_VISIBLE_DEVICES=cvd)


def _run(tmp_path, backend, device, mjcf, cvd="0"):
    cmd = [
        sys.executable, "-m", "quad.ppo", "--backend", backend, "--device", device,
        "--body", str(_ROOT / "assets" / "bodies" / "quad.body.toml"),
        "--manifest", str(_TRAIN / "quad" / "manifests" / "quad_train.toml"),
        "--n-envs", "64", "--threads", "4", "--torch-threads", "2",
        "--num-steps", "8", "--total-timesteps", "1024", "--num-minibatches", "2",
        "--update-epochs", "2", "--report-every", "1", "--max-minutes", "2",
        "--intent-range", "-1", "1", "-1", "1", "-1", "1",
        "--out", str(tmp_path),
    ]
    if mjcf is not None:
        cmd += ["--mjcf", str(mjcf)]
    return subprocess.run(cmd, cwd=_ROOT, env=_env(cvd), capture_output=True,
                          text=True, timeout=240)


def _assert_plain_ckpt(tmp_path):
    ck = torch.load(tmp_path / "ckpt_final.pt", map_location="cpu", weights_only=False)
    assert not any(k.startswith("_") for k in ck["spec"])
    assert not any(isinstance(v, torch.Tensor) for v in ck["spec"].values())


def test_warp_cuda0(tmp_path, mjcf_path):
    p = _run(tmp_path, "mujoco_warp", "cuda:0", mjcf_path)
    assert p.returncode == 0, p.stderr[-2000:]
    _assert_plain_ckpt(tmp_path)


def test_warp_cpu(tmp_path, mjcf_path):
    # warp-cpu × CUDA 可见堆损坏(E02g):CPU 跑一律 CVD=""。
    p = _run(tmp_path, "mujoco_warp", "cpu", mjcf_path, cvd="")
    assert p.returncode == 0, p.stderr[-2000:]
    _assert_plain_ckpt(tmp_path)


def test_mujoco_adapter_cpu(tmp_path, mjcf_path):
    p = _run(tmp_path, "mujoco", "cpu", mjcf_path, cvd="")
    assert p.returncode == 0, p.stderr[-2000:]
    _assert_plain_ckpt(tmp_path)


def test_rapier_adapter_cpu(tmp_path):
    p = _run(tmp_path, "rapier", "cpu", None, cvd="")
    assert p.returncode == 0, p.stderr[-2000:]
    _assert_plain_ckpt(tmp_path)


def test_mujoco_adapter_rejects_cuda(tmp_path, mjcf_path):
    p = _run(tmp_path, "mujoco", "cuda:0", mjcf_path)
    assert p.returncode != 0
    assert "只在 CPU" in (p.stderr + p.stdout)