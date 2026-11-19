"""E02h D2:统一张量循环 CPU 测试(无 GPU)。

退役自证:ppo.main 对三后端全路由 train_tensor;mujoco 物理仍走 rollout;
小训检查点结构不变(spec 无 _ 张量、rms numpy float64);--init-from 接续。
"""

from __future__ import annotations

import sys

import numpy as np
import pytest
import torch


def _argv(body, manifest, backend, out, mjcf=None, extra=()):
    a = [
        "quad.ppo", "--backend", backend, "--body", body, "--manifest", manifest,
        "--n-envs", "8", "--threads", "2", "--torch-threads", "2",
        "--num-steps", "8", "--total-timesteps", "128", "--num-minibatches", "2",
        "--update-epochs", "2", "--report-every", "1", "--max-minutes", "2",
        "--intent-range", "-1", "1", "-1", "1", "-1", "1", "--seed", "0",
        "--out", str(out),
    ]
    if mjcf is not None:
        a += ["--mjcf", mjcf]
    return a + list(extra)


@pytest.mark.parametrize("backend", ["rapier", "mujoco", "mujoco_warp"])
def test_main_routes_all_backends_to_tensor_loop(
    monkeypatch, body_path, manifest_path, mjcf_path, backend
):
    import quad.ppo_gpu as pg

    calls = []
    monkeypatch.setattr(pg, "train_tensor", lambda args: calls.append(args.backend))
    import quad.ppo as ppo

    monkeypatch.setattr(sys, "argv", _argv(body_path, manifest_path, backend, "unused", mjcf_path))
    ppo.main()
    assert calls == [backend]


def test_mujoco_adapter_uses_rollout(monkeypatch, body_path, manifest_path, mjcf_path):
    from quad.backends.mujoco import MujocoBackend
    from quad.backends.tensor import NumpyToTensorAdapter

    be = MujocoBackend(body_path, manifest_path, n_envs=4, threads=2,
                       episode_controls=600, intent_range=[[-1, 1], [-1, 1], [-1, 1]],
                       mjcf_path=mjcf_path)
    calls = []
    orig = be._rollout.rollout

    def spy(*a, **k):
        calls.append(1)
        return orig(*a, **k)

    monkeypatch.setattr(be._rollout, "rollout", spy)
    ad = NumpyToTensorAdapter(be)
    ad.reset(0)
    obs, _, _, _ = ad.step(torch.zeros(4, be.spec_dict["act_dim"], dtype=torch.float32))
    assert len(calls) == 1
    assert torch.isfinite(obs).all()


def _check_ckpt(path, total_steps):
    ck = torch.load(path, map_location="cpu", weights_only=False)
    assert ck["total_steps"] == total_steps
    spec = ck["spec"]
    assert not any(k.startswith("_") for k in spec)
    assert not any(isinstance(v, torch.Tensor) for v in spec.values())
    assert isinstance(ck["obs_mean"], np.ndarray) and ck["obs_mean"].dtype == np.float64
    assert isinstance(ck["obs_var"], np.ndarray) and ck["obs_var"].dtype == np.float64
    return ck


def test_unified_train_rapier_cpu(monkeypatch, tmp_path, body_path, manifest_path):
    import quad.ppo as ppo

    monkeypatch.setattr(sys, "argv", _argv(body_path, manifest_path, "rapier", tmp_path))
    ppo.main()
    _check_ckpt(tmp_path / "ckpt_final.pt", 128)
    lines = (tmp_path / "curve.csv").read_text(encoding="utf-8").strip().splitlines()
    assert lines[0] == "iter,total_steps,ep_rew_mean,ep_len_mean,fall_frac,steps_per_sec"
    assert len(lines) == 3


def test_unified_train_mujoco_cpu(monkeypatch, tmp_path, body_path, manifest_path, mjcf_path):
    import quad.ppo as ppo

    monkeypatch.setattr(sys, "argv", _argv(body_path, manifest_path, "mujoco", tmp_path, mjcf_path))
    ppo.main()
    ck = _check_ckpt(tmp_path / "ckpt_final.pt", 128)
    assert ck["args"]["backend"] == "mujoco"


def test_init_from_continues(monkeypatch, tmp_path, body_path, manifest_path):
    import quad.ppo as ppo

    monkeypatch.setattr(sys, "argv", _argv(body_path, manifest_path, "rapier", tmp_path))
    ppo.main()
    out2 = tmp_path / "cont"
    monkeypatch.setattr(sys, "argv", _argv(
        body_path, manifest_path, "rapier", out2,
        extra=("--init-from", str(tmp_path / "ckpt_final.pt"),
               "--total-timesteps", "128", "--freeze-obs-norm"),
    ))
    ppo.main()
    _check_ckpt(out2 / "ckpt_final.pt", 256)