"""E02f D3 parity:GPU 编码/解码/奖励 vs agent_env(Rust CPU)与 task.reward(numpy)。

随机 ≥256 组;obs/bf max abs ≤1e-4,decode targets ≤1e-5(raw/clamped 逐位),
reward ≤1e-5。手算案例 C1–C3 为架构师钉死期望,不许用被测函数反推。
"""

from __future__ import annotations

import numpy as np
import pytest
import torch

import agent_env
from quad import task
from quad.backends import mujoco_warp as mwbe
from quad.reward import reward_gpu

N = 256
SEED = 20261118


@pytest.fixture(scope="module")
def spec(body_path, manifest_path):
    body = open(body_path, encoding="utf-8").read()
    man = open(manifest_path, encoding="utf-8").read()
    return agent_env.spec_for(body, man, 600, 0.5), body, man


@pytest.fixture(scope="module")
def codec(spec):
    _, body, man = spec
    return agent_env.Codec(body, man)


@pytest.fixture(scope="module")
def sp(spec, device):
    return mwbe.spec_tensors(spec[0], device)


def _rand_state(device, n=N):
    g = torch.Generator().manual_seed(SEED)
    q = torch.randn(n, 4, generator=g)
    q = q / q.norm(dim=-1, keepdim=True)
    d = {
        "quat": q,
        "linvel": torch.randn(n, 3, generator=g) * 2.0,
        "angvel": torch.randn(n, 3, generator=g) * 3.0,
        "joint_pos": torch.randn(n, 12, generator=g) * 0.5,
        "joint_vel": torch.randn(n, 12, generator=g) * 3.0,
        "last_action": torch.randn(n, 12, generator=g) * 1.5,
        "intent": torch.rand(n, 3, generator=g) * 2.0 - 1.0,
    }
    return {k: v.to(device=device, dtype=torch.float32) for k, v in d.items()}


def _max_abs(a: torch.Tensor, b: np.ndarray) -> float:
    return float((a.detach().cpu().numpy() - b).__abs__().max())


def test_hand_c1_identity(sp, device):
    q = torch.tensor([[0.0, 0.0, 0.0, 1.0]], device=device)
    lv = torch.tensor([[1.0, 0.0, 0.0]], device=device)
    av = torch.zeros(1, 3, device=device)
    bf = mwbe.body_frame_gpu(q, lv, av)
    assert _max_abs(bf["base_linvel_body"], np.array([[1.0, 0.0, 0.0]])) < 1e-6
    assert _max_abs(bf["projected_gravity"], np.array([[0.0, -1.0, 0.0]])) < 1e-6
    assert _max_abs(bf["up_y"], np.array([[1.0]])) < 1e-6


def test_hand_c2_yaw90(sp, device):
    s = float(np.sqrt(0.5))
    q = torch.tensor([[0.0, s, 0.0, s]], device=device)
    lv = torch.tensor([[1.0, 0.0, 0.0]], device=device)
    av = torch.zeros(1, 3, device=device)
    bf = mwbe.body_frame_gpu(q, lv, av)
    assert _max_abs(bf["base_linvel_body"], np.array([[0.0, 0.0, 1.0]])) < 1e-6
    assert _max_abs(bf["projected_gravity"], np.array([[0.0, -1.0, 0.0]])) < 1e-6
    assert _max_abs(bf["up_y"], np.array([[1.0]])) < 1e-6


def test_hand_c3_angvel(sp, device):
    q = torch.tensor([[0.0, 0.0, 0.0, 1.0]], device=device)
    lv = torch.zeros(1, 3, device=device)
    av = torch.tensor([[0.0, 1.0, 0.0]], device=device)
    bf = mwbe.body_frame_gpu(q, lv, av)
    assert _max_abs(bf["base_angvel_body"], np.array([[0.0, 1.0, 0.0]])) < 1e-6
    assert _max_abs(bf["up_y"], np.array([[1.0]])) < 1e-6


def test_encode_body_frame_parity(sp, codec, device):
    st = _rand_state(device)
    obs, bf = mwbe.encode_gpu(sp, st, st["last_action"], st["intent"])
    np_q = st["quat"].cpu().numpy()
    np_lv = st["linvel"].cpu().numpy()
    np_av = st["angvel"].cpu().numpy()
    np_jp = st["joint_pos"].cpu().numpy()
    np_jv = st["joint_vel"].cpu().numpy()
    np_la = st["last_action"].cpu().numpy()
    np_it = st["intent"].cpu().numpy()
    ref_obs = codec.encode_batch(np_q, np_lv, np_av, np_jp, np_jv, np_la, np_it)
    assert _max_abs(obs, ref_obs) <= 1e-4
    ref_bf = codec.body_frame_batch(np_q, np_lv, np_av)
    for k in ("base_linvel_body", "base_angvel_body", "projected_gravity", "up_y"):
        assert _max_abs(bf[k], ref_bf[k]) <= 1e-4, k


def test_decode_parity(sp, codec, device):
    g = torch.Generator().manual_seed(SEED + 1)
    acts = (torch.randn(N, 12, generator=g) * 2.0).to(device=device, dtype=torch.float32)
    raw, targets, clamped = mwbe.decode_gpu(sp, acts)
    ref_raw, ref_targets, ref_clamped = codec.decode_batch(acts.cpu().numpy())
    assert np.array_equal(raw.cpu().numpy(), ref_raw)
    assert _max_abs(targets, ref_targets) <= 1e-5
    assert np.array_equal(clamped.cpu().numpy(), ref_clamped)


def test_decode_clamp_hand(sp, device):
    # 手算:限位收紧到 default±0.1,action=1 → raw=1,t=default+0.25 越上限
    # → targets=default+0.1,clamped=12(quad_train 原限位下钳位是 no-op,须此例)。
    default = np.asarray(sp["default_pose"], dtype=np.float32)
    narrow = dict(sp)
    narrow["joint_lower"] = (default - 0.1).tolist()
    narrow["joint_upper"] = (default + 0.1).tolist()
    sp2 = mwbe.spec_tensors(narrow, device)
    acts = torch.ones(2, 12, device=device)
    raw, targets, clamped = mwbe.decode_gpu(sp2, acts)
    assert np.array_equal(raw.cpu().numpy(), np.ones((2, 12), dtype=np.float32))
    assert _max_abs(targets, np.broadcast_to(default + 0.1, (2, 12))) <= 1e-6
    assert np.array_equal(clamped.cpu().numpy(), np.full(2, 12, dtype=np.uint32))


def test_reward_parity(device):
    g = torch.Generator().manual_seed(SEED + 2)
    ri_t = {
        "base_linvel_body": torch.randn(N, 3, generator=g) * 0.6,
        "base_angvel_body": torch.randn(N, 3, generator=g),
        "projected_gravity": torch.randn(N, 3, generator=g) * 0.3,
        "root_height": 0.2775 + torch.randn(N, 1, generator=g) * 0.05,
        "action": torch.randn(N, 12, generator=g),
        "prev_action": torch.randn(N, 12, generator=g),
        "intent": torch.rand(N, 3, generator=g) * 2.0 - 1.0,
    }
    ri_t = {k: v.to(device=device, dtype=torch.float32) for k, v in ri_t.items()}
    r_gpu = reward_gpu(ri_t)
    ri_np = {k: v.cpu().numpy() for k, v in ri_t.items()}
    r_ref = task.reward(ri_np)
    assert _max_abs(r_gpu, r_ref) <= 1e-5