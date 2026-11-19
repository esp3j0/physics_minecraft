"""E02g D1:统一设备路径 parity——device=cpu 与 cuda:0 同 tol 对 agent_env/task.reward;
初态 obs 对 legacy CPU-numpy 后端逐位≤1e-4。"""

from __future__ import annotations

import numpy as np
import pytest
import torch

import agent_env
from quad import task
from quad.backends import mujoco_warp as mwbe
from quad.backends.mujoco import MujocoBackend
from quad.backends.mujoco_warp import MujocoWarpBackend

from test_parity import N, SEED, _max_abs, _rand_state

IR = [[-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0]]
DEVICES = ["cpu", "cuda:0"]


@pytest.fixture(scope="module", params=DEVICES)
def dev(request):
    if request.param.startswith("cuda") and not torch.cuda.is_available():
        pytest.skip("无 CUDA GPU")
    return request.param


@pytest.fixture(scope="module")
def spec(body_path, manifest_path):
    body = open(body_path, encoding="utf-8").read()
    man = open(manifest_path, encoding="utf-8").read()
    return agent_env.spec_for(body, man, 600, 0.5), body, man


def test_encode_decode_reward_parity(dev, spec, body_path, manifest_path):
    sp = mwbe.spec_tensors(spec[0], torch.device(dev))
    codec = agent_env.Codec(spec[1], spec[2])
    st = _rand_state(torch.device(dev))
    obs, bf = mwbe.encode_gpu(sp, st, st["last_action"], st["intent"])
    npv = {k: v.cpu().numpy() for k, v in st.items()}
    ref_obs = codec.encode_batch(npv["quat"], npv["linvel"], npv["angvel"],
                                 npv["joint_pos"], npv["joint_vel"],
                                 npv["last_action"], npv["intent"])
    assert _max_abs(obs, ref_obs) <= 1e-4
    ref_bf = codec.body_frame_batch(npv["quat"], npv["linvel"], npv["angvel"])
    for k in ("base_linvel_body", "base_angvel_body", "projected_gravity", "up_y"):
        assert _max_abs(bf[k], ref_bf[k]) <= 1e-4, k
    g = torch.Generator().manual_seed(SEED + 1)
    acts = (torch.randn(N, 12, generator=g) * 2.0).to(device=dev, dtype=torch.float32)
    raw, targets, clamped = mwbe.decode_gpu(sp, acts)
    rr, rt, rc = codec.decode_batch(acts.cpu().numpy())
    assert np.array_equal(raw.cpu().numpy(), rr)
    assert _max_abs(targets, rt) <= 1e-5
    assert np.array_equal(clamped.cpu().numpy(), rc)
    g3 = torch.Generator().manual_seed(SEED + 2)
    ri = {
        "base_linvel_body": torch.randn(N, 3, generator=g3) * 0.6,
        "base_angvel_body": torch.randn(N, 3, generator=g3),
        "projected_gravity": torch.randn(N, 3, generator=g3) * 0.3,
        "root_height": 0.2775 + torch.randn(N, 1, generator=g3) * 0.05,
        "action": torch.randn(N, 12, generator=g3),
        "prev_action": torch.randn(N, 12, generator=g3),
        "intent": torch.rand(N, 3, generator=g3) * 2.0 - 1.0,
    }
    ri = {k: v.to(device=dev, dtype=torch.float32) for k, v in ri.items()}
    r_gpu = mwbe.reward_gpu(ri)
    r_ref = task.reward({k: v.cpu().numpy() for k, v in ri.items()})
    assert _max_abs(r_gpu, r_ref) <= 1e-5


def test_initial_obs_vs_legacy(dev, body_path, manifest_path, mjcf_path):
    legacy = MujocoBackend(
        body_path, manifest_path, n_envs=8, threads=2, episode_controls=600,
        intent_range=IR, mjcf_path=mjcf_path,
    )
    be = MujocoWarpBackend(
        body_path, manifest_path, n_envs=8, episode_controls=600,
        intent_range=IR, mjcf_path=mjcf_path, device=dev,
    )
    oc = legacy.reset(11)
    og = be.reset(11).cpu().numpy()
    assert np.abs(oc - og).max() <= 1e-4
    # 设备放置自证:状态张量确在指定设备。
    import warp as wp

    assert wp.to_torch(be._d.qpos).device.type == torch.device(dev).type