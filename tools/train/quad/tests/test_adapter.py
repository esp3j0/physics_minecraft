"""E02h D1:CPU 适配器/工厂测试(无 GPU)。

零拷贝(torch.from_numpy 共享内存)、dtype/shape/设备、reward_inputs 键完整、
reward_gpu 对 task.reward parity ≤1e-5、工厂路由与拒绝口径。
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
import torch

from quad.backends.tensor import NumpyToTensorAdapter, make_tensor_backend
from quad.reward import reward_gpu
from quad import task

IR = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0]


class _Stub:
    """numpy VecBackend 桩:返回固定缓冲,验证零拷贝与 actions 回转。"""

    def __init__(self, n=4, obs_dim=6, act_dim=3):
        self.n_envs = n
        self.spec_dict = {"obs_dim": obs_dim, "act_dim": act_dim}
        self._obs = np.arange(n * obs_dim, dtype=np.float32).reshape(n, obs_dim)
        self._ri = {
            "intent": np.ones((n, 3), dtype=np.float32),
            "root_height": np.full((n, 1), 0.3, dtype=np.float32),
        }
        self._term = np.zeros(n, dtype=bool)
        self._trunc = np.zeros(n, dtype=bool)
        self.last_actions = None

    def reset(self, seed):
        return self._obs

    def step(self, actions):
        self.last_actions = actions
        return self._obs, self._ri, self._term, self._trunc


def test_zero_copy_dtype_device():
    stub = _Stub()
    ad = NumpyToTensorAdapter(stub)
    assert ad.device == torch.device("cpu")
    assert ad.n_envs == 4 and ad.spec_dict is stub.spec_dict
    obs = ad.reset(0)
    assert isinstance(obs, torch.Tensor) and obs.dtype == torch.float32
    assert obs.shape == (4, 6) and obs.device.type == "cpu"
    assert np.shares_memory(obs.numpy(), stub._obs)
    a = torch.arange(12, dtype=torch.float32).reshape(4, 3) * 0.1
    o, ri, term, trunc = ad.step(a)
    assert o.dtype == torch.float32 and np.shares_memory(o.numpy(), stub._obs)
    for k, v in ri.items():
        assert isinstance(v, torch.Tensor) and v.dtype == torch.float32, k
        assert np.shares_memory(v.numpy(), stub._ri[k]), k
    assert term.dtype == torch.bool and trunc.dtype == torch.bool
    assert isinstance(stub.last_actions, np.ndarray)
    assert stub.last_actions.dtype == np.float32
    assert np.array_equal(stub.last_actions, a.numpy())


def _args(backend, body, manifest, mjcf=None, device="cpu"):
    return SimpleNamespace(
        backend=backend, device=device, body=body, manifest=manifest, mjcf=mjcf,
        n_envs=4, threads=2, episode_controls=600, intent_range=IR,
    )


def test_reward_keys_and_parity_rapier(body_path, manifest_path):
    from quad.backends.rapier import RapierBackend

    be = RapierBackend(body_path, manifest_path, n_envs=4, threads=2,
                       episode_controls=600, intent_range=[[-1, 1], [-1, 1], [-1, 1]])
    ad = NumpyToTensorAdapter(be)
    ad.reset(3)
    a = torch.randn(4, be.spec_dict["act_dim"], dtype=torch.float32) * 0.5
    _, ri, term, trunc = ad.step(a)
    keys = {name for name, _ in be.spec_dict["reward_keys"]}
    assert set(ri.keys()) == keys
    assert all(v.dtype == torch.float32 for v in ri.values())
    assert term.dtype == torch.bool and trunc.dtype == torch.bool
    r_t = reward_gpu(ri)
    r_ref = task.reward({k: v.numpy() for k, v in ri.items()})
    assert float(np.abs(r_t.numpy() - r_ref).max()) <= 1e-5


def test_reward_keys_and_parity_mujoco(body_path, manifest_path, mjcf_path):
    from quad.backends.mujoco import MujocoBackend

    be = MujocoBackend(body_path, manifest_path, n_envs=4, threads=2,
                       episode_controls=600, intent_range=[[-1, 1], [-1, 1], [-1, 1]],
                       mjcf_path=mjcf_path)
    ad = NumpyToTensorAdapter(be)
    ad.reset(3)
    a = torch.randn(4, be.spec_dict["act_dim"], dtype=torch.float32) * 0.5
    _, ri, _, _ = ad.step(a)
    keys = {name for name, _ in be.spec_dict["reward_keys"]}
    assert set(ri.keys()) == keys
    r_t = reward_gpu(ri)
    r_ref = task.reward({k: v.numpy() for k, v in ri.items()})
    assert float(np.abs(r_t.numpy() - r_ref).max()) <= 1e-5


def test_factory_rapier_mujoco(body_path, manifest_path, mjcf_path):
    ad = make_tensor_backend(_args("rapier", body_path, manifest_path))
    assert isinstance(ad, NumpyToTensorAdapter) and ad.device.type == "cpu"
    ad2 = make_tensor_backend(_args("mujoco", body_path, manifest_path, mjcf_path))
    assert isinstance(ad2, NumpyToTensorAdapter)
    from quad.backends.mujoco import MujocoBackend

    assert isinstance(ad2._be, MujocoBackend)


def test_factory_rejects(body_path, manifest_path, mjcf_path):
    with pytest.raises(SystemExit):
        make_tensor_backend(_args("nope", body_path, manifest_path))
    with pytest.raises(SystemExit):
        make_tensor_backend(_args("rapier", body_path, manifest_path, device="cuda:0"))
    with pytest.raises(SystemExit):
        make_tensor_backend(_args("mujoco", body_path, manifest_path, mjcf_path, device="cuda:1"))
    with pytest.raises(SystemExit):
        make_tensor_backend(_args("mujoco", body_path, manifest_path, mjcf=None))


def test_factory_warp_device(body_path, manifest_path, mjcf_path):
    pytest.importorskip("mujoco_warp")
    from quad.backends.mujoco_warp import MujocoWarpBackend

    be = make_tensor_backend(_args("mujoco_warp", body_path, manifest_path, mjcf_path, device="cpu"))
    assert isinstance(be, MujocoWarpBackend) and be.device == torch.device("cpu")