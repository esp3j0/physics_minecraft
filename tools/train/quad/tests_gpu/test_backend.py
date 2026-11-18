"""E02f D2 后端行为测试:初态/自动重置/截断/仿真时长(杀 M5/M6/M7/M9/M11)。"""

from __future__ import annotations

import numpy as np
import pytest
import torch

import agent_env
from quad.backends.mujoco_warp import MujocoWarpBackend

IR = [[-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0]]


@pytest.fixture(scope="module")
def be(body_path, manifest_path, mjcf_path):
    b = MujocoWarpBackend(
        body_path, manifest_path, n_envs=8, episode_controls=600,
        intent_range=IR, mjcf_path=mjcf_path, device="cuda:0",
    )
    b.reset(11)
    return b


def test_reset_obs(be):
    obs = be.reset(11)
    assert obs.shape == (8, be.obs_dim)
    assert obs.dtype == torch.float32
    assert torch.isfinite(obs).all()


def test_initial_obs_cpu_parity(body_path, manifest_path, mjcf_path):
    # 初态两侧同为 host settle,obs 应逐位接近(杀关节地址/quat 序等提取变异)。
    from quad.backends.mujoco import MujocoBackend

    cpu = MujocoBackend(
        body_path, manifest_path, n_envs=8, threads=2, episode_controls=600,
        intent_range=IR, mjcf_path=mjcf_path,
    )
    gpu = MujocoWarpBackend(
        body_path, manifest_path, n_envs=8, episode_controls=600,
        intent_range=IR, mjcf_path=mjcf_path, device="cuda:0",
    )
    oc = cpu.reset(11)
    og = gpu.reset(11).cpu().numpy()
    assert np.abs(oc - og).max() <= 1e-4


def test_sim_time_advance(be):
    be.reset(11)
    t0 = be.sim_time(0)
    be.step(torch.zeros(8, be.act_dim, device=be.device))
    dt = be.sim_time(0) - t0
    want = be.control_period_ticks / be.tick_hz
    assert abs(dt - want) < 1e-6, (dt, want)


def test_nonfinite_action_rejected(be):
    a = torch.zeros(8, be.act_dim, device=be.device)
    a[0, 0] = float("nan")
    with pytest.raises(ValueError):
        be.step(a)


def test_autoreset_intent_prev_state(body_path, manifest_path, mjcf_path):
    # fall_up_y_min=2.0:第一步全员 terminated,触发自动重置。
    b = MujocoWarpBackend(
        body_path, manifest_path, n_envs=4, episode_controls=600,
        intent_range=IR, fall_up_y_min=2.0, mjcf_path=mjcf_path, device="cuda:0",
    )
    b.reset(7)
    a = 0.5 * torch.ones(4, b.act_dim, device=b.device)
    obs1, ri1, term, trunc = b.step(a)
    assert bool(term.all()) and not bool(trunc.any())
    # M9:重置后状态回到初态(qpos 逐位;非零动作使末态偏离初态)。
    import warp as wp

    qp = wp.to_torch(b._d.qpos).cpu().numpy()
    assert np.abs(qp - b._initial["qpos"]).max() < 1e-6
    h = b._extract()["root_height"][:, 0].cpu().numpy()
    assert np.abs(h - b.initial_root_height).max() < 1e-4
    # M7:done 行 prev_action 清零 → 下一步 ri.prev_action 全 0(动作非零)。
    obs2, ri2, _, _ = b.step(a)
    assert float(ri2["prev_action"].abs().max()) == 0.0
    # M6:done 行重采 intent = sample_intent(episode_seed(base,i,1))。
    want = np.array(
        [agent_env.sample_intent(agent_env.episode_seed(7, i, 1), IR) for i in range(4)],
        dtype=np.float32,
    )
    got = ri2["intent"].cpu().numpy()
    assert np.abs(got - want).max() < 1e-6
    # 重置帧 obs 的 last_action 段为 0(done 行;raw=0.5 非零)。
    d = b.dof
    la = obs1[:, 3 + 3 + 3 + d + d : 3 + 3 + 3 + d + d + d]
    assert float(la.abs().max()) == 0.0


def test_truncation(body_path, manifest_path, mjcf_path):
    b = MujocoWarpBackend(
        body_path, manifest_path, n_envs=4, episode_controls=2,
        intent_range=IR, mjcf_path=mjcf_path, device="cuda:0",
    )
    b.reset(3)
    a = 0.8 * torch.ones(4, b.act_dim, device=b.device)
    _, _, term1, trunc1 = b.step(a)
    assert not bool(term1.any()) and not bool(trunc1.any())
    _, _, term2, trunc2 = b.step(a)
    assert bool(trunc2.all()) and not bool(term2.any())
    import warp as wp

    qp = wp.to_torch(b._d.qpos).cpu().numpy()
    assert np.abs(qp - b._initial["qpos"]).max() < 1e-6
    _, ri3, _, _ = b.step(a)
    assert float(ri3["prev_action"].abs().max()) == 0.0