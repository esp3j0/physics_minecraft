"""四足后端接口(E02b-1;同 train/backends/base.py,去掉 probe_env)。

换后端只换适配器:obs float32 [n_envs, obs_dim];actions float32
[n_envs, act_dim] 取值 [-1,1](后端钳位);step 返回
(obs, reward_inputs, terminated, truncated),reward_inputs 为
dict[str, float32 [n_envs, k]](键集合 = spec["reward_keys"]);回合结束的
实例自动 reset(obs 是新回合首帧,truncated[i] 标记到时)。reset 种子派生
只依赖 seed 与 env_id(与线程数无关;契约检查验证)。
"""

from __future__ import annotations

from typing import Protocol

import numpy as np


class VecBackend(Protocol):
    """统一后端接口(结构化类型;不要求继承)。"""

    spec_dict: dict
    n_envs: int

    def reset(self, seed: int) -> np.ndarray:
        """float32 [n_envs, obs_dim]。"""
        ...

    def step(
        self, actions: np.ndarray
    ) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray, np.ndarray]:
        """→ (obs, reward_inputs, terminated, truncated)。"""
        ...