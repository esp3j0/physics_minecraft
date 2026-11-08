"""后端接口(R2 补充):换后端只换适配器,PPO/奖励/导出不动。

接口约定:
- obs: float32 [n_envs, obs_dim],布局 = spec.OBS_LAYOUT;
- actions: float32 [n_envs, act_dim],取值 [-1, 1](后端钳位);
- step 返回 (obs, reward_inputs, terminated, truncated):
  reward_inputs: dict[str, float32 [n_envs, 1]],键集合 = spec.REWARD_KEYS;
  回合结束的实例自动 reset(obs/reward_inputs 是新回合首步的值,
  truncated[i] = True 标记;训练侧丢弃这些步的奖励)。
- reset(seed) 每环境的种子派生由后端负责,但必须只依赖 seed 与 env_id
  (与线程数无关;契约检查验证)。

下一步接 MuJoCo(transfer):新增 train/backends/mujoco.py,实现同一个
VecBackend,MJCF 由 `cargo xtask export-mjcf` 导出;spec 不变。
"""

from __future__ import annotations

from typing import Protocol

import numpy as np

from train import spec


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

    def probe_env(self, env_id: int, n: int, seed: int) -> list[int]:
        """L2 探针:固定动作序列逐拍哈希(线程无关性验证用)。"""
        ...
