"""rapier 后端适配器(E02b-1):包装 agent_env.VecEnv(PyO3 扩展)。

agent_env 由 crates/agent-env 的 cdylib 经 maturin develop --release 装进
本 venv;本文件是 quad 训练代码与 Rust 环境的唯一接触面。spec_dict 直接取
env.spec()(Rust 侧从身体定义与 manifest 算,Python 不重复写常量)。
"""

from __future__ import annotations

import numpy as np


class RapierBackend:
    spec_dict: dict
    n_envs: int

    def __init__(
        self,
        body_toml_path: str,
        manifest_toml_path: str,
        n_envs: int,
        threads: int,
        episode_controls: int,
        intent_range: list | None = None,
        settle_ticks: int = 120,
        fall_up_y_min: float = 0.5,
    ):
        import agent_env  # 延迟 import:换后端时不需要装 agent_env

        with open(body_toml_path, encoding="utf-8") as f:
            body_toml = f.read()
        with open(manifest_toml_path, encoding="utf-8") as f:
            manifest_toml = f.read()
        if intent_range is None:
            intent_range = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.n_envs = n_envs
        self.threads = threads
        self._env = agent_env.VecEnv(
            body_toml,
            manifest_toml,
            n_envs,
            threads,
            episode_controls,
            settle_ticks,
            fall_up_y_min,
            intent_range,
        )
        self.spec_dict = self._env.spec()

    def reset(self, seed: int) -> np.ndarray:
        obs = self._env.reset(seed)
        assert obs.dtype == np.float32
        assert obs.shape == (self.n_envs, self.spec_dict["obs_dim"])
        return obs

    def step(
        self, actions: np.ndarray
    ) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray, np.ndarray]:
        a = np.ascontiguousarray(actions, dtype=np.float32)
        obs, reward_inputs, terminated, truncated = self._env.step(a)
        return obs, reward_inputs, np.asarray(terminated), np.asarray(truncated)