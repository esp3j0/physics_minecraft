"""rapier 后端适配器(R2):包装 learn_env.VecEnv(PyO3 扩展)。

learn_env 由 demos/agents/learn 的 cdylib maturin 装进本 venv;
本文件是 tools/train 与 Rust 环境的唯一接触面。60 Hz 对照(hz60=True)
= 每拍设目标、步 1 拍。
"""

from __future__ import annotations

import numpy as np

from train import spec
from train.backends.base import VecBackend  # noqa: F401  (类型引用)


class RapierBackend:
    spec_dict: dict
    n_envs: int

    def __init__(
        self,
        n_envs: int,
        threads: int,
        seed: int,
        hz60: bool = False,
        mass_mult: float = 1.0,
        gain_mult: float = 1.0,
    ):
        import learn_env  # 延迟 import:换后端(transfer)时不需要装 learn_env

        self.n_envs = n_envs
        self.threads = threads
        self.hz60 = hz60
        # 物理变体(L5:连杆质量 ×mass_mult、增益 ×gain_mult;只改物理,
        # BackendSpec 不变)。
        self.mass_mult = mass_mult
        self.gain_mult = gain_mult
        self.spec_dict = spec.to_dict()
        self._env = learn_env.VecEnv(n_envs, threads, seed, hz60, mass_mult, gain_mult)

    def reset(self, seed: int) -> np.ndarray:
        obs = self._env.reset(seed)
        assert obs.dtype == np.float32 and obs.shape == (self.n_envs, spec.OBS_DIM)
        return obs

    def step(
        self, actions: np.ndarray
    ) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray, np.ndarray]:
        a = np.ascontiguousarray(actions, dtype=np.float32)
        obs, rew, terminated, truncated = self._env.step(a)
        # [n_envs, 7] → dict[str, [n_envs, 1]](键集合由 spec 固定)。
        reward_inputs = {
            k: rew[:, i: i + 1].copy() for i, k in enumerate(spec.REWARD_KEYS)
        }
        return obs, reward_inputs, np.asarray(terminated), np.asarray(truncated)

    def probe_env(self, env_id: int, n: int, seed: int) -> list[int]:
        return list(self._env.probe_env(env_id, n, seed))
