"""后端工厂:换后端只改这里(transfer 阶段加 mujoco 分支)。"""

from __future__ import annotations


def make_backend(
    name: str,
    n_envs: int,
    threads: int,
    seed: int,
    hz60: bool = False,
    mass_mult: float = 1.0,
    gain_mult: float = 1.0,
):
    if name == "rapier":
        from train.backends.rapier import RapierBackend

        return RapierBackend(n_envs, threads, seed, hz60, mass_mult, gain_mult)
    if name == "mujoco":
        # transfer 阶段实现;接口见 train/backends/base.py。
        from train.backends.mujoco import MujocoBackend  # noqa: F401

        raise NotImplementedError("mujoco 后端在 transfer 阶段实现")
    raise ValueError(f"未知后端 {name}(可选:rapier)")
