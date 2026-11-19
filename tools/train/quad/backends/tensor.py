"""统一张量后端接口(E02h D1)。

张量后端约定:`reset(seed)->torch.float32[n,obs_dim]`(在 device);
`step(torch actions)->(obs torch, reward_inputs dict[str,torch],
terminated torch.bool[n], truncated torch.bool[n])`;`spec_dict`/`n_envs`/`device`。
MujocoWarpBackend 原生符合;rapier/mujoco 的 numpy VecBackend 经
NumpyToTensorAdapter 零拷贝接入(torch.from_numpy 共享内存,只在 CPU)。
reward_inputs 转 torch 后由训练循环统一 reward_gpu 计算。
"""

from __future__ import annotations

import torch


class NumpyToTensorAdapter:
    """包 numpy VecBackend:obs/reward_inputs/flags 零拷贝转 torch,actions 转回 numpy。"""

    def __init__(self, backend):
        self._be = backend
        self.spec_dict = backend.spec_dict
        self.n_envs = backend.n_envs
        self.device = torch.device("cpu")

    def reset(self, seed: int) -> torch.Tensor:
        return torch.from_numpy(self._be.reset(seed))

    def step(self, actions: torch.Tensor):
        obs, ri, terminated, truncated = self._be.step(actions.detach().cpu().numpy())
        return (
            torch.from_numpy(obs),
            {k: torch.from_numpy(v) for k, v in ri.items()},
            torch.from_numpy(terminated),
            torch.from_numpy(truncated),
        )


def make_tensor_backend(args):
    """统一工厂:rapier/mujoco→Adapter(numpy 后端,仅 cpu);mujoco_warp→原生张量(device 可选)。"""
    device = getattr(args, "device", "cpu")
    lo = args.intent_range
    intent_range = [[lo[0], lo[1]], [lo[2], lo[3]], [lo[4], lo[5]]]
    if args.backend == "mujoco_warp":
        from quad.backends.mujoco_warp import MujocoWarpBackend

        return MujocoWarpBackend(
            args.body,
            args.manifest,
            n_envs=args.n_envs,
            episode_controls=args.episode_controls,
            intent_range=intent_range,
            mjcf_path=args.mjcf,
            device=device,
        )
    if args.backend not in ("rapier", "mujoco"):
        raise SystemExit(f"未知后端 `{args.backend}`(只支持 rapier|mujoco|mujoco_warp)")
    if torch.device(device).type != "cpu":
        raise SystemExit(
            f"--backend {args.backend} 只在 CPU(NumpyToTensorAdapter device=cpu),--device 请用 cpu"
        )
    if args.backend == "mujoco":
        if args.mjcf is None:
            raise SystemExit("--backend mujoco 需要 --mjcf")
        from quad.backends.mujoco import MujocoBackend

        be = MujocoBackend(
            args.body,
            args.manifest,
            n_envs=args.n_envs,
            threads=args.threads,
            episode_controls=args.episode_controls,
            intent_range=intent_range,
            mjcf_path=args.mjcf,
        )
    else:
        from quad.backends.rapier import RapierBackend

        be = RapierBackend(
            args.body,
            args.manifest,
            n_envs=args.n_envs,
            threads=args.threads,
            episode_controls=args.episode_controls,
            intent_range=intent_range,
        )
    return NumpyToTensorAdapter(be)