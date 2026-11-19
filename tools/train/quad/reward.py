"""统一奖励(E02h D2):torch 版,逐键加权同 quad.task.reward(numpy 生产 oracle)。

设备无关:CPU 适配器与 warp 张量共用同一份;parity 由测试强制 ≤1e-5。
"""

from __future__ import annotations

import torch

from quad import task


def reward_gpu(ri: dict) -> torch.Tensor:
    """v=base_linvel_body,ω=base_angvel_body,g=projected_gravity,h=root_height。"""
    v = ri["base_linvel_body"]
    w = ri["base_angvel_body"]
    g = ri["projected_gravity"]
    h = ri["root_height"][:, 0]
    a = ri["action"]
    ap = ri["prev_action"]
    cx = ri["intent"][:, 0] * task.V_FWD_MAX
    cz = ri["intent"][:, 1] * task.V_LAT_MAX
    cyaw = ri["intent"][:, 2] * task.YAW_MAX
    r = task.W["lin"] * torch.exp(-((v[:, 0] - cx) ** 2 + (v[:, 2] - cz) ** 2) / task.SIGMA)
    r = r + task.W["yaw"] * torch.exp(-((w[:, 1] - cyaw) ** 2) / task.SIGMA)
    r = r - task.W["vy"] * v[:, 1] ** 2
    r = r - task.W["ang_xz"] * (w[:, 0] ** 2 + w[:, 2] ** 2)
    r = r - task.W["orient"] * (g[:, 0] ** 2 + g[:, 2] ** 2)
    r = r - task.W["act_rate"] * torch.sum((a - ap) ** 2, dim=1)
    r = r - task.W["height"] * (h - task.H0) ** 2
    return r