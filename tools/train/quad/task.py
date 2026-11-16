"""四足任务常量与奖励(E02b-2 §1)。

坐标:x 向前,y 向上,z 侧向;偏航绕 y。intent = [前进, 侧移, 转向],各 ∈ [-1,1]。
奖励只读 reward_inputs(不读后端内部任何东西);terminated 的那一步奖励照算
(回合结束本身就是惩罚)。逐步公式见 E02b-2 任务书 §1。
"""

from __future__ import annotations

import numpy as np

V_FWD_MAX = 0.6  # m/s
V_LAT_MAX = 0.3  # m/s
YAW_MAX = 1.0  # rad/s
H0 = 0.2775  # 站立时机身离地高度(B1a 实测)
SIGMA = 0.25
W = dict(lin=1.0, yaw=0.5, vy=2.0, ang_xz=0.05, orient=1.0, act_rate=0.01, height=30.0)


def command(intent: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """[n,3] -> (cx, cz, cyaw),各 [n]。"""
    return intent[:, 0] * V_FWD_MAX, intent[:, 1] * V_LAT_MAX, intent[:, 2] * YAW_MAX


def reward(ri: dict[str, np.ndarray]) -> np.ndarray:
    """float32 [n]。v=base_linvel_body,ω=base_angvel_body,g=projected_gravity,
    h=root_height,a=action,a'=prev_action。"""
    v = ri["base_linvel_body"]
    w = ri["base_angvel_body"]
    g = ri["projected_gravity"]
    h = ri["root_height"][:, 0]
    a = ri["action"]
    ap = ri["prev_action"]
    cx, cz, cyaw = command(ri["intent"])
    r = W["lin"] * np.exp(-((v[:, 0] - cx) ** 2 + (v[:, 2] - cz) ** 2) / SIGMA)
    r = r + W["yaw"] * np.exp(-((w[:, 1] - cyaw) ** 2) / SIGMA)
    r = r - W["vy"] * v[:, 1] ** 2
    r = r - W["ang_xz"] * (w[:, 0] ** 2 + w[:, 2] ** 2)
    r = r - W["orient"] * (g[:, 0] ** 2 + g[:, 2] ** 2)
    r = r - W["act_rate"] * np.sum((a - ap) ** 2, axis=1)
    r = r - W["height"] * (h - H0) ** 2
    return r.astype(np.float32)