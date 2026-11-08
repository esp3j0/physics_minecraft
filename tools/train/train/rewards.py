"""奖励(R2):只读 reward_inputs,按 spec.REWARD_W 的权重组合。

形如(R2 再补充):−距离 − 小权重×动作变化 + 到达奖励(距离 ≤ 3 cm)。
动作变化用 reward_inputs 里的 act(上一步动作)与策略本步动作的差;
reach 是稀疏到达项,只在 dist ≤ spec.REACH_DIST 时给。

每控制步记一次(30 Hz);权重初值与最终值都写 README。
"""

from __future__ import annotations

import numpy as np

from train import spec


def reward(
    reward_inputs: dict[str, np.ndarray],
    action: np.ndarray,
    last_action: np.ndarray,
    step: int = -1,
) -> np.ndarray:
    """float32 [n_envs]。向量化的逐步奖励;不修改输入。

    step(回合内控制步号 0..89)用于"保持窗口加权":L1 判据是回合
    最后 0.5 s(15 步)末端保持在目标 3 cm 内,所以最后 15 步的
    dist/reach 项 ×10(主导 return 的形状,直接对齐判据;其余步只给
    行进 shaping)。"""
    dist = reward_inputs["dist"][:, 0]
    act1 = reward_inputs["act1"][:, 0]
    act2 = reward_inputs["act2"][:, 0]
    prev1 = last_action[:, 0]
    prev2 = last_action[:, 1]
    w = spec.REWARD_W
    hold = 3.0 if 0 <= step and step >= spec.EPISODE_CONTROLS - 15 else 1.0
    # 距离项 = 线性 0.5×dist(远处恒梯度;tanh(4d) 在 d≈1 处梯度仅 0.007,
    # 策略远处感受不到距离)+ tanh(4×dist)(近处陡,鼓励精修)。
    r = -w["dist"] * hold * (0.5 * dist + np.tanh(4.0 * dist))
    da1 = act1 - prev1
    da2 = act2 - prev2
    r -= w["act_rate"] * (da1 * da1 + da2 * da2)
    # 分级接近奖励:0.3 m / 0.1 m / 0.03 m 三级阶梯,形成"越近越值钱"
    # 的梯度(rewards 写 README)。
    reached = dist <= spec.REACH_DIST
    tier = (
        (dist <= 0.30).astype(np.float32) * 0.3
        + (dist <= 0.10).astype(np.float32) * 0.7
        + reached.astype(np.float32) * 1.0
    )
    r += w["reach"] * hold * tier
    # 对齐 shaping(只在没有到达时):末端速度在目标方向上的分量(>0
    # 才奖;朝目标移动的密集梯度)。到达后停住由 reach 持续给,避免
    # "冲过头再被 align 拉回来"的振荡解。
    tox = reward_inputs["tox"][:, 0]
    toy = reward_inputs["toy"][:, 0]
    tip_vx = reward_inputs["tip_vx"][:, 0]
    tip_vy = reward_inputs["tip_vy"][:, 0]
    v_along = tip_vx * tox + tip_vy * toy
    r += w["align"] * np.maximum(0.0, v_along) * (~reached).astype(np.float32)
    return r.astype(np.float32)
