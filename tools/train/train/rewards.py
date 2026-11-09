"""奖励(R3/P1 简单基线):只读 reward_inputs。

R3 放行指定的形式(总架构师探针验证过能学会:230/256 ≤ 3 cm、中位 8 mm):

    r = −dist − act_rate·‖a_t − a_{t−1}‖² + reach·[dist ≤ REACH_DIST]

R2 的 align(+max(0, v·u) 无上限)、tier 阶梯、保持窗口 ×3、线性+tanh
叠加全部撤除——P1 主因:align 来回摆动一半时间都在拿分,回报形状被
shaping 主导,策略学会"甩臂刷分"。R3 纪律:在基线上每加一项都要在
README 说明理由并给加前/加后的 L1 对照;当前权重 = 纯基线(align=0)。

每控制步记一次(30 Hz);权重 = spec.REWARD_W(初值 = 最终值)。
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

    step 参数保留(接口不变,后端适配器按步号传;简单基线不用步号)。
    """
    dist = reward_inputs["dist"][:, 0]
    act = reward_inputs["act1"][:, 0]
    act2 = reward_inputs["act2"][:, 0]
    prev1 = last_action[:, 0]
    prev2 = last_action[:, 1]
    w = spec.REWARD_W
    r = -w["dist"] * dist
    da1 = act - prev1
    da2 = act2 - prev2
    r -= w["act_rate"] * (da1 * da1 + da2 * da2)
    r += w["reach"] * (dist <= spec.REACH_DIST).astype(np.float32)
    return r.astype(np.float32)
