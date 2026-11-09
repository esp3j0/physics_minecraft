"""BackendSpec:后端无关的约定(R2 补充)。

两个后端(rapier 现在写;MuJoCo transfer 阶段写)必须完全一致:
- 观测布局(每一维的名字和顺序)、动作维度;
- 动作到关节目标的映射(增量式、ACT_SCALE、关节顺序、限位);
- 控制频率、reward_inputs 的键和形状;
- 身体标识(暂时写 quad-demo 的变体:arm-demo)。

ppo.py / rewards.py 只依赖本模块和 VecBackend 接口,不 import 任何
后端内部的东西。单一事实来源:Rust 侧 arm.rs 的常量与本文件一致
(契约检查 backends/check.py 抽样核对)。
"""

from __future__ import annotations

import math

BODY = "arm-demo"

# ── 维度 ────────────────────────────────────────────────────────────────
OBS_DIM = 14
ACT_DIM = 2
REWARD_KEYS = ["dist", "tip_vx", "tip_vy", "tau1", "tau2", "act1", "act2", "tox", "toy"]

# ── 观测布局(每维名字;顺序即下标)────────────────────────────────────
OBS_LAYOUT = [
    "q1_sin", "q1_cos", "q2_sin", "q2_cos",
    "q1_vel", "q2_vel",
    "target_x", "target_y",
    "tip_x", "tip_y",
    "target_minus_tip_x", "target_minus_tip_y",
    "last_act1", "last_act2",
]

# ── 身体与控制(Rust arm.rs 同值;control_hz = 60 Hz 物理)─────────────
LINK_LEN = 0.5          # m,两段同
LINK_MASS = 1.0         # kg,两段同
TIP_MASS = 0.01         # kg(末端标记球)
JOINT_LIMIT = math.radians(170.0)   # ±170°
KP = 2000.0             # PD 增益(README 记录选定依据)
KD = 60.0
TAU_LIMIT = 25.0        # N·m
CONTROL_HZ = 30         # 每回合 90 控制步 = 3 s(60 Hz 对照 = 60)
TICKS_PER_CONTROL = 2   # 60 Hz 物理 / 30 Hz 控制
EPISODE_CONTROLS = 90
TARGET_R_MIN = 0.3      # 可达环形区(面积均匀采样)
TARGET_R_MAX = 0.9
INIT_Q_RANGE = 0.5      # 出生角 ±0.5 rad
REACH_DIST = 0.03       # 到达门槛(3 cm)

# ── 动作映射(增量式;选型 README 记录)────────────────────────────────
# R3/P3:统一 0.1(Rust arm.rs 同值;两边曾经各写各的——Rust 0.1、这里
# 0.05,实际训练/评估都走 Rust 值。单一事实来源 = learn_env 导出常量,
# check.py 逐项核对)。
ACT_SCALE = 0.1
# 目标角_{t+1} = clamp(目标角_t + ACT_SCALE × action, −JOINT_LIMIT, +JOINT_LIMIT)
# 目标角从出生角起步;动作 ∈ [−1, 1] 由后端钳位。

# ── 奖励权重(R3/P1 简单基线:总架构师探针行三;每项见 rewards.py)──────
REWARD_W = {
    "dist": 1.0,        # −dist
    "act_rate": 0.01,   # −w·‖a_t − a_{t−1}‖²
    "reach": 1.0,       # +1·[dist ≤ REACH_DIST](稀疏到达项)
    "align": 0.0,       # P1 主因项,撤(可加项须 README 给对照;当前 0 = 不加)
    "tip_speed": 0.0,   # 备用:−w·‖末端速度‖²(不用,置 0)
}


def to_dict() -> dict:
    """spec 的可序列化形式(检查点、policy.toml 用;两个后端一致才接得上)。"""
    return {
        "body": BODY,
        "obs_dim": OBS_DIM,
        "obs_layout": OBS_LAYOUT,
        "act_dim": ACT_DIM,
        "reward_keys": REWARD_KEYS,
        "link_len": LINK_LEN,
        "link_mass": LINK_MASS,
        "tip_mass": TIP_MASS,
        "joint_limit_rad": JOINT_LIMIT,
        "kp": KP,
        "kd": KD,
        "tau_limit": TAU_LIMIT,
        "control_hz": CONTROL_HZ,
        "ticks_per_control": TICKS_PER_CONTROL,
        "episode_controls": EPISODE_CONTROLS,
        "target_r_min": TARGET_R_MIN,
        "target_r_max": TARGET_R_MAX,
        "init_q_range": INIT_Q_RANGE,
        "reach_dist": REACH_DIST,
        "act_scale": ACT_SCALE,
        "act_mode": "incremental",
        "reward_w": dict(REWARD_W),
    }


def spec_matches(other: dict, strict_reward_w: bool = True) -> bool:
    """检查点里的 spec 与当前 spec 是否一致(--init-from 核对)。

    微调(L5)只改物理(质量、增益),不改 spec 的约定;strict_reward_w
    = False 时允许奖励权重不同(策略语义不变,权重是训练超参)。
    """
    a = to_dict()
    if not strict_reward_w:
        a = {k: v for k, v in a.items() if k != "reward_w"}
        other = {k: v for k, v in other.items() if k != "reward_w"}
    return a == other
