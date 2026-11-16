"""四足 BackendSpec(E02b-1):用标准库 tomllib 独立解析身体与 manifest,
算出与 `agent_env.VecEnv.spec()` 同样键的字典。

单一事实来源 = Rust 侧 `spec()`;本模块是独立核对侧(契约检查第 1 项逐键比对)。
自由度顺序 = 身体 `joints` 先后中非 fixed 的那些(与 Rust `dof_joints()` 一致)。
"""

from __future__ import annotations

import tomllib

# 奖励原料键序与定宽(dof 相关键宽 = 自由度数);与 Rust reward_key_widths 一致。
_REWARD_FIXED = [
    ("base_linvel_body", 3),
    ("base_angvel_body", 3),
    ("projected_gravity", 3),
    ("root_height", 1),
    ("up_y", 1),
    ("joint_pos", None),
    ("joint_vel", None),
    ("action", None),
    ("prev_action", None),
    ("targets_clamped", 1),
    ("intent", 3),
]

_TERM_DIM = {
    "base_lin_vel": 3,
    "base_ang_vel": 3,
    "projected_gravity": 3,
    "joint_pos": "dof",
    "joint_vel": "dof",
    "last_action": "dof",
    "intent": 3,
}


def _dof_joints(body: dict) -> list[dict]:
    return [j for j in body.get("joints", []) if j.get("type") != "fixed"]


def reward_keys(dof: int) -> list[list]:
    return [[name, dof if k is None else k] for name, k in _REWARD_FIXED]


def load_spec(
    body_toml_path: str,
    manifest_toml_path: str,
    episode_controls: int,
    fall_up_y_min: float,
) -> dict:
    """独立解析两个文件,返回与 VecEnv.spec() 同键的字典。

    body_hash 直接取 manifest 里写的;tick_hz 写 60;episode_controls、
    fall_up_y_min 由调用方传入。
    """
    with open(body_toml_path, "rb") as f:
        body = tomllib.load(f)
    with open(manifest_toml_path, "rb") as f:
        manifest = tomllib.load(f)

    joints = _dof_joints(body)
    dof = len(joints)
    obs = manifest["obs"]
    obs_dim = sum(_TERM_DIM[e["term"]] if _TERM_DIM[e["term"]] != "dof" else dof for e in obs)
    default_pose = manifest.get("default_pose")
    if default_pose is None:
        default_pose = [j.get("default_angle", 0.0) for j in joints]

    def lim(field: str) -> list:
        return [j["limits"][field] if "limits" in j else None for j in joints]

    return {
        "body_name": body["name"],
        "body_hash": manifest["body_hash"],
        "dof": dof,
        "obs_dim": obs_dim,
        "act_dim": dof,
        "tick_hz": 60,
        "control_period_ticks": manifest["control_period_ticks"],
        "episode_controls": episode_controls,
        "fall_up_y_min": fall_up_y_min,
        "action_scale": manifest["action_scale"],
        "action_clip": manifest.get("action_clip"),
        "obs_clip": manifest.get("obs_clip"),
        "obs_terms": [[e["term"], e.get("scale", 1.0)] for e in obs],
        "default_pose": list(default_pose),
        "joint_lower": lim("lower"),
        "joint_upper": lim("upper"),
        "kp": [j.get("kp") for j in joints],
        "kd": [j.get("kd") for j in joints],
        "max_torque": [j.get("max_torque") for j in joints],
        "reward_keys": reward_keys(dof),
    }


def _eq(a, b) -> bool:
    """递归相等:浮点 abs ≤ 1e-6;tuple/list 视为同类序列;None/str/bool 精确。"""
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_eq(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) <= 1e-6
    return a == b


_MISSING = object()


def spec_matches(a: dict, b: dict) -> list[str]:
    """逐键比较,返回不一致的键列表(空 = 完全一致)。"""
    bad = []
    for k in sorted(set(a) | set(b)):
        if not _eq(a.get(k, _MISSING), b.get(k, _MISSING)):
            bad.append(k)
    return bad