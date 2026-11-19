"""MuJoCo-Warp GPU 后端(E02f D2/D3)。

物理 mujoco-warp 批推进,观测/解码/奖励/重置全在 torch CUDA 张量上做,
端到端留 GPU(不沿用 numpy VecBackend,避免每步 host 往返)。语义参照
`backends/mujoco.py`(CPU):批提取逐字同 `_extract_single`,初态同
`_build_initial`,自动 reset 同口径;编码/解码/机身系/奖励与
agent_env(Rust)及 quad.task.reward(numpy 生产)由 tests_gpu parity 强制一致。

坑(E02e 探针):mujoco 4.0 无 mjx;`mw.make_data` 必须传 host
`mujoco.MjModel`(传 warp model 触发上游 io.py 缺陷)。
"""

from __future__ import annotations

import numpy as np
import mujoco
import mujoco_warp as mw
import warp as wp
import torch

import agent_env

_FULLPHYSICS = mujoco.mjtState.mjSTATE_FULLPHYSICS.value


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


def quat_mat_wxyz(q: torch.Tensor) -> torch.Tensor:
    """[n,4] wxyz → [n,3,3] 旋转矩阵(世界←机身)。"""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - w * z)
    r02 = 2 * (x * z + w * y)
    r10 = 2 * (x * y + w * z)
    r11 = 1 - 2 * (x * x + z * z)
    r12 = 2 * (y * z - w * x)
    r20 = 2 * (x * z - w * y)
    r21 = 2 * (y * z + w * x)
    r22 = 1 - 2 * (x * x + y * y)
    return torch.stack(
        [torch.stack([r00, r01, r02], -1),
         torch.stack([r10, r11, r12], -1),
         torch.stack([r20, r21, r22], -1)],
        -2,
    )


def body_frame_gpu(quat_xyzw: torch.Tensor, linvel: torch.Tensor, angvel: torch.Tensor) -> dict:
    """语义同 agent_env.body_frame_batch:inv=Rᵀ 作用世界向量;up_y=(R·ŷ).y。"""
    q = quat_xyzw[:, [3, 0, 1, 2]]
    r = quat_mat_wxyz(q)
    rt = r.transpose(-1, -2)
    lin = torch.einsum("nij,nj->ni", rt, linvel)
    ang = torch.einsum("nij,nj->ni", rt, angvel)
    grav = torch.einsum("nij,j->ni", rt, torch.tensor([0.0, -1.0, 0.0], device=linvel.device))
    up = torch.einsum("nij,j->ni", r, torch.tensor([0.0, 1.0, 0.0], device=linvel.device))
    return {
        "base_linvel_body": lin,
        "base_angvel_body": ang,
        "projected_gravity": grav,
        "up_y": up[:, 1:2],
    }


def spec_tensors(spec: dict, device) -> dict:
    """spec 常量一次上 GPU(编码/解码共用;parity 测试同入口)。"""
    sp = dict(spec)
    sp["_default_pose_t"] = torch.as_tensor(
        sp["default_pose"], device=device, dtype=torch.float32
    )
    lo = [(-1e9 if v is None else v) for v in sp["joint_lower"]]
    hi = [(1e9 if v is None else v) for v in sp["joint_upper"]]
    sp["_lower_t"] = torch.as_tensor(lo, device=device, dtype=torch.float32)
    sp["_upper_t"] = torch.as_tensor(hi, device=device, dtype=torch.float32)
    return sp


def encode_gpu(spec: dict, ex: dict, last_action: torch.Tensor, intent: torch.Tensor):
    """语义同 agent_env.Codec.encode_batch + body_frame_batch(obs 未归一化 manifest)。"""
    bf = body_frame_gpu(ex["quat"], ex["linvel"], ex["angvel"])
    parts = []
    for name, scale in spec["obs_terms"]:
        s = float(scale)
        if name == "base_lin_vel":
            parts.append(bf["base_linvel_body"] * s)
        elif name == "base_ang_vel":
            parts.append(bf["base_angvel_body"] * s)
        elif name == "projected_gravity":
            parts.append(bf["projected_gravity"] * s)
        elif name == "joint_pos":
            parts.append((ex["joint_pos"] - spec["_default_pose_t"]) * s)
        elif name == "joint_vel":
            parts.append(ex["joint_vel"] * s)
        elif name == "last_action":
            parts.append(last_action * s)
        elif name == "intent":
            parts.append(intent * s)
        else:
            raise ValueError(f"未知 obs 项 {name}")
    obs = torch.cat(parts, dim=-1)
    if spec["obs_clip"] is not None:
        obs = obs.clamp(-float(spec["obs_clip"]), float(spec["obs_clip"]))
    return obs, bf


def decode_gpu(spec: dict, actions: torch.Tensor):
    """语义同 agent_env.Codec.decode_batch:raw=clip(actions);targets=default+scale*raw 夹限位。"""
    if spec["action_clip"] is not None:
        raw = actions.clamp(-float(spec["action_clip"]), float(spec["action_clip"]))
    else:
        raw = actions.clone()
    t = spec["_default_pose_t"] + float(spec["action_scale"]) * raw
    lo = spec["_lower_t"]
    hi = spec["_upper_t"]
    over = (t < lo) | (t > hi)
    targets = torch.max(torch.min(t, hi), lo)
    clamped = over.sum(dim=-1, dtype=torch.int32)
    return raw, targets, clamped


class MujocoWarpBackend:
    def __init__(
        self,
        body_toml_path: str,
        manifest_toml_path: str,
        n_envs: int,
        episode_controls: int,
        intent_range=None,
        settle_ticks: int = 120,
        fall_up_y_min: float = 0.5,
        mjcf_path: str | None = None,
        device: str = "cuda:0",
    ):
        if mjcf_path is None:
            raise ValueError("MuJoCo-Warp 后端需要 --mjcf")
        if n_envs < 1:
            raise ValueError("n_envs 须 ≥ 1")
        self.device = torch.device(device)
        body_toml = _read(body_toml_path)
        manifest_toml = _read(manifest_toml_path)
        self.spec_dict = agent_env.spec_for(
            body_toml, manifest_toml, episode_controls, fall_up_y_min
        )
        sp = self.spec_dict
        self.n_envs = n_envs
        self.episode_controls = episode_controls
        self.fall_up_y_min = fall_up_y_min
        self.obs_dim = sp["obs_dim"]
        self.act_dim = sp["act_dim"]
        self.dof = sp["dof"]
        self.control_period_ticks = sp["control_period_ticks"]
        self.tick_hz = sp["tick_hz"]
        if intent_range is None:
            intent_range = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.intent_range = [list(map(float, r)) for r in intent_range]

        self._host_model = mujoco.MjModel.from_xml_path(mjcf_path)
        m = self._host_model
        self.substeps = round(1.0 / (self.tick_hz * m.opt.timestep))
        if abs(self.tick_hz * self.substeps * m.opt.timestep - 1.0) > 1e-9:
            raise ValueError(
                f"timestep {m.opt.timestep} 无法整除 tick_hz {self.tick_hz}"
            )
        self._qadr = np.empty(self.dof, dtype=np.int64)
        self._dadr = np.empty(self.dof, dtype=np.int64)
        for i in range(self.act_dim):
            j = m.actuator_trnid[i, 0]
            self._qadr[i] = m.jnt_qposadr[j]
            self._dadr[i] = m.jnt_dofadr[j]
        self._root_body = int(m.jnt_bodyid[0])
        self._qadr_t = torch.as_tensor(self._qadr, device=self.device)
        self._dadr_t = torch.as_tensor(self._dadr, device=self.device)

        # spec 常量上 GPU(一次)。
        self.spec_dict = spec_tensors(self.spec_dict, self.device)

        # 初态同 CPU _build_initial:host settle 后 FULLPHYSICS、time=0,广播 n_envs。
        self._initial = self._build_initial(settle_ticks)
        self.initial_root_height = float(self._initial["qpos"][1])

        # put_model/make_data 无 device 参数,靠 warp 当前设备作用域(E02g 探针)。
        with wp.ScopedDevice(str(self.device)):
            self._m = mw.put_model(self._host_model)
            # 坑:make_data 必须传 host model(传 warp model 触发上游 bug)。
            self._d = mw.make_data(self._host_model, nworld=n_envs)

        self._base_seed = None
        self._intent = torch.zeros((n_envs, 3), device=self.device, dtype=torch.float32)
        self._prev_action = torch.zeros((n_envs, self.act_dim), device=self.device, dtype=torch.float32)
        self._steps = torch.zeros(n_envs, device=self.device, dtype=torch.int64)
        self._episode = torch.zeros(n_envs, device=self.device, dtype=torch.int64)

    def _build_initial(self, settle_ticks: int) -> dict:
        m = self._host_model
        d = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, d, 0)
        d.qpos[1] = agent_env.SPAWN_HEIGHT
        d.ctrl[:] = m.key_ctrl[0]
        for _ in range((1 + settle_ticks) * self.substeps):
            mujoco.mj_step(m, d)
        return {
            "qpos": d.qpos.copy(),
            "qvel": d.qvel.copy(),
            "act": d.act.copy(),
            "warmstart": d.qacc_warmstart.copy(),
        }

    def _write_initial(self, rows: torch.Tensor | None = None) -> None:
        d = self._d
        ini = self._initial
        qp = wp.to_torch(d.qpos)
        qv = wp.to_torch(d.qvel)
        ws = wp.to_torch(d.qacc_warmstart)
        if rows is None:
            qp.copy_(torch.as_tensor(ini["qpos"], device=self.device, dtype=qp.dtype).expand_as(qp))
            qv.copy_(torch.as_tensor(ini["qvel"], device=self.device, dtype=qv.dtype).expand_as(qv))
            ws.copy_(torch.as_tensor(ini["warmstart"], device=self.device, dtype=ws.dtype).expand_as(ws))
            wp.to_torch(d.time).zero_()
        else:
            qp[rows] = torch.as_tensor(ini["qpos"], device=self.device, dtype=qp.dtype)
            qv[rows] = torch.as_tensor(ini["qvel"], device=self.device, dtype=qv.dtype)
            ws[rows] = torch.as_tensor(ini["warmstart"], device=self.device, dtype=ws.dtype)
            wp.to_torch(d.time)[rows] = 0.0
        if len(ini["act"]):
            ac = wp.to_torch(d.act)
            if rows is None:
                ac.copy_(torch.as_tensor(ini["act"], device=self.device, dtype=ac.dtype).expand_as(ac))
            else:
                ac[rows] = torch.as_tensor(ini["act"], device=self.device, dtype=ac.dtype)
        # 同 CPU obs_from_state 的 mj_forward:写状态后刷新 xpos/xipos 再提取。
        mw.forward(self._m, self._d)

    def _extract(self) -> dict:
        """批提取(GPU),语义逐字同 CPU _extract_single。"""
        d = self._d
        qpos = wp.to_torch(d.qpos)
        qvel = wp.to_torch(d.qvel)
        quat = qpos[:, [4, 5, 6, 3]]
        r = quat_mat_wxyz(qpos[:, 3:7])
        ang_world = torch.einsum("nij,nj->ni", r, qvel[:, 3:6])
        c = wp.to_torch(d.xipos)[:, self._root_body] - wp.to_torch(d.xpos)[:, self._root_body]
        lin_world = qvel[:, 0:3] + torch.cross(ang_world, c, dim=-1)
        return {
            "quat": quat,
            "linvel": lin_world,
            "angvel": ang_world,
            "root_height": qpos[:, 1:2],
            "joint_pos": qpos[:, self._qadr_t],
            "joint_vel": qvel[:, self._dadr_t],
        }

    def reset(self, seed: int) -> torch.Tensor:
        self._base_seed = int(seed)
        self._episode.zero_()
        self._steps.zero_()
        self._prev_action.zero_()
        intent = np.empty((self.n_envs, 3), dtype=np.float32)
        for i in range(self.n_envs):
            s = agent_env.episode_seed(seed, i, 0)
            intent[i] = agent_env.sample_intent(s, self.intent_range)
        self._intent = torch.as_tensor(intent, device=self.device)
        self._write_initial()
        ex = self._extract()
        obs, _ = encode_gpu(
            self.spec_dict, ex,
            torch.zeros((self.n_envs, self.act_dim), device=self.device),
            self._intent,
        )
        return obs

    def step(self, actions: torch.Tensor):
        if self._base_seed is None:
            raise RuntimeError("未 reset 就 step")
        if actions.shape != (self.n_envs, self.act_dim):
            raise ValueError(
                f"动作形状应为 ({self.n_envs},{self.act_dim}),得到 {tuple(actions.shape)}"
            )
        if not torch.isfinite(actions).all():
            raise ValueError("动作含非有限值")
        actions = actions.to(device=self.device, dtype=torch.float32)
        raw, targets, clamped = decode_gpu(self.spec_dict, actions)
        wp.to_torch(self._d.ctrl).copy_(targets)
        nstep = self.control_period_ticks * self.substeps
        for _ in range(nstep):
            mw.step(self._m, self._d)
        self._steps += 1

        ex = self._extract()
        obs, bf = encode_gpu(self.spec_dict, ex, raw, self._intent)
        up_y = bf["up_y"][:, 0]
        terminated = up_y < self.fall_up_y_min
        truncated = (~terminated) & (self._steps >= self.episode_controls)

        ri = {
            "base_linvel_body": bf["base_linvel_body"],
            "base_angvel_body": bf["base_angvel_body"],
            "projected_gravity": bf["projected_gravity"],
            "root_height": ex["root_height"],
            "up_y": bf["up_y"],
            "joint_pos": ex["joint_pos"],
            "joint_vel": ex["joint_vel"],
            "action": raw,
            "prev_action": self._prev_action.clone(),
            "targets_clamped": clamped.reshape(self.n_envs, 1).to(torch.float32),
            "intent": self._intent.clone(),
        }

        done = terminated | truncated
        if done.any():
            rows = torch.nonzero(done, as_tuple=False).flatten()
            self._episode[rows] += 1
            intent = self._intent.cpu().numpy()
            for i in rows.cpu().tolist():
                s = agent_env.episode_seed(self._base_seed, int(i), int(self._episode[i]))
                intent[i] = agent_env.sample_intent(s, self.intent_range)
            self._intent = torch.as_tensor(intent, device=self.device)
            self._write_initial(rows)
            self._steps[rows] = 0
            self._prev_action[rows] = 0.0
            ex_new = self._extract()
            last = raw.clone()
            last[rows] = 0.0
            obs, _ = encode_gpu(self.spec_dict, ex_new, last, self._intent)
        self._prev_action = raw.clone()
        self._prev_action[done] = 0.0
        return obs, ri, terminated, truncated

    def sim_time(self, env: int) -> float:
        return float(wp.to_torch(self._d.time)[env])