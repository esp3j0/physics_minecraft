"""MuJoCo CPU 后端(E02c-1 §3)。

MuJoCo 只负责物理:观测编码、动作解码、机身系换算、意图采样、回合种子全部
调 Rust 里现有的同一份代码(agent_env.Codec / body_frame_batch / episode_seed /
sample_intent / spec_for)。Python 侧只做"从 MuJoCo 取状态"。

控制步 = control_period_ticks 个 tick,每 tick substeps 个 mj_step;用
mujoco.rollout 批量推进 n_envs 个环境。warmstart 不跨控制步保存(与连续
mj_step 的已知差异,不修)。
"""

from __future__ import annotations

import numpy as np
import mujoco
from mujoco import rollout

import agent_env

_FULLPHYSICS = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
_CTRL = mujoco.mjtState.mjSTATE_CTRL.value
_TOL = 1e-6


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


class MujocoBackend:
    def __init__(
        self,
        body_toml_path: str,
        manifest_toml_path: str,
        n_envs: int,
        threads: int,
        episode_controls: int,
        intent_range=None,
        settle_ticks: int = 120,
        fall_up_y_min: float = 0.5,
        mjcf_path: str | None = None,
    ):
        if mjcf_path is None:
            raise ValueError("MuJoCo 后端需要 --mjcf")
        if n_envs < 1 or threads < 1:
            raise ValueError("n_envs / threads 须 ≥ 1")
        body_toml = _read(body_toml_path)
        manifest_toml = _read(manifest_toml_path)
        self.spec_dict = agent_env.spec_for(
            body_toml, manifest_toml, episode_controls, fall_up_y_min
        )
        self._codec = agent_env.Codec(body_toml, manifest_toml)
        self.n_envs = n_envs
        self.threads = threads
        self.episode_controls = episode_controls
        self.fall_up_y_min = fall_up_y_min
        self.obs_dim = self.spec_dict["obs_dim"]
        self.act_dim = self.spec_dict["act_dim"]
        self.dof = self.spec_dict["dof"]
        self.control_period_ticks = self.spec_dict["control_period_ticks"]
        self.tick_hz = self.spec_dict["tick_hz"]
        if intent_range is None:
            intent_range = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.intent_range = [list(map(float, r)) for r in intent_range]

        self._model = mujoco.MjModel.from_xml_path(mjcf_path)
        m = self._model
        self.substeps = round(1.0 / (self.tick_hz * m.opt.timestep))
        if abs(self.tick_hz * self.substeps * m.opt.timestep - 1.0) > 1e-9:
            raise ValueError(
                f"timestep {m.opt.timestep} 无法整除 tick_hz {self.tick_hz}"
            )
        self._check_consistency(m)

        # 自由度顺序 → qpos/qvel 地址(经执行器→关节,不假设 qpos[7:])。
        self._qadr = np.empty(self.dof, dtype=np.int64)
        self._dadr = np.empty(self.dof, dtype=np.int64)
        for i in range(self.act_dim):
            j = m.actuator_trnid[i, 0]
            self._qadr[i] = m.jnt_qposadr[j]
            self._dadr[i] = m.jnt_dofadr[j]
        self._root_body = int(m.jnt_bodyid[0])
        self._nstate = mujoco.mj_stateSize(m, _FULLPHYSICS)

        self._scratch = mujoco.MjData(m)
        self._rollout = rollout.Rollout(nthread=threads)
        self._worker_data = [mujoco.MjData(m) for _ in range(threads)]
        self._initial = self._build_initial(settle_ticks)

        self._base_seed = None
        self._states = np.zeros((n_envs, self._nstate), dtype=np.float64)
        self._intent = np.zeros((n_envs, 3), dtype=np.float32)
        self._prev_action = np.zeros((n_envs, self.act_dim), dtype=np.float32)
        self._steps = np.zeros(n_envs, dtype=np.int64)
        self._episode = np.zeros(n_envs, dtype=np.int64)

    def _check_consistency(self, m) -> None:
        sp = self.spec_dict
        bad: list[str] = []
        name = m.names.split(b"\x00")[0].decode()
        if name != sp["body_name"]:
            bad.append(f"model name {name!r} != body_name {sp['body_name']!r}")
        if m.nu != self.dof:
            bad.append(f"nu {m.nu} != dof {self.dof}")

        def close(a, b):
            return abs(float(a) - float(b)) <= _TOL

        for i in range(min(m.nu, self.dof)):
            if not close(m.actuator_gainprm[i, 0], sp["kp"][i]):
                bad.append(f"actuator {i} kp {m.actuator_gainprm[i, 0]} != {sp['kp'][i]}")
            if not close(-m.actuator_biasprm[i, 2], sp["kd"][i]):
                bad.append(f"actuator {i} kd {-m.actuator_biasprm[i, 2]} != {sp['kd'][i]}")
            mt = sp["max_torque"][i]
            if mt is not None:
                if not close(m.actuator_forcerange[i, 0], -mt) or not close(
                    m.actuator_forcerange[i, 1], mt
                ):
                    bad.append(f"actuator {i} forcerange {m.actuator_forcerange[i]} != ±{mt}")
            lo, up = sp["joint_lower"][i], sp["joint_upper"][i]
            if lo is not None and up is not None:
                if not close(m.actuator_ctrlrange[i, 0], lo) or not close(
                    m.actuator_ctrlrange[i, 1], up
                ):
                    bad.append(
                        f"actuator {i} ctrlrange {m.actuator_ctrlrange[i]} != [{lo},{up}]"
                    )
        if bad:
            raise ValueError("MJCF 与 spec 不一致:\n  " + "\n  ".join(bad))

    def _build_initial(self, settle_ticks: int) -> np.ndarray:
        m = self._model
        d = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, d, 0)
        d.qpos[1] = agent_env.SPAWN_HEIGHT
        d.ctrl[:] = m.key_ctrl[0]
        for _ in range((1 + settle_ticks) * self.substeps):
            mujoco.mj_step(m, d)
        state = np.empty(self._nstate, dtype=np.float64)
        mujoco.mj_getState(m, d, state, _FULLPHYSICS)
        state[0] = 0.0
        return state

    def _extract_single(self, d) -> dict:
        """从已 forward 的 MjData 取根/关节状态(世界系速度)。step 与 obs_from_state 共用。"""
        qpos, qvel = d.qpos, d.qvel
        qw, qx, qy, qz = qpos[3:7]
        quat = np.array([qx, qy, qz, qw], dtype=np.float32)
        R = d.xmat[self._root_body].reshape(3, 3)
        ang_world = R @ qvel[3:6]
        c = d.xipos[self._root_body] - d.xpos[self._root_body]
        lin_world = qvel[0:3] + np.cross(ang_world, c)
        jp = np.array([qpos[a] for a in self._qadr], dtype=np.float32)
        jv = np.array([qvel[a] for a in self._dadr], dtype=np.float32)
        return {
            "quat": quat,
            "linvel": lin_world.astype(np.float32),
            "angvel": ang_world.astype(np.float32),
            "root_height": float(qpos[1]),
            "joint_pos": jp,
            "joint_vel": jv,
        }

    def _extract_batch(self, states: np.ndarray) -> dict:
        m = self._model
        n = states.shape[0]
        quat = np.empty((n, 4), dtype=np.float32)
        linvel = np.empty((n, 3), dtype=np.float32)
        angvel = np.empty((n, 3), dtype=np.float32)
        root_height = np.empty((n, 1), dtype=np.float32)
        joint_pos = np.empty((n, self.dof), dtype=np.float32)
        joint_vel = np.empty((n, self.dof), dtype=np.float32)
        for i in range(n):
            mujoco.mj_setState(m, self._scratch, states[i], _FULLPHYSICS)
            mujoco.mj_forward(m, self._scratch)
            e = self._extract_single(self._scratch)
            quat[i] = e["quat"]
            linvel[i] = e["linvel"]
            angvel[i] = e["angvel"]
            root_height[i, 0] = e["root_height"]
            joint_pos[i] = e["joint_pos"]
            joint_vel[i] = e["joint_vel"]
        return {
            "quat": quat,
            "linvel": linvel,
            "angvel": angvel,
            "root_height": root_height,
            "joint_pos": joint_pos,
            "joint_vel": joint_vel,
        }

    def _encode(self, ex: dict, last_action: np.ndarray, intent: np.ndarray):
        obs = self._codec.encode_batch(
            ex["quat"], ex["linvel"], ex["angvel"],
            ex["joint_pos"], ex["joint_vel"], last_action, intent,
        )
        bf = self._codec.body_frame_batch(ex["quat"], ex["linvel"], ex["angvel"])
        return obs, bf

    def reset(self, seed: int) -> np.ndarray:
        self._base_seed = int(seed)
        self._episode[:] = 0
        self._steps[:] = 0
        self._prev_action[:] = 0.0
        for i in range(self.n_envs):
            s = agent_env.episode_seed(seed, i, 0)
            self._intent[i] = agent_env.sample_intent(s, self.intent_range)
            self._states[i] = self._initial
        ex = self._extract_batch(self._states)
        obs, _ = self._encode(
            ex, np.zeros((self.n_envs, self.act_dim), dtype=np.float32), self._intent
        )
        return obs

    def step(self, actions: np.ndarray):
        if self._base_seed is None:
            raise RuntimeError("未 reset 就 step")
        actions = np.asarray(actions, dtype=np.float32)
        if actions.shape != (self.n_envs, self.act_dim):
            raise ValueError(
                f"动作形状应为 ({self.n_envs},{self.act_dim}),得到 {actions.shape}"
            )
        if not np.all(np.isfinite(actions)):
            raise ValueError("动作含非有限值")
        raw, targets, clamped = self._codec.decode_batch(actions)
        nstep = self.control_period_ticks * self.substeps
        ctrl = np.ascontiguousarray(targets.reshape(self.n_envs, 1, self.act_dim))
        sout, _ = self._rollout.rollout(
            self._model, self._worker_data, self._states, ctrl,
            control_spec=_CTRL, nstep=nstep,
        )
        self._states = np.ascontiguousarray(sout[:, -1, :])
        self._steps += 1

        ex = self._extract_batch(self._states)
        obs, bf = self._encode(ex, raw, self._intent)
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
            "prev_action": self._prev_action.copy(),
            "targets_clamped": clamped.reshape(self.n_envs, 1).astype(np.float32),
            "intent": self._intent.copy(),
        }

        # 自动重置(与 VecEnv 同口径):结束的环境 episode+1、新种子/意图、
        # 状态回初态,obs 取新回合第一帧;reward/flags 仍是结束步的。
        done = terminated | truncated
        if done.any():
            for i in np.nonzero(done)[0]:
                self._episode[i] += 1
                s = agent_env.episode_seed(self._base_seed, int(i), int(self._episode[i]))
                self._intent[i] = agent_env.sample_intent(s, self.intent_range)
                self._states[i] = self._initial
                self._steps[i] = 0
                self._prev_action[i] = 0.0
            ex_new = self._extract_batch(self._states)
            last = np.zeros((self.n_envs, self.act_dim), dtype=np.float32)
            last[~done] = raw[~done]
            obs, _ = self._encode(ex_new, last, self._intent)
        self._prev_action = raw.copy()
        self._prev_action[done] = 0.0
        return obs, ri, terminated, truncated

    def obs_from_state(self, qpos, qvel, last_action, intent):
        """单环境纯函数(检查用):从 qpos/qvel 取 obs 与额外量,与 step 共用提取代码。"""
        m = self._model
        self._scratch.qpos[:] = np.asarray(qpos, dtype=np.float64)
        self._scratch.qvel[:] = np.asarray(qvel, dtype=np.float64)
        mujoco.mj_forward(m, self._scratch)
        e = self._extract_single(self._scratch)
        ex = {
            "quat": e["quat"].reshape(1, 4),
            "linvel": e["linvel"].reshape(1, 3),
            "angvel": e["angvel"].reshape(1, 3),
            "joint_pos": e["joint_pos"].reshape(1, self.dof),
            "joint_vel": e["joint_vel"].reshape(1, self.dof),
        }
        la = np.asarray(last_action, dtype=np.float32).reshape(1, self.act_dim)
        it = np.asarray(intent, dtype=np.float32).reshape(1, 3)
        obs, bf = self._encode(ex, la, it)
        extra = {
            "base_linvel_body": bf["base_linvel_body"][0],
            "base_angvel_body": bf["base_angvel_body"][0],
            "projected_gravity": bf["projected_gravity"][0],
            "up_y": float(bf["up_y"][0, 0]),
            "root_height": e["root_height"],
        }
        return obs[0], extra

    def sim_time(self, env: int) -> float:
        return float(self._states[env, 0])