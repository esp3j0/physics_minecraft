"""四足后端契约检查(E02b-1 项1–6;E02c-1 加 --backend/--mjcf 与 MuJoCo 项7–14)。

命令行:
  python -m quad.backends.check --backend rapier --body <body.toml> --manifest <m.toml>
  python -m quad.backends.check --backend mujoco --body <body.toml> --manifest <m.toml> --mjcf <model.xml>
逐项打印 PASS/FAIL,有 FAIL 非零退出。
项1–6(两后端):常量单一来源 / 形状类型 / 线程无关 / 到时那一步 / 动作裁剪 / 错误。
项7–12(仅 mujoco,§5 手算期望):时间 / 偏航速度 / 横滚 / 关节映射 / 质心速度 /
  解码同一份 / 初态 / 意图同源 / 跨后端初态(只报数)。
项13–14(仅 mujoco,R1):动作驱动物理 / autoreset 回初态(逐位判据)。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import tempfile
from pathlib import Path

import numpy as np

from quad import spec
from quad.backends.rapier import RapierBackend
from quad.backends.mujoco import MujocoBackend

FAILED: list[str] = []
TOL = 1e-6


def repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def make_backend(name, body, manifest, n, threads, ep, fall, mjcf=None, intent_range=None):
    if name == "rapier":
        return RapierBackend(body, manifest, n, threads, ep, intent_range=intent_range,
                             fall_up_y_min=fall)
    if name == "mujoco":
        return MujocoBackend(body, manifest, n, threads, ep, intent_range=intent_range,
                             fall_up_y_min=fall, mjcf_path=mjcf)
    raise ValueError(f"未知后端 {name}")


def check(ok: bool, msg: str) -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {msg}")
    if not ok:
        FAILED.append(msg)


def bits(a: np.ndarray) -> np.ndarray:
    return np.asarray(a, dtype=np.float32).view(np.uint32)


def obs_offsets(obs_terms: list, dof: int) -> dict:
    off = {}
    start = 0
    for term, _scale in obs_terms:
        dim = dof if term in ("joint_pos", "joint_vel", "last_action") else 3
        off[term] = (start, start + dim)
        start += dim
    return off


def last_action_slice(obs_terms: list, dof: int) -> tuple[int, int]:
    return obs_offsets(obs_terms, dof)["last_action"]


def item1(make, body: str, manifest: str, ep: int, fall: float) -> None:
    print("项1:常量单一来源")
    backend = make(4, 4, ep, fall)
    independent = spec.load_spec(body, manifest, ep, fall)
    bad = spec.spec_matches(independent, backend.spec_dict)
    check(not bad, f"spec_matches(load_spec, env.spec()) 为空(不一致键:{bad})")


def item2(backend) -> None:
    print("项2:形状与类型")
    s = backend.spec_dict
    n = backend.n_envs
    obs_dim, act_dim = s["obs_dim"], s["act_dim"]
    obs = backend.reset(0)
    check(obs.dtype == np.float32, f"reset obs dtype {obs.dtype} == float32")
    check(obs.shape == (n, obs_dim), f"reset obs shape {obs.shape} == ({n},{obs_dim})")
    actions = np.zeros((n, act_dim), dtype=np.float32)
    o, rew, term, trunc = backend.step(actions)
    check(o.dtype == np.float32 and o.shape == (n, obs_dim), f"step obs {o.dtype} {o.shape}")
    check(term.dtype == np.bool_ and term.shape == (n,), f"terminated {term.dtype} {term.shape}")
    check(trunc.dtype == np.bool_ and trunc.shape == (n,), f"truncated {trunc.dtype} {trunc.shape}")
    want_keys = {k: w for k, w in s["reward_keys"]}
    check(set(rew) == set(want_keys), f"reward 键集合 == spec.reward_keys({sorted(want_keys)})")
    for k, w in want_keys.items():
        arr = rew[k]
        check(
            arr.dtype == np.float32 and arr.shape == (n, w),
            f"reward[{k}] {arr.dtype} {arr.shape} == ({n},{w})",
        )


def item3(make, ep: int, fall: float) -> None:
    print("项3:与线程数无关(4 环境,线程 1 vs 4,40 步)")
    n = 4
    b1 = make(n, 1, ep, fall)
    b4 = make(n, 4, ep, fall)
    act_dim = b1.spec_dict["act_dim"]
    o1 = b1.reset(11)
    o4 = b4.reset(11)
    check(np.array_equal(bits(o1), bits(o4)), "reset(11) obs 逐位相同")
    rng = np.random.default_rng(0)
    max_diff = 0
    all_eq = True
    for step in range(40):
        a = rng.uniform(-1.0, 1.0, size=(n, act_dim)).astype(np.float32)
        ob1, _, te1, tr1 = b1.step(a)
        ob4, _, te4, tr4 = b4.step(a)
        eq = (
            np.array_equal(bits(ob1), bits(ob4))
            and np.array_equal(te1, te4)
            and np.array_equal(tr1, tr4)
        )
        if not eq:
            all_eq = False
            max_diff = max(max_diff, int(np.max(np.abs(bits(ob1).astype(np.int64) - bits(ob4).astype(np.int64)))))
            print(f"    第 {step} 步不一致(obs 位差最大 {max_diff})")
    check(all_eq, "40 步 obs/terminated/truncated 每步逐位相同(线程 1 == 线程 4)")


def item4(make, fall: float) -> None:
    print("项4:到时那一步(episode_controls=5)")
    n = 4
    ep = 5
    b = make(n, 4, ep, fall)
    s = b.spec_dict
    act_dim = s["act_dim"]
    clip = s["action_clip"]
    lo, hi = last_action_slice(s["obs_terms"], s["dof"])
    b.reset(0)
    a = np.full((n, act_dim), 0.2, dtype=np.float32)
    clipped = np.clip(a, -clip, clip) if clip is not None else a
    for _ in range(4):
        b.step(a)
    o5, rew5, te5, tr5 = b.step(a)  # 第 5 步
    check(bool(np.all(tr5)), "第 5 步 truncated 全 True")
    check(not bool(np.any(te5)), "第 5 步 terminated 全 False(未摔倒)")
    check(
        np.array_equal(bits(rew5["action"]), bits(np.broadcast_to(clipped, (n, act_dim)))),
        "第 5 步 reward[action] 逐位等于动作裁剪到 action_clip",
    )
    check(
        bool(np.all(o5[:, lo:hi] == 0.0)),
        f"第 5 步 obs last_action 段 [{lo}:{hi}] 全 0(新回合首帧)",
    )
    _, rew6, _, _ = b.step(a)  # 第 6 步(新回合第一步)
    check(bool(np.all(rew6["prev_action"] == 0.0)), "第 6 步 reward[prev_action] 全 0")


def item5(backend) -> None:
    print("项5:动作裁剪")
    s = backend.spec_dict
    n = backend.n_envs
    act_dim, clip = s["act_dim"], s["action_clip"]
    backend.reset(0)
    a = np.full((n, act_dim), 5.0, dtype=np.float32)
    _, rew, _, _ = backend.step(a)
    check(
        bool(np.all(rew["action"] == clip)),
        f"全 5.0 一步,reward[action] 全等于 action_clip={clip}",
    )


def item6(make, ep: int, fall: float) -> None:
    print("项6:错误")
    b = make(1, 1, ep, fall)
    act_dim = b.spec_dict["act_dim"]
    try:
        b.step(np.zeros((1, act_dim), dtype=np.float32))
        check(False, "没 reset 就 step 应抛 RuntimeError(没抛)")
    except RuntimeError:
        check(True, "没 reset 就 step → RuntimeError")
    except Exception as e:  # noqa: BLE001
        check(False, f"没 reset 就 step 抛了 {type(e).__name__}(应 RuntimeError)")
    b.reset(0)
    try:
        b.step(np.zeros((1, act_dim + 1), dtype=np.float32))
        check(False, "动作 shape 错应抛 ValueError(没抛)")
    except ValueError:
        check(True, "动作 shape 错 → ValueError")
    except Exception as e:  # noqa: BLE001
        check(False, f"动作 shape 错抛了 {type(e).__name__}(应 ValueError)")


# ── MuJoCo 专项(§5)──


def _mj_state(b, root_pos, quat_wxyz, joint_delta=None, joint_vel=None,
              linvel=(0, 0, 0), angvel=(0, 0, 0)):
    m = b._model
    qpos = m.key_qpos[0].copy()
    qpos[0:3] = root_pos
    qpos[3:7] = quat_wxyz
    qvel = np.zeros(m.nv)
    qvel[0:3] = linvel
    qvel[3:6] = angvel
    if joint_delta is not None:
        for k in range(b.dof):
            qpos[b._qadr[k]] += joint_delta[k]
    if joint_vel is not None:
        for k in range(b.dof):
            qvel[b._dadr[k]] = joint_vel[k]
    return qpos, qvel


def item7(args) -> None:
    print("项7:时间(quad,reset 后 3 步)")
    b = make_backend("mujoco", args.body, args.manifest, 4, 4,
                     args.episode_controls, args.fall_up_y_min, args.mjcf)
    check(b.substeps == 10, f"substeps == 10(得 {b.substeps})")
    b.reset(0)
    a = np.zeros((4, b.act_dim), dtype=np.float32)
    for _ in range(3):
        b.step(a)
    t = b.sim_time(0)
    check(abs(t - 0.1) <= 1e-9, f"sim_time(0) == 0.1(得 {t},容差 1e-9)")


def item8(args) -> None:
    print("项8:机身系换算与关节映射(手算)")
    b = make_backend("mujoco", args.body, args.manifest, 1, 1,
                     args.episode_controls, args.fall_up_y_min, args.mjcf)
    off = obs_offsets(b.spec_dict["obs_terms"], b.dof)
    c45, s45 = math.cos(math.pi / 4), math.sin(math.pi / 4)
    zero_la = np.zeros(b.dof)
    zero_it = np.zeros(3)

    # 8a 偏航 +90°(绕 y)+ 速度
    qpos, qvel = _mj_state(b, (0, 0.3, 0), (c45, 0, s45, 0),
                           linvel=(1, 0, 0), angvel=(1, 0, 0))
    obs, ex = b.obs_from_state(qpos, qvel, zero_la, zero_it)
    lo, hi = off["base_lin_vel"]
    check(np.allclose(obs[lo:hi], [0, 0, 1], atol=TOL), f"8a base_lin_vel=(0,0,1)(得 {obs[lo:hi]})")
    lo, hi = off["base_ang_vel"]
    check(np.allclose(obs[lo:hi], [1, 0, 0], atol=TOL), f"8a base_ang_vel=(1,0,0)(得 {obs[lo:hi]})")
    lo, hi = off["projected_gravity"]
    check(np.allclose(obs[lo:hi], [0, -1, 0], atol=TOL), f"8a projected_gravity=(0,-1,0)(得 {obs[lo:hi]})")
    lo, hi = off["joint_pos"]
    check(np.allclose(obs[lo:hi], 0, atol=TOL), "8a joint_pos 段全 0")

    # 8b 横滚 +90°(绕 x),速度 0
    qpos, qvel = _mj_state(b, (0, 0.3, 0), (c45, s45, 0, 0))
    obs, ex = b.obs_from_state(qpos, qvel, zero_la, zero_it)
    check(np.allclose(ex["projected_gravity"], [0, 0, 1], atol=TOL),
          f"8b projected_gravity=(0,0,1)(得 {ex['projected_gravity']})")
    check(abs(ex["up_y"]) <= TOL, f"8b up_y=0(得 {ex['up_y']})")
    check(abs(ex["root_height"] - 0.3) <= TOL, f"8b root_height=0.3(得 {ex['root_height']})")

    # 8c 关节映射
    jd = [0.01 * (k + 1) for k in range(b.dof)]
    jv = [0.1 * (k + 1) for k in range(b.dof)]
    qpos, qvel = _mj_state(b, (0, 0.3, 0), (1, 0, 0, 0), joint_delta=jd, joint_vel=jv)
    obs, ex = b.obs_from_state(qpos, qvel, zero_la, zero_it)
    lo, hi = off["joint_pos"]
    check(np.allclose(obs[lo:hi], jd, atol=TOL), "8c joint_pos[k]=0.01·(k+1)")
    lo, hi = off["joint_vel"]
    check(np.allclose(obs[lo:hi], jv, atol=TOL), "8c joint_vel[k]=0.1·(k+1)")

    # 8d 质心速度(go2)
    root = repo_root()
    go2_body = str(root / "assets/bodies/go2.body.toml")
    go2_manifest = str(root / "tools/train/quad/manifests/go2_train.toml")
    go2_mjcf = str(root / "target/e02c/go2.xml")
    g = make_backend("mujoco", go2_body, go2_manifest, 1, 1,
                     args.episode_controls, args.fall_up_y_min, go2_mjcf)
    qpos, qvel = _mj_state(g, (0, 0.3, 0), (1, 0, 0, 0), angvel=(0, 1, 0))
    obs, ex = g.obs_from_state(qpos, qvel, np.zeros(g.dof), zero_it)
    check(np.allclose(ex["base_linvel_body"], [0, 0, -0.0630625], atol=TOL),
          f"8d base_lin_vel=(0,0,-0.0630625)(得 {ex['base_linvel_body']})")


def item9(args) -> None:
    print("项9:解码同一份(action_scale=100)")
    text = Path(args.manifest).read_text(encoding="utf-8")
    text = re.sub(r"action_scale\s*=\s*[-0-9.eE]+", "action_scale = 100.0", text)
    with tempfile.TemporaryDirectory() as td:
        mp = str(Path(td) / "manifest_scale100.toml")
        Path(mp).write_text(text, encoding="utf-8")
        b = make_backend("mujoco", args.body, mp, 4, 4,
                         args.episode_controls, args.fall_up_y_min, args.mjcf)
        b.reset(0)
        _, rew, _, _ = b.step(np.ones((4, b.act_dim), dtype=np.float32))
        check(bool(np.all(rew["targets_clamped"] == 12)), "全 1.0 动作 targets_clamped 全 12")
        b.reset(0)
        _, rew0, _, _ = b.step(np.zeros((4, b.act_dim), dtype=np.float32))
        check(bool(np.all(rew0["targets_clamped"] == 0)), "全 0 动作 targets_clamped 全 0")


def item10(args) -> None:
    print("项10:初态")
    b = make_backend("mujoco", args.body, args.manifest, 4, 4,
                     args.episode_controls, args.fall_up_y_min, args.mjcf)
    o1 = b.reset(0)
    o2 = b.reset(0)
    check(np.array_equal(bits(o1), bits(o2)), "reset(0) 两次 obs 逐位相同")
    b.reset(0)
    _, rew, _, _ = b.step(np.zeros((4, b.act_dim), dtype=np.float32))
    rh = rew["root_height"][:, 0]
    check(bool(np.all((rh > 0.20) & (rh < 0.35))), f"全 0 一步 root_height ∈ (0.20,0.35)(得 {rh})")


def item11(args) -> None:
    print("项11:意图同源(rapier vs mujoco,4 环境,intent_range[-1,1],ep=3)")
    ir = [[-1.0, 1.0]] * 3
    rp = make_backend("rapier", args.body, args.manifest, 4, 4, 3,
                      args.fall_up_y_min, intent_range=ir)
    mj = make_backend("mujoco", args.body, args.manifest, 4, 4, 3,
                      args.fall_up_y_min, args.mjcf, intent_range=ir)
    rp.reset(7)
    mj.reset(7)
    a = np.zeros((4, rp.spec_dict["act_dim"]), dtype=np.float32)
    snap = {}
    for k in range(1, 5):
        _, ri_r, _, _ = rp.step(a)
        _, ri_m, _, _ = mj.step(a)
        if k in (1, 4):
            snap[k] = (ri_r["intent"].copy(), ri_m["intent"].copy())
    for k in (1, 4):
        r, m = snap[k]
        check(np.array_equal(bits(r), bits(m)), f"第 {k} 步 intent 两后端逐位相同")


def item12(args) -> None:
    print("项12:跨后端初态(只报数,不设阈值)")
    rp = make_backend("rapier", args.body, args.manifest, 4, 4,
                      args.episode_controls, args.fall_up_y_min)
    mj = make_backend("mujoco", args.body, args.manifest, 4, 4,
                      args.episode_controls, args.fall_up_y_min, args.mjcf)
    orp = rp.reset(0)
    omj = mj.reset(0)
    off = obs_offsets(rp.spec_dict["obs_terms"], rp.spec_dict["dof"])
    for term, (lo, hi) in off.items():
        d = float(np.max(np.abs(orp[:, lo:hi] - omj[:, lo:hi])))
        print(f"    reset obs[{term}] 最大绝对差 {d:.6e}")
    a = np.zeros((4, rp.spec_dict["act_dim"]), dtype=np.float32)
    _, rr, _, _ = rp.step(a)
    _, rm, _, _ = mj.step(a)
    print(f"    全 0 一步 root_height:rapier {float(np.mean(rr['root_height'])):.6f} "
          f"mujoco {float(np.mean(rm['root_height'])):.6f}")


def item13(args) -> None:
    print("项13:动作驱动物理(全0 vs 全1 一步,joint_pos/joint_vel 段逐位不同)")
    b0 = make_backend("mujoco", args.body, args.manifest, 4, 4,
                      args.episode_controls, args.fall_up_y_min, args.mjcf)
    b1 = make_backend("mujoco", args.body, args.manifest, 4, 4,
                      args.episode_controls, args.fall_up_y_min, args.mjcf)
    b0.reset(0)
    b1.reset(0)
    off = obs_offsets(b0.spec_dict["obs_terms"], b0.spec_dict["dof"])
    o0, _, _, _ = b0.step(np.zeros((4, b0.act_dim), dtype=np.float32))
    o1, _, _, _ = b1.step(np.ones((4, b1.act_dim), dtype=np.float32))
    for term in ("joint_pos", "joint_vel"):
        lo, hi = off[term]
        check(
            not np.array_equal(bits(o0[:, lo:hi]), bits(o1[:, lo:hi])),
            f"全0 vs 全1 一步 obs[{term}] 段逐位不同(动作真的驱动物理)",
        )


def item14(args) -> None:
    print("项14:autoreset 回初态(ep=5,第5步新回合首帧物理段 == reset 首帧)")
    b = make_backend("mujoco", args.body, args.manifest, 4, 4, 5,
                     args.fall_up_y_min, args.mjcf)
    first = b.reset(0)
    off = obs_offsets(b.spec_dict["obs_terms"], b.spec_dict["dof"])
    a = np.ones((4, b.act_dim), dtype=np.float32)
    for _ in range(4):
        b.step(a)
    o5, _, te5, tr5 = b.step(a)  # 第 5 步 truncated → 自动 reset
    check(bool(np.all(tr5)) and not bool(np.any(te5)), "第 5 步 truncated 全 True、未摔倒")
    for term in ("base_lin_vel", "base_ang_vel", "projected_gravity",
                 "joint_pos", "joint_vel", "last_action"):
        lo, hi = off[term]
        check(
            np.array_equal(bits(o5[:, lo:hi]), bits(first[:, lo:hi])),
            f"新回合首帧 obs[{term}] 段与 reset 首帧逐位相同(intent 段除外)",
        )


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--body", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--backend", default="rapier", choices=["rapier", "mujoco"])
    ap.add_argument("--mjcf", default=None)
    ap.add_argument("--episode-controls", type=int, default=15)
    ap.add_argument("--fall-up-y-min", type=float, default=0.5)
    args = ap.parse_args()
    if args.backend == "mujoco" and args.mjcf is None:
        print("--backend mujoco 需要 --mjcf")
        return 2
    ep, fall = args.episode_controls, args.fall_up_y_min

    def make(n, threads, e, f, intent_range=None):
        return make_backend(args.backend, args.body, args.manifest, n, threads, e, f,
                            args.mjcf, intent_range)

    b15 = make(4, 4, ep, fall)
    item1(make, args.body, args.manifest, ep, fall)
    item2(b15)
    item3(make, ep, fall)
    item4(make, fall)
    item5(b15)
    item6(make, ep, fall)

    if args.backend == "mujoco":
        item7(args)
        item8(args)
        item9(args)
        item10(args)
        item11(args)
        item12(args)
        item13(args)
        item14(args)

    if FAILED:
        print(f"==== 结论: 有 FAIL({len(FAILED)} 项)====")
        return 1
    last = 14 if args.backend == "mujoco" else 6
    print(f"==== 结论: 项1–{last} 全部 PASS ====")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())