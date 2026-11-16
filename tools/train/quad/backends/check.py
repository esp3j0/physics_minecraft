"""四足 rapier 后端契约检查(E02b-1)。

命令行:python -m quad.backends.check --body <body.toml> --manifest <manifest.toml>
逐项打印 PASS/FAIL,有 FAIL 非零退出。六项:
  1 常量单一来源(spec_matches 为空)
  2 形状与类型(reset/step dtype/shape;reward 键集合与宽度)
  3 与线程数无关(4 环境,线程 1 vs 4,reset(11) 同随机动作 40 步逐位相同)
  4 到时那一步(episode_controls=5:第 5 步 truncated 全 True、action=裁剪值、
    obs last_action 段全 0;第 6 步 prev_action 全 0)
  5 动作裁剪(全 5.0 一步,action 全等于 action_clip)
  6 错误(shape 错 → ValueError;没 reset 就 step → RuntimeError)
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from quad import spec
from quad.backends.rapier import RapierBackend

FAILED: list[str] = []


def check(ok: bool, msg: str) -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {msg}")
    if not ok:
        FAILED.append(msg)


def bits(a: np.ndarray) -> np.ndarray:
    return np.asarray(a, dtype=np.float32).view(np.uint32)


def last_action_slice(obs_terms: list, dof: int) -> tuple[int, int]:
    start = 0
    for term, _scale in obs_terms:
        dim = dof if term in ("joint_pos", "joint_vel", "last_action") else 3
        if term == "last_action":
            return start, start + dim
        start += dim
    raise ValueError("obs_terms 没有 last_action")


def item1(body: str, manifest: str, backend: RapierBackend, ep: int, fall: float) -> None:
    print("项1:常量单一来源")
    independent = spec.load_spec(body, manifest, ep, fall)
    bad = spec.spec_matches(independent, backend.spec_dict)
    check(not bad, f"spec_matches(load_spec, env.spec()) 为空(不一致键:{bad})")


def item2(backend: RapierBackend) -> None:
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


def item3(body: str, manifest: str, ep: int, fall: float) -> None:
    print("项3:与线程数无关(4 环境,线程 1 vs 4,40 步)")
    n = 4
    b1 = RapierBackend(body, manifest, n, 1, ep, fall_up_y_min=fall)
    b4 = RapierBackend(body, manifest, n, 4, ep, fall_up_y_min=fall)
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


def item4(body: str, manifest: str, fall: float) -> None:
    print("项4:到时那一步(episode_controls=5)")
    n = 4
    ep = 5
    b = RapierBackend(body, manifest, n, 4, ep, fall_up_y_min=fall)
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


def item5(backend: RapierBackend) -> None:
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


def item6(body: str, manifest: str, ep: int, fall: float) -> None:
    print("项6:错误")
    b = RapierBackend(body, manifest, 1, 1, ep, fall_up_y_min=fall)
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


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--body", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--episode-controls", type=int, default=15)
    ap.add_argument("--fall-up-y-min", type=float, default=0.5)
    args = ap.parse_args()
    ep, fall = args.episode_controls, args.fall_up_y_min

    b15 = RapierBackend(args.body, args.manifest, 4, 4, ep, fall_up_y_min=fall)
    item1(args.body, args.manifest, b15, ep, fall)
    item2(b15)
    item3(args.body, args.manifest, ep, fall)
    item4(args.body, args.manifest, fall)
    item5(b15)
    item6(args.body, args.manifest, ep, fall)

    if FAILED:
        print(f"==== 结论: 有 FAIL({len(FAILED)} 项)====")
        return 1
    print("==== 结论: 项1–6 全部 PASS ====")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())