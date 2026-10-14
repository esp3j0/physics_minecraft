"""Demo A:实体跨分片迁移的轨迹连续性与成本(架构验证,对应 REQ-001 分片交互层)。

设计:实体跨分片只发生在 tick 边界,迁移 = 旧世界 remove_body + 新世界同状态重建。
三组对照分离误差来源:
  A 基准  —— 单世界、整块地面,不迁移;
  B 接缝  —— 单世界、两块地面在 x=0 拼接,不迁移(检验接缝本身的影响);
  C 迁移  —— 两世界(左/右各半地面),球越过 x=0 的那个 tick 边界删除+重建。

另测"飞行中迁移"(不接触地面)作对比,以及迁移操作本身的耗时。
运行:demos/_run.py shard-migration  或  .venv/Scripts/python.exe demos/shard-migration/main.py
"""
import time

import rapier3d as r

GRAVITY = (0.0, -9.81, 0.0)
BALL_R = 0.5
FRICTION = 0.6
BOUNDARY = 0.0
TOTAL_TICKS = 400
CROSS_X = 0.0  # 迁移触发:球心 x 越过此值


def make_world() -> r.PhysicsWorld:
    return r.PhysicsWorld(GRAVITY)


def add_ground(w: r.PhysicsWorld, x_center: float, half_x: float) -> r.RigidBodyHandle:
    return w.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.FIXED).translation((x_center, -0.5, 0.0)),
        colliders=[r.Collider.cuboid(half_x, 0.5, 20.0).friction(FRICTION).restitution(0.0)],
    )


def add_ball(w: r.PhysicsWorld, pos, lv):
    return w.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation(pos)
        .linvel(lv),
        colliders=[r.Collider.ball(BALL_R).friction(FRICTION).restitution(0.0).density(1.0)],
    )


def snapshot_state(w: r.PhysicsWorld, handle):
    """提取迁移所需的完整刚体状态(架构上的迁移载荷)。"""
    b = w.rigid_bodies[handle]
    return {
        "translation": tuple(b.translation),
        "rotation_scaled_axis": tuple(b.rotation.scaled_axis),
        "linvel": tuple(b.linvel),
        "angvel": tuple(b.angvel),
    }


def rebuild(w: r.PhysicsWorld, state) -> r.RigidBodyHandle:
    return w.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation(state["translation"])
        .rotation(state["rotation_scaled_axis"])
        .linvel(state["linvel"])
        .angvel(state["angvel"]),
        colliders=[r.Collider.ball(BALL_R).friction(FRICTION).restitution(0.0).density(1.0)],
    )


def run_mode(mode: str, flying: bool):
    """mode: control(整块地面) / seam(接缝不迁移) / migrate(跨世界迁移)。

    返回 (逐 tick 状态列表, 迁移耗时秒或 None)。
    """
    start = (-10.0, 8.0, 0.0) if flying else (-10.0, BALL_R + 0.001, 0.0)
    lv = (10.0, 0.0, 0.0) if flying else (5.0, 0.0, 0.0)

    if mode in ("control", "seam"):
        w = make_world()
        if mode == "control":
            add_ground(w, 0.0, 30.0)          # 整块:x ∈ [-30, 30]
        else:
            add_ground(w, -15.0, 15.0)        # 左半:x ∈ [-30, 0]
            add_ground(w, 15.0, 15.0)         # 右半:x ∈ [0, 30]
        h = add_ball(w, start, lv)
        trace = []
        for _ in range(TOTAL_TICKS):
            w.step()
            b = w.rigid_bodies[h]
            trace.append((tuple(b.translation), tuple(b.linvel)))
        return trace, None

    # migrate:两个世界,左世界起步,tick 边界迁移到右世界
    left, right = make_world(), make_world()
    add_ground(left, -15.0, 15.0)
    add_ground(right, 15.0, 15.0)
    h_left = add_ball(left, start, lv)
    h_right = None
    trace = []
    mig_cost = None
    for tick in range(TOTAL_TICKS):
        if h_right is None:
            left.step()
            b = left.rigid_bodies[h_left]
            pos = tuple(b.translation)
            trace.append((pos, tuple(b.linvel)))
            if pos[0] >= CROSS_X:
                # ---- tick 边界迁移:提取 -> 删除 -> 重建 ----
                state = snapshot_state(left, h_left)
                t0 = time.perf_counter()
                left.remove_body(h_left)
                h_right = rebuild(right, state)
                mig_cost = time.perf_counter() - t0
        else:
            right.step()
            b = right.rigid_bodies[h_right]
            trace.append((tuple(b.translation), tuple(b.linvel)))
    return trace, mig_cost


def err(a, b):
    """两条轨迹的位置误差序列(逐 tick 欧氏距离)。"""
    return [((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2 + (pa[2] - pb[2]) ** 2) ** 0.5
            for (pa, _), (pb, _) in zip(a, b)]


def summarize(name, e, migrate_tick):
    tail = e[migrate_tick:]
    print(f"  {name:<28} max={max(tail):.6e}  mean={sum(tail) / len(tail):.6e}  final={tail[-1]:.6e}")


def experiment(label: str, flying: bool):
    print(f"\n===== {label} =====")
    ctrl, _ = run_mode("control", flying)
    seam, _ = run_mode("seam", flying)
    mig, mig_cost = run_mode("migrate", flying)

    # 找到迁移发生的 tick(球心越过边界的第一个 tick)
    migrate_tick = next(i for i, ((x, _, _), _) in enumerate(ctrl) if x >= CROSS_X)
    end = (ctrl[migrate_tick - 1][0][0] - 0.0) if False else None  # noqa: F841 (留注释用)

    print(f"迁移发生在 tick {migrate_tick}(球从 x<0 进入 x>=0)")
    if mig_cost is not None:
        print(f"单次迁移耗时(remove+add):{mig_cost * 1e6:.1f} µs")

    summarize("C迁移 vs A基准(过界后)", err(mig, ctrl), migrate_tick)
    summarize("B接缝 vs A基准(过界后)", err(seam, ctrl), migrate_tick)
    summarize("C迁移 vs B接缝(过界后)", err(mig, seam), migrate_tick)

    # 迁移瞬间的速度突变:迁移后第一拍 vs 基准同一拍
    v_mig = mig[migrate_tick + 1][1] if migrate_tick + 1 < len(mig) else mig[-1][1]
    v_ctrl = ctrl[migrate_tick + 1][1] if migrate_tick + 1 < len(ctrl) else ctrl[-1][1]
    dv = sum((a - b) ** 2 for a, b in zip(v_mig, v_ctrl)) ** 0.5
    print(f"迁移后第一拍速度 |Δv| = {dv:.6e} m/s(基准同拍 {tuple(round(v, 3) for v in v_ctrl)})")

    # 迁移后是否落稳(滚动模式):终点位置对比
    print(f"终点:基准={tuple(round(v, 4) for v in ctrl[-1][0])} 迁移={tuple(round(v, 4) for v in mig[-1][0])}")


def bench_migration(n: int = 50):
    """重复完整迁移流程 n 次,统计迁移操作耗时分布。"""
    costs = []
    for _ in range(n):
        _, c = run_mode("migrate", flying=False)
        costs.append(c * 1e6)
    costs.sort()
    print(f"\n===== 迁移耗时基准(重复 {n} 次完整滚动迁移) =====")
    print(f"p50={costs[n // 2]:.1f}µs  p90={costs[int(n * 0.9)]:.1f}µs  max={costs[-1]:.1f}µs")


if __name__ == "__main__":
    experiment("滚动迁移(接触态穿越,最苛刻)", flying=False)
    experiment("飞行迁移(抛物线穿越)", flying=True)
    bench_migration()
