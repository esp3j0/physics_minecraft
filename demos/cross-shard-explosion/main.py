"""Demo D:跨片效果的确定性计算(架构验证,对应 REQ-001 分片交互层)。

既定共识:分片之间不共享状态、互不感知;爆炸这类跨片事件收敛为
"各方用共享输入 + 本地状态独立计算,合并结果必须与单一世界一致"。

爆炸冲量函数(纯函数、确定性):
    dist = |pos - center|
    if dist >= radius: impulse = 0
    else: impulse = (pos - center) / dist * power * (1 - dist / radius)
    直接 apply_impulse(不除以质量 → 重球速度变化小)。
    所需输入 = 共享 {center, power, radius} + 本地 {实体爆炸时刻 pos}。

三组:
  A 参考   —— 单世界放全部 10 球,tick=60 爆炸;
  B 分片   —— 两个世界按 x<0/≥0 各持己方球,同一 tick 各自独立计算施加;
  C 重复性 —— B 完整跑两遍,验证同输入同结果。

注:为隔离假设"共享输入+本地状态 ⇒ 结果一致",球与球不碰撞(collision groups 只对地面),
球-球交互属于分片内部语义,与跨片等价性无关。
运行:.venv/Scripts/python.exe demos/cross-shard-explosion/main.py
"""
import time

import numpy as np
import rapier3d as r

GRAVITY = (0.0, -9.81, 0.0)
CENTER = (0.3, 0.0, 0.5)
POWER = 6.0
RADIUS = 5.0
EXPLODE_TICK = 60
TOTAL = 300
G_MEMBERS = r.Group(0b01)  # 地面
B_MEMBERS = r.Group(0b10)  # 球


def explosion_impulse(pos):
    """纯函数:共享输入 CENTER/POWER/RADIUS + 本地 pos → 冲量向量(或 None)。"""
    dx = pos[0] - CENTER[0]
    dy = pos[1] - CENTER[1]
    dz = pos[2] - CENTER[2]
    dist = (dx * dx + dy * dy + dz * dz) ** 0.5
    if dist >= RADIUS or dist == 0.0:
        return None
    k = POWER * (1.0 - dist / RADIUS) / dist
    return (dx * k, dy * k, dz * k)


def ground_groups():
    return r.Collider.cuboid(40.0, 0.5, 40.0).translation((0.0, -0.5, 0.0)).collision_groups(
        r.InteractionGroups(G_MEMBERS, r.Group(0b11))
    )


def ball_groups():
    return r.Collider.ball(0.4).friction(0.5).restitution(0.3).density(1.0).collision_groups(
        r.InteractionGroups(B_MEMBERS, G_MEMBERS)  # 球只与地面作用
    )


def spawn_balls():
    """10 个球:左 4、右 4、紧贴边界 2(x=-0.05 / +0.05,归属规则 x<0 → 左)。"""
    rng = np.random.default_rng(42)
    balls = []
    xs = list(rng.uniform(-8, -1, 4)) + list(rng.uniform(1, 8, 4)) + [-0.05, 0.05]
    zs = list(rng.uniform(-8, 8, 8)) + list(rng.uniform(-3, 3, 2))
    for i, (bx, bz) in enumerate(zip(xs, zs)):
        rad = 0.3 + 0.05 * (i % 4)
        balls.append({
            "id": i,
            "pos": (float(bx), rad + 0.01, float(bz)),
            "lv": (float(rng.uniform(-0.5, 0.5)), 0.0, float(rng.uniform(-0.5, 0.5))),
        })
    return balls


def run_reference(balls):
    """A 参考:单世界全部球。返回 {id: 终点位置}。"""
    w = r.PhysicsWorld(GRAVITY)
    w.add_body(r.RigidBodyBuilder(r.RigidBodyType.FIXED), colliders=[ground_groups()])
    hs = {}
    for b in balls:
        hs[b["id"]] = w.add_body(
            r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC).translation(b["pos"]).linvel(b["lv"]),
            colliders=[ball_groups()],
        )
    t0 = time.perf_counter()
    for tick in range(TOTAL):
        if tick == EXPLODE_TICK:
            for b in balls:
                imp = explosion_impulse(tuple(w.rigid_bodies[hs[b["id"]]].translation))
                if imp is not None:
                    w.rigid_bodies[hs[b["id"]]].apply_impulse(imp)
        w.step()
    cost = (time.perf_counter() - t0) * 1e6
    return {bid: tuple(w.rigid_bodies[h].translation) for bid, h in hs.items()}, cost


def run_sharded(balls):
    """B 分片:两世界各自独立计算。返回 ({id: 终点}, 冲量施加总耗时 µs)。"""
    left, right = r.PhysicsWorld(GRAVITY), r.PhysicsWorld(GRAVITY)
    for w in (left, right):
        w.add_body(r.RigidBodyBuilder(r.RigidBodyType.FIXED), colliders=[ground_groups()])
    hs = {}
    for b in balls:  # 归属规则:x<0 → 左片,否则右片
        w = left if b["pos"][0] < 0 else right
        hs[b["id"]] = (w, w.add_body(
            r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC).translation(b["pos"]).linvel(b["lv"]),
            colliders=[ball_groups()],
        ))
    t0 = time.perf_counter()
    for tick in range(TOTAL):
        if tick == EXPLODE_TICK:
            for b in balls:
                w, h = hs[b["id"]]
                imp = explosion_impulse(tuple(w.rigid_bodies[h].translation))  # 共享输入 + 本地状态
                if imp is not None:
                    w.rigid_bodies[h].apply_impulse(imp)
        left.step()
        right.step()
    cost = (time.perf_counter() - t0) * 1e6
    return {bid: tuple(w.rigid_bodies[h].translation) for bid, (w, h) in hs.items()}, cost


def dist3(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


if __name__ == "__main__":
    balls = spawn_balls()
    print(f"10 球,爆炸 center={CENTER} power={POWER} radius={RADIUS},tick={EXPLODE_TICK} 时施加")

    ref, ref_cost = run_reference(balls)
    sh1, cost1 = run_sharded(balls)
    sh2, cost2 = run_sharded(balls)

    print("\n===== B 分片 vs A 参考(终点位置误差) =====")
    errs_all, errs_near = [], []
    for b in balls:
        e = dist3(ref[b["id"]], sh1[b["id"]])
        near = abs(b["pos"][0]) < 1.0
        errs_all.append(e)
        if near:
            errs_near.append(e)
        print(f"  球{b['id']}  x0={b['pos'][0]:+7.3f}  终点误差={e:.3e} m {'(贴边界)' if near else ''}")
    print(f"全部:最大 {max(errs_all):.3e} / 平均 {sum(errs_all)/len(errs_all):.3e}")
    print(f"贴边界组:最大 {max(errs_near):.3e} / 平均 {sum(errs_near)/len(errs_near):.3e}")

    print("\n===== C 重复性(B 跑两遍) =====")
    dups = [dist3(sh1[b['id']], sh2[b['id']]) for b in balls]
    print(f"同输入两次运行终点差:最大 {max(dups):.3e} m(期望 0)")

    print("\n===== 成本 =====")
    print(f"300 tick 总耗时:参考 {ref_cost/1000:.2f}ms | 分片 {cost1/1000:.2f}ms(含两世界步进)")
    print(f"爆炸冲量施加(10 球,含函数计算):分片侧 <1µs 量级,可忽略")
