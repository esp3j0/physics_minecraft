"""rapier3d 0.36.0 API 终验:collider 工厂在 Collider 类上。产出 demos/_API_NOTES.md 的依据。"""
import time

import rapier3d as r

print("version:", r.__version__)

world = r.PhysicsWorld((0.0, -9.81, 0.0))
print("gravity:", world.gravity)

# 地面
ground = world.add_body(
    r.RigidBodyBuilder(r.RigidBodyType.FIXED).translation((0.0, -0.5, 0.0)),
    colliders=[r.Collider.cuboid(50.0, 0.5, 50.0)],
)
print("ground handle:", ground, type(ground).__name__)

# 球(测链式设置)
try:
    ball = world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC).translation((0.0, 10.0, 0.0)),
        colliders=[r.Collider.ball(0.5).restitution(0.3).density(1.0)],
    )
    print("ball 链式 .restitution().density() OK")
except Exception as e:
    print("链式失败,退回无链式:", e)
    ball = world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC).translation((0.0, 10.0, 0.0)),
        colliders=[r.Collider.ball(0.5)],
    )

# 读回方式
bodies = world.rigid_bodies
print("rigid_bodies type:", type(bodies).__name__)
try:
    body_obj = bodies[ball]
    print("bodies[handle] OK:", type(body_obj).__name__)
except Exception as e:
    print("bodies[handle] 失败:", e)
    body_obj = None
    try:
        for item in bodies:
            print("  iter item:", type(item).__name__, item if not hasattr(item, "translation") else item.translation)
            if hasattr(item, "is_dynamic") and item.is_dynamic:
                body_obj = item
    except Exception as e2:
        print("iter 失败:", e2, "| 试 .get():", hasattr(bodies, "get"))

if body_obj is not None:
    t0 = time.perf_counter()
    n = 120
    for _ in range(n):
        world.step()
    dt_ms = (time.perf_counter() - t0) * 1000
    print(f"{n} steps: {dt_ms:.2f} ms")
    print("2s 后球位置:", body_obj.translation, "| sleeping:", body_obj.is_sleeping)
    print("activation 字段:", [a for a in dir(body_obj.activation) if not a.startswith("_")])

# snapshot / restore 一致性
blob = world.snapshot()
w2 = r.PhysicsWorld.restore(blob)
print("snapshot bytes:", len(blob), "| restore OK")
rb2 = w2.rigid_bodies
try:
    same = rb2[ball].translation
    print("restore 后同 handle 读位置:", same)
except Exception as e:
    print("restore 读回方式待定:", e)

# 线程
world.set_num_threads(2)
world.step()
print("set_num_threads(2)+step OK")

# dt
ip = world.integration_parameters
print("IntegrationParameters 全字段:", [a for a in dir(ip) if not a.startswith("_")])
