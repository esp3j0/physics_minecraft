# rapier3d Python 绑定 API 说明(demos 共用,已实测)

> 环境已配好:解释器 `C:\Users\esp3j\rep\physics_minecraft\.venv\Scripts\python.exe`,
> 已安装 `rapier3d==0.36.0` + `numpy`。**不要再 pip install 任何东西。**
> ⚠️ `.pyi` 存根与二进制不一致,以下内容全部经过运行时实测,以本文档为准。

## 已验证的最小工作示例

```python
import rapier3d as r

# 重力:构造参数直接给 tuple
world = r.PhysicsWorld((0.0, -9.81, 0.0))

# 静态体:RigidBodyBuilder(RigidBodyType.FIXED);动态体:DYNAMIC
ground = world.add_body(
    r.RigidBodyBuilder(r.RigidBodyType.FIXED).translation((0.0, -0.5, 0.0)),
    colliders=[r.Collider.cuboid(50.0, 0.5, 50.0)],   # 形状工厂在 Collider 类上!
)

# 动态体 + collider 链式参数(实测可用)
ball = world.add_body(
    r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
    .translation((0.0, 10.0, 0.0))
    .linvel((1.0, 0.0, 0.0)),
    colliders=[r.Collider.ball(0.5).restitution(0.3).density(1.0)],
)
# add_body 返回 RigidBodyHandle(index, generation),不是 body 对象

# 读回状态:world.rigid_bodies[handle] -> RigidBody
body = world.rigid_bodies[ball]
body.translation   # Vec3
body.rotation      # Rotation3
body.linvel        # Vec3
body.angvel        # Vec3
body.is_sleeping   # bool
body.mass          # float

world.step()       # 推进一步;固定步长,默认 dt 见下
```

## 关键事实

- **形状工厂在 `Collider` 类上**:`Collider.cuboid(hx,hy,hz)`、`Collider.ball(r)`、
  `Collider.compound([(isometry, shape), ...])`、`Collider.round_cuboid(...)` 等,
  返回可链式设置 `density/friction/restitution/sensor/...` 的对象。
  `ColliderBuilder` 在运行时**没有**构造函数也没有形状工厂(存根是假的,别用)。
- **形状类**:`SharedShape.cuboid/ball/compound(...)` 也存在,但可直接用 `Collider.*` 工厂。
- **别用 `Voxels` 形状**(已知 panic bug,任务书已确认)。
- **重力默认 (0,0,0)**,必须在 `PhysicsWorld(...)` 构造时显式给。
- **dt**:`world.integration_parameters.dt`(默认 1/60);step() 是固定步长。
- **睡眠**:`body.activation` 有 `linear_threshold/angular_threshold/time_until_sleep/sleeping/is_active` 等字段;
  builder 侧有 `can_sleep(bool)`、`sleeping(bool)`、`linear_damping/angular_damping`。
- **snapshot/restore**:`blob = world.snapshot()`(bincode bytes);`world2 = r.PhysicsWorld.restore(blob)`
  是 **staticmethod,返回新 world**(不是原地还原)。restore 后 handle 语义不变(实测同 handle 可读)。
- **单刚体序列化**:`RigidBodyBuilder.to_bytes()/from_bytes()` 存在;但迁移实体最稳的做法是
  手动提取 `translation/rotation/linvel/angvel` 四元组,在新世界重建。
- **线程**:pyclass 全部 unsendable(禁止跨线程转移同一对象);`world.set_num_threads(n)` 控制
  单世界内并行。**每线程自建自用 world 是合法且推荐的模式**(step() 内部释放 GIL)。
- **性能参考**(本机实测,冒烟场景):空场景+地面+1球,120 步 = 2.23ms。
- `world.remove_body(handle)`、`world.add_collider(builder, parent=handle)`、`world.wake_up(handle)` 可用。
- **RigidBody 运行时方法(已验证)**:`apply_impulse(v, wake_up=True)`、`apply_torque_impulse(v)`、
  `apply_impulse_at_point(v, point)`、`sleep()`、`wake_up(strong=True)`。
- **RigidBody 没有运行时 setter**:`set_linvel/set_translation/set_rotation` 均不存在(已验证)。
  改变已有刚体状态只有两条路:① 删掉重建(`remove_body` + 新 builder);
  ② 整个世界 `snapshot/restore`。迁移/传送实验请按"删除+重建"设计。
- **`builder.rotation(v)` 只接受 3 元组缩放轴角向量**,不接受 `Rotation3` 对象(存根字面有误导)。
  从已有刚体读回姿态重建时用 `body.rotation.scaled_axis`。`Rotation3.to_matrix()` 返回
  行主序 9 元展平数组。
- `RigidBodyType.FIXED / DYNAMIC / KINEMATIC_POSITION_BASED / KINEMATIC_VELOCITY_BASED`。

## 接触对与接触冲量(REQ-009 任务 00 实测,2026-10-25)

- `world.narrow_phase.contact_pairs()` 是**方法**(存根未标注),返回 `ContactPair` 列表;
  `world.step()` 后即有效,无需手动 update。同类陷阱:`has_any_active_contact`、
  `is_dynamic`、`is_sleeping` 是**属性**不是方法(带括号会 TypeError)。
- **ContactPair**:`collider1/collider2`(ColliderHandle)、`manifolds`、
  `total_impulse()` / `total_impulse_magnitude()`(方法)、`has_any_active_contact`(属性)。
- **ContactManifold**:`local_n1/local_n2`(法向)、`points`、`subshape1/subshape2`。
- **接触点**:`impulse`(法向冲量标量,每接触点每步)、`tangent_impulse`、`dist`(负=刺穿)、
  `local_p1/local_p2`(各自 collider 局部系)、`contact_id`、`fid1/fid2`。
- **collider → body**:`world.colliders.get(pair.collider1).parent`(属性)→ `RigidBodyHandle`。
- **⚠ 缺陷(0.36.0 实测)**:凡是**复合 collider**(如生产口径的地形薄片复合体)参与的接触对,
  `point.impulse` 恒读 **0**(静置、撞击步都是;`total_impulse()` 同样不可信);
  primitive↔primitive(尤其动态↔动态)读数精确(1 kg 静置 = 0.1635 = mg·dt,逐位吻合)。
  `ActiveEvents.CONTACT_FORCE_EVENTS` 注册与否无影响。
- **测地形冲量的可行路**(REQ-009 §5 认可的 fallback):动量差分。
  竖直 `I = m·Δv_y + m·g·dt`(触地一步吸收时 ≈ m·v_impact);矢量式
  `I⃗ = m·Δv⃗ − m·g⃗·dt`。叠放场景"上方载荷"用底块↔第二块的接触对直读
  (动态↔动态,可靠),叠加即得地形对底块的冲量(1–8 层实测与 n·mg·dt 零误差)。
- **激活阈值可运行时改**:`world.rigid_bodies[h].activation.linear_threshold = 0.09`
  / `angular_threshold = 0.049`(默认 0.4 级别,`time_until_sleep` 默认 0.5 s)。
  builder 侧 `can_sleep(bool)` 可用。
- 复合体几何:`r.Collider.compound([(r.Isometry3.from_translation(x,y,z), r.SharedShape.cuboid(hx,hy,hz)), ...])`;
  `Isometry3` 不可下标访问(要分量请存构造前的元组)。
