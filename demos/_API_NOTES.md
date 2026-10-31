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

## 复合体与逐格冲量(REQ-010 任务 00 实测,2026-10-26)

- **逐格冲量的可靠做法(强烈推荐)**:一个刚体 + **每格一个 primitive collider**
  (`colliders=[Collider.cuboid(...).translation(off), ...]` 列表即多 collider),
  地形同理用**固定体 + 多 primitive 薄片**(不要 `Collider.compound`)。
  此时 dynamic↔static 的 `point.impulse` 精确:静置全格合计 = m·g·dt(实测 0.715149 vs
  0.715149,偏差 0.00%);撞击峰值拍合计 vs 动量差分偏差 3.1%。复合体质量 = 格数×0.729
  实测吻合(6/7/8/9 格 → 4.374/5.103/5.832/6.561);把格合并成长方体(密度 1)质量相同。
- **补充(对 2026-10-25 记录的修正)**:单个 `Collider.compound` 参与的接触对,本次静置
  实测读到了 0.119192(非 0)——"复合 collider 冲量恒 0"的缺陷**没有在此构型复现**,
  触发条件比原记录更窄(原记录为 16×16 chunk 级复合地形)。生产走 Rust 不受影响
  (REQ-009 撞击 spike 已证 Rust 侧复合体可读);Python 侧请一律用多 primitive 路线。
- **collider 运行时读取**:`world.colliders.handles()` 按创建序产出;`world.colliders.get(h)`
  有 `.translation`(= **世界**坐标,含 body 位姿)与 `.parent`(→ `RigidBodyHandle`)。
  collider→格映射最稳的做法:创建时刻用 `body_pos + R·offset` 逐格按世界位置匹配。
- `pair.total_impulse()` 返回 **Vec3**(存根未标注),取 `.norm()`;逐点求和用
  `sum(pt.impulse for m in pair.manifolds for pt in m.points)` 即可。
- `Rotation3.transform_vector(v)` 可直接把局部向量变换到世界系(与 `to_matrix()` 行主序、
  world = M·local 的约定一致,builder.rotation 的缩放轴角与 Rodrigues 公式吻合)。
  **`np.array(Vec3)` 不行**(退化成对象数组),要 `np.array(tuple(v), dtype=float)` 中转。
- 生产口径"薄片跨面(半厚 0.075)× 0.9 立方内缩 0.05"的初始重叠 = 0.025:不会把复合体
  弹开(水平速度 0.149 m/s);墙面摩擦使沿面下滑减速 ~25%(不粘死)。
- 生成复合体必须把原点放在**格中心**(合法网格位姿):偶数尺寸形状的原点规则
  (包围盒中心同距取小)天然落在半格偏置上,若再把原点生成在非格中心处,
  全体格的 snap 误差会被系统性抬高 0.5 格。

## 撞击冲量流 demo 实测(F00,2026-10-30)

- **collider→格 映射按创建序对齐**:`world.colliders.handles()` 按创建序产出,
  与 `add_body(colliders=[...])` 的列表顺序一一对应(同 parent 过滤后)。
  不要对每薄片做"全 collider 集最近邻匹配"——薄量级(数千 collider)下是
  O(N²) 纯 Python 循环,分钟级卡死;按序 zip 是零成本。
- **惯量张量数值反演**:`apply_torque_impulse((1,0,0))` → 读 angvel 再施加
  (-1,0,0) 抵消,三轴各一次得 inv(I) 的列(实测与平行轴定理逐位吻合,
  撤销后残差 ≤ 1e-6)。角速度会残留 ~1e-7 浮点噪声,无 setter 清不掉,
  下一步 step() 会覆盖,不用管。
- **apply_impulse_at_point 的"点"是世界坐标**,r = point − body.translation
  (原点),不是质心——本 demo 的复合体原点恰在质心(对称格集合 + 格心原点规则),
  两者一致;一般情形要用质心(rapier 内部按质心算力矩)。
- **m_eff 是沿 d̂ 的标量**:验证公式时接触点速度变化要取 d̂·Δv_p 投影,
  取全模长会把"偏心分量"也算进去(3 格条偏心 0.4 m 时差 8%)。
- **接触点世界坐标 = collider 世界位置 + local_p**(fixed 体、collider 无
  相对旋转时;local_n1 指向 pair 侧 1 的对面,即指向 collider2 的动态体一侧;
  collider1=地形时 local_n1 指向动态体)。REQ-009 的"沿法向退 0.1 m 取格"
  用 -n × 0.1(指向地形内部)。
- **contact 对的归零时序**:分离后下一拍 manifolds points 为 0;持续接触时
  每拍重新累计。"步进前快照速度、步进后读冲量"的管线口径可用
  (tb 的 linvel/angvel 用于勘误 2 的接近速度过滤)。
- **rapier 单步法向冲量 ≈ 0.6–0.8 × m·v_n**(软约束分摊;0.729 kg 40 m/s 横撞
  单步峰值 ≈ 17.5 N·s;REQ-009 spike 1 kg 竖直 20 m/s = 15.12)。估算"多大速度
  能撞断"时,实际速度要按峰值/m·v 的反比放大 ~1.2–1.6×。
- 性能坑:41×41 世界(≈5000 薄片)slab_terrain 本体 < 0.1 s;真正慢的是
  逐 collider 的 Python 级最近邻/距离计算,避免即可。1600 格石板单事件
  (BFS 1024 + 建网 + EK)Python 全流程 ≈ 5 ms。

## 薄片内缩 demo 实测(F00b,2026-10-31)

- **复用 F00 管线换地形口径的接法**:`momentum.Pipeline.__init__` 与
  `process_tick`(断格重建地形)都引用它自己模块级的 `slab_terrain` 名字;
  把 `momentum.slab_terrain` 替换为按 `w.slab_mode` 分派的包装即可双口径,
  不用改 momentum.py(demos/inset-slab/common.py 的做法)。impact-flow 的
  common.py 以私有名加载时,临时占住 `sys.modules["common"]` 再还原,
  否则 momentum 顶部 `from common import ...` 会命中错误的模块。
- **场景平移 Δy 的核对方法**(F00 impact-corner 错一格的教训):世界地表
  **空气层** y=−60 对应局部 y=0,即格 y=−60 → 局部 0、实体格 −58(门楣)→
  局部 2;spawn 的 y=−57.4 → 局部 2.6。每处平移逐格打印核对,不要心算。
- **接触对出现的时序**:rapier 只在 AABB 相交后才给 contact pair;高速投射物
  (15–20 m/s,每拍 0.25–0.33 m)接触的第一拍 pair 列表可能为空、冲量读数
  出现在下一拍(F00 也如此)。判"是否接触过"用刚体速度突变(post_vx 骤降)
  或位移异常,不要只看 pair 数。
- **退距归格扫描的要点**(rest.py §4):接触点世界坐标 = 薄片 collider 平移 +
  local_p;内缩后接触点本身落在实心格表面(跨面时在 −0.075 的薄片前缘),
  退 0.1 m 两种口径正确率相同(对齐撞 100%、齐平侧撞 6/8,错点是棱角斜交
  的边缘情况,与口径无关)。
- **rapier 睡眠会把"斜靠悬空"的复合体判静止**:impact-corner 断下的 7 字
  以 34° 斜插、最低角点与地面薄片接触(支撑冲量 0.3–0.4/拍)即 sleeping;
  验证"是否真的被支撑"要看接触对冲量,不要只看 is_sleeping。
