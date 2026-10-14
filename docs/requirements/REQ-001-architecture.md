# REQ-001:总体架构——Rapier 多世界分片并行 + 单世界激活

- 状态:**草案**(待用户确认;确认前不写正式代码)
- 提出日期:2026-10-14
- 关联 demo:`demos/shard-migration`(A)、`demos/voxel-collider`(B)、`demos/solidify-bridge`(C)、`demos/cross-shard-explosion`(D)
- 代码证据:`docs/research/ferrumc-v2-survey.md`(下称【调研】,file:line 证据在里面)

---

## 1. 项目定义

用 Rapier 物理引擎构建"物理版 Minecraft":体素方块世界 + 真刚体动力学(方块可被撞碎、
炸飞、凝固回落),目标可多人联机、未来可接强化学习训练。

**非目标(本期)**:网络协议实现、客户端、插件系统、动态重划分片、渲染。

## 2. 既定共识(已定死,不再辩论)

1. 模拟核 headless、确定性、固定 dt(1/60),状态只能通过命令改变;
2. 体素数组是唯一真相,世界锚定坐标,chunk 化;
3. 客户端/服务端只通过一条缝通信:命令进 → 快照/增量出(接口形状 = 网络形状);
4. tick = 逻辑时钟(TickCounter),回放 = 快照 + 命令日志;
5. 体素层格子锁定是定义,任意位姿的方块属于实体层;两层的桥 = "动态体睡觉 → snap 去旋转
   → 写数组 → 换静态 collider → 删刚体";
6. 静态体永远不会自己醒来,激活/转化全部是游戏政策层的命令。

## 3. 已验证事实(不必复测)

前期实测(任务书给出):
- 同机确定性:两个独立世界相同输入 200 步位置差 = 0;
- 快照:`snapshot()` = bincode 全家桶,600 球 547KB / 0.9ms;`restore` 是 staticmethod、
  返回**新 world**(不是原地还原);
- 线程模型:pyclass 全部 unsendable,但 `step()` 内部释放 GIL → **每线程自建自用 world
  的 actor 模式完全合法**,4 线程实测 2.5× 加速;
- 单世界内 `set_num_threads(n)` 岛级并行扩展弱(4 线程 1.9×,16 线程退化);
- 实体迁移物理层成本 ~5-6µs/体;难的是协议不是成本;
- 激活数字:25000 静态 collider + 演员步进 0.06ms;chunk 换装 = 下一帧 ~2ms BVH 增量更新;
  1 万动态体同醒 = 44 步/秒(**真正的性能悬崖**);
- 体素→碰撞:暴露面(6 邻居判据)→ 静态 cuboid;Rapier 原生 `voxels` 形状有 panic bug。

本次环境/demo 补充(见各 demo README):
- PyPI 绑定实际版本 **rapier3d 0.36.0**(任务书记载的 0.35.3 不存在);Rust 正式实现用
  rapier crate,`enhanced_determinism` feature 是否开启为开放问题(§9.10);
- **RigidBody 无运行时 setter**(无 `set_linvel/set_translation`):传送/迁移只能"删除+重建"
  或整世界 snapshot/restore——这从 API 层面锁死了迁移协议的形状;
- Demo A:tick 边界"删除+重建"迁移,接触态误差 ~5e-6 m、飞行态位级一致、成本 p50=10µs/体;
  **静态 collider 布局差异会造成 0.26m 轨迹分叉**(接缝效应);
- Demo B:9488 静态 collider + 100 动态球,step 仅 ~0.2ms,三种组织策略几乎无差;
  **每 chunk 一个静态体 + compound** 结构最优,chunk 换装 0.3/1.4ms;贪心合并压缩 4.8×
  但收益 ~7%(可选优化,必须 chunk 界内进行);
- Demo D:爆炸"共享输入 + 本地状态独立计算"与单世界结果**位级一致**(含贴边界实体),
  重复运行也位级一致。

## 4. 三层架构

```
┌────────────────────────────────────────────────────────────┐
│ 第 1 层 分片并行:N 个 PhysicsWorld 分片,专属线程,无锁并行   │
│   ShardId(8×8 chunk)· 所有权表 · TickCoordinator · 屏障     │
├────────────────────────────────────────────────────────────┤
│ 第 2 层 跨分片交互:只在 tick 边界,两种动作                  │
│   消息(信封,apply_at = N+1)· 实体迁移(删除+重建)          │
│   跨片事件 = 广播共享参数,各方本地确定性求值(demo D)         │
├────────────────────────────────────────────────────────────┤
│ 第 3 层 单世界激活:体素数组 = 真相,collider/动态体 = 缓存    │
│   三档半径:chunk 加载 / collider 插入 / 动态体清醒           │
│   演员(玩家+怪+掉落物)驱动,滞回 + 预算;凝固桥              │
└────────────────────────────────────────────────────────────┘
```

### 4.1 第 1 层:分片并行

**职责**:把世界切成 8×8 chunk(128m×128m,chunk=16m,全高)的 2D 网格分片;每个分片一个
独立 `PhysicsWorld` + 自己的体素 chunk 数据 + 自己的实体集合,独占所有权,绑定专属 worker
线程,每 tick 独立 `step()`。

**数据所有权**(借 ADR-0002 与 v2 `ShardOwnershipMap`,【调研】§3):
- chunk 与实体**值语义**归属分片:一个 chunk 同一 tick 只属于一个分片(claim/release,
  重复 claim 报错);
- 实体同理:实体 ID 全局唯一(来源命名空间),同一 tick 至多被一个分片持有;
- 跨线程不共享内存:分片状态在 worker 间**所有权转移**(v2 shadow 模式:每 tick 把分片
  batch 转移给固定 worker 组,scheduler 等所有 batch 返回才进下一 tick)。

**接口**(对上):`register/despawn(shard)`、`submit(shard_id, SimCommand)`、
每 tick 结束后读取各分片 `Events`(迁移/消息/激活请求)。
**接口**(对下):分片 = 纯函数式 tick:`fn tick(shard: &mut Shard, inbox: Commands, dt)`。

**失败模式**:
- 慢分片拖慢全局:屏障语义下全局 TPS = 最慢分片(与 v2 "no catch-up ticks" 一致,超载
  降速不追赶);缓解:分片内预算(§4.3)+ 超大分片拆分(开放问题 §9.2);
- 分片 panic:fail-stop(借 ADR-0008 信任模型:不热修复,崩溃即停,靠回放恢复);
- worker 挂掉:capacity-one 通道 + 每 tick 汇合,能立刻发现(线程 join 超时)。

### 4.2 第 2 层:跨分片交互

分片默认互不感知。一切跨分片效果只允许两种动作,且只发生在 tick 边界
(tick N 完全结束、tick N+1 尚未开始):

**(a)消息(信封模型,照搬 v2 `CrossShardEnvelope`,【调研】§4)**
- 源分片 tick N 产出 `Intent{destination, payload}` → 调度器盖章为
  `Envelope{produced_at=N, apply_at=N+1, source, destination, sequence, payload}`;
- 调度器在 tick N+1 开始时把信封作为目的分片的**输入前缀**投递;
- 队列有界(默认 1024)、准入不驱逐、确定性排序键 `(destination, source, sequence)`;
  满则拒绝并反压(同 v2 拒绝原因枚举)。

**(b)实体迁移(自研,v2 完全没有,demo A 给出协议)**
- 触发:实体位置离开所属分片本体区(每 tick 末检查);
- 时序:tick N 结束 → 源分片提取状态 {translation, rotation, linvel, angvel, 实体 ID,
  附加数据} → `remove` → 调度器转交 → 目的分片 tick N+1 **前缀阶段** `add` → 再 step;
- 所有权:借 v2 所有权表,迁移 = 原 claim release + 新 claim,同一 tick 不允许双持有;
- 成本实测:10-20µs/体(demo A),千体级迁移 ~10-20ms,用迁移预算分摊到多 tick。

**跨片事件(爆炸/冲击波等范围效果,不移动实体,demo D 协议)**
- 事件 = `{类型, 逻辑 tick, 共享参数}`;调度器按 canonical 序广播给所有扩张区覆盖事件
  影响范围的分片;
- 各分片在自己的该逻辑 tick 用**本地实体状态**独立求值(demo D 证明与单世界位级一致);
- 事件顺序:同一 tick 多事件按 `(类型, 来源, 序号)` 排序(开放问题 §9.6 细化)。

**几何保证(扩张区域不变量,任务书原文)**:每个分片的有效管辖 = 本体区 ∪ 扩张区
(本体向外扩 `INTERACTION_MARGIN`),扩张区重叠、本体互斥。分片加载并模拟本体+扩张区内的
一切,因此任何跨片效果要么是"实体迁移"(实体出了本体),要么是"双方各自确定性计算"
(效果波及邻片本体),永远不需要跨世界的物理查询。
⚠️ 含糊点:任务书定义 `MARGIN > 弹射物射程 + 爆炸半径 + 最大实体尺寸`,但 demo A/D 表明
实体跨界即迁移、事件靠广播,物理效果从不"穿越"边界——MARGIN 的必要尺寸可能只需
`爆炸半径 + 最大实体尺寸 + 余量`。**列为开放问题 §9.1,由 REQ 确认敲定**。

**失败模式**:信封满(反压,拒绝并通知源);目的分片不存在(信封拒绝,枚举错误,同 v2);
事件重复投递(信封 sequence 去重,恰好一次);实体在迁移瞬间又被命令引用(命令只对
"上一 tick 末的所有权"有效,调度器负责改路由一 tick)。

### 4.3 第 3 层:单世界激活

分片内部不全量模拟。真相 = 体素数组(世界锚定、chunk 化);物理世界里的 collider 只是
派生缓存(demo B:每 chunk 静态体 + compound,`f(体素数组)` 纯函数生成,demo A 证明
非纯函数的 collider 布局差异会变成可见的物理分叉)。

**三档独立半径控制**(激活中心 = 演员集合:玩家、怪物、掉落物):

| 档 | 内容 | 半径(加载/卸载) | 成本依据 |
| --- | --- | --- | --- |
| ① chunk 数据 | 体素数据进内存 | 32m / 48m | 便宜,慷慨 |
| ② collider 插入 | chunk 静态体+compound | 32m / 48m | 换装 0.3-1.4ms/chunk(demo B),慷慨但受预算 |
| ③ 动态体清醒 | 真实模拟 | 24m / 32m | **吝啬**:1 万同醒 = 44 步/秒(悬崖) |

- 滞回:加载 32m、卸载 48m(+50%),加 3 秒冷却,防边界抖动;
- 预算:每 tick 换装预算(如 ≤2 chunk/tick),demo B 实测单 chunk 装载 ~0.3ms(Rust 侧),
  预算主要防激活风暴(传送/爆炸瞬间);
- 动态体清醒上限:每分片预算 `MAX_AWAKE_DYNAMIC`(默认 512,开放问题 §9.4);超预算按
  "离最近演员距离"排序,远的强制睡眠(kinematic 冻结或暂停语义:开放问题——被冻结实体
  对命令的响应语义需定义);
- **凝固桥**(共识 5,demo C):动态体线速度 <0.1 且角速度 <0.05 连续 20 tick(睡眠兜底)
  → snap 去旋转(24 轴对齐朝向取最近,**任何触发策略都不能省略**——睡眠角阈值 0.5 rad/s
  太松,睡着的体照样可能带着 yaw 偏角)→ 位置 round 到体素中心(格点整数、中心 = 格+0.5)
  → 写体素数组 → 该 chunk compound 重建 → 删刚体。实测桥操作 0.02-0.07ms/体、snap 后零穿深、
  后续 600 tick 稳定;**snap 误差由"物理体停在哪"决定而非触发策略**(demo C:三策略误差
  逐位相同);接口按"每帧批量收集 + 批量 remove/add"设计。纯政策层命令,静态体永不自动
  醒来(共识 6)。

**失败模式**:演员跨分片瞬移 → 源分片卸载与目标分片加载的接缝(两分片扩张区重叠保证
过渡期物理不空窗);激活风暴(预算强制摊平);collider 缓存与体素真相不一致(只允许
"重建整个 chunk compound"这一种更新路径,禁止细粒度增删)。

### 4.4 横切:tick 纪律(借 v2 TickCoordinator + driver,【调研】§2/§5)

- 单一全局 TickCounter,`advance()` 每拍 +1,不查墙钟、不追赶(lag 只上报);
- 每个 tick:① 调度器收命令/信封/事件,盖章分发 → ② 全 worker 并行 step 各自分片
  (分片所有权转移,capacity-one 通道)→ ③ **屏障**:所有 worker 返回 → ④ 处理迁移/
  消息/激活请求(写入下一 tick 的输入前缀)→ ⑤ 下一拍;
- 驱动 loop 借 v2 driver:`tokio::select! biased`(关机 > 到期 tick > 命令)、
  `MissedTickBehavior::Skip`、SimCommand 有界通道、收件箱满断开(权威输入);
- 模拟跑专用 OS 线程池(非 tokio task,ADR-0003);网络未来走独立线程(v1 模式)。

## 5. 轮廓含糊点与决议

| # | 含糊点 | 决议/状态 | 依据 |
| --- | --- | --- | --- |
| 1 | 分片怎么划分与迁移 | 8×8 chunk 固定网格,坐标 `ShardPos<<3`;**运行时不重划**(v1 决策);动态重划列为开放问题 | v2 ShardPartitioner 无状态静态映射【调研】§3 |
| 2 | 玩家进城时分片生命周期 | 懒创建(第一个实体/玩家进入时);**despawn 政策待定**(v2 无销毁 API,只有 Draining→Stopped 终态)——建议空片 + 无票 30s 后 Draining | 【调研】§5;开放问题 §9.3 |
| 3 | 空分片是否 tick | v2 空片照常 tick;我们改为:无 chunk 票据且无实体的分片跳过 step(仍占位所有权) | 省算力;风险:改变"每 runnable shard 每 tick 恰好访问一次"不变量,需测试覆盖 |
| 4 | 跨片爆炸确定性 | **已解决**:广播共享参数+本地求值,位级一致(demo D);多事件 canonical 排序 | demo D |
| 5 | tick 屏障与负载均衡 | 采纳 v2:每 tick 全员汇合屏障 + 分片所有权转移;负载均衡 v1 为静态轮转,动态均衡为开放问题 | 【调研】§5 |
| 6 | 实体迁移协议时序 | **已解决**:tick N 末提取 4 向量+ID → 删除 → 信封 → tick N+1 前缀重建;所有权表 claim/release | demo A + v2 信封模式 |
| 7 | 交互余量 MARGIN | 任务书公式偏保守(弹射物射程项可疑);建议 `MARGIN ≥ 爆炸半径 + 最大实体尺寸 + 安全余量`,数值待定 | §4.2 分析;开放问题 §9.1 |
| 8 | 激活政策归属 | 政策层(哪些实体醒、何时凝固)全部是显式命令,物理核不自主决策 | 共识 6 |
| 9 | 动态体超预算的语义 | 冻结(kinematic)还是整分片降速?**待定** | 开放问题 §9.4 |
| 10 | RigidBody 无 setter 的传送 | 一切位姿变更 = 删除+重建(或 chunk 级 snapshot/restore);写进编码规范 | 绑定实测 |

## 6. ferrumc v2 对照表(我们要造的 vs 它已解决的)

| 我们的架构件 | v2 状态 | 证据(见【调研】) | 我们要补 |
| --- | --- | --- | --- |
| 8×8 chunk 分片 + 所有权 | ✅ 已解决 | shard_pos.rs L9/L44;ownership.rs L19/L256-331 | 无(直接照搬) |
| 分片独占 + 无锁并行 tick | ✅ 机制完整 | ADR-0002;scheduler.rs L15-21(屏障+所有权转移) | 把 shadow 路径变成权威路径(它自己没投产) |
| tick 纪律(全局 tick、Skip、biased select、有界命令、满则断开) | ✅ 已解决 | driver.rs L719-784/L696-698/L212/L893-939 | 无 |
| 跨片消息(信封/排序/有界/恰好下一拍) | ⚠️ 实现但未接线 | cross_shard.rs L8-11/L69/L84-102/L245-371 | 接线 + 把 payload 扩展为实体快照/事件 |
| **跨片实体迁移** | ❌ 未实现 | migration/migrate/handoff 全库 0 命中 | **自研**(demo A 协议) |
| 分片生命周期(懒创建/销毁) | ⚠️ 部分 | ownership.rs L333-407 状态机;无 despawn | 懒创建 + despawn 政策 |
| 空片跳过 | ❌ 无(照常 tick) | INVARIANTS L54 | 自研(可选优化) |
| 动态负载均衡 | ❌ 无(静态轮转) | scheduler.rs L124-127 | 自研(可后置) |
| 每玩家 chunk 流(差分/切比雪夫/16+16/票据/持久化屏障) | ✅ 已解决 | chunk_stream.rs L33/L41/L379-415;ticket.rs L58-67;driver.rs L1396-1443 | 网络 REQ 阶段照搬 |
| 多周期调度(keepalive/world_sync 相位偏移) | ✅ v1 已解决 | game_loop.rs L243-329/L378-387 | 网络 REQ 阶段参考 |
| 体素→collider 派生/激活三档/凝固桥 | ❌ 无(v1/v2 均无物理) | —— | **自研**(demo B/C) |

## 7. Demo 实验汇总

| Demo | 问题 | 关键数字 | 结论 |
| --- | --- | --- | --- |
| A 迁移(shard-migration) | 迁移协议可行性 | 接触态误差 5e-6 m;飞行态 0;10-20µs/体;接缝分叉 0.26m | 迁移无损;collider 派生必须纯函数 |
| B collider(voxel-collider) | 体素→碰撞组织 | 9488 静态+100 球 step 0.2ms;compound/chunk 最优;换装 0.3/1.4ms | 静态慷慨;chunk 级重建;合并是可选优化 |
| C 凝固桥(solidify-bridge) | 体素↔实体桥 | 桥操作 0.02-0.07ms/体;snap 后零穿深;600 tick 验证稳定;三策略 snap 误差逐位相同(383mm+27.4°) | 桥可行且便宜;推荐 P2(0.1/0.05×20)+睡眠兜底;去旋转 snap 不可省略 |
| D 跨片爆炸(cross-shard-explosion) | 跨片事件等价性 | 10 球(含贴边界)终点误差 **0.000e+00**;重复运行 0 | 共享输入+本地求值 ≡ 单世界 |

## 8. 可开工模块清单(确认后按此立 crate)

| 模块 | 由谁改造 | 职责 | 关键接口 | 验收要点(对应 §10) |
| --- | --- | --- | --- | --- |
| `crates/voxel`(现 world 改造) | 改 | 体素数组唯一真相:chunk、世界锚定坐标、方块注册表;**不含物理类型** | `Chunk::get/set`、`World::chunk(pos)`、序列化 | 存取正确、可序列化 |
| `crates/physics-sim`(现 physics 改造) | 改 | 单分片物理:`PhysicsWorld` 封装;体素→compound 纯函数;三档激活数据结构;凝固桥;迁移状态提取/重建 | `ShardSim::new(region)`、`step(dt)`、`apply(cmd)`、`take_events()`、`snapshot/restore` | AC1/AC2/AC4/AC7 |
| `crates/shard`(新) | 新建 | ShardId/ShardPos、所有权表、调度器(线程池+capacity-one+屏障)、TickCoordinator、懒创建/Draining | `register/despawn`、`submit(cmd)`、`run_until(tick)` | AC5/AC6 |
| `crates/messaging`(新) | 新建 | SimCommand、信封(盖章/排序/有界队列)、跨片事件广播 | `Envelope::stamp()`、`Queue::admit()` | 信封恰好一次、确定性序 |
| `crates/sim-api`(新) | 新建 | headless 缝:命令进/快照+增量出(网络形状);回放器(快照+命令日志) | `SimSession::send(cmd)`、`SimSession::snapshot()` | AC3 + 回放一致 |
| `crates/app`(现 app) | 改 | 装配 + 驱动 loop(biased select / Skip / 有界通道) | 二进制入口 | tick 纪律 |

依赖方向:`app → sim-api → {shard, messaging, physics-sim} → voxel`;`physics-sim` 依赖
rapier(Rust 版)与 `voxel`。

## 9. 开放问题清单(需用户决策或后续 REQ 确认)

1. **MARGIN 数值与公式**:弹射物射程项是否保留?(建议改为爆炸半径+最大实体尺寸+安全量,
   默认 16m 起步,配置化)
2. **动态重划分片/负载均衡**:固定网格 + 静态 worker 轮转起步;超大分片拆分策略后置,触发
   阈值(step 耗时 > 5ms 持续 N tick?)需定;
3. **分片 despawn 政策**:空 + 无票据 30s → Draining?(v2 无先例)
4. **动态体超预算语义**:冻结(kinematic)vs 整分片降速;`MAX_AWAKE_DYNAMIC` 数值(建议 512);
5. **凝固桥参数**:已定 P2(线速度 0.1 / 角速度 0.05 连续 20 tick)+ 睡眠兜底(demo C);
   剩余:斜坡/堆叠慢爬行场景未覆盖,定稿前补一组斜坡 demo;误差超阈值(>半格或 >45°)
   升级为等睡眠的路径需实现;
6. **多事件 canonical 排序键**:建议 `(类型, 来源分片, 序号)`,需定全序规则;
7. **实体 ID 全局命名空间**:来源分片前缀 + 本地序号?迁移后 ID 不变;
8. **持久化后端**:体素 chunk/实体/快照的存储(v2 redb "trait 边界+单写者+批量 flush" 模式
   可借,后端自定);
9. **网络与每玩家 chunk 流**:独立 REQ(v2 chunk_stream 可整体照搬);
10. **rapier 版本与确定性 feature**:Rust 侧 rapier 版本对齐 0.36;`enhanced_determinism`
    是否开启(跨机器确定性 vs 性能);
11. **回放存储格式**:快照频率 + 命令日志切割;
12. **冻结实体的命令语义**:被预算冻结的实体收到爆炸/玩家交互时如何唤醒。

## 10. 架构级验收标准(全部可测)

- **AC1 确定性**:两个同构分片,相同输入 K=200 tick,任意实体位置差 = 0(浮点级);
- **AC2 迁移无损**:实体跨分片迁移前后轨迹与不迁移对照误差 < 1e-4 m(接触态);
- **AC3 跨片事件等价**:任意爆炸场景,分片各自求值的合并结果与单世界参考位级一致;
- **AC4 激活不变量**:任意时刻任意 chunk 至多被一个分片持有;扩张区覆盖完整(无物理空窗);
  换装满足预算;
- **AC5 无锁**:数据流为所有权单向转移,同一分片同一 tick 至多被一个 worker 进入;
- **AC6 tick 纪律**:全局 tick 单调 +1、无追赶;分片滞后时全局降速而非补帧;
- **AC7 桥一致**:凝固后体素数组 == collider 布局 == 视觉方块,三者一致;桥后物理稳定
  (无穿透/爆炸);
- **AC8 回放**:任意快照 + 命令日志重放,终态与原运行位级一致。
