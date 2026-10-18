# 代码布局蓝图:crate 划分、依赖规则与工程规范

- 状态:草案(随 REQ-001 已确认的架构落地,细化其 §8 模块清单)
- 日期:2026-10-15
- 上游文档:[REQ-001 总体架构](../requirements/REQ-001-architecture.md)(下称 REQ-001,其 §9 决议本文直接引用)
- 读者:未来在此仓库写代码的人(包括 AI 协作者)。**本文是"代码放哪、怎么长"的唯一裁判。**

---

## 0. 设计原则(软件工程思想 → 本项目具体化)

| 原则 | 本项目的具体形态 |
| --- | --- |
| **确定性核 / 副作用边缘**(六边形架构) | 模拟核(types/voxel/messaging/physics-sim/shard)纯内存、无 IO、无时钟、无随机,状态只因盖章输入而变;一切 IO(磁盘、未来网络、终端)在边缘(storage/app) |
| **依赖单向无环** | 五层依赖图(§2),每条边有明确理由;新增依赖必须能说出理由并保持无环 |
| **单一职责** | 每个 crate 一句话职责(§2 表),说不清就拆或并 |
| **防腐层(ACL)** | `rapier` 类型**不得泄漏出 physics-sim**;存储格式不得泄漏出 storage;网络协议形状只出现在 sim-api。引擎/后端可换,核不绑死 |
| **类型单一事实源** | 所有 ID/坐标/tick/迁移载荷定义在 types,其他 crate 只 re-export,禁止重复定义 |
| **最小可见性** | 默认 `pub(crate)`;跨 crate 只暴露构造函数、tick 接口与查询接口;内部结构体不 pub |
| **配置显式注入** | 核 crate 不读文件/环境变量,只接受 `XxxConfig` 结构体(§6);默认值集中在 app 层 |
| **fail-stop** | 一致性违规 = panic 即停(REQ-001 §4.1);可预期失败 = 类型化错误(§4) |

## 1. 两个关键决策的论证

**(a) 为什么加 `types` crate(REQ-001 §8 没有)**:信封的目的地是 `ShardId`,但 shard 又依赖 messaging(收命令)——若 ShardId 定义在 shard 则成环。所有跨 crate 传的"地址与载荷"(EntityId/ShardId/ShardPos/BlockPos/ChunkPos/Tick/EntityState/BlockId)下沉到 `types`,依赖图天然无环。ferrumc 同款做法(ferrumc-math)。

**(b) 为什么加 `storage` crate 且不让 shard 依赖它**:REQ-001 §9.8 决议持久化 = "trait 收口 + 单写 worker + 批量 flush;后端 redb"。落盘属副作用,按原则 1 只能在边缘;shard(despawn 决策)与 sim-api(checkpoint)只面向 trait,redb 实现与单写 worker 在 storage 内,由 app 装配。这样模拟核保持"纯函数式 tick",v2 的"卸载前持久化屏障"在驱动层复刻(【调研】§6)。

## 2. Crate 总览与依赖图

```
            ┌──────────────────────────────────────────────┐
   L4 边缘  │ app        装配/驱动/配置/未来网络/度量        │
            └──────┬────────────────────────┬───────────────┘
            ┌──────▼──────┐          ┌──────▼──────────────┐
   L3 缝    │ sim-api     │◀──仅用──▶│ storage  trait+实现 │
            │ 命令进/快照出/回放      │ 落盘唯一入口         │
            └──────┬──────┘          └──────┬──────────────┘
            ┌──────▼────────────────────────┤
   L2 编排  │ shard   调度器/所有权/tick/生命周期(纯内存,唯一允许线程处)│
            └──────┬──────────────┬─────────┘
            ┌──────▼──────────┐ ┌─▼────────────────────────┐
   L1 领域  │ physics-sim     │ │ messaging  命令/事件/信封 │
            │ 单分片物理(rapier)│ └─┬────────────────────────┘
            └──────┬──────────┘    │
            ┌──────▼───────────────▼──┐
   L0 真相  │ voxel(体素数组)  types(ID/坐标/tick/载荷,全 crate 地基)│
            └─────────────────────────┘
```

| Crate | 层 | 一句话职责 | 依赖 | 禁止 |
| --- | --- | --- | --- | --- |
| `types` | L0 | 跨 crate 传的标识、坐标、tick、迁移载荷、glam re-export | glam | 任何业务逻辑 |
| `voxel` | L0 | 体素数组唯一真相:chunk 存储、方块注册表、批量编辑、脏标记 | types | 物理概念、渲染概念 |
| `messaging` | L1 | SimCommand、跨片事件、信封(盖章/五元组排序/有界队列) | types | IO、rapier、voxel 内部结构 |
| `physics-sim` | L1 | 单分片确定性物理:rapier 封装、体素→collider 纯函数、凝固桥、迁移提取/重建、三档激活、tick 末迁移检测 | types, voxel, messaging, rapier | 线程、IO、时钟 |
| `shard` | L2 | ShardId 所有权表、worker 池(capacity-one+屏障)、TickCoordinator、生命周期(懒创建/Draining/30s despawn)、命令路由与反压 | types, messaging, physics-sim | IO、时钟(逻辑 tick 不看墙钟) |
| `storage` | L3 | ChunkStore/CheckpointStore trait、单写 worker、redb 与内存实现 | types, voxel | 依赖任何模拟 crate |
| `sim-api` | L3 | headless 缝:会话(命令进/快照出)、全局快照、回放器(checkpoint+外部命令日志) | types, messaging, shard, storage | rapier、网络实现 |
| `app` | L4 | 配置装配、驱动 loop(biased select/Skip/有界通道)、度量告警、(未来)网络与 CLI | 全部 | 业务规则 |

**依赖规则**:只允许高层→低层、同层禁止互依(`sim-api` 与 `storage` 互不相依,由 app 同时装配);`app` 是唯一允许"认识所有人"的地方。评审红线:PR 里出现反向依赖或跨层穿透(app 直摸 physics-sim 内部、messaging 出现 rapier 类型)一律打回。

## 3. 各 crate 内部模块(文件级图纸)

> 签名是**形状示意**(定接口不定实现);开工时先写空骨架 + doc + 测试。

### types
```
src/
├── lib.rs        //! re-export 一切;glam 转发
├── ids.rs        // EntityId(u64=(origin_shard<<32|local_seq),§9.7)、ShardPos{x,z}、
│                 // ShardId{world,dim,pos}、WorldId、DimensionId、BlockId(u16)
├── coords.rs     // BlockPos/ChunkPos 及运算(chunk_of/shard_min_corner<<3 等)
├── tick.rs       // Tick(u64) 单调
└── state.rs      // EntityState{translation,rotation,linvel,angvel: glam}
                  // (迁移载荷四向量,demo A 协议)
```

### voxel
```
src/
├── lib.rs
├── block.rs      // BlockRegistry trait(名称↔BlockId、属性);内置注册表(测试用)
├── chunk.rs      // Chunk:扁平数组起步,palette 压缩后置;get/set;序列化 trait
├── world.rs      // VoxelWorld:ChunkPos→Chunk 的内存 map(BTreeMap,确定性迭代)
└── edit.rs       // VoxelEdit 批量编辑;返回脏 ChunkPos 集(collider 重建的输入)
```

### messaging
```
src/
├── lib.rs
├── command.rs    // SimCommand(SetBlock/SpawnActor/Detonate/Teleport/...);
│                 // StampedInput{apply_at, payload}(统一输入模型,REQ-001 §4.2)
├── event.rs      // CrossEvent{Detonate{center,power,radius},...} + 共享参数类型
├── envelope.rs   // Intent→调度器盖章→Envelope{produced_at,apply_at≡+1,source,
│                 // destination,sequence,payload};有界队列(默认 1024,不驱逐,
│                 // 拒绝原因枚举)——照 v2 cross_shard.rs 移植
└── ordering.rs   // canonical 五元组 (apply_at,destination,类型序号,source,
                  // sequence) 字典序比较(§9.6;类型序号=枚举声明序)
```

### physics-sim
```
src/
├── lib.rs        // pub struct ShardSim —— 对外唯一入口
├── conv.rs       // glam↔rapier 转换(防腐层唯一出口)
├── collider_build.rs // 体素→暴露面→chunk compound 纯函数(demo B;含贪心合并,
│                     // chunk 界内);输入只有 voxel 数据
├── sim.rs        // ShardSim::new(cfg, region); apply_prefix(&[StampedInput]);
│                 // step(dt); take_outbox()→Outbox{migrations,events,dirty_chunks,
│                 // solidified};snapshot()->Vec<u8>;restore(bytes)->Self
├── activation.rs // 三档半径/滞回/预算的状态机(32/48、32/48、24/32m,3s 冷却,
│                 // MAX_AWAKE_DYNAMIC=512,冻结+距离淘汰,§9.4/§9.12)
├── migration.rs  // tick 末按实体中心检测越界;提取 EntityState;删除(demo A)
├── solidify.rs   // 凝固桥:P2 触发+睡眠兜底+阈值升级;24 朝向 snap;写体素+
│                 // chunk compound 重建+删刚体(demo C);批量接口
└── explosion.rs  // 跨片事件本地求值(纯函数,demo D 冲量公式)
```

### shard
```
src/
├── lib.rs
├── ownership.rs  // OwnershipMap:claim/release/owner(BTreeMap;重复 claim 报错)
│                 // ——照 v2 ownership.rs
├── lifecycle.rs  // Created→Active→Draining→Stopped;懒创建;空片+无票 30s despawn(§9.3)
├── scheduler.rs  // 固定 worker 池(OS 线程,capacity-one 通道,每 tick 全员汇合屏障,
│                 // 分片所有权转移;空片跳过但收输入必恢复,§5 行 3)
├── coordinator.rs// TickCoordinator:advance() 单调+1,不看墙钟,lag 只上报
└── router.rs     // 命令路由:有界收件箱、满则断开权威输入(照 v2 enforce_input_)
                  // delivery;信封盖章与分发(tick N+1 输入前缀)
```

### storage
```
src/
├── lib.rs
├── traits.rs     // ChunkStore(put_batch/get)、CheckpointStore(append/read);
│                 // 一切接口 Async 或返回 pending-ack(供驱动层做卸载屏障)
├── mem.rs        // 内存实现(测试/回放)
├── redb.rs       // redb 实现(key=(world,dim,chunk_pos),§9.8)
└── worker.rs     // 单写 worker:批量缓冲、显式 flush→oneshot ack(v2 持久化屏障)
```

### sim-api
```
src/
├── lib.rs
├── session.rs    // Session:send(SimCommand)、poll_outputs()(快照/增量);
│                 // 接口形状=网络形状(共识 3),未来协议直接映射
├── snapshot.rs   // GlobalSnapshot{tick, ownership, per_shard: Map<ShardId,Vec<u8>>}
│                 // (分片快照为不透明 bytes,rapier 格式不出 physics-sim)
├── log.rs        // 外部命令日志:按 tick 分段(bincode);信封/事件/迁移不落日志
└── replay.rs     // restore checkpoint + 重放命令日志;终态位级一致(AC8)
```

### app
```
src/
├── main.rs       // 装配:Config→各 crate→驱动 loop
├── config.rs     // 全部参数(§6),toml 可覆写;唯一默认值来源
├── driver.rs     // biased select(关机>tick>命令)、MissedTickBehavior::Skip、
│                 // tick 末端:收 outbox→盖章分发→持久化屏障→下一拍
└── telemetry.rs  // 分片 step P95>5ms×60tick 告警(§9.2);只度量不决策
```

## 4. 横切规范

**错误处理**
- types/messaging:纯数据,无 Result;
- voxel/physics-sim/shard:可预期错误 = 类型化 enum(thiserror);**一致性违规 = panic**(fail-stop):所有权重复 claim、快照恢复不匹配、越权 tick 等;
- storage/app:IO 错误 Result 逐层上抛,app 决定日志/退出;
- 禁止 `unwrap/expect` 于运行时输入;测试代码不限。

**确定性纪律**(核五 crate 强制)
- 禁:墙钟(`Instant/SystemTime`)、随机数、环境变量、文件/网络 IO、线程(shard worker 池是唯一例外);
- 禁:无序迭代影响输出 —— 容器一律 `BTreeMap/BTreeSet` 或先收集再按 canonical 序排序;
- 落地手段:clippy.toml `disallowed-methods` 列入 `std::time::*`、`rand::*`、`HashMap::iter`(核 crate 各自 `#![cfg]` 生效)+ CI 一条 grep 断言;
- 每次改核逻辑,AC1 确定性测试必须过(§5)。

**编码与格式**
- rustfmt 默认配置(不引争议项);clippy `all -D warnings`;`unsafe_code = forbid`(workspace 已设);
- rust-toolchain.toml pin stable(当前 1.98),升级工具链单独提交;
- rapier 版本 pin 到 minor(`0.36`,§9.10),升级走专门 REQ + AC1 回归;
- 命名:crate/模块单数名词小写;类型 UpperCamel;常量与配置字段全小写下划线;物理量带单位语义(`max_awake_dynamic`、`margin_meters`)。

**测试目录**
- 单测:`<crate>/src/*.rs` 内 `#[cfg(test)]`(纯逻辑)+ `<crate>/tests/*.rs`(crate 级行为);
- 等价性/集成套件:仓库根 `tests/`(workspace 级,跨 crate 场景)。

## 5. 测试策略与 AC 映射

| AC(REQ-001 §10) | 测试位置 | 形态 |
| --- | --- | --- |
| AC1 确定性(同输入位级一致) | 根 `tests/determinism.rs` + 各核 crate 冒烟 | 两实例同输入跑 200+ tick,逐实体断言相等 |
| AC2 迁移无损(<1e-4m) | 根 `tests/migration.rs`(demo A 移植) | 接触态滚动穿越 vs 单世界对照 |
| AC3 事件等价(位级) | 根 `tests/explosion_equivalence.rs`(demo D 移植) | 分片独立求值 vs 单世界 |
| AC4 激活不变量 | shard::tests + 根 `tests/activation.rs` | 唯一持有、扩张覆盖、预算上限 |
| AC5 无锁(所有权转移) | shard 单测 | 同分片同 tick 至多一 worker(状态断言) |
| AC6 tick 纪律 | shard::coordinator 单测 | 单调+1、无追赶、滞后上报 |
| AC7 桥一致 | physics-sim::solidify 单测(demo C 移植) | 体素=collider=稳定;零穿深 |
| AC8 回放位级一致 | sim-api::replay 集成 | checkpoint+命令日志→终态相等 |

demo → Rust 移植对应表:demos/shard-migration→AC2 测试;voxel-collider→`collider_build` 单测;solidify-bridge→AC7;cross-shard-explosion→AC3。**demo 结论进测试,demo 代码不直接翻译**(Python 实现仅作参照)。

## 6. 配置参数单一来源(REQ-001 §9 决议数值)

全部定义在 `app::config`(结构体 + Default),toml 可覆写,构造时按子结构注入各核 crate:

| 参数 | 默认 | 决议 |
| --- | --- | --- |
| `margin_meters` | 16 | §9.1 |
| `activation.load/unload_radius` | 32/48(chunk、collider 两档同参) | §4.3 |
| `activation.awake_radius` | 24/32 | §4.3 |
| `activation.cooldown_secs` | 3 | §4.3 |
| `max_awake_dynamic` | 512 | §9.4 |
| `swap_budget_chunks_per_tick` | 2 | §4.3 |
| `despawn_after_secs` | 30 | §9.3 |
| `envelope_queue_capacity` | 1024 | §4.2 |
| `checkpoint_interval_ticks` | 600 | §9.11 |
| `shard_alert_ms`(仅告警) | 5(P95,持续 60 tick) | §9.2 |
| `listen_addr` | "127.0.0.1:25565" | REQ-003 §4.2 |
| `dt` | 1/60 | 共识 1 |

核 crate 拿到的只是 `ShardSimConfig`、`SchedulerConfig` 等子结构(字段全显式,无全局单例)→ 测试可构造任意组合。

## 7. 演进与防乱机制

**变更归属判定表**(新代码放哪,一查便知):

| 新东西 | 去处 |
| --- | --- |
| 新方块种类/世界数据结构 | voxel(+types 若引入新 ID) |
| 新命令/事件/信封载荷/排序规则 | messaging |
| 物理行为、桥、激活算法、迁移提取 | physics-sim |
| 并发、tick、所有权、分片生命周期 | shard |
| 落盘/读盘、快照存储格式 | storage |
| 对外协议缝、回放、增量输出 | sim-api |
| 配置项、驱动流程、网络、CLI、度量 | app |

**结构与流程纪律**
- 新需求必须先有 REQ 文档(三驱动模型),REQ 里写明归属 crate——本表是判定依据;
- 文件超 ~500 行或模块出现第三个 unrelated 概念时必须拆;模块按概念切,不按技术层切(不搞 `utils`/`helpers`,公共小工具进最近的类型归属 crate);
- 跨 crate API 变更 = 先改测试(验收测试是契约);`pub` 之外一律 `pub(crate)`,新 `pub` 需在 PR 说明"谁消费";
- 禁止绕层调用:app 不得直操 physics-sim/shard 内部(只能经 sim-api 或装配期构造);网络代码未来只出现在 app,协议形状只出现在 sim-api;
- 每 crate lib.rs 顶部 doc comment 写:职责一句话 + 依赖边界 + 禁止事项(与 §2 表同步)。

**现有占位 crate 的迁移**(开工第一步,单独提交)
1. `git mv crates/world crates/voxel`、`git mv crates/physics crates/physics-sim`(占位代码改 doc 注释,旧接口废弃);
2. 新建 types/messaging/storage/shard/sim-api 空骨架(lib.rs doc + Cargo.toml,零逻辑);
3. `cargo check` 绿后,从 voxel(chunk.rs + 单测)起按 §3 图纸逐模块填充——每模块先写验收测试。

## 8. 明确不做(防过度工程)

- 不引入 xtask/benches/发布配置(cargo bench 内联即可);
- 不做 crate 版本独立管理(workspace 统一版本);
- 不预留"插件系统"挂钩(REQ-001 §9.9 网络之外的外围后置);
- 不为未来优化预建抽象(贪心合并、palette 压缩等留接口位不写实现)。
