# sim-api crate 实现任务书(总览)

- 状态:已定稿(架构师签发)
- 日期:2026-10-14
- 上游:[REQ-001 总体架构](../../requirements/REQ-001-architecture.md)(AC8)、[代码布局蓝图 §3 sim-api](../../design/codebase-layout.md)
- 实现方式:与 storage 轮相同——glm-5.3-flash 写核心(01-03),qwen 写骨架/集成测试(00/04)
  + 独立只读评审,交叉验证;cargo 门禁;架构师终审。

## 任务顺序(依赖序,串行执行)

| # | 任务书 | 模块 | 实现模型 | 内容 |
| --- | --- | --- | --- | --- |
| 0 | [00-skeleton.md](00-skeleton.md) | 全 crate | qwen | 依赖声明、模块文件、`cargo check` 绿 |
| 1 | [01-snapshot-restore.md](01-snapshot-restore.md) | physics-sim + shard 缝 | glm | `ShardSim::snapshot/restore`(rapier serde)+ 调度器透传/起始拍 |
| 2 | [02-log-snapshot.md](02-log-snapshot.md) | log.rs + snapshot.rs | glm | 命令日志 + 全局快照数据结构与序列化 |
| 3 | [03-session-replay.md](03-session-replay.md) | session.rs + replay.rs | glm | 会话缝(命令进)+ 回放器(AC8) |
| 4 | [04-integration.md](04-integration.md) | tests/ | qwen | AC8 端到端:中途快照→继续→重放→终态位级一致 |

## 每份任务书的公共完成定义

1. 严格 TDD:先写【测试清单】全部测试 + 最小 `todo!()` 桩,相关 `cargo test -p …` 见红
   (编译通过、测试失败),再实现到绿;
2. 门禁(机器判定,不可绕过;每任务书的【门禁包】列出):
   - `cargo test -p <门禁包…> --all-targets` 退出码 0;
   - `cargo clippy -p <门禁包…> --all-targets -- -D warnings` 退出码 0;
3. 只改任务书【交付物】与【授权改动】列出的文件。

## 架构师在本轮做出的具体决策(实现时不得偏离)

1. **蓝图 §2 修订**:sim-api 依赖 = types/messaging/shard/storage + **physics-sim
   (仅 `ShardSim` 恢复与注册,分片字节仍不透明)+ voxel(仅 `BlockRegistry` 注入)**;
   rapier 类型仍绝不出 physics-sim,网络形状仍只在 sim-api;
2. **快照机制**:rapier3d 开 `serde-serialize` feature,`PhysicsWorld` 整体
   Serialize/Deserialize(官方已把 pipeline/ccd_solver 等纯工作区 `serde(skip)`,
   快照天然只含有状态量);`ShardSim` 的私有快照结构含 cfg/voxels/phys/entities/
   chunk_bodies/outbox;**registry 不序列化,`restore` 重注入**(与 BlockRegistry
   Send 决策同理,存档不绑注册表实例);
3. **`GlobalSnapshot { tick, per_shard: BTreeMap<ShardId, Vec<u8>> }`**(简化蓝图
   原形状;ownership ≡ 键集,ShardId 升序确定);
4. **`LogEntry { tick, dest, source, sequence, command }`**:只记外部命令;
   信封/事件/迁移不落日志(蓝图 §3 log 行);bincode 2 序列化;
5. **Session 盖章规则**:`source` 默认 `InputSource(1)`(0 保留给调度器);
   `send` 盖 `apply_at = 当前 tick_now()`(下一次 `tick()` 调用即生效),
   `sequence` 每 session 单调;`send` 返回 `LogEntry` 供调用方记录;
6. **`SchedulerConfig.start_tick`**(默认 `Tick::ZERO`):回放从快照拍对齐续跑;
   tick 语义不变(第 k 次调用执行第 k-1 拍);
7. **AC8 验收形态**:原线跑到拍 K 快照 → 原线继续到拍 N;重放 = 快照恢复 +
   重放日志中 tick>K 的命令 → 跑到拍 N;两线**终态逐实体位级相等(`==`)**;
8. 回放经 `Session::submit_raw` 重放**原章输入**(不得重新盖章,否则序号漂移)。
