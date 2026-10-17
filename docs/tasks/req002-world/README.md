# REQ-002 实现任务书(总览)

- 状态:已定稿(架构师签发;需求 [REQ-002](../../requirements/REQ-002-world-loading.md)
  2026-10-17 用户拍板,决议 1-4 全部采纳)
- 实现方式:glm-5.3-flash 写核心(00-04),qwen 写验收套件(05)+ 独立评审;
  cargo 门禁;架构师终审。

## 任务顺序(依赖序,串行执行)

| # | 任务书 | 内容 | 实现模型 | 门禁包 |
| --- | --- | --- | --- | --- |
| 0 | [00-generator.md](00-generator.md) | WorldSeed + WorldGenerator + Superflat/SeededScatter | glm | voxel |
| 1 | [01-p2-gate.md](01-p2-gate.md) | 凝固桥 P2′ 位移门 + 演员豁免(REQ-001 §9.5 修订落地) | glm | physics-sim messaging |
| 2 | [02-observers-lifecycle.md](02-observers-lifecycle.md) | 观察者命令化 + 装载编排 + despawn 候选/卸载 | glm | messaging shard sim-api |
| 3 | [03-provider.md](03-provider.md) | 存储 worker 读操作 + ChunkProvider | glm | storage |
| 4 | [04-app-wiring.md](04-app-wiring.md) | 驱动接线:ensure_shard/取数回填/despawn 屏障 | glm | app |
| 5 | [05-acceptance.md](05-acceptance.md) | AC-G1~G6 验收套件 | qwen | e2e |

## 公共完成定义

1. 严格 TDD:先写【测试清单】全部测试 + 最小 `todo!()` 桩,门禁包见红再绿;
2. 门禁(机器判定):`cargo test -p <门禁包…> --all-targets` 与
   `cargo clippy -p <门禁包…> --all-targets -- -D warnings` 退出码均 0;
3. 只改任务书【交付物】与【授权改动】列出的文件。

## 架构师实现决策(不得偏离;需求侧依据 = REQ-002 §4/§9)

1. **位移门**:滚动的"最近 20 tick 位移"窗口(保留最近 21 个 translation,
   位移 = 首尾距离),**同时约束 P2 计数路径与睡眠兜底路径**;睡眠体自然静止后
   窗口位移衰减 → 合法凝固不受阻(demo 场景 C 语义修复);
2. **演员豁免**:`SimCommand::MarkActor { entity, actor: bool }`(枚举追加)
   置位实体演员标记;演员实体**完全豁免自动凝固**(含睡眠兜底),仅将来显式
   命令触发;触及不存在实体 → panic(fail-stop);
3. **观察者专用收件箱**(REQ-002 §4.2 细化):`Scheduler` 持观察者表 +
   调度器级收件箱,`submit_observer(StampedInput)` 入箱(apply_at 语义与
   分片收件箱一致:到期分发);`tick()` 开头应用当拍到期条目;
   `Session::move_observer` 走此路径;回放按命令变体路由
   (SetObserverPosition → submit_observer 原章直投);
4. **装载编排 API**:`take_activation_requests() -> Vec<(ShardId,
   ActivationDecisions)>`(两拍之间;每拍 tick 内收集);
   `apply_chunk_load(shard, pos, chunk)` / `apply_chunk_unload(shard, pos)
   -> Option<Chunk>`(两拍之间);
5. **despawn 候选**:tick 内判定 `idle_expired ∧ entity_count==0 ∧ 无票`
   (票 = 任一观察者距分片核心 AABB ≤ chunk_unload_radius 48m),只记录;
   `take_despawn_requests() -> Vec<ShardId>`;`unload_shard(shard) ->
   Vec<(ChunkPos, Chunk)`:走 Draining→note_drained→Stopped→release,
   逐 chunk unload,实体非空 → panic(候选条件被破坏,fail-stop);
6. **ensure_shard 在 app 层**(构造 ShardSim + register;调度器不知生成器);
   `Scheduler::observers() -> Vec<(InputSource, Vec3)>` 只读视图(供驱动);
7. **ChunkProvider**(storage crate,依赖 voxel):持 worker 操作通道克隆 +
   生成器 + seed;`get(key)`:worker 读(`Get` 操作,oneshot 回 `Option<Chunk>`)
   → None 则生成器;写路径复用 put_chunks;
8. **驱动编排**(每 tick 末端,顺序固定):①观察者分片 ensure;②激活请求
   → provider.get → apply_chunk_load / 卸载累积 put_chunks(fire-and-forget);
   ③despawn 候选 → unload_shard → put_chunks → **一次 flush + recv ack**;
   DriverCommand 追加 `MoveObserver { position }`;DriverOutcome 增计数字段
   (shards_spawned/shards_despawned/chunks_loaded/chunks_unloaded);
9. **Config 增量**:`world_seed: WorldSeed`(默认 0)、`world_kind:
   WorldKind { Superflat, SeededScatter }`(默认 Superflat)——§6 单一来源。
