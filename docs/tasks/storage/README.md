# storage crate 实现任务书(总览)

- 状态:已定稿(架构师签发)
- 日期:2026-10-14
- 上游:[REQ-001 总体架构](../../requirements/REQ-001-architecture.md)、[代码布局蓝图 §3 storage](../../design/codebase-layout.md)
- 实现方式:workflow 指挥子 agent 按任务书 TDD 实现;**模型分工(用户指定)**:
  核心(01-03)由 `glm-5.3-flash`(提供商"公司")实现;骨架(00)与集成测试(04)由
  `qwen3.8-flash-next-fp8` 实现(与实现者不同模型写验收测试,交叉验证);
  代码全量另由 qwen 独立评审。cargo test + clippy 为机器门禁;架构师最终人工验收。

## 任务顺序(依赖序,串行执行)

| # | 任务书 | 模块 | 实现模型 | 内容 |
| --- | --- | --- | --- | --- |
| 0 | [00-skeleton.md](00-skeleton.md) | 全 crate | qwen | 依赖声明、模块文件、`cargo check` 绿 |
| 1 | [01-traits-mem.md](01-traits-mem.md) | traits.rs + mem.rs | glm | 存储 trait 契约 + 内存实现(测试/回放) |
| 2 | [02-redb.md](02-redb.md) | redb.rs | glm | redb 4 持久化后端(真落盘、可重开) |
| 3 | [03-worker.md](03-worker.md) | worker.rs | glm | 单写线程 + 显式 flush→oneshot ack 屏障 |
| 4 | [04-integration.md](04-integration.md) | tests/ | qwen | 往返/后端等价/持久化屏障集成测试 |

## 每份任务书的公共完成定义

1. 严格 TDD:先写【测试清单】全部测试 + 最小 `todo!()` 桩,`cargo test -p storage` 见红
   (编译通过、测试失败),再实现到绿;
2. 门禁(机器判定,不可绕过):
   - `cargo test -p storage --all-targets` 退出码 0;
   - `cargo clippy -p storage --all-targets -- -D warnings` 退出码 0;
3. 只改任务书【交付物】与【授权改动】列出的文件。

## 架构师在本轮做出的具体决策(实现时不得偏离)

1. **同步 trait + pending-ack,不引入 tokio/async**(蓝图给的两条路"Async 或
   pending-ack"选后者):trait 方法全同步;单写 worker = 独立 OS 线程 + 标准库通道;
   app 层将来要 async 可以在边缘包一层;
2. **两个 trait 都加 `Send` supertrait**:worker 线程持有 `Box<dyn ChunkStore>` /
   `Box<dyn CheckpointStore>`,与 voxel `BlockRegistry: Send` 同理;
3. **chunk 序列化用 bincode 2(当前 2.0.1)**(serde 已在 voxel Chunk 上 derive;
   用 `bincode::serde::encode_to_vec/decode_from_slice` + `config::standard()`,
   以编译器与文档为准)。**注意:crates.io 的 bincode 3.0.0 是 `compile_error!`
   占位包(源码仅一行),不得使用**——2026-10-14 骨架实现者实测发现,升为决议;
   checkpoint blob 对 storage 不透明(sim-api 侧负责编码);
4. **redb 4.4 表设计**:chunk 表 key = `(world u32, dim u32, x i32, z i32)`,value =
   bincode(Chunk);checkpoint 表 key = `CheckpointId(u64)`(从 1 起),value 编码
   `(tick, blob)`(bincode 或 redb 原生元组,以能表达语义为准)。数据库路径由
   调用方给定;
5. **写入语义**:put/append 只入队(fire-and-forget,`&self`、无返回值);
   `flush()` 返回 oneshot ack 接收端,排空此前全部操作后回 `Result<(), StoreError>`
   ——这就是"卸载前持久化屏障"(照 v2);
6. **错误粘滞(latch)**:worker 内任一操作失败即粘滞记录,此后每次 flush ack 与
   shutdown 都返回该错误、永不清除(驱动层据此停机,fail-stop 边缘版);
7. **put_batch 同批重复键 = 后写覆盖(last-wins)**;redb 单写事务天然原子;
8. **storage 的单写线程不违反"核五 crate 无线程"纪律**(核 = types/voxel/messaging/
   physics-sim/shard;storage 本就是副作用边缘,蓝图 §2/§3);
9. **存储格式不得泄漏**:trait 面上只有 `voxel::Chunk` 与 `&[u8]`,bincode/redb
   细节全部私有。
