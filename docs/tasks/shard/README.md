# shard crate 实现任务书(总览)

- 状态:已定稿(架构师签发)
- 日期:2026-10-14
- 上游:[REQ-001 总体架构](../../requirements/REQ-001-architecture.md)、[代码布局蓝图 §3 shard](../../design/codebase-layout.md)
- 实现方式:workflow 指挥子 agent(模型 `qwen3.8-flash-next-fp8`)按任务书 TDD 实现;
  cargo test + clippy 为机器门禁;架构师最终人工验收。

## 任务顺序(依赖序,串行执行)

| # | 任务书 | 模块 | 内容 |
| --- | --- | --- | --- |
| 0 | [00-skeleton.md](00-skeleton.md) | 全 crate | 依赖声明、模块文件、`cargo check` 绿 |
| 1 | [01-coordinator.md](01-coordinator.md) | coordinator.rs | TickCoordinator:单调 +1,不看墙钟 |
| 2 | [02-ownership.md](02-ownership.md) | ownership.rs | OwnershipMap:claim/release,重复即 fail-stop |
| 3 | [03-lifecycle.md](03-lifecycle.md) | lifecycle.rs | Active→Draining→Stopped;空闲 despawn 判定 |
| 4 | [04-router.md](04-router.md) | router.rs | 有界收件箱 + 信封盖章分发(tick N+1 前缀) |
| 5 | [05-scheduler.md](05-scheduler.md) | scheduler.rs | 固定 worker 池 + capacity-one 通道 + 每拍屏障 |
| 6 | [06-integration.md](06-integration.md) | tests/ | 双分片迁移端到端 + 全链路确定性 |

## 每份任务书的公共完成定义

1. 严格 TDD:先写【测试清单】全部测试 + 最小 `todo!()` 桩,`cargo test -p shard` 见红
   (编译通过、测试失败),再实现到绿;
2. 门禁(机器判定,不可绕过):
   - `cargo test -p shard --all-targets` 退出码 0;
   - `cargo clippy -p shard --all-targets -- -D warnings` 退出码 0;
3. 只改任务书【交付物】与【授权改动】列出的文件。

## 架构师在本轮做出的具体决策(实现时不得偏离)

1. **收件箱默认容量 256**(v2 未定死;命令突发远小于信封,256 足够测试与首版);
2. **提交给未注册分片 = panic**(fail-stop:调用方 bug);收件箱满 = 类型化
   `RouteError::InboxFull`(运行期反压,调用方断开权威输入)——两者语义不同;
3. **信封转 StampedInput 时 source 统一用 `InputSource::SCHEDULER`**,sequence 用
   `envelope.source_sequence`(`Envelope::stamp` 已保证 apply_at ≡ produced_at + 1;
   信封间相对序由 `CrossShardQueue::take_batch` 的 canonical 排序 + 稳定排序保证);
4. **lifecycle 本轮只接线不自动化**:register→activate、tick 内记活动;"空片+无票满
   1800 拍自动 despawn"留到激活/票务轮次(需要实体数与票信号);
5. **`BlockRegistry` trait 增加 `Send` supertrait**(crate voxel 一行改动):worker 池
   要把 `ShardSim`(内含 `Box<dyn BlockRegistry>`)移过线程;`SimpleRegistry` 纯数据
   自动满足;
6. **迟到输入 = panic**:`submit` 的 `apply_at < 当前 tick` 说明该拍已分发,属调用方
   bug(fail-stop);
7. **调度器 `tick()` 语义**:第 k 次调用执行第 k-1 拍(从 0 起)——先按当前
   `coordinator.tick()` 分发前缀、step、回收 outbox,最后 `advance()`。
