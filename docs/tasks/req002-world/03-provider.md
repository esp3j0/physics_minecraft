# 任务 03:存储读操作 + ChunkProvider(storage)

## 目标

REQ-002 §4.3:worker 协议加读操作;`ChunkProvider` = 持久层优先 → 生成器兜底。

## 必读

- `crates/storage/src/worker.rs`(StoreOp/oneshot ack 模式)
- `crates/voxel/src/generator.rs`(任务 00 产出)
- `docs/tasks/req002-world/README.md` 决策 7

## 交付物

`crates/storage/src/worker.rs`(Get 操作)与 `crates/storage/src/provider.rs`
(`ChunkProvider`)+ lib.rs `pub mod provider;` + 内嵌测试。

## 接口设计(签名固定)

```rust
// StoreOp 追加:同步读(oneshot 回 Option<Chunk>;worker 单执行者,读写天然串行)
Get(ChunkKey, SyncSender<Option<voxel::Chunk>>),

/// 世界 chunk 数据源:持久层优先,生成器兜底(REQ-002 §4.3,AC-G2 地基)。
pub struct ChunkProvider { /* worker 操作通道克隆 + Box<dyn WorldGenerator> + WorldSeed */ }

impl ChunkProvider {
    pub fn new(worker: &StoreWorker, generator: Box<dyn WorldGenerator>, seed: WorldSeed) -> Self;
    /// 取一个 chunk:worker 读 → Some 用之;None → generator.generate。
    /// 阻塞 recv(调用点在驱动 tick 末端,毫秒级,可接受;doc 注明)。
    pub fn get(&self, key: ChunkKey) -> Option<voxel::Chunk>;
    /// 生成器直通(不查持久层;测试/预生成用)。
    pub fn generate(&self, key: ChunkPos) -> voxel::Chunk;
}
```

## 语义细则

1. worker 崩溃路径:Get 的 oneshot send 失败 → worker panic(fail-stop,
   与 Flush ack 同款);
2. `get` 的持久层读与写路径经同一 worker → 串行一致(无脏读);
3. ChunkProvider 持 `Sender<StoreOp>` 克隆(与 StoreWorker 句柄并存;
   句柄 shutdown 后 provider.get → 通道关闭 → panic fail-stop,doc 注明)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `provider_falls_back_to_generator` | 空 store → get 返回生成器结果 |
| `store_takes_priority_over_generator` | put 修改版 chunk + flush → get 返回修改值(AC-G2) |
| `get_roundtrips_through_real_worker` | 真 worker(MemChunkStore)路径读写一致 |
| `worker_down_provider_panics` | `#[should_panic]` 句柄 shutdown 后 get(fail-stop) |
| `provider_is_send_sync` | 编译期断言 |

## 完成定义

见 [README 公共完成定义](README.md)。
