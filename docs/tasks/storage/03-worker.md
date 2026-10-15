# 任务 03:单写 worker 与持久化屏障

## 目标

`worker.rs`:存储写入的唯一执行者(照 v2 持久化屏障)。一个 OS 线程独占
`Box<dyn ChunkStore>` + `Box<dyn CheckpointStore>`;外部只投递操作;
`flush()` 的 oneshot ack = "此前全部操作已落库"——卸载 chunk 前等待它,
持久化屏障即成(README 决策 5/6)。

## 必读

- `crates/storage/src/traits.rs`(任务 01)
- `crates/shard/src/scheduler.rs`(本仓库线程 + 通道 + join 的既有风格:
  线程命名、`Option::take` 幂等 join、Drop 兜底不 panic、中文 panic 消息)
- `docs/design/codebase-layout.md` §3 worker 行、§4
- `docs/tasks/storage/README.md` 决策 1/5/6

## 交付物

`crates/storage/src/worker.rs`:类型 + 内嵌单元测试(redb 临时库验证真落盘)。

## 接口设计(签名固定)

```rust
/// 存储操作命令(私有)。
// enum StoreOp { PutChunks(Vec<(ChunkKey, Chunk)>), AppendCheckpoint(Tick, Vec<u8>),
//                Flush(std::sync::mpsc::SyncSender<Result<(), StoreError>>) }

/// 单写 worker 的句柄(克隆/共享由调用方决定;内部通道多生产者安全)。
pub struct StoreWorker { /* op 发送端 + JoinHandle,私有 */ }

/// 关停结果。
impl StoreWorker {
    /// 起单写线程(命名 "storage-writer"),独占两个存储后端。
    pub fn new(chunks: Box<dyn ChunkStore>, checkpoints: Box<dyn CheckpointStore>) -> Self;

    /// 投递批量 chunk 写(fire-and-forget,&self 即可)。
    pub fn put_chunks(&self, batch: Vec<(ChunkKey, Chunk)>);

    /// 投递检查点追加(fire-and-forget)。
    pub fn append_checkpoint(&self, at: Tick, blob: Vec<u8>);

    /// 持久化屏障:排空此前全部操作后经 oneshot 回 ack。
    /// 粘滞错误(决策 6)下 ack 与 shutdown 恒为 Err。
    pub fn flush(&self) -> std::sync::mpsc::Receiver<Result<(), StoreError>>;

    /// 显式关停:排空队列 → join 线程 → 返回粘滞错误状态。
    /// Drop 同样兜底 join(不 panic,fail-stop 显式路径由本方法承担,照 Scheduler)。
    pub fn shutdown(self) -> Result<(), StoreError>;
}
```

oneshot 用 `std::sync::mpsc::sync_channel::<Result<(), StoreError>>(1)`;
操作通道用标准 `channel`(unbounded,投递永不阻塞 tick 循环)。

## 语义细则(逐条照做)

1. worker 循环 `recv()`:`PutChunks`/`AppendCheckpoint` 立即应用到后端,任一失败
   → 记入粘滞错误(此后不清除,决策 6);`Flush(ack_tx)` → `ack_tx.send(Ok/Err)`
   (send 失败 = 对端已放弃,fail-stop panic);
2. `flush()` 的 ack 语义 = "该 Flush 之前的全部操作已应用"(通道 FIFO 保证);
3. `shutdown(self)`:投递一个终末 Flush 感知排空(或直接 drop 发送端令 recv 退出,
   二选一,注明取舍)→ join → 返回粘滞错误;`Option::take` 幂等,Drop 与 shutdown
   不会双 join;
4. 线程内不读墙钟、不随机、无其它 IO(存储后端自身的 IO 除外);
5. 句柄 `Send + Sync`(通道端天然满足);`StoreWorker` 不 `Clone`(单句柄语义,
   需要多生产者时调用方持 `&self` 共享)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `flush_ack_then_reopen_sees_all_chunks` | worker + redb 临时库:投 3 批 → flush → `recv()` 得 Ok → shutdown → 重开库 → 全部 chunk 可读 |
| `checkpoint_barrier_survives_reopen` | append_checkpoint 2 条 → flush Ok → shutdown → 重开 → read_latest 为第 2 条 |
| `interleaved_ops_single_flush` | put/append 交错投递多次 → 一次 flush 全部落库 |
| `flush_without_ops_is_ok` | 空 flush → Ok |
| `shutdown_returns_and_joins` | 正常路径 shutdown 返回 Ok;Drop 路径不挂起 |
| `handle_is_send_sync` | 编译期断言 `StoreWorker: Send + Sync` |

错误粘滞路径(磁盘故障注入)本轮不测,注明 not-covered 即可。

## 完成定义

见 [README 公共完成定义](README.md)。
