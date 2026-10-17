//! 单写 worker:存储写入的唯一执行者(照 v2 持久化屏障)。
//!
//! 一个 OS 线程独占 `Box<dyn ChunkStore>` + `Box<dyn CheckpointStore>`;外部只经
//! unbounded 通道投递操作(fire-and-forget,永不阻塞 tick 循环);`flush()` 的
//! oneshot ack = "此前全部操作已落库"——卸载 chunk 前等待它,持久化屏障即成
//! (README 决策 5/6)。
//!
//! 错误粘滞(latch,决策 6):线程内任一操作失败即粘滞记录,此后每次 flush ack
//! 与 shutdown 都返回该错误、永不清除(驱动层据此停机,fail-stop 边缘版)。
//!
//! 线程风格照 `shard::scheduler`:线程命名、`Option::take` 幂等 join、Drop 兜底
//! 不 panic、中文 panic 消息。

use std::sync::mpsc::{Receiver, SyncSender, channel, sync_channel};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use types::Tick;
use voxel::Chunk;

use crate::traits::{CheckpointStore, ChunkKey, ChunkStore, StoreError};

/// 存储操作命令(通道 FIFO 即执行序,flush 的 ack 语义靠它;变体只许追加,
/// 声明序续在最后)。`pub(crate)`:操作协议是 worker 的私有面,`ChunkProvider`
/// 经同一通道发读请求(同模块树内可见,不泄漏出 crate)。
pub(crate) enum StoreOp {
    /// 原子批量写 chunk(同批重复键后写覆盖,由后端保证)。
    PutChunks(Vec<(ChunkKey, Chunk)>),
    /// 追加一条检查点(blob 对存储不透明)。
    AppendCheckpoint(Tick, Vec<u8>),
    /// 屏障:此前全部操作应用完后,经 oneshot 通道回 `Result<(), StoreError>`。
    /// `SyncSender` 容量 1 即 oneshot;send 失败 = 对端已放弃 → panic(fail-stop)。
    Flush(SyncSender<Result<(), StoreError>>),
    /// 同步读(REQ-002 §4.3):oneshot 回 `Option<Chunk>`,未存 = `None`。
    /// worker 单执行者,读写天然串行;读失败不 panic——记入粘滞错误(决策 6,
    /// 与写一致)并回 `None`(调用方兜底;粘滞错误经下一次 flush/shutdown 显形)。
    Get(ChunkKey, SyncSender<Option<Chunk>>),
}

/// 操作通道强发送端的共享槽(`ChunkProvider` 专用,`pub(crate)`):槽内是唯一
/// 强发送端,句柄 shutdown/Drop 将其取走 = 通道关闭。给 provider 弱化语义
/// (std mpsc stable 1.98 无 `WeakSender`/`downgrade`,实测):provider 存活
/// 不会令 worker 的 `recv` 永不退出,shutdown 的 join 不被阻塞;槽已空后再
/// `get` = 一致性违规 fail-stop(任务 03 细则 3)。
pub(crate) type SharedOpTx = Arc<Mutex<Option<std::sync::mpsc::Sender<StoreOp>>>>;

/// 单写 worker 的句柄(不 `Clone`:单句柄语义,多生产者由调用方持 `&self` 共享;
/// 内部通道多生产者安全)。
pub struct StoreWorker {
    /// 操作发送端共享槽:槽内是唯一强发送端,shutdown/Drop 从槽中 `take` 走即
    /// 释放(worker 的 recv 退出);`ChunkProvider` 持槽克隆(`SharedOpTx`),
    /// 槽空后其 `get` panic fail-stop(任务 03 细则 3)。
    op_tx: SharedOpTx,
    /// 写线程句柄;`Option::take` 保证 shutdown 与 Drop 不会双 join(照 Scheduler)。
    worker: Option<JoinHandle<()>>,
}

impl StoreWorker {
    /// 起单写线程(命名 "storage-writer"),独占两个存储后端。
    pub fn new(chunks: Box<dyn ChunkStore>, checkpoints: Box<dyn CheckpointStore>) -> Self {
        // unbounded channel:投递永不阻塞 tick 循环(任务书"操作通道用标准 channel")。
        let (op_tx, op_rx) = channel::<StoreOp>();
        let worker = std::thread::Builder::new()
            .name("storage-writer".to_string())
            .spawn(move || worker_main(op_rx, chunks, checkpoints))
            .expect("storage-writer 线程创建失败(系统资源耗尽)");
        Self {
            op_tx: Arc::new(Mutex::new(Some(op_tx))),
            worker: Some(worker),
        }
    }

    /// 投递批量 chunk 写(fire-and-forget,`&self` 即可;通道 unbounded,永不阻塞)。
    pub fn put_chunks(&self, batch: Vec<(ChunkKey, Chunk)>) {
        self.send(StoreOp::PutChunks(batch));
    }

    /// 投递检查点追加(fire-and-forget)。
    pub fn append_checkpoint(&self, at: Tick, blob: Vec<u8>) {
        self.send(StoreOp::AppendCheckpoint(at, blob));
    }

    /// 持久化屏障:排空此前全部操作后经 oneshot 回 ack(FIFO 保证序)。
    /// 粘滞错误(决策 6)下 ack 恒为 `Err`。
    pub fn flush(&self) -> std::sync::mpsc::Receiver<Result<(), StoreError>> {
        let (ack_tx, ack_rx) = sync_channel::<Result<(), StoreError>>(1);
        self.send(StoreOp::Flush(ack_tx));
        ack_rx
    }

    /// 操作通道共享槽的克隆(`ChunkProvider` 专用,`pub(crate)`,细则 3)。
    pub(crate) fn shared_op_tx(&self) -> SharedOpTx {
        Arc::clone(&self.op_tx)
    }

    /// 向操作通道投递一条命令。句柄已关停后再投递 = 一致性违规 panic
    /// (fail-stop,照 Scheduler 对"通道已关闭"的处理);线程意外死亡同此。
    fn send(&self, op: StoreOp) {
        // 锁中毒 = 持锁线程 panic 中(unwind 跨 Mutex)→ 一致性违规 fail-stop;
        // 槽空 = 句柄已关停后再投递,同 fail-stop。
        let guard = self
            .op_tx
            .lock()
            .expect("操作通道槽锁中毒:一致性违规(fail-stop)");
        let tx = guard
            .as_ref()
            .expect("StoreWorker 已关停后再投递操作(调用方 bug,fail-stop)");
        if tx.send(op).is_err() {
            panic!("存储操作通道已关闭:一致性违规(fail-stop)");
        }
    }

    /// 显式关停:投递终末 Flush 感知排空 → 释放发送端让线程退出 → join →
    /// 返回粘滞错误状态。幂等语义由 `Option::take` 保证,Drop 不会双 join。
    ///
    /// 取舍:选"终末 Flush"而非"直接 drop 发送端令 recv 退出"——终末 Flush 的
    /// ack = 队列排空的确定性感知点,关停路径能拿到确定的粘滞错误;直接 drop
    /// 只能靠 join 等退出,拿不到状态。代价是多一次通道往返。
    pub fn shutdown(mut self) -> Result<(), StoreError> {
        // 终末屏障:ack = 终末 Flush 之前的全部操作已应用(任务书细则 3)。
        let ack = self.flush();
        // 等终末 ack;线程 panic 中退出 → recv Err → 一致性违规 panic(fail-stop)。
        let sticky = ack
            .recv()
            .expect("storage-writer 在终末 flush 前死亡:一致性违规(fail-stop)");
        // 释放发送端(槽中取走,provider 的后续 get 同样 fail-stop)→ worker 的
        // recv 返回 Err → 正常退出;take 保证与 Drop 不双 join。
        self.op_tx
            .lock()
            .expect("操作通道槽锁中毒:一致性违规(fail-stop)")
            .take();
        if let Some(handle) = self.worker.take()
            && handle.join().is_err()
        {
            panic!("storage-writer 线程 panic 中退出:一致性违规(fail-stop)");
        }
        sticky
    }
}

impl Drop for StoreWorker {
    /// Drop 兜底:释放发送端让线程正常退出,再 join 防泄漏(`Option::take` 幂等,
    /// 与 shutdown 不双 join)。此处 join 失败不 panic(Drop 路径 panic 会 abort;
    /// fail-stop 的显式路径由 shutdown 承担,照 Scheduler::drop)。
    fn drop(&mut self) {
        if let Ok(mut guard) = self.op_tx.lock() {
            guard.take();
        }
        if let Some(handle) = self.worker.take() {
            let _ = handle.join();
        }
    }
}

/// 写线程主体:循环 `recv()`,操作立即应用到后端;`Flush` 回 oneshot ack。
/// 任一操作失败 → 记入粘滞错误(此后不清除,决策 6)。发送端全部释放 =
/// 正常退出。ack send 失败 = 对端已放弃 → panic(fail-stop)。
fn worker_main(
    op_rx: Receiver<StoreOp>,
    mut chunks: Box<dyn ChunkStore>,
    mut checkpoints: Box<dyn CheckpointStore>,
) {
    // 粘滞错误:一旦置位永不清除(决策 6);首错优先,后续错误不覆盖
    // (驱动层拿第一个即可停机)。`StoreError` 非 Clone,粘滞值与 ack 回传
    // 各持一份,用私有复制函数按变体重建。
    let mut sticky: Option<StoreError> = None;
    while let Ok(op) = op_rx.recv() {
        match op {
            StoreOp::PutChunks(batch) => {
                if let Err(e) = chunks.put_batch(&batch) {
                    sticky.get_or_insert(e);
                }
            }
            StoreOp::AppendCheckpoint(at, blob) => {
                if let Err(e) = checkpoints.append(at, &blob) {
                    sticky.get_or_insert(e);
                }
            }
            StoreOp::Get(key, ack_tx) => {
                // 单次读:Ok(Some/None) 照答;Err 记入粘滞(决策 6,与写一致)后
                // 回 `None` 让调用方兜底——粘滞错误经下一次 flush/shutdown 显形
                // (fail-stop 由粘滞通道保证,读路径自身不 panic)。ack send 失败 =
                // 调用方放弃 → panic(fail-stop,与 Flush ack 同款)。
                let answer = match chunks.get(key) {
                    Ok(found) => found,
                    Err(e) => {
                        sticky.get_or_insert(e);
                        None
                    }
                };
                if ack_tx.send(answer).is_err() {
                    panic!(
                        "get 应答对端已放弃:一致性违规(fail-stop),\
                         调用方不应在应答前释放 oneshot 接收端"
                    );
                }
            }
            StoreOp::Flush(ack_tx) => {
                let ack = match sticky.as_ref() {
                    Some(e) => Err(clone_sticky(e)),
                    None => Ok(()),
                };
                // send 失败 = 调用方放弃了 ack(对端已 drop Receiver)→ fail-stop。
                if ack_tx.send(ack).is_err() {
                    panic!(
                        "flush ack 对端已放弃:一致性违规(fail-stop),\
                         调用方不应在屏障前释放 ack 接收端"
                    );
                }
            }
        }
    }
}

/// 粘滞错误的按变体复制(`StoreError` 未 derive Clone;粘滞记录保留原件,
/// 每次 flush ack / shutdown 回传一份新副本,重复返回同一错误 = 决策 6)。
fn clone_sticky(err: &StoreError) -> StoreError {
    match err {
        StoreError::Io(e) => StoreError::Io(std::io::Error::new(e.kind(), e.to_string())),
        StoreError::Db(msg) => StoreError::Db(msg.clone()),
        StoreError::Decode(msg) => StoreError::Decode(msg.clone()),
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use crate::mem::{MemCheckpointStore, MemChunkStore};
    use crate::redb::RedbStore;
    use types::{ChunkPos, DimensionId, WorldId};

    const STONE: types::BlockId = types::BlockId(1);

    fn key(world: u32, dim: u32, x: i32, z: i32) -> ChunkKey {
        ChunkKey {
            world: WorldId(world),
            dim: DimensionId(dim),
            pos: ChunkPos::new(x, z),
        }
    }

    /// 空气 chunk,在 (x,y,z) 处放一块 STONE。
    fn stone_chunk(x: usize, y: usize, z: usize) -> Chunk {
        let mut c = Chunk::air();
        c.set(x, y, z, STONE);
        c
    }

    /// 每个测试独立的临时库:chunks 与 checkpoints 各一个文件——redb 同一路径
    /// 不能被两个 `Database` 实例同时打开(文件锁),而 worker 独占的两个后端
    /// 需要两个实例。
    fn temp_dbs() -> (tempfile::TempDir, PathBuf, PathBuf) {
        let dir = tempfile::tempdir().expect("创建临时目录");
        let chunks = dir.path().join("chunks.redb");
        let checkpoints = dir.path().join("checkpoints.redb");
        (dir, chunks, checkpoints)
    }

    fn assert_send_sync<T: Send + Sync>() {}

    /// 任务书测试 6:编译期断言 `StoreWorker: Send + Sync`。
    #[test]
    fn handle_is_send_sync() {
        assert_send_sync::<StoreWorker>();
    }

    /// 任务书测试 1:投 3 批 → flush → recv 得 Ok → shutdown → 重开库 → 全部可读。
    #[test]
    fn flush_ack_then_reopen_sees_all_chunks() {
        let (_dir, chunks_path, _cp_path) = temp_dbs();
        let store = RedbStore::open(&chunks_path).expect("首次打开即创建");
        let worker = StoreWorker::new(Box::new(store), Box::new(MemCheckpointStore::new()));

        let batches: Vec<Vec<(ChunkKey, Chunk)>> = (0..3)
            .map(|i| {
                vec![
                    (key(0, 0, i, i), stone_chunk(i as usize, 0, i as usize)),
                    (key(0, 0, i, i + 1), Chunk::air()),
                ]
            })
            .collect();
        for batch in &batches {
            worker.put_chunks(batch.clone());
        }

        let ack = worker.flush();
        assert!(
            ack.recv().expect("flush ack 必达").is_ok(),
            "正常路径 flush ack = Ok"
        );

        worker.shutdown().expect("正常路径 shutdown = Ok");

        // 重开同一路径:全部 chunk 真落盘、可读。
        let store = RedbStore::open(&chunks_path).expect("重开已有库");
        for batch in &batches {
            for (k, c) in batch {
                assert_eq!(store.get(*k).unwrap(), Some(c.clone()), "重开后取回 {k:?}");
            }
        }
        assert_eq!(store.len().unwrap(), 6, "3 批 × 每批 2 个不同键");
    }

    /// 任务书测试 2:append_checkpoint 2 条 → flush Ok → shutdown → 重开 →
    /// read_latest 为第 2 条。
    #[test]
    fn checkpoint_barrier_survives_reopen() {
        let (_dir, _chunks_path, cp_path) = temp_dbs();
        let store = RedbStore::open(&cp_path).expect("首次打开即创建");
        let worker = StoreWorker::new(Box::new(MemChunkStore::new()), Box::new(store));

        worker.append_checkpoint(Tick(100), b"one".to_vec());
        worker.append_checkpoint(Tick(200), b"two".to_vec());
        let ack = worker.flush();
        assert!(ack.recv().expect("flush ack 必达").is_ok());
        worker.shutdown().expect("正常路径 shutdown = Ok");

        let store = RedbStore::open(&cp_path).expect("重开已有库");
        let (tick, blob) = store.read_latest().unwrap().expect("两条检查点必有最新");
        assert_eq!(tick, Tick(200), "最新一条 = 第 2 条");
        assert_eq!(blob, b"two".to_vec());
    }

    /// 任务书测试 3:put/append 交错投递多次 → 一次 flush 全部落库。
    /// chunk 库与 checkpoint 库分开重开验证。
    #[test]
    fn interleaved_ops_single_flush() {
        let (_dir, chunks_path, cp_path) = temp_dbs();
        let worker = StoreWorker::new(
            Box::new(RedbStore::open(&chunks_path).expect("chunk 库首次打开")),
            Box::new(RedbStore::open(&cp_path).expect("checkpoint 库首次打开")),
        );

        for i in 0..4i32 {
            worker.put_chunks(vec![(key(0, 0, i, -i), stone_chunk(1, 2, 3))]);
            worker.append_checkpoint(Tick(i as u64 * 10), format!("cp-{i}").into_bytes());
        }
        let ack = worker.flush();
        assert!(
            ack.recv().expect("flush ack 必达").is_ok(),
            "一次 flush 全部落库"
        );
        worker.shutdown().expect("正常路径 shutdown = Ok");

        let store = RedbStore::open(&chunks_path).expect("重开 chunk 库");
        assert_eq!(store.len().unwrap(), 4, "4 个 chunk 键全落库");
        for i in 0..4i32 {
            assert!(store.get(key(0, 0, i, -i)).unwrap().is_some());
        }
        let store = RedbStore::open(&cp_path).expect("重开 checkpoint 库");
        let (tick, blob) = store.read_latest().unwrap().expect("4 条检查点必有最新");
        assert_eq!(tick, Tick(30));
        assert_eq!(blob, b"cp-3".to_vec());
    }

    /// 任务书测试 4:空 flush → Ok(队列里没有待应用操作,屏障立即达成)。
    #[test]
    fn flush_without_ops_is_ok() {
        let worker = StoreWorker::new(
            Box::new(crate::mem::MemChunkStore::new()),
            Box::new(MemCheckpointStore::new()),
        );
        let ack = worker.flush();
        assert!(
            ack.recv().expect("flush ack 必达").is_ok(),
            "空 flush 的 ack = Ok"
        );
        worker.shutdown().expect("正常路径 shutdown = Ok");
    }

    /// 任务书测试 5:正常路径 shutdown 返回 Ok(含 join 完成);Drop 路径不挂起
    /// (Drop 兜底 join;shutdown 已 take 的句柄也不会被 Drop 双 join)。
    #[test]
    fn shutdown_returns_and_joins() {
        // shutdown 路径:返回 Ok(线程确已 join,不会再有写)。
        let worker = StoreWorker::new(
            Box::new(crate::mem::MemChunkStore::new()),
            Box::new(MemCheckpointStore::new()),
        );
        worker.put_chunks(vec![(key(0, 0, 0, 0), Chunk::air())]);
        let result = worker.shutdown();
        assert!(result.is_ok(), "正常路径 shutdown = Ok,实际 {result:?}");

        // 纯 Drop 路径:直接丢弃句柄,不挂起、不 panic(Drop 兜底 join)。
        let worker = StoreWorker::new(
            Box::new(crate::mem::MemChunkStore::new()),
            Box::new(MemCheckpointStore::new()),
        );
        worker.append_checkpoint(Tick(1), b"drop-path".to_vec());
        drop(worker);
    }
}
