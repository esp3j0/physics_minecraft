//! ChunkProvider(REQ-002 §4.3):世界 chunk 数据源——持久层优先,生成器兜底。
//!
//! 读路径经与 [`crate::worker::StoreWorker`] 相同的操作通道(`StoreOp::Get`,
//! oneshot 回 `Option<Chunk>`),单写 worker 是唯一存储执行者 → 读写天然串行
//! 一致(无脏读)。未存(`None`)则以生成器兜底,保证持久层永远优先:玩家
//! 改过的 chunk 不会被生成器覆盖回初生值(数据源序,AC-G2)。
//!
//! 通道端持**共享槽**克隆([`crate::worker::SharedOpTx`],内含唯一的强发送端):
//! std mpsc 无 WeakSender(stable 1.98 实测无 `downgrade`),等价语义由槽实现
//! ——句柄 shutdown/Drop 从槽中取走强发送端 = 通道关闭,此后 `get` panic
//! fail-stop(任务细则 3);provider 存活不会令 shutdown 的 join 阻塞。
//!
//! `get` 阻塞等 oneshot:调用点在驱动 tick 末端(REQ-002 §4.4),毫秒级,可接受。

use std::sync::mpsc::sync_channel;

use types::{ChunkPos, WorldSeed};
use voxel::{Chunk, WorldGenerator};

use crate::traits::ChunkKey;
use crate::worker::{SharedOpTx, StoreOp, StoreWorker};

/// 世界 chunk 数据源:持久层优先,生成器兜底(REQ-002 §4.3,AC-G2 地基)。
pub struct ChunkProvider {
    /// worker 操作通道的共享槽克隆(与 StoreWorker 句柄并存;句柄 shutdown 后
    /// 强发送端被取走,`get` panic fail-stop,任务细则 3)。
    op_tx: SharedOpTx,
    /// 生成器兜底(纯函数,同 (seed, pos) 位级确定,AC-G1)。
    generator: Box<dyn WorldGenerator>,
    /// 世界种子:随 provider 固化(单一来源在 app 层 Config,REQ-002 §4.1)。
    seed: WorldSeed,
}

impl ChunkProvider {
    /// 从 worker 句柄建立 provider:取其操作通道共享槽的克隆(细则 3)。
    pub fn new(worker: &StoreWorker, generator: Box<dyn WorldGenerator>, seed: WorldSeed) -> Self {
        Self {
            op_tx: worker.shared_op_tx(),
            generator,
            seed,
        }
    }

    /// 取一个 chunk:worker 读 → Some 用之;None → generator.generate。
    ///
    /// 阻塞等 oneshot:调用点在驱动 tick 末端(REQ-002 §4.3),毫秒级,可接受。
    /// 句柄 shutdown/释放后调用 → panic(fail-stop,细则 3);worker 未应答
    /// (应在答前死亡)同为 fail-stop。存储读失败不在此显形——worker 记入
    /// 粘滞错误(决策 6)并回 `None`,下一次 flush/shutdown 必现 `Err`。
    pub fn get(&self, key: ChunkKey) -> Option<Chunk> {
        // oneshot 容量 1,send 必成功(接收端在本线程等待,容量即足够)。
        let (ack_tx, ack_rx) = sync_channel::<Option<Chunk>>(1);
        // 锁中毒 = 持锁线程 panic 中 → 一致性违规 fail-stop(与 worker.send 同款);
        // 槽空 = 句柄已 shutdown/Drop(强发送端被取走)→ 一致性违规 fail-stop。
        let guard = self
            .op_tx
            .lock()
            .expect("操作通道槽锁中毒:一致性违规(fail-stop)");
        let tx = guard
            .as_ref()
            .expect("ChunkProvider 在 StoreWorker 关停后取数:一致性违规(fail-stop)");
        if tx.send(StoreOp::Get(key, ack_tx)).is_err() {
            panic!("存储读操作通道已关闭:一致性违规(fail-stop)");
        }
        drop(guard);
        // worker 单执行者必答(oneshot);未答 = worker 在应答前死亡 → fail-stop。
        // `None` = 持久层未存 → 生成器兜底(数据源序:有存必用存,AC-G2)。
        match ack_rx.recv() {
            Ok(Some(chunk)) => Some(chunk),
            Ok(None) => Some(self.generate(key.pos)),
            Err(_) => panic!("存储读未获应答(storage-writer 应答前死亡):一致性违规(fail-stop)"),
        }
    }

    /// 生成器直通(不查持久层;测试/预生成用)。
    pub fn generate(&self, key: ChunkPos) -> Chunk {
        self.generator.generate(self.seed, key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mem::{MemCheckpointStore, MemChunkStore};
    use types::{DimensionId, WorldId};
    use voxel::{SeededScatter, Superflat};

    const STONE: types::BlockId = types::BlockId(1);

    fn key(world: u32, dim: u32, x: i32, z: i32) -> ChunkKey {
        ChunkKey {
            world: WorldId(world),
            dim: DimensionId(dim),
            pos: ChunkPos::new(x, z),
        }
    }

    /// 内存后端的真 worker(每个测试独立,线程经 shutdown/Drop 兜底回收)。
    fn mem_worker() -> StoreWorker {
        StoreWorker::new(
            Box::new(MemChunkStore::new()),
            Box::new(MemCheckpointStore::new()),
        )
    }

    fn assert_send<T: Send>() {}
    fn assert_send_sync<T: Send + Sync>() {}

    /// 任务书测试 5:编译期断言。通道槽与种子 Send+Sync;provider 本体因任务书
    /// 固定签名 `Box<dyn WorldGenerator>`(trait 仅约束 Send,voxel 生成器文件不
    /// 在本任务授权改动范围)不可证明 Sync——驱动为单线程 tick,Send 即足够;
    /// trait 加 Sync 约束后可同步加强此断言。
    #[test]
    fn provider_is_send_sync() {
        assert_send_sync::<SharedOpTx>();
        assert_send_sync::<WorldSeed>();
        assert_send::<ChunkProvider>();
    }

    /// 任务书测试 1:空 store → get 返回生成器结果。
    #[test]
    fn provider_falls_back_to_generator() {
        let worker = mem_worker();
        let seed = WorldSeed(42);
        let provider = ChunkProvider::new(&worker, Box::new(Superflat::classic()), seed);

        let k = key(0, 0, -3, 5);
        let expected = Superflat::classic().generate(seed, k.pos);
        assert_eq!(
            provider.get(k),
            Some(expected),
            "空持久层必走生成器兜底(数据源序:无修改则初生值)"
        );
        // 同键再取:生成器纯函数,位级一致(AC-G1)。
        let again = Superflat::classic().generate(seed, k.pos);
        assert_eq!(provider.get(k), Some(again), "重复取数位级确定");

        worker.shutdown().expect("正常路径 shutdown = Ok");
    }

    /// 任务书测试 2:put 修改版 chunk + flush → get 返回修改值(AC-G2)。
    #[test]
    fn store_takes_priority_over_generator() {
        let worker = mem_worker();
        let seed = WorldSeed(7);
        let generator = Superflat::classic();
        let provider = ChunkProvider::new(&worker, Box::new(generator.clone()), seed);

        let k = key(0, 0, 2, -1);
        // 修改版:把生成器的 stone 底层挖成 AIR——与初生值可区分。
        let mut modified = generator.generate(seed, k.pos);
        modified.set(5, 0, 5, types::BlockId::AIR);
        assert_ne!(
            modified,
            generator.generate(seed, k.pos),
            "前提:修改版与初生值不同"
        );

        worker.put_chunks(vec![(k, modified.clone())]);
        let ack = worker.flush();
        assert!(ack.recv().expect("flush ack 必达").is_ok(), "put 已应用");

        assert_eq!(
            provider.get(k),
            Some(modified),
            "持久层永远优先:修改值不被生成器覆盖(AC-G2)"
        );
        assert_ne!(
            provider.get(k),
            Some(generator.generate(seed, k.pos)),
            "兜底值不得泄漏:同键返回的是持久层修改版"
        );

        worker.shutdown().expect("正常路径 shutdown = Ok");
    }

    /// 任务书测试 3:真 worker(MemChunkStore)路径读写一致——先取为兜底值,
    /// 写入修改版并 flush 屏障后再取,读回的正是写入值(同 worker 串行,无脏读)。
    #[test]
    fn get_roundtrips_through_real_worker() {
        let worker = mem_worker();
        let seed = WorldSeed(0xfeed);
        let provider = ChunkProvider::new(&worker, Box::new(SeededScatter::new(0.4)), seed);

        let k = key(3, 0, 10, -7);
        let generated = provider.get(k).expect("空层必走生成器兜底");

        // 真 worker 写路径:在兜底值上落一块 STONE。
        let mut modified = generated;
        modified.set(1, 2, 3, STONE);
        worker.put_chunks(vec![(k, modified.clone())]);
        let ack = worker.flush();
        assert!(ack.recv().expect("flush ack 必达").is_ok(), "put 已应用");

        assert_eq!(
            provider.get(k),
            Some(modified),
            "读经同一 worker 通道 → 写后读一致(串行,无脏读)"
        );

        worker.shutdown().expect("正常路径 shutdown = Ok");
    }

    /// 任务书测试 4:句柄 shutdown 后 provider.get → 通道关闭 → panic(fail-stop,
    /// 细则 3)。弱化通道保证 shutdown 的 join 不被 provider 阻塞。
    #[test]
    #[should_panic(expected = "fail-stop")]
    fn worker_down_provider_panics() {
        let worker = mem_worker();
        let provider = ChunkProvider::new(&worker, Box::new(Superflat::classic()), WorldSeed(1));
        worker.shutdown().expect("正常路径 shutdown = Ok");
        // 句柄已 shutdown:强发送端全部释放 → upgrade 失败 → panic fail-stop。
        let _ = provider.get(key(0, 0, 0, 0));
    }
}
