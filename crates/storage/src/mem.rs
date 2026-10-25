//! `ChunkStore`/`CheckpointStore` 的内存实现(测试/回放用;蓝图 §3)。
//!
//! 容器一律 `BTreeMap`(确定性序,蓝图 §4);与 redb 后端可互换,测试在
//! 本模块内嵌(契约语义见 [crate::traits] 各方法 doc)。

use std::collections::BTreeMap;

use types::Tick;
use voxel::Chunk;

use crate::traits::{CheckpointId, CheckpointStore, ChunkKey, ChunkStore, StoreError};

/// chunk 的内存存储:`ChunkKey` 有序 → 确定性迭代。
#[derive(Debug, Default)]
pub struct MemChunkStore {
    chunks: BTreeMap<ChunkKey, Chunk>,
}

impl MemChunkStore {
    pub fn new() -> Self {
        Self::default()
    }
}

impl ChunkStore for MemChunkStore {
    fn put_batch(&mut self, batch: &[(ChunkKey, Chunk)]) -> Result<(), StoreError> {
        // 同批重复键:按序插入,BTreeMap::insert 天然后写覆盖(last-wins,README 决策 7)。
        for (key, chunk) in batch {
            self.chunks.insert(*key, chunk.clone());
        }
        Ok(())
    }

    fn get(&self, key: ChunkKey) -> Result<Option<Chunk>, StoreError> {
        Ok(self.chunks.get(&key).cloned())
    }

    fn len(&self) -> Result<usize, StoreError> {
        Ok(self.chunks.len())
    }

    fn is_empty(&self) -> Result<bool, StoreError> {
        Ok(self.chunks.is_empty())
    }
}

/// 检查点的内存存储:`next_id` 自增、从 1 起(0 永不有效)。
#[derive(Debug, Default)]
pub struct MemCheckpointStore {
    next_id: u64,
    records: BTreeMap<CheckpointId, (Tick, Vec<u8>)>,
    /// checkpoint 保留数(#17;0 = 不清理)。`new()` 恒 0(语义不变),
    /// `with_keep` 按参数设定:追加即只留最近 `keep` 个,ID 单调不复用。
    keep: usize,
}

impl MemCheckpointStore {
    pub fn new() -> Self {
        Self::default()
    }

    /// 带保留数的内存检查点存储(#17;语义同 `RedbStore::open_with_keep`,`keep = 0` 不清理)。
    pub fn with_keep(keep: usize) -> Self {
        Self {
            keep,
            ..Self::default()
        }
    }
}

impl CheckpointStore for MemCheckpointStore {
    fn append(&mut self, at: Tick, blob: &[u8]) -> Result<CheckpointId, StoreError> {
        // 自增主键从 1 起:先递增再占用,0 永不有效(traits.rs `CheckpointId` doc)。
        self.next_id += 1;
        let id = CheckpointId(self.next_id);
        self.records.insert(id, (at, blob.to_vec()));
        // 插入即删旧(#17;与 redb 实现同一事务语义):保留最近 `keep` 个,
        // 最新一个永不删;`keep = 0` / `新 id ≤ keep` 不清理(减法不下溢)。
        if self.keep > 0 && self.next_id > self.keep as u64 {
            let cutoff = self.next_id - self.keep as u64;
            self.records.retain(|k, _| k.0 > cutoff);
        }
        Ok(id)
    }

    fn read_latest(&self) -> Result<Option<(Tick, Vec<u8>)>, StoreError> {
        // CheckpointId 单调 ⇒ BTreeMap 最大键即最新(追加写语义,README 决策 5)。
        Ok(self.records.last_key_value().map(|(_, v)| v.clone()))
    }

    fn read(&self, id: CheckpointId) -> Result<Option<(Tick, Vec<u8>)>, StoreError> {
        Ok(self.records.get(&id).cloned())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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

    fn assert_send<T: Send>() {}

    #[test]
    fn mem_chunk_roundtrip() {
        let mut store = MemChunkStore::new();
        let k = key(0, 0, 1, -2);
        let chunk = stone_chunk(3, 100, 7);
        store.put_batch(&[(k, chunk.clone())]).unwrap();

        let back = store.get(k).unwrap();
        assert_eq!(back, Some(chunk), "put 后 get 回相等的 Chunk");
        // 未存键 → Ok(None)
        assert_eq!(store.get(key(0, 0, 9, 9)).unwrap(), None);
        // 其它世界/维度不串库
        assert_eq!(store.get(key(1, 0, 1, -2)).unwrap(), None);
        assert_eq!(store.get(key(0, 1, 1, -2)).unwrap(), None);
    }

    #[test]
    fn mem_batch_last_wins() {
        let mut store = MemChunkStore::new();
        let k = key(0, 0, 0, 0);
        let first = stone_chunk(0, 0, 0);
        let second = stone_chunk(15, 383, 15);
        store
            .put_batch(&[(k, first), (k, second.clone())])
            .expect("同批重复键合法:后写覆盖");

        assert_eq!(store.get(k).unwrap(), Some(second), "后写生效");
        assert_eq!(store.len().unwrap(), 1, "同批重复键只算一条");
    }

    #[test]
    fn mem_chunk_len_tracks() {
        let mut store = MemChunkStore::new();
        assert!(store.is_empty().unwrap());
        assert_eq!(store.len().unwrap(), 0);

        let n = 5;
        let batch: Vec<_> = (0..n as i32)
            .map(|i| (key(0, 0, i, i), Chunk::air()))
            .collect();
        store.put_batch(&batch).unwrap();
        assert!(!store.is_empty().unwrap());
        assert_eq!(store.len().unwrap(), n);
    }

    #[test]
    fn mem_checkpoint_ids_monotonic_from_one() {
        let mut store = MemCheckpointStore::new();
        assert_eq!(store.append(Tick(10), b"a").unwrap(), CheckpointId(1));
        assert_eq!(store.append(Tick(20), b"b").unwrap(), CheckpointId(2));
        assert_eq!(store.append(Tick(30), b"c").unwrap(), CheckpointId(3));
    }

    #[test]
    fn mem_checkpoint_read_latest_and_by_id() {
        let mut store = MemCheckpointStore::new();
        let blob = |s: &[u8]| s.to_vec();
        store.append(Tick(10), b"one").unwrap();
        store.append(Tick(20), b"two").unwrap();
        store.append(Tick(30), b"three").unwrap();

        let (tick, data) = store.read_latest().unwrap().expect("非空库必有最新");
        assert_eq!(tick, Tick(30));
        assert_eq!(data, blob(b"three"));

        let (tick, data) = store.read(CheckpointId(2)).unwrap().expect("id=2 存在");
        assert_eq!(tick, Tick(20));
        assert_eq!(data, blob(b"two"));

        assert_eq!(store.read(CheckpointId(99)).unwrap(), None, "不存在的 id");
        // id=0 永不有效
        assert_eq!(store.read(CheckpointId(0)).unwrap(), None);
    }

    #[test]
    fn mem_stores_are_send() {
        assert_send::<MemChunkStore>();
        assert_send::<MemCheckpointStore>();
    }

    /// #17 K1/K3(单元):`with_keep` 追加即保留最近 N 个;`keep = 0` 与
    /// `new()` 不清理。内存存储没跟上保留语义(与 redb 行为不一致)会红。
    #[test]
    fn mem_with_keep_retains_last_n() {
        let mut s3 = MemCheckpointStore::with_keep(3);
        for i in 1..=8u64 {
            assert_eq!(
                s3.append(Tick(i * 10), &[i as u8]).unwrap(),
                CheckpointId(i)
            );
        }
        let present: Vec<u64> = (1..=8)
            .filter(|i| s3.read(CheckpointId(*i)).unwrap().is_some())
            .collect();
        assert_eq!(present, vec![6, 7, 8], "keep = 3:只剩最近 3 个");
        let (tick, blob) = s3.read_latest().unwrap().expect("最新一个永远在");
        assert_eq!(tick, Tick(80));
        assert_eq!(blob, vec![8u8]);

        for mut store in [MemCheckpointStore::with_keep(0), MemCheckpointStore::new()] {
            for i in 1..=5u64 {
                store.append(Tick(i), &[1]).unwrap();
            }
            let present: Vec<u64> = (1..=5)
                .filter(|i| store.read(CheckpointId(*i)).unwrap().is_some())
                .collect();
            assert_eq!(present, vec![1, 2, 3, 4, 5], "keep = 0 / new():全保留");
        }
    }
}
