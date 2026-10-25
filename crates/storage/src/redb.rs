//! redb 实现(chunk/checkpoint 合一的单文件库;README 决策 4)。
//!
//! 表设计(README 决策 4,任务 02 细则 1):
//! - chunk 表:key = `(world u32, dim u32, x i32, z i32)`,value = `&[u8]`
//!   = bincode(`voxel::Chunk`,`bincode::serde` + `config::standard()`);
//! - checkpoint 表:key = `u64`(`CheckpointId`),value = redb 原生元组
//!   `(tick u64, blob Vec<u8>)`。
//!
//! 存储格式全部私有(README 决策 9):trait 面上只有 `voxel::Chunk` 与 `&[u8]`。

use std::path::Path;

use redb::{Database, ReadableDatabase, ReadableTable, ReadableTableMetadata, TableDefinition};
use types::Tick;
use voxel::Chunk;

use crate::traits::{CheckpointId, CheckpointStore, ChunkKey, ChunkStore, StoreError};

/// chunk 表名(任务 02 细则 6:常量私有,不泄漏到 crate 外)。
const CHUNKS_TABLE: &str = "chunks";
/// checkpoint 表名(同上)。
const CHECKPOINTS_TABLE: &str = "checkpoints";

/// chunk 表:key 四元组(world, dim, x, z)→ bincode(Chunk)。
const CHUNKS: TableDefinition<(u32, u32, i32, i32), &[u8]> = TableDefinition::new(CHUNKS_TABLE);
/// checkpoint 表:自增 id → (tick, blob)。
const CHECKPOINTS: TableDefinition<u64, (u64, Vec<u8>)> = TableDefinition::new(CHECKPOINTS_TABLE);

/// 打开(或创建)一个 chunk + checkpoint 合一的单文件存储。
///
/// `Database`(`Arc` 内部件)可跨线程共享:`RedbStore: Send` 满足两个 trait 的
/// supertrait(README 决策 2),由单写 worker 线程持有。
pub struct RedbStore {
    db: Database,
    /// checkpoint 自增计数器:`open` 时从表内最大键初始化(细则 3),
    /// 重启后不回退、不复用。`next_id` 即最近一次分配的 ID(0 = 尚未分配)。
    next_checkpoint_id: u64,
    /// checkpoint 保留数(#17;REQ-001 决议 11 修订):0 = 不清理;`keep > 0`
    /// 时每次 `append` 在同一写事务里删 `id ≤ 新 id − keep` 的旧行。
    /// `open` 恒 0(语义不变),`open_with_keep` 按参数设定并在打开时先清理。
    keep: usize,
}

/// 把 `Database::create` 的失败映射成 `StoreError`:IO(文件无法创建/打开、锁被占)
/// 包 `Io`(细则 5),其余(格式不符等)包 `Db`。
fn map_create_error(err: redb::DatabaseError) -> StoreError {
    match err {
        redb::DatabaseError::Storage(redb::StorageError::Io(io)) => StoreError::Io(io),
        other => StoreError::Db(other.to_string()),
    }
}

/// 把事务/表/提交路径上的 redb 错误统一包进 `StoreError::Db`(细则 5;
/// 这类错误不属于可预期 IO,数据库损坏时由上层 fail-stop)。
fn map_db_error<E: Into<redb::Error>>(err: E) -> StoreError {
    StoreError::Db(err.into().to_string())
}

impl RedbStore {
    /// 打开/创建数据库文件,并设定 checkpoint 保留数(#17;REQ-001 决议 11 修订、D1)。
    /// `keep = 0` 不清理(同 [`Self::open`]);`keep > 0` 时:打开时若现存 checkpoint 多于 `keep`,
    /// 立即删到只剩最近 `keep` 个;此后每次 `append` 在**同一个写事务**里插入新行并删除
    /// `id <= 新 id − keep` 的旧行。最新一个永不删除,ID 单调、不回退、不复用。
    ///
    pub fn open_with_keep(path: &Path, keep: usize) -> Result<Self, StoreError> {
        let mut store = Self::open(path)?;
        // 打开时清理(#17;REQ-001 决议 11 修订):现存多于 `keep` 个 → 立即删到
        // 只剩最近 `keep` 个(单个写事务,调小 keep 后重启生效)。范围删除
        // `id ≤ 最大键 − keep`,最新一个(最大键)永不删,故 ID 计数器(取自
        // 清理前的最大键)不受影响、不回退、不复用;`keep = 0` 不清理。
        if keep > 0 && store.next_checkpoint_id > keep as u64 {
            let write_txn = store.db.begin_write().map_err(map_db_error)?;
            {
                let mut table = write_txn.open_table(CHECKPOINTS).map_err(map_db_error)?;
                table
                    .retain_in(..=(store.next_checkpoint_id - keep as u64), |_, _| false)
                    .map_err(map_db_error)?;
            }
            write_txn.commit().map_err(map_db_error)?;
        }
        store.keep = keep;
        Ok(store)
    }

    /// 打开/创建数据库文件。路径由调用方给定(app 配置)。
    pub fn open(path: &Path) -> Result<Self, StoreError> {
        // `Database::create`:文件不存在/为空则新建,有效库则打开(redb 4 文档语义)。
        let db = Database::create(path).map_err(map_create_error)?;
        // checkpoint 计数器从表最大键恢复:重开旧库再 append,ID 继续增长(细则 3)。
        // 用最大键而非行数:append 失败会在序号上留洞(线程粘滞错误路径),
        // 行数 < 最大键时按行数恢复会复用已存在 ID、静默覆盖(qwen 评审发现,
        // 架构师复核升级采纳,2026-10-14)。#17 起 keep 清理会减少行数——按
        // 最大键恢复的口径保证清理后 ID 仍不复用。
        let next_checkpoint_id = {
            let read_txn = db.begin_read().map_err(map_db_error)?;
            match read_txn.open_table(CHECKPOINTS) {
                // 表已存在:最大键即历史最大已落库 ID(允许有洞,不回退、不复用)。
                Ok(table) => table
                    .last()
                    .map_err(map_db_error)?
                    .map_or(0, |(k, _)| k.value()),
                // 空库首开:表还不存在,计 0。
                Err(redb::TableError::TableDoesNotExist(_)) => 0,
                Err(e) => return Err(map_db_error(e)),
            }
        };
        Ok(Self {
            db,
            next_checkpoint_id,
            keep: 0,
        })
    }

    /// bincode 编码 chunk(`bincode::serde` + standard 配置,README 决策 3)。
    fn encode_chunk(chunk: &Chunk) -> Result<Vec<u8>, StoreError> {
        bincode::serde::encode_to_vec(chunk, bincode::config::standard())
            .map_err(|e| StoreError::Decode(e.to_string()))
    }

    /// bincode 解码 chunk;字节损坏属可预期失败 → 类型化 `Decode`。
    fn decode_chunk(bytes: &[u8]) -> Result<Chunk, StoreError> {
        bincode::serde::decode_from_slice(bytes, bincode::config::standard())
            .map(|(chunk, _)| chunk)
            .map_err(|e| StoreError::Decode(e.to_string()))
    }

    /// `ChunkKey` → 表键四元组(字段顺序 = 表定义,README 决策 4)。
    fn chunk_key_parts(key: ChunkKey) -> (u32, u32, i32, i32) {
        (key.world.0, key.dim.0, key.pos.x, key.pos.z)
    }
}

impl ChunkStore for RedbStore {
    fn put_batch(&mut self, batch: &[(ChunkKey, Chunk)]) -> Result<(), StoreError> {
        // 一个写事务 = 原子批量(redb 事务天然原子,细则 2);
        // 事务内按序 insert,同键后写覆盖(last-wins,README 决策 7)。
        let write_txn = self.db.begin_write().map_err(map_db_error)?;
        {
            let mut table = write_txn.open_table(CHUNKS).map_err(map_db_error)?;
            for (key, chunk) in batch {
                let encoded = Self::encode_chunk(chunk)?;
                table
                    .insert(Self::chunk_key_parts(*key), encoded.as_slice())
                    .map_err(map_db_error)?;
            }
        }
        write_txn.commit().map_err(map_db_error)?;
        Ok(())
    }

    fn get(&self, key: ChunkKey) -> Result<Option<Chunk>, StoreError> {
        let read_txn = self.db.begin_read().map_err(map_db_error)?;
        match read_txn.open_table(CHUNKS) {
            Ok(table) => {
                let bytes = table
                    .get(Self::chunk_key_parts(key))
                    .map_err(map_db_error)?;
                match bytes {
                    Some(guard) => {
                        // AccessGuard 解引用出存的字节,再走 bincode 解码。
                        Ok(Some(Self::decode_chunk(guard.value())?))
                    }
                    None => Ok(None),
                }
            }
            // 空库首开,表未建:语义上就是"未存"。
            Err(redb::TableError::TableDoesNotExist(_)) => Ok(None),
            Err(e) => Err(map_db_error(e)),
        }
    }

    fn len(&self) -> Result<usize, StoreError> {
        let read_txn = self.db.begin_read().map_err(map_db_error)?;
        match read_txn.open_table(CHUNKS) {
            // count API 直接给行数(redb 4 `ReadableTableMetadata::len`),无需迭代。
            Ok(table) => Ok(table.len().map_err(map_db_error)? as usize),
            Err(redb::TableError::TableDoesNotExist(_)) => Ok(0),
            Err(e) => Err(map_db_error(e)),
        }
    }

    fn is_empty(&self) -> Result<bool, StoreError> {
        Ok(self.len()? == 0)
    }
}

impl CheckpointStore for RedbStore {
    fn append(&mut self, at: Tick, blob: &[u8]) -> Result<CheckpointId, StoreError> {
        // 先递增再占用:ID 从 1 起,0 永不有效(traits.rs `CheckpointId` doc);
        // `next_checkpoint_id` 在 open 时已恢复到历史最大值,重启不回退(细则 3)。
        self.next_checkpoint_id += 1;
        let id = self.next_checkpoint_id;

        let write_txn = self.db.begin_write().map_err(map_db_error)?;
        {
            let mut table = write_txn.open_table(CHECKPOINTS).map_err(map_db_error)?;
            table
                .insert(id, (at.0, blob.to_vec()))
                .map_err(map_db_error)?;
            // 删旧与插入在同一个写事务(#17;REQ-001 决议 11 修订):中途崩溃
            // 不会留下多于/少于 keep 个的状态。范围删除 `id ≤ 新 id − keep`
            // (retain_in 界定范围,不做全表扫描);最新一个(新 id)永不删;
            // `keep = 0` 不清理;`新 id ≤ keep` 时不删(减法不下溢)。
            if self.keep > 0 && id > self.keep as u64 {
                table
                    .retain_in(..=(id - self.keep as u64), |_, _| false)
                    .map_err(map_db_error)?;
            }
        }
        write_txn.commit().map_err(map_db_error)?;
        Ok(CheckpointId(id))
    }

    fn read_latest(&self) -> Result<Option<(Tick, Vec<u8>)>, StoreError> {
        let read_txn = self.db.begin_read().map_err(map_db_error)?;
        match read_txn.open_table(CHECKPOINTS) {
            Ok(table) => {
                // CheckpointId 单调 ⇒ 最大键即最新(与 mem 实现同一契约)。
                match table.last().map_err(map_db_error)? {
                    Some((_, value)) => {
                        let (tick, blob) = value.value();
                        Ok(Some((Tick(tick), blob)))
                    }
                    None => Ok(None),
                }
            }
            Err(redb::TableError::TableDoesNotExist(_)) => Ok(None),
            Err(e) => Err(map_db_error(e)),
        }
    }

    fn read(&self, id: CheckpointId) -> Result<Option<(Tick, Vec<u8>)>, StoreError> {
        let read_txn = self.db.begin_read().map_err(map_db_error)?;
        match read_txn.open_table(CHECKPOINTS) {
            Ok(table) => match table.get(id.0).map_err(map_db_error)? {
                Some(guard) => {
                    let (tick, blob) = guard.value();
                    Ok(Some((Tick(tick), blob)))
                }
                None => Ok(None),
            },
            Err(redb::TableError::TableDoesNotExist(_)) => Ok(None),
            Err(e) => Err(map_db_error(e)),
        }
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

    /// 每个测试独立的临时目录:同一路径不能被两个 Database 实例同时打开。
    fn temp_db() -> (tempfile::TempDir, std::path::PathBuf) {
        let dir = tempfile::tempdir().expect("创建临时目录");
        let path = dir.path().join("data.redb");
        (dir, path)
    }

    #[test]
    fn redb_roundtrip_after_reopen() {
        let (_dir, path) = temp_db();
        let ka = key(0, 0, 1, -2);
        let kb = key(0, 0, 2, 3);
        let ca = stone_chunk(3, 100, 7);
        let cb = stone_chunk(15, 383, 15);

        {
            let mut store = RedbStore::open(&path).expect("首次打开即创建");
            store
                .put_batch(&[(ka, ca.clone()), (kb, cb.clone())])
                .unwrap();
        }
        // drop(store) 后重新 open 同路径:真落盘、可重开。
        let store = RedbStore::open(&path).expect("重开已有库");
        assert_eq!(store.get(ka).unwrap(), Some(ca), "chunk a 重开后取回");
        assert_eq!(store.get(kb).unwrap(), Some(cb), "chunk b 重开后取回");
        assert_eq!(store.len().unwrap(), 2);
    }

    #[test]
    fn redb_missing_returns_none() {
        let (_dir, path) = temp_db();
        let mut store = RedbStore::open(&path).unwrap();
        store.put_batch(&[(key(0, 0, 0, 0), Chunk::air())]).unwrap();

        assert_eq!(
            store.get(key(0, 0, 9, 9)).unwrap(),
            None,
            "同 world/dim 未写坐标"
        );
        assert_eq!(
            store.get(key(1, 0, 0, 0)).unwrap(),
            None,
            "同坐标不同 world"
        );
        assert_eq!(store.get(key(0, 1, 0, 0)).unwrap(), None, "同坐标不同 dim");
        assert_eq!(store.get(key(1, 1, 9, 9)).unwrap(), None, "全新键");
    }

    #[test]
    fn redb_world_dim_do_not_collide() {
        let (_dir, path) = temp_db();
        let mut store = RedbStore::open(&path).unwrap();
        let pos = (3, -4);
        let c00 = stone_chunk(0, 0, 0);
        let c10 = stone_chunk(1, 0, 1);
        let c01 = stone_chunk(2, 0, 2);
        store
            .put_batch(&[
                (key(0, 0, pos.0, pos.1), c00.clone()),
                (key(1, 0, pos.0, pos.1), c10.clone()),
                (key(0, 1, pos.0, pos.1), c01.clone()),
            ])
            .unwrap();

        assert_eq!(store.get(key(0, 0, pos.0, pos.1)).unwrap(), Some(c00));
        assert_eq!(store.get(key(1, 0, pos.0, pos.1)).unwrap(), Some(c10));
        assert_eq!(store.get(key(0, 1, pos.0, pos.1)).unwrap(), Some(c01));
        assert_eq!(
            store.len().unwrap(),
            3,
            "同 pos 三个 (world,dim) 键互不覆盖"
        );
    }

    #[test]
    fn redb_batch_is_atomic_last_wins() {
        let (_dir, path) = temp_db();
        let k = key(0, 0, 0, 0);
        let first = stone_chunk(0, 0, 0);
        let second = stone_chunk(15, 383, 15);

        {
            let mut store = RedbStore::open(&path).unwrap();
            store
                .put_batch(&[(k, first), (k, second.clone())])
                .expect("同批重复键合法:后写覆盖");
        }
        let store = RedbStore::open(&path).unwrap();
        assert_eq!(store.get(k).unwrap(), Some(second), "重开后验证:后值生效");
        assert_eq!(store.len().unwrap(), 1, "同批重复键只算一条");
    }

    #[test]
    fn redb_checkpoint_ids_continue_after_reopen() {
        let (_dir, path) = temp_db();
        {
            let mut store = RedbStore::open(&path).unwrap();
            assert_eq!(store.append(Tick(10), b"a").unwrap(), CheckpointId(1));
            assert_eq!(store.append(Tick(20), b"b").unwrap(), CheckpointId(2));
        }
        let mut store = RedbStore::open(&path).unwrap();
        // 重开旧库再 append:ID 继续增长,不回退、不复用。
        assert_eq!(store.append(Tick(30), b"c").unwrap(), CheckpointId(3));
    }

    #[test]
    fn redb_checkpoint_latest_survives_reopen() {
        let (_dir, path) = temp_db();
        {
            let mut store = RedbStore::open(&path).unwrap();
            store.append(Tick(10), b"one").unwrap();
            store.append(Tick(20), b"two").unwrap();
            store.append(Tick(30), b"three").unwrap();
        }
        let store = RedbStore::open(&path).unwrap();

        let (tick, data) = store.read_latest().unwrap().expect("非空库必有最新");
        assert_eq!(tick, Tick(30));
        assert_eq!(data, b"three".to_vec());

        let (tick, data) = store.read(CheckpointId(1)).unwrap().expect("id=1 存在");
        assert_eq!(tick, Tick(10));
        assert_eq!(data, b"one".to_vec());

        assert_eq!(store.read(CheckpointId(99)).unwrap(), None, "不存在的 id");
        assert_eq!(store.read(CheckpointId(0)).unwrap(), None, "id=0 永不有效");
    }

    #[test]
    fn redb_store_is_send() {
        assert_send::<RedbStore>();
    }

    /// #17 K1/K3(单元):`open_with_keep` 追加即保留最近 N 个;`keep = 1`
    /// 只留最新;`keep = 0` 不清理。删除多留/少留、keep = 0 也清理都会红。
    #[test]
    fn redb_open_with_keep_retains_last_n() {
        let (_dir, path) = temp_db();
        let mut store = RedbStore::open_with_keep(&path, 3).expect("打开");
        for i in 1..=8u64 {
            assert_eq!(
                store.append(Tick(i * 10), &[i as u8]).unwrap(),
                CheckpointId(i),
                "ID 从 1 起连续递增"
            );
        }
        for i in 1..=8u64 {
            let present = store.read(CheckpointId(i)).unwrap().is_some();
            assert_eq!(
                present,
                i > 5,
                "keep = 3:第 {i} 个应{}",
                if i > 5 { "保留" } else { "删除" }
            );
        }
        let (tick, blob) = store.read_latest().unwrap().expect("最新一个永远在");
        assert_eq!(tick, Tick(80));
        assert_eq!(blob, vec![8u8]);

        // keep = 1:只留最新一个。
        let (_dir1, path1) = temp_db();
        let mut s1 = RedbStore::open_with_keep(&path1, 1).expect("打开");
        for i in 1..=5u64 {
            s1.append(Tick(i), &[1]).unwrap();
        }
        for i in 1..=5u64 {
            assert_eq!(
                s1.read(CheckpointId(i)).unwrap().is_some(),
                i == 5,
                "keep = 1:只留第 5 个"
            );
        }

        // keep = 0:不清理(同 open 语义)。
        let (_dir0, path0) = temp_db();
        let mut s0 = RedbStore::open_with_keep(&path0, 0).expect("打开");
        for i in 1..=5u64 {
            s0.append(Tick(i), &[1]).unwrap();
        }
        for i in 1..=5u64 {
            assert!(
                s0.read(CheckpointId(i)).unwrap().is_some(),
                "keep = 0:全保留"
            );
        }
    }

    /// #17 K2(单元):清理后重开 ID 继续单调(按最大键恢复——按行数恢复的
    /// 实现会复用 ID,此处必红);打开时现存多于 keep 个立即删到 keep 个
    /// (调小 keep 后重启生效);`open` 语义不变(不清理)。
    #[test]
    fn redb_keep_prune_ids_monotonic_and_prune_on_open() {
        let (_dir, path) = temp_db();
        {
            let mut store = RedbStore::open_with_keep(&path, 3).expect("打开");
            for i in 1..=5u64 {
                store.append(Tick(i), &[1]).unwrap();
            }
            let present: Vec<u64> = (1..=5)
                .filter(|i| store.read(CheckpointId(*i)).unwrap().is_some())
                .collect();
            assert_eq!(present, vec![3, 4, 5], "追加期清理:只剩最近 3 个");
        }
        let mut store = RedbStore::open_with_keep(&path, 3).expect("重开");
        // 行数 = 3,按行数恢复的 ID 会是 4——必须得 6(最大键 5 + 1)。
        assert_eq!(
            store.append(Tick(6), &[2]).unwrap(),
            CheckpointId(6),
            "清理后重开:ID 继续单调,不复用"
        );
        let present: Vec<u64> = (1..=6)
            .filter(|i| store.read(CheckpointId(*i)).unwrap().is_some())
            .collect();
        assert_eq!(present, vec![4, 5, 6]);

        // `open` 不清理;以 keep = 3 重开 → 打开时即删到只剩最近 3 个。
        let (_dir2, path2) = temp_db();
        {
            let mut s2 = RedbStore::open(&path2).expect("打开(不清理)");
            for i in 1..=8u64 {
                s2.append(Tick(i), &[1]).unwrap();
            }
        }
        let s3 = RedbStore::open_with_keep(&path2, 3).expect("以 keep = 3 重开");
        let present: Vec<u64> = (1..=8)
            .filter(|i| s3.read(CheckpointId(*i)).unwrap().is_some())
            .collect();
        assert_eq!(present, vec![6, 7, 8], "打开时即删到只剩最近 3 个");
    }
}
