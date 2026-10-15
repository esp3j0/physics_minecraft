//! storage crate 级持久化集成测试(任务 04):trait 契约 × 两个后端 × 单写 worker
//! 的串起来验证——后端等价、chunk/检查点往返、flush→ack 持久化屏障。
//!
//! 与实现(任务 01-03,glm)交叉执笔(README 模型分工):本文件是对任务书的
//! 独立解读,只面向 `crates/storage/src/traits.rs` 的 trait 面与公开构造
//! (`RedbStore::open`、`StoreWorker`),不触碰 bincode/redb 私有细节(决策 9)。
//!
//! 工程纪律(任务 04 工程细节):每个测试独立 `tempfile::tempdir()`;
//! 两个 store 永不开同一文件(redb 文件锁:同一路径不能被两个 `Database`
//! 实例同时打开);大 blob 用递增字节填充(非全零,排除"零填充假通过")。

use std::path::{Path, PathBuf};

use storage::mem::{MemCheckpointStore, MemChunkStore};
use storage::redb::RedbStore;
use storage::traits::{CheckpointId, CheckpointStore, ChunkKey, ChunkStore, StoreError};
use storage::worker::StoreWorker;
use types::{ChunkPos, DimensionId, Tick, WorldId};
use voxel::{CHUNK_X, CHUNK_Y, CHUNK_Z, Chunk};

// ---------------------------------------------------------------------------
// 帮助函数
// ---------------------------------------------------------------------------

fn key(world: u32, dim: u32, x: i32, z: i32) -> ChunkKey {
    ChunkKey {
        world: WorldId(world),
        dim: DimensionId(dim),
        pos: ChunkPos::new(x, z),
    }
}

/// 由局部坐标确定性导出的方块 id(∈ 1..=250,非空气、可区分、跨后端可比)。
fn block_at(x: usize, y: usize, z: usize) -> types::BlockId {
    let h = (x as u32).wrapping_mul(73856093)
        ^ (y as u32).wrapping_mul(19349663)
        ^ (z as u32).wrapping_mul(83492791);
    types::BlockId((h % 250 + 1) as u16)
}

/// y 层底色(层号 + 1,非空气)。
fn layer_block(y: usize) -> types::BlockId {
    types::BlockId((y as u16 % 250) + 1)
}

/// 非平凡 chunk:若干层方块 + 逐坐标哈希散布 + 四角与中点特例。
/// 含最高层 (15, 383, 15) 与最低角 (0, 0, 0) 等边界点。
fn patterned_chunk() -> Chunk {
    let mut c = Chunk::air();
    // 若干层实心(y 为 3 的倍数):铺层底色。
    for y in (0..CHUNK_Y).step_by(3) {
        for x in 0..CHUNK_X {
            for z in 0..CHUNK_Z {
                c.set(x, y, z, layer_block(y));
            }
        }
    }
    // 哈希散布:每 5 层挖一个不同方块的点(内容非规则,抓序列化错位)。
    for y in (1..CHUNK_Y).step_by(5) {
        for (x, z) in [(0, 0), (7, 9), (15, 15), (15, 0), (0, 15)] {
            c.set(x, y, z, block_at(x, y, z));
        }
    }
    // 显式边界与中点特例。
    c.set(0, 0, 0, types::BlockId(250));
    c.set(CHUNK_X - 1, CHUNK_Y - 1, CHUNK_Z - 1, types::BlockId(7));
    c.set(8, 192, 8, types::BlockId(99));
    c
}

/// 抽查坐标集(≥ 20 个点,含三角边界与中心)。
fn spot_points() -> Vec<(usize, usize, usize)> {
    let mut pts = vec![
        (0, 0, 0),                               // 最低角
        (0, 0, CHUNK_Z - 1),                     // 底面对角
        (CHUNK_X - 1, 0, 0),                     // 底面另一角
        (CHUNK_X - 1, 0, CHUNK_Z - 1),           // 底面远角
        (0, CHUNK_Y - 1, 0),                     // 顶面角
        (CHUNK_X - 1, CHUNK_Y - 1, CHUNK_Z - 1), // 最高角
        (CHUNK_X / 2, CHUNK_Y / 2, CHUNK_Z / 2), // 中心
        (15, 383, 15),                           // 最高角(字面量)
        (8, 192, 8),                             // 中点特例
        (0, 383, 15),                            // 顶面对角
        (15, 0, 8),                              // 底层中线
    ];
    // 确定性网格采样:y ∈ {2, 3, 191}、x/z ∈ {0, 8, 15} 的组合 → 补足到 ≥ 20 点。
    for y in [2usize, 3, 191] {
        for x in [0usize, 8, 15] {
            for z in [0usize, 8, 15] {
                pts.push((x, y, z));
            }
        }
    }
    pts
}

/// 非全零、逐字节递增的 blob(任务 04 工程细节:`vec![b; 1 << 20]` 式大块,
/// 内容为 i as u8 递增循环,排除"写入未生效但读出零页"的假通过)。
fn ramp_blob(len: usize) -> Vec<u8> {
    (0..len).map(|i| (i % 251) as u8 + 1).collect()
}

/// 每测试独立临时目录里的 redb 路径(同一路径不得被两个 Database 同开)。
fn temp_db() -> (tempfile::TempDir, PathBuf) {
    let dir = tempfile::tempdir().expect("创建临时目录");
    let path = dir.path().join("data.redb");
    (dir, path)
}

/// worker 独占的 chunk/checkpoint 两个独立库文件。
fn temp_dbs() -> (tempfile::TempDir, PathBuf, PathBuf) {
    let dir = tempfile::tempdir().expect("创建临时目录");
    let chunks = dir.path().join("chunks.redb");
    let checkpoints = dir.path().join("checkpoints.redb");
    (dir, chunks, checkpoints)
}

fn open_store(path: &Path) -> RedbStore {
    RedbStore::open(path).expect("打开 redb 库")
}

// ---------------------------------------------------------------------------
// 测试
// ---------------------------------------------------------------------------

/// 后端等价:同一批数据分别写 MemChunkStore 与 RedbStore → 逐键 get 相等
/// (两边序列化路径一致的等价性检验)、len 相等。
#[test]
fn mem_and_redb_backends_agree() {
    let (_dir, path) = temp_db();
    let mut mem = MemChunkStore::new();
    let mut redb = open_store(&path);

    // 多键:覆盖不同 world/dim、负 chunk 坐标、空气与图案两种内容。
    let batch: Vec<(ChunkKey, Chunk)> = vec![
        (key(0, 0, 0, 0), patterned_chunk()),
        (key(0, 0, -7, 13), Chunk::air()),
        (key(1, 0, -7, 13), patterned_chunk()),
        (key(0, 2, 5, -5), {
            let mut c = Chunk::air();
            c.set(3, 100, 7, types::BlockId(42));
            c
        }),
        (key(9, 7, i32::MIN, i32::MAX), patterned_chunk()),
    ];
    mem.put_batch(&batch).expect("mem 写入");
    redb.put_batch(&batch).expect("redb 写入");

    for (k, expected) in &batch {
        let from_mem = mem.get(*k).expect("mem get").expect("mem 应有此键");
        let from_redb = redb.get(*k).expect("redb get").expect("redb 应有此键");
        assert_eq!(
            from_mem, from_redb,
            "等价性:键 {k:?} 两后端读回必须逐方块相等(redb 的 bincode 往返不改变内容)"
        );
        assert_eq!(from_mem, *expected, "键 {k:?} 读回 = 写入内容");
    }
    assert_eq!(
        mem.len().unwrap(),
        redb.len().unwrap(),
        "两后端行数一致(= 批内去重键数)"
    );
    assert_eq!(mem.len().unwrap(), batch.len(), "且 = 批内键数");
}

/// 非平凡 chunk(若干层方块)经 redb 落盘、重开后逐坐标一致:
/// 整体 PartialEq + `Chunk::get` 抽查 ≥ 20 点(含边界)。
#[test]
fn chunk_roundtrip_preserves_content() {
    let (_dir, path) = temp_db();
    let k = key(0, 0, -3, 4);
    let chunk = patterned_chunk();

    {
        let mut store = open_store(&path);
        store.put_batch(&[(k, chunk.clone())]).expect("写入");
    }
    // drop 后重开:真落盘。
    let store = open_store(&path);
    let back = store.get(k).expect("重开后 get").expect("重开后应有数据");
    assert_eq!(back, chunk, "整体逐方块相等");

    // 抽查 ≥ 20 个点(含边界),逐坐标 `Chunk::get` 一致。
    let pts = spot_points();
    assert!(pts.len() >= 20, "抽查点必须 ≥ 20,实际 {}", pts.len());
    for (x, y, z) in &pts {
        assert_eq!(
            back.get(*x, *y, *z),
            chunk.get(*x, *y, *z),
            "坐标 ({x},{y},{z}) 重开后不一致"
        );
    }
}

/// 任意 bytes blob(含空、含 1MB 递增大块)经 append → 重开 → read 逐字节相等。
#[test]
fn checkpoint_roundtrip_preserves_blob() {
    let (_dir, path) = temp_db();

    // 空、小、1MB 递增大块;记录 append 返回的 id。
    let blobs: Vec<(Tick, Vec<u8>)> = vec![
        (Tick(1), Vec::new()),
        (Tick(2), b"small-blob".to_vec()),
        (Tick(3), ramp_blob(1 << 20)),
    ];

    let mut ids = Vec::new();
    {
        let mut store = open_store(&path);
        for (tick, blob) in &blobs {
            let id = store.append(*tick, blob).expect("append");
            ids.push(id);
        }
        // 自增主键从 1 起(契约,顺带锁住 append 返回值)。
        assert_eq!(ids, vec![CheckpointId(1), CheckpointId(2), CheckpointId(3)]);
    }
    let store = open_store(&path);
    for ((tick, blob), id) in blobs.iter().zip(&ids) {
        let (got_tick, got_blob) = store.read(*id).expect("read").expect("重开后该 id 应在");
        assert_eq!(&got_tick, tick, "id {id:?} 的 tick");
        assert_eq!(
            &got_blob,
            blob,
            "id {id:?} 的 blob 逐字节相等(len {})",
            blob.len()
        );
    }
}

/// 持久化屏障方向:worker + redb,投递后 flush ack 为 Ok → shutdown → 重开 → 数据齐全
/// (ack Ok 之后重开必可见全部此前操作)。
#[test]
fn worker_flush_is_the_durability_barrier() {
    let (_dir, chunks_path, _cp_path) = temp_dbs();
    // chunk 后端 = redb(真落盘);checkpoint 后端与屏障方向无关,用内存替身。
    let worker = StoreWorker::new(
        Box::new(open_store(&chunks_path)),
        Box::new(MemCheckpointStore::new()),
    );

    let batches: Vec<Vec<(ChunkKey, Chunk)>> = (0..3)
        .map(|i| {
            vec![
                (key(0, 0, i, i), patterned_chunk()),
                (key(0, 0, i, i + 100), Chunk::air()),
            ]
        })
        .collect();
    for batch in &batches {
        worker.put_chunks(batch.clone());
    }

    // 屏障:ack Ok = 此前全部操作已落库(README 决策 5)。
    let ack: Result<(), StoreError> = worker.flush().recv().expect("flush ack 必达");
    assert!(ack.is_ok(), "正常路径 flush ack = Ok");
    worker.shutdown().expect("正常路径 shutdown = Ok");

    // ack 后重开:全部数据齐全(屏障保证的方向:ack 后必可见)。
    let store = open_store(&chunks_path);
    for batch in &batches {
        for (k, c) in batch {
            assert_eq!(store.get(*k).unwrap(), Some(c.clone()), "重开后取回 {k:?}");
        }
    }
    assert_eq!(store.len().unwrap(), 6, "3 批 × 每批 2 键");
}

/// chunk 与 checkpoint 交错投递 → 一次 flush → 重开 → 两类数据都齐全、read_latest 正确。
#[test]
fn worker_mixed_ops_all_durable_after_flush() {
    let (_dir, chunks_path, cp_path) = temp_dbs();
    let worker = StoreWorker::new(
        Box::new(open_store(&chunks_path)),
        Box::new(open_store(&cp_path)),
    );

    // 交错投递:每轮先 chunk 后 checkpoint(顺序由通道 FIFO 决定)。
    let rounds = 5u32;
    for i in 0..rounds {
        worker.put_chunks(vec![
            (key(0, 0, i as i32, 0), patterned_chunk()),
            (key(2, 1, -(i as i32), 7), Chunk::air()),
        ]);
        worker.append_checkpoint(Tick(u64::from(i) * 10), ramp_blob(64 + i as usize));
    }

    // 一次 flush:两类操作都进屏障(README 决策 5:排空此前全部操作)。
    let ack = worker.flush().recv().expect("flush ack 必达");
    assert!(ack.is_ok(), "一次 flush 两类操作都落库");
    worker.shutdown().expect("正常路径 shutdown = Ok");

    // 重开两库:chunk 齐全。
    let chunks = open_store(&chunks_path);
    assert_eq!(
        chunks.len().unwrap(),
        (rounds * 2) as usize,
        "chunk 键数齐全"
    );
    for i in 0..rounds {
        assert!(
            chunks.get(key(0, 0, i as i32, 0)).unwrap().is_some(),
            "第 {i} 轮 chunk a 在库"
        );
        assert!(
            chunks.get(key(2, 1, -(i as i32), 7)).unwrap().is_some(),
            "第 {i} 轮 chunk b 在库"
        );
    }
    // checkpoint 齐全且 read_latest = 最后投递的一条。
    let cps = open_store(&cp_path);
    for i in 0..rounds {
        let id = CheckpointId(u64::from(i) + 1);
        let (tick, blob) = cps
            .read(id)
            .unwrap()
            .unwrap_or_else(|| panic!("第 {i} 条检查点(id {id:?})在库"));
        assert_eq!(tick, Tick(u64::from(i) * 10), "id {id:?} 的 tick");
        assert_eq!(blob, ramp_blob(64 + i as usize), "id {id:?} 的 blob");
    }
    let (tick, blob) = cps.read_latest().unwrap().expect("非空必有最新");
    assert_eq!(
        tick,
        Tick(u64::from(rounds - 1) * 10),
        "read_latest = 最后一条"
    );
    assert_eq!(blob, ramp_blob(64 + (rounds - 1) as usize));
}

/// 检查点顺序跨重开:三次 append(递增 tick)→ 重开 → read_latest 为最大 tick;
/// read(1..=3) 各得其所。
#[test]
fn checkpoint_ordering_across_reopen() {
    let (_dir, path) = temp_db();

    {
        let mut store = open_store(&path);
        // 严格递增 tick;blob 按序号可区分(小尺寸,读回快)。
        for i in 0..3u64 {
            let id = store
                .append(Tick(100 + i * 10), format!("cp#{i}").as_bytes())
                .expect("append");
            assert_eq!(id, CheckpointId(i + 1), "append 返回自增 id");
        }
    }
    let store = open_store(&path);

    // read_latest = 最大 tick 的那条(第 3 条)。
    let (tick, blob) = store.read_latest().unwrap().expect("非空必有最新");
    assert_eq!(tick, Tick(120), "read_latest 为最大 tick");
    assert_eq!(blob, b"cp#2".to_vec());

    // read(1..=3) 各得其所。
    for i in 0..3u64 {
        let (got_tick, got_blob) = store
            .read(CheckpointId(i + 1))
            .unwrap()
            .unwrap_or_else(|| panic!("id={} 应在库", i + 1));
        assert_eq!(got_tick, Tick(100 + i * 10), "id={} 的 tick", i + 1);
        assert_eq!(
            got_blob,
            format!("cp#{i}").into_bytes(),
            "id={} 的 blob",
            i + 1
        );
    }
    // 第 4 条从未写入 → None(顺序读不越界)。
    assert_eq!(store.read(CheckpointId(4)).unwrap(), None);
}
