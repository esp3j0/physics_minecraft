# 任务 01:存储 trait 契约 + 内存实现

## 目标

定义全项目持久化的唯一接口(蓝图 §3 traits.rs):`ChunkStore`(体素 chunk 的批量写/
单读)与 `CheckpointStore`(全局检查点的追加/读)。shard/sim-api 只面向这两个 trait,
redb 与内存实现可互换(测试/回放用内存,生产用 redb)。同时交付内存实现。

## 必读

- `crates/voxel/src/chunk.rs`(`Chunk`:serde derive 已有,get/set/air)
- `crates/types/src/ids.rs`、`crates/types/src/tick.rs`
- `docs/design/codebase-layout.md` §3 storage、§4(错误处理)
- `docs/tasks/storage/README.md` 决策 2/5/7/9
- `crates/messaging/src/envelope.rs` 的错误定义风格(thiserror 参照)

## 交付物

`crates/storage/src/traits.rs` 与 `crates/storage/src/mem.rs`,各含类型 + 内嵌单元测试。

## 接口设计(签名固定)

`traits.rs`:

```rust
/// chunk 的存储键:世界 + 维度 + chunk 坐标(README 决策 4)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ChunkKey {
    pub world: WorldId,
    pub dim: DimensionId,
    pub pos: ChunkPos,
}

/// 检查点 ID(redb 自增主键;从 1 起,0 永不有效)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct CheckpointId(pub u64);

/// 存储层错误(类型化;IO 属可预期失败,不上 panic)。
#[derive(Debug, thiserror::Error)]
pub enum StoreError {
    #[error("IO 错误:{0}")]
    Io(#[from] std::io::Error),
    #[error("数据库错误:{0}")]
    Db(String),
    #[error("解码失败:{0}")]
    Decode(String),
}

/// 体素 chunk 存储(蓝图片面最小集:批量写、单读、计数)。
/// `Send`:单写 worker 线程持有 `Box<dyn ChunkStore>`(README 决策 2)。
pub trait ChunkStore: Send {
    /// 原子批量写;同批重复键 = 后写覆盖(last-wins,决策 7)。
    fn put_batch(&mut self, batch: &[(ChunkKey, Chunk)]) -> Result<(), StoreError>;
    /// 未存 → Ok(None)。
    fn get(&self, key: ChunkKey) -> Result<Option<Chunk>, StoreError>;
    fn len(&self) -> Result<usize, StoreError>;
    fn is_empty(&self) -> Result<bool, StoreError>;
}

/// 全局检查点存储:追加写、读最新/按 ID(blob 对本 crate 不透明,决策 3)。
pub trait CheckpointStore: Send {
    fn append(&mut self, at: Tick, blob: &[u8]) -> Result<CheckpointId, StoreError>;
    /// 最新一条(空库 → Ok(None))。
    fn read_latest(&self) -> Result<Option<(Tick, Vec<u8>)>, StoreError>;
    fn read(&self, id: CheckpointId) -> Result<Option<(Tick, Vec<u8>)>, StoreError>;
}
```

`mem.rs`:两个 BTreeMap 实现以上 trait(`MemChunkStore`/`MemCheckpointStore`,
`#[derive(Debug, Default)]` + `new()`);checkpoint 自增计数器从 1 起。注意 clippy
对 `new()` 的 Default 约定。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `mem_chunk_roundtrip` | put_batch 后 get 回相等 Chunk(voxel Chunk 有 PartialEq?没有则逐点 `get` 抽查或补 PartialEq——优先在测试内比较序列化字节);未存键 → None |
| `mem_batch_last_wins` | 同批两个相同键 → 后值生效;len 只 +1 |
| `mem_chunk_len_tracks` | 空 → is_empty;写 N 个不同键 → len == N |
| `mem_checkpoint_ids_monotonic_from_one` | 连续 append → 1、2、3 |
| `mem_checkpoint_read_latest_and_by_id` | 追加 3 条不同 tick;read_latest = 最后一条;read(2) = 第二条;read(99) → None |
| `mem_stores_are_send` | `fn assert_send<T: Send>() {}` 编译期断言两个实现都 Send |

若 voxel `Chunk` 未 derive `PartialEq`:在测试里用 `Chunk::get` 抽查若干坐标比较,
**不要**为本任务改 voxel(等价性断言由任务 04 在序列化层做)。

## 完成定义

见 [README 公共完成定义](README.md)。
