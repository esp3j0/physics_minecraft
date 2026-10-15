//! 存储唯一接口(蓝图 §3):`ChunkStore`(体素 chunk 的批量写/单读/计数)与
//! `CheckpointStore`(全局检查点的追加/读)。
//!
//! shard/sim-api 只面向这两个 trait;redb(`super::redb`)与内存(`super::mem`)
//! 实现可互换(测试/回放用内存,生产用 redb)。存储格式(bincode/redb 表结构)
//! 不在 trait 面上出现:chunk 值即 `voxel::Chunk`,检查点 blob 是 `&[u8]`
//! 对本 crate 不透明(README 决策 3/9)。

use types::{ChunkPos, DimensionId, Tick, WorldId};
use voxel::Chunk;

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

/// 存储层错误(类型化;IO 属可预期失败,不上 panic,蓝图 §4)。
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
    /// 未存 → `Ok(None)`。
    fn get(&self, key: ChunkKey) -> Result<Option<Chunk>, StoreError>;
    fn len(&self) -> Result<usize, StoreError>;
    fn is_empty(&self) -> Result<bool, StoreError>;
}

/// 全局检查点存储:追加写、读最新/按 ID(blob 对本 crate 不透明,决策 3)。
pub trait CheckpointStore: Send {
    fn append(&mut self, at: Tick, blob: &[u8]) -> Result<CheckpointId, StoreError>;
    /// 最新一条(空库 → `Ok(None)`)。
    fn read_latest(&self) -> Result<Option<(Tick, Vec<u8>)>, StoreError>;
    fn read(&self, id: CheckpointId) -> Result<Option<(Tick, Vec<u8>)>, StoreError>;
}
