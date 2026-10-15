# 任务 02:redb 持久化后端

## 目标

`redb.rs`:两个 trait 的 redb 4 单文件数据库实现。真落盘、可重开——持久化的
"真实含义"由本模块兑现(内存实现只是测试替身)。

## 必读

- `crates/storage/src/traits.rs`(任务 01 产出)
- `docs/tasks/storage/README.md` 决策 3/4/7/9
- redb 4 文档:https://docs.rs/redb 4.x(表定义、`Database::create`、读写事务、
  元组 key;以文档与编译器为准,本任务书片段仅示意)

## 授权改动

- 若发现 voxel `Chunk` 需要 `Clone` 才能从存储层返回,允许给 `Chunk` 补
  `derive(Clone)`(仅此一行);`PartialEq` 同理可补。除此之外不得改其它 crate。

## 交付物

`crates/storage/src/redb.rs`:类型 + 内嵌单元测试(测试用 `tempfile::tempdir()` 开
独立临时库;注意同一路径不能被两个 Database 实例同时打开,测试各自用独立目录)。

## 接口设计(签名固定)

```rust
/// 打开(或创建)一个 chunk + checkpoint 合一的单文件存储。
pub struct RedbStore { /* Database + 表定义,私有 */ }

impl RedbStore {
    /// 打开/创建数据库文件。路径由调用方给定(app 配置)。
    pub fn open(path: &Path) -> Result<Self, StoreError>;
}

impl ChunkStore for RedbStore { /* put_batch / get / len / is_empty */ }
impl CheckpointStore for RedbStore { /* append / read_latest / read */ }
```

## 语义细则

1. **表设计(决策 4)**:
   - chunk 表:key = `(u32, u32, i32, i32)`(world, dim, x, z),value = `&[u8]`
     = bincode(Chunk)(`bincode::serde::encode_to_vec` + `config::standard()`,
     决策 3;读回 `decode_from_slice`);
   - checkpoint 表:key = `u64`(CheckpointId),value 编码 `(tick: u64, blob)`——
     用 bincode 编码成单段 bytes,或 redb 原生元组 value,以能表达语义且测试过为准;
2. **put_batch = 一个写事务**(redb 事务天然原子,last-wins 由事务内顺序写入得到);
3. **checkpoint 计数器**:`open` 时从表现有行数初始化,append 时 +1,重启后不回退、
   不复用(重开旧库再 append,ID 继续增长);
4. **len** = chunk 表行数(redb 读事务 count;若 count API 不可用则全表迭代计数,
   注明取舍);
5. redb/bincode 的错误分别包进 `StoreError::Db`/`Decode`(traits.rs 已定);
   `Database::create` 的 IO 错误包 `Io`;
6. 表名用 `&'static str` 常量("chunks"/"checkpoints"),不得泄漏到 crate 外。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `redb_roundtrip_after_reopen` | 临时库:put 2 个 chunk → drop(store) → 重新 open 同路径 → get 全部回来(数据与 serde 字节等价) |
| `redb_missing_returns_none` | 未写过的键(含同坐标不同 world/dim)→ None |
| `redb_world_dim_do_not_collide` | 同 ChunkPos 不同 world/dim 三个键互不覆盖 |
| `redb_batch_is_atomic_last_wins` | 同批重复键 → 后值生效(重开后验证) |
| `redb_checkpoint_ids_continue_after_reopen` | 追加 2 条 → drop → 重开 → 再追加 → ID 为 3(计数器不回退) |
| `redb_checkpoint_latest_survives_reopen` | 追加 3 条 → drop → 重开 → read_latest 为第 3 条;read(1) 为第 1 条 |
| `redb_store_is_send` | 编译期 `assert_send::<RedbStore>()` |

## 完成定义

见 [README 公共完成定义](README.md)。
