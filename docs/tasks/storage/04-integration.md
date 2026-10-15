# 任务 04:持久化集成测试(往返/后端等价/屏障)

## 目标

crate 级行为测试:把 trait 契约、两个后端与 worker 串起来验证。本任务由
**与实现不同的模型**(qwen)执笔——测试与实现交叉,是对任务书的独立解读。

## 必读

- `docs/tasks/storage/README.md`(全部决策)
- 任务书 01-03(测试的是它们的契约)
- `crates/storage/src/traits.rs`、`mem.rs`、`redb.rs`、`worker.rs`(以源码为准)
- `crates/shard/tests/two_shards.rs`(本仓库集成测试风格参照)

## 交付物

`crates/storage/tests/persistence.rs`。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `mem_and_redb_backends_agree` | 同一批数据分别写 MemChunkStore 与 RedbStore → 逐键 get 相等(等价性:两边序列化路径一致);len 相等 |
| `chunk_roundtrip_preserves_content` | 非平凡 chunk(若干层方块)经 redb 落盘重开后逐坐标一致(用 Chunk::get 抽查 ≥ 20 个点,含边界) |
| `checkpoint_roundtrip_preserves_blob` | 任意 bytes blob(含空、含 1MB 大块)经 append → 重开 → read 逐字节相等 |
| `worker_flush_is_the_durability_barrier` | worker + redb:投递后 **flush ack Ok** → shutdown → 重开 → 数据齐全(屏障保证的方向:ack 后必可见) |
| `worker_mixed_ops_all_durable_after_flush` | chunk 与 checkpoint 交错投递 → 一次 flush → 重开 → 两类数据都齐全、read_latest 正确 |
| `checkpoint_ordering_across_reopen` | 三次 append(递增 tick)→ 重开 → read_latest 为最大 tick;read(1..=3) 各得其所 |

工程细节:每个测试用独立 `tempfile::tempdir()`;两个 store 不开同一文件;
大 blob 用 `vec![b; 1 << 20]`(内容非全零,写入递增字节)。

## 完成定义

见 [README 公共完成定义](README.md)。
