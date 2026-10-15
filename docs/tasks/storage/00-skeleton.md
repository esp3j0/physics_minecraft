# 任务 00:storage crate 骨架

## 目标

crate `storage` 目前只有 lib.rs 文档注释。建立编译通过的模块骨架:依赖声明 +
4 个模块文件(仅文档注释),不写任何逻辑与测试。

## 必读

- `docs/design/codebase-layout.md` §2(依赖表)与 §3 storage 图纸
- `crates/storage/src/lib.rs`(现有文档注释,保留)
- `crates/shard/Cargo.toml`(依赖声明风格参照)

## 交付物

1. `crates/storage/Cargo.toml`:

   ```toml
   [dependencies]
   types = { path = "../types" }
   voxel = { path = "../voxel" }
   redb = "4"
   bincode = "3"
   thiserror = "2"

   [dev-dependencies]
   tempfile = "3"

   [lints]
   workspace = true
   ```

   (`[lints]` 已存在则保留;dev-dependencies 供后续测试用临时目录。)

2. `crates/storage/src/lib.rs`:保留现有 `//!` 文档,追加模块声明:

   ```rust
   pub mod mem;
   pub mod redb;
   pub mod traits;
   pub mod worker;
   ```

3. 新建 4 个文件,各只含一行 `//!` 文档注释(照蓝图 §3 对应行):

   - `src/traits.rs` —— ChunkStore(put_batch/get)与 CheckpointStore(append/read);
     一切接口返回 pending-ack 供驱动层做卸载屏障
   - `src/mem.rs` —— 内存实现(测试/回放)
   - `src/redb.rs` —— redb 实现(key=(world,dim,chunk_pos))
   - `src/worker.rs` —— 单写 worker:批量缓冲、显式 flush→oneshot ack(v2 持久化屏障)

## 完成定义

- `cargo test -p storage --all-targets` 绿(0 个测试);
- `cargo clippy -p storage --all-targets -- -D warnings` 绿;
- 不改动本任务书未列出的任何文件。
