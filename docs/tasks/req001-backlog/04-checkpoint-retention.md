# 任务 04:checkpoint 保留最近 N 个 + v5 压缩(#17)

- 状态:**已完成(2026-10-25,总架构师验收通过;实现 [b51fb24](https://github.com/esp3j0/physics_minecraft/commit/b51fb24))**;用户已拍板 D1–D4 全部按推荐。
  实测(真实服务器,默认配置,约 3.6 分钟、21 个 checkpoint):单个约 33 KB(修复前约 2.66 MiB),库中恰好 10 个,
  `checkpoints.redb` 稳定在 0.78 MiB。
- issue:[#17](https://github.com/esp3j0/physics_minecraft/issues/17)(实测数据与方案比较见 issue 正文)。
- 需求:[REQ-001](../../requirements/REQ-001-architecture.md) §9 决议 11 的 #17 修订;
  [REQ-005 决策清单](../../requirements/REQ-005-decisions.md) D8 版本记录 v5。
- 性质:REQ-001/REQ-005 的 follow-up(同 #12 的做法),不单独立 REQ。

## 背景

`checkpoints.redb` 只追加不删除:每 10 s 一个约 2.66 MiB 的 checkpoint(98.8% 是体素,
几乎全是可压缩的空气),文件按 2 的幂扩张,约 2 GiB/小时。两处互补的修复:

- **A 保留最近 N 个**:文件有界;
- **B lz4 压缩**:单个约 2.66 MiB → 约 22 KB(平地),写盘量降 25–120 倍。

## 拍板(2026-10-25 用户)

| ID | 决定 |
| --- | --- |
| D1 | 默认保留 10 个(回放窗口 90 s);`checkpoint_keep = 0` 不清理 |
| D2 | app 层 `lz4_flex` 压缩,`CHECKPOINT_FORMAT_VERSION` 4 → 5 |
| D3 | 不调用 `compact()`(删旧后空闲页复用,文件有界即可) |
| D4 | 作为 REQ-001/REQ-005 的 follow-up,按本任务书执行 |

## 总架构师已提供

接口桩(签名固定,不得修改):

- `storage::redb::RedbStore::open_with_keep(path, keep)`;
- `storage::mem::MemCheckpointStore::with_keep(keep)`;
- `app::driver::encode_checkpoint` / `decode_checkpoint`(当前委托 v4 编码,由你改成 v5);
- `app::config::Config::checkpoint_keep`(字段与默认值 10 已加,未接线)。

验收用例 `crates/e2e/tests/issue17_acceptance.rs`(6 条,`#[ignore]`):

| 用例 | 内容 |
| --- | --- |
| `acc_k1_k3_keep_last_n` | redb 与内存两种存储保留最近 N 个;`keep = 0` 与 `MemCheckpointStore::new()` 不清理 |
| `acc_k2_ids_monotonic_and_prune_on_open` | 清理后重开库 ID 继续单调不复用;打开时现存多于 keep 个立即删到 keep 个 |
| `acc_k5_file_size_bounded` | 1 MiB × 150 次、keep = 4,文件 ≤ 32 MiB |
| `acc_z2_v5_codec_and_old_versions_rejected` | 版本为 5;v5 往返无损且确实压缩;v4 档报 `unsupported checkpoint format version 4`;乱码/空字节报错不 panic |
| `acc_c1_checkpoint_keep_config` | 配置默认 10、可覆写、负数报错(字段已加,本条现在就通过,作为防回归) |
| `acc_k4_z1_production_path_retention_and_compression` | 真驱动 + 真 redb,间隔 60 拍、keep = 3:库中恰好剩最近 3 个,最新一个 ≤ 64 KB 且为 v5,同目录重新 bootstrap 从最新的续跑 |

另外,三个旧验收文件里直接用 bincode 解 checkpoint 的辅助函数已改为调用 `decode_checkpoint`
(`crates/app/tests/app_e2e.rs`、`crates/e2e/tests/consistency.rs`、`crates/e2e/tests/world_loading.rs`;
consistency 的 AC-C9 改为对拍 `encode_checkpoint` 的字节),断言未动。

## 范围

1. **storage**:实现 `open_with_keep`、`with_keep`。删旧与插入新行在**同一个写事务**里完成;
   打开时按 keep 清理;`open` / `new` 语义不变(不清理)。
2. **app 编解码**:`encode_checkpoint` / `decode_checkpoint` 实现 v5——blob = 未压缩的
   `format_version` 前缀(bincode standard 的 u32 varint)+ `lz4_flex::compress_prepend_size(...)`;
   解码先读前缀,不是 5 → `AppError::Decode("unsupported checkpoint format version N")`,是 5 才解压、
   解码;任何解压/解码失败 → `AppError::Decode`,不 panic。驱动内部的 `encode` / `decode` 与 bootstrap 的
   读取全部改走这两个公开入口(只留一份编码)。`CHECKPOINT_FORMAT_VERSION` 改为 5,其文档注释追加
   v5 一段(与 REQ-005 D8 版本记录一致)。
3. **装配**:bootstrap 用 `RedbStore::open_with_keep(&checkpoints_path, config.checkpoint_keep)` 打开
   checkpoint 库;chunk 库照旧用 `open`。
4. **配置**:`checkpoint_keep` 接入 toml 读取(键名 `checkpoint_keep`)、`server.example.toml`(若有)与配置
   文档注释;`checkpoint_keep` 无上限,0 合法。
5. **依赖**:`crates/app/Cargo.toml` 加 `lz4_flex`(纯 Rust,默认 feature 即可)。
6. **旧测试**:`driver.rs` 单元测试里钉死 `CHECKPOINT_FORMAT_VERSION == 4` 的断言改为 5(版本升级本身
   就是需求变更,这一处授权修改);其他已有测试的断言不得修改。

## 不做

- 不调用 `compact()`(D3);
- 不做 chunk palette/RLE 编码(issue 的 C2,另立 REQ);
- 不改命令日志的分段与保存方式;
- 不写 v4 → v5 迁移工具(D8:旧库显式拒绝,清空 `data_dir`)。

## 预判风险

1. **ID 恢复**:`RedbStore` 在 open 时把 `next_checkpoint_id` 恢复为表内最大键;清理后最大键仍在
   (最新一个永不删),所以只要别改成按行数恢复就不会复用。打开时清理不能把 ID 计数器弄乱。
2. **同一事务**:插入与删除必须在一个 `begin_write` / `commit` 里;分两个事务时中途崩溃会留下
   多于 keep 个或少于 keep 个。删除用范围删除(`id ≤ new_id − keep`),不要逐个 `remove` 全表扫描。
3. **`keep` 边界**:`new_id ≤ keep` 时不删;`keep = 1` 只留最新一个;减法不要下溢。
4. **解码顺序**:必须先读版本前缀再解压。先解压会让 v4 档报"解压失败"而不是版本错误(Z2 会红)。
5. **确定性**:同一 bundle 两次编码逐字节相等(consistency 的 AC-C9 现在对拍 `encode_checkpoint`)。
   lz4_flex 是确定的,但不要在编码里混入时间戳、随机数或 HashMap 迭代。
6. **tick 线程开销**:压缩在 tick 线程上做,平地约 0.2 ms;不要为此引入线程或异步。
7. **磁盘满等任务范围外的问题**:停下报告,不要删除文件。

## 门禁

- `cargo test --workspace --all-targets` 退出码 0;
- `cargo clippy --workspace --all-targets -- -D warnings` 退出码 0;
- `cargo fmt --all -- --check` 退出码 0;
- 验收用例只允许删除 `#[ignore]` 行;接口签名不变;只改本任务书范围内的文件。

## 回评要求(在 #17)

CI 全绿链接;6 条 `acc_*` 在 CI 日志中 `ok` 的行;7 条风险逐条说明处理方式。完成后停,由总架构师
验收(真实服务器长跑的文件大小由总架构师实测)。
