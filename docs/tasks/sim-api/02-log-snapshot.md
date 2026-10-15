# 任务 02:命令日志与全局快照(数据结构)

## 目标

AC8 的两件数据:外部命令日志(重放的输入)与全局快照(checkpoint 的载荷)。
纯数据结构 + bincode 序列化,不含任何执行逻辑。

## 必读

- `crates/messaging/src/command.rs`(SimCommand/InputSource——LogEntry 的字段来源)
- `crates/shard/src/scheduler.rs`(snapshot_shard——GlobalSnapshot 的字节来源)
- `crates/storage/src/traits.rs`(CheckpointStore——快照 blob 的去处)
- `docs/tasks/sim-api/README.md` 决策 3/4

## 交付物

`crates/sim-api/src/log.rs` 与 `crates/sim-api/src/snapshot.rs`,各含类型 + 内嵌测试。

## 接口设计(签名固定)

`log.rs`:

```rust
/// 一条外部命令日志记录 = 盖章输入的完整复原信息(README 决策 4)。
/// 信封/事件/迁移不落日志(蓝图 §3):它们由分片间交互在重放中自然重生成。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LogEntry {
    pub tick: Tick,
    pub dest: ShardId,
    pub source: InputSource,
    pub sequence: u64,
    pub command: SimCommand,
}

/// 外部命令日志:按 tick 升序追加(按 tick 分段即条目序,整块 bincode)。
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct CommandLog {
    entries: Vec<LogEntry>,
}

impl CommandLog {
    pub fn new() -> Self;
    /// 追加(调用方保证 tick 单调不减;倒退 → panic,fail-stop)。
    pub fn append(&mut self, entry: LogEntry);
    pub fn entries(&self) -> &[LogEntry];
    pub fn len(&self) -> usize;
    pub fn is_empty(&self) -> bool;
    /// 取出全部(清空自己;回放后复用同一日志对象时用)。
    pub fn take(&mut self) -> Vec<LogEntry>;
    /// bincode 编/解码(standard 配置;损坏 → Err)。
    pub fn to_bytes(&self) -> Vec<u8>;
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, LogError>;
}

#[derive(Debug, thiserror::Error)]
pub enum LogError {
    #[error("日志解码失败:{0}")]
    Decode(String),
}
```

`snapshot.rs`:

```rust
/// 全局快照:拍号 + 每分片不透明字节(ShardId 升序,ownership ≡ 键集,决策 3)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GlobalSnapshot {
    pub tick: Tick,
    pub per_shard: BTreeMap<ShardId, Vec<u8>>,
}

impl GlobalSnapshot {
    pub fn new(tick: Tick) -> Self;
    pub fn insert_shard(&mut self, shard: ShardId, bytes: Vec<u8>);
    pub fn to_bytes(&self) -> Vec<u8>;
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, SnapshotError>;
}

#[derive(Debug, thiserror::Error)]
pub enum SnapshotError {
    #[error("快照解码失败:{0}")]
    Decode(String),
}
```

## 语义细则

1. bincode 2 + standard 配置(仓库统一);
2. `CommandLog::append` 的 tick 单调约束:`entry.tick >= 最后一条.tick`
   (相等合法——同拍多条命令);
3. 字段/容器序确定性:BTreeMap + Vec,serde 输出稳定(同输入同字节)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `log_append_keeps_order_and_rejects_tick_regression` | 追加三条(tick 5,5,7)有序;tick 3 → panic |
| `log_bytes_roundtrip` | to_bytes → from_bytes 相等;损坏字节 → Err |
| `log_take_empties` | take 后 is_empty |
| `snapshot_insert_and_ordering` | 乱序 insert 三个分片 → per_shard 迭代为 ShardId 升序 |
| `snapshot_bytes_roundtrip` | to_bytes/from_bytes 相等;`snapshot_bytes_are_stable`:同构造两次 to_bytes 字节相等 |
| `empty_log_and_snapshot_roundtrip` | 空对象也过序列化往返 |

## 门禁包

`-p sim-api`。

## 完成定义

见 [README 公共完成定义](README.md)。
