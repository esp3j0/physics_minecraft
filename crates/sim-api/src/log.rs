//! 外部命令日志:按 tick 分段(bincode);信封/事件/迁移不落日志。

use messaging::{InputSource, SimCommand};
use serde::{Deserialize, Serialize};
use types::{ShardId, Tick};

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
    /// 空日志。
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    /// 追加(调用方保证 tick 单调不减;倒退 → panic,fail-stop)。
    /// 相等合法——同拍多条命令(语义细则 2)。先查后插:panic 时日志不变。
    pub fn append(&mut self, entry: LogEntry) {
        if let Some(last) = self.entries.last()
            && entry.tick < last.tick
        {
            panic!(
                "日志 tick 单调约束被破坏:新条目 tick {} < 末条 tick {}(调用方 bug,fail-stop)",
                entry.tick.0, last.tick.0
            );
        }
        self.entries.push(entry);
    }

    /// 全部条目(追加序 = tick 升序)。
    pub fn entries(&self) -> &[LogEntry] {
        &self.entries
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// 取出全部(清空自己;回放后复用同一日志对象时用)。
    pub fn take(&mut self) -> Vec<LogEntry> {
        std::mem::take(&mut self.entries)
    }

    /// bincode 编码(standard 配置;全字段可序列化,不可能失败 → fail-stop)。
    pub fn to_bytes(&self) -> Vec<u8> {
        bincode::serde::encode_to_vec(self, bincode::config::standard())
            .expect("日志编码不可能失败(全字段可序列化,fail-stop)")
    }

    /// bincode 解码(standard 配置;损坏 → Err(LogError::Decode))。
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, LogError> {
        let (log, _len): (Self, usize) =
            bincode::serde::decode_from_slice(bytes, bincode::config::standard())
                .map_err(|e| LogError::Decode(e.to_string()))?;
        Ok(log)
    }
}

/// 日志层可预期失败:残缺/错误字节(一致性违规仍走 panic,蓝图 §4)。
#[derive(Debug, thiserror::Error)]
pub enum LogError {
    #[error("日志解码失败:{0}")]
    Decode(String),
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::{BlockPos, DimensionId, ShardPos, WorldId};

    /// 测试用分片身份(固定值,不引入随机;与 shard 测试同风格)。
    fn shard_id(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    /// 一条日志记录(参数 = tick/sequence,其余固定)。
    fn entry(tick: u64, sequence: u64) -> LogEntry {
        LogEntry {
            tick: Tick(tick),
            dest: shard_id(0, 0),
            source: InputSource(1),
            sequence,
            command: SimCommand::SetBlock {
                pos: BlockPos::new(1, 2, 3),
                block: types::BlockId(1),
            },
        }
    }

    /// 测试清单 1:追加三条(tick 5,5,7)有序;tick 3 → panic。
    #[test]
    fn log_append_keeps_order_and_rejects_tick_regression() {
        let mut log = CommandLog::new();
        log.append(entry(5, 0));
        log.append(entry(5, 1));
        log.append(entry(7, 2));
        let ticks: Vec<u64> = log.entries().iter().map(|e| e.tick.0).collect();
        assert_eq!(ticks, vec![5, 5, 7], "追加序即存储序(同拍合法)");
    }

    /// tick 倒退 → panic(fail-stop,语义细则 2)。
    #[test]
    #[should_panic(expected = "tick 单调")]
    fn log_append_tick_regression_panics() {
        let mut log = CommandLog::new();
        log.append(entry(5, 0));
        log.append(entry(3, 1)); // 3 < 5 → fail-stop
    }

    /// 测试清单 2:to_bytes → from_bytes 相等;损坏字节 → Err。
    #[test]
    fn log_bytes_roundtrip() {
        let mut log = CommandLog::new();
        log.append(entry(5, 0));
        log.append(entry(7, 1));
        let bytes = log.to_bytes();
        let decoded = CommandLog::from_bytes(&bytes).expect("自有编码必可解码");
        assert_eq!(decoded, log, "往返必须逐字段相等");

        let damaged: Vec<u8> = bytes
            .iter()
            .enumerate()
            .map(|(i, b)| if i == 0 { b ^ 0xFF } else { *b })
            .collect();
        assert!(
            matches!(CommandLog::from_bytes(&damaged), Err(LogError::Decode(_))),
            "损坏字节必须返回 Err(Decode),不 panic"
        );
    }

    /// 测试清单 3:take 后 is_empty。
    #[test]
    fn log_take_empties() {
        let mut log = CommandLog::new();
        log.append(entry(5, 0));
        log.append(entry(6, 1));
        assert_eq!(log.len(), 2);
        let taken = log.take();
        assert_eq!(taken.len(), 2, "取出全部条目");
        assert!(log.is_empty(), "take 清空自己");
        assert_eq!(log.entries(), &[] as &[LogEntry]);
    }

    /// 测试清单 6:空日志也过序列化往返。
    #[test]
    fn empty_log_and_snapshot_roundtrip() {
        let log = CommandLog::new();
        assert!(log.is_empty());
        let decoded = CommandLog::from_bytes(&log.to_bytes()).expect("空日志必可解码");
        assert!(decoded.is_empty(), "空日志往返后仍为空");
    }
}
