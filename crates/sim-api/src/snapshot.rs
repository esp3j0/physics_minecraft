//! GlobalSnapshot:tick + 每分片不透明字节。

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use types::{ShardId, Tick};

/// 全局快照:拍号 + 每分片不透明字节(ShardId 升序,ownership ≡ 键集,README 决策 3)。
/// 分片字节由 `ShardSim::snapshot` 产出,对本 crate 不透明(不自解、不重组)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GlobalSnapshot {
    pub tick: Tick,
    pub per_shard: BTreeMap<ShardId, Vec<u8>>,
}

impl GlobalSnapshot {
    /// 无分片载荷的空快照(拍号即 checkpoint 拍)。
    pub fn new(tick: Tick) -> Self {
        Self {
            tick,
            per_shard: BTreeMap::new(),
        }
    }

    /// 塞入一个分片的快照字节(同分片重塞 = 覆盖;BTreeMap 键序即 ShardId 升序)。
    pub fn insert_shard(&mut self, shard: ShardId, bytes: Vec<u8>) {
        self.per_shard.insert(shard, bytes);
    }

    /// bincode 编码(standard 配置;全字段可序列化,不可能失败 → fail-stop)。
    pub fn to_bytes(&self) -> Vec<u8> {
        bincode::serde::encode_to_vec(self, bincode::config::standard())
            .expect("快照编码不可能失败(全字段可序列化,fail-stop)")
    }

    /// bincode 解码(standard 配置;损坏 → Err(SnapshotError::Decode))。
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, SnapshotError> {
        let (snap, _len): (Self, usize) =
            bincode::serde::decode_from_slice(bytes, bincode::config::standard())
                .map_err(|e| SnapshotError::Decode(e.to_string()))?;
        Ok(snap)
    }
}

/// 快照层可预期失败:残缺/错误字节(一致性违规仍走 panic,蓝图 §4)。
#[derive(Debug, thiserror::Error)]
pub enum SnapshotError {
    #[error("快照解码失败:{0}")]
    Decode(String),
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::{DimensionId, ShardPos, WorldId};

    /// 测试用分片身份(固定值,不引入随机;与 shard 测试同风格)。
    fn shard_id(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    /// 测试清单 4:乱序 insert 三个分片 → per_shard 迭代为 ShardId 升序。
    #[test]
    fn snapshot_insert_and_ordering() {
        let mut snap = GlobalSnapshot::new(Tick(42));
        let a = shard_id(0, 0);
        let b = shard_id(0, 1);
        let c = shard_id(1, 0);
        snap.insert_shard(c, vec![3]);
        snap.insert_shard(a, vec![1]);
        snap.insert_shard(b, vec![2]);
        assert_eq!(snap.tick, Tick(42));
        let order: Vec<ShardId> = snap.per_shard.keys().copied().collect();
        assert_eq!(
            order,
            vec![a, b, c],
            "BTreeMap 键序 = ShardId 升序(Ord 按 world→dim→pos.x→pos.z)"
        );
    }

    /// 测试清单 5:to_bytes/from_bytes 相等;同构造两次 to_bytes 字节相等。
    #[test]
    fn snapshot_bytes_roundtrip() {
        let mut snap = GlobalSnapshot::new(Tick(7));
        snap.insert_shard(shard_id(0, 0), vec![1, 2, 3]);
        snap.insert_shard(shard_id(1, 0), vec![9]);

        let bytes = snap.to_bytes();
        let decoded = GlobalSnapshot::from_bytes(&bytes).expect("自有编码必可解码");
        assert_eq!(decoded, snap, "往返必须逐字段相等");

        // snapshot_bytes_are_stable:同构造两次 → 字节相等(确定性,细则 3)。
        let mut again = GlobalSnapshot::new(Tick(7));
        again.insert_shard(shard_id(0, 0), vec![1, 2, 3]);
        again.insert_shard(shard_id(1, 0), vec![9]);
        assert_eq!(snap.to_bytes(), again.to_bytes(), "同输入同字节");
    }

    /// 损坏字节 → Err(Decode),不 panic(可预期失败走类型化错误)。
    /// 截断必触发解码错误(比逐字节翻转更稳:变长编码可能容忍个别字节翻转)。
    #[test]
    fn snapshot_corrupt_bytes_error() {
        let mut snap = GlobalSnapshot::new(Tick(7));
        snap.insert_shard(shard_id(0, 0), vec![1, 2, 3]);
        let bytes = snap.to_bytes();
        let truncated = &bytes[..bytes.len() - 1];
        assert!(
            matches!(
                GlobalSnapshot::from_bytes(truncated),
                Err(SnapshotError::Decode(_))
            ),
            "截断字节必须返回 Err(Decode)"
        );
        // 逐字节翻转的首字节若恰好合法,也必须被后续结构校验拦下(永不 panic):
        let flipped: Vec<u8> = bytes
            .iter()
            .enumerate()
            .map(|(i, b)| if i == 0 { b ^ 0xFF } else { *b })
            .collect();
        if let Ok(snap) = GlobalSnapshot::from_bytes(&flipped) {
            let _ = snap; // 解码"成功"也不 panic——由截断分支兜底断言类型化错误。
        }
    }

    /// 测试清单 6:空快照也过序列化往返。
    #[test]
    fn empty_log_and_snapshot_roundtrip() {
        let snap = GlobalSnapshot::new(Tick(0));
        assert!(snap.per_shard.is_empty());
        let decoded = GlobalSnapshot::from_bytes(&snap.to_bytes()).expect("空快照必可解码");
        assert_eq!(decoded, snap, "空快照往返后仍为空");
    }
}
