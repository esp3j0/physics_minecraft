# 任务 02:OwnershipMap(分片所有权表)

## 目标

蓝图 §3:"OwnershipMap:claim/release/owner(BTreeMap;重复 claim 报错)——照 v2
ownership.rs"。AC5(同分片同 tick 至多一 worker)的结构保证:分片状态本身经通道在
worker 间转移,本表是"哪些分片当前被系统持有"的唯一事实。

## 必读

- `crates/types/src/ids.rs`(`ShardId`/`ShardPos`,注意 `ShardId` 已实现 Ord)
- `crates/shard/src/ownership.rs`(骨架)
- `docs/design/codebase-layout.md` §4(错误处理:一致性违规 panic)

## 交付物

`crates/shard/src/ownership.rs`:类型 + 内嵌单元测试。

## 接口设计(签名固定)

```rust
/// 一次 claim 的记录。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClaimRecord {
    /// 声明发生的拍。
    pub since_tick: Tick,
}

/// 分片所有权表:哪些分片当前被系统持有(BTreeMap,确定性迭代)。
/// 重复 claim / 释放未持有 = 一致性违规 → panic(fail-stop,蓝图 §4)。
#[derive(Debug, Default)]
pub struct OwnershipMap {
    claims: BTreeMap<ShardId, ClaimRecord>,
}

impl OwnershipMap {
    pub fn new() -> Self;

    /// 声明所有权。已持有 → panic。
    pub fn claim(&mut self, shard: ShardId, at: Tick);

    /// 释放所有权。未持有 → panic。
    pub fn release(&mut self, shard: ShardId);

    /// 查询(未持有返回 None;查询永不 panic)。
    pub fn owner(&self, shard: ShardId) -> Option<ClaimRecord>;

    pub fn contains(&self, shard: ShardId) -> bool;
    pub fn len(&self) -> usize;
    pub fn is_empty(&self) -> bool;

    /// 确定性迭代(ShardId 升序,BTreeMap 保证)。
    pub fn iter(&self) -> impl Iterator<Item = (ShardId, ClaimRecord)> + '_;
}
```

panic 消息用中文、含分片信息(如 `重复 claim:`),便于定位。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `claim_then_query_roundtrip` | claim 后 `owner()` 返回 `since_tick` 正确的记录 |
| `duplicate_claim_panics` | `#[should_panic]` 二次 claim |
| `release_unclaimed_panics` | `#[should_panic]` 释放未持有的分片 |
| `release_allows_reclaim` | release 后可重新 claim,`since_tick` 更新 |
| `iteration_is_deterministic` | 乱序插入 3 个分片,`iter()` 输出按 `ShardId` 升序 |
| `len_tracks_claims` | claim/release 后 `len`/`is_empty` 正确 |

## 完成定义

见 [README 公共完成定义](README.md)。
