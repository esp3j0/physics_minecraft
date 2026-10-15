# 任务 04:命令路由与信封盖章分发

## 目标

蓝图 §3:"命令路由:有界收件箱、满则断开权威输入(照 v2 enforce_input_delivery);
信封盖章与分发(tick N+1 输入前缀)"。Router 是调度器的输入面:本地命令走有界
收件箱(满 = 类型化错误,反压),跨片 Intent 经调度器盖章成 Envelope 入
`CrossShardQueue`,下一拍作为目的分片输入前缀分发。

## 必读

- `crates/messaging/src/envelope.rs`(`Intent`/`Envelope`/`Envelope::stamp`/
  `CrossShardQueue`/`AdmitError`/`DEFAULT_QUEUE_CAPACITY`——全部复用,不得重造)
- `crates/messaging/src/command.rs`(`StampedInput`/`InputSource::SCHEDULER`/
  `InputPayload`)
- `crates/messaging/src/ordering.rs`(`canonical_key()`——合并排序用)
- `crates/messaging/src/lib.rs`(re-export 的确切路径)
- `docs/design/codebase-layout.md` §3 router 行
- `docs/tasks/shard/README.md` 决策 1/3(收件箱容量 256;信封 source=SCHEDULER)

## 交付物

`crates/shard/src/router.rs`:类型 + 内嵌单元测试。依赖追加:
`serde` 暂不需要;`thiserror = "2"`(AdmitError 同款,参照 `crates/messaging/Cargo.toml` 的版本写法)。

## 接口设计(签名固定)

```rust
#[derive(Debug, Clone, Copy)]
pub struct RouterConfig {
    /// 每分片本地收件箱容量(默认 256,README 决策 1)。
    pub inbox_capacity: usize,
    /// 跨片信封队列容量(默认 `DEFAULT_QUEUE_CAPACITY` = 1024)。
    pub queue_capacity: usize,
}
// Default 按上述默认值实现。

/// 路由失败(可预期,类型化;反压语义:调用方断开该权威输入)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum RouteError {
    #[error("分片收件箱已满(容量 {capacity})")]
    InboxFull { capacity: usize },
}

pub struct Router {
    queue: CrossShardQueue,
    inboxes: BTreeMap<ShardId, VecDeque<StampedInput>>,
    /// 每个 source 分片的信封序号计数器(单调 +1,从 0 起)。
    sequences: BTreeMap<ShardId, u64>,
    cfg: RouterConfig,
}

impl Router {
    pub fn new(cfg: RouterConfig) -> Self;

    /// 本地权威命令入箱(有界;满即 Err,不驱逐已有条目)。
    pub fn route_local(&mut self, shard: ShardId, input: StampedInput) -> Result<(), RouteError>;

    /// tick 末:把源分片 Outbox 里的 Intent 逐个盖章入队。
    /// 序号按 source 严格单调;`AdmitError` 透传(已入队的不回滚,调用方处理)。
    pub fn collect_intents(
        &mut self,
        source: ShardId,
        intents: &[Intent],
        produced_at: Tick,
    ) -> Result<(), AdmitError>;

    /// tick 开:取 destination 在 apply_at 拍的信封批 + 收件箱中 apply_at 相符的命令,
    /// 合并为该拍输入前缀(canonical 序)。恰好一次:取走即移除。
    /// 未来拍的命令保留在收件箱,直到其 apply_at 到来。
    pub fn dispatch_prefix(&mut self, destination: ShardId, apply_at: Tick) -> Vec<StampedInput>;

    pub fn pending_inbox(&self, shard: ShardId) -> usize;
    pub fn pending_envelopes(&self) -> usize;
}
```

## 语义细则(确定性关键,逐条照做)

1. **盖章**:每个 intent 用 `Envelope::stamp(source, intent, produced_at, seq)`,
   `seq` 从该 source 的计数器取出后自增;
2. **信封→StampedInput**:`{ apply_at: e.apply_at, source: InputSource::SCHEDULER,
   sequence: e.source_sequence, payload: InputPayload::Envelope(e) }`(README 决策 3);
3. **合并排序**:命令与信封合并后 `sort_by_key(canonical_key)`(**稳定**排序;
   Rust `sort_by_key` 即稳定)。信封间相对序已由 `take_batch` 的 canonical 排序
   决定,稳定排序不会打乱;
4. **收件箱过滤**:只取 `apply_at == 目标拍` 的命令,其余原序保留在队列中
   (VecDeque 保序 split,不得整体清空)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `inbox_full_rejects_without_evicting` | 容量 2,第三条 `Err(InboxFull)`;前两条随后照常分发 |
| `future_command_held_until_due` | apply_at=105 的命令在 104 拍分发为空,105 拍出现 |
| `intents_stamped_monotonic_per_source` | 同 source 两批:序号 0..n 连续;apply_at = produced+1;不同 source 各自独立计数 |
| `same_shard_intent_rejected` | 透传 `AdmitError::SameShard` |
| `queue_full_propagates` | 小容量队列触发 `AdmitError::QueueFull` 透传 |
| `dispatch_merges_commands_and_envelopes_canonically` | 同分片同拍:命令 seq 5、seq 1 + 信封 1 枚 → 输出按 `canonical_key` 升序(信封与命令的类型序号不同,不并列) |
| `dispatch_is_exactly_once` | 第二次 dispatch 同拍为空 |
| `other_destinations_untouched` | 发往 B 的信封不影响 A 的分发;`pending_envelopes` 计数正确 |

## 完成定义

见 [README 公共完成定义](README.md)。
