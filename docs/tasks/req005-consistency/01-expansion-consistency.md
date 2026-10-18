# 任务 01:外部编辑的扩张一致性与持久化归属

## 目标

兑现 REQ-005 §4.1.2/§4.1.3 / D3/D6/D11(外部编辑部分):边界 `SetBlock`
在 owner + 所有 holder 上**同一 `apply_at`** 生效;各副本逐字节一致;只有
owner 落盘;网络 `BlockUpdates` 改为从统一的 block write outbox 取,不再靠
驱动镜像命令。

完成后:

- 玩家在分片边界改一个方块,两侧 collider 在同一拍更新,无接缝;
- 非 owner 副本卸载不写库,owner 卸载才写;
- `Outbox.block_updates` 成为 block write 的统一来源(凝固桥/未来破坏在
  任务 03 接入);
- 一次用户编辑的日志 = 每个目标分片一条 `LogEntry`,回放逐片重投。

## 必读

- `docs/requirements/REQ-005-consistency.md` §4.1.2/§4.1.3、AC-C2/C3/C9
- `docs/requirements/REQ-005-decisions.md` D3/D6/D11
- `crates/shard/src/scheduler.rs`、`router.rs`、`chunks.rs`(任务 00 产出)
- `crates/sim-api/src/session.rs`(`send`/`submit_raw`)、`log.rs`
- `crates/physics-sim/src/sim.rs`(`apply_command`、`Outbox`、`rebuild_chunk_collider`)
- `crates/app/src/driver.rs`(`consume_net_up`、`emit_outputs`、`orchestrate`)
- `crates/app/src/net.rs`(`translate`/`command_shard`;本任务只消费它算出的
  owner,不改协议)

## 授权改动

- `crates/physics-sim/src/sim.rs`:`Outbox.block_updates` + `BlockWrite`/
  `BlockWriteKind` + SetBlock 写入记录;快照/测试;
- `crates/physics-sim/src/lib.rs`:re-export `BlockWrite`/`BlockWriteKind`;
- `crates/shard/src/scheduler.rs`:`submit_batch`(原子批量入箱)、
  `take_block_updates`(owner 过滤)、tick 末收集;测试;
- `crates/shard/src/router.rs`:`route_batch`/容量预检(或等价原子批量 API);
- `crates/sim-api/src/session.rs`:`send_broadcast(owner, command)` /
  `set_block(owner, pos, block)`;测试;
- `crates/app/src/driver.rs`:网络 SetBlock 走 `send_broadcast` 并记全部
  `LogEntry`;`pending_blocks` 镜像改为 `take_block_updates`;持久化只处理
  owner chunk;`validate_external_command` 增"owner 匹配 / chunk 已加载"两条;测试;
- `crates/app/src/net.rs`:若 `command_shard` 需要和 owner 对齐才改;不改
  协议/翻译形状。
- 测试文件按需更新(既有 Detonate 两片手工广播模式保持不变)。

## 接口设计(签名固定)

```rust
// physics-sim::Outbox
/// 方块写入来源:外部命令(同拍广播)/ 涌现(下一拍信封)。
pub enum BlockWriteKind { External, Emergent }
pub struct BlockWrite {
    pub pos: BlockPos,
    pub block: BlockId,
    pub kind: BlockWriteKind,
}
pub struct Outbox {
    pub migrations: Vec<Intent>,
    pub dirty_chunks: BTreeSet<ChunkPos>,
    pub solidified: Vec<(EntityId, BlockPos)>,
    /// 本拍物理/命令产生的方块写入;本任务只产生 `External`。
    /// `Emergent` 由任务 03(凝固桥/未来破坏)产生并走信封。
    pub block_updates: Vec<BlockWrite>,
}

// shard
impl Scheduler {
    /// 原子批量投递:所有目标分片都已在注册表且收件箱有空位才投;
    /// 任一失败 → Err,不投任何一片(不半投)。
    pub fn submit_batch(
        &mut self,
        shards: &[ShardId],
        input: StampedInput,
    ) -> Result<(), RouteError>;

    /// 本拍 block write 的 owner 视角发布集(取走即清,恰一次)。
    /// 只返回 `shard == owner(pos.chunk_of())` 的写入;非 owner 副本的写入
    /// 不由本接口发布(任务 03 的 BlockWrite 通道处理跨界涌现写入)。
    pub fn take_block_updates(&mut self) -> Vec<(ShardId, BlockPos, BlockId)>;
}

// sim-api
impl Session {
    /// 外部广播命令:dest = owner + 当前 holders(按 ShardId 升序去重)。
    /// 同一 apply_at/source/sequence;每个 dest 一条 LogEntry;整批原子。
    pub fn send_broadcast(
        &mut self,
        owner: ShardId,
        command: SimCommand,
    ) -> Result<Vec<LogEntry>, SessionError>;

    /// SetBlock 专用便捷入口:校验 pos 的 owner == owner,再 send_broadcast。
    pub fn set_block(
        &mut self,
        owner: ShardId,
        pos: BlockPos,
        block: BlockId,
    ) -> Result<Vec<LogEntry>, SessionError>;

    /// 该分片是否已加载该 chunk(hostile SetBlock 的未加载 chunk 门卫用)。
    pub fn is_chunk_loaded(&self, shard: ShardId, pos: ChunkPos) -> bool;
}
```

## 语义细则

1. **广播目标**:
   - `owner` 由网络翻译/驱动给出,必须是 `pos.chunk_of().shard_of()` 对应的
     已注册分片;不是 → `SessionError`/丢弃(hostile 路径不 panic);
   - 目标集 = `{owner} ∪ holders_of(owner, pos.chunk_of())`,按 `ShardId`
     升序;owner 一定在 holder 表里;
   - 每个目标收到同一 `StampedInput`(apply_at/source/sequence 相同);
     canonical key 的 dest 分量不同,不会冲突;
   - `next_sequence` 每个用户动作只 +1(不是每个 dest +1)。
2. **原子性**:
   - `submit_batch` 先检查每个目标的 `pending_inbox < inbox_capacity`;
     任一满 → `Err(InboxFull)`,不投任何一片;
   - 预检通过后逐个 `route_local`;单线程驱动下不会中途满;若发生,panic
     (内部一致性违规);
   - `Session` 侧新增类型化 `SessionError::Rejected(&'static str)`(或等价)
     表示 owner 不匹配/未加载;hostile 路径由驱动丢弃,不 panic。
3. **日志**:
   - 每个目标一条 `LogEntry { tick, dest, source, sequence, command }`;
     顺序 = ShardId 升序;
   - 回放按日志逐条 `submit_raw(dest)` 重投,不需要 `broadcast` 标志;
   - 既有 `Session::send` 保留为单分片测试/装配入口;生产 SetBlock 走
     `send_broadcast`/`set_block`。
4. **block write outbox**:
   - `ShardSim::apply_command(SetBlock)` 成功写体素后,向 `outbox.block_updates`
     推 `BlockWrite { pos, block, kind: External }`;`rebuild_chunk_collider`
     逻辑不变;
   - 广播到多片时,每个 holder 都会产生同一条 block_update;调度器
     `take_block_updates` 只发布 `shard == owner` 的那条(网络/持久化看到
     一次);
   - 非 owner 的 block_update 暂不发布(任务 03 加跨界 `BlockWrite` 信封);
   - `take_solidified` 保留兼容(内部可由 `block_updates` 实现,测试不变)。
5. **网络 BlockUpdates 源**:
   - `ObserveOrchestrator` 不再在 `consume_net_up` 里镜像 SetBlock;
   - tick 分支在 observe 输出前收集 `session.take_block_updates()`(外部
     SetBlock 的 owner 写入)+ `session.take_solidified()`(凝固桥,本任务保留);
     两者按格位合并,同一格后者覆盖前者;
   - 任务 03 会把凝固桥也迁到 `block_updates` 的 `Emergent` 分支,届时
     `take_solidified` 只作兼容包装。
6. **持久化**:
   - 只持久化 owner chunk(任务 00 的 `apply_chunk_unload`/`unload_shard`
     已保证);本任务补测试:两片各持一份,先卸非 owner、后卸 owner,
     重载取回编辑值;
   - 非 owner 卸载返回的 `None` 不计入 `chunks_unloaded`。
7. **hostile 边界**:
   - 客户端 SetBlock 的 owner 不是位置分片、分片未注册、y 越界 → 驱动
     `validate_external_command` 丢弃,不进入 `send_broadcast`;
   - **未加载 chunk 的 SetBlock 必须拒绝**(外部入口):`validate_external_command`
     用 `session.is_chunk_loaded(owner, pos.chunk_of())` 预检;否则
     `VoxelWorld::set_block` 会自动建空气 chunk,之后生成器/持久化会把地形
     覆盖成空气。内部测试/回放可继续走 `send`(它们显式控制装载);
   - 批量中任一收件箱满 → 计数 `commands_rejected`,不断链。

## 测试清单(先红后绿)

| 测试名 | 断言意图 |
| --- | --- |
| `setblock_broadcast_reaches_owner_and_holders` | 两片持有同一 chunk;`set_block` 后两个 sim 的该格都变为新值 |
| `setblock_broadcast_same_apply_at` | 两片的前缀输入 apply_at 相同;下一拍同时生效 |
| `setblock_broadcast_logs_one_entry_per_shard` | 返回 2 条 LogEntry,dest 升序、同 tick/sequence |
| `setblock_batch_atomic_on_inbox_full` | 目标之一收件箱满 → Err;两个 sim 都未变、log 未增 |
| `setblock_owner_mismatch_rejected` | pos 的 owner != 传入 owner → Err/驱动丢弃(不 panic) |
| `setblock_unloaded_chunk_rejected` | 外部 SetBlock 落在未加载 chunk → 驱动丢弃;不自动建空气 chunk |
| `block_edit_copies_bitwise_equal` | 编辑后两片该 chunk 的 `Chunk` 逐字节相等 |
| `boundary_edit_physics_matches_single_world` | 边界编辑 + 实体滚动穿越,双片 vs 单世界误差 < 1e-4 m |
| `block_updates_published_once_by_owner` | 两片各产生 update,`take_block_updates` 只返回 owner 的一条 |
| `non_owner_unload_does_not_persist` | 非 owner 卸载 → None;store 无写 |
| `owner_unload_persists_edit` | owner 卸载 → Some(编辑后 chunk);重载取回编辑 |
| `network_block_updates_from_outbox` | 网络 SetBlock 成功 → 客户端收 BlockUpdates;不再依赖驱动镜像 |
| `single_shard_regression` | 单分片 SetBlock/网络测试行为不变 |

## 完成定义

- 门禁:`cargo test -p physics-sim -p shard -p sim-api -p app --all-targets` +
  `cargo clippy -p physics-sim -p shard -p sim-api -p app --all-targets
  -- -D warnings` 退出码 0;
- 既有 AC2/AC3/AC-N3 测试零回归(手工两片 Detonate 广播模式保持不变);
- 不实现跨界涌现 block write(凝固桥/未来破坏)——任务 03;本任务只保证
  外部编辑与 owner 持久化。

## 风险与备注

- **日志变多**:一次边界编辑可能 2 条;`CommandLog` 只要求 tick 单调,
  同 tick 多条合法;回放按 dest 重投,不能再假设"一条用户命令 = 一条日志"。
- **holder 集合在 apply_at 拍必须稳定**:`send_broadcast` 在 `tick_now()` 时
  取 holder 集合,下一拍分发;如果两拍之间 ensure/load 改变了 holder 集合,
  以投递时集合为准(日志已记录 dest)。测试要覆盖"编辑与邻片 ensure 同拍"
  的顺序。
- **block_updates 的 owner 过滤**:非 owner 副本也会产生 update,直接丢弃会
  让未来凝固桥跨界写丢块;任务 03 的 `BlockWrite` 信封必须补上,并在
  REQ-005 验收里覆盖"跨界凝固"。
