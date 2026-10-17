# 任务 02:观察者命令化 + 装载编排 + despawn(shard/sim-api)

## 目标

REQ-002 §4.2/§4.4/§4.5:观察者命令化(调度器级专用收件箱)、激活请求外露、
chunk 装卸编排 API、despawn 候选与卸载。

## 必读

- `docs/requirements/REQ-002-world-loading.md` §4.2/§4.4/§4.5
- `docs/tasks/req002-world/README.md` 决策 3/4/5/6
- `crates/shard/src/scheduler.rs`(tick 流程/生命周期接线现状)
- `crates/shard/src/router.rs`(收件箱 apply_at 语义)
- `crates/sim-api/src/session.rs` / `replay.rs`(盖章/原章直投)

## 交付物

1. `crates/messaging/src/command.rs`:`SetObserverPosition { position: Vec3 }`
   追加(ordinal 续);
2. `crates/shard/src/scheduler.rs`:观察者表 + 专用收件箱 + 编排 API + despawn;
3. `crates/sim-api/src/session.rs` / `replay.rs`:`move_observer` + 回放路由。

## 接口设计(签名固定)

```rust
// messaging:SimCommand 追加(声明序最后):
SetObserverPosition { position: Vec3 },

// shard::Scheduler 增:
/// 观察者只读视图(驱动 ensure_shard/票判定用)。
pub fn observers(&self) -> Vec<(InputSource, Vec3)>;
/// 观察者输入入箱(调度器级专用收件箱,apply_at 到期应用;非法 apply_at 沿用迟到语义)。
pub fn submit_observer(&mut self, input: StampedInput);
/// 两拍之间取本拍激活决策(chunk 装卸请求 + 已落实冻结)。
pub fn take_activation_requests(&mut self) -> Vec<(ShardId, ActivationDecisions)>;
/// 两拍之间装载一个 chunk(幂等:重复装载同 chunk = 重建)。
pub fn apply_chunk_load(&mut self, shard: ShardId, pos: ChunkPos, chunk: voxel::Chunk);
/// 两拍之间卸载一个 chunk(返回它供持久化;未装载 → None)。
pub fn apply_chunk_unload(&mut self, shard: ShardId, pos: ChunkPos) -> Option<voxel::Chunk>;
/// 两拍之间取 despawn 候选(空片+无票+idle,只记录不清)。
pub fn take_despawn_requests(&mut self) -> Vec<ShardId>;
/// 卸载整个分片:lifecycle 走 Draining→Stopped→release,逐 chunk 卸载返回。
/// 实体非空 → panic(fail-stop);未注册 → panic。
pub fn unload_shard(&mut self, shard: ShardId) -> Vec<(ChunkPos, voxel::Chunk)>;

// sim-api:
impl Session {
    /// 观察者移动(生产入口):盖章(apply_at=当前拍,下一 tick 生效)、
    /// 返回 LogEntry(dest=position 所在分片,信息性)。
    pub fn move_observer(&mut self, position: Vec3) -> Result<LogEntry, SessionError>;
}
// replay.rs:按命令变体路由——SetObserverPosition → submit_observer 原章直投;
// 其余 → submit_raw(dest, …) 照旧。
```

## 语义细则

1. 专用收件箱 = `VecDeque<StampedInput>`(容量上限沿 inbox_capacity?调度器级
   独立小上限 256,满 → 丢弃并 panic?观察者频率低,unbounded 可接受——
   **决策:与分片收件箱一致的有界语义复杂度不值得,直接 unbounded VecDeque**,
   doc 注明,REQ-003 网络层再收口);
2. `tick()` 开头(步骤 2 之前)应用收件箱中 `apply_at == at` 的全部条目
   (按 canonical 序;未来拍的留存——与 router inbox 同语义);
3. 票判定:观察者距分片核心 AABB(chunk_unload_radius 48m,水平距离到
   8×8 chunk 区域);`set_observers` 保留为整表替换直通;
4. 激活刷新顺序不变(观察者应用 → apply_activation → 前缀 → step);
5. despawn 候选在 tick 步骤 7(生命周期接线处)记录;lifecycle 本身
   **不推进到 Draining**(卸载由 unload_shard 显式驱动);
6. `move_observer` 的 LogEntry.sequence 沿 session 单调序列(与 send 共用)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `observer_command_updates_table_at_tick` | submit_observer(apply_at=N) → tick N 后 observers() 反映;N 前未应用 |
| `set_observers_direct_still_works` | 整表替换直通(兼容开关) |
| `activation_requests_exposed_between_ticks` | 有观察者移动后,take_activation_requests 非空含 chunk 装载请求;第二次取为空(恰一次) |
| `chunk_load_unload_roundtrip_via_orchestr_api` | apply_chunk_load 后 sim 生效(物理可见/查询);unload 返回该 chunk |
| `despawn_candidate_only_when_empty_and_unticketed` | 空片无票 idle 到期 → 候选;有实体或有票 → 无候选 |
| `unload_shard_walks_lifecycle_and_releases` | 卸载后 shard_ids 不含、lifecycle Stopped、ownership 释放;再 register 同分片可行;实体非空 → panic |
| sim-api:`move_observer_stamps_and_logs` | LogEntry 拍号/dest/sequence 正确;入 CommandLog 可序列化 |
| sim-api:`replay_routes_observer_entries` | 含 SetObserverPosition 的日志重放:观察者表再生(重放后 observers() 与原线一致) |

## 完成定义

见 [README 公共完成定义](README.md)。
