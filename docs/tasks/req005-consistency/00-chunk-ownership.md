# 任务 00:chunk 归属、区域激活与邻片 ensure

## 目标

兑现 REQ-005 §4.1 / D2/D4/D5/D6:把"chunk 归属 + holder 副本 + 区域过滤 +
邻片 ensure + owner-only 持久化"落成运行时不变量。完成后:

- 每个已加载 chunk 有唯一 owner(= 本体分片),holder 集合可查询;
- 激活只请求 `core ∪ margin_ring` 内的 chunk,不再出现"每个已注册分片
  都装载观察者附近同一批 chunk";
- 观察者靠近边界时,邻片被 ensure;迁移目的地不会成为信封黑洞;
- 非 owner 副本卸载不写库,owner 卸载才写。

## 必读

- `docs/requirements/REQ-005-consistency.md` §4.1/§8(AC-C1/C3/C4/C9)
- `docs/requirements/REQ-005-decisions.md` D2/D4/D5/D6
- `crates/shard/src/ownership.rs`、`scheduler.rs`、`router.rs`、`lifecycle.rs`
- `crates/physics-sim/src/activation.rs`、`sim.rs`(`apply_activation`/`core_contains`)
- `crates/app/src/driver.rs`(`orchestrate`/`shard_of_position`)、`config.rs`
- `crates/types/src/coords.rs`(`ShardPos::min_chunk`)、`ids.rs`(`ShardId`)
- 既有两分片装载测试:`crates/e2e/tests/migration.rs`、
  `crates/shard/tests/two_shards.rs`、`crates/e2e/tests/activation.rs` 的
  `chunk_ownership_is_unique_structurally`(注意:它目前是手工装载的结构测试)

## 授权改动

- `crates/types/src/coords.rs`:`ChunkRegion`(或等价)+ 单元测试;
- `crates/physics-sim/src/activation.rs`:`ActivationConfig` 加
  `region_margin_chunks`;`plan` 接收区域并过滤;测试更新;
- `crates/physics-sim/src/sim.rs`:`region` 计算/透传;快照兼容;测试;
- `crates/shard/src/chunks.rs`(新)或 `ownership.rs`:`ChunkHolders` holder 表;
- `crates/shard/src/scheduler.rs`:holder 登记、owner/holders 查询、
  `apply_chunk_load` 区域校验、`apply_chunk_unload` owner-only、
  `unload_shard` 过滤、`pending_shard_destinations`、测试;
- `crates/shard/src/router.rs`:`pending_destinations()`(只读遍历桶);
- `crates/app/src/config.rs`:`margin_meters` → `region_margin_chunks` 换算;
- `crates/app/src/driver.rs`:`orchestrate` ensure 观察者 32m 内分片 +
  pending destinations;测试;
- `crates/app/src/net.rs` 不改(本任务不碰网络协议)。

> 允许新增 `crates/shard/src/chunks.rs` 并在 `lib.rs` 导出;不要把
> `scheduler.rs` 继续堆到 1500 行。

## 接口设计(签名固定)

```rust
// types
/// chunk 网格的闭区间矩形(单位:chunk)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ChunkRegion { pub min: ChunkPos, pub max: ChunkPos }

impl ChunkRegion {
    /// 分片本体区:8×8 chunk,闭区间。
    pub fn core(shard: ShardId) -> Self;
    /// 向外扩 margin 个 chunk(四边都扩)。
    pub fn expanded(self, margin: i32) -> Self;
    pub fn contains(&self, pos: ChunkPos) -> bool;
}

// physics-sim
impl ActivationConfig {
    /// 激活/扩张区外扩 chunk 数;默认 1(=16m,由 app::Config.margin_meters 换算)。
    pub region_margin_chunks: i32,
}
impl ActivationController {
    pub fn plan(
        &mut self,
        observers: &[Vec3],
        loaded: &[ChunkPos],
        entities: &[(EntityId, Vec3, bool)],
        now: Tick,
        region: ChunkRegion,
    ) -> ActivationDecisions;
}
impl ShardSim {
    /// 本分片允许加载的静态范围 = core ∪ margin_ring。
    pub fn activation_region(&self) -> ChunkRegion;
}

// shard
impl Scheduler {
    /// owner = pos 的核心分片;world/dim 取自身份参数(单世界单维度 v1)。
    pub fn owner_of(shard: ShardId, pos: ChunkPos) -> ShardId;
    /// 当前持有该 chunk 的分片集合(含 owner);未加载 → 空集。
    pub fn holders_of(&self, owner: ShardId, pos: ChunkPos) -> BTreeSet<ShardId>;
    /// Router 待投递信封的目的分片(两拍之间),驱动 tick 末 ensure 用。
    pub fn pending_shard_destinations(&self) -> BTreeSet<ShardId>;
}

// app
impl Config {
    /// ceil(margin_meters / 16),至少 0;默认 16m → 1。
    pub fn region_margin_chunks(&self) -> i32;
}
```

## 语义细则

1. **owner 静态**:owner 只由 `pos.shard_of()` 决定,不做动态
   claim/release;`OwnershipMap` 继续只登记分片所有权。
2. **holder 表**:
   - `apply_chunk_load(shard, pos, chunk)` 前先断言 `activation_region()` 包含
     `pos` 且该 chunk 的 owner 分片已注册,否则 panic(调用方 bug/fail-stop);
   - 装配期 `register(sim)` 允许先注册 holder、后注册 owner(测试/回放按
     ShardId 升序注册);但**第一拍开始前**所有 holder 的 owner 必须已注册,
     `tick()` 开头校验,不满足 → panic;
   - 插入/替换 chunk 后,把 `shard` 登记为 `(owner, pos)` 的 holder;
   - `apply_chunk_unload(shard, pos)` 移除 holder;
     - `shard == owner` → `Some(chunk)`(交驱动持久化);
     - 非 owner → 丢弃副本,返回 `None`(不写库);
   - holder 集为空时删除条目;
   - 已注册分片的已加载 chunk 必须都能重建 holder(回放/恢复用)。
3. **激活区域**:
   - `ActivationController::plan` 的装载候选先过滤
     `region.contains(cp)`;观察者距离/滞回/冷却/预算语义不变;
   - 区域外的 chunk 即使距观察者很近也不请求;
   - 卸载候选不强制在区域内(已加载的扩张副本可以正常卸);
   - `region_margin_chunks` 至少 1 时才保证边界 collider;配置 0 明确允许
     (测试/实验用),生产由 Config 默认 16m 给出 1。
4. **邻片 ensure(驱动 `orchestrate`)**:
   - 对每个观察者位置,计算 `awake_unload_radius`(默认 32m)内所有分片
     (核心 AABB 距离;至多 4 个),未注册 → `new_sim` + `register`;
   - 再从 `pending_shard_destinations()` 取待投递信封目的分片,未注册 →
     ensure;确保本拍结束后不存在"有信封但无分片"的桶;
   - **迟到兜底**:`Scheduler::tick` 开始/结束时若发现 `apply_at <= 当前拍`
     且 destination 未注册的信封,说明 ensure 漏了(内部一致性违规)→
     panic(fail-stop);不允许信封永久沉在桶里。正常路径由观察者 32m
     ensure 覆盖,`pending_shard_destinations` 只处理下一拍的目的地;
   - ensure 计数并入 `shards_spawned`;同拍重复 ensure 只建一次;
   - 观察者所在分片仍由原逻辑覆盖。
5. **持久化**:
   - `unload_shard(shard)` 只把 `shard == owner(pos)` 的 chunk 放进返回值;
     非 owner 副本 `remove_chunk` 后丢弃;
   - 驱动 `put_chunks`/despawn 屏障只看到 owner chunk;
   - owner 分片不存在但 holder 存在 → panic(编排 bug,不应发生)。
6. **快照兼容**:
   - `ShardSimConfig.activation` 新增字段随快照走;旧 checkpoint 按 D8
     统一版本处理,不在本任务做兼容;
   - holder 表不入快照,恢复时从各 sim 的已加载 chunk 重建
     (`Scheduler::rebuild_holders()`)。

## 测试清单(先红后绿)

| 测试名 | 断言意图 |
| --- | --- |
| `chunk_region_core_and_margin` | core 8×8;expanded(1) 四边各扩 1;contains 边界正确 |
| `activation_never_loads_outside_region` | 观察者在片 0 中心,片 1 的 plan 对区域内 chunk 0 请求;区域外即使近也不请求 |
| `activation_two_shards_no_cross_core_duplicates` | 两分片 + 一观察者:同一 chunk 不会被两片都当"本体"请求;holder 只出现在 core∪margin |
| `holders_track_load_unload` | load 后 holder 含 shard;重复 load 不重复;unload 后移除 |
| `non_owner_unload_returns_none` | 邻片扩张副本 unload → None;owner 的 chunk 仍在 |
| `owner_unload_returns_chunk` | owner unload → Some(原 chunk) |
| `unload_shard_filters_to_owner_chunks` | 片 0 despawn:只返回 core 的 chunk;扩张副本丢弃 |
| `pending_migration_destination_is_ensured` | 构造未注册目的分片 + 待投递迁移信封;驱动 tick 末 ensure;下一拍不再 pending |
| `late_unregistered_destination_panics` | `apply_at <= 当前拍` 且目的分片未注册 → fail-stop(不允许黑洞) |
| `ensure_covers_observer_neighbors` | 观察者距边界 <32m → 邻片注册;>32m 不注册 |
| `margin_config_converts_to_chunks` | 16m→1、0m→0、17m→2(ceil) |
| `single_shard_activation_regression` | 单分片既有激活/预算/滞回测试不变(允许加载数 +margin) |

## 完成定义

- 门禁:`cargo test -p types -p physics-sim -p shard -p app --all-targets` +
  `cargo clippy -p types -p physics-sim -p shard -p app --all-targets -- -D warnings`
  退出码 0;
- 既有单分片/两分片/网络测试零回归(允许因 margin 环产生的计数更新,需在
  notes 说明);
- 不实现 D3(同拍编辑广播)/D7(全局快照)/D10(事件广播)——后续任务。

## 风险与备注

- **两分片测试会变**:手工装载的 0..=8 / 7..=15 仍可作为"构造前提",但
  `Scheduler` 现在会校验区域;测试里的手工 load 必须落在对应分片的
  `core ∪ margin` 内,否则会被 panic。更新测试时保持"扩张带重叠"意图。
- **owner 不存在**:如果先 load 一个 chunk 到邻片、但 owner 分片未注册,
  属于编排 bug;本任务选择 fail-stop,不静默允许。驱动 ensure 必须先生效。
- **margin=0**:允许配置,但边界 collider 可能缺失;生产默认 1。测试要覆盖
  0 的显式语义,避免"以为有环其实没有"。
