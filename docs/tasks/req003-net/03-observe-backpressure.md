# 任务 03:观察输出编排与出站背压

## 目标

REQ-003 §4.4:把模拟产出变成每玩家的观察流(实体状态增量+周期全量、方块变更),
接入任务 01 的两级出站队列与断连清理;AC-N4/N5 的服务器侧全部逻辑。

## 必读

- `docs/requirements/REQ-003-network.md` §4.2/§4.4、§9(AC-N4/N5)
- `docs/tasks/req003-net/README.md` 决策 6/8/9
- `crates/app/src/net/` 与 `driver.rs`(任务 01/02 产出)
- `crates/sim-api/src/proto.rs`(EntityStates/BlockUpdates)

## 交付物

driver.rs 观察输出编排 + net 出站填充与玩家生命周期接线 + 内嵌测试。

## 接口设计(要点固定)

```rust
/// 网络配置增量:observe_interval_ticks(10,增量周期)、
/// full_snapshot_interval_ticks(60,全量周期)、entity_view_filter = view_distance(复用订阅值)。

// 驱动侧编排(tick 末端,第 ⑤ 步):
// 1) Joined 上行 → 为玩家建:观察者(MoveObserver 已到)、last_sent: BTreeMap<EntityId, EntityState>;
// 2) Left 上行 → 观察者移除 + 流状态清理;
// 3) 输出周期到 → 对每订阅玩家:
//    收集视距内分片的 entity_states() → 距离过滤(≤ view_distance×16m,3D)
//    → 与 last_sent 差分(变化集 + 消失集)→ EntityStates 入 World 队列 → 更新 last_sent;
//    全量周期到(或差分首次)→ 整表(removed 空);
// 4) BlockUpdates 源 = 镜像外部 SetBlock 成功发送(session.send Ok 且命令为 SetBlock)
//    + Session::take_solidified();按格位视距覆盖广播给订阅者(单包合并);
// 5) Welcome 在 Joined 时入 Critical(player_id + 当前 tick)。
```

## 语义细则

1. 距离过滤用**玩家当前位置**(观察者表,MoveObserver 最新值);
2. 实体"消失"判定:上轮在 last_sent 且本轮过滤集无 → removed(迁移/凝固/移除
   三种来源同型;凝固实体同时出现在 BlockUpdates——位置由 take_solidified 提供);
3. last_sent 为每玩家私有副本(玩家互不影响;断连即弃);
4. 出站溢出策略沿用任务 01 队列(Critical 断连;World 丢最旧);
   EntityStates 因 latest-wins 语义丢旧无损正确性(下轮全量周期自愈,doc 注明);
5. 增量/全量两周期独立计数;输出编排挂在 tick 分支(不新增 select 臂)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `welcome_on_join_carries_player_and_tick` | Welcome{player_id, tick} 到达 Critical 通道 |
| `entity_states_incremental_then_full` | 拍 1 出生实体 → 首输出全量;移动后增量只含变化;周期到 → 全量(removed 空) |
| `removed_reported_on_solidify_and_removal` | 凝固实体 → 下轮增量 removed 含其 ID 且 BlockUpdates 含该格;RemoveEntity 同型 |
| `distance_filter_respects_view_distance` | 视距外实体不入包;移近后入包 |
| `block_updates_broadcast_by_proximity` | 玩家 A 视距内、B 外的方块变更:只 A 收到 |
| `left_removes_observer_and_stream` | 断连 → 观察者表移除(驱动可见)→ 空片无票可 despawn(缩短配置验证) |
| `slow_client_degrades_not_blocks` | 不读 socket 的客户端:World 队列丢最旧、tick 计数不减;Critical 触发断连 |

## 完成定义

见 [README 公共完成定义](README.md)。
