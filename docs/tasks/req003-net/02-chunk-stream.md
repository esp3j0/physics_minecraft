# 任务 02:每玩家 chunk 流

## 目标

REQ-003 §4.3:连接层 chunk 流(视距差分/center-out/每泵限额)+ 驱动侧
StreamChunks 往返(v2 模式:连接层永不自己取数)。

## 必读

- `docs/requirements/REQ-003-network.md` §4.3、§3.2(v2 chunk_stream 证据)
- `docs/tasks/req003-net/README.md` 决策 7/10
- `crates/app/src/net/`(任务 01 产出,以源码为准)
- `docs/research/ferrumc-v2-survey.md` §6(16/16、chebyshev、pending 单槽合并)

## 交付物

app::net 增 chunk 流模块(连接态 + pump)+ `driver.rs` 增 `DriverCommand::StreamChunks`
处理(第 ④ 编排步,README 决策 7)+ 内嵌测试。

## 接口设计(要点固定)

```rust
// DriverCommand 追加:
StreamChunks {
    load: Vec<ChunkPos>,
    unload: Vec<ChunkPos>,
    reply: oneshot::Sender<Vec<ChunkPos_chunk 对>>, // (ChunkPos, voxel::Chunk)
},

// 连接层 ChunkStream 状态(照 v2):
struct ChunkStreamState {
    center: ChunkPos,                 // pending 单槽合并(读路径只更新,不触发)
    view_distance: u8,                // Subscribe 时定(钳 32)
    loaded: BTreeSet<ChunkPos>,
}
// pump(独立 interval task,默认 10Hz):desired = center 周围切比雪夫视距方形;
// diff: to_load = desired - loaded(按 (chebyshev(center), x, z) 升序,截 16);
//       to_unload = loaded - desired(同序,截 16);
// 发 StreamChunks,回执的 chunk 逐个入 World 队列(ChunkData),unload 侧
// 先清 loaded 再发 ChunkUnload 给客户端;
// 回执失败(驱动关停)= 连接终止。
```

驱动侧:tick 末端编排第 ④ 步——drain 挂起的 StreamChannels 请求(网络侧经
NetUp 上行后由驱动持有):load 批量 `provider.get`(生成/持久兜底已内建);
unload 无模拟动作(观察者的模拟激活/票据已由 REQ-002 处理,chunk 流是纯视图);
oneshot 回复。**请求在编排点处理(同拍或下一拍),永不阻塞 tick。**

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `initial_pump_streams_view_square_center_out` | 订阅后首泵:按切比雪夫距升序收 ChunkData,数量 = 视距方形且 ≤16/泵,多泵补齐 |
| `view_distance_clamped_32` | 订阅 200 → 方形按 32 计 |
| `moving_center_diffs_minimally` | 中心移动 1 chunk → 只收新进入的列(差分),旧列收 ChunkUnload |
| `per_pump_limits_enforced` | 视距 8(289 chunk)→ 首泵恰 16,总泵数 ≥ ceil(289/16) |
| `server_chunks_come_from_provider` | 修改一个 chunk 并持久化 → 客户端收到的是修改值(真数据源) |
| `stream_reply_never_blocks_tick` | 大视距订阅期间驱动照常 tick(暂停时间下 tick 计数不减) |
| `disconnect_cleans_stream_state` | 断连后 pump 停止(无孤儿任务:连接 task 终止即停) |

## 完成定义

见 [README 公共完成定义](README.md)。
