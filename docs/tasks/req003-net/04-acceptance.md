# 任务 04:AC-N1~N6 验收(内嵌测试客户端)

## 目标

REQ-003 §9 六条验收的端到端验证(qwen 执笔,与实现交叉)。测试内嵌迷你客户端
(复用 app::net 公开的帧 codec;若未公开,授权在 net 模块加 `pub` 测试辅助)。

## 必读

- `docs/requirements/REQ-003-network.md` §9(验收原文)
- `crates/app/src/net/` + `driver.rs`(任务 01-03 产出,以源码为准)
- `crates/e2e/tests/world_loading.rs`(全链测试风格:真驱动 + 暂停时间)

## 交付物

`crates/e2e/tests/network.rs`(e2e dev-deps 增补按需授权)。

## 测试清单(逐条对 AC)

| 测试名 | 断言(AC) |
| --- | --- |
| `acn1_connect_join_and_observe` | 客户端连入 → Welcome → Subscribe → MoveObserver → 世界该区域分片被创建/装载(经驱动编排)(AC-N1) |
| `acn2_chunk_stream_center_out_capped` | 走入新区:ChunkData 按切比雪夫序、每泵 ≤16、远离收 ChunkUnload;视距钳制(AC-N2) |
| `acn3_command_roundtrip` | 挖(SetBlock air)/放/生成/爆炸经协议送达模拟并物理生效(实体被炸飞)(AC-N3) |
| `acn4_incremental_matches_full` | 同场景下"增量累积终态 == 全量快照"(逐实体位级);方块变更到达(AC-N4) |
| `acn5_slow_client_backpressure` | 慢客户端全程:服务器 tick 计数与正常客户端体验不受影响;Critical 触发断连(AC-N5) |
| `acn6_disconnect_reconnect` | 断连 → 票撤(despawn 缩短配置下分片回收)→ 重连恢复观察与 chunk 流;双客户端并发互不干扰(AC-N6) |

工程细节:监听 127.0.0.1:0 取实际端口;驱动跑暂停时间但**网络真实 IO**
——pump/keepalive 的 interval 在暂停时间下行为需实测(必要时这些测试用真实
短超时 + 小周期,或 tokio start_paused 与真实时间混合时以源码行为为准,
notes 说明取舍);物理量按容差、协议内容位级 `==`。

## 完成定义

见 [README 公共完成定义](README.md)。
