# 任务 02:e2e 冒烟(经 viewer::client)+ 独立评审

## 目标

ACV2:viewer 的**客户端层**(不开窗)在真实服务器上端到端可用;随后对整轮
(viewer + main 接线)做交叉评审。

## 必读

- `docs/requirements/REQ-004A-visual-viewer.md` §3
- `crates/viewer/src/client.rs`(任务 01 产出)
- `crates/e2e/tests/network.rs`(真服务器装配 + 客户端测试风格,照抄 harness)

## 交付物

1. `crates/e2e/tests/viewer_client.rs`(e2e dev-deps 增 viewer,授权);
2. 评审(只读)——发现按严重度回报(本任务书内完成,不建修复环:
   medium/high 直接如实报告,由架构师处置)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `viewer_client_connects_streams_and_moves` | 真 bootstrap + serve → ViewerClient::spawn → 收 Welcome/Chunk 流(center-out 有限步)→ 控制端 MoveObserver → 驱动侧世界装载推进(outcome 计数或 chunk 流推进)|
| `viewer_client_receives_entity_and_block_events` | 外部注入 SpawnDynamic + SetBlock → ViewEvent::Entities/Blocks 到达,内容位级正确 |
| `viewer_client_clean_disconnect` | Goodbye → Disconnected 事件 + 服务器侧 Left 清理(hub 注销口径) |

## 评审要点(qwen,只读)

1. main 接线正确性(ObserveLink 真值/监听打印/失败 fail-stop);
2. client 模块零 macroquad 依赖(模块级 use 检查)、帧解码全部复用 proto(无重复协议实现);
3. 渲染纯函数测试真实性(无窗口造假测试);
4. 既有 301 测试零回归。

## 完成定义

见 [README](README.md);门禁 `-p e2e`;评审发现全部如实回报(不设修复环)。
