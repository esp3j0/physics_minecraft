# REQ-004A 实现任务书(总览)

- 状态:已定稿(需求 [REQ-004A](../../requirements/REQ-004A-visual-viewer.md) 2026-10-18 确认)
- 实现方式:glm 实现(00-01),qwen 验收与评审(02);cargo 门禁;架构师终审
  (含 ACV4 手动验收清单实跑)。

| # | 任务书 | 内容 | 模型 | 门禁包 |
| --- | --- | --- | --- | --- |
| 0 | [00-main-net.md](00-main-net.md) | app main 网络装配接线 + 监听配置 | glm | app |
| 1 | [01-viewer.md](01-viewer.md) | viewer crate(client + render) | glm | viewer |
| 2 | [02-e2e-review.md](02-e2e-review.md) | e2e 冒烟(经 viewer::client) | qwen | e2e |

公共完成定义同前几轮(严格 TDD;门禁 = 门禁包 test + clippy -D warnings;
只改任务书列出文件)。渲染窗口/相机部分例外:**不写窗口测试**——纯函数单测 +
ACV4 手动清单(REQ-004A §3),禁造假断言。

## 架构师决策(不得偏离)

1. macroquad = "0.4";viewer 依赖:sim-api/voxel/types/messaging/tokio(全 features
   照 app)/macroquad;**client 模块零 macroquad 依赖**(模块级 use 纪律);
2. 帧编解码复用 sim-api::proto(对称 codec 已 pub);viewer 不重复实现协议;
3. 渲染线程模型:macroquad 主循环(非 async)+ 独立 tokio 线程跑 client,
   crossbeam 或 std mpsc 桥(主循环 try_recv 排空);
4. 暴露面提取:viewer 内自实现 6 邻居判定(~30 行,不引 physics-sim——
   collider_build 语义是碰撞薄片,与渲染面不同);
5. ChunkUnload → 移除对应 Mesh;EntityStates removed → 移除实体节点;
6. app::Config 增 `listen_addr: String`(默认 "127.0.0.1:25565",serde default),
   §6 表同步;main 接线 = serve + ObserveLink 真值(以源码 ObserveLink 形状为准)。
