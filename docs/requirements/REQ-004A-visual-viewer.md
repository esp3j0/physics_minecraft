# REQ-004A:最小可视化 viewer(demo)

- 状态:**已确认**(2026-10-18 用户拍板:viewer 为可视化 demo;正式客户端未来重新设计;
  MC 客户端直连不做——自定义物理实体无法在 MC 客户端忠实呈现)
- 上游:[REQ-003](REQ-003-network.md)(自定义协议/连接层/chunk 流/观察输出——viewer 的全部数据源)

## 1. 目标

跑起服务器,再跑起 viewer,**亲眼看到物理世界**:彩色体素地形、被炸飞的动态立方体、
凝固成方块的过程;自由飞行相机;基础 HUD。作为整条链路的可视化验证与今后的调试工具。

**定位边界(用户明确)**:这是 demo,不是未来客户端的地基——技术选型取最轻
(macroquad 即时渲染),不为未来做任何抽象预留。

## 2. 需求内容

1. **前置:app main 网络装配接线**(REQ-003 交付了 app::net::serve/OutboundHub/
   ObserveLink,但 main.rs 未接线 `observe: None` 占位)——服务器启动即监听;
2. viewer(新 crate `viewer`,workspace 自动收录):
   - **client 模块**(纯 tokio + sim-api::proto,不依赖 macroquad,e2e 可复用):
     连接/Hello/Subscribe/MoveObserver/帧循环/通道桥(独立线程 runtime → 主循环);
   - **render 模块**(macroquad 0.4):窗口、自由飞行相机(WASD + 鼠标视角,
     Shift/Ctrl 升降)、chunk→Mesh(**仅暴露面**,内置三色映射:air 透明/stone 灰/
   dirt 棕;chunk 增删跟随流)、实体立方体(按 EntityStates 的位姿更新,
     removed 即移除,颜色区分于地形)、HUD(tick/chunk 数/实体数/FPS);
   - 相机→观察者耦合:默认开(MoveObserver 跟随相机),按键切换(空格切换耦合);
3. 视距默认 4 chunk(81 chunk 上限,控制网格量)。

## 3. 验收标准

- **ACV1 main 网络接线**:服务器启动后外部客户端可连、收 Welcome(app 测试,
  复用 REQ-003 测试风格);
- **ACV2 client 层端到端**(e2e,不开窗):经 viewer::client 连真服务器 →
  订阅 → 收 chunk 流与实体流 → MoveObserver 反映;
- **ACV3 渲染纯函数**(单测):暴露面提取(6 邻居判定)正确、颜色映射表、
  Mesh 顶点生成(暴露面数 × 4 顶点);
- **ACV4 手动验收清单**(渲染不可断言,禁造假测试;清单由架构师/用户实跑勾选):
  ①地形彩色网格正确 ②WASD/鼠标飞行顺滑 ③生成实体→爆炸→飞散 ④凝固成方块
  (地形色变化)⑤相机移动带动世界装载/卸载 ⑥HUD 数字合理。

## 4. 决策

1. macroquad 0.4(demo 定位;弃 bevy——更重且未来客户端会重选型);
2. 渲染代码**不写窗口类测试**;纯函数(暴露面/颜色/顶点)单测覆盖;
3. viewer::client 与 render 分模块,client 零 macroquad 依赖;
4. 服务器监听地址入 app::Config(`listen_addr`,默认 "127.0.0.1:25565")。
