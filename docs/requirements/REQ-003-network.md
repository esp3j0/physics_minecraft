# REQ-003:网络与每玩家 chunk 流

- 状态:**完成**(2026-10-17 确认并开工,2026-10-18 落地。§8 四项决议执行完毕;
  AC-N1~N6 全部有测试(e2e/network.rs,内嵌测试客户端 + 真 TCP);
  评审抓出 1 high(远程白名单命令 DoS,fail-stop 语义泄漏到 hostile 输入——
  已修:驱动侧 validate_external_command 门卫两条外部入口全覆盖)+ 2 medium
  (观察者移除 API 缺失→remove_observer 全链;泵 try_send 无回滚→差分回滚),
  均已修复并有回归测试。实现:glm + qwen 交叉验证 + cargo 门禁 + 架构师终审。
  挂账:出站侧 2MiB 帧上限未强制(密集场景全量表可超限,评审 low,分包待做)、
  Welcome 双发(连接层+驱动各一,字面偏差)、WAN/压缩/加密不做)
- GitHub issue:[#5](https://github.com/esp3j0/physics_minecraft/issues/5)
- 提出日期:2026-10-17
- 上游:[REQ-001](REQ-001-architecture.md) §9.9(网络独立 REQ-003;照搬 v2 chunk_stream
  + v1 网络线程模式)、[REQ-002](REQ-002-world-loading.md)(观察者命令化/票据/ChunkProvider
  已就绪——本 REQ 的模拟侧地基)
- 调研:docs/research/ferrumc-v2-survey.md §6/§7 + 2026-10-17 网络栈专项调研
  (ferrumc-v2 真协议 772 实现、v1 网络线程、pumpkin 量级,证据在 §3)

---

## 1. 项目定义

给模拟核装上网络缝:真实进程外的客户端(TCP)能连入、成为玩家(观察者+票据)、
收发命令与观察。完成后的验收场景:两个测试客户端连接同一服务器,各自走进世界
不同区域,看到自己视距内的 chunk 流与实体运动,挖/放/引爆即时生效,慢客户端
被背压保护、服务器 tick 不受网络拖累。

一句话:**模拟核之外的进程,能以玩家的身份进世界、看世界、改世界。**

## 2. 既定共识(继承)

1. REQ-001/002 全部决议;改动任一条须先修订其 §9;
2. 观察者=玩家已就绪(source 即玩家 ID,`SetObserverPosition` 命令化、票据、
   despawn、ChunkProvider 全部落地)——**本 REQ 主要是网络传输与会话层,
   模拟侧零改动**;
3. ADR-0003:模拟跑专用线程池,网络 I/O 与异步编排归 tokio(单 runtime);
4. 模拟状态只因盖章输入而变——网络输入必须经命令通道盖章,网络层永不直改模拟。

## 3. 已验证事实(调研结论,不必复测)

1. **两参照系的共识架构**:每连接一个 task + **有界**命令通道进模拟
   (v1 的 crossbeam unbounded 教训被 v2 明确修正为 bounded + oneshot 回执);
   连接层"只发已 typed 已 trusted 的事件,翻译层只翻译路由不解析字节";
2. **每玩家 chunk 流放连接层**:连接持 loaded 集合 + cadence pump(视距差分、
   切比雪夫 center-out 排序、每泵 16 装/16 卸、视距钳制 32);chunk 的
   生成/ticket/编码在模拟侧,经带 oneshot 回执的命令往返(v2 `StreamChunks`);
   **socket 读路径不触发 chunk 工作**(只更新合并的 pending 中心);
3. **出站背压**:按优先级分级有界队列(v2:Critical 64 / State 128 / World 512 /
   Cosmetic 256),mandatory 包被丢 → 断连;慢客户端不拖累 tick;
4. **协议量级参照**:ferrumc-v2 真协议(单版本 772、离线、无加密)≈ 0.7 万行
   proto(生成式包表)+ 连接状态机;pumpkin(多版本+AES 加密+认证)3.8 万行;
   MC chunk 单包上限 2 MiB、zlib 压缩必要;
5. v1 的 TokioNetworkThread(独立线程 + 自建 runtime)动机是其主循环非 async;
   我们驱动本来就是 tokio(见 §9 决议 2)。

## 4. 架构(以 §9 决议 1 的推荐方案展开)

### 4.1 协议(v1 自定义二进制)

- 传输:TCP;帧 = 4 字节长度前缀 + bincode 载荷;帧上限 2 MiB(v2 教训,防炸弹);
  v1 不压缩(自定义协议 bincode 载荷小;压缩留升级位);
- 客户端→服务器(**C2S**):`Hello { client_id }`、`Subscribe { view_distance }`、
  `MoveObserver { position }`、命令包(映射现有 SimCommand:挖/放=SetBlock、
  生成=SpawnDynamic、爆炸=Detonate)、`Goodbye`;
- 服务器→客户端(**S2C**):`Welcome { player_id, tick }`、`ChunkData { pos, chunk }`、
  `ChunkUnload { pos }`、`EntityStates { tick, [(id, state)] }`(增量+周期全量,
  决议 3)、`BlockUpdates { [(pos, block)] }`、`Bye { reason }`;
- 协议类型定义在 **sim-api**(蓝图:接口形状=网络形状;bincode 序列化,
  与 LogEntry/GlobalSnapshot 同构——回放与网络同一形状,REQ-002 §4.2 伏笔兑现)。

### 4.2 连接与会话层(app::net)

- 每连接一个 tokio task:帧解码 → typed C2S → 会话状态机
  (Connected → Subscribed;未 Subscribe 不进世界);
- **翻译层纯函数**(照 v2 session 模式):C2S → DriverCommand 映射一处收口
  (MoveObserver→MoveObserver、命令包→Send、Hello→注册、断开→观察者移除);
- 每玩家出站通道(有界)+ 轻量优先级(v1 两级:Critical/World),Critical 满/丢 →
  断连;World(实体状态/chunk 流)满 → 丢弃合并(实体状态天然 latest-wins,
  chunk 流 pump 重算)——慢客户端体验降级不断服;
- keepalive:周期小包;超时未响应 → 断连(照 v2 ConnectionLiveness)。

### 4.3 每玩家 chunk 流(连接层,照 v2 chunk_stream)

- 连接态:center(合并 pending 单槽)、view_distance(Subscribe 钳制 ≤32 chunk)、
  `loaded: BTreeSet<ChunkPos>`;
- pump cadence(配置,默认 10 Hz):视距差分 → 切比雪夫 center-out 排序 →
  每泵 ≤16 装/≤16 卸 → `DriverCommand::StreamChunks { load, unload, oneshot }`
  → 驱动经 ChunkProvider 取数 + 票据登记 → 回包 → 按优先级入出站;
- 断线 → 卸载该玩家全部视距票(经 despawn 票据语义,REQ-002 已有)。

### 4.4 驱动侧(app)

- `DriverCommand` 追加:`StreamChunks { load, unload, reply }`(oneshot 回执,
  驱动 tick 末端处理,不阻塞 tick——回执在下一编排点);
- 观察输出源:①实体状态(每玩家视距过滤)②方块变更(SetBlock/凝固桥经
  dirty_chunks/outbox 已有)→ 按订阅周期组包;
- 网络监听与连接 task 与驱动同 runtime(决议 2)。

## 5. 与 v1/v2 的差异(有意为之)

| 项 | v1/v2 | 本设计 | 理由 |
| --- | --- | --- | --- |
| 协议 | 真 MC 协议 | 自定义 bincode(决议 1) | RL 效率 + 工程量(见 §9) |
| 网络线程 | 独立线程自建 runtime | 同 runtime 每连接 task | 驱动本就是 tokio;ADR-0003 不受影响 |
| chunk 编码 | MC 调色板+光照,2MiB | bincode(Chunk) 直传 | 自定义客户端无需 MC 格式 |
| 出站优先级 | 4 级 | 2 级 | v1 阶段够用,升级位保留 |

## 6. demo 论证

无可演示的物理/数值未知项(帧解析/背压/流差分是纯工程逻辑,Rust TDD 直接覆盖;
激活半径与票据语义 REQ-001/002 已验证)——照 REQ-002 先例,不设 Python demo。

## 7. 模块归属(蓝图 §7 增量)

| 新东西 | 去处 |
| --- | --- |
| C2S/S2C 协议类型(bincode) | sim-api(新 proto.rs 或 session.rs 增量) |
| 连接/会话/流/背压/keepalive | app::net(app 内模块;REQ-004 MC 桥时若膨胀再拆 crate) |
| StreamChunks 命令 + 观察输出编排 | app::driver |
| 验收套件(内嵌测试客户端) | e2e(或 app 集成测试,任务书定) |

## 8. 决议(2026-10-17 用户拍板:按推荐全部采纳)

1. **协议选型(核心)**:v1 自定义二进制协议(bincode over TCP),**不做真 MC 协议**;
   MC 协议桥(离线单版本,ferrumc-v2 路线)立为 REQ-004 候选。
   推荐 v1 自定义的依据:a) RL 训练对自定义协议效率最优(bincode 直反序列化);b) 真 MC
   客户端呈现我们的物理玩法需要实体包映射+方块表扩充(现仅 3 种内置方块)+光照/调色板,
   是独立大需求;c) 本轮骨架(连接/会话/通道/流)对 MC 桥完全可复用;
   **〔2026-10-18 用户拍板:MC 客户端直连不做——自定义物理实体无法在 MC 客户端忠实
   呈现(无对应实体类型,60Hz 物理运动只能降级映射),不期望 MC 直连;REQ-004 转为
   自研客户端路线,最小可视化 viewer(REQ-004A)先行〕**;
2. **网络并发模型**:与驱动同 runtime,每连接一 task(不用 v1 独立线程模式——
   其动机是主循环非 async,我们不适用);
3. **观察输出形态**:增量(变化实体 + 方块变更)+ 周期全量(周期可配 1~N tick,
   RL 可每拍全量),订阅时一次性全量打底;
4. **玩家是否物理实体**:v1 玩家 = 纯观察者(无碰撞箱、不受力);玩家实体化
   (被爆炸推走、真实碰撞)留玩法 REQ 与决议(涉及操控模型设计,单独拍板)。

## 9. 验收标准(草案)

- **AC-N1 连接闭环**:内嵌测试客户端连入 → Welcome → Subscribe → 观察者表注册,
  MoveObserver 反映到模拟(tick 生效);
- **AC-N2 chunk 流**:进入新区域按 pump 收到 center-out 序的 ChunkData(每泵 ≤16),
  离开收 ChunkUnload;视距钳制生效;
- **AC-N3 命令往返**:挖/放/生成/爆炸经协议到达模拟并生效(物理可见);
- **AC-N4 观察输出**:实体状态增量 + 方块变更按订阅推送;周期全量与增量一致
  (全量 == 增量累积的终态);
- **AC-N5 背压**:慢客户端(不读 socket)下服务器 tick 不受阻;Critical 丢弃 → 断连;
  World 拥塞 → 合并降级不断服;
- **AC-N6 断线重连**:断开 → 观察者移除/票据撤(可能触发 despawn);重连 →
  恢复观察与 chunk 流;多客户端并发互不干扰。

## 10. 未覆盖(v1 明确不做)

- 真 MC 协议/真实客户端(REQ-004 候选)、压缩与加密、鉴权;
- WAN 优化(丢包/RTT 补偿)、WebSocket;
- 玩家物理实体化与操控模型。
