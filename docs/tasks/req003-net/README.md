# REQ-003 实现任务书(总览)

- 状态:已定稿(架构师签发;需求 [REQ-003](../../requirements/REQ-003-network.md)
  2026-10-17 用户拍板,决议 1-4 全部采纳)
- 实现方式:glm-5.3-flash 写核心(00-03),qwen 写验收套件(04)+ 独立评审;
  cargo 门禁;架构师终审。

## 任务顺序(依赖序,串行执行)

| # | 任务书 | 内容 | 实现模型 | 门禁包 |
| --- | --- | --- | --- | --- |
| 0 | [00-proto-observe.md](00-proto-observe.md) | C2S/S2C 协议类型 + 观察输出 API(实体表/凝固方块) | glm | sim-api physics-sim shard |
| 1 | [01-connection.md](01-connection.md) | app::net 连接层(监听/帧/会话/翻译/keepalive) | glm | app |
| 2 | [02-chunk-stream.md](02-chunk-stream.md) | 每玩家 chunk 流(pump/差分/限额/回执) | glm | app |
| 3 | [03-observe-backpressure.md](03-observe-backpressure.md) | 观察输出编排 + 出站背压 | glm | app |
| 4 | [04-acceptance.md](04-acceptance.md) | AC-N1~N6 验收(内嵌测试客户端) | qwen | e2e |

## 公共完成定义

同前几轮:严格 TDD(先红后绿);门禁 = 门禁包 `cargo test --all-targets` +
`cargo clippy -D warnings` 退出码 0;只改任务书【交付物】与【授权改动】列出的文件。

## 架构师实现决策(不得偏离;需求依据 = REQ-003 §4/§8)

1. **协议类型**(`sim-api/src/proto.rs`,bincode serde):C2S/S2C 枚举,**追加式**
   (声明序即版本序);`EntityStates { tick, states, removed: Vec<EntityId> }`
   ——增量的删除通知显式携带;命令白名单:客户端可发的 SimCommand 仅
   SetBlock/SpawnDynamic/RemoveEntity/TeleportEntity/Detonate
   (SetObserverPosition 走 MoveObserver 专用包;MarkActor 服务端策略,不对客户端开放);
2. **帧格式**:4 字节 LE 长度前缀 + bincode;上限 2 MiB(超限断连);v1 不压缩;
3. **观察输出 API**(任务 00 落):`ShardSim::entity_states(shard) ->
   Vec<(EntityId, EntityState)>` 全表 → Scheduler/Session 透传;
   `Scheduler::take_solidified() -> Vec<(ShardId, BlockPos, BlockId)>`
   (tick 内收集凝固写入,两拍之间取,恰一次);
4. **翻译层纯函数**(app::net):C2S → DriverCommand/网络事件映射一处收口,
   单元测试直接喂 C2S 断言输出(不涉 socket);
5. **keepalive**:C2S::Ping / S2C::Pong;服务端读超时(默认 10s,配置)无任何
   入站帧 → 断连;
6. **出站背压**:每连接两级有界队列——Critical(Welcome/Bye/Pong,容量 64)
   与 World(ChunkData/EntityStates/BlockUpdates,容量 512);Critical 满 → 断连;
   World 满 → 丢最旧(EntityStates 天然 latest-wins;ChunkData 丢失由客户端
   重新 Subscribe 修复,doc 注明);
7. **StreamChunks 驱动处理**:tick 末端编排新增第 ④ 步——处理挂起的
   StreamChunks 请求(load → provider.get 批量;unload → 无服务端动作),
   oneshot 回复(下一编排点,不阻塞 tick);
8. **观察输出编排**:每订阅玩家维护 `last_sent: BTreeMap<EntityId, EntityState>`
   副本,输出周期(config,默认 10 tick)差分;全量周期(config,默认 60 tick)
   或订阅时整表;距离过滤 = 实体位置距玩家 ≤ view_distance×16m;
9. **BlockUpdates 源** = 驱动镜像的外部 SetBlock 成功发送 + take_solidified,
   广播给视距覆盖该格的订阅者;
10. **测试客户端**(e2e/集成测试内嵌):tokio TcpStream + 帧编解码复用 app::net
    的公开 codec(授权 pub(crate)→pub 测试用或复制小段,任务书 04 说明);
    每测试独立端口(127.0.0.1:0 由 OS 分配,监听方回传实际端口)。
