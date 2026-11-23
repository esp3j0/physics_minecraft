# REQ-013: 正式客户端——单机本地优先,多人协议/性能预设计

- 状态:开发中(2026-11-20 定稿;C01a、C01b-1、C01b-2、C01b-3、C02-1、C02-2、C03-1、C03-2、C04 验收通过,见 §9 进度)
- 提出日期:2026-11-19
- 关联讨论:[#9 未来客户端架构](https://github.com/esp3j0/physics_minecraft/issues/9)(结论收敛于本 REQ)
- 关联 demo:`demos/client/`(开工后填)
- 关联代码:`crates/client-core/`、`crates/client/`(单元拆分后定名)

## 1. 背景与目标

REQ-004A 的 macroquad viewer 是"服务器状态 → 插值渲染"的 demo(#9 方案 A),
不是正式客户端地基。用户 2026-11-19 拍板:

1. **产品形态:单机/本地 host 优先**。客户端捆绑完整模拟,零复制协议;
   多人以后走服务器权威路径(#9 结论)。
2. **渲染栈:bevy 重选型**(REQ-004A 预留方向)。viewer 的纯逻辑层可搬运。
3. **本地计算优先**:能本地算的全部本地算。
4. **多人必须提前设计**:协议与性能在单机阶段就按多人约束设计,
   不允许"先做单机、回头重写"。

### 1.1 "本地计算优先"的定形:单份计算,不是双份比对

讨论中曾设想"服务器发碰撞事件 → 客户端本地重算 → 与服务器逐位比对"。
裁定**不采用**,理由:

- **逐位一致只在同机成立**。REQ-001 §9.10 明确关闭 rapier
  `enhanced_determinism`,只保证同机、同二进制、同输入序列下位级一致;
  跨机器 SIMD/FMA/libm 差异使 f32 结果分叉,物理混沌放大后不可对账。
  跨机 lockstep 比对需要定点数或 enhanced_determinism,代价大,不做。
- **碰撞事件是输出不是输入**。本地重建模拟需要全部输入(初态 + 每拍
  意图/方块编辑/生成事件 + 拍序),光发碰撞描述不了摩擦/恢复/堆叠/
  warm-start,晚加入重建不了接触状态(#9 §2.3)。发全部输入就等于
  客户端跑全量模拟——不如直接让它跑。
- **同机双份模拟是纯开销**。既然同机位级一致,客户端进程内直接持有
  **唯一一份** sim(#9 方案 E):没有"比对"动作,没有双份 CPU,
  权威天然唯一。

因此:**单机客户端 = bevy 渲染壳 + 进程内 sim 内核 + 本地策略推理**。
策略推理也本地:`policy` crate(tract-onnx,CPU)已独立成库,
`app/controller/policy_worker.rs` 的"推理不阻塞 tick"工作线程模式
原样复用。多人时策略推理才留在服务器侧(权威/防作弊 + 跨机 NN
浮点同样不位级一致)。

## 2. 权威模型

| 计算 | 单机(本 REQ 实现) | 多人(未来 REQ,本 REQ 只预留) |
| --- | --- | --- |
| 体素编辑、物理、策略推理 | 全本地,客户端进程内唯一 sim | 服务器权威,客户端收快照 |
| 相机、拾取、UI、meshing、渲染 | 本地 | 本地 |
| 玩家自身运动 | 本地(sim 的一部分) | 本地预测 + 服务器校正 |
| 碎片/粒子等纯视觉物理 | 本地 | 本地,允许发散,不回写(#9 方案 D) |
| 远端实体表现 | 不存在 | 插值/外推(#9 方案 A→C) |

硬规则(现在就写进代码结构):

- **客户端逻辑永不回写权威状态**。单机下"权威"就是进程内 sim,
  客户端壳只发命令(C2S)、只消费帧(S2C);多人时同一条规则直接
  变成防作弊边界,不需要重构。
- **不做跨机位级一致假设**。任何"客户端算的结果必须等于服务器"
  的设计一律拒绝;比对只允许出现在同机测试里(见 AC2/AC5)。

## 3. 传输边界(多人预设计的核心)

### 3.1 边界 = 现有 C2S/S2C 消息口径

客户端核与 sim 宿主之间**不发明新私有接口**,直接以
`sim-api::proto::{C2S, S2C}` 为边界消息:

```text
┌────────────────────────┐   C2S    ┌─────────────────────────┐
│ client-core(纯逻辑)    │ ───────► │ sim 宿主                 │
│ 输入→命令、帧→世界视图、 │          │ 单机:进程内驱动          │
│ possess、拾取、脚本      │ ◄─────── │ (Session/Scheduler/      │
└────────────────────────┘   S2C    │  ShardSim + policy)      │
        ▲                           │ 多人:app 服务器 + 网络    │
        │ 渲染/输入接线              └─────────────────────────┘
┌───────┴────────┐
│ bevy 壳(crates/client) │
└────────────────┘
```

- **单机传输 = in-process**:实现 `Transport` trait,进程内把 C2S 交给
  驱动、把 S2C 泵回客户端,**不做序列化**(消息按值传递)。
- **多人传输 = 网络**:同一个 trait 换成 TCP/QUIC 实现,消息加 codec。
  客户端核代码路径两种模式完全一致,多人落地时不改客户端逻辑。
- 现有协议语义原样继承:枚举只追加(REQ-003 决议)、EntityStates
  增量 + 周期全量自愈(REQ-004A ACV4)、一连接一身体(REQ-012 D1/D12)、
  Spawned/Possessed 回执归属(REQ-012 R1)。
- 驱动侧复用 `app` 的命令翻译/校验白名单(net `translate`),
  **单机不绕校验**——命令合法性口径与多人完全一致。

### 3.2 拍与时间(线程模型)

- sim 固定 60 Hz(`dt = 1/60`,与 `app/config.rs` 共识 1 一致)。
- **宿主 = 独立后台 OS 线程**跑 `app::driver`(current_thread tokio
  runtime,与 `viewer::client` 同款模式),固定 60 Hz 步进;**bevy 主线程
  每帧排空 S2C 通道渲染最新态**。不用 bevy `FixedUpdate` 驱动 sim——
  渲染掉帧/卡顿天然不拖慢、不改变 sim 拍序(AC4 由结构保证),且与
  未来多人模式(独立网络线程)客户端侧线程/事件消费模型完全一致。
- 单元测试无时钟步进:`tokio::time::pause()`(app dev-deps 已有
  tokio/test-util)或单拍驱动入口;客户端核不读墙钟(继承 possess
  状态机纪律)。
- 单机渲染**直接取最新**(与 viewer possess 现行口径一致,不插值);
  多人远端实体插值需要的时间戳信息,`EntityStates.tick` 已携带,
  边界消息不需要改。
- **单机观察节奏 = 每拍推**:宿主 `ObserveCadence.interval_ticks = 1`
  (网络默认 10 拍 = 6 Hz,单机不插值下会呈 6 FPS 抽搐;全量周期
  沿用 60)。chunk 泵速单机适度放宽(`PumpConfig.interval_ms` 小于
  网络默认 100 ms)。AC2 对账两侧共用同一 cadence/泵配置。

### 3.2a 宿主装配(复用 app,不拼第二套)

单机宿主**直接复用 `app::driver`**,不另拼轻量 Session:

- `driver::run` 本身零 TCP 依赖(`DriverArgs.observe: Option<ObserveLink>`;
  socket 只在 `net::serve`);命令翻译 `net::translate`(纯函数)、
  chunk 流 `ChunkStreamState`/`StreamLink::spawn` 均独立可复用。
- 复用即零重写获得:命令白名单校验、Spawn ID 高水位、附身表与主体
  清扫、PolicyWorker 推理线程、StartupAgents、redb 持久化屏障与
  checkpoint 轮转。
- 落点 = **`app` crate 新模块 `app::local`**:`OutboundHub::register`
  是 `pub(crate)`,进程内注册本地玩家必须在 app 内;不改动既有
  可见性、不改 driver/net 行为。
- 存档:与服务器同一 `bootstrap` 路径、同一 redb 格式(v5 lz4 bundle),
  零格式改造。世界目录由 `--world-dir` 显式指定;测试/冒烟一律
  `tempfile::TempDir`,**绝不默认触碰仓库根 `data/`**。同一世界目录
  不能被两个进程同时打开(redb 文件锁),打开失败报清晰错误,
  不静默覆盖。

### 3.3 多人预留的协议扩展(本 REQ 定 schema,不实现网络)

按 #9 §4 的复制模型,以下原语在 schema 层设计好、以**枚举追加**
方式落进 `sim-api::proto`,单机路径不消费但不破坏:

| 原语 | 方向 | 语义 | 单机行为 |
| --- | --- | --- | --- |
| `InputAck { seq, tick }` | S2C | 输入确认:客户端命令带 seq,服务器回确认拍 | in-process 驱动同样回(路径一致) |
| `Correction { entity, state, tick }` | S2C | 预测实体的权威校正 | 单机无预测,不产生 |
| `Snapshot` 语义 | S2C | 复用 EntityStates `full=true` 作权威关键帧,不另立包 | 已有 |
| 兴趣管理 | C2S | 复用 Subscribe/MoveObserver + 视距;按 chunk 切比雪夫过滤已有 | 已有 |

命令 seq:现有 C2S 命令不带序号。扩展方式 = 追加
`Sequenced { seq: u64, inner: Box<C2S> }` 包装(只追加、旧包不动),
单机 in-process 也走包装,保证多人落地时客户端不重写输入路径。
**schema 前置到 C01a**(纯数据结构 + codec 往返,几十行),避免
C04 回头改 C01/C02 发送层;C01a 阶段驱动对 `Sequenced` 透明转发
inner(ack 语义 C04 落),`InputAck`/`Correction` 只落枚举与 codec、
不产生。

**带宽预设计**(多人落地时按此实施,本 REQ 只定口径与预算):

- 实体流:增量为主 + 周期全量自愈(已有);位置/速度量化
  (chunk 原点相对坐标 + i16/f16 压缩)留作未来追加包,不改现有包;
- 兴趣过滤:视距方形 chunk 集(已有切比雪夫 center-out 装载);
- 每客户端带宽预算:32 个可见主体稳态 **≤ 64 KB/s**(初始目标,
  多人 REQ 立项时实测校准,不是本 REQ 硬门禁)。

## 4. 性能预算(单机硬门禁 + 多人预留口径)

目标基准机:用户本机(2080Ti 级);数值为初始目标,C01 基线测量后
由架构师修订定值(修订走本文档,不私自放宽)。

| 项 | 预算 | 口径 |
| --- | --- | --- |
| sim 步进 | ≤ 8 ms/拍 @ 基线组合 + 视距 8 chunk | 60 Hz 帧预算 16.6 ms 的一半留给渲染;`Session` 单片基准 |
| 策略推理 | ≤ 2 ms/主体·控制拍 | tract CPU,control_period=2 拍(30 Hz);worker 线程不阻塞 tick(复用 D01b-2 模式) |
| 渲染 | ≥ 60 FPS @ 基线组合 | bevy 壳,视距 8 |
| meshing | 异步,不占渲染线程;单 subchunk 重网格 ≤ 4 ms | bevy AsyncComputeTaskPool;编辑 → 画面 ≤ 2 帧 |
| 输入延迟 | 输入 → sim 生效 ≤ 1 拍;方块编辑 → 画面 ≤ 2 帧 | 单机本地生效,无网络往返 |
| 内存 | chunk 缓存 LRU 上限可配;主体流环形缓冲 | 无界增长 = 缺陷 |
| 多人带宽 | ≤ 64 KB/s/客户端 @32 可见主体 | 预留口径,未来 REQ 校准 |

**基线组合(算力口径,防串行推理超预算)**:64 个**无策略**激活刚体
+ **4 只策略狗**(quad_gpu1)+ 视距 8 chunk。现有 PolicyWorker 是
单工作线程串行推理:控制拍预算 33.3 ms,4 × 2 ms = 8 ms 富余充足;
若 64 只全跑策略则 64 × 2 ms = 128 ms 必然超时降级——**64 只策略狗
不是本 REQ 基线**,batch/多线程推理单列未来优化项,不在 C01 口径内。

性能基线报告(sim ms/拍、推理 ms/主体、meshing ms/chunk、FPS、
内存)是验收件,归档本文档 §11。

## 5. bevy 渲染栈与可搬运性

- **sim 内核保持 bevy 无关**:physics-sim/shard/policy 以库形式被
  宿主驱动,**不用 bevy_rapier3d 插件**(现有 rapier3d 0.36
  内核 + agent 电机/冻结/迁移逻辑原样复用,换插件等于重写内核)。
- **版本与裁剪(Q1 拍板)**:锁定 crates.io 最新稳定版 **bevy 0.20.1**
  (2026-11-20 `cargo search` 核实),写死 Cargo.lock,**只在
  `crates/client` 引用,绝不泄漏到其他 crate**。`default-features = false`,
  首期必开:`bevy_asset`、`bevy_core_pipeline`、`bevy_pbr`、`bevy_render`、
  `bevy_scene`、`bevy_text`、`bevy_ui`、`bevy_winit`、`bevy_window`、
  `bevy_gizmos`(选区线框)、`tonemapping_luts`、`multi_threaded`、
  `png`(截图接 REQ-008 冒烟)、`default_font`、`bevy_ui_render`
  (后两项 C02-1 验收补记,2026-11-20:`default_font` 提供内嵌字形,
  缺它 HUD 文本无字形;`bevy_ui` 特性不含渲染插件,缺
  `bevy_ui_render` 则 UI 只布局不画——两者均为零/单包增量,
  见 #27 R3/R5b/R6 裁定)。暂不开:audio/vorbis、sprite/2d、
  animation、gltf、gilrs(手柄 §8 非目标)、wayland。C02 调试面板
  若只是移植 tools/panel 纯文本 HUD,原生 bevy_ui + bevy_text 够用,
  **不引 bevy_egui**(复杂交互需求出现再议)。
- **glam 版本隔离**:`types` 锁 glam 0.33,与 bevy_math 内置 glam
  不同版——`crates/client` 建薄 `conv.rs` 做 Vec3/Quat 字段转换,
  **不动 types**;世界坐标 → 渲染坐标统一收口一个函数(未来
  Floating Origin 只改一处)。
- **lint 口径**:workspace `unsafe_code = forbid`、clippy 禁
  `Instant::now`/`HashMap`/`HashSet`。client-core 严格遵守确定性核
  lint;`crates/client` 渲染边缘层按 app 同口径对渲染计时/缓存做
  局部 `#[allow]`(逐处注明理由)。
- **编译/帧率保护(Windows)**:根 Cargo.toml 给 bevy*/wgpu*/naga*
  配 `[profile.dev.package."*"] opt-level = 2`(依赖只编一次,
  Debug 运行也能 60 FPS,人工手测与基线才测得准);纯逻辑单测全部
  放 client-core,平时跑核心包测试不触发 bevy 编译。
- **viewer 纯逻辑层搬运**:`client.rs`(零 macroquad)、`possess/`、
  `world.rs`、`script/`、`tools/` 移入 client-core,既有测试全绿
  (AC7);**搬运后 viewer 改为依赖 client-core**(不两处维护同一份
  代码,V01 合并也干净);macroquad 渲染层不搬,bevy 重写。
- macroquad viewer 保留为 demo/测试床直到 bevy 客户端达到功能
  平价;V01(准星拾取附身任意主体,#26)仍落在 viewer,其成果
  (agent_pick、possess 扩展)随纯逻辑层一起搬运。
- 存档:复用 `storage`(redb),单机世界 = 本地存档,格式不新造
  (§3.2a)。

### 5a. 渲染组织(C01b 口径)

- **subchunk 切分**:底层 `voxel::Chunk` 是 16×384×16 全高柱
  (98,304 格 ≈ 192 KB);渲染网格实体按 **16×16×16 subchunk(垂直
  24 段)**组织,单格编辑只脏受影响段,不整柱重扫——AC6 编辑风暴
  与后期贪心合并/AO 都不用重构实体结构。
- **零整块拷贝**:渲染侧镜像用 `Arc<Chunk>`(或按段拆存,每段 8 KB),
  丢给 AsyncComputeTaskPool 只传 Arc 引用或受影响段 + 1 格边框
  (跨块边界剔除);脏队列优先级:**玩家编辑 > 远处新装载**,保
  "编辑 → 画面 ≤ 2 帧"。
- **顶点 AO + 太阳光**:mesher 顺手算基础 4 角顶点环境光遮蔽
  (纯函数,可单测)+ 一盏方向太阳光;方块棱角/洞口深度可读性
  显著提升,人工验收与截图冒烟直观。
- **主体渲染层级**:每连杆一个父 Transform 实体(AgentStates 驱动),
  碰撞形状 Mesh 挂子实体;留调试开关,未来换 glTF 视觉模型或叠加
  碰撞线框零侵入。

## 6. 验收标准

- [x] AC1:单机可玩闭环——bevy 客户端进程内起 sim,飞行相机 +
  chunk 流式装载 + 准星拾取挖/放方块,**零网络依赖**;方块编辑
  ≤ 1 拍本地生效(单元测试 + 人工)。C02-1 人工闭环通过
  (2026-11-20,见 §9;held 键/鼠标转视角注入限制与 MS23 锚归 C02-2)。
- [x] AC2:`Transport` trait + in-process 与 loopback(进程内消息
  队列)两个实现;同一客户端核分别接两种传输跑同一脚本,
  S2C 帧序列**同机逐位一致**(测试)。C04 验收:client-core
  `transport::tests::c04_t7_ac2_dual_transport_bitwise_reconcile` 绿
  (`SequencedTransport<LocalTransport>` 对
  `SequencedTransport<LoopbackTransport<LocalTransport>>`,整段 S2C
  编码字节序列逐位相等,锚点 seq 1..=7 连续 / ChunkData 25 / Spawned 2)。
- [x] AC3:协议追加包(`Sequenced`/`InputAck`/`Correction`)schema
  落地,codec 往返测试;既有 C2S/S2C 包与行为不变(既有测试全绿,
  枚举只追加)。schema 与 codec C01a 落地;C04 补齐 ack 语义:
  两入口剥尽 `Sequenced` 外→内、非 `Reject` 才上行 `NetUp::Ack`,
  驱动以 World 优先级回 `InputAck{seq, tick_now}`;workspace
  1793 passed / 0 failed,枚举零新增变体。
- [ ] AC4:sim 60 Hz 固定拍与渲染解耦;渲染掉帧不改变 sim 拍序
  (测试:人为阻塞渲染 N 帧,sim tick 计数与状态不受影响)。
- [x] AC5:本地策略推理驱动 quad(quad_gpu1),同机与服务器路径
  playback 位级一致(复用 check-policy 同机核对口径 + 回放比对)。
  C03-2 验收:app `dual_path::ac5_dual_path_bit_exact_consistency` 绿
  (quad_gpu1 真实策略包);check-policy quad_gpu1 加载 OK、quad_mj1_ft
  probe 总最大差 2.980e-7;见 §11 C03-2 归档。
- [ ] AC6:meshing 异步不阻塞渲染线程;方块编辑 → 画面 ≤ 2 帧;
  编辑风暴(脚本连挖 64 格)期间 FPS ≥ 55(实测报告)。
- [ ] AC7:viewer 纯逻辑层(client/possess/world/script/tools)
  搬运后既有测试全绿,语义不变。
- [x] AC8:性能基线报告归档 §11,§4 预算逐项实测对照;未达项
  如实报实测值并归因,不私自放宽。

## 7. 图形冒烟场景

bevy 客户端的冒烟接线(截图 + 机器检查通道)随 C01b 建立;机器检查
走进程内 sim 查询(与现有 smoke 同口径:`live_entities`、
`labeled_alive`、`block`、`chunks_loaded`、`no_floating`)。

1. `client_startup`:启动 → 地形可见、相机就位;检查 chunks_loaded
   达视距集、no_floating;截图 expect:地表连续、无空洞。
2. `client_block_edit`:准星挖一格、放一格;检查 block 值;
   截图 expect:编辑处几何变化、无闪烁残影。
3. `client_possess_walk`:P 生成胶囊并附身,WASD 行走;检查
   live_entities=1、位置随意图变化;截图 expect:第三人称胶囊。
4. `client_quad_policy`:生成 quad_gpu1 策略狗;检查
   labeled_alive、位置前进;截图 expect:四足行走姿态。

## 8. 非目标

- **多人网络传输的实际实现**(独立未来 REQ;本 REQ 只定 schema、
  边界与预算,loopback 即止)。
- 跨机器位级一致 / 定点物理 / enhanced_determinism。
- 客户端预测 + 校正的实际实现(只留 Correction schema)。
- 纯视觉碎片物理(#9 方案 D,未来 REQ)。
- 音频、mod/脚本 API 扩展、手柄/移动端输入。
- 替换 macroquad viewer(保留至平价)。

## 9. 单元拆分(种子,任务书另发)

| 单元 | 内容 | 依赖 |
| --- | --- | --- |
| C01a | 协议追加包 schema + codec(Sequenced/InputAck/Correction)+ `app::local` 进程内宿主 + client-core crate 与 Transport trait + 无头对账测试(AC2/AC3 的 schema 与宿主部分);**无 bevy** | — |
| C01b | bevy 壳 + 飞行相机 + subchunk 渲染(Arc/脏优先级/AO/太阳光)+ 冒烟接线 + 性能基线(AC1/AC4/AC6/AC8) | C01a |
| C02 | 体素交互(拾取/挖/放)+ possess 状态机搬运 + HUD;viewer 纯逻辑层移入 client-core、viewer 改依赖 client-core(AC7) | C01b |
| C03 | 主体渲染(BodyShapeTable 搬运)+ 本地策略推理(quad_gpu1)(AC5) | C01b |
| C04 | Sequenced ack 语义(驱动回 InputAck)+ loopback 传输对账(AC2 收尾) | C01a |
| C05 | AC4 渲染/sim 解耦验证:`stall_render` 渲染层阻塞步 + headless 双跑对账(阻塞窗口不改变拍计数与状态)+ 真实线程阻塞测试 + 冒烟(AC4 收尾) | C01b、C04 |

V01(#26,viewer 上的准星拾取附身)独立先行,成果并入 C02 搬运。

**进度**:

- C01a ✅ 验收通过(2026-11-20,含 R1 返修):提交 ebc740f(实施)+
  e022f1f(R1 补 T9 双队列竞争排序锚点)。T1–T9 全绿;架构师自注变异
  N1/N2/N3 击杀,M3(try_pop 先 world 后 critical)R1 后被 T9 击杀;
  门禁(sim-api/app/client-core/viewer 测试 + clippy + fmt + e2e 编译)
  架构师复跑全过。验收与 R1 裁定见 #27。
- C01b-1 ✅ 验收通过(2026-11-20,含 R1 返修):提交 ba282fa(实施)+
  9a0a52b(R1 补 MB4 邻 chunk 连带标脏锚点)。世界镜像(互斥双 BTreeSet
  脏队列、Arc 写时复制)+ subchunk 网格生成(固定面序/外平面三采样顶点
  AO/方向亮度烘焙)+ bevy 0.20.1 壳(CLI/飞行相机/同步 32 段每帧上传/
  F2 截图)。MB1–MB4/MA1–MA5/AC4-t 全绿;PM1–PM5 与架构师 N2/N3/N4 击杀,
  N1/N5(insert/remove 邻 chunk 连带标脏无锚点)R1 后被 MB4 击杀;
  门禁(app 203/client-core 11/sim-api 66 + clippy + fmt)复跑全过;
  人工验收审图通过(地表连续、相机位移、F2/Esc/CLI 守卫)。
  Cargo.lock 新增 329 包全为 bevy 系传递依赖;`zstd_rust` 为
  tonemapping_luts 强制 zstd 后端的最小补加,已认可。
  遗留:mesh 资产替换时旧 handle 未 remove,归 C01b-2 收口。
  验收与 R1 裁定见 #27。
- C01b-2 ✅ 验收通过(2026-11-20,含 R1 返修):提交 a216523(实施)+
  53039dc(R1 补 MS5 异步 meshing 接线锚点)。纯逻辑层搬运
  (viewer client/world/script → client-core,re-export 保旧路径,
  viewer 220→115、client-core 11→93,总数不变);异步 meshing
  (ChunkSnapshot 零拷贝快照 + epoch 世代计数 + requeue +
  AsyncComputeTaskPool 每帧≤8 spawn/collect 世代判定上传/
  旧 Mesh 句柄回收 + MeshStats 帧计数);client --script/--out
  冒烟模式 + xtask smoke --client + 两场景(client_startup/
  client_block_edit,机器断言 uploads>=1 且 1<=max_edit_upload_frames<=2)。
  MS1–MS5 全绿;PM6–PM13 击杀;架构师 N6(collect 忽略过期)/
  N7(替换不回收句柄)存活 → R1 无头 bevy App 接线锚点 MS5 四例后击杀。
  门禁(client-core 93/client 5/viewer 115/app/sim-api/e2e 54 二进制 +
  clippy + fmt + smoke --selftest)复跑全过;冒烟 manifest 与三张截图、
  人工回归两张截图目视通过。CARGO_INCREMENTAL=0 纪律:rustc ICE
  (增量指纹损坏)后全单元沿用。遗留:chunk 卸载路径句柄回收无机器锚点
  (两场景不触发卸载),归 C02/C03 视距移动场景;uploads 含空段且受帧对齐
  ±8 抖动,C01b-3 基线口径需注明。验收与 R1 裁定见 #27。
- C01b-3 ✅ 验收通过(2026-11-20,含 R1/R1b 返修):提交 2893347。
  性能测量钩子(policy_worker timed_act/infer_us、controller 仅 Ok 路径累计、
  client-core perf.rs 纯函数、manifest perf serde(default)、S2C::PerfStats
  判别 14 追加 golden 不动、driver 订阅门控 10 拍、FpsHistoryRes 7200 上限 +
  绝对索引标记 + reset、脚本 perf_reset 全链路、w03 不变量扩展、dev
  opt-level=2)+ 基线/编辑风暴两场景 + xtask 机器断言。R1:perf_reset 稳态窗、
  before 截图前移、80 拍动态相采样窗(凝固桥 rest+20 口径修订,手算裕量≥16 拍);
  R1b:LabeledAlive = entities ∪ agents。MS7–MS13 全绿;PM14–PM21 击杀
  (PM19/PM20 架构师补跑,PM19 超时遗留变异事故已还原);架构师 N8/N9 击杀。
  门禁复跑全过(e2e 33 二进制;acn2 首轮偶发 flake 复跑绿,环境竞态另案);
  四场景冒烟 failed 0;审图闭环(白盒 = 凝固桥 stone 写入,旧轮遗留截图误判
  已按 manifest+mtime 核对)。实测归档 §11:sim_p95 3.3231≤8 ✓、策略
  0.09640≤2 ✓、mesh_max 0.737≤4 ✓、编辑→画面 1≤2 ✓、稳态 fps_mean
  60.0223≥60 ✓;AC6 风暴 FPS 本机 780M 跨轮 52.28–56.86 不稳定达标,
  如实记录待 2080Ti 基准机复测,不私自放宽。遗留:storm 基准机复测;
  卸载句柄锚点归 C02/C03 视距移动场景。验收与 R1/R1b 裁定见 #27。
- C02-1 ✅ 验收通过(2026-11-20,含 R2–R6 裁定):提交 8c7095b(实施)+
  b3b28f2(R3 default_font)+ 997dc36(R4 HUD 组件集 + MS18)+
  1078fe1(R6 bevy_ui_render,lock 恰 +1 包)。交互接线(每帧拾取/
  1–4 选材/LMB 挖/RMB 放/M 锁定切换/P 附身圆环/滚轮语义路由)+
  possess/tools 纯逻辑搬运 client-core + HUD 五行 + MS16b/MS18/MS20/
  MS22 锚点 + client_interact 冒烟场景(19 步)。门禁复跑全过
  (client-core 199/client 9/viewer 19/app/messaging/sim-api + e2e 33
  二进制 + clippy 七包 + fmt);PM22–PM26 5/5 击杀,架构师 N10/N11 击杀
  并还原。R2:F5 tempfile 追认、F6 编辑回环 3 拍→wait 4、F7 相机 y=-56;
  R3/R4/R6:HUD 三层静默失败(缺字形/裸 Text 无 Node 不布局/bevy_ui
  不含渲染插件)逐层真机截图复核修复,根因属需求清单遗漏,已补记 Q1。
  AC1 人工真机闭环通过:HUD 五行、放 dirt/挖/选 stone 再放、P 附身
  (Possessed+第三人称)、再 P 解除(HasBody)、F2 落盘审图、Esc exit 0。
  限制如实记录:computer-use press_key 点按语义使 held 键(WASD/
  Shift/Space)与鼠标转视角无法人工注入(bevy 同帧 down+up 只置
  just_pressed),fly_camera_system held 平移/转视角无 headless 锚 →
  C02-2 追加 MS23(期望值架构师手算);附身意图行走需策略模型归 C03。
  遗留归 C02-2:LRU 上限 + 视距移动场景 + 卸载句柄机器锚、脚本 shot.hud
  HUD 显隐接线 + 冒烟可见性锚(R6 延期)、MS23 fly 锚。验收与 R2–R6
  裁定见 #27。
- C02-2 ✅ 验收通过(2026-11-21,含 R2–R4 裁定):提交 2dcf005。
  D1 ChunkMirror LRU(last_use/use_clock;enforce_cap 保护集外按
  (last_use,pos) 逐出;remove_chunk 清被逐区块自身脏标、保留 epochs、
  标记 loaded 邻居;MS25 t1 epoch=2、t5 dirty=24 架构师手算);
  D2 client_view_move 8 步场景 + xtask 机器断言(unloads>=1、
  evictions==0、assets==entities+baseline 三值等式、uploads>=1);
  D3 脚本 shot.hud 显隐 + 延迟一帧生成(R3:截图帧只切 Visibility 并记
  pending_spawn,次帧帧首 spawn Screenshot,早于两个 early-return 门;
  MS24 headless 锚 u1=0/u2=1)+ client_interact 第 21 步 hud=true
  视觉锚(审图五行白色 HUD 通过);MS23 fly headless 锚落地。
  R2:assets 与 entities 差 5 为 Bevy 内置不朽 Mesh(decal quad、
  fog plane+cube、PointList fallback、SpriteMesh 无条件首帧 quad),
  非产品泄漏 → PostUpdate `.after(AssetEventSystems)` 一次性采样
  assets_baseline(N13 证明挂载顺序必要)。R4:批跑偶发 72B 截图 =
  1x1(窗口创建物理尺寸报 0、wgpu 钳交换链),环境偶发、未归因产品
  代码;补 xtask verify_client_screenshots 解 PNG IHDR 宽高 >=16
  门禁堵"空图冒烟绿"缺口(N14)。门禁:client-core 204/client 12/
  viewer 19/app/messaging/sim-api/xtask + e2e 33 + clippy workspace 0
  + fmt 0 + selftest 5/5 + 六场景冒烟 exit 0 且全图 1280x720;
  PM27–PM31、N12–N14 全击杀并还原。实施归属如实:agy 4 次派发均
  print-timeout(worktree 存活、代码落地),门禁/变异/归因/裁定由
  架构师完成,含还原 agy 遗留 PM30 变异。验收与 R2–R4 裁定见 #27。
- C03-1 ✅ 验收通过(2026-11-22,含 R1/R2 机械强化):提交见 #27 回报。
  client-core `entities.rs` EntityMirror(AgentStates/EntityStates 的
  full 重建与增量 upsert+removed、EntityShapes 先到挂载)+ `bodies.rs`
  BodyShapeTable 自 viewer 搬运;client `entities.rs` 每帧四段对账
  (过期 agent despawn、agent spawn/附身青绿换材、compound 变更 solid
  重建、solid spawn/位姿更新;despawn 同步 meshes.remove 1:1)、
  `solid_mesh` 逐 cell 0.9 立方合并;MS27 t1–t4/MS28 t1–t2/MS29 锚点;
  xtask `--client` 八场景门禁;新场景 client_fall/client_possess_walk。
  门禁复跑全过:四包测试 313(client-core 212/client 26/viewer/xtask)
  + e2e 全二进制 + clippy 四包 0 + fmt 0 + selftest 5/5;八场景冒烟
  exit 0、failed_checks 全 0、mesh 三值等式全 True。PM32–PM35 与
  架构师 N16(removed 漏清 shapes 表)5/5 击杀并还原,还原字节比对一致。
  R1 机械强化:镜像就绪早于服务端可编辑/可 spawn(fall SetBlock 丢弃、
  possess_walk spawn 回执超时)→ 前置 60 拍;异步 meshing 与 UI 文本
  不按拍计 → fall before/pile 与 interact hud 截图前 +600 拍墙钟余量。
  R2 裁定(midfall 空帧,~50% 偶发):查证链 = SOLDIAG 镜像/渲染主世界
  正确 → 截图拍号按调度(tick 706=dig+20,live_entities=1)→ spawn 墙钟
  早于捕获 316ms 且无 despawn churn → 连拍 m20 空/m26 起可见 → 屏外
  预挖温暖柱探针 m20 可见(因果确认)。根因 = solid 材质首用触发着色器/
  管线编译使渲染线程停帧 ~0.4s,截图读到旧交换链帧;属 780M 核显驱动/
  环境行为,非 C03-1 实体同步逻辑;驱动层细节未归因。机械强化:
  client_fall 加屏外温暖柱预挖 +600 拍,强化后 midfall 可见 3/3
  (probe5/gate-r3/gate-r4)。同轮修 edit_storm before 截图落初始
  meshing 窗(C01b-3 遗留,纯天空色 15061B)→ wait_until+600 拍,
  强化后 before 可见、after 沟槽成立,风暴遥测与旧批一致
  (storm_frames 14–16、fps_storm_min 50.7–71.4)。审图:fall before
  4 格石柱/midfall 3 格蓝簇悬空与地面分离/pile 凝回石色无残影;
  possess_walk 青绿胶囊第三人称跟随;interact 21-hud 五行 HUD。
  磁盘:D 盘再被 PDB 占满(余 172K,LNK1318),按已批准先例删
  target/debug 下 387 个可再生 PDB 释放 57.9G。验收与 R1/R2 裁定见 #27。
- C03-2 ✅ 验收通过(2026-11-22,含 R4 机械强化):提交 3daa475(实施)、
  f2714ab(R4 钉死)。app `driver.rs` 生成即挂(C2S::SpawnAgent 与 startup
  B03a 同口径 `attached_at = now.next()`、pending 按 EntityId 排序、挂载
  意图清零)+ 策略哈希门(policy_body_hash 不符拒挂)+ `controller/mod.rs`
  drive_policies 到期/frozen/skip 计数;`local.rs` T-A2–T-A5 锚点;
  `dual_path.rs` AC5 同机位级一致(quad_gpu1);xtask check-policy 同机
  核对 + probe 核对。门禁复跑全过:workspace 1785 passed / 0 failed +
  clippy 0 + fmt 0;check-policy 双资产 OK(mj1_ft probe 2.980e-7);
  冒烟 client 9/9 + server 16/16 + req012 9/9 + selftest 4/4(DPI 对齐
  环境,见 §11)。架构师变异轮:M2/M3/M-Y1–M-Y4 共 6 击杀;M1(哈希门)/
  M4(挂载拍 now vs now.next())存活 → 归因(零 dof 静默 skip 的
  policy_no_state 链 / drain 丢弃生成拍 AgentStates)→ R4 钉死 t_a3
  `policy_no_state == 0` 与新增 t_a7 `attached_at == first_tick`,复注
  双红(60≠0 / 14≠15);M5(drain 在 drive 前/后)/M7(pending 排序)
  裁定等价变异(attached_at=now.next() 使新槽生成拍必不到期;slots 为
  BTreeMap,迭代序恒 EntityId 序)。R1 裁定:意图行走场景改用
  quad_mj1_ft(quad_gpu1 开环偏航漂移画圈,实测记 REQ-012 §15.3/§15.4;
  AC5 双路径仍用 quad_gpu1)。审图:quad 四图洋红行走 → 青绿附身转 →
  青绿转回 → 洋红恢复,全程站姿无翻倒。磁盘:D 盘再被 PDB 占满
  (LNK1318),按已批准先例删 target/debug 下 131 个可再生 PDB 释放 33G。
  验收与 R1–R4 裁定见 #27。
- C04 ✅ 验收通过(2026-11-23,R1 裁定后):提交 8ed1d72。`net.rs`
  `unwrap_sequenced` 剥尽外→内、`handle_frame` 非 `Reject` 才按序上行
  `NetUp::Ack`;`driver.rs` Ack 臂 `push_world(InputAck{seq, tick_now})`;
  `local.rs` send 同口径 + `spawn_in_current_runtime` 转 pub;
  client-core `transport.rs` `SequencedTransport`(seq 从 1 起、
  Hello/Goodbye 透传、追踪 last_ack)+ `LoopbackTransport`(send 同步
  flush 零相位差);T7 = AC2 双传输位级对账(整段 S2C 字节序列逐位相等);
  `engine_commands_to_c2s` 收口渲染/headless 共用映射;e2e T8 TCP
  Sequenced→Pong+InputAck。门禁复跑全过:workspace 1793 passed /
  0 failed + clippy 0 + fmt 0;冒烟 client 9/9 + req012 9/9 + selftest
  通过(DPI 对齐环境)。架构师变异轮:M-A(ack 拍号−1)/M-B(World→
  Critical)/M-D(unwrap 序反转)/M-E(seq 从 0 起)/M-F(Goodbye 被
  包装)/M-G'(loopback 延迟 flush)共 6 击杀;M-H(Reject 仍推 Ack)
  裁定等价变异(`disconnect_locked` 同步 `hub.unregister`,驱动消费时
  `push_world` 查无接收者静默丢弃,可观测行为不变)。R1 裁定:原任务书
  T3/T4/T7 与现有代码三处实质冲突(Spawned 走 Critical 先于 World ack、
  tick() 先于 consume_net_up 致 SetBlock 跨拍、loopback 延迟 send 相位差
  及 ChunkMirror/bodies_dir/MoveObserver 前置),实施方停机查证属实后
  修订重派。首批冒烟 client_interact 偶发 dig 回传 check 失败一次
  (341 拍仍 stone),单跑与整批复跑均绿,原因未归因。审图:quad 四图
  洋红行走 → 青绿附身转 → 青绿转回 → 洋红恢复,全程站姿无翻倒。
  磁盘:D 盘再被 PDB 占满(LNK1318),按已批准先例删 target/debug 下
  82 个可再生 PDB 释放 25G。验收与 R1 裁定见 #27。

## 10. 决策记录与开放问题

已拍板(用户 2026-11-19 立项;2026-11-20 定稿):

- D1:单机/本地 host 优先,客户端捆绑服务器;
- D2:bevy 重选型;
- D3:本地计算优先 = **单份计算客户端持有**(否决双份比对与
  碰撞事件驱动);策略推理单机本地化;
- D4:多人协议/性能提前设计——边界用现有 C2S/S2C 口径、
  追加包 schema 先行、性能预算先行;
- D5(Q1):bevy 0.20.1 锁版本、default-features=false 裁剪、
  只在 crates/client 引用;glam 0.33 隔离走 conv.rs;dev profile
  依赖 opt-level=2;不引 bevy_egui(纯文本 HUD 够用);
- D6(Q2):宿主直接复用 `app::driver`(独立后台线程 +
  current_thread tokio),落 `app::local` 模块;不用 FixedUpdate
  驱动 sim;单机 cadence interval_ticks=1、chunk 泵放宽;
- D7(Q3):存档与服务器同一 bootstrap/redb 路径;--world-dir
  显式指定,测试用 TempDir,不碰仓库根 data/;redb 文件锁冲突
  报清晰错误;
- D8(Q4):多人输入路由留未来 REQ,本 REQ 只保协议边界不破坏;
- D9:渲染按 16³ subchunk 组织 + Arc 零整块拷贝 + 脏优先级 +
  顶点 AO + 方向太阳光 + 连杆父子 Transform 层级(§5a);
- D10:viewer 搬运后改依赖 client-core,不两处维护;
- D11:性能基线组合 = 64 无策略刚体 + 4 策略狗(§4;64 只策略狗
  超单线程推理预算,不是基线);
- D12:Sequenced/InputAck/Correction schema 前置 C01a,C04 只补
  ack 语义与 loopback 对账。

开放问题:无(Q1–Q4 已拍板;实施中发现的新冲突按纪律停下回报)。

## 11. 性能基线报告(验收时填)

**C01b-3 归档(2026-11-20,验收通过)**。实测环境:用户本机 AMD 780M 核显
(非 §4 目标 2080Ti 级基准机);CARGO_INCREMENTAL=0、dev opt-level=2。
数据源:四场景冒烟 manifest(target/c01b3-r1-smoke-out/,failed_checks 全 0);
实施提交 2893347;实施自报 #27 comment 7046295876,架构师裁定 comment 7046312114。

基线组合口径:vd8 + 4 只 quad_gpu1 策略狗 + 64 无策略刚体。采样窗 =
spawn 后 80 拍**动态相**(perf_reset 重置后):凝固桥(REQ-001 §9.5)最早
rest+20 拍移除刚体,手算下落 8m≈76.6 拍 → 最早凝固≈96.6 拍,窗末 64 刚体
必存活(裕量≥16 拍);"64 静态刚体稳态"按游戏设计不存在,MarkActor 不对
客户端开放,无豁免路径。整轮 fps_mean 含初始装载停顿,不作稳态判据。

| 项 | §4 预算 | 实测(基线窗) | 判定 |
| --- | --- | --- | --- |
| sim 步进 | ≤8 ms/拍 | sim_p95 3.3231 ms(60 样本) | ✓ |
| 策略推理 | ≤2 ms/主体·控制拍 | 0.09640 ms/主体(2616 次推理) | ✓ |
| meshing 单段 | ≤4 ms | mesh_max 0.737 ms(7008 段) | ✓ |
| 编辑→画面 | ≤2 帧 | 1 帧(storm 与 block_edit) | ✓ |
| 稳态渲染 | ≥60 FPS | fps_mean 60.0223(fps_min 54.089) | ✓ |
| 风暴 FPS(AC6) | ≥55 | 本轮 56.856;跨轮 52.28/54.48/54.87/54.17/56.86 | 本机波动,不稳定达标 |

其余场景:startup fps_mean 57.485/fps_min 7.784(初始装载停顿,仅参考);
block_edit fps_mean 57.999/fps_storm_min 52.035(含中间截图 readback,仅参考);
edit_storm fps_mean 59.875/uploads 605。uploads 含空段且受帧对齐 ±8 抖动
(C01b-2 遗留口径)。

**未达/波动项归因(不私自放宽)**:AC6 风暴 FPS 在 780M 核显上跨轮不稳定达标;
截图 readback 停顿已证实会拉低窗内 FPS(窗内含 before 截图 20.11 vs 无截图
55.13,故 before 截图前移到采样窗之前);核显带宽/帧调度抖动部分未跨机验证,
**未归因**,待 2080Ti 级基准机复测后由架构师走 §4 修订流程定值。

审图:baseline 截图 09-baseline.png(tick 1310、live 64、chunks 289)顶视棕色
地表连续铺满全帧,与 expect 一致(主体/刚体渲染属 C03)。白盒归因:spawn 默认
半格 stone 立方体,凝固桥 rest+20 拍写体素并移除刚体 → 白盒只出现在凝固波后
的截图;复用输出目录遗留旧轮截图曾致误判,已按 manifest+mtime 核对闭环。

锚点与变异:MS7–MS13 全绿;PM14–PM21 击杀(PM19/PM20 架构师补跑,PM19 曾有
超时遗留变异事故,已还原);架构师自注 N8/N9 击杀。e2e acn2 首轮偶发 flake
(start_paused 与 spawn_blocking 竞态),单跑/整二进制/全量 33 二进制复跑全绿,
非本单元缺陷,复发另案。

**C03-1 归档(2026-11-22,验收通过)**。数据源:八场景门禁批
target/c03-1-gate-r3/(failed_checks 全 0)+ 补跑批 gate-r4(edit_storm/fall
时序强化后)。环境同 C01b-3(780M 核显、CARGO_INCREMENTAL=0、dev opt-level=2)。

| 项 | §4 预算 | 实测(perf_baseline 窗) | 判定 |
| --- | --- | --- | --- |
| sim 步进 | ≤8 ms/拍 | sim_p95 3.9467 ms(60 样本) | ✓ |
| 策略推理 | ≤2 ms/主体·控制拍 | 0.10099 ms/主体(2616 次推理) | ✓ |
| meshing 单段 | ≤4 ms | mesh_max 0.772 ms(6952 段) | ✓ |
| 编辑→画面 | ≤2 帧 | 1 帧(fall max_edit_upload_frames=1) | ✓ |

其余场景参考:fall fps_mean 114.26(fps_min 7.05 含首渲管线编译停顿,
R2 已用屏外温暖柱把该停顿移出 midfall 截图窗);edit_storm fps_storm_min
50.709/storm_frames 16(跨批 14–16、50.7–71.4,AC6 本机波动口径沿用
C01b-3,待 2080Ti 复测);possess_walk sim_p95 0.7799、mesh_max 0.392。
整轮 fps_mean 含初始装载停顿,不作稳态判据(口径同 C01b-3)。
C03-1 不含策略接线变更(perf_baseline 沿用 C01b-3 四狗组合);
本地策略 attach-on-spawn 与双路径位一致归 C03-2。

**C03-2 归档(2026-11-22,验收通过)**。数据源:DPI 对齐环境冒烟批
target/smoke/client-20261122-173911/(perf_baseline + quad_policy)、
client-20261122-174036/(client 七场景)、server 批 gate_smoke_sDE_dpi
(16/16)与 gate_smoke_sF_dpi(req012 9/9)、selftest 4/4;failed_checks 全 0。
环境同 C01b-3(780M 核显、CARGO_INCREMENTAL=0、dev opt-level=2),另加
`__COMPAT_LAYER=DpiUnaware` 使窗口物理分辨率对齐 1280×720(见环境注记)。

| 项 | §4 预算 | 实测(perf_baseline 窗) | 判定 |
| --- | --- | --- | --- |
| sim 步进 | ≤8 ms/拍 | sim_p95 3.4058 ms(60 样本) | ✓ |
| 策略推理 | ≤2 ms/主体·控制拍 | 0.10491 ms/主体(2616 次推理) | ✓ |
| meshing 单段 | ≤4 ms | mesh_max 0.521 ms(7040 段) | ✓ |
| 编辑→画面 | ≤2 帧 | 1 帧(block_edit/edit_storm/fall/interact max_edit_upload_frames=1) | ✓ |
| 稳态渲染 | ≥60 FPS | fps_mean 256.18(fps_min 196.60) | ✓ |
| 风暴 FPS(AC6) | ≥55 | 本轮 edit_storm 245.01(storm_frames 20)、block_edit 79.52 | 本机本轮达标;跨机定值仍待基准机 |

quad_policy 窗:fps_mean 233.89 / fps_min 49.54(含附身转向与截图 readback
停顿,仅参考)、sim_p95 1.2897、policy 0.26720 ms/主体(1459 次推理)、
mesh_max 0.588;相位拍 adopt 628 / possess 927 / 转回 intent 1648 /
unpossess 2369 / 恢复行走 2.0 m 核对 2921 全 ok。整轮 fps_mean 含初始装载
停顿,不作稳态判据(口径同 C01b-3)。AC6 跨轮波动口径沿用 C01b-3/C03-1:
780M 核显非 §4 基准机,最终定值待 2080Ti 级复测后走 §4 修订流程。

环境注记(DPI):架构师 Bash 启动上下文为 DPI-aware(1.75×),客户端逻辑
1280×720 被放大为物理 2240×1260(3.06× 像素),780M 核显批窗 fps ~16;
agy CLI 与 C03-1 gate-r3 为 1280×720。以 `__COMPAT_LAYER=DpiUnaware` 对齐后
1280×720、fps 233–256(与基线同量级),本轮验收冒烟全部在对齐环境复跑。
fps 非机器门禁;批窗 fps 争用部分未归因。

AC5 证据:dual_path 位级一致绿 + check-policy quad_gpu1 加载 OK /
quad_mj1_ft probe 2.980e-7。变异与锚点见 §9 C03-2(6 击杀、M1/M4 经 R4
钉死复注双红、M5/M7 等价裁定)。审图:173911 quad 四图(11-flat-walk 洋红
站姿行走 / 20-possessed-turn 青绿 / 23-turned-back 青绿转回 / 29-resumed
洋红恢复站姿),架构师亲审裁剪图,无翻倒/残影。