# REQ-014 多人联机:权威服务器 + LOD 分级状态流

- 状态:**定稿**(2026-11-26 用户拍板;[#28](https://github.com/esp3j0/physics_minecraft/issues/28);
  架构师合稿 gemini 独立分析;M01 已验收 `11229ab`;M02a-1 已验收 `4fe3a7b`;
  M02a-2 已验收 `8e58db9`+`1722640`;M02b 已验收 `ecb9654`;M03 已验收 `f117835`;
  M04a 已验收 `8b5c112`;M04b-1 已验收 `6db0333`;M04b-2 已验收 `d28654e`)
- 上游:REQ-013 §2/§3(多人预设计承诺:权威表、Transport trait、枚举只追加、
  Sequenced/InputAck/Correction 预留、ObserveCadence、兴趣裁剪、≤64KB/s 初始目标)
- 独立分析:gemini + 架构师双独立稿合稿;逐条裁决与代码核实见 §12 索引
  (裁决全文工作稿不入库,归档于 #28 评论)
- 已拍板取舍(用户 2026-11-26):① v1 无预测(自身 60Hz 权威专流,远端插值;
  预测+校正=可选 M06);② v1 判据 2–4 人合作,设计面向 10+ 扩展;③ TCP 先行
  (复用现 framing,零新依赖;QUIC 升级位);④ LOD 分级数据包为必选项
  (离远简包、靠近详包),性能与 10+ 扩展前置设计。

## 1 范围与规模

v1 交付:局域网/低 RTT 下 2–4 人合作同一世界:编辑互见、附身互斥、断线清扫、
重连自愈、分级状态流与带宽达标。设计边界面向 10–16 连接:每连接状态隔离、
编码/泵送与 sim 解耦、人群硬帽、错峰调度;10/16 人合成负载为 M05 实测项,
其曲线决定单服上限与扩容项(M05b),不在 v1 判据内硬承诺。

非目标(v1):客户端预测与校正(M06 可选,且仅限单连杆身体/根位移近似);
公网高 RTT 手感保证(手感=RTT,记录在案);跨机器逐位一致(权威单 sim,
客户端只镜像);语音/聊天/账号体系;反作弊 beyond 权威表(客户端永不写权威)。

## 2 权威与所有权(承 REQ-013 §2,不改)

- 体素/物理/策略推理全权威在服务器;服务器跑**一份** sim,客户端增加的是
  编码/泵送成本而非 sim 成本。客户端输入经 C2S 意图上行,服务器消费。
- 附身表 `possessed_by`/`possessor_of`(driver.rs:863–866,per InputSource
  严格互逆)为 LOD L0 档与专流 cadence 的单一来源;一连接一身体、已被他人
  附身拒绝、断线强制解除(现网语义保留,清扫缺陷修于 M03,见 §7 F4)。
- redb 单写者归服务器;**网络模式客户端不写本地存档**(单机模式不变)。
- 主体不进 EntityStates(REQ-012 A01 不变量保留);新流全部走追加枚举。
- 观察者:MoveObserver 上行;附身成功服务器移观察者到身体根(driver.rs:1294)
  现网语义保留;**线格式不依赖客户端推断观察者中心**(§4 lite 包内显式基准)。

## 3 传输与会话

- TCP 复用 `app/net.rs` framing:4B LE 长度 + bincode 2.0.1 standard()(varint)、
  FRAME_MAX 2MiB、v1 无压缩。客户端侧 `NetTransport` 实现 REQ-013 Transport
  trait(复用 client-core 读泵+心跳,EOF 合成 Bye);`--connect <addr>` CLI;
  断连 HUD(现交互分支 Bye 落 `_ => {}` 补齐)。
- **能力协商(C2S 判别 13 `SetAgentStream { lod: bool }`)**:新客户端握手后
  显式启用 LOD/紧凑流;未发送者(旧 viewer、全部既有 REQ-012/013 自动化)
  继续收原 AgentStates/ObserveCadence 流,**稳态帧序列逐位不变**(ACM8;
  出站积压时 P2 最新覆盖对全部连接生效,属有意语义)。
- **出站四级队列**(替换现 Critical 64 + World 512 丢最旧,net.rs:305–371):
  P0 Critical(Welcome/Bye/Possessed/Spawned,不丢);P1 可靠事件
  (BlockUpdates/ChunkUnload/EntityShapes/AgentMeta,不丢、满则反压计数);
  P2 状态流(每实体最新覆盖,丢旧不丢新);P3 ChunkData 限速穿插写
  (每拍全局配额,防 98KB/块突发队头阻塞 L0)。QUIC 升级时 1:1 映射多流。
- **每连接上行隔离**:现全局共享 up_tx cap 256(net.rs:1221)改每连接独立
  通道/配额,单坏客户端不得饿死其余连接。
- 收发两端 `set_nodelay(true)`(现网零调用;Nagle+延迟 ACK 会给 60Hz 专流
  加 40–200ms 抖动)。
- 现网缺陷修于 M01(核实证据见 §12):F1 Sequenced 握手门顺序;F2 accept_loop
  句柄运行期无界增长;F3 StreamChunks 热块优先读(§7 F3 同条,正确性 P0)。
- 会话状态收口 `ClientSession`(每连接:兴趣集、lod_table、驻留计时、错峰相位、
  capability、possessed 镜像、差分基准)+ 全局只读 `Arc<TickSnapshot>`(本拍
  快照);为多核并行编码与未来 shard/网关拆分留净边界(§7)。

## 4 LOD 分级状态流(服务器驱动,per-connection × per-entity)

距离口径:3D 欧氏距离平方(与现网实体/主体裁剪同口径 driver.rs:2135,2219;
地形 chunk 流仍 2D 切比雪夫方形,两口径并存是有意的)。vd = 连接视距(chunk 数)。

| 档 | 进入(升档) | 退出(降档) | 频率 | 载荷 |
|---|---|---|---|---|
| L0 附身专流 | `possessed_by[p]==e`,立即、绕过驻留锁 | 解除/移除/切换 | 60Hz(每拍) | Compact(tier=0);M04 前过渡用现网 AgentStates 单实体 |
| L1 近距详包 | d≤32m 且距离排位 ≤K_L1 | d≥40m 或被挤出 K_L1 | 10Hz(6 拍;6 vs 10 由 M05 定) | Compact(tier=1) |
| L2 远距简包 | d≤vd×16m(含 L1 溢出) | d≥vd×16+8m | 2Hz(30 拍) | Lite |
| L3 裁剪 | 超 L2 退出线/销毁 | 重新进入 | 0 | 进 L3 时发 removed |

小视距测试自动钳制:min(32, R_view)、min(40, R_view+8)。

**三重防抖**:① 8m 距离死区(冲刺上限 5.6m/s,client-core/possess/mod.rs:26,
穿越 ≥1.43s);② 升降档驻留锁 DWELL_TICKS=30(0.5s;L0 绕过);③ 人群硬帽
K_L1=8(可配):仅最近 8 只进 L1,其余无论多近溢出 L2;排位互换需
d_新 + 4m < d_旧(防聚堆第 8/9 名高频互换)。

**分档互斥所有权与自愈**:
- 同一 EntityId 任意时刻只属 L0/L1/L2 之一(服务器 lod_table 保证)。
- **换档不发 removed**:仅在新档包中下发该实体,客户端 upsert 改写档位标记;
  只有进 L3 才发 removed(消除换档闪没 1 帧)。
- **full 语义分档**:现网 full=true 清全表(client-core/entities.rs:56)是陷阱;
  新包 full=true 只清客户端标记为该 tier 且不在本包 states 的条目。
  L2 每包 full=true(表小,≈297B/包,天然自愈);L1 full 周期 60 拍;
  L0 无 full(生命周期归 P0 的 Possessed 回执)。丢包/丢换档包最迟一个
  全量周期内对账清除,不残幽灵(ACM3)。
- **升档补闪**:L2→L1 / L*→L0 置 force_send,下一拍立即下发完整紧凑姿态;
  首次进入视野者先推 `AgentMeta`(同现网 EntityShapes 先于状态包规则)。
- **客户端换档混合 ≤150ms**:各连杆局部姿态 lerp/slerp 混合,静姿↔权威姿态
  不闪跳(ACM7,截图架构师亲视)。

**Lite 视觉退化**:客户端加载 body.toml 时对 joints default_angle 做静态前向
运动学(纯函数,client-core bodies.rs 缓存各连杆相对根变换),L2 用根
(pos,yaw) 展开**全部 N 根连杆静姿**——保持 poses.len()==N,规避
client/entities.rs:282 的整棵 despawn+mesh 重建抖动。远距(≥32m,70°FOV/720p
下 0.4m 四足 ≈9 像素)静姿可接受;摆腿由客户端本地积分相位驱动,相位源 =
lite 的 motion_flags(低 7 位水平速率量化 0..6.35m/s、最高位 frozen/airborne),
**服务器不传假步态相位**(RL 策略无相位标量,仅开环脚本有,gait.rs:53)。
协议字段一次留齐,未来视觉增强不改协议。

**错峰**:每连接发送相位 phase_offset = player id,避免 N 连接同拍齐发全量尖峰。

## 5 线格式与带宽

**bincode varint 陷阱(核实)**:standard() 对整数 varint;有符号 i16 经 ZigZag
后 |v|≥126 占 3B(+50% 膨胀)。**一切量化定宽字段打包为 [u8;N]**(固定数组无
长度前缀,u8 逐字节;BodyHash([u8;8]) 先例)。无符号 varint:0..=250→1B、
251..=65535→3B、更大→5/9B;EntityId(服务器 seq)常见 1–3B;Tick(u64) 运行
>18.2 分钟后 5B。字节公式按最坏常见值(EntityId=3B、Tick=5B)估。

**追加 schema(枚举只追加;旧变体与 golden 字节一字不动)**:
- C2S=13 `SetAgentStream { lod: bool }`(§3 能力协商)。
- S2C=15 `AgentMeta { entries: Vec<(EntityId, BodyHash)> }`:首入视野前发送 +
  全量周期重发自愈;稳态姿态包省 8B/狗重复哈希。
- S2C=16 `AgentStatesCompact { tick, tier: u8, states: Vec<CompactAgentPose>,
  removed: Vec<EntityId>, full: bool }`;
  `CompactAgentPose { entity, root_pos: [f32;3], root_rot: [u8;6],
  root_linvel: [u8;6], child_links: Vec<[u8;12]> }`。
  root_rot = 最小三元 15bit 四元数(6B;最大角误差界由 M04a golden 实测钉死,
  不写未实测宣称值);root_linvel = 3×i16 打包(0.001m/s 步长,±32.767m/s);
  child 每连杆 [u8;12] = 根相对位置 3×u16(0.5mm 步长,±16.384m)+ 同式 6B 四元数。
  **子连杆线/角速度不传**(客户端渲染零消费,核实;根线速留供短外推)。
  13 连杆 quad = **172B/狗**(估算);1 连杆 capsule = 28B。
- S2C=17 `AgentStatesLite { tick, origin_chunk: [u8;8], states: Vec<LiteAgentPose>,
  removed: Vec<EntityId>, full: bool }`;origin_chunk = 基准 chunk (cx,cz) 2×i32 LE
  (包内显式,不依赖观察者中心);`LiteAgentPose { entity, rel_pos: [u8;6],
  yaw: [u8;2], motion_flags: u8 }` = **12B/狗**;rel_pos 3×u16 步长 1/32m
  (±1024m,覆盖 vd_max=32 即 512m 与全高);yaw u16 全圆。
- 预留 S2C=18 `ChunkDataCompressed { pos, lz4: Vec<u8> }`(lz4_flex 0.11.6 已在
  Cargo.lock,零新依赖;M05 实测突发后决定是否立项 M05b)。
- 帧开销(估算):Compact 14B、Lite 21B、Meta 6B(含 4B 长度前缀;vec 长 >250
  时 +2B/字段,公式按最坏另计)。

**M04a 落地注记(2026-11-27 验收,`8b5c112`)**:schema/量化/FK 全部落地为纯函数
(sim-api `quant.rs` 8 对 pack/unpack + client-core `bodies.rs::static_link_transforms`),
零发送路径改动(ACM8 由既有 golden + e2e 回归锚住)。单狗样本整帧实测:
Meta 17B / Compact-quad 185B / Compact-capsule 40B / Lite 31B(tick=300、
entity seq=300 口径;多狗摊薄与含头带宽 M05 实测)。四元数最小三元 15bit:
10000 LCG 样本扫描断言 ≤2.5e-4 rad 通过,架构师 f64 独立参考扫描最大 ≈1.2e-4 rad;
取整语义钉死 f32::round(半值远离零)、先钳制后转型;linvel 对称钳 ±32767
(不出 −32768)。静姿 FK 与内核零角生成+电机 default_angle 语义对齐,
T9 零重力 600 拍 PD 落定后逐连杆比对 ≤2e-3 m / 2e-3 rad 通过。
变异 B8(旋转复合顺序交换)存活:现有测试链均单轴/零角可交换,
多轴非零链缺口如实记录(B8b 位置复合丢父旋转已补杀 T7+T8);
unpack 末归一化为防御性步骤,去之存活(无独立锚),如实记录。

**M04b-1 落地注记(2026-11-28 验收,`6db0333`)**:服务器 LOD 状态机落地为
app 纯函数模块 `lod.rs`(LodTable::step + 周期判定 + pack_compact/pack_lite,
零网络零内核依赖),driver 按连接接线(emit_lod_streams;emit_outputs 跳过
lod 连接;AgentStream 切换三帧空 full 清表),net.rs P2 扩七槽
(t0/t1/lite 分槽最新覆盖)。常量:K_L1=8、驻留 30 拍(仅 L1↔L2、换档后起算)、
L1 10Hz/L2 2Hz/full 与 Meta 自愈 60 拍、死区 32/40、互换余量 4m、
L2 退出余量 8m;相位=player.0 错峰。R1 裁决两处任务书自身错误:
T2 触发距离 35→45(35 落死区,手算笔误);force_send 条件改为任何进入 L1
(含解除附身 0→1)均补闪,upgrades 语义保持排除 0→1(降档)。
变异 C1–C15 全杀(C14 槽身份由 T8/T11 锚住,T9 不锚)、S1 Meta 自愈去除杀、
S2 pending_removed 档内排序去除预期存活(无独立锚)如实记录。
观察项(非阻断):① pending_removed[0] 只积不消费(L0 removed 实际经
emit_l0_removed_if_lod P1 下发,表内残留为死数据,后续单元可清);
② 既有帧接收时序敏感测试在全负载并发下各偶发一次
(x3_double_run_bitwise 采样拍数 1906 vs 1916;
entity_states_incremental_then_full 全量帧收到空表),单跑/全套/复跑均绿,
非本单元引入(两测试连接均未启用 lod,发送路径零改动),M05 压测期可加固。

**带宽场景表(估算,32 只可见 13 连杆全移动狗/客户端;M05 实测校准)**:

| 方案 | L0 | L1 | L2 | 杂项 | 单客户端 | 4 人出口 | 10 人出口 | 16 人出口 |
|---|---|---|---|---|---|---|---|---|
| 无 LOD 基线 AgentStates@6Hz | — | 132.2 | — | 1.9 | **134.1 KB/s ✗** | 536 | 1340 | 2150 |
| 全紧凑:1 L0@60 + 8 L1@10 + 23 L2@2 | 11.2 | 13.9 | 0.6 | 2.1 | **27.8 KB/s ✓** | 111 | 278 | 445 |
| 极端聚堆 64 狗:1+8+55 | 11.2 | 13.9 | 1.4 | 2.1 | **28.6 KB/s ✓** | 114 | 286 | 458 |

(单位 KB/s;杂项 = 动态刚体/BlockUpdates/InputAck/Meta 对账 ≈2.1;含 TCP/IP
头约 +13%。)64KB/s 仍为**初始目标**(REQ-013 §3 先例),M05 含头实测后修订。
无 LOD 基线 134KB/s ≈ 2.1× 超标 → LOD+紧凑是达标必选项,非优化项。

**chunk 加入突发警告(估算)**:单 chunk 98,304 格(varint 小值 ≈98.3KB);
默认泵 16 块×10Hz → 首连 289 块突发 ≈15.7MB/s、10 人并发登入 ≈157MB/s。
不在稳态口径内,但单 TCP 流队头阻塞 L0 → P3 限速穿插(§3)+ M05 实测决定压缩。

## 6 客户端插值与表现

- `InterpBuffer`(client-core 纯逻辑、零墙钟单测):远端实体/多连杆主体按
  包内 tick 双帧 lerp/slerp,渲染时刻 = now − 缓冲时长(L1 估 150–250ms、
  L2 估 600ms,M05 定);缓冲欠载时 ≤50ms 钳制外推(仅根线速;子连杆不外推,
  防关节撕裂),超钳制冻结待包。
  **已落地**(M02b `ecb9654`,`client-core/src/interp.rs`,tick 域):
  双帧 lerp/slerp(linvel/angvel 取新帧);外推钳 `EXTRAP_CLAMP_TICKS=3.0`
  (=50ms@60Hz),仅根 translation,旋转/子连杆/速度冻结;向过去冻结不反推;
  每实体轨道 ≤`MAX_SNAPS=4`,迟到旧包忽略、同 tick 替换、full retain 对账。
  `RenderClock`:首帧一次性初始化 now=max(tick−buffer,0),后续 latest 只增
  now 不跳,speed=(1+err/16).clamp(0.75,1.25) 限速 slew;
  `DEFAULT_BUFFER_TICKS=12`(200ms)与 slew 常数 M05 实测再调。
- L0 与单机 interval=1 旁路零缓冲直写(现网零插值语义保留)。
  **已落地**(M02b):client 仅 `--connect` 网络模式启用 InterpRes;
  bypass = 单机 || 附身实体,`resolve_view` 直写最新态;结构对账/possess/
  脚本 WorldState 仍按服务器最新态,仅渲染 Transform 换源。
  ACM7「无 6/10Hz 抽搐」:M03 已钉双客户端语义 e2e(T1/T3);目视/实测留 M05。
- 换档混合 ≤150ms(§4);L2 静姿+程序化摆腿(§4);附身青绿高亮现网保留。
  **已落地**(M04b-2 `d28654e`):
  - `EntityMirror`(client-core/entities.rs)增 meta/agent_tiers/agent_speed
    与 defs/statics 缓存;Compact/Lite 解码→AgentView(子连杆世界系展开,
    速度零);Lite 按 statics 静姿展开 N 连杆,未知哈希仅根;分档 full 只清
    本档(跨档保留),removed 清四表;legacy full 亦清辅助表。缺 meta 跳过。
  - `present.rs`(纯逻辑零墙钟):tier2+已知 def 按 speed 积分摆腿相位
    (SWING_AMPLITUDE=0.35rad、STRIDE_LENGTH=0.8m,M05 目视校准),
    奇偶连杆反相;换档 150ms lerp/slerp 混合,完成逐位直写 target;
    相位跨档保留;首帧无混合;tier0/1 旁路逐位等价(单机/附身不变)。
  - `sync_agent_interp`(interp.rs):full 按 mirror 键集 retain,跨档轨道保留;
    `sync_camera_roots`:相机根表同语义对账。render 侧 tier2 额外延迟
    L2_EXTRA_DELAY_TICKS=24(总缓冲 36 拍=600ms,L1 12 拍=200ms)。
  - 能力门控:交互 `--connect` 且非脚本才发 SetAgentStream{lod:true};
    脚本/本地/ViewerClient::spawn 全走 legacy(ACM8 逐位不变,e2e T19 对照)。
  - R1-C 跨单元修复:M04b-1 pack_lite yaw 公式 `atan2(z,x)` 符号反
    (镜像朝向),改 `(-z).atan2(x)`,T6 golden [00,C0]→[00,40]。
  - 验收:23/26 变异击杀;D16/D22/D26 存活 = main.rs 交互接线(门控发送/
    插值喂入/相机+附身喂入)无无头锚,e2e 用 TestClient 不经二进制 drain,
    M05 脚本冒烟+目视兜底。R1/R2 共 7 处裁决均为任务书锚错误(eps/时钟推进/
    yaw 符号/相机接线/T22 预置/T23 混合感知/完成直写),生产语义经查证正确。
- 断连 HUD + 重连入口(M01);重连 = 新连接新身体,全量自愈(ACM3)。

## 7 10+ 扩展隔离与现网缺陷清单(核实证据见 §12)

现在隔离(M01/M02a/M03 内完成,不等 10 人):
- F1 **Sequenced 握手门**(net.rs:948 门先于 :963 unwrap):包装 Subscribe 必断连
  → 门作用于剥尽后内层;M01。
- F2 **accept_loop 句柄运行期无界增长**(net.rs:692 push、:701 仅关机 drain)
  → 周期 retain 已完成句柄;M01。
- F3 **迟到者旧地形 P0**:生产 put_chunks 仅卸载/despawn 批(driver.rs:2514),
  checkpoint=快照+命令日志;StreamChunks 读 provider.get(:2550)而非 session
  热块 → 活跃块编辑对迟到连接不可见。**热块优先读、provider 兜底**;M01,
  M03 用例钉死(ACM4)。
- F4 **handle_left panic**(driver.rs:1617,另 :1627):断线移除失败拖垮整服
  → 待清理队列下拍重试,绝不 panic;M03(ACM2)。
  **已落地**(M03 `f117835`):`pending_disconnect_bodies` 每连接至多一条,
  session.tick 后、net up 消费前重试;三分支:Ok 销账计数 /
  owner 无且非在途 → gone 销账 / 其余挂队列。观察者移除改 `if let Ok`。
  真实触发面 = Left 消费时 inbox 满(Route Err),T4 用 inbox_capacity=1 +
  同批 [SetBlock, Goodbye] 确定性复现;迁移窗口在途分支无 e2e 用例
  (A9 去 in_transit 检查存活,如实记录;迁移窗口断线用例留后续)。
- F5 **TCP_NODELAY 缺失** → 两端开启;M01。
- F6 **agent_root_state O(A·L) 克隆整片**(scheduler.rs:1447;drive_policies
  逐只调用)→ 单主体 O(log A) 查询(ownership 直定位分片);M02a。
  **已落地**(M02a-2 `8e58db9`):内核新增 `ShardSim::agent_root_state`
  (agents/frozen/links/bodies 四次 BTreeMap get),scheduler 经
  entity_ownership 直定位分片;结构性 O(log E+log S+log A+log L),
  无运行时复杂度锚(等价回退变异会存活,如实记录)。
- F7 **每连接重复编码/克隆**(driver.rs:2221,2235 双克隆)→ 每拍预量化一次
  共享只读表(Arc<TickSnapshot>),分连接拼帧;M02a。
  **已落地**(M02a-2 `8e58db9`):TickSnapshot 每输出轮构建一次,
  `PlayerStream.last_agents` Arc 化 + **内容相等**差分(每轮新 Arc,
  ptr_eq 不可用);仅对实发条目深克隆(全量=全部可见,增量=仅变化);
  线字节逐位不变;预量化推迟到 M04a 新包型。
- F8 **共享 up_tx 256** → 每连接隔离;M02a。
- F9 **policy 单工作线程 + 2ms 全拍预算**(config.rs:341;实测单狗 0.10–0.27ms,
  REQ-013 §11):32 狗同偶拍到期 3.36ms > 2ms → attached_at 奇偶错峰(峰值减半)
  + PolicyExecutor 边界抽象;M02a;扩池拐点 M05 实测(M05b)。
  **已落地**(M02a-2 `8e58db9`+`1722640`):`ControllerSlot.phase` 仅在
  Policy 挂载时按全局计数器 0/1 轮转(Script/None 恒 0 且不动计数器);
  due = `(now − attached_at + phase) % period == 0`;脚本公式不读 phase;
  period 1 与单策略首挂(phase 0)行为不变。`#[serde(skip)]` 存档兼容 v16,
  **观察项**:checkpoint 恢复后 phase/计数器归零,错峰失效直到重新挂载
  ——留 M05 评估是否需要持久化。
- F10 **chunk 流空转与无界 drain**(静止每 100ms 重建 289 节点方形;
  driver.rs:2531 同步逐块 provider.get)→ 静止短路 + 全局每拍 chunk 配额;
  M02a 短路、M05 配额与拆线程。
- F11 **单线程 runtime**(app/main.rs:31)与 sim/IO 争核 → 网络独立 runtime
  或驱动专属线程;M05(压测前置,否则测到的是互锁噪音)。
- F12 **redb 单 FIFO 读写互堵**(storage/worker.rs:158–213;checkpoint fsync
  堵并发 Get)→ MVCC 并发读池留 M05 实测尾延迟后决定。
- 每连接 ClientSession 独立结构(§3),未来 shard/网关拆分整体搬移。

## 8 判据(草案;标 ★ 者 M05 实测后修订)

- ACM1 双客户端编辑互见 ≤2 tick + RTT(M03)。
- ACM2 附身互斥/一连接一身体/断线强制解除;带负荷断线清扫不 panic(M03,F4)。
- ACM3 重连与丢包自愈:跨档无幽灵、无闪没;full 分档对账周期内收敛(M04b)。
- ACM4 迟到加入者看到活跃块编辑(F3;M01/M03)。
- ACM5 ★ 稳态带宽 ≤64KB/s/客户端 @32 可见分级流(含 TCP/IP 头实测;初始目标)。
- ACM6 ★ 服务器 tick P95 ≤8ms @4 客户端;10/16 合成负载曲线如实报告
  (16 人目标 P95 ≤12ms 为估算,实测定);policy_timeouts=0 @v1 规模。
- ACM7 L0 端到端 P95 ≤ RTT+2 tick(局域网);自身 60Hz 平滑;L1 插值无 6/10Hz
  抽搐;换档混合 ≤150ms 不闪跳(截图架构师亲视);L2 静姿+摆腿远距可接受。
- ACM8 未发 SetAgentStream 的连接(旧 viewer/既有自动化)**稳态**(无出站
  积压)帧序列逐位不变(golden;M04a)。积压时 P2 最新覆盖丢旧不丢新,对
  全部连接生效(队列是连接级基础设施);M02a-1 起 REQ-013 C05 T-A 的
  整段线字节逐位锚由「末帧 EntityStates 等价 + 字节数/帧数单调」取代。
- ACM9 ★ 100ms RTT + ±15ms 抖动 + 1% 丢包注入:状态收敛无幽灵;手感=RTT
  记录在案(M05)。
- ACM10 ★ chunk 首连突发期间 L0 专流停顿 ≤50ms(P3 限速;M02a 结构、M05 实测)。

## 9 单元划分

- **M01 网络传输与握手加固**:NetTransport(TCP)+ --connect + 断连 HUD;
  F1/F2/F5;F3 热块优先读。验收:ACM4 结构 + 握手/断连单测 + 旧流不变。
  **已验收**(2026-11-27,`11229ab`;四包 552 绿 + e2e 5 绿;变异 MA–ME 击杀、
  MF 拆批存活 = 既有套件无 TCP 批边界锚,缺口已由 M03 T7/T8 闭环;
  T5 Welcome 锚经 R1 裁决纠正为两阶段恰 2 条,架构师原锚错误认账)。
- **M02a 服务端专流与队列隔离**:ClientSession/Arc<TickSnapshot>;Outbound
  P0–P3 + P3 限速;L0 60Hz 专流(过渡用现网 AgentStates 单实体);F6/F7/F8;
  F9 奇偶错峰;F10 静止短路。验收:ACM10 结构 + 队列单测 + 单主体查询复杂度。
  **拆分**(2026-11-27):M02a-1 网络层(四级队列/P3 限速/L0 专流/F8/F10)
  与 M02a-2 查询/策略层(F6 单主体 O(log A)/F7 Arc<TickSnapshot>/F9 错峰)。
  **M02a-1 已验收**(2026-11-27,`4fe3a7b`;五包 627 绿 + e2e 13 绿;
  架构师变异 MA–MI 9/9 击杀;T8 双空轮数锚经 R1 纠正 20/20/21,架构师原锚
  错误认账;既有测试收帧手法经 R1/R2 授权改写 c05_ta 锚 2 / removed 顺序无关 /
  compound_shapes full 感知增量窗 6 拍,语义锚不变,生产代码零改动)。
  **M02a-2 已验收**(2026-11-27,`8e58db9`+R1 `1722640`;五包 905 绿 +
  e2e d01b 12 绿;架构师变异 N1–N8 8/8 击杀;N7 原 T5 存活归因 = 脚本
  actions_for_tick 不读 slot.phase,R1 补 T5b「脚本挂载不扰后续策略相位」
  锚后击杀;既有 W1–W5/U4/H2/H7/d01b 零改动,单策略首挂 phase 0 保证)。
- **M02b 客户端快照插值**(与 M02a 可并行):InterpBuffer 纯逻辑零墙钟单测;
  L0/单机旁路;≤50ms 钳制外推。验收:ACM7 插值部分。
  **已验收**(2026-11-27,`ecb9654`;client-core+client 292 绿 + e2e
  viewer_client 5 绿;架构师变异 N1–N8 8/8 击杀;既有测试零改动,
  单机旁路逐位等价为硬锚;ACM7 目视/实测部分留 M03/M05)。
- **M03 双客户端权威 e2e**:编辑互见/附身互斥/迟到者活跃块编辑/带负荷断线
  不 panic(F4)。验收:ACM1/2/4。
  **已验收**(2026-11-27,`f117835`;app 229 / client-core 264 / e2e 全绿,
  m03 七测:T1 编辑互见 ≤2 tick+RTT、T2 迟到者热块、T3 硬断线带负荷、
  T4 inbox 满重试、T5 重试/gone、T7 单 write 批锚、T8 cut_frame 排空;
  架构师变异 A1–A8 8/8 击杀(A4 兼杀既有 acn3),A9 去 in_transit 检查
  存活 = 无迁移窗口断线用例,缺口如实记录;M01 MF 拆批缺口就此闭环
  (T7 WriteRecorder 钉死单次 write_all + T8 多帧排空);
  实施方全量门禁遇 D 盘满按纪律停下回报,架构师清 16G incremental 缓存
  后复跑绿并代为提交)。
- **M04a schema 与量化数学**:C2S=13、S2C=15/16/17 追加 + [u8;N] 编解码往返
  golden(判别值钉死)+ 四元数/位置量化误差界实测钉死 + 静姿 FK 纯单测。
  验收:ACM8 + 误差界测试。
  **已验收**(2026-11-27,`8b5c112`;C2S=13 已于 M02a-1 落地不重复追加;
  sim-api 75 / client-core 266 / e2e 全绿,T1–T9 全过;架构师全量门禁
  workspace 测试/clippy/fmt 绿(首跑遇 D 盘满,清 incremental 缓存后
  CARGO_INCREMENTAL=0 复跑绿);变异 B1–B7、B9–B12 共 11 杀(B10 兼杀既有
  t3_c01b3),B8 旋转顺序交换存活(测试链单轴/零角可交换,缺口如实记录,
  B8b 丢父旋转补杀),S1 去 unpack 末归一化预期存活;e2e→client-core
  内部 dev-dep 边任务书授权,Cargo.lock 无新第三方包)。
- **M04b LOD 状态机与客户端表现**(拆两期,仿 M02a 先例):
  - **M04b-1 服务器 LOD 状态机(已验收 `6db0333`)**:三重防抖/K_L1/错峰/
    分档自愈/换档补闪;L0/L1 迁 Compact;net 七槽;ACM8 非 lod 连接逐位不变。
    验收:15/15 变异击杀(C14 经 T8/T11)+ S1 杀 + S2 预期存活;
    全量门禁 1901/0/117;R1 裁决 T2 距离与 force_send 0→1 两处任务书错误。
  - **M04b-2 客户端表现(已验收 `d28654e`)**:mirror 分档标签/分档 full 清表、
    Compact/Lite 解码→AgentView、150ms 换档混合(完成逐位直写)、L2 静姿 FK+
    程序化摆腿(phase 自 motion_flags 速率积分,跨档保留)、InterpRes/相机/
    附身接线;能力门控(脚本/本地/ViewerClient 走 legacy,ACM8)。
    验收:23/26 变异击杀,D16/D22/D26 存活(main.rs 交互接线无无头锚,
    M05 冒烟+目视兜底);R1/R2 共 7 处任务书锚错误裁决(含 R1-C 修 M04b-1
    yaw 符号);落地注记见 §6。ACM7 目视/ACM3 跨档幽灵留 M05。
- **M05 隔离与实测校准**:F11 拆线程 + F10 chunk 配额;分散 10/16 与聚堆
  10/16+32 狗两套压测;§10 矩阵;修订 §5/§8 标 ★ 判据;决定 M05b。
- **M05b(条件)**:ChunkDataCompressed(S2C=18,lz4 已在 lock)+ PolicyWorker
  扩池/批量;仅 M05 实测证明需要才立。
- **M06(可选)预测+校正**:仅限单连杆 capsule 身体或根位移近似
  (Correction 现 schema 单连杆,13 连杆 RL 主体永久走 L0 权威专流);
  启用 InputAck/Correction 预留位。WAN 需求出现再立。

顺序:M01 → M02a/M02b(并行)→ M03 → M04a → M04b-1 → M04b-2 → M05 →(M05b)→(M06)。
M03 在 M04 前 = LOD 的语义回归锚。每单元一任务书一回报,纪律同 agents.md。

## 10 M05 实测校准清单(禁止拍脑袋定硬门禁)

1. 线格式:应用层 vs 实发字节比(逐帧 write_all vs 合写);chunk 原始 vs lz4
   (Superflat/SeededScatter);1/4/10/16 客户端稳态+首连突发 → 定 M05b 与红线。
2. 增量抑制:32 狗全静止/半运动/全运动下 Compact/Lite 每秒字节 P50/P95/P99
   与静默跳过率 → 定稳态均值与 P95 门禁。
3. 插值体感:L1 6 vs 10Hz × RTT 0/30/80ms × ±15ms 抖动 × 1% 丢包:缓冲欠载率、
   L0 端到端 P95、换档混合截图 → 定 L1 频率与缓冲时长。
4. 扩展曲线:逐拍拆解(sim/policy/orchestrate+redb/emit/网络编码)×
   N=1,2,4,10,16 的 P50/P95/P99;policy_timeouts>0 拐点;分散与聚堆两套
   → 定单服上限、K_L1 终值、扩池阈值、redb 读池。

## 11 风险与被否替代方案

残余风险(不推翻已拍板取舍):① 无预测 v1 手感=RTT+16.7ms,公网 RTT>80ms
不保证(v1 仅局域网/低 RTT 判据);② TCP 内核重传 HOL 应用层队列消不掉
(QUIC 升级位核心依据);③ K_L1 硬帽下聚堆第 9 只近距降静姿(150ms 混合+
距离排序缓解,观感风险记录在案)。

被否方案:客户端请求档位(带宽放大/CPU DoS 面、升档多 1 RTT、转身突现);
纯外推 Dead Reckoning(13 连杆 50ms 即关节撕裂、碰撞穿模回弹);给旧枚举加
字段或 lite 塞 EntityStates(破只追加/golden、破 A01 不变量);关节角+客户端
FK 重建紧凑包(FK 求解器复杂度、rapier 柔度视觉偏差、未知哈希无兜底;172B
已仅占预算 43%);每客户端双 TCP 连接(握手/断连状态机翻倍、跨连接竞态;
四级队列零协议变动且可映射 QUIC)。

## 12 裁决与证据索引

合稿裁决全文:架构师工作稿(不入库),归档于 #28 评论(17 条 gemini 论断逐条核实 +
架构师认账 6 处 + 保留裁决 5 处)。关键代码证据(当前工作区行号):
proto.rs:247(varint standard);client-core/entities.rs:56(full 清全表);
client/entities.rs:282(poses.len 重建);driver.rs:863–866(附身表)、
:1294(附身移观察者)、:1617(断线 panic)、:2135/2219(欧氏半径)、
:2514(卸载才落库)、:2550(provider.get);net.rs:305/307/371(队列)、
:692/701(句柄)、:948/963(握手门)、:1221(共享 up_tx);
scheduler.rs:1447(整片克隆);net/chunk_stream.rs:36(方形);
controller/gait.rs:53(仅脚本相位);client-core/possess/mod.rs:26(冲刺 5.6);
voxel/chunk.rs:7–10(98,304 格);app/main.rs:31(单线程);
config.rs:331/334/341(checkpoint/worker/policy 预算);Cargo.lock(bincode
2.0.1、lz4_flex 0.11.6)。带宽/字节数均为估算,M05 实测校准。