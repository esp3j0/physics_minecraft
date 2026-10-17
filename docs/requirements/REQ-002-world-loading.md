# REQ-002:世界数据源与分片生命周期闭环

- 状态:**已确认**(2026-10-17 用户拍板:§9 四项决议按推荐全部采纳,开工)
- 提出日期:2026-10-17
- 上游:[REQ-001](REQ-001-architecture.md)(已完成,AC1-AC8 落地;本 REQ 落实其遗留:tier①② chunk 数据源、despawn 票据自动化)
- 关联 demo:`demos/solidify-slopes`(决议 5 义务,实验中;结论回填 §3)
- 网络/每玩家 chunk 流:**不在本 REQ**(REQ-001 §9.9 已划归 REQ-003;本 REQ 的接口形状为 REQ-003 预留)

---

## 1. 项目定义

把 REQ-001 的"测试手动装载 chunk"升级为生产形态:**程序化世界 + 命令化观察者 +
按需装载 + 票据驱动的分片生命周期**。完成后的验收场景:给一个种子与一名观察者,
app 跑起来——观察者移动,分片按需创建、chunk 按激活半径/预算装载、物理运转;
观察者离开,空片 30s 后 despawn 并持久化;观察者回来,分片从持久层复活,数据无损。

一句话:**世界自己长出来、自己活下去、自己收起来,且全程可回放。**

## 2. 既定共识(继承,不再辩论)

1. REQ-001 全部 13 项决议沿用(本 REQ 若需改动其中任何一条,必须先修订其 §9);
2. 分片快照已含体素数据(`ShardSim::snapshot` 含 voxels)——**世界生成器不入快照、
   不入回放**,AC8 兼容性天然成立;
3. 确定性纪律不变:生成器纯函数、同种子同输出(位级),核 crate 无墙钟/随机/IO;
4. 模拟状态只因盖章输入而变(蓝图 §0):观察者位置的变更必须**命令化**(见 §4.2,
   这是回放确定性的硬要求,不是风格选择);
5. 单写存储 worker 是唯一的存储执行者(storage 轮已定),一切读写经它。

## 3. 已验证事实(不必复测)

- 激活策略机已产出 `chunks_to_load/unload` 请求,预算/滞回/冷却语义有测试
  (挂账清偿轮;调度器当前忽略该返回——本 REQ 接线);
- `ChunkStore`/`CheckpointStore` + 单写 worker + flush 屏障可用(storage 轮);
- 扩张区重叠装载无物理空窗(e2e `expansion_overlap_no_physics_gap`);
- lifecycle 状态机具备 despawn 全部原语(idle_expired/Draining/note_drained/stop);
- 【已回填 2026-10-17】斜坡/堆叠慢爬行实验(demos/solidify-slopes,两次运行除耗时行外
  逐字节一致):**P2 现参数在受驱慢爬行场景 100% 误凝固**——平地钉速
  0.02/0.05/0.08 m/s 三档连续低于阈值 600 tick(每 20 tick 触发一次),阶梯下行
  6/6 配置首触发全在 tick 20 且早于级缘翻落(实体永远爬不到下一格);滑行掉落物
  在滑行中被冻(距最终静止位 8.2mm)。**堆叠不误凝固**(各层触发时已坐实,
  残余位移 ≤0.13mm;下层真凝固后上层落定零穿模 −0.1mm)。角速度项有效保护滚动体
  (ω≈2v)。**睡眠兜底对受驱实体空转**(wake_up 驱动 600 tick 不睡眠),且 Python
  绑定侧睡眠阈值被静默忽略。**P2′**(加"最近 20 tick 位移 ≤3mm"门)挡掉平地/滚动
  100%、阶梯 5/6 误触发,真静止/落定触发零延迟;仅剩的 1/6(1:4@0.09,顶住级缘
  d20 低至 0.4mm)在速度+位移信号上与真静止不可区分——须政策层豁免(§9 决议 4)。

## 4. 架构

### 4.1 世界生成(voxel)

- `WorldGenerator` trait:`generate(seed: WorldSeed, pos: ChunkPos) -> Chunk`
  ——纯函数、确定性、无 IO、`Send`;
- v1 实现两个:**Superflat**(层表参数化:bedrock/stone/dirt/grass)+ 
  **SeededScatter**(种子哈希散布方块,测试与 RL 简单多样性用);
- `WorldSeed(u64)` 新类型(types);种子入 `app::Config`(单一来源);
- 噪声地形:是否本轮做 → §9 决议 1。

### 4.2 观察者命令化(messaging + shard)

问题:`set_observers` 是外部直调,不进命令日志 → **回放无法重生产装载/冻结**,
破坏 AC8。方案:

- `SimCommand` **追加**变体 `SetObserverPosition { position: Vec3 }`
  (枚举只许追加,决议 6 兼容);
- 观察者表 = `BTreeMap<InputSource, Vec3>`(**source 即玩家/agent ID**,
  多玩家天然支持;source 0 保留给调度器);
- 投递路径 = **调度器级专用收件箱**(实现细化,2026-10-17):不走按分片 submit
  ——观察者必然走进尚未注册的分片区域,per-shard 通道会触发"未注册分片"
  fail-stop;调度器维护自己的观察者收件箱,`tick()` 开头应用当拍到期的
  `SetObserverPosition`(分片前缀里永远不会出现该变体);
- `Session::move_observer(position) -> LogEntry` 为生产入口(盖章+入日志,
  LogEntry.dest 取 position 所在分片,信息性);`Scheduler::set_observers`
  保留为测试/装配直通(同一张表)。

### 4.3 ChunkProvider(storage)

- 存储 worker 协议**追加读操作**:`Get(ChunkKey) → oneshot<Option<Chunk>>`
  (单执行者串行,无并发读问题;阻塞 recv 仅发生在驱动 tick 末端,可接受);
- `ChunkProvider`(storage crate,依赖 voxel 生成器,方向合法):
  `get(key) = worker 读 → Some 用之;None → generator.generate(seed, pos)`;
- **数据源序**:持久层永远优先——玩家改过的 chunk 不会被生成器覆盖回初生值;
- 写路径:despawn 卸载的 chunk 经既有 `put_chunks` + flush 屏障落库。

### 4.4 装载编排(shard + app)

- `Scheduler::take_activation_requests() -> Vec<(ShardId, ActivationDecisions)>`
  (两拍之间取,每拍刷新;含 chunk 装卸请求与冻结已落实的决策);
- `Scheduler::apply_chunk_load(shard, cp, chunk)` / 分片缺 chunk 时的行为:
  装载由驱动异步回填(取数在 tick 末端,装载在下一拍生效——晚一拍装载,
  与信封同语义,doc 注明);
- 驱动 tick 末端(app):取请求 → ChunkProvider 取数 → 回填 → despawn 候选处理。

### 4.5 分片生命周期票据(shard + app)

- **票 = 观察者表非空**(观察者所在分片及其激活半径覆盖的分片均视为有票);
- 调度器每拍判定:空片(entity_count == 0)+ 无票 + `idle_expired` →
  **despawn 候选**(不自动拆);`take_despawn_requests() -> Vec<ShardId>`;
- 驱动处理候选(app):`Scheduler::unload_shard(shard) -> Vec<(ChunkPos, Chunk)>`
  (内部走 Draining→note_drained→Stopped→release,lifecycle 全程合法)→
  chunk 批量 `put_chunks` → **flush + recv ack(屏障)** → 完成;
- 再进入:观察者移入已 despawn 区域 → 分片重建(chunk 从 ChunkProvider 恢复,
  持久层优先)——"复活"与"初生"对模拟不可区分(除持久层已有修改)。

## 5. 轮廓含糊点与决议(草案,待拍板)

见 §9。

## 6. ferrumc v2 对照

| 我们要造的 | v2 已有的 | 差异 |
| --- | --- | --- |
| chunk 按需装载(激活请求驱动) | chunk_stream.rs(玩家驱动 16+16 限制) | 我们以激活半径+预算为准,v2 以视距 |
| despawn 票据 | chunk tickets + 持久化屏障 | 同构:票=观察者覆盖;屏障复用 |
| 世界生成 | 外部世界数据 | 我们内置纯函数生成器(RL 可复现要求) |

## 7. Demo

- `demos/solidify-slopes`(决议 5 义务,进行中):斜坡/堆叠慢爬行的 P2 误凝固
  验证——**无其它数值未知项**:装载/生命周期是系统逻辑,Rust TDD 直接覆盖;
  生成确定性是代数性质(同输入同输出)非实验性质。

## 8. 模块归属(蓝图 §7 增量)

| 新东西 | 去处 |
| --- | --- |
| WorldSeed、WorldGenerator、Superflat、SeededScatter | types(Seed)/voxel(生成器) |
| SimCommand::SetObserverPosition 追加 | messaging |
| 观察者表、前缀截获、despawn 候选、unload_shard、装载编排 API | shard |
| worker Get 操作、ChunkProvider | storage |
| move_observer、种子配置、tick 末端编排(取数/回填/despawn 屏障) | app(sim-api 透传 move_observer) |
| 验收套件 | e2e |

## 9. 决议(2026-10-17 用户拍板:按推荐全部采纳)

1. **地形范围**:v1 = Superflat + SeededScatter,**不做噪声地形**(推荐:
   物理玩法(拆/炸/塌/凝固)在 superflat 上已完整;噪声地形牵扯高度图/生物群系
   设计,应单独立 REQ)。备选:本轮加一个简单高度噪声(2D value noise);
2. **观察者命令化**:采纳 §4.2 方案(推荐;回放确定性的硬要求)。
   `set_observers` 保留为测试直通;
3. **despawn 自动化**:默认开启(30s/1800 拍,配置化;推荐)——despawn 候选
   机制让驱动始终握有最终决定权,安全;
4. **凝固桥参数修订**(demo 数据已回填 §3;按 REQ-001 §9 修订流程改其决议 5/§4.3,
   待拍板):
   a) 被动体触发谓词加**位移门**:P2′ = lin<0.1 ∧ ang<0.05 ∧ **最近 20 tick 位移
      ≤3mm**,连续 20 tick(实测:合法落定 d20 ≤1.4mm,受驱爬行 ≥6.3mm,两侧
      各 ≥2× 裕度);
   b) **政策层豁免**(必要):有活动移动命令的实体(演员集)不适用自动凝固,
      只由显式命令/命令结束触发——顶住级缘的受驱实体与真静止在信号上不可区分
      (d20 低至 0.4mm),纯信号判据必漏;与共识 6"物理核不自主决策"自洽
      (演员集即 §4.2 的观察者/命令来源,天然可判);
   c) REQ-001 §4.3 阈值单位澄清为 **m/s、rad/s**(0.1 m/s ≈ 每 tick 1.67mm),
      防止 m/tick 误读;
   d) 睡眠兜底不得视为误凝固保险(对受驱实体空转,场景 H 实测)。

## 10. 验收标准(全部可测)

- **AC-G1 生成确定性**:同 `(seed, pos)` 跨实例两次生成位级相等;
- **AC-G2 数据源序**:chunk 被修改并持久化后,重装载取回**修改值**而非初生值;
- **AC-G3 装载闭环**:`move_observer` 命令移动观察者 → 激活请求 → 真数据装载
  → 物理可见(实体落在生成的地面上并被支撑);
- **AC-G4 生命周期闭环**:空片无票 1800 拍 → despawn 候选 → 卸载/持久化/屏障 →
  观察者返回 → 分片复活,chunk 数据等价(含修改过的 chunk);
- **AC-G5 回放兼容**:含观察者移动/装载/卸载/despawn-复活的场景,600 拍中途
  快照重放,终态位级一致(观察者命令入命令日志,装载与冻结确定性再生);
- **AC-G6 持久完整**:despawn 后"进程重启"(重开 redb 库),chunk 数据完整可读。
