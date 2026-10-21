# 任务 00:AC-T1 基线录制

- 状态:待放行(修订 2:按审核 B4 落实录制可靠性)。
- 对应 AC:**AC-T1 基线半边**(01 完成四项比对、02 再比一次)。
- 涉及 §9 白名单:**不涉及**(不改任何既有代码)。
- 涉及 §14 检查项:不涉及(本任务纯测试资产)。

## 目标与范围

在**未改动的当前 main**(`8702670` 与本地一致,REQ-007 尚未实施)上,录制观察者驱动
激活的现状行为,作为后续所有任务的回归标尺(AC-T1):

- 逐拍记录:**chunk 装载集**(①② 合一,现状合一)、**实体冻结/唤醒集**、
  **已注册分片集**(D16);
- 场景:观察者**移动、折返、远离、一次 ≥ 200 m 瞬间跳跃、移除**;常驻体(actor=true)
  在任何冻结事件前都已静止(AC-T1 前提);
- golden 文件入库;01 重跑同一场景做**四项逐拍严格相等比对**,02 再比一次。

**硬约束:不得改动任何 `crates/*/src`**;只新增 e2e 测试与 golden 文件。

## 交付物

1. `crates/e2e/tests/activation_baseline.rs`:场景驱动 + golden 读写与比对;
2. `crates/e2e/tests/golden/ac_t1_baseline.txt`:纯文本,每拍一行(B4.3)。

## 授权改动

- `crates/e2e/tests/activation_baseline.rs`(新);
- `crates/e2e/tests/golden/ac_t1_baseline.txt`(新,生成物入库)。

## 录制可靠性(B4,逐条落实)

1. **ensure 必须复刻**:驱动每拍末注册观察者 `ensure_radius`(= `awake_unload_radius`,
   32 m)内的分片,以及在途信封的目的分片。`driver::shards_within` 是私有函数 →
   在测试内复刻(分片核心网格上按到观察者核心距离 ≤ 32 m 判定,与 `orchestrate`
   逐行对齐,注明出处 `driver.rs` 对应行)。跳跃 200 m 后落进的分片因此被注册,
   基线才有内容。chunk 数据用**生产同款生成器**(`app::worldgen` 经 `new_registry`
   的同一来源)与**固定种子**(默认 `WorldSeed(0)`)。
2. **重录只经环境变量**:`REQ007_RECORD_BASELINE=1` 时录制并覆写 golden;默认运行
   只做比对。golden 入库后,**未经总架构师批准不得重录**。测试名
   `record_baseline_golden`(原 `record_baselinegolden` 有笔误)。
3. **golden 只存离散数据 + 纯文本**(e2e 无 serde_json,不改 `Cargo.toml`):每拍一行,
   ```
   <tick>|load=<ChunkPos 按 (x,z) 升序逗号列表>|frozen=<EntityId 升序>|woke=<EntityId 升序>|shards=<ShardId 升序>
   ```
   ChunkPos = `x,z`;EntityId = `world:seq`;ShardId = `wx:dim:sx:sz`(实现按
   `Display`/简单格式定,写入测试内注释)。空列表留空字段。
4. **CI 跨平台一致**:录制在 Windows、CI 在 ubuntu-latest——首次推送必须让 CI 通过
   (浮点只用 `total_cmp` 排序、无格式化输出差异;若 CI 失败,回评报告并停等)。

## 接口形状(观测面,签名示意)

用 `shard::Scheduler` 公开 API 在 e2e 内复刻驱动的激活职责(审核已同意,附 B4 条件):

```rust
// 每拍循环(复刻 driver orchestrate 的激活职责;每步注释对应 driver.rs 出处):
sched.tick();                                        // 分片 step
let reqs = sched.take_activation_requests();         // 激活决策(外露)
for (shard, d) in reqs {
    for cp in &d.chunks_to_load  { /* 生成器造 chunk → apply_chunk_load */ }
    for cp in &d.chunks_to_unload { /* apply_chunk_unload,丢弃返回值 */ }
}
// ensure 复刻:观察者 32 m 邻片注册(未注册则 ground_sim 等价地形分片 + 手动装载?)
// ——按现状 driver::orchestrate 的 ensure 路径:register(空/地形分片),不手动装 chunk。
// despawn 复刻:take_despawn_requests → unload_shard(记录分片集变化)。
// 记录:loaded 集 = 逐片扫描(is_chunk_loaded);frozen/woke = 激活决策;
//       shards = sched.shard_ids()。
```

场景脚本(拍数与观察者坐标序列写死;确定性由 AC1 位级纪律保证):

| 阶段 | 拍 | 动作 |
| --- | --- | --- |
| 布景 | 0 | 地形分片 (0,0) 注册;2 个常驻体(actor=true)在观察者 24 m 内落定(先推进 ≥ 21 拍确保静止);观察者 (8,8) |
| 移动 | ~60–120 | 水平移动 2 次(跨 chunk 边界,触发装载) |
| 折返 | ~120–240 | 快速回到出发点(§3.3-3 竞态场景的基线行为) |
| 远离 | ~240–360 | 移到 > 48 m 外(chunk 卸载窗口) |
| 跳跃 | 1 拍 | 传送到 ≥ 200 m 外(激活风暴路径,D16;ensure 复刻使目的分片注册) |
| 移除 | 末 | RemoveObserver → 空片 despawn 完成 |

## 语义细则

1. 基线录制约 500–700 拍,行数 = 拍数;
2. 场景中的常驻体**必须在冻结事件前静止**:录制测试内先断言(连续 20 拍位移 < 阈值),
   否则 panic——AC-T1 的前提本身被测试锚定;
3. golden 比较 = 逐行全等;失败信息给出第一个分叉行与两侧差异;
4. 确定性:不用墙钟,推进用固定拍数;随机参数一律写死。

## 测试清单

| 测试名 | 断言 | AC |
| --- | --- | --- |
| `record_baseline_golden` | 仅 `REQ007_RECORD_BASELINE=1` 时执行录制并写文件;断言场景前提(常驻体先静止);抽样断言关键拍(装载发生、卸载发生、跳跃拍风暴、末尾空片 despawn)非退化;默认(无环境变量)直接跳过 | T1(录制) |
| `baseline_replay_matches_golden` | 同场景重跑(无环境变量也运行),逐拍与入库 golden 全等(自校验;01 起冻结为回归标尺,测试代码可随 API 机械适配,golden 文件不变) | T1(自洽) |

## 门禁包

`cargo test -p e2e --all-targets` + `cargo clippy -p e2e --all-targets -- -D warnings` +
`cargo fmt --all -- --check`(workspace 无 src 改动,全量门禁应保持绿)。

## 完成定义

- 两个测试绿(本地含一次真实录制);golden 入库;`git status` 确认无 `crates/*/src` 改动;
- 推送后确认 CI(ubuntu)通过——跨平台一致性是 B4.3 的验收条件;
- 回评附 golden 拍数、文件大小、关键拍抽样摘要、CI 结果。

## 风险

- 复刻循环与真 driver 的激活职责存在细节差异 → 01/02 的比对出现假分叉:按审核意见,
  分叉一律先按 bug 处理(对齐复刻循环),确属语义变化的列出拍号与原因;
- ensure 复刻偏差(尤其跳跃后的注册时序)→ 基线跳跃拍录到空集:录制测试的
  "跳跃拍非退化"抽样断言会当场拦截。
