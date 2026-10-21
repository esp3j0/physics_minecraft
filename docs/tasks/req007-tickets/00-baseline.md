# 任务 00:AC-T1 基线录制

- 状态:待放行。
- 对应 AC:**AC-T1 基线半边**(实施后重跑同场景对比,对比断言在 02 完成)。
- 涉及 §9 白名单:**不涉及**(不改任何既有代码)。
- 涉及 §14 检查项:不涉及(本任务纯测试资产)。

## 目标与范围

在**未改动的当前 main**(`8702670` 与本地一致,REQ-007 尚未实施)上,录制观察者驱动
激活的现状行为,作为后续所有任务的回归标尺(AC-T1):

- 逐拍记录:**chunk 装载集**(①② 合一,现状合一)、**实体冻结/唤醒集**、
  **已注册分片集**(D16);
- 场景:观察者**移动、折返、远离、一次 ≥ 200 m 瞬间跳跃、移除**;常驻体(actor=true)
  在任何冻结事件前都已静止(AC-T1 前提);
- golden 文件入库;01+ 重跑同一场景逐拍比对。

**硬约束:不得改动任何 `crates/*/src`**;只新增 e2e 测试与 golden 文件。

## 交付物

1. `crates/e2e/tests/activation_baseline.rs`:场景驱动 + golden 序列化/比较
   (`--baseline` 环境变量或 feature 重录,默认比对);
2. `crates/e2e/tests/golden/ac_t1_baseline.json`(或 bincode):逐拍三元组序列。

## 授权改动

- `crates/e2e/tests/activation_baseline.rs`(新);
- `crates/e2e/tests/golden/ac_t1_baseline.json`(新,生成物入库)。

## 接口形状(观测面,签名示意)

不改 src 的前提下,用 `shard::Scheduler` 公开 API 在 e2e 内复刻驱动的激活职责
(偏离说明见 README §偏离 2):

```rust
// 每拍循环(复刻 driver orchestrate 的激活职责):
sched.tick();                                        // 1.4 分片 step
let reqs = sched.take_activation_requests();         // 1.5 激活决策(外露)
for (shard, d) in reqs { /* apply_chunk_load/unload(真体素 chunk) */ }
// despawn:take_despawn_requests → unload_shard(记录分片集变化)
// 记录:loaded chunk 集 = 逐片扫描(公开 voxel_chunk/is_chunk_loaded);
//       冻结/唤醒集 = 激活决策的 entities_to_freeze/wake;
//       注册分片集 = sched.shard_ids()。
```

golden 条目:

```text
{ tick, loaded: [ChunkPos], frozen: [EntityId], woke: [EntityId], shards: [ShardId] }
```

场景脚本(拍数写死,观察者坐标序列写死;确定性由 AC1 位级纪律保证):

| 阶段 | 拍 | 动作 |
| --- | --- | --- |
| 布景 | 0 | 地形分片 (0,0) 注册;2 个常驻体(actor=true)在观察者 24 m 内落定(先推进 ≥ 21 拍确保静止);观察者 (8,8) |
| 移动 | ~60–120 | 水平移动 2 次(跨 chunk 边界,触发装载) |
| 折返 | ~120–240 | 快速回到出发点(§3.3-3 竞态场景的基线行为) |
| 远离 | ~240–360 | 移到 > 48 m 外(chunk 卸载窗口) |
| 跳跃 | 1 拍 | 传送到 ≥ 200 m 外(激活风暴路径,D16) |
| 移除 | 末 | RemoveObserver → 空片 despawn 完成 |

## 语义细则

1. 基线录制约 500–700 拍,golden 条目数 = 拍数;文件体积可接受(逐拍全量 chunk 集按
   排序数组存);
2. 场景中的常驻体**必须在冻结事件前静止**:录制测试内先断言这一点(静止 = 连续
   20 拍位移 < 阈值),否则 panic——AC-T1 的前提本身被测试锚定;
3. golden 比较 = 逐拍全等;测试失败信息给出第一个分叉拍与两侧差异;
4. 确定性:不用墙钟,推进用固定拍数;随机参数一律写死。

## 测试清单

| 测试名 | 断言 | AC |
| --- | --- | --- |
| `record_baselinegolden` | 驱动场景;断言场景前提(常驻体先静止);写 golden 文件;抽样断言关键拍(装载发生、卸载发生、跳跃拍风暴、末尾空片 despawn)非退化 | T1(录制) |
| `baseline_replay_matches_golden` | 同场景重跑,逐拍与入库 golden 全等(自校验;01+ 起本测试冻结为回归标尺) | T1(自洽) |

## 门禁包

`cargo test -p e2e --all-targets` + `cargo clippy -p e2e --all-targets -- -D warnings` +
`cargo fmt --all -- --check`(全仓门禁同标准,workspace 无改动应保持绿)。

## 完成定义

- 两个测试绿;golden 文件入库;`git status` 确认无 `crates/*/src` 改动;
- 回评附 golden 拍数、文件大小、关键拍抽样摘要。

## 风险

- 若复刻循环与真 driver 的激活职责存在细节差异(如 ensure 时序),基线与 02 后真驱动
  的对比会出现假分叉——02 的 AC-T1 断言允许列出"§4.6 推迟唤醒"拍号,其余分叉一律
  按 bug 处理;
- golden 体积过大时(> 2 MiB)改差分编码,任务书内说明。
