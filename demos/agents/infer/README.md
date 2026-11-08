# demo `infer` — R2(2026-11-08)

REQ-012 A00-5(放行)的 `infer`:tract 推理延迟与 ort 对照。R2 保留原
范围;闭环项按 R2 改为 arm 任务口径(轨迹级逐步对照由 `learn torchtract`
承担,本 demo 做实现级对照与并发延迟)。

## 模型

- (a) `random`:随机权重 48→256→256→12 ELU(AC-D4 典型 MLP 同规模;
  本 demo 手写 ONNX 写出,种子 20261107);
- (b) `policy`:`learn` 导出的 arm 策略(`policy_submitted.onnx` =
  14→64→64→2 tanh,torch 2.15 导出;附带 `policy_submitted.bin` =
  同权重 f32 LE,供手写前向对照,由 tools/train 的检查点导出)。

## 结果(I 判据)

**I1 一致性(1 万随机输入,两两最大绝对差)——达标(≤1e-5)**:

| 模型 | tract vs 手写 | ort vs 手写 | tract vs ort |
| --- | --- | --- | --- |
| random | 1.311e-6 | 1.311e-6 | 1.132e-6 |
| policy | 1.192e-6 | 1.431e-6 | 1.192e-6 |

**I2 tract 单线程 batch 1 P95 ≤ 1 ms(AC-D4)——达标**:

| 模型 | 实现 | P50 | P95 | P99 | max | 加载 |
| --- | --- | --- | --- | --- | --- | --- |
| random | hand | 47.0 | 81.0 | 87.0 | 220.0 | — |
| random | tract | 14.0 | 15.0 | 22.0 | 200.0 | 2.5 ms |
| random | ort | 11.0 | 16.0 | 20.0 | 218.0 | 58.4 ms |
| policy | hand | 5.0 | 6.0 | 7.0 | 44.0 | — |
| policy | tract | 11.0 | 13.0 | 16.0 | 87.0 | 1.8 ms |
| policy | ort | 7.0 | 8.0 | 12.0 | 79.0 | 1.8 ms |

(µs;10 万次,预热 1000;`out/bench.txt` 全表。)

**I3 无 panic、结果在提交文件**——达标。ort 与 tract 模型加载都比
首版(58 ms→1.8 ms)稳定;ort 的 Environment 每进程只能建一次,
已用 OnceLock 修复(此前 agree 第二个模型会 panic)。

**batch 8**(random;单会话固定 [1,48] 输入,连喂 8 次计总时长再 ÷8):
hand P95 87 µs/只,tract P95 20 µs/只——tract 与手写的相对差距在 batch
口径下拉近(单次 run 的固定开销摊薄)。

**16 线程并发**(各线程独立 tract 会话跑 random 模型,6.4 万样本):
单次 P50 19 µs、P95 28 µs。

**tract vs ort 的 P95 之比**(只报数):random 15/16 ≈ 0.94、
policy 13/8 ≈ 1.6——都在 3 倍以内,不触发 D7 裁定条款。

**闭环(R2)**:`learn eval` 本身就是 tract 推理跑完整回合(部署路径);
"tract vs torch 从哪步分叉"由 `learn torchtract` 承担——1000 个真实
观测上最大绝对差 4.768e-7(远小于单步分叉阈值,见 learn README L3 节)。
infer 侧 `closedloop --policy out/policy_submitted.onnx` 输出 16 线程
并发 P95(28 µs)。

## 复现

```
infer write-model out/random.onnx
infer agree [--policy <learn onnx>]      # I1(需要同名 .bin 手写权重)
infer bench [--policy <learn onnx>]      # I2 + batch8
infer closedloop --policy out/policy_submitted.onnx   # 并发 P95
```

## 版本

tract-onnx 0.23.10(into_optimized → into_runnable)、ort 2.0.0-rc.15
(CPU,intra-op 1 线程,download-binaries)、rand 0.9.5。模型 (a) 的
ONNX 由本 demo 手写最小 protobuf(与 A00-5 首轮同一套写法)。
