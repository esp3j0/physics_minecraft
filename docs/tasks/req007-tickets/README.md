# REQ-007 实现任务书(总览)

- 状态:**评审中**(2026-10-21 撰写)。
- 需求:[REQ-007:激活票据、票据转递与瞬态体落地保证](../../requirements/REQ-007-activation-tickets.md)
  (**修订 3,提交 `bfdd16f`**,含 D1–D16 决议;唯一事实来源)。
- 上游 issue:[#14](https://github.com/esp3j0/physics_minecraft/issues/14)(含《总架构师裁定》)。
- 实现顺序:**00 → 01 → … → 07 串行**;每任务独立门禁、独立提交(Refs #14)、回评后停等放行。
- 格式参照:req006-entity 系列。与 req006 的两点流程差异:
  - **任务 00 是基线录制**:只新增 e2e 测试与 golden 文件,不碰任何 `crates/*/src`;
    01/02 的回归标尺就是重跑 00 场景断言与 golden 一致(AC-T1);
  - 任务多且相互咬合,**每份任务书的【授权改动】是硬边界**,越界即回归。

## 任务表

| # | 任务书 | 内容 | 门禁 crate | 对应 AC |
| --- | --- | --- | --- | --- |
| 0 | [00-baseline.md](00-baseline.md) | AC-T1 基线录制:调度器循环复刻驱动激活职责,golden 文件入库 | e2e(仅新增) | T1(基线半边) |
| 1 | [01-tier-split.md](01-tier-split.md) | ①② 拆分:独立半径/预算/冷却;② 分片本地派生;I1 + I2(简化足迹) | physics-sim app | T2(部分)、T1(①②等价) |
| 2 | [02-ticket-model.md](02-ticket-model.md) | 票据模型/票据表/票据收件箱;观察者票 + 区域票;plan 改读票据视图;ensure/保活/despawn 改读票据 | messaging types physics-sim shard sim-api app e2e | T1(全绿)、T18 |
| 3 | [03-transient-motion.md](03-transient-motion.md) | 瞬态运动票;当前/预载足迹;待地形;强制落地;虚空移除;支撑判定;凝固前提;承载(Pause/Suspend);阻塞请求(D15) | physics-sim shard app e2e | T3–T9、T20 |
| 4 | [04-actor-wake.md](04-actor-wake.md) | 常驻运动票;静止判据重置;统一唤醒 + 向上支撑闭包;地形变更唤醒;超预算常驻冻结 | physics-sim shard e2e | T10–T13 |
| 5 | [05-contact-wake.md](05-contact-wake.md) | 接触唤醒:撞前预测、冻结邻接图、动量传播、补偿路径 | physics-sim e2e | T14–T17 |
| 6 | [06-snapshot-config.md](06-snapshot-config.md) | 快照收口与 format 3→4;不变量 I3 + 测试钩子;幂等收口;§4.15 全参数配置;遥测 | 全 crate | T19、T21、T22 |
| 7 | [07-acceptance.md](07-acceptance.md) | AC-T1–T23 全量验收复核;§14 检查项逐条自查;人工冒烟步骤 | e2e | T1–T23 全量、T23 回归 |

## 公共完成定义(每个任务)

1. **严格 TDD**:先写测试清单 + 最小桩(编译通过、相关测试红),门禁包见红;再实现到绿。
   基线任务 00 例外(只录 golden,无"红转绿")。
2. **门禁**(退出码必须全 0):
   ```bash
   cargo test --workspace --all-targets
   cargo clippy --workspace --all-targets -- -D warnings
   cargo fmt --all -- --check
   ```
3. 只改任务书【授权改动】列出的文件;测试文件随授权清单。
4. 既有测试断言只能按需求 §9 白名单修改,**每处在提交说明中对应白名单条目**;
   白名单外的断言改动 = 回归,除非任务书【偏离与授权请求】节已列明并经总架构师放行。
5. 提交直接推 main;提交前 `git pull --rebase`;conventional 风格中文说明,末行 `Refs #14`;
   不 force push、不改写历史、不关闭 #14。
6. 回评按 issue #14 的【回评模板】,然后**停等放行**;被打回在新提交修复。

## 关键纪律(全任务适用)

- **承重墙(§4.2.2 / §14.1)**:`plan`、ensure、分片保活的实现中**不得出现按 `TicketKind`
  的分支**——决策层只消费 `RegionSpec / WakeSpec / priority / state`。02 起用扁平
  `TicketView` 传递,`TicketKind` 不进决策层作用域;评审时以此为检查口径。
- **阻塞请求(§4.12 / D15 / §14.8)**:由实体状态(待地形 / 推迟唤醒 / 待强制落地)推导,
  不依赖票种。
- **确定性**:核心 crate(types/voxel/messaging/physics-sim/shard)无墙钟、无随机、无 IO;
  新遍历一律按 `EntityId / ChunkPos / ShardId / TicketKey` 升序;rapier 类型不出 physics-sim。
- **只追加**:四个票据枚举、`SimCommand`、`C2S/S2C` 声明序只追加;旧编码保持可解。
- **票据操作全幂等(D14)**:实体票 Issue 已在 → 无操作(不重置时钟、不改状态);End/Pause/
  Suspend 等遇不存在票 → 无操作。正确性由 I3(06 落地)每拍校验兜底。
- **不变量**:I1(②⊆①)、I2(醒着 ⇒ 足迹 ② 齐)debug 断言 fail-stop(01 起);I3 每拍末
  校验(06 起)。
- **快照格式**:02 起新增字段使旧 checkpoint 自然 `UnexpectedEnd` 拒绝(bincode 非自描述,
  不可用 `#[serde(default)]` 声称兼容——REQ-005 D8 教训);代码注释如实标注
  "旧档不可用需清 data_dir",版本号统一在 06 升 4。
- **viewer 不动**:REQ-007 不改 viewer(人工冒烟只依赖既有 B/X 键)。
- **实操**:跑过 `app.exe`/`viewer.exe` 后再跑门禁会遇到文件锁(os error 5)——先杀进程,
  或用隔离 `CARGO_TARGET_DIR`(如 `target\req007-gate`)。

## 偏离需求示意之处(汇总;细节见各任务书)

以下偏离 / 具体化需总架构师在放行对应任务书时确认:

1. **② 的执行机制(01)**:② 不经 `plan` 决策外露,由 `ShardSim` 在 `apply_activation`
   开头按"① 已装 ∧ ② 未建 ∧ 距锚点 ≤ collider_load ∧ 冷却"本地派生构建(ChunkPos 序、
   预算截断);拆除与 ① 卸载同点(I1)。依据 §4.1"由内存中的 ① 纯函数派生,ShardSim
   自行构建/拆除"。
2. **00 基线的观测面(00)**:基线场景在 e2e 内以 `shard::Scheduler` 公开 API 复刻驱动的
   激活职责(ensure 注册、chunk 装卸执行、despawn),而非跑真 `app::driver`——否则无法在
   不改 src 的前提下逐拍读取激活决策。场景代码入库后冻结,01+ 重跑同一份。
3. **I2 的增量落地(01/03)**:01 先以"实体中心所在柱"为简化足迹断言 I2;03 引入完整
   当前足迹(AABB·√3 + terrain_guard)后 I2 口径升级;AC-T2 全量断言在 07 复核。
4. **快照字段的中间态(02–05)**:票据表、待地形速度、补偿种子等字段自 02 起随实现分批
   进入快照结构,版本号保持 3(旧档自然拒绝);06 统一升 4 并补版本拒绝测试。
5. **I2 阻塞装载排序之外的请求授权(01/02)**:`activation.rs` 既有 8 个测试的断言 /
   构造方式改动源于 ①② 拆分与 plan 改读票据视图,**不在 §9 六条白名单内**,需总架构师
   在放行任务书 01/02 时一并授权(改动明细已列于各任务书)。
6. **配置键名(06,§4.15 留给任务书)**:`data_load_radius` 等新键显式引入;旧键
   `load_radius` / `unload_radius` 保留为别名(读到时映射为 ①② 同值),配置测试锚定。
