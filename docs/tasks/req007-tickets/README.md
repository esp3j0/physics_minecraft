# REQ-007 实现任务书(总览)

- 状态:**修订 2,评审中**(2026-10-21 撰写;同日按总架构师审核意见 B1–B5/S1–S7 修订)。
- 需求:[REQ-007:激活票据、票据转递与瞬态体落地保证](../../requirements/REQ-007-activation-tickets.md)
  (**修订 4,提交 `1377883`**,含 D1–D17 决议;唯一事实来源)。
- 上游 issue:[#14](https://github.com/esp3j0/physics_minecraft/issues/14)(含《总架构师裁定》与任务书审核意见)。
- 实现顺序:**00 → 01 → … → 07 串行**;每任务独立门禁、独立提交(Refs #14)、回评后停等放行。
- 格式参照:req006-entity 系列。与 req006 的两点流程差异:
  - **任务 00 是基线录制**:只新增 e2e 测试与 golden 文件,不碰任何 `crates/*/src`;
    01/02 的回归标尺就是重跑 00 场景逐拍比对(B4.4:01 完成四项比对,02 再比一次);
  - 任务多且相互咬合,**每份任务书的【授权改动】是硬边界**,越界即回归。

## 任务表

| # | 任务书 | 内容 | 门禁 crate | 对应 AC |
| --- | --- | --- | --- | --- |
| 0 | [00-baseline.md](00-baseline.md) | AC-T1 基线录制:调度器循环复刻驱动激活职责(含 ensure),golden 文件入库(纯文本,环境变量重录) | e2e(仅新增) | T1(基线半边) |
| 1 | [01-tier-split.md](01-tier-split.md) | ①② 拆分:plan 是 ② 唯一决策者;I1;D17 开关分支;**format 3→4(本任务起)**;基线四项比对 | physics-sim app e2e | T2(部分)、T1、T24(部分) |
| 2 | [02-ticket-model.md](02-ticket-model.md) | 票据模型/票据表/票据收件箱;观察者票 + 区域票;plan 改读票据视图(order 排序键);ensure/保活/despawn 改读票据;开关推广 | messaging types physics-sim shard sim-api app e2e | T1(再比)、T18 |
| 3 | [03-transient-motion.md](03-transient-motion.md) | 瞬态运动票(调度器 claim 时签发);启用补签(D17);当前/预载足迹;待地形;I2;强制落地;虚空移除;支撑判定;凝固前提;承载;阻塞请求(D15) | physics-sim shard app e2e | T3–T9、T20、T24(补签) |
| 4 | [04-actor-wake.md](04-actor-wake.md) | 常驻运动票;静止判据重置;统一唤醒 + 向上支撑闭包(含常驻推迟唤醒);地形变更唤醒;超预算常驻冻结 | physics-sim shard e2e | T10–T13 |
| 5 | [05-contact-wake.md](05-contact-wake.md) | 接触唤醒:撞前预测、冻结邻接图、动量传播、补偿路径 | physics-sim e2e | T14–T17 |
| 6 | [06-snapshot-config.md](06-snapshot-config.md) | 快照收口终稿 + D17 开关入快照;I3 + 测试钩子;幂等收口;§4.15 全参数配置;遥测;v3/v2/v1/v0 拒绝测试 | 全 crate | T19、T21、T22、T24(开关快照) |
| 7 | [07-acceptance.md](07-acceptance.md) | AC-T1–T24 全量验收复核;§14 检查项逐条自查;人工冒烟步骤 | e2e | T1–T24 全量、T23 回归 |

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
   白名单外的断言改动 = 回归,除非任务书已列明且经总架构师批准(目前仅一条:
   01 的 `swap_budget_caps_combined_load_unload`,审核意见 B2 已批)。
   **§9 第 7 条(修订 4)**:激活未启用模式下运行的既有用例,一律不得改动断言。
5. 提交直接推 main;提交前 `git pull --rebase`;conventional 风格中文说明,末行 `Refs #14`;
   不 force push、不改写历史、不关闭 #14。
6. 回评按 issue #14 的【回评模板】,然后**停等放行**;被打回在新提交修复。
7. **凡改动 checkpoint 布局的任务(01 起),回评必须提醒:旧存档不可用,需清 data_dir**(B5)。

## 关键纪律(全任务适用)

- **D17 激活启用开关(§4.0)**:曾设置过观察者**或**区域票才启用;单向、随快照;
  **未启用时行为与 `8702670` 完全一致**(装载合一建①②、不派生票据、不校验 I2/I3、
  无待地形/强制落地/虚空移除/凝固前提修订/接触唤醒/地形变更唤醒)。01/02/03/06 各有分工,见任务表。
- **承重墙(§4.2.2 / §14.1,结构保证)**:`plan`、ensure、分片保活只消费
  `RegionSpec / WakeSpec / priority / state` 与**不透明排序键 `order: u32`**(S1);
  `TicketKind` 不进决策层作用域。Observer 与 Area 同规格同决策的对拍测试锚定。
- **② 的决策唯一出处(B3)**:半径、滞回、预算、冷却全部在 `plan`;输出
  `colliders_to_build / colliders_to_drop`;`ShardSim::apply_activation` 拿决策当场执行
  (与冻结/唤醒同方式);驱动层不执行 ②。
- **阻塞请求(§4.12 / D15 / §14.8)**:由实体状态(待地形 / 推迟唤醒 / 待强制落地)推导,
  不依赖票种。
- **确定性**:核心 crate(types/voxel/messaging/physics-sim/shard)无墙钟、无随机、无 IO;
  新遍历一律按 `EntityId / ChunkPos / ShardId / TicketKey` 升序;rapier 类型不出 physics-sim。
- **只追加**:四个票据枚举、`SimCommand`、`C2S/S2C` 声明序只追加;旧编码保持可解。
- **票据操作全幂等(D14)**:实体票 Issue 已在 → 无操作(不重置时钟、不改状态);End/Pause/
  Suspend 等遇不存在票 → 无操作。区域票同 id 覆盖 = 续期(TTL 从最近一次 Set 起算,S3)。
  正确性由 I3(06 落地)每拍校验兜底。
- **快照格式(B5)**:`CHECKPOINT_FORMAT_VERSION` **在 01 升 4**;01–05 期间布局继续变化
  时版本号不再逐任务升(都拒 v3 旧档),每任务回评提醒清 data_dir;06 收口终稿并补
  v3/v2/v1/v0 拒绝测试。bincode 非自描述,不允许任何"旧档兼容"声明(REQ-005 D8 教训)。
- **viewer 不动**:REQ-007 不改 viewer(人工冒烟只依赖既有 B/X 键)。
- **实操**:跑过 `app.exe`/`viewer.exe` 后再跑门禁会遇到文件锁(os error 5)——先杀进程,
  或用隔离 `CARGO_TARGET_DIR`(如 `target\req007-gate`)。

## 偏离与裁定记录(审核后;原 7 条偏离的最终口径)

| 原 # | 事项 | 最终口径(依据) |
| --- | --- | --- |
| 1 | ② 的决策归属 | **plan 是唯一决策者**(B3):决策(半径/滞回/预算/冷却)全在 plan,ShardSim 只执行;驱动不执行 ② |
| 2 | 00 用 Scheduler 公开 API 复刻驱动 | **同意,附 B4 条件**:复刻 ensure(32 m 邻片 + 在途信封目的分片);生产同款生成器固定种子;环境变量才重录;纯文本 golden;CI(ubuntu)首次推送必须通过 |
| 3 | I2 两步落地 | **改为 01 不做 I2**(S4):01 只做结构拆分 + I1 + 基线严格等价;I2/待地形/阻塞请求在 03,常驻推迟唤醒在 04 |
| 4 | 版本号保持 3 | **不同意(B5)**:01 升 4;06 收口 |
| 5 | activation.rs 8 测试授权 | **不改名(B2)**,授权请求消失;仅 `swap_budget_caps_combined_load_unload` 因预算拆分调整断言,**已批准** |
| 6 | 覆盖更新时钟 | **区域票覆盖 = 续期(时钟重置,S3)**;观察者票无上限,时钟无关紧要;实体票按 D14 不重置 |
| 7 | 旧配置键别名、新键优先 | **同意** |

其余各任务书偏离已获同意:03 的 `TicketOpRecord`、翻转冷却保留为滞回频率约束、
虚空移除放在 03;04 的几何闭包和 `BTreeSet` 去重;05 的扫掠只用线速度(质量取
rapier `mass()`,S7);06 的 I3 冻结信息来源、遥测打平进 TickSummary。
