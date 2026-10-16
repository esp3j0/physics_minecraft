# REQ-001 挂账清偿任务书(总览)

- 状态:已定稿(架构师签发)
- 日期:2026-10-16
- 上游:[REQ-001](../../requirements/REQ-001-architecture.md) §4.3/§9.4/§9.12/§10(AC2/AC3/AC4)
- 实现方式:与前几轮相同——glm-5.3-flash 写核心(02),qwen 写测试任务(00/01/03)
  + 独立评审;cargo 门禁;架构师终审。**完成即 REQ-001 全部 AC 落地。**

## 任务顺序(依赖序,串行执行)

| # | 任务书 | 内容 | 实现模型 |
| --- | --- | --- | --- |
| 0 | [00-ac2-migration.md](00-ac2-migration.md) | e2e crate 骨架 + AC2 迁移等价(接触态 vs 单世界,<1e-4m) | qwen |
| 1 | [01-ac3-explosion.md](01-ac3-explosion.md) | AC3 爆炸等价(位级)+ AC8 长程(600 拍) | qwen |
| 2 | [02-activation.md](02-activation.md) | AC4:激活策略机 + ShardSim 接线(冻结/唤醒/预算/滞回/冷却) | glm |
| 3 | [03-ac4-e2e.md](03-ac4-e2e.md) | AC4 端到端(经 Scheduler:观察者→冻结落实施→事件免疫) | qwen |

## 每份任务书的公共完成定义

1. 严格 TDD(策略/实现任务);纯测试任务(00/01/03 的清单即验收)照任务书场景写全;
2. 门禁(机器判定,不可绕过;各任务书【门禁包】列出):
   - `cargo test -p <门禁包…> --all-targets` 退出码 0;
   - `cargo clippy -p <门禁包…> --all-targets -- -D warnings` 退出码 0;
3. 只改任务书【交付物】与【授权改动】列出的文件。

## 架构师在本轮做出的具体决策(实现时不得偏离)

1. **根级等价套件落为新 workspace 成员 `crates/e2e`**(蓝图写"仓库根 tests/",
   但虚拟 workspace 清单不会编译根 tests/;`crates/*` 成员模式自动收录,
   src/lib.rs 仅文档注释,全部内容在 tests/。ACL 放宽:**e2e 允许 dev-dep
   rapier3d 构造"单世界参考"**——这是 demo A/D 的方法论(迁移路径 vs 连续路径
   的对照),test-only、非产品码,蓝图 §2 的"rapier 不出 physics-sim"红线
   仍约束生产 crate);
2. **AC2 参考世界 = 裸 PhysicsWorld**(同地面 collider、同刚体参数直接构造,
   无分片语义)——"不迁移的连续路径";分片侧走真实 Scheduler 两分片;
   误差 < 1e-4 m(REQ §10;demo A 实测 ~5e-6,余量充足);
3. **AC3 场景无地面、实体互不接触**(纯弹道)——每实体冲量与积分在两个世界
   里逐位独立,位级等价才有保证(demo D 结论);接触/地面属 AC2 侧覆盖;
4. **AC8 长程 = 600 拍,snapshot@300 中途**,重放到位级 ==(sim-api 测试,无墙钟);
5. **激活 = 纯策略机 + ShardSim 接线**:策略(`ActivationController::plan`)
   只做决策(输入:观察者/已载 chunk/实体表;输出:决策集),ShardSim 落实
   tier③(set_body_type 冻结/唤醒),tier①② 的 chunk 装卸请求返回给调用方
   (chunk 数据源是 REQ-002 的事,本轮调度器忽略该返回,doc 注明);
6. **冻结体语义**(§9.12):不响应事件(detonate 跳过)、不参与凝固桥与迁移
   检测(kinematic 自然不动,防御性跳过);**显式命令触及才唤醒**(Teleport/
   Spawn 重建即醒),唤醒后超预算立即按距离淘汰(`enforce_budget`);
7. **几何口径**:chunk 距离 = chunk 中心(局部 8,8)到观察者的水平(x,z)距离;
   实体距离 = 3D 欧氏;均取"到最近演员";滞回 = 近半径装载/唤醒、远半径卸载/
   冻结(中间带保持),冷却 180 拍内不得回换;换装预算 = 装卸**合计** ≤ 2/tick;
8. **Scheduler 增加 `set_observers`**(存表;tick 流程在取前缀之前对每分片
   `apply_activation`,冻结在本拍 step 生效;观察者两拍之间设置)。
