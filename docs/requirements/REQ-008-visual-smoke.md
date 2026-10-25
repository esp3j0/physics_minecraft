# REQ-008:自动化图形冒烟——脚本驱动 viewer、截图、机器检查与 AI 审图

- 状态:**已确认**(2026-10-25 用户拍板 D1–D3)
- 提出日期:2026-10-25
- 上游:
  - [REQ-004A](REQ-004A-visual-viewer.md)(viewer:渲染、飞行相机、B/X 交互;决议"渲染代码不写窗口类测试")
  - [REQ-003](REQ-003-network.md)(C2S/S2C 协议、命令白名单)
  - [REQ-007](REQ-007-activation-tickets.md)(人工冒烟步骤 → 本 REQ 的第一个场景)
- 关联 issue:[#16](https://github.com/esp3j0/physics_minecraft/issues/16)
- 关联 demo:无(理由见 §12)
- 关联代码(预计):`crates/viewer`(脚本模式)、`crates/xtask`(新建,编排器与审图)、`smoke/`(场景、
  审图提示词、校准集)、`.cargo/config.toml`(`xtask` 别名)

---

## 1. 背景与目标

每个 REQ 收尾时要用户手动开 viewer 冒烟,步骤繁琐、结果靠目测、不可重复。cargo 测试覆盖了模拟核、
网络和 viewer 的纯函数层,但"服务器 → 网络 → viewer 渲染"整条链路在真实窗口里看起来对不对,目前只能靠人。
REQ-004A 决定"渲染代码不写窗口类测试",视觉验证因此一直是手动的。

本 REQ 的目标:**一条命令**对真实服务器和真实 viewer 窗口执行脚本化场景——传送、飞行、生成、引爆、
等待——自动截图,机器检查客户端看到的世界状态,再让 AI 对照期望审图,最后输出报告。以后每个 REQ 从需求
阶段就附带冒烟场景,取代手动冒烟。

可行性已验证:zcode CLI 用 `--attach` 把图片交给 glm-5.3-flash,探针图(背景色、两个方块的颜色和左右
位置)识别全对。

## 2. 总体结构

```
smoke/scenarios/*.toml ──► cargo xtask smoke
                              │ 1. 编译 app、viewer(release)
                              │ 2. 每个场景:临时数据目录 + 空闲端口 + 配置 → 启动服务器
                              │ 3. 启动 viewer --script(真实窗口):按步骤执行、截图、机器检查、写 manifest
                              │ 4. 收尾:停服务器,扫描服务器日志
                              │ 5. AI 审图(zcode CLI;先跑校准集)
                              ▼
                    target/smoke/<时间戳>/report.md + report.json + 截图
```

| 组件 | 位置 | 职责 |
| --- | --- | --- |
| 场景脚本 | `smoke/scenarios/*.toml` | 一个文件一个场景:服务器配置覆盖 + 步骤列表 |
| 脚本引擎 | `crates/viewer/src/script/`(纯逻辑,零 macroquad) | 解析、步骤状态机、机器检查;可单测 |
| 脚本模式 | `crates/viewer/src/render.rs` | `--script` 时由引擎驱动相机与命令、截图、写 manifest |
| 编排器 | `crates/xtask`(`cargo xtask smoke`) | 服务器与 viewer 的生命周期、超时、日志、汇总 |
| 审图器 | `crates/xtask` | 调 zcode CLI 审图、校准、出报告 |

## 3. 场景脚本

### 3.1 文件结构

```toml
name = "req007-explode-fly-away"          # 必填,唯一;也是输出子目录名
req = "REQ-007"                            # 选填:来源需求
description = "生成方块、引爆、飞离 60 m、飞回:不悬空、全部凝固"
timeout_secs = 180                          # 选填:整场景墙钟上限(默认 180)

[server]                                    # 选填:覆盖服务器配置(键与 app::Config 的 toml 同名)
world_seed = 7

[viewer]                                    # 选填
view_distance = 4

[[step]]
camera = { pos = [8.0, -48.0, 24.0], yaw = -90.0, pitch = -30.0 }

[[step]]
label = "落地前引爆"                        # 选填:便于报告阅读
detonate = { nearest = true, power = 12.0, radius = 8.0 }
```

- 每个 `[[step]]` **恰好一个**动作键,外加可选的 `label`;
- 未知的顶层键、未知动作、缺必填参数、类型错误 → 解析报错,报出步骤序号与原因,场景不执行;
- 角度单位为度,距离单位为米,时间以服务器拍为单位(`timeout_secs` 除外)。

### 3.2 步骤词汇(v1)

| 动作 | 参数 | 语义 | 完成条件 |
| --- | --- | --- | --- |
| `camera` | `pos`、`yaw`、`pitch` | 相机瞬移(传送);耦合时立即上报观察者 | 当帧 |
| `look_at` | `target` | 调整偏航与俯仰,使相机看向目标点 | 当帧 |
| `fly` | `to`、`speed`(默认 16 m/s) | 沿直线匀速飞向目标,按帧时长推进;观察者按交互模式的节流规则跟随 | 到达目标点 |
| `couple` | `on` | 相机 → 观察者耦合开关(脚本模式默认开) | 当帧 |
| `spawn` | `at` 或(`ahead`、`lift`)、`count`(默认 1)、`spacing`(默认 1.6)、`velocity`(默认 0)、`label` | 生成动态立方体:`at` 为绝对坐标;`ahead`/`lift` 为相机前方距离与抬高;多个时按 5 列网格散开;`velocity` 为初速度(物体飞行);`label` 把服务端回执的实体 ID 记在该名字下 | 收齐 `count` 个 `Spawned` 回执 |
| `detonate` | `at` 或 `nearest = true`、`power`、`radius` | 引爆;`nearest` 取相机附近最近的活实体为爆心(同交互模式的吸附规则) | 当帧发出 |
| `set_block` | `pos`、`block`(方块名或 ID) | 放置或挖掉方块(`air`) | 当帧发出 |
| `wait_ticks` | `n` | 等服务器推进 `n` 拍(以 viewer 收到的服务器拍号为准) | 服务器拍号 ≥ 起点 + `n` |
| `wait_until` | 检查项(§3.3)+ `timeout_ticks` | 等到检查项全部成立 | 成立;超时则场景失败 |
| `check` | 检查项(§3.3) | 立即断言;失败记入结果,场景继续执行(便于收集后续截图) | 当帧 |
| `screenshot` | `name`、`expect`、`hud`(默认 false) | 当帧渲染完成后截图;`expect` 是给 AI 的自然语言期望;`hud = false` 时截图帧不画 HUD 文字 | 文件写出 |

等待精度受服务器实体状态流的发送周期限制(默认每 10 拍一包),`wait_ticks` 实际可能多等不足一个周期。

### 3.3 机器检查项(v1)

检查只基于 viewer 收到的世界状态(实体表、方块镜像、服务器拍号),不读服务器内部。

| 检查项 | 写法 | 成立条件 |
| --- | --- | --- |
| 活实体数 | `live_entities = { eq = 0 }`(也可 `le`、`ge`) | 实体表计数满足比较 |
| 标签实体存活数 | `labeled_alive = { label = "blast", eq = 0 }` | 该标签下仍在实体表里的个数满足比较 |
| 方块 | `block = { pos = [x, y, z], is = "stone" }` | 方块镜像中该格等于给定方块 |
| 已装载 chunk 数 | `chunks_loaded = { ge = 9 }` | 方块镜像中的 chunk 数满足比较 |
| 无悬空 | `no_floating = { height = 1.5, still_ticks = 30 }` | 不存在这样的活实体:底面高出所在柱最高实心方块 `height` 米以上,且在最近 `still_ticks` 拍内位置变化不超过 0.05 m(正在下落的不算) |

### 3.4 时间与确定性

服务器按真实时间运行,两次运行的碎片落点会有差异。因此**不做像素比对**,判定只靠机器检查(确定的状态条件)
加 AI 的语义判断。场景应使用固定 `world_seed`。

## 4. viewer 脚本模式

- **入口**:`viewer --script <场景文件> --out <输出目录> [--addr <服务器地址>]`;不带 `--script` 时行为与现在
  完全一致(REQ-004A 不变)。
- **输入**:脚本模式下忽略键盘与鼠标,不锁定指针;窗口固定 1280×720、不可调整大小;相机 → 观察者耦合默认开。
- **引擎与渲染分离**:脚本引擎是纯逻辑模块,每帧接收(帧时长、最新服务器拍号、世界视图的只读摘要),输出
  (相机位姿、要发送的命令、截图请求、检查结果、完成/失败)。渲染层只负责执行这些输出。引擎因此可以在没有
  窗口的单测里驱动。
- **截图**:在当帧 3D 场景(及 `hud = true` 时的 HUD)画完后取帧缓冲导出 PNG,文件名 `NN-<name>.png`
  (`NN` 为步骤序号,两位补零)。
- **manifest**:`<输出目录>/manifest.json`,包含场景元数据,以及每一步的序号、动作、标签、起止服务器拍号、
  相机位姿、结果(成功 / 失败 + 原因);截图步骤另记文件名、`expect`、`hud`、当时的活实体数与 chunk 数。
- **退出码**:0 = 全部步骤完成且检查全部成立;1 = 有检查失败;2 = 脚本错误(解析失败、`wait_until` 超时、
  断连、整场景超时)。场景结束后窗口自动关闭。
- **顺带修正**:viewer 的初始相机高度与爆心兜底高度(`BLAST_Y`)仍按旧地表 y = 4 写死,而生产超平坦地表在
  y = −60。改为相对实际地表(或改为合理的新常量),交互模式与脚本模式一致。

## 5. 编排器:`cargo xtask smoke`

- 新建 `crates/xtask`(bin),`.cargo/config.toml` 增加别名 `xtask = "run --package xtask --"`。
- **命令**:`cargo xtask smoke [场景名或路径 ...] [--no-ai] [--out <目录>]`;不给场景时跑 `smoke/scenarios/`
  下全部场景;另有 `cargo xtask smoke --selftest`(§10 AC-S8)。
- **每个场景**:
  1. 新建临时数据目录;选一个空闲端口;用 `app::config::Config` 生成配置文件(`listen_addr`、`data_dir`、
     场景 `[server]` 覆盖项),保证通过配置校验;
  2. 启动 `target/release/app <配置文件>`,等到标准输出出现"监听 <地址>"或端口可连接(上限 30 s);
  3. 启动 `target/release/viewer --script ... --out ...`,等待退出(上限 = 场景 `timeout_secs`);
  4. 无论 viewer 成败,都结束服务器进程;服务器标准输出与标准错误存到输出目录;日志中出现 panic 即判场景失败。
- **输出**:默认 `target/smoke/<YYYYmmdd-HHMMSS>/`,每个场景一个子目录(截图、manifest、服务器日志),顶层
  `report.md` 与 `report.json`。
- **退出码**:任一场景有机器检查失败、脚本错误或服务器 panic → 非 0;AI 审图结论不影响退出码(D1)。

## 6. AI 审图

- **调用**:每张截图单独一次,顺序执行(zcode 本地会话库不支持并发):
  `node <zcode.mjs> -p <提示词> --attach <截图> --mode plan --json --cwd <场景输出目录>`。`zcode.mjs` 路径取环境
  变量 `ZCODE_MJS`,缺省 `%LOCALAPPDATA%\zcode\runtime\current\bin\zcode.mjs`;模型用 zcode 默认配置
  (公司 / glm-5.3-flash / max)。单次超时 180 s。
- **提示词**:模板存于 `smoke/review-prompt.md`,内容包括:画面约定(天空色背景;地形与已凝固方块是地形色;
  **蓝色立方体是尚未凝固的活实体**;截图默认不含 HUD)、本张截图的 `expect`、要求先描述看到的内容再下结论、
  只输出一个 JSON 对象:`{"observed": "...", "pass": true|false, "issues": ["..."], "confidence": 0.0–1.0}`。
- **解析**:从回复中取第一个 JSON 对象(容忍代码块包裹);解析失败或调用失败 → 结论记为 `unknown`。
- **标记规则**:`pass = false`、`confidence < 0.7` 或 `unknown` → 标记"待复核",由总架构师打开截图复核(D1)。
- **校准**:`smoke/calibration/` 存放已知好坏的标准截图与 `calibration.toml`(每条:文件、`expect`、应得的
  `pass`),至少 2 张应判通过、2 张应判失败(例如:期望"空中没有蓝色立方体",而图中有悬空的蓝色立方体)。
  每次审图前先判校准集;**任何一条判错,本次所有 AI 结论作废**,全部截图标记待复核。校准图取自真实冒烟运行,
  经总架构师确认后入库。

## 7. 报告

- `report.md`:汇总表(每个场景的机器检查、脚本状态、AI 通过 / 待复核数、校准是否通过);每个场景一节,
  列出步骤结果,内嵌截图(相对路径),附 `expect` 与 AI 的 `observed`、结论、置信度、问题。
- `report.json`:同样内容的机器可读版本。

## 8. 纳入开发流程

- **需求阶段**:`docs/requirements/TEMPLATE.md` 增加"图形冒烟场景"一节;涉及玩法、渲染、流式装载或实体行为的
  REQ,由总架构师在写验收用例时一并写场景脚本。
- **实施门禁**:上述 REQ 的单元交付时,实施方运行相关场景,机器检查必须全部通过;回评附报告摘要与待复核截图清单。
- **验收**:总架构师查看报告,亲自复核待复核截图。
- **用户**:不再需要手动冒烟,除非用户主动要求。
- REQ-004A"渲染代码不写窗口类测试"的决议对 `cargo test` 继续有效;窗口层验证改由 `cargo xtask smoke` 承担,
  不进 `cargo test`,也不进 CI(D2)。

## 9. 首批场景(本 REQ 的验收载体)

1. **`basic-world`**:连接 → 等地形装载 → 依次传送到 3 个位置,各截一张俯视图(期望:地面完整可见,没有缺块、
   空洞或缺失的 chunk)→ 检查 `chunks_loaded`。
2. **`req007-explode-fly-away`**:REQ-007 的人工冒烟改写——相机就位 → 生成 8 个立方体 → 落地凝固前引爆 →
   截图"after-blast"(期望:若干蓝色立方体正在四散飞起)→ 飞到 80 m 外 → 等 300 拍 → 飞回 →
   `wait_until live_entities eq 0`(上限 900 拍)→ `check no_floating` → 截图"return"(期望:地面有凝固的方块,
   空中没有悬浮的蓝色立方体)。
3. **`selftest-negative`**:专供 `--selftest` 使用,包含必然失败的检查(例如引爆后立即断言 `live_entities eq 0`)。

## 10. 验收标准

- **AC-S1 脚本解析**:§3.2 全部动作与 §3.3 全部检查项能解析;未知键、未知动作、一步多个动作、缺必填参数、
  类型错误各自报错,并报出步骤序号。
- **AC-S2 引擎状态机**:用模拟输入(帧时长、服务器拍号、世界视图摘要)驱动,无窗口:`camera`/`look_at` 当帧生效;
  `fly` 按速度在预期帧数内到达;`wait_ticks` 在正确的拍号完成;`wait_until` 成立即完成、超时即失败;`spawn`
  收齐回执才完成并按标签记下 ID;`screenshot` 产生截图请求;步骤按顺序推进。
- **AC-S3 检查项**:每个检查项都有成立与不成立两种构造;`no_floating` 能识别悬空静止的实体,且不把正在下落的
  实体判为悬空。
- **AC-S4 脚本模式端到端**:对真实服务器跑 `basic-world`,产出 manifest 与截图:截图为 1280×720 PNG、不是纯色、
  方向正确(上部以天空色为主、下部以地形色为主);无 `--script` 时交互模式行为不变。
- **AC-S5 编排器**:临时目录与空闲端口、就绪等待、viewer 失败或超时时服务器仍被结束、服务器 panic 被识别、
  退出码符合 §5。纯逻辑部分(配置生成、日志扫描、退出码汇总)有单测。
- **AC-S6 AI 审图**:审图器经接口抽象,可用假审图器做单测:提示词构造、JSON 提取(含代码块包裹)、标记规则、
  校准失败作废全部结论。真实运行时校准集全部判对。
- **AC-S7 报告**:用固定的 manifest 与审图结果生成 `report.md`/`report.json`,字段齐全(单测)。
- **AC-S8 自检**:`cargo xtask smoke --selftest` 运行 `selftest-negative` 与校准集,确认机器检查能报失败、
  校准集中应判失败的图被判失败;自检本身以"预期的失败都发生了"为成功。
- **AC-S9 首个场景**:在当前 main 上 `cargo xtask smoke req007-explode-fly-away` 机器检查全部通过、校准通过;
  两张截图 AI 判通过,或被标记待复核的截图经总架构师复核确认无误。
- **AC-S10 回归**:`cargo test --workspace`、`cargo clippy --workspace --all-targets -- -D warnings`、fmt 全绿;
  新增纯逻辑代码有单测;CI 不运行窗口与 AI 相关步骤。

## 11. 非目标

- 进 CI(含无显示器的软件渲染)(D2);
- 像素级金标准截图比对;录像;性能基准;
- 多客户端同时冒烟;
- 让 AI 审图结论成为硬门禁(D1);
- 服务器锁步时间控制(脚本逐拍推进服务器)——后续候选;
- 自动生成场景脚本。

## 12. demo 记录

不做 Python demo:本 REQ 是工具链,没有新的数值算法。唯一的可行性风险(模型能否看图)已用 zcode CLI 探针验证。

## 13. 决议记录

| ID | 议题 | 决定 | 日期 |
| --- | --- | --- | --- |
| D1 | AI 审图的地位 | 辅助判定:机器检查是硬门禁;AI 判失败、低置信或无法解析的截图标记待复核,由总架构师复核 | 2026-10-25 |
| D2 | 运行环境 | 只在本机(Windows,有显卡,运行时弹出窗口);不进 CI | 2026-10-25 |
| D3 | 编排器 | `cargo xtask`(Rust,仓库内,可复用 `app::config::Config`) | 2026-10-25 |

## 14. 建议拆分(验收单元)

1. **01 脚本格式、引擎与检查项**(viewer 纯逻辑模块;AC-S1–S3);
2. **02 viewer 脚本模式、截图与 manifest**,含 §4 顺带修正(AC-S4);
3. **03 编排器与首批场景**(AC-S5;机器检查部分的 AC-S9);
4. **04 AI 审图、校准集、报告与流程收口**(AC-S6–S8、AC-S9 全部、TEMPLATE 与 process.md 更新)。

## 迁移清单(阶段 3 填写)

- [ ] 验收标准 AC-S1–S10 落成测试或可执行检查
- [ ] `cargo test --workspace` / clippy / fmt 全绿
- [ ] 首个场景经 `cargo xtask smoke` 跑通,报告经总架构师复核
- [ ] TEMPLATE.md 与 process.md 加入图形冒烟要求
