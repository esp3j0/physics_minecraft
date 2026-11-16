# physics_minecraft:实施方须知

Rust 体素物理游戏。这份文件给实施方(代码代理)看：项目怎么分工、怎么跑、哪些不能碰。任务的具体内容在 GitHub issue 里的任务书，这里只放长期有效的东西。

## 分工

- **架构师**写任务书、独立验收、维护 `docs/requirements/` 和 `docs/tasks/`。实施方不改这两个目录，也不改本文件。
- **实施方**按任务书实现、自测、提交、在 issue 里回报。任务书里的模块划分、数据结构、签名、算法步骤、测试期望值都照做，不自行另定。
- 设计与现有代码对不上、期望值疑似有错、遇到范围外的问题(磁盘满、Access denied、需要登录、网络不通):**停下，在 issue 里写清查证过程和实测值**,不要自己改设计、不要改期望值、不要删文件、不要改系统设置、不要装软件。
- 任务书的期望值由架构师手算。测试红了先查生产代码。不许用被测函数、也不许在测试里另写一套同样的算法来算期望。
- 如实报告：达不到就报实测值，原因未知写“未归因”,没有实验支持的解释不写。变异存活就写存活，不为了变红去放宽或拼凑测试。

## 仓库结构(crates/)

| crate | 内容 |
|---|---|
| `types` | 共享数据类型。`body.rs`:主体身体定义 `BodyDef`(连杆、关节、形状、哈希) |
| `voxel` | 体素与区块 |
| `physics-sim` | 物理内核(rapier3d 0.36)。`agent.rs`(很长):主体的生成、关节电机、冻结、迁移 |
| `shard`、`sim-api`、`messaging`、`storage` | 分片调度、对外接口、消息类型、存档(redb) |
| `app` | 服务器驱动(`driver.rs`)、网络、控制器(`controller/`) |
| `viewer` | 客户端 |
| `body-import` | URDF → `BodyDef` 离线导入库(`frames`、`tree`、`shapes`、`mesh`、`inertia`、`emit`) |
| `xtask` | `cargo xtask smoke`、`cargo xtask import-body` 等 |
| `e2e` | 端到端测试、压力工具 |

资产在 `assets/`(`assets/bodies/*.body.toml`),需求在 `docs/requirements/`,当前主线是 `REQ-012-agents.md`。

## 命令

- 相关包的测试：`cargo test -j 10 -p <包>`。**一律带 `-j 10`。**
- 全量门禁(约 20 分钟):`cargo test --workspace -j 10`、`cargo clippy --workspace --all-targets -j 10 -- -D warnings`、`cargo fmt --check`。任务书没要求时，实施方只跑相关包的测试、clippy、fmt;全量门禁和冒烟由架构师跑。
- 冒烟：`cargo xtask smoke --no-ai`。
- CI 用的 clippy 比本地新：别用下标遍历数组(`needless_range_loop`),文档注释的列表项别悬挂缩进。

## 运行纪律

- **长命令前台跑**:不要把命令放到后台再结束回合等通知——无人值守模式下进程会直接退出，后面的步骤就丢了。
- **输出要省**:cargo 的输出重定向到 `target/` 下的文件，再 `grep -E "test result|FAILED|panicked|^error"` 或 `tail -40`。不要把刚写的文件整份打印出来核对，不要反复打印大段 diff。长文件(`agent.rs`、`tree.rs`、`driver.rs`)不要整份读，grep 定位后读需要的行段。
- 不要同时跑两个 Rust 构建。不要另建 target 目录(`target` 是指向 D 盘的目录联接，直接用;**不要跑 `cargo clean`**,它会把联接删掉)。报 LNK1104 或内存不足时，先确认没有残留的 cargo / rustc 进程再重跑，仍不行就停下报告。
- 测试循环都要有上限。单个测试超过 10 分钟没结束当挂死处理。
- 变异自证：改在生产代码上，逐个注入、单独跑、跑完还原，`git status --short` 确认干净。提交里不能带变异。

## 不能碰的东西

- 仓库根的 **`data/`** 是用户的存档：不许打开、改动、删除。测试和冒烟只用 `tempfile::TempDir`,不许在系统临时目录留下文件或目录。
- **不下载任何东西**:第三方模型、网格、资源一律不下载。需要的第三方文件由架构师放进 `assets/third_party/`。
- 不新增第三方依赖，除非任务书写明。`Cargo.lock` 出现任务书没提到的新包就停下报告。
- `.gitignore` 里有 `data/` 规则：往任何 `tests/data/` 加文件要 `git add -f`。不要改 `.gitignore`、`.github/`。

## git 与回报

- 直接在 `main` 上提交。提交信息用中文，例如 `feat(req012): C02 ……`,正文最后一行写 `Refs #<issue 号>`。
- 不许 amend、不许改写已推送的提交、不许 force push。推送：`git pull --rebase origin main` 再 `git push origin HEAD:main`。CI 红了追加提交修。
- 回报写进 `target/` 下的文件，用 `gh issue comment <号> --body-file <文件>` 发，发完用 `gh api` 读回确认。`gh` 偶发超时隔半分钟重试，最多 3 次。
- 回报发完就停下，不要开始下一个单元。

## 已知的坑

- `BodyDef` 的连杆坐标系没有旋转：关节角为 0 时父子连杆朝向相同；形状只有平移 `offset`。URDF 的旋转在导入时全部折算进数据。
- 内核生成主体时不使用 `LinkDef.com` 和 `LinkDef.inertia`,惯量由碰撞形状推出；质量在生成时乘 `agent_mass_scale`(默认 0.01)。
- 内核建凸包碰撞体用 `convex_hull(..).expect(..)`:共线点集会 panic,共面点集不 panic 但得到零体积碰撞体。
- `urdf-rs` 把缺失的 `<limit>` 填成 0;对未知关节类型、重复的子元素在解析阶段就报错。
- 游戏坐标 y 向上、x 向前。URDF 是 z 向上：`(x, y, z)` → 游戏 `(x, z, −y)`。
