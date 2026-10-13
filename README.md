# physics_minecraft

用 Rust 实现"带真实物理"的 Minecraft 探索项目:在方块世界里做原版没有的物理模拟
(重力、结构坍塌、流体等),开发过程中对照几条成熟的 Rust 实现路线。

## 项目结构

```
physics_minecraft/
├── Cargo.toml            # workspace,聚合 crates/ 下所有成员
├── crates/
│   ├── app/              # 可执行入口,把各模块拼在一起(cargo run -p app)
│   ├── physics/          # 物理模拟:重力、碰撞、流体(参考 rapier)
│   └── world/            # 方块世界:方块、区块、存档(参考 pumpkin / ferrumc)
├── docs/
│   ├── process.md        # 开发流程:需求驱动 + 测试驱动 + demo 驱动
│   └── requirements/     # 需求文档(REQ-xxx,确认后才开发)
├── demos/                # Python 验证性 demo(验证通过后迁移进 crates/)
└── references/           # 参考项目(gitignore,仅本地)
    ├── RustCraft-Public/
    ├── pumpkin/
    ├── ferrumc/
    ├── ferrumc-v2/
    └── rapier/
```

## 快速开始

```bash
cargo run -p app     # 跑通入口 demo
cargo test           # 各模块单元测试
cargo check          # 快速编译检查
```

## 开发流程(需求驱动 + 测试驱动 + demo 驱动)

1. **需求先行**:在 `docs/requirements/` 写详细需求文档(模板 [TEMPLATE.md](docs/requirements/TEMPLATE.md)),确认后才开发
2. **demo 验证**:用 Python 在 `demos/` 快速验证核心算法/猜想,发现问题回写需求
3. **测试落地**:把验收标准逐条写成 Rust 单元测试
4. **迁移实现**:demo 无误后在 `crates/` 用 Rust 正式实现,测试全绿即完成

详见 [docs/process.md](docs/process.md)。

## 参考项目(本地克隆到 references/,不入库)

| 项目 | 说明 |
| --- | --- |
| [RustCraft-Public](https://github.com/RustCraftMC/RustCraft-Public) | 我的世界 Rust 重写 |
| [pumpkin](https://github.com/pumpkin-mc/pumpkin) | 我的世界 Rust 服务器 |
| [ferrumc](https://github.com/ferrumc-rs/ferrumc) | 我的世界 Rust 服务器,从 0 实现版 |
| [ferrumc v2](https://github.com/ferrumc-rs/ferrumc/tree/rework/v2-skeleton) | 同上仓库的 v2 从零重写分支(**主要看这个**) |
| [rapier](https://github.com/dimforge/rapier) | Rust 物理引擎 |

重新克隆参考项目:

```bash
git clone --depth 1 https://github.com/RustCraftMC/RustCraft-Public.git references/RustCraft-Public
git clone --depth 1 https://github.com/pumpkin-mc/pumpkin.git references/pumpkin
git clone --depth 1 https://github.com/ferrumc-rs/ferrumc.git references/ferrumc
git clone --depth 1 --branch rework/v2-skeleton https://github.com/ferrumc-rs/ferrumc.git references/ferrumc-v2
git clone --depth 1 https://github.com/dimforge/rapier.git references/rapier
```
