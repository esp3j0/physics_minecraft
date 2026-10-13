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
