# 任务 05:REQ-002 验收套件(AC-G1~G6)

## 目标

REQ-002 §10 六条验收标准的端到端验证(qwen 执笔,与实现交叉)。

## 必读

- `docs/requirements/REQ-002-world-loading.md` §10(验收原文)
- `demos/solidify-slopes/README.md`(P2′ 数据,理解 3mm 门)
- `crates/e2e/tests/`(既有套件风格;dev-deps 需追加 app 与 storage 的使用)
- 任务 00-04 产出的公开 API(以源码为准)

## 交付物

`crates/e2e/tests/world_loading.rs`(可能需要 e2e dev-deps 增补 app——授权)。

## 测试清单(逐条对 AC)

| 测试名 | 断言意图(AC) |
| --- | --- |
| `acg1_generation_is_bitwise_deterministic` | 同 (seed,pos) 跨实例两次生成位级相等;不同 seed SeededScatter 有差(AC-G1) |
| `acg2_store_takes_priority_over_generator` | chunk 修改→持久化→重装载取回修改值(经 provider 真路径)(AC-G2) |
| `acg3_observer_walk_builds_playable_world` | 驱动全链:MoveObserver 进新区 → 分片生成 → 地面装载 → SpawnDynamic 实体落在生成地面上被支撑(≥200 tick 稳定)(AC-G3) |
| `acg4_lifecycle_roundtrip_preserves_data` | 建片+改方块(chunk 修改)→ 观察者远离 → despawn(缩短 despawn 配置)→ 观察者回 → 分片复活,修改过的 chunk 数据等价(AC-G4) |
| `acg5_replay_with_observers_and_loading` | 含观察者移动/装载/爆炸的场景跑 600 拍,拍 300 快照,重放到位级一致(AC-G5;对拍 AC8 长程的扩展场景) |
| `acg6_persisted_after_process_restart` | despawn + 终末 flush 后,重开 redb 库逐 chunk 完整可读(AC-G6) |

工程细节:driver 路径用 `#[tokio::test(start_paused = true)]`;despawn 窗口
用小配置(如 despawn_after_secs=2)加速;AC-G4 的"复活"经驱动 ensure 路径
(观察者移回)而非手工 register;位级断言 `==`,物理量按容差。

## 完成定义

见 [README 公共完成定义](README.md)。
