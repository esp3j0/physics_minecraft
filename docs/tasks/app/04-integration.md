# 任务 04:断点续跑端到端集成测试

## 目标

app 的终极验收:**真 redb 库上的断点续跑**——跑 N 拍写检查点 → 关停 →
同目录重新 bootstrap → 从检查点拍继续;外加 toml 文件覆写与告警接线。
本任务由 qwen 执笔,与实现(glm)交叉。

## 必读

- `docs/tasks/app/README.md` 决策 4-7
- `crates/app/src/driver.rs` / `config.rs` / `telemetry.rs`(以源码为准)
- `crates/storage/tests/persistence.rs`(tempfile + 独立库的工程细节)

## 交付物

`crates/app/tests/app_e2e.rs`。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `resume_from_redb_checkpoint_continues_ticks` | tempfile 目录:bootstrap → run 7 拍(interval=5,含终末 checkpoint)→ 丢弃;再 bootstrap 同目录 → 续跑起始拍 == 7 → run 3 拍 → 再丢弃再续 → 起始拍 == 10(链式续跑) |
| `commands_after_resume_apply_to_restored_shards` | 首段注册带地形的分片 + SpawnDynamic 跑 7 拍关停;续跑后向同分片发 SetBlock → 生效(实体经快照恢复可见,续跑后 entity_state 仍 Some) |
| `toml_file_on_disk_overrides_config` | 写临时 toml(只覆写 worker_count 与 checkpoint_interval_ticks)→ Config::load(Some(path)) → 两字段生效、其余默认 |
| `fresh_bootstrap_without_store_starts_at_zero` | 全新目录 bootstrap → 续跑起始拍 None,Session 空 |
| `telemetry_wired_into_driver_outcome` | 阈值调 0.001ms 跑 60+ 拍 → outcome.alert_active_at_exit == true(证明驱动真的在喂 telemetry) |

工程细节:

- 每个测试独立 `tempfile::tempdir()`;`Config` 用 `data_dir` 指向它;
- run 需要 `#[tokio::test(start_paused = true)]`;命令通道在测试手里;
- 续跑链中,前一段必须完整退出(store.shutdown 已释放文件锁)再开下一段——
  这是 redb 文件锁语义,串行 await 即可;
- 实体/地形构造照 `crates/shard/tests/two_shards.rs` 帮助函数风格
  (凝固桥关闭、SimpleRegistry::builtin)。

## 完成定义

见 [README 公共完成定义](README.md)。
