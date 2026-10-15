# 任务 01:配置单一来源(config.rs)

## 目标

蓝图 §6:全部参数定义在 `app::config`(结构体 + Default),toml 可覆写,
**唯一默认值来源**;构造时按子结构注入各核 crate(字段全显式,无全局单例)。

## 必读

- `docs/design/codebase-layout.md` §6(参数表——默认值的唯一裁判)
- `crates/shard/src/scheduler.rs`(SchedulerConfig/RouterConfig)
- `crates/physics-sim/src/sim.rs`(ShardSimConfig/SolidifyConfig)
- `crates/sim-api/src/session.rs`(SessionConfig)
- `docs/tasks/app/README.md` 决策 9

## 交付物

`crates/app/src/config.rs`:类型 + 内嵌测试。

## 接口设计(签名固定)

```rust
/// 激活参数(REQ-001 §4.3;三档激活尚未实现,参数先入表作为单一来源,
/// 消费者在后续轮次接入——doc 注明)。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(default)]
pub struct ActivationConfig {
    pub load_radius: u32,        // 32(chunk、collider 两档同参)
    pub unload_radius: u32,      // 48
    pub awake_load_radius: u32,  // 24
    pub awake_unload_radius: u32, // 32
    pub cooldown_secs: u32,      // 3
}

/// 全部参数(蓝图 §6 表 + app 装配参数,README 决策 9)。
/// 全字段 #[serde(default)]:toml 文件可只写要覆写的字段。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
    pub dt: f32,                         // 1/60(共识 1)
    pub margin_meters: u32,              // 16(§9.1)
    pub activation: ActivationConfig,
    pub max_awake_dynamic: usize,        // 512(§9.4)
    pub swap_budget_chunks_per_tick: u32, // 2(§4.3)
    pub despawn_after_secs: u64,         // 30(§9.3)
    pub envelope_queue_capacity: usize,  // 1024(§4.2)
    pub inbox_capacity: usize,           // 256(shard README 决策 1)
    pub checkpoint_interval_ticks: u64,  // 600(§9.11)
    pub shard_alert_ms: f32,             // 5(P95,持续 60 tick,§9.2)
    pub worker_count: usize,             // 4(装配参数)
    pub data_dir: PathBuf,               // "data"(redb 库目录,装配参数)
}
// Default 全部按上表注释中的数值实现(§6 表 + 决策 9)。

#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("配置解析失败:{0}")]
    Parse(String),
    #[error("配置读取失败:{0}")]
    Io(#[from] std::io::Error),
}

impl Config {
    /// 从 toml 文本构造(部分覆写:缺省字段取 Default)。空串 = 全默认。
    pub fn from_toml_str(text: &str) -> Result<Self, ConfigError>;

    /// 读文件覆写;文件不存在 → 全默认(不报错);路径 None → 全默认。
    pub fn load(path: Option<&Path>) -> Result<Self, ConfigError>;

    /// 序列化回 toml(含全部字段,roundtrip 用)。
    pub fn to_toml_string(&self) -> String;

    /// → SchedulerConfig(despawn_after_ticks = round(secs/dt);
    /// router = {inbox_capacity, envelope_queue_capacity};start_tick = ZERO)。
    pub fn scheduler_config(&self) -> shard::SchedulerConfig;

    /// → SessionConfig(source = InputSource(1))。
    pub fn session_config(&self) -> sim_api::SessionConfig;

    /// → ShardSimConfig(gravity=(0,-9.81,0)、dt、solidify 默认)。
    pub fn shard_sim_config(&self, shard: ShardId) -> physics_sim::ShardSimConfig;
}
```

## 语义细则

1. `#[serde(default)]` 挂结构体级(嵌套 ActivationConfig 同)——toml 缺字段即默认;
2. `despawn_after_ticks = (despawn_after_secs as f32 / dt).round() as u64`
   (30s @ 1/60 = 1800,断言此值);
3. `data_dir` 用 `PathBuf`(serde 支持;toml 中字符串路径);
4. 数值字段拒绝 NaN/负值?不做显式校验(蓝图 §8 不做多余工程),
   但 `dt` 为 0 时 `scheduler_config` 会除零——`dt <= 0.0` 时 panic
   (配置错误 = fail-stop,doc 注明)。

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `defaults_match_blueprint_table` | 逐字段断言 §6 表 + 决策 9 的默认值(含嵌套 activation 五项) |
| `toml_partial_override_keeps_defaults` | toml 只写 `dt = 0.02` 与 `worker_count = 8` → 其余全默认 |
| `toml_roundtrip_is_lossless` | to_toml_string → from_toml_str → 相等 |
| `missing_file_is_defaults` | `load(None)` 与 `load(Some(不存在路径))` = Default |
| `scheduler_config_converts_units` | 30s @ dt=1/60 → despawn_after_ticks == 1800;queue/inbox 容量映射正确 |
| `shard_and_session_configs_convert` | dt/gravity 传递;session source == InputSource(1) |
| `zero_dt_panics` | `#[should_panic]` dt=0 → scheduler_config panic |

## 完成定义

见 [README 公共完成定义](README.md)。
