//! 全部参数(蓝图 §6)+ toml 可覆写;唯一默认值来源。
//!
//! 激活参数(REQ-001 §4.3):经 `shard_sim_config` 换算注入激活策略机
//! (任务书 req001-backlog/02 接入)。

use std::path::{Path, PathBuf};

use messaging::InputSource;
use physics_sim::{ShardSimConfig, SolidifyConfig};
use serde::{Deserialize, Serialize};
use shard::SchedulerConfig;
use sim_api::session::SessionConfig;
use types::{ShardId, Tick};

/// 激活参数(REQ-001 §4.3;经 `shard_sim_config` 换算注入策略机
/// `physics_sim::ActivationConfig`,任务书 req001-backlog/02 已接入)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ActivationConfig {
    /// 加载半径(chunk、collider 两档同参)。
    pub load_radius: u32,
    /// 卸载半径。
    pub unload_radius: u32,
    /// 唤醒加载半径。
    pub awake_load_radius: u32,
    /// 唤醒卸载半径。
    pub awake_unload_radius: u32,
    /// 激活冷却(秒)。
    pub cooldown_secs: u32,
}

impl Default for ActivationConfig {
    fn default() -> Self {
        Self {
            load_radius: 32,
            unload_radius: 48,
            awake_load_radius: 24,
            awake_unload_radius: 32,
            cooldown_secs: 3,
        }
    }
}

/// 全部参数(蓝图 §6 表 + app 装配参数,app README 决策 9)。
/// 结构体级 `#[serde(default)]`:toml 文件可只写要覆写的字段。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
    /// 步长(秒),1/60(共识 1)。
    pub dt: f32,
    /// 分片本体外扩边距(格),16(§9.1)。
    pub margin_meters: u32,
    pub activation: ActivationConfig,
    /// 唤醒动态实体预算,512(§9.4)。
    pub max_awake_dynamic: usize,
    /// 每拍 chunk 换装预算,2(§4.3)。
    pub swap_budget_chunks_per_tick: u32,
    /// 空片 despawn 阈值(秒),30(§9.3)。
    pub despawn_after_secs: u64,
    /// 跨片信封队列容量,1024(§4.2)。
    pub envelope_queue_capacity: usize,
    /// 每分片命令收件箱容量,256(app README 决策 1)。
    pub inbox_capacity: usize,
    /// checkpoint 间隔(拍),600(§9.11)。
    pub checkpoint_interval_ticks: u64,
    /// 分片 step P95 告警阈值(ms;持续 60 tick),5(§9.2)。
    pub shard_alert_ms: f32,
    /// worker 数(装配参数)。
    pub worker_count: usize,
    /// 数据目录(redb 库目录,装配参数)。
    pub data_dir: PathBuf,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            dt: 1.0 / 60.0,
            margin_meters: 16,
            activation: ActivationConfig::default(),
            max_awake_dynamic: 512,
            swap_budget_chunks_per_tick: 2,
            despawn_after_secs: 30,
            envelope_queue_capacity: 1024,
            inbox_capacity: 256,
            checkpoint_interval_ticks: 600,
            shard_alert_ms: 5.0,
            worker_count: 4,
            data_dir: PathBuf::from("data"),
        }
    }
}

/// 配置加载可预期失败(类型化错误;蓝图 §4:IO/解析错误逐层上抛)。
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("配置解析失败:{0}")]
    Parse(String),
    #[error("配置读取失败:{0}")]
    Io(#[from] std::io::Error),
}

impl Config {
    /// 从 toml 文本构造(部分覆写:缺省字段取 Default)。空串 = 全默认。
    pub fn from_toml_str(text: &str) -> Result<Self, ConfigError> {
        toml::from_str(text).map_err(|e| ConfigError::Parse(e.to_string()))
    }

    /// 读文件覆写;文件不存在 → 全默认(不报错);路径 None → 全默认。
    pub fn load(path: Option<&Path>) -> Result<Self, ConfigError> {
        let Some(path) = path else {
            return Ok(Self::default());
        };
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            // 文件不存在 = 未提供配置文件,取全默认(不报错)。
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Self::default()),
            Err(e) => return Err(e.into()),
        };
        Self::from_toml_str(&text)
    }

    /// 序列化回 toml(含全部字段,roundtrip 用)。
    pub fn to_toml_string(&self) -> String {
        toml::to_string(self).expect("Config 全字段可序列化(无_maps/枚举)")
    }

    /// → SchedulerConfig(despawn_after_ticks = round(secs/dt);
    /// router = {inbox_capacity, envelope_queue_capacity};start_tick = ZERO)。
    ///
    /// `dt <= 0.0` → panic(配置错误 = fail-stop,细则 4)。
    pub fn scheduler_config(&self) -> SchedulerConfig {
        assert!(
            self.dt > 0.0,
            "配置错误:dt = {} 必须 > 0(fail-stop)",
            self.dt
        );
        SchedulerConfig {
            worker_count: self.worker_count,
            router: shard::router::RouterConfig {
                inbox_capacity: self.inbox_capacity,
                queue_capacity: self.envelope_queue_capacity,
            },
            despawn_after_ticks: (self.despawn_after_secs as f32 / self.dt).round() as u64,
            start_tick: Tick::ZERO,
        }
    }

    /// → SessionConfig(source = InputSource(1))。
    pub fn session_config(&self) -> SessionConfig {
        SessionConfig {
            scheduler: self.scheduler_config(),
            source: InputSource(1),
        }
    }

    /// → ShardSimConfig(gravity=(0,-9.81,0)、dt、solidify 默认;
    /// activation 由本表换算(任务书 req001-backlog/02 接入):
    /// 半径 u32 → f32 直传,cooldown_secs 按拍换算(与 despawn_after_ticks
    /// 同式),预算字段直传)。
    pub fn shard_sim_config(&self, shard: ShardId) -> ShardSimConfig {
        ShardSimConfig {
            shard,
            gravity: types::glam::Vec3::new(0.0, -9.81, 0.0),
            dt: self.dt,
            solidify: SolidifyConfig::default(),
            activation: physics_sim::ActivationConfig {
                chunk_load_radius: self.activation.load_radius as f32,
                chunk_unload_radius: self.activation.unload_radius as f32,
                awake_load_radius: self.activation.awake_load_radius as f32,
                awake_unload_radius: self.activation.awake_unload_radius as f32,
                cooldown_ticks: (self.activation.cooldown_secs as f32 / self.dt).round() as u64,
                max_awake_dynamic: self.max_awake_dynamic,
                swap_budget_chunks_per_tick: self.swap_budget_chunks_per_tick as usize,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use physics_sim::SolidifyConfig;
    use types::{DimensionId, ShardPos, WorldId};

    /// 测试用分片身份(与 shard/sim-api 测试同风格)。
    fn shard_id(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    /// 测试清单 1:逐字段断言蓝图 §6 表 + 决策 9 的默认值(含嵌套 activation 五项)。
    #[test]
    fn defaults_match_blueprint_table() {
        let c = Config::default();
        assert_eq!(c.dt, 1.0 / 60.0, "dt = 1/60(共识 1)");
        assert_eq!(c.margin_meters, 16, "margin_meters = 16(§9.1)");
        assert_eq!(
            c.activation,
            ActivationConfig {
                load_radius: 32,
                unload_radius: 48,
                awake_load_radius: 24,
                awake_unload_radius: 32,
                cooldown_secs: 3,
            },
            "activation 五项(§4.3)"
        );
        assert_eq!(c.max_awake_dynamic, 512, "max_awake_dynamic = 512(§9.4)");
        assert_eq!(
            c.swap_budget_chunks_per_tick, 2,
            "swap_budget_chunks_per_tick = 2(§4.3)"
        );
        assert_eq!(c.despawn_after_secs, 30, "despawn_after_secs = 30(§9.3)");
        assert_eq!(
            c.envelope_queue_capacity, 1024,
            "envelope_queue_capacity = 1024(§4.2)"
        );
        assert_eq!(c.inbox_capacity, 256, "inbox_capacity = 256(决策 1)");
        assert_eq!(
            c.checkpoint_interval_ticks, 600,
            "checkpoint_interval_ticks = 600(§9.11)"
        );
        assert_eq!(c.shard_alert_ms, 5.0, "shard_alert_ms = 5(§9.2)");
        assert_eq!(c.worker_count, 4, "worker_count = 4(决策 9)");
        assert_eq!(c.data_dir, PathBuf::from("data"), "data_dir = data(决策 9)");
    }

    /// 测试清单 2:toml 只写 `dt = 0.02` 与 `worker_count = 8` → 其余全默认
    /// (结构体级 serde(default) 语义)。
    #[test]
    fn toml_partial_override_keeps_defaults() {
        let c = Config::from_toml_str("dt = 0.02\nworker_count = 8\n").expect("可解析");
        assert_eq!(c.dt, 0.02, "覆写生效");
        assert_eq!(c.worker_count, 8, "覆写生效");
        let d = Config::default();
        assert_eq!(c.margin_meters, d.margin_meters);
        assert_eq!(c.activation, d.activation);
        assert_eq!(c.max_awake_dynamic, d.max_awake_dynamic);
        assert_eq!(c.swap_budget_chunks_per_tick, d.swap_budget_chunks_per_tick);
        assert_eq!(c.despawn_after_secs, d.despawn_after_secs);
        assert_eq!(c.envelope_queue_capacity, d.envelope_queue_capacity);
        assert_eq!(c.inbox_capacity, d.inbox_capacity);
        assert_eq!(c.checkpoint_interval_ticks, d.checkpoint_interval_ticks);
        assert_eq!(c.shard_alert_ms, d.shard_alert_ms);
        assert_eq!(c.data_dir, d.data_dir);
    }

    /// 测试清单 3:to_toml_string → from_toml_str → 相等(含嵌套 activation)。
    #[test]
    fn toml_roundtrip_is_lossless() {
        let c = Config::default();
        let text = c.to_toml_string();
        let back = Config::from_toml_str(&text).expect("自有输出必可解析");
        assert_eq!(back, c, "roundtrip 无损");

        // 覆写后的实例同样无损(每个字段都真实走一遍序列化)。
        let custom = Config {
            dt: 0.03,
            activation: ActivationConfig {
                load_radius: 64,
                ..ActivationConfig::default()
            },
            data_dir: PathBuf::from("somedir/store"),
            ..Config::default()
        };
        let back2 = Config::from_toml_str(&custom.to_toml_string()).expect("可解析");
        assert_eq!(back2, custom, "覆写实例 roundtrip 亦无损");
    }

    /// 测试清单 4:`load(None)` 与 `load(Some(不存在路径))` = Default(不报错)。
    #[test]
    fn missing_file_is_defaults() {
        let none = Config::load(None).expect("None 路径不报错");
        assert_eq!(none, Config::default(), "None → 全默认");

        let missing = std::env::temp_dir().join("app-config-不存在-9e3f.toml");
        let absent = Config::load(Some(&missing)).expect("文件不存在不报错");
        assert_eq!(absent, Config::default(), "不存在的文件 → 全默认");
    }

    /// 测试清单 5:30s @ dt=1/60 → despawn_after_ticks == 1800;
    /// queue/inbox 容量映射正确;worker/start_tick 按注释映射。
    #[test]
    fn scheduler_config_converts_units() {
        let c = Config::default();
        let sc = c.scheduler_config();
        assert_eq!(
            sc.despawn_after_ticks, 1800,
            "30s @ 1/60 = 1800 拍(细则 2 断言此值)"
        );
        assert_eq!(sc.router.inbox_capacity, c.inbox_capacity, "收件箱容量映射");
        assert_eq!(
            sc.router.queue_capacity, c.envelope_queue_capacity,
            "信封队列容量映射"
        );
        assert_eq!(sc.worker_count, c.worker_count, "worker 数映射");
        assert_eq!(sc.start_tick, Tick::ZERO, "起始拍 = ZERO");

        // 换一组值验证换算公式(round)而非巧合:2s @ dt=0.05(每秒 20 拍)= 40 拍。
        let c2 = Config {
            dt: 0.05,
            despawn_after_secs: 2,
            ..Config::default()
        };
        assert_eq!(c2.scheduler_config().despawn_after_ticks, 40);
    }

    /// 测试清单 6:dt/gravity 传递;session source == InputSource(1)。
    #[test]
    fn shard_and_session_configs_convert() {
        let shard = shard_id(1, 2);
        let c = Config::default();

        let sim_cfg = c.shard_sim_config(shard);
        assert_eq!(sim_cfg.shard, shard, "分片身份传递");
        assert_eq!(sim_cfg.dt, c.dt, "dt 传递");
        assert_eq!(
            sim_cfg.gravity,
            types::glam::Vec3::new(0.0, -9.81, 0.0),
            "gravity = (0, -9.81, 0)"
        );
        // SolidifyConfig 无 PartialEq(外部 crate 类型):逐字段断言取默认。
        let sd = SolidifyConfig::default();
        assert!(
            sim_cfg.solidify.enabled == sd.enabled
                && sim_cfg.solidify.linear_threshold == sd.linear_threshold
                && sim_cfg.solidify.angular_threshold == sd.angular_threshold
                && sim_cfg.solidify.consecutive_ticks == sd.consecutive_ticks
                && sim_cfg.solidify.max_snap_blocks == sd.max_snap_blocks
                && sim_cfg.solidify.max_snap_angle_deg == sd.max_snap_angle_deg,
            "solidify 取默认"
        );

        let session = c.session_config();
        assert_eq!(session.source, InputSource(1), "会话 source = 1");
        assert_eq!(
            session.scheduler.despawn_after_ticks, 1800,
            "session.scheduler 与 scheduler_config 同源"
        );
        assert_eq!(
            session.scheduler.router.inbox_capacity, c.inbox_capacity,
            "session.scheduler 容量映射同源"
        );
    }

    /// 测试清单 7:dt=0 → scheduler_config panic(配置错误 = fail-stop,细则 4)。
    #[test]
    #[should_panic(expected = "dt")]
    fn zero_dt_panics() {
        let c = Config {
            dt: 0.0,
            ..Config::default()
        };
        let _ = c.scheduler_config();
    }

    /// 测试清单 8(任务书 req001-backlog/02 授权接线):shard_sim_config 的
    /// activation 换算——3s @ 1/60 → 180 拍;半径 u32 → f32;预算直传。
    /// 另以非默认值 + 非 1/60 的 dt 验证换算公式(round)而非巧合。
    #[test]
    fn shard_sim_config_converts_activation() {
        let c = Config::default();
        let a = c.shard_sim_config(shard_id(0, 0)).activation;
        assert_eq!(a.chunk_load_radius, 32.0, "加载半径 f32 化(§4.3)");
        assert_eq!(a.chunk_unload_radius, 48.0, "卸载半径 f32 化");
        assert_eq!(a.awake_load_radius, 24.0, "唤醒半径 f32 化");
        assert_eq!(a.awake_unload_radius, 32.0, "冻结半径 f32 化");
        assert_eq!(a.cooldown_ticks, 180, "3s × 60Hz = 180 拍(§4.3)");
        assert_eq!(a.max_awake_dynamic, 512, "预算直传(§9.4)");
        assert_eq!(a.swap_budget_chunks_per_tick, 2, "换装预算直传(§4.3)");

        // 换算公式:cooldown_secs=3(默认)@ dt=0.05(每拍 0.05s)= 60 拍。
        let c2 = Config {
            dt: 0.05,
            ..Config::default()
        };
        assert_eq!(
            c2.shard_sim_config(shard_id(0, 0))
                .activation
                .cooldown_ticks,
            60,
            "cooldown_secs/dt 四舍五入换算"
        );
    }
}
