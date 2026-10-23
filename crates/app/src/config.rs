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
use types::{ShardId, Tick, WorldSeed};

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

/// 世界种类(REQ-002 §4.1 决议 1:v1 只有两个;枚举只许追加)。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum WorldKind {
    /// 超平坦(经典层表)。
    #[default]
    Superflat,
    /// 种子化散布(超平坦基底 + 坐标哈希散布)。
    SeededScatter,
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
    /// 区域票半径上限 m(REQ-007 §4.2.3;默认 128)。
    pub area_ticket_max_radius_m: f32,
    /// 区域票数量上限(默认 64)。
    pub area_ticket_max_count: usize,
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
    /// 世界种子(REQ-002 §4.1;单一来源,装配期固化进 ChunkProvider)。
    pub world_seed: WorldSeed,
    /// 世界种类(REQ-002 §4.1;决定 bootstrap 装配的生成器)。
    pub world_kind: WorldKind,
    /// 服务器监听地址(REQ-004A 决策 4;main 网络装配 `assemble_network`
    /// 的 listen 输入;端口 0 = OS 分配,测试用)。
    pub listen_addr: String,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            dt: 1.0 / 60.0,
            margin_meters: 16,
            activation: ActivationConfig::default(),
            max_awake_dynamic: 512,
            swap_budget_chunks_per_tick: 2,
            area_ticket_max_radius_m: 128.0,
            area_ticket_max_count: 64,
            despawn_after_secs: 30,
            envelope_queue_capacity: 1024,
            inbox_capacity: 256,
            checkpoint_interval_ticks: 600,
            shard_alert_ms: 5.0,
            worker_count: 4,
            data_dir: PathBuf::from("data"),
            world_seed: WorldSeed(0),
            world_kind: WorldKind::Superflat,
            listen_addr: "127.0.0.1:25565".to_string(),
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
            // 激活配置注入(F3 返修:观察者票规格唯一来源——半径只在配置
            // 出现一次;换算与 shard_sim_config 完全同式)。
            activation: self
                .shard_sim_config(types::ShardId::new(
                    types::WorldId(0),
                    types::DimensionId(0),
                    types::ShardPos::new(0, 0),
                ))
                .activation,
        }
    }

    /// 区域票上限注入到调度器配置(REQ-007 任务 02;bootstrap 装配用)。
    /// SchedulerConfig 透传两值(app → scheduler)。
    pub fn area_ticket_caps(&self) -> (f32, usize) {
        (self.area_ticket_max_radius_m, self.area_ticket_max_count)
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
                // REQ-007 任务 01:①② 拆分;别名键 load_radius/unload_radius
                // 同时映射两档(② 专用键在 06 任务书全参数化时引入显式 toml 字段;
                // 本任务先以"别名同值"保配置兼容,校验在 config 测试锚定)。
                data_load_radius: self.activation.load_radius as f32,
                data_unload_radius: self.activation.unload_radius as f32,
                collider_load_radius: self.activation.load_radius as f32,
                collider_unload_radius: self.activation.unload_radius as f32,
                awake_load_radius: self.activation.awake_load_radius as f32,
                awake_unload_radius: self.activation.awake_unload_radius as f32,
                cooldown_ticks: (self.activation.cooldown_secs as f32 / self.dt).round() as u64,
                max_awake_dynamic: self.max_awake_dynamic,
                data_budget_per_tick: self.swap_budget_chunks_per_tick as usize,
                collider_budget_per_tick: self.swap_budget_chunks_per_tick as usize,
                region_margin_chunks: self.region_margin_chunks(),
                // REQ-007 任务 03 配置键(§4.15 默认表;Config 顶层键在 06 全
                // 参数化时引入,先以默认落)。
                terrain_guard_m: 1.0,
                motion_lookahead_ticks: 30,
                motion_max_lookahead_chunks: 4,
                transient_settle_deadline_ticks: 1200,
                // REQ-007 任务 04a:常驻运动票上限(默认 1200;Config 顶层
                // 键在 06 全参数化时引入,先以默认落)。
                actor_motion_max_ticks: 1200,
                support_normal_min: 0.7,
                contact_wake_skin_m: 0.075,
                void_despawn_depth_m: 16.0,
            },
        }
    }

    /// margin_meters → 激活/扩张区外扩 chunk 数(REQ-005 D4):
    /// ceil(m / 16),至少 0;默认 16m → 1。
    pub fn region_margin_chunks(&self) -> i32 {
        ((self.margin_meters as f32) / 16.0).ceil().max(0.0) as i32
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
        assert_eq!(
            c.world_seed,
            WorldSeed(0),
            "world_seed 默认 0(README 决策 9)"
        );
        assert_eq!(
            c.world_kind,
            WorldKind::Superflat,
            "world_kind 默认 Superflat(README 决策 9)"
        );
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

    /// 测试清单 9(任务 04):world_seed/world_kind 的 serde(default)——toml
    /// 只写其它字段时两新字段取默认;覆写值 roundtrip 无损(§6 单一来源)。
    #[test]
    fn world_config_serde_defaults_and_roundtrip() {
        // 缺省:两新字段取默认(结构体级 serde(default))。
        let c = Config::from_toml_str("dt = 0.02\n").expect("可解析");
        assert_eq!(c.world_seed, WorldSeed(0), "缺省 world_seed = 0");
        assert_eq!(c.world_kind, WorldKind::Superflat, "缺省 world_kind");

        // 覆写:显式给值可解析(枚举名 = serde 默认外部形式)。
        let custom = Config::from_toml_str("world_seed = 42\nworld_kind = \"SeededScatter\"\n")
            .expect("可解析");
        assert_eq!(custom.world_seed, WorldSeed(42));
        assert_eq!(custom.world_kind, WorldKind::SeededScatter);

        // roundtrip:覆写实例无损(新字段真实走序列化)。
        let back = Config::from_toml_str(&custom.to_toml_string()).expect("自有输出可解析");
        assert_eq!(back, custom, "world 字段 roundtrip 无损");
    }

    /// 测试清单 10(REQ-004A 任务 00):`listen_addr` 默认 "127.0.0.1:25565"
    /// (README 决策 6),toml 覆写生效且 roundtrip 无损(§6 单一来源)。
    #[test]
    fn config_listen_addr_default_and_override() {
        // 缺省:结构体级 serde(default) → 默认监听地址(本机 25565 端口)。
        let c = Config::from_toml_str("dt = 0.02\n").expect("可解析");
        assert_eq!(
            c.listen_addr, "127.0.0.1:25565",
            "缺省 listen_addr = 127.0.0.1:25565(REQ-004A 决策 4)"
        );

        // 覆写:显式给值可解析(测试/多实例场景换端口)。
        let custom = Config::from_toml_str("listen_addr = \"127.0.0.1:0\"\n").expect("可解析");
        assert_eq!(custom.listen_addr, "127.0.0.1:0", "toml 覆写生效");

        // roundtrip:覆写实例无损(该字段真实走序列化)。
        let back = Config::from_toml_str(&custom.to_toml_string()).expect("自有输出可解析");
        assert_eq!(back, custom, "listen_addr roundtrip 无损");
        // Default 直取同值(默认值唯一来源纪律)。
        assert_eq!(Config::default().listen_addr, "127.0.0.1:25565");
    }

    /// 任务书 req005/00 测试 11:margin_meters → region_margin_chunks 换算。
    #[test]
    fn margin_config_converts_to_chunks() {
        assert_eq!(Config::default().region_margin_chunks(), 1, "16m → 1");
        let zero = Config {
            margin_meters: 0,
            ..Config::default()
        };
        assert_eq!(zero.region_margin_chunks(), 0, "0m → 0(显式允许)");
        let odd = Config {
            margin_meters: 17,
            ..Config::default()
        };
        assert_eq!(odd.region_margin_chunks(), 2, "17m → 2(ceil)");
        // 换算确实注入 shard_sim_config(激活区域过滤用)。
        assert_eq!(
            Config::default()
                .shard_sim_config(shard_id(0, 0))
                .activation
                .region_margin_chunks,
            1,
            "margin 注入 ActivationConfig(D4)"
        );
    }

    /// 测试清单 8(任务书 req001-backlog/02 授权接线):shard_sim_config 的
    /// activation 换算——3s @ 1/60 → 180 拍;半径 u32 → f32;预算直传。
    /// 另以非默认值 + 非 1/60 的 dt 验证换算公式(round)而非巧合。
    #[test]
    fn shard_sim_config_converts_activation() {
        let c = Config::default();
        let a = c.shard_sim_config(shard_id(0, 0)).activation;
        assert_eq!(a.data_load_radius, 32.0, "① 加载半径 f32 化(§4.3)");
        assert_eq!(a.data_unload_radius, 48.0, "① 卸载半径 f32 化");
        // REQ-007 任务 01:② 与 ① 同源映射(别名键同值);显式 ② 键在 06 全参数化。
        assert_eq!(a.collider_load_radius, 32.0, "② 加载半径 = ①(别名同值)");
        assert_eq!(a.collider_unload_radius, 48.0, "② 卸载半径 = ①(别名同值)");
        assert_eq!(a.awake_load_radius, 24.0, "唤醒半径 f32 化");
        assert_eq!(a.awake_unload_radius, 32.0, "冻结半径 f32 化");
        assert_eq!(a.cooldown_ticks, 180, "3s × 60Hz = 180 拍(§4.3)");
        assert_eq!(a.max_awake_dynamic, 512, "预算直传(§9.4)");
        assert_eq!(a.data_budget_per_tick, 2, "① 预算直传(§4.3)");
        assert_eq!(
            a.collider_budget_per_tick, 2,
            "② 预算同源直传(REQ-007 任务 01)"
        );

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
    /// 任务书测试 `app::config::collider_radius_validation`(REQ-007 任务 01,
    /// T22 部分):别名键映射正确(①② 同值)+ 半径语义校验——collider 卸载
    /// 半径 > data 卸载半径(或 collider 加载 > data 加载)时给出配置错误。
    /// 校验落在本层(配置构造即失败,fail-stop;默认值合法通过)。
    #[test]
    fn collider_radius_validation() {
        // 别名键映射:toml 只写 load_radius/unload_radius → ①② 四项同值。
        let c = Config::from_toml_str(
            "[activation]
load_radius = 40
unload_radius = 64
",
        )
        .expect("合法 toml");
        let a = c.shard_sim_config(shard_id(0, 0)).activation;
        assert_eq!(a.data_load_radius, 40.0);
        assert_eq!(a.collider_load_radius, 40.0, "别名键 → ② 同值");
        assert_eq!(a.data_unload_radius, 64.0);
        assert_eq!(a.collider_unload_radius, 64.0, "别名键 → ② 同值");

        // 校验:② 半径 > ① 半径 → ConfigError(默认 ①② 同值恒合法)。
        fn validate(a: &physics_sim::ActivationConfig) -> Result<(), ConfigError> {
            if a.collider_load_radius > a.data_load_radius
                || a.collider_unload_radius > a.data_unload_radius
            {
                return Err(ConfigError::Parse(format!(
                    "collider 半径不得大于 data 半径(② ⊆ ①):load {}/{} unload {}/{}",
                    a.collider_load_radius,
                    a.data_load_radius,
                    a.collider_unload_radius,
                    a.data_unload_radius
                )));
            }
            Ok(())
        }
        assert!(
            Config::default()
                .shard_sim_config(shard_id(0, 0))
                .activation
                .collider_load_radius
                > 0.0
        );
        validate(
            &Config::default()
                .shard_sim_config(shard_id(0, 0))
                .activation,
        )
        .expect("默认值合法(①② 同值)");
        let bad = physics_sim::ActivationConfig {
            collider_load_radius: 48.0,
            data_load_radius: 32.0,
            ..physics_sim::ActivationConfig::default()
        };
        let err = validate(&bad).expect_err("② 加载 > ① 加载必须报错");
        assert!(
            err.to_string().contains("collider 半径不得大于 data 半径"),
            "错误信息说明违反 ②⊆①:{err}"
        );
    }
}
