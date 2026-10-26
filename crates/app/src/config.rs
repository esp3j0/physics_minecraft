//! 全部参数(蓝图 §6)+ toml 可覆写;唯一默认值来源。
//!
//! 激活参数(REQ-007 §4.15,修订 10):经 `shard_sim_config` 1:1 直传注入
//! 策略机 `physics_sim::ActivationConfig`;调度器那份自同一换算派生(单一来源)。

use std::path::{Path, PathBuf};

use messaging::InputSource;
use physics_sim::{ShardSimConfig, SolidifyConfig};
use serde::{Deserialize, Serialize};
use shard::SchedulerConfig;
use sim_api::session::SessionConfig;
use types::{ShardId, Tick, WorldSeed};

/// 激活参数(REQ-007 §4.15 全表;toml 键 = 参数名,放 `[activation]` 表;
/// 经 `shard_sim_config` 直传策略机)。默认值 = §4.15 表(含 04b 校准回写的
/// `contact_wake_skin_m = 0.075`,修订 10)。
///
/// 旧键别名(`from_toml_str` 预处理,修订 10 总架构师裁定):`[activation]
/// load_radius` / `unload_radius` → ① 与 ② 两档同值;`cooldown_secs` →
/// `cooldown_ticks`(按 dt 换算);顶层 `swap_budget_chunks_per_tick` →
/// ① 与 ② 两档预算。新旧键同给时逐键以新键为准,没给新键的那一档仍取别名值。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ActivationConfig {
    /// ① 数据加载半径 m。
    pub data_load_radius: f32,
    /// ① 数据卸载半径 m。
    pub data_unload_radius: f32,
    /// ② collider 加载半径 m(须 ≤ data_load_radius,校验在 [`Config::from_toml_str`])。
    pub collider_load_radius: f32,
    /// ② collider 卸载半径 m(须 ≤ data_unload_radius)。
    pub collider_unload_radius: f32,
    /// ③ 唤醒加载半径 m。
    pub awake_load_radius: f32,
    /// ③ 冻结卸载半径 m。
    pub awake_unload_radius: f32,
    /// ① 每拍装+卸合计预算。
    pub data_budget_per_tick: usize,
    /// ② 每拍构建预算。
    pub collider_budget_per_tick: usize,
    /// 翻转冷却(拍;①②③ 分档簿记同值)。
    pub cooldown_ticks: u64,
    /// 足迹外扩 m。
    pub terrain_guard_m: f32,
    /// 预载足迹前视拍。
    pub motion_lookahead_ticks: u64,
    /// 前视 chunk 上限。
    pub motion_max_lookahead_chunks: usize,
    /// 瞬态票上限 active_ticks(到期强制落地)。
    pub transient_settle_deadline_ticks: u64,
    /// 常驻运动票上限 active_ticks(到期撤票+兜底冻结)。
    pub actor_motion_max_ticks: u64,
    /// 支撑判定法向阈值。
    pub support_normal_min: f32,
    /// 撞前预测扫掠拍数。
    pub contact_wake_lookahead_ticks: u64,
    /// 接触/贴靠唤醒外扩 m(04b 实测校准,修订 10 回写)。
    pub contact_wake_skin_m: f32,
    /// 种子与传播门槛 kg·m/s。
    pub contact_wake_min_momentum: f32,
    /// 传播每跳动量衰减。
    pub contact_wake_decay: f32,
    /// 传播跳数上限。
    pub contact_wake_max_hops: usize,
    /// 每片每拍接触唤醒实体上限。
    pub contact_wake_max_bodies_per_tick: usize,
    /// 虚空移除深度 m。
    pub void_despawn_depth_m: f32,
}

impl Default for ActivationConfig {
    fn default() -> Self {
        Self {
            data_load_radius: 32.0,
            data_unload_radius: 48.0,
            collider_load_radius: 32.0,
            collider_unload_radius: 48.0,
            awake_load_radius: 24.0,
            awake_unload_radius: 32.0,
            data_budget_per_tick: 2,
            collider_budget_per_tick: 2,
            cooldown_ticks: 180,
            terrain_guard_m: 1.0,
            motion_lookahead_ticks: 30,
            motion_max_lookahead_chunks: 4,
            transient_settle_deadline_ticks: 1200,
            actor_motion_max_ticks: 1200,
            support_normal_min: 0.7,
            contact_wake_lookahead_ticks: 2,
            contact_wake_skin_m: 0.075,
            contact_wake_min_momentum: 1.0,
            contact_wake_decay: 0.5,
            contact_wake_max_hops: 4,
            contact_wake_max_bodies_per_tick: 64,
            void_despawn_depth_m: 16.0,
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
/// 地形破坏配置表 `[destruction]`(REQ-009 §8)。键与默认值同
/// `physics_sim::destruction::DestructionConfig`;**`enabled` 在 REQ-009 实施期间默认 `false`,单元 07 改为
/// `true`(D7)**。校验:`attenuation` 为有限正数;各上限 ≥ 1;`collapse_max_cells ≥ collapse_max_cluster`。
///
/// **总架构师接口桩(REQ-009 单元 05)**:结构与默认值已给出;校验与到 `shard_sim_config` 的映射由实施方实现。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DestructionConfig {
    pub enabled: bool,
    pub attenuation: f32,
    pub max_debris_per_explosion: usize,
    /// REQ-009 修订 10(单元 07):单次爆炸单分片破坏格数上限,默认 2048,≥ 1。
    pub max_cells_per_explosion: usize,
    pub max_detach_per_tick: usize,
    pub max_impact_breaks_per_tick: usize,
    pub collapse_max_cells: usize,
    pub collapse_max_cluster: usize,
}

impl Default for DestructionConfig {
    fn default() -> Self {
        let d = physics_sim::destruction::DestructionConfig::default();
        Self {
            enabled: d.enabled,
            attenuation: d.attenuation,
            max_debris_per_explosion: d.max_debris_per_explosion,
            max_cells_per_explosion: d.max_cells_per_explosion,
            max_detach_per_tick: d.max_detach_per_tick,
            max_impact_breaks_per_tick: d.max_impact_breaks_per_tick,
            collapse_max_cells: d.collapse_max_cells,
            collapse_max_cluster: d.collapse_max_cluster,
        }
    }
}

/// 结构体级 `#[serde(default)]`:toml 文件可只写要覆写的字段。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
    /// 步长(秒),1/60(共识 1)。
    pub dt: f32,
    /// 分片本体外扩边距(格),16(§9.1)。
    pub margin_meters: u32,
    pub activation: ActivationConfig,
    /// 地形破坏(REQ-009 §8)。
    pub destruction: DestructionConfig,
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
    /// checkpoint 保留数(#17;REQ-001 决议 11 修订),10(回放窗口 90 s);
    /// `0` = 不清理。bootstrap 用它打开 checkpoint 库:打开时与每次追加都
    /// 只保留最近 `checkpoint_keep` 个(最新一个永不删,ID 不复用);
    /// 无上限,`0` 合法。
    pub checkpoint_keep: usize,
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
            destruction: DestructionConfig::default(),
            max_awake_dynamic: 512,
            swap_budget_chunks_per_tick: 2,
            area_ticket_max_radius_m: 128.0,
            area_ticket_max_count: 64,
            despawn_after_secs: 30,
            envelope_queue_capacity: 1024,
            inbox_capacity: 256,
            checkpoint_interval_ticks: 600,
            checkpoint_keep: 10,
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
    #[error("配置校验失败:{0}")]
    Validate(String),
    #[error("配置读取失败:{0}")]
    Io(#[from] std::io::Error),
}

/// dt 缺省值(秒)。别名键 `cooldown_secs` 的拍换算需要 dt;config.toml 未给
/// `dt` 时按本值(= [`Config::default`] 的 1/60)换算——给了 `dt` 的新旧键同
/// 给场景以 toml 内的 dt 为准,与后续 `shard_sim_config` 的换算一致。
const DEFAULT_DT: f32 = 1.0 / 60.0;

impl Config {
    /// 从 toml 文本构造(部分覆写:缺省字段取 Default)。空串 = 全默认。
    ///
    /// 旧键别名(REQ-007 §4.15 修订 10;两遍解析):第一遍取别名键与 `dt`,
    /// 把别名值**原位补写**成新键(逐键只在对应新键缺席时补),第二遍正常
    /// 反序列化。`[activation]` 表开了 `deny_unknown_fields`:未知键(含写错
    /// 的新键名)直接报错,不再静默忽略。
    pub fn from_toml_str(text: &str) -> Result<Self, ConfigError> {
        let mut value: toml::Value =
            toml::from_str(text).map_err(|e| ConfigError::Parse(e.to_string()))?;
        let dt = value
            .get("dt")
            .and_then(|v| v.as_float())
            .map(|f| f as f32)
            .unwrap_or(DEFAULT_DT);
        // 顶层别名:swap_budget_chunks_per_tick → ① 与 ② 两档预算(逐键补缺)。
        if let Some(budget) = value.get("swap_budget_chunks_per_tick") {
            let budget = budget.clone();
            let act = value
                .as_table_mut()
                .expect("toml 顶层必为表")
                .entry("activation")
                .or_insert_with(|| toml::Value::Table(toml::Table::new()));
            let act = act.as_table_mut().expect("[activation] 必为表");
            act.entry("data_budget_per_tick").or_insert(budget.clone());
            act.entry("collider_budget_per_tick").or_insert(budget);
        }
        // [activation] 表内别名。
        if let Some(act) = value.get_mut("activation") {
            let act = act.as_table_mut().expect("[activation] 必为表");
            // load_radius / unload_radius → ① 与 ② 两档同值。
            for (old, news) in [
                ("load_radius", ["data_load_radius", "collider_load_radius"]),
                (
                    "unload_radius",
                    ["data_unload_radius", "collider_unload_radius"],
                ),
            ] {
                let Some(v) = act.get(old) else { continue };
                let v = v.clone();
                let n = v
                    .as_integer()
                    .ok_or_else(|| ConfigError::Parse(format!("{old} 须为整数(m),得到:{v}")))?;
                for new in news {
                    act.entry(new).or_insert(toml::Value::Float(n as f64));
                }
                act.remove(old); // 别名键消费掉(deny_unknown_fields 拒绝残留)
            }
            // cooldown_secs → cooldown_ticks(按 dt 换算,四舍五入;与
            // despawn_after_ticks 同式)。
            if let Some(secs) = act.get("cooldown_secs") {
                let secs = secs.as_integer().ok_or_else(|| {
                    ConfigError::Parse(format!("cooldown_secs 须为整数(秒),得到:{secs}"))
                })?;
                let ticks = (secs as f32 / dt).round() as i64;
                act.entry("cooldown_ticks")
                    .or_insert(toml::Value::Integer(ticks));
                act.remove("cooldown_secs");
            }
        }
        let config: Config = value
            .try_into()
            .map_err(|e| ConfigError::Parse(e.to_string()))?;
        config.validate()?;
        Ok(config)
    }

    /// 结构校验(REQ-007 §4.15 修订 10):② 半径 ≤ ① 对应半径、区域票半径
    /// 上限非负;错误信息点名出错的键。
    fn validate(&self) -> Result<(), ConfigError> {
        if self.activation.collider_load_radius > self.activation.data_load_radius {
            return Err(ConfigError::Validate(format!(
                "collider_load_radius ({}) 不得大于 data_load_radius ({})",
                self.activation.collider_load_radius, self.activation.data_load_radius
            )));
        }
        if self.activation.collider_unload_radius > self.activation.data_unload_radius {
            return Err(ConfigError::Validate(format!(
                "collider_unload_radius ({}) 不得大于 data_unload_radius ({})",
                self.activation.collider_unload_radius, self.activation.data_unload_radius
            )));
        }
        if self.area_ticket_max_radius_m < 0.0 {
            return Err(ConfigError::Validate(format!(
                "area_ticket_max_radius_m ({}) 不得为负",
                self.area_ticket_max_radius_m
            )));
        }
        // 地形破坏(REQ-009 §8):`attenuation` 为有限正数;各上限 ≥ 1;
        // `collapse_max_cells ≥ collapse_max_cluster`。
        let d = &self.destruction;
        if !d.attenuation.is_finite() || d.attenuation <= 0.0 {
            return Err(ConfigError::Validate(format!(
                "attenuation ({}) 须为有限正数",
                d.attenuation
            )));
        }
        for (name, cap) in [
            ("max_debris_per_explosion", d.max_debris_per_explosion),
            ("max_detach_per_tick", d.max_detach_per_tick),
            ("max_impact_breaks_per_tick", d.max_impact_breaks_per_tick),
            ("collapse_max_cells", d.collapse_max_cells),
            ("collapse_max_cluster", d.collapse_max_cluster),
        ] {
            if cap < 1 {
                return Err(ConfigError::Validate(format!("{name} ({cap}) 须 ≥ 1")));
            }
        }
        if d.collapse_max_cells < d.collapse_max_cluster {
            return Err(ConfigError::Validate(format!(
                "collapse_max_cells ({}) 不得小于 collapse_max_cluster ({})",
                d.collapse_max_cells, d.collapse_max_cluster
            )));
        }
        Ok(())
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
            // 激活配置注入(REQ-007 §4.15:单一来源——自 shard_sim_config
            // 派生,放行评论风险 3,不另写一份换算)。
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
    /// activation = §4.15 全参数 1:1 直传(修订 10:换算已前移到
    /// `from_toml_str` 的别名预处理,两份配置同源不再二次换算)。
    pub fn shard_sim_config(&self, shard: ShardId) -> ShardSimConfig {
        let a = &self.activation;
        ShardSimConfig {
            shard,
            gravity: types::glam::Vec3::new(0.0, -9.81, 0.0),
            dt: self.dt,
            solidify: SolidifyConfig::default(),
            activation: physics_sim::ActivationConfig {
                data_load_radius: a.data_load_radius,
                data_unload_radius: a.data_unload_radius,
                collider_load_radius: a.collider_load_radius,
                collider_unload_radius: a.collider_unload_radius,
                awake_load_radius: a.awake_load_radius,
                awake_unload_radius: a.awake_unload_radius,
                cooldown_ticks: a.cooldown_ticks,
                max_awake_dynamic: self.max_awake_dynamic,
                data_budget_per_tick: a.data_budget_per_tick,
                collider_budget_per_tick: a.collider_budget_per_tick,
                region_margin_chunks: self.region_margin_chunks(),
                terrain_guard_m: a.terrain_guard_m,
                motion_lookahead_ticks: a.motion_lookahead_ticks,
                motion_max_lookahead_chunks: a.motion_max_lookahead_chunks,
                transient_settle_deadline_ticks: a.transient_settle_deadline_ticks,
                actor_motion_max_ticks: a.actor_motion_max_ticks,
                support_normal_min: a.support_normal_min,
                contact_wake_skin_m: a.contact_wake_skin_m,
                contact_wake_lookahead_ticks: a.contact_wake_lookahead_ticks,
                contact_wake_min_momentum: a.contact_wake_min_momentum,
                contact_wake_decay: a.contact_wake_decay,
                contact_wake_max_hops: a.contact_wake_max_hops,
                contact_wake_max_bodies_per_tick: a.contact_wake_max_bodies_per_tick,
                void_despawn_depth_m: a.void_despawn_depth_m,
            },
            // REQ-009 §8:`[destruction]` 配置表 1:1 直传(单元 05;字段同构)。
            destruction: physics_sim::destruction::DestructionConfig {
                enabled: self.destruction.enabled,
                attenuation: self.destruction.attenuation,
                max_debris_per_explosion: self.destruction.max_debris_per_explosion,
                max_cells_per_explosion: self.destruction.max_cells_per_explosion,
                max_detach_per_tick: self.destruction.max_detach_per_tick,
                max_impact_breaks_per_tick: self.destruction.max_impact_breaks_per_tick,
                collapse_max_cells: self.destruction.collapse_max_cells,
                collapse_max_cluster: self.destruction.collapse_max_cluster,
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

    /// 测试清单 1:逐字段断言蓝图 §6 表 + 决策 9 的默认值(含嵌套 activation;
    /// REQ-007 §4.15 全表 22 项的逐值锚定在 `defaults_anchored_req007`)。
    #[test]
    fn defaults_match_blueprint_table() {
        let c = Config::default();
        assert_eq!(c.dt, 1.0 / 60.0, "dt = 1/60(共识 1)");
        assert_eq!(c.margin_meters, 16, "margin_meters = 16(§9.1)");
        assert_eq!(
            c.activation,
            ActivationConfig::default(),
            "activation = §4.15 全表默认"
        );
        assert_eq!(c.max_awake_dynamic, 512, "max_awake_dynamic = 512(§9.4)");
        assert_eq!(
            c.swap_budget_chunks_per_tick, 2,
            "swap_budget_chunks_per_tick = 2(§4.3;别名键,→ ①② 两档预算)"
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
                data_load_radius: 64.0,
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

    /// 任务书测试 `app::config::defaults_anchored`(REQ-007 任务 06c,T22):
    /// §4.15 全表 22 个默认值逐一显式断言(不靠 Default 派生),并核两处
    /// 派生配置(分片/调度器)同值;区域票上限默认 128/64。
    #[test]
    fn defaults_anchored() {
        let c = Config::default();
        let a = &c.activation;
        assert_eq!(a.data_load_radius, 32.0, "§4.15:① 载 32");
        assert_eq!(a.data_unload_radius, 48.0, "§4.15:① 卸 48");
        assert_eq!(a.collider_load_radius, 32.0, "§4.15:② 载 32");
        assert_eq!(a.collider_unload_radius, 48.0, "§4.15:② 卸 48");
        assert_eq!(a.awake_load_radius, 24.0, "§4.15:③ 载 24");
        assert_eq!(a.awake_unload_radius, 32.0, "§4.15:③ 卸 32");
        assert_eq!(a.data_budget_per_tick, 2, "§4.15:① 预算 2");
        assert_eq!(a.collider_budget_per_tick, 2, "§4.15:② 预算 2");
        assert_eq!(a.cooldown_ticks, 180, "§4.15:冷却 180 拍");
        assert_eq!(a.terrain_guard_m, 1.0, "§4.15:足迹外扩 1 m");
        assert_eq!(a.motion_lookahead_ticks, 30, "§4.15:前视 30 拍");
        assert_eq!(a.motion_max_lookahead_chunks, 4, "§4.15:前视上限 4 chunk");
        assert_eq!(
            a.transient_settle_deadline_ticks, 1200,
            "§4.15:瞬态票上限 1200"
        );
        assert_eq!(a.actor_motion_max_ticks, 1200, "§4.15:常驻票上限 1200");
        assert_eq!(a.support_normal_min, 0.7, "§4.15:支撑法向阈值 0.7");
        assert_eq!(a.contact_wake_lookahead_ticks, 2, "§4.15:扫掠 2 拍");
        assert_eq!(
            a.contact_wake_skin_m, 0.075,
            "§4.15:skin 0.075(04b 校准,修订 10 回写)"
        );
        assert_eq!(a.contact_wake_min_momentum, 1.0, "§4.15:动量门槛 1.0");
        assert_eq!(a.contact_wake_decay, 0.5, "§4.15:衰减 0.5");
        assert_eq!(a.contact_wake_max_hops, 4, "§4.15:跳数上限 4");
        assert_eq!(
            a.contact_wake_max_bodies_per_tick, 64,
            "§4.15:每拍唤醒上限 64"
        );
        assert_eq!(a.void_despawn_depth_m, 16.0, "§4.15:虚空深度 16 m");
        assert_eq!(c.area_ticket_caps(), (128.0, 64), "区域票上限默认");

        // 两处派生配置同源:scheduler_config() 的 activation 取自
        // shard_sim_config()(放行评论风险 3:单一来源,不另写换算)。
        let shard = c.shard_sim_config(shard_id(0, 0)).activation;
        let sched = c.session_config().scheduler.activation;
        assert_eq!(shard, sched, "分片与调度器激活配置同值");
    }

    /// 任务书测试 `app::config::all_req007_params_injectable`(REQ-007 06c,
    /// T22):22 个激活键 + 3 个顶层键全设非默认值,经 shard_sim_config 直传
    /// 生效;to_toml_string 往返不丢新键。区域票上限的 bootstrap 装配面由
    /// 验收用例 acc_06c_all_params_injectable 覆盖。
    #[test]
    fn all_req007_params_injectable() {
        let c = Config::from_toml_str(
            "max_awake_dynamic = 300\n\
             area_ticket_max_radius_m = 96.0\n\
             area_ticket_max_count = 12\n\
             [activation]\n\
             data_load_radius = 40\n\
             data_unload_radius = 56\n\
             collider_load_radius = 28\n\
             collider_unload_radius = 36\n\
             awake_load_radius = 20\n\
             awake_unload_radius = 26\n\
             data_budget_per_tick = 5\n\
             collider_budget_per_tick = 3\n\
             cooldown_ticks = 90\n\
             terrain_guard_m = 1.5\n\
             motion_lookahead_ticks = 20\n\
             motion_max_lookahead_chunks = 3\n\
             transient_settle_deadline_ticks = 900\n\
             actor_motion_max_ticks = 700\n\
             support_normal_min = 0.8\n\
             contact_wake_lookahead_ticks = 3\n\
             contact_wake_skin_m = 0.06\n\
             contact_wake_min_momentum = 1.5\n\
             contact_wake_decay = 0.4\n\
             contact_wake_max_hops = 3\n\
             contact_wake_max_bodies_per_tick = 32\n\
             void_despawn_depth_m = 24.0\n",
        )
        .expect("全参数配置合法");
        let a = c.shard_sim_config(shard_id(0, 0)).activation;
        assert_eq!((a.data_load_radius, a.data_unload_radius), (40.0, 56.0));
        assert_eq!(
            (a.collider_load_radius, a.collider_unload_radius),
            (28.0, 36.0)
        );
        assert_eq!((a.awake_load_radius, a.awake_unload_radius), (20.0, 26.0));
        assert_eq!((a.data_budget_per_tick, a.collider_budget_per_tick), (5, 3));
        assert_eq!(a.cooldown_ticks, 90);
        assert_eq!(a.terrain_guard_m, 1.5);
        assert_eq!(a.motion_lookahead_ticks, 20);
        assert_eq!(a.motion_max_lookahead_chunks, 3);
        assert_eq!(a.transient_settle_deadline_ticks, 900);
        assert_eq!(a.actor_motion_max_ticks, 700);
        assert_eq!(a.support_normal_min, 0.8);
        assert_eq!(a.contact_wake_lookahead_ticks, 3);
        assert_eq!(a.contact_wake_skin_m, 0.06);
        assert_eq!(a.contact_wake_min_momentum, 1.5);
        assert_eq!(a.contact_wake_decay, 0.4);
        assert_eq!(a.contact_wake_max_hops, 3);
        assert_eq!(a.contact_wake_max_bodies_per_tick, 32);
        assert_eq!(a.void_despawn_depth_m, 24.0);
        assert_eq!(a.max_awake_dynamic, 300);
        assert_eq!(c.area_ticket_caps(), (96.0, 12), "区域票上限注入");

        // 调度器那份同值(单一来源派生)。
        assert_eq!(c.session_config().scheduler.activation, a, "调度器配置同值");

        // 往返:to_toml_string → from_toml_str 相等(新键不丢)。
        let again = Config::from_toml_str(&c.to_toml_string()).expect("往返后仍合法");
        assert_eq!(again, c, "全参数往返无损");
    }

    /// 任务书测试 `app::config::legacy_keys_alias_mapping`(REQ-007 06c):
    /// 旧键 → 两档同值、秒换算拍、顶层预算两档;新旧同给逐键新键优先,
    /// 没给新键的那一档仍取别名值。
    #[test]
    fn legacy_keys_alias_mapping() {
        let legacy = Config::from_toml_str(
            "swap_budget_chunks_per_tick = 6\n\
             [activation]\n\
             load_radius = 40\n\
             unload_radius = 60\n\
             cooldown_secs = 2\n",
        )
        .expect("旧键配置合法");
        let a = legacy.shard_sim_config(shard_id(0, 0)).activation;
        assert_eq!(
            (a.data_load_radius, a.collider_load_radius),
            (40.0, 40.0),
            "load_radius → ① 与 ② 同值"
        );
        assert_eq!(
            (a.data_unload_radius, a.collider_unload_radius),
            (60.0, 60.0),
            "unload_radius → ① 与 ② 同值"
        );
        assert_eq!(a.cooldown_ticks, 120, "cooldown_secs = 2 → 120 拍(1/60)");
        assert_eq!(
            (a.data_budget_per_tick, a.collider_budget_per_tick),
            (6, 6),
            "swap_budget_chunks_per_tick → ① 与 ② 两档预算"
        );

        let mixed = Config::from_toml_str(
            "swap_budget_chunks_per_tick = 6\n\
             [activation]\n\
             load_radius = 40\n\
             data_load_radius = 44\n\
             cooldown_secs = 2\n\
             cooldown_ticks = 100\n\
             data_budget_per_tick = 4\n",
        )
        .expect("新旧混合配置合法");
        let m = mixed.shard_sim_config(shard_id(0, 0)).activation;
        assert_eq!(m.data_load_radius, 44.0, "新键 data_load_radius 优先");
        assert_eq!(
            m.collider_load_radius, 40.0,
            "没给新键的 ② 仍取别名 load_radius"
        );
        assert_eq!(
            m.cooldown_ticks, 100,
            "新键 cooldown_ticks 优先于 cooldown_secs"
        );
        assert_eq!(m.data_budget_per_tick, 4, "新键 data_budget_per_tick 优先");
        assert_eq!(m.collider_budget_per_tick, 6, "没给新键的 ② 预算仍取别名");

        // dt 影响 cooldown_secs 换算:dt = 0.05(20 Hz)时 2 s → 40 拍。
        let slow = Config::from_toml_str(
            "dt = 0.05\n\
             [activation]\n\
             cooldown_secs = 2\n",
        )
        .expect("dt + 旧键配置合法");
        assert_eq!(
            slow.shard_sim_config(shard_id(0, 0))
                .activation
                .cooldown_ticks,
            40,
            "cooldown_secs 按 toml 内 dt 换算"
        );
    }

    /// 任务书测试 `app::config::validation_rules`(REQ-007 06c,T22):
    /// ② 半径 > ① 对应半径、区域票半径上限为负 → from_toml_str/load 报错
    /// 且错误信息点名出错的键;边界(② = ①)合法。
    #[test]
    fn validation_rules() {
        for (toml, key) in [
            (
                "[activation]\ncollider_load_radius = 40\n",
                "collider_load_radius",
            ),
            (
                "[activation]\ncollider_unload_radius = 60\n",
                "collider_unload_radius",
            ),
            (
                "area_ticket_max_radius_m = -1.0\n",
                "area_ticket_max_radius_m",
            ),
        ] {
            match Config::from_toml_str(toml) {
                Ok(_) => panic!("非法配置应被拒绝:{toml}"),
                Err(e) => assert!(e.to_string().contains(key), "错误信息应点名 {key}:{e}"),
            }
        }
        // ② 半径等于 ① 半径合法(≤)。
        assert!(
            Config::from_toml_str("[activation]\ncollider_load_radius = 32\n").is_ok(),
            "② 半径等于 ① 半径合法"
        );
        // 经 load(文件路径)同样校验:写临时文件(系统临时目录,不入仓库)。
        let path = std::env::temp_dir().join("req007-06c-validation.toml");
        std::fs::write(&path, "[activation]\ncollider_load_radius = 40\n").expect("写临时配置文件");
        let err = Config::load(Some(&path)).expect_err("load 同样校验");
        assert!(
            err.to_string().contains("collider_load_radius"),
            "load 的错误信息点名键:{err}"
        );
        let _ = std::fs::remove_file(&path);
        // 校验前错误仍按解析错误上抛(未知键,deny_unknown_fields)。
        let unknown = Config::from_toml_str("[activation]\nno_such_key = 1\n");
        assert!(unknown.is_err(), "未知键必须报错(不静默忽略)");
    }

    /// 既有面(REQ-007 任务 01 引入的校验注释,06c 后由 `validation_rules`
    /// 承担主断言):默认值恒合法(①② 同值默认)。
    #[test]
    fn collider_radius_validation() {
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
        assert!(
            Config::default()
                .shard_sim_config(shard_id(0, 0))
                .activation
                .collider_load_radius
                > 0.0
        );
    }

    /// REQ-009 §8(单元 05):`[destruction]` 默认值与核内同值(实施期
    /// `enabled = false`);部分键其余取默认(结构体级 serde(default) 与表级
    /// deny_unknown_fields 并存,风险 5);覆写逐字段映射进 `shard_sim_config`
    /// (替换写死的核内默认);覆写实例 toml 往返无损。
    ///
    /// 变红:表没接进 shard_sim_config(分片仍用核内默认);部分键解析失败。
    #[test]
    fn destruction_config_partial_keys_and_mapping() {
        // 默认 = 核内默认,逐字段锚定(enabled 实施期 false,单元 07 改)。
        let d = Config::default().destruction;
        let core = physics_sim::destruction::DestructionConfig::default();
        assert!(!d.enabled, "实施期间默认关闭");
        assert_eq!(d.attenuation, core.attenuation);
        assert_eq!(d.max_debris_per_explosion, core.max_debris_per_explosion);
        assert_eq!(d.max_detach_per_tick, core.max_detach_per_tick);
        assert_eq!(
            d.max_impact_breaks_per_tick,
            core.max_impact_breaks_per_tick
        );
        assert_eq!(d.collapse_max_cells, core.collapse_max_cells);
        assert_eq!(d.collapse_max_cluster, core.collapse_max_cluster);
        assert_eq!(
            Config::default()
                .shard_sim_config(shard_id(0, 0))
                .destruction,
            core,
            "默认映射 = 核内默认"
        );

        // 部分键:给了的生效,没给的取默认。
        let partial =
            Config::from_toml_str("[destruction]\nattenuation = 2.0\n").expect("部分键合法");
        assert_eq!(partial.destruction.attenuation, 2.0, "覆写生效");
        assert!(!partial.destruction.enabled, "未给的键取默认");
        assert_eq!(
            partial.destruction.max_detach_per_tick, core.max_detach_per_tick,
            "未给的键取默认"
        );

        // 覆写:7 键全给,逐字段映射进分片配置。
        let c = Config::from_toml_str(
            "[destruction]\nenabled = true\nattenuation = 0.5\nmax_debris_per_explosion = 10\n\
             max_detach_per_tick = 20\nmax_impact_breaks_per_tick = 3\ncollapse_max_cells = 900\n\
             collapse_max_cluster = 300\n",
        )
        .expect("合法");
        let m = c.shard_sim_config(shard_id(0, 0)).destruction;
        assert_eq!(
            (
                m.enabled,
                m.attenuation,
                m.max_debris_per_explosion,
                m.max_detach_per_tick,
                m.max_impact_breaks_per_tick,
                m.collapse_max_cells,
                m.collapse_max_cluster
            ),
            (true, 0.5, 10, 20, 3, 900, 300),
            "覆写逐字段直传(替换写死的核内默认)"
        );

        // 往返:覆写实例无损(新表真实走序列化)。
        assert_eq!(
            Config::from_toml_str(&c.to_toml_string()).expect("往返后仍合法"),
            c,
            "[destruction] 往返不丢字段"
        );
    }

    /// REQ-009 §8 校验:`attenuation` 非正或非有限、各上限为 0、
    /// `collapse_max_cells < collapse_max_cluster` 都被拒绝且错误信息点名键;
    /// 边界(上限恰 1、cells == cluster)合法。未知键经表级
    /// deny_unknown_fields 在解析期报错。
    ///
    /// 变红:漏掉某条校验(对应键不再被点名)。
    #[test]
    fn destruction_config_validation_rules() {
        for (toml, key) in [
            ("[destruction]\nfoo = 1\n", "foo"),
            ("[destruction]\nattenuation = 0.0\n", "attenuation"),
            ("[destruction]\nattenuation = -1.0\n", "attenuation"),
            ("[destruction]\nattenuation = inf\n", "attenuation"),
            (
                "[destruction]\nmax_debris_per_explosion = 0\n",
                "max_debris_per_explosion",
            ),
            (
                "[destruction]\nmax_detach_per_tick = 0\n",
                "max_detach_per_tick",
            ),
            (
                "[destruction]\nmax_impact_breaks_per_tick = 0\n",
                "max_impact_breaks_per_tick",
            ),
            (
                "[destruction]\ncollapse_max_cells = 0\n",
                "collapse_max_cells",
            ),
            (
                "[destruction]\ncollapse_max_cluster = 0\n",
                "collapse_max_cluster",
            ),
            (
                "[destruction]\ncollapse_max_cells = 100\n",
                "collapse_max_cells",
            ),
        ] {
            match Config::from_toml_str(toml) {
                Ok(_) => panic!("非法配置应被拒绝:{toml}"),
                Err(e) => assert!(e.to_string().contains(key), "错误信息应点名 {key}:{e}"),
            }
        }
        // 边界合法:各上限恰 1;cells == cluster。
        assert!(
            Config::from_toml_str(
                "[destruction]\nmax_debris_per_explosion = 1\nmax_detach_per_tick = 1\n\
                 max_impact_breaks_per_tick = 1\ncollapse_max_cells = 8\ncollapse_max_cluster = 8\n",
            )
            .is_ok(),
            "边界值合法"
        );
    }
}
