//! REQ-012 E01a:训练环境(单环境 Rust API,AC-E1)。
//!
//! 两条结构保证(任务书设计要点):
//! 1. 环境建在 [`sim_api::session::Session`] 上,和服务器驱动层用同一套入口
//!    (`send_agent_joint_targets_to` + `tick`),动作同样进命令日志——"同一份
//!    物理"是结构保证的。
//! 2. 观测拼装和"动作 → 关节目标"复用 [`policy::PolicyCodec`](policy 库的同一段
//!    代码),训练时看到的观测和部署时逐位同一个函数算出来。
//!
//! 布景照 `crates/app/tests/req012_b02a.rs` 的 `run_quad`(整分片 8×8 chunk
//! 平地、主体生成分片中心、同 `SessionConfig`),写成 crate 内函数,不依赖 app
//! 测试代码。并行多实例、地形随机化、PyO3 都是后面的单元。

pub mod vec_env;
pub use vec_env::{VecEnv, VecStep, episode_seed};

#[cfg(feature = "python")]
mod py;

use messaging::InputSource;
use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
use policy::{AgentSnapshot, PolicyCodec, PolicyError, PolicyManifest};
use shard::SchedulerConfig;
use sim_api::log::CommandLog;
use sim_api::replay::replay_world_with_bodies;
use sim_api::session::{Session, SessionConfig};
use sim_api::snapshot::WorldSnapshot;
use types::glam::Vec3;
use types::{BlockId, ChunkPos, DimensionId, EntityId, EntityState, ShardId, ShardPos, WorldId};
use voxel::{Chunk, SimpleRegistry};

const STONE: BlockId = BlockId(1);
/// 地面顶面(world y;chunk y=64 的石头顶面 = 64 + WORLD_Y_MIN + 1 = 1.0)。
const GROUND_TOP_Y: f32 = 1.0;
/// 出生点(分片中心,顶面 + 0.45)。
const SPAWN: Vec3 = Vec3::new(64.0, 1.45, 64.0);
/// 铺满整个分片:8×8 chunk = 128×128 m。
const GROUND_CHUNKS: i32 = 8;

/// 环境配置(任务书 §2.1)。
#[derive(Clone)]
pub struct EnvConfig {
    /// 身体定义文本(如 `assets/bodies/quad.body.toml` 的内容)。
    pub body_toml: String,
    /// 观测布局、动作映射、控制周期;`normalization` 必须是 `None`。
    pub manifest: PolicyManifest,
    /// 一回合多少个控制步(≥ 1)。
    pub episode_controls: u32,
    /// 生成后先站多少拍再拍初态快照(默认 120)。
    pub settle_ticks: u32,
    /// 机身 up·y 低于它 = 摔倒(默认 0.5)。
    pub fall_up_y_min: f32,
    /// 每回合采样 intent 的 [lo, hi];lo == hi = 固定值。
    pub intent_range: [[f32; 2]; 3],
}

/// 一步的输出(任务书 §2.1)。
pub struct StepOutput {
    /// 长度 `codec.obs_dim()`。
    pub obs: Vec<f32>,
    pub reward: RewardInputs,
    /// 摔倒或主体被移除。
    pub terminated: bool,
    /// 到了 `episode_controls`。
    pub truncated: bool,
}

/// 奖励原料(任务书 §2.1;奖励函数本身在训练端,本单只给原料)。
pub struct RewardInputs {
    /// 机身系。
    pub base_linvel_body: [f32; 3],
    /// 机身系。
    pub base_angvel_body: [f32; 3],
    /// 机身系里的 (0, −1, 0)。
    pub projected_gravity: [f32; 3],
    /// 根连杆 y − 地面顶面 y。
    pub root_height: f32,
    pub up_y: f32,
    /// 原始关节角(不减 default)。
    pub joint_pos: Vec<f32>,
    pub joint_vel: Vec<f32>,
    /// 本步的 raw(裁剪后)。
    pub action: Vec<f32>,
    /// 上一步的 raw(回合第一步为全 0)。
    pub prev_action: Vec<f32>,
    pub targets_clamped: u32,
    pub intent: [f32; 3],
}

/// 奖励原料各键的固定顺序与宽度(E02b-1;`reward_columns` 与 `spec()` 共用)。
/// dof 为自由度数;`joint_pos`/`joint_vel`/`action`/`prev_action` 宽 dof,其余定宽。
pub fn reward_key_widths(dof: usize) -> Vec<(&'static str, usize)> {
    vec![
        ("base_linvel_body", 3),
        ("base_angvel_body", 3),
        ("projected_gravity", 3),
        ("root_height", 1),
        ("up_y", 1),
        ("joint_pos", dof),
        ("joint_vel", dof),
        ("action", dof),
        ("prev_action", dof),
        ("targets_clamped", 1),
        ("intent", 3),
    ]
}

/// 把一批 [`RewardInputs`] 摊平成各键的列数组(E02b-1;PyO3 `step` 与契约检查共用)。
///
/// 返回固定顺序的 `(键, 宽度 k, 数据)`:数据按环境下标拼接,长度 `n_envs × k`。
/// 键序与宽度见 [`reward_key_widths`]。dof 取自首个奖励的 `joint_pos` 长度;
/// 空切片返回各键空列(dof = 0)。
pub fn reward_columns(rewards: &[RewardInputs]) -> Vec<(&'static str, usize, Vec<f32>)> {
    let dof = rewards.first().map(|r| r.joint_pos.len()).unwrap_or(0);
    let specs = reward_key_widths(dof);
    let mut cols: Vec<Vec<f32>> = specs
        .iter()
        .map(|(_, k)| Vec::with_capacity(rewards.len() * k))
        .collect();
    for r in rewards {
        cols[0].extend_from_slice(&r.base_linvel_body);
        cols[1].extend_from_slice(&r.base_angvel_body);
        cols[2].extend_from_slice(&r.projected_gravity);
        cols[3].push(r.root_height);
        cols[4].push(r.up_y);
        cols[5].extend_from_slice(&r.joint_pos);
        cols[6].extend_from_slice(&r.joint_vel);
        cols[7].extend_from_slice(&r.action);
        cols[8].extend_from_slice(&r.prev_action);
        cols[9].push(r.targets_clamped as f32);
        cols[10].extend_from_slice(&r.intent);
    }
    specs
        .into_iter()
        .zip(cols)
        .map(|((name, k), col)| (name, k, col))
        .collect()
}

/// 环境错误(任务书 §2.1)。
#[derive(Debug, thiserror::Error)]
pub enum EnvError {
    #[error("身体定义错误:{0}")]
    Body(String),
    #[error("策略错误:{0}")]
    Policy(#[from] PolicyError),
    #[error("配置错误:{0}")]
    Config(String),
    #[error("动作维度不符:得到 {got},应为 {want}")]
    ActionDim { got: usize, want: usize },
    #[error("动作含非有限数")]
    NonFiniteAction,
    #[error("需要先 reset(上一步已结束或尚未 reset)")]
    NeedsReset,
    #[error("会话/生成错误:{0}")]
    Spawn(String),
}

/// 单个训练环境(任务书 §2.2)。
pub struct AgentEnv {
    cfg: EnvConfig,
    codec: PolicyCodec,
    bodies: BodyRegistry,
    entity: EntityId,
    initial: WorldSnapshot,
    session: Session,
    log: CommandLog,
    intent: [f32; 3],
    prev_action: Vec<f32>,
    steps: u32,
    finished: bool,
    reset_done: bool,
}

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn session_cfg() -> SessionConfig {
    // inbox_capacity 512:与 run_quad 同口径(只影响路由反压,不影响物理)。
    SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            router: shard::router::RouterConfig {
                inbox_capacity: 512,
                ..shard::router::RouterConfig::default()
            },
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    }
}

/// 整分片 8×8 chunk 平地(层 chunk y=64 全石,顶面 world y=1.0)。
fn load_flat_ground(session: &mut Session) {
    for cx in 0..GROUND_CHUNKS {
        for cz in 0..GROUND_CHUNKS {
            let mut chunk = Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    chunk.set(x, 64, z, STONE);
                }
            }
            session.apply_chunk_load(shard_id(0, 0), ChunkPos::new(cx, cz), chunk);
        }
    }
}

fn build_session(bodies: &BodyRegistry) -> Session {
    let mut session = Session::new(session_cfg());
    let cfg = ShardSimConfig::new(shard_id(0, 0));
    session.register(ShardSim::new_with_bodies(
        cfg,
        Box::new(SimpleRegistry::builtin()),
        bodies.clone(),
    ));
    load_flat_ground(&mut session);
    session
}

/// SplitMix64(任务书 §2.3;不引入随机数库)。
fn splitmix64_next(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E3779B97F4A7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}

fn sample_intent(seed: u64, range: &[[f32; 2]; 3]) -> [f32; 3] {
    let mut state = seed;
    let mut intent = [0.0f32; 3];
    for (i, &[lo, hi]) in range.iter().enumerate() {
        let u = (splitmix64_next(&mut state) >> 40) as f32 / 16777216.0;
        intent[i] = lo + u * (hi - lo);
    }
    intent
}

impl AgentEnv {
    /// 校验配置、装身体、建编解码器、布景生成主体、站 `settle_ticks` 拍、
    /// 取初态快照(任务书 §2.2 `new`)。
    pub fn new(cfg: EnvConfig) -> Result<Self, EnvError> {
        if cfg.episode_controls < 1 {
            return Err(EnvError::Config("episode_controls 须 ≥ 1".to_owned()));
        }
        if cfg.manifest.normalization.is_some() {
            return Err(EnvError::Config(
                "训练环境输出未归一化观测,manifest.normalization 必须是 None".to_owned(),
            ));
        }
        if !cfg.fall_up_y_min.is_finite() {
            return Err(EnvError::Config("fall_up_y_min 须有限".to_owned()));
        }
        for &[lo, hi] in &cfg.intent_range {
            if !lo.is_finite() || !hi.is_finite() || lo > hi || lo.abs() > 1.0 || hi.abs() > 1.0 {
                return Err(EnvError::Config(
                    "intent_range 每项须有限、lo ≤ hi、绝对值 ≤ 1".to_owned(),
                ));
            }
        }
        let mut bodies = BodyRegistry::new();
        let hash = bodies
            .insert_toml(&cfg.body_toml)
            .map_err(|e| EnvError::Body(e.to_string()))?;
        let def = bodies
            .get(&hash)
            .ok_or_else(|| EnvError::Body("身体未装进注册表".to_owned()))?
            .clone();
        // 哈希不符在这里报出来(PolicyCodec::new)。
        let codec = PolicyCodec::new(cfg.manifest.clone(), &def)?;
        let entity = EntityId::from_parts(1, 1);
        let mut session = build_session(&bodies);
        session
            .spawn_agent_from(
                InputSource::SCHEDULER,
                shard_id(0, 0),
                entity,
                hash,
                EntityState {
                    translation: SPAWN,
                    ..EntityState::default()
                },
            )
            .map_err(|e| EnvError::Spawn(format!("生成主体失败:{e}")))?;
        // spawn 后先走一拍:owner 表拍末登记(run_quad 同款)。
        session.tick();
        // 站 settle_ticks 拍(不发任何目标命令,内核保持出生目标)。
        for _ in 0..cfg.settle_ticks {
            session.tick();
        }
        let initial = session.world_snapshot();
        let dof = codec.dof();
        Ok(Self {
            cfg,
            codec,
            bodies,
            entity,
            initial,
            session,
            log: CommandLog::new(),
            intent: [0.0; 3],
            prev_action: vec![0.0; dof],
            steps: 0,
            finished: false,
            reset_done: false,
        })
    }

    pub fn obs_dim(&self) -> usize {
        self.codec.obs_dim()
    }

    pub fn act_dim(&self) -> usize {
        self.codec.dof()
    }

    /// 从初态快照恢复会话、采样 intent、步数清零、返回观测(任务书 §2.2 `reset`)。
    pub fn reset(&mut self, seed: u64) -> Result<Vec<f32>, EnvError> {
        let empty = CommandLog::new();
        let bodies = self.bodies.clone();
        let new_session = replay_world_with_bodies(
            &self.initial,
            &empty,
            session_cfg(),
            || Box::new(SimpleRegistry::builtin()),
            || bodies.clone(),
        )
        .map_err(|e| EnvError::Spawn(format!("恢复初态快照失败:{e}")))?;
        let old = std::mem::replace(&mut self.session, new_session);
        old.shutdown();
        self.log = CommandLog::new();
        self.intent = sample_intent(seed, &self.cfg.intent_range);
        self.steps = 0;
        self.prev_action = vec![0.0; self.codec.dof()];
        self.finished = false;
        self.reset_done = true;
        let zeros = vec![0.0; self.codec.dof()];
        self.observation(&zeros, self.intent)
    }

    /// 发目标 → 推一个控制周期 → 观测/奖励/结束判定(任务书 §2.2 `step`)。
    pub fn step(&mut self, action: &[f32]) -> Result<StepOutput, EnvError> {
        if !self.reset_done || self.finished {
            return Err(EnvError::NeedsReset);
        }
        let dof = self.codec.dof();
        if action.len() != dof {
            return Err(EnvError::ActionDim {
                got: action.len(),
                want: dof,
            });
        }
        if action.iter().any(|x| !x.is_finite()) {
            return Err(EnvError::NonFiniteAction);
        }
        let act = self.codec.decode(action)?;
        let entry = self
            .session
            .send_agent_joint_targets_to(InputSource::SCHEDULER, self.entity, act.targets.clone())
            .map_err(|e| EnvError::Spawn(format!("发关节目标失败:{e}")))?;
        self.log.append(entry);
        for _ in 0..self.cfg.manifest.control_period_ticks {
            self.session.tick();
        }
        let mut reward = RewardInputs {
            base_linvel_body: [0.0; 3],
            base_angvel_body: [0.0; 3],
            projected_gravity: [0.0; 3],
            root_height: 0.0,
            up_y: 0.0,
            joint_pos: vec![0.0; dof],
            joint_vel: vec![0.0; dof],
            action: act.raw.clone(),
            prev_action: self.prev_action.clone(),
            targets_clamped: act.clamped as u32,
            intent: self.intent,
        };
        let terminated;
        let obs;
        if !self.session.is_agent(self.entity) {
            terminated = true;
            obs = vec![0.0; self.codec.obs_dim()];
        } else {
            let root = self
                .session
                .agent_root_state(self.entity)
                .ok_or_else(|| EnvError::Spawn("取不到根状态".to_owned()))?;
            let joints = self
                .session
                .agent_joint_state(self.entity)
                .ok_or_else(|| EnvError::Spawn("取不到关节状态".to_owned()))?;
            let inv = root.rotation.inverse();
            let lin = inv * root.linvel;
            let ang = inv * root.angvel;
            let grav = inv * Vec3::new(0.0, -1.0, 0.0);
            let up_y = (root.rotation * Vec3::new(0.0, 1.0, 0.0)).y;
            reward.base_linvel_body = [lin.x, lin.y, lin.z];
            reward.base_angvel_body = [ang.x, ang.y, ang.z];
            reward.projected_gravity = [grav.x, grav.y, grav.z];
            reward.root_height = root.translation.y - GROUND_TOP_Y;
            reward.up_y = up_y;
            reward.joint_pos = joints.positions.clone();
            reward.joint_vel = joints.velocities.clone();
            let snapshot = AgentSnapshot {
                root_rotation: root.rotation,
                root_linvel: root.linvel,
                root_angvel: root.angvel,
                joint_pos: joints.positions,
                joint_vel: joints.velocities,
                last_action: act.raw.clone(),
                intent: self.intent,
            };
            let mut out = Vec::new();
            self.codec.encode(&snapshot, &mut out)?;
            obs = out;
            terminated = up_y < self.cfg.fall_up_y_min;
        }
        self.steps += 1;
        let truncated = !terminated && self.steps == self.cfg.episode_controls;
        self.prev_action = act.raw;
        self.finished = terminated || truncated;
        Ok(StepOutput {
            obs,
            reward,
            terminated,
            truncated,
        })
    }

    /// 本主体全部连杆状态(身体定义顺序)。
    pub fn link_states(&self) -> Vec<EntityState> {
        self.session
            .agent_states()
            .into_iter()
            .find(|(id, _, _)| *id == self.entity)
            .map(|(_, _, states)| states)
            .unwrap_or_default()
    }

    pub fn entity(&self) -> EntityId {
        self.entity
    }

    /// 当前 12 个关节角(自由度顺序;只读,E02a 姿态转储用)。
    pub fn joint_positions(&self) -> Vec<f32> {
        self.session
            .agent_joint_state(self.entity)
            .map(|j| j.positions)
            .unwrap_or_default()
    }

    /// 地面顶面 world y(E02a 姿态转储/站立高度基准用)。
    pub fn ground_top_y(&self) -> f32 {
        GROUND_TOP_Y
    }

    pub fn intent(&self) -> [f32; 3] {
        self.intent
    }

    /// 初态快照(settle 后取;每次 reset 从它恢复)。
    pub fn initial_snapshot(&self) -> &WorldSnapshot {
        &self.initial
    }

    /// 本回合 reset 以来的命令日志。
    pub fn command_log(&self) -> &CommandLog {
        &self.log
    }

    pub fn bodies(&self) -> &BodyRegistry {
        &self.bodies
    }

    /// 从当前会话状态拼观测(last_action / intent 由调用方给)。
    fn observation(&self, last_action: &[f32], intent: [f32; 3]) -> Result<Vec<f32>, EnvError> {
        let root = self
            .session
            .agent_root_state(self.entity)
            .ok_or_else(|| EnvError::Spawn("取不到根状态".to_owned()))?;
        let joints = self
            .session
            .agent_joint_state(self.entity)
            .ok_or_else(|| EnvError::Spawn("取不到关节状态".to_owned()))?;
        let snapshot = AgentSnapshot {
            root_rotation: root.rotation,
            root_linvel: root.linvel,
            root_angvel: root.angvel,
            joint_pos: joints.positions,
            joint_vel: joints.velocities,
            last_action: last_action.to_vec(),
            intent,
        };
        let mut obs = Vec::new();
        self.codec.encode(&snapshot, &mut obs)?;
        Ok(obs)
    }
}
