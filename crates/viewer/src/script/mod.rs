//! script —— 图形冒烟的场景脚本:解析、步骤状态机、机器检查(REQ-008 §3、§4)。
//!
//! 纯逻辑,零 macroquad:渲染层每帧把(帧时长、服务器拍号、世界只读查询、Spawned 回执)
//! 交给 [`Engine::frame`],执行它返回的相机位姿、命令与截图请求。因此整套脚本语义可在
//! 无窗口的单测里驱动(AC-S1–S3)。
//!
//! **总架构师接口桩(REQ-008 单元 01)**:本文件的公开类型、字段与函数签名固定,由实施方
//! 实现函数体;可以新增私有项或拆分子模块,但 `viewer::script::*` 的公开面不得改名、删字段
//! 或改类型。

mod engine;
mod parse;

pub mod manifest;

use types::glam::Vec3;
use types::{BlockId, BlockPos, EntityId, EntityState};

/// 解析后的场景(REQ-008 §3.1)。
#[derive(Debug, Clone, PartialEq)]
pub struct Scenario {
    pub name: String,
    pub req: Option<String>,
    pub description: String,
    /// 整场景墙钟上限(秒;缺省 180)。
    pub timeout_secs: u64,
    /// `[server]` 覆盖项,原样保留给编排器(键由 `app::Config` 校验;缺省空表)。
    pub server: toml::Table,
    /// `[viewer] view_distance`(缺省 None = viewer 默认)。
    pub view_distance: Option<u8>,
    pub steps: Vec<Step>,
}

/// 一个步骤:恰好一个动作 + 可选标签。
#[derive(Debug, Clone, PartialEq)]
pub struct Step {
    pub label: Option<String>,
    pub action: Action,
}

/// 步骤动作(REQ-008 §3.2)。角度单位为度,距离为米,时间为服务器拍。
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    Camera {
        pos: Vec3,
        yaw: f32,
        pitch: f32,
    },
    LookAt {
        target: Vec3,
    },
    /// `speed` 缺省 16 m/s。
    Fly {
        to: Vec3,
        speed: f32,
    },
    Couple {
        on: bool,
    },
    /// `count` 缺省 1,`spacing` 缺省 1.6,`velocity` 缺省 0。
    Spawn {
        at: SpawnAt,
        count: u32,
        spacing: f32,
        velocity: Vec3,
        label: Option<String>,
    },
    Detonate {
        at: DetonateAt,
        power: f32,
        radius: f32,
    },
    /// 方块名在解析时经 `voxel::SimpleRegistry::builtin()` 解析为 ID。
    SetBlock {
        pos: BlockPos,
        block: BlockId,
    },
    WaitTicks {
        n: u64,
    },
    WaitUntil {
        checks: Vec<Check>,
        timeout_ticks: u64,
    },
    Check {
        checks: Vec<Check>,
    },
    /// `hud` 缺省 false。
    Screenshot {
        name: String,
        expect: String,
        hud: bool,
    },
}

/// 生成位置:绝对坐标,或相机前方 `ahead` 米再抬高 `lift` 米。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpawnAt {
    At(Vec3),
    Ahead { ahead: f32, lift: f32 },
}

/// 爆心:绝对坐标,或相机 25 m 内最近的活实体。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetonateAt {
    At(Vec3),
    Nearest,
}

/// 计数比较(toml 写法 `{ eq = n }` / `{ le = n }` / `{ ge = n }`,恰好一个)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cmp {
    Eq(u64),
    Le(u64),
    Ge(u64),
}

/// 机器检查项(REQ-008 §3.3)。
#[derive(Debug, Clone, PartialEq)]
pub enum Check {
    LiveEntities(Cmp),
    LabeledAlive {
        label: String,
        cmp: Cmp,
    },
    Block {
        pos: BlockPos,
        is: BlockId,
    },
    ChunksLoaded(Cmp),
    /// 不存在"底面高出所在柱最高实心方块顶面 `height` 米以上、且最近 `still_ticks`
    /// 拍内位移不超过 0.05 m"的活实体。
    NoFloating {
        height: f32,
        still_ticks: u64,
    },
}

/// 解析错误。`step` 为出错步骤的序号(**从 1 开始**);顶层错误为 None。
#[derive(Debug, Clone, PartialEq)]
pub struct ScriptError {
    pub step: Option<usize>,
    pub message: String,
}

impl std::fmt::Display for ScriptError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.step {
            Some(n) => write!(f, "第 {n} 步:{}", self.message),
            None => write!(f, "{}", self.message),
        }
    }
}

impl std::error::Error for ScriptError {}

/// 解析场景文本(REQ-008 §3.1–§3.3;AC-S1)。
pub fn parse_scenario(text: &str) -> Result<Scenario, ScriptError> {
    parse::parse_scenario(text)
}

/// 相机位姿。朝向约定与 viewer 飞行相机一致:
/// `forward = (cos yaw · cos pitch, sin pitch, sin yaw · cos pitch)`(角度为度)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraPose {
    pub pos: Vec3,
    pub yaw: f32,
    pub pitch: f32,
}

impl CameraPose {
    /// 前向单位向量(见类型文档的约定)。
    pub fn forward(&self) -> Vec3 {
        let (sy, cy) = self.yaw.to_radians().sin_cos();
        let (sp, cp) = self.pitch.to_radians().sin_cos();
        Vec3::new(cy * cp, sp, sy * cp)
    }
}

/// 引擎读取世界的只读接口(由渲染层用 viewer 的世界视图实现;单测用假世界)。
pub trait WorldQuery {
    /// 当前实体表中的活实体及其位置。
    fn live_entities(&self) -> Vec<(EntityId, Vec3)>;
    /// 方块镜像中某格的方块;所在 chunk 未装载 → None。
    fn block_at(&self, pos: BlockPos) -> Option<BlockId>;
    /// 方块镜像中的 chunk 数。
    fn chunks_loaded(&self) -> usize;
}

/// 一帧的输入。
pub struct FrameInput<'a> {
    /// 帧时长(秒)。
    pub dt: f32,
    /// viewer 收到的最新服务器拍号。
    pub server_tick: u64,
    pub world: &'a dyn WorldQuery,
    /// 自上一帧以来收到的 `Spawned` 回执(按到达顺序)。
    pub spawned: &'a [EntityId],
}

/// 引擎要求渲染层发给服务器的命令。
#[derive(Debug, Clone, PartialEq)]
pub enum EngineCommand {
    /// 生成动态立方体(`EntityDesc` 用默认值)。
    Spawn {
        state: EntityState,
    },
    Detonate {
        center: Vec3,
        power: f32,
        radius: f32,
    },
    SetBlock {
        pos: BlockPos,
        block: BlockId,
    },
}

/// 截图请求:渲染层在当帧画完后截取并保存为 [`ScreenshotRequest::file_name`]。
#[derive(Debug, Clone, PartialEq)]
pub struct ScreenshotRequest {
    /// 步骤序号(从 1 开始)。
    pub step: usize,
    pub name: String,
    pub expect: String,
    pub hud: bool,
}

impl ScreenshotRequest {
    /// `NN-<name>.png`(NN 为两位补零的步骤序号)。
    pub fn file_name(&self) -> String {
        format!("{:02}-{}.png", self.step, self.name)
    }
}

/// 步骤结果。
#[derive(Debug, Clone, PartialEq)]
pub enum StepOutcome {
    Ok,
    /// 检查失败(各条失败原因);场景继续执行。
    CheckFailed(Vec<String>),
    /// 步骤错误(超时、无可吸附实体等);场景随之失败。
    Error(String),
}

/// 一个已结束步骤的记录(写入 manifest)。
#[derive(Debug, Clone, PartialEq)]
pub struct StepRecord {
    /// 步骤序号(从 1 开始)。
    pub step: usize,
    pub label: Option<String>,
    /// 动作名(与 toml 键相同,如 `"camera"`、`"wait_until"`)。
    pub action: String,
    pub start_tick: u64,
    pub end_tick: u64,
    /// 步骤结束时的相机位姿。
    pub camera: CameraPose,
    pub outcome: StepOutcome,
    pub screenshot: Option<ScreenshotRequest>,
}

/// 一帧的输出。
#[derive(Debug, Clone, PartialEq)]
pub struct FrameOutput {
    /// 本帧应使用的相机位姿。
    pub camera: CameraPose,
    /// 相机 → 观察者耦合是否开启。
    pub coupled: bool,
    pub commands: Vec<EngineCommand>,
    /// 每帧至多一个截图请求。
    pub screenshot: Option<ScreenshotRequest>,
    /// 本帧内结束的步骤(按序)。
    pub finished: Vec<StepRecord>,
}

/// 引擎状态。
#[derive(Debug, Clone, PartialEq)]
pub enum EngineStatus {
    /// 正在执行第 `step` 步(从 1 开始)。
    Running { step: usize },
    /// 全部步骤结束;`failed_checks` = 结果为 CheckFailed 的步骤数。
    Done { failed_checks: usize },
    /// 场景失败(脚本错误、超时、断连等)。
    Failed { step: usize, reason: String },
}

/// 步骤状态机(REQ-008 §3.2、§4;AC-S2、AC-S3)。
pub struct Engine {
    inner: engine::EngineInner,
    status: EngineStatus,
}

impl Engine {
    /// 以起始相机位姿创建引擎;耦合初始为开。
    pub fn new(scenario: Scenario, start: CameraPose) -> Self {
        Self {
            inner: engine::EngineInner::new(scenario, start),
            status: EngineStatus::Running { step: 1 },
        }
    }

    /// 推进一帧。
    pub fn frame(&mut self, input: &FrameInput<'_>) -> FrameOutput {
        let (output, status) = self.inner.frame(input);
        self.status = status;
        output
    }

    pub fn status(&self) -> &EngineStatus {
        &self.status
    }

    /// 结束后的退出码(REQ-008 §4):0 = 全部完成且检查成立;1 = 有检查失败;
    /// 2 = 场景失败。仍在运行 → None。
    pub fn exit_code(&self) -> Option<i32> {
        engine::exit_code(&self.status)
    }
}

#[cfg(test)]
#[path = "engine_tests.rs"]
mod tests;
