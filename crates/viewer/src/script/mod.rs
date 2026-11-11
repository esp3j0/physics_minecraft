//! script —— 图形冒烟的场景脚本:解析、步骤状态机、机器检查(REQ-008 §3、§4)。
//!
//! 纯逻辑,零 macroquad:渲染层每帧把(帧时长、服务器拍号、世界只读查询、Spawned 回执)
//! 交给 [`Engine::frame`],执行它返回的相机位姿、命令与截图请求。因此整套脚本语义可在
//! 无窗口的单测里驱动(AC-S1–S3)。
//!
//! **总架构师接口桩(REQ-008 单元 01)**:本文件的公开类型、字段与函数签名固定,由实施方
//! 实现函数体;可以新增私有项或拆分子模块,但 `viewer::script::*` 的公开面不得改名、删字段
//! 或改类型。

pub mod manifest;

// crate 内可见(REQ-011 W05:tools::stage 直接调用引擎的 fill_cells,
// 保证布景与脚本模式的分批顺序逐位一致;对外仍不公开)。
pub(crate) mod engine;
mod parse;

use types::glam::Vec3;
use types::{BlockId, BlockPos, CompoundShape, EntityDesc, EntityId, EntityState};

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
    /// `count` 缺省 1,`spacing` 缺省 1.6,`velocity` 缺省 0,`shape` 缺省
    /// None(单格;F04a:给了 `shape` 发复合体,`at` = 原点格 `[0,0,0]` 的格心)。
    Spawn {
        at: SpawnAt,
        count: u32,
        spacing: f32,
        velocity: Vec3,
        label: Option<String>,
        shape: Option<CompoundShape>,
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
    /// 长方体填充(REQ-009 §10):解析时把两个对角规整为 `from` = 各轴最小、`to` = 各轴最大
    /// (含两端);格数超过 [`FILL_MAX_CELLS`] 为解析错误。引擎按 `BlockPos` 升序展开为
    /// `SetBlock` 命令,每帧最多 [`FILL_BATCH_PER_FRAME`] 条,最后一批发出的那一帧该步结束,
    /// 后续瞬时步骤可在同一帧继续。
    Fill {
        from: BlockPos,
        to: BlockPos,
        block: BlockId,
    },
    WaitTicks {
        n: u64,
    },
    WaitUntil {
        checks: Vec<Check>,
        timeout_ticks: u64,
        /// 可选(缺省 0):全部条件须**连续**成立 N 拍才算满足;中途任一拍不成立
        /// 即清零重新计数;`timeout_ticks` 仍从步骤开始的第一拍起算(计数未满 N
        /// 拍同样会超时失败)。toml 写法 `wait_until = { .., hold_ticks = N }`;
        /// 不写或写 0 与旧语义逐位相同(任一拍成立即过)。用途(S01 冒烟降噪):
        /// `live_entities = { eq = 0 }` + `hold_ticks = 30` 防实体帧未到的空过,
        /// 并保证凝固回填已完成。
        hold_ticks: u64,
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
    /// 主体生成(A06 §4):发 `SpawnAgent`(body 固定 capsule 资产——viewer
    /// 冒烟统一用玩家胶囊;`at` = 出生位置),收到 `Spawned` 后登记标签。
    /// 一个场景最多一个身体(放行 §4;第二条 spawn_agent = 解析错误)。
    SpawnAgent {
        at: Vec3,
        yaw: f32,
        label: String,
    },
    /// 附身(A06 §4):`follow` 缺省 false(相机不动,冒烟截图机位确定);
    /// true 时切第三人称跟随。
    Possess {
        label: String,
        follow: bool,
    },
    /// 解除附身(A06 §4)。
    Unpossess,
    /// 主体意图(A06 §4):持续生效,直到下一条 `intent`;`move` = 世界系
    /// 水平速度 (vx, vz),`yaw` 度,`jump` 一次性。
    Intent {
        mv: (f32, f32),
        yaw: f32,
        jump: bool,
    },
    /// 记下当前主体根位置(A06 §4;供 `agent_moved` 用)。
    MarkAgent {
        label: String,
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
    /// 闭区间长方体(`from`/`to` 各分量不要求有序)内等于 `is` 的格数满足
    /// `cmp`;格数上限 `FILL_MAX_CELLS`(解析期拒绝);区域内有格所在 chunk
    /// 未装载 → 检查不成立(F04b-2:落点不确定的结论用区域计数表达)。
    Count {
        from: BlockPos,
        to: BlockPos,
        is: BlockId,
        cmp: Cmp,
    },
    ChunksLoaded(Cmp),
    /// 不存在"底面高出所在柱最高实心方块顶面 `height` 米以上、且最近 `still_ticks`
    /// 拍内位移不超过 0.05 m"的活实体。
    NoFloating {
        height: f32,
        still_ticks: u64,
    },
    /// 主体根位置在闭区间盒内(A06 §4;`from`/`to` 各分量不要求有序)。
    AgentIn {
        label: String,
        from: Vec3,
        to: Vec3,
    },
    /// 主体相对 `mark_agent` 记录点的位移满足比较(A06 §4;米,`ge`/`le`
    /// 恰一个——None 侧 = 无该向约束;两个都 None = 解析错误)。
    AgentMoved {
        label: String,
        ge: Option<f32>,
        le: Option<f32>,
    },
    /// 主体存活(A06 §4;标签对应的主体在主体表中)。
    AgentAlive {
        label: String,
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
    /// 实体的复合形状(REQ-010 §10 修订 4;单格实体 = None)。W04 调试面板
    /// 的复合体计数用(有形状的实体才算复合体);默认 None(假世界可不实现
    /// 复合体语义,`WorldState` 覆盖)。
    fn entity_shape(&self, _id: EntityId) -> Option<&CompoundShape> {
        None
    }
    /// 当前唯一主体及其根位置(A06 §4;一场景一身体。无主体 → None)。
    /// 假世界可不实现(默认 None)。
    fn live_agent(&self) -> Option<(EntityId, Vec3)> {
        None
    }
}

/// 一帧的输入。
pub struct FrameInput<'a> {
    /// 帧时长(秒)。
    pub dt: f32,
    /// viewer 收到的最新服务器拍号。
    pub server_tick: u64,
    pub world: &'a dyn WorldQuery,
    /// 自上一帧以来收到的 `Spawned` 回执(按到达顺序)。引擎按**自己记的
    /// 请求顺序**消费(R1 §3.3:`spawn_requests` 队列)——方块 `spawn` 与
    /// `spawn_agent` 都会收到回执,归属由引擎入队顺序决定。
    pub spawned: &'a [EntityId],
}

/// 引擎要求渲染层发给服务器的命令。
#[derive(Debug, Clone, PartialEq)]
pub enum EngineCommand {
    /// 生成动态实体(`shape` = None 时 `EntityDesc` 用默认单格;F04a:
    /// 有 `shape` 时带复合形状,`state.translation` = 原点格 `[0,0,0]` 格心)。
    Spawn {
        state: EntityState,
        desc: EntityDesc,
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
    /// 主体生成(A06 §4;发送层转 `C2S::SpawnAgent`,逐条不混批)。
    SpawnAgent {
        position: Vec3,
        yaw: f32,
    },
    /// 附身(A06 §4;发送层转 `C2S::Possess`)。
    Possess {
        entity: EntityId,
    },
    /// 解除附身(A06 §4;发送层转 `C2S::Unpossess`)。
    Unpossess,
    /// 主体意图(A06 §4;发送层转 `C2S::AgentIntent`;move_xz 世界系)。
    AgentIntent {
        entity: EntityId,
        move_xz: (f32, f32),
        yaw: f32,
        jump: bool,
    },
}

/// `fill` 单步格数上限(REQ-009 §10)。
pub const FILL_MAX_CELLS: usize = 4096;
/// `fill` 每帧最多发出的 `SetBlock` 条数(服务器分片收件箱每拍 256 条,REQ-009 §10)。
pub const FILL_BATCH_PER_FRAME: usize = 64;

/// 复合体每格 cuboid 半边长(F04a `spawn.shape`;与核内
/// `physics_sim::destruction::DEBRIS_HALF_SIZE` 同值——viewer 不依赖
/// physics-sim,常量镜像,`snapshot` 侧有同款先例;服务端按 desc 重建
/// collider,值不一致只会改质量/手感,不会拒绝)。
pub const COMPOUND_HALF_SIZE: f32 = 0.45;

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
    /// A06 §4:脚本附身状态(Some = 已附身主体;follow 是否切第三人称)。
    /// 渲染层据此决定相机模式(follow = false 时相机保持 fly 位姿)。
    pub possess: Option<(EntityId, bool)>,
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

    /// 当前附身的主体 ID(渲染层上色用;R1:脚本 possess 后胶囊画高亮色,
    /// 与交互模式同一口径——审图 expect 写"高亮色"就必须真的画高亮色)。
    pub fn possessed_id(&self) -> Option<EntityId> {
        self.inner.possessed_id()
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
