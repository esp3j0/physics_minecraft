//! stage —— 场景布景(REQ-011 §3.6;W05)。**零 macroquad**:把解析后的场景
//! 切成布景段 / 动作段(纯函数),再用一个小步进器在交互模式下执行布景段
//! (连接装载后自动一次)、`R` 重建、`K` 播放动作段。复用
//! [`crate::script::parse_scenario`] 与 [`EngineCommand`](fill 的分批规则与
//! 脚本引擎同源:同一 `fill_cells` 实现 + [`FILL_BATCH_PER_FRAME`])。脚本
//! 模式不经此模块,行为不变。
//!
//! 为什么不复用脚本 [`crate::script::Engine`]:引擎是一次性状态机(从第 1 步
//! 跑到场景尾、Check 失败计入 failed_checks、wait_until 超时 = 场景失败),
//! 而布景需要「任意时刻重放前缀段、跳过 check/screenshot/相机、超时继续」,
//! 语义不同;执行器只支持 §3.6 列出的动作,其余一律跳过。

use types::{BlockId, BlockPos, EntityDesc, EntityId, EntityState};

use crate::script::{
    Action, COMPOUND_HALF_SIZE, CameraPose, Check, Cmp, DetonateAt, EngineCommand,
    FILL_BATCH_PER_FRAME, Scenario, SpawnAt, WorldQuery,
};

/// 布景 / 动作两段(REQ-011 §3.6 切分结果;步骤下标,0 基)。
#[derive(Debug, Clone, PartialEq)]
pub struct StageSplit {
    /// 第一个动作步骤之前的全部步骤。
    pub staging: Vec<usize>,
    /// 第一个动作步骤起(含)的全部步骤。
    pub action: Vec<usize>,
}

/// 动作步骤判定(§3.6):`spawn`、`detonate`,或写入空气的 `set_block` / `fill`
/// (拆支撑、拆脚手架)。其余动作(camera、look_at、fly、couple、wait_*、
/// check、screenshot)都不是。
#[must_use]
pub fn is_action_step(action: &Action) -> bool {
    match action {
        Action::Spawn { .. } | Action::Detonate { .. } => true,
        Action::SetBlock { block, .. } => *block == BlockId::AIR,
        Action::Fill { block, .. } => *block == BlockId::AIR,
        _ => false,
    }
}

/// 切分(§3.6):布景段 = 第一个动作步骤之前的全部步骤,动作段 = 其余步骤。
/// 没有动作步骤 → 全部是布景段(如 materials)。
#[must_use]
pub fn split(scenario: &Scenario) -> StageSplit {
    let first = scenario
        .steps
        .iter()
        .position(|s| is_action_step(&s.action));
    match first {
        Some(i) => StageSplit {
            staging: (0..i).collect(),
            action: (i..scenario.steps.len()).collect(),
        },
        None => StageSplit {
            staging: (0..scenario.steps.len()).collect(),
            action: Vec::new(),
        },
    }
}

/// 场景是否含 `[server]` 表(§3.6:有则提示未应用)。
#[must_use]
pub fn has_server_table(scenario: &Scenario) -> bool {
    !scenario.server.is_empty()
}

/// 执行器发出的条目(打桩测试的断言面):一条 [`EngineCommand`] 或一条
/// `RemoveEntity`(R 键的清场)。按发出顺序排列。
#[derive(Debug, Clone, PartialEq)]
pub enum StageOut {
    Command(EngineCommand),
    Remove(EntityId),
}

/// 执行器状态(§3.6)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StagePhase {
    /// 布景段执行中(自动一次,或 R 之后的重放)。
    Building,
    /// 布景完成,等用户操作(`R` 重建 / `K` 播放)。
    Ready,
    /// 动作段播放中(段内第 `step` / 共 `n` 步;播放中再按 K 无效)。
    Playing { step: usize, n: usize },
    /// 动作段播放完毕(提示 `action playback done`)。
    Played,
}

/// 进行中的 `fill` 步骤(脚本引擎 `FillProgress` 的布景版:格队列 + 下标)。
#[derive(Debug, Clone, PartialEq)]
struct FillProgress {
    cells: Vec<BlockPos>,
    block: BlockId,
    next: usize,
}

/// 正在跑哪一段。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Segment {
    Staging,
    Action,
}

/// 布景执行器:只支持 §3.6 列出的动作;`screenshot`、`check`、`fly`、
/// `couple` 及其他一律跳过。
///
/// 帧驱动:交互主循环每帧把(帧时长、服务器拍号、世界只读查询)交给
/// [`StageDriver::frame`],取回相机位姿与应发条目。等待只看服务器拍号
/// (与脚本引擎同口径),`wait_until` 超时**不失败**——提示后继续下一步。
pub struct StageDriver {
    scenario: Scenario,
    /// 切分一次,重放 / 播放都按它跑。
    split: StageSplit,
    /// 当前相机位姿(camera / look_at 更新;其余帧保持)。
    camera: CameraPose,
    /// 执行阶段。
    phase: StagePhase,
    /// 当前段内执行到的位置(`staging` / `action` 的下标)。
    cursor: usize,
    /// 正在跑哪一段。
    segment: Segment,
    /// 进行中的 `fill`(分批:每帧最多 [`FILL_BATCH_PER_FRAME`] 条,发完最后
    /// 一批的那一帧该步结束)。
    fill: Option<FillProgress>,
    /// 等待步骤的起点拍号(进入该步那一帧的拍号)。
    wait_start: Option<u64>,
    /// `wait_until` 连续成立段的起始拍号(与脚本引擎同语义;步骤切换 /
    /// `R` 重建 / `K` 播放时清零)。
    wait_hold_since: Option<u64>,
    /// 本帧的超时提示(`wait_until` 超时;主循环读走)。
    timeout_notice: Option<String>,
    /// 按键拒绝提示(W05-2:Building 时按 `K`;主循环读走)。
    reject_notice: Option<String>,
    /// 场景含 `[server]` 表的提示(创建即置位;主循环读走)。
    server_notice: Option<String>,
}

impl StageDriver {
    /// 以场景与初始相机位姿创建;主循环首帧起自动步进布景段(连接装载后)。
    /// 含 `[server]` 表 → 提示待取。
    #[must_use]
    pub fn new(scenario: Scenario, start: CameraPose) -> Self {
        let server_notice = has_server_table(&scenario).then(|| {
            "[server] settings not applied (server already running); use cargo xtask smoke if needed"
                .to_owned()
        });
        Self {
            split: split(&scenario),
            scenario,
            camera: start,
            phase: StagePhase::Building,
            cursor: 0,
            segment: Segment::Staging,
            fill: None,
            wait_start: None,
            wait_hold_since: None,
            timeout_notice: None,
            reject_notice: None,
            server_notice,
        }
    }

    /// 场景含 `[server]` 表的提示(取走一次性)。
    pub fn take_server_notice(&mut self) -> Option<String> {
        self.server_notice.take()
    }

    /// 本帧的超时提示(`wait_until` 超时;取走一次性)。
    pub fn take_timeout_notice(&mut self) -> Option<String> {
        self.timeout_notice.take()
    }

    /// 按键拒绝提示(取走一次性;W05-2:布景搭建中按 `K` →
    /// `stage still building, wait`,走调用方的 3 秒提示通道)。
    pub fn take_reject_notice(&mut self) -> Option<String> {
        self.reject_notice.take()
    }

    /// 相机覆写(W05-2):只在 `Building`(搭布景,含 `R` 之后的重放)时返回
    /// `Some(布景位姿)`——render 据此每帧覆写飞行相机;`Playing` 时不覆写
    /// (§3.6:K 播放跳过相机类步骤,相机留给用户——punch 的动作段带等待,
    /// 播放持续数秒,锁死相机用户就看不到撞击);`Ready` / `Played` 同样
    /// `None`(不夺回控制权)。
    #[must_use]
    pub fn camera_override(&self) -> Option<CameraPose> {
        match self.phase {
            StagePhase::Building => Some(self.camera),
            StagePhase::Ready | StagePhase::Played | StagePhase::Playing { .. } => None,
        }
    }

    /// 当前相机位姿。
    #[must_use]
    pub fn camera(&self) -> CameraPose {
        self.camera
    }

    /// 执行阶段。
    #[must_use]
    pub fn phase(&self) -> StagePhase {
        self.phase
    }

    /// 状态行文本(交互 HUD;ASCII):building / ready / playing / done。
    #[must_use]
    pub fn status_line(&self) -> String {
        match self.phase {
            StagePhase::Building => "stage: building...".to_owned(),
            StagePhase::Ready => "stage: ready (R rebuild, K play)".to_owned(),
            StagePhase::Playing { step, n } => format!("stage: playing step {step}/{n}"),
            StagePhase::Played => "stage: done (action playback done)".to_owned(),
        }
    }

    /// `R`:对当前每个活实体发一条 `RemoveEntity`(**按 `EntityId` 升序**,打桩
    /// 断言),再从头重放布景段(相机回到布景位姿由段里的 camera / look_at
    /// 步骤恢复)。散落在别处的方块不清理(§3.6 非目标)。
    pub fn rebuild(&mut self, world: &dyn WorldQuery, out: &mut Vec<StageOut>) {
        let mut live: Vec<EntityId> = world
            .live_entities()
            .into_iter()
            .map(|(id, _)| id)
            .collect();
        live.sort_unstable();
        for id in live {
            out.push(StageOut::Remove(id));
        }
        self.segment = Segment::Staging;
        self.cursor = 0;
        self.phase = StagePhase::Building;
        self.fill = None;
        self.wait_start = None;
        self.wait_hold_since = None;
    }

    /// `K`:播放动作段。播放中再按无效(不发命令、不改状态);**布景搭建中
    /// 按 `K` 同样无效**并置提示 `stage still building, wait`(W05-2:布景
    /// 只搭一半就播放会撞上未完成的结构);播完进 `Played`(主循环渲染
    /// `action playback done`)。`Played` 后再按重放动作段(保持现状)。
    /// 空动作段(materials 类)同样合法:一步即完,直接提示播完。
    pub fn play(&mut self) {
        match self.phase {
            StagePhase::Playing { .. } => return, // 播放中再按 K 无效
            StagePhase::Building => {
                self.reject_notice = Some("stage still building, wait".to_owned());
                return;
            }
            StagePhase::Ready | StagePhase::Played => {}
        }
        self.segment = Segment::Action;
        self.cursor = 0;
        self.fill = None;
        self.wait_start = None;
        self.wait_hold_since = None;
        if self.split.action.is_empty() {
            self.phase = StagePhase::Played;
        } else {
            self.phase = StagePhase::Playing {
                step: 1,
                n: self.split.action.len(),
            };
        }
    }

    /// 推进一帧:执行当前段直到遇到等待步骤或 fill 分批截断;返回本帧应发的
    /// 条目(已按序排好)。`dt` 仅占位(布景等待只看服务器拍号,墙钟不进)。
    pub fn frame(&mut self, dt: f32, tick: u64, world: &dyn WorldQuery) -> Vec<StageOut> {
        let _ = dt; // 墙钟不进布景;签名保留给主循环统一调用
        let mut out = Vec::new();
        if matches!(self.phase, StagePhase::Ready | StagePhase::Played) {
            return out;
        }
        let is_staging = self.segment == Segment::Staging;
        let indices: Vec<usize> = if is_staging {
            self.split.staging.clone()
        } else {
            self.split.action.clone()
        };
        while self.cursor < indices.len() {
            let action = self.scenario.steps[indices[self.cursor]].action.clone();
            match action {
                // 相机类步骤:布景段执行,R 重放也执行(相机回布景位姿);
                // K 播放跳过(§3.6:相机留给用户)。
                Action::Camera { pos, yaw, pitch } => {
                    if is_staging {
                        self.camera = CameraPose { pos, yaw, pitch };
                    }
                    self.advance();
                }
                Action::LookAt { target } => {
                    if is_staging {
                        self.apply_look_at(target);
                    }
                    self.advance();
                }
                Action::Fill { from, to, block } => {
                    // 分批同脚本引擎:格队列一次展开,每帧 ≤ FILL_BATCH_PER_FRAME
                    // 条,最后一批发出那一帧该步结束(后续瞬时步骤可在同帧继续)。
                    let mut state = self.fill.take().unwrap_or_else(|| FillProgress {
                        cells: fill_cells(from, to),
                        block,
                        next: 0,
                    });
                    let end = (state.next + FILL_BATCH_PER_FRAME).min(state.cells.len());
                    for pos in &state.cells[state.next..end] {
                        out.push(StageOut::Command(EngineCommand::SetBlock {
                            pos: *pos,
                            block: state.block,
                        }));
                    }
                    state.next = end;
                    if state.next >= state.cells.len() {
                        self.advance();
                    } else {
                        self.fill = Some(state);
                        break;
                    }
                }
                Action::SetBlock { pos, block } => {
                    out.push(StageOut::Command(EngineCommand::SetBlock { pos, block }));
                    self.advance();
                }
                Action::WaitTicks { n } => {
                    let start = *self.wait_start.get_or_insert(tick);
                    if tick >= start.saturating_add(n) {
                        self.advance();
                    } else {
                        break;
                    }
                }
                Action::WaitUntil {
                    checks,
                    timeout_ticks,
                    hold_ticks,
                } => {
                    let start = *self.wait_start.get_or_insert(tick);
                    // hold_ticks 与脚本引擎同语义(必修 3):条件连续成立
                    // `hold_ticks` 个服务器拍才算满足(按拍号差计,不按观测
                    // 次数);不成立清空,重新成立记当前拍。超时提示后继续
                    // 下一步的行为不变(REQ-011 §3.6)。
                    if checks_hold(&checks, world) {
                        let since = *self.wait_hold_since.get_or_insert(tick);
                        let held = tick.saturating_sub(since) + 1;
                        if held >= hold_ticks {
                            self.advance();
                        } else {
                            break;
                        }
                    } else {
                        self.wait_hold_since = None;
                        if tick >= start.saturating_add(timeout_ticks) {
                            // §3.6:超时不退出,提示后继续下一步。
                            self.timeout_notice =
                                Some(format!("stage wait timeout: {}", describe_checks(&checks)));
                            self.advance();
                        } else {
                            break;
                        }
                    }
                }
                Action::Spawn {
                    at,
                    count,
                    spacing,
                    velocity,
                    shape,
                    ..
                } => {
                    // 全部命令当帧发出(同脚本引擎细则 6);不等 Spawned 回执
                    // (K 播放不校验,播完即提示)。
                    let base = match at {
                        SpawnAt::At(p) => p,
                        SpawnAt::Ahead { ahead, lift } => {
                            self.camera.pos
                                + self.camera.forward() * ahead
                                + types::glam::Vec3::new(0.0, lift, 0.0)
                        }
                    };
                    for i in 0..count {
                        let offset =
                            types::glam::Vec3::new((i % 5) as f32, 0.0, (i / 5) as f32) * spacing;
                        let desc = match &shape {
                            Some(sh) => EntityDesc::compound(COMPOUND_HALF_SIZE, sh.clone()),
                            None => EntityDesc::default(),
                        };
                        out.push(StageOut::Command(EngineCommand::Spawn {
                            state: EntityState {
                                translation: base + offset,
                                linvel: velocity,
                                ..EntityState::default()
                            },
                            desc,
                        }));
                    }
                    self.advance();
                }
                Action::Detonate { at, power, radius } => {
                    // nearest 吸附同引擎(相机 25 m 内最近活实体)。
                    let center = match at {
                        DetonateAt::At(p) => Some(p),
                        DetonateAt::Nearest => nearest_entity(&self.camera, world),
                    };
                    if let Some(center) = center {
                        out.push(StageOut::Command(EngineCommand::Detonate {
                            center,
                            power,
                            radius,
                        }));
                    }
                    // nearest 无可吸附实体 → 跳过该步(布景播放不失败)。
                    self.advance();
                }
                // 其余动作(fly / couple / check / screenshot)一律跳过。
                _ => self.advance(),
            }
        }

        // 段尾:布景段完 → Ready;动作段完 → Played(提示)。
        if self.cursor >= indices.len() {
            self.fill = None;
            self.wait_start = None;
            self.phase = if is_staging {
                StagePhase::Ready
            } else {
                StagePhase::Played
            };
        }
        // Playing 的步号同步(段内第几步,1 起)。
        if let StagePhase::Playing { step, n } = &mut self.phase {
            *step = (self.cursor + 1).min(*n);
        }
        out
    }

    /// 前进到段内下一步,清等待中间态。
    fn advance(&mut self) {
        self.cursor += 1;
        self.wait_start = None;
        self.wait_hold_since = None;
    }

    /// `look_at` 角度换算(与脚本引擎同式):目标与相机重合时保持原朝向。
    fn apply_look_at(&mut self, target: types::glam::Vec3) {
        let d = target - self.camera.pos;
        if d == types::glam::Vec3::ZERO {
            return;
        }
        let horiz = (d.x * d.x + d.z * d.z).sqrt();
        if horiz > 0.0 {
            self.camera.yaw = d.z.atan2(d.x).to_degrees();
        }
        self.camera.pitch = d.y.atan2(horiz).to_degrees();
    }
}

/// `from..=to` 按 `BlockPos` 升序展开(**直接调用脚本引擎的同一实现**,
/// 保证布景与脚本模式的分批顺序逐位一致)。
#[must_use]
pub fn fill_cells(from: BlockPos, to: BlockPos) -> Vec<BlockPos> {
    crate::script::engine::fill_cells(from, to)
}

/// nearest 爆心吸附(与脚本引擎同式:相机 25 m 内最近活实体)。
fn nearest_entity(camera: &CameraPose, world: &dyn WorldQuery) -> Option<types::glam::Vec3> {
    let mut best: Option<(f32, types::glam::Vec3)> = None;
    for (_, pos) in world.live_entities() {
        let dist = (pos - camera.pos).length();
        let closer = match best {
            Some((d, _)) => dist < d,
            None => true,
        };
        if dist <= 25.0 && closer {
            best = Some((dist, pos));
        }
    }
    best.map(|(_, p)| p)
}

/// 检查项是否全部成立(布景版:只求值;逐条同引擎口径,未装载 = 不成立)。
fn checks_hold(checks: &[Check], world: &dyn WorldQuery) -> bool {
    checks.iter().all(|c| check_holds(c, world))
}

fn check_holds(check: &Check, world: &dyn WorldQuery) -> bool {
    match check {
        Check::LiveEntities(cmp) => cmp_holds(*cmp, world.live_entities().len() as u64),
        // 布景执行器不跟踪 spawn 标签(播放不校验回执);有标签的检查按 0 个
        // 求值。现有场景的 wait_until 不用 labeled_alive,防御。
        Check::LabeledAlive { label, cmp } => {
            let _ = label;
            cmp_holds(*cmp, 0)
        }
        Check::Block { pos, is } => world.block_at(*pos) == Some(*is),
        Check::Count { from, to, is, cmp } => {
            let mut n = 0u64;
            for x in from.x..=to.x {
                for y in from.y..=to.y {
                    for z in from.z..=to.z {
                        match world.block_at(BlockPos::new(x, y, z)) {
                            Some(b) if b == *is => n += 1,
                            Some(_) => {}
                            // 任一格未装载 → 不成立(同引擎:不下结论)。
                            None => return false,
                        }
                    }
                }
            }
            cmp_holds(*cmp, n)
        }
        Check::ChunksLoaded(cmp) => cmp_holds(*cmp, world.chunks_loaded() as u64),
        // no_floating 需要位移历史(引擎的采样);布景不建历史 → 观测不足,
        // 不下结论(不成立)。现有场景的 wait_until 不用它,防御。
        Check::NoFloating { .. } => false,
        // A06 主体检查:布景执行器不跟踪主体(回执/标签归引擎);观测不足
        // 不下结论(不成立)。防御性(现有布景场景不用主体动作)。
        Check::AgentIn { .. } | Check::AgentMoved { .. } | Check::AgentAlive { .. } => false,
    }
}

fn cmp_holds(cmp: Cmp, n: u64) -> bool {
    match cmp {
        Cmp::Eq(v) => n == v,
        Cmp::Le(v) => n <= v,
        Cmp::Ge(v) => n >= v,
    }
}

/// 检查项的可读描述(超时提示;ASCII 英文)。
fn describe_checks(checks: &[Check]) -> String {
    let parts: Vec<String> = checks
        .iter()
        .map(|c| match c {
            Check::LiveEntities(cmp) => format!("live_entities {cmp:?}"),
            Check::LabeledAlive { label, cmp } => format!("alive({label}) {cmp:?}"),
            Check::Block { pos, is } => format!("block {pos:?} is {is:?}"),
            Check::Count { from, to, is, cmp } => {
                format!("count {from:?}..{to:?} is {is:?} {cmp:?}")
            }
            Check::ChunksLoaded(cmp) => format!("chunks_loaded {cmp:?}"),
            Check::NoFloating { .. } => "no_floating".to_owned(),
            Check::AgentIn { .. } => "agent_in".to_owned(),
            Check::AgentMoved { .. } => "agent_moved".to_owned(),
            Check::AgentAlive { .. } => "agent_alive".to_owned(),
        })
        .collect();
    parts.join(" && ")
}

#[cfg(test)]
#[path = "stage_tests.rs"]
mod tests;
