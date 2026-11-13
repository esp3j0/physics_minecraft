//! possess —— 附身状态机与意图映射(REQ-012 A06 §2;R1 重做为显式状态机)。
//! 纯逻辑、零 macroquad:渲染层每帧喂(按键快照、相机 yaw、服务器 Spawned
//! 回执、主体流、Possessed 回执),取回(要发的协议命令、意图、第三人称
//! 相机位姿);任何输入不 panic。`now` 一律由调用方传入,状态机不读墙钟,
//! 单测直接推时间。
//!
//! **自己的身体由回执 + 主体流确定**(R1 §3.2 归属规则,不改协议):
//! `Spawned` 回执只发给发请求的连接,只有 `SpawnAgent` 会生成主体,一个连接
//! 只有一个身体。所以:**收到过回执、又出现在 `AgentStates` 里的实体,就是
//! 我的身体。**这条规则不依赖回执与请求的顺序(方块回执先到也不归错——
//! 方块 ID 永不进主体流),请求被拒(服务器不回任何帧)也不会错位,服务器
//! 主动重生的身体(未请求的 Spawned + AgentStates 可见)同样适用。
//!
//! **插值口径**(放行 §1 二选一):连杆位姿**直接取最新**(不插值)。主体
//! 服务器流默认 10 拍一帧,60 FPS 渲染下插值收益有限,取最新实现简单且
//! HUD 显示与画面对齐(本模块文档即放行要求的文档注释)。

use std::collections::VecDeque;

use types::EntityId;
use types::glam::Vec3;

/// 附身模式下的水平行走速度(m/s;放行 §2 表)。
pub const WALK_SPEED: f32 = 4.3;
/// 附身模式下的 Shift 行走速度(m/s;放行 §2 表)。
pub const SPRINT_SPEED: f32 = 5.6;
/// 意图节流间隔(放行 §2:每 50 ms 最多一条)。
pub const INTENT_INTERVAL: std::time::Duration = std::time::Duration::from_millis(50);
/// 第三人称相机默认距离(m;滚轮范围 2~10)。
pub const CAMERA_DIST_DEFAULT: f32 = 4.0;
pub const CAMERA_DIST_MIN: f32 = 2.0;
pub const CAMERA_DIST_MAX: f32 = 10.0;
/// 第三人称相机看向身体根位置上方的高度(m;放行 §2)。
pub const LOOK_HEIGHT: f32 = 0.6;
/// `SpawnRequested`/`PossessRequested` 的等待上限:超过即视请求被拒,回退
/// (R1 §3.2 转移表 tick 行)。
pub const REQUEST_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(2);
/// 回执去重窗口容量:最近收到的 `Spawned` 回执最多留 64 条。
const RECEIPTS_CAP: usize = 64;

/// 一帧的按键快照(渲染层采集;纯逻辑不摸输入后端)。
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Keys {
    pub w: bool,
    pub a: bool,
    pub s: bool,
    pub d: bool,
    pub shift: bool,
    /// Space 本帧**物理按住**(按下沿由状态机逐帧 diff 得出:一次按键
    /// 一跳;按住不连发)。
    pub jump_down: bool,
}

/// 意图映射(放行 §2):WASD + 相机 yaw → 世界系水平速度方向与相机水平
/// 前向一致(误差 ≤ 1e-5,判据 A06-2)。yaw 单位**度**,与 viewer 飞行
/// 相机同约定(`forward = (cos yaw, 0, sin yaw)`)。
///
/// 无方向键按下 → 零速度(松键发零);Shift → 5.6,否则 4.3(速度乘
/// 单位方向;无方向时 Shift 无效果)。
pub fn intent_from_keys(keys: &Keys, yaw_deg: f32) -> (f32, f32) {
    let yaw = yaw_deg.to_radians();
    let (sy, cy) = yaw.sin_cos();
    // 前向 = (cy, sy);右 = 前 × Y = (−cz, 0, cx) = (−sy, cy)(飞行相机
    // 同一套向量代数,见 git 历史注释)。
    let fwd = (cy, sy);
    let right = (-sy, cy);
    let mut dx = 0.0_f32;
    let mut dz = 0.0_f32;
    if keys.w {
        dx += fwd.0;
        dz += fwd.1;
    }
    if keys.s {
        dx -= fwd.0;
        dz -= fwd.1;
    }
    if keys.d {
        dx += right.0;
        dz += right.1;
    }
    if keys.a {
        dx -= right.0;
        dz -= right.1;
    }
    let len = (dx * dx + dz * dz).sqrt();
    if len <= 0.0 {
        return (0.0, 0.0);
    }
    let speed = if keys.shift { SPRINT_SPEED } else { WALK_SPEED };
    (dx / len * speed, dz / len * speed)
}

/// 附身状态机的命令(渲染层转发给 [`crate::client::ControlHandle`])。
#[derive(Debug, Clone, PartialEq)]
pub enum PossessCommand {
    /// 生成身体(`P` 在屏幕中心拾取的地面格顶面上方;朝向 = 相机水平朝向)。
    SpawnAgent { position: Vec3, yaw: f32 },
    /// 附身某实体(自动:自己的身体出现在主体流;手动:P 有身体未附身)。
    Possess(EntityId),
    /// 解除附身(已附身时按 P)。
    Unpossess,
    /// 主体意图(保持状态;有变化才发,节流 50 ms;跳跃立即发)。
    Intent {
        move_xz: (f32, f32),
        yaw: f32,
        jump: bool,
    },
}

/// 状态机相位(R1 §3.2 转移表)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// 没有身体。
    NoBody,
    /// 已发 SpawnAgent,等身体出现。
    SpawnRequested,
    /// 有身体,未附身。
    HasBody,
    /// 已发 Possess。
    PossessRequested,
    /// 已附身。
    Possessed,
}

/// 附身状态机(交互模式一份;脚本模式不经此——脚本动作直发协议)。
#[derive(Debug)]
pub struct PossessState {
    phase: Phase,
    /// 我的身体(归属规则:收到过回执 + 出现在主体流)。
    my_body: Option<EntityId>,
    /// 最近收到的 `Spawned` 回执,最多留 64 条(去重窗口)。
    receipts: VecDeque<EntityId>,
    /// 身体出现后是否自动发 Possess(P 生成路径置位)。
    auto_possess: bool,
    /// `SpawnRequested` 的进入时刻(tick 超时用)。
    requested_at: Option<std::time::Instant>,
    /// 上一帧的意图(变化检测;None = 尚未发过)。
    last_intent: Option<((f32, f32), f32)>,
    /// 上次意图发送的时刻(节流 50 ms;None = 未发过,首条立即可发)。
    last_sent: Option<std::time::Instant>,
    /// 第三人称相机距离(m;滚轮调,2~10,缺省 4)。
    camera_dist: f32,
    /// 上一帧 Space 物理态(jump 按下沿 = 本帧按住 && 上一帧未按住)。
    prev_jump_down: bool,
}

impl Default for PossessState {
    fn default() -> Self {
        Self::new()
    }
}

impl PossessState {
    pub fn new() -> Self {
        Self {
            phase: Phase::NoBody,
            my_body: None,
            receipts: VecDeque::new(),
            auto_possess: false,
            requested_at: None,
            last_intent: None,
            last_sent: None,
            camera_dist: CAMERA_DIST_DEFAULT,
            prev_jump_down: false,
        }
    }

    /// 当前相位(HUD 用)。
    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// 我的身体 ID(归属规则确定;没有 = None)。
    pub fn my_body(&self) -> Option<EntityId> {
        self.my_body
    }

    /// 当前附身的主体(`Phase::Possessed` 时 = `my_body`)。
    pub fn possessed(&self) -> Option<EntityId> {
        match self.phase {
            Phase::Possessed => self.my_body,
            _ => None,
        }
    }

    /// 第三人称相机距离。
    pub fn camera_dist(&self) -> f32 {
        self.camera_dist
    }

    /// 滚轮调距离(±每档 1 m;夹 2~10)。
    pub fn adjust_camera_dist(&mut self, wheel: f32) {
        if wheel == 0.0 {
            return;
        }
        self.camera_dist =
            (self.camera_dist - wheel.signum()).clamp(CAMERA_DIST_MIN, CAMERA_DIST_MAX);
    }

    /// `Spawned` 回执:记入去重窗口(容量 64,满则挤掉最旧)。归属判定在
    /// [`Self::on_agents`](回执 + 主体流两个证据凑齐才算我的身体)。
    pub fn on_spawned(&mut self, id: EntityId) {
        if self.receipts.len() >= RECEIPTS_CAP {
            self.receipts.pop_front();
        }
        self.receipts.push_back(id);
    }

    /// 主体流更新(`ViewEvent::Agents`;present = 本帧可见的主体 ID,
    /// removed = 本帧被移除的):归属判定 + 转移。
    ///
    /// - present 里有 id ∈ receipts → 那是我的身体(从 receipts 删掉,防止
    ///   同 ID 重复判定):`auto_possess` 为真发 Possess、清标志、→
    ///   `PossessRequested`;否则 → `HasBody`。例外:该 ID 已被
    ///   `Possessed(Some)` 确认(服务器主动重生且回执先于主体流到达)→
    ///   只清 receipts,保持 `Possessed`(on_possessed(Some) 的"任何状态
    ///   → Possessed"不被主体流事件回退)。
    /// - removed 含 `my_body` → `my_body = None`,→ `NoBody`(等待服务器
    ///   的 `Possessed(None)` 做相机侧收尾;本方法只管归属)。
    pub fn on_agents(&mut self, present: &[EntityId], removed: &[EntityId]) -> Vec<PossessCommand> {
        let mut commands = Vec::new();
        for id in removed {
            if self.my_body == Some(*id) {
                self.my_body = None;
                // 附身中被移除:等 Possessed(None) 收尾(相机、意图状态);
                // 未附身有身体:直接没身体。两种都 → NoBody。
                self.phase = Phase::NoBody;
                self.auto_possess = false;
                self.requested_at = None;
            }
        }
        for id in present {
            if let Some(pos) = self.receipts.iter().position(|r| r == id) {
                self.receipts.remove(pos);
                if self.phase == Phase::Possessed && self.my_body == Some(*id) {
                    // 归属已由 Possessed(Some) 确认(见上例外)。
                    continue;
                }
                self.my_body = Some(*id);
                self.requested_at = None;
                if self.auto_possess {
                    self.auto_possess = false;
                    self.phase = Phase::PossessRequested;
                    commands.push(PossessCommand::Possess(*id));
                } else if matches!(self.phase, Phase::SpawnRequested | Phase::NoBody) {
                    self.phase = Phase::HasBody;
                }
            }
        }
        commands
    }

    /// `Possessed` 回执(渲染层透传;任何相位都消费):
    /// - `Some(x)`:服务器确认附身 → `my_body = x`、→ `Possessed`;意图
    ///   节流状态清零(下一条意图立即发)。
    /// - `None`:回飞行相机——`my_body` 还在 → `HasBody`,否则 → `NoBody`;
    ///   意图状态清零。
    pub fn on_possessed(&mut self, entity: Option<EntityId>) {
        self.last_intent = None;
        self.last_sent = None;
        match entity {
            Some(x) => {
                self.my_body = Some(x);
                self.phase = Phase::Possessed;
                self.requested_at = None;
            }
            None => {
                self.phase = if self.my_body.is_some() {
                    Phase::HasBody
                } else {
                    Phase::NoBody
                };
                self.requested_at = None;
                self.auto_possess = false;
            }
        }
    }

    /// `P` 按下沿(R1 §3.2 转移表):
    /// - `NoBody` + 有 spawn_target → 发 SpawnAgent、`auto_possess = true`、
    ///   → `SpawnRequested`;
    /// - `HasBody` → 发 Possess(my_body)、→ `PossessRequested`;
    /// - `Possessed` → 发 Unpossess(状态等服务器的 `Possessed(None)` 再变);
    /// - 其他(SpawnRequested / PossessRequested / 无目标)→ 忽略。
    pub fn on_p_pressed(
        &mut self,
        spawn_target: Option<(Vec3, f32)>,
        now: std::time::Instant,
    ) -> Vec<PossessCommand> {
        match self.phase {
            Phase::NoBody => match spawn_target {
                Some((position, yaw)) => {
                    self.auto_possess = true;
                    self.requested_at = Some(now);
                    self.phase = Phase::SpawnRequested;
                    vec![PossessCommand::SpawnAgent { position, yaw }]
                }
                None => Vec::new(),
            },
            Phase::HasBody => match self.my_body {
                Some(body) => {
                    self.requested_at = Some(now);
                    self.phase = Phase::PossessRequested;
                    vec![PossessCommand::Possess(body)]
                }
                None => Vec::new(),
            },
            Phase::Possessed => vec![PossessCommand::Unpossess],
            Phase::SpawnRequested | Phase::PossessRequested => Vec::new(),
        }
    }

    /// 已附身时每帧:意图映射(变化才发、50 ms 节流、跳跃按下沿立即发、
    /// 松键发零)。未附身 → 空。节流用传入的 `now`。
    pub fn on_keys(
        &mut self,
        keys: &Keys,
        yaw_deg: f32,
        now: std::time::Instant,
    ) -> Vec<PossessCommand> {
        if self.possessed().is_none() {
            self.prev_jump_down = keys.jump_down;
            return Vec::new();
        }
        let intent = intent_from_keys(keys, yaw_deg);
        let changed = match self.last_intent {
            None => true,
            Some((prev_mv, prev_yaw)) => prev_mv != intent || (prev_yaw - yaw_deg).abs() > 1e-4,
        };
        let throttled_ok = match self.last_sent {
            None => true,
            Some(at) => now.duration_since(at) >= INTENT_INTERVAL,
        };
        let jump_now = keys.jump_down && !self.prev_jump_down;
        self.prev_jump_down = keys.jump_down;
        if (changed && throttled_ok) || jump_now {
            let command = PossessCommand::Intent {
                move_xz: intent,
                yaw: yaw_deg,
                jump: jump_now,
            };
            self.last_intent = Some((intent, yaw_deg));
            self.last_sent = Some(now);
            return vec![command];
        }
        Vec::new()
    }

    /// 每帧推进(R1 §3.2 转移表 tick 行):请求等待超过 2 s 视为被拒——
    /// `SpawnRequested` → `NoBody`(清 auto_possess);`PossessRequested`
    /// → `HasBody`。R2-5:自动附身发出的 Possess(`on_agents` 路径,那里
    /// 拿不到 `now`)`requested_at` 为 None——在这里补起算(从这一帧开始
    /// 计时),否则服务器不回时永远停在 `PossessRequested`,`P` 键失效。
    pub fn tick(&mut self, now: std::time::Instant) {
        if self.requested_at.is_none()
            && matches!(self.phase, Phase::SpawnRequested | Phase::PossessRequested)
        {
            self.requested_at = Some(now);
        }
        let Some(at) = self.requested_at else {
            return;
        };
        if now.duration_since(at) < REQUEST_TIMEOUT {
            return;
        }
        match self.phase {
            Phase::SpawnRequested => {
                self.phase = Phase::NoBody;
                self.auto_possess = false;
                self.requested_at = None;
            }
            Phase::PossessRequested => {
                self.phase = Phase::HasBody;
                self.requested_at = None;
            }
            _ => self.requested_at = None,
        }
    }
}

/// 第三人称相机位姿(A06 §2):看向身体根位置上方 0.6 m,位于其后方当前
/// 距离处,抬高按 pitch。yaw/pitch 单位度(与飞行相机同约定);返回
/// (相机位置, 注视目标)。
pub fn third_person_camera(root: Vec3, yaw_deg: f32, pitch_deg: f32, dist: f32) -> (Vec3, Vec3) {
    let yaw = yaw_deg.to_radians();
    let pitch = pitch_deg.to_radians();
    let (sy, cy) = yaw.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    // 身体后方 = 前向的反方向;前向 = (cy·cp, sp, sy·cp)(飞行相机同式)。
    let fwd = Vec3::new(cy * cp, sp, sy * cp);
    let look = Vec3::new(root.x, root.y + LOOK_HEIGHT, root.z);
    let pos = look - fwd * dist;
    (pos, look)
}

/// 附身帧写回 `FlyCamera` 的相机位置(R1 §3.4 抽出的纯函数;MUT15 测试
/// 锚:渲染循环每帧用它写 `camera.position`,变异"不写回"在这里红)。
pub fn possessed_camera_position(root: Vec3, yaw_deg: f32, pitch_deg: f32, dist: f32) -> Vec3 {
    third_person_camera(root, yaw_deg, pitch_deg, dist).0
}

/// 身体没了、相机落在世界底下时的放回位置(R3 §6 修订:附身走远掉出
/// 世界后,相机跟到世界底下,准星下没有可拾取的地面,按 P 生成不了)。
/// - `current.y ≥ y_min` → 原样返回(主动按 P 退出附身时相机还在世界
///   高度以内,不受影响);
/// - 否则 `last_safe` 有值用它(附身期间记录的最后一次安全相机位),
///   没有就只把 y 抬到 `y_min + 16`。
pub fn camera_after_body_lost(current: Vec3, last_safe: Option<Vec3>, y_min: f32) -> Vec3 {
    if current.y >= y_min {
        return current;
    }
    match last_safe {
        Some(pos) => pos,
        None => Vec3::new(current.x, y_min + 16.0, current.z),
    }
}

/// 按 P 但发不出命令时给用户的提示(R3 §6 修订;能发命令或不需要提示
/// 时 None)。`NoBody` 且没有拾取目标 → 掉出世界后相机在世界底下时用户
/// 按 P 无任何反馈的兜底提示。
pub fn p_press_hint(phase: Phase, has_target: bool) -> Option<&'static str> {
    match phase {
        Phase::NoBody if !has_target => Some("aim at the ground, then press P"),
        _ => None,
    }
}

/// 输入路由表(R2-3):附身/非附身两态下,通用交互键各自是否仍生效。
/// 附身时全 false——空格是跳跃、滚轮是相机距离、WASD 是行走意图,原有
/// 的"空格切换耦合 / 滚轮调发射预设速度 / WASD 平移相机"必须让路(R1
/// 验收:附身时跳一下,退出后飞行相机不上报位置的根因就是空格把 `coupled`
/// 翻成了 false)。非附身时全 true(既有行为不变)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputRouting {
    /// 空格是否切换相机 ↔ 观察者耦合(附身时空格 = 跳跃,不切)。
    pub space_toggles_coupling: bool,
    /// 滚轮是否调发射预设速度(附身时滚轮 = 第三人称相机距离,不调)。
    pub wheel_sets_launch_speed: bool,
    /// WASD 是否平移飞行相机(附身时 WASD = 行走意图,不平移)。
    pub wasd_moves_camera: bool,
}

/// 按是否附身取输入路由:附身三项全 false,否则全 true。
pub fn input_routing(possessed: bool) -> InputRouting {
    let free = !possessed;
    InputRouting {
        space_toggles_coupling: free,
        wheel_sets_launch_speed: free,
        wasd_moves_camera: free,
    }
}

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
