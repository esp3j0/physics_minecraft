//! possess —— 附身状态机与意图映射(REQ-012 A06 §2)。纯逻辑、零 macroquad:
//! 渲染层每帧喂(按键快照、相机 yaw、服务器 Spawned 回执、Possessed 回执),
//! 取回(要发的协议命令、意图、第三人称相机位姿);任何输入不 panic。
//!
//! **回执对应口径**(放行 §2):`Spawned` 回执不带发起者,viewer 自己的
//! 方块 `Spawn`(Control::Spawn)与 `SpawnAgent`(Control::SpawnAgent)都
//! 会收到它——按**请求顺序**对应:`spawn_requests` 记录每条 `SpawnAgent`
//! 请求的序号;`Control::Spawn` 也占用回执序号。收到的第 k 条 `Spawned`
//! 对应第 k 条请求(两种请求共享一个全局序)。方块请求先入队序号小,不会被
//! 当成身体(`is_agent_request` 区分)。服务器拒绝(被拒不回包)会让序号
//! 错位——防御:附身请求发出前校验该 ID 在**主体流**里出现过(`agent_seen`),
//! 拿方块的 ID 附身会被服务器拒绝(不是主体),不会误附身。
//!
//! **插值口径**(放行 §1 二选一):连杆位姿**直接取最新**(不插值)。主体
//! 服务器流默认 10 拍一帧,60 FPS 渲染下插值收益有限,取最新实现简单且
//! HUD 显示与画面对齐(本模块文档即放行要求的文档注释)。

use std::collections::BTreeSet;

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
    // 前向 = (cy, sy);右向 = 前向 × (0,0,-1) 的水平右 = (−sy, cy)?
    // viewer 约定:flat_forward = (cos yaw, sin yaw)(x, z);右 = 前 × 下
    // 世界系。与飞行相机同式:right = flat_forward × Y 是左手系错误方向,
    // 飞行相机用 `flat_forward().cross(Y)`,实际渲染中 yaw 增大绕 y 逆时针。
    // 本函数与飞行相机同一套向量代数:A/S 前后沿 forward,D/A 右左沿
    // forward × (0,-1,0)??——直接对齐飞行相机的 update:
    //   move_dir = ±flat_forward ± right,right = flat_forward.cross(Y)。
    // macroquad Vec3::Y = (0,1,0);cross((cx,0,cz),(0,1,0)) = (0·0−cz·1,
    // cz·0−cx·0, cx·1−0·0) = (−cz, 0, cx)。所以 right = (−sz, 0, cx)。
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
    /// 附身某实体(收到 Spawned 且在主体流可见后自动发)。
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

/// 附身状态机(交互模式一份;脚本模式不经此——脚本动作直发协议)。
#[derive(Debug)]
pub struct PossessState {
    /// 当前附身的主体(None = 飞行相机/未附身)。
    possessed: Option<EntityId>,
    /// 已发出但未收到回执的 SpawnAgent 请求(按请求顺序;值 = 请求时的
    /// 相机水平 yaw,附身成功后第三人称相机初始朝向用——暂不消费,占位
    /// 供对应判定)。
    spawn_requests: Vec<f32>,
    /// 上一帧的意图(变化检测;None = 尚未发过)。
    last_intent: Option<((f32, f32), f32)>,
    /// 上次意图发送的墙钟(节流 50 ms;None = 未发过,首条立即可发)。
    last_sent: Option<std::time::Instant>,
    /// 主体流里见过的实体 ID(`AgentStates` 全量/增量累积;removed 清除)。
    /// 附身请求的前置校验:只对见过的 ID 发 Possess,方块 ID 不会被误附身。
    agent_seen: BTreeSet<EntityId>,
    /// 第三人称相机距离(m;滚轮调,2~10,缺省 4)。
    camera_dist: f32,
    /// 待自动附身(`P` 生成后,收到 Spawned 回执 + 主体流可见时发 Possess;
    /// 只发一次)。
    pending_auto_possess: bool,
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
            possessed: None,
            spawn_requests: Vec::new(),
            last_intent: None,
            last_sent: None,
            agent_seen: BTreeSet::new(),
            camera_dist: CAMERA_DIST_DEFAULT,
            pending_auto_possess: false,
            prev_jump_down: false,
        }
    }

    /// 当前附身的主体。
    pub fn possessed(&self) -> Option<EntityId> {
        self.possessed
    }

    /// 第三人称相机距离。
    pub fn camera_dist(&self) -> f32 {
        self.camera_dist
    }

    /// 自动附身待决(P 生成后、回执+主体流可见前的窗口;HUD 提示用)。
    pub fn auto_possess_pending(&self) -> bool {
        self.pending_auto_possess
    }

    /// 滚轮调距离(±每档 1 m;夹 2~10)。
    pub fn adjust_camera_dist(&mut self, wheel: f32) {
        if wheel == 0.0 {
            return;
        }
        self.camera_dist =
            (self.camera_dist - wheel.signum()).clamp(CAMERA_DIST_MIN, CAMERA_DIST_MAX);
    }

    /// 主体流更新(`AgentStates` 事件;渲染层透传):累积可见主体、removed
    /// 即删。被移除的主体若正附身,状态机**等待服务器的 Possessed(None)**
    /// 才回飞行相机(放行 §2:Possessed(None) 到达时回飞行相机——包括
    /// 服务器因身体被移除而发出的);本方法不自行解除。
    pub fn observe_agents(&mut self, states: &[EntityId], removed: &[EntityId]) {
        for id in states {
            self.agent_seen.insert(*id);
        }
        for id in removed {
            self.agent_seen.remove(id);
        }
    }

    /// `Spawned` 回执(按到达顺序;渲染层透传):消费一条 spawn 请求序号。
    /// 服务器对 SpawnAgent 成功必回 `Spawned`(放行 §2);方块请求也占序号。
    /// 回执不直接触发附身——`P` 按下的自动附身在 [`Self::on_spawn_receipt`]
    /// 里按"该 ID 已出现在主体流"校验后发 Possess。
    pub fn on_spawn_receipt(&mut self) {
        // 请求队列头弹出(空 = 服务器主动生成等历史回执,忽略)。
        self.spawn_requests.pop();
    }

    /// `Possessed` 回执:`Some` = 附身成功(登记);`None` = 回飞行相机
    /// (主动解除 / 服务器被迫解除,同一路径)。
    pub fn on_possessed(&mut self, entity: Option<EntityId>) {
        self.possessed = entity;
        if entity.is_none() {
            // 解除附身:意图状态清掉(回到飞行相机后不再发意图)。
            self.last_intent = None;
            self.last_sent = None;
        }
    }

    /// 每帧推进(放行 §2 表):
    /// - **未附身 + P 按下沿**:
    ///   - 没有身体:发 `SpawnAgent`(拾取格顶面上方;`spawn_target` 由渲染
    ///     层算好传入;朝向 = 相机水平 yaw)。收到 Spawned 且主体流可见后
    ///     **自动 Possess**(本方法内直接登记待附身,回执同步发)。
    ///   - 有身体但未附身:附身它(`agent_seen` 里的自己身体 = 上次 Spawned
    ///     且可见的;由渲染层传入身体 ID)。
    /// - **已附身 + P 按下沿**:发 `Unpossess`(回执到了才真正回飞行相机)。
    /// - 已附身:意图映射(`intent_from_keys`)+ 节流(50 ms;变化才发;
    ///   松开所有键发一次零速度;跳跃按下沿立即发一次)。
    ///
    /// `self_agent` = 未附身时的身体 ID(渲染层从主体表找 source 自己的;
    /// viewer 无法区分"哪个主体是我的"——服务器只对无人附身的本体放行
    /// Possess,所以这里对**任意已知主体**尝试即可:没有身体时 P 的自动
    /// 附身路径已保证第一个主体就是自己的)。简化口径:未附身时 P 优先
    /// 附身 `self_agent` 参数指定的主体。
    pub fn frame(
        &mut self,
        p_pressed: bool,
        keys: &Keys,
        yaw_deg: f32,
        spawn_target: Option<(Vec3, f32)>,
        self_agent: Option<EntityId>,
    ) -> Vec<PossessCommand> {
        let mut commands = Vec::new();
        if p_pressed {
            match self.possessed {
                None => {
                    if let Some(body) = self_agent {
                        // 有身体(自己此前生成的,或重生回来的)但未附身:附身。
                        if self.agent_seen.contains(&body) {
                            commands.push(PossessCommand::Possess(body));
                        }
                    } else if let Some((position, yaw)) = spawn_target {
                        // 没有身体:生成。朝向 = 相机水平朝向。
                        commands.push(PossessCommand::SpawnAgent { position, yaw });
                        self.spawn_requests.push(yaw);
                        self.pending_auto_possess = true;
                    }
                }
                Some(_) => commands.push(PossessCommand::Unpossess),
            }
        }
        if self.possessed.is_some() {
            // 意图:变化才发,50 ms 节流;跳跃按下沿立即发(一次按键一跳)。
            let intent = intent_from_keys(keys, yaw_deg);
            let changed = match self.last_intent {
                None => true,
                Some((prev_mv, prev_yaw)) => prev_mv != intent || (prev_yaw - yaw_deg).abs() > 1e-4,
            };
            #[allow(clippy::disallowed_methods)] // 墙钟节流(渲染侧;边缘 crate 局部授权)
            let throttled_ok = match self.last_sent {
                None => true,
                Some(at) => at.elapsed() >= INTENT_INTERVAL,
            };
            let jump_now = keys.jump_down && !self.prev_jump_down;
            self.prev_jump_down = keys.jump_down;
            if (changed && throttled_ok) || jump_now {
                commands.push(PossessCommand::Intent {
                    move_xz: intent,
                    yaw: yaw_deg,
                    jump: jump_now,
                });
                self.last_intent = Some((intent, yaw_deg));
                #[allow(clippy::disallowed_methods)]
                {
                    self.last_sent = Some(std::time::Instant::now());
                }
            }
        }
        commands
    }
}

impl PossessState {
    /// 渲染层每帧在 [`Self::frame`] 之后调用:`pending_auto_possess` 且生成
    /// 的实体已在主体流可见 → 发 Possess(自动附身;只发一次,回执
    /// Possessed(Some) 到达即登记)。`self_agent` 口径:渲染层把"主体表中
    /// 无人附身、且最近 Spawned 的 ID"传入;这里发 Possess 该 ID。
    pub fn try_auto_possess(&mut self, candidate: Option<EntityId>) -> Option<EntityId> {
        if !self.pending_auto_possess {
            return None;
        }
        let id = candidate?;
        if !self.agent_seen.contains(&id) {
            return None;
        }
        self.pending_auto_possess = false;
        Some(id)
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

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
