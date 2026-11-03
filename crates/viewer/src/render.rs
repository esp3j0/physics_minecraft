//! render —— macroquad 主循环(窗口/相机/HUD;REQ-004A §2;REQ-008 §4 脚本模式)。
//! **不写窗口类测试**(README 决策 2;ACV4 手动清单,禁造假断言)——本文件仅含装配
//! 编排,纯算力全部在 mesh(有单测)、client(零 macroquad)、world/cli/manifest
//! (REQ-008 纯逻辑层,各有单测)。

use std::collections::BTreeMap;
use std::net::SocketAddr;
use std::path::Path;
use std::time::Instant;

use macroquad::camera::{Camera3D, Projection, set_camera, set_default_camera};
use macroquad::color::{BLUE, DARKGRAY, GRAY, ORANGE, RED, SKYBLUE, WHITE, YELLOW};
use macroquad::input::{
    KeyCode, MouseButton, is_key_down, is_key_pressed, is_mouse_button_pressed,
    mouse_delta_position, mouse_wheel, set_cursor_grab, show_mouse,
};
use macroquad::math::vec3;
use macroquad::models::{Mesh, draw_cube, draw_cube_wires, draw_mesh};
use macroquad::text::draw_text;
use macroquad::time::{get_fps, get_frame_time};
use macroquad::window::{clear_background, next_frame, screen_height, screen_width};
use messaging::SimCommand;
use types::glam::Vec3 as TVec3;
use types::{BlockPos, ChunkPos, EntityDesc, EntityId, EntityState};
// 注:macroquad 0.4 内嵌 glam 0.27,types 用 glam 0.33——两版 Vec3 不互认,
// 一切跨界处按字段换算(下文 vec3(t.x, t.y, t.z) / types::glam::Vec3::new)。

use crate::cli::{ViewerArgs, ViewerMode};
use crate::client::{ViewEvent, ViewerClient};
use crate::mesh::chunk_mesh;
use crate::script::WorldQuery;
use crate::script::manifest::Manifest;
use crate::script::{
    CameraPose, Engine, EngineCommand, EngineStatus, FrameInput, ScreenshotRequest,
};
use crate::tools::{
    ChargeState, Hit, PanelColor, PanelState, Placeable, StageDriver, ToolAction, ToolState,
    adjust_preset_speed, cycle_placeable, dig, help_lines, launch_request, orientation_from_yaw,
    panel_lines, pick, place, preview_outline, speed_bar, toggle_help,
};
use crate::world::WorldState;

/// 相机移动速度(格/秒)。
const MOVE_SPEED: f32 = 16.0;
/// 视角灵敏度(rad/像素;反馈 1 定 0.003 → 反馈 2 提到 0.008 → 反馈 5
/// 用户实测仍太慢,×10。一行可调)。
const LOOK_SENSITIVITY: f32 = 0.08;
/// MoveObserver 节流间隔(≥100ms 一次,任务书接口设计)。
const OBSERVER_SEND_INTERVAL: std::time::Duration = std::time::Duration::from_millis(100);
/// 实体立方体边长(约一格;EntityDesc::default half_size 0.5 的直觉呈现)。
const ENTITY_CUBE_SIZE: f32 = 0.9;
/// 断连后自动退出的等待(秒;给用户看清 HUD 提示)。
const DISCONNECT_GRACE_SECS: f32 = 3.0;
/// 交互命令的目标距离(米):B 生成点 = 相机前 8m 抬 6m;X 爆心 = 相机前 12m。
const SPAWN_AHEAD: f32 = 8.0;
const SPAWN_LIFT: f32 = 6.0;
const BLAST_AHEAD: f32 = 12.0;
/// X 爆炸参数(power 12 与 e2e 同量级;radius 8 覆盖瞄准误差;爆心 y 固定
/// 在地表平面——见 X 键注释)。
const BLAST_POWER: f32 = 12.0;
const BLAST_RADIUS: f32 = 8.0;
/// 超平坦地表顶面(world y)= WORLD_Y_MIN + 4(REQ-008 §4 顺带修正:旧值按
/// y = 4 写死,生产超平坦地表顶面在 −60;交互与脚本模式共用)。
const GROUND_TOP_Y: f32 = (types::WORLD_Y_MIN + 4) as f32;
/// 爆心高度(地表顶面 + 0.7 ≈ 落地立方体中心;REQ-008 §4 顺带修正)。
const BLAST_Y: f32 = GROUND_TOP_Y + 0.7;
/// X 爆心吸附范围(米):相机附近最近的活实体直接作爆心(见 X 键注释)。
const BLAST_SNAP: f32 = 25.0;
/// 脚本模式观察者上报的立即触发阈值:相机较上次上报移动超过 1 m 即上报
/// (不等 100 ms 节流——传送当帧即可带动观察者;REQ-008 单元 02 任务书)。
const OBSERVER_JUMP_METERS: f32 = 1.0;

/// 拾取最远距离(REQ-011 §3.3)。
const PICK_MAX_DIST: f32 = 48.0;

/// 面板行颜色 → macroquad 颜色(§3.5.3 第 4 条:断开行红、标题黄、提示橙、
/// 其余白;纯函数层 [`PanelColor`] 的唯一映射点)。
fn panel_color(color: PanelColor) -> macroquad::color::Color {
    match color {
        PanelColor::Normal => WHITE,
        PanelColor::Break => RED,
        PanelColor::Header => YELLOW,
        PanelColor::Notice => ORANGE,
    }
}

/// 工具动作发出(纯函数层 [`ToolAction`] → 控制句柄):Single 走
/// `command`,Batch 打一条批(REQ-010 终审返修 1 原子编辑语义),Rejected
/// 写入工具提示(3 秒)。
fn apply_tool_action(
    action: ToolAction,
    tools: &mut ToolState,
    control: &crate::client::ControlHandle,
) {
    match action {
        ToolAction::Single(command) => control.command(command),
        ToolAction::Batch(commands) => control.command_batch(commands),
        ToolAction::Rejected(reason) => tools.reject(reason),
    }
}

/// 画一格线框(目标格高亮 / 预制轮廓逐格;格 (x,y,z) 覆盖 [x,x+1)³)。
fn draw_cell_wire(pos: BlockPos, color: macroquad::color::Color) {
    draw_cube_wires(
        vec3(pos.x as f32 + 0.5, pos.y as f32 + 0.5, pos.z as f32 + 0.5),
        vec3(1.0, 1.0, 1.0),
        color,
    );
}

/// 拾取(每帧;相机位姿 → 3D 线框用命中)。零 macroquad 依赖的纯函数在
/// tools::pick,这里只做 glam 版本换算。
fn pick_at_camera(camera: &FlyCamera, world: &WorldState) -> Option<Hit> {
    let o = camera.position;
    let d = camera.forward();
    pick(
        world,
        TVec3::new(o.x, o.y, o.z),
        TVec3::new(d.x, d.y, d.z),
        PICK_MAX_DIST,
    )
}

/// 交互模式的飞行相机初始位姿(地表上方约 12 m;REQ-008 §4 顺带修正:旧值
/// (8, 12, 24) 按 y=4 地表写死)。脚本模式以修正后的同一位姿为起始相机。
fn start_camera_pos() -> macroquad::math::Vec3 {
    vec3(8.0, GROUND_TOP_Y + 12.0, 24.0)
}

/// 自由飞行相机(欧拉:偏航绕 y,俯仰;WASD + 鼠标视角,Shift/Ctrl 升降)。
struct FlyCamera {
    position: macroquad::math::Vec3,
    yaw: f32,
    pitch: f32,
}

impl FlyCamera {
    /// 初始位姿:地表上方约 12 m,俯视地面(超平坦顶面 = WORLD_Y_MIN + 4)。
    fn new() -> Self {
        Self {
            position: start_camera_pos(),
            yaw: -90.0_f32.to_radians(),
            pitch: -30.0_f32.to_radians(),
        }
    }

    /// 前向单位向量(俯仰参与)。
    fn forward(&self) -> macroquad::math::Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        vec3(cy * cp, sp, sy * cp)
    }

    /// 水平前向(俯仰不参与;WASD 平移基准)。
    fn flat_forward(&self) -> macroquad::math::Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        vec3(cy, 0.0, sy)
    }

    /// 每帧推进(鼠标视角 + WASD 平移 + Shift/Ctrl 升降;dt 秒)。
    fn update(&mut self, dt: f32) {
        let delta = mouse_delta_position();
        // 水平:鼠标右移 → 视角右转(常规 FPS 方向;反馈 1 翻转符号)。
        // 俯仰:鼠标上移 → 抬头——macroquad 的 delta.y 实测向上为正
        // (与屏幕坐标直觉相反),故 +=;反馈 1 版本的 -= 使上滑低头
        // (反馈 5 修正)。
        self.yaw -= delta.x * LOOK_SENSITIVITY;
        self.pitch = (self.pitch + delta.y * LOOK_SENSITIVITY)
            .clamp(-89.0_f32.to_radians(), 89.0_f32.to_radians());

        let right = self.flat_forward().cross(macroquad::math::Vec3::Y);
        let mut move_dir = macroquad::math::Vec3::ZERO;
        if is_key_down(KeyCode::W) {
            move_dir += self.flat_forward();
        }
        if is_key_down(KeyCode::S) {
            move_dir -= self.flat_forward();
        }
        if is_key_down(KeyCode::D) {
            move_dir += right;
        }
        if is_key_down(KeyCode::A) {
            move_dir -= right;
        }
        if is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift) {
            move_dir += macroquad::math::Vec3::Y;
        }
        if is_key_down(KeyCode::LeftControl) || is_key_down(KeyCode::RightControl) {
            move_dir -= macroquad::math::Vec3::Y;
        }
        if move_dir != macroquad::math::Vec3::ZERO {
            self.position += move_dir.normalize_or_zero() * MOVE_SPEED * dt;
        }
    }

    /// → macroquad 相机(每帧现造;demo 定位无状态复用)。
    fn to_camera(&self) -> Camera3D {
        Camera3D {
            position: self.position,
            target: self.position + self.forward(),
            up: macroquad::math::Vec3::Y,
            fovy: 70.0_f32.to_radians(),
            projection: Projection::Perspective,
            ..Camera3D::default()
        }
    }

    /// → 脚本引擎的位姿(types::glam 字段换算;角度转度)。
    fn to_pose(&self) -> CameraPose {
        CameraPose {
            pos: types::glam::Vec3::new(self.position.x, self.position.y, self.position.z),
            yaw: self.yaw.to_degrees(),
            pitch: self.pitch.to_degrees(),
        }
    }

    /// 从脚本引擎的位姿设置(角度为度)。
    fn set_from_pose(&mut self, pose: &CameraPose) {
        self.position = vec3(pose.pos.x, pose.pos.y, pose.pos.z);
        self.yaw = pose.yaw.to_radians();
        self.pitch = pose.pitch.to_radians();
    }
}

/// HUD 文本行(HUD 四行:tick/chunks/entities/FPS)。
fn hud_lines(tick: u64, chunks: usize, entities: usize, fps: i32) -> [String; 4] {
    [
        format!("tick: {tick}"),
        format!("chunks: {chunks}"),
        format!("entities: {entities}"),
        format!("fps: {fps}"),
    ]
}

/// 实体渲染位姿:服务器流(默认 10 tick 一帧)只推进 target,渲染帧对
/// current 指数逼近——阶梯更新被平滑成连续运动(下落不"一格一格跳")。
/// 旋转同样按 slerp 逼近 target(复合体展开各格的方向,REQ-010 §10 修订 4;
/// 时间常数沿用 `POSE_SMOOTH`)。
struct EntityPose {
    current: macroquad::math::Vec3,
    target: macroquad::math::Vec3,
    current_rot: macroquad::math::Quat,
    target_rot: macroquad::math::Quat,
}

impl EntityPose {
    fn new(target: macroquad::math::Vec3, target_rot: macroquad::math::Quat) -> Self {
        Self {
            current: target,
            target,
            current_rot: target_rot,
            target_rot,
        }
    }

    /// 渲染帧插值:位置线性指数逼近、旋转 slerp 指数逼近(帧率无关,t 因子
    /// 同 `POSE_SMOOTH`)。
    fn advance(&mut self, k: f32) {
        self.current += (self.target - self.current) * k;
        self.current_rot = self.current_rot.slerp(self.target_rot, k.clamp(0.0, 1.0));
    }
}

/// 位置插值速率(1/s;越大越紧跟,14 ≈ 70ms 追平大半;旋转 slerp 同款因子)。
const POSE_SMOOTH: f32 = 14.0;

/// 世界渲染态(REQ-008 单元 02 重构):纯逻辑的 [`WorldState`(方块镜像/实体
/// 服务器位置/拍号/回执)] + 渲染侧的 Mesh 表与实体插值表。每帧用
/// `take_dirty_chunks` 同步 Mesh,实体插值表跟随 `live_entities`。
struct WorldView {
    /// 纯逻辑世界状态(脚本引擎经 WorldQuery 读它)。
    state: WorldState,
    /// chunk Mesh 表(脏 chunk 在则重建、不在则删除)。
    meshes: BTreeMap<ChunkPos, Mesh>,
    /// 实体插值表(跟随 state 的实体表;新实体从目标位姿起步、消失即删)。
    entities: BTreeMap<EntityId, EntityPose>,
}

impl WorldView {
    fn new() -> Self {
        Self {
            state: WorldState::new(),
            meshes: BTreeMap::new(),
            entities: BTreeMap::new(),
        }
    }

    /// chunk 世界原点(chunk 网格 → 世界坐标;y 全高,局部 0 = WORLD_Y_MIN)。
    fn origin_of(pos: ChunkPos) -> (f32, f32, f32) {
        (
            (pos.x * 16) as f32,
            types::WORLD_Y_MIN as f32,
            (pos.z * 16) as f32,
        )
    }

    /// 应用一个事件(增删改;Disconnected 由各主循环判终)。
    fn apply(&mut self, event: ViewEvent) {
        self.state.apply(event);
        self.sync_entities();
    }

    /// 每帧收尾:脏 chunk → Mesh 重建/删除;实体插值表对齐实体表。
    /// (事件路径只改 state 与插值表;Mesh 同步集中在这一处。)
    fn sync(&mut self) {
        for pos in self.state.take_dirty_chunks() {
            match self.state.chunk(pos) {
                Some(chunk) => {
                    self.meshes
                        .insert(pos, chunk_mesh(chunk, Self::origin_of(pos)));
                }
                None => {
                    self.meshes.remove(&pos);
                }
            }
        }
    }

    /// 实体插值表跟随 live_entities:新实体 current 从 target 起步、消失即删、
    /// 已有实体只推进 target(位置与服务器旋转,渲染帧插值逼近)。
    fn sync_entities(&mut self) {
        let live = self.state.live_entities();
        let present: std::collections::BTreeSet<EntityId> =
            live.iter().map(|(id, _)| *id).collect();
        self.entities.retain(|id, _| present.contains(id));
        for (id, pos) in live {
            let target = vec3(pos.x, pos.y, pos.z);
            let rot = mq_quat(self.state.entity_rotation(id));
            self.entities
                .entry(id)
                .and_modify(|pose| {
                    pose.target = target;
                    pose.target_rot = rot;
                })
                .or_insert_with(|| EntityPose::new(target, rot));
        }
    }

    /// 画一个实体(REQ-010 §10 修订 4 第 4 条):复合体按形状逐格画 0.9 m
    /// 蓝色立方体(轴对齐,与单格一致),位置 = 插值原点 + 插值旋转 × 偏移;
    /// 单格(形状未知)画插值原点一个立方体,与改前逐位一致。
    fn draw_entity(&self, id: EntityId, pose: &EntityPose) {
        match self.state.entity_shape(id) {
            None => draw_cube(
                pose.current,
                vec3(ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE),
                None,
                BLUE,
            ),
            Some(shape) => {
                for cell in shape.cells() {
                    let [x, y, z] = cell.offset;
                    let center =
                        pose.current + pose.current_rot * vec3(x as f32, y as f32, z as f32);
                    draw_cube(
                        center,
                        vec3(ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE),
                        None,
                        BLUE,
                    );
                }
            }
        }
    }
}

/// types glam 0.33 Quat → macroquad glam 0.27 Quat(字段换算;跨界纪律同 Vec3)。
fn mq_quat(q: types::glam::Quat) -> macroquad::math::Quat {
    macroquad::math::Quat::from_xyzw(q.x, q.y, q.z, q.w)
}

/// 主循环入口(main 调用):按模式分发。服务器地址优先级:
/// `--addr` > 环境变量 `VIEWER_ADDR` > `127.0.0.1:25565`;视距 = `VIEWER_VIEW`
/// 环境变量(缺省 4,REQ-004A §2)或场景 `[viewer] view_distance`。
pub async fn run(args: ViewerArgs) {
    let addr_str = args
        .addr
        .clone()
        .or_else(|| std::env::var("VIEWER_ADDR").ok())
        .unwrap_or_else(|| "127.0.0.1:25565".to_owned());
    let addr: SocketAddr = addr_str.parse().unwrap_or_else(|e| {
        eprintln!("服务器地址无效({addr_str}):{e}");
        std::process::exit(2);
    });
    let env_view = std::env::var("VIEWER_VIEW")
        .ok()
        .and_then(|v| v.parse::<u8>().ok());
    match args.mode {
        ViewerMode::Interactive => {
            run_viewer(addr, env_view.unwrap_or(4), None).await;
        }
        ViewerMode::Script { script, out } => {
            run_script(addr, env_view, &script, &out).await;
        }
        // REQ-011 §3.6(W05):布景交互 = 交互模式 + 场景文件。
        ViewerMode::Stage { script } => {
            run_viewer(addr, env_view.unwrap_or(4), Some(script.as_path())).await;
        }
    }
}

/// 布景模式的场景前置(非窗口部分):读文件、解析;失败 → 标准错误打印
/// 原因,退出码 2(同脚本模式口径)。不创建输出目录(布景无截图输出)。
fn prepare_stage(script: &Path) -> crate::script::Scenario {
    let text = match std::fs::read_to_string(script) {
        Ok(text) => text,
        Err(e) => {
            eprintln!("读取场景文件 {} 失败:{e}", script.display());
            std::process::exit(2);
        }
    };
    match crate::script::parse_scenario(&text) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("场景解析失败:{e}");
            std::process::exit(2);
        }
    }
}

/// 脚本模式启动前置(非窗口部分):读场景、解析、创建输出目录。
/// 失败 → 标准错误打印原因,退出码 2(REQ-008 §4)。
fn prepare_script(
    script: &Path,
    out: &Path,
) -> (crate::script::Scenario, crate::script::manifest::Manifest) {
    let text = match std::fs::read_to_string(script) {
        Ok(text) => text,
        Err(e) => {
            eprintln!("读取场景文件 {} 失败:{e}", script.display());
            std::process::exit(2);
        }
    };
    let scenario = match crate::script::parse_scenario(&text) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("场景解析失败:{e}");
            std::process::exit(2);
        }
    };
    if let Err(e) = std::fs::create_dir_all(out) {
        eprintln!("创建输出目录 {} 失败:{e}", out.display());
        std::process::exit(2);
    }
    let manifest = Manifest::new(&scenario);
    (scenario, manifest)
}

/// manifest 写盘(每条步骤记录后与收尾时重写;进程意外退出也留下已完成部分)。
fn write_manifest(manifest: &Manifest, out: &Path) {
    let path = out.join("manifest.json");
    if let Err(e) = std::fs::write(&path, manifest.to_json()) {
        eprintln!("写 {} 失败:{e}", path.display());
    }
}

/// 截图:取当帧帧缓冲导出 PNG(`<输出目录>/<file_name()>`)。
fn save_screenshot(request: &ScreenshotRequest, out: &Path) {
    let path = out.join(request.file_name());
    macroquad::prelude::get_screen_data().export_png(&path.display().to_string());
}

/// 把引擎命令转发给服务器:同帧命令打成**一条 Batch 消息**(REQ-010 终审
/// 返修 1——一次 fill 的逐格 SetBlock 是一次原子编辑,单条消息入队、单次
/// TCP 写发出;Spawn 逐条不混批,服务端分配 ID 的回执语义按条对应)。
fn forward_commands(commands: &[EngineCommand], control: &crate::client::ControlHandle) {
    let mut batch: Vec<SimCommand> = Vec::new();
    for command in commands {
        match command {
            EngineCommand::Spawn { state, desc } => {
                // 批内若有 Spawn:先把已积累的批发出(保序),Spawn 单发。
                if !batch.is_empty() {
                    control.command_batch(std::mem::take(&mut batch));
                }
                control.spawn(*state, desc.clone());
            }
            EngineCommand::Detonate {
                center,
                power,
                radius,
            } => batch.push(SimCommand::Detonate {
                center: *center,
                power: *power,
                radius: *radius,
            }),
            EngineCommand::SetBlock { pos, block } => batch.push(SimCommand::SetBlock {
                pos: *pos,
                block: *block,
            }),
        }
    }
    control.command_batch(batch);
}

/// 脚本模式观察者上报:耦合开启时,相机较上次上报变化超过 1 m 立即上报,
/// 否则按 100 ms 节流(传送当帧即可带动观察者)。
fn report_observer_if_due(
    camera: &FlyCamera,
    coupled: bool,
    disconnected: bool,
    last_sent: &mut Option<(macroquad::math::Vec3, Instant)>,
    control: &crate::client::ControlHandle,
) {
    if !coupled || disconnected {
        return;
    }
    // 距上次上报的位移(无记录 = 首报,视为已跳)与距上次上报的时长。
    #[allow(clippy::disallowed_methods)] // 墙钟节流(渲染侧;边缘 crate 局部授权)
    let jumped = match *last_sent {
        None => true,
        Some((pos, _)) => (camera.position - pos).length() > OBSERVER_JUMP_METERS,
    };
    #[allow(clippy::disallowed_methods)]
    let throttled_ok = match *last_sent {
        None => true,
        Some((_, at)) => at.elapsed() >= OBSERVER_SEND_INTERVAL,
    };
    if jumped || throttled_ok {
        let p = camera.position;
        control.move_observer(types::glam::Vec3::new(p.x, p.y, p.z));
        #[allow(clippy::disallowed_methods)]
        {
            *last_sent = Some((camera.position, Instant::now()));
        }
    }
}

/// 脚本模式主循环(REQ-008 §4):引擎驱动相机与命令、截图、写 manifest。
/// 收到第一个实体状态包(`tick_known`,修订 2 D4)之前不推进引擎——`Welcome`
/// 的拍号是占位值 0,且服务器要等观察者位置才发实体包;期间按初始相机位姿
/// 上报观察者。引擎结束 → finish → 写 manifest → 按 exit_code 退出;结束前
/// 断连 → Failed{当前步, "断连:…"} 收尾,退出码 2。
async fn run_script(addr: SocketAddr, env_view: Option<u8>, script: &Path, out: &Path) {
    let (scenario, mut manifest) = prepare_script(script, out);
    let view_distance = scenario.view_distance.or(env_view).unwrap_or(4);
    let (events, control) = ViewerClient::spawn(addr, view_distance)
        .unwrap_or_else(|e| panic!("连接服务器 {addr} 失败(fail-stop):{e}"));

    let total = scenario.steps.len();
    let scenario_name = scenario.name.clone();
    // 起始相机位姿 = 修正后的飞行相机初始位姿(交互/脚本一致);tick 已知前
    // 不创建、不推进引擎。
    let mut engine: Option<Engine> = None;
    let mut camera = FlyCamera::new();
    let start_pose = camera.to_pose();
    let mut world = WorldView::new();
    let mut disconnected: Option<String> = None;
    let mut finished = false;
    let mut coupled = true;
    let mut last_sent: Option<(macroquad::math::Vec3, Instant)> = None;

    loop {
        let dt = get_frame_time();

        // 事件排空:进 WorldState;Disconnected 单独记。
        while let Ok(event) = events.try_recv() {
            match event {
                ViewEvent::Disconnected { reason } => disconnected = Some(reason),
                other => world.apply(other),
            }
        }
        world.sync();

        if world.state.tick_known() && engine.is_none() {
            engine = Some(Engine::new(scenario.clone(), start_pose));
        }

        // 引擎未建(tick 未知)也要上报观察者:服务器要等到观察者位置才开始
        // 发实体状态包(初始位姿首报 + 100 ms 节流保活)。
        if engine.is_none() && disconnected.is_none() {
            report_observer_if_due(&camera, coupled, false, &mut last_sent, &control);
        }

        // 推进引擎(结束后不再推进):输出相机/耦合/命令/截图请求/步骤记录。
        let mut frame_shot: Option<ScreenshotRequest> = None;
        if let (false, Some(e)) = (finished, engine.as_mut()) {
            let spawned = world.state.take_spawned();
            let input = FrameInput {
                dt,
                server_tick: world.state.tick(),
                world: &world.state,
                spawned: &spawned,
            };
            let output = e.frame(&input);
            camera.set_from_pose(&output.camera);
            coupled = output.coupled;
            forward_commands(&output.commands, &control);
            report_observer_if_due(
                &camera,
                coupled,
                disconnected.is_some(),
                &mut last_sent,
                &control,
            );
            // 每条结束的步骤记录立即 push 并重写 manifest.json(进程意外退出
            // 也留下已完成部分)。
            for record in &output.finished {
                manifest.push(record, &world.state);
                write_manifest(&manifest, out);
            }
            frame_shot = output.screenshot;
            if !matches!(e.status(), EngineStatus::Running { .. }) {
                manifest.finish(e.status());
                write_manifest(&manifest, out);
                finished = true;
            }
        }

        // 结束前收到断连:以 Failed{当前步} 收尾(引擎已自然结束则走上面的路)。
        if let (false, Some(reason)) = (finished, &disconnected) {
            let step = match engine.as_ref().map(|e| e.status()) {
                Some(EngineStatus::Running { step }) => *step,
                Some(_) => 1,
                None => 1,
            };
            let status = EngineStatus::Failed {
                step,
                reason: format!("断连:{reason}"),
            };
            manifest.finish(&status);
            write_manifest(&manifest, out);
            finished = true;
        }

        // —— 3D 场景 ——
        clear_background(SKYBLUE);
        set_camera(&camera.to_camera());
        for mesh in world.meshes.values() {
            draw_mesh(mesh);
        }
        // 实体:current 指数逼近 target(帧率无关),画 current;复合体逐格
        // (REQ-010 §10 修订 4 第 4 条)。
        for pose in world.entities.values_mut() {
            let k = 1.0 - (-POSE_SMOOTH * dt).exp();
            pose.advance(k);
        }
        for (id, pose) in world.entities.iter() {
            world.draw_entity(*id, pose);
        }
        set_default_camera();

        // 截图(hud = false):3D 画完、HUD 之前截取。
        if let Some(shot) = &frame_shot
            && !shot.hud
        {
            save_screenshot(shot, out);
        }

        // —— HUD(脚本模式:四行 + SCRIPT 行 + 耦合 + 断连;不进 hud=false 截图)——
        let step_now = current_step(&engine);
        let script_line = format!("SCRIPT {scenario_name} 第 {step_now}/{total} 步");
        draw_hud(
            world.state.tick(),
            world.meshes.len(),
            world.state.live_entities().len(),
            &script_line,
            coupled,
            disconnected.as_ref(),
        );

        // 截图(hud = true):HUD 画完之后截取。
        if let Some(shot) = &frame_shot
            && shot.hud
        {
            save_screenshot(shot, out);
        }

        // 场景结束:窗口关闭,以 exit_code 退出(manifest 已在上面的收尾写盘)。
        if finished {
            control.goodbye();
            let code = engine.as_ref().and_then(|e| e.exit_code()).unwrap_or(2);
            std::process::exit(code);
        }

        next_frame().await
    }
}

/// 当前步(1 起;结束/失败后的帧停在该值;引擎未建 = 1)。
fn current_step(engine: &Option<Engine>) -> usize {
    match engine.as_ref().map(|e| e.status()) {
        Some(EngineStatus::Running { step }) => *step,
        Some(_) => 0,
        None => 1,
    }
}

/// 交互模式主循环(REQ-004A,行为不变;仅初始相机/BLAST_Y 常量修正)。
/// W05(REQ-011 §3.6):`stage = Some(场景文件)` 时进入布景交互——连接装载后
/// 自动执行场景布景段,`R` 重建、`K` 播放动作段;`None` 时行为与改前逐位一致。
pub async fn run_viewer(addr: SocketAddr, view_distance: u8, stage: Option<&Path>) {
    let (events, control) = ViewerClient::spawn(addr, view_distance)
        .unwrap_or_else(|e| panic!("连接服务器 {addr} 失败(fail-stop):{e}"));

    let mut camera = FlyCamera::new();
    let mut world = WorldView::new();
    let mut coupled = true; // 相机 → 观察者耦合(默认开)
    // 鼠标视角:锁定指针并隐藏光标(未锁定时指针到屏缘 delta 归零,视角
    // 转不动——ACV4 实跑反馈);M 键切换锁定,便于切出窗口。
    let mut mouse_grabbed = true;
    set_cursor_grab(mouse_grabbed);
    show_mouse(!mouse_grabbed);
    // viewer 生成实体的网格序号(仅用于出生点散开;ID 由服务端分配,
    // REQ-006 任务 02)。
    let mut spawn_seq: u32 = 0;
    // 节流用墙钟(渲染节流不属确定性核;边缘 crate 局部 allow 并注明,
    // clippy.toml 纪律,driver.rs 同款手法)。
    #[allow(clippy::disallowed_methods)]
    let mut last_send = {
        #[allow(clippy::disallowed_methods)]
        let now = Instant::now();
        now - OBSERVER_SEND_INTERVAL // 首位姿立即可发
    };
    let mut disconnected: Option<String> = None;
    let mut disconnect_elapsed = 0.0_f32;
    // REQ-011 W01:调试工具状态(交互模式独有;脚本模式不碰)。
    let mut tools = ToolState::default();
    // 每帧拾取目标(线框高亮、HUD、左/右键共用;M 释放鼠标时仍显示)。
    let mut target: Option<Hit>;
    // REQ-011 W02:F 蓄力状态机(按下/松开边沿跟踪 + 逐帧 dt)。
    let mut charge = ChargeState::default();
    let mut f_down_prev = false;
    // REQ-011 W04:调试面板(F3 切换、交互默认显示;计数基线/最近事件)。
    let mut panel_shown = true;
    let mut panel = PanelState::default();
    // 订阅只发一次(连上即发;服务器侧幂等,重发无副作用——本地一次性)。
    let mut subscribed = false;
    // REQ-011 W05:布景执行器(None = 普通交互模式;连接装载后自动搭布景,
    // R 重建、K 播放动作段)。
    let mut stage_driver: Option<StageDriver> =
        stage.map(|path| StageDriver::new(prepare_stage(path), camera.to_pose()));

    loop {
        let dt = get_frame_time();
        camera.update(dt);
        tools.tick(dt); // 拒绝提示计时(逐帧 dt;工具行为不进模拟)

        // 事件排空(主循环 try_recv 排空,README 决策 3)。
        while let Ok(event) = events.try_recv() {
            match event {
                ViewEvent::Disconnected { reason } => {
                    disconnected = Some(reason);
                }
                ViewEvent::DebugStats {
                    tick,
                    counters,
                    recent,
                } => panel.receive(tick, counters, recent),
                other => world.apply(other),
            }
        }
        world.sync();

        // W04:交互模式订阅调试遥测(连上即发一次;脚本模式不订阅)。
        if !subscribed {
            control.subscribe_debug();
            subscribed = true;
        }

        // —— REQ-011 W05:布景推进(连接装载后自动搭;tick 已知才动——
        //   布景的等待步骤按服务器拍号算,拍号未知时等无意义;断连不发)——
        if let (false, Some(driver)) = (disconnected.is_some(), stage_driver.as_mut())
            && world.state.tick_known()
        {
            // [server] 提示与超时提示 → 工具提示通道(3 秒过期,复用)。
            if let Some(notice) = driver.take_server_notice() {
                tools.reject(notice);
            }
            if let Some(notice) = driver.take_timeout_notice() {
                tools.reject(notice);
            }
            if let Some(notice) = driver.take_reject_notice() {
                tools.reject(notice);
            }
            let out = driver.frame(dt, world.state.tick(), &world.state);
            for entry in out {
                match entry {
                    crate::tools::StageOut::Command(command) => {
                        forward_commands(std::slice::from_ref(&command), &control);
                    }
                    crate::tools::StageOut::Remove(entity) => {
                        control.command(messaging::SimCommand::RemoveEntity { entity });
                    }
                }
            }
            // 布景相机跟随(W05-2):只在 Building(搭布景,含 R 重放)时
            // 覆写用户飞行;Playing 时相机留给用户(§3.6,K 播放跳过相机类
            // 步骤——punch 动作段带等待,锁死相机用户看不到撞击);Ready /
            // Played 不夺回控制权。判定收在 driver 的纯函数里,render 只管调用。
            if let Some(pose) = driver.camera_override() {
                camera.set_from_pose(&pose);
            }
        }

        // 拾取(REQ-011 §3.3):客户端已收到的 chunk 数据,最远 48 m。
        target = pick_at_camera(&camera, &world.state);

        // 空格切换耦合(边沿触发,不连发)。
        if is_key_pressed(KeyCode::Space) {
            coupled = !coupled;
        }
        // M 切换鼠标锁定(释放后可正常使用指针;再按 M 重新锁定转视角)。
        if is_key_pressed(KeyCode::M) {
            mouse_grabbed = !mouse_grabbed;
            set_cursor_grab(mouse_grabbed);
            show_mouse(!mouse_grabbed);
        }
        // —— REQ-011 W01 工具按键(仅交互模式;鼠标释放(M)时左右键不
        //   触发,避免点 HUD 误拆;断连后不发命令)——
        let tools_sendable = mouse_grabbed && disconnected.is_none();
        if is_key_pressed(KeyCode::Key1) {
            crate::tools::select_material(&mut tools, 0);
        }
        if is_key_pressed(KeyCode::Key2) {
            crate::tools::select_material(&mut tools, 1);
        }
        if is_key_pressed(KeyCode::Key3) {
            crate::tools::select_material(&mut tools, 2);
        }
        if is_key_pressed(KeyCode::Key4) {
            crate::tools::select_material(&mut tools, 3);
        }
        if is_key_pressed(KeyCode::T) {
            cycle_placeable(&mut tools);
        }
        if is_key_pressed(KeyCode::H) {
            toggle_help(&mut tools);
        }
        // W04:F3 切换面板;C 把当前计数设为基线(增量归零重新累计)。
        if is_key_pressed(KeyCode::F3) {
            panel_shown = !panel_shown;
        }
        if is_key_pressed(KeyCode::C) {
            panel.reset_baseline();
        }
        // —— REQ-011 W05:R 重建 / K 播放动作段(布景模式;断连不发)——
        if let (false, Some(driver)) = (disconnected.is_some(), stage_driver.as_mut()) {
            if is_key_pressed(KeyCode::R) {
                let mut out = Vec::new();
                driver.rebuild(&world.state, &mut out);
                for entry in out {
                    if let crate::tools::StageOut::Remove(entity) = entry {
                        control.command(messaging::SimCommand::RemoveEntity { entity });
                    }
                }
            }
            if is_key_pressed(KeyCode::K) {
                driver.play();
            }
        } else if (is_key_pressed(KeyCode::R) || is_key_pressed(KeyCode::K))
            && stage_driver.is_none()
        {
            // 普通交互模式按 R/K:提示未给 --stage(§3.6/放行说明)。
            tools.reject("no --stage scenario");
        }
        if tools_sendable {
            if is_mouse_button_pressed(MouseButton::Left) {
                let action = dig(target.as_ref());
                apply_tool_action(action, &mut tools, &control);
            }
            if is_mouse_button_pressed(MouseButton::Right) {
                let yaw = camera.yaw.to_degrees();
                let action = place(&tools, target.as_ref(), yaw, &world.state);
                apply_tool_action(action, &mut tools, &control);
            }
        }
        // —— REQ-011 W02:发射(F 点按/蓄力、E 形状、滚轮预设速度)——
        // E:循环切换形状(不受鼠标锁定影响;断连无妨,纯本地)。
        if is_key_pressed(KeyCode::E) {
            tools.launch_shape = tools.launch_shape.next();
        }
        // 滚轮:预设速度 ±5,一帧最多一档;鼠标释放(M)时不生效(§3.1)。
        if mouse_grabbed {
            let (_, wheel_y) = mouse_wheel();
            if wheel_y != 0.0 {
                tools.preset_speed = adjust_preset_speed(tools.preset_speed, wheel_y);
            }
        }
        // F:按下边沿 press;按住逐帧 update;松开边沿 release → 发射
        // (拒绝时蓄力结束、提示 3 秒)。断连(W02-2 返修 3):f_down 因
        // `disconnected.is_none()` 变 false 会误走松开分支——先判断连,
        // charge.cancel() 收掉按住状态,不走发射。
        let f_raw_down = is_key_down(KeyCode::F);
        if disconnected.is_some() && f_raw_down {
            if f_down_prev {
                charge.cancel();
            }
            f_down_prev = false;
        } else {
            let f_down = f_raw_down;
            if f_down && !f_down_prev {
                charge.press();
            }
            if f_down {
                charge.update(dt);
            }
            if f_down_prev && !f_down {
                let speed = charge.release(tools.preset_speed);
                let o = camera.position;
                let d = camera.forward();
                let got = launch_request(
                    tools.launch_shape,
                    tools.material,
                    speed,
                    TVec3::new(o.x, o.y, o.z),
                    TVec3::new(d.x, d.y, d.z),
                    camera.yaw.to_degrees(),
                    &world.state,
                );
                match got {
                    Ok((state, desc)) => {
                        tools.last_shot = Some((tools.launch_shape, tools.material.name(), speed));
                        control.spawn(state, desc);
                    }
                    Err(reason) => {
                        charge.cancel(); // 拒绝:蓄力结束,不发命令(§3.2)
                        tools.reject(reason);
                    }
                }
            }
            f_down_prev = f_down;
        }
        // B:准星前方生成动态立方体(落下→静止→凝固成地形色;ACV4 ③④)。
        // 连按散布:同点重叠的刚体被求解器永久抖动、永不满足凝固判据
        // (实跑反馈的蓝方块存留根因之一)——按序号在 5×5 网格散开出生点。
        if is_key_pressed(KeyCode::B) && disconnected.is_none() {
            let gx = (spawn_seq % 5) as f32 - 2.0;
            let gz = ((spawn_seq / 5) % 5) as f32 - 2.0;
            spawn_seq = spawn_seq.wrapping_add(1);
            let p = camera.position
                + camera.forward() * SPAWN_AHEAD
                + vec3(gx * 1.6, SPAWN_LIFT, gz * 1.6);
            // REQ-006 任务 02:客户端不选 EntityId;服务端分配并回 Spawned。
            control.spawn(
                EntityState {
                    translation: types::glam::Vec3::new(p.x, p.y, p.z),
                    ..EntityState::default()
                },
                EntityDesc::default(),
            );
        }
        // X:引爆(把立方体炸飞;ACV4 ③)。爆心吸附相机附近的**活实体**
        // (≤ BLAST_SNAP 取最近):"无法引爆"两个叠加根因(反馈 5)——
        // ① 立方体落地约半秒即凝固,凝固/冻结体免疫爆炸(§9.12 既定语义,
        // 实体表只含未凝固者:凝固即 removed,吸附必命中可炸目标且冲量
        // 全量);② 网格散布出生 + 定点爆心常贴 radius 边缘(冲量 <13%)。
        // 附近无实体 → 退回定点(前方 BLAST_AHEAD 米,地表高)。
        if is_key_pressed(KeyCode::X) && disconnected.is_none() {
            let mut c = camera.position + camera.forward() * BLAST_AHEAD;
            c.y = BLAST_Y;
            if let Some((_, nearest)) = world
                .entities
                .iter()
                .filter(|(_, pose)| (pose.target - camera.position).length() <= BLAST_SNAP)
                .min_by_key(|(_, pose)| (pose.target - camera.position).length().to_bits())
            {
                c = nearest.target;
            }
            control.command(SimCommand::Detonate {
                center: types::glam::Vec3::new(c.x, c.y, c.z),
                power: BLAST_POWER,
                radius: BLAST_RADIUS,
            });
        }
        // 耦合时:相机位姿节流上报(≥100ms 一次;仅未断连时)。
        if coupled && disconnected.is_none() {
            #[allow(clippy::disallowed_methods)] // 墙钟节流(同上,边缘授权)
            let due = last_send.elapsed() >= OBSERVER_SEND_INTERVAL;
            if due {
                let p = camera.position; // macroquad glam → types glam 字段换算
                control.move_observer(types::glam::Vec3::new(p.x, p.y, p.z));
                #[allow(clippy::disallowed_methods)]
                {
                    last_send = Instant::now();
                }
            }
        }

        // —— 3D 场景 ——
        clear_background(SKYBLUE);
        set_camera(&camera.to_camera());
        for mesh in world.meshes.values() {
            draw_mesh(mesh);
        }
        // 实体:current 指数逼近 target(帧率无关),画 current;复合体逐格
        // (REQ-010 §10 修订 4 第 4 条)。
        for pose in world.entities.values_mut() {
            let k = 1.0 - (-POSE_SMOOTH * dt).exp();
            pose.advance(k);
        }
        for (id, pose) in world.entities.iter() {
            world.draw_entity(*id, pose);
        }
        // —— REQ-011 W01:拾取线框 + 预制轮廓(3D 相机下画;线框颜色区分
        //   基岩:普通格白、基岩橙)——
        if let Some(hit) = &target {
            let color = if hit.block == voxel::builtin::BEDROCK {
                ORANGE
            } else {
                WHITE
            };
            draw_cell_wire(hit.block_pos, color);
        }
        // 轮廓预览(放置物 = 预制结构且有拾取目标):锚点 = 命中面外侧格,
        // 朝向 = 相机水平朝向取整;与实际放置同一展开函数,越界警示色(红)。
        if let (Placeable::Prefab(prefab), Some(hit)) = (&tools.placeable, &target) {
            let orientation = orientation_from_yaw(camera.yaw.to_degrees());
            let (cells, out_of_world) = preview_outline(*prefab, hit.place_pos(), orientation);
            let color = if out_of_world { RED } else { GRAY };
            for (pos, _) in &cells {
                draw_cell_wire(*pos, color);
            }
        }
        set_default_camera();

        // —— HUD(2D 默认相机;四行)——
        let lines = hud_lines(
            world.state.tick(),
            world.meshes.len(),
            world.entities.len(),
            get_fps(),
        );
        for (i, line) in lines.iter().enumerate() {
            draw_text(line, 12.0, 20.0 + i as f32 * 20.0, 18.0, WHITE);
        }
        draw_text(
            if coupled {
                "coupled: ON (Space to toggle)"
            } else {
                "coupled: OFF (Space to toggle)"
            },
            12.0,
            104.0,
            16.0,
            YELLOW,
        );
        // REQ-011 W04:调试面板(F3 切换,交互默认显示)画在四行 HUD + 耦合
        // 行下方;面板显示时第 1 段(工具)取代 W01 的临时工具行,不重复画;
        // 面板隐藏时工具行与提示行照旧(W01 行为)。W05:布景状态行画在面板
        // 下方(面板隐藏时在提示行下),提示已复用工具提示通道随面板/提示行
        // 显示。
        let stage_status = stage_driver
            .as_ref()
            .map(|driver| (driver.status_line(), driver.phase()));
        let panel_len = if panel_shown {
            let lines = panel_lines(&tools, target.as_ref(), &world.state, &panel);
            let len = lines.len();
            for (i, (line, color)) in lines.iter().enumerate() {
                draw_text(
                    line,
                    12.0,
                    124.0 + i as f32 * 18.0,
                    16.0,
                    panel_color(*color),
                );
            }
            Some(len)
        } else {
            // 面板隐藏:W01 临时工具行 + 提示行照旧(ASCII)。
            let tool_line = crate::tools::hud_tool_line(&tools, target.as_ref(), &world.state);
            draw_text(&tool_line, 12.0, 122.0, 16.0, WHITE);
            if let Some((message, _)) = &tools.notice {
                draw_text(message, 12.0, 142.0, 16.0, ORANGE);
            }
            None
        };
        // W05 布景状态行:面板显示时排在其下(y = 124 + 面板行数 × 18 + 6);
        // 隐藏时 y = 162(提示行 142 之下)。
        if let Some((line, _)) = &stage_status {
            let y = match panel_len {
                Some(len) => 124.0 + len as f32 * 18.0 + 6.0,
                None => 162.0,
            };
            draw_text(line, 12.0, y, 16.0, YELLOW);
        }
        // 蓄力条(W02;屏幕中央,蓄力阶段才显示;ASCII。中央 y 略偏上,
        // 避开断连提示带)。
        if let Some(speed) = charge.display_speed() {
            let bar = speed_bar(speed);
            let y = screen_height() * 0.5 - 40.0;
            let x = screen_width() * 0.5 - 60.0;
            draw_text(&bar, x, y, 18.0, YELLOW);
        }
        // 按键帮助表(H 切换;W04 布局:画在右半屏,与左侧面板不叠字)。
        if tools.help_shown {
            let help_x = screen_width() * 0.5 + 8.0;
            for (i, line) in help_lines().iter().enumerate() {
                draw_text(line, help_x, 20.0 + i as f32 * 18.0, 15.0, YELLOW);
            }
        }
        draw_text(
            "WASD+mouse fly Shift/Ctrl up/down | B spawn cube X blast | M grab/release mouse",
            12.0,
            screen_height() - 12.0,
            16.0,
            DARKGRAY,
        );
        // 断连提示放屏幕中部(W01-2:原先 y=128 与工具行 y=122 重叠;中部
        // 独立带,不与任何 HUD 行重叠,断连 3 s 宽限期里足够醒目。W04:
        // 原因英文化——已知客户端原因映射 + 非 ASCII 换 `?`)。
        if let Some(reason) = &disconnected {
            let text = crate::tools::disconnect_text(reason);
            let y = screen_height() * 0.5;
            draw_text(&text, 12.0, y, 18.0, YELLOW);
        }

        // 断连宽限后退出(让用户看到提示;Goodbye 后服务器 Bye/EOF 亦入此)。
        if disconnected.is_some() {
            disconnect_elapsed += dt;
            if disconnect_elapsed >= DISCONNECT_GRACE_SECS {
                break;
            }
        }

        next_frame().await
    }
}

/// HUD 绘制(脚本模式;四行 + 耦合 + SCRIPT 行 + 断连提示)。
fn draw_hud(
    tick: u64,
    chunks: usize,
    entities: usize,
    script_line: &str,
    coupled: bool,
    disconnected: Option<&String>,
) {
    let lines = hud_lines(tick, chunks, entities, get_fps());
    for (i, line) in lines.iter().enumerate() {
        draw_text(line, 12.0, 20.0 + i as f32 * 20.0, 18.0, WHITE);
    }
    draw_text(script_line, 12.0, 104.0, 16.0, YELLOW);
    draw_text(
        if coupled {
            "coupled: ON(脚本模式)"
        } else {
            "coupled: OFF(脚本模式)"
        },
        12.0,
        122.0,
        16.0,
        YELLOW,
    );
    if let Some(reason) = disconnected {
        let text = format!("已断连:{reason}");
        draw_text(&text, 12.0, 140.0, 18.0, YELLOW);
    }
}

/// 窗口配置(标题/尺寸;crate 根 `#[macroquad::main(window_conf)]` 入口用)。
/// 脚本模式:1280×720、不可调整大小、标题注明脚本模式。宏参数须是裸标识符,
/// 命令行参数经进程级暂存(读一次 std::env::args)。
/// 测试目标下不可达(dead_code 实警),allow 并注明。
#[allow(dead_code)] // 测试目标(cargo test)不含窗口入口;bin 装配需要
pub fn window_conf() -> macroquad::conf::Conf {
    // 命令行预读(parse_args 的结果由 run 再解析一次;此处只为挑窗口配置)。
    let script_mode = crate::cli::parse_args(std::env::args().skip(1))
        .map(|args| matches!(args.mode, ViewerMode::Script { .. }))
        .unwrap_or(false);
    let (title, resizable) = if script_mode {
        (
            "physics_minecraft viewer(脚本模式 REQ-008)".to_owned(),
            false,
        )
    } else {
        ("physics_minecraft viewer(REQ-004A demo)".to_owned(), true)
    };
    macroquad::conf::Conf {
        miniquad_conf: macroquad::miniquad::conf::Conf {
            window_title: title,
            window_width: 1280,
            window_height: 720,
            window_resizable: resizable,
            ..macroquad::miniquad::conf::Conf::default()
        },
        // 批缓冲扩容(ACV4 实跑反馈 3):macroquad 默认 10000 顶点/5000 索引,
        // 单个超平坦 chunk mesh ≈ 3072 顶点/4608 索引——再凝固十几个立方体
        // 索引即越过 5000,geometry() 截断该 mesh(警告刷屏 + 缺三角形"穿模")。
        // 顶点容量上限 65_535:quad_gl.rs 的批内索引重映射是
        // `index + vertices_count as u16`(macroquad 0.4.16 硬编码 u16),
        // 批内顶点 > 65_535 即溢出 panic(实跑反馈 4:扩到 20 万后启动闪退,
        // ~29 个 chunk 同批即触发)。65_535 × 每顶点 2 索引 → 索引容量 13 万,
        // 两容量任一到顶 macroquad 自动切批,draw call 数随 chunk 数有界增长。
        draw_call_vertex_capacity: 65_535,
        draw_call_index_capacity: 130_000,
        ..macroquad::conf::Conf::default()
    }
}

/// 窗口入口体(crate 根的宏展开 `main` 调用 `amain()`):parse_args → 按模式
/// 分发。参数错误 → 用法到标准错误,退出码 2(REQ-008 §4)。
#[allow(dead_code)] // 测试目标不可达(同上)
pub async fn amain() {
    let args = match crate::cli::parse_args(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(e) => {
            eprintln!("用法:viewer [--addr <地址>] [--script <场景文件> --out <输出目录>]");
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    run(args).await;
}
#[cfg(test)]
mod tests {
    use super::*;

    /// 旋转几乎同向(|dot| ≥ 1−ε 即夹角 ≲ √(2ε);不走 `angle_between`——
    /// 它经 acos_approx,dot≈1 处极小逼近误差 ~4e-4 rad,度量不了近恒等)。
    fn rot_close(a: macroquad::math::Quat, b: macroquad::math::Quat) -> bool {
        a.dot(b).abs() > 0.999_999
    }

    /// 实体位姿插值(纯计算,REQ-010 §10 修订 4 第 4 条):k = 1 一步到位
    /// (位置与旋转都追平 target);k = 0 不动;中间 k 位置线性逼近。
    ///
    /// 会变红的实现:旋转漏插值(slerp 行删掉——复合体永远按旧朝向画);
    /// k 符号反向(越插越远)。
    #[test]
    fn entity_pose_advance_lerps_position_and_slerps_rotation() {
        let mut pose = EntityPose::new(vec3(0.0, 0.0, 0.0), macroquad::math::Quat::IDENTITY);
        pose.target = vec3(10.0, 0.0, 0.0);
        let q = macroquad::math::Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        pose.target_rot = q;

        pose.advance(0.0);
        assert_eq!(pose.current, vec3(0.0, 0.0, 0.0), "k = 0 不动");
        assert!(
            rot_close(pose.current_rot, macroquad::math::Quat::IDENTITY),
            "k = 0 旋转不动"
        );

        pose.advance(1.0);
        assert_eq!(pose.current, vec3(10.0, 0.0, 0.0), "k = 1 位置追平");
        assert!(rot_close(pose.current_rot, q), "k = 1 旋转追平(slerp 生效)");

        // 中间值:位置线性逼近(0 < k < 1)。
        let mut pose2 = EntityPose::new(vec3(0.0, 0.0, 0.0), macroquad::math::Quat::IDENTITY);
        pose2.target = vec3(10.0, 0.0, 0.0);
        pose2.advance(0.5);
        assert!(
            (pose2.current.x - 5.0).abs() < 1e-5,
            "线性插值中点,实得 {}",
            pose2.current.x
        );
    }

    /// 发送层(F04a-2):`forward_commands` 把同帧 SetBlock/Detonate 打成一条
    /// Batch(REQ-010 终审返修 1 的原子编辑语义),Spawn 逐条不混批且
    /// **desc 原样透传**(F04a:复合 desc 不得在发送层被换成默认单格)。
    ///
    /// 会变红的实现:`control.spawn` 处改回 `EntityDesc::default()`(丢复合
    /// 形状);把 Spawn 混进 Batch(Spawned 回执按条对应语义被破坏);批序颠倒。
    #[test]
    fn forward_commands_batches_sets_and_forwards_spawn_descs() {
        let (control, rx) = crate::client::ControlHandle::for_test();

        let set = |x: i32| EngineCommand::SetBlock {
            pos: types::BlockPos::new(x, 4, 6),
            block: types::BlockId(3),
        };
        let compound_desc = || {
            EntityDesc::compound(
                0.45,
                types::CompoundShape::new(vec![
                    types::ShapeCell {
                        offset: [0, 0, 0],
                        block: types::BlockId(1),
                    },
                    types::ShapeCell {
                        offset: [1, 0, 0],
                        block: types::BlockId(1),
                    },
                ])
                .unwrap(),
            )
        };
        let spawn_at = |x: f32, desc: EntityDesc| EngineCommand::Spawn {
            state: EntityState {
                translation: types::glam::Vec3::new(x, -50.0, 0.0),
                ..EntityState::default()
            },
            desc,
        };

        let commands = vec![
            set(1),
            set(2),
            spawn_at(8.0, compound_desc()),
            set(3),
            spawn_at(12.0, EntityDesc::default()),
        ];
        forward_commands(&commands, &control);

        // 依次:批(2 条 SetBlock)→ Spawn(复合 desc)→ 批(1 条)→ Spawn(默认 desc)。
        let batch1 = rx.try_recv().expect("第一批(SetBlock×2)");
        let spawn1 = rx.try_recv().expect("复合 Spawn");
        let batch2 = rx.try_recv().expect("第二批(SetBlock×1)");
        let spawn2 = rx.try_recv().expect("默认 Spawn");

        assert_eq!(
            batch1,
            crate::client::Control::Batch(vec![
                SimCommand::SetBlock {
                    pos: types::BlockPos::new(1, 4, 6),
                    block: types::BlockId(3),
                },
                SimCommand::SetBlock {
                    pos: types::BlockPos::new(2, 4, 6),
                    block: types::BlockId(3),
                },
            ]),
            "Spawn 之前的批先发出(保序),两条 SetBlock 一条 Batch"
        );
        assert_eq!(
            spawn1,
            crate::client::Control::Spawn {
                state: EntityState {
                    translation: types::glam::Vec3::new(8.0, -50.0, 0.0),
                    ..EntityState::default()
                },
                desc: compound_desc(),
            },
            "复合 desc 原样透传(F04a:发送层不得丢 shape)"
        );
        assert_eq!(
            batch2,
            crate::client::Control::Batch(vec![SimCommand::SetBlock {
                pos: types::BlockPos::new(3, 4, 6),
                block: types::BlockId(3),
            }]),
            "两个 Spawn 之间的 SetBlock 独立成批"
        );
        assert_eq!(
            spawn2,
            crate::client::Control::Spawn {
                state: EntityState {
                    translation: types::glam::Vec3::new(12.0, -50.0, 0.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            },
            "单格 desc 仍为默认(逐位不变)"
        );
        assert!(rx.try_recv().is_err(), "通道应已收干(恰好 4 条消息)");
    }
}
