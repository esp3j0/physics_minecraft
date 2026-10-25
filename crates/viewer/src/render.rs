//! render —— macroquad 主循环(窗口/相机/HUD;REQ-004A §2;REQ-008 §4 脚本模式)。
//! **不写窗口类测试**(README 决策 2;ACV4 手动清单,禁造假断言)——本文件仅含装配
//! 编排,纯算力全部在 mesh(有单测)、client(零 macroquad)、world/cli/manifest
//! (REQ-008 纯逻辑层,各有单测)。

use std::collections::BTreeMap;
use std::net::SocketAddr;
use std::path::Path;
use std::time::Instant;

use macroquad::camera::{Camera3D, Projection, set_camera, set_default_camera};
use macroquad::color::{BLUE, DARKGRAY, SKYBLUE, WHITE, YELLOW};
use macroquad::input::{
    KeyCode, is_key_down, is_key_pressed, mouse_delta_position, set_cursor_grab, show_mouse,
};
use macroquad::math::vec3;
use macroquad::models::{Mesh, draw_cube, draw_mesh};
use macroquad::text::draw_text;
use macroquad::time::{get_fps, get_frame_time};
use macroquad::window::{clear_background, next_frame, screen_height};
use messaging::SimCommand;
use types::{ChunkPos, EntityDesc, EntityId, EntityState};
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
struct EntityPose {
    current: macroquad::math::Vec3,
    target: macroquad::math::Vec3,
}

/// 位置插值速率(1/s;越大越紧跟,14 ≈ 70ms 追平大半)。
const POSE_SMOOTH: f32 = 14.0;

/// 世界渲染态(REQ-008 单元 02 重构):纯逻辑的 [`WorldState`(方块镜像/实体
/// 服务器位置/拍号/回执)] + 渲染侧的 Mesh 表与实体插值表。每帧用
/// `take_dirty_chunks` 同步 Mesh,实体插值表跟随 `live_entities`。
struct WorldView {
    /// 纯逻辑世界状态(脚本引擎经 WorldQuery 读它)。
    state: WorldState,
    /// chunk Mesh 表(脏 chunk 在则重建、不在则删除)。
    meshes: BTreeMap<ChunkPos, Mesh>,
    /// 实体插值表(跟随 state 的实体表;新实体从目标位置起步、消失即删)。
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
    /// 已有实体只推进 target(渲染帧插值逼近)。
    fn sync_entities(&mut self) {
        let live = self.state.live_entities();
        let present: std::collections::BTreeSet<EntityId> =
            live.iter().map(|(id, _)| *id).collect();
        self.entities.retain(|id, _| present.contains(id));
        for (id, pos) in live {
            let target = vec3(pos.x, pos.y, pos.z);
            self.entities
                .entry(id)
                .and_modify(|pose| pose.target = target)
                .or_insert(EntityPose {
                    current: target,
                    target,
                });
        }
    }
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
            run_viewer(addr, env_view.unwrap_or(4)).await;
        }
        ViewerMode::Script { script, out } => {
            run_script(addr, env_view, &script, &out).await;
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

/// 把引擎命令转发给服务器(Spawn → control.spawn;Detonate/SetBlock → control.command)。
fn forward_commands(commands: &[EngineCommand], control: &crate::client::ControlHandle) {
    for command in commands {
        match command {
            EngineCommand::Spawn { state } => {
                control.spawn(*state, EntityDesc::default());
            }
            EngineCommand::Detonate {
                center,
                power,
                radius,
            } => {
                control.command(SimCommand::Detonate {
                    center: *center,
                    power: *power,
                    radius: *radius,
                });
            }
            EngineCommand::SetBlock { pos, block } => {
                control.command(SimCommand::SetBlock {
                    pos: *pos,
                    block: *block,
                });
            }
        }
    }
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
        // 实体:current 指数逼近 target(帧率无关),画 current。
        for pose in world.entities.values_mut() {
            let k = 1.0 - (-POSE_SMOOTH * dt).exp();
            pose.current += (pose.target - pose.current) * k;
            draw_cube(
                pose.current,
                vec3(ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE),
                None,
                BLUE,
            );
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
pub async fn run_viewer(addr: SocketAddr, view_distance: u8) {
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

    loop {
        let dt = get_frame_time();
        camera.update(dt);

        // 事件排空(主循环 try_recv 排空,README 决策 3)。
        while let Ok(event) = events.try_recv() {
            match event {
                ViewEvent::Disconnected { reason } => {
                    disconnected = Some(reason);
                }
                other => world.apply(other),
            }
        }
        world.sync();

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
        // 实体:current 指数逼近 target(帧率无关),画 current。
        for pose in world.entities.values_mut() {
            let k = 1.0 - (-POSE_SMOOTH * dt).exp();
            pose.current += (pose.target - pose.current) * k;
            draw_cube(
                pose.current,
                vec3(ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE),
                None,
                BLUE,
            );
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
                "coupled: ON(Space 切换)"
            } else {
                "coupled: OFF(Space 切换)"
            },
            12.0,
            104.0,
            16.0,
            YELLOW,
        );
        draw_text(
            "WASD+鼠标飞行 Shift/Ctrl 升降 | B 生成立方体 X 爆炸 | M 释放/锁定鼠标",
            12.0,
            screen_height() - 12.0,
            16.0,
            DARKGRAY,
        );
        if let Some(reason) = &disconnected {
            let text = format!("已断连:{reason}(窗口将关闭)");
            draw_text(&text, 12.0, 128.0, 18.0, YELLOW);
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
