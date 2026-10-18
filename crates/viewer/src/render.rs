//! render —— macroquad 主循环(窗口/相机/HUD;REQ-004A §2)。**不写窗口类
//! 测试**(README 决策 2;ACV4 手动清单,禁造假断言)——本文件仅含装配编排,
//! 纯算力全部在 mesh(有单测)与 client(零 macroquad)。

use std::collections::BTreeMap;
use std::time::Instant;

use macroquad::camera::{Camera3D, Projection, set_camera, set_default_camera};
use macroquad::color::{BLUE, DARKGRAY, SKYBLUE, WHITE, YELLOW};
use macroquad::input::{
    KeyCode, is_key_down, is_key_pressed, mouse_delta_position, set_cursor_grab, show_mouse,
};
use macroquad::math::vec3;
use macroquad::miniquad::conf::Conf;
use macroquad::models::{Mesh, draw_cube, draw_mesh};
use macroquad::text::draw_text;
use macroquad::time::{get_fps, get_frame_time};
use macroquad::window::{clear_background, next_frame, screen_height};
use messaging::SimCommand;
use types::{ChunkPos, EntityDesc, EntityId, EntityState};
// 注:macroquad 0.4 内嵌 glam 0.27,types 用 glam 0.33——两版 Vec3 不互认,
// 一切跨界处按字段换算(下文 vec3(t.x, t.y, t.z) / types::glam::Vec3::new)。

use crate::client::{ViewEvent, ViewerClient};
use crate::mesh::chunk_mesh;

/// 相机移动速度(格/秒)。
const MOVE_SPEED: f32 = 16.0;
/// 鼠标灵敏度(弧度/像素)。
const LOOK_SENSITIVITY: f32 = 0.003;
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
/// X 爆炸参数(与 e2e 爆炸测试同量级:全命中、可感飞散)。
const BLAST_POWER: f32 = 12.0;
const BLAST_RADIUS: f32 = 6.0;
/// viewer 生成实体的 EntityId origin 段(服务器分片序号 <<32 打包;9999 远离
/// 一切真实分片号,保证客户端生成 ID 不与服务器侧冲突——决议 7 单调不回收)。
const SPAWN_ORIGIN: u32 = 9999;

/// 自由飞行相机(欧拉:偏航绕 y,俯仰;WASD + 鼠标视角,Shift/Ctrl 升降)。
struct FlyCamera {
    position: macroquad::math::Vec3,
    yaw: f32,
    pitch: f32,
}

impl FlyCamera {
    /// 初始位姿:地表上方(超平坦顶面 world y=4),俯视地面。
    fn new() -> Self {
        Self {
            position: vec3(8.0, 12.0, 24.0),
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
        self.yaw += delta.x * LOOK_SENSITIVITY;
        self.pitch = (self.pitch - delta.y * LOOK_SENSITIVITY)
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

/// 世界渲染态(chunk Mesh 表 + 方块镜像 + 实体位姿表;增量事件驱动)。
struct WorldView {
    chunks: BTreeMap<ChunkPos, Mesh>,
    /// chunk 方块镜像(BlockUpdates 就地改写后整块重算 Mesh;demo 定位)。
    blocks: BTreeMap<ChunkPos, voxel::Chunk>,
    /// 实体位姿(增量 latest-wins;removed 即移除——README 决策 5)。
    entities: BTreeMap<EntityId, macroquad::math::Vec3>,
    tick: u64,
}

impl WorldView {
    fn new() -> Self {
        Self {
            chunks: BTreeMap::new(),
            blocks: BTreeMap::new(),
            entities: BTreeMap::new(),
            tick: 0,
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

    /// 应用一个事件(增删改;Disconnected 不在此处理——主循环判终)。
    fn apply(&mut self, event: ViewEvent) {
        match event {
            ViewEvent::Welcome { tick, .. } => self.tick = tick.0,
            ViewEvent::Chunk { pos, chunk } => {
                self.chunks
                    .insert(pos, chunk_mesh(&chunk, Self::origin_of(pos)));
                self.blocks.insert(pos, chunk);
            }
            ViewEvent::ChunkGone { pos } => {
                self.chunks.remove(&pos);
                self.blocks.remove(&pos);
            }
            ViewEvent::Entities {
                tick,
                states,
                removed,
            } => {
                self.tick = tick.0;
                for (id, state) in states {
                    let t = state.translation; // types::glam(0.33)→ macroquad(0.27)字段换算
                    self.entities.insert(id, vec3(t.x, t.y, t.z));
                }
                for id in removed {
                    self.entities.remove(&id);
                }
            }
            ViewEvent::Blocks { updates } => {
                // 方块增量:镜像就地改写,触及的 chunk 整块重算 Mesh(凝固/
                // 挖掘可见即达意;demo 不做局部补丁)。chunk 未到手(流未达)
                // 的格子先落镜像也不建 Mesh——ChunkData 随后整块覆盖。
                for (pos, block) in updates {
                    let cpos = pos.chunk_of();
                    let chunk = self.blocks.entry(cpos).or_insert_with(voxel::Chunk::air);
                    chunk.set(
                        pos.x.rem_euclid(16) as usize,
                        (pos.y - types::WORLD_Y_MIN) as usize,
                        pos.z.rem_euclid(16) as usize,
                        block,
                    );
                    self.chunks
                        .insert(cpos, chunk_mesh(chunk, Self::origin_of(cpos)));
                }
            }
            ViewEvent::Disconnected { .. } => {}
        }
    }
}

/// 主循环入口(main 调用):连接 → 装配 → 窗口循环。
/// 服务器地址由 main 传入(环境变量/默认);视距默认 4(REQ-004A §2)。
/// 相机 → 观察者耦合:默认开(MoveObserver 跟相机,≥100ms 节流),空格切换。
pub async fn run_viewer(addr: std::net::SocketAddr, view_distance: u8) {
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
    // viewer 生成实体的序号(SPAWN_ORIGIN 段内单调;断线重连从 0 重来,
    // 与服务器侧实体永不冲突——服务器分片号远小于 9999)。
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
        if is_key_pressed(KeyCode::B) && disconnected.is_none() {
            let entity = EntityId::from_parts(SPAWN_ORIGIN, spawn_seq);
            spawn_seq = spawn_seq.wrapping_add(1);
            let p = camera.position
                + camera.forward() * SPAWN_AHEAD
                + macroquad::math::Vec3::Y * SPAWN_LIFT;
            control.command(SimCommand::SpawnDynamic {
                entity,
                state: EntityState {
                    translation: types::glam::Vec3::new(p.x, p.y, p.z),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            });
        }
        // X:准星处引爆(把立方体炸飞;ACV4 ③)。
        if is_key_pressed(KeyCode::X) && disconnected.is_none() {
            let c = camera.position + camera.forward() * BLAST_AHEAD;
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
        for mesh in world.chunks.values() {
            draw_mesh(mesh);
        }
        for translation in world.entities.values() {
            draw_cube(
                *translation,
                vec3(ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE, ENTITY_CUBE_SIZE),
                None,
                BLUE,
            );
        }
        set_default_camera();

        // —— HUD(2D 默认相机;四行)——
        let lines = hud_lines(
            world.tick,
            world.chunks.len(),
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

/// 窗口配置(标题/尺寸;crate 根 `#[macroquad::main(window_conf)]` 入口用)。
/// 测试目标下不可达(dead_code 实警),allow 并注明。
#[allow(dead_code)] // 测试目标(cargo test)不含窗口入口;bin 装配需要
pub(crate) fn window_conf() -> Conf {
    Conf {
        window_title: "physics_minecraft viewer(REQ-004A demo)".to_owned(),
        window_width: 1280,
        window_height: 720,
        window_resizable: true,
        ..Conf::default()
    }
}

/// 窗口入口体(crate 根的宏展开 `main` 调用 `amain()`):读环境变量 →
/// run_viewer。服务器地址 = `VIEWER_ADDR`(缺省 127.0.0.1:25565,REQ-004A
/// 决策 4);视距 = `VIEWER_VIEW`(缺省 4,REQ-004A §2)。
#[allow(dead_code)] // 测试目标不可达(同上)
pub(crate) async fn amain() {
    let addr_str = std::env::var("VIEWER_ADDR").unwrap_or_else(|_| "127.0.0.1:25565".to_owned());
    let view_distance = std::env::var("VIEWER_VIEW")
        .ok()
        .and_then(|v| v.parse::<u8>().ok())
        .unwrap_or(4);
    let addr: std::net::SocketAddr = addr_str
        .parse()
        .unwrap_or_else(|e| panic!("VIEWER_ADDR 无效({addr_str}):{e}"));
    run_viewer(addr, view_distance).await;
}
