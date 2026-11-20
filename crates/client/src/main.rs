//! physics_minecraft Bevy 0.20.1 客户端壳(REQ-013 C01b-1/C01b-2)。
//!
//! 纯渲染与输入编排:世界镜像与 subchunk 网格生成全在 `client-core`,
//! 本 crate 负责 CLI(交互与脚本冒烟模式)、LocalTransport 装配、
//! `AsyncComputeTaskPool` 异步网格上传、飞行相机、光照与截图。

mod conv;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use app::config::Config;
use app::local::LocalConfig;
use bevy::app::AppExit;
use bevy::asset::RenderAssetUsages;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::light::GlobalAmbientLight;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, ScreenshotCaptured, save_to_disk};
use bevy::tasks::{AsyncComputeTaskPool, Task, futures::check_ready};
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow, WindowResolution};
use client_core::client::{ViewEvent, s2c_to_event};
use client_core::mesh::{SubchunkMesh, mesh_subchunk_snapshot, snapshot};
use client_core::mirror::{
    ChunkMirror, DirtyKind, SECTION_HEIGHT, SECTIONS_PER_CHUNK, SubchunkPos, is_stale,
};
use client_core::script::manifest::{Manifest, ManifestMeshStats};
use client_core::script::{
    CameraPose, Engine, EngineCommand, EngineStatus, FrameInput, Scenario, parse_scenario,
};
use client_core::world::WorldState;
use client_core::{LocalTransport, Transport, config_from_server_table};
use messaging::SimCommand;
use sim_api::proto::{C2S, S2C};
use types::{BlockPos, CHUNK_SIZE, ChunkPos, WORLD_Y_MAX, WORLD_Y_MIN};

/// 飞行相机移动速度(m/s,沿用 viewer `MOVE_SPEED`)。
const MOVE_SPEED: f32 = 16.0;
/// 鼠标视角灵敏度(rad/像素)。
const LOOK_SENSITIVITY: f32 = 0.003;
/// C01b-2 异步 meshing 每帧提交任务上限。
const MAX_SPAWN_PER_FRAME: usize = 8;
/// 飞行相机初始位置。
const START_CAMERA_POS: Vec3 = Vec3::new(8.0, -48.0, 24.0);
/// 飞行相机初始注视点(俯角约 30°)。
const START_CAMERA_LOOK_AT: Vec3 = Vec3::new(8.0, -61.0, 0.0);
/// 脚本模式观察者位移立即上报阈值(m,同 viewer `OBSERVER_JUMP_METERS`)。
const OBSERVER_JUMP_METERS: f32 = 1.0;

/// 运行模式:交互飞行或脚本冒烟。
enum RunMode {
    Interactive,
    Script { script: PathBuf, out: PathBuf },
}

/// 解析后的命令行参数。
struct CliArgs {
    world_dir: PathBuf,
    view_distance: Option<u8>,
    mode: RunMode,
}

fn parse_cli() -> Result<CliArgs, String> {
    let mut args = std::env::args().skip(1);
    let mut world_dir: Option<PathBuf> = None;
    let mut view_distance: Option<u8> = None;
    let mut script: Option<PathBuf> = None;
    let mut out: Option<PathBuf> = None;

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--world-dir" => {
                let val = args
                    .next()
                    .ok_or_else(|| "--world-dir 缺少路径参数".to_owned())?;
                world_dir = Some(PathBuf::from(val));
            }
            "--view-distance" => {
                let val = args
                    .next()
                    .ok_or_else(|| "--view-distance 缺少数值参数".to_owned())?;
                let parsed = val
                    .parse::<u8>()
                    .map_err(|e| format!("--view-distance 无效({val}): {e}"))?;
                view_distance = Some(parsed);
            }
            "--script" => {
                let val = args
                    .next()
                    .ok_or_else(|| "--script 缺少路径参数".to_owned())?;
                script = Some(PathBuf::from(val));
            }
            "--out" => {
                let val = args.next().ok_or_else(|| "--out 缺少目录参数".to_owned())?;
                out = Some(PathBuf::from(val));
            }
            "-h" | "--help" => {
                return Err(
                    "用法: client --world-dir <PATH> [--view-distance <u8, 默认 8>] [--script <TOML> --out <DIR>]"
                        .to_owned(),
                );
            }
            other => {
                return Err(format!("未知参数: {other}"));
            }
        }
    }

    let world_dir = world_dir
        .ok_or_else(|| "缺少必填参数 --world-dir <PATH>(禁止默认触碰仓库根 data/)".to_owned())?;
    let mode = match (script, out) {
        (None, None) => RunMode::Interactive,
        (Some(script), Some(out)) => RunMode::Script { script, out },
        (Some(_), None) => return Err("--script 必须与 --out 同时给出".to_owned()),
        (None, Some(_)) => return Err("--out 必须与 --script 同时给出".to_owned()),
    };
    Ok(CliArgs {
        world_dir,
        view_distance,
        mode,
    })
}

/// 进程内传输资源:App 退出或脚本结束显式释放时触发 `LocalTransport::drop`(Goodbye + join)。
#[derive(Resource)]
struct TransportRes {
    transport: Option<LocalTransport>,
}

/// 世界体素镜像与已上传 subchunk 实体/网格句柄映射。
#[derive(Resource)]
struct WorldRenderRes {
    mirror: ChunkMirror,
    entities: BTreeMap<SubchunkPos, (Entity, Handle<Mesh>)>,
    terrain_material: Handle<StandardMaterial>,
}

/// 自增渲染帧计数器(不读墙钟,供 Edit 延迟测量)。
#[derive(Resource, Default)]
struct FrameCounter(u64);

/// 单个正在后台 `AsyncComputeTaskPool` 中执行的网格任务。
struct PendingMesh {
    kind: DirtyKind,
    epoch: u64,
    task: Task<SubchunkMesh>,
}

/// 在途异步网格任务表。
#[derive(Resource, Default)]
struct PendingMeshesRes {
    tasks: BTreeMap<SubchunkPos, PendingMesh>,
}

/// 收到 `BlockUpdates` 时记录各受影响 Edit 段的到达帧号。
#[derive(Resource, Default)]
struct EditFrameTracker {
    frames: BTreeMap<SubchunkPos, u64>,
}

/// 异步网格上传测量钩子(REQ-013 C01b-2 §2.2 / C01b-3 基线复用)。
#[derive(Resource, Default, Debug, Clone, Copy)]
struct MeshStats {
    max_edit_upload_frames: u32,
    uploads: u64,
    discards: u64,
}

impl MeshStats {
    fn to_manifest(self) -> ManifestMeshStats {
        ManifestMeshStats {
            max_edit_upload_frames: self.max_edit_upload_frames,
            uploads: self.uploads,
            discards: self.discards,
        }
    }
}

/// 观察者 chunk 追踪器(交互模式):相机跨越 chunk 时上报 `MoveObserver`。
#[derive(Resource, Default)]
struct ObserverTracker {
    last_chunk: Option<ChunkPos>,
}

/// 截图目录与递增序号(裁定 §3:交互模式存 `<world-dir>/screenshots/client-<序号>.png`)。
#[derive(Resource)]
struct ScreenshotState {
    dir: PathBuf,
    next_seq: u32,
}

/// 脚本冒烟模式运行时状态(仅 `--script --out` 时插入)。
#[derive(Resource)]
struct ScriptModeRes {
    scenario: Scenario,
    out_dir: PathBuf,
    world: WorldState,
    engine: Option<Engine>,
    manifest: Manifest,
    coupled: bool,
    last_observer_pos: Option<Vec3>,
    pending_shots: Arc<AtomicUsize>,
    finished_exit: Option<i32>,
    disconnected: Option<String>,
}

/// 飞行相机状态组件。
#[derive(Component)]
struct FlyCamera {
    yaw: f32,
    pitch: f32,
    cursor_grabbed: bool,
}

impl FlyCamera {
    fn from_look(pos: Vec3, target: Vec3) -> Self {
        let dir = (target - pos).normalize_or_zero();
        let yaw = dir.z.atan2(dir.x);
        let pitch = dir.y.clamp(-1.0, 1.0).asin();
        Self {
            yaw,
            pitch,
            cursor_grabbed: true,
        }
    }

    fn forward(&self) -> Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        Vec3::new(cy * cp, sp, sy * cp)
    }

    fn flat_forward(&self) -> Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        Vec3::new(cy, 0.0, sy)
    }

    fn to_pose(&self, pos: Vec3) -> CameraPose {
        CameraPose {
            pos: conv::from_bevy_vec3(pos),
            yaw: self.yaw.to_degrees(),
            pitch: self.pitch.to_degrees(),
        }
    }
}

fn write_manifest(manifest: &Manifest, out_dir: &Path) {
    let path = out_dir.join("manifest.json");
    if let Err(err) = std::fs::write(&path, manifest.to_json()) {
        eprintln!("写 {} 失败: {err}", path.display());
    }
}

fn main() {
    let cli = match parse_cli() {
        Ok(c) => c,
        Err(msg) => {
            eprintln!("参数错误: {msg}");
            std::process::exit(2);
        }
    };

    let (config, view_distance, script_res) = match cli.mode {
        RunMode::Interactive => {
            // bodies_dir / policies_dir 沿用 app Config::default() 口径(assets/bodies, assets/policies)。
            let cfg = Config {
                data_dir: cli.world_dir.clone(),
                ..Config::default()
            };
            (cfg, cli.view_distance.unwrap_or(8), None)
        }
        RunMode::Script { script, out } => {
            let text = match std::fs::read_to_string(&script) {
                Ok(t) => t,
                Err(err) => {
                    eprintln!("读取场景文件 {} 失败: {err}", script.display());
                    std::process::exit(2);
                }
            };
            let scenario = match parse_scenario(&text) {
                Ok(s) => s,
                Err(err) => {
                    eprintln!("场景解析失败: {err}");
                    std::process::exit(2);
                }
            };
            if let Err(err) = std::fs::create_dir_all(&out) {
                eprintln!("创建输出目录 {} 失败: {err}", out.display());
                std::process::exit(2);
            }
            let cfg = match config_from_server_table(&scenario.server, &cli.world_dir) {
                Ok(c) => c,
                Err(err) => {
                    eprintln!("场景 [server] 配置错误: {err}");
                    std::process::exit(2);
                }
            };
            let vd = cli.view_distance.or(scenario.view_distance).unwrap_or(8);
            let manifest = Manifest::new(&scenario);
            let res = ScriptModeRes {
                scenario,
                out_dir: out,
                world: WorldState::new(),
                engine: None,
                manifest,
                coupled: true,
                last_observer_pos: None,
                pending_shots: Arc::new(AtomicUsize::new(0)),
                finished_exit: None,
                disconnected: None,
            };
            (cfg, vd, Some(res))
        }
    };

    let local_cfg = LocalConfig {
        config,
        view_distance,
    };

    let transport = match LocalTransport::new(local_cfg) {
        Ok(t) => t,
        Err(err) => {
            eprintln!(
                "启动单机宿主失败(world_dir={}): {err}",
                cli.world_dir.display()
            );
            std::process::exit(1);
        }
    };

    let screenshot_dir = cli.world_dir.join("screenshots");

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "physics_minecraft client".to_owned(),
            resolution: WindowResolution::new(1280, 720),
            ..default()
        }),
        ..default()
    }))
    .insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 150.0,
        ..default()
    })
    .insert_resource(TransportRes {
        transport: Some(transport),
    })
    .insert_resource(FrameCounter::default())
    .insert_resource(PendingMeshesRes::default())
    .insert_resource(EditFrameTracker::default())
    .insert_resource(MeshStats::default())
    .insert_resource(ObserverTracker::default())
    .insert_resource(ScreenshotState {
        dir: screenshot_dir,
        next_seq: 1,
    })
    .add_systems(Startup, setup_scene);

    if let Some(script_mode) = script_res {
        app.insert_resource(script_mode).add_systems(
            Update,
            (
                advance_frame_counter_system,
                drain_transport_system,
                collect_mesh_tasks_system,
                script_driver_system,
                spawn_mesh_tasks_system,
            )
                .chain(),
        );
    } else {
        app.add_systems(
            Update,
            (
                advance_frame_counter_system,
                fly_camera_system,
                sync_observer_system,
                drain_transport_system,
                collect_mesh_tasks_system,
                spawn_mesh_tasks_system,
                screenshot_system,
            )
                .chain(),
        );
    }

    app.run();
}

/// 初始化材质、飞行相机、平行光与交互模式初始鼠标锁定(脚本模式不锁定光标)。
fn setup_scene(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    script_mode: Option<Res<ScriptModeRes>>,
    mut cursor_q: Query<&mut CursorOptions, With<PrimaryWindow>>,
) {
    let terrain_material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        perceptual_roughness: 0.9,
        metallic: 0.0,
        ..default()
    });
    commands.insert_resource(WorldRenderRes {
        mirror: ChunkMirror::new(),
        entities: BTreeMap::new(),
        terrain_material,
    });

    // 飞行相机:初始位 (8, -48, 24) 朝向 (8, -61, 0),fov 70°。
    let mut fly = FlyCamera::from_look(START_CAMERA_POS, START_CAMERA_LOOK_AT);
    if script_mode.is_some() {
        fly.cursor_grabbed = false;
    }
    let initial_transform =
        Transform::from_translation(START_CAMERA_POS).looking_at(START_CAMERA_LOOK_AT, Vec3::Y);
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: 70.0_f32.to_radians(),
            ..default()
        }),
        initial_transform,
        fly,
    ));

    // 斜向下约 45° 的平行光。
    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            ..default()
        },
        Transform::from_xyz(30.0, 50.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    if script_mode.is_none()
        && let Ok(mut cursor) = cursor_q.single_mut()
    {
        cursor.visible = false;
        cursor.grab_mode = CursorGrabMode::Locked;
    }
}

/// 每帧推进渲染帧计数器。
fn advance_frame_counter_system(mut frame_counter: ResMut<FrameCounter>) {
    frame_counter.0 = frame_counter.0.saturating_add(1);
}

/// 飞行相机控制(交互模式):鼠标转视角、WASD 水平移动、Shift/Ctrl 升降、M 切换鼠标锁定、Esc 退出。
fn fly_camera_system(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mut exit_writer: MessageWriter<AppExit>,
    mut cursor_q: Query<&mut CursorOptions, With<PrimaryWindow>>,
    mut cam_q: Query<(&mut Transform, &mut FlyCamera)>,
) {
    if keys.just_pressed(KeyCode::Escape) {
        exit_writer.write(AppExit::Success);
        return;
    }

    let Ok((mut transform, mut fly)) = cam_q.single_mut() else {
        return;
    };

    if keys.just_pressed(KeyCode::KeyM) {
        fly.cursor_grabbed = !fly.cursor_grabbed;
        if let Ok(mut cursor) = cursor_q.single_mut() {
            cursor.visible = !fly.cursor_grabbed;
            cursor.grab_mode = if fly.cursor_grabbed {
                CursorGrabMode::Locked
            } else {
                CursorGrabMode::None
            };
        }
    }

    let delta = mouse_motion.delta;
    if (fly.cursor_grabbed || mouse_buttons.pressed(MouseButton::Right)) && delta != Vec2::ZERO {
        fly.yaw += delta.x * LOOK_SENSITIVITY;
        fly.pitch = (fly.pitch - delta.y * LOOK_SENSITIVITY)
            .clamp(-89.0_f32.to_radians(), 89.0_f32.to_radians());
    }

    let flat_fwd = fly.flat_forward();
    let right = flat_fwd.cross(Vec3::Y);
    let mut move_dir = Vec3::ZERO;
    if keys.pressed(KeyCode::KeyW) {
        move_dir += flat_fwd;
    }
    if keys.pressed(KeyCode::KeyS) {
        move_dir -= flat_fwd;
    }
    if keys.pressed(KeyCode::KeyD) {
        move_dir += right;
    }
    if keys.pressed(KeyCode::KeyA) {
        move_dir -= right;
    }
    if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
        move_dir += Vec3::Y;
    }
    if keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight) {
        move_dir -= Vec3::Y;
    }

    let dt = time.delta_secs();
    if move_dir != Vec3::ZERO {
        transform.translation += move_dir.normalize_or_zero() * MOVE_SPEED * dt;
    }

    let look_target = transform.translation + fly.forward();
    transform.look_at(look_target, Vec3::Y);
}

/// 相机所在 chunk 变化时上报 `C2S::MoveObserver`(交互模式)。
fn sync_observer_system(
    mut transport_res: ResMut<TransportRes>,
    mut tracker: ResMut<ObserverTracker>,
    cam_q: Query<&Transform, With<FlyCamera>>,
) {
    let Some(transport) = transport_res.transport.as_mut() else {
        return;
    };
    let Ok(transform) = cam_q.single() else {
        return;
    };
    let world_pos = conv::from_bevy_vec3(transform.translation);
    let chunk = ChunkPos::new(
        (world_pos.x / 16.0).floor() as i32,
        (world_pos.z / 16.0).floor() as i32,
    );
    if tracker.last_chunk != Some(chunk) {
        tracker.last_chunk = Some(chunk);
        transport.send(C2S::MoveObserver {
            position: world_pos,
        });
    }
}

/// 记录受 `BlockUpdates` 影响的 subchunk(含存在的边界邻段)到达帧号。
fn record_edit_frames(
    mirror: &ChunkMirror,
    updates: &[(BlockPos, types::BlockId)],
    frame: u64,
    tracker: &mut EditFrameTracker,
) {
    for &(world, _) in updates {
        if !(WORLD_Y_MIN..=WORLD_Y_MAX).contains(&world.y) {
            continue;
        }
        let chunk_pos = world.chunk_of();
        if mirror.chunk_arc(chunk_pos).is_none() {
            continue;
        }
        let local_x = world.x.rem_euclid(CHUNK_SIZE) as usize;
        let local_y = (world.y - WORLD_Y_MIN) as usize;
        let local_z = world.z.rem_euclid(CHUNK_SIZE) as usize;
        let section = (local_y / SECTION_HEIGHT) as u8;

        tracker.frames.insert(
            SubchunkPos {
                chunk: chunk_pos,
                section,
            },
            frame,
        );

        if local_x == 0 {
            let n = ChunkPos::new(chunk_pos.x - 1, chunk_pos.z);
            if mirror.chunk_arc(n).is_some() {
                tracker
                    .frames
                    .insert(SubchunkPos { chunk: n, section }, frame);
            }
        } else if local_x == 15 {
            let n = ChunkPos::new(chunk_pos.x + 1, chunk_pos.z);
            if mirror.chunk_arc(n).is_some() {
                tracker
                    .frames
                    .insert(SubchunkPos { chunk: n, section }, frame);
            }
        }

        if local_z == 0 {
            let n = ChunkPos::new(chunk_pos.x, chunk_pos.z - 1);
            if mirror.chunk_arc(n).is_some() {
                tracker
                    .frames
                    .insert(SubchunkPos { chunk: n, section }, frame);
            }
        } else if local_z == 15 {
            let n = ChunkPos::new(chunk_pos.x, chunk_pos.z + 1);
            if mirror.chunk_arc(n).is_some() {
                tracker
                    .frames
                    .insert(SubchunkPos { chunk: n, section }, frame);
            }
        }

        let y_in_sec = local_y % SECTION_HEIGHT;
        if y_in_sec == 0 && section > 0 {
            tracker.frames.insert(
                SubchunkPos {
                    chunk: chunk_pos,
                    section: section - 1,
                },
                frame,
            );
        } else if y_in_sec == 15 && section + 1 < SECTIONS_PER_CHUNK {
            tracker.frames.insert(
                SubchunkPos {
                    chunk: chunk_pos,
                    section: section + 1,
                },
                frame,
            );
        }
    }
}

/// 消费宿主 S2C 帧更新世界镜像(脚本模式同时喂 `WorldState`),并在 chunk 卸载时回收实体/资产/任务。
#[allow(clippy::too_many_arguments)]
fn drain_transport_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    frame_counter: Res<FrameCounter>,
    mut transport_res: ResMut<TransportRes>,
    mut world_res: ResMut<WorldRenderRes>,
    mut pending: ResMut<PendingMeshesRes>,
    mut edit_frames: ResMut<EditFrameTracker>,
    mut script_mode: Option<ResMut<ScriptModeRes>>,
) {
    let Some(transport) = transport_res.transport.as_mut() else {
        return;
    };
    for msg in transport.drain() {
        if let Some(script) = script_mode.as_mut()
            && let Some(ev) = s2c_to_event(msg.clone())
        {
            match ev {
                ViewEvent::Disconnected { reason } => script.disconnected = Some(reason),
                other => script.world.apply(other),
            }
        }

        match msg {
            S2C::ChunkData { pos, chunk } => {
                world_res.mirror.insert_chunk(pos, chunk);
            }
            S2C::BlockUpdates { updates } => {
                world_res.mirror.apply_updates(&updates);
                record_edit_frames(
                    &world_res.mirror,
                    &updates,
                    frame_counter.0,
                    &mut edit_frames,
                );
            }
            S2C::ChunkUnload { pos } => {
                world_res.mirror.remove_chunk(pos);
                for section in 0..SECTIONS_PER_CHUNK {
                    let sc = SubchunkPos {
                        chunk: pos,
                        section,
                    };
                    pending.tasks.remove(&sc);
                    edit_frames.frames.remove(&sc);
                    if let Some((entity, old_handle)) = world_res.entities.remove(&sc) {
                        meshes.remove(&old_handle);
                        commands.entity(entity).despawn();
                    }
                }
            }
            _ => {}
        }
    }
}

/// 轮询在途异步网格任务:世代匹配则上传/替换/回收空段并更新 `MeshStats`;过期则丢弃并回队。
fn collect_mesh_tasks_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    frame_counter: Res<FrameCounter>,
    mut world_res: ResMut<WorldRenderRes>,
    mut pending: ResMut<PendingMeshesRes>,
    mut edit_frames: ResMut<EditFrameTracker>,
    mut stats: ResMut<MeshStats>,
) {
    let material = world_res.terrain_material.clone();
    let mut completed: Vec<(SubchunkPos, DirtyKind, u64, SubchunkMesh)> = Vec::new();

    pending.tasks.retain(|&sc, entry| {
        if let Some(submesh) = check_ready(&mut entry.task) {
            completed.push((sc, entry.kind, entry.epoch, submesh));
            false
        } else {
            true
        }
    });

    for (sc, kind, pending_epoch, submesh) in completed {
        let current_epoch = world_res.mirror.epoch(sc);
        if is_stale(pending_epoch, current_epoch) {
            stats.discards += 1;
            world_res.mirror.requeue(sc, kind);
            continue;
        }

        // 若该 chunk 已卸载,不上传残留网格。
        if world_res.mirror.chunk_arc(sc.chunk).is_none() {
            edit_frames.frames.remove(&sc);
            continue;
        }

        if submesh.indices.is_empty() {
            if let Some((entity, old_handle)) = world_res.entities.remove(&sc) {
                meshes.remove(&old_handle);
                commands.entity(entity).despawn();
            }
        } else {
            let new_handle = meshes.add(to_bevy_mesh(submesh));
            if let Some((entity, old_handle)) = world_res.entities.get_mut(&sc) {
                meshes.remove(&*old_handle);
                *old_handle = new_handle.clone();
                commands.entity(*entity).insert(Mesh3d(new_handle));
            } else {
                let entity = commands
                    .spawn((
                        Mesh3d(new_handle.clone()),
                        MeshMaterial3d(material.clone()),
                        Transform::IDENTITY,
                    ))
                    .id();
                world_res.entities.insert(sc, (entity, new_handle));
            }
        }

        stats.uploads += 1;
        if kind == DirtyKind::Edit
            && let Some(arrived_frame) = edit_frames.frames.remove(&sc)
        {
            let diff = frame_counter.0.saturating_sub(arrived_frame) as u32;
            stats.max_edit_upload_frames = stats.max_edit_upload_frames.max(diff);
        }
    }
}

/// 将脚本引擎输出的 `EngineCommand` 列表翻译为 `C2S` 并经 `Transport` 发送
/// (与 `viewer/src/render.rs:739-796` `forward_commands` 同口径)。
fn forward_engine_commands(commands: &[EngineCommand], transport: &mut LocalTransport) {
    let mut batch: Vec<SimCommand> = Vec::new();
    let flush_batch = |batch: &mut Vec<SimCommand>, transport: &mut LocalTransport| {
        for sim_cmd in batch.drain(..) {
            transport.send(C2S::Command(sim_cmd));
        }
    };
    for command in commands {
        match command {
            EngineCommand::Spawn { state, desc } => {
                flush_batch(&mut batch, transport);
                transport.send(C2S::Spawn {
                    state: *state,
                    desc: desc.clone(),
                });
            }
            EngineCommand::SpawnAgent { position, yaw } => {
                flush_batch(&mut batch, transport);
                transport.send(C2S::SpawnAgent {
                    body: "capsule".to_owned(),
                    position: *position,
                    yaw: *yaw,
                });
            }
            EngineCommand::Possess { entity } => {
                flush_batch(&mut batch, transport);
                transport.send(C2S::Possess { entity: *entity });
            }
            EngineCommand::Unpossess => {
                flush_batch(&mut batch, transport);
                transport.send(C2S::Unpossess);
            }
            EngineCommand::AgentIntent {
                entity,
                move_xz,
                yaw,
                jump,
            } => {
                flush_batch(&mut batch, transport);
                transport.send(C2S::AgentIntent {
                    entity: *entity,
                    move_xz: *move_xz,
                    yaw: *yaw,
                    jump: *jump,
                });
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
    flush_batch(&mut batch, transport);
}

/// 脚本冒烟模式主驱动系统:等 `tick_known` → 推进 `Engine` → 相机/命令/截图/manifest → 优雅收口退出。
#[allow(clippy::too_many_arguments)]
fn script_driver_system(
    mut commands: Commands,
    time: Res<Time>,
    stats: Res<MeshStats>,
    mut transport_res: ResMut<TransportRes>,
    mut script: ResMut<ScriptModeRes>,
    mut cam_q: Query<(&mut Transform, &mut FlyCamera)>,
) {
    // 场景已结束:等待最后一张异步截图写盘完毕,再写终版 manifest、关闭宿主并按 exit_code 退出。
    if let Some(code) = script.finished_exit {
        if script.pending_shots.load(Ordering::SeqCst) > 0 {
            return;
        }
        script.manifest.mesh_stats = Some(stats.to_manifest());
        write_manifest(&script.manifest, &script.out_dir);
        // 显式 drop LocalTransport → 发送 Goodbye 并 join 宿主线程,确保 redb 收口。
        let _ = transport_res.transport.take();
        std::process::exit(code);
    }

    // 上一步的异步截图尚在渲染回读中:暂缓推进下一步,保证截得该步骤对应的准确画面。
    if script.pending_shots.load(Ordering::SeqCst) > 0 {
        return;
    }

    let Ok((mut transform, mut fly)) = cam_q.single_mut() else {
        return;
    };

    // 若尚未收到第一个 EntityStates 心跳(tick_known),先上报初始观察者位姿让分片注册发心跳。
    if !script.world.tick_known() {
        if script.last_observer_pos.is_none()
            && let Some(transport) = transport_res.transport.as_mut()
        {
            let pos = transform.translation;
            script.last_observer_pos = Some(pos);
            transport.send(C2S::MoveObserver {
                position: conv::from_bevy_vec3(pos),
            });
        }
        return;
    }

    // tick 已知后首次创建脚本引擎。
    if script.engine.is_none() {
        let start_pose = fly.to_pose(transform.translation);
        script.engine = Some(Engine::new(script.scenario.clone(), start_pose));
    }

    // 结束前收到断连:Failed 收尾(退出码 2)。
    if let Some(reason) = script.disconnected.clone() {
        let step = match script.engine.as_ref().map(Engine::status) {
            Some(EngineStatus::Running { step }) => *step,
            _ => 1,
        };
        let status = EngineStatus::Failed {
            step,
            reason: format!("断连:{reason}"),
        };
        script.manifest.finish(&status);
        script.manifest.mesh_stats = Some(stats.to_manifest());
        write_manifest(&script.manifest, &script.out_dir);
        script.finished_exit = Some(2);
        return;
    }

    let spawned = script.world.take_spawned();
    let tick = script.world.tick();
    let dt = time.delta_secs();
    let (output, final_status, exit_code) = {
        let ScriptModeRes { engine, world, .. } = &mut *script;
        let eng = engine.as_mut().expect("engine 已初始化");
        let input = FrameInput {
            dt,
            server_tick: tick,
            world,
            spawned: &spawned,
        };
        let output = eng.frame(&input);
        let status = eng.status().clone();
        let code = eng.exit_code();
        (output, status, code)
    };

    // 应用引擎输出的相机位姿。
    let cam_pos = conv::to_bevy_vec3(output.camera.pos);
    fly.yaw = output.camera.yaw.to_radians();
    fly.pitch = output
        .camera
        .pitch
        .to_radians()
        .clamp(-89.0_f32.to_radians(), 89.0_f32.to_radians());
    transform.translation = cam_pos;
    let look_target = cam_pos + fly.forward();
    transform.look_at(look_target, Vec3::Y);

    script.coupled = output.coupled;
    if let Some(transport) = transport_res.transport.as_mut() {
        forward_engine_commands(&output.commands, transport);
        if script.coupled {
            let need_send = match script.last_observer_pos {
                None => true,
                Some(prev) => (cam_pos - prev).length() > OBSERVER_JUMP_METERS,
            };
            if need_send {
                script.last_observer_pos = Some(cam_pos);
                transport.send(C2S::MoveObserver {
                    position: conv::from_bevy_vec3(cam_pos),
                });
            }
        }
    }

    if !output.finished.is_empty() {
        let ScriptModeRes {
            manifest,
            world,
            out_dir,
            ..
        } = &mut *script;
        for record in &output.finished {
            manifest.push(record, world);
        }
        manifest.mesh_stats = Some(stats.to_manifest());
        write_manifest(manifest, out_dir);
    }

    if let Some(shot) = output.screenshot {
        let path = script.out_dir.join(shot.file_name());
        script.pending_shots.fetch_add(1, Ordering::SeqCst);
        let done = Arc::clone(&script.pending_shots);
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path))
            .observe(move |_: On<ScreenshotCaptured>| {
                done.fetch_sub(1, Ordering::SeqCst);
            });
    }

    if !matches!(final_status, EngineStatus::Running { .. }) {
        script.manifest.finish(&final_status);
        script.manifest.mesh_stats = Some(stats.to_manifest());
        write_manifest(&script.manifest, &script.out_dir);
        script.finished_exit = Some(exit_code.unwrap_or(2));
    }
}

/// 每帧最多从脏队弹出 8 段提交到 `AsyncComputeTaskPool`;已有在途任务时跳过并回队防堆积。
fn spawn_mesh_tasks_system(
    mut world_res: ResMut<WorldRenderRes>,
    mut pending: ResMut<PendingMeshesRes>,
) {
    let pool = AsyncComputeTaskPool::get();
    let mut deferred: Vec<(SubchunkPos, DirtyKind)> = Vec::new();
    let mut spawned = 0usize;

    while spawned < MAX_SPAWN_PER_FRAME {
        let Some((sc, kind)) = world_res.mirror.pop_dirty() else {
            break;
        };
        if pending.tasks.contains_key(&sc) {
            deferred.push((sc, kind));
            continue;
        }
        let Some(snap) = snapshot(&world_res.mirror, sc) else {
            continue;
        };
        let epoch = world_res.mirror.epoch(sc);
        let task = pool.spawn(async move { mesh_subchunk_snapshot(&snap, sc) });
        pending.tasks.insert(sc, PendingMesh { kind, epoch, task });
        spawned += 1;
    }

    for (sc, kind) in deferred {
        world_res.mirror.requeue(sc, kind);
    }
}

/// 将 `client-core` 的 [`SubchunkMesh`] 转为 Bevy [`Mesh`]。
fn to_bevy_mesh(submesh: SubchunkMesh) -> Mesh {
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, submesh.positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, submesh.normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, submesh.colors)
    .with_inserted_indices(Indices::U32(submesh.indices))
}

/// F2 截图保存到 `<world-dir>/screenshots/client-<序号>.png`(裁定 §3,仅交互模式)。
fn screenshot_system(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<ScreenshotState>,
) {
    if !keys.just_pressed(KeyCode::F2) {
        return;
    }
    if let Err(err) = std::fs::create_dir_all(&state.dir) {
        eprintln!("创建截图目录 {} 失败: {err}", state.dir.display());
        return;
    }
    let mut seq = state.next_seq;
    let mut path = state.dir.join(format!("client-{seq:04}.png"));
    while path.exists() {
        seq += 1;
        path = state.dir.join(format!("client-{seq:04}.png"));
    }
    state.next_seq = seq + 1;
    let display_path = path.display().to_string();
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path));
    println!("截图已触发: {display_path}");
}
