//! physics_minecraft Bevy 0.20.1 客户端壳(REQ-013 C01b-1 §3)。
//!
//! 纯渲染与输入编排:世界镜像与 subchunk 网格生成全在 `client-core`,
//! 本 crate 负责 CLI、LocalTransport 装配、同步网格上传、飞行相机、光照与 F2 截图。

mod conv;

use std::collections::BTreeMap;
use std::path::PathBuf;

use app::config::Config;
use app::local::LocalConfig;
use bevy::app::AppExit;
use bevy::asset::RenderAssetUsages;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::light::GlobalAmbientLight;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow, WindowResolution};
use client_core::mesh::{SubchunkMesh, mesh_subchunk};
use client_core::mirror::{ChunkMirror, SECTIONS_PER_CHUNK, SubchunkPos};
use client_core::{LocalTransport, Transport};
use sim_api::proto::{C2S, S2C};
use types::ChunkPos;

/// 飞行相机移动速度(m/s,沿用 viewer `MOVE_SPEED`)。
const MOVE_SPEED: f32 = 16.0;
/// 鼠标视角灵敏度(rad/像素)。
const LOOK_SENSITIVITY: f32 = 0.003;
/// C01b-1 同步 meshing 每帧脏段上限。
const MAX_DIRTY_PER_FRAME: usize = 32;
/// 飞行相机初始位置。
const START_CAMERA_POS: Vec3 = Vec3::new(8.0, -48.0, 24.0);
/// 飞行相机初始注视点(俯角约 30°)。
const START_CAMERA_LOOK_AT: Vec3 = Vec3::new(8.0, -61.0, 0.0);

/// 解析后的命令行参数。
struct CliArgs {
    world_dir: PathBuf,
    view_distance: u8,
}

fn parse_cli() -> Result<CliArgs, String> {
    let mut args = std::env::args().skip(1);
    let mut world_dir: Option<PathBuf> = None;
    let mut view_distance: u8 = 8;

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
                view_distance = val
                    .parse::<u8>()
                    .map_err(|e| format!("--view-distance 无效({val}): {e}"))?;
            }
            "-h" | "--help" => {
                return Err(
                    "用法: client --world-dir <PATH> [--view-distance <u8, 默认 8>]".to_owned(),
                );
            }
            other => {
                return Err(format!("未知参数: {other}"));
            }
        }
    }

    let world_dir = world_dir
        .ok_or_else(|| "缺少必填参数 --world-dir <PATH>(禁止默认触碰仓库根 data/)".to_owned())?;
    Ok(CliArgs {
        world_dir,
        view_distance,
    })
}

/// 进程内传输资源:App 退出释放资源时自动触发 `LocalTransport::drop`(Goodbye + join)。
#[derive(Resource)]
struct TransportRes {
    transport: LocalTransport,
}

/// 世界体素镜像与已上传 subchunk 实体映射。
#[derive(Resource)]
struct WorldRenderRes {
    mirror: ChunkMirror,
    entities: BTreeMap<SubchunkPos, Entity>,
    terrain_material: Handle<StandardMaterial>,
}

/// 观察者 chunk 追踪器:相机跨越 chunk 时上报 `MoveObserver`。
#[derive(Resource, Default)]
struct ObserverTracker {
    last_chunk: Option<ChunkPos>,
}

/// 截图目录与递增序号(裁定 §3:存 `<world-dir>/screenshots/client-<序号>.png`)。
#[derive(Resource)]
struct ScreenshotState {
    dir: PathBuf,
    next_seq: u32,
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
}

fn main() {
    let cli = match parse_cli() {
        Ok(c) => c,
        Err(msg) => {
            eprintln!("参数错误: {msg}");
            std::process::exit(2);
        }
    };

    // bodies_dir / policies_dir 沿用 app Config::default() 口径(assets/bodies, assets/policies)。
    let local_cfg = LocalConfig {
        config: Config {
            data_dir: cli.world_dir.clone(),
            ..Config::default()
        },
        view_distance: cli.view_distance,
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

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
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
        .insert_resource(TransportRes { transport })
        .insert_resource(ObserverTracker::default())
        .insert_resource(ScreenshotState {
            dir: screenshot_dir,
            next_seq: 1,
        })
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                fly_camera_system,
                sync_observer_system,
                drain_transport_system,
                pump_dirty_meshes_system,
                screenshot_system,
            )
                .chain(),
        )
        .run();
}

/// 初始化材质、飞行相机、平行光与初始鼠标锁定。
fn setup_scene(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
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
    let fly = FlyCamera::from_look(START_CAMERA_POS, START_CAMERA_LOOK_AT);
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

    if let Ok(mut cursor) = cursor_q.single_mut() {
        cursor.visible = false;
        cursor.grab_mode = CursorGrabMode::Locked;
    }
}

/// 飞行相机控制:鼠标转视角、WASD 水平移动、Shift/Ctrl 升降、M 切换鼠标锁定、Esc 退出。
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

/// 相机所在 chunk 变化时上报 `C2S::MoveObserver`。
fn sync_observer_system(
    mut transport_res: ResMut<TransportRes>,
    mut tracker: ResMut<ObserverTracker>,
    cam_q: Query<&Transform, With<FlyCamera>>,
) {
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
        transport_res.transport.send(C2S::MoveObserver {
            position: world_pos,
        });
    }
}

/// 消费宿主 S2C 帧更新世界镜像,并在 chunk 卸载时回收对应 subchunk 渲染实体。
fn drain_transport_system(
    mut commands: Commands,
    mut transport_res: ResMut<TransportRes>,
    mut world_res: ResMut<WorldRenderRes>,
) {
    for msg in transport_res.transport.drain() {
        match msg {
            S2C::ChunkData { pos, chunk } => {
                world_res.mirror.insert_chunk(pos, chunk);
            }
            S2C::BlockUpdates { updates } => {
                world_res.mirror.apply_updates(&updates);
            }
            S2C::ChunkUnload { pos } => {
                world_res.mirror.remove_chunk(pos);
                for section in 0..SECTIONS_PER_CHUNK {
                    let sc = SubchunkPos {
                        chunk: pos,
                        section,
                    };
                    if let Some(entity) = world_res.entities.remove(&sc) {
                        commands.entity(entity).despawn();
                    }
                }
            }
            _ => {}
        }
    }
}

/// 每帧最多弹出 32 个脏段同步构建并上传网格。
fn pump_dirty_meshes_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut world_res: ResMut<WorldRenderRes>,
) {
    let material = world_res.terrain_material.clone();
    for _ in 0..MAX_DIRTY_PER_FRAME {
        let Some((sc, _kind)) = world_res.mirror.pop_dirty() else {
            break;
        };
        let submesh = mesh_subchunk(&world_res.mirror, sc);
        if submesh.indices.is_empty() {
            if let Some(entity) = world_res.entities.remove(&sc) {
                commands.entity(entity).despawn();
            }
            continue;
        }
        let handle = meshes.add(to_bevy_mesh(submesh));
        if let Some(&entity) = world_res.entities.get(&sc) {
            commands.entity(entity).insert(Mesh3d(handle));
        } else {
            let entity = commands
                .spawn((
                    Mesh3d(handle),
                    MeshMaterial3d(material.clone()),
                    Transform::IDENTITY,
                ))
                .id();
            world_res.entities.insert(sc, entity);
        }
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

/// F2 截图保存到 `<world-dir>/screenshots/client-<序号>.png`(裁定 §3)。
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
