//! physics_minecraft Bevy 0.20.1 客户端壳(REQ-013 C01b-1/C01b-2)。
//!
//! 纯渲染与输入编排:世界镜像与 subchunk 网格生成全在 `client-core`,
//! 本 crate 负责 CLI(交互与脚本冒烟模式)、LocalTransport 装配、
//! `AsyncComputeTaskPool` 异步网格上传、飞行相机、光照与截图。

mod conv;
mod entities;

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use app::config::Config;
use app::local::LocalConfig;
use bevy::app::AppExit;
use bevy::asset::{AssetEventSystems, RenderAssetUsages};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::light::GlobalAmbientLight;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, ScreenshotCaptured, save_to_disk};
use bevy::tasks::{AsyncComputeTaskPool, Task, futures::check_ready};
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow, WindowResolution};
use client_core::client::{ViewEvent, s2c_to_event};
use client_core::mesh::{ChunkSnapshot, SubchunkMesh, mesh_subchunk_snapshot, snapshot};
use client_core::mirror::{
    ChunkMirror, DirtyKind, SECTION_HEIGHT, SECTIONS_PER_CHUNK, SubchunkPos, is_stale,
};
use client_core::perf::{MeshPerf, acc_mesh_perf, fps_summary, storm_window};
use client_core::possess::{self, PossessState, p_press_hint};
use client_core::script::manifest::{Manifest, ManifestMeshStats, ManifestPerf};
use client_core::script::{
    CameraPose, Engine, EngineCommand, EngineStatus, FrameInput, Scenario, ScreenshotRequest,
    WorldQuery, engine_commands_to_c2s, parse_scenario,
};
use client_core::tools::{
    Hit, ToolAction, ToolState, dig, hud_text, pick_at_camera, place, select_material,
};
use client_core::world::WorldState;
use client_core::{LocalTransport, SequencedTransport, Transport, config_from_server_table};
use entities::{EntityMirrorRes, EntityRenderRes, sync_entities_system};
use messaging::SimCommand;
use sim_api::proto::{C2S, S2C};
use types::{BlockPos, CHUNK_SIZE, ChunkPos, EntityId, WORLD_Y_MAX, WORLD_Y_MIN};

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
/// FPS 帧耗时环形历史上限(REQ-013 C01b-3 §1.4:7200 帧 ≈ 60 Hz 下 120 s)。
const FPS_HISTORY_CAP: usize = 7200;
/// 交互工具提示生存时间(与 `viewer` `tools::NOTICE_SECS` 同为 3 秒)。
const NOTICE_TTL: Duration = Duration::from_secs(3);

/// 运行模式:交互飞行或脚本冒烟。
enum RunMode {
    Interactive,
    Script { script: PathBuf, out: PathBuf },
}

/// 解析后的命令行参数。
struct CliArgs {
    world_dir: PathBuf,
    view_distance: Option<u8>,
    chunk_cap: Option<usize>,
    mode: RunMode,
}

fn parse_cli() -> Result<CliArgs, String> {
    let mut args = std::env::args().skip(1);
    let mut world_dir: Option<PathBuf> = None;
    let mut view_distance: Option<u8> = None;
    let mut chunk_cap: Option<usize> = None;
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
            "--chunk-cap" => {
                let val = args
                    .next()
                    .ok_or_else(|| "--chunk-cap 缺少数值参数".to_owned())?;
                let parsed = val
                    .parse::<usize>()
                    .map_err(|e| format!("--chunk-cap 无效({val}): {e}"))?;
                chunk_cap = Some(parsed);
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
                    "用法: client --world-dir <PATH> [--view-distance <u8, 默认 8>] [--chunk-cap <usize>] [--script <TOML> --out <DIR>]"
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
        chunk_cap,
        mode,
    })
}

/// 进程内或测试假传输资源:App 退出或脚本结束显式释放时触发底层 `Drop`(Goodbye + join)。
#[derive(Resource)]
struct TransportRes {
    transport: Option<Mutex<Box<dyn Transport + Send>>>,
}

impl TransportRes {
    fn new(t: impl Transport + Send + 'static) -> Self {
        Self {
            transport: Some(Mutex::new(Box::new(t))),
        }
    }

    fn with_mut<R>(&mut self, f: impl FnOnce(&mut dyn Transport) -> R) -> Option<R> {
        let mtx = self.transport.as_mut()?;
        let guard = mtx.get_mut().expect("TransportRes mutex poisoned");
        Some(f(guard.as_mut()))
    }

    fn close(&mut self) {
        let _ = self.transport.take();
    }
}

/// 视距与镜像 LRU chunk 上限配置(REQ-013 C02-2 D1b;交互与脚本模式均插入)。
#[derive(Resource, Debug, Clone, Copy)]
struct ViewCapRes {
    view_distance: u8,
    chunk_cap: usize,
}

impl ViewCapRes {
    fn default_cap(view_distance: u8) -> usize {
        let side = 2usize * usize::from(view_distance) + 1;
        2 * side * side
    }
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

/// 单个正在后台 `AsyncComputeTaskPool` 中执行的网格任务(返回网格与构建微秒耗时)。
struct PendingMesh {
    kind: DirtyKind,
    epoch: u64,
    task: Task<(SubchunkMesh, u64)>,
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

/// 异步网格上传与卸载/驱逐测量钩子(REQ-013 C01b-2 §2.2 / C02-2 D1b + R2-3 基线)。
#[derive(Resource, Default, Debug, Clone, Copy)]
struct MeshStats {
    max_edit_upload_frames: u32,
    uploads: u64,
    discards: u64,
    unloads: u64,
    evictions: u64,
    baseline: Option<usize>,
}

impl MeshStats {
    fn to_manifest(self, assets_live_end: usize, entities_live_end: usize) -> ManifestMeshStats {
        ManifestMeshStats {
            max_edit_upload_frames: self.max_edit_upload_frames,
            uploads: self.uploads,
            discards: self.discards,
            unloads: self.unloads,
            evictions: self.evictions,
            assets_live_end,
            entities_live_end,
            assets_baseline: self.baseline.unwrap_or(0),
        }
    }
}

/// 身体资产目录路径资源(供 `setup_scene` 初始化 [`EntityRenderRes`],REQ-013 C03-1 D3)。
#[derive(Resource)]
struct BodiesDirRes(PathBuf);

/// 一次性采样 Bevy 内置插件自持 Mesh 资产基线(REQ-013 C02-2 R2-3 / C03-1 D5):
/// 挂在真 App 的 `PostUpdate` 且晚于 `AssetEventSystems`(晚于 `SpriteMeshPlugin::add_mesh`),
/// 首帧记录 `meshes.len() - (chunk 段实体 + 实体渲染 mesh 子实体)` 后永久幂等跳过;headless 裸 App 不挂载。
fn sample_mesh_assets_baseline_system(
    mut done: Local<bool>,
    meshes: Res<Assets<Mesh>>,
    world_res: Res<WorldRenderRes>,
    entity_render: Option<Res<EntityRenderRes>>,
    mut stats: ResMut<MeshStats>,
) {
    if *done {
        return;
    }
    *done = true;
    let live_entities =
        world_res.entities.len() + entity_render.as_ref().map_or(0, |r| r.live_mesh_entities());
    stats.baseline = Some(meshes.len().saturating_sub(live_entities));
}

/// 异步网格构建耗时累计遥测(REQ-013 C01b-3 §1.4)。
#[derive(Resource, Default, Debug, Clone, Copy)]
struct MeshPerfRes(MeshPerf);

/// 帧耗时历史与编辑风暴窗口追踪(REQ-013 C01b-3 §1.4;脚本与交互模式均记录)。
#[derive(Resource, Default)]
struct FpsHistoryRes {
    dts_ms: Vec<f64>,
    dropped_prefix: u64,
    first_dig_frame: Option<u64>,
    last_edit_upload_frame: Option<u64>,
}

impl FpsHistoryRes {
    fn push_dt_ms(&mut self, dt_ms: f64) {
        if dt_ms <= 0.0 || !dt_ms.is_finite() {
            return;
        }
        if self.dts_ms.len() >= FPS_HISTORY_CAP {
            self.dts_ms.remove(0);
            self.dropped_prefix = self.dropped_prefix.saturating_add(1);
        }
        self.dts_ms.push(dt_ms);
    }

    fn current_index(&self) -> u64 {
        self.dropped_prefix
            .saturating_add(self.dts_ms.len().saturating_sub(1) as u64)
    }

    fn record_first_dig(&mut self) {
        if self.first_dig_frame.is_none() {
            self.first_dig_frame = Some(self.current_index());
        }
    }

    fn record_edit_upload(&mut self) {
        self.last_edit_upload_frame = Some(self.current_index());
    }

    /// 清空 FPS 历史与风暴窗口标记(REQ-013 C01b-3 R1 §2.1):
    /// `dropped_prefix` 与两 marker 同清保证绝对下标自洽。
    fn reset(&mut self) {
        self.dts_ms.clear();
        self.dropped_prefix = 0;
        self.first_dig_frame = None;
        self.last_edit_upload_frame = None;
    }
}

/// 脚本 `perf_reset` 步执行分支(REQ-013 C01b-3 R1 §2.1):遇瞬时步即重置采样窗口。
fn apply_perf_reset_step(perf_reset: bool, fps_history: &mut FpsHistoryRes) {
    if perf_reset {
        fps_history.reset();
    }
}

/// 最近一次从服务端收到的 `S2C::PerfStats` 采样。
#[derive(Debug, Clone, Copy)]
struct ServerPerfSample {
    sim_p95_ms: f32,
    sim_samples: u64,
    policy_infer_count: u64,
    policy_total_us: u64,
}

/// 服务端性能遥测缓存(REQ-013 C01b-3 §1.4)。
#[derive(Resource, Default)]
struct ServerPerfRes {
    latest: Option<ServerPerfSample>,
}

/// 将客户端 FPS/网格遥测与最新服务端 `PerfStats` 汇总为 [`ManifestPerf`]。
fn build_manifest_perf(
    fps: &FpsHistoryRes,
    mesh: &MeshPerfRes,
    server: &ServerPerfRes,
) -> ManifestPerf {
    let (fps_mean, fps_min) = fps_summary(&fps.dts_ms);
    let first_dig = fps
        .first_dig_frame
        .map(|f| f.saturating_sub(fps.dropped_prefix));
    let last_edit = fps
        .last_edit_upload_frame
        .map(|f| f.saturating_sub(fps.dropped_prefix));
    let (fps_storm_min, storm_frames) =
        match storm_window(first_dig, last_edit, fps.dts_ms.len() as u64) {
            Some((start, end)) => {
                let (_, min_fps) = fps_summary(&fps.dts_ms[start..=end]);
                (Some(min_fps), (end - start + 1) as u64)
            }
            None => (None, 0),
        };
    let (sim_p95_ms, sim_samples, policy_infer_count, policy_ms_per_body) = match server.latest {
        Some(s) => {
            let ms_per_body = if s.policy_infer_count > 0 {
                Some(s.policy_total_us as f64 / 1000.0 / s.policy_infer_count as f64)
            } else {
                None
            };
            (
                Some(s.sim_p95_ms),
                s.sim_samples,
                s.policy_infer_count,
                ms_per_body,
            )
        }
        None => (None, 0, 0, None),
    };
    ManifestPerf {
        fps_mean,
        fps_min,
        fps_storm_min,
        storm_frames,
        mesh_max_ms: mesh.0.max_us as f64 / 1000.0,
        mesh_count: mesh.0.count,
        sim_p95_ms,
        sim_samples,
        policy_infer_count,
        policy_ms_per_body,
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

/// 最新服务器拍号(HUD 显示用,由 `drain_transport_system` 从 S2C 帧更新)。
#[derive(Resource, Default, Debug, Clone, Copy)]
struct ServerTickRes(u64);

/// 交互工具状态资源(REQ-013 C02-1 §2.2;仅交互模式使用)。
#[derive(Resource, Default)]
struct ToolRes {
    state: ToolState,
    last_hit: Option<Hit>,
    notices: Vec<(String, Instant)>,
}

impl ToolRes {
    fn push_notice(&mut self, msg: String, now: Instant) {
        self.notices.push((msg, now));
    }

    fn prune_notices(&mut self, now: Instant) {
        self.notices
            .retain(|(_, created)| now.duration_since(*created) < NOTICE_TTL);
    }
}

/// 交互附身状态资源(REQ-013 C02-1 §2.2;仅交互模式使用)。
#[derive(Resource, Default)]
struct PossessRes {
    state: PossessState,
    possessed: Option<EntityId>,
    /// 主体最新根位置表(供第三人称相机追踪,同 `viewer/src/render.rs:1469-1474`)。
    agents: BTreeMap<EntityId, types::glam::Vec3>,
    /// 附身期间最后一次位于世界高度内的安全相机位(同 `viewer/src/render.rs:1485-1491`)。
    last_safe_camera: Option<types::glam::Vec3>,
    was_possessed: bool,
}

/// HUD 根容器节点标记组件(REQ-013 C02-2 D3:脚本模式按 `shot.hud` 切换显隐)。
#[derive(Component)]
struct HudRoot;

/// 交互/脚本 HUD 文本节点标记组件(REQ-013 C02-1 §3.2)。
#[derive(Component)]
struct HudText;

/// 按布尔开关设置 HUD 根节点的可见性(`true` → `Inherited`,`false` → `Hidden`;供 MS24 直调)。
#[cfg_attr(not(test), allow(dead_code))]
fn set_hud_visibility(world: &mut World, show: bool) {
    let target = if show {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    };
    let mut q = world.query_filtered::<&mut Visibility, With<HudRoot>>();
    for mut vis in q.iter_mut(world) {
        *vis = target;
    }
}

/// 将 Bevy 键盘快照映射为 `possess::Keys`(REQ-013 C02-1 §2.2 / MS20 锚点)。
fn possess_keys_from_input(keys: &ButtonInput<KeyCode>) -> possess::Keys {
    possess::Keys {
        w: keys.pressed(KeyCode::KeyW),
        a: keys.pressed(KeyCode::KeyA),
        s: keys.pressed(KeyCode::KeyS),
        d: keys.pressed(KeyCode::KeyD),
        shift: keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight),
        jump_down: keys.pressed(KeyCode::Space),
    }
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
    /// REQ-013 C02-2 R3:延迟 1 帧 spawn `Screenshot` 实体的缓存 `(请求, shot 帧号)`。
    pending_spawn: Option<(ScreenshotRequest, u64)>,
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
                pending_spawn: None,
                finished_exit: None,
                disconnected: None,
            };
            (cfg, vd, Some(res))
        }
    };

    let bodies_dir = config.bodies_dir.clone();
    let local_cfg = LocalConfig {
        config,
        view_distance,
    };

    let local_transport = match LocalTransport::new(local_cfg) {
        Ok(t) => t,
        Err(err) => {
            eprintln!(
                "启动单机宿主失败(world_dir={}): {err}",
                cli.world_dir.display()
            );
            std::process::exit(1);
        }
    };
    let mut transport = SequencedTransport::new(local_transport);
    if script_res.is_some() {
        // 脚本模式启动即订阅服务端性能/调试遥测(REQ-013 C01b-3 §1.4;交互模式行为不变)。
        transport.send(C2S::SubscribeDebug);
    }

    let screenshot_dir = cli.world_dir.join("screenshots");
    let chunk_cap = cli
        .chunk_cap
        .unwrap_or_else(|| ViewCapRes::default_cap(view_distance));

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
    .insert_resource(TransportRes::new(transport))
    .insert_resource(ViewCapRes {
        view_distance,
        chunk_cap,
    })
    .insert_resource(FrameCounter::default())
    .insert_resource(PendingMeshesRes::default())
    .insert_resource(EditFrameTracker::default())
    .insert_resource(MeshStats::default())
    .insert_resource(MeshPerfRes::default())
    .insert_resource(FpsHistoryRes::default())
    .insert_resource(ServerPerfRes::default())
    .insert_resource(ServerTickRes::default())
    .insert_resource(ToolRes::default())
    .insert_resource(PossessRes::default())
    .insert_resource(EntityMirrorRes::default())
    .insert_resource(BodiesDirRes(bodies_dir))
    .insert_resource(ObserverTracker::default())
    .insert_resource(ScreenshotState {
        dir: screenshot_dir,
        next_seq: 1,
    })
    .add_systems(Startup, setup_scene)
    .add_systems(
        PostUpdate,
        sample_mesh_assets_baseline_system.after(AssetEventSystems),
    );

    if let Some(script_mode) = script_res {
        app.insert_resource(script_mode).add_systems(
            Update,
            (
                advance_frame_counter_system,
                drain_transport_system,
                sync_entities_system,
                collect_mesh_tasks_system,
                script_driver_system,
                spawn_mesh_tasks_system,
                hud_update_system,
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
                interact_system,
                drain_transport_system,
                sync_entities_system,
                collect_mesh_tasks_system,
                spawn_mesh_tasks_system,
                hud_update_system,
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
    bodies_dir: Option<Res<BodiesDirRes>>,
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
    let dir = bodies_dir
        .as_deref()
        .map(|b| b.0.as_path())
        .unwrap_or_else(|| Path::new("assets/bodies"));
    commands.insert_resource(EntityRenderRes::new(dir, &mut materials));

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

    // REQ-013 C02-1 §3.2 / C02-2 D3:交互与脚本模式均 spawn HUD 根 Node + 子 Text;
    // 脚本模式初始 Hidden,由 script_driver_system 按 shot.hud 切换。
    let hud_visibility = if script_mode.is_some() {
        Visibility::Hidden
    } else {
        Visibility::Inherited
    };
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(12.0),
                top: Val::Px(12.0),
                ..default()
            },
            hud_visibility,
            HudRoot,
        ))
        .with_children(|parent| {
            parent.spawn((
                Node::default(),
                Text::new(""),
                TextFont::from_font_size(16.0),
                TextColor(Color::WHITE),
                HudText,
            ));
        });
}

/// 每帧推进渲染帧计数器并记录帧耗时(脚本与交互模式均记录,上限 7200 帧)。
fn advance_frame_counter_system(
    time: Res<Time>,
    mut frame_counter: ResMut<FrameCounter>,
    mut fps_history: ResMut<FpsHistoryRes>,
) {
    frame_counter.0 = frame_counter.0.saturating_add(1);
    fps_history.push_dt_ms(time.delta_secs_f64() * 1000.0);
}

/// 飞行相机控制(交互模式):鼠标转视角、WASD 水平移动、Shift/Ctrl 升降、M 切换鼠标锁定、Esc 退出。
#[allow(clippy::too_many_arguments)]
fn fly_camera_system(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    possess_res: Res<PossessRes>,
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

    let routing = possess::input_routing(possess_res.possessed.is_some());
    if routing.wasd_moves_camera {
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
    let Ok(transform) = cam_q.single() else {
        return;
    };
    let world_pos = conv::from_bevy_vec3(transform.translation);
    let chunk = ChunkPos::new(
        (world_pos.x / 16.0).floor() as i32,
        (world_pos.z / 16.0).floor() as i32,
    );
    if tracker.last_chunk != Some(chunk) {
        transport_res.with_mut(|transport| {
            tracker.last_chunk = Some(chunk);
            transport.send(C2S::MoveObserver {
                position: world_pos,
            });
        });
    }
}

/// 交互模式体素编辑与附身系统(REQ-013 C02-1 §2.2):
/// 每帧拾取、1–4 选材、LMB 挖掘 / RMB 放置、P 键生成/附身/解除、附身中 WASD/Shift/Space 意图与
/// 第三人称相机跟随(语义逐位照搬 `viewer/src/render.rs:1226-1298, 1467-1503`)。
#[allow(clippy::too_many_arguments)]
fn interact_system(
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
    world_res: Res<WorldRenderRes>,
    mut transport_res: ResMut<TransportRes>,
    mut tool_res: ResMut<ToolRes>,
    mut possess_res: ResMut<PossessRes>,
    mut cam_q: Query<(&mut Transform, &FlyCamera)>,
) {
    let Ok((mut transform, fly)) = cam_q.single_mut() else {
        return;
    };

    #[allow(clippy::disallowed_methods)]
    // 墙钟(工具提示 3s 过期与附身节流/超时;渲染侧授权,同 viewer render.rs:1244)
    let now = Instant::now();
    tool_res.prune_notices(now);

    let cam_pos = conv::from_bevy_vec3(transform.translation);
    let yaw_deg = fly.yaw.to_degrees();
    let pitch_deg = fly.pitch.to_degrees();
    tool_res.last_hit = pick_at_camera(&world_res.mirror, cam_pos, yaw_deg, pitch_deg);

    if keys.just_pressed(KeyCode::Digit1) {
        select_material(&mut tool_res.state, 0);
    }
    if keys.just_pressed(KeyCode::Digit2) {
        select_material(&mut tool_res.state, 1);
    }
    if keys.just_pressed(KeyCode::Digit3) {
        select_material(&mut tool_res.state, 2);
    }
    if keys.just_pressed(KeyCode::Digit4) {
        select_material(&mut tool_res.state, 3);
    }

    transport_res.with_mut(|transport| {
        // 鼠标锁定时左右键挖掘/放置(同 viewer render.rs:1286)。
        if fly.cursor_grabbed {
            if mouse_buttons.just_pressed(MouseButton::Left) {
                let action = dig(tool_res.last_hit.as_ref());
                for reason in
                    forward_engine_commands(&[EngineCommand::ToolAction(action)], transport)
                {
                    tool_res.push_notice(reason, now);
                }
            }
            if mouse_buttons.just_pressed(MouseButton::Right) {
                let action = place(
                    &tool_res.state,
                    tool_res.last_hit.as_ref(),
                    yaw_deg,
                    &world_res.mirror,
                );
                for reason in
                    forward_engine_commands(&[EngineCommand::ToolAction(action)], transport)
                {
                    tool_res.push_notice(reason, now);
                }
            }
            if possess_res.possessed.is_some() && mouse_scroll.delta.y != 0.0 {
                possess_res.state.adjust_camera_dist(mouse_scroll.delta.y);
            }
        }

        // P 键按下沿(逐位照搬 viewer/src/render.rs:1231-1260)。
        if keys.just_pressed(KeyCode::KeyP) {
            let spawn_target = tool_res.last_hit.as_ref().map(|hit| {
                let top = hit.block_pos;
                let px = top.x as f32 + 0.5;
                let py = top.y as f32 + 1.0 + 0.9;
                let pz = top.z as f32 + 0.5;
                (types::glam::Vec3::new(px, py, pz), yaw_deg)
            });
            let commands = possess_res.state.on_p_pressed(spawn_target, now);
            if commands.is_empty()
                && let Some(hint) =
                    p_press_hint(possess_res.state.phase(), tool_res.last_hit.is_some())
            {
                tool_res.push_notice(hint.to_owned(), now);
            }
            let intent_target = possess_res.possessed;
            for cmd in commands {
                if let Some(c2s) = possess::to_c2s(&cmd, intent_target) {
                    transport.send(c2s);
                }
            }
        }

        // 附身中 WASD/Shift/Space 意图(逐位照搬 viewer/src/render.rs:1262-1277)。
        if possess_res.possessed.is_some() {
            let pkeys = possess_keys_from_input(&keys);
            let intent_target = possess_res.possessed;
            for cmd in possess_res.state.on_keys(&pkeys, yaw_deg, now) {
                if let Some(c2s) = possess::to_c2s(&cmd, intent_target) {
                    transport.send(c2s);
                }
            }
        }
    });

    possess_res.state.tick(now);

    // 第三人称相机跟随与身体消失放回(逐位照搬 viewer/src/render.rs:1467-1503)。
    let possessed_id = possess_res.possessed;
    if let Some(id) = possessed_id
        && let Some(&root) = possess_res.agents.get(&id)
    {
        let pos = possess::possessed_camera_position(
            root,
            yaw_deg,
            pitch_deg,
            possess_res.state.camera_dist(),
        );
        transform.translation = conv::to_bevy_vec3(pos);
        let look_target = transform.translation + fly.forward();
        transform.look_at(look_target, Vec3::Y);
        if pos.y >= WORLD_Y_MIN as f32 {
            possess_res.last_safe_camera = Some(pos);
        }
    }
    if possess_res.was_possessed && possessed_id.is_none() {
        let cur = conv::from_bevy_vec3(transform.translation);
        let fixed =
            possess::camera_after_body_lost(cur, possess_res.last_safe_camera, WORLD_Y_MIN as f32);
        transform.translation = conv::to_bevy_vec3(fixed);
        let look_target = transform.translation + fly.forward();
        transform.look_at(look_target, Vec3::Y);
    }
    possess_res.was_possessed = possessed_id.is_some();
}

/// 交互模式 HUD 文本更新系统(REQ-013 C02-1 §3.2 / MS18 & PM26 靶点)。
fn hud_update_system(
    fps_history: Res<FpsHistoryRes>,
    server_tick: Res<ServerTickRes>,
    world_res: Res<WorldRenderRes>,
    tool_res: Res<ToolRes>,
    possess_res: Res<PossessRes>,
    mut hud_q: Query<&mut Text, With<HudText>>,
) {
    let Ok(mut text) = hud_q.single_mut() else {
        return;
    };
    let (mean_f64, min_f64) = fps_summary(&fps_history.dts_ms);
    let notices: Vec<String> = tool_res.notices.iter().map(|(m, _)| m.clone()).collect();
    text.0 = hud_text(
        &tool_res.state,
        tool_res.last_hit.as_ref(),
        possess_res.state.phase(),
        (mean_f64 as f32, min_f64 as f32),
        server_tick.0,
        world_res.mirror.chunks_loaded(),
        &notices,
    );
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

/// 清理单个 chunk 关联的全部渲染实体、网格句柄、在途任务与编辑帧追踪(REQ-013 C02-2 D1b / PM29 靶点)。
///
/// 调用方负责先在镜像侧完成 `remove_chunk`(显式 `S2C::ChunkUnload` 先移除再调本函数;
/// LRU `enforce_cap` 内部已完成 `remove_chunk`)。
fn unload_chunk_visuals(
    pos: ChunkPos,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    world_res: &mut WorldRenderRes,
    pending: &mut PendingMeshesRes,
    edit_frames: &mut EditFrameTracker,
) {
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

/// 消费宿主 S2C 帧更新世界镜像(脚本模式同时喂 `WorldState`,交互与脚本模式维护 `PossessRes`),
/// 并在 chunk 卸载或 LRU 驱逐时经 [`unload_chunk_visuals`] 回收实体/资产/任务。
#[allow(clippy::too_many_arguments)]
fn drain_transport_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    frame_counter: Res<FrameCounter>,
    view_cap: Res<ViewCapRes>,
    mut transport_res: ResMut<TransportRes>,
    mut world_res: ResMut<WorldRenderRes>,
    mut pending: ResMut<PendingMeshesRes>,
    mut edit_frames: ResMut<EditFrameTracker>,
    mut stats: ResMut<MeshStats>,
    mut server_perf: Option<ResMut<ServerPerfRes>>,
    mut server_tick: ResMut<ServerTickRes>,
    mut possess_res: Option<ResMut<PossessRes>>,
    mut entity_mirror: Option<ResMut<EntityMirrorRes>>,
    mut script_mode: Option<ResMut<ScriptModeRes>>,
    cam_q: Query<&Transform, With<FlyCamera>>,
) {
    let Some(msgs) = transport_res.with_mut(|transport| transport.drain()) else {
        return;
    };
    let mut pending_sends: Vec<C2S> = Vec::new();
    for msg in msgs {
        if let Some(em) = entity_mirror.as_mut() {
            em.0.apply(&msg);
        }
        if let Some(script) = script_mode.as_mut()
            && let Some(ev) = s2c_to_event(msg.clone())
        {
            match ev {
                ViewEvent::Disconnected { reason } => script.disconnected = Some(reason),
                other => script.world.apply(other),
            }
        }

        match msg {
            S2C::Welcome { tick, .. } | S2C::EntityStates { tick, .. } => {
                server_tick.0 = tick.0;
            }
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
                unload_chunk_visuals(
                    pos,
                    &mut commands,
                    &mut meshes,
                    &mut world_res,
                    &mut pending,
                    &mut edit_frames,
                );
                stats.unloads = stats.unloads.saturating_add(1);
            }
            S2C::Spawned { entity } => {
                if let Some(possess) = possess_res.as_mut() {
                    possess.state.on_spawned(entity);
                }
            }
            S2C::Possessed { entity } => {
                if let Some(possess) = possess_res.as_mut() {
                    possess.state.on_possessed(entity);
                    possess.possessed = entity;
                }
            }
            S2C::AgentStates {
                tick,
                states,
                removed,
                full,
            } => {
                server_tick.0 = tick.0;
                if let Some(possess) = possess_res.as_mut() {
                    if full {
                        possess.agents.clear();
                    }
                    for id in &removed {
                        possess.agents.remove(id);
                    }
                    for (id, _, poses) in &states {
                        if let Some(root) = poses.first() {
                            possess.agents.insert(*id, root.translation);
                        }
                    }
                    let ids: Vec<EntityId> = states.iter().map(|(id, ..)| *id).collect();
                    let intent_target = possess.possessed;
                    for cmd in possess.state.on_agents(&ids, &removed) {
                        if let Some(c2s) = possess::to_c2s(&cmd, intent_target) {
                            pending_sends.push(c2s);
                        }
                    }
                    // 若主体被移除导致状态机回退 NoBody,同步清理 possessed 缓存。
                    if possess.state.possessed().is_none() && possess.possessed.is_some() {
                        possess.possessed = None;
                    }
                }
            }
            S2C::PerfStats {
                sim_p95_ms,
                sim_samples,
                policy_infer_count,
                policy_total_us,
                ..
            } => {
                if let Some(perf) = server_perf.as_mut() {
                    perf.latest = Some(ServerPerfSample {
                        sim_p95_ms,
                        sim_samples,
                        policy_infer_count,
                        policy_total_us,
                    });
                }
            }
            _ => {}
        }
    }
    if !pending_sends.is_empty() {
        transport_res.with_mut(|transport| {
            for c2s in pending_sends {
                transport.send(c2s);
            }
        });
    }

    // REQ-013 C02-2 D1b:按相机所在 chunk ± vd 构造保护集,超上限时按 LRU 驱逐并回收视觉句柄。
    let mut protect = BTreeSet::new();
    if let Ok(cam_tf) = cam_q.single() {
        let cam_pos = conv::from_bevy_vec3(cam_tf.translation);
        let cx = (cam_pos.x / 16.0).floor() as i32;
        let cz = (cam_pos.z / 16.0).floor() as i32;
        let vd = i32::from(view_cap.view_distance);
        for dx in -vd..=vd {
            for dz in -vd..=vd {
                protect.insert(ChunkPos::new(cx + dx, cz + dz));
            }
        }
    }
    let evicted = world_res.mirror.enforce_cap(view_cap.chunk_cap, &protect);
    if !evicted.is_empty() {
        stats.evictions = stats.evictions.saturating_add(evicted.len() as u64);
        for pos in evicted {
            unload_chunk_visuals(
                pos,
                &mut commands,
                &mut meshes,
                &mut world_res,
                &mut pending,
                &mut edit_frames,
            );
        }
    }
}

/// 轮询在途异步网格任务:世代匹配则上传/替换/回收空段并更新 `MeshStats`/`MeshPerfRes`;过期则丢弃并回队。
#[allow(clippy::too_many_arguments)]
fn collect_mesh_tasks_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    frame_counter: Res<FrameCounter>,
    mut world_res: ResMut<WorldRenderRes>,
    mut pending: ResMut<PendingMeshesRes>,
    mut edit_frames: ResMut<EditFrameTracker>,
    mut stats: ResMut<MeshStats>,
    mut mesh_perf: Option<ResMut<MeshPerfRes>>,
    mut fps_history: Option<ResMut<FpsHistoryRes>>,
) {
    let material = world_res.terrain_material.clone();
    let mut completed: Vec<(SubchunkPos, DirtyKind, u64, SubchunkMesh, u64)> = Vec::new();

    pending.tasks.retain(|&sc, entry| {
        if let Some((submesh, us)) = check_ready(&mut entry.task) {
            completed.push((sc, entry.kind, entry.epoch, submesh, us));
            false
        } else {
            true
        }
    });

    for (sc, kind, pending_epoch, submesh, us) in completed {
        if let Some(mp) = mesh_perf.as_mut() {
            acc_mesh_perf(&mut mp.0, us);
        }
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
        if kind == DirtyKind::Edit {
            if let Some(fps) = fps_history.as_mut() {
                fps.record_edit_upload();
            }
            if let Some(arrived_frame) = edit_frames.frames.remove(&sc) {
                let diff = frame_counter.0.saturating_sub(arrived_frame) as u32;
                stats.max_edit_upload_frames = stats.max_edit_upload_frames.max(diff);
            }
        }
    }
}

/// 将脚本引擎或交互工具输出的 `EngineCommand` 列表翻译为 `C2S` 并经发送闭包转发
/// (与 `viewer/src/render.rs:739-796` `forward_commands` 同口径;返回被拒绝的 `ToolAction` 提示)。
fn forward_engine_commands_with(
    commands: &[EngineCommand],
    mut send: impl FnMut(C2S),
) -> Vec<String> {
    for c2s in engine_commands_to_c2s(commands) {
        send(c2s);
    }
    commands
        .iter()
        .filter_map(|command| match command {
            EngineCommand::ToolAction(ToolAction::Rejected(reason)) => Some(reason.clone()),
            _ => None,
        })
        .collect()
}

/// 将 `EngineCommand` 列表翻译为 `C2S` 并经 `Transport` 发送(返回被拒绝的 `ToolAction` 提示)。
fn forward_engine_commands(
    commands: &[EngineCommand],
    transport: &mut dyn Transport,
) -> Vec<String> {
    forward_engine_commands_with(commands, |c2s| transport.send(c2s))
}

/// 脚本冒烟模式主驱动系统:等 `tick_known` → 推进 `Engine` → 相机/命令/截图/HUD 显隐/manifest → 优雅收口退出。
#[allow(clippy::too_many_arguments)]
fn script_driver_system(
    mut commands: Commands,
    time: Res<Time>,
    frame_counter: Res<FrameCounter>,
    meshes: Res<Assets<Mesh>>,
    world_res: Res<WorldRenderRes>,
    entity_render: Option<Res<EntityRenderRes>>,
    stats: Res<MeshStats>,
    mesh_perf: Res<MeshPerfRes>,
    mut fps_history: ResMut<FpsHistoryRes>,
    server_perf: Res<ServerPerfRes>,
    mut transport_res: ResMut<TransportRes>,
    mut script: ResMut<ScriptModeRes>,
    mut cam_q: Query<(&mut Transform, &mut FlyCamera)>,
    mut hud_root_q: Query<&mut Visibility, With<HudRoot>>,
) {
    // REQ-013 C02-2 R3:延迟 1 帧 spawn Screenshot 实体(确保 HUD Visibility 切换已完成渲染传播)。
    // 必须放在 finished_exit 与 pending_shots 门控之前:末张截图的延迟 spawn 发生在 finished_exit 已置位之后。
    if let Some((_, shot_frame)) = script.pending_spawn.as_ref()
        && frame_counter.0 > *shot_frame
    {
        let (shot, _) = script
            .pending_spawn
            .take()
            .expect("pending_spawn 刚完成帧号检查");
        let path = script.out_dir.join(shot.file_name());
        let done = Arc::clone(&script.pending_shots);
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path))
            .observe(move |_: On<ScreenshotCaptured>| {
                done.fetch_sub(1, Ordering::SeqCst);
            });
    }

    let sample_mesh_stats = || {
        let live_entities =
            world_res.entities.len() + entity_render.as_ref().map_or(0, |r| r.live_mesh_entities());
        stats.to_manifest(meshes.len(), live_entities)
    };

    // 场景已结束:等待最后一张异步截图写盘完毕,再写终版 manifest、关闭宿主并按 exit_code 退出。
    if let Some(code) = script.finished_exit {
        if script.pending_shots.load(Ordering::SeqCst) > 0 {
            return;
        }
        script.manifest.mesh_stats = Some(sample_mesh_stats());
        script.manifest.perf = Some(build_manifest_perf(&fps_history, &mesh_perf, &server_perf));
        write_manifest(&script.manifest, &script.out_dir);
        // 显式 drop Transport → 发送 Goodbye 并 join 宿主线程,确保 redb 收口。
        transport_res.close();
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
        if script.last_observer_pos.is_none() {
            let pos = transform.translation;
            transport_res.with_mut(|transport| {
                script.last_observer_pos = Some(pos);
                transport.send(C2S::MoveObserver {
                    position: conv::from_bevy_vec3(pos),
                });
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
        script.manifest.mesh_stats = Some(sample_mesh_stats());
        script.manifest.perf = Some(build_manifest_perf(&fps_history, &mesh_perf, &server_perf));
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

    // 应用引擎输出的相机位姿;possess follow=true 时切第三人称跟随视角(同 viewer render.rs:918-927)。
    let mut cam_pos = conv::to_bevy_vec3(output.camera.pos);
    fly.yaw = output.camera.yaw.to_radians();
    fly.pitch = output
        .camera
        .pitch
        .to_radians()
        .clamp(-89.0_f32.to_radians(), 89.0_f32.to_radians());
    transform.translation = cam_pos;
    let look_target = cam_pos + fly.forward();
    transform.look_at(look_target, Vec3::Y);

    if let Some((id, follow)) = output.possess
        && follow
        && let Some((_, poses)) = script.world.agent(id)
        && let Some(pose) = poses.first()
    {
        let root = conv::to_bevy_vec3(pose.translation);
        let look = Vec3::new(root.x, root.y + 0.6, root.z);
        cam_pos = Vec3::new(look.x, look.y + 4.0, look.z + 4.0);
        fly.yaw = -90.0_f32.to_radians();
        fly.pitch = -45.0_f32.to_radians();
        transform.translation = cam_pos;
        transform.look_at(look, Vec3::Y);
    }

    script.coupled = output.coupled;
    apply_perf_reset_step(output.perf_reset, &mut fps_history);
    if output.commands.iter().any(|cmd| {
        matches!(
            cmd,
            EngineCommand::SetBlock { .. }
                | EngineCommand::ToolAction(
                    ToolAction::Single(SimCommand::SetBlock { .. }) | ToolAction::Batch(_)
                )
        )
    }) {
        fps_history.record_first_dig();
    }
    let mut rejected_messages: Vec<String> = Vec::new();
    transport_res.with_mut(|transport| {
        rejected_messages = forward_engine_commands(&output.commands, transport);
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
    });

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
        if !rejected_messages.is_empty() {
            if let Some(step) = manifest
                .steps
                .iter_mut()
                .rev()
                .find(|s| s.action == "dig" || s.action == "place")
            {
                step.messages.extend(rejected_messages);
            } else if let Some(step) = manifest.steps.last_mut() {
                step.messages.extend(rejected_messages);
            }
        }
        manifest.mesh_stats = Some(sample_mesh_stats());
        manifest.perf = Some(build_manifest_perf(&fps_history, &mesh_perf, &server_perf));
        write_manifest(manifest, out_dir);
    }

    if let Some(shot) = output.screenshot {
        // REQ-013 C02-2 D3 + R3 / PM30 靶点:shot 帧只切换 HUD 根可见性、计入 pending_shots
        // 并缓存 pending_spawn,不于同帧 spawn Screenshot(下一帧由顶部延迟 spawn 段发出)。
        let hud_vis = if shot.hud {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        for mut vis in &mut hud_root_q {
            *vis = hud_vis;
        }
        script.pending_shots.fetch_add(1, Ordering::SeqCst);
        script.pending_spawn = Some((shot, frame_counter.0));
    }

    if !matches!(final_status, EngineStatus::Running { .. }) {
        script.manifest.finish(&final_status);
        script.manifest.mesh_stats = Some(sample_mesh_stats());
        script.manifest.perf = Some(build_manifest_perf(&fps_history, &mesh_perf, &server_perf));
        write_manifest(&script.manifest, &script.out_dir);
        script.finished_exit = Some(exit_code.unwrap_or(2));
    }
}

/// 测量单段异步网格构建耗时(微秒,REQ-013 C01b-3 §1.4):墙钟仅用于异步网格遥测面,
/// 与 `app::telemetry` 同口径,不参与确定性模拟决策。
#[allow(clippy::disallowed_methods)]
fn timed_mesh_subchunk(snap: &ChunkSnapshot, sc: SubchunkPos) -> (SubchunkMesh, u64) {
    let t0 = std::time::Instant::now();
    let mesh = mesh_subchunk_snapshot(snap, sc);
    let us = u64::try_from(t0.elapsed().as_micros()).unwrap_or(u64::MAX);
    (mesh, us)
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
        let task = pool.spawn(async move { timed_mesh_subchunk(&snap, sc) });
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

#[cfg(test)]
mod tests {
    use types::BlockId;
    use voxel::Chunk;

    use super::*;

    const STONE: BlockId = BlockId(1);
    const DIRT: BlockId = BlockId(2);

    /// 构造无头 Bevy App(`MinimalPlugins + AssetPlugin`,不装 `RenderPlugin`,无窗口)。
    fn headless_app(mirror: ChunkMirror, add_collect: bool, add_spawn: bool) -> App {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()));
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();

        let terrain_material = app
            .world_mut()
            .resource_mut::<Assets<StandardMaterial>>()
            .add(StandardMaterial::default());

        app.insert_resource(FrameCounter(0))
            .insert_resource(EditFrameTracker::default())
            .insert_resource(MeshStats::default())
            .insert_resource(MeshPerfRes::default())
            .insert_resource(FpsHistoryRes::default())
            .insert_resource(PendingMeshesRes::default())
            .insert_resource(WorldRenderRes {
                mirror,
                entities: BTreeMap::new(),
                terrain_material,
            });

        if add_collect {
            app.add_systems(Update, collect_mesh_tasks_system);
        }
        if add_spawn {
            app.add_systems(Update, spawn_mesh_tasks_system);
        }
        app
    }

    /// 经 `AsyncComputeTaskPool` 构造已就绪的轻量网格任务,保证单次 `app.update()` 确定性命中。
    fn ready_task(submesh: SubchunkMesh) -> Task<(SubchunkMesh, u64)> {
        let task = AsyncComputeTaskPool::get().spawn(async move { (submesh, 0) });
        for _ in 0..10_000 {
            if task.is_finished() {
                break;
            }
            std::thread::yield_now();
        }
        assert!(task.is_finished(), "轻量异步网格任务应在有界让出内完成");
        task
    }

    /// 构造 `chunk(0,0)` 已装载、排空初始 24 Load 并对 `(0,-60,0)` 写入 STONE(`epoch(s0)==2`)
    /// 且弹出该 Edit 脏条目的镜像。
    fn mirror_at_epoch_2() -> (ChunkMirror, SubchunkPos) {
        let s0 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let mut mirror = ChunkMirror::new();
        mirror.insert_chunk(ChunkPos::new(0, 0), Chunk::air());
        assert_eq!(mirror.epoch(s0), 1);
        for _ in 0..24 {
            assert!(matches!(mirror.pop_dirty(), Some((_, DirtyKind::Load))));
        }
        assert_eq!(mirror.dirty_len(), 0);

        mirror.apply_updates(&[(BlockPos::new(0, -60, 0), STONE)]);
        assert_eq!(mirror.epoch(s0), 2);
        assert_eq!(mirror.pop_dirty(), Some((s0, DirtyKind::Edit)));
        assert_eq!(mirror.dirty_len(), 0);
        (mirror, s0)
    }

    /// MS5-1:过期任务(epoch=1 < mirror epoch=2)在 `collect_mesh_tasks_system` 中被丢弃并回队。
    #[test]
    fn ms5_stale_discard_requeue() {
        let (mirror, s0) = mirror_at_epoch_2();
        let mut app = headless_app(mirror, true, false);

        app.world_mut()
            .resource_mut::<PendingMeshesRes>()
            .tasks
            .insert(
                s0,
                PendingMesh {
                    kind: DirtyKind::Edit,
                    epoch: 1,
                    task: ready_task(SubchunkMesh::default()),
                },
            );

        app.update();

        let stats = *app.world().resource::<MeshStats>();
        assert_eq!(stats.discards, 1);
        assert_eq!(stats.uploads, 0);
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 0);

        let mut world_res = app.world_mut().resource_mut::<WorldRenderRes>();
        assert!(world_res.entities.is_empty());
        assert_eq!(world_res.mirror.pop_dirty(), Some((s0, DirtyKind::Edit)));
    }

    /// MS5-2:世代匹配任务(epoch=2 == mirror epoch=2)在 `collect_mesh_tasks_system` 中正常上传。
    #[test]
    fn ms5_fresh_upload() {
        let (mirror, s0) = mirror_at_epoch_2();
        let snap = snapshot(&mirror, s0).expect("chunk(0,0) 已装载");
        let submesh = mesh_subchunk_snapshot(&snap, s0);
        assert!(!submesh.indices.is_empty(), "STONE 单块网格非空");

        let mut app = headless_app(mirror, true, false);
        app.world_mut()
            .resource_mut::<PendingMeshesRes>()
            .tasks
            .insert(
                s0,
                PendingMesh {
                    kind: DirtyKind::Edit,
                    epoch: 2,
                    task: ready_task(submesh),
                },
            );

        app.update();

        let stats = *app.world().resource::<MeshStats>();
        assert_eq!(stats.uploads, 1);
        assert_eq!(stats.discards, 0);
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 1);

        let mut world_res = app.world_mut().resource_mut::<WorldRenderRes>();
        assert!(world_res.entities.contains_key(&s0));
        assert_eq!(world_res.mirror.pop_dirty(), None);
    }

    /// MS5-3:同段二次编辑替换网格后,旧 Mesh 资产句柄立即被回收(`Assets<Mesh>.len()==1`)。
    #[test]
    fn ms5_replace_no_leak() {
        let (mirror, s0) = mirror_at_epoch_2();
        let snap1 = snapshot(&mirror, s0).expect("chunk(0,0) 已装载");
        let mesh1 = mesh_subchunk_snapshot(&snap1, s0);
        assert!(!mesh1.indices.is_empty());

        let mut app = headless_app(mirror, true, false);
        app.world_mut()
            .resource_mut::<PendingMeshesRes>()
            .tasks
            .insert(
                s0,
                PendingMesh {
                    kind: DirtyKind::Edit,
                    epoch: 2,
                    task: ready_task(mesh1),
                },
            );
        app.update();

        assert_eq!(app.world().resource::<MeshStats>().uploads, 1);
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 1);

        // 再编辑为 DIRT → epoch=3,准备新网格替换旧 Mesh3d。
        let mesh2 = {
            let mut world_res = app.world_mut().resource_mut::<WorldRenderRes>();
            world_res
                .mirror
                .apply_updates(&[(BlockPos::new(0, -60, 0), DIRT)]);
            assert_eq!(world_res.mirror.epoch(s0), 3);
            assert_eq!(world_res.mirror.pop_dirty(), Some((s0, DirtyKind::Edit)));
            let snap2 = snapshot(&world_res.mirror, s0).expect("chunk(0,0) 已装载");
            let m = mesh_subchunk_snapshot(&snap2, s0);
            assert!(!m.indices.is_empty());
            m
        };

        app.world_mut()
            .resource_mut::<PendingMeshesRes>()
            .tasks
            .insert(
                s0,
                PendingMesh {
                    kind: DirtyKind::Edit,
                    epoch: 3,
                    task: ready_task(mesh2),
                },
            );
        app.update();

        assert_eq!(app.world().resource::<MeshStats>().uploads, 2);
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 1);
        assert_eq!(app.world().resource::<WorldRenderRes>().entities.len(), 1);
    }

    /// MS5-4:同段已有在途任务时,`spawn_mesh_tasks_system` 跳过该段不顶替、继续 spawn 其余 8 段,
    /// 并将跳过的 `s0` 回队(`dirty_len==16`,首条 pop 为 `(s0, Load)`)。
    #[test]
    fn ms5_pending_skip_requeue() {
        let s0 = SubchunkPos {
            chunk: ChunkPos::new(0, 0),
            section: 0,
        };
        let mut mirror = ChunkMirror::new();
        mirror.insert_chunk(ChunkPos::new(0, 0), Chunk::air());
        assert_eq!(mirror.dirty_len(), 24);
        assert_eq!(mirror.epoch(s0), 1);

        let mut app = headless_app(mirror, false, true);
        app.world_mut()
            .resource_mut::<PendingMeshesRes>()
            .tasks
            .insert(
                s0,
                PendingMesh {
                    kind: DirtyKind::Load,
                    epoch: 1,
                    task: ready_task(SubchunkMesh::default()),
                },
            );

        app.update();

        {
            let pending = app.world().resource::<PendingMeshesRes>();
            let entry = pending.tasks.get(&s0).expect("s0 仍应在途");
            assert_eq!(entry.epoch, 1, "s0 在途任务不得被顶替");
            assert_eq!(pending.tasks.len(), 9, "原 s0 + 新 spawn 8 段 = 9");
        }

        let mut world_res = app.world_mut().resource_mut::<WorldRenderRes>();
        assert_eq!(world_res.mirror.dirty_len(), 16, "24 - 9 + 1 = 16");
        assert_eq!(world_res.mirror.pop_dirty(), Some((s0, DirtyKind::Load)));
    }

    /// MS12(REQ-013 C01b-3 R1 §2.1):无头验证 `FpsHistoryRes::reset()` 与 `perf_reset`
    /// 脚本步执行分支(PM18/PM19/PM20 靶点,期望值照裁定手算)。
    #[test]
    fn ms12_perf_reset_clears_history_and_markers() {
        let mut fps = FpsHistoryRes::default();
        // 1. push [16,16,16,16,16] → marker 均 Some(4), storm_window(4,4,5) == Some((4,4))。
        for dt in [16.0, 16.0, 16.0, 16.0, 16.0] {
            fps.push_dt_ms(dt);
        }
        fps.record_first_dig();
        fps.record_edit_upload();
        assert_eq!(fps.first_dig_frame, Some(4));
        assert_eq!(fps.last_edit_upload_frame, Some(4));
        assert_eq!(
            storm_window(
                fps.first_dig_frame,
                fps.last_edit_upload_frame,
                fps.dts_ms.len() as u64
            ),
            Some((4, 4))
        );

        // 2. 经脚本 Engine 执行 perf_reset = {} 步并调用步执行分支 → 清空历史、dropped_prefix 与两 marker。
        let scenario =
            parse_scenario("name = \"ms12\"\ndescription = \"d\"\n\n[[step]]\nperf_reset = {}\n")
                .expect("perf_reset 场景合法");
        let start_pose = CameraPose {
            pos: types::glam::Vec3::ZERO,
            yaw: 0.0,
            pitch: 0.0,
        };
        let mut engine = Engine::new(scenario, start_pose);
        let world = WorldState::new();
        let out = engine.frame(&FrameInput {
            dt: 0.016,
            server_tick: 1,
            world: &world,
            spawned: &[],
        });
        assert!(
            out.perf_reset,
            "perf_reset 步必须置位 FrameOutput.perf_reset"
        );
        apply_perf_reset_step(out.perf_reset, &mut fps);
        assert!(fps.dts_ms.is_empty());
        assert_eq!(fps.dropped_prefix, 0);
        assert_eq!(fps.first_dig_frame, None);
        assert_eq!(fps.last_edit_upload_frame, None);

        // 3. push [16,32,16] → marker 均 Some(2), storm_window(2,2,3) == Some((2,2)), 切片 [16] → (62.5, 62.5)。
        for dt in [16.0, 32.0, 16.0] {
            fps.push_dt_ms(dt);
        }
        fps.record_first_dig();
        fps.record_edit_upload();
        assert_eq!(fps.first_dig_frame, Some(2));
        assert_eq!(fps.last_edit_upload_frame, Some(2));
        let win = storm_window(
            fps.first_dig_frame,
            fps.last_edit_upload_frame,
            fps.dts_ms.len() as u64,
        );
        assert_eq!(win, Some((2, 2)));
        let (s, e) = win.expect("窗口非空");
        assert_eq!(&fps.dts_ms[s..=e], &[16.0]);
        assert_eq!(fps_summary(&fps.dts_ms[s..=e]), (62.5, 62.5));
    }

    /// MS18(REQ-013 C02-1 §3.2 + R4 / PM26 靶点):无头 Bevy App 调度 `setup_scene` +
    /// `hud_update_system`,验证 `HudText` 实体同时含 `Node`、`TextFont(16px)`、`TextColor(WHITE)`,
    /// 且插入 `dts=[16,16]`、`tick=42`、`mirror` 3 chunk、`ToolState::default()` 跑一帧后
    /// `Text.0 == hud_text(...)` 且恰 5 行。
    #[test]
    fn ms18_hud_update_system_headless() {
        let mut app = headless_app(ChunkMirror::new(), false, false);
        let mut fps = FpsHistoryRes::default();
        fps.push_dt_ms(16.0);
        fps.push_dt_ms(16.0);
        app.insert_resource(fps)
            .insert_resource(ServerTickRes(42))
            .insert_resource(ToolRes::default())
            .insert_resource(PossessRes::default())
            .add_systems(Startup, setup_scene)
            .add_systems(Update, hud_update_system);

        // 首帧执行 Startup::setup_scene 生成交互模式 HUD 根/子实体并初始化 WorldRenderRes。
        app.update();

        {
            let mut world_res = app.world_mut().resource_mut::<WorldRenderRes>();
            world_res
                .mirror
                .insert_chunk(ChunkPos::new(0, 0), Chunk::air());
            world_res
                .mirror
                .insert_chunk(ChunkPos::new(1, 0), Chunk::air());
            world_res
                .mirror
                .insert_chunk(ChunkPos::new(0, 1), Chunk::air());
            assert_eq!(world_res.mirror.chunks_loaded(), 3);
        }

        app.update();

        let hud_entity = {
            let mut q = app.world_mut().query_filtered::<Entity, With<HudText>>();
            q.single(app.world())
                .expect("setup_scene 必须生成唯一 HudText 实体")
        };

        assert!(
            app.world().get::<Node>(hud_entity).is_some(),
            "HudText 实体必须含 Node 组件供 bevy_ui 布局"
        );
        let font = app
            .world()
            .get::<TextFont>(hud_entity)
            .expect("HudText 实体必须含 TextFont 组件");
        assert_eq!(
            font.font_size,
            bevy::text::FontSize::Px(16.0),
            "HudText 字号必须为 16px"
        );
        let color = app
            .world()
            .get::<TextColor>(hud_entity)
            .expect("HudText 实体必须含 TextColor 组件");
        assert_eq!(color.0, Color::WHITE, "HudText 颜色必须为白色");

        let text = app
            .world()
            .get::<Text>(hud_entity)
            .expect("HUD Text 组件存在");
        let expected = hud_text(
            &ToolState::default(),
            None,
            possess::Phase::NoBody,
            (62.5, 62.5),
            42,
            3,
            &[],
        );
        assert_eq!(
            text.0, expected,
            "hud_update_system 必须将 hud_text 写入 Text"
        );
        assert_eq!(text.0.lines().count(), 5, "无 notices 时 HUD 恰 5 行");
    }

    /// MS20(REQ-013 C02-1 §2.2):Bevy `KeyCode::KeyW` → `possess_keys_from_input` →
    /// `intent_from_keys` 产出世界系水平前向速度,期望值取迁移 `possess` 测试已手算值。
    #[test]
    fn ms20_possess_keys_w_matches_forward() {
        let mut input = ButtonInput::<KeyCode>::default();
        input.press(KeyCode::KeyW);
        let pkeys = possess_keys_from_input(&input);
        assert!(pkeys.w && !pkeys.a && !pkeys.s && !pkeys.d && !pkeys.shift && !pkeys.jump_down);

        for yaw_deg in [0.0_f32, 90.0, 180.0, -45.0] {
            let yaw = yaw_deg.to_radians();
            let expect = (
                yaw.cos() * possess::WALK_SPEED,
                yaw.sin() * possess::WALK_SPEED,
            );
            let got = possess::intent_from_keys(&pkeys, yaw_deg);
            assert!(
                (got.0 - expect.0).abs() <= 1e-5 && (got.1 - expect.1).abs() <= 1e-5,
                "yaw {yaw_deg}°:W 单独按下实得 {got:?},期望 {expect:?}"
            );
        }
    }

    /// MS16b(REQ-013 C02-1 R1 F3 / PM22 端到端靶点):无头端到端 `app::local::LocalHost::spawn` +
    /// `LocalHandle` send/drain + `Engine` place→dig 步,有界轮询(上限 ≤4 拍 t4 口径,挂死上限 600)
    /// 验证 `(2,-60,8)` 先置 `BlockId(1)` 再挖回 `AIR` 且 `(2,-61,8)` 保持 `DIRT`。
    #[test]
    fn ms16b_e2e_place_then_dig_via_local_host() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let cfg = Config {
            data_dir: tmp.path().to_path_buf(),
            world_seed: types::WorldSeed(3),
            ..Config::default()
        };
        let (handle, join) = app::local::LocalHost::spawn(LocalConfig {
            config: cfg,
            view_distance: 2,
        })
        .expect("LocalHost::spawn");

        let mut mirror = ChunkMirror::new();
        let mut server_tick: u64 = 0;
        let mut tick_known = false;
        let ground_pos = BlockPos::new(2, -61, 8);
        let placed_pos = BlockPos::new(2, -60, 8);

        handle.send(C2S::MoveObserver {
            position: types::glam::Vec3::new(2.5, -59.0, 8.5),
        });

        // 预热:等待 chunk (0,0) 与初始地表 (2,-61,8)=DIRT 到达镜像,且收到至少一拍心跳。
        for _ in 0..600 {
            for msg in handle.drain() {
                match msg {
                    S2C::ChunkData { pos, chunk } => mirror.insert_chunk(pos, chunk),
                    S2C::BlockUpdates { updates } => mirror.apply_updates(&updates),
                    S2C::EntityStates { tick, .. } => {
                        server_tick = tick.0;
                        tick_known = true;
                    }
                    _ => {}
                }
            }
            if tick_known && mirror.block(ground_pos) == Some(DIRT) {
                break;
            }
            std::thread::sleep(Duration::from_millis(25));
        }
        assert!(tick_known, "600 轮内必须收到宿主拍号心跳");
        assert_eq!(
            mirror.block(ground_pos),
            Some(DIRT),
            "seed 3 超平坦顶格 (2,-61,8) 应为 DIRT"
        );
        assert_eq!(mirror.block(placed_pos), Some(BlockId::AIR));

        let scenario = parse_scenario(
            "name = \"ms16b\"\ndescription = \"e2e place then dig\"\n\n\
             [[step]]\ncamera = { pos = [2.5, -59.0, 8.5], yaw = -90.0, pitch = -89.0 }\n\
             [[step]]\nplace = { material = 1 }\n\
             [[step]]\nwait_ticks = { n = 1 }\n\
             [[step]]\ndig = {}\n",
        )
        .expect("ms16b 场景合法");
        let start_pose = CameraPose {
            pos: types::glam::Vec3::new(2.5, -59.0, 8.5),
            yaw: -90.0,
            pitch: -89.0,
        };
        let mut engine = Engine::new(scenario, start_pose);

        // 1) 执行 place={material=1} 并转发给 LocalHandle。
        let out_place = engine.frame(&FrameInput {
            dt: 0.016,
            server_tick,
            world: &mirror,
            spawned: &[],
        });
        let rej_place = forward_engine_commands_with(&out_place.commands, |c2s| handle.send(c2s));
        assert!(rej_place.is_empty(), "place 不应被拒绝:{rej_place:?}");

        let place_send_tick = server_tick;
        let mut place_waited_steps: Option<u32> = None;
        for step in 1..=600u32 {
            std::thread::sleep(Duration::from_millis(50));
            for msg in handle.drain() {
                match msg {
                    S2C::ChunkData { pos, chunk } => mirror.insert_chunk(pos, chunk),
                    S2C::BlockUpdates { updates } => {
                        mirror.apply_updates(&updates);
                        if updates.contains(&(placed_pos, STONE)) && place_waited_steps.is_none() {
                            place_waited_steps = Some(step);
                        }
                    }
                    S2C::EntityStates { tick, .. } => server_tick = tick.0,
                    _ => {}
                }
            }
            if place_waited_steps.is_some() {
                break;
            }
        }
        let place_waited =
            place_waited_steps.expect("上限 600 拍内必须收到含 (2,-60,8)=STONE 的 BlockUpdates");
        let place_tick_delta = server_tick.saturating_sub(place_send_tick);
        assert!(
            place_waited <= 4,
            "place 回传应在 ≤4 拍内到达(t4 口径),实测 waited={place_waited}, tick_delta={place_tick_delta}"
        );
        assert_eq!(mirror.block(placed_pos), Some(STONE));

        // 2) 推进至 wait_ticks=1 完成并执行 dig={}。
        let dig_engine_tick = server_tick.max(place_send_tick + 1);
        let out_dig = engine.frame(&FrameInput {
            dt: 0.016,
            server_tick: dig_engine_tick,
            world: &mirror,
            spawned: &[],
        });
        let rej_dig = forward_engine_commands_with(&out_dig.commands, |c2s| handle.send(c2s));
        assert!(rej_dig.is_empty(), "dig 不应被拒绝:{rej_dig:?}");

        let dig_send_tick = server_tick;
        let mut dig_waited_steps: Option<u32> = None;
        for step in 1..=600u32 {
            std::thread::sleep(Duration::from_millis(50));
            for msg in handle.drain() {
                match msg {
                    S2C::ChunkData { pos, chunk } => mirror.insert_chunk(pos, chunk),
                    S2C::BlockUpdates { updates } => {
                        mirror.apply_updates(&updates);
                        if updates.contains(&(placed_pos, BlockId::AIR))
                            && dig_waited_steps.is_none()
                        {
                            dig_waited_steps = Some(step);
                        }
                    }
                    S2C::EntityStates { tick, .. } => server_tick = tick.0,
                    _ => {}
                }
            }
            if dig_waited_steps.is_some() {
                break;
            }
        }
        let dig_waited =
            dig_waited_steps.expect("上限 600 拍内必须收到含 (2,-60,8)=AIR 的 BlockUpdates");
        let dig_tick_delta = server_tick.saturating_sub(dig_send_tick);
        assert!(
            dig_waited <= 4,
            "dig 回传应在 ≤4 拍内到达(t4 口径),实测 waited={dig_waited}, tick_delta={dig_tick_delta}"
        );
        assert_eq!(mirror.block(placed_pos), Some(BlockId::AIR));
        assert_eq!(mirror.block(ground_pos), Some(DIRT));
        assert_eq!(engine.status(), &EngineStatus::Done { failed_checks: 0 });
        eprintln!(
            "[MS16b] place_waited={place_waited} (tick_delta={place_tick_delta}), dig_waited={dig_waited} (tick_delta={dig_tick_delta})"
        );

        handle.send(C2S::Goodbye);
        drop(handle);
        join.join().expect("app-local-host 线程正常退出");
    }

    /// 假传输(C02-2 MS26/MS24 无头接线测试用):`send` 丢弃,`drain` 取空共享入队缓冲。
    struct FakeTransport(Arc<Mutex<Vec<S2C>>>);

    impl FakeTransport {
        fn new() -> (Self, Arc<Mutex<Vec<S2C>>>) {
            let queue = Arc::new(Mutex::new(Vec::new()));
            (Self(Arc::clone(&queue)), queue)
        }
    }

    impl Transport for FakeTransport {
        fn send(&mut self, _c2s: C2S) {}
        fn drain(&mut self) -> Vec<S2C> {
            let mut q = self.0.lock().expect("FakeTransport queue poisoned");
            std::mem::take(&mut *q)
        }
    }

    /// MS26(REQ-013 C02-2 D1b / PM29 靶点):假传输无头 App 调度
    /// `drain_transport + collect_mesh + spawn_mesh`,验证 LRU 驱逐与 `ChunkUnload`
    /// 均经 `unload_chunk_visuals` 清空实体并回收 `Assets<Mesh>`(`assets == entities`)。
    #[test]
    fn ms26_lru_evict_and_chunk_unload_visuals_headless() {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()));
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();

        let terrain_material = app
            .world_mut()
            .resource_mut::<Assets<StandardMaterial>>()
            .add(StandardMaterial::default());
        let (fake_transport, inbound) = FakeTransport::new();

        app.insert_resource(WorldRenderRes {
            mirror: ChunkMirror::new(),
            entities: BTreeMap::new(),
            terrain_material,
        })
        .insert_resource(TransportRes::new(fake_transport))
        .insert_resource(PendingMeshesRes::default())
        .insert_resource(EditFrameTracker::default())
        .insert_resource(MeshStats::default())
        .insert_resource(ServerTickRes::default())
        .insert_resource(FrameCounter::default())
        .insert_resource(ViewCapRes {
            view_distance: 0,
            chunk_cap: 3,
        })
        .add_systems(
            Update,
            (
                drain_transport_system,
                collect_mesh_tasks_system,
                spawn_mesh_tasks_system,
            )
                .chain(),
        );

        app.world_mut().spawn((
            Transform::from_xyz(0.5, -48.0, 0.5),
            FlyCamera {
                yaw: 0.0,
                pitch: 0.0,
                cursor_grabbed: true,
            },
        ));

        let make_stone_chunk = || {
            let mut c = Chunk::air();
            // 世界 (chunk_x*16, -60, 0) → 局部 (0, 4, 0),保证 section 0 网格非空。
            c.set(0, 4, 0, STONE);
            c
        };

        // ① 入队 (0,0)/(1,0)/(2,0),有界泵 ≤64 帧直到三 chunk 各有 ≥1 个 subchunk 实体。
        {
            let mut q = inbound.lock().expect("queue lock");
            for cx in 0..=2 {
                q.push(S2C::ChunkData {
                    pos: ChunkPos::new(cx, 0),
                    chunk: make_stone_chunk(),
                });
            }
        }
        let mut all_uploaded = false;
        for _ in 0..64 {
            app.update();
            let wr = app.world().resource::<WorldRenderRes>();
            let has = |cx: i32| {
                wr.entities
                    .keys()
                    .any(|sc| sc.chunk == ChunkPos::new(cx, 0))
            };
            if has(0) && has(1) && has(2) {
                all_uploaded = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(
            all_uploaded,
            "前置断言:≤64 帧内 (0,0)/(1,0)/(2,0) 必须各有 ≥1 个 subchunk 实体"
        );

        // ② 入队 (3,0)/(4,0),泵 1 帧 → protect={(0,0)},驱逐 (1,0)/(2,0),evictions==2,unloads==0。
        {
            let mut q = inbound.lock().expect("queue lock");
            q.push(S2C::ChunkData {
                pos: ChunkPos::new(3, 0),
                chunk: make_stone_chunk(),
            });
            q.push(S2C::ChunkData {
                pos: ChunkPos::new(4, 0),
                chunk: make_stone_chunk(),
            });
        }
        app.update();

        {
            let wr = app.world().resource::<WorldRenderRes>();
            assert_eq!(wr.mirror.chunk_count(), 3);
            assert!(wr.mirror.chunk_arc(ChunkPos::new(1, 0)).is_none());
            assert!(
                !wr.entities
                    .keys()
                    .any(|sc| sc.chunk == ChunkPos::new(1, 0) || sc.chunk == ChunkPos::new(2, 0)),
                "(1,0)/(2,0) 的 subchunk 实体必须全部 despawn"
            );
            assert_eq!(
                app.world().resource::<Assets<Mesh>>().len(),
                wr.entities.len(),
                "Assets<Mesh>.len() 必须恒等于 entities.len()"
            );
            let st = *app.world().resource::<MeshStats>();
            assert_eq!(st.evictions, 2);
            assert_eq!(st.unloads, 0);
        }

        // ③ 入队 ChunkUnload{(3,0)},泵 1 帧 → unloads==1,(3,0) 实体清空,assets==entities 仍成立。
        {
            let mut q = inbound.lock().expect("queue lock");
            q.push(S2C::ChunkUnload {
                pos: ChunkPos::new(3, 0),
            });
        }
        app.update();

        {
            let wr = app.world().resource::<WorldRenderRes>();
            let st = *app.world().resource::<MeshStats>();
            assert_eq!(st.unloads, 1);
            assert!(
                !wr.entities.keys().any(|sc| sc.chunk == ChunkPos::new(3, 0)),
                "(3,0) 实体必须清空"
            );
            assert_eq!(
                app.world().resource::<Assets<Mesh>>().len(),
                wr.entities.len(),
                "ChunkUnload 后 Assets<Mesh>.len() == entities.len() 必须成立"
            );
        }
    }

    /// MS24(REQ-013 C02-2 D3 + R1 B4 / PM30 靶点):无头 App 真实调度
    /// `Startup::setup_scene`(脚本模式)+ `script_driver_system` + `hud_update_system`,
    /// 验证初始 `Hidden`、手算五行文本串、`set_hud_visibility` 直调、以及三步脚本
    /// (`screenshot hud=true` → `wait_ticks 2` → `screenshot hud=false`)的四帧可见性时序。
    #[test]
    fn ms24_script_shot_hud_visibility_and_text() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let scenario = parse_scenario(
            "name = \"ms24\"\ndescription = \"shot.hud\"\n\n\
             [[step]]\nscreenshot = { name = \"s1\", expect = \"hud on\", hud = true }\n\
             [[step]]\nwait_ticks = { n = 2 }\n\
             [[step]]\nscreenshot = { name = \"s2\", expect = \"hud off\", hud = false }\n",
        )
        .expect("ms24 场景合法");
        let manifest = Manifest::new(&scenario);
        let script_res = ScriptModeRes {
            scenario,
            out_dir: tmp.path().to_path_buf(),
            world: WorldState::new(),
            engine: None,
            manifest,
            coupled: true,
            last_observer_pos: None,
            pending_shots: Arc::new(AtomicUsize::new(0)),
            pending_spawn: None,
            finished_exit: None,
            disconnected: None,
        };

        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()));
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();

        let (fake_transport, _inbound) = FakeTransport::new();
        app.insert_resource(TransportRes::new(fake_transport))
            .insert_resource(FrameCounter::default())
            .insert_resource(MeshStats::default())
            .insert_resource(MeshPerfRes::default())
            .insert_resource(FpsHistoryRes::default())
            .insert_resource(ServerPerfRes::default())
            .insert_resource(ServerTickRes::default())
            .insert_resource(ToolRes::default())
            .insert_resource(PossessRes::default())
            .insert_resource(script_res)
            .add_systems(Startup, setup_scene)
            .add_systems(
                Update,
                (
                    advance_frame_counter_system,
                    script_driver_system,
                    hud_update_system,
                )
                    .chain(),
            );

        let hud_vis = |world: &mut World| -> Visibility {
            let mut q = world.query_filtered::<&Visibility, With<HudRoot>>();
            *q.single(world).expect("必须存在唯一 HudRoot")
        };
        let screenshot_count = |world: &mut World| -> usize {
            let mut q = world.query_filtered::<Entity, With<Screenshot>>();
            q.iter(world).count()
        };

        // 首帧执行 Startup::setup_scene(tick_known=false, script_driver 早退)+ hud_update_system。
        app.update();
        assert_eq!(
            hud_vis(app.world_mut()),
            Visibility::Hidden,
            "脚本模式 setup 后 HUD 根初始必须为 Hidden"
        );
        {
            let mut q = app.world_mut().query_filtered::<&Text, With<HudText>>();
            let text = q.single(app.world()).expect("必须存在唯一 HudText");
            assert_eq!(
                text.0,
                "fps: 0.0 (min 0.0)\ntick: 0\nchunks: 0\ntool: material dirt | placeable single block | shape single | preset 20 m/s | target -\npossess: NoBody"
            );
        }

        // 直调 set_hud_visibility 断言。
        set_hud_visibility(app.world_mut(), true);
        assert_eq!(hud_vis(app.world_mut()), Visibility::Inherited);
        set_hud_visibility(app.world_mut(), false);
        assert_eq!(hud_vis(app.world_mut()), Visibility::Hidden);

        let push_tick = |world: &mut World, t: u64| {
            let mut sm = world.resource_mut::<ScriptModeRes>();
            sm.world.apply(ViewEvent::Entities {
                tick: types::Tick(t),
                states: vec![],
                removed: vec![],
                full: false,
            });
        };

        // update1(tick=1):step 1 screenshot hud=true 触发 → Inherited,且同帧不 spawn Screenshot(R3)。
        push_tick(app.world_mut(), 1);
        app.update();
        assert_eq!(
            hud_vis(app.world_mut()),
            Visibility::Inherited,
            "update1 执行 hud=true 截图步后必须为 Inherited"
        );
        assert_eq!(
            screenshot_count(app.world_mut()),
            0,
            "update1 shot 帧只切显隐与缓存 pending_spawn,世界内 Screenshot 实体数必须为 0(R3)"
        );

        // 无头无 RenderPlugin 不会触发 ScreenshotCaptured,模拟上一步截图完成以继续推进。
        app.world()
            .resource::<ScriptModeRes>()
            .pending_shots
            .store(0, Ordering::SeqCst);

        // update2(tick=2):延迟 1 帧 spawn Screenshot 实体(==1),step 2 wait_ticks 2 入等 → 仍 Inherited。
        push_tick(app.world_mut(), 2);
        app.update();
        assert_eq!(
            screenshot_count(app.world_mut()),
            1,
            "update2 延迟 1 帧后世界内必须恰有 1 个 Screenshot 实体(R3)"
        );
        assert_eq!(
            hud_vis(app.world_mut()),
            Visibility::Inherited,
            "update2 处于 wait_ticks 2 期间 HUD 显隐持续保持 Inherited"
        );

        // update3(tick=3):wait_ticks 2 未满(3 < 4)→ 仍 Inherited。
        push_tick(app.world_mut(), 3);
        app.update();
        assert_eq!(hud_vis(app.world_mut()), Visibility::Inherited);

        // update4(tick=4):wait_ticks 2 结束并执行 step 3 screenshot hud=false → Hidden。
        push_tick(app.world_mut(), 4);
        app.update();
        assert_eq!(
            hud_vis(app.world_mut()),
            Visibility::Hidden,
            "update4 执行 hud=false 截图步后必须切回 Hidden"
        );
    }

    /// MS23(REQ-013 C02-2 D4 / PM31 靶点):裸 App 调度 `fly_camera_system`,六例手算断言。
    #[test]
    fn ms23_fly_camera_headless_six_cases() {
        let run_case = |press_keys: &[KeyCode],
                        delta: Vec2,
                        possessed: Option<EntityId>|
         -> (Vec3, f32, f32, usize) {
            let mut app = App::new();
            app.add_message::<AppExit>();
            let mut time = Time::<()>::default();
            time.advance_by(Duration::from_secs_f32(0.5));
            let mut keys = ButtonInput::<KeyCode>::default();
            for &k in press_keys {
                keys.press(k);
            }
            let possess = PossessRes {
                possessed,
                ..PossessRes::default()
            };
            app.insert_resource(time)
                .insert_resource(keys)
                .insert_resource(ButtonInput::<MouseButton>::default())
                .insert_resource(AccumulatedMouseMotion { delta })
                .insert_resource(possess)
                .add_systems(Update, fly_camera_system);

            let cam = app
                .world_mut()
                .spawn((
                    Transform::from_xyz(0.0, 0.0, 0.0),
                    FlyCamera {
                        yaw: 0.0,
                        pitch: 0.0,
                        cursor_grabbed: true,
                    },
                ))
                .id();
            app.update();

            let tf = *app.world().get::<Transform>(cam).expect("Transform");
            let fly = app.world().get::<FlyCamera>(cam).expect("FlyCamera");
            let exits = app.world().resource::<Messages<AppExit>>().len();
            (tf.translation, fly.yaw, fly.pitch, exits)
        };

        // a: W held → (8, 0, 0) (16 m/s × 0.5 s, yaw 0 前向 = +x)。
        let (pos, _, _, _) = run_case(&[KeyCode::KeyW], Vec2::ZERO, None);
        assert!(
            (pos - Vec3::new(8.0, 0.0, 0.0)).length() < 1e-5,
            "MS23 a: 实得 {pos:?}"
        );

        // b: W+D held → (8/√2, 0, 8/√2), eps 1e-5;手算 5.656854249492380…,
        // f64 字面量转 f32 保精度并避 clippy::excessive_precision。
        let (pos, _, _, _) = run_case(&[KeyCode::KeyW, KeyCode::KeyD], Vec2::ZERO, None);
        assert!(
            (pos.x - 5.656_854_249_492_38_f64 as f32).abs() < 1e-5
                && pos.y.abs() < 1e-5
                && (pos.z - 5.656_854_249_492_38_f64 as f32).abs() < 1e-5,
            "MS23 b: 实得 {pos:?}"
        );

        // c: mouse delta (100, 50), grabbed → yaw == 0.3, pitch == -0.15, eps 1e-6。
        let (_, yaw, pitch, _) = run_case(&[], Vec2::new(100.0, 50.0), None);
        assert!(
            (yaw - 0.3).abs() < 1e-6 && (pitch - (-0.15)).abs() < 1e-6,
            "MS23 c: 实得 yaw={yaw}, pitch={pitch}"
        );

        // d: PossessRes.possessed = Some(EntityId(1)) + W held → translation 不变 (0, 0, 0)。
        let (pos, _, _, _) = run_case(
            &[KeyCode::KeyW],
            Vec2::ZERO,
            Some(EntityId::from_parts(0, 1)),
        );
        assert_eq!(pos, Vec3::ZERO, "MS23 d: 附身时 WASD 不得平移飞行相机");

        // e: just_pressed Escape → Messages<AppExit> 当帧消息数 == 1。
        let (_, _, _, exits) = run_case(&[KeyCode::Escape], Vec2::ZERO, None);
        assert_eq!(exits, 1, "MS23 e: Escape 必须发出 1 条 AppExit");

        // f: mouse delta (0, -10000) → pitch == 1.55334303, eps 1e-6 (89° clamp 上界)。
        let (_, _, pitch, _) = run_case(&[], Vec2::new(0.0, -10_000.0), None);
        assert!(
            (pitch - 1.553_343).abs() < 1e-6,
            "MS23 f: pitch 上界 clamp 实得 {pitch}"
        );
    }
}
