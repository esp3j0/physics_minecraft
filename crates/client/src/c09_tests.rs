//! C09 交互附身端到端无头集成测试(REQ-013 #27, IT1–IT5)。
//!
//! 装配口径:真服务器权威(`app::local::spawn_in_current_runtime`)+ tokio 暂停时钟 +
//! Bevy 0.20.1 无头交互系统链(`MinimalPlugins + AssetPlugin + GizmoPlugin`),
//! 零生产代码改动,使用 `QuadStand` 恒站立狗保证确定性。

use std::any::TypeId;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use app::config::{Config, StartupAgentConfig, StartupController};
use app::local::{LocalConfig, spawn_in_current_runtime};
use bevy::app::AppExit;
use bevy::gizmos::config::DefaultGizmoConfigGroup;
use bevy::gizmos::{GizmoAsset, GizmoHandles, GizmoPlugin};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::mesh::skinning::SkinnedMeshInverseBindposes;
use bevy::prelude::*;
use client_core::mirror::ChunkMirror;
use client_core::possess;
use client_core::script::{EngineCommand, engine_commands_to_c2s};
use client_core::{LocalTransport, SequencedTransport, Transport};
use sim_api::proto::{C2S, S2C};
use types::{BlockPos, EntityId, WORLD_Y_MIN, WorldSeed};

use super::*;

const MAX_WAIT_TICKS: usize = 600;
const MAX_TEST_TICKS: usize = 2000;
const VIEW_DISTANCE: u8 = 3;
const CHUNK_CAP: usize = 512;

fn repo_bodies_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies")
}

fn quad_stand_agent(pos: [f32; 3]) -> StartupAgentConfig {
    StartupAgentConfig {
        body: "quad".to_owned(),
        position: pos,
        yaw_deg: 0.0,
        controller: StartupController::QuadStand,
        leash_m: 16.0,
        policy: None,
        intent: [0.0; 3],
    }
}

/// 记录型传输 wrapper:`send` 记录原始 `C2S` 帧后再委托给内层 `SequencedTransport`。
struct RecordingTransportWrapper<T: Transport> {
    inner: T,
    sent: Arc<Mutex<Vec<C2S>>>,
}

impl<T: Transport> Transport for RecordingTransportWrapper<T> {
    fn send(&mut self, c2s: C2S) {
        self.sent
            .lock()
            .expect("RecordingTransportWrapper sent lock")
            .push(c2s.clone());
        self.inner.send(c2s);
    }

    fn drain(&mut self) -> Vec<S2C> {
        self.inner.drain()
    }
}

/// C09 共享无头测试装配(真服务器 + 暂停时钟 + Bevy 交互系统链)。
struct C09Harness {
    _temp: tempfile::TempDir,
    driver: tokio::task::JoinHandle<app::driver::DriverOutcome>,
    app: App,
    cam: Entity,
    hud: Entity,
    sent: Arc<Mutex<Vec<C2S>>>,
    total_ticks: usize,
    pending_release: Vec<KeyCode>,
}

impl C09Harness {
    async fn spawn(agents: Vec<StartupAgentConfig>, cam_pos: Vec3, yaw: f32, pitch: f32) -> Self {
        let temp = tempfile::TempDir::new().expect("TempDir");
        let bodies_dir = repo_bodies_dir();
        let cfg = LocalConfig {
            config: Config {
                data_dir: temp.path().to_path_buf(),
                bodies_dir: bodies_dir.clone(),
                policies_dir: temp.path().join("policies"),
                world_seed: WorldSeed(3),
                agents,
                ..Config::default()
            },
            view_distance: VIEW_DISTANCE,
        };

        let (handle, driver) = spawn_in_current_runtime(cfg)
            .await
            .expect("spawn_in_current_runtime");
        // ac4 暂停时钟对齐口径:先让驱动消费首枚立即 tick。
        tokio::task::yield_now().await;

        let sent = Arc::new(Mutex::new(Vec::new()));
        let transport = RecordingTransportWrapper {
            inner: SequencedTransport::new(LocalTransport::from_handle(handle)),
            sent: Arc::clone(&sent),
        };

        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()));
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();
        app.init_asset::<SkinnedMeshInverseBindposes>();
        app.add_plugins(GizmoPlugin);
        app.add_message::<AppExit>();

        let (terrain_material, outline_material, entity_render) = {
            let mut mats = app.world_mut().resource_mut::<Assets<StandardMaterial>>();
            let tm = mats.add(StandardMaterial::default());
            let om = mats.add(StandardMaterial::default());
            let er = EntityRenderRes::new(&bodies_dir, &mut mats);
            (tm, om, er)
        };

        app.insert_resource(TransportRes::new(transport))
            .insert_resource(WorldRenderRes {
                mirror: ChunkMirror::new(),
                entities: BTreeMap::new(),
                terrain_material,
            })
            .insert_resource(OutlineRes {
                enabled: false,
                entities: BTreeMap::new(),
                material: outline_material,
            })
            .insert_resource(EntityMirrorRes::default())
            .insert_resource(entity_render)
            .insert_resource(PossessRes::default())
            .insert_resource(ToolRes::default())
            .insert_resource(ViewCapRes {
                view_distance: VIEW_DISTANCE,
                chunk_cap: CHUNK_CAP,
            })
            .insert_resource(FrameCounter::default())
            .insert_resource(PendingMeshesRes::default())
            .insert_resource(EditFrameTracker::default())
            .insert_resource(MeshStats::default())
            .insert_resource(ServerTickRes::default())
            .insert_resource(FpsHistoryRes::default())
            .insert_resource(ObserverTracker::default())
            .insert_resource(ButtonInput::<KeyCode>::default())
            .insert_resource(ButtonInput::<MouseButton>::default())
            .insert_resource(AccumulatedMouseMotion::default())
            .insert_resource(AccumulatedMouseScroll::default())
            .add_systems(
                Update,
                (
                    fly_camera_system,
                    sync_observer_system,
                    interact_system,
                    drain_transport_system,
                    sync_entities_system,
                    agent_highlight_system,
                    hud_update_system,
                )
                    .chain(),
            );

        let cam = app
            .world_mut()
            .spawn((
                Transform::from_translation(cam_pos),
                FlyCamera {
                    yaw,
                    pitch,
                    cursor_grabbed: true,
                },
            ))
            .id();
        let hud = app
            .world_mut()
            .spawn((Text::new(String::new()), HudText))
            .id();

        Self {
            _temp: temp,
            driver,
            app,
            cam,
            hud,
            sent,
            total_ticks: 0,
            pending_release: Vec::new(),
        }
    }

    /// 推进 1 拍:advance(17ms)+yield → 可选瞄准写回 → 按键单沿更新 → `app.update()`。
    async fn step(&mut self, aim_target: Option<EntityId>, press_keys: &[KeyCode]) {
        assert!(
            self.total_ticks < MAX_TEST_TICKS,
            "单测试总拍数超过上限 {MAX_TEST_TICKS}(当前 tick={}, phase={:?}, possessed={:?})",
            self.server_tick(),
            self.phase(),
            self.possessed(),
        );
        self.total_ticks += 1;

        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;

        if let Some(target_id) = aim_target
            && let Some(root) = self.agent_root(target_id)
        {
            self.aim_at_world_pos(conv::to_bevy_vec3(root));
        }

        {
            let mut keys = self.app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
            keys.clear();
            for k in self.pending_release.drain(..) {
                keys.release(k);
            }
            for &k in press_keys {
                keys.press(k);
                self.pending_release.push(k);
            }
        }

        self.app.update();
    }

    /// 由相机 Transform 与世界目标点反算写回 `FlyCamera` 的弧度 yaw/pitch。
    fn aim_at_world_pos(&mut self, target: Vec3) {
        let cam_pos = self.cam_pos();
        let delta = target - cam_pos;
        let len = delta.length();
        if len > 1e-6 {
            let mut fly = self
                .app
                .world_mut()
                .get_mut::<FlyCamera>(self.cam)
                .expect("FlyCamera");
            fly.yaw = delta.z.atan2(delta.x);
            fly.pitch = (delta.y / len).clamp(-1.0, 1.0).asin();
        }
    }

    fn set_camera_pos(&mut self, pos: Vec3) {
        let mut tf = self
            .app
            .world_mut()
            .get_mut::<Transform>(self.cam)
            .expect("cam Transform");
        tf.translation = pos;
    }

    fn set_camera_angles(&mut self, yaw: f32, pitch: f32) {
        let mut fly = self
            .app
            .world_mut()
            .get_mut::<FlyCamera>(self.cam)
            .expect("FlyCamera");
        fly.yaw = yaw;
        fly.pitch = pitch;
    }

    fn cam_pos(&self) -> Vec3 {
        self.app
            .world()
            .get::<Transform>(self.cam)
            .expect("cam Transform")
            .translation
    }

    fn hud_text(&self) -> String {
        self.app
            .world()
            .get::<Text>(self.hud)
            .expect("HudText")
            .0
            .clone()
    }

    fn server_tick(&self) -> u64 {
        self.app.world().resource::<ServerTickRes>().0
    }

    fn phase(&self) -> possess::Phase {
        self.app.world().resource::<PossessRes>().state.phase()
    }

    fn possessed(&self) -> Option<EntityId> {
        self.app.world().resource::<PossessRes>().possessed
    }

    fn state_possessed(&self) -> Option<EntityId> {
        self.app.world().resource::<PossessRes>().state.possessed()
    }

    fn my_body(&self) -> Option<EntityId> {
        self.app.world().resource::<PossessRes>().state.my_body()
    }

    fn agent_target(&self) -> Option<EntityId> {
        self.app
            .world()
            .resource::<ToolRes>()
            .agent_target
            .map(|h| h.entity)
    }

    fn last_hit_block(&self) -> Option<BlockPos> {
        self.app
            .world()
            .resource::<ToolRes>()
            .last_hit
            .map(|h| h.block_pos)
    }

    fn block_at(&self, pos: BlockPos) -> Option<types::BlockId> {
        self.app
            .world()
            .resource::<WorldRenderRes>()
            .mirror
            .block(pos)
    }

    fn agent_root(&self, id: EntityId) -> Option<types::glam::Vec3> {
        self.app
            .world()
            .resource::<EntityMirrorRes>()
            .0
            .agents
            .get(&id)
            .and_then(|v| v.poses.first())
            .map(|p| p.translation)
    }

    fn find_agent_near_x(&self, expected_x: f32, tol: f32) -> Option<EntityId> {
        self.app
            .world()
            .resource::<EntityMirrorRes>()
            .0
            .agents
            .iter()
            .find_map(|(&id, view)| {
                let root = view.poses.first()?;
                ((root.translation.x - expected_x).abs() <= tol).then_some(id)
            })
    }

    fn sent_snapshot(&self) -> Vec<C2S> {
        self.sent.lock().expect("sent lock").clone()
    }

    fn send_c2s(&mut self, msgs: impl IntoIterator<Item = C2S>) {
        self.app
            .world_mut()
            .resource_mut::<TransportRes>()
            .with_mut(|t| {
                for m in msgs {
                    t.send(m);
                }
            });
    }

    async fn wait_for_agent_near(&mut self, expected_x: f32) -> (EntityId, usize) {
        for waited in 1..=MAX_WAIT_TICKS {
            self.step(None, &[]).await;
            if let Some(id) = self.find_agent_near_x(expected_x, 1.5) {
                return (id, waited);
            }
        }
        let ids: Vec<(EntityId, Option<types::glam::Vec3>)> = self
            .app
            .world()
            .resource::<EntityMirrorRes>()
            .0
            .agents
            .iter()
            .map(|(&id, v)| (id, v.poses.first().map(|p| p.translation)))
            .collect();
        panic!(
            "上限 {MAX_WAIT_TICKS} 拍内未在 x≈{expected_x} 见到主体(tick={}, agents={ids:?})",
            self.server_tick()
        );
    }

    async fn wait_aim_hit(&mut self, target: EntityId) -> usize {
        for waited in 1..=MAX_WAIT_TICKS {
            self.step(Some(target), &[]).await;
            if self.agent_target() == Some(target) {
                return waited;
            }
        }
        panic!(
            "上限 {MAX_WAIT_TICKS} 拍内瞄准 {target:?} 未命中(agent_target={:?}, last_hit={:?}, cam={:?}, root={:?})",
            self.agent_target(),
            self.last_hit_block(),
            self.cam_pos(),
            self.agent_root(target),
        );
    }

    async fn wait_agent_settled(&mut self, target: EntityId, aim: Option<EntityId>) -> usize {
        let mut prev: Option<types::glam::Vec3> = None;
        let mut stable = 0usize;
        for waited in 1..=MAX_WAIT_TICKS {
            self.step(aim, &[]).await;
            let cur = self.agent_root(target);
            if let (Some(p), Some(c)) = (prev, cur) {
                if (c - p).length() < 1e-4 {
                    stable += 1;
                    if stable >= 3 {
                        return waited;
                    }
                } else {
                    stable = 0;
                }
            }
            prev = cur;
        }
        panic!(
            "上限 {MAX_WAIT_TICKS} 拍内主体 {target:?} 未站稳(root={:?}, prev={prev:?})",
            self.agent_root(target),
        );
    }

    async fn wait_possessed(&mut self, expected: Option<EntityId>, aim: Option<EntityId>) -> usize {
        for waited in 1..=MAX_WAIT_TICKS {
            self.step(aim, &[]).await;
            if self.possessed() == expected
                && self.state_possessed() == expected
                && (expected.is_none() || self.phase() == possess::Phase::Possessed)
            {
                return waited;
            }
        }
        panic!(
            "上限 {MAX_WAIT_TICKS} 拍内未达 possessed={expected:?}(实得 possessed={:?}, state_possessed={:?}, phase={:?}, tick={})",
            self.possessed(),
            self.state_possessed(),
            self.phase(),
            self.server_tick(),
        );
    }

    async fn shutdown(mut self) {
        self.app.world_mut().resource_mut::<TransportRes>().close();
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        self.driver.await.expect("app-local-host driver 正常退出");
    }
}

fn color_close(a: LinearRgba, b: LinearRgba, tol: f32) -> bool {
    (a.red - b.red).abs() <= tol
        && (a.green - b.green).abs() <= tol
        && (a.blue - b.blue).abs() <= tol
        && (a.alpha - b.alpha).abs() <= tol
}

/// IT1(清单 1):指狗附身 / 第三人称相机位置 / WASD 路由到意图而非相机 / 解除回 NoBody。
#[tokio::test(start_paused = true)]
async fn c09_it1_possess_third_person_wasd_unpossess() {
    let mut h = C09Harness::spawn(
        vec![quad_stand_agent([24.5, -59.5, 24.5])],
        Vec3::new(20.5, -59.5, 24.5),
        0.0,
        0.0,
    )
    .await;

    let (a_id, w_appear) = h.wait_for_agent_near(24.5).await;
    let w_settle = h.wait_agent_settled(a_id, Some(a_id)).await;
    let w_aim = h.wait_aim_hit(a_id).await;
    assert_eq!(h.agent_target(), Some(a_id), "按 P 前必须命中狗 A");

    // 1. 按 P → 发 C2S::Possess { entity: A },等待服务器确认附身。
    h.step(Some(a_id), &[KeyCode::KeyP]).await;
    let sent_after_p = h.sent_snapshot();
    let possess_idx = sent_after_p
        .iter()
        .position(|c| *c == C2S::Possess { entity: a_id })
        .unwrap_or_else(|| {
            panic!("IT1 按 P 后记录必须含 C2S::Possess({a_id:?}),实得:{sent_after_p:?}")
        });

    let w_possess = h.wait_possessed(Some(a_id), Some(a_id)).await;
    assert_eq!(h.phase(), possess::Phase::Possessed);
    assert_eq!(h.state_possessed(), Some(a_id));

    // 2. 停瞄准,置 yaw=0 / pitch=0 → 下一拍验证第三人称手算相机位。
    h.set_camera_angles(0.0, 0.0);
    h.step(None, &[]).await;
    let root_a = h.agent_root(a_id).expect("mirror 必含狗 A root");
    let cam_after_tp = h.cam_pos();
    // 手算:fwd=(1,0,0), dist=CAMERA_DIST_DEFAULT=4.0, LOOK_HEIGHT=0.6, pos=look-fwd*dist。
    let expected_cam = Vec3::new(root_a.x - 4.0, root_a.y + 0.6, root_a.z);
    let cam_err = (cam_after_tp - expected_cam).length();
    assert!(
        cam_err <= 1e-3,
        "IT1 第三人称相机位不符:实得 {cam_after_tp:?},手算期望 {expected_cam:?}(root={root_a:?}),误差={cam_err}"
    );

    // 3. 相机校验拍无按键已发首条零速意图并消费 last_sent,须跨过 50ms 真实墙钟节流
    //    (暂停 tokio 时钟不影响 Instant::now;55ms 不推进服务器拍),再按 W 验证非零 move_xz
    //    的 C2S::AgentIntent 且相机位置不变。
    std::thread::sleep(Duration::from_millis(55));
    let sent_before_w = h.sent_snapshot().len();
    let cam_before_w = h.cam_pos();
    h.step(None, &[KeyCode::KeyW]).await;
    let cam_after_w = h.cam_pos();
    let sent_w = h.sent_snapshot();
    let intent_idx = sent_w[sent_before_w..]
        .iter()
        .position(|c| {
            matches!(
                c,
                C2S::AgentIntent { entity, move_xz, .. }
                    if *entity == a_id && (move_xz.0.abs() > 1e-3 || move_xz.1.abs() > 1e-3)
            )
        })
        .map(|rel| sent_before_w + rel)
        .unwrap_or_else(|| {
            panic!(
                "IT1 附身按 W 必须发出非零 AgentIntent({a_id:?}),实得:{:?}",
                &sent_w[sent_before_w..]
            )
        });
    let cam_shift_w = (cam_after_w - cam_before_w).length();
    assert!(
        cam_shift_w <= 1e-3,
        "IT1 附身按 W 相机位置不得平移(tol 1e-3):before={cam_before_w:?}, after={cam_after_w:?}, diff={cam_shift_w}"
    );

    // 4. 再按 P → 发 C2S::Unpossess → 等 Possessed(None) → 相位 NoBody、y >= WORLD_Y_MIN。
    let sent_before_un = h.sent_snapshot().len();
    h.step(None, &[KeyCode::KeyP]).await;
    let sent_un = h.sent_snapshot();
    let unpossess_idx = sent_un[sent_before_un..]
        .iter()
        .position(|c| *c == C2S::Unpossess)
        .map(|rel| sent_before_un + rel)
        .unwrap_or_else(|| {
            panic!(
                "IT1 附身按 P 必须发出 C2S::Unpossess,实得:{:?}",
                &sent_un[sent_before_un..]
            )
        });

    let w_unpossess = h.wait_possessed(None, None).await;
    assert_eq!(
        h.phase(),
        possess::Phase::NoBody,
        "IT1 解除附身后 my_body 为空必须回 NoBody"
    );
    assert_eq!(h.my_body(), None);
    let final_y = h.cam_pos().y;
    assert!(
        final_y >= WORLD_Y_MIN as f32,
        "IT1 解除后相机 y={final_y} 必须 >= WORLD_Y_MIN({WORLD_Y_MIN})"
    );

    eprintln!(
        "[C09 IT1] a_id={a_id:?}, waited(appear={w_appear}, settle={w_settle}, aim={w_aim}, possess={w_possess}, unpossess={w_unpossess}), total_ticks={}, c2s_idx(possess={possess_idx}, intent={intent_idx}, unpossess={unpossess_idx}), cam_err={cam_err:.6}, cam_shift_w={cam_shift_w:.6}",
        h.total_ticks
    );

    h.shutdown().await;
}

/// IT2(清单 2):隔墙放置 STONE 于 (22,-60,24) 遮挡狗 A,瞄准 A 时 agent_target==None,
/// 按 P 后 60 拍内不发 `C2S::Possess { entity: A }`。
#[tokio::test(start_paused = true)]
async fn c09_it2_wall_occlusion_prevents_possess() {
    let mut h = C09Harness::spawn(
        vec![quad_stand_agent([24.5, -59.5, 24.5])],
        Vec3::new(20.5, -59.5, 24.5),
        0.0,
        0.0,
    )
    .await;

    let wall_pos = BlockPos::new(22, -60, 24);
    let (a_id, w_appear) = h.wait_for_agent_near(24.5).await;

    // 等待 (22,-60,24) 所在 chunk 到达镜像。
    let mut w_chunk = 0usize;
    for waited in 1..=MAX_WAIT_TICKS {
        h.step(None, &[]).await;
        if h.block_at(wall_pos).is_some() {
            w_chunk = waited;
            break;
        }
    }
    assert!(
        h.block_at(wall_pos).is_some(),
        "IT2 上限 {MAX_WAIT_TICKS} 拍内 chunk(1,1) 必须到达镜像"
    );

    // 经协议映射器构造并发送 SetBlock((22,-60,24), STONE)。
    let edit_msgs = engine_commands_to_c2s(&[EngineCommand::SetBlock {
        pos: wall_pos,
        block: voxel::builtin::STONE,
    }]);
    h.send_c2s(edit_msgs);

    let mut w_stone = 0usize;
    for waited in 1..=MAX_WAIT_TICKS {
        h.step(None, &[]).await;
        if h.block_at(wall_pos) == Some(voxel::builtin::STONE) {
            w_stone = waited;
            break;
        }
    }
    assert_eq!(
        h.block_at(wall_pos),
        Some(voxel::builtin::STONE),
        "IT2 上限 {MAX_WAIT_TICKS} 拍内 (22,-60,24) 必须变为 STONE"
    );

    // 瞄准狗 A:视线被墙格 t=1.5 < agent t≈4 遮挡,agent_target 必须为 None。
    h.step(Some(a_id), &[]).await;
    assert_eq!(
        h.agent_target(),
        None,
        "IT2 隔墙瞄准 A 时 agent_target 必须为 None(last_hit={:?}, root={:?})",
        h.last_hit_block(),
        h.agent_root(a_id),
    );
    assert_eq!(
        h.last_hit_block(),
        Some(wall_pos),
        "IT2 隔墙瞄准 A 时应命中墙格 {wall_pos:?}"
    );

    // 按 P 后继续推 60 拍,断言全程记录无 C2S::Possess { entity: a_id }。
    h.step(Some(a_id), &[KeyCode::KeyP]).await;
    for _ in 0..60 {
        let aim = if h.possessed().is_none() {
            Some(a_id)
        } else {
            None
        };
        h.step(aim, &[]).await;
    }

    let sent = h.sent_snapshot();
    assert!(
        !sent.contains(&C2S::Possess { entity: a_id }),
        "IT2 隔墙按 P 后 60 拍内不得发出 C2S::Possess({a_id:?}),实得:{sent:?}"
    );
    assert_ne!(
        h.possessed(),
        Some(a_id),
        "IT2 隔墙按 P 后不得附身狗 A({a_id:?})"
    );

    eprintln!(
        "[C09 IT2] a_id={a_id:?}, waited(appear={w_appear}, chunk={w_chunk}, stone={w_stone}), total_ticks={}, sent_len={}",
        h.total_ticks,
        sent.len()
    );

    h.shutdown().await;
}

/// IT3(清单 3):无狗指地面实格 (24,-61,20) 按 P → 发 SpawnAgent → 自动附身新主体,
/// 记录中 SpawnAgent 先于任何 Possess。
#[tokio::test(start_paused = true)]
async fn c09_it3_ground_spawn_and_auto_possess() {
    let ground_cell = BlockPos::new(24, -61, 20);
    let mut h = C09Harness::spawn(
        Vec::new(),
        Vec3::new(24.5, -59.5, 20.5),
        0.0,
        -std::f32::consts::FRAC_PI_2,
    )
    .await;

    let mut w_ground = 0usize;
    for waited in 1..=MAX_WAIT_TICKS {
        h.step(None, &[]).await;
        if h.block_at(ground_cell) == Some(voxel::builtin::DIRT)
            && h.last_hit_block() == Some(ground_cell)
        {
            w_ground = waited;
            break;
        }
    }
    assert_eq!(
        h.block_at(ground_cell),
        Some(voxel::builtin::DIRT),
        "IT3 seed 3 地面实格 (24,-61,20) 必须为 DIRT"
    );
    assert_eq!(
        h.last_hit_block(),
        Some(ground_cell),
        "IT3 垂直下视必须命中地面实格 {ground_cell:?}"
    );
    assert_eq!(
        h.agent_target(),
        None,
        "IT3 无狗时 agent_target 必须为 None"
    );

    // 按 P → 发 C2S::SpawnAgent。
    h.step(None, &[KeyCode::KeyP]).await;
    let sent_after_p = h.sent_snapshot();
    assert!(
        sent_after_p
            .iter()
            .any(|c| matches!(c, C2S::SpawnAgent { .. })),
        "IT3 指地面按 P 必须发出 C2S::SpawnAgent,实得:{sent_after_p:?}"
    );

    // 等 Spawned + AgentStates + Possessed → 相位 Possessed、possessed()==新 id。
    let mut w_auto = 0usize;
    let mut new_id = None;
    for waited in 1..=MAX_WAIT_TICKS {
        h.step(None, &[]).await;
        if h.phase() == possess::Phase::Possessed
            && let Some(id) = h.possessed()
        {
            w_auto = waited;
            new_id = Some(id);
            break;
        }
    }
    let new_id = new_id.unwrap_or_else(|| {
        panic!(
            "IT3 上限 {MAX_WAIT_TICKS} 拍内未完成自动附身(phase={:?}, possessed={:?}, my_body={:?}, tick={})",
            h.phase(),
            h.possessed(),
            h.my_body(),
            h.server_tick()
        )
    });
    assert_eq!(h.state_possessed(), Some(new_id));
    assert_eq!(h.my_body(), Some(new_id));
    assert!(
        h.agent_root(new_id).is_some(),
        "IT3 新生成主体 {new_id:?} 必须存在于主体镜像"
    );

    let sent = h.sent_snapshot();
    let spawn_idx = sent
        .iter()
        .position(|c| matches!(c, C2S::SpawnAgent { .. }))
        .expect("IT3 记录必含 SpawnAgent");
    let first_possess_idx = sent
        .iter()
        .position(|c| matches!(c, C2S::Possess { .. }))
        .unwrap_or_else(|| panic!("IT3 记录必含自动发出的 Possess,实得:{sent:?}"));
    assert!(
        spawn_idx < first_possess_idx,
        "IT3 记录中 SpawnAgent(idx={spawn_idx}) 必须先于任何 Possess(idx={first_possess_idx})"
    );
    assert_eq!(
        sent[first_possess_idx],
        C2S::Possess { entity: new_id },
        "IT3 自动发出的 Possess 目标必须为新生成主体 {new_id:?}"
    );

    eprintln!(
        "[C09 IT3] new_id={new_id:?}, waited(ground={w_ground}, auto_possess={w_auto}), total_ticks={}, c2s_idx(spawn={spawn_idx}, possess={first_possess_idx})",
        h.total_ticks
    );

    h.shutdown().await;
}

/// IT4(清单 4):两狗 A+B 两按切换 — 附身 A → 瞄准 B 首按 P 得 Unpossess(无 Possess(B)) →
/// 等 Possessed(None) 后移相机至 (28.5,-59.5,20.5) 瞄准 B → 再按 P 得 Possess(B) → 确认 Possessed(B)。
#[tokio::test(start_paused = true)]
async fn c09_it4_two_press_switch_agent() {
    let mut h = C09Harness::spawn(
        vec![
            quad_stand_agent([24.5, -59.5, 24.5]),
            quad_stand_agent([28.5, -59.5, 24.5]),
        ],
        Vec3::new(20.5, -59.5, 24.5),
        0.0,
        0.0,
    )
    .await;

    let (a_id, w_a) = h.wait_for_agent_near(24.5).await;
    let (b_id, w_b) = h.wait_for_agent_near(28.5).await;
    assert_ne!(a_id, b_id, "IT4 狗 A 与狗 B 必须为不同实体");

    // 1. 先附身 A。
    let w_aim_a = h.wait_aim_hit(a_id).await;
    h.step(Some(a_id), &[KeyCode::KeyP]).await;
    let sent_1 = h.sent_snapshot();
    let idx_possess_a = sent_1
        .iter()
        .position(|c| *c == C2S::Possess { entity: a_id })
        .unwrap_or_else(|| panic!("IT4 按 P 必发 Possess(A),实得:{sent_1:?}"));
    let w_possess_a = h.wait_possessed(Some(a_id), Some(a_id)).await;
    assert_eq!(h.phase(), possess::Phase::Possessed);

    // 2. 附身 A 中瞄准 B 并首按 P → 必发 Unpossess 且此刻之前无 Possess(B)。
    h.step(Some(b_id), &[KeyCode::KeyP]).await;
    let sent_2 = h.sent_snapshot();
    let idx_unpossess = sent_2
        .iter()
        .position(|c| *c == C2S::Unpossess)
        .unwrap_or_else(|| panic!("IT4 附身 A 中按 P 必发 Unpossess,实得:{sent_2:?}"));
    assert!(
        !sent_2.contains(&C2S::Possess { entity: b_id }),
        "IT4 首按 P(解除前)不得出现 C2S::Possess(B),实得:{sent_2:?}"
    );

    let w_unpossess = h.wait_possessed(None, None).await;
    assert_eq!(h.phase(), possess::Phase::NoBody);

    // 3. 解除确认后移相机至 (28.5,-59.5,20.5) 瞄准 B(+z 方向避开 A 遮挡),再按 P → Possess(B)。
    h.set_camera_pos(Vec3::new(28.5, -59.5, 20.5));
    let w_aim_b = h.wait_aim_hit(b_id).await;
    assert_eq!(h.agent_target(), Some(b_id), "IT4 移位后瞄准 B 必须命中 B");

    h.step(Some(b_id), &[KeyCode::KeyP]).await;
    let sent_3 = h.sent_snapshot();
    let idx_possess_b = sent_3
        .iter()
        .position(|c| *c == C2S::Possess { entity: b_id })
        .unwrap_or_else(|| panic!("IT4 二按 P 必发 Possess(B),实得:{sent_3:?}"));

    let w_possess_b = h.wait_possessed(Some(b_id), Some(b_id)).await;
    assert_eq!(h.phase(), possess::Phase::Possessed);
    assert_eq!(h.state_possessed(), Some(b_id));

    assert!(
        idx_possess_a < idx_unpossess && idx_unpossess < idx_possess_b,
        "IT4 帧序号顺序错误:possess_a={idx_possess_a}, unpossess={idx_unpossess}, possess_b={idx_possess_b}"
    );

    eprintln!(
        "[C09 IT4] a_id={a_id:?}, b_id={b_id:?}, waited(a={w_a}, b={w_b}, aim_a={w_aim_a}, pos_a={w_possess_a}, unpos={w_unpossess}, aim_b={w_aim_b}, pos_b={w_possess_b}), total_ticks={}, c2s_idx(possess_a={idx_possess_a}, unpossess={idx_unpossess}, possess_b={idx_possess_b})",
        h.total_ticks
    );

    h.shutdown().await;
}

/// IT5(清单 5):瞄准狗 A 且未附身时验证 HUD 提示行与 Gizmo 琥珀色 cube(scale 2) 顶点几何,
/// 转瞄准空处后下一拍 agent_target==None、Gizmo 句柄槽为 None、HUD 不含 agent 行。
#[tokio::test(start_paused = true)]
async fn c09_it5_gizmo_wireframe_and_hud() {
    let mut h = C09Harness::spawn(
        vec![quad_stand_agent([24.5, -59.5, 24.5])],
        Vec3::new(20.5, -59.5, 24.5),
        0.0,
        0.0,
    )
    .await;

    let (a_id, w_appear) = h.wait_for_agent_near(24.5).await;
    let w_aim = h.wait_aim_hit(a_id).await;
    assert_eq!(h.possessed(), None, "IT5 必须处于未附身状态");
    assert_eq!(h.agent_target(), Some(a_id));

    // 1. HUD 文本断言。
    let expected_agent_line = format!("agent: {} — P 附身/解除", a_id.to_bits());
    let hud_hit = h.hud_text();
    assert!(
        hud_hit.contains(&expected_agent_line),
        "IT5 命中狗 A 时 HUD 必须含 `{expected_agent_line}`,实得:\n{hud_hit}"
    );

    // 2. Gizmo 公开读取路径:Res<GizmoHandles> + Res<Assets<GizmoAsset>>。
    let group_ty = TypeId::of::<DefaultGizmoConfigGroup>();
    let handle = match h
        .app
        .world()
        .resource::<GizmoHandles>()
        .handles()
        .get(&group_ty)
    {
        Some(Some(handle)) => handle.clone(),
        other => panic!("IT5 命中狗 A 时 GizmoHandles 槽位应为 Some(Some(handle)),实得:{other:?}"),
    };
    let gizmo_asset = h
        .app
        .world()
        .resource::<Assets<GizmoAsset>>()
        .get(handle.id())
        .expect("IT5 GizmoAsset 必须存在");
    let buf = gizmo_asset.buffer();

    assert_eq!(
        buf.strip_positions.len(),
        11,
        "IT5 cube strip_positions 长度必须为 11"
    );
    assert_eq!(
        buf.list_positions.len(),
        6,
        "IT5 cube list_positions 长度必须为 6"
    );
    assert_eq!(
        buf.strip_colors.len(),
        11,
        "IT5 cube strip_colors 长度必须为 11"
    );
    assert_eq!(
        buf.list_colors.len(),
        6,
        "IT5 cube list_colors 长度必须为 6"
    );

    let break_pos = buf.strip_positions[10];
    assert!(
        break_pos.x.is_nan() && break_pos.y.is_nan() && break_pos.z.is_nan(),
        "IT5 strip_positions[10] 必须全分量 NaN(strip 断点标记),实得:{break_pos:?}"
    );
    let break_color = buf.strip_colors[10];
    assert!(
        break_color.red.is_nan()
            && break_color.green.is_nan()
            && break_color.blue.is_nan()
            && break_color.alpha.is_nan(),
        "IT5 strip_colors[10] 必须全分量 NaN(strip 断点标记),实得:{break_color:?}"
    );

    let expected_amber = LinearRgba::from(Color::srgb(1.0, 0.75, 0.2));
    for (i, &c) in buf.strip_colors[..10].iter().enumerate() {
        assert!(
            color_close(c, expected_amber, 1e-6),
            "IT5 strip_colors[{i}] 应为琥珀色 {expected_amber:?},实得:{c:?}"
        );
    }
    for (i, &c) in buf.list_colors.iter().enumerate() {
        assert!(
            color_close(c, expected_amber, 1e-6),
            "IT5 list_colors[{i}] 应为琥珀色 {expected_amber:?},实得:{c:?}"
        );
    }

    // 8 个独立顶点 = strip_positions[0..4] + strip_positions[5..9],
    // 有限点(strip_positions[0..10] + list_positions 6 点)按 tol 1e-4 去重亦恰 8 个。
    let mut verts = Vec::with_capacity(8);
    verts.extend_from_slice(&buf.strip_positions[0..4]);
    verts.extend_from_slice(&buf.strip_positions[5..9]);
    assert_eq!(verts.len(), 8);

    let mut dedup: Vec<Vec3> = Vec::new();
    for &p in buf.strip_positions[..10]
        .iter()
        .chain(buf.list_positions.iter())
    {
        assert!(p.is_finite(), "IT5 有限点集合不应含 NaN/Inf,实得:{p:?}");
        if !dedup.iter().any(|&q| (p - q).length() <= 1e-4) {
            dedup.push(p);
        }
    }
    assert_eq!(
        dedup.len(),
        8,
        "IT5 cube 全部顶点按 tol 1e-4 去重必须恰为 8 个,实得 {}",
        dedup.len()
    );

    let root_a = conv::to_bevy_vec3(h.agent_root(a_id).expect("mirror 必含狗 A root"));
    let centroid = verts.iter().copied().sum::<Vec3>() / 8.0;
    let centroid_err = (centroid - root_a).length();
    assert!(
        centroid_err <= 1e-3,
        "IT5 8 顶点质心必须等于 root A:centroid={centroid:?}, root={root_a:?}, err={centroid_err}"
    );

    let expected_corner_dist = 3.0_f32.sqrt();
    let mut max_dist_err = 0.0_f32;
    for (i, &v) in verts.iter().enumerate() {
        let d = (v - root_a).length();
        let err = (d - expected_corner_dist).abs();
        max_dist_err = max_dist_err.max(err);
        assert!(
            err <= 1e-3,
            "IT5 顶点[{i}]={v:?} 距 root={root_a:?} 距离={d},期望 √3≈{expected_corner_dist},误差={err}"
        );
    }

    // 3. 转瞄准空处(抬头看天空)→ 下一拍 agent_target==None、Gizmo 句柄槽 None、HUD 无 agent 行。
    h.set_camera_angles(0.0, 0.5);
    h.step(None, &[]).await;
    assert_eq!(
        h.agent_target(),
        None,
        "IT5 转瞄准空处后 agent_target 必须为 None"
    );
    let slot_after = h
        .app
        .world()
        .resource::<GizmoHandles>()
        .handles()
        .get(&group_ty);
    assert!(
        matches!(slot_after, Some(None)),
        "IT5 无绘制帧后 GizmoHandles 槽位必须为 Some(None),实得:{slot_after:?}"
    );
    let hud_miss = h.hud_text();
    assert!(
        !hud_miss.contains("agent:"),
        "IT5 转瞄准空处后 HUD 不得含 agent 行,实得:\n{hud_miss}"
    );

    eprintln!(
        "[C09 IT5] a_id={a_id:?}, waited(appear={w_appear}, aim={w_aim}), total_ticks={}, centroid_err={centroid_err:.6}, max_dist_err={max_dist_err:.6}",
        h.total_ticks
    );

    h.shutdown().await;
}
