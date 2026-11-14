//! REQ-012 B02a 判据测试(任务书 §3):经 `Session` 的站立与行走。
//!
//! 布景:平地一层石头(顶面 world y = 1.0),四足出生在顶面 + 0.45 =
//! y 1.45(与内核 `quad_origin` / B01a 同口径)。一个分片 = 8×8 chunk =
//! 128×128 m;地面铺**满整个分片**(8×8 chunk),出生在分片中心 (64,64):
//! 3600 拍 @ §13 `gait` 速度 0.53 m/s ≈ 32 m 直线位移,从中心到边缘 64 m,
//! 走不出地面、更走不到本体区边界(多连杆迁移 B03 才做,出去会 fail-stop;
//! 首轮 3×3 chunk 布景已实证 3600 拍会走出去,任务书"地面要够大"即此)。
//! 测试全程断言四足在本体区内。
//!
//! 度量口径(文档注释在辅助函数上):
//! - 机身高度 = 根连杆原点 y − 1.0;
//! - `up·y` = 机身局部 y 轴在世界系的 y 分量;
//! - 穿透 = 1.0 − 全部连杆全部形状的最低点(负数取 0);最低点由连杆位姿
//!   和 `BodyDef` 形状解析算出:球 = 中心 y − 半径;`capsule_y` = 两端球心
//!   较低者 − 半径;盒 = 8 个角点最低者。形状的 `offset` 先被连杆姿态旋转
//!   再加到连杆位置上;
//! - 沿航向平均速度 = `demos/agents/gait/src/main.rs:320` 的 `vfwd` 同款:
//!   根连杆线速度点乘航向单位向量 `fh = normalize((fwd.x, 0, fwd.z))`,
//!   其中 `fwd = rotation * (1, 0, 0)`(main.rs:282),逐拍累加取时间均值
//!   (main.rs:417 `fwd_speed_mean = vfwd_sum / vel_n`)。

use messaging::{InputSource, SimCommand};
use physics_sim::{BodyRegistry, ShardSim, ShardSimConfig};
use shard::SchedulerConfig;
use sim_api::log::CommandLog;
use sim_api::replay::replay_world_with_bodies;
use sim_api::session::{Session, SessionConfig};
use types::glam::{Quat, Vec3};
use types::{
    BlockId, ChunkPos, DimensionId, EntityId, EntityState, ShardId, ShardPos, Tick, WorldId,
};
use voxel::{Chunk, SimpleRegistry};

use app::controller::gait::QUAD_DOF;
use app::controller::{ControllerHost, ControllerKind, ScriptKind, TROT_SETTLE_TICKS};

const STONE: BlockId = BlockId(1);
/// 地面顶面(world y)。
const GROUND_TOP: f32 = 1.0;
/// 出生点(分片中心)。
const SPAWN: Vec3 = Vec3::new(64.0, 1.45, 64.0);
/// 铺满整个分片:8×8 chunk = 128×128 m,x/z ∈ [0, 128)。
const GROUND_CHUNKS: i32 = 8;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn session_cfg() -> SessionConfig {
    // inbox_capacity 512:B6 回放把 600 拍的日志(300 条)一次性直投进
    // 收件箱,默认 256 会反压 InboxFull。只影响路由反压,不影响物理。
    SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            router: shard::router::RouterConfig {
                inbox_capacity: 512,
                ..shard::router::RouterConfig::default()
            },
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    }
}

fn quad_asset() -> &'static str {
    include_str!("../../../assets/bodies/quad.body.toml")
}

fn registry() -> BodyRegistry {
    let mut bodies = BodyRegistry::new();
    bodies
        .insert_toml(quad_asset())
        .expect("quad.body.toml 合法");
    bodies
}

fn quad_sim() -> ShardSim {
    let cfg = ShardSimConfig::new(shard_id(0, 0));
    ShardSim::new_with_bodies(cfg, Box::new(SimpleRegistry::builtin()), registry())
}

/// 3×3 chunk 平板地形(层 world y = 0 全石,顶面 = 1.0)。
fn load_flat_ground(session: &mut Session) {
    for cx in 0..GROUND_CHUNKS {
        for cz in 0..GROUND_CHUNKS {
            let mut chunk = Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    chunk.set(x, 64, z, STONE);
                }
            }
            session.apply_chunk_load(shard_id(0, 0), ChunkPos::new(cx, cz), chunk);
        }
    }
}

/// quad 身体定义(穿透度量用形状)。
fn quad_def() -> types::BodyDef {
    registry()
        .get(&registry().hash_of("quad").expect("quad 已装"))
        .expect("哈希必在")
        .clone()
}

// ---- 度量辅助 ---------------------------------------------------------------

/// 单形状在连杆局部系里的最低点(按任务书口径:球 = 中心 y − 半径;
/// `capsule_y` = 两端球心较低者 − 半径;盒 = 8 角点最低者。`offset` 已被
/// 调用方旋转并平移到世界)。
fn shape_lowest_y(shape: &types::ShapeDef, pos: Vec3, rot: Quat) -> f32 {
    match *shape {
        types::ShapeDef::Box { half, .. } => {
            // 8 个角点(局部系),旋转 + 平移,取最低。
            let mut lowest = f32::INFINITY;
            for i in 0..8 {
                let corner = Vec3::new(
                    if i & 1 == 0 { -half[0] } else { half[0] },
                    if i & 2 == 0 { -half[1] } else { half[1] },
                    if i & 4 == 0 { -half[2] } else { half[2] },
                );
                lowest = lowest.min(pos.y + (rot * corner).y);
            }
            lowest
        }
        types::ShapeDef::Sphere { radius, .. } => pos.y - radius,
        types::ShapeDef::CapsuleY {
            half_height,
            radius,
            ..
        } => {
            // 两端球心(局部 ±half_height),旋转 + 平移,较低者 − 半径。
            let a = pos + rot * Vec3::new(0.0, half_height, 0.0);
            let b = pos + rot * Vec3::new(0.0, -half_height, 0.0);
            a.y.min(b.y) - radius
        }
        types::ShapeDef::ConvexHull { ref points, .. } => {
            let mut lowest = f32::INFINITY;
            for p in points {
                let wp = pos + rot * Vec3::new(p[0], p[1], p[2]);
                lowest = lowest.min(wp.y);
            }
            lowest
        }
    }
}

/// 一帧里全部连杆全部形状的最低点(world y)。
///
/// 穿透(任务书)= `1.0 − 最低点`(负数取 0);形状 `offset` 先被连杆姿态
/// 旋转再加到连杆位置上。
fn links_lowest_y(def: &types::BodyDef, states: &[EntityState]) -> f32 {
    let mut lowest = f32::INFINITY;
    for (link, st) in def.links.iter().zip(states) {
        for shape in &link.shapes {
            let offset = match *shape {
                types::ShapeDef::Box { offset, .. }
                | types::ShapeDef::Sphere { offset, .. }
                | types::ShapeDef::CapsuleY { offset, .. }
                | types::ShapeDef::ConvexHull { offset, .. } => offset,
            };
            let world_pos =
                st.translation + st.rotation * Vec3::new(offset[0], offset[1], offset[2]);
            lowest = lowest.min(shape_lowest_y(shape, world_pos, st.rotation));
        }
    }
    lowest
}

/// 穿透(m;地面顶面 1.0 减去最低点,负数取 0)。
fn penetration(def: &types::BodyDef, states: &[EntityState]) -> f32 {
    (GROUND_TOP - links_lowest_y(def, states)).max(0.0)
}

/// 沿航向的平均速度(m/s;`demos/agents/gait/src/main.rs` 的 `vfwd` 口径:
/// main.rs:282 `fwd = rotation * (1,0,0)`,main.rs:320
/// `vfwd = linvel · normalize((fwd.x, 0, fwd.z))`,main.rs:417 时间均值)。
fn vfwd(state: &EntityState) -> f32 {
    let fwd = state.rotation * Vec3::new(1.0, 0.0, 0.0);
    let fh = Vec3::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
    state.linvel.dot(fh)
}

/// `up·y`:机身局部 y 轴在世界系的 y 分量。
fn up_y(state: &EntityState) -> f32 {
    (state.rotation * Vec3::new(0.0, 1.0, 0.0)).y
}

/// 一段的累计度量。
struct Metrics {
    height_sum: f64,
    height_sq_sum: f64,
    n: u64,
    up_min: f32,
    pen_max: f32,
    vfwd_sum: f64,
    vel_n: u64,
    /// 起步位置(首次采样的根 xz)。
    start: Option<(f32, f32)>,
    /// 末次采样的根位置(完整,含 y)。
    last_root: Vec3,
    /// 末次采样的航向 f32 对(start fwd 的 z.atan2(x) 差,度)。
    heading_start: Option<f32>,
    heading_last: f32,
    /// 全程 xz 是否都在地面范围 [0, 48)(含余量 1 m)。
    out_of_ground: bool,
    /// 全程根位姿有限。
    all_finite: bool,
}

impl Metrics {
    fn new() -> Self {
        Self {
            height_sum: 0.0,
            height_sq_sum: 0.0,
            n: 0,
            up_min: f32::INFINITY,
            pen_max: 0.0,
            vfwd_sum: 0.0,
            vel_n: 0,
            start: None,
            last_root: Vec3::ZERO,
            heading_start: None,
            heading_last: 0.0,
            out_of_ground: false,
            all_finite: true,
        }
    }

    /// 采一拍(根状态 + 全连杆最低点穿透)。`ground_half` = 地面半宽
    /// (48;断言 xz ∈ [0+margin, 48−margin])。
    fn sample(&mut self, def: &types::BodyDef, states: &[EntityState], sample_vfwd: bool) {
        let root = &states[0];
        let y = f64::from(root.translation.y);
        self.height_sum += y;
        self.height_sq_sum += y * y;
        self.n += 1;
        self.up_min = self.up_min.min(up_y(root));
        self.pen_max = self.pen_max.max(penetration(def, states));
        if sample_vfwd {
            self.vfwd_sum += f64::from(vfwd(root));
            self.vel_n += 1;
        }
        if self.start.is_none() {
            self.start = Some((root.translation.x, root.translation.z));
            let fwd = root.rotation * Vec3::new(1.0, 0.0, 0.0);
            self.heading_start = Some(fwd.z.atan2(fwd.x));
        }
        self.last_root = root.translation;
        let fwd = root.rotation * Vec3::new(1.0, 0.0, 0.0);
        self.heading_last = fwd.z.atan2(fwd.x);
        // 全程逐拍都在地面/本体区内(xz ∈ [1, 127],留 1 m 余量)。
        if root.translation.x < 1.0
            || root.translation.x > 127.0
            || root.translation.z < 1.0
            || root.translation.z > 127.0
        {
            self.out_of_ground = true;
        }
        if !root.translation.is_finite() || !root.linvel.is_finite() || !root.rotation.is_finite() {
            self.all_finite = false;
        }
    }

    fn height_mean(&self) -> f64 {
        self.height_sum / self.n.max(1) as f64
    }

    fn height_std(&self) -> f64 {
        let mean = self.height_mean();
        let var = self.height_sq_sum / self.n.max(1) as f64 - mean * mean;
        var.max(0.0).sqrt()
    }

    fn vfwd_mean(&self) -> f64 {
        self.vfwd_sum / self.vel_n.max(1) as f64
    }

    fn net_displacement(&self) -> f32 {
        let (sx, sz) = self.start.expect("至少采过一拍");
        let dx = self.last_root.x - sx;
        let dz = self.last_root.z - sz;
        (dx * dx + dz * dz).sqrt()
    }

    fn heading_change_deg(&self) -> f32 {
        (self.heading_start.expect("至少采过一拍") - self.heading_last).to_degrees()
    }
}

/// 平地会话 + 四足,跑 `total_ticks` 拍的公共驱动:
/// - `host`:可挂控制者(None 档传空 host);每拍先 `host.drive(&mut session, now)`
///   再 `session.tick()`(任务书 §2.2 调用时机);
/// - 从 `sample_from` 拍开始采样;
/// - 返回 (metrics, 逐拍根位姿, void_removed 主体数, 日志)。`now` 从 0 计
///   (spawn 盖在拍 0,tick() 第 k 次执行拍 k−1;动作从拍 1 起生效,与 S1
///   模板一致)。
fn run_quad(
    total_ticks: u32,
    sample_from: u32,
    mut host: Option<&mut ControllerHost>,
    log: &mut CommandLog,
) -> (Metrics, Vec<EntityState>, u64) {
    let shard = shard_id(0, 0);
    let a = EntityId::from_parts(1, 1);
    let def = quad_def();

    let mut session = Session::new(session_cfg());
    session.register(quad_sim());
    load_flat_ground(&mut session);

    let hash = registry().hash_of("quad").expect("quad 已装");
    let entry = session
        .spawn_agent_from(
            InputSource::SCHEDULER,
            shard,
            a,
            hash,
            EntityState {
                translation: SPAWN,
                ..EntityState::default()
            },
        )
        .expect("盖章入日志");
    log.append(entry);
    // spawn 后先走一拍:owner 表在拍末由 outbox.spawned 登记(S1 同款)。
    session.tick();

    let mut metrics = Metrics::new();
    let mut void_removed_total = 0u64;
    let mut poses: Vec<EntityState> = Vec::new();
    let mut void_removed;
    for tick in 0..total_ticks {
        let now = Tick(u64::from(tick) + 1); // 已走过 spawn 后那一拍
        if let Some(host) = host.as_deref_mut() {
            host.drive(&mut session, now);
        }
        let summary = session.tick();
        void_removed_total += summary.telemetry.void_removals;
        let removed = session.take_void_removed_agents();
        void_removed = !removed.is_empty();
        if void_removed {
            void_removed_total += 1;
        }
        if tick + 1 >= sample_from {
            let states = session
                .agent_states()
                .into_iter()
                .find(|(id, _, _)| *id == a)
                .map(|(_, _, s)| s)
                .expect("四足全程在(移除判据单独断言)");
            if tick + 1 == sample_from {
                metrics = Metrics::new();
            }
            metrics.sample(&def, &states, true);
            if tick + 1 == total_ticks || (total_ticks - tick) <= 2 {
                poses.push(states[0]);
            }
        }
    }
    assert!(
        !metrics.out_of_ground,
        "四足走出地面/本体区范围(x/z ∉ [1, 127])"
    );
    assert!(metrics.all_finite, "出现非有限值");
    assert!(
        session.is_agent(a),
        "四足被移除(void_removed / 压垮 / 失控)"
    );
    session.shutdown();
    (metrics, poses, void_removed_total)
}

/// 干净世界 600 拍 trot(B2s / B6 共用)。
fn run_trot_600(sample_from: u32) -> (Metrics, CommandLog) {
    let mut log = CommandLog::new();
    let mut host = ControllerHost::default();
    host.attach(
        EntityId::from_parts(1, 1),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        2,
        Tick(0),
    )
    .expect("挂载合法");
    let (metrics, _poses, void_removed) = run_quad(600, sample_from, Some(&mut host), &mut log);
    assert_eq!(void_removed, 0, "没有任何移除");
    (metrics, log)
}

// ---- 门禁判据 ---------------------------------------------------------------

/// B1a:无控制者站立 3600 拍。第 600 拍之后:高度标准差 ≤ 0.005 m,
/// `up·y` ≥ 0.99。报入睡的拍号;入睡之后每拍根位姿逐位不变。
#[test]
fn b1a_stand_no_controller_3600() {
    let shard = shard_id(0, 0);
    let a = EntityId::from_parts(1, 1);
    let def = quad_def();
    let mut session = Session::new(session_cfg());
    session.register(quad_sim());
    load_flat_ground(&mut session);
    let hash = registry().hash_of("quad").expect("quad 已装");
    session
        .spawn_agent_from(
            InputSource::SCHEDULER,
            shard,
            a,
            hash,
            EntityState {
                translation: SPAWN,
                ..EntityState::default()
            },
        )
        .expect("盖章入日志");
    session.tick();

    // 采样:高度/倾角统计从拍 600 起;入睡拍 = 根位姿首次进入"之后每拍
    // 逐位不变"序列的拍(睡眠 ⇒ 物理不再改位姿)。窗口 40 拍(≥ 任何合理
    // 睡眠判定余量),防止短暂静止误判。
    const WINDOW: usize = 40;
    let mut frozen: Vec<EntityState> = Vec::new();
    let mut asleep_tick: Option<u32> = None;
    let mut metrics = Metrics::new();
    let mut prev_root: Option<EntityState> = None;
    for tick in 0..3600u32 {
        let summary = session.tick();
        let _ = summary;
        let removed = session.take_void_removed_agents();
        assert!(removed.is_empty(), "拍 {tick}:四足被虚空移除");
        let states = session
            .agent_states()
            .into_iter()
            .find(|(id, _, _)| *id == a)
            .map(|(_, _, s)| s)
            .expect("四足全程在");
        let root = states[0];
        if tick + 1 >= 600 {
            metrics.sample(&def, &states, false);
        }
        match prev_root {
            Some(prev) if prev == root => frozen.push(root),
            _ => frozen.clear(),
        }
        prev_root = Some(root);
        if asleep_tick.is_none() && frozen.len() >= WINDOW {
            asleep_tick = Some(tick + 1 - WINDOW as u32);
        }
    }
    assert!(session.is_agent(a), "四足被移除");
    session.shutdown();

    assert!(
        !metrics.out_of_ground,
        "四足走出地面/本体区范围(x/z ∉ [1, 127])"
    );
    assert!(metrics.all_finite, "出现非有限值");
    let std = metrics.height_std();
    let up_min = metrics.up_min;
    let asleep_report = match asleep_tick {
        Some(t) => format!("入睡拍 {t}"),
        None => "未入睡".to_string(),
    };
    // 如实回评用:打印实测数字。
    println!(
        "B1a: height_mean {:.4} height_std {:.6} up_min {:.4} pen_max {:.6} asleep {}",
        metrics.height_mean() - f64::from(GROUND_TOP),
        std,
        up_min,
        metrics.pen_max,
        asleep_report
    );
    assert!(std <= 0.005, "第 600 拍之后高度标准差 {std:.6} > 0.005 m");
    assert!(up_min >= 0.99, "第 600 拍之后 up·y 最小 {up_min} < 0.99");
}

/// N1:对照——不挂控制者走 600 拍,沿航向平均速度的绝对值 < 0.02 m/s
/// (B2 的速度不是布景自己给的)。
#[test]
fn n1_no_controller_walk_speed_near_zero() {
    let mut log = CommandLog::new();
    // 采样从拍 120 起(排出生下坠/站稳段;B1a 实测 160 拍入睡,入睡后
    // 逐位不动,前段动力学不影响"稳态下布景自己会不会走"的问题)。
    let (metrics, _poses, void_removed) = run_quad(600, 121, None, &mut log);
    assert_eq!(void_removed, 0, "对照线不应有移除");
    let v = metrics.vfwd_mean();
    println!(
        "N1: vfwd_mean {:.5} net {:.4}",
        v,
        metrics.net_displacement()
    );
    assert!(
        v.abs() < 0.02,
        "无控制者对照线沿航向平均速度 |{v:.5}| ≥ 0.02 m/s:布景自己会走,判据失效"
    );
}

/// B2s:B2a 的 600 拍短版(起步后 480 拍采样):`up·y` ≥ 0.7、穿透 ≤ 0.05、
/// 平均速度 ≥ 0.1。
#[test]
fn b2s_trot_600_gate() {
    // 起步 = TROT_SETTLE_TICKS(120 拍)之后;采样 120..600(起步后 480 拍)。
    let (metrics, _log) = run_trot_600(TROT_SETTLE_TICKS as u32 + 1);
    let v = metrics.vfwd_mean();
    println!(
        "B2s: up_min {:.4} pen_max {:.6} vfwd_mean {:.5} net {:.4} heading {:.1}°",
        metrics.up_min,
        metrics.pen_max,
        v,
        metrics.net_displacement(),
        metrics.heading_change_deg()
    );
    assert!(metrics.up_min >= 0.7, "up·y 最小 {} < 0.7", metrics.up_min);
    assert!(
        metrics.pen_max <= 0.05,
        "最大穿透 {} > 0.05 m",
        metrics.pen_max
    );
    assert!(v >= 0.1, "沿航向平均速度 {v:.5} < 0.1 m/s(起步后采样)");
}

/// B6:`QuadTrot` 600 拍,从拍 0 快照 + 日志重放,与原时间线末态快照逐位
/// 相同;日志里的 `AgentJointTargets` 条数 = 300。
#[test]
fn b6_trot_600_replay_bitwise() {
    let shard = shard_id(0, 0);
    let a = EntityId::from_parts(1, 1);
    let def = quad_def();

    // 原线:600 拍 trot,逐拍采根位姿(用 ac 结构断言回放对照)。
    let mut log = CommandLog::new();
    let mut host = ControllerHost::default();
    host.attach(
        a,
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        2,
        Tick(0),
    )
    .expect("挂载合法");

    let mut session = Session::new(session_cfg());
    session.register(quad_sim());
    load_flat_ground(&mut session);
    let snapshot = session.world_snapshot();
    let hash = registry().hash_of("quad").expect("quad 已装");
    let entry = session
        .spawn_agent_from(
            InputSource::SCHEDULER,
            shard,
            a,
            hash,
            EntityState {
                translation: SPAWN,
                ..EntityState::default()
            },
        )
        .expect("盖章入日志");
    log.append(entry);
    session.tick();
    let mut original_tail: Vec<EntityState> = Vec::new();
    for tick in 0..600u32 {
        let now = Tick(u64::from(tick) + 1);
        // B6 显式经 send_agent_joint_targets_to(条目自己 append 进日志;
        // 与 drive 等价——drive 就是逐条调它,但 drive 吞掉返回的 LogEntry,
        // 日志由调用方维护,这里要核对条数)。
        for (entity, targets) in host.actions_for_tick(now) {
            let entry = session
                .send_agent_joint_targets_to(InputSource::SCHEDULER, entity, targets)
                .expect("盖章入日志");
            log.append(entry);
        }
        session.tick();
        assert!(
            session.take_void_removed_agents().is_empty(),
            "拍 {tick} 移除"
        );
        if tick >= 598 {
            let states = session
                .agent_states()
                .into_iter()
                .find(|(id, _, _)| *id == a)
                .map(|(_, _, s)| s)
                .expect("四足在");
            original_tail.push(states[0]);
        }
    }
    assert!(session.is_agent(a));
    session.shutdown();

    // 日志自检:SpawnAgent 1 + AgentJointTargets 300(600 拍每 2 拍一条)。
    let joint_entries: Vec<_> = log
        .entries()
        .iter()
        .filter(|e| matches!(e.command, SimCommand::AgentJointTargets { .. }))
        .collect();
    assert_eq!(joint_entries.len(), 300, "AgentJointTargets 条数应为 300");
    let spawn_entries: Vec<_> = log
        .entries()
        .iter()
        .filter(|e| matches!(e.command, SimCommand::SpawnAgent { .. }))
        .collect();
    assert_eq!(spawn_entries.len(), 1, "日志恰有一条 SpawnAgent");

    // 回放:拍 0 快照 + 全日志,末态根位姿与原线逐位相同。
    let new_registry = || -> Box<dyn voxel::BlockRegistry> { Box::new(SimpleRegistry::builtin()) };
    let new_bodies = registry;
    let mut replayed =
        replay_world_with_bodies(&snapshot, &log, session_cfg(), new_registry, new_bodies)
            .expect("世界快照 + 日志恢复");
    // 原线共 601 拍(spawn 后 1 拍 + 600 主循环),回放对齐 601 次。
    for _ in 0..601u32 {
        replayed.tick();
    }
    let replay_tail: EntityState = replayed
        .agent_states()
        .into_iter()
        .find(|(id, _, _)| *id == a)
        .map(|(_, _, s)| s[0])
        .expect("回放线四足在");
    assert!(replayed.is_agent(a), "回放线四足在");
    replayed.shutdown();
    assert_eq!(
        original_tail.last().expect("原线采过末拍"),
        &replay_tail,
        "末态根连杆状态与原时间线逐位不同"
    );
    let _ = &def;
}

// ---- release 档(#[ignore])-------------------------------------------------

/// B1b:`QuadStand` 站立 3600 拍,同样的高度和倾角判据。另报:是否入睡、
/// 水平滑移速度(只报数,不设门槛);并观察发同样的目标会不会唤醒已入睡
/// 的四足(照实写)。
#[test]
#[ignore = "release 档:3600 拍站立,任务书 §3 B1b"]
fn b1b_stand_script_3600() {
    let shard = shard_id(0, 0);
    let a = EntityId::from_parts(1, 1);
    let def = quad_def();
    let mut session = Session::new(session_cfg());
    session.register(quad_sim());
    load_flat_ground(&mut session);
    let hash = registry().hash_of("quad").expect("quad 已装");
    session
        .spawn_agent_from(
            InputSource::SCHEDULER,
            shard,
            a,
            hash,
            EntityState {
                translation: SPAWN,
                ..EntityState::default()
            },
        )
        .expect("盖章入日志");
    session.tick();

    let mut host = ControllerHost::default();
    host.attach(
        a,
        ControllerKind::Script(ScriptKind::QuadStand),
        QUAD_DOF,
        2,
        Tick(0),
    )
    .expect("挂载合法");

    let mut metrics = Metrics::new();
    let mut prev_root: Option<EntityState> = None;
    let mut frozen: Vec<EntityState> = Vec::new();
    let mut asleep_tick: Option<u32> = None;
    const WINDOW: usize = 40;
    // 唤醒观察:入睡之后统计"目标继续发、位姿仍不变"的拍数,以及"发目标
    // 拍之后位姿变化"的事件数(照实报)。
    let mut wake_ticks_after_asleep = 0u64;
    let mut root_changed_after_asleep = 0u64;
    for tick in 0..3600u32 {
        let now = Tick(u64::from(tick) + 1);
        host.drive(&mut session, now);
        session.tick();
        assert!(
            session.take_void_removed_agents().is_empty(),
            "拍 {tick} 移除"
        );
        let states = session
            .agent_states()
            .into_iter()
            .find(|(id, _, _)| *id == a)
            .map(|(_, _, s)| s)
            .expect("四足全程在");
        let root = states[0];
        if tick + 1 >= 600 {
            metrics.sample(&def, &states, true);
        }
        match prev_root {
            Some(prev) if prev == root => frozen.push(root),
            _ => frozen.clear(),
        }
        prev_root = Some(root);
        if asleep_tick.is_none() && frozen.len() >= WINDOW {
            asleep_tick = Some(tick + 1 - WINDOW as u32);
        }
        if asleep_tick.is_some() {
            // drive 每 2 拍发一次站立目标(period 2):入睡后照发的拍数。
            if (tick + 1) % 2 == 0 {
                wake_ticks_after_asleep += 1;
            }
            // 入睡后逐位不变窗被打断(本拍 root ≠ 上一拍 root)= 位姿变化
            // 事件;计数说明"发同样的目标把入睡的四足唤醒了没有"。
            if frozen.is_empty() {
                root_changed_after_asleep += 1;
            }
        }
    }
    assert!(session.is_agent(a), "四足被移除");
    session.shutdown();

    assert!(!metrics.out_of_ground, "走出地面");
    assert!(metrics.all_finite, "出现非有限值");
    let std = metrics.height_std();
    println!(
        "B1b: height_mean {:.4} height_std {:.6} up_min {:.4} pen_max {:.6} \
         asleep {} slide_vfwd_mean {:.5} net {:.4} heading {:.1}° \
         drive_ticks_after_asleep {} root_changed_after_asleep {}",
        metrics.height_mean() - f64::from(GROUND_TOP),
        std,
        metrics.up_min,
        metrics.pen_max,
        match asleep_tick {
            Some(t) => format!("拍 {t}"),
            None => "未入睡".to_string(),
        },
        metrics.vfwd_mean(),
        metrics.net_displacement(),
        metrics.heading_change_deg(),
        wake_ticks_after_asleep,
        root_changed_after_asleep
    );
    assert!(std <= 0.005, "高度标准差 {std:.6} > 0.005 m");
    assert!(
        metrics.up_min >= 0.99,
        "up·y 最小 {} < 0.99",
        metrics.up_min
    );
}

/// B2a:`QuadTrot` 3600 拍,干净世界:全程有限、`up·y` ≥ 0.7、穿透 ≤ 0.05、
/// 沿航向平均速度 ≥ 0.1 m/s(从起步后算)、没有任何移除。
#[test]
#[ignore = "release 档:3600 拍 trot,任务书 §3 B2a"]
fn b2a_trot_3600_clean() {
    let mut log = CommandLog::new();
    let mut host = ControllerHost::default();
    host.attach(
        EntityId::from_parts(1, 1),
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        2,
        Tick(0),
    )
    .expect("挂载合法");
    let (metrics, _poses, void_removed) = run_quad(
        3600,
        TROT_SETTLE_TICKS as u32 + 1,
        Some(&mut host),
        &mut log,
    );
    assert_eq!(
        void_removed, 0,
        "没有任何移除(压垮/失控/掉出世界计数均为 0)"
    );
    let v = metrics.vfwd_mean();
    println!(
        "B2a: up_min {:.4} pen_max {:.6} vfwd_mean {:.5} net {:.4} heading {:.1}° \
         height_mean {:.4}",
        metrics.up_min,
        metrics.pen_max,
        v,
        metrics.net_displacement(),
        metrics.heading_change_deg(),
        metrics.height_mean() - f64::from(GROUND_TOP)
    );
    assert!(metrics.all_finite, "全程有限");
    assert!(metrics.up_min >= 0.7, "up·y 最小 {} < 0.7", metrics.up_min);
    assert!(
        metrics.pen_max <= 0.05,
        "最大穿透 {} > 0.05 m",
        metrics.pen_max
    );
    assert!(v >= 0.1, "沿航向平均速度 {v:.5} < 0.1 m/s");
}

/// B2b:同 B2a,但先在 10 m 外生成 16 个动态方块、等它们全部睡着,再生成
/// 四足(打乱连杆顺序)。同样的判据。自检:末态根位置与 B2a 不逐位相同
/// (证明顺序确实被打乱了)。
#[test]
#[ignore = "release 档:3600 拍 trot + 16 方块,任务书 §3 B2b"]
fn b2b_trot_3600_shuffled_order() {
    // 自检的对照线:同代码同配置、无方块的干净世界(= B2a)末态根位置。
    let (clean_metrics, _p, clean_void) = {
        let mut log = CommandLog::new();
        let mut host = ControllerHost::default();
        host.attach(
            EntityId::from_parts(1, 1),
            ControllerKind::Script(ScriptKind::QuadTrot),
            QUAD_DOF,
            2,
            Tick(0),
        )
        .expect("挂载合法");
        run_quad(
            3600,
            TROT_SETTLE_TICKS as u32 + 1,
            Some(&mut host),
            &mut log,
        )
    };
    assert_eq!(clean_void, 0, "对照线不应有移除");

    let shard = shard_id(0, 0);
    let a = EntityId::from_parts(1, 1);
    let def = quad_def();
    let mut session = Session::new(session_cfg());
    session.register(quad_sim());
    load_flat_ground(&mut session);

    // 16 个动态方块:10 m 外(+x),4×4 排布,间隔 1.5 m,放在地面顶面上。
    // MarkActor(actor = true)豁免睡眠兜底凝固(A02b 同款手法):§13 连杆
    // 顺序的打乱要求"主体之前插入的会睡着的刚体"在主体生成时**仍然存在**
    // ——首版布景没豁免凝固,16 个方块入睡即被凝固 despawn,四足生成时
    // 世界已干净,顺序没被打乱(B2a ≈ B2b,自检要抓的正是这个)。
    let mut block_ids = Vec::new();
    for i in 0..16u32 {
        let id = EntityId::from_parts(100 + i, 1);
        let state = EntityState {
            translation: Vec3::new(
                SPAWN.x + 10.0 + (i % 4) as f32 * 1.5,
                1.5,
                SPAWN.z + (i / 4) as f32 * 1.5,
            ),
            ..EntityState::default()
        };
        let desc = types::EntityDesc {
            half_size: 0.5,
            block: STONE,
            shape: None,
        };
        session
            .send(
                shard,
                SimCommand::SpawnDynamic {
                    entity: id,
                    state,
                    desc,
                },
            )
            .expect("盖章入日志");
        // 同拍 MarkActor:canonical 序 SpawnDynamic(1) < MarkActor(5),
        // 同拍先 spawn 后标记,触及的实体已存在。
        session
            .send(
                shard,
                SimCommand::MarkActor {
                    entity: id,
                    actor: true,
                },
            )
            .expect("盖章入日志");
        block_ids.push(id);
    }
    // 等全部睡着:Session 层没有 awake 读口,用结构证据——actor 豁免凝固,
    // 方块必须仍在(entity_state 为 Some,变 None = 凝固桥吃了它 = 布景
    // 错误,自检必败)且速度恰 0。连续 60 拍满足视为全睡(超限 600 拍
    // panic,写明在等什么)。
    let mut still_n = 0u32;
    let mut waited = 0u32;
    while still_n < 60 {
        session.tick();
        waited += 1;
        assert!(
            waited <= 600,
            "等 16 个方块睡着超 600 拍(在等全部方块速度连续 60 拍为 0)"
        );
        let all_still = block_ids
            .iter()
            .all(|&id| match session.entity_state(shard, id) {
                None => panic!("方块 {id:?} 被凝固 despawn:MarkActor 豁免未生效,布景错误"),
                Some(s) => s.linvel == Vec3::ZERO && s.angvel == Vec3::ZERO,
            });
        if all_still {
            still_n += 1;
        } else {
            still_n = 0;
        }
    }
    // 顺序打乱自检放在末态对照(与 B2a 比)。

    let hash = registry().hash_of("quad").expect("quad 已装");
    session
        .spawn_agent_from(
            InputSource::SCHEDULER,
            shard,
            a,
            hash,
            EntityState {
                translation: SPAWN,
                ..EntityState::default()
            },
        )
        .expect("盖章入日志");
    session.tick();

    let mut host = ControllerHost::default();
    host.attach(
        a,
        ControllerKind::Script(ScriptKind::QuadTrot),
        QUAD_DOF,
        2,
        Tick(0),
    )
    .expect("挂载合法");

    let mut metrics = Metrics::new();
    let mut void_removed_total = 0u64;
    let mut first_sample = true;
    for tick in 0..3600u32 {
        let now = Tick(u64::from(tick) + 1);
        host.drive(&mut session, now);
        let summary = session.tick();
        void_removed_total += summary.telemetry.void_removals;
        assert!(
            session.take_void_removed_agents().is_empty(),
            "拍 {tick} 四足被移除"
        );
        if tick + 1 > TROT_SETTLE_TICKS as u32 {
            let states = session
                .agent_states()
                .into_iter()
                .find(|(id, _, _)| *id == a)
                .map(|(_, _, s)| s)
                .expect("四足全程在");
            if first_sample {
                metrics = Metrics::new();
                first_sample = false;
            }
            metrics.sample(&def, &states, true);
        }
    }
    assert!(session.is_agent(a), "四足被移除");
    let final_root = metrics.last_root;
    session.shutdown();

    assert_eq!(void_removed_total, 0, "没有任何移除");
    assert!(!metrics.out_of_ground, "走出地面");
    assert!(metrics.all_finite, "出现非有限值");
    let v = metrics.vfwd_mean();
    println!(
        "B2b: up_min {:.4} pen_max {:.6} vfwd_mean {:.5} net {:.4} heading {:.1}° \
         final_root ({:.4},{:.4},{:.4}) waited_for_blocks {}",
        metrics.up_min,
        metrics.pen_max,
        v,
        metrics.net_displacement(),
        metrics.heading_change_deg(),
        final_root.x,
        final_root.y,
        final_root.z,
        waited
    );
    assert!(metrics.up_min >= 0.7, "up·y 最小 {} < 0.7", metrics.up_min);
    assert!(
        metrics.pen_max <= 0.05,
        "最大穿透 {} > 0.05 m",
        metrics.pen_max
    );
    assert!(v >= 0.1, "沿航向平均速度 {v:.5} < 0.1 m/s");
    // 顺序打乱自检(任务书 §3 B2b):末态根位置与干净世界(同代码 B2a)
    // 不逐位相同,证明 16 个先睡的刚体确实打乱了连杆顺序。
    assert_ne!(
        final_root, clean_metrics.last_root,
        "末态根位置与干净世界逐位相同:连杆顺序没有被打破(自检失败)"
    );
    println!(
        "B2b 对照: clean final_root ({:.4},{:.4},{:.4}) vfwd {:.5}",
        clean_metrics.last_root.x,
        clean_metrics.last_root.y,
        clean_metrics.last_root.z,
        clean_metrics.vfwd_mean()
    );
}
