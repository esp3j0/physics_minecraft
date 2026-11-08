//! REQ-012 A00-4 demo `capsule`(2026-11-07,R1 返修)。
//!
//! 问题(放行原文):玩家的单连杆胶囊能不能走得稳、跳得准?行走控制器(§2.2)
//! 的参数取多少?胶囊被砸、站在方块堆上、走过 chunk 接缝时会不会出问题?
//!
//! 统一口径:60 Hz、生产求解参数(只覆写 dt)、不开子步、k = 1/100
//! (胶囊质量 0.8 = 80 kg × 1/100,方块质量 1)。每一列都是测量值。
//!
//! 身体(放行):总高 1.8 m、半径 0.3 m 的 y 向胶囊(`capsule_y(0.6, 0.3)`,
//! 2·(0.6+0.3) = 1.8),质量 0.8,`lock_rotations()`,朝向按意图 yaw 直接
//! `set_rotation`(只在变化时写,允许入睡)。摩擦是矩阵参数,与地面按
//! **Min 规则**合成(R1/CR1:胶囊侧 `friction_combine_rule(Min)` 优先于
//! 地面默认 Average,合成有效值 = min(μ, 对方)),restitution 0
//! (放行未列,取 0,README 写明)。
//!
//! 控制器(§2.2,只用力和冲量,不直接设速度):
//! - 水平力 `F = clamp(m·(v_target − v_h)/τ, F_max)`;空中乘 `air`;
//! - 跳跃冲量只在接地时生效:先把竖直速度清零,再加 `m·sqrt(2 g h_jump)`;
//! - 控制周期 2 拍(30 Hz,D10 默认);rapier 0.36.1 的 `add_force`
//!   **不自动清零**(持续生效到 `reset_forces`)→ 每拍先 `reset_forces`
//!   再施加本次;`|F| < 0.01 N` 不施加(允许入睡,站立判据依赖);
//! - 接地判定两种都实现并在矩阵比较:射线(胶囊底部向下,行程 ground_eps)
//!   与球投射(半径 CAP_R − 0.02 的球,从下半球球心上方 5 mm 下扫
//!   ground_eps + 0.02 + 0.005,R1/CR2);命中面法线 y ≥ 0.7 才算接地。
//!
//! 接缝修正钩子(R1/CR3):胶囊碰撞体挂 `MODIFY_SOLVER_CONTACTS`,
//! 每拍 `step_with_events` 步进;钩子把"推测接触打在下一个 chunk 碰撞体
//! 顶边"的斜法线摆成竖直(条件:法线 |n.y| ∈ [0.7, 0.99999)、对固定
//! 刚体、全部接触点在地形一侧的 y 都在胶囊底部 ±3 cm 内),dist 改成
//! 竖直间隙。`edgefix=off` 只用于接缝对照行。
//!
//! 场景(README 有逐条判据):平地行走 / 站立 / 台阶(d = 胶囊表面到台阶面
//! 的间隙,R1/CR5) / 空中连按跳 / 接缝(三档速度 × 8 航向 × fix 开关,
//! R1/CR7) / 方块堆+推方块 / 被砸 / 重物(无防护 + V_GUARD=200) /
//! 连杆顺序对照 / det。T1 域扩到装下全部航向全速度完整路径 + 16 m 余量
//! (R1/CR4),`end_reason` 区分 `out_of_domain` 与 `fell`。
//!
//! 流程:`sweep`(平地 648 全组合 × 3 速度 + 1 格台阶 h_jump 扫描,机械规则
//! 选 3 套候选写 `out/cands.txt`)→ `matrix`(候选跑全部场景)→ `det`
//! (全矩阵双跑逐行哈希)。提交件 = `*_submitted.txt`(手动拷贝,不覆盖)。

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::Instant;

use physics_sim::collider_build::{greedy_colliders, BoxCollider, NeighborChunks};
use quad::{production_params, yaw_quat, GROUND_FRICTION, GROUND_RESTITUTION};
use rapier3d::dynamics::{CoefficientCombineRule, RigidBodyBuilder, RigidBodyHandle};
use rapier3d::geometry::{ColliderBuilder, ColliderHandle, PairContacts, Ray, SharedShape};
use rapier3d::math::{Pose, Vector};
use rapier3d::parry::query::ShapeCastOptions;
use rapier3d::pipeline::{
    ActiveHooks, ContactModificationContext, PhysicsHooks, PhysicsWorld, QueryFilter,
};
use types::{ChunkPos, WorldSeed};
use voxel::{BlockRegistry as _, WorldGenerator as _};
use voxel::{Chunk, SimpleRegistry, Superflat};

// ── 常量 ────────────────────────────────────────────────────────────────

/// 重力(与生产一致,`world.gravity = -9.81`)。
const GRAVITY: f32 = 9.81;
/// 胶囊圆柱段半高:2·(0.6 + 0.3) = 1.8 m 总高。
const HALF_H: f32 = 0.6;
/// 胶囊半径。
const CAP_R: f32 = 0.3;
/// 胶囊质量(80 kg × 1/100)。
const CAP_MASS: f32 = 0.8;
/// T0 大平面顶面(同 gait/mass:fixed cuboid 顶面 = 0)。
const GROUND_TOP: f32 = 0.0;
/// T1 体素地面顶面(Superflat classic,同 terrain/migrate:世界 y = −60)。
const GROUND_TOP_T1: f32 = -60.0;
/// T1 域(R1/CR4:扩到装下全部航向 × 全速度完整路径含制动 + ≥16 m 余量)。
/// 最远路径:5.6 m/s × 1080 拍意图 = 100.8 m + 制动 ~1 m。8 航向各分量
/// 最大 ~101 m → 出生 ±101 m 再 +16 m 余量 = 世界 x ∈ [−120, 280]、
/// z ∈ [−120, 120]。chunk 16 m:z −8..=8(z −128..144)、x −8..=18
/// (x −128..304)。
const CHUNK_X_LO: i32 = -8;
const CHUNK_X_HI: i32 = 19;
const CHUNK_Z_LO: i32 = -8;
const CHUNK_Z_HI: i32 = 9;
/// T1 出生点:x=160 = 16×10、z=0 都是 chunk 边界——出生点正好是 4-chunk 角,
/// h0/h180 沿 z=0 缝走、h90/h270 沿 x=160 缝走(放行"包括正好沿接缝走")。
const T1_SPAWN_X: f32 = 160.0;
/// 台阶面 x(块体 x ∈ [6, 14],深 8 m:4.3 m/s 跑跳滞空 ~0.73 s 前进 ~3.1 m,
/// 落点必须在台上)。
const STEP_FACE_X: f32 = 6.0;
/// 台阶成功判定的 x 上界(台深 8:面 6 → 尾 14)。
const STEP_TAIL_X: f32 = 14.0;
/// 接缝场景三档速度(R1/CR7)。
const SEAM_SPEEDS: [f32; 3] = [1.5, 4.3, 5.6];
/// 接缝/行走场景默认速度(行走/顺序场景用中档)。
const SEAM_SPEED: f32 = 4.3;
/// 球投射(R1/CR2):半径 = CAP_R − 0.02,起点 = 下半球球心上方 5 mm,
/// 行程 = ground_eps + 0.02 + 0.005。
const BALL_CAST_R: f32 = CAP_R - 0.02;
/// 接缝钩子(R1/CR3):接触点在地形一侧 y 与胶囊底部差 ≤ 此值才算共面缝。
const EDGE_BAND: f32 = 0.03;
/// 卡住判定:连续 30 拍水平速度 < 目标的 50%。
const STUCK_TICKS: u32 = 30;
/// 制动停止阈值。
const STOP_SPEED: f32 = 0.05;
/// 重物防护阈值(放行 V_GUARD = 200)。
const V_GUARD: f32 = 200.0;
/// 行走时间表(全矩阵):0..60 落地安定、60..1140 意图、300..1139 稳态窗、
/// 1140 意图清零(控制器制动)、总 1500。
const FULL_RELEASE: usize = 1140;
const FULL_TOTAL: usize = 1500;
const STEADY0: usize = 300;
/// 扫描版行走(短):意图 60..780、稳态 300..779、总 900。
const SWEEP_RELEASE: usize = 780;
const SWEEP_TOTAL: usize = 900;

/// 扫描网格(放行)。
const TAUS: [f32; 3] = [0.05, 0.1, 0.2];
const FMAXS: [f32; 3] = [2.0, 4.0, 8.0];
const MUS: [f32; 3] = [0.0, 0.3, 0.6];
const AIRS: [f32; 2] = [0.0, 0.2];
const HJUMPS: [f32; 3] = [1.15, 1.25, 1.4];
const GES: [f32; 2] = [0.02, 0.05];
/// 行走扫描速度(判据 1 的三档)。
const WALK_SPEEDS: [f32; 3] = [1.5, 4.3, 5.6];
/// 被砸落点网格(25 点,同 mass 的 5×5 口径;角点 (±0.3,±0.3) 距轴 0.424 >
/// 半径 0.3,方块落在旁边,位移如实报 ~0)。
const HIT_GRID: [f32; 5] = [-0.3, -0.15, 0.0, 0.15, 0.3];
/// 重物落点(9 点,同 mass M7)。
const HEAVY_GRID: [f32; 3] = [-0.3, 0.0, 0.3];
/// 高度微扰(mass 的教训:撞击类必须扫落点并加微扰)。
const D_GRID: [f32; 5] = [0.0, 0.017, 0.034, 0.051, 0.068];
/// 台阶按跳距离。
const STEP_D: [f32; 3] = [0.3, 0.6, 1.0];
/// 小幅位置微扰表(10 次;确定性序列,非随机)。
const PERTURB: [(f32, f32); 10] = [
    (0.0, 0.0),
    (0.02, 0.01),
    (-0.02, -0.01),
    (0.01, -0.02),
    (-0.01, 0.02),
    (0.03, 0.02),
    (-0.03, -0.02),
    (0.015, -0.015),
    (-0.015, 0.015),
    (0.025, -0.01),
];

// ── 参数与场景 ──────────────────────────────────────────────────────────

/// 一套控制器参数(候选 = 全 7 项)。
#[derive(Clone, Copy, PartialEq)]
struct Params {
    /// 速度误差增益时间常数(s)。
    tau: f32,
    /// 力上限倍数(× m·g)。
    fmax_mul: f32,
    /// 胶囊摩擦系数(与对方按 Min 规则合成,有效值 = min(μ, 对方))。
    mu: f32,
    /// 空中力系数。
    air: f32,
    /// 目标起跳高度(m)。
    h_jump: f32,
    /// 接地判定距离(m)。
    ground_eps: f32,
    /// true = 球投射,false = 射线。
    shape_cast: bool,
}

/// R1/CR7:接缝场景的 edgefix 开关(true = 钩子开,false = 对照行)。
#[derive(Clone, Copy, PartialEq)]
enum EdgeFix {
    Fix1,
    Fix0,
}

#[derive(Clone, Copy, PartialEq)]
enum Scene {
    /// 场景 1:平地行走(sweep_a = 扫描短版,只跑 h0)。
    Walk {
        t1: bool,
        speed: f32,
        heading: usize,
        sweep_a: bool,
    },
    /// 场景 2:站立 3600 拍(零意图 = 控制器目标速度 0,主动保持)。
    Stand { t1: bool },
    /// 场景 3:台阶(height 1|2;run = 跑动起跳;d = 胶囊表面到台阶面的
    /// 间隙(R1/CR5);trial = 微扰号)。
    Step {
        height: i32,
        run: bool,
        d: f32,
        trial: usize,
    },
    /// 场景 4:空中连按跳(repeat)/ 单次跳对照。
    AirJump { repeat: bool },
    /// 场景 5:接缝(R1/CR7:出生 (160,0) = 4-chunk 角,三档速度 × 8 航向
    /// × edgefix 开/关;fix0 = 对照行)。
    Seam {
        speed: f32,
        heading: usize,
        fix: EdgeFix,
    },
    /// 场景 6:方块堆(layers 1|2,已睡着,自动连跳走过去)。
    Pile { layers: i32 },
    /// 场景 6b:推一块静止(睡着)单格方块,只报位移。
    Push,
    /// 场景 7:被 1 格方块砸(3 m,25 落点 × 5 高度微扰)。
    Hit { ox: f32, oz: f32, d: f32 },
    /// 场景 8:重物(half h ∈ {1,2,4} → 质量 8/64/512,5 m,9 落点 × 5 微扰;
    /// v_guard Some = 防护重跑)。
    Heavy { h: f32, ox: f32, oz: f32, d: f32 },
    /// 场景 9:连杆顺序对照(主体之前插入 16 个会睡着的刚体;ball = migrate
    /// 式 1 cm 球、醒着落地后入睡;否则 = 放行字面:无碰撞体、出生即睡)。
    Order { ball: bool, heading: usize },
}

impl Scene {
    /// 跑在 T1 体素域上的场景(域断言用)。
    fn t1_domain(&self) -> bool {
        matches!(
            self,
            Scene::Walk { t1: true, .. } | Scene::Seam { .. } | Scene::Stand { t1: true }
        )
    }
}

struct Cfg {
    name: String,
    scene: Scene,
    p: Params,
    v_guard: Option<f32>,
}

fn total_ticks(cfg: &Cfg) -> usize {
    match cfg.scene {
        Scene::Walk { sweep_a, .. } => {
            if sweep_a {
                SWEEP_TOTAL
            } else {
                FULL_TOTAL
            }
        }
        Scene::Seam { .. } | Scene::Order { .. } => FULL_TOTAL,
        Scene::Stand { .. } => 3600,
        Scene::Step { .. } => 480,
        Scene::AirJump { .. } => 720,
        Scene::Pile { .. } => 1200,
        Scene::Push => 660,
        Scene::Hit { .. } | Scene::Heavy { .. } => 360,
    }
}

fn release_tick(cfg: &Cfg) -> usize {
    match cfg.scene {
        Scene::Walk { sweep_a, .. } => {
            if sweep_a {
                SWEEP_RELEASE
            } else {
                FULL_RELEASE
            }
        }
        Scene::Seam { .. } | Scene::Order { .. } => FULL_RELEASE,
        _ => usize::MAX,
    }
}

fn steady0(cfg: &Cfg) -> usize {
    match cfg.scene {
        Scene::Walk { .. } | Scene::Seam { .. } | Scene::Order { .. } => STEADY0,
        _ => usize::MAX,
    }
}

fn heading_rad(idx: usize) -> f32 {
    idx as f32 * std::f32::consts::FRAC_PI_4
}

/// 航向 → 水平方向(terrain/migrate 同款:`from_rotation_y(θ)` 把 +x 转向 −z)。
fn heading_dir(idx: usize) -> Vector {
    let th = heading_rad(idx);
    Vector::new(th.cos(), 0.0, -th.sin())
}

// ── 名字编码(完整名字段筛行;0.3 → "0p3",−0.15 → "-0p15") ─────────────

/// 两位小数裁尾(参数用):0.05→"0p05"、1.4→"1p4"、4→"4"、0→"0"。
fn fp(v: f32) -> String {
    let s = format!("{:.2}", v);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    s.replace('.', "p")
}

/// 一位小数(台阶距离):0.3→"0p3"、1.0→"1p0"。
fn fd(v: f32) -> String {
    format!("{:.1}", v).replace('.', "p")
}

/// 三位小数裁尾(落点/微扰):−0.15→"-0p15"、0→"0"、0.017→"0p017"。
fn f3(v: f32) -> String {
    let s = format!("{:.3}", v);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    s.replace('.', "p")
}

impl Params {
    /// 扫描行共用的元组段(不含 h_jump)。
    fn tuple_tag(self) -> String {
        format!(
            "tau{}_fmax{}_mu{}_air{}_ge{}_{}",
            fp(self.tau),
            fp(self.fmax_mul),
            fp(self.mu),
            fp(self.air),
            fp(self.ground_eps),
            if self.shape_cast { "shape" } else { "ray" },
        )
    }
}

// ── 生成 ────────────────────────────────────────────────────────────────

fn spawn_capsule(
    world: &mut PhysicsWorld,
    pos: Vector,
    p: &Params,
) -> (RigidBodyHandle, ColliderHandle) {
    let (body, collider) = world.insert(
        RigidBodyBuilder::dynamic().lock_rotations().pose(Pose {
            translation: pos,
            ..Pose::IDENTITY
        }),
        ColliderBuilder::capsule_y(HALF_H, CAP_R)
            .mass(CAP_MASS)
            .friction(p.mu)
            // R1/CR1:Min 优先于地面默认 Average → 合成 μ_eff = min(μ, 0.6)。
            .friction_combine_rule(CoefficientCombineRule::Min)
            // R1/CR3:接缝修正钩子挂在胶囊碰撞体上(edgefix=off 时世界照跑,
            // 钩子在步进侧判断开关)。
            .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS)
            .restitution(0.0),
    );
    (body, collider)
}

/// 生产材质的单格方块(density 1、friction 0.6、restitution 0.1,质量 1)。
/// `side` = 边长;`asleep` = 出生即睡(方块堆/推方块"已经睡着")。
fn spawn_block(
    world: &mut PhysicsWorld,
    pos: Vector,
    asleep: bool,
    side: f32,
) -> (RigidBodyHandle, ColliderHandle) {
    let mut b = RigidBodyBuilder::dynamic().pose(Pose {
        translation: pos,
        ..Pose::IDENTITY
    });
    if asleep {
        b = b.sleeping(true);
    }
    world.insert(
        b,
        ColliderBuilder::cuboid(side * 0.5, side * 0.5, side * 0.5)
            .density(1.0)
            .friction(0.6)
            .restitution(0.1),
    )
}

// ── 接缝修正钩子(R1/CR3) ────────────────────────────────────────────────

/// 把"推测接触打在下一个 chunk 碰撞体顶边"的斜法线摆成竖直。探针实测
/// (验收 CR3):跨 chunk 边界时前方 chunk 顶边进入预测距离,生成法线
/// (0, 0.957, 0.290) 的倾斜接触,solver 挡住沿法线的接近速度 → 竖直
/// 速度 1.54 m/s 腾空 ~20 拍。条件全满足时把法线摆成 (0,±1,0)、dist 改
/// 竖直间隙。只处理胶囊对固定刚体(地形);动态方块不处理。
/// `PhysicsHooks: MaybeSync`(默认要求 `Sync`,demo 不开 `unsync-callbacks`):
/// 内部用原子类型记开关与本拍摆正次数。
struct EdgeFixHooks {
    cap_collider: AtomicU32,
    cap_gen: AtomicU32,
    enabled: AtomicBool,
    /// 本拍被摆正的流形次数(每拍步进前清零,步进后采样)。
    fixed: AtomicU32,
}

impl EdgeFixHooks {
    fn set_cap(&self, h: ColliderHandle) {
        let (idx, gen) = h.into_raw_parts();
        // 索引与代数各占一半(u32 对,合存 u64 不必要:demo 单世界、代数增量小)。
        self.cap_collider.store(idx, Ordering::Relaxed);
        self.cap_gen.store(gen, Ordering::Relaxed);
    }

    fn cap(&self) -> ColliderHandle {
        ColliderHandle::from_raw_parts(
            self.cap_collider.load(Ordering::Relaxed),
            self.cap_gen.load(Ordering::Relaxed),
        )
    }

    fn enabled(&self) -> bool {
        self.enabled.load(Ordering::Relaxed)
    }

    fn reset_fixed(&self) {
        self.fixed.store(0, Ordering::Relaxed);
    }

    fn take_fixed(&self) -> u32 {
        self.fixed.load(Ordering::Relaxed)
    }

    fn bump_fixed(&self) {
        self.fixed.fetch_add(1, Ordering::Relaxed);
    }
}

impl PhysicsHooks for EdgeFixHooks {
    fn modify_solver_contacts(&self, ctx: &mut ContactModificationContext) {
        if !self.enabled() {
            return;
        }
        let cap = self.cap();
        let cap1 = ctx.collider1 == cap;
        let cap2 = ctx.collider2 == cap;
        if !(cap1 || cap2) {
            return;
        }
        // 只处理胶囊对固定刚体(地形 compound);动态方块不处理。
        let other_handle = if cap1 {
            ctx.rigid_body2
        } else {
            ctx.rigid_body1
        };
        let other_fixed = other_handle
            .and_then(|h| ctx.bodies.get(h))
            .is_some_and(|rb| rb.is_fixed());
        if !other_fixed {
            return;
        }
        // 胶囊底部 y(先取:rigid_mut 可变借用 ctx 后不能再读 ctx 字段)。
        let cap_handle = if cap1 {
            ctx.rigid_body1
        } else {
            ctx.rigid_body2
        };
        let Some(cap_y) = cap_handle
            .and_then(|h| ctx.bodies.get(h))
            .map(|rb| rb.translation().y)
        else {
            return;
        };
        let rigid = match ctx.rigid_mut() {
            Some(r) => r,
            None => return,
        };
        let ny = rigid.normal.y;
        // 斜向上(|n.y| ≥ 0.7)但不是竖直:共面缝的推测接触特征。
        if ny.abs() < 0.7 || ny.abs() >= 0.99999 {
            return;
        }
        let cap_bottom = cap_y - HALF_H - CAP_R;
        // 全部接触点在地形一侧的 y 都在胶囊底部 ±3 cm 以内(共面接缝,
        // 不是台阶边):台阶边的接触点在地形侧 y 跨度大,不满足。
        let in_band = rigid.solver_contacts.iter().all(|sc| {
            // 锚点在钩子内是世界空间;地形侧 = 与胶囊相反的一侧。
            let terrain_pt = if cap1 { sc.anchor2 } else { sc.anchor1 };
            (terrain_pt.y - cap_bottom).abs() <= EDGE_BAND
        });
        if !in_band {
            return;
        }
        // 法线摆成竖直(符号同原法线 y);dist 改成竖直间隙。
        let sign = ny.signum();
        *rigid.normal = Vector::new(0.0, sign, 0.0);
        for sc in rigid.solver_contacts.iter_mut() {
            let (a1, a2) = (sc.anchor1, sc.anchor2);
            let cap_pt = if cap1 { a1 } else { a2 };
            let terrain_pt = if cap1 { a2 } else { a1 };
            let vert_gap = cap_pt.y - terrain_pt.y;
            sc.dist = (vert_gap * sign).max(sc.dist.min(0.0));
        }
        self.bump_fixed();
    }
}

fn hash_f32(h: u64, v: f32) -> u64 {
    let mut h = h;
    for b in &v.to_ne_bytes() {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

// ── T1 地形(与 terrain/migrate 同构:greedy_colliders → fixed compound)──

fn neighbor_lookup(voxels: &[(ChunkPos, Chunk)], x: i32, z: i32) -> Option<&Chunk> {
    voxels
        .iter()
        .find(|(c, _)| c.x == x && c.z == z)
        .map(|(_, ch)| ch)
}

fn build_terrain_t1(world: &mut PhysicsWorld) -> Vec<ColliderHandle> {
    let gen = Superflat::classic();
    let reg = SimpleRegistry::builtin();
    let mut voxels = Vec::new();
    for cx in CHUNK_X_LO..CHUNK_X_HI {
        for cz in CHUNK_Z_LO..=CHUNK_Z_HI {
            let cp = ChunkPos::new(cx, cz);
            voxels.push((cp, gen.generate(WorldSeed(0), cp)));
        }
    }
    let mut handles = Vec::new();
    for (cp, chunk) in &voxels {
        let neighbors = NeighborChunks {
            px: neighbor_lookup(&voxels, cp.x + 1, cp.z),
            nx: neighbor_lookup(&voxels, cp.x - 1, cp.z),
            pz: neighbor_lookup(&voxels, cp.x, cp.z + 1),
            nz: neighbor_lookup(&voxels, cp.x, cp.z - 1),
        };
        let boxes: Vec<BoxCollider> = greedy_colliders(chunk, *cp, &neighbors, |b| reg.is_solid(b));
        if boxes.is_empty() {
            continue;
        }
        let parts: Vec<(Pose, SharedShape)> = boxes
            .iter()
            .map(|b| {
                (
                    Pose {
                        translation: Vector::new(b.center.x, b.center.y, b.center.z),
                        ..Pose::IDENTITY
                    },
                    SharedShape::cuboid(b.half.x, b.half.y, b.half.z),
                )
            })
            .collect();
        let (_, collider) = world.insert(
            RigidBodyBuilder::fixed(),
            ColliderBuilder::compound(parts)
                .friction(GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
        );
        handles.push(collider);
    }
    handles
}

// ── 接地判定(两种投射) ─────────────────────────────────────────────────

/// 返回 Some(命中面法线 y) = 行程内有命中;调用方判 ny ≥ 0.7。
/// 射线从胶囊底部上提 5 mm 起点向下(resting 时 toi≈0 的法线不可靠,故上提);
/// 球投射(R1/CR2):半径 CAP_R − 0.02 的球,从下半球球心上方 5 mm 向下,
/// 行程 ground_eps + 0.02 + 0.005——验收实证整胶囊投射在贴地行走时
/// 614/1500 拍返回 None、481 拍法线 y≈0(推测接触把整胶囊判离地),
/// 小球只测底弧正下方,贴地静止/行走都稳定命中。
fn ground_ny(
    world: &PhysicsWorld,
    cap_collider: ColliderHandle,
    pos: Vector,
    p: &Params,
    ball_shape: &SharedShape,
) -> Option<f32> {
    let lift = 0.005;
    let filter = QueryFilter::default().exclude_collider(cap_collider);
    if p.shape_cast {
        let max_d = p.ground_eps + 0.02 + lift;
        // 球心起点:下半球球心(胶囊底部圆心)上方 5 mm。
        let origin = Vector::new(pos.x, pos.y - HALF_H + lift, pos.z);
        let hit = world.cast_shape(
            &Pose {
                translation: origin,
                ..Pose::IDENTITY
            },
            Vector::new(0.0, -max_d, 0.0),
            &**ball_shape,
            ShapeCastOptions {
                max_time_of_impact: 1.0,
                // target_distance 必须 > 0:resting 时接触被推到 slop 内,
                // target=0 返回 None(探针 probe_cast2 实测)。
                target_distance: 0.002,
                stop_at_penetration: true,
                compute_impact_geometry_on_penetration: true,
            },
            filter,
        )?;
        Some(hit.1.normal1.y)
    } else {
        let max_d = p.ground_eps + lift;
        let ray = Ray {
            origin: Vector::new(pos.x, pos.y - HALF_H - CAP_R + lift, pos.z),
            dir: Vector::new(0.0, -1.0, 0.0),
        };
        let (_, hit) = world.cast_ray_and_get_normal(&ray, max_d, true, filter)?;
        Some(hit.normal.y)
    }
}

// ── 输出行 ──────────────────────────────────────────────────────────────

struct Outcome {
    name: String,
    end_tick: usize,
    end_reason: &'static str,
    nan: bool,
    panicked: bool,
    guard_removed: u32,
    panic_msg: String,
    speed_target: Option<f32>,
    v_mean: Option<f32>,
    speed_err_pct: Option<f32>,
    /// R1/CR6:从意图开始(第 60 拍)到水平速度首 ≥ 90% 目标的毫秒数;
    /// 到稳态窗口结束都没达到 = None(输出 "-")。
    t90_ms: Option<f32>,
    stop_dist_m: Option<f32>,
    stop_end_v: Option<f32>,
    /// R1/第 7 条:从第 60 拍(意图起)统计的 |vy| 峰值,出生落地不计。
    vy_peak: f32,
    /// 第 0–59 拍的 |vy| 峰值(出生落地)。
    vy_peak_settle: f32,
    /// 行走稳态窗口内控制拍判为接地的比例(R1/第 2 条)。
    ground_frac: Option<f32>,
    /// 全程被接缝钩子摆正的流形次数(R1/第 3 条)。
    edge_snaps: u32,
    /// 方块堆场景:最后一块方块落回地面那一拍的对地最深穿透(R1/第 10 条)。
    pile_land_pen: Option<f32>,
    /// 同一拍胶囊的竖直速度(R1/第 10 条)。
    pile_land_vy: Option<f32>,
    pen_min: Option<f32>,
    pen_obj_min: Option<f32>,
    drift_m: Option<f32>,
    sink_m: Option<f32>,
    sleep_ratio: Option<f32>,
    slept: Option<bool>,
    jump_max_h: Option<f32>,
    bounce_vy: Option<f32>,
    hit_disp_h: Option<f32>,
    speed_max: f32,
    block_disp_max: Option<f32>,
    blocks_woken: Option<u32>,
    dip_pct: Option<f32>,
    stuck: Option<bool>,
    step_success: Option<bool>,
    course_err_deg: Option<f32>,
    state_hash: u64,
}

fn opt_f(v: Option<f32>) -> String {
    v.map_or_else(|| "-".to_string(), |x| format!("{x:.4}"))
}

fn opt_b(v: Option<bool>) -> String {
    v.map_or_else(|| "-".to_string(), |x| x.to_string())
}

fn opt_u(v: Option<u32>) -> String {
    v.map_or_else(|| "-".to_string(), |x| x.to_string())
}

fn fmt4(v: f32) -> String {
    format!("{v:.4}")
}

fn outcome_csv(o: &Outcome) -> String {
    // 列头(34 列)见 main 的 MATRIX_HEADER;panic 行除前 7 列全 "-"。
    let hash_cell = if o.panicked {
        "PANIC".to_string()
    } else {
        format!("{:016x}", o.state_hash)
    };
    [
        o.name.clone(),
        o.end_tick.to_string(),
        o.end_reason.to_string(),
        o.nan.to_string(),
        o.panicked.to_string(),
        o.guard_removed.to_string(),
        o.panic_msg.clone(),
        opt_f(o.speed_target),
        opt_f(o.v_mean),
        opt_f(o.speed_err_pct),
        opt_f(o.t90_ms),
        opt_f(o.stop_dist_m),
        opt_f(o.stop_end_v),
        fmt4(o.vy_peak),
        fmt4(o.vy_peak_settle),
        opt_f(o.ground_frac),
        o.edge_snaps.to_string(),
        opt_f(o.pile_land_pen),
        opt_f(o.pile_land_vy),
        opt_f(o.pen_min),
        opt_f(o.pen_obj_min),
        opt_f(o.drift_m),
        opt_f(o.sink_m),
        opt_f(o.sleep_ratio),
        opt_b(o.slept),
        opt_f(o.jump_max_h),
        opt_f(o.bounce_vy),
        opt_f(o.hit_disp_h),
        fmt4(o.speed_max),
        opt_f(o.block_disp_max),
        opt_u(o.blocks_woken),
        opt_f(o.dip_pct),
        opt_b(o.stuck),
        opt_b(o.step_success),
        opt_f(o.course_err_deg),
        hash_cell,
    ]
    .join(",")
}

fn panic_row(cfg: &Cfg, msg: &str) -> Outcome {
    Outcome {
        name: cfg.name.clone(),
        end_tick: 0,
        end_reason: "panic",
        nan: false,
        panicked: true,
        guard_removed: 0,
        panic_msg: msg.to_string(),
        speed_target: None,
        v_mean: None,
        speed_err_pct: None,
        t90_ms: None,
        stop_dist_m: None,
        stop_end_v: None,
        vy_peak: 0.0,
        vy_peak_settle: 0.0,
        ground_frac: None,
        edge_snaps: 0,
        pile_land_pen: None,
        pile_land_vy: None,
        pen_min: None,
        pen_obj_min: None,
        drift_m: None,
        sink_m: None,
        sleep_ratio: None,
        slept: None,
        jump_max_h: None,
        bounce_vy: None,
        hit_disp_h: None,
        speed_max: 0.0,
        block_disp_max: None,
        blocks_woken: None,
        dip_pct: None,
        stuck: None,
        step_success: None,
        course_err_deg: None,
        state_hash: 0,
    }
}

fn run(cfg: &Cfg) -> Outcome {
    match catch_unwind(AssertUnwindSafe(|| run_inner(cfg))) {
        Ok(o) => o,
        Err(e) => {
            let msg = e
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_else(|| "non-string panic".into());
            panic_row(cfg, &msg.replace(',', "; "))
        }
    }
}

// ── 单次运行 ────────────────────────────────────────────────────────────

fn run_inner(cfg: &Cfg) -> Outcome {
    let p = cfg.p;
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
    world.integration_parameters = production_params(1.0 / 60.0);
    // R1/CR7:edgefix 开关(只有接缝 fix0 行关,其余全开)。
    let edgefix_on = match cfg.scene {
        Scene::Seam { fix, .. } => fix == EdgeFix::Fix1,
        _ => true,
    };

    // ── 布景 ──
    let t1 = match cfg.scene {
        Scene::Walk { t1, .. } | Scene::Stand { t1 } => t1,
        Scene::Seam { .. } => true,
        _ => false,
    };
    let ground_top = if t1 { GROUND_TOP_T1 } else { GROUND_TOP };
    let mut terrain: Vec<ColliderHandle> = Vec::new();
    if t1 {
        terrain = build_terrain_t1(&mut world);
    } else {
        let (_, c) = world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, ground_top - 0.5, 0.0)),
            ColliderBuilder::cuboid(200.0, 0.5, 200.0)
                .friction(GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
        );
        terrain.push(c);
    }
    // 台阶 = 固定块体(像摆放的体素;方块堆才是可推动的动态块)。
    let step_collider = if let Scene::Step { height, .. } = cfg.scene {
        let hf = height as f32;
        let (_, c) = world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(10.0, hf * 0.5, 0.0)),
            ColliderBuilder::cuboid(4.0, hf * 0.5, 3.0)
                .friction(GROUND_FRICTION)
                .restitution(GROUND_RESTITUTION),
        );
        Some(c)
    } else {
        None
    };
    // 场景 9:16 个刚体在主体之前插入。位置 z≈110 m 在全部 8 条行走路径之外
    // (最远行走点 |z|≤90);migrate 实证入睡事件的 swap_remove 与距离无关,
    // 200 m 外同样生效。ball 变体 = migrate 同款(醒着落地后入睡);
    // 默认 = 放行字面(无碰撞体、出生即睡)。
    if let Scene::Order { ball, .. } = cfg.scene {
        for i in 0..16usize {
            let z = 110.0 + i as f32 * 0.8;
            if ball {
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(0.0, ground_top + 0.5, z)),
                    ColliderBuilder::ball(0.01)
                        .density(1.0)
                        .friction(0.6)
                        .restitution(0.1),
                );
            } else {
                world.insert_body(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(0.0, ground_top + 0.5, z))
                        .sleeping(true),
                );
            }
        }
    }

    // ── 主体出生 ──
    let spawn = match cfg.scene {
        Scene::Walk { t1: true, .. } | Scene::Seam { .. } => {
            Vector::new(T1_SPAWN_X, GROUND_TOP_T1 + 0.902, 0.0)
        }
        Scene::Walk { .. } => Vector::new(0.0, GROUND_TOP + 0.902, 0.0),
        Scene::Stand { t1: true } => Vector::new(168.0, GROUND_TOP_T1 + 0.902, 8.0),
        Scene::Stand { .. } => Vector::new(8.0, GROUND_TOP + 0.902, 8.0),
        Scene::Step { run, d, trial, .. } => {
            let (dx, dz) = PERTURB[trial % PERTURB.len()];
            if run {
                Vector::new(dx, GROUND_TOP + 0.902, dz)
            } else {
                // R1/CR5:d = 胶囊表面到台阶面的间隙 → 中心 x = 面 x − CAP_R − d。
                Vector::new(STEP_FACE_X - CAP_R - d + dx, GROUND_TOP + 0.902, dz)
            }
        }
        _ => Vector::new(0.0, GROUND_TOP + 0.902, 0.0),
    };
    let (cap_body, cap_collider) = spawn_capsule(&mut world, spawn, &p);
    let cap_mass = world.bodies.get(cap_body).expect("alive").mass();
    assert!(
        (cap_mass - CAP_MASS).abs() < 1e-4,
        "capsule mass {cap_mass}"
    );
    // R1/CR2:球投射形状(半径 CAP_R − 0.02)。
    let ball_shape = SharedShape::ball(BALL_CAST_R);
    // R1/CR3:接缝修正钩子(每拍 step_with_events 步进)。
    let hooks = EdgeFixHooks {
        cap_collider: AtomicU32::new(0),
        cap_gen: AtomicU32::new(0),
        enabled: AtomicBool::new(edgefix_on),
        fixed: AtomicU32::new(0),
    };
    hooks.set_cap(cap_collider);

    // ── 场景方块(方块堆/推方块:出生即睡)──
    let mut blocks: Vec<(RigidBodyHandle, ColliderHandle, Vector)> = Vec::new();
    match cfg.scene {
        Scene::Pile { layers } => {
            for layer in 0..layers {
                for ix in 0..3i32 {
                    for iz in 0..3i32 {
                        let pos =
                            Vector::new(6.5 + ix as f32, 0.5 + layer as f32, -1.0 + iz as f32);
                        let (b, c) = spawn_block(&mut world, pos, true, 1.0);
                        blocks.push((b, c, pos));
                    }
                }
            }
        }
        Scene::Push => {
            let pos = Vector::new(4.0, 0.5, 0.0);
            let (b, c) = spawn_block(&mut world, pos, true, 1.0);
            blocks.push((b, c, pos));
        }
        _ => {}
    }
    let mut ever_awake = vec![false; blocks.len()];

    // ── 脚本与统计 ──
    let total = total_ticks(cfg);
    let rel = release_tick(cfg);
    let s0 = steady0(cfg);
    let walk_like = matches!(
        cfg.scene,
        Scene::Walk { .. } | Scene::Seam { .. } | Scene::Order { .. }
    );
    let target_speed = match cfg.scene {
        Scene::Walk { speed, .. } | Scene::Seam { speed, .. } => Some(speed),
        Scene::Order { .. } => Some(SEAM_SPEED),
        Scene::Pile { .. } => Some(2.0),
        Scene::Push => Some(4.3),
        _ => None,
    };
    let course_dir = match cfg.scene {
        Scene::Walk { heading, .. }
        | Scene::Seam { heading, .. }
        | Scene::Order { heading, .. } => Some(heading_dir(heading)),
        _ => None,
    };
    let mut pressed = false;
    let mut move_until = usize::MAX;
    let mut held_force = Vector::ZERO;
    // 首拍强制写入 yaw(NaN 与任何数比较为 false → 触发写入)。
    let mut last_yaw = f32::NAN;
    let mut cap_alive = true;
    let mut nan = false;
    let mut guard_removed = 0u32;
    let mut end_reason = "completed";
    let mut end_tick = total;
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    // R1/第 7 条:vy_peak 从第 60 拍起统计(出生落地不计),另报第 0–59 拍。
    let mut vy_peak = 0.0f32;
    let mut vy_peak_settle = 0.0f32;
    let mut speed_max = 0.0f32;
    let mut pen_min = f32::INFINITY;
    let mut pen_obj = f32::INFINITY;
    let mut v_sum = 0.0f64;
    let mut v_n = 0u64;
    let mut c_dot = 0.0f64;
    let mut c_cross = 0.0f64;
    // R1/CR6:t90 从意图开始拍(60)起算,窗口扩到整个意图期;稳态窗内没
    // 达到就保持 None(输出 "-")。dip/stuck 仍在稳态窗(t90 达到后)。
    let mut t90: Option<usize> = None;
    let mut ground_hits = 0u64;
    let mut ground_n = 0u64;
    let mut edge_snaps = 0u32;
    let mut dip = 0.0f32;
    let mut stuck_run = 0u32;
    let mut stuck_max = 0u32;
    let mut stop_base: Option<Vector> = None;
    let mut stop_dist = 0.0f32;
    let mut stop_v_end = 0.0f32;
    let mut stopped = false;
    let mut stand_base: Option<Vector> = None;
    let mut sleep_ticks = 0u64;
    let mut jump_max_h: Option<f32> = None;
    let mut landed_tick: Option<usize> = None;
    let mut bounce_vy = 0.0f32;
    let mut hit_base: Option<Vector> = None;
    let mut block_disp_max = 0.0f32;

    for tick in 0..total {
        // 事件:被砸/重物在第 60 拍(安定后)按当时机身位姿定位生成。
        if tick == 60 && cap_alive && matches!(cfg.scene, Scene::Hit { .. } | Scene::Heavy { .. }) {
            let bp = world.bodies.get(cap_body).expect("alive").translation();
            hit_base = Some(bp);
            let top = bp.y + HALF_H + CAP_R;
            match cfg.scene {
                Scene::Hit { ox, oz, d } => {
                    let pos = Vector::new(bp.x + ox, top + 0.5 + 3.0 + d, bp.z + oz);
                    let (b, c) = spawn_block(&mut world, pos, false, 1.0);
                    blocks.push((b, c, pos));
                    ever_awake.push(false);
                }
                Scene::Heavy { h, ox, oz, d } => {
                    let pos = Vector::new(bp.x + ox, top + h + 5.0 + d, bp.z + oz);
                    let (b, c) = spawn_block(&mut world, pos, false, 2.0 * h);
                    let m = world.bodies.get(b).expect("alive").mass();
                    let want = 8.0 * h * h * h;
                    assert!(
                        (m - want).abs() < want * 1e-3,
                        "heavy mass {m} != 8·h³ = {want}"
                    );
                    blocks.push((b, c, pos));
                    ever_awake.push(false);
                }
                _ => unreachable!(),
            }
        }
        if tick == 120 && matches!(cfg.scene, Scene::Stand { .. }) && cap_alive {
            stand_base = Some(world.bodies.get(cap_body).expect("alive").translation());
        }

        // ── 控制器(30 Hz 决策)──
        if cap_alive && tick % 2 == 0 {
            let (mv, yaw, want_jump) = match cfg.scene {
                Scene::Walk { speed, heading, .. } => {
                    let act = tick >= 60 && tick < rel;
                    (
                        if act {
                            heading_dir(heading) * speed
                        } else {
                            Vector::ZERO
                        },
                        heading_rad(heading),
                        false,
                    )
                }
                Scene::Seam { speed, heading, .. } => {
                    let act = tick >= 60 && tick < rel;
                    (
                        if act {
                            heading_dir(heading) * speed
                        } else {
                            Vector::ZERO
                        },
                        heading_rad(heading),
                        false,
                    )
                }
                Scene::Order { heading, .. } => {
                    let act = tick >= 60 && tick < rel;
                    (
                        if act {
                            heading_dir(heading) * SEAM_SPEED
                        } else {
                            Vector::ZERO
                        },
                        heading_rad(heading),
                        false,
                    )
                }
                Scene::Stand { .. } => (Vector::ZERO, 0.0, false),
                Scene::Step { run, d, .. } => {
                    let mut want = false;
                    if run {
                        // 跑动起跳:意图从 60 拍起,表面到面间隙 ≤ d 时按跳
                        // (帧量化:4.3 m/s 每拍 0.072 m,实际按跳距离 ∈ (d−0.072, d])。
                        if !pressed {
                            let cap_x = world.bodies.get(cap_body).expect("alive").translation().x;
                            if STEP_FACE_X - cap_x - CAP_R <= d {
                                pressed = true;
                                move_until = tick + 72;
                                want = true;
                            }
                        }
                    } else if tick == 120 {
                        // 静止起跳:安定 1 s 后按跳并同时给移动意图。
                        pressed = true;
                        move_until = tick + 72;
                        want = true;
                    }
                    let mv = if (run && tick >= 60 && (!pressed || tick < move_until))
                        || (!run && pressed && tick < move_until)
                    {
                        Vector::new(4.3, 0.0, 0.0)
                    } else {
                        Vector::ZERO
                    };
                    (mv, 0.0, want)
                }
                Scene::AirJump { repeat } => (
                    Vector::ZERO,
                    0.0,
                    if repeat {
                        (60..660).contains(&tick)
                    } else {
                        tick == 60
                    },
                ),
                Scene::Pile { .. } => (
                    if tick >= 60 {
                        Vector::new(2.0, 0.0, 0.0)
                    } else {
                        Vector::ZERO
                    },
                    0.0,
                    tick >= 60,
                ),
                Scene::Push => (
                    if tick >= 60 && tick < rel {
                        Vector::new(4.3, 0.0, 0.0)
                    } else {
                        Vector::ZERO
                    },
                    0.0,
                    false,
                ),
                Scene::Hit { .. } | Scene::Heavy { .. } => (Vector::ZERO, 0.0, false),
            };
            // 接地判定(只在控制拍做)。
            let cap_pos = world.bodies.get(cap_body).expect("alive").translation();
            let grounded = ground_ny(&world, cap_collider, cap_pos, &p, &ball_shape)
                .is_some_and(|ny| ny >= 0.7);
            // R1/第 2 条:稳态窗内控制拍判为接地的比例(与驱动力同一判定)。
            if tick >= s0 && tick < rel {
                ground_n += 1;
                if grounded {
                    ground_hits += 1;
                }
            }
            // 水平力:F = clamp(m·(v_target − v_h)/τ, F_max);空中 × air。
            let v = world.bodies.get(cap_body).expect("alive").linvel();
            let vh = Vector::new(v.x, 0.0, v.z);
            let mut f = (mv - vh) * (CAP_MASS / p.tau);
            let fmax = p.fmax_mul * CAP_MASS * GRAVITY;
            let fl = f.length();
            if fl > fmax {
                f *= fmax / fl;
            }
            if !grounded {
                f *= p.air;
            }
            held_force = if f.length() < 0.01 { Vector::ZERO } else { f };
            // 朝向按意图 yaw 直接设置(只在变化时写,允许入睡)。
            if (yaw - last_yaw).abs() > 1e-6 {
                world
                    .bodies
                    .get_mut(cap_body)
                    .expect("alive")
                    .set_rotation(yaw_quat(yaw), true);
                last_yaw = yaw;
            }
            // 跳跃:只在接地时生效;先清零竖直速度再加冲量。
            if want_jump && grounded {
                let rb = world.bodies.get_mut(cap_body).expect("alive");
                let mut lv = rb.linvel();
                lv.y = 0.0;
                rb.set_linvel(lv, true);
                rb.apply_impulse(
                    Vector::new(0.0, CAP_MASS * (2.0 * GRAVITY * p.h_jump).sqrt(), 0.0),
                    true,
                );
            }
        }
        // rapier 0.36.1 的 add_force 不自动清零(文档:力持续生效直到
        // reset_forces)→ 每拍先清旧力再施加本次(30 Hz 决策、60 Hz 施加)。
        if cap_alive {
            let rb = world.bodies.get_mut(cap_body).expect("alive");
            rb.reset_forces(false);
            if held_force != Vector::ZERO {
                rb.add_force(held_force, true);
            }
        }

        // R1/CR3:接缝修正钩子步进(每拍清零计数,步进后采样)。
        hooks.reset_fixed();
        world.step_with_events(&hooks, &());
        edge_snaps += hooks.take_fixed();

        // 失控防护(重物 vg 行;先查非有限再查阈值)。
        if cap_alive && cfg.v_guard.is_some() {
            let rb = world.bodies.get(cap_body).expect("alive");
            let v = rb.linvel();
            let pv = rb.translation();
            let q = rb.rotation();
            let bad = !v.x.is_finite()
                || !v.y.is_finite()
                || !v.z.is_finite()
                || !pv.x.is_finite()
                || !pv.y.is_finite()
                || !pv.z.is_finite()
                || !q.w.is_finite()
                || !q.x.is_finite()
                || !q.y.is_finite()
                || !q.z.is_finite();
            if bad {
                nan = true;
            }
            if bad || v.length() > V_GUARD {
                world.remove_body(cap_body);
                cap_alive = false;
                guard_removed = 1;
            }
        }

        // ── 采样 ──
        if cap_alive {
            let rb = world.bodies.get(cap_body).expect("alive");
            let pv = rb.translation();
            let v = rb.linvel();
            let q = rb.rotation();
            if !pv.x.is_finite()
                || !pv.y.is_finite()
                || !pv.z.is_finite()
                || !v.x.is_finite()
                || !v.y.is_finite()
                || !v.z.is_finite()
            {
                nan = true;
                end_reason = "nan";
                end_tick = tick;
                break;
            }
            for c in [pv.x, pv.y, pv.z] {
                hash = hash_f32(hash, c);
            }
            for c in [q.w, q.x, q.y, q.z] {
                hash = hash_f32(hash, c);
            }
            for c in [v.x, v.y, v.z] {
                hash = hash_f32(hash, c);
            }
            // R1/第 7 条:vy_peak 分窗——第 0–59 拍(出生落地)单独报,
            // 意图期起的另报 vy_peak。
            if tick < 60 {
                vy_peak_settle = vy_peak_settle.max(v.y.abs());
            } else {
                vy_peak = vy_peak.max(v.y.abs());
            }
            speed_max = speed_max.max(v.length());
            if pv.y < ground_top - 5.0 {
                end_reason = "fell";
                end_tick = tick;
                break;
            }
            // R1/CR4:水平位置离开 T1 域(世界 x ∈ chunk 域、z 同)。
            if t1 {
                let x_lo = (CHUNK_X_LO as f32) * 16.0;
                let x_hi = (CHUNK_X_HI as f32) * 16.0;
                let z_lo = (CHUNK_Z_LO as f32) * 16.0;
                let z_hi = (CHUNK_Z_HI as f32) * 16.0;
                if pv.x < x_lo || pv.x > x_hi || pv.z < z_lo || pv.z > z_hi {
                    end_reason = "out_of_domain";
                    end_tick = tick;
                    break;
                }
            }
            // R1/第 10 条:方块堆场景只报结果——对地最深穿透(pen_min,全程;
            // 落回地面拍即全程最深,上一版口径 −0.0633 m)与第 60 拍后的
            // |vy| 峰值(= 落回地面时刻的竖直速度,上一版 6.86 m/s)。
            // 收尾时:pile_land_pen = pen_min、pile_land_vy = vy_peak。
            if matches!(cfg.scene, Scene::Stand { .. }) {
                if tick == 120 {
                    stand_base = Some(pv);
                }
                if tick >= 120 && rb.is_sleeping() {
                    sleep_ticks += 1;
                }
            }
            let vh2 = (v.x * v.x + v.z * v.z).sqrt();
            if walk_like {
                // R1/CR6:t90 从意图开始拍起,整个意图窗内找首 ≥ 90%。
                if tick >= 60
                    && tick < rel
                    && t90.is_none()
                    && vh2 >= 0.9 * target_speed.unwrap_or(0.0)
                {
                    t90 = Some(tick);
                }
                if tick >= s0 && tick < rel {
                    let ts = target_speed.expect("walk-like has target");
                    v_sum += f64::from(vh2);
                    v_n += 1;
                    let dir = course_dir.expect("walk-like has dir");
                    c_dot += f64::from(dir.x * v.x + dir.z * v.z);
                    c_cross += f64::from(dir.x * v.z - dir.z * v.x);
                    if t90.is_some() {
                        dip = dip.max((ts - vh2) / ts);
                        if vh2 < 0.5 * ts {
                            stuck_run += 1;
                            stuck_max = stuck_max.max(stuck_run);
                        } else {
                            stuck_run = 0;
                        }
                    }
                }
                if tick == rel {
                    stop_base = Some(pv);
                }
                if let Some(sb) = stop_base.filter(|_| !stopped) {
                    let dx = pv.x - sb.x;
                    let dz = pv.z - sb.z;
                    stop_dist = (dx * dx + dz * dz).sqrt();
                    stop_v_end = vh2;
                    if vh2 < STOP_SPEED {
                        stopped = true;
                    }
                }
            }
            if matches!(cfg.scene, Scene::Step { .. } | Scene::AirJump { .. }) {
                let bottom = pv.y - HALF_H - CAP_R - ground_top;
                jump_max_h = Some(jump_max_h.map_or(bottom, |m: f32| m.max(bottom)));
            }
            if let Scene::Step { height, .. } = cfg.scene {
                let top = height as f32;
                let bottom = pv.y - HALF_H - CAP_R;
                if landed_tick.is_none()
                    && pv.x > STEP_FACE_X
                    && bottom <= top + 0.01
                    && bottom > top - 0.5
                    && v.y <= 0.05
                {
                    landed_tick = Some(tick);
                }
                if let Some(lt) = landed_tick {
                    if tick <= lt + 30 {
                        bounce_vy = bounce_vy.max(v.y);
                    }
                }
            }
            // 穿透:对地(地面/体素)与对物(台阶/方块)分开。
            for pair in world.narrow_phase.contact_graph().interactions() {
                if let PairContacts::Rigid(rigid) = &pair.contacts {
                    let cap_in = pair.collider1 == cap_collider || pair.collider2 == cap_collider;
                    if !cap_in {
                        continue;
                    }
                    let other = if pair.collider1 == cap_collider {
                        pair.collider2
                    } else {
                        pair.collider1
                    };
                    for manifold in &rigid.manifolds {
                        if manifold.points.is_empty() {
                            continue;
                        }
                        // 口径(capsule 探针实证):capsule-vs-cuboid 流形含
                        // "上端点 vs 地面"的投影接触点(dist 恒 1.202,>
                        // prediction 0.02 不进 solver),按 mass 的 reduce(max)
                        // 取 deepest 会把它选中。穿透语义 = 流形内最深穿透,
                        // 所以每流形取 **min(dist)**(dist<0 = 穿透,>0 = 间隙),
                        // 跨流形/跨拍再取 min。
                        let deepest = manifold
                            .points
                            .iter()
                            .map(|pt| pt.dist)
                            .reduce(f32::min)
                            .expect("points nonempty");
                        if terrain.contains(&other) {
                            pen_min = pen_min.min(deepest);
                        } else if step_collider == Some(other)
                            || blocks.iter().any(|(_, bc, _)| bc == &other)
                        {
                            pen_obj = pen_obj.min(deepest);
                        }
                    }
                }
            }
        }
        // 方块哈希与统计(主体被防护移除后世界照跑)。
        for (i, (bh, _bc, bpos)) in blocks.iter().enumerate() {
            let rb = world.bodies.get(*bh).expect("alive");
            let bp = rb.translation();
            let bq = rb.rotation();
            for c in [bp.x, bp.y, bp.z] {
                hash = hash_f32(hash, c);
            }
            for c in [bq.w, bq.x, bq.y, bq.z] {
                hash = hash_f32(hash, c);
            }
            if !rb.is_sleeping() {
                ever_awake[i] = true;
            }
            block_disp_max = block_disp_max.max((bp - bpos).length());
        }
    }

    // ── 收尾 ──
    let v_mean = if v_n > 0 {
        (v_sum / v_n as f64) as f32
    } else {
        0.0
    };
    let speed_err_pct = target_speed.map(|ts| (ts - v_mean).abs() / ts * 100.0);
    let course_err_deg = if walk_like && v_n > 0 {
        let dot = c_dot / v_n as f64;
        let cross = c_cross / v_n as f64;
        Some((f32::atan2(cross as f32, dot as f32)) * 180.0 / std::f32::consts::PI)
    } else {
        None
    };
    let t90_ms = t90.map(|t| (t - 60) as f32 * 1000.0 / 60.0);
    let ground_frac = if walk_like && ground_n > 0 {
        Some(ground_hits as f32 / ground_n as f32)
    } else {
        None
    };
    // R1/第 10 条:方块堆场景只报结果(落回地面拍 = 全程对地最深穿透 +
    // 第 60 拍起 |vy| 峰值;口径见采样段注释)。
    let (pile_land_pen, pile_land_vy) = if matches!(cfg.scene, Scene::Pile { .. }) {
        (
            if pen_min.is_finite() {
                Some(pen_min)
            } else {
                None
            },
            Some(vy_peak),
        )
    } else {
        (None, None)
    };
    let stop_fields = if walk_like {
        Some((stop_dist, stop_v_end))
    } else {
        None
    };
    let (drift_m, sink_m, sleep_ratio, slept) = if matches!(cfg.scene, Scene::Stand { .. }) {
        if cap_alive {
            let pv = world.bodies.get(cap_body).expect("alive").translation();
            let base = stand_base.unwrap_or(pv);
            let dx = pv.x - base.x;
            let dz = pv.z - base.z;
            (
                Some((dx * dx + dz * dz).sqrt()),
                Some(base.y - pv.y),
                Some(sleep_ticks as f32 / (total - 120) as f32),
                Some(world.bodies.get(cap_body).expect("alive").is_sleeping()),
            )
        } else {
            (Some(0.0), Some(0.0), Some(0.0), Some(false))
        }
    } else {
        (None, None, None, None)
    };
    let hit_disp_h = match cfg.scene {
        Scene::Hit { .. } | Scene::Heavy { .. } => {
            if cap_alive {
                let pv = world.bodies.get(cap_body).expect("alive").translation();
                let base = hit_base.unwrap_or(pv);
                Some(((pv.x - base.x).powi(2) + (pv.z - base.z).powi(2)).sqrt())
            } else {
                None
            }
        }
        _ => None,
    };
    let step_success = if let Scene::Step { height, .. } = cfg.scene {
        let ok = cap_alive && {
            let rb = world.bodies.get(cap_body).expect("alive");
            let pv = rb.translation();
            let v = rb.linvel();
            let bottom = pv.y - HALF_H - CAP_R;
            bottom > height as f32 - 0.2
                && pv.x > STEP_FACE_X + 0.2
                && pv.x < STEP_TAIL_X - 0.2
                && v.length() < 1.5
        };
        Some(ok)
    } else {
        None
    };
    let opt_pen = |v: f32| if v.is_finite() { Some(v) } else { None };
    let blocks_woken = if blocks.is_empty() {
        None
    } else {
        Some(ever_awake.iter().filter(|&&a| a).count() as u32)
    };
    let block_disp = if blocks.is_empty() {
        None
    } else {
        Some(block_disp_max)
    };
    let stuck = if walk_like {
        Some(stuck_max >= STUCK_TICKS)
    } else {
        None
    };
    let dip_pct = if walk_like && t90.is_some() {
        Some(dip * 100.0)
    } else {
        None
    };

    Outcome {
        name: cfg.name.clone(),
        end_tick,
        end_reason,
        nan,
        panicked: false,
        guard_removed,
        panic_msg: String::new(),
        speed_target: target_speed,
        v_mean: if walk_like { Some(v_mean) } else { None },
        speed_err_pct,
        t90_ms,
        stop_dist_m: stop_fields.map(|s| s.0),
        stop_end_v: stop_fields.map(|s| s.1),
        vy_peak,
        vy_peak_settle,
        ground_frac,
        edge_snaps,
        pile_land_pen,
        pile_land_vy,
        pen_min: opt_pen(pen_min),
        pen_obj_min: opt_pen(pen_obj),
        drift_m,
        sink_m,
        sleep_ratio,
        slept,
        jump_max_h,
        bounce_vy: if landed_tick.is_some() {
            Some(bounce_vy)
        } else {
            None
        },
        hit_disp_h,
        speed_max,
        block_disp_max: block_disp,
        blocks_woken,
        dip_pct,
        stuck,
        step_success,
        course_err_deg,
        state_hash: hash,
    }
}

// ── 扫描(阶段 A:平地全组合;阶段 B:1 格台阶 h_jump)与候选选择 ────────

struct TupleAgg {
    p: Params,
    err_max: f32,
    /// 每速度档 err(顺序同 WALK_SPEEDS;供分层回退与排序)。
    err_by_speed: [f32; 3],
    /// 三档全部 ≤5%(严格口径;纯 P 下 s1.5 档网格内不可达,见 README)。
    pass_a: bool,
    /// 4.3 与 5.6 档 ≤5%(回退层;判据 1 的 s1.5 缺口如实上报)。
    pass_hi: bool,
}

struct JumpAgg {
    p: Params,
    ok: u32,
    n: u32,
}

struct SweepRow {
    name: String,
    stage: &'static str,
    speed_target: String,
    v_mean: String,
    speed_err_pct: String,
    t90_ms: String,
    vy_peak: String,
    step_ok: String,
    step_n: String,
    pass: String,
    nan: bool,
    state_hash: String,
}

const SWEEP_HEADER: &str = "name,stage,speed_target,v_mean,speed_err_pct,t90_ms,\
     vy_peak,step_ok,step_n,pass,nan,state_hash";

fn sweep_row_csv(r: &SweepRow) -> String {
    format!(
        "{},{},{},{},{},{},{},{},{},{},{},{}",
        r.name,
        r.stage,
        r.speed_target,
        r.v_mean,
        r.speed_err_pct,
        r.t90_ms,
        r.vy_peak,
        r.step_ok,
        r.step_n,
        r.pass,
        r.nan,
        r.state_hash,
    )
}

/// 机械选择规则(README 同文):passB 项按 (err_max 升序, step 失败数升序,
/// tau, fmax, mu, air, ge 升序, shape=false<true, h_jump 升序) 全序排序,
/// 取 首 / 中 / 尾 各一套(去重)。passB 为空时分层回退:passA(三档 ≤5%)
/// → pass_hi(仅 4.3/5.6 档 ≤5%,hj=1.25)→ 全部元组(err_max 排序,
/// hj=1.25;判据 1 缺口如实上报)。
/// R1:Min 摩擦合成下 μ_eff = min(μ, 0.6),μ=0 → 0,纯 P 律稳态误差不再
/// 含摩擦项,τ·μ_eff·g = 0,网格内三档均可过(实际见 sweep 输出)。
fn select_candidates(aggs: &[TupleAgg], jumps: &[JumpAgg]) -> Vec<(String, Params)> {
    let mut items: Vec<(f32, u32, Params)> = jumps
        .iter()
        .filter(|j| j.ok == j.n)
        .map(|j| {
            let a = aggs
                .iter()
                .find(|t| same_tuple(&t.p, &j.p))
                .expect("jump tuple must exist in aggs");
            (a.err_max, j.n - j.ok, j.p)
        })
        .collect();
    if items.is_empty() {
        items = aggs
            .iter()
            .filter(|t| t.pass_a)
            .map(|t| {
                let mut p = t.p;
                p.h_jump = 1.25;
                (t.err_max, 0u32, p)
            })
            .collect();
    }
    if items.is_empty() {
        items = aggs
            .iter()
            .filter(|t| t.pass_hi)
            .map(|t| {
                let mut p = t.p;
                p.h_jump = 1.25;
                (t.err_by_speed[1], 0u32, p)
            })
            .collect();
    }
    if items.is_empty() {
        items = aggs
            .iter()
            .map(|t| {
                let mut p = t.p;
                p.h_jump = 1.25;
                (t.err_by_speed[1], 0u32, p)
            })
            .collect();
    }
    assert!(!items.is_empty(), "sweep: 无任何通过的元组");
    items.sort_by(|a, b| {
        a.0.total_cmp(&b.0)
            .then(b.1.cmp(&a.1))
            .then(a.2.tau.total_cmp(&b.2.tau))
            .then(a.2.fmax_mul.total_cmp(&b.2.fmax_mul))
            .then(a.2.mu.total_cmp(&b.2.mu))
            .then(a.2.air.total_cmp(&b.2.air))
            .then(a.2.ground_eps.total_cmp(&b.2.ground_eps))
            .then(a.2.shape_cast.cmp(&b.2.shape_cast))
            .then(a.2.h_jump.total_cmp(&b.2.h_jump))
    });
    let n = items.len();
    let mut picked = vec![items[0].2, items[n / 2].2, items[n - 1].2];
    picked.dedup();
    picked
        .into_iter()
        .enumerate()
        .map(|(i, p)| (format!("c{}", i + 1), p))
        .collect()
}

fn same_tuple(a: &Params, b: &Params) -> bool {
    a.tau == b.tau
        && a.fmax_mul == b.fmax_mul
        && a.mu == b.mu
        && a.air == b.air
        && a.ground_eps == b.ground_eps
        && a.shape_cast == b.shape_cast
}

fn write_cands(path: &str, cands: &[(String, Params)]) {
    let mut s = String::from(
        "# capsule A00-4 R1 候选(sweep 机械规则选出;规则见 README):\n\
         # 首选 passB(step 1 格跑跳 d=1.0,3/3 成功)项;为空时分层回退:\n\
         # passA(三速度档 err≤5%)→ pass_hi(仅 4.3/5.6 档 ≤5%,hj=1.25)\n\
         # → 全部元组(按 4.3 档 err 排序,hj=1.25)。\n\
         # 层内排序:(err 升序, step 失败数升序, tau, fmax, mu, air, ge 升序,\n\
         # shape=false<true, h_jump 升序) 全序,取首/中/尾。\n\
         # R1:摩擦 Min 合成,μ_eff = min(μ, 对方);合成后 μ=0 → 0。\n",
    );
    for (tag, p) in cands {
        s.push_str(&format!(
            "{tag} tau={} fmax={} mu={} air={} hj={} ge={} mode={}\n",
            p.tau,
            p.fmax_mul,
            p.mu,
            p.air,
            p.h_jump,
            p.ground_eps,
            if p.shape_cast { "shape" } else { "ray" },
        ));
    }
    std::fs::write(path, s).expect("write cands");
}

fn parse_cands(path: &str) -> Vec<(String, Params)> {
    let mut out = Vec::new();
    for line in std::fs::read_to_string(path).expect("read cands").lines() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let mut it = line.split_whitespace();
        let tag = it.next().expect("tag").to_string();
        let mut p = Params {
            tau: 0.0,
            fmax_mul: 0.0,
            mu: 0.0,
            air: 0.0,
            h_jump: 0.0,
            ground_eps: 0.0,
            shape_cast: false,
        };
        for kv in it {
            let (k, v) = kv.split_once('=').expect("kv");
            match k {
                "tau" => p.tau = v.parse().expect("float"),
                "fmax" => p.fmax_mul = v.parse().expect("float"),
                "mu" => p.mu = v.parse().expect("float"),
                "air" => p.air = v.parse().expect("float"),
                "hj" => p.h_jump = v.parse().expect("float"),
                "ge" => p.ground_eps = v.parse().expect("float"),
                "mode" => p.shape_cast = v == "shape",
                _ => panic!("unknown cand key {k}"),
            }
        }
        out.push((tag, p));
    }
    out
}

fn run_sweep() {
    let mut rows: Vec<SweepRow> = Vec::new();
    let mut aggs: Vec<TupleAgg> = Vec::new();
    let mut jumps: Vec<JumpAgg> = Vec::new();

    // 阶段 A:平地(T0)行走 648 元组 × 3 速度,heading 0。
    for &tau in &TAUS {
        for &fmax in &FMAXS {
            for &mu in &MUS {
                for &air in &AIRS {
                    for &ge in &GES {
                        for &shape in &[false, true] {
                            let p = Params {
                                tau,
                                fmax_mul: fmax,
                                mu,
                                air,
                                h_jump: 1.25,
                                ground_eps: ge,
                                shape_cast: shape,
                            };
                            let tuple = p.tuple_tag();
                            let mut err_max = 0.0f32;
                            let mut err_by_speed = [0.0f32; 3];
                            let mut any_nan = false;
                            let mut all_pass = true;
                            let mut all_vy_ok = true;
                            for (si, &speed) in WALK_SPEEDS.iter().enumerate() {
                                let name = format!("swA_s{}_{}", fp(speed), tuple);
                                let cfg = Cfg {
                                    name: name.clone(),
                                    scene: Scene::Walk {
                                        t1: false,
                                        speed,
                                        heading: 0,
                                        sweep_a: true,
                                    },
                                    p,
                                    v_guard: None,
                                };
                                let o = run(&cfg);
                                let err = o.speed_err_pct.unwrap_or(100.0);
                                err_max = err_max.max(err);
                                err_by_speed[si] = err;
                                any_nan |= o.nan || o.panicked;
                                let row_pass = err <= 5.0 && o.vy_peak <= 0.5 && !o.nan;
                                all_pass &= row_pass;
                                all_vy_ok &= o.vy_peak <= 0.5;
                                rows.push(SweepRow {
                                    pass: "-".into(), // 元组级 flag 收齐后回填
                                    name,
                                    stage: "A",
                                    speed_target: format!("{speed:.1}"),
                                    v_mean: opt_f(o.v_mean),
                                    speed_err_pct: opt_f(o.speed_err_pct),
                                    t90_ms: opt_f(o.t90_ms),
                                    vy_peak: format!("{:.4}", o.vy_peak),
                                    step_ok: "-".into(),
                                    step_n: "-".into(),
                                    nan: o.nan || o.panicked,
                                    state_hash: if o.panicked {
                                        "PANIC".into()
                                    } else {
                                        format!("{:016x}", o.state_hash)
                                    },
                                });
                            }
                            // 回退层(README 同文):4.3/5.6 档 ≤5% 且 vy OK。
                            let pass_hi = err_by_speed[1] <= 5.0
                                && err_by_speed[2] <= 5.0
                                && all_vy_ok
                                && !any_nan;
                            aggs.push(TupleAgg {
                                p,
                                err_max,
                                err_by_speed,
                                pass_a: all_pass && !any_nan,
                                pass_hi,
                            });
                            // 回填元组级 pass。
                            let back = if all_pass && !any_nan { "A" } else { "-" };
                            let tail = rows.len();
                            for r in &mut rows[tail - WALK_SPEEDS.len()..] {
                                r.pass = back.into();
                            }
                        }
                    }
                }
            }
        }
    }

    // 阶段 B:1 格台阶跑跳 d=1.0(表面间隙口径,R1/CR5),3 次微扰,
    // 对 passA 元组 × 3 档 h_jump;passA 为空时对 pass_hi 元组跑(README 写明)。
    // (d=1.0 保留:Min 合成后按跳判定更可靠,但跑跳可用距离仍由滞空决定。)
    let b_pool: Vec<&TupleAgg> = if aggs.iter().any(|t| t.pass_a) {
        aggs.iter().filter(|t| t.pass_a).collect()
    } else {
        aggs.iter().filter(|t| t.pass_hi).collect()
    };
    for agg in &b_pool {
        if !agg.pass_a && !agg.pass_hi {
            continue;
        }
        for &hj in &HJUMPS {
            let mut p = agg.p;
            p.h_jump = hj;
            let tuple = p.tuple_tag();
            let mut ok = 0u32;
            let n = 3u32;
            let first_row = rows.len();
            for trial in 0..3usize {
                let name = format!("swB_hj{}_{}_t{}", fp(hj), tuple, trial);
                let cfg = Cfg {
                    name: name.clone(),
                    scene: Scene::Step {
                        height: 1,
                        run: true,
                        d: 1.0,
                        trial,
                    },
                    p,
                    v_guard: None,
                };
                let o = run(&cfg);
                ok += u32::from(o.step_success == Some(true));
                rows.push(SweepRow {
                    pass: "-".into(),
                    name,
                    stage: "B",
                    speed_target: "-".into(),
                    v_mean: "-".into(),
                    speed_err_pct: "-".into(),
                    t90_ms: "-".into(),
                    vy_peak: format!("{:.4}", o.vy_peak),
                    step_ok: "-".into(),
                    step_n: "-".into(),
                    nan: o.nan || o.panicked,
                    state_hash: if o.panicked {
                        "PANIC".into()
                    } else {
                        format!("{:016x}", o.state_hash)
                    },
                });
            }
            let pass = ok == n;
            for r in &mut rows[first_row..] {
                r.step_ok = ok.to_string();
                r.step_n = n.to_string();
                r.pass = if pass { "B" } else { "-" }.into();
            }
            let _ = first_row;
            jumps.push(JumpAgg { p, ok, n });
        }
    }

    let cands = select_candidates(&aggs, &jumps);
    let _ = std::fs::create_dir_all("out");
    let mut csv = String::from(SWEEP_HEADER);
    csv.push('\n');
    for r in &rows {
        csv.push_str(&sweep_row_csv(r));
        csv.push('\n');
    }
    std::fs::write("out/sweep_raw.csv", &csv).expect("write sweep");
    write_cands("out/cands.txt", &cands);
    println!("sweep: {} rows; candidates:", rows.len());
    for (tag, p) in &cands {
        println!(
            "{tag} tau={} fmax={} mu={} air={} hj={} ge={} mode={}",
            p.tau,
            p.fmax_mul,
            p.mu,
            p.air,
            p.h_jump,
            p.ground_eps,
            if p.shape_cast { "shape" } else { "ray" },
        );
    }
}

// ── 全矩阵 ──────────────────────────────────────────────────────────────

fn build_matrix(cands: &[(String, Params)]) -> Vec<Cfg> {
    let mut out = Vec::new();
    for (tag, p) in cands {
        // 1 平地行走:3 速度 × 8 航向 × T0/T1。
        for t1 in [false, true] {
            for &speed in &WALK_SPEEDS {
                for h in 0..8usize {
                    out.push(Cfg {
                        name: format!(
                            "walk_{}_s{}_h{}_{}",
                            if t1 { "T1" } else { "T0" },
                            fp(speed),
                            h,
                            tag
                        ),
                        scene: Scene::Walk {
                            t1,
                            speed,
                            heading: h,
                            sweep_a: false,
                        },
                        p: *p,
                        v_guard: None,
                    });
                }
            }
        }
        // 2 站立:T0/T1。
        for t1 in [false, true] {
            out.push(Cfg {
                name: format!("stand_{}_{}", if t1 { "T1" } else { "T0" }, tag),
                scene: Scene::Stand { t1 },
                p: *p,
                v_guard: None,
            });
        }
        // 3 台阶:1/2 格 × 跑动/静止 × d ∈ {0.3,0.6,1.0} × 10 次微扰。
        for height in [1, 2] {
            for run in [true, false] {
                for &d in &STEP_D {
                    for t in 0..10usize {
                        out.push(Cfg {
                            name: format!(
                                "step_{}{}_{}_t{}_{}",
                                height,
                                if run { "run" } else { "stand" },
                                fd(d),
                                t,
                                tag
                            ),
                            scene: Scene::Step {
                                height,
                                run,
                                d,
                                trial: t,
                            },
                            p: *p,
                            v_guard: None,
                        });
                    }
                }
            }
        }
        // 4 空中连按跳 + 单次对照。
        for (nm, rep) in [("airjump_single", false), ("airjump_repeat", true)] {
            out.push(Cfg {
                name: format!("{nm}_{tag}"),
                scene: Scene::AirJump { repeat: rep },
                p: *p,
                v_guard: None,
            });
        }
        // 5 接缝(R1/CR7):三档速度 × 8 航向 × edgefix 开/关;
        // 判据 5 按 fix1 行判,fix0 行只作对照。
        for &spd in &SEAM_SPEEDS {
            for h in 0..8usize {
                for (fix, fx) in [(EdgeFix::Fix1, "fix1"), (EdgeFix::Fix0, "fix0")] {
                    out.push(Cfg {
                        name: format!("seam_s{}_h{}_{}_{}", fp(spd), h, fx, tag),
                        scene: Scene::Seam {
                            speed: spd,
                            heading: h,
                            fix,
                        },
                        p: *p,
                        v_guard: None,
                    });
                }
            }
        }
        // 6 方块堆(3×3×1、3×3×2,睡着)+ 推静止单格。
        for layers in [1, 2] {
            out.push(Cfg {
                name: format!("pile_33{}_{}", layers, tag),
                scene: Scene::Pile { layers },
                p: *p,
                v_guard: None,
            });
        }
        out.push(Cfg {
            name: format!("push_one_{tag}"),
            scene: Scene::Push,
            p: *p,
            v_guard: None,
        });
        // 7 被砸:25 落点 × 5 高度微扰。
        for &ox in &HIT_GRID {
            for &oz in &HIT_GRID {
                for &d in &D_GRID {
                    out.push(Cfg {
                        name: format!("hit_ox{}_oz{}_d{}_{}", f3(ox), f3(oz), f3(d), tag),
                        scene: Scene::Hit { ox, oz, d },
                        p: *p,
                        v_guard: None,
                    });
                }
            }
        }
        // 8 重物:质量 8/64/512,9 落点 × 5 微扰,无防护 + V_GUARD=200 重跑。
        for h in [1.0f32, 2.0, 4.0] {
            for &ox in &HEAVY_GRID {
                for &oz in &HEAVY_GRID {
                    for &d in &D_GRID {
                        for vg in [None, Some(V_GUARD)] {
                            let vg_seg = if vg.is_some() { "_vg200" } else { "" };
                            out.push(Cfg {
                                name: format!(
                                    "heavy_m{}_ox{}_oz{}_d{}{}_{}",
                                    (8.0 * h * h * h) as i32,
                                    f3(ox),
                                    f3(oz),
                                    f3(d),
                                    vg_seg,
                                    tag
                                ),
                                scene: Scene::Heavy { h, ox, oz, d },
                                p: *p,
                                v_guard: vg,
                            });
                        }
                    }
                }
            }
        }
        // 9 连杆顺序对照:16 个先插入的会睡着刚体(两种实现),4.3 m/s × 8 航向。
        for ball in [false, true] {
            for h in 0..8usize {
                out.push(Cfg {
                    name: format!("order{}_h{}_{}", if ball { "16ball" } else { "16" }, h, tag),
                    scene: Scene::Order { ball, heading: h },
                    p: *p,
                    v_guard: None,
                });
            }
        }
    }
    out
}

fn run_matrix(cands: &[(String, Params)]) {
    let all = build_matrix(cands);
    let _ = std::fs::create_dir_all("out");
    let mut csv = String::from(MATRIX_HEADER);
    csv.push('\n');
    for cfg in &all {
        #[allow(clippy::disallowed_methods)] // demo 自身要报每行耗时;纪律自束(同 terrain/gait)
        let t0 = Instant::now();
        let o = run(cfg);
        // R1/CR4:推荐参数的行走行不允许走出域(域扩过就断言住)。
        if cfg.scene.t1_domain() && matches!(cfg.scene, Scene::Walk { .. }) {
            assert!(
                o.end_reason != "out_of_domain",
                "walk row {} left T1 domain",
                o.name
            );
        }
        println!("{} | {:.1}s", outcome_csv(&o), t0.elapsed().as_secs_f32());
        csv.push_str(&outcome_csv(&o));
        csv.push('\n');
    }
    std::fs::write("out/matrix_run.csv", &csv).expect("write matrix");
}

fn run_det(cands: &[(String, Params)]) {
    let all = build_matrix(cands);
    let _ = std::fs::create_dir_all("out");
    let mut txt = String::new();
    for cfg in &all {
        let o1 = run(cfg);
        let o2 = run(cfg);
        let line = if o1.panicked || o2.panicked {
            format!(
                "{} {} vs {} -> {}",
                cfg.name,
                if o1.panicked { "PANIC" } else { "hash" },
                if o2.panicked { "PANIC" } else { "hash" },
                if o1.panicked && o2.panicked {
                    "PANIC_BOTH"
                } else {
                    "PANIC_ONE"
                },
            )
        } else {
            format!(
                "{} {:016x} vs {:016x} -> {}",
                cfg.name,
                o1.state_hash,
                o2.state_hash,
                if o1.state_hash == o2.state_hash {
                    "IDENTICAL"
                } else {
                    "DIFFER"
                }
            )
        };
        println!("{line}");
        txt.push_str(&line);
        txt.push('\n');
    }
    std::fs::write("out/det_run.txt", &txt).expect("write det");
}

/// 冒烟:固定参数跑一小组,验证布景与统计接线(数据不用于结论)。
fn run_smoke() {
    let p = Params {
        tau: 0.1,
        fmax_mul: 4.0,
        mu: 0.3,
        air: 0.2,
        h_jump: 1.25,
        ground_eps: 0.02,
        shape_cast: false,
    };
    let cands = vec![("c1".to_string(), p)];
    let all = build_matrix(&cands);
    let keep = [
        "walk_T0_s4p3_h0",
        "walk_T1_s4p3_h0",
        "stand_T0",
        "airjump_single",
        "airjump_repeat",
        "step_1run_0p6_t0",
        "step_1stand_0p6_t0",
        "step_2run_0p6_t0",
        "seam_s4p3_h0_fix1",
        "seam_s4p3_h0_fix0",
        "pile_331",
        "push_one",
        "hit_ox0_oz0_d0",
        "heavy_m8_ox0_oz0_d0_",
        "heavy_m512_ox0_oz0_d0_vg200",
        "order16_h0",
    ];
    let _ = std::fs::create_dir_all("out");
    let mut csv = String::from(MATRIX_HEADER);
    csv.push('\n');
    for cfg in &all {
        if !keep.iter().any(|k| cfg.name.starts_with(k)) {
            continue;
        }
        let o = run(cfg);
        println!("{}", outcome_csv(&o));
        csv.push_str(&outcome_csv(&o));
        csv.push('\n');
    }
    std::fs::write("out/smoke.csv", &csv).expect("write smoke");
}

const MATRIX_HEADER: &str = "name,end_tick,end_reason,nan,panicked,guard_removed,panic_msg,\
     speed_target,v_mean,speed_err_pct,t90_ms,stop_dist_m,stop_end_v,vy_peak,vy_peak_settle,\
     ground_frac,edge_snaps,pile_land_pen,pile_land_vy,pen_min,\
     pen_obj_min,drift_m,sink_m,sleep_ratio,slept,jump_max_h,bounce_vy,hit_disp_h,\
     speed_max,block_disp_max,blocks_woken,dip_pct,stuck,step_success,course_err_deg,\
     state_hash";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mode = args.first().map(String::as_str).unwrap_or("");
    // 预期内的重物 panic:行内已捕获 payload 并写入 panic 行,免刷屏。
    std::panic::set_hook(Box::new(|info| {
        eprintln!("PANIC: {info}");
    }));
    match mode {
        "--list" => {
            println!("modes: sweep | matrix | det | smoke");
            println!("sweep: 平地 648 元组 × 3 速度 + 1 格台阶 hj 扫描 → out/sweep_raw.csv + out/cands.txt");
            println!("matrix: 读 out/cands.txt 跑全部场景 → out/matrix_run.csv");
            println!("det: 全矩阵双跑逐行哈希 → out/det_run.txt");
        }
        "sweep" => run_sweep(),
        "matrix" => {
            let cands = parse_cands("out/cands.txt");
            run_matrix(&cands);
        }
        "det" => {
            let cands = parse_cands("out/cands.txt");
            run_det(&cands);
        }
        "smoke" => run_smoke(),
        _ => {
            println!("usage: capsule [--list|sweep|matrix|det|smoke]");
            std::process::exit(if mode.is_empty() { 0 } else { 1 });
        }
    }
}
