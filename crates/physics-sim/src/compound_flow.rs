//! compound_flow —— 自由复合体的惯性汇点冲量流(REQ-010 §5.8,修订 12;
//! 单元 G01 纯函数)。生产接入(采样、分派、配置键、遥测)在单元 G02。
//!
//! 与 `impact_flow`(§5.7.1)同一套网络与容量,汇点换成每格随刚体运动所需的
//! 惯性容量 `d_i = m_i·max(0, Δv_i·d̂)`(§5.8.1,D16),动量分配与 §5.7.2 同构
//! (§5.8.2,D17):R 扮演地基,超出冲量 `E = ρ·J` 退还给 R、修正给对方、
//! 余下给 S。
//!
//! # 坐标约定(整个模块只有这一套坐标系)
//!
//! **B 的刚体局部系**:格心 = `BlockPos + 0.5`(相对刚体原点的偏移;复合体
//! 刚体原点在形状的中格,即 `cohesion::compound_origin` 的语义)。接触点的
//! 局部系作用点、指向 B 内部的局部系单位法向,由**调用方**完成世界 ↔ 局部
//! 换算(含刚体与对方 collider 的旋转;§5.8.1"事件的坐标系"条)——本模块
//! 不做任何世界坐标换算,G00 前两轮的错误结论都来自世界坐标与局部格心混用。
//!
//! # 输入
//!
//! 一个事件(§5.8.1):B 的格与材料、计入的接触点(所在格、法向冲量、局部
//! 作用点、局部单位法向)、B 步进后的刚体速度场(v、ω)、可选的 §5.7 撞击者
//! 修正冲量(从 J 中扣除)与各动态对方的 `m_eff,o` 与接触冲量份额。
//!
//! # 复用
//!
//! `cohesion::Dinic`(最小割与算法无关)、`cohesion::impact_bond_between` /
//! `impact_crush_between`(粘结容量)、`impact_flow` 的量化与惯量工具
//! (`quant_supply`/`quant_cap`/`component_center_of_mass`/
//! `component_inv_inertia`/`inertia_mul`)、`min_impact`(整化口径与预筛阈值
//! 一致)。

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use types::glam::Vec3;
use types::{BlockId, BlockPos};
use voxel::BlockRegistry;

use crate::cohesion::{Dinic, impact_bond_between, impact_crush_between};
use crate::impact_flow::{
    component_center_of_mass, component_inertia_tensor, component_inv_inertia, inertia_mul,
    invert_symmetric_3x3_f64, min_impact, quant_cap,
};

/// 一格的质量(密度 1 × (2·0.45)³;与 `impact_flow` 的每格质量一致——该常量
/// 私有,此处独立声明、语义由测试钉住)。
const CELL_MASS: f32 = 0.729;

/// 量化单位(N·s;与 `impact_flow::QUANT` 一致——该常量 pub(crate) 但语义
/// 属 §5.7,本模块自带字面量、由测试钉住一致性)。
const QUANT: f32 = 1000.0;

/// 6 邻居的偏移(与承载判定、impact_flow 同序;压向判定用)。
const NEIGHBOR_DELTAS: [(i32, i32, i32); 6] = [
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
];

/// 压向判定阈值:`u·d̂ ≥ cos 45°`(u = a 指向 b 的局部轴向量)→ 压向(§5.8.1)。
const COS_PRESS: f32 = std::f32::consts::FRAC_1_SQRT_2;

/// 一个计入的接触点(§5.8.1 事件;**B 的刚体局部系**)。
#[derive(Debug, Clone, PartialEq)]
pub struct ContactPoint {
    /// 点所在的本体格(至少有一个点的格 = 接触格 C)。
    pub cell: BlockPos,
    /// 法向冲量 `j_k ≥ 0`。
    pub impulse: f32,
    /// 局部系作用点(格面点,调用方换算)。
    pub point: Vec3,
    /// 指向 B 内部的局部系**单位**法向 `d_k`(调用方保证归一)。
    pub normal: Vec3,
}

/// 一个复合体冲量流事件(§5.8.1;**B 的刚体局部系**)。
#[derive(Debug, Clone, PartialEq)]
pub struct CompoundEvent {
    /// B 的全部格(局部 `BlockPos`)与方块(求粘结容量;去重由调用方保证)。
    pub cells: Vec<(BlockPos, BlockId)>,
    /// 计入的接触点(勘误 2 口径的采样结果)。
    pub contacts: Vec<ContactPoint>,
    /// B 步进后的刚体线速度(局部系)。
    pub linvel: Vec3,
    /// B 步进后的刚体角速度(局部系)。
    pub angvel: Vec3,
    /// §5.7 撞击者修正冲量(B 是 §5.7 撞击者且本拍已被修正时给出;§5.8.1:
    /// `J` 减去该修正。局部系)。
    pub correction: Vec3,
}

impl CompoundEvent {
    /// 冲量矢量 `J = Σ j_k·d_k − correction`(局部系)。
    pub fn impulse_j(&self) -> Vec3 {
        let mut j = Vec3::ZERO;
        for pt in &self.contacts {
            j += pt.normal * pt.impulse;
        }
        j - self.correction
    }

    /// 作用点 `p` = 计入接触点按冲量加权的质心(局部系)。
    pub fn weighted_point(&self) -> Vec3 {
        let mut acc = Vec3::ZERO;
        let mut w = 0.0_f32;
        for pt in &self.contacts {
            w += pt.impulse;
            acc += pt.point * pt.impulse;
        }
        if w > 0.0 { acc / w } else { Vec3::ZERO }
    }

    /// `d̂ = normalize(J)`;`|J| = 0` 视为无事件(调用方不建网)。
    pub fn dir_hat(&self) -> Vec3 {
        self.impulse_j().normalize_or_zero()
    }

    /// 供给合计 `s = |J|`(§5.8.1 用矢量和,与 §5.7 的 Σs_c 标量和不同口径)。
    pub fn total_impulse(&self) -> f32 {
        self.impulse_j().length()
    }
}

/// B 的质量属性(§5.8.1 刚体响应;局部系)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyProps {
    /// 质量 `M`(格数 × 0.729)。
    pub mass: f32,
    /// 质心 `c`(局部系)。
    pub center_of_mass: Vec3,
    /// 完整惯量张量 `I`(3×3 行主序,**含惯性积**;平行轴定理,逐格 cuboid
    /// 半边长 0.45、密度 1;单格自身惯量 `m(2h)²/6`)。
    pub inertia: [[f32; 3]; 3],
}

/// B 的质量属性(局部系)。惯量单一来源:`component_inertia_tensor`(§5.7.2
/// 的张量构造;`base` = `m/12·(4h²+4h²)` = 立方格 `m(a²+b²)/12` =
/// `m(2h)²/6`——写成 /12 会把 Δω 放大 2 倍,G00 复核)。
pub fn body_props(cells: &[BlockPos]) -> BodyProps {
    BodyProps {
        mass: cells.len() as f32 * CELL_MASS,
        center_of_mass: component_center_of_mass(cells),
        inertia: component_inertia_tensor(cells),
    }
}

/// 网络求值内部的量化辅助:f64 域算再走 §5.7.1 的 floor/ceil 口径。
/// 供给 `s_c`、汇容量 `d_i` 在 216 格满载荷下可达 1e5 N·s × 1000 =
/// 1e8 量子,f32(24 位尾数,整数精度上限 1.678e7)逐格舍入会破
/// `Σ d_i ≥ s_q` 的整数域不变量(平落 1e5 曾因此 S = 全体)——刚体响应的
/// 逐格 `Δω × arm` 累计漂移 ~10 量子,与逐格 floor 的供给损失在边界上
/// 相互翻转。f64 域计算与 §5.8.1"整化同 §5.7.1"不冲突(口径仍是
/// 0.001 N·s 的 floor/ceil;§5.7 的冲量量级小 ~3 个数量级,不受影响)。
fn quant_supply_f64(s: f64) -> u64 {
    if s <= 0.0 {
        0
    } else {
        (s * 1000.0).floor() as u64
    }
}

/// 容量量化(f64 入参;口径同 `impact_flow::quant_cap`,∞ 哨兵同值)。
fn quant_cap_f64(c: f64) -> u64 {
    if c.is_infinite() || c >= 4.0e18 {
        INF_CAP_SENTINEL
    } else if c <= 0.0 {
        0
    } else {
        ((c * 1000.0).ceil() as u64).min(INF_CAP_SENTINEL)
    }
}

/// 容量 ∞ 哨兵(与 `impact_flow::INF_CAP` 同值;该常量私有,此处独立声明、
/// 由穷举割测试对拍钉住)。
const INF_CAP_SENTINEL: u64 = u64::MAX / 4;

/// 刚体响应(§5.8.1):返回 `(Δω, Δv_i)`,
/// `Δω = I⁻¹((p − c) × J)`、`Δv_i = J/M + Δω × (x_i − c)`(局部系;
/// `x_i` = 格心 `BlockPos + 0.5`)。`J` 为零矢量时返回零响应。
pub fn rigid_response(
    props: &BodyProps,
    cells: &[BlockPos],
    point: Vec3,
    impulse: Vec3,
) -> (Vec3, BTreeMap<BlockPos, Vec3>) {
    let mut dv: BTreeMap<BlockPos, Vec3> = BTreeMap::new();
    if impulse == Vec3::ZERO {
        for &c in cells {
            dv.insert(c, Vec3::ZERO);
        }
        return (Vec3::ZERO, dv);
    }
    let r = point - props.center_of_mass;
    let domega = inertia_mul(&component_inv_inertia(cells), r.cross(impulse));
    let v_com = impulse / props.mass;
    for &c in cells {
        let arm =
            Vec3::new(c.x as f32 + 0.5, c.y as f32 + 0.5, c.z as f32 + 0.5) - props.center_of_mass;
        dv.insert(c, v_com + domega.cross(arm));
    }
    (domega, dv)
}

/// 网络判定结果(§5.8.1;字段齐全)。
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CompoundFlowOutcome {
    /// 供给合计的量化值(接触格均摊 `s/|C|` 向下取整之和;预筛命中或无事件
    /// 时为 0)。
    pub supply_q: u64,
    /// 最大流 `C`(量化单位;预筛命中时不建网,为 0)。
    pub cut_q: u64,
    /// S:残量网络中源点可达的格(`BlockPos` 升序;扛住为空)。
    pub detached: Vec<BlockPos>,
    /// S 按 `I > 0` 共面连通划分的分量 S_k(分量内升序,各分量按首格升序;
    /// 每个分量必含接触格,§5.8.2)。
    pub s_components: Vec<Vec<BlockPos>>,
    /// 被切断的粘结数(恰一端在 S、`impact_bond_between > 0` 的共面连接)。
    pub cut_bonds: usize,
    /// 预筛命中:`s < I_min` 不建网(§5.8.1)。
    pub prefiltered: bool,
}

/// R 的一个连通分量及其断面(§5.8.2 动量分配的 R_j)。
#[derive(Debug, Clone, PartialEq)]
pub struct RComponent {
    /// 分量的格(`BlockPos` 升序)。
    pub cells: Vec<BlockPos>,
    /// 断面容量 `T_j`(R_j 一侧被割粘结的容量和;合成校验里可为 0)。
    pub cut_capacity: f32,
    /// 断面质心 `q_j`(被割粘结两格中心连线中点按容量加权;ΣT = 0 时取
    /// 分量质心)。
    pub cut_point: Vec3,
}

/// 各动态对方的分配输入(§5.8.2:对方修正)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OtherBody {
    /// `m_eff,o`:对方在接触点沿 d̂ 的等效质量(§5.7.2 公式;调用方算,
    /// 保持本模块不依赖对方刚体)。
    pub effective_mass: f32,
    /// 对方的接触冲量占 J 的比例(单对方 = 1;多点对多体时调用方按
    /// `Σ j_k(对方 o)/s` 拆)。
    pub impulse_share: f32,
}

/// 动量分配结果(§5.8.2;字段齐全,守恒式对账用)。
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MomentumSplit {
    /// `ρ = (s − C)/s`(浮点;扛住时 0)。
    pub rho: f32,
    /// 超出冲量 `E = ρ·J`。
    pub excess: Vec3,
    /// R 的各连通分量(R_j;扛住时为空)。
    pub r_components: Vec<RComponent>,
    /// R_j 退还 `−(T_j/ΣT)·E`(ΣT = 0 时按质量比),作用于各自的 q_j;
    /// 与 `r_components` 一一对应。
    pub r_refunds: Vec<(Vec3, Vec3)>,
    /// 对方修正:`(m_eff,o/(m_eff,o+M_S))·E_o`(§5.8.2;地形份额无修正)。
    /// 与输入 `others` 同序。
    pub other_corrections: Vec<Vec3>,
    /// S_k 获得的冲量 `M_k/M_S × (S 应得部分)`,与 `outcome.s_components`
    /// 一一对应。
    pub s_impulses: Vec<Vec3>,
    /// S_k 的作用点 `p_k`(分量内接触点冲量加权质心;无接触点取事件 p)。
    pub s_points: Vec<Vec3>,
    /// 守恒残差 `|Σ R 退还 + Σ 对方修正 + Σ S 获得|`(量化单位内应 ≈ 0)。
    pub residual: f32,
}

/// `I > 0` 共面连通分量(§5.8.2;`cells` 去重,输出分量内升序、按首格升序)。
fn bond_components(
    cells: &[BlockPos],
    block_of: &BTreeMap<BlockPos, BlockId>,
    registry: &dyn BlockRegistry,
) -> Vec<Vec<BlockPos>> {
    let unique: BTreeSet<BlockPos> = cells.iter().copied().collect();
    let mut comps: Vec<Vec<BlockPos>> = Vec::new();
    let mut seen: BTreeSet<BlockPos> = BTreeSet::new();
    for &start in &unique {
        if seen.contains(&start) {
            continue;
        }
        let mut comp = vec![start];
        let mut q = VecDeque::from([start]);
        seen.insert(start);
        while let Some(a) = q.pop_front() {
            for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
                let b = BlockPos::new(a.x + dx, a.y + dy, a.z + dz);
                if !unique.contains(&b) || seen.contains(&b) {
                    continue;
                }
                let bonded = impact_bond_between(registry, block_of[&a], block_of[&b]) > 0.0;
                if bonded {
                    seen.insert(b);
                    comp.push(b);
                    q.push_back(b);
                }
            }
        }
        comp.sort();
        comps.push(comp);
    }
    comps
}

/// `side` 与 `other` 之间被割粘结的容量和与质心(§5.8.2 的 T_j 与 q_j)。
/// 容量按撞击粘结 `I(a,b)`(断面与方向无关,§5.7.2 同);`q` = 被割粘结
/// 两格中心连线中点按容量加权;无被割粘结时容量 0、质心取 `side` 自身质心。
fn cut_face(
    side: &[BlockPos],
    other: &BTreeSet<BlockPos>,
    block_of: &BTreeMap<BlockPos, BlockId>,
    registry: &dyn BlockRegistry,
) -> (f32, Vec3) {
    let side_set: BTreeSet<BlockPos> = side.iter().copied().collect();
    let mut t = 0.0_f32;
    let mut acc = Vec3::ZERO;
    for &a in &side_set {
        for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
            let b = BlockPos::new(a.x + dx, a.y + dy, a.z + dz);
            if !other.contains(&b) {
                continue;
            }
            let cap = impact_bond_between(registry, block_of[&a], block_of[&b]);
            if cap > 0.0 {
                let ca = Vec3::new(a.x as f32 + 0.5, a.y as f32 + 0.5, a.z as f32 + 0.5);
                let cb = Vec3::new(b.x as f32 + 0.5, b.y as f32 + 0.5, b.z as f32 + 0.5);
                t += cap;
                acc += (ca + cb) * (0.5 * cap);
            }
        }
    }
    if t > 0.0 {
        (t, acc / t)
    } else {
        (0.0, component_center_of_mass(side))
    }
}

/// 被切断的粘结数(恰一端在 S、`impact_bond_between > 0`)。
fn cut_bond_count(
    detached: &[BlockPos],
    cellset: &BTreeSet<BlockPos>,
    block_of: &BTreeMap<BlockPos, BlockId>,
    registry: &dyn BlockRegistry,
) -> usize {
    let s_set: BTreeSet<BlockPos> = detached.iter().copied().collect();
    let mut n = 0;
    for &a in &s_set {
        for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
            let b = BlockPos::new(a.x + dx, a.y + dy, a.z + dz);
            if cellset.contains(&b)
                && !s_set.contains(&b)
                && impact_bond_between(registry, block_of[&a], block_of[&b]) > 0.0
            {
                n += 1;
            }
        }
    }
    n
}

/// 复合体冲量流(§5.8.1;纯函数,一个事件单独建网):
///
/// - **J、p、d̂、s**:`J = Σ j_k·d_k − correction`、`p` = 接触点冲量加权质心、
///   `s = |J|`(矢量和口径);接触格 C = 至少有一个计入点的格,供给均摊
///   `s_c = s / |C|`;
/// - **预筛**:`s < I_min` 不建网(`prefiltered`,同 §5.7.1);`|J| = 0` 或
///   无接触点视为无事件;
/// - **源边**:源点 → 接触格,容量 `s_c` 向下取整;
/// - **格间边**(B 的格之间,局部系相邻):`u·d̂ ≥ cos45°` → 压向
///   `I_c(a,b)`,否则 `I(a,b)`;容量向上取整;
/// - **汇边**:`d_i = m_i·max(0, Δv_i·d̂)`(刚体响应;向上取整)——只切汇点
///   边的割 ≥ s(`Σ d_i ≥ s`),`C < s` 只能来自粘结不足,S 永不为全体;
/// - **判定在整数域**:`C ≥ s_q` 扛住;否则 S = 残量网络中源点可达的格
///   (最小源侧,唯一);
/// - **数值兜底**:纯平移时 Σd_i 与 s 在实数域恰好相等,超大冲量(平落
///   ≥ 1e5 N·s)下 `J/M` 的浮点舍入与逐格 floor 的供给损失是否破整数域
///   不变量是刀刃。因此**求值在 f64 域进行**(J、p、s、质心、刚体响应的
///   Δω/Δv_i、点积与量化输入全部 f64;惯量张量取 `component_inertia_tensor`
///   转 f64 后求逆),兜底之上,若可达集仍覆盖全部格,**按扛住处理**
///   (`detached` 为空)——R 为空的退化分割(把全部格断下来)没有物理意义,
///   扛住是保守侧;这是数值兜底,不是需求语义(需求语义由 f64 求值保证)。
/// - 节点 = B 的全部格(≤ `compound_split_size³`,无需区域预算)。
pub fn evaluate_compound_flow(
    registry: &dyn BlockRegistry,
    event: &CompoundEvent,
) -> CompoundFlowOutcome {
    let mut outcome = CompoundFlowOutcome::default();
    let j = event.impulse_j();
    if j == Vec3::ZERO || event.contacts.is_empty() || event.cells.is_empty() {
        return outcome;
    }
    // f64 域求值(见文档"数值兜底"):J、p、d̂、s 与刚体响应全程 f64。
    let mut j64 = [0.0_f64; 3];
    for pt in &event.contacts {
        j64[0] += f64::from(pt.normal.x) * f64::from(pt.impulse);
        j64[1] += f64::from(pt.normal.y) * f64::from(pt.impulse);
        j64[2] += f64::from(pt.normal.z) * f64::from(pt.impulse);
    }
    j64[0] -= f64::from(event.correction.x);
    j64[1] -= f64::from(event.correction.y);
    j64[2] -= f64::from(event.correction.z);
    let s = (j64[0] * j64[0] + j64[1] * j64[1] + j64[2] * j64[2]).sqrt();
    if s <= 0.0 {
        return outcome;
    }
    let d_hat = [j64[0] / s, j64[1] / s, j64[2] / s];
    let mut pw = [0.0_f64; 3];
    let mut w_total = 0.0_f64;
    for pt in &event.contacts {
        w_total += f64::from(pt.impulse);
        pw[0] += f64::from(pt.point.x) * f64::from(pt.impulse);
        pw[1] += f64::from(pt.point.y) * f64::from(pt.impulse);
        pw[2] += f64::from(pt.point.z) * f64::from(pt.impulse);
    }
    let point = if w_total > 0.0 {
        [pw[0] / w_total, pw[1] / w_total, pw[2] / w_total]
    } else {
        [0.0; 3]
    };
    let block_of: BTreeMap<BlockPos, BlockId> = event.cells.iter().copied().collect();
    let cells: Vec<BlockPos> = block_of.keys().copied().collect();
    let cellset: BTreeSet<BlockPos> = cells.iter().copied().collect();
    // 接触格 C(至少一个计入点;去重)。
    let mut contact_cells: BTreeSet<BlockPos> = BTreeSet::new();
    for pt in &event.contacts {
        debug_assert!(
            cellset.contains(&pt.cell),
            "接触点归格必须在 B 的格内(调用方口径)"
        );
        contact_cells.insert(pt.cell);
    }
    let s_each = s / contact_cells.len() as f64;
    // 预筛(§5.8.1;s < I_min 不建网;s 是非负有限 f64,直接比)。
    if s < f64::from(min_impact(registry)) {
        outcome.prefiltered = true;
        return outcome;
    }
    // 刚体响应(f64):M、c、I 全部由 B 的格算,局部系。
    let tensor = component_inertia_tensor(&cells);
    let tensor64: [[f64; 3]; 3] = tensor.map(|row| row.map(f64::from));
    let inv64 = invert_symmetric_3x3_f64(&tensor64);
    let com64 = {
        let c = component_center_of_mass(&cells);
        [f64::from(c.x), f64::from(c.y), f64::from(c.z)]
    };
    let mass64 = cells.len() as f64 * f64::from(CELL_MASS);
    let r = [
        point[0] - com64[0],
        point[1] - com64[1],
        point[2] - com64[2],
    ];
    let torque = [
        r[1] * j64[2] - r[2] * j64[1],
        r[2] * j64[0] - r[0] * j64[2],
        r[0] * j64[1] - r[1] * j64[0],
    ];
    let domega = [
        inv64[0][0] * torque[0] + inv64[0][1] * torque[1] + inv64[0][2] * torque[2],
        inv64[1][0] * torque[0] + inv64[1][1] * torque[1] + inv64[1][2] * torque[2],
        inv64[2][0] * torque[0] + inv64[2][1] * torque[1] + inv64[2][2] * torque[2],
    ];
    let v_com = [j64[0] / mass64, j64[1] / mass64, j64[2] / mass64];
    // 网络:节点 = 全部格 + 源 + 汇。
    let idx: BTreeMap<BlockPos, usize> =
        cells.iter().enumerate().map(|(i, c)| (*c, i + 2)).collect();
    let mut net = Dinic::new(cells.len() + 2);
    const SRC: usize = 0;
    const SNK: usize = 1;
    let mut supply_q = 0_u64;
    for &c in &cells {
        let uc = idx[&c];
        // 汇边:惯性容量 d_i = m_i·max(0, Δv_i·d̂)(§5.8.1;f64 域算再量化)。
        let x = [
            f64::from(c.x) + 0.5 - com64[0],
            f64::from(c.y) + 0.5 - com64[1],
            f64::from(c.z) + 0.5 - com64[2],
        ];
        let dvi = [
            v_com[0] + domega[1] * x[2] - domega[2] * x[1],
            v_com[1] + domega[2] * x[0] - domega[0] * x[2],
            v_com[2] + domega[0] * x[1] - domega[1] * x[0],
        ];
        let d_i = f64::from(CELL_MASS)
            * (dvi[0] * d_hat[0] + dvi[1] * d_hat[1] + dvi[2] * d_hat[2]).max(0.0);
        let cap = quant_cap_f64(d_i);
        if cap > 0 {
            net.edge(uc, SNK, cap);
        }
        // 源边:接触格的均摊供给(向下取整)。
        if contact_cells.contains(&c) {
            let q = quant_supply_f64(s_each);
            supply_q += q;
            if q > 0 {
                net.edge(SRC, uc, q);
            }
        }
        // 格间边:压向 / 剪拉(§5.8.1,与 §5.7.1 同)。
        let block_a = block_of[&c];
        for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
            let b = BlockPos::new(c.x + dx, c.y + dy, c.z + dz);
            if !cellset.contains(&b) {
                continue;
            }
            let u_dot = Vec3::new(dx as f32, dy as f32, dz as f32).dot(event.dir_hat());
            let block_b = block_of[&b];
            let cap_raw = if u_dot >= COS_PRESS {
                impact_crush_between(registry, block_a, block_b)
            } else {
                impact_bond_between(registry, block_a, block_b)
            };
            let cap = quant_cap(cap_raw);
            if cap > 0 {
                net.edge(uc, idx[&b], cap);
            }
        }
    }
    outcome.supply_q = supply_q;
    net.max_flow(SRC, SNK);
    outcome.cut_q = net.flow_value(SRC);
    let reach = net.reachable(SRC);
    if outcome.cut_q < outcome.supply_q {
        // C < s:残量网络中源点可达的格 S 断下(最小源侧;必有源边不饱和,
        // S 非空)。接触格恒在源侧。
        let detached: Vec<BlockPos> = cells.iter().filter(|c| reach[idx[*c]]).copied().collect();
        debug_assert!(!detached.is_empty(), "C < s 时 S 必非空");

        if detached.len() >= cells.len() {
            // 数值兜底(见文档):可达集覆盖全部格 = R 为空的退化分割,
            // 按扛住处理(保守侧)。f64 求值下正常事件不触发。
            outcome.detached.clear();
            outcome.s_components.clear();
            outcome.cut_bonds = 0;
            return outcome;
        }
        outcome.cut_bonds = cut_bond_count(&detached, &cellset, &block_of, registry);
        outcome.s_components = bond_components(&detached, &block_of, registry);
        outcome.detached = detached;
    }
    outcome
}

/// 动量分配(§5.8.2;纯函数)。`outcome` 须为"断开且未预筛"(`detached`
/// 非空),扛住时返回 `rho = 0` 的空结果(调用方按扛住处理)。
///
/// - `ρ = (s − C)/s`、`E = ρ·J`(浮点);
/// - **R 退还**:R 按 `I > 0` 连通分 R_j;`R_j` 在断面质心 `q_j` 受
///   `−(T_j/ΣT)·E`;ΣT = 0(合成校验)按 R_j 质量比;
/// - **对方修正**:各动态对方 `m_eff,o/(m_eff,o + M_S)·E_o`(份额由输入拆;
///   地形份额无修正,调用方不传);
/// - **S 获得**:S_k 按 `M_k/M_S` 分"S 应得部分"(`E − Σ 对方修正`),
///   作用于各自接触点质心 `p_k`(分量内无接触点时取事件 p);
/// - **守恒残差**:`|Σ R 退还 + Σ 对方修正 + Σ S 获得|`(量化单位内 ≈ 0)。
pub fn distribute_momentum(
    registry: &dyn BlockRegistry,
    event: &CompoundEvent,
    outcome: &CompoundFlowOutcome,
    others: &[OtherBody],
) -> MomentumSplit {
    let mut split = MomentumSplit::default();
    if outcome.prefiltered || outcome.detached.is_empty() {
        return split;
    }
    let j = event.impulse_j();
    let s = j.length();
    if s <= 0.0 {
        return split;
    }
    let c = outcome.cut_q as f32 / QUANT;
    let rho = ((s - c) / s).clamp(0.0, 1.0);
    let excess = j * rho;
    split.rho = rho;
    split.excess = excess;
    let block_of: BTreeMap<BlockPos, BlockId> = event.cells.iter().copied().collect();
    let cellset: BTreeSet<BlockPos> = event.cells.iter().map(|(c, _)| *c).collect();
    let s_set: BTreeSet<BlockPos> = outcome.detached.iter().copied().collect();
    let r_cells: Vec<BlockPos> = cellset.difference(&s_set).copied().collect();
    // R 分量与断面(T_j、q_j)。
    let r_comps = bond_components(&r_cells, &block_of, registry);
    let mut tot_t = 0.0_f32;
    let mut r_list: Vec<(Vec<BlockPos>, f32, Vec3)> = Vec::new();
    for comp in &r_comps {
        let (t, q) = cut_face(comp, &s_set, &block_of, registry);
        tot_t += t;
        r_list.push((comp.clone(), t, q));
    }
    let m_s: f32 = outcome.detached.len() as f32 * CELL_MASS;
    let r_mass: f32 = r_cells.len() as f32 * CELL_MASS;
    // S 应得部分 = E − Σ 对方修正。
    let mut corr_total = Vec3::ZERO;
    for o in others {
        let denom = o.effective_mass + m_s;
        let corr = if denom > 0.0 {
            excess * o.impulse_share * (o.effective_mass / denom)
        } else {
            Vec3::ZERO
        };
        split.other_corrections.push(corr);
        corr_total += corr;
    }
    let s_due = excess - corr_total;
    // R 退还(ΣT = 0 时按质量比;份额合计恒 1,总和恒 −E,§5.8.2)。
    let mut refund_total = Vec3::ZERO;
    for (comp, t, q) in &r_list {
        let share = if tot_t > 0.0 {
            t / tot_t
        } else {
            (comp.len() as f32 * CELL_MASS) / r_mass
        };
        let refund = excess * -share;
        refund_total += refund;
        split.r_components.push(RComponent {
            cells: comp.clone(),
            cut_capacity: *t,
            cut_point: *q,
        });
        split.r_refunds.push((*q, refund));
    }
    // S 获得(M_k/M_S × s_due;作用点 = 分量内接触点冲量加权质心)。
    let m_s_parts: f32 = outcome
        .s_components
        .iter()
        .map(|k| k.len() as f32 * CELL_MASS)
        .sum();
    for comp_k in &outcome.s_components {
        let m_k = comp_k.len() as f32 * CELL_MASS;
        let kset: BTreeSet<BlockPos> = comp_k.iter().copied().collect();
        let mut w = 0.0_f32;
        let mut acc = Vec3::ZERO;
        for pt in &event.contacts {
            if kset.contains(&pt.cell) {
                w += pt.impulse;
                acc += pt.point * pt.impulse;
            }
        }
        let p_k = if w > 0.0 {
            acc / w
        } else {
            event.weighted_point()
        };
        split.s_impulses.push(s_due * (m_k / m_s_parts));
        split.s_points.push(p_k);
    }
    // 守恒残差(§5.8.2 验收式)。
    let total = refund_total + corr_total + split.s_impulses.iter().copied().sum::<Vec3>();
    split.residual = total.length();
    split
}

/// 各块初速度(§5.8.2"各块速度"):B 步进后的刚体速度场(`v + ω × (质心_块 −
/// 质心_B)`,局部系)与 `ω`,再加上该块所受冲量 `P` 的刚体响应(`P/M_块`、
/// `I_块⁻¹(r × P)`,`r = p_块 − 质心_块`)——即多格块 `(场 + P/M_块,
/// ω + I_块⁻¹(r × P))`,**`P` 为零时同样保留 `ω`**(速度场与自旋都来自
/// B 步进后的状态,与冲量无关);`single_cell = true`(单格块)丢角速度
/// `(场 + P/m, 0)`——只丢 `ω`,**线冲量 `P/m` 保留**(丢了它 S 的 E 份额
/// 就整个消失,动量不守恒)。返回 `(v, ω)`。
pub fn piece_velocity(
    cells: &[BlockPos],
    body: &BodyProps,
    linvel: Vec3,
    angvel: Vec3,
    point: Vec3,
    impulse: Vec3,
    single_cell: bool,
) -> (Vec3, Vec3) {
    let com = component_center_of_mass(cells);
    let field = linvel + angvel.cross(com - body.center_of_mass);
    if single_cell {
        let m = cells.len() as f32 * CELL_MASS;
        return (field + impulse / m, Vec3::ZERO);
    }
    if impulse == Vec3::ZERO {
        return (field, angvel);
    }
    let own = body_props(cells);
    let r = point - com;
    let dw = inertia_mul(&component_inv_inertia(cells), r.cross(impulse));
    (field + impulse / own.mass, angvel + dw)
}

#[cfg(test)]
mod tests;
