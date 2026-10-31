//! impact_flow —— 撞击冲量流(REQ-010 §5.7,修订 11;单元 F01 纯函数)。
//!
//! 一次撞击 = "冲量要经粘结传进地基":粘结能传的冲量有上限,传不完的那部分
//! 就是断下来的部分带走的动量(§5.7.1,静载最大流 §4.3 的冲量版)。本模块只含
//! 纯函数:网络构造、整数化最大流与最小源侧判定(§5.7.1)、动量分配与撞击者
//! 修正(§5.7.2–§5.7.3)。生产接入(`evaluate_impacts` 分派、配置键、遥测)在
//! 单元 F02。

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use types::glam::Vec3;
use types::{BlockId, BlockPos, WORLD_Y_MIN};
use voxel::BlockRegistry;

use crate::cohesion::impact_crush_between;
use crate::cohesion::{Dinic, impact_bond_between};
use crate::destruction::DEBRIS_HALF_SIZE;

/// 6 邻居的偏移(与承载判定同序:+x、−x、+y、−y、+z、−z;BFS 层内确定性)。
const NEIGHBOR_DELTAS: [(i32, i32, i32); 6] = [
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
];

/// 正下方(NEIGHBOR_DELTAS[3];WORLD_Y_MIN 汇边用)。
const DOWN: (i32, i32, i32) = (0, -1, 0);

/// 压向判定的方向阈值:u·d̂ ≥ cos 45°(u = a 指向 b 的单位轴向量)→ 压向(§5.7.1)。
const COS_PRESS: f32 = std::f32::consts::FRAC_1_SQRT_2;

/// 量化单位(§5.7.1):所有容量与供给按 0.001 N·s 量化为整数,使最小割唯一
/// 且与算法无关。
const QUANT: u64 = 1000;

/// 容量 ∞ 的哨兵( bedrock、未指定材料;远大于任何合法供给与容量之和)。
const INF_CAP: u64 = u64::MAX / 4;

/// 一次撞击事件(§5.7.1):一个动态实体与地形之间所有计入的接触点。
/// 采样本身在 F02 接入 `evaluate_impacts`;本单元只消费采样结果。
#[derive(Debug, Clone, PartialEq)]
pub struct ImpactEvent {
    /// 各被撞格的供给 `s_c`(单格实体 = 该格各点法向冲量之和;复合体 = 均摊
    /// `J_T / |T|`)。格须已装载且实心;冲量加权的接触点质心由 [`ImpactEvent::point`]
    /// 给出,不再逐格展开。
    pub supplies: Vec<(BlockPos, f32)>,
    /// 冲量矢量 `J = Σ j_k·d_k`(d_k = 撞击方向,压向地形)。
    pub impulse: Vec3,
    /// 作用点 `p`(地形侧接触点按冲量加权的质心)。
    pub point: Vec3,
}

impl ImpactEvent {
    /// 供给合计 `s = Σ s_c`(浮点;整数域比较用 [`ImpactFlowOutcome::supply_q`])。
    pub fn total_supply(&self) -> f32 {
        self.supplies.iter().map(|(_, s)| *s).sum()
    }

    /// `d̂ = normalize(J)`;`|J| = 0` 视为无事件(调用方不建网)。
    pub fn dir_hat(&self) -> Vec3 {
        self.impulse.normalize_or_zero()
    }
}

/// 撞击冲量流的判定结果(纯函数输出;字段齐全,§5.7.1/§5.7.3)。
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ImpactFlowOutcome {
    /// 供给合计的量化值(`s` = 各 `s_c` 向下取整后之和;与网络里的源边容量同一口径,
    /// §5.7.1"整数化"——比较一律在整数域进行)。
    pub supply_q: u64,
    /// 最大流 `C`(量化单位;预筛命中时不建网,为 0)。
    pub cut_q: u64,
    /// S:残量网络中从源点可达的格(最小割的最小源侧,`BlockPos` 升序、唯一)。
    /// 扛住(`cut_q ≥ supply_q`)为空。
    pub detached: Vec<BlockPos>,
    /// S 按"撞击粘结 I > 0 的共面连接"划分的连通分量(§5.7.3 动量分配的刚体分量 k;
    /// 分量内 `BlockPos` 升序,各分量按首格升序)。
    pub components: Vec<Vec<BlockPos>>,
    /// 被切断的粘结数:恰一端在 S、`impact_bond_between > 0` 的共面连接数(遥测口径,
    /// 与承载判定的 `cut_bonds` 同义)。
    pub cut_bonds: usize,
    /// BFS 到达的实心格数(遥测口径 `impact_flow_cells_visited`)。
    pub visited: usize,
    /// BFS 因 `max_cells` 截断(截断处相邻格当汇点;遥测口径 `impact_flow_budget_hits`)。
    pub truncated: bool,
    /// 预筛命中:`s < I_min` 不建网(遥测口径 `impact_flow_prefiltered`)。
    pub prefiltered: bool,
}

/// 撞击者与分量动量分配所需的刚体质量属性(§5.7.2;由调用方从 rapier 质量属性取,
/// 保持 `impact_flow` 不依赖 rapier)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpactBodyProps {
    /// 撞击者质量 m。
    pub mass: f32,
    /// 世界系逆惯量 `I_w⁻¹`(3×3,行主序;`r × (I_w⁻¹ · v)` 语义由 [`inertia_mul`] 使用)。
    /// 静态惯量无限大的实体(不存在,类型上允许)传零矩阵。
    pub inv_inertia: [[f32; 3]; 3],
    /// 撞击者质心(世界系)。
    pub center_of_mass: Vec3,
}

/// 3×3 行主序矩阵乘向量。
fn inertia_mul(m: &[[f32; 3]; 3], v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

/// 撞击者在作用点沿 d̂ 的等效质量 `m_eff = 1 / (1/m + (r × d̂)·I_w⁻¹·(r × d̂))`
/// (§5.7.2;`r = p − 撞击者质心`)。分母为 0(质量与惯量都无限)返回 ∞。
pub fn effective_mass(props: &ImpactBodyProps, point: Vec3, d_hat: Vec3) -> f32 {
    let r = point - props.center_of_mass;
    let rx = r.cross(d_hat);
    let ang = rx.dot(inertia_mul(&props.inv_inertia, rx));
    let denom = 1.0 / props.mass + ang;
    if denom <= 0.0 {
        f32::INFINITY
    } else {
        1.0 / denom
    }
}

/// 动量分配结果(§5.7.2–§5.7.3;一个刚体分量 k 一条)。
#[derive(Debug, Clone, PartialEq)]
pub struct ComponentMomentum {
    /// 分量的格(`BlockPos` 升序)。
    pub cells: Vec<BlockPos>,
    /// 分量质量 `M_k`(格数 × 0.729)。
    pub mass: f32,
    /// 分量的作用点 `p_k`:落在 k 内的接触点按冲量加权的质心;分量内没有接触点时
    /// 取事件作用点 p(全部冲量都经被撞格传入,F00 口径)。
    pub point: Vec3,
    /// 分量冲量 `P_k = M_k / (m_eff + M_S) · E`。
    pub impulse: Vec3,
    /// 分量作为整体的刚体响应:`Δv_k = P_k / M_k`。
    pub delta_v: Vec3,
    /// `Δω_k = I_k⁻¹ (r_k × P_k)`,`r_k = p_k − 分量质心`;`I_k` 是分量的完整
    /// 惯量张量(逐格 cuboid 半边长 0.45、密度 1、平行轴定理含惯性积,§5.7.2)。
    pub delta_w: Vec3,
}

/// 动量分配(纯函数,§5.7.2–§5.7.3):`ρ = (s − C)/s`、超出冲量 `E = ρ·J`;
/// 撞击者在 p 处受修正冲量 `m_eff/(m_eff + M_S)·E`;S 的各分量 k 在 `p_k` 处受
/// `M_k/(m_eff + M_S)·E`。传入的 `outcome` 须为"断开"且未预筛(`detached` 非空),
/// 否则返回空 `Vec` 与零修正(调用方按扛住处理)。
///
/// `contact_points`:事件的各接触点 `(归格, 地形侧位置, 冲量)`——**归格由调用方
/// 按 §5.7.1 口径完成**(地形侧位置沿法向退 0.1 m 取格);纯函数里不再对接触点
/// 坐标取整:接触点恰在格面上(例如从 +x 撞 x = 4 的格,接触点 x = 5.0)时
/// `floor()` 会落进空气一侧的格,`p_k` 会错误地退回事件作用点(F01 验收返修 3)。
/// 分量内没有接触点时该分量的 `p_k` 取事件作用点 p。顺序无关(加权质心按
/// 收集序求和,浮点加法交换误差 ≤ 量化单位,验收口径允许)。
#[allow(clippy::too_many_arguments)]
pub fn distribute_momentum(
    event: &ImpactEvent,
    outcome: &ImpactFlowOutcome,
    contact_points: &[(BlockPos, Vec3, f32)],
    impactor: &ImpactBodyProps,
) -> (Vec3, Vec<ComponentMomentum>) {
    let mut correction = Vec3::ZERO;
    let mut out = Vec::new();
    // 只处理"断开且未预筛"的判定结果(扛住 = 无修正、无分量)。
    if outcome.prefiltered || outcome.detached.is_empty() {
        return (correction, out);
    }
    let s = outcome.supply_q as f32 / QUANT as f32;
    if s <= 0.0 {
        return (correction, out);
    }
    let d_hat = event.dir_hat();
    let j = event.impulse;
    let c = outcome.cut_q as f32 / QUANT as f32;
    let rho = ((s - c) / s).clamp(0.0, 1.0);
    let excess = j * rho;
    let m_s = outcome.detached.len() as f32 * DEBRIS_MASS_PER_CELL;
    let m_eff = effective_mass(impactor, event.point, d_hat);
    // m_eff = ∞(无限质量撞击者)在类型上可能;此时修正与分量冲量按无撞击者
    // 动量分配退化(不会出现在生产路径:撞击者恒为动态实体)。
    let denom = m_eff + m_s;
    if denom <= 0.0 {
        return (correction, out);
    }
    correction = excess * (m_eff / denom);
    // 各分量的接触点(用于 p_k):按调用方归好的格归属。
    for cells in &outcome.components {
        let mass = cells.len() as f32 * DEBRIS_MASS_PER_CELL;
        let cell_set: BTreeSet<BlockPos> = cells.iter().copied().collect();
        let mut w_sum = 0.0_f32;
        let mut p_acc = Vec3::ZERO;
        for &(cell, pt, imp) in contact_points {
            if cell_set.contains(&cell) {
                w_sum += imp;
                p_acc += pt * imp;
            }
        }
        let point = if w_sum > 0.0 {
            p_acc / w_sum
        } else {
            event.point
        };
        let impulse = excess * (mass / denom);
        let delta_v = impulse / mass;
        let com = component_center_of_mass(cells);
        let r_k = point - com;
        let inv_inertia = component_inv_inertia(cells);
        let delta_w = inertia_mul(&inv_inertia, r_k.cross(impulse));
        out.push(ComponentMomentum {
            cells: cells.clone(),
            mass,
            point,
            impulse,
            delta_v,
            delta_w,
        });
    }
    (correction, out)
}

/// 一格碎块的质量(密度 1 × (2·0.45)³,与 `DEBRIS_MASS` 一致;此处独立成常量
/// 以免 `destruction` 模块的实现细节进入本模块的公开契约)。
const DEBRIS_MASS_PER_CELL: f32 = 0.729;

/// 分量质心 = 各格中心的平均(等质量格;世界坐标)。公开供验收对照(AC-F4)。
pub fn component_center_of_mass(cells: &[BlockPos]) -> Vec3 {
    let n = cells.len() as f32;
    cells
        .iter()
        .map(|c| Vec3::new(c.x as f32 + 0.5, c.y as f32 + 0.5, c.z as f32 + 0.5))
        .sum::<Vec3>()
        / n
}

/// 分量的世界系逆惯量张量(§5.7.2:**完整 3×3**,行主序;轴对齐姿态)。
///
/// 逐格 cuboid(半边长 0.45、密度 1)对自身质心的惯量张量
/// `I = m/12 · diag(b²+c², a²+c², a²+b²)`(a = b = c = 0.9,对角、无惯性积),
/// 平行轴定理平移到分量质心:**含惯性积** `−Σ m·dx·dy` 等——L 形、7 字等
/// 不对称分量的惯性积不为零,只取对角会让 Δω 方向出错(F01 验收返修 2);
/// 最后整体求逆(3×3 解析逆)。
///
/// 返回 `Option`:分量退化为零体积(不可能,构造保证 ≥ 1 格)时为 `None`。
fn component_inv_inertia(cells: &[BlockPos]) -> [[f32; 3]; 3] {
    let com = component_center_of_mass(cells);
    let half = DEBRIS_HALF_SIZE;
    let cell_mass = DEBRIS_MASS_PER_CELL;
    // 每格自身惯量(对角;立方格三轴相等)。
    let base = cell_mass / 12.0 * (4.0 * half * half + 4.0 * half * half);
    // 对角与惯性积累加:I_xy = −Σ m·dx·dy 等(平行轴定理的惯性积项)。
    let mut i = [[0.0_f32; 3]; 3];
    for c in cells {
        let d = Vec3::new(c.x as f32 + 0.5, c.y as f32 + 0.5, c.z as f32 + 0.5) - com;
        i[0][0] += base + cell_mass * (d.y * d.y + d.z * d.z);
        i[1][1] += base + cell_mass * (d.x * d.x + d.z * d.z);
        i[2][2] += base + cell_mass * (d.x * d.x + d.y * d.y);
        i[0][1] -= cell_mass * d.x * d.y;
        i[0][2] -= cell_mass * d.x * d.z;
        i[1][2] -= cell_mass * d.y * d.z;
    }
    i[1][0] = i[0][1];
    i[2][0] = i[0][2];
    i[2][1] = i[1][2];
    invert_symmetric_3x3(&i)
}

/// 3×3 对称矩阵的解析逆(行主序;奇异性不可能——至少 1 格的非零惯量)。
/// `A = [[a,b,c],[b,d,e],[c,e,f]]`,伴随 = 余子式矩阵的转置(对称同位),
/// `det = a·C00 + b·C01 + c·C02`。
fn invert_symmetric_3x3(m: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let (a, b, c, d, e, f) = (m[0][0], m[0][1], m[0][2], m[1][1], m[1][2], m[2][2]);
    let c00 = d * f - e * e;
    let c01 = c * e - b * f;
    let c02 = b * e - d * c;
    let c11 = a * f - c * c;
    let c12 = b * c - a * e;
    let c22 = a * d - b * b;
    let det = a * c00 + b * c01 + c * c02;
    debug_assert!(det > 0.0, "惯量张量正定(至少 1 格)");
    let inv_det = 1.0 / det;
    [
        [c00 * inv_det, c01 * inv_det, c02 * inv_det],
        [c01 * inv_det, c11 * inv_det, c12 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ]
}

/// 分量被切成多段时各段的初速度(§5.7.3 第 3 条;与 §7.3 第 5 条 `fracture_pieces`
/// 同一口径):`v_seg = Δv_k + Δω_k × (质心_seg − 质心_k)`、`ω_seg = Δω_k`。
/// `segments`:各段的格(段内顺序不限;输出与输入同序)。
pub fn segment_velocity(
    component: &ComponentMomentum,
    segments: &[Vec<BlockPos>],
) -> Vec<(Vec3, Vec3)> {
    let com_k = component_center_of_mass(&component.cells);
    segments
        .iter()
        .map(|seg| {
            let com_seg = component_center_of_mass(seg);
            let v = component.delta_v + component.delta_w.cross(com_seg - com_k);
            (v, component.delta_w)
        })
        .collect()
}

/// 撞击冲量流(纯函数,§5.7.1;一个事件单独建网):
///
/// - `block_at(q) = None` = 所在 chunk 未装载(汇点);`in_core(q) = false` = 格在
///   本体区外(副本格,跨分片保守视为地基,汇点);
/// - **节点** = 从**全部**被撞格出发(**多源 BFS**:第一层 = 全部供给格,按
///   `BlockPos` 升序)、经共面相邻 BFS 到达的实心格(本拍已被先前事件写成 air
///   的格 `block_at` 已是空气,天然不入网),至多 `max_cells` 个(BFS 按层、
///   层内 `BlockPos` 升序,确定性)——复合体同时压在两处互不相连的地形上
///   (长条架在两根柱上),两处都在网内;
/// - **源边**:源点 → 区域内供给格 c 容量 `s_c` 的量化值(逐格向下取整);
///   **被撞格不在区域内**(已写成 air、本体区外、未装载、超出预算)时,它的
///   供给既不建源边也不计入 `s`——`s` 恒等于源边容量之和(代码内以
///   `debug_assert` 钉住;F01 曾只从供给最大的格出发 BFS,不相连的被撞格的
///   供给计入了 `s` 却没有源边,判"断开"而 S = ∅);
/// - **格间边**:区域内相邻实心格 a → b——`u` = a 指向 b 的单位轴向量,
///   `u·d̂ ≥ cos 45°`(b 在 a 的受力背后,a 被压向 b)→ 压向容量
///   `I_c(a,b) = min(I_c,a, I_c,b)`;否则(剪、拉)→ `I(a,b) = min(I_a, I_b)`;
/// - **汇点**(看作地基,冲量传到即被吸收):`Support::Anchor` 材料的格、
///   `y == WORLD_Y_MIN` 格的向下方向、相邻格在未装载 chunk、相邻格在本体区外、
///   相邻格超出 BFS 预算——容量按上一条规则以 a 与该相邻格的材料计
///   (相邻格无材料或 ∞ 时取 a 侧);
/// - **整数化**:供给向下取整、容量向上取整,∞ 用哨兵;
/// - **判定在整数域**:`C ≥ supply_q` 扛住;否则 S = 残量网络中源点可达的格
///   (最小割的最小源侧)——`supply_q` 与源边容量同一口径,因此
///   `C < supply_q` 时必有源边不饱和,S 必非空(F00 曾拿向上取整的 s 与 C 比,
///   造出"判断开却 S = ∅"的退化情形,实现不得如此——本句是断言级规格);
/// - **预筛**:`s < I_min`(注册表全部材料 `min(I, I_c)` 的最小值)不建网络;
///   `|J| = 0` 视为无事件。
pub fn evaluate_impact_flow(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    in_core: &dyn Fn(BlockPos) -> bool,
    registry: &dyn BlockRegistry,
    event: &ImpactEvent,
    max_cells: usize,
) -> ImpactFlowOutcome {
    let mut outcome = ImpactFlowOutcome::default();
    // |J| = 0 视为无事件。
    if event.impulse == Vec3::ZERO || event.supplies.is_empty() {
        return outcome;
    }
    let d_hat = event.dir_hat();
    // 供给格(去重;同一格重复出现按求和)。
    let mut supplies: BTreeMap<BlockPos, f32> = BTreeMap::new();
    for &(c, s) in &event.supplies {
        *supplies.entry(c).or_insert(0.0) += s;
    }
    // BFS 区域:多源(第一层 = 全部供给格,按 BlockPos 升序),按层、层内升序,
    // 至多 max_cells 个实心格。
    let (region, truncated) = bfs_region(
        block_at,
        in_core,
        registry,
        supplies.keys().copied(),
        max_cells,
    );
    outcome.visited = region.len();
    outcome.truncated = truncated;
    // 不在区域内的被撞格:供给既不建源边也不计入 s(§5.7.1 源边条)。
    // (空气、本体区外、未装载、超出预算的格都不在 region。)
    let in_region_supplies: BTreeMap<BlockPos, f32> = supplies
        .iter()
        .filter(|(c, _)| region.contains(c))
        .map(|(c, &s)| (*c, s))
        .collect();
    // 预筛:s < I_min(全部材料 min(I, I_c) 的最小值)→ 任何割都比供给大,不建网。
    let i_min = min_impact(registry);
    let s_total: f32 = in_region_supplies.values().sum();
    outcome.supply_q = in_region_supplies
        .values()
        .map(|&s| quant_supply(s))
        .sum::<u64>()
        .min(INF_CAP);
    if s_total < i_min {
        outcome.prefiltered = true;
        return outcome;
    }
    let idx: BTreeMap<BlockPos, usize> = region
        .iter()
        .enumerate()
        .map(|(i, c)| (*c, i + 2))
        .collect();
    let in_region = |c: BlockPos| idx.contains_key(&c);
    let mut net = Dinic::new(region.len() + 2);
    const SRC: usize = 0;
    const SNK: usize = 1;
    for &a in &region {
        let ua = idx[&a];
        // 源边:供给逐格向下取整(区域内其他格也可能有供给,多点接触)。
        if let Some(&s) = in_region_supplies.get(&a) {
            let q = quant_supply(s);
            if q > 0 {
                net.edge(SRC, ua, q);
            }
        }
        let ba = block_at(a).expect("BFS 区域内的格已装载");
        let ma = registry.material(ba);
        for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
            let b = BlockPos::new(a.x + dx, a.y + dy, a.z + dz);
            let u_dot = Vec3::new(dx as f32, dy as f32, dz as f32).dot(d_hat);
            // 相邻格分类:未装载(None)→ 汇;已装载但非实心(空气)→ 无边;
            // 实心 → 格间边或汇(本体区外/截断)。
            let b_block = block_at(b);
            let mb_none_unloaded = b_block.is_none();
            let mb = b_block.and_then(|bb| registry.material(bb));
            if !mb_none_unloaded && mb.is_none() {
                continue; // 空气:冲量传不进,无边(先前事件写成 air 的格不入网)
            }
            let cap_raw = match (ma, mb) {
                // 两侧都有材料:压向用 I_c(a,b)、剪拉用 I(a,b)(cohesion 的
                // 取较小值辅助,与 impact_bond_between 并列;§5.7.1)。
                (Some(_), Some(_)) if u_dot >= COS_PRESS => {
                    impact_crush_between(registry, ba, b_block.expect("已检查"))
                }
                (Some(_), Some(_)) => impact_bond_between(registry, ba, b_block.expect("已检查")),
                // 相邻格无材料或 ∞ 时取 a 侧(§5.7.1 汇点口径)。
                (Some(x), _) if u_dot >= COS_PRESS => x.impact_crush,
                (Some(x), _) => x.impact_threshold,
                _ => 0.0,
            };
            // 未装载 chunk → 汇点(容量按 a 侧材料计,§5.7.1)。
            if mb_none_unloaded {
                let cap = quant_cap(cap_raw);
                if cap > 0 {
                    net.edge(ua, SNK, cap);
                }
                continue;
            }
            // 本体区外、BFS 截断 → 汇点(走到这里的 b 一定是实心格)。
            let to_sink = !in_core(b) || !in_region(b);
            if to_sink {
                let cap = quant_cap(cap_raw);
                if cap > 0 {
                    net.edge(ua, SNK, cap);
                }
                continue;
            }
            // 区域内的实心格(含 Anchor 材料的格之间——Anchor 是"汇点材料",
            // 但两个实心格之间的边照常按容量连接;汇边已单独给出)。
            let cap = quant_cap(cap_raw);
            if cap > 0 {
                net.edge(ua, idx[&b], cap);
            }
        }
        // Anchor 材料:a → 汇点(不限量;bedrock 本身就是地基)。
        if ma.is_some_and(|x| x.support == voxel::Support::Anchor) {
            net.edge(ua, SNK, INF_CAP);
        }
        // WORLD_Y_MIN 格的向下方向:a → 汇点(容量按 a 与正下方(无材料)计)。
        if a.y == WORLD_Y_MIN {
            let down = DOWN;
            let u_dot = Vec3::new(down.0 as f32, down.1 as f32, down.2 as f32).dot(d_hat);
            let cap_raw = match (ma, u_dot >= COS_PRESS) {
                (Some(x), true) => x.impact_crush,
                (Some(x), false) => x.impact_threshold,
                _ => 0.0,
            };
            let cap = quant_cap(cap_raw);
            if cap > 0 {
                net.edge(ua, SNK, cap);
            }
        }
    }
    net.max_flow(SRC, SNK);
    outcome.cut_q = net.flow_value(SRC);
    // 断言级规格:s 恒等于源边容量之和(不在区域内的被撞格不计入 s;
    // F01 验收返修 1)。零供给的退化情形(全部被撞格都不在区域内)也满足。
    debug_assert_eq!(
        outcome.supply_q,
        region
            .iter()
            .filter_map(|a| in_region_supplies.get(a))
            .map(|&s| quant_supply(s))
            .sum::<u64>()
            .min(INF_CAP),
        "supply_q 必须等于源边容量之和"
    );
    let reach = net.reachable(SRC);
    if outcome.cut_q < outcome.supply_q {
        // C < s:残量网络中从源点可达的格 S 断下(必有源边不饱和,S 非空)。
        let detached: Vec<BlockPos> = region.iter().filter(|c| reach[idx[*c]]).copied().collect();
        outcome.cut_bonds = count_cut_bonds(block_at, registry, &detached);
        outcome.components = impact_components(block_at, registry, &detached);
        outcome.detached = detached;
    }
    outcome
}

/// 注册表全部材料 `min(I, I_c)` 的最小值(预筛阈值 I_min;§5.7.1)。
/// 无任何实心材料时返回 ∞(一切都扛住)。
pub fn min_impact(registry: &dyn BlockRegistry) -> f32 {
    (0..registry.block_count())
        .filter_map(|i| registry.material(BlockId(i)))
        .map(|m| m.impact_threshold.min(m.impact_crush))
        .fold(f32::INFINITY, f32::min)
}

/// 供给量化:逐格向下取整(§5.7.1;负供给不存在,冲量非负)。
fn quant_supply(s: f32) -> u64 {
    if s <= 0.0 {
        0
    } else {
        (s * QUANT as f32).floor() as u64
    }
}

/// 容量量化:向上取整;∞ 用哨兵(§5.7.1)。
fn quant_cap(c: f32) -> u64 {
    if c.is_infinite() {
        INF_CAP
    } else if c <= 0.0 {
        0
    } else {
        let q = (c * QUANT as f32).ceil() as u64;
        q.min(INF_CAP)
    }
}

/// BFS 区域(§5.7.1,**多源**):第一层 = 全部供给格(迭代序 = `BlockPos`
/// 升序),之后按层、层内 `BlockPos` 升序,至多 `max_cells` 个实心格;截断 =
/// 某层入队后预算用完仍有格未出队。本拍已被先前事件写成 air 的格 `block_at`
/// 已是空气,天然不入网;不在本体区内的供给格同样不入网(其供给不计入 s)。
fn bfs_region(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    in_core: &dyn Fn(BlockPos) -> bool,
    registry: &dyn BlockRegistry,
    sources: impl Iterator<Item = BlockPos>,
    max_cells: usize,
) -> (Vec<BlockPos>, bool) {
    let solid =
        |q: BlockPos| in_core(q) && block_at(q).is_some_and(|b| registry.material(b).is_some());
    let mut seen: BTreeSet<BlockPos> = BTreeSet::new();
    let mut frontier: Vec<BlockPos> = Vec::new();
    for c in sources {
        // 去重 + 只进实心格(去重靠 seen:BTreeSet 先到先得,重复源天然合并)。
        if seen.insert(c) && solid(c) {
            frontier.push(c);
        }
    }
    // 第一层 = 全部供给格,按 BlockPos 升序(sources 已按 BTreeMap 迭代序给出,
    // 此处排序兜底并保持与后续层一致)。
    frontier.sort_unstable();
    let mut region: Vec<BlockPos> = Vec::new();
    let mut truncated = false;
    while !frontier.is_empty() {
        // 层内升序(BTreeSet 迭代序);预算在出队时消耗。
        frontier.sort_unstable();
        let mut next: Vec<BlockPos> = Vec::new();
        for c in frontier.drain(..) {
            if region.len() >= max_cells {
                truncated = true;
                break;
            }
            region.push(c);
            for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
                let n = BlockPos::new(c.x + dx, c.y + dy, c.z + dz);
                if !seen.insert(n) || !solid(n) {
                    continue;
                }
                next.push(n);
            }
        }
        if truncated {
            break;
        }
        frontier = next;
    }
    if !frontier.is_empty() {
        truncated = true;
    }
    (region, truncated)
}

/// 被切断的粘结数:恰一端在 S、另一端是 S 之外的实心格、
/// `impact_bond_between > 0` 的共面连接(与承载判定 `cut_bonds` 同口径)。
fn count_cut_bonds(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    registry: &dyn BlockRegistry,
    detached: &[BlockPos],
) -> usize {
    let set: BTreeSet<BlockPos> = detached.iter().copied().collect();
    let mut n = 0usize;
    for &c in detached {
        let bc = block_at(c).expect("S 中的格已装载");
        for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
            let nb = BlockPos::new(c.x + dx, c.y + dy, c.z + dz);
            if set.contains(&nb) {
                continue;
            }
            if let Some(bn) = block_at(nb)
                && registry.material(bn).is_some()
                && impact_bond_between(registry, bc, bn) > 0.0
            {
                n += 1;
            }
        }
    }
    n
}

/// S 按"撞击粘结 I > 0 的共面连接"求连通分量(§5.7.3;分量内 `BlockPos` 升序,
/// 各分量按首格升序)。
fn impact_components(
    block_at: &dyn Fn(BlockPos) -> Option<BlockId>,
    registry: &dyn BlockRegistry,
    detached: &[BlockPos],
) -> Vec<Vec<BlockPos>> {
    let set: BTreeSet<BlockPos> = detached.iter().copied().collect();
    let mut seen: BTreeSet<BlockPos> = BTreeSet::new();
    let mut out = Vec::new();
    for &start in &set {
        if !seen.insert(start) {
            continue;
        }
        let mut comp: BTreeSet<BlockPos> = BTreeSet::from([start]);
        let mut queue = VecDeque::from([start]);
        while let Some(c) = queue.pop_front() {
            for &(dx, dy, dz) in &NEIGHBOR_DELTAS {
                let n = BlockPos::new(c.x + dx, c.y + dy, c.z + dz);
                if !set.contains(&n)
                    || comp.contains(&n)
                    || match (block_at(c), block_at(n)) {
                        (Some(a), Some(b)) => impact_bond_between(registry, a, b) <= 0.0,
                        _ => true,
                    }
                {
                    continue;
                }
                comp.insert(n);
                queue.push_back(n);
            }
        }
        seen.extend(comp.iter().copied());
        out.push(comp.into_iter().collect());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use voxel::{Material, SimpleRegistry, Support, builtin};

    fn p(x: i32, y: i32, z: i32) -> BlockPos {
        BlockPos::new(x, y, z)
    }

    /// 指定格为给定方块、其余空气的世界(始终装载、始终本体区内)。
    fn block_world(
        cells: impl IntoIterator<Item = (BlockPos, BlockId)>,
    ) -> impl Fn(BlockPos) -> Option<BlockId> {
        let map: BTreeMap<BlockPos, BlockId> = cells.into_iter().collect();
        move |q| Some(map.get(&q).copied().unwrap_or(builtin::AIR))
    }

    /// 全域本体区(单测里不测本体区外汇点时用)。
    fn all_core() -> impl Fn(BlockPos) -> bool {
        |_| true
    }

    /// 横撞 -x(d̂ = (−1,0,0))的事件:单点供给 s、作用点 = 命中格中心。
    fn side_hit(hit: BlockPos, s: f32) -> ImpactEvent {
        ImpactEvent {
            supplies: vec![(hit, s)],
            impulse: Vec3::new(-s, 0.0, 0.0),
            point: Vec3::new(hit.x as f32 + 0.5, hit.y as f32 + 0.5, hit.z as f32 + 0.5),
        }
    }

    /// 从上砸(d̂ = (0,−1,0))的事件。
    fn top_hit(hit: BlockPos, s: f32) -> ImpactEvent {
        ImpactEvent {
            supplies: vec![(hit, s)],
            impulse: Vec3::new(0.0, -s, 0.0),
            point: Vec3::new(hit.x as f32 + 0.5, hit.y as f32 + 0.5, hit.z as f32 + 0.5),
        }
    }

    const MAX_CELLS: usize = 1024;

    /// 带地基的伸出格世界(F00 工况 1 口径):伸出格 (0,0,0) 接一堵 5×5 石墙
    /// (x=1、y=−2..2、z=−2..2),墙下整层 bedrock(y=−3、x=−1..1、z=−2..2)。
    /// 伸出格的唯一"窄口"是 (0,0,0)-(1,0,0) 那根——横撞剪切 18、迎面压向 72;
    /// 墙→bedrock 界面有 5 条竖直剪切 18 = 90(横撞)/ 90(迎面),都不窄于窄口。
    fn protruding_world() -> impl Fn(BlockPos) -> Option<BlockId> {
        let mut cells: Vec<(BlockPos, BlockId)> = vec![(p(0, 0, 0), builtin::STONE)];
        for y in -2..=2 {
            for z in -2..=2 {
                cells.push((p(1, y, z), builtin::STONE));
                cells.push((p(1, -3, z), builtin::BEDROCK));
            }
        }
        block_world(cells)
    }

    // ------------------------------------------------------------------ 容量

    /// AC-F2 压向判定:横撞伸出格(d̂ = (−1,0,0)),唯一粘结在 +x 侧
    /// (u = (+1,0,0),u·d̂ = −1 < cos45°)→ 剪切 I = 18;迎面撞回墙里
    /// (d̂ = (+1,0,0),u·d̂ = +1 ≥ cos45°)→ 压向 I_c = 72(§5.7.1 表第 4/5 行)。
    ///
    /// 会变红的实现:压向方向取反(横撞 72 / 迎面 18);漏取两格较小值。
    #[test]
    fn crush_vs_shear_direction() {
        let reg = SimpleRegistry::builtin();
        let world = protruding_world();
        // 横撞:供给 18.0005 → 量化 18000 = C 扛住;18.002 → 18002 > 18000 断。
        let hold = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(0, 0, 0), 18.0005),
            MAX_CELLS,
        );
        assert_eq!(hold.cut_q, 18_000, "剪切割 = 18");
        assert_eq!(hold.supply_q, 18_000);
        assert!(hold.detached.is_empty(), "C = s 扛住");
        let brk = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(0, 0, 0), 18.002),
            MAX_CELLS,
        );
        assert_eq!(brk.cut_q, 18_000);
        assert_eq!(brk.detached, vec![p(0, 0, 0)], "S = 被撞格");
        // 迎面撞回墙里:压向割 = 72。
        let push = ImpactEvent {
            supplies: vec![(p(0, 0, 0), 72.002)],
            impulse: Vec3::new(72.002, 0.0, 0.0),
            point: Vec3::new(0.5, 0.5, 0.5),
        };
        let out = evaluate_impact_flow(&world, &all_core(), &reg, &push, MAX_CELLS);
        assert_eq!(out.cut_q, 72_000, "压向割 = I_c = 72");
        assert_eq!(out.detached, vec![p(0, 0, 0)]);
    }

    /// AC-F2 整数化:供给向下取整;判定在整数域(C ≥ supply_q 扛住)。
    ///
    /// 会变红的实现:供给向上取整(18.0004 → 18001 > 18000 误判断开)。
    #[test]
    fn integer_domain_comparison() {
        let reg = SimpleRegistry::builtin();
        let world = protruding_world();
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(0, 0, 0), 18.0004),
            MAX_CELLS,
        );
        assert_eq!(out.supply_q, 18_000);
        assert!(out.detached.is_empty(), "C ≥ s(整数域)扛住");
    }

    /// AC-F3 退化判定:C < s 时 S 必非空(整数域比较的断言级规格)。
    ///
    /// 会变红的实现:拿向上取整的 s 与 C 比(18.001 → 18001 仍断,但
    /// C = s 的 18.0005 会误判断开而 S = ∅)。
    #[test]
    fn cut_below_supply_implies_nonempty_s() {
        let reg = SimpleRegistry::builtin();
        let world = protruding_world();
        // 横撞窄口 = 剪切 18:供给 18.0005 → 向下取整 18000 = C → 扛住(S 必空)。
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(0, 0, 0), 18.0005),
            MAX_CELLS,
        );
        assert_eq!(out.supply_q, 18_000);
        assert!(out.detached.is_empty());
        // 18.001 → 18001 > 18000 → 断,S 非空。
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(0, 0, 0), 18.001),
            MAX_CELLS,
        );
        assert_eq!(out.supply_q, 18_001);
        assert_eq!(out.cut_q, 18_000);
        assert!(!out.detached.is_empty(), "C < s 时 S 必非空");
    }

    /// AC-F2 五类汇点,逐一构造。
    ///
    /// 会变红的实现:少任何一类汇点(C 偏小,S 偏大)。
    #[test]
    fn five_sink_kinds() {
        let reg = SimpleRegistry::builtin();
        // 汇 1 Anchor:石墙脚下是 bedrock(Anchor 材料的格自身是地基),
        // 1 格厚墙(F00 工况 2a 几何:y = 0..4、z = −2..2,脚在 bedrock)正面
        // 中部横撞 → C = 4×18 = 72(§5.7.1 表第 1 行)。
        let mut cells: Vec<(BlockPos, BlockId)> = Vec::new();
        for y in 0..=4 {
            for z in -2..=2 {
                cells.push((p(-1, y, z), builtin::STONE));
            }
        }
        for z in -2..=2 {
            cells.push((p(-1, -1, z), builtin::BEDROCK));
        }
        let world = block_world(cells);
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(-1, 2, 0), 72.002),
            MAX_CELLS,
        );
        assert_eq!(out.cut_q, 72_000, "bedrock 支承:割 = 四周 4 根剪切");
        assert_eq!(out.detached, vec![p(-1, 2, 0)]);

        // 汇 2 WORLD_Y_MIN:y = −64 的孤立石格被向下砸:向下方向 u·d̂ = 1 →
        // 压向 I_c = 72(漏了这一汇,BFS 沿 −y 无界或 C = ∞)。
        let world = block_world([(p(0, WORLD_Y_MIN, 0), builtin::STONE)]);
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &top_hit(p(0, WORLD_Y_MIN, 0), 72.002),
            MAX_CELLS,
        );
        assert_eq!(out.cut_q, 72_000, "WORLD_Y_MIN 向下汇(压向 I_c)");
        assert_eq!(out.detached, vec![p(0, WORLD_Y_MIN, 0)]);
        // 横撞同一格:向下汇 u·d̂ = 0 → 剪切 I = 18。
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(0, WORLD_Y_MIN, 0), 18.002),
            MAX_CELLS,
        );
        assert_eq!(out.cut_q, 18_000, "WORLD_Y_MIN 向下汇(剪切口径)");

        // 汇 3 未装载:block_at 返回 None 的方向当汇。石格 (0,0,0),−x 方向
        // 未装载(None),其余方向是已装载的空气(Some(AIR),无边):
        // u = (−1,0,0)、d̂ = (−1,0,0) → u·d̂ = 1 ≥ cos45° → 压向 72。
        let world = move |q: BlockPos| {
            if q == p(0, 0, 0) {
                Some(builtin::STONE)
            } else if q.x >= 0 {
                Some(builtin::AIR) // x ≥ 0 半空间已装载
            } else {
                None // x < 0:未装载 chunk
            }
        };
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(0, 0, 0), 72.002),
            MAX_CELLS,
        );
        assert_eq!(out.cut_q, 72_000, "未装载 chunk 汇(压向侧)");
        assert_eq!(out.detached, vec![p(0, 0, 0)]);

        // 汇 4 本体区外:in_core = false 的方向当地基(跨分片保守)。
        // 本体区 = x ≥ 0:hit (0,0,0) 在区内,−x 邻格 (−1,0,0) 在区外 → 汇。
        let world = block_world([(p(-1, 0, 0), builtin::STONE), (p(0, 0, 0), builtin::STONE)]);
        let core_x_ge_0 = |q: BlockPos| q.x >= 0;
        // 横撞(u·d̂ = −1?u = a→b = (−1,0,0),d̂ = (−1,0,0) → u·d̂ = 1 → 压向):
        // 割 = 区外方向汇 72。
        let out = evaluate_impact_flow(
            &world,
            &core_x_ge_0,
            &reg,
            &side_hit(p(0, 0, 0), 72.002),
            MAX_CELLS,
        );
        assert_eq!(out.cut_q, 72_000, "本体区外汇点");

        // 汇 5 BFS 截断:max_cells = 1 → 只有命中格入网,实心邻格是截断汇。
        let out = evaluate_impact_flow(&world, &all_core(), &reg, &side_hit(p(0, 0, 0), 72.002), 1);
        assert!(out.truncated);
        assert_eq!(out.visited, 1);
        assert_eq!(out.cut_q, 72_000, "截断处当汇(压向 72)");
    }

    /// AC-F2 BFS 层序确定:预算恰好 = 区域大小时不截断;截断时 visited = 预算。
    ///
    /// 会变红的实现:DFS 序(截断集合不同);层内不排序(同层截断集含随机格)。
    #[test]
    fn bfs_layered_order_and_truncation_flag() {
        let reg = SimpleRegistry::builtin();
        let mut cells: Vec<(BlockPos, BlockId)> =
            (0..6).map(|x| (p(x, 0, 0), builtin::STONE)).collect();
        cells.push((p(2, 1, 0), builtin::STONE));
        let world = block_world(cells);
        let ok = evaluate_impact_flow(&world, &all_core(), &reg, &side_hit(p(0, 0, 0), 18.002), 7);
        assert!(!ok.truncated, "预算恰好 = 区域大小时不截断");
        assert_eq!(ok.visited, 7);
        let cut = evaluate_impact_flow(&world, &all_core(), &reg, &side_hit(p(0, 0, 0), 18.002), 3);
        assert!(cut.truncated);
        assert_eq!(cut.visited, 3);
    }

    /// AC-F2 已写成 air 的格不入网:block_at 已返回 AIR 的格不进 BFS。
    ///
    /// 会变红的实现:BFS 穿过已破坏的格(把先前事件的洞又连回来)。
    #[test]
    fn cells_written_air_are_excluded() {
        let reg = SimpleRegistry::builtin();
        let world = block_world([
            (p(0, 0, 0), builtin::STONE),
            (p(1, 0, 0), builtin::AIR), // 本拍先前事件写成 air
            (p(2, 0, 0), builtin::STONE),
            (p(2, -1, 0), builtin::BEDROCK),
        ]);
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(2, 0, 0), 72.002),
            MAX_CELLS,
        );
        assert_eq!(
            out.cut_q, 18_000,
            "air 格断开传播:割 = bedrock 界面剪切 18(压向侧被 air 洞断掉)"
        );
        assert_eq!(out.visited, 2, "air 格不入网(hit + bedrock 支承格)");
    }

    /// AC-F3 预筛:s < I_min(sand 的 min(I, I_c) = 4)不建网;|J| = 0 无事件。
    ///
    /// 会变红的实现:预筛阈值取 min(I) 漏 I_c;0 冲量仍建网。
    #[test]
    fn prefilter_and_zero_impulse() {
        let reg = SimpleRegistry::builtin();
        let world = block_world([(p(0, 0, 0), builtin::SAND), (p(1, 0, 0), builtin::SAND)]);
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(0, 0, 0), 3.999),
            MAX_CELLS,
        );
        assert!(out.prefiltered, "s < I_min = 4 不建网");
        assert_eq!(out.cut_q, 0);
        assert!(out.detached.is_empty());
        // visited = BFS 到达的实心格数(多源 BFS 先于预筛执行;遥测口径是
        // "访问过的格",预筛只是不建网、不抹掉这次访问的事实)。
        assert_eq!(out.visited, 2);
        // 恰好 = 4:建网(扛住但 visited > 0)。
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(0, 0, 0), 4.0),
            MAX_CELLS,
        );
        assert!(!out.prefiltered);
        assert!(out.visited > 0);
        // |J| = 0:无事件。
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &ImpactEvent {
                supplies: vec![(p(0, 0, 0), 100.0)],
                impulse: Vec3::ZERO,
                point: Vec3::ZERO,
            },
            MAX_CELLS,
        );
        assert_eq!(out, ImpactFlowOutcome::default(), "|J| = 0 无事件");
        assert_eq!(min_impact(&reg), 4.0, "I_min = min(I, I_c) 的最小值(沙 4)");
    }

    // ------------------------------------------------------------------ 材料

    /// AC-F1:`impact_crush_between` 两格取较小值;任一侧非实心为 0。
    ///
    /// 会变红的实现:取较大值;非实心侧取 ∞。
    #[test]
    fn crush_between_takes_min_and_zero_for_non_solid() {
        let reg = SimpleRegistry::builtin();
        assert_eq!(
            impact_crush_between(&reg, builtin::STONE, builtin::STONE),
            72.0
        );
        assert_eq!(
            impact_crush_between(&reg, builtin::STONE, builtin::DIRT),
            28.0
        );
        assert_eq!(
            impact_crush_between(&reg, builtin::DIRT, builtin::STONE),
            28.0
        );
        assert_eq!(
            impact_crush_between(&reg, builtin::SAND, builtin::DIRT),
            16.0
        );
        assert_eq!(
            impact_crush_between(&reg, builtin::STONE, builtin::BEDROCK),
            72.0,
            "bedrock ∞ 不拉低下界"
        );
        assert_eq!(
            impact_crush_between(&reg, builtin::STONE, builtin::AIR),
            0.0
        );
        assert_eq!(impact_crush_between(&reg, builtin::STONE, BlockId(99)), 0.0);
    }

    /// 材料:`Material` 带 `impact_crush` 字段(compile-check;内置数值断言在 voxel)。
    #[test]
    fn material_field_roundtrip() {
        let m = Material {
            resistance: 1.0,
            impact_threshold: 2.0,
            support: Support::Connected,
            pulverize_factor: 2.0,
            bond: 3,
            impact_crush: 8.0,
        };
        assert_eq!(m.impact_crush, 8.0);
        assert_eq!(
            Material::INDESTRUCTIBLE.impact_crush,
            f32::INFINITY,
            "未指定材料 ∞"
        );
    }

    // ------------------------------------------------------------------ §5.7.2

    /// AC-F4 m_eff:共线撞击(r ∥ d̂ → r × d̂ = 0)时 m_eff = m;偏心时 m_eff < m
    /// (转动分担冲量)。
    ///
    /// 会变红的实现:漏转动项(m_eff 恒 = m);r 用了局部坐标。
    #[test]
    fn effective_mass_collinear_and_offcenter() {
        let props = ImpactBodyProps {
            mass: 4.374,
            inv_inertia: [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]],
            center_of_mass: Vec3::new(10.0, 10.0, 10.0),
        };
        // 共线:p 在质心正前方沿 d̂。
        let d = Vec3::new(-1.0, 0.0, 0.0);
        let m = effective_mass(&props, Vec3::new(8.0, 10.0, 10.0), d);
        assert!((m - 4.374).abs() < 1e-4, "共线 m_eff = m,实得 {m}");
        // 偏心:r = (0,1,0),r × d̂ = (0,0,−1),转动项 = (0,0,1)·I⁻¹·(0,0,−1) = 2。
        let m = effective_mass(&props, Vec3::new(10.0, 11.0, 10.0), d);
        let expect = 1.0 / (1.0 / 4.374 + 2.0);
        assert!((m - expect).abs() < 1e-4, "偏心 m_eff = {expect},实得 {m}");
    }

    /// AC-F4 守恒:`(C/s)·J + 修正 + Σ P_k = J`(逐分量)。单格 S、共线撞击:
    /// m_eff = m(接触点在质心行进线上)、M_S = 0.729 → 修正 = P_k = E/2,
    /// Δω = 0(r_k = 0)。
    ///
    /// 会变红的实现:ρ 用 (s−C)/C;漏 M_S(修正 = E);P_k 不按质量比例。
    #[test]
    fn momentum_conservation_single_component() {
        let reg = SimpleRegistry::builtin();
        let world = protruding_world();
        let event = side_hit(p(0, 0, 0), 20.0);
        let outcome = evaluate_impact_flow(&world, &all_core(), &reg, &event, MAX_CELLS);
        assert_eq!(outcome.cut_q, 18_000);
        assert_eq!(outcome.detached, vec![p(0, 0, 0)], "S = 伸出的单格");
        let impactor = ImpactBodyProps {
            mass: 0.729,
            inv_inertia: [[0.122, 0.0, 0.0], [0.0, 0.122, 0.0], [0.0, 0.0, 0.122]],
            center_of_mass: Vec3::new(3.5, 0.5, 0.5),
        };
        let contacts = vec![(p(0, 0, 0), event.point, 20.0)];
        let (correction, comps) = distribute_momentum(&event, &outcome, &contacts, &impactor);
        assert_eq!(comps.len(), 1);
        let s = 20.0_f32;
        let c = 18.0_f32;
        let j = event.impulse;
        let e = j * ((s - c) / s);
        // p − 质心 = (−3,0,0) ∥ d̂ → 共线,m_eff = 0.729 = M_S → 修正 = P_k = E/2。
        assert!(
            (correction - e * 0.5).length() < 1e-4,
            "修正 = E·m_eff/(m_eff+M_S) = E/2"
        );
        let k = &comps[0];
        assert!((k.impulse - e * 0.5).length() < 1e-4);
        assert!((k.delta_v - e * 0.5 / 0.729).length() < 1e-5, "Δv = P/M");
        // 守恒式逐分量(残差 ≤ 量化单位 × 接触点数,这里 1e-3 N·s = 1 量化单位)。
        let resid = j * (c / s) + correction + k.impulse - j;
        assert!(resid.length() < 1e-3, "守恒残差 {resid:?}");
        // p_k = 格中心 = S 质心 → r_k = 0 → Δω = 0(共线对中)。
        assert!(k.delta_w.length() < 1e-6, "共线撞击 Δω = 0");
    }

    /// AC-F4 多分量与偏心:两个不相邻的伸出格同时断下(割 = 各自剪切之和 36),
    /// 分量冲量按质量比例;接触点挪到格内偏角处 → p_k ≠ 分量质心 → Δω ≠ 0。
    ///
    /// 会变红的实现:p_k 用事件作用点(两分量共享,Δω 全零);分量按承载 B 划分
    /// (沙 0 也连通/断开)。
    #[test]
    fn momentum_offcenter_and_multi_component() {
        let reg = SimpleRegistry::builtin();
        // 两个伸出格共享一面墙:(0,0,0) 与 (0,2,0) 各自连着 x=1 的墙列
        // (y=0..2),墙脚 (1,−1,0) 与墙顶 (1,3,0) 都是 bedrock → 墙列两端各一条
        // 剪切 18 的汇通路,并联 ≥ 伸根两根 18。横撞割 = 36。
        let mut cells: Vec<(BlockPos, BlockId)> = Vec::new();
        for y in 0..=2 {
            cells.push((p(1, y, 0), builtin::STONE));
        }
        cells.push((p(1, -1, 0), builtin::BEDROCK));
        cells.push((p(1, 3, 0), builtin::BEDROCK));
        cells.push((p(0, 0, 0), builtin::STONE));
        cells.push((p(0, 2, 0), builtin::STONE));
        let world = block_world(cells);
        // d̂ = −x:两根伸出格各自的割 = 剪切 18;源 → 两格各 20,C = 36 < 40。
        let event = ImpactEvent {
            supplies: vec![(p(0, 0, 0), 20.0), (p(0, 2, 0), 20.0)],
            impulse: Vec3::new(-40.0, 0.0, 0.0),
            point: Vec3::new(0.5, 1.0, 0.5),
        };
        let outcome = evaluate_impact_flow(&world, &all_core(), &reg, &event, MAX_CELLS);
        assert_eq!(outcome.cut_q, 36_000);
        assert_eq!(outcome.detached.len(), 2);
        assert_eq!(outcome.components.len(), 2, "互不相邻 → 2 个分量");
        let impactor = ImpactBodyProps {
            mass: 0.729,
            inv_inertia: [[0.122, 0.0, 0.0], [0.0, 0.122, 0.0], [0.0, 0.0, 0.122]],
            center_of_mass: Vec3::new(5.5, 1.5, 0.5),
        };
        let contacts = vec![
            (p(0, 0, 0), Vec3::new(0.5, 0.5, 0.5), 20.0),
            (p(0, 2, 0), Vec3::new(0.5, 2.5, 0.5), 20.0),
        ];
        let (correction, comps) = distribute_momentum(&event, &outcome, &contacts, &impactor);
        assert_eq!(comps.len(), 2);
        let e = event.impulse * 0.1; // E = J·(40−36)/40
        let m_s = 2.0 * 0.729;
        let m_eff = effective_mass(&impactor, event.point, Vec3::new(-1.0, 0.0, 0.0));
        for k in &comps {
            assert!(
                (k.impulse - e * (0.729 / (m_eff + m_s))).length() < 1e-5,
                "P_k = M_k/(m_eff+M_S)·E"
            );
        }
        // 守恒:
        let resid =
            event.impulse * (36.0 / 40.0) + correction + comps[0].impulse + comps[1].impulse
                - event.impulse;
        assert!(resid.length() < 1e-3, "守恒残差 {resid:?}");
        // p_k:低分量格内的接触点 (0.5,0.5,0.5) → p_k = 该点(不是事件作用点)。
        let k_low = comps
            .iter()
            .find(|k| k.cells[0] == p(0, 0, 0))
            .expect("分量按首格升序");
        assert!((k_low.point - Vec3::new(0.5, 0.5, 0.5)).length() < 1e-6);
        // 偏心:接触点挪到格内偏角 → r_k ≠ 0 → Δω ≠ 0。
        let contacts2 = vec![
            (p(0, 0, 0), Vec3::new(0.1, 0.9, 0.5), 20.0),
            (p(0, 2, 0), Vec3::new(0.5, 2.5, 0.5), 20.0),
        ];
        let (_, comps2) = distribute_momentum(&event, &outcome, &contacts2, &impactor);
        let k_low2 = comps2.iter().find(|k| k.cells[0] == p(0, 0, 0)).unwrap();
        assert!(k_low2.delta_w.length() > 1e-3, "偏心接触 Δω ≠ 0");
    }

    /// AC-F4 切段速度:v_seg = Δv_k + Δω_k × (质心_seg − 质心_k)、ω_seg = Δω_k。
    #[test]
    fn segment_velocity_rigid_consistency() {
        let component = ComponentMomentum {
            cells: vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0), p(3, 0, 0)],
            mass: 4.0 * DEBRIS_MASS_PER_CELL,
            point: Vec3::new(0.5, 0.5, 0.5),
            impulse: Vec3::new(-2.0, 0.0, 1.0),
            delta_v: Vec3::new(-1.0, 0.0, 0.5),
            delta_w: Vec3::new(0.0, 0.7, 0.0),
        };
        let segments = vec![vec![p(0, 0, 0), p(1, 0, 0)], vec![p(2, 0, 0), p(3, 0, 0)]];
        let vels = segment_velocity(&component, &segments);
        assert_eq!(vels.len(), 2);
        let com = component_center_of_mass(&component.cells);
        for (seg, (v, w)) in segments.iter().zip(&vels) {
            let com_seg = component_center_of_mass(seg);
            let expect = component.delta_v + component.delta_w.cross(com_seg - com);
            assert!((v - expect).length() < 1e-5, "v_seg = Δv + Δω × r");
            assert!((w - component.delta_w).length() < 1e-7, "ω_seg = Δω_k");
        }
        // 两段速度不同(旋转所致),相对速度 = Δω × (质心差)。
        let rel = vels[0].0 - vels[1].0;
        let dcom = component_center_of_mass(&segments[0]) - component_center_of_mass(&segments[1]);
        assert!((rel - component.delta_w.cross(dcom)).length() < 1e-5);
    }

    // ------------------------------------------- §5.7.1 表 spot-check(模块内)

    /// AC-F3 表 spot-check:3 格厚墙割 = I_c + 4I = 144(逐格打穿,无冲剪塞);
    /// 沙堆横撞割 = 28;1 格宽土桥上砸割 = 14。完整 10 行、±1 量化单位、
    /// EK 对照、打乱不变在 e2e(AC-F3)。
    ///
    /// 会变红的实现:厚墙按冲剪塞(S 非单格);沙堆割按 I_c = 16。
    #[test]
    fn table_spot_checks() {
        let reg = SimpleRegistry::builtin();
        // 3 格厚石墙 x = −3..−1、y = 0..6、z = −4..4,立在 bedrock(x = −4 一列
        // 是 bedrock),正面中部横撞 (−1,3,0)。
        let mut cells: Vec<(BlockPos, BlockId)> = Vec::new();
        for x in -3..=-1 {
            for y in 0..=6 {
                for z in -4..=4 {
                    cells.push((p(x, y, z), builtin::STONE));
                }
            }
        }
        for y in 0..=6 {
            cells.push((p(-4, y, 0), builtin::BEDROCK));
        }
        let world = block_world(cells);
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(-1, 3, 0), 144.002),
            MAX_CELLS,
        );
        assert_eq!(out.cut_q, 144_000, "3 格厚墙:被撞格压向 72 + 四周剪切 72");
        assert_eq!(out.detached, vec![p(-1, 3, 0)], "无冲剪塞,S = 单格");

        // 沙堆横撞(F00 工况 8:被撞格背后压向 16 + 其余剪切,C = 28)。
        // 沙堆下方垫土层(y=−3..−1)+ bedrock(y=−4)(F00 的超平坦地面);
        // 内芯沙 y=−1 覆盖同格 dirt(BTreeMap collect 后写覆盖先写)。
        let mut cells: Vec<(BlockPos, BlockId)> = Vec::new();
        for x in -2..=4 {
            for z in -4..=4 {
                cells.push((p(x, -4, z), builtin::BEDROCK));
                cells.push((p(x, -3, z), builtin::DIRT));
                cells.push((p(x, -2, z), builtin::DIRT));
                cells.push((p(x, -1, z), builtin::DIRT));
            }
        }
        for x in 0..=2 {
            for z in -2..=2 {
                cells.push((p(x, 0, z), builtin::SAND));
            }
        }
        for x in 0..=1 {
            for z in -1..=1 {
                cells.push((p(x, -1, z), builtin::SAND));
            }
        }
        let world = block_world(cells);
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &side_hit(p(2, 0, 0), 28.002),
            MAX_CELLS,
        );
        assert_eq!(
            out.cut_q, 28_000,
            "沙堆:背后压向 16 + 下方剪切 4 + 侧 4 + 上 ∞?——F00 实测 28"
        );
        assert_eq!(out.detached, vec![p(2, 0, 0)]);

        // 1 格宽土桥中部上砸:两端剪切 2×7 = 14(F00 工况 4a)。
        let mut cells: Vec<(BlockPos, BlockId)> =
            (0..=6).map(|x| (p(x, 3, 0), builtin::DIRT)).collect();
        for x in [-1i32, 7] {
            for y in 0..=3 {
                cells.push((p(x, y, 0), builtin::STONE));
            }
            for y in 1..=4 {
                cells.push((p(x, -y, 0), builtin::BEDROCK));
            }
        }
        let world = block_world(cells);
        let out = evaluate_impact_flow(
            &world,
            &all_core(),
            &reg,
            &top_hit(p(3, 3, 0), 14.002),
            MAX_CELLS,
        );
        assert_eq!(out.cut_q, 14_000, "土桥:两端 2 根剪切 14");
        assert_eq!(out.detached, vec![p(3, 3, 0)]);
    }
}
