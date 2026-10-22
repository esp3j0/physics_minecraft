//! 激活策略机(REQ-001 §4.3 第 3 层;任务书 req001-backlog/02)。
//!
//! [`ActivationController`] 是**纯策略**:输入(观察者/已载 chunk/实体表/当前拍)
//! → 输出 [`ActivationDecisions`];不改施体、不做 IO。执行方:
//! - tier③ 冻结/唤醒 → `ShardSim::apply_activation`(施体类型切换);
//! - tier①② chunk 装卸 → 调用方(数据源属 REQ-002,本轮调度器忽略该返回)。
//!
//! 语义(README 决策 5/7 + §4.3):
//! - 滞回:近半径装载/唤醒、远半径卸载/冻结,中间带保持;
//! - 冷却:同一 chunk/实体两次换装/翻转至少隔 `cooldown_ticks` 拍(首见视为可换);
//! - 预算:① 装卸**合计** ≤ `data_budget_per_tick`,② 构建 ≤
//!   `collider_budget_per_tick`(拆除不计);醒数超过
//!   `max_awake_dynamic` 时按到最近演员距离降序**立即**冻结超出者(§9.4,不受冷却);
//! - 决策即簿记:被采纳的换装/翻转记入冷却表——同输入连续两次 `plan`,
//!   第二次输出空(有意语义,见 [`ActivationController::plan`] 文档)。

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{CHUNK_SIZE, ChunkPos, ChunkRegion, EntityId, Tick};

/// 激活配置(REQ-007 任务 01:①② 拆分;核内独立结构,app::Config 注入,
/// 无全局单例)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ActivationConfig {
    /// ① 数据加载半径(m;REQ-007 任务 01,原 chunk_load_radius)。
    pub data_load_radius: f32,
    /// ① 数据卸载半径(m)。
    pub data_unload_radius: f32,
    /// ② collider 加载半径(m;校验 ≤ data_load_radius,app 配置层)。
    pub collider_load_radius: f32,
    /// ② collider 卸载半径(m)。
    pub collider_unload_radius: f32,
    /// tier③ 唤醒半径(m)。
    pub awake_load_radius: f32,
    /// tier③ 冻结半径(m)。
    pub awake_unload_radius: f32,
    /// 换装/翻转冷却(拍;3s × 60Hz;①② ③ 分档簿记用同值,REQ-007 §4.15)。
    pub cooldown_ticks: u64,
    /// 醒着动态体上限(§9.4)。
    pub max_awake_dynamic: usize,
    /// ① 每拍装+卸合计预算(REQ-007 任务 01,原 swap_budget_chunks_per_tick)。
    pub data_budget_per_tick: usize,
    /// ② 每拍构建预算(拆除不计;REQ-007 任务 01)。
    pub collider_budget_per_tick: usize,
    /// 激活/扩张区外扩 chunk 数(REQ-005 D4):装载候选必须落在
    /// `core ∪ margin_ring`;由 app::Config.margin_meters 换算
    /// (ceil(m/16),默认 16m → 1)。0 显式允许(测试/实验;边界 collider
    /// 可能缺失),生产默认 1。
    pub region_margin_chunks: i32,
    /// 足迹外扩 m(REQ-007 §4.5.2;默认 1.0)。
    pub terrain_guard_m: f32,
    /// 预载足迹前视拍(默认 30 = 0.5s)。
    pub motion_lookahead_ticks: u64,
    /// 前视 chunk 上限(默认 4)。
    pub motion_max_lookahead_chunks: usize,
    /// 瞬态票上限 active_ticks(默认 1200 = 20s)。
    pub transient_settle_deadline_ticks: u64,
    /// 支撑判定法向阈值(默认 0.7)。
    pub support_normal_min: f32,
    /// 虚空移除深度 m(默认 16;y < WORLD_Y_MIN − 深度 即删)。
    pub void_despawn_depth_m: f32,
}

impl Default for ActivationConfig {
    fn default() -> Self {
        Self {
            data_load_radius: 32.0,
            data_unload_radius: 48.0,
            collider_load_radius: 32.0,
            collider_unload_radius: 48.0,
            awake_load_radius: 24.0,
            awake_unload_radius: 32.0,
            cooldown_ticks: 180,
            max_awake_dynamic: 512,
            data_budget_per_tick: 2,
            collider_budget_per_tick: 2,
            region_margin_chunks: 1,
            terrain_guard_m: 1.0,
            motion_lookahead_ticks: 30,
            motion_max_lookahead_chunks: 4,
            transient_settle_deadline_ticks: 1200,
            support_normal_min: 0.7,
            void_despawn_depth_m: 16.0,
        }
    }
}

/// 一轮激活决策(策略纯输出;执行方 = ShardSim / 调用方)。
///
/// REQ-007 任务 01(①② 拆分):`chunks_to_load/unload` 保留原名(B2,语义
/// = ①);② 由 `plan` 决策(唯一决策者,B3)、`ShardSim::apply_activation`
/// 执行;驱动层不执行 ②。
///
/// REQ-007 修订 5(D18)② 执行时序三种情况:随 ① 同点构建
/// ([`Self::colliders_pending_build`],N 拍同决策、① 装载完成点立即建)、
/// 单独构建([`Self::colliders_to_build`],apply_activation 内 step 前)、
/// 拆除(① 卸载点同点拆 = I1;单独拆仅限 ① 仍保留的 chunk,R2)。
#[derive(Debug, Default, PartialEq)]
pub struct ActivationDecisions {
    /// 请求装载的 chunk(① 体素;按到最近演员距离升序;与卸载合计已截断
    /// 到 ① 预算)。
    pub chunks_to_load: Vec<ChunkPos>,
    /// 请求卸载的 chunk(① 体素;含"① 卸载时同步拆 ②"的 chunk)。
    pub chunks_to_unload: Vec<ChunkPos>,
    /// 请求构建 ② 的 chunk(单独构建:① 已在内存、② 缺、距最近演员
    /// ≤ collider_load_radius、冷却已过;ChunkPos 升序,预算内)。
    pub colliders_to_build: Vec<ChunkPos>,
    /// 随 ① 同点构建的 ②(修订 5 D18 情况 1):同一份决策里 ① 装载被采纳、
    /// 且此刻满足 ② 条件(≤ collider_load、② 冷却已过、② 预算有余)的
    /// chunk。N 拍决定;`ShardSim` 暂存为 pending 集合,`apply_chunk_load`
    /// 送 ① 到场时立即构建;未兑现条目在下一次 `apply_activation` 作废并
    /// 重新决策。与 `colliders_to_build` 共享 ② 预算、互不相交。
    pub colliders_pending_build: Vec<ChunkPos>,
    /// 请求拆除 ② 的 chunk(**单独拆除**,R2:仅当 ① 仍被保留——距离
    /// ≤ data_unload_radius——而 ② 已 > collider_unload_radius;与
    /// `chunks_to_unload` 不相交:① 已该卸/排队待卸的 chunk,其 ② 随 ①
    /// 在卸载点一起拆,不提前拆)。
    pub colliders_to_drop: Vec<ChunkPos>,
    /// 请求唤醒的实体(到最近演员距离升序,近者先)。
    pub entities_to_wake: Vec<EntityId>,
    /// 请求冻结的实体(到最近演员距离降序,远者先;§9.4 超预算淘汰亦在此)。
    pub entities_to_freeze: Vec<EntityId>,
}

/// 激活策略机:滞回/冷却/预算的簿记与决策(纯内存,serde 可快照)。
///
/// REQ-007 任务 01(①② 拆分):①/② 各一套冷却簿记表,互不影响。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActivationController {
    cfg: ActivationConfig,
    /// ① 冷却簿记(装/卸共用)。
    chunk_last_swap: BTreeMap<ChunkPos, Tick>,
    /// ② 冷却簿记(构建;重建不进此表)。
    collider_last_build: BTreeMap<ChunkPos, Tick>,
    entity_last_flip: BTreeMap<EntityId, Tick>,
}

/// 实体距离排序键(升序):近者先;同距按 EntityId 升序。
fn cmp_entity_asc(a: &(EntityId, f32), b: &(EntityId, f32)) -> Ordering {
    a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0))
}

/// 实体距离排序键(降序):远者先;同距按 EntityId 升序(确定性)。
fn cmp_entity_desc(a: &(EntityId, f32), b: &(EntityId, f32)) -> Ordering {
    b.1.total_cmp(&a.1).then_with(|| a.0.cmp(&b.0))
}

/// 点 p 到最近观察者的距离。`horizontal = true` 只算水平 (x, z) 分量
/// (chunk 口径,README 决策 7),否则 3D 欧氏(实体口径)。无观察者 → None。
fn nearest_observer_distance(observers: &[Vec3], p: Vec3, horizontal: bool) -> Option<f32> {
    observers
        .iter()
        .map(|o| {
            if horizontal {
                let (dx, dz) = (o.x - p.x, o.z - p.z);
                (dx * dx + dz * dz).sqrt()
            } else {
                (*o - p).length()
            }
        })
        .min_by(|a, b| a.total_cmp(b))
}

/// 票的 ② 规格是否覆盖 chunk(Radius 圆 / Chunks 集合;§14-1:无票种分支,
/// plan 只见 RegionSpec)。
fn collider_covers(t: &crate::tickets::TicketView, cp: &ChunkPos) -> bool {
    match &t.collider {
        crate::tickets::RegionSpec::Radius { load_m, .. } => {
            horizontal_chunk_distance(*cp, t.anchor) <= *load_m
        }
        crate::tickets::RegionSpec::Chunks(chunks) => chunks.contains(cp),
        _ => false,
    }
}

/// chunk 中心(局部 8,8)到锚点的水平距离(票据口径;单票时与
/// `chunk_distance` 同值)。
fn horizontal_chunk_distance(cp: ChunkPos, anchor: Vec3) -> f32 {
    let half = CHUNK_SIZE as f32 * 0.5;
    let center = Vec3::new(
        (cp.x * CHUNK_SIZE) as f32 + half,
        0.0,
        (cp.z * CHUNK_SIZE) as f32 + half,
    );
    let (dx, dz) = (anchor.x - center.x, anchor.z - center.z);
    (dx * dx + dz * dz).sqrt()
}

/// 冷却是否已过:`now - last ≥ cooldown`。tick 回退 = 一致性违规 → panic。
fn cooldown_elapsed(last: Tick, now: Tick, cooldown: u64) -> bool {
    assert!(
        now >= last,
        "tick 回退(一致性违规,fail-stop):now = {:?} < last = {:?}",
        now.0,
        last.0
    );
    now.0 - last.0 >= cooldown
}

/// 装载/卸载候选:(chunk, 最优覆盖键 (priority, dist, order), 是否装载,
/// 是否阻塞请求)。
pub type ChunkCand = (ChunkPos, (u8, f32, u32), bool, bool);

impl ActivationController {
    pub fn new(cfg: ActivationConfig) -> Self {
        Self {
            cfg,
            chunk_last_swap: BTreeMap::new(),
            collider_last_build: BTreeMap::new(),
            entity_last_flip: BTreeMap::new(),
        }
    }

    /// 一轮决策(README 决策 5/7;REQ-007 任务 01:①② 拆分;修订 5 D18 时序):
    /// - ① chunk 数据:未载 && 最近距离 ≤ data_load → 装载;已载 && > data_unload
    ///   → 卸载;冷却门控;装载候选 = 各锚点周围 ceil(data_unload/16)+2 网格;
    ///   装卸合计截断到 data_budget;
    /// - ② collider(B3:plan 是唯一决策者;半径/滞回/预算/冷却全在此),三种
    ///   情况(D18):
    ///   1. **随 ① 同点构建**([`ActivationDecisions::colliders_pending_build`]):
    ///      ① 装载被采纳、且此刻 ≤ collider_load && ② 冷却已过 && ② 预算有余 →
    ///      同拍决定,`ShardSim` 在 ① 装载完成点立即建(消除"N 拍装载/N+1 判距"
    ///      的边界掠过:装载与构建用同拍距离);
    ///   2. **单独构建**([`ActivationDecisions::colliders_to_build`]):① 已载、
    ///      ② 缺、≤ collider_load && 冷却(ChunkPos 升序;与情况 1 共享 ② 预算);
    ///   3. **拆除**:① 卸载决策里的 chunk,② 随 ① 卸载点同点拆(I1,不在本
    ///      决策);**单独拆除**([`ActivationDecisions::colliders_to_drop`],R2)
    ///      仅当 ① 仍被保留:data_unload ≥ 距离 > collider_unload;① 已该卸或
    ///      排队待卸的 chunk,② 不动。拆除记 ② 簿记("决策即簿记"对称);
    ///      拆除不计 ② 构建预算;
    /// - 实体:醒 && > awake_unload && 冷却 → 冻结;冻 && ≤ awake_load && 冷却 → 唤醒;
    /// - 超预算(醒数 > max_awake_dynamic):按到最近演员距离降序立即冻结
    ///   超出者(不受冷却,§9.4)。
    ///
    /// **决策即簿记**:被采纳的换装/翻转记入冷却表——同一输入两次调用,
    /// 第二次输出空(有意语义)。② 构建记 ② 冷却表(重建不受它约束)。
    ///
    /// **纯决策**:不改施体、不做 IO。无锚点 = 无激活中心:距离视为 +∞ →
    /// 不唤醒任何实体,全体醒着的实体进入(冷却门控的)冻结。
    /// 几何口径:chunk 水平 (x, z) 距离 / 实体 3D 欧氏,均取最近锚点。
    ///
    /// 超预算口径:「醒数」= **输入当前醒着的实体数**(冻结体不计;测试
    /// `over_budget_freezes_farthest_immediately` 锚定:拍 0 全冻 → 唤醒
    /// 全体不触发预算)。
    // 8 参(self + 7):任务书 02/03 签名固定(views/①/②/实体/拍/区域/阻塞),
    // 收口成 struct 会破坏任务书接口;§4.12 blocked 输入的判定在调用方。
    #[allow(clippy::too_many_arguments)]
    pub fn plan(
        &mut self,
        tickets: &[crate::tickets::TicketView],
        loaded: &[ChunkPos],
        collider_loaded: &[ChunkPos],
        entities: &[(EntityId, Vec3, bool)],
        now: Tick,
        region: ChunkRegion,
        blocked: &BTreeSet<ChunkPos>,
    ) -> ActivationDecisions {
        let mut d = ActivationDecisions::default();

        // ---- 票据预处理(REQ-007 任务 02;§14-1:只消费 RegionSpec/WakeSpec/
        // priority/state,无票种分支)----
        // 占区域票(Active/Paused;Suspended 不占);锚点 = view.anchor。
        // 唤醒中心 = WakeSpec::Radius 的占区域票(③ 语义)。
        // ①② 覆盖 = 各票 Radius 的 (进, 出) 区间;无 Radius 规格 → 无覆盖。
        let occupying: Vec<&crate::tickets::TicketView> =
            tickets.iter().filter(|t| t.occupies()).collect();
        // 决策用锚点(所有占区域票的锚点;③ 距离中心 = WakeSpec::Radius 票锚点)。
        let wake_centers: Vec<Vec3> = occupying
            .iter()
            .filter(|t| matches!(t.wake, crate::tickets::WakeSpec::Radius { .. }))
            .map(|t| t.anchor)
            .collect();
        // ① 出区域判定用:任一票的 unload 半径覆盖(chunk 到该票锚点 ≤ unload_m)。
        // 仅观察者票(任务 01 等价路径)时 == 现状 data_unload_radius。
        // ③ 淘汰/滞回半径:仅当存在 WakeSpec::Radius 票;取该票半径(本任务
        // 观察者票 = cfg.awake_*,区域票自带)。多票时……任务书细则 5:按
        // "到最近唤醒中心距离"判定,每中心带自己的半径。
        let wake_specs: Vec<(Vec3, f32, f32)> = occupying
            .iter()
            .filter_map(|t| match t.wake {
                crate::tickets::WakeSpec::Radius { load_m, unload_m } => {
                    Some((t.anchor, load_m, unload_m))
                }
                _ => None,
            })
            .collect();
        // 无任何占区域票 → 与"无观察者"同:不唤醒任何实体,全体醒着的
        // 进入(冷却门控的)冻结;①② 无候选。直接走实体档(距离 +∞)。

        // ---- tier① chunk 数据档(决策返回给调用方执行,数据源属 REQ-002)----
        // ① 覆盖 = Σ 占区域票 RegionSpec::Radius(进/出);无 Radius → 无覆盖
        // (任务书细则 4:装载候选 = Σ 票"进"区域 − 已载;卸载候选 = 已载 ∧
        // 不在任何票"出"区域)。
        let chunk_f = CHUNK_SIZE as f32;
        let data_specs: Vec<(Vec3, f32, f32)> = occupying
            .iter()
            .filter_map(|t| match t.data {
                crate::tickets::RegionSpec::Radius { load_m, unload_m } => {
                    Some((t.anchor, load_m, unload_m))
                }
                _ => None,
            })
            .collect();
        // 覆盖集合:任一 Radius 票锚点水平距离 ≤ 半径的 chunk(网格半径 =
        // ceil(半径/16)+1,同旧 reach 形状的保守覆盖)。仅观察者票时
        // (load/unload = cfg 同源半径)与任务 01 行为逐位一致。
        let mut in_cover: BTreeSet<ChunkPos> = BTreeSet::new();
        let mut out_cover: BTreeSet<ChunkPos> = BTreeSet::new();
        for (o, load_m, unload_m) in &data_specs {
            let cx = (o.x / chunk_f).floor() as i32;
            let cz = (o.z / chunk_f).floor() as i32;
            let r_in = (*load_m / chunk_f).ceil() as i32 + 1;
            for dx in -r_in..=r_in {
                for dz in -r_in..=r_in {
                    let cp = ChunkPos::new(cx + dx, cz + dz);
                    if horizontal_chunk_distance(cp, *o) <= *load_m {
                        in_cover.insert(cp);
                    }
                }
            }
            let r_out = (*unload_m / chunk_f).ceil() as i32 + 1;
            for dx in -r_out..=r_out {
                for dz in -r_out..=r_out {
                    let cp = ChunkPos::new(cx + dx, cz + dz);
                    if horizontal_chunk_distance(cp, *o) <= *unload_m {
                        out_cover.insert(cp);
                    }
                }
            }
        }
        // Chunks 规格(REQ-007 任务 03/R1:实体票的足迹由调度器解析成 chunk 集
        // 交给 plan;进=出,无滞回)。通用地并入覆盖(§14-1:无票种分支——
        // plan 只见 RegionSpec)。
        for t in &occupying {
            if let crate::tickets::RegionSpec::Chunks(chunks) = &t.data {
                in_cover.extend(chunks.iter().copied());
                out_cover.extend(chunks.iter().copied());
            }
        }
        let loaded_set: BTreeSet<ChunkPos> = loaded.iter().copied().collect();
        // 候选集:(chunk, best(priority, dist, order), 是否装载)。多票覆盖
        // 同一 chunk 取最优 = 最小 (priority, 距锚点距离, order)(任务书细则 4:
        // 最小 priority、其次最近距离;order 为确定性平局决胜)。
        // 候选:(chunk, best(priority, dist, order), 是否装载, 是否阻塞)。
        let mut cands: Vec<ChunkCand> = Vec::new();
        for &cp in &in_cover {
            if loaded_set.contains(&cp) {
                continue;
            }
            // 区域过滤(REQ-005 D4):仅装载候选受区域约束;卸载候选不强制
            // 在区域内(已加载的扩张副本可正常卸载)。
            if !region.contains(cp) {
                continue;
            }
            // 最优覆盖票(必存在:in_cover 的条目由某票"进"区域产生)。
            let mut key: Option<(u8, f32, u32)> = None;
            for t in &occupying {
                match &t.data {
                    crate::tickets::RegionSpec::Radius { load_m, .. } => {
                        let dist = horizontal_chunk_distance(cp, t.anchor);
                        if dist > *load_m {
                            continue;
                        }
                        let k = (t.priority, dist, t.order);
                        if key.is_none_or(|cur| k < cur) {
                            key = Some(k);
                        }
                    }
                    // Chunks(实体票足迹解析结果):锚点距离 = 到票锚点水平距离
                    // (被卡实体位置,§4.12 阻塞排序同源;无滞回,在集即覆盖)。
                    crate::tickets::RegionSpec::Chunks(chunks) => {
                        if !chunks.contains(&cp) {
                            continue;
                        }
                        let dist = horizontal_chunk_distance(cp, t.anchor);
                        let k = (t.priority, dist, t.order);
                        if key.is_none_or(|cur| k < cur) {
                            key = Some(k);
                        }
                    }
                    _ => {}
                }
            }
            let key = key.expect("in_cover 条目必有覆盖票(构造保证)");
            // 装载同样受冷却门控(任务书:「两者均需冷却」)——换装决策
            // 采纳即记簿记,冷却窗内重复候选不再产出。
            if self.chunk_cooled(&cp, now) {
                // 阻塞请求免冷却(D15;blocked 输入,§14-8)。
                cands.push((cp, key, true, blocked.contains(&cp)));
            }
        }
        // 卸载候选:已载 ∧ 不在任何 Active/Paused 票 ① "出"区域 ∧ 冷却。
        // 排序键:priority = 现存占区域票的最小 priority(卸载是装载的对偶,
        // 与装载同权竞争预算;单票场景 = 该票 priority,与任务 01 纯距离序
        // 一致——AC-T1 golden 依赖此序);dist = 到最近占区域票锚点的水平
        // 距离(无票 → +∞)。
        let min_priority = occupying
            .iter()
            .map(|t| t.priority)
            .min()
            .unwrap_or(u8::MAX);
        let nearest_anchor = |cp: ChunkPos| -> f32 {
            occupying
                .iter()
                .map(|t| horizontal_chunk_distance(cp, t.anchor))
                .fold(f32::INFINITY, f32::min)
        };
        for &cp in loaded {
            if out_cover.contains(&cp) {
                continue;
            }
            if self.chunk_cooled(&cp, now) {
                cands.push((cp, (min_priority, nearest_anchor(cp), 0), false, false));
            }
        }
        // (是否阻塞[阻塞在前], priority, dist, ChunkPos) 升序(§4.12/D15);
        // 装卸合计截断 ① 预算。
        cands.sort_by(|a, b| {
            b.3.cmp(&a.3)
                .then_with(|| a.1.0.cmp(&b.1.0))
                .then_with(|| a.1.1.total_cmp(&b.1.1))
                .then_with(|| a.0.cmp(&b.0))
        });
        // ② 预算余量(修订 5 D18 情况 1:随 ① 同点构建与单独构建共享预算;
        // 单独构建段扣减同一余量)。
        let mut collider_budget_left = self.cfg.collider_budget_per_tick;
        for (cp, _key, is_load, _blocked) in cands {
            if d.chunks_to_load.len() + d.chunks_to_unload.len() >= self.cfg.data_budget_per_tick {
                break;
            }
            // 决策即簿记:被采纳的换装记入冷却表。
            self.chunk_last_swap.insert(cp, now);
            if is_load {
                d.chunks_to_load.push(cp);
                // D18 情况 1:① 装载被采纳且此刻满足 ② 条件(≤ collider_load、
                // ② 冷却已过、② 预算有余)→ ② 同拍决定,随 ① 装载完成点构建。
                // 装载候选已保证 ≤ data_load;若两档半径不同,② 条件更严,
                // 需单独判定。
                // ② 条件(D18 情况 1):任一票的 collider "进"区域覆盖。
                let collider_covered = occupying.iter().any(|t| collider_covers(t, &cp));
                if collider_budget_left > 0 && collider_covered && self.collider_cooled(&cp, now) {
                    collider_budget_left -= 1;
                    self.collider_last_build.insert(cp, now);
                    d.colliders_pending_build.push(cp);
                }
            } else {
                d.chunks_to_unload.push(cp);
            }
        }

        // ---- tier② collider 档(REQ-007 任务 01/B3:决策唯一出处)----
        let collider_loaded_set: BTreeSet<ChunkPos> = collider_loaded.iter().copied().collect();
        // 单独构建(D18 情况 2):① 已在内存、② 缺、≤ collider_load、冷却已过
        // (ChunkPos 升序 = BTreeSet 键序;与 pending 共享 ② 预算)。
        for &cp in &loaded_set {
            if collider_loaded_set.contains(&cp) || !region.contains(cp) {
                continue;
            }
            // ② 覆盖 = 任一票 collider "进"区域。
            let collider_covered = occupying.iter().any(|t| collider_covers(t, &cp));
            if collider_covered && self.collider_cooled(&cp, now) {
                if collider_budget_left == 0 {
                    break;
                }
                collider_budget_left -= 1;
                // 决策即簿记:构建记 ② 冷却表。
                self.collider_last_build.insert(cp, now);
                d.colliders_to_build.push(cp);
            }
        }
        // 单独拆除(D18 情况 3/R2):**仅当 ① 仍被保留**——距离 ≤
        // data_unload_radius(① 未到卸载线)且 > collider_unload_radius。
        // ① 已该卸(> data_unload)或因预算/冷却排队待卸的 chunk,其 ② 保持
        // 不动、随 ① 在卸载点一起拆(I1)。与 chunks_to_unload 不相交(本条件
        // 下天然成立);拆除记 ② 簿记("决策即簿记"对称);不计 ② 构建预算。
        let to_unload_set: BTreeSet<ChunkPos> = d.chunks_to_unload.iter().copied().collect();
        for &cp in collider_loaded {
            if to_unload_set.contains(&cp) {
                continue;
            }
            // ② 超出**所有**占区域票的 collider "出"区域,且 ① 仍被保留
            // (任一票 ① "出"区域覆盖 = out_cover)→ 单独拆 ②(R2/D18 情况 3:
            // 排队待卸的 chunk 不在 out_cover 卸载候选之外时,② 不动)。
            // "出"= 距所有票锚点 > collider_unload_m(非"进"判定——滞回带
            // (load, unload] 内 ② 保持)。
            let collider_out = occupying.iter().all(|t| match &t.collider {
                crate::tickets::RegionSpec::Radius { unload_m, .. } => {
                    horizontal_chunk_distance(cp, t.anchor) > *unload_m
                }
                crate::tickets::RegionSpec::Chunks(chunks) => !chunks.contains(&cp),
                _ => true,
            });
            if collider_out && out_cover.contains(&cp) && self.collider_cooled(&cp, now) {
                self.collider_last_build.insert(cp, now);
                d.colliders_to_drop.push(cp);
            }
        }

        // ---- tier③ 实体档(任务书细则 5:唤醒中心 = WakeSpec::Radius 的
        // 占区域票;每中心带自己的半径;无中心 = 距离 +∞,全体冻结)----
        let mut wake: Vec<(EntityId, f32)> = Vec::new();
        let mut freeze: Vec<(EntityId, f32)> = Vec::new();
        let mut awake: Vec<(EntityId, f32)> = Vec::new();
        // 判定口径:任一中心 ≤ 该中心 load → 进;所有中心 > 各自 unload → 出;
        // 混合滞回带(多中心带宽重叠才出现)保持。仅观察者票时
        // (load/unload = cfg.awake_* 同源)与任务 01 数值等价。
        for &(id, pos, frozen) in entities {
            let dist =
                nearest_observer_distance(&wake_centers, pos, false).unwrap_or(f32::INFINITY);
            let inside = wake_specs
                .iter()
                .any(|(c, load_m, _)| (*c - pos).length() <= *load_m);
            let outside = wake_specs
                .iter()
                .all(|(c, _, unload_m)| (*c - pos).length() > *unload_m);
            let (wake_it, freeze_it) = match (inside, outside) {
                (true, _) => (frozen, false),
                (false, true) => (false, !frozen),
                (false, false) => (false, false), // 滞回带:保持
            };
            if !frozen {
                awake.push((id, dist));
            }
            if wake_it && self.entity_cooled(&id, now) {
                wake.push((id, dist));
            } else if freeze_it && self.entity_cooled(&id, now) {
                freeze.push((id, dist));
            }
        }
        // 超预算(§9.4):当前醒数超限 → 距离降序立即冻结超出者,不受冷却。
        // 溢出计数 = 醒数 − 预算 − **本拍已被滞回决策冻结的醒体数**(决策即簿记:
        // 这些名额已被占,plan 落实后醒数即达标)——否则「近处全体唤醒」后同拍
        // 滞回淘汰最远者时,会把距离序近处的醒体也多余冻结(超出 §9.4「冻结
        // 超出者」最小集)。pool 仍排除已决策者(避免重复);条目仍记簿记
        // (唤醒须等冷却)。扣减后本拍不再需要额外淘汰 → 不产出。
        if awake.len() > self.cfg.max_awake_dynamic {
            let decided: BTreeSet<EntityId> = freeze.iter().map(|(id, _)| *id).collect();
            let already_frozen = freeze.len();
            let overflow = awake
                .len()
                .saturating_sub(already_frozen)
                .saturating_sub(self.cfg.max_awake_dynamic);
            if overflow > 0 {
                let mut pool: Vec<(EntityId, f32)> = awake
                    .iter()
                    .copied()
                    .filter(|(id, _)| !decided.contains(id))
                    .collect();
                pool.sort_by(cmp_entity_desc);
                for (id, dist) in pool.into_iter().take(overflow) {
                    freeze.push((id, dist));
                }
            }
        }
        wake.sort_by(cmp_entity_asc);
        freeze.sort_by(cmp_entity_desc);
        for (id, _) in &wake {
            self.entity_last_flip.insert(*id, now);
            d.entities_to_wake.push(*id);
        }
        for (id, _) in &freeze {
            self.entity_last_flip.insert(*id, now);
            d.entities_to_freeze.push(*id);
        }
        d
    }

    /// 预算强制(§9.12 命令唤醒路径):醒数超限时按距离降序给出应冻结者。
    /// 独立于 plan 的冷却(显式命令触及 = 立即生效)。
    pub fn enforce_budget(&self, observers: &[Vec3], awake: &[(EntityId, Vec3)]) -> Vec<EntityId> {
        if awake.len() <= self.cfg.max_awake_dynamic {
            return Vec::new();
        }
        let overflow = awake.len() - self.cfg.max_awake_dynamic;
        let mut pool: Vec<(EntityId, f32)> = awake
            .iter()
            .map(|(id, pos)| {
                (
                    *id,
                    nearest_observer_distance(observers, *pos, false).unwrap_or(f32::INFINITY),
                )
            })
            .collect();
        pool.sort_by(cmp_entity_desc);
        pool.into_iter().take(overflow).map(|(id, _)| id).collect()
    }

    /// chunk 冷却是否已过(首见视为可换)。
    fn chunk_cooled(&self, cp: &ChunkPos, now: Tick) -> bool {
        match self.chunk_last_swap.get(cp) {
            Some(&last) => cooldown_elapsed(last, now, self.cfg.cooldown_ticks),
            None => true,
        }
    }

    /// ② 构建冷却是否已过(首见视为可建;重建不进 ② 冷却表,不受此约束)。
    fn collider_cooled(&self, cp: &ChunkPos, now: Tick) -> bool {
        match self.collider_last_build.get(cp) {
            Some(&last) => cooldown_elapsed(last, now, self.cfg.cooldown_ticks),
            None => true,
        }
    }

    /// 实体翻转冷却是否已过(首见视为可换)。
    fn entity_cooled(&self, id: &EntityId, now: Tick) -> bool {
        match self.entity_last_flip.get(id) {
            Some(&last) => cooldown_elapsed(last, now, self.cfg.cooldown_ticks),
            None => true,
        }
    }

    /// 记一次实体翻转簿记(crate 内:命令唤醒路径的淘汰条目与 plan 的决策
    /// 同语义——决策即簿记,唤醒须等冷却)。
    pub(crate) fn note_flip(&mut self, id: &EntityId, now: Tick) {
        self.entity_last_flip.insert(*id, now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 任务书 req005/00 测试 2:装载候选受区域过滤——区域内请求、区域外
    /// 即使距观察者很近也不请求;卸载候选不受区域约束。
    #[test]
    fn activation_never_loads_outside_region() {
        // shard(0,0) core、margin = 0(纯本体区;测试用显式语义)。
        let cfg = ActivationConfig {
            region_margin_chunks: 0,
            ..ActivationConfig::default()
        };
        let mut c = ActivationController::new(cfg);
        let core = types::ChunkRegion::core(types::ShardId::new(
            types::WorldId(0),
            types::DimensionId(0),
            types::ShardPos::new(0, 0),
        ));

        // 观察者贴东界(x=120):chunk 7(区域内)与 chunk 8(区域外,中心
        // x=136)都在装载半径内。
        let obs = [Vec3::new(120.0, 0.0, 8.0)];
        let d = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[],
            Tick::ZERO,
            core,
            &std::collections::BTreeSet::new(),
        );
        assert!(
            d.chunks_to_load.contains(&types::ChunkPos::new(7, 0)),
            "区域内且近 → 请求装载:{:?}",
            d.chunks_to_load
        );
        assert!(
            !d.chunks_to_load.contains(&types::ChunkPos::new(8, 0)),
            "区域外即使距观察者 ≤ 半径也不请求:{:?}",
            d.chunks_to_load
        );

        // 卸载不受区域约束:已加载的区域外远 chunk 照常卸载。观察者放在
        // 本体区外(装载候选全被区域过滤 → 预算留给卸载,隔离被测语义)。
        let mut c2 = ActivationController::new(cfg);
        let outside = [Vec3::new(200.0, 0.0, 8.0)];
        let far_loaded = [types::ChunkPos::new(30, 0)];
        let d2 = c2.plan(
            &obs_tickets(&outside),
            &far_loaded,
            &[],
            &[],
            Tick::ZERO,
            core,
            &std::collections::BTreeSet::new(),
        );
        assert!(
            d2.chunks_to_unload.contains(&types::ChunkPos::new(30, 0)),
            "卸载候选不强制在区域内:{:?}",
            d2.chunks_to_unload
        );
    }

    /// 测试用宽区域(覆盖既有测试全部坐标;区域语义本身由
    /// `activation_never_loads_outside_region` 专项测试)。
    fn wide_region() -> types::ChunkRegion {
        types::ChunkRegion::core(types::ShardId::new(
            types::WorldId(0),
            types::DimensionId(0),
            types::ShardPos::new(0, 0),
        ))
        .expanded(64)
    }

    /// 测试用实体身份。
    fn eid(n: u32) -> EntityId {
        EntityId::from_parts(1, n)
    }

    /// 构造一张观察者票(§9-5 既有测试适配辅助):半径取默认激活配置
    /// (32/48、24/32),与任务 01 的 observers 入口逐位一致;order 0 /
    /// priority 0。多观察者 = order 按入参序。
    fn obs_tickets(obs: &[Vec3]) -> Vec<crate::tickets::TicketView> {
        cfg_tickets(&ActivationConfig::default(), obs)
    }

    /// 同上,但半径取测试自定的激活配置(凡 controller 用了非默认 cfg 的
    /// 测试必须用它——票规格与 controller cfg 同源,否则 ①② 覆盖判定与
    /// 旧 observers 语义错位)。
    fn cfg_tickets(cfg: &ActivationConfig, obs: &[Vec3]) -> Vec<crate::tickets::TicketView> {
        obs.iter()
            .map(|&pos| crate::tickets::TicketView {
                order: 0,
                priority: 0,
                data: crate::tickets::RegionSpec::Radius {
                    load_m: cfg.data_load_radius,
                    unload_m: cfg.data_unload_radius,
                },
                collider: crate::tickets::RegionSpec::Radius {
                    load_m: cfg.collider_load_radius,
                    unload_m: cfg.collider_unload_radius,
                },
                wake: crate::tickets::WakeSpec::Radius {
                    load_m: cfg.awake_load_radius,
                    unload_m: cfg.awake_unload_radius,
                },
                state: crate::tickets::TicketState::Active,
                anchor: pos,
            })
            .collect()
    }

    /// 测试清单 1:距离 35(32 < 35 < 48)中带:已载不卸、未载不装。
    /// 观察者取 chunk (0,0) 中心 (8,8);中带 chunk (3,0) 中心 (56,8) 水平距恰 48。
    #[test]
    fn hysteresis_middle_band_holds() {
        let obs = [Vec3::new(8.0, 0.0, 8.0)];
        let mid = [ChunkPos::new(3, 0)];
        let mut c = ActivationController::new(ActivationConfig::default());
        let d = c.plan(
            &obs_tickets(&obs),
            &mid,
            &[],
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert!(
            !d.chunks_to_load.contains(&ChunkPos::new(3, 0))
                && !d.chunks_to_unload.contains(&ChunkPos::new(3, 0)),
            "距 48(32 < 48 ≤ 48)中带 chunk:不装、不卸"
        );
        // 实体 25(24 < 25 ≤ 32)中带:醒不冻、冻不醒。
        let d = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[
                (eid(1), Vec3::new(33.0, 0.0, 8.0), false),
                (eid(2), Vec3::new(33.0, 0.0, 8.0), true),
            ],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert!(
            d.entities_to_wake.is_empty() && d.entities_to_freeze.is_empty(),
            "距 25 实体:保持现状"
        );
        // 非退化对照:同拍近距离 chunk 会被装载((0,0) 距 0;次近 16m 组按
        // (x, z) 字典序取 (-1,0)),预算 2 截断。
        let mut c2 = ActivationController::new(ActivationConfig::default());
        let d2 = c2.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d2.chunks_to_load,
            vec![ChunkPos::new(0, 0), ChunkPos::new(-1, 0)]
        );
    }

    /// 测试清单 2:装载后立刻移远(> 48)→ 179 拍内不卸,第 180 拍可卸。
    /// 远观察者取 (1000,0,0):其邻域 ≤ 32 的 12 个 chunk 以"已载"传入,
    /// 排除装载候选占预算的可能,卸载窗口不被挤占。
    #[test]
    fn cooldown_blocks_flip_back() {
        let obs = [Vec3::new(24.0, 0.0, 8.0)]; // chunk (1,0) 中心
        let far = [Vec3::new(1000.0, 0.0, 0.0)];
        let mut c = ActivationController::new(ActivationConfig::default());
        // 拍 0:近 → 装载(簿记 swap@0);(1,0) 距 0,(0,0) 距 16。
        let d0 = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d0.chunks_to_load,
            vec![ChunkPos::new(1, 0), ChunkPos::new(0, 0)]
        );
        // 远观察者周围 ≤ 32 的 12 个 chunk(逐实测):中心 @0:(62,0);@16:
        // (61,0)/(63,0);@24:(62,±1)/(62,-2);@28.8:(61,±1)/(63,±1)/(61,-2)/(63,-2)。
        // 以"已载"传入。
        let far_ring: Vec<ChunkPos> = [
            (62, 0),
            (61, 0),
            (63, 0),
            (62, 1),
            (62, -1),
            (61, 1),
            (61, -1),
            (63, 1),
            (63, -1),
            (62, -2),
            (61, -2),
            (63, -2),
        ]
        .iter()
        .map(|(x, z)| ChunkPos::new(*x, *z))
        .collect();
        let loaded_far: Vec<ChunkPos> = d0.chunks_to_load.iter().copied().chain(far_ring).collect();
        // 拍 1 / 179:卸载候选但冷却未过 → 不卸。
        assert!(
            c.plan(
                &obs_tickets(&far),
                &loaded_far,
                &[],
                &[],
                Tick(1),
                wide_region(),
                &std::collections::BTreeSet::new()
            )
            .chunks_to_unload
            .is_empty()
        );
        assert!(
            c.plan(
                &obs_tickets(&far),
                &loaded_far,
                &[],
                &[],
                Tick(179),
                wide_region(),
                &std::collections::BTreeSet::new()
            )
            .chunks_to_unload
            .is_empty(),
            "179 拍仍在冷却窗内"
        );
        // 第 180 拍:冷却已过 → 卸载,按距离升序((1,0)@≈976 先于 (0,0)@≈992)。
        let d = c.plan(
            &obs_tickets(&far),
            &loaded_far,
            &[],
            &[],
            Tick(180),
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d.chunks_to_unload,
            vec![ChunkPos::new(1, 0), ChunkPos::new(0, 0)]
        );
        assert!(d.chunks_to_load.is_empty(), "远观察者邻域已载,无装载候选");
    }

    /// 测试清单 3:3 装 + 3 卸候选 → 合计截断到预算 2 且按距离序;
    /// 再验 1 装 + 3 卸时卸载可占预算槽位。
    #[test]
    fn swap_budget_caps_combined_load_unload() {
        let obs = [Vec3::ZERO];
        // 12 个近 chunk(中心距原点 ≤ 32):11.31 ×4 + 25.30 ×8(实测清单)。
        let near = [
            ChunkPos::new(0, 0),
            ChunkPos::new(-1, 0),
            ChunkPos::new(0, -1),
            ChunkPos::new(-1, -1),
            ChunkPos::new(-2, -1),
            ChunkPos::new(-2, 0),
            ChunkPos::new(-1, -2),
            ChunkPos::new(-1, 1),
            ChunkPos::new(0, -2),
            ChunkPos::new(0, 1),
            ChunkPos::new(1, -1),
            ChunkPos::new(1, 0),
        ];
        // 3 个远 chunk(> 48):(3,1) 与 (1,3) 距 ≈60.9,(2,3) 距 ≈68.8。
        let far = [
            ChunkPos::new(3, 1),
            ChunkPos::new(1, 3),
            ChunkPos::new(2, 3),
        ];

        // 相位 1:3 装(留 (0,0),(-1,0),(-1,-1) 未载)+ 3 卸 → 合计 2,按距离序。
        let mut c = ActivationController::new(ActivationConfig::default());
        let loaded1: Vec<ChunkPos> = near
            .iter()
            .copied()
            .filter(|cp| {
                !matches!(
                    cp,
                    ChunkPos { x: 0, z: 0 } | ChunkPos { x: -1, z: 0 } | ChunkPos { x: -1, z: -1 }
                )
            })
            .chain(far)
            .collect();
        let d = c.plan(
            &obs_tickets(&obs),
            &loaded1,
            &[],
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d.chunks_to_load.len() + d.chunks_to_unload.len(),
            2,
            "装卸合计截断到预算 2"
        );
        assert_eq!(
            d.chunks_to_load,
            vec![ChunkPos::new(-1, -1), ChunkPos::new(-1, 0)],
            "同距 11.31 → (x, z) 字典序;更近的装载占满预算"
        );
        assert!(d.chunks_to_unload.is_empty(), "卸载候选被预算截掉");

        // 相位 2:1 装(只留 (0,0) 未载)+ 3 卸 → 卸载进入预算槽位。
        let mut c2 = ActivationController::new(ActivationConfig::default());
        let loaded2: Vec<ChunkPos> = near
            .iter()
            .copied()
            .filter(|cp| *cp != ChunkPos::new(0, 0))
            .chain(far)
            .collect();
        let d2 = c2.plan(
            &obs_tickets(&obs),
            &loaded2,
            &[],
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(d2.chunks_to_load, vec![ChunkPos::new(0, 0)]);
        assert_eq!(
            d2.chunks_to_unload,
            vec![ChunkPos::new(1, 3)],
            "60.9 同距 → x 小者先;(3,1)/(2,3) 被预算截掉"
        );
    }

    /// 测试清单 4:max = 2、3 醒实体 → 最远者冻结,不受冷却
    /// (翻转仅过 1 拍仍立即冻结)。
    #[test]
    fn over_budget_freezes_farthest_immediately() {
        let obs = [Vec3::ZERO];
        let mut c = ActivationController::new(ActivationConfig {
            max_awake_dynamic: 2,
            ..ActivationConfig::default()
        });
        let (e1, e2, e3) = (eid(1), eid(2), eid(3));
        // 拍 0:三只冻结、全部 ≤ 24 → 全醒(簿记 flip@0)。
        let d0 = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[
                (e1, Vec3::new(5.0, 0.0, 0.0), true),
                (e2, Vec3::new(15.0, 0.0, 0.0), true),
                (e3, Vec3::new(20.0, 0.0, 0.0), true),
            ],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(d0.entities_to_wake, vec![e1, e2, e3]);
        // 拍 1:3 醒 > 2 → 最远者(e3,距 20)立即冻结,不受冷却(flip@0 才过 1 拍)。
        let d1 = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[
                (e1, Vec3::new(5.0, 0.0, 0.0), false),
                (e2, Vec3::new(15.0, 0.0, 0.0), false),
                (e3, Vec3::new(20.0, 0.0, 0.0), false),
            ],
            Tick(1),
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(d1.entities_to_freeze, vec![e3], "距离降序淘汰超出者");
        assert!(d1.entities_to_wake.is_empty());
    }

    /// 测试清单 4b:超预算淘汰 = **最小集**——同拍已被滞回决策冻结的实体
    /// 必须从溢出计数中扣减(评审探针:max=1,A 距 100 > 32 予定冻、B 距 5 醒;
    /// 醒数 2 > 1 溢出 1,但 A 已占 1 个冻结名额 → 不得再冻 B)。
    /// 旧实现 `overflow = awake.len() - max` 不扣减 → B(距离序最近)被多余冻结。
    #[test]
    fn overflow_counts_decided_freezes() {
        let obs = [Vec3::ZERO];
        let mut c = ActivationController::new(ActivationConfig {
            max_awake_dynamic: 1,
            ..ActivationConfig::default()
        });
        let (near, far) = (eid(1), eid(2));
        // 拍 0:only-far 冻结计入簿记,确立滞回冻结资格?否——冷却由
        // `entity_last_flip` 驱动,首见即可冻;直接构造同拍双候选:
        // far(距 100 > 32)走滞回冻结,near(距 5,醒)留在醒表。
        let d = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[
                (near, Vec3::new(5.0, 0.0, 0.0), false),
                (far, Vec3::new(100.0, 0.0, 0.0), false),
            ],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d.entities_to_freeze,
            vec![far],
            "溢出 1 已由滞回冻结的 far 占满:near 不得被多余冻结(§9.4 最小集)"
        );
        assert!(d.entities_to_wake.is_empty());

        // 对照(证明场景非退化):去掉 far → near 醒着在预算内,无冻结。
        let mut c2 = ActivationController::new(ActivationConfig {
            max_awake_dynamic: 1,
            ..ActivationConfig::default()
        });
        let d2 = c2.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[(near, Vec3::new(5.0, 0.0, 0.0), false)],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert!(
            d2.entities_to_freeze.is_empty(),
            "预算内醒者不冻(非退化对照)"
        );

        // 对照(证明扣减语义不止"全免"):3 醒(5/15/20)> 1,无滞回冻结候选
        // → 距离降序冻 2 个(20 与 15),近者保留。
        let mut c3 = ActivationController::new(ActivationConfig {
            max_awake_dynamic: 1,
            ..ActivationConfig::default()
        });
        let d3 = c3.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[
                (near, Vec3::new(5.0, 0.0, 0.0), false),
                (eid(3), Vec3::new(15.0, 0.0, 0.0), false),
                (eid(4), Vec3::new(20.0, 0.0, 0.0), false),
            ],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d3.entities_to_freeze,
            vec![eid(4), eid(3)],
            "无滞回冻结可用时按距离降序溢出淘汰"
        );
    }

    /// 测试清单 5:25(24 < 25 < 32)保持;> 32 冷却后冻;回 ≤ 24 冷却后醒。
    /// 另验边界:24 恰唤醒、32 恰不冻。
    #[test]
    fn entity_wake_freeze_hysteresis() {
        let obs = [Vec3::ZERO];
        // 边界:冻 @24 → 醒;醒 @32 → 不冻。
        let mut c = ActivationController::new(ActivationConfig::default());
        let d = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[(eid(3), Vec3::new(24.0, 0.0, 0.0), true)],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(d.entities_to_wake, vec![eid(3)], "≤ 24 → 唤醒");
        let d = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[(eid(4), Vec3::new(32.0, 0.0, 0.0), false)],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert!(d.entities_to_freeze.is_empty(), "32 不冻(> 32 才冻)");

        // 完整滞回弧。
        let mut c = ActivationController::new(ActivationConfig::default());
        let e = eid(1);
        let near = (e, Vec3::new(20.0, 0.0, 0.0), true);
        let far = (e, Vec3::new(40.0, 0.0, 0.0), false);
        let d = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[near],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(d.entities_to_wake, vec![e], "冻 @20(≤ 24)→ 唤醒(flip@0)");
        assert!(
            c.plan(
                &obs_tickets(&obs),
                &[],
                &[],
                &[far],
                Tick(1),
                wide_region(),
                &std::collections::BTreeSet::new()
            )
            .entities_to_freeze
            .is_empty()
        );
        assert!(
            c.plan(
                &obs_tickets(&obs),
                &[],
                &[],
                &[far],
                Tick(179),
                wide_region(),
                &std::collections::BTreeSet::new()
            )
            .entities_to_freeze
            .is_empty(),
            "> 32 但 179 拍仍在冷却 → 不冻"
        );
        let d = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[far],
            Tick(180),
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d.entities_to_freeze,
            vec![e],
            "> 32 且冷却已过 → 冻结(flip@180)"
        );
        assert!(
            c.plan(
                &obs_tickets(&obs),
                &[],
                &[],
                &[near],
                Tick(181),
                wide_region(),
                &std::collections::BTreeSet::new()
            )
            .entities_to_wake
            .is_empty()
        );
        assert!(
            c.plan(
                &obs_tickets(&obs),
                &[],
                &[],
                &[near],
                Tick(359),
                wide_region(),
                &std::collections::BTreeSet::new()
            )
            .entities_to_wake
            .is_empty(),
            "回 ≤ 24 但 179 拍仍在冷却 → 不醒"
        );
        let d = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[near],
            Tick(360),
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(d.entities_to_wake, vec![e], "≤ 24 且冷却已过 → 唤醒");
    }

    /// 测试清单 6:同输入第二次 plan 输出空(决策即簿记,任务书细则 2)。
    #[test]
    fn plan_is_bookkeeping_idempotent() {
        let obs = [Vec3::ZERO];
        let (e1, e2) = (eid(1), eid(2));
        // 11 个近 chunk 已载 + 1 远 chunk 已载:1 装候选 + 1 卸候选(≤ 预算,
        // 全被采纳)。近 chunk 全集见 swap_budget 测试注释(12 个 ≤ 32)。
        let loaded: Vec<ChunkPos> = vec![
            ChunkPos::new(-1, 0),
            ChunkPos::new(0, -1),
            ChunkPos::new(-1, -1),
            ChunkPos::new(-2, -1),
            ChunkPos::new(-2, 0),
            ChunkPos::new(-1, -2),
            ChunkPos::new(-1, 1),
            ChunkPos::new(0, -2),
            ChunkPos::new(0, 1),
            ChunkPos::new(1, -1),
            ChunkPos::new(1, 0),
            ChunkPos::new(3, 1),
        ];
        let ents = [
            (e1, Vec3::new(8.0, 0.0, 8.0), true), // 冻、距 ≈11.3 ≤ 24 → 唤
            (e2, Vec3::new(100.0, 0.0, 0.0), false), // 醒、距 100 > 32 → 冻
        ];
        let mut c = ActivationController::new(ActivationConfig::default());
        // REQ-007 任务 01:plan 需要 ①/② 两档状态输入;调用方执行决策后把
        // 产出并入状态(① 由驱动装载、② 由 apply_activation 构建)。
        let mut collider_loaded: Vec<ChunkPos> = Vec::new();
        let d1 = c.plan(
            &obs_tickets(&obs),
            &loaded,
            &collider_loaded,
            &ents,
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(d1.chunks_to_load, vec![ChunkPos::new(0, 0)]);
        assert_eq!(d1.chunks_to_unload, vec![ChunkPos::new(3, 1)]);
        assert_eq!(d1.entities_to_wake, vec![e1]);
        assert_eq!(d1.entities_to_freeze, vec![e2]);
        collider_loaded = d1.colliders_to_build.clone();
        // 同输入(状态随第一次决策前移)第二次:①③ 被冷却簿记挡下 → 空;
        // ② 构建有积压(半径内未建 ① 多于 ② 预算)→ 逐拍续取(REQ-007 任务
        // 01 新语义:② 预算截断的候选下拍重提,直到建满/冷却)。
        let d2 = c.plan(
            &obs_tickets(&obs),
            &loaded,
            &collider_loaded,
            &ents,
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert!(d2.chunks_to_load.is_empty() && d2.chunks_to_unload.is_empty());
        assert!(d2.entities_to_wake.is_empty() && d2.entities_to_freeze.is_empty());
        assert!(
            !d2.colliders_to_build.is_empty() && d2.colliders_to_build != d1.colliders_to_build,
            "② 积压续取:不与第一拍重复(已建集合已前移)"
        );
    }

    /// R2:① 已该卸(> data_unload)但因冷却排队 → ② 不单独拆(随 ① 卸载
    /// 点同点拆,不提前);单独拆 ② 仅当 ① 仍被保留(data_unload ≥ 距离 >
    /// R2(架构师验收指示:真实预算排队场景):① 预算 1 下多个卸载候选排队,
    /// 排队者的 ② 不提前拆(随 ① 卸载点同点拆);合法单独拆路径照常工作。
    /// 场景(② 卸 32 < ① 卸 48,① 预算 1、② 预算 2):
    /// - A = (4,0) 距 40 ∈ (32,48]:② 单独拆、① 保留(合法路径);
    /// - B = (6,0) 距 72 > 48:① 该卸(拍 0 采纳,预算 1);② 不单独拆
    ///   (> 48,R2);
    /// - C = (8,0) 距 104 > 48:① 该卸但预算尽 → 排队(不记簿记,下拍重提);
    ///   ② 同样不提前拆。
    ///
    /// 拍 1(状态前移:A ② 已拆,B/C 仍载已建):B ① 卸载被冷却挡(采纳过
    /// 即簿记,@0)→ C 重提(未采纳过,无簿记)——真实排队的续取语义;
    /// B/C ② 依旧不单独拆。
    #[test]
    fn queued_unload_keeps_collider_intact() {
        let cfg = ActivationConfig {
            collider_unload_radius: 32.0,
            data_load_radius: 0.0,
            data_budget_per_tick: 1,
            collider_budget_per_tick: 2,
            ..ActivationConfig::default()
        };
        let obs = [Vec3::new(32.0, 0.0, 8.0)];
        let mut c = ActivationController::new(cfg);
        let a = ChunkPos::new(4, 0); // 中心 (72,8) 距 40
        let b = ChunkPos::new(6, 0); // 中心 (104,8) 距 72
        let cc = ChunkPos::new(8, 0); // 中心 (136,8) 距 104
        let loaded = [a, b, cc];
        let collider = [a, b, cc];
        // 拍 0。
        let d0 = c.plan(
            &cfg_tickets(&cfg, &obs),
            &loaded,
            &collider,
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d0.chunks_to_unload,
            vec![b],
            "① 预算 1:距 96 的 B 被采纳(卸载键 = 最近锚点距离升序);C 排队"
        );
        assert_eq!(
            d0.colliders_to_drop,
            vec![a],
            "A 距 40 ∈ (32,48]:② 单独拆(合法路径,R2)"
        );
        assert!(
            !d0.colliders_to_drop.contains(&b) && !d0.colliders_to_drop.contains(&cc),
            "排队/该卸者的 ② 不提前拆(随 ① 卸载点同点拆)"
        );
        assert!(
            d0.colliders_to_build.is_empty() && d0.colliders_pending_build.is_empty(),
            "全部已建,无构建"
        );
        // 拍 1:状态前移(A 的 ② 已拆);驱动未执行 B 卸载(B 仍载)。
        let d1 = c.plan(
            &cfg_tickets(&cfg, &obs),
            &loaded,
            &[b, cc],
            &[],
            Tick(1),
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert!(
            !d1.chunks_to_unload.contains(&b),
            "B ① 卸载冷却 @0(拍 0 采纳即簿记)→ 排队"
        );
        assert_eq!(
            d1.chunks_to_unload,
            vec![cc],
            "C 未采纳过(无簿记)→ 真实排队续取"
        );
        assert!(
            !d1.colliders_to_drop.contains(&b) && !d1.colliders_to_drop.contains(&cc),
            "排队者的 ② 不提前拆(任何拍)"
        );
    }

    /// 测试清单 7:命令唤醒路径的超限淘汰按距离降序;预算内返回空。
    #[test]
    fn enforce_budget_picks_farthest() {
        let c = ActivationController::new(ActivationConfig {
            max_awake_dynamic: 2,
            ..ActivationConfig::default()
        });
        let ids = [eid(1), eid(2), eid(3), eid(4)];
        let awake = [
            (ids[0], Vec3::new(5.0, 0.0, 0.0)),
            (ids[1], Vec3::new(10.0, 0.0, 0.0)),
            (ids[2], Vec3::new(15.0, 0.0, 0.0)),
            (ids[3], Vec3::new(20.0, 0.0, 0.0)),
        ];
        assert_eq!(
            c.enforce_budget(&[Vec3::ZERO], &awake),
            vec![ids[3], ids[2]],
            "超限 2 个:距离降序淘汰最远者"
        );
        assert!(
            c.enforce_budget(&[Vec3::ZERO], &awake[..2]).is_empty(),
            "预算内 → 不淘汰"
        );
    }

    // ==== REQ-007 任务 01 新增(①② 拆分;任务书测试清单)====

    /// I1:卸 ① 必同步拆 ②(执行点 = unload_chunk;此处锚定 plan 决策侧:
    /// ① 卸载决策与 ② 拆除决策不相交——同 chunk 的 ② 随 ① 卸载同点拆,N2)。
    #[test]
    fn i1_unload_data_and_collider_drop_disjoint() {
        let obs = [Vec3::new(200.0, 0.0, 8.0)]; // 远锚点:(3,1) ①② 双超半径
        // 关 ① 装载候选:锚点邻域的近距装载候选会按距离升序占满 ① 预算,
        // 卸载候选 (3,1)(距 144.9)永远轮空。半径归零后仅锚点所在 chunk
        // (12,0)(距 0)仍是候选,预算 2 内装 1 卸 1,卸载决策可达。
        let cfg = ActivationConfig {
            data_load_radius: 0.0,
            ..ActivationConfig::default()
        };
        let mut c = ActivationController::new(cfg);
        let loaded = [ChunkPos::new(3, 1)];
        let collider = [ChunkPos::new(3, 1)];
        let d = c.plan(
            &cfg_tickets(
                &ActivationConfig {
                    data_load_radius: 0.0,
                    ..ActivationConfig::default()
                },
                &obs,
            ),
            &loaded,
            &collider,
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d.chunks_to_unload,
            vec![ChunkPos::new(3, 1)],
            "① 超半径 → 卸"
        );
        assert!(
            !d.colliders_to_drop.contains(&ChunkPos::new(3, 1)),
            "N2:同 chunk 的 ② 不重复产出拆除(随 ① 卸载同点拆,I1)"
        );
    }

    /// ② 先拆:两档半径不同(② 卸 32 < ① 卸 48),32–48 m 环带 ② 拆、① 保留
    /// (B3 补漏路径:有 ① 无 ② 状态)。
    #[test]
    fn collider_drop_before_data_drop() {
        let cfg = ActivationConfig {
            collider_unload_radius: 32.0,
            ..ActivationConfig::default()
        };
        // 中心 (72,8):距 12.7 ≤ 32。取 (3,0) 中心 (56,8):距 4。
        // 需要一个 ① 在 32–48 带内:chunk (4,0) 中心 (72,8) 距锚 (40,8) = 32.x?
        // 直接构造:锚点 (8,8);(4,0) 中心 (72,8) 距 64.06 > 48 → ①② 双拆,
        // 不合用。改锚 (56,8):与 (4,0) 中心 (72,8) 距 16 ∈ (32 带外?)
        // 16 < 32 → ② 不拆。需精确 32<dist≤48 对 (4,0):锚 x = 72-40 = 32 → (32,8)。
        let obs2 = [Vec3::new(32.0, 0.0, 8.0)];
        let mut c = ActivationController::new(cfg);
        let loaded = [ChunkPos::new(4, 0)];
        let collider = [ChunkPos::new(4, 0)];
        let d = c.plan(
            &cfg_tickets(&cfg, &obs2),
            &loaded,
            &collider,
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        let center_dist = {
            let dx = 72.0_f32 - 32.0;
            (dx * dx).sqrt()
        };
        assert!(
            center_dist > 32.0 && center_dist <= 48.0,
            "前置:(4,0) 中心距锚应在 (32,48] 带:实测 {center_dist}"
        );
        assert!(
            d.chunks_to_unload.is_empty(),
            "① 在 48 卸载半径内 → 保留:{:?}",
            d.chunks_to_unload
        );
        assert_eq!(
            d.colliders_to_drop,
            vec![ChunkPos::new(4, 0)],
            "② 超 32 卸载半径且 ① 保留 → 单独拆 ②(有①无②状态)"
        );
    }

    /// ② 构建前提:① 未载不候选(即使半径内);预算独立:① 预算 0 不建 ②,
    /// ② 预算 1 时 ①② 各自推进;pending(随 ① 同点)与单独构建共享 ② 预算
    /// (修订 5 D18)。
    #[test]
    fn collider_budget_independent_of_data() {
        // ① 预算 0:无装载 → loaded 恒空 → ② 候选为空(前提:① 已载)。
        let cfg0 = ActivationConfig {
            data_budget_per_tick: 0,
            ..ActivationConfig::default()
        };
        let obs = [Vec3::new(8.0, 0.0, 8.0)];
        let mut c0 = ActivationController::new(cfg0);
        let d = c0.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert!(d.chunks_to_load.is_empty(), "① 预算 0");
        assert!(d.colliders_to_build.is_empty(), "① 未载 → ② 不候选(前提)");
        assert!(d.colliders_pending_build.is_empty());

        // ①② 预算各 1:loaded=近 chunk 已载未建 → ② 单独建一个(占预算);
        // ① 装载 (0,0) 的 pending 判定因预算已耗尽 → 不产出。
        let cfg1 = ActivationConfig {
            data_budget_per_tick: 1,
            collider_budget_per_tick: 1,
            ..ActivationConfig::default()
        };
        let mut c1 = ActivationController::new(cfg1);
        let d = c1.plan(
            &obs_tickets(&obs),
            &[ChunkPos::new(-1, 0)],
            &[],
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d.chunks_to_load,
            vec![ChunkPos::new(0, 0)],
            "① 按预算 1 装载"
        );
        // 共享预算(D18):装载采纳的 pending 先占 ② 预算 1 → 已载未建的
        // (-1,0) 单独构建被饿(下拍续取,机制同 ① 预算截断)。
        assert_eq!(
            d.colliders_pending_build,
            vec![ChunkPos::new(0, 0)],
            "装载候选的 ② 同拍 pending(占 ② 预算)"
        );
        assert!(
            d.colliders_to_build.is_empty(),
            "② 预算被 pending 占满 → 单独构建饿(下拍续取)"
        );
        // 下拍:状态前移((0,0) 已载已建、(-1,0) 已载未建;关 ① 装载消额外
        // 候选);(-1,0) 首见 ② 可建(① 簿记与 ② 簿记独立)→ 单独构建产出
        // (被饿的单独构建下拍续取,机制同 ① 预算截断)。
        let cfg2 = ActivationConfig {
            data_load_radius: 0.0,
            ..cfg1
        };
        let mut c2 = ActivationController::new(cfg2);
        let d2 = c2.plan(
            &cfg_tickets(&cfg2, &obs),
            &[ChunkPos::new(-1, 0), ChunkPos::new(0, 0)],
            &[ChunkPos::new(0, 0)],
            &[],
            Tick(1),
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert!(d2.chunks_to_load.is_empty() && d2.colliders_pending_build.is_empty());
        assert_eq!(
            d2.colliders_to_build,
            vec![ChunkPos::new(-1, 0)],
            "被饿的单独构建下拍续取"
        );
    }

    /// D18 情况 1(R1):同一份决策里 ① 装载被采纳、且满足 ② 条件(≤
    /// collider_load、② 冷却、② 预算余)→ 进 `colliders_pending_build`,
    /// 与装载同拍决定(消除"N 拍装载/N+1 判距"的边界掠过)。
    #[test]
    fn collider_pending_build_with_data_load() {
        let obs = [Vec3::new(8.0, 0.0, 8.0)];
        let mut c = ActivationController::new(ActivationConfig::default());
        let d = c.plan(
            &obs_tickets(&obs),
            &[],
            &[],
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        // ① 预算 2:距 0 的 (0,0) + 距 16 的 (-1,0)(候选排序距离升序)。
        assert_eq!(
            d.chunks_to_load,
            vec![ChunkPos::new(0, 0), ChunkPos::new(-1, 0)],
            "① 装载决策(锚点所在 chunk 最近,预算 2 内)"
        );
        // 两个装载候选都 ≤ collider_load(32)且 ② 预算 2 有余 → 全部 pending。
        assert_eq!(
            d.colliders_pending_build, d.chunks_to_load,
            "每个装载候选 ② 条件满足即同拍 pending(随 ① 同点构建)"
        );
        assert!(d.colliders_to_build.is_empty(), "无已载未建 → 无单独构建");
    }

    /// ② 冷却独立(含 pending):装载/构建决策后同拍再判 → ② 冷却挡下
    /// (pending 不重复产出);重建不进 ② 簿记(sim 侧直执行,不经 plan)。
    #[test]
    fn collider_cooldown_separate_from_data() {
        let obs = [Vec3::new(8.0, 0.0, 8.0)];
        let mut c = ActivationController::new(ActivationConfig::default());
        // 拍 0:装载 (0,0)(pending)与已载 (-1,0) 单独构建,双双记 ② 簿记 @0。
        let d = c.plan(
            &obs_tickets(&obs),
            &[ChunkPos::new(-1, 0)],
            &[],
            &[],
            Tick::ZERO,
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        // 预算 2 全给装载 [(0,0) 距 0,(0,-1) 距 16 首个] + pending;已载未建的
        // (-1,0) 单独构建被饿(共享预算,D18)。
        assert!(
            d.colliders_to_build.is_empty(),
            "预算被 pending 占满 → 单独构建饿"
        );
        assert_eq!(
            d.colliders_pending_build,
            vec![ChunkPos::new(0, 0), ChunkPos::new(0, -1)]
        );
        assert_eq!(
            d.chunks_to_load,
            vec![ChunkPos::new(0, 0), ChunkPos::new(0, -1)]
        );
        // 拍 1(冷却未过):(0,0) 已装载(输入前移)、(-1,0) 已建(输入前移)
        // → 两路全空;同拍 SetBlock 重建是 sim 内部行为(不经 plan),无"重建
        // 决策"通道可断言——非退化由 sim::setblock_on_data_without_collider
        // 覆盖重建路径。
        // 拍 1 状态前移:输入只载拍 0 装的两个;新候选 (0,1) 距 16 照常
        // 装载+pending(首见 ② 可建,合法);**已决策过的 (0,0)/(0,-1) 不重复
        // 出现**(① ② 簿记 @0 双重挡下)——这就是"决策即簿记"的分档证据。
        let d = c.plan(
            &obs_tickets(&obs),
            &[
                ChunkPos::new(-1, 0),
                ChunkPos::new(0, 0),
                ChunkPos::new(0, -1),
            ],
            &[
                ChunkPos::new(-1, 0),
                ChunkPos::new(0, 0),
                ChunkPos::new(0, -1),
            ],
            &[],
            Tick(1),
            wide_region(),
            &std::collections::BTreeSet::new(),
        );
        assert_eq!(
            d.chunks_to_load,
            vec![ChunkPos::new(0, 1), ChunkPos::new(1, 0)],
            "仅新候选(距 16 批次;预算 2)"
        );
        assert_eq!(
            d.colliders_pending_build,
            vec![ChunkPos::new(0, 1), ChunkPos::new(1, 0)]
        );
        assert!(
            !d.chunks_to_load.contains(&ChunkPos::new(0, 0))
                && !d.chunks_to_load.contains(&ChunkPos::new(0, -1)),
            "① 簿记 @0:已装载候选不重复"
        );
        assert!(
            !d.colliders_pending_build.contains(&ChunkPos::new(0, 0))
                && !d.colliders_pending_build.contains(&ChunkPos::new(0, -1)),
            "② 簿记 @0:已 pending 过的候选不重复"
        );
        assert!(d.colliders_to_build.is_empty() && d.colliders_to_drop.is_empty());
    }

    /// 有 ① 无 ② 的 chunk 收 SetBlock:体素生效、本 chunk 不建 ②、已有 ② 的
    /// 邻居照常重建(sim 侧;plan 不建 ② 的路径)。
    #[test]
    fn setblock_on_data_without_collider() {
        use crate::sim::{ShardSim, ShardSimConfig};
        use messaging::{InputPayload, SimCommand, StampedInput};
        use types::{BlockPos, DimensionId, ShardPos, WorldId};
        use voxel::SimpleRegistry;

        let shard = types::ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0));
        let mut s = ShardSim::new(
            ShardSimConfig::new(shard),
            Box::new(SimpleRegistry::builtin()),
        );
        // 实心 chunk(每格 stone):面剔除后至少一个 collider。
        let solid = {
            let mut ch = voxel::Chunk::air();
            for x in 0..types::CHUNK_SIZE {
                for y in 0..4 {
                    for z in 0..types::CHUNK_SIZE {
                        ch.set(x as usize, y as usize, z as usize, voxel::BlockId(1));
                    }
                }
            }
            ch
        };
        // 只装 ①(启用模式驱动路径):chunk (0,0) 有 ① 无 ②。
        s.load_data_only(ChunkPos::new(0, 0), solid.clone());
        assert!(!s.has_collider(ChunkPos::new(0, 0)), "① 在 ② 不在");
        // SetBlock 写入该 chunk:体素生效;本 chunk 不建 ②(有①无② 路径)。
        let input = StampedInput {
            apply_at: Tick::ZERO,
            source: messaging::InputSource(1),
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SetBlock {
                pos: BlockPos::new(0, types::WORLD_Y_MIN + 1, 0),
                block: voxel::BlockId(1),
            }),
        };
        s.apply_prefix(&[input]);
        assert!(
            !s.has_collider(ChunkPos::new(0, 0)),
            "有①无② chunk 写块不建 ②"
        );
        // 邻居照常:给邻居先建 ②(合一装载),再写邻居 → ② 重建(非退化)。
        s.load_chunk(ChunkPos::new(1, 0), solid);
        assert!(s.has_collider(ChunkPos::new(1, 0)), "合一装载建 ②(对照)");
    }
}
