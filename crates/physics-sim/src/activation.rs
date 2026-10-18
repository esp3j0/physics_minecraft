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
//! - 预算:装卸**合计** ≤ `swap_budget_chunks_per_tick`;醒数超过
//!   `max_awake_dynamic` 时按到最近演员距离降序**立即**冻结超出者(§9.4,不受冷却);
//! - 决策即簿记:被采纳的换装/翻转记入冷却表——同输入连续两次 `plan`,
//!   第二次输出空(有意语义,见 [`ActivationController::plan`] 文档)。

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{CHUNK_SIZE, ChunkPos, ChunkRegion, EntityId, Tick};

/// 激活配置(§4.3 三档;核内独立结构,app::Config 注入,无全局单例)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ActivationConfig {
    /// tier①② 加载半径(m)。
    pub chunk_load_radius: f32,
    /// tier①② 卸载半径(m)。
    pub chunk_unload_radius: f32,
    /// tier③ 唤醒半径(m)。
    pub awake_load_radius: f32,
    /// tier③ 冻结半径(m)。
    pub awake_unload_radius: f32,
    /// 换装/翻转冷却(拍;3s × 60Hz)。
    pub cooldown_ticks: u64,
    /// 醒着动态体上限(§9.4)。
    pub max_awake_dynamic: usize,
    /// 每拍换装预算(装+卸合计)。
    pub swap_budget_chunks_per_tick: usize,
    /// 激活/扩张区外扩 chunk 数(REQ-005 D4):装载候选必须落在
    /// `core ∪ margin_ring`;由 app::Config.margin_meters 换算
    /// (ceil(m/16),默认 16m → 1)。0 显式允许(测试/实验;边界 collider
    /// 可能缺失),生产默认 1。
    pub region_margin_chunks: i32,
}

impl Default for ActivationConfig {
    fn default() -> Self {
        Self {
            chunk_load_radius: 32.0,
            chunk_unload_radius: 48.0,
            awake_load_radius: 24.0,
            awake_unload_radius: 32.0,
            cooldown_ticks: 180,
            max_awake_dynamic: 512,
            swap_budget_chunks_per_tick: 2,
            region_margin_chunks: 1,
        }
    }
}

/// 一轮激活决策(策略纯输出;执行方 = ShardSim / 调用方)。
#[derive(Debug, Default, PartialEq)]
pub struct ActivationDecisions {
    /// 请求装载的 chunk(按到最近演员距离升序;与卸载合计已截断到预算)。
    pub chunks_to_load: Vec<ChunkPos>,
    /// 请求卸载的 chunk(同上)。
    pub chunks_to_unload: Vec<ChunkPos>,
    /// 请求唤醒的实体(到最近演员距离升序,近者先)。
    pub entities_to_wake: Vec<EntityId>,
    /// 请求冻结的实体(到最近演员距离降序,远者先;§9.4 超预算淘汰亦在此)。
    pub entities_to_freeze: Vec<EntityId>,
}

/// 激活策略机:滞回/冷却/预算的簿记与决策(纯内存,serde 可快照)。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActivationController {
    cfg: ActivationConfig,
    chunk_last_swap: BTreeMap<ChunkPos, Tick>,
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

/// chunk 中心(局部 8,8)到最近观察者的水平距离;无观察者 → +∞。
fn chunk_distance(cp: ChunkPos, observers: &[Vec3]) -> f32 {
    let half = CHUNK_SIZE as f32 * 0.5;
    let center = Vec3::new(
        (cp.x * CHUNK_SIZE) as f32 + half,
        0.0,
        (cp.z * CHUNK_SIZE) as f32 + half,
    );
    nearest_observer_distance(observers, center, true).unwrap_or(f32::INFINITY)
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

impl ActivationController {
    pub fn new(cfg: ActivationConfig) -> Self {
        Self {
            cfg,
            chunk_last_swap: BTreeMap::new(),
            entity_last_flip: BTreeMap::new(),
        }
    }

    /// 一轮决策(README 决策 5/7):
    /// - chunk:未载 && 最近距离 ≤ load → 装载;已载 && > unload → 卸载;
    ///   两者均需冷却(该 chunk 上次换装 ≥ cooldown_ticks 拍;首见视为可换);
    ///   装载候选 = 各观察者周围 ceil(unload/16)+2 chunk 网格内未载者;
    ///   装卸合并按距离排序后截断到 swap_budget 合计;
    /// - 实体:醒 && > awake_unload && 冷却 → 冻结;冻 && ≤ awake_load && 冷却 → 唤醒;
    /// - 超预算(醒数 > max_awake_dynamic):按到最近演员距离降序立即冻结
    ///   超出者(不受冷却,§9.4)。
    ///
    /// **决策即簿记**:被采纳的换装/翻转记入冷却表——同一输入两次调用,
    /// 第二次因冷却簿记已更新而输出空(有意语义)。因此超预算淘汰只排除
    /// 「本拍已翻转」的实体:调用方落实决策后不再重复冻结;调用方不落实时
    /// (醒数仍超限)下一拍会继续淘汰——强制语义按拍持续生效。
    ///
    /// **纯决策**:不改施体、不做 IO。无观察者 = 无激活中心(§4.3):
    /// 距离视为 +∞ → 不唤醒任何实体,全体醒着的实体进入(冷却门控的)冻结。
    /// 几何口径:chunk 水平 (x, z) 距离 / 实体 3D 欧氏,均取最近演员。
    ///
    /// 超预算口径:「醒数」= **输入当前醒着的实体数**(冻结体不计;任务书
    /// 测试 `over_budget_freezes_farthest_immediately` 锚定:拍 0 全冻 → 唤醒
    /// 全体不触发预算)。由此「近处全体唤醒」理论上可瞬时越过预算;预算的
    /// 持续强制由后续各拍的淘汰路径保证。
    pub fn plan(
        &mut self,
        observers: &[Vec3],
        loaded: &[ChunkPos],
        entities: &[(EntityId, Vec3, bool)],
        now: Tick,
        region: ChunkRegion,
    ) -> ActivationDecisions {
        let mut d = ActivationDecisions::default();

        // ---- tier①② chunk 档(决策返回给调用方执行,数据源属 REQ-002)----
        // 装载候选网格:各观察者周围 ceil(unload/16)+2;再按区域过滤
        // (REQ-005 D4):区域外(即使距观察者很近)一律不请求——分片不
        // 装载自己 core ∪ margin_ring 之外的 chunk。
        let chunk_f = CHUNK_SIZE as f32;
        let reach = (self.cfg.chunk_unload_radius / chunk_f).ceil() as i32 + 2;
        let mut grid: BTreeSet<ChunkPos> = BTreeSet::new();
        for o in observers {
            let cx = (o.x / chunk_f).floor() as i32;
            let cz = (o.z / chunk_f).floor() as i32;
            for dx in -reach..=reach {
                for dz in -reach..=reach {
                    grid.insert(ChunkPos::new(cx + dx, cz + dz));
                }
            }
        }
        let loaded_set: BTreeSet<ChunkPos> = loaded.iter().copied().collect();
        // 候选集:(chunk, 最近距离, 是否装载);装载候选限网格 ∩ 区域内未载者。
        let mut cands: Vec<(ChunkPos, f32, bool)> = Vec::new();
        for &cp in &grid {
            if loaded_set.contains(&cp) {
                continue;
            }
            // 区域过滤(REQ-005 D4):仅装载候选受区域约束;卸载候选不强制
            // 在区域内(已加载的扩张副本可正常卸载)。
            if !region.contains(cp) {
                continue;
            }
            let dist = chunk_distance(cp, observers);
            // 装载同样受冷却门控(任务书:「两者均需冷却」)——换装决策
            // 采纳即记簿记,冷却窗内重复候选不再产出。
            if dist <= self.cfg.chunk_load_radius && self.chunk_cooled(&cp, now) {
                cands.push((cp, dist, true));
            }
        }
        for &cp in loaded {
            let dist = chunk_distance(cp, observers);
            if dist > self.cfg.chunk_unload_radius && self.chunk_cooled(&cp, now) {
                cands.push((cp, dist, false));
            }
        }
        // 距离升序、同距 (x, z) 字典序(确定性);装卸合计截断到预算。
        cands.sort_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)));
        for (cp, _dist, is_load) in cands {
            if d.chunks_to_load.len() + d.chunks_to_unload.len()
                >= self.cfg.swap_budget_chunks_per_tick
            {
                break;
            }
            // 决策即簿记:被采纳的换装记入冷却表。
            self.chunk_last_swap.insert(cp, now);
            if is_load {
                d.chunks_to_load.push(cp);
            } else {
                d.chunks_to_unload.push(cp);
            }
        }

        // ---- tier③ 实体档 ----
        let mut wake: Vec<(EntityId, f32)> = Vec::new();
        let mut freeze: Vec<(EntityId, f32)> = Vec::new();
        let mut awake: Vec<(EntityId, f32)> = Vec::new();
        for &(id, pos, frozen) in entities {
            let dist = nearest_observer_distance(observers, pos, false).unwrap_or(f32::INFINITY);
            if !frozen {
                awake.push((id, dist));
            }
            if frozen && dist <= self.cfg.awake_load_radius && self.entity_cooled(&id, now) {
                wake.push((id, dist));
            } else if !frozen && dist > self.cfg.awake_unload_radius && self.entity_cooled(&id, now)
            {
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
        let d = c.plan(&obs, &[], &[], Tick::ZERO, core);
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
        let d2 = c2.plan(&outside, &far_loaded, &[], Tick::ZERO, core);
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

    /// 测试清单 1:距离 35(32 < 35 < 48)中带:已载不卸、未载不装。
    /// 观察者取 chunk (0,0) 中心 (8,8);中带 chunk (3,0) 中心 (56,8) 水平距恰 48。
    #[test]
    fn hysteresis_middle_band_holds() {
        let obs = [Vec3::new(8.0, 0.0, 8.0)];
        let mid = [ChunkPos::new(3, 0)];
        let mut c = ActivationController::new(ActivationConfig::default());
        let d = c.plan(&obs, &mid, &[], Tick::ZERO, wide_region());
        assert!(
            !d.chunks_to_load.contains(&ChunkPos::new(3, 0))
                && !d.chunks_to_unload.contains(&ChunkPos::new(3, 0)),
            "距 48(32 < 48 ≤ 48)中带 chunk:不装、不卸"
        );
        // 实体 25(24 < 25 ≤ 32)中带:醒不冻、冻不醒。
        let d = c.plan(
            &obs,
            &[],
            &[
                (eid(1), Vec3::new(33.0, 0.0, 8.0), false),
                (eid(2), Vec3::new(33.0, 0.0, 8.0), true),
            ],
            Tick::ZERO,
            wide_region(),
        );
        assert!(
            d.entities_to_wake.is_empty() && d.entities_to_freeze.is_empty(),
            "距 25 实体:保持现状"
        );
        // 非退化对照:同拍近距离 chunk 会被装载((0,0) 距 0;次近 16m 组按
        // (x, z) 字典序取 (-1,0)),预算 2 截断。
        let mut c2 = ActivationController::new(ActivationConfig::default());
        let d2 = c2.plan(&obs, &[], &[], Tick::ZERO, wide_region());
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
        let d0 = c.plan(&obs, &[], &[], Tick::ZERO, wide_region());
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
            c.plan(&far, &loaded_far, &[], Tick(1), wide_region())
                .chunks_to_unload
                .is_empty()
        );
        assert!(
            c.plan(&far, &loaded_far, &[], Tick(179), wide_region())
                .chunks_to_unload
                .is_empty(),
            "179 拍仍在冷却窗内"
        );
        // 第 180 拍:冷却已过 → 卸载,按距离升序((1,0)@≈976 先于 (0,0)@≈992)。
        let d = c.plan(&far, &loaded_far, &[], Tick(180), wide_region());
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
        let d = c.plan(&obs, &loaded1, &[], Tick::ZERO, wide_region());
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
        let d2 = c2.plan(&obs, &loaded2, &[], Tick::ZERO, wide_region());
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
            &obs,
            &[],
            &[
                (e1, Vec3::new(5.0, 0.0, 0.0), true),
                (e2, Vec3::new(15.0, 0.0, 0.0), true),
                (e3, Vec3::new(20.0, 0.0, 0.0), true),
            ],
            Tick::ZERO,
            wide_region(),
        );
        assert_eq!(d0.entities_to_wake, vec![e1, e2, e3]);
        // 拍 1:3 醒 > 2 → 最远者(e3,距 20)立即冻结,不受冷却(flip@0 才过 1 拍)。
        let d1 = c.plan(
            &obs,
            &[],
            &[
                (e1, Vec3::new(5.0, 0.0, 0.0), false),
                (e2, Vec3::new(15.0, 0.0, 0.0), false),
                (e3, Vec3::new(20.0, 0.0, 0.0), false),
            ],
            Tick(1),
            wide_region(),
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
            &obs,
            &[],
            &[
                (near, Vec3::new(5.0, 0.0, 0.0), false),
                (far, Vec3::new(100.0, 0.0, 0.0), false),
            ],
            Tick::ZERO,
            wide_region(),
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
            &obs,
            &[],
            &[(near, Vec3::new(5.0, 0.0, 0.0), false)],
            Tick::ZERO,
            wide_region(),
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
            &obs,
            &[],
            &[
                (near, Vec3::new(5.0, 0.0, 0.0), false),
                (eid(3), Vec3::new(15.0, 0.0, 0.0), false),
                (eid(4), Vec3::new(20.0, 0.0, 0.0), false),
            ],
            Tick::ZERO,
            wide_region(),
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
            &obs,
            &[],
            &[(eid(3), Vec3::new(24.0, 0.0, 0.0), true)],
            Tick::ZERO,
            wide_region(),
        );
        assert_eq!(d.entities_to_wake, vec![eid(3)], "≤ 24 → 唤醒");
        let d = c.plan(
            &obs,
            &[],
            &[(eid(4), Vec3::new(32.0, 0.0, 0.0), false)],
            Tick::ZERO,
            wide_region(),
        );
        assert!(d.entities_to_freeze.is_empty(), "32 不冻(> 32 才冻)");

        // 完整滞回弧。
        let mut c = ActivationController::new(ActivationConfig::default());
        let e = eid(1);
        let near = (e, Vec3::new(20.0, 0.0, 0.0), true);
        let far = (e, Vec3::new(40.0, 0.0, 0.0), false);
        let d = c.plan(&obs, &[], &[near], Tick::ZERO, wide_region());
        assert_eq!(d.entities_to_wake, vec![e], "冻 @20(≤ 24)→ 唤醒(flip@0)");
        assert!(
            c.plan(&obs, &[], &[far], Tick(1), wide_region())
                .entities_to_freeze
                .is_empty()
        );
        assert!(
            c.plan(&obs, &[], &[far], Tick(179), wide_region())
                .entities_to_freeze
                .is_empty(),
            "> 32 但 179 拍仍在冷却 → 不冻"
        );
        let d = c.plan(&obs, &[], &[far], Tick(180), wide_region());
        assert_eq!(
            d.entities_to_freeze,
            vec![e],
            "> 32 且冷却已过 → 冻结(flip@180)"
        );
        assert!(
            c.plan(&obs, &[], &[near], Tick(181), wide_region())
                .entities_to_wake
                .is_empty()
        );
        assert!(
            c.plan(&obs, &[], &[near], Tick(359), wide_region())
                .entities_to_wake
                .is_empty(),
            "回 ≤ 24 但 179 拍仍在冷却 → 不醒"
        );
        let d = c.plan(&obs, &[], &[near], Tick(360), wide_region());
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
        let d1 = c.plan(&obs, &loaded, &ents, Tick::ZERO, wide_region());
        assert_eq!(d1.chunks_to_load, vec![ChunkPos::new(0, 0)]);
        assert_eq!(d1.chunks_to_unload, vec![ChunkPos::new(3, 1)]);
        assert_eq!(d1.entities_to_wake, vec![e1]);
        assert_eq!(d1.entities_to_freeze, vec![e2]);
        // 同输入第二次:全部被冷却簿记挡下 → 空。
        let d2 = c.plan(&obs, &loaded, &ents, Tick::ZERO, wide_region());
        assert_eq!(
            d2,
            ActivationDecisions::default(),
            "决策即簿记:第二次输出空"
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
}
