//! 玩家回出生点簿记(REQ-012 A06 R1,§6"玩家回出生点")。
//!
//! 纯逻辑模块:三张表(出生点、玩家身体归属、重生待附身)收在这里,
//! 不碰 [`sim_api::session::Session`]——单测直接推时间,不起网络/内核。
//! 全部驱动内存,不进快照(重生动作走与网络请求同一条校验/日志路径,
//! 回放线由日志里的 SpawnAgent/Possess 自再生,D6 同款口径)。
//!
//! 要点(返修单 3.1):
//! - 待附身期限**按条目算**(`issued_at`),没有跨连接共用的计数器——
//!   上一版的全局 `respawn_possess_wait` 计数器会在第二次重生时把第一
//!   次用掉的宽限算进头,直接误判失败(P1 缺陷)。
//! - `bodies` 里因断线移除等路径漏清的死条目无害:实体 ID 不复用,
//!   断线时 `on_disconnect` 把该连接三张表的条目一起清。

use std::collections::BTreeMap;

use messaging::InputSource;
use types::glam::Vec3;
use types::{BlockPos, EntityId, Tick};

/// 等出生点 chunk 装载的上限(拍;§6"等区块装载"修订)。
pub const RESPAWN_LOAD_TIMEOUT_TICKS: u64 = 600;
/// 安全落点的水平搜索半径(格,切比雪夫距离;§6"安全落点"修订)。
pub const SAFE_SEARCH_RADIUS: i32 = 8;
/// 换列时身体根位置比地面顶面高多少(与 viewer 按 P 生成同口径:0.9 半高
/// + 0.1 余量)。
pub const SAFE_ROOT_ABOVE_GROUND: f32 = 1.0;

/// 一个连接的出生点:最近一次被接受的 `SpawnAgent` 的位置、朝向与身体名。
#[derive(Debug, Clone, PartialEq)]
pub struct SpawnPoint {
    pub position: Vec3,
    pub yaw: f32,
    pub body: String,
}

/// 重生后等待附身的条目:目标实体 + 发起拍(宽限从这一拍起算)。
#[derive(Debug, Clone, Copy, PartialEq)]
struct PendingPossess {
    entity: EntityId,
    issued_at: Tick,
}

/// 重生等区块装载的条目:出生点 + 发起拍(装载超时从这一拍起算)。
#[derive(Debug, Clone, PartialEq)]
struct PendingRespawn {
    point: SpawnPoint,
    issued_at: Tick,
}

/// 附身宽限(拍):生成命令 `apply_at` = 发起拍(分片收件箱下一拍 tick
/// 应用),之后实体应出现在内核;超过宽限仍不出现 = 生成被拒,计失败。
pub const POSSESS_GRACE_TICKS: u64 = 3;

/// [`poll_possess`] 的逐条结果。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PossessPoll {
    /// 实体已在内核出现 → 走 `handle_possess`(成功计 `player_respawned`)。
    Ready(InputSource, EntityId),
    /// 超过宽限仍不出现 → 计 `player_respawn_failed` + 补发 `Possessed(None)`。
    TimedOut(InputSource, EntityId),
}

/// 重生簿记:连接 → 出生点、玩家身体 → 连接、连接 → 待附身、连接 →
/// 重生等装载。
#[derive(Debug, Default)]
pub struct RespawnBook {
    /// 连接 → 最近一次被接受的出生点(`on_spawn_accepted` 覆盖写)。
    spawn_points: BTreeMap<InputSource, SpawnPoint>,
    /// 玩家身体 → 连接(`on_void_removed` 反查归属用)。
    bodies: BTreeMap<EntityId, InputSource>,
    /// 连接 → 重生后等待附身的条目。
    pending: BTreeMap<InputSource, PendingPossess>,
    /// 连接 → 重生等区块装载的条目(§6"等区块装载"修订)。
    pending_respawn: BTreeMap<InputSource, PendingRespawn>,
}

impl RespawnBook {
    /// 生成被接受(`handle_spawn_agent` 成功处调用):出生点覆盖写、身体
    /// 归属登记。
    pub fn on_spawn_accepted(&mut self, player: InputSource, entity: EntityId, point: SpawnPoint) {
        self.spawn_points.insert(player, point);
        self.bodies.insert(entity, player);
    }

    /// 连接断开:四张表里该连接的条目一起清(出生点失效、身体归属不再
    /// 有效——断线移除会把它移除;待附身等待与重生等装载也没有意义)。
    pub fn on_disconnect(&mut self, player: InputSource) {
        self.spawn_points.remove(&player);
        self.bodies.retain(|_, owner| *owner != player);
        self.pending.remove(&player);
        self.pending_respawn.remove(&player);
    }

    /// 虚空移除:是玩家身体就返回 (连接, 出生点) 并从归属表删掉;
    /// AI 主体 / 未知实体 → `None`(不重生)。
    pub fn on_void_removed(&mut self, entity: EntityId) -> Option<(InputSource, SpawnPoint)> {
        let player = self.bodies.remove(&entity)?;
        let point = self.spawn_points.get(&player).cloned()?;
        Some((player, point))
    }

    /// 出生点只读视图(测试断言"出生点取最近一次"用)。
    pub fn spawn_point(&self, player: InputSource) -> Option<&SpawnPoint> {
        self.spawn_points.get(&player)
    }

    /// 出生点改回给定值(§6"安全落点":重生用的落点不覆盖出生点——
    /// `handle_spawn_agent` 成功路径的覆盖写会把换列落点当成新出生点,
    /// 重生路径生成成功后用它还原)。
    pub fn restore_spawn_point(&mut self, player: InputSource, point: SpawnPoint) {
        self.spawn_points.insert(player, point);
    }

    /// 登记重生等区块装载(§6"等区块装载"):驱动查到玩家身体被虚空移除
    /// 后先入表,装载好之前不生成。
    pub fn begin_respawn_wait(&mut self, player: InputSource, point: SpawnPoint, now: Tick) {
        self.pending_respawn.insert(
            player,
            PendingRespawn {
                point,
                issued_at: now,
            },
        );
    }

    /// 等待中的条目,按连接升序:`(连接, 出生点, 发起拍)`。
    pub fn pending_respawns(&self) -> Vec<(InputSource, SpawnPoint, Tick)> {
        self.pending_respawn
            .iter()
            .map(|(&p, e)| (p, e.point.clone(), e.issued_at))
            .collect()
    }

    /// 出表,返回是否原来在表里。
    pub fn end_respawn_wait(&mut self, player: InputSource) -> bool {
        self.pending_respawn.remove(&player).is_some()
    }

    /// 该连接是否在重生等装载的等待中(等待期间忽略其 `MoveObserver`)。
    pub fn is_respawn_pending(&self, player: InputSource) -> bool {
        self.pending_respawn.contains_key(&player)
    }

    /// 重生生成成功后登记待附身(下一拍实体在内核出现后再走
    /// `handle_possess`)。
    pub fn begin_possess_wait(&mut self, player: InputSource, entity: EntityId, now: Tick) {
        self.pending.insert(
            player,
            PendingPossess {
                entity,
                issued_at: now,
            },
        );
    }

    /// 每拍一次,逐条判定待附身条目:
    /// - `exists(entity)` 为真 → [`PossessPoll::Ready`] 并出表;
    /// - `now ≥ issued_at + POSSESS_GRACE_TICKS` → [`PossessPoll::TimedOut`]
    ///   并出表;
    /// - 否则留表(下一拍再看)。
    pub fn poll_possess(
        &mut self,
        now: Tick,
        mut exists: impl FnMut(EntityId) -> bool,
    ) -> Vec<PossessPoll> {
        if self.pending.is_empty() {
            return Vec::new();
        }
        let entries: Vec<(InputSource, PendingPossess)> =
            self.pending.iter().map(|(&p, &e)| (p, e)).collect();
        let mut polls = Vec::new();
        for (player, entry) in entries {
            if exists(entry.entity) {
                self.pending.remove(&player);
                polls.push(PossessPoll::Ready(player, entry.entity));
            } else if now.0 >= entry.issued_at.0 + POSSESS_GRACE_TICKS {
                self.pending.remove(&player);
                polls.push(PossessPoll::TimedOut(player, entry.entity));
            }
        }
        polls
    }
}

/// [`safe_spawn_position`] 的逐项结果。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SafeSpot {
    /// 可用的根位置。
    Found(Vec3),
    /// 出生点自己那一列所需的格还读不到(chunk 未装载)。
    NotLoaded,
    /// 半径内没有站得住的列。
    NoGround,
}

/// 安全落点(§6"安全落点"修订;纯函数,地形经 `solid` 闭包读——`Some(true)`
/// 实心、`Some(false)` 非实心、`None` 读不到)。规则:
/// 1. `feet = floor(point.y − half_height)`,列 = `(floor(x), floor(z))`。
/// 2. **原位置**:占用的两格 `(feet)`、`(feet+1)` 都非实心,且 `feet−1`
///    往下到 `y_min` 有实心格 → `Found(point)` 原样返回。这一列任何用到的
///    格读不到 → `NotLoaded`。
/// 3. **换列**:方环由近到远(r = 1..=SAFE_SEARCH_RADIUS),环内先 z 后 x
///    升序,只取 `max(|dx|,|dz|) == r` 的。每列从 `feet+2` 往下扫到
///    `y_min`,找第一个"自己实心、上面两格非实心"的格 `g` →
///    `Found((cx+dx+0.5, g+1+SAFE_ROOT_ABOVE_GROUND, cz+dz+0.5))`。列里有
///    格读不到 → 跳过这一列。
/// 4. 都没有 → `NoGround`。
pub fn safe_spawn_position(
    point: Vec3,
    half_height: f32,
    y_min: i32,
    solid: impl Fn(BlockPos) -> Option<bool>,
) -> SafeSpot {
    let feet = (point.y - half_height).floor() as i32;
    let cx = point.x.floor() as i32;
    let cz = point.z.floor() as i32;
    // 原位置:占用的两格必须可读(读不到 = 出生点列不可判 → 等装载重试);
    // 实心与否不在这里挡——两格之一实心同样落入换列(S4)。
    let feet_a = solid(BlockPos::new(cx, feet, cz));
    let feet_b = solid(BlockPos::new(cx, feet + 1, cz));
    if feet_a.is_none() || feet_b.is_none() {
        return SafeSpot::NotLoaded;
    }
    let mut ground = false;
    let mut y = feet - 1;
    while y >= y_min {
        match solid(BlockPos::new(cx, y, cz)) {
            Some(true) => {
                ground = true;
                break;
            }
            Some(false) => y -= 1,
            None => return SafeSpot::NotLoaded,
        }
    }
    if feet_a == Some(false) && feet_b == Some(false) && ground {
        return SafeSpot::Found(point);
    }
    // 换列:方环由近到远,环内先 z 后 x 升序(切比雪夫距离 == r)。
    for r in 1..=SAFE_SEARCH_RADIUS {
        for dz in -r..=r {
            for dx in -r..=r {
                if dx.abs().max(dz.abs()) != r {
                    continue;
                }
                let (nx, nz) = (cx + dx, cz + dz);
                // 列扫描:从 feet+2 往下找第一个"自己实心、上面两格非实心"
                // 的格;中途任何一格读不到 → 整列跳过(候选列必须全列可判,
                // 防止落在未装载列的边上)。
                let mut candidate = None;
                let mut y = feet + 2;
                let mut readable = true;
                while y >= y_min {
                    let a = solid(BlockPos::new(nx, y, nz));
                    let b = solid(BlockPos::new(nx, y + 1, nz));
                    let c = solid(BlockPos::new(nx, y + 2, nz));
                    match (a, b, c) {
                        (Some(true), Some(false), Some(false)) => {
                            candidate = Some(y);
                            break;
                        }
                        // 可读但不是落点(空气续扫 / 实心上面还有实心)→
                        // 继续往下;任一格 None → 整列不可判,跳过。
                        (Some(_), Some(_), Some(_)) => y -= 1,
                        _ => {
                            readable = false;
                            break;
                        }
                    }
                }
                if !readable {
                    continue;
                }
                let Some(g) = candidate else {
                    continue;
                };
                return SafeSpot::Found(Vec3::new(
                    nx as f32 + 0.5,
                    g as f32 + 1.0 + SAFE_ROOT_ABOVE_GROUND,
                    nz as f32 + 0.5,
                ));
            }
        }
    }
    SafeSpot::NoGround
}

#[cfg(test)]
mod tests {
    use super::*;

    fn player(n: u32) -> InputSource {
        InputSource(n)
    }

    fn entity(seq: u32) -> EntityId {
        EntityId::from_parts(0, seq)
    }

    fn point(x: f32) -> SpawnPoint {
        SpawnPoint {
            position: Vec3::new(x, -58.0, 8.5),
            yaw: 0.0,
            body: "capsule".to_owned(),
        }
    }

    /// T1·期限按条目算:A 在第 10 拍入表、B 在第 12 拍入表,第 13 拍
    /// A 超时(A = 10 + 3)、B 不超时(12 + 3 = 15 > 13)。
    /// 变红实现:任何跨条目共用的计数器(上一版 `respawn_possess_wait`)。
    #[test]
    fn t1_deadline_is_per_entry() {
        let mut book = RespawnBook::default();
        book.begin_possess_wait(player(1), entity(1), Tick(10));
        book.begin_possess_wait(player(2), entity(2), Tick(12));
        // 第 11、12 拍:谁都没超期,也没有 Ready。
        for now in [Tick(11), Tick(12)] {
            assert!(
                book.poll_possess(now, |_| false).is_empty(),
                "拍 {now:?} 不应有任何判定"
            );
        }
        // 第 13 拍:A 超时(issued 10 + 宽限 3),B 还在等(15 才到期)。
        let polls = book.poll_possess(Tick(13), |_| false);
        assert_eq!(polls, vec![PossessPoll::TimedOut(player(1), entity(1))]);
        // B 仍留表,第 15 拍才超时。
        assert!(book.poll_possess(Tick(14), |_| false).is_empty());
        let polls = book.poll_possess(Tick(15), |_| false);
        assert_eq!(polls, vec![PossessPoll::TimedOut(player(2), entity(2))]);
    }

    /// T1·Ready 出表:实体出现即 Ready,且下一拍不再重复出。
    #[test]
    fn t1_ready_removes_entry() {
        let mut book = RespawnBook::default();
        book.begin_possess_wait(player(1), entity(1), Tick(10));
        let polls = book.poll_possess(Tick(11), |e| e == entity(1));
        assert_eq!(polls, vec![PossessPoll::Ready(player(1), entity(1))]);
        assert!(
            book.poll_possess(Tick(12), |_| true).is_empty(),
            "Ready 后条目应已出表"
        );
    }

    /// T1·出生点取最近一次:同连接两次 `on_spawn_accepted`,出生点是后者。
    /// 变红实现:插入改 `or_insert`(只记第一次)。
    #[test]
    fn t1_spawn_point_overwrites_latest() {
        let mut book = RespawnBook::default();
        book.on_spawn_accepted(player(1), entity(1), point(8.0));
        book.on_spawn_accepted(player(1), entity(2), point(40.0));
        // 第一个身体被虚空移除(正常流程:重生前旧身体出表)。
        let got = book.on_void_removed(entity(2)).expect("玩家身体应命中");
        assert_eq!(got.0, player(1));
        assert_eq!(got.1, point(40.0), "出生点 = 最近一次被接受的 SpawnAgent");
    }

    /// T1·断线清三张表,之后虚空移除返回 None(不重生)。
    /// 变红实现:断线路由调重生(A06-5"断线移除也重生")。
    #[test]
    fn t1_disconnect_clears_all_tables() {
        let mut book = RespawnBook::default();
        book.on_spawn_accepted(player(1), entity(1), point(8.0));
        book.begin_possess_wait(player(1), entity(9), Tick(10));
        book.on_disconnect(player(1));
        assert_eq!(book.on_void_removed(entity(1)), None, "身体归属已清");
        assert!(
            book.poll_possess(Tick(20), |_| true).is_empty(),
            "待附身已清"
        );
    }

    // -------------------------------------------------------------------
    // 等区块装载表(R3 §6 修订):登记/出表/查询/断线清/取消
    // -------------------------------------------------------------------

    /// 等待表基本流转:登记可见、出表返回 true、二次出表 false。
    #[test]
    fn s8_respawn_wait_table_roundtrip() {
        let mut book = RespawnBook::default();
        assert!(!book.is_respawn_pending(player(1)));
        book.begin_respawn_wait(player(1), point(8.0), Tick(10));
        assert!(book.is_respawn_pending(player(1)));
        assert_eq!(
            book.pending_respawns(),
            vec![(player(1), point(8.0), Tick(10))],
            "等待条目按连接升序、带出生点与发起拍"
        );
        assert!(book.end_respawn_wait(player(1)), "出表返回 true");
        assert!(!book.end_respawn_wait(player(1)), "二次出表 false");
        assert!(!book.is_respawn_pending(player(1)));
    }

    /// 断线同时清重生等装载表(等待中的连接断开,不再重生)。
    #[test]
    fn s8_disconnect_clears_pending_respawn() {
        let mut book = RespawnBook::default();
        book.begin_respawn_wait(player(1), point(8.0), Tick(10));
        book.on_disconnect(player(1));
        assert!(
            !book.is_respawn_pending(player(1)),
            "断线清等待表,之后不再重生"
        );
    }

    // -------------------------------------------------------------------
    // safe_spawn_position(S1–S7;§6"安全落点"修订)
    // -------------------------------------------------------------------

    /// 平地布景:y ≤ base 实心(地表在 base)、y > base 空气;y < 世界底
    /// 读不到(闭包只在测试需要的窗口上给出地形)。
    fn flat(base: i32) -> impl Fn(BlockPos) -> Option<bool> {
        move |p: BlockPos| {
            if p.y < types::WORLD_Y_MIN {
                None
            } else {
                Some(p.y <= base)
            }
        }
    }

    /// 平地布景(y ≤ −59 实心);出生点列 (24, ·, 12) 全空气。
    fn dug_ground(dug: impl Fn(BlockPos) -> bool) -> impl Fn(BlockPos) -> Option<bool> {
        move |p: BlockPos| {
            if p.y < types::WORLD_Y_MIN {
                return None;
            }
            if dug(p) {
                Some(false)
            } else {
                Some(p.y <= -59)
            }
        }
    }

    /// S1:原位置可用 → 原样返回(逐位相等)。
    #[test]
    fn s1_original_position_returned_as_is() {
        let p = Vec3::new(8.5, -58.0 + 0.9, 8.5);
        // feet = floor(-58.0 + 0.9 - 0.9) = -58;地面在 -59(feet-1)。
        let got = safe_spawn_position(p, 0.9, types::WORLD_Y_MIN, flat(-59));
        assert_eq!(got, SafeSpot::Found(p), "原位置可用 → 原样返回");
    }

    /// S2:原列挖穿 → 第 1 环第一列 (cx−1, cz−1),y = 地面顶面 + 1.0。
    /// 变红实现:恒返回 Found(point)(变异 3)或环内先 x 后 z(变异 8)。
    #[test]
    fn s2_dug_column_falls_to_first_ring_first_column() {
        let ground = dug_ground(|p| p.x == 24 && p.z == 12);
        let p = Vec3::new(24.5, -58.0 + 0.9, 12.5);
        let got = safe_spawn_position(p, 0.9, types::WORLD_Y_MIN, ground);
        assert_eq!(
            got,
            SafeSpot::Found(Vec3::new(23.5, -59.0 + 1.0 + SAFE_ROOT_ABOVE_GROUND, 11.5)),
            "第 1 环第一列 = (cx−1, cz−1)(先 z 后 x 升序),y = g+1+1.0 = -57.0"
        );
    }

    /// S3:第 1 环全挖穿 → 落在第 2 环第一列(cx−2, cz−2)。
    #[test]
    fn s3_first_ring_dug_falls_to_second_ring() {
        let ground = dug_ground(|p| (p.x - 24).abs().max((p.z - 12).abs()) <= 1);
        let p = Vec3::new(24.5, -58.0 + 0.9, 12.5);
        let got = safe_spawn_position(p, 0.9, types::WORLD_Y_MIN, ground);
        assert_eq!(
            got,
            SafeSpot::Found(Vec3::new(22.5, -57.0, 10.5)),
            "第 2 环第一列 = (cx−2, cz−2),y = g+1+1.0 = -57.0"
        );
    }

    /// S4:原位置两格之一实心 → 换列(不 Found 原点)。
    #[test]
    fn s4_solid_at_body_cell_moves_to_neighbor_column() {
        // 出生点列 feet 格实心(卡在方块里),邻列平地完好。
        let ground = |p: BlockPos| {
            if p.y < types::WORLD_Y_MIN {
                return None;
            }
            if p.x == 24 && p.z == 12 && p.y == -58 {
                Some(true)
            } else {
                Some(p.y <= -59)
            }
        };
        let p = Vec3::new(24.5, -58.0 + 0.9, 12.5);
        let got = safe_spawn_position(p, 0.9, types::WORLD_Y_MIN, ground);
        assert_ne!(got, SafeSpot::Found(p), "占用格实心 → 不落原位置");
        assert!(
            matches!(got, SafeSpot::Found(v) if v.x != p.x || v.z != p.z),
            "换到邻列,实得 {got:?}"
        );
    }

    /// S5:原列读不到 → NotLoaded(chunk 未装载的等价情形)。
    #[test]
    fn s5_unreadable_original_column_not_loaded() {
        let ground = |p: BlockPos| {
            if p.x == 24 && p.z == 12 {
                None
            } else {
                Some(p.y <= -59)
            }
        };
        let p = Vec3::new(24.5, -58.0 + 0.9, 12.5);
        assert_eq!(
            safe_spawn_position(p, 0.9, types::WORLD_Y_MIN, ground),
            SafeSpot::NotLoaded,
            "原列读不到 → NotLoaded"
        );
    }

    /// S6:半径内全空(无任何实心列)→ NoGround。
    #[test]
    fn s6_no_ground_within_radius() {
        let p = Vec3::new(24.5, -58.0 + 0.9, 12.5);
        assert_eq!(
            safe_spawn_position(p, 0.9, types::WORLD_Y_MIN, |_| Some(false)),
            SafeSpot::NoGround,
            "半径内全空 → NoGround"
        );
    }

    /// S7:候选列读不到 → 跳过,取下一个可读列。第 1 环第一列
    /// (23,11) 不可读;先 z 后 x 的环序下下一个候选是 (24,11)。
    /// (先 x 后 z 的错误环序会取 (23,12)——变异 8 的红点。)
    #[test]
    fn s7_unreadable_candidate_column_skipped() {
        let ground = |p: BlockPos| {
            if p.x == 23 && p.z == 11 {
                // 候选列整列读不到(chunk 未装载)。
                return None;
            }
            if p.x == 24 && p.z == 12 {
                // 原列挖穿。
                Some(false)
            } else {
                Some(p.y <= -59)
            }
        };
        let p = Vec3::new(24.5, -58.0 + 0.9, 12.5);
        let got = safe_spawn_position(p, 0.9, types::WORLD_Y_MIN, ground);
        assert_eq!(
            got,
            SafeSpot::Found(Vec3::new(24.5, -57.0, 11.5)),
            "第 1 环第一列 (23,11) 读不到 → 跳过,环序先 z 后 x 取 (24,11)"
        );
    }
}
