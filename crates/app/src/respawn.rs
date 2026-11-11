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
use types::{EntityId, Tick};

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

/// 重生簿记:连接 → 出生点、玩家身体 → 连接、连接 → 待附身。
#[derive(Debug, Default)]
pub struct RespawnBook {
    /// 连接 → 最近一次被接受的出生点(`on_spawn_accepted` 覆盖写)。
    spawn_points: BTreeMap<InputSource, SpawnPoint>,
    /// 玩家身体 → 连接(`on_void_removed` 反查归属用)。
    bodies: BTreeMap<EntityId, InputSource>,
    /// 连接 → 重生后等待附身的条目。
    pending: BTreeMap<InputSource, PendingPossess>,
}

impl RespawnBook {
    /// 生成被接受(`handle_spawn_agent` 成功处调用):出生点覆盖写、身体
    /// 归属登记。
    pub fn on_spawn_accepted(&mut self, player: InputSource, entity: EntityId, point: SpawnPoint) {
        self.spawn_points.insert(player, point);
        self.bodies.insert(entity, player);
    }

    /// 连接断开:三张表里该连接的条目一起清(出生点失效、身体归属不再
    /// 有效——断线移除会把它移除;待附身等待也没有意义)。
    pub fn on_disconnect(&mut self, player: InputSource) {
        self.spawn_points.remove(&player);
        self.bodies.retain(|_, owner| *owner != player);
        self.pending.remove(&player);
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
}
