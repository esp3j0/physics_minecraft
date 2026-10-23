//! REQ-007 验收用例(总架构师编写;issue #14)。
//!
//! 约定:
//! - 本文件由验收方编写,实施方**不得修改断言**。实现完成后,实施方只做一件事:
//!   去掉对应单元用例上的 `#[ignore]`。若认为某条断言与需求不符,在 #14 提出,
//!   由总架构师裁定后修改;
//! - 全部用例走生产路径(`Session` + 复刻驱动 `orchestrate` 的每拍编排),
//!   不直接调用分片内部函数(纪律 11);
//! - 每个用例注释写明"在什么错误实现下会变红"(纪律 12)。
//!
//! 场景地形:生产默认 Superflat,地表顶面 y = -60;半尺寸 0.5 的实体贴地时
//! 中心 y = -59.5。

use std::collections::BTreeSet;

use app::config::Config;
use messaging::{InputSource, SimCommand};
use physics_sim::tickets::{TicketHolder, TicketKind};
use sim_api::session::Session;
use storage::traits::ChunkKey;
use types::glam::Vec3;
use types::{DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, WorldId};

/// 地表上贴地实体的中心高度。
const GROUND_Y: f32 = -59.5;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn origin() -> ShardId {
    shard_id(0, 0)
}

/// 生产装配 + 每拍编排的测试夹具。
struct Harness {
    session: Session,
    store: storage::worker::StoreWorker,
    provider: storage::provider::ChunkProvider,
    new_sim: Box<dyn FnMut(ShardId) -> physics_sim::ShardSim + Send>,
}

impl Harness {
    /// `tweak_origin`:对分片 (0,0) 的 `ShardSimConfig` 做场景定制(例如零重力、
    /// 醒体预算);其余分片按生产配置由 ensure 懒创建。
    fn new(tweak_origin: impl FnOnce(&mut physics_sim::ShardSimConfig)) -> Self {
        let config = Config::default();
        let store = storage::worker::StoreWorker::new(
            Box::new(storage::mem::MemChunkStore::new()),
            Box::new(storage::mem::MemCheckpointStore::new()),
        );
        let mut session = Session::new(config.session_config());
        let (provider, new_sim) =
            app::driver::wire_provider_and_new_sim(config.clone(), &store, || {
                Box::new(voxel::SimpleRegistry::builtin())
            });
        let mut cfg = config.shard_sim_config(origin());
        tweak_origin(&mut cfg);
        session.register(physics_sim::ShardSim::new(
            cfg,
            Box::new(voxel::SimpleRegistry::builtin()),
        ));
        Self {
            session,
            store,
            provider,
            new_sim,
        }
    }

    /// 复刻 `driver::orchestrate` 的每拍末编排:票据 ensure + 在途目的分片 ensure
    /// → 激活装卸(含 BlockWrite 待写格延迟卸载)→ despawn。
    fn orchestrate(&mut self) {
        let mut ensure: BTreeSet<ShardId> = self.session.ticket_ensure_shards();
        ensure.extend(self.session.pending_shard_destinations());
        for shard in ensure {
            if !self.session.shard_ids().contains(&shard) {
                let sim = (self.new_sim)(shard);
                self.session.register(sim);
            }
        }
        let deferred = self.session.pending_blockwrite_cells();
        for (shard, decisions) in self.session.take_activation_requests() {
            for pos in decisions.chunks_to_load {
                let key = ChunkKey {
                    world: shard.world,
                    dim: shard.dimension,
                    pos,
                };
                let chunk = self
                    .provider
                    .get(key)
                    .unwrap_or_else(|| panic!("provider.get 返回 None:{key:?}"));
                self.session.apply_chunk_load(shard, pos, chunk);
            }
            for pos in decisions.chunks_to_unload {
                if deferred.get(&shard).is_some_and(|c| c.contains(&pos)) {
                    continue;
                }
                let _ = self.session.apply_chunk_unload(shard, pos);
            }
        }
        let pending = self.session.pending_shard_destinations();
        for shard in self.session.take_despawn_requests() {
            if !self.session.shard_ids().contains(&shard) || pending.contains(&shard) {
                continue;
            }
            let _ = self.session.unload_shard(shard);
        }
    }

    /// 推进一拍并编排。
    fn step(&mut self) {
        self.session.tick();
        self.orchestrate();
    }

    fn steps(&mut self, n: usize) {
        for _ in 0..n {
            self.step();
        }
    }

    /// 在分片 (0,0) 生成一个常驻体(同拍 MarkActor)。
    fn spawn_actor(&mut self, e: EntityId, pos: Vec3, vel: Vec3) {
        self.session
            .send(
                origin(),
                SimCommand::SpawnDynamic {
                    entity: e,
                    state: EntityState {
                        translation: pos,
                        linvel: vel,
                        ..EntityState::default()
                    },
                    desc: EntityDesc::default(),
                },
            )
            .expect("spawn");
        self.session
            .send(
                origin(),
                SimCommand::MarkActor {
                    entity: e,
                    actor: true,
                },
            )
            .expect("mark actor");
    }

    fn set_observer(&mut self, pos: Vec3) {
        self.session
            .move_observer_from(InputSource(1), pos)
            .expect("设观察者");
    }

    fn remove_observer(&mut self) {
        self.session
            .remove_observer_from(InputSource(1))
            .expect("移除观察者");
    }

    /// 只装载、不唤醒的区域票(档位 ①②,`WakeSpec::None`):提供地形并启用激活,
    /// 但不构成唤醒范围。
    fn terrain_only_area(&mut self, center: Vec3, radius_m: f32) {
        self.session
            .set_area_ticket(1, center, radius_m, 0b011, None)
            .expect("区域票");
    }

    fn has_actor_ticket(&self, e: EntityId) -> bool {
        self.session
            .ticket_table()
            .get((TicketHolder::Entity(e), TicketKind::ActorMotion))
            .is_some()
    }

    fn frozen(&self, e: EntityId) -> bool {
        self.session.is_frozen(origin(), e)
    }

    fn pos(&self, e: EntityId) -> Vec3 {
        self.session
            .entity_state(origin(), e)
            .unwrap_or_else(|| panic!("实体 {e:?} 应在分片 (0,0)"))
            .translation
    }

    fn finish(self) {
        self.session.shutdown();
        let _ = self.store.shutdown();
    }
}

fn actor(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

// ===========================================================================
// 04a:常驻运动票、静止判定与重置、超预算冻结撤票
// (AC-T11、AC-T8 常驻半边;需求 §4.3.4、§4.4、§4.5.1、§4.12)
// ===========================================================================

/// AC-T11 第一项:常驻体运动中离开所有唤醒范围,不冻结;持常驻运动票直到落地静止,
/// 静止后才冻结并撤票,冻结时贴地。
///
/// 会变红的实现:沿用旧规则"离观察者远就冻结"(在空中被冻住);运动中不签发
/// `ActorMotion`;静止后不撤票;冻结时不要求静止。
#[test]
fn acc_04a_moving_actor_not_frozen_midair() {
    let mut h = Harness::new(|_| {});
    h.set_observer(Vec3::new(64.0, GROUND_Y, 64.0));
    h.steps(60);

    // 离地 10 m、水平 4 m/s 抛出;随即移除唯一的观察者(此后没有任何唤醒范围)。
    let e = actor(1);
    h.spawn_actor(
        e,
        Vec3::new(64.0, GROUND_Y + 10.0, 64.0),
        Vec3::new(4.0, 0.0, 0.0),
    );
    h.step();
    h.remove_observer();

    let mut saw_ticket = false;
    let mut frozen_at: Option<(usize, f32)> = None;
    for t in 0..900 {
        h.step();
        let y = h.pos(e).y;
        let frozen = h.frozen(e);
        if y > GROUND_Y + 0.3 {
            assert!(
                !frozen,
                "拍 {t}:常驻体在空中(y = {y})被冻结,违反'冻结只在静止时'"
            );
        }
        saw_ticket |= h.has_actor_ticket(e);
        if frozen {
            frozen_at = Some((t, y));
            break;
        }
    }
    assert!(saw_ticket, "运动中离开唤醒范围的常驻体应持有常驻运动票");
    let (t_freeze, y_freeze) = frozen_at.expect("落地静止后应冻结(900 拍内)");
    assert!(
        (y_freeze - GROUND_Y).abs() < 0.15,
        "冻结时应贴地:y = {y_freeze},期望约 {GROUND_Y}(拍 {t_freeze})"
    );
    h.steps(3);
    assert!(h.frozen(e), "冻结后保持冻结");
    assert!(
        !h.has_actor_ticket(e),
        "静止冻结后常驻运动票应已撤销(I3 第 4 条)"
    );
    h.finish();
}

/// AC-T11 第二项:常驻运动票到达上限(默认 `actor_motion_max_ticks` = 1200)后撤票,
/// 实体随即冻结(兜底降级,即使仍在运动),且不会立刻被重新签发。
///
/// 场景:零重力分片中匀速漂移、永不静止;只有"仅装载"的区域票,没有唤醒范围。
///
/// 会变红的实现:漂移体被立即冻结;票没有上限;到上限撤票后又在下一拍重新签发
/// (上限形同虚设);撤票后不冻结。
#[test]
fn acc_04a_actor_motion_ticket_capped() {
    let mut h = Harness::new(|cfg| cfg.gravity = Vec3::ZERO);
    let start = Vec3::new(64.0, GROUND_Y + 20.0, 64.0);
    h.terrain_only_area(start, 32.0);
    h.steps(40);

    let e = actor(1);
    h.spawn_actor(e, start, Vec3::new(0.4, 0.0, 0.0));
    h.step();

    let mut first_ticket: Option<usize> = None;
    let mut ended_at: Option<usize> = None;
    for t in 0..1400 {
        h.step();
        let has = h.has_actor_ticket(e);
        if first_ticket.is_none() && has {
            first_ticket = Some(t);
        }
        if first_ticket.is_some() && ended_at.is_none() {
            if !has {
                ended_at = Some(t);
                continue;
            }
            assert!(!h.frozen(e), "拍 {t}:持票漂移中的常驻体不应被冻结");
        }
    }
    let t_first = first_ticket.expect("漂移中的常驻体应签发常驻运动票");
    assert!(
        t_first <= 5,
        "常驻运动票应在生成后数拍内签发(实得拍 {t_first})"
    );
    let t_end = ended_at.expect("常驻运动票应在上限处撤销");
    assert!(
        (1150..=1260).contains(&t_end),
        "撤票拍 {t_end} 应在上限 1200 拍附近"
    );
    // 撤票后:冻结,且整个剩余观察期内不再被重新签发。
    assert!(h.frozen(e), "到达上限撤票后应冻结(兜底降级)");
    assert!(
        !h.has_actor_ticket(e),
        "到达上限撤票后不应被重新签发(否则上限无效)"
    );
    h.finish();
}

/// AC-T8 常驻半边:超预算淘汰冻结的常驻体,同时撤销其常驻运动票(D14 补漏)。
///
/// 场景:零重力分片、醒体预算 2;e1 先在远处漂移并持票,随后在区域票中心附近
/// 再生成 e2、e3,醒数超限;按到唤醒中心距离降序淘汰,最远的 e1 被冻结。
///
/// 会变红的实现:被淘汰冻结的常驻体仍持有 `ActorMotion`;淘汰了错误的实体;
/// e1 在超限前就被冻结(运动中不该冻结)。
#[test]
fn acc_04a_over_budget_actor_frozen_and_ticket_ended() {
    let mut h = Harness::new(|cfg| {
        cfg.gravity = Vec3::ZERO;
        cfg.activation.max_awake_dynamic = 2;
    });
    let center = Vec3::new(64.0, GROUND_Y + 20.0, 64.0);
    h.terrain_only_area(center, 32.0);
    h.steps(40);

    let (e1, e2, e3) = (actor(1), actor(2), actor(3));
    // e1 距中心 10 m,向外慢速漂移。
    h.spawn_actor(
        e1,
        center + Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(0.2, 0.0, 0.0),
    );
    h.steps(10);
    assert!(h.has_actor_ticket(e1), "前置:e1 漂移中应持常驻运动票");
    assert!(!h.frozen(e1), "前置:e1 运动中不应被冻结");

    // e2、e3 距中心 3 m,醒数 3 > 预算 2。
    h.spawn_actor(
        e2,
        center + Vec3::new(0.0, 0.0, -3.0),
        Vec3::new(0.0, 0.0, -0.1),
    );
    h.spawn_actor(
        e3,
        center + Vec3::new(0.0, 0.0, 3.0),
        Vec3::new(0.0, 0.0, 0.1),
    );
    h.steps(3);

    assert!(h.frozen(e1), "最远的 e1 应被超预算淘汰冻结");
    assert!(
        !h.has_actor_ticket(e1),
        "超预算冻结的常驻体必须同时撤销常驻运动票(§4.12、D14)"
    );
    assert!(!h.frozen(e2) && !h.frozen(e3), "较近的 e2、e3 保持清醒");
    assert!(
        h.has_actor_ticket(e2) && h.has_actor_ticket(e3),
        "e2、e3 在唤醒范围外运动,应持常驻运动票"
    );
    h.finish();
}

/// AC-T11 第三项:任何唤醒都重置静止判据——醒来后至少 `consecutive_ticks`
/// (20)拍内不被判定为静止。
///
/// 观测方式:常驻体静止冻结 → 冷却过后观察者回到 24 m 内将其唤醒 → 随即移除
/// 观察者。若静止判据已重置,醒来后它被判定"运动中",会短暂签发常驻运动票,
/// 约 20 拍后判定静止再撤销;若沿用冻结前的旧窗口,它一醒来就被判定静止,
/// 永远不会签发。
///
/// 会变红的实现:唤醒时不重置 `slow_ticks`/位移窗口;唤醒后从不签发常驻票;
/// 静止后不撤票。
#[test]
fn acc_04a_wake_resets_stillness() {
    let mut h = Harness::new(|_| {});
    // chunk (4,4) 中心 (72,72);实体放在 chunk 中心,足迹不跨 chunk。
    let spot = Vec3::new(72.0, GROUND_Y, 72.0);
    h.set_observer(Vec3::new(72.0, GROUND_Y, 60.0));
    h.steps(60);
    let e = actor(1);
    h.spawn_actor(e, spot, Vec3::ZERO);
    h.steps(80);
    assert!(!h.frozen(e), "前置:观察者 12 m 内,实体醒着");

    // 观察者移到 40 m 外(> 32 冻结半径,< 48 chunk 保留半径):静止的常驻体冻结。
    h.set_observer(Vec3::new(112.0, GROUND_Y, 72.0));
    let mut froze = false;
    for _ in 0..30 {
        h.step();
        assert!(!h.has_actor_ticket(e), "静止的常驻体离开范围不应签发常驻票");
        if h.frozen(e) {
            froze = true;
            break;
        }
    }
    assert!(froze, "前置:静止常驻体离开唤醒范围后应冻结");

    // 等翻转冷却(180 拍)过去,观察者回到 12 m 内唤醒它。
    h.steps(200);
    h.set_observer(Vec3::new(72.0, GROUND_Y, 60.0));
    let mut woke = false;
    for _ in 0..10 {
        h.step();
        if !h.frozen(e) {
            woke = true;
            break;
        }
    }
    assert!(woke, "前置:冷却过后观察者回到 24 m 内应唤醒");

    // 醒来后立刻移除观察者。
    h.remove_observer();
    let mut saw_ticket = false;
    for _ in 0..6 {
        h.step();
        saw_ticket |= h.has_actor_ticket(e);
    }
    assert!(
        saw_ticket,
        "唤醒后静止判据应重置:醒来后 20 拍内被判定为运动中,应短暂持有常驻运动票"
    );
    let mut ended = false;
    for _ in 0..60 {
        h.step();
        if !h.has_actor_ticket(e) {
            ended = true;
            break;
        }
    }
    assert!(ended, "静止判据重新满足后(约 20 拍)应撤销常驻运动票");
    h.finish();
}
