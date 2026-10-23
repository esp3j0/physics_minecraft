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
use physics_sim::tickets::{TicketHolder, TicketKind, TicketState};
use sim_api::session::Session;
use storage::traits::ChunkKey;
use types::glam::Vec3;
use types::{
    BlockId, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, WorldId,
};

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

    fn spawn_transient(&mut self, e: EntityId, pos: Vec3) {
        self.session
            .send(
                origin(),
                SimCommand::SpawnDynamic {
                    entity: e,
                    state: EntityState {
                        translation: pos,
                        ..EntityState::default()
                    },
                    desc: EntityDesc::default(),
                },
            )
            .expect("spawn transient");
    }

    fn teleport(&mut self, e: EntityId, pos: Vec3) {
        self.session
            .send(
                origin(),
                SimCommand::TeleportEntity {
                    entity: e,
                    state: EntityState {
                        translation: pos,
                        ..EntityState::default()
                    },
                },
            )
            .expect("teleport");
    }

    fn transient_ticket_state(&self, e: EntityId) -> Option<TicketState> {
        self.session
            .ticket_table()
            .get((TicketHolder::Entity(e), TicketKind::TransientMotion))
            .map(|t| t.state)
    }

    fn exists(&self, e: EntityId) -> bool {
        self.session.entity_state(origin(), e).is_some()
    }

    fn has_collider(&self, cp: ChunkPos) -> bool {
        self.session.has_collider(origin(), cp)
    }

    /// 分片 (0,0) 中某格的方块(chunk 未装载时返回 None)。
    fn block_at(&self, x: i32, y: i32, z: i32) -> Option<BlockId> {
        let cp = ChunkPos::new(x.div_euclid(16), z.div_euclid(16));
        let chunk = self.session.voxel_chunk(origin(), cp)?;
        Some(chunk.get(
            x.rem_euclid(16) as usize,
            (y - types::WORLD_Y_MIN) as usize,
            z.rem_euclid(16) as usize,
        ))
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
/// 场景:零重力分片、醒体预算 2;e1 先漂移并持票,随后再生成 e2、e3,醒数超限。
/// 区域票只装载、不构成唤醒范围,三者到唤醒中心的距离都是 +∞,按 §4.12 的
/// 平局规则(`EntityId` 升序)淘汰 e1。本用例锚定的是"淘汰冻结必须撤票",
/// 不是距离排序;距离排序由实施方单测覆盖。
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

// ===========================================================================
// 04b:统一唤醒入口、推迟唤醒(当前足迹 ② 前置 + 阻塞请求)、向上支撑闭包
// (AC-T10、AC-T12、AC-T9 中由 03 移来的"支撑闭包同组唤醒"段;§4.5、§4.6、§4.9)
// ===========================================================================

/// 场景位置:chunk (4,4) 的中心,实体足迹不跨 chunk。
const SPOT: Vec3 = Vec3::new(72.0, GROUND_Y, 72.0);
const SPOT_CHUNK: ChunkPos = ChunkPos::new(4, 4);

/// 观察者位置:A 正下方 23.5 m。A 距 23.5 m(≤ 24,满足唤醒),B 距 24.5 m(> 24)。
const BELOW_A: Vec3 = Vec3::new(72.0, GROUND_Y - 23.5, 72.0);

/// AC-T10:复现 §3.3 第 3 条的折返竞态。冻结的常驻体在脚下 chunk 被卸载后,
/// 观察者于 ① 冷却期内折返。要求:② 就绪前保持**冻结**(推迟唤醒,不是唤醒后
/// 进入待地形);缺失的 chunk 作为阻塞请求**免冷却**重新装载;② 就绪后醒来;
/// 全程不穿地。
///
/// 会变红的实现:唤醒不检查当前足迹 ②(醒来后掉进待地形或坠落);推迟唤醒
/// 不发阻塞请求(chunk 要等约 160 拍 ① 冷却才重装);② 就绪后不唤醒。
#[test]
fn acc_04b_roundtrip_wake_deferred_until_terrain() {
    let mut h = Harness::new(|_| {});
    let near = Vec3::new(72.0, GROUND_Y, 60.0); // 距实体 12 m
    let mid = Vec3::new(112.0, GROUND_Y, 72.0); // 40 m:冻结,但 chunk 仍保留
    let far = Vec3::new(232.0, GROUND_Y, 72.0); // 160 m:chunk 卸载
    h.set_observer(near);
    h.steps(60);
    let e = actor(1);
    h.spawn_actor(e, SPOT, Vec3::ZERO);
    h.steps(80);

    // 移到 40 m:静止的常驻体冻结,chunk 保留。
    h.set_observer(mid);
    let mut froze = false;
    for _ in 0..30 {
        h.step();
        if h.frozen(e) {
            froze = true;
            break;
        }
    }
    assert!(froze, "前置:静止常驻体在 40 m 外应冻结");
    let mut since_freeze = 0usize;
    h.steps(170);
    since_freeze += 170;

    // 移到 160 m:等实体脚下的 chunk 被卸载。
    h.set_observer(far);
    let mut unloaded = false;
    for _ in 0..120 {
        h.step();
        since_freeze += 1;
        if !h.session.is_chunk_loaded(origin(), SPOT_CHUNK) {
            unloaded = true;
            break;
        }
    }
    assert!(unloaded, "前置:观察者 160 m 外,实体脚下的 chunk 应被卸载");
    assert!(h.frozen(e), "前置:实体保持冻结");
    // 保证翻转冷却(180 拍)已过,折返时满足唤醒条件;此时 ① 冷却仍未过。
    while since_freeze < 185 {
        h.step();
        since_freeze += 1;
    }

    // 折返到 12 m。
    h.set_observer(near);
    let mut collider_back_at = None;
    let mut woke_at = None;
    for k in 0..90 {
        h.step();
        let y = h.pos(e).y;
        assert!(y > GROUND_Y - 0.1, "拍 {k}:实体穿地(y = {y})");
        let has_ground = h.has_collider(SPOT_CHUNK);
        if has_ground && collider_back_at.is_none() {
            collider_back_at = Some(k);
        }
        if !has_ground {
            assert!(
                h.frozen(e),
                "拍 {k}:脚下 ② 未就绪时应保持冻结(推迟唤醒),而不是醒来"
            );
        }
        if !h.frozen(e) {
            woke_at = Some(k);
            break;
        }
    }
    let back = collider_back_at.expect("脚下 chunk 的 ② 应重新就绪");
    assert!(
        back < 40,
        "推迟唤醒的缺失 chunk 是阻塞请求,应免冷却装载;实得 {back} 拍(① 冷却会让它拖到约 160 拍)"
    );
    let woke = woke_at.expect("② 就绪后应唤醒");
    assert!(
        woke >= back,
        "唤醒不得早于 ② 就绪(唤醒拍 {woke} < ② 就绪拍 {back})"
    );
    h.finish();
}

/// 竖叠两个常驻体:A 在地面,B 压在 A 上。两者静止后在 40 m 外冻结,
/// 再等翻转冷却过去。
fn stacked_frozen_pair(h: &mut Harness) -> (EntityId, EntityId) {
    let (a, b) = (actor(1), actor(2));
    h.set_observer(Vec3::new(72.0, GROUND_Y, 60.0));
    h.steps(60);
    h.spawn_actor(a, SPOT, Vec3::ZERO);
    h.steps(40);
    h.spawn_actor(b, SPOT + Vec3::new(0.0, 1.05, 0.0), Vec3::ZERO);
    h.steps(80);
    assert!(
        (h.pos(b).y - (GROUND_Y + 1.0)).abs() < 0.1,
        "前置:B 叠在 A 上(B 的 y = {})",
        h.pos(b).y
    );
    h.set_observer(Vec3::new(112.0, GROUND_Y, 72.0));
    let mut both = false;
    for _ in 0..60 {
        h.step();
        if h.frozen(a) && h.frozen(b) {
            both = true;
            break;
        }
    }
    assert!(both, "前置:A、B 静止后都在 40 m 外冻结");
    h.steps(200); // 翻转冷却
    (a, b)
}

/// AC-T12 第一项:唤醒 A 时,压在 A 上的冻结体 B 经向上支撑闭包同拍一起醒来。
/// 观察者在 A 正下方,只有 A 落在唤醒半径内。
///
/// 会变红的实现:没有支撑闭包(B 在唤醒半径外保持冻结,A 一动 B 就悬空);
/// 闭包晚一拍或分开生效。
#[test]
fn acc_04b_closure_wakes_stack_together() {
    let mut h = Harness::new(|_| {});
    let (a, b) = stacked_frozen_pair(&mut h);
    h.set_observer(BELOW_A);
    let mut woke = false;
    for k in 0..15 {
        h.step();
        let (fa, fb) = (h.frozen(a), h.frozen(b));
        assert_eq!(
            fa, fb,
            "拍 {k}:A、B 必须同拍唤醒(A 冻结 = {fa},B 冻结 = {fb})"
        );
        if !fa {
            woke = true;
            break;
        }
    }
    assert!(woke, "A 进入唤醒半径后,A 与压在其上的 B 应一起醒来");
    h.finish();
}

/// AC-T12 第二项:醒体预算放不下整组时,整组推迟,无一单独醒来。
///
/// 会变红的实现:先唤醒 A 再按预算处理(A 单独醒来、B 留在空中);闭包成员被
/// 拆开淘汰。
#[test]
fn acc_04b_closure_deferred_when_budget_short() {
    let mut h = Harness::new(|cfg| cfg.activation.max_awake_dynamic = 1);
    let (a, b) = stacked_frozen_pair(&mut h);
    h.set_observer(BELOW_A);
    for k in 0..30 {
        h.step();
        assert!(
            h.frozen(a) && h.frozen(b),
            "拍 {k}:组大小 2 > 预算 1,整组应推迟(A 冻结 = {},B 冻结 = {})",
            h.frozen(a),
            h.frozen(b)
        );
    }
    h.finish();
}

/// AC-T9 中由 03 移到 04b 的一段:瞬态方块 T 压在冻结常驻体 A 上(T 被承载,
/// 静止后冻结、票据挂起)。唤醒 A 时 T 经闭包同拍醒来,票据从 Suspended 回到
/// Paused(仍被承载);随后把 A 传送走,T 失去支撑下落,落地后凝固成方块,
/// 实体消失、票据终结。
///
/// 会变红的实现:T 不随 A 唤醒(悬空);唤醒后票据回到 Active(仍被承载却恢复
/// 计时);T 失去支撑后不下落或不凝固;凝固后票据不终结。
#[test]
fn acc_04b_carried_block_wakes_with_supporter_then_lands() {
    let mut h = Harness::new(|_| {});
    let (a, t) = (actor(1), EntityId::from_parts(1, 9));
    h.set_observer(Vec3::new(72.0, GROUND_Y, 60.0));
    h.steps(60);
    h.spawn_actor(a, SPOT, Vec3::ZERO);
    h.steps(40);
    h.spawn_transient(t, SPOT + Vec3::new(0.0, 1.05, 0.0));
    h.steps(80);
    assert!(h.exists(t), "前置:压在常驻体上的瞬态方块不凝固(§4.7)");
    assert_eq!(
        h.transient_ticket_state(t),
        Some(TicketState::Paused),
        "前置:被承载的瞬态票据处于 Paused"
    );

    // 观察者移到 40 m:A 静止冻结,T 被承载且静止 → 冻结 + Suspended。
    h.set_observer(Vec3::new(112.0, GROUND_Y, 72.0));
    let mut both = false;
    for _ in 0..60 {
        h.step();
        if h.frozen(a) && h.frozen(t) {
            both = true;
            break;
        }
    }
    assert!(both, "前置:A 与其承载的 T 都冻结");
    h.steps(2); // 票据操作 N+1 生效
    assert_eq!(
        h.transient_ticket_state(t),
        Some(TicketState::Suspended),
        "前置:承载静止冻结的瞬态票据处于 Suspended"
    );
    h.steps(200);

    // 从 A 正下方唤醒 A;T 在 24 m 外,只能经闭包醒来。
    h.set_observer(BELOW_A);
    let mut woke = false;
    for k in 0..15 {
        h.step();
        assert_eq!(h.frozen(a), h.frozen(t), "拍 {k}:A 与 T 必须同拍唤醒");
        if !h.frozen(a) {
            woke = true;
            break;
        }
    }
    assert!(woke, "A 被唤醒时 T 应经闭包同拍醒来");
    h.steps(2); // 票据操作 N+1 生效
    assert_eq!(
        h.transient_ticket_state(t),
        Some(TicketState::Paused),
        "唤醒后 T 仍被承载,票据应回到 Paused 而不是 Active"
    );

    // 把 A 传送到 6 m 外:T 失去支撑,下落、落地、凝固。
    assert_eq!(
        h.block_at(72, -60, 72),
        Some(BlockId::AIR),
        "前置:落点格为空气"
    );
    h.teleport(a, SPOT + Vec3::new(6.0, 0.0, 0.0));
    let mut gone = false;
    for _ in 0..240 {
        h.step();
        if !h.exists(t) {
            gone = true;
            break;
        }
    }
    assert!(gone, "T 失去支撑后应下落并凝固(240 拍内)");
    assert_ne!(
        h.block_at(72, -60, 72),
        Some(BlockId::AIR),
        "T 应凝固在 A 原来所在的格 (72,-60,72)"
    );
    h.steps(2);
    assert_eq!(h.transient_ticket_state(t), None, "凝固后瞬态票据应已终结");
    h.finish();
}
