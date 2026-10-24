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
use shard::tickets::StoredTicket;
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
        self.spawn_actor_sized(e, pos, vel, EntityDesc::default().half_size);
    }

    /// 同 `spawn_actor`,指定立方体半边长(密度 1 → 质量 = (2·half)³ kg)。
    fn spawn_actor_sized(&mut self, e: EntityId, pos: Vec3, vel: Vec3, half_size: f32) {
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
                    desc: EntityDesc {
                        half_size,
                        ..EntityDesc::default()
                    },
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

    /// 任意档位的区域票(`tiers`:0b001 ①、0b010 ②、0b100 ③)。区域票的唤醒范围
    /// 没有滞回(load = unload = radius)。
    fn area_ticket(&mut self, id: u64, center: Vec3, radius_m: f32, tiers: u8) {
        self.session
            .set_area_ticket(id, center, radius_m, tiers, None)
            .expect("区域票");
    }

    fn vel(&self, e: EntityId) -> Vec3 {
        self.session
            .entity_state(origin(), e)
            .unwrap_or_else(|| panic!("实体 {e:?} 应在分片 (0,0)"))
            .linvel
    }

    /// 外部 SetBlock(本地受信命令,owner = 分片 (0,0))。
    fn set_block(&mut self, x: i32, y: i32, z: i32, block: BlockId) {
        self.session
            .set_block(origin(), types::BlockPos::new(x, y, z), block)
            .expect("set_block");
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

/// 场景位置:chunk (4,4) 中部、**方块格 (72,·,72) 的格心**,实体足迹不跨 chunk。
/// 取格心而不是 72.0:72.0 恰在格边界上,物理接触的毫米级漂移会让凝固时的
/// `floor` 落到相邻格(04b 实施中暴露的验收用例缺陷,总架构师修正)。
const SPOT: Vec3 = Vec3::new(72.5, GROUND_Y, 72.5);
const SPOT_CHUNK: ChunkPos = ChunkPos::new(4, 4);

/// 观察者位置:A 正下方 23.5 m。A 距 23.5 m(≤ 24,满足唤醒),B 距 24.5 m(> 24)。
const BELOW_A: Vec3 = Vec3::new(72.5, GROUND_Y - 23.5, 72.5);

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

// ===========================================================================
// 04c:地形变更唤醒(AC-T13;需求 §4.9、§4.11)
// ===========================================================================

/// 观察者的"中距"位置:距 SPOT 约 40 m(> 32 冻结半径,< 48 chunk 保留半径)。
const MID: Vec3 = Vec3::new(112.0, GROUND_Y, 72.5);

/// 在 SPOT 放一个常驻体,静止后让观察者退到中距使其冻结,并**立刻返回**
/// (翻转冷却远未过去)。
fn freshly_frozen_actor(h: &mut Harness) -> EntityId {
    let e = actor(1);
    h.set_observer(Vec3::new(72.5, GROUND_Y, 60.0));
    h.steps(60);
    h.spawn_actor(e, SPOT, Vec3::ZERO);
    h.steps(80);
    h.set_observer(MID);
    for _ in 0..30 {
        h.step();
        if h.frozen(e) {
            return e;
        }
    }
    panic!("前置:静止常驻体在中距应冻结");
}

/// AC-T13 主体:挖掉冻结常驻体脚下的方块,下一拍它被唤醒并落下;唤醒
/// **不受翻转冷却门控**(它刚冻结不久,按距离规则要 180 拍后才可能醒)。
/// 醒来后它在唤醒范围外运动,应持常驻运动票。
///
/// 会变红的实现:冻结体不响应地形变化(悬在空中);地形变更唤醒被冷却挡住;
/// 醒来后不签发常驻票。
#[test]
fn acc_04c_removed_support_wakes_frozen_actor() {
    let mut h = Harness::new(|_| {});
    let e = freshly_frozen_actor(&mut h);
    // 脚下的地表方块:SPOT 中心 y = -59.5,脚底 -60,正下方的格是 (72,-61,72)。
    // 挖 3×3,而不是只挖正下方一格:1 m 立方体与 1 m 坑尺寸相同,边缘恰与四周
    // 顶面重合,接触会把它托住,测不出"落下"。只有正下方那格与实体 AABB 相交,
    // 外圈 8 格只为让坑足够大。
    assert_ne!(
        h.block_at(72, -61, 72),
        Some(BlockId::AIR),
        "前置:脚下有方块"
    );
    for dx in -1..=1 {
        for dz in -1..=1 {
            h.set_block(72 + dx, -61, 72 + dz, BlockId::AIR);
        }
    }

    let mut woke_at = None;
    for k in 0..3 {
        h.step();
        if !h.frozen(e) {
            woke_at = Some(k);
            break;
        }
    }
    assert!(
        woke_at.is_some(),
        "脚下方块被移除后,冻结体应在下一拍被唤醒(不受翻转冷却门控)"
    );
    let mut fell = false;
    let mut saw_ticket = false;
    for _ in 0..60 {
        h.step();
        saw_ticket |= h.has_actor_ticket(e);
        if h.pos(e).y < GROUND_Y - 0.5 {
            fell = true;
            break;
        }
    }
    assert!(fell, "醒来后应落进被挖开的格");
    assert!(saw_ticket, "醒来后在唤醒范围外运动,应持常驻运动票");
    h.finish();
}

/// AC-T13 第二项:醒体预算已满时,地形变更唤醒仍然生效(当拍允许超限)。
///
/// 场景:预算 1;B 在观察者旁边醒着占满预算,A 在 40 m 外冻结。挖掉 A 脚下的
/// 方块后,A 仍须醒来。只断言唤醒那一拍,之后的常规淘汰不在本用例范围内。
///
/// 会变红的实现:地形变更唤醒走"只占空闲预算"或"先唤醒再淘汰"的路径,
/// 预算满时 A 不醒或在唤醒当拍就被淘汰。
#[test]
fn acc_04c_terrain_wake_not_truncated_by_budget() {
    let mut h = Harness::new(|cfg| cfg.activation.max_awake_dynamic = 1);
    let a = freshly_frozen_actor(&mut h);
    let b = actor(2);
    h.spawn_actor(b, MID + Vec3::new(-6.0, 0.0, 0.0), Vec3::ZERO);
    h.steps(40);
    assert!(!h.frozen(b), "前置:B 在观察者旁醒着,占满预算 1");
    assert!(h.frozen(a), "前置:A 冻结");

    h.set_block(72, -61, 72, BlockId::AIR);
    let mut a_woke = false;
    for _ in 0..3 {
        h.step();
        if !h.frozen(a) {
            a_woke = true;
            break;
        }
    }
    assert!(a_woke, "预算已满时,地形变更唤醒仍应生效(当拍允许超限,§4.9)");
    h.finish();
}

/// AC-T13 第三项:地形变更唤醒同样带向上支撑闭包。挖掉 A 脚下的方块,
/// 压在 A 上的 B 必须与 A 同拍醒来。
///
/// 会变红的实现:地形变更唤醒绕过统一唤醒入口,不做闭包(B 悬空)。
#[test]
fn acc_04c_terrain_wake_carries_closure() {
    let mut h = Harness::new(|_| {});
    let (a, b) = stacked_frozen_pair(&mut h);
    h.set_block(72, -61, 72, BlockId::AIR);
    let mut woke = false;
    for k in 0..3 {
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
    assert!(woke, "挖掉 A 脚下方块后,A 与压在其上的 B 应一起醒来");
    h.finish();
}

/// AC-T13 第四项:涌现写入(凝固桥)同样触发地形变更唤醒。一个瞬态方块贴地
/// 生成在冻结常驻体的侧旁(留 0.02 m 缝,小于 skin 0.05 m,不接触、不下落),
/// 静止后凝固成方块 (73,-60,72);新方块与常驻体贴靠,常驻体下一拍醒来。
/// (不让它从空中落下:下落会擦到常驻体边角被弹开,凝固格不可控。)
///
/// 会变红的实现:只对外部 SetBlock 登记地形变更,漏掉凝固/强制落地/BlockWrite
/// 信封等涌现写入。
#[test]
fn acc_04c_solidify_write_wakes_adjacent_frozen_actor() {
    let mut h = Harness::new(|_| {});
    let e = freshly_frozen_actor(&mut h);
    // 瞬态方块贴地放在 x 方向紧邻的格 (73,-60,72),与常驻体之间留 0.02 m。
    let t = EntityId::from_parts(1, 9);
    h.spawn_transient(t, Vec3::new(73.52, GROUND_Y, 72.5));
    let mut solidified_at = None;
    for k in 0..300 {
        h.step();
        if h.block_at(73, -60, 72) != Some(BlockId::AIR) {
            solidified_at = Some(k);
            break;
        }
        assert!(h.frozen(e), "拍 {k}:新方块出现之前,常驻体保持冻结");
    }
    assert!(
        solidified_at.is_some(),
        "前置:瞬态方块应在 (73,-60,72) 凝固(300 拍内)"
    );
    assert!(!h.exists(t), "前置:凝固后瞬态实体消失");
    let mut woke = false;
    for _ in 0..3 {
        h.step();
        if !h.frozen(e) {
            woke = true;
            break;
        }
    }
    assert!(
        woke,
        "凝固写入的格与冻结常驻体贴靠,常驻体应在下一拍被唤醒(涌现写入同样登记)"
    );
    h.finish();
}

// ===========================================================================
// 05a:撞前预测的接触唤醒、静止接触不唤醒(AC-T14、AC-T16;需求 §4.10.1–4.10.2)
// ===========================================================================

/// AC-T14:冻结体对求解器是无限质量。若等接触发生后才唤醒,撞击者会像撞墙一样
/// 被弹回、被撞者拿不到动量;撞前预测要在撞击那一步之前把被撞者唤醒,让 rapier
/// 按两个动态体交换动量。
///
/// 场景:零重力分片(没有地面摩擦干扰),只有"仅装载"的区域票(没有唤醒范围)。
/// B(1 kg 常驻体)静止后冻结;A(1 kg 常驻体)从 5 m 外以 5 m/s 正面撞来。
/// 等质量、恢复系数约 0.1 的正碰:B 应获得明显速度,A 明显减速但不反向。
/// 被撞醒的 B 在唤醒范围外运动,应持常驻运动票。
///
/// 会变红的实现:没有接触唤醒(A 以约 -0.5 m/s 反弹,B 不动);只做撞后补偿
/// (撞击那一步 B 仍是墙,A 已被弹回);唤醒后不签发常驻票。
#[test]
fn acc_05a_prestep_wake_exchanges_momentum() {
    let mut h = Harness::new(|cfg| cfg.gravity = Vec3::ZERO);
    let at = Vec3::new(72.5, GROUND_Y + 20.0, 72.5);
    h.terrain_only_area(at, 32.0);
    h.steps(40);

    let (a, b) = (actor(1), actor(2));
    h.spawn_actor(b, at, Vec3::ZERO);
    let mut b_frozen = false;
    for _ in 0..40 {
        h.step();
        if h.frozen(b) {
            b_frozen = true;
            break;
        }
    }
    assert!(b_frozen, "前置:静止的 B 在唤醒范围外应冻结");

    h.spawn_actor(a, at - Vec3::new(6.0, 0.0, 0.0), Vec3::new(5.0, 0.0, 0.0));
    // 5 m 间隙、5 m/s:约 60 拍后接触;观察到 90 拍。
    h.steps(90);
    let (va, vb) = (h.vel(a), h.vel(b));
    assert!(!h.frozen(b), "B 应在撞击前被唤醒(撞前预测)");
    assert!(
        vb.x > 1.0,
        "B 应从碰撞中获得明显速度(vB.x = {}),说明撞击那一步双方都是动态体",
        vb.x
    );
    assert!(
        va.x > 0.0 && va.x < 4.0,
        "A 应明显减速且不反向弹回(vA.x = {});撞墙式反弹约为 -0.5",
        va.x
    );
    h.steps(2);
    assert!(
        h.has_actor_ticket(b),
        "被撞醒的 B 在唤醒范围外运动,应持常驻运动票"
    );
    h.finish();
}

/// AC-T16:静止接触不触发接触唤醒(冲击动量 m·max(0, v·n̂) ≈ 0)。
///
/// 场景:常规重力。B 在地面,A 叠在 B 上;一张带唤醒范围的区域票罩住 A
/// (唤醒范围没有滞回,A 在半径内、B 在半径外)。于是 A 醒着、静止地压在
/// 冻结的 B 上,持续 600 拍。B 必须一直冻结。
///
/// 这是防误报的守卫用例:在没有接触唤醒的旧实现下本来就是绿的;只有当实现
/// "一有接触就唤醒"或补偿路径把静载冲量当作撞击时才会变红。
#[test]
fn acc_05a_resting_contact_never_wakes() {
    let mut h = Harness::new(|_| {});
    let (b, a) = (actor(1), actor(2));
    // 区域票中心在 SPOT 正上方 5 m:A(叠在 B 上,中心高 1 m)距 4 m,B 距 5 m;
    // 半径 4.5:A 在唤醒范围内,B 在外。
    let center = SPOT + Vec3::new(0.0, 5.0, 0.0);
    h.area_ticket(1, center, 4.5, 0b111);
    h.steps(40);
    h.spawn_actor(b, SPOT, Vec3::ZERO);
    h.steps(40);
    h.spawn_actor(a, SPOT + Vec3::new(0.0, 1.05, 0.0), Vec3::ZERO);
    h.steps(60);
    assert!(
        (h.pos(a).y - (GROUND_Y + 1.0)).abs() < 0.1,
        "前置:A 叠在 B 上(A 的 y = {})",
        h.pos(a).y
    );
    assert!(h.frozen(b), "前置:B 在唤醒范围外静止,应冻结");
    assert!(!h.frozen(a), "前置:A 在唤醒范围内,醒着");

    for k in 0..600 {
        h.step();
        assert!(h.frozen(b), "拍 {k}:A 静止压在 B 上,不应触发接触唤醒");
    }
    h.finish();
}

// ===========================================================================
// 05b:冻结邻接图上的动量传播、成组截断(AC-T15;需求 §4.10.3)
// ===========================================================================

/// 零重力下一排 5 个相互贴靠的冻结常驻体(间隙 0.02 m < skin 0.05 m),沿 +x
/// 排列,B1 在最左。只有"仅装载"的区域票,没有唤醒范围。返回 (B1..B5)。
fn frozen_row_of_five(h: &mut Harness, center: Vec3) -> [EntityId; 5] {
    h.terrain_only_area(center, 32.0);
    h.steps(40);
    let row = [actor(11), actor(12), actor(13), actor(14), actor(15)];
    for (i, e) in row.iter().enumerate() {
        let x = (i as f32 - 2.0) * 1.02;
        h.spawn_actor(*e, center + Vec3::new(x, 0.0, 0.0), Vec3::ZERO);
    }
    let mut all = false;
    for _ in 0..40 {
        h.step();
        if row.iter().all(|e| h.frozen(*e)) {
            all = true;
            break;
        }
    }
    assert!(all, "前置:一排 5 个静止常驻体在唤醒范围外都应冻结");
    row
}

/// 推进直到 `first` 醒来,返回醒来那一拍各成员是否冻结。
fn frozen_flags_when_first_wakes(h: &mut Harness, first: EntityId, row: &[EntityId]) -> Vec<bool> {
    for _ in 0..120 {
        h.step();
        if !h.frozen(first) {
            return row.iter().map(|e| h.frozen(*e)).collect();
        }
    }
    panic!("前置:被直接撞到的 {first:?} 应在撞击前被唤醒(05a 撞前预测)");
}

/// AC-T15 主体(需求 §4.10.3 的示例):1 kg 以 5 m/s 撞进一排 5 个贴靠的冻结体。
/// 种子 B1 的冲击动量 5,沿冻结邻接图每跳 ×0.5:B2 2.5、B3 1.25、B4 0.625
/// (< 门槛 1.0)。默认参数下**恰好唤醒 B1–B3**,而且在同一拍;B4、B5 保持冻结。
///
/// 会变红的实现:只唤醒直接撞到的 B1(没有传播);不衰减或衰减系数不对(醒得
/// 太多);传播分多拍生效。
#[test]
fn acc_05b_row_of_five_wakes_exactly_three() {
    let mut h = Harness::new(|cfg| cfg.gravity = Vec3::ZERO);
    let center = Vec3::new(72.5, GROUND_Y + 20.0, 72.5);
    let row = frozen_row_of_five(&mut h, center);
    // A 从 B1 左侧 5 m 处以 5 m/s 撞来。
    let a = actor(1);
    let b1_x = center.x - 2.0 * 1.02;
    h.spawn_actor(
        a,
        Vec3::new(b1_x - 6.0, center.y, center.z),
        Vec3::new(5.0, 0.0, 0.0),
    );
    let flags = frozen_flags_when_first_wakes(&mut h, row[0], &row);
    assert_eq!(
        flags,
        vec![false, false, false, true, true],
        "p0 = 5、衰减 0.5、门槛 1.0:应在同一拍唤醒 B1–B3,B4、B5 保持冻结(冻结标志 = {flags:?})"
    );
    h.finish();
}

/// AC-T15:接触唤醒只占空闲预算,按组内最大 p 降序纳入,放不下的组跳过,
/// **不淘汰任何已醒实体**。预算 3,撞击者 A 已占 1,空闲 2:只唤醒 B1(p=5)
/// 和 B2(p=2.5),B3(p=1.25)被截断;A 不被淘汰。
///
/// 会变红的实现:接触唤醒越过预算(唤醒 3 个);为腾预算而淘汰 A;截断顺序不按
/// 动量大小。
#[test]
fn acc_05b_contact_wake_uses_only_idle_budget() {
    let mut h = Harness::new(|cfg| {
        cfg.gravity = Vec3::ZERO;
        cfg.activation.max_awake_dynamic = 3;
    });
    let center = Vec3::new(72.5, GROUND_Y + 20.0, 72.5);
    let row = frozen_row_of_five(&mut h, center);
    let a = actor(1);
    let b1_x = center.x - 2.0 * 1.02;
    h.spawn_actor(
        a,
        Vec3::new(b1_x - 6.0, center.y, center.z),
        Vec3::new(5.0, 0.0, 0.0),
    );
    let flags = frozen_flags_when_first_wakes(&mut h, row[0], &row);
    assert_eq!(
        flags,
        vec![false, false, true, true, true],
        "空闲预算 2:只唤醒动量最大的 B1、B2(冻结标志 = {flags:?})"
    );
    assert!(!h.frozen(a), "接触唤醒不得为腾出预算而淘汰已醒的 A");
    h.finish();
}

/// AC-T15:多个种子同拍时,每个节点取所有路径中的最大动量,结果与处理顺序无关。
/// 两侧各有一个 5 m/s 的撞击者同拍撞向 B1 与 B5:B1、B5 为 5,B2、B4 为 2.5,
/// B3 为 1.25(两条路径都是 1.25),5 个全部在同一拍醒来。只从一侧撞时 B4、B5
/// 不醒(见 `acc_05b_row_of_five_wakes_exactly_three`),说明这里的结果来自两侧
/// 传播的合并。
///
/// 会变红的实现:只处理第一个种子;按路径求和而非取最大(仍会全醒,但由
/// 其他两条用例约束);多源传播分多拍生效。
#[test]
fn acc_05b_two_sided_impact_merges_by_max() {
    let mut h = Harness::new(|cfg| cfg.gravity = Vec3::ZERO);
    let center = Vec3::new(72.5, GROUND_Y + 20.0, 72.5);
    let row = frozen_row_of_five(&mut h, center);
    let (a, c) = (actor(1), actor(2));
    let half = 2.0 * 1.02;
    h.spawn_actor(
        a,
        Vec3::new(center.x - half - 6.0, center.y, center.z),
        Vec3::new(5.0, 0.0, 0.0),
    );
    h.spawn_actor(
        c,
        Vec3::new(center.x + half + 6.0, center.y, center.z),
        Vec3::new(-5.0, 0.0, 0.0),
    );
    let flags = frozen_flags_when_first_wakes(&mut h, row[0], &row);
    assert_eq!(
        flags,
        vec![false; 5],
        "两侧同拍撞击:5 个应在同一拍全部醒来(冻结标志 = {flags:?})"
    );
    h.finish();
}

// ===========================================================================
// 05c:撞后冲量补偿路径(AC-T17;需求 §4.10.4、§4.14)
// ===========================================================================

/// 构造一次"撞前预测必然漏判"的撞击(零重力)。一排三块水平贴靠(间隙 0):
/// A1、A2 在唤醒范围内醒着且静止,B 在范围外冻结;C 以 `c_speed` 从 A1 外侧撞来。
///
/// 为什么预测看不到:B 只和 A2 相邻,而 A2 在物理步开始时速度为 0(p = 0);
/// C 离 B 隔着两块(中心距约 3 m),超出 C 的扫掠包络与 B 的包络之和(约 2.1 m)。
/// 这一步里 C→A1→A2 的冲量全部打在 B 上,B 被当作墙,三块一起停下,步后
/// A2 也没有指向 B 的速度——只有撞后冲量补偿能在下一拍唤醒 B。
///
/// 撞击必须赶在 A1、A2 进入引擎睡眠(静止 0.5 s = 30 拍)之前:睡着的 A2 不参与
/// 求解,冲量传不到 B,A1 会带着速度压进 A2,下一拍由预测唤醒 B——那就测不出补偿。
///
/// 返回 B 与"撞击发生的拍"(C 的 x 速度第一次降到 `c_speed` 的 3/4 以下)。
fn missed_prediction_impact(h: &mut Harness, c_speed: f32, c_gap: f32) -> (EntityId, usize) {
    let y = GROUND_Y + 20.0;
    // 唤醒范围:中心在 A1、A2 之间,半径 0.9(区域票无滞回)。A1、A2 中心距 0.5
    // 在内;B 中心距 1.5、最近面 1.0,都在外。
    h.area_ticket(1, Vec3::new(72.5, y, 72.5), 0.9, 0b111);
    h.steps(40);
    let (a1, a2, b, c) = (actor(1), actor(2), actor(3), actor(4));
    h.spawn_actor(b, Vec3::new(74.0, y, 72.5), Vec3::ZERO);
    h.steps(40);
    assert!(h.frozen(b), "前置:B 在唤醒范围外静止,应冻结");
    h.spawn_actor(a1, Vec3::new(72.0, y, 72.5), Vec3::ZERO);
    h.spawn_actor(a2, Vec3::new(73.0, y, 72.5), Vec3::ZERO);
    h.steps(2);
    assert!(
        !h.frozen(a1) && !h.frozen(a2),
        "前置:A1、A2 在唤醒范围内,醒着"
    );
    assert!(h.frozen(b), "前置:与 A2 静止贴靠不唤醒 B");
    // C 的右面与 A1 的左面相距 c_gap。
    h.spawn_actor(
        c,
        Vec3::new(72.0 - 1.0 - c_gap, y, 72.5),
        Vec3::new(c_speed, 0.0, 0.0),
    );
    for k in 0..25 {
        h.step();
        if h.vel(c).x < c_speed * 0.75 {
            return (b, k);
        }
        assert!(h.frozen(b), "拍 {k}:撞击发生前 B 保持冻结");
    }
    panic!("前置:C 应在 25 拍内撞上 A1(赶在 A1、A2 引擎睡眠之前)");
}

/// AC-T17 主体:预测漏判的撞击由撞后冲量补偿,被撞的冻结体在下一拍醒来;
/// 补偿种子随 checkpoint 保存——在撞击那一拍之后截世界快照并恢复,恢复线同样
/// 在下一拍唤醒 B。
///
/// 会变红的实现:没有补偿路径(B 一直冻结,三块被 B 当墙挡住);补偿种子不进
/// 快照或恢复时丢失(恢复线里 B 不醒)。
#[test]
fn acc_05c_missed_prediction_compensated_next_tick() {
    let mut h = Harness::new(|cfg| cfg.gravity = Vec3::ZERO);
    let (b, _k) = missed_prediction_impact(&mut h, 8.0, 1.5);
    assert!(h.frozen(b), "前置:撞击那一拍 B 仍冻结(预测确实漏判)");

    // 撞击那一拍之后、下一拍之前:截世界快照(补偿种子此刻待处理)。
    let snapshot = h.session.world_snapshot();

    // 原线:下一拍(最多两拍)内 B 醒来。
    let mut woke = false;
    for _ in 0..2 {
        h.step();
        if !h.frozen(b) {
            woke = true;
            break;
        }
    }
    assert!(woke, "预测漏判的撞击应由撞后冲量补偿,被撞的 B 在下一拍醒来");

    // 恢复线:同样在下一拍内醒来。
    let mut restored = sim_api::replay::replay_world(
        &snapshot,
        &sim_api::log::CommandLog::new(),
        Config::default().session_config(),
        || Box::new(voxel::SimpleRegistry::builtin()),
    )
    .expect("自有快照必可恢复");
    assert!(
        restored.is_frozen(origin(), b),
        "前置:恢复线起点 B 仍冻结(补偿尚未生效)"
    );
    let mut woke_r = false;
    for _ in 0..2 {
        restored.tick();
        if !restored.is_frozen(origin(), b) {
            woke_r = true;
            break;
        }
    }
    assert!(
        woke_r,
        "补偿种子应随 checkpoint 保存:恢复线里 B 同样在下一拍醒来"
    );
    restored.shutdown();
    h.finish();
}

/// AC-T17 守卫:补偿路径同样受动量门槛约束。C 只以 0.4 m/s(1 kg)推这一排,
/// 打到 B 上的冲量远低于 1.0 kg·m/s,B 不应被唤醒。
///
/// 会变红的实现:补偿对任何非零接触冲量都唤醒(没有用 `contact_wake_min_momentum`
/// 作门槛)。
#[test]
fn acc_05c_weak_push_below_threshold_not_compensated() {
    let mut h = Harness::new(|cfg| cfg.gravity = Vec3::ZERO);
    let (b, _k) = missed_prediction_impact(&mut h, 0.4, 0.05);
    for k in 0..60 {
        h.step();
        assert!(h.frozen(b), "拍 {k}:低于门槛的推挤不应触发补偿唤醒");
    }
    h.finish();
}

/// AC-T16 在补偿路径上的守卫:醒着的重物静压在冻结体上。边长 2 m(8 kg)的 H
/// 压在冻结的边长 3 m(27 kg)的 B 顶上,每步支撑冲量 m·g·dt ≈ 1.31 kg·m/s,
/// 高于门槛 1.0——但这是静载,不是撞击。补偿不计支撑方向的接触(§4.10.4),
/// 静载不得唤醒 B。
///
/// 会变红的实现:补偿把支撑接触的冲量也和门槛比较——B 一冻结就被静载唤醒,
/// 醒 → 静止 → 冻结 → 又醒,循环往复。
#[test]
fn acc_05c_static_load_never_compensates() {
    let mut h = Harness::new(|_| {});
    // B 落在地面上;H 放在 B 顶面正上方。
    let b_spawn = Vec3::new(72.5, GROUND_Y + 1.0, 72.5);
    let top_spawn = Vec3::new(72.5, GROUND_Y + 3.5, 72.5);
    // 唤醒范围:中心在 H 上方 0.5 m、半径 1.2。H 中心距约 0.5 在内,B 中心距约 3.0 在外。
    h.area_ticket(1, top_spawn + Vec3::new(0.0, 0.5, 0.0), 1.2, 0b111);
    h.steps(40);
    let (b, top) = (actor(1), actor(2));
    h.spawn_actor_sized(b, b_spawn, Vec3::ZERO, 1.5);
    let mut frozen = false;
    for _ in 0..300 {
        h.step();
        if h.frozen(b) {
            frozen = true;
            break;
        }
    }
    assert!(frozen, "前置:B 在唤醒范围外静止,应冻结");

    h.spawn_actor_sized(top, top_spawn, Vec3::ZERO, 1.0);
    // 放上去的那一下(新接触、去穿透)可以有扰动;先让它过去,再等"B 冻结、
    // H 醒着压在上面"的稳态出现。(冻结支撑者会连带冻结上面的醒体;H 在唤醒
    // 范围内,下一拍自己醒来。)
    h.steps(120);
    let mut settled = false;
    for _ in 0..600 {
        h.step();
        if h.frozen(b) && !h.frozen(top) {
            settled = true;
            break;
        }
    }
    assert!(settled, "前置:应出现 B 冻结、H 醒着压在上面的稳态");

    for k in 0..600 {
        h.step();
        assert!(h.frozen(b), "拍 {k}:静载不应触发补偿唤醒");
        assert!(!h.frozen(top), "拍 {k}:H 在唤醒范围内,应保持醒着");
    }
    let rel = h.pos(top) - h.pos(b);
    assert!(
        (rel - Vec3::new(0.0, 2.5, 0.0)).length() < 0.1,
        "H 始终压在 B 顶上(否则本用例不成立):相对位置 {rel:?}"
    );
    h.finish();
}

// ===========================================================================
// 06a:不变量 I3 与操作幂等(AC-T19;需求 §4.3.1、§4.3.6)
// ===========================================================================
//
// 违反用测试钩子 `Session::test_hook_tickets_mut` 制造(feature `test-hooks`,只在
// 测试构建里打开):在两拍之间改票据表,其后一拍末的 I3 校验必须 fail-stop。
// 约定 panic 消息含被违反条款的编号 `I3-<n>`(n 为 §4.3.6 列表序号 1–4),用例
// 据此确认是哪一条拦下的。

fn transient_key(e: EntityId) -> (TicketHolder, TicketKind) {
    (TicketHolder::Entity(e), TicketKind::TransientMotion)
}

/// 某实体持有的全部实体票种类(键序)。
fn entity_ticket_kinds(h: &Harness, e: EntityId) -> Vec<TicketKind> {
    h.session
        .ticket_table()
        .iter()
        .filter(|(k, _)| k.0 == TicketHolder::Entity(e))
        .map(|(k, _)| k.1)
        .collect()
}

impl Harness {
    /// 带初速度的瞬态体。
    fn spawn_transient_v(&mut self, e: EntityId, pos: Vec3, vel: Vec3) {
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
            .expect("spawn transient");
    }

    /// 外部 MarkActor(owner = 分片 (0,0))。
    fn mark_actor(&mut self, e: EntityId, actor: bool) {
        self.session
            .send(origin(), SimCommand::MarkActor { entity: e, actor })
            .expect("mark actor");
    }

    /// 某张票的时钟。
    fn ticket_clock(&self, key: (TicketHolder, TicketKind)) -> physics_sim::tickets::TicketClock {
        self.session
            .ticket_table()
            .get(key)
            .unwrap_or_else(|| panic!("票 {key:?} 应存在"))
            .clock
            .clone()
    }
}

/// 启用激活:观察者在 SPOT 南侧 12.5 m,等地形就绪。
fn enable_activation(h: &mut Harness) {
    h.set_observer(Vec3::new(72.5, GROUND_Y, 60.0));
    h.steps(60);
}

/// 在 `at` 放一个瞬态体让它下落(5 m 约 1 s,期间一直持票),返回它的
/// TransientMotion 票副本——钩子制造违反时用它当合法的票据原料。
fn ticket_of_falling_transient(h: &mut Harness, e: EntityId, at: Vec3) -> StoredTicket {
    h.spawn_transient(e, at);
    h.steps(3);
    h.session
        .ticket_table()
        .get(transient_key(e))
        .cloned()
        .expect("前置:下落中的瞬态体持 TransientMotion 票")
}

/// I3 第 1 条:存活的瞬态体必须恰持一张 TransientMotion 票。钩子删掉一个下落中
/// 瞬态体的票,其后一拍末 fail-stop。
///
/// 会变红的实现:没有 I3 校验;校验不在 `tick` 的生产路径上(只在单测里调用);
/// 只查"票指向的实体还在",不查"实体都有票"。
#[test]
#[should_panic(expected = "I3-1")]
fn acc_06a_i3_live_transient_without_ticket_fails_stop() {
    let mut h = Harness::new(|_| {});
    enable_activation(&mut h);
    let t = EntityId::from_parts(1, 9);
    let _ = ticket_of_falling_transient(&mut h, t, SPOT + Vec3::new(4.0, 5.0, 0.0));
    h.session.test_hook_tickets_mut().end(transient_key(t));
    assert!(h.exists(t), "前置:瞬态体仍在下落");
    h.steps(3);
}

/// I3 第 2 条:实体票的持有者必须存活或迁移在途。钩子给一个从未存在过的实体
/// 签一张票,其后一拍末 fail-stop。
///
/// 会变红的实现:只按"实体都有票"单向检查;不存在的持有者被静默跳过。
#[test]
#[should_panic(expected = "I3-2")]
fn acc_06a_i3_ticket_for_dead_entity_fails_stop() {
    let mut h = Harness::new(|_| {});
    enable_activation(&mut h);
    let t = EntityId::from_parts(1, 9);
    let ticket = ticket_of_falling_transient(&mut h, t, SPOT + Vec3::new(4.0, 5.0, 0.0));
    let ghost = EntityId::from_parts(1, 777);
    let now = h.session.tick_now();
    h.session
        .test_hook_tickets_mut()
        .issue(transient_key(ghost), ticket, now);
    h.steps(3);
}

/// I3 第 3 条:常驻体不持 TransientMotion。钩子给一个醒着的常驻体塞一张瞬态票,
/// 其后一拍末 fail-stop。
///
/// 会变红的实现:不核对票种与实体的 actor 标记;只检查瞬态体一侧。
#[test]
#[should_panic(expected = "I3-3")]
fn acc_06a_i3_actor_holding_transient_ticket_fails_stop() {
    let mut h = Harness::new(|_| {});
    enable_activation(&mut h);
    let a = actor(1);
    h.spawn_actor(a, SPOT, Vec3::ZERO);
    let t = EntityId::from_parts(1, 9);
    let ticket = ticket_of_falling_transient(&mut h, t, SPOT + Vec3::new(4.0, 5.0, 0.0));
    assert!(h.exists(a), "前置:常驻体存活");
    let now = h.session.tick_now();
    h.session
        .test_hook_tickets_mut()
        .issue(transient_key(a), ticket, now);
    h.steps(3);
}

/// I3 第 4 条(前半):冻结的常驻体不持 ActorMotion。钩子给刚冻结的常驻体塞一张
/// 常驻运动票,其后一拍末 fail-stop。
///
/// 会变红的实现:I3 不看冻结状态;冻结信息没有从分片汇聚到调度器。
#[test]
#[should_panic(expected = "I3-4")]
fn acc_06a_i3_frozen_actor_holding_motion_ticket_fails_stop() {
    let mut h = Harness::new(|_| {});
    let e = freshly_frozen_actor(&mut h);
    let t = EntityId::from_parts(1, 9);
    let ticket = ticket_of_falling_transient(&mut h, t, MID + Vec3::new(0.0, 5.0, 0.0));
    assert!(
        h.frozen(e) && !h.has_actor_ticket(e),
        "前置:常驻体冻结且不持票"
    );
    let now = h.session.tick_now();
    h.session.test_hook_tickets_mut().issue(
        (TicketHolder::Entity(e), TicketKind::ActorMotion),
        ticket,
        now,
    );
    h.steps(3);
}

/// I3 第 4 条(后半):承载静止冻结的瞬态体,其 TransientMotion 必须是 Suspended。
/// 场景同 04b:瞬态方块 T 压在常驻体 A 上,两者在远处静止冻结、T 的票挂起;
/// 钩子把 T 的票改回 Active,其后一拍末 fail-stop。
///
/// 会变红的实现:I3 只查票是否存在,不查承载冻结体的票据状态。
#[test]
#[should_panic(expected = "I3-4")]
fn acc_06a_i3_carried_frozen_transient_must_stay_suspended() {
    let mut h = Harness::new(|_| {});
    let (a, t) = (actor(1), EntityId::from_parts(1, 9));
    h.set_observer(Vec3::new(72.0, GROUND_Y, 60.0));
    h.steps(60);
    h.spawn_actor(a, SPOT, Vec3::ZERO);
    h.steps(40);
    h.spawn_transient(t, SPOT + Vec3::new(0.0, 1.05, 0.0));
    h.steps(80);
    h.set_observer(Vec3::new(112.0, GROUND_Y, 72.0));
    let mut suspended = false;
    for _ in 0..60 {
        h.step();
        if h.frozen(a) && h.frozen(t) && h.transient_ticket_state(t) == Some(TicketState::Suspended)
        {
            suspended = true;
            break;
        }
    }
    assert!(suspended, "前置:A、T 冻结,T 的票挂起");
    h.session
        .test_hook_tickets_mut()
        .set_state(transient_key(t), TicketState::Active);
    h.steps(3);
}

/// AC-T19 守卫:I3 不得误报。激活启用后,瞬态体与常驻体各一个飞越 x = 128 的
/// 分片边界(迁移),瞬态体在新分片落地凝固;另一个瞬态体在半空中先切成常驻、
/// 再切回瞬态,落地凝固。全程每拍末 I3 都必须成立(不 panic)。
///
/// 会变红的实现:I3 把迁移在途的实体当成已终结,或把在途实体的票当成悬空票
/// (在途窗口误报 I3-1/I3-2);凝固终结与迁移 despawn 混淆;MarkActor 切换当拍
/// 票据没收口。
#[test]
fn acc_06a_i3_quiet_across_migration_solidify_and_mark_actor() {
    let mut h = Harness::new(|_| {});
    h.set_observer(Vec3::new(128.0, GROUND_Y, 72.5));
    h.steps(120);
    let east = shard_id(1, 0);
    let (t1, a, t2) = (
        EntityId::from_parts(1, 9),
        actor(1),
        EntityId::from_parts(1, 10),
    );
    h.spawn_transient_v(
        t1,
        Vec3::new(124.5, GROUND_Y + 1.5, 72.5),
        Vec3::new(6.0, 2.0, 0.0),
    );
    h.spawn_actor(
        a,
        Vec3::new(124.5, GROUND_Y + 1.5, 76.5),
        Vec3::new(6.0, 2.0, 0.0),
    );
    h.spawn_transient(t2, Vec3::new(120.5, GROUND_Y + 15.0, 68.5));
    h.steps(5);
    h.mark_actor(t2, true);
    h.steps(30);
    h.mark_actor(t2, false);
    let mut t1_crossed = false;
    for _ in 0..600 {
        h.step();
        t1_crossed |= h.session.entity_state(east, t1).is_some();
    }
    assert!(t1_crossed, "前置:瞬态体 T1 飞越边界进入分片 (1,0)");
    assert!(
        h.session.entity_state(east, a).is_some(),
        "前置:常驻体 A 落在分片 (1,0)"
    );
    assert_eq!(
        entity_ticket_kinds(&h, a),
        Vec::<TicketKind>::new(),
        "A 在唤醒范围内静止:不持实体票"
    );
    for e in [t1, t2] {
        assert!(
            !h.exists(e) && h.session.entity_state(east, e).is_none(),
            "瞬态体 {e:?} 应已落地凝固"
        );
        assert_eq!(
            entity_ticket_kinds(&h, e),
            Vec::<TicketKind>::new(),
            "瞬态体 {e:?} 凝固后票据全部终结"
        );
    }
    h.finish();
}

/// AC-T19(幂等,走生产路径):同一拍里重复的 MarkActor、以及对已是瞬态的实体
/// 再发 MarkActor(false),都不得重签票据或重置时钟;切回瞬态时新票的时钟从切换
/// 拍起算。
///
/// 会变红的实现:实体票 Issue 遇键已在时覆盖(时钟被重置);重复 MarkActor 报错
/// 或留下两种票;切回瞬态沿用旧票时钟。
#[test]
fn acc_06a_repeated_mark_actor_keeps_ticket_clock() {
    let mut h = Harness::new(|_| {});
    enable_activation(&mut h);
    let t = EntityId::from_parts(1, 9);
    // 30 m 高处下落约 2.5 s,整个用例期间都在空中。
    h.spawn_transient(t, SPOT + Vec3::new(4.0, 30.0, 0.0));
    h.steps(3);
    assert!(h.transient_ticket_state(t).is_some(), "前置:瞬态体持票");

    h.mark_actor(t, true);
    h.mark_actor(t, true);
    h.steps(2);
    assert!(
        !entity_ticket_kinds(&h, t).contains(&TicketKind::TransientMotion),
        "切成常驻后不再持瞬态票"
    );

    h.mark_actor(t, false);
    h.mark_actor(t, false);
    h.steps(2);
    assert_eq!(
        entity_ticket_kinds(&h, t),
        vec![TicketKind::TransientMotion],
        "切回瞬态:恰持一张瞬态票"
    );
    let first = h.ticket_clock(transient_key(t));
    assert!(
        first.active_ticks <= 2,
        "新瞬态票的时钟从切换拍起算:{first:?}"
    );

    h.steps(10);
    h.mark_actor(t, false); // 已是瞬态:无操作
    h.steps(2);
    let later = h.ticket_clock(transient_key(t));
    assert_eq!(
        later.issued_at, first.issued_at,
        "重复 MarkActor(false) 不重签"
    );
    assert_eq!(
        later.active_ticks,
        first.active_ticks + 12,
        "时钟连续走,不被重置"
    );
    h.finish();
}

// ===========================================================================
// 06b:快照收口与中途 checkpoint 一致(AC-T21、AC-T24;需求 §4.0、§4.14)
// ===========================================================================
//
// 恢复一律走 checkpoint 的编码:世界快照经 bincode 往返,再 `replay_world`,
// 套上与原线相同的编排(新建的 provider/store——Harness 不落盘卸载的 chunk,
// 两线重新装载时都从生成器取,天然一致)。快照前后都不发外部命令,恢复用
// 空日志。

/// 世界快照 → bincode 往返 → `replay_world` → 带编排的恢复线。
fn restore_harness(snapshot: &sim_api::snapshot::WorldSnapshot) -> Harness {
    let bytes = bincode::serde::encode_to_vec(snapshot, bincode::config::standard())
        .expect("世界快照可编码");
    let (decoded, _): (sim_api::snapshot::WorldSnapshot, usize) =
        bincode::serde::decode_from_slice(&bytes, bincode::config::standard())
            .expect("世界快照可解码");
    let config = Config::default();
    let store = storage::worker::StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(storage::mem::MemCheckpointStore::new()),
    );
    let session = sim_api::replay::replay_world(
        &decoded,
        &sim_api::log::CommandLog::new(),
        config.session_config(),
        || Box::new(voxel::SimpleRegistry::builtin()),
    )
    .expect("自有快照必可恢复");
    let (provider, new_sim) = app::driver::wire_provider_and_new_sim(config, &store, || {
        Box::new(voxel::SimpleRegistry::builtin())
    });
    Harness {
        session,
        store,
        provider,
        new_sim,
    }
}

/// 一条线在某一拍的可比状态:票据表(含时钟与状态)、全部实体(状态逐位、
/// 冻结、待地形)、全部已注册分片的 ①② 集合。完整的调度器快照(另含观察者、
/// 在途信封、生命周期)序列化代价高,由 `assert_lines_match` 每 50 拍比一次。
#[derive(Debug, PartialEq)]
struct LineProbe {
    tick: types::Tick,
    tickets: shard::tickets::TicketTable,
    entities: Vec<(ShardId, EntityId, EntityState, bool, bool)>,
    chunks: Vec<(ShardId, ChunkPos, bool, bool)>,
}

fn probe(h: &Harness) -> LineProbe {
    let mut entities = Vec::new();
    let mut chunks = Vec::new();
    for shard in h.session.shard_ids() {
        for (e, st) in h.session.entity_states(shard).unwrap_or_default() {
            entities.push((
                shard,
                e,
                st,
                h.session.is_frozen(shard, e),
                h.session.is_awaiting_terrain(shard, e),
            ));
        }
        for cx in shard.pos.x * 8..shard.pos.x * 8 + 8 {
            for cz in shard.pos.z * 8..shard.pos.z * 8 + 8 {
                let cp = ChunkPos::new(cx, cz);
                chunks.push((
                    shard,
                    cp,
                    h.session.is_chunk_loaded(shard, cp),
                    h.session.has_collider(shard, cp),
                ));
            }
        }
    }
    LineProbe {
        tick: h.session.tick_now(),
        tickets: h.session.ticket_table().clone(),
        entities,
        chunks,
    }
}

/// 两线逐拍比对 `ticks` 拍(分项断言,出错时指明哪一项、哪一拍);完整调度器
/// 快照在起点、每 50 拍与终点比对。
fn assert_lines_match(a: &mut Harness, b: &mut Harness, ticks: usize) {
    let (pa, pb) = (probe(a), probe(b));
    assert_eq!(pa, pb, "恢复后起点两线一致");
    for k in 0..ticks {
        a.step();
        b.step();
        let (pa, pb) = (probe(a), probe(b));
        assert_eq!(pa.tick, pb.tick, "拍号对齐");
        assert_eq!(
            pa.tickets, pb.tickets,
            "快照后第 {k} 拍:票据表(键、规格、时钟、状态)两线不一致"
        );
        if k % 50 == 0 || k + 1 == ticks {
            assert_eq!(
                a.session.world_snapshot().scheduler,
                b.session.world_snapshot().scheduler,
                "快照后第 {k} 拍:调度器快照(观察者、在途信封、生命周期等)两线不一致"
            );
        }
        assert_eq!(
            pa.entities, pb.entities,
            "快照后第 {k} 拍:实体状态(逐位、冻结、待地形)两线不一致"
        );
        assert_eq!(pa.chunks, pb.chunks, "快照后第 {k} 拍:①② 集合两线不一致");
    }
}

/// AC-T21 主体:四种票、票据三态、承载、冻结、待地形、①② 冷却同时在场时截
/// checkpoint;恢复线与原线逐拍位级一致 600 拍,终态体素一致。
///
/// 快照时刻在场的状态:观察者票;只装载、带 TTL 的区域票(到期落在窗口内);
/// 下落中的瞬态体 F(Active,窗口内落地凝固);常驻体 A 上压着瞬态 C(Paused);
/// 区域票下静止冻结的常驻体 B 上压着瞬态 D(Suspended);区域票下坠落中的常驻
/// 体 M(持 ActorMotion);刚出生在无地形处、带初速度的瞬态体 W(待地形,保存
/// 速度);观察者刚挪动 12 m(①② 装卸与冷却簿记跨过快照)。
///
/// 会变红的实现:§4.14 任一项没进快照或没恢复——票据时钟/状态、区域票 TTL、
/// 待地形的保存速度、承载记录、①② 集合或冷却簿记、静止判据窗口;快照字段被
/// serde 跳过而按默认值恢复。
#[test]
#[ignore = "REQ-007 06b 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_06b_midway_checkpoint_replays_identically() {
    let mut h = Harness::new(|_| {});
    h.set_observer(Vec3::new(72.5, GROUND_Y, 60.0));
    let far = Vec3::new(110.5, GROUND_Y, 72.5);
    h.session
        .set_area_ticket(7, far, 8.0, 0b011, Some(500))
        .expect("区域票");
    h.steps(60);
    let (a, c, b, d) = (
        actor(1),
        EntityId::from_parts(1, 11),
        actor(2),
        EntityId::from_parts(1, 12),
    );
    h.spawn_actor(a, SPOT, Vec3::ZERO);
    h.spawn_actor(b, far, Vec3::ZERO);
    h.steps(40);
    h.spawn_transient(c, SPOT + Vec3::new(0.0, 1.05, 0.0));
    h.spawn_transient(d, far + Vec3::new(0.0, 1.05, 0.0));
    h.steps(120);
    assert_eq!(
        h.transient_ticket_state(c),
        Some(TicketState::Paused),
        "前置:C 被承载"
    );
    assert_eq!(
        h.transient_ticket_state(d),
        Some(TicketState::Suspended),
        "前置:D 承载静止冻结"
    );
    assert!(h.frozen(b) && !h.frozen(a), "前置:B 冻结、A 醒着");

    let (f, m, w) = (
        EntityId::from_parts(1, 13),
        actor(3),
        EntityId::from_parts(1, 14),
    );
    h.spawn_transient(f, SPOT + Vec3::new(4.0, 30.0, 0.0));
    h.spawn_actor(m, far + Vec3::new(4.0, 15.0, -4.0), Vec3::ZERO);
    h.steps(10);
    h.set_observer(Vec3::new(72.5, GROUND_Y, 72.0));
    h.steps(3);
    h.spawn_transient_v(
        w,
        Vec3::new(8.5, GROUND_Y + 3.0, 120.5),
        Vec3::new(1.0, 2.0, 0.5),
    );
    // 出生那拍先按动态体走一步,下一拍的激活检查发现足迹缺地形 → 待地形。
    h.steps(2);
    assert!(
        h.session.is_awaiting_terrain(origin(), w),
        "前置:W 出生在无地形处,待地形"
    );
    assert!(
        h.has_actor_ticket(m),
        "前置:M 在唤醒范围外坠落,持常驻运动票"
    );
    assert_eq!(
        h.transient_ticket_state(f),
        Some(TicketState::Active),
        "前置:F 下落中"
    );

    let snapshot = h.session.world_snapshot();
    let mut r = restore_harness(&snapshot);
    assert_lines_match(&mut h, &mut r, 600);

    // 非退化:窗口内区域票到期、F 落地凝固、W 恢复运动。
    assert!(
        h.session
            .ticket_table()
            .get((TicketHolder::Area(7), TicketKind::Area))
            .is_none(),
        "区域票在窗口内到期(两线已逐拍比对)"
    );
    assert!(!h.exists(f), "F 在窗口内落地凝固");
    assert!(!h.session.is_awaiting_terrain(origin(), w), "W 已恢复运动");
    // 终态体素:两线全部已装载 chunk 的内容逐格相等。
    for shard in h.session.shard_ids() {
        for cx in shard.pos.x * 8..shard.pos.x * 8 + 8 {
            for cz in shard.pos.z * 8..shard.pos.z * 8 + 8 {
                let cp = ChunkPos::new(cx, cz);
                assert_eq!(
                    h.session.voxel_chunk(shard, cp),
                    r.session.voxel_chunk(shard, cp),
                    "终态体素两线一致:{shard:?} {cp:?}"
                );
            }
        }
    }
    h.finish();
    r.finish();
}

/// §4.14"待处理的地形变更唤醒格"随快照保存:挖掉冻结常驻体脚下的方块,写入
/// 当拍截快照(唤醒在下一拍消费),恢复线与原线一样在下一拍唤醒它,并逐拍一致。
///
/// 会变红的实现:待处理唤醒格不进快照(恢复线里常驻体一直悬空冻结)。
#[test]
#[ignore = "REQ-007 06b 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_06b_pending_terrain_wake_survives_checkpoint() {
    let mut h = Harness::new(|_| {});
    let e = freshly_frozen_actor(&mut h);
    let p = h.pos(e);
    let below = (
        p.x.floor() as i32,
        p.y.floor() as i32 - 1,
        p.z.floor() as i32,
    );
    h.set_block(below.0, below.1, below.2, BlockId(0));
    h.step();
    assert_eq!(
        h.block_at(below.0, below.1, below.2),
        Some(BlockId(0)),
        "前置:写入已生效"
    );
    assert!(h.frozen(e), "前置:写入当拍,唤醒尚未消费");
    let snapshot = h.session.world_snapshot();
    let mut r = restore_harness(&snapshot);
    assert_lines_match(&mut h, &mut r, 120);
    assert!(!r.frozen(e), "恢复线里常驻体同样被地形变更唤醒");
    h.finish();
    r.finish();
}

/// AC-T24:激活启用开关随快照往返,且单向不回退。
/// - 从未启用的会话:快照恢复后仍未启用,新生成的瞬态体不签票;
/// - 启用后又撤掉全部观察者的会话:快照恢复后仍是启用态,新瞬态体照常签票。
///
/// 会变红的实现:开关不进快照(恢复后按"观察者表是否为空"重新推断——第二种
/// 情况会退回未启用);开关随观察者移除回退。
#[test]
#[ignore = "REQ-007 06b 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_06b_activation_switch_survives_checkpoint() {
    let t = EntityId::from_parts(1, 9);

    let mut never = Harness::new(|_| {});
    never.steps(5);
    assert!(!never.session.activation_enabled(), "前置:从未启用");
    let mut r = restore_harness(&never.session.world_snapshot());
    assert!(
        !r.session.activation_enabled(),
        "未启用的快照恢复后仍未启用"
    );
    r.spawn_transient(t, SPOT + Vec3::new(0.0, 5.0, 0.0));
    r.steps(3);
    assert!(
        r.transient_ticket_state(t).is_none(),
        "未启用:恢复线不签瞬态票"
    );
    never.finish();
    r.finish();

    let mut once = Harness::new(|_| {});
    once.set_observer(Vec3::new(72.5, GROUND_Y, 60.0));
    once.steps(30);
    once.remove_observer();
    once.steps(5);
    assert!(
        once.session.activation_enabled(),
        "前置:启用后撤掉观察者仍是启用态"
    );
    let mut r = restore_harness(&once.session.world_snapshot());
    assert!(
        r.session.activation_enabled(),
        "启用后的快照恢复后仍是启用态(单向)"
    );
    r.spawn_transient(t, SPOT + Vec3::new(0.0, 5.0, 0.0));
    r.steps(3);
    assert!(
        r.transient_ticket_state(t).is_some(),
        "恢复线按启用规则签瞬态票"
    );
    once.finish();
    r.finish();
}

/// §4.6 待地形恢复"速度原样写回"(任务 03 遗留缺陷,06b 一并修):带初速度
/// (1, 2, 0.5) 的瞬态体出生在无地形处,进入待地形;地形就绪后恢复,恢复那一步
/// 的水平速度必须仍是 (1, ·, 0.5),竖直速度只少一拍重力。
///
/// 会变红的实现:恢复时先在 kinematic 状态下写速度、再切回 Dynamic——rapier
/// 0.36 对 KinematicPositionBased 刚体的 `set_linvel` 不生效,保存的速度丢失,
/// 实体从零速度起步(当前代码即如此)。
#[test]
#[ignore = "REQ-007 06b 验收用例:实现完成后由实施方移除 ignore,不得修改断言"]
fn acc_06b_awaiting_terrain_resumes_with_saved_velocity() {
    let mut h = Harness::new(|_| {});
    enable_activation(&mut h);
    let w = EntityId::from_parts(1, 14);
    h.spawn_transient_v(
        w,
        Vec3::new(8.5, GROUND_Y + 3.0, 120.5),
        Vec3::new(1.0, 2.0, 0.5),
    );
    let mut before: Option<Vec3> = None;
    let mut resumed: Option<Vec3> = None;
    for _ in 0..20 {
        let v = h.session.entity_state(origin(), w).map(|s| s.linvel);
        h.step();
        let awaiting = h.session.is_awaiting_terrain(origin(), w);
        if awaiting && before.is_none() {
            before = v; // 进入待地形前一拍末的速度 = 应保存的速度
        }
        if !awaiting && before.is_some() {
            resumed = h.session.entity_state(origin(), w).map(|s| s.linvel);
            break;
        }
    }
    let before = before.expect("前置:W 进入待地形");
    let resumed = resumed.expect("前置:地形就绪后 W 恢复运动");
    let g_dt = 9.81_f32 / 60.0;
    assert!(
        (resumed.x - before.x).abs() < 1e-4 && (resumed.z - before.z).abs() < 1e-4,
        "恢复后水平速度原样写回:进入前 {before:?},恢复后 {resumed:?}"
    );
    assert!(
        (resumed.y - (before.y - g_dt)).abs() < 1e-3,
        "恢复后竖直速度只少一拍重力:进入前 {before:?},恢复后 {resumed:?}"
    );
    h.finish();
}
