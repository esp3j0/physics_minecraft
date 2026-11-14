//! 启动主体(REQ-012 B02b §2.2):配置 `[[agents]]` 的待生成簿记与每拍
//! 轮询。**只在全新启动**(没有恢复检查点)带配置;恢复启动传空表——
//! 世界快照里已有这些主体(§2.2 契约),重新生成会造出第二只(E2)。
//!
//! 每拍在 `session.tick()` 之前(§2.3 顺序第 ① 步)对每条待生成项:
//! 1. 第一次见到时放一张区域票据(中心 = position,半径 =
//!    `leash_m + STARTUP_TICKET_EXTRA_M`,常驻,①② 档)——出生点区块靠它
//!    装载(E8 的"不会被装载"布景就是不放观察者、只靠这张票也够不着);
//! 2. 出生列方块可读且该处碰撞体已建好 → 生成(InputSource::SCHEDULER,
//!    条目进日志;复用 A06 重生等装载的同一组判定缝:
//!    `shard_ids().contains && is_chunk_loaded && has_collider &&
//!    block_solid_at` 可读),挂控制者(host.attach,leash 圆心 =
//!    position),从 pending 移除,`spawned += 1`;
//! 3. 等待超过 [`STARTUP_SPAWN_TIMEOUT_TICKS`] → 放弃这一条,
//!    `spawn_timeouts += 1`,写一条错误日志(eprintln),不 panic。
//!
//! 实体 ID 用驱动现有的服务端序号分配(`spawn_next_seq` 那一套,#12 语义,
//! 经 `alloc_id` 闭包注入);票据 ID 取保留区间 [`STARTUP_TICKET_BASE`,
//! `STARTUP_TICKET_BASE + STARTUP_AGENTS_MAX`)(u64 高位段,与连接/测试
//! 用的小值 1、7、8 等永不冲突——现有区域票 id 无校验上限,保留区间是
//! 约定)。

use messaging::InputSource;
use sim_api::log::CommandLog;
use sim_api::session::Session;
use types::{BlockPos, EntityId, Tick};

use crate::config::{
    STARTUP_AGENTS_MAX, STARTUP_TICKET_EXTRA_M, StartupAgentConfig, StartupController,
};
use crate::controller::{
    ControllerHost, ControllerKind, DEFAULT_CONTROL_PERIOD_TICKS, Leash, ScriptKind,
};

/// 出生点等待超时(拍;§2.2 = 600 拍 = 10 s @ 60 Hz)。
pub const STARTUP_SPAWN_TIMEOUT_TICKS: u32 = 600;

/// startup 区域票的保留 ID 区间基址(u64 高位段;`BASE .. BASE + 16`)。
/// 连接发来的区域票 id 没有校验上限,本区间纯属约定——现有生产路径
/// (`DriverCommand::SetAreaTicket` 本地管理面)与测试(1、7、8)都在低位。
pub const STARTUP_TICKET_BASE: u64 = u64::MAX - (STARTUP_AGENTS_MAX as u64) + 1;

/// 单条待生成项。
#[derive(Debug, Clone)]
struct PendingStartup {
    /// 配置里的条目序号(1 起;错误信息用)。
    index: usize,
    cfg: StartupAgentConfig,
    /// 身体哈希(构造时从注册表解析)。
    hash: types::BodyHash,
    /// 身体自由度(挂载校验)。
    dof: usize,
    /// 区域票 ID(保留区间内按条目序分配)。
    ticket_id: u64,
    /// 已等待拍数。
    waited: u32,
}

/// 启动主体计数(遥测;DriverOutcome.startup_counters)。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StartupCounters {
    /// 已生成(含挂控制者)。
    pub spawned: u64,
    /// 等装载超时放弃的条数。
    pub spawn_timeouts: u64,
}

/// 构造失败(类型化;校验前置在 `Config::validate*`,`new` 只对"配置有
/// 条目但注册表里查不到身体"兜底)。
#[derive(Debug, thiserror::Error)]
#[error("启动主体构造失败:{0}")]
pub struct StartupAgentsError(pub String);

/// 启动主体簿记。
#[derive(Debug, Default)]
pub struct StartupAgents {
    pending: Vec<PendingStartup>,
    pub counters: StartupCounters,
}

impl StartupAgents {
    /// 只在全新启动(没有恢复检查点)时带上配置;恢复启动传空表。
    /// 每条 `body` 必须在注册表里(配置校验已保证;查不到 = 类型化错误)。
    pub fn new(
        cfgs: &[StartupAgentConfig],
        bodies: &physics_sim::BodyRegistry,
    ) -> Result<Self, StartupAgentsError> {
        let mut pending = Vec::new();
        for (i, cfg) in cfgs.iter().enumerate() {
            let hash = bodies.hash_of(&cfg.body).ok_or_else(|| {
                StartupAgentsError(format!(
                    "第 {} 条:身体 \"{}\" 不在注册表里",
                    i + 1,
                    cfg.body
                ))
            })?;
            let dof = bodies.get(&hash).map(|def| def.dof_count()).unwrap_or(0);
            // 票据 ID:保留区间内按条目序分配(条数 ≤ STARTUP_AGENTS_MAX,
            // 配置校验保证不越界)。
            let ticket_id = STARTUP_TICKET_BASE + i as u64;
            pending.push(PendingStartup {
                index: i + 1,
                cfg: cfg.clone(),
                hash,
                dof,
                ticket_id,
                waited: 0,
            });
        }
        Ok(Self {
            pending,
            counters: StartupCounters::default(),
        })
    }

    /// 每拍在 `session.tick()` 之前调(§2.3 顺序)。对每条待生成项:
    /// 放票(首次)→ 出生列可读且碰撞体已建 → 生成 + 挂载;超时放弃。
    /// `alloc_id` = 驱动的服务端序号分配器(`spawn_next_seq` 那一套;
    /// 跳过现存 ID)。
    pub fn poll(
        &mut self,
        session: &mut Session,
        log: &mut CommandLog,
        host: &mut ControllerHost,
        alloc_id: &mut impl FnMut(&Session) -> EntityId,
        now: Tick,
    ) {
        if self.pending.is_empty() {
            return;
        }
        let mut done: Vec<usize> = Vec::new();
        let mut timed_out: Vec<usize> = Vec::new();
        for (i, item) in self.pending.iter_mut().enumerate() {
            // ① 首次见到:放区域票(常驻;①② 档 = 数据 + 碰撞体;票计费
            // 不用条目自己——生成完不移除,主体常驻其活动区)。
            if item.waited == 0 {
                let center = types::glam::Vec3::new(
                    item.cfg.position[0],
                    item.cfg.position[1],
                    item.cfg.position[2],
                );
                let radius = item.cfg.leash_m + STARTUP_TICKET_EXTRA_M;
                match session.set_area_ticket(item.ticket_id, center, radius, 0b011, None) {
                    Ok(entry) => log.append(entry),
                    // 上限拒(半径/数量):配置校验已挡(radius ≤ leash_max +
                    // extra = 48 < 默认上限 128);这里照任务书"不 panic",
                    // 计超时放弃。
                    Err(err) => {
                        eprintln!(
                            "[startup-agents] 第 {} 条区域票拒绝,放弃生成:{err:?}",
                            item.index
                        );
                        timed_out.push(i);
                        continue;
                    }
                }
            }
            // ② 出生点装载判定(与 A06 `flush_pending_respawns` 同缝:分片
            // 注册 + chunk 已载;再加 ② 碰撞体已建 + 出生列可读)。y 出世界
            // (网络生成同款校验,handle_spawn_agent ②)的列**永远不可读**,
            // 走超时放弃(E8 口径"不会被装载");越界生成会让主体掉虚空,
            // void 移除孤儿运动票触发内核 I3-2 fail-stop。
            let cell = BlockPos::new(
                item.cfg.position[0].floor() as i32,
                item.cfg.position[1].floor() as i32,
                item.cfg.position[2].floor() as i32,
            );
            let y_in_world = (types::WORLD_Y_MIN..=types::WORLD_Y_MAX).contains(&cell.y);
            let chunk = cell.chunk_of();
            let shard =
                types::ShardId::new(types::WorldId(0), types::DimensionId(0), chunk.shard_of());
            let loaded = y_in_world
                && session.shard_ids().contains(&shard)
                && session.is_chunk_loaded(shard, chunk)
                && session.has_collider(shard, chunk);
            let readable = loaded && session.block_solid_at(shard, cell).is_some();
            if readable {
                // 生成:服务端序号分配、SCHEDULER 盖章(内核换算
                // owner.source = None = 服务端主体)、日志条目进 CommandLog。
                let entity = alloc_id(session);
                let state = types::EntityState {
                    translation: types::glam::Vec3::new(
                        item.cfg.position[0],
                        item.cfg.position[1],
                        item.cfg.position[2],
                    ),
                    rotation: types::glam::Quat::from_rotation_y(item.cfg.yaw_deg.to_radians()),
                    ..types::EntityState::default()
                };
                match session.spawn_agent_from(
                    InputSource::SCHEDULER,
                    shard,
                    entity,
                    item.hash,
                    state,
                ) {
                    Ok(entry) => log.append(entry),
                    Err(err) => {
                        // 分片收件箱满等:留表下一拍重试(不 panic;票仍在)。
                        eprintln!(
                            "[startup-agents] 第 {} 条生成投递被拒,下一拍重试:{err:?}",
                            item.index
                        );
                        continue;
                    }
                }
                // 挂控制者(§2.2:leash 圆心 = position);attach 失败 =
                // 布景/配置错误(挂载契约:dof/period/重复),不静默。
                let kind = match item.cfg.controller {
                    StartupController::None => ControllerKind::None,
                    StartupController::QuadStand => ControllerKind::Script(ScriptKind::QuadStand),
                    StartupController::QuadTrot => ControllerKind::Script(ScriptKind::QuadTrot),
                };
                if let Err(err) = host.attach(
                    entity,
                    kind,
                    item.dof,
                    DEFAULT_CONTROL_PERIOD_TICKS,
                    now,
                    Some(Leash {
                        center_xz: [item.cfg.position[0], item.cfg.position[2]],
                        radius_m: item.cfg.leash_m,
                    }),
                ) {
                    panic!(
                        "[startup-agents] 第 {} 条控制者挂载失败(配置校验应已挡,fail-stop):{err:?}",
                        item.index
                    );
                }
                self.counters.spawned += 1;
                done.push(i);
            } else {
                // ③ 等装载;超时放弃(§2.2:写错误日志,不 panic)。
                item.waited += 1;
                if item.waited > STARTUP_SPAWN_TIMEOUT_TICKS {
                    eprintln!(
                        "[startup-agents] 第 {} 条({})等待装载超 {} 拍,放弃生成",
                        item.index, item.cfg.body, STARTUP_SPAWN_TIMEOUT_TICKS
                    );
                    timed_out.push(i);
                }
            }
        }
        // 出表(倒序 remove,保序)。
        for i in done.into_iter().rev() {
            self.pending.remove(i);
        }
        for i in timed_out.into_iter().rev() {
            self.counters.spawn_timeouts += 1;
            self.pending.remove(i);
        }
    }
}

#[cfg(test)]
mod tests {
    use sim_api::session::{Session, SessionConfig};
    use types::glam::Vec3;
    use types::{BlockId, ChunkPos, DimensionId, ShardId, ShardPos, WorldId};
    use voxel::{Chunk, SimpleRegistry};

    use super::*;
    use crate::controller::ControllerKind;

    const STONE: BlockId = BlockId(1);

    fn shard_id(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    fn quad_registry() -> physics_sim::BodyRegistry {
        let mut b = physics_sim::BodyRegistry::new();
        b.insert_toml(include_str!("../../../assets/bodies/quad.body.toml"))
            .expect("quad 合法");
        b
    }

    fn quad_cfg(position: [f32; 3], controller: StartupController) -> StartupAgentConfig {
        StartupAgentConfig {
            body: "quad".to_owned(),
            position,
            yaw_deg: 0.0,
            controller,
            leash_m: 16.0,
        }
    }

    fn cfg() -> SessionConfig {
        SessionConfig::default()
    }

    /// 平地一层石头(chunk (0,0) 层 world y = 0 全石,顶面 1.0),注册分片。
    fn load_ground(session: &mut Session) {
        session.register(physics_sim::ShardSim::new_with_bodies(
            physics_sim::ShardSimConfig::new(shard_id(0, 0)),
            Box::new(SimpleRegistry::builtin()),
            quad_registry(),
        ));
        let mut chunk = Chunk::air();
        for x in 0..voxel::CHUNK_X {
            for z in 0..voxel::CHUNK_Z {
                chunk.set(x, 64, z, STONE);
            }
        }
        session.apply_chunk_load(shard_id(0, 0), ChunkPos::new(0, 0), chunk);
    }

    fn alloc(session: &Session) -> EntityId {
        thread_local! {
            static SEQ: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
        }
        loop {
            let seq = SEQ.get();
            SEQ.set(seq + 1);
            let e = EntityId::from_parts(0, seq);
            if session.entity_owner(e).is_none() {
                return e;
            }
        }
    }

    /// S1:装载完成后生成并挂载——出生点 (8.5, 1.45, 8.5)(地面顶面 1.0 +
    /// 0.45)在已装载 chunk (0,0) 内;首拍 poll 放票(不可读,票当拍才生效),
    /// 下一拍装载判定过 → 生成;owner.source = None(SCHEDULER);host 挂
    /// QuadTrot,leash 圆心 = position;spawned = 1;日志恰 1 条 SetAreaTicket
    /// + 1 条 SpawnAgent;票据 ID 在保留区间。
    #[test]
    fn s1_spawn_when_loaded_and_attach() {
        let bodies = quad_registry();
        let mut startup = StartupAgents::new(
            &[quad_cfg([8.5, 1.45, 8.5], StartupController::QuadTrot)],
            &bodies,
        )
        .expect("构造");
        let mut session = Session::new(cfg());
        load_ground(&mut session);
        let mut log = CommandLog::new();
        let mut host = crate::controller::ControllerHost::default();

        // 第 1 拍前:票当拍生效(调度器票据收件箱),chunk 装载判定已过
        // (布景直载)→ 当拍即生成。
        startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(0));
        session.tick();
        assert_eq!(startup.counters.spawned, 1, "装载好的出生点当拍生成");
        assert!(startup.pending.is_empty(), "出表");
        session.shutdown();

        // 日志:SetAreaTicket 1(保留区间 ID)+ SpawnAgent 1(SCHEDULER)。
        let mut tickets = 0u64;
        let mut spawns = 0u64;
        let mut ticket_id = None;
        for e in log.entries() {
            match &e.command {
                messaging::SimCommand::SetAreaTicket { id, radius_m, .. } => {
                    tickets += 1;
                    ticket_id = Some(*id);
                    assert!(
                        (radius_m - (16.0 + STARTUP_TICKET_EXTRA_M)).abs() < 1e-4,
                        "票半径 = leash + extra,实得 {radius_m}"
                    );
                }
                messaging::SimCommand::SpawnAgent { entity, .. } => {
                    spawns += 1;
                    assert_eq!(*entity, EntityId::from_parts(0, 0), "服务端序号 0 起");
                    assert_eq!(e.source, InputSource::SCHEDULER, "SCHEDULER 盖章");
                }
                _ => {}
            }
        }
        assert_eq!(tickets, 1, "恰一张区域票");
        assert_eq!(spawns, 1, "恰一条 SpawnAgent");
        let id = ticket_id.expect("票 ID");
        assert!(
            id >= STARTUP_TICKET_BASE && id - STARTUP_TICKET_BASE < STARTUP_AGENTS_MAX as u64,
            "票据 ID 在保留区间 [{STARTUP_TICKET_BASE}, +{STARTUP_AGENTS_MAX}):{id}"
        );
    }

    /// S2:超时放弃——出生点在未注册分片的坐标上(分片没注册 = 列读不到),
    /// 走 STARTUP_SPAWN_TIMEOUT_TICKS 拍后 spawn_timeouts = 1、不 panic、
    /// 出表;票已放(第一次见到时)。
    #[test]
    fn s2_timeout_gives_up_without_panic() {
        let bodies = quad_registry();
        let mut startup = StartupAgents::new(
            &[quad_cfg([8.5, -59.5, 8.5], StartupController::None)],
            &bodies,
        )
        .expect("构造");
        let mut session = Session::new(cfg());
        // 不注册任何分片:shard_ids 空 → loaded false → 永等。
        let mut log = CommandLog::new();
        let mut host = crate::controller::ControllerHost::default();
        for now in 0..=STARTUP_SPAWN_TIMEOUT_TICKS as u64 {
            startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(now));
        }
        assert_eq!(startup.counters.spawn_timeouts, 1, "600 拍后超时放弃");
        assert_eq!(startup.counters.spawned, 0, "从未生成");
        assert!(startup.pending.is_empty(), "出表");
        assert!(host.first_slot().is_none(), "没有挂载");
        // 票在第一次见到时放了一条(第 0 拍)。
        let tickets = log
            .entries()
            .iter()
            .filter(|e| matches!(e.command, messaging::SimCommand::SetAreaTicket { .. }))
            .count();
        assert_eq!(tickets, 1, "首拍放票 1 张");
        session.shutdown();
    }

    /// S3:多条目——两条不同位置,同拍生成两条;EntityId 序号递增;每条
    /// 各一张票(ID 相邻,保留区间内);挂载两个槽位。
    #[test]
    fn s3_two_entries_spawn_and_attach() {
        let bodies = quad_registry();
        let cfgs = vec![
            quad_cfg([8.5, 1.45, 8.5], StartupController::QuadStand),
            quad_cfg([14.5, 1.45, 8.5], StartupController::None),
        ];
        let mut startup = StartupAgents::new(&cfgs, &bodies).expect("构造");
        let mut session = Session::new(cfg());
        load_ground(&mut session);
        let mut log = CommandLog::new();
        let mut host = crate::controller::ControllerHost::default();
        startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(0));
        session.tick();
        assert_eq!(startup.counters.spawned, 2);
        session.shutdown();
        let spawns = log
            .entries()
            .iter()
            .filter(|e| matches!(e.command, messaging::SimCommand::SpawnAgent { .. }))
            .count();
        assert_eq!(spawns, 2);
        // 两个槽位:QuadStand 与 None。
        let ids: Vec<EntityId> = {
            let recs = host.to_records();
            recs.iter().map(|r| r.entity).collect()
        };
        assert_eq!(ids.len(), 2);
        assert_eq!(
            host.kind_of(ids[0]),
            Some(ControllerKind::Script(
                crate::controller::ScriptKind::QuadStand
            ))
        );
        assert_eq!(host.kind_of(ids[1]), Some(ControllerKind::None));
        // leash 圆心 = 各自 position。
        let recs = host.to_records();
        assert_eq!(
            recs[0].slot.leash.as_ref().map(|l| l.center_xz),
            Some([8.5, 8.5])
        );
        // 票 ID 相邻且都在保留区间。
        let mut ticket_ids: Vec<u64> = log
            .entries()
            .iter()
            .filter_map(|e| match e.command {
                messaging::SimCommand::SetAreaTicket { id, .. } => Some(id),
                _ => None,
            })
            .collect();
        ticket_ids.sort_unstable();
        assert_eq!(
            ticket_ids,
            vec![STARTUP_TICKET_BASE, STARTUP_TICKET_BASE + 1],
            "票 ID 按条目序取保留区间相邻两值"
        );
        let _ = Vec3::ZERO;
    }

    /// S4:恢复启动传空表 → pending 空、poll 零动作(AC-A8 的 startup 侧)。
    #[test]
    fn s4_empty_config_noop() {
        let bodies = quad_registry();
        let mut startup = StartupAgents::new(&[], &bodies).expect("构造");
        let mut session = Session::new(cfg());
        let mut log = CommandLog::new();
        let mut host = crate::controller::ControllerHost::default();
        startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(0));
        assert_eq!(startup.counters, StartupCounters::default());
        assert!(log.entries().is_empty(), "空配置零日志条目");
        session.shutdown();
    }
}
