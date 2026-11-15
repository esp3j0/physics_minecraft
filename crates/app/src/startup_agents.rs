//! 启动主体(REQ-012 B02b §2.2):配置 `[[agents]]` 的待生成簿记与每拍
//! 轮询。全新启动经 [`StartupAgents::new`] 带全量配置;恢复启动
//! (B03d)经 [`StartupAgents::resume`] 只为检查点 `StartupRecord.pending`
//! 列出的序号建待生成项——世界快照里已生成的主体不再重复生成(E2)。
//!
//! 每拍在 `session.tick()` 之前(§2.3 顺序第 ① 步)对每条待生成项:
//! 1. 第一次见到时放一张区域票据(中心 = position,半径 =
//!    `leash_m + STARTUP_TICKET_EXTRA_M`,常驻,①②③ 档——B03c 3.1 起
//!    唤醒档照常生效:配置生成的主体无人观察时不被冻结)——出生点区块靠它
//!    装载(E8 的"不会被装载"布景就是不放观察者、只靠这张票也够不着);
//! 2. 出生列方块可读且该处碰撞体已建好 → 生成(InputSource::SCHEDULER,
//!    条目进日志;复用 A06 重生等装载的同一组判定缝:
//!    `shard_ids().contains && is_chunk_loaded && has_collider &&
//!    block_solid_at` 可读),挂控制者(host.attach,leash 圆心 =
//!    position),从 pending 移除,`spawned += 1`,票进票表;
//! 3. 等待超过 [`STARTUP_SPAWN_TIMEOUT_TICKS`] → 放弃这一条,
//!    `spawn_timeouts += 1`,写一条错误日志(eprintln),不 panic。
//!
//! 已生成主体的票进票表(B03d):每拍末驱动调 [`StartupAgents::release_dead`]
//! (紧跟 `host.retain_alive` 之后,同一个存活判断)——主体没了就撤票,
//! 那片区域不再常驻装载/唤醒。持久状态经 [`StartupAgents::to_record`]
//! 随检查点走(v15)。
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
    /// 已放票(R1-4:放票只做一次——重试拍不重复放票、不重复写
    /// SetAreaTicket 日志)。
    ticket_placed: bool,
    /// 已等待拍数(生成投递被拒的重试拍也计入——重试不是免费等待)。
    waited: u32,
}

/// 启动主体的持久状态(随检查点走,B03d v15)。
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StartupRecord {
    /// 还没生成也没放弃的配置条目序号(0 起,对应 config.agents 的下标)。
    pub pending: Vec<u32>,
    /// 已生成主体与它的启动区域票:(主体, 票号)。
    pub tickets: Vec<(EntityId, u64)>,
}

/// 启动主体计数(遥测;DriverOutcome.startup_counters)。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StartupCounters {
    /// 已生成(含挂控制者)。
    pub spawned: u64,
    /// 等装载超时放弃的条数。
    pub spawn_timeouts: u64,
    /// 主体没了之后撤掉的启动票数(B03d)。
    pub tickets_released: u64,
    /// 恢复时丢弃的待生成项(序号越界或身体不在注册表;B03d)。
    pub resume_dropped: u64,
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
    /// 已生成主体与它的启动区域票(主体没了就撤票;B03d)。
    tickets: Vec<(EntityId, u64)>,
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
                ticket_placed: false,
                waited: 0,
            });
        }
        Ok(Self {
            pending,
            tickets: Vec::new(),
            counters: StartupCounters::default(),
        })
    }

    /// 恢复启动(B03d):只为 `record.pending` 列出的序号建待生成项。
    /// 序号 ≥ cfgs.len(),或该条的身体不在注册表 → 丢弃
    /// (`resume_dropped += 1`,打一行日志,不报错);票号仍是
    /// `STARTUP_TICKET_BASE + 序号`,`ticket_placed = false`(重放一次票,
    /// 同号覆盖即续期);`waited = 0`;`tickets = record.tickets`。
    pub fn resume(
        cfgs: &[StartupAgentConfig],
        bodies: &physics_sim::BodyRegistry,
        record: &StartupRecord,
    ) -> Self {
        let mut pending = Vec::new();
        let mut dropped = 0u64;
        for &i in &record.pending {
            let Some(cfg) = cfgs.get(i as usize) else {
                eprintln!(
                    "[startup-agents] 恢复:待生成序号 {i} 越界(配置 {} 条),丢弃",
                    cfgs.len()
                );
                dropped += 1;
                continue;
            };
            let Some(hash) = bodies.hash_of(&cfg.body) else {
                eprintln!(
                    "[startup-agents] 恢复:第 {} 条身体 \"{}\" 不在注册表里,丢弃",
                    i + 1,
                    cfg.body
                );
                dropped += 1;
                continue;
            };
            let dof = bodies.get(&hash).map(|def| def.dof_count()).unwrap_or(0);
            pending.push(PendingStartup {
                index: i as usize + 1,
                cfg: cfg.clone(),
                hash,
                dof,
                // 票号 = BASE + 0 起下标(与 new 同式;PendingStartup.index
                // 是 1 起,这里用 record 的 0 起下标)。
                ticket_id: STARTUP_TICKET_BASE + i as u64,
                ticket_placed: false,
                waited: 0,
            });
        }
        Self {
            pending,
            tickets: record.tickets.clone(),
            counters: StartupCounters {
                resume_dropped: dropped,
                ..StartupCounters::default()
            },
        }
    }

    /// 当前持久状态(写检查点时取;pending 按序号升序——保持条目表顺序,
    /// 生成/放弃只整条出表,顺序天然稳定)。
    pub fn to_record(&self) -> StartupRecord {
        StartupRecord {
            // PendingStartup.index 是 1 起,record 要 0 起下标。
            pending: self.pending.iter().map(|p| (p.index - 1) as u32).collect(),
            tickets: self.tickets.clone(),
        }
    }

    /// 主体没了就撤它的启动票(B03d;每拍末紧跟 `host.retain_alive` 之后)。
    /// `alive` 与 `retain_alive` 用同一个判断(`is_agent` || 迁移在途)。
    /// 每撤一张:日志追加 RemoveAreaTicket,`tickets_released += 1`,条目
    /// 出表;撤票被拒 → 打日志,条目保留,下一拍重试。没有票条目时零动作
    /// (不调 Session)。
    pub fn release_dead(
        &mut self,
        session: &mut Session,
        log: &mut CommandLog,
        alive: impl Fn(EntityId) -> bool,
    ) {
        if self.tickets.is_empty() {
            return;
        }
        // 一次性出表(retain):撤成功的条目出表,被拒的保留到下一拍。
        let mut released = 0u64;
        self.tickets.retain(|&(entity, ticket_id)| {
            if alive(entity) {
                return true;
            }
            match session.remove_area_ticket(ticket_id) {
                Ok(entry) => {
                    log.append(entry);
                    released += 1;
                    false
                }
                Err(err) => {
                    eprintln!(
                        "[startup-agents] 主体 {entity:?} 没了但撤票 {ticket_id} 被拒,下一拍重试:{err:?}"
                    );
                    true
                }
            }
        });
        self.counters.tickets_released += released;
    }

    /// 预置已等待拍数(仅测试:构造"同拍一条生成、另一条到点放弃"的布景)。
    #[cfg(test)]
    fn set_waited(&mut self, index: usize, waited: u32) {
        self.pending[index - 1].waited = waited;
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
        // 一拍内每条恰好一个结果;循环结束后按结果**一次性**出表(R1-2:
        // 生成表/放弃表分开倒序 remove 时,第二轮下标相对原表已错位——
        // 同拍既有生成又有放弃会删错条目或越界 panic)。
        enum Outcome {
            /// 继续等(票已放但还没装载好,或生成投递被拒下一拍重试)。
            Keep,
            /// 本拍已生成 + 挂载。
            Spawned,
            /// 本拍放弃(超时 / 放票被拒)。
            Abandoned,
        }
        let mut outcomes: Vec<Outcome> = Vec::with_capacity(self.pending.len());
        let mut ticket_ids_to_remove: Vec<u64> = Vec::new();
        for item in self.pending.iter_mut() {
            // ① 首次见到:放区域票(常驻;①②③ 档 = 数据 + 碰撞体 + 唤醒;
            // B03c §3 规则 2/3.1:配置生成的主体的启动区域票升到 ③ 档,
            // 无人观察时照常活动、不被冻结)。票计费不用条目自己——生成完
            // 不移除,主体常驻其活动区。R1-4:只放一次,重试拍不再重复放票。
            if !item.ticket_placed {
                let center = types::glam::Vec3::new(
                    item.cfg.position[0],
                    item.cfg.position[1],
                    item.cfg.position[2],
                );
                let radius = item.cfg.leash_m + STARTUP_TICKET_EXTRA_M;
                match session.set_area_ticket(item.ticket_id, center, radius, 0b111, None) {
                    Ok(entry) => {
                        log.append(entry);
                        item.ticket_placed = true;
                    }
                    // 上限拒(半径/数量):配置校验已挡(radius ≤ leash_max +
                    // extra = 48 < 默认上限 128);这里照任务书"不 panic",
                    // 计超时放弃(票没放上,无需撤)。
                    Err(err) => {
                        eprintln!(
                            "[startup-agents] 第 {} 条区域票拒绝,放弃生成:{err:?}",
                            item.index
                        );
                        outcomes.push(Outcome::Abandoned);
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
                        // R1-4:重试拍计入等待(否则重试不计超时,可无限重试);
                        // 超限同样放弃(撤票)。
                        item.waited += 1;
                        if item.waited > STARTUP_SPAWN_TIMEOUT_TICKS {
                            eprintln!(
                                "[startup-agents] 第 {} 条生成投递持续被拒超 {} 拍,放弃生成",
                                item.index, STARTUP_SPAWN_TIMEOUT_TICKS
                            );
                            if item.ticket_placed {
                                ticket_ids_to_remove.push(item.ticket_id);
                            }
                            outcomes.push(Outcome::Abandoned);
                        } else {
                            outcomes.push(Outcome::Keep);
                        }
                        eprintln!(
                            "[startup-agents] 第 {} 条生成投递被拒,下一拍重试:{err:?}",
                            item.index
                        );
                        continue;
                    }
                }
                // 挂控制者(§2.2:leash 圆心 = position);attach 失败 =
                // 布景/配置错误(挂载契约:dof/period/重复),不静默。
                // attached_at = now.next()(B03a X1:spawn 命令 apply_at =
                // now,拍末 tick 应用、owner 拍末 claim;attached_at = now 时
                // 同拍 drive 的 n = 0 会立即发目标,owner 还没 claim →
                // send_rejected 1 次)。周期 = DEFAULT_CONTROL_PERIOD_TICKS
                // = 2,第一次发目标在 n = 0 即 attached_at = now+1;
                // §2.2 的 30 Hz 周期不变。
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
                    now.next(),
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
                // 票跟主体走(B03d):进票表,主体没了由 release_dead 撤。
                // 放弃的两支(超时/放票被拒)不进——出表时已撤票。
                self.tickets.push((entity, item.ticket_id));
                outcomes.push(Outcome::Spawned);
            } else {
                // ③ 等装载;超时放弃(§2.2:写错误日志,不 panic)。
                item.waited += 1;
                if item.waited > STARTUP_SPAWN_TIMEOUT_TICKS {
                    eprintln!(
                        "[startup-agents] 第 {} 条({})等待装载超 {} 拍,放弃生成",
                        item.index, item.cfg.body, STARTUP_SPAWN_TIMEOUT_TICKS
                    );
                    // R1-4:放弃时撤掉已放的区域票(否则那片区域永远装载)。
                    if item.ticket_placed {
                        ticket_ids_to_remove.push(item.ticket_id);
                    }
                    outcomes.push(Outcome::Abandoned);
                } else {
                    outcomes.push(Outcome::Keep);
                }
            }
        }
        // 撤票(放弃条;R1-4):条目进日志(回放同款再生)。撤票失败不放弃
        // 撤票语义(票可能已被外部同名 ID 覆盖——保留区间约定下不可能,
        // 防御性忽略),照旧出表。
        for id in ticket_ids_to_remove {
            match session.remove_area_ticket(id) {
                Ok(entry) => log.append(entry),
                Err(err) => eprintln!("[startup-agents] 撤区域票 {id} 失败:{err:?}"),
            }
        }
        // 出表(一次性;保留 Keep,放弃计数按结果加)。
        self.counters.spawn_timeouts += outcomes
            .iter()
            .filter(|o| matches!(o, Outcome::Abandoned))
            .count() as u64;
        let mut it = outcomes.into_iter();
        self.pending
            .retain(|_| matches!(it.next(), Some(Outcome::Keep)));
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
        // S9(B03c 3.3):启动票档位 = 0b111(③ 唤醒档,B03c 3.1)——
        // shutdown 前从票据表读 wake 覆盖核验。
        {
            let id = log
                .entries()
                .iter()
                .find_map(|e| match &e.command {
                    messaging::SimCommand::SetAreaTicket { id, .. } => Some(*id),
                    _ => None,
                })
                .expect("布景自检:日志含 SetAreaTicket");
            let tier_wake = session
                .ticket_table()
                .get((
                    physics_sim::tickets::TicketHolder::Area(id),
                    physics_sim::tickets::TicketKind::Area,
                ))
                .expect("票已在表")
                .wake
                .clone();
            assert!(
                matches!(
                    tier_wake,
                    physics_sim::tickets::WakeSpec::Radius { load_m, unload_m }
                        if (load_m - (16.0 + STARTUP_TICKET_EXTRA_M)).abs() < 1e-4
                            && (unload_m - (16.0 + STARTUP_TICKET_EXTRA_M)).abs() < 1e-4
                ),
                "S9:启动票的 ③ 唤醒档 = Radius(leash + extra),实得 {tier_wake:?}"
            );
        }
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
    /// 出表;票已放(第一次见到时,全程恰 1 条——重试不重复放票,R1-4);
    /// 放弃后撤票(RemoveAreaTicket 恰 1 条、同一 ID,R1-4)。
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
        // 第 599 拍(第 600 次 poll)后:还没超时——阈值 600 绝对锚定,
        // 不引用常量(变异把阈值改小时断言不跟着走,R1-3)。第 600 拍整
        // 不放弃(waited == 600 不 > 600),第 601 拍(waited 601)才超。
        for now in 0..600u64 {
            startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(now));
        }
        assert_eq!(
            startup.counters.spawn_timeouts, 0,
            "第 600 次 poll 后未超时"
        );
        assert_eq!(startup.pending.len(), 1, "仍在等");
        // 第 601 次 poll:waited 超限 → 放弃。
        startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(600));
        assert_eq!(startup.counters.spawn_timeouts, 1, "第 601 次 poll 后超时");
        assert_eq!(startup.counters.spawned, 0, "从未生成");
        assert!(startup.pending.is_empty(), "出表");
        assert!(host.first_slot().is_none(), "没有挂载");
        // 票在第一次见到时放了恰 1 条(600 拍等待期间不重复放票,R1-4)。
        let mut ticket_ids: Vec<u64> = Vec::new();
        for e in log.entries() {
            if let messaging::SimCommand::SetAreaTicket { id, .. } = e.command {
                ticket_ids.push(id);
            }
        }
        assert_eq!(ticket_ids, vec![STARTUP_TICKET_BASE], "首拍放票 1 张");
        // 放弃后撤票:RemoveAreaTicket 恰 1 条、同一 ID(R1-4)。
        let removed: Vec<u64> = log
            .entries()
            .iter()
            .filter_map(|e| match e.command {
                messaging::SimCommand::RemoveAreaTicket { id } => Some(id),
                _ => None,
            })
            .collect();
        assert_eq!(removed, vec![STARTUP_TICKET_BASE], "放弃后撤同一张票");
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

    /// S5(返修 R1-2):同一拍里一条生成、另一条放弃——出表只做一次,
    /// 不因两张下标表错位而删错条目/越界 panic。
    /// 布景:生成条在已装载分片(当拍可读 → 生成);放弃条在出世界列
    /// (y 远低于世界底,列恒不可读,E8 同款),`set_waited`(#[cfg(test)])
    /// 预置到超时临界 → 本拍 waited+1 超限放弃。旧实现两张下标表都指向
    /// 原 pending:顺序 A(先生成 i=0、后放弃 i=1)在 remove(0) 后
    /// remove(1) 越界 panic;顺序 B 恰好被倒序救住——修法(retain 一遍)
    /// 不依赖顺序,两个都跑。
    #[test]
    fn s5_spawn_and_timeout_same_tick_retains_consistent() {
        let spawned_at = [8.5f32, 1.45, 8.5];
        let timeout_at = [8.5f32, -200.0, 8.5];
        for order in 0..2 {
            let cfgs = if order == 0 {
                vec![
                    quad_cfg(spawned_at, StartupController::None),
                    quad_cfg(timeout_at, StartupController::None),
                ]
            } else {
                vec![
                    quad_cfg(timeout_at, StartupController::None),
                    quad_cfg(spawned_at, StartupController::None),
                ]
            };
            let bodies = quad_registry();
            let mut startup = StartupAgents::new(&cfgs, &bodies).expect("构造");
            // 放弃条(顺序 0 = 第 2 条、顺序 1 = 第 1 条)预置到超时临界:
            // 本拍 waited+1 即超限(600 拍等不完)。
            let timeout_index = if order == 0 { 2 } else { 1 };
            startup.set_waited(timeout_index, STARTUP_SPAWN_TIMEOUT_TICKS);
            let mut session = Session::new(cfg());
            load_ground(&mut session);
            let mut log = CommandLog::new();
            let mut host = crate::controller::ControllerHost::default();
            startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(0));
            session.tick();
            assert_eq!(startup.counters.spawned, 1, "顺序 {order}:当拍生成 1 条");
            assert_eq!(
                startup.counters.spawn_timeouts, 1,
                "顺序 {order}:当拍放弃 1 条"
            );
            assert!(
                startup.pending.is_empty(),
                "顺序 {order}:两笔同拍出表,表空(残留 {:?})",
                startup.pending.iter().map(|p| p.index).collect::<Vec<_>>()
            );
            session.shutdown();
            // 生成的恰是装好那条:日志 SpawnAgent 的位置 = spawned_at。
            let spawn_pos = log.entries().iter().find_map(|e| match e.command {
                messaging::SimCommand::SpawnAgent { state, .. } => Some(state.translation),
                _ => None,
            });
            assert_eq!(
                spawn_pos,
                Some(Vec3::new(spawned_at[0], spawned_at[1], spawned_at[2])),
                "顺序 {order}:生成的是装好那条(位置核对)"
            );
        }
    }

    /// S7(返修 R1-3):`yaw_deg` 进入生成状态——`yaw_deg = 90` 的条目
    /// 生成后,SpawnAgent 日志条目的 rotation 逐分量等于
    /// `Quat::from_rotation_y(90°)`(容差 1e-6)。生成时忽略 yaw 的变异
    /// (rotation 恒 identity)在此红。
    #[test]
    fn s7_yaw_deg_goes_into_spawn_rotation() {
        let bodies = quad_registry();
        let mut agent_cfg = quad_cfg([8.5, 1.45, 8.5], StartupController::None);
        agent_cfg.yaw_deg = 90.0;
        let mut startup = StartupAgents::new(&[agent_cfg], &bodies).expect("构造");
        let mut session = Session::new(cfg());
        load_ground(&mut session);
        let mut log = CommandLog::new();
        let mut host = crate::controller::ControllerHost::default();
        startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(0));
        session.tick();
        assert_eq!(startup.counters.spawned, 1);
        session.shutdown();
        let rot = log
            .entries()
            .iter()
            .find_map(|e| match e.command {
                messaging::SimCommand::SpawnAgent { state, .. } => Some(state.rotation),
                _ => None,
            })
            .expect("恰一条 SpawnAgent(S1 同布景)");
        let want = types::glam::Quat::from_rotation_y(90f32.to_radians());
        assert!(
            (rot.x - want.x).abs() < 1e-6
                && (rot.y - want.y).abs() < 1e-6
                && (rot.z - want.z).abs() < 1e-6
                && (rot.w - want.w).abs() < 1e-6,
            "rotation = from_rotation_y(90°)(实得 {rot:?},want {want:?})"
        );
    }

    /// S8(返修 R1-3,变异 7):碰撞体未建不生成——放一张布景区域票让激活
    /// 启用(`tickets_ever_set`)后 `apply_chunk_load` 只装 ①(体素),
    /// `has_collider == false`:poll 判定不过、不生成;激活决策(单独构建档)
    /// 建好 ② 后的 poll 才生成。poll 里去掉 `has_collider` 判定的变异在此红
    /// (首拍就生成)。
    ///
    /// 布景细节:startup 条目的票半径 = leash + extra = 18(最小 leash 2),
    /// 锚取 (10.5,10.5) 使覆盖 = {(0,0),(1,0),(0,1)}(CHUNK_SIZE 16:
    /// 三个中心距 3.5/13.7/13.7 ≤ 18,(-1,0)/(0,-1) 18.7 > 18 出圈,且
    /// 覆盖集全在分片 (0,0)——不跨 owner)。覆盖内未载邻居会进 pending
    /// 每拍占满 ② 预算(2)、单独构建档永不上桌——所以把 (1,0)/(0,1)
    /// 手工预载(只 ①):三 chunk 全走"单独构建"候选,ChunkPos 升序
    /// 2 个/拍,(0,0) 第 1 位首拍建好。
    #[test]
    fn s8_no_spawn_until_collider_built() {
        let bodies = quad_registry();
        let mut agent_cfg = quad_cfg([10.5, 1.45, 10.5], StartupController::None);
        agent_cfg.leash_m = crate::config::LEASH_MIN_M; // 票半径 = 2 + 16 = 18
        let mut startup = StartupAgents::new(&[agent_cfg], &bodies).expect("构造");
        let mut session = Session::new(cfg());
        // 布景票(低位 ID,与保留区间无关;半径 8 只覆盖 chunk (0,0))→
        // 激活启用;tick 兑现(submit_observer 的票在下一次 tick 生效)。
        session
            .set_area_ticket(7, Vec3::new(10.5, 1.45, 10.5), 8.0, 0b011, None)
            .expect("布景票");
        session.tick();
        load_ground(&mut session);
        // startup 票(18)覆盖的另外两个 chunk 手工预载(激活启用 → 只 ①),
        // 平地一层石头,同 load_ground 口径;都在分片 (0,0) 本地。
        for (cx, cz) in [(1i32, 0i32), (0, 1)] {
            let mut chunk = voxel::Chunk::air();
            for x in 0..voxel::CHUNK_X {
                for z in 0..voxel::CHUNK_Z {
                    chunk.set(x, 64, z, STONE);
                }
            }
            session.apply_chunk_load(shard_id(0, 0), types::ChunkPos::new(cx, cz), chunk);
        }
        let shard = shard_id(0, 0);
        assert!(
            session.is_chunk_loaded(shard, types::ChunkPos::new(0, 0)),
            "布景:chunk 已装载(①)"
        );
        assert!(
            !session.has_collider(shard, types::ChunkPos::new(0, 0)),
            "布景:激活启用 → ② 未建"
        );
        let mut log = CommandLog::new();
        let mut host = crate::controller::ControllerHost::default();
        // 第 0 拍 poll:startup 自己的票照放;出生列可读但 ② 未建 → 不生成。
        startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(0));
        assert_eq!(
            startup.counters.spawned, 0,
            "碰撞体未建不生成(实得生成 {})",
            startup.counters.spawned
        );
        // 一拍:激活决策(单独构建档)建好 ② → 下一次 poll 生成。
        session.tick();
        assert!(
            session.has_collider(shard, types::ChunkPos::new(0, 0)),
            "tick 后激活决策已建 ②"
        );
        startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(1));
        session.tick();
        assert_eq!(startup.counters.spawned, 1, "碰撞体建好后生成");
        assert!(startup.pending.is_empty(), "出表");
        session.shutdown();
    }

    // ---- B03d:S10–S13 ------------------------------------------------------

    /// S10 往返:三条配置——第 1 条已生成(装载好的位置)、第 2 条在等
    /// (出世界列,永不装载)、第 3 条已放弃(同款出世界列,预置超时临界)。
    /// `to_record()`:`pending == [1]`(0 起下标;第 2 条在等、第 3 条已
    /// 放弃出表)、`tickets == [(第 1 条的主体, BASE + 0)]`;`resume` 之后
    /// 待生成项恰一条(第 2 条),序号、票号(BASE + 1)对。
    #[test]
    fn s10_record_roundtrip_pending_and_tickets() {
        let bodies = quad_registry();
        let cfgs = vec![
            quad_cfg([8.5, 1.45, 8.5], StartupController::None), // 生成
            quad_cfg([8.5, -200.0, 8.5], StartupController::None), // 在等
            quad_cfg([14.5, -200.0, 8.5], StartupController::None), // 放弃
        ];
        let mut startup = StartupAgents::new(&cfgs, &bodies).expect("构造");
        let mut session = Session::new(cfg());
        load_ground(&mut session);
        let mut log = CommandLog::new();
        let mut host = crate::controller::ControllerHost::default();
        // 第 1 条当拍生成;第 3 条预置超时临界同拍放弃;第 2 条 waited=0 在等。
        startup.set_waited(3, STARTUP_SPAWN_TIMEOUT_TICKS);
        startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(0));
        session.tick();
        assert_eq!(startup.counters.spawned, 1);
        assert_eq!(startup.counters.spawn_timeouts, 1);
        assert_eq!(startup.pending.len(), 1, "第 2 条仍在等");
        session.shutdown();

        let spawned_entity = match log.entries()[0].command {
            _ => log
                .entries()
                .iter()
                .find_map(|e| match e.command {
                    messaging::SimCommand::SpawnAgent { entity, .. } => Some(entity),
                    _ => None,
                })
                .expect("布景自检:第 1 条已生成"),
        };
        let record = startup.to_record();
        assert_eq!(record.pending, vec![1], "在等的是 0 起下标 1(第 2 条)");
        assert_eq!(
            record.tickets,
            vec![(spawned_entity, STARTUP_TICKET_BASE)],
            "已生成主体与它的票(BASE + 0)"
        );

        // resume:只第 2 条进待生成表;序号(1 起 = 2)、票号(BASE + 1)。
        let mut resumed = StartupAgents::resume(&cfgs, &bodies, &record);
        assert_eq!(resumed.counters.resume_dropped, 0, "两条都合法");
        assert_eq!(resumed.pending.len(), 1);
        // PendingStartup 私有;经 to_record 往返核对序号,经 poll 首拍放票
        // 核对票号(S13 同款)。
        assert_eq!(resumed.to_record().pending, vec![1]);
        assert_eq!(resumed.to_record().tickets, record.tickets, "票表原样带回");
    }

    /// S11 恢复时丢弃:`pending = [1, 7]` 但配置只有 2 条 → 留 1 条
    /// (序号 1),`resume_dropped == 1`;身体不在注册表的那种情况同样丢弃
    /// (构造一个注册表只有 quad、pending 指向会用到未知身体的序号——
    /// 直接把 record.pending 指到合法序号但换空注册表)。
    #[test]
    fn s11_resume_drops_out_of_range_and_unknown_body() {
        let bodies = quad_registry();
        let cfgs = vec![
            quad_cfg([8.5, 1.45, 8.5], StartupController::None),
            quad_cfg([14.5, 1.45, 8.5], StartupController::None),
        ];
        // 序号越界:7 ≥ 2 条。
        let record = StartupRecord {
            pending: vec![1, 7],
            tickets: Vec::new(),
        };
        let mut resumed = StartupAgents::resume(&cfgs, &bodies, &record);
        assert_eq!(resumed.counters.resume_dropped, 1, "越界 1 条丢弃");
        assert_eq!(resumed.to_record().pending, vec![1], "留序号 1");
        // 身体不在注册表:空注册表,任何序号都查不到身体。
        let empty_bodies = physics_sim::BodyRegistry::new();
        let record2 = StartupRecord {
            pending: vec![0],
            tickets: Vec::new(),
        };
        let resumed2 = StartupAgents::resume(&cfgs, &empty_bodies, &record2);
        assert_eq!(resumed2.counters.resume_dropped, 1, "身体未知 1 条丢弃");
        assert!(resumed2.pending.is_empty(), "全丢弃");
        assert!(resumed2.to_record().pending.is_empty());
    }

    /// S12 撤票:先造出一张已生成主体的票(装载好的位置当拍生成),然后
    /// 按三种存活判定调 release_dead——
    /// - 主体活着(alive = true)→ 不撤(票表不动、零日志);
    /// - 迁移在途(alive 闭包对在途 ID 返回 true 同口径)→ 不撤;
    /// - 主体没了(alive = false)→ 恰撤一次:日志一条 RemoveAreaTicket
    ///   (同一票号)、票表没有这张票、`tickets_released == 1`;再调一次
    ///   零动作(表已空,连 Session 都不碰——表空早退)。
    #[test]
    fn s12_release_dead_releases_only_dead() {
        let bodies = quad_registry();
        let mut startup = StartupAgents::new(
            &[quad_cfg([8.5, 1.45, 8.5], StartupController::None)],
            &bodies,
        )
        .expect("构造");
        let mut session = Session::new(cfg());
        load_ground(&mut session);
        let mut log = CommandLog::new();
        let mut host = crate::controller::ControllerHost::default();
        startup.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(0));
        session.tick();
        assert_eq!(startup.counters.spawned, 1);
        let (entity, ticket_id) = startup.to_record().tickets[0];

        // 活着 → 不撤。
        let n_before = log.entries().len();
        startup.release_dead(&mut session, &mut log, |_| true);
        assert_eq!(
            startup.to_record().tickets,
            vec![(entity, ticket_id)],
            "活着不撤"
        );
        assert_eq!(
            log.entries().len(),
            n_before,
            "活着:零日志条目(不调 Session)"
        );
        assert_eq!(startup.counters.tickets_released, 0);
        // 在途(alive 对该 ID 返回 true,与 retain_alive 的 is_agent || 在途
        // 同口径)→ 不撤。
        let in_transit = std::collections::BTreeSet::from([entity]);
        startup.release_dead(&mut session, &mut log, |id| in_transit.contains(&id));
        assert_eq!(
            startup.to_record().tickets,
            vec![(entity, ticket_id)],
            "在途不撤"
        );
        assert_eq!(startup.counters.tickets_released, 0);
        // 没了 → 恰撤一次。
        startup.release_dead(&mut session, &mut log, |_| false);
        assert!(startup.to_record().tickets.is_empty(), "死了:条目出表");
        assert_eq!(startup.counters.tickets_released, 1);
        let removed: Vec<u64> = log
            .entries()
            .iter()
            .filter_map(|e| match e.command {
                messaging::SimCommand::RemoveAreaTicket { id } => Some(id),
                _ => None,
            })
            .collect();
        assert_eq!(
            removed,
            vec![ticket_id],
            "日志恰一条 RemoveAreaTicket(同一票号)"
        );
        // 再调一次零动作(表空早退,不产生新日志)。
        let n_after = log.entries().len();
        startup.release_dead(&mut session, &mut log, |_| false);
        assert_eq!(log.entries().len(), n_after, "表空再调零动作");
        assert_eq!(startup.counters.tickets_released, 1);
        session.shutdown();
    }

    /// S13 恢复后重放票:`resume` 出来的待生成项第一拍放票(日志里一条
    /// SetAreaTicket,票号与恢复前相同——同号覆盖即续期),装载好后生成
    /// 并挂控制者(QuadTrot → 槽位在)。
    #[test]
    fn s13_resume_replays_ticket_then_spawns() {
        let bodies = quad_registry();
        let cfgs = vec![quad_cfg([8.5, 1.45, 8.5], StartupController::QuadTrot)];
        // 恢复前:只放票不生成(布景无地面,poll 只放票、等待)。
        let mut first = StartupAgents::new(&cfgs, &bodies).expect("构造");
        let mut session = Session::new(cfg());
        let mut log = CommandLog::new();
        let mut host = crate::controller::ControllerHost::default();
        first.poll(&mut session, &mut log, &mut host, &mut alloc, Tick(0));
        session.tick();
        assert_eq!(first.counters.spawned, 0, "布景:没装载不生成");
        let ticket_before = first.to_record().tickets.first().map(|t| t.1);
        assert_eq!(ticket_before, None, "未生成:票表空;票在待生成项上");
        session.shutdown();
        let placed: Vec<u64> = log
            .entries()
            .iter()
            .filter_map(|e| match e.command {
                messaging::SimCommand::SetAreaTicket { id, .. } => Some(id),
                _ => None,
            })
            .collect();
        assert_eq!(
            placed,
            vec![STARTUP_TICKET_BASE],
            "布景自检:第一段放票 1 张"
        );

        // 恢复:pending = [0](0 起下标),票号仍 BASE + 0。
        let record = first.to_record();
        let mut resumed = StartupAgents::resume(&cfgs, &bodies, &record);
        // 换有地面的会话:恢复后第一拍放票(重放,同号覆盖),下一拍生成。
        let mut session2 = Session::new(cfg());
        load_ground(&mut session2);
        let mut log2 = CommandLog::new();
        let mut host2 = crate::controller::ControllerHost::default();
        resumed.poll(&mut session2, &mut log2, &mut host2, &mut alloc, Tick(0));
        session2.tick();
        assert_eq!(
            resumed.counters.spawned, 1,
            "布景直载(chunk 已在)首拍即生成,S1 同口径"
        );
        assert!(resumed.pending.is_empty(), "出表");
        let replayed: Vec<u64> = log2
            .entries()
            .iter()
            .filter_map(|e| match e.command {
                messaging::SimCommand::SetAreaTicket { id, .. } => Some(id),
                _ => None,
            })
            .collect();
        assert_eq!(
            replayed,
            vec![STARTUP_TICKET_BASE],
            "恢复后第一拍重放票:同号(实得 {replayed:?})"
        );
        // 挂控制者:QuadTrot 槽位在。
        let recs = host2.to_records();
        assert_eq!(recs.len(), 1, "挂了控制者");
        assert_eq!(
            host2.kind_of(recs[0].entity),
            Some(ControllerKind::Script(
                crate::controller::ScriptKind::QuadTrot
            ))
        );
        assert_eq!(resumed.to_record().tickets.len(), 1, "生成后票进票表");
        session2.shutdown();
    }
}
