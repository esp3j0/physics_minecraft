//! Session:命令进/快照出的 headless 缝(接口形状 = 网络形状)。

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{ShardSim, SimSnapshotError};
use shard::router::RouteError;
use shard::{Scheduler, SchedulerConfig, TickSummary};
use types::glam::Vec3;
use types::{BlockPos, DimensionId, EntityId, EntityState, ShardId, Tick, WorldId};
use voxel::BlockRegistry;

use crate::log::LogEntry;
use crate::snapshot::{GlobalSnapshot, WorldSnapshot};

/// 会话配置(接口设计,签名固定):调度器配置 + 会话作为输入源。
#[derive(Debug, Clone)]
pub struct SessionConfig {
    pub scheduler: SchedulerConfig,
    /// 会话作为输入源(默认 `InputSource(1)`;0 保留给调度器,README 决策 5)。
    pub source: InputSource,
}

impl Default for SessionConfig {
    /// scheduler 默认 + `InputSource(1)`(决策 5:0 保留给调度器)。
    fn default() -> Self {
        Self {
            scheduler: SchedulerConfig::default(),
            source: InputSource(1),
        }
    }
}

/// 会话层可预期失败:路由(收件箱满)与快照恢复(残缺字节)透传;
/// `Rejected` = 广播路径的校验拒绝(owner 不匹配 / 不支持的广播命令 /
/// 目标未注册;hostile 路径由调用方丢弃,不 panic)。
/// 其余(未注册分片、迟到输入、原始直投未登记)沿用调度器语义 → panic(fail-stop)。
#[derive(Debug, thiserror::Error)]
pub enum SessionError {
    #[error("路由失败:{0}")]
    Route(#[from] RouteError),
    #[error("快照恢复失败:{0}")]
    Snapshot(#[from] SimSnapshotError),
    #[error("命令被拒绝:{0}")]
    Rejected(&'static str),
}

/// headless 会话:命令进/快照出(蓝图 §3 session;网络形状后置 REQ-003)。
///
/// 盖章规则(README 决策 5):`send` 盖 `apply_at = tick_now()`(下一次 `tick()`
/// 即生效),`source` = 配置值(默认 1),`sequence` 每 session 单调从 0 起;
/// `send` 返回 [`LogEntry`] 供调用方记录外部命令日志(决策 5)。
pub struct Session {
    scheduler: Scheduler,
    source: InputSource,
    next_sequence: u64,
}

impl Session {
    /// 起会话(worker 线程池随 scheduler 配置启动)。
    pub fn new(cfg: SessionConfig) -> Self {
        Self {
            scheduler: Scheduler::new(cfg.scheduler),
            source: cfg.source,
            next_sequence: 0,
        }
    }

    /// 直接注册现成分片(装配/测试用)。已注册 → panic(调度器所有权语义)。
    pub fn register(&mut self, sim: ShardSim) {
        self.scheduler.register(sim);
    }

    /// 从分片快照恢复并注册(README 决策 1:registry 由调用方注入)。
    /// 解码失败 → `Err(SessionError::Snapshot)`;恢复成功即注册。
    pub fn register_snapshot(
        &mut self,
        bytes: &[u8],
        registry: Box<dyn BlockRegistry>,
    ) -> Result<ShardId, SessionError> {
        let sim = ShardSim::restore(bytes, registry)?;
        let shard = sim.shard();
        self.scheduler.register(sim);
        Ok(shard)
    }

    /// 命令进入缝:盖 `apply_at = tick_now()`(下一次 `tick()` 生效)、
    /// `source` = 本会话、`sequence` 单调;返回 [`LogEntry`] 供调用方记录(决策 5)。
    /// 收件箱满 → `Err(SessionError::Route)`;未注册分片 → panic(调度器语义)。
    pub fn send(&mut self, shard: ShardId, command: SimCommand) -> Result<LogEntry, SessionError> {
        self.send_from(self.source, shard, command)
    }

    /// 带来源的命令进入缝(REQ-005 任务书 02,签名固定):与 [`Session::send`]
    /// 同语义(盖章 `apply_at = tick_now()`、沿会话单调 `next_sequence`),
    /// 但 `source` 用参数——网络路径的日志 source 必须是真实玩家,不是会话 1
    /// (D9:回放 canonical 序与观察者来源一致)。`send` 即 `send_from(会话)。
    pub fn send_from(
        &mut self,
        source: InputSource,
        shard: ShardId,
        command: SimCommand,
    ) -> Result<LogEntry, SessionError> {
        let tick = self.scheduler.tick_now();
        let sequence = self.next_sequence;
        self.scheduler.submit(
            shard,
            StampedInput {
                apply_at: tick,
                source,
                sequence,
                payload: InputPayload::Command(command),
            },
        )?;
        self.next_sequence += 1;
        Ok(LogEntry {
            tick,
            dest: shard,
            source,
            sequence,
            command,
        })
    }

    /// 外部广播命令(REQ-005 D3,任务书 req005-consistency/01 签名固定):
    /// dest = `owner` + 当前 holders(按 ShardId 升序去重);同一
    /// apply_at/source/sequence(整批只 +1,不是每 dest +1);每个 dest 一条
    /// [`LogEntry`](dest 升序);整批原子(`submit_batch` 任一收件箱满 → 全拒)。
    ///
    /// 校验(不过 → `Err(SessionError::Rejected)`,hostile 路径由调用方丢弃):
    /// 本任务只支持 `SetBlock`;owner 必须确为 `command 作用位置` 的归属分片
    /// (`pos.chunk_of()` + world/dim 组装,与 [`Scheduler::owner_of`] 同口径)
    /// 且已注册。收件箱满 → `Err(SessionError::Route)`(不消耗 sequence)。
    pub fn send_broadcast(
        &mut self,
        owner: ShardId,
        command: SimCommand,
    ) -> Result<Vec<LogEntry>, SessionError> {
        self.send_broadcast_from(self.source, owner, command)
    }

    /// 带来源的外部广播命令(REQ-005 任务书 02,签名固定):与
    /// [`Session::send_broadcast`] 同语义(校验/目标集/同一盖章/整批原子),
    /// 但 `source` 用参数——网络 SetBlock 的日志 source 必须是真实玩家;
    /// `send_broadcast` 即 `send_broadcast_from(会话)`。
    pub fn send_broadcast_from(
        &mut self,
        source: InputSource,
        owner: ShardId,
        command: SimCommand,
    ) -> Result<Vec<LogEntry>, SessionError> {
        let pos = match command {
            SimCommand::SetBlock { pos, .. } => pos,
            _ => return Err(SessionError::Rejected("不支持的广播命令(仅 SetBlock)")),
        };
        let chunk = pos.chunk_of();
        let true_owner = Scheduler::owner_of(owner, chunk);
        if owner != true_owner {
            return Err(SessionError::Rejected("owner 不匹配(非归属分片)"));
        }
        if !self.scheduler.shard_ids().contains(&owner) {
            return Err(SessionError::Rejected("owner 分片未注册"));
        }
        // 目标集 = {owner} ∪ holders(owner, chunk),升序去重(BTreeSet);
        // owner 一定在 holder 表里(它加载着该 chunk 才可能有这次编辑)。
        let mut targets: std::collections::BTreeSet<ShardId> =
            self.scheduler.holders_of(owner, chunk);
        targets.insert(owner);
        // 盖章:同一 apply_at/source/sequence(整批 +1);走 submit_batch(整批原子)。
        let tick = self.scheduler.tick_now();
        let sequence = self.next_sequence;
        let stamped = StampedInput {
            apply_at: tick,
            source,
            sequence,
            payload: InputPayload::Command(command),
        };
        let targets: Vec<ShardId> = targets.into_iter().collect();
        self.scheduler.submit_batch(&targets, stamped)?;
        self.next_sequence += 1;
        Ok(targets
            .into_iter()
            .map(|dest| LogEntry {
                tick,
                dest,
                source,
                sequence,
                command,
            })
            .collect())
    }

    /// SetBlock 专用便捷入口(REQ-005 D3):校验 `pos` 的归属分片 == `owner`
    /// 后组 `SetBlock` 走 [`Session::send_broadcast`](同款校验/原子性/日志)。
    pub fn set_block(
        &mut self,
        owner: ShardId,
        pos: BlockPos,
        block: voxel::BlockId,
    ) -> Result<Vec<LogEntry>, SessionError> {
        self.send_broadcast(owner, SimCommand::SetBlock { pos, block })
    }

    /// 该分片是否已加载该 chunk(REQ-005 任务 01:外部 SetBlock 的未加载
    /// chunk 门卫;Session → Scheduler → ShardSim 透传)。未注册分片 → false。
    pub fn is_chunk_loaded(&self, shard: ShardId, pos: types::ChunkPos) -> bool {
        self.scheduler.is_chunk_loaded(shard, pos)
    }

    /// 分片某个已加载 chunk 的只读视图(REQ-005 任务 02:重放一致性测试
    /// 断言用;调度器透传)。未注册分片或未加载 → None。
    pub fn voxel_chunk(&self, shard: ShardId, pos: types::ChunkPos) -> Option<&voxel::Chunk> {
        self.scheduler.voxel_chunk(shard, pos)
    }

    /// Router 待投递信封的目的分片(REQ-005 D5;驱动 tick 末 ensure 用;
    /// 调度器透传)。
    pub fn pending_shard_destinations(&self) -> std::collections::BTreeSet<ShardId> {
        self.scheduler.pending_shard_destinations()
    }

    /// 原章输入直投(回放专用;不得用于普通路径,README 决策 8):
    /// 不重新盖章、不改 sequence,目的地由调用方显式给出(决策 9——
    /// `StampedInput` 不携带 dest,显式参数比内部登记表直白,无身份冲突面)。
    /// 未注册分片/迟到输入沿用调度器语义 → panic(fail-stop)。
    pub fn submit_raw(&mut self, shard: ShardId, input: StampedInput) -> Result<(), SessionError> {
        self.scheduler.submit(shard, input)?;
        Ok(())
    }

    /// 观察者移动(REQ-002 §4.2 生产入口):盖章(apply_at = 当前拍,下一次
    /// `tick()` 生效)后走调度器专用收件箱(`submit_observer`);返回
    /// [`LogEntry`] 供调用方记录外部命令日志——dest = position 所在分片
    /// (信息性,`chunk_of().shard_of()`,floor 语义),sequence 沿会话
    /// 单调序列(与 `send` 共用)。source = 会话(测试/装配入口)。
    pub fn move_observer(&mut self, position: Vec3) -> Result<LogEntry, SessionError> {
        self.move_observer_from(self.source, position)
    }

    /// 带来源的观察者移动(REQ-005 任务书 02,签名固定):与
    /// [`Session::move_observer`] 同语义(盖章当前拍、沿会话 `next_sequence`
    /// 单调、走观察者专用收件箱),但 `source` 用参数——网络玩家的日志
    /// source 必须是玩家身份,不是会话 1(D9);`move_observer` 即
    /// `move_observer_from(会话)`。
    pub fn move_observer_from(
        &mut self,
        source: InputSource,
        position: Vec3,
    ) -> Result<LogEntry, SessionError> {
        let tick = self.scheduler.tick_now();
        let sequence = self.next_sequence;
        self.scheduler.submit_observer(StampedInput {
            apply_at: tick,
            source,
            sequence,
            payload: InputPayload::Command(SimCommand::SetObserverPosition { position }),
        });
        self.next_sequence += 1;
        // dest = position 所在分片(chunk → shard,floor 语义;信息性,不用于投递
        // ——观察者必然可走进未注册分片区域,投递走调度器专用收件箱)。
        let dest = Self::observer_dest_of(position);
        Ok(LogEntry {
            tick,
            dest,
            source,
            sequence,
            command: SimCommand::SetObserverPosition { position },
        })
    }

    /// 带来源的观察者移除(REQ-005 D9;签名固定):盖章 [`SimCommand::RemoveObserver`]
    /// 走观察者专用收件箱(`apply_at` 到期后删除该 source 条目,幂等——
    /// 不在表无害,票撤语义与 `remove_observer` 一致);沿会话 `next_sequence`
    /// 单调;返回 [`LogEntry`] 供调用方记录(断线入日志 → 可回放,无幽灵观察者)。
    ///
    /// `dest` 用**哨兵分片** `WorldId(0)/DimensionId(0)/ShardPos(0,0)`:观察者
    /// 命令不看 dest(投递走调度器收件箱,不经分片),日志的 dest 字段仅为
    /// 满足 `LogEntry` 形状;回放对观察者命令也忽略 dest(走
    /// `submit_observer_raw`)。以会话 source 直呼的便捷入口
    /// [`Session::remove_observer`](立即生效版)保留不动。
    pub fn remove_observer_from(&mut self, source: InputSource) -> Result<LogEntry, SessionError> {
        let tick = self.scheduler.tick_now();
        let sequence = self.next_sequence;
        self.scheduler.submit_observer(StampedInput {
            apply_at: tick,
            source,
            sequence,
            payload: InputPayload::Command(SimCommand::RemoveObserver),
        });
        self.next_sequence += 1;
        Ok(LogEntry {
            tick,
            // 哨兵分片(见 doc):观察者命令不看 dest,仅满足 LogEntry 形状。
            dest: ShardId::new(WorldId(0), DimensionId(0), types::ShardPos::new(0, 0)),
            source,
            sequence,
            command: SimCommand::RemoveObserver,
        })
    }

    /// 观察者命令的 dest 换算(信息性):position 所在分片(floor 语义)。
    fn observer_dest_of(position: Vec3) -> ShardId {
        ShardId::new(
            WorldId(0),
            DimensionId(0),
            types::ChunkPos::new(
                (position.x / 16.0).floor() as i32,
                (position.z / 16.0).floor() as i32,
            )
            .shard_of(),
        )
    }

    /// 观察者只读视图(调度器透传):`(source, position)`,InputSource 升序。
    pub fn observers(&self) -> Vec<(InputSource, Vec3)> {
        self.scheduler.observers()
    }

    // ---- 任务 04 授权透传(escalate 决议:任务书 02 遗漏,驱动唯一句柄是
    // Session,装载/despawn 编排 API 必须可达;签名与 shard::Scheduler 一致,
    // 一行委托,无附加逻辑)----

    /// 调度器透传(REQ-002 §4.4;驱动 tick 末端编排取用):本拍激活决策
    /// (chunk 装卸请求 + 已落实冻结),取走即清(恰一次)。
    pub fn take_activation_requests(&mut self) -> Vec<(ShardId, physics_sim::ActivationDecisions)> {
        self.scheduler.take_activation_requests()
    }

    /// 调度器透传(REQ-003 §4.4 观察输出源②;任务书 00 签名固定):本拍凝固
    /// 写入的方块(世界坐标),取走即清(恰一次)。
    pub fn take_solidified(&mut self) -> Vec<(ShardId, BlockPos, voxel::BlockId)> {
        self.scheduler.take_solidified()
    }

    /// 调度器透传(REQ-005 D11;任务书 01 签名固定):本拍方块写入的 owner
    /// 发布集(非 owner 副本的写入已由调度器丢弃),取走即清(恰一次)。
    pub fn take_block_updates(&mut self) -> Vec<(ShardId, BlockPos, voxel::BlockId)> {
        self.scheduler.take_block_updates()
    }

    /// 调度器透传(REQ-003 §4.4 观察输出源①;任务书 00):分片全实体表
    /// (EntityId 升序);未注册分片 → `None`。
    pub fn entity_states(&self, shard: ShardId) -> Option<Vec<(EntityId, EntityState)>> {
        self.scheduler.entity_states(shard)
    }

    /// 调度器透传(REQ-002 §4.4):装载一个 chunk(驱动经 provider 取数后回填)。
    pub fn apply_chunk_load(&mut self, shard: ShardId, pos: types::ChunkPos, chunk: voxel::Chunk) {
        self.scheduler.apply_chunk_load(shard, pos, chunk)
    }

    /// 调度器透传(REQ-002 §4.4):卸载一个 chunk(返回供持久化;未装载 → None)。
    pub fn apply_chunk_unload(
        &mut self,
        shard: ShardId,
        pos: types::ChunkPos,
    ) -> Option<voxel::Chunk> {
        self.scheduler.apply_chunk_unload(shard, pos)
    }

    /// 调度器透传(REQ-002 §4.5):despawn 候选(空片+无票+idle 到期;只记录不清)。
    pub fn take_despawn_requests(&mut self) -> Vec<ShardId> {
        self.scheduler.take_despawn_requests()
    }

    /// 调度器透传(REQ-002 §4.5):卸载整个分片(lifecycle 走
    /// Draining→Stopped→release,逐 chunk 卸载返回;实体非空/未注册 → panic,
    /// 调度器语义)。
    pub fn unload_shard(&mut self, shard: ShardId) -> Vec<(types::ChunkPos, voxel::Chunk)> {
        self.scheduler.unload_shard(shard)
    }

    /// 观察者原章直投(回放专用,与 [`Session::submit_raw`] 同语义):
    /// 不重新盖章、不改 sequence,直入调度器专用收件箱(迟到输入沿用
    /// 调度器语义 → panic,fail-stop)。
    pub fn submit_observer_raw(&mut self, input: StampedInput) {
        self.scheduler.submit_observer(input);
    }

    /// 调度器状态恢复(REQ-005 D7;`replay_world` 专用缝):委托
    /// `Scheduler::restore_state`——校验 tick/键集/信封目的地(违反 =
    /// 内部状态损坏 → panic fail-stop),恢复观察者表/Router/ lifecycles,
    /// 并从各片已加载 chunk 重建 holder 表。调用前提:分片已全部注册、
    /// coordinator 已对齐快照拍。
    pub fn restore_scheduler_state(&mut self, snap: shard::SchedulerSnapshot) {
        self.scheduler.restore_state(snap);
    }

    /// 移除一个观察者(REQ-003 §4.2/03;评审修复②:断连 → 票撤——
    /// 观察者表条目删除,空片无票可 despawn)。调度器透传:在表 →
    /// `Some`(原位置);不在表 → `None`(幂等)。
    pub fn remove_observer(&mut self, source: InputSource) -> Option<Vec3> {
        self.scheduler.remove_observer(source)
    }

    /// 执行一拍(全屏障;摘要透传)。
    pub fn tick(&mut self) -> TickSummary {
        self.scheduler.tick()
    }

    /// 下一将被执行的拍号(调度器透传)。
    pub fn tick_now(&self) -> Tick {
        self.scheduler.tick_now()
    }

    /// 已注册分片,ShardId 升序。
    pub fn shard_ids(&self) -> Vec<ShardId> {
        self.scheduler.shard_ids()
    }

    /// 实体当前状态;未注册分片或实体不在该分片 → None。
    pub fn entity_state(&self, shard: ShardId, entity: EntityId) -> Option<EntityState> {
        self.scheduler.entity_state(shard, entity)
    }

    /// 全局快照:**只在两拍之间调用**(tick 进行中不存在该窗口——单线程驱动,
    /// `tick()` 返回后调用方才能再进;调度器 `snapshot_shard` 同款约定)。
    /// tick 取 `tick_now()`(下一将被执行的拍号 = 已完成拍数,与重放
    /// `start_tick = snapshot.tick` 续跑语义对齐);`per_shard` 经
    /// `Scheduler::snapshot_shard` 逐分片取(升序)。
    pub fn global_snapshot(&self) -> GlobalSnapshot {
        let mut snap = GlobalSnapshot::new(self.scheduler.tick_now());
        for shard in self.scheduler.shard_ids() {
            let bytes = self
                .scheduler
                .snapshot_shard(shard)
                .expect("已注册分片必有快照(shard_ids 与 sims 同表)");
            snap.insert_shard(shard, bytes);
        }
        snap
    }

    /// 世界快照(REQ-005 D7;**只在两拍之间调用**,同 [`Session::global_snapshot`]):
    /// 分片快照 + 调度器状态一次取齐(checkpoint 的原子单位)。两个子快照的
    /// tick 不等 = 内部状态错位 → panic fail-stop(细则 3:断言 tick 相等;
    /// 单线程取数下两者同源 `tick_now`,不可能不等)。
    pub fn world_snapshot(&self) -> WorldSnapshot {
        let shards = self.global_snapshot();
        let scheduler = self.scheduler.snapshot_state();
        if shards.tick != scheduler.tick {
            panic!(
                "世界快照拍错位:分片快照 tick {} ≠ 调度器快照 tick {}(fail-stop)",
                shards.tick.0, scheduler.tick.0
            );
        }
        WorldSnapshot { shards, scheduler }
    }

    /// 关停:生命周期收尾,释放 sim 与 worker(join 全部)。
    pub fn shutdown(self) {
        self.scheduler.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use physics_sim::{ShardSimConfig, SolidifyConfig};
    use types::glam::Vec3;
    use types::{BlockPos, DimensionId, ShardPos, WorldId};
    use voxel::{Chunk, SimpleRegistry, VoxelWorld};

    /// 测试用分片身份(固定值,不引入随机;与 shard 测试同风格)。
    fn shard_id(x: i32, z: i32) -> ShardId {
        ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
    }

    /// 凝固桥关闭的虚空分片(调度器测试同款:不关心桥行为)。
    fn sim(x: i32, z: i32) -> ShardSim {
        let mut cfg = ShardSimConfig::new(shard_id(x, z));
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
    }

    fn session_cfg() -> SessionConfig {
        SessionConfig {
            scheduler: SchedulerConfig {
                worker_count: 2,
                ..SchedulerConfig::default()
            },
            ..SessionConfig::default()
        }
    }

    /// 测试清单 1:send 后 tick_now 为 N → LogEntry.tick == N;
    /// 下一次 tick() 后实体已出现。
    #[test]
    fn send_stamps_apply_at_current_tick() {
        let mut session = Session::new(session_cfg());
        let shard = shard_id(0, 0);
        session.register(sim(0, 0));
        assert_eq!(session.tick_now(), Tick::ZERO);

        let entity = EntityId::from_parts(1, 1);
        let entry = session
            .send(
                shard,
                SimCommand::SpawnDynamic {
                    entity,
                    state: EntityState {
                        translation: Vec3::new(1.5, 3.0, 1.5),
                        ..EntityState::default()
                    },
                    desc: types::EntityDesc::default(),
                },
            )
            .expect("收件箱未满");
        assert_eq!(entry.tick, Tick::ZERO, "apply_at = send 时的 tick_now");
        assert_eq!(entry.dest, shard);
        assert_eq!(entry.source, InputSource(1), "会话默认 source = 1(决策 5)");
        assert_eq!(entry.sequence, 0);
        assert!(
            session.entity_state(shard, entity).is_none(),
            "apply_at 拍未执行:实体尚未出现"
        );
        let summary = session.tick();
        assert_eq!(summary.tick, Tick::ZERO);
        assert!(
            session.entity_state(shard, entity).is_some(),
            "下一次 tick() 后实体已出现"
        );
        session.shutdown();
    }

    /// 测试清单 2:同 session 连续 send:sequence 0,1,2;
    /// 不同 source 互不影响(config 换 source,各自计数独立)。
    #[test]
    fn send_sequence_is_monotonic() {
        let mut session = Session::new(session_cfg());
        let shard = shard_id(0, 0);
        session.register(sim(0, 0));
        let cmd = SimCommand::SetBlock {
            pos: BlockPos::new(1, 2, 3),
            block: types::BlockId(1),
        };
        for expected in 0..3u64 {
            let entry = session.send(shard, cmd).expect("收件箱未满");
            assert_eq!(entry.sequence, expected, "sequence 每 session 单调");
            assert_eq!(entry.source, InputSource(1));
        }
        session.shutdown();

        // 换 source 的新会话:互不影响,sequence 仍从 0 起。
        let cfg = SessionConfig {
            source: InputSource(7),
            ..session_cfg()
        };
        let mut other = Session::new(cfg);
        other.register(sim(0, 0));
        let entry = other.send(shard, cmd).expect("收件箱未满");
        assert_eq!(entry.sequence, 0, "source 不同 → 计数独立,不受他处影响");
        assert_eq!(entry.source, InputSource(7));
        other.shutdown();
    }

    /// 测试清单 3:快照 → register_snapshot → entity_state 可查、tick 可跑。
    #[test]
    fn register_snapshot_restores_shard() {
        // 造快照:一块地形 + 一个实体,跑 3 拍。
        let shard = shard_id(0, 0);
        let mut sim0 = sim(0, 0);
        let mut voxels = VoxelWorld::new();
        let mut chunk = Chunk::air();
        for x in 0..voxel::CHUNK_X {
            for z in 0..voxel::CHUNK_Z {
                // 世界 y = 0 ↔ 局部 y = 64(WORLD_Y_MIN = -64,与既有测试同款)。
                chunk.set(x, 64, z, types::BlockId(1));
            }
        }
        voxels.insert_chunk(types::ChunkPos::new(0, 0), chunk);
        for cp in voxels.chunk_positions().collect::<Vec<_>>() {
            let chunk = voxels.remove_chunk(cp).expect("刚列出");
            sim0.load_chunk(cp, chunk);
        }
        let entity = EntityId::from_parts(1, 1);
        sim0.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource(1),
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity,
                state: EntityState {
                    translation: Vec3::new(1.5, 3.0, 1.5),
                    ..EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);
        for _ in 0..3 {
            sim0.step();
        }
        let bytes = sim0.snapshot();

        let mut session = Session::new(session_cfg());
        let restored = session
            .register_snapshot(&bytes, Box::new(SimpleRegistry::builtin()))
            .expect("自有快照必可解码");
        assert_eq!(restored, shard, "恢复后分片身份不变");
        assert_eq!(session.shard_ids(), vec![shard]);
        assert!(
            session.entity_state(shard, entity).is_some(),
            "恢复后 entity_state 可查"
        );
        let summary = session.tick();
        assert_eq!(summary.tick, Tick::ZERO, "tick 可跑(自 0 拍起)");
        assert_eq!(summary.stepped_shards, 1);
        session.shutdown();

        // 垃圾字节 → Err(Snapshot)(类型化错误透传,不 panic)。
        let mut broken = Session::new(session_cfg());
        assert!(
            broken
                .register_snapshot(&[0xFF; 8], Box::new(SimpleRegistry::builtin()))
                .is_err(),
            "残缺快照 → Err(SessionError::Snapshot)"
        );
        broken.shutdown();
    }

    /// 测试清单 4:两分片 → per_shard 键升序、tick 正确。
    #[test]
    fn global_snapshot_contains_all_shards_sorted() {
        let (a, b) = (shard_id(0, 0), shard_id(1, 0));
        let mut session = Session::new(session_cfg());
        session.register(sim(1, 0));
        session.register(sim(0, 0));
        session.tick();
        session.tick();

        let snap = session.global_snapshot();
        assert_eq!(snap.tick, Tick(2), "tick = 已执行的拍数(tick_now)");
        let keys: Vec<ShardId> = snap.per_shard.keys().copied().collect();
        assert_eq!(keys, vec![a, b], "per_shard 键 = ShardId 升序");
        for bytes in snap.per_shard.values() {
            assert!(!bytes.is_empty(), "各分片都有非空快照字节");
        }
        session.shutdown();
    }

    // ---- 任务书 02:观察者命令化(测试清单) ----

    /// 测试清单 7 `move_observer_stamps_and_logs`:LogEntry 拍号/dest/sequence
    /// 正确;入 CommandLog 可序列化;tick 后 observers() 反映。
    #[test]
    fn move_observer_stamps_and_logs() {
        use std::collections::BTreeMap;

        use types::ChunkPos;

        let mut session = Session::new(session_cfg());
        session.register(sim(0, 0));
        assert_eq!(session.tick_now(), Tick::ZERO);

        let pos = Vec3::new(8.0, 3.0, 8.0); // chunk (0,0) → 分片 (0,0)
        let entry = session.move_observer(pos).expect("观察者收件箱 unbounded");
        assert_eq!(entry.tick, Tick::ZERO, "apply_at = 当前拍(下一 tick 生效)");
        assert_eq!(
            entry.dest,
            ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(0, 0)),
            "dest = position 所在分片(信息性)"
        );
        assert_eq!(entry.source, InputSource(1), "会话 source");
        assert_eq!(entry.sequence, 0, "与会话单调序列共用");
        assert_eq!(
            entry.command,
            SimCommand::SetObserverPosition { position: pos },
            "日志携带原命令"
        );

        // sequence 与 send 共用单调序列:再 send → 1,再 move → 2。
        let shard = shard_id(0, 0);
        let sent = session
            .send(
                shard,
                SimCommand::SetBlock {
                    pos: BlockPos::new(1, 2, 3),
                    block: types::BlockId(1),
                },
            )
            .expect("收件箱未满");
        assert_eq!(sent.sequence, 1, "send 沿同一单调序列");
        let moved = session
            .move_observer(Vec3::new(200.0, 3.0, 8.0))
            .expect("观察者收件箱 unbounded");
        assert_eq!(moved.sequence, 2, "move_observer 沿同一单调序列");
        assert_eq!(
            moved.dest,
            ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(1, 0)),
            "x=200 → chunk 12 → 分片 (1,0)"
        );

        // 入 CommandLog 可序列化(bincode 往返)。
        let mut log = crate::log::CommandLog::new();
        log.append(entry.clone());
        let decoded =
            crate::log::CommandLog::from_bytes(&log.to_bytes()).expect("自有编码必可解码");
        assert_eq!(
            decoded.entries(),
            std::slice::from_ref(&entry),
            "日志往返逐字段相等"
        );

        // tick 后 observers() 反映(下一 tick 生效语义)。同 source 两次移动
        // = 覆盖(观察者表 = BTreeMap<InputSource, Vec3>,一个玩家一个位置)。
        assert!(session.observers().is_empty(), "tick 前未应用");
        session.tick();
        assert_eq!(
            session.observers(),
            vec![(InputSource(1), Vec3::new(200.0, 3.0, 8.0))],
            "tick 后观察者入表;同 source 以最后一次移动为准"
        );

        // dest 换算口径自检:position.chunk_of().shard_of()(floor 语义)。
        assert_eq!(
            ChunkPos::new(12, 0).shard_of(),
            ShardPos::new(1, 0),
            "chunk→分片换算(floor_div)"
        );
        let _ = BTreeMap::<i32, i32>::new(); // 保序容器引用(纪律自证,可删)
        session.shutdown();
    }

    /// 测试清单 8 `move_observer_apply_at_next_tick`:未来拍语义——观察者
    /// 命令在 move 之后的第一个 tick() 生效(与 send 同拍语义)。
    #[test]
    fn move_observer_apply_at_next_tick() {
        let mut session = Session::new(session_cfg());
        session.register(sim(0, 0));
        for _ in 0..3 {
            session.tick();
        }
        let pos = Vec3::new(40.0, 0.0, 40.0);
        let entry = session.move_observer(pos).expect("unbounded");
        assert_eq!(entry.tick, Tick(3), "apply_at = 当前拍 3");
        assert!(session.observers().is_empty(), "未 tick 不应用");
        session.tick();
        assert_eq!(
            session.observers(),
            vec![(InputSource(1), pos)],
            "下一 tick(拍 3)应用"
        );
        session.shutdown();
    }

    /// 任务 04 授权透传的最小测试:编排 API 经 Session 可达且行为与
    /// Scheduler 一致(激活请求恰一次;chunk 装载物理可见;卸载返回;
    /// unload_shard 后 shard_ids 不含)。全流程冒烟,详测在 shard/app。
    #[test]
    fn orchestration_pass_through_reachable() {
        use types::ChunkPos;

        let mut session = Session::new(session_cfg());
        let shard = shard_id(0, 0);
        session.register(sim(0, 0));
        // 观察者入表(直通,激活语义开关)→ tick 收集激活请求。
        session
            .move_observer(Vec3::new(8.0, 0.0, 8.0))
            .expect("unbounded");
        session.tick();
        let requests = session.take_activation_requests();
        assert!(!requests.is_empty(), "有观察者 → 本拍激活决策非空");
        assert!(
            session.take_activation_requests().is_empty(),
            "恰一次:取走即清"
        );
        // 装载 → 卸载往返。
        let cp = ChunkPos::new(0, 0);
        let chunk = voxel::Chunk::air();
        session.apply_chunk_load(shard, cp, chunk.clone());
        assert_eq!(
            session.apply_chunk_unload(shard, cp),
            Some(chunk),
            "卸载返回装载值"
        );
        session.apply_chunk_load(shard, cp, voxel::Chunk::air());
        // 空片卸载(未到 idle 期也可显式卸载:调度器语义允许)。
        let chunks = session.unload_shard(shard);
        assert_eq!(chunks, vec![(cp, voxel::Chunk::air())], "逐 chunk 卸载返回");
        assert!(!session.shard_ids().contains(&shard), "卸载后不含该分片");
        session.shutdown();
    }

    // ---- 任务书 00(REQ-003):观察输出透传(测试清单) ----

    /// 测试清单 5 `take_solidified_exactly_once`:真静止实体经凝固桥落格 →
    /// 该拍 tick 后 take_solidified 得 `(shard, pos, block)`;再取 = 空(恰一次)。
    #[test]
    fn take_solidified_exactly_once() {
        // 凝固桥开启 + 关位移门(直通,几拍内即凝固)。
        let mut cfg = ShardSimConfig::new(shard_id(0, 0));
        cfg.gravity = Vec3::ZERO;
        cfg.solidify.displacement_gate_m = 0.0;
        let mut sim0 = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
        let e = EntityId::from_parts(1, 1);
        sim0.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource(1),
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(60.5, 3.5, 60.5),
                    ..EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);
        let shard = shard_id(0, 0);
        let mut session = Session::new(session_cfg());
        session.register(sim0);

        // 跑到凝固发生(真静止 20 tick 内;位移门已关)。
        let mut cell = None;
        for _ in 0..40 {
            session.tick();
            let taken = session.take_solidified();
            if !taken.is_empty() {
                cell = Some(taken);
                break;
            }
        }
        let taken = cell.expect("真静止 40 拍内凝固,观察输出必收到");
        assert_eq!(taken.len(), 1, "单实体单格");
        let (got_shard, pos, block) = taken[0];
        assert_eq!(got_shard, shard);
        assert_eq!(pos, BlockPos::new(60, 3, 60), "格 = snap 后的体素中心格");
        assert_eq!(block, types::BlockId(1), "方块 = EntityDesc 默认写入值");
        assert!(
            session.take_solidified().is_empty(),
            "再取 = 空(恰一次:取走即清)"
        );
        session.shutdown();
    }

    /// 测试清单 6 `solidified_survives_via_session`:Session 透传可达(最小
    /// 冒烟)——entity_states 透传与调度器行为一致(全表升序/未注册 None),
    /// 与 take_solidified 同一会话内并存可用。
    #[test]
    fn solidified_survives_via_session() {
        let shard = shard_id(0, 0);
        let mut session = Session::new(session_cfg());
        assert!(session.entity_states(shard).is_none(), "未注册分片 → None");

        // 凝固桥开启 + 关位移门:实体真静止落格(复用清单 5 场景)。
        let mut cfg = ShardSimConfig::new(shard);
        cfg.gravity = Vec3::ZERO;
        cfg.solidify.displacement_gate_m = 0.0;
        let mut sim0 = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
        let e = EntityId::from_parts(1, 1);
        sim0.apply_prefix(&[StampedInput {
            apply_at: Tick::ZERO,
            source: InputSource(1),
            sequence: 0,
            payload: InputPayload::Command(SimCommand::SpawnDynamic {
                entity: e,
                state: EntityState {
                    translation: Vec3::new(60.5, 3.5, 60.5),
                    ..EntityState::default()
                },
                desc: types::EntityDesc::default(),
            }),
        }]);
        session.register(sim0);

        // 凝固前:全表可见(单条,升序)。
        let table = session.entity_states(shard).expect("已注册分片必有表");
        assert_eq!(table.iter().map(|(id, _)| *id).collect::<Vec<_>>(), vec![e]);

        // 跑到凝固:take_solidified 收到,实体从表中消失(已凝固移除)。
        let mut solidified = false;
        for _ in 0..40 {
            session.tick();
            if !session.take_solidified().is_empty() {
                solidified = true;
                break;
            }
        }
        assert!(solidified, "透传链可达:凝固方块经 Session 冒出");
        assert!(
            session.entity_states(shard).expect("表可达").is_empty(),
            "实体已凝固移除 → 全表为空"
        );
        session.shutdown();
    }

    // ---- 任务书 req005-consistency/01:外部编辑同拍广播(测试清单) ----

    /// 两片装载同一边界 chunk(owner (0,0) + 扩张副本 (1,0))的会话。
    fn session_two_holders() -> Session {
        let mut session = Session::new(session_cfg());
        session.register(sim(0, 0));
        session.register(sim(1, 0));
        let cp = types::ChunkPos::new(7, 0); // owner = 片 (0,0)
        session.apply_chunk_load(shard_id(0, 0), cp, voxel::Chunk::air());
        session.apply_chunk_load(shard_id(1, 0), cp, voxel::Chunk::air());
        session
    }

    /// 任务书测试 `setblock_broadcast_reaches_owner_and_holders`:两片持有同一
    /// chunk;`set_block` 后下一拍两个 sim 的该格都变为新值。
    #[test]
    fn setblock_broadcast_reaches_owner_and_holders() {
        let mut session = session_two_holders();
        let (owner, holder) = (shard_id(0, 0), shard_id(1, 0));
        let pos = BlockPos::new(120, 40, 8);
        let entries = session
            .set_block(owner, pos, types::BlockId(2))
            .expect("两片收件箱均未满");
        assert_eq!(entries.len(), 2, "owner + holder 各一条");
        session.tick();
        let cp = types::ChunkPos::new(7, 0);
        for shard in [owner, holder] {
            assert_eq!(
                session
                    .scheduler
                    .voxel_chunk(shard, cp)
                    .expect("已装载")
                    .get(8, 104, 8),
                types::BlockId(2),
                "{shard:?} 的副本同拍变为新值"
            );
        }
        session.shutdown();
    }

    /// 任务书测试 `setblock_broadcast_same_apply_at`:两片的广播输入 apply_at
    /// 相同(同一盖章拍),下一拍同时生效。
    #[test]
    fn setblock_broadcast_same_apply_at() {
        let mut session = session_two_holders();
        let owner = shard_id(0, 0);
        for _ in 0..3 {
            session.tick();
        }
        let pos = BlockPos::new(120, 40, 8);
        let entries = session
            .set_block(owner, pos, types::BlockId(2))
            .expect("收件箱未满");
        assert_eq!(session.tick_now(), Tick(3));
        assert!(
            entries.iter().all(|e| e.tick == Tick(3)),
            "全部条目同一 apply_at = 当前拍:{entries:?}"
        );
        // 生效前不应用。
        assert!(
            session
                .scheduler
                .voxel_chunk(shard_id(1, 0), types::ChunkPos::new(7, 0))
                .expect("已装载")
                .get(8, 104, 8)
                == types::BlockId::AIR,
            "apply_at 拍未执行:holder 副本尚未变"
        );
        session.tick();
        // 下一拍同时生效(两片同拍)。
        for shard in [shard_id(0, 0), shard_id(1, 0)] {
            assert_eq!(
                session
                    .scheduler
                    .voxel_chunk(shard, types::ChunkPos::new(7, 0))
                    .expect("已装载")
                    .get(8, 104, 8),
                types::BlockId(2),
                "{shard:?} 同一拍生效"
            );
        }
        session.shutdown();
    }

    /// 任务书测试 `setblock_broadcast_logs_one_entry_per_shard`:返回 2 条
    /// LogEntry,dest 升序、同 tick/source/sequence(整批只 +1)。
    #[test]
    fn setblock_broadcast_logs_one_entry_per_shard() {
        let mut session = session_two_holders();
        let owner = shard_id(0, 0);
        let pos = BlockPos::new(120, 40, 8);
        let entries = session
            .set_block(owner, pos, types::BlockId(2))
            .expect("收件箱未满");
        assert_eq!(entries.len(), 2);
        assert_eq!(
            entries.iter().map(|e| e.dest).collect::<Vec<_>>(),
            vec![shard_id(0, 0), shard_id(1, 0)],
            "dest 升序"
        );
        let first = &entries[0];
        for e in &entries {
            assert_eq!(e.tick, first.tick, "同 tick");
            assert_eq!(e.source, first.source, "同 source");
            assert_eq!(e.sequence, first.sequence, "同 sequence(整批 +1)");
            assert_eq!(
                e.command,
                SimCommand::SetBlock {
                    pos,
                    block: types::BlockId(2)
                },
                "同命令"
            );
        }
        // sequence 恰 +1:下一次 send 用下一号(不是每 dest +1)。
        let next = session
            .send(
                owner,
                SimCommand::SetBlock {
                    pos,
                    block: types::BlockId(3),
                },
            )
            .expect("收件箱未满");
        assert_eq!(
            next.sequence,
            first.sequence + 1,
            "广播一批只消耗一个 sequence"
        );
        session.shutdown();
    }

    /// 任务书测试 `setblock_owner_mismatch_rejected`:pos 的 owner != 传入
    /// owner → `Err(SessionError::Rejected)`,不投任何片、不消耗 sequence。
    #[test]
    fn setblock_owner_mismatch_rejected() {
        let mut session = session_two_holders();
        let pos = BlockPos::new(120, 40, 8); // 归属 = 片 (0,0)
        let entries = session.set_block(shard_id(1, 0), pos, types::BlockId(2));
        match entries {
            Err(SessionError::Rejected(why)) => {
                assert!(why.contains("owner"), "理由指明 owner 不匹配:{why}");
            }
            other => panic!("owner 不匹配必须 Err(Rejected),实为 {other:?}"),
        }
        // 未消耗 sequence:下一个合法命令仍是 0 号。
        let legit = session
            .send(
                shard_id(0, 0),
                SimCommand::SetBlock {
                    pos,
                    block: types::BlockId(2),
                },
            )
            .expect("收件箱未满");
        assert_eq!(legit.sequence, 0, "被拒广播不消耗 sequence");
        session.shutdown();
    }

    /// send_broadcast 对非 SetBlock 命令 → Err(Rejected)(本任务只支持
    /// SetBlock;Detonate 广播属 REQ-005 §4.3 后续任务)。
    #[test]
    fn send_broadcast_rejects_non_setblock() {
        let mut session = session_two_holders();
        let result = session.send_broadcast(
            shard_id(0, 0),
            SimCommand::Detonate {
                center: Vec3::ZERO,
                power: 6.0,
                radius: 5.0,
            },
        );
        assert!(
            matches!(result, Err(SessionError::Rejected(_))),
            "非 SetBlock 广播 → Err(Rejected)"
        );
        session.shutdown();
    }

    /// is_chunk_loaded 透传:已装载 true / 未装载 false / 未注册分片 false。
    #[test]
    fn is_chunk_loaded_passthrough_session() {
        let session = session_two_holders();
        let cp = types::ChunkPos::new(7, 0);
        assert!(
            session.is_chunk_loaded(shard_id(0, 0), cp),
            "owner 已装载 → true"
        );
        assert!(
            session.is_chunk_loaded(shard_id(1, 0), cp),
            "holder 已装载 → true"
        );
        assert!(
            !session.is_chunk_loaded(shard_id(0, 0), types::ChunkPos::new(3, 3)),
            "未装载 chunk → false"
        );
        assert!(
            !session.is_chunk_loaded(shard_id(9, 9), cp),
            "未注册分片 → false"
        );
        session.shutdown();
    }

    /// pending_shard_destinations 透传:无待投递信封 → 空集(转发 = 调度器
    /// 视图;信封产出的端到端场景由 shard 级 pending_destinations 测试与
    /// app 级 pending_destination_ensured_before_next_tick 覆盖)。
    #[test]
    fn pending_shard_destinations_passthrough() {
        use std::collections::BTreeSet;

        let session = Session::new(session_cfg());
        assert_eq!(
            session.pending_shard_destinations(),
            BTreeSet::new(),
            "透传 = 调度器视图"
        );
        session.shutdown();
    }

    // ---- 任务书 req005-consistency/02:全局快照 / 观察者来源 API(测试清单)----

    /// 任务书测试 `world_snapshot_tick_consistency`:`world.shards.tick ==
    /// world.scheduler.tick`(同一 tick_now 同源;不等 = 内部错位 panic,
    /// 正常路径断言相等即可)。
    #[test]
    fn world_snapshot_tick_consistency() {
        let mut session = Session::new(session_cfg());
        session.register(sim(0, 0));
        for _ in 0..5 {
            session.tick();
        }
        let world = session.world_snapshot();
        assert_eq!(world.shards.tick, Tick(5));
        assert_eq!(
            world.shards.tick, world.scheduler.tick,
            "两个子快照的 tick 必相等(两拍之间同源取数)"
        );
        session.shutdown();
    }

    /// 任务书测试 `send_from_logs_player_source`:网络 Send 的 LogEntry.source
    /// = 玩家(source 参数),不是会话 1;sequence 沿会话单调与 send 共用。
    #[test]
    fn send_from_logs_player_source() {
        let mut session = Session::new(session_cfg());
        let shard = shard_id(0, 0);
        session.register(sim(0, 0));
        let player = InputSource(9);
        let entry = session
            .send_from(
                player,
                shard,
                SimCommand::SetBlock {
                    pos: BlockPos::new(1, 2, 3),
                    block: types::BlockId(1),
                },
            )
            .expect("收件箱未满");
        assert_eq!(entry.source, player, "日志 source = 玩家,非会话 1");
        assert_ne!(entry.source, InputSource(1));
        assert_eq!(entry.tick, Tick::ZERO);
        assert_eq!(entry.dest, shard);

        // 广播版同语义:send_broadcast_from 的全部条目 source = 玩家。
        session.apply_chunk_load(shard, types::ChunkPos::new(0, 0), voxel::Chunk::air());
        let entries = session
            .send_broadcast_from(
                player,
                shard,
                SimCommand::SetBlock {
                    pos: BlockPos::new(4, 5, 4),
                    block: types::BlockId(2),
                },
            )
            .expect("收件箱未满");
        assert!(entries.iter().all(|e| e.source == player));
        // sequence 沿同一单调序列:send_from(0) → 广播批(1)→ 会话 send(2)。
        let session_entry = session
            .send(
                shard,
                SimCommand::SetBlock {
                    pos: BlockPos::new(1, 2, 3),
                    block: types::BlockId(1),
                },
            )
            .expect("收件箱未满");
        assert_eq!(session_entry.sequence, 2, "三种入口共用会话单调序列");
        assert_eq!(
            session_entry.source,
            InputSource(1),
            "会话入口 = 会话 source"
        );
        session.shutdown();
    }

    /// 任务书测试 `checkpoint_bytes_deterministic`:同状态两次 `world_snapshot`
    /// 编码字节相等(确定性:快照字节同输入确定,AC-C5)。
    #[test]
    fn checkpoint_bytes_deterministic() {
        let mut session = Session::new(session_cfg());
        session.register(sim(0, 0));
        session
            .move_observer(Vec3::new(8.0, 3.0, 8.0))
            .expect("unbounded");
        for _ in 0..3 {
            session.tick();
        }
        let a = session.world_snapshot();
        let b = session.world_snapshot();
        // 两路确定性口径:分片字节(bcrate 编码)+ 调度器快照整字段 + 整体
        // bincode 字节直接对拍再解码对拍(serde 现已派生,CheckpointBundle
        // 落盘路径同源)。
        assert_eq!(a.shards.to_bytes(), b.shards.to_bytes(), "分片快照字节确定");
        assert_eq!(a.scheduler, b.scheduler, "调度器快照逐字段相等");
        let bytes_a = bincode::serde::encode_to_vec(&a, bincode::config::standard())
            .expect("编码不可能失败(fail-stop)");
        let bytes_b = bincode::serde::encode_to_vec(&b, bincode::config::standard())
            .expect("编码不可能失败(fail-stop)");
        assert_eq!(
            bytes_a, bytes_b,
            "同状态两次 world_snapshot 的整体编码字节相等"
        );
        let (ea, _): (WorldSnapshot, usize) =
            bincode::serde::decode_from_slice(&bytes_a, bincode::config::standard())
                .expect("自有编码必可解码");
        let (eb, _): (WorldSnapshot, usize) =
            bincode::serde::decode_from_slice(&bytes_b, bincode::config::standard())
                .expect("自有编码必可解码");
        assert_eq!(ea, eb, "同状态两次 world_snapshot 的编码解码后逐字段相等");
        session.shutdown();
    }

    /// 任务书测试 `checkpoint_restores_observers_without_move`:快照后经
    /// replay_world 恢复,不发任何 MoveObserver,observers() 非空且激活继续
    /// (观察者表快照兜底 + 无需日志条目)。
    #[test]
    fn checkpoint_restores_observers_without_move() {
        use crate::replay::replay_world;

        let mut original = Session::new(session_cfg());
        original.register(sim(0, 0));
        let pos = Vec3::new(8.0, 3.0, 8.0);
        let entry = original.move_observer(pos).expect("unbounded");
        let mut log = crate::log::CommandLog::new();
        log.append(entry);
        original.tick(); // 拍 0:观察者入表(进入快照)
        let world = original.world_snapshot();
        original.shutdown();
        assert_eq!(world.scheduler.observers.len(), 1, "前置:观察者入快照");

        // 空日志重放:表来自快照兜底,不再发 MoveObserver。
        let empty_log = crate::log::CommandLog::new();
        let mut replayed = replay_world(&world, &empty_log, session_cfg(), || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("自有快照必可恢复");
        assert_eq!(
            replayed.observers(),
            vec![(InputSource(1), pos)],
            "恢复线观察者表非空(不发 MoveObserver,快照兜底)"
        );
        // 激活继续:再 tick 一拍,激活请求仍按观察者收集(非空)。
        replayed.tick();
        assert!(
            !replayed.take_activation_requests().is_empty(),
            "恢复线激活继续(观察者中心生效)"
        );
        replayed.shutdown();
    }
}
