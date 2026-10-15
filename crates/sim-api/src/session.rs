//! Session:命令进/快照出的 headless 缝(接口形状 = 网络形状)。

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{ShardSim, SimSnapshotError};
use shard::router::RouteError;
use shard::{Scheduler, SchedulerConfig, TickSummary};
use types::{EntityId, EntityState, ShardId, Tick};
use voxel::BlockRegistry;

use crate::log::LogEntry;
use crate::snapshot::GlobalSnapshot;

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

/// 会话层可预期失败:路由(收件箱满)与快照恢复(残缺字节)透传。
/// 其余(未注册分片、迟到输入、原始直投未登记)沿用调度器语义 → panic(fail-stop)。
#[derive(Debug, thiserror::Error)]
pub enum SessionError {
    #[error("路由失败:{0}")]
    Route(#[from] RouteError),
    #[error("快照恢复失败:{0}")]
    Snapshot(#[from] SimSnapshotError),
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
        let tick = self.scheduler.tick_now();
        let sequence = self.next_sequence;
        self.scheduler.submit(
            shard,
            StampedInput {
                apply_at: tick,
                source: self.source,
                sequence,
                payload: InputPayload::Command(command),
            },
        )?;
        self.next_sequence += 1;
        Ok(LogEntry {
            tick,
            dest: shard,
            source: self.source,
            sequence,
            command,
        })
    }

    /// 原章输入直投(回放专用;不得用于普通路径,README 决策 8):
    /// 不重新盖章、不改 sequence,目的地由调用方显式给出(决策 9——
    /// `StampedInput` 不携带 dest,显式参数比内部登记表直白,无身份冲突面)。
    /// 未注册分片/迟到输入沿用调度器语义 → panic(fail-stop)。
    pub fn submit_raw(&mut self, shard: ShardId, input: StampedInput) -> Result<(), SessionError> {
        self.scheduler.submit(shard, input)?;
        Ok(())
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
}
