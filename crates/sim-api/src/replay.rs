//! restore checkpoint + 重放命令日志;终态位级一致(AC8)。

use messaging::{InputPayload, SimCommand, StampedInput};
use voxel::BlockRegistry;

use crate::log::{CommandLog, LogEntry};
use crate::session::{Session, SessionConfig, SessionError};
use crate::snapshot::GlobalSnapshot;

/// 从全局快照 + 命令日志重建会话(AC8 执行侧)。
///
/// - `start_tick` 对齐 `snapshot.tick`(README 决策 6):**覆盖**
///   `cfg.scheduler.start_tick = snapshot.tick`(以快照为准,语义细则 2);
/// - 只重放 `tick > snapshot.tick` 的条目(≤ 的已烙进快照),按日志条目序
///   (append 序)原章直投:经 `submit_raw(dest, …)` 显式携带目的地,
///   不重新盖章、不改 sequence(README 决策 8/9——重新盖章会让序号漂移,
///   两线 canonical 序不再一致);
/// - registry 经工厂逐分片注入(不序列化,任务 01 决策)。
///
/// 返回的 Session 已注册全部分片并投递完命令;调用方 tick 到目标拍后比对终态。
pub fn replay(
    snapshot: &GlobalSnapshot,
    log: &CommandLog,
    cfg: SessionConfig,
    new_registry: impl Fn() -> Box<dyn BlockRegistry>,
) -> Result<Session, SessionError> {
    let mut cfg = cfg;
    cfg.scheduler.start_tick = snapshot.tick;

    let mut session = Session::new(cfg);
    // 逐分片恢复注册(BTreeMap 迭代 = ShardId 升序;registry 每片新造)。
    for bytes in snapshot.per_shard.values() {
        session.register_snapshot(bytes, new_registry())?;
    }
    // 按日志条目序投递(tick/source/sequence/command/dest 全部原样):
    // SetObserverPosition → 观察者专用收件箱原章直投(README 决策 3——
    // 它不走分片收件箱;经 submit_raw 会触发"未注册分片"或沉入分片前缀,
    // 两者都错);其余 → submit_raw(dest, …) 照旧。
    // 迟到检查说明:submit_observer 拒绝 apply_at < 起始拍的输入(fail-stop),
    // 而 replay 只重放 tick > snapshot.tick 的条目 = 起始拍之后,恒通过。
    for entry in log.entries() {
        if entry.tick > snapshot.tick {
            match entry.command {
                SimCommand::SetObserverPosition { position: _ } => {
                    session.submit_observer_raw(stamped(entry.clone()));
                }
                _ => session.submit_raw(entry.dest, stamped(entry.clone()))?,
            }
        }
    }
    Ok(session)
}

/// 日志条目 → 盖章输入(重放直投用):tick → apply_at,其余字段原样;
/// 目的地由 `LogEntry.dest` 显式传入 `submit_raw`(决策 9)。
fn stamped(entry: LogEntry) -> StampedInput {
    StampedInput {
        apply_at: entry.tick,
        source: entry.source,
        sequence: entry.sequence,
        payload: InputPayload::Command(entry.command),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::log::LogEntry;
    use crate::session::SessionConfig;
    use messaging::{InputSource, SimCommand};
    use shard::SchedulerConfig;
    use types::{BlockPos, DimensionId, ShardPos, Tick, WorldId};

    use voxel::SimpleRegistry;

    /// 测试用分片身份(固定值,不引入随机)。
    fn shard_id(x: i32, z: i32) -> types::ShardId {
        types::ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
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

    fn registry_factory() -> Box<dyn BlockRegistry> {
        Box::new(SimpleRegistry::builtin())
    }

    /// 一条日志条目(tick/sequence/dest 可控,source 固定会话默认 1)。
    fn log_entry(tick: u64, sequence: u64, dest: types::ShardId) -> LogEntry {
        LogEntry {
            tick: Tick(tick),
            dest,
            source: InputSource(1),
            sequence,
            command: SimCommand::SetBlock {
                pos: BlockPos::new(1, 2, 3),
                block: types::BlockId(1),
            },
        }
    }

    /// 测试清单 5:快照拍 20;日志含 tick 10/20/21 → 只重放 21
    /// (观察:重放后 send 的 next_sequence 不受干扰、tick_now == 20)。
    #[test]
    fn replay_skips_entries_at_or_before_snapshot_tick() {
        let shard = shard_id(0, 0);
        // 真快照:建会话注册分片跑 20 拍后取(保证 register_snapshot 可解码)。
        let mut original = Session::new(session_cfg());
        original.register(physics_sim::ShardSim::new(
            physics_sim::ShardSimConfig::new(shard),
            registry_factory(),
        ));
        for _ in 0..20 {
            original.tick();
        }
        let snapshot = original.global_snapshot();
        assert_eq!(snapshot.tick, Tick(20));
        original.shutdown();

        let mut log = CommandLog::new();
        log.append(log_entry(10, 0, shard));
        log.append(log_entry(20, 1, shard));
        log.append(log_entry(21, 2, shard));

        let mut replayed =
            replay(&snapshot, &log, session_cfg(), registry_factory).expect("快照自有必可恢复");
        assert_eq!(replayed.tick_now(), Tick(20), "start_tick 对齐快照拍");

        // 只重放 tick 21:send 的 next_sequence 不受重放条目干扰(从 0 起)。
        let entry = replayed
            .send(
                shard,
                SimCommand::SetBlock {
                    pos: BlockPos::new(2, 2, 2),
                    block: types::BlockId(1),
                },
            )
            .expect("收件箱未满");
        assert_eq!(
            entry.sequence, 0,
            "重放条目原章直投,不占 send 的 sequence 空间"
        );
        assert_eq!(entry.tick, Tick(20));
        replayed.shutdown();
    }

    /// 测试清单 6:replay 后 tick_now == snapshot.tick;
    /// 再 tick() 的 summary.tick == snapshot.tick。
    #[test]
    fn replay_restores_from_start_tick() {
        let shard = shard_id(0, 0);
        let mut original = Session::new(session_cfg());
        original.register(physics_sim::ShardSim::new(
            physics_sim::ShardSimConfig::new(shard),
            registry_factory(),
        ));
        for _ in 0..7 {
            original.tick();
        }
        let snapshot = original.global_snapshot();
        original.shutdown();

        let log = CommandLog::new();
        let mut replayed =
            replay(&snapshot, &log, session_cfg(), registry_factory).expect("快照自有必可恢复");
        assert_eq!(
            replayed.tick_now(),
            Tick(7),
            "replay 后 tick_now == snapshot.tick"
        );
        let summary = replayed.tick();
        assert_eq!(summary.tick, Tick(7), "再 tick() 执行 snapshot.tick 拍");
        assert_eq!(replayed.tick_now(), Tick(8));
        replayed.shutdown();
    }

    /// 测试清单 7 `replay_routes_observer_entries`(任务书 02):含
    /// SetObserverPosition 的日志重放——观察者表再生,重放后 observers()
    /// 与原线一致(原章直投:source/sequence 原样;不进分片收件箱)。
    /// 时序:命令盖拍 4(= 快照拍 3 之后)、原线在拍 4 应用;重放线把
    /// 条目直投收件箱后跑 2 拍,表在同拍再生。
    #[test]
    fn replay_routes_observer_entries() {
        use types::glam::Vec3;

        let shard = shard_id(0, 0);
        // 原线:注册分片 → 3 拍 → 快照(拍 3 = 已完成拍数)。
        let mut original = Session::new(session_cfg());
        original.register(physics_sim::ShardSim::new(
            physics_sim::ShardSimConfig::new(shard),
            registry_factory(),
        ));
        for _ in 0..3 {
            original.tick();
        }
        let snapshot = original.global_snapshot();
        assert_eq!(snapshot.tick, Tick(3));

        // 命令在拍 3 末投递(盖拍 3,拍 3 是下一拍)。为使条目 tick 严格
        // > 快照拍(重放过滤条件),先跑 1 拍把 tick_now 推到 4,再投命令
        // (盖拍 4)。原线再跑 2 拍(拍 4/5)→ 观察者命令在拍 4 应用。
        original.tick();
        let p1 = Vec3::new(8.0, 3.0, 8.0);
        let p2 = Vec3::new(40.0, 3.0, 40.0);
        let e1 = original.move_observer(p1).expect("unbounded");
        let e2 = original
            .send(
                shard,
                SimCommand::SetBlock {
                    pos: BlockPos::new(1, 2, 3),
                    block: types::BlockId(1),
                },
            )
            .expect("收件箱未满");
        let e3 = original.move_observer(p2).expect("unbounded");
        assert_eq!(e1.tick, Tick(4), "命令盖拍 4(> 快照拍 3)");
        for _ in 0..2 {
            original.tick();
        }
        assert_eq!(
            original.observers(),
            vec![(InputSource(1), p2)],
            "原线观察者表:同 source 两次移动 = 覆盖(表 = BTreeMap<source, pos>)"
        );
        original.shutdown();

        // 重放:同快照 + 同日志 → 条目全部重放(tick 4 > 3),观察者表再生。
        let mut log = CommandLog::new();
        log.append(e1);
        log.append(e2);
        log.append(e3);
        let mut replayed =
            replay(&snapshot, &log, session_cfg(), registry_factory).expect("快照自有必可恢复");
        assert!(
            replayed.observers().is_empty(),
            "重放条目入收件箱,尚未 tick:表暂空(与原线拍 4 前一致)"
        );
        replayed.tick(); // 执行拍 3:无观察者条目到期。
        replayed.tick(); // 执行拍 4:观察者条目到期 → 表再生。
        assert_eq!(
            replayed.observers(),
            vec![(InputSource(1), p2)],
            "重放后观察者表与原线一致(SetObserverPosition → submit_observer 路由)"
        );
        replayed.shutdown();
    }
}
