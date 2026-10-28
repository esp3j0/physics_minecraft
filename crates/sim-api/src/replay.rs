//! restore checkpoint + 重放命令日志;终态位级一致(AC8)。
//! REQ-005 任务书 02:`replay_world`(WorldSnapshot + scheduler 状态恢复)。

use messaging::{InputPayload, SimCommand, StampedInput};
use types::Tick;
use voxel::BlockRegistry;

use crate::log::{CommandLog, LogEntry};
use crate::session::{Session, SessionConfig, SessionError};
use crate::snapshot::{GlobalSnapshot, WorldSnapshot};

/// 从全局快照 + 命令日志重建会话(AC8 执行侧;旧入口,保留给既有
/// GlobalSnapshot 测试——回放规则与 [`replay_world`] 一致)。
///
/// - `start_tick` 对齐 `snapshot.tick`(**覆盖** `cfg.scheduler.start_tick`,
///   以快照为准,语义细则 2);
/// - 重放窗口 **`entry.tick >= snapshot.tick`**(细则 7 修正):checkpoint 的
///   `tick` 是"下一待执行拍"(`tick_now()`),等于该拍的命令尚未应用,必须
///   回放;`< snapshot.tick` 的已烙进快照。旧 `>` 过滤会把"checkpoint 后、
///   下一拍前到达"的 `==` 条目丢掉(数据丢失面),本任务统一 `>=`;
/// - 按日志条目序(append 序)原章直投:观察者命令(SetObserverPosition)→
///   `submit_observer_raw` 原章重投(apply_at = entry.tick、source =
///   entry.source、sequence = entry.sequence);其余 → `submit_raw(dest)`,
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
    replay_log_entries(&mut session, snapshot.tick, log.entries())?;
    Ok(session)
}

/// 从世界快照 + 命令日志重建会话(REQ-005 任务书 02:恢复顺序细则 6):
/// 1. `Scheduler::new(start_tick = world.shards.tick)`(覆盖 cfg,快照为准);
/// 2. 逐片 `register_snapshot`(ShardId 升序;分片字节先恢复注册——
///    register 末尾的 holder 重建此刻只看本片);
/// 3. `restore_state(world.scheduler)`:校验 tick/键集/信封目的地(内部状态
///    不一致 → panic fail-stop),恢复观察者表/Router(sequences 原样)/
///    lifecycles,并从各片已加载 chunk 重建 holder 表;
/// 4. 回放日志 `tick >= snapshot.tick`(观察者命令走观察者收件箱,其余按
///    dest 原章直投)。
///
/// 调用方随后照常 ensure/装载/despawn(驱动编排)并 tick 到目标拍。
pub fn replay_world(
    world: &WorldSnapshot,
    log: &CommandLog,
    cfg: SessionConfig,
    new_registry: impl Fn() -> Box<dyn BlockRegistry>,
) -> Result<Session, SessionError> {
    let mut cfg = cfg;
    cfg.scheduler.start_tick = world.shards.tick;

    let mut session = Session::new(cfg);
    for bytes in world.shards.per_shard.values() {
        session.register_snapshot(bytes, new_registry())?;
    }
    session.restore_scheduler_state(world.scheduler.clone());
    replay_log_entries(&mut session, world.shards.tick, log.entries())?;
    Ok(session)
}

/// 日志条目按重放窗口投递(两条回放入口共用;细则 7):
/// `entry.tick < snapshot.tick` 跳过(已烙进快照);`>=` 全部回放——`==` 的
/// 待执行命令(checkpoint 后、下一拍前到达)必须应用,丢了 = 数据丢失。
/// 观察者命令(SetObserverPosition/RemoveObserver)→ 观察者专用收件箱原章
/// 重投;其余 → `submit_raw(dest)` 原章(不重新盖章、不改 sequence)。
fn replay_log_entries(
    session: &mut Session,
    snapshot_tick: Tick,
    entries: &[LogEntry],
) -> Result<(), SessionError> {
    for entry in entries {
        if entry.tick < snapshot_tick {
            continue;
        }
        match entry.command {
            SimCommand::SetObserverPosition { .. } | SimCommand::RemoveObserver => {
                session.submit_observer_raw(stamped(entry.clone()));
            }
            _ => session.submit_raw(entry.dest, stamped(entry.clone()))?,
        }
    }
    Ok(())
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

    /// REQ-010 §12.1 第 3 条迁移:本模块用例按 REQ-009 及更早语义编写(悬空
    /// SetBlock 保持原样),与默认开启的承载判定冲突 → 装配处显式关闭本开关,
    /// 断言不变。
    fn sim_cfg(shard: types::ShardId) -> physics_sim::ShardSimConfig {
        let mut cfg = physics_sim::ShardSimConfig::new(shard);
        cfg.cohesion.enabled = false;
        cfg
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

    /// 测试清单 5(替换旧 `..._at_or_before` 语义):`replay_skips_only_entries_
    /// before_snapshot_tick`——`< snapshot.tick` 跳过;`==` 与 `>` 都回放
    /// (细则 7:snapshot.tick 是下一待执行拍,== 的待执行命令必须应用)。
    /// 观察面:重放后 send 的 next_sequence 不受干扰、tick_now == 快照拍。
    #[test]
    fn replay_skips_only_entries_before_snapshot_tick() {
        let shard = shard_id(0, 0);
        // 真快照:建会话注册分片跑 20 拍后取(保证 register_snapshot 可解码)。
        let mut original = Session::new(session_cfg());
        original.register(physics_sim::ShardSim::new(
            sim_cfg(shard),
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
        log.append(log_entry(20, 1, shard)); // == 快照拍:必须回放(不丢)
        log.append(log_entry(21, 2, shard));

        let mut replayed =
            replay(&snapshot, &log, session_cfg(), registry_factory).expect("快照自有必可恢复");
        assert_eq!(replayed.tick_now(), Tick(20), "start_tick 对齐快照拍");

        // 只回放 20/21:send 的 next_sequence 不受重放条目干扰(从 0 起)。
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
            sim_cfg(shard),
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
            sim_cfg(shard),
            registry_factory(),
        ));
        for _ in 0..3 {
            original.tick();
        }
        let snapshot = original.global_snapshot();
        assert_eq!(snapshot.tick, Tick(3));

        // 命令在拍 3 末投递(盖拍 3,拍 3 是下一拍)。重放窗口 >= 快照拍:
        // 盖拍 3 的条目(==)也回放——与生产时序一致(checkpoint 后、下一拍前
        // 到达的命令不再丢失)。原线再跑 2 拍(拍 3/4)→ 观察者命令在拍 3 应用。
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
        assert_eq!(e1.tick, Tick(3), "命令盖拍 3(== 快照拍,>= 回放)");
        for _ in 0..2 {
            original.tick();
        }
        assert_eq!(
            original.observers(),
            vec![(InputSource(1), p2)],
            "原线观察者表:同 source 两次移动 = 覆盖(表 = BTreeMap<source, pos>)"
        );
        original.shutdown();

        // 重放:同快照 + 同日志 → 条目全部重放(tick 3 >= 3),观察者表再生。
        let mut log = CommandLog::new();
        log.append(e1);
        log.append(e2);
        log.append(e3);
        let mut replayed =
            replay(&snapshot, &log, session_cfg(), registry_factory).expect("快照自有必可恢复");
        assert!(
            replayed.observers().is_empty(),
            "重放条目入收件箱,尚未 tick:表暂空(与原线拍 3 前一致)"
        );
        replayed.tick(); // 执行拍 3:观察者条目到期 → 表再生。
        assert_eq!(
            replayed.observers(),
            vec![(InputSource(1), p2)],
            "重放后观察者表与原线一致(SetObserverPosition → submit_observer 路由)"
        );
        replayed.shutdown();
    }
}
