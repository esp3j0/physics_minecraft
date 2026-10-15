//! AC8 回放端到端集成测试(任务书 04):中途快照 → 原线继续 → 重放 → 终态位级一致。
//!
//! 场景固定(任务书 04):2 分片 (0,0)/(1,0) 带地面 chunk(照 two_shards.rs 帮助函数,
//! 凝固桥关闭);e1 于 x=110 出生、linvel=(60,0,0)(dt=1/60 → 恰 1 m/tick);
//! 命令时间线(全部经 `session.send` 并 append 进 `CommandLog`,决策 5:`send` 盖
//! `apply_at = tick_now()`,所以"拍 N 的命令"在 `tick_now == N` 时发送):
//! 拍 1 SpawnDynamic(e1)、拍 15 SpawnDynamic(静止 e2,凑双分片多实体)、
//! 拍 25 Detonate(两实体发送时刻的中点,重放含力学期)。
//! 原线跑到拍 20 取 `global_snapshot` → 继续到拍 40;重放从快照拍对齐续跑到拍 40,
//! 终态逐实体位级 `==`(EntityState 的 PartialEq,不许近似)。
//!
//! 与任务书估数的差异(以源码为准,任务书原文"约拍 19 前后过界 → 迁移发生在快照
//! 之后"):按 `Session::send` 盖章语义 + worker `apply_prefix → step` 顺序
//! (crates/shard/src/scheduler.rs:33-44),出生拍即步进,110 → 本体区界 128.0 整
//! 18 m、1 m/tick → 实测(本测试取证):**第 18 拍**盖章入队并消失、**第 19 拍**
//! 在 (1,0) 重建,**早于**拍 20 快照。快照因此完整携带迁移后状态(无在途信封可丢);
//! 重放窗 [20,40) 内信封链路的忠实性以"两线逐拍采样/摘要完全一致"验证,另在本
//! 测试内补一个快照拍 10(迁移前)的同场景变体,验证信封流由重放自然重生成
//! (log.rs:8"信封/事件/迁移不落日志",README 决策 4)。

use messaging::SimCommand;
use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
use shard::{SchedulerConfig, TickSummary};
use sim_api::log::CommandLog;
use sim_api::replay::replay;
use sim_api::session::{Session, SessionConfig};
use sim_api::snapshot::GlobalSnapshot;
use storage::mem::MemCheckpointStore;
use storage::traits::CheckpointStore;
use types::glam::Vec3;
use types::{
    BlockId, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, Tick,
    WorldId,
};
use voxel::{BlockRegistry, Chunk, SimpleRegistry};

/// 快照拍(任务书固定)。
const SNAPSHOT_TICK: u64 = 20;
/// 终态拍(任务书固定):执行第 0..39 拍,结束 `tick_now == Tick(40)`。
const FINAL_TICK: Tick = Tick(40);
/// 命令时间线拍号(任务书固定)。
const SPAWN_E1_TICK: u64 = 1;
const SPAWN_E2_TICK: u64 = 15;
const DETONATE_TICK: u64 = 25;
/// 信封流变体的快照拍:在迁移(实测拍 18/19)之前,重放窗内必须重生成信封流。
const PREMIGRATION_SNAPSHOT_TICK: u64 = 10;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

/// 测试用实体(命令显式给定身份,与 two_shards 同风格)。
fn entity(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

/// 顶层实心面在 world y = top_ly - 64 的地形 chunk(照 two_shards.rs 帮助函数)。
fn ground_chunk(top_ly: usize) -> Chunk {
    let mut c = Chunk::air();
    for y in 0..top_ly {
        for x in 0..16 {
            for z in 0..16 {
                c.set(x, y, z, BlockId(1));
            }
        }
    }
    c
}

/// 带地面 chunk 的分片,凝固桥关闭(任务书 04:照 two_shards.rs 帮助 + 关桥)。
/// chunk 全高 [-64,319],top_ly=200 → 顶面 world y = 136。
fn ground_sim(shard_x: i32, chunk_xs: [i32; 2]) -> ShardSim {
    let mut cfg = ShardSimConfig::new(shard_id(shard_x, 0));
    cfg.solidify = SolidifyConfig {
        enabled: false,
        ..SolidifyConfig::default()
    };
    let mut s = ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()));
    for cx in chunk_xs {
        s.load_chunk(ChunkPos::new(cx, 0), ground_chunk(200));
    }
    s
}

/// 两条线共用的会话配置(任务书:重放用同 SessionConfig)。
fn session_cfg() -> SessionConfig {
    SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    }
}

/// registry 工厂:快照不绑注册表实例,恢复时逐分片新造(README 决策 2)。
fn registry_factory() -> Box<dyn BlockRegistry> {
    Box::new(SimpleRegistry::builtin())
}

/// SpawnDynamic 命令:e1 出生 x=110(分片 (0,0) 本体区内)、地面顶面(136)上方
/// 10 m、60 m/s 向 +x(dt=1/60 → 恰 1 m/tick);e2 静止于 x=120(凑双分片多实体)。
/// 40 拍自由落体仅下落 ~2.1 m,飞行段不接触地形。
fn spawn_command(id: EntityId, translation: Vec3, linvel: Vec3) -> SimCommand {
    SimCommand::SpawnDynamic {
        entity: id,
        state: EntityState {
            translation,
            linvel,
            angvel: Vec3::ZERO,
            ..EntityState::default()
        },
        desc: EntityDesc::default(),
    }
}

/// 某拍的实体采样(源/目的片里 e1 + 源片里 e2)。
#[derive(Debug, PartialEq)]
struct Sample {
    src_e1: Option<EntityState>,
    dst_e1: Option<EntityState>,
    src_e2: Option<EntityState>,
}

/// 终态条目:分片 × 实体 × 状态(不在片 = None),按 ShardId 升序展开。
#[derive(Debug, PartialEq)]
struct FinalEntity {
    shard: ShardId,
    entity: EntityId,
    state: Option<EntityState>,
}

/// 一条线的完整运行记录(原线或重放线;每次调用独立构造,无共享可变状态)。
struct Line {
    /// 每拍执行完后的采样/摘要(重放线自快照拍起;绝对拍号 = 下标 + 起始拍)。
    samples: Vec<Sample>,
    summaries: Vec<TickSummary>,
    /// samples[0] 的绝对拍号(原线 = 0,重放线 = 快照拍)。
    offset: u64,
    /// 原线的命令日志;重放线不发送新命令,恒为空日志。
    log: CommandLog,
    final_entities: Vec<FinalEntity>,
    shard_ids: Vec<ShardId>,
    tick_now: Tick,
}

impl Line {
    fn sample_of(session: &Session) -> Sample {
        let (src, dst) = (shard_id(0, 0), shard_id(1, 0));
        Sample {
            src_e1: session.entity_state(src, entity(1)),
            dst_e1: session.entity_state(dst, entity(1)),
            src_e2: session.entity_state(src, entity(2)),
        }
    }

    fn final_entities_of(session: &Session) -> Vec<FinalEntity> {
        let mut out = Vec::new();
        for shard in session.shard_ids() {
            for id in [entity(1), entity(2)] {
                out.push(FinalEntity {
                    shard,
                    entity: id,
                    state: session.entity_state(shard, id),
                });
            }
        }
        out
    }

    /// `migrations_admitted > 0` 的拍号(升序)。
    fn admitted_ticks(&self) -> Vec<Tick> {
        self.summaries
            .iter()
            .filter(|s| s.migrations_admitted > 0)
            .map(|s| s.tick)
            .collect()
    }

    /// e1 从源片 Some→None 的拍(消失拍;绝对拍号)。
    fn vanish_tick(&self) -> u64 {
        self.samples
            .windows(2)
            .position(|w| w[0].src_e1.is_some() && w[1].src_e1.is_none())
            .expect("e1 必须先从源片消失") as u64
            + 1
            + self.offset
    }

    /// e1 首次在目的片出现的拍(绝对拍号)。
    fn appear_tick(&self) -> u64 {
        self.samples
            .iter()
            .position(|s| s.dst_e1.is_some())
            .expect("e1 必须随后在目的片出现") as u64
            + self.offset
    }
}

/// `send` 并 append 进日志(任务书:命令时间线全部经 session.send 并进 CommandLog)。
fn send_log(session: &mut Session, log: &mut CommandLog, shard: ShardId, command: SimCommand) {
    let entry = session.send(shard, command).expect("收件箱未满");
    log.append(entry);
}

/// 跑原线:注册两分片 → 按固定时间线 `send` + `tick` 到拍 40;
/// `snapshot_at` 命中时在两拍之间取 `global_snapshot`(快照是拍间只读操作)。
fn run_original_line(snapshot_at: Option<Tick>) -> (Line, Option<GlobalSnapshot>) {
    let (src, dst) = (shard_id(0, 0), shard_id(1, 0));
    let mut session = Session::new(session_cfg());
    // 地面:分片 (0,0) 带 chunk x∈{6,7},分片 (1,0) 带 chunk x∈{8,9}
    // (覆盖飞行走廊 x∈[96,160),z=10 行);照 two_shards.rs 先 load_chunk 再 register。
    session.register(ground_sim(0, [6, 7]));
    session.register(ground_sim(1, [8, 9]));

    let mut log = CommandLog::new();
    let mut samples = Vec::new();
    let mut summaries = Vec::new();
    let mut snapshot = None;
    while session.tick_now() < FINAL_TICK {
        let at = session.tick_now();
        if Some(at) == snapshot_at {
            snapshot = Some(session.global_snapshot());
        }
        // 拍 N 的命令在 tick_now == N 时发送(send 盖 apply_at = N,本次 tick() 生效)。
        if at.0 == SPAWN_E1_TICK {
            send_log(
                &mut session,
                &mut log,
                src,
                spawn_command(
                    entity(1),
                    Vec3::new(110.0, 146.0, 10.0),
                    Vec3::new(60.0, 0.0, 0.0),
                ),
            );
        }
        if at.0 == SPAWN_E2_TICK {
            send_log(
                &mut session,
                &mut log,
                src,
                spawn_command(entity(2), Vec3::new(120.0, 146.0, 10.0), Vec3::ZERO),
            );
        }
        if at.0 == DETONATE_TICK {
            // 爆心 = 发送时刻两实体位置中点(两线状态位级一致 → 重放中点同值)。
            let p1 = session
                .entity_state(dst, entity(1))
                .expect("拍 25:e1 已迁入 (1,0)")
                .translation;
            let p2 = session
                .entity_state(src, entity(2))
                .expect("拍 25:e2 在 (0,0)")
                .translation;
            let center = (p1 + p2) * 0.5;
            // 投给 e1 所在片 (1,0):半径覆盖 e1(距中点 ~7 m),重放含力学期。
            send_log(
                &mut session,
                &mut log,
                dst,
                SimCommand::Detonate {
                    center,
                    power: 12.0,
                    radius: 16.0,
                },
            );
        }
        summaries.push(session.tick());
        samples.push(Line::sample_of(&session));
    }
    let line = Line {
        samples,
        summaries,
        offset: 0,
        log,
        final_entities: Line::final_entities_of(&session),
        shard_ids: session.shard_ids(),
        tick_now: session.tick_now(),
    };
    session.shutdown();
    (line, snapshot)
}

/// 跑重放线:`replay` 快照恢复 + 原章重放(tick > 快照拍的命令,决策 8),
/// 续跑到拍 40,记录同构采样。
fn run_replay_line(snapshot: &GlobalSnapshot, log: &CommandLog) -> Line {
    let mut session =
        replay(snapshot, log, session_cfg(), registry_factory).expect("自有快照必可恢复");
    assert_eq!(
        session.tick_now(),
        snapshot.tick,
        "重放线自快照拍对齐续跑(决策 6)"
    );
    let mut samples = Vec::new();
    let mut summaries = Vec::new();
    while session.tick_now() < FINAL_TICK {
        summaries.push(session.tick());
        samples.push(Line::sample_of(&session));
    }
    let line = Line {
        samples,
        summaries,
        offset: snapshot.tick.0,
        log: CommandLog::new(),
        final_entities: Line::final_entities_of(&session),
        shard_ids: session.shard_ids(),
        tick_now: session.tick_now(),
    };
    session.shutdown();
    line
}

/// AC8 主链路:原线到拍 20 快照 → 继续到拍 40;重放(快照+日志,同配置,
/// SimpleRegistry 工厂)到拍 40 → 逐分片逐实体 EntityState 位级 `==`;
/// tick_now 相等;shard_ids 相等。
#[test]
fn ac8_final_state_is_bitwise_identical_after_replay() {
    let (orig, snapshot) = run_original_line(Some(Tick(SNAPSHOT_TICK)));
    let snapshot = snapshot.expect("拍 20 必已截取快照");
    assert_eq!(snapshot.tick, Tick(SNAPSHOT_TICK));
    assert_eq!(orig.log.len(), 3, "固定时间线:3 条外部命令");

    let replayed = run_replay_line(&snapshot, &orig.log);
    assert_eq!(
        replayed.final_entities, orig.final_entities,
        "AC8:两线到拍 40 终态逐实体位级相等(==,不许近似)"
    );
    assert_eq!(replayed.tick_now, orig.tick_now);
    assert_eq!(replayed.tick_now, FINAL_TICK);
    assert_eq!(replayed.shard_ids, orig.shard_ids, "ownership ≡ 键集");
    // 非退化保障:e1 已在 (1,0) 存活到终态(快照确实携带迁移后实体)。
    assert!(
        orig.final_entities
            .iter()
            .any(|f| f.entity == entity(1) && f.state.is_some()),
        "场景必须把 e1 带到终态"
    );
}

/// 信封流忠实重建:
/// (a) 固定快照(拍 20)——实测迁移在拍 18/19 完成、快照完整携带,重放窗
///     [20,40) 内两线逐拍采样与逐拍 TickSummary 完全一致(含 Detonate 力学期);
///     原线自身的信封链路段式与 two_shards.rs 一致(produced == vanish、
///     appear == produced + 1,且均在快照拍前完成)。
/// (b) 迁移前快照变体(拍 10)——迁移落在重放窗内,信封不落日志(log.rs:8),
///     必须由重放自然重生成:两线消失拍/出现拍/盖章拍与逐拍采样完全一致。
#[test]
fn replay_reproduces_migration_envelope_flow() {
    // (a) 固定场景:重放窗内逐拍一致。
    let (orig, snapshot) = run_original_line(Some(Tick(SNAPSHOT_TICK)));
    let snapshot = snapshot.expect("拍 20 必已截取快照");
    let replayed = run_replay_line(&snapshot, &orig.log);

    let produced = orig.admitted_ticks();
    let vanish = orig.vanish_tick();
    let appear = orig.appear_tick();
    assert_eq!(produced.len(), 1, "场景只有一个实体一次迁移:{produced:?}");
    assert_eq!(
        vanish, produced[0].0,
        "迁移意图在源片删除实体的当拍盖章入队"
    );
    assert_eq!(appear, vanish + 1, "信封在 produced+1 拍成为目的片前缀");
    assert!(
        appear <= SNAPSHOT_TICK,
        "以源码为准:迁移(拍 {vanish}/{appear})早于快照拍 {SNAPSHOT_TICK},\
         快照完整携带迁移后状态(任务书估数差异见模块头注释)"
    );
    // 两线逐拍采样/摘要完全一致(0 号偏移 = 快照拍)。
    let offset = SNAPSHOT_TICK as usize;
    for (i, (o, r)) in orig.samples[offset..]
        .iter()
        .zip(&replayed.samples)
        .enumerate()
    {
        assert_eq!(o, r, "重放窗内绝对拍 {} 采样位级一致", offset + i);
    }
    for (i, (o, r)) in orig.summaries[offset..]
        .iter()
        .zip(&replayed.summaries)
        .enumerate()
    {
        assert_eq!(o, r, "重放窗内绝对拍 {} TickSummary 一致", offset + i);
    }
    assert!(
        replayed.admitted_ticks().is_empty(),
        "固定快照下重放窗内无迁移可发生(e1 已在 (1,0) 且不再越界)"
    );

    // (b) 迁移前快照:信封流必须被重放重建(场景其余全同)。
    let (orig2, snapshot2) = run_original_line(Some(Tick(PREMIGRATION_SNAPSHOT_TICK)));
    let snapshot2 = snapshot2.expect("拍 10 必已截取快照");
    let replayed2 = run_replay_line(&snapshot2, &orig2.log);
    let produced2 = orig2.admitted_ticks();
    assert!(
        produced2[0].0 > PREMIGRATION_SNAPSHOT_TICK,
        "变体前提:迁移落在重放窗内(实测 {}",
        produced2[0].0
    );
    assert_eq!(
        replayed2.admitted_ticks(),
        produced2,
        "重放逐拍盖章入队与原线一致(信封流重生成)"
    );
    assert_eq!(
        replayed2.vanish_tick(),
        orig2.vanish_tick(),
        "e1 从源片消失的拍完全一致"
    );
    assert_eq!(
        replayed2.appear_tick(),
        orig2.appear_tick(),
        "e1 在目的片出现的拍完全一致"
    );
    let offset2 = PREMIGRATION_SNAPSHOT_TICK as usize;
    for (i, (o, r)) in orig2.samples[offset2..]
        .iter()
        .zip(&replayed2.samples)
        .enumerate()
    {
        assert_eq!(o, r, "变体窗内绝对拍 {} 采样位级一致", offset2 + i);
    }
    assert_eq!(
        replayed2.final_entities, orig2.final_entities,
        "变体终态同样位级一致"
    );
}

/// checkpoint 经 storage 落盘再回放:`snapshot.to_bytes()` → MemCheckpointStore
/// append/read → from_bytes → replay → 终态仍位级一致(blob 过真实存储路径)。
#[test]
fn checkpoint_via_storage_roundtrips_into_replay() {
    let (orig, snapshot) = run_original_line(Some(Tick(SNAPSHOT_TICK)));
    let snapshot = snapshot.expect("拍 20 必已截取快照");

    let mut store = MemCheckpointStore::new();
    let blob = snapshot.to_bytes();
    let id = store
        .append(snapshot.tick, &blob)
        .expect("内存存储 append 不会失败");
    let (tick, read_back) = store
        .read(id)
        .expect("刚 append 必可读")
        .expect("id 必存在");
    assert_eq!(tick, snapshot.tick, "checkpoint 拍号随 blob 存储");
    assert_eq!(read_back, blob, "blob 原样往返(存储不解释内容)");

    let decoded = GlobalSnapshot::from_bytes(&read_back).expect("自有编码必可解码");
    assert_eq!(decoded, snapshot, "快照经存储路径往返逐字段相等");

    let replayed = run_replay_line(&decoded, &orig.log);
    assert_eq!(
        replayed.final_entities, orig.final_entities,
        "blob 过 CheckpointStore 真实路径后终态仍位级一致"
    );
}

/// 命令日志字节往返:`log.to_bytes()` → `from_bytes` → 用解码后的日志重放 →
/// 终态仍位级一致。
#[test]
fn log_bytes_roundtrip_replays_identically() {
    let (orig, snapshot) = run_original_line(Some(Tick(SNAPSHOT_TICK)));
    let snapshot = snapshot.expect("拍 20 必已截取快照");

    let bytes = orig.log.to_bytes();
    let decoded = CommandLog::from_bytes(&bytes).expect("自有编码必可解码");
    assert_eq!(decoded, orig.log, "日志经字节往返逐字段相等");

    let replayed = run_replay_line(&snapshot, &decoded);
    assert_eq!(
        replayed.final_entities, orig.final_entities,
        "解码日志重放与原线终态位级一致"
    );
}

/// AC8 对照基线:同场景两条原线(不做快照/重放)逐拍采样、逐拍摘要、终态
/// 位级一致——不稳定的话上面的位级断言无意义。
#[test]
fn determinism_baseline_two_fresh_runs() {
    let (a, _) = run_original_line(None);
    let (b, _) = run_original_line(None);
    assert_eq!(a.samples, b.samples, "逐拍采样位级一致");
    assert_eq!(a.summaries, b.summaries, "逐拍 TickSummary 位级一致");
    assert_eq!(a.final_entities, b.final_entities, "终态位级一致");
    assert_eq!(a.tick_now, b.tick_now);
    // 非退化保障:确定性比较的意义建立在场景确实发生迁移之上。
    assert!(a.appear_tick() <= 40, "场景必须发生迁移");
}
