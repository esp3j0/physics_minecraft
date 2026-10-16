//! AC8 长程(600 拍)回放测试(任务书 01;README 决策 4:600 拍、snapshot@300、
//! 重放到位级 ==;§9.11 checkpoint 周期即 600)。
//!
//! 场景 = 既有 AC8 场景(`tests/replay.rs`,2 分片/出生/穿越/爆炸)拉伸到 600 拍:
//! - 2 分片 (0,0)/(1,0),地面 chunk 铺满走廊 x∈[0,256)(ground_chunk(200),
//!   顶面 world y=136),凝固桥关闭;
//! - 拍 1 SpawnDynamic(e1):(110,146,10),linvel=(24,0,0)——自由落体 ~83 拍,
//!   拍 ~46 空中越过 x=128(迁移先于快照,快照完整携带迁移后态);触地后摩擦
//!   减速停在 (1,0) 本体区内;
//! - 拍 15 SpawnDynamic(e2):(100,146,10) 静止——在 e1 出生点(x=110)后方
//!   (e1 只向 +x 飞,永不回头,两实体零接触),自由落体拍 ~45 落在 (0,0)
//!   地面保持静止;
//! - 拍 320 Detonate(> 快照拍 300 → 走"快照 + tick>300 日志"重放路径):
//!   爆心 = 两实体采样位置中点(两线位级一致 → 重放取同值),
//!   power=12、radius=0.6×|p1-p2| → 两实体各获 12×(1-0.5/0.6)=2 m/s 反向冲量
//!   (demo D 对称),扰动后仍在各自本体区内(不产生新迁移)。
//!
//! 原线跑到拍 300 取 `global_snapshot` → 继续到拍 600;重放(快照 + tick>300
//! 日志,同 SessionConfig,SimpleRegistry 工厂)从拍 300 对齐续跑到拍 600,
//! 重放窗内逐拍采样位级一致 + 终态逐实体位级 `==`(不许近似)。
//!
//! 非退化保障:① e1 确实迁移(消失拍/出现拍差 1,且早于快照拍);
//! ② 爆炸确实把静止的 e2 炸动(拍 320 前 |v|<0.1,拍 321 起出现 |v|>0.5);
//! ③ 终态两实体分别在 (1,0)/(0,0) 存活。

use messaging::SimCommand;
use physics_sim::{ShardSim, ShardSimConfig, SolidifyConfig};
use shard::{SchedulerConfig, TickSummary};
use sim_api::log::CommandLog;
use sim_api::replay::replay;
use sim_api::session::{Session, SessionConfig};
use sim_api::snapshot::GlobalSnapshot;
use types::glam::Vec3;
use types::{
    BlockId, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, Tick,
    WorldId,
};
use voxel::{BlockRegistry, Chunk, SimpleRegistry};

/// 中途快照拍(README 决策 4)。
const SNAPSHOT_TICK: u64 = 300;
/// 终态拍:执行第 0..599 拍,结束 `tick_now == Tick(600)`。
const FINAL_TICK: Tick = Tick(600);
/// 命令时间线拍号(拉伸版,依据见模块头注释)。
const SPAWN_E1_TICK: u64 = 1;
const SPAWN_E2_TICK: u64 = 15;
const DETONATE_TICK: u64 = 320;
/// 爆炸参数:radius = 两实体间距 × 此系数(> 0.5 保证双方都在半径内)。
const BLAST_RADIUS_SCALE: f32 = 0.6;
const BLAST_POWER: f32 = 12.0;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn entity(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

/// 顶层实心面在 world y = top_ly - 64 的地形 chunk(照 replay.rs/two_shards.rs 帮助函数)。
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

/// 带地面 chunk(区间 cx_start..cx_end)的分片,凝固桥关闭(replay.rs 同款)。
fn ground_sim(shard_x: i32, chunk_xs: std::ops::Range<i32>) -> ShardSim {
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

/// 每拍采样(重放窗内逐拍对照 + 非退化取证)。
#[derive(Debug, PartialEq)]
struct Sample {
    src_e1: Option<EntityState>,
    dst_e1: Option<EntityState>,
    src_e2: Option<EntityState>,
}

/// 终态条目:分片 × 实体 × 状态(replay.rs 同构)。
#[derive(Debug, PartialEq)]
struct FinalEntity {
    shard: ShardId,
    entity: EntityId,
    state: Option<EntityState>,
}

/// 一条线的完整运行记录(replay.rs 同构;每次调用独立构造)。
struct Line {
    samples: Vec<Sample>,
    summaries: Vec<TickSummary>,
    /// samples[0] 的绝对拍号(原线 = 0,重放线 = 快照拍)。
    offset: u64,
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

    /// 绝对拍 k 执行完后的采样(k ∈ [offset, offset+len))。
    fn sample_at(&self, k: u64) -> &Sample {
        &self.samples[(k - self.offset) as usize]
    }

    /// `migrations_admitted > 0` 的拍号(升序)。
    fn admitted_ticks(&self) -> Vec<Tick> {
        self.summaries
            .iter()
            .filter(|s| s.migrations_admitted > 0)
            .map(|s| s.tick)
            .collect()
    }

    /// e1 从源片 Some→None 的绝对拍。
    fn vanish_tick(&self) -> u64 {
        self.samples
            .windows(2)
            .position(|w| w[0].src_e1.is_some() && w[1].src_e1.is_none())
            .expect("e1 必须先从源片消失") as u64
            + 1
            + self.offset
    }

    /// e1 首次在目的片出现的绝对拍。
    fn appear_tick(&self) -> u64 {
        self.samples
            .iter()
            .position(|s| s.dst_e1.is_some())
            .expect("e1 必须随后在目的片出现") as u64
            + self.offset
    }
}

fn send_log(session: &mut Session, log: &mut CommandLog, shard: ShardId, command: SimCommand) {
    let entry = session.send(shard, command).expect("收件箱未满");
    log.append(entry);
}

/// 跑原线:地面铺满 x∈[0,256)(片 0 = chunk 0..8,片 1 = chunk 8..16),
/// 固定时间线到拍 600;拍 300 在两拍之间取全局快照。
fn run_original_line(snapshot_at: Option<Tick>) -> (Line, Option<GlobalSnapshot>) {
    let (src, dst) = (shard_id(0, 0), shard_id(1, 0));
    let mut session = Session::new(session_cfg());
    session.register(ground_sim(0, 0..8));
    session.register(ground_sim(1, 8..16));

    let mut log = CommandLog::new();
    let mut samples = Vec::new();
    let mut summaries = Vec::new();
    let mut snapshot = None;
    while session.tick_now() < FINAL_TICK {
        let at = session.tick_now();
        if Some(at) == snapshot_at {
            snapshot = Some(session.global_snapshot());
        }
        if at.0 == SPAWN_E1_TICK {
            send_log(
                &mut session,
                &mut log,
                src,
                spawn_command(
                    entity(1),
                    Vec3::new(110.0, 146.0, 10.0),
                    Vec3::new(24.0, 0.0, 0.0),
                ),
            );
        }
        if at.0 == SPAWN_E2_TICK {
            send_log(
                &mut session,
                &mut log,
                src,
                spawn_command(entity(2), Vec3::new(100.0, 146.0, 10.0), Vec3::ZERO),
            );
        }
        if at.0 == DETONATE_TICK {
            // 爆心 = 采样位置中点(两线位级一致 → 重放中同值,radius 同理)。
            let p1 = session
                .entity_state(dst, entity(1))
                .expect("拍 320:e1 已迁入 (1,0)")
                .translation;
            let p2 = session
                .entity_state(src, entity(2))
                .expect("拍 320:e2 在 (0,0)")
                .translation;
            let center = (p1 + p2) * 0.5;
            let radius = (p1 - p2).length() * BLAST_RADIUS_SCALE;
            assert!(
                radius > 0.0 && radius.is_finite(),
                "爆轰半径必须有效:p1={p1:?} p2={p2:?}"
            );
            // 广播语义:同一 Detonate 命令分别投给两分片(覆盖双方,各自本地求值)。
            for shard in [src, dst] {
                send_log(
                    &mut session,
                    &mut log,
                    shard,
                    SimCommand::Detonate {
                        center,
                        power: BLAST_POWER,
                        radius,
                    },
                );
            }
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

/// 跑重放线:快照恢复 + tick>300 日志原章重放,续跑到拍 600。
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

/// AC8 长程主断言:原线 600 拍(拍 300 快照)vs 重放线(快照 + tick>300 日志)
/// 到拍 600 终态逐实体位级 `==`;重放窗内逐拍采样/摘要位级一致;时间线非退化
/// (迁移先于快照、爆炸晚于快照走日志重放)。
#[test]
fn ac8_long_replay_600_ticks_bitwise() {
    let (orig, snapshot) = run_original_line(Some(Tick(SNAPSHOT_TICK)));
    let snapshot = snapshot.expect("拍 300 必已截取快照");
    assert_eq!(snapshot.tick, Tick(SNAPSHOT_TICK));
    assert_eq!(orig.log.len(), 4, "固定时间线:2 出生 + 2 广播 Detonate");
    assert_eq!(orig.tick_now, FINAL_TICK);

    // 非退化①:迁移恰好一次,先于快照拍(快照完整携带迁移后态)。
    let produced = orig.admitted_ticks();
    assert_eq!(produced.len(), 1, "场景恰一次迁移:{produced:?}");
    let vanish = orig.vanish_tick();
    let appear = orig.appear_tick();
    assert_eq!(vanish, produced[0].0, "消失拍 = 盖章入队拍");
    assert_eq!(appear, vanish + 1, "信封在 produced+1 拍重建");
    assert!(
        appear <= SNAPSHOT_TICK,
        "迁移(拍 {vanish}/{appear})必须先于快照拍 {SNAPSHOT_TICK}"
    );

    let replayed = run_replay_line(&snapshot, &orig.log);

    // 重放窗 [300,600) 逐拍采样/摘要位级一致(含 Detonate 力学期)。
    assert_eq!(replayed.samples.len(), (600 - SNAPSHOT_TICK) as usize);
    for (i, (o, r)) in orig.samples[SNAPSHOT_TICK as usize..]
        .iter()
        .zip(&replayed.samples)
        .enumerate()
    {
        assert_eq!(
            o,
            r,
            "重放窗内绝对拍 {} 采样位级一致",
            SNAPSHOT_TICK as usize + i
        );
    }
    for (i, (o, r)) in orig.summaries[SNAPSHOT_TICK as usize..]
        .iter()
        .zip(&replayed.summaries)
        .enumerate()
    {
        assert_eq!(
            o,
            r,
            "重放窗内绝对拍 {} TickSummary 一致",
            SNAPSHOT_TICK as usize + i
        );
    }

    // 终态位级 ==。
    assert_eq!(
        replayed.final_entities, orig.final_entities,
        "AC8 长程:两线到拍 600 终态逐实体位级相等(==,不许近似)"
    );
    assert_eq!(replayed.tick_now, orig.tick_now);
    assert_eq!(replayed.shard_ids, orig.shard_ids, "ownership ≡ 键集");
    // 非退化③:终态两实体分别存活于 (1,0)/(0,0)。
    assert!(
        replayed
            .final_entities
            .iter()
            .any(|f| f.entity == entity(1) && f.shard == shard_id(1, 0) && f.state.is_some()),
        "e1 必须经迁移在 (1,0) 存活到终态"
    );
    assert!(
        replayed
            .final_entities
            .iter()
            .any(|f| f.entity == entity(2) && f.shard == shard_id(0, 0) && f.state.is_some()),
        "e2 必须在 (0,0) 存活到终态"
    );
}

/// 非退化②:爆炸真实改变了静止的 e2——拍 319(执行完)近似静止(<0.1 m/s),
/// 拍 320(Detonate 生效拍)起线速度出现 >0.5 m/s 的窗口。若爆炸不生效,
/// e2 落地(~拍 100)后就再无任何速度来源。
#[test]
fn ac8_long_blast_actually_moved_e2() {
    let (orig, _) = run_original_line(None);
    let before = orig.sample_at(DETONATE_TICK - 1);
    let e2_before = before.src_e2.expect("拍 319:e2 在 (0,0)");
    assert!(
        e2_before.linvel.length() < 0.1,
        "爆炸前 e2 必须已静止(取样才有意义):v={}",
        e2_before.linvel.length()
    );
    let mut max_after = 0.0f32;
    for k in DETONATE_TICK..DETONATE_TICK + 30 {
        if let Some(s) = orig.sample_at(k).src_e2 {
            max_after = max_after.max(s.linvel.length());
        }
    }
    assert!(
        max_after > 0.5,
        "爆炸必须把 e2 炸动(30 拍窗内 |v|max = {max_after} m/s ≤ 0.5 → 场景退化)"
    );
}
