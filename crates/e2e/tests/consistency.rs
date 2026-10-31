//! REQ-005 任务 04:AC-C1~C9 一致性验收套件(qwen 执笔,与任务 00–03 实现交叉)。
//!
//! 真实性红线(任务书):真驱动(`app::driver::run`)、真持久层(真 redb 或
//! 任务书既许的 `SharedCheckpoints`/`SharedChunks` 内存镜像 + `ChunkProvider`
//! 真路径)、真 TCP(127.0.0.1:0);不 mock 被验收层。断言全部指向行为结果:
//! 位级 `==`、给定容差(<1e-4 m,写进消息)、精确集合、逐字节相等;不写
//! "不 panic/非空/计数>=1"式弱断言(计数字段仅作为"回归场景确实运行"的旁证,
//! 主断言永远是位级/集合)。
//!
//! 装置说明:
//! - session/scheduler 级测试复刻驱动编排(`acg5_orchestrate` 同款:取激活
//!   请求 → provider/生成器取数 → `apply_chunk_load`,owner 惰性 ensure),
//!   被测层(Session/Scheduler/Router)一律真身;
//! - driver 级测试用 `#[tokio::test(start_paused = true)]` + `advance_ticks`
//!   (world_loading.rs 同款虚拟时钟),网络测试照 `network.rs` harness 内嵌
//!   真 TCP 客户端;
//! - 物理对照统一零重力纯弹道(AC3/explosion_equivalence 口径)或
//!   migration.rs 的单世界 greedy_colliders 参考(容差 1e-4 m)。
//!
//! `acc8_workspace_regression_and_viewer_smoke` 的 `cargo test --workspace`
//! / clippy / fmt 三段属编排层门禁(任务书完成定义),本测试体只承担
//! viewer::client 连真服务器收 Welcome/ChunkData 的冒烟部分。

use std::collections::{BTreeMap, BTreeSet};
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use app::config::Config;
use app::driver::{CheckpointBundle, DriverArgs, DriverCommand, run};
use messaging::{CrossEvent, InputSource, SimCommand};
use physics_sim::{
    ActivationConfig as SimActivationConfig, ShardSim, ShardSimConfig, SolidifyConfig,
};
use shard::{Scheduler, SchedulerConfig};
use sim_api::log::CommandLog;
use sim_api::replay::replay_world;
use sim_api::session::{Session, SessionConfig};
use storage::mem::MemCheckpointStore;
use storage::provider::ChunkProvider;
use storage::redb::RedbStore;
use storage::traits::{CheckpointId, CheckpointStore, ChunkKey, ChunkStore, StoreError};
use storage::worker::StoreWorker;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use types::glam::{Quat, Vec3};
use types::{
    BlockId, BlockPos, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos,
    Tick, WorldId, WorldSeed,
};
use voxel::{BlockRegistry, Chunk, SimpleRegistry, Superflat, WorldGenerator};

// ---------------------------------------------------------------------------
// 通用装置(world_loading.rs / network.rs / viewer_client.rs 同款)
// ---------------------------------------------------------------------------

/// Superflat classic 地面顶面 world y(-64 + 层厚 4)。
const GROUND_TOP_Y: f32 = types::WORLD_Y_MIN as f32 + 4.0;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn eid(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

fn key(world: u32, dim: u32, x: i32, z: i32) -> ChunkKey {
    ChunkKey {
        world: WorldId(world),
        dim: DimensionId(dim),
        pos: ChunkPos::new(x, z),
    }
}

fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(SimpleRegistry::builtin())
}

/// 共享 mem 检查点存储(测试侧读回;任务书既许装置)。
#[derive(Clone, Default)]
struct SharedCheckpoints(Arc<Mutex<MemCheckpointStore>>);

impl CheckpointStore for SharedCheckpoints {
    fn append(&mut self, at: types::Tick, blob: &[u8]) -> Result<CheckpointId, StoreError> {
        self.0.lock().expect("检查点锁毒化").append(at, blob)
    }
    fn read_latest(&self) -> Result<Option<(types::Tick, Vec<u8>)>, StoreError> {
        self.0.lock().expect("检查点锁毒化").read_latest()
    }
    fn read(&self, id: CheckpointId) -> Result<Option<(types::Tick, Vec<u8>)>, StoreError> {
        self.0.lock().expect("检查点锁毒化").read(id)
    }
}

/// 共享 mem chunk 存储(读回断言持久化内容)。
#[derive(Clone, Default)]
struct SharedChunks(Arc<Mutex<storage::mem::MemChunkStore>>);

impl ChunkStore for SharedChunks {
    fn put_batch(&mut self, batch: &[(ChunkKey, Chunk)]) -> Result<(), StoreError> {
        self.0.lock().expect("chunk 锁毒化").put_batch(batch)
    }
    fn get(&self, k: ChunkKey) -> Result<Option<Chunk>, StoreError> {
        self.0.lock().expect("chunk 锁毒化").get(k)
    }
    fn len(&self) -> Result<usize, StoreError> {
        self.0.lock().expect("chunk 锁毒化").len()
    }
    fn is_empty(&self) -> Result<bool, StoreError> {
        self.0.lock().expect("chunk 锁毒化").is_empty()
    }
}

/// 暂停时间下推进 N 拍(world_loading.rs 同款)。
async fn advance_ticks(ticks: u32) {
    let dt = Duration::from_secs_f32(1.0 / 60.0);
    for _ in 0..ticks {
        tokio::time::sleep(dt).await;
        tokio::task::yield_now().await;
    }
}

/// 解码检查点 blob → bundle(#17:走驱动的公开解码入口,v5 起带版本前缀与压缩体)。
fn decode_bundle(blob: &[u8]) -> CheckpointBundle {
    app::driver::decode_checkpoint(blob).expect("驱动自有编码必可解码")
}

/// 顶层实心面 world y = top_ly - 64 的地形 chunk(migration.rs 同款)。
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

/// 凝固桥关闭的虚空分片(零重力可参)。
fn void_sim(x: i32, z: i32, zero_g: bool) -> ShardSim {
    let mut cfg = ShardSimConfig::new(shard_id(x, z));
    if zero_g {
        cfg.gravity = Vec3::ZERO;
    }
    cfg.solidify = SolidifyConfig {
        enabled: false,
        ..SolidifyConfig::default()
    };
    ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
}

/// 零重力 + 凝固桥直通(位移门 0;scheduler.rs `spawn_resting_entity` 同款
/// ——静止体 20 拍内必然凝固,走真实凝固 → Emergent 路径)。
fn solidifying_sim(x: i32, z: i32) -> ShardSim {
    let mut cfg = ShardSimConfig::new(shard_id(x, z));
    cfg.gravity = Vec3::ZERO;
    cfg.solidify = SolidifyConfig {
        enabled: true,
        displacement_gate_m: 0.0,
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

/// 出生态构造(固定位置/速度,零重力弹道场景共用)。
fn born(x: f32, y: f32, z: f32, v: Vec3) -> EntityState {
    EntityState {
        translation: Vec3::new(x, y, z),
        rotation: Quat::IDENTITY,
        linvel: v,
        angvel: Vec3::ZERO,
    }
}

// ---------------------------------------------------------------------------
// AC-C1 / AC-C9:chunk 归属、区域、holder 表(调度器真身 + 编排复刻)
// ---------------------------------------------------------------------------

/// AC-C1(任务书 04 表):两分片 + 一观察者贴边界,400 拍(盖冷却 180 +
/// 预算 2/拍)逐拍经**真实编排路径**装载。断言意图(全部精确集合/唯一性):
/// ① 装载候选只落 core∪margin(D4 区域不变量);
/// ② holder 表精确:每个 (owner, chunk) 的 holder 集合 == 手算请求者集合,
///    holder 只出现在自己 core∪margin(扩张副本语义),深处 core chunk 恰
///    owner 单 holder——同一 chunk 绝不被两片当本体重复装载;
/// ③ owner 唯一:`owner_of` 纯函数与 chunk 坐标逐点吻合;
/// ④ 同一 (shard, chunk) 装载落实恰一次(重复装载 = 簿记 bug)。
#[test]
fn acc1_chunk_ownership_region_and_holders() {
    let mut sched = Scheduler::new(SchedulerConfig {
        worker_count: 2,
        ..SchedulerConfig::default()
    });
    sched.register(void_sim(0, 0, true));
    sched.register(void_sim(1, 0, true));
    // 观察者贴边界 (126,0,64):两片激活中心共用,装载候选区在 x=7/8 重叠。
    sched.set_observers(vec![Vec3::new(126.0, 0.0, 64.0)]);

    let mut requesters: BTreeMap<ChunkPos, BTreeSet<ShardId>> = BTreeMap::new();
    let mut applied: BTreeMap<(ShardId, ChunkPos), u32> = BTreeMap::new();
    for _ in 0..400 {
        sched.tick();
        for (shard, decisions) in sched.take_activation_requests() {
            let region = types::ChunkRegion::core(shard).expanded(1);
            for cp in decisions.chunks_to_load {
                // ① 区域不变量(编排侧独立复查,不依赖调度器内部校验)。
                assert!(
                    region.contains(cp),
                    "分片 {shard:?} 请求区域外 chunk {cp:?}(D4)"
                );
                sched.apply_chunk_load(shard, cp, ground_chunk(4));
                *applied.entry((shard, cp)).or_insert(0) += 1;
                requesters.entry(cp).or_default().insert(shard);
            }
        }
    }

    // ④ 同一 (shard, chunk) 恰一次装载落实(loaded 集合过滤生效,无重复)。
    for ((shard, cp), n) in &applied {
        assert_eq!(*n, 1, "{shard:?} 重复装载 {cp:?} 共 {n} 次(簿记被破坏)");
    }

    // 手算精确请求集(观察者 (126,0,64)、加载半径 32、margin 1 区域过滤、
    // 预算 2/拍 × 400 拍 + 冷却窗重火 → 每个够近且区域内的候选必被装载):
    // chunk 中心 (16x+8, 16z+8) 水平距观察者 ≤ 32 的 12 个候选中,区域过滤后
    // 两片各得 6 个;x=7/8 列是 margin 重叠区(双 holder),x=6 属 (0,0) 深处
    // core、x=9 属 (1,0) 深处 core(单 holder)。
    let s0 = shard_id(0, 0);
    let s1 = shard_id(1, 0);
    let mut expected: BTreeMap<ChunkPos, BTreeSet<ShardId>> = BTreeMap::new();
    for z in 2..=5 {
        expected.insert(ChunkPos::new(7, z), [s0, s1].into_iter().collect());
        expected.insert(ChunkPos::new(8, z), [s0, s1].into_iter().collect());
    }
    for z in [3, 4] {
        expected.insert(ChunkPos::new(6, z), [s0].into_iter().collect());
        expected.insert(ChunkPos::new(9, z), [s1].into_iter().collect());
    }
    assert_eq!(
        requesters, expected,
        "装载分布必须与手算精确集合全等(深处 core chunk 恰单请求,双请求只允许在 margin 重叠列)"
    );

    // ② holder 表 = 请求者集合(精确),且每个 holder 持该 chunk 都在自己
    //   core∪margin 内;owner 的 core chunk 从不作为「别人的 core」出现。
    for (cp, reqs) in &expected {
        let owner = ShardId::new(WorldId(0), DimensionId(0), cp.shard_of());
        let holders = sched.holders_of(owner, *cp);
        assert_eq!(
            &holders, reqs,
            "chunk {cp:?} 的 holder 集合必须恰为请求者集 {reqs:?},实得 {holders:?}"
        );
        for h in &holders {
            assert!(
                types::ChunkRegion::core(*h).expanded(1).contains(*cp),
                "holder {h:?} 持有越区副本 {cp:?}(D2/D4 泄漏)"
            );
            // ③ owner 唯一:同一 chunk 只有一个 owner(纯函数逐点复核)。
            assert_eq!(
                Scheduler::owner_of(*h, *cp),
                owner,
                "owner 判定漂移:{cp:?} 经 {h:?} 上下文算出非 owner"
            );
        }
        // 本体重复装载 = 0:把 cp 当自家 core 的 holder 恰 = {owner}。
        let core_holders: BTreeSet<ShardId> = holders
            .iter()
            .filter(|h| types::ChunkRegion::core(**h).contains(*cp))
            .copied()
            .collect();
        assert_eq!(
            core_holders,
            BTreeSet::from([owner]),
            "chunk {cp:?} 的 core-持有者必须恰为 {{owner={owner:?}}}(本体重复 = 违规)"
        );
    }
    sched.shutdown();
}

// ---------------------------------------------------------------------------
// AC-C2:边界编辑副本一致 + 单世界对照
// ---------------------------------------------------------------------------

/// AC-C2(副本面):边界 chunk(chunk 7,owner (0,0),(1,0) 持 margin 副本)
/// 一次 `set_block` 广播:
/// ① 日志恰 2 条、dest 升序 {(0,0),(1,0)}、同 tick 同 sequence(D3 原子广播);
/// ② tick 后两片该 chunk **逐字节相等**且 == 手算修改值(Chunk Eq 全格);
/// ③ collider 同拍生效(owner 面物理探针):chunk 8 实体承托足迹 4 列
///    (world x∈{139,140} × z∈{7,8})先挖后生 → 实体落入下移一格的驻留带
///    (= 编辑拍的 `rebuild_chunk_collider` 已重建;holder 副本的 collider
///    与体素同为 chunk 数据的纯函数,②的逐字节相等 ⟹ 重建输入一致)。
#[test]
fn acc2_boundary_edit_same_tick() {
    let chunk7 = ChunkPos::new(7, 0);
    let chunk8 = ChunkPos::new(8, 0);
    let mut session = Session::new(session_cfg());
    session.register(void_sim(0, 0, false));
    session.register(void_sim(1, 0, false));
    session.apply_chunk_load(shard_id(0, 0), chunk7, ground_chunk(4));
    session.apply_chunk_load(shard_id(1, 0), chunk7, ground_chunk(4)); // margin 副本
    session.apply_chunk_load(shard_id(1, 0), chunk8, ground_chunk(4));

    // ①/② 边界编辑:chunk 7 内底层 stone → dirt(owner (0,0))。
    let edited = BlockPos::new(120, types::WORLD_Y_MIN, 8);
    let entries = session
        .set_block(shard_id(0, 0), edited, BlockId(2))
        .expect("owner 已注册,广播必成");
    assert_eq!(
        entries.iter().map(|e| e.dest).collect::<Vec<_>>(),
        vec![shard_id(0, 0), shard_id(1, 0)],
        "广播日志恰 2 条 dest 升序 {entries:?}"
    );
    assert_eq!(
        entries[0].sequence, entries[1].sequence,
        "整批只 +1:同 sequence(D3)"
    );
    session.tick();
    let mut expect7 = ground_chunk(4);
    expect7.set(8, 0, 8, BlockId(2));
    assert_eq!(
        session
            .voxel_chunk(shard_id(0, 0), chunk7)
            .expect("owner 副本在"),
        &expect7,
        "owner chunk 逐字节 == 修改值"
    );
    assert_eq!(
        session
            .voxel_chunk(shard_id(1, 0), chunk7)
            .expect("holder 副本在"),
        session.voxel_chunk(shard_id(0, 0), chunk7).expect("同上"),
        "AC-C2:边界编辑后两片该 chunk 逐字节相等"
    );

    // ③ collider 探针:chunk 8 实体承托足迹挖空 → owner (1,0) 同拍重建。
    // 足迹精确集:实体 (140,·,8) 半宽 0.5 → 占 world x∈{139,140}、z∈{7,8}
    // 的 4 根承托列(单挖 2 列会剩 z=7 行承托住实体 = 场景退化)。实体先
    // 挖后生(同拍生效),落稳必须到**下移一格**的驻留带(顶面 -61 上方;
    // 旧带 ≈ -59.5,两带互斥 → collider 同拍重建的行为证据)。
    let e = eid(1);
    for (x, z) in [(139i32, 7i32), (139, 8), (140, 7), (140, 8)] {
        session
            .set_block(shard_id(1, 0), BlockPos::new(x, -61, z), BlockId::AIR)
            .expect("owner (1,0) 广播");
    }
    session
        .send(
            shard_id(1, 0),
            SimCommand::SpawnDynamic {
                entity: e,
                state: born(140.0, -55.0, 8.0, Vec3::ZERO),
                desc: EntityDesc::default(),
            },
        )
        .expect("收件箱未满");
    for _ in 0..140 {
        session.tick();
    }
    let fallen = session
        .entity_state(shard_id(1, 0), e)
        .expect("凝固桥关:实体恒在");
    assert!(
        fallen.translation.y < GROUND_TOP_Y && fallen.translation.y > GROUND_TOP_Y - 1.6,
        "挖空后实体落入下移一格的驻留带(collider 同拍重建,顶面 -61 上方):y={}",
        fallen.translation.y
    );
    // 对照:未挖区实体仍驻留原顶面(编辑只作用于目标列,collider 未整体失效)。
    session
        .send(
            shard_id(1, 0),
            SimCommand::SpawnDynamic {
                entity: eid(2),
                state: born(134.0, -55.0, 8.0, Vec3::ZERO),
                desc: EntityDesc::default(),
            },
        )
        .expect("收件箱未满");
    for _ in 0..140 {
        session.tick();
    }
    let kept = session
        .entity_state(shard_id(1, 0), eid(2))
        .expect("实体在");
    assert!(
        kept.translation.y > GROUND_TOP_Y && kept.translation.y < GROUND_TOP_Y + 1.5,
        "未挖列实体驻留原顶面(编辑局部性):y={}",
        kept.translation.y
    );
    session.shutdown();
}

/// AC-C2(物理对照面):边界走廊双片(地面 + 扩张带重叠 + 边界 chunk 挖洞编辑)
/// vs 裸 PhysicsWorld(生产同构:逐 chunk fixed compound body):
/// 实体 v=(60,0,0) 跨片滚过挖洞 chunk,45 步后 translation 各分量差 **< 1e-4 m**
/// (容差写进消息,migration.rs AC2 同口径);迁移确实发生(非退化)。
/// F05c 返修说明(裁定"F05c 第 1 条"):洞放回**分片边界格**(chunk 7 最东格,
/// x=127,骑在分片交接面 x=128 上)。第一版测试的 `set_block` 用 (127,64,8) 是
/// **off-by-one**:局部 y = 64−(−64) = 128 = 地面上方的空气层(挖掘 no-op),
/// 而参考侧 chunk 编辑按注释挖局部 (15,127,8) = 世界格 (127,63,8) = 顶层地面格
/// ——两侧挖的从来不是同一个格。跨面口径下洞口被伸入的薄片封住,错位被 <1e-4
/// 掩盖(假绿);D15 内缩后洞是净宽 1.0 的真洞,分片侧(无洞)与参考侧(有洞)
/// 在拍 27 实体落地弹跳处分叉(诊断实测第一分叉拍 = 27,45 拍终态差
/// (9.74,−5.15,−0.19) m,与总架构师独立复跑逐位一致);两侧同格(SetBlock 改
/// (127,63,8))后 45 拍全程逐位一致(含拍 29 迁移在途),C 组对照数据见 #21
/// 回评。修复 = 消除 off-by-one(与原注释意图一致),洞仍在边界格、考察点不变;
/// 参考侧逐 chunk 分组(生产同构)保留。
#[test]
fn acc2_boundary_edit_physics_vs_single_world() {
    // 走廊 = ground_chunk(128)(顶面 world y=64,migration.rs 场景)。
    const TOP_LY: usize = 128;
    // 边界 chunk 编辑:格 (127, 63, 8)(chunk 7 局部 (15,127,8),owner (0,0))
    // 挖空——顶层地面格,分片边界格(骑分片交接面 x=128)。
    let hole = BlockPos::new(127, 63, 8);
    let mut corridor_edited: BTreeMap<ChunkPos, Chunk> = BTreeMap::new();
    let mut corridor_pristine: BTreeMap<ChunkPos, Chunk> = BTreeMap::new();
    for cx in 0..16i32 {
        let cp = ChunkPos::new(cx, 0);
        let mut g = ground_chunk(TOP_LY);
        if cx == 7 {
            g.set(15, 127, 8, BlockId::AIR);
        }
        corridor_edited.insert(cp, g.clone());
        corridor_pristine.insert(cp, ground_chunk(TOP_LY));
    }

    // --- 双片侧(Session 真身;分片各自载 core + 扩张带,chunk 7 双副本)---
    let mut session = Session::new(session_cfg());
    let mut sim0 = ShardSim::new(
        ShardSimConfig {
            solidify: SolidifyConfig {
                enabled: false,
                ..SolidifyConfig::default()
            },
            ..ShardSimConfig::new(shard_id(0, 0))
        },
        Box::new(SimpleRegistry::builtin()),
    );
    let mut sim1 = ShardSim::new(
        ShardSimConfig {
            solidify: SolidifyConfig {
                enabled: false,
                ..SolidifyConfig::default()
            },
            ..ShardSimConfig::new(shard_id(1, 0))
        },
        Box::new(SimpleRegistry::builtin()),
    );
    for cx in 0..9i32 {
        sim0.load_chunk(
            ChunkPos::new(cx, 0),
            corridor_pristine[&ChunkPos::new(cx, 0)].clone(),
        );
    }
    for cx in 7..16i32 {
        sim1.load_chunk(
            ChunkPos::new(cx, 0),
            corridor_pristine[&ChunkPos::new(cx, 0)].clone(),
        );
    }
    session.register(sim0);
    session.register(sim1);
    let e = eid(1);
    session
        .send(
            shard_id(0, 0),
            SimCommand::SpawnDynamic {
                entity: e,
                state: born(100.0, 65.5, 8.0, Vec3::new(60.0, 0.0, 0.0)),
                desc: EntityDesc::default(),
            },
        )
        .expect("收件箱未满");
    // 与 spawn 同拍的边界编辑(send_broadcast → owner+holder 同拍应用)。
    session
        .set_block(shard_id(0, 0), hole, BlockId::AIR)
        .expect("chunk 7 已双载,owner (0,0) 广播");
    let mut crossed = false;
    for k in 0..45u64 {
        session.tick();
        if session.entity_state(shard_id(1, 0), e).is_some() && !crossed {
            crossed = true;
            assert!(k >= 2, "非退化:迁移必须发生在场景中段(拍 {k})");
        }
    }
    let shard_end = session
        .entity_state(shard_id(1, 0), e)
        .expect("迁移后实体在目的片");
    session.shutdown();
    assert!(crossed, "场景必须发生跨片迁移(等价断言的对象 = 分片路径)");

    // --- 参考侧:裸 PhysicsWorld(同 gravity/dt/刚体参数;编辑后的地面按
    // 生产同构分组 = 逐 chunk 一个 fixed compound body——分片机制不引入物理
    // 差异是本测试的考察点;spawn+挖洞命令语义 = 出生前地面已含洞)。
    // F05c:参考侧逐 chunk 分组保留(生产同构);两侧同格后 45 拍全程逐位一致,
    // 迁移在途拍(实体 GONE 一拍)后两侧仍续一致——迁移丢 warm-start/接触缓存
    // 在本场景(实体空中弹道,无持续接触)不产生可见差异。
    use rapier3d::prelude::*;
    let cfg = ShardSimConfig::new(shard_id(0, 0));
    let mut phys = PhysicsWorld::new();
    phys.gravity = cfg.gravity;
    phys.integration_parameters.dt = cfg.dt;
    let registry = SimpleRegistry::builtin();
    let get = |x: i32, z: i32| corridor_edited.get(&ChunkPos::new(x, z));
    for cx in 0..16 {
        let cp = ChunkPos::new(cx, 0);
        let neighbors = physics_sim::collider_build::NeighborChunks {
            px: get(cx + 1, 0),
            nx: get(cx - 1, 0),
            pz: get(cx, 1),
            nz: get(cx, -1),
        };
        let parts: Vec<(Pose, SharedShape)> = physics_sim::collider_build::greedy_colliders(
            get(cx, 0).expect("走廊 chunk"),
            cp,
            &neighbors,
            |b| registry.is_solid(b),
        )
        .into_iter()
        .map(|b| {
            (
                Pose {
                    rotation: Quat::IDENTITY,
                    translation: b.center,
                    ..Pose::IDENTITY
                },
                SharedShape::cuboid(b.half.x, b.half.y, b.half.z),
            )
        })
        .collect();
        if parts.is_empty() {
            continue;
        }
        phys.insert(
            RigidBodyBuilder::fixed(),
            ColliderBuilder::compound(parts)
                .friction(0.6)
                .restitution(0.0),
        );
    }
    let s0 = born(100.0, 65.5, 8.0, Vec3::new(60.0, 0.0, 0.0));
    let h = EntityDesc::default().half_size;
    let (handle, _) = phys.insert(
        RigidBodyBuilder::dynamic()
            .pose(Pose {
                rotation: s0.rotation,
                translation: s0.translation,
                ..Pose::IDENTITY
            })
            .linvel(s0.linvel)
            .angvel(s0.angvel),
        ColliderBuilder::cuboid(h, h, h)
            .density(1.0)
            .friction(0.6)
            .restitution(0.1),
    );
    // 双片侧:spawn 盖 apply_at=拍 0(首次 tick 即生效)→ 拍 0..44 共 45 步
    // (跨片交接不吞步:出界拍在源片照走步,下一拍信封前缀入目的片再步);
    // 参考侧同步 45 步。
    for _ in 0..45 {
        phys.step();
    }
    let b = phys.bodies.get(handle).expect("自家刚体");
    let reference = EntityState {
        translation: b.translation(),
        rotation: *b.rotation(),
        linvel: b.linvel(),
        angvel: b.angvel(),
    };

    let d = shard_end.translation - reference.translation;
    assert!(
        d.x.abs() < 1e-4 && d.y.abs() < 1e-4 && d.z.abs() < 1e-4,
        "AC-C2 边界编辑 + 跨片滚动 vs 单世界参考:位移差超限(容差 1e-4 m):\
         Δ=({},{},{}) m,分片侧={:?} 参考={:?}",
        d.x,
        d.y,
        d.z,
        shard_end.translation,
        reference.translation
    );
}

// ---------------------------------------------------------------------------
// AC-C3:owner-only 持久化(真 redb + ChunkProvider 真路径)
// ---------------------------------------------------------------------------

/// 一轮"双副本 → 修改 → 卸载(指定顺序)→ 库读回"的编排,返回观察值:
/// 每次卸载的返回值(Option:None = 编排不得写库)+ 库终值 + 重取值。
fn owner_only_roundtrip(
    dir: &std::path::Path,
    unload_holder_first: bool,
) -> Vec<(String, Option<Chunk>)> {
    let chunks_store = RedbStore::open(&dir.join("chunks.redb")).expect("真 redb");
    let worker = StoreWorker::new(Box::new(chunks_store), Box::new(MemCheckpointStore::new()));
    let seed = WorldSeed(7);
    let provider = ChunkProvider::new(&worker, Box::new(Superflat::classic()), seed);

    let mut session = Session::new(session_cfg());
    session.register(void_sim(0, 0, true));
    session.register(void_sim(1, 0, true));
    let c7 = ChunkPos::new(7, 0);
    let generated = Superflat::classic().generate(seed, c7);
    session.apply_chunk_load(shard_id(0, 0), c7, generated.clone());
    session.apply_chunk_load(shard_id(1, 0), c7, generated.clone());

    // 边界修改(双副本同拍更新,AC-C2 已锚)。
    session
        .set_block(
            shard_id(0, 0),
            BlockPos::new(120, types::WORLD_Y_MIN, 8),
            BlockId(2),
        )
        .expect("广播");
    session.tick();

    let mut observations: Vec<(String, Option<Chunk>)> = Vec::new();
    {
        let mut unload = |session: &mut Session, shard: ShardId, label: &str| {
            let returned = session.apply_chunk_unload(shard, c7);
            // 驱动同款语义(app/driver.rs orchestrate):返回 Some(owner chunk)
            // 才入持久化批;None(非 owner 副本)零写入。
            if let Some(chunk) = returned.clone() {
                worker.put_chunks(vec![(key(0, 0, 7, 0), chunk)]);
                assert!(
                    worker.flush().recv().expect("flush ack").is_ok(),
                    "put 已应用(屏障)"
                );
            }
            observations.push((label.to_string(), returned));
        };
        if unload_holder_first {
            unload(&mut session, shard_id(1, 0), "holder");
            unload(&mut session, shard_id(0, 0), "owner");
        } else {
            unload(&mut session, shard_id(0, 0), "owner");
            unload(&mut session, shard_id(1, 0), "holder");
        }
    }

    // 库终值 + provider 重取(重载取回修改值 = 持久层优先,非生成器兜底)。
    observations.push(("stored".into(), provider.get(key(0, 0, 7, 0))));
    observations.push(("reloaded".into(), provider.get(key(0, 0, 7, 0))));
    session.shutdown();
    worker.shutdown().expect("正常关停");
    observations
}

/// AC-C3(真 redb):双副本 chunk 修改后
/// ① **非 owner 卸载不写库**(返回 None → 无 put → 库在此刻仍空);
/// ② owner 卸载写库(返回修改值,库 == 修改值);
/// ③ 重载取回修改值(provider 路径,非生成器初生值);
/// ④ 双副本两种卸载顺序(holder→owner / owner→holder)结果一致——非 owner
///    副本永不覆盖库值(顺序无关性)。
#[test]
fn acc3_owner_only_persistence() {
    let dir_a = tempfile::tempdir().expect("临时目录 A");
    let dir_b = tempfile::tempdir().expect("临时目录 B");

    let mut expect = Superflat::classic().generate(WorldSeed(7), ChunkPos::new(7, 0));
    expect.set(8, 0, 8, BlockId(2));

    for (obs, order) in [
        (owner_only_roundtrip(dir_a.path(), true), "holder→owner"),
        (owner_only_roundtrip(dir_b.path(), false), "owner→holder"),
    ] {
        let mut map: BTreeMap<&str, &Option<Chunk>> = BTreeMap::new();
        for (label, chunk) in &obs {
            map.insert(label, chunk);
        }
        // ① holder 的卸载返回 None(驱动写库条件 = Some ⟹ 非 owner 永不写)。
        assert!(
            map["holder"].is_none(),
            "[{order}] holder 副本卸载必须返回 None(不写库,D6),实得 {:?}",
            map["holder"].as_ref().map(|_c| "Some(chunk)")
        );
        // ② owner 卸载返回修改值(可持久化)。
        assert_eq!(
            map["owner"].as_ref(),
            Some(&expect),
            "[{order}] owner 卸载返回修改值"
        );
        // ②/③ 库终值与 provider 重取 = 修改值(持久层优先,不被初生覆盖)。
        assert_eq!(
            map["stored"].as_ref(),
            Some(&expect),
            "[{order}] 库终值 == 修改值"
        );
        assert_eq!(
            map["reloaded"].as_ref(),
            Some(&expect),
            "[{order}] provider 重取 = 修改值"
        );
    }

    // ④ 中间态专项:holder 卸载后、owner 卸载前,库必须仍**无**该键
    // (非 owner 零写入的直接证据;独立小场景精确复现该时序)。
    // 同步读库 = 关停 worker 释放 redb 锁 → 重开同路径直读 ChunkStore::get
    // (不用 provider.get:它对缺键走生成器兜底,无法证明"库中无键")。
    let dir_c = tempfile::tempdir().expect("临时目录 C");
    let db_path = dir_c.path().join("chunks.redb");
    let store = RedbStore::open(&db_path).expect("真 redb");
    let worker = StoreWorker::new(Box::new(store), Box::new(MemCheckpointStore::new()));
    let mut session = Session::new(session_cfg());
    session.register(void_sim(0, 0, true));
    session.register(void_sim(1, 0, true));
    let c7 = ChunkPos::new(7, 0);
    let generated = Superflat::classic().generate(WorldSeed(7), c7);
    session.apply_chunk_load(shard_id(0, 0), c7, generated.clone());
    session.apply_chunk_load(shard_id(1, 0), c7, generated.clone());
    session
        .set_block(
            shard_id(0, 0),
            BlockPos::new(120, types::WORLD_Y_MIN, 8),
            BlockId(2),
        )
        .expect("广播");
    session.tick();
    assert!(
        session.apply_chunk_unload(shard_id(1, 0), c7).is_none(),
        "holder 卸载 → None(前置)"
    );
    // 驱动写库条件 = Some ⟹ holder 的 None 意味零 put;屏障对齐后关停,
    // 重开同库直读:键必须缺席(非 owner 副本没写进去,D6)。
    assert!(worker.flush().recv().expect("ack").is_ok(), "屏障对齐");
    worker.shutdown().expect("关停释放锁");
    let reopened = RedbStore::open(&db_path).expect("重开同库");
    assert_eq!(
        reopened.get(key(0, 0, 7, 0)).expect("库读无错"),
        None,
        "非 owner 卸载后库必须仍无该键(不写库,D6)"
    );
    // owner 卸载 → Some → 写库 → 重开立读 = 修改值(时序闭环)。
    let owner_chunk = session
        .apply_chunk_unload(shard_id(0, 0), c7)
        .expect("owner 卸载 Some");
    let worker2 = StoreWorker::new(Box::new(reopened), Box::new(MemCheckpointStore::new()));
    worker2.put_chunks(vec![(key(0, 0, 7, 0), owner_chunk.clone())]);
    assert!(worker2.flush().recv().expect("ack").is_ok());
    worker2.shutdown().expect("关停");
    assert_eq!(
        RedbStore::open(&db_path)
            .expect("三开同库")
            .get(key(0, 0, 7, 0))
            .expect("库读无错")
            .as_ref(),
        Some(&owner_chunk),
        "owner 写入后重开立读 = 修改值(写库闭环)"
    );
    session.shutdown();
}

// ---------------------------------------------------------------------------
// AC-C4:迁移目的片 ensure / 无信封黑洞
// ---------------------------------------------------------------------------

/// 跨片实体构造:零重力、x=127.9、v=(60,0,0) → 首步 x≈128.9 越界,
/// 迁移信封 apply_at = 下一拍。
fn cross_boundary_spawn() -> SimCommand {
    SimCommand::SpawnDynamic {
        entity: eid(1),
        state: born(127.9, 8.0, 8.0, Vec3::new(60.0, 0.0, 0.0)),
        desc: EntityDesc::default(),
    }
}

/// AC-C4(目的片已 ensure):观察者距边界 8m(<32m)→ 驱动 ① 的 ensure 集
/// = {(0,0),(1,0)}(复刻:两片都已注册);实体跨片:
/// ① 迁移拍后 `pending_shard_destinations` == {(1,0)}(信封在途,非退化);
/// ② **下一拍**实体出现在已注册的目的片(位置连续性由迁移包络保证);
/// ③ 出现后 pending 恰空(无残留信封)。
#[test]
fn acc4_migration_destination_ensured() {
    let mut session = Session::new(session_cfg());
    // 观察者命令入表(source=1,(120,0,8) 距 (1,0) core 面 8m < 32m)。
    // 盖章语义:apply_at = 当前拍 → 下一拍 tick 生效,故先 tick 落表。
    session.register(void_sim(0, 0, true));
    session
        .move_observer(Vec3::new(120.0, 0.0, 8.0))
        .expect("unbounded 收件箱");
    session.tick();
    // 驱动 ① 语义复刻:shards_within((120,0,8), ensure_radius=32) 恰
    // {(0,0),(1,0)} → 邻片先注册(目的片已创建的成因)。
    session.register(void_sim(1, 0, true));
    assert_eq!(
        session.observers(),
        vec![(InputSource(1), Vec3::new(120.0, 0.0, 8.0))],
        "前置:观察者表就位(ensure 依据)"
    );

    session
        .send(shard_id(0, 0), cross_boundary_spawn())
        .expect("入箱");
    session.tick(); // spawn + step → 迁移意图入 Router
    assert!(
        session.entity_state(shard_id(0, 0), eid(1)).is_none(),
        "实体已离开源片"
    );
    assert_eq!(
        session.pending_shard_destinations(),
        [shard_id(1, 0)].into_iter().collect(),
        "迁移信封在途:pending dest 恰 {{(1,0)}}"
    );
    session.tick(); // 拍 1:信封到期投递
    let arrived = session
        .entity_state(shard_id(1, 0), eid(1))
        .expect("AC-C4:实体下一拍出现在目的片");
    assert!(
        arrived.translation.x > 128.0,
        "位置连续(不丢失不回落):x={}",
        arrived.translation.x
    );
    assert!(
        session.pending_shard_destinations().is_empty(),
        "投递后 pending 恰空"
    );
    session.shutdown();
}

/// AC-C4(黑洞防线:ensure 抢救路径):目的片未注册时造出迁移信封,
/// 在**下一拍 tick 前**按 pending 集 ensure(驱动 tick 末 ensure 语义)→
/// 下一拍信封正常落地、pending 空;真正的黑洞 fail-stop(ensure 缺席下一拍
/// 必 panic)由 scheduler.rs `envelope_black_hole_*` 专项锚定,这里断言
/// "抢救后不得再报 pending"。
#[test]
fn acc4_no_envelope_black_hole() {
    let mut session = Session::new(session_cfg());
    session.register(void_sim(0, 0, true)); // 只注册源片:目的 (1,0) 未注册
    session
        .send(shard_id(0, 0), cross_boundary_spawn())
        .expect("入箱");
    session.tick();
    let pending = session.pending_shard_destinations();
    assert_eq!(
        pending,
        [shard_id(1, 0)].into_iter().collect(),
        "前置:未注册目的片的迁移信封已沉在 Router(pending == {{(1,0)}})"
    );
    // 驱动 tick 末 ensure:按 pending 集补注册(任务 00 D5 生产路径同语义)。
    for dest in pending {
        let (x, z) = (dest.pos.x, dest.pos.z);
        session.register(void_sim(x, z, true));
    }
    session.tick(); // 下一拍:信封 dispatch(若 ensure 缺位,此拍 fail-stop)
    assert!(
        session.pending_shard_destinations().is_empty(),
        "ensure 后下一拍 pending_shard_destinations 必空(黑洞条件解除)"
    );
    assert!(
        session.entity_state(shard_id(1, 0), eid(1)).is_some(),
        "信封落到抢救出的目的片"
    );
    session.shutdown();
}

// ---------------------------------------------------------------------------
// AC-C5:checkpoint 携带 pending 迁移 / 观察者 + 生命周期
// ---------------------------------------------------------------------------

/// AC-C5(迁移信封入快照):在 **tick N 末 Router 里确有迁移信封** 时截
/// checkpoint(非退化前置:快照的 `scheduler.router.envelopes` 非空);
/// `replay_world` 续跑到 N+2 与原线逐位一致(实体在目的片,四向量 `==`)。
#[test]
fn acc5_checkpoint_pending_migration() {
    // --- 原线:拍 0 spawn 跨片;拍 0 末(信封入 Router)+ ensure 后截快照 ---
    let mut original = Session::new(session_cfg());
    original.register(void_sim(0, 0, true));
    original
        .send(shard_id(0, 0), cross_boundary_spawn())
        .expect("入箱");
    original.tick(); // 拍 0 执行:迁移信封入 Router(apply_at = 1)
    original.register(void_sim(1, 0, true)); // tick 末 ensure(快照前注册 → 快照键集含目的片)
    let snapshot = original.world_snapshot();
    // 非退化红线:**制造**出的非空信封集,不是挑干净拍。
    assert_eq!(snapshot.shards.tick, Tick(1), "快照拍 = 下一待执行拍 N=1");
    assert_eq!(
        snapshot
            .scheduler
            .router
            .envelopes
            .iter()
            .map(|e| (e.destination, e.apply_at))
            .collect::<Vec<_>>(),
        vec![(shard_id(1, 0), Tick(1))],
        "前置:快照必须带着未应用的迁移信封(dest/apply_at 精确)"
    );
    // 拍 1 的命令日志(盖拍 1 的条目 = 快照后到达,重放窗 >= 覆盖)。
    let mut log = CommandLog::new();
    let later = original
        .send(
            shard_id(1, 0),
            SimCommand::SpawnDynamic {
                entity: eid(2),
                state: born(150.0, 8.0, 8.0, Vec3::new(24.0, 0.0, 0.0)),
                desc: EntityDesc::default(),
            },
        )
        .expect("入箱");
    assert_eq!(later.tick, Tick(1), "命令盖快照拍(== 必回放,不丢)");
    log.append(later);
    original.tick(); // 拍 1:信封投递 + eid2 spawn
    original.tick(); // 拍 2:eid2 第一步
    let end_a = original
        .entity_state(shard_id(1, 0), eid(1))
        .expect("迁移实体在目的片");
    let end_b = original
        .entity_state(shard_id(1, 0), eid(2))
        .expect("快照拍 spawn 实体存在");
    let end_obs = original.observers();
    original.shutdown();

    // --- 重放线:快照 + 日志 → tick 到 N+2,逐位对照 ---
    let mut replayed =
        replay_world(&snapshot, &log, session_cfg(), new_registry()).expect("自有快照必可恢复");
    assert_eq!(replayed.tick_now(), Tick(1), "重放线对齐快照拍");
    replayed.tick(); // 拍 1
    replayed.tick(); // 拍 2
    assert_eq!(
        replayed.entity_state(shard_id(1, 0), eid(1)),
        Some(end_a),
        "AC-C5:迁移实体经快照+重放后终态位级一致"
    );
    assert_eq!(
        replayed.entity_state(shard_id(1, 0), eid(2)),
        Some(end_b),
        "AC-C5:盖快照拍的命令不丢(replay 窗 >= 语义)"
    );
    assert_eq!(
        replayed.observers(),
        end_obs,
        "观察者表一致(本场景为空 → 同为空;防 ghost)"
    );
    assert!(replayed.pending_shard_destinations().is_empty());
    replayed.shutdown();
}

/// AC-C5(观察者表 + 生命周期入快照):双观察者(不同 source)+ 一个空闲
/// 无票分片计时中(despawn_after_ticks=30 的紧配置)在拍 25 截快照
/// (非退化前置:scheduler 快照的 observers 2 条、lifecycles 3 条非空)。
/// 恢复后**不发任何新 Move**:
/// ① 观察者表位级一致(票/激活中心再生);
/// ② 激活继续:重放线 tick 后的激活请求与同拍原线逐字段一致;
/// ③ despawn 候选拍不被重置:空闲分片在两线**同一拍**首次成为候选。
#[test]
fn acc5_checkpoint_observers_and_lifecycle() {
    let cfg = SessionConfig {
        scheduler: SchedulerConfig {
            worker_count: 2,
            despawn_after_ticks: 30,
            ..SchedulerConfig::default()
        },
        ..SessionConfig::default()
    };
    // 冷却关 + 预算大(两线激活时钟在快照后一致;acg5 同款口径)。
    let act = SimActivationConfig {
        cooldown_ticks: 0,
        data_budget_per_tick: 14,
        collider_budget_per_tick: 14,
        ..SimActivationConfig::default()
    };
    let mk = |x: i32, z: i32| {
        let mut s = ShardSimConfig::new(shard_id(x, z));
        s.gravity = Vec3::ZERO;
        s.activation = act;
        s.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        ShardSim::new(s, Box::new(SimpleRegistry::builtin()))
    };

    let mut original = Session::new(cfg.clone());
    original.register(mk(0, 0));
    original.register(mk(1, 0));
    original.register(mk(5, 5)); // 空闲无票分片:despawn 计时被测对象
    let pa = Vec3::new(8.0, 0.0, 8.0);
    let pb = Vec3::new(200.0, 0.0, 8.0);
    original
        .move_observer_from(InputSource(2), pa)
        .expect("unbounded");
    original
        .move_observer_from(InputSource(3), pb)
        .expect("unbounded");
    for _ in 0..25 {
        original.tick();
    }
    let snapshot = original.world_snapshot();
    // 非退化红线:快照必须带着非空的观察者表与生命周期集(制造而非偶遇)。
    assert_eq!(
        snapshot
            .scheduler
            .observers
            .iter()
            .map(|(k, v)| (*k, *v))
            .collect::<Vec<_>>(),
        vec![(InputSource(2), pa), (InputSource(3), pb)],
        "前置:快照携带双观察者条目"
    );
    assert_eq!(
        snapshot.scheduler.lifecycles.len(),
        3,
        "前置:3 条生命周期记录"
    );
    let log = CommandLog::new(); // 快照后无新命令(恢复线仅靠快照再生状态)

    // 两线并行推进:逐拍取 despawn 候选 + 激活决策流 + 调度器快照。
    // 冷却 0 + 从不装载(不喂编排)→ plan 每拍对同一近邻候选重发,两线
    // 激活簿记(activation 控制器与 activation_now 随分片快照恢复)同态。
    let run_to = |s: &mut Session,
                  ticks: u64,
                  first_candidate: &mut Option<u64>|
     -> (
        Vec<Vec<(ShardId, physics_sim::ActivationDecisions)>>,
        Vec<shard::SchedulerSnapshot>,
    ) {
        let mut reqs = Vec::new();
        let mut snaps = Vec::new();
        for _ in 0..ticks {
            s.tick();
            let t = s.tick_now().0;
            if first_candidate.is_none() && s.take_despawn_requests().contains(&shard_id(5, 5)) {
                *first_candidate = Some(t);
            }
            reqs.push(s.take_activation_requests());
            snaps.push(s.world_snapshot().scheduler);
        }
        (reqs, snaps)
    };
    let mut first_orig = None;
    let (orig_reqs, orig_snaps) = run_to(&mut original, 15, &mut first_orig);
    let mut first_replay = None;
    let mut replayed = replay_world(&snapshot, &log, cfg, new_registry()).expect("恢复成功");
    let (replay_reqs, replay_snaps) = run_to(&mut replayed, 15, &mut first_replay);

    // ③ despawn 候选拍不被重置:两线首次候选拍一致(恢复的是 last_activity,
    //   不是重开计时;despawn_after_ticks=30、注册拍 0 → 拍 30 到期)。
    let fo = first_orig.expect("原线空片必须到期成候选(场景非退化)");
    let fr = first_replay.expect("重放线同场景必须同判");
    assert_eq!(
        fo, fr,
        "despawn 候选拍不被 checkpoint 重置:原线拍 {fo} vs 重放线拍 {fr}"
    );
    // ① 观察者表/生命周期:全程逐拍位级一致(无新 Move,票与计时状态存活)。
    for (a, b) in orig_snaps.iter().zip(&replay_snaps) {
        assert_eq!(a.observers, b.observers, "观察者表逐拍位级一致");
        assert_eq!(a.lifecycles, b.lifecycles, "生命周期逐拍位级一致");
    }
    // ② 激活继续:重放线不发新 Move,激活请求流与原线逐拍逐字段一致
    //   (非退化:观察者在场 → 必有非空请求拍)。
    let mut saw_loads = false;
    for (k, (a, b)) in orig_reqs.iter().zip(&replay_reqs).enumerate() {
        assert_eq!(a, b, "拍 {k} 起算:两线激活请求逐字段一致");
        if a.iter().any(|(_, d)| !d.chunks_to_load.is_empty()) {
            saw_loads = true;
        }
    }
    assert!(saw_loads, "非退化:观察者驱动的装载候选确在两线中发生");
    original.shutdown();
    replayed.shutdown();
}

// ---------------------------------------------------------------------------
// AC-C6:网络观察者 Move/Remove 入日志 + 重放一致(真 TCP)
// ---------------------------------------------------------------------------

/// 内嵌迷你 TCP 客户端(network.rs 授权复制口径:codec = sim-api::proto 对称
/// 公开接口,帧 = 4B LE 长度 + bincode)。
struct NetClient {
    w: tokio::net::tcp::OwnedWriteHalf,
}

impl NetClient {
    async fn send(&mut self, c2s: &sim_api::proto::C2S) -> std::io::Result<()> {
        self.w
            .write_all(&sim_api::proto::encode_c2s_frame(c2s))
            .await
    }
}

async fn net_connect(
    addr: SocketAddr,
) -> (NetClient, mpsc::UnboundedReceiver<sim_api::proto::S2C>) {
    let stream = TcpStream::connect(addr).await.expect("TCP 连入");
    let (mut r, w) = stream.into_split();
    let (tx, rx) = mpsc::unbounded_channel();
    tokio::spawn(async move {
        let mut buf: Vec<u8> = Vec::with_capacity(16 * 1024);
        let mut chunk = [0u8; 16 * 1024];
        loop {
            let frame = loop {
                if buf.len() >= 4 {
                    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize + 4;
                    if buf.len() >= len {
                        break buf.drain(..len).collect::<Vec<u8>>();
                    }
                }
                match r.read(&mut chunk).await {
                    Ok(0) | Err(_) => return,
                    Ok(n) => buf.extend_from_slice(&chunk[..n]),
                }
            };
            if let Some(s2c) = sim_api::proto::decode_frame(&frame)
                && tx.send(s2c).is_err()
            {
                return;
            }
        }
    });
    (NetClient { w }, rx)
}

/// 软窗收一帧(暂停虚拟时钟 50ms;network.rs 同款)。
async fn net_frame(
    rx: &mut mpsc::UnboundedReceiver<sim_api::proto::S2C>,
) -> Option<sim_api::proto::S2C> {
    tokio::time::timeout(Duration::from_millis(50), rx.recv())
        .await
        .ok()
        .flatten()
}

async fn net_join(
    c: &mut NetClient,
    rx: &mut mpsc::UnboundedReceiver<sim_api::proto::S2C>,
) -> InputSource {
    c.send(&sim_api::proto::C2S::Hello { client_id: 1 })
        .await
        .expect("Hello 写入");
    c.send(&sim_api::proto::C2S::Subscribe { view_distance: 1 })
        .await
        .expect("Subscribe 写入");
    for _ in 0..64 {
        match net_frame(rx).await {
            Some(sim_api::proto::S2C::Welcome { player_id, .. }) => return player_id,
            Some(_) => continue,
            None => panic!("软窗预算内未见 Welcome"),
        }
    }
    panic!("64 软窗内未收到 Welcome")
}

/// AC-C6:两网络玩家真 TCP 入会 → 各发 MoveObserver(入日志,D9:source =
/// 玩家)→ A 断开(RemoveObserver 入日志)→ checkpoint(拍 60 + 终末)→
/// `replay_world` 恢复:
/// ① 日志含 A/B 的 Move 与 A 的 Remove(条目数/source/位置精确);
/// ② 重放线 tick 后观察者表与原线**位级一致**:只有 B 与其位置;
/// ③ 无幽灵:A 的身份(表键与 ever-set 语义)在恢复线绝不出现;
/// ④ 票/激活与观察者表一致:B 区激活请求照表发生,A 区无(表是唯一中心)。
#[tokio::test(start_paused = true)]
async fn acc6_network_observer_replay() {
    use app::driver::ObserveCadence;
    use app::net::{NetConfig, NetUp, OutboundHub, serve};
    use sim_api::proto::C2S;

    let config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 12, // 场景中点截一张(重放素材:激活在途中)
        // ④ 的素材设计:预算 1 片/拍 + 冷却 0 → B 区(cp12 时 12 候选,手算
        // 见下)必仍有未载完的在途候选;重放线"表→激活"闭环不用等簿记过期。
        // (REQ-007 06c:预算键升级为 ①② 分档,两档同给同值语义不变。)
        activation: app::config::ActivationConfig {
            cooldown_ticks: 0,
            data_budget_per_tick: 1,
            collider_budget_per_tick: 1,
            ..app::config::ActivationConfig::default()
        },
        ..Config::default()
    };
    let net_cfg = NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 600,
        observe_interval_ticks: 2,
        full_snapshot_interval_ticks: 60,
        pump: Default::default(),
    };
    let cadence = ObserveCadence::from(&net_cfg);
    let (up_tx, up_rx) = mpsc::channel::<NetUp>(256);
    let hub = Arc::new(OutboundHub::new());
    let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
    let addr = serve(net_cfg, up_tx, bcast_rx, hub.clone())
        .await
        .expect("监听 127.0.0.1:0");
    std::mem::forget(bcast_tx);

    let checkpoints = SharedCheckpoints::default();
    let session = Session::new(config.session_config());
    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(checkpoints.clone()),
    );
    let (provider, _) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let sim_source = config.clone();
    let new_sim = Box::new(move |shard: ShardId| {
        let mut cfg = sim_source.shard_sim_config(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
    });
    let driver = tokio::spawn(run(DriverArgs {
        config: config.clone(),
        session,
        store,
        log: CommandLog::new(),
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: Some(app::driver::ObserveLink {
            up: up_rx,
            hub: hub.clone(),
            cadence,
        }),
        spawn_next_seq: 0,
    }));

    // A 进区 (8,·,8),B 进区 (200,·,8);A 中途断开。
    let (mut a, mut rx_a) = net_connect(addr).await;
    let (mut b, mut rx_b) = net_connect(addr).await;
    tokio::task::yield_now().await;
    let id_a = net_join(&mut a, &mut rx_a).await;
    let id_b = net_join(&mut b, &mut rx_b).await;
    let pos_a = Vec3::new(8.0, -58.0, 8.0);
    let pos_b = Vec3::new(200.0, -58.0, 8.0);
    advance_ticks(4).await;
    a.send(&C2S::MoveObserver { position: pos_a })
        .await
        .expect("写入");
    b.send(&C2S::MoveObserver { position: pos_b })
        .await
        .expect("写入");
    // 跑过拍 12 检查点(interval=12:默认预算 2/拍下两区装载仍在途——重放
    // 线必须能"继续"这份激活,非退化素材),此刻 A 仍在场;然后 A 断开。
    advance_ticks(12).await;
    a.send(&C2S::Goodbye).await.expect("Goodbye 写入");
    advance_ticks(60).await; // Left → Remove 入日志;驱动继续推进

    tx.send(DriverCommand::Shutdown).await.expect("驱动存活");
    let outcome = driver.await.expect("驱动不 panic");
    drop(rx_a);
    drop(rx_b);

    // 素材:拍 12 检查点(重放起点;log.take 切走 Move)+ 拍 24 检查点
    // (Remove 落在此段的 take)+ 终末检查点(驱动侧事实:观察者表终值)。
    let (_t1, blob1) = checkpoints
        .read(CheckpointId(1))
        .expect("读检查点")
        .expect("拍 12 必有一张(interval=12)");
    let bundle1 = decode_bundle(&blob1);
    assert_eq!(bundle1.world.shards.tick, Tick(12));
    let (_t2, blob2) = checkpoints
        .read(CheckpointId(2))
        .expect("读检查点")
        .expect("拍 24 必有一张");
    let bundle2 = decode_bundle(&blob2);
    let (_tt, blobn) = checkpoints
        .read_latest()
        .expect("读最新")
        .expect("终末检查点");
    let bundle_n = decode_bundle(&blobn);

    // ① 日志精确:Move×2(入 12 拍的 take)与 Remove×1(入其后的 take;
    // Remove 落拍 ∈ [12, 24) → 必在 bundle1.log ∪ bundle2.log)。
    let mut log1 = CommandLog::from_bytes(&bundle1.log_bytes).expect("自有编码");
    let log2 = CommandLog::from_bytes(&bundle2.log_bytes).expect("自有编码");
    let moves: Vec<_> = log1
        .entries()
        .iter()
        .chain(log2.entries())
        .filter(|e| matches!(e.command, SimCommand::SetObserverPosition { .. }))
        .collect();
    assert_eq!(
        moves.len(),
        2,
        "Move 入日志恰 2 条:{:?} + {:?}",
        log1.entries(),
        log2.entries()
    );
    let removes: Vec<_> = log1
        .entries()
        .iter()
        .chain(log2.entries())
        .filter(|e| e.command == SimCommand::RemoveObserver)
        .collect();
    assert_eq!(removes.len(), 1, "A 断连的 RemoveObserver 入日志恰 1 条");
    assert_eq!(removes[0].source, id_a, "Remove 的 source = A 玩家身份");
    assert!(moves.iter().any(|e| e.source == id_a));
    assert!(moves.iter().any(|e| e.source == id_b));

    // 合并续日志(tick 升序:log1 entries < log2 entries,驱动落拍天然有序;
    // CommandLog::append 的 tick 单调约束即一致性检查)。
    for e in log2.entries() {
        log1.append(e.clone());
    }

    // ②/③ 重放:快照@12(观察者表烙入 A+B;Remove 若未烙进则 tick>=12
    //   必回放,幂等)+ 合并日志 → 无幽灵复现,终态 == 驱动侧事实。
    let mut replayed = replay_world(
        &bundle1.world,
        &log1,
        config.session_config(),
        new_registry(),
    )
    .expect("自有快照必可恢复");
    // ④ 激活继续(表 = 唯一激活中心,行为面非计数):
    //   素材手算(cp12 观察者 A(8,·,8)+B(200,·,8),预算 1 片/拍·分片,
    //   冷却 0):B 距 (1,0) core 32m 内候选 = 中心 ∈ [168,232]×[-24,40]
    //   ∩ 区域 = {x10..14}×{z-1..2} 共 20 片,驱动至多装载 7 片 → 重放线
    //   必有在途 B 区请求;A 区 (0,0) 同理 16 候选 > Remove 前拍数 →
    //   A 在场期必有 A 区请求(A 离场后必须归零 = 票撤即激活撤)。
    let mut saw_load_a_before_remove = false;
    let mut saw_load_for_b = false;
    let mut a_removed = false;
    for _ in 0..70 {
        replayed.tick();
        for (shard, d) in replayed.take_activation_requests() {
            if shard.pos == ShardPos::new(1, 0) && !d.chunks_to_load.is_empty() {
                saw_load_for_b = true;
            }
            if shard.pos == ShardPos::new(0, 0) && !d.chunks_to_load.is_empty() {
                if a_removed {
                    panic!("AC-C6:A 离场后 A 区片 (0,0) 仍在报装载请求(票撤未生效)");
                }
                saw_load_a_before_remove = true;
            }
        }
        let obs = replayed.observers();
        if !obs.iter().any(|(src, _)| *src == id_a) {
            a_removed = true;
        } else if a_removed {
            panic!("AC-C6:幽灵 A 在移除后复现(表 = {obs:?})");
        }
    }
    assert!(
        a_removed,
        "前置:Remove 必须在重放线生效(快照烙入或日志回放)"
    );
    assert!(
        saw_load_a_before_remove,
        "非退化:A 在场期激活请求确随表发生(Remove 有可观察的边界)"
    );
    assert_eq!(
        replayed.observers(),
        bundle_n
            .world
            .scheduler
            .observers
            .iter()
            .map(|(k, v)| (*k, *v))
            .collect::<Vec<_>>(),
        "AC-C6:重放终态观察者表 == 驱动侧终末检查点事实(位级;仅 B 与其位置)"
    );
    assert_eq!(
        replayed.observers(),
        vec![(id_b, pos_b)],
        "AC-C6:终表精确集合 = {{(B, pos_b)}}——A 无幽灵"
    );
    assert!(
        saw_load_for_b,
        "AC-C6:重放线激活按 B 的观察者表继续(B 区出现装载候选)"
    );
    replayed.shutdown();
    assert!(outcome.ticks_run >= 60, "前置:驱动跑过检查点拍");
}

// ---------------------------------------------------------------------------
// AC-C7:边界广播爆炸 / 涌现事件 N+1 / 凝固 BlockWrite 跨片
// ---------------------------------------------------------------------------

/// AC-C7(detónate 广播面):`Session::detonate` 边界爆心两片各收一份:
/// ① 日志恰 2 条同 sequence(dest 升序精确集)——每片恰一次;
/// ② 零重力下两片实体的终态 linvel **位级 == explosion_impulse 纯函数值**
///    (恰一次冲量:两次施加会位级翻倍,可区分);
/// ③ 合并终态 vs 单世界参考(同位置单 ShardSim,同命令前缀)< 1e-4 m。
#[test]
fn acc7_boundary_detonate_broadcast() {
    let center = Vec3::new(128.0, 8.0, 8.0);
    let (power, radius) = (12.0_f32, 72.0_f32);
    let (p0, p1) = (Vec3::new(112.0, 8.0, 8.0), Vec3::new(144.0, 8.0, 8.0));
    let (e0, e1) = (eid(1), eid(2));

    let mut session = Session::new(session_cfg());
    session.register(void_sim(0, 0, true));
    session.register(void_sim(1, 0, true));
    for (shard, e, p) in [(shard_id(0, 0), e0, p0), (shard_id(1, 0), e1, p1)] {
        session
            .send(
                shard,
                SimCommand::SpawnDynamic {
                    entity: e,
                    state: born(p.x, p.y, p.z, Vec3::ZERO),
                    desc: EntityDesc::default(),
                },
            )
            .expect("入箱");
    }
    let entries = session.detonate(center, power, radius).expect("覆盖集非空");
    assert_eq!(
        entries
            .iter()
            .map(|e| (e.dest, e.sequence))
            .collect::<Vec<_>>(),
        vec![(shard_id(0, 0), 2), (shard_id(1, 0), 2)],
        "① 每片恰一条、同 sequence(一次广播 = 每片恰一次)"
    );
    for _ in 0..30 {
        session.tick();
    }
    // ② 冲量位级锚。
    for (e, p, shard, sign) in [
        (e0, p0, shard_id(0, 0), -1.0),
        (e1, p1, shard_id(1, 0), 1.0),
    ] {
        let st = session.entity_state(shard, e).expect("实体在归属片");
        let expected =
            physics_sim::explosion::explosion_impulse(center, power, radius, p).expect("半径内");
        assert!(expected.x * sign > 0.0, "前置:冲量背离爆心");
        assert_eq!(
            st.linvel, expected,
            "② 终态 linvel 位级 = 单份冲量(两次施加会位级不同)"
        );
    }
    // ③ vs 单世界参考(同位置同命令前缀;零重力弹道按容差 <1e-4 m)。
    for (e, p, sx) in [(e0, p0, 0), (e1, p1, 1)] {
        let mut single = void_sim(sx, 0, true);
        single.apply_prefix(&[
            messaging::StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 0,
                payload: messaging::InputPayload::Command(SimCommand::SpawnDynamic {
                    entity: e,
                    state: born(p.x, p.y, p.z, Vec3::ZERO),
                    desc: EntityDesc::default(),
                }),
            },
            messaging::StampedInput {
                apply_at: Tick::ZERO,
                source: messaging::InputSource::SCHEDULER,
                sequence: 1,
                payload: messaging::InputPayload::Command(SimCommand::Detonate {
                    center,
                    power,
                    radius,
                }),
            },
        ]);
        for _ in 0..30 {
            single.step();
        }
        let ref_state = single.entity_state(e).expect("参考实体在");
        let two = session
            .entity_state(shard_id(sx, 0), e)
            .expect("分片实体在");
        let d = two.translation - ref_state.translation;
        assert!(
            d.x.abs() < 1e-4 && d.y.abs() < 1e-4 && d.z.abs() < 1e-4,
            "③ 双片广播 vs 单世界参考 < 1e-4 m(容差入消息):Δ=({},{},{})",
            d.x,
            d.y,
            d.z
        );
    }
    session.shutdown();
}

/// AC-C7(涌现事件面):`Scheduler::broadcast_event` 目标拍 = N+1(信封
/// apply_at 精确);origin 不重复(origin 分片实体在事件窗内 linvel 恒 0,
/// 目标片实体在 N+1 恰获得单份冲量);信封 source_sequence 单调 canonical
/// (第二次广播 = 序号 +1 且 dispatch 先 0 后 1)。
#[test]
fn acc7_emergent_event_next_tick() {
    let mut sched = Scheduler::new(SchedulerConfig {
        worker_count: 2,
        ..SchedulerConfig::default()
    });
    sched.register(void_sim(0, 0, true));
    sched.register(void_sim(1, 0, true));
    // 两片各一实体(出生经 submit 直投)。
    let center = Vec3::new(128.0, 8.0, 8.0);
    let (power, radius) = (12.0_f32, 72.0_f32);
    let spawn_at = |sched: &mut Scheduler, shard: ShardId, e: EntityId, p: Vec3| {
        sched
            .submit(
                shard,
                messaging::StampedInput {
                    apply_at: Tick(0),
                    source: messaging::InputSource::SCHEDULER,
                    sequence: 0,
                    payload: messaging::InputPayload::Command(SimCommand::SpawnDynamic {
                        entity: e,
                        state: born(p.x, p.y, p.z, Vec3::ZERO),
                        desc: EntityDesc::default(),
                    }),
                },
            )
            .expect("入箱");
    };
    spawn_at(
        &mut sched,
        shard_id(0, 0),
        eid(1),
        Vec3::new(112.0, 8.0, 8.0),
    );
    spawn_at(
        &mut sched,
        shard_id(1, 0),
        eid(2),
        Vec3::new(144.0, 8.0, 8.0),
    );

    // 拍 0 后广播(produced_at = 拍 0):目标 (1,0),apply_at = 1。
    sched.tick();
    sched
        .broadcast_event(
            shard_id(0, 0),
            CrossEvent::Detonate {
                center,
                power,
                radius,
            },
            Tick(0),
        )
        .expect("目标集非空");
    // N 拍末:实体都静止(事件未生效)。
    assert_eq!(
        sched
            .entity_state(shard_id(1, 0), eid(2))
            .expect("在")
            .linvel,
        Vec3::ZERO,
        "N 拍:目标实体尚未受事件(生效拍 = N+1)"
    );
    sched.tick(); // 拍 1:信封到期 → (1,0) 恰收一份
    let hit = sched.entity_state(shard_id(1, 0), eid(2)).expect("在");
    let expected = physics_sim::explosion::explosion_impulse(
        center,
        power,
        radius,
        Vec3::new(144.0, 8.0, 8.0),
    )
    .expect("在半径内");
    assert_eq!(hit.linvel, expected, "N+1 生效且恰一次(位级 = 单份冲量)");
    // origin 不重复:origin 分片实体全程零速(不收自己广播的事件)。
    let origin_ent = sched.entity_state(shard_id(0, 0), eid(1)).expect("在");
    assert_eq!(
        origin_ent.linvel,
        Vec3::ZERO,
        "origin 分片实体绝不双重受事件(信封排除 origin)"
    );

    // canonical 序:第二次广播的信封 source_sequence = 1(router 簿记精确)。
    sched
        .broadcast_event(
            shard_id(0, 0),
            CrossEvent::Detonate {
                center: center + Vec3::X,
                power,
                radius,
            },
            sched.tick_now(),
        )
        .expect("目标集非空");
    let snap = sched.snapshot_state();
    assert_eq!(
        snap.router.sequences.get(&shard_id(0, 0)).copied(),
        Some(2),
        "信封序号簿记 = 已发 2 封(单调 canonical)"
    );
    let seqs: Vec<u64> = snap
        .router
        .envelopes
        .iter()
        .map(|e| e.source_sequence)
        .collect();
    assert_eq!(
        seqs,
        vec![1],
        "在途信封只剩第二封,source_sequence=1(先 0 已消费)"
    );
    sched.shutdown();
}

/// AC-C7/D11(凝固 BlockWrite 跨片):零重力直通凝固的实体落在 chunk 7
/// (owner (0,0))、(1,0) 持 margin 副本:
/// ① 产生拍:owner 发布集含该格(take_block_updates),副本**下一拍**逐字节
///    与 owner 一致(BlockWrite 信封 N+1);
/// ② owner 卸载 → Some(修改值)可持久化,holder 卸载 → None;库取回 = 修改值;
/// ③ 无广播回环:信封应用拍后产生/接收两侧均不再产新 BlockWrite
///    (pending_blockwrite 空 + 更新集空 + 副本保持相等)。
#[test]
fn acc7_blockwrite_across_boundary() {
    let c7 = ChunkPos::new(7, 0);
    let mut session = Session::new(session_cfg());
    session.register(solidifying_sim(0, 0)); // producer/owner:凝固直通
    session.register(void_sim(1, 0, true)); // holder 副本(凝固不关它的事)
    session.apply_chunk_load(shard_id(0, 0), c7, Chunk::air());
    session.apply_chunk_load(shard_id(1, 0), c7, Chunk::air());
    // 实体中心 (127.4,5.5,8.5) ∈ (0,0) core:snap 格必落 chunk 7(x<128)。
    session
        .send(
            shard_id(0, 0),
            SimCommand::SpawnDynamic {
                entity: eid(1),
                state: born(127.4, 5.5, 8.5, Vec3::ZERO),
                desc: EntityDesc::default(),
            },
        )
        .expect("入箱");
    // 跑到凝固(≤40 拍,凝固 = 实体消失 + Emergent 产出)。
    let mut cell = None;
    let mut updates_at_emerge = Vec::new();
    for _ in 0..40 {
        session.tick();
        let u = session.take_block_updates();
        if session.entity_state(shard_id(0, 0), eid(1)).is_none() {
            cell = u
                .iter()
                .find(|(s, _, _)| *s == shard_id(0, 0))
                .map(|(_, c, b)| (*c, *b));
            updates_at_emerge = u;
            break;
        }
        assert!(u.is_empty(), "凝固前无方块写入");
    }
    let (cell, block) = cell.expect("① 凝固必须产出写入格");
    assert_eq!(block, EntityDesc::default().block, "凝固块 = 实体方块类型");
    assert_eq!(cell.chunk_of(), c7, "凝固格必须落在跨界目标 chunk 7");
    assert_eq!(
        updates_at_emerge.len(),
        1,
        "① 产生拍发布集恰 1 条(产生者,无重复):{updates_at_emerge:?}"
    );

    // ② 信封落副本:下一拍两片 chunk 7 逐字节相等。
    session.tick();
    let owner_chunk = session.voxel_chunk(shard_id(0, 0), c7).expect("owner 在");
    let holder_chunk = session.voxel_chunk(shard_id(1, 0), c7).expect("holder 在");
    assert_eq!(
        holder_chunk, owner_chunk,
        "② N+1:holder 副本逐字节 == owner(凝固格跨片一致)"
    );
    // ③ 无回环:应用拍后无新更新、无 pending。
    assert!(
        session.take_block_updates().is_empty(),
        "③ 信封应用后不得再产 BlockWrite(回环 = 违规)"
    );
    assert!(
        session.pending_blockwrite_cells().is_empty(),
        "③ 信封已消费:pending_blockwrite 空"
    );
    session.tick();
    assert!(
        session.voxel_chunk(shard_id(1, 0), c7) == session.voxel_chunk(shard_id(0, 0), c7),
        "③ 副本持续一致(未回环漂移)"
    );

    // owner-only 持久化(D6/D11):holder 卸载 None、owner 卸载 Some(修改值)。
    assert!(
        session.apply_chunk_unload(shard_id(1, 0), c7).is_none(),
        "非 owner 卸载不得返回 chunk(不可写库)"
    );
    let persisted = session
        .apply_chunk_unload(shard_id(0, 0), c7)
        .expect("owner 卸载返回修改 chunk");
    let mut expect = Chunk::air();
    // 局部 ly = 世界 y − WORLD_Y_MIN(i32 减法;−60 − (−64) = 4)。
    let ly = cell.y - types::WORLD_Y_MIN;
    assert!(
        (0..voxel::CHUNK_Y as i32).contains(&ly),
        "凝固格必须在世界内:ly={ly}"
    );
    expect.set(
        cell.x.rem_euclid(16) as usize,
        ly as usize,
        cell.z.rem_euclid(16) as usize,
        block,
    );
    assert_eq!(
        persisted, expect,
        "owner 持久化值 = 空气 chunk + 凝固格(逐字节)"
    );
    session.shutdown();
}

// ---------------------------------------------------------------------------
// AC-C8:viewer 冒烟(真驱动 + 真网络;workspace/clippy/fmt 归编排层门禁)
// ---------------------------------------------------------------------------

/// AC-C8:viewer::client 连**真服务器**(真驱动 + 真 TCP,serve 真接):
/// 收 Welcome(身份精确)+ ChunkData 首泵恰 16(center-out 精确序)且
/// 中心 chunk 位级 == Superflat 生成值。`cargo test --workspace` / clippy -D
/// warnings / fmt 三段按任务书完成定义由编排层执行,不在测试体内。
#[tokio::test(start_paused = true)]
async fn acc8_workspace_regression_and_viewer_smoke() {
    use app::net::chunk_stream::MAX_LOADS_PER_PUMP;
    use viewer::client::{ViewEvent, ViewerClient};

    let config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 600,
        ..Config::default()
    };
    let net_cfg = app::net::NetConfig {
        listen_addr: "127.0.0.1:0".to_string(),
        read_timeout_secs: 600,
        observe_interval_ticks: 2,
        full_snapshot_interval_ticks: 60,
        pump: Default::default(),
    };
    let cadence = app::driver::ObserveCadence::from(&net_cfg);
    let (up_tx, up_rx) = mpsc::channel::<app::net::NetUp>(256);
    let hub = Arc::new(app::net::OutboundHub::new());
    let (bcast_tx, bcast_rx) = tokio::sync::broadcast::channel::<()>(1);
    let addr = app::net::serve(net_cfg, up_tx, bcast_rx, hub.clone())
        .await
        .expect("serve 真监听");
    std::mem::forget(bcast_tx);

    let session = Session::new(config.session_config());
    let (tx, rx) = mpsc::channel::<DriverCommand>(256);
    let store = StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(MemCheckpointStore::new()),
    );
    let (provider, _) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let sim_source = config.clone();
    let new_sim = Box::new(move |shard: ShardId| {
        let mut cfg = sim_source.shard_sim_config(shard);
        cfg.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        ShardSim::new(cfg, Box::new(SimpleRegistry::builtin()))
    });
    let driver = tokio::spawn(run(DriverArgs {
        config: config.clone(),
        session,
        store,
        log: CommandLog::new(),
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: Some(app::driver::ObserveLink {
            up: up_rx,
            hub: hub.clone(),
            cadence,
        }),
        spawn_next_seq: 0,
    }));

    let (events, _control) = ViewerClient::spawn(addr, 2).expect("spawn 连真服务器");
    // 双时钟推进:viewer 线程走真时间,事件桥 try_recv + 1ms 让片(viewer_client.rs 同款)。
    let slice = || std::thread::sleep(Duration::from_millis(1));
    let mut player = None;
    let mut chunks: Vec<(ChunkPos, Chunk)> = Vec::new();
    for _ in 0..800 {
        match events.try_recv() {
            Ok(ViewEvent::Welcome { player_id, .. }) if player.is_none() => {
                player = Some(player_id)
            }
            Ok(ViewEvent::Chunk { pos, chunk }) => {
                if chunks.len() < MAX_LOADS_PER_PUMP {
                    chunks.push((pos, chunk));
                }
            }
            Ok(_) => {}
            Err(std::sync::mpsc::TryRecvError::Disconnected) => break,
            Err(std::sync::mpsc::TryRecvError::Empty) => {
                slice();
                advance_ticks(1).await;
            }
        }
        if player.is_some() && chunks.len() >= MAX_LOADS_PER_PUMP {
            break;
        }
    }
    assert_eq!(
        player,
        Some(InputSource(1)),
        "AC-C8:真服务器 Welcome 必达(身份 1)"
    );
    assert_eq!(
        chunks.len(),
        MAX_LOADS_PER_PUMP,
        "AC-C8:ChunkData 首泵恰 16(实 {})",
        chunks.len()
    );
    let expected = {
        let mut v: Vec<ChunkPos> = app::net::chunk_stream::view_square(ChunkPos::new(0, 0), 2)
            .into_iter()
            .collect();
        v.sort_by_key(|p| {
            (
                app::net::chunk_stream::chebyshev(ChunkPos::new(0, 0), *p),
                p.x,
                p.z,
            )
        });
        v
    };
    assert_eq!(
        chunks.iter().map(|(p, _)| *p).collect::<Vec<_>>(),
        expected[..MAX_LOADS_PER_PUMP],
        "首泵 center-out 精确序"
    );
    let center = chunks
        .iter()
        .find(|(p, _)| *p == ChunkPos::new(0, 0))
        .expect("中心 chunk 在首泵")
        .1
        .clone();
    assert_eq!(
        center,
        Superflat::classic().generate(config.world_seed, ChunkPos::new(0, 0)),
        "ChunkData 位级 == 真数据源生成值"
    );

    tx.send(DriverCommand::Shutdown).await.expect("驱动存活");
    let outcome = driver.await.expect("驱动不 panic");
    assert!(outcome.ticks_run > 0);
}

// ---------------------------------------------------------------------------
// AC-C9:无重复装载 / 事件 fanout 精确 / checkpoint 字节确定
// ---------------------------------------------------------------------------

/// AC-C9:
/// ① 同拍装载计数无重复:任何一拍 `chunks_to_load` 桶内无重复元素,且与
///    该分片**已载集合**(apply_chunk_load 落实记录)交集为空;
/// ② 事件 fanout 精确:`broadcast_event` 在途信封数 == |shards_covering −
///    origin|(远片不进、origin 不收自己、逐 dest 恰一封,逐字段精确);
/// ③ checkpoint 字节确定:同一世界状态两次 bincode 编码 + 解码再编码全部
///    **逐字节相等**(bundle 无 HashMap 类不确定源;驱动同款 standard 配置)。
#[test]
fn acc9_no_duplicate_loads_or_fanout() {
    // —— ① 无重复装载(复用 AC-C1 场景几何)——
    let mut sched = Scheduler::new(SchedulerConfig {
        worker_count: 2,
        ..SchedulerConfig::default()
    });
    sched.register(void_sim(0, 0, true));
    sched.register(void_sim(1, 0, true));
    sched.set_observers(vec![Vec3::new(126.0, 0.0, 64.0)]);
    let mut loaded: BTreeSet<(ShardId, ChunkPos)> = BTreeSet::new();
    for _ in 0..400 {
        sched.tick();
        for (shard, decisions) in sched.take_activation_requests() {
            let unique: BTreeSet<ChunkPos> = decisions.chunks_to_load.iter().copied().collect();
            assert_eq!(
                unique.len(),
                decisions.chunks_to_load.len(),
                "{shard:?} 同拍请求桶内出现重复 chunk:{:?}",
                decisions.chunks_to_load
            );
            for cp in decisions.chunks_to_load {
                assert!(
                    loaded.insert((shard, cp)),
                    "{shard:?} 对已载 chunk {cp:?} 再次请求装载(重复装载 = 簿记 bug)"
                );
                sched.apply_chunk_load(shard, cp, ground_chunk(4));
            }
        }
    }

    // —— ② 事件 fanout 精确 ——
    // (0,0) 与 (1,0) 都覆盖边界爆心,(9,9) 远离:目标集 = {(1,0)}(origin
    // (0,0) 排除),信封恰 1 封;换 origin 再验一次对称集。
    sched
        .broadcast_event(
            shard_id(0, 0),
            CrossEvent::Detonate {
                center: Vec3::new(128.0, 3.0, 64.0),
                power: 6.0,
                radius: 1.0,
            },
            sched.tick_now(),
        )
        .expect("覆盖集含两片");
    let covering = sched.shards_covering(Vec3::new(128.0, 3.0, 64.0), 1.0);
    assert_eq!(
        covering,
        [shard_id(0, 0), shard_id(1, 0)]
            .into_iter()
            .collect::<BTreeSet<_>>(),
        "边界爆心的覆盖集 = 两片(远片不进)"
    );
    let snap = sched.snapshot_state();
    let dests: Vec<ShardId> = snap
        .router
        .envelopes
        .iter()
        .map(|e| e.destination)
        .collect();
    assert_eq!(
        dests,
        vec![shard_id(1, 0)],
        "fanout 恰 = 目标分片数(|covering − origin| = 1),逐 dest 恰一封"
    );
    // origin 换 (1,0):目标变 (0,0),封数仍 1。
    let snap2 = {
        sched
            .broadcast_event(
                shard_id(1, 0),
                CrossEvent::Detonate {
                    center: Vec3::new(128.0, 3.0, 64.0),
                    power: 6.0,
                    radius: 1.0,
                },
                sched.tick_now(),
            )
            .expect("同上");
        sched.snapshot_state()
    };
    let dests2: BTreeSet<ShardId> = snap2
        .router
        .envelopes
        .iter()
        .map(|e| e.destination)
        .collect();
    assert_eq!(
        dests2,
        [shard_id(0, 0), shard_id(1, 0)].into_iter().collect(),
        "两封在途:一封 per origin 目标(dest 多重集无重复)"
    );
    sched.shutdown();

    // —— ③ checkpoint 字节确定 ——
    let mut session = Session::new(session_cfg());
    session.register(void_sim(0, 0, true));
    session
        .move_observer(Vec3::new(8.0, 0.0, 8.0))
        .expect("unbounded");
    session.tick();
    let bundle = CheckpointBundle {
        format_version: app::driver::CHECKPOINT_FORMAT_VERSION,
        world: session.world_snapshot(),
        log_bytes: CommandLog::new().to_bytes(),
        spawn_next_seq: 0,
    };
    // #17:对拍生产编码(v5 = 版本前缀 + lz4 压缩体),压缩也必须确定。
    let bytes_a = app::driver::encode_checkpoint(&bundle);
    let bytes_b = app::driver::encode_checkpoint(&bundle);
    assert_eq!(bytes_a, bytes_b, "AC-C9:两次编码逐字节相等");
    let decoded = decode_bundle(&bytes_a);
    let bytes_c = app::driver::encode_checkpoint(&decoded);
    assert_eq!(bytes_a, bytes_c, "AC-C9:解码再编码仍逐字节相等(往返确定)");
    session.shutdown();
}

// ---------------------------------------------------------------------------
// 驱动级回归:pending_blockwrite_cells 延迟卸载窗(任务书 04 新增条目)
// ---------------------------------------------------------------------------

/// 驱动级回归:观察者迁移 + 实体凝固(Emergent 跨片信封)+ 卸载链全驱动跑通,
/// 断言零干扰结局:凝固格跨片副本一致(经检查点恢复逐字节)、库内持久化
/// 精确、卸载/回收最终完成、全程无 panic。
///
/// **为何本测试是回归而非竞态直测**(结构性三重排除,逐条给源码证):
/// `pending_blockwrite_cells` 延迟窗要拦的竞态 = "BlockWrite 信封在途
/// (N → N+1 一拍窗)时,接收方目标 chunk 恰被激活决策卸载" → 应用侧
/// `sim.rs:273-278` 的 fail-stop assert 触发。该竞态在现实现中三重不可
/// 构造:
/// ① **凝固要求 awake**:凝固桥对 kinematic/frozen 体整段跳过
///    (physics-sim `sim.rs::step` 的 `is_kinematic` 早退),Emergent 写入
///    ⟹ 实体 awake ⟹ 观察者 3D ≤ awake_unload 32m ⟹ 凝固格所在 chunk 中心
///    水平距观察者 ≤ 32 + 11.31 < 48 = chunk 卸载半径(滞回 >48m 才出卸载
///    候选)⟹ 信封一拍窗内目标 chunk 不可能进入卸载决策;
/// ② **despawn 双闸**:实体凝固前 entity_count>0 且观察者票 48m 内(同上
///    距离)→ 分片既不满足"空片"也不满足"无票",despawn 无从谈起;驱动
///    ②/③ 还各有一道 `pending_shard_destinations`/`deferred_blockwrites`
///    过滤(app/driver.rs orchestrate);
/// ③ **逃逸路径被冷却封死**:唯一能把观察者挪出 48m 的是移动命令,而目标
///    chunk 的卸载要过 180 拍冷却(`cooldown_ticks`,决策即簿记:凝固前的
///    装载即记簿),1 拍信封窗 ≪ 冷却窗;瞬移逃逸同样吃这道闸。
/// 结论:竞态窗口在真实配置下不可自然构造;若绕过①③硬造(直插 frozen 体
/// + 冷却 0 + 手工挪观察者),测的是被验收层的内部不变量而非生产路径,
/// 故按任务书红线交付本回归测试(零干扰结局逐字节断言)+ 本注释论证,
/// 并在 submit_result 的 deviations 中如实上报。
#[tokio::test(start_paused = true)]
async fn pending_blockwrite_cells_defer_window() {
    // 真驱动 + 真 Superflat + 凝固桥开(new_sim 生产默认);检查点密到 30 拍
    // 以便取"凝固后、卸载前"的中间快照验跨片副本一致。
    let mut config = Config {
        worker_count: 2,
        checkpoint_interval_ticks: 30,
        despawn_after_secs: 2, // 加速回收(工程细节,acg4 同款授权)
        ..Config::default()
    };
    // ①② 预算给足(§4.15 新键;邻域一拍装齐,预算 2 会拖拍序)。
    config.activation.data_budget_per_tick = 14;
    config.activation.collider_budget_per_tick = 14;
    let session = Session::new(config.session_config());
    let (tx, rx) = tokio::sync::mpsc::channel::<DriverCommand>(16);
    let checkpoints = SharedCheckpoints::default();
    let chunks_store = SharedChunks::default();
    let store = StoreWorker::new(
        Box::new(chunks_store.clone()),
        Box::new(checkpoints.clone()),
    );
    let (provider, new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    let driver = tokio::spawn(run(DriverArgs {
        config: config.clone(),
        session,
        store,
        log: CommandLog::new(),
        commands: rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: None,
        spawn_next_seq: 0,
    }));

    // ① 观察者贴边界 (120,-58,8):(0,0) 本体 + (1,0) 邻片(8m < 32m)双 ensure,
    //    凝固态实体的出生列 chunk 7 将同时是 (1,0) 的 margin 副本。
    tx.send(DriverCommand::MoveObserver {
        position: Vec3::new(120.0, GROUND_TOP_Y, 8.0),
    })
    .await
    .expect("驱动存活");
    advance_ticks(6).await;
    // ② 出生 (127.5,-55,8.5) → 落地面驻留 → sleep → 凝固(sleep 兜底无条件
    //    凝固,REQ-003 §4.4 语义):Emergent 写入 + BlockWrite 信封跨片一拍窗。
    tx.send(DriverCommand::Send {
        shard: shard_id(0, 0),
        command: SimCommand::SpawnDynamic {
            entity: eid(1),
            state: born(127.5, -55.0, 8.5, Vec3::ZERO),
            desc: EntityDesc::default(),
        },
    })
    .await
    .expect("驱动存活");
    // ③ 跑过凝固窗(落稳 ~80 拍 + 睡眠/凝固 ~几十拍;检查点每 30 拍一张)。
    advance_ticks(200).await;

    // 中间检查点:从"凝固后"的任一张里取两片的 chunk 7/8 副本,要求两片
    // 逐字节一致(跨片副本一致的位级证据),且全库至多一格差异(凝固格)。
    let generator = Superflat::classic();
    let mut solidified_chunk: Option<(ChunkKey, Chunk)> = None;
    for id in 1..=8u64 {
        let Some((_tick, blob)) = checkpoints.read(CheckpointId(id)).expect("读检查点") else {
            break;
        };
        let bundle = decode_bundle(&blob);
        let (a, b) = (
            bundle.world.shards.per_shard.get(&shard_id(0, 0)),
            bundle.world.shards.per_shard.get(&shard_id(1, 0)),
        );
        let (Some(a), Some(b)) = (a, b) else { continue };
        let sa = ShardSim::restore(a, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        let sb = ShardSim::restore(b, Box::new(SimpleRegistry::builtin())).expect("自有快照");
        if sa.entity_state(eid(1)).is_some() {
            continue; // 凝固前:实体还在,跳过
        }
        for cp in [ChunkPos::new(7, 0), ChunkPos::new(8, 0)] {
            let (Some(ca), Some(cb)) = (sa.voxel().get_chunk(cp), sb.voxel().get_chunk(cp)) else {
                continue;
            };
            assert_eq!(
                ca, cb,
                "凝固后中间快照:两片对 {cp:?} 的副本必须逐字节一致(BlockWrite N+1 完成)"
            );
            let pristine = generator.generate(config.world_seed, cp);
            if ca != &pristine {
                // 凝固格所在 chunk:与初生差**恰一格**。实体中心 x=127.5 →
                // snap 格 x=127 → chunk 7 局部 (15, ·, 8);顶面凝固 y=-60 →
                // 局部 ly = -60 - WORLD_Y_MIN = 4。
                let mut diff: Vec<(usize, usize, usize, BlockId)> = Vec::new();
                for y in 0..voxel::CHUNK_Y {
                    for z in 0..voxel::CHUNK_Z {
                        for x in 0..voxel::CHUNK_X {
                            if ca.get(x, y, z) != pristine.get(x, y, z) {
                                diff.push((x, y, z, ca.get(x, y, z)));
                            }
                        }
                    }
                }
                assert_eq!(
                    diff,
                    vec![(15, 4, 8, EntityDesc::default().block)],
                    "{cp:?} 与初生差异必须恰为顶面凝固格 (15,4,8)=stone(精确集合)"
                );
                solidified_chunk = Some((key(0, 0, cp.x, cp.z), ca.clone()));
            }
        }
        break;
    }
    let (sk, sc) = solidified_chunk.expect("凝固必须恰产出一个持久化 chunk 修改");

    // ④ 观察者瞬移远离 → pending 窗早已关闭(一拍窗 ≪ 迁移耗时),卸载链
    //    完整走完:chunk 逐卸载 + 空片无票 idle(2s)→ 两片回收,零 panic。
    tx.send(DriverCommand::MoveObserver {
        position: Vec3::new(50_000.0, 0.0, 8.0),
    })
    .await
    .expect("驱动存活");
    advance_ticks(260).await; // idle 120 拍 + 远区 ensure 片 idle 再回收余量
    tx.send(DriverCommand::Shutdown).await.expect("驱动存活");
    let outcome = driver
        .await
        .expect("驱动不 panic(竞态若被触发,应用侧 assert 必先红)");

    // 回归结局(精确面):
    // a) 远走前两片都载过、远走后确实卸载/回收(chunks_unloaded/despawn 非零
    //    = 覆盖性旁证;主断言在 b/c 的位级)。
    assert!(
        outcome.chunks_unloaded >= 1 && outcome.shards_despawned >= 2,
        "卸载链确实运行(unloaded={}, despawned={})——回归场景覆盖性旁证",
        outcome.chunks_unloaded,
        outcome.shards_despawned
    );
    // b) 持久层最终值 = 生成器 + 恰凝固格(逐字节;延迟过滤未污染写库值)。
    let stored = ChunkStore::get(&*chunks_store.0.lock().expect("锁"), sk)
        .expect("读库")
        .expect("凝固 chunk 必已落库(owner 卸载路径)");
    assert_eq!(stored, sc, "库内 chunk 与凝固后副本逐字节一致");
    // c) 终末检查点的分片键集:观察者已挪至 (50000,·,8)。手算 ensure 集
    //    (shards_within r=32):chunk (3125,0) → shard (390,0)(z=8 在核内);
    //    shard (390,-1) 核心 z∈[-128,0),点距 8m < 32m → 同集。**精确集 =
    //    {(390,-1),(390,0)}**(原两片完整回收,无半死残留)。
    let (_tick, blob) = checkpoints
        .read_latest()
        .expect("读最新")
        .expect("终末检查点");
    let final_bundle = decode_bundle(&blob);
    let far_expected = vec![shard_id(390, -1), shard_id(390, 0)];
    assert_eq!(
        final_bundle
            .world
            .shards
            .per_shard
            .keys()
            .copied()
            .collect::<Vec<_>>(),
        far_expected,
        "终末快照键集 = 观察者远端 ensure 两片(原两片完整回收)"
    );
}

// ---------------------------------------------------------------------------
// REQ-007 AC-T2(07 补锚):REQ-005 owner/holder 既有验收零回归
// ---------------------------------------------------------------------------

/// AC-T2(07 任务书"owner/holder 既有验收零回归"显式锚):REQ-007 激活票据
/// 接管 owner/holder 编排(票据 ensure / 激活装卸)后,REQ-005 的既有验收面
/// 不回归——分两段,各走生产路径:
/// ① **holder 注册精确 + 装载幂等**(调度器级,acc1 同几何):观察者票贴
///    边界,margin 重叠列 chunk 必为双 holder、装载恰一次;卸载后 holder
///    登记撤销(经 `apply_chunk_unload` 走调度器同一条撤销路径);
/// ② **owner-only 持久化 + 重载取库值**(Session 级,acc3 语义):观察者
///    移出 48 m 卸载半径,owner 侧卸载返回修改 chunk(驱动写库条件)、
///    holder 侧返回 None(零写入);观察者折返后重载,两片体素 = 修改值
///    (持久层优先,不被生成器初生覆盖)。
///
/// 会变红的实现:票据化后 ensure/卸载绕过 chunk_holders 登记(holder 表缺失
/// 或残留);扩张副本被写库(owner-only 被破坏);重复装载(acc1 ① 幂等)。
#[test]
fn acc_t2_owner_holder_semantics_survive_ticket_orchestration() {
    let seed = WorldSeed(7);

    // —— ① 调度器级:holder 注册精确 + 装载幂等 + 卸载撤销 ——
    let mut sched = Scheduler::new(SchedulerConfig {
        worker_count: 2,
        ..SchedulerConfig::default()
    });
    sched.register(void_sim(0, 0, true));
    sched.register(void_sim(1, 0, true));
    sched.set_observers(vec![Vec3::new(126.0, 0.0, 64.0)]);
    let s0 = shard_id(0, 0);
    let s1 = shard_id(1, 0);
    let mut applied: BTreeMap<(ShardId, ChunkPos), u32> = BTreeMap::new();
    for _ in 0..400 {
        sched.tick();
        for (shard, decisions) in sched.take_activation_requests() {
            for cp in decisions.chunks_to_load {
                sched.apply_chunk_load(shard, cp, ground_chunk(4));
                *applied.entry((shard, cp)).or_insert(0) += 1;
            }
        }
    }
    // 同一 (shard, chunk) 恰一次装载(acc1 ④;票据化簿记不得重复请求)。
    for ((shard, cp), n) in &applied {
        assert_eq!(*n, 1, "{shard:?} 重复装载 {cp:?} 共 {n} 次(票据化回归)");
    }
    // margin 重叠列(x=7/8)必为双 holder(acc1 ② 核心面,非退化;owner 按
    // chunk 位置推导——x=8 列的 owner 是分片 (1,0))。
    for cx in [7, 8] {
        let cp = ChunkPos::new(cx, 3);
        let owner = ShardId::new(WorldId(0), DimensionId(0), cp.shard_of());
        let holders = sched.holders_of(owner, cp);
        assert_eq!(
            holders,
            [s0, s1].into_iter().collect(),
            "margin 重叠列 chunk {cp:?} 必为双 holder:{holders:?}(票据化回归)"
        );
    }
    // 卸载一份副本 → 该副本的 holder 登记撤销(另一份保留);登记与装载态
    // 同步(acc1 ② 的对偶面;走 apply_chunk_unload 同一条撤销路径)。
    let c7 = ChunkPos::new(7, 3);
    assert!(
        applied.contains_key(&(s0, c7)) && applied.contains_key(&(s1, c7)),
        "非退化:c7 应为双 holder 重叠列(实得 {:?})",
        (
            applied.contains_key(&(s0, c7)),
            applied.contains_key(&(s1, c7))
        )
    );
    assert!(
        sched.apply_chunk_unload(s1, c7).is_none(),
        "非 owner 副本:None(零写入条件)"
    );
    let holders_after = sched.holders_of(s0, c7);
    assert_eq!(
        holders_after,
        [s0].into_iter().collect(),
        "holder 卸载后登记撤销,只剩 owner:{holders_after:?}"
    );
    sched.shutdown();

    // —— ② Session 级:owner-only 持久化 + 折返重载取库值 ——
    let dir = tempfile::tempdir().expect("临时目录");
    let chunks_store = Box::new(RedbStore::open(&dir.path().join("chunks.redb")).expect("真 redb"));
    let worker = StoreWorker::new(chunks_store, Box::new(MemCheckpointStore::new()));
    let provider = ChunkProvider::new(&worker, Box::new(Superflat::classic()), seed);
    let mut session = Session::new(session_cfg());
    session.register(void_sim(0, 0, true));
    session.register(void_sim(1, 0, true));
    session
        .move_observer_from(InputSource(2), Vec3::new(126.0, 0.0, 64.0))
        .expect("观察者入表");
    let mut c7_loaded = false;
    for _ in 0..400 {
        session.tick();
        for (shard, decisions) in session.take_activation_requests() {
            for cp in decisions.chunks_to_load {
                let key = ChunkKey {
                    world: WorldId(0),
                    dim: DimensionId(0),
                    pos: cp,
                };
                let chunk = provider
                    .get(key)
                    .unwrap_or_else(|| panic!("provider.get 返回 None:{key:?}"));
                session.apply_chunk_load(shard, cp, chunk);
                if cp == c7 {
                    c7_loaded = true;
                }
            }
        }
    }
    assert!(c7_loaded, "前置:c7 经票据驱动装载(观察者贴边界)");

    // 边界修改 → 观察者移出 48 m 卸载半径 → 激活卸载:owner 返回修改值、
    // holder 先卸返回 None(acc3 语义;激活卸载路径)。
    let generated = Superflat::classic().generate(seed, c7);
    let mut expect = generated;
    // 边界修改落在 c7 = (7,3) 内(x=120∈[112,128)、z=56∈[48,64);局部格
    // (8, 0, 8) 与 acc3 同格位,只换 chunk 行。
    expect.set(8, 0, 8, BlockId(2));
    session
        .set_block(s0, BlockPos::new(120, types::WORLD_Y_MIN, 56), BlockId(2))
        .expect("边界修改");
    session.tick();
    session
        .move_observer_from(InputSource(2), Vec3::new(2000.0, 0.0, 64.0))
        .expect("观察者移出卸载半径");
    let (mut holder_return, mut owner_return) = (None, None);
    // s0 与 s1 对 c7 的卸载预算序不同(s0 的 c7 = 自家 core,距离序靠后;
    // s1 的 c7 = 扩张副本,同款靠后)——不设提前 break,跑满冷却窗让两侧
    // 决策自然到齐,再按记录值断言。
    for _ in 0..600 {
        session.tick();
        for (shard, decisions) in session.take_activation_requests() {
            for cp in decisions.chunks_to_unload {
                let returned = session.apply_chunk_unload(shard, cp);
                if cp == c7 {
                    if shard == s1 {
                        holder_return = returned;
                    } else if shard == s0 {
                        owner_return = returned;
                    }
                }
            }
        }
        if owner_return.is_some() && holder_return.is_some() {
            break;
        }
    }
    let owner_chunk = owner_return.expect("前置:观察者远移后 owner 侧 ① 应被卸载");
    assert_eq!(
        holder_return, None,
        "holder 侧(先卸)返回 None(零写入;owner-only 未被票据化破坏)"
    );
    assert_eq!(owner_chunk, expect, "owner 卸载返回修改值(可持久化)");
    // 驱动写库条件 = Some(orchestrate 同款):owner chunk 修改值入持久层。
    worker.put_chunks(vec![(
        ChunkKey {
            world: WorldId(0),
            dim: DimensionId(0),
            pos: c7,
        },
        owner_chunk,
    )]);
    assert!(worker.flush().recv().expect("flush ack").is_ok(), "屏障");

    // 观察者折返 → 票据请求重载:两片体素 = 修改值(库值优先,非生成器初生)。
    session
        .move_observer_from(InputSource(2), Vec3::new(126.0, 0.0, 64.0))
        .expect("观察者折返");
    let mut reloaded = BTreeSet::new();
    for _ in 0..600 {
        session.tick();
        for (shard, decisions) in session.take_activation_requests() {
            for cp in decisions.chunks_to_load {
                let key = ChunkKey {
                    world: WorldId(0),
                    dim: DimensionId(0),
                    pos: cp,
                };
                let chunk = provider.get(key).expect("重载必命中库值");
                if cp == c7 {
                    assert_eq!(
                        chunk, expect,
                        "重载取回修改值(持久层优先,不被生成器初生覆盖)"
                    );
                    reloaded.insert(shard);
                }
                session.apply_chunk_load(shard, cp, chunk);
            }
        }
        if reloaded.len() == 2 {
            break;
        }
    }
    assert_eq!(
        reloaded,
        [s0, s1].into_iter().collect(),
        "折返后两片照常重载 c7(卸载冷却后;票据路径不吞请求)"
    );
    session.shutdown();
    worker.shutdown().expect("正常关停");
}
