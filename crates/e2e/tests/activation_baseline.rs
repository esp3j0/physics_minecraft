//! AC-T1 基线录制(REQ-007 任务书 00;golden 文件 = AC-T1 回归标尺)。
//!
//! 在**未改动**的 `8702670` 行为上录制观察者驱动激活的现状:逐拍记录
//! chunk 装载集、实体冻结/唤醒集、已注册分片集(D16)。场景含观察者移动、
//! 折返、远离、一次 ≥ 200 m 瞬间跳跃(D16)、移除;常驻体(actor=true)在
//! 任何冻结事件前都已静止(AC-T1 前提,测试内断言)。
//!
//! 观测面(N1 裁定):直接调用生产装配 `app::driver::wire_provider_and_new_sim`
//! (生成器/种子/`ShardSimConfig` 与生产一致),在 e2e 内按
//! `driver::orchestrate` 的固定顺序复刻每拍末编排(观察者 32 m 邻片
//! ensure → 在途信封目的分片 ensure → 激活装卸含 BlockWrite 延迟卸载 →
//! despawn)。`shards_within` 是 driver 私有函数,在测试内逐行复刻
//! (driver.rs:1096-1119,行为对齐)。
//!
//! golden(B4):纯文本,每拍一行,只存排序离散列表;
//! `REQ007_RECORD_BASELINE=1` 时录制并覆写,默认只比对;
//! golden 入库后未经总架构师批准不得重录。

use std::collections::BTreeSet;
use std::path::PathBuf;

use app::config::Config;
use messaging::{InputSource, SimCommand};
use sim_api::session::Session;
use storage::traits::ChunkKey;
use types::glam::Vec3;
use types::{ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, ShardId, ShardPos, WorldId};
use voxel::BlockRegistry;

/// 测试用分片身份(与其他 e2e 套件同风格)。
fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn new_registry() -> impl Fn() -> Box<dyn BlockRegistry> + Send + 'static {
    || Box::new(voxel::SimpleRegistry::builtin())
}

/// `driver::shards_within` 的测试复刻(driver.rs:1096-1119 逐行对齐):
/// 点 p 周围 radius 内、核心 AABB 水平距离 ≤ radius 的分片,升序由双层
/// 循环的 (sx, sz) 顺序决定(与生产一致,调用方不排序)。
fn shards_within(p: Vec3, radius: f32) -> Vec<ShardId> {
    const SHARD_M: f32 = (types::SHARD_CHUNKS * types::CHUNK_SIZE) as f32;
    let min_x = ((p.x - radius) / SHARD_M).floor() as i32;
    let max_x = ((p.x + radius) / SHARD_M).floor() as i32;
    let min_z = ((p.z - radius) / SHARD_M).floor() as i32;
    let max_z = ((p.z + radius) / SHARD_M).floor() as i32;
    let mut out = Vec::new();
    for sx in min_x..=max_x {
        for sz in min_z..=max_z {
            // 点到该分片核心 AABB 的水平距离(每轴 clamp 到盒外量)。
            let (cx, cz) = (sx as f32 * SHARD_M, sz as f32 * SHARD_M);
            let dx = (cx - p.x).max(0.0).max(p.x - (cx + SHARD_M));
            let dz = (cz - p.z).max(0.0).max(p.z - (cz + SHARD_M));
            if (dx * dx + dz * dz).sqrt() <= radius {
                out.push(shard_id(sx, sz));
            }
        }
    }
    out
}

// ---- golden 格式(B4.3:纯文本,每拍一行;空列表留空字段;A1:装载集按分片)----
// <tick>|load=<sx:sz[x,z;x,z]/sx:sz[...]>
//       |frozen=<w:s;...>|woke=<w:s;...>|shards=<wx:dim:sx:sz;...>
// load:分片按 ShardId 升序,片内 chunk 按 (x,z) 升序;已注册但零装载的
// 分片**仍记录**(空方括号),与未注册(整片不出现)显式区分;
// 扫描范围 = 该分片 activation_region(core + margin 环,REQ-005 D4)。

/// 逐分片装载观测:每分片 (ShardId, 已装载 chunk 有序集),分片键序。
type LoadedPerShard = Vec<(ShardId, Vec<ChunkPos>)>;

fn fmt_loaded_per_shard(per_shard: &LoadedPerShard) -> String {
    per_shard
        .iter()
        .map(|(s, chunks)| {
            let inner = chunks
                .iter()
                .map(|cp| format!("{},{}", cp.x, cp.z))
                .collect::<Vec<_>>()
                .join(";");
            format!("{}:{}[{}]", s.pos.x, s.pos.z, inner)
        })
        .collect::<Vec<_>>()
        .join("/")
}

fn fmt_entities(list: &[EntityId]) -> String {
    list.iter()
        .map(|e| format!("{}:{}", e.origin_shard(), e.local_seq()))
        .collect::<Vec<_>>()
        .join(";")
}

fn fmt_shards(set: &BTreeSet<ShardId>) -> String {
    set.iter()
        .map(|s| format!("{}:{}:{}:{}", s.world.0, s.dimension.0, s.pos.x, s.pos.z))
        .collect::<Vec<_>>()
        .join(";")
}

/// 一拍的观测快照。
struct Frame {
    tick: u64,
    /// 逐分片装载集(A1);分片 ShardId 升序,片内 chunk (x,z) 升序。
    loaded: LoadedPerShard,
    /// 逐分片 ② 集(REQ-007 任务 01 四项比对;同范围同序;**不入 golden**,
    /// golden 格式冻结——比对测试经字段直接断言)。
    colliders: LoadedPerShard,
    frozen: Vec<EntityId>,
    woke: Vec<EntityId>,
    shards: BTreeSet<ShardId>,
}

impl Frame {
    fn to_line(&self) -> String {
        // 冻结/唤醒集已按 EntityId 升序(激活决策序),再排序一次防御。
        let mut frozen = self.frozen.clone();
        frozen.sort();
        let mut woke = self.woke.clone();
        woke.sort();
        format!(
            "{}|load={}|frozen={}|woke={}|shards={}",
            self.tick,
            fmt_loaded_per_shard(&self.loaded),
            fmt_entities(&frozen),
            fmt_entities(&woke),
            fmt_shards(&self.shards),
        )
    }

    fn parse_line(line: &str) -> (u64, String) {
        // 比对按整行全等;tick 单独提出供分叉报告。
        let (head, _) = line.split_once('|').expect("golden 行必有 tick 前缀");
        (head.parse().expect("golden tick 数字"), line.to_string())
    }
}

fn golden_path() -> PathBuf {
    // CARGO_MANIFEST_DIR = crates/e2e。
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden/ac_t1_baseline.txt")
}

fn read_golden_lines() -> Vec<String> {
    let text = std::fs::read_to_string(golden_path()).expect("golden 文件应已入库");
    text.lines()
        .filter(|l| !l.trim().is_empty())
        .map(str::to_string)
        .collect()
}

// ---- 场景脚本(拍数与观察者坐标写死;确定性由 AC1 位级纪律保证)----

/// 观察者航点:(起点 (8,8))→ 东移跨 chunk → 折返 → 远离(> 48 m)→
/// 跳跃 ≥ 200 m → 移除。每段拍数写死。
/// 观察者/实体同层高度(Superflat 顶面 -60 + half_size 0.5)。
const BASE_Y: f32 = -59.5;
const SETTLE_TICKS: u64 = 40; // 常驻体落定(静止窗口 ≥ 20 拍)后设观察者
const LEG_A_TICKS: u64 = 60; // (8,8) → (72,8):跨 chunk(4),触发装载
const LEG_B_TICKS: u64 = 60; // 折返 (72,8) → (8,8)(§3.3-3 竞态场景)
const LEG_C_TICKS: u64 = 60; // (8,8) → (200,8):> 48 m,卸载窗口
const JUMP_TICKS: u64 = 40; // 跳跃 (200,8) → (8,508):≥ 200 m(D16)
const REMOVE_IDLE_TICKS: u64 = 1801; // 移除观察者 → idle(1800 拍)满 → despawn

/// 观察者第 `tick_in_leg` 拍的位置(线性插值;移动速度 1 m/拍,慢于物理
/// 稳定要求,快于装载预算可观察)。
fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// 跑完整场景并逐拍收集 Frame(录制与比对共用;录制模式多写文件)。
fn run_scenario() -> Vec<Frame> {
    let config = Config::default(); // Superflat + WorldSeed(0),生产默认
    let store = storage::worker::StoreWorker::new(
        Box::new(storage::mem::MemChunkStore::new()),
        Box::new(storage::mem::MemCheckpointStore::new()),
    );
    let mut session = Session::new(config.session_config());
    let (provider, mut new_sim) =
        app::driver::wire_provider_and_new_sim(config.clone(), &store, new_registry());
    // 初始分片(0,0)预注册(对齐生产 bootstrap:首分片装配期注册,chunk
    // 仍只经激活请求 + provider.get 装载;后续分片由每拍 ensure 注册)。
    session.register(new_sim(shard_id(0, 0)));

    let source = InputSource(1);
    let me = source;
    let actor_a = EntityId::from_parts(1, 1);
    let actor_b = EntityId::from_parts(1, 2);

    let mut frames = Vec::new();
    let mut tick: u64 = 0;

    /// 每拍循环体:tick → 按 orchestrate 顺序编排 → 观测。
    /// 闭包借用太繁,用宏展开(单处定义,场景各段共用)。
    macro_rules! step_and_observe {
        ($frames:expr, $tick:expr, $session:expr) => {{
            let summary = $session.tick();
            let _ = summary;

            // —— 复刻 orchestrate(driver.rs:939-1057,顺序固定)——
            let ensure_radius = config.activation.awake_unload_radius as f32; // 32 m
            // ① 观察者分片 + 邻片 ensure。
            let mut ensured: BTreeSet<ShardId> = BTreeSet::new();
            for (_src, position) in $session.observers() {
                for shard in shards_within(position, ensure_radius) {
                    if ensured.insert(shard) && !$session.shard_ids().contains(&shard) {
                        $session.register(new_sim(shard));
                    }
                }
            }
            // ①′ 在途信封目的分片 ensure。
            for dest in $session.pending_shard_destinations() {
                if ensured.insert(dest) && !$session.shard_ids().contains(&dest) {
                    $session.register(new_sim(dest));
                }
            }
            // ② 激活装卸(含 BlockWrite 待写格延迟卸载)。
            let deferred_blockwrites = $session.pending_blockwrite_cells();
            let mut frozen_all: Vec<EntityId> = Vec::new();
            let mut woke_all: Vec<EntityId> = Vec::new();
            for (shard, decisions) in $session.take_activation_requests() {
                frozen_all.extend(&decisions.entities_to_freeze);
                woke_all.extend(&decisions.entities_to_wake);
                for pos in decisions.chunks_to_load {
                    let key = ChunkKey {
                        world: shard.world,
                        dim: shard.dimension,
                        pos,
                    };
                    let chunk = provider
                        .get(key)
                        .unwrap_or_else(|| panic!("provider.get 返回 None(生成器缺陷):{key:?}"));
                    $session.apply_chunk_load(shard, pos, chunk);
                }
                for pos in decisions.chunks_to_unload {
                    if deferred_blockwrites
                        .get(&shard)
                        .is_some_and(|chunks| chunks.contains(&pos))
                    {
                        continue;
                    }
                    let _ = $session.apply_chunk_unload(shard, pos);
                }
            }
            // ③ despawn(在途信封目的分片跳过;存储批 = mem,无需屏障)。
            let pending_destinations = $session.pending_shard_destinations();
            for shard in $session.take_despawn_requests() {
                if !$session.shard_ids().contains(&shard) {
                    continue;
                }
                if pending_destinations.contains(&shard) {
                    continue;
                }
                let _ = $session.unload_shard(shard);
            }

            // —— 观测(A1:逐分片,范围 = activation_region core+margin)——
            let shard_list = $session.shard_ids(); // BTreeMap 键序 = ShardId 升序
            let margin = config.region_margin_chunks(); // 与 ShardSim 激活区域同源
            let mut loaded: LoadedPerShard = Vec::new();
            let mut colliders: LoadedPerShard = Vec::new();
            for &s in &shard_list {
                // 复刻 ShardSim::activation_region()(sim.rs:485-487):
                // ChunkRegion::core(shard).expanded(margin)。
                let core_min = s.pos.min_chunk();
                let min = ChunkPos::new(core_min.x - margin, core_min.z - margin);
                let side = types::SHARD_CHUNKS + 2 * margin;
                let mut chunks: Vec<ChunkPos> = Vec::new();
                let mut cols: Vec<ChunkPos> = Vec::new();
                for dx in 0..side {
                    for dz in 0..side {
                        let cp = ChunkPos::new(min.x + dx as i32, min.z + dz as i32);
                        if $session.is_chunk_loaded(s, cp) {
                            chunks.push(cp);
                        }
                        if $session.has_collider(s, cp) {
                            cols.push(cp);
                        }
                    }
                }
                // (x,z) 升序:双层循环 dx 外 dz 内已按行序产出,天然升序;
                // 空分片仍记录(显式区分"注册但全空"与"未注册")。
                loaded.push((s, chunks));
                colliders.push((s, cols));
            }
            frozen_all.sort();
            woke_all.sort();
            $frames.push(Frame {
                tick: $tick,
                loaded,
                colliders,
                frozen: frozen_all,
                woke: woke_all,
                shards: $session.shard_ids().into_iter().collect(),
            });
            $tick += 1;
        }};
    }

    // —— 布景(不记录;对齐生产"玩家先连入才有 spawn"):设观察者启用激活
    // → 等观察者 24 m 内 chunk 装载(预算 2/拍、180 拍冷却 → 13 个 chunk
    // 需 ≥ 7 拍装载,再留 60 拍裕度)→ 贴地 spawn 两个常驻体 → 推进至静止。
    session
        .move_observer_from(me, Vec3::new(8.0, BASE_Y, 8.0))
        .expect("设观察者");
    let mut pre_frames = Vec::new();
    for _ in 0..70 {
        step_and_observe!(pre_frames, tick, session);
    }
    assert!(
        session.is_chunk_loaded(shard_id(0, 0), ChunkPos::new(0, 0)),
        "布景:观察者 24 m 内 chunk (0,0) 应已装载"
    );
    session
        .send(
            shard_id(0, 0),
            SimCommand::SpawnDynamic {
                entity: actor_a,
                state: EntityState {
                    translation: Vec3::new(12.0, -59.5, 12.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            },
        )
        .expect("spawn actor_a");
    session
        .send(
            shard_id(0, 0),
            SimCommand::SpawnDynamic {
                entity: actor_b,
                state: EntityState {
                    translation: Vec3::new(16.0, -59.5, 16.0),
                    ..EntityState::default()
                },
                desc: EntityDesc::default(),
            },
        )
        .expect("spawn actor_b");
    session
        .send(
            shard_id(0, 0),
            SimCommand::MarkActor {
                entity: actor_a,
                actor: true,
            },
        )
        .expect("mark actor_a");
    session
        .send(
            shard_id(0, 0),
            SimCommand::MarkActor {
                entity: actor_b,
                actor: true,
            },
        )
        .expect("mark actor_b");
    // 场景前提(AC-T1):常驻体已静止(≥ 20 拍位移 < 3 mm)——断言,否则
    // panic(前提本身被锚定)。SETTLE_TICKS=40 拍足够落定(0.5 m 落差)。
    // A2:布景全程走同一套编排(step_and_observe! + 丢弃帧),落定与静止
    // 核验期间的激活请求/ensure/despawn 与生产循环一致,不丢拍。
    for _ in 0..SETTLE_TICKS {
        step_and_observe!(pre_frames, tick, session);
    }
    for e in [actor_a, actor_b] {
        let s1 = session
            .entity_state(shard_id(0, 0), e)
            .expect("常驻体应仍在表");
        for _ in 0..25 {
            step_and_observe!(pre_frames, tick, session);
        }
        let s2 = session.entity_state(shard_id(0, 0), e).expect("仍在表");
        assert!(
            (s1.translation - s2.translation).length() < 0.003,
            "常驻体 {e:?} 未在基线记录开始前静止(AC-T1 前提失败):{:?} → {:?}",
            s1.translation,
            s2.translation
        );
    }
    // —— 基线记录自此开始(航点逐拍记录;布景段的装载不进 golden,
    // 移动/折返/远离/跳跃段的装载变化才是 AC-T1 标尺内容)。
    pre_frames.clear(); // 布景帧不进 golden(编排副作用已发生)。
    tick = 0; // golden 行号从记录起点计(布景全部结束后重置)。
    // 段 A:(8,8) → (72,8),60 拍(跨 chunk 4,触发装载)。
    for k in 0..LEG_A_TICKS {
        let t = (k + 1) as f32 / LEG_A_TICKS as f32;
        let x = lerp(8.0, 72.0, t);
        session
            .move_observer_from(me, Vec3::new(x, BASE_Y, 8.0))
            .expect("移动");
        step_and_observe!(frames, tick, session);
    }
    // 段 B:折返 (72,8) → (8,8),60 拍。
    for k in 0..LEG_B_TICKS {
        let t = (k + 1) as f32 / LEG_B_TICKS as f32;
        let x = lerp(72.0, 8.0, t);
        session
            .move_observer_from(me, Vec3::new(x, BASE_Y, 8.0))
            .expect("折返");
        step_and_observe!(frames, tick, session);
    }
    // 段 C:(8,8) → (200,8),60 拍(> 48 m,卸载窗口)。
    for k in 0..LEG_C_TICKS {
        let t = (k + 1) as f32 / LEG_C_TICKS as f32;
        let x = lerp(8.0, 200.0, t);
        session
            .move_observer_from(me, Vec3::new(x, BASE_Y, 8.0))
            .expect("远离");
        step_and_observe!(frames, tick, session);
    }
    // 段 D:跳跃 (200,8) → (8,508),单拍瞬移 ≥ 200 m(D16),停留 40 拍;
    // 再跳回 (8,8)(同样 ≥ 200 m):观察者回到实体 24 m 内 → 冻结实体被
    // 唤醒(远离段冻结,flip 簿记已过 180 拍冷却)→ golden 覆盖唤醒事件。
    session
        .move_observer_from(me, Vec3::new(8.0, BASE_Y, 508.0))
        .expect("跳跃");
    for _ in 0..JUMP_TICKS {
        step_and_observe!(frames, tick, session);
    }
    session
        .move_observer_from(me, Vec3::new(8.0, BASE_Y, 8.0))
        .expect("跳回");
    for _ in 0..JUMP_TICKS {
        step_and_observe!(frames, tick, session);
    }
    // 段 E:移除观察者 → idle 1801 拍 → 空片 despawn(记录全程)。
    session.remove_observer_from(me).expect("移除观察者");
    for _ in 0..REMOVE_IDLE_TICKS {
        step_and_observe!(frames, tick, session);
    }

    // 收尾注记(§3.3-2 现状语义的锚定):观察者移除 + idle 满后,分片
    // **不** despawn——常驻体仍在实体表钉住分片(despawn 要求空片;这正是
    // REQ-007 要修的"冻结体/实体钉死分片"缺口)。基线记录该现状;
    // 01+ 重放时同值即零回归。若未来任务改此语义,golden 需经总架构师批准重录。
    let tail = frames.last().expect("有末帧");
    assert!(
        !tail.shards.is_empty(),
        "现状语义:常驻体钉住分片,移除观察者后不应 despawn(§3.3-2)"
    );

    store.shutdown().expect("关停存储");
    session.shutdown();
    frames
}

/// 抽样非退化断言(录制与比对都跑):场景必须真的产生装载/卸载/风暴/despawn。
fn assert_non_degenerate(frames: &[Frame]) {
    // A1:loaded 逐分片;总量 = 各片装载之和。
    let total = |f: &Frame| f.loaded.iter().map(|(_, c)| c.len()).sum::<usize>();
    let ever_loaded = frames.iter().any(|f| total(f) > 0);
    assert!(ever_loaded, "非退化:场景应发生 chunk 装载");
    let load_grew = frames.windows(2).any(|w| total(&w[1]) > total(&w[0]));
    assert!(load_grew, "非退化:装载集应增长过(观察者移动触发)");
    let shrank = frames.windows(2).any(|w| total(&w[1]) < total(&w[0]));
    assert!(shrank, "非退化:远离段应发生 chunk 卸载");
    // 跳跃拍(D16):单拍内注册分片数显著增加(≥ 2:目的分片 + 邻片)。
    // 记录起点 = 段 A 第一拍(布景不记录),跳跃发生在段 C 末的下一拍。
    let jump_at = LEG_A_TICKS + LEG_B_TICKS + LEG_C_TICKS;
    let jump_frame = frames
        .iter()
        .find(|f| f.tick == jump_at)
        .expect("跳跃拍应有帧");
    let prev = frames
        .iter()
        .find(|f| f.tick == jump_at - 1)
        .expect("跳跃前一拍应有帧");
    assert!(
        jump_frame.shards.len() >= prev.shards.len() + 2,
        "非退化:跳跃拍应 ensure 新分片(风暴路径);prev={:?} jump={:?}",
        prev.shards,
        jump_frame.shards
    );
    let ever_frozen = frames.iter().any(|f| !f.frozen.is_empty());
    assert!(ever_frozen, "非退化:远离段应发生冻结(常驻体静止 + > 32 m)");
    let ever_woke = frames.iter().any(|f| !f.woke.is_empty());
    assert!(ever_woke, "非退化:跳回后应发生唤醒(冷却已过)");
    // 收尾(§3.3-2 现状):常驻体钉住分片,末帧分片不空。
    assert!(
        !frames.last().expect("有末帧").shards.is_empty(),
        "非退化:末尾分片被常驻体钉住(§3.3-2 现状)"
    );
}

/// 录制(仅 `REQ007_RECORD_BASELINE=1`):跑场景 → 写 golden → 抽样断言。
#[test]
fn record_baseline_golden() {
    if std::env::var("REQ007_RECORD_BASELINE").as_deref() != Ok("1") {
        println!("跳过录制:未设 REQ007_RECORD_BASELINE=1(golden 只读比对模式)");
        return;
    }
    let frames = run_scenario();
    assert_non_degenerate(&frames);
    let text = frames
        .iter()
        .map(Frame::to_line)
        .collect::<Vec<_>>()
        .join("\n");
    let byte_len = text.len();
    std::fs::create_dir_all(golden_path().parent().unwrap()).expect("建 golden 目录");
    std::fs::write(golden_path(), &text).expect("写 golden");
    println!(
        "基线已录制:{} 拍,{} 字节 → {}",
        frames.len(),
        byte_len,
        golden_path().display()
    );
}

/// 比对(默认运行):同场景重跑,逐拍与入库 golden 全等;
/// 失败信息给出第一个分叉行与两侧差异。
#[test]
fn baseline_replay_matches_golden() {
    let golden = read_golden_lines();
    let frames = run_scenario();
    assert_non_degenerate(&frames);
    assert_eq!(
        frames.len(),
        golden.len(),
        "拍数与 golden 不一致(场景脚本或节拍变动?golden 冻结,不得顺手重录)"
    );
    for (f, g) in frames.iter().zip(&golden) {
        let (gtick, gline) = Frame::parse_line(g);
        let line = f.to_line();
        assert_eq!(f.tick, gtick, "拍号错位");
        assert_eq!(
            line, *gline,
            "拍 {} 基线分叉(第一个分叉行):\n  实际:{line}\n  golden:{gline}",
            f.tick
        );
    }
}

/// 四项比对(REQ-007 任务 01/B4.4;任务书测试
/// `ac_t1_matches_golden_after_tier_split`):
/// ① ① 集 == golden chunk 集;③ 冻结/唤醒集 == golden;④ 分片集 == golden
/// ——三项 + 拍号经**整行严格相等**(与 `baseline_replay_matches_golden` 同标尺,
/// 零推迟唤醒差异:唤醒集逐拍相等,I2 在 03/04 才做,不允许 §4.6 差异)。
/// ② ② 集 vs golden chunk 集——**不逐拍严格相等,按语义三层断言**(详见
/// `tier_split_collider_invariants`;根因:装载判定在拍 N、② 构建判定在拍
/// N+1,观察者 1 m/拍移动时 32 m 判定圆边缘的 chunk 装时合格、建时超距,
/// ② 稳态缺失——AC-T1「② 集 == 基线 chunk 集」的字面逐拍相等与细则 3/11
/// 的时序在默认参数下不可兼得,已在任务 01 回评上报总架构师裁决)。
#[test]
fn ac_t1_matches_golden_after_tier_split() {
    let golden = read_golden_lines();
    let frames = run_scenario();
    assert_non_degenerate(&frames);
    assert_eq!(
        frames.len(),
        golden.len(),
        "拍数与 golden 不一致(场景脚本或节拍变动?golden 冻结,不得顺手重录)"
    );
    for (f, g) in frames.iter().zip(&golden) {
        let (gtick, gline) = Frame::parse_line(g);
        assert_eq!(f.tick, gtick, "拍号错位");
        assert_eq!(
            f.to_line(),
            *gline,
            "拍 {} ①/冻结/唤醒/分片 基线分叉(第一个分叉行):
  实际:{}
  golden:{gline}",
            f.tick,
            f.to_line()
        );
    }
}

/// ② 侧不变量(与四项比对拆分,保持各自失败信息清晰):
/// - 首拍(布景末:静止 + 合一装载 + 无差档历史)= ② == ①;
/// - 末帧(idle 1801 拍后:观察者已移除,滞回拆除不再发生;只剩常驻体钉住的
///   chunk,其 ② 从未被拆)= ② == ①;
/// - I1 集合级:全程逐拍 ② ⊆ ①(同分片、同 chunk);
/// - 瞬态差额(|①| − |②|)只打印不断言:移动段存在边界掠过(装载判定拍 N、
///   ② 构建判定拍 N+1,1 m/拍移动使 32 m 圆边缘 chunk 装时合格建时超距)与
///   跳跃风暴追赶(② 预算 2/拍/片),峰值是场景结构的函数,不是不变量。
#[test]
fn tier_split_collider_invariants() {
    let frames = run_scenario();
    assert_non_degenerate(&frames);
    let total = |m: &LoadedPerShard| m.iter().map(|(_, c)| c.len()).sum::<usize>();
    // 首拍:布景以合一装载(② 同建)+ 40 拍静止 → ② == ①。
    let f0 = &frames[0];
    assert_eq!(
        f0.colliders, f0.loaded,
        "布景末(静止 + 合一装载)② 应与 ① 全等"
    );
    let mut max_gap = 0usize;
    for f in &frames {
        // I1 集合级:每个分片的 ② ⊆ ①。
        for (s, cols) in &f.colliders {
            let loaded = &f
                .loaded
                .iter()
                .find(|(ls, _)| ls == s)
                .expect("分片同时在两观测集")
                .1;
            assert!(
                cols.iter().all(|c| loaded.contains(c)),
                "拍 {} 分片 {s:?} I1 违反:② 含 ① 之外的 chunk",
                f.tick
            );
        }
        max_gap = max_gap.max(total(&f.loaded) - total(&f.colliders));
    }
    // 末帧:观察者已移除(无滞回拆除),只剩常驻体钉住的 chunk,② == ①。
    let tail = frames.last().expect("有末帧");
    assert_eq!(
        tail.colliders, tail.loaded,
        "末帧 ② 应与 ① 全等(钉住 chunk 的 ② 从未拆)"
    );
    println!("② 瞬态差额峰值(边界掠过 + 跳跃风暴追赶):{max_gap} @2061 拍");
}
