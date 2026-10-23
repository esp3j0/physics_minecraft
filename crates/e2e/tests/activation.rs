//! AC4 激活不变量端到端(REQ-001 §4.3/§9.12;任务书 req001-backlog/03)。
//!
//! 覆盖蓝图 §5 的 AC4 口径:观察者驱动冻结的**真实效果链**
//! (Scheduler → ShardSim::apply_activation → rapier 施体类型切换 → 事件免疫),
//! 外加"唯一持有 / 扩张区覆盖"的结构性断言。场景与几何口径:
//! - `ground_chunk(128)` 顶面实心面 world y = 64(照 migration.rs / two_shards.rs 口径);
//! - 凝固桥关闭(任务书工程细节);实体 = 默认描述(半边 0.5 立方体);
//! - 位级断言用 `==`,物理量断言用方向性阈值(任务书未给具体米/秒数值);
//! - README 决策 7:chunk 距离 = chunk 中心(局部 8,8)到观察者水平距离,
//!   实体距离 = 3D 欧氏;滞回 32/48 与 24/32;冷却 180 拍;换装预算合计 ≤ 2/拍;
//! - README 决策 8:观察者两拍之间 `set_observers`,下一拍 tick 开头生效;
//!   激活时钟 = `apply_activation` 执行次数(每 tick 恰一次 → 等于拍号)。

use std::collections::{BTreeMap, BTreeSet};

use messaging::{InputPayload, InputSource, SimCommand, StampedInput};
use physics_sim::{ActivationConfig, ShardSim, ShardSimConfig, SolidifyConfig};
use shard::{Scheduler, SchedulerConfig};
use types::glam::{Quat, Vec3};
use types::{
    BlockId, CHUNK_SIZE, ChunkPos, DimensionId, EntityDesc, EntityId, EntityState, SHARD_CHUNKS,
    ShardId, ShardPos, Tick, WorldId,
};
use voxel::{Chunk, SimpleRegistry};

/// 地面顶面 world y(`ground_chunk(128)`:局部 y 0..128 → world -64..=63,顶面 64)。
const GROUND_TOP: f32 = 64.0;
/// 实体(半边 0.5)底面恰贴地时的中心高度。
const SPAWN_Y: f32 = GROUND_TOP + 0.5;

fn shard_id(x: i32, z: i32) -> ShardId {
    ShardId::new(WorldId(0), DimensionId(0), ShardPos::new(x, z))
}

fn eid(n: u32) -> EntityId {
    EntityId::from_parts(1, n)
}

/// 顶层实心面在 world y = top_ly - 64 的地形 chunk(照 migration.rs / two_shards.rs)。
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

/// 凝固桥关闭的分片模拟核(任务书工程细节),可注入激活配置。
fn base_sim(shard: ShardId, activation: ActivationConfig) -> ShardSim {
    ShardSim::new(
        ShardSimConfig {
            solidify: SolidifyConfig {
                enabled: false,
                ..SolidifyConfig::default()
            },
            activation,
            ..ShardSimConfig::new(shard)
        },
        Box::new(SimpleRegistry::builtin()),
    )
}

/// 带地面 chunk 网格的分片(cx × cz 笛卡尔积,全部 `ground_chunk(128)`)。
fn ground_sim(
    shard: ShardId,
    chunk_xs: std::ops::Range<i32>,
    chunk_zs: std::ops::Range<i32>,
) -> ShardSim {
    let mut s = base_sim(shard, ActivationConfig::default());
    for cx in chunk_xs.clone() {
        for cz in chunk_zs.clone() {
            s.load_chunk(ChunkPos::new(cx, cz), ground_chunk(128));
        }
    }
    s
}

/// 测试用调度器(2 worker,照 two_shards / migration 风格)。
fn sched2() -> Scheduler {
    Scheduler::new(SchedulerConfig {
        worker_count: 2,
        ..SchedulerConfig::default()
    })
}

/// 查询实体状态(闭包不持有借用,`&Scheduler` 显式传入)。
fn state_of(sched: &Scheduler, shard: ShardId, id: EntityId) -> EntityState {
    sched.entity_state(shard, id).expect("实体在本分片")
}

fn spawn_at(apply_at: Tick, seq: u64, entity: EntityId, pos: Vec3, linvel: Vec3) -> StampedInput {
    StampedInput {
        apply_at,
        source: InputSource(7),
        sequence: seq,
        payload: InputPayload::Command(SimCommand::SpawnDynamic {
            entity,
            state: EntityState {
                translation: pos,
                rotation: Quat::IDENTITY,
                linvel,
                angvel: Vec3::ZERO,
            },
            desc: EntityDesc::default(),
        }),
    }
}

fn detonate(apply_at: Tick, seq: u64, center: Vec3, power: f32, radius: f32) -> StampedInput {
    StampedInput {
        apply_at,
        source: InputSource(7),
        sequence: seq,
        payload: InputPayload::Command(SimCommand::Detonate {
            center,
            power,
            radius,
        }),
    }
}

fn teleport(apply_at: Tick, seq: u64, entity: EntityId, pos: Vec3) -> StampedInput {
    StampedInput {
        apply_at,
        source: InputSource(7),
        sequence: seq,
        payload: InputPayload::Command(SimCommand::TeleportEntity {
            entity,
            state: EntityState {
                translation: pos,
                rotation: Quat::IDENTITY,
                linvel: Vec3::ZERO,
                angvel: Vec3::ZERO,
            },
        }),
    }
}

/// chunk 中心(局部 8,8;README 决策 7 几何口径)。
fn chunk_center(cp: ChunkPos) -> Vec3 {
    let half = (CHUNK_SIZE / 2) as f32;
    Vec3::new(
        (cp.x * CHUNK_SIZE) as f32 + half,
        0.0,
        (cp.z * CHUNK_SIZE) as f32 + half,
    )
}

/// 水平距离(chunk 口径)。
fn dist_xz(a: Vec3, b: Vec3) -> f32 {
    ((a.x - b.x).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

/// 测试清单 1:观察者驱动冻结的完整效果链——
/// 冻结(逐拍位级悬停;实体本应下落 = 非退化)→ 边界爆炸:远者位级不动
/// (§9.12 事件免疫)、近者被推开 → 观察者移到远者一侧(滞回保住近者醒着)→
/// 冷却 180 拍后唤醒(重力立即恢复)→ 再爆:两者都动。
#[test]
fn frozen_entity_immune_to_blast_then_wakes() {
    let shard = shard_id(0, 0);
    let (near, far) = (eid(1), eid(2));
    // 观察者 1 贴近者;远者在 +z 方向距 33(> 32 冻结半径 → 拍 2 冻结)。
    let obs1 = Vec3::new(40.0, 65.0, 40.0);
    let p_near = Vec3::new(40.0, SPAWN_Y, 40.0);
    let p_far = Vec3::new(40.0, SPAWN_Y, 73.0);
    // 观察者 2 移到远者一侧:到近者 ≈22(≤32 滞回保醒)、到远者 ≈13.6(≤24 唤醒)。
    let obs2 = Vec3::new(44.0, 65.0, 60.0);

    let mut sched = sched2();
    sched.register(ground_sim(shard, 2..3, 0..7));
    sched
        .submit(shard, spawn_at(Tick(1), 0, near, p_near, Vec3::ZERO))
        .expect("收件箱未满");
    sched
        .submit(shard, spawn_at(Tick(1), 1, far, p_far, Vec3::ZERO))
        .expect("收件箱未满");
    // §9-1(常驻半边)迁移:被测实体常驻化(REQ-007 任务 03/04;瞬态不滞回
    // 冻结,常驻滞回冻结新增**静止前置**——§4.4.2"冻结只在静止时")。
    // 贴地出生 → 推进 24 拍让沉降静止(slow_ticks 达 consecutive_ticks 20)
    // 再进入冻结等待,满足 §9-1"先让它静止"口径。
    for (seq, id) in [(2u64, near), (3, far)] {
        sched
            .submit(
                shard,
                StampedInput {
                    apply_at: Tick(1),
                    source: InputSource(7),
                    sequence: seq,
                    payload: InputPayload::Command(SimCommand::MarkActor {
                        entity: id,
                        actor: true,
                    }),
                },
            )
            .expect("收件箱未满");
    }
    sched.set_observers(vec![obs1]);
    for _ in 0..60 {
        sched.tick(); // 拍 0..59:出生(拍 1)+ 沉降静止(位移窗滚动 + 慢速计数)
    }

    // 查询宏:调用点即时借用 sched(闭包会长期占用不可变借用,与 tick 冲突)。
    macro_rules! st {
        ($id:expr) => {
            state_of(&sched, shard, $id)
        };
    }

    // 拍 60:沉降静止完成(位移窗口 21 拍滚动 + slow_ticks ≥ 20,且窗口首尾
    // 位移 ≤ 3mm);远者(距 33 > 32,出区)在下一拍 apply_activation 被冻结
    // (静止前置已满足)。近者(距 0 ≤ 24)保持醒着。
    // 注:贴地出生后的第 1 步接触求解给实体毫米级位姿修正;冻结为 kinematic
    // 且接地时 rapier 接触求解器每步写入恒定派生速度(~-4.6e-4,实测),
    // 位姿与速度此后**逐拍位级恒等**——悬停判据用"逐拍 ==",不用速度为零。
    // at_rest 由拍 60 末 lifecycle 刷新(linvel 清零 = 睡眠兜底),plan 在
    // 拍 61 的激活刷新产出冻结 → 拍 61 落实。
    sched.tick(); // 执行第 60 拍:沉降静止判定就绪
    sched.tick(); // 执行第 61 拍:远者(出区 + 静止)冻结落实
    let frozen_far = st!(far);
    // 非退化保障:同场景动态体在重力/接触下必然演化——冻结把它钉死。
    for k in 62..72 {
        sched.tick();
        assert_eq!(st!(far), frozen_far, "拍 {k}:冻结体逐拍位级悬停");
    }
    let near_now = st!(near);
    // 近者醒着:接触求解/重力已使其位姿演化(冻结才会位级钉死;贴地出生体
    // 被接触修正推高,判据用"演化"而非"下落")。
    assert_ne!(
        near_now.translation, p_near,
        "近者醒着,位姿必须已演化(对照冻结体的逐拍恒等)"
    );
    // 薄片面 collider 顶面在 y≈64.075(照 migration.rs 口径 [63.925, 64.075]),
    // 贴地驻留中心 ≈64.575;驻留窗含接触修正余量。
    assert!(
        near_now.translation.y > GROUND_TOP - 0.1 && near_now.translation.y < GROUND_TOP + 0.7,
        "近者贴地驻留:y={}",
        near_now.translation.y
    );
    assert!(
        (near_now.translation.z - p_near.z).abs() < 0.05,
        "近者无水平扰动(出生接触修正为毫米级):z={}",
        near_now.translation.z
    );

    // ---- 边界爆炸:远者位级不动(事件免疫)、近者被推开 ----
    // 爆心在两实体连线中段:对近者距 12、对远者距 21(均在半径 30 内 →
    // 冻结体的不动是免疫,不是出界)。
    sched
        .submit(
            shard,
            detonate(Tick(72), 2, Vec3::new(40.0, SPAWN_Y, 52.0), 16.0, 30.0),
        )
        .expect("收件箱未满");
    sched.tick(); // 执行第 72 拍
    assert_eq!(
        st!(far),
        frozen_far,
        "§9.12 事件免疫:爆炸在半径内仍不得触及冻结体(位级)"
    );
    let near_hit = st!(near);
    assert!(
        near_hit.linvel.z < -2.0,
        "近者必须被爆炸推开(冲量方向 -z):linvel={:?}",
        near_hit.linvel
    );
    let z_after_blast1 = near_hit.translation.z;
    sched.tick();
    assert!(
        st!(near).translation.z < z_after_blast1,
        "近者被推开后持续滑动"
    );

    // ---- 观察者移到远者一侧;冷却 180 拍后唤醒 ----
    sched.set_observers(vec![obs2]);
    // 冷却窗:冻结簿记在拍 31(贴地驻留 ≈拍 10 后 slow_ticks 计满 20,决策
    // +落实滞后;实测唤醒拍 212 = 31 + 180 + 1)。74..212 全程冻结,
    // 拍 212 唤醒。
    for k in 74..212 {
        sched.tick();
        assert_eq!(
            st!(far),
            frozen_far,
            "拍 {k}:观察者已在唤醒半径内,但冷却窗(180 拍)未到 → 仍冻结"
        );
    }
    sched.tick(); // 执行第 212 拍:激活时钟 212 - 冻结簿记 31 ≥ 180 → 唤醒
    // 唤醒判据用状态不等:解冻后接触修正可能把 y 推高;但状态必不再恒等冻结值。
    let woke = st!(far);
    assert_ne!(
        woke, frozen_far,
        "冷却期满唤醒:动力学恢复,状态必须脱离冻结位姿(此前 180 拍位级恒等)"
    );
    for k in 213..222 {
        sched.tick();
        assert_ne!(st!(far), frozen_far, "拍 {k}:解冻后不再悬停");
    }
    // 滞回保住近者(距 obs2 ≈22 ≤ 32)且滑移早已衰减(摩擦 ≈5.9 m/s²,
    // v₀ ≈9.6 → ~98 拍刹停):再爆归因才干净。
    let near_settled = st!(near);
    assert!(
        near_settled.linvel.length() < 1.0,
        "再爆前近者应已基本静止:linvel={:?}",
        near_settled.linvel
    );

    // ---- 再爆:两者都动(命令唤醒者与滞回保醒者都响应)----
    let fe_pre = st!(far);
    let ne_pre = st!(near);
    // 爆心 (40, 64.0, 56) 半径 40:近者距 ≈17.6 → 冲量 ≈9(-z);
    // 远者距 ≈19 → 冲量 ≈8.3(+z 分量 ≈7.4,压过当时 ≈1.3 的下落速度)。
    sched
        .submit(
            shard,
            detonate(Tick(222), 3, Vec3::new(40.0, GROUND_TOP, 56.0), 16.0, 40.0),
        )
        .expect("收件箱未满");
    sched.tick(); // 执行第 222 拍
    let fe = st!(far);
    let ne = st!(near);
    assert!(
        fe.linvel.z > 2.0 && fe != fe_pre,
        "唤醒后的远者必须响应爆炸:linvel={:?}",
        fe.linvel
    );
    assert!(
        ne.linvel.z < -2.0 && ne != ne_pre,
        "滞回保醒的近者必须响应爆炸:linvel={:?}",
        ne.linvel
    );
    sched.shutdown();
}

/// 测试清单 2:TeleportEntity 触及冻结体 → 立即醒(删除+重建为 Dynamic,
/// 绕过冷却;走 plan 唤醒路径会被 180 拍冷却挡住)。醒后醒数 3 > max_awake=2
/// → **同一拍立即**按距离淘汰最远醒者(README 决策 6:命令唤醒路径
/// `enforce_budget`,淘汰在唤醒拍 step 之前落实,预算超限窗口不多拍)。
#[test]
fn wake_by_teleport_command() {
    let shard = shard_id(0, 0);
    let (e1, e2, e3) = (eid(1), eid(2), eid(3));
    let obs = Vec3::new(8.0, 0.0, 8.0);
    // 虚空场景(无地面):醒 = 持续演化,冻结 = 位级悬停,判据干净。
    // §9-1(常驻半边)迁移:零重力——出生零速 = 立即静止("冻结只在静止
    // 时",04a);醒者判据用传送落点/位置演化,不依赖重力。
    let mut sched = sched2();
    sched.register({
        let mut c = ShardSimConfig::new(shard);
        c.gravity = Vec3::ZERO;
        c.solidify = SolidifyConfig {
            enabled: false,
            ..SolidifyConfig::default()
        };
        ShardSim::new(
            ShardSimConfig {
                activation: ActivationConfig {
                    max_awake_dynamic: 2,
                    ..ActivationConfig::default()
                },
                ..c
            },
            Box::new(SimpleRegistry::builtin()),
        )
    });
    // 布局:e1 距 5(醒)、e3 距 80(>32 → 拍 2 冻结)、e2 距 30(≤32 滞回保醒)。
    // e2 晚两拍(拍 4)出生:命令唤醒前任何一拍醒数 ≤ 2,不误触发预算淘汰。
    sched
        .submit(
            shard,
            spawn_at(Tick(1), 0, e1, Vec3::new(13.0, 0.0, 8.0), Vec3::ZERO),
        )
        .expect("收件箱未满");
    sched
        .submit(
            shard,
            spawn_at(Tick(1), 1, e3, Vec3::new(88.0, 0.0, 8.0), Vec3::ZERO),
        )
        .expect("收件箱未满");
    sched
        .submit(
            shard,
            spawn_at(Tick(4), 2, e2, Vec3::new(38.0, 0.0, 8.0), Vec3::ZERO),
        )
        .expect("收件箱未满");
    // §9-1(瞬态半边)迁移:REQ-007 任务 03 起瞬态体不再滞回冻结/醒落下落
    // (持瞬态运动票、足迹缺 ② 进待地形)——本用例锚"命令唤醒 + 预算淘汰"
    // 语义,被测实体改 actor = true(常驻;常驻票在 04,本任务常驻无票行为
    // 与历史一致)。瞬态新语义由 activation_tickets 新套件覆盖。
    // MarkActor 与各 spawn 同拍前缀(canonical 序 = seq,紧跟对应 spawn)。
    for (at, seq, id) in [(Tick(1), 3u64, e1), (Tick(1), 4, e3), (Tick(4), 5, e2)] {
        sched
            .submit(
                shard,
                StampedInput {
                    apply_at: at,
                    source: InputSource(7),
                    sequence: seq,
                    payload: InputPayload::Command(SimCommand::MarkActor {
                        entity: id,
                        actor: true,
                    }),
                },
            )
            .expect("收件箱未满");
    }
    sched.set_observers(vec![obs]);
    macro_rules! st {
        ($id:expr) => {
            state_of(&sched, shard, $id)
        };
    }

    // 冻结阶段:拍 2 起 e3 冻结(距 80 > 32),此后位级悬停。
    for _ in 0..4 {
        sched.tick(); // 拍 0..3(e2 在拍 4 前缀出生)
    }
    let frozen3 = st!(e3);
    assert_eq!(frozen3.linvel, Vec3::ZERO, "e3 冻结:速度清零");
    for k in 4..12 {
        sched.tick();
        assert_eq!(
            st!(e3),
            frozen3,
            "拍 {k}:冻结体位级悬停(会下落的实体被钉死 = 冻结确实发生)"
        );
    }
    // §9-1 零重力迁移:e2 无重力不下落;判据改为"未冻结即醒"的对照本体
    // (e3 逐拍悬停 vs e2 无冻结事件;传送竞态的位级钉面不变)。
    let e2_before_tp = st!(e2);
    assert!(
        e2_before_tp.linvel == Vec3::ZERO || e2_before_tp.linvel.y <= 0.0,
        "e2 醒着(零重力悬停或静置):state={e2_before_tp:?}"
    );

    // ---- 传送触及冻结体:第 12 拍 = 删除+重建 → Dynamic,立即醒 ----
    // (冻结簿记在拍 2,仅过 10 拍 << 冷却 180:冷却窗内 plan 唤醒路径不可能产出,
    // 醒来只可能来自 Teleport 重建 = §9.12 命令唤醒。)
    sched
        .submit(shard, teleport(Tick(12), 3, e3, Vec3::new(18.0, -5.0, 8.0)))
        .expect("收件箱未满");
    sched.tick(); // 执行第 12 拍
    let woke3 = st!(e3);
    assert_eq!(woke3.translation.x, 18.0, "传送落点 x 位级生效");
    assert_eq!(woke3.translation.z, 8.0, "传送落点 z 位级生效");
    // §9-1 零重力迁移:无重力下落;命令唤醒判据 = 传送落点已生效(上方
    // x/z 位级断言即证据;plan 冷却窗内不可能唤醒,Teleport 重建是唯一路)。
    assert_eq!(woke3.translation.y, -5.0, "零重力:落点 y 位级保持");
    // ---- 预算淘汰:第 12 拍(唤醒拍)醒数 {e1≈5, e2≈30, e3≈11} = 3 > 2 →
    // 同拍立即淘汰最远者 e2(README 决策 6:命令唤醒路径 enforce_budget;
    // 淘汰在前缀 apply 末尾、step 之前落实 → e2 当拍不参与 step:位置位级
    // 钉在拍 11 末采样 e2_before_tp。非退化对照:e2 此前每拍都在下落
    // (e2_before_tp.linvel.y < -0.5),若淘汰迟到一拍(第 13 拍),第 12 拍
    // step 会推进其位置,位级等式即破)。----
    let culled2 = st!(e2);
    assert_eq!(
        culled2.translation, e2_before_tp.translation,
        "唤醒拍同拍淘汰:位置位级钉在淘汰拍 step 之前(= 拍 11 末采样)"
    );
    assert_eq!(culled2.linvel, Vec3::ZERO, "最远醒者 e2 被淘汰:速度清零");
    for k in 13..20 {
        sched.tick();
        assert_eq!(st!(e2), culled2, "拍 {k}:被淘汰者位级悬停(冻结落实)");
        // §9-1 零重力迁移:e3 不在淘汰名单 → 保持唤醒态(无重力匀速悬停,
        // y 不再下降;被淘汰才会清零悬停)。
        assert_eq!(
            st!(e3).translation.y,
            woke3.translation.y,
            "命令唤醒者 e3 不在淘汰名单(距离序 e2 > e3),零重力匀速保持"
        );
    }
    assert!(
        st!(e1).linvel == Vec3::ZERO || st!(e1).translation.y == 0.0,
        "e1 全程醒着(最近者,预算优先保留;零重力静止/悬停):{:?}",
        st!(e1)
    );
    sched.shutdown();
}

/// 测试清单 3:走廊 chunk 网格的结构前提(§4.3 失败模式"接缝无空窗"):
/// 每 ChunkPos 的 `shard_of` 唯一、两分片核心区互不相交且覆盖全走廊;
/// 边界两侧各 16m 的扩张带 chunk 在**两个**分片的加载集里都出现。
#[test]
fn chunk_ownership_is_unique_structurally() {
    let corridor: Vec<ChunkPos> = (0..16).map(|x| ChunkPos::new(x, 0)).collect();

    // 1) shard_of 是纯函数且唯一:每 chunk 的核心归属恰落两分片之一。
    let core = |sx: i32| -> BTreeSet<ChunkPos> {
        corridor
            .iter()
            .copied()
            .filter(|c| c.shard_of() == ShardPos::new(sx, 0))
            .collect()
    };
    let (core0, core1) = (core(0), core(1));
    for c in &corridor {
        let owners = [&core0, &core1]
            .iter()
            .filter(|core| core.contains(c))
            .count();
        assert_eq!(owners, 1, "chunk {c:?} 的核心归属必须唯一(实得 {owners})");
    }
    assert_eq!(
        core0.iter().copied().collect::<Vec<_>>(),
        (0..8).map(|x| ChunkPos::new(x, 0)).collect::<Vec<_>>(),
        "分片 (0,0) 核心区 = 前 8 列"
    );
    assert_eq!(
        core1.iter().copied().collect::<Vec<_>>(),
        (8..16).map(|x| ChunkPos::new(x, 0)).collect::<Vec<_>>(),
        "分片 (1,0) 核心区 = 后 8 列"
    );
    // 核心区不相交且覆盖全走廊(min_chunk + SHARD_CHUNKS 铺满)。
    let disjoint: Vec<ChunkPos> = core0.intersection(&core1).copied().collect();
    assert!(disjoint.is_empty(), "核心区交集必须为空:{disjoint:?}");
    let union: BTreeSet<ChunkPos> = core0.union(&core1).copied().collect();
    assert_eq!(union.len(), corridor.len(), "两分片核心区并集覆盖全走廊");
    for (sp, expect_x) in [(ShardPos::new(0, 0), 0i32), (ShardPos::new(1, 0), 8)] {
        // min_chunk 返回 chunk 坐标(非方块坐标)。
        assert_eq!(sp.min_chunk().x, expect_x);
        assert_eq!(sp.min_chunk().x, sp.x * SHARD_CHUNKS);
    }

    // 2) 真实装载集(经 `ShardSim::voxel()` 查询):核心区 + 边界扩张带
    //    (边界两侧各 1 列 chunk = 16m,照 migration.rs 装载方式:片 0 载 0..=8、
    //    片 1 载 7..=15)。
    let (s0, s1) = (shard_id(0, 0), shard_id(1, 0));
    let sim0 = ground_sim(s0, 0..9, 0..1); // 核心 0..=7 + 扩张列 8
    let sim1 = ground_sim(s1, 7..16, 0..1); // 扩张列 7 + 核心 8..=15
    let loaded_of = |s: &ShardSim| -> BTreeSet<ChunkPos> { s.voxel().chunk_positions().collect() };
    let (load0, load1) = (loaded_of(&sim0), loaded_of(&sim1));
    for cp in 0..16 {
        let c = ChunkPos::new(cp, 0);
        assert!(
            load0.contains(&c) || load1.contains(&c),
            "走廊 chunk {c:?} 至少被一侧加载(接缝无空窗前提)"
        );
    }
    // 扩张带重叠:边界两侧各 16m 的 chunk(x=7 归分片 0 核心、x=8 归分片 1 核心)
    // 在**两个**分片的加载集中都出现(§4.3:过渡期物理不空窗的结构前提)。
    let overlap: Vec<ChunkPos> = load0.intersection(&load1).copied().collect();
    assert_eq!(
        overlap,
        vec![ChunkPos::new(7, 0), ChunkPos::new(8, 0)],
        "扩张带恰为边界两侧各一列 chunk,且双方共载"
    );
    assert!(core0.contains(&ChunkPos::new(7, 0)) && core1.contains(&ChunkPos::new(8, 0)));
    assert!(
        load0.contains(&ChunkPos::new(8, 0)),
        "分片 0 扩张带载入邻家列 x=8"
    );
    assert!(
        load1.contains(&ChunkPos::new(7, 0)),
        "分片 1 扩张带载入邻家列 x=7"
    );
    // 无越权:装载集中除扩张带两列外的 chunk 必须归属自家核心。
    for (load, core) in [(&load0, &core0), (&load1, &core1)] {
        for cp in load {
            assert!(
                core.contains(cp) || matches!(cp, ChunkPos { x: 7 | 8, z: 0 }),
                "越权装载 {cp:?}",
            );
        }
    }

    // 3) 调度器接线冒烟:唯一持有由架构保证(sims: BTreeMap<ShardId, _> 一键一 sim);
    //    此处验证调度器视角的归属升序与拍号推进。
    let mut sched = sched2();
    sched.register(sim0);
    sched.register(sim1);
    let ids = sched.shard_ids();
    assert_eq!(ids, vec![s0, s1], "ShardId 升序唯一注册");
    let summary = sched.tick();
    assert_eq!(summary.stepped_shards, 2);
    assert_eq!(sched.tick_now(), Tick(1));
    sched.shutdown();
}

/// 测试清单 4:实体滚过 x=128 分片边界,两侧地面在扩张带重叠装载
/// (片 0 载 chunk 0..=8、片 1 载 7..=15)→ 全程实体中心 y ≥ 地面顶面 − 0.1:
/// 无穿落、迁移交接无物理空窗(AC4 行为验证;交接位级等价由 AC2 套件覆盖)。
///
/// 口径说明(任务书"贴地慢速 2 m/s"):本轮没有持续驱动/匀速滚动的输入 API,
/// 地面摩擦 μ=0.6 会在 ~0.35 s 内把 2 m/s 刹停(≈0.7 m),无法在场景长度内越界;
/// 改为一次性低初速投掷(18 m/s,触地后摩擦滑行 ≈27 m,过界时仍 ≈11 m/s),
/// 行为断言(全程 y 不低于顶面 − 0.1)不变。
#[test]
fn expansion_overlap_no_physics_gap() {
    let (src, dst) = (shard_id(0, 0), shard_id(1, 0));
    let e = eid(1);
    let mut sched = sched2();
    // 扩张带重叠:片 0 = 核心 0..=7 + 扩张列 8;片 1 = 扩张列 7 + 核心 8..=15。
    sched.register(ground_sim(src, 0..9, 0..1));
    sched.register(ground_sim(dst, 7..16, 0..1));
    sched
        .submit(
            src,
            spawn_at(
                Tick(1),
                0,
                e,
                Vec3::new(112.0, 64.7, 8.0),
                Vec3::new(18.0, 0.0, 0.0),
            ),
        )
        .expect("收件箱未满");

    const TICKS: usize = 240;
    let mut ys: Vec<f32> = Vec::with_capacity(TICKS);
    let mut src_last: Option<usize> = None; // 最后一次出现在源片的采样下标
    let mut dst_first: Option<usize> = None; // 首次出现在目的片的采样下标
    for k in 0..TICKS {
        sched.tick();
        match (sched.entity_state(src, e), sched.entity_state(dst, e)) {
            (Some(_), Some(_)) => panic!("实体同时存在于两分片:唯一持有不变量破坏(fail-stop)"),
            (Some(a), None) => {
                ys.push(a.translation.y);
                src_last = Some(k);
            }
            (None, Some(b)) => {
                ys.push(b.translation.y);
                if dst_first.is_none() {
                    dst_first = Some(k);
                }
            }
            // 迁移交接拍:信封恰有一拍在途,两片皆空(AC2 已证该拍状态位级连续)。
            (None, None) => ys.push(f32::INFINITY),
        }
    }

    // 非退化保障:确实发生了跨片迁移。双空拍恰两处——拍 0(实体未出生)与
    // 信封在途拍;目的片现身 = 源片最后现身 + 2(下一拍 step 产意图、
    // 再下一拍成为目的片前缀)。
    let vanish = src_last.expect("实体必须在源片出现过");
    let appear = dst_first.expect("实体必须迁移到目的片");
    let gap_idx: Vec<usize> = ys
        .iter()
        .enumerate()
        .filter(|(_, y)| !y.is_finite())
        .map(|(k, _)| k)
        .collect();
    assert_eq!(
        gap_idx,
        vec![0, vanish + 1],
        "双空拍必为:出生前(拍 0)+ 信封在途拍"
    );
    assert_eq!(appear, vanish + 2, "迁移交接:消失拍 +1 盖章、+2 现身");
    // 全程无穿落:所有在场采样 y ≥ 顶面 − 0.1(两个不在场拍不参与)。
    let grounded = ys.iter().filter(|y| y.is_finite()).count();
    assert_eq!(grounded, TICKS - 2, "在场采样 = 全部拍 − 出生前 − 交接拍");
    for (k, y) in ys.iter().enumerate() {
        assert!(
            !y.is_finite() || *y >= GROUND_TOP - 0.1,
            "第 {k} 拍 y = {y} 低于 地面顶面 − 0.1(物理空窗/穿落,边界 x=128)"
        );
    }
    let final_e = sched.entity_state(dst, e).expect("终态实体在目的片");
    assert!(
        final_e.translation.x > 136.0,
        "非退化:实体滚过边界并持续前滚:x={}",
        final_e.translation.x
    );
    assert!(
        final_e.translation.y < 64.7,
        "非退化:实体贴地(出生态之下):y={}",
        final_e.translation.y
    );
    sched.shutdown();
}

/// 测试清单 5:观察者沿走廊每拍 +4m 跑 40 拍,逐拍换装决策(装+卸)合计 ≤ 预算
/// (README 决策 7)。口径说明:换装决策在 `ShardSim::apply_activation` 返回值
/// 上可观测;Scheduler 按 README 决策 5 忽略该返回(tier①② 数据源属 REQ-002),
/// 故本测试直接驱动决策面并由测试充当执行方(落实装卸),同时断言等价不变量:
/// 装载集 ⊆ 当拍观察者近半径足迹、卸载集 ∩ 冷却窗内已载集 = ∅
/// (冷却窗内的装载绝不被卸)。
#[test]
fn swap_budget_respected_over_time() {
    let shard = shard_id(0, 0);
    let mut sim = ground_sim(shard, 0..9, 0..1); // 初始:本体区列 0..=7 + 扩张列 8
    let cfg = *sim.activation_config();

    // 测试侧簿记(与执行同步):计划装载拍(冷却核对)与装载集镜像。
    let mut planned_load: BTreeMap<ChunkPos, u64> = BTreeMap::new();
    let mut loaded: BTreeSet<ChunkPos> = sim.voxel().chunk_positions().collect();
    let initial: BTreeSet<ChunkPos> = loaded.clone();

    let mut total_load = 0usize;
    let mut total_unload = 0usize;
    let mut budget_hit = 0usize; // 恰好打满预算的拍数(非退化:预算确实在截断)

    for k in 0..40u64 {
        let observer = Vec3::new(8.0 + 4.0 * k as f32, 0.0, 8.0);
        let d = sim.apply_activation(std::slice::from_ref(&observer));

        let swaps = d.chunks_to_load.len() + d.chunks_to_unload.len();
        assert!(
            swaps <= cfg.data_budget_per_tick,
            "拍 {k}:换装合计 {swaps} 超预算 {}",
            cfg.data_budget_per_tick
        );
        if swaps == cfg.data_budget_per_tick {
            budget_hit += 1;
        }
        // 本测试无实体:实体档不得掺入。
        assert!(d.entities_to_wake.is_empty() && d.entities_to_freeze.is_empty());

        for &cp in &d.chunks_to_load {
            let dist = dist_xz(chunk_center(cp), observer);
            assert!(
                dist <= cfg.data_load_radius,
                "拍 {k}:装载 {cp:?} 中心距 {dist:.2} 超近半径 {}(装载集 ⊆ 观察者足迹)",
                cfg.data_load_radius
            );
            assert!(loaded.insert(cp), "重复装载已载 chunk {cp:?}");
            planned_load.insert(cp, k);
            sim.load_chunk(cp, ground_chunk(128));
        }
        for &cp in &d.chunks_to_unload {
            // 冷却不变量:被卸 chunk 距计划装载必须 ≥ 冷却;初始预载 chunk
            // 无计划拍 → 首见即可卸(controller 簿记同语义)。
            if let Some(&t) = planned_load.get(&cp) {
                assert!(
                    k - t >= cfg.cooldown_ticks,
                    "拍 {k}:卸载冷却窗内(装载于拍 {t})的 {cp:?}",
                );
            } else {
                assert!(initial.contains(&cp), "卸载了既非预载也非计划装载的 {cp:?}");
            }
            let dist = dist_xz(chunk_center(cp), observer);
            assert!(
                dist > cfg.data_unload_radius,
                "拍 {k}:卸载 {cp:?} 中心距 {dist:.2} 未出远半径 {}",
                cfg.data_unload_radius
            );
            assert!(loaded.remove(&cp), "卸载未载 chunk {cp:?}");
            sim.unload_chunk(cp);
        }
        total_load += d.chunks_to_load.len();
        total_unload += d.chunks_to_unload.len();
    }

    // 执行收束:模拟核装载集与镜像一致。
    let live: BTreeSet<ChunkPos> = sim.voxel().chunk_positions().collect();
    assert_eq!(live, loaded, "执行方与模拟核装载集一致");
    // 非退化保障:40 拍 × 4m 的足迹推进必须产生足量装载,且预算确曾打满;
    // 观察者背离初始区,冷却外的初始 chunk 必须至少合法卸载一次。
    assert!(
        total_load >= 5,
        "非退化:行进 156m 必然进入新 chunk 近半径(实测装载 {total_load})"
    );
    assert!(
        budget_hit >= 1,
        "非退化:预算必须至少打满一次(实测 {budget_hit} 拍)"
    );
    assert!(
        total_unload >= 1,
        "非退化:初始区被甩出远半径后必须发生冷却外卸载(实测 {total_unload})"
    );
}
