//! C2S/S2C 协议类型与单帧编解码(REQ-003 §4.1)。
//!
//! 自定义二进制协议(REQ-003 决议 1):传输 TCP,帧 = 4 字节 LE 长度前缀 +
//! bincode(standard 配置)载荷;帧上限 [`FRAME_MAX`](2 MiB,超限断连);
//! v1 不压缩(压缩留升级位)。协议类型 = 接口形状(蓝图 §3):bincode 序列化,
//! 与 `LogEntry`/`GlobalSnapshot` 同构——回放与网络同一形状(REQ-002 §4.2 伏笔)。
//!
//! 枚举只许**追加**(README 决策 1:声明序即版本序);删除/重排变体 = 破坏
//! 协议兼容,禁止。

use messaging::{InputSource, SimCommand};
use serde::{Deserialize, Serialize};
use types::glam::Vec3;
use types::{BlockId, BlockPos, ChunkPos, EntityDesc, EntityId, EntityState, Tick};
use voxel::Chunk;

/// 客户端 → 服务器(追加式枚举;帧 = 4B LE 长度 + bincode,上限见 [`FRAME_MAX`])。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum C2S {
    Hello {
        client_id: u64,
    },
    Subscribe {
        view_distance: u8,
    },
    MoveObserver {
        position: Vec3,
    },
    /// 命令白名单:SetBlock/RemoveEntity/TeleportEntity/Detonate
    /// (SpawnDynamic 已由 REQ-006 任务 02 的 [`C2S::Spawn`] 取代;SetObserverPosition
    /// 走 [`C2S::MoveObserver`] 专用包;MarkActor 为服务端策略,不对客户端开放,
    /// README 决策 1)。**类型层面无运行时过滤**:白名单判定在翻译层
    /// (app::net `translate`,任务 01)。
    Command(SimCommand),
    Ping,
    Goodbye,
    /// 外部生成请求(REQ-006 任务 02/D1=A/D2=A):客户端不提供 EntityId;
    /// 服务端分配并通过 [`S2C::Spawned`] 回执。只能出生在已加载 chunk。
    Spawn {
        state: EntityState,
        desc: EntityDesc,
    },
    /// 订阅调试遥测(REQ-011 §3.5.2;W03,枚举末尾追加):该连接开始接收
    /// [`S2C::DebugStats`];重复发送幂等;断连即失效。未订阅连接的帧序列
    /// 与不订阅时逐位相同。
    SubscribeDebug,
    /// 主体生成(REQ-012 A03 §2;追加在 `SubscribeDebug` 之后):客户端只按
    /// 名字引用身体(§2.1;哈希是服务端权威换算,不信任客户端算术)。服务端
    /// 校验(身体名在注册表、坐标/yaw 有限、出生 chunk 已加载、该连接无存活
    /// 玩家身体——D1/D12 一连接一身体)全部通过才生成;成功复用
    /// [`S2C::Spawned`] 回执;被拒不回包,只计数(驱动遥测)。
    SpawnAgent {
        body: String,
        position: Vec3,
        yaw: f32,
    },
    /// 附身(REQ-012 A03 §3):客户端请求附身一个主体(自己的身体,或一个
    /// 没人附身的服务端主体 `source = None`)。结果经 [`S2C::Possessed`] 回执;
    /// 被拒不回包,只计数。
    Possess {
        entity: EntityId,
    },
    /// 解除附身(REQ-012 A03 §3):给原主体发一条零意图(经日志投递,主体
    /// 停下),回 [`S2C::Possessed{ entity: None }`]。
    Unpossess,
    /// 主体意图(REQ-012 A03 §4;AC-A7):带实体——服务端只有在发送方正
    /// 附身这个实体时才投递,否则拒绝并计数;数值检查(有限值/钳制)仍由
    /// 内核做(A02),驱动不重复。
    AgentIntent {
        entity: EntityId,
        move_xz: (f32, f32),
        yaw: f32,
        jump: bool,
    },
}

/// 服务器 → 客户端(追加式)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum S2C {
    Welcome {
        player_id: InputSource,
        tick: Tick,
    },
    ChunkData {
        pos: ChunkPos,
        chunk: Chunk,
    },
    ChunkUnload {
        pos: ChunkPos,
    },
    /// 实体状态流:增量 = 变化实体 + 消失实体(迁移/凝固/移除,REQ-003 决议 3);
    /// 全量 = 整表(removed 恒空)。`full` 标记供客户端对账:全量包以 states
    /// 为准**重建**实体表——增量 removed 丢失(队列丢最旧)或移除恰逢全量
    /// 周期(driver 全量路径不发 removed)时,下一个全量周期自愈幽灵实体
    /// (ACV4 实跑反馈 2 修复,Refs #6)。S2C 不参与 canonical 输入排序,
    /// 字段追加不破坏既有决议。
    EntityStates {
        tick: Tick,
        states: Vec<(EntityId, EntityState)>,
        removed: Vec<EntityId>,
        full: bool,
    },
    BlockUpdates {
        updates: Vec<(BlockPos, BlockId)>,
    },
    Pong,
    Bye {
        reason: String,
    },
    /// 服务端分配的外部生成回执(REQ-006 任务 02):客户端据此获得权威 ID。
    Spawned {
        entity: EntityId,
    },
    /// 复合实体形状(REQ-010 §10 修订 4):先于该玩家第一次看到这些复合体的状态包、以及每个全量包发出;
    /// 按 `EntityId` 升序;单格实体从不出现。**枚举只追加**(REQ-003)。
    EntityShapes {
        shapes: Vec<(EntityId, types::CompoundShape)>,
    },
    /// 调试遥测(REQ-011 §3.5.2;W03,枚举末尾追加):仅发给已订阅
    /// ([`C2S::SubscribeDebug`])的连接,每 10 拍一次;未订阅连接收不到,
    /// 帧序列与改动前逐位相同。
    DebugStats {
        /// 本包对应的最新拍号。
        tick: Tick,
        /// 服务器启动以来的累计计数(13 项,§3.5.2)。
        counters: DebugCounters,
        /// 最近 8 条非空冲量流事件(带拍号,旧 → 新)。
        recent: Vec<(Tick, FlowEventWire)>,
    },
    /// 主体状态流(REQ-012 A03 §1;追加在 `DebugStats` 之后):连杆姿态按
    /// `BodyDef.links` 的顺序排(单连杆 = 1 条);增量、全量、距离过滤的
    /// 语义与周期同 [`S2C::EntityStates`],**两者分开发送**。主体永远不出
    /// 现在[`S2C::EntityStates`]里(内核 entity_states 已过滤,A01)。
    ///
    /// 与 `EntityStates` 的一处不同:全量包即使为空也照发是 `EntityStates`
    /// 的拍号心跳语义;`AgentStates` 只在这个连接视距内有主体、或上一次发
    /// 给它的主体表不为空时才发——世界里没有主体时,帧序列与改动前逐位
    /// 相同(AC-A8)。
    AgentStates {
        tick: Tick,
        /// `(EntityId, BodyHash, 连杆姿态列表)`;姿态按 `BodyDef.links` 顺序。
        states: Vec<(EntityId, types::BodyHash, Vec<EntityState>)>,
        /// 本周期消失的主体(迁移/移除;语义同 `EntityStates.removed`)。
        removed: Vec<EntityId>,
        /// 全量标记(语义同 `EntityStates.full`:以 states 重建主体表)。
        full: bool,
    },
    /// 附身回执(REQ-012 A03 §3):`Some` = 附身成功;`None` = 解除附身
    /// (客户端请求或主体被移除而被迫解除)。
    Possessed {
        entity: Option<EntityId>,
    },
}

/// 调试遥测累计计数(REQ-011 §3.5.2;W03):sim-api 内定义的协议结构,
/// 不直接暴露 physics-sim 类型;各项对应 `CohesionStats` / `DestructionStats`
/// 中的同名字段(驱动逐拍累加,重启从 0 开始,不入检查点)。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DebugCounters {
    /// §5.7 撞击冲量流:建网求值的事件数。
    pub impact_flow_events: u64,
    /// §5.7:断开事件数(`C < s`)。
    pub impact_flow_breaks: u64,
    /// §5.7:断下格数 |S| 合计。
    pub impact_flow_detached_cells: u64,
    /// §5.8 复合体冲量流:成形并进入处理循环的事件数(含预筛命中,
    /// §5.8.4 口径)。
    pub compound_flow_events: u64,
    /// §5.8:断开事件数。
    pub compound_flow_breaks: u64,
    /// §5.8:断下格数 |S| 合计。
    pub compound_flow_detached_cells: u64,
    /// §5.8:真正施加的对方修正次数(§5.8.4)。
    pub compound_flow_corrections: u64,
    /// 被切断的粘结数合计(承载 + 冲量流两路)。
    pub bonds_broken: u64,
    /// 复合体生成次数(承载塌落 / 摔断 / 撞散 / 落体成形)。
    pub compounds_spawned: u64,
    /// 复合体凝固次数。
    pub compounds_solidified: u64,
    /// 爆炸粉碎格数。
    pub cells_blasted: u64,
    /// 撞击破坏格数(逐格路径 + 冲量流路径合计)。
    pub cells_impacted: u64,
    /// 承载塌落转换格数。
    pub cells_collapsed: u64,
}

/// [`physics_sim::FlowEventInfo`](冲量流事件摘要)的协议镜像(REQ-011
/// §3.5.2;sim-api 不依赖 physics-sim,字段一比一)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FlowEventWire {
    /// 事件种类(true = Impact,false = Compound)。
    pub impact: bool,
    /// 供给合计 `s`(N·s)。
    pub supply: f32,
    /// 最小割 `C`(N·s)。
    pub cut: f32,
    /// 断开与否(`C < s`)。
    pub broke: bool,
    /// 断下格数 |S|。
    pub detached_cells: u64,
    /// 世界坐标(Impact = 被撞格格心;Compound = 刚体平移)。
    pub position: Vec3,
}

/// 单帧字节上限(REQ-003 §4.1;超限断连)。
pub const FRAME_MAX: usize = 2 * 1024 * 1024;

/// 帧编码(纯函数):4B LE 长度前缀 + bincode(standard)载荷。
/// 载荷全字段可序列化,编码不可能失败 → fail-stop;超限帧不在此拒绝
/// (判定属解码侧与连接层读循环:长度 > [`FRAME_MAX`] → 断连,任务 01 细则 2)。
pub fn encode_frame(s2c: &S2C) -> Vec<u8> {
    encode_frame_of(
        &bincode::serde::encode_to_vec(s2c, bincode::config::standard())
            .expect("S2C 编码不可能失败(全字段可序列化,fail-stop)"),
    )
}

/// 单帧解码:输入 = 完整帧(4B 前缀 + 载荷;流式切帧属任务 01 的连接层)。
/// 前缀残缺 / 长度超限(> [`FRAME_MAX`])/ 字节数与长度前缀不符 /
/// 载荷解码失败 → `None`(完整帧 → `Some`)。
pub fn decode_frame(bytes: &[u8]) -> Option<S2C> {
    let payload = decode_frame_payload(bytes)?;
    let (s2c, _consumed): (S2C, usize) =
        bincode::serde::decode_from_slice(payload, bincode::config::standard()).ok()?;
    Some(s2c)
}

/// C2S 方向的帧编码(与 [`encode_frame`] 同一帧格式;对称接口,任务 01
/// 连接层解码与任务 04 内嵌客户端编码共用,帧格式单一事实源)。
pub fn encode_c2s_frame(c2s: &C2S) -> Vec<u8> {
    encode_frame_of(
        &bincode::serde::encode_to_vec(c2s, bincode::config::standard())
            .expect("C2S 编码不可能失败(全字段可序列化,fail-stop)"),
    )
}

/// C2S 方向的帧解码(与 [`decode_frame`] 同一帧格式与拒绝规则)。
pub fn decode_c2s_frame(bytes: &[u8]) -> Option<C2S> {
    let payload = decode_frame_payload(bytes)?;
    let (c2s, _consumed): (C2S, usize) =
        bincode::serde::decode_from_slice(payload, bincode::config::standard()).ok()?;
    Some(c2s)
}

/// 方向无关的帧组装:4B LE 长度前缀 + 载荷(私有;两个方向共用)。
fn encode_frame_of(payload: &[u8]) -> Vec<u8> {
    let len = u32::try_from(payload.len()).expect("载荷长度超 u32(不可能:FRAME_MAX 远小于)");
    let mut frame = Vec::with_capacity(4 + payload.len());
    frame.extend_from_slice(&len.to_le_bytes());
    frame.extend_from_slice(payload);
    frame
}

/// 方向无关的帧头校验:前缀完整、长度在 [`FRAME_MAX`] 内且与字节数吻合
/// → 返回载荷切片;否则 `None`(私有;两个方向共用)。
fn decode_frame_payload(bytes: &[u8]) -> Option<&[u8]> {
    if bytes.len() < 4 {
        return None;
    }
    let len = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) as usize;
    if len > FRAME_MAX || bytes.len() - 4 != len {
        return None;
    }
    Some(&bytes[4..])
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::glam::Quat;
    use types::{EntityDesc, WorldId};

    /// 每个变体各一个样本(覆盖全部 S2C 变体)。
    fn s2c_all_variants() -> Vec<S2C> {
        let mut chunk = Chunk::air();
        chunk.set(1, 2, 3, BlockId(1));
        vec![
            S2C::Welcome {
                player_id: InputSource(1),
                tick: Tick(42),
            },
            S2C::ChunkData {
                pos: ChunkPos::new(-3, 7),
                chunk,
            },
            S2C::ChunkUnload {
                pos: ChunkPos::new(0, 0),
            },
            S2C::EntityStates {
                tick: Tick(7),
                states: vec![(
                    EntityId::from_parts(1, 2),
                    EntityState {
                        translation: Vec3::new(1.5, 2.5, 3.5),
                        rotation: Quat::IDENTITY,
                        linvel: Vec3::X,
                        angvel: Vec3::ZERO,
                    },
                )],
                removed: vec![EntityId::from_parts(9, 9)],
                full: false,
            },
            S2C::BlockUpdates {
                updates: vec![
                    (BlockPos::new(-1, 5, -2), BlockId(1)),
                    (BlockPos::new(4, 6, 8), BlockId::AIR),
                ],
            },
            S2C::Pong,
            S2C::Bye {
                reason: "shutdown".to_string(),
            },
            S2C::Spawned {
                entity: EntityId::from_parts(0, 7),
            },
        ]
    }

    /// 测试清单 1 `frame_roundtrip_all_variants`:每个变体 encode → decode 相等。
    #[test]
    fn frame_roundtrip_all_variants() {
        for s2c in s2c_all_variants() {
            let frame = encode_frame(&s2c);
            assert!(frame.len() >= 4, "帧至少含 4B 长度前缀");
            assert_eq!(decode_frame(&frame), Some(s2c), "变体必须单帧往返相等");
        }
    }

    /// 测试清单 2 `frame_max_enforced`:构造超限载荷(大实体全表)→ 判定路径
    /// 存在:encode 后长度 > FRAME_MAX,decode 拒绝(None);≤ 上限的正常往返。
    #[test]
    fn frame_max_enforced() {
        assert_eq!(FRAME_MAX, 2 * 1024 * 1024, "上限 = 2 MiB(REQ-003 §4.1)");
        // ~4.5 万实体状态 ≈ 2.5 MiB > FRAME_MAX(EntityState 定长 52B + id)。
        let states: Vec<(EntityId, EntityState)> = (0..45_000u32)
            .map(|i| {
                (
                    EntityId::from_parts(1, i),
                    EntityState {
                        translation: Vec3::new(i as f32, 0.0, 0.0),
                        ..EntityState::default()
                    },
                )
            })
            .collect();
        let big = S2C::EntityStates {
            tick: Tick(0),
            states: states.clone(),
            removed: Vec::new(),
            full: true,
        };
        let encoded = encode_frame(&big);
        let payload_len = encoded.len() - 4;
        assert!(
            payload_len > FRAME_MAX,
            "构造前提:载荷超限({payload_len} > {FRAME_MAX})"
        );
        assert_eq!(
            decode_frame(&encoded),
            None,
            "超限帧:decode 必须拒绝(连接层据此断连)"
        );

        // 同型小载荷 ≤ 上限:正常。
        let small = S2C::EntityStates {
            tick: Tick(0),
            states: states.into_iter().take(10).collect(),
            removed: Vec::new(),
            full: true,
        };
        let encoded = encode_frame(&small);
        assert!(encoded.len() - 4 <= FRAME_MAX, "小载荷在上限内");
        assert_eq!(decode_frame(&encoded), Some(small), "≤ 上限正常往返");
    }

    /// 测试清单 3 `c2s_command_whitelist_types`:命令包形状可构造且
    /// 可经帧编解码完整复原;类型层面无运行时过滤(非白名单同样可编码),
    /// 断连判定在翻译层(app::net `translate`,任务 01,注释指向)。
    #[test]
    fn c2s_command_whitelist_types() {
        let whitelist = vec![
            SimCommand::SetBlock {
                pos: BlockPos::new(1, 2, 3),
                block: BlockId(1),
            },
            SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 1),
                state: EntityState::default(),
                desc: EntityDesc::default(),
            },
            SimCommand::RemoveEntity {
                entity: EntityId::from_parts(1, 1),
            },
            SimCommand::TeleportEntity {
                entity: EntityId::from_parts(1, 1),
                state: EntityState::default(),
            },
            SimCommand::Detonate {
                center: Vec3::ZERO,
                power: 6.0,
                radius: 5.0,
            },
        ];
        for cmd in &whitelist {
            let c2s = C2S::Command(cmd.clone());
            let decoded = decode_c2s_frame(&encode_c2s_frame(&c2s)).expect("命令包往返");
            assert_eq!(decoded, c2s, "白名单命令包 {cmd:?} 帧往返相等");
        }
        // 类型层面不过滤:非白名单命令同样可编码(协议是宽松传输形状);
        // 白名单断连在翻译层执行(任务 01 translate,README 决策 1)。
        let not_open = C2S::Command(SimCommand::MarkActor {
            entity: EntityId::from_parts(1, 1),
            actor: true,
        });
        assert_eq!(
            decode_c2s_frame(&encode_c2s_frame(&not_open)),
            Some(not_open),
            "类型层面无运行时过滤(判定在翻译层)"
        );
        // REQ-006 任务 02:外部 Spawn 请求只带 state/desc,不带 ID。
        let spawn = C2S::Spawn {
            state: EntityState::default(),
            desc: EntityDesc::default(),
        };
        assert_eq!(
            decode_c2s_frame(&encode_c2s_frame(&spawn)),
            Some(spawn),
            "Spawn 请求帧往返"
        );
        let _ = WorldId(0); // 保序引用(纪律自证,可删)
    }

    /// AC-E7(REQ-006 任务 04):追加 Spawn/Spawned 后,旧变体帧字节不变。
    /// golden 由 REQ-006 前的 C2S/S2C 声明序与 bincode standard 固定。
    #[test]
    fn ace7_old_variant_golden_bytes() {
        assert_eq!(
            encode_c2s_frame(&C2S::Hello { client_id: 1 }),
            vec![2, 0, 0, 0, 0, 1]
        );
        assert_eq!(
            encode_c2s_frame(&C2S::Subscribe { view_distance: 4 }),
            vec![2, 0, 0, 0, 1, 4]
        );
        assert_eq!(encode_c2s_frame(&C2S::Ping), vec![1, 0, 0, 0, 4]);
        assert_eq!(encode_c2s_frame(&C2S::Goodbye), vec![1, 0, 0, 0, 5]);
        assert_eq!(encode_frame(&S2C::Pong), vec![1, 0, 0, 0, 5]);
        assert_eq!(
            encode_frame(&S2C::Welcome {
                player_id: InputSource(1),
                tick: Tick(1),
            }),
            vec![3, 0, 0, 0, 0, 1, 1]
        );
    }

    /// AC-W8(REQ-011 W03):两个新变体编解码往返;判别值钉在声明序
    /// (追加在末尾)——`SubscribeDebug` = 7(Hello0/Subscribe1/MoveObserver2/
    /// Command3/Ping4/Goodbye5/Spawn6),`DebugStats` = 9(Welcome0/ChunkData1/
    /// ChunkUnload2/EntityStates3/BlockUpdates4/Pong5/Bye6/Spawned7/
    /// EntityShapes8/DebugStats9);既有变体 golden 不变由
    /// [`Self::ace7_old_variant_golden_bytes`] 与上面两处判别值钉住。
    #[test]
    fn w8_debug_variants_roundtrip_and_discriminants() {
        // C2S::SubscribeDebug(无字段)往返 + 判别值。
        let frame = encode_c2s_frame(&C2S::SubscribeDebug);
        assert_eq!(
            frame,
            vec![1, 0, 0, 0, 7],
            "SubscribeDebug 判别值 = 7(声明序追加)"
        );
        assert_eq!(
            decode_c2s_frame(&frame),
            Some(C2S::SubscribeDebug),
            "SubscribeDebug 帧往返"
        );
        // S2C::DebugStats 往返 + 判别值(空 recent 最短帧;13 项计数 = 104 B)。
        let stats = S2C::DebugStats {
            tick: Tick(120),
            counters: DebugCounters {
                impact_flow_events: 3,
                impact_flow_breaks: 1,
                impact_flow_detached_cells: 4,
                compound_flow_events: 9,
                compound_flow_breaks: 2,
                compound_flow_detached_cells: 6,
                compound_flow_corrections: 1,
                bonds_broken: 12,
                compounds_spawned: 5,
                compounds_solidified: 2,
                cells_blasted: 60,
                cells_impacted: 7,
                cells_collapsed: 8,
            },
            recent: vec![(
                Tick(118),
                FlowEventWire {
                    impact: true,
                    supply: 9.217,
                    cut: 7.0,
                    broke: true,
                    detached_cells: 1,
                    position: Vec3::new(10.5, 5.5, 8.5),
                },
            )],
        };
        let frame = encode_frame(&stats);
        // 判别值 = 载荷首字节(varint)。
        assert_eq!(frame[4], 9, "DebugStats 判别值 = 9(声明序追加)");
        assert_eq!(decode_frame(&frame), Some(stats), "DebugStats 帧往返");
        // NaN 不可能出现(supply/cut 来自量化值除法);无穷 recent 空表往返。
        let empty = S2C::DebugStats {
            tick: Tick(0),
            counters: DebugCounters::default(),
            recent: Vec::new(),
        };
        assert_eq!(
            decode_frame(&encode_frame(&empty)),
            Some(empty),
            "空 recent 往返"
        );
    }

    /// REQ-012 A03 判据 A03-1:六个新变体编解码往返 + 判别值钉在声明序
    /// (追加在末尾)——C2S:`SpawnAgent` = 8(Hello0/Subscribe1/MoveObserver2/
    /// Command3/Ping4/Goodbye5/Spawn6/SubscribeDebug7)、`Possess` = 9、
    /// `Unpossess` = 10、`AgentIntent` = 11;S2C:`AgentStates` = 10
    /// (Welcome0/ChunkData1/ChunkUnload2/EntityStates3/BlockUpdates4/Pong5/
    /// Bye6/Spawned7/EntityShapes8/DebugStats9)、`Possessed` = 11;既有变体
    /// golden 不变由 [`Self::ace7_old_variant_golden_bytes`] 与
    /// [`Self::w8_debug_variants_roundtrip_and_discriminants`] 钉住。
    #[test]
    fn a03_agent_variants_roundtrip_and_discriminants() {
        // C2S::SpawnAgent 往返 + 判别值(载荷首字节 = varint 判别值)。
        let spawn_agent = C2S::SpawnAgent {
            body: "capsule".to_string(),
            position: Vec3::new(8.0, -60.0, 8.0),
            yaw: 1.25,
        };
        let frame = encode_c2s_frame(&spawn_agent);
        assert_eq!(frame[4], 8, "SpawnAgent 判别值 = 8(声明序追加)");
        assert_eq!(
            decode_c2s_frame(&frame),
            Some(spawn_agent),
            "SpawnAgent 往返"
        );

        // C2S::Possess。
        let possess = C2S::Possess {
            entity: EntityId::from_parts(0, 7),
        };
        let frame = encode_c2s_frame(&possess);
        assert_eq!(frame[4], 9, "Possess 判别值 = 9");
        assert_eq!(decode_c2s_frame(&frame), Some(possess), "Possess 往返");

        // C2S::Unpossess(无字段)。
        let frame = encode_c2s_frame(&C2S::Unpossess);
        assert_eq!(frame[4], 10, "Unpossess 判别值 = 10");
        assert_eq!(
            decode_c2s_frame(&frame),
            Some(C2S::Unpossess),
            "Unpossess 往返"
        );

        // C2S::AgentIntent。
        let intent = C2S::AgentIntent {
            entity: EntityId::from_parts(0, 7),
            move_xz: (1.0, -0.5),
            yaw: -2.5,
            jump: true,
        };
        let frame = encode_c2s_frame(&intent);
        assert_eq!(frame[4], 11, "AgentIntent 判别值 = 11");
        assert_eq!(decode_c2s_frame(&frame), Some(intent), "AgentIntent 往返");

        // S2C::AgentStates:增量与全量两条(带 BodyHash 与连杆姿态列表)。
        let states = S2C::AgentStates {
            tick: Tick(240),
            states: vec![(
                EntityId::from_parts(0, 7),
                types::BodyHash::fnv1a(b"probe"),
                vec![EntityState {
                    translation: Vec3::new(8.0, -60.0, 8.0),
                    ..EntityState::default()
                }],
            )],
            removed: vec![EntityId::from_parts(0, 3)],
            full: false,
        };
        let frame = encode_frame(&states);
        assert_eq!(frame[4], 10, "AgentStates 判别值 = 10(声明序追加)");
        assert_eq!(decode_frame(&frame), Some(states), "AgentStates 增量往返");
        let full = S2C::AgentStates {
            tick: Tick(300),
            states: Vec::new(),
            removed: Vec::new(),
            full: true,
        };
        assert_eq!(
            decode_frame(&encode_frame(&full)),
            Some(full),
            "AgentStates 全量(空表)往返"
        );

        // S2C::Possessed:Some 与 None 两条。
        let possessed_some = S2C::Possessed {
            entity: Some(EntityId::from_parts(0, 7)),
        };
        let frame = encode_frame(&possessed_some);
        assert_eq!(frame[4], 11, "Possessed 判别值 = 11");
        assert_eq!(
            decode_frame(&frame),
            Some(possessed_some),
            "Possessed(Some) 往返"
        );
        let possessed_none = S2C::Possessed { entity: None };
        assert_eq!(
            decode_frame(&encode_frame(&possessed_none)),
            Some(possessed_none),
            "Possessed(None) 往返"
        );
    }
}
