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
            let c2s = C2S::Command(*cmd);
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
}
