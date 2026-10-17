# 任务 00:协议类型 + 观察输出 API

## 目标

REQ-003 §4.1 的协议形状(sim-api)+ 驱动观察输出所需的模拟侧 API
(实体全表/凝固方块外露)——后续网络任务的稳定地基。

## 必读

- `docs/requirements/REQ-003-network.md` §4.1/§4.4
- `docs/tasks/req003-net/README.md` 决策 1/3
- `crates/messaging/src/command.rs`(SimCommand——命令白名单的来源)
- `crates/physics-sim/src/sim.rs`(Outbox.solidified 已有;entity_state 单查已有)
- `crates/shard/src/scheduler.rs`(tick 流程:outbox 处理位、快照/透传风格)

## 授权改动

- `crates/physics-sim/src/sim.rs`:`entity_states()` + solidified 外露配合;
- `crates/shard/src/scheduler.rs`:tick 收集 solidified + 透传方法;
- `crates/sim-api/src/session.rs`:透传(照编排透传同款)。

## 交付物

1. `crates/sim-api/src/proto.rs` + lib.rs 导出:C2S/S2C + 帧常量 + 内嵌测试;
2. 上述授权改动 + 各自内嵌测试。

## 接口设计(签名固定)

```rust
// sim-api/src/proto.rs
/// 客户端 → 服务器(追加式枚举;帧 = 4B LE 长度 + bincode,上限见 FRAME_MAX)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum C2S {
    Hello { client_id: u64 },
    Subscribe { view_distance: u8 },
    MoveObserver { position: Vec3 },
    /// 命令白名单(SetBlock/SpawnDynamic/RemoveEntity/TeleportEntity/Detonate);
    /// 其余 SimCommand 到达 = 协议违规 → 断连(翻译层执行,README 决策 1)。
    Command(SimCommand),
    Ping,
    Goodbye,
}

/// 服务器 → 客户端(追加式)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum S2C {
    Welcome { player_id: InputSource, tick: Tick },
    ChunkData { pos: ChunkPos, chunk: voxel::Chunk },
    ChunkUnload { pos: ChunkPos },
    /// 增量:变化实体 + 消失实体(迁移/凝固/移除);全量时 removed 为空。
    EntityStates { tick: Tick, states: Vec<(EntityId, EntityState)>, removed: Vec<EntityId> },
    BlockUpdates { updates: Vec<(BlockPos, BlockId)> },
    Pong,
    Bye { reason: String },
}

/// 单帧字节上限(REQ-003 §4.1;超限断连)。
pub const FRAME_MAX: usize = 2 * 1024 * 1024;

/// 帧编解码(4B LE 长度前缀;encode 纯函数,decode 从预读缓冲消费)。
pub fn encode_frame(s2c: &S2C) -> Vec<u8>;          // → bytes
pub fn decode_frame(bytes: &[u8]) -> Option<S2C>;   // 完整帧 → Some
// (流式切帧属任务 01 的连接层;此处只定单帧往返。)
```

```rust
// physics-sim / shard / sim-api 观察输出 API
impl ShardSim {
    /// 全实体表(观察输出用;BTreeMap 序 = EntityId 升序)。
    pub fn entity_states(&self) -> Vec<(EntityId, EntityState)>;
}
impl Scheduler {
    /// 本拍凝固写入的方块(世界坐标),两拍之间取,恰一次(tick 内收集)。
    pub fn take_solidified(&mut self) -> Vec<(ShardId, BlockPos, BlockId)>;
    pub fn entity_states(&self, shard: ShardId) -> Option<Vec<(EntityId, EntityState)>>;
}
impl Session { /* 两者透传,照编排透传同款 */ }
```

## 测试清单

| 测试名 | 断言意图 |
| --- | --- |
| `frame_roundtrip_all_variants` | 每个变体 encode→decode 相等 |
| `frame_max_enforced` | 构造超限载荷(大 Chunk)→ encode 后长度 > FRAME_MAX 的判定路径存在(decode 拒绝);≤ 上限正常 |
| `c2s_command_whitelist_types` | 白名单五命令可构造;类型层面无需运行时过滤(过滤在翻译层,注释指向) |
| `entity_states_lists_all_in_id_order` | 3 实体分片 → 全表升序 |
| `take_solidified_exactly_once` | 凝固桥触发 → take 得 (shard, pos, block);再 take 空 |
| `solidified_survives_via_session` | Session 透传可达(最小冒烟) |

## 完成定义

见 [README 公共完成定义](README.md)。
