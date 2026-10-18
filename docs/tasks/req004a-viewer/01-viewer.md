# 任务 01:viewer crate(client + render)

## 目标

REQ-004A §2:可视化 demo——连服务器、飞行相机、体素地形渲染、动态实体、HUD。

## 必读

- `docs/requirements/REQ-004A-visual-viewer.md`(全部)
- `crates/sim-api/src/proto.rs`(C2S/S2C 与对称 codec——**全部复用**)
- `crates/app/src/net.rs` 测试里的迷你客户端(连接/帧循环参考)
- `crates/voxel/src/chunk.rs`(Chunk 结构与坐标)

## 交付物

新 crate `crates/viewer`:
- `Cargo.toml`(bin;deps:sim-api/voxel/types/messaging/tokio/macrossquad="0.4";
  `[lints] workspace = true`);
- `src/main.rs`(装配与窗口入口)、`src/client.rs`(协议客户端层)、
  `src/render.rs`(渲染)、`src/mesh.rs`(暴露面/颜色/顶点纯函数);
- 纯函数单测;**窗口/相机无测试**(ACV4 手动清单,禁造假断言)。

## 接口设计(要点固定)

```rust
// client.rs(零 macroquad 依赖;可被 e2e 复用)
pub struct ViewerClient; // 连接/Hello{client_id:固定或随机?——固定 0(非权威输入不需要唯一)/
                         // Subscribe{view_distance: 4}/MoveObserver/帧循环
pub enum ViewEvent {                     // client → 主循环(通道桥)
    Welcome { player_id, tick },
    Chunk { pos, voxel::Chunk },
    ChunkGone { pos },
    Entities { tick, states, removed },
    Blocks { updates },
    Disconnected { reason },
}
impl ViewerClient {
    /// 在独立 tokio 线程跑连接循环,返回 (事件接收端, 控制端:发 MoveObserver/Goodbye)。
    pub fn spawn(addr: SocketAddr, view_distance: u8) -> (Receiver<ViewEvent>, ControlHandle);
}

// mesh.rs(纯函数,单测主战场)
/// 6 邻居暴露面提取 → [(局部格, 面方向)](air 或界外为暴露)。
pub fn exposed_faces(chunk: &Chunk) -> Vec<((u32,u32,u32), FaceDir)>;
/// BlockId → RGBA(表:stone 灰 (0.55,0.55,0.58)/dirt 棕 (0.45,0.33,0.22);
/// 未知 → 品红警示色)。
pub fn block_color(id: BlockId) -> [f32; 4];
/// chunk + 世界原点 → macroquad Mesh 顶点(每暴露面 4 顶点 2 三角,面朝向正确,
/// 简单方向光照:法线 dot 定亮度)。
pub fn chunk_mesh(chunk: &Chunk, origin: (f32,f32,f32)) -> Vec<(Vertex...)或等价>;
```

render.rs:macroquad 主循环(conf_macroquad 或 main):相机(WASD + 鼠标视角,
Shift/Ctrl 升降;空格切换"相机→观察者耦合",默认耦合 = MoveObserver 跟相机);
Chunk/ChunkGone 维护 Mesh 表;Entities 增删改实体节点(单位立方体,蓝色系,
位姿从 EntityState);HUD 四行文本(tick/chunks/entities/FPS)。
控制端 MoveObserver 节流(≥100ms 一次)。

## 测试清单(纯函数;窗口行为走 ACV4 手动清单)

| 测试名 | 断言意图 |
| --- | --- |
| `exposed_faces_count_superflat_surface` | 超平坦 chunk:顶面 256 + 侧面界外暴露数,埋藏块零面 |
| `exposed_faces_respects_neighbors` | 中央凸块:5 面暴露(底面贴地不暴露) |
| `block_color_table` | 三内置色 + 未知品红 |
| `chunk_mesh_vertex_count_matches_faces` | 顶点数 = 暴露面 × 4;面法线与亮度非零 |
| `client_frame_roundtrip_against_codec` | ViewEvent 与 proto codec 的往返一致(喂编码帧) |

## 完成定义

见 [README](README.md);门禁 `-p viewer`(编译 macroquad + 单测 + clippy);
ACV4 手动清单留架构师实跑,notes 里给出**运行方法一行**(两个终端的命令)。
