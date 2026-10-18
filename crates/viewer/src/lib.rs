//! physics_minecraft 可视化 viewer(REQ-004A)库入口。
//!
//! `client` 模块零 macroquad 依赖(README 决策 1),供 e2e 验收套件复用
//! 做不开窗的端到端冒烟(任务书 02);`mesh`/`render` 为渲染侧纯函数与
//! 窗口编排。bin 入口在 main.rs(双 target 各自编译,窗口行为不测试,
//! ACV4 手动清单)。

pub mod client;
pub mod mesh;
pub mod render;
