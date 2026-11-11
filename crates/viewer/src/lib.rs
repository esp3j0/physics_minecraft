//! physics_minecraft 可视化 viewer(REQ-004A)库入口。
//!
//! `client` 模块零 macroquad 依赖(README 决策 1),供 e2e 验收套件复用
//! 做不开窗的端到端冒烟(任务书 02);`mesh`/`render` 为渲染侧纯函数与
//! 窗口编排。bin 入口在 main.rs(双 target 各自编译,窗口行为不测试,
//! ACV4 手动清单)。

/// REQ-012 A06:哈希 → 形状表(纯逻辑,零 macroquad)。
pub mod bodies;
/// REQ-008:命令行参数(交互 / 脚本模式)。
pub mod cli;
pub mod client;
pub mod mesh;
/// REQ-012 A06:附身状态机与意图映射(纯逻辑,零 macroquad)。
pub mod possess;
pub mod render;
/// REQ-008:场景脚本(纯逻辑,零 macroquad)。
pub mod script;
/// REQ-011 W01:交互模式调试工具(纯逻辑层;拾取/预制/动作)。
pub mod tools;
/// REQ-008:世界状态(纯逻辑;渲染与脚本引擎共用)。
pub mod world;
