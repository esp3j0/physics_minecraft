//! viewer —— 可视化 demo(REQ-004A):连服务器、飞行相机、体素地形渲染、动态实体、HUD。
//!
//! 装配:render 模块的窗口入口体经 `#[macroquad::main]` 展开为本 bin 的
//! main → ViewerClient::spawn(独立线程连接循环)→ 事件通道桥 → 渲染主循环。
//! bin 逻辑全在 render 模块(窗口行为不测试,README 决策 2);client/mesh
//! 皆为库级纯函数层。宏挂在 crate 根:测试目标(--test/--bin test)不含
//! window,窗口入口放在模块内会被 dead_code 误伤。
//!
//! 宏参数须是裸标识符(macroquad_macro 只取 Ident,展开 = `Window::from_config(
//! window_conf(), amain())`),故 `window_conf` 经 use 提名到根作用域。

use crate::render::window_conf;

pub mod client;
pub mod mesh;
pub mod render;

/// macroquad 窗口入口(读 VIEWER_ADDR / VIEWER_VIEW 环境变量,见 render::amain)。
#[macroquad::main(window_conf)]
async fn main() {
    render::amain().await;
}
