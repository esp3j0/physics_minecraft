//! viewer —— 可视化 demo(REQ-004A):连服务器、飞行相机、体素地形渲染、动态实体、HUD。
//! REQ-008:脚本模式(命令行参数见 `viewer::cli`,窗口循环在 `render`)。
//!
//! 装配:render 模块的窗口入口体经 `#[macroquad::main]` 展开为本 bin 的
//! main → ViewerClient::spawn(独立线程连接循环)→ 事件通道桥 → 渲染主循环。
//! bin 直接使用库 crate(cli/client/mesh/render/script/world 全在 lib;REQ-008
//! 单元 02 起不再在 bin 重复声明模块),窗口行为不测试(README 决策 2)。
//!
//! 宏参数须是裸标识符(macroquad_macro 只取 Ident,展开 = `Window::from_config(
//! window_conf(), amain())`),故 `window_conf` 经 use 提名到根作用域。

use viewer::render::window_conf;

/// macroquad 窗口入口(parse_args 分发交互/脚本模式,见 render::amain)。
#[macroquad::main(window_conf)]
async fn main() {
    viewer::render::amain().await;
}
