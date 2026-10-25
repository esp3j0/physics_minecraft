//! xtask —— 仓库内开发工具(REQ-008 D3)。子命令:`smoke`(图形冒烟编排)。
//!
//! **总架构师接口桩(REQ-008 单元 03)**:`smoke` 模块的公开函数签名固定,函数体由实施方实现;
//! `run` 的命令行解析与编排流程由实施方实现。

pub mod smoke;

/// 命令行入口(不含程序名),返回进程退出码。
pub fn run(args: Vec<String>) -> i32 {
    let _ = args;
    unimplemented!("REQ-008 单元 03:由实施方实现")
}
