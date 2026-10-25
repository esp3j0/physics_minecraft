//! `cargo xtask` 入口(REQ-008)。

fn main() {
    std::process::exit(xtask::run(std::env::args().skip(1).collect()));
}
