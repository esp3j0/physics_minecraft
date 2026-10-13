//! 入口:把世界和物理拼在一起的最小演示。

use physics::{step, Vec3};
use world::{AIR, BlockId, Chunk, CHUNK_X, CHUNK_Y, CHUNK_Z};

fn main() {
    println!("physics_minecraft v{}", env!("CARGO_PKG_VERSION"));

    // 世界:一个全是石头的区块,把中心顶部挖成空气(占位演示)。
    let mut chunk = Chunk::filled(BlockId(1));
    chunk.set(CHUNK_X / 2, CHUNK_Y - 1, CHUNK_Z / 2, AIR);
    println!(
        "chunk {CHUNK_X}x{CHUNK_Y}x{CHUNK_Z} 已创建,中心顶部方块已设为空气"
    );

    // 物理:一个从 y=64 开始下落的实体,半隐式欧拉,每 0.5s 打印一次。
    let (mut pos, mut vel) = (Vec3::new(0.0, 64.0, 0.0), Vec3::ZERO);
    println!("t(s)    y        vy");
    for i in 1..=10 {
        (pos, vel) = step(pos, vel, 0.5);
        println!("{:>4.1}  {:>7.2}  {:>7.2}", i as f64 * 0.5, pos.y, vel.y);
    }
}
