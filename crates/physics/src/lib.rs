//! 物理模拟模块。
//!
//! 计划承接 Minecraft 里原版没有的真实物理:重力、碰撞、结构坍塌、流体等。
//! 实现时重点参考 [rapier](https://github.com/dimforge/rapier)(本地在 `references/rapier`)。

use std::ops::{Add, Mul, Sub};

/// 三维向量(占位实现,后续可能换成 `glam` 或 `nalgebra`)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn length_squared(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;

    fn mul(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k, self.z * k)
    }
}

/// 重力加速度(方块/秒²)。原版实体等效重力约为 32 m/s²,沿用该量级方便对齐手感。
pub const GRAVITY: f64 = 32.0;

/// 半隐式欧拉积分推进一步,返回新的 (位置, 速度)。
pub fn step(pos: Vec3, vel: Vec3, dt: f64) -> (Vec3, Vec3) {
    let vel = vel + Vec3::new(0.0, -GRAVITY * dt, 0.0);
    (pos + vel * dt, vel)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_fall_one_step() {
        let (pos, vel) = step(Vec3::ZERO, Vec3::ZERO, 1.0);
        assert_eq!(vel, Vec3::new(0.0, -GRAVITY, 0.0));
        assert_eq!(pos, Vec3::new(0.0, -GRAVITY, 0.0));
    }
}
