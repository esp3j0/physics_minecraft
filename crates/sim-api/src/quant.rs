//! REQ-014 §5 紧凑/简包姿态流量化与打包纯函数。
//!
//! 所有取整用 `f32::round`(四舍五入远离零),**先钳制后转型**;每个打包函数配
//! 对应解包函数。定宽字段统一为 `[u8; N]`(规避 bincode standard varint 膨胀)。

use types::glam::{Quat, Vec3};

const QUAT_SCALE: f32 = std::f32::consts::FRAC_1_SQRT_2;

/// 最小三元 15bit 四元数打包:6B LE(`idx:2 | a0:15 | a1:15 | a2:15`,每分量 ±1/√2、步长 √2/32767)。
pub fn pack_quat(q: Quat) -> [u8; 6] {
    let c = [q.x, q.y, q.z, q.w];
    let mut max_idx = 0usize;
    let mut max_abs = c[0].abs();
    for (i, &v) in c.iter().enumerate().skip(1) {
        let a = v.abs();
        if a > max_abs {
            max_abs = a;
            max_idx = i;
        }
    }
    let sign = if c[max_idx] < 0.0 { -1.0_f32 } else { 1.0_f32 };
    let mut packed = [0u64; 3];
    let mut k = 0usize;
    for (i, &v) in c.iter().enumerate() {
        if i != max_idx {
            packed[k] = ((v * sign / QUAT_SCALE + 1.0) * 0.5 * 32767.0)
                .round()
                .clamp(0.0, 32767.0) as u64;
            k += 1;
        }
    }
    let bits = (max_idx as u64) | (packed[0] << 2) | (packed[1] << 17) | (packed[2] << 32);
    let le = bits.to_le_bytes();
    [le[0], le[1], le[2], le[3], le[4], le[5]]
}

/// 最小三元 15bit 四元数解包:6B LE → 归一化 `Quat`(重构最大正分量)。
pub fn unpack_quat(b: &[u8; 6]) -> Quat {
    let bits = u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], 0, 0]);
    let idx = (bits & 0x3) as usize;
    let a0 = ((bits >> 2) & 0x7FFF) as f32;
    let a1 = ((bits >> 17) & 0x7FFF) as f32;
    let a2 = ((bits >> 32) & 0x7FFF) as f32;
    let vals = [
        (a0 / 32767.0 * 2.0 - 1.0) * QUAT_SCALE,
        (a1 / 32767.0 * 2.0 - 1.0) * QUAT_SCALE,
        (a2 / 32767.0 * 2.0 - 1.0) * QUAT_SCALE,
    ];
    let missing = (1.0 - vals[0] * vals[0] - vals[1] * vals[1] - vals[2] * vals[2])
        .max(0.0)
        .sqrt();
    let mut out = [0.0_f32; 4];
    let mut k = 0usize;
    for (i, slot) in out.iter_mut().enumerate() {
        if i == idx {
            *slot = missing;
        } else {
            *slot = vals[k];
            k += 1;
        }
    }
    Quat::from_xyzw(out[0], out[1], out[2], out[3]).normalize()
}

/// 根线速度打包:3×i16 LE(步长 0.001 m/s,对称钳制 ±32.767 m/s)。
pub fn pack_linvel(v: Vec3) -> [u8; 6] {
    let x = ((v.x * 1000.0).round().clamp(-32767.0, 32767.0) as i16).to_le_bytes();
    let y = ((v.y * 1000.0).round().clamp(-32767.0, 32767.0) as i16).to_le_bytes();
    let z = ((v.z * 1000.0).round().clamp(-32767.0, 32767.0) as i16).to_le_bytes();
    [x[0], x[1], y[0], y[1], z[0], z[1]]
}

/// 根线速度解包:3×i16 LE → `Vec3`(步长 0.001 m/s)。
pub fn unpack_linvel(b: &[u8; 6]) -> Vec3 {
    let x = i16::from_le_bytes([b[0], b[1]]) as f32 * 0.001;
    let y = i16::from_le_bytes([b[2], b[3]]) as f32 * 0.001;
    let z = i16::from_le_bytes([b[4], b[5]]) as f32 * 0.001;
    Vec3::new(x, y, z)
}

/// 子连杆根相对位置打包:3×u16 LE(步长 0.5mm、偏置 ±16.384m,范围 [-16.384, +16.3835]m)。
pub fn pack_child_pos(v: Vec3) -> [u8; 6] {
    let x = (((v.x + 16.384) / 0.0005).round().clamp(0.0, 65535.0) as u16).to_le_bytes();
    let y = (((v.y + 16.384) / 0.0005).round().clamp(0.0, 65535.0) as u16).to_le_bytes();
    let z = (((v.z + 16.384) / 0.0005).round().clamp(0.0, 65535.0) as u16).to_le_bytes();
    [x[0], x[1], y[0], y[1], z[0], z[1]]
}

/// 子连杆根相对位置解包:3×u16 LE → `Vec3`(步长 0.5mm、偏置 ±16.384m)。
pub fn unpack_child_pos(b: &[u8; 6]) -> Vec3 {
    let x = u16::from_le_bytes([b[0], b[1]]) as f32 * 0.0005 - 16.384;
    let y = u16::from_le_bytes([b[2], b[3]]) as f32 * 0.0005 - 16.384;
    let z = u16::from_le_bytes([b[4], b[5]]) as f32 * 0.0005 - 16.384;
    Vec3::new(x, y, z)
}

/// 子连杆位姿打包:12B(`[0..6]` 位置 3×u16 LE + `[6..12]` 最小三元 15bit 四元数 6B)。
pub fn pack_child_link(pos: Vec3, rot: Quat) -> [u8; 12] {
    let p = pack_child_pos(pos);
    let r = pack_quat(rot);
    [
        p[0], p[1], p[2], p[3], p[4], p[5], r[0], r[1], r[2], r[3], r[4], r[5],
    ]
}

/// 子连杆位姿解包:12B → `(根相对位置, 根相对旋转)`。
pub fn unpack_child_link(b: &[u8; 12]) -> (Vec3, Quat) {
    let p = [b[0], b[1], b[2], b[3], b[4], b[5]];
    let r = [b[6], b[7], b[8], b[9], b[10], b[11]];
    (unpack_child_pos(&p), unpack_quat(&r))
}

/// Lite 位置打包:3×u16 LE(步长 1/32m、偏置 ±1024m,范围 [-1024, +1023.96875]m)。
pub fn pack_lite_pos(v: Vec3) -> [u8; 6] {
    let x = (((v.x + 1024.0) * 32.0).round().clamp(0.0, 65535.0) as u16).to_le_bytes();
    let y = (((v.y + 1024.0) * 32.0).round().clamp(0.0, 65535.0) as u16).to_le_bytes();
    let z = (((v.z + 1024.0) * 32.0).round().clamp(0.0, 65535.0) as u16).to_le_bytes();
    [x[0], x[1], y[0], y[1], z[0], z[1]]
}

/// Lite 位置解包:3×u16 LE → `Vec3`(步长 1/32m、偏置 ±1024m)。
pub fn unpack_lite_pos(b: &[u8; 6]) -> Vec3 {
    let x = u16::from_le_bytes([b[0], b[1]]) as f32 / 32.0 - 1024.0;
    let y = u16::from_le_bytes([b[2], b[3]]) as f32 / 32.0 - 1024.0;
    let z = u16::from_le_bytes([b[4], b[5]]) as f32 / 32.0 - 1024.0;
    Vec3::new(x, y, z)
}

/// 航向角打包:u16 LE(全圆 [0, 2π),步长 2π/65536 rad,round 到 65536 回绕 0)。
pub fn pack_yaw(yaw: f32) -> [u8; 2] {
    let w = yaw.rem_euclid(std::f32::consts::TAU);
    let q = ((w / std::f32::consts::TAU * 65536.0)
        .round()
        .clamp(0.0, 65536.0) as u32
        & 0xFFFF) as u16;
    q.to_le_bytes()
}

/// 航向角解包:u16 LE → [0, 2π) rad。
pub fn unpack_yaw(b: &[u8; 2]) -> f32 {
    u16::from_le_bytes(*b) as f32 / 65536.0 * std::f32::consts::TAU
}

/// 运动标记打包:u8(低 7 位水平速率,步长 0.05 m/s、范围 0..=6.35;bit7 = frozen/airborne)。
pub fn pack_motion_flags(horizontal_speed: f32, frozen_airborne: bool) -> u8 {
    let low7 = (horizontal_speed / 0.05).round().clamp(0.0, 127.0) as u8;
    low7 | if frozen_airborne { 0x80 } else { 0 }
}

/// 运动标记解包:u8 → `(水平速率 m/s, frozen_airborne)`。
pub fn unpack_motion_flags(b: u8) -> (f32, bool) {
    ((b & 0x7F) as f32 * 0.05, (b & 0x80) != 0)
}

/// Lite 基准 chunk 打包:8B(`cx: i32 LE ++ cz: i32 LE`)。
pub fn pack_origin_chunk(cx: i32, cz: i32) -> [u8; 8] {
    let x = cx.to_le_bytes();
    let z = cz.to_le_bytes();
    [x[0], x[1], x[2], x[3], z[0], z[1], z[2], z[3]]
}

/// Lite 基准 chunk 解包:8B → `(cx, cz)`。
pub fn unpack_origin_chunk(b: &[u8; 8]) -> (i32, i32) {
    let cx = i32::from_le_bytes([b[0], b[1], b[2], b[3]]);
    let cz = i32::from_le_bytes([b[4], b[5], b[6], b[7]]);
    (cx, cz)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quat_angle_err(a: Quat, b: Quat) -> f64 {
        // 用 f64 归一化后求 2*acos(min(1,|dot|)),避免 f32 在 |dot|≈1 处 acos 精度塌陷(2^-10)。
        let a = [a.x as f64, a.y as f64, a.z as f64, a.w as f64];
        let b = [b.x as f64, b.y as f64, b.z as f64, b.w as f64];
        let na = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]).sqrt();
        let nb = (b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + b[3] * b[3]).sqrt();
        let dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]) / (na * nb);
        2.0 * dot.abs().min(1.0).acos()
    }

    /// REQ-014 M04a T3:四元数最小三元 15bit golden、±q 等价与 10000 样本误差界。
    #[test]
    fn m04a_t3_quat_golden_and_bound() {
        assert_eq!(
            pack_quat(Quat::IDENTITY),
            [0x03, 0x00, 0x01, 0x80, 0x00, 0x40],
            "identity 四元数 golden"
        );

        let s = std::f32::consts::FRAC_1_SQRT_2;
        let tie = Quat::from_xyzw(0.0, s, 0.0, s);
        assert_eq!(
            pack_quat(tie),
            [0x01, 0x00, 0x01, 0x80, 0xFF, 0x7F],
            "|y|==|w| 并列取最小下标 y,w 量化满 32767"
        );

        assert_eq!(
            pack_quat(Quat::IDENTITY),
            pack_quat(-Quat::IDENTITY),
            "q 与 -q 必须同编码(identity)"
        );
        assert_eq!(pack_quat(tie), pack_quat(-tie), "q 与 -q 必须同编码(tie)");

        let id_err = quat_angle_err(Quat::IDENTITY, unpack_quat(&pack_quat(Quat::IDENTITY)));
        assert!(
            id_err <= 1e-4,
            "identity 往返角误差应 ≤ 1e-4 rad,实测 {id_err}"
        );

        let mut state: u32 = 0x1234_5678;
        let mut next_f64 = || -> f64 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (state as f64) / ((1u64 << 31) as f64) - 1.0
        };

        let mut count = 0usize;
        let mut max_err: f64 = 0.0;
        for _ in 0..20_000 {
            if count == 10_000 {
                break;
            }
            let x = next_f64();
            let y = next_f64();
            let z = next_f64();
            let w = next_f64();
            let norm = (x * x + y * y + z * z + w * w).sqrt();
            if norm < 1e-6 {
                continue;
            }
            let q = Quat::from_xyzw(
                (x / norm) as f32,
                (y / norm) as f32,
                (z / norm) as f32,
                (w / norm) as f32,
            );
            let q2 = unpack_quat(&pack_quat(q));
            let err = quat_angle_err(q, q2);
            if err > max_err {
                max_err = err;
            }
            count += 1;
        }
        assert_eq!(count, 10_000, "LCG 应抽满 10000 有效样本");
        assert!(max_err > 0.0, "误差界扫描应非平凡(max > 0)");
        assert!(
            max_err <= 2.5e-4,
            "10000 样本最大角误差应 ≤ 2.5e-4 rad,实测 {max_err}"
        );
    }

    /// REQ-014 M04a T4:根线速度、子连杆根相对位置与 12B 子连杆打包 golden 及往返误差。
    #[test]
    fn m04a_t4_linvel_child_golden() {
        assert_eq!(
            pack_linvel(Vec3::new(1.2345, -0.0005, 32.9)),
            [0xD3, 0x04, 0xFF, 0xFF, 0xFF, 0x7F],
            "linvel golden(1235 / -1 / 钳 32767)"
        );
        assert_eq!(
            pack_linvel(Vec3::splat(100.0)),
            [0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F],
            "linvel 正超限三轴钳 32767"
        );
        assert_eq!(
            pack_linvel(Vec3::splat(-100.0)),
            [0x01, 0x80, 0x01, 0x80, 0x01, 0x80],
            "linvel 负超限三轴对称钳 -32767(不得出 -32768)"
        );

        assert_eq!(
            pack_child_pos(Vec3::new(0.0, 0.5, -16.384)),
            [0x00, 0x80, 0xE8, 0x83, 0x00, 0x00],
            "child_pos golden(32768 / 33768 / 0)"
        );
        assert_eq!(
            pack_child_pos(Vec3::splat(100.0)),
            [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
            "child_pos 正超限三轴钳 65535"
        );
        assert_eq!(
            pack_child_pos(Vec3::splat(-100.0)),
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            "child_pos 负超限三轴钳 0"
        );

        let link_bytes = pack_child_link(Vec3::new(0.0, 0.5, -16.384), Quat::IDENTITY);
        assert_eq!(
            link_bytes,
            [
                0x00, 0x80, 0xE8, 0x83, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00, 0x40
            ],
            "pack_child_link 12B golden"
        );
        let (up_pos, up_rot) = unpack_child_link(&link_bytes);
        assert!((up_pos - Vec3::new(0.0, 0.5, -16.384)).abs().max_element() <= 1e-6);
        assert!(quat_angle_err(up_rot, Quat::IDENTITY) <= 1e-4);

        let samples = [0.0_f32, 0.123, -0.123, 1.5, -1.5, 16.0, -16.0];
        for &s in &samples {
            let v = Vec3::new(s, -s, s);
            let lv_err = (unpack_linvel(&pack_linvel(v)) - v).abs();
            assert!(
                lv_err.x <= 0.0005 + 1e-6 && lv_err.y <= 0.0005 + 1e-6 && lv_err.z <= 0.0005 + 1e-6,
                "linvel 往返误差超界:v={v:?}, err={lv_err:?}"
            );
            let cp_err = (unpack_child_pos(&pack_child_pos(v)) - v).abs();
            assert!(
                cp_err.x <= 0.00025 + 1e-6
                    && cp_err.y <= 0.00025 + 1e-6
                    && cp_err.z <= 0.00025 + 1e-6,
                "child_pos 往返误差超界:v={v:?}, err={cp_err:?}"
            );
        }
    }

    /// REQ-014 M04a T5:Lite 位置、yaw、motion_flags golden 与往返误差。
    #[test]
    fn m04a_t5_lite_yaw_flags_golden() {
        assert_eq!(
            pack_lite_pos(Vec3::new(1.5, -2.25, 1023.96875)),
            [0x30, 0x80, 0xB8, 0x7F, 0xFF, 0xFF],
            "lite_pos golden(32816 / 32696 / 65535)"
        );
        assert_eq!(
            pack_lite_pos(Vec3::splat(2000.0)),
            [0xFF; 6],
            "lite_pos 正超限三轴钳 65535"
        );
        assert_eq!(
            pack_lite_pos(Vec3::splat(-2000.0)),
            [0x00; 6],
            "lite_pos 负超限三轴钳 0"
        );

        let pos_samples = [0.0_f32, 0.3, -0.3, 100.015625, -100.015625];
        for &s in &pos_samples {
            let v = Vec3::new(s, -s, s);
            let err = (unpack_lite_pos(&pack_lite_pos(v)) - v).abs();
            assert!(
                err.x <= 1.0 / 64.0 + 1e-6
                    && err.y <= 1.0 / 64.0 + 1e-6
                    && err.z <= 1.0 / 64.0 + 1e-6,
                "lite_pos 往返误差超界:v={v:?}, err={err:?}"
            );
        }

        assert_eq!(pack_yaw(0.0), [0x00, 0x00], "yaw 0");
        assert_eq!(
            pack_yaw(std::f32::consts::PI),
            [0x00, 0x80],
            "yaw PI = 32768"
        );
        assert_eq!(
            pack_yaw(-std::f32::consts::FRAC_PI_2),
            [0x00, 0xC0],
            "yaw -PI/2 rem_euclid = 49152"
        );
        assert_eq!(
            pack_yaw(std::f32::consts::TAU),
            [0x00, 0x00],
            "yaw TAU 回绕 0"
        );

        let yaw_bound = std::f32::consts::PI / 65536.0 + 1e-6;
        for &y in &[0.1_f32, 1.0, 3.0, 6.2] {
            let decoded = unpack_yaw(&pack_yaw(y));
            let diff = (decoded - y).rem_euclid(std::f32::consts::TAU);
            let err = diff.min(std::f32::consts::TAU - diff);
            assert!(
                err <= yaw_bound,
                "yaw 往返角误差超界:y={y}, decoded={decoded}, err={err}"
            );
        }

        assert_eq!(pack_motion_flags(1.0, true), 0x94, "flags(1.0, true)");
        assert_eq!(pack_motion_flags(10.0, false), 127, "flags 超限钳 127");
        assert_eq!(pack_motion_flags(0.0, false), 0, "flags 零值");
        assert_eq!(unpack_motion_flags(0x94), (1.0, true), "flags 解包");
    }

    /// REQ-014 M04a T6:origin_chunk 打包与极值往返。
    #[test]
    fn m04a_t6_origin_chunk() {
        assert_eq!(
            pack_origin_chunk(-3, 7),
            [0xFD, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00],
            "origin_chunk(-3, 7) golden"
        );
        assert_eq!(unpack_origin_chunk(&pack_origin_chunk(-3, 7)), (-3, 7));
        assert_eq!(
            unpack_origin_chunk(&pack_origin_chunk(i32::MIN, i32::MAX)),
            (i32::MIN, i32::MAX),
            "origin_chunk 极值往返"
        );
    }
}
