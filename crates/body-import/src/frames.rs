//! 坐标换算(REQ-012 C01a 任务书 1.3)。全程 f64 计算,写进 `BodyDef`
//! 时转 f32;绝对值 < 1e-9 的分量归零(避免 `-0` 与数值噪声进哈希)。
//!
//! 记号(任务书 1.3):
//! - `R(rpy)` = `Rz(yaw) · Ry(pitch) · Rx(roll)`;
//! - `G`(竖直轴换算):游戏 y 向上、x 向前;`UpAxis::Z` 时 URDF 的
//!   `(x, y, z)` → 游戏 `(x, z, −y)`(行列式 +1 的旋转),`UpAxis::Y` 时
//!   `G` = 单位阵;
//! - `A_L`(连杆累计旋转):根 = `I`,关节 `J`(父 P → 子 C)给出
//!   `A_C = A_P · R(rpy_J)` —— 关节角全 0 时 URDF 连杆坐标系相对 URDF
//!   根坐标系的朝向;
//! - `W_L = G · A_L`:URDF 连杆 L 坐标系 → BodyDef 连杆 L 坐标系。
//!   BodyDef 各连杆坐标系互相平行,同一 `W` 也适用于位置差。

/// 3×3 矩阵(行主序:`m[row][col]`)。
pub(crate) type Mat3 = [[f64; 3]; 3];

/// 竖直轴换算 `G`(任务书 1.3;变异自证 1:手性反 = 取 `(x, −z, y)`)。
pub(crate) fn up_axis_matrix(up: crate::UpAxis) -> Mat3 {
    match up {
        // URDF (x, y, z) → 游戏 (x, z, −y)。
        crate::UpAxis::Z => [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        crate::UpAxis::Y => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    }
}

/// URDF rpy → 旋转矩阵 `R = Rz(yaw) · Ry(pitch) · Rx(roll)`(任务书 1.3;
/// 变异自证 3:按 `Rx · Ry · Rz` 组合会被带 roll+yaw 的测试抓红)。
pub(crate) fn rpy_to_mat(rpy: [f64; 3]) -> Mat3 {
    let [roll, pitch, yaw] = rpy;
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    // Rx(roll)
    let rx: Mat3 = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]];
    // Ry(pitch)
    let ry: Mat3 = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]];
    // Rz(yaw)
    let rz: Mat3 = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]];
    mat_mul(&rz, &mat_mul(&ry, &rx))
}

/// 矩阵乘 `a · b`。
pub(crate) fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

/// 矩阵 · 向量。
pub(crate) fn mat_vec(a: &Mat3, v: [f64; 3]) -> [f64; 3] {
    let mut out = [0.0; 3];
    for (i, row) in out.iter_mut().enumerate() {
        *row = (0..3).map(|k| a[i][k] * v[k]).sum();
    }
    out
}

/// 向量加。
pub(crate) fn vec_add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// f64 → f32,绝对值 < 1e-9 归零(任务书 1.3:噪声不进哈希)。
pub(crate) fn clean(v: f64) -> f32 {
    if v.abs() < 1e-9 { 0.0 } else { v as f32 }
}

/// `[f64; 3]` → `[f32; 3]`,逐分量 [`clean`]。
pub(crate) fn clean_vec(v: [f64; 3]) -> [f32; 3] {
    [clean(v[0]), clean(v[1]), clean(v[2])]
}

/// 归一化(变体 [`crate::ImportError::BadAxis`] 由调用方先查零/非有限)。
pub(crate) fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    [v[0] / n, v[1] / n, v[2] / n]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 期望值手算字面量,不用被测函数算。
    fn assert_close(actual: [f64; 3], expected: [f64; 3], what: &str) {
        for i in 0..3 {
            assert!(
                (actual[i] - expected[i]).abs() < 1e-6,
                "{what}[{i}] = {} 期望 {}",
                actual[i],
                expected[i]
            );
        }
    }

    /// rpy 全零 → 单位阵(逐元素字面量)。
    #[test]
    fn rpy_zero_is_identity() {
        let m = rpy_to_mat([0.0; 3]);
        let e: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        assert_eq!(m, e);
    }

    /// roll = 90°(绕 x):y → z、z → −y。手算:Rx(π/2) =
    /// [[1,0,0],[0,0,−1],[0,1,0]]。
    #[test]
    fn rpy_roll_90() {
        let m = rpy_to_mat([std::f64::consts::FRAC_PI_2, 0.0, 0.0]);
        let e: Mat3 = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]];
        for i in 0..3 {
            for j in 0..3 {
                assert!((m[i][j] - e[i][j]).abs() < 1e-12, "m[{i}][{j}]={}", m[i][j]);
            }
        }
    }

    /// yaw = 90°(绕 z):x → y、y → −x。手算:Rz(π/2) =
    /// [[0,−1,0],[1,0,0],[0,0,1]]。
    #[test]
    fn rpy_yaw_90() {
        let m = rpy_to_mat([0.0, 0.0, std::f64::consts::FRAC_PI_2]);
        let e: Mat3 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        for i in 0..3 {
            for j in 0..3 {
                assert!((m[i][j] - e[i][j]).abs() < 1e-12, "m[{i}][{j}]={}", m[i][j]);
            }
        }
    }

    /// roll = 90°、yaw = 90° 同时非零:R = Rz · Ry · Rx(pitch = 0,Ry = I)。
    /// 手算:Rx(π/2)·v 与 Rz(π/2) 复合 = [[0,0,−1],[1,0,0],[0,1,0]]
    /// (x → x,(0,1,0) → (0,0,−1)·? 逐点:x=(1,0,0) → Rx → (1,0,0) → Rz → (0,1,0);
    /// y=(0,1,0) → Rx → (0,0,1) → Rz → (0,0,1);z=(0,0,1) → Rx → (0,−1,0) → Rz → (1,0,0)。
    /// 即 R 的列:(0,1,0)、(0,0,1)、(1,0,0))。
    /// 变异自证 3(Rx·Ry·Rz 组合)给出不同矩阵,本测试红。
    #[test]
    fn rpy_roll_yaw_90_order() {
        let h = std::f64::consts::FRAC_PI_2;
        let m = rpy_to_mat([h, 0.0, h]);
        let e: Mat3 = [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        for i in 0..3 {
            for j in 0..3 {
                assert!((m[i][j] - e[i][j]).abs() < 1e-12, "m[{i}][{j}]={}", m[i][j]);
            }
        }
    }

    /// pitch = 90°(绕 y):x → −z、z → x。手算:Ry(π/2) =
    /// [[0,0,1],[0,1,0],[−1,0,0]]。
    #[test]
    fn rpy_pitch_90() {
        let m = rpy_to_mat([0.0, std::f64::consts::FRAC_PI_2, 0.0]);
        let e: Mat3 = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]];
        for i in 0..3 {
            for j in 0..3 {
                assert!((m[i][j] - e[i][j]).abs() < 1e-12, "m[{i}][{j}]={}", m[i][j]);
            }
        }
    }

    /// G(z 向上):URDF (0,0,1) → 游戏 (0,1,0);(0,1,0) → (0,0,−1);
    /// (1,0,0) → (1,0,0)。手算直接查表。
    #[test]
    fn up_axis_z_maps() {
        let g = up_axis_matrix(crate::UpAxis::Z);
        assert_close(mat_vec(&g, [0.0, 0.0, 1.0]), [0.0, 1.0, 0.0], "z轴");
        assert_close(mat_vec(&g, [0.0, 1.0, 0.0]), [0.0, 0.0, -1.0], "y轴");
        assert_close(mat_vec(&g, [1.0, 0.0, 0.0]), [1.0, 0.0, 0.0], "x轴");
    }

    /// G(y 向上)= 单位阵。
    #[test]
    fn up_axis_y_identity() {
        let g = up_axis_matrix(crate::UpAxis::Y);
        assert_close(mat_vec(&g, [0.3, -1.5, 2.0]), [0.3, -1.5, 2.0], "任意向量");
    }

    /// G·A 链:绕 z 转 90° 的连杆,G(z 向上):A 把 (1,0,0) → (0,1,0),
    /// G 再把 (0,1,0) → (0,0,−1)(I3 的手算口径)。
    #[test]
    fn g_times_a_chain() {
        let h = std::f64::consts::FRAC_PI_2;
        let a = rpy_to_mat([0.0, 0.0, h]);
        let g = up_axis_matrix(crate::UpAxis::Z);
        let w = mat_mul(&g, &a);
        assert_close(mat_vec(&w, [1.0, 0.0, 0.0]), [0.0, 0.0, -1.0], "W·x");
        assert_close(mat_vec(&w, [0.0, 0.0, 1.0]), [0.0, 1.0, 0.0], "W·z");
    }

    /// clean:< 1e-9 归零(含 −0),常规值转 f32。
    #[test]
    fn clean_small_values() {
        assert_eq!(clean(-1e-12), 0.0);
        assert_eq!(clean(5e-10), 0.0);
        assert_eq!(clean(0.5), 0.5);
        assert_eq!(clean(-2.25), -2.25);
    }

    /// normalize:(3,4,0) → (0.6, 0.8, 0)。
    #[test]
    fn normalize_345() {
        let v = normalize([3.0, 4.0, 0.0]);
        assert!((v[0] - 0.6).abs() < 1e-12 && (v[1] - 0.8).abs() < 1e-12 && v[2] == 0.0);
    }
}
