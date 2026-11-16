//! URDF 几何 → ShapeDef(REQ-012 C01a 任务书 1.5)。

use crate::frames::{Mat3, clean_vec, mat_vec, normalize};

/// 一个形状的换算结果:ShapeDef 本体 + 报告记录。
pub(crate) struct ConvertedShape {
    pub shape: types::ShapeDef,
    pub note: crate::ShapeNote,
}

/// URDF 几何 → ShapeDef(任务书 1.5 表格)。
///
/// - `link`:报告用连杆名;
/// - `geometry`:urdf-rs 解析结果;
/// - `offset`:形状偏移(已换算 = `W_L · xyz_collision`);
/// - `r_s`:`R_s = W_L · R(rpy_collision)`(形状坐标系 → BodyDef 连杆系);
/// - `segments`:圆柱转凸包每圈点数(调用方已校验 8..=64)。
///
/// `Err` 时的变体:`UnsupportedShape` / `MeshNotYetSupported`。
pub(crate) fn convert(
    link: &str,
    geometry: &urdf_rs::Geometry,
    offset: [f32; 3],
    r_s: &Mat3,
    segments: u32,
) -> Result<ConvertedShape, crate::ImportError> {
    match geometry {
        urdf_rs::Geometry::Sphere { radius } => {
            if !radius.is_finite() || *radius <= 0.0 {
                return Err(bad_shape(link, format!("球半径 {radius} 非法")));
            }
            let radius = *radius as f32;
            Ok(ConvertedShape {
                shape: types::ShapeDef::Sphere { radius, offset },
                note: crate::ShapeNote {
                    link: link.to_owned(),
                    from: "sphere".into(),
                    to: "sphere".into(),
                    volume_src: sphere_volume(f64::from(radius)) as f32,
                    volume_out: sphere_volume(f64::from(radius)) as f32,
                    hull_points: 0,
                },
            })
        }
        urdf_rs::Geometry::Box { size } => {
            let s = size.0;
            if s.iter().any(|v| !v.is_finite() || *v <= 0.0) {
                return Err(bad_shape(link, format!("盒尺寸 {s:?} 非法")));
            }
            // R_s 是带符号的轴置换(每元素离 0 或 ±1 不超过 1e-6)→ 轴对齐盒,
            // 半边长 = |R_s| · size/2(即按置换对调)。
            if is_axis_permutation(r_s) {
                let half = [
                    (r_s[0][0].abs() * s[0] + r_s[0][1].abs() * s[1] + r_s[0][2].abs() * s[2])
                        * 0.5,
                    (r_s[1][0].abs() * s[0] + r_s[1][1].abs() * s[1] + r_s[1][2].abs() * s[2])
                        * 0.5,
                    (r_s[2][0].abs() * s[0] + r_s[2][1].abs() * s[1] + r_s[2][2].abs() * s[2])
                        * 0.5,
                ];
                let half = clean_vec(half);
                let volume = half[0] * half[1] * half[2] * 8.0;
                Ok(ConvertedShape {
                    shape: types::ShapeDef::Box { half, offset },
                    note: crate::ShapeNote {
                        link: link.to_owned(),
                        from: "box".into(),
                        to: "box".into(),
                        volume_src: box_volume(&s),
                        volume_out: volume,
                        hull_points: 0,
                    },
                })
            } else {
                // 其它朝向 → 8 点凸包:R_s · (±h),按生成顺序写,不排序不去重。
                let half = [s[0] * 0.5, s[1] * 0.5, s[2] * 0.5];
                let mut points = Vec::with_capacity(8);
                for i in 0..8 {
                    let corner = [
                        if i & 1 != 0 { half[0] } else { -half[0] },
                        if i & 2 != 0 { half[1] } else { -half[1] },
                        if i & 4 != 0 { half[2] } else { -half[2] },
                    ];
                    points.push(clean_vec(mat_vec(r_s, corner)));
                }
                let volume = box_volume(&s);
                Ok(ConvertedShape {
                    shape: types::ShapeDef::ConvexHull { points, offset },
                    note: crate::ShapeNote {
                        link: link.to_owned(),
                        from: "box".into(),
                        to: "convex_hull".into(),
                        volume_src: volume,
                        volume_out: volume,
                        hull_points: 8,
                    },
                })
            }
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            if !radius.is_finite() || !length.is_finite() || *radius <= 0.0 || *length <= 0.0 {
                return Err(bad_shape(link, format!("圆柱 r={radius} l={length} 非法")));
            }
            let points = cylinder_hull_points(*radius, *length, r_s, segments);
            let volume_src = cylinder_volume(*radius, *length);
            // 输出体积:凸包 = 圆内接 n 棱柱体积 × 长(报告记录用)。
            let volume_out = polygon_area(segments, *radius) * length;
            Ok(ConvertedShape {
                shape: types::ShapeDef::ConvexHull { points, offset },
                note: crate::ShapeNote {
                    link: link.to_owned(),
                    from: "cylinder".into(),
                    to: "convex_hull".into(),
                    volume_src: volume_src as f32,
                    volume_out: volume_out as f32,
                    hull_points: (2 * segments) as usize,
                },
            })
        }
        urdf_rs::Geometry::Capsule { radius, length } => {
            if !radius.is_finite() || !length.is_finite() || *radius <= 0.0 || *length <= 0.0 {
                return Err(bad_shape(link, format!("胶囊 r={radius} l={length} 非法")));
            }
            // URDF 胶囊沿自身 z 轴。轴在 R_s 下落在 ±y 上 → CapsuleY;
            // 其它朝向不支持。
            let axis = mat_vec(r_s, [0.0, 0.0, 1.0]);
            let x = mat_vec(r_s, [1.0, 0.0, 0.0]);
            let axis_clean = clean_vec(axis);
            let x_clean = clean_vec(x);
            let up_y =
                axis_clean[1].abs() > 1.0 - 1e-6 && axis_clean[0] == 0.0 && axis_clean[2] == 0.0;
            if !up_y || x_clean[1].abs() > 1e-6 {
                return Err(crate::ImportError::UnsupportedShape {
                    link: link.to_owned(),
                    what: format!("胶囊轴非 y 向(轴 = {axis_clean:?}),本版本不支持"),
                });
            }
            let radius_f = *radius as f32;
            let half_height = (*length * 0.5) as f32;
            // 胶囊体积 = 圆柱段 + 两个半球(= 一个整球)。
            let volume = (cylinder_volume(*radius, *length) + sphere_volume(*radius)) as f32;
            Ok(ConvertedShape {
                shape: types::ShapeDef::CapsuleY {
                    half_height,
                    radius: radius_f,
                    offset,
                },
                note: crate::ShapeNote {
                    link: link.to_owned(),
                    from: "capsule".into(),
                    to: "capsule_y".into(),
                    volume_src: volume,
                    volume_out: volume,
                    hull_points: 0,
                },
            })
        }
        urdf_rs::Geometry::Mesh { filename, .. } => Err(crate::ImportError::MeshNotYetSupported {
            link: link.to_owned(),
            file: filename.clone(),
        }),
    }
}

/// 圆柱凸包顶点(任务书 1.5):两圈各 `segments` 个点,圆周均分;
/// 第一个点取在与轴垂直的固定参考方向上——参考方向 = R_s 下 x 轴
/// (`R_s · (1,0,0)`)在垂直平面内的投影,归一化;圆周按
/// `cos(2πk/n)` 生成(角度递增 = 绕 URDF 轴 +z 右手正向),k 从 0。
/// 顶点按生成顺序写(k = 0..n 顶圈、k = 0..n 底圈),不排序、不去重。
pub(crate) fn cylinder_hull_points(
    radius: f64,
    length: f64,
    r_s: &Mat3,
    segments: u32,
) -> Vec<[f32; 3]> {
    let n = segments as usize;
    let half = length * 0.5;
    // URDF 圆柱沿自身 z:圆心 (0,0,±half)。
    let dir = normalize(mat_vec(r_s, [1.0, 0.0, 0.0]));
    let up = normalize(mat_vec(r_s, [0.0, 0.0, 1.0]));
    // 参考方向 = dir 投影到垂直 up 的平面(URDF z 下两者天然垂直,投影保原)。
    let dot = dir[0] * up[0] + dir[1] * up[1] + dir[2] * up[2];
    let proj = [
        dir[0] - dot * up[0],
        dir[1] - dot * up[1],
        dir[2] - dot * up[2],
    ];
    let ref_dir = normalize(proj);
    // 第二参考方向 = up × ref(右手系,保证圆周方向确定)。
    let cross = [
        up[1] * ref_dir[2] - up[2] * ref_dir[1],
        up[2] * ref_dir[0] - up[0] * ref_dir[2],
        up[0] * ref_dir[1] - up[1] * ref_dir[0],
    ];
    let mut points = Vec::with_capacity(2 * n);
    for z in [half, -half] {
        for k in 0..n {
            let t = 2.0 * std::f64::consts::PI * (k as f64) / (n as f64);
            let (s_t, c_t) = t.sin_cos();
            // 圆周点(URDF 形状系):ref·r·cos + cross·r·sin + z·up。
            let px = radius * (c_t * ref_dir[0] + s_t * cross[0]) + z * up[0];
            let py = radius * (c_t * ref_dir[1] + s_t * cross[1]) + z * up[1];
            let pz = radius * (c_t * ref_dir[2] + s_t * cross[2]) + z * up[2];
            points.push(clean_vec([px, py, pz]));
        }
    }
    points
}

fn bad_shape(link: &str, what: String) -> crate::ImportError {
    crate::ImportError::UnsupportedShape {
        link: link.to_owned(),
        what,
    }
}

/// R_s 是否为带符号轴置换(每元素离 0 或 ±1 不超过 1e-6,任务书 1.5)。
pub(crate) fn is_axis_permutation(r_s: &Mat3) -> bool {
    r_s.iter().all(|row| {
        row.iter()
            .all(|v| v.abs() < 1e-6 || (v.abs() - 1.0).abs() < 1e-6)
    })
}

fn box_volume(size: &[f64; 3]) -> f32 {
    (size[0] * size[1] * size[2]) as f32
}

fn sphere_volume(radius: f64) -> f64 {
    (4.0 / 3.0) * std::f64::consts::PI * radius * radius * radius
}

fn cylinder_volume(radius: f64, length: f64) -> f64 {
    std::f64::consts::PI * radius * radius * length
}

/// 圆内接正 n 边形面积:½ n r² sin(2π/n)(体积比 I6 用)。
fn polygon_area(n: u32, radius: f64) -> f64 {
    0.5 * f64::from(n) * radius * radius * (2.0 * std::f64::consts::PI / f64::from(n)).sin()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// I6 体积比:圆柱 n=16 凸包体积 / 圆柱体积 = (½·16·sin(2π/16))/π
    /// = (16 / 2π)·sin(2π / 16) ≈ 0.974495…
    #[test]
    fn cylinder_volume_ratio_matches_formula() {
        let src = cylinder_volume(0.5, 2.0);
        let out = polygon_area(16, 0.5) * 2.0;
        let ratio = out / src;
        let expected =
            (16.0 / (2.0 * std::f64::consts::PI)) * (2.0 * std::f64::consts::PI / 16.0).sin();
        assert!((ratio - expected).abs() < 1e-5, "ratio={ratio}");
    }

    /// 轴置换判定:恒等阵是;带 0.7071 的旋转不是。
    #[test]
    fn axis_permutation_detection() {
        let identity: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        assert!(is_axis_permutation(&identity));
        let rot45: Mat3 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        assert!(is_axis_permutation(&rot45));
        let c = std::f64::consts::FRAC_1_SQRT_2;
        let yaw45: Mat3 = [[c, -c, 0.0], [c, c, 0.0], [0.0, 0.0, 1.0]];
        assert!(!is_axis_permutation(&yaw45));
    }

    /// 圆柱凸包:n=8、半径 1、长 2、R_s=I(轴在 z)。顶圈第一点 = (1,0,1)
    /// (ref = x 方向,k=0 角度 0);第二点 = 圆周 45°:(√2/2, √2/2, 1)。
    #[test]
    fn cylinder_hull_identity_axis_z() {
        let identity: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let pts = cylinder_hull_points(1.0, 2.0, &identity, 8);
        assert_eq!(pts.len(), 16, "2 圈 × 8 点");
        // 顶圈 z = 1(k = 0..8),底圈 z = −1(k = 8..16)。
        let c = std::f64::consts::FRAC_1_SQRT_2;
        let expect_first = [1.0, 0.0, 1.0];
        for i in 0..3 {
            assert!((f64::from(pts[0][i]) - expect_first[i]).abs() < 1e-6);
        }
        let expect_second = [c, c, 1.0];
        for i in 0..3 {
            assert!(
                (f64::from(pts[1][i]) - expect_second[i]).abs() < 1e-6,
                "pts[1] = {:?}",
                pts[1]
            );
        }
        // 底圈第一点 = (1,0,−1)。
        let expect_bottom = [1.0, 0.0, -1.0];
        for i in 0..3 {
            assert!((f64::from(pts[8][i]) - expect_bottom[i]).abs() < 1e-6);
        }
    }

    /// 圆柱轴沿 URDF −z(R_s = diag(1,−1,−1)):up = (0,0,−1)。
    /// 顶圈(z = +1·up 分量):第一点 = ref·r = (1,0,0)(ref 投影不变),
    /// 圆周 k=1 = (c, −c, 0)+z·up,z·up = −1·(0,0,−1) → 点 = (c,−c,1)?
    /// 手算:点 = r·(c·ref + s·cross) + z·up;cross = up × ref =
    /// (0,0,−1)×(1,0,0) = (0·0−(−1)·0, (−1)·1−0·0, 0·0−0·1) = (0,−1,0)。
    /// k=1: t=45°,圆周 = c·(1,0,0) + c·(0,−1,0) = (c,−c,0);
    /// z = +1(顶圈),点 = (c,−c,0) + 1·(0,0,−1) = (c,−c,−1)。
    #[test]
    fn cylinder_hull_flipped_axis() {
        let r_s: Mat3 = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]];
        let pts = cylinder_hull_points(1.0, 2.0, &r_s, 8);
        let c = std::f64::consts::FRAC_1_SQRT_2;
        // 顶圈第一点 = (1,0,−1)(z·up = 1·(0,0,−1))。
        for i in 0..3 {
            let e = [1.0, 0.0, -1.0][i];
            assert!((f64::from(pts[0][i]) - e).abs() < 1e-6);
        }
        // k=1 = (c,−c,−1)。
        for i in 0..3 {
            let e = [c, -c, -1.0][i];
            assert!(
                (f64::from(pts[1][i]) - e).abs() < 1e-6,
                "pts[1] = {:?}",
                pts[1]
            );
        }
    }

    /// 转了 45°(绕 z)的盒 → 非置换 → 8 点凸包;对角点手算:
    /// R_s = Rz(45°) = [[c,−c,0],[c,c,0],[0,0,1]],角 (±h,±h,±h)。
    /// corner (h,h,h) → (c·h−c·h, c·h+c·h, h) = (0, 2ch, h);
    /// corner (h,−h,h) → (c·h+c·h, c·h−c·h, h) = (2ch, 0, h)。h = 0.5。
    #[test]
    fn box_45deg_yields_hull_corners() {
        let c = std::f64::consts::FRAC_1_SQRT_2;
        let r_s: Mat3 = [[c, -c, 0.0], [c, c, 0.0], [0.0, 0.0, 1.0]];
        let geometry = urdf_rs::Geometry::Box {
            size: urdf_rs::Vec3([1.0, 1.0, 2.0]),
        };
        let out = convert("testlink", &geometry, [0.1, 0.2, 0.3], &r_s, 16).expect("45° 盒转凸包");
        assert_eq!(out.note.from, "box");
        assert_eq!(out.note.to, "convex_hull");
        assert_eq!(out.note.hull_points, 8);
        // 角点顺序 i=0..8:i&1 → +x,i&2 → +y,i&4 → +z。
        // i=0:(−h,−h,−h) → (−c·0.5+c·0.5, −c·0.5−c·0.5, −1) = (0, −0.7071, −1)
        // i=1:(h,−h,−h) → (c·0.5+c·0.5, c·0.5−c·0.5, −1) = (0.7071, 0, −1)
        // i=2:(−h,h,−h) → (−c−c·0.5? 手算:−h·c − h·c = −0.7071·2·0.5 = −0.7071…
        //   x = c·(−0.5) − c·(0.5) = −c = −0.7071;y = c·(−0.5) + c·(0.5) = 0;z = −1。
        // i=3:(h,h,−h) → x = c·0.5 − c·0.5 = 0;y = c·0.5 + c·0.5 = 0.7071;z = −1。
        // i=6:(−h,−h,h) → x = c·(−0.5) − c·(−0.5) = 0;y = c·(−0.5) + c·(−0.5) = −0.7071;z = 1。
        let c2 = c; // 0.7071…
        let ch = 0.5 * c2; // 0.35355…
        let expected: [[f64; 3]; 8] = [
            [0.0, -2.0 * ch, -1.0],
            [2.0 * ch, 0.0, -1.0],
            [-2.0 * ch, 0.0, -1.0],
            [0.0, 2.0 * ch, -1.0],
            [0.0, -2.0 * ch, 1.0],
            [2.0 * ch, 0.0, 1.0],
            [-2.0 * ch, 0.0, 1.0],
            [0.0, 2.0 * ch, 1.0],
        ];
        let points = match &out.shape {
            types::ShapeDef::ConvexHull { points, offset } => {
                assert_eq!(*offset, [0.1, 0.2, 0.3]);
                points.clone()
            }
            other => panic!("期望 ConvexHull,得到 {other:?}"),
        };
        for (p, e) in points.iter().zip(expected.iter()) {
            for i in 0..3 {
                assert!((f64::from(p[i]) - e[i]).abs() < 1e-6, "p={p:?} e={e:?}");
            }
        }
    }
}
