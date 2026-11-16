//! REQ-012 C01b:网格文件查找、STL / OBJ 解析、凸包与精简(任务书 1.3–1.5)。
//! 不引入网格解析库:STL / OBJ 只取顶点和面。

use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use crate::frames::{Mat3, clean_vec, mat_vec};

/// 路径文本里是否含 `..` 段(任务书 1.3:含 `..` 不解析)。
fn has_parent_seg(s: &str) -> bool {
    s.split(['/', '\\']).any(|seg| seg == "..")
}

/// 按规则列出候选路径(有先后),返回第一个存在的;都不存在则返回
/// Err(全部候选的文本)。任务书 1.3。
pub(crate) fn resolve(
    filename: &str,
    base_dir: &Path,
    packages: &BTreeMap<String, PathBuf>,
) -> Result<PathBuf, Vec<String>> {
    let blocked = || Err(vec![format!("{filename}(含 `..` 路径段,不解析)")]);
    let candidates: Vec<PathBuf> = if let Some(uri) = filename.strip_prefix("package://") {
        let Some((pkg, rest)) = uri.split_once('/') else {
            return Err(vec![format!("{filename}(package:// 后须为 <包名>/<路径>)")]);
        };
        if has_parent_seg(rest) {
            return blocked();
        }
        match packages.get(pkg) {
            Some(dir) => vec![dir.join(rest)],
            None => vec![
                base_dir.join(rest),
                base_dir.join("..").join(rest),
                base_dir.join("..").join("..").join(rest),
            ],
        }
    } else if let Some(abs) = filename.strip_prefix("file://") {
        if has_parent_seg(abs) {
            return blocked();
        }
        vec![PathBuf::from(abs)]
    } else {
        if has_parent_seg(filename) {
            return blocked();
        }
        let p = Path::new(filename);
        if p.is_absolute() {
            vec![p.to_path_buf()]
        } else {
            vec![base_dir.join(filename)]
        }
    };
    for c in &candidates {
        if c.exists() {
            return Ok(c.clone());
        }
    }
    Err(candidates.iter().map(|p| p.display().to_string()).collect())
}

/// 文件里的原始网格:顶点 + 三角形下标(任务书 1.4)。
pub(crate) struct RawMesh {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[usize; 3]>,
}

/// OBJ:只认 `v x y z`(第 4 个分量忽略)和 `f …`;其它行忽略。
/// 面下标取 `/` 之前;正下标从 1 起,负下标相对当前顶点数;多边形按扇形拆。
pub(crate) fn parse_obj(text: &str) -> Result<RawMesh, String> {
    let mut vertices: Vec<[f64; 3]> = Vec::new();
    let mut triangles: Vec<[usize; 3]> = Vec::new();
    for (lineno, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut tok = line.split_whitespace();
        match tok.next() {
            Some("v") => {
                let mut c = [0.0f64; 3];
                for slot in c.iter_mut() {
                    let t = tok
                        .next()
                        .ok_or_else(|| format!("第 {} 行:v 不足 3 个分量", lineno + 1))?;
                    *slot = t
                        .parse::<f64>()
                        .map_err(|e| format!("第 {} 行:v 分量 `{t}` 解析失败:{e}", lineno + 1))?;
                    if !slot.is_finite() {
                        return Err(format!("第 {} 行:v 分量非有限", lineno + 1));
                    }
                }
                vertices.push(c);
            }
            Some("f") => {
                let mut idx: Vec<usize> = Vec::new();
                for t in tok {
                    let head = t.split('/').next().unwrap_or("");
                    let i: isize = head
                        .parse()
                        .map_err(|e| format!("第 {} 行:面下标 `{t}` 解析失败:{e}", lineno + 1))?;
                    let n = vertices.len() as isize;
                    let zero = if i > 0 { i - 1 } else { n + i };
                    if zero < 0 || zero >= n {
                        return Err(format!(
                            "第 {} 行:面下标 {i} 越界(当前 {n} 个顶点)",
                            lineno + 1
                        ));
                    }
                    idx.push(zero as usize);
                }
                if idx.len() < 3 {
                    return Err(format!(
                        "第 {} 行:面只有 {} 个顶点(< 3)",
                        lineno + 1,
                        idx.len()
                    ));
                }
                for k in 1..idx.len() - 1 {
                    triangles.push([idx[0], idx[k], idx[k + 1]]);
                }
            }
            _ => {}
        }
    }
    if vertices.is_empty() {
        return Err("OBJ 没有任何顶点".to_owned());
    }
    Ok(RawMesh {
        vertices,
        triangles,
    })
}

/// STL:去空白后以 `solid` 起头且全文含 `facet` → ASCII,否则二进制。
pub(crate) fn parse_stl(bytes: &[u8]) -> Result<RawMesh, String> {
    let start = bytes
        .iter()
        .position(|b| !b.is_ascii_whitespace())
        .unwrap_or(bytes.len());
    let ascii = bytes[start..].starts_with(b"solid") && bytes.windows(5).any(|w| w == b"facet");
    let vertices: Vec<[f64; 3]> = if ascii {
        let text = std::str::from_utf8(bytes).map_err(|e| format!("ASCII STL 非 UTF-8:{e}"))?;
        let mut vertices = Vec::new();
        for (lineno, line) in text.lines().enumerate() {
            let mut tok = line.split_whitespace();
            if tok.next() != Some("vertex") {
                continue;
            }
            let mut c = [0.0f64; 3];
            for slot in c.iter_mut() {
                let t = tok
                    .next()
                    .ok_or_else(|| format!("第 {} 行:vertex 不足 3 个分量", lineno + 1))?;
                *slot = t
                    .parse::<f64>()
                    .map_err(|e| format!("第 {} 行:vertex 分量 `{t}` 解析失败:{e}", lineno + 1))?;
                if !slot.is_finite() {
                    return Err(format!("第 {} 行:vertex 分量非有限", lineno + 1));
                }
            }
            vertices.push(c);
        }
        if vertices.len() % 3 != 0 {
            return Err(format!("ASCII STL 顶点数 {} 不是 3 的倍数", vertices.len()));
        }
        vertices
    } else {
        if bytes.len() < 84 {
            return Err(format!("二进制 STL 长度 {} < 84", bytes.len()));
        }
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as u64;
        let expect = 84 + 50 * n;
        if bytes.len() as u64 != expect {
            return Err(format!(
                "二进制 STL 长度 {} ≠ 84 + 50 × {n} = {expect}",
                bytes.len()
            ));
        }
        let mut vertices = Vec::with_capacity(3 * n as usize);
        for t in 0..n as usize {
            let base = 84 + 50 * t + 12; // 跳过法线,取 3 个顶点
            for k in 0..3 {
                let o = base + 12 * k;
                let mut c = [0.0f64; 3];
                for (j, slot) in c.iter_mut().enumerate() {
                    let v = f32::from_le_bytes([
                        bytes[o + 4 * j],
                        bytes[o + 4 * j + 1],
                        bytes[o + 4 * j + 2],
                        bytes[o + 4 * j + 3],
                    ]);
                    if !v.is_finite() {
                        return Err(format!("二进制 STL 第 {t} 个三角形顶点非有限"));
                    }
                    *slot = f64::from(v);
                }
                vertices.push(c);
            }
        }
        vertices
    };
    if vertices.is_empty() {
        return Err("STL 没有任何顶点".to_owned());
    }
    let triangles = (0..vertices.len() / 3)
        .map(|t| [3 * t, 3 * t + 1, 3 * t + 2])
        .collect();
    Ok(RawMesh {
        vertices,
        triangles,
    })
}

/// 网格统计(MeshNote 去掉 link/file 的部分,任务书 1.5)。
pub(crate) struct MeshStats {
    pub vertices: usize,
    pub triangles: usize,
    pub hull_full: usize,
    pub hull_out: usize,
    pub mesh_volume: f32,
    pub hull_volume: f32,
    pub out_volume: f32,
}

/// 凸包转换结果:写进 ShapeDef 的点 + 统计。
pub(crate) struct HullResult {
    pub points: Vec<[f32; 3]>,
    pub note: MeshStats,
}

/// 按 (x, y, z) 的 f32::total_cmp 字典序(任务书 1.5 第 3 步)。
fn point_cmp(a: &[f32; 3], b: &[f32; 3]) -> Ordering {
    a[0].total_cmp(&b[0])
        .then(a[1].total_cmp(&b[1]))
        .then(a[2].total_cmp(&b[2]))
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn sub(a: [f32; 3], b: [f64; 3]) -> [f64; 3] {
    [
        f64::from(a[0]) - b[0],
        f64::from(a[1]) - b[1],
        f64::from(a[2]) - b[2],
    ]
}

/// 三角形集的带符号四面体体积之和取绝对值(f64 算)。
/// `origin` 先减掉(凸包体积与原点无关,任务书 1.5 第 4 步)。
fn tris_volume(points: &[[f32; 3]], tris: &[[u32; 3]], origin: [f64; 3]) -> f32 {
    let mut sum = 0.0f64;
    for t in tris {
        let a = sub(points[t[0] as usize], origin);
        let b = sub(points[t[1] as usize], origin);
        let c = sub(points[t[2] as usize], origin);
        sum += dot(a, cross(b, c)) / 6.0;
    }
    sum.abs() as f32
}

fn to_vectors(points: &[[f32; 3]]) -> Vec<parry3d::math::Vector3> {
    points
        .iter()
        .map(|p| parry3d::math::Vector3::new(p[0], p[1], p[2]))
        .collect()
}

/// 最远点采样(任务书 1.5 第 6 步):起始 = x/y/z 各轴最小、最大顶点
/// (并列取排序靠前的,重复只留一次);之后每轮取"到已选集合最小距离"
/// 最大的未选顶点(并列取排序靠前的)。返回选入顺序的下标。
fn farthest_sample(sorted: &[[f32; 3]], max_points: usize) -> Vec<usize> {
    let mut selected: Vec<usize> = Vec::new();
    let mut chosen = vec![false; sorted.len()];
    for axis in 0..3 {
        for want_max in [false, true] {
            let mut best = 0;
            for (i, p) in sorted.iter().enumerate() {
                let better = if want_max {
                    p[axis] > sorted[best][axis]
                } else {
                    p[axis] < sorted[best][axis]
                };
                if better {
                    best = i;
                }
            }
            if !chosen[best] {
                chosen[best] = true;
                selected.push(best);
            }
        }
    }
    while selected.len() < max_points {
        let mut best_i = None;
        let mut best_d = -1.0f64;
        for (i, p) in sorted.iter().enumerate() {
            if chosen[i] {
                continue;
            }
            let mut d = f64::INFINITY;
            for &j in &selected {
                let q = sorted[j];
                let dx = f64::from(p[0]) - f64::from(q[0]);
                let dy = f64::from(p[1]) - f64::from(q[1]);
                let dz = f64::from(p[2]) - f64::from(q[2]);
                d = d.min(dx * dx + dy * dy + dz * dz);
            }
            if d > best_d {
                best_d = d;
                best_i = Some(i);
            }
        }
        match best_i {
            Some(i) => {
                chosen[i] = true;
                selected.push(i);
            }
            None => break,
        }
    }
    selected
}

/// 网格 → 凸包(任务书 1.5,步骤顺序固定保证确定)。
/// `scale`:URDF mesh 的 scale;`r_s`:形状系 → 连杆系旋转。
pub(crate) fn hull_from_mesh(
    raw: &RawMesh,
    scale: [f64; 3],
    r_s: &Mat3,
    max_points: u32,
) -> Result<HullResult, String> {
    // 1. 逐分量乘 scale,再乘 r_s,clean 成 f32。
    if scale.iter().any(|s| !s.is_finite() || *s == 0.0) {
        return Err(format!("scale {scale:?} 含非有限或 0 分量"));
    }
    let mut pts: Vec<[f32; 3]> = raw
        .vertices
        .iter()
        .map(|v| {
            let scaled = [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]];
            clean_vec(mat_vec(r_s, scaled))
        })
        .collect();
    // 2. mesh_volume:Σ a·(b×c)/6 取绝对值(f64)。没有面时为 0。
    let mut sum = 0.0f64;
    for t in &raw.triangles {
        let a = pts[t[0]].map(f64::from);
        let b = pts[t[1]].map(f64::from);
        let c = pts[t[2]].map(f64::from);
        sum += dot(a, cross(b, c)) / 6.0;
    }
    let mesh_volume = sum.abs() as f32;
    // 3. 排序去重。
    pts.sort_by(point_cmp);
    pts.dedup();
    // 4. 完整凸包;顶点排序,hull_volume 先减第一个顶点再算。
    let (hull_pts, hull_tris) = parry3d::transformation::try_convex_hull(&to_vectors(&pts))
        .map_err(|e| format!("try_convex_hull 失败:{e}"))?;
    let hull_verts: Vec<[f32; 3]> = hull_pts.iter().map(|v| [v.x, v.y, v.z]).collect();
    let origin = hull_verts[0].map(f64::from);
    let hull_volume = tris_volume(&hull_verts, &hull_tris, origin);
    let mut sorted = hull_verts;
    sorted.sort_by(point_cmp);
    let hull_full = sorted.len();
    // 5/6. 不超上限直接输出;否则最远点采样精简后再建一次凸包。
    let (points, out_volume) = if hull_full <= max_points as usize {
        (sorted, hull_volume)
    } else {
        let sel = farthest_sample(&sorted, max_points as usize);
        let out_pts: Vec<[f32; 3]> = sel.iter().map(|&i| sorted[i]).collect();
        let (_, tris) = parry3d::transformation::try_convex_hull(&to_vectors(&out_pts))
            .map_err(|e| format!("精简后 try_convex_hull 失败:{e}"))?;
        let origin = out_pts[0].map(f64::from);
        let vol = tris_volume(&out_pts, &tris, origin);
        (out_pts, vol)
    };
    // 7. 少于 4 个点建不出碰撞体。
    if points.len() < 4 {
        return Err(format!("输出点数 {} < 4", points.len()));
    }
    let hull_out = points.len();
    Ok(HullResult {
        points,
        note: MeshStats {
            vertices: raw.vertices.len(),
            triangles: raw.triangles.len(),
            hull_full,
            hull_out,
            mesh_volume,
            hull_volume,
            out_volume,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn obj_vertices_faces_and_fan() {
        let text = "# c\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0 9\nvn 0 0 1\nf 1 2 3 4\n";
        let m = parse_obj(text).expect("解析成功");
        assert_eq!(m.vertices.len(), 4);
        assert_eq!(m.vertices[3], [0.0, 1.0, 0.0], "第 4 分量忽略");
        assert_eq!(m.triangles, vec![[0, 1, 2], [0, 2, 3]], "四边形扇形拆");
    }

    #[test]
    fn obj_negative_and_slash_indices() {
        let text = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3/1/1 -2/2/1 -1/3/1\n";
        let m = parse_obj(text).expect("解析成功");
        assert_eq!(m.triangles, vec![[0, 1, 2]]);
    }

    #[test]
    fn obj_no_faces_ok() {
        let m = parse_obj("v 0 0 0\nv 1 0 0\n").expect("只有 v 行也合法");
        assert!(m.triangles.is_empty());
        assert_eq!(m.vertices.len(), 2);
    }

    #[test]
    fn obj_errors() {
        for bad in [
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n", // 越界
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 0\n", // 0 下标
            "v 0 0 x\n",                            // 数字解析失败
            "v 0 0 0\nv 1 0 0\nf 1 2\n",            // 面 < 3 顶点
            "v nan 0 0\n",                          // 非有限
            "# 只有注释\n",                         // 无顶点
            "v 0 0\n",                              // v 分量不足
        ] {
            assert!(parse_obj(bad).is_err(), "应报错:{bad:?}");
        }
    }

    #[test]
    fn stl_ascii_one_facet() {
        let text = "solid t\nfacet normal 0 0 0\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n";
        let m = parse_stl(text.as_bytes()).expect("ASCII 解析成功");
        assert_eq!(m.vertices.len(), 3);
        assert_eq!(m.triangles, vec![[0, 1, 2]]);
    }

    #[test]
    fn stl_ascii_bad_vertex_count() {
        let text = "solid t\nfacet normal 0 0 0\nvertex 0 0 0\nvertex 1 0 0\n";
        assert!(
            parse_stl(text.as_bytes()).is_err(),
            "顶点数 2 不是 3 的倍数"
        );
    }

    /// 手工拼一个二进制 STL(1 个三角形)。
    fn binary_one_tri() -> Vec<u8> {
        let mut b = vec![0u8; 80];
        b.extend_from_slice(&1u32.to_le_bytes());
        b.extend_from_slice(&[0.0f32; 3].map(f32::to_le_bytes).concat());
        for v in [[0.0f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
            b.extend_from_slice(&v.map(f32::to_le_bytes).concat());
        }
        b.extend_from_slice(&[0u8; 2]);
        b
    }

    #[test]
    fn stl_binary_roundtrip_and_length() {
        let b = binary_one_tri();
        assert_eq!(b.len(), 134);
        let m = parse_stl(&b).expect("二进制解析成功");
        assert_eq!(m.vertices.len(), 3);
        assert_eq!(m.vertices[1], [1.0, 0.0, 0.0]);
        assert_eq!(m.triangles, vec![[0, 1, 2]]);
        assert!(parse_stl(&b[..133]).is_err(), "长度不对应报错");
        assert!(parse_stl(&b[..40]).is_err(), "不足 84 应报错");
    }

    #[test]
    fn stl_solid_header_without_facet_is_binary() {
        // 头 5 字节恰为 "solid" 的二进制文件:无 "facet" → 走二进制。
        let mut b = binary_one_tri();
        b[..5].copy_from_slice(b"solid");
        let m = parse_stl(&b).expect("按二进制解析");
        assert_eq!(m.triangles.len(), 1);
    }

    #[test]
    fn resolve_rules() {
        let tmp = tempfile::TempDir::new().expect("临时目录");
        let base = tmp.path().join("pkg").join("urdf");
        let meshes = tmp.path().join("pkg").join("meshes");
        std::fs::create_dir_all(&base).expect("建 urdf 目录");
        std::fs::create_dir_all(&meshes).expect("建 meshes 目录");
        std::fs::write(meshes.join("box.obj"), "v 0 0 0\n").expect("写网格");
        let empty = BTreeMap::new();

        // 回退第二候选 base_dir/../meshes/box.obj。
        let p = resolve("package://pkg/meshes/box.obj", &base, &empty).expect("回退找到");
        assert!(p.ends_with("meshes/box.obj") || p.ends_with("meshes\\box.obj"));
        // 都不存在 → 3 条候选。
        let tried = resolve("package://pkg/meshes/none.stl", &base, &empty).expect_err("找不到");
        assert_eq!(tried.len(), 3);
        // 显式包目录:只试 1 条。
        let mut pkgs = BTreeMap::new();
        pkgs.insert("pkg".to_owned(), tmp.path().join("pkg"));
        let p = resolve("package://pkg/meshes/box.obj", &base, &pkgs).expect("显式包目录");
        assert_eq!(p, meshes.join("box.obj"));
        let tried = resolve("package://pkg/none.stl", &base, &pkgs).expect_err("只试包目录");
        assert_eq!(tried.len(), 1);
        // 相对路径 / file:// / 绝对路径。
        let p = resolve("meshes/box.obj", &meshes.parent().expect("父目录"), &empty)
            .expect("相对 base_dir");
        assert_eq!(p, meshes.join("box.obj"));
        let abs = meshes.join("box.obj");
        let uri = format!("file://{}", abs.display());
        assert_eq!(resolve(&uri, &base, &empty).expect("file://"), abs);
        assert_eq!(
            resolve(&abs.display().to_string(), &base, &empty).expect("绝对路径"),
            abs
        );
        // `..` 拦截:目标真实存在也不解析。
        std::fs::write(tmp.path().join("pkg").join("secret.obj"), "v 0 0 0\n").expect("写");
        let tried = resolve("../secret.obj", &base, &empty).expect_err("`..` 应拦截");
        assert!(tried[0].contains(".."), "候选文本写明含 ..:{tried:?}");
        assert!(resolve("package://pkg/../secret.obj", &base, &empty).is_err());
    }
}
