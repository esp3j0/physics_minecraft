//! REQ-012 C01a:`cargo xtask import-body <robot.urdf> --out <路径.body.toml>`
//! —— URDF → body.toml 子命令。
//!
//! 读 URDF → [`body_import::import_urdf`] → 内存里生成完整 toml 文本与报告
//! 文本,都成功才写盘;失败时不创建、不修改 `out` 与 `report`。成功时标准
//! 输出打印报告(或写 `--report` 文件),退出码 0;失败时标准错误打印错误,
//! 退出码非 0。

use std::path::{Path, PathBuf};

use body_import::{ImportOptions, UpAxis};

/// `import-body` 的全部参数(子命令名之后的部分)。
pub struct ImportBodyArgs {
    pub urdf: PathBuf,
    pub out: PathBuf,
    pub name: Option<String>,
    pub up: UpAxis,
    pub kp: f32,
    pub kd: f32,
    pub friction: f32,
    pub pose: Vec<(String, f32)>,
    pub cylinder_segments: u32,
    pub max_hull_points: u32,
    pub packages: std::collections::BTreeMap<String, PathBuf>,
    pub report: Option<PathBuf>,
}

impl Default for ImportBodyArgs {
    fn default() -> Self {
        Self {
            urdf: PathBuf::new(),
            out: PathBuf::new(),
            name: None,
            up: UpAxis::Z,
            kp: 100.0,
            kd: 3.0,
            friction: 0.6,
            pose: Vec::new(),
            cylinder_segments: 16,
            max_hull_points: 32,
            packages: std::collections::BTreeMap::new(),
            report: None,
        }
    }
}

/// 解析命令行参数(子命令名之后的部分)。未知参数、缺值、数字解析失败都返回 Err。
pub fn parse_args(args: &[String]) -> Result<ImportBodyArgs, String> {
    let mut parsed = ImportBodyArgs::default();
    let mut urdf: Option<String> = None;
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--out" => {
                if !parsed.out.as_os_str().is_empty() {
                    return Err("--out 重复给出".to_owned());
                }
                let value = iter.next().ok_or_else(|| "--out 缺少参数值".to_owned())?;
                parsed.out = PathBuf::from(value);
            }
            "--name" => {
                if parsed.name.is_some() {
                    return Err("--name 重复给出".to_owned());
                }
                let value = iter.next().ok_or_else(|| "--name 缺少参数值".to_owned())?;
                parsed.name = Some(value.clone());
            }
            "--up" => {
                let value = iter.next().ok_or_else(|| "--up 缺少参数值".to_owned())?;
                parsed.up = match value.as_str() {
                    "z" => UpAxis::Z,
                    "y" => UpAxis::Y,
                    other => return Err(format!("--up 只接受 z 或 y,实为 `{other}`")),
                };
            }
            "--kp" => parsed.kp = parse_f32("--kp", iter.next())?,
            "--kd" => parsed.kd = parse_f32("--kd", iter.next())?,
            "--friction" => parsed.friction = parse_f32("--friction", iter.next())?,
            "--pose" => {
                let value = iter.next().ok_or_else(|| "--pose 缺少参数值".to_owned())?;
                let (joint, angle) = value
                    .split_once('=')
                    .ok_or_else(|| format!("--pose 须为 <关节>=<角>,实为 `{value}`"))?;
                let angle = angle
                    .parse::<f32>()
                    .map_err(|e| format!("--pose 角度解析失败 `{value}`:{e}"))?;
                parsed.pose.push((joint.to_owned(), angle));
            }
            "--cylinder-segments" => {
                let value = iter
                    .next()
                    .ok_or_else(|| "--cylinder-segments 缺少参数值".to_owned())?;
                parsed.cylinder_segments = value
                    .parse::<u32>()
                    .map_err(|e| format!("--cylinder-segments 解析失败 `{value}`:{e}"))?;
            }
            "--max-hull-points" => {
                let value = iter
                    .next()
                    .ok_or_else(|| "--max-hull-points 缺少参数值".to_owned())?;
                parsed.max_hull_points = value
                    .parse::<u32>()
                    .map_err(|e| format!("--max-hull-points 解析失败 `{value}`:{e}"))?;
            }
            "--package" => {
                let value = iter
                    .next()
                    .ok_or_else(|| "--package 缺少参数值".to_owned())?;
                let (name, dir) = value
                    .split_once('=')
                    .ok_or_else(|| format!("--package 须为 <名>=<目录>,实为 `{value}`"))?;
                if parsed
                    .packages
                    .insert(name.to_owned(), PathBuf::from(dir))
                    .is_some()
                {
                    return Err(format!("--package 包名 `{name}` 重复给出"));
                }
            }
            "--report" => {
                if parsed.report.is_some() {
                    return Err("--report 重复给出".to_owned());
                }
                let value = iter
                    .next()
                    .ok_or_else(|| "--report 缺少参数值".to_owned())?;
                parsed.report = Some(PathBuf::from(value));
            }
            other => {
                if other.starts_with("--") {
                    return Err(format!("未知参数 `{other}`"));
                }
                if urdf.is_some() {
                    return Err(format!("多余的位置参数 `{other}`(URDF 只给一个)"));
                }
                urdf = Some((*arg).to_owned());
            }
        }
    }
    let Some(urdf) = urdf else {
        return Err("缺少 URDF 路径".to_owned());
    };
    parsed.urdf = PathBuf::from(urdf);
    if parsed.out.as_os_str().is_empty() {
        return Err("缺少 --out <路径.body.toml>".to_owned());
    }
    Ok(parsed)
}

fn parse_f32(flag: &str, value: Option<&String>) -> Result<f32, String> {
    let value = value.ok_or_else(|| format!("{flag} 缺少参数值"))?;
    value
        .parse::<f32>()
        .map_err(|e| format!("{flag} 解析失败 `{value}`:{e}"))
}

/// 读 URDF → 导入 → 写文件。返回报告的 markdown。先在内存里生成完整的 toml
/// 文本和报告文本,都成功了才写盘;失败时不创建、不修改 `out` 和 `report`。
pub fn run(args: &ImportBodyArgs) -> Result<String, String> {
    let urdf_xml = std::fs::read_to_string(&args.urdf)
        .map_err(|e| format!("读 {} 失败:{e}", args.urdf.display()))?;
    let base_dir = args
        .urdf
        .parent()
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."));
    let name = match &args.name {
        Some(name) => name.clone(),
        None => {
            body_import::robot_name(&urdf_xml).map_err(|e| format!("取 robot name 失败:{e}"))?
        }
    };
    let mut default_angles = std::collections::BTreeMap::new();
    for (joint, angle) in &args.pose {
        if default_angles.insert(joint.clone(), *angle).is_some() {
            return Err(format!("--pose 关节 `{joint}` 重复给出"));
        }
    }
    let opts = ImportOptions {
        name,
        up_axis: args.up,
        kp: args.kp,
        kd: args.kd,
        friction: args.friction,
        default_angles,
        cylinder_segments: args.cylinder_segments,
        max_hull_points: args.max_hull_points,
        package_dirs: args.packages.clone(),
    };
    let (def, report) = body_import::import_urdf(&urdf_xml, &base_dir, &opts)
        .map_err(|e| format!("导入失败:{e}"))?;
    let toml_text = body_import::body_to_toml(&def);
    let report_md = body_import::report_to_markdown(&report);

    // 带注释的完整 toml 文本(注释在后加,避免混进 canonical 文本)。
    let source_name = args
        .urdf
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("?")
        .to_owned();
    let pose_text = args
        .pose
        .iter()
        .map(|(j, a)| format!("{j}={a}"))
        .collect::<Vec<_>>()
        .join(",");
    let up_text = match args.up {
        UpAxis::Z => "z",
        UpAxis::Y => "y",
    };
    let header = format!(
        "# 来源:{source_name}\n# 选项:up={up_text} kp={} kd={} friction={} cylinder_segments={} max_hull_points={} pose={pose_text}\n# 由 cargo xtask import-body 生成,不要手改\n",
        args.kp, args.kd, args.friction, args.cylinder_segments, args.max_hull_points,
    );
    let full_toml = format!("{header}{toml_text}");
    // 注释是 toml 行注释,加完仍须可解析(这里先验,再写盘)。
    types::BodyDef::from_toml_str(&full_toml)
        .map_err(|e| format!("带注释的输出不能解析(内部错误):{e}"))?;

    if let Some(report_path) = &args.report {
        std::fs::write(report_path, &report_md)
            .map_err(|e| format!("写 {} 失败:{e}", report_path.display()))?;
    }
    std::fs::write(&args.out, &full_toml)
        .map_err(|e| format!("写 {} 失败:{e}", args.out.display()))?;
    Ok(report_md)
}

/// `cargo xtask` 的子命令入口(成功打印报告、返回 0;失败打印错误、返回 2)。
pub fn cmd_import_body(args: &[String]) -> i32 {
    match parse_args(args).and_then(|parsed| run(&parsed)) {
        Ok(report_md) => {
            print!("{report_md}");
            0
        }
        Err(e) => {
            eprintln!("import-body 失败:{e}");
            2
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn data_dir() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../body-import/tests/data")
            .components()
            .collect()
    }

    fn testbot() -> PathBuf {
        data_dir().join("testbot.urdf")
    }

    fn args(urdf: &Path, out: &Path, report: Option<&Path>) -> ImportBodyArgs {
        ImportBodyArgs {
            urdf: urdf.to_path_buf(),
            out: out.to_path_buf(),
            report: report.map(Path::to_path_buf),
            ..ImportBodyArgs::default()
        }
    }

    /// x1:对 testbot 调 run(带 --report)→ 输出文件存在;from_toml_str 能
    /// 解析;hash() 等于直接调 import_urdf 的 hash();前三行都以 # 开头;
    /// 报告文件存在且含 15。
    #[test]
    fn x1_import_testbot() {
        let tmp = tempfile::TempDir::new().expect("临时目录");
        let out = tmp.path().join("testbot.body.toml");
        let report = tmp.path().join("report.md");
        let report_md = run(&args(&testbot(), &out, Some(&report))).expect("run 成功");
        assert!(!report_md.is_empty());
        let out_path = tmp.path().join("testbot.body.toml");
        let text = std::fs::read_to_string(&out_path).expect("输出文件存在");
        let first_three = text.lines().take(3).collect::<Vec<_>>();
        assert_eq!(first_three.len(), 3);
        for line in &first_three {
            assert!(line.starts_with('#'), "前三行都是注释:{line}");
        }
        let def = types::BodyDef::from_toml_str(&text).expect("带注释输出可解析");
        let opts = ImportOptions {
            name: "testbot".to_owned(),
            up_axis: UpAxis::Z,
            kp: 100.0,
            kd: 3.0,
            friction: 0.6,
            default_angles: std::collections::BTreeMap::new(),
            cylinder_segments: 16,
            max_hull_points: 32,
            package_dirs: std::collections::BTreeMap::new(),
        };
        let xml = std::fs::read_to_string(testbot()).expect("读 testbot");
        let (direct, _) = body_import::import_urdf(&xml, &data_dir(), &opts).expect("直接导入");
        assert_eq!(def.hash(), direct.hash(), "写出文件与直接导入同哈希");
        let report_text = std::fs::read_to_string(&report).expect("报告文件存在");
        assert!(report_text.contains("15"), "报告含总质量 15:{report_text}");
    }

    /// x2:闭环 URDF → Err,out/report 上没有文件;out 预置 `old` → Err 且
    /// 内容不变。六个惯量属性写全,保证 URDF 能解析、失败发生在**导入阶段**
    /// (闭环检查),断言错误文本含"闭环"确认原因(返修 R1 存活 2:原 URDF
    /// 缺 ixy 等在解析阶段就报错,没走到导入)。
    #[test]
    fn x2_failure_leaves_nothing() {
        let tmp = tempfile::TempDir::new().expect("临时目录");
        let urdf = tmp.path().join("loop.urdf");
        std::fs::write(
            &urdf,
            r#"<?xml version="1.0"?>
<robot name="loop">
  <link name="a"><inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <link name="b"><inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="ab" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 1"/></joint>
  <joint name="ba" type="fixed"><parent link="b"/><child link="a"/><origin xyz="0 0 -1"/></joint>
</robot>
"#,
        )
        .expect("写闭环 URDF");
        let out = tmp.path().join("loop.body.toml");
        let report = tmp.path().join("report.md");
        let result = run(&args(&urdf, &out, Some(&report)));
        let err = result.expect_err("闭环应导入失败");
        assert!(err.contains("闭环"), "失败原因应是 Cycle(闭环),实为:{err}");
        assert!(!out.exists(), "失败时不创建 out");
        assert!(!report.exists(), "失败时不创建 report");

        // 预置旧文件:失败后内容不变。
        std::fs::write(&out, "old").expect("预置 old");
        let result = run(&args(&urdf, &out, Some(&report)));
        assert!(result.is_err());
        assert_eq!(
            std::fs::read_to_string(&out).expect("旧文件还在"),
            "old",
            "失败时不改写 out"
        );
        assert!(!report.exists());
    }

    /// x2b(返修 R1):非法 XML(`<robot`)→ Err,out、report 都不存在
    /// (解析阶段失败同样零副作用)。
    #[test]
    fn x2b_bad_xml_leaves_nothing() {
        let tmp = tempfile::TempDir::new().expect("临时目录");
        let urdf = tmp.path().join("bad.urdf");
        std::fs::write(&urdf, "<robot").expect("写非法 XML");
        let out = tmp.path().join("bad.body.toml");
        let report = tmp.path().join("report.md");
        let result = run(&args(&urdf, &out, Some(&report)));
        assert!(result.is_err(), "非法 XML 应失败");
        assert!(!out.exists(), "失败时不创建 out");
        assert!(!report.exists(), "失败时不创建 report");
    }

    /// x3:参数解析各字段;缺 --out、--up 非法、--pose 无 =、未知参数都报 Err。
    #[test]
    fn x3_parse_args() {
        let a = parse_args(&[
            "a.urdf".to_owned(),
            "--out".to_owned(),
            "b.toml".to_owned(),
            "--up".to_owned(),
            "y".to_owned(),
            "--pose".to_owned(),
            "hip_joint=0.5".to_owned(),
            "--pose".to_owned(),
            "thigh_joint=-1".to_owned(),
        ])
        .expect("合法参数");
        assert_eq!(a.urdf, PathBuf::from("a.urdf"));
        assert_eq!(a.out, PathBuf::from("b.toml"));
        assert_eq!(a.up, UpAxis::Y);
        assert_eq!(
            a.pose,
            vec![
                ("hip_joint".to_owned(), 0.5),
                ("thigh_joint".to_owned(), -1.0)
            ]
        );
        assert_eq!(a.kp, 100.0);
        assert_eq!(a.kd, 3.0);
        assert_eq!(a.friction, 0.6);
        assert_eq!(a.cylinder_segments, 16);
        assert!(a.name.is_none());
        assert!(a.report.is_none());

        for bad in [
            vec!["a.urdf"],
            vec!["a.urdf", "--out", "b.toml", "--up", "w"],
            vec!["a.urdf", "--out", "b.toml", "--pose", "bad"],
            vec!["a.urdf", "--out", "b.toml", "--zzz", "1"],
        ] {
            let args: Vec<String> = bad.iter().map(|s| (*s).to_owned()).collect();
            assert!(parse_args(&args).is_err(), "应报错:{bad:?}");
        }
    }

    /// x4(C01b):--max-hull-points 与可重复的 --package 解析;
    /// --package 无 `=`、--max-hull-points 非数字都报 Err。
    #[test]
    fn x4_parse_mesh_args() {
        let a = parse_args(&[
            "a.urdf".to_owned(),
            "--out".to_owned(),
            "b.toml".to_owned(),
            "--max-hull-points".to_owned(),
            "16".to_owned(),
            "--package".to_owned(),
            "a=dirA".to_owned(),
            "--package".to_owned(),
            "b=dirB".to_owned(),
        ])
        .expect("合法参数");
        assert_eq!(a.max_hull_points, 16);
        assert_eq!(a.packages.len(), 2);
        assert_eq!(a.packages["a"], PathBuf::from("dirA"));
        assert_eq!(a.packages["b"], PathBuf::from("dirB"));
        // 默认值。
        let d = parse_args(&["a.urdf".to_owned(), "--out".to_owned(), "b.toml".to_owned()])
            .expect("默认参数");
        assert_eq!(d.max_hull_points, 32);
        assert!(d.packages.is_empty());
        for bad in [
            vec!["a.urdf", "--out", "b.toml", "--package", "bad"],
            vec!["a.urdf", "--out", "b.toml", "--max-hull-points", "x"],
        ] {
            let args: Vec<String> = bad.iter().map(|s| (*s).to_owned()).collect();
            assert!(parse_args(&args).is_err(), "应报错:{bad:?}");
        }
    }
}
