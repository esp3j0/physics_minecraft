//! cli —— viewer 命令行参数(REQ-008 §4):交互模式或脚本模式。
//!
//! 写法:`viewer [--addr <地址>] [--script <场景文件> --out <输出目录>]
//! [--stage <场景文件>]`。
//! - 不带参数 → 交互模式(行为同 REQ-004A);
//! - `--script` 与 `--out` 必须同时给出;
//! - `--stage` 布景交互(REQ-011 §3.6,W05):交互模式下自动执行场景的布景段,
//!   与 `--script` 互斥,也不接受 `--out`(脚本模式输出目录对布景无意义);
//! - `--addr` 缺省时由调用方回落到环境变量 `VIEWER_ADDR` 或 `127.0.0.1:25565`;
//! - 未知参数、缺少参数值、重复参数 → 错误。

use std::path::PathBuf;

/// 运行模式。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ViewerMode {
    Interactive,
    Script {
        script: PathBuf,
        out: PathBuf,
    },
    /// 布景交互(REQ-011 §3.6;W05):交互模式 + 场景文件。
    Stage {
        script: PathBuf,
    },
}

/// 解析后的命令行参数。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViewerArgs {
    pub mode: ViewerMode,
    pub addr: Option<String>,
}

/// 解析命令行参数(不含程序名)。
pub fn parse_args<I: IntoIterator<Item = String>>(args: I) -> Result<ViewerArgs, String> {
    let mut addr: Option<String> = None;
    let mut script: Option<PathBuf> = None;
    let mut out: Option<PathBuf> = None;
    let mut stage: Option<PathBuf> = None;

    let mut iter = args.into_iter();
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--addr" => {
                if addr.is_some() {
                    return Err("--addr 重复给出".to_owned());
                }
                let value = iter.next().ok_or_else(|| "--addr 缺少参数值".to_owned())?;
                addr = Some(value);
            }
            "--script" => {
                if script.is_some() {
                    return Err("--script 重复给出".to_owned());
                }
                let value = iter
                    .next()
                    .ok_or_else(|| "--script 缺少参数值".to_owned())?;
                script = Some(PathBuf::from(value));
            }
            "--out" => {
                if out.is_some() {
                    return Err("--out 重复给出".to_owned());
                }
                let value = iter.next().ok_or_else(|| "--out 缺少参数值".to_owned())?;
                out = Some(PathBuf::from(value));
            }
            "--stage" => {
                if stage.is_some() {
                    return Err("--stage 重复给出".to_owned());
                }
                let value = iter.next().ok_or_else(|| "--stage 缺少参数值".to_owned())?;
                stage = Some(PathBuf::from(value));
            }
            other => return Err(format!("未知参数:{other}")),
        }
    }

    let mode = match (script.is_some(), out.is_some(), stage.is_some()) {
        (false, false, false) => ViewerMode::Interactive,
        (true, true, false) => ViewerMode::Script {
            script: script.unwrap(),
            out: out.unwrap(),
        },
        (false, false, true) => ViewerMode::Stage {
            script: stage.unwrap(),
        },
        (true, false, false) => return Err("--script 必须与 --out 成对给出".to_owned()),
        (false, true, false) => return Err("--out 必须与 --script 成对给出".to_owned()),
        (_, _, true) if script.is_some() => {
            return Err("--stage 与 --script 互斥:布景在交互模式下运行,不走脚本引擎".to_owned());
        }
        (_, _, true) => {
            return Err("--stage 不接受 --out:布景交互没有截图输出目录".to_owned());
        }
    };
    Ok(ViewerArgs { mode, addr })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(list: &[&str]) -> Result<ViewerArgs, String> {
        parse_args(list.iter().map(|s| (*s).to_owned()))
    }

    /// 无参数 → 交互模式、addr None;有 --addr 时透传(锚定缺省回落由调用方做)。
    #[test]
    fn cli_no_args_is_interactive() {
        assert_eq!(
            args(&[]),
            Ok(ViewerArgs {
                mode: ViewerMode::Interactive,
                addr: None
            })
        );
        assert_eq!(
            args(&["--addr", "1.2.3.4:9"]),
            Ok(ViewerArgs {
                mode: ViewerMode::Interactive,
                addr: Some("1.2.3.4:9".to_owned())
            })
        );
    }

    /// 成对与乱序;不成对 / 未知 / 缺值 / 重复各自报错(锚定 AC-S4 命令行语义;
    /// 断言若失效——例如缺 --out 被静默容忍——本测试先红)。
    #[test]
    fn cli_pairs_and_errors() {
        assert_eq!(
            args(&["--script", "s.toml", "--out", "o"]),
            Ok(ViewerArgs {
                mode: ViewerMode::Script {
                    script: "s.toml".into(),
                    out: "o".into()
                },
                addr: None
            })
        );
        assert_eq!(
            args(&["--out", "o", "--script", "s.toml"]),
            Ok(ViewerArgs {
                mode: ViewerMode::Script {
                    script: "s.toml".into(),
                    out: "o".into()
                },
                addr: None
            })
        );
        for (bad, what) in [
            (&["--script", "s"][..], "缺 out"),
            (&["--out", "o"], "缺 script"),
            (&["--script"], "script 缺值"),
            (&["--addr"], "addr 缺值"),
            (
                &["--script", "a", "--out", "o", "--script", "b"],
                "重复 script",
            ),
            (&["--out", "a", "--out", "b", "--script", "s"], "重复 out"),
            (&["--addr", "x", "--addr", "y"], "重复 addr"),
            (&["--fullscreen"], "未知参数"),
            (&["s.toml"], "位置参数"),
        ] {
            assert!(args(bad).is_err(), "{what}:应报错");
        }
    }

    /// `--addr` 的值可以形如任意串(解析成 SocketAddr 是调用方的事),不被吞。
    #[test]
    fn cli_addr_value_is_verbatim() {
        assert_eq!(
            args(&["--script", "s", "--out", "o", "--addr", "localhost:1"]),
            Ok(ViewerArgs {
                mode: ViewerMode::Script {
                    script: "s".into(),
                    out: "o".into()
                },
                addr: Some("localhost:1".to_owned())
            })
        );
    }

    /// `--stage` 单独给出 → Stage 模式(REQ-011 §3.6,W05);与 `--script`
    /// 互斥、不接受 `--out`(AC-W13 CLI 解析测试);缺值 / 重复各自报错。
    #[test]
    fn cli_stage_mode_and_exclusions() {
        assert_eq!(
            args(&["--stage", "scene.toml"]),
            Ok(ViewerArgs {
                mode: ViewerMode::Stage {
                    script: "scene.toml".into()
                },
                addr: None
            })
        );
        assert_eq!(
            args(&["--stage", "scene.toml", "--addr", "1.2.3.4:9"]),
            Ok(ViewerArgs {
                mode: ViewerMode::Stage {
                    script: "scene.toml".into()
                },
                addr: Some("1.2.3.4:9".to_owned())
            })
        );
        for (bad, what) in [
            (
                &["--stage", "a", "--script", "s", "--out", "o"][..],
                "stage+script",
            ),
            (&["--stage", "a", "--out", "o"], "stage+out"),
            (&["--stage"], "stage 缺值"),
            (&["--stage", "a", "--stage", "b"], "重复 stage"),
        ] {
            assert!(args(bad).is_err(), "{what}:应报错");
        }
    }
}
