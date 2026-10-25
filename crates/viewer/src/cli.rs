//! cli —— viewer 命令行参数(REQ-008 §4):交互模式或脚本模式。
//!
//! 写法:`viewer [--addr <地址>] [--script <场景文件> --out <输出目录>]`。
//! - 不带参数 → 交互模式(行为同 REQ-004A);
//! - `--script` 与 `--out` 必须同时给出;
//! - `--addr` 缺省时由调用方回落到环境变量 `VIEWER_ADDR` 或 `127.0.0.1:25565`;
//! - 未知参数、缺少参数值、重复参数 → 错误。

use std::path::PathBuf;

/// 运行模式。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ViewerMode {
    Interactive,
    Script { script: PathBuf, out: PathBuf },
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
            other => return Err(format!("未知参数:{other}")),
        }
    }

    let mode = match (script, out) {
        (None, None) => ViewerMode::Interactive,
        (Some(script), Some(out)) => ViewerMode::Script { script, out },
        (Some(_), None) => return Err("--script 必须与 --out 成对给出".to_owned()),
        (None, Some(_)) => return Err("--out 必须与 --script 成对给出".to_owned()),
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
}
