//! zcode —— 真实审图器:调 zcode CLI 审截图(REQ-008 §6;单元 04 放行评论范围 2)。
//!
//! 调用形式(顺序执行,zcode 本地会话库不支持并发):
//! `node <zcode.mjs> -p <提示词> --attach <截图> --mode plan --json --cwd <截图所在目录>`;
//! `zcode.mjs` 路径取环境变量 `ZCODE_MJS`,缺省
//! `%LOCALAPPDATA%\zcode\runtime\current\bin\zcode.mjs`;模型用 zcode 默认配置;
//! 单次超时 180 s;标准输出为 UTF-8 JSON,取 `response` 字段作为模型回复。
//! 调用失败、超时、非 0 退出、JSON 缺字段都报错(上层转 `Unknown`),不 panic。

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Duration;

use crate::review::Reviewer;

/// 单次调用超时(REQ-008 §6:180 s)。
const CALL_TIMEOUT: Duration = Duration::from_secs(180);

/// zcode CLI 审图器。
pub struct ZcodeReviewer {
    /// `zcode.mjs` 的路径(构造时解析)。
    script: PathBuf,
    /// 进度输出(逐张审图时避免看起来卡住;放行评论风险 5)。
    progress: bool,
}

/// 解析 `zcode.mjs` 路径:`ZCODE_MJS` 优先,缺省 `%LOCALAPPDATA%\zcode\runtime\current\bin\zcode.mjs`;
/// 文件不存在或两个环境变量都缺 → 错误(让调用方在审图开始前就知道,而不是每张图报一遍)。
/// 环境取值作参数传入(纯函数;`new` 负责读真实环境)。
fn resolve_script(
    zcode_mjs: Option<OsString>,
    localappdata: Option<OsString>,
) -> Result<PathBuf, String> {
    let script = match zcode_mjs {
        Some(path) => PathBuf::from(path),
        None => {
            let local = localappdata.ok_or_else(|| {
                "环境变量 ZCODE_MJS 与 LOCALAPPDATA 都未设置,找不到 zcode.mjs".to_owned()
            })?;
            PathBuf::from(local)
                .join("zcode")
                .join("runtime")
                .join("current")
                .join("bin")
                .join("zcode.mjs")
        }
    };
    if !script.is_file() {
        return Err(format!("zcode.mjs 不存在:{}", script.display()));
    }
    Ok(script)
}

impl ZcodeReviewer {
    /// 用环境变量解析 `zcode.mjs`;文件不存在 → 错误。
    pub fn new() -> Result<Self, String> {
        Ok(Self {
            script: resolve_script(
                std::env::var_os("ZCODE_MJS"),
                std::env::var_os("LOCALAPPDATA"),
            )?,
            progress: false,
        })
    }

    /// 是否打印每次调用的进度(默认关;编排器开启)。
    pub fn with_progress(mut self, progress: bool) -> Self {
        self.progress = progress;
        self
    }
}

impl Reviewer for ZcodeReviewer {
    fn review(&mut self, image: &Path, prompt: &str) -> Result<String, String> {
        if self.progress {
            println!("    调用 zcode 审 {} …", image.display());
        }
        // 提示词经命令行参数传递;参数数组直接交给 spawn,不经 shell,不解析引号。
        let mut command = Command::new("node");
        command
            .arg(&self.script)
            .arg("-p")
            .arg(prompt)
            .arg("--attach")
            .arg(image)
            .arg("--mode")
            .arg("plan")
            .arg("--json")
            .arg("--cwd")
            .arg(image.parent().unwrap_or_else(|| Path::new(".")));
        let mut child = command
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| format!("启动 node 失败:{e}"))?;
        // 回复可能很长(风险 2):读管道放独立线程,避免子进程写满管道缓冲
        // 被阻塞、我们却在等退出的死锁。子进程退出(或超时被杀)后管道关闭,
        // 读线程随之结束。
        let stdout_pipe = child
            .stdout
            .take()
            .ok_or_else(|| "拿不到 zcode 标准输出管道".to_owned())?;
        let stderr_pipe = child
            .stderr
            .take()
            .ok_or_else(|| "拿不到 zcode 标准错误管道".to_owned())?;
        fn read_all<R: std::io::Read>(mut pipe: R) -> String {
            let mut text = String::new();
            let _ = std::io::Read::read_to_string(&mut pipe, &mut text);
            text
        }
        let stdout_reader = std::thread::spawn(move || read_all(stdout_pipe));
        let stderr_reader = std::thread::spawn(move || read_all(stderr_pipe));

        // 调用超时的墙钟(本 crate 是本机工具,不在确定性核内;lib.rs 顶部已注明)。
        #[allow(clippy::disallowed_methods)]
        let started = std::time::Instant::now();
        let mut timed_out = false;
        let status = loop {
            match child.try_wait() {
                Ok(Some(status)) => break Some(status),
                Ok(None) => {}
                Err(e) => {
                    let _ = child.kill();
                    let _ = child.wait();
                    return Err(format!("等待 zcode 失败:{e}"));
                }
            }
            #[allow(clippy::disallowed_methods)]
            if started.elapsed() > CALL_TIMEOUT {
                let _ = child.kill();
                timed_out = true;
                break None;
            }
            std::thread::sleep(Duration::from_millis(100));
        };
        // 收尸并等读线程排空管道。
        let _ = child.wait();
        let stdout = stdout_reader.join().unwrap_or_default();
        let stderr = stderr_reader.join().unwrap_or_default();
        if timed_out {
            return Err(format!(
                "zcode 调用超时({} s),已终止",
                CALL_TIMEOUT.as_secs()
            ));
        }
        let status = status.expect("非超时路径循环必以进程退出结束");
        if !status.success() {
            let tail: String = stderr
                .lines()
                .last()
                .unwrap_or_default()
                .chars()
                .take(200)
                .collect();
            return Err(format!(
                "zcode 以 {status} 退出{}",
                if tail.is_empty() {
                    String::new()
                } else {
                    format!(";标准错误末行:{tail}")
                }
            ));
        }
        let value: serde_json::Value = serde_json::from_str(stdout.trim()).map_err(|e| {
            format!(
                "zcode 标准输出不是 JSON:{e};原文开头:{:?}",
                stdout.chars().take(200).collect::<String>()
            )
        })?;
        let response = value
            .get("response")
            .and_then(serde_json::Value::as_str)
            .ok_or_else(|| "zcode 输出 JSON 里没有字符串字段 response".to_owned())?;
        if self.progress {
            println!("    zcode 返回({} 字节)", response.len());
        }
        Ok(response.to_owned())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 缺省路径解析(纯函数):`ZCODE_MJS` 优先;`LOCALAPPDATA` 拼出缺省路径;
    /// 文件缺失或两个变量都缺 → 报错。用临时目录伪造文件,不依赖真机安装位置。
    ///
    /// 会变红的实现:环境变量优先级颠倒;缺省路径拼错段;缺失文件不报错。
    #[test]
    fn resolve_script_prefers_env_and_defaults_to_localappdata() {
        let dir = tempfile::tempdir().expect("临时目录");
        let script = dir.path().join("zcode.mjs");
        std::fs::write(&script, "// stub").expect("写桩文件");

        assert_eq!(
            resolve_script(
                Some(script.clone().into_os_string()),
                Some(dir.path().into())
            )
            .expect("ZCODE_MJS 存在"),
            script,
            "ZCODE_MJS 优先"
        );
        assert!(
            resolve_script(
                Some(dir.path().join("missing.mjs").into_os_string()),
                Some(dir.path().into())
            )
            .is_err(),
            "ZCODE_MJS 指向不存在的文件 → 报错"
        );
        let default = dir
            .path()
            .join("zcode")
            .join("runtime")
            .join("current")
            .join("bin")
            .join("zcode.mjs");
        assert!(
            resolve_script(None, Some(dir.path().into())).is_err(),
            "缺省路径下没有 zcode.mjs → 报错"
        );
        std::fs::create_dir_all(default.parent().expect("有父目录")).expect("建目录");
        std::fs::write(&default, "// stub").expect("写桩文件");
        assert_eq!(
            resolve_script(None, Some(dir.path().into())).expect("缺省路径"),
            default
        );
        assert!(resolve_script(None, None).is_err(), "两个变量都缺 → 报错");
    }
}
