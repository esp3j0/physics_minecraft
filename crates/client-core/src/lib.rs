//! # client-core —— 客户端纯逻辑核心(REQ-013 §3.1)
//!
//! 零渲染依赖(零 bevy、零 macroquad),严格遵守确定性核 lint。
//! C01a 建立传输边界 [`Transport`] 与进程内单机实现 [`LocalTransport`];
//! C01b-1 建立世界镜像 [`mirror`] 与 subchunk 网格生成 [`mesh`]。

pub mod bodies;
pub mod client;
pub mod entities;
pub mod mesh;
pub mod mirror;
pub mod perf;
pub mod possess;
pub mod script;
pub mod tools;
pub mod transport;
pub mod world;

#[cfg(test)]
mod ac4_tests;

pub use transport::{LoopbackTransport, SequencedTransport};

use std::collections::BTreeSet;
use std::path::Path;

use app::config::Config;
use app::driver::AppError;
use app::local::{LocalConfig, LocalHandle, LocalHost};
use sim_api::proto::{C2S, S2C};

/// 将场景 TOML 的 `[server]` 覆写表与 `data_dir` 转为 `app::config::Config`
/// (与 `xtask::smoke::server_config` 同白名单与 `Config::from_toml_str` 校验口径,用于
/// `client --script` 单机进程内装配)。
pub fn config_from_server_table(server: &toml::Table, data_dir: &Path) -> Result<Config, String> {
    let known: BTreeSet<String> = Config::default()
        .to_toml_string()
        .parse::<toml::Table>()
        .expect("默认配置的 toml 输出必为表")
        .keys()
        .cloned()
        .collect();
    for key in server.keys() {
        if !known.contains(key) {
            return Err(format!("未知的 [server] 覆盖键 `{key}`"));
        }
    }
    let overrides = toml::to_string(server).map_err(|e| format!("覆盖项序列化失败:{e}"))?;
    let text = format!(
        "data_dir = {:?}\n{overrides}",
        data_dir.display().to_string()
    );
    Config::from_toml_str(&text).map_err(|e| e.to_string())
}

/// 传输边界(REQ-013 §3.1):单机 in-process,未来多人换网络实现,
/// 客户端核代码路径不变。
pub trait Transport {
    fn send(&mut self, c2s: C2S);
    fn drain(&mut self) -> Vec<S2C>;
}

/// 单机进程内传输:封装 [`LocalHandle`] 与后台宿主线程。
///
/// Drop 时先发 `Goodbye` 并释放句柄以触发驱动关停,再回收后台线程,
/// 确保 redb 存储 flush 完整收口且不留文件锁。
pub struct LocalTransport {
    handle: Option<LocalHandle>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl LocalTransport {
    /// 启动单机进程内宿主并建立传输句柄。
    pub fn new(cfg: LocalConfig) -> Result<Self, AppError> {
        let (handle, thread) = LocalHost::spawn(cfg)?;
        Ok(Self {
            handle: Some(handle),
            thread: Some(thread),
        })
    }

    /// 由既有 [`LocalHandle`] 构造传输(无后台线程;供 headless / 暂停时钟对账用)。
    pub fn from_handle(handle: LocalHandle) -> Self {
        Self {
            handle: Some(handle),
            thread: None,
        }
    }
}

impl Transport for LocalTransport {
    fn send(&mut self, c2s: C2S) {
        if let Some(handle) = self.handle.as_ref() {
            handle.send(c2s);
        }
    }

    fn drain(&mut self) -> Vec<S2C> {
        self.handle
            .as_ref()
            .map(LocalHandle::drain)
            .unwrap_or_default()
    }
}

impl Drop for LocalTransport {
    fn drop(&mut self) {
        if let Some(handle) = self.handle.take() {
            handle.send(C2S::Goodbye);
            drop(handle);
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use app::config::Config;
    use types::Tick;

    use super::*;

    /// T8 Transport 转发:真实 OS 线程 `LocalTransport::new` + `send(Ping)` →
    /// 有界实时等待(≤ 5 s,每轮 sleep 10 ms)内收到 `Welcome` 与 `Pong`。
    #[test]
    fn t8_local_transport_welcome_and_ping_pong() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let cfg = LocalConfig {
            config: Config {
                data_dir: tmp.path().to_path_buf(),
                bodies_dir: tmp.path().join("bodies"),
                policies_dir: tmp.path().join("policies"),
                ..Config::default()
            },
            view_distance: 2,
        };

        let mut transport = LocalTransport::new(cfg).expect("LocalTransport::new");
        transport.send(C2S::Subscribe { view_distance: 2 });
        transport.send(C2S::Ping);

        let mut got_welcome = false;
        let mut got_pong = false;
        for _ in 0..500 {
            for msg in transport.drain() {
                match msg {
                    S2C::Welcome {
                        player_id,
                        tick: Tick(0),
                    } if player_id.0 == 1 => {
                        got_welcome = true;
                    }
                    S2C::Pong => {
                        got_pong = true;
                    }
                    _ => {}
                }
            }
            if got_welcome && got_pong {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }

        assert!(got_welcome, "5 s 内必须收到 Welcome(player_id=1, tick=0)");
        assert!(got_pong, "5 s 内必须收到 Ping 对应的 Pong");
    }

    /// AC4-t:真实线程宿主自主步进 — 启动后 500 ms 不 drain,再发 MoveObserver + SetBlock,
    /// 有界 ≤ 5 s 轮询 drain 必须收到含 `(0,-60,0) -> STONE` 的 `BlockUpdates`。
    #[test]
    fn t_ac4_host_ticks_without_drain() {
        use messaging::SimCommand;
        use types::BlockPos;
        use types::glam::Vec3;

        let tmp = tempfile::TempDir::new().expect("TempDir");
        let cfg = LocalConfig {
            config: Config {
                data_dir: tmp.path().to_path_buf(),
                bodies_dir: tmp.path().join("bodies"),
                policies_dir: tmp.path().join("policies"),
                ..Config::default()
            },
            view_distance: 2,
        };

        let mut transport = LocalTransport::new(cfg).expect("LocalTransport::new");
        // 不 drain 静置 500 ms:验证宿主在独立线程自主推进。
        std::thread::sleep(Duration::from_millis(500));

        transport.send(C2S::MoveObserver {
            position: Vec3::new(8.0, -60.0, 8.0),
        });
        std::thread::sleep(Duration::from_millis(300));

        let target = BlockPos::new(0, -60, 0);
        transport.send(C2S::Command(SimCommand::SetBlock {
            pos: target,
            block: voxel::builtin::STONE,
        }));

        let mut got_update = false;
        for _ in 0..500 {
            for msg in transport.drain() {
                if let S2C::BlockUpdates { updates } = msg
                    && updates.contains(&(target, voxel::builtin::STONE))
                {
                    got_update = true;
                    break;
                }
            }
            if got_update {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }

        assert!(
            got_update,
            "5 s 内必须收到含 (0,-60,0)->STONE 的 BlockUpdates"
        );
    }
}
