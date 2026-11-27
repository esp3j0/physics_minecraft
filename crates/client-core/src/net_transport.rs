//! TCP 网络传输实现(REQ-014 M01 §2.2):实现 [`Transport`] trait,
//! 复用 [`crate::client::run_raw_pump`] 原始帧泵与 3s 心跳。
//!
//! 本类型**不做自动入会**:`Hello`/`Subscribe` 由调用方经 `Transport::send` 发出,
//! 使外层 [`crate::SequencedTransport`] 的序号包装生效。

use std::io;
use std::net::SocketAddr;
use std::sync::mpsc as std_mpsc;

use sim_api::proto::{C2S, S2C};
use tokio::net::TcpStream;

use crate::Transport;
use crate::client::{CONNECT_TIMEOUT, CONTROL_CAPACITY, EVENT_CAPACITY, run_raw_pump};

/// TCP 客户端传输:独立 OS 线程运行 current_thread tokio runtime + 原始收发泵,
/// 上行经有界 std 通道 + 桥线程搬入 tokio 通道,下行经有界 std 通道由主循环 `drain()` 排空。
pub struct NetTransport {
    up_tx: Option<std_mpsc::SyncSender<Vec<C2S>>>,
    down_rx: Option<std_mpsc::Receiver<S2C>>,
    dropped_uplink: u64,
    pump_thread: Option<std::thread::JoinHandle<()>>,
    bridge_thread: Option<std::thread::JoinHandle<()>>,
}

impl NetTransport {
    /// 连接到指定服务端地址并启动收发泵。
    ///
    /// 建连成功后开启 `TCP_NODELAY`,不自动发送 `Hello`/`Subscribe`。
    /// 建连失败或超过 [`CONNECT_TIMEOUT`] 返回 `Err`。
    pub fn connect(addr: SocketAddr) -> io::Result<Self> {
        let (addr_tx, addr_rx) = std_mpsc::sync_channel::<io::Result<()>>(1);
        let (down_tx, down_rx) = std_mpsc::sync_channel::<S2C>(EVENT_CAPACITY);
        let (up_std_tx, up_std_rx) = std_mpsc::sync_channel::<Vec<C2S>>(CONTROL_CAPACITY);
        let (up_tx, up_rx) = tokio::sync::mpsc::channel::<Vec<C2S>>(CONTROL_CAPACITY);

        let pump_thread = std::thread::Builder::new()
            .name("net-transport-pump".to_owned())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("net-transport runtime 构建失败");
                rt.block_on(async move {
                    match TcpStream::connect(addr).await {
                        Ok(stream) => {
                            if let Err(err) = stream.set_nodelay(true) {
                                let _ = addr_tx.send(Err(err));
                                return;
                            }
                            let _ = addr_tx.send(Ok(()));
                            drop(addr_tx);
                            run_raw_pump(stream, up_rx, down_tx).await;
                        }
                        Err(err) => {
                            let _ = addr_tx.send(Err(err));
                        }
                    }
                });
            })
            .expect("net-transport 泵线程 spawn 失败");

        match addr_rx.recv_timeout(CONNECT_TIMEOUT) {
            Ok(Ok(())) => {}
            Ok(Err(err)) => {
                let _ = pump_thread.join();
                return Err(err);
            }
            Err(_) => {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "连接超时"));
            }
        }

        let bridge_thread = std::thread::Builder::new()
            .name("net-transport-bridge".to_owned())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("net-transport 桥 runtime 构建失败");
                rt.block_on(async {
                    while let Ok(msgs) = up_std_rx.recv() {
                        if up_tx.send(msgs).await.is_err() {
                            return;
                        }
                    }
                });
            })
            .expect("net-transport 桥线程 spawn 失败");

        Ok(Self {
            up_tx: Some(up_std_tx),
            down_rx: Some(down_rx),
            dropped_uplink: 0,
            pump_thread: Some(pump_thread),
            bridge_thread: Some(bridge_thread),
        })
    }

    /// 上行通道满导致 `send` 丢弃的累计次数。
    pub fn dropped_uplink(&self) -> u64 {
        self.dropped_uplink
    }
}

impl Transport for NetTransport {
    fn send(&mut self, c2s: C2S) {
        let Some(tx) = self.up_tx.as_ref() else {
            return;
        };
        match tx.try_send(vec![c2s]) {
            Ok(()) => {}
            Err(std_mpsc::TrySendError::Full(_)) => {
                self.dropped_uplink = self.dropped_uplink.saturating_add(1);
            }
            Err(std_mpsc::TrySendError::Disconnected(_)) => {}
        }
    }

    fn drain(&mut self) -> Vec<S2C> {
        let Some(rx) = self.down_rx.as_ref() else {
            return Vec::new();
        };
        let mut out = Vec::new();
        while let Ok(msg) = rx.try_recv() {
            out.push(msg);
        }
        out
    }
}

impl Drop for NetTransport {
    fn drop(&mut self) {
        // 先关上行触发泵写 Goodbye 收口,再释放下行收端防满通道阻塞泵线程。
        drop(self.up_tx.take());
        drop(self.down_rx.take());
        if let Some(pump) = self.pump_thread.take() {
            let _ = pump.join();
        }
        if let Some(bridge) = self.bridge_thread.take() {
            let _ = bridge.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::sync::Arc;
    use std::time::Duration;

    use app::config::Config;
    use app::driver::{
        DriverArgs, DriverCommand, ObserveCadence, ObserveLink, bootstrap, run,
        wire_provider_and_new_sim,
    };
    use app::net::{NetConfig, OutboundHub, serve};
    use messaging::InputSource;
    use sim_api::proto::{C2S, S2C};
    use types::ChunkPos;
    use voxel::SimpleRegistry;

    use super::*;
    use crate::SequencedTransport;

    /// M01 T5(含 R1 裁决纠正):真实 `app::net::serve` + `driver::run`(TempDir)→
    /// `NetTransport::connect` + `SequencedTransport` 包装 → `Hello{99}` + `Subscribe{2}` + `Ping`:
    /// - `Welcome` 恰 2 条、按序 `[(InputSource(1), Tick(0)), (InputSource(1), Tick(t))]` 且 `t >= 1`
    ///   (连接层 `Tick(0)` + 驱动拍号刷新,全部 `player_id == InputSource(1)`);
    /// - `ChunkData` 恰 25(`(2×2+1)²`);
    /// - `Pong >= 1`;
    /// - 服务器关机广播后 `drain` 含 `S2C::Bye`;
    /// - `drop(transport)` 后服务侧连接收口且 `serve`/驱动任务正常退出。
    #[tokio::test]
    async fn m01_t5_net_transport_full_handshake_chunks_ping_shutdown() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let config = Config {
            data_dir: tmp.path().to_path_buf(),
            bodies_dir: tmp.path().join("bodies"),
            policies_dir: tmp.path().join("policies"),
            listen_addr: "127.0.0.1:0".to_owned(),
            ..Config::default()
        };

        let (session, log, store, _resume_from, spawn_next_seq, resume) =
            bootstrap(&config, || Box::new(SimpleRegistry::builtin())).expect("bootstrap 成功");
        let (provider, new_sim, bodies) = wire_provider_and_new_sim(config.clone(), &store, || {
            Box::new(SimpleRegistry::builtin())
        })
        .expect("wire_provider_and_new_sim 成功");

        let net_cfg = NetConfig {
            listen_addr: config.listen_addr.clone(),
            ..NetConfig::default()
        };
        let cadence = ObserveCadence::from(&net_cfg);
        let (up_tx, up_rx) = tokio::sync::mpsc::channel(256);
        let hub = Arc::new(OutboundHub::new());
        let (shutdown_tx, shutdown_rx) = tokio::sync::broadcast::channel::<()>(1);
        let addr = serve(net_cfg, up_tx, shutdown_rx, Arc::clone(&hub))
            .await
            .expect("app::net::serve 监听成功");

        let (cmd_tx, cmd_rx) = tokio::sync::mpsc::channel(config.inbox_capacity);
        let driver = tokio::spawn(run(DriverArgs {
            config,
            session,
            store,
            log,
            commands: cmd_rx,
            listen_ctrl_c: false,
            provider,
            new_sim,
            observe: Some(ObserveLink {
                up: up_rx,
                hub: Arc::clone(&hub),
                cadence,
            }),
            spawn_next_seq,
            bodies,
            resume,
        }));

        let net = NetTransport::connect(addr).expect("NetTransport::connect 成功");
        assert_eq!(net.dropped_uplink(), 0);
        let mut transport = SequencedTransport::new(net);

        transport.send(C2S::Hello { client_id: 99 });
        transport.send(C2S::Subscribe { view_distance: 2 });
        transport.send(C2S::Ping);

        let mut welcomes = Vec::new();
        let mut chunk_positions = BTreeSet::<ChunkPos>::new();
        let mut chunk_data_count = 0usize;
        let mut pong_count = 0usize;

        for _ in 0..500 {
            for msg in transport.drain() {
                match msg {
                    S2C::Welcome { player_id, tick } => welcomes.push((player_id, tick)),
                    S2C::ChunkData { pos, .. } => {
                        chunk_data_count += 1;
                        chunk_positions.insert(pos);
                    }
                    S2C::Pong => pong_count += 1,
                    _ => {}
                }
            }
            if welcomes.len() >= 2 && chunk_data_count >= 25 && pong_count >= 1 {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        // 短暂静置再排空,确认不会多发 Welcome 或 ChunkData。
        for _ in 0..25 {
            tokio::time::sleep(Duration::from_millis(10)).await;
            for msg in transport.drain() {
                match msg {
                    S2C::Welcome { player_id, tick } => welcomes.push((player_id, tick)),
                    S2C::ChunkData { pos, .. } => {
                        chunk_data_count += 1;
                        chunk_positions.insert(pos);
                    }
                    S2C::Pong => pong_count += 1,
                    _ => {}
                }
            }
        }

        assert_eq!(
            chunk_data_count, 25,
            "view_distance=2 下 ChunkData 必须恰为 25((2×2+1)²)"
        );
        assert_eq!(chunk_positions.len(), 25, "25 块坐标必须互异");
        assert!(pong_count >= 1, "Ping 后必须收到至少一条 Pong");

        // 服务器关机广播 → 客户端 drain 必须收到 S2C::Bye。
        let _ = shutdown_tx.send(());
        let mut saw_bye = false;
        for _ in 0..500 {
            for msg in transport.drain() {
                if matches!(msg, S2C::Bye { .. }) {
                    saw_bye = true;
                }
            }
            if saw_bye {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(saw_bye, "服务器关机广播后 drain 必须包含 S2C::Bye");

        // drop transport:释放上行并 join 泵/桥线程。
        drop(transport);

        // 服务侧连接收口(hub 注销 + accept_loop 退出不再接受连接)。
        let mut server_closed = false;
        for _ in 0..500 {
            if hub.subscribers().is_empty() && TcpStream::connect(addr).await.is_err() {
                server_closed = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(
            server_closed,
            "drop transport 与关机广播后服务侧连接及 serve 监听必须收口"
        );

        cmd_tx
            .send(DriverCommand::Shutdown)
            .await
            .expect("驱动关机命令写入");
        tokio::time::timeout(Duration::from_secs(5), driver)
            .await
            .expect("驱动 5s 内退出")
            .expect("驱动不 panic");

        assert_eq!(
            welcomes.len(),
            2,
            "Welcome 必须恰 2 条(连接层 Tick(0) + 驱动拍号刷新),实测: {welcomes:?}"
        );
        assert!(
            welcomes.iter().all(|(p, _)| *p == InputSource(1)),
            "全部 Welcome 的 player_id 必须为 InputSource(1),实测: {welcomes:?}"
        );
        assert_eq!(
            welcomes[0],
            (InputSource(1), types::Tick(0)),
            "第 1 条 Welcome 必须为连接层 (InputSource(1), Tick(0)),实测: {welcomes:?}"
        );
        assert!(
            welcomes[1].1.0 >= 1,
            "第 2 条 Welcome 必须为驱动拍号刷新 (t >= 1),实测: {welcomes:?}"
        );
    }

    /// M01 §2.2:`send` 在上行通道满时丢弃并递增 `dropped_uplink`。
    #[test]
    fn m01_net_transport_dropped_uplink_counts_full() {
        let (up_tx, _up_rx) = std_mpsc::sync_channel::<Vec<C2S>>(1);
        let (_down_tx, down_rx) = std_mpsc::sync_channel::<S2C>(1);
        let mut net = NetTransport {
            up_tx: Some(up_tx),
            down_rx: Some(down_rx),
            dropped_uplink: 0,
            pump_thread: None,
            bridge_thread: None,
        };
        net.send(C2S::Ping);
        assert_eq!(net.dropped_uplink(), 0);
        net.send(C2S::Ping);
        assert_eq!(net.dropped_uplink(), 1);
    }
}
