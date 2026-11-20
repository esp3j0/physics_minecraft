//! app::local —— 单机进程内宿主(REQ-013 §3.2a, D6)。
//!
//! 直接复用 [`crate::driver::bootstrap`]、[`crate::driver::run`]、[`crate::net::translate`]
//! 与 [`crate::net::StreamLink`],零 TCP、零序列化,与网络模式共用同一份模拟与校验口径。

use std::sync::{Arc, Mutex, mpsc as std_mpsc};

use messaging::InputSource;
use sim_api::proto::{C2S, S2C};
use tokio::sync::mpsc;
use types::{ChunkPos, Tick};

use crate::config::Config;
use crate::driver::{
    self, AppError, DriverArgs, DriverCommand, DriverOutcome, ObserveCadence, ObserveLink,
};
use crate::net::chunk_stream::{ChunkStreamState, PumpConfig};
use crate::net::{
    DisconnectReason, EnqueueResult, NetUp, Outbound, OutboundHub, OutboundPriority, StreamLink,
    TranslateOutcome, VIEW_DISTANCE_MAX, translate,
};

/// 单机本地玩家固定身份(0 保留给调度器,1 起分配给玩家)。
const LOCAL_PLAYER: InputSource = InputSource(1);
/// 单机观察节奏:每拍推增量(interval_ticks=1),全量周期沿用 60(REQ-013 §3.2)。
const LOCAL_CADENCE: ObserveCadence = ObserveCadence {
    interval_ticks: 1,
    full_interval_ticks: 60,
};
/// 单机 chunk 流泵节奏:相对网络默认 100 ms 放宽为 25 ms(REQ-013 §3.2)。
const LOCAL_PUMP: PumpConfig = PumpConfig {
    interval_ms: 25,
    max_catchup_rounds: 4,
};
/// 上行事件通道容量(与 `net::assemble_network` 同口径)。
const UP_CAPACITY: usize = 256;

/// 单机宿主装配配置。
pub struct LocalConfig {
    /// 驱动配置(`data_dir` 由调用方显式指定;测试须用 `TempDir`,不碰仓库根 `data/`)。
    pub config: Config,
    /// 订阅视距(chunk;钳制 ≤ [`VIEW_DISTANCE_MAX`])。
    pub view_distance: u8,
}

/// 单机进程内宿主入口。
pub struct LocalHost;

impl LocalHost {
    /// 起独立 OS 线程 + `current_thread` tokio runtime(与 `viewer::client`
    /// 同款隔离模型),在其中运行 `bootstrap + driver::run`。
    pub fn spawn(cfg: LocalConfig) -> Result<(LocalHandle, std::thread::JoinHandle<()>), AppError> {
        let (ready_tx, ready_rx) = std_mpsc::sync_channel::<Result<LocalHandle, AppError>>(1);
        let thread = std::thread::Builder::new()
            .name("app-local-host".to_owned())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("local host tokio runtime 构建失败");
                rt.block_on(async move {
                    match spawn_in_current_runtime(cfg).await {
                        Ok((handle, driver_task)) => {
                            let _ = ready_tx.send(Ok(handle));
                            drop(ready_tx);
                            let _ = driver_task.await;
                        }
                        Err(err) => {
                            let _ = ready_tx.send(Err(err));
                        }
                    }
                });
            })
            .expect("app-local-host 线程创建失败");

        match ready_rx
            .recv()
            .expect("app-local-host 线程启动回执前意外退出")
        {
            Ok(handle) => Ok((handle, thread)),
            Err(err) => {
                let _ = thread.join();
                Err(err)
            }
        }
    }
}

/// 在当前 tokio runtime 内完成宿主装配并 `tokio::spawn` 驱动任务。
///
/// 生产路径([`LocalHost::spawn`])与暂停时钟确定性测试共享同一套装配逻辑,
/// 零代码分叉。
pub(crate) async fn spawn_in_current_runtime(
    cfg: LocalConfig,
) -> Result<(LocalHandle, tokio::task::JoinHandle<DriverOutcome>), AppError> {
    let view_distance = cfg.view_distance.min(VIEW_DISTANCE_MAX);
    let (session, log, store, _resume_from, spawn_next_seq, resume) =
        driver::bootstrap(&cfg.config, || Box::new(voxel::SimpleRegistry::builtin()))?;
    let (provider, new_sim, bodies) =
        driver::wire_provider_and_new_sim(cfg.config.clone(), &store, || {
            Box::new(voxel::SimpleRegistry::builtin())
        })?;

    let (cmd_tx, cmd_rx) = mpsc::channel::<DriverCommand>(cfg.config.inbox_capacity);
    let (up_tx, up_rx) = mpsc::channel::<NetUp>(UP_CAPACITY);
    let hub = Arc::new(OutboundHub::new());
    let mut outbound = Outbound::new();

    // 启动即完成 Hello+Subscribe 等效装配(REQ-013 §3.2a;中心与 net.rs:980 同为 (0, 0))。
    let _ = outbound.push(
        OutboundPriority::Critical,
        S2C::Welcome {
            player_id: LOCAL_PLAYER,
            tick: Tick(0),
        },
    );
    let stream_state = Arc::new(Mutex::new(ChunkStreamState::new(
        ChunkPos::new(0, 0),
        view_distance,
    )));
    let stream_link = StreamLink::spawn(
        stream_state.clone(),
        LOCAL_PUMP,
        up_tx.clone(),
        outbound.world_writer(),
        LOCAL_PLAYER,
    );
    hub.register(LOCAL_PLAYER, outbound.world_writer());
    hub.register_center(LOCAL_PLAYER, stream_state);

    let _ = up_tx.try_send(NetUp::Joined {
        player: LOCAL_PLAYER,
    });
    let _ = up_tx.try_send(NetUp::Subscribed {
        player: LOCAL_PLAYER,
        view_distance,
    });

    let driver_task = tokio::spawn(driver::run(DriverArgs {
        config: cfg.config,
        session,
        store,
        log,
        commands: cmd_rx,
        listen_ctrl_c: false,
        provider,
        new_sim,
        observe: Some(ObserveLink {
            up: up_rx,
            hub: hub.clone(),
            cadence: LOCAL_CADENCE,
        }),
        spawn_next_seq,
        bodies,
        resume,
    }));

    let handle = LocalHandle {
        up_tx,
        hub,
        outbound: Mutex::new(outbound),
        state: Mutex::new(LocalSessionState {
            connected: true,
            stream_link: Some(stream_link),
            driver_cmd_tx: Some(cmd_tx),
        }),
    };

    Ok((handle, driver_task))
}

struct LocalSessionState {
    connected: bool,
    stream_link: Option<StreamLink>,
    driver_cmd_tx: Option<mpsc::Sender<DriverCommand>>,
}

/// 本地玩家通信句柄:进程内发 C2S、非阻塞排空 S2C。
pub struct LocalHandle {
    up_tx: mpsc::Sender<NetUp>,
    hub: Arc<OutboundHub>,
    outbound: Mutex<Outbound>,
    state: Mutex<LocalSessionState>,
}

impl LocalHandle {
    /// 会话门 + `net::translate` + 上行;协议违规 → 本地断连(后续 `send` 无效)。
    pub fn send(&self, c2s: C2S) {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        if !state.connected {
            return;
        }

        match translate(LOCAL_PLAYER, c2s) {
            TranslateOutcome::Hello => {}
            TranslateOutcome::Subscribe(view_distance) => {
                let vd = view_distance.min(VIEW_DISTANCE_MAX);
                if let Some(link) = state.stream_link.as_ref() {
                    link.state
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .resubscribe(vd);
                }
                let _ = self.up_tx.try_send(NetUp::Subscribed {
                    player: LOCAL_PLAYER,
                    view_distance: vd,
                });
            }
            TranslateOutcome::Move(cmd) => {
                if let (Some(link), DriverCommand::MoveObserver { position }) =
                    (state.stream_link.as_ref(), &cmd)
                {
                    let center = ChunkPos::new(
                        (position.x / 16.0).floor() as i32,
                        (position.z / 16.0).floor() as i32,
                    );
                    link.state
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .set_center(center);
                }
                let _ = self.up_tx.try_send(NetUp::DriverCommand {
                    player: LOCAL_PLAYER,
                    cmd,
                });
            }
            TranslateOutcome::Send(cmd) | TranslateOutcome::Agent(cmd) => {
                let _ = self.up_tx.try_send(NetUp::DriverCommand {
                    player: LOCAL_PLAYER,
                    cmd,
                });
            }
            TranslateOutcome::SubscribeDebug => {
                let _ = self.up_tx.try_send(NetUp::SubscribeDebug {
                    player: LOCAL_PLAYER,
                });
            }
            TranslateOutcome::Ping => {
                let res = self
                    .outbound
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(OutboundPriority::Critical, S2C::Pong);
                if res == EnqueueResult::CriticalFull {
                    self.disconnect_locked(&mut state, DisconnectReason::CriticalOverflow, false);
                }
            }
            TranslateOutcome::Goodbye => {
                self.disconnect_locked(&mut state, DisconnectReason::Goodbye, true);
            }
            TranslateOutcome::Reject(why) => {
                self.disconnect_locked(&mut state, DisconnectReason::ProtocolViolation(why), false);
            }
        }
    }

    /// 非阻塞排空出站:Critical 先于 World,各自 FIFO(复用 `Outbound::try_pop`)。
    pub fn drain(&self) -> Vec<S2C> {
        let mut outbound = self.outbound.lock().unwrap_or_else(|p| p.into_inner());
        let mut out = Vec::new();
        while let Some(msg) = outbound.try_pop() {
            out.push(msg);
        }
        out
    }

    /// 本地断连收口:停泵 + 注销 hub + 尽力推 `Bye` + 上报 `NetUp::Left`;
    /// `shutdown_driver` 为 true 时同步触发驱动关机。
    fn disconnect_locked(
        &self,
        state: &mut LocalSessionState,
        reason: DisconnectReason,
        shutdown_driver: bool,
    ) {
        if state.connected {
            state.connected = false;
            state.stream_link = None;
            let _ = self.hub.unregister(&LOCAL_PLAYER);
            if reason != DisconnectReason::CriticalOverflow
                && reason != DisconnectReason::SocketClosed
            {
                let _ = self
                    .outbound
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(
                        OutboundPriority::Critical,
                        S2C::Bye {
                            reason: bye_text(&reason),
                        },
                    );
            }
            let _ = self.up_tx.try_send(NetUp::Left {
                player: LOCAL_PLAYER,
                reason,
            });
        }
        if shutdown_driver && let Some(tx) = state.driver_cmd_tx.take() {
            let _ = tx.try_send(DriverCommand::Shutdown);
        }
    }
}

impl Drop for LocalHandle {
    fn drop(&mut self) {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        self.disconnect_locked(&mut state, DisconnectReason::Goodbye, true);
    }
}

fn bye_text(reason: &DisconnectReason) -> String {
    match reason {
        DisconnectReason::Goodbye => "goodbye".to_string(),
        DisconnectReason::Timeout => "read timeout".to_string(),
        DisconnectReason::ProtocolViolation(what) => format!("protocol violation: {what}"),
        DisconnectReason::CriticalOverflow => "critical queue overflow".to_string(),
        DisconnectReason::SocketClosed => "socket closed".to_string(),
        DisconnectReason::Shutdown => "server shutdown".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::path::Path;
    use std::time::Duration;

    use messaging::SimCommand;
    use types::glam::Vec3;
    use types::{BlockPos, EntityDesc, EntityId, EntityState};

    use super::*;
    use crate::net::{chebyshev, view_square};

    fn local_test_config(dir: &Path, view_distance: u8) -> LocalConfig {
        let config = Config {
            data_dir: dir.to_path_buf(),
            bodies_dir: dir.join("bodies"),
            policies_dir: dir.join("policies"),
            ..Config::default()
        };
        LocalConfig {
            config,
            view_distance,
        }
    }

    async fn advance_one_tick() {
        tokio::time::advance(Duration::from_secs_f32(1.0 / 60.0)).await;
        tokio::task::yield_now().await;
    }

    async fn advance_ticks(ticks: u32) {
        for _ in 0..ticks {
            advance_one_tick().await;
        }
    }

    /// T3 启动与 chunk 流:
    /// - 首次 drain(在首泵已推 ChunkData 后)首帧必为 `Welcome { player_id: InputSource(1), tick: 0 }`(Critical 先于 World);
    /// - 上限 200 轮 × advance 100 ms 内收齐 25 个 ChunkData,pos 集合 = view_square((0,0), 2);
    /// - 前 16 个 ChunkData 的 pos 按切比雪夫 center-out 升序。
    #[tokio::test(start_paused = true)]
    async fn t3_startup_welcome_and_chunk_stream() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let (handle, driver) = spawn_in_current_runtime(local_test_config(tmp.path(), 2))
            .await
            .expect("spawn_in_current_runtime");

        let center = ChunkPos::new(0, 0);
        let expected_set = view_square(center, 2);
        assert_eq!(expected_set.len(), 25, "手算:(2×2+1)² = 25");

        let mut received_order: Vec<ChunkPos> = Vec::new();
        let mut received_set: BTreeSet<ChunkPos> = BTreeSet::new();
        let mut saw_welcome_first = false;

        // 先推进 1 轮 100 ms 让首泵 ChunkData 进入 World 队列,再执行首次 drain:
        // Critical 队列的 Welcome(tick=0) 必须在所有 World 队列的 ChunkData 之前。
        for round in 0..200 {
            tokio::time::advance(Duration::from_millis(100)).await;
            tokio::task::yield_now().await;
            let batch = handle.drain();
            if round == 0 {
                assert!(
                    !batch.is_empty(),
                    "首轮推进后 drain 不得为空(至少含 Welcome)"
                );
                assert_eq!(
                    batch[0],
                    S2C::Welcome {
                        player_id: InputSource(1),
                        tick: Tick(0),
                    },
                    "Critical 优先级:Welcome(tick=0) 必须在首帧(先于 World 队列 ChunkData)"
                );
                saw_welcome_first = true;
            }
            for msg in batch {
                if let S2C::ChunkData { pos, .. } = msg
                    && received_set.insert(pos)
                {
                    received_order.push(pos);
                }
            }
            if received_set.len() >= 25 {
                break;
            }
        }

        assert!(saw_welcome_first, "必须收到 Welcome(tick=0)");
        assert_eq!(
            received_order.len(),
            25,
            "视距 2 必须收齐 25 个不重复 ChunkData"
        );
        assert_eq!(
            received_set, expected_set,
            "pos 集合 = view_square((0,0), 2)"
        );
        assert_eq!(received_order[0], center, "首帧 ChunkData 必为中心 (0,0)");
        let first_16_keys: Vec<(i32, i32, i32)> = received_order[..16]
            .iter()
            .map(|p| (chebyshev(center, *p), p.x, p.z))
            .collect();
        let mut sorted_keys = first_16_keys.clone();
        sorted_keys.sort();
        assert_eq!(
            first_16_keys, sorted_keys,
            "前 16 个 ChunkData 必须按切比雪夫 center-out 升序"
        );

        handle.send(C2S::Goodbye);
        advance_one_tick().await;
        driver.await.expect("驱动正常退出");
    }

    /// T4 方块编辑闭环:先 MoveObserver(8,-60,8) + advance 10 拍(分片注册 + chunk(0,0) 装载),
    /// 再 SetBlock(0,-60,0, STONE)→ cadence=1 下 ≤ 4 拍内收到含该格的 BlockUpdates。
    #[tokio::test(start_paused = true)]
    async fn t4_block_edit_roundtrip() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let (handle, driver) = spawn_in_current_runtime(local_test_config(tmp.path(), 2))
            .await
            .expect("spawn_in_current_runtime");

        handle.send(C2S::MoveObserver {
            position: Vec3::new(8.0, -60.0, 8.0),
        });
        // 10 拍预热:完成分片注册与 chunk(0,0) 激活装载(同时对齐 10 拍边界便于检验 cadence=1)。
        advance_ticks(10).await;
        let _ = handle.drain();

        let target = BlockPos::new(0, -60, 0);
        handle.send(C2S::Command(SimCommand::SetBlock {
            pos: target,
            block: voxel::builtin::STONE,
        }));

        let mut got_at_tick: Option<u32> = None;
        for step in 1..=200 {
            advance_one_tick().await;
            for msg in handle.drain() {
                if let S2C::BlockUpdates { updates } = msg
                    && updates.contains(&(target, voxel::builtin::STONE))
                {
                    got_at_tick = Some(step);
                    break;
                }
            }
            if got_at_tick.is_some() {
                break;
            }
        }

        let waited = got_at_tick.expect("上限 200 轮内必须收到含 (0,-60,0)=STONE 的 BlockUpdates");
        assert!(
            waited <= 4,
            "cadence=1 下编辑应在数拍内(≤4 拍)到达,实测 {waited} 拍"
        );

        handle.send(C2S::Goodbye);
        advance_one_tick().await;
        driver.await.expect("驱动正常退出");
    }

    /// T5 白名单不绕:发非白名单 `SpawnDynamic` → translate Reject → 本地断连;
    /// 此后 `send(Ping)` → `drain` 无 `Pong`。
    #[tokio::test(start_paused = true)]
    async fn t5_whitelist_rejects_and_disconnects() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let (handle, driver) = spawn_in_current_runtime(local_test_config(tmp.path(), 2))
            .await
            .expect("spawn_in_current_runtime");

        let _ = handle.drain();
        // 对照:断连前 Ping 有 Pong。
        handle.send(C2S::Ping);
        let pre = handle.drain();
        assert!(pre.contains(&S2C::Pong), "断连前 Ping 必须返回 Pong");

        // 发非白名单命令 SpawnDynamic → 协议违规本地断连。
        handle.send(C2S::Command(SimCommand::SpawnDynamic {
            entity: EntityId::from_parts(1, 1),
            state: EntityState::default(),
            desc: EntityDesc::default(),
        }));
        let _ = handle.drain();

        // 断连后 send(Ping) 无效,drain 不得再收到 Pong。
        handle.send(C2S::Ping);
        advance_ticks(3).await;
        let post = handle.drain();
        assert!(
            !post.contains(&S2C::Pong),
            "Reject 断连后 send(Ping) 不得收到 Pong,实际:{post:?}"
        );

        drop(handle);
        advance_one_tick().await;
        driver.await.expect("驱动正常退出");
    }

    /// T6 逐位确定:同配置同脚本跑两次(各自 TempDir),完整 `Vec<S2C>`(含 tick)逐位相等。
    #[tokio::test(start_paused = true)]
    async fn t6_deterministic_s2c_frames() {
        async fn run_script(dir: &Path) -> Vec<S2C> {
            let (handle, driver) = spawn_in_current_runtime(local_test_config(dir, 2))
                .await
                .expect("spawn_in_current_runtime");
            let mut frames = Vec::new();
            frames.extend(handle.drain());

            handle.send(C2S::MoveObserver {
                position: Vec3::new(8.0, -60.0, 8.0),
            });
            for _ in 0..10 {
                advance_one_tick().await;
                frames.extend(handle.drain());
            }

            handle.send(C2S::Command(SimCommand::SetBlock {
                pos: BlockPos::new(0, -60, 0),
                block: voxel::builtin::STONE,
            }));
            for _ in 0..4 {
                advance_one_tick().await;
                frames.extend(handle.drain());
            }

            handle.send(C2S::Goodbye);
            advance_one_tick().await;
            driver.await.expect("驱动正常退出");
            frames.extend(handle.drain());
            frames
        }

        let dir_a = tempfile::TempDir::new().expect("TempDir A");
        let dir_b = tempfile::TempDir::new().expect("TempDir B");
        let frames_a = run_script(dir_a.path()).await;
        let frames_b = run_script(dir_b.path()).await;

        assert!(
            frames_a.contains(&S2C::Welcome {
                player_id: InputSource(1),
                tick: Tick(0),
            }),
            "序列必须含初始 Welcome"
        );
        let target = (BlockPos::new(0, -60, 0), voxel::builtin::STONE);
        assert!(
            frames_a
                .iter()
                .any(|m| matches!(m, S2C::BlockUpdates { updates } if updates.contains(&target))),
            "固定 4 拍推进内必须收到编辑后的 BlockUpdates(cadence=1)"
        );
        assert_eq!(
            frames_a, frames_b,
            "同配置同脚本两次运行 S2C 帧序列必须逐位相等"
        );
    }

    /// T7 Sequenced 透明转发:发 `Sequenced { seq: 1, inner: Command(SetBlock 同 T4) }`,
    /// 同样收到含 `(0,-60,0)=STONE` 的 `BlockUpdates`。
    #[tokio::test(start_paused = true)]
    async fn t7_sequenced_transparent_forward() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let (handle, driver) = spawn_in_current_runtime(local_test_config(tmp.path(), 2))
            .await
            .expect("spawn_in_current_runtime");

        handle.send(C2S::MoveObserver {
            position: Vec3::new(8.0, -60.0, 8.0),
        });
        advance_ticks(10).await;
        let _ = handle.drain();

        let target = BlockPos::new(0, -60, 0);
        handle.send(C2S::Sequenced {
            seq: 1,
            inner: Box::new(C2S::Command(SimCommand::SetBlock {
                pos: target,
                block: voxel::builtin::STONE,
            })),
        });

        let mut got = false;
        for _ in 0..200 {
            advance_one_tick().await;
            for msg in handle.drain() {
                if let S2C::BlockUpdates { updates } = msg
                    && updates.contains(&(target, voxel::builtin::STONE))
                {
                    got = true;
                    break;
                }
            }
            if got {
                break;
            }
        }

        assert!(
            got,
            "Sequenced 包装的 SetBlock 必须透明转发并产出 BlockUpdates"
        );

        handle.send(C2S::Goodbye);
        advance_one_tick().await;
        driver.await.expect("驱动正常退出");
    }

    /// T9 两队列均非空时的排序锚点(C01a-R1):
    /// 6 轮 advance(100ms)+yield 不 drain 使 Critical 含 2 个 Welcome、World 含 25 个 ChunkData,
    /// 再发 Ping 使 Pong 入 Critical 队尾,单次 drain 必按 Critical FIFO 在前、World 25 块在后。
    #[tokio::test(start_paused = true)]
    async fn t9_critical_before_world_under_contention() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let (handle, driver) = spawn_in_current_runtime(local_test_config(tmp.path(), 2))
            .await
            .expect("spawn_in_current_runtime");

        for _ in 0..6 {
            tokio::time::advance(Duration::from_millis(100)).await;
            tokio::task::yield_now().await;
        }
        handle.send(C2S::Ping);

        let batch = handle.drain();
        assert_eq!(
            batch.len(),
            28,
            "手算:2 Welcome + 1 Pong + 25 ChunkData = 28"
        );
        assert_eq!(
            batch[0],
            S2C::Welcome {
                player_id: InputSource(1),
                tick: Tick(0),
            },
            "batch[0] 必为初始 Welcome(tick=0)"
        );
        assert_eq!(
            batch[1],
            S2C::Welcome {
                player_id: InputSource(1),
                tick: Tick(1),
            },
            "batch[1] 必为驱动 Joined 回执 Welcome(tick=1)"
        );
        assert_eq!(batch[2], S2C::Pong, "batch[2] 必为 Critical 队尾的 Pong");

        let mut chunk_positions: BTreeSet<ChunkPos> = BTreeSet::new();
        for msg in &batch[3..] {
            match msg {
                S2C::ChunkData { pos, .. } => {
                    assert!(
                        chunk_positions.insert(*pos),
                        "ChunkData pos 不得重复:{pos:?}"
                    );
                }
                other => panic!("batch[3..] 必须全为 ChunkData,实际遇到:{other:?}"),
            }
        }
        assert_eq!(
            chunk_positions,
            view_square(ChunkPos::new(0, 0), 2),
            "batch[3..] pos 集合必为 view_square((0,0), 2)(25 个不重复)"
        );

        drop(handle);
        driver.await.expect("驱动正常退出");
    }
}
