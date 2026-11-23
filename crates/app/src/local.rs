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
/// headless / 暂停时钟入口(供确定性测试与双传输对账使用);生产路径([`LocalHost::spawn`])
/// 与暂停时钟测试共享同一套装配逻辑,零代码分叉。
pub async fn spawn_in_current_runtime(
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

        let (seqs, outcome) = crate::net::unwrap_sequenced(translate(LOCAL_PLAYER, c2s));
        if !matches!(outcome, TranslateOutcome::Reject(_)) {
            for seq in seqs {
                let _ = self.up_tx.try_send(NetUp::Ack {
                    player: LOCAL_PLAYER,
                    seq,
                });
            }
        }

        match outcome {
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
            TranslateOutcome::Sequenced { .. } => {
                unreachable!("unwrap_sequenced 已剥尽 Sequenced 层")
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

    // ---- W1 单测(REQ-013 §3.2a, C03-2:T-A2 ~ T-A6) ----------------------------

    const FIXTURE_ZERO_ONNX: &[u8] =
        include_bytes!("../../../assets/policies/test_fixtures/zero45x12.onnx");

    fn repo_assets_dir(sub: &str) -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets")
            .join(sub)
    }

    fn quad_hash_hex_from_assets() -> String {
        let mut bodies = physics_sim::BodyRegistry::new();
        bodies
            .insert_toml(include_str!("../../../assets/bodies/quad.body.toml"))
            .expect("quad.body.toml 合法");
        bodies.hash_of("quad").expect("quad 在注册表").hex()
    }

    fn terms45_manifest(name: &str, hash_hex: &str) -> String {
        format!(
            "name = \"{name}\"\n\
             body_hash = \"{hash_hex}\"\n\
             control_period_ticks = 2\n\
             action_scale = 0.25\n\
             [[obs]]\nterm = \"base_lin_vel\"\n\
             [[obs]]\nterm = \"base_ang_vel\"\n\
             [[obs]]\nterm = \"projected_gravity\"\n\
             [[obs]]\nterm = \"joint_pos\"\n\
             [[obs]]\nterm = \"joint_vel\"\n\
             [[obs]]\nterm = \"last_action\"\n"
        )
    }

    fn write_fixture_pack(dir: &Path, name: &str, hash_hex: &str) {
        let pack_dir = dir.join(name);
        std::fs::create_dir_all(&pack_dir).expect("创建夹具策略包目录");
        std::fs::write(pack_dir.join("policy.onnx"), FIXTURE_ZERO_ONNX).expect("写 policy.onnx");
        std::fs::write(
            pack_dir.join("policy.toml"),
            terms45_manifest(name, hash_hex),
        )
        .expect("写 policy.toml");
    }

    fn local_policy_config(
        data_dir: &Path,
        policies_dir: &Path,
        default_policy: Option<&str>,
    ) -> LocalConfig {
        let config = Config {
            worker_count: 2,
            world_seed: types::WorldSeed(3),
            data_dir: data_dir.to_path_buf(),
            bodies_dir: repo_assets_dir("bodies"),
            policies_dir: policies_dir.to_path_buf(),
            default_policy: default_policy.map(str::to_owned),
            policy_wait_budget_us: 1_000_000,
            ..Config::default()
        };
        LocalConfig {
            config,
            view_distance: 2,
        }
    }

    /// 精确推进 1 个逻辑模拟拍(tokio 定时器轮为 1 ms 粒度,dt = 16.667 ms 向上取整 17 ms,
    /// 推进 17 ms 恰好触发 1 次 `MissedTickBehavior::Skip` interval,避免 16.667 ms 隔拍漏触)。
    async fn advance_sim_tick() {
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
    }

    /// 等到 chunk(0,0) 的 ChunkData 到达(先 MoveObserver(8,-60,8),上限 200 拍)。
    async fn wait_for_chunk_0_0(handle: &LocalHandle) {
        handle.send(C2S::MoveObserver {
            position: Vec3::new(8.0, -60.0, 8.0),
        });
        let mut saw_chunk = false;
        for _ in 0..200 {
            advance_sim_tick().await;
            for msg in handle.drain() {
                if matches!(msg, S2C::ChunkData { pos, .. } if pos == ChunkPos::new(0, 0)) {
                    saw_chunk = true;
                }
            }
            if saw_chunk {
                break;
            }
        }
        assert!(saw_chunk, "200 拍内必须收到 chunk(0,0) 的 ChunkData");
        // 再推进 10 拍确保分片激活/chunk 装载就绪。
        for _ in 0..10 {
            advance_sim_tick().await;
        }
        let _ = handle.drain();
    }

    /// 发 SpawnAgent 并等待 Spawned 回执(上限 20 拍)。
    async fn spawn_agent_and_wait(handle: &LocalHandle, body: &str, position: Vec3) -> EntityId {
        handle.send(C2S::SpawnAgent {
            body: body.to_owned(),
            position,
            yaw: 0.0,
        });
        for _ in 0..20 {
            advance_sim_tick().await;
            for msg in handle.drain() {
                if let S2C::Spawned { entity } = msg {
                    return entity;
                }
            }
        }
        panic!("20 拍内未收到 SpawnAgent({body}) 的 Spawned 回执");
    }

    /// T-A2 attach-on-spawn:local 宿主 + default_policy=夹具名 → SpawnAgent quad 推 300 拍 →
    /// policy_inferences >= 100、policy_errors == 0、send_rejected == 0;对照 None → == 0。
    #[tokio::test(start_paused = true)]
    async fn t_a2_attach_on_spawn_and_control_none() {
        let policies_dir = tempfile::TempDir::new().expect("policies TempDir");
        write_fixture_pack(
            policies_dir.path(),
            "stand_fix",
            &quad_hash_hex_from_assets(),
        );

        // 实验组:default_policy = Some("stand_fix")。
        let data_on = tempfile::TempDir::new().expect("data_on TempDir");
        let (handle, driver) = spawn_in_current_runtime(local_policy_config(
            data_on.path(),
            policies_dir.path(),
            Some("stand_fix"),
        ))
        .await
        .expect("spawn_in_current_runtime");
        wait_for_chunk_0_0(&handle).await;
        let _entity = spawn_agent_and_wait(&handle, "quad", Vec3::new(8.5, -59.5, 8.5)).await;
        for _ in 0..300 {
            advance_sim_tick().await;
            let _ = handle.drain();
        }
        handle.send(C2S::Goodbye);
        let outcome = driver.await.expect("驱动正常退出");
        let c = outcome.controller_counters;
        assert!(
            c.policy_inferences >= 100,
            "300 拍理论 ~150 次推理,期望 >= 100,实测 {}",
            c.policy_inferences
        );
        assert_eq!(c.policy_errors, 0, "policy_errors 必须为 0");
        assert_eq!(c.send_rejected, 0, "send_rejected 必须为 0");

        // 对照组:default_policy = None → policy_inferences == 0。
        let data_off = tempfile::TempDir::new().expect("data_off TempDir");
        let (handle2, driver2) = spawn_in_current_runtime(local_policy_config(
            data_off.path(),
            policies_dir.path(),
            None,
        ))
        .await
        .expect("spawn_in_current_runtime(None)");
        wait_for_chunk_0_0(&handle2).await;
        let _entity2 = spawn_agent_and_wait(&handle2, "quad", Vec3::new(8.5, -59.5, 8.5)).await;
        for _ in 0..300 {
            advance_sim_tick().await;
            let _ = handle2.drain();
        }
        handle2.send(C2S::Goodbye);
        let outcome2 = driver2.await.expect("对照组驱动正常退出");
        assert_eq!(
            outcome2.controller_counters.policy_inferences, 0,
            "default_policy = None 对照组推理计数必须为 0"
        );
    }

    /// T-A3 哈希门:default_policy 指向 quad 夹具包,但 SpawnAgent capsule →
    /// Spawned 照收、policy_inferences == 0、不 panic。
    #[tokio::test(start_paused = true)]
    async fn t_a3_hash_gate_skips_capsule() {
        let policies_dir = tempfile::TempDir::new().expect("policies TempDir");
        write_fixture_pack(
            policies_dir.path(),
            "stand_fix",
            &quad_hash_hex_from_assets(),
        );
        let data = tempfile::TempDir::new().expect("data TempDir");

        let (handle, driver) = spawn_in_current_runtime(local_policy_config(
            data.path(),
            policies_dir.path(),
            Some("stand_fix"),
        ))
        .await
        .expect("spawn_in_current_runtime");
        wait_for_chunk_0_0(&handle).await;
        let _entity = spawn_agent_and_wait(&handle, "capsule", Vec3::new(8.5, -59.0, 8.5)).await;
        for _ in 0..120 {
            advance_sim_tick().await;
            let _ = handle.drain();
        }
        handle.send(C2S::Goodbye);
        let outcome = driver.await.expect("驱动正常退出");
        assert_eq!(
            outcome.controller_counters.policy_inferences, 0,
            "capsule 与 quad 策略哈希不匹配,不挂策略 → policy_inferences == 0"
        );
        assert_eq!(outcome.controller_counters.policy_errors, 0);
        assert_eq!(outcome.controller_counters.send_rejected, 0);
        assert_eq!(
            outcome.controller_counters.policy_no_state, 0,
            "哈希门失守时 capsule 会被挂上策略槽,每到期拍 policy_no_state 递增"
        );
    }

    /// T-A4 附身闭环:attach-on-spawn(quad_gpu1)+ Possess → Possessed(Some);
    /// AgentIntent move(0.5, 0) 推 120 拍 → 根 +x 位移 > 0;
    /// Unpossess → Possessed(None);全程 policy_inferences > 0 且 policy_errors == 0。
    #[tokio::test(start_paused = true)]
    async fn t_a4_possess_intent_unpossess_loop() {
        let data = tempfile::TempDir::new().expect("data TempDir");
        let policies_dir = repo_assets_dir("policies");
        let (handle, driver) = spawn_in_current_runtime(local_policy_config(
            data.path(),
            &policies_dir,
            Some("quad_gpu1"),
        ))
        .await
        .expect("spawn_in_current_runtime");
        wait_for_chunk_0_0(&handle).await;
        let entity = spawn_agent_and_wait(&handle, "quad", Vec3::new(8.5, -59.5, 8.5)).await;

        let mut last_x: Option<f32> = None;
        let drain_states = |h: &LocalHandle, lx: &mut Option<f32>| -> Vec<S2C> {
            let msgs = h.drain();
            for m in &msgs {
                if let S2C::AgentStates { states, .. } = m {
                    for (id, _, links) in states {
                        if *id == entity
                            && let Some(root) = links.first()
                        {
                            *lx = Some(root.translation.x);
                        }
                    }
                }
            }
            msgs
        };

        // 先站稳 120 拍。
        for _ in 0..120 {
            advance_sim_tick().await;
            let _ = drain_states(&handle, &mut last_x);
        }

        // Possess 自己的策略身体 → 收到 Possessed(Some(entity))。
        handle.send(C2S::Possess { entity });
        let mut saw_possessed = false;
        for _ in 0..20 {
            advance_sim_tick().await;
            for msg in drain_states(&handle, &mut last_x) {
                if msg
                    == (S2C::Possessed {
                        entity: Some(entity),
                    })
                {
                    saw_possessed = true;
                }
            }
            if saw_possessed {
                break;
            }
        }
        assert!(saw_possessed, "Possess 后必须收到 Possessed(Some(entity))");

        let start_x = last_x.expect("附身前必已采样到根 x");
        handle.send(C2S::AgentIntent {
            entity,
            move_xz: (0.5, 0.0),
            yaw: 0.0,
            jump: false,
        });
        for _ in 0..120 {
            advance_sim_tick().await;
            let _ = drain_states(&handle, &mut last_x);
        }
        let end_x = last_x.expect("行走后必有根 x");
        let dx = end_x - start_x;
        assert!(
            dx > 0.0,
            "附身 + AgentIntent(0.5, 0) 120 拍后根 +x 位移须 > 0,实测 start={start_x:.4} end={end_x:.4} dx={dx:.4}"
        );

        // Unpossess → 收到 Possessed(None)。
        handle.send(C2S::Unpossess);
        let mut saw_unpossessed = false;
        for _ in 0..20 {
            advance_sim_tick().await;
            for msg in drain_states(&handle, &mut last_x) {
                if msg == (S2C::Possessed { entity: None }) {
                    saw_unpossessed = true;
                }
            }
            if saw_unpossessed {
                break;
            }
        }
        assert!(saw_unpossessed, "Unpossess 后必须收到 Possessed(None)");

        handle.send(C2S::Goodbye);
        let outcome = driver.await.expect("驱动正常退出");
        assert!(
            outcome.controller_counters.policy_inferences > 0,
            "全程 policy_inferences 须 > 0"
        );
        assert_eq!(outcome.controller_counters.policy_errors, 0);
        assert_eq!(outcome.controller_counters.send_rejected, 0);
    }

    /// T-A5 恢复:同 data_dir 二次 `spawn_in_current_runtime`(检查点里有策略槽位)→
    /// 不 fail-stop、继续推理(再推进 120 拍 inferences 增量 > 0)。
    #[tokio::test(start_paused = true)]
    async fn t_a5_recovery_keeps_policy_running() {
        let policies_dir = tempfile::TempDir::new().expect("policies TempDir");
        write_fixture_pack(
            policies_dir.path(),
            "stand_fix",
            &quad_hash_hex_from_assets(),
        );
        let data = tempfile::TempDir::new().expect("data TempDir");

        // 第一段:生成 quad 并挂 default_policy,推 60 拍后直接 Goodbye(不推进新拍)
        // 使终末检查点包含该主体的 Policy 槽位。
        let (handle1, driver1) = spawn_in_current_runtime(local_policy_config(
            data.path(),
            policies_dir.path(),
            Some("stand_fix"),
        ))
        .await
        .expect("首段 spawn_in_current_runtime");
        wait_for_chunk_0_0(&handle1).await;
        let entity1 = spawn_agent_and_wait(&handle1, "quad", Vec3::new(8.5, -59.5, 8.5)).await;
        for _ in 0..60 {
            advance_sim_tick().await;
            let _ = handle1.drain();
        }
        handle1.send(C2S::Goodbye);
        let outcome1 = driver1.await.expect("首段驱动正常退出");
        assert!(
            outcome1.controller_counters.policy_inferences > 0,
            "首段必已有推理"
        );

        // 核验检查点里确有 Policy(stand_fix) 槽位。
        {
            use storage::traits::CheckpointStore as _;
            let store = storage::redb::RedbStore::open(&data.path().join("checkpoints.redb"))
                .expect("检查点库可读");
            let (_, blob) = store.read_latest().expect("读最新").expect("存在检查点");
            let bundle = driver::decode_checkpoint(&blob).expect("检查点可解");
            let has_slot = bundle.controllers.iter().any(|r| {
                r.entity == entity1
                    && r.slot.kind
                        == crate::controller::ControllerKind::Policy(
                            crate::controller::PolicyKey::of("stand_fix"),
                        )
            });
            assert!(has_slot, "终末检查点必须包含 stand_fix 策略槽位");
        }

        // 第二段:同 data_dir + 同 default_policy 恢复启动 → 不 fail-stop;
        // 恢复首拍清理断线玩家旧身体后,新连接再生成 quad 并推 120 拍 → inferences > 0。
        let (handle2, driver2) = spawn_in_current_runtime(local_policy_config(
            data.path(),
            policies_dir.path(),
            Some("stand_fix"),
        ))
        .await
        .expect("恢复启动不得 fail-stop");
        wait_for_chunk_0_0(&handle2).await;
        let _entity2 = spawn_agent_and_wait(&handle2, "quad", Vec3::new(8.5, -59.5, 8.5)).await;
        for _ in 0..120 {
            advance_sim_tick().await;
            let _ = handle2.drain();
        }
        handle2.send(C2S::Goodbye);
        let outcome2 = driver2.await.expect("恢复段驱动正常退出");
        assert!(
            outcome2.controller_counters.policy_inferences > 0,
            "恢复段推 120 拍后 inferences 须 > 0,实测 {}",
            outcome2.controller_counters.policy_inferences
        );
        assert_eq!(outcome2.controller_counters.policy_errors, 0);
        assert_eq!(outcome2.controller_counters.send_rejected, 0);
    }

    /// T-A6 fail-stop:default_policy 指不存在的包 → bootstrap Err;
    /// 包 body_hash 与注册表任何身体都不匹配 → Err(消息含策略名)。
    #[test]
    fn t_a6_fail_stop_missing_or_mismatched_default_policy() {
        let policies_dir = tempfile::TempDir::new().expect("policies TempDir");
        let data1 = tempfile::TempDir::new().expect("data1 TempDir");
        let cfg_missing =
            local_policy_config(data1.path(), policies_dir.path(), Some("no_such_pack"));
        let err1 = match driver::bootstrap(&cfg_missing.config, || {
            Box::new(voxel::SimpleRegistry::builtin())
        }) {
            Err(e) => e,
            Ok(_) => panic!("不存在的 default_policy 包必须 fail-stop"),
        };
        let msg1 = format!("{err1}");
        assert!(
            msg1.contains("no_such_pack"),
            "错误消息须含策略名 no_such_pack,实际:{msg1}"
        );

        write_fixture_pack(policies_dir.path(), "bad_hash_pack", "0000000000000000");
        let data2 = tempfile::TempDir::new().expect("data2 TempDir");
        let cfg_mismatch =
            local_policy_config(data2.path(), policies_dir.path(), Some("bad_hash_pack"));
        let err2 = match driver::bootstrap(&cfg_mismatch.config, || {
            Box::new(voxel::SimpleRegistry::builtin())
        }) {
            Err(e) => e,
            Ok(_) => panic!("body_hash 无匹配身体的 default_policy 必须 fail-stop"),
        };
        let msg2 = format!("{err2}");
        assert!(
            msg2.contains("bad_hash_pack") && msg2.contains("0000000000000000"),
            "错误消息须含策略名与 body_hash,实际:{msg2}"
        );
    }

    /// T-A7 挂载拍口径:C2S::SpawnAgent 生成即挂,`attached_at = now.next()`
    /// (REQ-013 §3.2a,照 startup B03a X1 口径)。观测:Spawned 回执后首个含该
    /// 主体的 AgentStates 拍号 = 生成处理拍 + 1 = attached_at;终末检查点槽位可证。
    #[tokio::test(start_paused = true)]
    async fn t_a7_c2s_spawn_attaches_at_next_tick() {
        let policies_dir = tempfile::TempDir::new().expect("policies TempDir");
        write_fixture_pack(
            policies_dir.path(),
            "stand_fix",
            &quad_hash_hex_from_assets(),
        );
        let data = tempfile::TempDir::new().expect("data TempDir");
        let (handle, driver) = spawn_in_current_runtime(local_policy_config(
            data.path(),
            policies_dir.path(),
            Some("stand_fix"),
        ))
        .await
        .expect("spawn_in_current_runtime");
        wait_for_chunk_0_0(&handle).await;
        let entity = spawn_agent_and_wait(&handle, "quad", Vec3::new(8.5, -59.5, 8.5)).await;
        let mut first_tick: Option<Tick> = None;
        for _ in 0..120 {
            advance_sim_tick().await;
            for m in handle.drain() {
                if let S2C::AgentStates { tick, states, .. } = m
                    && first_tick.is_none()
                    && states.iter().any(|(id, _, _)| *id == entity)
                {
                    first_tick = Some(tick);
                }
            }
        }
        let first_tick = first_tick.expect("120 拍内必收到含该主体的 AgentStates");
        handle.send(C2S::Goodbye);
        let _outcome = driver.await.expect("驱动正常退出");
        use storage::traits::CheckpointStore as _;
        let store = storage::redb::RedbStore::open(&data.path().join("checkpoints.redb"))
            .expect("检查点库可读");
        let (_, blob) = store.read_latest().expect("读最新").expect("存在检查点");
        let bundle = driver::decode_checkpoint(&blob).expect("检查点可解");
        let record = bundle
            .controllers
            .iter()
            .find(|r| r.entity == entity)
            .expect("检查点含该主体槽位");
        assert_eq!(
            record.slot.attached_at, first_tick,
            "attached_at 应为 Spawned 后首个 AgentStates 拍(first={first_tick:?})"
        );
    }

    // ---- REQ-013 C04 单测(T2 / T3 / T4,按 R1 修订裁定) -------------------------

    /// C04 T2:local spawn 自动订阅(无需 Subscribe);推进 10 次(每次 drain 弃)后发
    /// `Sequenced{7, Ping}`,再推进 1 次 → drain 含 `InputAck{seq:7, tick:Tick(11)}`,
    /// 且 `Pong` 先于 `InputAck`。
    #[tokio::test(start_paused = true)]
    async fn c04_t2_sequenced_ping_ack_tick_and_priority() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let (handle, driver) = spawn_in_current_runtime(local_test_config(tmp.path(), 2))
            .await
            .expect("spawn_in_current_runtime");
        // 让 driver_task 在 t=0 初始化 ticker 并吃掉首枚立即 tick,使推进 K 次与迭代 K 对齐。
        tokio::task::yield_now().await;

        for _ in 0..10 {
            tokio::time::advance(Duration::from_millis(17)).await;
            tokio::task::yield_now().await;
            let _ = handle.drain();
        }

        handle.send(C2S::Sequenced {
            seq: 7,
            inner: Box::new(C2S::Ping),
        });

        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        let batch = handle.drain();

        let pong_idx = batch
            .iter()
            .position(|m| matches!(m, S2C::Pong))
            .expect("第 11 拍 batch 必含 Pong");
        let ack_idx = batch
            .iter()
            .position(|m| {
                matches!(
                    m,
                    S2C::InputAck {
                        seq: 7,
                        tick: Tick(11)
                    }
                )
            })
            .unwrap_or_else(|| panic!("第 11 拍 batch 必含 InputAck(7, Tick(11)),实际:{batch:?}"));
        assert!(
            pong_idx < ack_idx,
            "Pong(Critical) 必须先于 InputAck(World),实际 pong_idx={pong_idx}, ack_idx={ack_idx}"
        );

        handle.send(C2S::Goodbye);
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        driver.await.expect("驱动正常退出");
    }

    /// C04 T3(R1 修订):MoveObserver 裸发 + 推进至 ChunkData(0,0) 后:
    /// 1) 发 `Sequenced{3, SpawnAgent capsule}` 推进 1 次 → 同批 `Spawned` 先于
    ///    `InputAck{3}`(Critical 先 World),且 `ack.tick == 同批 EntityStates.tick`;
    /// 2) 发 `Sequenced{4, Command(SetBlock(0,-60,0,STONE))}`:
    ///    - 推进第 1 次批含 `InputAck{4}` 且不含该格 `BlockUpdates`;
    ///    - 推进第 2 次批含该格 `BlockUpdates`,且 `ack.tick + 1 == 该批 EntityStates.tick`。
    #[tokio::test(start_paused = true)]
    async fn c04_t3_sequenced_spawn_agent_and_set_block_ordering() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let cfg = LocalConfig {
            config: Config {
                data_dir: tmp.path().to_path_buf(),
                bodies_dir: repo_assets_dir("bodies"),
                policies_dir: tmp.path().join("policies"),
                ..Config::default()
            },
            view_distance: 2,
        };
        let (handle, driver) = spawn_in_current_runtime(cfg)
            .await
            .expect("spawn_in_current_runtime");

        handle.send(C2S::MoveObserver {
            position: Vec3::new(8.0, -60.0, 8.0),
        });
        let mut saw_chunk = false;
        for _ in 0..200 {
            tokio::time::advance(Duration::from_millis(17)).await;
            tokio::task::yield_now().await;
            for msg in handle.drain() {
                if matches!(msg, S2C::ChunkData { pos, .. } if pos == ChunkPos::new(0, 0)) {
                    saw_chunk = true;
                }
            }
            if saw_chunk {
                break;
            }
        }
        assert!(saw_chunk, "200 拍内必须收到 ChunkData(0,0)");

        // 1) Sequenced{3, SpawnAgent capsule}
        handle.send(C2S::Sequenced {
            seq: 3,
            inner: Box::new(C2S::SpawnAgent {
                body: "capsule".to_owned(),
                position: Vec3::new(8.5, -59.5, 8.5),
                yaw: 0.0,
            }),
        });
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        let spawn_batch = handle.drain();

        let spawned_idx = spawn_batch
            .iter()
            .position(|m| matches!(m, S2C::Spawned { .. }))
            .unwrap_or_else(|| panic!("同批必含 Spawned,实际:{spawn_batch:?}"));
        let (ack3_idx, ack3_tick) = spawn_batch
            .iter()
            .enumerate()
            .find_map(|(i, m)| match m {
                S2C::InputAck { seq: 3, tick } => Some((i, *tick)),
                _ => None,
            })
            .unwrap_or_else(|| panic!("同批必含 InputAck(3),实际:{spawn_batch:?}"));
        let es_tick = spawn_batch
            .iter()
            .find_map(|m| match m {
                S2C::EntityStates { tick, .. } => Some(*tick),
                _ => None,
            })
            .unwrap_or_else(|| panic!("同批必含 EntityStates,实际:{spawn_batch:?}"));

        assert!(
            spawned_idx < ack3_idx,
            "Spawned(Critical) 必须先于 InputAck(World),实际 spawned_idx={spawned_idx}, ack3_idx={ack3_idx}"
        );
        assert_eq!(
            ack3_tick, es_tick,
            "InputAck(3).tick 必须等于同批 EntityStates.tick"
        );

        // 2) Sequenced{4, Command(SetBlock(0,-60,0,STONE))}
        let target_pos = BlockPos::new(0, -60, 0);
        let target_cell = (target_pos, voxel::builtin::STONE);
        handle.send(C2S::Sequenced {
            seq: 4,
            inner: Box::new(C2S::Command(SimCommand::SetBlock {
                pos: target_pos,
                block: voxel::builtin::STONE,
            })),
        });

        // 推进第 1 次:含 InputAck{4},不含该格 BlockUpdates
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        let batch1 = handle.drain();
        let ack4_tick = batch1
            .iter()
            .find_map(|m| match m {
                S2C::InputAck { seq: 4, tick } => Some(*tick),
                _ => None,
            })
            .unwrap_or_else(|| panic!("推进第 1 次批必含 InputAck(4),实际:{batch1:?}"));
        let has_block_in_batch1 = batch1
            .iter()
            .any(|m| matches!(m, S2C::BlockUpdates { updates } if updates.contains(&target_cell)));
        assert!(
            !has_block_in_batch1,
            "推进第 1 次批不得含该格 BlockUpdates(SetBlock n 拍盖章、n+1 拍执行)"
        );

        // 推进第 2 次:含该格 BlockUpdates,且 ack.tick + 1 == 该批 EntityStates.tick
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        let batch2 = handle.drain();
        let has_block_in_batch2 = batch2
            .iter()
            .any(|m| matches!(m, S2C::BlockUpdates { updates } if updates.contains(&target_cell)));
        assert!(
            has_block_in_batch2,
            "推进第 2 次批必含 (0,-60,0)->STONE 的 BlockUpdates,实际:{batch2:?}"
        );
        let es2_tick = batch2
            .iter()
            .find_map(|m| match m {
                S2C::EntityStates { tick, .. } => Some(*tick),
                _ => None,
            })
            .unwrap_or_else(|| panic!("推进第 2 次批必含 EntityStates,实际:{batch2:?}"));
        assert_eq!(
            Tick(ack4_tick.0 + 1),
            es2_tick,
            "ack4.tick + 1 必须等于第 2 批 EntityStates.tick"
        );

        handle.send(C2S::Goodbye);
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        driver.await.expect("驱动正常退出");
    }

    /// C04 T4(R1 修订):`Sequenced{9, Command(SpawnDynamic)}` → 断连(后续 send 无效)+ Bye;
    /// drain 不含任何 `InputAck`。
    #[tokio::test(start_paused = true)]
    async fn c04_t4_sequenced_reject_disconnects_without_ack() {
        let tmp = tempfile::TempDir::new().expect("TempDir");
        let (handle, driver) = spawn_in_current_runtime(local_test_config(tmp.path(), 2))
            .await
            .expect("spawn_in_current_runtime");

        let _ = handle.drain();

        handle.send(C2S::Sequenced {
            seq: 9,
            inner: Box::new(C2S::Command(SimCommand::SpawnDynamic {
                entity: EntityId::from_parts(1, 1),
                state: EntityState::default(),
                desc: EntityDesc::default(),
            })),
        });

        let mut all_msgs = handle.drain();
        for _ in 0..3 {
            tokio::time::advance(Duration::from_millis(17)).await;
            tokio::task::yield_now().await;
            all_msgs.extend(handle.drain());
        }

        assert!(
            all_msgs.iter().any(|m| matches!(m, S2C::Bye { .. })),
            "Reject 断连必含 Bye,实际:{all_msgs:?}"
        );
        assert!(
            !all_msgs.iter().any(|m| matches!(m, S2C::InputAck { .. })),
            "Reject 路径不得收到任何 InputAck,实际:{all_msgs:?}"
        );

        // 断连后后续 send 无效。
        handle.send(C2S::Sequenced {
            seq: 10,
            inner: Box::new(C2S::Ping),
        });
        for _ in 0..3 {
            tokio::time::advance(Duration::from_millis(17)).await;
            tokio::task::yield_now().await;
        }
        let post = handle.drain();
        assert!(
            !post.contains(&S2C::Pong) && !post.iter().any(|m| matches!(m, S2C::InputAck { .. })),
            "断连后 send 无效,不得收到 Pong 或 InputAck,实际:{post:?}"
        );

        drop(handle);
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        driver.await.expect("驱动正常退出");
    }
}
