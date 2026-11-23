//! 传输包装层(REQ-013 C04 §1.2/§1.3):[`SequencedTransport`] 与 [`LoopbackTransport`]。

use std::collections::VecDeque;

use sim_api::proto::{C2S, S2C, decode_c2s_frame, decode_frame, encode_c2s_frame, encode_frame};
use types::Tick;

use crate::Transport;

/// 客户端 seq 包装传输:对除 `Hello`/`Goodbye` 外的所有上行帧自动分配自增 `seq`(从 1 起)
/// 并包装为 `C2S::Sequenced`;下行 `drain` 原样透传,途中追踪最新 `S2C::InputAck`。
pub struct SequencedTransport<T: Transport> {
    inner: T,
    next_seq: u64,
    last_ack: Option<(u64, Tick)>,
    ack_count: u64,
}

impl<T: Transport> SequencedTransport<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            next_seq: 1,
            last_ack: None,
            ack_count: 0,
        }
    }

    pub fn last_ack(&self) -> Option<(u64, Tick)> {
        self.last_ack
    }

    pub fn ack_count(&self) -> u64 {
        self.ack_count
    }
}

impl<T: Transport> Transport for SequencedTransport<T> {
    fn send(&mut self, c2s: C2S) {
        match c2s {
            C2S::Hello { .. } | C2S::Goodbye => self.inner.send(c2s),
            other => {
                let seq = self.next_seq;
                self.next_seq += 1;
                self.inner.send(C2S::Sequenced {
                    seq,
                    inner: Box::new(other),
                });
            }
        }
    }

    fn drain(&mut self) -> Vec<S2C> {
        let msgs = self.inner.drain();
        for msg in &msgs {
            if let S2C::InputAck { seq, tick } = msg {
                self.last_ack = Some((*seq, *tick));
                self.ack_count += 1;
            }
        }
        msgs
    }
}

/// 同步进程内 loopback 传输:零线程、零时钟,在上下行两端完整走 `sim_api::proto` 帧编解码往返。
///
/// `send` 同步编码入 `up` 队列并立即顺序解码转发给 `inner`(零相位差);
/// `drain` 将 `inner.drain()` 编码入 `down` 队列再顺序解码返回。
pub struct LoopbackTransport<T: Transport> {
    inner: T,
    up: VecDeque<Vec<u8>>,
    down: VecDeque<Vec<u8>>,
}

impl<T: Transport> LoopbackTransport<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            up: VecDeque::new(),
            down: VecDeque::new(),
        }
    }
}

impl<T: Transport> Transport for LoopbackTransport<T> {
    fn send(&mut self, c2s: C2S) {
        self.up.push_back(encode_c2s_frame(&c2s));
        while let Some(frame) = self.up.pop_front() {
            let decoded = decode_c2s_frame(&frame).expect("loopback 自产帧必可解");
            self.inner.send(decoded);
        }
    }

    fn drain(&mut self) -> Vec<S2C> {
        for msg in self.inner.drain() {
            self.down.push_back(encode_frame(&msg));
        }
        let mut out = Vec::with_capacity(self.down.len());
        while let Some(frame) = self.down.pop_front() {
            out.push(decode_frame(&frame).expect("loopback 自产帧必可解"));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;
    use std::time::Duration;

    use app::config::Config;
    use app::local::{LocalConfig, spawn_in_current_runtime};
    use messaging::{InputSource, SimCommand};
    use sim_api::proto::{C2S, S2C, encode_frame};
    use types::glam::Vec3;
    use types::{BlockPos, Tick};

    use super::*;
    use crate::LocalTransport;
    use crate::client::s2c_to_event;
    use crate::script::{
        CameraPose, Engine, EngineStatus, FrameInput, engine_commands_to_c2s, parse_scenario,
    };
    use crate::world::WorldState;

    struct StubTransport {
        sent: Vec<C2S>,
        incoming: Vec<S2C>,
    }

    impl StubTransport {
        fn new(incoming: Vec<S2C>) -> Self {
            Self {
                sent: Vec::new(),
                incoming,
            }
        }
    }

    impl Transport for StubTransport {
        fn send(&mut self, c2s: C2S) {
            self.sent.push(c2s);
        }

        fn drain(&mut self) -> Vec<S2C> {
            std::mem::take(&mut self.incoming)
        }
    }

    /// C04 T5:`SequencedTransport` 对非 Hello/Goodbye 帧依次分配 seq 1/2/3 包装,
    /// Hello/Goodbye 原样透传,并在 drain 时更新 `last_ack` / `ack_count`。
    #[test]
    fn c04_t5_sequenced_transport_wrapping_and_ack_tracking() {
        let stub = StubTransport::new(vec![
            S2C::Pong,
            S2C::InputAck {
                seq: 2,
                tick: Tick(5),
            },
        ]);
        let mut t = SequencedTransport::new(stub);

        let set_block = SimCommand::SetBlock {
            pos: BlockPos::new(0, -60, 0),
            block: voxel::builtin::STONE,
        };
        t.send(C2S::Subscribe { view_distance: 2 });
        t.send(C2S::Ping);
        t.send(C2S::Command(set_block.clone()));
        t.send(C2S::Hello { client_id: 99 });
        t.send(C2S::Goodbye);

        assert_eq!(
            t.inner.sent,
            vec![
                C2S::Sequenced {
                    seq: 1,
                    inner: Box::new(C2S::Subscribe { view_distance: 2 }),
                },
                C2S::Sequenced {
                    seq: 2,
                    inner: Box::new(C2S::Ping),
                },
                C2S::Sequenced {
                    seq: 3,
                    inner: Box::new(C2S::Command(set_block)),
                },
                C2S::Hello { client_id: 99 },
                C2S::Goodbye,
            ]
        );

        let drained = t.drain();
        assert_eq!(
            drained,
            vec![
                S2C::Pong,
                S2C::InputAck {
                    seq: 2,
                    tick: Tick(5),
                },
            ]
        );
        assert_eq!(t.last_ack(), Some((2, Tick(5))));
        assert_eq!(t.ack_count(), 1);
    }

    /// C04 T6:`LoopbackTransport` 上行同步编解码转发、下行编解码往返,顺序与值逐位不变。
    #[test]
    fn c04_t6_loopback_transport_roundtrip_and_sync_send() {
        let s2c_a = S2C::Welcome {
            player_id: InputSource(1),
            tick: Tick(0),
        };
        let s2c_b = S2C::InputAck {
            seq: 1,
            tick: Tick(3),
        };
        let stub = StubTransport::new(vec![s2c_a.clone(), s2c_b.clone()]);
        let mut lb = LoopbackTransport::new(stub);

        let c2s_a = C2S::Sequenced {
            seq: 1,
            inner: Box::new(C2S::Ping),
        };
        let c2s_b = C2S::MoveObserver {
            position: Vec3::new(8.0, -60.0, 8.0),
        };
        lb.send(c2s_a.clone());
        lb.send(c2s_b.clone());

        // R1 §1.3:send 同步 flush,drain 前 inner 即已按序收到解码后的原值。
        assert_eq!(lb.inner.sent, vec![c2s_a, c2s_b]);

        let out = lb.drain();
        assert_eq!(out, vec![s2c_a, s2c_b]);
    }

    const AC2_SCENARIO_TOML: &str = r#"
name = "c04_ac2_reconcile"
req = "REQ-013"
description = "AC2 双传输位级对账(无 perf/无 debug/无截图)"
timeout_secs = 60
[viewer]
view_distance = 2
[[step]]
camera = { pos = [28.5, -53.5, 32.5], yaw = -90.0, pitch = -35.0 }
[[step]]
wait_ticks = { n = 15 }
[[step]]
set_block = { pos = [0, -60, 0], block = "stone" }
[[step]]
wait_ticks = { n = 3 }
[[step]]
check = { block = { pos = [0, -60, 0], is = "stone" } }
[[step]]
spawn = { at = [4.0, -59.0, 4.0] }
[[step]]
spawn_agent = { at = [8.0, -59.0, 8.0], label = "cap" }
[[step]]
possess = { label = "cap", follow = false }
[[step]]
intent = { move = [0.3, 0.0], yaw = 0.0, jump = false }
[[step]]
wait_ticks = { n = 8 }
[[step]]
unpossess = {}
[[step]]
wait_ticks = { n = 4 }
"#;

    struct ReconcileRun {
        wire_bytes: Vec<u8>,
        failed_checks: usize,
        last_ack: Option<(u64, Tick)>,
        saw_welcome: bool,
        chunk_data_count: usize,
        saw_stone_update: bool,
        spawned_count: usize,
        possessed_some: usize,
        possessed_none: usize,
        ack_seqs: Vec<u64>,
        advances: usize,
    }

    fn repo_bodies_dir() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/bodies")
    }

    async fn run_ac2_stack<T: Transport>(
        mut transport: SequencedTransport<T>,
        driver: tokio::task::JoinHandle<app::driver::DriverOutcome>,
    ) -> ReconcileRun {
        let scenario = parse_scenario(AC2_SCENARIO_TOML).expect("AC2 内联场景合法");
        let mut engine = Engine::new(
            scenario,
            CameraPose {
                pos: Vec3::ZERO,
                yaw: 0.0,
                pitch: 0.0,
            },
        );
        let mut world = WorldState::new();

        // R1 §2:前置经包装层发 MoveObserver(8,-60,8)(seq 1);不发 Subscribe。
        transport.send(C2S::MoveObserver {
            position: Vec3::new(8.0, -60.0, 8.0),
        });

        let mut wire_bytes = Vec::new();
        let mut saw_welcome = false;
        let mut chunk_data_count = 0usize;
        let mut saw_stone_update = false;
        let mut spawned_count = 0usize;
        let mut possessed_some = 0usize;
        let mut possessed_none = 0usize;
        let mut ack_seqs = Vec::new();
        let mut done_failed_checks: Option<usize> = None;
        let mut advances = 0usize;

        let target_cell = (BlockPos::new(0, -60, 0), voxel::builtin::STONE);

        for step in 1..=49usize {
            tokio::time::advance(Duration::from_millis(17)).await;
            tokio::task::yield_now().await;
            advances = step;

            for msg in transport.drain() {
                wire_bytes.extend_from_slice(&encode_frame(&msg));
                match &msg {
                    S2C::Welcome { .. } => saw_welcome = true,
                    S2C::ChunkData { .. } => chunk_data_count += 1,
                    S2C::BlockUpdates { updates } => {
                        if updates.contains(&target_cell) {
                            saw_stone_update = true;
                        }
                    }
                    S2C::Spawned { .. } => spawned_count += 1,
                    S2C::Possessed { entity: Some(_) } => possessed_some += 1,
                    S2C::Possessed { entity: None } => possessed_none += 1,
                    S2C::InputAck { seq, .. } => ack_seqs.push(*seq),
                    _ => {}
                }
                if let Some(ev) = s2c_to_event(msg) {
                    world.apply(ev);
                }
            }

            let spawned = world.take_spawned();
            let server_tick = world.tick();
            let out = engine.frame(&FrameInput {
                dt: 1.0 / 60.0,
                server_tick,
                world: &world,
                spawned: &spawned,
            });
            for c2s in engine_commands_to_c2s(&out.commands) {
                transport.send(c2s);
            }

            match engine.status() {
                EngineStatus::Done { failed_checks } => {
                    done_failed_checks = Some(*failed_checks);
                    break;
                }
                EngineStatus::Failed { step, reason } => {
                    panic!("AC2 场景在第 {step} 步失败(advances={advances}):{reason}");
                }
                EngineStatus::Running { .. } => {}
            }
        }

        let failed_checks = done_failed_checks.unwrap_or_else(|| {
            panic!(
                "AC2 场景未在硬上限 49 次推进内结束(当前状态:{:?})",
                engine.status()
            )
        });
        let last_ack = transport.last_ack();

        drop(transport);
        tokio::time::advance(Duration::from_millis(17)).await;
        tokio::task::yield_now().await;
        driver.await.expect("驱动正常退出");

        ReconcileRun {
            wire_bytes,
            failed_checks,
            last_ack,
            saw_welcome,
            chunk_data_count,
            saw_stone_update,
            spawned_count,
            possessed_some,
            possessed_none,
            ack_seqs,
            advances,
        }
    }

    fn assert_ac2_anchors(label: &str, run: &ReconcileRun) {
        assert_eq!(run.failed_checks, 0, "{label}: failed_checks 必须为 0");
        assert!(!run.wire_bytes.is_empty(), "{label}: 字节序列非空");
        assert!(run.saw_welcome, "{label}: 必须收到 Welcome");
        assert_eq!(
            run.chunk_data_count, 25,
            "{label}: vd=2 下 ChunkData 必须恰为 25"
        );
        assert!(
            run.saw_stone_update,
            "{label}: 必须收到含 (0,-60,0)->STONE 的 BlockUpdates"
        );
        assert_eq!(
            run.spawned_count, 2,
            "{label}: spawn + spawn_agent 必须恰收 2 次 Spawned"
        );
        assert!(
            run.possessed_some >= 1,
            "{label}: Possessed(Some) 必须 >= 1"
        );
        assert!(
            run.possessed_none >= 1,
            "{label}: Possessed(None) 必须 >= 1"
        );
        assert_eq!(
            run.ack_seqs,
            vec![1, 2, 3, 4, 5, 6, 7],
            "{label}: InputAck seq 序列必须为 1..=7 连续(advances={})",
            run.advances
        );
    }

    /// C04 T7 = AC2 双传输位级对账(R1 修订):
    /// 栈 A = `SequencedTransport<LocalTransport>`、
    /// 栈 B = `SequencedTransport<LoopbackTransport<LocalTransport>>`
    /// 跑同一无渲染脚本场景,整段编码后字节序列逐位相等且全部锚点成立。
    #[tokio::test(start_paused = true)]
    async fn c04_t7_ac2_dual_transport_bitwise_reconcile() {
        let dir_a = tempfile::TempDir::new().expect("TempDir A");
        let cfg_a = LocalConfig {
            config: Config {
                data_dir: dir_a.path().to_path_buf(),
                bodies_dir: repo_bodies_dir(),
                policies_dir: dir_a.path().join("policies"),
                ..Config::default()
            },
            view_distance: 2,
        };
        let (handle_a, driver_a) = spawn_in_current_runtime(cfg_a)
            .await
            .expect("spawn stack A");
        let stack_a = SequencedTransport::new(LocalTransport::from_handle(handle_a));
        let run_a = run_ac2_stack(stack_a, driver_a).await;

        let dir_b = tempfile::TempDir::new().expect("TempDir B");
        let cfg_b = LocalConfig {
            config: Config {
                data_dir: dir_b.path().to_path_buf(),
                bodies_dir: repo_bodies_dir(),
                policies_dir: dir_b.path().join("policies"),
                ..Config::default()
            },
            view_distance: 2,
        };
        let (handle_b, driver_b) = spawn_in_current_runtime(cfg_b)
            .await
            .expect("spawn stack B");
        let stack_b = SequencedTransport::new(LoopbackTransport::new(LocalTransport::from_handle(
            handle_b,
        )));
        let run_b = run_ac2_stack(stack_b, driver_b).await;

        assert_ac2_anchors("stack_a(Local)", &run_a);
        assert_ac2_anchors("stack_b(Loopback)", &run_b);
        assert_eq!(
            run_a.wire_bytes,
            run_b.wire_bytes,
            "AC2 双传输整段 S2C 字节序列必须逐位相等(A len={}, B len={})",
            run_a.wire_bytes.len(),
            run_b.wire_bytes.len()
        );
        assert_eq!(run_a.last_ack, run_b.last_ack, "两栈 last_ack 必须相等");
    }
}
