//! 引擎状态机(REQ-008 §3.2、§4;AC-S2、AC-S3)。
//!
//! 纯逻辑、零墙钟、零 macroquad:超时只看累计帧时长,等待只看传入的服务器拍号,
//! 世界状态只经 [`WorldQuery`] 只读读取,因此任何输入都不得 panic、结果确定。

use std::collections::{BTreeMap, BTreeSet};

use types::glam::Vec3;
use types::{BlockId, BlockPos, EntityId, EntityState, WORLD_Y_MIN};
use voxel::{BlockRegistry, SimpleRegistry};

use super::{
    Action, CameraPose, Check, Cmp, DetonateAt, EngineCommand, EngineStatus, FILL_BATCH_PER_FRAME,
    FrameInput, FrameOutput, Scenario, ScreenshotRequest, SpawnAt, StepOutcome, StepRecord,
    WorldQuery,
};
use types::EntityDesc;

/// 结束后的退出码(REQ-008 §4):0 = 全部完成且检查成立;1 = 有检查失败;2 = 场景失败。
pub(super) fn exit_code(status: &EngineStatus) -> Option<i32> {
    match status {
        EngineStatus::Running { .. } => None,
        EngineStatus::Done { failed_checks: 0 } => Some(0),
        EngineStatus::Done { .. } => Some(1),
        EngineStatus::Failed { .. } => Some(2),
    }
}

/// `spawn` 回执超时:起点 + 120 拍收不齐 → 场景失败(细则 6)。
const SPAWN_RECEIPT_TIMEOUT_TICKS: u64 = 120;
/// `detonate nearest` 吸附半径:与相机的三维距离 ≤ 25 m(细则 7)。
const DETONATE_SNAP_RADIUS: f32 = 25.0;
/// 实体半尺寸:底面 = 中心 y − 0.5(细则 9)。
const ENTITY_HALF_SIZE: f32 = 0.5;
/// 静止判定:观测窗口内最大位移 ≤ 0.05 m(细则 9)。
const STILL_DRIFT_MAX: f32 = 0.05;

/// 进行中的 `fill` 步骤(REQ-009 §10):长方体按 `BlockPos` 升序展开的待发格队列,
/// 每帧最多发 [`FILL_BATCH_PER_FRAME`] 条。
#[derive(Debug, Clone, PartialEq)]
struct FillProgress {
    /// 全部待填格,`BlockPos` 升序,展开时一次算好(解析已保证 ≤ 4096 格)。
    cells: Vec<BlockPos>,
    block: BlockId,
    /// 下一条待发格的下标。
    next: usize,
    /// 步骤开始那一帧的拍号(写进 StepRecord.start_tick)。
    start_tick: u64,
}

/// `from..=to`(解析已保证各轴 from ≤ to)按 `BlockPos` 升序展开;`BlockPos` 的
/// `Ord` 是 (x, y, z) 字典序,三重循环顺序即升序,每格恰好一次。`pub(super)`:
/// tools::stage(W05 布景执行器)直接调用同一实现,保证分批顺序逐位一致。
pub(crate) fn fill_cells(from: BlockPos, to: BlockPos) -> Vec<BlockPos> {
    let mut cells = Vec::new();
    for x in from.x..=to.x {
        for y in from.y..=to.y {
            for z in from.z..=to.z {
                cells.push(BlockPos::new(x, y, z));
            }
        }
    }
    cells
}

/// 等待步骤的跨帧状态(与当前步骤一一对应;即时步骤不进入等待状态)。
#[derive(Debug, Clone, Copy, PartialEq)]
enum WaitState {
    Fly {
        start_tick: u64,
    },
    /// possess 等待主体表可见(A06 §4;回执后下一拍才内核生效)。
    Possess {
        start_tick: u64,
        entity: EntityId,
        follow: bool,
    },
    WaitTicks {
        start_tick: u64,
    },
    /// `ok_since` = 当前连续成立段的起始拍号(Some = 正在连续成立中);
    /// 条件不成立即清空,重新成立时记当前拍。满足判据见 WaitUntil 分支:
    /// `tick - ok_since + 1 >= hold_ticks`(按拍号算,不按观测次数)。
    WaitUntil {
        start_tick: u64,
        ok_since: Option<u64>,
    },
    Spawn {
        start_tick: u64,
        emitted: bool,
    },
}

impl WaitState {
    fn start_tick(self) -> u64 {
        match self {
            WaitState::Fly { start_tick }
            | WaitState::WaitTicks { start_tick }
            | WaitState::Possess { start_tick, .. } => start_tick,
            WaitState::WaitUntil { start_tick, .. } => start_tick,
            WaitState::Spawn { start_tick, .. } => start_tick,
        }
    }
}

pub(super) struct EngineInner {
    scenario: Scenario,
    camera: CameraPose,
    coupled: bool,
    /// 正在执行的步骤(0 基);== 步骤数表示全部结束。
    step: usize,
    status: EngineStatus,
    /// 结果为 CheckFailed 的步骤数(`Done::failed_checks`)。
    failed_checks: usize,
    /// spawn 动作 `label` → 收到的回执实体 ID(细则 6;无标签不记)。
    labels: BTreeMap<String, Vec<EntityId>>,
    /// 实体位置历史(细则 9):实体 → [(拍号, 位置)],按拍号递增,只留需要的窗口。
    histories: BTreeMap<EntityId, Vec<(u64, Vec3)>>,
    /// 上次采样时的服务器拍号(细则 9:拍号变化时采样)。
    sampled_tick: Option<u64>,
    /// 场景内最大的 `no_floating.still_ticks`(历史修剪窗口;无该检查则 0)。
    max_still_ticks: u64,
    /// 实心判断用内置注册表(细则 8)。
    registry: SimpleRegistry,
    wait: Option<WaitState>,
    /// 进行中的 `fill` 步骤(REQ-009 §10;与当前步骤一一对应)。
    fill: Option<FillProgress>,
    /// 当前 spawn 步骤已收到的回执(细则 6:按到达顺序消费)。
    spawn_received: Vec<EntityId>,
    /// 场景墙钟累计,秒(细则 10:含当前帧;与服务器拍号无关)。
    elapsed_secs: f32,
    /// A06 §4:spawn_agent 的标签 → 实体 ID(收到 Spawned 回执后登记;
    /// 一场景最多一个身体,解析期已限一条 spawn_agent)。
    agent_labels: BTreeMap<String, EntityId>,
    /// A06 §4:mark_agent 的标签 → 记录时的根位置(agent_moved 的基准)。
    agent_marks: BTreeMap<String, Vec3>,
    /// A06 §4:当前附身的主体(spawn_agent 生成的;possess 成功即登记,
    /// unpossess 清掉;intent/mark_agent 的操作对象)。
    possessed: Option<EntityId>,
    /// A06 §4:possess 步骤的跨帧状态(已发 Possess 命令,等回执与主体流
    /// 可见;`follow` 缺省 false——相机不动)。
    possess_pending: Option<PossessPending>,
}

/// possess 步骤的跨帧状态(A06 §4):等待 Spawned 回执 → 发 Possess →
/// 等主体表出现该 ID(视为附身成功;协议 Possessed 回执不进脚本引擎——
/// 引擎按"主体表可见即 possession 生效"判定,与渲染层 PossessState 的
/// 回执流解耦)。
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PossessPending {
    label: String,
    follow: bool,
    /// possess 步骤开始的拍号(超时判定)。
    start_tick: u64,
}

impl EngineInner {
    pub(super) fn new(scenario: Scenario, start: CameraPose) -> Self {
        let max_still_ticks = scenario
            .steps
            .iter()
            .flat_map(|s| match &s.action {
                Action::Check { checks } => checks.as_slice(),
                Action::WaitUntil { checks, .. } => checks.as_slice(),
                _ => &[],
            })
            .filter_map(|c| match c {
                Check::NoFloating { still_ticks, .. } => Some(*still_ticks),
                _ => None,
            })
            .max()
            .unwrap_or(0);
        Self {
            camera: start,
            coupled: true,
            step: 0,
            status: EngineStatus::Running { step: 1 },
            failed_checks: 0,
            labels: BTreeMap::new(),
            histories: BTreeMap::new(),
            sampled_tick: None,
            max_still_ticks,
            registry: SimpleRegistry::builtin(),
            wait: None,
            fill: None,
            spawn_received: Vec::new(),
            elapsed_secs: 0.0,
            agent_labels: BTreeMap::new(),
            agent_marks: BTreeMap::new(),
            possessed: None,
            possess_pending: None,
            scenario,
        }
    }

    /// 推进一帧,返回输出与结束后的引擎状态。
    pub(super) fn frame(&mut self, input: &FrameInput<'_>) -> (FrameOutput, EngineStatus) {
        let mut out = FrameOutput {
            camera: self.camera,
            coupled: self.coupled,
            commands: Vec::new(),
            screenshot: None,
            finished: Vec::new(),
            possess: None,
        };
        if !matches!(self.status, EngineStatus::Running { .. }) {
            return (out, self.status.clone()); // 细则 11:结束后不再推进
        }

        // 细则 10:墙钟超时只看累计帧时长(含当前帧),与拍号无关。
        self.elapsed_secs += input.dt;
        let timeout_secs = self.scenario.timeout_secs as f32;
        if self.elapsed_secs > timeout_secs {
            let step = self.step + 1;
            self.status = EngineStatus::Failed {
                step,
                reason: format!(
                    "整场景墙钟超时:累计 {:.3} s,超过上限 {} s",
                    self.elapsed_secs, self.scenario.timeout_secs
                ),
            };
            return (out, self.status.clone());
        }

        // 细则 9:拍号变化时先采样,再执行本帧步骤。
        self.sample(input);
        let tick = input.server_tick;

        while let Some(step) = self.scenario.steps.get(self.step) {
            let step_no = self.step + 1; // 步骤序号从 1 开始(细则 1)
            let label = step.label.clone();
            match step.action.clone() {
                Action::Camera { pos, yaw, pitch } => {
                    self.camera = CameraPose { pos, yaw, pitch };
                    self.finish_step(
                        &mut out,
                        step_no,
                        label,
                        "camera",
                        StepOutcome::Ok,
                        None,
                        tick,
                        tick,
                    );
                }
                Action::LookAt { target } => {
                    self.apply_look_at(target);
                    self.finish_step(
                        &mut out,
                        step_no,
                        label,
                        "look_at",
                        StepOutcome::Ok,
                        None,
                        tick,
                        tick,
                    );
                }
                Action::Couple { on } => {
                    self.coupled = on;
                    self.finish_step(
                        &mut out,
                        step_no,
                        label,
                        "couple",
                        StepOutcome::Ok,
                        None,
                        tick,
                        tick,
                    );
                }
                // REQ-009 §10:按 BlockPos 升序每帧最多 FILL_BATCH_PER_FRAME 条;
                // 最后一批发出的那一帧该步结束,后续瞬时步骤可在同一帧继续。
                Action::Fill { from, to, block } => {
                    let mut state = self.fill.take().unwrap_or_else(|| FillProgress {
                        cells: fill_cells(from, to),
                        block,
                        next: 0,
                        start_tick: tick,
                    });
                    let end = (state.next + FILL_BATCH_PER_FRAME).min(state.cells.len());
                    for pos in &state.cells[state.next..end] {
                        out.commands.push(EngineCommand::SetBlock {
                            pos: *pos,
                            block: state.block,
                        });
                    }
                    state.next = end;
                    if state.next >= state.cells.len() {
                        let start_tick = state.start_tick;
                        self.finish_step(
                            &mut out,
                            step_no,
                            label,
                            "fill",
                            StepOutcome::Ok,
                            None,
                            start_tick,
                            tick,
                        );
                    } else {
                        self.fill = Some(state);
                        break;
                    }
                }
                Action::SetBlock { pos, block } => {
                    out.commands.push(EngineCommand::SetBlock { pos, block });
                    self.finish_step(
                        &mut out,
                        step_no,
                        label,
                        "set_block",
                        StepOutcome::Ok,
                        None,
                        tick,
                        tick,
                    );
                }
                Action::Detonate { at, power, radius } => match self
                    .resolve_detonate(at, input.world)
                {
                    Some(center) => {
                        out.commands.push(EngineCommand::Detonate {
                            center,
                            power,
                            radius,
                        });
                        self.finish_step(
                            &mut out,
                            step_no,
                            label,
                            "detonate",
                            StepOutcome::Ok,
                            None,
                            tick,
                            tick,
                        );
                    }
                    None => {
                        let reason = "detonate nearest:相机 25 m 内没有可吸附的活实体".to_owned();
                        self.finish_step(
                            &mut out,
                            step_no,
                            label,
                            "detonate",
                            StepOutcome::Error(reason.clone()),
                            None,
                            tick,
                            tick,
                        );
                        self.status = EngineStatus::Failed {
                            step: step_no,
                            reason,
                        };
                        break;
                    }
                },
                Action::Check { checks } => {
                    let msgs = self.run_checks(&checks, input);
                    let outcome = if msgs.is_empty() {
                        StepOutcome::Ok
                    } else {
                        StepOutcome::CheckFailed(msgs)
                    };
                    if matches!(outcome, StepOutcome::CheckFailed(_)) {
                        self.failed_checks += 1;
                    }
                    self.finish_step(&mut out, step_no, label, "check", outcome, None, tick, tick);
                }
                Action::Screenshot { name, expect, hud } => {
                    let req = ScreenshotRequest {
                        step: step_no,
                        name,
                        expect,
                        hud,
                    };
                    out.screenshot = Some(req.clone());
                    self.finish_step(
                        &mut out,
                        step_no,
                        label,
                        "screenshot",
                        StepOutcome::Ok,
                        Some(req),
                        tick,
                        tick,
                    );
                    break; // 细则 2:截图帧发出请求后停止推进
                }
                Action::Fly { to, speed } => {
                    let start_tick = self.enter_wait(|t| WaitState::Fly { start_tick: t }, tick);
                    let remaining = to - self.camera.pos;
                    let dist = remaining.length();
                    let advance = (speed * input.dt).max(0.0);
                    if dist <= advance {
                        self.camera.pos = to; // 到达:位置精确等于 to,不越过(细则 4)
                        self.finish_step(
                            &mut out,
                            step_no,
                            label,
                            "fly",
                            StepOutcome::Ok,
                            None,
                            start_tick,
                            tick,
                        );
                    } else {
                        // dist > advance ≥ 0 → dist > 0,除法安全。
                        self.camera.pos += remaining * (advance / dist);
                        break;
                    }
                }
                Action::WaitTicks { n } => {
                    let start_tick =
                        self.enter_wait(|t| WaitState::WaitTicks { start_tick: t }, tick);
                    if tick >= start_tick.saturating_add(n) {
                        self.finish_step(
                            &mut out,
                            step_no,
                            label,
                            "wait_ticks",
                            StepOutcome::Ok,
                            None,
                            start_tick,
                            tick,
                        );
                    } else {
                        break;
                    }
                }
                Action::WaitUntil {
                    checks,
                    timeout_ticks,
                    hold_ticks,
                } => {
                    let start_tick = self.enter_wait(
                        |t| WaitState::WaitUntil {
                            start_tick: t,
                            ok_since: None,
                        },
                        tick,
                    );
                    let msgs = self.run_checks(&checks, input);
                    let msgs_is_empty = msgs.is_empty();
                    // 连续成立段起点(ok_since):成立且无段 → 记当前拍;
                    // 不成立 → 清空(重新计数)。同一拍内的重复帧不重复更新。
                    let ok_since = match &mut self.wait {
                        Some(WaitState::WaitUntil { ok_since, .. }) => {
                            if msgs_is_empty {
                                if ok_since.is_none() {
                                    *ok_since = Some(tick);
                                }
                            } else {
                                *ok_since = None;
                            }
                            *ok_since
                        }
                        _ => None,
                    };
                    // 按拍号判连续成立时长(不按观测次数):viewer 每 ~10 拍才
                    // 拿到一次新拍,hold_ticks = N 实际等待的是 N 个服务器拍。
                    let held_ticks = ok_since.map(|s| tick.saturating_sub(s) + 1);
                    // msgs_is_empty 时 ok_since 必为 Some(hold_ticks = 0 也记段,
                    // held = 1 ≥ 0 立即满足 = 旧语义首拍即过);None 只在条件
                    // 不成立时出现,此处必不满足。
                    let hold_met = held_ticks.is_some_and(|n| n >= hold_ticks);
                    if msgs_is_empty && hold_met {
                        self.finish_step(
                            &mut out,
                            step_no,
                            label,
                            "wait_until",
                            StepOutcome::Ok,
                            None,
                            start_tick,
                            tick,
                        );
                    } else if tick >= start_tick.saturating_add(timeout_ticks) {
                        let reason = if hold_ticks > 0 {
                            let held = held_ticks.unwrap_or(0);
                            format!(
                                "wait_until 超时:{timeout_ticks} 拍内条件未连续成立 {hold_ticks} 拍(当前连续 {held} 拍)"
                            )
                        } else {
                            format!("wait_until 超时:{timeout_ticks} 拍内条件未全部成立")
                        };
                        self.finish_step(
                            &mut out,
                            step_no,
                            label,
                            "wait_until",
                            StepOutcome::Error(reason.clone()),
                            None,
                            start_tick,
                            tick,
                        );
                        self.status = EngineStatus::Failed {
                            step: step_no,
                            reason,
                        };
                        break;
                    } else {
                        break;
                    }
                }
                Action::Spawn {
                    at,
                    count,
                    spacing,
                    velocity,
                    label: spawn_label,
                    shape,
                } => {
                    let start_tick = match self.wait {
                        Some(WaitState::Spawn { start_tick, .. }) => start_tick,
                        _ => {
                            self.wait = Some(WaitState::Spawn {
                                start_tick: tick,
                                emitted: false,
                            });
                            self.spawn_received.clear();
                            tick
                        }
                    };
                    let emitted = matches!(self.wait, Some(WaitState::Spawn { emitted: true, .. }));
                    if !emitted {
                        // 细则 6:全部命令在开始那一帧发出。
                        let base = match at {
                            SpawnAt::At(p) => p,
                            SpawnAt::Ahead { ahead, lift } => {
                                self.camera.pos
                                    + self.camera.forward() * ahead
                                    + Vec3::new(0.0, lift, 0.0)
                            }
                        };
                        for i in 0..count {
                            let offset = Vec3::new((i % 5) as f32, 0.0, (i / 5) as f32) * spacing;
                            // F04a:`at` = 刚体原点 = 形状 offset [0,0,0] 那格的
                            // 格心;有 shape 时发复合 EntityDesc(`control.spawn`
                            // 透传,服务端 spawn_owned 生成复合体),无 shape 时
                            // 默认单格(逐位不变)。clone:count > 1 时同形状多发。
                            let desc = match &shape {
                                Some(sh) => {
                                    EntityDesc::compound(super::COMPOUND_HALF_SIZE, sh.clone())
                                }
                                None => EntityDesc::default(),
                            };
                            out.commands.push(EngineCommand::Spawn {
                                state: EntityState {
                                    translation: base + offset,
                                    linvel: velocity,
                                    ..EntityState::default()
                                },
                                desc,
                            });
                        }
                        self.wait = Some(WaitState::Spawn {
                            start_tick,
                            emitted: true,
                        });
                    }
                    // 按到达顺序消费回执;超出 count 的不属于本步骤,丢弃。
                    let want = count as usize;
                    let take = want.saturating_sub(self.spawn_received.len());
                    for id in input.spawned.iter().take(take) {
                        self.spawn_received.push(*id);
                        if let Some(name) = &spawn_label {
                            self.labels.entry(name.clone()).or_default().push(*id);
                        }
                    }
                    if self.spawn_received.len() >= want {
                        self.finish_step(
                            &mut out,
                            step_no,
                            label,
                            "spawn",
                            StepOutcome::Ok,
                            None,
                            start_tick,
                            tick,
                        );
                    } else if tick >= start_tick.saturating_add(SPAWN_RECEIPT_TIMEOUT_TICKS) {
                        let reason = format!(
                            "spawn 回执超时:{SPAWN_RECEIPT_TIMEOUT_TICKS} 拍内只收到 {} / {want} 个",
                            self.spawn_received.len()
                        );
                        self.finish_step(
                            &mut out,
                            step_no,
                            label,
                            "spawn",
                            StepOutcome::Error(reason.clone()),
                            None,
                            start_tick,
                            tick,
                        );
                        self.status = EngineStatus::Failed {
                            step: step_no,
                            reason,
                        };
                        break;
                    } else {
                        break;
                    }
                }
                // ---- A06 §4 主体动作 ----
                // spawn_agent:发 SpawnAgent 命令,等 agent_spawned 回执登记
                // 标签(超时 = 场景失败;一场景最多一个,解析期已限)。
                Action::SpawnAgent { at, yaw, label } => {
                    let start_tick = match self.wait {
                        Some(WaitState::Spawn { start_tick, .. }) => start_tick,
                        _ => {
                            self.wait = Some(WaitState::Spawn {
                                start_tick: tick,
                                emitted: false,
                            });
                            tick
                        }
                    };
                    let emitted = matches!(self.wait, Some(WaitState::Spawn { emitted: true, .. }));
                    if !emitted {
                        out.commands
                            .push(EngineCommand::SpawnAgent { position: at, yaw });
                        self.wait = Some(WaitState::Spawn {
                            start_tick,
                            emitted: true,
                        });
                    }
                    if let Some(id) = input.agent_spawned.first() {
                        self.agent_labels.insert(label.clone(), *id);
                        self.finish_step(
                            &mut out,
                            step_no,
                            Some(label.clone()),
                            "spawn_agent",
                            StepOutcome::Ok,
                            None,
                            start_tick,
                            tick,
                        );
                    } else if tick >= start_tick.saturating_add(SPAWN_RECEIPT_TIMEOUT_TICKS) {
                        let reason = format!(
                            "spawn_agent 回执超时:{SPAWN_RECEIPT_TIMEOUT_TICKS} 拍内未收到"
                        );
                        self.finish_step(
                            &mut out,
                            step_no,
                            Some(label),
                            "spawn_agent",
                            StepOutcome::Error(reason.clone()),
                            None,
                            start_tick,
                            tick,
                        );
                        self.status = EngineStatus::Failed {
                            step: step_no,
                            reason,
                        };
                        break;
                    } else {
                        break;
                    }
                }
                // possess:内核生成在回执后下一拍才生效(Spawned 回执 = 命令
                // 投递时发),立即发 Possess 会被驱动以"实体不存在"拒绝——
                // 等主体表出现该 ID 再发(与渲染层 try_auto_possess 同口径)。
                // follow 只影响渲染层相机,不改引擎状态。
                Action::Possess { label, follow } => {
                    let Some(id) = self.agent_labels.get(&label).copied() else {
                        let reason = format!("possess:标签 \"{label}\" 还没有回执");
                        self.finish_step(
                            &mut out,
                            step_no,
                            None,
                            "possess",
                            StepOutcome::Error(reason.clone()),
                            None,
                            tick,
                            tick,
                        );
                        self.status = EngineStatus::Failed {
                            step: step_no,
                            reason,
                        };
                        break;
                    };
                    let start_tick = self.enter_wait(
                        |t| WaitState::Possess {
                            start_tick: t,
                            entity: id,
                            follow,
                        },
                        tick,
                    );
                    if input.world.live_agent().is_some_and(|(lid, _)| lid == id) {
                        out.commands.push(EngineCommand::Possess { entity: id });
                        self.possessed = Some(id);
                        self.possess_pending = Some(PossessPending {
                            label: label.clone(),
                            follow,
                            start_tick,
                        });
                        self.finish_step(
                            &mut out,
                            step_no,
                            Some(label.clone()),
                            "possess",
                            StepOutcome::Ok,
                            None,
                            start_tick,
                            tick,
                        );
                    } else if tick >= start_tick.saturating_add(SPAWN_RECEIPT_TIMEOUT_TICKS) {
                        let reason =
                            format!("possess 超时:{SPAWN_RECEIPT_TIMEOUT_TICKS} 拍内主体不可见");
                        self.finish_step(
                            &mut out,
                            step_no,
                            Some(label.clone()),
                            "possess",
                            StepOutcome::Error(reason.clone()),
                            None,
                            start_tick,
                            tick,
                        );
                        self.status = EngineStatus::Failed {
                            step: step_no,
                            reason,
                        };
                        break;
                    } else {
                        break;
                    }
                }
                Action::Unpossess => {
                    self.possessed = None;
                    self.possess_pending = None;
                    self.finish_step(
                        &mut out,
                        step_no,
                        label,
                        "unpossess",
                        StepOutcome::Ok,
                        None,
                        tick,
                        tick,
                    );
                }
                // intent:持续生效直到下一条——登记保持意图,每帧若主体
                // 存活重发(连接断开/主体移除即停;发一条即可,保持状态在
                // 服务端)。jump 一次性:只在本帧命令带 true。
                Action::Intent { mv, yaw, jump } => {
                    if let Some(entity) = self.possessed {
                        out.commands.push(EngineCommand::AgentIntent {
                            entity,
                            move_xz: mv,
                            yaw,
                            jump,
                        });
                    }
                    self.finish_step(
                        &mut out,
                        step_no,
                        label,
                        "intent",
                        StepOutcome::Ok,
                        None,
                        tick,
                        tick,
                    );
                }
                // mark_agent:记下当前根位置(主体表无该主体 = 失败)。
                Action::MarkAgent { label } => {
                    let pos = self
                        .possessed
                        .and_then(|_| input.world.live_agent())
                        .filter(|(id, _)| Some(*id) == self.possessed)
                        .map(|(_, p)| p);
                    match pos {
                        Some(p) => {
                            self.agent_marks.insert(label.clone(), p);
                            self.finish_step(
                                &mut out,
                                step_no,
                                None,
                                "mark_agent",
                                StepOutcome::Ok,
                                None,
                                tick,
                                tick,
                            );
                        }
                        None => {
                            let reason = "mark_agent:主体不在主体表(未生成或已移除)".to_owned();
                            self.finish_step(
                                &mut out,
                                step_no,
                                None,
                                "mark_agent",
                                StepOutcome::Error(reason.clone()),
                                None,
                                tick,
                                tick,
                            );
                            self.status = EngineStatus::Failed {
                                step: step_no,
                                reason,
                            };
                            break;
                        }
                    }
                }
            }
        }

        if matches!(self.status, EngineStatus::Running { .. })
            && self.step >= self.scenario.steps.len()
        {
            self.status = EngineStatus::Done {
                failed_checks: self.failed_checks,
            };
        }
        out.camera = self.camera;
        out.coupled = self.coupled;
        // A06 §4:脚本附身状态(follow 从 possess_pending 取;已解除 = None)。
        out.possess = match self.possessed {
            Some(id) => {
                let follow = self
                    .possess_pending
                    .as_ref()
                    .map(|p| p.follow)
                    .unwrap_or(false);
                Some((id, follow))
            }
            None => None,
        };
        (out, self.status.clone())
    }

    /// 结束当前步骤:写记录、推进下标、清等待状态。
    #[allow(clippy::too_many_arguments)]
    fn finish_step(
        &mut self,
        out: &mut FrameOutput,
        step: usize,
        label: Option<String>,
        action: &str,
        outcome: StepOutcome,
        screenshot: Option<ScreenshotRequest>,
        start_tick: u64,
        end_tick: u64,
    ) {
        out.finished.push(StepRecord {
            step,
            label,
            action: action.to_owned(),
            start_tick,
            end_tick,
            camera: self.camera,
            outcome,
            screenshot,
        });
        self.step += 1;
        self.wait = None;
        self.spawn_received.clear();
    }

    /// 进入(或续跑)当前步骤的等待状态,返回起点拍号(细则 3)。
    fn enter_wait(&mut self, make: impl FnOnce(u64) -> WaitState, tick: u64) -> u64 {
        if let Some(w) = self.wait {
            return w.start_tick();
        }
        self.wait = Some(make(tick));
        tick
    }

    /// `look_at` 角度换算(细则 5):yaw = atan2(dz, dx)、pitch = atan2(dy, √(dx²+dz²)),
    /// 单位度;目标与相机重合时保持原朝向。
    fn apply_look_at(&mut self, target: Vec3) {
        let d = target - self.camera.pos;
        if d == Vec3::ZERO {
            return;
        }
        let horiz = (d.x * d.x + d.z * d.z).sqrt();
        if horiz > 0.0 {
            self.camera.yaw = d.z.atan2(d.x).to_degrees();
        }
        self.camera.pitch = d.y.atan2(horiz).to_degrees();
    }

    /// `detonate` 爆心(细则 7):绝对坐标原样;nearest 取相机 25 m 内最近的活实体。
    fn resolve_detonate(&self, at: DetonateAt, world: &dyn WorldQuery) -> Option<Vec3> {
        match at {
            DetonateAt::At(p) => Some(p),
            DetonateAt::Nearest => {
                let mut best: Option<(f32, Vec3)> = None;
                for (_, pos) in world.live_entities() {
                    let dist = (pos - self.camera.pos).length();
                    let closer = match best {
                        Some((best_dist, _)) => dist < best_dist,
                        None => true,
                    };
                    if dist <= DETONATE_SNAP_RADIUS && closer {
                        best = Some((dist, pos));
                    }
                }
                best.map(|(_, pos)| pos)
            }
        }
    }

    /// 位置历史采样(细则 9):拍号变化时采一帧;实体消失即丢弃历史;
    /// 只保留最大 still_ticks 窗口内的样本。
    fn sample(&mut self, input: &FrameInput<'_>) {
        if self.sampled_tick == Some(input.server_tick) {
            return;
        }
        self.sampled_tick = Some(input.server_tick);
        let live = input.world.live_entities();
        let present: BTreeSet<EntityId> = live.iter().map(|(id, _)| *id).collect();
        self.histories.retain(|id, _| present.contains(id));
        let tick = input.server_tick;
        let cutoff = tick.saturating_sub(self.max_still_ticks);
        for (id, pos) in live {
            let samples = self.histories.entry(id).or_default();
            samples.push((tick, pos));
            samples.retain(|(t, _)| *t >= cutoff);
        }
    }

    /// 逐条求值检查项(细则 8),返回失败原因(空 = 全部成立)。
    fn run_checks(&self, checks: &[Check], input: &FrameInput<'_>) -> Vec<String> {
        let mut msgs = Vec::new();
        let live = input.world.live_entities();
        for check in checks {
            match check {
                Check::LiveEntities(cmp) => {
                    let n = live.len() as u64;
                    if !cmp_holds(*cmp, n) {
                        msgs.push(format!("活实体数 {n} 不满足 {cmp:?}"));
                    }
                }
                Check::LabeledAlive { label, cmp } => {
                    // 未知标签按 0 个(细则 8)。
                    let n = self
                        .labels
                        .get(label)
                        .map(|ids| {
                            ids.iter()
                                .filter(|id| live.iter().any(|(lid, _)| lid == *id))
                                .count()
                        })
                        .unwrap_or(0) as u64;
                    if !cmp_holds(*cmp, n) {
                        msgs.push(format!("标签 \"{label}\" 下存活 {n} 个,不满足 {cmp:?}"));
                    }
                }
                Check::Block { pos, is } => match input.world.block_at(*pos) {
                    Some(b) if b == *is => {}
                    // 未装载(None)不等于任何方块,含 air(细则 8)。
                    Some(b) => msgs.push(format!("方块 {pos:?} 是 {b:?},应为 {is:?}")),
                    None => msgs.push(format!("方块 {pos:?} 未装载,不等于任何方块")),
                },
                Check::Count { from, to, is, cmp } => {
                    // 区域内任一格未装载 → 检查不成立(不下结论,不能当 0 格数)。
                    let mut n = 0u64;
                    let mut verdict = Ok(());
                    for x in from.x..=to.x {
                        for y in from.y..=to.y {
                            for z in from.z..=to.z {
                                match input.world.block_at(BlockPos::new(x, y, z)) {
                                    Some(b) if b == *is => n += 1,
                                    Some(_) => {}
                                    None => {
                                        verdict = Err(format!(
                                            "区域 {from:?}..{to:?} 含未装载格 ({x},{y},{z}),不成立"
                                        ));
                                        break;
                                    }
                                }
                            }
                            if verdict.is_err() {
                                break;
                            }
                        }
                        if verdict.is_err() {
                            break;
                        }
                    }
                    match verdict {
                        Err(m) => msgs.push(m),
                        Ok(()) if !cmp_holds(*cmp, n) => msgs.push(format!(
                            "区域 {from:?}..{to:?} 内 {is:?} 格数 {n} 不满足 {cmp:?}"
                        )),
                        Ok(()) => {}
                    }
                }
                Check::ChunksLoaded(cmp) => {
                    let n = input.world.chunks_loaded() as u64;
                    if !cmp_holds(*cmp, n) {
                        msgs.push(format!("已装载 chunk 数 {n} 不满足 {cmp:?}"));
                    }
                }
                Check::NoFloating {
                    height,
                    still_ticks,
                } => {
                    if self.has_floating(*height, *still_ticks, input.server_tick, input.world) {
                        msgs.push(format!(
                            "存在悬空且静止的活实体:底面高出所在柱实心顶面 {height} m 以上,且 {still_ticks} 拍内位移 ≤ {STILL_DRIFT_MAX} m"
                        ));
                    }
                }
                // A06 §4 主体检查:label 经 spawn_agent 登记;位置取主体表
                // 的当前根位置(主体不在表 = 检查不成立,如实报失败)。
                Check::AgentIn { label, from, to } => {
                    let Some(id) = self.agent_labels.get(label) else {
                        msgs.push(format!(
                            "agent_in:标签 \"{label}\" 未登记(spawn_agent 回执未到)"
                        ));
                        continue;
                    };
                    match input.world.live_agent().filter(|(lid, _)| lid == id) {
                        Some((_, p)) => {
                            let inside = p.x >= from.x
                                && p.x <= to.x
                                && p.y >= from.y
                                && p.y <= to.y
                                && p.z >= from.z
                                && p.z <= to.z;
                            if !inside {
                                msgs.push(format!(
                                    "agent_in:主体根位置 ({:.2}, {:.2}, {:.2}) 不在盒 {from:?}..{to:?} 内",
                                    p.x, p.y, p.z
                                ));
                            }
                        }
                        None => msgs.push(format!(
                            "agent_in:主体(标签 \"{label}\")不在主体表(未生成或已移除)"
                        )),
                    }
                }
                Check::AgentMoved { label, ge, le } => {
                    let Some(id) = self.agent_labels.get(label) else {
                        msgs.push(format!(
                            "agent_moved:标签 \"{label}\" 未登记(spawn_agent 回执未到)"
                        ));
                        continue;
                    };
                    let Some(mark) = self.agent_marks.get(label) else {
                        msgs.push(format!(
                            "agent_moved:标签 \"{label}\" 没有 mark_agent 基准点(解析期应已拦截;运行期标记步骤失败)"
                        ));
                        continue;
                    };
                    match input.world.live_agent().filter(|(lid, _)| lid == id) {
                        Some((_, p)) => {
                            let dist = (p - *mark).length();
                            if let Some(ge_m) = ge
                                && dist < *ge_m
                            {
                                msgs.push(format!("agent_moved:位移 {dist:.3} m < 下限 {ge_m} m"));
                            }
                            if let Some(le_m) = le
                                && dist > *le_m
                            {
                                msgs.push(format!("agent_moved:位移 {dist:.3} m > 上限 {le_m} m"));
                            }
                        }
                        None => msgs.push(format!(
                            "agent_moved:主体(标签 \"{label}\")不在主体表(未生成或已移除)"
                        )),
                    }
                }
                Check::AgentAlive { label } => {
                    let alive = self.agent_labels.get(label).is_some_and(|id| {
                        input.world.live_agent().is_some_and(|(lid, _)| lid == *id)
                    });
                    if !alive {
                        msgs.push(format!("agent_alive:标签 \"{label}\" 的主体不在主体表"));
                    }
                }
            }
        }
        msgs
    }

    /// `no_floating`(细则 9):存在"底面高出所在柱最高实心方块顶面 `height` 米以上、
    /// 且最近 `still_ticks` 拍内位移不超过 0.05 m"的活实体 → true。
    /// 观测不足、柱未装载 → 不下结论(不算违规)。
    fn has_floating(
        &self,
        height: f32,
        still_ticks: u64,
        tick: u64,
        world: &dyn WorldQuery,
    ) -> bool {
        let window_from = tick.saturating_sub(still_ticks);
        for (id, pos) in world.live_entities() {
            let Some(samples) = self.histories.get(&id) else {
                continue;
            };
            let Some(&(first_tick, _)) = samples.first() else {
                continue;
            };
            if first_tick > window_from {
                continue; // 观测不足 still_ticks 拍,不下结论
            }
            let pts: Vec<Vec3> = samples
                .iter()
                .filter(|(t, _)| *t >= window_from)
                .map(|(_, p)| *p)
                .collect();
            if diameter(&pts) > STILL_DRIFT_MAX {
                continue; // 在动(如下落中),不算悬空
            }
            let bottom = pos.y - ENTITY_HALF_SIZE;
            let Some(top) = self.ground_top(
                world,
                pos.x.floor() as i32,
                pos.z.floor() as i32,
                bottom.floor() as i32,
            ) else {
                continue; // 柱未装载,不下结论
            };
            if bottom - top > height {
                return true;
            }
        }
        false
    }

    /// 柱 (x, z) 上从 `from_y`(含)向下找最高实心方块,返回其顶面 y;
    /// 碰到未装载的格或世界底 → None(不下结论)。
    fn ground_top(&self, world: &dyn WorldQuery, x: i32, z: i32, from_y: i32) -> Option<f32> {
        let mut y = from_y;
        while y >= WORLD_Y_MIN {
            let b = world.block_at(BlockPos::new(x, y, z))?;
            if self.registry.is_solid(b) {
                return Some(y as f32 + 1.0);
            }
            y -= 1;
        }
        None
    }
}

/// 比较是否成立。
fn cmp_holds(cmp: Cmp, n: u64) -> bool {
    match cmp {
        Cmp::Eq(v) => n == v,
        Cmp::Le(v) => n <= v,
        Cmp::Ge(v) => n >= v,
    }
}

/// 点集的最大成对距离(细则 9 的"最大位移")。
fn diameter(pts: &[Vec3]) -> f32 {
    let mut max_sq = 0.0_f32;
    for (i, a) in pts.iter().enumerate() {
        for b in &pts[i + 1..] {
            max_sq = max_sq.max((*a - *b).length_squared());
        }
    }
    max_sq.sqrt()
}
