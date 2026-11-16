//! 脚本控制者宿主(REQ-012 B02a,§2.2):驱动层的控制者登记表与动作
//! 生成。**不进确定性内核**(§2.3):它产生的动作和玩家按键一样,都是带
//! 拍号的输入,经 `Session::send_agent_joint_targets_to` 写进输入日志。
//!
//! 本单只做 `Script`(`QuadStand` / `QuadTrot`)和 `None`,脚本是开环的:
//! 输出只取决于"挂上去之后过了多少拍",不读任何观测。
//!
//! D01b-1(REQ-012 §2.3):追加 `Policy(PolicyKey)`——同步推理,观测经
//! `Session::agent_joint_state` / `agent_root_state` 取出,动作同样经
//! `send_agent_joint_targets_to` 进命令日志(回放不加载模型)。
//!
//! B02b(§2.3/§2.4):槽位追加活动范围(`leash`,检查点随
//! `ControllerRecord` 走);`apply_leash` 对超出活动圆的 `QuadTrot` 永久
//! 改挂 `QuadStand`——B03a 起是普通配置项:脚本主体的活动半径(迁移
//! 已落地,走出分片边界不再停机)。

pub mod gait;
pub mod policy_table;

#[cfg(test)]
mod tests;

use std::collections::BTreeMap;

use messaging::InputSource;
use policy::AgentSnapshot;
use serde::{Deserialize, Serialize};
use sim_api::log::CommandLog;
use sim_api::session::{AgentJointState, Session};
use types::{EntityId, EntityState, Tick};

use gait::{Gait, QUAD_DOF, SIM_DT, stand_targets, trot_targets};
use policy_table::PolicyTable;

/// 策略名的 FNV-1a 64 位(REQ-012 D01b-1 §1.2;算法与 `BodyHash::fnv1a`
/// 相同,对名字的 UTF-8 字节算)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct PolicyKey(pub [u8; 8]);

impl PolicyKey {
    pub fn of(name: &str) -> Self {
        Self(types::BodyHash::fnv1a(name.as_bytes()).0)
    }
}

/// 脚本种类(§2.2)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ScriptKind {
    /// 恒发站立目标。
    QuadStand,
    /// 先站 `TROT_SETTLE_TICKS` 拍,之后对角小跑。
    QuadTrot,
}

/// 控制者种类(§2.3 的本单子集)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControllerKind {
    /// 无控制者:电机保持最后目标,从不发动作。
    None,
    /// 固定函数脚本。
    Script(ScriptKind),
    /// 策略控制(§2.3;D01b-1)。放在枚举最后。
    Policy(PolicyKey),
}

/// 活动范围(§2.4;配置项:脚本主体的活动半径):水平距离根连杆超出
/// `radius_m` 的 `QuadTrot` 永久改挂 `QuadStand`(attached_at 不动,不再
/// 恢复行走)。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Leash {
    /// 活动圆心(世界 xz;= 配置的生成点)。
    pub center_xz: [f32; 2],
    /// 活动半径 m。
    pub radius_m: f32,
}

/// 一只主体的控制者槽位。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControllerSlot {
    pub kind: ControllerKind,
    /// 关节目标长度契约(挂载时校验)。
    pub dof: usize,
    /// 控制周期(拍);默认 [`DEFAULT_CONTROL_PERIOD_TICKS`](30 Hz)。
    pub period_ticks: u32,
    /// 挂载拍(`n = now - attached_at` 的零点)。
    pub attached_at: Tick,
    /// 活动范围(§2.4;配置生成带,`attach` 传入;无 = 不约束)。
    pub leash: Option<Leash>,
    /// 给策略的意图观测 [move_x, move_z, yaw](D01b-1 §1.2)。脚本和
    /// None 不用,恒为 0。
    pub intent: [f32; 3],
}

/// 检查点里的控制者条目(§2.5):EntityId + 槽位,恢复时 `from_records`
/// 重建宿主。计数不进检查点(遥测,重启从 0)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControllerRecord {
    pub entity: EntityId,
    pub slot: ControllerSlot,
}

/// 控制者计数(遥测;§2.2)。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ControllerCounters {
    pub actions_emitted: u64,
    pub send_rejected: u64,
    /// leash 触发的永久改挂次数(§2.4;只增不减)。
    pub leash_stops: u64,
    /// 主体迁移在途,本拍跳过发目标的次数(B03a 2.1;只增不减)。
    pub skipped_in_transit: u64,
    /// 主体已冻结,本拍跳过发目标的次数(B03c 3.2;只增不减)。
    pub skipped_frozen: u64,
    /// 成功推理并发出目标的次数(D01b-1 §1.2;只增不减)。
    pub policy_inferences: u64,
    /// act 返回 Err 的次数(本拍不发目标,电机保持上次目标;只增不减)。
    pub policy_errors: u64,
    /// 累计被关节限位夹住的自由度个数(只增不减)。
    pub policy_targets_clamped: u64,
    /// 到了控制拍但取不到状态而跳过的次数(只增不减)。
    pub policy_no_state: u64,
}

/// 挂载失败原因(类型化,不 panic)。
#[derive(Debug, thiserror::Error, PartialEq)]
pub enum AttachError {
    #[error("dof 不匹配:期望 {expected},得到 {got}")]
    DofMismatch { expected: usize, got: usize },
    #[error("period_ticks 为 0")]
    ZeroPeriod,
    #[error("dof 为 0")]
    ZeroDof,
    #[error("该实体已挂控制者")]
    AlreadyAttached,
}

/// 默认控制周期:每 2 拍一次(30 Hz,§2.3)。
pub const DEFAULT_CONTROL_PERIOD_TICKS: u32 = 2;
/// `QuadTrot` 起步前先发站立目标的拍数(demo `gait` 的 1 s 站立稳定段;
/// 120 拍 = 2 s @ 60 Hz——脚注:demo rise 是 1 s 渐入 + 站稳,这里取 2 s
/// 站立目标,渐入本身在 `foot_target` 里还有 1 s)。
pub const TROT_SETTLE_TICKS: u64 = 120;

/// 一只策略主体的运行态(D01b-1 §1.2;不进检查点,恢复后也从全 0 开始)。
#[derive(Debug)]
struct PolicyRunState {
    last_action: Vec<f32>,
}

/// 控制者宿主:EntityId → 槽位;动作按 EntityId 升序产出。
#[derive(Debug, Default)]
pub struct ControllerHost {
    slots: BTreeMap<EntityId, ControllerSlot>,
    pub counters: ControllerCounters,
    policy_state: BTreeMap<EntityId, PolicyRunState>,
}

impl ControllerHost {
    /// 挂载控制者。已挂 → `AlreadyAttached`;脚本身体 dof 不符 →
    /// `DofMismatch`(`None` 不限 dof);策略 dof 0 → `ZeroDof`(dof 与
    /// 周期由调用方从 `PolicyTable` 取出传入,D01b-1 §1.3);周期 0 →
    /// `ZeroPeriod`。
    /// `leash` = 活动范围(§2.4;配置生成的调用方传 `Some`,旧调用传 `None`)。
    pub fn attach(
        &mut self,
        entity: EntityId,
        kind: ControllerKind,
        dof: usize,
        period_ticks: u32,
        now: Tick,
        leash: Option<Leash>,
    ) -> Result<(), AttachError> {
        if self.slots.contains_key(&entity) {
            return Err(AttachError::AlreadyAttached);
        }
        if matches!(kind, ControllerKind::Script(_)) && dof != QUAD_DOF {
            return Err(AttachError::DofMismatch {
                expected: QUAD_DOF,
                got: dof,
            });
        }
        if matches!(kind, ControllerKind::Policy(_)) && dof == 0 {
            return Err(AttachError::ZeroDof);
        }
        if period_ticks == 0 {
            return Err(AttachError::ZeroPeriod);
        }
        if matches!(kind, ControllerKind::Policy(_)) {
            self.policy_state.insert(
                entity,
                PolicyRunState {
                    last_action: vec![0.0; dof],
                },
            );
        }
        self.slots.insert(
            entity,
            ControllerSlot {
                kind,
                dof,
                period_ticks,
                attached_at: now,
                leash,
                intent: [0.0; 3],
            },
        );
        Ok(())
    }

    /// 设置策略意图观测(D01b-1 §1.4:启动逻辑 attach 后取配置值写入)。
    pub fn set_intent(&mut self, entity: EntityId, intent: [f32; 3]) {
        if let Some(slot) = self.slots.get_mut(&entity) {
            slot.intent = intent;
        }
    }

    /// 摘除控制者。存在并摘除 → true;不存在 → false。
    pub fn detach(&mut self, entity: EntityId) -> bool {
        self.policy_state.remove(&entity);
        self.slots.remove(&entity).is_some()
    }

    /// 主体被移除(despawned / void_removed)时调用;不存在也不报错。
    pub fn on_removed(&mut self, entity: EntityId) {
        self.policy_state.remove(&entity);
        self.slots.remove(&entity);
    }

    /// 实体的控制者种类;未挂 → None。
    pub fn kind_of(&self, entity: EntityId) -> Option<ControllerKind> {
        self.slots.get(&entity).map(|s| s.kind)
    }

    /// 任一条目存在即 `Some`(驱动每拍"有无挂载"的零成本检查)。
    pub fn first_slot(&self) -> Option<(EntityId, ControllerKind)> {
        self.slots.iter().next().map(|(&e, s)| (e, s.kind))
    }

    /// 纯函数:本拍该发的动作,按 EntityId 升序。只在
    /// `n % period_ticks == 0` 的拍发(`n = now - attached_at`);`None`
    /// 从不发。
    pub fn actions_for_tick(&self, now: Tick) -> Vec<(EntityId, Vec<f32>)> {
        let mut out = Vec::new();
        for (&entity, slot) in &self.slots {
            let ControllerKind::Script(script) = slot.kind else {
                continue;
            };
            // 挂载拍在未来的条目不发目标(B03a:startup attach 的
            // attached_at = spawn 拍 +1,spawn 命令拍末 tick 才应用、owner
            // 拍末才 claim;saturating_sub 的下溢会把 n 折成 0,0%period==0
            // 误触发同拍发目标 → send_rejected)。now ≥ attached_at 后
            // 与原口径逐位一致。
            if now.0 < slot.attached_at.0 {
                continue;
            }
            let n = now.0.saturating_sub(slot.attached_at.0);
            if n % u64::from(slot.period_ticks) != 0 {
                continue;
            }
            let targets: Vec<f32> = match script {
                ScriptKind::QuadStand => stand_targets(&Gait::DEFAULT).to_vec(),
                ScriptKind::QuadTrot => {
                    if n < TROT_SETTLE_TICKS {
                        stand_targets(&Gait::DEFAULT).to_vec()
                    } else {
                        let t = (n - TROT_SETTLE_TICKS) as f32 * SIM_DT;
                        trot_targets(&Gait::DEFAULT, t).to_vec()
                    }
                }
            };
            out.push((entity, targets));
        }
        out
    }

    /// 调 [`Self::actions_for_tick`],逐条
    /// `session.send_agent_joint_targets_to(InputSource::SCHEDULER, ..)`。
    /// `skip` = 本拍迁移在途的主体集合(B03a 2.1:在途投递会被内核静默
    /// 拒绝,提前跳过,不算 `send_rejected`、不进日志;驱动在 `drive` 之前
    /// 取一次 `in_transit_entities()` 传入)。`frozen` = 本拍已冻结的主体
    /// 集合(B03c 3.2:冻结期间控制者不发目标,跳过记 `skipped_frozen`,
    /// 不进日志;驱动在 `drive` 之前取一次 `frozen_agents()` 传入)。
    /// 成功:条目 append 进 `log`,`actions_emitted += 1`;在途跳过:
    /// `skipped_in_transit += 1`;冻结跳过:`skipped_frozen += 1`;被拒
    /// (其他原因):`send_rejected += 1`,`log` 不动,继续下一条,不 panic。
    pub fn drive(
        &mut self,
        session: &mut Session,
        log: &mut CommandLog,
        now: Tick,
        skip: &std::collections::BTreeSet<EntityId>,
        frozen: &std::collections::BTreeSet<EntityId>,
    ) {
        for (entity, targets) in self.actions_for_tick(now) {
            if skip.contains(&entity) {
                self.counters.skipped_in_transit += 1;
                continue;
            }
            if frozen.contains(&entity) {
                self.counters.skipped_frozen += 1;
                continue;
            }
            match session.send_agent_joint_targets_to(InputSource::SCHEDULER, entity, targets) {
                Ok(entry) => {
                    log.append(entry);
                    self.counters.actions_emitted += 1;
                }
                Err(_) => self.counters.send_rejected += 1,
            }
        }
    }

    /// 活动范围(§2.4;配置项:脚本主体的活动半径):对每个带 leash 的
    /// `Script(QuadTrot)` 条目,读根连杆位置(经 `root_xz` 闭包,驱动侧从
    /// `session.agent_states()` 取);水平距离 **>** `radius_m` → kind 永久
    /// 改成 `Script(QuadStand)`,`attached_at` 不动,`leash_stops += 1`。
    /// 已改挂的(`QuadStand`)不再受检查,走回圈内也不恢复行走。
    pub fn apply_leash(&mut self, root_xz: impl Fn(EntityId) -> Option<[f32; 2]>) {
        let mut stopped: Vec<EntityId> = Vec::new();
        for (&entity, slot) in &self.slots {
            let (ControllerKind::Script(kind), Some(leash)) = (&slot.kind, slot.leash) else {
                continue;
            };
            if *kind != ScriptKind::QuadTrot {
                continue;
            }
            let Some([x, z]) = root_xz(entity) else {
                continue;
            };
            let dx = x - leash.center_xz[0];
            let dz = z - leash.center_xz[1];
            if dx * dx + dz * dz > leash.radius_m * leash.radius_m {
                stopped.push(entity);
            }
        }
        for entity in stopped {
            if let Some(slot) = self.slots.get_mut(&entity) {
                slot.kind = ControllerKind::Script(ScriptKind::QuadStand);
                self.counters.leash_stops += 1;
            }
        }
    }

    /// 摘掉 `alive` 判否的条目(主体被移除后调用;§2.3 每拍末),返回摘掉
    /// 的条数。
    pub fn retain_alive(&mut self, alive: impl Fn(EntityId) -> bool) -> usize {
        let dead: Vec<EntityId> = self.slots.keys().copied().filter(|&e| !alive(e)).collect();
        for e in &dead {
            self.slots.remove(e);
            self.policy_state.remove(e);
        }
        dead.len()
    }

    /// 检查点条目(§2.5):EntityId 升序。
    pub fn to_records(&self) -> Vec<ControllerRecord> {
        self.slots
            .iter()
            .map(|(&entity, slot)| ControllerRecord {
                entity,
                slot: slot.clone(),
            })
            .collect()
    }

    /// 从检查点条目重建宿主(计数清零——遥测不进检查点;策略运行态
    /// `last_action` 也从全 0 开始,D01b-1 §1.2)。
    pub fn from_records(records: Vec<ControllerRecord>) -> Self {
        let mut host = Self::default();
        for record in records {
            if matches!(record.slot.kind, ControllerKind::Policy(_)) {
                host.policy_state.insert(
                    record.entity,
                    PolicyRunState {
                        last_action: vec![0.0; record.slot.dof],
                    },
                );
            }
            host.slots.insert(record.entity, record.slot);
        }
        host
    }

    /// 策略主体本拍的动作(D01b-1 §1.3)。到控制拍的条件与脚本相同
    /// (`now ≥ attached_at` 且 `(now − attached_at) % period_ticks == 0`)。
    /// 对每只到拍的策略主体(EntityId 升序):
    ///   在 skip 里 → skipped_in_transit += 1;在 frozen 里 → skipped_frozen += 1;
    ///   取不到关节状态或根状态 → policy_no_state += 1;
    ///   否则拼 AgentSnapshot(intent 取槽位的,last_action 取运行态的)→ policy.act:
    ///     Ok(action)  → 发 targets(同 drive:成功则进日志、actions_emitted += 1、
    ///                   policy_inferences += 1、policy_targets_clamped += action.clamped,
    ///                   运行态 last_action = action.raw;被 session 拒绝则
    ///                   send_rejected += 1,last_action 不更新);
    ///     Err(_)      → policy_errors += 1,不发目标,last_action 不更新。
    /// 没有策略槽位时零动作(不调 Session 的状态方法)。
    pub fn drive_policies(
        &mut self,
        session: &mut Session,
        log: &mut CommandLog,
        table: &mut PolicyTable,
        now: Tick,
        skip: &std::collections::BTreeSet<EntityId>,
        frozen: &std::collections::BTreeSet<EntityId>,
    ) {
        let due: Vec<EntityId> = self
            .slots
            .iter()
            .filter(|(_, slot)| {
                matches!(slot.kind, ControllerKind::Policy(_))
                    && now.0 >= slot.attached_at.0
                    && now.0.saturating_sub(slot.attached_at.0) % u64::from(slot.period_ticks) == 0
            })
            .map(|(&entity, _)| entity)
            .collect();
        for entity in due {
            if skip.contains(&entity) {
                self.counters.skipped_in_transit += 1;
                continue;
            }
            if frozen.contains(&entity) {
                self.counters.skipped_frozen += 1;
                continue;
            }
            let ControllerKind::Policy(key) = self.slots[&entity].kind else {
                continue;
            };
            let intent = self.slots[&entity].intent;
            let Some(joints) = session.agent_joint_state(entity) else {
                self.counters.policy_no_state += 1;
                continue;
            };
            let Some(root) = session.agent_root_state(entity) else {
                self.counters.policy_no_state += 1;
                continue;
            };
            let last_action = self
                .policy_state
                .get(&entity)
                .map(|state| state.last_action.clone())
                .unwrap_or_default();
            let snapshot = build_snapshot(&root, &joints, intent, &last_action);
            let entry = table
                .get_mut(key)
                .expect("策略键必在表中(启动/恢复已校验,fail-stop)");
            match entry.policy.act(&snapshot) {
                Ok(action) => {
                    match session.send_agent_joint_targets_to(
                        InputSource::SCHEDULER,
                        entity,
                        action.targets,
                    ) {
                        Ok(log_entry) => {
                            log.append(log_entry);
                            self.counters.actions_emitted += 1;
                            self.counters.policy_inferences += 1;
                            self.counters.policy_targets_clamped += action.clamped as u64;
                            if let Some(state) = self.policy_state.get_mut(&entity) {
                                state.last_action = action.raw;
                            }
                        }
                        Err(_) => self.counters.send_rejected += 1,
                    }
                }
                Err(_) => self.counters.policy_errors += 1,
            }
        }
    }
}

/// 拼策略观测快照(D01b-1 §1.3;纯函数,可单测):根状态取世界系速度,
/// 关节角/角速度按自由度顺序透传(`observe` 内部再减 default_pose),
/// intent 与 last_action 原样带入。
pub fn build_snapshot(
    root: &EntityState,
    joints: &AgentJointState,
    intent: [f32; 3],
    last_action: &[f32],
) -> AgentSnapshot {
    AgentSnapshot {
        root_rotation: root.rotation,
        root_linvel: root.linvel,
        root_angvel: root.angvel,
        joint_pos: joints.positions.clone(),
        joint_vel: joints.velocities.clone(),
        last_action: last_action.to_vec(),
        intent,
    }
}
