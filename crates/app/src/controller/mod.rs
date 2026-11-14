//! 脚本控制者宿主(REQ-012 B02a,§2.2):驱动层的控制者登记表与动作
//! 生成。**不进确定性内核**(§2.3):它产生的动作和玩家按键一样,都是带
//! 拍号的输入,经 `Session::send_agent_joint_targets_to` 写进输入日志。
//!
//! 本单只做 `Script`(`QuadStand` / `QuadTrot`)和 `None`,脚本是开环的:
//! 输出只取决于"挂上去之后过了多少拍",不读任何观测。
//!
//! B02b(§2.3/§2.4):槽位追加活动范围(`leash`,检查点随
//! `ControllerRecord` 走);`apply_leash` 对超出活动圆的 `QuadTrot` 永久
//! 改挂 `QuadStand`——B03 做完多连杆迁移后放宽(届时迁移兜底接管,
//! leash 只是临时护栏)。

pub mod gait;

#[cfg(test)]
mod tests;

use std::collections::BTreeMap;

use messaging::InputSource;
use serde::{Deserialize, Serialize};
use sim_api::log::CommandLog;
use sim_api::session::Session;
use types::{EntityId, Tick};

use gait::{Gait, QUAD_DOF, SIM_DT, stand_targets, trot_targets};

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
}

/// 活动范围(§2.4;B03 之前的临时保护):水平距离根连杆超出 `radius_m`
/// 的 `QuadTrot` 永久改挂 `QuadStand`(attached_at 不动,不再恢复行走)。
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
}

/// 挂载失败原因(类型化,不 panic)。
#[derive(Debug, thiserror::Error, PartialEq)]
pub enum AttachError {
    #[error("dof 不匹配:期望 {expected},得到 {got}")]
    DofMismatch { expected: usize, got: usize },
    #[error("period_ticks 为 0")]
    ZeroPeriod,
    #[error("该实体已挂控制者")]
    AlreadyAttached,
}

/// 默认控制周期:每 2 拍一次(30 Hz,§2.3)。
pub const DEFAULT_CONTROL_PERIOD_TICKS: u32 = 2;
/// `QuadTrot` 起步前先发站立目标的拍数(demo `gait` 的 1 s 站立稳定段;
/// 120 拍 = 2 s @ 60 Hz——脚注:demo rise 是 1 s 渐入 + 站稳,这里取 2 s
/// 站立目标,渐入本身在 `foot_target` 里还有 1 s)。
pub const TROT_SETTLE_TICKS: u64 = 120;

/// 控制者宿主:EntityId → 槽位;动作按 EntityId 升序产出。
#[derive(Debug, Default)]
pub struct ControllerHost {
    slots: BTreeMap<EntityId, ControllerSlot>,
    pub counters: ControllerCounters,
}

impl ControllerHost {
    /// 挂载控制者。已挂 → `AlreadyAttached`;脚本身体 dof 不符 →
    /// `DofMismatch`(`None` 不限 dof);周期 0 → `ZeroPeriod`。
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
        if period_ticks == 0 {
            return Err(AttachError::ZeroPeriod);
        }
        self.slots.insert(
            entity,
            ControllerSlot {
                kind,
                dof,
                period_ticks,
                attached_at: now,
                leash,
            },
        );
        Ok(())
    }

    /// 摘除控制者。存在并摘除 → true;不存在 → false。
    pub fn detach(&mut self, entity: EntityId) -> bool {
        self.slots.remove(&entity).is_some()
    }

    /// 主体被移除(despawned / void_removed)时调用;不存在也不报错。
    pub fn on_removed(&mut self, entity: EntityId) {
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
    /// 成功:条目 append 进 `log`,`actions_emitted += 1`;被拒(迁移窗口
    /// 等):`send_rejected += 1`,`log` 不动,继续下一条,不 panic。
    pub fn drive(&mut self, session: &mut Session, log: &mut CommandLog, now: Tick) {
        for (entity, targets) in self.actions_for_tick(now) {
            match session.send_agent_joint_targets_to(InputSource::SCHEDULER, entity, targets) {
                Ok(entry) => {
                    log.append(entry);
                    self.counters.actions_emitted += 1;
                }
                Err(_) => self.counters.send_rejected += 1,
            }
        }
    }

    /// 活动范围(§2.4;**B03 做完多连杆迁移后放宽**——届时迁移兜底接管,
    /// 本函数只是"走不到分片边界"的临时护栏):对每个带 leash 的
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

    /// 从检查点条目重建宿主(计数清零——遥测不进检查点)。
    pub fn from_records(records: Vec<ControllerRecord>) -> Self {
        let mut host = Self::default();
        for record in records {
            host.slots.insert(record.entity, record.slot);
        }
        host
    }
}
