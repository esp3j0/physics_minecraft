//! 脚本控制者宿主(REQ-012 B02a,§2.2):驱动层的控制者登记表与动作
//! 生成。**不进确定性内核**(§2.3):它产生的动作和玩家按键一样,都是带
//! 拍号的输入,经 `Session::send_agent_joint_targets_to` 写进输入日志。
//!
//! 本单只做 `Script`(`QuadStand` / `QuadTrot`)和 `None`,脚本是开环的:
//! 输出只取决于"挂上去之后过了多少拍",不读任何观测。宿主不持有
//! `Session`,不读世界状态;状态暂不进检查点(B02b 处理)。

pub mod gait;

#[cfg(test)]
mod tests;

use std::collections::BTreeMap;

use messaging::InputSource;
use sim_api::session::Session;
use types::{EntityId, Tick};

use gait::{Gait, QUAD_DOF, SIM_DT, stand_targets, trot_targets};

/// 脚本种类(§2.2)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScriptKind {
    /// 恒发站立目标。
    QuadStand,
    /// 先站 `TROT_SETTLE_TICKS` 拍,之后对角小跑。
    QuadTrot,
}

/// 控制者种类(§2.3 的本单子集)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerKind {
    /// 无控制者:电机保持最后目标,从不发动作。
    None,
    /// 固定函数脚本。
    Script(ScriptKind),
}

/// 一只主体的控制者槽位。
#[derive(Debug, Clone, PartialEq)]
pub struct ControllerSlot {
    pub kind: ControllerKind,
    /// 关节目标长度契约(挂载时校验)。
    pub dof: usize,
    /// 控制周期(拍);默认 [`DEFAULT_CONTROL_PERIOD_TICKS`](30 Hz)。
    pub period_ticks: u32,
    /// 挂载拍(`n = now - attached_at` 的零点)。
    pub attached_at: Tick,
}

/// 控制者计数(遥测;§2.2)。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ControllerCounters {
    pub actions_emitted: u64,
    pub send_rejected: u64,
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
    pub fn attach(
        &mut self,
        entity: EntityId,
        kind: ControllerKind,
        dof: usize,
        period_ticks: u32,
        now: Tick,
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
    /// 成功 `actions_emitted += 1`;被拒(迁移窗口等)`send_rejected += 1`,
    /// 继续下一条,不 panic。
    pub fn drive(&mut self, session: &mut Session, now: Tick) {
        for (entity, targets) in self.actions_for_tick(now) {
            match session.send_agent_joint_targets_to(InputSource::SCHEDULER, entity, targets) {
                Ok(_) => self.counters.actions_emitted += 1,
                Err(_) => self.counters.send_rejected += 1,
            }
        }
    }
}
