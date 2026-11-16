//! REQ-012 D01a:观测函数(任务书 1.3)。纯函数:不读全局状态,
//! 除 `out` 外不分配内存。地形高度采样、足端接触本单不做。

use serde::{Deserialize, Serialize};
use types::glam::{Quat, Vec3};

use crate::error::PolicyError;
use crate::manifest::ObsEntry;

/// 观测项类别(snake_case 序列化;不认识的 term 在解析阶段报错)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObsTerm {
    /// 3:根连杆线速度,变到机身系。
    BaseLinVel,
    /// 3:根连杆角速度,变到机身系。
    BaseAngVel,
    /// 3:世界系重力方向 (0, −1, 0) 变到机身系。
    ProjectedGravity,
    /// dof:关节角 − default_pose。
    JointPos,
    /// dof:关节角速度。
    JointVel,
    /// dof:上一次的原始动作(裁剪后、缩放前);第一次是全 0。
    LastAction,
    /// 3:[move_x, move_z, yaw]。
    Intent,
}

impl ObsTerm {
    /// 该项的维度。
    pub fn dim(self, dof: usize) -> usize {
        match self {
            Self::JointPos | Self::JointVel | Self::LastAction => dof,
            Self::BaseLinVel | Self::BaseAngVel | Self::ProjectedGravity | Self::Intent => 3,
        }
    }
}

/// 一只主体在某一拍末的状态。由调用方(D01b 的驱动层)填。
#[derive(Debug, Clone, PartialEq)]
pub struct AgentSnapshot {
    /// 机身系 → 世界系。
    pub root_rotation: Quat,
    /// 世界系。
    pub root_linvel: Vec3,
    /// 世界系。
    pub root_angvel: Vec3,
    /// 长度 dof,自由度顺序。
    pub joint_pos: Vec<f32>,
    pub joint_vel: Vec<f32>,
    pub last_action: Vec<f32>,
    pub intent: [f32; 3],
}

fn push_vec3(out: &mut Vec<f32>, v: Vec3, scale: f32) {
    out.extend([v.x * scale, v.y * scale, v.z * scale]);
}

/// 按 `entries` 的顺序把各项拼成观测向量,写进 `out`(先清空)。
/// 机身系向量 = root_rotation 的逆作用在世界系向量上;每一项乘它的 `scale`。
/// `snapshot` 里三个数组的长度不等于 `default_pose.len()` → `SnapshotDof`。
pub fn observe(
    entries: &[ObsEntry],
    default_pose: &[f32],
    snapshot: &AgentSnapshot,
    out: &mut Vec<f32>,
) -> Result<(), PolicyError> {
    let dof = default_pose.len();
    for arr in [&snapshot.joint_pos, &snapshot.joint_vel, &snapshot.last_action] {
        if arr.len() != dof {
            return Err(PolicyError::SnapshotDof {
                expected: dof,
                got: arr.len(),
            });
        }
    }
    out.clear();
    let inv = snapshot.root_rotation.inverse();
    for e in entries {
        match e.term {
            ObsTerm::BaseLinVel => push_vec3(out, inv * snapshot.root_linvel, e.scale),
            ObsTerm::BaseAngVel => push_vec3(out, inv * snapshot.root_angvel, e.scale),
            ObsTerm::ProjectedGravity => {
                push_vec3(out, inv * Vec3::new(0.0, -1.0, 0.0), e.scale);
            }
            ObsTerm::JointPos => {
                out.extend(
                    snapshot
                        .joint_pos
                        .iter()
                        .zip(default_pose)
                        .map(|(q, d)| (q - d) * e.scale),
                );
            }
            ObsTerm::JointVel => {
                out.extend(snapshot.joint_vel.iter().map(|v| v * e.scale));
            }
            ObsTerm::LastAction => {
                out.extend(snapshot.last_action.iter().map(|a| a * e.scale));
            }
            ObsTerm::Intent => push_vec3(out, Vec3::from_array(snapshot.intent), e.scale),
        }
    }
    Ok(())
}