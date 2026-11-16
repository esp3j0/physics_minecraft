//! REQ-012 E01a:策略的输入输出编解码,不含模型(任务书 §1)。
//!
//! 把 [`crate::runtime::Policy::act`] 里与模型无关的两段拆出来:观测拼装
//! ([`PolicyCodec::encode`])和动作 → 关节目标([`PolicyCodec::decode`])。
//! 训练环境(`agent-env`)与部署(`Policy`)共用同一段代码,保证训练时看到
//! 的观测和部署时逐位同一个函数算出来(AC-E1)。
//!
//! [`PolicyCodec::new`] 做 `Policy` 加载里所有与模型无关的校验,顺序与报错
//! 类型保持现状(身体哈希 → dof → default_pose 长度 → normalization 长度)。

use crate::error::PolicyError;
use crate::manifest::PolicyManifest;
use crate::observe::{AgentSnapshot, observe};
use crate::runtime::Action;

/// 策略的输入输出编解码,不含模型。
#[derive(Debug, Clone)]
pub struct PolicyCodec {
    manifest: PolicyManifest,
    default_pose: Vec<f32>,
    /// 自由度顺序的关节限位(没有限位为 None)。
    limits: Vec<Option<types::JointLimits>>,
    dof: usize,
    obs_dim: usize,
}

impl PolicyCodec {
    /// 做 `Policy` 加载里所有与模型无关的校验(身体哈希、default_pose 长度、
    /// normalization 长度、obs 维度可算等),顺序与报错类型保持现状。
    pub fn new(manifest: PolicyManifest, body: &types::BodyDef) -> Result<Self, PolicyError> {
        // 1. 自身校验。
        manifest.validate()?;
        // 2. body_hash:16 位小写十六进制,且与身体一致。
        let h = &manifest.body_hash;
        let is_hex = h.len() == 16
            && h.bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b));
        if !is_hex {
            return Err(PolicyError::Manifest(format!(
                "body_hash 须为 16 位小写十六进制:{h:?}"
            )));
        }
        let got = body.hash().hex();
        if *h != got {
            return Err(PolicyError::BodyHashMismatch {
                expected: h.clone(),
                got,
            });
        }
        // 3. 自由度。
        let dof = body.dof_count();
        if dof == 0 {
            return Err(PolicyError::NoDof);
        }
        // 4. default_pose 长度;缺省取身体各关节 default_angle(没有就 0)。
        if let Some(p) = &manifest.default_pose
            && p.len() != dof
        {
            return Err(PolicyError::DefaultPoseLen {
                expected: dof,
                got: p.len(),
            });
        }
        let default_pose = manifest.default_pose.clone().unwrap_or_else(|| {
            body.dof_joints()
                .map(|j| j.kind.params().default_angle.unwrap_or(0.0))
                .collect()
        });
        // 5. normalization 长度。
        let obs_dim = manifest.obs_dim(dof);
        if let Some(n) = &manifest.normalization {
            if n.mean.len() != obs_dim {
                return Err(PolicyError::NormalizationLen {
                    expected: obs_dim,
                    got: n.mean.len(),
                });
            }
            if n.std.len() != obs_dim {
                return Err(PolicyError::NormalizationLen {
                    expected: obs_dim,
                    got: n.std.len(),
                });
            }
        }
        let limits = body.dof_joints().map(|j| j.kind.params().limits).collect();
        Ok(Self {
            manifest,
            default_pose,
            limits,
            dof,
            obs_dim,
        })
    }

    pub fn manifest(&self) -> &PolicyManifest {
        &self.manifest
    }

    pub fn dof(&self) -> usize {
        self.dof
    }

    pub fn obs_dim(&self) -> usize {
        self.obs_dim
    }

    /// `act` 的第 1–4 步:观测 → 归一化 → 裁剪 → 非有限检查。写进 `out`(先清空)。
    pub fn encode(&self, snapshot: &AgentSnapshot, out: &mut Vec<f32>) -> Result<(), PolicyError> {
        // 1. 观测。
        observe(&self.manifest.obs, &self.default_pose, snapshot, out)?;
        // 2. 归一化。
        if let Some(n) = &self.manifest.normalization {
            for (x, (m, s)) in out.iter_mut().zip(n.mean.iter().zip(&n.std)) {
                *x = (*x - m) / s;
            }
        }
        // 3. 观测裁剪。
        if let Some(c) = self.manifest.obs_clip {
            for x in out.iter_mut() {
                *x = x.clamp(-c, c);
            }
        }
        // 4. 非有限观测不推理。
        if out.iter().any(|x| !x.is_finite()) {
            return Err(PolicyError::NonFiniteObservation);
        }
        Ok(())
    }

    /// `act` 的第 6–8 步:非有限检查 → 动作裁剪 → 目标 = default + scale × raw,
    /// 夹进限位。`model_out.len() != dof` → `ActionDimMismatch`。
    pub fn decode(&self, model_out: &[f32]) -> Result<Action, PolicyError> {
        if model_out.len() != self.dof {
            return Err(PolicyError::ActionDimMismatch {
                body: self.dof,
                model: model_out.len(),
            });
        }
        // 6. 非有限输出。
        if model_out.iter().any(|x| !x.is_finite()) {
            return Err(PolicyError::NonFiniteOutput);
        }
        // 7. 动作裁剪 → raw。
        let raw: Vec<f32> = match self.manifest.action_clip {
            Some(c) => model_out.iter().map(|a| a.clamp(-c, c)).collect(),
            None => model_out.to_vec(),
        };
        // 8. 目标 = default_pose + action_scale × raw,夹进关节限位。
        let mut targets = Vec::with_capacity(self.dof);
        let mut clamped = 0;
        for (i, (&r, &d)) in raw.iter().zip(&self.default_pose).enumerate() {
            let t = d + self.manifest.action_scale * r;
            let t = match self.limits[i] {
                Some(l) => {
                    if t < l.lower || t > l.upper {
                        clamped += 1;
                    }
                    t.clamp(l.lower, l.upper)
                }
                None => t,
            };
            targets.push(t);
        }
        Ok(Action {
            raw,
            targets,
            clamped,
        })
    }
}

#[cfg(test)]
mod tests;
