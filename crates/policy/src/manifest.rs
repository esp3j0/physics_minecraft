//! REQ-012 D01a:`policy.toml` 的结构与自身校验(任务书 1.2)。
//! 与身体相关的校验(哈希、dof、长度)在 [`crate::runtime`] 里按顺序做。

use serde::{Deserialize, Serialize};

use crate::error::PolicyError;
use crate::observe::ObsTerm;

fn one() -> f32 {
    1.0
}

/// 策略包清单(`policy.toml`)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyManifest {
    pub name: String,
    /// 16 位小写十六进制,= `BodyHash::hex()`。
    pub body_hash: String,
    /// ≥ 1。
    pub control_period_ticks: u32,
    /// 有限,> 0。
    pub action_scale: f32,
    /// 可选;有则有限,> 0。
    #[serde(default)]
    pub action_clip: Option<f32>,
    /// 可选;有则有限,> 0。
    #[serde(default)]
    pub obs_clip: Option<f32>,
    /// 可选;长度必须等于自由度数(在 runtime 校验)。
    #[serde(default)]
    pub default_pose: Option<Vec<f32>>,
    pub obs: Vec<ObsEntry>,
    #[serde(default)]
    pub normalization: Option<Normalization>,
}

/// 一个观测项。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObsEntry {
    pub term: ObsTerm,
    /// 可选,默认 1;有限,≠ 0。
    #[serde(default = "one")]
    pub scale: f32,
}

/// 观测归一化(可选;长度校验在 runtime,需要 dof)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Normalization {
    pub mean: Vec<f32>,
    pub std: Vec<f32>,
}

impl PolicyManifest {
    /// 解析 toml 文本并做自身校验。
    pub fn from_toml_str(text: &str) -> Result<Self, PolicyError> {
        let manifest: Self =
            toml::from_str(text).map_err(|e| PolicyError::Manifest(e.to_string()))?;
        manifest.validate()?;
        Ok(manifest)
    }

    /// 观测总维度(给定自由度数)。
    pub fn obs_dim(&self, dof: usize) -> usize {
        self.obs.iter().map(|e| e.term.dim(dof)).sum()
    }

    /// 自身校验(不依赖身体的部分)。
    pub(crate) fn validate(&self) -> Result<(), PolicyError> {
        let bad = |what: &str| PolicyError::Manifest(what.to_owned());
        if self.control_period_ticks == 0 {
            return Err(bad("control_period_ticks 须 ≥ 1"));
        }
        if !self.action_scale.is_finite() || self.action_scale <= 0.0 {
            return Err(bad("action_scale 须有限且 > 0"));
        }
        if let Some(c) = self.action_clip {
            if !c.is_finite() || c <= 0.0 {
                return Err(bad("action_clip 须有限且 > 0"));
            }
        }
        if let Some(c) = self.obs_clip {
            if !c.is_finite() || c <= 0.0 {
                return Err(bad("obs_clip 须有限且 > 0"));
            }
        }
        if self.obs.is_empty() {
            return Err(bad("obs 不能为空"));
        }
        for e in &self.obs {
            if !e.scale.is_finite() || e.scale == 0.0 {
                return Err(bad("obs.scale 须有限且 ≠ 0"));
            }
        }
        if let Some(n) = &self.normalization {
            for s in &n.std {
                if !s.is_finite() || *s <= 0.0 {
                    return Err(bad("normalization.std 每项须有限且 > 0"));
                }
            }
        }
        Ok(())
    }
}