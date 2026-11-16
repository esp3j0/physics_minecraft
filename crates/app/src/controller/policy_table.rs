//! 策略登记表(REQ-012 D01b-1 §1.2):PolicyKey → (名字, Policy)。
//! 不进检查点;启动和恢复时都从配置重新加载。

use std::collections::BTreeMap;
use std::path::Path;

use super::PolicyKey;

/// 一条已加载的策略。
#[derive(Debug)]
pub struct PolicyEntry {
    pub name: String,
    pub policy: policy::Policy,
    /// 校验时对着的身体的自由度数(D01b-2 裁定:startup 挂载用)。
    pub dof: usize,
}

/// 策略元数据(D01b-2 裁定方案 2):startup 挂载只读这份,不借整张表。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PolicyMeta {
    pub dof: usize,
    pub control_period_ticks: u32,
}

/// PolicyKey → 元数据。`PolicyTable::meta()` 产出,交给 `startup.poll`。
pub type PolicyMetaTable = BTreeMap<PolicyKey, PolicyMeta>;

/// 策略加载失败原因(驱动层再补"第几条"信息;PolicyError 原文保留)。
#[derive(Debug, thiserror::Error)]
pub enum PolicyLoadError {
    /// 同名策略已加载且身体哈希不同。
    #[error("策略 {name} 已加载且身体哈希不同")]
    Conflict { name: String },
    /// 策略包加载/校验失败。
    #[error("策略 {name}:{source}")]
    Policy {
        name: String,
        #[source]
        source: policy::PolicyError,
    },
}

/// 已加载的策略:PolicyKey → (名字, Policy)。
#[derive(Debug, Default)]
pub struct PolicyTable {
    entries: BTreeMap<PolicyKey, PolicyEntry>,
}

impl PolicyTable {
    /// 加载 `<policies_dir>/<name>`,对着 `body` 校验。同名重复加载:
    /// 身体哈希相同则复用,不同则 Err。
    pub fn load(
        &mut self,
        policies_dir: &Path,
        name: &str,
        body: &types::BodyDef,
    ) -> Result<PolicyKey, PolicyLoadError> {
        let key = PolicyKey::of(name);
        if let Some(entry) = self.entries.get(&key) {
            if entry.policy.manifest().body_hash == body.hash().hex() {
                return Ok(key);
            }
            return Err(PolicyLoadError::Conflict {
                name: name.to_owned(),
            });
        }
        let policy = policy::Policy::load(&policies_dir.join(name), body).map_err(|source| {
            PolicyLoadError::Policy {
                name: name.to_owned(),
                source,
            }
        })?;
        self.entries.insert(
            key,
            PolicyEntry {
                name: name.to_owned(),
                policy,
                dof: body.dof_count(),
            },
        );
        Ok(key)
    }

    /// 只读元数据快照(D01b-2 裁定方案 2):交出表所有权给工作线程前,
    /// startup 挂载需要的 dof / 控制周期先抽出来。
    pub fn meta(&self) -> PolicyMetaTable {
        self.entries
            .iter()
            .map(|(&key, entry)| {
                (
                    key,
                    PolicyMeta {
                        dof: entry.dof,
                        control_period_ticks: entry.policy.manifest().control_period_ticks,
                    },
                )
            })
            .collect()
    }

    /// 只读取条目(启动逻辑取 dof / 控制周期用)。
    pub fn get(&self, key: PolicyKey) -> Option<&PolicyEntry> {
        self.entries.get(&key)
    }

    pub fn get_mut(&mut self, key: PolicyKey) -> Option<&mut PolicyEntry> {
        self.entries.get_mut(&key)
    }

    pub fn contains(&self, key: PolicyKey) -> bool {
        self.entries.contains_key(&key)
    }
}
