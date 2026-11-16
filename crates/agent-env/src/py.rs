//! REQ-012 E02b-1:`agent_env` Python 模块(PyO3 薄封装,`python` 特性)。
//!
//! 包装 [`crate::vec_env::VecEnv`],把 Rust 的观测/奖励原料转成 numpy 数组。
//! 环境常量全部由 Rust 侧从身体定义与 manifest 取([`VecEnv::spec`]),Python
//! 训练代码不重复写常量(单一事实来源;契约检查 `quad.backends.check` 核对)。
//!
//! GIL:`reset`/`step` 的 Rust 计算段释放 GIL(`Python::allow_threads`),多线程
//! 真并行。错误:`EnvError::NeedsReset` → `RuntimeError`,其余 → `ValueError`。

use numpy::{PyArray1, PyArray2, PyReadonlyArray2};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule};

use crate::vec_env::VecEnv as RustVecEnv;
use crate::{EnvConfig, EnvError, reward_columns, reward_key_widths};
use policy::{ObsTerm, PolicyManifest};
use types::BodyDef;

/// `EnvError` → Python 异常:`NeedsReset` → `RuntimeError`,其余 → `ValueError`。
fn env_err(e: EnvError) -> PyErr {
    match e {
        EnvError::NeedsReset => PyRuntimeError::new_err(e.to_string()),
        other => PyValueError::new_err(other.to_string()),
    }
}

/// 观测项名(manifest 里的 snake_case 写法)。
fn term_name(t: ObsTerm) -> &'static str {
    match t {
        ObsTerm::BaseLinVel => "base_lin_vel",
        ObsTerm::BaseAngVel => "base_ang_vel",
        ObsTerm::ProjectedGravity => "projected_gravity",
        ObsTerm::JointPos => "joint_pos",
        ObsTerm::JointVel => "joint_vel",
        ObsTerm::LastAction => "last_action",
        ObsTerm::Intent => "intent",
    }
}

/// 把扁平缓冲按行宽切二维 numpy 数组(float32 [rows, width])。
fn arr2<'py>(py: Python<'py>, data: &[f32], width: usize) -> PyResult<Bound<'py, PyArray2<f32>>> {
    let nested: Vec<Vec<f32>> = data.chunks(width).map(|c| c.to_vec()).collect();
    PyArray2::from_vec2(py, &nested).map_err(|e| PyValueError::new_err(e.to_string()))
}

/// 并行多实例训练环境(`agent_env.VecEnv`)。
///
/// `unsendable`:内部 [`RustVecEnv`] 含 mpsc 通道(非 `Sync`),只能在创建它的
/// Python 线程访问(GIL 下主线程串行调用,符合用法)。
#[pyclass(unsendable)]
pub struct VecEnv {
    inner: RustVecEnv,
    def: BodyDef,
    manifest: PolicyManifest,
    episode_controls: u32,
    fall_up_y_min: f32,
    n_envs: usize,
    obs_dim: usize,
    act_dim: usize,
}

#[pymethods]
impl VecEnv {
    /// `body_toml`/`manifest_toml` 传文件内容(不是路径)。
    #[new]
    #[pyo3(signature = (
        body_toml, manifest_toml, n_envs, n_threads, episode_controls,
        settle_ticks = 120, fall_up_y_min = 0.5,
        intent_range = vec![vec![0.0, 0.0], vec![0.0, 0.0], vec![0.0, 0.0]],
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        body_toml: &str,
        manifest_toml: &str,
        n_envs: usize,
        n_threads: usize,
        episode_controls: u32,
        settle_ticks: u32,
        fall_up_y_min: f32,
        intent_range: Vec<Vec<f32>>,
    ) -> PyResult<Self> {
        let manifest = PolicyManifest::from_toml_str(manifest_toml)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let def =
            BodyDef::from_toml_str(body_toml).map_err(|e| PyValueError::new_err(e.to_string()))?;
        if intent_range.len() != 3 {
            return Err(PyValueError::new_err("intent_range 须 3 项"));
        }
        let mut ir = [[0.0f32; 2]; 3];
        for (slot, pair) in ir.iter_mut().zip(&intent_range) {
            if pair.len() != 2 {
                return Err(PyValueError::new_err("intent_range 每项须 2 个数"));
            }
            *slot = [pair[0], pair[1]];
        }
        let cfg = EnvConfig {
            body_toml: body_toml.to_string(),
            manifest: manifest.clone(),
            episode_controls,
            settle_ticks,
            fall_up_y_min,
            intent_range: ir,
        };
        let inner = RustVecEnv::new(cfg, n_envs, n_threads).map_err(env_err)?;
        let obs_dim = inner.obs_dim();
        let act_dim = inner.act_dim();
        Ok(Self {
            inner,
            def,
            manifest,
            episode_controls,
            fall_up_y_min,
            n_envs,
            obs_dim,
            act_dim,
        })
    }

    /// 重置全部环境,返回 obs float32 [n_envs, obs_dim]。
    fn reset<'py>(&mut self, py: Python<'py>, seed: u64) -> PyResult<Bound<'py, PyArray2<f32>>> {
        let ve = &mut self.inner;
        let obs = py.allow_threads(|| ve.reset(seed)).map_err(env_err)?;
        arr2(py, &obs, self.obs_dim)
    }

    /// 一个控制周期。actions float32 [n_envs, act_dim];返回
    /// (obs, reward_inputs, terminated, truncated)。reward_inputs 为
    /// dict[str, float32 [n_envs, k]];terminated/truncated 为 bool [n_envs]。
    #[allow(clippy::type_complexity)]
    fn step<'py>(
        &mut self,
        py: Python<'py>,
        actions: PyReadonlyArray2<f32>,
    ) -> PyResult<(
        Bound<'py, PyArray2<f32>>,
        Bound<'py, PyDict>,
        Bound<'py, PyArray1<bool>>,
        Bound<'py, PyArray1<bool>>,
    )> {
        let acts: Vec<f32> = actions.as_array().iter().copied().collect();
        let ve = &mut self.inner;
        let vs = py.allow_threads(|| ve.step(&acts)).map_err(env_err)?;
        let obs = arr2(py, &vs.obs, self.obs_dim)?;
        let dict = PyDict::new(py);
        for (key, k, col) in reward_columns(&vs.reward) {
            dict.set_item(key, arr2(py, &col, k)?)?;
        }
        let terminated = PyArray1::from_vec(py, vs.terminated);
        let truncated = PyArray1::from_vec(py, vs.truncated);
        Ok((obs, dict, terminated, truncated))
    }

    /// 环境规格(全部由 Rust 侧从身体定义与 manifest 取)。
    fn spec<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let d = PyDict::new(py);
        let dof = self.act_dim;
        let dof_joints: Vec<&types::JointDef> = self.def.dof_joints().collect();
        let default_pose: Vec<f32> = self.manifest.default_pose.clone().unwrap_or_else(|| {
            dof_joints
                .iter()
                .map(|j| j.kind.params().default_angle.unwrap_or(0.0))
                .collect()
        });
        let lower: Vec<Option<f32>> = dof_joints
            .iter()
            .map(|j| j.kind.params().limits.map(|l| l.lower))
            .collect();
        let upper: Vec<Option<f32>> = dof_joints
            .iter()
            .map(|j| j.kind.params().limits.map(|l| l.upper))
            .collect();
        let kp: Vec<Option<f32>> = dof_joints.iter().map(|j| j.kind.params().kp).collect();
        let kd: Vec<Option<f32>> = dof_joints.iter().map(|j| j.kind.params().kd).collect();
        let max_torque: Vec<Option<f32>> = dof_joints
            .iter()
            .map(|j| j.kind.params().max_torque)
            .collect();
        let obs_terms: Vec<(&'static str, f32)> = self
            .manifest
            .obs
            .iter()
            .map(|e| (term_name(e.term), e.scale))
            .collect();

        d.set_item("body_name", self.def.name.clone())?;
        d.set_item("body_hash", self.def.hash().hex())?;
        d.set_item("dof", dof)?;
        d.set_item("obs_dim", self.obs_dim)?;
        d.set_item("act_dim", self.act_dim)?;
        d.set_item("tick_hz", 60u32)?;
        d.set_item("control_period_ticks", self.manifest.control_period_ticks)?;
        d.set_item("episode_controls", self.episode_controls)?;
        d.set_item("fall_up_y_min", self.fall_up_y_min)?;
        d.set_item("action_scale", self.manifest.action_scale)?;
        d.set_item("action_clip", self.manifest.action_clip)?;
        d.set_item("obs_clip", self.manifest.obs_clip)?;
        d.set_item("obs_terms", obs_terms)?;
        d.set_item("default_pose", default_pose)?;
        d.set_item("joint_lower", lower)?;
        d.set_item("joint_upper", upper)?;
        d.set_item("kp", kp)?;
        d.set_item("kd", kd)?;
        d.set_item("max_torque", max_torque)?;
        d.set_item("reward_keys", reward_key_widths(dof))?;
        Ok(d)
    }

    /// 环境数。
    fn n_envs(&self) -> usize {
        self.n_envs
    }
}

/// Python 模块注册(maturin cdylib 入口)。
#[pymodule]
fn agent_env(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<VecEnv>()?;
    Ok(())
}
