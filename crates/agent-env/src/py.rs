//! REQ-012 E02b-1 / E02c-1:`agent_env` Python 模块(PyO3 薄封装,`python` 特性)。
//!
//! 包装 [`crate::vec_env::VecEnv`],把 Rust 的观测/奖励原料转成 numpy 数组。
//! 环境常量全部由 Rust 侧从身体定义与 manifest 取([`crate::spec_data`]),Python
//! 训练代码不重复写常量(单一事实来源;契约检查 `quad.backends.check` 核对)。
//!
//! E02c-1 新增:自由函数 `episode_seed`/`sample_intent`/`spec_for`、常量
//! `SPAWN_HEIGHT`、`Codec`(批量编码/解码/机身系换算,逐行调 Rust 同一份代码)。
//! MuJoCo 后端只从 MuJoCo 取状态,观测/动作映射全经这里。
//!
//! GIL:`reset`/`step` 的 Rust 计算段释放 GIL(`Python::allow_threads`),多线程
//! 真并行。错误:`EnvError::NeedsReset` → `RuntimeError`,其余 → `ValueError`。

use numpy::{PyArray1, PyArray2, PyReadonlyArray2};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule};

use crate::vec_env::VecEnv as RustVecEnv;
use crate::{
    EnvConfig, EnvError, SpecData, body_frame_batch, decode_actions, encode_snapshots,
    reward_columns, sample_intent, spec_data,
};
use policy::{AgentSnapshot, PolicyCodec, PolicyError, PolicyManifest};
use types::BodyDef;
use types::glam::{Quat, Vec3};

/// `EnvError` → Python 异常:`NeedsReset` → `RuntimeError`,其余 → `ValueError`。
fn env_err(e: EnvError) -> PyErr {
    match e {
        EnvError::NeedsReset => PyRuntimeError::new_err(e.to_string()),
        other => PyValueError::new_err(other.to_string()),
    }
}

/// `PolicyError` → `ValueError`(消息带原文)。
fn policy_err(e: PolicyError) -> PyErr {
    PyValueError::new_err(e.to_string())
}

/// 把扁平缓冲按行宽切二维 numpy 数组(float32 [rows, width])。
fn arr2<'py>(py: Python<'py>, data: &[f32], width: usize) -> PyResult<Bound<'py, PyArray2<f32>>> {
    let nested: Vec<Vec<f32>> = data.chunks(width).map(|c| c.to_vec()).collect();
    PyArray2::from_vec2(py, &nested).map_err(|e| PyValueError::new_err(e.to_string()))
}

/// 取二维只读数组的形状(行, 列)。
fn dims(a: &PyReadonlyArray2<f32>) -> (usize, usize) {
    a.as_array().dim()
}

/// 摊平二维只读数组。
fn flat(a: &PyReadonlyArray2<f32>) -> Vec<f32> {
    a.as_array().iter().copied().collect()
}

/// `SpecData` → Python dict(键值与 E02b-1 `VecEnv.spec()` 不变)。
fn spec_to_dict<'py>(py: Python<'py>, s: &SpecData) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    d.set_item("body_name", s.body_name.clone())?;
    d.set_item("body_hash", s.body_hash.clone())?;
    d.set_item("dof", s.dof)?;
    d.set_item("obs_dim", s.obs_dim)?;
    d.set_item("act_dim", s.act_dim)?;
    d.set_item("tick_hz", s.tick_hz)?;
    d.set_item("control_period_ticks", s.control_period_ticks)?;
    d.set_item("episode_controls", s.episode_controls)?;
    d.set_item("fall_up_y_min", s.fall_up_y_min)?;
    d.set_item("action_scale", s.action_scale)?;
    d.set_item("action_clip", s.action_clip)?;
    d.set_item("obs_clip", s.obs_clip)?;
    d.set_item("obs_terms", s.obs_terms.clone())?;
    d.set_item("default_pose", s.default_pose.clone())?;
    d.set_item("joint_lower", s.joint_lower.clone())?;
    d.set_item("joint_upper", s.joint_upper.clone())?;
    d.set_item("kp", s.kp.clone())?;
    d.set_item("kd", s.kd.clone())?;
    d.set_item("max_torque", s.max_torque.clone())?;
    d.set_item("reward_keys", s.reward_keys.clone())?;
    Ok(d)
}

/// 解析 intent_range(3 项,每项 2 个数)。
fn parse_intent_range(intent_range: &[Vec<f32>]) -> PyResult<[[f32; 2]; 3]> {
    if intent_range.len() != 3 {
        return Err(PyValueError::new_err("intent_range 须 3 项"));
    }
    let mut ir = [[0.0f32; 2]; 3];
    for (slot, pair) in ir.iter_mut().zip(intent_range) {
        if pair.len() != 2 {
            return Err(PyValueError::new_err("intent_range 每项须 2 个数"));
        }
        *slot = [pair[0], pair[1]];
    }
    Ok(ir)
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
        let ir = parse_intent_range(&intent_range)?;
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
        Ok(Self {
            inner,
            def,
            manifest,
            episode_controls,
            fall_up_y_min,
            n_envs,
            obs_dim,
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

    /// 环境规格(全部由 Rust 侧从身体定义与 manifest 取;调 [`spec_data`])。
    fn spec<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let sd = spec_data(
            &self.def,
            &self.manifest,
            self.episode_controls,
            self.fall_up_y_min,
        )
        .map_err(policy_err)?;
        spec_to_dict(py, &sd)
    }

    /// 环境数。
    fn n_envs(&self) -> usize {
        self.n_envs
    }
}

/// 观测/动作编解码器(`agent_env.Codec`):逐行调 Rust [`PolicyCodec`],
/// MuJoCo 后端与 rapier 共用同一份观测编码、动作解码、机身系换算。
///
/// 四元数分量顺序 **x y z w**(glam 顺序)。`manifest.normalization` 必须为
/// `None`(训练环境输出未归一化观测),否则 `ValueError`。
#[pyclass(unsendable)]
pub struct Codec {
    inner: PolicyCodec,
}

#[pymethods]
impl Codec {
    /// `body_toml`/`manifest_toml` 传文件内容(不是路径)。
    #[new]
    fn new(body_toml: &str, manifest_toml: &str) -> PyResult<Self> {
        let manifest = PolicyManifest::from_toml_str(manifest_toml)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        if manifest.normalization.is_some() {
            return Err(PyValueError::new_err(
                "Codec 要求 manifest.normalization 为 None(训练环境输出未归一化观测)",
            ));
        }
        let def =
            BodyDef::from_toml_str(body_toml).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let inner = PolicyCodec::new(manifest, &def).map_err(policy_err)?;
        Ok(Self { inner })
    }

    /// 观测维度。
    #[getter]
    fn obs_dim(&self) -> usize {
        self.inner.obs_dim()
    }

    /// 自由度数。
    #[getter]
    fn dof(&self) -> usize {
        self.inner.dof()
    }

    /// 批量编码 → float32 [n, obs_dim]。各数组行数须一致;维度不符 → ValueError。
    #[allow(clippy::too_many_arguments)]
    fn encode_batch<'py>(
        &self,
        py: Python<'py>,
        root_quat_xyzw: PyReadonlyArray2<f32>,
        linvel_world: PyReadonlyArray2<f32>,
        angvel_world: PyReadonlyArray2<f32>,
        joint_pos: PyReadonlyArray2<f32>,
        joint_vel: PyReadonlyArray2<f32>,
        last_action: PyReadonlyArray2<f32>,
        intent: PyReadonlyArray2<f32>,
    ) -> PyResult<Bound<'py, PyArray2<f32>>> {
        let dof = self.inner.dof();
        let (n, qw) = dims(&root_quat_xyzw);
        if qw != 4 {
            return Err(PyValueError::new_err(format!(
                "root_quat_xyzw 列宽 {qw} ≠ 4"
            )));
        }
        for (name, arr, w) in [
            ("linvel_world", &linvel_world, 3),
            ("angvel_world", &angvel_world, 3),
            ("joint_pos", &joint_pos, dof),
            ("joint_vel", &joint_vel, dof),
            ("last_action", &last_action, dof),
            ("intent", &intent, 3),
        ] {
            let (r, c) = dims(arr);
            if r != n || c != w {
                return Err(PyValueError::new_err(format!(
                    "{name} 形状 ({r},{c}) ≠ ({n},{w})"
                )));
            }
        }
        let q = flat(&root_quat_xyzw);
        let lv = flat(&linvel_world);
        let av = flat(&angvel_world);
        let jp = flat(&joint_pos);
        let jv = flat(&joint_vel);
        let la = flat(&last_action);
        let it = flat(&intent);
        let mut snaps = Vec::with_capacity(n);
        for i in 0..n {
            snaps.push(AgentSnapshot {
                root_rotation: Quat::from_xyzw(q[i * 4], q[i * 4 + 1], q[i * 4 + 2], q[i * 4 + 3]),
                root_linvel: Vec3::new(lv[i * 3], lv[i * 3 + 1], lv[i * 3 + 2]),
                root_angvel: Vec3::new(av[i * 3], av[i * 3 + 1], av[i * 3 + 2]),
                joint_pos: jp[i * dof..(i + 1) * dof].to_vec(),
                joint_vel: jv[i * dof..(i + 1) * dof].to_vec(),
                last_action: la[i * dof..(i + 1) * dof].to_vec(),
                intent: [it[i * 3], it[i * 3 + 1], it[i * 3 + 2]],
            });
        }
        let obs = encode_snapshots(&self.inner, &snaps).map_err(policy_err)?;
        arr2(py, &obs, self.inner.obs_dim())
    }

    /// 批量解码 actions [n, dof] → (raw [n,dof], targets [n,dof], clamped uint32 [n])。
    #[allow(clippy::type_complexity)]
    fn decode_batch<'py>(
        &self,
        py: Python<'py>,
        actions: PyReadonlyArray2<f32>,
    ) -> PyResult<(
        Bound<'py, PyArray2<f32>>,
        Bound<'py, PyArray2<f32>>,
        Bound<'py, PyArray1<u32>>,
    )> {
        let dof = self.inner.dof();
        let (n, c) = dims(&actions);
        if c != dof {
            return Err(PyValueError::new_err(format!(
                "actions 列宽 {c} ≠ dof {dof}"
            )));
        }
        let acts = flat(&actions);
        let d = decode_actions(&self.inner, &acts, n).map_err(policy_err)?;
        let raw = arr2(py, &d.raw, dof)?;
        let targets = arr2(py, &d.targets, dof)?;
        let clamped = PyArray1::from_vec(py, d.clamped);
        Ok((raw, targets, clamped))
    }

    /// 批量机身系换算 → dict:base_linvel_body [n,3]、base_angvel_body [n,3]、
    /// projected_gravity [n,3]、up_y [n,1]。四元数 x y z w。
    fn body_frame_batch<'py>(
        &self,
        py: Python<'py>,
        root_quat_xyzw: PyReadonlyArray2<f32>,
        linvel_world: PyReadonlyArray2<f32>,
        angvel_world: PyReadonlyArray2<f32>,
    ) -> PyResult<Bound<'py, PyDict>> {
        let (n, qw) = dims(&root_quat_xyzw);
        if qw != 4 {
            return Err(PyValueError::new_err(format!(
                "root_quat_xyzw 列宽 {qw} ≠ 4"
            )));
        }
        for (name, arr) in [
            ("linvel_world", &linvel_world),
            ("angvel_world", &angvel_world),
        ] {
            let (r, c) = dims(arr);
            if r != n || c != 3 {
                return Err(PyValueError::new_err(format!(
                    "{name} 形状 ({r},{c}) ≠ ({n},3)"
                )));
            }
        }
        let bf = body_frame_batch(
            &flat(&root_quat_xyzw),
            &flat(&linvel_world),
            &flat(&angvel_world),
            n,
        );
        let d = PyDict::new(py);
        d.set_item("base_linvel_body", arr2(py, &bf.linvel, 3)?)?;
        d.set_item("base_angvel_body", arr2(py, &bf.angvel, 3)?)?;
        d.set_item("projected_gravity", arr2(py, &bf.projected_gravity, 3)?)?;
        d.set_item("up_y", arr2(py, &bf.up_y, 1)?)?;
        Ok(d)
    }
}

/// 环境 env_id 第 episode 个回合用的种子(与 Rust `vec_env::episode_seed` 同一份)。
#[pyfunction]
fn episode_seed(base_seed: u64, env_id: u64, episode: u64) -> u64 {
    crate::vec_env::episode_seed(base_seed, env_id, episode)
}

/// 采样 intent(3 个;与 Rust `sample_intent` 同一份)。intent_range 3 项各 2 数。
#[pyfunction]
#[pyo3(name = "sample_intent")]
fn sample_intent_py(seed: u64, intent_range: Vec<Vec<f32>>) -> PyResult<Vec<f32>> {
    let ir = parse_intent_range(&intent_range)?;
    Ok(sample_intent(seed, &ir).to_vec())
}

/// 环境规格 dict(与 `VecEnv.spec()` 同一段 Rust [`spec_data`])。
#[pyfunction]
fn spec_for<'py>(
    py: Python<'py>,
    body_toml: &str,
    manifest_toml: &str,
    episode_controls: u32,
    fall_up_y_min: f32,
) -> PyResult<Bound<'py, PyDict>> {
    let manifest = PolicyManifest::from_toml_str(manifest_toml)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    let def =
        BodyDef::from_toml_str(body_toml).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let sd = spec_data(&def, &manifest, episode_controls, fall_up_y_min).map_err(policy_err)?;
    spec_to_dict(py, &sd)
}

/// Python 模块注册(maturin cdylib 入口)。
#[pymodule]
fn agent_env(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<VecEnv>()?;
    m.add_class::<Codec>()?;
    m.add_function(wrap_pyfunction!(episode_seed, m)?)?;
    m.add_function(wrap_pyfunction!(sample_intent_py, m)?)?;
    m.add_function(wrap_pyfunction!(spec_for, m)?)?;
    m.add("SPAWN_HEIGHT", crate::SPAWN_HEIGHT)?;
    Ok(())
}
