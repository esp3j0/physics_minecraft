//! REQ-012 D01a:策略包加载、校验、推理、动作 → 关节目标(任务书 1.4)。
//! tract 用法照 demo `infer`:加载时 into_optimized → into_runnable,
//! `act` 里不重复加载。

use std::path::Path;

use tract_onnx::prelude::*;

use crate::codec::PolicyCodec;
use crate::error::PolicyError;
use crate::manifest::PolicyManifest;
use crate::observe::AgentSnapshot;

type RunnableModel = tract_onnx::tract_core::model::RunnableModel<
    tract_onnx::prelude::TypedFact,
    Box<dyn tract_onnx::prelude::TypedOp>,
>;

/// 加载并校验好的策略包。
pub struct Policy {
    codec: PolicyCodec,
    model: std::sync::Arc<RunnableModel>,
    /// 复用的观测缓冲区。
    obs_buf: Vec<f32>,
}

/// 一次推理的结果。
#[derive(Debug, Clone, PartialEq)]
pub struct Action {
    /// 裁剪后的原始动作(下一次观测里的 last_action)。
    pub raw: Vec<f32>,
    /// 关节目标角,已夹进关节限位。长度 dof。
    pub targets: Vec<f32>,
    /// 被关节限位夹住的自由度个数(遥测用)。
    pub clamped: usize,
}

impl std::fmt::Debug for Policy {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Policy")
            .field("manifest", self.manifest())
            .field("dof", &self.dof())
            .field("obs_dim", &self.obs_dim())
            .finish_non_exhaustive()
    }
}

/// 取固定二维形状;不是固定二维 → ModelShape。
fn concrete_rank2(fact: &InferenceFact, which: &str) -> Result<[usize; 2], PolicyError> {
    let concrete = fact
        .shape
        .as_concrete_finite()
        .map_err(|e| PolicyError::ModelShape(format!("{which}形状:{e}")))?;
    match concrete {
        Some(s) if s.len() == 2 => Ok([s[0], s[1]]),
        _ => Err(PolicyError::ModelShape(format!(
            "{which}形状不是固定二维:{:?}",
            fact.shape
        ))),
    }
}

impl Policy {
    /// 读 `<dir>/policy.toml` 和 `<dir>/policy.onnx`,对着 `body` 校验。
    /// 任何一项不符都返回 Err,不返回半成品。
    pub fn load(dir: &Path, body: &types::BodyDef) -> Result<Self, PolicyError> {
        let toml_path = dir.join("policy.toml");
        let text = std::fs::read_to_string(&toml_path).map_err(|e| PolicyError::Io {
            path: toml_path.display().to_string(),
            reason: e.to_string(),
        })?;
        let manifest = PolicyManifest::from_toml_str(&text)?;
        let onnx_path = dir.join("policy.onnx");
        let bytes = std::fs::read(&onnx_path).map_err(|e| PolicyError::Io {
            path: onnx_path.display().to_string(),
            reason: e.to_string(),
        })?;
        Self::from_parts(manifest, &bytes, body)
    }

    /// 同 [`Self::load`],但内容从内存给(测试和以后的打包用)。
    /// 校验顺序(先报先停):manifest 自身 → 哈希 → dof → default_pose →
    /// normalization → ONNX 解析 → 输入输出个数 → 输入形状 → 输出形状。
    pub fn from_parts(
        manifest: PolicyManifest,
        onnx_bytes: &[u8],
        body: &types::BodyDef,
    ) -> Result<Self, PolicyError> {
        // 1–5. 与模型无关的校验与编解码器(顺序与报错类型保持现状)。
        let codec = PolicyCodec::new(manifest, body)?;
        let obs_dim = codec.obs_dim();
        let dof = codec.dof();
        // 6. ONNX 解析。
        let model = tract_onnx::onnx()
            .model_for_read(&mut &onnx_bytes[..])
            .map_err(|e| PolicyError::Onnx(e.to_string()))?;
        // 7. 恰好一个输入、一个输出。
        if model.inputs.len() != 1 || model.outputs.len() != 1 {
            return Err(PolicyError::ModelShape(format!(
                "输入 {} 个、输出 {} 个,应各恰好 1 个",
                model.inputs.len(),
                model.outputs.len()
            )));
        }
        // 8. 输入形状 [1, obs_dim]。
        let in_fact = model
            .input_fact(0)
            .map_err(|e| PolicyError::Onnx(e.to_string()))?;
        let in_shape = concrete_rank2(in_fact, "输入")?;
        if in_shape[0] != 1 {
            return Err(PolicyError::ModelShape(format!(
                "输入第一维 {} ≠ 1",
                in_shape[0]
            )));
        }
        if in_shape[1] != obs_dim {
            return Err(PolicyError::ObsDimMismatch {
                manifest: obs_dim,
                model: in_shape[1],
            });
        }
        // 优化(输出形状在分析后才定形,第 9 步在优化后的模型上查)。
        let typed = model
            .with_input_fact(0, f32::fact([1, obs_dim]).into())
            .map_err(|e| PolicyError::Onnx(e.to_string()))?
            .into_optimized()
            .map_err(|e| PolicyError::Onnx(e.to_string()))?;
        // 9. 输出形状 [1, dof]。
        let out_fact = typed
            .output_fact(0)
            .map_err(|e| PolicyError::Onnx(e.to_string()))?;
        let out_shape = match out_fact.shape.as_concrete() {
            Some(s) if s.len() == 2 => [s[0], s[1]],
            _ => {
                return Err(PolicyError::ModelShape(format!(
                    "输出形状不是固定二维:{:?}",
                    out_fact.shape
                )));
            }
        };
        if out_shape[0] != 1 {
            return Err(PolicyError::ModelShape(format!(
                "输出第一维 {} ≠ 1",
                out_shape[0]
            )));
        }
        if out_shape[1] != dof {
            return Err(PolicyError::ActionDimMismatch {
                body: dof,
                model: out_shape[1],
            });
        }
        let model = typed
            .into_runnable()
            .map_err(|e| PolicyError::Onnx(e.to_string()))?;
        Ok(Self {
            codec,
            model,
            obs_buf: Vec::with_capacity(obs_dim),
        })
    }

    pub fn manifest(&self) -> &PolicyManifest {
        self.codec.manifest()
    }

    pub fn dof(&self) -> usize {
        self.codec.dof()
    }

    pub fn obs_dim(&self) -> usize {
        self.codec.obs_dim()
    }

    /// 观测 → 归一化 → 裁剪 → 推理 → 动作。
    /// 同一个 Policy、同样的 snapshot,结果逐位相同。
    pub fn act(&mut self, snapshot: &AgentSnapshot) -> Result<Action, PolicyError> {
        // 1–4. 观测 → 归一化 → 裁剪 → 非有限检查(与训练环境共用 codec)。
        self.codec.encode(snapshot, &mut self.obs_buf)?;
        // 5. 推理。
        let arr = tract_onnx::tract_core::ndarray::Array2::<f32>::from_shape_vec(
            (1, self.obs_dim()),
            self.obs_buf.clone(),
        )
        .map_err(|e| PolicyError::Inference(e.to_string()))?;
        let result = self
            .model
            .run(tvec![arr.into_tvalue()])
            .map_err(|e| PolicyError::Inference(e.to_string()))?;
        let out = result
            .into_iter()
            .next()
            .ok_or_else(|| PolicyError::Inference("模型没有输出".to_owned()))?
            .into_tensor()
            .into_plain_array::<f32>()
            .map_err(|e| PolicyError::Inference(e.to_string()))?
            .into_raw_vec_and_offset()
            .0;
        // 6–8. 非有限检查 → 动作裁剪 → 目标(与训练环境共用 codec)。
        self.codec.decode(&out)
    }
}
