//! REQ-012 D01a:策略包错误类型(任务书 1.5)。

/// 策略包加载 / 观测 / 推理的错误。
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PolicyError {
    #[error("读取 {path} 失败:{reason}")]
    Io { path: String, reason: String },
    #[error("policy.toml 非法:{0}")]
    Manifest(String),
    #[error("身体哈希不符:策略包 {expected},身体 {got}")]
    BodyHashMismatch { expected: String, got: String },
    #[error("身体没有可动关节")]
    NoDof,
    #[error("default_pose 长度 {got},应为 {expected}")]
    DefaultPoseLen { expected: usize, got: usize },
    #[error("normalization 长度 {got},应为 {expected}")]
    NormalizationLen { expected: usize, got: usize },
    #[error("ONNX 加载失败:{0}")]
    Onnx(String),
    #[error("模型输入输出个数或形状不符合约定:{0}")]
    ModelShape(String),
    #[error("观测维度不符:策略包 {manifest},模型 {model}")]
    ObsDimMismatch { manifest: usize, model: usize },
    #[error("动作维度不符:身体 {body},模型 {model}")]
    ActionDimMismatch { body: usize, model: usize },
    #[error("状态数组长度 {got},应为 {expected}")]
    SnapshotDof { expected: usize, got: usize },
    #[error("观测含非有限数")]
    NonFiniteObservation,
    #[error("推理失败:{0}")]
    Inference(String),
    #[error("模型输出含非有限数")]
    NonFiniteOutput,
}
