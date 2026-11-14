//! 四足步态纯函数(REQ-012 B02a,§2.1):从 `demos/agents/quad/src/lib.rs`
//! 第 381–469 行移植,数值路径逐式相同(对角小跑、1 秒渐入、支撑相匀速
//! 后扫、摆动相 smoothstep + 正弦抬脚、膝朝后的逆解)。不"改进"公式。
//!
//! 关节下标 = `leg * 3 + {0: lat, 1: hip_pitch, 2: knee}`,腿序 FL、FR、
//! RL、RR(与 `assets/bodies/quad.body.toml` 的 joints 顺序一致,T5 钉住)。
//! 控制者在驱动层,不进确定性内核(§2.3);动作经 `Session` 写输入日志。

/// 四足自由度 = 4 腿 × 3 关节。
pub const QUAD_DOF: usize = 12;
/// 大腿、小腿长(Go2 真实几何,quad demo 定值)。
pub const QUAD_L: f32 = 0.213;
/// 站高(足端目标离髋高度)。
pub const QUAD_FOOT_HEIGHT: f32 = 0.28;
/// 物理/控制步长(全世界 60 Hz,D4)。
pub const SIM_DT: f32 = 1.0 / 60.0;

/// 开环步态参数(足端轨迹口径;demo `Gait` 同名同义)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gait {
    /// 步态频率(Hz)。
    pub freq: f32,
    /// 步长(单步前后位移,m)。
    pub stride: f32,
    /// 抬脚高度(m)。
    pub lift: f32,
    /// 支撑相占比(0..1),其余为摆动相。
    pub duty: f32,
    /// 足端目标离髋高度(站高,m)。
    pub h: f32,
    /// 足端目标前后中心(相对髋,m)。
    pub x0: f32,
}

impl Gait {
    /// §13 `gait` 代表组:freq 3.0、stride 0.15、lift 0.06、duty 0.5、
    /// h 0.28、x0 0.0。
    pub const DEFAULT: Gait = Gait {
        freq: 3.0,
        stride: 0.15,
        lift: 0.06,
        duty: 0.5,
        h: QUAD_FOOT_HEIGHT,
        x0: 0.0,
    };

    /// 腿 `leg` 在行走时刻 `t` 的足端目标(相对髋,x 前 / y 下为负)。
    /// 腿序 [FL, FR, RL, RR],对角小跑:FL/RR 相位 0,FR/RL 相位 0.5;
    /// 步幅/抬脚 1 s 渐入。
    pub fn foot_target(&self, leg: usize, t: f32) -> (f32, f32) {
        let ph0 = if leg == 0 || leg == 3 { 0.0 } else { 0.5 };
        let ramp = (t / 1.0).min(1.0); // 步幅/抬脚 1 s 渐入
        let p = (self.freq * t + ph0).rem_euclid(1.0);
        let stride = self.stride * ramp;
        let lift = self.lift * ramp;
        if p < self.duty {
            // 支撑相:匀速后扫。
            let u = p / self.duty;
            (self.x0 + stride / 2.0 - stride * u, -self.h)
        } else {
            // 摆动相:smoothstep 前摆 + 正弦抬脚。
            let u = (p - self.duty) / (1.0 - self.duty);
            let sm = u * u * (3.0 - 2.0 * u);
            (
                self.x0 - stride / 2.0 + stride * sm,
                -self.h + lift * (std::f32::consts::PI * u).sin(),
            )
        }
    }

    /// 站姿 / 摆动相位无关的初始目标:直腿降下来后取 (x0, −h)。
    pub fn stand_foot(&self) -> (f32, f32) {
        (self.x0, -self.h)
    }
}

/// 足端(相对髋)→ (hip_pitch, knee)。膝朝后(Go2 口径)。
pub fn ik(dx: f32, dy: f32) -> (f32, f32) {
    let d2 = dx * dx + dy * dy;
    let c = ((d2 - 2.0 * QUAD_L * QUAD_L) / (2.0 * QUAD_L * QUAD_L)).clamp(-1.0, 1.0);
    let k = c.acos();
    let phi = dx.atan2(-dy);
    (phi - k / 2.0, k)
}

/// 行走时刻 `t`(起步后秒数)的全部关节目标(dof 顺序)。
pub fn trot_targets(gait: &Gait, t: f32) -> [f32; QUAD_DOF] {
    let mut out = [0.0f32; QUAD_DOF];
    for leg in 0..4 {
        let (dx, dy) = gait.foot_target(leg, t);
        let (a, k) = ik(dx, dy);
        out[leg * 3] = 0.0;
        out[leg * 3 + 1] = a;
        out[leg * 3 + 2] = k;
    }
    out
}

/// 站立姿态的全部关节目标。
pub fn stand_targets(gait: &Gait) -> [f32; QUAD_DOF] {
    let (dx, dy) = gait.stand_foot();
    let (a, k) = ik(dx, dy);
    let mut out = [0.0f32; QUAD_DOF];
    for leg in 0..4 {
        out[leg * 3 + 1] = a;
        out[leg * 3 + 2] = k;
    }
    out
}
