//! launch —— 发射(W02,REQ-011 §3.2):形状表、发射请求组装、蓄力状态机、
//! 预设速度调节。全部纯函数,零 macroquad;render.rs 只做按键/滚轮到纯函数
//! 的接线。
//!
//! 坐标约定(同 [`super::prefab`]):局部 u = 右、v = 上、w = 前;yaw 的
//! 约定沿 viewer 相机(`forward = (cos yaw·cos pitch, sin pitch, sin yaw·
//! cos pitch)`),经 [`super::prefab::orientation_from_yaw`] 取整到 ±x / ±z。
//!
//! **出生位置**(§3.2,W02-2 返修澄清):`translation = cam_pos + d̂·d`
//! 是**连续值,不取整**——`d = 2 − min(0, min_k(o_k · d̂))`,其中 `o_k`
//! 为各格偏移的世界向量、`d̂` 为视线单位方向。单格 o = 0 → d = 2;复合体
//! 中若有格的偏移与 d̂ 反向(点积 < 0),d 被抬高,保证离相机最近那格的
//! 格心沿视线也在 2 m 外,且轨迹严格穿过准星。实心判定按各格格心
//! `c_k = translation + o_k` 的 `floor(c_k)` 查块,非空气拒绝(None 放行)。
//!
//! **组装**(§3.2):单格 `EntityDesc { block: 材料, ..default }`(半边长
//! 0.5,与 B 键同型);复合体 `EntityDesc::compound(COMPOUND_HALF_SIZE,
//! shape)`(每格同一材料),与脚本 `spawn.shape` 路径完全同型(AC-W4)。

use types::glam::Vec3;
use types::{CompoundShape, EntityDesc, EntityState, ShapeCell};

use super::Material;
use super::prefab::orientation_from_yaw;
use crate::script::{COMPOUND_HALF_SIZE, WorldQuery};

/// 发射速度下限(§3.2;点按预设与蓄力起点)。
pub const SPEED_MIN: f32 = 5.0;
/// 发射速度上限(§3.2;REQ-010 §5.8.5:不开 CCD 时单拍越格的变形界)。
pub const SPEED_MAX: f32 = 55.0;
/// 预设速度滚轮步长(§3.2)。
pub const SPEED_STEP: f32 = 5.0;
/// 预设速度默认值(§3.2)。
pub const SPEED_PRESET_DEFAULT: f32 = 20.0;
/// 蓄力进入阈值(按住秒数;不足 = 点按)。
pub const CHARGE_THRESHOLD: f32 = 0.25;
/// 蓄力增长斜率(m/s²;2 s 从 5 涨满 55)。
pub const CHARGE_RATE: f32 = 25.0;
/// 出生距离基准(米;最近格格心沿视线至少这么远)。
pub const SPAWN_CLEARANCE: f32 = 2.0;

/// 发射形状(REQ-011 §3.2 表;E 循环序)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaunchShape {
    /// 单格 (0,0,0)。
    Single,
    /// 6 格条·纵:w = 0..5(轴向撞击,punch 场景)。
    BarLongitudinal,
    /// 6 格条·横:u = 0..5(侧向撞击、击退)。
    BarCross,
    /// 2×2 板:u = 0..1、v = 0..1(面撞击)。
    Plate2x2,
    /// 3×3×3 块:u, v, w = 0..2(重物破墙)。
    Block3x3x3,
}

impl LaunchShape {
    /// `E` 循环的下一个形状。
    #[must_use]
    pub fn next(self) -> Self {
        match self {
            Self::Single => Self::BarLongitudinal,
            Self::BarLongitudinal => Self::BarCross,
            Self::BarCross => Self::Plate2x2,
            Self::Plate2x2 => Self::Block3x3x3,
            Self::Block3x3x3 => Self::Single,
        }
    }

    /// ASCII 名(HUD / 帮助)。
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Single => "single",
            Self::BarLongitudinal => "bar-long",
            Self::BarCross => "bar-wide",
            Self::Plate2x2 => "plate 2x2",
            Self::Block3x3x3 => "cube 3x3x3",
        }
    }

    /// 局部 (u, v, w) 偏移表(§3.2 表;原点格 [0,0,0] 恒在)。
    #[must_use]
    pub fn local_cells(self) -> Vec<[i32; 3]> {
        match self {
            Self::Single => vec![[0, 0, 0]],
            Self::BarLongitudinal => (0..6).map(|w| [0, 0, w]).collect(),
            Self::BarCross => (0..6).map(|u| [u, 0, 0]).collect(),
            Self::Plate2x2 => {
                let mut v = Vec::new();
                for u in 0..=1 {
                    for vv in 0..=1 {
                        v.push([u, vv, 0]);
                    }
                }
                v
            }
            Self::Block3x3x3 => {
                let mut v = Vec::new();
                for u in 0..=2 {
                    for vv in 0..=2 {
                        for w in 0..=2 {
                            v.push([u, vv, w]);
                        }
                    }
                }
                v
            }
        }
    }

    /// 偏移换到世界轴(u/v/w → x/z/y;经 yaw 取整,同预制结构)。
    /// 返回世界偏移(相对原点格)。
    #[must_use]
    pub fn world_cells(self, yaw_deg: f32) -> Vec<[i32; 3]> {
        let o = orientation_from_yaw(yaw_deg);
        let (du_x, du_z) = o.u;
        let (dw_x, dw_z) = o.w;
        self.local_cells()
            .into_iter()
            .map(|[u, v, w]| [u * du_x + w * dw_x, v, u * du_z + w * dw_z])
            .collect()
    }
}

/// 发射请求(§3.2;AC-W3 / AC-W4):
/// - `translation = cam_pos + d̂·d`(**连续值不取整**,W02-2 返修:
///   `d = 2 − min(0, min_k(o_k · d̂))`,`o_k` = 各格世界偏移;取整会让轨迹
///   平行错开视线、破坏"最近格 ≥ 2 m"保证并造出齐平对齐干扰);
/// - `linvel = d̂ × speed`、`rotation` 单位四元数、`angvel` 0;
/// - 单格 `EntityDesc { block: 材料, ..default }`(半边长 0.5 = B 键同型);
///   复合体 `EntityDesc::compound(COMPOUND_HALF_SIZE, shape)`(每格同一材料);
/// - 拒绝:材料基岩;任一格的格心 `c_k = translation + o_k` 所在块
///   **`floor(c_k)`** 为实心(`block_at` = Some(非空气);`None` 未装载
///   不算实心,交给服务器现有规则);文案给冲突格坐标。
///
/// `yaw_deg` 只用于形状偏移取整;`cam_dir` 内部单位化,零向量防御性回退
/// +x(窗口初始化帧的病态输入不 panic)。
pub fn launch_request(
    shape: LaunchShape,
    material: Material,
    speed: f32,
    cam_pos: Vec3,
    cam_dir: Vec3,
    yaw_deg: f32,
    world: &dyn WorldQuery,
) -> Result<(EntityState, EntityDesc), String> {
    // 材料:基岩不可发射。
    if material == Material::Bedrock {
        return Err(super::notices::BEDROCK_LAUNCH.to_owned());
    }
    let dir = if cam_dir == Vec3::ZERO {
        Vec3::X
    } else {
        cam_dir.normalize()
    };
    let block = material.block();
    let offsets: Vec<[i32; 3]> = shape.world_cells(yaw_deg);

    // d = 2 − min(0, min_k(o_k · d̂)):离相机最近的那格沿视线也在 2 m 外。
    let mut min_dot = 0.0_f32;
    for [ox, oy, oz] in &offsets {
        let o = Vec3::new(*ox as f32, *oy as f32, *oz as f32);
        min_dot = min_dot.min(o.dot(dir));
    }
    let d = SPAWN_CLEARANCE - min_dot.min(0.0);
    let translation = cam_pos + dir * d;

    // 实心判定:每格格心 c_k = translation + o_k,查 floor(c_k) 所在块
    // (连续出生点不落格,按格心所在格查询;非空气 → 拒绝,文案给该格)。
    for [ox, oy, oz] in &offsets {
        let c = translation + Vec3::new(*ox as f32, *oy as f32, *oz as f32);
        let cell = [c.x.floor() as i32, c.y.floor() as i32, c.z.floor() as i32];
        let pos = types::BlockPos::new(cell[0], cell[1], cell[2]);
        if let Some(b) = world.block_at(pos)
            && !b.is_air()
        {
            return Err(format!(
                "spawn cell ({},{},{}) inside solid block, launch rejected",
                cell[0], cell[1], cell[2]
            ));
        }
    }

    // 组装:translation 连续(不取整;脚本 spawn 的 at 语义对齐到"原点格
    // [0,0,0] 的格心"是格对齐输入,发射的输入是连续视线,二者同型在 desc)。
    let state = EntityState {
        translation,
        linvel: dir * speed,
        ..EntityState::default()
    };
    let desc = match shape {
        LaunchShape::Single => EntityDesc {
            block,
            ..EntityDesc::default()
        },
        // 复合体:每格同一材料,与脚本 spawn.shape 组装同型(AC-W4)。
        _ => {
            let cells = offsets
                .into_iter()
                .map(|off| ShapeCell { offset: off, block })
                .collect();
            match CompoundShape::new(cells) {
                Ok(s) => EntityDesc::compound(COMPOUND_HALF_SIZE, s),
                // 单格之外不可能失败(≥2 格、偏移唯一);防御:降级单格。
                Err(_) => EntityDesc {
                    block,
                    ..EntityDesc::default()
                },
            }
        }
    };
    Ok((state, desc))
}

/// 蓄力状态机(§3.2;纯逻辑,render 逐帧驱动):
/// - `press()` 记下按下;`update(dt)` 逐帧累加按住时长;
/// - `release()`:按住 < 0.25 s = 点按 → 预设速度;≥ 0.25 s = 蓄力 →
///   `clamp(5 + 25 × (t − 0.25), 5, 55)`;
/// - 发射被拒绝时调用 `cancel()`(蓄力同样结束,无速度产出)。
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ChargeState {
    /// 按住中的起点累计时长(未按下 = None)。
    held: Option<f32>,
}

impl ChargeState {
    /// `F` 按下。
    pub fn press(&mut self) {
        self.held = Some(0.0);
    }

    /// 逐帧推进(按住时累加;dt 秒,墙钟不进模拟)。
    pub fn update(&mut self, dt: f32) {
        if let Some(t) = &mut self.held {
            *t += dt;
        }
    }

    /// `F` 松开:产出本发的速度(点按 = 预设;蓄力 = 公式)并复位。
    #[must_use]
    pub fn release(&mut self, preset: f32) -> f32 {
        match self.held.take() {
            None => preset, // 未按下时松开(防御)= 预设
            Some(t) if t < CHARGE_THRESHOLD => preset,
            Some(t) => charge_speed(t),
        }
    }

    /// 拒绝路径:结束按住,不产速度。
    pub fn cancel(&mut self) {
        self.held = None;
    }

    /// 当前按住中?(蓄力条显示用)。
    #[must_use]
    pub fn holding(&self) -> bool {
        self.held.is_some()
    }

    /// 当前蓄力显示速度(蓄力条;未按住 = None)。点按阶段(< 0.25 s)
    /// 返回 None(条只在蓄力阶段显示,§3.2"蓄力期间")。
    #[must_use]
    pub fn display_speed(&self) -> Option<f32> {
        let t = self.held?;
        (t >= CHARGE_THRESHOLD).then(|| charge_speed(t))
    }
}

/// 蓄力速度公式(§3.2):`clamp(5 + 25 × (t − 0.25), 5, 55)`。
#[must_use]
pub fn charge_speed(held_secs: f32) -> f32 {
    (SPEED_MIN + CHARGE_RATE * (held_secs - CHARGE_THRESHOLD)).clamp(SPEED_MIN, SPEED_MAX)
}

/// 滚轮调预设速度(§3.2):`wheel` 取 `mouse_wheel().1` 的符号(正 +1 档、
/// 负 −1 档,一帧最多一档),0 不动;结果夹 5–55。
#[must_use]
pub fn adjust_preset_speed(preset: f32, wheel: f32) -> f32 {
    if wheel > 0.0 {
        (preset + SPEED_STEP).min(SPEED_MAX)
    } else if wheel < 0.0 {
        (preset - SPEED_STEP).max(SPEED_MIN)
    } else {
        preset
    }
}

/// 蓄力速度条 ASCII(§3.2"屏幕中央显示速度条与数值";W02 放行示例
/// `[########......] 30 m/s`):14 格条按速度线性填充,数值取整。
#[must_use]
pub fn speed_bar(speed: f32) -> String {
    const WIDTH: usize = 14;
    let filled = ((speed - SPEED_MIN) / (SPEED_MAX - SPEED_MIN) * WIDTH as f32).round() as usize;
    let filled = filled.min(WIDTH);
    format!(
        "[{}{}] {} m/s",
        "#".repeat(filled),
        ".".repeat(WIDTH - filled),
        speed.round() as i32
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::tests_support::FakeVoxelWorld;
    use types::BlockId;
    use types::glam::Vec3;
    use voxel::builtin;

    const DIRT: BlockId = builtin::DIRT;

    /// yaw = 0°(w=+x、u=+z)、pitch = 0 的水平视线,原点在相机前 2 m 的
    /// 格心。手算锚点(放行说明要求逐值断言的基础)。
    fn flat_dir() -> Vec3 {
        Vec3::new(1.0, 0.0, 0.0)
    }

    // ---- AC-W3:五形状 × 四朝向 ----

    /// 五形状 yaw=0°:世界偏移表(放行说明要求回评附此表)。
    #[test]
    fn world_cells_yaw0_matches_table() {
        let assert_offsets = |shape: LaunchShape, want: &[[i32; 3]]| {
            let got = shape.world_cells(0.0);
            let mut want = want.to_vec();
            want.sort();
            let mut got = got;
            got.sort();
            assert_eq!(got, want, "{shape:?} yaw=0° 偏移");
        };
        assert_offsets(LaunchShape::Single, &[[0, 0, 0]]);
        // 6 格条·纵:w=0..5 → +x。
        assert_offsets(
            LaunchShape::BarLongitudinal,
            &[
                [0, 0, 0],
                [1, 0, 0],
                [2, 0, 0],
                [3, 0, 0],
                [4, 0, 0],
                [5, 0, 0],
            ],
        );
        // 6 格条·横:u=0..5 → +z。
        assert_offsets(
            LaunchShape::BarCross,
            &[
                [0, 0, 0],
                [0, 0, 1],
                [0, 0, 2],
                [0, 0, 3],
                [0, 0, 4],
                [0, 0, 5],
            ],
        );
        // 2×2 板:u=0..1、v=0..1(yaw=0 的 u=+z → u 进 z)。
        assert_offsets(
            LaunchShape::Plate2x2,
            &[[0, 0, 0], [0, 1, 0], [0, 0, 1], [0, 1, 1]],
        );
        // 3×3×3:u, v, w = 0..2,27 格。
        let cube = LaunchShape::Block3x3x3.world_cells(0.0);
        assert_eq!(cube.len(), 27);
        assert!(cube.contains(&[2, 2, 2]));
        assert!(cube.contains(&[0, 0, 0]));
        // 四朝向:6 格条·纵(yaw 0/90/180/270)沿 w 转。
        let long = LaunchShape::BarLongitudinal;
        assert!(long.world_cells(0.0).contains(&[5, 0, 0]), "yaw 0 → w=+x");
        assert!(long.world_cells(90.0).contains(&[0, 0, 5]), "yaw 90 → w=+z");
        assert!(
            long.world_cells(180.0).contains(&[-5, 0, 0]),
            "yaw 180 → w=−x"
        );
        assert!(
            long.world_cells(-90.0).contains(&[0, 0, -5]),
            "yaw −90 → w=−z"
        );
    }

    /// yaw = 0、pitch = 0 的出生格心与 d(单格:d = 2;复合体所有 o·d̂ ≥ 0
    /// 时 d 仍 = 2)。
    #[test]
    fn launch_translation_yaw0_pitch0() {
        let world = FakeVoxelWorld::empty();
        // 相机在 (10.0, −56.0, 20.0),朝 +x 平视。
        let cam = Vec3::new(10.0, -56.0, 20.0);
        // 单格:d = 2 → cam + (2,0,0) = (12,−56,20);格 = (12,−56,20)
        // (小数部分 .0 直接落格点);格心 = +0.5。
        let (state, desc) = launch_request(
            LaunchShape::Single,
            crate::tools::Material::Dirt,
            20.0,
            cam,
            flat_dir(),
            0.0,
            &world,
        )
        .expect("单格发射");
        // translation = cam + d̂·d 连续值(W02-2):10+2=12、−56、20,不取整。
        assert_eq!(
            state.translation,
            Vec3::new(12.0, -56.0, 20.0),
            "连续出生点"
        );
        assert_eq!(state.linvel, Vec3::new(20.0, 0.0, 0.0), "linvel = d̂·speed");
        assert_eq!(state.rotation, types::glam::Quat::IDENTITY);
        assert_eq!(state.angvel, Vec3::ZERO);
        assert_eq!(desc.block, DIRT);
        assert_eq!(desc.half_size, 0.5, "单格半边长与 B 键同型");
        assert!(desc.shape().is_none(), "单格无形状");

        // 6 格条·纵(w=+x,偏移 0..5 沿 +x):所有 o·d̂ ≥ 0 → d = 2,原点同
        // 单格;形状 6 格每格 dirt。
        let (state, desc) = launch_request(
            LaunchShape::BarLongitudinal,
            crate::tools::Material::Dirt,
            20.0,
            cam,
            flat_dir(),
            0.0,
            &world,
        )
        .expect("条纵发射");
        assert_eq!(state.translation, Vec3::new(12.0, -56.0, 20.0));
        let shape = desc.shape().expect("复合体带形状");
        assert_eq!(shape.cells().len(), 6);
        assert_eq!(desc.half_size, COMPOUND_HALF_SIZE);
        assert_eq!(desc.block, DIRT, "compound 的 block = 第一格方块");
        // 偏移 canonical:0..5 沿 x。
        let offs: Vec<[i32; 3]> = shape.cells().iter().map(|c| c.offset).collect();
        assert_eq!(
            offs,
            vec![
                [0, 0, 0],
                [1, 0, 0],
                [2, 0, 0],
                [3, 0, 0],
                [4, 0, 0],
                [5, 0, 0]
            ]
        );
    }

    /// 低头看(pitch = −60°)发射 3×3×3 块:偏移的 y 只有 v(0..2,≥0),
    /// 低头时 d̂.y < 0,v=2 的格与 d̂ 反向(点积 < 0)→ d 被抬高。
    #[test]
    fn launch_pitch_down_raises_d() {
        let world = FakeVoxelWorld::empty();
        let cam = Vec3::new(10.0, -50.0, 20.0);
        // yaw = 0、pitch = −60°:d̂ = (cy·cp, sp, sy·cp) = (0.5, −0.866, 0)
        // (单位化后即此;|·| = √(0.25+0.75) = 1)。
        let (sp, cp) = (-60.0_f32.to_radians()).sin_cos();
        let (sy, cy) = 0.0_f32.to_radians().sin_cos();
        let dir = Vec3::new(cy * cp, sp, sy * cp);
        let (state, _) = launch_request(
            LaunchShape::Block3x3x3,
            crate::tools::Material::Stone,
            20.0,
            cam,
            dir,
            0.0,
            &world,
        )
        .expect("低头发射");
        // 最小点积格:o=(0,2,0)(v=2),o·d̂ = 2×(−0.866) = −1.732;
        // d = 2 + 1.732 = 3.732。translation = cam + d̂·d(连续,W02-2)
        //   = (10 + 0.5×3.732, −50 − 0.866×3.732, 20) ≈ (11.866, −53.232, 20)。
        assert!(
            (state.translation.x - 11.866).abs() < 1e-3
                && (state.translation.y - (-53.232)).abs() < 1e-3
                && (state.translation.z - 20.0).abs() < 1e-3,
            "低头 d 抬高后的连续出生点,实得 {}",
            state.translation
        );
    }

    /// 拒绝:基岩材料;出生格心在实心里(W02-2:非整数出生点,按各格格心
    /// floor 查块)。
    #[test]
    fn launch_rejections() {
        // 基岩。
        let world = FakeVoxelWorld::empty();
        let got = launch_request(
            LaunchShape::Single,
            crate::tools::Material::Bedrock,
            20.0,
            Vec3::new(10.0, -56.0, 20.0),
            flat_dir(),
            0.0,
            &world,
        );
        assert!(got.is_err(), "基岩拒绝");
        assert!(got.unwrap_err().contains("bedrock"), "拒绝文案含 bedrock");

        // 非整数出生点冲突(W02-2):相机 (10.3,−55.6,20.2) 平视 +x →
        // translation = (12.3,−55.6,20.2),格心所在格 floor = (12,−56,20);
        // 在该格放土 → 拒绝,文案含 (12,-56,20)。
        let mut world = FakeVoxelWorld::empty();
        world.set_block(12, -56, 20, DIRT);
        let got = launch_request(
            LaunchShape::Single,
            crate::tools::Material::Dirt,
            20.0,
            Vec3::new(10.3, -55.6, 20.2),
            flat_dir(),
            0.0,
            &world,
        );
        assert!(
            got.is_err(),
            "出生格 floor(12.3,−55.6,20.2)=(12,−56,20) 实心 → 拒绝"
        );
        assert!(
            got.unwrap_err().contains("(12,-56,20)"),
            "拒绝文案含冲突格坐标"
        );

        // 同一出生点,土挪到 (13,−56,20)(下一格,非出生格)→ 放行。
        let mut world = FakeVoxelWorld::empty();
        world.set_block(13, -56, 20, DIRT);
        let (state, _) = launch_request(
            LaunchShape::Single,
            crate::tools::Material::Dirt,
            20.0,
            Vec3::new(10.3, -55.6, 20.2),
            flat_dir(),
            0.0,
            &world,
        )
        .expect("冲突格不在出生格 → 放行");
        assert!(
            (state.translation.x - 12.3).abs() < 1e-5
                && (state.translation.y - (-55.6)).abs() < 1e-5
                && (state.translation.z - 20.2).abs() < 1e-5,
            "连续出生点 (12.3,−55.6,20.2),实得 {}",
            state.translation
        );

        // 整数出生点(W02-2 前的旧用例口径,期望改连续值):格 (12,−56,20)
        // 放土、出生点恰 (12,−56,20) → 同样拒绝。
        let mut world = FakeVoxelWorld::empty();
        world.set_block(12, -56, 20, DIRT);
        let got = launch_request(
            LaunchShape::Single,
            crate::tools::Material::Dirt,
            20.0,
            Vec3::new(10.0, -56.0, 20.0),
            flat_dir(),
            0.0,
            &world,
        );
        assert!(got.is_err(), "整数出生点恰在实心格 → 拒绝");

        // 未装载(None)不算实心:同一位置在未装载 chunk,放行。
        let unloaded = FakeVoxelWorld::empty();
        let (state, _) = launch_request(
            LaunchShape::Single,
            crate::tools::Material::Dirt,
            20.0,
            Vec3::new(10.0, -56.0, 20.0),
            flat_dir(),
            0.0,
            &unloaded,
        )
        .expect("None 不算实心");
        assert_eq!(state.translation, Vec3::new(12.0, -56.0, 20.0));
    }

    /// W02-2 补充(AC-W3):水平斜视抬高 d——yaw = −30°、pitch = 0,发
    /// 6 格条·横。w 取整为 +x、u = +z,d̂ = (0.866, 0, −0.5);u = 5 那格
    /// o = (0,0,5),o·d̂ = −2.5 → d = 4.5(只靠水平分量触发,抓"只看 y
    /// 分量"的实现错误)。
    #[test]
    fn launch_yaw_diagonal_raises_d() {
        let world = FakeVoxelWorld::empty();
        let cam = Vec3::new(10.0, -56.0, 20.0);
        let dir = Vec3::new(
            (-30.0_f32).to_radians().cos(),
            0.0,
            (-30.0_f32).to_radians().sin(),
        );
        let (state, _) = launch_request(
            LaunchShape::BarCross,
            crate::tools::Material::Dirt,
            20.0,
            cam,
            dir,
            -30.0,
            &world,
        )
        .expect("斜视发射");
        // d = 4.5:translation = cam + d̂·4.5
        //   = (10 + 0.866×4.5, −56, 20 − 0.5×4.5) ≈ (13.897, −56, 17.75)。
        assert!(
            (state.translation.x - (10.0 + 0.866_025_4 * 4.5)).abs() < 1e-3
                && (state.translation.y - (-56.0)).abs() < 1e-5
                && (state.translation.z - (20.0 - 0.5 * 4.5)).abs() < 1e-5,
            "yaw=−30° 条·横 d=4.5 的连续出生点,实得 {}",
            state.translation
        );
    }

    // ---- 蓄力状态机(AC-W3)----

    /// 0.2 s 松开 = 点按(预设);1.25 s = 30;5 s = 55;拒绝取消不产速度。
    #[test]
    fn charge_state_thresholds() {
        let mut c = ChargeState::default();
        c.press();
        // 0.2 s(< 0.25)逐帧推进后松开 = 预设 20。
        let mut rest = 0.2_f32;
        while rest > 0.0 {
            let dt = rest.min(1.0 / 60.0);
            c.update(dt);
            rest -= dt;
        }
        assert_eq!(c.release(20.0), 20.0, "点按 = 预设速度");

        // 1.25 s → clamp(5 + 25×1.0) = 30。
        let mut c = ChargeState::default();
        c.press();
        let mut rest = 1.25_f32;
        while rest > 1e-6 {
            let dt = rest.min(1.0 / 60.0);
            c.update(dt);
            rest -= dt;
        }
        let speed = c.release(20.0);
        assert!((speed - 30.0).abs() < 1e-3, "1.25 s 蓄力 = 30,实得 {speed}");

        // 5 s → 满 55。
        let mut c = ChargeState::default();
        c.press();
        for _ in 0..(5.0 * 60.0) as usize {
            c.update(1.0 / 60.0);
        }
        assert_eq!(c.release(20.0), 55.0, "5 s 蓄力封顶 55");

        // 拒绝取消:cancel 后 release = 预设(且不再按蓄力公式)。
        let mut c = ChargeState::default();
        c.press();
        c.update(2.0);
        c.cancel();
        assert_eq!(c.release(20.0), 20.0, "取消后点按预设");
        assert!(!c.holding());
    }

    /// W02-2 补充(AC-W3):蓄力阈值边界——`update(0.25)` 恰满 0.25 s
    /// 即进入蓄力(§3.2"按住满 0.25 s 进入蓄力")→ 速度 5;`update(0.249)`
    /// 仍点按 → 预设 20。抓 `<` 误写成 `<=` 的实现。
    #[test]
    fn charge_threshold_boundary() {
        // 恰 0.25 s:进入蓄力,速度 = clamp(5 + 25×0) = 5。
        let mut c = ChargeState::default();
        c.press();
        c.update(0.25);
        let speed = c.release(20.0);
        assert!(
            (speed - 5.0).abs() < 1e-4,
            "恰满 0.25 s = 蓄力起点 5,实得 {speed}"
        );

        // 0.249 s:仍点按 = 预设 20。
        let mut c = ChargeState::default();
        c.press();
        c.update(0.249);
        assert_eq!(c.release(20.0), 20.0, "0.249 s 仍点按 = 预设");

        // display_speed 同口径:恰 0.25 s 起显示(条只在蓄力阶段出现)。
        let mut c = ChargeState::default();
        c.press();
        c.update(0.249);
        assert_eq!(c.display_speed(), None, "0.249 s 无速度条");
        c.update(0.001);
        assert!(
            (c.display_speed().expect("恰 0.25 s 有速度条") - 5.0).abs() < 1e-4,
            "恰 0.25 s 速度条显示 5"
        );
    }

    /// 帧切分无关:同一总时长(1.25 s),60 等分 vs 7 等分 vs 不等长序列,
    /// 速度差 < 1e-3。
    #[test]
    fn charge_frame_split_independent() {
        let total = 1.25_f32;
        let run = |splits: &[f32]| {
            let mut c = ChargeState::default();
            c.press();
            for &dt in splits {
                c.update(dt);
            }
            c.release(20.0)
        };
        // 60 等分。
        let mut s60 = Vec::new();
        for _ in 0..60 {
            s60.push(total / 60.0);
        }
        // 7 等分(不等长:最后一帧补齐)。
        let mut s7 = vec![0.2_f32; 6];
        s7.push(total - 1.2);
        // 杂数序列。
        let smixed = vec![0.5, 0.3, 0.25, 0.2];
        let a = run(&s60);
        let b = run(&s7);
        let c2 = run(&smixed);
        assert!((a - 30.0).abs() < 1e-3, "60 帧: {a}");
        assert!((b - a).abs() < 1e-3, "7 帧 vs 60 帧:{b} vs {a}");
        assert!((c2 - a).abs() < 1e-3, "杂数 vs 60 帧:{c2} vs {a}");
    }

    /// 滚轮夹值:默认 20,+5/−5 一档,5/55 夹住,0 不动。
    #[test]
    fn wheel_clamps_preset() {
        assert_eq!(adjust_preset_speed(20.0, 1.0), 25.0);
        assert_eq!(adjust_preset_speed(20.0, -0.5), 15.0);
        assert_eq!(adjust_preset_speed(50.0, 1.0), 55.0);
        assert_eq!(adjust_preset_speed(50.0, 1.0), 55.0, "上限处不再加");
        assert_eq!(adjust_preset_speed(5.0, -1.0), 5.0, "下限处不再减");
        assert_eq!(adjust_preset_speed(20.0, 0.0), 20.0, "无滚动不动");
    }

    // ---- AC-W4:与脚本 spawn.shape 组装同型 ----

    /// 同一形状(6 格条·纵放,block dirt)分别经发射路径与脚本 `parse`
    /// 的 spawn.shape 路径组装,EntityDesc 相等。
    #[test]
    fn desc_matches_script_spawn_shape_path() {
        // 脚本路径:解析一段 toml,取 spawn 步骤的 shape(与冒烟场景同款写法)。
        let toml = r#"
name = "probe"
req = "REQ-011"
[[step]]
spawn = { at = [0.0, 0.0, 0.0], count = 1, shape = { cells = [[0,0,0],[1,0,0],[2,0,0],[3,0,0],[4,0,0],[5,0,0]], block = "dirt" } }
[[step]]
check = { live_entities = { ge = 0 } }
"#;
        let scenario = crate::script::parse_scenario(toml).expect("探针场景解析");
        let script_shape = match &scenario.steps[0].action {
            crate::script::Action::Spawn { shape, .. } => shape.clone().expect("脚本带形状"),
            other => panic!("首步应为 spawn,实为 {other:?}"),
        };
        // 发射路径:同偏移、同材料(dirt)组装。
        let world = FakeVoxelWorld::empty();
        let (_, desc) = launch_request(
            LaunchShape::BarLongitudinal,
            crate::tools::Material::Dirt,
            20.0,
            Vec3::new(10.0, -56.0, 20.0),
            flat_dir(),
            0.0,
            &world,
        )
        .expect("发射组装");
        assert_eq!(
            desc.shape().expect("发射带形状"),
            &script_shape,
            "AC-W4 同型"
        );
        assert_eq!(desc.half_size, COMPOUND_HALF_SIZE);
    }

    /// 速度条 ASCII:5 → 空、30 → 半、55 → 满。
    #[test]
    fn speed_bar_renders_ascii() {
        let bar = speed_bar(30.0);
        assert!(bar.starts_with('[') && bar.ends_with("m/s"), "{bar}");
        assert!(bar.contains("30"), "{bar}");
        assert_eq!(speed_bar(5.0), "[..............] 5 m/s");
        assert_eq!(speed_bar(55.0), "[##############] 55 m/s");
    }

    /// E 循环序:单格 → 条纵 → 条横 → 板 → 块 → 单格。
    #[test]
    fn shape_cycle_order() {
        let mut s = LaunchShape::Single;
        let order = [
            LaunchShape::Single,
            LaunchShape::BarLongitudinal,
            LaunchShape::BarCross,
            LaunchShape::Plate2x2,
            LaunchShape::Block3x3x3,
        ];
        for want in order {
            assert_eq!(s, want);
            s = s.next();
        }
        assert_eq!(s, LaunchShape::Single, "循环回单格");
        // ASCII 名。
        assert_eq!(LaunchShape::BarLongitudinal.name(), "bar-long");
        assert_eq!(LaunchShape::Plate2x2.name(), "plate 2x2");
    }
}
