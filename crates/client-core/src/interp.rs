//! interp —— 客户端快照插值缓冲与渲染时钟(REQ-014 §6)。
//!
//! tick 域纯逻辑,零 bevy、零墙钟;渲染时刻由调用方按帧推进。
//! 提供双帧 lerp/slerp、≤50ms 钳制线速外推(子连杆与旋转冻结)、
//! 限速 slew 渲染时钟以及单机/附身旁路解析纯函数。

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use types::{EntityId, EntityState};

use crate::entities::{AgentView, SolidView};

/// 服务器仿真拍率(Hz;镜像服务器默认 `dt = 1/60`,见 `app::config::Config`)。
pub const TICKS_PER_SEC: f32 = 60.0;
/// 外推上限(tick;50ms @ 60Hz)。
pub const EXTRAP_CLAMP_TICKS: f64 = 3.0;
/// 默认缓冲延迟(tick;L1 200ms,L2/抖动参数 M05 实测再调)。
pub const DEFAULT_BUFFER_TICKS: u32 = 12;
/// 每实体最多保留快照数。
pub const MAX_SNAPS: usize = 4;

/// 可在快照间插值并做短时钳制外推的视图类型。
pub trait Interpolatable: Clone {
    /// 按 `alpha ∈ [0, 1]` 在 `a` 与 `b` 之间插值。
    fn interpolate(a: &Self, b: &Self, alpha: f32) -> Self;
    /// 以当前快照向前外推 `dt_ticks` 拍。
    fn extrapolate(&self, dt_ticks: f32) -> Self;
}

impl Interpolatable for AgentView {
    fn interpolate(a: &Self, b: &Self, alpha: f32) -> Self {
        if a.poses.len() != b.poses.len() {
            return b.clone();
        }
        let poses = a
            .poses
            .iter()
            .zip(&b.poses)
            .map(|(pa, pb)| EntityState {
                translation: pa.translation.lerp(pb.translation, alpha),
                rotation: pa.rotation.slerp(pb.rotation, alpha),
                linvel: pb.linvel,
                angvel: pb.angvel,
            })
            .collect();
        Self {
            hash: b.hash,
            poses,
        }
    }

    fn extrapolate(&self, dt_ticks: f32) -> Self {
        let mut out = self.clone();
        if let Some(root) = out.poses.first_mut() {
            root.translation += root.linvel * (dt_ticks / TICKS_PER_SEC);
        }
        out
    }
}

impl Interpolatable for SolidView {
    fn interpolate(a: &Self, b: &Self, alpha: f32) -> Self {
        Self {
            state: EntityState {
                translation: a.state.translation.lerp(b.state.translation, alpha),
                rotation: a.state.rotation.slerp(b.state.rotation, alpha),
                linvel: b.state.linvel,
                angvel: b.state.angvel,
            },
            compound: b.compound.clone(),
        }
    }

    fn extrapolate(&self, dt_ticks: f32) -> Self {
        let mut out = self.clone();
        out.state.translation += out.state.linvel * (dt_ticks / TICKS_PER_SEC);
        out
    }
}

/// 按实体分轨的快照插值缓冲(轨内按服务器 tick 严格升序,最多保留 [`MAX_SNAPS`] 帧)。
#[derive(Debug, Clone)]
pub struct InterpBuffer<V: Interpolatable> {
    tracks: BTreeMap<EntityId, VecDeque<(u64, V)>>,
}

impl<V: Interpolatable> Default for InterpBuffer<V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<V: Interpolatable> InterpBuffer<V> {
    /// 构造空缓冲。
    pub fn new() -> Self {
        Self {
            tracks: BTreeMap::new(),
        }
    }

    /// 推入实体在 `tick` 拍的快照。
    ///
    /// 若 `tick` 小于轨道末拍则忽略(迟到旧包);等于末拍则替换末元素;
    /// 大于末拍则追加并在超过 [`MAX_SNAPS`] 时从队首裁剪。
    pub fn push(&mut self, tick: u64, id: EntityId, value: V) {
        let track = self.tracks.entry(id).or_default();
        if let Some((latest_tick, latest_val)) = track.back_mut() {
            if tick < *latest_tick {
                return;
            }
            if tick == *latest_tick {
                *latest_val = value;
                return;
            }
        }
        track.push_back((tick, value));
        while track.len() > MAX_SNAPS {
            track.pop_front();
        }
    }

    /// 移除指定实体的快照轨道。
    pub fn remove(&mut self, id: EntityId) {
        self.tracks.remove(&id);
    }

    /// 全量快照对账:仅保留 `keep` 集合内的实体轨道。
    pub fn retain(&mut self, keep: &BTreeSet<EntityId>) {
        self.tracks.retain(|id, _| keep.contains(id));
    }

    /// 在 `render_tick` 采样实体视图:
    /// 无轨道返回 `None`;早于首帧向过去冻结;落入相邻帧区间做双帧插值;
    /// 超出末帧按 [`EXTRAP_CLAMP_TICKS`] 钳制外推。
    pub fn sample(&self, id: EntityId, render_tick: f64) -> Option<V> {
        let track = self.tracks.get(&id)?;
        let front = track.front()?;
        if render_tick <= front.0 as f64 {
            return Some(front.1.clone());
        }
        for ((t0, v0), (t1, v1)) in track.iter().zip(track.iter().skip(1)) {
            if *t0 as f64 <= render_tick && render_tick <= *t1 as f64 {
                let alpha = ((render_tick - *t0 as f64) / (*t1 - *t0) as f64) as f32;
                return Some(V::interpolate(v0, v1, alpha));
            }
        }
        let back = track.back()?;
        let dt = (render_tick - back.0 as f64).min(EXTRAP_CLAMP_TICKS);
        Some(back.1.extrapolate(dt as f32))
    }

    /// 返回指定实体当前缓存的快照数(测试/观测用)。
    pub fn track_len(&self, id: EntityId) -> usize {
        self.tracks.get(&id).map_or(0, VecDeque::len)
    }
}

/// 渲染时钟(tick 域):维护落后最新快照 `buffer_ticks` 拍的目标渲染时刻并限速 slew 追赶。
#[derive(Debug, Clone)]
pub struct RenderClock {
    now: f64,
    buffer_ticks: u32,
    latest: Option<u64>,
}

impl RenderClock {
    /// 构造初始渲染时钟(`now = 0.0`, `latest = None`)。
    pub fn new(buffer_ticks: u32) -> Self {
        Self {
            now: 0.0,
            buffer_ticks,
            latest: None,
        }
    }

    /// 收到服务器快照拍号时调用:首帧一次性初始化 `now`,后续仅单调推进 `latest` 且不跳变 `now`。
    pub fn on_snapshot(&mut self, tick: u64) {
        match self.latest {
            None => {
                self.latest = Some(tick);
                self.now = (tick as f64 - f64::from(self.buffer_ticks)).max(0.0);
            }
            Some(prev) => {
                if tick > prev {
                    self.latest = Some(tick);
                }
            }
        }
    }

    /// 按帧推进渲染时刻 `dt_ticks` 拍(有快照时以 ±25% 限速 slew 追赶目标时刻)。
    pub fn advance(&mut self, dt_ticks: f64) {
        let Some(latest) = self.latest else {
            self.now += dt_ticks;
            return;
        };
        let target = (latest as f64 - f64::from(self.buffer_ticks)).max(0.0);
        let err = target - self.now;
        let speed = (1.0 + err / 16.0).clamp(0.75, 1.25);
        self.now += dt_ticks * speed;
    }

    /// 当前渲染时刻(tick)。
    pub fn now(&self) -> f64 {
        self.now
    }
}

/// 旁路解析纯函数:`bypass` 时直接返回 `latest` 克隆,否则优先采样插值缓冲并缺省回退 `latest`。
pub fn resolve_view<V: Interpolatable>(
    buffer: Option<&InterpBuffer<V>>,
    render_tick: f64,
    id: EntityId,
    latest: &V,
    bypass: bool,
) -> V {
    if bypass {
        latest.clone()
    } else {
        buffer
            .and_then(|b| b.sample(id, render_tick))
            .unwrap_or_else(|| latest.clone())
    }
}

/// 把镜像已解码的主体快照喂入插值缓冲(M04b-2 Compact/Lite 共用)。
///
/// `full` 时按 `mirror.agents` 键集 retain(分档 full 后镜像即权威,跨档轨道保留);
/// `ids` 中存在于 `mirror.agents` 者推入 `tick`;`removed` 逐条移除;最后推进 `clock.on_snapshot(tick)`。
pub fn sync_agent_interp(
    buffer: &mut InterpBuffer<AgentView>,
    clock: &mut RenderClock,
    mirror: &crate::entities::EntityMirror,
    tick: u64,
    ids: &[EntityId],
    removed: &[EntityId],
    full: bool,
) {
    if full {
        let keep: BTreeSet<EntityId> = mirror.agents.keys().copied().collect();
        buffer.retain(&keep);
    }
    for id in ids {
        if let Some(view) = mirror.agents.get(id) {
            buffer.push(tick, *id, view.clone());
        }
    }
    for id in removed {
        buffer.remove(*id);
    }
    clock.on_snapshot(tick);
}

#[cfg(test)]
mod tests {
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

    use types::glam::{Quat, Vec3};
    use types::{BlockId, BodyHash, CompoundShape, ShapeCell};

    use super::*;

    const EPS_F32: f32 = 1e-5;
    const EPS_F64: f64 = 1e-9;

    fn eid(n: u32) -> EntityId {
        EntityId::from_parts(0, n)
    }

    fn test_hash() -> BodyHash {
        BodyHash::fnv1a(b"test")
    }

    fn single_agent(pos: Vec3, rot: Quat, linvel: Vec3) -> AgentView {
        AgentView {
            hash: test_hash(),
            poses: vec![EntityState {
                translation: pos,
                rotation: rot,
                linvel,
                angvel: Vec3::ZERO,
            }],
        }
    }

    fn three_cell_column() -> CompoundShape {
        CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [0, 1, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [0, 2, 0],
                block: BlockId(1),
            },
        ])
        .expect("三格复合形合法")
    }

    fn assert_vec3_near(actual: Vec3, expected: Vec3, label: &str) {
        assert!(
            (actual.x - expected.x).abs() < EPS_F32
                && (actual.y - expected.y).abs() < EPS_F32
                && (actual.z - expected.z).abs() < EPS_F32,
            "{label}: 实得 {actual:?}, 期望 {expected:?}"
        );
    }

    fn assert_quat_near(actual: Quat, expected: Quat, label: &str) {
        assert!(
            (actual.x - expected.x).abs() < EPS_F32
                && (actual.y - expected.y).abs() < EPS_F32
                && (actual.z - expected.z).abs() < EPS_F32
                && (actual.w - expected.w).abs() < EPS_F32,
            "{label}: 实得 {actual:?}, 期望 {expected:?}"
        );
    }

    fn assert_f64_near(actual: f64, expected: f64, label: &str) {
        assert!(
            (actual - expected).abs() < EPS_F64,
            "{label}: 实得 {actual}, 期望 {expected}"
        );
    }

    /// T1 双帧 lerp/slerp:单连杆 tick10 (0,0,0)/IDENTITY 与 tick20 (10,0,0)/PI/2 Y。
    #[test]
    fn m02b_t1_two_frame_lerp_slerp() {
        let mut buf = InterpBuffer::new();
        let id = eid(1);
        let q90 = Quat::from_rotation_y(FRAC_PI_2);
        let q45 = Quat::from_rotation_y(FRAC_PI_4);
        buf.push(10, id, single_agent(Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO));
        buf.push(
            20,
            id,
            single_agent(Vec3::new(10.0, 0.0, 0.0), q90, Vec3::ZERO),
        );

        let s15 = buf.sample(id, 15.0).expect("sample(15)");
        assert_vec3_near(
            s15.poses[0].translation,
            Vec3::new(5.0, 0.0, 0.0),
            "s15 pos",
        );
        assert_quat_near(s15.poses[0].rotation, q45, "s15 rot");

        let s125 = buf.sample(id, 12.5).expect("sample(12.5)");
        assert_vec3_near(
            s125.poses[0].translation,
            Vec3::new(2.5, 0.0, 0.0),
            "s12.5 pos",
        );

        let s10 = buf.sample(id, 10.0).expect("sample(10)");
        assert_vec3_near(s10.poses[0].translation, Vec3::ZERO, "s10 pos");
        assert_quat_near(s10.poses[0].rotation, Quat::IDENTITY, "s10 rot");

        let s20 = buf.sample(id, 20.0).expect("sample(20)");
        assert_vec3_near(
            s20.poses[0].translation,
            Vec3::new(10.0, 0.0, 0.0),
            "s20 pos",
        );
        assert_quat_near(s20.poses[0].rotation, q90, "s20 rot");
    }

    /// T2 括号选择与裁剪:推入 5 帧裁至 MAX_SNAPS=4,按区间线性插值。
    #[test]
    fn m02b_t2_bracket_selection_and_trim() {
        let mut buf = InterpBuffer::new();
        let id = eid(2);
        for tick in [10_u64, 20, 30, 40, 50] {
            buf.push(
                tick,
                id,
                single_agent(Vec3::new(tick as f32, 0.0, 0.0), Quat::IDENTITY, Vec3::ZERO),
            );
        }
        assert_eq!(buf.track_len(id), 4, "应裁至 MAX_SNAPS=4(丢弃 tick10)");

        let s38 = buf.sample(id, 38.0).expect("sample(38)");
        assert_vec3_near(
            s38.poses[0].translation,
            Vec3::new(38.0, 0.0, 0.0),
            "s38 pos",
        );

        let s25 = buf.sample(id, 25.0).expect("sample(25)");
        assert_vec3_near(
            s25.poses[0].translation,
            Vec3::new(25.0, 0.0, 0.0),
            "s25 pos",
        );
    }

    /// T3 钳制外推 + 子连杆/旋转冻结:仅根连杆按线速外推且钳制在 3 tick,旋转与子连杆不变。
    #[test]
    fn m02b_t3_clamped_extrapolation_and_frozen_child_rot() {
        let mut buf = InterpBuffer::new();
        let id = eid(3);
        let q90 = Quat::from_rotation_y(FRAC_PI_2);
        let child_pos = Vec3::new(1.0, 2.0, 3.0);
        let child_linvel = Vec3::new(9.0, 9.0, 9.0);
        let child_angvel = Vec3::new(1.0, 0.0, 0.0);
        buf.push(
            10,
            id,
            AgentView {
                hash: test_hash(),
                poses: vec![
                    EntityState {
                        translation: Vec3::ZERO,
                        rotation: q90,
                        linvel: Vec3::new(6.0, 0.0, 0.0),
                        angvel: Vec3::new(0.0, 3.0, 0.0),
                    },
                    EntityState {
                        translation: child_pos,
                        rotation: Quat::IDENTITY,
                        linvel: child_linvel,
                        angvel: child_angvel,
                    },
                ],
            },
        );

        let s12 = buf.sample(id, 12.0).expect("sample(12)");
        assert_vec3_near(
            s12.poses[0].translation,
            Vec3::new(0.2, 0.0, 0.0),
            "s12 root pos",
        );
        assert_quat_near(s12.poses[0].rotation, q90, "s12 root rot");
        assert_eq!(
            s12.poses[1].translation, child_pos,
            "s12 子连杆位移必须逐位不变"
        );
        assert_eq!(
            s12.poses[1].rotation,
            Quat::IDENTITY,
            "s12 子连杆旋转必须逐位不变"
        );

        let s13 = buf.sample(id, 13.0).expect("sample(13)");
        assert_vec3_near(
            s13.poses[0].translation,
            Vec3::new(0.3, 0.0, 0.0),
            "s13 root pos",
        );

        let s14 = buf.sample(id, 14.0).expect("sample(14)");
        assert_vec3_near(
            s14.poses[0].translation,
            Vec3::new(0.3, 0.0, 0.0),
            "s14 root pos(钳制)",
        );

        let s100 = buf.sample(id, 100.0).expect("sample(100)");
        assert_vec3_near(
            s100.poses[0].translation,
            Vec3::new(0.3, 0.0, 0.0),
            "s100 root pos(钳制)",
        );
    }

    /// T4 RenderClock:首帧初始化、线性 slew、上下钳制(0.75/1.25)、旧 tick 忽略、首帧钳 0。
    #[test]
    fn m02b_t4_render_clock() {
        let mut clk = RenderClock::new(12);
        clk.on_snapshot(100);
        assert_f64_near(clk.now(), 88.0, "首帧初始化");

        clk.advance(1.0);
        assert_f64_near(clk.now(), 89.0, "err=0 speed=1");

        clk.advance(1.0);
        assert_f64_near(clk.now(), 89.9375, "err=-1 speed=0.9375");

        clk.on_snapshot(110);
        assert_f64_near(clk.now(), 89.9375, "新快照不跳变 now");

        clk.advance(1.0);
        assert_f64_near(clk.now(), 91.1875, "err=8.0625 上钳 speed=1.25");

        clk.on_snapshot(90);
        assert_eq!(clk.latest, Some(110), "旧 tick 应忽略,latest 仍 110");
        assert_f64_near(clk.now(), 91.1875, "旧 tick 不改 now");

        // 下钳验证:err=-10 → speed 钳 0.75。
        let mut clk2 = RenderClock::new(12);
        clk2.on_snapshot(100);
        assert_f64_near(clk2.now(), 88.0, "clk2 首帧");
        clk2.advance(10.0);
        assert_f64_near(clk2.now(), 98.0, "clk2 advance(10)");
        clk2.advance(1.0);
        assert_f64_near(clk2.now(), 98.75, "clk2 下钳 speed=0.75");

        // 首帧拍号小于缓冲延迟时钳 0。
        let mut clk3 = RenderClock::new(12);
        clk3.on_snapshot(5);
        assert_f64_near(clk3.now(), 0.0, "首帧钳 0");
    }

    /// T5 旁路 resolve_view:bypass 直写、非旁路采样、无缓冲回退、未知 id 回退。
    #[test]
    fn m02b_t5_bypass_resolve_view() {
        let mut buf = InterpBuffer::new();
        let id1 = eid(1);
        let latest = single_agent(Vec3::new(10.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ZERO);
        buf.push(
            10,
            id1,
            single_agent(Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO),
        );
        buf.push(20, id1, latest.clone());

        let bypassed = resolve_view(Some(&buf), 15.0, id1, &latest, true);
        assert_vec3_near(
            bypassed.poses[0].translation,
            Vec3::new(10.0, 0.0, 0.0),
            "bypass 直写",
        );

        let sampled = resolve_view(Some(&buf), 15.0, id1, &latest, false);
        assert_vec3_near(
            sampled.poses[0].translation,
            Vec3::new(5.0, 0.0, 0.0),
            "非旁路插值",
        );

        let no_buf = resolve_view(None, 15.0, id1, &latest, false);
        assert_vec3_near(
            no_buf.poses[0].translation,
            Vec3::new(10.0, 0.0, 0.0),
            "无缓冲回退 latest",
        );

        let unknown = resolve_view(Some(&buf), 15.0, eid(999), &latest, false);
        assert_vec3_near(
            unknown.poses[0].translation,
            Vec3::new(10.0, 0.0, 0.0),
            "未知 id 回退 latest",
        );
    }

    /// T6 迟到旧包忽略、同 tick 替换、full retain 对账与 remove 移除。
    #[test]
    fn m02b_t6_late_same_tick_and_full_reconcile() {
        let mut buf = InterpBuffer::new();
        let id1 = eid(1);
        let id2 = eid(2);

        buf.push(
            20,
            id1,
            single_agent(Vec3::new(20.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ZERO),
        );
        buf.push(
            10,
            id1,
            single_agent(Vec3::new(10.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ZERO),
        );
        assert_eq!(buf.track_len(id1), 1, "迟到旧包必须忽略");
        let v = buf.sample(id1, 20.0).expect("id1 应存在");
        assert_vec3_near(
            v.poses[0].translation,
            Vec3::new(20.0, 0.0, 0.0),
            "旧包忽略后保留 20",
        );

        buf.push(
            20,
            id1,
            single_agent(Vec3::new(99.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ZERO),
        );
        assert_eq!(buf.track_len(id1), 1, "同 tick 替换不增长度");
        let v = buf.sample(id1, 20.0).expect("id1 应存在");
        assert_vec3_near(
            v.poses[0].translation,
            Vec3::new(99.0, 0.0, 0.0),
            "同 tick 替换",
        );

        buf.push(
            20,
            id2,
            single_agent(Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ZERO),
        );
        let keep = BTreeSet::from([id2]);
        buf.retain(&keep);
        assert!(buf.sample(id1, 20.0).is_none(), "retain 后 id1 必须删除");
        assert!(buf.sample(id2, 20.0).is_some(), "retain 后 id2 应保留");

        buf.remove(id2);
        assert!(buf.sample(id2, 20.0).is_none(), "remove 后 id2 必须删除");
    }

    /// T7 SolidView 双帧插值与单帧外推(含 compound 逐位保留)。
    #[test]
    fn m02b_t7_solid_view_interp_and_extrap() {
        let shape = three_cell_column();
        let q90 = Quat::from_rotation_y(FRAC_PI_2);
        let q45 = Quat::from_rotation_y(FRAC_PI_4);
        let id = eid(7);

        let snap10 = SolidView {
            state: EntityState {
                translation: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                linvel: Vec3::new(6.0, 0.0, 0.0),
                angvel: Vec3::ZERO,
            },
            compound: Some(shape.clone()),
        };
        let snap20 = SolidView {
            state: EntityState {
                translation: Vec3::new(10.0, 0.0, 0.0),
                rotation: q90,
                linvel: Vec3::new(6.0, 0.0, 0.0),
                angvel: Vec3::ZERO,
            },
            compound: Some(shape.clone()),
        };

        let mut buf = InterpBuffer::new();
        buf.push(10, id, snap10.clone());
        buf.push(20, id, snap20);

        let s15 = buf.sample(id, 15.0).expect("solid sample(15)");
        assert_vec3_near(
            s15.state.translation,
            Vec3::new(5.0, 0.0, 0.0),
            "solid s15 pos",
        );
        assert_quat_near(s15.state.rotation, q45, "solid s15 rot");
        assert_eq!(
            s15.compound,
            Some(shape.clone()),
            "solid s15 compound 逐位保留"
        );

        let mut single_buf = InterpBuffer::new();
        single_buf.push(10, id, snap10);
        let s12 = single_buf.sample(id, 12.0).expect("solid sample(12)");
        assert_vec3_near(
            s12.state.translation,
            Vec3::new(0.2, 0.0, 0.0),
            "solid s12 extrap pos",
        );
        assert_eq!(s12.compound, Some(shape), "solid s12 compound 逐位保留");
        assert_quat_near(s12.state.rotation, Quat::IDENTITY, "solid s12 rot 不变");
    }

    /// T8 向过去冻结:早于首帧的 render_tick 直接返回首帧原值,不做反向外推。
    #[test]
    fn m02b_t8_freeze_towards_past() {
        let mut buf = InterpBuffer::new();
        let id = eid(8);
        buf.push(
            10,
            id,
            single_agent(
                Vec3::new(7.0, 0.0, 0.0),
                Quat::IDENTITY,
                Vec3::new(6.0, 0.0, 0.0),
            ),
        );

        let s9 = buf.sample(id, 9.0).expect("sample(9)");
        assert_eq!(
            s9.poses[0].translation.x, 7.0,
            "早于首帧必须逐位冻结,不得反向外推"
        );
        let s10 = buf.sample(id, 10.0).expect("sample(10)");
        assert_eq!(s10.poses[0].translation.x, 7.0, "首帧原值");
    }

    /// REQ-014 M04b-2 T21:`sync_agent_interp` 分档 full 保留跨档轨道、推入目标拍并初始化时钟,removed 清除轨道。
    #[test]
    fn sync_agent_interp_per_tier_full() {
        use sim_api::proto::{CompactAgentPose, LiteAgentPose, S2C};
        use types::{BodyDef, Tick};

        let toml = r#"
name = "two_interp"
[[links]]
name = "base"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[links]]
name = "leg"
mass = 1.0
[[links.shapes]]
kind = "box"
half = [0.1, 0.1, 0.1]
[[joints]]
parent = "base"
child = "leg"
type = "revolute"
axis = [0.0, 1.0, 0.0]
anchor = [0.0, 0.0, 0.0]
child_anchor = [-1.0, 0.0, 0.0]
default_angle = 0.0
"#;
        let def = BodyDef::from_toml_str(toml).expect("two_interp 合法");
        let hash = def.hash();
        let id1 = eid(1);
        let id2 = eid(2);

        let mut mirror = crate::entities::EntityMirror::new();
        mirror.set_defs([(hash, def)]);
        mirror.apply(&S2C::AgentMeta {
            entries: vec![(id1, hash), (id2, hash)],
        });
        mirror.apply(&S2C::AgentStatesCompact {
            tick: Tick(10),
            tier: 1,
            states: vec![CompactAgentPose {
                entity: id1,
                root_pos: [10.0, 0.0, 0.0],
                root_rot: [0x03, 0x00, 0x01, 0x80, 0x00, 0x40],
                root_linvel: [0u8; 6],
                child_links: vec![[
                    0xD0, 0x87, 0x00, 0x80, 0x00, 0x80, 0x03, 0x00, 0x01, 0x80, 0x00, 0x40,
                ]],
            }],
            removed: vec![],
            full: false,
        });
        // x=20: 20*32 + 32768 = 33408 = 0x8280 -> [0x80, 0x82]
        mirror.apply(&S2C::AgentStatesLite {
            tick: Tick(10),
            origin_chunk: [0u8; 8],
            states: vec![LiteAgentPose {
                entity: id2,
                rel_pos: [0x80, 0x82, 0x00, 0x80, 0x00, 0x80],
                yaw: [0, 0],
                motion_flags: 0x80,
            }],
            removed: vec![],
            full: true,
        });

        let mut buf: InterpBuffer<AgentView> = InterpBuffer::new();
        buf.push(10, id1, mirror.agents[&id1].clone());
        buf.push(10, id2, mirror.agents[&id2].clone());
        let mut clock = RenderClock::new(DEFAULT_BUFFER_TICKS);

        // 模拟 tier1 full=true 仅含 id1,镜像仍含跨档 id2。
        sync_agent_interp(&mut buf, &mut clock, &mirror, 20, &[id1], &[], true);
        assert_eq!(buf.track_len(id2), 1, "跨档轨道 id2 必须保留");
        assert_eq!(buf.track_len(id1), 2, "id1 应推入 tick=20");
        assert_eq!(
            buf.tracks.get(&id1).and_then(|t| t.back()).map(|(t, _)| *t),
            Some(20),
            "id1 末拍应为 20"
        );
        assert_f64_near(clock.now(), 8.0, "clock 首帧 20 - 12 = 8");

        sync_agent_interp(&mut buf, &mut clock, &mirror, 21, &[], &[id2], false);
        assert_eq!(buf.track_len(id2), 0, "removed 必须移除 id2 轨道");
    }
}
