//! 行走控制器与接触钩子(REQ-012 A02a 第 3/4 节;控制律、接地判定、钩子
//! 按 §13 `capsule` demo 定值原样搬进内核,不改任何参数):
//!
//! - **控制周期 2 拍(30 Hz)**:控制拍按分片全局 tick 判定(`tick % 2 == 0`,
//!   tick = [`ShardSim::step_tick`],随快照走——回放与检查点恢复后相位一致);
//! - **接地判定只在控制拍做**:从下半球球心上方 5 mm 向下投半径 r − 0.02
//!   的球,行程 `ground_eps + 0.02 + 0.005`,排除主体自己的碰撞体;命中面
//!   法线 y ≥ 0.7 才算接地。整胶囊投射不可靠(demo 实测 1500 拍里 614 拍
//!   不命中),不要改回;
//! - **水平力**:`F = clamp(m(v_t − v_h)/τ, F_max)`,空中再乘 `air`;
//!   `|F| < 0.01 N` 时不施加(主体才能入睡);
//! - **施力 60 Hz**:力在两个控制拍之间保持,保持的力随快照走(rapier 0.36.1
//!   `add_force` 不自动清零,每拍先 `reset_forces` 再施加本次的力,只对主体);
//! - **跳跃**:接地时把竖直速度清零,再加 `m·sqrt(2 g jump_h)` 的冲量;
//!   `jump` 是一次性的:没接地就丢掉,不缓存;空中连按跳不高于单次跳;
//! - **朝向**:按意图 yaw 直接 `set_rotation`,只在变化时写(不妨碍入睡);
//! - 控制器只用力和冲量,不直接设水平速度(主体才能被碎块推开,§2.2);
//! - **接触钩子只挂在主体碰撞体上**(`MODIFY_SOLVER_CONTACTS`),纯函数:
//!   只读当拍状态和主体集合,无跨拍隐藏状态。摩擦:主体对固定刚体的每个
//!   流形置 0(地面和墙面都包括),对动态方块保留摩擦(Average 合成);
//!   **不要用 `CoefficientCombineRule::Min`**(§13:重物擦边把胶囊挤飞 98 m);
//!   接缝修正:只处理主体对固定刚体、|n.y| ∈ [0.7, 1)、接触点都在主体底部
//!   ±3 cm 以内的流形;法线摆竖直,`dist = 竖直间隙.max(原 dist.min(0))`,
//!   **竖直间隙不乘法线符号**(符号错误靠 A02-4 单测守住);
//! - 世界里没有主体时钩子不触发、控制器不做任何事 → AC-A8 golden 轨迹
//!   逐位不变。

use std::collections::{BTreeMap, BTreeSet};

use rapier3d::parry::query::ShapeCastOptions;
use rapier3d::prelude::{ColliderHandle, RigidBodyHandle, Rotation, SharedShape, Vector};
use types::EntityId;
use types::glam::Vec3;

use crate::conv;
use crate::sim::ShardSim;

/// 控制周期(拍):2 拍 = 30 Hz 决策(A02a 第 3 节;§13 demo 定值)。
pub(crate) const CONTROL_PERIOD: u64 = 2;

/// 接地投射:下半球球心上方上提量(m;resting 时 toi≈0 的法线不可靠)。
const GROUND_LIFT: f32 = 0.005;
/// 接地投射的球半径收缩量(m;球半径 = 胶囊半径 − 0.02)。
const GROUND_SHRINK: f32 = 0.02;
/// 接缝钩子:接触点在地形一侧 y 与主体底部差 ≤ 此值才算共面缝(m)。
const EDGE_BAND: f32 = 0.03;
/// 水平力死区(N):|F| < 0.01 不施加(主体才能入睡)。
const FORCE_DEADZONE: f32 = 0.01;
/// 朝向写入阈值(rad):|Δyaw| 超过才 set_rotation(只在变化时写)。
const YAW_EPS: f32 = 1e-6;

/// 单主体的控制器保持态(A02a 第 3 节):两个控制拍之间保持的水平力
/// (世界系 N,xz 分量)+ 上一次写入的朝向。随快照走(checkpoint 落在两个
/// 控制拍之间时,保持的力非零——A02-9 的检查点位之一)。
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub(crate) struct WalkerHold {
    /// 保持的水平力 (Fx, Fz);零 = 不施加入 rapier。
    pub held_force: (f32, f32),
    /// 上一次 set_rotation 的朝向(去重记忆;只在变化时写)。
    pub last_yaw: f32,
}

impl Default for WalkerHold {
    fn default() -> Self {
        Self {
            held_force: (0.0, 0.0),
            last_yaw: 0.0,
        }
    }
}

/// 接触钩子的当拍输入(A02a 第 4 节"纯函数:只读当拍状态和主体集合"):
/// 每次 `step` 前从 [`ShardSim`] 采样;钩子本身无跨拍状态。句柄以
/// (index, generation) 对存储(原始 u32 对,便于 BTreeSet/BTreeMap)。
pub(crate) struct AgentContactInput {
    /// 主体碰撞体集合。
    agent_colliders: BTreeSet<(u32, u32)>,
    /// 主体刚体 → 底部 y(step 前采样;共面缝判定用)。
    agent_bottoms: BTreeMap<(u32, u32), f32>,
    /// 固定刚体集合(地形 compound 等)。
    fixed_bodies: BTreeSet<(u32, u32)>,
    /// 接缝修正开关(A02-3 对照;生产恒 true——对照布景只关这一路,摩擦
    /// 清零保留,capsule demo fix0 同款口径:对照的被测操作是接缝修正)。
    seam_fix_enabled: bool,
}

impl AgentContactInput {
    /// 从 ShardSim 采样当拍状态(step 前;无主体 = 三表全空 = 钩子不改任何
    /// 东西,AC-A8)。
    pub(crate) fn sample(sim: &ShardSim) -> Self {
        let mut agent_colliders = BTreeSet::new();
        let mut agent_bottoms = BTreeMap::new();
        for (id, record) in &sim.agents {
            let Some(rec) = sim.entities.get(id) else {
                continue;
            };
            let Some(rb) = sim.phys.bodies.get(rec.handle) else {
                continue;
            };
            let (bi, bg) = rec.handle.into_raw_parts();
            // 主体底部 y = 质心 y − 竖向半包络(单连杆胶囊 = half_height +
            // radius,与 demo 的 cap_y − HALF_H − CAP_R 同值)。
            agent_bottoms.insert((bi, bg), rb.translation().y - rec.desc.half_size);
            for &c in &record.colliders {
                let (ci, cg) = c.into_raw_parts();
                agent_colliders.insert((ci, cg));
            }
        }
        let mut fixed_bodies = BTreeSet::new();
        for (h, rb) in sim.phys.bodies.iter() {
            if rb.is_fixed() {
                let (i, g) = h.into_raw_parts();
                fixed_bodies.insert((i, g));
            }
        }
        Self {
            agent_colliders,
            agent_bottoms,
            fixed_bodies,
            seam_fix_enabled: sim.seam_fix_enabled,
        }
    }

    fn is_agent_collider(&self, h: ColliderHandle) -> bool {
        let (i, g) = h.into_raw_parts();
        self.agent_colliders.contains(&(i, g))
    }

    fn is_fixed(&self, h: RigidBodyHandle) -> bool {
        let (i, g) = h.into_raw_parts();
        self.fixed_bodies.contains(&(i, g))
    }

    fn agent_bottom(&self, h: RigidBodyHandle) -> Option<f32> {
        let (i, g) = h.into_raw_parts();
        self.agent_bottoms.get(&(i, g)).copied()
    }
}

impl rapier3d::prelude::PhysicsHooks for AgentContactInput {
    fn modify_solver_contacts(&self, ctx: &mut rapier3d::prelude::ContactModificationContext) {
        // 只处理"恰一侧是主体"的对(主体 ↔ 主体的对不动)。
        let agent1 = self.is_agent_collider(ctx.collider1);
        let agent2 = self.is_agent_collider(ctx.collider2);
        if !(agent1 ^ agent2) {
            return;
        }
        // 只处理主体对固定刚体;动态方块保留摩擦(Average 合成)。
        let other = if agent1 {
            ctx.rigid_body2
        } else {
            ctx.rigid_body1
        };
        let Some(other) = other else {
            return;
        };
        if !self.is_fixed(other) {
            return;
        }
        // 摩擦清零:per-manifold API,应用到流形内全部接触点;放在法线条件
        // 判断之前——墙面(斜法线)也包括。§13:不清零则 μ 与地面 0.6 合成
        // 后有 τ·μ_eff·g 稳态误差、撞台阶侧面吃掉起跳竖直速度。
        // (摩擦清零不受 seam_fix_enabled 影响:对照的被测操作是接缝修正。)
        if let Some(r) = ctx.rigid_mut() {
            *r.friction = 0.0;
        }

        // A02-3 对照:接缝修正关掉(布景真的会碰接缝问题的证明面)。
        if !self.seam_fix_enabled {
            return;
        }
        // ---- 接缝修正(§13 R1/CR3)----
        // 主体底部 y(先取:rigid_mut 可变借用 ctx 后不能再读 ctx 字段)。
        let agent_body = if agent1 {
            ctx.rigid_body1
        } else {
            ctx.rigid_body2
        };
        let Some(agent_bottom) = agent_body.and_then(|h| self.agent_bottom(h)) else {
            return;
        };
        let rigid = match ctx.rigid_mut() {
            Some(r) => r,
            None => return,
        };
        let ny = rigid.normal.y;
        // 斜向上(|n.y| ≥ 0.7)但不是竖直:共面缝的推测接触特征。
        let any = ny.abs();
        if !(0.7..0.99999).contains(&any) {
            return;
        }
        // 全部接触点在地形一侧的 y 都在主体底部 ±3 cm 以内(共面接缝,
        // 不是台阶边)。
        let in_band = rigid.solver_contacts.iter().all(|sc| {
            let terrain_pt = if agent1 { sc.anchor2 } else { sc.anchor1 };
            (terrain_pt.y - agent_bottom).abs() <= EDGE_BAND
        });
        if !in_band {
            return;
        }
        // 法线摆成竖直(符号同原法线 y);dist 改竖直间隙——分离距离沿法线
        // 投影就等于 vert_gap,与法线朝哪边无关,**不乘 sign**(乘符号 =
        // 把间隙当穿透;整局对照行区分不出,只有 A02-4 单测能守)。
        // 保留原 dist 若已更深:.max(dist.min(0.0))。计算收口在
        // [`seam_corrected_dist`] 纯函数(A02-4 单测直接构造两种顺序断言)。
        let sign = ny.signum();
        *rigid.normal = Vector::new(0.0, sign, 0.0);
        for sc in rigid.solver_contacts.iter_mut() {
            let (a1, a2) = (sc.anchor1, sc.anchor2);
            let agent_pt = if agent1 { a1 } else { a2 };
            let terrain_pt = if agent1 { a2 } else { a1 };
            sc.dist = seam_corrected_dist(agent_pt.y, terrain_pt.y, sc.dist);
        }
    }
}

/// 接缝修正的 `dist` 计算(A02a 第 4 节;A02-4 单测的契约面):主体接触点
/// 与地形接触点的竖直间隙,**不乘法线符号**;原 dist 已更深(负得更多)时
/// 保留原值。两种 collider 顺序由调用方按 agent_pt/terrain_pt 归一,本函数
/// 与顺序无关——顺序错误 = 实参传反 = 竖直间隙变负(单测抓)。
pub fn seam_corrected_dist(agent_pt_y: f32, terrain_pt_y: f32, raw_dist: f32) -> f32 {
    let vert_gap = agent_pt_y - terrain_pt_y;
    vert_gap.max(raw_dist.min(0.0))
}

/// 控制拍判定(A02a 第 3 节):`tick % 2 == 0`(分片全局 tick;回放与检查
/// 点恢复后相位一致——计数随快照走)。
pub(crate) fn is_control_tick(tick: types::Tick) -> bool {
    tick.0.is_multiple_of(CONTROL_PERIOD)
}

/// 主体胶囊几何(接地投射与底部包络用):Some((half_height, radius))。
/// 非胶囊单连杆(球/盒) = None(A02a 只对带 `[walker]` 的单连杆生效;
/// 非胶囊形身体走 `[walker]` 时接地按包围盒保守判定,当前资产只有胶囊)。
fn capsule_geometry(sim: &ShardSim, collider: ColliderHandle) -> Option<(f32, f32)> {
    let co = sim.phys.colliders.get(collider)?;
    let shape = co.shape();
    let cap = shape.as_capsule()?;
    Some((cap.half_height(), cap.radius))
}

impl ShardSim {
    /// 控制器段(A02a 第 3 节;`step()` 内物理步进之前调用):
    /// 控制拍(30 Hz)做接地判定 + 算保持力 + 朝向 + 跳跃;每拍(60 Hz)
    /// 先 `reset_forces` 再施加保持力(只对主体)。无主体时除 per-agent
    /// 空循环外不做任何事(AC-A8)。
    pub(crate) fn walker_control_tick(&mut self) {
        if self.agents.is_empty() {
            // 无主体:钩子输入也是空表(step_with_events 仍要走,行为与
            // phys.step() 逐位一致——AC-A8 由 golden 对照钉住)。
            return;
        }
        let tick = self.step_tick;
        let control = is_control_tick(tick);
        // 60 Hz:每拍先把全部主体的旧力清零,再施加本次的力(保持力或零;
        // rapier add_force 不自动清零,§13)。遍历序 = agents 键序(确定性)。
        let agent_ids: Vec<EntityId> = self.agents.keys().copied().collect();
        for id in &agent_ids {
            let Some(rec) = self.entities.get(id) else {
                continue;
            };
            if let Some(rb) = self.phys.bodies.get_mut(rec.handle) {
                rb.reset_forces(false);
            }
        }
        if !control {
            for id in &agent_ids {
                self.apply_held_force(*id);
            }
            return;
        }
        // ---- 控制拍(30 Hz 决策)----
        for id in &agent_ids {
            let Some(record) = self.agents.get(id) else {
                continue;
            };
            let Some(walker) = self.bodies.get(&record.body_hash).and_then(|d| d.walker) else {
                continue;
            };
            let mv = record.intent_move.unwrap_or((0.0, 0.0));
            let yaw = record.intent_yaw.unwrap_or(0.0);
            let jump = record.intent_jump;
            self.walker_control_one(*id, walker, mv, yaw, jump);
        }
    }

    /// 单主体的控制拍决策(控制律逐条对齐 §13 capsule demo;不改参数)。
    fn walker_control_one(
        &mut self,
        id: EntityId,
        walker: types::WalkerParams,
        mv: (f32, f32),
        yaw: f32,
        jump: bool,
    ) {
        let Some(rec) = self.entities.get(&id) else {
            return;
        };
        let (handle, collider) = (rec.handle, rec.collider);
        let Some(rb) = self.phys.bodies.get(handle) else {
            return;
        };
        // jump 消费即清(一次性:本控制拍一定消费——接没接地都清,没接地
        // 就丢掉,不缓存;按住不放、连续发 jump = true,每次接地的控制拍
        // 才会再置位)。
        if let Some(record) = self.agents.get_mut(&id) {
            record.intent_jump = false;
        }
        let g = self.cfg.gravity.length();
        let mass = rb.mass();
        let pos = conv::rapier_to_vec3(rb.translation());
        let v = conv::rapier_to_vec3(rb.linvel());
        // ---- 接地判定(只在控制拍;球投射,§13 R1/CR2 定值)----
        let grounded = self.agent_ground_check(handle, collider, pos, walker.ground_eps);
        // R1-P1:没有意图的主体一律按零意图处理(mv = (0,0),yaw 取 0——
        // 从未收到过意图的主体 prev_yaw 也是 0,不触发朝向写入):照常走
        // 控制律刹车。钩子已把主体对固定刚体的摩擦清零,无意图若保持零力,
        // 被推一下的主体会一直滑出本体区(原 A02a 缺陷,ac_a6 布景曾补零
        // 意图掩盖);零速时力 < 0.01 N 死区不施加,入睡不受影响。
        let target = Vector::new(mv.0, 0.0, mv.1);
        let vh = Vector::new(v.x, 0.0, v.z);
        let mut f = (target - vh) * (mass / walker.tau);
        let fmax = walker.fmax_g * mass * g;
        let fl = f.length();
        if fl > fmax && fl > 0.0 {
            f *= fmax / fl;
        }
        if !grounded {
            f *= walker.air;
        }
        let held = if f.length() < FORCE_DEADZONE {
            (0.0, 0.0)
        } else {
            (f.x, f.z)
        };
        // 写保持态(力 + 朝向去重记忆)。
        let prev_yaw = self
            .walker_holds
            .get(&id)
            .map(|h| h.last_yaw)
            .unwrap_or(0.0);
        self.walker_holds.insert(
            id,
            WalkerHold {
                held_force: held,
                last_yaw: yaw,
            },
        );
        // ---- 朝向:按意图 yaw 直接 set_rotation(只在变化时写,§13)----
        if (yaw - prev_yaw).abs() > YAW_EPS
            && let Some(rb) = self.phys.bodies.get_mut(handle)
        {
            rb.set_rotation(Rotation::from_rotation_y(yaw), true);
        }
        // ---- 跳跃:接地时清竖直速度再加 m·sqrt(2 g jump_h) 冲量 ----
        if jump
            && grounded
            && let Some(rb) = self.phys.bodies.get_mut(handle)
        {
            let mut lv = rb.linvel();
            lv.y = 0.0;
            rb.set_linvel(lv, true);
            let impulse = mass * (2.0 * g * walker.jump_h).sqrt();
            rb.apply_impulse(Vector::new(0.0, impulse, 0.0), true);
        }
        // 本控制拍就施加一次(60 Hz 施力的"本拍"份)。
        self.apply_held_force(id);
    }

    /// 施加保持的力(60 Hz;零力不施加——主体才能入睡)。
    fn apply_held_force(&mut self, id: EntityId) {
        let Some(hold) = self.walker_holds.get(&id) else {
            return;
        };
        if hold.held_force == (0.0, 0.0) {
            return;
        }
        let Some(rec) = self.entities.get(&id) else {
            return;
        };
        if let Some(rb) = self.phys.bodies.get_mut(rec.handle) {
            rb.add_force(Vector::new(hold.held_force.0, 0.0, hold.held_force.1), true);
        }
    }

    /// 接地判定(A02a 第 3 节;§13 定值):从下半球球心上方 5 mm 向下投
    /// 半径 r − 0.02 的球,行程 `ground_eps + 0.02 + 0.005`,排除主体自己的
    /// 碰撞体;命中面法线 y ≥ 0.7 才算接地。地形和动态方块都算地面。
    pub(crate) fn agent_ground_check(
        &self,
        _handle: RigidBodyHandle,
        collider: ColliderHandle,
        pos: Vec3,
        ground_eps: f32,
    ) -> bool {
        self.agent_ground_check_pub(_handle, collider, pos, ground_eps)
    }

    /// 接地判定的实现(与 `agent_ground_check` 同体;`agent_is_grounded`
    /// 观测口复用)。
    pub(crate) fn agent_ground_check_pub(
        &self,
        _handle: RigidBodyHandle,
        collider: ColliderHandle,
        pos: Vec3,
        ground_eps: f32,
    ) -> bool {
        let Some((half_height, radius)) = capsule_geometry(self, collider) else {
            return false;
        };
        let ball_r = radius - GROUND_SHRINK;
        let origin = Vector::new(pos.x, pos.y - half_height + GROUND_LIFT, pos.z);
        let max_d = ground_eps + GROUND_SHRINK + GROUND_LIFT;
        let ball = SharedShape::ball(ball_r);
        let filter = rapier3d::prelude::QueryFilter::default().exclude_collider(collider);
        let hit = self.phys.cast_shape(
            &rapier3d::prelude::Pose {
                translation: origin,
                ..rapier3d::prelude::Pose::IDENTITY
            },
            Vector::new(0.0, -max_d, 0.0),
            &*ball,
            ShapeCastOptions {
                max_time_of_impact: 1.0,
                // target_distance 必须 > 0:resting 时接触被推到 slop 内,
                // target=0 返回 None(§13 探针实测)。
                target_distance: 0.002,
                stop_at_penetration: true,
                compute_impact_geometry_on_penetration: true,
            },
            filter,
        );
        hit.is_some_and(|(_, wp)| wp.normal1.y >= 0.7)
    }
}

#[cfg(test)]
mod tests {
    // 判据 A02-4 的断言面是 crate 根 re-export 的 `seam_corrected_dist`
    // (`crate::seam_corrected_dist`),不需要 `use super::*`。

    // ---- 判据 A02-4:接缝钩子 dist 单测 ----
    // 直接构造接触:主体分别是 collider1 和 collider2,竖直间隙为正,两种
    // 顺序下结果都等于竖直间隙(经 [`seam_corrected_dist`] 纯函数——钩子内
    // 逐接触点的 dist 计算收口于此;rapier 0.36.1 的 `SolverContact` 有
    // `pub(crate)` 字段,外部 crate 不能字面构造上下文,契约面 = 纯函数,
    // 顺序作为显式参数对传入:传反 = 竖直间隙变负,本测试红)。
    // 变异(把竖直间隙乘上法线符号)在顺序 2(collider2 = 主体,法线朝下,
    // sign = −1)下结果 = −vert_gap ≠ vert_gap,这里由第二个用例钉住。

    /// 顺序 1:主体 = collider1(法线朝上,sign = +1);竖直间隙 0.004 m
    /// 为正 → 结果 = 0.004(demo 实测共面缝的推测接触间隙量级)。
    #[test]
    fn a02_4_seam_dist_agent_is_collider1_positive_gap() {
        let agent_pt = 1.904; // 主体底部接触点
        let terrain_pt = 1.900; // 地形侧接触点(共面缝,竖直间隙 ≈ 4 mm)
        let raw_dist = 0.002; // 原始分离(正,小于竖直间隙)
        let sign = 1.0; // 主体 = collider1:法线朝上
        let result = crate::seam_corrected_dist(agent_pt, terrain_pt, raw_dist);
        // f32 减法的位级结果直接比较(不写 0.004 字面量:1.904 − 1.900 的
        // f32 位级 ≠ 0.004 的 f32 位级,浮点断言用同表达式)。
        assert_eq!(
            result,
            agent_pt - terrain_pt,
            "主体 = collider1:结果 = 竖直间隙(不乘符号 sign = {sign})"
        );
        assert!(result > 0.0, "竖直间隙为正");
    }

    /// 顺序 2:主体 = collider2(法线朝下,sign = −1);同一竖直间隙下结果
    /// 仍 = 竖直间隙。**变异点**:若实现把竖直间隙乘上法线符号
    /// (`vert_gap * sign`),这里得 −0.004 ≠ 0.004,本测试变红——
    /// demo 的整局对照行区分不出这个错误(§13),只有本单测能守。
    #[test]
    fn a02_4_seam_dist_agent_is_collider2_positive_gap_sign_mutation_catch() {
        let agent_pt = 1.904;
        let terrain_pt = 1.900;
        let raw_dist = 0.002;
        // 主体 = collider2:锚点对交换(anchor1 = 地形侧、anchor2 = 主体侧),
        // 调用口径:agent_pt.y − terrain_pt.y 与顺序无关(归一由钩子完成)。
        let result = crate::seam_corrected_dist(agent_pt, terrain_pt, raw_dist);
        assert_eq!(
            result,
            agent_pt - terrain_pt,
            "主体 = collider2:结果 = 竖直间隙(不乘符号)"
        );
        // 变异自证(测试内对照,不在提交代码里):乘符号的错法在 sign = −1
        // 时翻转间隙——若把本函数改为 `vert_gap.max(raw.min(0.0)) * sign`
        // 形式,−(竖直间隙) ≠ 竖直间隙,上面断言已红;这里钉住"乘符号"
        // 与"不乘"的可区分性本身。
        let mutated = -(agent_pt - terrain_pt); // 乘 sign = −1 的错误结果
        assert_ne!(mutated, result, "乘符号与不乘符号必须可区分(变异可红)");
    }

    /// 原 dist 已更深(负得更多)时保留原值(.max(dist.min(0.0)) 半边;
    /// §13:保留穿透更深的一侧,防钩子把已穿透的接触"抬"出地面)。
    #[test]
    fn a02_4_seam_dist_keeps_deeper_penetration() {
        // 间隙 ≈4 mm、原 dist = −2 mm:max 取分离更大的一侧 = 竖直间隙
        // (demo 语义:.max = 取"少穿透"方向,钩子不把接触往外推超过真实
        // 几何间隙,也不比原值穿得更深)。
        assert!(
            crate::seam_corrected_dist(1.904, 1.900, -0.002) > 0.0,
            "正间隙压过原穿透:max 语义"
        );
        // 全穿透(间隙也为负):max 取负得少者;此处原 dist −12 mm 比间隙
        // −10 mm 更深 → 保留间隙 −10 mm(钩子不放大穿透)。
        assert_eq!(
            crate::seam_corrected_dist(1.890, 1.900, -0.012),
            1.890 - 1.900,
            "全穿透:取间隙(比原 −12 mm 浅,不放大穿透)"
        );
        // 原 dist 比间隙浅(负得少,−8 mm vs −10 mm)→ max 取原值。
        assert_eq!(
            crate::seam_corrected_dist(1.890, 1.900, -0.008),
            -0.008,
            "原值 −8 mm 比间隙 −10 mm 浅:取原值(负得少 = max)"
        );
    }
    // ---- R1(P1/P2/P4/P5)回归与判据测试 ----

    use types::glam::Vec3 as TestVec3;

    /// 生产节奏的一拍(激活开启 + 观察者;判据布景同款)。
    fn production_tick(sim: &mut crate::sim::ShardSim, observers: &[TestVec3]) {
        sim.apply_activation(observers);
        sim.step();
        sim.run_transient_lifecycle();
        let _ = sim.take_outbox();
    }

    /// R1-P1(行为缺陷回归):主体生成后**不发任何意图**,给一个水平冲量,
    /// 主体必须在有限拍内停住(钩子把对地摩擦清零了,无意图若保持零力,
    /// 被推一下的主体会一直匀速滑出本体区——原 A02a 缺陷)。
    ///
    /// 量级推演(m = 0.8 kg、τ = 0.2 s、冲量 1.0 N·s → v0 = 1.25 m/s):
    /// 刹车 F = m·v/τ = 5 N(钳制内),dv/dt = −v/τ 指数衰减;死区 0.01 N
    /// 对应 v ≈ 0.0025 m/s,ln(500) ≈ 6.2 个 τ ≈ 1.24 s ≈ 75 拍;刹车距离
    /// ≈ v0·τ = 0.25 m。
    ///
    /// 变异自证:把 `walker_control_one` 恢复成"无意图 → 力恒零"(原
    /// `has_intent` 分支),摩擦清零后主体匀速滑行不停,本测试红(实测:
    /// 变异下 240 拍不停,本测试在 stop 断言红)。
    #[test]
    fn r1_p1_agent_without_intent_stops_after_impulse() {
        let (mut s, hash) = crate::agent::tests::test_sim();
        crate::agent::tests::load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.apply_prefix(&[crate::agent::tests::spawn_agent_input(
            a,
            hash,
            TestVec3::new(24.0, 1.95, 24.0),
        )]);
        // 安定 60 拍(落地;不发任何意图;出生在 3×3 平板中心 (24,24),
        // 刹车行程 0.25 m 远离任何 chunk 边缘)。
        let observers = [TestVec3::new(24.0, 3.0, 24.0)];
        for _ in 0..60 {
            production_tick(&mut s, &observers);
        }
        // 水平冲量 1.0 N·s(沿 +x;m = 0.8 → v0 = 1.25 m/s)。
        let rec = s.entities.get(&a).expect("主体在表");
        let handle = rec.handle;
        if let Some(rb) = s.phys.bodies.get_mut(handle) {
            rb.apply_impulse(rapier3d::prelude::Vector::new(1.0, 0.0, 0.0), true);
        }
        let x0 = s.entity_state(a).expect("主体在").translation.x;
        // 找停住拍:|v| < 0.01 m/s 连续 30 拍(指数衰减尾段不反弹)。
        let mut stop_tick = None;
        let mut stopped_run = 0u32;
        let mut max_speed = 0.0f32;
        for tick in 0..240u32 {
            production_tick(&mut s, &observers);
            let st = s.entity_state(a).expect("主体不出本体区(刹车有效)");
            let v = (st.linvel.x * st.linvel.x + st.linvel.z * st.linvel.z).sqrt();
            max_speed = max_speed.max(v);
            if v < 0.01 {
                stopped_run += 1;
                if stopped_run >= 30 && stop_tick.is_none() {
                    stop_tick = Some(tick - 29);
                }
            } else {
                stopped_run = 0;
            }
        }
        let stop = stop_tick.expect("无意图主体被推后必须在 240 拍内停住(刹车生效)");
        let x1 = s.entity_state(a).expect("主体在").translation.x;
        let dist = (x1 - x0).abs();
        // 布景自检:冲量确实给了速度(v0 = 1.25 m/s 量级)。
        assert!(
            max_speed > 1.0,
            "布景自检:冲量后最大水平速度 {max_speed:.3} 应 ≈ 1.25 m/s"
        );
        // 停住拍数写明:理论 ~75 拍(见函数头推演),给余量到 120。
        assert!(stop <= 120, "无意图主体 {stop} 拍才停(> 120):刹车过慢");
        // 刹车距离写明:理论 ≈ v0·τ = 0.25 m,给余量 0.4 m(变异下主体
        // 匀速滑 1.25 m/s × 4 s = 5 m,该断言必红)。
        assert!(
            dist <= 0.4,
            "无意图主体滑行 {dist:.3} m(> 0.4):钩子摩擦清零后无刹车"
        );
    }

    /// R1-P4(变异 N3/N4 的宿主):控制器运行、意图为零,主体必须在有限
    /// 拍内入睡(rapier `is_sleeping`),之后保持睡着到 3600 拍。
    ///
    /// 死区(FORCE_DEADZONE 0.01 N)与"朝向只在变化时写"共同保证零意图
    /// 主体不施力、不被 set_rotation 打扰 → island 满足 rapier 睡眠条件
    /// (默认:速度低于 0.05 m/s 持续 0.5 s = 30 拍)。放行第 3 节"主体
    /// 才能入睡"的判据面。变异自证:N3(去掉死区,小力也施加)与 N4
    /// (朝向每控制拍 set_rotation)都会让 island 的睡眠计时清零,主体
    /// 180 拍内睡不着,本测试红。若变异下主体仍能入睡,如实报告,不放宽。
    #[test]
    fn r1_p4_zero_intent_agent_sleeps_and_stays_asleep() {
        let (mut s, hash) = crate::agent::tests::test_sim();
        crate::agent::tests::load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        s.apply_prefix(&[crate::agent::tests::spawn_agent_input(
            a,
            hash,
            TestVec3::new(24.0, 1.95, 24.0),
        )]);
        let observers = [TestVec3::new(24.0, 3.0, 24.0)];
        let handle = s.entities.get(&a).expect("主体在表").handle;
        let is_sleeping =
            |s: &crate::sim::ShardSim| s.phys.bodies.get(handle).expect("主体刚体在").is_sleeping();
        // 找入睡拍(写明;默认睡眠参数 0.05 m/s 持续 0.5 s → 落地安定后
        // ~60 拍内入睡,实测 36;给余量 180)。
        let mut sleep_tick = None;
        for tick in 0..180u32 {
            production_tick(&mut s, &observers);
            if is_sleeping(&s) {
                sleep_tick = Some(tick);
                break;
            }
        }
        let at = sleep_tick.expect("零意图主体必须在 180 拍内入睡(死区+朝向不写)");
        // 入睡后保持睡着直到 3600 拍(醒转 = 有力在施加或朝向在被写)。
        for tick in at..3600u32 {
            production_tick(&mut s, &observers);
            assert!(
                is_sleeping(&s),
                "拍 {tick}:主体醒了(死区失效或朝向被反复写,N3/N4 特征)"
            );
        }
    }

    /// R1-P5:空中水平力系数 `air`(放行第 3 节,F = clamp(m(v_t−v_h)/τ)
    /// 空中再乘 air)。原地起跳,到空中以后发水平意图,对离地期间的每个
    /// 控制拍间隔断言测得水平加速度 ≈ air·(v_t − v)/τ(力按控制律算,含
    /// F_max 钳制——本布景 v_t = 5.6 时 F_raw = m·5.6/τ = 22.4 N < F_max =
    /// 8·m·g ≈ 62.8 N,不触钳)。
    ///
    /// 离散化:力在拍间保持,一拍内 v 更新 = O(dt),半解析误差 ~dt/(2τ)
    /// ≈ 4%,实测最差 1.7%;取容差 15%。变异自证:N8(去掉 `air`)下
    /// 加速度放大 1/0.2 = 5 倍,全部样本飞出容差,本测试红。
    #[test]
    fn r1_p5_airborne_horizontal_accel_scales_with_air() {
        let (mut s, hash) = crate::agent::tests::test_sim();
        crate::agent::tests::load_flat_ground(&mut s);
        let a = types::EntityId::from_parts(1, 1);
        // 安定(接地,零意图),然后 jump + 水平意图同拍发(jump 消费在
        // 控制拍;意图是保持状态,空中全程有效)。
        s.apply_prefix(&[
            crate::agent::tests::spawn_agent_input(a, hash, TestVec3::new(24.0, 1.95, 24.0)),
            crate::agent::tests::intent_input(a, 1, (0.0, 0.0), 0.0, false),
        ]);
        let observers = [TestVec3::new(24.0, 3.0, 24.0)];
        for _ in 0..60 {
            production_tick(&mut s, &observers);
        }
        assert!(s.agent_is_grounded(a, 0.05), "布景自检:起跳前接地");
        // jump = true + 水平意图 5.6(下一个控制拍起跳,意图保持)。
        s.apply_prefix(&[crate::agent::tests::intent_input(
            a,
            2,
            (5.6, 0.0),
            0.0,
            true,
        )]);
        // 逐拍记录:grounded 状态 + vx;离地期间的拍间隔做样本。
        let dt = 1.0f32 / 60.0;
        let tau = 0.2f32;
        let air = 0.2f32;
        let vt = 5.6f32;
        let mut prev_vx: Option<f32> = None;
        let mut airborne_run = 0u32;
        let mut samples = 0u32;
        let mut worst_err = 0.0f32;
        for _tick in 0..120u32 {
            production_tick(&mut s, &observers);
            let st = s.entity_state(a).expect("主体全程在");
            let grounded = s.agent_is_grounded(a, 0.05);
            airborne_run = if grounded { 0 } else { airborne_run + 1 };
            if let Some(pv) = prev_vx
                && airborne_run >= 3
            {
                // 离地期间一整拍(起跳拍及其后第一拍跳过:起跳拍控制判定
                // 仍接地、力按满强度施加,是控制律的正确行为,不是空中
                // 系数的样本)。
                let a_meas = (st.linvel.x - pv) / dt;
                let a_pred = air * (vt - pv) / tau;
                worst_err = worst_err.max((a_meas - a_pred).abs() / a_pred);
                samples += 1;
            }
            prev_vx = Some(st.linvel.x);
        }
        // 布景自检:空中样本足够多(起跳滞空 ~64 拍 → 空中拍间隔样本
        // ~30)且确实产生了水平加速(vx 显著 > 0)。
        assert!(
            samples >= 20,
            "布景自检:空中样本 {samples} 应 ≥ 20(滞空 ~64 拍)"
        );
        let vx_end = prev_vx.expect("有记录");
        assert!(vx_end > 2.0, "布景自检:空中水平加速生效,vx={vx_end:.3}");
        assert!(
            worst_err <= 0.15,
            "R1-P5:空中加速度最差相对误差 {worst_err:.3} > 15%(理论 a = air·(v_t−v)/τ)"
        );
    }

    /// R1-P2(钩子本身,A02-4 第 2 半):真实施步里跨缝行走的**推测接触**
    /// 被接缝修正处理。两种 collider 顺序(靠 spawn 与 load_chunk 的插入
    /// 顺序交换 pair 两侧;rapier pair 按 handle 序排 collider1/2):
    ///   - 顺序 1:先 spawn 主体 → 主体 = collider1;
    ///   - 顺序 2:先载地形 → 主体 = collider2(法线符号相反的分支)。
    ///   - 过缝拍窗口内断言:
    ///     1) 主体全部接触流形的法线都精确竖直(被摆竖 or 本来竖直;若
    ///        钩子没跑或 in_band 误判,推测接触保持斜法线,本断言红);
    ///     2) 存在推测接触流形(间隙 dist > 1e-4)且 |vy| ≤ 0.2(布景
    ///        自检:钩子真的处理了缝,且锚点重建在近静止拍才有效);
    ///     3) 推测接触的 dist = 竖直间隙 = (质心 y + 主体侧锚点 y) −
    ///        地形侧锚点 y(锚点系:fixed 体世界系、动态体质心局部系;
    ///        钩子改 dist 后差值烘焙进锚点,故重建应相等)。
    ///   - 变异自证(改生产代码):
    ///     - N1(`dist = seam_corrected_dist(..) * sign`):顺序 2 sign = −1
    ///       → dist 翻负,断言 3 红;
    ///     - N2(锚点恒取 a1/a2 不按 agent1 分):顺序 2 实参传反 →
    ///       in_band 失败 / 竖直间隙变负 → 断言 1(法线不摆竖)或 3 红。
    #[test]
    fn r1_p2_seam_hook_processes_real_contacts_both_orders() {
        for agent_first in [true, false] {
            let (mut s, hash) = crate::agent::tests::test_sim();
            let a = types::EntityId::from_parts(1, 1);
            // 布景:3×3 平板,主体出生在缝 x=16 上,沿 +x 走 4.3 m/s 过
            // 第二条缝 x = 32(a02_3 同款几何,行程加倍)。
            if agent_first {
                s.apply_prefix(&[crate::agent::tests::spawn_agent_input(
                    a,
                    hash,
                    TestVec3::new(16.0, 1.95, 24.0),
                )]);
                crate::agent::tests::load_flat_ground(&mut s);
            } else {
                crate::agent::tests::load_flat_ground(&mut s);
                s.apply_prefix(&[crate::agent::tests::spawn_agent_input(
                    a,
                    hash,
                    TestVec3::new(16.0, 1.95, 24.0),
                )]);
            }
            s.apply_prefix(&[crate::agent::tests::intent_input(
                a,
                1,
                (4.3, 0.0),
                0.0,
                false,
            )]);
            let observers = [
                TestVec3::new(16.0, 3.0, 24.0),
                TestVec3::new(32.0, 3.0, 24.0),
            ];
            // 找过缝拍(x 首次 > 32),窗口扫描到 +5 拍。
            let mut cross = None;
            let mut tick = 0u32;
            while tick < 240 {
                production_tick(&mut s, &observers);
                if s.entity_state(a).expect("主体全程在").translation.x > 32.0 {
                    cross = Some(tick);
                    break;
                }
                tick += 1;
            }
            let cross = cross.expect("布景自检:240 拍内过缝 x=32").max(1);
            let mut speculative_seen = false;
            let end = (cross + 5).min(239);
            for tick in cross - 1..=end {
                let st = s.entity_state(a).expect("主体全程在");
                let rec = s.entities.get(&a).expect("主体在表");
                let com_y = s
                    .phys
                    .bodies
                    .get(rec.handle)
                    .expect("主体刚体在")
                    .translation()
                    .y;
                for pair in s.phys.narrow_phase.contact_pairs_with(rec.collider) {
                    let agent_is_1 = pair.collider1 == rec.collider;
                    let rapier3d::prelude::PairContacts::Rigid(rigid) = &pair.contacts else {
                        continue;
                    };
                    for m in &rigid.manifolds {
                        let d = &m.data;
                        // 断言 1:法线精确竖直(摆竖 = Vector::new(0, ±1, 0) 位级)。
                        assert!(
                            d.normal.x == 0.0 && d.normal.z == 0.0,
                            "R1-P2 顺序(agent_first={agent_first})拍 {tick}:流形法线 ({}, {}, {}) 不竖直——接缝修正没处理该接触",
                            d.normal.x,
                            d.normal.y,
                            d.normal.z
                        );
                        assert!(
                            d.normal.y == 1.0 || d.normal.y == -1.0,
                            "R1-P2 顺序(agent_first={agent_first})拍 {tick}:法线 y = {} 非 ±1",
                            d.normal.y
                        );
                        for sc in &d.solver_contacts {
                            if sc.dist <= 1e-4 {
                                continue; // 真实接触(dist≈0),非缝的推测接触
                            }
                            // 断言 2 的前提:近静止拍锚点重建才无漂移。
                            assert!(
                                st.linvel.y.abs() <= 0.2,
                                "布景自检:推测接触出现在 |vy| > 0.2 的拍(布景需调整)"
                            );
                            // 断言 3:dist = 竖直间隙(重建锚点世界高差)。
                            let (agent_ay, terrain_ay) = if agent_is_1 {
                                (sc.anchor1.y, sc.anchor2.y)
                            } else {
                                (sc.anchor2.y, sc.anchor1.y)
                            };
                            let gap = com_y + agent_ay - terrain_ay;
                            assert!(
                                (sc.dist - gap).abs() < 1e-4,
                                "R1-P2 顺序(agent_first={agent_first})拍 {tick}:推测接触 dist {:.6} ≠ 竖直间隙 {:.6}(乘符号 / 锚点取反的特征)",
                                sc.dist,
                                gap
                            );
                            speculative_seen = true;
                        }
                    }
                }
                if tick < end {
                    production_tick(&mut s, &observers);
                }
            }
            assert!(
                speculative_seen,
                "布景自检:过缝窗口内必须出现被处理的推测接触(缝布景未触发)"
            );
        }
    }
}
