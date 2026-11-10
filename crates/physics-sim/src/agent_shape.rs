//! 全局主体形状表(REQ-012 §6"暂缓写入",A05)。
//!
//! 每拍开始、分片步进之前,调度器汇总**上一拍末**全部主体的连杆碰撞体与
//! 世界位姿,作为本拍的输入交给每个分片;迁移在途的主体取 Router 信封里
//! 的位姿。分片的"格与主体重叠"判定**只查这份表**,不查自己的 `agents`:
//! 拥有该格的分片和持有副本的邻片输入相同 ⇒ 结论相同,副本不分叉。
//!
//! 表是派生状态,不进快照;恢复后由快照里的主体状态与在途信封重算
//! (调度器每拍重建,无跨拍状态)。条目按 `EntityId` 升序(构造侧
//! BTreeMap 保序),重叠判定 = parry `intersection_test` 精确相交,不加余量。
//!
//! 每个分片只需要和自己已装载区域相交的条目:先按条目 AABB(连杆位姿处
//! 形状的包围球/盒外扩)粗筛,再精确判定。

use std::collections::BTreeMap;

use rapier3d::parry::query::intersection_test;
use rapier3d::prelude::{Pose, SharedShape};
use types::glam::Vec3;
use types::{BlockPos, CHUNK_SIZE, EntityId, SHARD_CHUNKS, ShardPos};

/// 表里一条连杆:局部形状(共享,克隆廉价)+ 世界位姿 + 粗筛 AABB。
#[derive(Debug, Clone)]
pub struct AgentLinkEntry {
    /// 世界系位姿(上一拍末;在途主体 = 信封位姿)。
    pub pose: Pose,
    /// 连杆碰撞形状(与内核生成同源:agent.rs `shape_collider` 的形状部分)。
    pub shape: SharedShape,
    /// 形状局部包围球(粗筛用:`compute_local_aabb` 的球包络)。
    pub bound_radius: f32,
    /// 连杆中心(位姿平移;粗筛中心)。
    pub center: Vec3,
}

/// 一条主体:EntityId → 连杆条目(EntityId 升序由外层 BTreeMap 保证)。
#[derive(Debug, Clone, Default)]
pub struct AgentShapeTable {
    /// EntityId → 连杆列表(按 `BodyDef.links` 顺序;单连杆 = 1 条)。
    pub links: BTreeMap<EntityId, Vec<AgentLinkEntry>>,
}

/// 连杆形状组的包围球半径(粗筛用):单形状按解析值;多形状(compound)
/// 对每个子形状的包围球(中心 0,半径 r_i)取最大半径——子形状都以位姿
/// 原点为中心(compound 无子偏移,生成层保证),最大子半径就是整体包络。
pub fn link_bound_radius(shapes: &[types::ShapeDef]) -> f32 {
    shapes
        .iter()
        .map(|s| match s {
            types::ShapeDef::Box { half } => {
                (half[0] * half[0] + half[1] * half[1] + half[2] * half[2]).sqrt()
            }
            types::ShapeDef::Sphere { radius } => *radius,
            types::ShapeDef::CapsuleY {
                half_height,
                radius,
            } => half_height + radius,
            types::ShapeDef::ConvexHull { points } => points
                .iter()
                .map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt())
                .fold(0.5f32, f32::max),
        })
        .fold(0.5f32, f32::max)
}

/// 从形状定义构造局部形状与包围球半径(与 agent.rs 生成层同一映射;
/// 本模块不依赖 agent 私有函数,形状构造在这里复制一份最小实现)。
pub fn link_shape(shape: &types::ShapeDef) -> (SharedShape, f32) {
    match shape {
        types::ShapeDef::Box { half } => (
            SharedShape::cuboid(half[0], half[1], half[2]),
            (half[0] * half[0] + half[1] * half[1] + half[2] * half[2]).sqrt(),
        ),
        types::ShapeDef::Sphere { radius } => (SharedShape::ball(*radius), *radius),
        types::ShapeDef::CapsuleY {
            half_height,
            radius,
        } => (
            SharedShape::capsule_y(*half_height, *radius),
            half_height + radius,
        ),
        types::ShapeDef::ConvexHull { points } => {
            let pts: Vec<rapier3d::parry::math::Vector> =
                points.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
            let hull = SharedShape::convex_hull(&pts).expect("凸包顶点合法(校验保证非退化)");
            let aabb = hull.compute_local_aabb();
            let half = (aabb.maxs - aabb.mins) * 0.5;
            (
                hull,
                (half.x * half.x + half.y * half.y + half.z * half.z).sqrt(),
            )
        }
    }
}

impl AgentShapeTable {
    pub fn is_empty(&self) -> bool {
        self.links.is_empty()
    }

    /// 用信封里的根状态给一个主体建条目(迁移在途,REQ-012 A04:信封带
    /// 根位姿与身体哈希;单连杆 = 根即整体)。`shapes` = 注册表里该身体的
    /// 连杆形状定义(调用方从任一分片的注册表取——全分片同源)。
    pub fn insert_from_envelope(
        &mut self,
        id: EntityId,
        state: types::EntityState,
        body_links: &[&[types::ShapeDef]],
    ) {
        let pose = Pose {
            rotation: state.rotation,
            translation: state.translation,
            ..Pose::IDENTITY
        };
        let mut entries = Vec::with_capacity(body_links.len());
        for shapes in body_links {
            entries.push(AgentLinkEntry {
                pose,
                shape: crate::agent::agent_link_shared_shape_pub(shapes),
                bound_radius: link_bound_radius(shapes),
                center: state.translation,
            });
        }
        self.links.insert(id, entries);
    }

    /// 格立方体(格心 (x+0.5, y+0.5, z+0.5)、半边长 0.5)是否与表里任一
    /// 连杆**精确相交**(parry `intersection_test`,不加余量)。先 AABB
    /// 粗筛(包围球 vs 格盒),再精确判定。
    pub fn overlaps(&self, pos: BlockPos) -> bool {
        // 格盒中心与半边长。
        let cell_center = Vec3::new(pos.x as f32 + 0.5, pos.y as f32 + 0.5, pos.z as f32 + 0.5);
        let cell_half = Vec3::splat(0.5);
        let cell_shape = SharedShape::cuboid(0.5, 0.5, 0.5);
        let cell_pose = Pose {
            translation: cell_center,
            ..Pose::IDENTITY
        };
        for entry in self.links.values().flatten() {
            // 粗筛:连杆包围球 vs 格盒(AABB 最近点距离 > 半径 ⇒ 必不相交)。
            let d = (entry.center - cell_center).abs();
            let outside = d.x - cell_half.x;
            let oy = (entry.center.y - cell_center.y).abs() - cell_half.y;
            let oz = (entry.center.z - cell_center.z).abs() - cell_half.z;
            let gap_x = outside.max(0.0);
            let gap_y = oy.max(0.0);
            let gap_z = oz.max(0.0);
            if gap_x * gap_x + gap_y * gap_y + gap_z * gap_z
                > entry.bound_radius * entry.bound_radius
            {
                continue;
            }
            let hit = intersection_test(
                &entry.pose,
                entry.shape.as_ref(),
                &cell_pose,
                cell_shape.as_ref(),
            )
            .expect("shape/shape 相交查询恒支持(Unsupported 仅点/射线专属)");
            if hit.intersecting {
                return true;
            }
        }
        false
    }
}

/// 表对一格集合的批量判定(方便调用方一次查多格)。
impl AgentShapeTable {
    pub fn overlaps_any<'a>(&self, cells: impl IntoIterator<Item = &'a BlockPos>) -> bool {
        cells.into_iter().any(|&c| self.overlaps(c))
    }

    /// 暂缓队列长度瞬时值的观测别名(调度器汇总用)。
    pub fn len(&self) -> usize {
        self.links.len()
    }
}

/// 分片本体区的 block 范围(与 sim.rs 各判定同式)。
pub fn core_block_bounds(shard: ShardPos) -> (types::BlockPos, i32, i32) {
    let min = shard.min_chunk().block_min();
    let max_x = min.x + SHARD_CHUNKS * CHUNK_SIZE;
    let max_z = min.z + SHARD_CHUNKS * CHUNK_SIZE;
    (min, max_x, max_z)
}

/// 分片是否可能接触该格(本体区水平范围;精确判定由调用方做)。
pub fn in_core(shard: ShardPos, pos: BlockPos) -> bool {
    let (min, max_x, max_z) = core_block_bounds(shard);
    pos.x >= min.x && pos.x < max_x && pos.z >= min.z && pos.z < max_z
}
