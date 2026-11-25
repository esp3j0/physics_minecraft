//! entities —— Bevy 0.20.1 主体与动态刚体/脱落簇渲染层(REQ-013 C03-1 D3/D4/D5)。
//!
//! 职责:
//! - 纯网格构建器(`link_shape_mesh`/`solid_mesh`/`unknown_agent_placeholder_mesh`),无头可测(MS28);
//! - 每帧对账系统 [`sync_entities_system`]:消费 [`EntityMirrorRes`],维护主体三层实体树
//!   (root → link Transform → shape Mesh 子实体)与脱落簇/单格实体,支持附身青绿高亮与未知哈希占位;
//! - 生命周期与 `Assets<Mesh>` 回收:每个网格子实体/刚体实体与 `Assets<Mesh>` 句柄严格 1:1,
//!   移除时同步 `despawn` + `meshes.remove`(PM35 / MS29 靶点)。

use std::collections::BTreeMap;
use std::path::Path;

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::*;
use client_core::bodies::{BodyShapeTable, LinkShape};
use client_core::entities::{ENTITY_CUBE_SIZE, EntityMirror};
use types::{BodyHash, CompoundShape, EntityId};

use crate::{PossessRes, conv};

/// 普通主体色(品红;与 `viewer/src/render.rs:374` `AGENT_COLOR` 同源)。
pub(crate) const AGENT_COLOR: Color = Color::srgb(228.0 / 255.0, 92.0 / 255.0, 240.0 / 255.0);
/// 附身主体高亮色(青绿;与 `viewer/src/render.rs:377` `POSSESSED_AGENT_COLOR` 同源)。
pub(crate) const POSSESSED_AGENT_COLOR: Color =
    Color::srgb(92.0 / 255.0, 240.0 / 255.0, 178.0 / 255.0);
/// 动态刚体/脱落簇色(纯蓝;与 `viewer/src/render.rs:348` `BLUE` 同源)。
pub(crate) const SOLID_COLOR: Color = Color::srgb(0.0, 0.0, 1.0);

/// 客户端实体纯逻辑镜像资源(由 `drain_transport_system` 喂入 S2C 包)。
#[derive(Resource, Default)]
pub(crate) struct EntityMirrorRes(pub EntityMirror);

/// 单个主体的 Bevy 渲染树记录(root → link 实体列表 → shape mesh 子实体与 1:1 网格句柄)。
struct AgentRenderRecord {
    root: Entity,
    hash: BodyHash,
    links: Vec<Entity>,
    shape_entities: Vec<Entity>,
    mesh_handles: Vec<Handle<Mesh>>,
    possessed: bool,
}

/// 单个动态刚体/脱落簇的 Bevy 渲染记录(1:1 网格句柄 + 已建复合形快照)。
struct SolidRenderRecord {
    entity: Entity,
    mesh_handle: Handle<Mesh>,
    compound: Option<CompoundShape>,
}

/// 主体/脱落簇渲染状态资源(交互与脚本模式均插入)。
#[derive(Resource)]
pub(crate) struct EntityRenderRes {
    table: BodyShapeTable,
    /// 同 `(hash, link_idx, shape_idx)` 复用已构建 CPU 网格模板;实体生成时
    /// `meshes.add(template.clone())` 分配 1:1 独立句柄,保证 despawn `meshes.remove` 零串扰且
    /// `assets_live_end == entities_live_end + assets_baseline` 恒成立。
    mesh_templates: BTreeMap<(BodyHash, usize, usize), Mesh>,
    agent_material: Handle<StandardMaterial>,
    possessed_material: Handle<StandardMaterial>,
    solid_material: Handle<StandardMaterial>,
    agents: BTreeMap<EntityId, AgentRenderRecord>,
    solids: BTreeMap<EntityId, SolidRenderRecord>,
}

impl EntityRenderRes {
    pub(crate) fn new(bodies_dir: &Path, materials: &mut Assets<StandardMaterial>) -> Self {
        Self::from_table(BodyShapeTable::from_dir(bodies_dir), materials)
    }

    pub(crate) fn from_table(
        table: BodyShapeTable,
        materials: &mut Assets<StandardMaterial>,
    ) -> Self {
        let agent_material = materials.add(StandardMaterial {
            base_color: AGENT_COLOR,
            perceptual_roughness: 0.7,
            metallic: 0.0,
            ..default()
        });
        let possessed_material = materials.add(StandardMaterial {
            base_color: POSSESSED_AGENT_COLOR,
            perceptual_roughness: 0.6,
            metallic: 0.0,
            ..default()
        });
        let solid_material = materials.add(StandardMaterial {
            base_color: SOLID_COLOR,
            perceptual_roughness: 0.7,
            metallic: 0.0,
            ..default()
        });
        Self {
            table,
            mesh_templates: BTreeMap::new(),
            agent_material,
            possessed_material,
            solid_material,
            agents: BTreeMap::new(),
            solids: BTreeMap::new(),
        }
    }

    /// 身体形状表访问器(供主体拾取系统读取)。
    pub fn table(&self) -> &BodyShapeTable {
        &self.table
    }

    /// 客户端自有全部带 Mesh 的非地形实体数(主体连杆 shape 子实体 + solid 实体;D5 口径)。
    pub(crate) fn live_mesh_entities(&self) -> usize {
        let agent_meshes: usize = self.agents.values().map(|a| a.mesh_handles.len()).sum();
        agent_meshes + self.solids.len()
    }
}

/// 平移网格所有顶点坐标(用于将 `LinkShape` 的连杆局部 `offset` 烘入网格)。
fn translate_mesh(mesh: &mut Mesh, offset: Vec3) {
    if offset == Vec3::ZERO {
        return;
    }
    if let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    {
        for p in positions {
            p[0] += offset.x;
            p[1] += offset.y;
            p[2] += offset.z;
        }
    }
}

/// 构建单个连杆碰撞形状的 Bevy `Mesh`(纯函数,C03-1 D3 / MS28 t1 靶点)。
///
/// - `Box(half, offset)` → `Cuboid` 全边长 `2 * half` + 平移 `offset`;
/// - `Sphere(r, offset)` → `Sphere(r)` + 平移 `offset`;
/// - `CapsuleY { half_height, radius, offset }` → `Capsule3d::new(radius, 2*half_height)` + 平移 `offset`;
/// - `ConvexHull(points, offset)` → 局部包围盒 `Cuboid`(与 viewer 平价:`bevy_mesh 0.20.1`
///   无 convex hull builder,未来 glTF 单元替换)。
pub(crate) fn link_shape_mesh(shape: &LinkShape) -> Mesh {
    match shape {
        LinkShape::Box(half, offset) => {
            let mut mesh = Mesh::from(Cuboid::new(half[0] * 2.0, half[1] * 2.0, half[2] * 2.0));
            translate_mesh(&mut mesh, Vec3::from_array(*offset));
            mesh
        }
        LinkShape::Sphere(radius, offset) => {
            let mut mesh = Mesh::from(Sphere::new(*radius));
            translate_mesh(&mut mesh, Vec3::from_array(*offset));
            mesh
        }
        LinkShape::CapsuleY {
            half_height,
            radius,
            offset,
        } => {
            let mut mesh = Mesh::from(Capsule3d::new(*radius, 2.0 * *half_height));
            translate_mesh(&mut mesh, Vec3::from_array(*offset));
            mesh
        }
        LinkShape::ConvexHull(points, offset) => {
            // bevy_mesh 0.20.1 无 convex hull builder,画连杆局部包围盒(与 viewer 平价;未来 glTF 单元替换)。
            if points.is_empty() {
                let mut mesh = Mesh::from(Cuboid::new(0.2, 0.2, 0.2));
                translate_mesh(&mut mesh, Vec3::from_array(*offset));
                return mesh;
            }
            let mut min = [f32::MAX; 3];
            let mut max = [f32::MIN; 3];
            for p in points {
                for axis in 0..3 {
                    min[axis] = min[axis].min(p[axis]);
                    max[axis] = max[axis].max(p[axis]);
                }
            }
            let mid = Vec3::new(
                (min[0] + max[0]) * 0.5,
                (min[1] + max[1]) * 0.5,
                (min[2] + max[2]) * 0.5,
            );
            let sx = (max[0] - min[0]).max(0.01);
            let sy = (max[1] - min[1]).max(0.01);
            let sz = (max[2] - min[2]).max(0.01);
            let mut mesh = Mesh::from(Cuboid::new(sx, sy, sz));
            translate_mesh(&mut mesh, Vec3::from_array(*offset) + mid);
            mesh
        }
    }
}

/// 未知身体哈希占位盒(0.9 × 1.8 × 0.9,D3 口径,不 panic)。
pub(crate) fn unknown_agent_placeholder_mesh() -> Mesh {
    Mesh::from(Cuboid::new(0.9, 1.8, 0.9))
}

/// 构建动态刚体/脱落簇的 Bevy `Mesh`(纯函数,C03-1 D4 / MS28 t2 & PM34 靶点)。
///
/// - `None` → 单格 `0.9` 立方体(24 顶点 / 36 索引);
/// - `Some(shape)` → 逐 cell 一个 `0.9` 立方体按 `cell.offset` 平移合并进同一 `Mesh`
///   (不做 cell 间面剔除,顶点数 = `24 * cells.len()`,索引数 = `36 * cells.len()`)。
pub(crate) fn solid_mesh(compound: Option<&CompoundShape>) -> Mesh {
    let base = Mesh::from(Cuboid::from_length(ENTITY_CUBE_SIZE));
    let Some(shape) = compound else {
        return base;
    };

    let base_pos = match base.attribute(Mesh::ATTRIBUTE_POSITION) {
        Some(VertexAttributeValues::Float32x3(v)) => v.as_slice(),
        _ => unreachable!("Cuboid 必含 Float32x3 顶点位置"),
    };
    let base_norm = match base.attribute(Mesh::ATTRIBUTE_NORMAL) {
        Some(VertexAttributeValues::Float32x3(v)) => v.as_slice(),
        _ => unreachable!("Cuboid 必含 Float32x3 法线"),
    };
    let base_uv = match base.attribute(Mesh::ATTRIBUTE_UV_0) {
        Some(VertexAttributeValues::Float32x2(v)) => v.as_slice(),
        _ => unreachable!("Cuboid 必含 Float32x2 UV"),
    };
    let base_indices: Vec<u32> = match base.indices() {
        Some(Indices::U32(v)) => v.clone(),
        Some(Indices::U16(v)) => v.iter().map(|&i| u32::from(i)).collect(),
        None => unreachable!("Cuboid 必含索引缓冲"),
    };

    let cells = shape.cells();
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(base_pos.len() * cells.len());
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(base_norm.len() * cells.len());
    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(base_uv.len() * cells.len());
    let mut indices: Vec<u32> = Vec::with_capacity(base_indices.len() * cells.len());

    // PM34 靶点:必须遍历全部 cell 合并,不得只画首格。
    for cell in cells {
        let base_vertex = positions.len() as u32;
        let ox = cell.offset[0] as f32;
        let oy = cell.offset[1] as f32;
        let oz = cell.offset[2] as f32;
        for &[px, py, pz] in base_pos {
            positions.push([px + ox, py + oy, pz + oz]);
        }
        normals.extend_from_slice(base_norm);
        uvs.extend_from_slice(base_uv);
        for &idx in &base_indices {
            indices.push(base_vertex + idx);
        }
    }

    let mut merged = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    merged.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    merged.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    merged.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    merged.insert_indices(Indices::U32(indices));
    merged
}

/// 每帧对账系统(C03-1 D3/D4/D5;挂 `Update` 且在 `drain_transport_system` 之后,两模式均挂)。
///
/// 对比 [`EntityMirrorRes`] 与 [`EntityRenderRes`]:
/// - 移除或结构变更的实体:`despawn` + 逐句柄 `meshes.remove`(PM35 靶点);
/// - 新增实体:按层级 spawn 并分配 1:1 `Handle<Mesh>`;
/// - 既有实体:直接写入最新 `Transform`(零插值)并在附身切换时更新材质句柄。
#[allow(clippy::too_many_arguments)]
pub(crate) fn sync_entities_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    entity_mirror: Res<EntityMirrorRes>,
    mut entity_render: ResMut<EntityRenderRes>,
    possess_res: Option<Res<PossessRes>>,
    mut tf_q: Query<&mut Transform>,
    mut mat_q: Query<&mut MeshMaterial3d<StandardMaterial>>,
) {
    let possessed_id = possess_res.as_ref().and_then(|p| p.possessed);
    let mirror = &entity_mirror.0;

    // 1. 清理已消失或结构(hash / link 数)变化的主体。
    let stale_agents: Vec<EntityId> = entity_render
        .agents
        .iter()
        .filter_map(|(&id, rec)| match mirror.agents.get(&id) {
            None => Some(id),
            Some(view) if view.hash != rec.hash || view.poses.len() != rec.links.len() => Some(id),
            Some(_) => None,
        })
        .collect();
    for id in stale_agents {
        if let Some(rec) = entity_render.agents.remove(&id) {
            // PM35 靶点:despawn 时逐句柄回收 Assets<Mesh>。
            for handle in rec.mesh_handles {
                meshes.remove(&handle);
            }
            commands.entity(rec.root).despawn();
        }
    }

    // 2. 更新既有主体或 spawn 新主体。
    let possessed_mat = entity_render.possessed_material.clone();
    let agent_mat = entity_render.agent_material.clone();
    for (&id, view) in &mirror.agents {
        let is_possessed = possessed_id == Some(id);
        if let Some(rec) = entity_render.agents.get_mut(&id) {
            for (&link_entity, pose) in rec.links.iter().zip(&view.poses) {
                if let Ok(mut tf) = tf_q.get_mut(link_entity) {
                    tf.translation = conv::to_bevy_vec3(pose.translation);
                    tf.rotation = conv::to_bevy_quat(pose.rotation);
                }
            }
            if rec.possessed != is_possessed {
                rec.possessed = is_possessed;
                let mat = if is_possessed {
                    possessed_mat.clone()
                } else {
                    agent_mat.clone()
                };
                for &shape_entity in &rec.shape_entities {
                    if let Ok(mut mesh_mat) = mat_q.get_mut(shape_entity) {
                        mesh_mat.0 = mat.clone();
                    }
                }
            }
            continue;
        }

        let mat = if is_possessed {
            entity_render.possessed_material.clone()
        } else {
            entity_render.agent_material.clone()
        };
        let known_links: Option<Vec<Vec<LinkShape>>> =
            entity_render.table.get(&view.hash).map(|ls| ls.to_vec());
        if known_links.is_none() {
            entity_render.table.note_unknown();
        }

        let root = commands
            .spawn((Transform::IDENTITY, Visibility::Inherited))
            .id();
        let mut links = Vec::with_capacity(view.poses.len());
        let mut shape_entities = Vec::new();
        let mut mesh_handles = Vec::new();

        for (link_idx, pose) in view.poses.iter().enumerate() {
            let link_tf = Transform::from_translation(conv::to_bevy_vec3(pose.translation))
                .with_rotation(conv::to_bevy_quat(pose.rotation));
            let link_entity = commands
                .spawn((link_tf, Visibility::Inherited, ChildOf(root)))
                .id();
            links.push(link_entity);

            match &known_links {
                Some(def_links) => {
                    if let Some(shapes) = def_links.get(link_idx) {
                        for (shape_idx, shape) in shapes.iter().enumerate() {
                            let template = entity_render
                                .mesh_templates
                                .entry((view.hash, link_idx, shape_idx))
                                .or_insert_with(|| link_shape_mesh(shape))
                                .clone();
                            let handle = meshes.add(template);
                            let shape_entity = commands
                                .spawn((
                                    Mesh3d(handle.clone()),
                                    MeshMaterial3d(mat.clone()),
                                    Transform::IDENTITY,
                                    ChildOf(link_entity),
                                ))
                                .id();
                            shape_entities.push(shape_entity);
                            mesh_handles.push(handle);
                        }
                    }
                }
                None => {
                    // 未知哈希占位:在根连杆挂 0.9×1.8×0.9 占位盒,不 panic。
                    if link_idx == 0 {
                        let handle = meshes.add(unknown_agent_placeholder_mesh());
                        let shape_entity = commands
                            .spawn((
                                Mesh3d(handle.clone()),
                                MeshMaterial3d(mat.clone()),
                                Transform::IDENTITY,
                                ChildOf(link_entity),
                            ))
                            .id();
                        shape_entities.push(shape_entity);
                        mesh_handles.push(handle);
                    }
                }
            }
        }

        entity_render.agents.insert(
            id,
            AgentRenderRecord {
                root,
                hash: view.hash,
                links,
                shape_entities,
                mesh_handles,
                possessed: is_possessed,
            },
        );
    }

    // 3. 清理已消失或复合形发生变更的动态刚体/脱落簇。
    let stale_solids: Vec<EntityId> = entity_render
        .solids
        .iter()
        .filter_map(|(&id, rec)| match mirror.solids.get(&id) {
            None => Some(id),
            Some(view) if view.compound != rec.compound => Some(id),
            Some(_) => None,
        })
        .collect();
    for id in stale_solids {
        if let Some(rec) = entity_render.solids.remove(&id) {
            // PM35 靶点:despawn 时同步回收 Assets<Mesh>。
            meshes.remove(&rec.mesh_handle);
            commands.entity(rec.entity).despawn();
        }
    }

    // 4. 更新既有 solid 位姿或 spawn 新 solid。
    for (&id, view) in &mirror.solids {
        let tf_pos = conv::to_bevy_vec3(view.state.translation);
        let tf_rot = conv::to_bevy_quat(view.state.rotation);
        if let Some(rec) = entity_render.solids.get(&id) {
            if let Ok(mut tf) = tf_q.get_mut(rec.entity) {
                tf.translation = tf_pos;
                tf.rotation = tf_rot;
            }
            continue;
        }

        let handle = meshes.add(solid_mesh(view.compound.as_ref()));
        let entity = commands
            .spawn((
                Mesh3d(handle.clone()),
                MeshMaterial3d(entity_render.solid_material.clone()),
                Transform::from_translation(tf_pos).with_rotation(tf_rot),
            ))
            .id();
        entity_render.solids.insert(
            id,
            SolidRenderRecord {
                entity,
                mesh_handle: handle,
                compound: view.compound.clone(),
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_api::proto::S2C;
    use types::glam::{Quat as TQuat, Vec3 as TVec3};
    use types::{BlockId, BodyDef, EntityState, ShapeCell, Tick};

    fn eid(n: u32) -> EntityId {
        EntityId::from_parts(0, n)
    }

    fn pose_at(x: f32, y: f32, z: f32) -> EntityState {
        EntityState {
            translation: TVec3::new(x, y, z),
            rotation: TQuat::IDENTITY,
            linvel: TVec3::ZERO,
            angvel: TVec3::ZERO,
        }
    }

    fn index_len(mesh: &Mesh) -> usize {
        match mesh.indices() {
            Some(Indices::U32(v)) => v.len(),
            Some(Indices::U16(v)) => v.len(),
            None => 0,
        }
    }

    /// MS28 t1(REQ-013 C03-1 D4):`Box([0.5, 1.0, 1.5])` 网格顶点 24 / 索引 36(手算 6 面×4 顶 / 6×6 索引)。
    #[test]
    fn ms28_t1_box_mesh_vertices_and_indices() {
        let mesh = link_shape_mesh(&LinkShape::Box([0.5, 1.0, 1.5], [0.0, 0.0, 0.0]));
        assert_eq!(mesh.count_vertices(), 24, "Box 立方体应为 24 顶点");
        assert_eq!(index_len(&mesh), 36, "Box 立方体应为 36 索引");
    }

    /// MS28 t2(REQ-013 C03-1 D4 / PM34 靶点):三 cell 复合体合并网格顶点 72 / 索引 108(24×3 / 36×3,手算)。
    #[test]
    fn ms28_t2_compound_three_cell_mesh_vertices_and_indices() {
        let shape = CompoundShape::new(vec![
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
        .expect("三格复合形合法");
        let mesh = solid_mesh(Some(&shape));
        assert_eq!(mesh.count_vertices(), 72, "三格复合网格应为 72 顶点");
        assert_eq!(index_len(&mesh), 108, "三格复合网格应为 108 索引");
    }

    /// MS29(REQ-013 C03-1 D5 / PM32 & PM35 靶点):无头 App 调度 `sync_entities_system`,
    /// 验证 solid 与 agent 的 spawn → `meshes.len()` 增 1,removed → despawn + `meshes.remove`
    /// 活句柄归 0、`Assets<Mesh>.len()` 严格回到基线(chunk + baseline)。
    #[test]
    fn ms29_solid_and_agent_lifecycle_reclaims_mesh_handles() {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()));
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();

        let render_res = {
            let mut mats = app.world_mut().resource_mut::<Assets<StandardMaterial>>();
            EntityRenderRes::new(Path::new("no/such/bodies/dir"), &mut mats)
        };
        app.insert_resource(EntityMirrorRes::default())
            .insert_resource(render_res)
            .insert_resource(PossessRes::default())
            .add_systems(Update, sync_entities_system);

        // 基线(无地形段、无插件内置资产 → 0)。
        let baseline = app.world().resource::<Assets<Mesh>>().len();
        assert_eq!(baseline, 0);

        // 1. solid 路径:apply 造 solid → 同步 spawn → meshes.len 增 1。
        app.world_mut()
            .resource_mut::<EntityMirrorRes>()
            .0
            .apply(&S2C::EntityStates {
                tick: Tick(1),
                states: vec![(eid(10), pose_at(8.5, -58.5, 8.5))],
                removed: vec![],
                full: false,
            });
        app.update();
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), baseline + 1);
        assert_eq!(
            app.world()
                .resource::<EntityRenderRes>()
                .live_mesh_entities(),
            1
        );

        // apply removed → despawn + meshes.remove → 活句柄 0、assets 回到 baseline。
        app.world_mut()
            .resource_mut::<EntityMirrorRes>()
            .0
            .apply(&S2C::EntityStates {
                tick: Tick(2),
                states: vec![],
                removed: vec![eid(10)],
                full: false,
            });
        app.update();
        assert_eq!(
            app.world()
                .resource::<EntityRenderRes>()
                .live_mesh_entities(),
            0
        );
        assert!(app.world().resource::<EntityRenderRes>().solids.is_empty());
        assert_eq!(
            app.world().resource::<Assets<Mesh>>().len(),
            baseline,
            "solid removed 后必须回收 Mesh 句柄回到基线"
        );

        // 2. agent 路径(内置 capsule = 1 连杆 1 胶囊形状):spawn + removed 同路径回收。
        let capsule_toml = include_str!("../../../assets/bodies/capsule.body.toml");
        let capsule_hash = BodyDef::from_toml_str(capsule_toml)
            .expect("capsule 合法")
            .hash();
        app.world_mut()
            .resource_mut::<EntityMirrorRes>()
            .0
            .apply(&S2C::AgentStates {
                tick: Tick(3),
                states: vec![(eid(20), capsule_hash, vec![pose_at(4.0, -59.0, 8.0)])],
                removed: vec![],
                full: false,
            });
        app.update();
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), baseline + 1);
        assert_eq!(
            app.world()
                .resource::<EntityRenderRes>()
                .live_mesh_entities(),
            1
        );

        app.world_mut()
            .resource_mut::<EntityMirrorRes>()
            .0
            .apply(&S2C::AgentStates {
                tick: Tick(4),
                states: vec![],
                removed: vec![eid(20)],
                full: false,
            });
        app.update();
        assert_eq!(
            app.world()
                .resource::<EntityRenderRes>()
                .live_mesh_entities(),
            0
        );
        assert!(app.world().resource::<EntityRenderRes>().agents.is_empty());
        assert_eq!(
            app.world().resource::<Assets<Mesh>>().len(),
            baseline,
            "agent removed 后必须回收全部子 Mesh 句柄回到基线"
        );
    }
}
