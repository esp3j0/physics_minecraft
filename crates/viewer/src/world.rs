//! world —— viewer 的世界状态(REQ-008 单元 02):chunk 方块镜像、实体的服务器位置、
//! 服务器拍号、待取的 Spawned 回执。纯逻辑,零 macroquad;渲染层据此维护 Mesh 与实体插值,
//! 脚本引擎经 [`WorldQuery`] 读取。
//!
//! 语义(验收用例据此编写):
//! - `Welcome` / `Entities` 更新拍号;
//! - `Chunk` 整块替换,`ChunkGone` 移除;两者都把该 chunk 记为"脏"(渲染层据此重建或删除 Mesh);
//! - `Blocks` 只改已装载 chunk 的镜像并记脏;**未装载 chunk 的方块增量忽略**(随后到达的
//!   ChunkData 已含最新内容,不为它造一个只有空气的假 chunk);
//! - `Entities` 增量按 latest-wins 更新、`removed` 删除;`full` 时以 `states` 重建实体表;
//!   `live_entities` 返回服务器位置(不是渲染插值后的位置);
//! - `Shapes` 按 ID 记复合体形状(覆盖旧值;REQ-010 §10 修订 4);`removed`
//!   与全量包清掉过期形状;
//! - `Spawned` 回执按到达顺序累积,由 [`WorldState::take_spawned`] 取走;
//! - `Disconnected` 不改状态(由渲染层处理)。

use std::collections::BTreeMap;

use types::glam::Vec3;
use types::{BlockId, BlockPos, ChunkPos, CompoundShape, EntityId};

use crate::client::ViewEvent;

/// 复合实体各格中心(纯函数,REQ-010 §10 修订 4):`position + rotation × offset`;`shape` 为 `None` 时只有原点。
pub fn cube_centers(
    position: Vec3,
    rotation: types::glam::Quat,
    shape: Option<&CompoundShape>,
) -> Vec<Vec3> {
    match shape {
        None => vec![position],
        Some(shape) => shape
            .cells()
            .iter()
            .map(|cell| {
                let [x, y, z] = cell.offset;
                position + rotation * Vec3::new(x as f32, y as f32, z as f32)
            })
            .collect(),
    }
}
use crate::script::WorldQuery;

/// viewer 的世界状态。
#[derive(Debug, Default)]
pub struct WorldState {
    chunks: BTreeMap<ChunkPos, voxel::Chunk>,
    entities: BTreeMap<EntityId, Vec3>,
    /// 实体的服务器旋转(REQ-010 §10 修订 4 第 3 条):复合体展开各格要按
    /// 服务器旋转转偏移;随实体状态 latest-wins 记录。
    rotations: BTreeMap<EntityId, types::glam::Quat>,
    /// 复合体形状(REQ-010 §10 修订 4 第 3 条):`Shapes` 事件按 ID 记(覆盖
    /// 旧值);`removed` 与全量包清掉过期形状。
    shapes: BTreeMap<EntityId, CompoundShape>,
    tick: u64,
    /// 是否已从实体状态包得知真实拍号(REQ-008 修订 2,D4):`Welcome` 的拍号
    /// 是占位值 0,不算;第一个 `Entities` 包(含空的全量包)带来真实拍号,此后恒真。
    tick_known: bool,
    /// 自上次 `take_dirty_chunks` 以来变脏的 chunk(BTreeSet:升序、去重)。
    dirty: std::collections::BTreeSet<ChunkPos>,
    /// 自上次 `take_spawned` 以来收到的 Spawned 回执(到达顺序)。
    spawned: Vec<EntityId>,
}

impl WorldState {
    pub fn new() -> Self {
        Self::default()
    }

    /// chunk 局部坐标换算(负方向 floor;y 全高,局部 0 = WORLD_Y_MIN)。
    fn local(pos: BlockPos) -> (usize, usize, usize) {
        (
            pos.x.rem_euclid(types::CHUNK_SIZE) as usize,
            (pos.y - types::WORLD_Y_MIN) as usize,
            pos.z.rem_euclid(types::CHUNK_SIZE) as usize,
        )
    }

    /// 应用一个事件(语义见模块文档)。
    pub fn apply(&mut self, event: ViewEvent) {
        match event {
            ViewEvent::Welcome { tick, .. } => self.tick = tick.0,
            ViewEvent::Chunk { pos, chunk } => {
                self.chunks.insert(pos, chunk);
                self.dirty.insert(pos);
            }
            ViewEvent::ChunkGone { pos } => {
                self.chunks.remove(&pos);
                self.dirty.insert(pos);
            }
            ViewEvent::Entities {
                tick,
                states,
                removed,
                full,
            } => {
                self.tick = tick.0;
                // 第一个实体状态包(含空的全量心跳)即带来真实拍号(修订 2,D4);
                // 一旦已知即保持(断连不改)。
                self.tick_known = true;
                if full {
                    // 全量对账:以 states 为准重建实体表(协议 full 同语义);
                    // 形状同样按 states 对账(不在 states 里的形状清掉——World
                    // 队列丢最旧时形状包可能丢,全量重发自愈,§10 修订 4)。
                    self.entities.clear();
                    self.shapes
                        .retain(|id, _| states.iter().any(|(present, _)| present == id));
                }
                for (id, state) in states {
                    self.entities.insert(id, state.translation);
                    self.rotations.insert(id, state.rotation);
                }
                for id in removed {
                    self.entities.remove(&id);
                    self.rotations.remove(&id);
                    // removed 删掉实体时一并删掉形状(ID 复用时不再按旧形状画)。
                    self.shapes.remove(&id);
                }
            }
            ViewEvent::Blocks { updates } => {
                // 只改已装载 chunk 的镜像;未装载忽略(不造假 chunk)。
                for (pos, block) in updates {
                    let cpos = pos.chunk_of();
                    let Some(chunk) = self.chunks.get_mut(&cpos) else {
                        continue;
                    };
                    let (lx, ly, lz) = Self::local(pos);
                    chunk.set(lx, ly, lz, block);
                    self.dirty.insert(cpos);
                }
            }
            ViewEvent::Spawned { entity } => self.spawned.push(entity),
            // 形状按 ID 记(覆盖旧值;§10 修订 4 第 3 条)。
            ViewEvent::Shapes { shapes } => {
                for (id, shape) in shapes {
                    self.shapes.insert(id, shape);
                }
            }
            ViewEvent::Disconnected { .. } => {}
        }
    }

    /// 实体各格的中心(服务器位姿,不是渲染插值;REQ-010 §10 修订 4):原点 +
    /// 服务器旋转 × 偏移,形状未知按单格(只有原点);实体不存在为 `None`。
    pub fn entity_cube_centers(&self, id: EntityId) -> Option<Vec<Vec3>> {
        let position = *self.entities.get(&id)?;
        let shape = self.shapes.get(&id);
        Some(cube_centers(
            position,
            self.rotations.get(&id).copied().unwrap_or_default(),
            shape,
        ))
    }

    /// 实体的复合体形状(渲染层按插值位姿逐格画用);单格实体为 `None`。
    pub fn entity_shape(&self, id: EntityId) -> Option<&CompoundShape> {
        self.shapes.get(&id)
    }

    /// 实体的服务器旋转(渲染层旋转插值用);未知 = 恒等。
    pub fn entity_rotation(&self, id: EntityId) -> types::glam::Quat {
        self.rotations.get(&id).copied().unwrap_or_default()
    }

    /// 最新服务器拍号。
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// 是否已从实体状态包得知真实拍号(REQ-008 修订 2,D4)。`Welcome` 的拍号是占位值,
    /// 不算;收到第一个 `Entities` 事件后恒为 true。脚本模式据此决定何时开始推进引擎。
    pub fn tick_known(&self) -> bool {
        self.tick_known
    }

    /// 某个已装载 chunk 的方块镜像。
    pub fn chunk(&self, pos: ChunkPos) -> Option<&voxel::Chunk> {
        self.chunks.get(&pos)
    }

    /// 取走自上次调用以来变"脏"的 chunk(升序、去重)。
    pub fn take_dirty_chunks(&mut self) -> Vec<ChunkPos> {
        let taken = std::mem::take(&mut self.dirty);
        taken.into_iter().collect()
    }

    /// 取走自上次调用以来收到的 Spawned 回执(按到达顺序)。
    pub fn take_spawned(&mut self) -> Vec<EntityId> {
        std::mem::take(&mut self.spawned)
    }
}

impl WorldQuery for WorldState {
    fn live_entities(&self) -> Vec<(EntityId, Vec3)> {
        self.entities.iter().map(|(id, p)| (*id, *p)).collect()
    }

    fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
        let (lx, ly, lz) = Self::local(pos);
        if ly >= voxel::CHUNK_Y {
            return None;
        }
        self.chunks
            .get(&pos.chunk_of())
            .map(|chunk| chunk.get(lx, ly, lz))
    }

    fn chunks_loaded(&self) -> usize {
        self.chunks.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use messaging::InputSource;
    use types::{EntityState, ShapeCell, Tick};

    fn block_at_pos(w: &WorldState, pos: BlockPos) -> Option<BlockId> {
        w.block_at(pos)
    }

    /// 未装载 chunk 的增量被忽略,不虚增 chunks_loaded、不造假可查格(锚定模块
    /// 文档"未装载忽略";若实现造出假 chunk,`chunks_loaded` 与 `block_at` 断言红)。
    #[test]
    fn world_blocks_update_ignores_unloaded_chunks() {
        let mut w = WorldState::new();
        assert_eq!(w.chunks_loaded(), 0);
        assert_eq!(block_at_pos(&w, BlockPos::new(0, -61, 0)), None);
        w.apply(ViewEvent::Blocks {
            updates: vec![(BlockPos::new(0, -61, 0), BlockId(1))],
        });
        assert_eq!(w.chunks_loaded(), 0, "未装载 chunk 的增量不造假 chunk");
        assert_eq!(block_at_pos(&w, BlockPos::new(0, -61, 0)), None);
    }

    /// 负坐标换算:世界 (−5, −61, 40) → chunk (−1, 2) 局部 (11, 3, 8)(rem_euclid
    /// 语义;若实现用向零截断,读 −6 列时错位,本测试红)。
    #[test]
    fn world_block_at_maps_negative_coords() {
        let mut w = WorldState::new();
        let west = ChunkPos::new(-1, 2);
        let mut chunk = voxel::Chunk::air();
        chunk.set(11, 3, 8, BlockId(2));
        w.apply(ViewEvent::Chunk { pos: west, chunk });
        assert_eq!(w.chunks_loaded(), 1);
        assert_eq!(
            block_at_pos(&w, BlockPos::new(-5, -61, 40)),
            Some(BlockId(2))
        );
        assert_eq!(
            block_at_pos(&w, BlockPos::new(-6, -61, 40)),
            Some(BlockId::AIR)
        );
        // 超出世界 y 高度:不可查(防 y 越界 panic / 误读)。
        assert_eq!(block_at_pos(&w, BlockPos::new(-5, 400, 40)), None);
    }

    /// 脏集合:Chunk / ChunkGone / Blocks 各自记脏;升序去重;取走即清。
    #[test]
    fn world_dirty_chunks_are_ordered_deduped_and_cleared() {
        let mut w = WorldState::new();
        let a = ChunkPos::new(1, 0);
        let b = ChunkPos::new(-2, 3);
        w.apply(ViewEvent::Chunk {
            pos: a,
            chunk: voxel::Chunk::air(),
        });
        w.apply(ViewEvent::Chunk {
            pos: b,
            chunk: voxel::Chunk::air(),
        });
        w.apply(ViewEvent::Blocks {
            updates: vec![(BlockPos::new(20, -61, 0), BlockId(1))], // chunk a 内
        });
        w.apply(ViewEvent::ChunkGone { pos: a });
        assert_eq!(w.take_dirty_chunks(), vec![b, a], "升序、去重(a 两脏合一)");
        assert!(w.take_dirty_chunks().is_empty(), "取走即清");
    }

    /// 实体表与拍号:增量 latest-wins、removed 删除、full 重建;Welcome/Entities
    /// 更新拍号;Spawned 回执按到达顺序取走;Disconnected 不改任何状态。
    #[test]
    fn world_entities_ticks_and_receipts_follow_events() {
        let mut w = WorldState::new();
        w.apply(ViewEvent::Welcome {
            player_id: InputSource(9),
            tick: Tick(3),
        });
        assert_eq!(w.tick(), 3);
        let p1 = Vec3::new(1.0, 2.0, 3.0);
        let p2 = Vec3::new(4.0, 5.0, 6.0);
        w.apply(ViewEvent::Entities {
            tick: Tick(8),
            states: vec![(
                EntityId::from_parts(0, 1),
                EntityState {
                    translation: p1,
                    ..EntityState::default()
                },
            )],
            removed: vec![],
            full: false,
        });
        w.apply(ViewEvent::Entities {
            tick: Tick(12),
            states: vec![(
                EntityId::from_parts(0, 1),
                EntityState {
                    translation: p2,
                    ..EntityState::default()
                },
            )],
            removed: vec![],
            full: false,
        });
        assert_eq!(w.tick(), 12, "Entities 包推进拍号");
        assert_eq!(
            w.live_entities(),
            vec![(EntityId::from_parts(0, 1), p2)],
            "latest-wins"
        );

        w.apply(ViewEvent::Entities {
            tick: Tick(20),
            states: vec![(
                EntityId::from_parts(0, 7),
                EntityState {
                    translation: p1,
                    ..EntityState::default()
                },
            )],
            removed: vec![EntityId::from_parts(0, 1)],
            full: false,
        });
        assert_eq!(
            w.live_entities(),
            vec![(EntityId::from_parts(0, 7), p1)],
            "removed 删除"
        );

        w.apply(ViewEvent::Entities {
            tick: Tick(30),
            states: vec![],
            removed: vec![],
            full: true,
        });
        assert!(w.live_entities().is_empty(), "full 清空实体表(全量对账)");
        assert_eq!(w.tick(), 30);

        w.apply(ViewEvent::Spawned {
            entity: EntityId::from_parts(0, 2),
        });
        w.apply(ViewEvent::Spawned {
            entity: EntityId::from_parts(0, 1),
        });
        assert_eq!(
            w.take_spawned(),
            vec![EntityId::from_parts(0, 2), EntityId::from_parts(0, 1)],
            "回执按到达顺序"
        );
        assert!(w.take_spawned().is_empty(), "取走即清");

        let entities_before = w.live_entities();
        let tick_before = w.tick();
        w.apply(ViewEvent::Disconnected {
            reason: "bye".to_owned(),
        });
        assert_eq!(w.tick(), tick_before, "断连不改拍号");
        assert_eq!(w.live_entities(), entities_before, "断连不改实体表");
    }

    /// 拍号已知标志(REQ-008 修订 2,D4;锚定模块文档):初始未知;`Welcome`
    /// 的拍号是占位值 0,不算;第一个实体状态包(含**空的全量包**)为真,
    /// 此后(含断连)恒真。
    ///
    /// 会变红的实现:把 Welcome 当作拍号已知(引擎从占位拍号 0 起算,等待类
    /// 步骤在首个真实拍号到达时立即误判超时);空的全量包不算数(空场景拍号
    /// 永远未知,wait_ticks 卡死)。
    #[test]
    fn tick_known_only_after_first_entity_states() {
        let mut w = WorldState::new();
        assert!(!w.tick_known(), "初始未知");
        w.apply(ViewEvent::Welcome {
            player_id: InputSource(1),
            tick: Tick(0),
        });
        assert!(!w.tick_known(), "Welcome 的拍号是占位值,不算");
        // 连非空增量也不算"已经知道真实拍号"?算——任何实体状态包都行;
        // 这里先验证空全量包(空场景唯一能收到的形态)。
        w.apply(ViewEvent::Entities {
            tick: Tick(20_617),
            states: vec![],
            removed: vec![],
            full: true,
        });
        assert!(w.tick_known(), "空的全量包同样带来真实拍号");
        assert_eq!(w.tick(), 20_617);
        w.apply(ViewEvent::Disconnected {
            reason: "bye".to_owned(),
        });
        assert!(w.tick_known(), "一旦已知即保持");
    }

    /// 2 格梁(测试形状;BlockId 任意)。
    fn beam2() -> CompoundShape {
        CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [1, 0, 0],
                block: BlockId(1),
            },
        ])
        .expect("2 格梁")
    }

    fn entity_state(p: Vec3, rot: types::glam::Quat) -> EntityState {
        EntityState {
            translation: p,
            rotation: rot,
            ..EntityState::default()
        }
    }

    /// `Shapes` 事件按 ID 记形状且**覆盖旧值**(§10 修订 4 第 3 条);单格实体
    /// 从不进形状表(发过形状的实体,状态照常更新)。
    ///
    /// 会变红的实现:同 ID 二次 Shapes 被丢弃(按形状内容去重等自作主张)。
    #[test]
    fn world_shapes_overwrite_by_id() {
        let id = EntityId::from_parts(0, 9);
        let pa = Vec3::new(1.0, 2.0, 3.0);
        let pb = Vec3::new(4.0, 5.0, 6.0);
        let mut w = WorldState::new();
        w.apply(ViewEvent::Shapes {
            shapes: vec![(id, beam2())],
        });
        w.apply(ViewEvent::Entities {
            tick: Tick(1),
            states: vec![(id, entity_state(pa, types::glam::Quat::IDENTITY))],
            removed: vec![],
            full: false,
        });
        let mut want: Vec<Vec3> = (0..=1)
            .map(|x| pa + Vec3::new(x as f32, 0.0, 0.0))
            .collect();
        assert_eq!(
            w.entity_cube_centers(id),
            Some(want.clone()),
            "首份形状生效"
        );

        // 覆盖:同一 ID 换成不同形状(此处用 3 格)后再查,应按新形状展开。
        let beam3 = CompoundShape::new(vec![
            ShapeCell {
                offset: [0, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [1, 0, 0],
                block: BlockId(1),
            },
            ShapeCell {
                offset: [2, 0, 0],
                block: BlockId(1),
            },
        ])
        .expect("3 格梁");
        w.apply(ViewEvent::Shapes {
            shapes: vec![(id, beam3)],
        });
        want.push(pa + Vec3::new(2.0, 0.0, 0.0));
        assert_eq!(
            w.entity_cube_centers(id),
            Some(want),
            "同 ID 的 Shapes 覆盖旧形状"
        );

        // 状态更新不丢形状(位置变了仍按形状展开)。
        w.apply(ViewEvent::Entities {
            tick: Tick(2),
            states: vec![(id, entity_state(pb, types::glam::Quat::IDENTITY))],
            removed: vec![],
            full: false,
        });
        assert_eq!(
            w.entity_cube_centers(id),
            Some(vec![
                pb,
                pb + Vec3::new(1.0, 0.0, 0.0),
                pb + Vec3::new(2.0, 0.0, 0.0)
            ]),
            "状态更新保留形状"
        );
    }

    /// `removed` 删掉实体时一并删掉形状;全量包删掉不在 `states` 里的形状
    /// (§10 修订 4 第 3 条;ID 复用/过期形状不再画出来)。旋转随实体状态
    /// 记录:`entity_rotation` 返回服务器旋转,未收到状态 = 恒等。
    ///
    /// 会变红的实现:形状永不清理;全量对账漏形状;旋转没记。
    #[test]
    fn world_shapes_removed_and_full_reconcile() {
        let (a, b) = (EntityId::from_parts(0, 1), EntityId::from_parts(0, 2));
        let rot = types::glam::Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        let pa = Vec3::new(0.5, 1.5, 2.5);
        let mut w = WorldState::new();
        assert_eq!(
            w.entity_rotation(a),
            types::glam::Quat::IDENTITY,
            "未收到状态 = 恒等旋转"
        );
        w.apply(ViewEvent::Shapes {
            shapes: vec![(a, beam2()), (b, beam2())],
        });
        w.apply(ViewEvent::Entities {
            tick: Tick(1),
            states: vec![(a, entity_state(pa, rot))],
            removed: vec![],
            full: false,
        });
        assert_eq!(w.entity_rotation(a), rot, "服务器旋转随状态记录");

        // 全量包只含 a → b 的形状被清掉(全量对账)。
        w.apply(ViewEvent::Entities {
            tick: Tick(2),
            states: vec![(a, entity_state(pa, rot))],
            removed: vec![],
            full: true,
        });
        assert!(
            w.entity_shape(b).is_none(),
            "全量包清掉不在 states 里的形状"
        );
        assert!(w.entity_shape(a).is_some(), "states 里的形状保留");

        // removed a → 形状同删;同 ID 再出现(无新形状)按单格 + 恒等旋转。
        w.apply(ViewEvent::Entities {
            tick: Tick(3),
            states: vec![],
            removed: vec![a],
            full: false,
        });
        assert_eq!(w.entity_cube_centers(a), None, "实体删除");
        w.apply(ViewEvent::Entities {
            tick: Tick(4),
            states: vec![(a, entity_state(pa, types::glam::Quat::IDENTITY))],
            removed: vec![],
            full: false,
        });
        assert_eq!(
            w.entity_cube_centers(a),
            Some(vec![pa]),
            "removed 已清掉 a 的形状:按单格"
        );
        assert_eq!(w.entity_rotation(a), types::glam::Quat::IDENTITY);
    }

    /// `cube_centers` 旋转展开与单格退化(纯函数;渲染与 `entity_cube_centers`
    /// 共用):偏移随旋转转动(绕 y 90°:+x → −z);`None` 只有原点。
    ///
    /// 会变红的实现:偏移没乘旋转;用了 `inverse()`;单格也展开。
    #[test]
    fn world_cube_centers_rotates_offsets() {
        let pos = Vec3::new(10.5, 3.5, -2.5);
        let rot = types::glam::Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        let got = cube_centers(pos, rot, Some(&beam2()));
        let want = vec![pos, pos + Vec3::new(0.0, 0.0, -1.0)];
        assert!(
            got.len() == want.len()
                && got
                    .iter()
                    .zip(&want)
                    .all(|(g, w)| (*g - *w).length() < 1e-5),
            "绕 y 转 90°:+x 偏移转到 −z,实得 {got:?}"
        );
        assert_eq!(cube_centers(pos, rot, None), vec![pos], "单格只有原点");
    }
}
