//! world —— viewer 的世界状态(REQ-008 单元 02):chunk 方块镜像、实体的服务器位置、
//! 服务器拍号、待取的 Spawned 回执。纯逻辑,零 macroquad;渲染层据此维护 Mesh 与实体插值,
//! 脚本引擎经 [`WorldQuery`] 读取。
//!
//! **总架构师接口桩(REQ-008 单元 02)**:公开类型与签名固定,函数体由实施方实现。
//!
//! 语义(验收用例据此编写):
//! - `Welcome` / `Entities` 更新拍号;
//! - `Chunk` 整块替换,`ChunkGone` 移除;两者都把该 chunk 记为"脏"(渲染层据此重建或删除 Mesh);
//! - `Blocks` 只改已装载 chunk 的镜像并记脏;**未装载 chunk 的方块增量忽略**(随后到达的
//!   ChunkData 已含最新内容,不为它造一个只有空气的假 chunk);
//! - `Entities` 增量按 latest-wins 更新、`removed` 删除;`full` 时以 `states` 重建实体表;
//!   `live_entities` 返回服务器位置(不是渲染插值后的位置);
//! - `Spawned` 回执按到达顺序累积,由 [`WorldState::take_spawned`] 取走;
//! - `Disconnected` 不改状态(由渲染层处理)。

use std::collections::BTreeMap;

use types::glam::Vec3;
use types::{BlockId, BlockPos, ChunkPos, EntityId};

use crate::client::ViewEvent;
use crate::script::WorldQuery;

/// viewer 的世界状态。
#[derive(Debug, Default)]
pub struct WorldState {
    chunks: BTreeMap<ChunkPos, voxel::Chunk>,
    entities: BTreeMap<EntityId, Vec3>,
    tick: u64,
}

impl WorldState {
    pub fn new() -> Self {
        Self::default()
    }

    /// 应用一个事件(语义见模块文档)。
    pub fn apply(&mut self, event: ViewEvent) {
        let _ = (event, &self.chunks, &self.entities);
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }

    /// 最新服务器拍号。
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// 某个已装载 chunk 的方块镜像。
    pub fn chunk(&self, pos: ChunkPos) -> Option<&voxel::Chunk> {
        let _ = pos;
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }

    /// 取走自上次调用以来变"脏"的 chunk(升序、去重)。
    pub fn take_dirty_chunks(&mut self) -> Vec<ChunkPos> {
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }

    /// 取走自上次调用以来收到的 Spawned 回执(按到达顺序)。
    pub fn take_spawned(&mut self) -> Vec<EntityId> {
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }
}

impl WorldQuery for WorldState {
    fn live_entities(&self) -> Vec<(EntityId, Vec3)> {
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }

    fn block_at(&self, pos: BlockPos) -> Option<BlockId> {
        let _ = pos;
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }

    fn chunks_loaded(&self) -> usize {
        unimplemented!("REQ-008 单元 02:由实施方实现")
    }
}
