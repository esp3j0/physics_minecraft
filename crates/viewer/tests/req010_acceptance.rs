//! REQ-010 viewer 验收用例(单元 03b;总架构师编写,实施方只许删 `#[ignore]`)。
//!
//! 规格:REQ-010 §10 修订 4——`S2C::EntityShapes` → `ViewEvent::Shapes`;`WorldState` 记形状、`removed` 与全量包
//! 清掉过期形状;各格中心 = 原点 + 旋转 × 偏移,形状未知按单格。

use sim_api::proto::S2C;
use types::glam::{Quat, Vec3};
use types::{BlockId, CompoundShape, EntityId, EntityState, ShapeCell, Tick};
use viewer::client::{ViewEvent, s2c_to_event};
use viewer::world::{WorldState, cube_centers};

fn beam() -> CompoundShape {
    CompoundShape::new(
        (-2..=3)
            .map(|x| ShapeCell {
                offset: [x, 0, 0],
                block: BlockId(1),
            })
            .collect(),
    )
    .expect("6 格梁")
}

fn state(translation: Vec3, rotation: Quat) -> EntityState {
    EntityState {
        translation,
        rotation,
        ..EntityState::default()
    }
}

fn entities(states: Vec<(EntityId, EntityState)>, removed: Vec<EntityId>, full: bool) -> ViewEvent {
    ViewEvent::Entities {
        tick: Tick(5),
        states,
        removed,
        full,
    }
}

fn near(a: &[Vec3], b: &[Vec3]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (*x - *y).length() < 1e-5)
}

/// AC-C14:`S2C::EntityShapes` 映射成 `ViewEvent::Shapes`,内容原样。
///
/// 会变红的实现:形状包被吞掉(viewer 永远不知道形状,复合体画成一个方块)。
#[test]
#[ignore = "REQ-010 单元 03b 待实施方解除"]
fn acc_c14_shapes_event_mapping() {
    let id = EntityId::from_parts(0, 7);
    let shapes = vec![(id, beam())];
    match s2c_to_event(S2C::EntityShapes {
        shapes: shapes.clone(),
    }) {
        Some(ViewEvent::Shapes { shapes: got }) => assert_eq!(got, shapes),
        other => panic!("期望 Shapes 事件,实得 {other:?}"),
    }
}

/// AC-C14:各格中心 = 原点 + 旋转 × 偏移;没有形状时只有原点。
///
/// 会变红的实现:偏移没随旋转转动;用了 `rotation.inverse()`;单格也按形状展开。
#[test]
#[ignore = "REQ-010 单元 03b 待实施方解除"]
fn acc_c14_cube_centers_rotate_offsets() {
    let pos = Vec3::new(10.5, 3.5, -2.5);
    let rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    let shape = beam();
    let want: Vec<Vec3> = (-2..=3)
        .map(|x| pos + Vec3::new(0.0, 0.0, -(x as f32)))
        .collect();
    assert!(
        near(&cube_centers(pos, rot, Some(&shape)), &want),
        "绕 y 转 90°:+x 偏移转到 −z"
    );
    assert!(near(&cube_centers(pos, rot, None), &[pos]), "单格只有原点");
}

/// AC-C14:`WorldState` 记形状——先收形状、再收状态,实体按服务器旋转展开成 6 格;`removed` 删掉实体时一并删掉形状
/// (同 ID 再出现且没有新形状时按单格);全量包删掉不在 `states` 里的形状;没有形状的实体按单格。
///
/// 会变红的实现:形状永不清理(复用 ID 时画错形状);全量包没清过期形状;状态里的旋转没记下来。
#[test]
#[ignore = "REQ-010 单元 03b 待实施方解除"]
fn acc_c14_world_state_tracks_shapes() {
    let (a, b, c) = (
        EntityId::from_parts(0, 1),
        EntityId::from_parts(0, 2),
        EntityId::from_parts(0, 3),
    );
    let rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    let pa = Vec3::new(1.5, 2.5, 3.5);
    let pc = Vec3::new(-4.5, 2.5, 0.5);
    let mut w = WorldState::new();
    w.apply(ViewEvent::Shapes {
        shapes: vec![(a, beam()), (b, beam())],
    });
    w.apply(entities(
        vec![(a, state(pa, rot)), (c, state(pc, Quat::IDENTITY))],
        vec![],
        true,
    ));
    // 独立写出期望(不借 `cube_centers`):绕 y 转 90°,+x 偏移转到 −z。
    let want_a: Vec<Vec3> = (-2..=3)
        .map(|x| pa + Vec3::new(0.0, 0.0, -(x as f32)))
        .collect();
    assert!(
        near(&w.entity_cube_centers(a).expect("a 在场"), &want_a),
        "a 按形状与旋转展开"
    );
    assert_eq!(w.entity_cube_centers(c), Some(vec![pc]), "c 没有形状:单格");
    assert_eq!(w.entity_cube_centers(b), None, "b 不在场");

    // 全量包不含 b → b 的形状被清掉:之后 b 以增量出现时按单格。
    let pb = Vec3::new(7.5, 2.5, 7.5);
    w.apply(entities(
        vec![(b, state(pb, Quat::IDENTITY))],
        vec![],
        false,
    ));
    assert_eq!(
        w.entity_cube_centers(b),
        Some(vec![pb]),
        "全量包已清掉 b 的过期形状"
    );

    // removed 删掉 a 与它的形状;同 ID 再出现(无新形状)→ 单格。
    w.apply(entities(vec![], vec![a], false));
    assert_eq!(w.entity_cube_centers(a), None);
    w.apply(entities(vec![(a, state(pa, rot))], vec![], false));
    assert_eq!(
        w.entity_cube_centers(a),
        Some(vec![pa]),
        "removed 已清掉 a 的形状"
    );
}
