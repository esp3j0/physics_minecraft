//! 探针:单 link + 单 Z 轴 revolute + 运行时 set_motor(AngZ),看关节动不动。
fn main() {
    use rapier3d::dynamics::{JointAxis, MotorModel, RevoluteJointBuilder, RigidBodyBuilder};
    use rapier3d::math::Vector;
    use rapier3d::pipeline::PhysicsWorld;

    let mut world = PhysicsWorld::new();
    let anchor = Vector::new(0.0, 1.0, 0.0);
    // 动态根(floating base)对照:根为 fixed 时电机不动,root dynamic 时呢?
    let dynamic_root = std::env::args().any(|a| a == "dyn");
    let body = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(anchor)
            .additional_mass(10.0),
    );
    let _ = dynamic_root;
    let link = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(anchor - Vector::new(0.0, 0.1, 0.0))
            .additional_mass(0.9),
    );
    let joint = RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0))
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::new(0.0, 0.1, 0.0))
        .build();
    world.insert_multibody_joint(body, link, joint);

    // 找到 link 对应的 (multibody, link_id)。
    let (mb_index, link_id) = {
        let lid = world
            .multibody_joints
            .rigid_body_link(link)
            .expect("link id");
        (lid.multibody, lid.id)
    };
    println!(
        "ndofs={}",
        world
            .multibody_joints
            .get_multibody(mb_index)
            .unwrap()
            .ndofs()
    );

    for tick in 0..120 {
        {
            let mb = world.multibody_joints.get_multibody_mut(mb_index).unwrap();
            let l = mb.link_mut(link_id).unwrap();
            l.joint
                .data
                .set_motor(JointAxis::AngX, 0.5, 0.0, 20.0, 0.5)
                .set_motor_model(JointAxis::AngX, MotorModel::ForceBased);
            l.joint.data.motors[3].max_force = 23.7;
            if tick == 0 {
                println!(
                    "before step: motor_axes={:b} locked={:b} num_vc={}",
                    l.joint.data.motor_axes.bits(),
                    l.joint.data.locked_axes.bits(),
                    l.joint.num_velocity_constraints()
                );
            }
        }
        world.step();
        if tick == 0 {
            let mb = world.multibody_joints.get_multibody(mb_index).unwrap();
            let l = mb.link(link_id).unwrap();
            println!(
                "after step:  motor_axes={:b} target_pos={} ndofs={}",
                l.joint.data.motor_axes.bits(),
                l.joint.data.motors[5].target_pos,
                mb.ndofs()
            );
        }
        if tick % 20 == 0 {
            let mb = world.multibody_joints.get_multibody(mb_index).unwrap();
            let l = mb.link(link_id).unwrap();
            let t = world.bodies.get(link).unwrap().translation();
            println!(
                "tick {tick}: coords[3..6]={:?} link_pos=({:.3},{:.3},{:.3})",
                &l.joint.coords()[3..6],
                t.x,
                t.y,
                t.z
            );
        }
    }
}
