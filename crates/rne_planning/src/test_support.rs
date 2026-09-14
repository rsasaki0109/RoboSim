//! Shared deterministic test fixtures for `rne_planning` unit tests.

use bevy_ecs::prelude::World;
use rne_ecs::{spawn_named, Entity};
use rne_math::{Quat, Vec3};
use rne_robot::{IkOptions, Joint, JointKind, JointLimits, Link, Robot};
use rne_world::Transform3;

/// A planar 2R arm: base -> link1 -> ee (revolute) -> tool (fixed), with each
/// link 1 m long. The returned entity is the tool frame, whose position depends
/// on both joint values. Joint limits are `[-pi, pi]` with a 1 rad/s velocity
/// limit.
pub(crate) fn arm_world() -> (World, Entity, Entity) {
    let mut world = World::new();
    let robot = spawn_named(&mut world, "arm");
    let base = spawn_named(&mut world, "base");
    let link1 = spawn_named(&mut world, "link1");
    let ee = spawn_named(&mut world, "ee");
    let tool = spawn_named(&mut world, "tool");

    for (link, name, local) in [
        (base, "base", Vec3::ZERO),
        (link1, "link1", Vec3::ZERO),
        (ee, "ee", Vec3::new(1.0, 0.0, 0.0)),
        (tool, "tool", Vec3::new(1.0, 0.0, 0.0)),
    ] {
        world.entity_mut(link).insert((
            Link {
                robot,
                name: name.to_string(),
            },
            Transform3::from_translation_rotation(local, Quat::IDENTITY),
            rne_physics::Collider::sphere(0.05),
        ));
    }
    world.entity_mut(robot).insert(Robot {
        robot_id: Default::default(),
        model_name: "arm".into(),
        base_link: base,
    });

    let limits = JointLimits {
        lower: -std::f64::consts::PI,
        upper: std::f64::consts::PI,
        max_velocity: 1.0,
        ..JointLimits::default()
    };
    for (parent, child, kind, name) in [
        (base, link1, JointKind::Revolute, "joint1"),
        (link1, ee, JointKind::Revolute, "joint2"),
        (ee, tool, JointKind::Fixed, "tool_joint"),
    ] {
        let joint = spawn_named(&mut world, name);
        world.entity_mut(joint).insert(Joint {
            robot,
            parent_link: parent,
            child_link: child,
            kind,
            limits,
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        });
    }

    (world, robot, tool)
}

/// Position-only inverse kinematics options.
pub(crate) fn position_only_ik() -> IkOptions {
    IkOptions {
        solve_orientation: false,
        ..IkOptions::default()
    }
}
