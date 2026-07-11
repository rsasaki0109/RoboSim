//! URDF joint wiring for Rapier impulse articulations.

use crate::parse::rpy_to_quat;
use crate::schema::{UrdfJointType, UrdfRobot};
use crate::spawn::{SpawnedUrdfRobot, UrdfSpawnError};
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::{
    FixedJointDesc, JointMotor, MultibodyLink, PrismaticJointDesc, RevoluteJointDesc, RigidBody,
    RigidBodyType,
};
use std::collections::HashSet;

/// Configuration for [`attach_urdf_articulation`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UrdfArticulationConfig {
    /// Rigid-body type applied to the robot base link.
    pub base_body_type: RigidBodyType,
    /// Maximum motor force applied to each created revolute joint.
    pub motor_max_force: f32,
    /// When true, links are wired as one reduced-coordinate multibody.
    pub multibody: bool,
}

impl Default for UrdfArticulationConfig {
    fn default() -> Self {
        Self {
            base_body_type: RigidBodyType::Fixed,
            motor_max_force: 50.0,
            multibody: false,
        }
    }
}

/// Summary of joints wired into the physics backend.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UrdfArticulationAttached {
    /// Number of revolute / continuous joints wired with motors.
    pub revolute_joints: usize,
    /// Number of prismatic joints wired with linear motors.
    pub prismatic_joints: usize,
    /// Number of fixed joints wired as rigid welds.
    pub fixed_joints: usize,
}

/// Attaches Rapier revolute joints and velocity motors for movable URDF joints.
///
/// [`RevoluteJointDesc`] and [`JointMotor`] are inserted on each actuated child link,
/// matching the diff-drive wheel convention used by `rne_physics_rapier`.
pub fn attach_urdf_articulation(
    world: &mut World,
    urdf: &UrdfRobot,
    spawned: &SpawnedUrdfRobot,
    config: UrdfArticulationConfig,
) -> Result<UrdfArticulationAttached, UrdfSpawnError> {
    let multibody_links = if config.multibody {
        multibody_link_names(urdf, spawned)
    } else {
        HashSet::new()
    };
    if config.multibody {
        for name in &multibody_links {
            if let Some(entity) = spawned.links.get(name) {
                world.entity_mut(*entity).insert(MultibodyLink);
            }
        }
    }
    if let Some(mut base_body) = world.get_mut::<RigidBody>(spawned.base_link) {
        base_body.body_type = config.base_body_type;
    }

    let mut revolute_joints = 0_usize;
    let mut prismatic_joints = 0_usize;
    let mut fixed_joints = 0_usize;
    for joint in &urdf.joints {
        if config.multibody && !multibody_links.contains(&joint.child) {
            continue;
        }
        let parent = *spawned
            .links
            .get(&joint.parent)
            .ok_or_else(|| UrdfSpawnError::UnknownLink(joint.parent.clone()))?;
        let child = *spawned
            .links
            .get(&joint.child)
            .ok_or_else(|| UrdfSpawnError::UnknownLink(joint.child.clone()))?;

        match joint.joint_type {
            UrdfJointType::Fixed => {
                ensure_dynamic_link(world, child);
                world.entity_mut(child).insert(FixedJointDesc {
                    parent,
                    anchor_parent_m: joint.origin_xyz,
                    anchor_child_m: Vec3::ZERO,
                    relative_rotation: rpy_to_quat(joint.origin_rpy),
                });
                fixed_joints += 1;
            }
            UrdfJointType::Revolute | UrdfJointType::Continuous => {
                ensure_dynamic_link(world, child);
                let (lower_rad, upper_rad) = revolute_limits_rad(joint);
                world.entity_mut(child).insert((
                    RevoluteJointDesc {
                        parent,
                        axis: normalize_axis(joint.axis),
                        anchor_parent_m: joint.origin_xyz,
                        anchor_child_m: Vec3::ZERO,
                        lower_rad,
                        upper_rad,
                    },
                    JointMotor::default(),
                ));
                revolute_joints += 1;
            }
            UrdfJointType::Prismatic => {
                ensure_dynamic_link(world, child);
                world.entity_mut(child).insert((
                    PrismaticJointDesc {
                        parent,
                        axis: normalize_axis(joint.axis),
                        anchor_parent_m: joint.origin_xyz,
                        anchor_child_m: Vec3::ZERO,
                    },
                    JointMotor::default(),
                ));
                prismatic_joints += 1;
            }
        }
    }

    Ok(UrdfArticulationAttached {
        revolute_joints,
        prismatic_joints,
        fixed_joints,
    })
}

fn multibody_link_names(urdf: &UrdfRobot, spawned: &SpawnedUrdfRobot) -> HashSet<String> {
    let mut names: HashSet<String> = urdf
        .joints
        .iter()
        .filter(|joint| joint.joint_type != UrdfJointType::Fixed)
        .flat_map(|joint| [joint.parent.clone(), joint.child.clone()])
        .collect();
    if let Some((root_name, _)) = spawned
        .links
        .iter()
        .find(|(_, entity)| **entity == spawned.base_link)
    {
        names.insert(root_name.clone());
    }

    loop {
        let parents: Vec<String> = urdf
            .joints
            .iter()
            .filter(|joint| names.contains(&joint.child) && !names.contains(&joint.parent))
            .map(|joint| joint.parent.clone())
            .collect();
        if parents.is_empty() {
            break;
        }
        names.extend(parents);
    }
    names
}

fn ensure_dynamic_link(world: &mut World, entity: Entity) {
    if world.get::<RigidBody>(entity).is_none() {
        return;
    }
    if let Some(mut body) = world.get_mut::<RigidBody>(entity) {
        if body.body_type != RigidBodyType::Fixed {
            body.body_type = RigidBodyType::Dynamic;
        }
    }
}

fn normalize_axis(axis: Vec3) -> Vec3 {
    if axis.length_squared() <= f64::EPSILON {
        Vec3::Y
    } else {
        axis.normalize()
    }
}

fn revolute_limits_rad(joint: &crate::schema::UrdfJoint) -> (Option<f64>, Option<f64>) {
    if joint.joint_type == UrdfJointType::Continuous {
        return (None, None);
    }
    joint
        .limit
        .map(|limit| (Some(limit.lower), Some(limit.upper)))
        .unwrap_or((None, None))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::parse_urdf;
    use crate::spawn::{spawn_urdf_robot_with_config, UrdfSpawnConfig};
    use rne_core::SimDuration;
    use rne_math::Hertz;
    use rne_physics::{
        Collider, FixedJointDesc, PhysicsBackend, PhysicsWorldDesc, PrismaticJointDesc,
    };
    use rne_physics_rapier::{step_physics, RapierBackend};
    use rne_robot::JointKind;
    use rne_world::{world_transform_of, Transform3};

    const FIXTURE: &str = include_str!("../tests/fixtures/mm_minimal_arm.urdf");

    fn spawn_arm_world() -> (World, SpawnedUrdfRobot) {
        let urdf = parse_urdf(FIXTURE).unwrap();
        let mut world = World::new();
        let spawned = spawn_urdf_robot_with_config(
            &mut world,
            &urdf,
            UrdfSpawnConfig {
                base_body_type: RigidBodyType::Fixed,
                ..UrdfSpawnConfig::default()
            },
        )
        .unwrap();
        attach_urdf_articulation(
            &mut world,
            &urdf,
            &spawned,
            UrdfArticulationConfig::default(),
        )
        .unwrap();
        world
            .entity_mut(spawned.base_link)
            .insert(Transform3::from_translation_rotation(
                rne_math::Vec3::new(0.0, 0.3, 0.0),
                rne_math::Quat::IDENTITY,
            ));
        (world, spawned)
    }

    const PRISMATIC_FIXTURE: &str = include_str!("../tests/fixtures/prismatic_slider.urdf");

    #[test]
    fn attach_articulation_wires_prismatic_motor() {
        let urdf = parse_urdf(PRISMATIC_FIXTURE).unwrap();
        let mut world = World::new();
        let spawned = spawn_urdf_robot_with_config(
            &mut world,
            &urdf,
            UrdfSpawnConfig {
                base_body_type: RigidBodyType::Fixed,
                ..UrdfSpawnConfig::default()
            },
        )
        .unwrap();
        let attached = attach_urdf_articulation(
            &mut world,
            &urdf,
            &spawned,
            UrdfArticulationConfig::default(),
        )
        .unwrap();

        assert_eq!(attached.revolute_joints, 0);
        assert_eq!(attached.prismatic_joints, 1);
        let slider = spawned.links["slider_link"];
        assert!(world.get::<PrismaticJointDesc>(slider).is_some());
        assert!(world.get::<JointMotor>(slider).is_some());
    }

    #[test]
    fn prismatic_motor_slides_child_along_axis() {
        let urdf = parse_urdf(PRISMATIC_FIXTURE).unwrap();
        let mut world = World::new();
        let spawned = spawn_urdf_robot_with_config(
            &mut world,
            &urdf,
            UrdfSpawnConfig {
                base_body_type: RigidBodyType::Fixed,
                ..UrdfSpawnConfig::default()
            },
        )
        .unwrap();
        attach_urdf_articulation(
            &mut world,
            &urdf,
            &spawned,
            UrdfArticulationConfig::default(),
        )
        .unwrap();
        world
            .entity_mut(spawned.base_link)
            .insert(Transform3::from_translation_rotation(
                rne_math::Vec3::new(0.0, 0.5, 0.0),
                rne_math::Quat::IDENTITY,
            ));

        let slider = spawned.links["slider_link"];
        let initial = world_transform_of(&world, slider).translation;

        // Linear motor command: 0.5 m/s along the joint's +X sliding axis.
        world.get_mut::<JointMotor>(slider).unwrap().velocity_rad_s = 0.5;

        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(PhysicsWorldDesc::default()).unwrap();
        let dt = SimDuration::from_hertz(Hertz::new(60.0));
        for _ in 0..120 {
            step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
        }

        let moved = world_transform_of(&world, slider).translation - initial;
        // The prismatic joint locks all but the sliding axis, so the child must
        // translate along +X and stay put on the other axes despite gravity.
        assert!(
            moved.x > 0.1,
            "slider should advance along +X under the linear motor, moved.x={}",
            moved.x
        );
        assert!(
            moved.y.abs() < 1e-3 && moved.z.abs() < 1e-3,
            "slider should not drift off the locked axes, moved={moved:?}"
        );
    }

    #[test]
    fn attach_articulation_wires_fixed_joint_as_weld() {
        let (world, spawned) = spawn_arm_world();
        // mm_minimal_arm.urdf attaches gripper_base_link via a fixed joint; it must be
        // wired as a rigid weld (FixedJointDesc) instead of left as a free body.
        let gripper_base = spawned.links["gripper_base_link"];
        assert!(world.get::<FixedJointDesc>(gripper_base).is_some());
    }

    #[test]
    fn attach_articulation_wires_revolute_motors() {
        let urdf = parse_urdf(FIXTURE).unwrap();
        let (world, spawned) = spawn_arm_world();
        let upper_arm = spawned.links["upper_arm_link"];
        let forearm = spawned.links["forearm_link"];

        assert!(world.get::<RevoluteJointDesc>(upper_arm).is_some());
        assert!(world.get::<RevoluteJointDesc>(forearm).is_some());
        assert!(world.get::<JointMotor>(upper_arm).is_some());
        assert_eq!(
            world
                .get::<rne_robot::Joint>(spawned.joints["shoulder_joint"])
                .unwrap()
                .kind,
            JointKind::Revolute
        );
        let _ = urdf;
    }

    #[test]
    fn shoulder_motor_moves_forearm() {
        let (mut world, spawned) = spawn_arm_world();
        let ground = rne_ecs::spawn_named(&mut world, "ground");
        world.entity_mut(ground).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider::cuboid(rne_math::Vec3::new(10.0, 0.05, 10.0)),
            Transform3::from_translation_rotation(
                rne_math::Vec3::new(0.0, -0.05, 0.0),
                rne_math::Quat::IDENTITY,
            ),
        ));

        let forearm = spawned.links["forearm_link"];
        let upper_arm = spawned.links["upper_arm_link"];
        let initial = world_transform_of(&world, forearm).translation;

        world
            .get_mut::<JointMotor>(upper_arm)
            .unwrap()
            .velocity_rad_s = 3.0;

        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(PhysicsWorldDesc::default()).unwrap();
        let dt = SimDuration::from_hertz(Hertz::new(60.0));

        for _ in 0..360 {
            step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
        }

        let displacement = (world_transform_of(&world, forearm).translation - initial).length();
        assert!(
            displacement > 0.025,
            "forearm should move under shoulder motor, displacement={displacement}"
        );
    }
}
