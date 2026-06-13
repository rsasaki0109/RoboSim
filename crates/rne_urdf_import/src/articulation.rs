//! URDF joint wiring for Rapier impulse articulations.

use crate::schema::{UrdfJointType, UrdfRobot};
use crate::spawn::{SpawnedUrdfRobot, UrdfSpawnError};
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::{JointMotor, RevoluteJointDesc, RigidBody, RigidBodyType};

/// Configuration for [`attach_urdf_articulation`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UrdfArticulationConfig {
    /// Rigid-body type applied to the robot base link.
    pub base_body_type: RigidBodyType,
    /// Maximum motor force applied to each created revolute joint.
    pub motor_max_force: f32,
}

impl Default for UrdfArticulationConfig {
    fn default() -> Self {
        Self {
            base_body_type: RigidBodyType::Fixed,
            motor_max_force: 50.0,
        }
    }
}

/// Summary of joints wired into the physics backend.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UrdfArticulationAttached {
    /// Number of revolute / continuous joints wired with motors.
    pub revolute_joints: usize,
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
    if let Some(mut base_body) = world.get_mut::<RigidBody>(spawned.base_link) {
        base_body.body_type = config.base_body_type;
    }

    let mut revolute_joints = 0_usize;
    for joint in &urdf.joints {
        let parent = *spawned
            .links
            .get(&joint.parent)
            .ok_or_else(|| UrdfSpawnError::UnknownLink(joint.parent.clone()))?;
        let child = *spawned
            .links
            .get(&joint.child)
            .ok_or_else(|| UrdfSpawnError::UnknownLink(joint.child.clone()))?;

        match joint.joint_type {
            UrdfJointType::Fixed => {}
            UrdfJointType::Revolute | UrdfJointType::Continuous => {
                ensure_dynamic_link(world, child);
                world.entity_mut(child).insert((
                    RevoluteJointDesc {
                        parent,
                        axis: normalize_axis(joint.axis),
                        anchor_parent_m: joint.origin_xyz,
                        anchor_child_m: Vec3::ZERO,
                    },
                    JointMotor::default(),
                ));
                revolute_joints += 1;
            }
            UrdfJointType::Prismatic => {
                return Err(UrdfSpawnError::InvalidGraph(format!(
                    "prismatic joint `{}` is not supported yet",
                    joint.name
                )));
            }
        }
    }

    Ok(UrdfArticulationAttached { revolute_joints })
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::parse_urdf;
    use crate::spawn::{spawn_urdf_robot_with_config, UrdfSpawnConfig};
    use rne_core::SimDuration;
    use rne_math::Hertz;
    use rne_physics::{Collider, PhysicsBackend, PhysicsWorldDesc};
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

        for _ in 0..240 {
            step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
        }

        let displacement = (world_transform_of(&world, forearm).translation - initial).length();
        assert!(
            displacement > 0.03,
            "forearm should move under shoulder motor, displacement={displacement}"
        );
    }
}
