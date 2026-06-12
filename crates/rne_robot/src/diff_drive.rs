//! Differential drive robot spawning and kinematics metadata.

use crate::actuator::{ActuatorLimits, ActuatorTarget, ControlMode};
use crate::components::{Actuator, Joint, JointKind, JointLimits, Link, Robot, RobotId};
use bevy_ecs::prelude::Component;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, RigidBody, RigidBodyType};
use rne_world::Transform3;

/// Differential drive metadata attached to a robot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DifferentialDrive {
    /// Robot root entity.
    pub robot: Entity,
    /// Base link entity.
    pub base_link: Entity,
    /// Left wheel actuator entity.
    pub left_actuator: Entity,
    /// Right wheel actuator entity.
    pub right_actuator: Entity,
    /// Wheel radius in meters.
    pub wheel_radius_m: f64,
    /// Track width in meters.
    pub track_width_m: f64,
}

/// Configuration for spawning a differential drive robot.
#[derive(Clone, Debug, PartialEq)]
pub struct DiffDriveConfig {
    /// Robot model name.
    pub model_name: String,
    /// Initial base translation in meters.
    pub initial_translation_m: Vec3,
    /// Wheel radius in meters.
    pub wheel_radius_m: f64,
    /// Track width in meters.
    pub track_width_m: f64,
    /// Base link half extents in meters.
    pub base_half_extents_m: Vec3,
    /// Maximum wheel velocity in radians per second.
    pub max_wheel_velocity_rad_s: f64,
}

impl Default for DiffDriveConfig {
    fn default() -> Self {
        Self {
            model_name: "diff_drive".into(),
            initial_translation_m: Vec3::new(0.0, 0.25, 0.0),
            wheel_radius_m: 0.1,
            track_width_m: 0.45,
            base_half_extents_m: Vec3::new(0.25, 0.15, 0.2),
            max_wheel_velocity_rad_s: 10.0,
        }
    }
}

/// Entities created by [`spawn_diff_drive_robot`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiffDriveSpawned {
    /// Robot root entity.
    pub robot: Entity,
    /// Base link entity.
    pub base_link: Entity,
    /// Left wheel link entity.
    pub left_wheel: Entity,
    /// Right wheel link entity.
    pub right_wheel: Entity,
    /// Left wheel actuator entity.
    pub left_actuator: Entity,
    /// Right wheel actuator entity.
    pub right_actuator: Entity,
    /// Differential drive component entity (same as robot root).
    pub drive: DifferentialDrive,
}

/// Spawns a minimal differential drive robot into the ECS world.
pub fn spawn_diff_drive_robot(world: &mut World, config: &DiffDriveConfig) -> DiffDriveSpawned {
    let robot = spawn_named(world, &config.model_name);
    let base_link = spawn_named(world, "base_link");
    let left_wheel = spawn_named(world, "left_wheel");
    let right_wheel = spawn_named(world, "right_wheel");
    let left_actuator = spawn_named(world, "left_motor");
    let right_actuator = spawn_named(world, "right_motor");

    world.entity_mut(robot).insert(Robot {
        robot_id: RobotId::default(),
        model_name: config.model_name.clone(),
        base_link,
    });

    world.entity_mut(base_link).insert((
        Link {
            robot,
            name: "base_link".into(),
        },
        Transform3::from_translation_rotation(config.initial_translation_m, Quat::IDENTITY),
        RigidBody {
            body_type: RigidBodyType::Kinematic,
            mass_kg: 5.0,
            ..RigidBody::default()
        },
        Collider::cuboid(config.base_half_extents_m),
    ));

    let half_track = config.track_width_m * 0.5;
    let wheel_offset = Vec3::new(
        0.0,
        -config.base_half_extents_m.y + config.wheel_radius_m,
        0.0,
    );

    for (wheel, name, x_offset, actuator_entity) in [
        (left_wheel, "left_wheel", -half_track, left_actuator),
        (right_wheel, "right_wheel", half_track, right_actuator),
    ] {
        world.entity_mut(wheel).insert((
            Link {
                robot,
                name: name.into(),
            },
            Joint {
                robot,
                parent_link: base_link,
                child_link: wheel,
                kind: JointKind::Continuous,
                limits: JointLimits::default(),
                axis: Vec3::Y,
                position: 0.0,
                velocity: 0.0,
            },
            Transform3::from_translation_rotation(
                Vec3::new(x_offset, wheel_offset.y, 0.0),
                Quat::IDENTITY,
            ),
        ));

        world.entity_mut(actuator_entity).insert(Actuator {
            robot,
            joint: Some(wheel),
            name: format!("{name}_motor"),
            mode: ControlMode::Velocity,
            target: ActuatorTarget::default(),
            limits: ActuatorLimits {
                min_velocity_rad_s: -config.max_wheel_velocity_rad_s,
                max_velocity_rad_s: config.max_wheel_velocity_rad_s,
                ..ActuatorLimits::default()
            },
        });
    }

    let drive = DifferentialDrive {
        robot,
        base_link,
        left_actuator,
        right_actuator,
        wheel_radius_m: config.wheel_radius_m,
        track_width_m: config.track_width_m,
    };

    world.entity_mut(robot).insert(DiffDriveComponent(drive));

    DiffDriveSpawned {
        robot,
        base_link,
        left_wheel,
        right_wheel,
        left_actuator,
        right_actuator,
        drive,
    }
}

/// ECS component storing differential drive metadata on the robot root.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct DiffDriveComponent(pub DifferentialDrive);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spawn_diff_drive_robot_creates_links_and_actuators() {
        let mut world = World::new();
        let spawned = spawn_diff_drive_robot(&mut world, &DiffDriveConfig::default());

        assert!(world.get::<Robot>(spawned.robot).is_some());
        assert!(world.get::<Link>(spawned.base_link).is_some());
        assert!(world.get::<Actuator>(spawned.left_actuator).is_some());
        assert!(world.get::<Actuator>(spawned.right_actuator).is_some());
        assert!(world.get::<DiffDriveComponent>(spawned.robot).is_some());
    }
}
